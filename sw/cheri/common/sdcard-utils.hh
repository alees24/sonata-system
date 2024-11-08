/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>
#include <platform-spi.hh>
#include <platform-gpio.hh>

#include "console.hh"

using namespace CHERI;

typedef CHERI::Capability<volatile SonataSpi> &SpiRef;
typedef CHERI::Capability<volatile SonataGPIO> &GpioRef;

class SdCard {
 private:
  // Access to SPI controller.
  SpiRef spi;
  // Access to GPIO block (required for SD card detection).
  GpioRef gpio;
  // Chip select (single bit set).
  uint32_t cs;
  // SD card `detect` pin (single bit set).
  uint32_t det;

  // Access to diagnostic logging.
  Log *log;

  // TODO: I seemingly cannot persuade the Integral card to use a non-512 byte block length.
  #define BLOCK_LEN 0x200u

  static constexpr uint8_t ones[] = {0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu};

 public:
  SdCard(SpiRef spi_, GpioRef gpio_, unsigned cs_ = 1u, unsigned det_ = 16u, Log *log_ = nullptr)
      : spi(spi_), gpio(gpio_), cs(1u << cs_), det(1u << det_), log(log_) {}

  // SD command codes.
  enum {
    CMD_GO_IDLE_STATE       = 0,
    CMD1                    = 1,
    CMD_SEND_IF_COND        = 8,
    CMD_SEND_CSD            = 9,
    CMD_SEND_CID            = 10,
    CMD_STOP_TRANSMISSION   = 12,
    CMD_SET_BLOCKLEN        = 16,
    CMD_READ_SINGLE_BLOCK   = 17,
    CMD_READ_MULTIPLE_BLOCK = 18,
    ACMD41                  = 41,
    CMD_APP_CMD             = 55,
    CMD_READ_OCR            = 58,
    CMD_CRC_ON_OFF          = 59,
  };

  // Indicates whether there is an SD card present in the slot.
  bool present() const { return !(gpio->input & det); }

  void select_card(bool enable) { spi->cs = enable ? (spi->cs & ~cs) : (spi->cs | cs); }

  // Initialise the SD card ready for use.
  bool init() {
    // Every card tried seems to be more than capable of keeping up with 20Mbps.
    constexpr unsigned kSpiSpeed = 0u;
    spi->init(false, false, true, kSpiSpeed);

    // Apparently we're required to send at least 74 SD CLK cycles with
    // the device _not_ selected before talking to it.
    spi->blocking_write(ones, 10);
    spi->wait_idle();

    select_card(true);

    // Note that this is a very stripped-down card initialisation sequence
    // that assumes SDHC version 2, so use a more recent microSD card.

    do {
      send_command(CMD_GO_IDLE_STATE, 0u, 0x95u);
    } while (0x01 != get_response());

    send_command(CMD_SEND_IF_COND, 0x1aau, 0x87u);
    get_response5();

    // Read supported voltage range of the card.
    send_command(CMD_READ_OCR, 0, 0xffu);
    get_response5();

    do {
      send_command(CMD_APP_CMD, 0, 0xffu);
      get_response();
      send_command(ACMD41, 1u << 30, 0xffu);
    } while (1 & get_response());

    if (log) {
      log->println("Setting block length to {}", BLOCK_LEN);
    }

    // Read card capacity information.
    send_command(CMD_READ_OCR, 0, 0xffu);
    get_response5();

    send_command(CMD_SET_BLOCKLEN, BLOCK_LEN, 0xffu);
    uint8_t rd = get_response();
    if (log) {
      log->println("Response: {:#04x}", rd);
    }
    select_card(false);

    return true;
  }

  // Read Card Specific Data.
  bool read_csd(uint8_t *buf, size_t len) {
    uint8_t crc16[2];

    select_card(true);

    send_command(CMD_SEND_CSD, 0, 0xffu);
    while (get_response() != 0xfeu);

    spi->blocking_read(buf, len);
    spi->blocking_read(crc16, sizeof(crc16));

    select_card(false);

    return true;
  }

  // Read Card Identification.
  bool read_cid(uint8_t *buf, size_t len) {
    uint8_t crc16[2];

    select_card(true);

    send_command(CMD_SEND_CID, 0, 0xffu);
    while (get_response() != 0xfeu);

    spi->blocking_read(buf, len);
    spi->blocking_read(crc16, sizeof(crc16));

    select_card(false);

    return true;
  }

  // Read a number of contiguous blocks from the SD card.
  bool read_blocks(uint32_t block, uint8_t *buf, size_t num_blocks = 1u, bool blocking = true) {
    // TODO: Have not yet been able to perform multi-block reads successfully.
    const bool multi = false;  // num_blocks > 1u;

    select_card(true);

    for (size_t blk = 0u; blk < num_blocks; blk++) {
      uint8_t crc16[2];
      if (log) {
        log->println("Reading block {}", block + blk);
      }

      if (multi) {
        // Is this the first block of the read request?
        if (!blk) {
          send_command(SdCard::CMD_READ_MULTIPLE_BLOCK, block, 0xffu);
          while (get_response() != 0xfeu);
        }
      } else {
        send_command(SdCard::CMD_READ_SINGLE_BLOCK, block + blk, 0xffu);
        while (get_response() != 0xfeu);
      }

      spi->blocking_read(&buf[blk * BLOCK_LEN], BLOCK_LEN);
      spi->blocking_read(crc16, sizeof(crc16));

      if (log) {
        log->println("Read block CRC {:#06x}", (crc16[0] << 8) | crc16[1]);
      }
    }

    if (multi) {
      uint8_t dummy;
      send_command(SdCard::CMD_STOP_TRANSMISSION, 0u, 0xffu);
      spi->blocking_read(&dummy, sizeof(dummy));
      (void)get_response();
    }

    select_card(false);
    return true;
  }

  // TODO: Calculate CRC7 for command.
  void send_command(uint8_t cmdCode, uint32_t arg, uint8_t crc7) {
    uint8_t cmd[6];
    if (log) {
      log->println("Sending command {:#04x}", cmdCode);
    }

    // Apparently we need to clock 8 times before sending the command.
    uint8_t dummy = 0xffu;
    spi->blocking_write(&dummy, 1u);

    cmd[0] = 0x40u | cmdCode;
    cmd[1] = (uint8_t)(arg >> 24);
    cmd[2] = (uint8_t)(arg >> 16);
    cmd[3] = (uint8_t)(arg >> 8);
    cmd[4] = (uint8_t)(arg >> 0);
    cmd[5] = crc7;

    spi->blocking_write(cmd, sizeof(cmd));
  }

  uint8_t get_response() {
    spi->wait_idle();
    while (true) {
      uint8_t rd1;

      spi->transmitFifo = 0xffu;
      spi->control      = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
      spi->start        = 1u;
      spi->wait_idle();
      while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {
      }
      rd1 = static_cast<uint8_t>(spi->receiveFifo);

      switch (rd1) {
        case 0x00u:
          // write_str(uart, ".");
          break;
        case 0x01u:
          // write_str(uart, "\r\n");
          break;
        default:
          if (rd1 != 0xffu && false) {
            //            write_str(uart, "Response ");
            //            write_hex8b(uart, rd1);
            //            write_str(uart, "\r\n");
          }
          break;
      }

      if (rd1 != 0xffu) {
        return rd1;
      }
    }
  }

  uint8_t get_nb_response1() {
    uint8_t rd1;

    spi->wait_idle();
    spi->transmitFifo = 0xffu;
    spi->control      = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
    spi->start        = 1u;
    spi->wait_idle();
    while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {
    }
    rd1 = static_cast<uint8_t>(spi->receiveFifo);
    return rd1;
  }

  void get_response5() {
    (void)get_response();

    for (int r = 0; r < 4; ++r) {
      volatile uint8_t rd2;
      spi->transmitFifo = 0xffu;
      spi->control      = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
      spi->start        = 1u;
      spi->wait_idle();
      while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {
      }
      rd2 = static_cast<uint8_t>(spi->receiveFifo);
    }
  }

  // Utility function that converts Cylinder, Head, Sector (CHS) addressing into Logical Block Addressing
  // (LBA), according to the specified disk geometry.
  static uint32_t chs_to_lba(uint16_t c, uint8_t h, uint8_t s, uint8_t nheads, uint8_t nsecs) {
    // Notes: cylinder and head are zero-based but sector number is 1-based (0 is invalid).
    // CHS-addressed drives were limited to 255 heads and 63 sectors.
    if (h >= nheads || !s || s > nsecs) {
      return UINT32_MAX;
    }
    return ((c * nheads + h) * nsecs) + (s - 1);
  }
};
