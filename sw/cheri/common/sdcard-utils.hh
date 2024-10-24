/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>
#include <platform-spi.hh>
#include <platform-gpio.hh>

#include "uart-utils.hh"

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

// TODO: I seemingly cannot persuade the Integral card to use a non-512 byte block length.
#if 1
#define BLOCK_LEN 0x200u
#else
#define BLOCK_LEN 0x400u
#endif

  static constexpr uint8_t ones[] = {
    0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu
  };

 public:
  SdCard(SpiRef spi_, GpioRef gpio_, unsigned cs_ = 3u, unsigned det_ = 17u) : spi(spi_), gpio(gpio_), cs(1u << cs_), det(1u << det_) {}

  // SD command codes.
  enum {
    CMD0 = 0,
    CMD1 = 1,
    CMD8 = 8,
    CMD12 = 12,
    CMD16 = 16,
    CMD17 = 17,
    CMD18 = 18,
    ACMD41 = 41,
    CMD55 = 55,
    CMD58 = 58,
  };

  // Indicates whether there is an SD card present in the slot.
  bool present() const { return !(gpio->input & det); }

  void select_card(bool enable) { spi->cs = enable ? (spi->cs & ~cs) : (spi->cs | cs); }

  // Initialise the SD card ready for use.
  bool init(Capability<volatile OpenTitanUart> &uart) {

  constexpr unsigned kSpiSpeed = 0u;
    spi->init(false, false, true, kSpiSpeed);

    // Apparently we're required to send at least 74 SD CLK cycles with
    // the device _not_ selected before talking to it.
    spi->blocking_write(ones, 10);
    spi->wait_idle();

    select_card(true);

    // Note that this is a very stripped-down card initialisation sequence
    // that assumes SDHC version 2, so use a more recent microSD card.

    // CMD0
    do {
      send_command(CMD0, 0u, 0x95u, uart);
    } while (0x01 != get_response(uart));

    send_command(CMD8, 0x1aau, 0x87u, uart);
    get_response5(uart);

    do {
      send_command(CMD55, 0, 0xffu, uart);
      get_response(uart);
      send_command(ACMD41, 1u << 30, 0xffu, uart);
    } while (1 & get_response(uart));

    send_command(CMD58, 0, 0xffu, uart);
    get_response5(uart);

//    write_str(uart, "Setting block length\r\n");

    send_command(CMD16, BLOCK_LEN, 0xffu, uart);
    uint8_t rd = get_response(uart);
//    if (true) {
//      write_str(uart, "Response: ");
//      write_hex8b(uart, rd);
//      write_str(uart, "\r\n");
//    }

    select_card(false);

    return true;
  }

  void send_command(uint8_t cmdCode, uint32_t arg, uint8_t crc, Capability<volatile OpenTitanUart> &uart) {
    uint8_t cmd[6];

//  write_str(uart, "Sending ");
//  write_hex8b(uart, cmdCode);
//  write_str(uart, "\r\n");

    // Apparently we need to clock 8 times before sending the command.
    uint8_t dummy = 0xffu;
    spi->blocking_write(&dummy, 1u);

    cmd[0] = 0x40u | cmdCode;
    cmd[1] = (uint8_t)(arg >> 24);
    cmd[2] = (uint8_t)(arg >> 16);
    cmd[3] = (uint8_t)(arg >> 8);
    cmd[4] = (uint8_t)(arg >> 0);
    cmd[5] = crc;

    spi->blocking_write(cmd, sizeof(cmd));
  }

  uint8_t get_response(Capability<volatile OpenTitanUart> &uart) {
    spi->wait_idle();
    while (true) {
      uint8_t rd1;

      spi->transmitFifo = 0xffu;
      spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
      spi->start = 1u;
      spi->wait_idle();
		  while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {}
		  rd1 = static_cast<uint8_t>(spi->receiveFifo);

      switch (rd1) {
        case 0x00u:
          //write_str(uart, ".");
          break;
        case 0x01u:
          //write_str(uart, "\r\n");
          break;
        default:
          if (rd1 != 0xffu && false) {
            write_str(uart, "Response ");
            write_hex8b(uart, rd1);
            write_str(uart, "\r\n");
          }
          break;
      }

      if (rd1 != 0xffu) {
        return rd1;
      }
    }
  }

  uint8_t get_nb_response1(Capability<volatile OpenTitanUart> &uart) {
    uint8_t rd1;

    spi->wait_idle();
    spi->transmitFifo = 0xffu;
    spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
    spi->start = 1u;
    spi->wait_idle();
	  while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {}
	  rd1 = static_cast<uint8_t>(spi->receiveFifo);
    return rd1;
  }

  void get_response5(Capability<volatile OpenTitanUart> &uart) {
    (void)get_response(uart);

    for (int r = 0; r < 4; ++r) {
      volatile uint8_t rd2;
      spi->transmitFifo = 0xffu;
      spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
      spi->start = 1u;
      spi->wait_idle();
	    while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {}
	    rd2 = static_cast<uint8_t>(spi->receiveFifo);
    }
  }

//  read_blocks();

};
