/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <assert.h>
#include <ctype.h>
#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>
#include <platform-spi.hh>

#include "../common/console.hh"
#include "../common/filesys-utils.hh"
#include "../common/platform-pinmux.hh"
#include "../common/sdcard-utils.hh"
#include "../common/sonata-devices.hh"
#include "../common/uart-utils.hh"

#include "../tests/test_runner.hh"

#include "lcd_check.hh"

using namespace CHERI;

#define MAX_BLOCKS 0x10

static uint8_t dataBuffer[MAX_BLOCKS * BLOCK_LEN];

// Dump out a sequence of bytes as hexadecimal and ASCII text.
static void dump_data(const uint8_t *buf, size_t blkBytes, Log& log) {
  for (size_t off = 0u; off < blkBytes; ++off) {
    log.print("{:02x}", buf[off]);
    if ((off & 0xfu) == 0xfu) {
      for (size_t aoff = (off & ~0xfu); aoff < off; aoff++) {
        char text[2];
        text[0] = buf[aoff];
        if (!isprint(text[0])) text[0] = '.';
        text[1] = '\0';
        log.print(text);
      }
      log.println("");
    } else {
      log.print(" ");
    }
  }
}

// Read and report the properties of the SD card itself (CSD and CID).
static unsigned read_card_properties(SdCard& sd, Log& log) {
  unsigned failures = 0u;
  uint8_t buf[16];
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("Reading Card Specific Data (CSD)");
  if (sd.read_csd(buf)) {
    dump_data(buf, sizeof(buf), log);
  } else {   
    failures++;
  }
  log.println("... Done");

  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("Reading Card Identification (CID)");
  if (sd.read_cid(buf)) {
    dump_data(buf, sizeof(buf), log);
  } else {   
    failures++;
  }
  log.println("... Done");
  return failures;
}

/**
 * C++ entry point for the loader. This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  auto uart0 = uart_ptr(root);
  uart0->init(BAUD_RATE);

  WriteUart uartWr{uart0};
  Log log(uartWr);

  // The SPI controller talkes to the microSD card in SPI mode.
  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS + 2 * SPI_RANGE;
  spi.bounds()                       = SPI_BOUNDS;

  // We need to use the pinmux to select the microSD card for SPI controller 2 reads (CIPO),
  // as well as preventing outbound traffic to the microSD card also reaching the application
  // flash (for safety; it _should_ ignore traffic not accompanied by Chip Select assertion).
  Capability<volatile uint8_t> pinmux = root.cast<volatile uint8_t>();
  pinmux.address()                    = PINMUX_ADDRESS;
  pinmux.bounds()                     = PINMUX_BOUNDS;
  SonataPinmux Pinmux                 = SonataPinmux(pinmux);
  // Suppress traffic to the application flash.
  constexpr uint8_t Disabled = 0;
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_cs, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_clk, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_d0, Disabled);
  // Direct SPI controller 2 to drive the microSD pins.
  constexpr uint8_t EnableFirstInput = 1;
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_dat3, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_clk, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_cmd, EnableFirstInput);
  // Select microSD CIPO as SPI controller input.
  constexpr uint8_t PmuxSpi0CipoToSdDat0 = 2;
  Pinmux.block_input_select(SonataPinmux::BlockInput::spi_0_cipo, PmuxSpi0CipoToSdDat0);

  // We need to use the GPIO to detect card presence.
  Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                       = GPIO_ADDRESS;
  gpio.bounds()                        = GPIO_BOUNDS;

  log.println("Card check");

  // microSD card is on Chip Select 1 (0 goes to the application flash).
  constexpr unsigned csBit = 1u;
  // microSD card detection bit is on input 16.
  constexpr unsigned detBit = 16u;

  SdCard sd(spi, gpio, csBit, detBit, &log);

  // Wait until a card is detected.
  while (!sd.present()) {
    log.println("Please insert a microSD card into the slot...");
    while (!sd.present());
  }

  sd.init();

  unsigned failures = 0u;

//  failures += read_card_properties(sd, log);

  // Report the first bit of each block at the start of the card.
  const size_t numBlocks = 4u;
  const size_t blkBytes = BLOCK_LEN;

  for (uint32_t blk = 0u; blk < numBlocks; ++blk) {
    log.println("Reading block {}", blk);

    sd.read_blocks( blk, &dataBuffer[blk * BLOCK_LEN], 1u);
    // Output the first bit of data...
    dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
  }

  if (dataBuffer[0x1fe] == 0x55 && dataBuffer[0x1ff] == 0xaa) {
    uint32_t blk_offset;
    bool use_lba = true;
    bool exFat = true;
    bool found = false;

    for (unsigned part = 0u; part < 1u; part++) {
      const unsigned partDesc = 0x1be + (part << 4);
      uint8_t part_type = dataBuffer[partDesc + 4];
      uint32_t lba_start = fileSystemUtils::read32le(&dataBuffer[partDesc + 8]);
      uint32_t num_secs = fileSystemUtils::read32le(&dataBuffer[partDesc + 12]);
      uint16_t start_c, end_c;
      uint8_t start_h, end_h;
      uint8_t start_s, end_s;
      fileSystemUtils::read_chs(start_c, start_h, start_s, &dataBuffer[partDesc + 1]);
      fileSystemUtils::read_chs(end_c, end_h, end_s, &dataBuffer[partDesc + 5]);
      log.println("Partition {} : type {} : start C {} H {} S {} : end C {} H {} S {}",
                  part, part_type, start_c, start_h, start_s, end_c, end_h, end_s);
      log.println("   LBA start: {:#010x} sectors: {:#010x}", lba_start, num_secs);

      switch (part_type) {
        // FAT32 a viable alternative.
        case 0x0B: use_lba = false;
        case 0x0C: exFat = false;
        // exFAT preferred.
        case 0x07: {
          const uint16_t nheads = 255u;
          const uint16_t nsecs = 63u;
          if (use_lba) {
            blk_offset = lba_start;
          } else {
            blk_offset = sd.chs_to_lba(start_c, start_h, start_s, nheads, nsecs);
          }
          log.println("Expecting EBR at block {:#x}", blk_offset);
          found = true;
        }
        break;
        default:
          log.println("Not a suitable partition");
          break;
      }
    }

    if (!found) {
      log.println("Unable to locate a suitable partition");
      check_result(log, false);
    }

    // Read the EBR at the start of the partition.
    for (uint32_t blk = 0u; blk < 1u; blk++) {
      log.println("Reading block {}", blk_offset + blk);
      sd.read_blocks(blk_offset + blk, &dataBuffer[blk * BLOCK_LEN], 1u);
      // Output the first bit of data...
      dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
    }

    uint8_t bytesPerSectorShift;
    uint8_t secsPerClusterShift;
    uint32_t clusterHeapOffset;
    uint32_t rootCluster;
    uint32_t rootStart;

    if (exFat) {
      clusterHeapOffset = fileSystemUtils::read32le(&dataBuffer[88]);
      rootCluster = fileSystemUtils::read32le(&dataBuffer[96]);
      bytesPerSectorShift = dataBuffer[108];
      secsPerClusterShift = dataBuffer[109];
    } else {
      rootCluster = fileSystemUtils::read32le(&dataBuffer[0x2c]);
      uint32_t secsPerFAT = fileSystemUtils::read32le(&dataBuffer[0x24]);
      uint16_t bytesPerSector = fileSystemUtils::read16le(&dataBuffer[0x0b]);
      log.println("FAT32 secs per FAT {} bytes per sector {}", secsPerFAT, bytesPerSector);
      bytesPerSectorShift = 0u;
      while (bytesPerSector > 1u) {
        bytesPerSector >>= 1;
        bytesPerSectorShift++;
      }
      uint8_t secsPerCluster = dataBuffer[0xd];
      secsPerClusterShift = 0u;
      while (secsPerCluster > 1u) {
        secsPerCluster >>= 1;
        secsPerClusterShift++;
      }

      clusterHeapOffset = ((3 + (2 * secsPerFAT)) << bytesPerSectorShift) / BLOCK_LEN;
    }

    rootStart = ((rootCluster - 2) << secsPerClusterShift << bytesPerSectorShift) / BLOCK_LEN;
    rootStart += clusterHeapOffset;

    log.println("Cluster heap offset {} Root cluster {} log2(bytes/sec) {} log2(secs/cluster) {}",
                clusterHeapOffset, rootCluster, bytesPerSectorShift, secsPerClusterShift);

    // Populate the buffer with invalid data that cannot be misinterpreted.
    for (int off = 0; off < 4u * BLOCK_LEN; off++) {
      dataBuffer[off] = 0xbd;
    }

    sd.read_blocks(blk_offset + rootStart, dataBuffer, 4u);

    for (uint32_t blk = 0u; blk < 4u; blk++) {
      // Output the first bit of data...
      dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
    }

  } else {
    log.println("No valid Master Boot Record found (signature not detected)");
    check_result(log, false);
  }

  while (1) asm(" ");
}
