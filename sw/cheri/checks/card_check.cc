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
#include "../common/sdcard-utils.hh"
#include "../common/sonata-devices.hh"
#include "../common/uart-utils.hh"

#include "lcd_check.hh"

using namespace CHERI;

#define MAX_BLOCKS 0x10

static uint8_t dataBuffer[MAX_BLOCKS * BLOCK_LEN];

// Read Cylinder, Head and Sector ('CHS address').
static inline void read_chs(uint16_t& c, uint8_t& h, uint8_t& s, const uint8_t *p) {
  h = p[0];
  c = ((p[1] << 2) & 0x300u) | p[2];
  s = (p[1] & 0x3fu);
}

// Read 32-bit Little Endian word.
static inline uint32_t read32le(const uint8_t *p) {
  return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void read_blocks(SdCard &sd, Capability<volatile SonataSpi> &spi, uint32_t block, uint8_t *buf,
                        size_t num_blocks, Capability<volatile OpenTitanUart> &uart,
                        Log &log) {
  sd.select_card(true);

  if (num_blocks > 1u) {
    sd.send_command(SdCard::CMD18, block, 0xffu, uart);
  } else {
    sd.send_command(SdCard::CMD17, block, 0xffu, uart);
  }

  while (sd.get_response(uart) != 0xfeu);

  for (size_t blk = 0u; blk < num_blocks; blk++) {
    uint8_t crc16[2];

    spi->blocking_read(&buf[blk * BLOCK_LEN], BLOCK_LEN);
    spi->blocking_read(crc16, sizeof(crc16));

    if (true) {
      log.println("Read block CRC {:#04x}{:02x}", crc16[1], crc16[0]);
    }
  }

  if (num_blocks > 1u) {
    uint8_t dummy;
    sd.send_command(SdCard::CMD12, 0u, 0xffu, uart);
    spi->blocking_read(&dummy, sizeof(dummy));
    (void)sd.get_response(uart);
  }

  sd.select_card(false);
}

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

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  auto uart0 = uart_ptr(root);
  uart0->init(BAUD_RATE * 3/4);
  WriteUart uartWr{uart0};
  Log log(uartWr);

  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS + 0x3000;
  spi.bounds()                       = SPI_BOUNDS;

  Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                       = GPIO_ADDRESS;
  gpio.bounds()                        = GPIO_BOUNDS;

  // TODO: What happens here when spi1 is not initialised, but instead 'spi' is...
  // we appear to have a NULL dereference (data access to 0xc in sim, but it just loops
  // and doesn't fail)... display_init-> spi->init appeared to hang but no LEDs.

  log.println("Card check");

  SdCard sd(spi, gpio);

  // Wait until a card is detected.
  while (!sd.present()) {
    log.println("Please insert a microSD card into the slot...");
    while (!sd.present());
  }

  // TODO: Set the pinmux so that we can access the card slot.
  typedef struct {
    uint32_t reg;
  } pinmux_t;

  Capability<volatile pinmux_t> pinmux = root.cast<volatile pinmux_t>();
  pinmux.address() = 0x80005808;
  pinmux.bounds()  = 4u;
  ((volatile uint8_t*)&pinmux->reg)[0] = 8;

  sd.init(uart);

  // Report the first bit of each block at the start of the card.
  const size_t numBlocks = 4u;
  const size_t blkBytes = BLOCK_LEN;

  for (uint32_t blk = 0u; blk < numBlocks; ++blk) {
    log.println("Reading block {}", blk);

    read_blocks(sd, spi, blk, &dataBuffer[blk * BLOCK_LEN], 1u, uart, log);
    // Output the first bit of data...
    dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
  }

  if (dataBuffer[0x1fe] == 0x55 && dataBuffer[0x1ff] == 0xaa) {
    for (unsigned part = 0u; part < 1u; part++) {
      const unsigned partDesc = 0x1be + (part << 4);
      uint8_t part_type = dataBuffer[partDesc + 4];
      uint32_t lba_start = read32le(&dataBuffer[partDesc + 8]);
      uint32_t num_secs = read32le(&dataBuffer[partDesc + 12]);
      uint16_t start_c, end_c;
      uint8_t start_h, end_h;
      uint8_t start_s, end_s;
      read_chs(start_c, start_h, start_s, &dataBuffer[partDesc + 1]);
      read_chs(end_c, end_h, end_s, &dataBuffer[partDesc + 5]);
      log.println("Partition {} : type {} : start C {} H {} S {} : end C {} H {} S {}",
                  part, part_type, start_c, start_h, start_s, end_c, end_h, end_s);
      log.println("   LBA start: {:#010x} sectors: {:#010x}", lba_start, num_secs);
    }

    const uint16_t nheads = 255u;
    const uint16_t nsecs = 63u;

    uint32_t blk_offset = 0x8000u;
    log.println("Expecting EBR at block {}", blk_offset);

    for (uint32_t blk = 0u; blk < 1u; blk++) {
      log.println("Reading block {}", blk_offset + blk);
      read_blocks(sd, spi, blk_offset + blk, &dataBuffer[blk * BLOCK_LEN], 1u, uart, log);
      // Output the first bit of data...
      dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
    }

    uint32_t clusterHeapOffset = read32le(&dataBuffer[88]);
    uint32_t rootCluster = read32le(&dataBuffer[96]);
    uint8_t bytesPerSectorShift = dataBuffer[108];
    uint8_t secsPerClusterShift = dataBuffer[109];

    uint32_t rootStart = ((rootCluster - 2) << secsPerClusterShift << bytesPerSectorShift) / BLOCK_LEN;
    rootStart += clusterHeapOffset;

    log.println("Cluster heap offset {} Root cluster {} log2(bytes/sec) {} log2(secs/cluster) {}",
                clusterHeapOffset, rootCluster, bytesPerSectorShift, secsPerClusterShift);

    for (int off = 0; off < 4u * BLOCK_LEN; off++) dataBuffer[off] = 0xbd;

    for (uint32_t blk = 0u; blk < 4u; blk++) {
      log.println("Reading block {}", blk_offset + rootStart + blk);
#if 0
      read_blocks(sd, spi, blk_offset + rootStart + blk, &dataBuffer[blk * BLOCK_LEN], 1u, uart, log);
#else
      if (!blk) read_blocks(sd, spi, blk_offset + rootStart, dataBuffer, 4u, uart, log);
#endif
      // Output the first bit of data...
      dump_data(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
    }

  } else {
    log.println("No valid Master Boot Record found (signature not detected)");
  }

  while (1) asm(" ");
}
