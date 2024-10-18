/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <assert.h>
#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>
#include <platform-spi.hh>

#include "../common/uart-utils.hh"

#include "lcd_check.hh"

using namespace CHERI;

#define BLOCK_LEN 0x200u

enum {
  CMD0 = 0,
  CMD1 = 1,
  CMD8 = 8,
  CMD16 = 16,
  CMD17 = 17,
  ACMD41 = 41,
  CMD55 = 55,
  CMD58 = 58,
};

static uint8_t dataBuffer[164 * BLOCK_LEN];

static void send_command(Capability<volatile SonataSpi> &spi, uint8_t cmdCode, uint32_t arg, uint8_t crc,
                         Capability<volatile OpenTitanUart> &uart) {
  static uint8_t cmd[6];

//  write_str(uart, "Sending ");
//  write_hex8b(uart, cmdCode);
//  write_str(uart, "\r\n");

// TODO: Do we need this?
  uint8_t dummy = 0xffu;
  spi->blocking_write(&dummy, 1u);
  spi->wait_idle();

  cmd[0] = 0x40u | cmdCode;
  cmd[1] = (uint8_t)(arg >> 24);
  cmd[2] = (uint8_t)(arg >> 16);
  cmd[3] = (uint8_t)(arg >> 8);
  cmd[4] = (uint8_t)(arg >> 0);
  cmd[5] = crc;

  spi->blocking_write(cmd, sizeof(cmd));
}

static uint8_t get_response(Capability<volatile SonataSpi> &spi,
                            Capability<volatile OpenTitanUart> &uart) {
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
//    spi->blocking_read(&rd1, sizeof(rd1));
    if (rd1 != 0xffu) {
//      uint8_t dummy;
      // Supposedly the card requires a further 8 clock cycles.
//      spi->blocking_read(&dummy, 1u);
      return rd1;
    }
  }
}

static void get_response5(Capability<volatile SonataSpi> &spi,
                          Capability<volatile OpenTitanUart> &uart) {
  (void)get_response(spi, uart);

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

static void read_block(Capability<volatile SonataSpi> &spi, uint32_t block, uint8_t *buf, size_t n,
                       Capability<volatile OpenTitanUart> &uart) {
  uint8_t crc16[2];

  spi->cs = spi->cs & ~8u;

  send_command(spi, CMD17, block, 0xffu, uart);
  while (get_response(spi, uart) != 0xfeu);
  spi->blocking_read(buf, n);
  spi->blocking_read(crc16, sizeof(crc16));

  spi->cs = spi->cs | 8u;

  if (false) {
    write_str(uart, "Read block CRC ");
    write_hex8b(uart, crc16[0]);
    write_hex8b(uart, crc16[1]);
    write_str(uart, "\r\n");
  }
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
[[noreturn]] extern "C" void entry_point(void *rwRoot) {
  static const uint8_t ones[] = {
    0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu, 0xffu, 0xffu,
    0xffu, 0xffu
  };

  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS + 0x3000;
  spi.bounds()                       = SPI_BOUNDS;

  Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                       = GPIO_ADDRESS;
  gpio.bounds()                        = GPIO_BOUNDS;

  Capability<volatile SonataSpi> spi1 = root.cast<volatile SonataSpi>();
  spi1.address()                      = SPI_ADDRESS + 0x1000;
  spi1.bounds()                       = SPI_BOUNDS;
  // TODO: What happens here when spi1 is not initialised, but instead 'spi' is...
  // we appear to have a NULL dereference (data access to 0xc in sim, but it just loops
  // and doesn't fail)... display_init-> spi->init appeared to hang but no LEDs.

  uart->init(BAUD_RATE);
  write_str(uart, "Card check\r\n");

  display_init(spi1, gpio, uart);

  // TODO: Detect the presence of a card.

  // TODO: Set the pinmux so that we can access the card slot.
  typedef struct {
    uint32_t reg;
  } pinmux_t;
  Capability<volatile pinmux_t> pinmux = root.cast<volatile pinmux_t>();
  pinmux.address() = 0x80005808;
  pinmux.bounds()  = 4u;
  ((volatile uint8_t*)&pinmux->reg)[0] = 8;

  constexpr unsigned kSpiSpeed = 0u;
  spi->init(false, false, true, kSpiSpeed);

  // Apparently we're required to send at least 74 SD CLK cycles with
  // the device _not_ selected before talking to it.
  spi->blocking_write(ones, 10);

  spi->wait_idle();
  spi->cs = spi->cs & ~8u;

  // Note that this is a very stripped-down card initialisation sequence
  // that assumes SDHC version 2, so use a more recent microSD card.

  // CMD0
  do {
    send_command(spi, CMD0, 0u, 0x95u, uart);
  } while (0x01 != get_response(spi, uart));

  send_command(spi, CMD8, 0x1aau, 0x87u, uart);
  get_response5(spi, uart);

  do {
    send_command(spi, CMD55, 0, 0xffu, uart);
    get_response(spi, uart);
    send_command(spi, ACMD41, 1u << 30, 0xffu, uart);
  } while (1 & get_response(spi, uart));

  send_command(spi, CMD58, 0, 0xffu, uart);
  get_response5(spi, uart);

  write_str(uart, "Setting block length\r\n");

  send_command(spi, CMD16, BLOCK_LEN, 0xffu, uart);
  get_response(spi, uart);

  spi->cs = spi->cs | 8u;

  if (false) {
    for (uint32_t blk = 0u; blk < 82; ++blk) {
      write_str(uart, "Reading block ");
      write_hex(uart, blk);
      write_str(uart, "\r\n");

      read_block(spi, blk, &dataBuffer[blk * BLOCK_LEN], BLOCK_LEN, uart);
      // Output the first bit of the data...
      for (size_t off = 0u; off < 0x10; ++off) {
        write_hex8b(uart, dataBuffer[blk * BLOCK_LEN + off]);
        if ((off & 0xfu) == 0xfu) {
          write_str(uart, "\r\n");
        } else {
          write_str(uart, " ");
        }
      }
    }
  }

/*  
  uint32_t usedBytes = 0u;
  uint32_t readOffset = 0u;
  uint32_t writeOffset = 0u;
  while (true) {
    uint32_t writeMax = dataBuffer + sizeof(databuffer);
    if (writeOffset < readOffset) {
      writeMax = readOffset & ~(BLOCK_LEN - 1U);
    }
    while (writeOffset < writeMax) {
      read_block(spi, nextBlk,
    }

    draw_image(spi1, gpio, 0, 0, width - 1u, height - 1u, data1, len1, data2, len2);
    readOffset += len1 + len2;
    if (readOffset >= usedBytes) readOffset -= usedBytes;
  }
*/
  spi1->wait_idle();

  const size_t imgBytes = width * height * 2u;  // This is a multiple of the block length.
  uint32_t nextBlk = 0u;
  int wrBuffer = 0u;
  while (true) {
    uint32_t rdOffset = imgBytes * !wrBuffer;
    uint32_t wrOffset = imgBytes *  wrBuffer;
    assert(!(imgBytes & (BLOCK_LEN - 1)));
#if 0
    for (uint32_t blk = 0u; blk < 82; ++blk) {
      read_block(spi, nextBlk + blk, &dataBuffer[blk * BLOCK_LEN], BLOCK_LEN, uart);
    }
    draw_image(spi1, gpio, 0, 0, width - 1u, height - 1u, dataBuffer, imgBytes, nullptr, 0u);
#else
    // Set up the write to the display.
    set_address(spi1, gpio, 0u, 0u, width - 1u, height - 1u);
    write_command(spi1, gpio, &ramwr, 1u);
    set_cs_dc(spi1, gpio, false, true);

    for (unsigned blk = 0; blk < (imgBytes / BLOCK_LEN); blk++) {
      uint8_t crc16[2];

      // Set up the write to the LCD.
      spi1->wait_idle();
      spi1->control = SonataSpi::ControlTransmitEnable;
      spi1->start = BLOCK_LEN;

      // Set up the read from the card.
      spi->cs = spi->cs & ~8u;
      send_command(spi, CMD17, nextBlk + blk, 0xffu, uart);
      while (get_response(spi, uart) != 0xfeu);

      // Our card access currently transfers only a single block...
      spi->control = SonataSpi::ControlReceiveEnable;
      spi->start = BLOCK_LEN;

      uint32_t bytes_tx = 0u;
      uint32_t bytes_rx = 0u;
      while (bytes_tx < BLOCK_LEN || bytes_rx < BLOCK_LEN) {
			  unsigned transmitAvailable = 8 - (spi1->status & SonataSpi::StatusTxFifoLevel);
        while (bytes_tx < BLOCK_LEN && transmitAvailable-- > 0) {
          spi1->transmitFifo = dataBuffer[rdOffset++];
          bytes_tx++;
        }
        while (bytes_rx < BLOCK_LEN && (spi->status & SonataSpi::StatusRxFifoLevel)) {
          dataBuffer[wrOffset++] = spi->receiveFifo;
          bytes_rx++;
        }
//write_hex(uart, bytes_tx);
//write_str(uart, " : ");
//write_hex(uart, bytes_rx);
//write_str(uart, "\r\n");
      }

      // Complete the card read.
      spi->blocking_read(crc16, sizeof(crc16));
      spi->cs = spi->cs | 8u;
    }

    spi1->wait_idle();
    set_cs_dc(spi1, gpio, true, true);
    wrBuffer = !wrBuffer;
#endif
    nextBlk += 80;
    if (nextBlk >= 3097 * 80) nextBlk = 0u;
  }
}
