/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include <stdint.h>

// clang-format off
#include "../../common/defs.h"
// clang-format on
#include <cheri.hh>
#include <platform-spi.hh>

#include "../common/uart-utils.hh"

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

static uint8_t get_response(Capability<volatile SonataSpi> &spi, Capability<volatile OpenTitanUart> &uart) {
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
        write_str(uart, ".");
        break;
      case 0x01u:
        write_str(uart, "\r\n");
        break;
      default:
        if (rd1 != 0xffu) {
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

static uint8_t get_response5(Capability<volatile SonataSpi> &spi, Capability<volatile OpenTitanUart> &uart) {
  uint8_t rd1 = get_response(spi, uart);

  for (int r = 0; r < 4; ++r) {
    uint8_t rd2;
    spi->transmitFifo = 0xffu;
    spi->control = SonataSpi::ControlTransmitEnable | SonataSpi::ControlReceiveEnable;
    spi->start = 1u;
    spi->wait_idle();
	  while ((spi->status & SonataSpi::StatusRxFifoLevel) == 0) {}
	  rd2 = static_cast<uint8_t>(spi->receiveFifo);
  }
  return rd1;
}

static void read_block(Capability<volatile SonataSpi> &spi, uint32_t block, uint8_t *buf, size_t n, Capability<volatile OpenTitanUart> &uart) {
  uint8_t crc16[2];

  spi->cs = spi->cs & ~8u;

  send_command(spi, CMD17, block, 0xffu, uart);
  while (get_response(spi, uart) != 0xfeu);
  spi->blocking_read(buf, n);
  spi->blocking_read(crc16, sizeof(crc16));

  spi->cs = spi->cs | 8u;

  write_str(uart, "Read block CRC ");
  write_hex8b(uart, crc16[0]);
  write_hex8b(uart, crc16[1]);
  write_str(uart, "\r\n");
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
  static uint8_t data[BLOCK_LEN];

  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS + 0x3000;
  spi.bounds()                       = SPI_BOUNDS;

  write_str(uart, "card check\r\n");

  // TODO: Set the pinmux so that we can access the card slot.
  typedef struct {
    uint32_t reg;
  } pinmux_t;
  Capability<volatile pinmux_t> pinmux = root.cast<volatile pinmux_t>();
  pinmux.address() = 0x80005808;
  pinmux.bounds()  = 4u;
  ((volatile uint8_t*)&pinmux->reg)[0] = 8;

// TODO:
  spi->init(false, false, true, 0);  // 300kHz operation.

  spi->blocking_write(ones, 10);  // at least 74 SD CLK cycles.

  spi->wait_idle();
  spi->cs = spi->cs & ~8u;

  // CMD0
  do {
    send_command(spi, CMD0, 0u, 0x95u, uart);
  } while (0x01 != get_response(spi, uart));
#if 0
  do {
//    spi->cs = spi->cs | 8u;
    send_command(spi, CMD1, 0, 0xffu, uart);
  } while (0x01 != get_response(spi, uart));
//  spi->cs = spi->cs | 8u;

//  spi->cs = spi->cs & ~8u;
#endif

  send_command(spi, CMD8, 0x1aau, 0x87u, uart);
  uint8_t rd = get_response5(spi, uart);

  write_hex8b(uart, rd);

//  spi->cs = spi->cs | 8u;

//  spi->cs = spi->cs & ~8u;
  do {
    send_command(spi, CMD55, 0, 0xffu, uart);
    get_response(spi, uart);
    send_command(spi, ACMD41, 1u << 30, 0xffu, uart);
  } while (1 & get_response(spi, uart));
//  spi->cs = spi->cs | 8u;


//  spi->cs = spi->cs & ~8u;

  send_command(spi, CMD58, 0, 0xffu, uart);
  (void)get_response5(spi, uart);

//  spi->cs = spi->cs | 8u;

  write_str(uart, "Setting block length\r\n");

  send_command(spi, CMD16, BLOCK_LEN, 0xffu, uart);
  get_response(spi, uart);

  spi->cs = spi->cs | 8u;

  for (uint32_t blk = 0u; blk < 2; ++blk) {
    write_str(uart, "Reading block ");
    write_hex(uart, blk);
    write_str(uart, "\r\n");

    read_block(spi, blk, data, sizeof(data), uart);
    // Output the data.
    for (size_t off = 0u; off < sizeof(data); ++off) {
      write_hex8b(uart, data[off]);
      if ((off & 0xfu) == 0xfu) {
        write_str(uart, "\r\n");
      } else {
        write_str(uart, " ");
      }
    }
  }

  spi->cs = spi->cs | 8u;

  while (true) {
    asm("");
  }
}
