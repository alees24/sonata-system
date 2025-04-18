/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"
#include "../common/uart-utils.hh"
#include "../common/platform-pinmux.hh"
#include "../common/timer-utils.hh"
#include "../common/sonata-devices.hh"
#include <cheri.hh>

#define RS485_UART 2

using namespace CHERI;

// Continuosly outputs a message with an incrementing number on RS485 serial
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  pin_sinks_ptr(root)->get(SonataPinmux::PinSink::rs485_tx).select(1);
  block_sinks_ptr(root)->get(SonataPinmux::BlockSink::uart_2_rx).select(3);

  UartPtr uart = uart_ptr(root, 0);
  uart->init(BAUD_RATE);

  Capability<volatile OpenTitanUart> rs485_uart = root.cast<volatile OpenTitanUart>();
  rs485_uart.address()                          = UART_ADDRESS + RS485_UART * UART_RANGE;
  rs485_uart.bounds()                           = UART_BOUNDS;
  rs485_uart->init(BAUD_RATE);

  write_str(uart, "Spamming RS-485\r\n");

  uint32_t num = 0;
  while (1) {
    write_str(rs485_uart, "0x");
    write_hex(rs485_uart, num);
    write_str(rs485_uart, " is a number\r\n");

    while (rs485_uart->transmit_fifo_level() > 0) {
    }
    num++;
    // Chosen experimentally looking at rs485 signals with a logic analyzer.
    // With this exact delay sometimes the messages will come out back to back
    // and sometimes there will be a brief (~25-75ns) gap where the RS485 driver
    // is disabled
    wait_mcycle(415);
  }
}
