/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */
#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE

#include <stdint.h>

#include <cheri.hh>
// clang-format off
#include "../../common/defs.h"
// clang-format on
#include "../common/console.hh"
#include "../common/timer-utils.hh"

using namespace CHERI;

struct SonataDma {
  /**
   * Interrupt State Register.
   */
  uint32_t interruptState;
  /**
   * Interrupt Enable Register.
   */
  uint32_t interruptEnable;
  /**
   * Interrupt Test Register.
   */
  uint32_t interruptTest;

  uint32_t control;
  uint32_t status;
  uint32_t error;

  uint32_t reserved0;
  uint32_t reserved1;

  /**
   * Source configuration.
   */
  uint32_t src_config;
  /**
   * Source status information.
   */
  uint32_t src_status;
  /**
   * Source buffer.
   */
  volatile const uint8_t *src_buf;
  /** 
   * Source address stride.
   * - address increment in bytes from the final word read of one row
   *   to the first word of the next row to be read.
   */
  int32_t  src_stride;
  /**
   * Source row length, as words - 1.
   */
  uint32_t src_row_len;
  /**
   * Source row count, as rows - 1.
   */
  uint32_t src_rows;

  uint32_t reserved2;

  /**
   * Destination configuration.
   */
  uint32_t dst_config;
  /**
   * Destination status information.
   */
  uint32_t dst_status;
  /**
   * Destination buffer.
   */
  volatile uint8_t *dst_buf;
  /**
   * Destination address stride.
   * - address increment in bytes from the final word write of one row
   *   to the first word of the next row to be written.
   */
  int32_t  dst_stride;
  /**
   * Destination row length, as words - 1.
   */
  uint32_t dst_row_len;
  /**
   * Destination row count, as rows - 1.
   */
  uint32_t dst_rows;
};

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" uint32_t entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART.
  Capability<volatile OpenTitanUart> uart0 = root.cast<volatile OpenTitanUart>();
  uart0.address()                          = UART_ADDRESS;
  uart0.bounds()                           = UART_BOUNDS;

  uart0->init(BAUD_RATE);
  WriteUart uart{uart0};
  Log log(uart);

  // Create a bounded capability to the DMA.
  Capability<volatile SonataDma> dma = root.cast<volatile SonataDma>();
  dma.address()                      = DMA_ADDRESS;
  dma.bounds()                       = DMA_BOUNDS;

  // Properties of source buffer.
  uint32_t src_line_words = 0x800u;  // in 32-bit words.
  uint32_t src_lines = 0x8u;

  // Properties of destination buffer.
  uint32_t dst_line_words = 0x1000u;  // in 32-bit words.
  uint32_t dst_lines = 0x4u;

  // Create a bounded capability to the source buffer.
  // TODO: We have set up stride parameters here that require the buffers
  // to be double height, so that we have some gaps between the rows.
  Capability<volatile uint8_t> src = root.cast<volatile uint8_t>();
  src.address()                    = SRAM_ADDRESS;
  src.bounds()                     = 2 * src_lines * src_line_words * 4u;

  // Create a bounded capability to the destination buffer.
  Capability<volatile uint8_t> dst = root.cast<volatile uint8_t>();
  dst.address()                    = HYPERRAM_ADDRESS;
  dst.bounds()                     = 2 * dst_lines * dst_line_words * 4u;

  while (true) {
    const bool logging = false;

    // Set up a simple copy from SRAM to HyperRAM
    // 8 rows x 0x800 words/row = 16384 words to be transferred.
    dma->src_config  = 0x11u;
    dma->src_buf     = src.get();
    dma->src_stride  = (src_line_words << 2) + 4;
    dma->src_rows    = src_lines - 1u;
    dma->src_row_len = src_line_words - 1u;

    dma->dst_config  = 0x11u;
    dma->dst_buf     = dst.get();
    dma->dst_stride  = (dst_line_words << 2) + 4;
    dma->dst_rows    = dst_lines - 1u;
    dma->dst_row_len = dst_line_words - 1u;

    log.println("Starting transfer");
    uint32_t start_time = get_mcycle();
    dma->control        = 1;

    int cnt = 0;
    if (logging) {
      log.println("Started, state {}", dma->status & 0xfu);
    }
    while (dma->status & 0xfu) {
      asm("");
      if (++cnt < 10) {
        if (logging) {
          log.println(" - status {:#x}", dma->status);
        }
      }
    }

    log.println("Took {} cycles", get_mcycle() - start_time);
  }
}
