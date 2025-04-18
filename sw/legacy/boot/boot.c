// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdbool.h>

#include "gpio.h"
#include "pwm.h"
#include "sonata_system.h"
#include "timer.h"

/**
 * Is the code running in simulation (as opposed to on FPGA)?
 */
#define SIMULATION 0

static const bool dual_uart = true;
static uart_t uart0, uart1;

static inline void write_both(char ch0, char ch1) {
  uart_out(uart0, ch0);
  uart_out(uart1, ch1);
}

int main(void) {
  if (dual_uart) {
    const char *signon = "Legacy boot code; UART ";
    uart0              = UART_FROM_BASE_ADDR(UART0_BASE);
    uart1              = UART_FROM_BASE_ADDR(UART1_BASE);
    uart_init(uart0);
    uart_init(uart1);
    // Send a sign-on message to both UARTs.
    char ch;
    while ('\0' != (ch = *signon)) {
      write_both(ch, ch);
      signon++;
    }
    write_both('0', '1');
    write_both('\r', '\r');
    write_both('\n', '\n');
  } else {
    uart0 = DEFAULT_UART;
    uart_init(uart0);
    puts("Legacy boot code");
  }

  // This indicates how often the timer gets updated.
  timer_init();
#if SIMULATION
  timer_enable(SYSCLK_FREQ / 500);
#else
  timer_enable(SYSCLK_FREQ / 5);
#endif

  uint64_t last_elapsed_time = get_elapsed_time();

  // Reset user LEDs to having just one on
  set_outputs(GPIO_OUT, 0x01);

  // PWM variables
  uint32_t counter    = UINT8_MAX;
  uint32_t brightness = 0;
  bool ascending      = true;
  // The three least significant bits correspond to RGB, where B is the leas significant.
  uint8_t color = 7;

  while (1) {
    uint64_t cur_time = get_elapsed_time();
    if (cur_time != last_elapsed_time) {
      last_elapsed_time = cur_time;

      // Cycle through LEDs
      uint32_t out_val = read_gpio(GPIO_OUT);
      out_val          = (out_val << 1) & GPIO_LED_MASK;
      out_val         |= !out_val;
      set_outputs(GPIO_OUT, out_val);

      // Going from bright to dim on PWM
      for (int i = 0; i < NUM_PWM_MODULES; i++) {
        set_pwm(PWM_FROM_ADDR_AND_INDEX(PWM_BASE, i), ((1 << (i % 3)) & color) ? counter : 0,
                brightness ? 1 << (brightness - 1) : 0);
      }
      if (ascending) {
        brightness++;
        if (brightness >= 5) {
          ascending = false;
        }
      } else {
        brightness--;
        // When LEDs are off cycle through the colors
        if (brightness == 0) {
          ascending = true;
          color++;
          if (color >= 8) {
            color = 1;
          }
        }
      }
    }

    asm volatile("wfi");
  }
}
