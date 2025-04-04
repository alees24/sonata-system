// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#define CHERIOT_NO_AMBIENT_MALLOC
#define CHERIOT_NO_NEW_DELETE
#define CHERIOT_PLATFORM_CUSTOM_UART

#include "../../common/defs.h"

#include <platform-pwm.hh>
#include <platform-spi.hh>
#include <platform-uart.hh>

#include "../common/timer-utils.hh"
#include "../common/uart-utils.hh"

using namespace CHERI;
using SonataPulseWidthModulation::LcdBacklight;
using SpiLcdCap = Capability<volatile SonataSpi::Lcd>;

#include "../../../vendor/display_drivers/st7735/lcd_st7735_init.h"
#include "lowrisc_logo_native.h"

// When running in simulation, eliminate the delays?
#define SIMULATION 0

const int LcdOutEnPin = 3, LcdRstPin = 2, LcdDcPin = 1, LcdCsPin = 0;

// Use landscape orientation rather than portrait?
#define LANDSCAPE 0

// LCD properties.
#if LANDSCAPE
const uint32_t width  = 160u;
const uint32_t height = 128u;
#else
const uint32_t width  = 128u;
const uint32_t height = 160u;
#endif
// Logo properties.
const uint32_t img_width  = 105u;
const uint32_t img_height = 80u;

#define set_output_bit(a, b, c)                                                                                        \
  (a)->chipSelects = (c) ? ((a)->chipSelects | (1 << (b))) : ((a)->chipSelects & ~(1 << (b)))

// Commonly-used LCD command codes.
static const uint8_t madctl = ST7735_MADCTL;
static const uint8_t caset  = ST7735_CASET;
static const uint8_t raset  = ST7735_RASET;
static const uint8_t ramwr  = ST7735_RAMWR;
static const uint8_t rdid1  = ST7735_RDID1;
static const uint8_t rdid2  = ST7735_RDID2;
static const uint8_t rdid3  = ST7735_RDID3;

// TODO: adjust for clock frequency.
static inline void delay(int delay_ms) {
#if !SIMULATION
  wait_mcycle(30000 * delay_ms);
#endif
}

static inline void set_cs_dc(SpiLcdCap &spi, bool cs, bool d) {
  set_output_bit(spi, LcdDcPin, d);
  set_output_bit(spi, LcdCsPin, cs);
}


static void write_command(SpiLcdCap &spi, const uint8_t *cmd, size_t len) {
  set_cs_dc(spi, false, false);
  spi->blocking_write(cmd, len);
  spi->wait_idle();
}

static void write_buffer(SpiLcdCap &spi, const uint8_t *data, size_t len) {
  while (len > 0u) {
    // SPI controller supports only 0x7ff bytes in a single command.
    size_t chunk = len;
    if (chunk >> 11) chunk = 0x7ffu;
    spi->blocking_write(data, chunk);
    data += chunk;
    len -= chunk;
  }
}

static void write_data(SpiLcdCap &spi, const uint8_t *data, size_t len) {
  set_cs_dc(spi, false, true);
  write_buffer(spi, data, len);
  spi->wait_idle();
  set_cs_dc(spi, true, true);
}

static void read_data(SpiLcdCap &spi, const uint8_t *cmd, uint8_t *data, size_t len) {
  set_cs_dc(spi, false, true);
  write_command(spi, cmd, 1u);
  spi->wait_idle();
  set_output_bit(spi, LcdOutEnPin, 0x0);
  while (len > 0u) {
    // SPI controller supports only 0x7ff bytes in a single command.
    size_t chunk = len;
    if (chunk >> 11) chunk = 0x7ffu;
    spi->blocking_read(data, chunk);
    data += chunk;
    len -= chunk;
  }
  spi->wait_idle();
  set_output_bit(spi, LcdOutEnPin, 0x1);
  set_cs_dc(spi, true, true);
}

static void set_address(SpiLcdCap &spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t coord[4];

  coord[0] = (uint8_t)(x0 >> 8);
  coord[1] = (uint8_t)x0;
  coord[2] = (uint8_t)(x1 >> 8);
  coord[3] = (uint8_t)x1;
  write_command(spi, &caset, 1u);
  write_data(spi, coord, 4u);

  coord[0] = (uint8_t)(y0 >> 8);
  coord[1] = (uint8_t)y0;
  coord[2] = (uint8_t)(y1 >> 8);
  coord[3] = (uint8_t)y1;
  write_command(spi, &raset, 1u);
  write_data(spi, coord, 4u);
}

static void fill_rect(SpiLcdCap &spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint32_t pix) {
  set_address(spi, x0, y0, x1, y1);

  write_command(spi, &ramwr, 1u);
  set_cs_dc(spi, false, true);
  uint32_t npix = (1 + y1 - y0) * (1 + x1 - x0);
  while (npix-- > 0u) write_buffer(spi, (uint8_t *)&pix, 2u);
  spi->wait_idle();
  set_cs_dc(spi, true, true);
}

static void draw_image(SpiLcdCap &spi, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, const uint8_t *data) {
  uint32_t npix = (1 + y1 - y0) * (1 + x1 - x0);

  set_address(spi, x0, y0, x1, y1);
  write_command(spi, &ramwr, 1u);
  write_data(spi, data, npix << 1);
}

static void run_script(Capability<volatile OpenTitanUart> &uart, SpiLcdCap &spi, const uint8_t *addr) {
  uint8_t numCommands, numArgs;
  uint16_t delay_ms;

  numCommands = NEXT_BYTE(addr);  // Number of commands to follow

  while (numCommands--) {  // For each command...
    const uint8_t cmd = NEXT_BYTE(addr);

    write_command(spi, &cmd, 1u);

    numArgs  = NEXT_BYTE(addr);  // Number of args to follow
    delay_ms = numArgs & DELAY;  // If hibit set, delay follows args
    numArgs &= ~DELAY;           // Mask out delay bit

    write_data(spi, addr, numArgs);

    addr += numArgs;
    if (delay_ms) {
      delay_ms = NEXT_BYTE(addr);                     // Read post-command delay time (ms)
      delay_ms = (delay_ms == 255) ? 500 : delay_ms;  // If 255, delay for 500 ms
      delay(delay_ms);
    }
  }
}

/**
 * C++ entry point for the loader.  This is called from assembly, with the
 * read-write root in the first argument.
 */
extern "C" [[noreturn]] void entry_point(void *rwRoot) {
  Capability<void> root{rwRoot};

  // Create a bounded capability to the UART
  Capability<volatile OpenTitanUart> uart = root.cast<volatile OpenTitanUart>();
  uart.address()                          = UART_ADDRESS;
  uart.bounds()                           = UART_BOUNDS;

  // The LCD is driven by SPI controller 0.
  SpiLcdCap spi = root.cast<volatile SonataSpi::Lcd>();
  spi.address() = SPI_ADDRESS;
  spi.bounds()  = SPI_BOUNDS;

  // Create a bounded capability for the final PWM channel since this drives the LCD backlight.
  Capability<volatile LcdBacklight> pwm = root.cast<volatile LcdBacklight>();
  pwm.address()                         = PWM_ADDRESS + PWM_LCD * PWM_RANGE;
  pwm.bounds()                          = PWM_BOUNDS;
  pwm->output_set(1u, 255u);

  uart->init(BAUD_RATE);
  write_str(uart, "LCD check\r\n");

  // This is maximum speed (15Mbps presently) which works fine, but we can run at a lower speed
  // to exercise interrupts/FIFO behaviour more thoroughly.
  const uint8_t SpiSpeed = 7u;
  spi->init(false, false, true, SpiSpeed);

  // Set the initial state of the LCD control pins.
  set_output_bit(spi, LcdDcPin, 0x0);

  // Reset LCD.
  set_output_bit(spi, LcdRstPin, 0x0);
  delay(150);
  set_output_bit(spi, LcdRstPin, 0x1);

  // Send display initialisation commands.
  run_script(uart, spi, init_script_b);
  run_script(uart, spi, init_script_r);
  run_script(uart, spi, init_script_r3);

  uint8_t data[8];
  read_data(spi, &rdid1, data, 1);
  write_str(uart, "RDID1: ");
  write_hex(uart, data[0]);
  write_str(uart, "\r\n");
  read_data(spi, &rdid2, data, 1);
  write_str(uart, "RDID2: ");
  write_hex(uart, data[0]);
  write_str(uart, "\r\n");
  read_data(spi, &rdid3, data, 1);
  write_str(uart, "RDID3: ");
  write_hex(uart, data[0]);
  write_str(uart, "\r\n");

  while (true) {
    asm("");
  }
}
