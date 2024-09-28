// Copyright lowRISC Contributors.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "../../common/defs.h"
#include "../common/console-utils.hh"
#include "../common/sonata-devices.hh"
#include "../common/uart-utils.hh"
#include "test_runner.hh"
#include <cheri.hh>
#include <platform-i2c.hh>
#include <platform-uart.hh>

using namespace CHERI;

/**
 * Configures the number of test iterations to perform.
 * This can be overridden via a compilation flag.
 */
#ifndef I2C_TEST_ITERATIONS
#define I2C_TEST_ITERATIONS (1U)
#endif

/**
 * Configures whether a Raspberry Pi Sense HAT is connected to the board.
 * This can be overridden via a compilation flag.
 */
#ifndef I2C_RPI_HAT_AVAILABLE
#define I2C_RPI_HAT_AVAILABLE true
#endif

/**
 * Configures whether a AS6212 Sparkfun Temperature sensor is connected.
 * This can be overriden via a compilation flag.
 */
#ifndef I2C_AS6212_AVAILABLE
#define I2C_AS6212_AVAILABLE true
#endif

/**
 * Configure whether the RPI HAT ID EEPROM test should just test that *some*
 * contents are read (false), or whether the contents should be verified to
 * be what we actually expect (true). This is made optional as the EEPROM
 * device is not read-only and supports byte/page writes, and thus might
 * change and cause the test to fail even if the I2C is working.
 * This can be overriden via a compilation flag.
 */
#ifndef I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS
#define I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS true
#endif

/**
 * The location of the pins used to access the Raspberry Pi Sense HAT's
 * ID EEPROM registers, and the length of data to read in tests for
 * testing purposes.
 */
constexpr uint8_t RpiHatIdEepromAddr       = 0x50u;
constexpr uint8_t RpiHatIdEepromHeaderSize = 0x80u;

/**
 * The expected (if not manually written) EEPROM ID Header from the
 * Raspberry Pi Sense HAT.
 */
static const uint8_t expectedRpiHatIdEepromHeader[] = {
    0x52, 0x2D, 0x50, 0x69, 0x01, 0x00, 0x03, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,  // R-Pi............
    0x2D, 0x00, 0x00, 0x00, 0xE7, 0x58, 0xD1, 0x95, 0xF1, 0x56, 0x28, 0xAB, 0xCB, 0x4E, 0x99, 0x42,  // -....X...V(..N.B
    0xC7, 0x79, 0xD6, 0xA3, 0x01, 0x00, 0x01, 0x00, 0x0C, 0x09, 0x52, 0x61, 0x73, 0x70, 0x62, 0x65,  // .y........Raspbe
    0x72, 0x72, 0x79, 0x20, 0x50, 0x69, 0x53, 0x65, 0x6E, 0x73, 0x65, 0x20, 0x48, 0x41, 0x54, 0x7F,  // rry PiSense HAT.
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ................
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ................
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // ................
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00   // ................
};

/**
 * The location of the Raspberry Pi Sense HAT's Accelerometer and
 * Gyroscope's `WHO_AM_I` register, and the value that we expect
 * this to report.
 */
constexpr uint8_t RpiHatAccelGyroWhoAmIAddr = 0x6A;
constexpr uint8_t ExpectedRpiHatAccelGyroID = 0x68;

/**
 * The location of the Raspberry Pi Sense HAT's Magnetic Sensor's
 * `WHO_AM_I` register, and the value that we expect this to report.
 */
constexpr uint8_t RpiHatIdMagneticWhoAmIAddr = 0x1C;
constexpr uint8_t ExpectedRpiHatMagneticID   = 0x3D;

/**
 * The location of the AS6212 Temperature Sensor in I2C.
 */
constexpr uint8_t As6212Addr = 0x48;

/**
 * The addresses of various I2C device registers used in testing.
 */
constexpr uint8_t RpiHatWhoAmIRegAddr = 0x0F;
constexpr uint8_t As6212TempRegAddr   = 0x00;
constexpr uint8_t As6212ConfigRegAddr = 0x01;

/**
 * This test expects a Raspberry Pi Sense HAT to be connected to the Sonata
 * board. The test configures the I2C, and then performs a write and read
 * to the appropriate RPI HAT ID EEPROM pins to access the data stored in
 * the I2C EEPROM header. Depending on the definition of
 * `I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS`, this test will either verify that
 * *something* is read, or will verify the exact contents match.
 * Returns the number of failures that occur.
 */
int i2c_rpi_hat_id_eeprom_test(I2cPtr i2c) {
  int failures = 0;

  // Setup the i2c bus, configuring it in host mode with speed of 100.
  i2c->reset_fifos();
  i2c->host_mode_set();
  i2c->speed_set(100);

  // Send two 0x0000 byte addresses and skip the STOP condition.
  const uint8_t addr[] = {0, 0};
  i2c->blocking_write(RpiHatIdEepromAddr, addr, sizeof(addr), true);

  // Fill a read buffer with dummy data so that we see changes.
  constexpr uint8_t DummyReadVal = 0x3D;
  std::array<uint8_t, RpiHatIdEepromHeaderSize> eeprom_data;
  eeprom_data.fill(DummyReadVal);

  // Read a page from the RPI Hat ID EEPROM
  if (!i2c->blocking_read(RpiHatIdEepromAddr, eeprom_data.data(), RpiHatIdEepromHeaderSize)) {
    failures++;
  }

  // Verify the contents of the EEPROM header that were read.
  bool buffer_changed = false;
  for (uint32_t index = 0; index < RpiHatIdEepromHeaderSize; index++) {
    if (I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS && eeprom_data[index] != expectedRpiHatIdEepromHeader[index]) {
      failures++;
    } else if (!I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS && eeprom_data[index] != DummyReadVal) {
      buffer_changed = true;
    }
  }

  // If not checking the contents, check that at least some value in the buffer
  // changed. Should not fail unless the entire contents of the EEPROM header
  // are 0x3D.
  if (!I2C_TEST_RPI_HAT_ID_EEPROM_CONTENTS && !buffer_changed) {
    failures++;
  }

  return failures;
}

/**
 * This test expects a Raspberry Pi Sense HAT to be connected to the Sonata
 * board. The test configures the I2C, and then performs write and read
 * to the appropriate addresses to obtain the value of the `WHO_AM_I`
 * identifying registers for the Accelerometer & Gyroscope and magnetic
 * sensor devices. It then checks that these values match the values we
 * would expect.
 * Returns the number of failures that occur.
 */
int i2c_rpi_hat_imu_whoami_test(I2cPtr i2c) {
  int failures = 0;

  // Setup the i2c bus, configuring it in host mode with speed of 100.
  i2c->reset_fifos();
  i2c->host_mode_set();
  i2c->speed_set(100);

  // Send the address and skip the STOP condition.
  const uint8_t addr[] = {RpiHatWhoAmIRegAddr};
  i2c->blocking_write(RpiHatAccelGyroWhoAmIAddr, addr, sizeof(addr), true);

  // Read from the `WHO_AM_I` register of the Accelerometer & Gyroscope,
  // and check it matches the expected value.
  uint8_t data[1] = {0xFF};
  if (!i2c->blocking_read(RpiHatAccelGyroWhoAmIAddr, data, 1U) || data[0] != ExpectedRpiHatAccelGyroID) {
    failures++;
  }

  // Send one 0x0F byte address and skip the STOP condition.
  i2c->blocking_write(RpiHatIdMagneticWhoAmIAddr, addr, sizeof(addr), true);

  // Read from the `WHO_AM_I` register of the Magnetic Sensor, and check
  // it matches the expected value.
  if (!i2c->blocking_read(RpiHatIdMagneticWhoAmIAddr, data, 1U) || data[0] != ExpectedRpiHatMagneticID) {
    failures++;
  }

  return failures;
}

/**
 * This test expects an AS6212 Sparkfun temperature sensor to beconnected to
 * the Sonata board. This test configures the I2C, and then performs an I2C
 * write and read so as to read from the config register and temperature
 * values. It checks that the config register is configured to some value,
 * and that the temperature value that is being reported is between 20 and
 * 80 degrees Celsius.
 * Returns the number of failures.
 */
int i2c_as6212_temperature_sense_test(I2cPtr i2c) {
  int failures = 0;

  // Setup the i2c bus, configuring it in host mode with speed of 100.
  i2c->reset_fifos();
  i2c->host_mode_set();
  i2c->speed_set(100);

  // Read from the config register, and check the dummy data was modified.
  uint8_t addr[] = {As6212ConfigRegAddr};
  i2c->blocking_write(As6212Addr, addr, sizeof(addr), false);
  uint8_t data[2] = {0xF0, 0x0F};
  if (!i2c->blocking_read(As6212Addr, data, 2U) || data[0] == 0xF0) {
    failures++;
  }

  // Read from the temperature register
  addr[0] = As6212TempRegAddr;
  i2c->blocking_write(As6212Addr, addr, sizeof(addr), false);
  if (!i2c->blocking_read(As6212Addr, data, 2U)) {
    failures++;
  }
  const uint16_t temp_data = (static_cast<uint16_t>(data[0]) << 8) | data[1];
  const int16_t temp       = temp_data;

  // Check that the measured temperature is between 0-75 degrees Celsius.
  if (temp < 0x0000 /*0C*/ || temp > 0x2580 /*75C*/) {
    failures++;
  }
  return failures;
}

/**
 * Run the whole suite of I2C tests.
 */
void i2c_tests(CapRoot root, UartPtr console) {
  // Execute the specified number of iterations of each test.
  for (size_t i = 0; i < I2C_TEST_ITERATIONS; i++) {
    write_str(console, "\r\nrunning i2c_test: ");
    write_hex8b(console, i);
    write_str(console, "\\");
    write_hex8b(console, I2C_TEST_ITERATIONS - 1);
    write_str(console, "\r\n");

    bool test_failed = false;
    int failures     = 0;

    if (I2C_RPI_HAT_AVAILABLE) {
      // Retrieve bounded capabilities for both I2C controllers
      I2cPtr i2c0 = i2c_ptr(root, 0);
      I2cPtr i2c1 = i2c_ptr(root, 1);

      write_str(console, "  Running RPI HAT ID EEPROM test... ");
      failures = i2c_rpi_hat_id_eeprom_test(i2c0);
      test_failed |= (failures > 0);
      write_test_result(console, failures);

      write_str(console, "  Running RPI HAT ID WHO_AM_I test... ");
      failures = i2c_rpi_hat_imu_whoami_test(i2c1);
      test_failed |= (failures > 0);
      write_test_result(console, failures);
    }

    if (I2C_AS6212_AVAILABLE) {
      // Retrieve bounded capabilities for I2C controller 1
      I2cPtr i2c1 = i2c_ptr(root, 1);

      write_str(console, "  Running AS6212 Temperature test... ");
      failures = i2c_as6212_temperature_sense_test(i2c1);
      test_failed |= (failures > 0);
      write_test_result(console, failures);
    }

    check_result(console, !test_failed);
  }
}