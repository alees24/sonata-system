// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <assert.h>
#include <stdio.h>
#include "spidpi.hh"

// -------------------------- SPI microSD model -------------------------------
class spi_microsd : public spidpi {
public:
  spi_microsd(unsigned dataW,      // Number of data lines.
              unsigned oobInW,     // Width of Out-Of-Band input data (bits).
              unsigned oobOutW,    // Width of Out-Of-Band output data (bits).
              const char *sdFile,  // Filename of the SD card image.
              bool log = false) :  // Enable diagnostic logging?
              spidpi(dataW, oobInW, oobOutW, log) {
      assert(sdFile);
      sd = fopen(sdFile, "rb");
      logText("microSD model attempting to open image '%s'\n", sdFile);
      reset();
   }

protected:
  // Device reset.
  virtual void reset();

  // Write a byte to the SPI flash.
  virtual void writeByte(uint8_t inByte, uint32_t oobIn);

  // Read a byte of data from the SPI flash.
  virtual bool readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut);

  // Change in the state of the CS line.
  virtual void csChanged(bool csAsserted, uint32_t oobIn);

  // SD command received.
  virtual void startCommand();

  // Send response from SD card.
  virtual void sendResponse(unsigned len, uint64_t d); // Up to 7 bytes.

private:

  // SD card commands are 48 bits (6 bytes) with the data being transmitted MSB first.
  struct {
    uint8_t  cmdCode;
    uint32_t address;
    uint8_t  crc;
  } cmd;

  // The number of bytes thus far received.
  uint8_t cmdBytes;

  // Responding to a command?
  bool responding;
  uint8_t rsp[7];  // TODO:
  unsigned rspBytes;
  unsigned rspLen;

  // Reading data from the microSD card?
  bool reading;
  enum {
    ReadState_CmdStatus,
    ReadState_DataBytes,
    ReadState_CRC0,
    ReadState_CRC1
  } readState;
  // Number of bytes returned within the current data packet.
  uint32_t readBytes;

  // Storage medium (raw microSD card image; format and contents unspecified).
  // If the file handle is NULL this means that there is no microSD card present.
  FILE *sd;
  // Size of the storage medium.
  uint64_t memSize;
  // Current byte offset within the storage.
  uint64_t memOffset;
};
