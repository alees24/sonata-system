// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include "spi_microsd.hh"

enum {
  CMD0 = 0,
  CMD1 = 1,
  CMD8 = 8,
  CMD12 = 12,
  CMD_SET_BLOCKLEN = 16,
  CMD_READ_BLOCK = 17,
  CMD_WRITE_BLOCK = 24,
  ACMD41 = 41,
  CMD55 = 55,
  CMD58 = 58
};

void spi_microsd::reset() {
  spidpi::reset();
  cmdBytes = 0u;
  responding = false;
  reading = false;
}

void spi_microsd::writeByte(uint8_t inByte, uint32_t oobIn) {
  logText("microSD %0x\n", inByte);

  switch (cmdBytes) {
    case 0u:
      if ((inByte & 0xc0) == 0x40) {
        cmd.cmdCode = inByte & 0x3f;
        cmdBytes = 1u;
      }
      break;
    case 5u:
      cmd.crc = inByte;
      startCommand();
      cmdBytes = 0u;
      break;
    default:
      // Collect the next byte of the address.
      cmd.address = (cmd.address << 8) | inByte;
      cmdBytes++;
      break;
  }
}

bool spi_microsd::readByte(uint32_t oobIn, uint8_t &outByte, uint32_t &oobOut) {
  if (responding) {
    if (rspBytes < rspLen) {
      outByte = rsp[rspBytes++];
      // OOB data carries SD card detect - absent (1) or present (0).
      oobOut = !sd;
      return true;
    }
    if (reading) {
      readState = ReadState_DataBytes;
    }
    responding = false;
  }
  if (reading) {
    // Checksum?
    switch (readState) {
      case ReadState_DataBytes: {
        // Return the next byte from the memory.
        int ch = fgetc(sd); // TODO: perhaps implement some buffering?
        assert(ch != EOF);    
        outByte = ch;
        memOffset++;
        if (!(++readBytes & 0x1ffu)) {
          readState = ReadState_CRC0;
        }
      }
      break;
      case ReadState_CRC0:
        outByte = 0u;
        readState = ReadState_CRC1;
        break;
      case ReadState_CRC1:
        readBytes = 0u;
        outByte = 0u;
        readState = ReadState_DataBytes;
        break;
      default:
        outByte = 0xffu;
        reading = false;
        break;
    }

    // OOB data carries SD card detect - absent (1) or present (0).
    oobOut = !sd;
    logText("Read byte 0x%0x\n", outByte);
    return true;
  } else {
    return false;
  }
}

// Transition on one or more CS lines.
void spi_microsd::csChanged(bool csAsserted, uint32_t oobIn) {
  cmdBytes = 0u;
  responding = false;
  reading = false;
}

// TODO: prevent command processing trying to read from non-extant SD card;
// reject out-of-order command codes etc.
void spi_microsd::startCommand() {
  logText("Starting command %02x,%08x,%02x", cmd.cmdCode, cmd.address, cmd.crc);
  switch (cmd.cmdCode) {
    case CMD0:
      sendResponse(1u, 0x01);
      break;
    case CMD8:
      sendResponse(5u, 0);
      break;
    // case CMD12:
    case CMD_SET_BLOCKLEN:
      sendResponse(1u, 0);
      break;
    case CMD_READ_BLOCK:
      sendResponse(1u, 0xfe);
      reading = true;
      readState = ReadState_CmdStatus;
      readBytes = 0u;
      break;
    // case CMD_WRITE_BLOCK:
    case ACMD41:
      sendResponse(1, 0x00);
      break;
    case CMD55:
      sendResponse(1, 0x00);
      break;
    case CMD58:
      sendResponse(5u, 0);
      break;    

    default:
      // Treat anything else as an illegal command.
      sendResponse(1u, 0x05);
      break;
  }
}

void spi_microsd::sendResponse(unsigned len, uint64_t d) {
  responding = true;
  rspBytes = 0u;
  rspLen = len;
  rsp[0] = (uint8_t)d;
  rsp[1] = (uint8_t)(d >> 8);
  rsp[2] = (uint8_t)(d >> 16);
  rsp[3] = (uint8_t)(d >> 24);
  rsp[4] = (uint8_t)(d >> 32);
  rsp[5] = (uint8_t)(d >> 40);
  rsp[6] = (uint8_t)(d >> 48);
}

