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
#include "../common/filesys-utils.hh"
#include "../common/platform-pinmux.hh"
#include "../common/sdcard-utils.hh"
#include "../common/sonata-devices.hh"

#include "../tests/test_runner.hh"

// Lorem Ipsum sample text.
#include "lorem_text.hh"

using namespace CHERI;

#define MAX_BLOCKS 0x10

// Set this to true to emit the `lorem ipsum` sample text for capture and subsequent
// writing to a FAT32-formatted microSD card as `LOREM.IPS` within the root directory.
static constexpr bool emitText = false;

static uint8_t dataBuffer[MAX_BLOCKS * BLOCK_LEN];
static uint8_t fileBuffer[BLOCK_LEN];

// Dump out a sequence of bytes as hexadecimal and ASCII text.
static void dump_bytes(const uint8_t *buf, size_t blkBytes, Log &log) {
  for (size_t off = 0u; off < blkBytes; ++off) {
    log.print("{:02x}", buf[off]);
    if ((off & 0xfu) == 0xfu) {
      log.print(" : ");
      for (size_t aoff = (off & ~0xfu); aoff <= off; aoff++) {
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

// Compare a sequence of bytes against a reference, returning the number of mismatches.
static int compare_bytes(const char *ref, unsigned& offset, const uint8_t *data, size_t len, Log& log) {
  unsigned mismatches = 0u;
  while (len-- > 0u) {
    // Compare retrieved data byte against reference text.
    uint8_t dch = *data++;
    char ch = ref[offset++];
    // It's quite likely that the data stored on the card is LF-terminated rather than
    // the CR,LF termination that we expect, so we permit that and continue checking.
    if ((char)dch == '\n' && ch == '\r') {
      ch = ref[offset++];
    }
    mismatches += (char)dch != ch;
  }
  return mismatches;
}

// Read and report the properties of the SD card itself (CSD and CID).
static int read_card_properties(SdCard &sd, Log &log, bool logging = false) {
  int failures = 0u;
  uint8_t buf[16];
  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("  Reading Card Specific Data (CSD)");
  if (sd.read_csd(buf, sizeof(buf))) {
    if (logging) {
      dump_bytes(buf, sizeof(buf), log);
    }
    // TODO: Are there any sense/validation checks that we may perform?
  } else {
    failures++;
  }
  write_test_result(log, failures);

  for (int i = 0; i < sizeof(buf); i++) {
    buf[i] = 0xbd;
  }
  log.print("  Reading Card Identification (CID)");
  if (sd.read_cid(buf, sizeof(buf))) {
    if (logging) {
      dump_bytes(buf, sizeof(buf), log);
    }
    // TODO: Are there any sense/validation checks that we may perform?
  } else {
    failures++;
  }
  write_test_result(log, failures);

  return failures;
}

/**
 * Run the set of SD card tests; test card presence, read access to the card itself
 * and then the data stored within the flash. The test expects a FAT32-formatted
 * SD card with a sample file called `LOREM.IPS` in the root directory.
 */
void sdcard_tests(CapRoot &root, Log &log) {
  constexpr bool logging = false;

  log.println("Card check");

  // Have we been asked to emit the sample text?
  if (emitText) {
    log.println(
        "Capture everything between the dotted lines, being careful not "
        "to introduce any additional line breaks.");
    log.println("--------");
    log.print(lorem_text);
    log.println("--------");
    log.println(
        "Each of these single-line paragraphs shall be CR,LF terminated "
        "and followed by a blank line.");
    log.println("This includes the final one, and thus the file itself ends with a blank line.");
    log.println("The file should be 4,210 bytes in length.");
  }

  // The SPI controller talkes to the microSD card in SPI mode.
  Capability<volatile SonataSpi> spi = root.cast<volatile SonataSpi>();
  spi.address()                      = SPI_ADDRESS + 2 * SPI_RANGE;
  spi.bounds()                       = SPI_BOUNDS;

  // We need to use the pinmux to select the microSD card for SPI controller 2 reads (CIPO),
  // as well as preventing outbound traffic to the microSD card also reaching the application
  // flash (for safety; it _should_ ignore traffic not accompanied by Chip Select assertion).
  Capability<volatile uint8_t> pinmux = root.cast<volatile uint8_t>();
  pinmux.address()                    = PINMUX_ADDRESS;
  pinmux.bounds()                     = PINMUX_BOUNDS;
  SonataPinmux Pinmux                 = SonataPinmux(pinmux);
  // Suppress traffic to the application flash.
  constexpr uint8_t Disabled = 0;
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_cs, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_clk, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_d0, Disabled);
  // Direct SPI controller 2 to drive the microSD pins.
  constexpr uint8_t EnableFirstInput = 1;
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_dat3, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_clk, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_cmd, EnableFirstInput);
  // Select microSD CIPO as SPI controller input.
  constexpr uint8_t PmuxSpi0CipoToSdDat0 = 2;
  Pinmux.block_input_select(SonataPinmux::BlockInput::spi_0_cipo, PmuxSpi0CipoToSdDat0);

  // We need to use the GPIO to detect card presence.
  Capability<volatile SonataGPIO> gpio = root.cast<volatile SonataGPIO>();
  gpio.address()                       = GPIO_ADDRESS;
  gpio.bounds()                        = GPIO_BOUNDS;

  // microSD card is on Chip Select 1 (0 goes to the application flash).
  constexpr unsigned csBit = 1u;
  // microSD card detection bit is on input 16.
  constexpr unsigned detBit = 16u;

  SdCard sd(spi, gpio, csBit, detBit);  //, &log);

  // Wait until a card is detected.
  while (!sd.present()) {
    log.println("Please insert a microSD card into the slot...");
    while (!sd.present());
  }

  sd.init();

  fileSysUtils fs;

  int failures = 0u;

  log.println("Reading card properties.... ");
  failures += read_card_properties(sd, log);
  write_test_result(log, failures);

  // Report the first bit of each block at the start of the card.
  const size_t numBlocks = 4u;
  const size_t blkBytes  = BLOCK_LEN;

  for (uint32_t blk = 0u; blk < numBlocks; ++blk) {
    if (logging) {
      log.println("Reading block {}", blk);
    }
    sd.read_blocks(blk, &dataBuffer[blk * BLOCK_LEN], 1u);
    // Output the first bit of data...
    if (logging) dump_bytes(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
  }

  if (dataBuffer[0x1fe] == 0x55 && dataBuffer[0x1ff] == 0xaa) {
    uint32_t blk_offset;
    bool use_lba = true;
    bool exFat   = true;
    bool found   = false;

    for (unsigned part = 0u; part < 1u; part++) {
      const unsigned partDesc = 0x1be + (part << 4);
      uint8_t part_type       = dataBuffer[partDesc + 4];
      uint32_t lba_start      = fileSysUtils::read32le(&dataBuffer[partDesc + 8]);
      uint32_t num_secs       = fileSysUtils::read32le(&dataBuffer[partDesc + 12]);
      uint16_t start_c, end_c;
      uint8_t start_h, end_h;
      uint8_t start_s, end_s;
      fileSysUtils::read_chs(start_c, start_h, start_s, &dataBuffer[partDesc + 1]);
      fileSysUtils::read_chs(end_c, end_h, end_s, &dataBuffer[partDesc + 5]);
      log.println("Partition {} : type {} : start C {} H {} S {} : end C {} H {} S {}", part, part_type, start_c,
                  start_h, start_s, end_c, end_h, end_s);
      log.println("   LBA start: {:#010x} sectors: {:#010x}", lba_start, num_secs);

      switch (part_type) {
        // FAT32 a viable alternative.
        case 0x0B:
          use_lba = false;
          // no break
        case 0x0C:
          exFat = false;
          // no break
        // exFAT preferred.
        case 0x07: {
          const uint16_t nheads = 255u;
          const uint16_t nsecs  = 63u;
          if (use_lba) {
            blk_offset = lba_start;
          } else {
            blk_offset = sd.chs_to_lba(start_c, start_h, start_s, nheads, nsecs);
          }
          log.println("Expecting EBR at block {:#x}", blk_offset);
          found = true;
        } break;
        default:
          log.println("Not a suitable partition");
          break;
      }
    }

    if (!found) {
      log.println("Unable to locate a suitable partition");
      check_result(log, false);
    }

    // Read the EBR at the start of the partition.
    for (uint32_t blk = 0u; blk < 1u; blk++) {
      if (logging) {
        log.println("Reading block {}", blk_offset + blk);
      }
      sd.read_blocks(blk_offset + blk, &dataBuffer[blk * BLOCK_LEN], 1u);
      if (logging) {
        // Output the first bit of data...
        dump_bytes(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
      }
    }

    uint8_t bytesPerSectorShift;
    uint8_t secsPerClusterShift;
    uint32_t clusterHeapOffset;
    uint32_t rootCluster;
    uint32_t rootStart;
    uint32_t fatOffset;

    if (exFat) {
      clusterHeapOffset   = fileSysUtils::read32le(&dataBuffer[88]);
      rootCluster         = fileSysUtils::read32le(&dataBuffer[96]);
      bytesPerSectorShift = dataBuffer[108];
      secsPerClusterShift = dataBuffer[109];
    } else {
      uint16_t bytesPerSector = fileSysUtils::read16le(&dataBuffer[0x0b]);
      uint16_t resvdSectors   = fileSysUtils::read16le(&dataBuffer[0xe]);
      uint8_t numFATs         = dataBuffer[0x10];
      uint32_t secsPerFAT     = fileSysUtils::read32le(&dataBuffer[0x24]);
      rootCluster             = fileSysUtils::read32le(&dataBuffer[0x2c]);

      log.println("FAT32 {} FATs, secs per FAT {}, bytes/sec {}", numFATs, secsPerFAT, bytesPerSector);
      log.println(" resvdSectors {}", resvdSectors);

      bytesPerSectorShift = 0u;
      while (bytesPerSector > 1u) {
        bytesPerSector >>= 1;
        bytesPerSectorShift++;
      }
      uint8_t secsPerCluster = dataBuffer[0xd];
      secsPerClusterShift    = 0u;
      while (secsPerCluster > 1u) {
        secsPerCluster >>= 1;
        secsPerClusterShift++;
      }

      fatOffset         = resvdSectors;
      clusterHeapOffset = ((resvdSectors + (numFATs * secsPerFAT)) << bytesPerSectorShift) / BLOCK_LEN;
    }

    rootStart = ((rootCluster - 2) << secsPerClusterShift << bytesPerSectorShift) / BLOCK_LEN;
    rootStart += clusterHeapOffset;

    log.println("Cluster heap offset {} Root cluster {} log2(bytes/sec) {} log2(secs/cluster) {}", clusterHeapOffset,
                rootCluster, bytesPerSectorShift, secsPerClusterShift);

    // Populate the buffer with invalid data that cannot be misinterpreted.
    // TODO: more blocks; this is to be replaced by directory reading code in fileSysUtils.
    for (int off = 0; off < 4u * BLOCK_LEN; off++) {
      dataBuffer[off] = 0xbd;
    }
    sd.read_blocks(blk_offset + rootStart, dataBuffer, 4u);
    if (logging) {
      for (uint32_t blk = 0u; blk < 1u; blk++) {
        // Output the first bit of data...
        dump_bytes(&dataBuffer[blk * BLOCK_LEN], blkBytes, log);
      }
    }

    // TODO: Change this to a normal string and update the comparison code in fileSysUtils.
    const char *file = "LOREM   IPS";

    // Locate a directory entry.
    const uint8_t *dir_entry = dataBuffer;
    while (fileSysUtils::namecmp(dir_entry, file, 11)) {
      // TODO:
      dir_entry += 0x20;
    }

    uint32_t fileLen = fileSysUtils::read32le(&dir_entry[0x1c]);
    if (logging) {
      log.println("File located at {} : {} bytes", dir_entry - dataBuffer, fileLen);
    }

    uint32_t cluster =
        ((uint32_t)fileSysUtils::read16le(&dir_entry[0x14]) << 16) | fileSysUtils::read16le(&dir_entry[0x1a]);
    if (logging) {
      log.println(" - first cluster {:#010x}", cluster);
    }

    log.print("Checking contents of file {}.... ", file);

    // Read the cluster chain.
    uint32_t fatBlock = (cluster * 4) >> bytesPerSectorShift;
    sd.read_blocks(blk_offset + fatOffset + fatBlock, dataBuffer, 1u);
    const uint32_t end_cluster = 0x0fffffffu;
    uint32_t sampleOffset      = 0u;
    do {
      // Show this cluster.
      uint32_t blk = ((cluster - 2) << secsPerClusterShift);  // cluster_to_sec();
      while (fileLen > 0u) {
        fs.file_read(blk, fileBuffer, 1u << secsPerClusterShift,

                     blk_offset + clusterHeapOffset, secsPerClusterShift, sd, log);

        uint32_t chunkLen = (fileLen >= sizeof(fileBuffer)) ? sizeof(fileBuffer) : fileLen;
        if (logging) {
          dump_bytes(fileBuffer, chunkLen, log);
        }
        // Compare this data against the sample text.
        failures += compare_bytes(lorem_text, sampleOffset, fileBuffer, chunkLen, log);
        fileLen -= chunkLen;
        blk++;
      }

      // Read the next cluster.
      cluster = fileSysUtils::read32le(&dataBuffer[(cluster * 4) & ((1 << bytesPerSectorShift) - 1)]);
      if (logging) {
        log.println(" - next cluster {:#010x}", cluster);
      }
      if (cluster != end_cluster) {
        uint32_t block = (cluster * 4) >> bytesPerSectorShift;
        if (block != fatBlock) {
          fatBlock = block;
          sd.read_blocks(blk_offset + fatOffset + fatBlock, dataBuffer, 1u);
        }
      }
    } while (cluster != end_cluster);
  } else {
    log.println("No valid Master Boot Record found (signature not detected)");
    failures++;
  }

  write_test_result(log, failures);

  check_result(log, !failures);

  // Be a good citizen and put the pinmux back in its default state.
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_cs, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_clk, EnableFirstInput);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::appspi_d0, EnableFirstInput);
  // Direct SPI controller 2 to drive the microSD pins.
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_dat3, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_clk, Disabled);
  Pinmux.output_pin_select(SonataPinmux::OutputPin::microsd_cmd, Disabled);
  // Select microSD CIPO as SPI controller input.
  constexpr uint8_t PmuxSpi0CipoToAppSpiD1 = 1;
  Pinmux.block_input_select(SonataPinmux::BlockInput::spi_0_cipo, PmuxSpi0CipoToAppSpiD1);
}
