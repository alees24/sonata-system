/**
 * Copyright lowRISC contributors.
 * Licensed under the Apache License, Version 2.0, see LICENSE for details.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cheri.hh>

#include <assert.h>

#include "console.hh"
#include "sdcard-utils.hh"

/**
 * Very simple layer for read access to the files within the root directory
 * of a FAT32 or exFAT partition on an SD card.
 */
class fileSysUtils {
 private:
  Log *log;

  // Some SD cards seem to require this, and SPI mode transfers are always in
  // terms of 512-byte blocks anyway.
  static constexpr unsigned kBlockLen = 0x200u;

  // Properties of the FAT32/exFAT partition.
  uint32_t lbaOffset;
  uint32_t numSectors;
  uint32_t clusterHeapOffset;
  uint32_t clusterCount;
  uint32_t rootCluster;
  uint8_t bytesPerSectorShift;
  uint8_t secsPerClusterShift;

  uint8_t dataBuffer[kBlockLen];

  // Return the list of clusters holds with the given file/directory entry.
  void list_clusters() {}

 public:
  struct dirEntry {
    uint8_t entryType;
    uint8_t customDefined[0x13];
    uint32_t firstCluster;
    uint32_t dataLength;
  };

  fileSysUtils(Log *log_ = nullptr) : log(log_) {}

  // Test for the presence of an exFAT partition, read the partition properties
  // and then locate the cluster index and root directory.
  bool init(SdCard &sd) {
    // Read the Master Boot Record (MBR) from block 0 at the very start of the medium.
    if (!sd.read_blocks(0, dataBuffer, 1u)) {
      if (log) {
        log->println("Unable to read the MBR of the SD card");
      }
      return false;
    }

    if (dataBuffer[0x1fe] == 0x55 && dataBuffer[0x1ff] == 0xaa) {
      // The MBR describes up to four primary partitions.
      for (unsigned part = 0u; part < 4u; ++part) {
        // Each partition is described using 16 bytes.
        const unsigned partDesc = 0x1be + (part << 4);
        uint8_t part_type       = dataBuffer[partDesc + 4];
        // exFAT partitions are type 7.
        if (part_type == 0x07) {
          uint32_t lbaStart   = read32le(&dataBuffer[partDesc + 8]);
          uint32_t numSectors = read32le(&dataBuffer[partDesc + 12]);

          if (log) {
            log->println("Located EBR of exFAT partition at {}", lbaOffset);
            log->println("  {} sectors", numSectors);
          }
          // TODO:
          //          sd.read_blocks();

          if (dataBuffer[0] == 0xeb && dataBuffer[1] == 0x77 && dataBuffer[2] == 0x90) {
            // Read properties of the exFAT partition.
            clusterHeapOffset   = read32le(&dataBuffer[88]);
            rootCluster         = read32le(&dataBuffer[96]);
            bytesPerSectorShift = dataBuffer[108];
            secsPerClusterShift = dataBuffer[109];

            // We should now have access to the root directory when required.
            return true;
          }

          if (log) {
            log->println("Unable to parse the EBR of the SD card");
          }
          return false;
        }
      }
    }

    if (log) {
      log->println("Unable to parse the MBR of the SD card");
    }
    return false;
  }

  // Return a reference to the first file/directory entry within the root directory.
  void rootdir_first() {}

  //
  void rootdir_next() {}

  static int namecmp(const uint8_t *d, const char *s, size_t n) {
    while (n-- > 0) {
      if (*d++ != *s++) return 1;
    }
    return 0;
  }

  //
  void file_open() {}

  bool file_read(uint32_t blk, uint8_t *buf, size_t blks, uint32_t heapOffset, uint8_t secsPerClusterShift, SdCard &sd,
                 Log &log) {
    sd.read_blocks(heapOffset + blk, buf, 1u);  // blks
    return true;
  }

  void file_close() {}

  // TODO: perhaps we want not to fail if the cluster number is invalid?
  uint32_t exfat_cluster_to_block(uint32_t cluster) {
    // Check that we have sensible configuration.
    assert(clusterCount && rootCluster >= 2u && clusterHeapOffset);
    // Validate the requested cluster number.
    assert(cluster >= 2u && cluster < clusterCount);
  }

  // Read Cylinder, Head and Sector ('CHS address').
  static inline void read_chs(uint16_t &c, uint8_t &h, uint8_t &s, const uint8_t *p) {
    h = p[0];
    c = ((p[1] << 2) & 0x300u) | p[2];
    s = (p[1] & 0x3fu);
  }

  // Read 32-bit Little Endian word.
  static inline uint32_t read32le(const uint8_t *p) {
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  }

  // Read 16-bit Little Endian word.
  static inline uint16_t read16le(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
};
