# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

// This is a memory copying function intended to exercise both the instruction memory system
// and the load/store data performance by performing a number of back-to-back transactions.
//
// Any permutation of the following may be in the HyperRAM:
// (i)   source data buffer
// (ii)  destination data buffer
// (iii) instruction sequence
// To ensure that data loads and stores are back-to-back, and to facilitate copying of this
// code into HyperRAM for execution, we employ handwritten assembler.
//
// Functions are aligned to 32-byte boundaries on the expectation that this is a multiple of
// the burst length and thus the code starts at the beginning of a read buffer. This just makes
// the performance measurements more robust against changes elsewhere in the code. All
// alignments are checked elsewhere.

  .section .text, "ax", @progbits

  .option norvc
  .globl hyperram_copy_block
  .p2align 5
  .type hyperram_copy_block,@function

// Controlled, performant copying routine that is position-independent and may be copied into
// the HyperRAM for in-place execution.
//
// ca0 -> destination (word-aligned)
// ca1 -> source (word-aligned)
// a2  =  number of bytes to copy
// return -> beyond destination data.
hyperram_copy_block:
  addi a2, a2, -32
  bltz a2, copy32fix
// TODO: cincaddr ca2, ca1, a2
// Ouroboros?
copy32:
  clw a3,   (ca1)
  clw a4,  4(ca1)
  clw a5,  8(ca1)
  clw t0, 12(ca1)
  csw a3,   (ca0)
  csw a4,  4(ca0)
  csw a5,  8(ca0)
  csw t0, 12(ca0)
  clw a3, 16(ca1)
  clw a4, 20(ca1)
  clw a5, 24(ca1)
  clw t0, 28(ca1)
  cincoffset ca1, ca1, 32
  csw a3, 16(ca0)
  csw a4, 20(ca0)
  csw a5, 24(ca0)
  csw t0, 28(ca0)
  cincoffset ca0, ca0, 32
// TODO: This is noob code!
  addi a2, a2, -32
  bgez a2, copy32
copy32fix:
  addi a2, a2, 28
  bltz a2, copy4fix
copy4:
  // TODO: Finish this with tail code.
copy4fix:
#  cl
  cret
# Dummy code to ensure that the above is enough to copy itself.
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
hyperram_copy_end:

// Size of the copying code, in bytes; the copying routine is itself copied into HyperRAM in
// order to exercise instruction and load/store traffic simultaneously.
  .globl hyperram_copy_size
hyperram_copy_size:
  .long hyperram_copy_end-hyperram_copy_block

