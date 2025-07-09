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

  .section .text, "ax", @progbits

  .option norvc
  .globl hyperram_copy_block
  .p2align 2
  .type hyperram_copy_block,@function

// Controlled, performant copying routine that is position-independent and may be copied into
// the HyperRAM for in-place execution.
//
// ca0 -> destination (word-aligned)
// ca1 -> source (word-aligned)
// a2  = number of bytes to copy from ca1 to ca0
// return -> beyond destination data.
hyperram_copy_block:
  addi a2, a2, -16
  bltz a2, copy16fix
copy16:
  clw a4, (ca1)
  clw a5, 4(ca1)
  clw t0, 8(ca1)
  clw t1, 12(ca1)
  csw a4, (ca0)
  csw a5, 4(ca0)
  csw t0, 8(ca0)
  csw t1, 12(ca0)
  cincoffset ca1, ca1, 16
  cincoffset ca0, ca0, 16
  addi a2, a2, -16
  bgez a2, copy16
copy16fix:
  addi a2, a2, 12
  bltz a2, copy4fix
copy4:

copy4fix:
#  cl
  cret
# Dummy code to ensure that the above is enough to copy itself.
  nop
  nop
  nop
  nop
hyperram_copy_end:

// Size of the copying code, in bytes.
  .globl hyperram_copy_size
hyperram_copy_size:
  .long hyperram_copy_end-hyperram_copy_block

DUMMY_CODE:
  lui a0, 0x1234
  cret

