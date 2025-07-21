# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

// A set of memory initialisation functions using different access sizes, to check that
// the write transactions are properly coalesced into write bursts to the HyperBus
// Memory Controller.
//
// Each routine is expected to be writing a single, defined byte to each address that is
// modified. By initialising the entirety of a target buffer to a different value first,
// the set of modified addresses may be ascertained.

  .section .text, "ax", @progbits

  .option norvc

// Byte-based memory writing.
//
// entry ca0 -> byte-aligned destination buffer
//       a1   = byte to be stored
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_b
  .p2align 5
hyperram_memset_b:
  addi a2, a2, -8
  bltz a2, memset_b_8fix
memset_b_8:
  sb a1,  (ca0)
  sb a1, 1(ca0)
  sb a1, 2(ca0)
  sb a1, 3(ca0)
  sb a1, 4(ca0)
  sb a1, 5(ca0)
  sb a1, 6(ca0)
  sb a1, 7(ca0)
  cincoffset ca0, ca0, 8
  addi a2, a2, -8
  bgez a2, memset_b_8
memset_b_8fix:
  addi a2, a2, 7
  bgez a2, memset_b_fin
  // Just complete the request using byte stores.

  addi a2, a2, -1
  
memset_b_fin:
  cret

// Byte and half word-based memory writing; each word is written using
// 2 byte stores and a half-word store.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_hb
  .p2align 5
hyperram_memset_hb:
  addi a2, a2, -4
  bltz a2, memset_hb_4fix
memset_hb_4:
  sb a1,  (ca0)
  sb a1, 1(ca0)
  sh a1, 2(ca0)
  cincoffset ca0, ca0, 4
  addi a2, a2, -4
  bgez a2, memset_hb_4
memset_hb_4fix:
  cret

// Half word-based memory writing.
//
// entry ca0 -> half-word aligned destination buffer
//       a1   = byte to be stored, replicated throughout half-word
//       a2   = number of bytes
// exit  -
  .globl hyperram_memset_h
  .p2align 5
hyperram_memset_h:
  addi a2, a2, -16
  bltz a2, memset_h_16fix
memset_h_16:
  sh a1,   (ca0)
  sh a1,  2(ca0)
  sh a1,  4(ca0)
  sh a1,  6(ca0)
  sh a1,  8(ca0)
  sh a1, 10(ca0)
  sh a1, 12(ca0)
  sh a1, 14(ca0)
  cincoffset ca0, ca0, 16
  addi a2, a2, -16
  bgez a2, memset_h_16
memset_h_16fix:
  cret

// Word-based memory writing.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_w
  .p2align 5
hyperram_memset_w:
  addi a2, a2, -32
  bltz a2, memset_w_32fix
memset_w_32:
  sw a1,   (ca0)
  sw a1,  4(ca0)
  sw a1,  8(ca0)
  sw a1, 12(ca0)
  sw a1, 16(ca0)
  sw a1, 20(ca0)
  sw a1, 24(ca0)
  sw a1, 28(ca0)
  cincoffset ca0, ca0, 32
  addi a2, a2, -32
  bgez a2, memset_w_32
memset_w_32fix:
  cret

// Repeated words memory writing; the performance of this code is of no consequence. It is
// concerned purely with ensuring the correctness of the written data.
//
// entry ca0 -> word-aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_wr
  .p2align 5
hyperram_memset_wr:
  addi a2, a2, -4
  bltz a2, memset_wr_4fix
  xor a3, a1, -1
memset_wr_4:
  // This word should be overwritten...
  sw a3, (ca0)
  // ...by the original value.
  sw a1, (ca0)
  cincoffset ca0, ca0, 4
  addi a2, a2, -4
  bgez a2, memset_wr_4
memset_wr_4fix:
  cret

// Word-based memory writing to descending addresses.
//
// entry ca0 -> word-aligned end of destination buffer, exclusive
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_wd
  .p2align 5
hyperram_memset_wd:
  addi a2, a2, -8
  bltz a2, memset_wd_8fix
memset_wd_8:
  sw a1, -4(ca0)
  sw a1, -8(ca0)
  cincoffset ca0, ca0, -8
  addi a2, a2, -8
  bgez a2, memset_wd_8
memset_wd_8fix:
  cret

// Capability stores to ascending addresses.
//
// entry ca0 -> double-word aligned destination buffer
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_c
  .p2align 5
hyperram_memset_c:
  // Replicate the data word to yield a double word.
  cincoffset csp, csp, -8
  sw a1,  (csp)
  sw a1, 4(csp)
  clc ca1,(csp)
  cincoffset csp, csp, 8
  addi a2, a2, -8
  bltz a2, memset_c_8fix
memset_c_8:
  csc ca1, (ca0)
  cincoffset ca0, ca0, 8
  addi a2, a2, -8
  bgez a2, memset_c_8
memset_c_8fix:
  cret

// Capability stores to descending addresses.
//
// entry ca0 -> double-word aligned end of destination buffer, exclusive
//       a1   = byte to be stored, replicated throughout word
//       a2   = number of bytes
  .globl hyperram_memset_cd
  .p2align 5
hyperram_memset_cd:
  // Replicate the data word to yield a double word.
  cincoffset csp, csp, -8
  sw a1,  (csp)
  sw a1, 4(csp)
  clc ca1,(csp)
  cincoffset csp, csp, 8
  addi a2, a2, -8
  bltz a2, memset_cd_8fix
memset_cd_8:
  csc ca1, -8(ca0)
  cincoffset ca0, ca0, -8
  addi a2, a2, -8
  bgez a2, memset_cd_8
memset_cd_8fix:
  cret

