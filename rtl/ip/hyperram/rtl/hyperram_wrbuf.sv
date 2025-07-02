// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Write buffer retains a burst write whilst its under construction, coalescing the individual
// writes from the TL-UL bus since it does not support burst transfers.
//
// Note that the write buffer also supports read access for coherency between writes and reads;
// e.g. this is particularly important for data on the program stack.

module hyperram_wrbuf #(
  // System bus side.
  parameter int unsigned AW = 20,  // Width of address, bits.
  parameter int unsigned DW = 32,  // Width of data, bits.
  // HyperRAM side.
  // parameter int unsigned RamDW = 64,  // Width of data, bits.
  parameter int unsigned RamDW = 32,

  parameter int unsigned ABIT = $clog2(DW / 8),

  localparam int unsigned DBW = DW / 8,

  localparam int unsigned BufWords = 1 << (BBIT - ABIT)
) (
  input                   clk_i,
  input                   rst_ni,

  // Control.

  // Hit test for instruction access.
  input       [AW-1:ABIT] i_addr_i,
  output                  i_matches_o,
  output                  i_ready_o,

  // Hit test for data access.
  input                   d_req_i,
  output                  d_gnt_i,
  input                   d_write_i,
  input       [AW-1:ABIT] d_addr_i,
  output                  d_matches_o,
  input         [DBW-1:0] d_wmask_i,
  input          [DW-1:0] d_wdata_i,
  output         [DW-1:0] d_rdata_o,

  // Burst writes to the HyperRAM.
 output    hr_req_o,
 input     hr_gnt_i,
 output [] hr_addr_o,
 output [] hr_len_o,
 output [] hr_wstrb_o,
 output [] hr_wdata_o,

  input                   hr_read_i,
  output      [RamDw-1:0] hr_rdata_o
);

Writes into the buffer shall be accepted only if it's a wholesale replacement of any existing data
at that address. We have a single 'valid' bit for the entire 32-bit word?



We probably need to support out-of-order writes, which is perhaps problematic because we then
end up with discontiguous data.

csc 8(csp) <- stores to offset 8, offset 0xc.
csc 0(csp) <- surely stores out to the lower address first, offset 0x0 and then 0x4!


logic [] d_offset;
assign d_offset = d_addr_i[];

// Does the buffer have valid configuration?
logic configured;
// Base address of buffer contents.
logic [AW-1:BBIT] base_addr;
// Validity bit for each word within the buffer
logic []

// Write strobes for the most recent write.
logic [top_pkg::TL_DBW-1:0] latest_wr_strb;
logic [top_pkg::

// Hit test on buffer contents

always_ff @(posedge clk_i or negedge rst_ni) begin
  if () begin
  end else if () begin
  end
end

// Burst writing; the write data must be transferred to the HyperRAM controller _before_
// the command is issued.
always_ff @(posedge clk_i or negedge rst_ni) begin
  wr_state <= ;
  end else if () begin
    wr_state <= ;
  end
end

// So perhaps we want to collect write data within the...we can write out dummy words,
// but maybe we want to 


/* Collect partial word writes until we encounter:
   - a write that cannot be combined with the under-construction burst.
   - a timeout
 */

It takes time to write out burst data; if we get additional write traffic in the meantime,
  what are we to do? Stall the TL-UL? I suppose we should ping-pong an address bit on the RAM
  probably; leave the write data to trickle out...but then what if there's a read hit?

Write data must go into the FIFO before the command presumably, but we must arbitrate up front
  I guess...arbitrate, move write burst to the controller and then

Perhaps this is an argument for losing the dfifo entirely? We can then track two write bursts
  here?

We technically need to support two read hit tests simultaneously for the proposed architecture;

There is a danger of this becoming too complicated; perhaps a read hit should just flush the
  write out? Maybe we support data load but a read hit from the I port just causes flushing?


prim_ram_2p #(
  .Width    (RamDW),
  .Depth    (BufWords),

) u_buf(
  .clk_a_i    (clk_i),
  .clk_b_i    (clk_i),

  // Read/write port (TL-UL side).
  .a_req_i    (d_req_i),
  .a_write_i  (d_write_i),
  .a_addr_i   (d_offset),
  .a_wdata_i  (d_wdata_i),
  .a_wmask_i  (d_wmask_i),
  .a_rdata_o  (d_rdata_o),

  // Read port (HyperRAM side).
  .b_req_i    (hr_read_i),
  .b_write_i  (1'b0),
  .b_addr_i   (),
  .b_wdata_i  ('0),  // Read-only port.
  .b_wmask_i  ('0),
  .b_rdata_i  (hr_rdata_o),

  .cfg_i      ('0)
);

endmodule

