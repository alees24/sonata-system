// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Read buffer retains the contents of a read burst, on the premise that subsequent words within
// the burst will be required by the CPU in the near future.

module hyperram_rdbuf #(
  // System bus side.
  parameter int unsigned AW = 20, // Width of address, bits.
  parameter int unsigned DW = 32, // Width of data, bits.
  // HyperRAM side.
  // TODO: We shall likely need to widen this memory for throughput reasons.
  // parameter int unsigned RamDW = 64,  // Width of data, bits.
  parameter int unsigned RamDW = 32,
  // Burst size of 16 bytes
  parameter int unsigned BBIT = 4,  // 16 bytes/burst.

  // LSB of word address.
  localparam int unsigned ABIT = $clog2(DW / 8),
  // Size of read buffer in words of 'RamDW' bits.
  localparam int unsigned BufWords = 1 << (BBIT - ABIT)
) (
  input                    clk_i,
  input                    rst_ni,

  // Constant indicating port number.
  input                    portid_i,

  // Control.
  input                    invalidate_i,
  input                    set_i,
  // Hit test for read access.
  input        [AW-1:ABIT] addr_i,
  output                   matches_o,
  output                   valid_o,
  output logic       [3:0] seq_o,

  // Reading from the buffer.
  input                    read_i,
  input      [BBIT-1:ABIT] roffset_i,
  output          [DW-1:0] rdata_o,

  // Writing to the buffer (HyperRAM side).
  input                    write_i,
  input              [3:0] wseq_i,
  input        [RamDW-1:0] wdata_i
);

// Does the buffer have valid information?
logic configured;
// Base address of buffer contents.
logic [AW-1:BBIT] base_addr;
logic [BBIT-1:ABIT] woffset;

// Hit test on buffer contents.
logic [BBIT-1:ABIT] aoffset;
assign aoffset = addr_i[BBIT-1:ABIT];  // Address bits selecting word within burst.
// Indicates that the address matches within this read buffer.
assign matches_o = &{configured, addr_i[AW-1:BBIT] == base_addr[AW-1:BBIT]};
// When qualified with `matches_o` this indicates that the availability of valid data.
assign valid_o = valid[aoffset];

always_ff @(posedge clk_i or negedge rst_ni) begin
  if (!rst_ni) begin
    configured <= 1'b0;
  end else if (invalidate_i | set_i) begin
    configured <= set_i;
  end
end

// Sequence number for the buffer contents.
logic [2:0] next_seq;
logic [2:0] seq;
always_ff @(posedge clk_i) begin
  if (!rst_ni) seq <= '0;
  else if (set_i) seq <= next_seq;
end
// A single bit suffices for the sequence number.
assign next_seq = seq + 'b1;
// When issuing a new burst read, it's the new sequence number that is required, so that we
// accept the returned data.
assign seq_o = {portid_i, next_seq};

// Base address of buffer contents.
always_ff @(posedge clk_i) begin
  if (set_i) base_addr <= addr_i[AW-1:BBIT];
end

// Writes are accepted only if the buffer is still configured and the write data belongs in
// the currently-buffered content.
logic wr_accepted;
assign wr_accepted = &{write_i, configured, (wseq_i == {portid_i, seq})};

// Validity bits for buffer words.
logic [BufWords-1:0] valid;
always_ff @(posedge clk_i) begin
  if (invalidate_i | set_i) begin
    valid <= 'b0;
  end else if (wr_accepted) begin
    valid[woffset] <= 1'b1;
  end
end

// Writing into the buffer.
always_ff @(posedge clk_i) begin
  if (set_i) begin
    // Retain the offset of the first word that will be returned by the wrapping burst.
    woffset <= addr_i[BBIT-1:ABIT];
  end else begin
    // Wrapping bursts are achieved by overflowing at the end of the burst.
    woffset <= woffset + {{(BBIT-ABIT-1){1'b0}}, wr_accepted};
  end
end

// Use a dual-port implementation for simplicity because the design is targeting an FPGA
// implementation. Read-write collisions will be infrequent but we DO need to handle them.
prim_ram_2p #(
  .Width            (RamDW),
  .Depth            (BufWords),
  .DataBitsPerMask  (RamDW)  // We do not require partial writes.
) u_buf(
  .clk_a_i      (clk_i),
  .clk_b_i      (clk_i),

  // Read port (TL-UL side).
  .a_req_i      (read_i),
  .a_write_i    (1'b0),
  .a_addr_i     (roffset_i),
  .a_wdata_i    ('0),  // Read-only port.
  .a_wmask_i    ('0),
  .a_rdata_o    (rdata_o),

  // Write port (HyperRAM side).
  .b_req_i      (write_i),
  .b_write_i    (1'b1),
  .b_addr_i     (woffset),
  .b_wdata_i    (wdata_i),
  .b_wmask_i    ('1),
  .b_rdata_o    (),  // Write-only port.

  .cfg_i        ('0)
);

endmodule

