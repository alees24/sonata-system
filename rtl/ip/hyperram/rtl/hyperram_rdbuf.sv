// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Read buffer retains the contents of a read burst, on the premise that subsequent words within
// the burst will be required by the CPU in the near future.
module hyperram_rdbuf #(
  // System bus side.
  parameter int unsigned AW = 20, // Width of address, bits.
  parameter int unsigned DW = 32, // Width of data, bits.
  parameter int unsigned DBW = 4,  // Number of update strobes.
  parameter int unsigned NumBufs = 4,  // Number of read buffers.
  parameter int unsigned PortIDWidth = 1, // Width of Port ID, bits.
  parameter int unsigned Log2MaxBufs = 2,
  parameter int unsigned SeqWidth = 6, // Width of sequence number, bits.
  // Burst size of 16 bytes
  parameter int unsigned BBIT = 4,  // 16 bytes/burst.

  // LSB of word address.
  localparam int unsigned ABIT = $clog2(DW / 8),
  // Size of read buffer in words of 'DW' bits.
  localparam int unsigned BufWords = 1 << (BBIT - ABIT)
) (
  input                       clk_i,
  input                       rst_ni,

  // Constant indicating port number.
  input     [PortIDWidth-1:0] portid_i,

//TODO:
input wr_notify_i,

  // Control.
  input                       invalidate_i,
  input                       set_i,
  // Hit test for read access.
  input           [AW-1:ABIT] addr_i,
  output                      matches_o,
  output                      valid_o,
  output logic [SeqWidth-1:0] seq_o,

  // Write notification test.
  input           [AW-1:ABIT] wr_notify_addr_i,
  output                      wr_matches_o,
  output                      wr_valid_o,

  // Updating the buffer (System bus side).
  input                       update_i,
  input         [BBIT-1:ABIT] uoffset_i,
  input             [DBW-1:0] umask_i,
  input              [DW-1:0] udata_i,

  // Reading from the buffer (System bus side).
  input                       read_i,
  input         [BBIT-1:ABIT] roffset_i,
  output logic       [DW-1:0] rdata_o,

  // Writing to the buffer (HyperRAM side).
  input                       write_i,
  input        [SeqWidth-1:0] wseq_i,
  input              [DW-1:0] wdata_i
);

// Round robin replacement of read buffers.
logic [NumBufs-1:0] buf_replace;
if (NumBufs > 1) begin : gen_round_robin_replace
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) buf_replace <= 'b1;
    else if (set_i) buf_replace <= {buf_replace[NumBufs-2:0], buf_replace[NumBufs-1]};
  end
end else begin
  assign buf_replace = 1'b0;
end

logic [NumBufs-1:0] configured;

// Decide upon the buffer to be replaced; if at least one is available, arbitrarily pick the
// lowest-numbered available buffer. If none is available then we use round-robin replacement.
wire [NumBufs-1:0] avail_msb = ~configured & ~(~configured - 'b1);
wire [NumBufs-1:0] buf_set = &configured ? buf_replace : avail_msb;

// Validity bits for buffer words.
logic [NumBufs-1:0][BufWords-1:0] valid;

// Base address of buffer contents.
logic [NumBufs-1:0][AW-1:BBIT] base_addr;

logic [NumBufs-1:0] matches_all;
logic [NumBufs-1:0] valid_all;
logic [NumBufs-1:0] wr_matches_all;
logic [NumBufs-1:0] wr_valid_all;

// TODO: Some of the parent logic surely wants to be imported.
assign matches_o    = |matches_all;
assign valid_o      = |valid_all;
assign wr_matches_o = |wr_matches_all;
assign wr_valid_o   = |wr_valid_all;

// TODO:
wire [NumBufs-1:0] u_valid_all = wr_notify_i ? wr_valid_all : valid_all;

// Invalidate occurs when matching but not valid and we need to know which valid signal to consult.
// TODO: Change the interface/redistribute the logic.
wire [NumBufs-1:0] invalidate = {NumBufs{invalidate_i}} & u_valid_all;
wire [NumBufs-1:0] set = {NumBufs{set_i}} & buf_set;

// Hit test on buffer contents.
logic [BBIT-1:ABIT] aoffset;
assign aoffset = addr_i[BBIT-1:ABIT];  // Address bits selecting word within burst.

// Return the bit index of the single bit set within the input; the output is undefined in the
// event of zero bits or more than one bit being set and shall not be used.
// This could only happen as a result of an internal logic error, with multiple buffers holding
// data for the same address.
function automatic logic [Log2MaxBufs-1:0] one_hot_enc(logic [NumBufs-1:0] in);
  logic [Log2MaxBufs-1:0] out = 0;
  for (int unsigned b = 0; b < Log2MaxBufs; b++) begin
    for (int unsigned i = 0; i < NumBufs; i++) out[b] = out[b] | (in[i] & i[b]);
  end
  return out;
endfunction

// Buffer to be read.
// TODO: This description is not the whole story!
wire [Log2MaxBufs-1:0] a_buf = one_hot_enc(valid_all);

// Write notification test; this is done in parallel with normal read buffer access because
// normally writes on other TL-UL ports will not collide with buffered read data.
logic [BBIT-1:ABIT] wn_offset;
assign wn_offset = wr_notify_addr_i[BBIT-1:ABIT];  // Address bits selecting word within burst.

// Buffer to be updated.
wire [Log2MaxBufs-1:0] ud_buf = one_hot_enc(u_valid_all);

// When receiving data from the HyperRAM controller, it arrives tagged with buffer number.
logic [Log2MaxBufs-1:0] wr_buf = wseq_i[3 +: Log2MaxBufs];
logic [NumBufs-1:0] wr_accepted;

logic [NumBufs-1:0][BBIT-1:ABIT] woffset;

// Sequence number is driven to non-zero only by the buffer which is about to be (re-)filled,
// so we can just OR all of the sequence numbers together.
logic [NumBufs-1:0][SeqWidth-1:0] seq_all;
always_comb begin
  seq_o = 0;
  for (int unsigned b = 0; b < NumBufs; b++) seq_o = seq_o | seq_all[b];
end

// The read buffer is capable of retaining a number of bursts of read data, so some of the
// control logic - chiefly the address matching - must be replicated for each of these
// internal buffers.
for (genvar b = 0; b < NumBufs; b++) begin : gen_buf_state
  // Does the buffer have valid information?
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      configured[b] <= 1'b0;
    end else if (invalidate[b] | set[b]) begin
      configured[b] <= set[b];
    end
  end

  // Indicates that the address matches within this read buffer.
  assign matches_all[b] = &{configured[b], addr_i[AW-1:BBIT] == base_addr[b][AW-1:BBIT]};
  // When qualified with `matches_o` this indicates that the availability of valid data.
  assign valid_all[b] = matches_all[b] & valid[b][aoffset];

  assign wr_matches_all[b] = &{configured[b], wr_notify_addr_i[AW-1:BBIT] == base_addr[b][AW-1:BBIT]};
  assign wr_valid_all[b] = wr_matches_all[b] & valid[b][wn_offset];

  // Sequence number for the buffer contents.
  logic [2:0] next_seq;
  logic [2:0] seq;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) seq <= '0;
    else if (set[b]) seq <= next_seq;
  end
  // A single bit suffices for the sequence number.
  assign next_seq = seq + 'b1;
  // When issuing a new burst read, it's the new sequence number that is required, so that we
  // accept the returned data.
  assign seq_all[b] = {SeqWidth{set[b]}} & {portid_i, b[Log2MaxBufs-1:0], next_seq};

  // Base address of buffer contents.
  always_ff @(posedge clk_i) begin
    if (set[b]) base_addr[b] <= addr_i[AW-1:BBIT];
  end

  // Writes are accepted only if the buffer is still configured and the write data belongs in
  // the currently-buffered content. The port ID number has already been checked; only the
  // buffer number and the sequence number matter here.
  assign wr_accepted[b] = &{write_i, configured[wr_buf], wr_buf == b,
                           (wseq_i[SeqWidth-1-PortIDWidth-Log2MaxBufs:0] == seq)};

  // Writing into the buffer.
  always_ff @(posedge clk_i) begin
    if (set[b]) begin
      // Retain the offset of the first word that will be returned by the wrapping burst.
      woffset[b] <= addr_i[BBIT-1:ABIT];
    end else begin
      // Wrapping bursts are achieved by overflowing at the end of the burst.
      woffset[b] <= woffset[b] + {{(BBIT-ABIT-1){1'b0}}, wr_accepted[b]};
    end
  end

  // Updating of validity bits.
  always_ff @(posedge clk_i) begin
    if (invalidate[b] | set[b]) begin
      valid[b] <= 'b0;
    end else if (wr_accepted[b]) begin
      valid[b][woffset[b]] <= 1'b1;
    end
  end
end

// RAM submodule wants a single write strobe per data line.
localparam int unsigned DataBitsPerMask = DW / DBW;
logic [DW-1:0] umask_full;
always_comb begin
  for (int unsigned b = 0; b < DBW; b++) begin
    umask_full[b*DataBitsPerMask +: DataBitsPerMask] = {DataBitsPerMask{umask_i[b]}};
  end
end

// Use a dual-port implementation for simplicity because the design is targeting an FPGA
// implementation. Read-write collisions will be infrequent but we DO need to handle them.
prim_ram_2p #(
  .Width            (DW),
  .Depth            (NumBufs * BufWords),
  .DataBitsPerMask  (DataBitsPerMask)
) u_buf(
  .clk_a_i      (clk_i),
  .clk_b_i      (clk_i),

  // Read port (TL-UL side).
  // - update and read shall not occur simultaneously; update takes precedence.
  .a_req_i      (read_i | update_i),
  .a_write_i    (update_i),
  .a_addr_i     (update_i ? {ud_buf, uoffset_i} : {a_buf, roffset_i}),
  .a_wdata_i    (udata_i),
  .a_wmask_i    (umask_full),
  .a_rdata_o    (rdata_o),

  // Write port (HyperRAM side).
  .b_req_i      (write_i),
  .b_write_i    (1'b1),
  .b_addr_i     ({wr_buf, woffset[wr_buf]}),
  .b_wdata_i    (wdata_i),
  .b_wmask_i    ('1),
  .b_rdata_o    (),  // Write-only port.

  .cfg_i        ('0)
);

logic unused;
assign unused = ^wseq_i;

endmodule

