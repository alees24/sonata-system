// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// A port provides read access to the HyperRAM, and optionally write access too.
// It retains up to a full burst of read data and must maintain coherency with any write traffic
// in the event that writes are supported.
// Instruction ports need not support write operations.
module hbmc_tl_port import tlul_pkg::*; #(
  parameter int unsigned HyperRAMAddrW = 20,
  // log2(burst length in bytes)
  parameter int unsigned Log2BurstLen = 4,
  // TODO: We probably want to widen this!
  parameter int unsigned RamDW = 32,
  // Does this port need to support write operations?
  parameter bit SupportWrites = 1
) (
  input                             clk_i,
  input                             rst_ni,

  // Constant indicating port number.
  input                             portid_i,

  // TL-UL interface.
  input  tl_h2d_t                   tl_i,
  output tl_d2h_t                   tl_o,

  // Interface to the write buffer
  // TODO: There is no write buffer...
  //input  logic [] wrbuf_base,
  //input  logic [] wrbuf_valid,
  //wrbuf_req
  //wrbuf_gnt
  //output logic [] wrbuf_data
  //wrbuf_flush

  // Command data to the HyperRAM controller; command, address and burst length
  output logic                      cmd_wvalid,
  input                             cmd_wready,
  output        [HyperRAMAddrW-1:1] cmd_mem_addr,
  output                      [6:0] cmd_word_cnt,
  output logic                      cmd_wr_not_rd,
  output                            cmd_wrap_not_incr,
  output                      [3:0] cmd_seq,
  output logic                      tag_cmd_req,
  input                             tag_cmd_wready,
  output                            tag_cmd_wcap,
  output logic                      dfifo_wr_ena,
  input                             dfifo_wr_full,
  output      [top_pkg::TL_DBW-1:0] dfifo_wr_strb,
  output       [top_pkg::TL_DW-1:0] dfifo_wr_din,

  // Read data from the HyperRAM
  // TODO: We probably want to change this interface once stable.
  output logic                      ufifo_rd_ena,
  input                             ufifo_rd_empty,
  input                 [RamDW-1:0] ufifo_rd_dout,
  input                       [3:0] ufifo_rd_seq,
  input                             ufifo_rd_last,

  // Tag read data interface.
  output                            tag_rdata_rready,
  input                             tl_tag_bit
);

//`define HYPERRAM_BUFFER_WORDS 8  // Burst length = 8 x 32-bit words.
`define HYPERRAM_BUFFER_WORDS 4  // Burst length = 4 x 32-bit words.

  // TODO: We perhaps want to accept and re-time the request to limit the combinational logic
  // presented to the TL-UL fabric A channel.
  wire rd_req = tl_i.a_valid & (tl_i.a_opcode == Get);
  wire wr_req = tl_i.a_valid & (tl_i.a_opcode == PutFullData || tl_i.a_opcode == PutPartialData);

/*----------------------------------------------------------------------------------------------------------------------------*/

// TODO: ratify/simplify this.
  logic can_issue;

  logic rdbuf_matches;  // Address matches within the read buffer.
  logic rdbuf_valid;    // Valid data is available within the read buffer.
  logic [3:0] rdbuf_seq;  // Sequence number of read buffer contents.
  logic [top_pkg::TL_DW-1:0] rdbuf_dout;

  // Invalidate the read buffer contents when a write occurs.
  wire rdbuf_invalidate = wr_req & rdbuf_matches;
  // Issue a new burst read if a read is performed outside of the current buffered address range.
  wire rdbuf_set = rd_req & ~rdbuf_matches
  & can_issue;
  wire rdbuf_hit = rd_req &  rdbuf_matches;
  wire rdbuf_re  = &{rd_req, rdbuf_matches, rdbuf_valid};  // Read data available.
// TODO: Think about the implications of stalling for the above signals.

// TODO:
  localparam int unsigned ABIT = $clog2(top_pkg::TL_DW / 8);
//localparam int unsigned BBIT = 5;
  localparam int unsigned BBIT = 4;

  // Read buffer retains up to a single burst of data read from the HyperRAM for this port;
  // the data arrives incrementally and may be returned as soon as it becomes available.
  hyperram_rdbuf #(
    .AW       (HyperRAMAddrW),
    .DW       (top_pkg::TL_DW),
    .BBIT     (BBIT),  // TODO: Derive this from a log2(max burst length) parameter of some sort.
    .RamDW    (RamDW)
  ) u_readbuf(
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),

    // Constant indicating port number.
    .portid_i         (portid_i),

    // Control/configuration.
    .invalidate_i     (rdbuf_invalidate),
    .set_i            (rdbuf_set),
    .addr_i           (tl_i.a_address[HyperRAMAddrW-1:ABIT]),
    .matches_o        (rdbuf_matches),
    .valid_o          (rdbuf_valid),
    .seq_o            (rdbuf_seq),

    // Reading from buffer.
    .read_i           (rdbuf_re),
    .roffset_i        (tl_i.a_address[BBIT-1:ABIT]),
    .rdata_o          (rdbuf_dout),

    // Writing into buffer.
    .write_i          (ufifo_rd_ena),
    .wseq_i           (ufifo_rd_seq),
    .wdata_i          (ufifo_rd_dout)
  );

/*----------------------------------------------------------------------------------------------------------------------------*/

  localparam int unsigned TL_REQ_FIFO_DEPTH = 4;
  localparam int unsigned TLReqFifoDepthW = prim_util_pkg::vbits(TL_REQ_FIFO_DEPTH+1);

  // Metadata from inbound tilelink transactions that needs to be saved to produce the response
  typedef struct packed {
    logic [top_pkg::TL_AIW-1:0] tl_source;
    logic [top_pkg::TL_SZW-1:0] tl_size;
    logic                       cmd_fetch;
    logic                       cmd_wr_not_rd;
  } tl_req_info_t;

  tl_req_info_t tl_req_fifo_wdata, tl_req_fifo_rdata;

  logic tl_req_fifo_wvalid, tl_req_fifo_wready;
  logic tl_req_fifo_rvalid, tl_req_fifo_rready;
  logic [TLReqFifoDepthW-1:0] tl_req_fifo_depth;

  // Reads from the buffer are issued into the TL request FIFO only when it is empty, because
  // otherwise we would need additional storage for the read data.
  logic tl_req_fifo_empty;
  assign tl_req_fifo_empty = ~|tl_req_fifo_depth;

  logic tl_a_ready;

  tl_d2h_t tl_o_int;

  // We can accept an incoming tilelink transaction when we've got space in the hyperram, tag
  // and tilelink request FIFOs. If we're taking in a write transaction we also need space in the
  // downstream FIFO(dfifo) for the write data.
  // If a read hits in the buffer but the data is not yet available, wait until it arrives
  // from the HyperRAM controller.
  //
  // TODO: If a read hits in the RAM we wait until the TL request FIFO is empty because we don't
  // have a FIFO for the read data itself; perhaps we should change this?
  assign can_issue = (tl_i.a_valid && tl_req_fifo_wready && tag_cmd_wready &&
                     ((rdbuf_re && tl_req_fifo_empty) || (!rdbuf_hit && cmd_wready)) &&
                     (tl_i.a_opcode == Get || ~dfifo_wr_full));

  // Logic for handling incoming tilelink requests
  // TODO: We perhaps want to accept and re-time the request to limit the combinational logic
  // presented to the TL-UL fabric A channel.
  always_comb begin
    cmd_wvalid         = 1'b0;
    cmd_wr_not_rd      = 1'b0;
    tag_cmd_req        = 1'b0;
    dfifo_wr_ena       = 1'b0;
    tl_req_fifo_wvalid = 1'b0;
    tl_a_ready         = 1'b0;

    if (can_issue) begin
      // Write to the relevant FIFOs and indicate ready on tilelink A channel
      cmd_wvalid         = !rdbuf_re;
      tag_cmd_req        = 1'b1;
      tl_req_fifo_wvalid = 1'b1;
      tl_a_ready         = 1'b1;

      if (tl_i.a_opcode != Get) begin
        cmd_wr_not_rd    = 1'b1;
        dfifo_wr_ena     = 1'b1;
      end
    end
  end

  assign dfifo_wr_strb = tl_i.a_mask;
  assign dfifo_wr_din  = tl_i.a_data;

  assign tl_req_fifo_wdata = '{
    tl_source     : tl_i.a_source,
    tl_size       : tl_i.a_size,
    cmd_fetch     : cmd_wvalid,
    cmd_wr_not_rd : cmd_wr_not_rd
  };

  // We decant the read data from the 'Upstream FIFO' into the read buffer as soon as possible,
  // both to prevent the FIFO from overflowing and to avoid holding up other read ports.
  assign ufifo_rd_ena = (ufifo_rd_seq[3] == portid_i) & ~ufifo_rd_empty;

  // Track the reading of bursts from the HyperRAM controller.
  logic ufifo_rd_bursting;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ufifo_rd_bursting <= 1'b0;
    end else if (ufifo_rd_ena & (ufifo_rd_last | ~ufifo_rd_bursting))
      ufifo_rd_bursting <= !ufifo_rd_last;
  end

  // First word of data returned as part of a burst read operation.
  logic [top_pkg::TL_DW-1:0] ufifo_dout_first;
  assign ufifo_dout_first = ufifo_rd_dout[top_pkg::TL_DW-1:0];

  // If the data from the read buffer is not accepted immediately by the host we must register it
  // to prevent it being invalidated by another read.
  logic rdbuf_valid_q;
  logic [top_pkg::TL_DW-1:0] rdbuf_dout_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rdbuf_valid_q <= 1'b0;
    end else if (tl_o_int.d_valid) begin
      if (tl_i.d_ready) rdbuf_valid_q <= 1'b0;  // Response sent.
      else begin
        // Capture read data and keep it stable until it is accepted by the host.
        rdbuf_valid_q <= !tl_req_fifo_rdata.cmd_wr_not_rd;
        if (!rdbuf_valid_q) begin
          rdbuf_dout_q <= tl_req_fifo_rdata.cmd_fetch ? ufifo_dout_first : rdbuf_dout;
        end
      end
    end
  end

  // Logic for sending out TileLink responses.
  // - write responses may be sent as soon as the host is ready to accept them.
  // - read responses may be sent as soon as the first word of data is available; we employ
  //   wrapping bursts to ensure that the first word of the burst is the one being requested
  //   by the TileLink host.  
  always_comb begin
    tl_o_int           = '0;
    if (tl_req_fifo_rvalid) begin
      // We have an incoming request that needs a response
      if (tl_req_fifo_rdata.cmd_wr_not_rd) begin
        // If it's a write then return an immediate response (early response is reasonable as any
        // read that could observe the memory cannot occur until the write has actually happened)
        tl_o_int.d_valid   = 1'b1;
      end else begin
        // Otherwise wait until we have the first word of data to return.
        tl_o_int.d_valid   = |{ufifo_rd_ena & ~ufifo_rd_bursting,  // Initial word of burst read.
                               ~tl_req_fifo_rdata.cmd_fetch,  // From read buffer.
                               rdbuf_valid_q};  // Holding read data stable until accepted.
      end
    end

    tl_o_int.d_opcode          = tl_req_fifo_rdata.cmd_wr_not_rd ? AccessAck : AccessAckData;
    tl_o_int.d_size            = tl_req_fifo_rdata.tl_size;
    tl_o_int.d_source          = tl_req_fifo_rdata.tl_source;
    tl_o_int.d_data            = rdbuf_valid_q ? rdbuf_dout_q :
                                (tl_req_fifo_rdata.cmd_fetch ? ufifo_dout_first : rdbuf_dout);
    tl_o_int.d_user.capability = tl_tag_bit;
    tl_o_int.a_ready           = tl_a_ready;
  end

  // Complete the TL request as soon the response is accepted; this avoids the need to register
  // the properties of the response.
  assign tl_req_fifo_rready = tl_o_int.d_valid & tl_i.d_ready;

  // Discard the tag read data once the _read_ data is accepted.
  assign tag_rdata_rready = tl_o_int.d_valid & tl_i.d_ready & ~tl_req_fifo_rdata.cmd_wr_not_rd;

  // Generate integrity for outgoing response.
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(0),
    .EnableDataIntgGen(0)
  ) u_tlul_rsp_intg_gen (
    .tl_i(tl_o_int),
    .tl_o(tl_o)
  );

  // Queue of pending TileLink requests.
  prim_fifo_sync #(
    .Width($bits(tl_req_info_t)),
    .Depth(TL_REQ_FIFO_DEPTH),
    .Pass(1'b0)
  ) u_tl_req_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (1'b0),
    .wvalid_i (tl_req_fifo_wvalid),
    .wready_o (tl_req_fifo_wready),
    .wdata_i  (tl_req_fifo_wdata),
    .rvalid_o (tl_req_fifo_rvalid),
    .rready_i (tl_req_fifo_rready),
    .rdata_o  (tl_req_fifo_rdata),

    .full_o  (),
    .depth_o (tl_req_fifo_depth),
    .err_o   ()
  );

  // Command requests to the HyperRAM controller.
  assign cmd_mem_addr      = tl_i.a_address[HyperRAMAddrW-1:1];
  assign cmd_word_cnt      = cmd_wr_not_rd ? 7'd2 : 7'(`HYPERRAM_BUFFER_WORDS << 1);
  // Write bursts are linear, reads wrap. Since we're not performing extensive write bursts
  // presently, wrapping does not impact write behaviour, but using linear bursts on writes prevents
  // the HBMC from repeatedly changing the programmed burst length.
  assign cmd_wrap_not_incr = ~cmd_wr_not_rd;
  assign cmd_seq           = rdbuf_seq;

  assign tag_cmd_wcap = tl_i.a_user.capability;

endmodule

