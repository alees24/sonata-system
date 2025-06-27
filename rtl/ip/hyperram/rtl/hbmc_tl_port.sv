// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// A port provides read/write access to the HyperRAM, retaining read data and ensuring coherency
// with the write buffer.
//
// TODO: If we _do_ manage to support multiple ports, then we shall wants parameters for
// `burst length` and `write support`, and perhaps others.
module hbmc_tl_port import tlul_pkg::*; #(
  parameter int unsigned HyperRAMAddrW = 20
) (
  input                             clk_i,
  input                             rst_ni,

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
  input        [top_pkg::TL_DW-1:0] ufifo_rd_dout,
  input                             tl_tag_bit
);

//`define HYPERRAM_BUFFER
`define HYPERRAM_BUFFER_WORDS 32

/*----------------------------------------------------------------------------------------------------------------------------*/

  // Metadata from inbound tilelink transactions that needs to be saved to produce the response
  typedef struct packed {
    logic [top_pkg::TL_AIW-1:0] tl_source;
    logic [top_pkg::TL_SZW-1:0] tl_size;
    logic                       cmd_wr_not_rd;
  } tl_req_info_t;

  tl_req_info_t tl_req_fifo_wdata, tl_req_fifo_rdata;

  logic tl_req_fifo_wvalid, tl_req_fifo_wready;
  logic tl_req_fifo_rvalid, tl_req_fifo_rready;

  logic tl_a_ready;

  tl_d2h_t tl_o_int;

  // Logic for handling incoming tilelink requests
  always_comb begin
    cmd_wvalid         = 1'b0;
    cmd_wr_not_rd      = 1'b0;
    tag_cmd_req        = 1'b0;
    dfifo_wr_ena       = 1'b0;
    tl_req_fifo_wvalid = 1'b0;
    tl_a_ready         = 1'b0;

    if (tl_i.a_valid && tl_req_fifo_wready && tag_cmd_wready && cmd_wready &&
      (tl_i.a_opcode == Get || ~dfifo_wr_full)) begin
      // We can accept an incoming tilelink transaction when we've got space in the hyperram, tag
      // and tilelink request FIFOs. If we're taking in a write transaction we also need space in the
      // downstream FIFO(dfifo) for the write data

      // Write to the relevant FIFOs and indicate ready on tilelink A channel
      cmd_wvalid         = 1'b1;
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
    cmd_wr_not_rd : cmd_wr_not_rd
  };

  // Logic for sending out tilelink responses
  always_comb begin
    tl_o_int           = '0;
    tl_req_fifo_rready = 1'b0;
    ufifo_rd_ena       = 1'b0;

    if (tl_req_fifo_rvalid) begin
      // We have an incoming request that needs a response
      if (tl_req_fifo_rdata.cmd_wr_not_rd) begin
        // If it's a write then return an immediate response (early response is reasonable as any
        // read that could observe the memory cannot occur until the write has actually happened)
        tl_o_int.d_valid   = 1'b1;
        tl_req_fifo_rready = tl_i.d_ready;
      end else begin
        // Otherwise wait until we have read data to return
        tl_o_int.d_valid   = ~ufifo_rd_empty;
        // Only dequeue read data from the upstream FIFO (ufifo) and request FIFO when the tilelink
        // D channel is ready
        ufifo_rd_ena       = tl_i.d_ready & ~ufifo_rd_empty;
        tl_req_fifo_rready = ufifo_rd_ena;
      end
    end

    tl_o_int.d_opcode          = tl_req_fifo_rdata.cmd_wr_not_rd ? AccessAck : AccessAckData;
    tl_o_int.d_size            = tl_req_fifo_rdata.tl_size;
    tl_o_int.d_source          = tl_req_fifo_rdata.tl_source;
    tl_o_int.d_data            = ufifo_rd_dout;
    tl_o_int.d_user.capability = tl_tag_bit;
    tl_o_int.a_ready           = tl_a_ready;
  end

  // Generate integrity for outgoing response
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(0),
    .EnableDataIntgGen(0)
  ) u_tlul_rsp_intg_gen (
    .tl_i(tl_o_int),
    .tl_o(tl_o)
  );

  localparam TL_REQ_FIFO_DEPTH = 4;

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
    .depth_o (),
    .err_o   ()
  );

  assign cmd_mem_addr      = tl_i.a_address[HyperRAMAddrW-1:1];
`ifdef HYPERRAM_BUFFER
  assign cmd_word_cnt      = cmd_wr_not_rd ? 7'(`HYPERRAM_BUFFER_WORDS << 1) : 7'd2;
  assign cmd_wrap_not_incr = 1'b1;
`else
  assign cmd_word_cnt      = 7'd2;
  assign cmd_wrap_not_incr = 1'b0;
`endif

  assign tag_cmd_wcap = tl_i.a_user.capability;

endmodule

