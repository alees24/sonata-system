// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module dma
  import cheri_pkg::*;
  import top_pkg::*;
  import tlul_pkg::*; 
#(
  // Source IDs on TL-UL network.
  parameter bit [TL_AIW-1:0] SourceReadMin  = '0,
  parameter bit [TL_AIW-1:0] SourceReadMax  = '0,  
  parameter bit [TL_AIW-1:0] SourceWriteMin = '0,
  parameter bit [TL_AIW-1:0] SourceWriteMax = '0,
  // Enable support for CHERIoT capabilities.
  parameter bit CHERIoTEn = 1,
  // Design parameters, relating to capabilities.
  // TODO: This perhaps becomes a mode enumeration rather than a bit?
  parameter bit CopyCaps = 1,
  // Temporal Safety map.
  parameter bit [TL_AW-1:0] TSAddrRangeBase = '0,
  parameter bit [TL_AW-1:0] TSMapSize = 2048,
  // Logical parameters, affecting max transfer size.
  parameter int unsigned MaxRowLen = 'h10_0000,
  parameter int unsigned MaxRows = 'h1_0000,
  // Physical parameters.
  parameter int unsigned FIFODepth = 8
) (
  input           clk_i,
  input           rst_ni,

  // Register interface
  input  tl_h2d_t tl_i,
  output tl_d2h_t tl_o,

  // Host read port
  output tl_h2d_t host_rd_o,
  input  tl_d2h_t host_rd_i,

  // Host write port
  output tl_h2d_t host_wr_o,
  input  tl_d2h_t host_wr_i,

  // Interrupts
  output          intr_completed_o,
  output          intr_error_o
);

  import dma_reg_pkg::*;

  /* Open questions:
     - How big should the inter-row stride be?
     - Paused: should update all register state and allow it to be replaced.
   */

  localparam int WordSize = $clog2(top_pkg::TL_DBW);
  // LSB of word address.
  localparam int unsigned ABIT = WordSize;

  // Memory/bus-related parameters.
  localparam int unsigned AW  = top_pkg::TL_AW;
  localparam int unsigned DW  = top_pkg::TL_DW;

  localparam int unsigned FIFOWidth  = DW + 32'(CopyCaps);
  localparam int unsigned FIFODepthW = $clog2(FIFODepth + 1);

  // Note that the logic is programmed with 'n-1' to specify a row length of 'n' words,
  // and similarly for the row count.
  localparam int unsigned RowLenW = $clog2(MaxRowLen);
  localparam int unsigned RowsW = $clog2(MaxRows);

  dma_reg2hw_t reg2hw;
  dma_hw2reg_t hw2reg;

  logic unused_reg2hw;
  assign unused_reg2hw = |reg2hw;

  // Control signals
  wire start = reg2hw.control.start.qe & reg2hw.control.start.q;
  wire pause = reg2hw.control.pause.qe & reg2hw.control.pause.q;
  wire abort = reg2hw.control.abort.qe & reg2hw.control.abort.q;
  wire rd_enable = reg2hw.src_config.enable.q;
  wire wr_enable = reg2hw.dst_config.enable.q;
  // TODO: There needs to be a distinction here between a sw request and a control
  // signal into the read/write-side logic.
  // TODO:
  wire clr_error = 1'b0;

  // Status of read/write sides.
  logic rd_active, rd_paused, rd_idle;
  logic wr_active, wr_paused, wr_idle;
  // Error signalling.
  logic err_rd_bus, err_wr_bus;
  logic err_rd_cap_tag, err_wr_cap_tag;
  logic err_rd_cap_perms, err_wr_cap_perms;
  logic err_rd_cap_bounds, err_wr_cap_bounds;
  wire error = |{err_rd_bus, err_wr_bus,
                 err_rd_cap_tag, err_wr_cap_tag,
                 err_rd_cap_perms, err_wr_cap_perms,
                 err_rd_cap_bounds, err_wr_cap_bounds};

  // Register interface.
  dma_reg_top u_regs(
    .clk_i,
    .rst_ni,

    .reg2hw,
    .hw2reg,

    .tl_i,
    .tl_o
  );

  // Collect the 'valid' tags of the src and dst capabilities.
  // TODO: We cannot read them back as valid capabilities, but perhaps that's beneficial?
  wire src_addr_match = (tl_i.a_address[BlockAw-1:0] == DMA_SRC_CAP_LO_OFFSET) ||
                        (tl_i.a_address[BlockAw-1:0] == DMA_SRC_CAP_HI_OFFSET);
  wire dst_addr_match = (tl_i.a_address[BlockAw-1:0] == DMA_DST_CAP_LO_OFFSET) ||
                        (tl_i.a_address[BlockAw-1:0] == DMA_DST_CAP_HI_OFFSET);
  logic src_cap_valid, dst_cap_valid;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      src_cap_valid <= 1'b0;
      dst_cap_valid <= 1'b0;
    end else if (&{tl_i.a_valid, tl_o.a_ready, tl_i.a_opcode != Get}) begin
      if (src_addr_match) src_cap_valid <= tl_i.a_user.capability;
      if (dst_addr_match) dst_cap_valid <= tl_i.a_user.capability;
    end
  end

  wire rd_returned = host_rd_i.d_valid & wr_enable;
  // TODO: Check whether this is actually required!
  logic [TL_AIW-1:0] rd_source;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) rd_source <= SourceReadMin;
    else if (rd_accepted ^ rd_returned) begin
      // TODO:
      rd_source <= rd_accepted ? (rd_source + 'b1) : (rd_source - 'b1);
    end
  end

  // Control state.
  typedef enum {
    CtrlState_Idle = 0,
    CtrlState_Active,
    CtrlState_Aborting,
    CtrlState_Aborted,
    CtrlState_Pausing,
    CtrlState_Paused,
    CtrlState_Error
  } ctrl_state_e;

  ctrl_state_e ctrl_state;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) ctrl_state <= CtrlState_Idle;
    else if (error) ctrl_state <= CtrlState_Error;
    else begin
      case (ctrl_state)
        CtrlState_Idle: if (start) ctrl_state <= CtrlState_Active;
        CtrlState_Active:
          ctrl_state <= abort ? CtrlState_Aborting : // Abort requests have highest priority.
                       (pause ? CtrlState_Pausing :
                      ((wr_idle & rd_idle) ? CtrlState_Idle : CtrlState_Active));
        CtrlState_Pausing: if (wr_idle & rd_idle) ctrl_state <= CtrlState_Paused;
        CtrlState_Aborting: if (wr_idle & rd_idle) ctrl_state <= CtrlState_Aborted;
        CtrlState_Aborted,
        CtrlState_Paused: if (start) ctrl_state <= CtrlState_Active;
        CtrlState_Error: begin
          if (start) ctrl_state <= CtrlState_Active;
          else if (clr_error) ctrl_state <= CtrlState_Idle;
        end
        default: ctrl_state <= CtrlState_Idle;
      endcase
    end
  end

  // Issue of new requests shall _not_ occur when Pausing/Aborting.
  wire stopping = (ctrl_state == CtrlState_Pausing || ctrl_state == CtrlState_Aborting);

  // FIFO state.
  logic [FIFODepthW-1:0] fifo_committed;  // Number of committed entries.

  // Read requests.
  // TODO: We don't care about source IDs at the moment.
  wire rd_request  = rd_active & (fifo_committed < FIFODepthW'(FIFODepth)) & ~stopping;
  wire wr_accepted = host_wr_o.a_valid & host_wr_i.a_ready;
  wire rd_accepted = rd_request & host_rd_i.a_ready;

  // Track committed space.
  wire fifo_commit = rd_accepted;
  wire fifo_read = wr_accepted;
  wire [FIFODepthW-1:0] commit_delta = {{(FIFODepthW-1){fifo_read}}, 1'b1};
  always_ff @(posedge clk_i) begin
    if (start) fifo_committed <= '0;
    else if (fifo_commit ^ fifo_read) fifo_committed <= fifo_committed + commit_delta;
  end

  // The read side is idle once all words have been fetched and the FIFO has emptied.
  assign rd_idle = ~|{rd_active, fifo_committed};

  // Read addressing.
  logic               rd_update;
  logic   [AW-1:ABIT] rd_addr;
  logic [RowLenW-1:0] rd_row_len;
  logic   [RowsW-1:0] rd_rows;

  dma_addr #(
    .AW       (AW),
    .ABIT     (ABIT),
    .RowLenW  (RowLenW),
    .RowsW    (RowsW),
    .Store    (0)
  ) u_addr_rd (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    .enable_i       (rd_enable),

    // Control signals.
    .start_i        (start),
    .abort_i        (abort),
    .pause_i        (pause),
    .error_i        (err_rd_bus),

    // Configuration.
    .cfg_cap_i      ({src_cap_valid,
                      reg2hw.src_cap_hi.q,
                      reg2hw.src_cap_lo.q[AW-1:ABIT]}),
    .cfg_colinc_i   (reg2hw.src_config.col_inc.q),
    .cfg_stride_i   (reg2hw.src_stride.q[AW-1:ABIT]),
    .cfg_row_len_i  (reg2hw.src_row_len.q[RowLenW-1:0]),
    .cfg_rows_i     (reg2hw.src_rows.q[RowsW-1:0]),

    .accepted_i     (rd_accepted),

    // Current state.
    .active_o       (rd_active),
    .paused_o       (rd_paused),
    .update_o       (rd_update),
    .addr_o         (rd_addr),
    .row_len_o      (rd_row_len),
    .rows_o         (rd_rows),

    // Error indicators.
    .err_tag_o      (err_rd_cap_tag),
    .err_perms_o    (err_rd_cap_perms),
    .err_bounds_o   (err_rd_cap_bounds)
  );

  // Read port.
  always_comb begin
    host_rd_o = '0;
    host_rd_o.a_valid   = rd_request;
    host_rd_o.a_opcode  = Get;
    host_rd_o.a_size    = top_pkg::TL_SZW'(WordSize);
    host_rd_o.a_source  = rd_source;
    host_rd_o.a_address = {rd_addr, {ABIT{1'b0}}};
    host_rd_o.a_mask    = '1;
    host_rd_o.a_user.instr_type = prim_mubi_pkg::MuBi4False;

    host_rd_o.d_ready   = 1'b1;  // We don't over-commit the FIFO.
  end
  // Read error, e.g. invalid address.
  assign err_rd_bus = host_rd_i.d_valid & host_rd_i.d_error;

  // Write data into FIFO.
  wire [DW+32'(CopyCaps)-1:0] fifo_wdata = {host_rd_i.d_user.capability, host_rd_i.d_data};

  // Read data from FIFO.
  logic                   fifo_rvalid;
  logic [FIFOWidth-1:0] fifo_rdata;

  // Data FIFO.
  prim_fifo_sync #(
    .Width  (FIFOWidth),
    .Pass   (1'b0),
    .Depth  (FIFODepth)
  ) u_fifo(
    .clk_i,
    .rst_ni,

    .clr_i    (abort),

    // Write data (from read bus).
    .wvalid_i (host_rd_i.d_valid & wr_enable),
    .wready_o (),
    .wdata_i  (fifo_wdata),

    // Read data (to write bus).
    .rvalid_o (fifo_rvalid),
    .rready_i (host_wr_i.a_ready),
    .rdata_o  (fifo_rdata),

    // FIFO state.
    .full_o   (),
    .depth_o  (),
    .err_o    ()
  );

  // Write addressing.
  logic               wr_update;
  logic   [AW-1:ABIT] wr_addr;
  logic [RowLenW-1:0] wr_row_len;
  logic   [RowsW-1:0] wr_rows;

  dma_addr #(
    .AW       (AW),
    .ABIT     (ABIT),
    .RowLenW  (RowLenW),
    .RowsW    (RowsW),
    .Store    (1)
  ) u_addr_wr (
    .clk_i          (clk_i),
    .rst_ni         (rst_ni),

    .enable_i       (wr_enable),

    // Control signals.
    .start_i        (start),
    .abort_i        (abort),
    .pause_i        (pause),
    .error_i        (err_wr_bus),

    // Configuration.
    .cfg_cap_i      ({dst_cap_valid,
                      reg2hw.dst_cap_hi.q,
                      reg2hw.dst_cap_lo.q[AW-1:ABIT]}),
    .cfg_colinc_i   (reg2hw.dst_config.col_inc.q),
    .cfg_stride_i   (reg2hw.dst_stride.q[AW-1:ABIT]),
    .cfg_row_len_i  (reg2hw.dst_row_len.q[RowLenW-1:0]),
    .cfg_rows_i     (reg2hw.dst_rows.q[RowsW-1:0]),

    .accepted_i     (wr_accepted),

    // Current state.
    .active_o       (wr_active),
    .paused_o       (wr_paused),
    .update_o       (wr_update),
    .addr_o         (wr_addr),
    .row_len_o      (wr_row_len),
    .rows_o         (wr_rows),

    // Error indicators.
    .err_tag_o      (err_wr_cap_tag),
    .err_perms_o    (err_wr_cap_perms),
    .err_bounds_o   (err_wr_cap_bounds)
  );

  // Tracking of write completions; the width of this counter limits the number of pending write
  // operations but this should be more than sufficient without impacting throughput.
  logic [FIFODepthW:0] wr_pending;
  wire [FIFODepthW:0] wr_pend_delta = {{(FIFODepthW){host_wr_i.d_valid}}, 1'b1};
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_pending <= '0;
    end else if (wr_accepted ^ host_wr_i.d_valid)
      wr_pending <= wr_pending + wr_pend_delta;
  end

  // Request write to destination
  // - FIFO contains read data, or not reading.
  // - Have not reached the maximum number of pending writes; we need to monitor their
  //   completion and we can only count so many.
  // - No pause or abort request in progress.
  wire wr_request = &{wr_active, fifo_rvalid | ~rd_enable, ~&wr_pending, ~stopping};
  // The write side is idle once all writes have completed.
  assign wr_idle = ~|{wr_active, wr_pending};

  // Write port.
  always_comb begin
    host_wr_o = '0;
    host_wr_o.a_valid   = wr_request;
    host_wr_o.a_opcode  = PutFullData;
    host_wr_o.a_size    = top_pkg::TL_SZW'(WordSize);
    host_wr_o.a_source  = '0;
    host_wr_o.a_address = {wr_addr, {ABIT{1'b0}}};
    host_wr_o.a_mask    = '1;
    host_wr_o.a_data    = rd_enable ? fifo_rdata[DW-1:0] : reg2hw.src_cap_lo.q[DW-1:0];
    host_wr_o.a_user.capability = CopyCaps & rd_enable & fifo_rdata[DW];
    host_wr_o.a_user.instr_type = prim_mubi_pkg::MuBi4False;

    host_wr_o.d_ready   = 1'b1;
  end
  // Write error, e.g. invalid address.
  assign err_wr_bus = host_wr_i.d_valid & host_wr_i.d_error;

  // Single-cycle assertion upon detecting that both sides are idle.
  wire completed = (ctrl_state == CtrlState_Error) ||
                  &{ctrl_state == CtrlState_Active, rd_idle, wr_idle};

  always_comb begin
    hw2reg = '0;
    // Report hardware status.
    hw2reg.status.state.d      = 4'(ctrl_state);
    // Report internal hardware details.
    hw2reg.status.rd_active.d  = rd_active;
    hw2reg.status.rd_paused.d  = rd_paused;
    hw2reg.status.rd_idle.d    = rd_idle;
    hw2reg.status.wr_active.d  = wr_active;
    hw2reg.status.wr_paused.d  = wr_paused;
    hw2reg.status.wr_idle.d    = wr_idle;
    hw2reg.status.committed.d  = 8'(fifo_committed);
    hw2reg.status.wr_pending.d = 8'(wr_pending);

    // Update configuration registers with current state when pausing/aborting.
    hw2reg.src_cap_lo.de = rd_update;
    hw2reg.src_cap_lo.d = {rd_addr, {ABIT{1'b0}}};
    // Note: the low part of the source capability is never modified.
    hw2reg.src_row_len.de = rd_update;
    hw2reg.src_row_len.d = rd_row_len;
    hw2reg.src_rows.de = rd_update;
    hw2reg.src_rows.d = rd_rows;
    hw2reg.dst_cap_lo.de = wr_update;
    hw2reg.dst_cap_lo.d = {wr_addr, {ABIT{1'b0}}};
    // Note: the high part of the destination capability is never modified.
    hw2reg.dst_row_len.de = wr_update;
    hw2reg.dst_row_len.d = wr_row_len;
    hw2reg.dst_rows.de = wr_update;
    hw2reg.dst_rows.d = wr_rows;
  end

  // Completion interrupt.
  prim_intr_hw #(.Width(1), .IntrT("Event")) intr_hw_complete (
    .clk_i,
    .rst_ni,
    .event_intr_i           (completed),

    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.completed.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.completed.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.completed.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.completed.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.completed.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.completed.d),

    .intr_o                 (intr_completed_o)
  );

  // Error interrupt.
  prim_intr_hw #(.Width(1), .IntrT("Event")) intr_hw_error (
    .clk_i,
    .rst_ni,
    .event_intr_i           (error),

    .reg2hw_intr_enable_q_i (reg2hw.intr_enable.error.q),
    .reg2hw_intr_test_q_i   (reg2hw.intr_test.error.q),
    .reg2hw_intr_test_qe_i  (reg2hw.intr_test.error.qe),
    .reg2hw_intr_state_q_i  (reg2hw.intr_state.error.q),
    .hw2reg_intr_state_de_o (hw2reg.intr_state.error.de),
    .hw2reg_intr_state_d_o  (hw2reg.intr_state.error.d),

    .intr_o                 (intr_error_o)
  );

  // TODO: Stuff that has not yet been done.
  assign {rd_paused, wr_paused} = 'b0;

  logic unused;
  assign unused = ^{CHERIoTEn, TSAddrRangeBase, TSMapSize,
                    SourceReadMin, SourceReadMax, SourceWriteMin, SourceWriteMax,
                    host_rd_i, host_wr_i};

endmodule

