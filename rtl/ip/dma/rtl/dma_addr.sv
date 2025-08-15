// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

module dma_addr #(
  // Memory/bus-related parameters.
  parameter  int unsigned AW,
  parameter  int unsigned ABIT,
  // Logical parameters, affecting max transfer size.
  parameter  int unsigned RowLenW,
  parameter  int unsigned RowsW,
  // Store permission required, rather than load?
  parameter  bit          Store
) (
  input                       clk_i,
  input                       rst_ni,

  input                       enable_i,

  // Control signals.
  input                       start_i,
  input                       abort_i,
  input                       pause_i,
  input                       error_i,

  // Configuration.
  input           [2*AW:ABIT] cfg_cap_i,
  input                 [1:0] cfg_colinc_i,
  input           [AW-1:ABIT] cfg_stride_i,
  input         [RowLenW-1:0] cfg_row_len_i,
  input           [RowsW-1:0] cfg_rows_i,

  input                       accepted_i,

  // Current state.
  output logic                active_o,
  output logic                paused_o,
  output logic                update_o,
  output logic    [AW-1:ABIT] addr_o,
  output logic  [RowLenW-1:0] row_len_o,
  output logic    [RowsW-1:0] rows_o,

  // Error indicators.
  output logic                err_tag_o,
  output logic                err_perms_o,
  output logic                err_bounds_o
);

  // Access to capability checks.
  import cheri_pkg::*;

  // We share a single decrementer among the row length and the row count.
  localparam int unsigned NextW = (RowLenW > RowsW) ? RowLenW : RowsW;

  // Start address from the configuration; this is just part of the capability and we must first
  // check whether we can use it.
  wire [AW-1:ABIT] cfg_addr = cfg_cap_i[AW-1:ABIT];

  // Current state.
  wire last_row = ~|rows_o;
  wire row_end = ~|row_len_o;
  wire last_word = last_row & row_end;

  // Capability to the data buffer.
  full_cap_t cap_full;
  reg_cap_t cap_reg;
  assign cap_reg = mem2regcap_fmt0(cfg_cap_i[2*AW:AW],
                                  {cfg_cap_i[2*AW], cfg_cap_i[AW-1:ABIT], {ABIT{1'b0}}}, 4'h0);
  // TODO: The address checking here becomes rather expensive and is probably
  // quite wasteful. Also likely to be beneficial to introduce a simple state machine here rather
  // than multiplexing the address.
  wire [AW:ABIT] chk_addr = {1'b0, start_i ? cfg_addr : addr_o};
  assign cap_full = reg2fullcap(cap_reg, {cfg_cap_i[AW-1:ABIT], {ABIT{1'b0}}});
  wire top_vio  = (chk_addr            > cap_full.top33[AW   :ABIT]);
  wire base_vio = (chk_addr[AW-1:ABIT] < cap_full.base32[AW-1:ABIT]);
  logic [cheri_pkg::PERMS_W-1:0] perms;
  assign perms = cheri_pkg::expand_perms(cap_reg.cperms);
  // Have we got a capability?
  wire cap_tag = cfg_cap_i[2*AW];
  // Have we got the appropriate permissions?
  wire cap_perm = Store ? perms[cheri_pkg::PERM_SD] : perms[cheri_pkg::PERM_LD];
  // Is the access address within the bounds of the capability?
  wire cap_bounds = ~|{base_vio, top_vio};
  // Capability valid?
  wire cap_valid = &{cap_tag, cap_perm, cap_bounds};
  // Final request issued?
  wire completed = accepted_i & last_word;
  // Aborting upon request or as a result of an error.
  wire aborting = abort_i | (active_o & !cap_bounds);

  // Activity.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) active_o <= 1'b0;
    else if (enable_i) begin
      // Starting has the highest priority.
      if (start_i) begin
        // Check the capability and the transfer properties.
        active_o <= cap_valid;
      end else begin
        // Aborting has the next highest priority.
        if (|{aborting, error_i, pause_i, completed}) active_o <= 1'b0;
      end
      // Pausing is the lowest priority control signal.
      paused_o  <= pause_i & ~|{start_i, abort_i, error_i};
    end
  end

  // Error indicators.
  assign err_tag_o    = &{enable_i, start_i, !cap_tag};
  assign err_perms_o  = &{enable_i, start_i,  cap_tag, !cap_perm};
  assign err_bounds_o = &{enable_i, active_o, cap_tag,  cap_perm, !cap_bounds};

  // Address increment after word transfer (=column increment).
  // 00: Do not modify; fixed address for the entire row.
  // 01: Add one word.
  // 10: Subtract two words (unlikely to be useful).
  // 11: Subtract one word.
  wire [AW-1:ABIT] addr_inc = {{(AW-ABIT-1){cfg_colinc_i[1]}}, cfg_colinc_i[0]};

  // Addressing arithmetic.
  wire [NextW-1:0] next = (row_end ? NextW'(rows_o) : NextW'(row_len_o)) - 'b1;

  always_ff @(posedge clk_i) begin
    if (enable_i & start_i) begin
      addr_o    <= cfg_addr;
      row_len_o <= cfg_row_len_i;
      rows_o    <= cfg_rows_i;
    end else if (accepted_i) begin
      // Advance the address.
      addr_o    <= addr_o + (row_end ? cfg_stride_i : addr_inc);
      // Update count of words within the line.
      row_len_o <= row_end ? cfg_row_len_i : next[RowLenW-1:0];
      // Advance to the next row?
      if (row_end) rows_o <= next[RowsW-1:0];
    end
  end

  // Report current status, when pausing or aborting.
  assign update_o = (aborting | pause_i) & !start_i;

  // Some unused signals.
  logic unused;
  assign unused = ^{cap_full, cap_reg,
                    abort_i, pause_i};  // TODO: Not yet implemented.
endmodule
