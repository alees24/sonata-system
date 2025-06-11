// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// This is a basic functional model of a HyperRAM device for simulation,
// based on the W956D8MBYA.
//
// Presently it has the following limitations:
// - It does not model internal refresh operations so it has a fixed access latency.
module hyperram_W956(
  // Asynchronous reset.
  input       rstn,
  // Differential clocking (DDR).
  input       ckp,
  input       ckn,
  // Chip Select.
  input       csn,
  // Bidirectional read/write data strobe.
  inout       rwds,
  // Bidirectional data bus.
  inout [7:0] dq
);

  // HyperRAM storage.
  // TODO: Perhaps mirror the prim_ram_1p wrapper?
  logic [21:0] hram_addr;
  logic [15:0] hram_wdata;
  logic [15:0] hram_wmask;
  logic [15:0] hram_rdatap;
  logic [15:0] hram_rdatan;
  logic        hram_req;
  logic        hram_we;
  logic [1:0]  hram_be;

  // Expand the write strobes to bit level as required by the memory model.
  assign hram_wmask = {{8{hram_be[1]}}, {8{hram_be[0]}}};

  prim_generic_ram_2p #(
    .Width           (16),
    .Depth           ('h40_0000),
    .DataBitsPerMask (8),
    .MemInitFile     ()
  ) u_ram(
    // Q: Does it suffice to just drive both ports like this?
    .clk_a_i    (ckp),
    .clk_b_i    (ckn),

    .a_req_i    (hram_req),
    .a_write_i  (hram_we),
    .a_addr_i   (hram_addr),
    .a_wdata_i  (hram_wdata),
    .a_wmask_i  (hram_wmask),
    .a_rdata_o  (hram_rdatap),

    .b_req_i    (hram_req),
    .b_write_i  (hram_we),
    .b_addr_i   (hram_addr),
    .b_wdata_i  (hram_wdata),
    .b_wmask_i  (hram_wmask),
    .b_rdata_o  (hram_rdatan),

    .cfg_i      ('0)
   );

/* Port connections for the ram_1p
    .clk_i      (ckp),
    .cfg_i      ('0),

    .req_i      (hram_req),
    .write_i    (hram_we),
    .addr_i     (hram_addr[22:0]),
    .wdata_i    (hram_wdata),
    .wmask_i    (8'hff),
    .rdata_o    (hram_rdata)
 */

  // HyperRAM model; this is a very basic model, just enough us to allow us to simulate
  // actual usage. Note that the timing may not be accurate even then.
  typedef enum logic [3:0] {
    HRAM_Idle,
    HRAM_Cmd0,
    HRAM_Cmd1,
    HRAM_Cmd2,
    HRAM_Cmd3,
    HRAM_Cmd4,
    HRAM_Cmd5,
    HRAM_Latency1,
    HRAM_Latency2,
    HRAM_Writing,  // Actively collecting write data until CS raised.
    HRAM_Reading,  // Actively returning read data until CS raised.
    HRAM_Ignoring  // Ignoring register write.
  } hram_state_e;

  // TODO: Decide how to accommodate the access latency.
  logic hram_re;
  assign hram_we = &{hram_state == HRAM_Writing, !csn, !rwds};
  assign hram_re = &{hram_state == HRAM_Reading, !csn};
  assign hram_req = hram_we | hram_re;

  hram_state_e hram_state;
  logic [2:0] hram_cmd;
  logic [2:0] hram_lat;
  logic hram_lsb;  // LSByte within the current word?

  // Write data and strobes.
  assign hram_be[1] = &{hram_we, !hram_lsb};
  assign hram_be[0] = &{hram_we,  hram_lsb};
  assign hram_wdata = {2{dq}};

  // Read data path.
  logic hram_drv_oe;
  logic hram_rd_sel;
  logic hram_rd_lsb;
  logic [15:0] hram_rdata_pre;  // Selected rdata port.
  logic [7:0] hram_rdata;  // Selected byte; MS byte is returned first.

  // TODO: lose the extra complexity of the dual-port RAM?
  // assign hram_rdata_pre = hram_rd_sel ? hram_rdatan : hram_rdatap;
  // assign hram_rdata_pre = hram_rdatan;
  assign hram_rdata_pre = hram_rdatap;
  assign hram_rdata = hram_rd_lsb ? hram_rdata_pre[7:0] : hram_rdata_pre[15:8];

  // Address for next transfer.
  // TODO: Implement wrapping bursts.
  logic [21:0] next_addr;
  assign next_addr = hram_addr + 22'(hram_lsb);

  // HyperRAM model state machine.
  // Note: this logic is active on _both_ edges of _ckp, so it does not use _ckn.
  //
  // csn is used as an asynchronous reset because the clock is not running when
  // this active low CS signal is deasserted.
  always_ff @(edge ckp, posedge csn, negedge rstn) begin
    if (!rstn || csn) begin
      hram_state <= HRAM_Idle;
      hram_drv_oe <= 1'b0;
    end else begin
      case (hram_state)
// TODO: Sanitised clocking means do NOT ignore the first transition.
        HRAM_Idle, //: hram_state <= HRAM_Cmd0;
        HRAM_Cmd0: hram_state <= HRAM_Cmd1;
        HRAM_Cmd1: hram_state <= HRAM_Cmd2;
        HRAM_Cmd2: hram_state <= HRAM_Cmd3;
        HRAM_Cmd3: hram_state <= HRAM_Cmd4;
        HRAM_Cmd4: hram_state <= HRAM_Cmd5;
        HRAM_Cmd5: hram_state <= HRAM_Latency1;
        HRAM_Latency1: begin
          if (~|hram_lat) begin
            case (hram_cmd[2:1])
              2'b00:   hram_state <= HRAM_Writing;
              2'b10:   hram_state <= HRAM_Reading;
              default: hram_state <= HRAM_Ignoring;
            endcase
          end
        end
        HRAM_Ignoring,
        HRAM_Reading,
        HRAM_Writing: /* persistent until CS deasserted */;
        default: hram_state <= HRAM_Idle;
      endcase
      // if (hram_state != HRAM_Idle) $display("HR DATA 0x%02x", dq);
      case (hram_state)
        HRAM_Idle, HRAM_Cmd0: hram_cmd <= dq[7:5];
        HRAM_Cmd1: hram_addr[21:19]    <= dq[2:0];
        HRAM_Cmd2: hram_addr[18:11]    <= dq;
        HRAM_Cmd3: hram_addr[10:3]     <= dq;
        HRAM_Cmd4: /* reserved data */;
        HRAM_Cmd5: begin
          hram_addr[2:0] <= dq[2:0];
          hram_lsb       <= 1'b0;
          // Initialise latency counter.
          // TODO: This is supposed to come from a configuration register, I think.
          hram_lat       <= 'h5;
          // Read state.
          hram_rd_sel   <= 1'b1;
          hram_rd_lsb   <= 1'b0;
        end
        HRAM_Latency1: hram_lat <= hram_lat - 'b1;
        HRAM_Writing: begin
          // Advance the addressing; MS byte of each word is received first.
          hram_lsb  <= !hram_lsb;
          hram_addr  <= next_addr;
        end
        HRAM_Reading: begin
          // Advance the addressing; MS byte of each word is returned first.
          hram_lsb  <= !hram_lsb;
          hram_addr <= next_addr;
        end
        default: /* do nothing */;
      endcase
      hram_drv_oe <= hram_re;
      if (hram_drv_oe) begin
        hram_rd_lsb <= !hram_rd_lsb;  // Next byte within 16-bit word.
        hram_rd_sel <= hram_rd_sel ^ hram_rd_lsb;  // Advance after LSB.
      end
    end
  end

  // Read data from HyperRAM model.
  assign dq = hram_drv_oe ? hram_rdata : {8{1'bz}};
  // Read strobes from HyperRAM model.
  assign rwds = hram_drv_oe ? !hram_rd_lsb : 1'bz;

  logic unused_hyperram;
  assign unused_hyperram = ^{hram_addr, hram_cmd[0], hram_rdatan};

endmodule

