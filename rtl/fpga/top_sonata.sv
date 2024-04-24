// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Sonata system top level for the Sonata PCB
module top_sonata (
  input  logic mainClk,
  input  logic nrst,

  output logic [7:0] usrLed,
  output logic       led_bootok,
  output logic       led_halted,
  output logic       led_cheri,
  output logic       led_legacy,
  output logic [8:0] cheriErr,

  input  logic [4:0] navSw,
  input  logic [7:0] usrSw,

  output logic       lcd_rst,
  output logic       lcd_dc,
  output logic       lcd_copi,
  output logic       lcd_clk,
  output logic       lcd_cs,
  output logic       lcd_backlight,

  output logic       rgbled0,

  output logic       ser0_tx,
  input  logic       ser0_rx,

  // I2C buses
  inout  logic       scl0,
  inout  logic       sda0,

  output logic       scl1,
  output logic       sda1,

  // Status input from USB transceiver
  input  logic       usrusb_vbusdetect,

  // Control of USB transceiver
  output logic       usrusb_softcn,
  // Configure the USB transceiver for Full Speed operation.
  output logic       usrusb_spd,

  // Reception from USB host via transceiver
  input  logic       usrusb_v_p,
  input  logic       usrusb_v_n,
  input  logic       usrusb_rcv,

  // Transmission to USB host via transceiver
  output logic       usrusb_vpo,
  output logic       usrusb_vmo,

  // Always driven configuration signals to the USB transceiver.
  output logic       usrusb_oe,
  output logic       usrusb_sus,

  input  logic       tck_i,
  input  logic       tms_i,
  input  logic       td_i,
  output logic       td_o,

  // USB monitoring (logic analyzer attachment).
  output logic       usbmon_vbus,
  output logic       usbmon_dp,
  output logic       usbmon_dm
);
  // System clock frequency.
  parameter int SysClkFreq = 25_000_000;

  parameter SRAMInitFile = "";

  // Main system clock and reset
  logic main_clk_buf;
  logic clk_sys;
  logic rst_sys_n;

  // USB device clock and reset
  logic clk_usb;
  wire  rst_usb_n = rst_sys_n;

  logic [7:0] reset_counter;
  logic pll_locked;
  logic rst_btn;

  logic [4:0] nav_sw_n;
  logic [7:0] user_sw_n;

  assign led_bootok = rst_sys_n;

  // Switch inputs have pull-ups and switches pull to ground when on. Invert here so CPU sees 1 for
  // on and 0 for off.
  assign nav_sw_n = ~navSw;
  assign user_sw_n = ~usrSw;

  assign usrusb_spd = 1'b1;  // Full Speed operation.

  logic dp_en_d2p;
  logic rx_enable_d2p;
  assign usrusb_oe  = !dp_en_d2p;  // Active low Output Enable.
  assign usrusb_sus = !rx_enable_d2p;

  logic cheri_en;

  logic scl0_o, scl0_oe;
  logic sda0_o, sda0_oe;

  logic scl1_o, scl1_oe;
  logic sda1_o, sda1_oe;

  // Open Drain drivers onto I2C bus.
  assign scl0 = scl0_oe ? scl0_o : 1'bZ;
  assign sda0 = sda0_oe ? sda0_o : 1'bZ;

  assign scl1 = scl1_oe ? scl1_o : 1'bZ;
  assign sda1 = sda1_oe ? sda1_o : 1'bZ;

  // USB monitoring; run the input signals through a couple of flops for metastability
  // avoidance, and delay the outbound traffic by the same number of cycles before using
  // the delayed output enable to switch cleanly at 48MHz.
  logic usrusb_vbus_q,  usrusb_v_p_q,  usrusb_v_n_q;
  logic usrusb_vbus_qq, usrusb_v_p_qq, usrusb_v_n_qq;
  logic usrusb_oe_q,    usrusb_vpo_q,  usrusb_vmo_q;
  logic usrusb_oe_qq,   usrusb_vpo_qq, usrusb_vmo_qq;

  always_ff @(posedge clk_usb or negedge rst_usb_n) begin
    if (!rst_usb_n) begin
      {usrusb_vbus_q,  usrusb_v_p_q,  usrusb_v_n_q } <= 'b0;
      {usrusb_vbus_qq, usrusb_v_p_qq, usrusb_v_n_qq} <= 'b0;
      {usrusb_oe_q,    usrusb_vpo_q,  usrusb_vmo_q } <= 'b0;
      {usrusb_oe_qq,   usrusb_vpo_qq, usrusb_vmo_qq} <= 'b0;
    end else begin
      // Inputs from differential receiver.
      {usrusb_vbus_q,  usrusb_v_p_q,  usrusb_v_n_q } <= {usrusb_vbusdetect, usrusb_v_p, usrusb_v_n};
      {usrusb_vbus_qq, usrusb_v_p_qq, usrusb_v_n_qq} <= {usrusb_vbus_q, usrusb_v_p_q, usrusb_v_n_q};
      // Outputs from USB device.
      {usrusb_oe_q,    usrusb_vpo_q,  usrusb_vmo_q } <= {dp_en_d2p,   usrusb_vpo,   usrusb_vmo};
      {usrusb_oe_qq,   usrusb_vpo_qq, usrusb_vmo_qq} <= {usrusb_oe_q, usrusb_vpo_q, usrusb_vmo_q};
    end
  end
  // USB monitor signals to logic analyzer.
  assign usbmon_vbus = usrusb_vbus_qq;
  assign usbmon_dp   = usrusb_oe_qq ? usrusb_vpo_qq : usrusb_v_p_qq;
  assign usbmon_dm   = usrusb_oe_qq ? usrusb_vmo_qq : usrusb_v_n_qq;

  sonata_system #(
    .GpiWidth     ( 13           ),
    .GpoWidth     ( 12           ),
    .PwmWidth     (  0           ),
    .CheriErrWidth(  9           ),
    .SRAMInitFile ( SRAMInitFile )
  ) u_sonata_system (
    // Main system clock and reset
    .clk_sys_i      (clk_sys),
    .rst_sys_ni     (rst_sys_n),

    // USB device clock and reset
    .clk_usb_i      (clk_usb),
    .rst_usb_ni     (rst_usb_n),

    .gp_i           ({user_sw_n, nav_sw_n}),
    .gp_o           ({usrLed, lcd_backlight, lcd_dc, lcd_rst, lcd_cs}),

    .uart_rx_i      (ser0_rx),
    .uart_tx_o      (ser0_tx),

    .pwm_o(),

    .spi_rx_i       (1'b0),
    .spi_tx_o       (lcd_copi),
    .spi_sck_o      (lcd_clk),

    .cheri_err_o    (cheriErr),
    .cheri_en_o     (cheri_en),

    // I2C bus 0
    .i2c0_scl_i     (scl0),
    .i2c0_scl_o     (scl0_o),
    .i2c0_scl_en_o  (scl0_oe),
    .i2c0_sda_i     (sda0),
    .i2c0_sda_o     (sda0_o),
    .i2c0_sda_en_o  (sda0_oe),

    // I2C bus 1
    .i2c1_scl_i     (scl1),
    .i2c1_scl_o     (scl1_o),
    .i2c1_scl_en_o  (scl1_oe),
    .i2c1_sda_i     (sda1),
    .i2c1_sda_o     (sda1_o),
    .i2c1_sda_en_o  (sda1_oe),

    // Reception from USB host via transceiver
    .usb_dp_i         (usrusb_v_p),
    .usb_dn_i         (usrusb_v_n),
    .usb_rx_d_i       (usrusb_rcv),

    // Transmission to USB host via transceiver
    .usb_dp_o         (usrusb_vpo),
    .usb_dp_en_o      (dp_en_d2p),
    .usb_dn_o         (usrusb_vmo),
    .usb_dn_en_o      (),

    // Configuration and control of USB transceiver
    .usb_sense_i      (usrusb_vbusdetect),
    .usb_dp_pullup_o  (usrusb_softcn),
    .usb_dn_pullup_o  (),
    .usb_rx_enable_o  (rx_enable_d2p),

    .tck_i,
    .tms_i,
    .trst_ni(rst_sys_n),
    .td_i,
    .td_o
  );

  assign led_cheri = cheri_en;
  assign led_legacy = ~cheri_en;
  assign led_halted = 1'b0;

  // Produce 50 MHz system clock from 25 MHz Sonata board clock.
  clkgen_sonata #(
    .SysClkFreq(SysClkFreq)
  ) u_clkgen(
    .IO_CLK    (mainClk),
    .IO_CLK_BUF(main_clk_buf),
    .clk_sys,
    .clk_usb,
    .locked    (pll_locked)
  );

  // Produce reset signal at beginning of time and when button pressed.
  assign rst_btn = ~nrst;

  rst_ctrl u_rst_ctrl (
    .clk_i       (main_clk_buf),
    .pll_locked_i(pll_locked),
    .rst_btn_i   (rst_btn),
    .rst_no      (rst_sys_n)
  );

  // Drive RGB LEDs to off state
  logic rgb_led_data_last;
  logic rgb_led_data_ack;
  logic rgb_led_data_out;

  always_ff @(posedge main_clk_buf or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      rgb_led_data_last <= 1'b0;
    end else begin
      if (rgb_led_data_ack) begin
        rgb_led_data_last <= ~rgb_led_data_last;
      end
    end
  end

  assign rgbled0 = ~rgb_led_data_out;

  ws281x_drv u_rgb_led_drv (
    .clk_i(main_clk_buf),
    .rst_ni(rst_sys_n),

    .go_i(1'b1),
    .idle_o(),
    .data_i({8'd0, 8'd0, 8'd0}),
    .data_valid_i(1'b1),
    .data_last_i(rgb_led_data_last),
    .data_ack_o(rgb_led_data_ack),
    .ws281x_dout_o(rgb_led_data_out)
  );

endmodule
