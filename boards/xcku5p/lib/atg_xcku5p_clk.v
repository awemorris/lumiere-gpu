/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_xcku5p_clk: Clock and reset generation for the HamGeek XCKU5P
 * (xcku5p-ffvb676-2-i).
 *
 * Input: 200 MHz differential oscillator (sys_clk_p/n) -> IBUFDS ->
 * sys_ibuf (feeds MMCME4_BASE CLKIN1 and DDR4 c0_sys_clk_i directly without
 * intermediate BUFG), plus BUFG ref_clk for reset/control syncs.
 *
 * MMCME4_BASE (VCO = 200 MHz * 6.000 = 1200 MHz):
 *   CLKOUT0: 1200 / GCLK_DIVIDE -> gclk (default 3.000 -> 400.0 MHz)
 *   CLKOUT1: 1200 / 48 -> 25.00 MHz (640x480 pixel clock)
 *   CLKOUT2: 1200 / 16 -> 75.00 MHz (720p pixel clock)
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_xcku5p_clk #(
	parameter real GCLK_DIVIDE = 3.000	/* 1200 MHz / GCLK_DIVIDE (3 = 400 MHz) */
) (
	input wire sys_clk_p,
	input wire sys_clk_n,
	input wire cpu_reset,
	input wire pclk_sel,
	output wire gclk,
	output wire grst_n,
	output wire ref_ibuf,
	output wire ref_clk,
	output wire ref_rst_n,
	output wire pclk,
	output wire prst_n
);
	(* clock_buffer_type = "none" *) wire sys_ibuf;
	wire clkfb_out, clkfb_in, mmcm_locked;
	wire g_out, px_a, px_b;

	IBUFDS sys_buf (.I(sys_clk_p), .IB(sys_clk_n), .O(sys_ibuf));
	assign ref_ibuf = sys_ibuf;
	BUFG ref_bufg (.I(sys_ibuf), .O(ref_clk));

	MMCME4_BASE #(
		.BANDWIDTH("OPTIMIZED"),
		.CLKIN1_PERIOD(5.000),
		.DIVCLK_DIVIDE(1),
		.CLKFBOUT_MULT_F(6.000),
		.CLKOUT0_DIVIDE_F(GCLK_DIVIDE),
		.CLKOUT1_DIVIDE(48),
		.CLKOUT2_DIVIDE(16),
		.STARTUP_WAIT("FALSE")
	) mmcm_g (
		.CLKIN1(sys_ibuf),
		.CLKFBIN(clkfb_in),
		.RST(cpu_reset),
		.PWRDWN(1'b0),
		.CLKFBOUT(clkfb_out),
		.CLKFBOUTB(),
		.CLKOUT0(g_out),
		.CLKOUT0B(),
		.CLKOUT1(px_a),
		.CLKOUT1B(),
		.CLKOUT2(px_b),
		.CLKOUT2B(),
		.CLKOUT3(),
		.CLKOUT3B(),
		.CLKOUT4(),
		.CLKOUT5(),
		.CLKOUT6(),
		.LOCKED(mmcm_locked)
	);

	BUFG fb_bufg (.I(clkfb_out), .O(clkfb_in));
	BUFG g_bufg (.I(g_out), .O(gclk));
	BUFGMUX mux_px (.I0(px_a), .I1(px_b), .S(pclk_sel), .O(pclk));

	wire grst_sync_n;
	wire grst_raw_n = !cpu_reset && mmcm_locked;
	am_sync sync_g (.clk(gclk), .rst_n(grst_raw_n), .d(1'b1), .q(grst_sync_n));

	/*
	 * Replicable reset root for non-GPU board infrastructure; ASYNC_REG on
	 * am_sync prevents Vivado from duplicating sync_g/s2_reg directly.
	 */
	(* max_fanout = 64 *) reg grst_pipe = 1'b0;
	always @(posedge gclk or negedge grst_raw_n) begin
		if (!grst_raw_n)
			grst_pipe <= 1'b0;
		else
			grst_pipe <= grst_sync_n;
	end
	assign grst_n = grst_pipe;

	am_sync sync_ref (.clk(ref_clk), .rst_n(!cpu_reset), .d(1'b1), .q(ref_rst_n));
	am_sync sync_p (.clk(pclk), .rst_n(!cpu_reset && mmcm_locked), .d(1'b1), .q(prst_n));
endmodule

`default_nettype wire
