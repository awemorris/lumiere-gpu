/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_st_pipe: 1-cycle pipeline buffer slice on the CP -> GU control/state
 * broadcast (st_we, st_idx, st_wdata, and optional sideband kick/clear bits).
 *
 * Keeps a dedicated local register copy next to each GPU unit instead of
 * driving high-fanout combinational state decodes directly across the die.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_st_pipe #(
	parameter IDX_W  = 9,
	parameter CTRL_W = 1
) (
	input  wire              clk,
	input  wire              rst_n,
	input  wire              in_we,
	input  wire [IDX_W-1:0]  in_idx,
	input  wire [31:0]       in_wdata,
	input  wire [CTRL_W-1:0] in_ctrl,
	output wire              out_we,
	output wire [IDX_W-1:0]  out_idx,
	output wire [31:0]       out_wdata,
	output wire [CTRL_W-1:0] out_ctrl,
	output wire              out_rst_n
);
	(* keep = "true", equivalent_register_removal = "no" *) reg              r_we;
	(* keep = "true", equivalent_register_removal = "no" *) reg [IDX_W-1:0]  r_idx;
	(* keep = "true", equivalent_register_removal = "no" *) reg [31:0]       r_wdata;
	(* keep = "true", equivalent_register_removal = "no" *) reg [CTRL_W-1:0] r_ctrl;
	/* Forbid BUFGCE insertion on reset while allowing local fanout/LUT1 replication. */
	(* clock_buffer_type = "none", max_fanout = 64, equivalent_register_removal = "no" *) reg r_rst_n;

	/* Async reset only on tiny strobe/control bits to keep **async_default** load minimal. */
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_we    <= 1'b0;
			r_ctrl  <= {CTRL_W{1'b0}};
			r_rst_n <= 1'b0;
		end else begin
			r_we    <= in_we;
			r_ctrl  <= in_ctrl;
			r_rst_n <= 1'b1;
		end
	end

	always @(posedge clk) begin
		r_idx   <= in_idx;
		r_wdata <= in_wdata;
	end

	assign out_we    = r_we;
	assign out_idx   = r_idx;
	assign out_wdata = r_wdata;
	assign out_ctrl  = r_ctrl;
	assign out_rst_n = r_rst_n;
endmodule

`default_nettype wire
