/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_setup: Stream Setup Engine (Phase 022):
 * Fully-pipelined hardware setup engine replacing the microcoded FSM.
 * Direct feedforward sequencing for perspective divide, viewport mapping,
 * integer edge calculation (atg_rx_edge), plane equations, and depth bias.
 * Instant 1-cycle triangle loading and zero-latency early culling bypass.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_rx_setup #(
	parameter ENABLE_MULTIVIEWPORT = 0
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* From atg_rx_clip */
	input wire cl_valid,
	output wire cl_ready,
	input wire [701:0] cl_tri,
	input wire cl_last,
	/* From Mesh Shader (atg_su_exp_mesh) */
	input wire mesh_valid,
	output wire mesh_ready,
	input wire [701:0] mesh_tri,
	input wire mesh_last,
	input wire [15:0] mesh_id,
	/* To atg_rx_scan */
	output reg sp_valid,
	input wire sp_ready,
	output wire [767:0] sp_tri,
	output wire sp_last,
	output wire busy,
	/* Extended primitive interfaces */
	output wire [2:0] prim_type,
	output wire [15:0] cur_line_width,
	output wire [15:0] cur_point_size,
	output wire [3:0] cur_viewport_idx
);
	`include "ghci_regs.vh"

`ifdef SYNTH_KINTEX7
`define RX_BUF1(inst_name, in_sig, out_sig) \
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) inst_name (.I0(in_sig), .O(out_sig))
`elsif SYNTH_KINTEX_ULTRASCALE
`define RX_BUF1(inst_name, in_sig, out_sig) \
	assign out_sig = in_sig
`else
`define RX_BUF1(inst_name, in_sig, out_sig) assign out_sig = in_sig
`endif

	localparam [31:0] ONE  = 32'h3F800000;
	localparam [31:0] HALF = 32'h3F000000;

	/* STATE: VIEWPORT 0x050-0x055, SCISSOR_MIN/MAX 0x056-0x057 (16-entry arrays when ENABLE_MULTIVIEWPORT != 0),
	 * CULL 0x058, RAST_MODE 0x05A, DEPTH_BIAS_* 0x060-0x062, LINE_WIDTH 0x063,
	 * POINT_SIZE 0x064, VIEWPORT_IDX 0x065, RT_SIZE 0x073.
	 * VIEWPORT words: 0 x_scale, 1 y_scale, 2 x_offset, 3 y_offset, 4 z_scale, 5 z_offset. */
	reg [15:0] line_width;
	reg [15:0] point_size;
	reg [15:0] rts_lo, rts_hi;
	reg [15:0] b_const_lo, b_const_hi;
	reg [15:0] b_slope_lo, b_slope_hi;
	reg [15:0] b_clamp_lo, b_clamp_hi;
	wire [31:0] rts     = {rts_hi, rts_lo};
	wire [31:0] b_const = {b_const_hi, b_const_lo};
	wire [31:0] b_slope = {b_slope_hi, b_slope_lo};
	wire [31:0] b_clamp = {b_clamp_hi, b_clamp_lo};
	reg [2:0] cull;
	reg [5:0] rm;	/* RAST_MODE[5:0] */

	wire [31:0] cur_smin;
	wire [31:0] cur_smax;
	wire [15:0] cur_vp_lo [0:5];
	wire [15:0] cur_vp_hi [0:5];
	wire [3:0] cur_vp_idx_w;

	(* keep = "true" *) wire rst_setup_high = !rst_n;
	(* keep = "true" *) wire rst_setup_pp   = !rst_n;
	wire rst_n_edge = rst_n;
	wire rst_n_fpu = rst_n;
	wire [31:0] st_wdata_vp_b = st_wdata;
	wire [31:0] st_wdata_sc_b = st_wdata;
	(* keep = "true" *) reg r_st_lw_we;
	(* keep = "true" *) reg [31:0] r_st_lw_data;
	(* keep = "true" *) reg r_st_ps_we;
	(* keep = "true" *) reg [31:0] r_st_ps_data;
	(* keep = "true" *) reg [5:0] r_st_lw_pipe;
	(* keep = "true" *) reg r_st_lw_is_fp;
	(* keep = "true" *) reg [15:0] r_st_lw_raw_lo;
	(* keep = "true" *) reg [6:0] r_st_ps_pipe;
	(* keep = "true" *) reg r_st_ps_is_fp;
	(* keep = "true" *) reg [15:0] r_st_ps_raw_lo;
	(* keep = "true" *) reg [31:0] r_st_ps_raw;
	(* keep = "true" *) reg r_ps_is_zero;
	(* keep = "true" *) reg r_ps_is_one;
	(* keep = "true" *) reg [15:0] r_ps_pipe_q;
	(* keep = "true" *) reg [31:0] r_ps_div_b;
	wire [31:0] st_wdata_fpu_lw = r_st_lw_data;
	wire [31:0] st_wdata_fpu_ps = r_st_ps_data;
	wire [31:0] st_wdata_cfg_reg = st_wdata;
	wire [15:0] vp_w_lo [0:5];
	wire [15:0] vp_w_hi [0:5];
	wire [15:0] smin_lo_buf = st_wdata[15:0];
	wire [15:0] smin_hi_buf = st_wdata[31:16];
	wire [15:0] smax_lo_buf = st_wdata[15:0];
	wire [15:0] smax_hi_buf = st_wdata[31:16];
	wire [15:0] misc_lo_buf = st_wdata[15:0];
	wire [15:0] misc_hi_buf = st_wdata[31:16];
	wire [2:0] st_idx_vp_lo, st_idx_vp_hi, st_idx_cfg, st_idx_lwps;
	`RX_BUF1(u_sivpl0, st_idx[0], st_idx_vp_lo[0]);
	`RX_BUF1(u_sivpl1, st_idx[1], st_idx_vp_lo[1]);
	`RX_BUF1(u_sivpl2, st_idx[2], st_idx_vp_lo[2]);
	`RX_BUF1(u_sivph0, st_idx[0], st_idx_vp_hi[0]);
	`RX_BUF1(u_sivph1, st_idx[1], st_idx_vp_hi[1]);
	`RX_BUF1(u_sivph2, st_idx[2], st_idx_vp_hi[2]);
	`RX_BUF1(u_sicfg0, st_idx[0], st_idx_cfg[0]);
	`RX_BUF1(u_sicfg1, st_idx[1], st_idx_cfg[1]);
	`RX_BUF1(u_sicfg2, st_idx[2], st_idx_cfg[2]);
	`RX_BUF1(u_silw0,  st_idx[0], st_idx_lwps[0]);
	`RX_BUF1(u_silw1,  st_idx[1], st_idx_lwps[1]);
	`RX_BUF1(u_silw2,  st_idx[2], st_idx_lwps[2]);
	wire [8:3] st_idx_hi_cfg = st_idx[8:3];
	wire [8:0] st_idx_c = {st_idx_hi_cfg, st_idx_cfg};
	wire [8:0] st_idx_l = {st_idx_hi_cfg, st_idx_lwps};
	wire we_vp = st_we && (st_idx_c >= GHCI_S_VIEWPORT && st_idx_c <= GHCI_S_VIEWPORT + 9'd5);
	wire we_vp_lo, we_vp_hi;
	`RX_BUF1(u_we_vpl, we_vp, we_vp_lo);
	`RX_BUF1(u_we_vph, we_vp, we_vp_hi);
	wire [5:0] we_vp_w_lo, we_vp_w_hi;
	genvar gvw;
	generate
		for (gvw = 0; gvw < 6; gvw = gvw + 1) begin : g_vp_dec_sim
			`RX_BUF1(u_wvp_wl, we_vp_lo && (st_idx_vp_lo == gvw[2:0]), we_vp_w_lo[gvw]);
			`RX_BUF1(u_wvp_wh, we_vp_hi && (st_idx_vp_hi == gvw[2:0]), we_vp_w_hi[gvw]);
			assign vp_w_lo[gvw] = st_wdata[15:0];
			assign vp_w_hi[gvw] = st_wdata[31:16];
		end
	endgenerate
	wire we_smin_raw = st_we && (st_idx_c == GHCI_S_SCISSOR_MIN);
	wire we_smax_raw = st_we && (st_idx_c == GHCI_S_SCISSOR_MAX);
	wire we_bias_raw = st_we && (st_idx_c == GHCI_S_DEPTH_BIAS_CONST || st_idx_c == GHCI_S_DEPTH_BIAS_SLOPE || st_idx_c == GHCI_S_DEPTH_BIAS_CLAMP || st_idx_c == GHCI_S_RT_SIZE);
	wire we_smin_lo, we_smin_hi, we_smax_lo, we_smax_hi, we_bias_lo, we_bias_hi;
	`RX_BUF1(u_wsml, we_smin_raw, we_smin_lo);
	`RX_BUF1(u_wsmh, we_smin_raw, we_smin_hi);
	`RX_BUF1(u_wsxl, we_smax_raw, we_smax_lo);
	`RX_BUF1(u_wsxh, we_smax_raw, we_smax_hi);
	`RX_BUF1(u_wbl,  we_bias_raw, we_bias_lo);
	`RX_BUF1(u_wbh,  we_bias_raw, we_bias_hi);

	generate
		/* 16-entry multi-viewport/scissor path: unused under GHCI 1.0 (maxViewports=1); retained for unit tests/future extensions. */
		if (ENABLE_MULTIVIEWPORT != 0) begin : g_mvp
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] vp_array_lo [0:15][0:5];
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] vp_array_hi [0:15][0:5];
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] smin_array_lo [0:15];
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] smin_array_hi [0:15];
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] smax_array_lo [0:15];
			(* ram_style = "registers", extract_reset = "no" *) reg [15:0] smax_array_hi [0:15];
`ifndef SYNTHESIS
			integer ini_i;
			initial begin
				for (ini_i = 0; ini_i < 16; ini_i = ini_i + 1) begin
					smin_array_lo[ini_i] = 16'd0;
					smin_array_hi[ini_i] = 16'd0;
					smax_array_lo[ini_i] = 16'h7FFF;
					smax_array_hi[ini_i] = 16'h7FFF;
				end
			end
`endif
			(* max_fanout = 16 *) reg [3:0] vp_idx;
			wire [3:0] vp_idx_w_lo [0:5];
			wire [3:0] vp_idx_w_hi [0:5];
			wire [3:0] vp_idx_ws_smin_l = vp_idx;
			wire [3:0] vp_idx_ws_smin_h = vp_idx;
			wire [3:0] vp_idx_ws_smx_l  = vp_idx;
			wire [3:0] vp_idx_ws_smx_h  = vp_idx;
			wire [3:0] vp_idx_rs_smin_b0 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b1 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b2 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b3 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b4 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b5 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b6 = vp_idx;
			wire [3:0] vp_idx_rs_smin_b7 = vp_idx;
			wire [3:0] vp_idx_rs_smx_b0  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b1  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b2  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b3  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b4  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b5  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b6  = vp_idx;
			wire [3:0] vp_idx_rs_smx_b7  = vp_idx;
			wire [3:0] vp_idx_r_lo_b0 [0:5];
			wire [3:0] vp_idx_r_lo_b1 [0:5];
			wire [3:0] vp_idx_r_lo_b2 [0:5];
			wire [3:0] vp_idx_r_lo_b3 [0:5];
			wire [3:0] vp_idx_r_hi_b0 [0:5];
			wire [3:0] vp_idx_r_hi_b1 [0:5];
			wire [3:0] vp_idx_r_hi_b2 [0:5];
			wire [3:0] vp_idx_r_hi_b3 [0:5];
			for (gvw = 0; gvw < 6; gvw = gvw + 1) begin : g_vpi_sim
				assign vp_idx_w_lo[gvw] = vp_idx;
				assign vp_idx_w_hi[gvw] = vp_idx;
				assign vp_idx_r_lo_b0[gvw] = vp_idx;
				assign vp_idx_r_lo_b1[gvw] = vp_idx;
				assign vp_idx_r_lo_b2[gvw] = vp_idx;
				assign vp_idx_r_lo_b3[gvw] = vp_idx;
				assign vp_idx_r_hi_b0[gvw] = vp_idx;
				assign vp_idx_r_hi_b1[gvw] = vp_idx;
				assign vp_idx_r_hi_b2[gvw] = vp_idx;
				assign vp_idx_r_hi_b3[gvw] = vp_idx;
				assign cur_vp_lo[gvw] = {vp_array_lo[vp_idx_r_lo_b3[gvw]][gvw][15:12], vp_array_lo[vp_idx_r_lo_b2[gvw]][gvw][11:8], vp_array_lo[vp_idx_r_lo_b1[gvw]][gvw][7:4], vp_array_lo[vp_idx_r_lo_b0[gvw]][gvw][3:0]};
				assign cur_vp_hi[gvw] = {vp_array_hi[vp_idx_r_hi_b3[gvw]][gvw][15:12], vp_array_hi[vp_idx_r_hi_b2[gvw]][gvw][11:8], vp_array_hi[vp_idx_r_hi_b1[gvw]][gvw][7:4], vp_array_hi[vp_idx_r_hi_b0[gvw]][gvw][3:0]};
			end
			assign cur_smin = {smin_array_hi[vp_idx_rs_smin_b7][15:12], smin_array_hi[vp_idx_rs_smin_b6][11:8],
			                   smin_array_hi[vp_idx_rs_smin_b5][7:4],   smin_array_hi[vp_idx_rs_smin_b4][3:0],
			                   smin_array_lo[vp_idx_rs_smin_b3][15:12], smin_array_lo[vp_idx_rs_smin_b2][11:8],
			                   smin_array_lo[vp_idx_rs_smin_b1][7:4],   smin_array_lo[vp_idx_rs_smin_b0][3:0]};
			assign cur_smax = {smax_array_hi[vp_idx_rs_smx_b7][15:12],  smax_array_hi[vp_idx_rs_smx_b6][11:8],
			                   smax_array_hi[vp_idx_rs_smx_b5][7:4],   smax_array_hi[vp_idx_rs_smx_b4][3:0],
			                   smax_array_lo[vp_idx_rs_smx_b3][15:12],  smax_array_lo[vp_idx_rs_smx_b2][11:8],
			                   smax_array_lo[vp_idx_rs_smx_b1][7:4],   smax_array_lo[vp_idx_rs_smx_b0][3:0]};
			assign cur_vp_idx_w = vp_idx;
			integer e;
	always @(posedge clk or posedge rst_setup_high) begin
				if (rst_setup_high) begin
					vp_idx <= 4'd0;
				end else begin
					for (e = 0; e < 16; e = e + 1) begin
						if (we_vp_w_lo[0] && (vp_idx_w_lo[0] == e[3:0])) vp_array_lo[e][0] <= vp_w_lo[0];
						if (we_vp_w_hi[0] && (vp_idx_w_hi[0] == e[3:0])) vp_array_hi[e][0] <= vp_w_hi[0];
						if (we_vp_w_lo[1] && (vp_idx_w_lo[1] == e[3:0])) vp_array_lo[e][1] <= vp_w_lo[1];
						if (we_vp_w_hi[1] && (vp_idx_w_hi[1] == e[3:0])) vp_array_hi[e][1] <= vp_w_hi[1];
						if (we_vp_w_lo[2] && (vp_idx_w_lo[2] == e[3:0])) vp_array_lo[e][2] <= vp_w_lo[2];
						if (we_vp_w_hi[2] && (vp_idx_w_hi[2] == e[3:0])) vp_array_hi[e][2] <= vp_w_hi[2];
						if (we_vp_w_lo[3] && (vp_idx_w_lo[3] == e[3:0])) vp_array_lo[e][3] <= vp_w_lo[3];
						if (we_vp_w_hi[3] && (vp_idx_w_hi[3] == e[3:0])) vp_array_hi[e][3] <= vp_w_hi[3];
						if (we_vp_w_lo[4] && (vp_idx_w_lo[4] == e[3:0])) vp_array_lo[e][4] <= vp_w_lo[4];
						if (we_vp_w_hi[4] && (vp_idx_w_hi[4] == e[3:0])) vp_array_hi[e][4] <= vp_w_hi[4];
						if (we_vp_w_lo[5] && (vp_idx_w_lo[5] == e[3:0])) vp_array_lo[e][5] <= vp_w_lo[5];
						if (we_vp_w_hi[5] && (vp_idx_w_hi[5] == e[3:0])) vp_array_hi[e][5] <= vp_w_hi[5];

						if (we_smin_lo && (vp_idx_ws_smin_l == e[3:0])) smin_array_lo[e] <= smin_lo_buf;
						if (we_smin_hi && (vp_idx_ws_smin_h == e[3:0])) smin_array_hi[e] <= smin_hi_buf;
						if (we_smax_lo && (vp_idx_ws_smx_l == e[3:0])) smax_array_lo[e] <= smax_lo_buf;
						if (we_smax_hi && (vp_idx_ws_smx_h == e[3:0])) smax_array_hi[e] <= smax_hi_buf;
					end
					if (st_we && (st_idx == GHCI_S_VIEWPORT_IDX))
						vp_idx <= st_wdata_cfg_reg[3:0];
				end
			end
		end else begin : g_svp
			(* extract_reset = "no" *) reg [15:0] vp_single_lo [0:5];
			(* extract_reset = "no" *) reg [15:0] vp_single_hi [0:5];
			reg [15:0] smin_single_lo, smin_single_hi;
			reg [15:0] smax_single_lo, smax_single_hi;
			for (gvw = 0; gvw < 6; gvw = gvw + 1) begin : g_svp_rd
				assign cur_vp_lo[gvw] = vp_single_lo[gvw];
				assign cur_vp_hi[gvw] = vp_single_hi[gvw];
			end
			assign cur_smin = {smin_single_hi, smin_single_lo};
			assign cur_smax = {smax_single_hi, smax_single_lo};
			assign cur_vp_idx_w = 4'd0;
			integer swi;
			always @(posedge clk) begin
				for (swi = 0; swi < 6; swi = swi + 1) begin
					if (we_vp_w_lo[swi]) vp_single_lo[swi] <= vp_w_lo[swi];
					if (we_vp_w_hi[swi]) vp_single_hi[swi] <= vp_w_hi[swi];
				end
			end
	always @(posedge clk or posedge rst_setup_high) begin
				if (rst_setup_high) begin
					smin_single_lo <= 16'd0;
					smin_single_hi <= 16'd0;
					smax_single_lo <= 16'h7FFF;
					smax_single_hi <= 16'h7FFF;
				end else begin
					if (we_smin_lo) smin_single_lo <= smin_lo_buf;
					if (we_smin_hi) smin_single_hi <= smin_hi_buf;
					if (we_smax_lo) smax_single_lo <= smax_lo_buf;
					if (we_smax_hi) smax_single_hi <= smax_hi_buf;
				end
			end
		end
	endgenerate

	wire [23:0] lw_fix, ps_fix;
	atg_rx_f2fix_pipe u_lw_fix (.clk(clk), .rst_n(rst_n_fpu), .i_f(st_wdata_fpu_lw), .o_q(lw_fix));
	atg_rx_f2fix_pipe u_ps_fix (.clk(clk), .rst_n(rst_n_fpu), .i_f(st_wdata_fpu_ps), .o_q(ps_fix));

	wire [31:0] ps_in_fp32;
	atg_rx_fix2f_pipe #(.W(16), .F(8), .SIGNED(0)) u_ps2f (.clk(clk), .rst_n(rst_n_fpu), .i_v(st_wdata_fpu_ps[15:0]), .o_f(ps_in_fp32));
	reg [15:0] inv_ps_lo, inv_ps_hi;
	wire [31:0] inv_ps = {inv_ps_hi, inv_ps_lo};
	wire [31:0] div_inv_ps;
	wire div_inv_done, div_busy;
	wire [31:0] div_b = r_ps_div_b;
	wire div_inv_done_lo, div_inv_done_hi;
	`RX_BUF1(u_didl, div_inv_done, div_inv_done_lo);
	`RX_BUF1(u_didh, div_inv_done, div_inv_done_hi);
	reg div_inv_start, r_lw_ps_busy;
	wire lw_ps_pipe_busy = r_lw_ps_busy;
	atg_rx_fdiv u_ps_div (
		.clk(clk), .rst_n(rst_n_fpu),
		.i_start(div_inv_start),
		.i_a(32'h3F800000), .i_b(div_b),
		.busy(div_busy), .o_done(div_inv_done), .o_r(div_inv_ps)
	);

`ifdef SYNTH_KINTEX7
	assign prim_type = {1'b0, kind_prim};
`elsif SYNTH_KINTEX_ULTRASCALE
	assign prim_type = {1'b0, kind_prim};
`else
	assign prim_type = {1'b0, kind};
`endif
	assign cur_line_width = line_width;
	assign cur_point_size = point_size;
	assign cur_viewport_idx = cur_vp_idx_w;

	wire we_ps_ctrl, we_ps_raw_lo, we_ps_raw_hi;
	wire ps_p5_ctrl, ps_p5_dlo, ps_p5_dhi;
	wire ps_p6_ps, ps_p6_ilo, ps_p6_ihi, ps_p6_ctrl;
	wire ps_isfp_q, ps_isfp_dlo, ps_isfp_dhi;
	wire ps_zo_ilo, ps_zo_ihi, ps_zo_div;
	wire we_bc_lo, we_bc_hi, we_bs_lo, we_bs_hi, we_bcl_lo, we_bcl_hi, we_rts_lo, we_rts_hi;
	wire we_lw_raw = st_we && (st_idx_l == GHCI_S_LINE_WIDTH);
	wire we_ps_raw_in = st_we && (st_idx_l == GHCI_S_POINT_SIZE);
	wire we_lw_d0, we_lw_d1, we_lw_p;
	wire we_ps_d0, we_ps_d1, we_ps_p;
	wire we_lw_ctrl_b, we_lw_rlo_b;
	`RX_BUF1(u_wlwd0,  we_lw_raw,    we_lw_d0);
	`RX_BUF1(u_wlwd1,  we_lw_raw,    we_lw_d1);
	`RX_BUF1(u_wlwp,   we_lw_raw,    we_lw_p);
	`RX_BUF1(u_wpsd0,  we_ps_raw_in, we_ps_d0);
	`RX_BUF1(u_wpsd1,  we_ps_raw_in, we_ps_d1);
	`RX_BUF1(u_wpsp,   we_ps_raw_in, we_ps_p);
	`RX_BUF1(u_wlwcb,  r_st_lw_we,   we_lw_ctrl_b);
	`RX_BUF1(u_wlwrlb, r_st_lw_we,   we_lw_rlo_b);

	`RX_BUF1(u_wpsc,   r_st_ps_we, we_ps_ctrl);
	`RX_BUF1(u_wpsrl,  r_st_ps_we, we_ps_raw_lo);
	`RX_BUF1(u_wpsrh,  r_st_ps_we, we_ps_raw_hi);
	`RX_BUF1(u_pp5c,   r_st_ps_pipe[5], ps_p5_ctrl);
	`RX_BUF1(u_pp5dl,  r_st_ps_pipe[5], ps_p5_dlo);
	`RX_BUF1(u_pp5dh,  r_st_ps_pipe[5], ps_p5_dhi);
	`RX_BUF1(u_pp6p,   r_st_ps_pipe[6], ps_p6_ps);
	`RX_BUF1(u_pp6il,  r_st_ps_pipe[6], ps_p6_ilo);
	`RX_BUF1(u_pp6ih,  r_st_ps_pipe[6], ps_p6_ihi);
	`RX_BUF1(u_pp6c,   r_st_ps_pipe[6], ps_p6_ctrl);
	`RX_BUF1(u_pifq,   r_st_ps_is_fp, ps_isfp_q);
	`RX_BUF1(u_pifdl,  r_st_ps_is_fp, ps_isfp_dlo);
	`RX_BUF1(u_pifdh,  r_st_ps_is_fp, ps_isfp_dhi);
	`RX_BUF1(u_pzoil,  r_ps_is_zero || r_ps_is_one, ps_zo_ilo);
	`RX_BUF1(u_pzoih,  r_ps_is_zero || r_ps_is_one, ps_zo_ihi);
	`RX_BUF1(u_pzod,   r_ps_is_zero || r_ps_is_one, ps_zo_div);
	`RX_BUF1(u_wbcl,   we_bias_lo && (st_idx_c == GHCI_S_DEPTH_BIAS_CONST), we_bc_lo);
	`RX_BUF1(u_wbch,   we_bias_hi && (st_idx_c == GHCI_S_DEPTH_BIAS_CONST), we_bc_hi);
	`RX_BUF1(u_wbsl,   we_bias_lo && (st_idx_c == GHCI_S_DEPTH_BIAS_SLOPE), we_bs_lo);
	`RX_BUF1(u_wbsh,   we_bias_hi && (st_idx_c == GHCI_S_DEPTH_BIAS_SLOPE), we_bs_hi);
	`RX_BUF1(u_wbcll,  we_bias_lo && (st_idx_c == GHCI_S_DEPTH_BIAS_CLAMP), we_bcl_lo);
	`RX_BUF1(u_wbclh,  we_bias_hi && (st_idx_c == GHCI_S_DEPTH_BIAS_CLAMP), we_bcl_hi);
	`RX_BUF1(u_wrtsl,  we_bias_lo && (st_idx_c == GHCI_S_RT_SIZE),          we_rts_lo);
	`RX_BUF1(u_wrtsh,  we_bias_hi && (st_idx_c == GHCI_S_RT_SIZE),          we_rts_hi);

	always @(posedge clk or posedge rst_setup_high)
		if (rst_setup_high) begin
			r_st_lw_we <= 1'b0;
			r_st_lw_data <= 32'd0;
			r_st_ps_we <= 1'b0;
			r_st_ps_data <= 32'd0;
			r_st_lw_pipe <= 6'd0;
			r_st_lw_is_fp <= 1'b0;
			r_st_lw_raw_lo <= 16'd0;
			r_st_ps_pipe <= 7'd0;
			r_st_ps_is_fp <= 1'b0;
			r_st_ps_raw_lo <= 16'd0;
			r_st_ps_raw <= 32'd0;
			r_ps_is_zero <= 1'b0;
			r_ps_is_one <= 1'b0;
			r_ps_pipe_q <= 16'h0100;
			r_ps_div_b <= 32'h3F800000;
			line_width <= 16'h0100;
			point_size <= 16'h0100;
			inv_ps_lo <= 16'h0000;
			inv_ps_hi <= 16'h3F80;
			div_inv_start <= 1'b0;
			r_lw_ps_busy <= 1'b0;
			rts_lo <= 16'd0;
			rts_hi <= 16'd0;
			cull <= 3'd0;
			rm <= 6'd0;
			b_const_lo <= 16'd0;
			b_const_hi <= 16'd0;
			b_slope_lo <= 16'd0;
			b_slope_hi <= 16'd0;
			b_clamp_lo <= 16'd0;
			b_clamp_hi <= 16'd0;
		end else begin
			r_st_lw_we <= we_lw_raw;
			if (we_lw_d0) r_st_lw_data[15:0]  <= st_wdata[15:0];
			if (we_lw_d1) r_st_lw_data[31:16] <= st_wdata[31:16];
			if (we_lw_p)  r_st_lw_pipe <= 6'd0;
			else          r_st_lw_pipe <= {r_st_lw_pipe[4:0], r_st_lw_we};

			r_st_ps_we <= we_ps_raw_in;
			if (we_ps_d0) r_st_ps_data[15:0]  <= st_wdata[15:0];
			if (we_ps_d1) r_st_ps_data[31:16] <= st_wdata[31:16];
			if (we_ps_p)  r_st_ps_pipe <= 7'd0;
			else          r_st_ps_pipe <= {r_st_ps_pipe[5:0], r_st_ps_we};

			if (we_lw_ctrl_b) r_st_lw_is_fp  <= (r_st_lw_data[31:23] != 9'd0);
			if (we_lw_rlo_b)  r_st_lw_raw_lo <= r_st_lw_data[15:0];

			if (we_ps_ctrl) begin
				r_st_ps_is_fp <= (r_st_ps_data[31:23] != 9'd0);
				r_st_ps_raw_lo <= r_st_ps_data[15:0];
			end
			if (we_ps_raw_lo)
				r_st_ps_raw[15:0] <= r_st_ps_data[15:0];
			if (we_ps_raw_hi)
				r_st_ps_raw[31:16] <= r_st_ps_data[31:16];

			div_inv_start <= 1'b0;
			begin : b_lw_ps_busy
				reg p0_n, p1_n, p2_n;
				p0_n = !(r_st_lw_we | (|r_st_lw_pipe[4:0]) | r_st_ps_we);
				p1_n = !((|r_st_ps_pipe[5:0]) | div_inv_start | div_busy);
				p2_n = !(div_inv_done | (ps_p6_ctrl & !(r_ps_is_zero | r_ps_is_one)));
				r_lw_ps_busy <= we_lw_raw | we_ps_raw_in | !(p0_n & p1_n & p2_n);
			end
			if (r_st_lw_pipe[5]) begin
				line_width <= r_st_lw_is_fp ? lw_fix[15:0] : r_st_lw_raw_lo;
			end
			if (ps_p5_ctrl) begin
				r_ps_is_zero <= ps_isfp_q ? (ps_fix[15:0] == 16'd0) : (r_st_ps_raw_lo == 16'd0);
				r_ps_is_one <= (r_st_ps_raw == 32'h0100) || (r_st_ps_raw == 32'h3F800000);
				r_ps_pipe_q <= ps_isfp_q ? ps_fix[15:0] : r_st_ps_raw_lo;
			end
			if (ps_p5_dlo)
				r_ps_div_b[15:0] <= ps_isfp_dlo ? r_st_ps_data[15:0] : ps_in_fp32[15:0];
			if (ps_p5_dhi)
				r_ps_div_b[31:16] <= ps_isfp_dhi ? r_st_ps_data[31:16] : ps_in_fp32[31:16];
			if (ps_p6_ps)
				point_size <= r_ps_pipe_q;
			if (ps_p6_ilo && ps_zo_ilo)
				inv_ps_lo <= 16'h0000;
			if (ps_p6_ihi && ps_zo_ihi)
				inv_ps_hi <= 16'h3F80;
			if (ps_p6_ctrl && !ps_zo_div)
				div_inv_start <= 1'b1;
			if (st_we) begin
				if (st_idx_l == GHCI_S_CULL)
					cull <= st_wdata_cfg_reg[2:0];
				if (st_idx_l == GHCI_S_RAST_MODE)
					rm <= st_wdata_cfg_reg[5:0];
			end
			if (we_bc_lo)
				b_const_lo <= misc_lo_buf;
			if (we_bc_hi)
				b_const_hi <= misc_hi_buf;
			if (we_bs_lo)
				b_slope_lo <= misc_lo_buf;
			if (we_bs_hi)
				b_slope_hi <= misc_hi_buf;
			if (we_bcl_lo)
				b_clamp_lo <= misc_lo_buf;
			if (we_bcl_hi)
				b_clamp_hi <= misc_hi_buf;
			if (we_rts_lo)
				rts_lo <= misc_lo_buf;
			if (we_rts_hi)
				rts_hi <= misc_hi_buf;
			if (div_inv_done_lo)
				inv_ps_lo <= div_inv_ps[15:0];
			if (div_inv_done_hi)
				inv_ps_hi <= div_inv_ps[31:16];
		end

	/*
	 * Hardware Sequencer FSM
	 */
	localparam [3:0]
		S_IDLE      = 4'd0,
		S_PERSP_W   = 4'd1,
		S_PERSP_XYZ = 4'd2,
		S_RECT      = 4'd3,
		S_EDGE      = 4'd4,
		S_PLANE_DIV = 4'd5,
		S_PLANE     = 4'd6,
		S_BIAS      = 4'd7,
		S_OUT       = 4'd8;

	(* keep = "true", dont_touch = "true", fsm_encoding = "none", extract_enable = "no", extract_reset = "no" *) reg [3:0] st, st_rep_a, st_rep_b, st_rep_c, st_rep_d, st_rep_e, st_rep_f;
	(* keep = "true", dont_touch = "true" *) reg [8:0] st_oh;
	(* keep = "true", dont_touch = "true" *) reg [4:0] r_st_idle_br;
	(* keep = "true", dont_touch = "true", fsm_encoding = "none", extract_enable = "no", extract_reset = "no" *) reg [4:0] seq_step, seq_step_hi, seq_step_v, seq_step_rep_a, seq_step_rep_b, seq_step_rep_c, seq_step_rep_d;
	(* keep = "true" *) reg [31:0] seq_oh;
	(* keep = "true", dont_touch = "true" *) reg [31:0] seq_oh_s0, seq_oh_s1, seq_oh_s2, seq_oh_s3, seq_oh_s4, seq_oh_s5, seq_oh_s6, seq_oh_s7;
	(* keep = "true" *) reg [15:0] p_step_oh;
	(* keep = "true", dont_touch = "true" *) reg [15:0] p_step_oh_s0, p_step_oh_s1, p_step_oh_s2, p_step_oh_s3;
	reg r_fma_px_hold, r_fma_px_fmr, r_fmb_r28, r_fmb_r21, r_fmb_r22, r_fmb_r25, r_fmb_r29, r_fmb_r30, r_fma_pl_we;
	reg r_ps_2_7, r_ps_3_6, r_ps_5_9, r_ps_4_8_12, r_ps_0_1;
	reg r_ps_want_fa, r_ps_want_fm, r_ps_want_wrap;
	reg r_ps_ne_0100, r_bias_rm3;
	reg r_bcl_nan, r_bcl_is_zero;
	(* keep = "true", dont_touch = "true" *) reg [3:0] r_p_plane_oh_a0, r_p_plane_oh_a1, r_p_plane_oh_a2, r_p_plane_oh_a3, r_p_plane_oh_a4, r_p_plane_oh_a5;
	(* keep = "true", dont_touch = "true" *) reg [3:0] r_p_plane_oh_b0, r_p_plane_oh_b1, r_p_plane_oh_b2, r_p_plane_oh_b3, r_p_plane_oh_b4, r_p_plane_oh_b5;
	(* keep = "true", dont_touch = "true", fsm_encoding = "none", extract_reset = "no" *) reg [1:0] p_plane;	/* 0: Z, 1: RW, 2: B0, 3: B1 */
	(* keep = "true", dont_touch = "true", fsm_encoding = "none", extract_enable = "no", extract_reset = "no" *) reg [3:0] p_step, p_step_rep_a;
	(* keep = "true", dont_touch = "true", equivalent_register_removal = "no", extract_enable = "no", extract_reset = "no" *) reg [1:0] kind, kind_prim, kind_out, kind_edge, kind_fpu;
	reg [26:0] keep;
	reg lastq;
	reg raw;
	reg sub;
	reg more;

	 wire [1:0] p_plane_pf0_l, p_plane_pf0_h;
	 wire [1:0] p_plane_pf1_l, p_plane_pf1_h;
	 wire [1:0] p_plane_pf2_l, p_plane_pf2_h;
	 wire [1:0] p_plane_2_l, p_plane_2_h;
	 wire [1:0] p_plane_3_l, p_plane_3_h;
	 wire [1:0] p_plane_pl_l, p_plane_pl_h;
	`RX_BUF1(u_pp_f0l0, p_plane[0], p_plane_pf0_l[0]);
	`RX_BUF1(u_pp_f0l1, p_plane[1], p_plane_pf0_l[1]);
	`RX_BUF1(u_pp_f0h0, p_plane[0], p_plane_pf0_h[0]);
	`RX_BUF1(u_pp_f0h1, p_plane[1], p_plane_pf0_h[1]);
	`RX_BUF1(u_pp_f1l0, p_plane[0], p_plane_pf1_l[0]);
	`RX_BUF1(u_pp_f1l1, p_plane[1], p_plane_pf1_l[1]);
	`RX_BUF1(u_pp_f1h0, p_plane[0], p_plane_pf1_h[0]);
	`RX_BUF1(u_pp_f1h1, p_plane[1], p_plane_pf1_h[1]);
	`RX_BUF1(u_pp_f2l0, p_plane[0], p_plane_pf2_l[0]);
	`RX_BUF1(u_pp_f2l1, p_plane[1], p_plane_pf2_l[1]);
	`RX_BUF1(u_pp_f2h0, p_plane[0], p_plane_pf2_h[0]);
	`RX_BUF1(u_pp_f2h1, p_plane[1], p_plane_pf2_h[1]);
	`RX_BUF1(u_pp_2l0,  p_plane[0], p_plane_2_l[0]);
	`RX_BUF1(u_pp_2l1,  p_plane[1], p_plane_2_l[1]);
	`RX_BUF1(u_pp_2h0,  p_plane[0], p_plane_2_h[0]);
	`RX_BUF1(u_pp_2h1,  p_plane[1], p_plane_2_h[1]);
	`RX_BUF1(u_pp_3l0,  p_plane[0], p_plane_3_l[0]);
	`RX_BUF1(u_pp_3l1,  p_plane[1], p_plane_3_l[1]);
	`RX_BUF1(u_pp_3h0,  p_plane[0], p_plane_3_h[0]);
	`RX_BUF1(u_pp_3h1,  p_plane[1], p_plane_3_h[1]);
	`RX_BUF1(u_pp_pll0, p_plane[0], p_plane_pl_l[0]);
	`RX_BUF1(u_pp_pll1, p_plane[1], p_plane_pl_l[1]);
	`RX_BUF1(u_pp_plh0, p_plane[0], p_plane_pl_h[0]);
	`RX_BUF1(u_pp_plh1, p_plane[1], p_plane_pl_h[1]);

	 wire [3:0] st_fma_s [0:7];
	 wire [3:0] st_fmb_s [0:7];
	 wire [3:0] st_faa_s [0:7];
	 wire [3:0] st_fab_s [0:7];
	 wire [3:0] st_R_lo0, st_R_lo1, st_R_hi0, st_R_hi1;
	 wire [3:0] st_pl_lo, st_pl_hi, st_T_lo, st_T_hi;
	 wire [3:0] st_fdb_lo0, st_fdb_lo1, st_fdb_hi0, st_fdb_hi1;

	 wire [4:0] seq_fma_s [0:7];
	 wire [4:0] seq_fmb_s [0:7];
	 wire [4:0] seq_faa_s [0:7];
	 wire [4:0] seq_fab_s [0:7];
	 wire [4:0] seq_R_lo0, seq_R_lo1, seq_R_hi0, seq_R_hi1;
	 wire [4:0] seq_pl_lo, seq_pl_hi, seq_fdb_lo, seq_fdb_hi, seq_T_lo, seq_T_hi;

	 wire [3:0] p_step_a_s [0:7];
	 wire [3:0] p_step_b_s [0:7];
	 wire [3:0] p_step_ctrl, p_step_pl_lo, p_step_pl_hi, p_step_T_lo, p_step_T_hi;

	genvar gps, gsq, gsb;
	generate
		for (gsb = 0; gsb < 4; gsb = gsb + 1) begin : g_st_sl_buf
			for (gps = 0; gps < 4; gps = gps + 1) begin : g_st_b
				`RX_BUF1(u_st_fma_lo, st_rep_a[gps], st_fma_s[gsb][gps]);
				`RX_BUF1(u_st_fmb_lo, st_rep_a[gps], st_fmb_s[gsb][gps]);
				`RX_BUF1(u_st_faa_lo, st_rep_a[gps], st_faa_s[gsb][gps]);
				`RX_BUF1(u_st_fab_lo, st_rep_a[gps], st_fab_s[gsb][gps]);
				`RX_BUF1(u_st_fma_hi, st_rep_b[gps], st_fma_s[gsb + 4][gps]);
				`RX_BUF1(u_st_fmb_hi, st_rep_b[gps], st_fmb_s[gsb + 4][gps]);
				`RX_BUF1(u_st_faa_hi, st_rep_b[gps], st_faa_s[gsb + 4][gps]);
				`RX_BUF1(u_st_fab_hi, st_rep_b[gps], st_fab_s[gsb + 4][gps]);
				`RX_BUF1(u_psa_lo,    p_step[gps],       p_step_a_s[gsb][gps]);
				`RX_BUF1(u_psb_lo,    p_step[gps],       p_step_b_s[gsb][gps]);
				`RX_BUF1(u_psa_hi,    p_step_rep_a[gps], p_step_a_s[gsb + 4][gps]);
				`RX_BUF1(u_psb_hi,    p_step_rep_a[gps], p_step_b_s[gsb + 4][gps]);
			end
			for (gsq = 0; gsq < 5; gsq = gsq + 1) begin : g_sq_b
				`RX_BUF1(u_sq_fma_lo, seq_step_rep_a[gsq], seq_fma_s[gsb][gsq]);
				`RX_BUF1(u_sq_fmb_lo, seq_step_rep_a[gsq], seq_fmb_s[gsb][gsq]);
				`RX_BUF1(u_sq_faa_lo, seq_step_rep_b[gsq], seq_faa_s[gsb][gsq]);
				`RX_BUF1(u_sq_fab_lo, seq_step_rep_b[gsq], seq_fab_s[gsb][gsq]);
				`RX_BUF1(u_sq_fma_hi, seq_step_rep_c[gsq], seq_fma_s[gsb + 4][gsq]);
				`RX_BUF1(u_sq_fmb_hi, seq_step_rep_c[gsq], seq_fmb_s[gsb + 4][gsq]);
				`RX_BUF1(u_sq_faa_hi, seq_step_rep_d[gsq], seq_faa_s[gsb + 4][gsq]);
				`RX_BUF1(u_sq_fab_hi, seq_step_rep_d[gsq], seq_fab_s[gsb + 4][gsq]);
			end
		end
		for (gps = 0; gps < 4; gps = gps + 1) begin : g_st_ps_misc
			`RX_BUF1(u_st_rlo0,  st_rep_c[gps], st_R_lo0[gps]);
			`RX_BUF1(u_st_rlo1,  st_rep_c[gps], st_R_lo1[gps]);
			`RX_BUF1(u_st_rhi0,  st_rep_c[gps], st_R_hi0[gps]);
			`RX_BUF1(u_st_rhi1,  st_rep_c[gps], st_R_hi1[gps]);
			`RX_BUF1(u_st_pllo,  st_rep_c[gps], st_pl_lo[gps]);
			`RX_BUF1(u_st_plhi,  st_rep_c[gps], st_pl_hi[gps]);
			`RX_BUF1(u_st_fdbl0, st_rep_c[gps], st_fdb_lo0[gps]);
			`RX_BUF1(u_st_fdbl1, st_rep_c[gps], st_fdb_lo1[gps]);
			`RX_BUF1(u_st_fdbh0, st_rep_c[gps], st_fdb_hi0[gps]);
			`RX_BUF1(u_st_fdbh1, st_rep_c[gps], st_fdb_hi1[gps]);
			`RX_BUF1(u_st_tlo,   st_rep_c[gps], st_T_lo[gps]);
			`RX_BUF1(u_st_thi,   st_rep_c[gps], st_T_hi[gps]);

			`RX_BUF1(u_psc,    p_step[gps],       p_step_ctrl[gps]);
			`RX_BUF1(u_ps_pll, p_step_rep_a[gps], p_step_pl_lo[gps]);
			`RX_BUF1(u_ps_plh, p_step_rep_a[gps], p_step_pl_hi[gps]);
			`RX_BUF1(u_ps_tl,  p_step_rep_a[gps], p_step_T_lo[gps]);
			`RX_BUF1(u_ps_th,  p_step_rep_a[gps], p_step_T_hi[gps]);
		end
		for (gsq = 0; gsq < 5; gsq = gsq + 1) begin : g_seq_misc
			`RX_BUF1(u_sq_rlo0, seq_step_rep_a[gsq], seq_R_lo0[gsq]);
			`RX_BUF1(u_sq_rlo1, seq_step_rep_b[gsq], seq_R_lo1[gsq]);
			`RX_BUF1(u_sq_rhi0, seq_step_rep_c[gsq], seq_R_hi0[gsq]);
			`RX_BUF1(u_sq_rhi1, seq_step_rep_d[gsq], seq_R_hi1[gsq]);
			`RX_BUF1(u_sq_pllo, seq_step_rep_a[gsq], seq_pl_lo[gsq]);
			`RX_BUF1(u_sq_plhi, seq_step_rep_b[gsq], seq_pl_hi[gsq]);
			`RX_BUF1(u_sq_fdbl, seq_step_rep_c[gsq], seq_fdb_lo[gsq]);
			`RX_BUF1(u_sq_fdbh, seq_step_rep_d[gsq], seq_fdb_hi[gsq]);
			`RX_BUF1(u_sq_tlo,  seq_step_rep_c[gsq], seq_T_lo[gsq]);
			`RX_BUF1(u_sq_thi,  seq_step_rep_d[gsq], seq_T_hi[gsq]);
		end
	endgenerate

	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] R_lo [0:79];
	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] R_hi [0:79];
	wire [31:0] R [0:79];
	genvar gr;
	generate
		for (gr = 0; gr < 80; gr = gr + 1) begin : g_R_wire
			assign R[gr] = {R_hi[gr], R_lo[gr]};
		end
	endgenerate

	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] pl_lo [0:11];
	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] pl_hi [0:11];
	wire [31:0] pl [0:11];
	genvar gpl;
	generate
		for (gpl = 0; gpl < 12; gpl = gpl + 1) begin : g_pl_wire
			assign pl[gpl] = {pl_hi[gpl], pl_lo[gpl]};
		end
	endgenerate

	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] T0_lo, T0_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] T1_lo, T1_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] T2_lo, T2_hi;
	wire [31:0] T0 = {T0_hi, T0_lo};
	wire [31:0] T1 = {T1_hi, T1_lo};
	wire [31:0] T2 = {T2_hi, T2_lo};

	(* extract_reset = "no", extract_enable = "no" *) reg [15:0] r_bias_fa_lo, r_bias_fa_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_is_zero_lo, r_bcl_is_zero_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_sign_lo, r_bcl_sign_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_mag_lt_lo, r_bcl_mag_lt_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_mag_eq_lo, r_bcl_mag_eq_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_fa_nan_lo, r_fa_nan_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_fa_sign_lo, r_fa_sign_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_fa_is_zero_lo, r_fa_is_zero_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_chk_mag_lo, r_bcl_chk_mag_hi;
	(* extract_reset = "no", extract_enable = "no" *) reg r_bcl_use_fa_lo, r_bcl_use_fa_hi;
	wire [31:0] bm_val = max_mag({1'b0, pl[0][30:0]}, {1'b0, pl[1][30:0]});

	 wire [7:0] fa_o_s, fm_o_s;
	 wire fa_o_R_lo, fa_o_R_hi, fa_o_pl_lo, fa_o_pl_hi, fa_o_c_st, fa_o_c_seq, fa_o_c_ps, fa_o_c_v;
	 wire fm_o_R_lo, fm_o_R_hi, fm_o_pl_lo, fm_o_pl_hi, fm_o_c_st, fm_o_c_seq, fm_o_c_ps, fm_o_c_v;
	 wire fd_o_w, fd_o_fdb_lo, fd_o_fdb_hi, fd_o_plane_lo, fd_o_plane_hi, fd_o_plane_ctrl;

	genvar gfo;
	generate
		for (gfo = 0; gfo < 8; gfo = gfo + 1) begin : g_fo_sl
			`RX_BUF1(u_fao_s, fa_o, fa_o_s[gfo]);
			`RX_BUF1(u_fmo_s, fm_o, fm_o_s[gfo]);
		end
	endgenerate
	`RX_BUF1(u_fao_rlo,  fa_o, fa_o_R_lo);
	`RX_BUF1(u_fao_rhi,  fa_o, fa_o_R_hi);
	`RX_BUF1(u_fao_pllo, fa_o, fa_o_pl_lo);
	`RX_BUF1(u_fao_plhi, fa_o, fa_o_pl_hi);
	`RX_BUF1(u_fao_cst,  fa_o, fa_o_c_st);
	`RX_BUF1(u_fao_csq,  fa_o, fa_o_c_seq);
	`RX_BUF1(u_fao_cps,  fa_o, fa_o_c_ps);
	`RX_BUF1(u_fao_cv,   fa_o, fa_o_c_v);

	`RX_BUF1(u_fmo_rlo,  fm_o, fm_o_R_lo);
	`RX_BUF1(u_fmo_rhi,  fm_o, fm_o_R_hi);
	`RX_BUF1(u_fmo_pllo, fm_o, fm_o_pl_lo);
	`RX_BUF1(u_fmo_plhi, fm_o, fm_o_pl_hi);
	`RX_BUF1(u_fmo_cst,  fm_o, fm_o_c_st);
	`RX_BUF1(u_fmo_csq,  fm_o, fm_o_c_seq);
	`RX_BUF1(u_fmo_cps,  fm_o, fm_o_c_ps);
	`RX_BUF1(u_fmo_cv,   fm_o, fm_o_c_v);

	`RX_BUF1(u_fdo_w,    fd_o, fd_o_w);
	`RX_BUF1(u_fdo_fdbl, fd_o, fd_o_fdb_lo);
	`RX_BUF1(u_fdo_fdbh, fd_o, fd_o_fdb_hi);
	`RX_BUF1(u_fdo_pllo, fd_o, fd_o_plane_lo);
	`RX_BUF1(u_fdo_plhi, fd_o, fd_o_plane_hi);
	`RX_BUF1(u_fdo_pctl, fd_o, fd_o_plane_ctrl);

	wire [31:0] fa_r_R, fa_r_pl;
	wire [31:0] fm_r_R, fm_r_pl;
	wire [6:0] e_ra_b [0:7];
	genvar gbf, gnb;
	generate
		for (gbf = 0; gbf < 32; gbf = gbf + 1) begin : g_fpu_res_buf
			`RX_BUF1(u_far_r, fa_r[gbf], fa_r_R[gbf]);
			`RX_BUF1(u_far_p, fa_r[gbf], fa_r_pl[gbf]);
			`RX_BUF1(u_fmr_r, fm_r[gbf], fm_r_R[gbf]);
			`RX_BUF1(u_fmr_p, fm_r[gbf], fm_r_pl[gbf]);
		end
		for (gnb = 0; gnb < 8; gnb = gnb + 1) begin : g_era_nib
			for (gbf = 0; gbf < 7; gbf = gbf + 1) begin : g_era_bit
				`RX_BUF1(u_era, e_ra[gbf], e_ra_b[gnb][gbf]);
			end
		end
	endgenerate

	wire e_empty_buf;
	`RX_BUF1(u_buf_empty, e_empty, e_empty_buf);

	/* the bias's r for D16 (2^-16) and D24 (2^-24) */
`ifdef MUT_BIAS_R
	localparam [31:0] R24 = 32'h34000000;
`else
	localparam [31:0] R24 = 32'h33800000;
`endif
	wire [31:0] rfmt = rm[5:4] == 2'd1 ? 32'h37800000 : rm[5:4] == 2'd2 ? R24 : 32'd0;

	/* atg_rx_edge (the integer steps), sharing the register file. */
	reg e_start;
	wire [6:0] e_ra, e_wa;
	reg [31:0] e_rd;
	wire e_we, e_done, e_front, e_empty;
	wire [31:0] e_wd;
	wire [287:0] e_edges;
	wire [63:0] e_bbox;

	atg_rx_edge edge_u (
		.clk(clk), .rst_n(rst_n_edge), .start(e_start),
		.mode({raw, sub, kind_edge}), .smin(cur_smin), .smax(cur_smax), .rts(rts), .cull(cull),
		.vprect(rm[3]), .line_width(line_width), .point_size(point_size),
		.r_ra(e_ra), .r_rd(e_rd), .r_we(e_we), .r_wa(e_wa), .r_wd(e_wd),
		.done(e_done), .edges(e_edges), .bbox(e_bbox), .front(e_front), .empty(e_empty)
	);

	wire [6:0] we_r40_l, we_r40_h;
	genvar gr40;
	generate
		for (gr40 = 0; gr40 < 7; gr40 = gr40 + 1) begin : g_we_r40
			wire we_raw = e_we && (e_wa == (7'd40 + gr40));
			`RX_BUF1(u_wer40_l, we_raw, we_r40_l[gr40]);
			`RX_BUF1(u_wer40_h, we_raw, we_r40_h[gr40]);
		end
	endgenerate

	always @(posedge clk) begin
		if (we_r40_l[0]) R_lo[40] <= e_wd[15:0];
		if (we_r40_h[0]) R_hi[40] <= e_wd[31:16];
		if (we_r40_l[1]) R_lo[41] <= e_wd[15:0];
		if (we_r40_h[1]) R_hi[41] <= e_wd[31:16];
		if (we_r40_l[2]) R_lo[42] <= e_wd[15:0];
		if (we_r40_h[2]) R_hi[42] <= e_wd[31:16];
		if (we_r40_l[3]) R_lo[43] <= e_wd[15:0];
		if (we_r40_h[3]) R_hi[43] <= e_wd[31:16];
		if (we_r40_l[4]) R_lo[44] <= e_wd[15:0];
		if (we_r40_h[4]) R_hi[44] <= e_wd[31:16];
		if (we_r40_l[5]) R_lo[45] <= e_wd[15:0];
		if (we_r40_h[5]) R_hi[45] <= e_wd[31:16];
		if (we_r40_l[6]) R_lo[46] <= e_wd[15:0];
		if (we_r40_h[6]) R_hi[46] <= e_wd[31:16];
	end

	/* Floating-point arithmetic units */
	(* max_fanout = 16 *) reg fa_v, fm_v, fd_v;
	(* max_fanout = 16, extract_enable = "no", extract_reset = "no" *) reg fa_sub;
	(* extract_enable = "no", extract_reset = "no" *) reg [15:0] fd_b_lo, fd_b_hi;
	wire [31:0] fa_a, fa_b, fm_a, fm_b;
	wire [31:0] fd_a = ONE;
	wire [31:0] fd_b = {fd_b_hi, fd_b_lo};
	wire fa_o, fm_o, fd_o, fd_busy;
	wire [31:0] fa_r, fm_r, fd_r;

	atg_rx_fadd #(.LAT(4)) fadd (
		.clk(clk), .rst_n(rst_n_fpu), .i_valid(fa_v), .i_a(fa_a), .i_b(fa_b),
		.i_sub(fa_sub), .o_valid(fa_o), .o_r(fa_r)
	);
	atg_rx_fmul #(.LAT(4)) fmul (
		.clk(clk), .rst_n(rst_n_fpu), .i_valid(fm_v), .i_a(fm_a), .i_b(fm_b),
		.o_valid(fm_o), .o_r(fm_r)
	);
	atg_rx_fdiv fdiv (
		.clk(clk), .rst_n(rst_n_fpu), .i_start(fd_v), .i_a(fd_a), .i_b(fd_b),
		.busy(fd_busy), .o_done(fd_o), .o_r(fd_r)
	);

	/* Pure-LUT 31-bit float magnitude comparator for depth bias clamp (<= 7 cmos3) */
	wire bcl_mag_lt;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(31)) u_bcl_sub (
		.a(fa_r[30:0]),
		.b(b_clamp[30:0]),
		.bi(1'b0),
		.d(),
		.bo(bcl_mag_lt)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(31)) u_bcl_sub (
		.a(fa_r[30:0]),
		.b(b_clamp[30:0]),
		.bi(1'b0),
		.d(),
		.bo(bcl_mag_lt)
	);
`else
	wire [7:0] bcl_lt4, bcl_eq4;
	genvar gbc;
	generate
	for (gbc = 0; gbc < 7; gbc = gbc + 1) begin : g_bcl_c4
		wire [3:0] a4 = fa_r[4 * gbc +: 4];
		wire [3:0] b4 = b_clamp[4 * gbc +: 4];
		assign bcl_lt4[gbc] = (a4 < b4);
		assign bcl_eq4[gbc] = (a4 == b4);
	end
	endgenerate
	assign bcl_lt4[7] = (fa_r[30:28] <  b_clamp[30:28]);
	assign bcl_eq4[7] = (fa_r[30:28] == b_clamp[30:28]);
	wire [3:0] bcl_lt8, bcl_eq8;
	generate
	for (gbc = 0; gbc < 4; gbc = gbc + 1) begin : g_bcl_c8
		assign bcl_lt8[gbc] = bcl_lt4[2*gbc + 1] | (bcl_eq4[2*gbc + 1] & bcl_lt4[2*gbc]);
		assign bcl_eq8[gbc] = bcl_eq4[2*gbc + 1] & bcl_eq4[2*gbc];
	end
	endgenerate
	assign bcl_mag_lt = bcl_lt8[3] |
		(bcl_eq8[3] & bcl_lt8[2]) |
		(bcl_eq8[3] & bcl_eq8[2] & bcl_lt8[1]) |
		(bcl_eq8[3] & bcl_eq8[2] & bcl_eq8[1] & bcl_lt8[0]);
`endif
`ifdef SYNTH_KINTEX7
	wire bcl_mag_eq = (fa_r[30:0] == b_clamp[30:0]);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bcl_mag_eq = (fa_r[30:0] == b_clamp[30:0]);
`else
	wire bcl_mag_eq = &bcl_eq8;
`endif
	wire bcl_is_zero = r_bcl_is_zero;
	wire bcl_nan = r_bcl_nan;
	wire bcl_nan_free_lo = !r_fa_nan_lo;
	wire bcl_nan_free_hi = !r_fa_nan_hi;
	wire bcl_fa_le_lo = bcl_nan_free_lo && (
		r_fa_sign_lo ? (r_bcl_sign_lo ? !r_bcl_mag_lt_lo : 1'b1) :
		               (r_bcl_sign_lo ? (r_fa_is_zero_lo && r_bcl_is_zero_lo) : (r_bcl_mag_lt_lo || r_bcl_mag_eq_lo))
	);
	wire bcl_fa_le_hi = bcl_nan_free_hi && (
		r_fa_sign_hi ? (r_bcl_sign_hi ? !r_bcl_mag_lt_hi : 1'b1) :
		               (r_bcl_sign_hi ? (r_fa_is_zero_hi && r_bcl_is_zero_hi) : (r_bcl_mag_lt_hi || r_bcl_mag_eq_hi))
	);
	wire bcl_ge_lo = bcl_nan_free_lo && (
		r_bcl_sign_lo ? (r_fa_sign_lo ? (r_bcl_mag_lt_lo || r_bcl_mag_eq_lo) : 1'b1) :
		                (r_fa_sign_lo ? (r_fa_is_zero_lo && r_bcl_is_zero_lo) : !r_bcl_mag_lt_lo)
	);
	wire bcl_ge_hi = bcl_nan_free_hi && (
		r_bcl_sign_hi ? (r_fa_sign_hi ? (r_bcl_mag_lt_hi || r_bcl_mag_eq_hi) : 1'b1) :
		                (r_fa_sign_hi ? (r_fa_is_zero_hi && r_bcl_is_zero_hi) : !r_bcl_mag_lt_hi)
	);
	(* keep = "true" *) wire bcl_use_fa_lo = r_bcl_use_fa_lo || (r_bcl_chk_mag_lo && (r_bcl_mag_lt_lo || r_bcl_mag_eq_lo));
	(* keep = "true" *) wire bcl_use_fa_hi = r_bcl_use_fa_hi || (r_bcl_chk_mag_hi && (r_bcl_mag_lt_hi || r_bcl_mag_eq_hi));
	wire unused_bcl_flags = &{1'b0,
		r_bcl_is_zero_lo, r_bcl_sign_lo, r_fa_nan_lo, r_fa_sign_lo, r_fa_is_zero_lo,
		r_bcl_is_zero_hi, r_bcl_sign_hi, r_fa_nan_hi, r_fa_sign_hi, r_fa_is_zero_hi};

	function [31:0] max_mag;
		input [31:0] a, b;
		max_mag = b[30:0] > a[30:0] ? b : a;
	endfunction

	/* Pre-register bm_sel (pl[0] and pl[1] are written >100 cycles before S_BIAS) to eliminate CARRY4 in fm_a_sl path */
	/* Pure-LUT 0-XOR 2->4->8->16->32 AOI3/OAI3 comparator for bm_sel (<= 8 cmos3 vs 14) */
	wire [30:0] bm_gtn = (~pl[1][30:0]) | pl[0][30:0];
	wire [30:0] bm_gen = (~pl[1][30:0]) & pl[0][30:0];
	wire [15:0] bm_gt2, bm_ge2;
	genvar gbm;
	generate
	for (gbm = 0; gbm < 15; gbm = gbm + 1) begin : g_bm2
		assign bm_gt2[gbm] = !(bm_gen[2*gbm+1] | (bm_gtn[2*gbm+1] & bm_gtn[2*gbm]));
		assign bm_ge2[gbm] = !(bm_gen[2*gbm+1] | (bm_gtn[2*gbm+1] & bm_gen[2*gbm]));
	end
	assign bm_gt2[15] = !bm_gtn[30];
	assign bm_ge2[15] = !bm_gen[30];
	wire [7:0] bm_gtn4, bm_gen4;
	for (gbm = 0; gbm < 8; gbm = gbm + 1) begin : g_bm4
		assign bm_gtn4[gbm] = !(bm_gt2[2*gbm+1] | (bm_ge2[2*gbm+1] & bm_gt2[2*gbm]));
		assign bm_gen4[gbm] = !(bm_gt2[2*gbm+1] | (bm_ge2[2*gbm+1] & bm_ge2[2*gbm]));
	end
	wire [3:0] bm_gt8, bm_ge8;
	for (gbm = 0; gbm < 4; gbm = gbm + 1) begin : g_bm8
		assign bm_gt8[gbm] = !(bm_gtn4[2*gbm+1] & (bm_gen4[2*gbm+1] | bm_gtn4[2*gbm]));
		assign bm_ge8[gbm] = !(bm_gtn4[2*gbm+1] & (bm_gen4[2*gbm+1] | bm_gen4[2*gbm]));
	end
	wire [1:0] bm_gtn16, bm_gen16;
	for (gbm = 0; gbm < 2; gbm = gbm + 1) begin : g_bm16
		assign bm_gtn16[gbm] = !(bm_gt8[2*gbm+1] | (bm_ge8[2*gbm+1] & bm_gt8[2*gbm]));
		assign bm_gen16[gbm] = !(bm_gt8[2*gbm+1] | (bm_ge8[2*gbm+1] & bm_ge8[2*gbm]));
	end
	endgenerate
	wire bm_sel = !(bm_gtn16[1] & (bm_gen16[1] | bm_gtn16[0]));
	wire unused_bm_ge = bm_gen16[0];
	(* keep = "true", dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg r_bm_sel_lo, r_bm_sel_hi;
	always @(posedge clk) begin
		r_bm_sel_lo <= bm_sel;
		r_bm_sel_hi <= bm_sel;
	end
	wire [15:0] bm_val_lo = r_bm_sel_lo ? pl_lo[1] : pl_lo[0];
	wire [15:0] bm_val_hi = r_bm_sel_hi ? {1'b0, pl_hi[1][14:0]} : {1'b0, pl_hi[0][14:0]};

	wire [7:0] ea23;
	wire su_bo_unused;
	(* extract_enable = "no", extract_reset = "no" *) reg [7:0] ea_in;
	(* extract_enable = "no", extract_reset = "no" *) reg [7:0] ea23_r;
	(* extract_enable = "no", extract_reset = "no" *) reg       ea_gt_23_r;
	am_sub #(.W(8)) u_ea23 (.a(ea_in), .b(8'd23), .bi(1'b0), .d(ea23), .bo(su_bo_unused));

	wire [7:0] z0_exp = R[33][30:23];
	wire [7:0] z1_exp = R[36][30:23];
	wire [7:0] z2_exp = R[39][30:23];
	wire z0_lt_z1, z1_lt_z2, z0_lt_z2;
	wire [23:0] zexp_d_unused;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(8)) u_z01 (.a(z0_exp), .b(z1_exp), .bi(1'b0), .d(zexp_d_unused[7:0]),   .bo(z0_lt_z1));
	am_sub #(.W(8)) u_z12 (.a(z1_exp), .b(z2_exp), .bi(1'b0), .d(zexp_d_unused[15:8]),  .bo(z1_lt_z2));
	am_sub #(.W(8)) u_z02 (.a(z0_exp), .b(z2_exp), .bi(1'b0), .d(zexp_d_unused[23:16]), .bo(z0_lt_z2));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(8)) u_z01 (.a(z0_exp), .b(z1_exp), .bi(1'b0), .d(zexp_d_unused[7:0]),   .bo(z0_lt_z1));
	am_sub #(.W(8)) u_z12 (.a(z1_exp), .b(z2_exp), .bi(1'b0), .d(zexp_d_unused[15:8]),  .bo(z1_lt_z2));
	am_sub #(.W(8)) u_z02 (.a(z0_exp), .b(z2_exp), .bi(1'b0), .d(zexp_d_unused[23:16]), .bo(z0_lt_z2));
`else
	function automatic lt8_z;
		input [7:0] a, b;
		reg [7:0] cg, cp;
		reg [3:0] g2, p2;
		reg g4_0, g4_1, p4_1;
		begin
			cg = (~a) & b;
			cp = (~a) | b;
			g2 = {cg[7]|(cp[7]&cg[6]), cg[5]|(cp[5]&cg[4]), cg[3]|(cp[3]&cg[2]), cg[1]|(cp[1]&cg[0])};
			p2 = {cp[7]&cp[6], cp[5]&cp[4], cp[3]&cp[2], cp[1]&cp[0]};
			g4_0 = g2[1] | (p2[1] & g2[0]);
			g4_1 = g2[3] | (p2[3] & g2[2]);
			p4_1 = p2[3] & p2[2];
			lt8_z = g4_1 | (p4_1 & g4_0);
		end
	endfunction
	assign zexp_d_unused = 24'd0;
	assign z0_lt_z1 = lt8_z(z0_exp, z1_exp);
	assign z1_lt_z2 = lt8_z(z1_exp, z2_exp);
	assign z0_lt_z2 = lt8_z(z0_exp, z2_exp);
`endif
	wire [7:0] max_z_exp = (z0_lt_z2 && z1_lt_z2) ? z2_exp : (z0_lt_z1 ? z1_exp : z0_exp);

	/* Stream Selection: cl_* vs mesh_* (Option 4 Static MUX with cl_* priority) */
	 wire cl_valid_b0, cl_valid_b1;
	`RX_BUF1(u_clv_b0, cl_valid, cl_valid_b0);
	`RX_BUF1(u_clv_b1, cl_valid, cl_valid_b1);
	wire in_mesh = mesh_valid && !cl_valid_b0;
	wire in_valid = cl_valid_b0 || mesh_valid;
	wire [701:0] in_tri;
	wire in_last = in_mesh ? mesh_last : cl_last;

	wire in_raw = cl_valid_b1 ? cl_tri[699] : mesh_tri[699];
	wire [1:0] in_kind_hdr = cl_valid_b1 ? cl_tri[701:700] : mesh_tri[701:700];
	 wire cl_valid_kp0, cl_valid_kp1;
	`RX_BUF1(u_clv_kp0, cl_valid_b0, cl_valid_kp0);
	`RX_BUF1(u_clv_kp1, cl_valid_b1, cl_valid_kp1);
	wire [26:0] in_keep_hdr = {
		cl_valid_kp1 ? cl_tri[698:688] : mesh_tri[698:688],
		cl_valid_kp0 ? cl_tri[687:672] : mesh_tri[687:672]
	};

	 wire [3:0] in_mesh_t1;
	 wire [43:0] in_mesh_leaf;
	 wire [21:0] in_mesh_leaf_r31;
	(* keep = "true" *) wire [351:0] in_tri_r31;
	genvar gmt, gml;
	generate
		for (gmt = 0; gmt < 4; gmt = gmt + 1) begin : g_im_t1
			`RX_BUF1(u_imt, in_mesh, in_mesh_t1[gmt]);
			for (gml = 0; gml < 11; gml = gml + 1) begin : g_im_leaf
				`RX_BUF1(u_iml, in_mesh_t1[gmt], in_mesh_leaf[gmt * 11 + gml]);
			end
		end
		for (gml = 0; gml < 44; gml = gml + 1) begin : g_im_mux
			localparam integer HI = (gml == 43) ? 701 : (gml * 16 + 15);
			localparam integer LO = gml * 16;
			assign in_tri[HI:LO] = in_mesh_leaf[gml] ? mesh_tri[HI:LO] : cl_tri[HI:LO];
		end
		for (gml = 0; gml < 22; gml = gml + 1) begin : g_im_r31
			`RX_BUF1(u_im_r31, in_mesh_t1[gml / 6], in_mesh_leaf_r31[gml]);
			assign in_tri_r31[gml * 16 +: 16] = in_mesh_leaf_r31[gml] ? mesh_tri[gml * 16 +: 16] : cl_tri[gml * 16 +: 16];
		end
	endgenerate

	 wire in_valid_ctrl, in_valid_pl, in_valid_fd, in_valid_fdb_lo, in_valid_fdb_hi;
	(* keep = "true" *) wire we_idle_in = st_oh[S_IDLE] && in_valid_ctrl && !lw_ps_pipe_busy;
	 wire we_idle_ctrl, we_idle_fsm, we_idle_keep_lo, we_idle_keep_hi;
	 wire [4:0] iv_branch;
	 wire [20:0] we_in_lo, we_in_hi;
	 wire [6:0] we_vpr_lo, we_vpr_hi;

	`RX_BUF1(u_iv_ctrl,  in_valid, in_valid_ctrl);
	`RX_BUF1(u_iv_pl,    in_valid, in_valid_pl);
	`RX_BUF1(u_iv_fd,    in_valid, in_valid_fd);
	`RX_BUF1(u_iv_fdbl,  in_valid, in_valid_fdb_lo);
	`RX_BUF1(u_iv_fdbh,  in_valid, in_valid_fdb_hi);

	`RX_BUF1(u_wid_c,   we_idle_in, we_idle_ctrl);
	`RX_BUF1(u_wid_fsm, we_idle_in, we_idle_fsm);
	`RX_BUF1(u_wid_kl,  we_idle_in, we_idle_keep_lo);
	`RX_BUF1(u_wid_kh,  we_idle_in, we_idle_keep_hi);

`ifdef SYNTH_KINTEX7
	`RX_BUF1(u_iv_b0, we_idle_in, iv_branch[0]);
	`RX_BUF1(u_iv_b1, we_idle_in, iv_branch[1]);
	`RX_BUF1(u_iv_b2, we_idle_in, iv_branch[2]);
	`RX_BUF1(u_iv_b3, we_idle_in, iv_branch[3]);
	`RX_BUF1(u_iv_b4, we_idle_in, iv_branch[4]);
`elsif SYNTH_KINTEX_ULTRASCALE
	`RX_BUF1(u_iv_b0, we_idle_in, iv_branch[0]);
	`RX_BUF1(u_iv_b1, we_idle_in, iv_branch[1]);
	`RX_BUF1(u_iv_b2, we_idle_in, iv_branch[2]);
	`RX_BUF1(u_iv_b3, we_idle_in, iv_branch[3]);
	`RX_BUF1(u_iv_b4, we_idle_in, iv_branch[4]);
`else
	assign iv_branch[0] = r_st_idle_br[0] && in_valid_ctrl && !lw_ps_pipe_busy;
	assign iv_branch[1] = r_st_idle_br[1] && in_valid_ctrl && !lw_ps_pipe_busy;
	assign iv_branch[2] = r_st_idle_br[2] && in_valid_ctrl && !lw_ps_pipe_busy;
	assign iv_branch[3] = r_st_idle_br[3] && in_valid_ctrl && !lw_ps_pipe_busy;
	assign iv_branch[4] = r_st_idle_br[4] && in_valid_ctrl && !lw_ps_pipe_busy;
`endif

	genvar giv;
	generate
		for (giv = 0; giv < 21; giv = giv + 1) begin : g_we_in
			localparam integer BR = (giv < 6) ? 0 : (giv < 12) ? 1 : (giv < 18) ? 2 : 3;
			`RX_BUF1(u_wel, iv_branch[BR], we_in_lo[giv]);
			`RX_BUF1(u_weh, iv_branch[BR], we_in_hi[giv]);
		end
		for (giv = 0; giv < 7; giv = giv + 1) begin : g_we_vp
			localparam integer BRV = (giv < 3) ? 3 : 4;
			`RX_BUF1(u_wvpl, iv_branch[BRV], we_vpr_lo[giv]);
			`RX_BUF1(u_wvph, iv_branch[BRV], we_vpr_hi[giv]);
		end
	endgenerate

	 wire cl_ready_out, mesh_ready_out, busy_out;
	`RX_BUF1(u_cr_out, (st == S_IDLE) && !lw_ps_pipe_busy, cl_ready_out);
	`RX_BUF1(u_mr_out, (st == S_IDLE) && !lw_ps_pipe_busy, mesh_ready_out);
	`RX_BUF1(u_bs_out, (st != S_IDLE), busy_out);
	assign cl_ready = cl_ready_out;
	assign mesh_ready = mesh_ready_out;
	assign busy = busy_out;
	assign sp_last = lastq && !more;

	wire [15:0] cur_A_lo = ({16{r_p_plane_oh_a0[0]}} & pl_lo[0]) | ({16{r_p_plane_oh_a0[1]}} & pl_lo[3]) |
	                       ({16{r_p_plane_oh_a0[2]}} & pl_lo[6]) | ({16{r_p_plane_oh_a0[3]}} & pl_lo[9]);
	wire [15:0] cur_A_hi = ({16{r_p_plane_oh_b0[0]}} & pl_hi[0]) | ({16{r_p_plane_oh_b0[1]}} & pl_hi[3]) |
	                       ({16{r_p_plane_oh_b0[2]}} & pl_hi[6]) | ({16{r_p_plane_oh_b0[3]}} & pl_hi[9]);
	wire [31:0] cur_A = {cur_A_hi, cur_A_lo};

	wire [15:0] cur_B_lo = ({16{r_p_plane_oh_a1[0]}} & pl_lo[1]) | ({16{r_p_plane_oh_a1[1]}} & pl_lo[4]) |
	                       ({16{r_p_plane_oh_a1[2]}} & pl_lo[7]) | ({16{r_p_plane_oh_a1[3]}} & pl_lo[10]);
	wire [15:0] cur_B_hi = ({16{r_p_plane_oh_b1[0]}} & pl_hi[1]) | ({16{r_p_plane_oh_b1[1]}} & pl_hi[4]) |
	                       ({16{r_p_plane_oh_b1[2]}} & pl_hi[7]) | ({16{r_p_plane_oh_b1[3]}} & pl_hi[10]);
	wire [31:0] cur_B = {cur_B_hi, cur_B_lo};

	wire [15:0] R_pf0_lo = ({16{r_p_plane_oh_a2[0]}} & R_lo[33]) | ({16{r_p_plane_oh_a2[1]}} & R_lo[28]) |
	                       ({16{r_p_plane_oh_a2[2]}} & R_lo[52]) | ({16{r_p_plane_oh_a2[3]}} & R_lo[55]);
	wire [15:0] R_pf0_hi = ({16{r_p_plane_oh_b2[0]}} & R_hi[33]) | ({16{r_p_plane_oh_b2[1]}} & R_hi[28]) |
	                       ({16{r_p_plane_oh_b2[2]}} & R_hi[52]) | ({16{r_p_plane_oh_b2[3]}} & R_hi[55]);
	wire [31:0] R_pf0 = {R_pf0_hi, R_pf0_lo};

	wire [15:0] R_pf0_b_lo = ({16{r_p_plane_oh_a5[0]}} & R_lo[33]) | ({16{r_p_plane_oh_a5[1]}} & R_lo[28]) |
	                         ({16{r_p_plane_oh_a5[2]}} & R_lo[52]) | ({16{r_p_plane_oh_a5[3]}} & R_lo[55]);
	wire [15:0] R_pf0_b_hi = ({16{r_p_plane_oh_b5[0]}} & R_hi[33]) | ({16{r_p_plane_oh_b5[1]}} & R_hi[28]) |
	                         ({16{r_p_plane_oh_b5[2]}} & R_hi[52]) | ({16{r_p_plane_oh_b5[3]}} & R_hi[55]);
	wire [31:0] R_pf0_b = {R_pf0_b_hi, R_pf0_b_lo};

	wire [15:0] R_pf1_lo = ({16{r_p_plane_oh_a3[0]}} & R_lo[36]) | ({16{r_p_plane_oh_a3[1]}} & R_lo[29]) |
	                       ({16{r_p_plane_oh_a3[2]}} & R_lo[53]) | ({16{r_p_plane_oh_a3[3]}} & R_lo[56]);
	wire [15:0] R_pf1_hi = ({16{r_p_plane_oh_b3[0]}} & R_hi[36]) | ({16{r_p_plane_oh_b3[1]}} & R_hi[29]) |
	                       ({16{r_p_plane_oh_b3[2]}} & R_hi[53]) | ({16{r_p_plane_oh_b3[3]}} & R_hi[56]);
	wire [31:0] R_pf1 = {R_pf1_hi, R_pf1_lo};

	wire [15:0] R_pf2_lo = ({16{r_p_plane_oh_a4[0]}} & R_lo[39]) | ({16{r_p_plane_oh_a4[1]}} & R_lo[30]) |
	                       ({16{r_p_plane_oh_a4[2]}} & R_lo[54]) | ({16{r_p_plane_oh_a4[3]}} & R_lo[57]);
	wire [15:0] R_pf2_hi = ({16{r_p_plane_oh_b4[0]}} & R_hi[39]) | ({16{r_p_plane_oh_b4[1]}} & R_hi[30]) |
	                       ({16{r_p_plane_oh_b4[2]}} & R_hi[54]) | ({16{r_p_plane_oh_b4[3]}} & R_hi[57]);
	wire [31:0] R_pf2 = {R_pf2_hi, R_pf2_lo};

	always @(posedge clk) begin
		case (e_ra_b[0])
		7'd31: e_rd[3:0] <= R_lo[31][3:0];
		7'd32: e_rd[3:0] <= R_lo[32][3:0];
		7'd33: e_rd[3:0] <= R_lo[33][3:0];
		7'd34: e_rd[3:0] <= R_lo[34][3:0];
		7'd35: e_rd[3:0] <= R_lo[35][3:0];
		7'd36: e_rd[3:0] <= R_lo[36][3:0];
		7'd37: e_rd[3:0] <= R_lo[37][3:0];
		7'd38: e_rd[3:0] <= R_lo[38][3:0];
		7'd39: e_rd[3:0] <= R_lo[39][3:0];
		7'd76: e_rd[3:0] <= R_lo[76][3:0];
		7'd77: e_rd[3:0] <= R_lo[77][3:0];
		7'd78: e_rd[3:0] <= R_lo[78][3:0];
		7'd79: e_rd[3:0] <= R_lo[79][3:0];
		default: e_rd[3:0] <= 4'd0;
		endcase
		case (e_ra_b[1])
		7'd31: e_rd[7:4] <= R_lo[31][7:4];
		7'd32: e_rd[7:4] <= R_lo[32][7:4];
		7'd33: e_rd[7:4] <= R_lo[33][7:4];
		7'd34: e_rd[7:4] <= R_lo[34][7:4];
		7'd35: e_rd[7:4] <= R_lo[35][7:4];
		7'd36: e_rd[7:4] <= R_lo[36][7:4];
		7'd37: e_rd[7:4] <= R_lo[37][7:4];
		7'd38: e_rd[7:4] <= R_lo[38][7:4];
		7'd39: e_rd[7:4] <= R_lo[39][7:4];
		7'd76: e_rd[7:4] <= R_lo[76][7:4];
		7'd77: e_rd[7:4] <= R_lo[77][7:4];
		7'd78: e_rd[7:4] <= R_lo[78][7:4];
		7'd79: e_rd[7:4] <= R_lo[79][7:4];
		default: e_rd[7:4] <= 4'd0;
		endcase
		case (e_ra_b[2])
		7'd31: e_rd[11:8] <= R_lo[31][11:8];
		7'd32: e_rd[11:8] <= R_lo[32][11:8];
		7'd33: e_rd[11:8] <= R_lo[33][11:8];
		7'd34: e_rd[11:8] <= R_lo[34][11:8];
		7'd35: e_rd[11:8] <= R_lo[35][11:8];
		7'd36: e_rd[11:8] <= R_lo[36][11:8];
		7'd37: e_rd[11:8] <= R_lo[37][11:8];
		7'd38: e_rd[11:8] <= R_lo[38][11:8];
		7'd39: e_rd[11:8] <= R_lo[39][11:8];
		7'd76: e_rd[11:8] <= R_lo[76][11:8];
		7'd77: e_rd[11:8] <= R_lo[77][11:8];
		7'd78: e_rd[11:8] <= R_lo[78][11:8];
		7'd79: e_rd[11:8] <= R_lo[79][11:8];
		default: e_rd[11:8] <= 4'd0;
		endcase
		case (e_ra_b[3])
		7'd31: e_rd[15:12] <= R_lo[31][15:12];
		7'd32: e_rd[15:12] <= R_lo[32][15:12];
		7'd33: e_rd[15:12] <= R_lo[33][15:12];
		7'd34: e_rd[15:12] <= R_lo[34][15:12];
		7'd35: e_rd[15:12] <= R_lo[35][15:12];
		7'd36: e_rd[15:12] <= R_lo[36][15:12];
		7'd37: e_rd[15:12] <= R_lo[37][15:12];
		7'd38: e_rd[15:12] <= R_lo[38][15:12];
		7'd39: e_rd[15:12] <= R_lo[39][15:12];
		7'd76: e_rd[15:12] <= R_lo[76][15:12];
		7'd77: e_rd[15:12] <= R_lo[77][15:12];
		7'd78: e_rd[15:12] <= R_lo[78][15:12];
		7'd79: e_rd[15:12] <= R_lo[79][15:12];
		default: e_rd[15:12] <= 4'd0;
		endcase
		case (e_ra_b[4])
		7'd31: e_rd[19:16] <= R_hi[31][3:0];
		7'd32: e_rd[19:16] <= R_hi[32][3:0];
		7'd33: e_rd[19:16] <= R_hi[33][3:0];
		7'd34: e_rd[19:16] <= R_hi[34][3:0];
		7'd35: e_rd[19:16] <= R_hi[35][3:0];
		7'd36: e_rd[19:16] <= R_hi[36][3:0];
		7'd37: e_rd[19:16] <= R_hi[37][3:0];
		7'd38: e_rd[19:16] <= R_hi[38][3:0];
		7'd39: e_rd[19:16] <= R_hi[39][3:0];
		7'd76: e_rd[19:16] <= R_hi[76][3:0];
		7'd77: e_rd[19:16] <= R_hi[77][3:0];
		7'd78: e_rd[19:16] <= R_hi[78][3:0];
		7'd79: e_rd[19:16] <= R_hi[79][3:0];
		default: e_rd[19:16] <= 4'd0;
		endcase
		case (e_ra_b[5])
		7'd31: e_rd[23:20] <= R_hi[31][7:4];
		7'd32: e_rd[23:20] <= R_hi[32][7:4];
		7'd33: e_rd[23:20] <= R_hi[33][7:4];
		7'd34: e_rd[23:20] <= R_hi[34][7:4];
		7'd35: e_rd[23:20] <= R_hi[35][7:4];
		7'd36: e_rd[23:20] <= R_hi[36][7:4];
		7'd37: e_rd[23:20] <= R_hi[37][7:4];
		7'd38: e_rd[23:20] <= R_hi[38][7:4];
		7'd39: e_rd[23:20] <= R_hi[39][7:4];
		7'd76: e_rd[23:20] <= R_hi[76][7:4];
		7'd77: e_rd[23:20] <= R_hi[77][7:4];
		7'd78: e_rd[23:20] <= R_hi[78][7:4];
		7'd79: e_rd[23:20] <= R_hi[79][7:4];
		default: e_rd[23:20] <= 4'd0;
		endcase
		case (e_ra_b[6])
		7'd31: e_rd[27:24] <= R_hi[31][11:8];
		7'd32: e_rd[27:24] <= R_hi[32][11:8];
		7'd33: e_rd[27:24] <= R_hi[33][11:8];
		7'd34: e_rd[27:24] <= R_hi[34][11:8];
		7'd35: e_rd[27:24] <= R_hi[35][11:8];
		7'd36: e_rd[27:24] <= R_hi[36][11:8];
		7'd37: e_rd[27:24] <= R_hi[37][11:8];
		7'd38: e_rd[27:24] <= R_hi[38][11:8];
		7'd39: e_rd[27:24] <= R_hi[39][11:8];
		7'd76: e_rd[27:24] <= R_hi[76][11:8];
		7'd77: e_rd[27:24] <= R_hi[77][11:8];
		7'd78: e_rd[27:24] <= R_hi[78][11:8];
		7'd79: e_rd[27:24] <= R_hi[79][11:8];
		default: e_rd[27:24] <= 4'd0;
		endcase
		case (e_ra_b[7])
		7'd31: e_rd[31:28] <= R_hi[31][15:12];
		7'd32: e_rd[31:28] <= R_hi[32][15:12];
		7'd33: e_rd[31:28] <= R_hi[33][15:12];
		7'd34: e_rd[31:28] <= R_hi[34][15:12];
		7'd35: e_rd[31:28] <= R_hi[35][15:12];
		7'd36: e_rd[31:28] <= R_hi[36][15:12];
		7'd37: e_rd[31:28] <= R_hi[37][15:12];
		7'd38: e_rd[31:28] <= R_hi[38][15:12];
		7'd39: e_rd[31:28] <= R_hi[39][15:12];
		7'd76: e_rd[31:28] <= R_hi[76][15:12];
		7'd77: e_rd[31:28] <= R_hi[77][15:12];
		7'd78: e_rd[31:28] <= R_hi[78][15:12];
		7'd79: e_rd[31:28] <= R_hi[79][15:12];
		default: e_rd[31:28] <= 4'd0;
		endcase
	end

	/* ====================================================================
	 * Split Always Blocks for Fanout <= 24 (ASIC 5nm 1.5 GHz Rule 2)
	 * ==================================================================== */

	/* 1. FPU Input Registers (fm_a, fm_b, fa_a, fa_b) split into 8 x 4-bit slices */
	 wire [1:0] kind_s [0:7];
	 wire [7:0] rm3_s, ea_gt23_s;
	wire ea_gt_23 = ea_gt_23_r;
	wire [31:0] bm_val_32 = {bm_val_hi, bm_val_lo};
	wire [31:0] b_clamp_32 = {b_clamp_hi, b_clamp_lo};
	wire [31:0] r_bias_fa_32 = {r_bias_fa_hi, r_bias_fa_lo};
	wire [31:0] ea23_f32 = {1'b0, ea23_r, 23'd0};

	genvar gs;
	generate
	for (gs = 0; gs < 8; gs = gs + 1) begin : g_fpu_slice
		localparam integer B0 = gs * 4;
		localparam integer B1 = gs * 4 + 3;
		`RX_BUF1(u_kd0_s, kind_fpu[0], kind_s[gs][0]);
		`RX_BUF1(u_kd1_s, kind_fpu[1], kind_s[gs][1]);
		`RX_BUF1(u_rm3_s, rm[3],       rm3_s[gs]);
		`RX_BUF1(u_eag_s, ea_gt_23,    ea_gt23_s[gs]);

		(* extract_enable = "no", extract_reset = "no" *) reg [3:0] fm_a_sl, fm_b_sl, fa_a_sl, fa_b_sl;
		assign fm_a[B1:B0] = fm_a_sl;
		assign fm_b[B1:B0] = fm_b_sl;
		assign fa_a[B1:B0] = fa_a_sl;
		assign fa_b[B1:B0] = fa_b_sl;

		wire [3:0] r21_abs_sl = (gs == 7) ? {1'b0, R[21][30:28]} : R[21][B1:B0];
		wire [3:0] r22_abs_sl = (gs == 7) ? {1'b0, R[22][30:28]} : R[22][B1:B0];
		wire bcl_use_fa_sl = (gs < 4) ? bcl_use_fa_lo : bcl_use_fa_hi;

		wire [31:0] seq_oh_sl =
			(gs == 0) ? seq_oh_s0 : (gs == 1) ? seq_oh_s1 :
			(gs == 2) ? seq_oh_s2 : (gs == 3) ? seq_oh_s3 :
			(gs == 4) ? seq_oh_s4 : (gs == 5) ? seq_oh_s5 :
			(gs == 6) ? seq_oh_s6 : seq_oh_s7;
		wire [15:0] ps_oh_sl  = (gs < 2) ? p_step_oh_s0 : (gs < 4) ? p_step_oh_s1 : (gs < 6) ? p_step_oh_s2 : p_step_oh_s3;

		wire fma_pl_we_s, fma_px_hold_s, fma_px_fmr_s;
		wire fma_px_0, fma_px_1, fma_px_2, fma_px_3;
		wire fmb_r28_s, fmb_r21_s, fmb_r22_s, fmb_r25_s, fmb_r29_s, fmb_r30_s;
		wire ps_2_7_s, ps_3_6_s, ps_5_9_s, ps_4_8_12_s, ps_0_1_s;
		`RX_BUF1(u_fpx_0,  r_fma_st_px[gs], fma_px_0);
		`RX_BUF1(u_fpx_1,  r_fma_st_px[gs], fma_px_1);
		`RX_BUF1(u_fpx_2,  r_fma_st_px[gs], fma_px_2);
		`RX_BUF1(u_fpx_3,  r_fma_st_px[gs], fma_px_3);
		`RX_BUF1(u_fplw_s, r_fma_pl_we,   fma_pl_we_s);
		`RX_BUF1(u_fpxh_s, r_fma_px_hold, fma_px_hold_s);
		`RX_BUF1(u_fpxf_s, r_fma_px_fmr,  fma_px_fmr_s);
		`RX_BUF1(u_fr28_s, r_fmb_r28,     fmb_r28_s);
		`RX_BUF1(u_fr21_s, r_fmb_r21,     fmb_r21_s);
		`RX_BUF1(u_fr22_s, r_fmb_r22,     fmb_r22_s);
		`RX_BUF1(u_fr25_s, r_fmb_r25,     fmb_r25_s);
		`RX_BUF1(u_fr29_s, r_fmb_r29,     fmb_r29_s);
		`RX_BUF1(u_fr30_s, r_fmb_r30,     fmb_r30_s);
		`RX_BUF1(u_p27_s,  r_ps_2_7,      ps_2_7_s);
		`RX_BUF1(u_p36_s,  r_ps_3_6,      ps_3_6_s);
		`RX_BUF1(u_p59_s,  r_ps_5_9,      ps_5_9_s);
		`RX_BUF1(u_p48_s,  r_ps_4_8_12,   ps_4_8_12_s);
		`RX_BUF1(u_p01_s,  r_ps_0_1,      ps_0_1_s);

		wire fma_st_px     = fma_px_3;
		wire fma_st_rect   = r_fma_st_rect[gs] && seq_oh_sl[0];
		wire fma_st_edge   = r_fma_st_edge[gs];
		wire fma_st_pdiv   = r_fma_st_pdiv[gs] && (kind_s[gs] == 2'd0) && seq_oh_sl[1];
		wire fma_st_pl     = r_fma_st_pl[gs] && fma_pl_we_s;
		wire fma_st_bias_s = r_fma_st_bias[gs];
		wire fma_st_bias   = fma_st_bias_s && (seq_oh_sl[0] || seq_oh_sl[2] || seq_oh_sl[3]);
		wire fma_we_sl     = fma_st_px || fma_st_rect || fma_st_edge || fma_st_pdiv || fma_st_pl || fma_st_bias;

		wire [3:0] fma_g0 = ({4{fma_px_0 && seq_oh_sl[0]}}  & R[0][B1:B0])  | ({4{fma_px_0 && seq_oh_sl[3]}}  & R[1][B1:B0])  |
		                    ({4{fma_px_0 && seq_oh_sl[6]}}  & R[2][B1:B0])  | ({4{fma_px_0 && seq_oh_sl[9]}}  & R[12][B1:B0]);
		wire [3:0] fma_g1 = ({4{fma_px_1 && seq_oh_sl[10]}} & R[13][B1:B0]) | ({4{fma_px_1 && seq_oh_sl[11]}} & R[4][B1:B0])  |
		                    ({4{fma_px_1 && seq_oh_sl[14]}} & R[5][B1:B0])  | ({4{fma_px_1 && seq_oh_sl[17]}} & R[6][B1:B0]);
		wire [3:0] fma_g2 = ({4{fma_px_2 && seq_oh_sl[20]}} & R[15][B1:B0]) | ({4{fma_px_2 && seq_oh_sl[21]}} & R[16][B1:B0]) |
		                    ({4{fma_px_2 && seq_oh_sl[22]}} & R[8][B1:B0])  | ({4{fma_px_2 && seq_oh_sl[25]}} & R[9][B1:B0]);
		wire [3:0] fma_g3 = ({4{fma_px_3 && seq_oh_sl[28]}} & R[10][B1:B0]) | ({4{fma_px_3 && seq_oh_sl[31]}} & R[18][B1:B0]) |
		                    ({4{fma_px_3 && fma_px_fmr_s}}  & fm_r[B1:B0])  | ({4{fma_st_rect}} & R[19][B1:B0]);
		wire [3:0] fma_g4 = ({4{fma_st_edge}} & R[45][B1:B0]) | ({4{fma_st_pdiv}} & R[46][B1:B0]) |
		                    ({4{fma_st_pl && ps_2_7_s}} & T0[B1:B0]) | ({4{fma_st_pl && ps_3_6_s}} & T1[B1:B0]);
		wire [3:0] fma_g5 = ({4{fma_st_pl && ps_5_9_s}} & fa_r[B1:B0]) | ({4{fma_st_pl && ps_oh_sl[10]}} & cur_A[B1:B0]) |
		                    ({4{fma_st_pl && ps_oh_sl[11]}} & cur_B[B1:B0]) |
		                    ({4{fma_st_bias_s && seq_oh_sl[0]}} & bm_val_32[B1:B0]) |
		                    ({4{fma_st_bias_s && seq_oh_sl[2]}} & rfmt[B1:B0]) |
		                    ({4{fma_st_bias_s && seq_oh_sl[3] && ea_gt23_s[gs]}} & ea23_f32[B1:B0]);
		/* NOR2 -> NAND3 saves 3 cmos3 after fma_g0..g5 */
		wire [3:0] fma_g01_n = ~(fma_g0 | fma_g1);
		wire [3:0] fma_g23_n = ~(fma_g2 | fma_g3);
		wire [3:0] fma_g45_n = ~(fma_g4 | fma_g5);
		wire [3:0] fma_nxt_val = ~(fma_g01_n & fma_g23_n & fma_g45_n);

		wire [3:0] fmb_px_val =
			({4{fmb_r28_s}} & R[28][B1:B0]) | ({4{fmb_r21_s}} & R[21][B1:B0]) |
			({4{fmb_r22_s}} & R[22][B1:B0]) | ({4{fmb_r25_s}} & R[25][B1:B0]) |
			({4{fmb_r29_s}} & R[29][B1:B0]) | ({4{fmb_r30_s}} & R[30][B1:B0]);
		wire [3:0] fmb_pl_val =
			({4{ps_oh_sl[2]}}  & R[43][B1:B0]) |
			({4{ps_oh_sl[3]}}  & R[41][B1:B0]) |
			({4{ps_5_9_s}}     & R[47][B1:B0]) |
			({4{ps_oh_sl[6]}}  & R[40][B1:B0]) |
			({4{ps_oh_sl[7]}}  & R[42][B1:B0]) |
			({4{ps_oh_sl[10]}} & R[45][B1:B0]) |
			({4{ps_oh_sl[11]}} & R[46][B1:B0]);

		wire [3:0] fab_px_val =
			({4{seq_oh_sl[2] || seq_oh_sl[13] || seq_oh_sl[24]}} & R[23][B1:B0]) |
			({4{seq_oh_sl[5] || seq_oh_sl[16] || seq_oh_sl[27]}} & R[24][B1:B0]) |
			({4{seq_oh_sl[8] || seq_oh_sl[19] || seq_oh_sl[30]}} & R[26][B1:B0]);
		wire [3:0] fab_pl_val =
			({4{ps_0_1_s}}    & R_pf0[B1:B0]) |
			({4{ps_4_8_12_s}} & fm_r[B1:B0])  |
			({4{ps_oh_sl[13]}} & fa_r[B1:B0]);
		wire fab_st_px   = (st_fab_s[gs] == S_PERSP_XYZ) && fm_o_s[gs] && fma_px_hold_s;
		wire fab_st_rect = (st_fab_s[gs] == S_RECT) && ((seq_oh_sl[1] && fm_o_s[gs] && rm3_s[gs]) || ((seq_oh_sl[2] || seq_oh_sl[3] || seq_oh_sl[4]) && fa_o_s[gs]));
		wire fab_st_edge = (st_fab_s[gs] == S_EDGE);
		wire fab_st_pdiv = (st_fab_s[gs] == S_PLANE_DIV) && (kind_s[gs] == 2'd0) && (((seq_oh_sl[0] || seq_oh_sl[3]) && fm_o_s[gs]) || (seq_oh_sl[2] && fa_o_s[gs]));
		wire fab_st_pl   = (st_fab_s[gs] == S_PLANE) && (ps_oh_sl[0] || (ps_oh_sl[1] && fa_o_s[gs]) || (ps_4_8_12_s && fm_o_s[gs]) || (ps_oh_sl[13] && fa_o_s[gs]));
		wire fab_st_b4   = (st_fab_s[gs] == S_BIAS) && seq_oh_sl[4] && fm_o_s[gs];
		wire fab_st_b6   = (st_fab_s[gs] == S_BIAS) && seq_oh_sl[6];

		always @(posedge clk) begin
			if (fma_we_sl) fm_a_sl <= fma_nxt_val;

			case (st_fmb_s[gs])
				S_PERSP_XYZ:
					if (!fma_px_hold_s) fm_b_sl <= fmb_px_val;
				S_RECT:
					if (seq_oh_sl[0]) fm_b_sl <= R[30][B1:B0];
				S_EDGE:
					fm_b_sl <= inv_ps[B1:B0];
				S_PLANE_DIV:
					if ((kind_s[gs] == 2'd0) && seq_oh_sl[1]) fm_b_sl <= inv_ps[B1:B0];
				S_PLANE:
					if (fma_pl_we_s) fm_b_sl <= fmb_pl_val;
				S_BIAS:
					if (seq_oh_sl[1] || seq_oh_sl[2] || seq_oh_sl[3])
						fm_b_sl <= ({4{seq_oh_sl[1]}} & b_slope[B1:B0]) | ({4{seq_oh_sl[2] || seq_oh_sl[3]}} & b_const[B1:B0]);
				default: ;
			endcase

			case (st_faa_s[gs])
				S_PERSP_XYZ:
					if (fm_o_s[gs] && fma_px_hold_s)
						fa_a_sl <= fm_r[B1:B0];
				S_RECT:
					if ((seq_oh_sl[1] && fm_o_s[gs] && rm3_s[gs]) || ((seq_oh_sl[2] || seq_oh_sl[3] || seq_oh_sl[4]) && fa_o_s[gs]))
						fa_a_sl <= ((seq_oh_sl[3] || seq_oh_sl[4]) ? R[24][B1:B0] : R[23][B1:B0]);
				S_EDGE:
					fa_a_sl <= HALF[B1:B0];
				S_PLANE_DIV:
					if ((kind_s[gs] == 2'd0) && ((((seq_oh_sl[0] || seq_oh_sl[3]) && fm_o_s[gs]) || (seq_oh_sl[2] && fa_o_s[gs]))))
						fa_a_sl <= HALF[B1:B0];
				S_PLANE:
					if (ps_oh_sl[0] || (ps_oh_sl[1] && fa_o_s[gs]) || (ps_4_8_12_s && fm_o_s[gs]) || (ps_oh_sl[13] && fa_o_s[gs]))
						fa_a_sl <= ({4{ps_oh_sl[0]}} & R_pf1[B1:B0]) | ({4{ps_oh_sl[1]}} & R_pf2[B1:B0]) |
						           ({4{ps_4_8_12_s}} & T2[B1:B0])    | ({4{ps_oh_sl[13]}} & R_pf0_b[B1:B0]);
				S_BIAS:
					if ((seq_oh_sl[4] && fm_o_s[gs]) || seq_oh_sl[7])
						fa_a_sl <= (seq_oh_sl[7] ? pl[2][B1:B0] : T0[B1:B0]);
				default: ;
			endcase

			if (fab_st_px || fab_st_rect || fab_st_edge || fab_st_pdiv || fab_st_pl || fab_st_b4 || fab_st_b6) begin
				fa_b_sl <=
					({4{fab_st_px}}                  & fab_px_val) |
					({4{fab_st_rect}}                & ((seq_oh_sl[3] || seq_oh_sl[4]) ? r22_abs_sl : r21_abs_sl)) |
					({4{fab_st_edge}}                & R[45][B1:B0]) |
					({4{fab_st_pdiv}}                & (seq_oh_sl[2] ? R[46][B1:B0] : fm_r[B1:B0])) |
					({4{fab_st_pl}}                  & fab_pl_val) |
					({4{fab_st_b4}}                  & fm_r[B1:B0]) |
					({4{fab_st_b6 &&  bcl_use_fa_sl}} & r_bias_fa_32[B1:B0]) |
					({4{fab_st_b6 && !bcl_use_fa_sl}} & b_clamp_32[B1:B0]);
			end
		end
	end
	endgenerate

	/* 2. S_IDLE Preload Registers: R_lo[0:27], R_hi[0:27], keep, raw, kind*, lastq */
	wire [1:0] next_kind_idle = (in_raw || in_kind_hdr == 2'd3) ? 2'd2 : in_kind_hdr;
	genvar gri;
	generate
	for (gri = 0; gri < 21; gri = gri + 1) begin : g_r_idle_load
		always @(posedge clk) begin
			if (we_in_lo[gri]) R_lo[gri] <= in_tri[gri * 32 +: 16];
			if (we_in_hi[gri]) R_hi[gri] <= in_tri[gri * 32 + 16 +: 16];
		end
	end
	endgenerate

	always @(posedge clk) begin
		if (we_idle_ctrl) begin
			raw <= in_raw;
			kind <= next_kind_idle;
			kind_prim <= next_kind_idle;
			kind_out <= next_kind_idle;
			kind_edge <= next_kind_idle;
			kind_fpu <= next_kind_idle;
			lastq <= in_last;
		end
		if (we_idle_keep_lo) keep[13:0] <= in_keep_hdr[13:0];
		if (we_idle_keep_hi) keep[26:14] <= in_keep_hdr[26:14];

		if (we_vpr_lo[0]) R_lo[21] <= cur_vp_lo[0];
		if (we_vpr_hi[0]) R_hi[21] <= cur_vp_hi[0];
		if (we_vpr_lo[1]) R_lo[22] <= cur_vp_lo[1];
		if (we_vpr_hi[1]) R_hi[22] <= cur_vp_hi[1];
		if (we_vpr_lo[2]) R_lo[23] <= cur_vp_lo[2];
		if (we_vpr_hi[2]) R_hi[23] <= cur_vp_hi[2];
		if (we_vpr_lo[3]) R_lo[24] <= cur_vp_lo[3];
		if (we_vpr_hi[3]) R_hi[24] <= cur_vp_hi[3];
		if (we_vpr_lo[4]) R_lo[25] <= cur_vp_lo[4];
		if (we_vpr_hi[4]) R_hi[25] <= cur_vp_hi[4];
		if (we_vpr_lo[5]) R_lo[26] <= cur_vp_lo[5];
		if (we_vpr_hi[5]) R_hi[26] <= cur_vp_hi[5];
		if (we_vpr_lo[6]) R_lo[27] <= ONE[15:0];
		if (we_vpr_hi[6]) R_hi[27] <= ONE[31:16];
	end

	/* 3. R_lo[28:39, 47, 52:57, 76:79] and R_hi[28:39, 47, 52:57, 76:79]
	 * Split write enables into 8-bit halves (lo0/lo1/hi0/hi1) so max fanout <= 16 */
	 wire in_raw_b0;
	`RX_BUF1(u_in_raw_b0, in_raw, in_raw_b0);
	(* keep = "true" *) wire raw_load_b0 = (st_rep_c == S_IDLE) && in_valid_ctrl && in_raw_b0;
	(* keep = "true" *) wire raw_load_b1 = (st_rep_d == S_IDLE) && in_valid_fd   && in_raw_b0;
	 wire [8:0] we_raw_r_lo0, we_raw_r_lo1, we_raw_r_hi0, we_raw_r_hi1;
	genvar grw;
	generate
		for (grw = 0; grw < 9; grw = grw + 1) begin : g_we_raw_r
			`RX_BUF1(u_wrl0, raw_load_b0, we_raw_r_lo0[grw]);
			`RX_BUF1(u_wrl1, raw_load_b0, we_raw_r_lo1[grw]);
			`RX_BUF1(u_wrh0, raw_load_b1, we_raw_r_hi0[grw]);
			`RX_BUF1(u_wrh1, raw_load_b1, we_raw_r_hi1[grw]);
		end
	endgenerate

	wire line_dup_cond = (st_R_lo1 == S_RECT) && (kind == 2'd1) &&
		(((seq_oh_s1[1]) && fm_o_R_lo && !rm[3]) || ((|seq_oh_s1[7:5]) && fa_o_R_lo));
	 wire we_ldup_r30_lo0, we_ldup_r30_lo1, we_ldup_r30_hi0, we_ldup_r30_hi1;
	 wire we_ldup_r39_lo0, we_ldup_r39_lo1, we_ldup_r39_hi0, we_ldup_r39_hi1;
	 wire we_ldup_r54_lo0, we_ldup_r54_lo1, we_ldup_r54_hi0, we_ldup_r54_hi1;
	 wire we_ldup_r57_lo0, we_ldup_r57_lo1, we_ldup_r57_hi0, we_ldup_r57_hi1;
	`RX_BUF1(u_ld30l0, line_dup_cond, we_ldup_r30_lo0);
	`RX_BUF1(u_ld30l1, line_dup_cond, we_ldup_r30_lo1);
	`RX_BUF1(u_ld30h0, line_dup_cond, we_ldup_r30_hi0);
	`RX_BUF1(u_ld30h1, line_dup_cond, we_ldup_r30_hi1);
	`RX_BUF1(u_ld39l0, line_dup_cond, we_ldup_r39_lo0);
	`RX_BUF1(u_ld39l1, line_dup_cond, we_ldup_r39_lo1);
	`RX_BUF1(u_ld39h0, line_dup_cond, we_ldup_r39_hi0);
	`RX_BUF1(u_ld39h1, line_dup_cond, we_ldup_r39_hi1);
	`RX_BUF1(u_ld54l0, line_dup_cond, we_ldup_r54_lo0);
	`RX_BUF1(u_ld54l1, line_dup_cond, we_ldup_r54_lo1);
	`RX_BUF1(u_ld54h0, line_dup_cond, we_ldup_r54_hi0);
	`RX_BUF1(u_ld54h1, line_dup_cond, we_ldup_r54_hi1);
	`RX_BUF1(u_ld57l0, line_dup_cond, we_ldup_r57_lo0);
	`RX_BUF1(u_ld57l1, line_dup_cond, we_ldup_r57_lo1);
	`RX_BUF1(u_ld57h0, line_dup_cond, we_ldup_r57_hi0);
	`RX_BUF1(u_ld57h1, line_dup_cond, we_ldup_r57_hi1);

	wire pw_base_cond = (st_R_lo0 == S_PERSP_W) && fd_o_w;
	 wire we_r28_lo, we_r28_hi, we_r29_lo, we_r29_hi;
	 wire we_r30_pw_lo0, we_r30_pw_lo1, we_r30_pw_hi0, we_r30_pw_hi1;
	`RX_BUF1(u_wr28l,  pw_base_cond && seq_oh_s0[0], we_r28_lo);
	`RX_BUF1(u_wr28h,  pw_base_cond && seq_oh_s2[0], we_r28_hi);
	`RX_BUF1(u_wr29l,  pw_base_cond && seq_oh_s0[1], we_r29_lo);
	`RX_BUF1(u_wr29h,  pw_base_cond && seq_oh_s2[1], we_r29_hi);
	`RX_BUF1(u_wr30l0, pw_base_cond && (|seq_oh_s0[4:2]), we_r30_pw_lo0);
	`RX_BUF1(u_wr30l1, pw_base_cond && (|seq_oh_s0[4:2]), we_r30_pw_lo1);
	`RX_BUF1(u_wr30h0, pw_base_cond && (|seq_oh_s2[4:2]), we_r30_pw_hi0);
	`RX_BUF1(u_wr30h1, pw_base_cond && (|seq_oh_s2[4:2]), we_r30_pw_hi1);

	wire pxyz_fa_lo0 = (st_R_lo0 == S_PERSP_XYZ) && fa_o_R_lo;
	wire pxyz_fa_hi0 = (st_R_hi0 == S_PERSP_XYZ) && fa_o_R_hi;
	wire pxyz_fa_lo1 = (st_R_lo1 == S_PERSP_XYZ) && fa_o_R_lo;
	wire pxyz_fa_hi1 = (st_R_hi1 == S_PERSP_XYZ) && fa_o_R_hi;
	 wire [8:0] we_pxyz_lo0, we_pxyz_lo1, we_pxyz_hi0, we_pxyz_hi1;
	`RX_BUF1(u_wpx0l0, pxyz_fa_lo0 && seq_oh_s0[3],  we_pxyz_lo0[0]);
	`RX_BUF1(u_wpx0l1, pxyz_fa_lo0 && seq_oh_s0[3],  we_pxyz_lo1[0]);
	`RX_BUF1(u_wpx0h0, pxyz_fa_hi0 && seq_oh_s2[3],  we_pxyz_hi0[0]);
	`RX_BUF1(u_wpx0h1, pxyz_fa_hi0 && seq_oh_s2[3],  we_pxyz_hi1[0]);
	`RX_BUF1(u_wpx1l0, pxyz_fa_lo0 && seq_oh_s0[6],  we_pxyz_lo0[1]);
	`RX_BUF1(u_wpx1l1, pxyz_fa_lo0 && seq_oh_s0[6],  we_pxyz_lo1[1]);
	`RX_BUF1(u_wpx1h0, pxyz_fa_hi0 && seq_oh_s2[6],  we_pxyz_hi0[1]);
	`RX_BUF1(u_wpx1h1, pxyz_fa_hi0 && seq_oh_s2[6],  we_pxyz_hi1[1]);
	`RX_BUF1(u_wpx2l0, pxyz_fa_lo0 && seq_oh_s0[9],  we_pxyz_lo0[2]);
	`RX_BUF1(u_wpx2l1, pxyz_fa_lo0 && seq_oh_s0[9],  we_pxyz_lo1[2]);
	`RX_BUF1(u_wpx2h0, pxyz_fa_hi0 && seq_oh_s2[9],  we_pxyz_hi0[2]);
	`RX_BUF1(u_wpx2h1, pxyz_fa_hi0 && seq_oh_s2[9],  we_pxyz_hi1[2]);
	`RX_BUF1(u_wpx3l0, pxyz_fa_lo0 && seq_oh_s0[14], we_pxyz_lo0[3]);
	`RX_BUF1(u_wpx3l1, pxyz_fa_lo0 && seq_oh_s0[14], we_pxyz_lo1[3]);
	`RX_BUF1(u_wpx3h0, pxyz_fa_hi0 && seq_oh_s2[14], we_pxyz_hi0[3]);
	`RX_BUF1(u_wpx3h1, pxyz_fa_hi0 && seq_oh_s2[14], we_pxyz_hi1[3]);
	`RX_BUF1(u_wpx4l0, pxyz_fa_lo0 && seq_oh_s0[17], we_pxyz_lo0[4]);
	`RX_BUF1(u_wpx4l1, pxyz_fa_lo0 && seq_oh_s0[17], we_pxyz_lo1[4]);
	`RX_BUF1(u_wpx4h0, pxyz_fa_hi0 && seq_oh_s2[17], we_pxyz_hi0[4]);
	`RX_BUF1(u_wpx4h1, pxyz_fa_hi0 && seq_oh_s2[17], we_pxyz_hi1[4]);
	`RX_BUF1(u_wpx5l0, pxyz_fa_lo1 && seq_oh_s1[20], we_pxyz_lo0[5]);
	`RX_BUF1(u_wpx5l1, pxyz_fa_lo1 && seq_oh_s1[20], we_pxyz_lo1[5]);
	`RX_BUF1(u_wpx5h0, pxyz_fa_hi1 && seq_oh_s3[20], we_pxyz_hi0[5]);
	`RX_BUF1(u_wpx5h1, pxyz_fa_hi1 && seq_oh_s3[20], we_pxyz_hi1[5]);
	`RX_BUF1(u_wpx6l0, pxyz_fa_lo1 && seq_oh_s1[25], we_pxyz_lo0[6]);
	`RX_BUF1(u_wpx6l1, pxyz_fa_lo1 && seq_oh_s1[25], we_pxyz_lo1[6]);
	`RX_BUF1(u_wpx6h0, pxyz_fa_hi1 && seq_oh_s3[25], we_pxyz_hi0[6]);
	`RX_BUF1(u_wpx6h1, pxyz_fa_hi1 && seq_oh_s3[25], we_pxyz_hi1[6]);
	`RX_BUF1(u_wpx7l0, pxyz_fa_lo1 && seq_oh_s1[28], we_pxyz_lo0[7]);
	`RX_BUF1(u_wpx7l1, pxyz_fa_lo1 && seq_oh_s1[28], we_pxyz_lo1[7]);
	`RX_BUF1(u_wpx7h0, pxyz_fa_hi1 && seq_oh_s3[28], we_pxyz_hi0[7]);
	`RX_BUF1(u_wpx7h1, pxyz_fa_hi1 && seq_oh_s3[28], we_pxyz_hi1[7]);
	`RX_BUF1(u_wpx8l0, pxyz_fa_lo1 && seq_oh_s1[31], we_pxyz_lo0[8]);
	`RX_BUF1(u_wpx8l1, pxyz_fa_lo1 && seq_oh_s1[31], we_pxyz_lo1[8]);
	`RX_BUF1(u_wpx8h0, pxyz_fa_hi1 && seq_oh_s3[31], we_pxyz_hi0[8]);
	`RX_BUF1(u_wpx8h1, pxyz_fa_hi1 && seq_oh_s3[31], we_pxyz_hi1[8]);

	wire pxyz_fm_lo1 = (st_R_lo1 == S_PERSP_XYZ) && fm_o_R_lo;
	wire pxyz_fm_hi1 = (st_R_hi1 == S_PERSP_XYZ) && fm_o_R_hi;
	wire rect_fm_lo1 = (st_R_lo1 == S_RECT) && fm_o_R_lo;
	wire rect_fm_hi1 = (st_R_hi1 == S_RECT) && fm_o_R_hi;
	wire rect_fa_lo1 = (st_R_lo1 == S_RECT) && fa_o_R_lo;
	wire rect_fa_hi1 = (st_R_hi1 == S_RECT) && fa_o_R_hi;

	 wire we_r52_lo, we_r52_hi, we_r53_lo, we_r53_hi, we_r55_lo, we_r55_hi, we_r56_lo, we_r56_hi;
	 wire we_r54_fm_lo0, we_r54_fm_lo1, we_r54_fm_hi0, we_r54_fm_hi1;
	 wire we_r57_fm_lo0, we_r57_fm_lo1, we_r57_fm_hi0, we_r57_fm_hi1;
	 wire we_r76_lo, we_r76_hi, we_r77_lo, we_r77_hi, we_r78_lo, we_r78_hi, we_r79_lo, we_r79_hi;
	 wire we_r47_lo, we_r47_hi;

	`RX_BUF1(u_wr52l, pxyz_fm_lo1 && seq_oh_s1[10], we_r52_lo);
	`RX_BUF1(u_wr52h, pxyz_fm_hi1 && seq_oh_s3[10], we_r52_hi);
	`RX_BUF1(u_wr55l, pxyz_fm_lo1 && seq_oh_s1[11], we_r55_lo);
	`RX_BUF1(u_wr55h, pxyz_fm_hi1 && seq_oh_s3[11], we_r55_hi);
	`RX_BUF1(u_wr53l, pxyz_fm_lo1 && seq_oh_s1[21], we_r53_lo);
	`RX_BUF1(u_wr53h, pxyz_fm_hi1 && seq_oh_s3[21], we_r53_hi);
	`RX_BUF1(u_wr56l, pxyz_fm_lo1 && seq_oh_s1[22], we_r56_lo);
	`RX_BUF1(u_wr56h, pxyz_fm_hi1 && seq_oh_s3[22], we_r56_hi);

	`RX_BUF1(u_wr54l0, rect_fm_lo1 && seq_oh_s1[0], we_r54_fm_lo0);
	`RX_BUF1(u_wr54l1, rect_fm_lo1 && seq_oh_s1[0], we_r54_fm_lo1);
	`RX_BUF1(u_wr54h0, rect_fm_hi1 && seq_oh_s3[0], we_r54_fm_hi0);
	`RX_BUF1(u_wr54h1, rect_fm_hi1 && seq_oh_s3[0], we_r54_fm_hi1);
	`RX_BUF1(u_wr57l0, rect_fm_lo1 && seq_oh_s1[1], we_r57_fm_lo0);
	`RX_BUF1(u_wr57l1, rect_fm_lo1 && seq_oh_s1[1], we_r57_fm_lo1);
	`RX_BUF1(u_wr57h0, rect_fm_hi1 && seq_oh_s3[1], we_r57_fm_hi0);
	`RX_BUF1(u_wr57h1, rect_fm_hi1 && seq_oh_s3[1], we_r57_fm_hi1);

	`RX_BUF1(u_wr76l, rect_fa_lo1 && seq_oh_s1[2], we_r76_lo);
	`RX_BUF1(u_wr76h, rect_fa_hi1 && seq_oh_s3[2], we_r76_hi);
	`RX_BUF1(u_wr77l, rect_fa_lo1 && seq_oh_s1[3], we_r77_lo);
	`RX_BUF1(u_wr77h, rect_fa_hi1 && seq_oh_s3[3], we_r77_hi);
	`RX_BUF1(u_wr78l, rect_fa_lo1 && seq_oh_s1[4], we_r78_lo);
	`RX_BUF1(u_wr78h, rect_fa_hi1 && seq_oh_s3[4], we_r78_hi);
	`RX_BUF1(u_wr79l, rect_fa_lo1 && (|seq_oh_s1[7:5]), we_r79_lo);
	`RX_BUF1(u_wr79h, rect_fa_hi1 && (|seq_oh_s3[7:5]), we_r79_hi);

	`RX_BUF1(u_wr47l, (st_R_lo1 == S_PLANE_DIV) && (kind != 2'd0) && fd_o_plane_lo, we_r47_lo);
	`RX_BUF1(u_wr47h, (st_R_hi1 == S_PLANE_DIV) && (kind != 2'd0) && fd_o_plane_hi, we_r47_hi);

	always @(posedge clk) begin
		if (we_r28_lo) R_lo[28] <= fd_r[15:0];
		if (we_r29_lo) R_lo[29] <= fd_r[15:0];
		if (we_r30_pw_lo0) R_lo[30][7:0] <= fd_r[7:0];
		else if (we_ldup_r30_lo0) R_lo[30][7:0] <= R_lo[28][7:0];
		if (we_r30_pw_lo1) R_lo[30][15:8] <= fd_r[15:8];
		else if (we_ldup_r30_lo1) R_lo[30][15:8] <= R_lo[28][15:8];

		if (we_raw_r_lo0[0]) R_lo[31][7:0] <= in_tri_r31[7:0];
		else if (we_pxyz_lo0[0]) R_lo[31][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[0]) R_lo[31][15:8] <= in_tri_r31[15:8];
		else if (we_pxyz_lo1[0]) R_lo[31][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[1]) R_lo[32][7:0] <= in_tri_r31[39:32];
		else if (we_pxyz_lo0[1]) R_lo[32][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[1]) R_lo[32][15:8] <= in_tri_r31[47:40];
		else if (we_pxyz_lo1[1]) R_lo[32][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[2]) R_lo[33][7:0] <= in_tri_r31[71:64];
		else if (we_pxyz_lo0[2]) R_lo[33][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[2]) R_lo[33][15:8] <= in_tri_r31[79:72];
		else if (we_pxyz_lo1[2]) R_lo[33][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[3]) R_lo[34][7:0] <= in_tri_r31[135:128];
		else if (we_pxyz_lo0[3]) R_lo[34][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[3]) R_lo[34][15:8] <= in_tri_r31[143:136];
		else if (we_pxyz_lo1[3]) R_lo[34][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[4]) R_lo[35][7:0] <= in_tri_r31[167:160];
		else if (we_pxyz_lo0[4]) R_lo[35][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[4]) R_lo[35][15:8] <= in_tri_r31[175:168];
		else if (we_pxyz_lo1[4]) R_lo[35][15:8] <= fa_r_R[15:8];
	end

	always @(posedge clk) begin
		if (we_r28_hi) R_hi[28] <= fd_r[31:16];
		if (we_r29_hi) R_hi[29] <= fd_r[31:16];
		if (we_r30_pw_hi0) R_hi[30][7:0] <= fd_r[23:16];
		else if (we_ldup_r30_hi0) R_hi[30][7:0] <= R_hi[28][7:0];
		if (we_r30_pw_hi1) R_hi[30][15:8] <= fd_r[31:24];
		else if (we_ldup_r30_hi1) R_hi[30][15:8] <= R_hi[28][15:8];

		if (we_raw_r_hi0[0]) R_hi[31][7:0] <= in_tri_r31[23:16];
		else if (we_pxyz_hi0[0]) R_hi[31][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[0]) R_hi[31][15:8] <= in_tri_r31[31:24];
		else if (we_pxyz_hi1[0]) R_hi[31][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[1]) R_hi[32][7:0] <= in_tri_r31[55:48];
		else if (we_pxyz_hi0[1]) R_hi[32][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[1]) R_hi[32][15:8] <= in_tri_r31[63:56];
		else if (we_pxyz_hi1[1]) R_hi[32][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[2]) R_hi[33][7:0] <= in_tri_r31[87:80];
		else if (we_pxyz_hi0[2]) R_hi[33][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[2]) R_hi[33][15:8] <= in_tri_r31[95:88];
		else if (we_pxyz_hi1[2]) R_hi[33][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[3]) R_hi[34][7:0] <= in_tri_r31[151:144];
		else if (we_pxyz_hi0[3]) R_hi[34][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[3]) R_hi[34][15:8] <= in_tri_r31[159:152];
		else if (we_pxyz_hi1[3]) R_hi[34][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[4]) R_hi[35][7:0] <= in_tri_r31[183:176];
		else if (we_pxyz_hi0[4]) R_hi[35][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[4]) R_hi[35][15:8] <= in_tri_r31[191:184];
		else if (we_pxyz_hi1[4]) R_hi[35][15:8] <= fa_r_R[31:24];
	end

	/* 4. R_lo[36:39, 47, 52:57, 76:79] and R_hi[36:39, 47, 52:57, 76:79] */
	always @(posedge clk) begin
		if (we_raw_r_lo0[5]) R_lo[36][7:0] <= in_tri_r31[199:192];
		else if (we_pxyz_lo0[5]) R_lo[36][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[5]) R_lo[36][15:8] <= in_tri_r31[207:200];
		else if (we_pxyz_lo1[5]) R_lo[36][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[6]) R_lo[37][7:0] <= in_tri_r31[263:256];
		else if (we_pxyz_lo0[6]) R_lo[37][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[6]) R_lo[37][15:8] <= in_tri_r31[271:264];
		else if (we_pxyz_lo1[6]) R_lo[37][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[7]) R_lo[38][7:0] <= in_tri_r31[295:288];
		else if (we_pxyz_lo0[7]) R_lo[38][7:0] <= fa_r_R[7:0];
		if (we_raw_r_lo1[7]) R_lo[38][15:8] <= in_tri_r31[303:296];
		else if (we_pxyz_lo1[7]) R_lo[38][15:8] <= fa_r_R[15:8];

		if (we_raw_r_lo0[8]) R_lo[39][7:0] <= in_tri_r31[327:320];
		else if (we_pxyz_lo0[8]) R_lo[39][7:0] <= fa_r_R[7:0];
		else if (we_ldup_r39_lo0) R_lo[39][7:0] <= R_lo[33][7:0];
		if (we_raw_r_lo1[8]) R_lo[39][15:8] <= in_tri_r31[335:328];
		else if (we_pxyz_lo1[8]) R_lo[39][15:8] <= fa_r_R[15:8];
		else if (we_ldup_r39_lo1) R_lo[39][15:8] <= R_lo[33][15:8];

		if (we_r52_lo) R_lo[52] <= fm_r_R[15:0];
		if (we_r55_lo) R_lo[55] <= fm_r_R[15:0];
		if (we_r53_lo) R_lo[53] <= fm_r_R[15:0];
		if (we_r56_lo) R_lo[56] <= fm_r_R[15:0];

		if (we_ldup_r54_lo0) R_lo[54][7:0] <= R_lo[52][7:0];
		else if (we_r54_fm_lo0) R_lo[54][7:0] <= fm_r_R[7:0];
		if (we_ldup_r54_lo1) R_lo[54][15:8] <= R_lo[52][15:8];
		else if (we_r54_fm_lo1) R_lo[54][15:8] <= fm_r_R[15:8];

		if (we_ldup_r57_lo0) R_lo[57][7:0] <= R_lo[55][7:0];
		else if (we_r57_fm_lo0) R_lo[57][7:0] <= fm_r_R[7:0];
		if (we_ldup_r57_lo1) R_lo[57][15:8] <= R_lo[55][15:8];
		else if (we_r57_fm_lo1) R_lo[57][15:8] <= fm_r_R[15:8];

		if (we_r76_lo) R_lo[76] <= fa_r_R[15:0];
		if (we_r77_lo) R_lo[77] <= fa_r_R[15:0];
		if (we_r78_lo) R_lo[78] <= fa_r_R[15:0];
		if (we_r79_lo) R_lo[79] <= fa_r_R[15:0];
		if (we_r47_lo) R_lo[47] <= fd_r[15:0];
	end

	always @(posedge clk) begin
		if (we_raw_r_hi0[5]) R_hi[36][7:0] <= in_tri_r31[215:208];
		else if (we_pxyz_hi0[5]) R_hi[36][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[5]) R_hi[36][15:8] <= in_tri_r31[223:216];
		else if (we_pxyz_hi1[5]) R_hi[36][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[6]) R_hi[37][7:0] <= in_tri_r31[279:272];
		else if (we_pxyz_hi0[6]) R_hi[37][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[6]) R_hi[37][15:8] <= in_tri_r31[287:280];
		else if (we_pxyz_hi1[6]) R_hi[37][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[7]) R_hi[38][7:0] <= in_tri_r31[311:304];
		else if (we_pxyz_hi0[7]) R_hi[38][7:0] <= fa_r_R[23:16];
		if (we_raw_r_hi1[7]) R_hi[38][15:8] <= in_tri_r31[319:312];
		else if (we_pxyz_hi1[7]) R_hi[38][15:8] <= fa_r_R[31:24];

		if (we_raw_r_hi0[8]) R_hi[39][7:0] <= in_tri_r31[343:336];
		else if (we_pxyz_hi0[8]) R_hi[39][7:0] <= fa_r_R[23:16];
		else if (we_ldup_r39_hi0) R_hi[39][7:0] <= R_hi[33][7:0];
		if (we_raw_r_hi1[8]) R_hi[39][15:8] <= in_tri_r31[351:344];
		else if (we_pxyz_hi1[8]) R_hi[39][15:8] <= fa_r_R[31:24];
		else if (we_ldup_r39_hi1) R_hi[39][15:8] <= R_hi[33][15:8];

		if (we_r52_hi) R_hi[52] <= fm_r_R[31:16];
		if (we_r55_hi) R_hi[55] <= fm_r_R[31:16];
		if (we_r53_hi) R_hi[53] <= fm_r_R[31:16];
		if (we_r56_hi) R_hi[56] <= fm_r_R[31:16];

		if (we_ldup_r54_hi0) R_hi[54][7:0] <= R_hi[52][7:0];
		else if (we_r54_fm_hi0) R_hi[54][7:0] <= fm_r_R[23:16];
		if (we_ldup_r54_hi1) R_hi[54][15:8] <= R_hi[52][15:8];
		else if (we_r54_fm_hi1) R_hi[54][15:8] <= fm_r_R[31:24];

		if (we_ldup_r57_hi0) R_hi[57][7:0] <= R_hi[55][7:0];
		else if (we_r57_fm_hi0) R_hi[57][7:0] <= fm_r_R[23:16];
		if (we_ldup_r57_hi1) R_hi[57][15:8] <= R_hi[55][15:8];
		else if (we_r57_fm_hi1) R_hi[57][15:8] <= fm_r_R[31:24];

		if (we_r76_hi) R_hi[76] <= fa_r_R[31:16];
		if (we_r77_hi) R_hi[77] <= fa_r_R[31:16];
		if (we_r78_hi) R_hi[78] <= fa_r_R[31:16];
		if (we_r79_hi) R_hi[79] <= fa_r_R[31:16];
		if (we_r47_hi) R_hi[47] <= fd_r[31:16];
	end

	/* 5. Plane Registers pl_lo[0:11] and pl_hi[0:11] */
	wire pl_clr_cond = (st_pl_lo == S_IDLE) && in_valid_pl;
	wire pl_point_cond = (st_pl_lo == S_EDGE) && e_done && !e_empty_buf && (kind == 2'd0);
	wire pl_point_cond_lo, pl_point_cond_hi;
	`RX_BUF1(u_pplc_l, pl_point_cond, pl_point_cond_lo);
	`RX_BUF1(u_pplc_h, pl_point_cond, pl_point_cond_hi);
	 wire [11:0] clr_pl_lo, clr_pl_hi, pt_pl_lo, pt_pl_hi;
	 wire we_pl8_div_lo, we_pl8_div_hi, we_pl11_div_lo, we_pl11_div_hi;
	`RX_BUF1(u_wpl8dl,  (st_pl_lo == S_PLANE_DIV) && (kind == 2'd0) && ((seq_pl_lo == 5'd1) || (seq_pl_lo == 5'd2)) && fa_o_pl_lo, we_pl8_div_lo);
	`RX_BUF1(u_wpl8dh,  (st_pl_hi == S_PLANE_DIV) && (kind == 2'd0) && ((seq_pl_hi == 5'd1) || (seq_pl_hi == 5'd2)) && fa_o_pl_hi, we_pl8_div_hi);
	`RX_BUF1(u_wpl11dl, (st_pl_lo == S_PLANE_DIV) && (kind == 2'd0) && (seq_pl_lo == 5'd4) && fa_o_pl_lo, we_pl11_div_lo);
	`RX_BUF1(u_wpl11dh, (st_pl_hi == S_PLANE_DIV) && (kind == 2'd0) && (seq_pl_hi == 5'd4) && fa_o_pl_hi, we_pl11_div_hi);
	genvar gpc;
	generate
		for (gpc = 0; gpc < 12; gpc = gpc + 1) begin : g_pl_ctrl_buf
			`RX_BUF1(u_cpl_l, pl_clr_cond,      clr_pl_lo[gpc]);
			`RX_BUF1(u_cpl_h, pl_clr_cond,      clr_pl_hi[gpc]);
			`RX_BUF1(u_ppl_l, pl_point_cond_lo, pt_pl_lo[gpc]);
			`RX_BUF1(u_ppl_h, pl_point_cond_hi, pt_pl_hi[gpc]);
		end
	endgenerate

	always @(posedge clk or posedge rst_setup_high) begin
		if (rst_setup_high) begin
			pl_lo[0] <= 16'd0; pl_lo[1] <= 16'd0; pl_lo[2] <= 16'd0; pl_lo[3] <= 16'd0;
			pl_lo[4] <= 16'd0; pl_lo[5] <= 16'd0; pl_lo[6] <= 16'd0; pl_lo[7] <= 16'd0;
			pl_lo[8] <= 16'd0; pl_lo[9] <= 16'd0; pl_lo[10] <= 16'd0; pl_lo[11] <= 16'd0;
		end else begin
			if (clr_pl_lo[0] || pt_pl_lo[0]) pl_lo[0] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd6) && fm_o_pl_lo && (p_plane_pl_l == 2'd0)) pl_lo[0] <= fm_r_pl[15:0];

		if (clr_pl_lo[1] || pt_pl_lo[1]) pl_lo[1] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd10) && fm_o_pl_lo && (p_plane_pl_l == 2'd0)) pl_lo[1] <= fm_r_pl[15:0];

		if (clr_pl_lo[2]) pl_lo[2] <= 16'd0;
		else if (pt_pl_lo[2]) pl_lo[2] <= R_lo[33];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo >= 4'd14) && fa_o_pl_lo && (p_plane_pl_l == 2'd0)) pl_lo[2] <= fa_r_pl[15:0];
		else if ((st_pl_lo == S_BIAS) && (seq_pl_lo >= 5'd8) && fa_o_pl_lo) pl_lo[2] <= fa_r_pl[15:0];

		if (clr_pl_lo[3] || pt_pl_lo[3]) pl_lo[3] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd6) && fm_o_pl_lo && (p_plane_pl_l == 2'd1)) pl_lo[3] <= fm_r_pl[15:0];

		if (clr_pl_lo[4] || pt_pl_lo[4]) pl_lo[4] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd10) && fm_o_pl_lo && (p_plane_pl_l == 2'd1)) pl_lo[4] <= fm_r_pl[15:0];

		if (clr_pl_lo[5]) pl_lo[5] <= 16'd0;
		else if (pt_pl_lo[5]) pl_lo[5] <= ONE[15:0];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo >= 4'd14) && fa_o_pl_lo && (p_plane_pl_l == 2'd1)) pl_lo[5] <= fa_r_pl[15:0];

		if (clr_pl_lo[6]) pl_lo[6] <= 16'd0;
		else if (pt_pl_lo[6]) pl_lo[6] <= inv_ps[15:0];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd6) && fm_o_pl_lo && (p_plane_pl_l == 2'd2)) pl_lo[6] <= fm_r_pl[15:0];

		if (clr_pl_lo[7] || pt_pl_lo[7]) pl_lo[7] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd10) && fm_o_pl_lo && (p_plane_pl_l == 2'd2)) pl_lo[7] <= fm_r_pl[15:0];

		if (clr_pl_lo[8]) pl_lo[8] <= 16'd0;
		else if (we_pl8_div_lo) pl_lo[8] <= fa_r_pl[15:0];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo >= 4'd14) && fa_o_pl_lo && (p_plane_pl_l == 2'd2)) pl_lo[8] <= fa_r_pl[15:0];

		if (clr_pl_lo[9] || pt_pl_lo[9]) pl_lo[9] <= 16'd0;
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd6) && fm_o_pl_lo && (p_plane_pl_l == 2'd3)) pl_lo[9] <= fm_r_pl[15:0];

		if (clr_pl_lo[10]) pl_lo[10] <= 16'd0;
		else if (pt_pl_lo[10]) pl_lo[10] <= inv_ps[15:0];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo == 4'd10) && fm_o_pl_lo && (p_plane_pl_l == 2'd3)) pl_lo[10] <= fm_r_pl[15:0];

		if (clr_pl_lo[11]) pl_lo[11] <= 16'd0;
		else if (we_pl11_div_lo) pl_lo[11] <= fa_r_pl[15:0];
		else if ((st_pl_lo == S_PLANE) && (p_step_pl_lo >= 4'd14) && fa_o_pl_lo && (p_plane_pl_l == 2'd3)) pl_lo[11] <= fa_r_pl[15:0];
		end
	end

	always @(posedge clk or posedge rst_setup_high) begin
		if (rst_setup_high) begin
			pl_hi[0] <= 16'd0; pl_hi[1] <= 16'd0; pl_hi[2] <= 16'd0; pl_hi[3] <= 16'd0;
			pl_hi[4] <= 16'd0; pl_hi[5] <= 16'd0; pl_hi[6] <= 16'd0; pl_hi[7] <= 16'd0;
			pl_hi[8] <= 16'd0; pl_hi[9] <= 16'd0; pl_hi[10] <= 16'd0; pl_hi[11] <= 16'd0;
		end else begin
			if (clr_pl_hi[0] || pt_pl_hi[0]) pl_hi[0] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd6) && fm_o_pl_hi && (p_plane_pl_h == 2'd0)) pl_hi[0] <= fm_r_pl[31:16];

		if (clr_pl_hi[1] || pt_pl_hi[1]) pl_hi[1] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd10) && fm_o_pl_hi && (p_plane_pl_h == 2'd0)) pl_hi[1] <= fm_r_pl[31:16];

		if (clr_pl_hi[2]) pl_hi[2] <= 16'd0;
		else if (pt_pl_hi[2]) pl_hi[2] <= R_hi[33];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi >= 4'd14) && fa_o_pl_hi && (p_plane_pl_h == 2'd0)) pl_hi[2] <= fa_r_pl[31:16];
		else if ((st_pl_hi == S_BIAS) && (seq_pl_hi >= 5'd8) && fa_o_pl_hi) pl_hi[2] <= fa_r_pl[31:16];

		if (clr_pl_hi[3] || pt_pl_hi[3]) pl_hi[3] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd6) && fm_o_pl_hi && (p_plane_pl_h == 2'd1)) pl_hi[3] <= fm_r_pl[31:16];

		if (clr_pl_hi[4] || pt_pl_hi[4]) pl_hi[4] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd10) && fm_o_pl_hi && (p_plane_pl_h == 2'd1)) pl_hi[4] <= fm_r_pl[31:16];

		if (clr_pl_hi[5]) pl_hi[5] <= 16'd0;
		else if (pt_pl_hi[5]) pl_hi[5] <= ONE[31:16];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi >= 4'd14) && fa_o_pl_hi && (p_plane_pl_h == 2'd1)) pl_hi[5] <= fa_r_pl[31:16];

		if (clr_pl_hi[6]) pl_hi[6] <= 16'd0;
		else if (pt_pl_hi[6]) pl_hi[6] <= inv_ps[31:16];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd6) && fm_o_pl_hi && (p_plane_pl_h == 2'd2)) pl_hi[6] <= fm_r_pl[31:16];

		if (clr_pl_hi[7] || pt_pl_hi[7]) pl_hi[7] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd10) && fm_o_pl_hi && (p_plane_pl_h == 2'd2)) pl_hi[7] <= fm_r_pl[31:16];

		if (clr_pl_hi[8]) pl_hi[8] <= 16'd0;
		else if (we_pl8_div_hi) pl_hi[8] <= fa_r_pl[31:16];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi >= 4'd14) && fa_o_pl_hi && (p_plane_pl_h == 2'd2)) pl_hi[8] <= fa_r_pl[31:16];

		if (clr_pl_hi[9] || pt_pl_hi[9]) pl_hi[9] <= 16'd0;
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd6) && fm_o_pl_hi && (p_plane_pl_h == 2'd3)) pl_hi[9] <= fm_r_pl[31:16];

		if (clr_pl_hi[10]) pl_hi[10] <= 16'd0;
		else if (pt_pl_hi[10]) pl_hi[10] <= inv_ps[31:16];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi == 4'd10) && fm_o_pl_hi && (p_plane_pl_h == 2'd3)) pl_hi[10] <= fm_r_pl[31:16];

		if (clr_pl_hi[11]) pl_hi[11] <= 16'd0;
		else if (we_pl11_div_hi) pl_hi[11] <= fa_r_pl[31:16];
		else if ((st_pl_hi == S_PLANE) && (p_step_pl_hi >= 4'd14) && fa_o_pl_hi && (p_plane_pl_h == 2'd3)) pl_hi[11] <= fa_r_pl[31:16];
		end
	end

	/* 6. Temporary Registers T0, T1, T2 */
	always @(posedge clk) begin
		if ((st_T_lo == S_PLANE) && p_step_oh[1] && fa_o_pl_lo) T0_lo <= fa_r[15:0];
		else if ((st_T_lo == S_BIAS) && seq_oh[2] && fm_o_pl_lo) T0_lo <= fm_r[15:0];

		if ((st_T_lo == S_PLANE) && p_step_oh[2] && fa_o_pl_lo) T1_lo <= fa_r[15:0];

		if ((st_T_lo == S_PLANE) && (p_step_oh[3] || p_step_oh[7] || p_step_oh[11]) && fm_o_pl_lo)
			T2_lo <= fm_r[15:0];
	end

	always @(posedge clk) begin
		if ((st_T_hi == S_PLANE) && p_step_oh[1] && fa_o_pl_hi) T0_hi <= fa_r[31:16];
		else if ((st_T_hi == S_BIAS) && seq_oh[2] && fm_o_pl_hi) T0_hi <= fm_r[31:16];

		if ((st_T_hi == S_PLANE) && p_step_oh[2] && fa_o_pl_hi) T1_hi <= fa_r[31:16];

		if ((st_T_hi == S_PLANE) && (p_step_oh[3] || p_step_oh[7] || p_step_oh[11]) && fm_o_pl_hi)
			T2_hi <= fm_r[31:16];
	end

	/* 7. Divider Operands fd_b_lo/hi (split into 8-bit halves so st_fdb_* fanout <= 16) */
	always @(posedge clk) begin
		case (st_fdb_lo0)
		S_IDLE:
			if (!in_raw && in_valid_fdb_lo) fd_b_lo[7:0] <= in_tri[103:96];
		S_PERSP_W:
			if (fd_o_fdb_lo) begin
				if (seq_fdb_lo == 5'd0) fd_b_lo[7:0] <= R_lo[7][7:0];
				else if (seq_fdb_lo == 5'd1) fd_b_lo[7:0] <= R_lo[11][7:0];
			end
		S_EDGE:
			fd_b_lo[7:0] <= R_lo[44][7:0];
		default: ;
		endcase

		case (st_fdb_lo1)
		S_IDLE:
			if (!in_raw && in_valid_fdb_lo) fd_b_lo[15:8] <= in_tri[111:104];
		S_PERSP_W:
			if (fd_o_fdb_lo) begin
				if (seq_fdb_lo == 5'd0) fd_b_lo[15:8] <= R_lo[7][15:8];
				else if (seq_fdb_lo == 5'd1) fd_b_lo[15:8] <= R_lo[11][15:8];
			end
		S_EDGE:
			fd_b_lo[15:8] <= R_lo[44][15:8];
		default: ;
		endcase
	end

	always @(posedge clk) begin
		case (st_fdb_hi0)
		S_IDLE:
			if (!in_raw && in_valid_fdb_hi) fd_b_hi[7:0] <= in_tri[119:112];
		S_PERSP_W:
			if (fd_o_fdb_hi) begin
				if (seq_fdb_hi == 5'd0) fd_b_hi[7:0] <= R_hi[7][7:0];
				else if (seq_fdb_hi == 5'd1) fd_b_hi[7:0] <= R_hi[11][7:0];
			end
		S_EDGE:
			fd_b_hi[7:0] <= R_hi[44][7:0];
		default: ;
		endcase

		case (st_fdb_hi1)
		S_IDLE:
			if (!in_raw && in_valid_fdb_hi) fd_b_hi[15:8] <= in_tri[127:120];
		S_PERSP_W:
			if (fd_o_fdb_hi) begin
				if (seq_fdb_hi == 5'd0) fd_b_hi[15:8] <= R_hi[7][15:8];
				else if (seq_fdb_hi == 5'd1) fd_b_hi[15:8] <= R_hi[11][15:8];
			end
		S_EDGE:
			fd_b_hi[15:8] <= R_hi[44][15:8];
		default: ;
		endcase
	end

	/* 8. Depth Bias Clamp Pipeline Registers (fanout <= 16 per enable buffer) */
	wire bias_s5_raw = (st_T_lo == S_BIAS) && (seq_T_lo == 5'd5) && fa_o_c_v;
	 wire we_bias_s5_lo, we_bias_s5_hi, we_bias_s5_flg;
	`RX_BUF1(u_wbs5_l, bias_s5_raw, we_bias_s5_lo);
	`RX_BUF1(u_wbs5_h, bias_s5_raw, we_bias_s5_hi);
	`RX_BUF1(u_wbs5_f, bias_s5_raw, we_bias_s5_flg);

	wire s5_fa_nan = ((fa_r[30:23] == 8'hFF) && (fa_r[22:0] != 23'd0)) || bcl_nan;
	wire s5_force1 = bcl_is_zero || (!s5_fa_nan && (b_clamp[31] ? !fa_r[31] : (fa_r[31] || (fa_r[30:0] == 31'd0))));
	wire s5_chkmag = !bcl_is_zero && !s5_fa_nan && (b_clamp[31] ? fa_r[31] : (!fa_r[31] && (fa_r[30:0] != 31'd0)));

	always @(posedge clk) begin
		if (we_bias_s5_lo)  r_bias_fa_lo <= fa_r[15:0];
		if (we_bias_s5_hi)  r_bias_fa_hi <= fa_r[31:16];
		if (we_bias_s5_flg) begin
			r_bcl_is_zero_lo <= bcl_is_zero;
			r_bcl_sign_lo <= b_clamp[31];
			r_bcl_mag_lt_lo <= bcl_mag_lt;
			r_bcl_mag_eq_lo <= (fa_r[30:0] == b_clamp[30:0]);
			r_fa_nan_lo <= s5_fa_nan;
			r_fa_sign_lo <= fa_r[31];
			r_fa_is_zero_lo <= (fa_r[30:0] == 31'd0);

			r_bcl_is_zero_hi <= bcl_is_zero;
			r_bcl_sign_hi <= b_clamp[31];
			r_bcl_mag_lt_hi <= bcl_mag_lt;
			r_bcl_mag_eq_hi <= (fa_r[30:0] == b_clamp[30:0]);
			r_fa_nan_hi <= s5_fa_nan;
			r_fa_sign_hi <= fa_r[31];
			r_fa_is_zero_hi <= (fa_r[30:0] == 31'd0);
			r_bcl_use_fa_lo <= s5_force1;
			r_bcl_use_fa_hi <= s5_force1;
			r_bcl_chk_mag_lo <= s5_chkmag;
			r_bcl_chk_mag_hi <= s5_chkmag;
		end
	end

	/* 9. Main Control FSM: Compute st_next, seq_next, p_step_next ONCE combinationally
	 * so replicating st_rep_*, seq_step_rep_*, p_step_rep_* does NOT duplicate next-state logic cones! */
	wire ps_ne_0100 = r_ps_ne_0100;
	wire [2:0] seq_inc = {
		seq_step_rep_b[2] ^ (seq_step_rep_b[1] & seq_step_rep_b[0]),
		seq_step_rep_b[1] ^ seq_step_rep_b[0],
		~seq_step_rep_b[0]
	};
	wire seq_inc_co_unused = 1'b0;
	wire seq_inc_hi_co_unused = 1'b0;
	wire [4:3] seq_inc_hi = {
		seq_step_hi[4] ^ (&seq_step_hi[3:0]),
		seq_step_hi[3] ^ (&seq_step_hi[2:0])
	};
	(* keep = "true" *) reg [7:0] r_fma_st_px, r_fma_st_rect, r_fma_st_edge, r_fma_st_pdiv, r_fma_st_pl, r_fma_st_bias;
	(* keep = "true" *) reg xyz_use_fa_r, xyz_use_fa_hi_r;

	(* keep = "true" *) reg [3:0] s_plane_done_st_r;
	wire [3:0] s_plane_done_st_w = raw ? S_OUT :
		(p_plane != 2'd3) ? S_PLANE :
		(kind == 2'd1) ? S_OUT :
		(rm[0] && (rm[5:4] != 2'd0)) ? S_BIAS : S_OUT;
	wire seq_ge_2 = |seq_step_rep_a[4:1];
	wire seq_ge_5 = seq_step_rep_a[4] || seq_step_rep_a[3] || (seq_step_rep_a[2] && (|seq_step_rep_a[1:0]));
	wire seq_ge_8 = |seq_step_rep_a[4:3];
	wire p_step_ge_14 = r_ps_want_wrap;
	wire next_is_idle = (st == S_IDLE) ? !we_idle_fsm :
		(st == S_OUT) ? (sp_valid && sp_ready && !more) : 1'b0;

	/* 1-hot state transition logic (<= 6 cmos3 vs 9-branch case priority) */
	wire [8:0] st_leave;
	assign st_leave[S_IDLE]      = we_idle_fsm;
	assign st_leave[S_PERSP_W]   = fd_o_w && seq_ge_2;
	assign st_leave[S_PERSP_XYZ] = seq_oh[31] && fa_o_c_st;
	assign st_leave[S_RECT]      = (seq_oh[1] && fm_o_c_st && !rm[3]) || (seq_ge_5 && fa_o_c_st);
	assign st_leave[S_EDGE]      = e_done;
	assign st_leave[S_PLANE_DIV] = (kind == 2'd0) ? (seq_oh[4] && fa_o_c_st) : fd_o_plane_ctrl;
	assign st_leave[S_PLANE]     = p_step_ge_14 && fa_o_c_st;
	assign st_leave[S_BIAS]      = seq_ge_8 && fa_o_c_st;
	assign st_leave[S_OUT]       = sp_valid && sp_ready;

	wire st_enter_idle = st_oh[S_OUT]       && st_leave[S_OUT] && !more;
	wire st_enter_pw   = st_oh[S_IDLE]      && st_leave[S_IDLE] && !in_raw;
	wire st_enter_pxyz = st_oh[S_PERSP_W]   && st_leave[S_PERSP_W];
	wire st_enter_rect = st_oh[S_PERSP_XYZ] && st_leave[S_PERSP_XYZ];
	wire st_enter_edge = (st_oh[S_IDLE]     && st_leave[S_IDLE] && in_raw) ||
	                     (st_oh[S_RECT]     && st_leave[S_RECT]) ||
	                     (st_oh[S_OUT]      && st_leave[S_OUT] && more);
	wire st_enter_pdiv = st_oh[S_EDGE]      && st_leave[S_EDGE] && !e_empty_buf;
	wire st_enter_pl = st_oh[S_PLANE_DIV] && (kind != 2'd0) && fd_o_plane_ctrl;
	wire st_enter_pl_oh;
	`RX_BUF1(u_sepl_o, st_enter_pl, st_enter_pl_oh);
	wire st_enter_out  = (st_oh[S_EDGE]     && st_leave[S_EDGE] && e_empty_buf) ||
	                     (st_oh[S_PLANE_DIV] && (kind == 2'd0) && seq_oh[4] && fa_o_c_st) ||
	                     (st_oh[S_BIAS]     && st_leave[S_BIAS]);
	wire st_enter_pl_done = st_oh[S_PLANE]  && st_leave[S_PLANE];

	wire [8:0] st_oh_next = {
		(st_oh[S_OUT]       & ~st_leave[S_OUT])       | st_enter_out  | (st_enter_pl_done & (s_plane_done_st_r == S_OUT)),
		(st_oh[S_BIAS]      & ~st_leave[S_BIAS])      | (st_enter_pl_done & (s_plane_done_st_r == S_BIAS)),
		(st_oh[S_PLANE]     & ~st_leave[S_PLANE])     | st_enter_pl_oh| (st_enter_pl_done & (s_plane_done_st_r == S_PLANE)),
		(st_oh[S_PLANE_DIV] & ~st_leave[S_PLANE_DIV]) | st_enter_pdiv,
		(st_oh[S_EDGE]      & ~st_leave[S_EDGE])      | st_enter_edge,
		(st_oh[S_RECT]      & ~st_leave[S_RECT])      | st_enter_rect,
		(st_oh[S_PERSP_XYZ] & ~st_leave[S_PERSP_XYZ]) | st_enter_pxyz,
		(st_oh[S_PERSP_W]   & ~st_leave[S_PERSP_W])   | st_enter_pw,
		(st_oh[S_IDLE]      & ~st_leave[S_IDLE])      | st_enter_idle
	};
	(* keep = "true" *) wire [3:0] st_next = {
		st_oh_next[S_OUT],
		st_oh_next[S_EDGE]      | st_oh_next[S_PLANE_DIV] | st_oh_next[S_PLANE] | st_oh_next[S_BIAS],
		st_oh_next[S_PERSP_XYZ] | st_oh_next[S_RECT]      | st_oh_next[S_PLANE] | st_oh_next[S_BIAS],
		st_oh_next[S_PERSP_W]   | st_oh_next[S_RECT]      | st_oh_next[S_PLANE_DIV] | st_oh_next[S_BIAS]
	};

	wire fm_o_c_seq_nx, fa_o_c_seq_nx;
	`RX_BUF1(u_fmonx, fm_o_c_seq, fm_o_c_seq_nx);
	`RX_BUF1(u_faonx, fa_o_c_seq, fa_o_c_seq_nx);
	(* keep = "true" *) reg [2:0] seq_next_lo;
	always @(*) begin
		seq_next_lo = seq_step[2:0];
		case (st_rep_d)
			S_IDLE:
				seq_next_lo = 3'd0;
			S_PERSP_W:
				if (fd_o_w) begin
					if (seq_oh[0]) seq_next_lo = 3'd1;
					else if (seq_oh[1]) seq_next_lo = 3'd2;
					else seq_next_lo = 3'd0;
				end
			S_PERSP_XYZ:
				if (xyz_adv)
					seq_next_lo = seq_inc[2:0];
			S_RECT:
				if (seq_oh[0] && fm_o_c_seq_nx) seq_next_lo = 3'd1;
				else if (seq_oh[1] && fm_o_c_seq_nx && rm[3]) seq_next_lo = 3'd2;
				else if ((seq_oh[2] || seq_oh[3] || seq_oh[4]) && fa_o_c_seq_nx) seq_next_lo = seq_inc[2:0];
			S_EDGE:
				if (e_done && !e_empty_buf) begin
					if ((kind == 2'd0) && !r_ps_ne_0100) seq_next_lo = 3'd2;
					else seq_next_lo = 3'd0;
				end
			S_PLANE_DIV:
				if (kind == 2'd0) begin
					if      (seq_oh[0] && fm_o_c_seq_nx) seq_next_lo = 3'd1;
					else if (seq_oh[1] && fa_o_c_seq_nx) seq_next_lo = 3'd3;
					else if (seq_oh[2] && fa_o_c_seq_nx) seq_next_lo = 3'd4;
					else if (seq_oh[3] && fm_o_c_seq_nx) seq_next_lo = 3'd4;
				end else if (fd_o_plane_ctrl) begin
					seq_next_lo = 3'd0;
				end
			S_BIAS:
				if      (seq_oh[0] || seq_oh[1] || seq_oh[6] || seq_oh[7]) seq_next_lo = seq_inc[2:0];
				else if (seq_oh[2] && fm_o_c_seq_nx) seq_next_lo = r_bias_rm3 ? 3'd3 : 3'd4;
				else if (seq_oh[3]) seq_next_lo = 3'd4;
				else if (seq_oh[4] && fm_o_c_seq_nx) seq_next_lo = 3'd5;
				else if (seq_oh[5] && fa_o_c_seq_nx) seq_next_lo = 3'd6;
			default: ;
			endcase
	end

	wire seq_hi_set1 = st_oh[S_BIAS] && seq_oh[7];
	wire seq_hi_bias_clr = seq_oh[0] || seq_oh[1] || seq_oh[6] ||
		(seq_oh[2] && fm_o_c_seq_nx) || seq_oh[3] || (seq_oh[4] && fm_o_c_seq_nx) || (seq_oh[5] && fa_o_c_seq_nx);
	wire seq_hi_rect_clr = (seq_oh[0] && fm_o_c_seq_nx) || (seq_oh[1] && fm_o_c_seq_nx && rm[3]) ||
		((seq_oh[2] || seq_oh[3] || seq_oh[4]) && fa_o_c_seq_nx);
	wire seq_hi_pdiv_clr = (kind != 2'd0) ? fd_o_plane_ctrl :
		((seq_oh[0] && fm_o_c_seq_nx) || (seq_oh[1] && fa_o_c_seq_nx) || (seq_oh[2] && fa_o_c_seq_nx) || (seq_oh[3] && fm_o_c_seq_nx));
	wire seq_hi_clr0 = st_oh[S_IDLE] || (st_oh[S_PERSP_W] && fd_o_w) || (st_oh[S_RECT] && seq_hi_rect_clr) ||
		(st_oh[S_EDGE] && e_done && !e_empty_buf) || (st_oh[S_PLANE_DIV] && seq_hi_pdiv_clr) ||
		(st_oh[S_BIAS] && seq_hi_bias_clr);
	(* keep = "true" *) wire [4:3] seq_next_hi =
		seq_hi_set1 ? 2'd1 :
		(seq_hi_clr0 ? 2'd0 :
		((st_oh[S_PERSP_XYZ] && xyz_adv) ? seq_inc_hi[4:3] : seq_step_hi[4:3]));
	(* keep = "true" *) wire [4:0] seq_next = {seq_next_hi, seq_next_lo};
	wire xyz_adv = seq_oh[0] || (xyz_use_fa_r ? fa_o_c_seq : fm_o_c_seq);
	wire xyz_step_m1 = (seq_oh[2]  | seq_oh[5]  | seq_oh[8]) |
	                   (seq_oh[13] | seq_oh[16] | seq_oh[19]) |
	                   (seq_oh[24] | seq_oh[27] | seq_oh[30]);
	wire next_xyz_use_fa = st_oh[S_PERSP_XYZ] && (xyz_adv ? xyz_step_m1 : xyz_use_fa_r);

	wire [3:0] p_step_ctrl_inc = {
		p_step_ctrl[3] ^ (&p_step_ctrl[2:0]),
		p_step_ctrl[2] ^ (p_step_ctrl[1] & p_step_ctrl[0]),
		p_step_ctrl[1] ^ p_step_ctrl[0],
		~p_step_ctrl[0]
	};
	wire p_step_ctrl_inc_co_unused = 1'b0;

	wire [1:0] p_plane_inc;
	wire p_plane_inc_co_unused;
	am_add #(.W(2)) u_inc_p_plane (
		.a(p_plane),
		.b(2'd1),
		.ci(1'b0),
		.s(p_plane_inc),
		.co(p_plane_inc_co_unused)
	);

	wire [3:0] p_step_next = ps_enter_w ? 4'd0 :
	                         ps_wrap_w  ? 4'd0 :
	                         ps_adv_w   ? p_step_ctrl_inc : p_step_ctrl;

	/* Unchained 1-hot seq_oh_next and p_step_oh_next (<= 7 cmos3, zero post-next-oh OR trees) */
	wire ps_adv_w  = (st_rep_f == S_PLANE) && (p_step_oh[0] || (r_ps_want_fa && fa_o_c_ps) || (r_ps_want_fm && fm_o_c_ps));
	wire ps_wrap_w = (st_rep_f == S_PLANE) && r_ps_want_wrap && fa_o_c_ps;
	wire ps_enter_w = (st_rep_f == S_PLANE_DIV) && (kind != 2'd0) && fd_o_plane_ctrl;

	wire ps_want_fa_adv   = p_step_oh[0] | p_step_oh[1] | p_step_oh[4] | p_step_oh[8] | p_step_oh[12];
	wire ps_want_fm_adv   = p_step_oh[2] | p_step_oh[3] | p_step_oh[5] | p_step_oh[6] | p_step_oh[7] | p_step_oh[9] | p_step_oh[10] | p_step_oh[11];
	wire ps_want_wrap_adv = p_step_oh[13] | p_step_oh[14];
	wire fma_pl_we_adv    = p_step_oh[1] | p_step_oh[2] | p_step_oh[4] | p_step_oh[5] | p_step_oh[6] | p_step_oh[8] | p_step_oh[9] | p_step_oh[10];
	wire ps_2_7_adv       = p_step_oh[1] | p_step_oh[6];
	wire ps_3_6_adv       = p_step_oh[2] | p_step_oh[5];
	wire ps_5_9_adv       = p_step_oh[4] | p_step_oh[8];
	wire ps_4_8_12_adv    = p_step_oh[3] | p_step_oh[7] | p_step_oh[11];
	wire ps_0_1_adv       = p_step_oh[0];

	wire [3:0] ps_adv_q, ps_wrap_q, ps_enter_q;
	`RX_BUF1(u_psal0, ps_adv_w,   ps_adv_q[0]);
	`RX_BUF1(u_psal1, ps_adv_w,   ps_adv_q[1]);
	`RX_BUF1(u_psah0, ps_adv_w,   ps_adv_q[2]);
	`RX_BUF1(u_psah1, ps_adv_w,   ps_adv_q[3]);
	`RX_BUF1(u_pswl0, ps_wrap_w,  ps_wrap_q[0]);
	`RX_BUF1(u_pswl1, ps_wrap_w,  ps_wrap_q[1]);
	`RX_BUF1(u_pswh0, ps_wrap_w,  ps_wrap_q[2]);
	`RX_BUF1(u_pswh1, ps_wrap_w,  ps_wrap_q[3]);
	`RX_BUF1(u_psel0, ps_enter_w, ps_enter_q[0]);
	`RX_BUF1(u_psel1, ps_enter_w, ps_enter_q[1]);
	`RX_BUF1(u_pseh0, ps_enter_w, ps_enter_q[2]);
	`RX_BUF1(u_pseh1, ps_enter_w, ps_enter_q[3]);

	wire [15:0] p_step_oh_next = {
		ps_enter_q[3] ? 4'd0 : (ps_wrap_q[3] ? 4'd0 : (ps_adv_q[3] ? p_step_oh[14:11] : p_step_oh[15:12])),
		ps_enter_q[2] ? 4'd0 : (ps_wrap_q[2] ? 4'd0 : (ps_adv_q[2] ? p_step_oh[10:7]  : p_step_oh[11:8])),
		ps_enter_q[1] ? 4'd0 : (ps_wrap_q[1] ? 4'd0 : (ps_adv_q[1] ? p_step_oh[6:3]   : p_step_oh[7:4])),
		ps_enter_q[0] ? 4'd1 : (ps_wrap_q[0] ? 4'd1 : (ps_adv_q[0] ? {p_step_oh[2:0], 1'b0} : p_step_oh[3:0]))
	};

	wire [3:0] st_rep_d_l0, st_rep_d_l1, st_rep_d_h0, st_rep_d_h1;
	wire [3:0] seq_oh_nx;
	wire fm_o_c_seq_lo, fa_o_c_seq_lo, xyz_adv_h0, xyz_adv_h1;
	`RX_BUF1(u_fmosl, fm_o_c_seq, fm_o_c_seq_lo);
	`RX_BUF1(u_faosl, fa_o_c_seq, fa_o_c_seq_lo);
	`RX_BUF1(u_xyzh0, xyz_adv,    xyz_adv_h0);
	`RX_BUF1(u_xyzh1, xyz_adv,    xyz_adv_h1);
	genvar gsd;
	generate
		for (gsd = 0; gsd < 4; gsd = gsd + 1) begin : g_st_d_buf
			`RX_BUF1(u_srdl0, st_rep_d[gsd], st_rep_d_l0[gsd]);
			`RX_BUF1(u_srdl1, st_rep_d[gsd], st_rep_d_l1[gsd]);
			`RX_BUF1(u_srdh0, st_rep_d[gsd], st_rep_d_h0[gsd]);
			`RX_BUF1(u_srdh1, st_rep_d[gsd], st_rep_d_h1[gsd]);
			`RX_BUF1(u_sonx,  seq_oh[gsd],   seq_oh_nx[gsd]);
		end
	endgenerate

	reg [3:0] seq_oh_next_l0;
	always @(*) begin
		seq_oh_next_l0 = seq_oh[3:0];
		case (st_rep_d_l0)
			S_IDLE:
				seq_oh_next_l0 = 4'd1;
			S_PERSP_W:
				if (fd_o_w) seq_oh_next_l0 = seq_oh_nx[2] ? 4'd1 : {1'b0, seq_oh_nx[1:0], 1'b0};
			S_PERSP_XYZ:
				if (xyz_adv) seq_oh_next_l0 = {seq_oh_nx[2:0], seq_oh[31]};
			S_RECT:
				if (seq_oh_nx[0] && fm_o_c_seq_lo) seq_oh_next_l0 = 4'd2;
				else if (seq_oh_nx[1] && fm_o_c_seq_lo && rm[3]) seq_oh_next_l0 = 4'd4;
				else if ((seq_oh_nx[2] || seq_oh_nx[3] || seq_oh[4]) && fa_o_c_seq_lo) seq_oh_next_l0 = {seq_oh_nx[2:0], 1'b0};
			S_EDGE:
				if (e_done && !e_empty_buf)
					seq_oh_next_l0 = ((kind == 2'd0) && !r_ps_ne_0100) ? 4'd4 : 4'd1;
			S_PLANE_DIV:
				if (kind == 2'd0) begin
					if      (seq_oh_nx[0] && fm_o_c_seq_lo) seq_oh_next_l0 = 4'd2;
					else if (seq_oh_nx[1] && fa_o_c_seq_lo) seq_oh_next_l0 = 4'd8;
					else if (seq_oh_nx[2] && fa_o_c_seq_lo) seq_oh_next_l0 = 4'd0;
					else if (seq_oh_nx[3] && fm_o_c_seq_lo) seq_oh_next_l0 = 4'd0;
				end else if (fd_o_plane_ctrl) begin
					seq_oh_next_l0 = 4'd1;
				end
			S_BIAS:
				if      (seq_oh_nx[0] || seq_oh_nx[1] || seq_oh[6] || seq_oh[7]) seq_oh_next_l0 = {seq_oh_nx[2:0], 1'b0};
				else if (seq_oh_nx[2] && fm_o_c_seq_lo) seq_oh_next_l0 = r_bias_rm3 ? 4'd8 : 4'd0;
				else if (seq_oh_nx[3]) seq_oh_next_l0 = 4'd0;
				else if (seq_oh[4] && fm_o_c_seq_lo) seq_oh_next_l0 = 4'd0;
				else if (seq_oh[5] && fa_o_c_seq_lo) seq_oh_next_l0 = 4'd0;
			default: ;
		endcase
	end
	reg [7:4] seq_oh_next_l1;
	always @(*) begin
		seq_oh_next_l1 = seq_oh[7:4];
		case (st_rep_d_l1)
			S_IDLE, S_PERSP_W, S_EDGE:
				if ((st_rep_d_l1 == S_IDLE) || (st_rep_d_l1 == S_PERSP_W && fd_o_w) || (st_rep_d_l1 == S_EDGE && e_done && !e_empty_buf))
					seq_oh_next_l1 = 4'd0;
			S_PERSP_XYZ:
				if (xyz_adv) seq_oh_next_l1 = seq_oh[6:3];
			S_RECT:
				if ((seq_oh_nx[0] && fm_o_c_seq_lo) || (seq_oh_nx[1] && fm_o_c_seq_lo && rm[3])) seq_oh_next_l1 = 4'd0;
				else if ((seq_oh_nx[2] || seq_oh_nx[3] || seq_oh[4]) && fa_o_c_seq_lo) seq_oh_next_l1 = seq_oh[6:3];
			S_PLANE_DIV:
				if (kind == 2'd0) begin
					if      (seq_oh_nx[0] && fm_o_c_seq_lo) seq_oh_next_l1 = 4'd0;
					else if (seq_oh_nx[1] && fa_o_c_seq_lo) seq_oh_next_l1 = 4'd0;
					else if (seq_oh_nx[2] && fa_o_c_seq_lo) seq_oh_next_l1 = 4'd1;
					else if (seq_oh_nx[3] && fm_o_c_seq_lo) seq_oh_next_l1 = 4'd1;
				end else if (fd_o_plane_ctrl) begin
					seq_oh_next_l1 = 4'd0;
				end
			S_BIAS:
				if      (seq_oh_nx[0] || seq_oh_nx[1] || seq_oh[6] || seq_oh[7]) seq_oh_next_l1 = seq_oh[6:3];
				else if (seq_oh_nx[2] && fm_o_c_seq_lo) seq_oh_next_l1 = r_bias_rm3 ? 4'd0 : 4'd1;
				else if (seq_oh_nx[3]) seq_oh_next_l1 = 4'd1;
				else if (seq_oh[4] && fm_o_c_seq_lo) seq_oh_next_l1 = 4'd2;
				else if (seq_oh[5] && fa_o_c_seq_lo) seq_oh_next_l1 = 4'd4;
			default: ;
		endcase
	end
	wire [7:0] seq_oh_next_lo = {seq_oh_next_l1, seq_oh_next_l0};
	wire [19:8] seq_oh_next_m = (st_rep_d_h0 == S_IDLE || st_rep_d_h0 == S_PLANE_DIV || st_rep_d_h0 == S_EDGE) ? 12'd0 :
		((st_rep_d_h0 == S_PERSP_XYZ && xyz_adv_h0) ? seq_oh[18:7] : seq_oh[19:8]);
	wire [31:20] seq_oh_next_hi = (st_rep_d_h1 == S_IDLE || st_rep_d_h1 == S_PLANE_DIV || st_rep_d_h1 == S_EDGE) ? 12'd0 :
		((st_rep_d_h1 == S_PERSP_XYZ && xyz_adv_h1) ? seq_oh[30:19] : seq_oh[31:20]);
	wire [31:0] seq_oh_next = {seq_oh_next_hi, seq_oh_next_m, seq_oh_next_lo};

	/* Compute S_PERSP_XYZ FPU slice flags in parallel with xyz_adv (no post-seq_oh_next OR chain!) */
	wire xyz_enter_w = (st_rep_d == S_PERSP_W) && fd_o_w && seq_oh[2];
	wire fma_px_hold_adv = seq_oh[1]  | seq_oh[4]  | seq_oh[7]  |
	                       seq_oh[12] | seq_oh[15] | seq_oh[18] |
	                       seq_oh[23] | seq_oh[26] | seq_oh[29];
	wire fma_px_fmr_adv  = seq_oh[0]  | seq_oh[3]  | seq_oh[6]  |
	                       seq_oh[11] | seq_oh[14] | seq_oh[17] |
	                       seq_oh[22] | seq_oh[25] | seq_oh[28];
	wire fmb_r28_adv     = seq_oh[2]  | seq_oh[5]  | seq_oh[8]  | seq_oh[9]  | seq_oh[31];
	wire fmb_r21_adv     = seq_oh[0]  | seq_oh[11] | seq_oh[22];
	wire fmb_r22_adv     = seq_oh[3]  | seq_oh[14] | seq_oh[25];
	wire fmb_r25_adv     = seq_oh[6]  | seq_oh[17] | seq_oh[28];
	wire fmb_r29_adv     = seq_oh[10] | seq_oh[13] | seq_oh[16] | seq_oh[19] | seq_oh[20];
	wire fmb_r30_adv     = seq_oh[21] | seq_oh[24] | seq_oh[27] | seq_oh[30];

	wire fma_px_hold_nxt = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fma_px_hold_adv : r_fma_px_hold);
	wire fma_px_fmr_nxt  = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fma_px_fmr_adv  : r_fma_px_fmr);
	wire fmb_r28_nxt     = xyz_enter_w ? 1'b1 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r28_adv     : r_fmb_r28);
	wire fmb_r21_nxt     = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r21_adv     : r_fmb_r21);
	wire fmb_r22_nxt     = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r22_adv     : r_fmb_r22);
	wire fmb_r25_nxt     = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r25_adv     : r_fmb_r25);
	wire fmb_r29_nxt     = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r29_adv     : r_fmb_r29);
	wire fmb_r30_nxt     = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? fmb_r30_adv     : r_fmb_r30);

	/* Pre-registered S_PERSP_XYZ fm_v flags (balanced 4-stage tree, <= 7 cmos3) */
	reg r_xyz_fmv_fm, r_xyz_fmv_fa;
	wire [6:0] xyz_fm2_n = {
		!seq_oh[28],
		!(seq_oh[22] | seq_oh[25]),
		!(seq_oh[20] | seq_oh[21]),
		!(seq_oh[14] | seq_oh[17]),
		!(seq_oh[10] | seq_oh[11]),
		!(seq_oh[6]  | seq_oh[9]),
		!(seq_oh[0]  | seq_oh[3])
	};
	wire [2:0] xyz_fm4 = {
		!(xyz_fm2_n[5] & xyz_fm2_n[4]),
		!(xyz_fm2_n[3] & xyz_fm2_n[2]),
		!(xyz_fm2_n[1] & xyz_fm2_n[0])
	};
	wire xyz_fmv_fm_adv = !(!(xyz_fm4[2] | xyz_fm4[1]) & !(xyz_fm4[0] | !xyz_fm2_n[6]));
	wire [3:0] xyz_fa2_n = {
		!(seq_oh[24] | seq_oh[27]),
		!(seq_oh[16] | seq_oh[19]),
		!(seq_oh[8]  | seq_oh[13]),
		!(seq_oh[2]  | seq_oh[5])
	};
	wire [1:0] xyz_fa4 = {!(xyz_fa2_n[3] & xyz_fa2_n[2]), !(xyz_fa2_n[1] & xyz_fa2_n[0])};
	wire xyz_fmv_fa_adv = xyz_fa4[1] | xyz_fa4[0] | seq_oh[30];
	wire xyz_fmv_fm_nxt = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? xyz_fmv_fm_adv : r_xyz_fmv_fm);
	wire xyz_fmv_fa_nxt = xyz_enter_w ? 1'b0 : ((st_rep_d == S_PERSP_XYZ && xyz_adv) ? xyz_fmv_fa_adv : r_xyz_fmv_fa);

	wire ps_clr_raw = ps_enter_w || ps_wrap_w;
	wire ps_clr_r, ps_adv_r;
	`RX_BUF1(u_psclrr, ps_clr_raw, ps_clr_r);
	`RX_BUF1(u_psadvr, ps_adv_w,   ps_adv_r);

	always @(posedge clk or posedge rst_setup_high) begin
		if (rst_setup_high) begin
			st <= S_IDLE;
			st_oh <= 9'd1;
			st_rep_a <= S_IDLE;
			st_rep_b <= S_IDLE;
			st_rep_c <= S_IDLE;
			st_rep_d <= S_IDLE;
			st_rep_e <= S_IDLE;
			st_rep_f <= S_IDLE;
			r_st_idle_br <= 5'b11111;

			seq_step <= 5'd0;
			seq_step_hi <= 5'd0;
			seq_step_v <= 5'd0;
			seq_step_rep_a <= 5'd0;
			seq_step_rep_b <= 5'd0;
			seq_step_rep_c <= 5'd0;
			seq_step_rep_d <= 5'd0;
			seq_oh <= 32'd1;
			seq_oh_s0 <= 32'd1;
			seq_oh_s1 <= 32'd1;
			seq_oh_s2 <= 32'd1;
			seq_oh_s3 <= 32'd1;
			seq_oh_s4 <= 32'd1;
			seq_oh_s5 <= 32'd1;
			seq_oh_s6 <= 32'd1;
			seq_oh_s7 <= 32'd1;
			p_step_oh <= 16'd1;
			p_step_oh_s0 <= 16'd1;
			p_step_oh_s1 <= 16'd1;
			p_step_oh_s2 <= 16'd1;
			p_step_oh_s3 <= 16'd1;
			r_fma_px_hold <= 1'b0;
			r_fma_px_fmr  <= 1'b0;
			r_fmb_r28 <= 1'b1;
			r_fmb_r21 <= 1'b0;
			r_fmb_r22 <= 1'b0;
			r_fmb_r25 <= 1'b0;
			r_fmb_r29 <= 1'b0;
			r_fmb_r30 <= 1'b0;
			r_fma_st_px <= 8'd0;
			r_fma_st_rect <= 8'd0;
			r_fma_st_edge <= 8'd0;
			r_fma_st_pdiv <= 8'd0;
			r_fma_st_pl <= 8'd0;
			r_fma_st_bias <= 8'd0;
			r_xyz_fmv_fm <= 1'b0;
			r_xyz_fmv_fa <= 1'b0;
			r_fma_pl_we <= 1'b0;
			r_ps_2_7 <= 1'b0;
			r_ps_3_6 <= 1'b0;
			r_ps_5_9 <= 1'b0;
			r_ps_4_8_12 <= 1'b0;
			r_ps_0_1 <= 1'b1;
			r_ps_want_fa <= 1'b0;
			r_ps_want_fm <= 1'b0;
			r_ps_want_wrap <= 1'b0;
			r_ps_ne_0100 <= 1'b0;
			r_bias_rm3 <= 1'b0;
			r_bcl_nan <= 1'b0;
			r_bcl_is_zero <= 1'b1;
			xyz_use_fa_r <= 1'b0;
			xyz_use_fa_hi_r <= 1'b0;

			p_step <= 4'd0;
			p_step_rep_a <= 4'd0;

			fa_v <= 1'b0;
			fm_v <= 1'b0;
			fd_v <= 1'b0;
			e_start <= 1'b0;

			sp_valid <= 1'b0;
			more <= 1'b0;
			sub <= 1'b0;
			fa_sub <= 1'b0;
			ea_in <= 8'd0;
			ea23_r <= 8'd0;
			ea_gt_23_r <= 1'b0;
			s_plane_done_st_r <= S_OUT;
		end else begin
			st <= st_next;
			st_oh <= st_oh_next;
			st_rep_a <= st_next;
			st_rep_b <= st_next;
			st_rep_c <= st_next;
			st_rep_d <= st_next;
			st_rep_e <= st_next;
			st_rep_f <= st_next;
			r_st_idle_br <= {5{next_is_idle}};
			s_plane_done_st_r <= s_plane_done_st_w;

			seq_step <= seq_next;
			seq_step_hi <= seq_next;
			seq_step_v <= seq_next;
			seq_step_rep_a <= seq_next;
			seq_step_rep_b <= seq_next;
			seq_step_rep_c <= seq_next;
			seq_step_rep_d <= seq_next;
			seq_oh <= seq_oh_next;
			seq_oh_s0 <= seq_oh_next;
			seq_oh_s1 <= seq_oh_next;
			seq_oh_s2 <= seq_oh_next;
			seq_oh_s3 <= seq_oh_next;
			seq_oh_s4 <= seq_oh_next;
			seq_oh_s5 <= seq_oh_next;
			seq_oh_s6 <= seq_oh_next;
			seq_oh_s7 <= seq_oh_next;
			p_step_oh <= p_step_oh_next;
			p_step_oh_s0 <= p_step_oh_next;
			p_step_oh_s1 <= p_step_oh_next;
			p_step_oh_s2 <= p_step_oh_next;
			p_step_oh_s3 <= p_step_oh_next;
			r_fma_px_hold <= fma_px_hold_nxt;
			r_fma_px_fmr  <= fma_px_fmr_nxt;
			r_fmb_r28 <= fmb_r28_nxt;
			r_fmb_r21 <= fmb_r21_nxt;
			r_fmb_r22 <= fmb_r22_nxt;
			r_fmb_r25 <= fmb_r25_nxt;
			r_fmb_r29 <= fmb_r29_nxt;
			r_fmb_r30 <= fmb_r30_nxt;
			r_fma_st_px   <= {8{st_oh_next[S_PERSP_XYZ] && !fma_px_hold_nxt}};
			r_fma_st_rect <= {8{st_oh_next[S_RECT]}};
			r_fma_st_edge <= {8{st_oh_next[S_EDGE]}};
			r_fma_st_pdiv <= {8{st_oh_next[S_PLANE_DIV]}};
			r_fma_st_pl   <= {8{st_oh_next[S_PLANE]}};
			r_fma_st_bias <= {8{st_oh_next[S_BIAS]}};
			r_xyz_fmv_fm <= xyz_fmv_fm_nxt;
			r_xyz_fmv_fa <= xyz_fmv_fa_nxt;
			if (ps_clr_r) begin
				r_fma_pl_we <= 1'b0;
				r_ps_2_7 <= 1'b0;
				r_ps_3_6 <= 1'b0;
				r_ps_5_9 <= 1'b0;
				r_ps_4_8_12 <= 1'b0;
				r_ps_0_1 <= 1'b1;
				r_ps_want_fa <= 1'b0;
				r_ps_want_fm <= 1'b0;
				r_ps_want_wrap <= 1'b0;
			end else if (ps_adv_r) begin
				r_fma_pl_we <= fma_pl_we_adv;
				r_ps_2_7 <= ps_2_7_adv;
				r_ps_3_6 <= ps_3_6_adv;
				r_ps_5_9 <= ps_5_9_adv;
				r_ps_4_8_12 <= ps_4_8_12_adv;
				r_ps_0_1 <= ps_0_1_adv;
				r_ps_want_fa <= ps_want_fa_adv;
				r_ps_want_fm <= ps_want_fm_adv;
				r_ps_want_wrap <= ps_want_wrap_adv;
			end
			r_ps_ne_0100 <= (point_size != 16'h0100) && (point_size != 16'd0);
			r_bias_rm3 <= (rm[5:4] == 2'd3);
			r_bcl_nan <= (b_clamp[30:23] == 8'hFF) && (b_clamp[22:0] != 23'd0);
			r_bcl_is_zero <= (b_clamp_lo == 16'd0) && (b_clamp_hi[14:0] == 15'd0);
			xyz_use_fa_r <= next_xyz_use_fa;
			xyz_use_fa_hi_r <= next_xyz_use_fa;

			p_step <= p_step_next;
			p_step_rep_a <= p_step_next;

			fa_v <= 1'b0;
			fm_v <= 1'b0;
			fd_v <= 1'b0;
			e_start <= 1'b0;

			case (st_rep_e)
			S_IDLE:
			if (we_idle_fsm) begin
				sub <= 1'b0;
				more <= 1'b0;
				if (in_raw) e_start <= 1'b1;
				else fd_v <= 1'b1;
			end

		S_PERSP_W:
			if (fd_o_w && ((seq_step_v == 5'd0) || (seq_step_v == 5'd1)))
				fd_v <= 1'b1;

		S_PERSP_XYZ: begin
			if (seq_oh[0] || (r_xyz_fmv_fm && fm_o_c_v) || (r_xyz_fmv_fa && fa_o_c_v))
				fm_v <= 1'b1;
			if (r_fma_px_hold && fm_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
		end

		S_RECT:
			case (seq_step_v)
			5'd0: if (fm_o_c_v) fm_v <= 1'b1;
			5'd1: if (fm_o_c_v) begin
				if (!rm[3]) e_start <= 1'b1;
				else begin
					fa_v <= 1'b1;
					fa_sub <= 1'b1;
				end
			end
			5'd2, 5'd4: if (fa_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
			5'd3: if (fa_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b1;
			end
			default: if (fa_o_c_v) e_start <= 1'b1;
			endcase

		S_EDGE:
			if (e_done && !e_empty_buf) begin
				if (kind == 2'd0) begin
					if (ps_ne_0100) fm_v <= 1'b1;
					else begin
						fa_v <= 1'b1;
						fa_sub <= 1'b0;
					end
				end else begin
					fd_v <= 1'b1;
				end
			end

		S_PLANE_DIV:
			if (kind == 2'd0) begin
				case (seq_step_v)
				5'd0, 5'd3: if (fm_o_c_v) begin
					fa_v <= 1'b1;
					fa_sub <= 1'b0;
				end
				5'd1: if (fa_o_c_v) fm_v <= 1'b1;
				5'd2: if (fa_o_c_v) begin
					fa_v <= 1'b1;
					fa_sub <= 1'b0;
				end
				5'd4: if (fa_o_c_v) more <= !sub;
				default: ;
				endcase
			end else if (fd_o_plane_ctrl) begin
			end

		S_PLANE:
			case (p_step_ctrl)
			4'd0: begin
				fa_v <= 1'b1;
				fa_sub <= 1'b1;
			end
			4'd1: if (fa_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b1;
			end
			4'd2, 4'd5, 4'd9: if (fa_o_c_v) fm_v <= 1'b1;
			4'd3, 4'd6, 4'd7, 4'd10, 4'd11: if (fm_o_c_v) fm_v <= 1'b1;
			4'd4, 4'd8: if (fm_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b1;
			end
			4'd12: if (fm_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
			4'd13: if (fa_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
			default: if (fa_o_c_v) begin
				if (!raw && (p_plane == 2'd3) && (kind == 2'd1)) more <= !sub;
			end
			endcase

		S_BIAS:
			case (seq_step_v)
			5'd1: begin
				fm_v <= 1'b1;
				if (rm[5:4] == 2'd3) ea_in <= max_z_exp;
			end
			5'd3: fm_v <= 1'b1;
			5'd2: if (fm_o_c_v) begin
				if (rm[5:4] == 2'd3) begin
					ea23_r <= ea23;
					ea_gt_23_r <= (ea_in > 8'd23);
				end else begin
					fm_v <= 1'b1;
				end
			end
			5'd4: if (fm_o_c_v) begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
			5'd7: begin
				fa_v <= 1'b1;
				fa_sub <= 1'b0;
			end
			default: ;
			endcase

		S_OUT: begin
			sp_valid <= 1'b1;
			if (sp_valid && sp_ready) begin
				sp_valid <= 1'b0;
				if (more) begin
					more <= 1'b0;
					sub <= 1'b1;
					e_start <= 1'b1;
				end
			end
		end
		default: ;
		endcase
		end
	end

	wire pp_rst_cond = st_oh[S_PLANE_DIV] && (kind != 2'd0) && fd_o_plane_ctrl;
	wire pp_adv_cond = st_oh[S_PLANE] && r_ps_want_wrap && fa_o_c_v && !raw && (p_plane != 2'd3);
	wire pp_rst_p, pp_adv_p;
	wire [5:0] pp_rst_k, pp_adv_k;
	`RX_BUF1(u_pprp, pp_rst_cond, pp_rst_p);
	`RX_BUF1(u_ppap, pp_adv_cond, pp_adv_p);
	genvar gpp;
	generate
		for (gpp = 0; gpp < 6; gpp = gpp + 1) begin : g_pp_buf
			`RX_BUF1(u_pprk, pp_rst_cond, pp_rst_k[gpp]);
			`RX_BUF1(u_ppak, pp_adv_cond, pp_adv_k[gpp]);
		end
	endgenerate

	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) p_plane <= 2'd0;
		else if (pp_rst_p) p_plane <= 2'd0;
		else if (pp_adv_p) p_plane <= p_plane_inc;
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a0 <= 4'b0001; r_p_plane_oh_b0 <= 4'b0001; end
		else if (pp_rst_k[0]) begin r_p_plane_oh_a0 <= 4'b0001; r_p_plane_oh_b0 <= 4'b0001; end
		else if (pp_adv_k[0]) begin r_p_plane_oh_a0 <= {r_p_plane_oh_a0[2:0], 1'b0}; r_p_plane_oh_b0 <= {r_p_plane_oh_b0[2:0], 1'b0}; end
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a1 <= 4'b0001; r_p_plane_oh_b1 <= 4'b0001; end
		else if (pp_rst_k[1]) begin r_p_plane_oh_a1 <= 4'b0001; r_p_plane_oh_b1 <= 4'b0001; end
		else if (pp_adv_k[1]) begin r_p_plane_oh_a1 <= {r_p_plane_oh_a1[2:0], 1'b0}; r_p_plane_oh_b1 <= {r_p_plane_oh_b1[2:0], 1'b0}; end
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a2 <= 4'b0001; r_p_plane_oh_b2 <= 4'b0001; end
		else if (pp_rst_k[2]) begin r_p_plane_oh_a2 <= 4'b0001; r_p_plane_oh_b2 <= 4'b0001; end
		else if (pp_adv_k[2]) begin r_p_plane_oh_a2 <= {r_p_plane_oh_a2[2:0], 1'b0}; r_p_plane_oh_b2 <= {r_p_plane_oh_b2[2:0], 1'b0}; end
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a3 <= 4'b0001; r_p_plane_oh_b3 <= 4'b0001; end
		else if (pp_rst_k[3]) begin r_p_plane_oh_a3 <= 4'b0001; r_p_plane_oh_b3 <= 4'b0001; end
		else if (pp_adv_k[3]) begin r_p_plane_oh_a3 <= {r_p_plane_oh_a3[2:0], 1'b0}; r_p_plane_oh_b3 <= {r_p_plane_oh_b3[2:0], 1'b0}; end
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a4 <= 4'b0001; r_p_plane_oh_b4 <= 4'b0001; end
		else if (pp_rst_k[4]) begin r_p_plane_oh_a4 <= 4'b0001; r_p_plane_oh_b4 <= 4'b0001; end
		else if (pp_adv_k[4]) begin r_p_plane_oh_a4 <= {r_p_plane_oh_a4[2:0], 1'b0}; r_p_plane_oh_b4 <= {r_p_plane_oh_b4[2:0], 1'b0}; end
	end
	always @(posedge clk or posedge rst_setup_pp) begin
		if (rst_setup_pp) begin r_p_plane_oh_a5 <= 4'b0001; r_p_plane_oh_b5 <= 4'b0001; end
		else if (pp_rst_k[5]) begin r_p_plane_oh_a5 <= 4'b0001; r_p_plane_oh_b5 <= 4'b0001; end
		else if (pp_adv_k[5]) begin r_p_plane_oh_a5 <= {r_p_plane_oh_a5[2:0], 1'b0}; r_p_plane_oh_b5 <= {r_p_plane_oh_b5[2:0], 1'b0}; end
	end

	genvar g;
	generate
	for (g = 0; g < 12; g = g + 1) begin : g_plane
		assign sp_tri[352 + 32 * g +: 32] = pl[g];
	end
	endgenerate
	assign sp_tri[351:0] = {e_bbox, e_edges};
	assign sp_tri[767:736] = {1'b0, kind_out, raw, e_front, more ? 3'd0 : keep[26:24], keep[23:0]};

	wire unused_ok = &{1'b0, fd_busy, rm[2:1], su_bo_unused, zexp_d_unused,
		lw_fix[23:16], ps_fix[23:16], div_busy, mesh_id, in_valid_ctrl, in_valid_fd,
		seq_inc_co_unused, seq_inc_hi_co_unused, p_step_ctrl_inc_co_unused, p_plane_inc_co_unused,
		unused_bcl_flags, seq_fma_s[0], seq_fma_s[1], seq_fma_s[2], seq_fma_s[3],
		seq_fma_s[4], seq_fma_s[5], seq_fma_s[6], seq_fma_s[7],
		seq_fmb_s[0], seq_fmb_s[1], seq_fmb_s[2], seq_fmb_s[3],
		seq_fmb_s[4], seq_fmb_s[5], seq_fmb_s[6], seq_fmb_s[7],
		seq_faa_s[0], seq_faa_s[1], seq_faa_s[2], seq_faa_s[3],
		seq_faa_s[4], seq_faa_s[5], seq_faa_s[6], seq_faa_s[7],
		seq_fab_s[0], seq_fab_s[1], seq_fab_s[2], seq_fab_s[3],
		seq_fab_s[4], seq_fab_s[5], seq_fab_s[6], seq_fab_s[7],
		p_step_a_s[0], p_step_a_s[1], p_step_a_s[2], p_step_a_s[3],
		p_step_a_s[4], p_step_a_s[5], p_step_a_s[6], p_step_a_s[7],
		p_step_b_s[0], p_step_b_s[1], p_step_b_s[2], p_step_b_s[3],
		p_step_b_s[4], p_step_b_s[5], p_step_b_s[6], p_step_b_s[7],
		p_plane_pf0_l, p_plane_pf0_h, p_plane_pf1_l, p_plane_pf1_h,
		p_plane_pf2_l, p_plane_pf2_h, p_plane_2_l, p_plane_2_h, p_plane_3_l, p_plane_3_h,
		seq_T_lo, seq_T_hi, p_step_T_lo, p_step_T_hi};

`undef RX_BUF1
endmodule

`default_nettype wire
