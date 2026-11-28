/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_tcrd: a lane's texel coordinates with 8 fraction bits for
 * atg_tu_addr's stage A: (the normalized coordinate's atg_tu_coord) ×
 * size >> 16, a cube's face coordinate × the face size >> 16, or an
 * unnormalized coordinate's floor(x × 256) (atg_tu_unnorm).
 *
 * Latency:
 * - LAT = 0: combinational
 * - LAT = 1: 1 cycle pipeline register in input + 1 cycle in multiplier (total 2 cycles)
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_tcrd #(
	parameter LAT = 1,
	parameter ENABLE_3D_CUBE = 1
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] cw0,	/* the lane's u, v and w (r) words */
	input wire [31:0] cw1,
	input wire [31:0] cw4,
	input wire [31:0] cw0_un,
	input wire [31:0] cw1_un,
	input wire [1:0] mu,
	input wire [1:0] mv,
	input wire [1:0] mw,
	input wire [15:0] wl,
	input wire [15:0] hl,
	input wire [15:0] dl,
	input wire signed [26:0] fcs,	/* a cube's face coordinates */
	input wire signed [26:0] fct,
	input wire cube,
	input wire unn,
	output wire signed [30:0] cu,
	output wire signed [30:0] cv,
	output wire signed [30:0] cw
);
	wire rst_inv = !rst_n;

	wire signed [29:0] nu, nv, nw, un0, un1;
	wire signed [46:0] pu, pv, pw;
	wire signed [43:0] qu, qv;
	atg_tu_coord #(.LAT(LAT)) u_nu (.clk(clk), .rst_n(rst_n), .en(en), .f(cw0), .mode(mu), .c(nu));
	atg_tu_coord #(.LAT(LAT)) u_nv (.clk(clk), .rst_n(rst_n), .en(en), .f(cw1), .mode(mv), .c(nv));
	generate
	if (ENABLE_3D_CUBE) begin : g_nw
		atg_tu_coord #(.LAT(LAT)) u_nw (.clk(clk), .rst_n(rst_n), .en(en), .f(cw4), .mode(mw), .c(nw));
	end else begin : g_no_nw
		assign nw = 30'sd0;
	end
	endgenerate
	atg_tu_unnorm #(.LAT(LAT)) u_un0 (.clk(clk), .rst_n(rst_n), .en(en), .f(cw0_un), .c(un0));
	atg_tu_unnorm #(.LAT(LAT)) u_un1 (.clk(clk), .rst_n(rst_n), .en(en), .f(cw1_un), .c(un1));

	/* LAT >= 1 pipeline registers */
	reg signed [29:0] r_nu, r_nv, r_nw;
	(* keep = "true", equivalent_register_removal = "no", srl_style = "register", extract_reset = "no" *) reg signed [29:0] r_un0, r_un1;
	(* keep = "true", equivalent_register_removal = "no", srl_style = "register", extract_reset = "no" *) reg signed [29:0] r_un0_d, r_un1_d;
	(* keep = "true", equivalent_register_removal = "no", srl_style = "register", extract_reset = "no" *) reg signed [29:0] r_un0_d2, r_un1_d2;
	reg signed [26:0] r_fcs, r_fct;
	reg r_cube, r_unn;
	reg r_cube_d, r_unn_d;
	reg r_cube_d2, r_unn_d2;

	always @(posedge clk) begin
		if (en) begin
			r_un0    <= un0;
			r_un1    <= un1;
			r_un0_d  <= r_un0;
			r_un1_d  <= r_un1;
			r_un0_d2 <= r_un0_d;
			r_un1_d2 <= r_un1_d;
		end
	end

	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r_nu      <= 30'sd0;
			r_nv      <= 30'sd0;
			r_nw      <= 30'sd0;
			r_fcs     <= 27'sd0;
			r_fct     <= 27'sd0;
			r_cube    <= 1'b0;
			r_unn     <= 1'b0;
			r_cube_d  <= 1'b0;
			r_unn_d   <= 1'b0;
			r_cube_d2 <= 1'b0;
			r_unn_d2  <= 1'b0;
		end else if (en) begin
			r_nu      <= nu;
			r_nv      <= nv;
			r_nw      <= nw;
			r_fcs     <= fcs;
			r_fct     <= fct;
			r_cube    <= cube;
			r_unn     <= unn;
			r_cube_d  <= r_cube;
			r_unn_d   <= r_unn;
			r_cube_d2 <= r_cube_d;
			r_unn_d2  <= r_unn_d;
		end
	end

	wire signed [29:0] in_nu  = (LAT >= 1) ? r_nu : nu;
	wire signed [29:0] in_nv  = (LAT >= 1) ? r_nv : nv;
	wire signed [29:0] in_nw  = (LAT >= 1) ? r_nw : nw;
	wire signed [26:0] in_fcs = (LAT >= 1) ? r_fcs : fcs;
	wire signed [26:0] in_fct = (LAT >= 1) ? r_fct : fct;
	wire signed [29:0] un0_val = (LAT >= 1) ? r_un0_d2 : un0;
	wire signed [29:0] un1_val = (LAT >= 1) ? r_un1_d2 : un1;
	wire final_cube            = (LAT >= 1) ? r_cube_d2 : cube;
	wire final_unn             = (LAT >= 1) ? r_unn_d2  : unn;

	wire mclk = (LAT >= 1) ? clk : 1'b0;
	localparam MLAT = (LAT >= 2) ? 3 : (LAT >= 1) ? 2 : 0;

	wire signed [30:0] pu_w, pv_w, pw_w;
`ifdef SYNTH_ASIC
	wire signed [38:0] pu_lo, pu_hi, pv_lo, pv_hi;
	wire [38:0] pu_sum, pv_sum;
	wire pu_sum_co_unused, pv_sum_co_unused;
	am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pu_lo (.clk(mclk), .en(en), .a(in_nu), .b({1'b0, wl[7:0]}),  .p(pu_lo));
	am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pu_hi (.clk(mclk), .en(en), .a(in_nu), .b({1'b0, wl[15:8]}), .p(pu_hi));
	am_add #(.W(39)) u_pu_sum (.a({{8{pu_lo[38]}}, pu_lo[38:8]}), .b(pu_hi), .ci(1'b0), .s(pu_sum), .co(pu_sum_co_unused));
	am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pv_lo (.clk(mclk), .en(en), .a(in_nv), .b({1'b0, hl[7:0]}),  .p(pv_lo));
	am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pv_hi (.clk(mclk), .en(en), .a(in_nv), .b({1'b0, hl[15:8]}), .p(pv_hi));
	am_add #(.W(39)) u_pv_sum (.a({{8{pv_lo[38]}}, pv_lo[38:8]}), .b(pv_hi), .ci(1'b0), .s(pv_sum), .co(pv_sum_co_unused));
	assign pu_w = pu_sum[38:8];
	assign pv_w = pv_sum[38:8];
	assign pu   = {pu_w, pu_sum[7:0], pu_lo[7:0]};
	assign pv   = {pv_w, pv_sum[7:0], pv_lo[7:0]};
`else
	am_mul #(.AW(30), .BW(17), .SIGNED(1), .LAT(MLAT)) u_pu (.clk(mclk), .en(en), .a(in_nu), .b({1'b0, wl}),
		.p(pu));
	am_mul #(.AW(30), .BW(17), .SIGNED(1), .LAT(MLAT)) u_pv (.clk(mclk), .en(en), .a(in_nv), .b({1'b0, hl}),
		.p(pv));
	assign pu_w = pu[46:16];
	assign pv_w = pv[46:16];
`endif
	generate
	if (ENABLE_3D_CUBE) begin : g_mul_3d_cube
`ifdef SYNTH_ASIC
		wire signed [38:0] pw_lo, pw_hi;
		wire [38:0] pw_sum;
		wire pw_sum_co_unused;
		am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pw_lo (.clk(mclk), .en(en), .a(in_nw), .b({1'b0, dl[7:0]}),  .p(pw_lo));
		am_mul #(.AW(30), .BW(9), .SIGNED(1), .LAT(MLAT)) u_pw_hi (.clk(mclk), .en(en), .a(in_nw), .b({1'b0, dl[15:8]}), .p(pw_hi));
		am_add #(.W(39)) u_pw_sum (.a({{8{pw_lo[38]}}, pw_lo[38:8]}), .b(pw_hi), .ci(1'b0), .s(pw_sum), .co(pw_sum_co_unused));
		assign pw_w = pw_sum[38:8];
		assign pw   = {pw_w, pw_sum[7:0], pw_lo[7:0]};
`else
		am_mul #(.AW(30), .BW(17), .SIGNED(1), .LAT(MLAT)) u_pw (.clk(mclk), .en(en), .a(in_nw), .b({1'b0, dl}),
			.p(pw));
		assign pw_w = pw[46:16];
`endif
		am_mul #(.AW(27), .BW(17), .SIGNED(1), .LAT(MLAT)) u_qu (.clk(mclk), .en(en), .a(in_fcs), .b({1'b0, wl}),
			.p(qu));
		am_mul #(.AW(27), .BW(17), .SIGNED(1), .LAT(MLAT)) u_qv (.clk(mclk), .en(en), .a(in_fct), .b({1'b0, wl}),
			.p(qv));
	end else begin : g_no_mul_3d_cube
		assign pw = 47'sd0;
		assign pw_w = 31'sd0;
		assign qu = 44'sd0;
		assign qv = 44'sd0;
	end
	endgenerate

	/* Post-DSP output registers for control and split-mul alignment (when LAT >= 2) */
	(* keep = "true", equivalent_register_removal = "no", srl_style = "register", extract_reset = "no" *) reg signed [29:0] r_un0_d3, r_un1_d3, r_un0_d4, r_un1_d4;
	(* extract_reset = "no" *) reg signed [30:0] r_s2_pu, r_s2_pv, r_s2_pw;
	(* extract_reset = "no" *) reg signed [27:0] r_s2_qu, r_s2_qv;
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] r_cube_u_b, r_cube_v_b, r_cube_u_b2, r_cube_v_b2;
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] r_unn_u_b, r_unn_v_b, r_unn_u_b2, r_unn_v_b2;
	always @(posedge clk) begin
		if (en) begin
			r_un0_d3 <= r_un0_d2;
			r_un1_d3 <= r_un1_d2;
			r_un0_d4 <= r_un0_d3;
			r_un1_d4 <= r_un1_d3;
			r_s2_pu  <= pu_w;
			r_s2_pv  <= pv_w;
			r_s2_pw  <= pw_w;
			r_s2_qu  <= qu[43:16];
			r_s2_qv  <= qv[43:16];
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r_cube_u_b  <= 3'd0;
			r_cube_v_b  <= 3'd0;
			r_unn_u_b   <= 3'd0;
			r_unn_v_b   <= 3'd0;
			r_cube_u_b2 <= 3'd0;
			r_cube_v_b2 <= 3'd0;
			r_unn_u_b2  <= 3'd0;
			r_unn_v_b2  <= 3'd0;
		end else if (en) begin
			r_cube_u_b  <= {3{r_cube_d2}};
			r_cube_v_b  <= {3{r_cube_d2}};
			r_unn_u_b   <= {3{r_unn_d2}};
			r_unn_v_b   <= {3{r_unn_d2}};
			r_cube_u_b2 <= r_cube_u_b;
			r_cube_v_b2 <= r_cube_v_b;
			r_unn_u_b2  <= r_unn_u_b;
			r_unn_v_b2  <= r_unn_v_b;
		end
	end

	wire signed [30:0] s2_pu    = (LAT >= 2) ? r_s2_pu : pu_w;
	wire signed [30:0] s2_pv    = (LAT >= 2) ? r_s2_pv : pv_w;
	wire signed [30:0] s2_pw    = (LAT >= 2) ? r_s2_pw : pw_w;
	wire signed [27:0] s2_qu    = (LAT >= 2) ? r_s2_qu : qu[43:16];
	wire signed [27:0] s2_qv    = (LAT >= 2) ? r_s2_qv : qv[43:16];
	wire signed [29:0] s2_un0   = (LAT >= 2) ? r_un0_d4 : un0_val;
	wire signed [29:0] s2_un1   = (LAT >= 2) ? r_un1_d4 : un1_val;
	wire [2:0] s2_cube_u_b = (LAT >= 2) ? r_cube_u_b2 : {3{final_cube}};
	wire [2:0] s2_cube_v_b = (LAT >= 2) ? r_cube_v_b2 : {3{final_cube}};
	wire [2:0] s2_unn_u_b  = (LAT >= 2) ? r_unn_u_b2  : {3{final_unn}};
	wire [2:0] s2_unn_v_b  = (LAT >= 2) ? r_unn_v_b2  : {3{final_unn}};

	wire [30:0] qu_ext  = {{3{s2_qu[27]}}, s2_qu};
	wire [30:0] qv_ext  = {{3{s2_qv[27]}}, s2_qv};
	wire [30:0] un0_ext = {s2_un0[29], s2_un0};
	wire [30:0] un1_ext = {s2_un1[29], s2_un1};

	assign cu[9:0]   = s2_cube_u_b[0] ? qu_ext[9:0]   : s2_unn_u_b[0] ? un0_ext[9:0]   : s2_pu[9:0];
	assign cu[19:10] = s2_cube_u_b[1] ? qu_ext[19:10] : s2_unn_u_b[1] ? un0_ext[19:10] : s2_pu[19:10];
	assign cu[30:20] = s2_cube_u_b[2] ? qu_ext[30:20] : s2_unn_u_b[2] ? un0_ext[30:20] : s2_pu[30:20];
	assign cv[9:0]   = s2_cube_v_b[0] ? qv_ext[9:0]   : s2_unn_v_b[0] ? un1_ext[9:0]   : s2_pv[9:0];
	assign cv[19:10] = s2_cube_v_b[1] ? qv_ext[19:10] : s2_unn_v_b[1] ? un1_ext[19:10] : s2_pv[19:10];
	assign cv[30:20] = s2_cube_v_b[2] ? qv_ext[30:20] : s2_unn_v_b[2] ? un1_ext[30:20] : s2_pv[30:20];
	assign cw = s2_pw;

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, rst_n, en} : 1'b0;
	wire unused_ok = &{1'b0, pu[15:0], pv[15:0], pw[15:0], qu[15:0], qv[15:0], unused_lat0};
endmodule

`default_nettype wire
