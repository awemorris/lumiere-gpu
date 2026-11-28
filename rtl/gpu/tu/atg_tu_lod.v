/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_lod: the quad's level of detail of texb (part of atg_tu_addr,
 * the reference model tumodel lod_quad is the reference, bit for bit).
 *
 * Latency:
 * - LAT = 0: combinational
 * - LAT = 1: 1 cycle pipeline register between magnitude calculation and log2/sum
 * - LAT = 2: 1 cycle in input fixed-point + 1 cycle in magnitude
 * - LAT = 3: 1 cycle in input + 1 cycle in magnitude + 1 cycle in log2/sum output
 * - LAT = 4: 1 cycle in input u_fx + 1 cycle in fa + 1 cycle in magnitude + 1 cycle in log2/sum
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_lod #(
	parameter LAT = 1
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [95:0] u,	/* lanes 0-2, lane 0 in the low bits */
	input wire [95:0] v,
	input wire [95:0] w,	/* the third coordinates (3D) */
	input wire [80:0] cs,	/* cube: s of lanes 0-2 on lane 0's face (27 bits each) */
	input wire [80:0] ct,
	input wire is3d,
	input wire cube,
	input wire [15:0] w0,	/* level 0: width, height, depth (3D) */
	input wire [15:0] h0,
	input wire [15:0] d0,
	input wire [1:0] ax,	/* the axis: 0 u, 1 v, 2 w */
	output wire signed [16:0] lam
);
	`include "atg_tu_fn.vh"
	localparam signed [16:0] LAM_MIN = -17'sd32768;

	wire rst_inv = !rst_n;

	/* Delay cs, ct, w0, h0, d0 for LAT >= 4 to match u_fx pipeline (reset-free datapath) */
	(* extract_reset = "no" *) reg [80:0] r_cs, r_ct, r2_cs, r2_ct;
	(* extract_reset = "no" *) reg [15:0] r_w0_in, r_h0_in, r_d0_in, r2_w0_in, r2_h0_in, r2_d0_in;
	always @(posedge clk) begin
		if (en) begin
			r_cs     <= cs;
			r_ct     <= ct;
			r_w0_in  <= w0;
			r_h0_in  <= h0;
			r_d0_in  <= d0;
			r2_cs    <= r_cs;
			r2_ct    <= r_ct;
			r2_w0_in <= r_w0_in;
			r2_h0_in <= r_h0_in;
			r2_d0_in <= r_d0_in;
		end
	end

	wire [80:0] cs_in = (LAT >= 5) ? r2_cs    : (LAT >= 4) ? r_cs    : cs;
	wire [80:0] ct_in = (LAT >= 5) ? r2_ct    : (LAT >= 4) ? r_ct    : ct;
	wire [15:0] w0_in = (LAT >= 5) ? r2_w0_in : (LAT >= 4) ? r_w0_in : w0;
	wire [15:0] h0_in = (LAT >= 5) ? r2_h0_in : (LAT >= 4) ? r_h0_in : h0;
	wire [15:0] d0_in = (LAT >= 5) ? r2_d0_in : (LAT >= 4) ? r_d0_in : d0;

	/* The axis' fixed-point lane values: floor(x × 2^16), or the cube's
	 * >> 8, and its size. */
	wire [95:0] fa_comb;
	genvar gi;
	generate
	for (gi = 0; gi < 3; gi = gi + 1) begin : g_fa
		wire [31:0] fx;
		wire [3:0] ax1_in_s  = {4{ax[1]}};
		wire [3:0] ax0_in_s  = {4{ax[0]}};
		wire [3:0] cube_fa_s = {4{cube && !ax[1]}};
		wire [3:0] ax0_fa_s  = {4{ax[0]}};
		wire [31:0] f_in = {
			ax1_in_s[3] ? w[gi*32 + 24 +: 8] : ax0_in_s[3] ? v[gi*32 + 24 +: 8] : u[gi*32 + 24 +: 8],
			ax1_in_s[2] ? w[gi*32 + 16 +: 8] : ax0_in_s[2] ? v[gi*32 + 16 +: 8] : u[gi*32 + 16 +: 8],
			ax1_in_s[1] ? w[gi*32 + 8  +: 8] : ax0_in_s[1] ? v[gi*32 + 8  +: 8] : u[gi*32 + 8  +: 8],
			ax1_in_s[0] ? w[gi*32 + 0  +: 8] : ax0_in_s[0] ? v[gi*32 + 0  +: 8] : u[gi*32 + 0  +: 8]
		};
		wire rst_n_fa;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *)
		LUT1 #(.INIT(2'b10)) u_rst_fa (.I0(rst_n), .O(rst_n_fa));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign rst_n_fa = rst_n;
`else
		assign rst_n_fa = rst_n;
`endif
`ifdef SYNTH_KINTEX7
		(* keep_hierarchy = "yes" *)
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep_hierarchy = "yes" *)
`endif
		atg_tu_fx32 #(.LAT(LAT >= 5 ? 2 : LAT >= 4 ? 1 : 0), .SH_CONST(16)) u_fx (.clk(clk), .rst_n(rst_n_fa), .en(en),
			.f(f_in), .sh(5'd16),
			.q(fx));
		wire [31:0] ct_ext = {{13{ct_in[gi*27 + 26]}}, ct_in[gi*27 + 8 +: 19]};
		wire [31:0] cs_ext = {{13{cs_in[gi*27 + 26]}}, cs_in[gi*27 + 8 +: 19]};
		assign fa_comb[gi*32 + 0  +: 8] = cube_fa_s[0] ? (ax0_fa_s[0] ? ct_ext[7:0]   : cs_ext[7:0])   : fx[7:0];
		assign fa_comb[gi*32 + 8  +: 8] = cube_fa_s[1] ? (ax0_fa_s[1] ? ct_ext[15:8]  : cs_ext[15:8])  : fx[15:8];
		assign fa_comb[gi*32 + 16 +: 8] = cube_fa_s[2] ? (ax0_fa_s[2] ? ct_ext[23:16] : cs_ext[23:16]) : fx[23:16];
		assign fa_comb[gi*32 + 24 +: 8] = cube_fa_s[3] ? (ax0_fa_s[3] ? ct_ext[31:24] : cs_ext[31:24]) : fx[31:24];
	end
	endgenerate
	wire [15:0] n_comb = ax[1] ? d0_in : ax[0] && !cube ? h0_in : w0_in;
	wire [3:0] n_nib_nz;
	wire [1:0] n_nib_hi [0:3];
	genvar gnn;
	generate
	for (gnn = 0; gnn < 4; gnn = gnn + 1) begin : g_n_nib
		wire [3:0] nb = n_comb[gnn*4 +: 4];
		assign n_nib_nz[gnn] = |nb;
		assign n_nib_hi[gnn] = nb[3] ? 2'd3 : nb[2] ? 2'd2 : nb[1] ? 2'd1 : 2'd0;
	end
	endgenerate
	wire [3:0] p_n_comb = n_nib_nz[3] ? {2'd3, n_nib_hi[3]} :
	                      n_nib_nz[2] ? {2'd2, n_nib_hi[2]} :
	                      n_nib_nz[1] ? {2'd1, n_nib_hi[1]} :
	                      n_nib_nz[0] ? {2'd0, n_nib_hi[0]} : 4'd0;

	/* --- Stage 0A -> Stage 0B Pipeline Registers (LAT >= 2) --- */
	(* extract_reset = "no" *) reg [95:0] r_fa;
	(* extract_reset = "no" *) reg [15:0] r_n0;
	(* extract_reset = "no" *) reg [3:0]  r_p_n;
	reg        r_is3d0;

	always @(posedge clk) begin
		if (en) begin
			r_fa  <= fa_comb;
			r_n0  <= n_comb;
			r_p_n <= p_n_comb;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv)
			r_is3d0 <= 1'b0;
		else if (en)
			r_is3d0 <= is3d;
	end

	wire [95:0] fa       = (LAT >= 2) ? r_fa    : fa_comb;
	wire [15:0] n        = (LAT >= 2) ? r_n0    : n_comb;
	wire [3:0]  p_n      = (LAT >= 2) ? r_p_n   : p_n_comb;
	wire        is3d_s0b = (LAT >= 2) ? r_is3d0 : is3d;

	/* The axis: the lanes' differences to lane 0, the larger magnitude mx */
	wire [32:0] d1, d2, m1, m2;
	wire [6:0] lod_carry_unused;
	am_sub #(.W(33)) u_d1 (.a({fa[63], fa[63:32]}), .b({fa[31], fa[31:0]}), .bi(1'b0), .d(d1),
		.bo(lod_carry_unused[0]));
	am_sub #(.W(33)) u_d2 (.a({fa[95], fa[95:64]}), .b({fa[31], fa[31:0]}), .bi(1'b0), .d(d2),
		.bo(lod_carry_unused[1]));
	wire [23:0] n_ext = {n, 8'd0};
	wire [7:0] m_n_g [0:3];
	genvar gmn;
	generate
	for (gmn = 0; gmn < 4; gmn = gmn + 1) begin : g_mn
		wire [11:0] nq = n_ext[gmn*4 +: 12];
		assign m_n_g[gmn] = (p_n[1:0] == 2'd3) ? nq[10:3] :
		                    (p_n[1:0] == 2'd2) ? nq[9:2]  :
		                    (p_n[1:0] == 2'd1) ? nq[8:1]  : nq[7:0];
	end
	endgenerate
	wire [7:0] m_n_comb = p_n[3] ? (p_n[2] ? m_n_g[3] : m_n_g[2]) :
	                               (p_n[2] ? m_n_g[1] : m_n_g[0]);
	wire [13:0] nl2_comb = {2'b00, p_n, m_n_comb};

	/* Stage 0B1 -> Stage 0B2 Pipeline Registers (when LAT >= 5) */
	(* extract_reset = "no" *) reg [32:0] r_d1, r_d2;
	(* extract_reset = "no" *) reg [13:0] r_nl2;
	reg        r_is3d_b1;
	always @(posedge clk) begin
		if (en) begin
			r_d1  <= d1;
			r_d2  <= d2;
			r_nl2 <= nl2_comb;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv)
			r_is3d_b1 <= 1'b0;
		else if (en)
			r_is3d_b1 <= is3d_s0b;
	end

	wire [32:0] s_d1    = (LAT >= 5) ? r_d1      : d1;
	wire [32:0] s_d2    = (LAT >= 5) ? r_d2      : d2;
	wire [13:0] s_nl2   = (LAT >= 5) ? r_nl2     : nl2_comb;
	wire        is3d_b1 = (LAT >= 5) ? r_is3d_b1 : is3d_s0b;

	/* Absolute value |s_d1| and |s_d2| via fast decoupled 33b +1 nibble-prefix (<= 9 cmos3) */
	wire [32:0] m_in [0:1];
	wire [32:0] m_out [0:1];
	assign m_in[0] = s_d1;
	assign m_in[1] = s_d2;
	genvar gm, gn;
	generate
	for (gm = 0; gm < 2; gm = gm + 1) begin : g_abs
		wire [32:0] md = m_in[gm];
		(* keep = "true" *) wire m_s_lo = md[32];
		(* keep = "true" *) wire m_s_hi = md[32];
		wire [32:0] m_neg_in = ~md;
		wire [7:0]  m_z4;
		for (gn = 0; gn < 8; gn = gn + 1) begin : g_mz
			assign m_z4[gn] = (md[gn*4 +: 4] == 4'd0);
		end
		wire m_p8_0  = m_z4[1] && m_z4[0];
		wire m_p8_1  = m_z4[3] && m_z4[2];
		wire m_p8_2  = m_z4[5] && m_z4[4];
		wire m_p16_0 = m_p8_1 && m_p8_0;
		wire [8:0] m_cin;
		assign m_cin[0] = 1'b1;
		assign m_cin[1] = m_z4[0];
		assign m_cin[2] = m_p8_0;
		assign m_cin[3] = m_p8_0 && m_z4[2];
		assign m_cin[4] = m_p16_0;
		assign m_cin[5] = m_p16_0 && m_z4[4];
		assign m_cin[6] = m_p16_0 && m_p8_2;
		assign m_cin[7] = m_p16_0 && m_p8_2 && m_z4[6];
		assign m_cin[8] = m_p16_0 && m_p8_2 && m_z4[7] && m_z4[6];
		wire [32:0] m_neg;
		for (gn = 0; gn < 8; gn = gn + 1) begin : g_ms
			wire [3:0] mn = m_neg_in[gn*4 +: 4];
			wire mc = m_cin[gn];
			assign m_neg[gn*4 + 0] = mn[0] ^ mc;
			assign m_neg[gn*4 + 1] = mn[1] ^ (mc && mn[0]);
			assign m_neg[gn*4 + 2] = mn[2] ^ (mc && mn[1] && mn[0]);
			assign m_neg[gn*4 + 3] = mn[3] ^ (mc && mn[2] && mn[1] && mn[0]);
		end
		assign m_neg[32] = m_neg_in[32] ^ m_cin[8];
		assign m_out[gm] = {m_s_hi ? m_neg[32:16] : md[32:16], m_s_lo ? m_neg[15:0] : md[15:0]};
	end
	endgenerate
	assign m1 = m_out[0];
	assign m2 = m_out[1];
	assign lod_carry_unused[3:2] = 2'b00;
	wire [4:0] n_bias_hi;
	am_sub #(.W(5)) u_nbias (.a({3'd0, s_nl2[13:12]}), .b(5'd1), .bi(1'b0), .d(n_bias_hi),
		.bo(lod_carry_unused[4]));
	wire [16:0] n_bias_comb = {n_bias_hi, s_nl2[11:0]};

	/* Stage 0B2 -> Stage 0B3 Pipeline Registers (when LAT >= 5) */
	(* extract_reset = "no" *) reg [32:0] r_m1, r_m2;
	(* srl_style = "register", extract_reset = "no" *) reg [16:0] r_nbias_b2;
	reg        r_is3d_b2;
	always @(posedge clk) begin
		if (en) begin
			r_m1       <= m1;
			r_m2       <= m2;
			r_nbias_b2 <= n_bias_comb;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv)
			r_is3d_b2 <= 1'b0;
		else if (en)
			r_is3d_b2 <= is3d_b1;
	end

	wire [32:0] s_m1       = (LAT >= 5) ? r_m1       : m1;
	wire [32:0] s_m2       = (LAT >= 5) ? r_m2       : m2;
	wire [16:0] s_nbias_b2 = (LAT >= 5) ? r_nbias_b2 : n_bias_comb;
	wire        is3d_b2    = (LAT >= 5) ? r_is3d_b2  : is3d_b1;

	wire [32:0] m_cmp_d_unused;
	wire        m_cmp_bo;
	am_sub #(.W(33)) u_mcmp (.a(s_m1), .b(s_m2), .bi(1'b0), .d(m_cmp_d_unused), .bo(m_cmp_bo));

	/* Stage 0B3 -> Stage 0B4 Pipeline Registers (when LAT >= 5) */
	(* extract_reset = "no" *) reg [32:0] r_m1_b3, r_m2_b3;
	(* keep = "true", equivalent_register_removal = "no" *) reg        r_m1_ge_m2_lo, r_m1_ge_m2_hi0, r_m1_ge_m2_hi1;
	(* srl_style = "register", extract_reset = "no" *) reg [16:0] r_nbias_b3;
	reg        r_is3d_b3;
	always @(posedge clk) begin
		if (en) begin
			r_m1_b3    <= s_m1;
			r_m2_b3    <= s_m2;
			r_nbias_b3 <= s_nbias_b2;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r_m1_ge_m2_lo  <= 1'b0;
			r_m1_ge_m2_hi0 <= 1'b0;
			r_m1_ge_m2_hi1 <= 1'b0;
			r_is3d_b3      <= 1'b0;
		end else if (en) begin
			r_m1_ge_m2_lo  <= !m_cmp_bo;
			r_m1_ge_m2_hi0 <= !m_cmp_bo;
			r_m1_ge_m2_hi1 <= !m_cmp_bo;
			r_is3d_b3      <= is3d_b2;
		end
	end

	wire [32:0] s0_mx = (LAT >= 5) ? {
		r_m1_ge_m2_hi1 ? r_m1_b3[32:24] : r_m2_b3[32:24],
		r_m1_ge_m2_hi0 ? r_m1_b3[23:16] : r_m2_b3[23:16],
		r_m1_ge_m2_lo  ? r_m1_b3[15:0]  : r_m2_b3[15:0]
	} : (!m_cmp_bo ? s_m1 : s_m2);
	wire [16:0] s0_nbias    = (LAT >= 5) ? r_nbias_b3 : s_nbias_b2;
	wire        s0_is3d_out = (LAT >= 5) ? r_is3d_b3  : is3d_b2;

	/* --- Stage 0B -> Stage 1 Pipeline Registers (LAT >= 1) --- */
	(* extract_reset = "no" *) reg [32:0] r0_mx;
	(* srl_style = "register", extract_reset = "no" *) reg [16:0] r0_nbias;
	reg        r0_is3d;

	always @(posedge clk) begin
		if (en) begin
			r0_mx    <= s0_mx;
			r0_nbias <= s0_nbias;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv)
			r0_is3d <= 1'b0;
		else if (en)
			r0_is3d <= s0_is3d_out;
	end

	wire [32:0] s1_mx    = (LAT >= 1) ? r0_mx    : s0_mx;
	wire [16:0] s1_nbias = (LAT >= 1) ? r0_nbias : s0_nbias;
	wire        s1_is3d  = (LAT >= 1) ? r0_is3d  : s0_is3d_out;

	/* Stage 1A: leading position p of s1_mx (nibble-based priority encoder <= 8 cmos3) */
	wire [7:0] mx_nib_nz;
	wire [1:0] mx_nib_hi [0:7];
	genvar gmx;
	generate
	for (gmx = 0; gmx < 8; gmx = gmx + 1) begin : g_mx_nib
		wire [3:0] nb = s1_mx[gmx*4 +: 4];
		assign mx_nib_nz[gmx] = |nb;
		assign mx_nib_hi[gmx] = nb[3] ? 2'd3 : nb[2] ? 2'd2 : nb[1] ? 2'd1 : 2'd0;
	end
	endgenerate
	wire [1:0] mx_q_nz = { |mx_nib_nz[7:4], |mx_nib_nz[3:0] };
	wire [3:0] mx_q_lo = mx_nib_nz[3] ? {2'd3, mx_nib_hi[3]} :
	                     mx_nib_nz[2] ? {2'd2, mx_nib_hi[2]} :
	                     mx_nib_nz[1] ? {2'd1, mx_nib_hi[1]} : {2'd0, mx_nib_hi[0]};
	wire [3:0] mx_q_hi = mx_nib_nz[7] ? {2'd3, mx_nib_hi[7]} :
	                     mx_nib_nz[6] ? {2'd2, mx_nib_hi[6]} :
	                     mx_nib_nz[5] ? {2'd1, mx_nib_hi[5]} : {2'd0, mx_nib_hi[4]};
	wire [5:0] p_mx_w = s1_mx[32] ? 6'd32 :
	                    mx_q_nz[1] ? {2'b01, mx_q_hi} :
	                    mx_q_nz[0] ? {2'b00, mx_q_lo} : 6'd0;
	wire mx_zero_comb = (s1_mx == 33'd0);

	/* Stage 1A -> Stage 1B Pipeline Registers (when LAT >= 5) */
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [5:0]  r1a_p_mx_0, r1a_p_mx_1, r1a_p_mx_2, r1a_p_mx_3, r1a_p_mx_c;
	(* extract_reset = "no" *) reg [32:0] r1a_mx;
	reg        r1a_mx_zero;
	(* srl_style = "register", extract_reset = "no" *) reg [16:0] r1a_nbias;
	reg        r1a_is3d;
	always @(posedge clk) begin
		if (en) begin
			r1a_p_mx_0 <= p_mx_w;
			r1a_p_mx_1 <= p_mx_w;
			r1a_p_mx_2 <= p_mx_w;
			r1a_p_mx_3 <= p_mx_w;
			r1a_p_mx_c <= p_mx_w;
			r1a_mx     <= s1_mx;
			r1a_nbias  <= s1_nbias;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r1a_mx_zero <= 1'b0;
			r1a_is3d    <= 1'b0;
		end else if (en) begin
			r1a_mx_zero <= mx_zero_comb;
			r1a_is3d    <= s1_is3d;
		end
	end

	wire [5:0]  s1b_p_mx_0  = (LAT >= 5) ? r1a_p_mx_0  : p_mx_w;
	wire [5:0]  s1b_p_mx_1  = (LAT >= 5) ? r1a_p_mx_1  : p_mx_w;
	wire [5:0]  s1b_p_mx_2  = (LAT >= 5) ? r1a_p_mx_2  : p_mx_w;
	wire [5:0]  s1b_p_mx_3  = (LAT >= 5) ? r1a_p_mx_3  : p_mx_w;
	wire [5:0]  s1b_p_mx_c  = (LAT >= 5) ? r1a_p_mx_c  : p_mx_w;
	wire [32:0] s1b_mx      = (LAT >= 5) ? r1a_mx      : s1_mx;
	wire        s1b_mx_zero = (LAT >= 5) ? r1a_mx_zero : mx_zero_comb;
	wire [16:0] s1b_nbias   = (LAT >= 5) ? r1a_nbias   : s1_nbias;
	wire        s1b_is3d    = (LAT >= 5) ? r1a_is3d    : s1_is3d;

	wire [40:0] mx_ext = {s1b_mx, 8'd0};
	function [1:0] sel_mx2;
		input [33:0] sl;
		input [5:0]  idx;
		case (idx)
		6'd0:  sel_mx2 = sl[1:0];
		6'd1:  sel_mx2 = sl[2:1];
		6'd2:  sel_mx2 = sl[3:2];
		6'd3:  sel_mx2 = sl[4:3];
		6'd4:  sel_mx2 = sl[5:4];
		6'd5:  sel_mx2 = sl[6:5];
		6'd6:  sel_mx2 = sl[7:6];
		6'd7:  sel_mx2 = sl[8:7];
		6'd8:  sel_mx2 = sl[9:8];
		6'd9:  sel_mx2 = sl[10:9];
		6'd10: sel_mx2 = sl[11:10];
		6'd11: sel_mx2 = sl[12:11];
		6'd12: sel_mx2 = sl[13:12];
		6'd13: sel_mx2 = sl[14:13];
		6'd14: sel_mx2 = sl[15:14];
		6'd15: sel_mx2 = sl[16:15];
		6'd16: sel_mx2 = sl[17:16];
		6'd17: sel_mx2 = sl[18:17];
		6'd18: sel_mx2 = sl[19:18];
		6'd19: sel_mx2 = sl[20:19];
		6'd20: sel_mx2 = sl[21:20];
		6'd21: sel_mx2 = sl[22:21];
		6'd22: sel_mx2 = sl[23:22];
		6'd23: sel_mx2 = sl[24:23];
		6'd24: sel_mx2 = sl[25:24];
		6'd25: sel_mx2 = sl[26:25];
		6'd26: sel_mx2 = sl[27:26];
		6'd27: sel_mx2 = sl[28:27];
		6'd28: sel_mx2 = sl[29:28];
		6'd29: sel_mx2 = sl[30:29];
		6'd30: sel_mx2 = sl[31:30];
		6'd31: sel_mx2 = sl[32:31];
		6'd32: sel_mx2 = sl[33:32];
		default: sel_mx2 = 2'd0;
		endcase
	endfunction
	wire [7:0] m_mx_comb = {
		sel_mx2(mx_ext[39:6], s1b_p_mx_3),
		sel_mx2(mx_ext[37:4], s1b_p_mx_2),
		sel_mx2(mx_ext[35:2], s1b_p_mx_1),
		sel_mx2(mx_ext[33:0], s1b_p_mx_0)
	};
	wire [13:0] l2_mx_comb = {s1b_p_mx_c, m_mx_comb};

	/* Stage 1B -> Stage 1C Pipeline Registers (when LAT >= 5) */
	(* extract_reset = "no" *) reg [13:0] r1b_l2_mx;
	reg        r1b_mx_zero;
	(* srl_style = "register", extract_reset = "no" *) reg [16:0] r1b_nbias;
	reg        r1b_is3d;
	always @(posedge clk) begin
		if (en) begin
			r1b_l2_mx <= l2_mx_comb;
			r1b_nbias <= s1b_nbias;
		end
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r1b_mx_zero <= 1'b0;
			r1b_is3d    <= 1'b0;
		end else if (en) begin
			r1b_mx_zero <= s1b_mx_zero;
			r1b_is3d    <= s1b_is3d;
		end
	end

	wire [13:0] s1c_l2_mx   = (LAT >= 5) ? r1b_l2_mx   : l2_mx_comb;
	wire        s1c_mx_zero = (LAT >= 5) ? r1b_mx_zero : s1b_mx_zero;
	wire [16:0] s1c_nbias   = (LAT >= 5) ? r1b_nbias   : s1b_nbias;
	wire        s1c_is3d    = (LAT >= 5) ? r1b_is3d    : s1b_is3d;

	/* Stage 1C: log2(mx) + (log2(n) - 1.0) (Q8 in a signed 17-bit value). */
	wire [16:0] lsum;
	am_add #(.W(17)) u_lsum (.a({3'd0, s1c_l2_mx}), .b(s1c_nbias), .ci(1'b0), .s(lsum),
		.co(lod_carry_unused[5]));

	/* Stage 1C -> Stage 1D Pipeline Registers (when LAT >= 5) */
	(* extract_reset = "no" *) reg [16:0] r1c_lsum;
	reg        r1c_mx_zero;
	reg        r1c_is3d;
	always @(posedge clk) begin
		if (en) r1c_lsum <= lsum;
	end
	always @(posedge clk or posedge rst_inv) begin
		if (rst_inv) begin
			r1c_mx_zero <= 1'b0;
			r1c_is3d    <= 1'b0;
		end else if (en) begin
			r1c_mx_zero <= s1c_mx_zero;
			r1c_is3d    <= s1c_is3d;
		end
	end

	wire [16:0] s1d_lsum    = (LAT >= 5) ? r1c_lsum    : lsum;
	wire        s1d_mx_zero = (LAT >= 5) ? r1c_mx_zero : s1c_mx_zero;
	wire        s1d_is3d    = (LAT >= 5) ? r1c_is3d    : s1c_is3d;

	wire signed [16:0] lam_comb = ax[1] && !s1d_is3d ? LAM_MIN : s1d_mx_zero ? LAM_MIN : s1d_lsum;

	/* --- Stage 1 -> Output Pipeline Register (LAT >= 3) --- */
	(* extract_reset = "no" *) reg signed [16:0] r_lam;
	always @(posedge clk) begin
		if (en) r_lam <= lam_comb;
	end

	assign lam = (LAT >= 3) ? r_lam : lam_comb;

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, rst_n, en} : 1'b0;
	wire unused_ok = &{1'b0, cs[7:0], cs[34:27], cs[61:54], ct[7:0], ct[34:27], ct[61:54], unused_lat0, m_cmp_d_unused, lod_carry_unused[6], mx_ext[40]};
endmodule

`default_nettype wire
