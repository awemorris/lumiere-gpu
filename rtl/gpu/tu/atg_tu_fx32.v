/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_fx32: floor(f × 2^sh), saturated to 32 bits; zero, denormal and
 * NaN give 0 (the former function tu_fx32 of include/atg_tu_fn.vh;
 * the reference model tumodel fx32).
 *
 * Latency:
 * - LAT = 0: combinational
 * - LAT = 1: 1 cycle pipeline register between barrel shift and negate/clamp
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_fx32 #(
	parameter LAT = 0,
	parameter SH_CONST = -1
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] f,
	input wire [4:0] sh,
	output wire [31:0] q
);
	wire [3:0] unused_rst = 4'd0;

	/* k = e + sh - 150 and nk = 150 - sh - e via 3:2 CSAs + single 10b add (<= 11 cmos3) */
	wire [9:0] k_s0, nk_s0;
	wire [31:0] r;
	wire [3:0] carry_unused;
	/* Fast 10b 2-bit-pair CLA adders for k_s0 = e + sh - 150 and nk_s0 = 150 - sh - e (<= 8 cmos3) */
	wire [9:0] k_a_in, k_b_in, nk_a_in, nk_b_in;
	generate
	if (SH_CONST >= 0) begin : g_const_sh
		localparam [9:0] K_C = (SH_CONST - 150) & 10'h3FF;
		localparam [9:0] NK_C = (151 - SH_CONST) & 10'h3FF; /* 150 - SH_CONST + 1 for ~e */
		assign k_a_in  = {2'b00, f[30:23]};
		assign k_b_in  = K_C;
		assign nk_a_in = {2'b11, ~f[30:23]};
		assign nk_b_in = NK_C;
	end else begin : g_var_sh
		wire [9:0] k_a1 = {2'b00, f[30:23]};
		wire [9:0] k_b1 = {5'b00000, sh};
		wire [9:0] k_c1 = 10'h36A;	/* -150 mod 1024 */
		assign k_a_in = k_a1 ^ k_b1 ^ k_c1;
		assign k_b_in = {((k_a1[8:0] & k_b1[8:0]) | (k_a1[8:0] & k_c1[8:0]) | (k_b1[8:0] & k_c1[8:0])), 1'b0};

		wire [9:0] nk_a2 = 10'd152;	/* 150 + 2 for ~sh + ~e */
		wire [9:0] nk_b2 = {5'b11111, ~sh};
		wire [9:0] nk_c2 = {2'b11, ~f[30:23]};
		assign nk_a_in = nk_a2 ^ nk_b2 ^ nk_c2;
		assign nk_b_in = {((nk_a2[8:0] & nk_b2[8:0]) | (nk_a2[8:0] & nk_c2[8:0]) | (nk_b2[8:0] & nk_c2[8:0])), 1'b0};
	end
	endgenerate
	wire [4:0] k_g2, k_p2;
	genvar gk2;
	generate
	for (gk2 = 0; gk2 < 5; gk2 = gk2 + 1) begin : g_k_cla
		wire [1:0] a2 = k_a_in[gk2*2 +: 2];
		wire [1:0] b2 = k_b_in[gk2*2 +: 2];
		wire g0 = a2[0] && b2[0], p0 = a2[0] ^ b2[0];
		wire g1 = a2[1] && b2[1], p1 = a2[1] ^ b2[1];
		assign k_g2[gk2] = g1 || (p1 && g0);
		assign k_p2[gk2] = p1 && p0;
	end
	endgenerate
	wire [4:0] k_cin;
	assign k_cin[0] = 1'b0;
	assign k_cin[1] = k_g2[0];
	assign k_cin[2] = k_g2[1] || (k_p2[1] && k_g2[0]);
	assign k_cin[3] = k_g2[2] || (k_p2[2] && (k_g2[1] || (k_p2[1] && k_g2[0])));
	assign k_cin[4] = k_g2[3] || (k_p2[3] && k_g2[2]) || (k_p2[3] && k_p2[2] && (k_g2[1] || (k_p2[1] && k_g2[0])));
	generate
	for (gk2 = 0; gk2 < 5; gk2 = gk2 + 1) begin : g_k_sum
		wire [1:0] a2 = k_a_in[gk2*2 +: 2];
		wire [1:0] b2 = k_b_in[gk2*2 +: 2];
		wire c_in = k_cin[gk2];
		assign k_s0[gk2*2 + 0] = (a2[0] ^ b2[0]) ^ c_in;
		assign k_s0[gk2*2 + 1] = (a2[1] ^ b2[1]) ^ ((a2[0] && b2[0]) || ((a2[0] ^ b2[0]) && c_in));
	end
	endgenerate

	wire [4:0] nk_g2, nk_p2;
	generate
	for (gk2 = 0; gk2 < 5; gk2 = gk2 + 1) begin : g_nk_cla
		wire [1:0] a2 = nk_a_in[gk2*2 +: 2];
		wire [1:0] b2 = nk_b_in[gk2*2 +: 2];
		wire g0 = a2[0] && b2[0], p0 = a2[0] ^ b2[0];
		wire g1 = a2[1] && b2[1], p1 = a2[1] ^ b2[1];
		assign nk_g2[gk2] = g1 || (p1 && g0);
		assign nk_p2[gk2] = p1 && p0;
	end
	endgenerate
	wire [4:0] nk_cin;
	assign nk_cin[0] = 1'b0;
	assign nk_cin[1] = nk_g2[0];
	assign nk_cin[2] = nk_g2[1] || (nk_p2[1] && nk_g2[0]);
	assign nk_cin[3] = nk_g2[2] || (nk_p2[2] && (nk_g2[1] || (nk_p2[1] && nk_g2[0])));
	assign nk_cin[4] = nk_g2[3] || (nk_p2[3] && nk_g2[2]) || (nk_p2[3] && nk_p2[2] && (nk_g2[1] || (nk_p2[1] && nk_g2[0])));
	generate
	for (gk2 = 0; gk2 < 5; gk2 = gk2 + 1) begin : g_nk_sum
		wire [1:0] a2 = nk_a_in[gk2*2 +: 2];
		wire [1:0] b2 = nk_b_in[gk2*2 +: 2];
		wire c_in = nk_cin[gk2];
		assign nk_s0[gk2*2 + 0] = (a2[0] ^ b2[0]) ^ c_in;
		assign nk_s0[gk2*2 + 1] = (a2[1] ^ b2[1]) ^ ((a2[0] && b2[0]) || ((a2[0] ^ b2[0]) && c_in));
	end
	endgenerate
	wire unused_cla_top = &{1'b0, k_g2[4], k_p2[4], nk_g2[4], nk_p2[4], (SH_CONST >= 0 ? sh : 5'd0)};
	assign carry_unused[2:0] = {2'd0, unused_cla_top};

	/* Stage 0A -> Stage 0B Pipeline Registers (when LAT >= 2, reset-free datapath) */
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [9:0]  r0_k_a, r0_k_b, r0_k_c;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [9:0]  r0_nk_a, r0_nk_b, r0_nk_c, r0_nk_d;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [31:0] r0_f_l, r0_f_r, r0_f_s;
	always @(posedge clk) begin
		if (en) begin
			r0_k_a  <= k_s0;
			r0_k_b  <= k_s0;
			r0_k_c  <= k_s0;
			r0_nk_a <= nk_s0;
			r0_nk_b <= nk_s0;
			r0_nk_c <= nk_s0;
			r0_nk_d <= nk_s0;
			r0_f_l  <= f;
			r0_f_r  <= f;
			r0_f_s  <= f;
		end
	end

	wire [9:0]  k_a    = (LAT >= 2) ? r0_k_a  : k_s0;
	wire [9:0]  k_b    = (LAT >= 2) ? r0_k_b  : k_s0;
	wire [9:0]  k      = (LAT >= 2) ? r0_k_c  : k_s0;
	wire [9:0]  nk_a   = (LAT >= 2) ? r0_nk_a : nk_s0;
	wire [9:0]  nk_b   = (LAT >= 2) ? r0_nk_b : nk_s0;
	wire [9:0]  nk     = (LAT >= 2) ? r0_nk_c : nk_s0;
	wire [9:0]  nk_st1 = (LAT >= 2) ? r0_nk_d : nk_s0;
	wire [31:0] f_s_l  = (LAT >= 2) ? r0_f_l  : f;
	wire [31:0] f_s_r  = (LAT >= 2) ? r0_f_r  : f;
	wire [31:0] f_s    = (LAT >= 2) ? r0_f_s  : f;

	(* keep = "true" *) wire [3:0] kl_s = {4{!k[9] && k[8:3] == 6'd0}};	/* a left shift (0 <= k < 8) */
	(* keep = "true" *) wire [3:0] kr_s = {4{k[9] && nk[9:5] == 5'd0 && (nk[4:3] != 2'b11)}};	/* a right shift (-24 < k < 0) */
	wire kr_st = k[9] && nk[9:5] == 5'd0 && (nk[4:3] != 2'b11);
	wire [31:0] qs_l_lo, qs_l_hi, qs_r_lo, qs_r_hi;
	am_shift #(.W(32), .SW(3)) u_qs_l_lo (.a({8'd0, 1'b1, f_s_l[22:0]}), .amt(k_a[2:0]), .left(1'b1),
		.arith(1'b0), .y(qs_l_lo));
	am_shift #(.W(32), .SW(3)) u_qs_l_hi (.a({8'd0, 1'b1, f_s_l[22:0]}), .amt(k_b[2:0]), .left(1'b1),
		.arith(1'b0), .y(qs_l_hi));
	am_shift #(.W(32), .SW(5)) u_qs_r_lo (.a({8'd0, 1'b1, f_s_r[22:0]}), .amt(nk_a[4:0]), .left(1'b0),
		.arith(1'b0), .y(qs_r_lo));
	am_shift #(.W(32), .SW(5)) u_qs_r_hi (.a({8'd0, 1'b1, f_s_r[22:0]}), .amt(nk_b[4:0]), .left(1'b0),
		.arith(1'b0), .y(qs_r_hi));
	wire [31:0] qs_l = {qs_l_hi[31:16], qs_l_lo[15:0]};
	wire [31:0] qs_r = {8'd0, qs_r_hi[23:12], qs_r_lo[11:0]};
	wire [23:0] f_s_pad = {1'b0, f_s[22:0]};
	wire [5:0]  st_nib;
	wire [5:0]  nib_gt;
	assign nib_gt[0] = |nk_st1[4:2];
	assign nib_gt[1] = nk_st1[4] || nk_st1[3];
	assign nib_gt[2] = nk_st1[4] || (nk_st1[3] && nk_st1[2]);
	assign nib_gt[3] = nk_st1[4];
	assign nib_gt[4] = nk_st1[4] && nk_st1[2];
	assign nib_gt[5] = 1'b0;
	genvar gb_st;
	generate
	for (gb_st = 0; gb_st < 6; gb_st = gb_st + 1) begin : g_st_nib
		wire [3:0] nib = f_s_pad[gb_st*4 +: 4];
		wire nib_all  = |nib;
		wire nib_part = (nk[1:0] == 2'd1) ? nib[0] :
		                (nk[1:0] == 2'd2) ? (|nib[1:0]) :
		                (nk[1:0] == 2'd3) ? (|nib[2:0]) : 1'b0;
		assign st_nib[gb_st] = (nib_gt[gb_st] && nib_all) || ((nk_st1[4:2] == gb_st[2:0]) && nib_part);
	end
	endgenerate
	wire st = (k[9] && !kr_st) || (kr_st && (|st_nib));
	wire [31:0] qv_s0 = {
		kl_s[3] ? qs_l[31:24] : kr_s[3] ? qs_r[31:24] : 8'd0,
		kl_s[2] ? qs_l[23:16] : kr_s[2] ? qs_r[23:16] : 8'd0,
		kl_s[1] ? qs_l[15:8]  : kr_s[1] ? qs_r[15:8]  : 8'd0,
		kl_s[0] ? qs_l[7:0]   : kr_s[0] ? qs_r[7:0]   : 8'd0
	};
	wire st_s0 = st;
	wire sign_s0 = f_s[31];
	wire zero_s0 = (f_s[30:23] == 8'd0) || (f_s[30:23] == 8'hFF && f_s[22:0] != 23'd0);
	wire ovf_s0  = (f_s[30:23] == 8'hFF) || (!k[9] && k[8:3] != 6'd0);

	wire neg_ci_s0 = sign_s0 && !st_s0;
	wire use_r_s0  = !zero_s0 && !ovf_s0 &&  sign_s0;
	wire use_q_s0  = !zero_s0 && !ovf_s0 && !sign_s0;
	(* dont_touch = "true" *) wire alt_lo_s0_0 = (!zero_s0 && ovf_s0 && !sign_s0);
	(* dont_touch = "true" *) wire alt_lo_s0_1 = (!zero_s0 && ovf_s0 && !sign_s0);
	(* dont_touch = "true" *) wire alt_hi_s0_0 = (!zero_s0 && ovf_s0 && !sign_s0);
	(* dont_touch = "true" *) wire alt_hi_s0_1 = (!zero_s0 && ovf_s0 && !sign_s0);
	(* dont_touch = "true" *) wire alt_hi_s0_m = (!zero_s0 && ovf_s0 &&  sign_s0);
	wire [15:0] alt_lo_s0 = {{8{alt_lo_s0_1}}, {8{alt_lo_s0_0}}};
	wire [15:0] alt_hi_s0 = {alt_hi_s0_m, {7{alt_hi_s0_1}}, {8{alt_hi_s0_0}}};

	/* Stage 0 -> Stage 1 Pipeline Registers (when LAT >= 1) */
	(* extract_reset = "no" *) reg [31:0] r_qv;
	reg r_neg_ci;
	(* keep = "true", equivalent_register_removal = "no" *) reg [3:0] r_use_r_b, r_use_q_b;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [15:0] r_alt_lo, r_alt_hi;

	always @(posedge clk) begin
		if (en) begin
			r_qv     <= qv_s0;
			r_alt_lo <= alt_lo_s0;
			r_alt_hi <= alt_hi_s0;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_neg_ci  <= 1'b0;
			r_use_r_b <= 4'd0;
			r_use_q_b <= 4'd0;
		end else if (en) begin
			r_neg_ci  <= neg_ci_s0;
			r_use_r_b <= {4{use_r_s0}};
			r_use_q_b <= {4{use_q_s0}};
		end
	end

	wire [31:0] s1_qv_raw   = (LAT >= 1) ? r_qv       : qv_s0;
	wire        s1_neg_ci   = (LAT >= 1) ? r_neg_ci   : neg_ci_s0;
	wire [3:0]  s1_use_r    = (LAT >= 1) ? r_use_r_b  : {4{use_r_s0}};
	wire [3:0]  s1_use_q    = (LAT >= 1) ? r_use_q_b  : {4{use_q_s0}};
	wire [15:0] s1_alt_lo   = (LAT >= 1) ? r_alt_lo   : alt_lo_s0;
	wire [15:0] s1_alt_hi   = (LAT >= 1) ? r_alt_hi   : alt_hi_s0;

	/* Compute ~r_qv + !r_st in parallel with r_qv using 3-ary CMOS3 trees (<= 9 cmos3) */
	wire [7:0] r_lo3_z;
	wire [9:0] r_g3;
	genvar grn;
	generate
	for (grn = 0; grn < 8; grn = grn + 1) begin : g_r_lo3
		assign r_lo3_z[grn] = ~(|s1_qv_raw[grn*4 +: 3]);
	end
	for (grn = 0; grn < 10; grn = grn + 1) begin : g_r_g3
		assign r_g3[grn] = ~(|s1_qv_raw[grn*3 +: 3]);
	end
	endgenerate
	wire r_nz9_0 = ~(&r_g3[2:0]);
	wire r_nz9_1 = ~(&r_g3[5:3]);
	wire r_nz6_2 = ~(&r_g3[7:6]);
	wire r_nz9_2 = ~(&r_g3[8:6]);
	wire r_z18   = ~(|{r_nz9_0, r_nz9_1});
	wire r_z24   = ~(|{r_nz9_0, r_nz9_1, r_nz6_2});
	wire r_z27   = ~(|{r_nz9_0, r_nz9_1, r_nz9_2});
	wire [3:0] r_cin;
	assign r_cin[0] = s1_neg_ci;
	assign r_cin[1] = s1_neg_ci && (s1_qv_raw[3:0] == 4'd0);
	assign r_cin[2] = s1_neg_ci && (s1_qv_raw[7:0] == 8'd0);
	assign r_cin[3] = s1_neg_ci && !r_nz9_0 && r_g3[3];
	generate
	for (grn = 0; grn < 4; grn = grn + 1) begin : g_r_sum
		wire [3:0] rn = ~s1_qv_raw[grn*4 +: 4];
		wire rc = r_cin[grn];
		assign r[grn*4 + 0] = rn[0] ^ rc;
		assign r[grn*4 + 1] = rn[1] ^ (rc && rn[0]);
		assign r[grn*4 + 2] = rn[2] ^ (rc && rn[1] && rn[0]);
		assign r[grn*4 + 3] = rn[3] ^ (rc && r_lo3_z[grn]);
	end
	endgenerate
	/* Direct shallow carries for [31:16] via 3-ary tree (<= 9 cmos3 total to q_out) */
	wire r_z15_9 = r_g3[3] && r_g3[4] && !s1_qv_raw[15];
	wire r_z16_9 = r_g3[3] && r_g3[4] && (s1_qv_raw[16:15] == 2'd0);
	wire r_hi_16 = r_z15_9 && s1_neg_ci;
	wire r_hi_17 = r_z16_9 && s1_neg_ci;
	wire r_hi_19 = !s1_qv_raw[18] && s1_neg_ci;
	wire r_hi_20 = !(s1_qv_raw[19] || s1_qv_raw[18]) && s1_neg_ci;
	wire r_hi_21 = r_g3[6] && s1_neg_ci;
	wire r_hi_22 = r_g3[6] && !s1_qv_raw[21] && s1_neg_ci;
	wire r_hi_23 = r_g3[6] && !(s1_qv_raw[22] || s1_qv_raw[21]) && s1_neg_ci;
	wire r_hi_25 = !s1_qv_raw[24] && s1_neg_ci;
	wire r_hi_26 = !(s1_qv_raw[25] || s1_qv_raw[24]) && s1_neg_ci;
	wire r_hi_28 = !s1_qv_raw[27] && s1_neg_ci;
	wire r_hi_29 = !(s1_qv_raw[28] || s1_qv_raw[27]) && s1_neg_ci;
	wire r_hi_30 = r_g3[9] && s1_neg_ci;
	wire r_hi_31 = r_g3[9] && !s1_qv_raw[30] && s1_neg_ci;
	wire r_c16 = !r_nz9_0 && r_hi_16;
	wire r_c17 = !r_nz9_0 && r_hi_17;
	wire r_c18 = r_z18 && s1_neg_ci;
	wire r_c19 = r_z18 && r_hi_19;
	wire r_c20 = r_z18 && r_hi_20;
	wire r_c21 = r_z18 && r_hi_21;
	wire r_c22 = r_z18 && r_hi_22;
	wire r_c23 = r_z18 && r_hi_23;
	wire r_c24 = r_z24 && s1_neg_ci;
	wire r_c25 = r_z24 && r_hi_25;
	wire r_c26 = r_z24 && r_hi_26;
	wire r_c27 = r_z27 && s1_neg_ci;
	wire r_c28 = r_z27 && r_hi_28;
	wire r_c29 = r_z27 && r_hi_29;
	wire r_c30 = r_z27 && r_hi_30;
	wire r_c31 = r_z27 && r_hi_31;
	assign r[16] = ~s1_qv_raw[16] ^ r_c16;
	assign r[17] = ~s1_qv_raw[17] ^ r_c17;
	assign r[18] = ~s1_qv_raw[18] ^ r_c18;
	assign r[19] = ~s1_qv_raw[19] ^ r_c19;
	assign r[20] = ~s1_qv_raw[20] ^ r_c20;
	assign r[21] = ~s1_qv_raw[21] ^ r_c21;
	assign r[22] = ~s1_qv_raw[22] ^ r_c22;
	assign r[23] = ~s1_qv_raw[23] ^ r_c23;
	assign r[24] = ~s1_qv_raw[24] ^ r_c24;
	assign r[25] = ~s1_qv_raw[25] ^ r_c25;
	assign r[26] = ~s1_qv_raw[26] ^ r_c26;
	assign r[27] = ~s1_qv_raw[27] ^ r_c27;
	assign r[28] = ~s1_qv_raw[28] ^ r_c28;
	assign r[29] = ~s1_qv_raw[29] ^ r_c29;
	assign r[30] = ~s1_qv_raw[30] ^ r_c30;
	assign r[31] = ~s1_qv_raw[31] ^ r_c31;
	wire unused_lo3_hi = &{1'b0, r_lo3_z[7:4]};
	assign carry_unused[3] = 1'b0;
	wire [31:0] q_base = {
		s1_use_q[3] ? s1_qv_raw[31:24] : s1_alt_hi[15:8],
		s1_use_q[2] ? s1_qv_raw[23:16] : s1_alt_hi[7:0],
		s1_use_q[1] ? s1_qv_raw[15:8]  : s1_alt_lo[15:8],
		s1_use_q[0] ? s1_qv_raw[7:0]   : s1_alt_lo[7:0]
	};
	wire [31:0] q_out = {
		s1_use_r[3] ? r[31:24] : q_base[31:24],
		s1_use_r[2] ? r[23:16] : q_base[23:16],
		s1_use_r[1] ? r[15:8]  : q_base[15:8],
		s1_use_r[0] ? r[7:0]   : q_base[7:0]
	};
	assign q = q_out;

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, rst_n, en, f_s_l[31:23], f_s_r[31:23]} : &{1'b0, f_s_l[31:23], f_s_r[31:23]};
	wire unused_ok = &{1'b0, unused_rst, unused_lo3_hi, unused_lat0, k_a[9:3], k_b[9:3], nk_a[9:5], nk_b[9:5], nk_st1[9:5], qs_l_lo[31:16], qs_l_hi[15:0], qs_r_lo[31:12], qs_r_hi[31:24], qs_r_hi[11:0]};
endmodule

`default_nettype wire
