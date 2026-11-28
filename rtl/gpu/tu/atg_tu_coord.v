/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_coord: the normalized coordinate with 24 fraction bits before
 * the texel coordinate: repeat floor(u × 2^24) mod 2^24, mirrored repeat
 * mod 2^25, clamp to edge and border floor(u × 2^24) clamped to
 * [−16, 17] × 2^24 (the former function tu_coord of
 * include/atg_tu_fn.vh; the reference model tumodel coord_t).
 *
 * Latency:
 * - LAT = 0: combinational
 * - LAT = 1: 1 cycle pipeline register between shift and add/clamp
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_coord #(
	parameter LAT = 0
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] f,
	input wire [1:0] mode,
	output wire signed [29:0] c
);
	/* Stage 0: k = e - 126: 1.f << k (0 <= k < 25) or >> -k with the sticky (-24 < k < 0) */
	wire [9:0] k_s0, nk_s0;
	wire [1:0] carry_unused_0;
	am_sub #(.W(10)) u_k (.a({2'b00, f[30:23]}), .b(10'd126), .bi(1'b0), .d(k_s0),
		.bo(carry_unused_0[0]));
	am_sub #(.W(10)) u_nk (.a(10'd126), .b({2'b00, f[30:23]}), .bi(1'b0), .d(nk_s0), .bo(carry_unused_0[1]));

	/* Stage 0A -> Stage 0B Pipeline Registers (when LAT >= 2, reset-free datapath) */
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [9:0]  r0_k_a, r0_k_b0, r0_k_b1, r0_k_c, r0_k_d;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [9:0]  r0_nk_a, r0_nk_b, r0_nk_c;
	(* keep = "true", equivalent_register_removal = "no", extract_reset = "no" *) reg [31:0] r0_f_l0, r0_f_l1, r0_f_r0, r0_f_r1, r0_f_s;
	(* extract_reset = "no" *) reg [20:0] r0_qs_hi_p;
	(* extract_reset = "no" *) reg r0_kl_lo, r0_kl_hi, r0_kr_lo, r0_kr_hi, r0_kr_st;
	reg [1:0]  r0_mode;
	/* Direct 8-bit exponent bounds (e in [126,150] for kl, [103,125] for kr) in parallel with u_k/u_nk */
	wire [7:0] e_s0 = f[30:23];
	wire kl_s0 = (e_s0[7:1] == 7'b0111111) || (e_s0[7:4] == 4'b1000) ||
	             ((e_s0[7:3] == 5'b10010) && (e_s0[2:0] != 3'd7));
	wire kr_s0 = (e_s0 == 8'd103) || (e_s0[7:3] == 5'b01101) || (e_s0[7:3] == 5'b01110) ||
	             ((e_s0[7:3] == 5'b01111) && (e_s0[2:0] <= 3'd5));
	/* Stage-0 k[4:3] decode from exponent e_s0[7,4:1] (5 inputs = 1 LUT each) to precompute qs_hi_p before Stage 1 */
	wire k_hi_0_s0 = !e_s0[7] || (!e_s0[4] && !e_s0[3] && (!e_s0[2] || !e_s0[1]));
	wire k_hi_1_s0 = e_s0[7] && !e_s0[4] && ((e_s0[3:1] == 3'b011) || (e_s0[3:2] == 2'b10) || (e_s0[3:1] == 3'b110));
	wire k_hi_2_s0 = e_s0[7] && ((e_s0[4:1] == 4'b0111) || (e_s0[4:2] == 3'b100) || (e_s0[4:1] == 4'b1010));
	(* keep = "true" *) wire [1:0] k_hi_s0_lo = {!k_hi_0_s0 && !k_hi_1_s0, k_hi_1_s0 || (!k_hi_0_s0 && !k_hi_2_s0)};
	(* keep = "true" *) wire [1:0] k_hi_s0_hi = {!k_hi_0_s0 && !k_hi_1_s0, k_hi_1_s0 || (!k_hi_0_s0 && !k_hi_2_s0)};
	wire [20:0] qs_hi_p_s0;
	assign qs_hi_p_s0[10:0] = (k_hi_s0_lo == 2'd0) ? f[18:8] :
	                          (k_hi_s0_lo == 2'd1) ? f[10:0] :
	                          (k_hi_s0_lo == 2'd2) ? {f[2:0], 8'd0} : 11'd0;
	assign qs_hi_p_s0[20:11] = (k_hi_s0_hi == 2'd0) ? {5'd0, 1'b1, f[22:19]} :
	                           (k_hi_s0_hi == 2'd1) ? f[20:11] :
	                           (k_hi_s0_hi == 2'd2) ? f[12:3] : {f[4:0], 5'd0};
	always @(posedge clk) begin
		if (en) begin
			r0_k_a  <= k_s0;
			r0_k_b0 <= k_s0;
			r0_k_b1 <= k_s0;
			r0_k_c  <= k_s0;
			r0_k_d  <= k_s0;
			r0_nk_a <= nk_s0;
			r0_nk_b <= nk_s0;
			r0_nk_c <= nk_s0;
			r0_kl_lo <= kl_s0;
			r0_kl_hi <= kl_s0;
			r0_kr_lo <= kr_s0;
			r0_kr_hi <= kr_s0;
			r0_kr_st <= kr_s0;
			r0_f_l0 <= f;
			r0_f_l1 <= f;
			r0_f_r0 <= f;
			r0_f_r1 <= f;
			r0_f_s  <= f;
			r0_qs_hi_p <= qs_hi_p_s0;
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			r0_mode <= 2'd0;
		else if (en)
			r0_mode <= mode;
	end

	wire [9:0]  k_a    = (LAT >= 2) ? r0_k_a  : k_s0;
	wire [9:0]  k_b0   = (LAT >= 2) ? r0_k_b0 : k_s0;
	wire [9:0]  k_b1   = (LAT >= 2) ? r0_k_b1 : k_s0;
	wire [9:0]  k      = (LAT >= 2) ? r0_k_c  : k_s0;
	wire [9:0]  k_hi   = (LAT >= 2) ? r0_k_d  : k_s0;
	wire [9:0]  nk_a   = (LAT >= 2) ? r0_nk_a : nk_s0;
	wire [9:0]  nk_b   = (LAT >= 2) ? r0_nk_b : nk_s0;
	wire [9:0]  nk     = (LAT >= 2) ? r0_nk_c : nk_s0;
	wire [31:0] f_s_l0 = (LAT >= 2) ? r0_f_l0 : f;
	wire [31:0] f_s_l1 = (LAT >= 2) ? r0_f_l1 : f;
	wire [31:0] f_s_r0 = (LAT >= 2) ? r0_f_r0 : f;
	wire [31:0] f_s_r1 = (LAT >= 2) ? r0_f_r1 : f;
	wire [31:0] f_s    = (LAT >= 2) ? r0_f_s  : f;
	wire [1:0]  mode_s = (LAT >= 2) ? r0_mode : mode;

	(* dont_touch = "true" *) wire kl_lo = (LAT >= 2) ? r0_kl_lo : kl_s0;
	wire kl_lo0, kl_lo1;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_kl0 (.I0(kl_lo), .O(kl_lo0));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_kl1 (.I0(kl_lo), .O(kl_lo1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign kl_lo0 = kl_lo;
	assign kl_lo1 = kl_lo;
`else
	assign kl_lo0 = kl_lo;
	assign kl_lo1 = kl_lo;
`endif
	(* dont_touch = "true" *) wire kl_hi = (LAT >= 2) ? r0_kl_hi : kl_s0;
	(* dont_touch = "true" *) wire kr_lo = (LAT >= 2) ? r0_kr_lo : kr_s0;
	(* dont_touch = "true" *) wire kr_hi = (LAT >= 2) ? r0_kr_hi : kr_s0;
	wire kr_st = (LAT >= 2) ? r0_kr_st : kr_s0;
	wire [14:0] sh3_lo_15;
	wire [14:0] qs_l_lo_15 = (k_a[4:3] == 2'd0) ? sh3_lo_15 :
	                         (k_a[4:3] == 2'd1) ? {sh3_lo_15[6:0], 8'd0} : 15'd0;
	wire [13:0] qs_l_hi0, qs_l_hi1;
	wire [28:0] qs_r_lo, qs_r_hi;
	am_shift #(.W(15), .SW(3)) u_qs_l_lo (.a(f_s_l0[14:0]), .amt(k_a[2:0]), .left(1'b1),
		.arith(1'b0), .y(sh3_lo_15));
	wire [20:0] qs_hi_p = (LAT >= 2) ? r0_qs_hi_p : qs_hi_p_s0;
	am_shift #(.W(14), .SW(3)) u_qs_l_hi0 (.a(qs_hi_p[13:0]), .amt(k_b0[2:0]), .left(1'b1),
		.arith(1'b0), .y(qs_l_hi0));
	am_shift #(.W(14), .SW(3)) u_qs_l_hi1 (.a(qs_hi_p[20:7]), .amt(k_b1[2:0]), .left(1'b1),
		.arith(1'b0), .y(qs_l_hi1));
	am_shift #(.W(29), .SW(5)) u_qs_r_lo (.a({5'd0, 1'b1, f_s_r0[22:0]}), .amt(nk_a[4:0]), .left(1'b0),
		.arith(1'b0), .y(qs_r_lo));
	am_shift #(.W(29), .SW(5)) u_qs_r_hi (.a({5'd0, 1'b1, f_s_r1[22:0]}), .amt(nk_b[4:0]), .left(1'b0),
		.arith(1'b0), .y(qs_r_hi));
	wire [28:0] qs_l = {qs_l_hi1[13:7], qs_l_hi0[13:7], qs_l_lo_15};
	wire unused_qs_l = &{1'b0, f_s_l0[22:15], f_s_l1, qs_l_hi0[6:0], qs_l_hi1[6:0], k_hi[9:0], k_b0[9:3], k_b1[9:3]};
	wire [28:0] qs_r = {5'd0, qs_r_hi[23:12], qs_r_lo[11:0]};
	/* Nibble-parallel sticky computation replacing 23-step sequential OR loop */
	wire [23:0] f_s_pad = {1'b0, f_s[22:0]};
	wire [5:0] nib_gt;
	assign nib_gt[0] = (nk[4] || nk[3] || nk[2]);
	assign nib_gt[1] = (nk[4] || nk[3]);
	assign nib_gt[2] = (nk[4] || (nk[3:2] == 2'd3));
	assign nib_gt[3] = nk[4];
	assign nib_gt[4] = (nk[4:2] == 3'd5);
	assign nib_gt[5] = 1'b0;
	wire [5:0] st_nib;
	genvar gc_st;
	generate
	for (gc_st = 0; gc_st < 6; gc_st = gc_st + 1) begin : g_st_nib
		wire [3:0] nib = f_s_pad[gc_st*4 +: 4];
		wire nib_all  = |nib;
		wire nib_part = (nk[1:0] == 2'd1) ? nib[0] :
		                (nk[1:0] == 2'd2) ? (|nib[1:0]) :
		                (nk[1:0] == 2'd3) ? (|nib[2:0]) : 1'b0;
		assign st_nib[gc_st] = (nib_gt[gc_st] && nib_all) || ((nk[4:2] == gc_st[2:0]) && nib_part);
	end
	endgenerate
	wire st = (k[9] && !kr_st) || (kr_st && (|st_nib));
	wire [14:0] q_lo = {
		kl_lo1 ? qs_l[14:8] : kr_lo ? qs_r[14:8] : 7'd0,
		kl_lo0 ? qs_l[7:0]  : kr_lo ? qs_r[7:0]  : 8'd0
	};
	wire [13:0] q_hi = kl_hi ? qs_l[28:15] : kr_hi ? qs_r[28:15] : 14'd0;
	wire [28:0] q = {q_hi, q_lo};
	wire nan_0 = (f_s[30:23] == 8'hFF && f_s[22:0] != 23'd0);
	wire k_ge25_0 = (!k[9] && !kl_lo1);

	/* Stage 1 Pipeline Registers (LAT >= 1) */
	reg [14:0] r_q_lo;
	reg [13:0] r_q_hi;
	reg r_st;
	(* keep = "true", equivalent_register_removal = "no" *) reg r_sign_lo, r_sign_hi;
	reg [7:0] r_exp;
	reg r_nan;
	reg r_k_ge25;
	reg [1:0] r_mode;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_q_lo    <= 15'd0;
			r_q_hi    <= 14'd0;
			r_st      <= 1'b0;
			r_sign_lo <= 1'b0;
			r_sign_hi <= 1'b0;
			r_exp     <= 8'd0;
			r_nan     <= 1'b0;
			r_k_ge25  <= 1'b0;
			r_mode    <= 2'd0;
		end else if (en) begin
			r_q_lo    <= q_lo;
			r_q_hi    <= q_hi;
			r_st      <= st;
			r_sign_lo <= f_s[31];
			r_sign_hi <= f_s[31];
			r_exp     <= f_s[30:23];
			r_nan     <= nan_0;
			r_k_ge25  <= k_ge25_0;
			r_mode    <= mode_s;
		end
	end

	wire [28:0] s_q      = (LAT >= 1) ? {r_q_hi, r_q_lo} : q;
	wire        s_st     = (LAT >= 1) ? r_st      : st;
	wire s_sign_lo = (LAT >= 1) ? r_sign_lo : f_s[31];
	wire s_sign_hi = (LAT >= 1) ? r_sign_hi : f_s[31];
	wire [7:0]  s_exp    = (LAT >= 1) ? r_exp     : f_s[30:23];
	wire        s_nan    = (LAT >= 1) ? r_nan     : nan_0;
	wire        s_k_ge25 = (LAT >= 1) ? r_k_ge25  : k_ge25_0;
	wire [1:0]  s_mode   = (LAT >= 1) ? r_mode    : mode_s;

	/* Stage 2: compute ~s_q + !s_st in parallel with s_q, select by s_sign (<= 10 cmos3) */
	wire [29:0] tv_s1;
	wire carry_unused_1 = 1'b0;
	wire [31:0] tv_inv_pad = {2'b00, 1'b1, ~s_q};
	wire        tv_ci      = !s_st;
	wire [7:0]  tv_nib_all1;
	wire [7:0]  tv_lo3_all1;
	wire [7:0]  tv_cin;
	genvar gtv;
	generate
	for (gtv = 0; gtv < 8; gtv = gtv + 1) begin : g_tv_all1
		assign tv_nib_all1[gtv] = &tv_inv_pad[gtv*4 +: 4];
		assign tv_lo3_all1[gtv] = &tv_inv_pad[gtv*4 +: 3];
	end
	endgenerate
	wire tv_p8_0    = tv_nib_all1[1] && tv_nib_all1[0];
	wire tv_p8_1    = tv_nib_all1[3] && tv_nib_all1[2];
	wire tv_p8_2    = tv_nib_all1[5] && tv_nib_all1[4];
	wire tv_lo_all1 = tv_p8_0 && tv_p8_1;
	assign tv_cin[0] = tv_ci;
	assign tv_cin[1] = tv_ci && tv_nib_all1[0];
	assign tv_cin[2] = tv_ci && tv_p8_0;
	assign tv_cin[3] = tv_ci && (tv_p8_0 && tv_nib_all1[2]);
	assign tv_cin[4] = tv_ci && tv_lo_all1;
	assign tv_cin[5] = tv_ci && (tv_lo_all1 && tv_nib_all1[4]);
	assign tv_cin[6] = tv_ci && (tv_lo_all1 && tv_p8_2);
	assign tv_cin[7] = tv_ci && (tv_lo_all1 && (tv_p8_2 && tv_nib_all1[6]));
	wire [31:0] tv_s_pad;
	generate
	for (gtv = 0; gtv < 8; gtv = gtv + 1) begin : g_tv_sum
		wire [3:0] tn = tv_inv_pad[gtv*4 +: 4];
		wire tc = tv_cin[gtv];
		assign tv_s_pad[gtv*4 + 0] = tn[0] ^ tc;
		assign tv_s_pad[gtv*4 + 1] = tn[1] ^ (tc && tn[0]);
		assign tv_s_pad[gtv*4 + 2] = tn[2] ^ (tc && tn[1] && tn[0]);
		assign tv_s_pad[gtv*4 + 3] = tn[3] ^ (tc && tv_lo3_all1[gtv]);
	end
	endgenerate
	assign tv_s1[14:0]  = s_sign_lo ? tv_s_pad[14:0]  : s_q[14:0];
	assign tv_s1[29:15] = s_sign_hi ? tv_s_pad[29:15] : {1'b0, s_q[28:15]};
	wire unused_tv_pad = &{1'b0, tv_s_pad[31:30], tv_nib_all1[7]};

	/* Check tv_s1[15:0] != 0 directly from s_q[15:0] without waiting for tv_s1 carry chain */
	wire tv_lo16_nz_w = (s_sign_lo && s_st) ? !(&s_q[15:0]) : (s_q[15:0] != 16'd0);

	/* Stage 2A -> Stage 2B Pipeline Registers (when LAT >= 2) */
	reg [29:0] r2_tv;
	reg        r2_tv_lo16_nz;
	reg        r2_sign, r2_nan, r2_k_ge25;
	reg [7:0]  r2_exp;
	(* keep = "true", equivalent_register_removal = "no" *) reg [1:0] r2_mode_0, r2_mode_1, r2_mode_2;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r2_tv         <= 30'd0;
			r2_tv_lo16_nz <= 1'b0;
			r2_sign       <= 1'b0;
			r2_nan        <= 1'b0;
			r2_k_ge25     <= 1'b0;
			r2_exp        <= 8'd0;
			r2_mode_0     <= 2'd0;
			r2_mode_1     <= 2'd0;
			r2_mode_2     <= 2'd0;
		end else if (en) begin
			r2_tv         <= tv_s1;
			r2_tv_lo16_nz <= tv_lo16_nz_w;
			r2_sign       <= s_sign_lo;
			r2_nan        <= s_nan;
			r2_k_ge25     <= s_k_ge25;
			r2_exp        <= s_exp;
			r2_mode_0     <= s_mode;
			r2_mode_1     <= s_mode;
			r2_mode_2     <= s_mode;
		end
	end

	wire [29:0] tv       = (LAT >= 2) ? r2_tv     : tv_s1;
	wire        f_sign   = (LAT >= 2) ? r2_sign   : s_sign_lo;
	wire        f_nan    = (LAT >= 2) ? r2_nan    : s_nan;
	wire        f_k_ge25 = (LAT >= 2) ? r2_k_ge25 : s_k_ge25;
	wire [7:0]  f_exp    = (LAT >= 2) ? r2_exp    : s_exp;
	wire [1:0]  f_mode_0 = (LAT >= 2) ? r2_mode_0 : s_mode;
	wire [1:0]  f_mode_1 = (LAT >= 2) ? r2_mode_1 : s_mode;
	wire [1:0]  f_mode_2 = (LAT >= 2) ? r2_mode_2 : s_mode;

	wire [24:0] t = tv[24:0];
	wire signed [29:0] tvs = tv;
	wire tv_lo24_nz = (LAT >= 2) ? (r2_tv_lo16_nz || (tvs[23:16] != 8'd0)) : (tvs[23:0] != 24'd0);
	wire tv_lt_min = tvs[29] && !tvs[28];
	wire tv_gt_max = !tvs[29] && (tvs[28] && tvs[27:24] != 4'd0 && (tvs[27:24] != 4'd1 || tv_lo24_nz));
	wire exp_ge131 = f_exp[7] && (f_exp[6:2] != 5'd0 || f_exp[1:0] == 2'b11);
	(* keep = "true" *) wire [2:0] zero_nan_b  = {3{(f_exp == 8'd0 || f_nan)}};
	(* keep = "true" *) wire [2:0] clamp_min_b = {3{(exp_ge131 ? f_sign : tv_lt_min)}};
	(* keep = "true" *) wire [2:0] clamp_max_b = {3{(exp_ge131 ? !f_sign : tv_gt_max)}};
	(* keep = "true" *) wire [2:0] wrap_zero_b = {3{(f_exp == 8'hFF || f_k_ge25)}};
	wire [29:0] c_min = -30'sd268435456;
	wire [29:0] c_max =  30'sd285212672;
	wire [29:0] c_wrap = {5'd0, f_mode_2[0] ? t[24] : 1'b0, t[23:0]};

	assign c[9:0]   = zero_nan_b[0] ? 10'd0 :
		f_mode_0[1] ? (clamp_min_b[0] ? c_min[9:0]   : clamp_max_b[0] ? c_max[9:0]   : tvs[9:0])   :
		wrap_zero_b[0] ? 10'd0 : c_wrap[9:0];
	assign c[19:10] = zero_nan_b[1] ? 10'd0 :
		f_mode_1[1] ? (clamp_min_b[1] ? c_min[19:10] : clamp_max_b[1] ? c_max[19:10] : tvs[19:10]) :
		wrap_zero_b[1] ? 10'd0 : c_wrap[19:10];
	assign c[29:20] = zero_nan_b[2] ? 10'd0 :
		f_mode_2[1] ? (clamp_min_b[2] ? c_min[29:20] : clamp_max_b[2] ? c_max[29:20] : tvs[29:20]) :
		wrap_zero_b[2] ? 10'd0 : c_wrap[29:20];

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, rst_n, en, f_s_l0[31:23], f_s_l1[31:23], f_s_r0[31:23], f_s_r1[31:23]} :
	                                &{1'b0, f_s_l0[31:23], f_s_l1[31:23], f_s_r0[31:23], f_s_r1[31:23]};
	wire unused_ok = &{1'b0, carry_unused_0, carry_unused_1, unused_lat0, k_a[9:5], k_b0[9:5], k_b1[9:5], nk_a[9:5], nk_b[9:5], unused_qs_l, qs_r_lo[28:12], qs_r_hi[28:24], qs_r_hi[11:0]};
endmodule

`default_nettype wire
