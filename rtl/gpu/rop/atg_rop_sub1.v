/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_sub1: 5-stage pipelined IEEE 754 float32 computation of (1.0f - x) (LAT = 5).
 *   - Stage 1 (S1A): Pure-LUT specials, magnitude compare |x| <= 1.0, exponent diff dab,
 *                    and operand muxing (0 CARRY4s, <= 3 LUT levels).
 *   - Stage 2 (S1B): Pure-LUT 3-level 4:1 barrel shift, nibble sticky, and sub_op XOR
 *                    (0 CARRY4s, <= 3 LUT levels).
 *   - Stage 3 (S2):  Dedicated 28-bit mantissa adder u_sum driven directly from registers
 *                    and registered directly into r0c_sum (Pre-LUTs = 1, Post-LUTs = 0).
 *   - Stage 4 (S3):  Pre-normalization muxing, rounding bit extraction, and nibble LZC
 *                    from registered r0c_sum (0 CARRY4s, <= 2 LUT levels).
 *   - Stage 5 (S4):  Close-path shift + Far-path pure-LUT 2-level rounding increment +
 *                    final output mux & register (0 CARRY4s, <= 3 LUT levels).
 * RNE rounding, denormals flushed to zero, canonical NaN 0x7FC00000.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_sub1 #(
	parameter LAT = 5
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] i_x,
	input wire [7:0]  i_x_eb,
	output wire [31:0] o_r
);
	(* keep = "true", dont_touch = "true" *) wire rst_sync_lo = !rst_n;
	(* keep = "true", dont_touch = "true" *) wire rst_sync_hi = !rst_n;

	wire [11:0] en_s1a = {12{en}};
	wire [9:0]  en_s1b = {10{en}};
	wire [7:0]  en_s2  = {8{en}};
	wire [13:0] en_s3  = {14{en}};
	wire [3:0]  en_s4  = {4{en}};

	/* =========================================================================
	 * STAGE 1 (S1A): Specials, Pure-LUT Exponent Diff, and Alignment Setup
	 *                (0 CARRY4s, <= 3 LUT levels)
	 * ========================================================================= */
	wire s_x_raw = i_x[31];
`ifdef SYNTH_KINTEX7
	wire s_x, sub_op_lo, sub_op_hi;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sx   (.I0(s_x_raw),  .O(s_x));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h1)) u_sopl (.I0(s_x_raw),  .O(sub_op_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h1)) u_soph (.I0(s_x_raw),  .O(sub_op_hi));
	wire [7:0] ea, ea_cmp;
	genvar gea;
	generate
	for (gea = 0; gea < 8; gea = gea + 1) begin : g_ea_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ea0 (.I0(i_x[23 + gea]), .O(ea[gea]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ea1 (.I0(i_x[23 + gea]), .O(ea_cmp[gea]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire s_x, sub_op_lo, sub_op_hi;
	assign s_x = s_x_raw;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h1)) u_sopl (.I0(s_x_raw),  .O(sub_op_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h1)) u_soph (.I0(s_x_raw),  .O(sub_op_hi));
	wire [7:0] ea, ea_cmp;
	genvar gea;
	generate
	for (gea = 0; gea < 8; gea = gea + 1) begin : g_ea_buf
		assign ea[gea] = i_x[23 + gea];
		assign ea_cmp[gea] = i_x[23 + gea];
	end
	endgenerate
`else
	wire s_x       = s_x_raw;
	wire sub_op_lo = !s_x_raw;
	wire sub_op_hi = !s_x_raw;
	wire [7:0] ea     = i_x[30:23];
	wire [7:0] ea_cmp = i_x[30:23];
`endif
	wire [7:0] eab = i_x_eb;
	wire [22:0] ma = i_x[22:0];

	(* keep = "true" *) wire [3:0] ma_nz;
	assign ma_nz[0] = (ma[5:0]   != 6'd0);
	assign ma_nz[1] = (ma[11:6]  != 6'd0);
	assign ma_nz[2] = (ma[17:12] != 6'd0);
	assign ma_nz[3] = (ma[22:18] != 5'd0);
	(* keep = "true" *) wire ma_is_zero = !(|ma_nz);

	wire is_nan  = (ea_cmp == 8'hFF) && (!ma_is_zero);
	wire is_inf  = (ea_cmp == 8'hFF) && ma_is_zero;
	wire is_zero = (ea_cmp == 8'd0);
	wire is_one  = (!s_x) && (ea_cmp == 8'd127) && ma_is_zero;

	wire sp_s1a = is_nan || is_inf || is_zero || is_one;
	wire [31:0] spv_s1a = is_nan ? 32'h7FC00000 :
		is_inf ? (s_x ? 32'h7F800000 : 32'hFF800000) :
		is_one ? 32'h00000000 : 32'h3F800000;

	wire dab1_bo_unused = 1'b0;
	wire dab2_bo_unused = 1'b0;

	/* Magnitude compare |x| <= 1.0 (ea < 127 || (ea == 127 && ma == 0)) */
	wire ea_lt_127 = !ea_cmp[7] && !(&ea_cmp[6:0]);
	wire ea_eq_127 = !ea_cmp[7] &&  (&ea_cmp[6:0]);
	wire x_le_1_raw = ea_lt_127 || (ea_eq_127 && ma_is_zero);
	(* keep = "true", dont_touch = "true" *) wire [10:0] x_le_1_buf;
	genvar gxl;
	generate
	for (gxl = 0; gxl < 11; gxl = gxl + 1) begin : g_x_le_1_rep
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_xl1 (.I0(x_le_1_raw), .O(x_le_1_buf[gxl]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign x_le_1_buf[gxl] = x_le_1_raw;
`else
		assign x_le_1_buf[gxl] = x_le_1_raw;
`endif
	end
	endgenerate

	/* Pure-LUT |ea - 127|: (~ea[6:0]) when !ea[7], ({1'b0, ea[6:0]} + 1) when ea[7] (<= 5 cmos3, independent of ma) */
	wire [2:0] e_and3 = {ea[5] & ea[4] & ea[3], ea[2] & ea[1] & ea[0], 1'b0};
	wire [7:0] ea_m127 = {
		ea[6] & e_and3[2] & e_and3[1],
		ea[6] ^ (e_and3[2] & e_and3[1]),
		ea[5] ^ (ea[4] & ea[3] & e_and3[1]),
		ea[4] ^ (ea[3] & e_and3[1]),
		ea[3] ^ e_and3[1],
		ea[2] ^ (ea[1] & ea[0]),
		ea[1] ^ ea[0],
		~ea[0]
	};
	(* keep = "true" *) wire [7:0] dab_s1a = ea[7] ? ea_m127 : {1'b0, ~ea[6:0]};
	/* |127 - ea| >= 27 iff ea <= 100 (0x64) or ea >= 154 (0x9A) */
	wire ea_le_100 = !ea_cmp[7] && (!ea_cmp[6] || !ea_cmp[5] || (!ea_cmp[4] && !ea_cmp[3] && (!ea_cmp[2] || (!ea_cmp[1] && !ea_cmp[0]))));
	wire ea_ge_154 =  ea_cmp[7] && ( ea_cmp[6] ||  ea_cmp[5] || ( ea_cmp[4] &&  ea_cmp[3] && ( ea_cmp[2] ||  ea_cmp[1])));
	wire dab_ge27_s1a = ea_le_100 || ea_ge_154;

	(* keep = "true", dont_touch = "true" *) wire [7:0] ex_s1a = x_le_1_buf[1] ? 8'd127 : eab;

	wire [7:0] ea_m1, ea_p1, ea_p2;
	wire eam1_bo_unused, eap1_co_unused, eap2_co_unused;
	am_sub #(.W(8)) u_ea_m1 (.a(eab), .b(8'd1), .bi(1'b0), .d(ea_m1), .bo(eam1_bo_unused));
	am_add #(.W(8)) u_ea_p1 (.a(eab), .b(8'd1), .ci(1'b0), .s(ea_p1), .co(eap1_co_unused));
	am_add #(.W(8)) u_ea_p2 (.a(eab), .b(8'd2), .ci(1'b0), .s(ea_p2), .co(eap2_co_unused));

	(* keep = "true", dont_touch = "true" *) wire [7:0] ex_m1_s1a = x_le_1_buf[1] ? 8'd126 : ea_m1;
	(* keep = "true", dont_touch = "true" *) wire [7:0] ex_p1_s1a = x_le_1_buf[2] ? 8'd128 : ea_p1;
	(* keep = "true", dont_touch = "true" *) wire [7:0] ex_p2_s1a = x_le_1_buf[2] ? 8'd129 : ea_p2;

	wire [23:0] m24 = {1'b1, ma};
	wire sub_op_s1a   = sub_op_lo;
	wire [23:0] m24_x = {sub_op_hi ? ~m24[23:12] : m24[23:12], sub_op_lo ? ~m24[11:0] : m24[11:0]};
	(* keep = "true", dont_touch = "true" *) wire [26:0] shi_s1a;
	assign shi_s1a[12:0]  = x_le_1_buf[3] ? {m24[9:0], 3'b000} : 13'd0;
	assign shi_s1a[26:13] = x_le_1_buf[4] ? m24[23:10] : 14'h2000;
	(* keep = "true", dont_touch = "true" *) wire [26:0] shi_x_s1a;
	assign shi_x_s1a[12:0]  = x_le_1_buf[8] ? {m24_x[9:0], {3{sub_op_lo}}} : {13{sub_op_lo}};
	assign shi_x_s1a[26:13] = x_le_1_buf[9] ? m24_x[23:10] : {~sub_op_hi, {13{sub_op_hi}}};

	(* keep = "true", dont_touch = "true" *) wire [26:0] px_s1a;
	assign px_s1a[13:0]  = x_le_1_buf[5] ? 14'd0    : {m24[10:0], 3'b000};
	assign px_s1a[26:14] = x_le_1_buf[6] ? 13'h1000 : m24[23:11];

	wire sx_s1a       = x_le_1_buf[7] ? 1'b0 : (!s_x);
	wire is_close_s1a = (!s_x) && ((ea_cmp == 8'd126) || (ea_cmp == 8'd127));
	wire [6:0] stk_gt_s1a, stk_eq_s1a;
	genvar gsk;
	generate
	for (gsk = 0; gsk < 7; gsk = gsk + 1) begin : g_stk_cmp_s1a
		assign stk_gt_s1a[gsk] = (dab_s1a[4:2] >  gsk[2:0]);
		assign stk_eq_s1a[gsk] = (dab_s1a[4:2] == gsk[2:0]);
	end
	endgenerate

	/* Stage 1A -> Stage 1B Pipeline Registers */
	(* keep = "true", dont_touch = "true" *) reg [3:0]  r0a_dab0_oh_a0, r0a_dab0_oh_a1;
	(* keep = "true", dont_touch = "true" *) reg [3:0]  r0a_dab0_oh_b0, r0a_dab0_oh_b1;
	(* keep = "true", dont_touch = "true" *) reg [6:0]  r0a_dab1_7oh_a0, r0a_dab1_7oh_a1;
	(* keep = "true", dont_touch = "true" *) reg [3:0]  r0a_dab1_7oh_b;
	(* keep = "true", dont_touch = "true" *) reg        r0a_ge27_sub_a0, r0a_ge27_sub_a1, r0a_ge16_sub_b;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0]  r0a_dab_stk_lo;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [6:0]  r0a_stk_gt, r0a_stk_eq;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r0a_dab_ge27_a;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:0] r0a_shi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:0] r0a_shi_x;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:0] r0a_px;
	(* keep = "true", dont_touch = "true" *) reg        r0a_sub_op_a, r0a_sub_op_b, r0a_sub_op_ci;
	(* srl_style = "register" *) reg [7:0]  r0a_ex, r0a_ex_m1, r0a_ex_p1, r0a_ex_p2;
	(* srl_style = "register" *) reg        r0a_sx, r0a_sp, r0a_is_close;
	(* srl_style = "register" *) reg [31:0] r0a_spv;

	/* =========================================================================
	 * STAGE 2 (S1B): 1-hot Barrel Shift, Nibble Sticky, and Pre-inverted Sub
	 *                (0 CARRY4s, <= 10 cmos3)
	 * ========================================================================= */
	wire [29:0] shi_s0_pad = {{3{r0a_sub_op_b}}, r0a_shi_x};
	wire [26:0] shr_s0;
	genvar gsi;
	generate
	for (gsi = 0; gsi < 7; gsi = gsi + 1) begin : g_shr0_lo0
		assign shr_s0[gsi] =
			(r0a_dab0_oh_a0[0] & shi_s0_pad[gsi])     |
			(r0a_dab0_oh_a0[1] & shi_s0_pad[gsi + 1]) |
			(r0a_dab0_oh_a0[2] & shi_s0_pad[gsi + 2]) |
			(r0a_dab0_oh_a0[3] & shi_s0_pad[gsi + 3]);
	end
	for (gsi = 7; gsi < 14; gsi = gsi + 1) begin : g_shr0_lo1
		assign shr_s0[gsi] =
			(r0a_dab0_oh_a1[0] & shi_s0_pad[gsi])     |
			(r0a_dab0_oh_a1[1] & shi_s0_pad[gsi + 1]) |
			(r0a_dab0_oh_a1[2] & shi_s0_pad[gsi + 2]) |
			(r0a_dab0_oh_a1[3] & shi_s0_pad[gsi + 3]);
	end
	for (gsi = 14; gsi < 21; gsi = gsi + 1) begin : g_shr0_hi0
		assign shr_s0[gsi] =
			(r0a_dab0_oh_b0[0] & shi_s0_pad[gsi])     |
			(r0a_dab0_oh_b0[1] & shi_s0_pad[gsi + 1]) |
			(r0a_dab0_oh_b0[2] & shi_s0_pad[gsi + 2]) |
			(r0a_dab0_oh_b0[3] & shi_s0_pad[gsi + 3]);
	end
	for (gsi = 21; gsi < 27; gsi = gsi + 1) begin : g_shr0_hi1
		assign shr_s0[gsi] =
			(r0a_dab0_oh_b1[0] & shi_s0_pad[gsi])     |
			(r0a_dab0_oh_b1[1] & shi_s0_pad[gsi + 1]) |
			(r0a_dab0_oh_b1[2] & shi_s0_pad[gsi + 2]) |
			(r0a_dab0_oh_b1[3] & shi_s0_pad[gsi + 3]);
	end
	endgenerate
	wire [50:0] shr_s0_pad = {{24{r0a_sub_op_b}}, shr_s0};

	/* Nibble sticky calculation for bits shifted out (k < r0a_dab_stk) */
	wire [27:0] shi_pad = {1'b0, r0a_shi};
	wire [6:0] stk_nib;
	genvar gn;
	generate
	for (gn = 0; gn < 7; gn = gn + 1) begin : g_stk_nib
		wire [3:0] pn = shi_pad[4 * gn +: 4];
		wire nib_or = |pn;
		wire nib_part = (r0a_dab_stk_lo[1] & (r0a_dab_stk_lo[0] ? (|pn[2:0]) : (|pn[1:0]))) |
		                (!r0a_dab_stk_lo[1] & r0a_dab_stk_lo[0] & pn[0]);
		assign stk_nib[gn] = (r0a_stk_gt[gn] & nib_or) | (r0a_stk_eq[gn] & nib_part);
	end
	endgenerate
	wire stk_lo  = |stk_nib[3:0];
	wire stk_hi  = r0a_dab_ge27_a | (|stk_nib[6:4]);
	wire stk_any = stk_lo | stk_hi;
	wire sh0_x =
		(r0a_dab1_7oh_a0[0] & shr_s0_pad[0])  | (r0a_dab1_7oh_a0[1] & shr_s0_pad[4])  |
		(r0a_dab1_7oh_a0[2] & shr_s0_pad[8])  | (r0a_dab1_7oh_a0[3] & shr_s0_pad[12]) |
		(r0a_dab1_7oh_a0[4] & shr_s0_pad[16]) | (r0a_dab1_7oh_a0[5] & shr_s0_pad[20]) |
		(r0a_dab1_7oh_a0[6] & shr_s0_pad[24]);

	wire [27:0] py_b_s1b;
	assign py_b_s1b[0] = r0a_sub_op_a ? (sh0_x & ~stk_any) : (sh0_x | stk_any);
	genvar gpy;
	generate
	for (gpy = 1; gpy < 7; gpy = gpy + 1) begin : g_py_lo0
		assign py_b_s1b[gpy] =
			(((r0a_dab1_7oh_a0[0] & shr_s0_pad[gpy + 0])  | (r0a_dab1_7oh_a0[1] & shr_s0_pad[gpy + 4]))  |
			 ((r0a_dab1_7oh_a0[2] & shr_s0_pad[gpy + 8])  | (r0a_dab1_7oh_a0[3] & shr_s0_pad[gpy + 12]))) |
			(((r0a_dab1_7oh_a0[4] & shr_s0_pad[gpy + 16]) | (r0a_dab1_7oh_a0[5] & shr_s0_pad[gpy + 20])) |
			 ((r0a_dab1_7oh_a0[6] & shr_s0_pad[gpy + 24]) | r0a_ge27_sub_a0));
	end
	for (gpy = 7; gpy < 14; gpy = gpy + 1) begin : g_py_lo1
		assign py_b_s1b[gpy] =
			(((r0a_dab1_7oh_a1[0] & shr_s0_pad[gpy + 0])  | (r0a_dab1_7oh_a1[1] & shr_s0_pad[gpy + 4]))  |
			 ((r0a_dab1_7oh_a1[2] & shr_s0_pad[gpy + 8])  | (r0a_dab1_7oh_a1[3] & shr_s0_pad[gpy + 12]))) |
			(((r0a_dab1_7oh_a1[4] & shr_s0_pad[gpy + 16]) | (r0a_dab1_7oh_a1[5] & shr_s0_pad[gpy + 20])) |
			 ((r0a_dab1_7oh_a1[6] & shr_s0_pad[gpy + 24]) | r0a_ge27_sub_a1));
	end
	for (gpy = 14; gpy < 27; gpy = gpy + 1) begin : g_py_hi
		assign py_b_s1b[gpy] =
			((r0a_dab1_7oh_b[0] & shr_s0_pad[gpy + 0])  | (r0a_dab1_7oh_b[1] & shr_s0_pad[gpy + 4])) |
			((r0a_dab1_7oh_b[2] & shr_s0_pad[gpy + 8])  | (r0a_dab1_7oh_b[3] & shr_s0_pad[gpy + 12])) |
			r0a_ge16_sub_b;
	end
	endgenerate
	assign py_b_s1b[27] = r0a_sub_op_b;
	wire unused_shr_pad = &{1'b0, shr_s0_pad[50:27]};

	/* Stage 1B -> Stage 2 Pipeline Registers (directly feeding u_sum) */
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:0] r0b_px;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [27:0] r0b_py_b;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r0b_sub_op_ci;
	(* srl_style = "register" *) reg [7:0]  r0b_ex, r0b_ex_m1, r0b_ex_p1, r0b_ex_p2;
	(* srl_style = "register" *) reg        r0b_sx, r0b_sp, r0b_is_close;
	(* srl_style = "register" *) reg [31:0] r0b_spv;

	/* =========================================================================
	 * STAGE 3 (S2): Dedicated 28-bit Mantissa Adder u_sum
	 *               (Pre-LUTs = 1, Post-LUTs = 0 -> registered directly into r0c_sum)
	 * ========================================================================= */
	wire [27:0] sum_s2;
	wire sum_co_unused;
	am_add #(.W(28)) u_sum (
		.a({1'b0, r0b_px}),
		.b(r0b_py_b),
		.ci(r0b_sub_op_ci),
		.s(sum_s2),
		.co(sum_co_unused)
	);

	/* Replicated r0c_sum registers so Post-LUTs = 0 and Stage 4 fanout <= 12 */
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [27:0] r0c_sum_far;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:0] r0c_sum_cl;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0]  r0c_sum_top_a, r0c_sum_top_b, r0c_sum_top_c;
	(* srl_style = "register" *) reg [7:0]  r0c_ex, r0c_ex_m1, r0c_ex_p1, r0c_ex_p2;
	(* srl_style = "register" *) reg        r0c_sx, r0c_sp, r0c_is_close;
	(* srl_style = "register" *) reg [31:0] r0c_spv;

	/* =========================================================================
	 * STAGE 4 (S3): Pre-Normalization & Fast Nibble LZC from Registered r0c_sum
	 *               (0 CARRY4s, <= 3 LUT levels)
	 * ========================================================================= */
	(* keep = "true", dont_touch = "true" *) wire [23:0] s1_m_far_pre;
	assign s1_m_far_pre[11:0]  = r0c_sum_top_a[1] ? r0c_sum_far[15:4]  : r0c_sum_top_a[0] ? r0c_sum_far[14:3]  : r0c_sum_far[13:2];
	assign s1_m_far_pre[23:12] = r0c_sum_top_b[1] ? r0c_sum_far[27:16] : r0c_sum_top_b[0] ? r0c_sum_far[26:15] : r0c_sum_far[25:14];

	(* keep = "true", dont_touch = "true" *) wire s1_inc_c = r0c_sum_far[3] && (r0c_sum_far[2] || r0c_sum_far[1] || r0c_sum_far[0] || r0c_sum_far[4]);
	(* keep = "true", dont_touch = "true" *) wire s1_inc_n = r0c_sum_far[2] && (r0c_sum_far[1] || r0c_sum_far[0] || r0c_sum_far[3]);
	(* keep = "true", dont_touch = "true" *) wire s1_inc_u = r0c_sum_far[1] && (r0c_sum_far[0] || r0c_sum_far[2]);
	(* keep = "true", dont_touch = "true" *) wire s1_inc_far = r0c_sum_top_c[1] ? s1_inc_c : r0c_sum_top_c[0] ? s1_inc_n : s1_inc_u;

	wire [6:0] s1_mf_p3 = {
		&s1_m_far_pre[20:18], &s1_m_far_pre[17:15], &s1_m_far_pre[14:12], &s1_m_far_pre[11:9],
		&s1_m_far_pre[8:6],   &s1_m_far_pre[5:3],   &s1_m_far_pre[2:0]
	};
	wire s1_mf_p9_0 = &s1_mf_p3[2:0];
	wire s1_mf_p9_1 = &s1_mf_p3[5:3];
	wire s1_mf_p18  = s1_mf_p9_0 & s1_mf_p9_1;

	(* keep = "true", dont_touch = "true" *) wire s1_mf_g0 = &s1_m_far_pre[4:0];
	(* keep = "true", dont_touch = "true" *) wire s1_mf_g1 = &s1_m_far_pre[9:5];
	(* keep = "true", dont_touch = "true" *) wire s1_mf_g2 = &s1_m_far_pre[14:10];
	(* keep = "true", dont_touch = "true" *) wire s1_mf_g3 = &s1_m_far_pre[19:15];
	(* keep = "true", dont_touch = "true" *) wire s1_mf_g4 = &s1_m_far_pre[23:20];
	(* keep = "true", dont_touch = "true" *) wire s1_m_far_ov = s1_inc_far & s1_mf_g0 & s1_mf_g1 & s1_mf_g2 & s1_mf_g3 & s1_mf_g4;

	(* keep = "true", dont_touch = "true" *) wire [7:0] s1_e_far_no_ov = r0c_sum_top_c[1] ? r0c_ex_p1 : r0c_sum_top_c[0] ? r0c_ex : r0c_ex_m1;
	(* keep = "true", dont_touch = "true" *) wire [7:0] s1_e_far_ov    = r0c_sum_top_c[1] ? r0c_ex_p2 : r0c_sum_top_c[0] ? r0c_ex_p1 : r0c_ex;
	(* keep = "true", dont_touch = "true" *) wire s1_ef_no_hi = &s1_e_far_no_ov[7:4];
	(* keep = "true", dont_touch = "true" *) wire s1_ef_ov_hi = &s1_e_far_ov[7:4];
	(* keep = "true", dont_touch = "true" *) wire s1_ov_no_ov   = s1_ef_no_hi & (&s1_e_far_no_ov[3:0]);
	(* keep = "true", dont_touch = "true" *) wire s1_ov_with_ov = s1_ef_ov_hi & (&s1_e_far_ov[3:0]);

	wire [27:0] s1_sum_ext = {r0c_sum_cl[26:0], 1'b0};
	(* keep = "true", dont_touch = "true" *) wire [6:0] s1_g_act;
	(* keep = "true", dont_touch = "true" *) wire [1:0] s1_intra [0:6];

	genvar g_s1_cl;
	generate
	for (g_s1_cl = 0; g_s1_cl < 7; g_s1_cl = g_s1_cl + 1) begin : g_s1_cl_nib
		assign s1_g_act[g_s1_cl] = |s1_sum_ext[4 * g_s1_cl +: 4];
		assign s1_intra[g_s1_cl] = s1_sum_ext[4 * g_s1_cl + 3] ? 2'd0 :
		                           s1_sum_ext[4 * g_s1_cl + 2] ? 2'd1 :
		                           s1_sum_ext[4 * g_s1_cl + 1] ? 2'd2 : 2'd3;
	end
	endgenerate

	wire [6:0] s1_g_act_grp, s1_g_act_oh, s1_g_act_in;
`ifdef SYNTH_KINTEX7
	genvar g_ga;
	generate
	for (g_ga = 0; g_ga < 7; g_ga = g_ga + 1) begin : g_ga_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_gag (.I0(s1_g_act[g_ga]), .O(s1_g_act_grp[g_ga]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_gao (.I0(s1_g_act[g_ga]), .O(s1_g_act_oh[g_ga]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_gai (.I0(s1_g_act[g_ga]), .O(s1_g_act_in[g_ga]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_ga;
	generate
	for (g_ga = 0; g_ga < 7; g_ga = g_ga + 1) begin : g_ga_buf
		assign s1_g_act_grp[g_ga] = s1_g_act[g_ga];
		assign s1_g_act_oh[g_ga] = s1_g_act[g_ga];
		assign s1_g_act_in[g_ga] = s1_g_act[g_ga];
	end
	endgenerate
`else
	assign s1_g_act_grp = s1_g_act;
	assign s1_g_act_oh  = s1_g_act;
	assign s1_g_act_in  = s1_g_act;
`endif

	(* keep = "true", dont_touch = "true" *) wire [2:0] s1_grp =
		s1_g_act_grp[6] ? 3'd6 :
		s1_g_act_grp[5] ? 3'd5 :
		s1_g_act_grp[4] ? 3'd4 :
		s1_g_act_grp[3] ? 3'd3 :
		s1_g_act_grp[2] ? 3'd2 :
		s1_g_act_grp[1] ? 3'd1 : 3'd0;
	(* keep = "true" *) wire [6:0] s1_grp_oh_lo;
	assign s1_grp_oh_lo[6] =  s1_g_act_grp[6];
	assign s1_grp_oh_lo[5] = !s1_g_act_grp[6] &&  s1_g_act_grp[5];
	assign s1_grp_oh_lo[4] = !s1_g_act_grp[6] && !s1_g_act_grp[5] &&  s1_g_act_grp[4];
	assign s1_grp_oh_lo[3] = !(s1_g_act_grp[6] || s1_g_act_grp[5] || s1_g_act_grp[4]) && s1_g_act_grp[3];
	assign s1_grp_oh_lo[2] = !(s1_g_act_grp[6] || s1_g_act_grp[5] || s1_g_act_grp[4] || s1_g_act_grp[3]) && s1_g_act_grp[2];
	assign s1_grp_oh_lo[1] = !(s1_g_act_grp[6] || s1_g_act_grp[5] || s1_g_act_grp[4] || s1_g_act_grp[3] || s1_g_act_grp[2]) && s1_g_act_grp[1];
	assign s1_grp_oh_lo[0] = !(s1_g_act_grp[6] || s1_g_act_grp[5] || s1_g_act_grp[4] || s1_g_act_grp[3] || s1_g_act_grp[2] || s1_g_act_grp[1]);

	(* keep = "true" *) wire [6:0] s1_grp_oh_mid;
	assign s1_grp_oh_mid[6] =  s1_g_act_oh[6];
	assign s1_grp_oh_mid[5] = !s1_g_act_oh[6] &&  s1_g_act_oh[5];
	assign s1_grp_oh_mid[4] = !s1_g_act_oh[6] && !s1_g_act_oh[5] &&  s1_g_act_oh[4];
	assign s1_grp_oh_mid[3] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4]) && s1_g_act_oh[3];
	assign s1_grp_oh_mid[2] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3]) && s1_g_act_oh[2];
	assign s1_grp_oh_mid[1] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3] || s1_g_act_oh[2]) && s1_g_act_oh[1];
	assign s1_grp_oh_mid[0] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3] || s1_g_act_oh[2] || s1_g_act_oh[1]);

	(* keep = "true" *) wire [6:0] s1_grp_oh_hi;
	assign s1_grp_oh_hi[6] =  s1_g_act_oh[6];
	assign s1_grp_oh_hi[5] = !s1_g_act_oh[6] &&  s1_g_act_oh[5];
	assign s1_grp_oh_hi[4] = !s1_g_act_oh[6] && !s1_g_act_oh[5] &&  s1_g_act_oh[4];
	assign s1_grp_oh_hi[3] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4]) && s1_g_act_oh[3];
	assign s1_grp_oh_hi[2] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3]) && s1_g_act_oh[2];
	assign s1_grp_oh_hi[1] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3] || s1_g_act_oh[2]) && s1_g_act_oh[1];
	assign s1_grp_oh_hi[0] = !(s1_g_act_oh[6] || s1_g_act_oh[5] || s1_g_act_oh[4] || s1_g_act_oh[3] || s1_g_act_oh[2] || s1_g_act_oh[1]);

	wire [13:1] s1_coarse_lo =
		(s1_grp_oh_lo[6] ? s1_sum_ext[13:1]        : 13'd0) |
		(s1_grp_oh_lo[5] ? {s1_sum_ext[9:0], 3'b0} : 13'd0) |
		(s1_grp_oh_lo[4] ? {s1_sum_ext[5:0], 7'b0} : 13'd0) |
		(s1_grp_oh_lo[3] ? {s1_sum_ext[1:0], 11'b0}: 13'd0);
	wire [20:11] s1_coarse_mid =
		(s1_grp_oh_mid[6] ? s1_sum_ext[20:11]        : 10'd0) |
		(s1_grp_oh_mid[5] ? s1_sum_ext[16:7]         : 10'd0) |
		(s1_grp_oh_mid[4] ? s1_sum_ext[12:3]         : 10'd0) |
		(s1_grp_oh_mid[3] ? {s1_sum_ext[8:0], 1'b0}  : 10'd0) |
		(s1_grp_oh_mid[2] ? {s1_sum_ext[4:0], 5'b0}  : 10'd0) |
		(s1_grp_oh_mid[1] ? {s1_sum_ext[0],   9'b0}  : 10'd0);
	wire [26:18] s1_coarse_hi =
		(s1_grp_oh_hi[6] ? s1_sum_ext[26:18]        : 9'd0) |
		(s1_grp_oh_hi[5] ? s1_sum_ext[22:14]        : 9'd0) |
		(s1_grp_oh_hi[4] ? s1_sum_ext[18:10]        : 9'd0) |
		(s1_grp_oh_hi[3] ? s1_sum_ext[14:6]         : 9'd0) |
		(s1_grp_oh_hi[2] ? s1_sum_ext[10:2]         : 9'd0) |
		(s1_grp_oh_hi[1] ? {s1_sum_ext[6:0], 2'b0}  : 9'd0) |
		(s1_grp_oh_hi[0] ? {s1_sum_ext[2:0], 6'b0}  : 9'd0);

	(* keep = "true", dont_touch = "true" *) wire s1_hi_act  = s1_g_act_in[6] | s1_g_act_in[5] | s1_g_act_in[4];
	(* keep = "true", dont_touch = "true" *) wire s1_mid_act = s1_g_act_in[3] | s1_g_act_in[2];
	(* keep = "true", dont_touch = "true" *) wire [1:0] s1_intra_hi  = s1_g_act_in[6] ? s1_intra[6] : (s1_g_act_in[5] ? s1_intra[5] : s1_intra[4]);
	(* keep = "true", dont_touch = "true" *) wire [1:0] s1_intra_mid = s1_g_act_in[3] ? s1_intra[3] : s1_intra[2];
	(* keep = "true", dont_touch = "true" *) wire [1:0] s1_intra_bot = s1_g_act_in[1] ? s1_intra[1] : s1_intra[0];
	(* keep = "true", dont_touch = "true" *) wire [1:0] s1_active_intra = s1_hi_act ? s1_intra_hi : (s1_mid_act ? s1_intra_mid : s1_intra_bot);
	wire [3:0] s1_intra_oh = {
		(s1_active_intra == 2'd3), (s1_active_intra == 2'd2),
		(s1_active_intra == 2'd1), (s1_active_intra == 2'd0)
	};

	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [13:1] r1_coarse_lo;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [20:11] r1_coarse_mid;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [26:18] r1_coarse_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [23:0] r1_m_far_pre;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [6:0]  r1_mf_p3;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_mf_p9_0, r1_mf_p18;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_inc_far_a;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_mf_g0, r1_mf_g1, r1_mf_g2, r1_mf_g3;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_m_far_ov_lo, r1_m_far_ov_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [7:0]  r1_e_far_no_ov, r1_e_far_ov;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_ov_no_ov, r1_ov_with_ov;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [2:0]  r1_w_hi, r1_w_hi_m1;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [3:0]  r1_intra_lo_oh, r1_intra_mid_oh, r1_intra_hi_oh;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0]  r1_intra_exp;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_ex0;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_sx_lo, r1_sx_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_sp_lo, r1_sp_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg        r1_is_close_lo, r1_is_close_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [15:0] r1_spv_lo, r1_spv_hi;

	/* =========================================================================
	 * STAGE 5 (S4): Dual-Path Close Shift + Pure-LUT Far Increment + Output Reg
	 *               (0 CARRY4s, <= 10 cmos3)
	 * ========================================================================= */
	wire [13:4] fine_lo =
		(r1_intra_lo_oh[0] ? r1_coarse_lo[13:4] : 10'd0) |
		(r1_intra_lo_oh[1] ? r1_coarse_lo[12:3] : 10'd0) |
		(r1_intra_lo_oh[2] ? r1_coarse_lo[11:2] : 10'd0) |
		(r1_intra_lo_oh[3] ? r1_coarse_lo[10:1] : 10'd0);

	wire [20:14] fine_mid =
		(r1_intra_mid_oh[0] ? r1_coarse_mid[20:14] : 7'd0) |
		(r1_intra_mid_oh[1] ? r1_coarse_mid[19:13] : 7'd0) |
		(r1_intra_mid_oh[2] ? r1_coarse_mid[18:12] : 7'd0) |
		(r1_intra_mid_oh[3] ? r1_coarse_mid[17:11] : 7'd0);

	wire [26:21] fine_hi =
		(r1_intra_hi_oh[0] ? r1_coarse_hi[26:21] : 6'd0) |
		(r1_intra_hi_oh[1] ? r1_coarse_hi[25:20] : 6'd0) |
		(r1_intra_hi_oh[2] ? r1_coarse_hi[24:19] : 6'd0) |
		(r1_intra_hi_oh[3] ? r1_coarse_hi[23:18] : 6'd0);

	wire [22:0] m_close = {fine_hi, fine_mid, fine_lo};

	wire [1:0] active_intra_exp = r1_intra_exp;
	wire [1:0] w_lo_close = ~active_intra_exp;
	wire [1:0] w_lo_close_m1 = {w_lo_close[1] ^ (!w_lo_close[0]), ~w_lo_close[0]};
	wire [1:0] w_res_lo = r1_ex0 ? w_lo_close : w_lo_close_m1;

	wire borrow = (active_intra_exp == 2'b11) && (!r1_ex0);
	/* w_hi = grp_exp + 1, w_hi_m1 = grp_exp pre-computed in Stage 4 */
	wire [2:0] w_res_hi = borrow ? r1_w_hi_m1 : r1_w_hi;
	wire [7:0] e_close = {3'b011, w_res_hi, w_res_lo};

	/* Pre-computed p3/p9/p18 +1 incrementer for m_far (<= 6 cmos3) */
	wire [22:0] mf_a = r1_m_far_pre[22:0];
	wire mf_c0 = r1_inc_far_a;
	wire [22:0] mf_cy;
	assign mf_cy[0]  = mf_c0;
	assign mf_cy[1]  = mf_c0 & mf_a[0];
	assign mf_cy[2]  = mf_c0 & mf_a[0] & mf_a[1];
	assign mf_cy[3]  = mf_c0 & r1_mf_p3[0];
	assign mf_cy[4]  = mf_c0 & r1_mf_p3[0] & mf_a[3];
	assign mf_cy[5]  = mf_c0 & r1_mf_p3[0] & mf_a[3] & mf_a[4];
	assign mf_cy[6]  = mf_c0 & r1_mf_p3[0] & r1_mf_p3[1];
	assign mf_cy[7]  = mf_c0 & r1_mf_p3[0] & r1_mf_p3[1] & mf_a[6];
	assign mf_cy[8]  = mf_c0 & r1_mf_p3[0] & r1_mf_p3[1] & mf_a[6] & mf_a[7];
	assign mf_cy[9]  = mf_c0 & r1_mf_p9_0;
	assign mf_cy[10] = mf_c0 & r1_mf_p9_0 & mf_a[9];
	assign mf_cy[11] = mf_c0 & r1_mf_p9_0 & mf_a[9] & mf_a[10];
	assign mf_cy[12] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3];
	assign mf_cy[13] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3] & mf_a[12];
	assign mf_cy[14] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3] & mf_a[12] & mf_a[13];
	assign mf_cy[15] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3] & r1_mf_p3[4];
	assign mf_cy[16] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3] & r1_mf_p3[4] & mf_a[15];
	assign mf_cy[17] = mf_c0 & r1_mf_p9_0 & r1_mf_p3[3] & r1_mf_p3[4] & mf_a[15] & mf_a[16];
	assign mf_cy[18] = mf_c0 & r1_mf_p18;
	assign mf_cy[19] = mf_c0 & r1_mf_p18 & mf_a[18];
	assign mf_cy[20] = mf_c0 & r1_mf_p18 & mf_a[18] & mf_a[19];
	assign mf_cy[21] = mf_c0 & r1_mf_p18 & r1_mf_p3[6];
	assign mf_cy[22] = mf_c0 & r1_mf_p18 & r1_mf_p3[6] & mf_a[21];
	wire [22:0] m_far = mf_a ^ mf_cy;
	wire mfar_co_unused = mf_cy[22] & mf_a[22] & r1_mf_g0 & r1_mf_g1 & r1_mf_g2 & r1_mf_g3 & r1_mf_p3[2] & r1_mf_p3[5];

	wire [7:0] e_far = r1_m_far_ov_hi ? r1_e_far_ov : r1_e_far_no_ov;
	wire is_ov_far_lo = r1_m_far_ov_lo ? r1_ov_with_ov : r1_ov_no_ov;
	wire is_ov_far_hi = r1_m_far_ov_hi ? r1_ov_with_ov : r1_ov_no_ov;

	wire [15:0] norm_far_lo = is_ov_far_lo ? 16'd0 : m_far[15:0];
	wire [15:0] norm_far_hi = is_ov_far_hi ? {r1_sx_hi, 8'hFF, 7'd0} : {r1_sx_hi, e_far, m_far[22:16]};

	wire [15:0] norm_close_lo = m_close[15:0];
	wire [15:0] norm_close_hi = {r1_sx_hi, e_close, m_close[22:16]};

	wire [15:0] res_lo = r1_is_close_lo ? norm_close_lo : norm_far_lo;
	wire [15:0] res_hi = r1_is_close_hi ? norm_close_hi : norm_far_hi;

	wire [15:0] o_r_lo = r1_sp_lo ? r1_spv_lo : res_lo;
	wire [15:0] o_r_hi = r1_sp_hi ? r1_spv_hi : res_hi;

	reg [15:0] r2_out_lo;
	reg [15:0] r2_out_hi;

	always @(posedge clk or posedge rst_sync_lo) begin
		if (rst_sync_lo) begin
			r2_out_lo <= 16'd0;
		end else begin
			if (en_s4[0])
				r2_out_lo[7:0]  <= o_r_lo[7:0];
			if (en_s4[1])
				r2_out_lo[15:8] <= o_r_lo[15:8];
		end
	end

	always @(posedge clk or posedge rst_sync_hi) begin
		if (rst_sync_hi) begin
			r2_out_hi <= 16'd0;
		end else begin
			if (en_s4[2])
				r2_out_hi[7:0]  <= o_r_hi[7:0];
			if (en_s4[3])
				r2_out_hi[15:8] <= o_r_hi[15:8];
		end
	end

	always @(posedge clk) begin
		if (en_s1a[0]) begin
			r0a_dab0_oh_a0 <= {
				(dab_s1a[1:0] == 2'd3), (dab_s1a[1:0] == 2'd2),
				(dab_s1a[1:0] == 2'd1), (dab_s1a[1:0] == 2'd0)
			};
			r0a_dab0_oh_a1 <= {
				(dab_s1a[1:0] == 2'd3), (dab_s1a[1:0] == 2'd2),
				(dab_s1a[1:0] == 2'd1), (dab_s1a[1:0] == 2'd0)
			};
			r0a_dab0_oh_b0 <= {
				(dab_s1a[1:0] == 2'd3), (dab_s1a[1:0] == 2'd2),
				(dab_s1a[1:0] == 2'd1), (dab_s1a[1:0] == 2'd0)
			};
			r0a_dab0_oh_b1 <= {
				(dab_s1a[1:0] == 2'd3), (dab_s1a[1:0] == 2'd2),
				(dab_s1a[1:0] == 2'd1), (dab_s1a[1:0] == 2'd0)
			};
			r0a_dab1_7oh_a0 <= {
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd6),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd5),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd4),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd3),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd2),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd1),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd0)
			};
			r0a_dab1_7oh_a1 <= {
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd6),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd5),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd4),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd3),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd2),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd1),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd0)
			};
			r0a_dab1_7oh_b <= {
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd3),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd2),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd1),
				(!dab_ge27_s1a) && (dab_s1a[4:2] == 3'd0)
			};
			r0a_ge27_sub_a0 <= dab_ge27_s1a && sub_op_s1a;
			r0a_ge27_sub_a1 <= dab_ge27_s1a && sub_op_s1a;
			r0a_ge16_sub_b  <= (dab_ge27_s1a || dab_s1a[4]) && sub_op_s1a;
			r0a_dab_stk_lo <= dab_s1a[1:0];
			r0a_stk_gt     <= stk_gt_s1a;
			r0a_stk_eq     <= stk_eq_s1a;
		end
		if (en_s1a[1]) begin
			r0a_dab_ge27_a  <= dab_ge27_s1a;
			r0a_sub_op_a    <= sub_op_s1a;
			r0a_sub_op_b    <= sub_op_s1a;
			r0a_sub_op_ci   <= sub_op_s1a;
			r0a_sx          <= sx_s1a;
			r0a_sp          <= sp_s1a;
			r0a_is_close    <= is_close_s1a;
		end
		if (en_s1a[2]) begin
			r0a_shi[13:0]   <= shi_s1a[13:0];
			r0a_shi_x[13:0] <= shi_x_s1a[13:0];
		end
		if (en_s1a[3])
			r0a_px[13:0]   <= px_s1a[13:0];
		if (en_s1a[4]) begin
			r0a_shi[26:14]   <= shi_s1a[26:14];
			r0a_shi_x[26:14] <= shi_x_s1a[26:14];
		end
		if (en_s1a[5])
			r0a_px[26:14]  <= px_s1a[26:14];
		if (en_s1a[6]) begin
			r0a_ex         <= ex_s1a;
			r0a_ex_m1      <= ex_m1_s1a;
		end
		if (en_s1a[7]) begin
			r0a_ex_p1      <= ex_p1_s1a;
			r0a_ex_p2      <= ex_p2_s1a;
		end
		if (en_s1a[8])
			r0a_spv[15:0]  <= spv_s1a[15:0];
		if (en_s1a[9])
			r0a_spv[31:16] <= spv_s1a[31:16];

		if (en_s1b[0])
			r0b_px[13:0]    <= r0a_px[13:0];
		if (en_s1b[1])
			r0b_py_b[13:0]  <= py_b_s1b[13:0];
		if (en_s1b[2])
			r0b_px[26:14]   <= r0a_px[26:14];
		if (en_s1b[3])
			r0b_py_b[27:14] <= py_b_s1b[27:14];
		if (en_s1b[4]) begin
			r0b_sub_op_ci <= r0a_sub_op_ci;
			r0b_sx        <= r0a_sx;
			r0b_sp        <= r0a_sp;
			r0b_is_close  <= r0a_is_close;
		end
		if (en_s1b[5]) begin
			r0b_ex    <= r0a_ex;
			r0b_ex_m1 <= r0a_ex_m1;
		end
		if (en_s1b[6]) begin
			r0b_ex_p1 <= r0a_ex_p1;
			r0b_ex_p2 <= r0a_ex_p2;
		end
		if (en_s1b[7])
			r0b_spv[15:0]  <= r0a_spv[15:0];
		if (en_s1b[8])
			r0b_spv[31:16] <= r0a_spv[31:16];

		if (en_s2[0])
			r0c_sum_far[13:0] <= sum_s2[13:0];
		if (en_s2[1])
			r0c_sum_far[27:14] <= sum_s2[27:14];
		if (en_s2[2])
			r0c_sum_cl[13:0] <= sum_s2[13:0];
		if (en_s2[3])
			r0c_sum_cl[26:14] <= sum_s2[26:14];
		if (en_s2[4]) begin
			r0c_sum_top_a <= sum_s2[27:26];
			r0c_sum_top_b <= sum_s2[27:26];
			r0c_sum_top_c <= sum_s2[27:26];
			r0c_sx        <= r0b_sx;
			r0c_sp        <= r0b_sp;
			r0c_is_close  <= r0b_is_close;
		end
		if (en_s2[5]) begin
			r0c_ex    <= r0b_ex;
			r0c_ex_m1 <= r0b_ex_m1;
		end
		if (en_s2[6]) begin
			r0c_ex_p1 <= r0b_ex_p1;
			r0c_ex_p2 <= r0b_ex_p2;
		end
		if (en_s2[7]) begin
			r0c_spv <= r0b_spv;
		end

		if (en_s3[0])
			r1_coarse_lo <= s1_coarse_lo;
		if (en_s3[1]) begin
			r1_coarse_mid <= s1_coarse_mid;
			r1_coarse_hi  <= s1_coarse_hi;
		end
		if (en_s3[2])
			r1_m_far_pre[11:0] <= s1_m_far_pre[11:0];
		if (en_s3[3]) begin
			r1_m_far_pre[23:12] <= s1_m_far_pre[23:12];
			r1_mf_p3            <= s1_mf_p3;
			r1_mf_p9_0          <= s1_mf_p9_0;
			r1_mf_p18           <= s1_mf_p18;
			r1_inc_far_a        <= s1_inc_far;
			r1_mf_g0            <= s1_mf_g0;
			r1_mf_g1            <= s1_mf_g1;
			r1_mf_g2            <= s1_mf_g2;
			r1_mf_g3            <= s1_mf_g3;
		end
		if (en_s3[4]) begin
			r1_m_far_ov_lo <= s1_m_far_ov;
			r1_m_far_ov_hi <= s1_m_far_ov;
			r1_ov_no_ov    <= s1_ov_no_ov;
			r1_ov_with_ov  <= s1_ov_with_ov;
			r1_ex0         <= r0c_ex[0];
		end
		if (en_s3[5]) begin
			r1_e_far_no_ov <= s1_e_far_no_ov;
			r1_e_far_ov    <= s1_e_far_ov;
		end
		if (en_s3[6]) begin
			r1_w_hi_m1 <= s1_grp;
			r1_w_hi    <= {s1_grp[2] ^ (s1_grp[1] & s1_grp[0]), s1_grp[1] ^ s1_grp[0], ~s1_grp[0]};
		end
		if (en_s3[7]) begin
			r1_intra_lo_oh  <= s1_intra_oh;
			r1_intra_mid_oh <= s1_intra_oh;
			r1_intra_hi_oh  <= s1_intra_oh;
			r1_intra_exp    <= s1_active_intra;
		end
		if (en_s3[8]) begin
			r1_sx_lo       <= r0c_sx;
			r1_sx_hi       <= r0c_sx;
			r1_sp_lo       <= r0c_sp;
			r1_sp_hi       <= r0c_sp;
			r1_is_close_lo <= r0c_is_close;
			r1_is_close_hi <= r0c_is_close;
		end
		if (en_s3[9]) begin
			r1_spv_lo <= r0c_spv[15:0];
			r1_spv_hi <= r0c_spv[31:16];
		end
	end

	assign o_r = {r2_out_hi, r2_out_lo};

	wire unused_ok = &{1'b0, sum_co_unused, r1_m_far_pre[23], r1_sx_lo, en_s1a[11:10], en_s1b[9:9], en_s3[13:10],
		dab1_bo_unused, dab2_bo_unused, eam1_bo_unused, eap1_co_unused, eap2_co_unused, mfar_co_unused};
endmodule

`default_nettype wire
