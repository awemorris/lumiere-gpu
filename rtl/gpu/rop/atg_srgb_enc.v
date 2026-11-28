/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_srgb_enc: Fast 4-stage pipelined fp32 to sRGB 8-bit unorm encoder (LAT = 4).
 *   - Stage 1: Pure-LUT constant comparisons b >= TH_HI[0..16] into r1_c_hi[14:0] (0 CARRY4s, 3 LUT levels).
 *   - Stage 2: Thermometer decode r1_c_hi[14:0] -> r2_hi_lo[0:14], r2_hi_hi[0:14] (0 CARRY4s, 2 LUT levels).
 *   - Stage 3: Pure-LUT 2-bit-slice fused ROM lookup & comparison tree against tl_0..tl_14 (0 CARRY4s, 3 LUT levels).
 *   - Stage 4: Final comparison combine, lo[3:0] thermometer decode, and clamp mux (0 CARRY4s, 3 LUT levels).
 * 100% bit-exact with IEEE 754 float32 sRGB EOTF thresholds in atg_srgb.vh.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_srgb_enc (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] b_a,
	input wire [31:0] b_b,
	input wire [31:0] b_c,
	output wire [7:0] srgb
);
	(* keep = "true" *) wire rst_sync = !rst_n;

	wire [6:0]  en_s1_buf = {7{en}};
	wire [13:0] en_s2_buf = {14{en}};
	wire [3:0]  en_s3_buf = {4{en}};

	/* =========================================================================
	 * STAGE 1: Constant Threshold Comparisons (0 CARRY4s, 3 LUT levels)
	 * ========================================================================= */
	localparam [31*17-1:0] TH_S1 = {
		31'h3F7EDC0E, /* 16: thr[254] (max) */
		31'h391F22B4, /* 15: thr[0]   (min) */
		31'h3F5E0486, /* 14 */
		31'h3F3DDCE5, /* 13 */
		31'h3F209A3C, /* 12 */
		31'h3F062827, /* 11 */
		31'h3EDCE2B2, /* 10 */
		31'h3EB2BF00, /* 9  */
		31'h3E8DB641, /* 8  */
		31'h3E5B2D9A, /* 7  */
		31'h3E2457B6, /* 6  */
		31'h3DECF395, /* 5  */
		31'h3DA23384, /* 4  */
		31'h3D4EC8D2, /* 3  */
		31'h3CED78B5, /* 2  */
		31'h3C668654, /* 1  */
		31'h3BA314BD  /* 0  */
	};

	(* keep = "true" *) wire [16:0] ge_012_s1, gt34_s1, eq34_s1;
	genvar gc1;
	generate
	for (gc1 = 0; gc1 < 17; gc1 = gc1 + 1) begin : g_cmp_s1
		localparam [30:0] C = TH_S1[31 * gc1 +: 31];
		wire [30:0] x = (gc1 < 6) ? b_a[30:0] : (gc1 < 12) ? b_b[30:0] : b_c[30:0];

		/* Level 1: 5 slices of 6 bits (1 LUT6 each, 0 CARRY4s) */
		/* verilator lint_off UNSIGNED */
		/* verilator lint_off CMPCONST */
		(* keep = "true" *) wire ge0 = (x[5:0]   >= C[5:0]);
		(* keep = "true" *) wire gt1 = (x[11:6]  >  C[11:6]);
		(* keep = "true" *) wire eq1 = (x[11:6]  == C[11:6]);
		(* keep = "true" *) wire gt2 = (x[17:12] >  C[17:12]);
		(* keep = "true" *) wire eq2 = (x[17:12] == C[17:12]);
		(* keep = "true" *) wire gt3 = (x[23:18] >  C[23:18]);
		(* keep = "true" *) wire eq3 = (x[23:18] == C[23:18]);
		(* keep = "true" *) wire gt4 = (x[29:24] >  C[29:24]);
		(* keep = "true" *) wire eq4 = (x[29:24] == C[29:24]);
		/* verilator lint_on CMPCONST */
		/* verilator lint_on UNSIGNED */

		/* Level 2: combine low 18 bits and high 13 bits (1 LUT5 each, 0 CARRY4s) */
		assign ge_012_s1[gc1] = gt2 | (eq2 & gt1) | (eq2 & eq1 & ge0);
		assign gt34_s1[gc1]   = x[30] | gt4 | (eq4 & gt3);
		assign eq34_s1[gc1]   = (~x[30]) & eq4 & eq3;
	end
	endgenerate

	(* keep = "true" *) wire [14:0] c_hi_s1;
	generate
	for (gc1 = 0; gc1 < 15; gc1 = gc1 + 1) begin : g_chi_s1
		assign c_hi_s1[gc1] = gt34_s1[gc1] | (eq34_s1[gc1] & ge_012_s1[gc1]);
	end
	endgenerate

	(* keep = "true" *) wire [3:0] b_mnz;
	assign b_mnz[0] = (b_c[5:0]   != 6'd0);
	assign b_mnz[1] = (b_c[11:6]  != 6'd0);
	assign b_mnz[2] = (b_c[17:12] != 6'd0);
	assign b_mnz[3] = (b_c[22:18] != 5'd0);
	(* keep = "true" *) wire is_nan_s1 = (b_c[30:23] == 8'hFF) && (|b_mnz);
	wire is_neg_s1 = b_c[31];
	wire cmp15_s1  = gt34_s1[15] | (eq34_s1[15] & ge_012_s1[15]);
	wire cmp16_s1  = gt34_s1[16] | (eq34_s1[16] & ge_012_s1[16]);
	wire is_zero_s1 = is_nan_s1 || is_neg_s1 || (!cmp15_s1);
	wire is_max_s1  = (!is_neg_s1) && cmp16_s1;

	/* Stage 1 -> Stage 2 Pipeline Registers (3 LUT levels from input FF) */
	(* keep = "true", dont_touch = "true" *) reg [14:0] r0_c_hi;
	(* keep = "true", dont_touch = "true" *) reg [2:0]  r0_c_hi7_rep;
	(* keep = "true", dont_touch = "true" *) reg [30:0] r0_b_rep [0:2];
	reg r0_is_zero, r0_is_max;

	/* =========================================================================
	 * STAGE 2: Thermometer Decode r0_c_hi -> r1_hi_lo / r1_hi_hi (2 LUT levels)
	 * ========================================================================= */
	(* keep = "true" *) wire hi0_3 = r0_c_hi[14] | (~r0_c_hi[13] & r0_c_hi[12]);
	(* keep = "true" *) wire hi0_2 = r0_c_hi[10] | (~r0_c_hi[9]  & r0_c_hi[8]);
	(* keep = "true" *) wire hi0_1 = r0_c_hi[6]  | (~r0_c_hi[5]  & r0_c_hi[4]);
	(* keep = "true" *) wire hi0_0 = r0_c_hi[2]  | (~r0_c_hi[1]  & r0_c_hi[0]);
	(* keep = "true" *) wire hi1_h = r0_c_hi[13] | (~r0_c_hi[11] & r0_c_hi[9]);
	(* keep = "true" *) wire hi1_l = r0_c_hi[5]  | (~r0_c_hi[3]  & r0_c_hi[1]);

	(* keep = "true", dont_touch = "true" *) wire [3:0] hi_w_a = {
		r0_c_hi7_rep[0],
		r0_c_hi7_rep[0] ? r0_c_hi[11] : r0_c_hi[3],
		r0_c_hi7_rep[0] ? hi1_h : hi1_l,
		r0_c_hi7_rep[0] ? (r0_c_hi[11] ? hi0_3 : hi0_2) : (r0_c_hi[3] ? hi0_1 : hi0_0)
	};
	(* keep = "true", dont_touch = "true" *) wire [3:0] hi_w_b = {
		r0_c_hi7_rep[1],
		r0_c_hi7_rep[1] ? r0_c_hi[11] : r0_c_hi[3],
		r0_c_hi7_rep[1] ? hi1_h : hi1_l,
		r0_c_hi7_rep[1] ? (r0_c_hi[11] ? hi0_3 : hi0_2) : (r0_c_hi[3] ? hi0_1 : hi0_0)
	};
	(* keep = "true", dont_touch = "true" *) wire [3:0] hi_w_c = {
		r0_c_hi7_rep[2],
		r0_c_hi7_rep[2] ? r0_c_hi[11] : r0_c_hi[3],
		r0_c_hi7_rep[2] ? hi1_h : hi1_l,
		r0_c_hi7_rep[2] ? (r0_c_hi[11] ? hi0_3 : hi0_2) : (r0_c_hi[3] ? hi0_1 : hi0_0)
	};

	/* Stage 2 -> Stage 3 Pipeline Registers (replicated scalars so Yosys keeps copies separate, fanout <= 15) */
	(* keep = "true", dont_touch = "true" *) reg [3:0]  r1_hi_out;
	(* keep = "true", dont_touch = "true" *) reg [59:0] r1_hi_lo_vec, r1_hi_hi_vec;
	(* keep = "true", dont_touch = "true" *) reg [92:0] r1_b_rep_vec;
	reg r1_is_zero, r1_is_max;
	integer i1;
	function [30:0] get_tl;
		input [3:0] k_idx;
		input [3:0] h;
		begin
			case ({k_idx, h})
			{4'd0, 4'd0}: get_tl = 31'h391F22B4; {4'd0, 4'd1}: get_tl = 31'h3BB0A731;
			{4'd0, 4'd2}: get_tl = 31'h3C72DD71; {4'd0, 4'd3}: get_tl = 31'h3CF6D72E;
			{4'd0, 4'd4}: get_tl = 31'h3D553E73; {4'd0, 4'd5}: get_tl = 31'h3DA66510;
			{4'd0, 4'd6}: get_tl = 31'h3DF22D50; {4'd0, 4'd7}: get_tl = 31'h3E278099;
			{4'd0, 4'd8}: get_tl = 31'h3E5EE9D4; {4'd0, 4'd9}: get_tl = 31'h3E8FE170;
			{4'd0, 4'd10}: get_tl = 31'h3EB53A71; {4'd0, 4'd11}: get_tl = 31'h3EDFB168;
			{4'd0, 4'd12}: get_tl = 31'h3F07BA92; {4'd0, 4'd13}: get_tl = 31'h3F225913;
			{4'd0, 4'd14}: get_tl = 31'h3F3FC975; {4'd0, 4'd15}: get_tl = 31'h3F60200E;

			{4'd1, 4'd0}: get_tl = 31'h39EEB40E; {4'd1, 4'd1}: get_tl = 31'h3BBEDCB7;
			{4'd1, 4'd2}: get_tl = 31'h3C7F950F; {4'd1, 4'd3}: get_tl = 31'h3D0035FC;
			{4'd1, 4'd4}: get_tl = 31'h3D5BD1D3; {4'd1, 4'd5}: get_tl = 31'h3DAAA6A0;
			{4'd1, 4'd6}: get_tl = 31'h3DF77817; {4'd1, 4'd7}: get_tl = 31'h3E2AB27D;
			{4'd1, 4'd8}: get_tl = 31'h3E62AF7E; {4'd1, 4'd9}: get_tl = 31'h3E92118B;
			{4'd1, 4'd10}: get_tl = 31'h3EB7BB00; {4'd1, 4'd11}: get_tl = 31'h3EE2856A;
			{4'd1, 4'd12}: get_tl = 31'h3F094FB9; {4'd1, 4'd13}: get_tl = 31'h3F241ABC;
			{4'd1, 4'd14}: get_tl = 31'h3F41B8EB; {4'd1, 4'd15}: get_tl = 31'h3F623E90;

			{4'd2, 4'd0}: get_tl = 31'h3A46EB61; {4'd2, 4'd1}: get_tl = 31'h3BCDB76D;
			{4'd2, 4'd2}: get_tl = 31'h3C865702; {4'd2, 4'd3}: get_tl = 31'h3D051BB4;
			{4'd2, 4'd4}: get_tl = 31'h3D628318; {4'd2, 4'd5}: get_tl = 31'h3DAEF847;
			{4'd2, 4'd6}: get_tl = 31'h3DFCD3FC; {4'd2, 4'd7}: get_tl = 31'h3E2DED68;
			{4'd2, 4'd8}: get_tl = 31'h3E667E9F; {4'd2, 4'd9}: get_tl = 31'h3E944696;
			{4'd2, 4'd10}: get_tl = 31'h3EBA40B1; {4'd2, 4'd11}: get_tl = 31'h3EE55EBD;
			{4'd2, 4'd12}: get_tl = 31'h3F0AE79F; {4'd2, 4'd13}: get_tl = 31'h3F25DF38;
			{4'd2, 4'd14}: get_tl = 31'h3F43AB48; {4'd2, 4'd15}: get_tl = 31'h3F64600C;

			{4'd3, 4'd0}: get_tl = 31'h3A8B3E5E; {4'd3, 4'd1}: get_tl = 31'h3BDD3967;
			{4'd3, 4'd2}: get_tl = 31'h3C8D148F; {4'd3, 4'd3}: get_tl = 31'h3D0A1CEC;
			{4'd3, 4'd4}: get_tl = 31'h3D69526A; {4'd3, 4'd5}: get_tl = 31'h3DB35A15;
			{4'd3, 4'd6}: get_tl = 31'h3E012087; {4'd3, 4'd7}: get_tl = 31'h3E313161;
			{4'd3, 4'd8}: get_tl = 31'h3E6A573E; {4'd3, 4'd9}: get_tl = 31'h3E968095;
			{4'd3, 4'd10}: get_tl = 31'h3EBCCB85; {4'd3, 4'd11}: get_tl = 31'h3EE83D63;
			{4'd3, 4'd12}: get_tl = 31'h3F0C8244; {4'd3, 4'd13}: get_tl = 31'h3F27A689;
			{4'd3, 4'd14}: get_tl = 31'h3F45A08F; {4'd3, 4'd15}: get_tl = 31'h3F668485;

			{4'd4, 4'd0}: get_tl = 31'h3AB3070B; {4'd4, 4'd1}: get_tl = 31'h3BED64AF;
			{4'd4, 4'd2}: get_tl = 31'h3C940396; {4'd4, 4'd3}: get_tl = 31'h3D0F39D0;
			{4'd4, 4'd4}: get_tl = 31'h3D703FEE; {4'd4, 4'd5}: get_tl = 31'h3DB7CC1B;
			{4'd4, 4'd6}: get_tl = 31'h3E03DFAE; {4'd4, 4'd7}: get_tl = 31'h3E347E70;
			{4'd4, 4'd8}: get_tl = 31'h3E6E3962; {4'd4, 4'd9}: get_tl = 31'h3E98BF89;
			{4'd4, 4'd10}: get_tl = 31'h3EBF5B81; {4'd4, 4'd11}: get_tl = 31'h3EEB215D;
			{4'd4, 4'd12}: get_tl = 31'h3F0E1FAA; {4'd4, 4'd13}: get_tl = 31'h3F2970AF;
			{4'd4, 4'd14}: get_tl = 31'h3F4798BF; {4'd4, 4'd15}: get_tl = 31'h3F68ABFB;

			{4'd5, 4'd0}: get_tl = 31'h3ADACFB8; {4'd5, 4'd1}: get_tl = 31'h3BFE3B46;
			{4'd5, 4'd2}: get_tl = 31'h3C9B247C; {4'd5, 4'd3}: get_tl = 31'h3D14728A;
			{4'd5, 4'd4}: get_tl = 31'h3D774BCA; {4'd5, 4'd5}: get_tl = 31'h3DBC4E6B;
			{4'd5, 4'd6}: get_tl = 31'h3E06A77B; {4'd5, 4'd7}: get_tl = 31'h3E37D49C;
			{4'd5, 4'd8}: get_tl = 31'h3E722511; {4'd5, 4'd9}: get_tl = 31'h3E9B0377;
			{4'd5, 4'd10}: get_tl = 31'h3EC1F0A7; {4'd5, 4'd11}: get_tl = 31'h3EEE0AB1;
			{4'd5, 4'd12}: get_tl = 31'h3F0FBFD2; {4'd5, 4'd13}: get_tl = 31'h3F2B3DAD;
			{4'd5, 4'd14}: get_tl = 31'h3F4993DB; {4'd5, 4'd15}: get_tl = 31'h3F6AD670;

			{4'd6, 4'd0}: get_tl = 31'h3B014C33; {4'd6, 4'd1}: get_tl = 31'h3C07DF91;
			{4'd6, 4'd2}: get_tl = 31'h3CA277A6; {4'd6, 4'd3}: get_tl = 31'h3D19C745;
			{4'd6, 4'd4}: get_tl = 31'h3D7E7624; {4'd6, 4'd5}: get_tl = 31'h3DC0E114;
			{4'd6, 4'd6}: get_tl = 31'h3E0977F6; {4'd6, 4'd7}: get_tl = 31'h3E3B33EC;
			{4'd6, 4'd8}: get_tl = 31'h3E761A52; {4'd6, 4'd9}: get_tl = 31'h3E9D4C62;
			{4'd6, 4'd10}: get_tl = 31'h3EC48AF9; {4'd6, 4'd11}: get_tl = 31'h3EF0F95F;
			{4'd6, 4'd12}: get_tl = 31'h3F1162BE; {4'd6, 4'd13}: get_tl = 31'h3F2D0D83;
			{4'd6, 4'd14}: get_tl = 31'h3F4B91E3; {4'd6, 4'd15}: get_tl = 31'h3F6D03E5;

			{4'd7, 4'd0}: get_tl = 31'h3B153089; {4'd7, 4'd1}: get_tl = 31'h3C10F91B;
			{4'd7, 4'd2}: get_tl = 31'h3CA9FD78; {4'd7, 4'd3}: get_tl = 31'h3D1F382C;
			{4'd7, 4'd4}: get_tl = 31'h3D82DF90; {4'd7, 4'd5}: get_tl = 31'h3DC58429;
			{4'd7, 4'd6}: get_tl = 31'h3E0C5126; {4'd7, 4'd7}: get_tl = 31'h3E3E9C67;
			{4'd7, 4'd8}: get_tl = 31'h3E7A192C; {4'd7, 4'd9}: get_tl = 31'h3E9F9A4C;
			{4'd7, 4'd10}: get_tl = 31'h3EC72A7C; {4'd7, 4'd11}: get_tl = 31'h3EF3ED6B;
			{4'd7, 4'd12}: get_tl = 31'h3F13086E; {4'd7, 4'd13}: get_tl = 31'h3F2EE032;
			{4'd7, 4'd14}: get_tl = 31'h3F4D92D8; {4'd7, 4'd15}: get_tl = 31'h3F6F345A;

			{4'd8, 4'd0}: get_tl = 31'h3B2914DF; {4'd8, 4'd1}: get_tl = 31'h3C1A6B32;
			{4'd8, 4'd2}: get_tl = 31'h3CB1B653; {4'd8, 4'd3}: get_tl = 31'h3D24C567;
			{4'd8, 4'd4}: get_tl = 31'h3D869372; {4'd8, 4'd5}: get_tl = 31'h3DCA37B9;
			{4'd8, 4'd6}: get_tl = 31'h3E0F3314; {4'd8, 4'd7}: get_tl = 31'h3E420E15;
			{4'd8, 4'd8}: get_tl = 31'h3E7E21A5; {4'd8, 4'd9}: get_tl = 31'h3EA1ED38;
			{4'd8, 4'd10}: get_tl = 31'h3EC9CF32; {4'd8, 4'd11}: get_tl = 31'h3EF6E6D8;
			{4'd8, 4'd12}: get_tl = 31'h3F14B0E4; {4'd8, 4'd13}: get_tl = 31'h3F30B5BD;
			{4'd8, 4'd14}: get_tl = 31'h3F4F96BD; {4'd8, 4'd15}: get_tl = 31'h3F7167D2;

			{4'd9, 4'd0}: get_tl = 31'h3B3CF936; {4'd9, 4'd1}: get_tl = 31'h3C2436C8;
			{4'd9, 4'd2}: get_tl = 31'h3CB9A298; {4'd9, 4'd3}: get_tl = 31'h3D2A6F22;
			{4'd9, 4'd4}: get_tl = 31'h3D8A56CB; {4'd9, 4'd5}: get_tl = 31'h3DCEFBD6;
			{4'd9, 4'd6}: get_tl = 31'h3E121DC5; {4'd9, 4'd7}: get_tl = 31'h3E4588FB;
			{4'd9, 4'd8}: get_tl = 31'h3E8119E2; {4'd9, 4'd9}: get_tl = 31'h3EA4452B;
			{4'd9, 4'd10}: get_tl = 31'h3ECC791E; {4'd9, 4'd11}: get_tl = 31'h3EF9E5A8;
			{4'd9, 4'd12}: get_tl = 31'h3F165C22; {4'd9, 4'd13}: get_tl = 31'h3F328E24;
			{4'd9, 4'd14}: get_tl = 31'h3F519D92; {4'd9, 4'd15}: get_tl = 31'h3F739E4D;

			{4'd10, 4'd0}: get_tl = 31'h3B50F2D1; {4'd10, 4'd1}: get_tl = 31'h3C2E5CC7;
			{4'd10, 4'd2}: get_tl = 31'h3CC1C2A9; {4'd10, 4'd3}: get_tl = 31'h3D303584;
			{4'd10, 4'd4}: get_tl = 31'h3D8E29AB; {4'd10, 4'd5}: get_tl = 31'h3DD3D08F;
			{4'd10, 4'd6}: get_tl = 31'h3E151143; {4'd10, 4'd7}: get_tl = 31'h3E490D22;
			{4'd10, 4'd8}: get_tl = 31'h3E8327C7; {4'd10, 4'd9}: get_tl = 31'h3EA6A226;
			{4'd10, 4'd10}: get_tl = 31'h3ECF2842; {4'd10, 4'd11}: get_tl = 31'h3EFCE9DE;
			{4'd10, 4'd12}: get_tl = 31'h3F180A29; {4'd10, 4'd13}: get_tl = 31'h3F346968;
			{4'd10, 4'd14}: get_tl = 31'h3F53A758; {4'd10, 4'd15}: get_tl = 31'h3F75D7CC;

			{4'd11, 4'd0}: get_tl = 31'h3B65FB9B; {4'd11, 4'd1}: get_tl = 31'h3C38DE1A;
			{4'd11, 4'd2}: get_tl = 31'h3CCA16E3; {4'd11, 4'd3}: get_tl = 31'h3D3618B7;
			{4'd11, 4'd4}: get_tl = 31'h3D920C27; {4'd11, 4'd5}: get_tl = 31'h3DD8B5F5;
			{4'd11, 4'd6}: get_tl = 31'h3E180D95; {4'd11, 4'd7}: get_tl = 31'h3E4C9A90;
			{4'd11, 4'd8}: get_tl = 31'h3E853A86; {4'd11, 4'd9}: get_tl = 31'h3EA9042E;
			{4'd11, 4'd10}: get_tl = 31'h3ED1DCA2; {4'd11, 4'd11}: get_tl = 31'h3EFFF37E;
			{4'd11, 4'd12}: get_tl = 31'h3F19BAFA; {4'd11, 4'd13}: get_tl = 31'h3F36478B;
			{4'd11, 4'd14}: get_tl = 31'h3F55B411; {4'd11, 4'd15}: get_tl = 31'h3F781451;

			{4'd12, 4'd0}: get_tl = 31'h3B7C3403; {4'd12, 4'd1}: get_tl = 31'h3C43BBA4;
			{4'd12, 4'd2}: get_tl = 31'h3CD29FA4; {4'd12, 4'd3}: get_tl = 31'h3D3C18E4;
			{4'd12, 4'd4}: get_tl = 31'h3D95FE4F; {4'd12, 4'd5}: get_tl = 31'h3DDDAC19;
			{4'd12, 4'd6}: get_tl = 31'h3E1B12C2; {4'd12, 4'd7}: get_tl = 31'h3E50314C;
			{4'd12, 4'd8}: get_tl = 31'h3E875222; {4'd12, 4'd9}: get_tl = 31'h3EAB6B44;
			{4'd12, 4'd10}: get_tl = 31'h3ED49641; {4'd12, 4'd11}: get_tl = 31'h3F018145;
			{4'd12, 4'd12}: get_tl = 31'h3F1B6E96; {4'd12, 4'd13}: get_tl = 31'h3F38288F;
			{4'd12, 4'd14}: get_tl = 31'h3F57C3BE; {4'd12, 4'd15}: get_tl = 31'h3F7A53DD;

			{4'd13, 4'd0}: get_tl = 31'h3B89D060; {4'd13, 4'd1}: get_tl = 31'h3C4EF648;
			{4'd13, 4'd2}: get_tl = 31'h3CDB5D4B; {4'd13, 4'd3}: get_tl = 31'h3D423632;
			{4'd13, 4'd4}: get_tl = 31'h3D9A0035; {4'd13, 4'd5}: get_tl = 31'h3DE2B30A;
			{4'd13, 4'd6}: get_tl = 31'h3E1E20D1; {4'd13, 4'd7}: get_tl = 31'h3E53D15D;
			{4'd13, 4'd8}: get_tl = 31'h3E896E9D; {4'd13, 4'd9}: get_tl = 31'h3EADD76D;
			{4'd13, 4'd10}: get_tl = 31'h3ED75521; {4'd13, 4'd11}: get_tl = 31'h3F030B82;
			{4'd13, 4'd12}: get_tl = 31'h3F1D24FF; {4'd13, 4'd13}: get_tl = 31'h3F3A0C73;
			{4'd13, 4'd14}: get_tl = 31'h3F59D65F; {4'd13, 4'd15}: get_tl = 31'h3F7C9671;

			{4'd14, 4'd0}: get_tl = 31'h3B962333; {4'd14, 4'd1}: get_tl = 31'h3C5A8EE4;
			{4'd14, 4'd2}: get_tl = 31'h3CE45032; {4'd14, 4'd3}: get_tl = 31'h3D4870CA;
			{4'd14, 4'd4}: get_tl = 31'h3D9E11EC; {4'd14, 4'd5}: get_tl = 31'h3DE7CAD9;
			{4'd14, 4'd6}: get_tl = 31'h3E2137CB; {4'd14, 4'd7}: get_tl = 31'h3E577ACA;
			{4'd14, 4'd8}: get_tl = 31'h3E8B8FFC; {4'd14, 4'd9}: get_tl = 31'h3EB048AA;
			{4'd14, 4'd10}: get_tl = 31'h3EDA1946; {4'd14, 4'd11}: get_tl = 31'h3F049877;
			{4'd14, 4'd12}: get_tl = 31'h3F1EDE36; {4'd14, 4'd13}: get_tl = 31'h3F3BF33A;
			{4'd14, 4'd14}: get_tl = 31'h3F5BEBF7; {4'd14, 4'd15}: get_tl = 31'h3F7EDC0E;
			default: get_tl = 31'd0;
			endcase
		end
	endfunction

	wire [14:0] s2_ge_lo8, s2_gt_mid8, s2_eq_mid8, s2_gt_hia, s2_eq_hia, s2_gt_hib, s2_eq_hib;
	genvar gk, gp, gg;
	generate
	for (gk = 0; gk < 15; gk = gk + 1) begin : g_cmp_s2
		localparam [3:0] K_IDX = gk[3:0];
		wire [30:0] b_k = r1_b_rep_vec[31 * (gk / 5) +: 31];
		wire [30:0] tl_lo = get_tl(K_IDX, r1_hi_lo_vec[4 * gk +: 4]);
		wire [30:0] tl_hi = get_tl(K_IDX, r1_hi_hi_vec[4 * gk +: 4]);
		wire [30:0] tl_k  = {tl_hi[30:16], tl_lo[15:0]};

		/* Level 1: 0-XOR 1-bit active-low compare (1 cmos3) + 4-stage AOI3/OAI3 tree (<= 9 cmos3 total) */
		wire [30:0] gtn1 = (~b_k) | tl_k;
		wire [30:0] gen1 = (~b_k) & tl_k;
		wire [14:0] gt2, ge2;
		assign ge2[0] = !(gen1[1] | (gtn1[1] & gen1[0]));
		assign gt2[0] = !(gen1[1] | (gtn1[1] & gtn1[0]));
		for (gp = 1; gp < 15; gp = gp + 1) begin : g_pair
			assign ge2[gp] = !(gen1[2 * gp + 1] | (gtn1[2 * gp + 1] & gen1[2 * gp]));
			assign gt2[gp] = !(gen1[2 * gp + 1] | (gtn1[2 * gp + 1] & gtn1[2 * gp]));
		end
		wire [14:1] eq0;
		for (gp = 1; gp < 15; gp = gp + 1) begin : g_eqpair
			assign eq0[gp] = (b_k[2 * gp +: 2] == tl_k[2 * gp +: 2]);
		end

		wire gtn_lo4_lo = !(gt2[1] | (ge2[1] & ge2[0]));
		wire gen_lo4_lo = gtn_lo4_lo;
		wire gtn_lo4_hi = !(gt2[3] | (ge2[3] & gt2[2]));
		wire gen_lo4_hi = !(gt2[3] | (ge2[3] & ge2[2]));
		wire ge_lo8_w   = !(gtn_lo4_hi & (gen_lo4_hi | gtn_lo4_lo));

		wire gtn_mid4_lo = !(gt2[5] | (ge2[5] & gt2[4]));
		wire gtn_mid4_hi = !(gt2[7] | (ge2[7] & gt2[6]));
		wire gen_mid4_hi = !(gt2[7] | (ge2[7] & ge2[6]));
		wire gt_mid8_w   = !(gtn_mid4_hi & (gen_mid4_hi | gtn_mid4_lo));
		wire eq_mid4_lo  = eq0[5] & eq0[4];
		wire eq_mid4_hi  = eq0[7] & eq0[6];
		wire eq_mid8_w   = eq_mid4_hi & eq_mid4_lo;

		wire gtn_hia_lo = !(gt2[9]  | (ge2[9]  & gt2[8]));
		wire gtn_hia_hi = !(gt2[11] | (ge2[11] & gt2[10]));
		wire gen_hia_hi = !(gt2[11] | (ge2[11] & ge2[10]));
		wire gt_hia_w   = !(gtn_hia_hi & (gen_hia_hi | gtn_hia_lo));
		wire eq_hia_lo  = eq0[9]  & eq0[8];
		wire eq_hia_hi  = eq0[11] & eq0[10];
		wire eq_hia_w   = eq_hia_hi & eq_hia_lo;

		wire gtn_hib_lo  = !(gt2[13] | (ge2[13] & gt2[12]));
		wire gt_hib_top  = b_k[30] | gt2[14];
		wire ge_hib_top  = b_k[30] | ge2[14];
		wire gtn_hib_top = !gt_hib_top;
		wire gen_hib_top = !ge_hib_top;
		wire gt_hib_w    = !(gtn_hib_top & (gen_hib_top | gtn_hib_lo));
		wire eq_hib_lo   = eq0[13] & eq0[12];
		wire eq_hib_top  = (~b_k[30]) & eq0[14];
		wire eq_hib_w    = eq_hib_top & eq_hib_lo;
		wire unused_gen_lo = gen_lo4_lo;

		assign s2_ge_lo8[gk]  = ge_lo8_w;
		assign s2_gt_mid8[gk] = gt_mid8_w;
		assign s2_eq_mid8[gk] = eq_mid8_w;
		assign s2_gt_hia[gk]  = gt_hia_w;
		assign s2_eq_hia[gk]  = eq_hia_w;
		assign s2_gt_hib[gk]  = gt_hib_w;
		assign s2_eq_hib[gk]  = eq_hib_w;
	end
	endgenerate

	/* Stage 2 -> Stage 3 Pipeline Registers */
	reg [14:0] r2_ge_lo8, r2_gt_mid8, r2_eq_mid8, r2_gt_hia, r2_eq_hia, r2_gt_hib, r2_eq_hib;
	reg [3:0]  r2_hi;
	reg        r2_is_zero, r2_is_max;

	/* =========================================================================
	 * STAGE 3: Final Compare Combine, Thermometer Decode & Output Register
	 *          (0 CARRY4s, <= 10 cmos3)
	 * ========================================================================= */
	wire [14:0] cmp_lo;
	generate
	for (gk = 0; gk < 15; gk = gk + 1) begin : g_cmp_s3
		wire ge_lo16 = r2_gt_mid8[gk] | (r2_eq_mid8[gk] & r2_ge_lo8[gk]);
		wire gt_hi15 = r2_gt_hib[gk]  | (r2_eq_hib[gk]  & r2_gt_hia[gk]);
		wire eq_hi15 = r2_eq_hib[gk]  & r2_eq_hia[gk];
		assign cmp_lo[gk] = gt_hi15 | (eq_hi15 & ge_lo16);
	end
	endgenerate

	wire lo1_top = cmp_lo[13] | (~cmp_lo[11] & cmp_lo[9]);
	wire lo1_bot = cmp_lo[5]  | (~cmp_lo[3]  & cmp_lo[1]);
	wire lo0_3   = cmp_lo[14] | (~cmp_lo[13] & cmp_lo[12]);
	wire lo0_2   = cmp_lo[10] | (~cmp_lo[9]  & cmp_lo[8]);
	wire lo0_1   = cmp_lo[6]  | (~cmp_lo[5]  & cmp_lo[4]);
	wire lo0_0   = cmp_lo[2]  | (~cmp_lo[1]  & cmp_lo[0]);

	wire [3:0] lo_s3;
	assign lo_s3[3] = cmp_lo[7];
	assign lo_s3[2] = cmp_lo[7] ? cmp_lo[11] : cmp_lo[3];
	assign lo_s3[1] = cmp_lo[7] ? lo1_top : lo1_bot;
	assign lo_s3[0] = cmp_lo[7] ? (cmp_lo[11] ? lo0_3 : lo0_2) : (cmp_lo[3] ? lo0_1 : lo0_0);

	wire [7:0] srgb_s3 = r2_is_zero ? 8'd0 : r2_is_max ? 8'd255 : {r2_hi, lo_s3};

	reg [7:0] r3_srgb;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r0_c_hi      <= 15'd0;
			r0_c_hi7_rep <= 3'd0;
			r0_b_rep[0]  <= 31'd0;
			r0_b_rep[1] <= 31'd0;
			r0_b_rep[2] <= 31'd0;
			r0_is_zero  <= 1'b0;
			r0_is_max   <= 1'b0;
			for (i1 = 0; i1 < 15; i1 = i1 + 1) begin
				r1_hi_lo_vec[4 * i1 +: 4] <= 4'd0;
				r1_hi_hi_vec[4 * i1 +: 4] <= 4'd0;
			end
			r1_hi_out    <= 4'd0;
			r1_b_rep_vec <= 93'd0;
			r1_is_zero  <= 1'b0;
			r1_is_max   <= 1'b0;
			r2_ge_lo8   <= 15'd0;
			r2_gt_mid8  <= 15'd0;
			r2_eq_mid8  <= 15'd0;
			r2_gt_hia   <= 15'd0;
			r2_eq_hia   <= 15'd0;
			r2_gt_hib   <= 15'd0;
			r2_eq_hib   <= 15'd0;
			r2_hi       <= 4'd0;
			r2_is_zero  <= 1'b0;
			r2_is_max   <= 1'b0;
			r3_srgb     <= 8'd0;
		end else begin
			/* Stage 1 -> Stage 2 */
			if (en_s1_buf[0]) begin
				r0_c_hi      <= c_hi_s1;
				r0_c_hi7_rep <= {3{c_hi_s1[7]}};
				r0_is_zero   <= is_zero_s1;
				r0_is_max    <= is_max_s1;
			end
			if (en_s1_buf[1])
				r0_b_rep[0][15:0]  <= b_a[15:0];
			if (en_s1_buf[2])
				r0_b_rep[0][30:16] <= b_a[30:16];
			if (en_s1_buf[3])
				r0_b_rep[1][15:0]  <= b_b[15:0];
			if (en_s1_buf[4])
				r0_b_rep[1][30:16] <= b_b[30:16];
			if (en_s1_buf[5])
				r0_b_rep[2][15:0]  <= b_c[15:0];
			if (en_s1_buf[6])
				r0_b_rep[2][30:16] <= b_c[30:16];

			/* Stage 2 -> Stage 3 */
			if (en_s2_buf[0]) begin
				for (i1 = 0; i1 < 4; i1 = i1 + 1)
					r1_hi_lo_vec[4 * i1 +: 4] <= hi_w_a;
			end
			if (en_s2_buf[1]) begin
				for (i1 = 4; i1 < 8; i1 = i1 + 1)
					r1_hi_lo_vec[4 * i1 +: 4] <= hi_w_a;
			end
			if (en_s2_buf[2]) begin
				for (i1 = 8; i1 < 12; i1 = i1 + 1)
					r1_hi_lo_vec[4 * i1 +: 4] <= hi_w_b;
			end
			if (en_s2_buf[3]) begin
				for (i1 = 12; i1 < 15; i1 = i1 + 1)
					r1_hi_lo_vec[4 * i1 +: 4] <= hi_w_b;
				r1_hi_out  <= hi_w_b;
			end
			if (en_s2_buf[4]) begin
				for (i1 = 0; i1 < 4; i1 = i1 + 1)
					r1_hi_hi_vec[4 * i1 +: 4] <= hi_w_c;
			end
			if (en_s2_buf[5]) begin
				for (i1 = 4; i1 < 8; i1 = i1 + 1)
					r1_hi_hi_vec[4 * i1 +: 4] <= hi_w_c;
			end
			if (en_s2_buf[6]) begin
				for (i1 = 8; i1 < 12; i1 = i1 + 1)
					r1_hi_hi_vec[4 * i1 +: 4] <= hi_w_c;
			end
			if (en_s2_buf[7]) begin
				for (i1 = 12; i1 < 15; i1 = i1 + 1)
					r1_hi_hi_vec[4 * i1 +: 4] <= hi_w_c;
				r1_is_zero <= r0_is_zero;
				r1_is_max  <= r0_is_max;
			end
			if (en_s2_buf[8])
				r1_b_rep_vec[0  +: 16] <= r0_b_rep[0][15:0];
			if (en_s2_buf[9])
				r1_b_rep_vec[16 +: 15] <= r0_b_rep[0][30:16];
			if (en_s2_buf[10])
				r1_b_rep_vec[31 +: 16] <= r0_b_rep[1][15:0];
			if (en_s2_buf[11])
				r1_b_rep_vec[47 +: 15] <= r0_b_rep[1][30:16];
			if (en_s2_buf[12])
				r1_b_rep_vec[62 +: 16] <= r0_b_rep[2][15:0];
			if (en_s2_buf[13])
				r1_b_rep_vec[78 +: 15] <= r0_b_rep[2][30:16];

			/* Stage 3 -> Stage 4 */
			if (en_s3_buf[0]) begin
				r2_ge_lo8  <= s2_ge_lo8;
				r2_gt_mid8 <= s2_gt_mid8;
				r2_eq_mid8 <= s2_eq_mid8;
			end
			if (en_s3_buf[1]) begin
				r2_gt_hia <= s2_gt_hia;
				r2_gt_hib <= s2_gt_hib;
			end
			if (en_s3_buf[2]) begin
				r2_eq_hia <= s2_eq_hia;
				r2_eq_hib <= s2_eq_hib;
			end
			if (en_s3_buf[3]) begin
				r2_hi      <= r1_hi_out;
				r2_is_zero <= r1_is_zero;
				r2_is_max  <= r1_is_max;
			end
			if (en)
				r3_srgb <= srgb_s3;
		end
	end

	assign srgb = r3_srgb;

	wire unused_b31 = &{1'b0, b_a[31], b_b[31]};
endmodule

`default_nettype wire
