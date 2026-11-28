/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_fpre: the per-lane front of atg_tu_filter (P1 → P5): the
 * compare reference Dref (the lane's tc_value word) as unorm24 (as
 * atg_fmt_depth_u24), unorm16 (as atg_fmt_f2unorm with 16 bits) or the fp32
 * order key (atg_fmt_depth_key), the texels' keys, VkCompareOp i_op of Dref
 * against each texel (as atg_depth_pass: D32F a NaN on either side passes
 * NOT_EQUAL and ALWAYS only); the bilinear weights of the four texels
 * ((256 - fu or fu)(256 - fv or fv); without bilinear 65536 for texel 0)
 * and the pass's (fw or 256 - fw)(fl or 256 - fl); the LOD query's
 * magnitudes |tc_value[15:0]|, |[31:16]|.
 *
 * Pipelined, no stall: the inputs in cycle 1 (atg_tu_filter's P1),
 * o_ok in cycle 4 (combinational from P4's registers), the weights and
 * the magnitudes in cycle 5 (registers).
 * See spec/gpu/tu/atg_tu_filter.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_fpre (
	input wire clk,
	input wire rst_n,
	input wire [31:0] i_vw,	/* Dref, or the query's fixed-point values */
	input wire [127:0] i_lt,	/* the lane's texels */
	input wire [31:0] i_wl,	/* the lane's {fw, fl, fv, fu} */
	input wire i_bil,
	input wire i_p17,
	input wire i_p18,
	input wire i_d32,	/* D32F (else D24, or D16) */
	input wire i_d16,
	input wire [2:0] i_op,
	output wire [3:0] o_ok,
	output wire [67:0] o_w,	/* texel t's weight in [17t +: 17] */
	output wire [16:0] o_wp,
	output wire [31:0] o_fa
);
	`include "atg_depth.vh"
	integer i, t;

	/* ---- P1 → P2: Dref's conversions (as atg_fmt_depth_u24, atg_fmt_f2unorm
	 * with 16 bits and atg_fmt_depth_key), the texels' keys, the weights ---- */
	wire [7:0] de = i_vw[30:23];
	(* keep = "true" *) wire dnan = de == 8'hFF && i_vw[22:0] != 23'd0;
	wire [47:0] p24, p16;
	wire [4:0] t24;
	wire [5:0] sam1;
	wire [32:0] dkey;
	wire p24_nc, p16_nc, t24_nc, sam1_nc;

	/* m (2^24 − 1) and m (2^16 − 1), m the significand */
	am_sub #(.W(48)) u_p24 (.a({1'b1, i_vw[22:0], 24'd0}), .b({24'd0, 1'b1, i_vw[22:0]}),
		.bi(1'b0), .d(p24), .bo(p24_nc));
	am_sub #(.W(48)) u_p16 (.a({8'd0, 1'b1, i_vw[22:0], 16'd0}), .b({24'd0, 1'b1, i_vw[22:0]}),
		.bi(1'b0), .d(p16), .bo(p16_nc));
	am_sub #(.W(5)) u_t24 (.a(5'd30), .b(i_vw[27:23]), .bi(1'b0), .d(t24), .bo(t24_nc));
	am_sub #(.W(6)) u_sam1 (.a(6'd21), .b(i_vw[28:23]), .bi(1'b0), .d(sam1), .bo(sam1_nc));
	/* the fp32 order keys of Dref and of the four texels (fast 33b decoupled +1 prefix, <= 10 cmos3) */
	wire [31:0] key_in [0:4];
	wire [32:0] key_sm [0:4];
	assign key_in[0] = i_vw;
	assign key_in[1] = i_lt[31:0];
	assign key_in[2] = i_lt[63:32];
	assign key_in[3] = i_lt[95:64];
	assign key_in[4] = i_lt[127:96];

	genvar gk, gt;
	generate
	for (gk = 0; gk < 5; gk = gk + 1) begin : g_key
		wire [31:0] kf = key_in[gk];
		(* keep = "true" *) wire k_sub_lo = kf[31];
		(* keep = "true" *) wire k_sub_hi = kf[31];
		(* keep = "true" *) wire k_z_lo = (kf[30:23] == 8'd0);
		(* keep = "true" *) wire k_z_hi = (kf[30:23] == 8'd0);
		wire [32:0] k_neg_in = {2'b11, ~kf[30:0]};
		wire [7:0]  k_z4;
		genvar gn;
		for (gn = 0; gn < 7; gn = gn + 1) begin : g_kn
			assign k_z4[gn] = (kf[gn*4 +: 4] == 4'd0);
		end
		assign k_z4[7] = (kf[30:28] == 3'd0);
		wire k_p8_0  = k_z4[1] && k_z4[0];
		wire k_p8_1  = k_z4[3] && k_z4[2];
		wire k_p8_2  = k_z4[5] && k_z4[4];
		wire k_p16_0 = k_p8_1 && k_p8_0;
		wire [8:0] k_cin;
		assign k_cin[0] = 1'b1;
		assign k_cin[1] = k_z4[0];
		assign k_cin[2] = k_p8_0;
		assign k_cin[3] = k_p8_0 && k_z4[2];
		assign k_cin[4] = k_p16_0;
		assign k_cin[5] = k_p16_0 && k_z4[4];
		assign k_cin[6] = k_p16_0 && k_p8_2;
		assign k_cin[7] = k_p16_0 && k_p8_2 && k_z4[6];
		assign k_cin[8] = k_p16_0 && k_p8_2 && k_z4[7] && k_z4[6];
		wire [32:0] k_neg;
		for (gn = 0; gn < 8; gn = gn + 1) begin : g_ks
			wire [3:0] kn = k_neg_in[gn*4 +: 4];
			wire kc = k_cin[gn];
			assign k_neg[gn*4 + 0] = kn[0] ^ kc;
			assign k_neg[gn*4 + 1] = kn[1] ^ (kc && kn[0]);
			assign k_neg[gn*4 + 2] = kn[2] ^ (kc && kn[1] && kn[0]);
			assign k_neg[gn*4 + 3] = kn[3] ^ (kc && kn[2] && kn[1] && kn[0]);
		end
		assign k_neg[32] = k_neg_in[32] ^ k_cin[8];
		assign key_sm[gk] = {
			k_z_hi ? 17'd0 : (k_sub_hi ? k_neg[32:16] : {2'b00, kf[30:16]}),
			k_z_lo ? 16'd0 : (k_sub_lo ? k_neg[15:0]  : kf[15:0])
		};
	end
	endgenerate

	wire d_sm_co = 1'b0;
	wire [3:0] t_sm_co = 4'd0;
	assign dkey = key_sm[0];
	wire [32:0] tkey [0:3];
	assign tkey[0] = key_sm[1];
	assign tkey[1] = key_sm[2];
	assign tkey[2] = key_sm[3];
	assign tkey[3] = key_sm[4];

	wire [8:0] ru, rv, rw, rl;
	wire [15:0] fan0, fan1;
	wire [5:0] w_nc;

	am_sub #(.W(9)) u_ru (.a(9'd256), .b({1'b0, i_wl[7:0]}), .bi(1'b0), .d(ru), .bo(w_nc[0]));
	am_sub #(.W(9)) u_rv (.a(9'd256), .b({1'b0, i_wl[15:8]}), .bi(1'b0), .d(rv), .bo(w_nc[1]));
	am_sub #(.W(9)) u_rl (.a(9'd256), .b({1'b0, i_wl[23:16]}), .bi(1'b0), .d(rl), .bo(w_nc[2]));
	am_sub #(.W(9)) u_rw (.a(9'd256), .b({1'b0, i_wl[31:24]}), .bi(1'b0), .d(rw), .bo(w_nc[3]));
	/* the LOD query's magnitudes */
	am_sub #(.W(16)) u_fa0 (.a(16'd0), .b(i_vw[15:0]), .bi(1'b0), .d(fan0), .bo(w_nc[4]));
	am_sub #(.W(16)) u_fa1 (.a(16'd0), .b(i_vw[31:16]), .bi(1'b0), .d(fan1), .bo(w_nc[5]));

	(* extract_reset = "no", max_fanout = 16 *) reg [47:0] p24_2, p16_2;
	(* extract_reset = "no", max_fanout = 16 *) reg [4:0] t24_2;
	(* extract_reset = "no", max_fanout = 16 *) reg [5:0] sam1_2;
	(* extract_reset = "no", shreg_extract = "no" *) reg [32:0] ka32_2;
	reg u24z2, u24o2, u16z2, u16o2;
	(* extract_reset = "no", shreg_extract = "no" *) reg [131:0] kb2;
	reg dnan2;
	reg [3:0] tnan2;
	(* extract_reset = "no", shreg_extract = "no" *) reg [8:0] wu2 [0:3];
	(* extract_reset = "no", shreg_extract = "no" *) reg [8:0] wv2 [0:3];
	(* extract_reset = "no", shreg_extract = "no" *) reg [8:0] wa2, wb2;
	(* extract_reset = "no", shreg_extract = "no" *) reg [31:0] fa2;
	(* keep = "true" *) wire [3:0] bil_u = {4{i_bil}};
	(* keep = "true" *) wire [3:0] bil_v = {4{i_bil}};
	/* texel t's key (D32F: the order key; D16, D24: the bits) and NaN */
	wire [131:0] kbw;
	wire [3:0] tnan_w;
	generate
	for (gt = 0; gt < 4; gt = gt + 1) begin : g_kb
		wire [30:0] tw = i_lt[gt*32 +: 31];	/* (the sign: in tkey) */
		(* dont_touch = "true" *) wire d32_lo  = i_d32;
		(* dont_touch = "true" *) wire d32_hi0 = i_d32;
		(* dont_touch = "true" *) wire d32_hi1 = i_d32;
		(* dont_touch = "true" *) wire d16_hi0 = i_d16;
		assign tnan_w[gt] = tw[30:23] == 8'hFF && tw[22:0] != 23'd0;
		assign kbw[gt*33 +: 16] = d32_lo ? tkey[gt][15:0] : tw[15:0];
		assign kbw[gt*33 + 16 +: 8] = d32_hi0 ? tkey[gt][23:16] : d16_hi0 ? 8'd0 : tw[23:16];
		assign kbw[gt*33 + 24 +: 9] = d32_hi1 ? tkey[gt][32:24] : 9'd0;
	end
	endgenerate
	always @(posedge clk) begin
		p24_2 <= p24;
		p16_2 <= p16;
		t24_2 <= t24;
		sam1_2 <= sam1;
		ka32_2 <= dkey;
		u24z2 <= de == 8'd0 || i_vw[31] || dnan || de < 8'd102;
		u24o2 <= de >= 8'd127;
		u16z2 <= i_vw[31] || dnan || de < 8'd103;
		u16o2 <= de >= 8'd127;
		kb2 <= kbw;
		dnan2 <= dnan;
		tnan2 <= tnan_w;
		for (t = 0; t < 4; t = t + 1) begin
			/* texel t's bilinear factors (without bilinear 256 × 256 for texel 0) */
			wu2[t] <= !bil_u[t] ? (t == 0 ? 9'd256 : 9'd0) : t[0] ? {1'b0, i_wl[7:0]} : ru;
			wv2[t] <= !bil_v[t] ? 9'd256 : t[1] ? {1'b0, i_wl[15:8]} : rv;
		end
		wa2 <= i_p17 ? {1'b0, i_wl[31:24]} : rw;
		wb2 <= i_p18 ? {1'b0, i_wl[23:16]} : rl;
		fa2 <= {i_vw[31] ? fan1 : i_vw[31:16], i_vw[15] ? fan0 : i_vw[15:0]};
	end

	/* ---- P2 → P3 ---- */
	wire [71:0] z24;
	wire [47:0] s16;

	am_shift #(.W(72), .SW(5)) u_z24 (.a({p24_2, 24'd0}), .amt(t24_2), .left(1'b0),
		.arith(1'b0), .y(z24));
	am_shift #(.W(48), .SW(6)) u_s16 (.a(p16_2), .amt(sam1_2), .left(1'b0), .arith(1'b0),
		.y(s16));

	reg [47:0] m24, m16;
	always @(*)
		for (i = 0; i < 48; i = i + 1) begin
			m24[i] = (i + 24 >= {27'd0, t24_2}) && (i <= 22 || {27'd0, t24_2} >= i - 22);
			m16[i] = i[5:0] < sam1_2;
		end

	wire [17:0] w5 [0:3];
	wire [17:0] wp5;
	generate
	for (gt = 0; gt < 4; gt = gt + 1) begin : g_w
		am_mul #(.AW(9), .BW(9), .SIGNED(0), .LAT(3)) u_w (
			.clk(clk), .en(1'b1), .a(wu2[gt]), .b(wv2[gt]), .p(w5[gt])
		);
	end
	endgenerate
	am_mul #(.AW(9), .BW(9), .SIGNED(0), .LAT(3)) u_wp (
		.clk(clk), .en(1'b1), .a(wa2), .b(wb2), .p(wp5)
	);

	reg [24:0] z3;	/* bits 71:47 */
	reg st24_3, st16_3;
	reg [16:0] s16_3;	/* bits 16:0 */
	(* extract_reset = "no", shreg_extract = "no" *) reg [32:0] ka32_3;
	(* max_fanout = 16 *) reg sel3_d32, sel3_q16, sel3_q24, sel3_z_lo, sel3_o_lo, sel3_z_mid, sel3_o_mid;
	(* extract_reset = "no", shreg_extract = "no" *) reg [131:0] kb3;
	reg [3:0] kn3;
	(* extract_reset = "no", shreg_extract = "no" *) reg [31:0] fa3;
	always @(posedge clk) begin
		z3 <= z24[71:47];
		st24_3 <= (p24_2 & m24) != 48'd0;
		s16_3 <= s16[16:0];
		st16_3 <= (p16_2 & m16) != 48'd0;
		{ka32_3, kb3, fa3} <= {ka32_2, kb2, fa2};
		sel3_d32   <= d32_2;
		sel3_q16   <= !d32_2 &&  d16_2 && !u16z2 && !u16o2;
		sel3_q24   <= !d32_2 && !d16_2 && !u24z2 && !u24o2;
		sel3_z_lo  <= !d32_2 && (d16_2 ? u16z2 : u24z2);
		sel3_o_lo  <= !d32_2 && (d16_2 ? (!u16z2 && u16o2) : (!u24z2 && u24o2));
		sel3_z_mid <= !d32_2 && (d16_2 || u24z2);
		sel3_o_mid <= !d32_2 && !d16_2 && !u24z2 && u24o2;
		for (t = 0; t < 4; t = t + 1)
			kn3[t] <= d32_2 && (dnan2 || tnan2[t]);
	end

	/* ---- P3 → P4: parallel carry-select 16-chunk compare pre-stage (<= 11 cmos3) ---- */
	wire ci24 = z3[0] && (st24_3 || z3[1]);
	wire [11:0] cin24;
	wire [5:0] z24_p4;
	genvar gq24;
	generate
	for (gq24 = 0; gq24 < 6; gq24 = gq24 + 1) begin : g_q24_p4
		assign z24_p4[gq24] = &z3[1 + gq24*4 +: 4];
	end
	endgenerate
	wire z24_p8_0 = z24_p4[1] && z24_p4[0];
	wire z24_p8_1 = z24_p4[3] && z24_p4[2];
	assign cin24[0]  = ci24;
	assign cin24[1]  = ci24 && (&z3[2:1]);
	assign cin24[2]  = ci24 && z24_p4[0];
	assign cin24[3]  = ci24 && z24_p4[0] && (&z3[6:5]);
	assign cin24[4]  = ci24 && z24_p8_0;
	assign cin24[5]  = ci24 && z24_p8_0 && (&z3[10:9]);
	assign cin24[6]  = ci24 && z24_p8_0 && z24_p4[2];
	assign cin24[7]  = ci24 && z24_p8_0 && z24_p4[2] && (&z3[14:13]);
	assign cin24[8]  = ci24 && z24_p8_0 && z24_p8_1;
	assign cin24[9]  = ci24 && z24_p8_0 && z24_p8_1 && (&z3[18:17]);
	assign cin24[10] = ci24 && z24_p8_0 && z24_p8_1 && z24_p4[4];
	assign cin24[11] = ci24 && z24_p8_0 && z24_p8_1 && z24_p4[4] && (&z3[22:21]);

	wire ci16 = s16_3[0] && (st16_3 || s16_3[1]);
	wire [7:0] cin16;
	wire [3:0] s16_p4;
	generate
	for (gq24 = 0; gq24 < 4; gq24 = gq24 + 1) begin : g_q16_p4
		assign s16_p4[gq24] = &s16_3[1 + gq24*4 +: 4];
	end
	endgenerate
	wire s16_p8_0 = s16_p4[1] && s16_p4[0];
	assign cin16[0] = ci16;
	assign cin16[1] = ci16 && (&s16_3[2:1]);
	assign cin16[2] = ci16 && s16_p4[0];
	assign cin16[3] = ci16 && s16_p4[0] && (&s16_3[6:5]);
	assign cin16[4] = ci16 && s16_p8_0;
	assign cin16[5] = ci16 && s16_p8_0 && (&s16_3[10:9]);
	assign cin16[6] = ci16 && s16_p8_0 && s16_p4[2];
	assign cin16[7] = ci16 && s16_p8_0 && s16_p4[2] && (&s16_3[14:13]);

	/* the compare's fields to P4 */
	(* max_fanout = 16 *) reg d32_2, d32_3, d16_2;
	(* max_fanout = 16 *) reg [2:0] op2, op3;
	(* extract_reset = "no" *) reg [15:0] p4_eq2 [0:3];
	(* extract_reset = "no" *) reg [15:0] p4_lt2 [0:3];
	(* extract_reset = "no" *) reg [3:0] p4_flt, p4_feq, p4_inv_eff;
	(* extract_reset = "no", shreg_extract = "no" *) reg [31:0] fa4;

	wire op3_use_base = (op3 != 3'd0) && (op3 != 3'd7);
	wire op3_nan_pass = (op3 == 3'd5);
	wire op3_always_1 = (op3 == 3'd7);

	wire [15:0] eq2_p3 [0:3];
	wire [15:0] lt2_p3 [0:3];
	generate
	for (gt = 0; gt < 4; gt = gt + 1) begin : g_cmp_p3
		wire [32:0] kb3_t = kb3[gt*33 +: 33];
		genvar gc2;
		for (gc2 = 0; gc2 < 15; gc2 = gc2 + 1) begin : g_c2
			wire [1:0] b2 = kb3_t[gc2*2 +: 2];
			wire [1:0] ad = ka32_3[gc2*2 +: 2];
			wire [1:0] xd = ~(ad ^ b2);
			wire eq_d32 = &xd;
			wire lt_d32 = (!ad[1] && b2[1]) || (xd[1] && !ad[0] && b2[0]);
			if (gc2 >= 12) begin : g_hi_c
				/* d24 and d16 are 0 in bits [29:24] */
				wire eq_z = (b2 == 2'b00);
				wire lt_z = (b2 != 2'b00);
				assign eq2_p3[gt][gc2] = d32_3 ? eq_d32 : eq_z;
				assign lt2_p3[gt][gc2] = d32_3 ? lt_d32 : lt_z;
			end else begin : g_lo_c
				wire [1:0] z0 = z3[1 + gc2*2 +: 2];
				wire [1:0] z1 = {z0[1] ^ z0[0], ~z0[0]};
				wire [1:0] xz0 = ~(z0 ^ b2);
				wire [1:0] xz1 = ~(z1 ^ b2);
				wire eq_24_0 = &xz0;
				wire lt_24_0 = (!z0[1] && b2[1]) || (xz0[1] && !z0[0] && b2[0]);
				wire eq_24_1 = &xz1;
				wire lt_24_1 = (!z1[1] && b2[1]) || (xz1[1] && !z1[0] && b2[0]);
				wire eq_24_q = cin24[gc2] ? eq_24_1 : eq_24_0;
				wire lt_24_q = cin24[gc2] ? lt_24_1 : lt_24_0;
				if (gc2 >= 8) begin : g_16z
					wire eq_const_mid = (sel3_d32 && eq_d32) || (sel3_z_mid && (b2 == 2'b00)) || (sel3_o_mid && (b2 == 2'b11));
					wire lt_const_mid = (sel3_d32 && lt_d32) || (sel3_z_mid && (b2 != 2'b00));
					assign eq2_p3[gt][gc2] = (sel3_q24 && eq_24_q) || eq_const_mid;
					assign lt2_p3[gt][gc2] = (sel3_q24 && lt_24_q) || lt_const_mid;
				end else begin : g_16q
					wire [1:0] s0 = s16_3[1 + gc2*2 +: 2];
					wire [1:0] s1 = {s0[1] ^ s0[0], ~s0[0]};
					wire [1:0] xs0 = ~(s0 ^ b2);
					wire [1:0] xs1 = ~(s1 ^ b2);
					wire eq_16_0 = &xs0;
					wire lt_16_0 = (!s0[1] && b2[1]) || (xs0[1] && !s0[0] && b2[0]);
					wire eq_16_1 = &xs1;
					wire lt_16_1 = (!s1[1] && b2[1]) || (xs1[1] && !s1[0] && b2[0]);
					wire eq_16_q = cin16[gc2] ? eq_16_1 : eq_16_0;
					wire lt_16_q = cin16[gc2] ? lt_16_1 : lt_16_0;
					wire eq_const_lo = (sel3_d32 && eq_d32) || (sel3_z_lo && (b2 == 2'b00)) || (sel3_o_lo && (b2 == 2'b11));
					wire lt_const_lo = (sel3_d32 && lt_d32) || (sel3_z_lo && (b2 != 2'b00));
					assign eq2_p3[gt][gc2] = (sel3_q16 && eq_16_q) || (sel3_q24 && eq_24_q) || eq_const_lo;
					assign lt2_p3[gt][gc2] = (sel3_q16 && lt_16_q) || (sel3_q24 && lt_24_q) || lt_const_lo;
				end
			end
		end
		/* Chunk 15 includes inverted sign bit [32] so signed < maps to unsigned < */
		wire [2:0] a3 = d32_3 ? {!ka32_3[32], ka32_3[31:30]} : 3'b100;
		wire [2:0] b3 = {!kb3_t[32], kb3_t[31:30]};
		wire [2:0] x3 = ~(a3 ^ b3);
		assign eq2_p3[gt][15] = &x3;
		assign lt2_p3[gt][15] = (!a3[2] && b3[2]) || (x3[2] && !a3[1] && b3[1]) || (x3[2] && x3[1] && !a3[0] && b3[0]);
	end
	endgenerate
	wire unused_p4_tops = &{1'b0, z24_p4[5], s16_p4[3]};

	always @(posedge clk) begin
		{d32_2, d16_2, op2} <= {i_d32, i_d16, i_op};
		{d32_3, op3}        <= {d32_2, op2};
		fa4 <= fa3;
		for (t = 0; t < 4; t = t + 1) begin
			p4_eq2[t]     <= eq2_p3[t];
			p4_lt2[t]     <= lt2_p3[t];
			p4_flt[t]     <= op3_use_base && !kn3[t] && (op3[2] ^ op3[0]);
			p4_feq[t]     <= op3_use_base && !kn3[t] && (op3[2] ^ op3[1]);
			p4_inv_eff[t] <= (op3_use_base && !kn3[t]) ? op3[2] : (op3_always_1 || (op3_nan_pass && kn3[t]));
		end
	end

	/* P4: 4-level binary tree over 16 chunks + 2-gate op collapse */
	generate
	for (gt = 0; gt < 4; gt = gt + 1) begin : g_cmp
		wire [15:0] ceq0 = p4_eq2[gt];
		wire [15:0] clt0 = p4_lt2[gt];
		wire [7:0] ceq1, clt1;
		wire [3:0] ceq2, clt2;
		wire [1:0] ceq3, clt3;
		genvar gn;
		for (gn = 0; gn < 8; gn = gn + 1) begin : g_l1
			assign ceq1[gn] = ceq0[2*gn+1] && ceq0[2*gn];
			assign clt1[gn] = clt0[2*gn+1] || (ceq0[2*gn+1] && clt0[2*gn]);
		end
		for (gn = 0; gn < 4; gn = gn + 1) begin : g_l2
			assign ceq2[gn] = ceq1[2*gn+1] && ceq1[2*gn];
			assign clt2[gn] = clt1[2*gn+1] || (ceq1[2*gn+1] && clt1[2*gn]);
		end
		for (gn = 0; gn < 2; gn = gn + 1) begin : g_l3
			assign ceq3[gn] = ceq2[2*gn+1] && ceq2[2*gn];
			assign clt3[gn] = clt2[2*gn+1] || (ceq2[2*gn+1] && clt2[2*gn]);
		end
		wire eq_full = ceq3[1] && ceq3[0];
		wire lt_full = clt3[1] || (ceq3[1] && clt3[0]);
		wire raw_cmp = (p4_flt[gt] && lt_full) || (p4_feq[gt] && eq_full);
		assign o_ok[gt] = p4_inv_eff[gt] ^ raw_cmp;
	end
	endgenerate

	(* extract_reset = "no", shreg_extract = "no" *) reg [31:0] fa5;
	always @(posedge clk) begin
		fa5 <= fa4;
	end
	assign o_w = {w5[3][16:0], w5[2][16:0], w5[1][16:0], w5[0][16:0]};
	assign o_wp = wp5[16:0];
	assign o_fa = fa5;

	wire unused_ok = &{1'b0, rst_n, unused_p4_tops, d_sm_co, t_sm_co, p16_nc, p24_nc, sam1_nc, t24_nc, w_nc, s16[47:17], z24[46:0], wp5[17], w5[0][17], w5[1][17],
		w5[2][17], w5[3][17]};
endmodule

`default_nettype wire
