/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * am_fp_fma_p: fused a * b + c with one rounding under rm (am_fp_rnd's
 * modes; 6: RDN's sign of an exact zero sum), denormals flushed, NaN
 * 0x7FC00000; noc: no addend (fmul).  p is the product of the
 * significands (am_fp_fma has its own multiplier; atg_su_fpu shares one).
 * Supports:
 *   LAT = 0: combinational (0 cycles)
 *   LAT = 1: 2-stage pipelined (Stage 0+1 combinational, Stage 2 registered)
 *   LAT = 2: 3-stage pipelined (Stage 0: align, Stage 1: add, Stage 2: LOD+norm+round)
 * See spec/library/fp/am_fp_fma_p.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module am_fp_fma_p #(
	parameter LAT = 0
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire [31:0] a0,
	input wire [31:0] b0,
	input wire [31:0] c0,
	input wire [2:0] rm,
	input wire noc,
	input wire [47:0] p,
	output wire [31:0] r
);
	wire [31:0] a = a0[30:23] == 8'd0 ? {a0[31], 31'd0} : a0;
	wire [31:0] b = b0[30:23] == 8'd0 ? {b0[31], 31'd0} : b0;
	wire [31:0] c = c0[30:23] == 8'd0 ? {c0[31], 31'd0} : c0;
	wire sp = a[31] ^ b[31];
	wire sc = c[31];
	wire rdn = rm == 3'd2 || rm == 3'd6;
	wire nan_a = a[30:23] == 8'hFF && a[22:0] != 0;
	wire nan_b = b[30:23] == 8'hFF && b[22:0] != 0;
	wire nan_c = !noc && c[30:23] == 8'hFF && c[22:0] != 0;
	wire inf_a = a[30:23] == 8'hFF && a[22:0] == 0;
	wire inf_b = b[30:23] == 8'hFF && b[22:0] == 0;
	wire inf_c = !noc && c[30:23] == 8'hFF && c[22:0] == 0;
	wire za = a[30:23] == 8'd0;
	wire zb = b[30:23] == 8'd0;
	wire zc = noc || c[30:23] == 8'd0;
	wire [23:0] mc = zc ? 24'd0 : {1'b1, c[22:0]};
	wire [12:0] carry_unused;

	/* k: C's LSB relative to P's (units of P's LSB), the window has two bits
	 * below P's LSB: k = ec - ea - eb + 150 */
	wire [11:0] ab_0, c150_0, c152_0, c101_0;
	am_add #(.W(12)) u_ab (.a({4'd0, a[30:23]}), .b({4'd0, b[30:23]}), .ci(1'b0), .s(ab_0),
		.co(carry_unused[0]));
	am_add #(.W(12)) u_c150 (.a({4'd0, c[30:23]}), .b(12'd150), .ci(1'b0), .s(c150_0),
		.co(carry_unused[1]));
	am_add #(.W(12)) u_c101 (.a({4'd0, c[30:23]}), .b(12'd101), .ci(1'b0), .s(c101_0),
		.co(carry_unused[3]));
	am_add #(.W(12)) u_c152 (.a({4'd0, c[30:23]}), .b(12'd152), .ci(1'b0), .s(c152_0),
		.co(carry_unused[5]));

	/* Special cases determined in Stage 0a0 */
	wire spec_nan = nan_a || nan_b || nan_c || (inf_a && zb) || (inf_b && za);
	wire spec_inf_ab = (inf_a || inf_b) && !spec_nan;
	wire spec_inf_c = inf_c && !spec_nan && !spec_inf_ab;
	wire spec_zero = (za || zb) && !spec_nan && !spec_inf_ab && !spec_inf_c;
	wire spec_hit = spec_nan || spec_inf_ab || spec_inf_c || spec_zero;
	wire [31:0] spec_res =
		spec_nan ? 32'h7FC00000 :
		spec_inf_ab ? (inf_c && sc != sp ? 32'h7FC00000 : {sp, 8'hFF, 23'd0}) :
		spec_inf_c ? c :
		(!zc ? c : noc ? {sp, 31'd0} : {rdn ? (sp | sc) : (sp & sc), 31'd0});

	/* --- Stage 0a0 -> Stage 0a1 Pipeline Register (LAT >= 7) --- */
	reg [47:0] r0a0_p;
	reg [23:0] r0a0_mc;
	reg        r0a0_zc, r0a0_sp, r0a0_sc, r0a0_rdn, r0a0_spec_hit;
	reg [2:0]  r0a0_rm;
	reg [11:0] r0a0_ab, r0a0_c150, r0a0_c101, r0a0_c152;
	reg [31:0] r0a0_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0a0_p <= 48'd0; r0a0_mc <= 24'd0; r0a0_zc <= 1'b0; r0a0_sp <= 1'b0;
			r0a0_sc <= 1'b0; r0a0_rdn <= 1'b0; r0a0_spec_hit <= 1'b0; r0a0_rm <= 3'd0;
			r0a0_ab <= 12'd0; r0a0_c150 <= 12'd0; r0a0_c101 <= 12'd0; r0a0_c152 <= 12'd0;
			r0a0_spec_res <= 32'd0;
		end else if (en) begin
			r0a0_p <= p; r0a0_mc <= mc; r0a0_zc <= zc; r0a0_sp <= sp;
			r0a0_sc <= sc; r0a0_rdn <= rdn; r0a0_spec_hit <= spec_hit; r0a0_rm <= rm;
			r0a0_ab <= ab_0; r0a0_c150 <= c150_0; r0a0_c101 <= c101_0; r0a0_c152 <= c152_0;
			r0a0_spec_res <= spec_res;
		end
	end

	wire [47:0] p0a1        = (LAT >= 7) ? r0a0_p        : p;
	wire [23:0] mc0a1       = (LAT >= 7) ? r0a0_mc       : mc;
	wire        zc0a1       = (LAT >= 7) ? r0a0_zc       : zc;
	wire        sp0a1       = (LAT >= 7) ? r0a0_sp       : sp;
	wire        sc0a1       = (LAT >= 7) ? r0a0_sc       : sc;
	wire        rdn0a1      = (LAT >= 7) ? r0a0_rdn      : rdn;
	wire        spec_hit0a1 = (LAT >= 7) ? r0a0_spec_hit : spec_hit;
	wire [2:0]  rm0a1       = (LAT >= 7) ? r0a0_rm       : rm;
	wire [11:0] ab          = (LAT >= 7) ? r0a0_ab       : ab_0;
	wire [11:0] c150        = (LAT >= 7) ? r0a0_c150     : c150_0;
	wire [11:0] c101        = (LAT >= 7) ? r0a0_c101     : c101_0;
	wire [11:0] c152        = (LAT >= 7) ? r0a0_c152     : c152_0;
	wire [31:0] spec_res0a1 = (LAT >= 7) ? r0a0_spec_res : spec_res;

	wire [11:0] k_c, kadj_c, kp2_c;
	wire [4:0] rs_c;
	am_sub #(.W(12)) u_k (.a(c150), .b(ab), .bi(1'b0), .d(k_c), .bo(carry_unused[2]));
	am_sub #(.W(12)) u_kadj (.a(c101), .b(ab), .bi(1'b0), .d(kadj_c), .bo(carry_unused[4]));
	am_sub #(.W(12)) u_kp2 (.a(c152), .b(ab), .bi(1'b0), .d(kp2_c), .bo(carry_unused[6]));
	am_sub #(.W(5)) u_rs (.a(ab[4:0]), .b(c152[4:0]), .bi(1'b0), .d(rs_c),
		.bo(carry_unused[7]));	/* 1..23: -(c152 - ab) mod 32 */

	/* --- Stage 0a -> Stage 0b Pipeline Register (LAT >= 5) --- */
	reg [47:0] r0a_p;
	reg [23:0] r0a_mc;
	reg        r0a_zc, r0a_sp, r0a_sc, r0a_rdn, r0a_spec_hit;
	reg [2:0]  r0a_rm;
	reg [11:0] r0a_ab, r0a_k, r0a_kadj, r0a_kp2;
	reg [4:0]  r0a_rs;
	reg [31:0] r0a_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0a_p <= 48'd0; r0a_mc <= 24'd0; r0a_zc <= 1'b0; r0a_sp <= 1'b0;
			r0a_sc <= 1'b0; r0a_rdn <= 1'b0; r0a_spec_hit <= 1'b0; r0a_rm <= 3'd0;
			r0a_ab <= 12'd0; r0a_k <= 12'd0; r0a_kadj <= 12'd0; r0a_kp2 <= 12'd0;
			r0a_rs <= 5'd0; r0a_spec_res <= 32'd0;
		end else if (en) begin
			r0a_p <= p0a1; r0a_mc <= mc0a1; r0a_zc <= zc0a1; r0a_sp <= sp0a1;
			r0a_sc <= sc0a1; r0a_rdn <= rdn0a1; r0a_spec_hit <= spec_hit0a1; r0a_rm <= rm0a1;
			r0a_ab <= ab; r0a_k <= k_c; r0a_kadj <= kadj_c; r0a_kp2 <= kp2_c;
			r0a_rs <= rs_c; r0a_spec_res <= spec_res0a1;
		end
	end

	wire [47:0] p0b        = (LAT >= 5) ? r0a_p        : p0a1;
	wire [23:0] mc0b       = (LAT >= 5) ? r0a_mc       : mc0a1;
	wire        zc0b       = (LAT >= 5) ? r0a_zc       : zc0a1;
	wire        sp0b       = (LAT >= 5) ? r0a_sp       : sp0a1;
	wire        sc0b       = (LAT >= 5) ? r0a_sc       : sc0a1;
	wire        rdn0b      = (LAT >= 5) ? r0a_rdn      : rdn0a1;
	wire        spec_hit0b = (LAT >= 5) ? r0a_spec_hit : spec_hit0a1;
	wire [2:0]  rm0b       = (LAT >= 5) ? r0a_rm       : rm0a1;
	wire [11:0] ab0b       = (LAT >= 5) ? r0a_ab       : ab;
	wire [11:0] k          = (LAT >= 5) ? r0a_k        : k_c;
	wire [11:0] kadj       = (LAT >= 5) ? r0a_kadj     : kadj_c;
	wire [11:0] kp2        = (LAT >= 5) ? r0a_kp2      : kp2_c;
	wire [4:0]  rs         = (LAT >= 5) ? r0a_rs       : rs_c;
	wire [31:0] spec_res0b = (LAT >= 5) ? r0a_spec_res : spec_res0a1;

	wire signed [11:0] ks = k;
	wire kbig = ks > 12'sd49;
	wire kleft = !kp2[11];
	wire kright = ks > -12'sd26;

	/* --- Stage 0b0 -> Stage 0b1 Pipeline Register (LAT >= 16) --- */
	reg [47:0] r0b_cs_p;
	reg [23:0] r0b_cs_mc;
	reg        r0b_cs_zc, r0b_cs_sp, r0b_cs_sc, r0b_cs_rdn, r0b_cs_spec_hit;
	reg        r0b_cs_kbig, r0b_cs_kleft, r0b_cs_kright;
	reg [2:0]  r0b_cs_rm;
	reg [11:0] r0b_cs_ab, r0b_cs_kadj, r0b_cs_kp2;
	reg [4:0]  r0b_cs_rs;
	reg [31:0] r0b_cs_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0b_cs_p <= 48'd0; r0b_cs_mc <= 24'd0; r0b_cs_zc <= 1'b0; r0b_cs_sp <= 1'b0;
			r0b_cs_sc <= 1'b0; r0b_cs_rdn <= 1'b0; r0b_cs_spec_hit <= 1'b0;
			r0b_cs_kbig <= 1'b0; r0b_cs_kleft <= 1'b0; r0b_cs_kright <= 1'b0;
			r0b_cs_rm <= 3'd0; r0b_cs_ab <= 12'd0; r0b_cs_kadj <= 12'd0; r0b_cs_kp2 <= 12'd0;
			r0b_cs_rs <= 5'd0; r0b_cs_spec_res <= 32'd0;
		end else if (en) begin
			r0b_cs_p <= p0b; r0b_cs_mc <= mc0b; r0b_cs_zc <= zc0b; r0b_cs_sp <= sp0b;
			r0b_cs_sc <= sc0b; r0b_cs_rdn <= rdn0b; r0b_cs_spec_hit <= spec_hit0b;
			r0b_cs_kbig <= kbig; r0b_cs_kleft <= kleft; r0b_cs_kright <= kright;
			r0b_cs_rm <= rm0b; r0b_cs_ab <= ab0b; r0b_cs_kadj <= kadj; r0b_cs_kp2 <= kp2;
			r0b_cs_rs <= rs; r0b_cs_spec_res <= spec_res0b;
		end
	end

	wire [47:0] p0c        = (LAT >= 16) ? r0b_cs_p        : p0b;
	wire [23:0] mc0c       = (LAT >= 16) ? r0b_cs_mc       : mc0b;
	wire        zc0c       = (LAT >= 16) ? r0b_cs_zc       : zc0b;
	wire        sp0c       = (LAT >= 16) ? r0b_cs_sp       : sp0b;
	wire        sc0c       = (LAT >= 16) ? r0b_cs_sc       : sc0b;
	wire        rdn0c      = (LAT >= 16) ? r0b_cs_rdn      : rdn0b;
	wire        spec_hit0c = (LAT >= 16) ? r0b_cs_spec_hit : spec_hit0b;
	wire        kbig_c     = (LAT >= 16) ? r0b_cs_kbig     : kbig;
	wire        kleft_c    = (LAT >= 16) ? r0b_cs_kleft    : kleft;
	wire        kright_c   = (LAT >= 16) ? r0b_cs_kright   : kright;
	wire [2:0]  rm0c       = (LAT >= 16) ? r0b_cs_rm       : rm0b;
	wire [11:0] ab0c       = (LAT >= 16) ? r0b_cs_ab       : ab0b;
	wire [11:0] kadj_c2    = (LAT >= 16) ? r0b_cs_kadj     : kadj;
	wire [11:0] kp2_c2     = (LAT >= 16) ? r0b_cs_kp2      : kp2;
	wire [4:0]  rs_c2      = (LAT >= 16) ? r0b_cs_rs       : rs;
	wire [31:0] spec_res0c = (LAT >= 16) ? r0b_cs_spec_res : spec_res0b;

	wire [76:0] cs;
	am_shift #(.W(77), .SW(7)) u_cs (.a({53'd0, mc0c}), .amt(kleft_c ? kp2_c2[6:0] : {2'd0, rs_c2}), .left(kleft_c),
		.arith(1'b0), .y(cs));
	/* the sticky bit: the bits of mc0c below rs_c2 */
	wire [23:0] cmk;
	am_shift #(.W(24), .SW(5), .DIR(2)) u_cmk (.a(24'hFFFFFF), .amt(rs_c2), .left(1'b1), .arith(1'b0), .y(cmk));
	wire cst = |(mc0c & ~cmk);
	wire [11:0] adj = !zc0c && kbig_c ? kadj_c2 : 12'd0;
	wire [76:0] pw = !zc0c && kbig_c ? {76'd0, 1'b1} : {27'd0, p0c, 2'b00};
	wire [76:0] cw = zc0c ? 77'd0 : kbig_c ? {2'b00, mc0c, 51'd0} : kleft_c ? cs : kright_c ? {cs[76:1], cs[0] | cst} : 77'd1;
	/* the sum: pw + cw, pw - cw or cw - pw (the larger first) */
	wire same = zc0c || sp0c == sc0c;

	/* --- Stage 0 -> Stage 1 Pipeline Register (LAT >= 2) --- */
	reg [76:0] r0_pw;
	reg [76:0] r0_cw;
	reg        r0_same;
	reg        r0_sp;
	reg        r0_sc;
	reg [11:0] r0_ab;
	reg [11:0] r0_adj;
	reg [2:0]  r0_rm;
	reg        r0_rdn;
	reg        r0_spec_hit;
	reg [31:0] r0_spec_res;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0_pw       <= 77'd0;
			r0_cw       <= 77'd0;
			r0_same     <= 1'b0;
			r0_sp       <= 1'b0;
			r0_sc       <= 1'b0;
			r0_ab       <= 12'd0;
			r0_adj      <= 12'd0;
			r0_rm       <= 3'd0;
			r0_rdn      <= 1'b0;
			r0_spec_hit <= 1'b0;
			r0_spec_res <= 32'd0;
		end else if (en) begin
			r0_pw       <= pw;
			r0_cw       <= cw;
			r0_same     <= same;
			r0_sp       <= sp0c;
			r0_sc       <= sc0c;
			r0_ab       <= ab0c;
			r0_adj      <= adj;
			r0_rm       <= rm0c;
			r0_rdn      <= rdn0c;
			r0_spec_hit <= spec_hit0c;
			r0_spec_res <= spec_res0c;
		end
	end

	/* --- Stage 1: 77-bit compare (Stage 1a) and 3-stage 26+26+25 add/sub (Stage 1b1/1b2/1c) --- */
	wire [76:0] s0_pw       = (LAT >= 2) ? r0_pw       : pw;
	wire [76:0] s0_cw       = (LAT >= 2) ? r0_cw       : cw;
	wire        s0_same     = (LAT >= 2) ? r0_same     : same;
	wire        s0_sp       = (LAT >= 2) ? r0_sp       : sp0c;
	wire        s0_sc       = (LAT >= 2) ? r0_sc       : sc0c;
	wire [11:0] s0_ab       = (LAT >= 2) ? r0_ab       : ab0c;
	wire [11:0] s0_adj      = (LAT >= 2) ? r0_adj      : adj;
	wire [2:0]  s0_rm       = (LAT >= 2) ? r0_rm       : rm0c;
	wire        s0_rdn      = (LAT >= 2) ? r0_rdn      : rdn0c;
	wire        s0_spec_hit = (LAT >= 2) ? r0_spec_hit : spec_hit0c;
	wire [31:0] s0_spec_res = (LAT >= 2) ? r0_spec_res : spec_res0c;

	wire [9:0] pge_gt_c, pge_eq_c;
	genvar gc;
	generate
		for (gc = 0; gc < 9; gc = gc + 1) begin : g_pge_c
			assign pge_gt_c[gc] = s0_pw[gc*8 +: 8] >  s0_cw[gc*8 +: 8];
			assign pge_eq_c[gc] = s0_pw[gc*8 +: 8] == s0_cw[gc*8 +: 8];
		end
	endgenerate
	assign pge_gt_c[9] = s0_pw[76:72] >  s0_cw[76:72];
	assign pge_eq_c[9] = s0_pw[76:72] == s0_cw[76:72];

	/* --- Stage 1a0 -> Stage 1a1 Pipeline Register (LAT >= 13) --- */
	reg [9:0]  r0b_gt, r0b_eq;
	reg [76:0] r0b_pw, r0b_cw;
	reg        r0b_same, r0b_sp, r0b_sc, r0b_rdn, r0b_spec_hit;
	reg [11:0] r0b_ab, r0b_adj;
	reg [2:0]  r0b_rm;
	reg [31:0] r0b_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0b_gt <= 10'd0; r0b_eq <= 10'd0; r0b_pw <= 77'd0; r0b_cw <= 77'd0;
			r0b_same <= 1'b0; r0b_sp <= 1'b0; r0b_sc <= 1'b0; r0b_rdn <= 1'b0;
			r0b_spec_hit <= 1'b0; r0b_ab <= 12'd0; r0b_adj <= 12'd0; r0b_rm <= 3'd0;
			r0b_spec_res <= 32'd0;
		end else if (en) begin
			r0b_gt <= pge_gt_c; r0b_eq <= pge_eq_c; r0b_pw <= s0_pw; r0b_cw <= s0_cw;
			r0b_same <= s0_same; r0b_sp <= s0_sp; r0b_sc <= s0_sc; r0b_rdn <= s0_rdn;
			r0b_spec_hit <= s0_spec_hit; r0b_ab <= s0_ab; r0b_adj <= s0_adj; r0b_rm <= s0_rm;
			r0b_spec_res <= s0_spec_res;
		end
	end

	wire [9:0]  pge_gt_w     = (LAT >= 13) ? r0b_gt       : pge_gt_c;
	wire [9:0]  pge_eq_w     = (LAT >= 13) ? r0b_eq       : pge_eq_c;
	wire [76:0] s0b_pw       = (LAT >= 13) ? r0b_pw       : s0_pw;
	wire [76:0] s0b_cw       = (LAT >= 13) ? r0b_cw       : s0_cw;
	wire        s0b_same     = (LAT >= 13) ? r0b_same     : s0_same;
	wire        s0b_sp       = (LAT >= 13) ? r0b_sp       : s0_sp;
	wire        s0b_sc       = (LAT >= 13) ? r0b_sc       : s0_sc;
	wire [11:0] s0b_ab       = (LAT >= 13) ? r0b_ab       : s0_ab;
	wire [11:0] s0b_adj      = (LAT >= 13) ? r0b_adj      : s0_adj;
	wire [2:0]  s0b_rm       = (LAT >= 13) ? r0b_rm       : s0_rm;
	wire        s0b_rdn      = (LAT >= 13) ? r0b_rdn      : s0_rdn;
	wire        s0b_spec_hit = (LAT >= 13) ? r0b_spec_hit : s0_spec_hit;
	wire [31:0] s0b_spec_res = (LAT >= 13) ? r0b_spec_res : s0_spec_res;

	wire pge_g2 = pge_gt_w[9] || (pge_eq_w[9] && pge_gt_w[8]);
	wire pge_e2 = pge_eq_w[9] && pge_eq_w[8];
	wire pge_g1 = pge_gt_w[7] || (pge_eq_w[7] && (pge_gt_w[6] || (pge_eq_w[6] && (pge_gt_w[5] || (pge_eq_w[5] && pge_gt_w[4])))));
	wire pge_e1 = &pge_eq_w[7:4];
	wire pge_g0 = pge_gt_w[3] || (pge_eq_w[3] && (pge_gt_w[2] || (pge_eq_w[2] && (pge_gt_w[1] || (pge_eq_w[1] && (pge_gt_w[0] || pge_eq_w[0]))))));
	wire        s0_pge      = pge_g2 || (pge_e2 && (pge_g1 || (pge_e1 && pge_g0)));
	wire [76:0] s0_op_a     = s0b_same || s0_pge ? s0b_pw : s0b_cw;
	wire [76:0] s0_op_b     = s0b_same || s0_pge ? s0b_cw : s0b_pw;
	wire        s0_op_sub   = !s0b_same;
	wire [76:0] s0_op_bx    = s0_op_sub ? ~s0_op_b : s0_op_b;
	wire        s0_sr       = s0b_same || s0_pge ? s0b_sp : s0b_sc;

	/* --- Stage 1a -> Stage 1b Pipeline Register (LAT >= 8) --- */
	reg [76:0] r0c_op_a, r0c_op_bx;
	reg        r0c_ci, r0c_sr, r0c_rdn, r0c_spec_hit;
	reg [11:0] r0c_ab, r0c_adj;
	reg [2:0]  r0c_rm;
	reg [31:0] r0c_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r0c_op_a <= 77'd0; r0c_op_bx <= 77'd0; r0c_ci <= 1'b0; r0c_sr <= 1'b0;
			r0c_rdn <= 1'b0; r0c_spec_hit <= 1'b0; r0c_ab <= 12'd0; r0c_adj <= 12'd0;
			r0c_rm <= 3'd0; r0c_spec_res <= 32'd0;
		end else if (en) begin
			r0c_op_a <= s0_op_a; r0c_op_bx <= s0_op_bx; r0c_ci <= s0_op_sub; r0c_sr <= s0_sr;
			r0c_rdn <= s0b_rdn; r0c_spec_hit <= s0b_spec_hit; r0c_ab <= s0b_ab; r0c_adj <= s0b_adj;
			r0c_rm <= s0b_rm; r0c_spec_res <= s0b_spec_res;
		end
	end

	wire [76:0] s1a_op_a     = (LAT >= 8) ? r0c_op_a     : s0_op_a;
	wire [76:0] s1a_op_bx    = (LAT >= 8) ? r0c_op_bx    : s0_op_bx;
	wire        s1a_ci       = (LAT >= 8) ? r0c_ci       : s0_op_sub;
	wire        s1a_sr       = (LAT >= 8) ? r0c_sr       : s0_sr;
	wire [11:0] s1a_ab       = (LAT >= 8) ? r0c_ab       : s0b_ab;
	wire [11:0] s1a_adj      = (LAT >= 8) ? r0c_adj      : s0b_adj;
	wire [2:0]  s1a_rm       = (LAT >= 8) ? r0c_rm       : s0b_rm;
	wire        s1a_rdn      = (LAT >= 8) ? r0c_rdn      : s0b_rdn;
	wire        s1a_spec_hit = (LAT >= 8) ? r0c_spec_hit : s0b_spec_hit;
	wire [31:0] s1a_spec_res = (LAT >= 8) ? r0c_spec_res : s0b_spec_res;

	wire [76:0] sum;
	wire        s1c_sr;
	wire [11:0] s1c_ab;
	wire [11:0] s1c_adj;
	wire [2:0]  s1c_rm;
	wire        s1c_rdn;
	wire        s1c_spec_hit;
	wire [31:0] s1c_spec_res;
	generate
		if (LAT >= 9) begin : g_sum_split
			wire [25:0] sum_lo_w;
			wire        sum_c0_w;
			am_add #(.W(26)) u_sum0 (.a(s1a_op_a[25:0]), .b(s1a_op_bx[25:0]), .ci(s1a_ci),
				.s(sum_lo_w), .co(sum_c0_w));

			reg [25:0] r1b1_sum0;
			reg        r1b1_c0, r1b1_sr, r1b1_rdn, r1b1_spec_hit;
			reg [50:0] r1b1_op_a, r1b1_op_bx;
			reg [11:0] r1b1_ab, r1b1_adj;
			reg [2:0]  r1b1_rm;
			reg [31:0] r1b1_spec_res;
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					r1b1_sum0 <= 26'd0; r1b1_c0 <= 1'b0; r1b1_sr <= 1'b0; r1b1_rdn <= 1'b0;
					r1b1_spec_hit <= 1'b0; r1b1_op_a <= 51'd0; r1b1_op_bx <= 51'd0;
					r1b1_ab <= 12'd0; r1b1_adj <= 12'd0; r1b1_rm <= 3'd0; r1b1_spec_res <= 32'd0;
				end else if (en) begin
					r1b1_sum0 <= sum_lo_w; r1b1_c0 <= sum_c0_w; r1b1_sr <= s1a_sr; r1b1_rdn <= s1a_rdn;
					r1b1_spec_hit <= s1a_spec_hit; r1b1_op_a <= s1a_op_a[76:26]; r1b1_op_bx <= s1a_op_bx[76:26];
					r1b1_ab <= s1a_ab; r1b1_adj <= s1a_adj; r1b1_rm <= s1a_rm; r1b1_spec_res <= s1a_spec_res;
				end
			end

			wire [25:0] sum_mid_w;
			wire        sum_c1_w;
			am_add #(.W(26)) u_sum1 (.a(r1b1_op_a[25:0]), .b(r1b1_op_bx[25:0]), .ci(r1b1_c0),
				.s(sum_mid_w), .co(sum_c1_w));

			if (LAT >= 10) begin : g_sum_split3
				reg [51:0] r1b2_sum01;
				reg        r1b2_c1, r1b2_sr, r1b2_rdn, r1b2_spec_hit;
				reg [24:0] r1b2_op_a, r1b2_op_bx;
				reg [11:0] r1b2_ab, r1b2_adj;
				reg [2:0]  r1b2_rm;
				reg [31:0] r1b2_spec_res;
				always @(posedge clk or negedge rst_n) begin
					if (!rst_n) begin
						r1b2_sum01 <= 52'd0; r1b2_c1 <= 1'b0; r1b2_sr <= 1'b0; r1b2_rdn <= 1'b0;
						r1b2_spec_hit <= 1'b0; r1b2_op_a <= 25'd0; r1b2_op_bx <= 25'd0;
						r1b2_ab <= 12'd0; r1b2_adj <= 12'd0; r1b2_rm <= 3'd0; r1b2_spec_res <= 32'd0;
					end else if (en) begin
						r1b2_sum01 <= {sum_mid_w, r1b1_sum0}; r1b2_c1 <= sum_c1_w;
						r1b2_sr <= r1b1_sr; r1b2_rdn <= r1b1_rdn; r1b2_spec_hit <= r1b1_spec_hit;
						r1b2_op_a <= r1b1_op_a[50:26]; r1b2_op_bx <= r1b1_op_bx[50:26];
						r1b2_ab <= r1b1_ab; r1b2_adj <= r1b1_adj; r1b2_rm <= r1b1_rm;
						r1b2_spec_res <= r1b1_spec_res;
					end
				end
				wire [24:0] sum_hi_w;
				am_add #(.W(25)) u_sum2 (.a(r1b2_op_a), .b(r1b2_op_bx), .ci(r1b2_c1),
					.s(sum_hi_w), .co(carry_unused[8]));
				assign sum          = {sum_hi_w, r1b2_sum01};
				assign s1c_sr       = r1b2_sr;
				assign s1c_ab       = r1b2_ab;
				assign s1c_adj      = r1b2_adj;
				assign s1c_rm       = r1b2_rm;
				assign s1c_rdn      = r1b2_rdn;
				assign s1c_spec_hit = r1b2_spec_hit;
				assign s1c_spec_res = r1b2_spec_res;
			end else begin : g_sum_split2
				wire [24:0] sum_hi_w;
				am_add #(.W(25)) u_sum2 (.a(r1b1_op_a[50:26]), .b(r1b1_op_bx[50:26]), .ci(sum_c1_w),
					.s(sum_hi_w), .co(carry_unused[8]));
				assign sum          = {sum_hi_w, sum_mid_w, r1b1_sum0};
				assign s1c_sr       = r1b1_sr;
				assign s1c_ab       = r1b1_ab;
				assign s1c_adj      = r1b1_adj;
				assign s1c_rm       = r1b1_rm;
				assign s1c_rdn      = r1b1_rdn;
				assign s1c_spec_hit = r1b1_spec_hit;
				assign s1c_spec_res = r1b1_spec_res;
			end
		end else begin : g_sum_single
			am_add #(.W(77)) u_sum (.a(s1a_op_a), .b(s1a_op_bx), .ci(s1a_ci), .s(sum),
				.co(carry_unused[8]));
			assign s1c_sr       = s1a_sr;
			assign s1c_ab       = s1a_ab;
			assign s1c_adj      = s1a_adj;
			assign s1c_rm       = s1a_rm;
			assign s1c_rdn      = s1a_rdn;
			assign s1c_spec_hit = s1a_spec_hit;
			assign s1c_spec_res = s1a_spec_res;
		end
	endgenerate

	/* --- Stage 1 -> Stage 2 Pipeline Register (LAT >= 1) --- */
	reg [76:0] r1_sum;
	reg        r1_sr;
	reg [11:0] r1_ab;
	reg [11:0] r1_adj;
	reg [2:0]  r1_rm;
	reg        r1_rdn;
	reg        r1_spec_hit;
	reg [31:0] r1_spec_res;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r1_sum      <= 77'd0;
			r1_sr       <= 1'b0;
			r1_ab       <= 12'd0;
			r1_adj      <= 12'd0;
			r1_rm       <= 3'd0;
			r1_rdn      <= 1'b0;
			r1_spec_hit <= 1'b0;
			r1_spec_res <= 32'd0;
		end else if (en) begin
			r1_sum      <= sum;
			r1_sr       <= s1c_sr;
			r1_ab       <= s1c_ab;
			r1_adj      <= s1c_adj;
			r1_rm       <= s1c_rm;
			r1_rdn      <= s1c_rdn;
			r1_spec_hit <= s1c_spec_hit;
			r1_spec_res <= s1c_spec_res;
		end
	end

	/* --- Stage 2: LOD, Normalization --- */
	wire [76:0] s1_sum      = (LAT >= 1) ? r1_sum      : sum;
	wire        s1_sr       = (LAT >= 1) ? r1_sr       : s1c_sr;
	wire [11:0] s1_ab       = (LAT >= 1) ? r1_ab       : s1c_ab;
	wire [11:0] s1_adj      = (LAT >= 1) ? r1_adj      : s1c_adj;
	wire [2:0]  s1_rm       = (LAT >= 1) ? r1_rm       : s1c_rm;
	wire        s1_rdn      = (LAT >= 1) ? r1_rdn      : s1c_rdn;
	wire        s1_spec_hit = (LAT >= 1) ? r1_spec_hit : s1c_spec_hit;
	wire [31:0] s1_spec_res = (LAT >= 1) ? r1_spec_res : s1c_spec_res;

	/* normalised to bit 76; the exponent msb + ea + eb + adj - 175 */
	wire [6:0] msb_c;
	wire sum_nz_c;
	am_msb #(.W(77), .SW(7)) u_msb (.a(s1_sum), .n(msb_c), .nz(sum_nz_c));
	wire [11:0] t1_c;
	am_add #(.W(12)) u_t1 (.a(s1_ab), .b(s1_adj), .ci(1'b0), .s(t1_c), .co(carry_unused[10]));

	/* --- Stage 2a -> Stage 2m Pipeline Register (LAT >= 6) --- */
	reg [76:0] r1m_sum;
	reg [6:0]  r1m_msb;
	reg        r1m_sum_nz, r1m_sr, r1m_rdn, r1m_spec_hit;
	reg [11:0] r1m_t1;
	reg [2:0]  r1m_rm;
	reg [31:0] r1m_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r1m_sum <= 77'd0; r1m_msb <= 7'd0; r1m_sum_nz <= 1'b0; r1m_sr <= 1'b0;
			r1m_rdn <= 1'b0; r1m_spec_hit <= 1'b0; r1m_t1 <= 12'd0; r1m_rm <= 3'd0;
			r1m_spec_res <= 32'd0;
		end else if (en) begin
			r1m_sum <= s1_sum; r1m_msb <= msb_c; r1m_sum_nz <= sum_nz_c; r1m_sr <= s1_sr;
			r1m_rdn <= s1_rdn; r1m_spec_hit <= s1_spec_hit; r1m_t1 <= t1_c; r1m_rm <= s1_rm;
			r1m_spec_res <= s1_spec_res;
		end
	end

	wire [76:0] s1m_sum      = (LAT >= 6) ? r1m_sum      : s1_sum;
	wire [6:0]  msb          = (LAT >= 6) ? r1m_msb      : msb_c;
	wire        sum_nz       = (LAT >= 6) ? r1m_sum_nz   : sum_nz_c;
	wire        s1m_sr       = (LAT >= 6) ? r1m_sr       : s1_sr;
	wire        s1m_rdn      = (LAT >= 6) ? r1m_rdn      : s1_rdn;
	wire        s1m_spec_hit = (LAT >= 6) ? r1m_spec_hit : s1_spec_hit;
	wire [11:0] t1           = (LAT >= 6) ? r1m_t1       : t1_c;
	wire [2:0]  s1m_rm       = (LAT >= 6) ? r1m_rm       : s1_rm;
	wire [31:0] s1m_spec_res = (LAT >= 6) ? r1m_spec_res : s1_spec_res;

	wire [6:0] nsh_c;
	wire [11:0] t2_c;
	am_sub #(.W(7)) u_nsh (.a(7'd76), .b(msb), .bi(1'b0), .d(nsh_c), .bo(carry_unused[9]));
	am_add #(.W(12)) u_t2 (.a(t1), .b({5'd0, msb}), .ci(1'b0), .s(t2_c), .co(carry_unused[11]));

	/* --- Stage 2b0 -> Stage 2b1 Pipeline Register (LAT >= 14) --- */
	reg [76:0] r1b0_sum;
	reg [6:0]  r1b0_nsh;
	reg [11:0] r1b0_t2;
	reg        r1b0_sum_nz, r1b0_sr, r1b0_rdn, r1b0_spec_hit;
	reg [2:0]  r1b0_rm;
	reg [31:0] r1b0_spec_res;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r1b0_sum <= 77'd0; r1b0_nsh <= 7'd0; r1b0_t2 <= 12'd0; r1b0_sum_nz <= 1'b0;
			r1b0_sr <= 1'b0; r1b0_rdn <= 1'b0; r1b0_spec_hit <= 1'b0; r1b0_rm <= 3'd0;
			r1b0_spec_res <= 32'd0;
		end else if (en) begin
			r1b0_sum <= s1m_sum; r1b0_nsh <= nsh_c; r1b0_t2 <= t2_c; r1b0_sum_nz <= sum_nz;
			r1b0_sr <= s1m_sr; r1b0_rdn <= s1m_rdn; r1b0_spec_hit <= s1m_spec_hit; r1b0_rm <= s1m_rm;
			r1b0_spec_res <= s1m_spec_res;
		end
	end

	wire [76:0] s1b0_sum      = (LAT >= 14) ? r1b0_sum      : s1m_sum;
	wire [6:0]  nsh           = (LAT >= 14) ? r1b0_nsh      : nsh_c;
	wire [11:0] t2            = (LAT >= 14) ? r1b0_t2       : t2_c;
	wire        s1b0_sum_nz   = (LAT >= 14) ? r1b0_sum_nz   : sum_nz;
	wire        s1b0_sr       = (LAT >= 14) ? r1b0_sr       : s1m_sr;
	wire        s1b0_rdn      = (LAT >= 14) ? r1b0_rdn      : s1m_rdn;
	wire        s1b0_spec_hit = (LAT >= 14) ? r1b0_spec_hit : s1m_spec_hit;
	wire [2:0]  s1b0_rm       = (LAT >= 14) ? r1b0_rm       : s1m_rm;
	wire [31:0] s1b0_spec_res = (LAT >= 14) ? r1b0_spec_res : s1m_spec_res;

	wire [11:0] er;
	am_sub #(.W(12)) u_er (.a(t2), .b(12'd175), .bi(1'b0), .d(er), .bo(carry_unused[12]));

	/* --- Stage 2a -> Stage 2b Pipeline Register (LAT >= 4) --- */
	reg [76:0] r1b_sum;
	reg [6:0]  r1b_nsh;
	reg [11:0] r1b_er;
	reg        r1b_sum_nz;
	reg        r1b_sr;
	reg [2:0]  r1b_rm;
	reg        r1b_rdn;
	reg        r1b_spec_hit;
	reg [31:0] r1b_spec_res;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r1b_sum      <= 77'd0;
			r1b_nsh      <= 7'd0;
			r1b_er       <= 12'd0;
			r1b_sum_nz   <= 1'b0;
			r1b_sr       <= 1'b0;
			r1b_rm       <= 3'd0;
			r1b_rdn      <= 1'b0;
			r1b_spec_hit <= 1'b0;
			r1b_spec_res <= 32'd0;
		end else if (en) begin
			r1b_sum      <= s1b0_sum;
			r1b_nsh      <= nsh;
			r1b_er       <= er;
			r1b_sum_nz   <= s1b0_sum_nz;
			r1b_sr       <= s1b0_sr;
			r1b_rm       <= s1b0_rm;
			r1b_rdn      <= s1b0_rdn;
			r1b_spec_hit <= s1b0_spec_hit;
			r1b_spec_res <= s1b0_spec_res;
		end
	end

	wire [76:0] s1b_sum      = (LAT >= 4) ? r1b_sum      : s1b0_sum;
	wire [6:0]  s1b_nsh      = (LAT >= 4) ? r1b_nsh      : nsh;
	wire [11:0] s1b_er       = (LAT >= 4) ? r1b_er       : er;
	wire        s1b_sum_nz   = (LAT >= 4) ? r1b_sum_nz   : s1b0_sum_nz;
	wire        s1b_sr       = (LAT >= 4) ? r1b_sr       : s1b0_sr;
	wire [2:0]  s1b_rm       = (LAT >= 4) ? r1b_rm       : s1b0_rm;
	wire        s1b_rdn      = (LAT >= 4) ? r1b_rdn      : s1b0_rdn;
	wire        s1b_spec_hit = (LAT >= 4) ? r1b_spec_hit : s1b0_spec_hit;
	wire [31:0] s1b_spec_res = (LAT >= 4) ? r1b_spec_res : s1b0_spec_res;

	wire [76:0] nrm;
	am_shift #(.W(77), .SW(7), .DIR(2)) u_nrm (.a(s1b_sum), .amt(s1b_nsh), .left(1'b1), .arith(1'b0), .y(nrm));

	/* --- Stage 2 -> Stage 3 Pipeline Register (LAT >= 3) --- */
	reg        r2_sr;
	reg [11:0] r2_er;
	reg [23:0] r2_m;
	reg        r2_g;
	reg [7:0]  r2_st_part;
	reg [2:0]  r2_rm;
	reg        r2_rdn;
	reg        r2_spec_hit;
	reg [31:0] r2_spec_res;
	reg        r2_sum_nz;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r2_sr       <= 1'b0;
			r2_er       <= 12'd0;
			r2_m        <= 24'd0;
			r2_g        <= 1'b0;
			r2_st_part  <= 8'd0;
			r2_rm       <= 3'd0;
			r2_rdn      <= 1'b0;
			r2_spec_hit <= 1'b0;
			r2_spec_res <= 32'd0;
			r2_sum_nz   <= 1'b0;
		end else if (en) begin
			r2_sr       <= s1b_sr;
			r2_er       <= s1b_er;
			r2_m        <= nrm[76:53];
			r2_g        <= nrm[52];
			r2_st_part[0] <= |nrm[6:0];
			r2_st_part[1] <= |nrm[13:7];
			r2_st_part[2] <= |nrm[20:14];
			r2_st_part[3] <= |nrm[27:21];
			r2_st_part[4] <= |nrm[34:28];
			r2_st_part[5] <= |nrm[41:35];
			r2_st_part[6] <= |nrm[48:42];
			r2_st_part[7] <= |nrm[51:49];
			r2_rm       <= s1b_rm;
			r2_rdn      <= s1b_rdn;
			r2_spec_hit <= s1b_spec_hit;
			r2_spec_res <= s1b_spec_res;
			r2_sum_nz   <= s1b_sum_nz;
		end
	end

	/* --- Stage 3: Rounding and Final Result --- */
	wire        s2_sr       = (LAT >= 3) ? r2_sr       : s1_sr;
	wire [11:0] s2_er       = (LAT >= 3) ? r2_er       : er;
	wire [23:0] s2_m        = (LAT >= 3) ? r2_m        : nrm[76:53];
	wire        s2_g        = (LAT >= 3) ? r2_g        : nrm[52];
	wire        s2_st       = (LAT >= 3) ? (|r2_st_part) : |nrm[51:0];
	wire [2:0]  s2_rm       = (LAT >= 3) ? r2_rm       : s1_rm;
	wire        s2_rdn      = (LAT >= 3) ? r2_rdn      : s1_rdn;
	wire        s2_spec_hit = (LAT >= 3) ? r2_spec_hit : s1_spec_hit;
	wire [31:0] s2_spec_res = (LAT >= 3) ? r2_spec_res : s1_spec_res;
	wire        s2_sum_nz   = (LAT >= 3) ? r2_sum_nz   : sum_nz;

	generate
		if (LAT >= 11) begin : g_rnd_pipe
			reg s3_up_w;
			always @(*)
				case (s2_rm)
				3'd1, 3'd5, 3'd6: s3_up_w = 1'b0;
				3'd2: s3_up_w = s2_sr && (s2_g || s2_st);
				3'd3: s3_up_w = !s2_sr && (s2_g || s2_st);
				3'd4: s3_up_w = s2_g;
				default: s3_up_w = s2_g && (s2_st || s2_m[0]);
				endcase
			wire s3_m_carry_w = (&s2_m) && s3_up_w;
			wire s3_lsb_w = s2_m[0] | ((s2_rm == 3'd5 || s2_rm == 3'd6) && (s2_g || s2_st));

			reg        r3a_up, r3a_m_carry, r3a_sr, r3a_rdn, r3a_spec_hit, r3a_sum_nz;
			reg [23:0] r3a_m_in;
			reg [11:0] r3a_er;
			reg [2:0]  r3a_rm;
			reg [31:0] r3a_spec_res;
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					r3a_up <= 1'b0; r3a_m_carry <= 1'b0; r3a_sr <= 1'b0; r3a_rdn <= 1'b0;
					r3a_spec_hit <= 1'b0; r3a_sum_nz <= 1'b0; r3a_m_in <= 24'd0;
					r3a_er <= 12'd0; r3a_rm <= 3'd0; r3a_spec_res <= 32'd0;
				end else if (en) begin
					r3a_up <= s3_up_w; r3a_m_carry <= s3_m_carry_w; r3a_sr <= s2_sr;
					r3a_rdn <= s2_rdn; r3a_spec_hit <= s2_spec_hit; r3a_sum_nz <= s2_sum_nz;
					r3a_m_in <= {s2_m[23:1], s3_lsb_w}; r3a_er <= s2_er; r3a_rm <= s2_rm;
					r3a_spec_res <= s2_spec_res;
				end
			end

			wire [24:0] mr0_3;
			wire signed [11:0] er_3;
			wire [1:0] rnd3_co;
			am_add #(.W(25)) u_mr3 (.a({1'b0, r3a_m_in}), .b(25'd0), .ci(r3a_up), .s(mr0_3), .co(rnd3_co[0]));
			am_add #(.W(12)) u_er3 (.a(r3a_er), .b(12'd0), .ci(r3a_m_carry), .s(er_3), .co(rnd3_co[1]));
			wire [24:0] mr_3_w = r3a_m_carry ? {1'b0, 1'b1, 23'd0} : mr0_3;

			reg [22:0]        r3b_mr;
			reg signed [11:0] r3b_er;
			reg               r3b_sr, r3b_rdn, r3b_spec_hit, r3b_sum_nz;
			reg [2:0]         r3b_rm;
			reg [31:0]        r3b_spec_res;
			always @(posedge clk or negedge rst_n) begin
				if (!rst_n) begin
					r3b_mr <= 23'd0; r3b_er <= 12'sd0; r3b_sr <= 1'b0; r3b_rdn <= 1'b0;
					r3b_spec_hit <= 1'b0; r3b_sum_nz <= 1'b0; r3b_rm <= 3'd0; r3b_spec_res <= 32'd0;
				end else if (en) begin
					r3b_mr <= mr_3_w[22:0]; r3b_er <= er_3; r3b_sr <= r3a_sr; r3b_rdn <= r3a_rdn;
					r3b_spec_hit <= r3a_spec_hit; r3b_sum_nz <= r3a_sum_nz; r3b_rm <= r3a_rm;
					r3b_spec_res <= r3a_spec_res;
				end
			end

			wire [22:0]        s3b_mr       = (LAT >= 15) ? r3b_mr       : mr_3_w[22:0];
			wire signed [11:0] s3b_er       = (LAT >= 15) ? r3b_er       : er_3;
			wire               s3b_sr       = (LAT >= 15) ? r3b_sr       : r3a_sr;
			wire               s3b_rdn      = (LAT >= 15) ? r3b_rdn      : r3a_rdn;
			wire               s3b_spec_hit = (LAT >= 15) ? r3b_spec_hit : r3a_spec_hit;
			wire               s3b_sum_nz   = (LAT >= 15) ? r3b_sum_nz   : r3a_sum_nz;
			wire [2:0]         s3b_rm       = (LAT >= 15) ? r3b_rm       : r3a_rm;
			wire [31:0]        s3b_spec_res = (LAT >= 15) ? r3b_spec_res : r3a_spec_res;

			wire [31:0] rnd_3 = s3b_er >= 12'sd255 ?
				(s3b_rm == 3'd1 || s3b_rm == 3'd5 || s3b_rm == 3'd6 || (s3b_rm == 3'd2 && !s3b_sr) || (s3b_rm == 3'd3 && s3b_sr) ?
					{s3b_sr, 8'hFE, 23'h7FFFFF} : {s3b_sr, 8'hFF, 23'd0}) :
				s3b_er <= 12'sd0 ? {s3b_sr, 31'd0} : {s3b_sr, s3b_er[7:0], s3b_mr};
			wire [31:0] res_3 = s3b_spec_hit ? s3b_spec_res : (!s3b_sum_nz ? {s3b_rdn, 31'd0} : rnd_3);

			if (LAT >= 12) begin : g_out_reg
				reg [31:0] r3_out;
				always @(posedge clk or negedge rst_n) begin
					if (!rst_n)
						r3_out <= 32'd0;
					else if (en)
						r3_out <= res_3;
				end
				assign r = r3_out;
			end else begin : g_out_comb
				assign r = res_3;
			end
			wire unused_rnd3 = &{1'b0, rnd3_co, mr_3_w[24:23]};
		end else begin : g_rnd_comb
			wire [31:0] rnd;
			am_fp_rnd #(.LAT(0)) u_rnd (.s(s2_sr), .e(s2_er), .m(s2_m), .g(s2_g), .st(s2_st), .rm(s2_rm), .r(rnd));

			assign r = s2_spec_hit ? s2_spec_res :
				!s2_sum_nz ? {s2_rdn, 31'd0} : rnd;
		end
	endgenerate

	wire unused_lat0 = (LAT == 0) ? &{1'b0, clk, rst_n, en} : 1'b0;
	wire unused_ok = &{1'b0, kp2[10:7], carry_unused, unused_lat0};
endmodule

`default_nettype wire
