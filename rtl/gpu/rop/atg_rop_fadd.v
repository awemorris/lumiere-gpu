/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_fadd: optimized fp32 adder / subtractor for ROP pipeline (LAT = 4).
 * Dual-path normalisation & rounding architecture compliant with ASIC 5nm 1.5 GHz
 * timing, primitive budget (Pre-CARRY4 <= 1, Post-CARRY4 <= 1, <= 4 LUTs/stage),
 * and fanout limits.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_fadd #(
	parameter integer LAT = 4
) (
	input wire clk,
	input wire rst_n,
	input wire en,
	input wire i_valid,
	input wire [31:0] i_a,
	input wire [31:0] i_b,
	input wire i_sub,
	output wire o_valid,
	output wire [31:0] o_r
);
	wire [8:0]  en_s1 = {9{en}};
	wire [8:0]  en_s2 = {9{en}};
	wire [11:0] en_s3 = {12{en}};
	wire [2:0]  en_s4 = {3{en}};

	/* =========================================================================
	 * Stage 1: Pure-LUT unpack, specials, exponent diff, and magnitude order
	 * (0 CARRY4s, <= 4 LUT levels, fanout <= 10).
	 * ========================================================================= */
	wire b_sign = i_b[31] ^ i_sub;
	wire [31:0] b_in = {b_sign, i_b[30:0]};
	wire [7:0] ea = i_a[30:23];
	wire [7:0] eb = b_in[30:23];

	(* keep = "true" *) wire [3:0] a_mnz, b_mnz;
	assign a_mnz[0] = (i_a[5:0]   != 6'd0);
	assign a_mnz[1] = (i_a[11:6]  != 6'd0);
	assign a_mnz[2] = (i_a[17:12] != 6'd0);
	assign a_mnz[3] = (i_a[22:18] != 5'd0);
	assign b_mnz[0] = (b_in[5:0]   != 6'd0);
	assign b_mnz[1] = (b_in[11:6]  != 6'd0);
	assign b_mnz[2] = (b_in[17:12] != 6'd0);
	assign b_mnz[3] = (b_in[22:18] != 5'd0);

	wire za    = (ea[7:4] == 4'd0) && (ea[3:0] == 4'd0);
	wire zb    = (eb[7:4] == 4'd0) && (eb[3:0] == 4'd0);
	wire ea_ff = (&ea[7:4]) && (&ea[3:0]);
	wire eb_ff = (&eb[7:4]) && (&eb[3:0]);
	wire nan_a = ea_ff && (|a_mnz);
	wire nan_b = eb_ff && (|b_mnz);
	wire inf_a = ea_ff && !(|a_mnz);
	wire inf_b = eb_ff && !(|b_mnz);

	genvar gc;
	wire [7:0] dab, dba;
	wire dab_bo_unused, dba_bo_unused;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(8)) u_sub_dab (.a(ea), .b(eb), .bi(1'b0), .d(dab), .bo(dab_bo_unused));
	am_sub #(.W(8)) u_sub_dba (.a(eb), .b(ea), .bi(1'b0), .d(dba), .bo(dba_bo_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(8)) u_sub_dab (.a(ea), .b(eb), .bi(1'b0), .d(dab), .bo(dab_bo_unused));
	am_sub #(.W(8)) u_sub_dba (.a(eb), .b(ea), .bi(1'b0), .d(dba), .bo(dba_bo_unused));
`else
	wire [3:0] dab_lo, dab_h0, dab_h1, dba_lo, dba_h0, dba_h1;
	wire [3:0] dab_bg_l = (~ea[3:0]) & eb[3:0];
	wire [3:0] dab_bp_l = (~ea[3:0]) | eb[3:0];
	wire [1:0] dab_g2_l = {dab_bg_l[3] | (dab_bp_l[3] & dab_bg_l[2]), dab_bg_l[1] | (dab_bp_l[1] & dab_bg_l[0])};
	wire [1:0] dab_p2_l = {dab_bp_l[3] & dab_bp_l[2], dab_bp_l[1] & dab_bp_l[0]};
	wire dab_b4 = dab_g2_l[1] | (dab_p2_l[1] & dab_g2_l[0]);

	wire [3:0] dba_bg_l = (~eb[3:0]) & ea[3:0];
	wire [3:0] dba_bp_l = (~eb[3:0]) | ea[3:0];
	wire [1:0] dba_g2_l = {dba_bg_l[3] | (dba_bp_l[3] & dba_bg_l[2]), dba_bg_l[1] | (dba_bp_l[1] & dba_bg_l[0])};
	wire [1:0] dba_p2_l = {dba_bp_l[3] & dba_bp_l[2], dba_bp_l[1] & dba_bp_l[0]};
	wire dba_b4 = dba_g2_l[1] | (dba_p2_l[1] & dba_g2_l[0]);

	wire [3:0] dab_bg_h = (~ea[7:4]) & eb[7:4];
	wire [3:0] dab_bp_h = (~ea[7:4]) | eb[7:4];
	wire [1:0] dab_g2_h = {dab_bg_h[3] | (dab_bp_h[3] & dab_bg_h[2]), dab_bg_h[1] | (dab_bp_h[1] & dab_bg_h[0])};
	wire [1:0] dab_p2_h = {dab_bp_h[3] & dab_bp_h[2], dab_bp_h[1] & dab_bp_h[0]};
	wire dab_b8_0 = dab_g2_h[1] | (dab_p2_h[1] & dab_g2_h[0]);
	wire dab_b8_1 = dab_g2_h[1] | (dab_p2_h[1] & (dab_g2_h[0] | dab_p2_h[0]));

	wire dab_b4_unused, dba_b4_unused, dab_u0, dab_u1, dba_u0, dba_u1;
	am_sub #(.W(4)) u_sub_dab_l  (.a(ea[3:0]), .b(eb[3:0]), .bi(1'b0), .d(dab_lo), .bo(dab_b4_unused));
	am_sub #(.W(4)) u_sub_dab_h0 (.a(ea[7:4]), .b(eb[7:4]), .bi(1'b0), .d(dab_h0), .bo(dab_u0));
	am_sub #(.W(4)) u_sub_dab_h1 (.a(ea[7:4]), .b(eb[7:4]), .bi(1'b1), .d(dab_h1), .bo(dab_u1));
	am_sub #(.W(4)) u_sub_dba_l  (.a(eb[3:0]), .b(ea[3:0]), .bi(1'b0), .d(dba_lo), .bo(dba_b4_unused));
	am_sub #(.W(4)) u_sub_dba_h0 (.a(eb[7:4]), .b(ea[7:4]), .bi(1'b0), .d(dba_h0), .bo(dba_u0));
	am_sub #(.W(4)) u_sub_dba_h1 (.a(eb[7:4]), .b(ea[7:4]), .bi(1'b1), .d(dba_h1), .bo(dba_u1));
	assign dab = {dab_b4 ? dab_h1 : dab_h0, dab_lo};
	assign dba = {dba_b4 ? dba_h1 : dba_h0, dba_lo};
	assign dab_bo_unused = (dab_b4 ? dab_b8_1 : dab_b8_0) | (1'b0 & (dab_b4_unused | dba_b4_unused | dab_u0 | dab_u1));
	assign dba_bo_unused = dba_b4 ? dba_u1 : dba_u0;
`endif
	wire ea_lt_eb = dab_bo_unused;

	/* Pure-LUT 31-bit magnitude comparison (Level 1..3 -> folded into Level 4 muxes) */
	(* keep = "true" *) wire [10:0] lt0, eq0;
	generate
	for (gc = 0; gc < 10; gc = gc + 1) begin : g_cmp_s0
		wire [2:0] sa = i_a[3 * gc +: 3];
		wire [2:0] sb = i_b[3 * gc +: 3];
		assign eq0[gc] = (sa == sb);
		assign lt0[gc] = (~sa[2] & sb[2]) |
			((sa[2] ~^ sb[2]) & ((~sa[1] & sb[1]) | ((sa[1] ~^ sb[1]) & (~sa[0] & sb[0]))));
	end
	endgenerate
	assign eq0[10] = (i_a[30] == i_b[30]);
	assign lt0[10] = (~i_a[30] & i_b[30]);

	(* keep = "true" *) wire [3:0] lt1, eq1;
	generate
	for (gc = 0; gc < 3; gc = gc + 1) begin : g_cmp_s1
		assign eq1[gc] = eq0[3 * gc + 2] & eq0[3 * gc + 1] & eq0[3 * gc];
		assign lt1[gc] = lt0[3 * gc + 2] | (eq0[3 * gc + 2] & lt0[3 * gc + 1]) |
			(eq0[3 * gc + 2] & eq0[3 * gc + 1] & lt0[3 * gc]);
	end
	endgenerate
	assign eq1[3] = eq0[10] & eq0[9];
	assign lt1[3] = lt0[10] | (eq0[10] & lt0[9]);

	/* Replicated Level-3 comparator terms (each <= 5 inputs in Level 3, fanout <= 9) */
	(* keep = "true", dont_touch = "true" *) wire [6:0] lt_hi_r, lt_lo_r;
	generate
	for (gc = 0; gc < 7; gc = gc + 1) begin : g_cmp_s2_rep
		assign lt_hi_r[gc] = lt1[3] | (eq1[3] & lt1[2]);
		assign lt_lo_r[gc] = eq1[3] & eq1[2] & (lt1[1] | (eq1[1] & lt1[0]));
	end
	endgenerate

	function [3:0] fadd_cla3;
		input [2:0] a, b;
		input ci;
		reg [2:0] p, g, c;
		begin
			p = a ^ b;
			g = a & b;
			c[0] = ci;
			c[1] = g[0] | (p[0] & ci);
			c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & ci);
			fadd_cla3 = {
				g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & ci),
				p ^ c
			};
		end
	endfunction

	function [3:0] fadd_sub3;
		input [2:0] a, b;
		input bi;
		reg [2:0] nb, p, g, c;
		begin
			nb = ~b;
			p = a ^ nb;
			g = a & nb;
			c[0] = ~bi;
			c[1] = g[0] | (p[0] & c[0]);
			c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & c[0]);
			fadd_sub3 = {
				~(g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & c[0])),
				p ^ c
			};
		end
	endfunction

	wire v1_w       = i_valid;
	wire sp_nan1_w  = nan_a || nan_b || (inf_a && inf_b && (i_a[31] != b_sign));
	wire sp_inf1_w  = !sp_nan1_w && (inf_a || inf_b);
	wire sp_zero1_w = !sp_nan1_w && !sp_inf1_w && (za && zb);
	wire sp1_w      = sp_nan1_w || sp_inf1_w || sp_zero1_w;

	(* keep = "true", dont_touch = "true" *) wire sx_norm1_w = (lt_hi_r[6] | lt_lo_r[6]) ? b_sign : i_a[31];
	wire sp_sx1_w   = !sp_nan1_w && (inf_a ? i_a[31] : inf_b ? b_sign : (i_a[31] && b_sign));
	wire sub1_w     = (i_a[31] != b_sign);
	wire y_zero1_w  = za || zb;

	(* keep = "true", dont_touch = "true" *) wire [7:0] ex1_w;
	(* keep = "true", dont_touch = "true" *) wire [23:0] mx1_w, my1_w;
	assign mx1_w[23] = 1'b1;
	assign my1_w[23] = 1'b1;
	generate
	for (gc = 0; gc < 8; gc = gc + 1) begin : g_ex1_mux
		assign ex1_w[gc] = (lt_hi_r[6] | lt_lo_r[6]) ? eb[gc] : ea[gc];
	end
	for (gc = 0; gc < 23; gc = gc + 1) begin : g_m1_mux
		localparam integer R_MX = gc / 8;
		localparam integer R_MY = 3 + (gc / 8);
		assign mx1_w[gc] = (lt_hi_r[R_MX] | lt_lo_r[R_MX]) ? b_in[gc] : i_a[gc];
		assign my1_w[gc] = (lt_hi_r[R_MY] | lt_lo_r[R_MY]) ? i_a[gc] : b_in[gc];
	end
	endgenerate

	/* Direct 5-bit difference + [7:5] in-range check for d1_coarse_oh_w (<= 10 cmos3, 0 extra FO on ea/eb) */
	wire d_b2_ab = (ea[1:0] < eb[1:0]);
	wire d_b2_ba = (eb[1:0] < ea[1:0]);
	wire d_b5_ab = (ea[4:0] < eb[4:0]);
	wire d_b5_ba = (eb[4:0] < ea[4:0]);
	wire [2:0] ea_hi3_p1 = ea[7:5] ^ 3'b001 ^ {ea[5] & ea[6] ? 1'b1 : 1'b0, ea[5], 1'b0};
	wire [2:0] eb_hi3_p1 = eb[7:5] ^ 3'b001 ^ {eb[5] & eb[6] ? 1'b1 : 1'b0, eb[5], 1'b0};
	wire hi3_eq  = (ea[7:5] == eb[7:5]);
	wire hi3_ab1 = (ea[7:5] == eb_hi3_p1);
	wire hi3_ba1 = (eb[7:5] == ea_hi3_p1);
	wire dab_in32 = !y_zero1_w && ((hi3_eq && !d_b5_ab) || (hi3_ab1 && d_b5_ab));
	wire dba_in32 = !y_zero1_w && ((hi3_eq &&  d_b5_ab) || (hi3_ba1 && !d_b5_ab));
	wire [3:0] d42_dab_4 = fadd_sub3(ea[4:2], eb[4:2], d_b2_ab);
	wire [3:0] d42_dba_4 = fadd_sub3(eb[4:2], ea[4:2], d_b2_ba);
	wire [2:0] d42_dab_w = d42_dab_4[2:0];
	wire [2:0] d42_dba_w = d42_dba_4[2:0];
`ifdef SYNTH_KINTEX7
	wire [4:0] d1_w = ea_lt_eb ? dba[4:0] : dab[4:0];
	wire [3:0] dab_hi_w = dab[7:4];
	wire [3:0] dba_hi_w = dba[7:4];
	wire unused_dab_hi = (&dab_hi_w) & (&dba_hi_w) & d42_dab_4[3] & d42_dba_4[3];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [4:0] d1_w = ea_lt_eb ? dba[4:0] : dab[4:0];
	wire [3:0] dab_hi_w = dab[7:4];
	wire [3:0] dba_hi_w = dba[7:4];
	wire unused_dab_hi = (&dab_hi_w) & (&dba_hi_w) & d42_dab_4[3] & d42_dba_4[3];
`else
	wire [4:0] d1_w = ea_lt_eb ? {d42_dba_w, dba_lo[1:0]} : {d42_dab_w, dab_lo[1:0]};
	wire [3:0] dab_hi_w = dab_b4 ? dab_h1 : dab_h0;
	wire [3:0] dba_hi_w = dba_b4 ? dba_h1 : dba_h0;
	wire unused_dab_hi = (&dab_hi_w) & (&dba_hi_w) & d42_dab_4[3] & d42_dba_4[3];
`endif
	wire [6:0] d1_coarse_oh_w;
	genvar gco;
	generate
	for (gco = 0; gco < 7; gco = gco + 1) begin : g_d1_coh
		assign d1_coarse_oh_w[gco] =
			(dab_in32 && (d42_dab_w == gco[2:0])) ||
			(dba_in32 && (d42_dba_w == gco[2:0]));
	end
	endgenerate
	wire [3:0] d1_lo_dab   = { dab[1] &  dab[0],  dab[1] & ~dab[0], ~dab[1] &  dab[0], ~dab[1] & ~dab[0]};
	wire [3:0] d1_lo_dba   = { dba[1] &  dba[0],  dba[1] & ~dba[0], ~dba[1] &  dba[0], ~dba[1] & ~dba[0]};
	wire [3:0] d1_mid_dab  = { dab[3] &  dab[2],  dab[3] & ~dab[2], ~dab[3] &  dab[2], ~dab[3] & ~dab[2]};
	wire [3:0] d1_mid_dba  = { dba[3] &  dba[2],  dba[3] & ~dba[2], ~dba[3] &  dba[2], ~dba[3] & ~dba[2]};
	wire [3:0] d1_lo_oh_w  = ea_lt_eb ? d1_lo_dba  : d1_lo_dab;
	wire [3:0] d1_mid_oh_w = ea_lt_eb ? d1_mid_dba : d1_mid_dab;
	wire d1_bit0_w  = ea[0] ^ eb[0];
	wire ealtb1_w   = ea_lt_eb;
	wire dab_hi1_w  = (dab[7:5] != 3'd0);
	wire dba_hi1_w  = (dba[7:5] != 3'd0);
	wire dab_cc1_w  = (dab[4:1] == 4'd0);
	wire dba_cc1_w  = (dba[4:1] == 4'd0);
	wire cc_en1_w   = (i_a[31] != b_sign) && !za && !zb;

	(* keep = "true" *) reg r_v1, r_sp1, r_sp_nan1, r_sp_inf1, r_sx_norm1, r_sp_sx1;
	(* keep = "true" *) reg r_ealtb1, r_dab_hi1, r_dba_hi1, r_dab_cc1, r_dba_cc1, r_cc_en1;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg r_y_zero1_a, r_y_zero1_b;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg r_sub1_a, r_sub1_b, r_sub1_c;
	reg [7:0] r_ex1;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg [4:0] r_d1_a, r_d1_b, r_d1_c, r_d1_d;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg [3:0] r_d1_lo_oh_a, r_d1_lo_oh_b, r_d1_mid_oh_a, r_d1_mid_oh_b;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg [6:0] r_d1_coarse_oh_a, r_d1_coarse_oh_b;
	(* equivalent_register_removal = "no", max_fanout = 14 *) reg r_d1_e0, r_d1_f0;
	(* keep = "true", dont_touch = "true" *) reg [2:0] r_d1_e0_rep;
	reg [23:0] r_mx1, r_my1;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_v1 <= 1'b0;
		end else if (en_s1[0]) begin
			r_v1 <= v1_w;
		end
	end

	always @(posedge clk) begin
		if (en_s1[0]) begin
			r_sp1       <= sp1_w;
			r_sp_nan1   <= sp_nan1_w;
			r_sp_inf1   <= sp_inf1_w;
			r_sx_norm1  <= sx_norm1_w;
			r_sp_sx1    <= sp_sx1_w;
			r_sub1_a    <= sub1_w;
			r_sub1_b    <= sub1_w;
			r_sub1_c    <= sub1_w;
		end
		if (en_s1[1]) begin
			r_y_zero1_a <= y_zero1_w;
			r_y_zero1_b <= y_zero1_w;
			r_ealtb1    <= ealtb1_w;
			r_dab_hi1   <= dab_hi1_w;
			r_dba_hi1   <= dba_hi1_w;
			r_dab_cc1   <= dab_cc1_w;
			r_dba_cc1   <= dba_cc1_w;
			r_cc_en1    <= cc_en1_w;
			r_d1_e0     <= d1_bit0_w;
			r_d1_e0_rep <= {3{d1_bit0_w}};
			r_d1_f0     <= d1_bit0_w;
		end
		if (en_s1[2]) begin
			r_ex1            <= ex1_w;
			r_d1_a           <= d1_w;
			r_d1_lo_oh_a     <= d1_lo_oh_w;
			r_d1_mid_oh_a    <= d1_mid_oh_w;
			r_d1_coarse_oh_a <= d1_coarse_oh_w;
		end
		if (en_s1[3]) begin
			r_d1_b           <= d1_w;
			r_d1_c           <= d1_w;
			r_d1_lo_oh_b     <= d1_lo_oh_w;
			r_d1_mid_oh_b    <= d1_mid_oh_w;
			r_d1_coarse_oh_b <= d1_coarse_oh_w;
		end
		if (en_s1[4]) begin
			r_d1_d       <= d1_w;
			r_mx1[6:0]   <= mx1_w[6:0];
		end
		if (en_s1[5]) r_mx1[18:7]  <= mx1_w[18:7];
		if (en_s1[6]) begin
			r_mx1[23:19] <= mx1_w[23:19];
			r_my1[6:0]   <= my1_w[6:0];
		end
		if (en_s1[7]) r_my1[18:7]  <= my1_w[18:7];
		if (en_s1[8]) r_my1[23:19] <= my1_w[23:19];
	end

	wire v1          = r_v1;
	wire sp1         = r_sp1;
	wire sp_nan1     = r_sp_nan1;
	wire sp_inf1     = r_sp_inf1;
	wire sx1         = r_sp1 ? r_sp_sx1 : r_sx_norm1;
	wire sub1_a      = r_sub1_a;
	wire sub1_b      = r_sub1_b;
	wire sub1_c      = r_sub1_c;
	wire y_zero1_a   = r_y_zero1_a;
	wire y_zero1_b   = r_y_zero1_b;
	wire ealtb1      = r_ealtb1;
	wire dab_hi1     = r_dab_hi1;
	wire dba_hi1     = r_dba_hi1;
	wire close_cand1 = r_cc_en1 && (r_ealtb1 ? (!r_dba_hi1 && r_dba_cc1) : (!r_dab_hi1 && r_dab_cc1));
	wire [7:0] ex1   = r_ex1;
	wire [23:0] mx1  = r_mx1;
	wire [23:0] my1  = r_my1;
	wire [4:0] d1_a  = r_d1_a;
	wire [4:0] d1_b  = r_d1_b;
	wire [4:0] d1_c  = r_d1_c;
	wire [4:0] d1_d  = r_d1_d;
	wire [3:0] d1_lo_oh_a  = r_d1_lo_oh_a;
	wire [3:0] d1_lo_oh_b  = r_d1_lo_oh_b;
	wire [3:0] d1_mid_oh_a = r_d1_mid_oh_a;
	wire [3:0] d1_mid_oh_b = r_d1_mid_oh_b;
	wire [6:0] d1_coarse_oh_a = r_d1_coarse_oh_a;
	wire [6:0] d1_coarse_oh_b = r_d1_coarse_oh_b;
	wire d1_e0       = r_d1_e0;
	wire [2:0] d1_e0_rep = r_d1_e0_rep;
	wire d1_f0       = r_d1_f0;

	/* =========================================================================
	 * Stage 2:
	 * - Far path: align smaller operand, 3-level sticky bit, and XOR with sub1.
	 * - Close path: dual 25-bit exact subtractors directly from FFs (Pre-LUTs = 1,
	 *   Post-LUTs = 1).
	 * - Pure-LUT ex1 + 1 and ex1 - 1 (0 CARRY4s).
	 * ========================================================================= */
	wire [26:0] shi = {my1, 3'b000};
	wire [26:0] shr_0, shr_1, shr_2;
`ifdef SYNTH_KINTEX7
	am_shift #(.W(27), .SW(5)) u_shr_0 (.a(shi), .amt(d1_a), .left(1'b0), .arith(1'b0), .y(shr_0));
	am_shift #(.W(27), .SW(5)) u_shr_1 (.a(shi), .amt(d1_d), .left(1'b0), .arith(1'b0), .y(shr_1));
	am_shift #(.W(27), .SW(5)) u_shr_2 (.a(shi), .amt(d1_b), .left(1'b0), .arith(1'b0), .y(shr_2));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_shift #(.W(27), .SW(5)) u_shr_0 (.a(shi), .amt(d1_a), .left(1'b0), .arith(1'b0), .y(shr_0));
	am_shift #(.W(27), .SW(5)) u_shr_1 (.a(shi), .amt(d1_d), .left(1'b0), .arith(1'b0), .y(shr_1));
	am_shift #(.W(27), .SW(5)) u_shr_2 (.a(shi), .amt(d1_b), .left(1'b0), .arith(1'b0), .y(shr_2));
`else
	wire [29:0] shi_pad = {3'b000, shi};
	wire [26:0] shr_s0_w, shr_s1_w, shr_w;
	genvar gb;
	generate
	for (gb = 0; gb < 27; gb = gb + 1) begin : g_shr_s0
		wire [3:0] loh = (gb < 14) ? d1_lo_oh_a : d1_lo_oh_b;
		assign shr_s0_w[gb] = (loh[0] & shi_pad[gb])     |
		                      (loh[1] & shi_pad[gb + 1]) |
		                      (loh[2] & shi_pad[gb + 2]) |
		                      (loh[3] & shi_pad[gb + 3]);
	end
	for (gb = 0; gb < 27; gb = gb + 1) begin : g_shr_s1
		wire [6:0] coh = (gb < 14) ? d1_coarse_oh_a : d1_coarse_oh_b;
		wire [6:0] ct;
		genvar gc7;
		for (gc7 = 0; gc7 < 7; gc7 = gc7 + 1) begin : g_c7
			if (gb + 4*gc7 < 27) begin : g_in
				assign ct[gc7] = coh[gc7] & shr_s0_w[gb + 4*gc7];
			end else begin : g_z
				assign ct[gc7] = 1'b0;
			end
		end
		assign shr_w[gb] = |ct;
		assign shr_s1_w[gb] = 1'b0;
	end
	endgenerate
	assign shr_0 = shr_w;
	assign shr_1 = shr_w;
	assign shr_2 = shr_w;
`endif

	(* keep = "true", dont_touch = "true" *) wire [5:0] st_full, st_part;
	genvar gs;
	generate
	for (gs = 0; gs < 6; gs = gs + 1) begin : g_stk_chunk
		wire [3:0] ch = my1[4 * gs +: 4];
		assign st_full[gs] = |ch;
		assign st_part[gs] = ch[0] || ((!d1_c[1] || !d1_c[0]) && ch[1]) || ((d1_c[1] ^ d1_c[0]) && ch[2]) || ((d1_c[1] && !d1_c[0]) && ch[3]);
	end
	endgenerate

	/* 3-LUT-level sticky reduction (Level 1: st_full/st_part/st_mode -> Level 2: stk_pair -> Level 3: py2_w[0]) */
	wire d1_ge3  = d1_c[4] | d1_c[3] | d1_c[2] | (d1_c[1] & d1_c[0]);
	wire d1_ge7  = d1_c[4] | d1_c[3] | (&d1_c[2:0]);
	wire d1_ge11 = d1_c[4] | (d1_c[3] & (d1_c[2] | (d1_c[1] & d1_c[0])));
	wire d1_ge15 = d1_c[4] | (&d1_c[3:0]);
	wire d1_ge19 = d1_c[4] & (d1_c[3] | d1_c[2] | (d1_c[1] & d1_c[0]));
	wire d1_ge23 = d1_c[4] & (d1_c[3] | (&d1_c[2:0]));
	wire d1_ge27 = d1_c[4] & d1_c[3] & (d1_c[2] | (d1_c[1] & d1_c[0]));

	(* keep = "true", dont_touch = "true" *) wire [2:0] stk_pair;
	assign stk_pair[0] = ((d1_ge3 & !d1_ge7)  & st_part[0]) | ((d1_ge7  & !d1_ge11) & (st_full[0] | st_part[1])) | (d1_ge11 & (st_full[0] | st_full[1]));
	assign stk_pair[1] = ((d1_ge11 & !d1_ge15) & st_part[2]) | ((d1_ge15 & !d1_ge19) & (st_full[2] | st_part[3])) | (d1_ge19 & (st_full[2] | st_full[3]));
	assign stk_pair[2] = ((d1_ge19 & !d1_ge23) & st_part[4]) | ((d1_ge23 & !d1_ge27) & (st_full[4] | st_part[5])) | (d1_ge27 & (st_full[4] | st_full[5]));

	(* keep = "true", dont_touch = "true" *) wire clr_sel_0 = ealtb1 ? dba_hi1 : dab_hi1;
	(* keep = "true", dont_touch = "true" *) wire [3:0] y_clr_rep;
	assign y_clr_rep[0] = y_zero1_a || (ealtb1 ? dba_hi1 : dab_hi1);
	assign y_clr_rep[1] = y_zero1_a || (ealtb1 ? dba_hi1 : dab_hi1);
	assign y_clr_rep[2] = y_zero1_b || (ealtb1 ? dba_hi1 : dab_hi1);
	assign y_clr_rep[3] = y_zero1_b || (ealtb1 ? dba_hi1 : dab_hi1);

	wire stk_p01 = clr_sel_0 || stk_pair[0];
	wire stk_p2z = stk_pair[1] || stk_pair[2];
	wire py2_bit0 = !y_zero1_a && (stk_p01 || stk_p2z);
	wire [13:0] py2_raw_lo = {
		y_clr_rep[1] ? 6'd0 : shr_1[13:8],
		y_clr_rep[0] ? 7'd0 : shr_0[7:1],
		py2_bit0
	};
	wire [26:14] py2_raw_hi = {
		y_clr_rep[3] ? 7'd0 : shr_2[26:20],
		y_clr_rep[2] ? 6'd0 : {shr_2[19:15], shr_1[14]}
	};
	wire [26:0] py2_raw = {py2_raw_hi, py2_raw_lo};

	/* Close path subtractors: 13+12 carry-select + d1_f0 pre-mux on [24:13] (<= 11 cmos3) */
	wire [24:0] close_diff_d0, close_diff_d1;
	wire [24:0] close_diff2_w;
`ifdef SYNTH_KINTEX7
	wire close_bo0_unused, close_bo1_unused;
	am_sub #(.W(25)) u_close_sub_d0 (
		.a({mx1, 1'b0}), .b({my1, 1'b0}), .bi(1'b0), .d(close_diff_d0), .bo(close_bo0_unused)
	);
	am_sub #(.W(25)) u_close_sub_d1 (
		.a({mx1, 1'b0}), .b({1'b0, my1}), .bi(1'b0), .d(close_diff_d1), .bo(close_bo1_unused)
	);
	assign close_diff2_w = {
		d1_f0 ? close_diff_d1[24:13] : close_diff_d0[24:13],
		d1_e0 ? close_diff_d1[12:0]  : close_diff_d0[12:0]
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire close_bo0_unused, close_bo1_unused;
	am_sub #(.W(25)) u_close_sub_d0 (
		.a({mx1, 1'b0}), .b({my1, 1'b0}), .bi(1'b0), .d(close_diff_d0), .bo(close_bo0_unused)
	);
	am_sub #(.W(25)) u_close_sub_d1 (
		.a({mx1, 1'b0}), .b({1'b0, my1}), .bi(1'b0), .d(close_diff_d1), .bo(close_bo1_unused)
	);
	assign close_diff2_w = {
		d1_f0 ? close_diff_d1[24:13] : close_diff_d0[24:13],
		d1_e0 ? close_diff_d1[12:0]  : close_diff_d0[12:0]
	};
`else
	/* 7+6+6+6 fast borrow-lookahead close subtractor with 3-bit fadd_sub3 blocks (<= 9 cmos3) */
	wire [24:0] cb_eff;
	assign cb_eff[0] = d1_e0_rep[0] & my1[0];
	genvar gcb;
	generate
	for (gcb = 1; gcb < 9; gcb = gcb + 1) begin : g_cbe_lo
		assign cb_eff[gcb] = d1_e0_rep[0] ? my1[gcb] : my1[gcb - 1];
	end
	for (gcb = 9; gcb < 17; gcb = gcb + 1) begin : g_cbe_mid
		assign cb_eff[gcb] = d1_e0_rep[1] ? my1[gcb] : my1[gcb - 1];
	end
	for (gcb = 17; gcb < 24; gcb = gcb + 1) begin : g_cbe_hi
		assign cb_eff[gcb] = d1_e0_rep[2] ? my1[gcb] : my1[gcb - 1];
	end
	endgenerate
	assign cb_eff[24] = d1_e0_rep[2] ? 1'b0 : my1[23];
	wire [24:0] ca_eff = {mx1, 1'b0};
	wire [24:0] cbg_i = (~ca_eff) & cb_eff;
	wire [24:0] cbp_i = ~(ca_eff ^ cb_eff);
	wire [2:0] cbg3_0, cbp3_0;
	wire [1:0] cbg3_1, cbp3_1;
	wire cbg3_2, cbp3_2;
	/* Borrows at bit 7, 13, 19 via 3-bit/4-bit block lookahead */
	wire cb_g4_0 = cbg_i[3] | (cbp_i[3] & cbg_i[2]) | (cbp_i[3] & cbp_i[2] & cbg_i[1]) | (cbp_i[3] & cbp_i[2] & cbp_i[1] & cbg_i[0]);
	wire cb_p4_0 = &cbp_i[3:0];
	wire cb_g3_1 = cbg_i[6] | (cbp_i[6] & cbg_i[5]) | (cbp_i[6] & cbp_i[5] & cbg_i[4]);
	wire cb_p3_1 = &cbp_i[6:4];
	wire cb_b7   = cb_g3_1 | (cb_p3_1 & cb_g4_0);
	assign cbg3_0 = {cbg_i[9]  | (cbp_i[9]  & cbg_i[8])  | (cbp_i[9]  & cbp_i[8]  & cbg_i[7]),  cbg_i[12] | (cbp_i[12] & cbg_i[11]) | (cbp_i[12] & cbp_i[11] & cbg_i[10]), 1'b0};
	assign cbp3_0 = {&cbp_i[9:7], &cbp_i[12:10], 1'b0};
	wire cb_g6_1 = cbg3_0[1] | (cbp3_0[1] & cbg3_0[2]);
	wire cb_p6_1 = cbp3_0[1] & cbp3_0[2];
	assign cbg3_1 = {cbg_i[18] | (cbp_i[18] & cbg_i[17]) | (cbp_i[18] & cbp_i[17] & cbg_i[16]), cbg_i[15] | (cbp_i[15] & cbg_i[14]) | (cbp_i[15] & cbp_i[14] & cbg_i[13])};
	assign cbp3_1 = {&cbp_i[18:16], &cbp_i[15:13]};
	wire cb_g6_2 = cbg3_1[1] | (cbp3_1[1] & cbg3_1[0]);
	wire cb_p6_2 = cbp3_1[1] & cbp3_1[0];
	wire cb_b13  = cb_g6_1 | (cb_p6_1 & cb_b7);
	wire cb_b19  = cb_g6_2 | (cb_p6_2 & cb_g6_1) | (cb_p6_2 & cb_p6_1 & cb_b7);
	assign cbg3_2 = 1'b0; assign cbp3_2 = 1'b0;

	wire cdl_b0  = (~ca_eff[0]) & cb_eff[0];
	wire cdl_s0  = ca_eff[0] ^ cb_eff[0];
	wire [3:0] cdl_31_4 = fadd_sub3(ca_eff[3:1], cb_eff[3:1], cdl_b0);
	wire [3:0] cdl_64_4 = fadd_sub3(ca_eff[6:4], cb_eff[6:4], cdl_31_4[3]);
	wire [6:0] cdl_s = {cdl_64_4[2:0], cdl_31_4[2:0], cdl_s0};

	wire [3:0] cdm_0_l = fadd_sub3(ca_eff[9:7],   cb_eff[9:7],   1'b0);
	wire [3:0] cdm_1_l = fadd_sub3(ca_eff[9:7],   cb_eff[9:7],   1'b1);
	wire [3:0] cdm_0_h = fadd_sub3(ca_eff[12:10], cb_eff[12:10], cdm_0_l[3]);
	wire [3:0] cdm_1_h = fadd_sub3(ca_eff[12:10], cb_eff[12:10], cdm_1_l[3]);
	wire [5:0] cdm_0   = {cdm_0_h[2:0], cdm_0_l[2:0]};
	wire [5:0] cdm_1   = {cdm_1_h[2:0], cdm_1_l[2:0]};

	wire [3:0] cdh2_0_l = fadd_sub3(ca_eff[15:13], cb_eff[15:13], 1'b0);
	wire [3:0] cdh2_1_l = fadd_sub3(ca_eff[15:13], cb_eff[15:13], 1'b1);
	wire [3:0] cdh2_0_h = fadd_sub3(ca_eff[18:16], cb_eff[18:16], cdh2_0_l[3]);
	wire [3:0] cdh2_1_h = fadd_sub3(ca_eff[18:16], cb_eff[18:16], cdh2_1_l[3]);
	wire [5:0] cdh2_0   = {cdh2_0_h[2:0], cdh2_0_l[2:0]};
	wire [5:0] cdh2_1   = {cdh2_1_h[2:0], cdh2_1_l[2:0]};

	wire [3:0] cdh3_0_l = fadd_sub3(ca_eff[21:19], cb_eff[21:19], 1'b0);
	wire [3:0] cdh3_1_l = fadd_sub3(ca_eff[21:19], cb_eff[21:19], 1'b1);
	wire [3:0] cdh3_0_h = fadd_sub3(ca_eff[24:22], cb_eff[24:22], cdh3_0_l[3]);
	wire [3:0] cdh3_1_h = fadd_sub3(ca_eff[24:22], cb_eff[24:22], cdh3_1_l[3]);
	wire [5:0] cdh3_0   = {cdh3_0_h[2:0], cdh3_0_l[2:0]};
	wire [5:0] cdh3_1   = {cdh3_1_h[2:0], cdh3_1_l[2:0]};

	assign close_diff_d0 = 25'd0;
	assign close_diff_d1 = 25'd0;
	assign close_diff2_w = {
		cb_b19 ? cdh3_1 : cdh3_0,
		cb_b13 ? cdh2_1 : cdh2_0,
		cb_b7  ? cdm_1  : cdm_0,
		cdl_s
	};
	wire unused_cd_bo = &{1'b0, d1_e0, cdl_64_4[3], cdm_0_h[3], cdm_1_h[3], cdh2_0_h[3], cdh2_1_h[3], cdh3_0_h[3], cdh3_1_h[3], cb_p4_0, cbg3_0[0], cbp3_0[0], cbg3_2, cbp3_2, d1_f0};
`endif

	wire [7:0] ex2_p1_w, ex2_sub1_w;
	wire ex2_p1_co_unused, ex2_sub1_bo_unused;
	am_add #(.W(8)) u_ex2_p1 (.a(ex1), .b(8'd1), .ci(1'b0), .s(ex2_p1_w), .co(ex2_p1_co_unused));
	am_sub #(.W(8)) u_ex2_sub1 (.a(ex1), .b(8'd1), .bi(1'b0), .d(ex2_sub1_w), .bo(ex2_sub1_bo_unused));

	wire v2_w          = v1;
	wire sp2_w         = sp1;
	wire sp_nan2_w     = sp_nan1;
	wire sp_inf2_w     = sp_inf1;
	wire sx2_w         = sx1;
	wire close_cand2_w = close_cand1;
	wire [7:0] ex2_w   = ex1;
	wire [26:0] px2_w  = {mx1, 3'b000};
	(* keep = "true", dont_touch = "true" *) wire [26:0] py2_w = {
		py2_raw[26:18] ^ {9{sub1_c}},
		py2_raw[17:9]  ^ {9{sub1_b}},
		py2_raw[8:0]   ^ {9{sub1_a}}
	};

	reg r_v2, r_sp2, r_sp_nan2, r_sp_inf2, r_sx2, r_close_cand2;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg r_sub2_a, r_sub2_b, r_sub2_c, r_sub2_ci;
	reg [7:0] r_ex2, r_ex2_p1, r_ex2_sub1;
	reg [26:0] r_px2, r_py2;
	reg [24:0] r_close_diff2;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_v2 <= 1'b0;
		end else if (en_s2[0]) begin
			r_v2 <= v2_w;
		end
	end

	always @(posedge clk) begin
		if (en_s2[0]) begin
			r_sp2         <= sp2_w;
			r_sp_nan2     <= sp_nan2_w;
			r_sp_inf2     <= sp_inf2_w;
			r_sx2         <= sx2_w;
			r_sub2_a      <= sub1_a;
			r_sub2_b      <= sub1_a;
			r_sub2_c      <= sub1_a;
			r_sub2_ci     <= sub1_a;
			r_close_cand2 <= close_cand2_w;
		end
		if (en_s2[1]) begin
			r_ex2      <= ex2_w;
			r_ex2_p1   <= ex2_p1_w;
		end
		if (en_s2[2]) begin
			r_ex2_sub1  <= ex2_sub1_w;
			r_px2[6:0]  <= px2_w[6:0];
		end
		if (en_s2[3]) r_px2[18:7]  <= px2_w[18:7];
		if (en_s2[4]) begin
			r_px2[26:19] <= px2_w[26:19];
			r_py2[4:0]   <= py2_w[4:0];
		end
		if (en_s2[5]) r_py2[16:5]  <= py2_w[16:5];
		if (en_s2[6]) r_py2[26:17] <= py2_w[26:17];
		if (en_s2[7]) r_close_diff2[12:0]  <= close_diff2_w[12:0];
		if (en_s2[8]) r_close_diff2[24:13] <= close_diff2_w[24:13];
	end

	wire v2          = r_v2;
	wire sp2         = r_sp2;
	wire sp_nan2     = r_sp_nan2;
	wire sp_inf2     = r_sp_inf2;
	wire sx2         = r_sx2;
	wire close_cand2 = r_close_cand2;
	wire [7:0] ex2      = r_ex2;
	wire [7:0] ex2_p1   = r_ex2_p1;
	wire [7:0] ex2_sub1 = r_ex2_sub1;
	wire [26:0] px2         = r_px2;
	wire [26:0] py2         = r_py2;
	wire [24:0] close_diff2 = r_close_diff2;

	/* =========================================================================
	 * Stage 3:
	 * - Far path: 28-bit addition u_sum (Pre-LUTs = 1, Post-LUTs = 1 via direct
	 *   <= 5-input functions of sum[5:0] and 6-bit slice prefix-ANDs).
	 * - Close path: 2-level nibble LZC and zero-detect from registered close_diff2
	 *   (0 CARRY4s, <= 3 LUT levels).
	 * ========================================================================= */
	wire [27:0] sum;
	wire sum_co_unused;
`ifdef SYNTH_KINTEX7
	am_add #(.W(28)) u_sum (
		.a({1'b0, px2}),
		.b({r_sub2_a, py2}),
		.ci(r_sub2_ci),
		.s(sum),
		.co(sum_co_unused)
	);
	wire far_c_w = sum[27];
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(28)) u_sum (
		.a({1'b0, px2}),
		.b({r_sub2_a, py2}),
		.ci(r_sub2_ci),
		.s(sum),
		.co(sum_co_unused)
	);
	wire far_c_w = sum[27];
`else
	/* Speculative 6-bit carry-select with fast 2-bit -> 6-bit CLA + 3+3 split adders (<= 10 cmos3) */
	wire [5:0] sum_s0;
	wire [2:0] sum_s0_h0, sum_s0_h1;
	wire sum_c3, sum_c6_s0, sum_c6_s1;
	wire [5:0] sum_6_0, sum_6_1, sum_12_0, sum_12_1, sum_18_0, sum_18_1;
	wire [3:0] sum_t0, sum_t1;
	wire sum_c6, sum_c12, sum_c18, sum_c24;
	wire sum_co_t0, sum_co_t1;

	/* Flat 3-bit -> 6/9-bit CLA + 3-bit fadd_cla3 blocks + direct far_c_w (<= 9 cmos3) */
	wire [26:0] cla_g = px2[26:0] & py2[26:0];
	wire [26:0] cla_p = px2[26:0] | py2[26:0];
	wire [8:0] g3_w, p3_w;
	genvar g3i;
	generate
	for (g3i = 0; g3i < 9; g3i = g3i + 1) begin : g_cla3
		assign p3_w[g3i] = &cla_p[3*g3i +: 3];
		assign g3_w[g3i] = cla_g[3*g3i+2] | (cla_p[3*g3i+2] & cla_g[3*g3i+1]) | (cla_p[3*g3i+2] & cla_p[3*g3i+1] & cla_g[3*g3i]);
	end
	endgenerate
	wire [3:0] cla_g6_w, cla_p6_w;
	generate
	for (g3i = 0; g3i < 4; g3i = g3i + 1) begin : g_cla6
		assign cla_p6_w[g3i] = p3_w[2*g3i+1] & p3_w[2*g3i];
		assign cla_g6_w[g3i] = g3_w[2*g3i+1] | (p3_w[2*g3i+1] & g3_w[2*g3i]);
	end
	endgenerate
	wire p9_top = p3_w[8] & p3_w[7] & p3_w[6];
	wire g9_top = g3_w[8] | (p3_w[8] & g3_w[7]) | (p3_w[8] & p3_w[7] & g3_w[6]);

	wire [3:0] sum_s0l_4  = fadd_cla3(px2[2:0], py2[2:0], r_sub2_ci);
	wire [3:0] sum_s0h0_4 = fadd_cla3(px2[5:3], py2[5:3], 1'b0);
	wire [3:0] sum_s0h1_4 = fadd_cla3(px2[5:3], py2[5:3], 1'b1);
	assign sum_s0[2:0] = sum_s0l_4[2:0];
	assign sum_c3      = sum_s0l_4[3];
	assign sum_s0_h0   = sum_s0h0_4[2:0];
	assign sum_s0_h1   = sum_s0h1_4[2:0];
	assign sum_c6_s0   = sum_s0h0_4[3];
	assign sum_c6_s1   = sum_s0h1_4[3];
	assign sum_s0[5:3] = sum_c3 ? sum_s0_h1 : sum_s0_h0;
	assign sum_c6      = sum_c3 ? sum_c6_s1 : sum_c6_s0;
	assign sum_c12     = cla_g6_w[1] | (cla_p6_w[1] & cla_g6_w[0]) | (cla_p6_w[1] & cla_p6_w[0] & r_sub2_ci);
	assign sum_c18     = cla_g6_w[2] | (cla_p6_w[2] & cla_g6_w[1]) | (cla_p6_w[2] & cla_p6_w[1] & cla_g6_w[0]) |
	                     (cla_p6_w[2] & cla_p6_w[1] & cla_p6_w[0] & r_sub2_ci);
	assign sum_c24     = cla_g6_w[3] | (cla_p6_w[3] & cla_g6_w[2]) | (cla_p6_w[3] & cla_p6_w[2] & cla_g6_w[1]) |
	                     (cla_p6_w[3] & cla_p6_w[2] & cla_p6_w[1] & cla_g6_w[0]) |
	                     (cla_p6_w[3] & cla_p6_w[2] & cla_p6_w[1] & cla_p6_w[0] & r_sub2_ci);

	/* 3+3 split carry-select with flat 3-bit CLA fadd_cla3 (<= 5 cmos3 before c-mux) */
	wire [3:0] s6_l0_4 = fadd_cla3(px2[8:6],   py2[8:6],   1'b0);
	wire [3:0] s6_l1_4 = fadd_cla3(px2[8:6],   py2[8:6],   1'b1);
	wire [3:0] s6_h0_4 = fadd_cla3(px2[11:9],  py2[11:9],  1'b0);
	wire [3:0] s6_h1_4 = fadd_cla3(px2[11:9],  py2[11:9],  1'b1);
	wire [2:0] s6_lo0 = s6_l0_4[2:0], s6_lo1 = s6_l1_4[2:0], s6_hi0 = s6_h0_4[2:0], s6_hi1 = s6_h1_4[2:0];
	wire c9_0 = s6_l0_4[3], c9_1 = s6_l1_4[3], c9_u0 = s6_h0_4[3], c9_u1 = s6_h1_4[3];
	assign sum_6_0 = {c9_0 ? s6_hi1 : s6_hi0, s6_lo0};
	assign sum_6_1 = {c9_1 ? s6_hi1 : s6_hi0, s6_lo1};

	wire [3:0] s12_l0_4 = fadd_cla3(px2[14:12], py2[14:12], 1'b0);
	wire [3:0] s12_l1_4 = fadd_cla3(px2[14:12], py2[14:12], 1'b1);
	wire [3:0] s12_h0_4 = fadd_cla3(px2[17:15], py2[17:15], 1'b0);
	wire [3:0] s12_h1_4 = fadd_cla3(px2[17:15], py2[17:15], 1'b1);
	wire [2:0] s12_lo0 = s12_l0_4[2:0], s12_lo1 = s12_l1_4[2:0], s12_hi0 = s12_h0_4[2:0], s12_hi1 = s12_h1_4[2:0];
	wire c15_0 = s12_l0_4[3], c15_1 = s12_l1_4[3], c15_u0 = s12_h0_4[3], c15_u1 = s12_h1_4[3];
	assign sum_12_0 = {c15_0 ? s12_hi1 : s12_hi0, s12_lo0};
	assign sum_12_1 = {c15_1 ? s12_hi1 : s12_hi0, s12_lo1};

	wire [3:0] s18_l0_4 = fadd_cla3(px2[20:18], py2[20:18], 1'b0);
	wire [3:0] s18_l1_4 = fadd_cla3(px2[20:18], py2[20:18], 1'b1);
	wire [3:0] s18_h0_4 = fadd_cla3(px2[23:21], py2[23:21], 1'b0);
	wire [3:0] s18_h1_4 = fadd_cla3(px2[23:21], py2[23:21], 1'b1);
	wire [2:0] s18_lo0 = s18_l0_4[2:0], s18_lo1 = s18_l1_4[2:0], s18_hi0 = s18_h0_4[2:0], s18_hi1 = s18_h1_4[2:0];
	wire c21_0 = s18_l0_4[3], c21_1 = s18_l1_4[3], c21_u0 = s18_h0_4[3], c21_u1 = s18_h1_4[3];
	assign sum_18_0 = {c21_0 ? s18_hi1 : s18_hi0, s18_lo0};
	assign sum_18_1 = {c21_1 ? s18_hi1 : s18_hi0, s18_lo1};

	wire far_c27_raw = g9_top | (p9_top & cla_g6_w[2]) | (p9_top & cla_p6_w[2] & cla_g6_w[1]) |
	                   (p9_top & cla_p6_w[2] & cla_p6_w[1] & cla_g6_w[0]) | (p9_top & (&cla_p6_w[2:0]) & r_sub2_a);
	wire far_c_w = r_sub2_a ^ far_c27_raw;

	wire [3:0] sum_t0_4 = fadd_cla3(px2[26:24], py2[26:24], 1'b0);
	wire [3:0] sum_t1_4 = fadd_cla3(px2[26:24], py2[26:24], 1'b1);
	assign sum_t0 = {r_sub2_a ^ sum_t0_4[3], sum_t0_4[2:0]};
	assign sum_t1 = {r_sub2_a ^ sum_t1_4[3], sum_t1_4[2:0]};
	assign sum_co_t0 = 1'b0;
	assign sum_co_t1 = 1'b0;

	assign sum = {
		sum_c24 ? sum_t1   : sum_t0,
		sum_c18 ? sum_18_1 : sum_18_0,
		sum_c12 ? sum_12_1 : sum_12_0,
		sum_c6  ? sum_6_1  : sum_6_0,
		sum_s0
	};
	assign sum_co_unused = (sum_c24 ? sum_co_t1 : sum_co_t0) | (&{1'b0, c9_u0, c9_u1, c15_u0, c15_u1, c21_u0, c21_u1});
`endif

	/* Direct <= 5-input LUT expressions of sum[5:0] (Post-LUTs = 1 guaranteed!) */
	(* keep = "true" *) wire [1:0] fmu_c_w;
	assign fmu_c_w[0] = sum[4] ^ (sum[3] & (sum[2] | sum[1] | sum[0] | sum[4]));
	assign fmu_c_w[1] = sum[5] ^ (sum[4] & sum[3]);
	(* keep = "true" *) wire cin6_c_w = sum[5] & sum[4] & sum[3];

	(* keep = "true" *) wire [2:0] fmu_n_w;
	assign fmu_n_w[0] = sum[3] ^ (sum[2] & (sum[1] | sum[0] | sum[3]));
	assign fmu_n_w[1] = sum[4] ^ (sum[3] & sum[2]);
	assign fmu_n_w[2] = sum[5] ^ (sum[4] & sum[3] & sum[2]);
	(* keep = "true" *) wire cin6_n_w = sum[5] & sum[4] & sum[3] & sum[2];

	(* keep = "true" *) wire [3:0] fmu_u_w;
	assign fmu_u_w[0] = sum[2] ^ (sum[1] & (sum[0] | sum[2]));
	assign fmu_u_w[1] = sum[3] ^ (sum[2] & sum[1]);
	assign fmu_u_w[2] = sum[4] ^ (sum[3] & sum[2] & sum[1]);
	assign fmu_u_w[3] = sum[5] ^ (sum[4] & sum[3] & sum[2] & sum[1]);
	(* keep = "true" *) wire cin6_u_w = sum[5] & sum[4] & sum[3] & sum[2] & sum[1];

	(* keep = "true" *) wire [11:6] p6_w;
	(* keep = "true" *) wire [17:12] p12_w;
	(* keep = "true" *) wire [23:18] p18_w;
	(* keep = "true" *) wire [26:24] p24_w;
`ifdef SYNTH_KINTEX7
	assign p6_w[6]  = sum[6];
	assign p6_w[7]  = &sum[7:6];
	assign p6_w[8]  = &sum[8:6];
	assign p6_w[9]  = &sum[9:6];
	assign p6_w[10] = &sum[10:6];
	assign p6_w[11] = &sum[11:6];

	assign p12_w[12] = sum[12];
	assign p12_w[13] = &sum[13:12];
	assign p12_w[14] = &sum[14:12];
	assign p12_w[15] = &sum[15:12];
	assign p12_w[16] = &sum[16:12];
	assign p12_w[17] = &sum[17:12];

	assign p18_w[18] = sum[18];
	assign p18_w[19] = &sum[19:18];
	assign p18_w[20] = &sum[20:18];
	assign p18_w[21] = &sum[21:18];
	assign p18_w[22] = &sum[22:18];
	assign p18_w[23] = &sum[23:18];

	assign p24_w[24] = sum[24];
	assign p24_w[25] = &sum[25:24];
	assign p24_w[26] = &sum[26:24];
`elsif SYNTH_KINTEX_ULTRASCALE
	assign p6_w[6]  = sum[6];
	assign p6_w[7]  = &sum[7:6];
	assign p6_w[8]  = &sum[8:6];
	assign p6_w[9]  = &sum[9:6];
	assign p6_w[10] = &sum[10:6];
	assign p6_w[11] = &sum[11:6];

	assign p12_w[12] = sum[12];
	assign p12_w[13] = &sum[13:12];
	assign p12_w[14] = &sum[14:12];
	assign p12_w[15] = &sum[15:12];
	assign p12_w[16] = &sum[16:12];
	assign p12_w[17] = &sum[17:12];

	assign p18_w[18] = sum[18];
	assign p18_w[19] = &sum[19:18];
	assign p18_w[20] = &sum[20:18];
	assign p18_w[21] = &sum[21:18];
	assign p18_w[22] = &sum[22:18];
	assign p18_w[23] = &sum[23:18];

	assign p24_w[24] = sum[24];
	assign p24_w[25] = &sum[25:24];
	assign p24_w[26] = &sum[26:24];
`else
	wire [11:6] p6_0_w, p6_1_w;
	assign p6_0_w[6]  = sum_6_0[0];
	assign p6_0_w[7]  = &sum_6_0[1:0];
	assign p6_0_w[8]  = &sum_6_0[2:0];
	assign p6_0_w[9]  = &sum_6_0[3:0];
	assign p6_0_w[10] = &sum_6_0[4:0];
	assign p6_0_w[11] = &sum_6_0[5:0];
	assign p6_1_w[6]  = sum_6_1[0];
	assign p6_1_w[7]  = &sum_6_1[1:0];
	assign p6_1_w[8]  = &sum_6_1[2:0];
	assign p6_1_w[9]  = &sum_6_1[3:0];
	assign p6_1_w[10] = &sum_6_1[4:0];
	assign p6_1_w[11] = &sum_6_1[5:0];
	assign p6_w = sum_c6 ? p6_1_w : p6_0_w;

	wire [17:12] p12_0_w, p12_1_w;
	assign p12_0_w[12] = sum_12_0[0];
	assign p12_0_w[13] = &sum_12_0[1:0];
	assign p12_0_w[14] = &sum_12_0[2:0];
	assign p12_0_w[15] = &sum_12_0[3:0];
	assign p12_0_w[16] = &sum_12_0[4:0];
	assign p12_0_w[17] = &sum_12_0[5:0];
	assign p12_1_w[12] = sum_12_1[0];
	assign p12_1_w[13] = &sum_12_1[1:0];
	assign p12_1_w[14] = &sum_12_1[2:0];
	assign p12_1_w[15] = &sum_12_1[3:0];
	assign p12_1_w[16] = &sum_12_1[4:0];
	assign p12_1_w[17] = &sum_12_1[5:0];
	assign p12_w = sum_c12 ? p12_1_w : p12_0_w;

	wire [23:18] p18_0_w, p18_1_w;
	assign p18_0_w[18] = sum_18_0[0];
	assign p18_0_w[19] = &sum_18_0[1:0];
	assign p18_0_w[20] = &sum_18_0[2:0];
	assign p18_0_w[21] = &sum_18_0[3:0];
	assign p18_0_w[22] = &sum_18_0[4:0];
	assign p18_0_w[23] = &sum_18_0[5:0];
	assign p18_1_w[18] = sum_18_1[0];
	assign p18_1_w[19] = &sum_18_1[1:0];
	assign p18_1_w[20] = &sum_18_1[2:0];
	assign p18_1_w[21] = &sum_18_1[3:0];
	assign p18_1_w[22] = &sum_18_1[4:0];
	assign p18_1_w[23] = &sum_18_1[5:0];
	assign p18_w = sum_c18 ? p18_1_w : p18_0_w;

	wire [26:24] p24_0_w, p24_1_w;
	assign p24_0_w[24] = sum_t0[0];
	assign p24_0_w[25] = &sum_t0[1:0];
	assign p24_0_w[26] = &sum_t0[2:0];
	assign p24_1_w[24] = sum_t1[0];
	assign p24_1_w[25] = &sum_t1[1:0];
	assign p24_1_w[26] = &sum_t1[2:0];
	assign p24_w = sum_c24 ? p24_1_w : p24_0_w;
`endif

	/* 3-level nibble LZC on registered close_diff2[24:0] (3 LUT levels max, 0 CARRY4s) */
	(* keep = "true", dont_touch = "true" *) wire [5:0] nib_nz;
	(* keep = "true" *) wire [5:0] n_oh0, n_oh1, n_oh2, n_oh3;
	generate
	for (gs = 0; gs < 6; gs = gs + 1) begin : g_lzc_nib
		wire [3:0] nb = close_diff2[4 * gs +: 4];
		assign nib_nz[gs] = |nb;
		assign n_oh1[gs]  = nb[3];
		assign n_oh2[gs]  = !nb[3] && nb[2];
		assign n_oh3[gs]  = !nb[3] && !nb[2] && nb[1];
		assign n_oh0[gs]  = !nb[3] && !nb[2] && !nb[1] && nb[0];
	end
	endgenerate

	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n5 = close_diff2[23] ? 5'd1  : close_diff2[22] ? 5'd2  : close_diff2[21] ? 5'd3  : 5'd4;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n4 = close_diff2[19] ? 5'd5  : close_diff2[18] ? 5'd6  : close_diff2[17] ? 5'd7  : 5'd8;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n3 = close_diff2[15] ? 5'd9  : close_diff2[14] ? 5'd10 : close_diff2[13] ? 5'd11 : 5'd12;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n2 = close_diff2[11] ? 5'd13 : close_diff2[10] ? 5'd14 : close_diff2[9]  ? 5'd15 : 5'd16;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n1 = close_diff2[7]  ? 5'd17 : close_diff2[6]  ? 5'd18 : close_diff2[5]  ? 5'd19 : 5'd20;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_n0 = close_diff2[3]  ? 5'd21 : close_diff2[2]  ? 5'd22 : close_diff2[1]  ? 5'd23 : 5'd24;

	(* keep = "true", dont_touch = "true" *) wire hi_nz  = close_diff2[24] | nib_nz[5] | nib_nz[4];
	(* keep = "true", dont_touch = "true" *) wire mid_nz = nib_nz[3] | nib_nz[2];
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_hi  = close_diff2[24] ? 5'd0 : (nib_nz[5] ? lz_n5 : lz_n4);
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_mid = nib_nz[3] ? lz_n3 : lz_n2;
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz_lo  = nib_nz[1] ? lz_n1 : (nib_nz[0] ? lz_n0 : 5'd27);
	(* keep = "true", dont_touch = "true" *) wire [4:0] lz = hi_nz ? lz_hi : (mid_nz ? lz_mid : lz_lo);

	wire p54_nz = nib_nz[5] | nib_nz[4];
	wire p32_nz = nib_nz[3] | nib_nz[2];
	wire p10_nz = nib_nz[1] | nib_nz[0];
	wire [3:0] p54_oh = {nib_nz[5] ? n_oh3[5] : n_oh3[4], nib_nz[5] ? n_oh2[5] : n_oh2[4], nib_nz[5] ? n_oh1[5] : n_oh1[4], nib_nz[5] ? n_oh0[5] : n_oh0[4]};
	wire [3:0] p32_oh = {nib_nz[3] ? n_oh3[3] : n_oh3[2], nib_nz[3] ? n_oh2[3] : n_oh2[2], nib_nz[3] ? n_oh1[3] : n_oh1[2], nib_nz[3] ? n_oh0[3] : n_oh0[2]};
	wire [3:0] p10_oh = {nib_nz[1] ? n_oh3[1] : n_oh3[0], nib_nz[1] ? n_oh2[1] : n_oh2[0], nib_nz[1] ? n_oh1[1] : n_oh1[0], nib_nz[1] ? n_oh0[1] : n_oh0[0]};
	wire [3:0] lz10_oh_base;
	assign lz10_oh_base[0] = close_diff2[24] || (p54_nz ? p54_oh[0] : (p32_nz ? p32_oh[0] : (p10_nz && p10_oh[0])));
	assign lz10_oh_base[1] = !close_diff2[24] && (p54_nz ? p54_oh[1] : (p32_nz ? p32_oh[1] : (p10_nz && p10_oh[1])));
	assign lz10_oh_base[2] = !close_diff2[24] && (p54_nz ? p54_oh[2] : (p32_nz ? p32_oh[2] : (p10_nz && p10_oh[2])));
	assign lz10_oh_base[3] = !close_diff2[24] && (p54_nz ? p54_oh[3] : (p32_nz ? p32_oh[3] : (p10_nz ? p10_oh[3] : 1'b1)));
	(* keep = "true", dont_touch = "true" *) wire [3:0] lz10_oh_a = lz10_oh_base;
	(* keep = "true", dont_touch = "true" *) wire [3:0] lz10_oh_b = lz10_oh_base;
	(* keep = "true", dont_touch = "true" *) wire [3:0] lz10_oh_c = lz10_oh_base;

	wire is_close3_w = close_cand2 && (close_diff2[24:23] == 2'b00);
	wire sz3_w       = close_cand2 && !hi_nz && !mid_nz && !(nib_nz[1] | nib_nz[0]);
	wire ex2_le31_w  = (ex2[7:5] == 3'd0);
	wire ex2_0_w     = (ex2[7:4] == 4'd0) && (ex2[3:0] == 4'd0);
	wire ex2_1_w     = (ex2[7:4] == 4'd0) && (ex2[3:0] == 4'd1);
	wire ex2_254_w   = (&ex2[7:4]) && (ex2[3:0] == 4'hE);

	wire v3_w              = v2;
	wire sp3_w             = sp2;
	wire sp_nan3_w         = sp_nan2;
	wire sp_inf3_w         = sp_inf2;
	wire sx3_w             = sx2;
	wire [7:0] ex2_3_w     = ex2;
	wire [7:0] ex2_p1_3_w  = ex2_p1;
	wire [7:0] ex2_sub1_3_w = ex2_sub1;
	wire [27:0] c_in_pad3  = {2'b00, close_diff2[22:0], 3'b000};
	wire [24:0] nl_s1_3_w;
	generate
	for (gs = 0; gs < 25; gs = gs + 1) begin : g_nls1_s3
		wire [3:0] oh_sel = (gs < 9) ? lz10_oh_a : (gs < 17) ? lz10_oh_b : lz10_oh_c;
		assign nl_s1_3_w[gs] = (oh_sel[0] & c_in_pad3[gs + 3]) |
		                       (oh_sel[1] & c_in_pad3[gs + 2]) |
		                       (oh_sel[2] & c_in_pad3[gs + 1]) |
		                       (oh_sel[3] & c_in_pad3[gs]);
	end
	endgenerate

	reg r_v3, r_sp3, r_sp_nan3, r_sp_inf3, r_sx3, r_sz3;
	(* equivalent_register_removal = "no", max_fanout = 14 *) reg r_is_close_a, r_is_close_b;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg r_far_c_a, r_far_c_b, r_far_c_c, r_far_c_d, r_far_c_e;
	(* equivalent_register_removal = "no", max_fanout = 12 *) reg r_far_norm_a, r_far_norm_b, r_far_norm_c, r_far_norm_d, r_far_norm_e;
	reg [26:6] r_sum3;
	reg [1:0] r_fmu_c3;
	reg [2:0] r_fmu_n3;
	reg [3:0] r_fmu_u3;
	reg r_cin6_c3, r_cin6_n3, r_cin6_u3;
	reg [11:6] r_p6_3;
	reg [17:12] r_p12_3;
	reg [23:18] r_p18_3;
	reg [26:24] r_p24_3;
	reg [24:0] r_nls1_3;
	(* equivalent_register_removal = "no", max_fanout = 14 *) reg [4:0] r_lz3_a, r_lz3_b;
	reg [7:0] r_ex2_3, r_ex2_p1_3, r_ex2_sub1_3;
	reg r_ex2_le31_3, r_ex2_0_3, r_ex2_1_3, r_ex2_254_3;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_v3 <= 1'b0;
		end else if (en_s3[0]) begin
			r_v3 <= v3_w;
		end
	end

	always @(posedge clk) begin
		if (en_s3[0]) begin
			r_sp3        <= sp3_w;
			r_sp_nan3    <= sp_nan3_w;
			r_sp_inf3    <= sp_inf3_w;
			r_sx3        <= sx3_w;
			r_sz3        <= sz3_w;
			r_is_close_a <= is_close3_w;
			r_is_close_b <= is_close3_w;
			r_ex2_le31_3 <= ex2_le31_w;
			r_ex2_0_3    <= ex2_0_w;
			r_ex2_1_3    <= ex2_1_w;
			r_ex2_254_3  <= ex2_254_w;
		end
		if (en_s3[1]) begin
			r_far_c_a    <= far_c_w;
			r_far_c_b    <= far_c_w;
			r_far_c_c    <= far_c_w;
			r_far_c_d    <= far_c_w;
			r_far_c_e    <= far_c_w;
			r_far_norm_a <= sum[26];
			r_far_norm_b <= sum[26];
			r_far_norm_c <= sum[26];
			r_far_norm_d <= sum[26];
			r_far_norm_e <= sum[26];
		end
		if (en_s3[2]) begin
			r_fmu_c3  <= fmu_c_w;
			r_fmu_n3  <= fmu_n_w;
			r_fmu_u3  <= fmu_u_w;
			r_cin6_c3 <= cin6_c_w;
			r_cin6_n3 <= cin6_n_w;
			r_cin6_u3 <= cin6_u_w;
		end
		if (en_s3[3]) begin
			r_p6_3  <= p6_w;
			r_p12_3 <= p12_w;
		end
		if (en_s3[4]) begin
			r_p18_3 <= p18_w;
			r_p24_3 <= p24_w;
		end
		if (en_s3[5]) r_sum3[16:6]  <= sum[16:6];
		if (en_s3[6]) begin
			r_sum3[26:17] <= sum[26:17];
			r_lz3_a       <= lz;
		end
		if (en_s3[7]) begin
			r_lz3_b        <= lz;
			r_nls1_3[6:0]  <= nl_s1_3_w[6:0];
		end
		if (en_s3[8])
			r_nls1_3[24:7] <= nl_s1_3_w[24:7];
		if (en_s3[9]) begin
			r_ex2_3             <= ex2_3_w;
			r_ex2_p1_3          <= ex2_p1_3_w;
		end
		if (en_s3[10])
			r_ex2_sub1_3        <= ex2_sub1_3_w;
	end

	wire v3          = r_v3;
	wire sp3         = r_sp3;
	wire sp_nan3     = r_sp_nan3;
	wire sp_inf3     = r_sp_inf3;
	wire sx3         = r_sx3;
	wire sz3         = r_sz3;
	wire is_close_a  = r_is_close_a;
	wire is_close_b  = r_is_close_b;
	wire far_c_a     = r_far_c_a;
	wire far_c_b     = r_far_c_b;
	wire far_c_c     = r_far_c_c;
	wire far_c_d     = r_far_c_d;
	wire far_c_e     = r_far_c_e;
	wire far_norm_a  = r_far_norm_a;
	wire far_norm_b  = r_far_norm_b;
	wire far_norm_c  = r_far_norm_c;
	wire far_norm_d  = r_far_norm_d;
	wire far_norm_e  = r_far_norm_e;
	wire [26:6] sum3        = r_sum3;
	wire [1:0] fmu_c3       = r_fmu_c3;
	wire [2:0] fmu_n3       = r_fmu_n3;
	wire [3:0] fmu_u3       = r_fmu_u3;
	wire cin6_c3            = r_cin6_c3;
	wire cin6_n3            = r_cin6_n3;
	wire cin6_u3            = r_cin6_u3;
	wire [11:6] p6_3        = r_p6_3;
	wire [17:12] p12_3      = r_p12_3;
	wire [23:18] p18_3      = r_p18_3;
	wire [26:24] p24_3      = r_p24_3;
	wire [24:0] nls1_3      = r_nls1_3;
	wire [4:0] lz3_a        = r_lz3_a;
	wire [4:0] lz3_b        = r_lz3_b;
	wire [7:0] ex2_3        = r_ex2_3;
	wire [7:0] ex2_p1_3     = r_ex2_p1_3;
	wire [7:0] ex2_sub1_3   = r_ex2_sub1_3;
	wire ex2_le31_3         = r_ex2_le31_3;
	wire ex2_0_3            = r_ex2_0_3;
	wire ex2_1_3            = r_ex2_1_3;
	wire ex2_254_3          = r_ex2_254_3;

	/* =========================================================================
	 * Stage 4:
	 * - Close path: 2-stage left shift (2 LUT levels) and pure-LUT 8-bit exponent
	 *   subtract (2 LUT levels, 0 CARRY4s).
	 * - Far path: 2-level factored-carry mantissa selection & 3-level output pack.
	 * ========================================================================= */
	wire [48:0] nl_s1_pad = {nls1_3, 24'd0};
	(* keep = "true", dont_touch = "true" *) wire [24:0] nl_hi0;
	(* keep = "true", dont_touch = "true" *) wire [24:13] nl_hi1;
	genvar gk;
	generate
	for (gk = 0; gk < 13; gk = gk + 1) begin : g_nl_lo
		assign nl_hi0[gk] = (lz3_a[3:2] == 2'd0) ? nl_s1_pad[gk + 24] :
		                    (lz3_a[3:2] == 2'd1) ? nl_s1_pad[gk + 20] :
		                    (lz3_a[3:2] == 2'd2) ? nl_s1_pad[gk + 16] : nl_s1_pad[gk + 12];
	end
	for (gk = 13; gk < 25; gk = gk + 1) begin : g_nl_hi
		assign nl_hi0[gk] = (lz3_b[3:2] == 2'd0) ? nl_s1_pad[gk + 24] :
		                    (lz3_b[3:2] == 2'd1) ? nl_s1_pad[gk + 20] :
		                    (lz3_b[3:2] == 2'd2) ? nl_s1_pad[gk + 16] : nl_s1_pad[gk + 12];
		assign nl_hi1[gk] = (lz3_b[3:2] == 2'd0) ? nl_s1_pad[gk + 8]  :
		                    (lz3_b[3:2] == 2'd1) ? nl_s1_pad[gk + 4]  :
		                    (lz3_b[3:2] == 2'd2) ? nl_s1_pad[gk]      : 1'b0;
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) wire close_m22 = lz3_b[4] ? nl_hi1[23] : nl_hi0[23];

	wire [7:0] close_e;
	wire close_e_bo_unused;
	am_sub #(.W(8)) u_close_e (.a(ex2_3), .b({3'd0, lz3_b}), .bi(1'b0), .d(close_e), .bo(close_e_bo_unused));

	(* keep = "true", dont_touch = "true" *) wire cudf_lo = (ex2_3[2:0] <= lz3_a[2:0]);
	(* keep = "true", dont_touch = "true" *) wire cudf_eq = is_close_a && ex2_le31_3 && (ex2_3[4:3] == lz3_a[4:3]);
	(* keep = "true", dont_touch = "true" *) wire cudf_lt = is_close_a && ex2_le31_3 && (ex2_3[4:3] < lz3_a[4:3]);

	/* Far path Stage 4 */
	(* keep = "true", dont_touch = "true" *) wire cin_6  = far_c_a ? cin6_c3 : (far_norm_a ? cin6_n3 : cin6_u3);
	(* keep = "true", dont_touch = "true" *) wire cin_12 = (far_c_c ? cin6_c3 : (far_norm_c ? cin6_n3 : cin6_u3)) & p6_3[11];
	(* keep = "true", dont_touch = "true" *) wire p24_sel = far_c_c ? 1'b0 : (far_norm_c ? p24_3[26] : p24_3[25]);
	(* keep = "true", dont_touch = "true" *) wire far_m0_cout = cin_12 & p12_3[17] & p18_3[23] & p24_sel;

	(* keep = "true", dont_touch = "true" *) wire [22:4] mu_bit;
	(* keep = "true", dont_touch = "true" *) wire [22:4] p_mux;
	wire [10:5] w_p6  = {p6_3[10:6], 1'b1};
	wire [16:11] w_p12 = {p12_3[16:12], 1'b1};
	wire [22:17] w_p18 = {p18_3[22:18], 1'b1};

	genvar gi;
	generate
	for (gi = 4; gi < 11; gi = gi + 1) begin : g_mu_lo
		assign mu_bit[gi] = far_c_a ? sum3[gi + 4] : (far_norm_a ? sum3[gi + 3] : sum3[gi + 2]);
	end
	for (gi = 11; gi < 18; gi = gi + 1) begin : g_mu_mid
		assign mu_bit[gi] = far_c_b ? sum3[gi + 4] : (far_norm_b ? sum3[gi + 3] : sum3[gi + 2]);
	end
	for (gi = 18; gi < 23; gi = gi + 1) begin : g_mu_hi
		assign mu_bit[gi] = far_c_c ? sum3[gi + 4] : (far_norm_c ? sum3[gi + 3] : sum3[gi + 2]);
	end
	for (gi = 4; gi <= 7; gi = gi + 1) begin : g_pmux_4_7
		assign p_mux[gi] = far_c_a ? w_p6[gi + 3] : (far_norm_a ? w_p6[gi + 2] : w_p6[gi + 1]);
	end
	for (gi = 10; gi <= 13; gi = gi + 1) begin : g_pmux_10_13
		assign p_mux[gi] = far_c_b ? w_p12[gi + 3] : (far_norm_b ? w_p12[gi + 2] : w_p12[gi + 1]);
	end
	for (gi = 16; gi <= 19; gi = gi + 1) begin : g_pmux_16_19
		wire fc = (gi < 18) ? far_c_b : far_c_c;
		wire fn = (gi < 18) ? far_norm_b : far_norm_c;
		assign p_mux[gi] = fc ? w_p18[gi + 3] : (fn ? w_p18[gi + 2] : w_p18[gi + 1]);
	end
	endgenerate

	assign p_mux[8]  = far_c_a ? p6_3[11] : (far_norm_a ? p6_3[10] : p6_3[9]);
	assign p_mux[9]  = far_c_a ? (p6_3[11] & p12_3[12]) : (far_norm_a ? p6_3[11] : p6_3[10]);
	assign p_mux[14] = far_c_b ? p12_3[17] : (far_norm_b ? p12_3[16] : p12_3[15]);
	assign p_mux[15] = far_c_b ? (p12_3[17] & p18_3[18]) : (far_norm_b ? p12_3[17] : p12_3[16]);
	assign p_mux[20] = far_c_c ? p18_3[23] : (far_norm_c ? p18_3[22] : p18_3[21]);
	assign p_mux[21] = far_c_c ? (p18_3[23] & p24_3[24]) : (far_norm_c ? p18_3[23] : p18_3[22]);
	assign p_mux[22] = far_c_c ? p24_3[25] : (far_norm_c ? p24_3[24] : 1'b1);

	(* keep = "true", dont_touch = "true" *) wire [22:0] far_m;
	assign far_m[0] = far_c_a ? fmu_c3[0] : (far_norm_a ? fmu_n3[0] : fmu_u3[0]);
	assign far_m[1] = far_c_a ? fmu_c3[1] : (far_norm_a ? fmu_n3[1] : fmu_u3[1]);
	assign far_m[2] = far_c_a ? (sum3[6] ^ cin6_c3) : (far_norm_a ? fmu_n3[2] : fmu_u3[2]);
	(* keep = "true", dont_touch = "true" *) wire mu3_c = sum3[7] ^ (cin6_c3 & p6_3[6]);
	assign far_m[3] = far_c_a ? mu3_c : (far_norm_a ? (sum3[6] ^ cin6_n3) : fmu_u3[3]);

	generate
	for (gi = 4; gi <= 9; gi = gi + 1) begin : g_fm_4_9
		assign far_m[gi] = mu_bit[gi] ^ (cin_6 & p_mux[gi]);
	end
	for (gi = 10; gi <= 15; gi = gi + 1) begin : g_fm_10_15
		assign far_m[gi] = mu_bit[gi] ^ (cin_12 & p_mux[gi]);
	end
	for (gi = 16; gi <= 21; gi = gi + 1) begin : g_fm_16_21
		assign far_m[gi] = mu_bit[gi] ^ (cin_12 & p12_3[17] & p_mux[gi]);
	end
	endgenerate
	assign far_m[22] = mu_bit[22] ^ (cin_12 & p12_3[17] & p18_3[23] & p_mux[22]);

	/* Final packing (<= 3-4 LUT levels total, 0 CARRY4s) */
	(* keep = "true", dont_touch = "true" *) wire far_mz = !is_close_a && (ex2_0_3 || (ex2_1_3 && !far_c_d && !far_norm_d) || (ex2_254_3 && far_c_d));
	(* keep = "true", dont_touch = "true" *) wire force_mz_a = sp3 || sz3 || far_mz || cudf_lt || (cudf_eq && cudf_lo);
	(* keep = "true", dont_touch = "true" *) wire force_mz_b = sp3 || sz3 || far_mz || cudf_lt || (cudf_eq && cudf_lo);

	(* keep = "true", dont_touch = "true" *) wire [22:0] m_out;
	generate
	for (gi = 0; gi < 12; gi = gi + 1) begin : g_mout_lo
		assign m_out[gi] = !force_mz_a && (is_close_a ? (!lz3_a[4] && nl_hi0[gi + 1]) : far_m[gi]);
	end
	for (gi = 12; gi < 22; gi = gi + 1) begin : g_mout_hi
		assign m_out[gi] = !force_mz_b && (is_close_b ? (lz3_b[4] ? nl_hi1[gi + 1] : nl_hi0[gi + 1]) : far_m[gi]);
	end
	endgenerate
	assign m_out[22] = sp_nan3 || (!force_mz_b && (is_close_b ? close_m22 : far_m[22]));

	(* keep = "true", dont_touch = "true" *) wire use_close_e = !sp3 && !sz3 && is_close_b && !cudf_lt && !(cudf_eq && cudf_lo);
	(* keep = "true", dont_touch = "true" *) wire sp_force_ff = sp_nan3 || sp_inf3;
	(* keep = "true", dont_touch = "true" *) wire sp_force_0  = (sp3 && !sp_nan3 && !sp_inf3) || sz3 || is_close_b;
	(* keep = "true", dont_touch = "true" *) wire far_zero_c0 = ex2_0_3 || (ex2_1_3 && !far_c_e && !far_norm_e);
	(* keep = "true", dont_touch = "true" *) wire far_ovf_c1  = ex2_254_3 && (far_c_e || far_norm_e);

	(* keep = "true", dont_touch = "true" *) wire [7:0] ebit_c0, ebit_c1;
	(* keep = "true", dont_touch = "true" *) wire [7:0] far_e_c0, far_e_c1;
	(* keep = "true", dont_touch = "true" *) wire [7:0] e_out;
	generate
	for (gi = 0; gi < 8; gi = gi + 1) begin : g_eout
		wire fc_e = (gi < 4) ? far_c_d : far_c_e;
		wire fn_e = (gi < 4) ? far_norm_d : far_norm_e;
		assign ebit_c0[gi] = fc_e ? ex2_p1_3[gi] : (fn_e ? ex2_3[gi] : ex2_sub1_3[gi]);
		assign ebit_c1[gi] = (fc_e || fn_e) ? ex2_p1_3[gi] : ex2_3[gi];
		assign far_e_c0[gi] = (sp_force_ff || (!sp_force_0 && ex2_254_3 && far_c_e)) ? 1'b1 :
		                      (sp_force_0 || far_zero_c0) ? 1'b0 : ebit_c0[gi];
		assign far_e_c1[gi] = (sp_force_ff || (!sp_force_0 && far_ovf_c1)) ? 1'b1 :
		                      (sp_force_0 || ex2_0_3) ? 1'b0 : ebit_c1[gi];
		assign e_out[gi] = use_close_e ? close_e[gi] : (far_m0_cout ? far_e_c1[gi] : far_e_c0[gi]);
	end
	endgenerate

	wire o_valid_comb = v3;
	(* keep = "true", dont_touch = "true" *) wire s_out = !sp_nan3 && sx3 && !sz3;
	wire [31:0] o_r_comb = {s_out, e_out, m_out};

	reg r_ovalid;
	(* max_fanout = 16 *) reg [31:0] r_or;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			r_ovalid <= 1'b0;
		end else if (en_s4[0]) begin
			r_ovalid <= o_valid_comb;
		end
	end

	always @(posedge clk) begin
		if (en_s4[1]) r_or[15:0]  <= o_r_comb[15:0];
		if (en_s4[2]) r_or[31:16] <= o_r_comb[31:16];
	end

	assign o_valid = r_ovalid;
	assign o_r     = r_or;

	wire unused_bits = &{1'b0, shr_0[26:8], shr_0[0], shr_1[26:15], shr_1[7:0], shr_2[14:0],
`ifndef SYNTH_ASIC
		d1_coarse_oh_a, d1_coarse_oh_b,
`else
		shr_s1_w, d1_mid_oh_a, d1_mid_oh_b, d1_a[4:2], d1_b[4:2],
`endif
		sum[27],
`ifdef SYNTH_KINTEX7
		close_bo0_unused, close_bo1_unused,
`elsif SYNTH_KINTEX_ULTRASCALE
		close_bo0_unused, close_bo1_unused,
`else
		unused_cd_bo,
`endif
		nl_hi0[24], nl_hi0[0], nl_hi1[24], dab[7:5], dba[7:5], r_sub2_b, r_sub2_c,
		c_in_pad3[27:25], nl_s1_pad[11:0], en_s3[11], sum_co_unused,
		dab_bo_unused, dba_bo_unused, ex2_p1_co_unused, ex2_sub1_bo_unused, close_e_bo_unused, LAT[31:0]};
endmodule

`default_nettype wire
