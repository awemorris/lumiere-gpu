/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_fadd: optimized fp32 adder / subtractor for RX pipeline (LAT = 0 or 4).
 * Dual-path normalisation & rounding architecture compliant with ASIC 5nm 1.5 GHz
 * timing and primitive limits (LAT = 4).
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rx_fadd #(
	parameter LAT = 4
) (
	input wire clk,
	input wire rst_n,
	input wire i_valid,
	input wire [31:0] i_a,
	input wire [31:0] i_b,
	input wire i_sub,
	output wire o_valid,
	output wire [31:0] o_r
);
	/* Stage 1: unpack, specials, order by magnitude. */
	(* keep = "true" *) wire b_s31 = i_b[31] ^ i_sub;
	wire [31:0] b_in = {b_s31, i_b[30:0]};
`ifdef SYNTH_KINTEX7
	wire [7:0] ea, ea_cmp, eb, eb_cmp;
	genvar geab;
	generate
	for (geab = 0; geab < 8; geab = geab + 1) begin : g_eab_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eas (.I0(i_a[23 + geab]),  .O(ea[geab]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eac (.I0(i_a[23 + geab]),  .O(ea_cmp[geab]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ebs (.I0(b_in[23 + geab]), .O(eb[geab]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ebc (.I0(b_in[23 + geab]), .O(eb_cmp[geab]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] ea, ea_cmp, eb, eb_cmp;
	genvar geab;
	generate
	for (geab = 0; geab < 8; geab = geab + 1) begin : g_eab_buf
		assign ea[geab] = i_a[23 + geab];
		assign ea_cmp[geab] = i_a[23 + geab];
		assign eb[geab] = b_in[23 + geab];
		assign eb_cmp[geab] = b_in[23 + geab];
	end
	endgenerate
`else
	wire [7:0] ea = i_a[30:23];
	wire [7:0] ea_cmp = i_a[30:23];
	wire [7:0] eb = b_in[30:23];
	wire [7:0] eb_cmp = b_in[30:23];
`endif
	wire za = ea_cmp == 8'd0;
	wire zb = eb_cmp == 8'd0;
	wire nan_a = ea_cmp == 8'hFF && i_a[22:0] != 23'd0;
	wire nan_b = eb_cmp == 8'hFF && b_in[22:0] != 23'd0;
	wire inf_a = ea_cmp == 8'hFF && i_a[22:0] == 23'd0;
	wire inf_b = eb_cmp == 8'hFF && b_in[22:0] == 23'd0;

	/* 4+4 split carry-select exponent difference (<= 6 cmos3) */
	wire [7:0] dab, dba;
	wire ea_lt_eb, eb_lt_ea, dba_bo_unused;
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
`ifdef SYNTH_KINTEX7
	am_sub #(.W(8)) u_dab (.a(i_a[30:23]), .b(b_in[30:23]), .bi(1'b0), .d(dab), .bo(ea_lt_eb));
	am_sub #(.W(8)) u_dba (.a(b_in[30:23]), .b(i_a[30:23]), .bi(1'b0), .d(dba), .bo(eb_lt_ea));
	assign dba_bo_unused = 1'b0;
	wire use_dba_oh = !eb_lt_ea;
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(8)) u_dab (.a(i_a[30:23]), .b(b_in[30:23]), .bi(1'b0), .d(dab), .bo(ea_lt_eb));
	am_sub #(.W(8)) u_dba (.a(b_in[30:23]), .b(i_a[30:23]), .bi(1'b0), .d(dba), .bo(eb_lt_ea));
	assign dba_bo_unused = 1'b0;
	wire use_dba_oh = !eb_lt_ea;
`else
	/* Pure-LUT 4+4 zero-XOR borrow-lookahead exponent difference (<= 6 cmos3) */
	function [4:0] fadd_sub4_fast;
		input [3:0] a, b;
		input bi;
		reg [3:0] nb, p, g, c;
		begin
			nb = ~b;
			p = a ^ nb;
			g = a & nb;
			c[0] = ~bi;
			c[1] = g[0] | (p[0] & c[0]);
			c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & c[0]);
			c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & c[0]);
			fadd_sub4_fast = {
				~(g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & c[0])),
				p ^ c
			};
		end
	endfunction

	wire [3:0] dab_bg_h = (~ea[7:4]) & eb[7:4];
	wire [3:0] dab_bp_h = (~ea[7:4]) | eb[7:4];
	wire [1:0] dab_g2_h = {dab_bg_h[3] | (dab_bp_h[3] & dab_bg_h[2]), dab_bg_h[1] | (dab_bp_h[1] & dab_bg_h[0])};
	wire [1:0] dab_p2_h = {dab_bp_h[3] & dab_bp_h[2], dab_bp_h[1] & dab_bp_h[0]};
	wire dab_b8_0 = dab_g2_h[1] | (dab_p2_h[1] & dab_g2_h[0]);
	wire dab_b8_1 = dab_g2_h[1] | (dab_p2_h[1] & (dab_g2_h[0] | dab_p2_h[0]));

	wire [4:0] dab_l_5  = fadd_sub4_fast(ea[3:0], eb[3:0], 1'b0);
	wire [4:0] dab_h0_5 = fadd_sub4_fast(ea[7:4], eb[7:4], 1'b0);
	wire [4:0] dab_h1_5 = fadd_sub4_fast(ea[7:4], eb[7:4], 1'b1);
	wire [4:0] dba_l_5  = fadd_sub4_fast(eb[3:0], ea[3:0], 1'b0);
	wire [4:0] dba_h0_5 = fadd_sub4_fast(eb[7:4], ea[7:4], 1'b0);
	wire [4:0] dba_h1_5 = fadd_sub4_fast(eb[7:4], ea[7:4], 1'b1);
	wire [3:0] dab_l = dab_l_5[3:0], dab_h0 = dab_h0_5[3:0], dab_h1 = dab_h1_5[3:0];
	wire [3:0] dba_l = dba_l_5[3:0], dba_h0 = dba_h0_5[3:0], dba_h1 = dba_h1_5[3:0];
	assign dab      = {dab_b4 ? dab_h1 : dab_h0, dab_l};
	assign ea_lt_eb =  dab_b4 ? dab_b8_1 : dab_b8_0;
	assign dba           = {dba_b4 ? dba_h1 : dba_h0, dba_l};
	assign dba_bo_unused = dab_l_5[4] & dab_h0_5[4] & dab_h1_5[4] & dba_l_5[4] & dba_h0_5[4] & dba_h1_5[4];
	wire use_dba_oh = ea_lt_eb;
	wire eb_lt_ea_unused = 1'b0;
`endif

	/* Pure-LUT 3-bit CLA and 3-bit borrow-sub functions (0 arith operators) */
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

	/* Pure-LUT 3->9->31 comparison with explicit LUT6/LUT5/LUT3 under SYNTH_KINTEX7 (<= 4 LUTs total) */
	(* keep = "true" *) wire [10:0] lt0, eq0;
	genvar gc;
	generate
	for (gc = 0; gc < 10; gc = gc + 1) begin : g_cmp_s0
		wire [2:0] sa = i_a[3 * gc +: 3];
		wire [2:0] sb = b_in[3 * gc +: 3];
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq0 (
			.I0(sa[0]), .I1(sa[1]), .I2(sa[2]), .I3(sb[0]), .I4(sb[1]), .I5(sb[2]), .O(eq0[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'h7F3F1F0F07030100)) u_lt0 (
			.I0(sa[0]), .I1(sa[1]), .I2(sa[2]), .I3(sb[0]), .I4(sb[1]), .I5(sb[2]), .O(lt0[gc])
		);
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'h8040201008040201)) u_eq0 (
			.I0(sa[0]), .I1(sa[1]), .I2(sa[2]), .I3(sb[0]), .I4(sb[1]), .I5(sb[2]), .O(eq0[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'h7F3F1F0F07030100)) u_lt0 (
			.I0(sa[0]), .I1(sa[1]), .I2(sa[2]), .I3(sb[0]), .I4(sb[1]), .I5(sb[2]), .O(lt0[gc])
		);
`else
		assign eq0[gc] = (sa == sb);
		assign lt0[gc] = (~sa[2] & sb[2]) |
			((sa[2] ~^ sb[2]) & ((~sa[1] & sb[1]) | ((sa[1] ~^ sb[1]) & (~sa[0] & sb[0]))));
`endif
	end
	endgenerate
	assign eq0[10] = (i_a[30] == b_in[30]);
	assign lt0[10] = (~i_a[30] & b_in[30]);

	(* keep = "true" *) wire [3:0] lt1, eq1;
	generate
	for (gc = 0; gc < 3; gc = gc + 1) begin : g_cmp_s1
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'h80)) u_eq1 (
			.I0(eq0[3 * gc]), .I1(eq0[3 * gc + 1]), .I2(eq0[3 * gc + 2]), .O(eq1[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'hFEFCF0F0)) u_lt1 (
			.I0(lt0[3 * gc]), .I1(lt0[3 * gc + 1]), .I2(lt0[3 * gc + 2]),
			.I3(eq0[3 * gc + 1]), .I4(eq0[3 * gc + 2]), .O(lt1[gc])
		);
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'h80)) u_eq1 (
			.I0(eq0[3 * gc]), .I1(eq0[3 * gc + 1]), .I2(eq0[3 * gc + 2]), .O(eq1[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT5 #(.INIT(32'hFEFCF0F0)) u_lt1 (
			.I0(lt0[3 * gc]), .I1(lt0[3 * gc + 1]), .I2(lt0[3 * gc + 2]),
			.I3(eq0[3 * gc + 1]), .I4(eq0[3 * gc + 2]), .O(lt1[gc])
		);
`else
		assign eq1[gc] = eq0[3 * gc + 2] & eq0[3 * gc + 1] & eq0[3 * gc];
		assign lt1[gc] = lt0[3 * gc + 2] | (eq0[3 * gc + 2] & lt0[3 * gc + 1]) |
			(eq0[3 * gc + 2] & eq0[3 * gc + 1] & lt0[3 * gc]);
`endif
	end
	endgenerate
	assign eq1[3] = eq0[10] & eq0[9];
	assign lt1[3] = lt0[10] | (eq0[10] & lt0[9]);

`ifdef SYNTH_KINTEX7
	/* Level 3 explicit LUT3/LUT2 replicated per datapath slice + Level 4 LUT5/LUT6 mux (<= 4 LUTs total) */
	wire [6:0] cmp_lt_hi_k, cmp_eq_hi_k, cmp_lt_lo_k;
	generate
	for (gc = 0; gc < 7; gc = gc + 1) begin : g_cmp_s2_rep
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hEA)) u_clthi (
			.I0(lt1[3]), .I1(eq1[3]), .I2(lt1[2]), .O(cmp_lt_hi_k[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_ceqhi (
			.I0(eq1[3]), .I1(eq1[2]), .O(cmp_eq_hi_k[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hEA)) u_cltlo (
			.I0(lt1[1]), .I1(eq1[1]), .I2(lt1[0]), .O(cmp_lt_lo_k[gc])
		);
	end
	endgenerate
	wire cmp_lt_ctl = cmp_lt_hi_k[4] | (cmp_eq_hi_k[4] & cmp_lt_lo_k[4]);
	wire cmp_diff_unused = eq1[0];

	wire v1_w = i_valid;
	wire sp_nan1_w = nan_a || nan_b || (inf_a && inf_b && i_a[31] != b_in[31]);
	wire sp_inf1_w = !sp_nan1_w && (inf_a || inf_b);
	wire sp_zero1_w = !sp_nan1_w && !sp_inf1_w && (za && zb);
	wire sp1_w = sp_nan1_w || sp_inf1_w || sp_zero1_w;
	wire sx_norm1_w = cmp_lt_ctl ? b_in[31] : i_a[31];
	wire sp_sx1_w = !sp_nan1_w && (inf_a ? i_a[31] : inf_b ? b_in[31] : (i_a[31] && b_in[31]));
	wire sx1_w = sp1_w ? sp_sx1_w : sx_norm1_w;
	wire sub1_w = i_a[31] != b_in[31];
	(* keep = "true", dont_touch = "true" *) wire sub1_lo_k = i_a[31] ^ b_in[31];
	(* keep = "true", dont_touch = "true" *) wire sub1_hi_k = i_a[31] ^ b_in[31];
	wire y_zero1_w = za || zb;
	wire [7:0] ex1_w;
	wire [23:0] mx1_w, my1_w, my1_x_w;
	assign mx1_w[23] = 1'b1;
	assign my1_w[23] = 1'b1;
	assign my1_x_w[23] = ~sub1_w;
	/* LUT5 INIT for sel = I2 | (I1 & I0):
	 * sel ? I4 : I3 -> 32'hFF00_F0F0 with sel=I2|(I1&I0)
	 * sel mask over {I2,I1,I0}: bits 7,6,5,3 = 1 -> 8'b1110_1000 = 8'hE8?
	 * Check {I2,I1,I0}: 000->0, 001->0, 010->0, 011->1(I1&I0), 100->1(I2), 101->1, 110->1, 111->1 -> sel=8'hF8!
	 * Wait: I4=1,I3=0 -> {31..16}: I4=1,I3=1 -> 1; I4=1,I3=0 -> sel=F8; I4=0,I3=1 -> ~sel=07; I4=0,I3=0 -> 0 -> 32'hFF_F8_07_00!
	 * Actually {I4,I3,I2,I1,I0}:
	 * 00_xxx (I4=0,I3=0) -> 0x00
	 * 01_xxx (I4=0,I3=1) -> ~sel = 0x07
	 * 10_xxx (I4=1,I3=0) -> sel  = 0xF8
	 * 11_xxx (I4=1,I3=1) -> 0xFF
	 * So sel ? I4 : I3 = 32'hFF_F8_07_00!
	 */
	generate
	for (gc = 0; gc < 8; gc = gc + 1) begin : g_ex1_mux
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_exm (
			.I0(cmp_lt_lo_k[4]), .I1(cmp_eq_hi_k[4]), .I2(cmp_lt_hi_k[4]),
			.I3(ea[gc]), .I4(eb[gc]), .O(ex1_w[gc])
		);
	end
	for (gc = 0; gc < 12; gc = gc + 1) begin : g_m1_mux_lo
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_mxl (
			.I0(cmp_lt_lo_k[0]), .I1(cmp_eq_hi_k[0]), .I2(cmp_lt_hi_k[0]),
			.I3(i_a[gc]), .I4(b_in[gc]), .O(mx1_w[gc])
		);
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_myl (
			.I0(cmp_lt_lo_k[2]), .I1(cmp_eq_hi_k[2]), .I2(cmp_lt_hi_k[2]),
			.I3(b_in[gc]), .I4(i_a[gc]), .O(my1_w[gc])
		);
		(* keep = "true" *) LUT6 #(.INIT(64'h0007F8FFFFF80700)) u_myxl (
			.I0(cmp_lt_lo_k[5]), .I1(cmp_eq_hi_k[5]), .I2(cmp_lt_hi_k[5]),
			.I3(b_in[gc]), .I4(i_a[gc]), .I5(sub1_lo_k), .O(my1_x_w[gc])
		);
	end
	for (gc = 12; gc < 23; gc = gc + 1) begin : g_m1_mux_hi
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_mxh (
			.I0(cmp_lt_lo_k[1]), .I1(cmp_eq_hi_k[1]), .I2(cmp_lt_hi_k[1]),
			.I3(i_a[gc]), .I4(b_in[gc]), .O(mx1_w[gc])
		);
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_myh (
			.I0(cmp_lt_lo_k[3]), .I1(cmp_eq_hi_k[3]), .I2(cmp_lt_hi_k[3]),
			.I3(b_in[gc]), .I4(i_a[gc]), .O(my1_w[gc])
		);
		(* keep = "true" *) LUT6 #(.INIT(64'h0007F8FFFFF80700)) u_myxh (
			.I0(cmp_lt_lo_k[6]), .I1(cmp_eq_hi_k[6]), .I2(cmp_lt_hi_k[6]),
			.I3(b_in[gc]), .I4(i_a[gc]), .I5(sub1_hi_k), .O(my1_x_w[gc])
		);
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Level 3 explicit LUT3/LUT2 replicated per datapath slice + Level 4 LUT5/LUT6 mux (<= 4 LUTs total) */
	wire [6:0] cmp_lt_hi_k, cmp_eq_hi_k, cmp_lt_lo_k;
	generate
	for (gc = 0; gc < 7; gc = gc + 1) begin : g_cmp_s2_rep
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hEA)) u_clthi (
			.I0(lt1[3]), .I1(eq1[3]), .I2(lt1[2]), .O(cmp_lt_hi_k[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_ceqhi (
			.I0(eq1[3]), .I1(eq1[2]), .O(cmp_eq_hi_k[gc])
		);
		(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hEA)) u_cltlo (
			.I0(lt1[1]), .I1(eq1[1]), .I2(lt1[0]), .O(cmp_lt_lo_k[gc])
		);
	end
	endgenerate
	wire cmp_lt_ctl = cmp_lt_hi_k[4] | (cmp_eq_hi_k[4] & cmp_lt_lo_k[4]);
	wire cmp_diff_unused = eq1[0];

	wire v1_w = i_valid;
	wire sp_nan1_w = nan_a || nan_b || (inf_a && inf_b && i_a[31] != b_in[31]);
	wire sp_inf1_w = !sp_nan1_w && (inf_a || inf_b);
	wire sp_zero1_w = !sp_nan1_w && !sp_inf1_w && (za && zb);
	wire sp1_w = sp_nan1_w || sp_inf1_w || sp_zero1_w;
	wire sx_norm1_w = cmp_lt_ctl ? b_in[31] : i_a[31];
	wire sp_sx1_w = !sp_nan1_w && (inf_a ? i_a[31] : inf_b ? b_in[31] : (i_a[31] && b_in[31]));
	wire sx1_w = sp1_w ? sp_sx1_w : sx_norm1_w;
	wire sub1_w = i_a[31] != b_in[31];
	(* keep = "true", dont_touch = "true" *) wire sub1_lo_k = i_a[31] ^ b_in[31];
	(* keep = "true", dont_touch = "true" *) wire sub1_hi_k = i_a[31] ^ b_in[31];
	wire y_zero1_w = za || zb;
	wire [7:0] ex1_w;
	wire [23:0] mx1_w, my1_w, my1_x_w;
	assign mx1_w[23] = 1'b1;
	assign my1_w[23] = 1'b1;
	assign my1_x_w[23] = ~sub1_w;
	/* LUT5 INIT for sel = I2 | (I1 & I0):
	 * sel ? I4 : I3 -> 32'hFF00_F0F0 with sel=I2|(I1&I0)
	 * sel mask over {I2,I1,I0}: bits 7,6,5,3 = 1 -> 8'b1110_1000 = 8'hE8?
	 * Check {I2,I1,I0}: 000->0, 001->0, 010->0, 011->1(I1&I0), 100->1(I2), 101->1, 110->1, 111->1 -> sel=8'hF8!
	 * Wait: I4=1,I3=0 -> {31..16}: I4=1,I3=1 -> 1; I4=1,I3=0 -> sel=F8; I4=0,I3=1 -> ~sel=07; I4=0,I3=0 -> 0 -> 32'hFF_F8_07_00!
	 * Actually {I4,I3,I2,I1,I0}:
	 * 00_xxx (I4=0,I3=0) -> 0x00
	 * 01_xxx (I4=0,I3=1) -> ~sel = 0x07
	 * 10_xxx (I4=1,I3=0) -> sel  = 0xF8
	 * 11_xxx (I4=1,I3=1) -> 0xFF
	 * So sel ? I4 : I3 = 32'hFF_F8_07_00!
	 */
	generate
	for (gc = 0; gc < 8; gc = gc + 1) begin : g_ex1_mux
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_exm (
			.I0(cmp_lt_lo_k[4]), .I1(cmp_eq_hi_k[4]), .I2(cmp_lt_hi_k[4]),
			.I3(ea[gc]), .I4(eb[gc]), .O(ex1_w[gc])
		);
	end
	for (gc = 0; gc < 12; gc = gc + 1) begin : g_m1_mux_lo
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_mxl (
			.I0(cmp_lt_lo_k[0]), .I1(cmp_eq_hi_k[0]), .I2(cmp_lt_hi_k[0]),
			.I3(i_a[gc]), .I4(b_in[gc]), .O(mx1_w[gc])
		);
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_myl (
			.I0(cmp_lt_lo_k[2]), .I1(cmp_eq_hi_k[2]), .I2(cmp_lt_hi_k[2]),
			.I3(b_in[gc]), .I4(i_a[gc]), .O(my1_w[gc])
		);
		(* keep = "true" *) LUT6 #(.INIT(64'h0007F8FFFFF80700)) u_myxl (
			.I0(cmp_lt_lo_k[5]), .I1(cmp_eq_hi_k[5]), .I2(cmp_lt_hi_k[5]),
			.I3(b_in[gc]), .I4(i_a[gc]), .I5(sub1_lo_k), .O(my1_x_w[gc])
		);
	end
	for (gc = 12; gc < 23; gc = gc + 1) begin : g_m1_mux_hi
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_mxh (
			.I0(cmp_lt_lo_k[1]), .I1(cmp_eq_hi_k[1]), .I2(cmp_lt_hi_k[1]),
			.I3(i_a[gc]), .I4(b_in[gc]), .O(mx1_w[gc])
		);
		(* keep = "true" *) LUT5 #(.INIT(32'hFFF80700)) u_myh (
			.I0(cmp_lt_lo_k[3]), .I1(cmp_eq_hi_k[3]), .I2(cmp_lt_hi_k[3]),
			.I3(b_in[gc]), .I4(i_a[gc]), .O(my1_w[gc])
		);
		(* keep = "true" *) LUT6 #(.INIT(64'h0007F8FFFFF80700)) u_myxh (
			.I0(cmp_lt_lo_k[6]), .I1(cmp_eq_hi_k[6]), .I2(cmp_lt_hi_k[6]),
			.I3(b_in[gc]), .I4(i_a[gc]), .I5(sub1_hi_k), .O(my1_x_w[gc])
		);
	end
	endgenerate
`else
	(* keep = "true", dont_touch = "true" *) wire [4:0] lt_hi_r, lt_lo_r;
	generate
	for (gc = 0; gc < 5; gc = gc + 1) begin : g_cmp_s2_rep
		assign lt_hi_r[gc] = lt1[3] | (eq1[3] & lt1[2]);
		assign lt_lo_r[gc] = eq1[3] & eq1[2] & (lt1[1] | (eq1[1] & lt1[0]));
	end
	endgenerate
	wire cmp_lt_mx_lo = lt_hi_r[0] | lt_lo_r[0];
	wire cmp_lt_mx_hi = lt_hi_r[1] | lt_lo_r[1];
	wire cmp_lt_my_lo = lt_hi_r[2] | lt_lo_r[2];
	wire cmp_lt_my_hi = lt_hi_r[3] | lt_lo_r[3];
	wire cmp_lt_ctl   = lt_hi_r[4] | lt_lo_r[4];
	wire cmp_diff_unused = eq1[0];

	wire v1_w = i_valid;
	wire sp_nan1_w = nan_a || nan_b || (inf_a && inf_b && i_a[31] != b_in[31]);
	wire sp_inf1_w = !sp_nan1_w && (inf_a || inf_b);
	wire sp_zero1_w = !sp_nan1_w && !sp_inf1_w && (za && zb);
	wire sp1_w = sp_nan1_w || sp_inf1_w || sp_zero1_w;
	wire sx_norm1_w = cmp_lt_ctl ? b_in[31] : i_a[31];
	wire sp_sx1_w = !sp_nan1_w && (inf_a ? i_a[31] : inf_b ? b_in[31] : (i_a[31] && b_in[31]));
	wire sx1_w = sp1_w ? sp_sx1_w : sx_norm1_w;
	wire sub1_w = i_a[31] != b_in[31];
	(* keep = "true" *) wire sub1_lo_k = i_a[31] ^ b_in[31];
	(* keep = "true" *) wire sub1_hi_k = i_a[31] ^ b_in[31];
	wire y_zero1_w = za || zb;
	wire [7:0] ex1_w;
	wire [23:0] mx1_w, my1_w, my1_x_w;
	assign mx1_w[23] = 1'b1;
	assign my1_w[23] = 1'b1;
	assign my1_x_w[23] = ~sub1_w;
	generate
	for (gc = 0; gc < 8; gc = gc + 1) begin : g_ex1_mux
		assign ex1_w[gc] = cmp_lt_ctl ? eb[gc] : ea[gc];
	end
	for (gc = 0; gc < 12; gc = gc + 1) begin : g_m1_mux_lo
		assign mx1_w[gc]   = cmp_lt_mx_lo ? b_in[gc]   : i_a[gc];
		assign my1_w[gc]   = cmp_lt_my_lo ? i_a[gc]    : b_in[gc];
		assign my1_x_w[gc] = sub1_lo_k ^ ((lt_hi_r[2] | lt_lo_r[2]) ? i_a[gc] : b_in[gc]);
	end
	for (gc = 12; gc < 23; gc = gc + 1) begin : g_m1_mux_hi
		assign mx1_w[gc]   = cmp_lt_mx_hi ? b_in[gc]   : i_a[gc];
		assign my1_w[gc]   = cmp_lt_my_hi ? i_a[gc]    : b_in[gc];
		assign my1_x_w[gc] = sub1_hi_k ^ ((lt_hi_r[3] | lt_lo_r[3]) ? i_a[gc] : b_in[gc]);
	end
	endgenerate
`endif
	wire [4:0] d1_w = ea_lt_eb ? dba[4:0] : dab[4:0];
	wire ealtb1_w = ea_lt_eb;
	/* Direct fast [7:5] in-range and bit-4 difference without waiting for full dab[7:4] ripple (<= 8 cmos3) */
	wire d_b5_cmp = ((~ea[4]) & eb[4]) | (((~ea[4]) | eb[4]) & dab_b4);
	wire d_b5_ba  = ((~eb[4]) & ea[4]) | (((~eb[4]) | ea[4]) & dba_b4);
	wire [2:0] ea_hi3_p1 = ea[7:5] ^ 3'b001 ^ {ea[5] & ea[6] ? 1'b1 : 1'b0, ea[5], 1'b0};
	wire [2:0] eb_hi3_p1 = eb[7:5] ^ 3'b001 ^ {eb[5] & eb[6] ? 1'b1 : 1'b0, eb[5], 1'b0};
	wire hi3_eq  = (ea[7:5] == eb[7:5]);
	wire hi3_ab1 = (ea[7:5] == eb_hi3_p1);
	wire hi3_ba1 = (eb[7:5] == ea_hi3_p1);
	wire dab_hi1_w = !((hi3_eq && !d_b5_cmp) || (hi3_ab1 && d_b5_cmp));
	wire dba_hi1_w = !((hi3_eq && !d_b5_ba)  || (hi3_ba1 && d_b5_ba));
	wire dab_cc1_w = (dab[4:1] == 4'd0);
	wire dba_cc1_w = (dba[4:1] == 4'd0);
	wire cc_en1_w = (i_a[31] != b_in[31]) && !za && !zb;

	wire dab_in32 = !y_zero1_w && ((hi3_eq && !d_b5_cmp) || (hi3_ab1 && d_b5_cmp));
	wire dba_in32 = !y_zero1_w && ((hi3_eq &&  d_b5_cmp) || (hi3_ba1 && !d_b5_cmp));
	wire [2:0] d42_dab_w = {ea[4] ^ eb[4] ^ dab_b4, dab[3:2]};
	wire [2:0] d42_dba_w = {eb[4] ^ ea[4] ^ dba_b4, dba[3:2]};
	wire [4:0] dab5_fast_w = {ea[4] ^ eb[4] ^ dab_b4, dab[3:0]};
	wire [4:0] dba5_fast_w = {eb[4] ^ ea[4] ^ dba_b4, dba[3:0]};

	wire [6:0] d1_coarse_oh_w;
	genvar gco;
	generate
	for (gco = 0; gco < 7; gco = gco + 1) begin : g_d1_coh
		assign d1_coarse_oh_w[gco] =
			(dab_in32 && (d42_dab_w == gco[2:0])) ||
			(dba_in32 && (d42_dba_w == gco[2:0]));
	end
	endgenerate

	wire [3:0] d1_lo_dab  = { dab[1] &  dab[0],  dab[1] & ~dab[0], ~dab[1] &  dab[0], ~dab[1] & ~dab[0]};
	wire [3:0] d1_lo_dba  = { dba[1] &  dba[0],  dba[1] & ~dba[0], ~dba[1] &  dba[0], ~dba[1] & ~dba[0]};
	wire [3:0] d1_mid_dab = { dab[3] &  dab[2],  dab[3] & ~dab[2], ~dab[3] &  dab[2], ~dab[3] & ~dab[2]};
	wire [3:0] d1_mid_dba = { dba[3] &  dba[2],  dba[3] & ~dba[2], ~dba[3] &  dba[2], ~dba[3] & ~dba[2]};
	wire [3:0] d1_lo_oh_w  = use_dba_oh ? d1_lo_dba  : d1_lo_dab;
	wire [3:0] d1_mid_oh_w = use_dba_oh ? d1_mid_dba : d1_mid_dab;
	wire [3:0] d1_lo_oh_a, d1_lo_oh_b, d1_lo_oh_d;
	wire [3:0] d1_mid_oh_a, d1_mid_oh_b, d1_mid_oh_d;
	wire [6:0] d1_coarse_oh_a, d1_coarse_oh_b, d1_coarse_oh_d;

	/* Stage 1 sticky threshold vector {ge27..ge3} (<= 9 cmos3); part/enable moved to Stage 2 */
	wire [4:0] dab5_w = dab5_fast_w;
	wire [4:0] dba5_w = dba5_fast_w;
	function automatic [6:0] stk_ge_fn;
		input [4:0] d5;
		reg ge3, ge7, ge11, ge15, ge19, ge23, ge27;
		begin
			ge3  = d5[4] | d5[3] | d5[2] | (d5[1] & d5[0]);
			ge7  = d5[4] | d5[3] | (&d5[2:0]);
			ge11 = d5[4] | (d5[3] & (d5[2] | (d5[1] & d5[0])));
			ge15 = d5[4] | (&d5[3:0]);
			ge19 = d5[4] & (d5[3] | d5[2] | (d5[1] & d5[0]));
			ge23 = d5[4] & (d5[3] | (&d5[2:0]));
			ge27 = d5[4] & d5[3] & (d5[2] | (d5[1] & d5[0]));
			stk_ge_fn = {ge27, ge23, ge19, ge15, ge11, ge7, ge3};
		end
	endfunction
	wire [6:0] dab_stk_ge = stk_ge_fn(dab5_w);
	wire [6:0] dba_stk_ge = stk_ge_fn(dba5_w);
	wire [6:0] d1_stk_ge_w = use_dba_oh ? dba_stk_ge : dab_stk_ge;
	wire y_clr_w = y_zero1_w || (ea_lt_eb ? (dba_hi1_w || (&d42_dba_w)) : (dab_hi1_w || (&d42_dab_w)));
	wire py2_hi1_w = !y_zero1_w && (ea_lt_eb ? dba_hi1_w : dab_hi1_w);
	wire [6:0] d1_stk_ge;
	wire y_clr_lo, y_clr_hi;
	wire py2_hi1;
	wire ex_xor_eb_0_w = ea[0] ^ eb[0];
	wire ex_xor_eb_0_a, ex_xor_eb_0_b, ex_xor_eb_0_c, ex_xor_eb_0_d;

	wire v1, sp1, sp_nan1, sp_inf1, sx1, sub1_a, sub1_b, y_zero1_a, y_zero1_b, close_cand1;
	wire ealtb1, dab_hi1, dba_hi1;
	wire [7:0] ex1;
	wire [4:0] d1_a, d1_a_hi, d1_b, d1_c, d1_d;
	wire d1_e0, d1_f0;
	wire [23:0] mx1, my1, my1_b, my1_c, my1_x;

	/* Stage 2:
	 * - Far path: fast 2-level barrel right shift (4:1 + 7:1 mux), chunked sticky bit, and XOR with sub1.
	 * - Close path: 25-bit exact subtraction for d1 <= 1.
	 */
	wire [26:0] shi = {my1_x, {3{sub1_b}}};
	wire [26:0] shr_0, shr_1, shr_2;
`ifdef SYNTH_ASIC
	wire [26:0] sh_s0_lo, sh_s0_mid, sh_s0_hi;
	wire [26:0] sh_s1_lo, sh_s1_mid, sh_s1_hi;
	genvar gd1;
	generate
	for (gd1 = 0; gd1 < 27; gd1 = gd1 + 1) begin : g_fsh0
		wire s_b1 = (gd1 <= 25) ? shi[gd1 + 1] : sub1_b;
		wire s_b2 = (gd1 <= 24) ? shi[gd1 + 2] : sub1_b;
		wire s_b3 = (gd1 <= 23) ? shi[gd1 + 3] : sub1_b;
		assign sh_s0_lo[gd1]  = (d1_lo_oh_a[0] & shi[gd1]) | (d1_lo_oh_a[1] & s_b1) | (d1_lo_oh_a[2] & s_b2) | (d1_lo_oh_a[3] & s_b3);
		assign sh_s0_mid[gd1] = (d1_lo_oh_d[0] & shi[gd1]) | (d1_lo_oh_d[1] & s_b1) | (d1_lo_oh_d[2] & s_b2) | (d1_lo_oh_d[3] & s_b3);
		assign sh_s0_hi[gd1]  = (d1_lo_oh_b[0] & shi[gd1]) | (d1_lo_oh_b[1] & s_b1) | (d1_lo_oh_b[2] & s_b2) | (d1_lo_oh_b[3] & s_b3);
	end
	for (gd1 = 0; gd1 < 27; gd1 = gd1 + 1) begin : g_fsh1
		wire [6:0] lo_t, mid_t, hi_t;
		genvar gc7;
		for (gc7 = 0; gc7 < 7; gc7 = gc7 + 1) begin : g_c7
			if (gd1 + 4*gc7 < 27) begin : g_in
				assign lo_t[gc7]  = d1_coarse_oh_a[gc7] & sh_s0_lo[gd1 + 4*gc7];
				assign mid_t[gc7] = d1_coarse_oh_d[gc7] & sh_s0_mid[gd1 + 4*gc7];
				assign hi_t[gc7]  = d1_coarse_oh_b[gc7] & sh_s0_hi[gd1 + 4*gc7];
			end else begin : g_z
				assign lo_t[gc7]  = d1_coarse_oh_a[gc7] & sub1_a;
				assign mid_t[gc7] = d1_coarse_oh_d[gc7] & sub1_a;
				assign hi_t[gc7]  = d1_coarse_oh_b[gc7] & sub1_b;
			end
		end
		assign shr_0[gd1] = y_clr_lo ? sub1_a : (|lo_t);
		assign shr_1[gd1] = y_clr_lo ? sub1_a : (|mid_t);
		assign shr_2[gd1] = y_clr_hi ? sub1_b : (|hi_t);
		assign sh_s1_lo[gd1]  = 1'b0;
		assign sh_s1_mid[gd1] = 1'b0;
		assign sh_s1_hi[gd1]  = 1'b0;
	end
	endgenerate
`else
	wire [26:0] shi_sim = {my1, 3'b000};
	reg [26:0] sh_s0_lo, sh_s0_mid, sh_s0_hi;
	reg [26:0] sh_s1_lo, sh_s1_mid, sh_s1_hi;
	integer si0;
	always @(*) begin
		for (si0 = 0; si0 < 27; si0 = si0 + 1) begin
			if (si0 < 14) begin
				case (d1_a[1:0])
				2'd0: sh_s0_lo[si0] = shi_sim[si0];
				2'd1: sh_s0_lo[si0] = (si0 <= 25) ? shi_sim[si0 + 1] : 1'b0;
				2'd2: sh_s0_lo[si0] = (si0 <= 24) ? shi_sim[si0 + 2] : 1'b0;
				default: sh_s0_lo[si0] = (si0 <= 23) ? shi_sim[si0 + 3] : 1'b0;
				endcase
			end else begin
				case (d1_a_hi[1:0])
				2'd0: sh_s0_lo[si0] = shi_sim[si0];
				2'd1: sh_s0_lo[si0] = (si0 <= 25) ? shi_sim[si0 + 1] : 1'b0;
				2'd2: sh_s0_lo[si0] = (si0 <= 24) ? shi_sim[si0 + 2] : 1'b0;
				default: sh_s0_lo[si0] = (si0 <= 23) ? shi_sim[si0 + 3] : 1'b0;
				endcase
			end
			case (d1_d[1:0])
			2'd0: sh_s0_mid[si0] = shi_sim[si0];
			2'd1: sh_s0_mid[si0] = (si0 <= 25) ? shi_sim[si0 + 1] : 1'b0;
			2'd2: sh_s0_mid[si0] = (si0 <= 24) ? shi_sim[si0 + 2] : 1'b0;
			default: sh_s0_mid[si0] = (si0 <= 23) ? shi_sim[si0 + 3] : 1'b0;
			endcase
			case (d1_b[1:0])
			2'd0: sh_s0_hi[si0] = shi_sim[si0];
			2'd1: sh_s0_hi[si0] = (si0 <= 25) ? shi_sim[si0 + 1] : 1'b0;
			2'd2: sh_s0_hi[si0] = (si0 <= 24) ? shi_sim[si0 + 2] : 1'b0;
			default: sh_s0_hi[si0] = (si0 <= 23) ? shi_sim[si0 + 3] : 1'b0;
			endcase
		end
		for (si0 = 0; si0 < 27; si0 = si0 + 1) begin
			case (d1_a[3:2])
			2'd0: sh_s1_lo[si0] = sh_s0_lo[si0];
			2'd1: sh_s1_lo[si0] = (si0 <= 22) ? sh_s0_lo[si0 + 4] : 1'b0;
			2'd2: sh_s1_lo[si0] = (si0 <= 18) ? sh_s0_lo[si0 + 8] : 1'b0;
			default: sh_s1_lo[si0] = (si0 <= 14) ? sh_s0_lo[si0 + 12] : 1'b0;
			endcase
			case (d1_d[3:2])
			2'd0: sh_s1_mid[si0] = sh_s0_mid[si0];
			2'd1: sh_s1_mid[si0] = (si0 <= 22) ? sh_s0_mid[si0 + 4] : 1'b0;
			2'd2: sh_s1_mid[si0] = (si0 <= 18) ? sh_s0_mid[si0 + 8] : 1'b0;
			default: sh_s1_mid[si0] = (si0 <= 14) ? sh_s0_mid[si0 + 12] : 1'b0;
			endcase
			case (d1_b[3:2])
			2'd0: sh_s1_hi[si0] = sh_s0_hi[si0];
			2'd1: sh_s1_hi[si0] = (si0 <= 22) ? sh_s0_hi[si0 + 4] : 1'b0;
			2'd2: sh_s1_hi[si0] = (si0 <= 18) ? sh_s0_hi[si0 + 8] : 1'b0;
			default: sh_s1_hi[si0] = (si0 <= 14) ? sh_s0_hi[si0 + 12] : 1'b0;
			endcase
		end
	end
`endif

	(* keep = "true" *) wire y_clr_lo_sim = y_zero1_a || (ealtb1 ? dba_hi1 : dab_hi1);
	(* keep = "true" *) wire y_clr_hi_sim = y_zero1_b || (ealtb1 ? dba_hi1 : dab_hi1);
`ifndef SYNTH_ASIC
	genvar gd4;
	generate
	for (gd4 = 0; gd4 < 27; gd4 = gd4 + 1) begin : g_shr_s1
		wire s16_lo, s16_mid, s16_hi;
		if (gd4 + 16 < 27) begin : g_s16
			assign s16_lo  = sh_s1_lo[gd4 + 16];
			assign s16_mid = sh_s1_mid[gd4 + 16];
			assign s16_hi  = sh_s1_hi[gd4 + 16];
		end else begin : g_s16z
			assign s16_lo  = 1'b0;
			assign s16_mid = 1'b0;
			assign s16_hi  = 1'b0;
		end
		assign shr_0[gd4] = !y_clr_lo_sim && (d1_a[4] ? s16_lo  : sh_s1_lo[gd4]);
		assign shr_1[gd4] = !y_clr_lo_sim && (d1_d[4] ? s16_mid : sh_s1_mid[gd4]);
		assign shr_2[gd4] = !y_clr_hi_sim && (d1_b[4] ? s16_hi  : sh_s1_hi[gd4]);
	end
	endgenerate
`endif

	wire [11:0] d1_stk_sel = {
		d1_stk_ge[6:1] & {6{!y_zero1_a}},
		d1_stk_ge[5:0] & ~d1_stk_ge[6:1] & {6{!y_zero1_a}}
	};
	wire [5:0] chk_stk_n;
	genvar gs;
	generate
	for (gs = 0; gs < 6; gs = gs + 1) begin : g_stk_chunk
		wire [3:0] ch = my1_b[4 * gs +: 4];
		wire ch_lo_n = !(ch[0] | ch[1]);
		wire ch_hi_n = !(ch[2] | ch[3]);
		wire st_full_n = ch_lo_n & ch_hi_n;
		wire p_c23_n = !(((d1_lo_oh_d[1] | d1_lo_oh_d[2]) & ch[2]) | (d1_lo_oh_d[2] & ch[3]));
		wire p_c01_n = !(ch[0] | (!d1_lo_oh_d[3] & ch[1]));
		wire st_part_n = p_c23_n & p_c01_n;
		wire full_hit_n = !(d1_stk_sel[gs + 6] & !st_full_n);
		wire part_hit_n = !(d1_stk_sel[gs]     & !st_part_n);
		assign chk_stk_n[gs] = full_hit_n & part_hit_n;
	end
	endgenerate

	wire stk_g0 = !(chk_stk_n[0] & chk_stk_n[1]);
	wire stk_g1 = !(chk_stk_n[2] & chk_stk_n[3]);
	wire stk_g2 = !(chk_stk_n[4] & chk_stk_n[5]);
	wire stk_lo_n = !(stk_g0 | stk_g1);
	wire stk_hi_n = !(stk_g2 | py2_hi1);
	wire py2_bit0_stk = !(stk_lo_n & stk_hi_n);
`ifdef SYNTH_ASIC
	wire py2_bit0_w = sub1_a ? (shr_0[0] & ~py2_bit0_stk) : (shr_0[0] | py2_bit0_stk);
	wire [26:0] py2_w = {shr_2[26:15], shr_1[14:8], shr_0[7:1], py2_bit0_w};
	wire [26:0] py2_raw = 27'd0;
`else
	wire py2_bit0 = py2_bit0_stk | shr_0[0];
	wire [13:0] py2_raw_lo = {{shr_1[13:8], shr_0[7:1]}, py2_bit0};
	wire [26:14] py2_raw_hi = {shr_2[26:15], shr_1[14]};
	wire [26:0] py2_raw = {py2_raw_hi, py2_raw_lo};
`endif

	wire [24:0] ca_eff = {mx1, 1'b0};
	wire [24:0] cb_eff = {
		ex_xor_eb_0_c ? {1'b0, my1_c[23:16]} : my1_c[23:15],
		ex_xor_eb_0_b ? my1_c[15:8] : my1_c[14:7],
		ex_xor_eb_0_a ? my1_c[7:0]  : {my1_c[6:0], 1'b0}
	};
	wire [24:0] close_diff2_w;
	wire close_sub_bo_unused;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(25)) u_close_sub (.a(ca_eff), .b(cb_eff), .bi(1'b0), .d(close_diff2_w), .bo(close_sub_bo_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(25)) u_close_sub (.a(ca_eff), .b(cb_eff), .bi(1'b0), .d(close_diff2_w), .bo(close_sub_bo_unused));
`else
	/* 7+6+6+6 fast borrow-lookahead close subtractor with replicated ex_xor_eb_0_d for bit0 (<= 9 cmos3) */
	wire [24:0] cb_eff_b7 = {
		cb_eff[24:7],
		ex_xor_eb_0_d ? my1_c[6:0] : {my1_c[5:0], 1'b0}
	};
	wire [24:0] cbg_i = (~ca_eff) & cb_eff_b7;
	wire [24:0] cbp_i = (~ca_eff) | cb_eff_b7;
	wire [2:0] cbg3_0, cbp3_0;
	wire [1:0] cbg3_1, cbp3_1;
	wire cb_g2_0 = cbg_i[1] | (cbp_i[1] & cbg_i[0]);
	wire cb_p2_0 = cbp_i[1] & cbp_i[0];
	wire cb_g2_1 = cbg_i[3] | (cbp_i[3] & cbg_i[2]);
	wire cb_p2_1 = cbp_i[3] & cbp_i[2];
	wire cb_g4_0 = cb_g2_1 | (cb_p2_1 & cb_g2_0);
	wire cb_p4_0 = cb_p2_1 & cb_p2_0;
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

	/* Zero-XOR carry-lookahead for cdl_s[6:0] using cbg_i/cbp_i[6:0] (<= 8 cmos3 vs 13) */
	wire [6:0] cdl_px = ca_eff[6:0] ^ cb_eff[6:0];
	wire cdl_c1 = cbg_i[0];
	wire cdl_c2 = cb_g2_0;
	wire cdl_c3 = cbg_i[2] | (cbp_i[2] & cb_g2_0);
	wire cdl_c4 = cb_g4_0;
	wire cdl_c5 = cbg_i[4] | (cbp_i[4] & cb_g4_0);
	wire cdl_g54 = cbg_i[5] | (cbp_i[5] & cbg_i[4]);
	wire cdl_p54 = cbp_i[5] & cbp_i[4];
	wire cdl_c6 = cdl_g54 | (cdl_p54 & cb_g4_0);
	wire [6:0] cdl_s = cdl_px ^ {cdl_c6, cdl_c5, cdl_c4, cdl_c3, cdl_c2, cdl_c1, 1'b0};
	wire [3:0] cdl_64_4 = 4'd0;

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

	assign close_diff2_w = {
		cb_b19 ? cdh3_1 : cdh3_0,
		cb_b13 ? cdh2_1 : cdh2_0,
		cb_b7  ? cdm_1  : cdm_0,
		cdl_s
	};
	assign close_sub_bo_unused = &{1'b0, d1_e0, d1_f0, cb_p4_0, cbg3_0[0], cbp3_0[0], cdl_64_4[3],
		cdm_0_h[3], cdm_1_h[3], cdh2_0_h[3], cdh2_1_h[3], cdh3_0_h[3], cdh3_1_h[3]};
`endif

	wire [7:0] ex2_p1_w, ex2_sub1_w;
	wire ex2_p1_co_unused, ex2_sub1_bo_unused;
	am_add #(.W(8)) u_ex2_p1 (.a(ex1), .b(8'd1), .ci(1'b0), .s(ex2_p1_w), .co(ex2_p1_co_unused));
	am_sub #(.W(8)) u_ex2_sub1 (.a(ex1), .b(8'd1), .bi(1'b0), .d(ex2_sub1_w), .bo(ex2_sub1_bo_unused));

	wire v2_w = v1;
	wire sp2_w = sp1;
	wire sp_nan2_w = sp_nan1;
	wire sp_inf2_w = sp_inf1;
	wire sx2_w = sx1;
	wire sub2_w = sub1_a;
	wire close_cand2_w = close_cand1;
	wire [7:0] ex2_w = ex1;
	wire [26:0] px2_w = {mx1, 3'b000};
`ifndef SYNTH_ASIC
	wire [26:0] py2_w = {py2_raw[26:14] ^ {13{sub1_b}}, py2_raw[13:0] ^ {14{sub1_a}}};
`endif

	wire v2, sp2, sp_nan2, sp_inf2, sx2, sub2, close_cand2;
	wire [7:0] ex2, ex2_p1, ex2_sub1;
	wire [26:0] px2, py2;
	wire [24:0] close_diff2;

	/* Stage 3:
	 * - Far path: 28-bit addition u_sum, plus 1-LUT slice prefix-ANDs and low-bit rounding.
	 * - Close path: LZC and zero-detect from registered close_diff2 (0 CARRY4s).
	 */
	wire [27:0] sum;
	wire sum_co_unused;
`ifdef SYNTH_KINTEX7
	am_add #(.W(28)) u_sum (.a({1'b0, px2}), .b({sub2, py2}), .ci(sub2), .s(sum), .co(sum_co_unused));
	wire far_c_w = sum[27];
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(28)) u_sum (.a({1'b0, px2}), .b({sub2, py2}), .ci(sub2), .s(sum), .co(sum_co_unused));
	wire far_c_w = sum[27];
`else
	/* Speculative 6-bit carry-select with alternating OAI3/AOI3 2->4->8->16 prefix CLA (<= 10 cmos3) */
	wire [5:0] sum_s0;
	wire [2:0] sum_s0_h0, sum_s0_h1;
	wire sum_c3, sum_c6_s0, sum_c6_s1;
	wire [5:0] sum_6_0, sum_6_1, sum_12_0, sum_12_1, sum_18_0, sum_18_1;
	wire [3:0] sum_t0, sum_t1;
	wire sum_c6, sum_c12, sum_c18, sum_c24;
	wire [11:0] sum_c12_d_un;
	wire [17:0] sum_c18_d_un;
	wire [23:0] sum_c24_d_un;
	wire sum_c6_0_un, sum_c6_1_un, sum_c12_0_un, sum_c12_1_un, sum_c18_0_un, sum_c18_1_un, sum_co_t0, sum_co_t1;

	wire [26:0] cla_gn = ~(px2[26:0] & py2[26:0]);
	wire [26:0] cla_pn = ~(px2[26:0] | py2[26:0]);
	wire [11:0] cla_g2, cla_p2;
	genvar g3i;
	generate
	for (g3i = 0; g3i < 12; g3i = g3i + 1) begin : g_cla2
		assign cla_g2[g3i] = !(cla_gn[2*g3i+1] & (cla_pn[2*g3i+1] | cla_gn[2*g3i]));
		assign cla_p2[g3i] = !(cla_pn[2*g3i+1] | cla_pn[2*g3i]);
	end
	wire [5:0] cla_gn4, cla_pn4;
	for (g3i = 0; g3i < 6; g3i = g3i + 1) begin : g_cla4
		assign cla_gn4[g3i] = !(cla_g2[2*g3i+1] | (cla_p2[2*g3i+1] & cla_g2[2*g3i]));
		assign cla_pn4[g3i] = !(cla_p2[2*g3i+1] & cla_p2[2*g3i]);
	end
	wire [2:0] cla_g8, cla_p8;
	for (g3i = 0; g3i < 3; g3i = g3i + 1) begin : g_cla8
		assign cla_g8[g3i] = !(cla_gn4[2*g3i+1] & (cla_pn4[2*g3i+1] | cla_gn4[2*g3i]));
		assign cla_p8[g3i] = !(cla_pn4[2*g3i+1] | cla_pn4[2*g3i]);
	end
	endgenerate
	wire cla_gn16 = !(cla_g8[1] | (cla_p8[1] & cla_g8[0]));
	wire cla_pn16 = !(cla_p8[1] & cla_p8[0]);
	wire cla_c8_n  = !(cla_g8[0] | (cla_p8[0] & sub2));
	wire cla_c16   = !(cla_gn16 & (cla_pn16 | !sub2));

	wire [3:0] sum_s0l_4  = fadd_cla3(px2[2:0], py2[2:0], sub2);
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

	assign sum_c12 = !(cla_gn4[2] & (cla_pn4[2] | cla_c8_n));
	assign sum_c18 = cla_g2[8] | (cla_p2[8] & cla_c16);
	assign sum_c24 = cla_g8[2] | (cla_p8[2] & cla_c16);

	wire top_g2 = !(cla_gn[25] & (cla_pn[25] | cla_gn[24]));
	wire top_p2 = !(cla_pn[25] | cla_pn[24]);
	wire top_gn3 = cla_gn[26] & (cla_pn[26] | !top_g2);
	wire top_p3  = !cla_pn[26] & top_p2;
	wire far_c27_raw = !(top_gn3 & (!top_p3 | !sum_c24));
	wire far_c_w     = sub2 ^ far_c27_raw;
	wire cla_gn0_u = cla_gn[0] & cla_pn[0];

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

	wire [3:0] sum_t0_4 = fadd_cla3(px2[26:24], py2[26:24], 1'b0);
	wire [3:0] sum_t1_4 = fadd_cla3(px2[26:24], py2[26:24], 1'b1);
	assign sum_t0 = {sub2 ^ sum_t0_4[3], sum_t0_4[2:0]};
	assign sum_t1 = {sub2 ^ sum_t1_4[3], sum_t1_4[2:0]};
	assign sum_co_t0 = 1'b0;
	assign sum_co_t1 = 1'b0;

	assign sum = {sum_c24 ? sum_t1 : sum_t0, sum_c18 ? sum_18_1 : sum_18_0, sum_c12 ? sum_12_1 : sum_12_0, sum_c6 ? sum_6_1 : sum_6_0, sum_s0};
	assign sum_co_unused = (sum_c24 ? sum_co_t1 : sum_co_t0) | (&{1'b0, c9_u0, c9_u1, c15_u0, c15_u1, c21_u0, c21_u1, cla_gn0_u});
	assign sum_c6_0_un  = 1'b0;
	assign sum_c6_1_un  = 1'b0;
	assign sum_c12_0_un = 1'b0;
	assign sum_c12_1_un = 1'b0;
	assign sum_c18_0_un = 1'b0;
	assign sum_c18_1_un = 1'b0;
	assign sum_c12_d_un = 12'd0;
	assign sum_c18_d_un = 18'd0;
	assign sum_c24_d_un = 24'd0;
`endif

	wire inc_c = sum[3] && (sum[2] || sum[1] || sum[0] || sum[4]);
	wire inc_n = sum[2] && (sum[1] || sum[0] || sum[3]);
	wire inc_u = sum[1] && (sum[0] || sum[2]);

	wire [1:0] fmu_c_w;
	assign fmu_c_w[0] = sum[4] ^ inc_c;
	assign fmu_c_w[1] = sum[5] ^ (inc_c & sum[4]);
	wire cin6_c_w = inc_c & sum[4] & sum[5];

	wire [2:0] fmu_n_w;
	assign fmu_n_w[0] = sum[3] ^ inc_n;
	assign fmu_n_w[1] = sum[4] ^ (inc_n & sum[3]);
	assign fmu_n_w[2] = sum[5] ^ (inc_n & sum[3] & sum[4]);
	wire cin6_n_w = inc_n & (&sum[5:3]);

	wire [3:0] fmu_u_w;
	assign fmu_u_w[0] = sum[2] ^ inc_u;
	assign fmu_u_w[1] = sum[3] ^ (inc_u & sum[2]);
	assign fmu_u_w[2] = sum[4] ^ (inc_u & sum[2] & sum[3]);
	assign fmu_u_w[3] = sum[5] ^ (inc_u & (&sum[4:2]));
	wire cin6_u_w = inc_u & (&sum[5:2]);

	wire [11:6] p6_w;
	wire [17:12] p12_w;
	wire [23:18] p18_w;
	wire [26:24] p24_w;
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
	wire [11:6] p6_xor  = px2[11:6]  ^ py2[11:6];
	wire [17:12] p12_xor = px2[17:12] ^ py2[17:12];
	wire [23:18] p18_xor = px2[23:18] ^ py2[23:18];
	wire [26:24] p24_xor = px2[26:24] ^ py2[26:24];

	wire [2:0] p6_1_lo  = {&s6_lo1[2:0],  &s6_lo1[1:0],  s6_lo1[0]};
	wire [2:0] p6_1_h0  = {&s6_hi0[2:0],  &s6_hi0[1:0],  s6_hi0[0]};
	wire [2:0] p6_1_h1  = {&s6_hi1[2:0],  &s6_hi1[1:0],  s6_hi1[0]};
	wire [11:6] p6_0_w  = {&p6_xor[11:6],  &p6_xor[10:6],  &p6_xor[9:6],  &p6_xor[8:6],  &p6_xor[7:6],  p6_xor[6]};
	wire [11:6] p6_1_w  = {p6_1_lo[2] ? (c9_1  ? p6_1_h1  : p6_1_h0)  : 3'b000, p6_1_lo};
	assign p6_w = sum_c6 ? p6_1_w : p6_0_w;

	wire [2:0] p12_1_lo = {&s12_lo1[2:0], &s12_lo1[1:0], s12_lo1[0]};
	wire [2:0] p12_1_h0 = {&s12_hi0[2:0], &s12_hi0[1:0], s12_hi0[0]};
	wire [2:0] p12_1_h1 = {&s12_hi1[2:0], &s12_hi1[1:0], s12_hi1[0]};
	wire [17:12] p12_0_w = {&p12_xor[17:12], &p12_xor[16:12], &p12_xor[15:12], &p12_xor[14:12], &p12_xor[13:12], p12_xor[12]};
	wire [17:12] p12_1_w = {p12_1_lo[2] ? (c15_1 ? p12_1_h1 : p12_1_h0) : 3'b000, p12_1_lo};
	assign p12_w = sum_c12 ? p12_1_w : p12_0_w;

	wire [2:0] p18_1_lo = {&s18_lo1[2:0], &s18_lo1[1:0], s18_lo1[0]};
	wire [2:0] p18_1_h0 = {&s18_hi0[2:0], &s18_hi0[1:0], s18_hi0[0]};
	wire [2:0] p18_1_h1 = {&s18_hi1[2:0], &s18_hi1[1:0], s18_hi1[0]};
	wire [23:18] p18_0_w = {&p18_xor[23:18], &p18_xor[22:18], &p18_xor[21:18], &p18_xor[20:18], &p18_xor[19:18], p18_xor[18]};
	wire [23:18] p18_1_w = {p18_1_lo[2] ? (c21_1 ? p18_1_h1 : p18_1_h0) : 3'b000, p18_1_lo};
	assign p18_w = sum_c18 ? p18_1_w : p18_0_w;

	wire [26:24] p24_0_w = {&p24_xor[26:24], &p24_xor[25:24], p24_xor[24]};
	wire [26:24] p24_1_w = {&sum_t1[2:0],    &sum_t1[1:0],    sum_t1[0]};
	assign p24_w = sum_c24 ? p24_1_w : p24_0_w;
`endif

	/* 3-level nibble LZC & 1-stage mod-4 fine shift on registered close_diff2[24:0] (<= 8 cmos3) */
	genvar gnl;
	(* keep = "true", dont_touch = "true" *) wire [5:0] nib_nz;
	(* keep = "true" *) wire [5:0] n_oh0, n_oh1, n_oh2, n_oh3;
	generate
	for (gnl = 0; gnl < 6; gnl = gnl + 1) begin : g_lzc_nib
		wire [3:0] nb = close_diff2[4 * gnl +: 4];
		assign nib_nz[gnl] = |nb;
		assign n_oh1[gnl]  = nb[3];
		assign n_oh2[gnl]  = !nb[3] && nb[2];
		assign n_oh3[gnl]  = !nb[3] && !nb[2] && nb[1];
		assign n_oh0[gnl]  = !nb[3] && !nb[2] && !nb[1] && nb[0];
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
	wire ex2_le31_w = (ex2[7:5] == 3'd0);
	wire ex2_0_w = (ex2 == 8'd0);
	wire ex2_1_w = (ex2 == 8'd1);
	wire ex2_254_w = (ex2 == 8'd254);

	wire v3_w = v2;
	wire sp3_w = sp2;
	wire sp_nan3_w = sp_nan2;
	wire sp_inf3_w = sp_inf2;
	wire sx3_w = sx2;
	wire [7:0] ex2_3_w = ex2;
	wire [7:0] ex2_p1_3_w = ex2_p1;
	wire [7:0] ex2_sub1_3_w = ex2_sub1;
	wire [27:0] c_in_pad3  = {2'b00, close_diff2[22:0], 3'b000};
	wire [24:0] nl_s1_3_w;
	generate
	for (gnl = 0; gnl < 25; gnl = gnl + 1) begin : g_nls1_s3
		wire [3:0] oh_sel = (gnl < 9) ? lz10_oh_a : (gnl < 17) ? lz10_oh_b : lz10_oh_c;
		assign nl_s1_3_w[gnl] = (oh_sel[0] & c_in_pad3[gnl + 3]) |
		                        (oh_sel[1] & c_in_pad3[gnl + 2]) |
		                        (oh_sel[2] & c_in_pad3[gnl + 1]) |
		                        (oh_sel[3] & c_in_pad3[gnl]);
	end
	endgenerate
	wire [24:0] close_diff3_w = nl_s1_3_w;

	wire v3, sp3, sp_nan3, sp_inf3, sx3, sz3;
	wire is_close_a, is_close_b;
	wire far_c_a, far_c_b, far_c_c, far_c_d, far_c_e;
	wire far_norm_a, far_norm_b, far_norm_c, far_norm_d, far_norm_e;
	wire sel_fc_a, sel_fn_a, sel_fu_a;
	wire sel_fc_b, sel_fn_b, sel_fu_b;
	wire sel_fc_c, sel_fn_c, sel_fu_c;
	wire [26:6] sum3, sum3_b, sum3_c;
	wire [1:0] fmu_c3;
	wire [2:0] fmu_n3;
	wire [3:0] fmu_u3;
	wire cin6_c3, cin6_n3, cin6_u3;
	wire [11:6] p6_3;
	wire [17:12] p12_3;
	wire [23:18] p18_3;
	wire [26:24] p24_3;
	wire [24:0] close_diff3;
	wire [4:0] lz3_a, lz3_b;
	wire [7:0] ex2_3, ex2_p1_3, ex2_sub1_3;
	wire ex2_le31_3, ex2_0_3, ex2_1_3, ex2_254_3;

	/* Stage 4:
	 * - Close path: 1-stage 4-bit-granularity left shift and 8-bit exponent subtract.
	 * - Far path: 2-level factored-carry mantissa selection & 3-level output pack.
	 */
	wire [48:0] nl_s1_pad = {close_diff3, 24'd0};
	(* keep = "true", dont_touch = "true" *) wire [24:0] nl_hi0;
	(* keep = "true", dont_touch = "true" *) wire [24:13] nl_hi1;
	generate
	for (gnl = 0; gnl < 13; gnl = gnl + 1) begin : g_nl_lo
		assign nl_hi0[gnl] = (lz3_a[3:2] == 2'd0) ? nl_s1_pad[gnl + 24] :
		                    (lz3_a[3:2] == 2'd1) ? nl_s1_pad[gnl + 20] :
		                    (lz3_a[3:2] == 2'd2) ? nl_s1_pad[gnl + 16] : nl_s1_pad[gnl + 12];
	end
	for (gnl = 13; gnl < 25; gnl = gnl + 1) begin : g_nl_hi
		assign nl_hi0[gnl] = (lz3_b[3:2] == 2'd0) ? nl_s1_pad[gnl + 24] :
		                    (lz3_b[3:2] == 2'd1) ? nl_s1_pad[gnl + 20] :
		                    (lz3_b[3:2] == 2'd2) ? nl_s1_pad[gnl + 16] : nl_s1_pad[gnl + 12];
		assign nl_hi1[gnl] = (lz3_b[3:2] == 2'd0) ? nl_s1_pad[gnl + 8]  :
		                    (lz3_b[3:2] == 2'd1) ? nl_s1_pad[gnl + 4]  :
		                    (lz3_b[3:2] == 2'd2) ? nl_s1_pad[gnl]      : 1'b0;
	end
	endgenerate
	wire [22:0] close_m;
	generate
	for (gnl = 0; gnl < 12; gnl = gnl + 1) begin : g_cm_lo
		assign close_m[gnl] = !lz3_a[4] && nl_hi0[gnl + 1];
	end
	for (gnl = 12; gnl < 23; gnl = gnl + 1) begin : g_cm_hi
		assign close_m[gnl] = lz3_b[4] ? nl_hi1[gnl + 1] : nl_hi0[gnl + 1];
	end
	endgenerate
	wire [24:0] nl = {nl_hi0[24], close_m, nl_hi0[0]};
	wire [7:0] close_e;
	wire close_e_bo_unused;
	am_sub #(.W(8)) u_close_e (.a(ex2_3), .b({3'd0, lz3_b}), .bi(1'b0), .d(close_e), .bo(close_e_bo_unused));
	(* keep = "true" *) wire cudf_lo = (ex2_3[2:0] <= lz3_a[2:0]);
	(* keep = "true" *) wire cudf_eq = is_close_a && ex2_le31_3 && (ex2_3[4:3] == lz3_a[4:3]);
	(* keep = "true" *) wire cudf_lt = is_close_a && ex2_le31_3 && (ex2_3[4:3] < lz3_a[4:3]);

	/* Far path Stage 4 */
	wire cin_6  = far_c_a ? cin6_c3 : (far_norm_a ? cin6_n3 : cin6_u3);
	wire cin_12 = (far_c_c ? cin6_c3 : (far_norm_c ? cin6_n3 : cin6_u3)) & p6_3[11];
	wire p24_sel = far_c_c ? 1'b0 : (far_norm_c ? p24_3[26] : p24_3[25]);
	wire far_m0_cout = cin_12 & p12_3[17] & p18_3[23] & p24_sel;

	wire [22:4] mu_bit, mu_bit_inv;
	wire [22:4] p_mux;
	wire [10:5] w_p6  = {p6_3[10:6], 1'b1};
	wire [16:11] w_p12 = {p12_3[16:12], 1'b1};
	wire [22:17] w_p18 = {p18_3[22:18], 1'b1};

	genvar gi;
	generate
	for (gi = 4; gi < 11; gi = gi + 1) begin : g_mu_lo
		assign mu_bit[gi]     = (sel_fc_a &  sum3[gi + 4])   | (sel_fn_a &  sum3[gi + 3])   | (sel_fu_a &  sum3[gi + 2]);
		assign mu_bit_inv[gi] = (sel_fc_a & ~sum3[gi + 4])   | (sel_fn_a & ~sum3[gi + 3])   | (sel_fu_a & ~sum3[gi + 2]);
	end
	for (gi = 11; gi < 18; gi = gi + 1) begin : g_mu_mid
		assign mu_bit[gi]     = (sel_fc_b &  sum3_b[gi + 4]) | (sel_fn_b &  sum3_b[gi + 3]) | (sel_fu_b &  sum3_b[gi + 2]);
		assign mu_bit_inv[gi] = (sel_fc_b & ~sum3_b[gi + 4]) | (sel_fn_b & ~sum3_b[gi + 3]) | (sel_fu_b & ~sum3_b[gi + 2]);
	end
	for (gi = 18; gi < 23; gi = gi + 1) begin : g_mu_hi
		assign mu_bit[gi]     = (sel_fc_c &  sum3_c[gi + 4]) | (sel_fn_c &  sum3_c[gi + 3]) | (sel_fu_c &  sum3_c[gi + 2]);
		assign mu_bit_inv[gi] = (sel_fc_c & ~sum3_c[gi + 4]) | (sel_fn_c & ~sum3_c[gi + 3]) | (sel_fu_c & ~sum3_c[gi + 2]);
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

	wire [22:0] far_m;
	assign far_m[0] = far_c_a ? fmu_c3[0] : (far_norm_a ? fmu_n3[0] : fmu_u3[0]);
	assign far_m[1] = far_c_a ? fmu_c3[1] : (far_norm_a ? fmu_n3[1] : fmu_u3[1]);
	assign far_m[2] = far_c_a ? (sum3[6] ^ cin6_c3) : (far_norm_a ? fmu_n3[2] : fmu_u3[2]);
	wire mu3_c = sum3[7] ^ (cin6_c3 & p6_3[6]);
	assign far_m[3] = far_c_a ? mu3_c : (far_norm_a ? (sum3[6] ^ cin6_n3) : fmu_u3[3]);

	generate
	for (gi = 4; gi <= 9; gi = gi + 1) begin : g_fm_4_9
		wire fc_b = cin_6 & p_mux[gi];
		assign far_m[gi] = fc_b ? mu_bit_inv[gi] : mu_bit[gi];
	end
	for (gi = 10; gi <= 15; gi = gi + 1) begin : g_fm_10_15
		wire fc_b = cin_12 & p_mux[gi];
		assign far_m[gi] = fc_b ? mu_bit_inv[gi] : mu_bit[gi];
	end
	for (gi = 16; gi <= 21; gi = gi + 1) begin : g_fm_16_21
		wire fc_b = cin_12 & p12_3[17] & p_mux[gi];
		assign far_m[gi] = fc_b ? mu_bit_inv[gi] : mu_bit[gi];
	end
	endgenerate
	wire fc_22 = cin_12 & p12_3[17] & p18_3[23] & p_mux[22];
	assign far_m[22] = fc_22 ? mu_bit_inv[22] : mu_bit[22];

	/* Final packing (<= 3 LUT levels total, <= 1 Post-LUT after u_close_e) */
	wire far_mz = !is_close_a && (ex2_0_3 || (ex2_1_3 && !far_c_d && !far_norm_d) || (ex2_254_3 && far_c_d));
	wire force_mz_a = sp3 || sz3 || far_mz || cudf_lt || (cudf_eq && cudf_lo);
	wire force_mz_b = sp3 || sz3 || far_mz || cudf_lt || (cudf_eq && cudf_lo);
	wire close_en_a = !force_mz_a &&  is_close_a;
	wire far_en_a   = !force_mz_a && !is_close_a;
	wire close_en_b = !force_mz_b &&  is_close_b;
	wire far_en_b   = !force_mz_b && !is_close_b;

	wire [22:0] m_out;
	generate
	for (gi = 0; gi < 12; gi = gi + 1) begin : g_mout_lo
		assign m_out[gi] = (close_en_a & close_m[gi]) | (far_en_a & far_m[gi]);
	end
	for (gi = 12; gi < 22; gi = gi + 1) begin : g_mout_hi
		assign m_out[gi] = (close_en_b & close_m[gi]) | (far_en_b & far_m[gi]);
	end
	endgenerate
	wire sp_or_close22 = sp_nan3 || (close_en_b & close_m[22]);
	assign m_out[22] = sp_or_close22 | (far_en_b & far_m[22]);

	(* keep = "true" *) wire use_close_e = !sp3 && !sz3 && is_close_b && !cudf_lt && !(cudf_eq && cudf_lo);
	wire far_ovf_c0 = ex2_254_3 && far_c_e;
	wire far_udf_c0 = ex2_0_3 || (ex2_1_3 && !far_c_e && !far_norm_e);
	wire far_ovf_c1 = ex2_254_3 && (far_c_e || far_norm_e);
	wire far_udf_c1 = ex2_0_3;

	(* keep = "true" *) wire [7:0] far_e_c0, far_e_c1;
	wire [7:0] e_out;
	generate
	for (gi = 0; gi < 8; gi = gi + 1) begin : g_eout
		wire fc_e = (gi < 4) ? far_c_d : far_c_e;
		wire fn_e = (gi < 4) ? far_norm_d : far_norm_e;
		wire ebit_c0 = fc_e ? ex2_p1_3[gi] : (fn_e ? ex2_3[gi] : ex2_sub1_3[gi]);
		wire ebit_c1 = (fc_e || fn_e) ? ex2_p1_3[gi] : ex2_3[gi];
		assign far_e_c0[gi] = (sp_nan3 || sp_inf3 || (!sp3 && !sz3 && !is_close_b && far_ovf_c0)) ? 1'b1 :
		                      (sp3 || sz3 || is_close_b || far_udf_c0) ? 1'b0 : ebit_c0;
		assign far_e_c1[gi] = (sp_nan3 || sp_inf3 || (!sp3 && !sz3 && !is_close_b && far_ovf_c1)) ? 1'b1 :
		                      (sp3 || sz3 || is_close_b || far_udf_c1) ? 1'b0 : ebit_c1;
		assign e_out[gi] = use_close_e ? close_e[gi] : (far_m0_cout ? far_e_c1[gi] : far_e_c0[gi]);
	end
	endgenerate

	wire o_valid_comb = v3;
	wire [31:0] o_r_comb = {sx3 && !sz3, e_out, m_out};

	wire unused_bits = &{1'b0, shr_0[26:8], shr_0[0], shr_1[26:15], shr_1[7:0], shr_2[14:0],
		sh_s1_lo, sh_s1_mid, sh_s1_hi, d1_mid_oh_a, d1_mid_oh_b, d1_mid_oh_d,
		d1_a[4:2], d1_a_hi[4:2], d1_b[4:2], d1_d[4:2], y_clr_lo, y_clr_hi, sum[27],
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		sum_c12_d_un, sum_c18_d_un, sum_c24_d_un, sum_c6_0_un, sum_c6_1_un,
		sum_c12_0_un, sum_c12_1_un, sum_c18_0_un, sum_c18_1_un, eb_lt_ea_unused,
`endif
		nl[24], nl[0], close_e_bo_unused, dba_bo_unused,
		close_sub_bo_unused, sum_co_unused, ex2_p1_co_unused, ex2_sub1_bo_unused,
		cmp_diff_unused};

	generate
		if (LAT == 0) begin : g_lat0
			assign v1 = i_valid;
			assign sp1 = sp1_w;
			assign sp_nan1 = sp_nan1_w;
			assign sp_inf1 = sp_inf1_w;
			assign sx1 = sx1_w;
			assign sub1_a = sub1_w;
			assign sub1_b = sub1_w;
			assign y_zero1_a = y_zero1_w;
			assign y_zero1_b = y_zero1_w;
			assign ealtb1 = ealtb1_w;
			assign dab_hi1 = dab_hi1_w;
			assign dba_hi1 = dba_hi1_w;
			assign close_cand1 = cc_en1_w && (ealtb1_w ? (!dba_hi1_w && dba_cc1_w) : (!dab_hi1_w && dab_cc1_w));
			assign ex1 = ex1_w;
			assign mx1 = mx1_w;
			assign my1 = my1_w;
			assign my1_b = my1_w;
			assign my1_c = my1_w;
			assign my1_x = my1_x_w;
			assign d1_a = d1_w;
			assign d1_a_hi = d1_w;
			assign d1_b = d1_w;
			assign d1_c = d1_w;
			assign d1_d = d1_w;
			assign d1_e0 = d1_w[0];
			assign ex_xor_eb_0_a = ex_xor_eb_0_w;
			assign ex_xor_eb_0_b = ex_xor_eb_0_w;
			assign ex_xor_eb_0_c = ex_xor_eb_0_w;
			assign ex_xor_eb_0_d = ex_xor_eb_0_w;
			assign d1_f0 = d1_w[0];
			assign d1_lo_oh_a = d1_lo_oh_w;
			assign d1_lo_oh_b = d1_lo_oh_w;
			assign d1_lo_oh_d = d1_lo_oh_w;
			assign d1_mid_oh_a = d1_mid_oh_w;
			assign d1_mid_oh_b = d1_mid_oh_w;
			assign d1_mid_oh_d = d1_mid_oh_w;
			assign d1_coarse_oh_a = d1_coarse_oh_w;
			assign d1_coarse_oh_b = d1_coarse_oh_w;
			assign d1_coarse_oh_d = d1_coarse_oh_w;
			assign d1_stk_ge = d1_stk_ge_w;
			assign y_clr_lo  = y_clr_w;
			assign y_clr_hi  = y_clr_w;
			assign py2_hi1 = py2_hi1_w;

			assign v2 = v1;
			assign sp2 = sp2_w;
			assign sp_nan2 = sp_nan2_w;
			assign sp_inf2 = sp_inf2_w;
			assign sx2 = sx2_w;
			assign sub2 = sub2_w;
			assign close_cand2 = close_cand2_w;
			assign ex2 = ex2_w;
			assign ex2_p1 = ex2_p1_w;
			assign ex2_sub1 = ex2_sub1_w;
			assign px2 = px2_w;
			assign py2 = py2_w;
			assign close_diff2 = close_diff2_w;

			assign v3 = v2;
			assign sp3 = sp3_w;
			assign sp_nan3 = sp_nan3_w;
			assign sp_inf3 = sp_inf3_w;
			assign sx3 = sx3_w;
			assign sz3 = sz3_w;
			assign is_close_a = is_close3_w;
			assign is_close_b = is_close3_w;
			assign far_c_a = far_c_w;
			assign far_c_b = far_c_w;
			assign far_c_c = far_c_w;
			assign far_c_d = far_c_w;
			assign far_c_e = far_c_w;
			assign far_norm_a = sum[26];
			assign far_norm_b = sum[26];
			assign far_norm_c = sum[26];
			assign far_norm_d = sum[26];
			assign far_norm_e = sum[26];
			assign sel_fc_a = far_c_w;
			assign sel_fc_b = far_c_w;
			assign sel_fc_c = far_c_w;
			assign sel_fn_a = !far_c_w && sum[26];
			assign sel_fn_b = !far_c_w && sum[26];
			assign sel_fn_c = !far_c_w && sum[26];
			assign sel_fu_a = !far_c_w && !sum[26];
			assign sel_fu_b = !far_c_w && !sum[26];
			assign sel_fu_c = !far_c_w && !sum[26];
			assign sum3 = sum[26:6];
			assign sum3_b = sum[26:6];
			assign sum3_c = sum[26:6];
			assign fmu_c3 = fmu_c_w;
			assign fmu_n3 = fmu_n_w;
			assign fmu_u3 = fmu_u_w;
			assign cin6_c3 = cin6_c_w;
			assign cin6_n3 = cin6_n_w;
			assign cin6_u3 = cin6_u_w;
			assign p6_3 = p6_w;
			assign p12_3 = p12_w;
			assign p18_3 = p18_w;
			assign p24_3 = p24_w;
			assign close_diff3 = close_diff3_w;
			assign lz3_a = lz;
			assign lz3_b = lz;
			assign ex2_3 = ex2_3_w;
			assign ex2_p1_3 = ex2_p1_3_w;
			assign ex2_sub1_3 = ex2_sub1_3_w;
			assign ex2_le31_3 = ex2_le31_w;
			assign ex2_0_3 = ex2_0_w;
			assign ex2_1_3 = ex2_1_w;
			assign ex2_254_3 = ex2_254_w;

			assign o_valid = o_valid_comb;
			assign o_r = o_r_comb;

			wire unused_clk = &{1'b0, clk, v1_w, v2_w, v3_w, unused_bits};
		end else begin : g_lat4
			(* keep = "true" *) reg r_v1, r_sp1, r_sp_nan1, r_sp_inf1, r_sx_norm1, r_sp_sx1;
			(* keep = "true" *) reg r_ealtb1, r_dab_hi1, r_dba_hi1, r_dab_cc1, r_dba_cc1, r_cc_en1;
			(* keep = "true", dont_touch = "true" *) reg r_y_zero1_a, r_y_zero1_b;
			(* keep = "true", dont_touch = "true" *) reg r_sub1_a, r_sub1_b;
			(* extract_enable = "no", extract_reset = "no" *) reg [7:0] r_ex1;
			(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [4:0] r_d1_a, r_d1_a_hi, r_d1_b, r_d1_c, r_d1_d;
			(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [3:0] r_d1_lo_oh_a, r_d1_lo_oh_b, r_d1_lo_oh_d;
			(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [3:0] r_d1_mid_oh_a, r_d1_mid_oh_b, r_d1_mid_oh_d;
			(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [6:0] r_d1_coarse_oh_a, r_d1_coarse_oh_b, r_d1_coarse_oh_d;
			(* keep = "true", dont_touch = "true", extract_reset = "no" *) reg [6:0] r_d1_stk_ge;
			(* keep = "true", dont_touch = "true" *) reg r_y_clr_lo, r_y_clr_hi;
			(* keep = "true", dont_touch = "true" *) reg r_py2_hi1;
			(* keep = "true" *) reg r_d1_e0, r_d1_f0;
			(* keep = "true", dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg r_ex_xor_eb_0_a, r_ex_xor_eb_0_b, r_ex_xor_eb_0_c, r_ex_xor_eb_0_d;
			(* keep = "true", dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg [23:0] r_mx1, r_my1, r_my1_b, r_my1_c, r_my1_x;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v1 <= 1'b0;
				else
					r_v1 <= i_valid;
			end

			always @(posedge clk) begin
				r_sp1 <= sp1_w;
				r_sp_nan1 <= sp_nan1_w;
				r_sp_inf1 <= sp_inf1_w;
				r_sx_norm1 <= sx_norm1_w;
				r_sp_sx1 <= sp_sx1_w;
				r_sub1_a <= sub1_w;
				r_sub1_b <= sub1_w;
				r_y_zero1_a <= y_zero1_w;
				r_y_zero1_b <= y_zero1_w;
				r_ealtb1 <= ealtb1_w;
				r_dab_hi1 <= dab_hi1_w;
				r_dba_hi1 <= dba_hi1_w;
				r_dab_cc1 <= dab_cc1_w;
				r_dba_cc1 <= dba_cc1_w;
				r_cc_en1 <= cc_en1_w;
				r_ex1 <= ex1_w;
				r_mx1 <= mx1_w;
				r_my1 <= my1_w;
				r_my1_b <= my1_w;
				r_my1_c <= my1_w;
				r_my1_x <= my1_x_w;
				r_d1_a <= d1_w;
				r_d1_a_hi <= d1_w;
				r_d1_b <= d1_w;
				r_d1_c <= d1_w;
				r_d1_d <= d1_w;
				r_d1_lo_oh_a <= d1_lo_oh_w;
				r_d1_lo_oh_b <= d1_lo_oh_w;
				r_d1_lo_oh_d <= d1_lo_oh_w;
				r_d1_mid_oh_a <= d1_mid_oh_w;
				r_d1_mid_oh_b <= d1_mid_oh_w;
				r_d1_mid_oh_d <= d1_mid_oh_w;
				r_d1_coarse_oh_a <= d1_coarse_oh_w;
				r_d1_coarse_oh_b <= d1_coarse_oh_w;
				r_d1_coarse_oh_d <= d1_coarse_oh_w;
				r_d1_stk_ge      <= d1_stk_ge_w;
				r_y_clr_lo       <= y_clr_w;
				r_y_clr_hi       <= y_clr_w;
				r_py2_hi1        <= py2_hi1_w;
				r_d1_e0 <= d1_w[0];
				r_ex_xor_eb_0_a <= ex_xor_eb_0_w;
				r_ex_xor_eb_0_b <= ex_xor_eb_0_w;
				r_ex_xor_eb_0_c <= ex_xor_eb_0_w;
				r_ex_xor_eb_0_d <= ex_xor_eb_0_w;
				r_d1_f0 <= d1_w[0];
			end

			assign v1 = r_v1;
			assign sp1 = r_sp1;
			assign sp_nan1 = r_sp_nan1;
			assign sp_inf1 = r_sp_inf1;
			assign sx1 = r_sp1 ? r_sp_sx1 : r_sx_norm1;
			assign sub1_a = r_sub1_a;
			assign sub1_b = r_sub1_b;
			assign y_zero1_a = r_y_zero1_a;
			assign y_zero1_b = r_y_zero1_b;
			assign ealtb1 = r_ealtb1;
			assign dab_hi1 = r_dab_hi1;
			assign dba_hi1 = r_dba_hi1;
			assign close_cand1 = r_cc_en1 && (r_ealtb1 ? (!r_dba_hi1 && r_dba_cc1) : (!r_dab_hi1 && r_dab_cc1));
			assign ex1 = r_ex1;
			assign mx1 = r_mx1;
			assign my1 = r_my1;
			assign my1_b = r_my1_b;
			assign my1_c = r_my1_c;
			assign my1_x = r_my1_x;
			assign d1_a = r_d1_a;
			assign d1_a_hi = r_d1_a_hi;
			assign d1_b = r_d1_b;
			assign d1_c = r_d1_c;
			assign d1_d = r_d1_d;
			assign d1_e0 = r_d1_e0;
			assign ex_xor_eb_0_a = r_ex_xor_eb_0_a;
			assign ex_xor_eb_0_b = r_ex_xor_eb_0_b;
			assign ex_xor_eb_0_c = r_ex_xor_eb_0_c;
			assign ex_xor_eb_0_d = r_ex_xor_eb_0_d;
			assign d1_f0 = r_d1_f0;
			assign d1_lo_oh_a = r_d1_lo_oh_a;
			assign d1_lo_oh_b = r_d1_lo_oh_b;
			assign d1_lo_oh_d = r_d1_lo_oh_d;
			assign d1_mid_oh_a = r_d1_mid_oh_a;
			assign d1_mid_oh_b = r_d1_mid_oh_b;
			assign d1_mid_oh_d = r_d1_mid_oh_d;
			assign d1_coarse_oh_a = r_d1_coarse_oh_a;
			assign d1_coarse_oh_b = r_d1_coarse_oh_b;
			assign d1_coarse_oh_d = r_d1_coarse_oh_d;
			assign d1_stk_ge = r_d1_stk_ge;
			assign y_clr_lo  = r_y_clr_lo;
			assign y_clr_hi  = r_y_clr_hi;
			assign py2_hi1 = r_py2_hi1;

			reg r_v2, r_sp2, r_sp_nan2, r_sp_inf2, r_sx2, r_sub2, r_close_cand2;
			reg [7:0] r_ex2, r_ex2_p1, r_ex2_sub1;
			reg [26:0] r_px2, r_py2;
			reg [24:0] r_close_diff2;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v2 <= 1'b0;
				else
					r_v2 <= v1;
			end

			always @(posedge clk) begin
				r_sp2 <= sp2_w;
				r_sp_nan2 <= sp_nan2_w;
				r_sp_inf2 <= sp_inf2_w;
				r_sx2 <= sx2_w;
				r_sub2 <= sub2_w;
				r_close_cand2 <= close_cand2_w;
				r_ex2 <= ex2_w;
				r_ex2_p1 <= ex2_p1_w;
				r_ex2_sub1 <= ex2_sub1_w;
				r_px2 <= px2_w;
				r_py2 <= py2_w;
				r_close_diff2 <= close_diff2_w;
			end

			assign v2 = r_v2;
			assign sp2 = r_sp2;
			assign sp_nan2 = r_sp_nan2;
			assign sp_inf2 = r_sp_inf2;
			assign sx2 = r_sx2;
			assign sub2 = r_sub2;
			assign close_cand2 = r_close_cand2;
			assign ex2 = r_ex2;
			assign ex2_p1 = r_ex2_p1;
			assign ex2_sub1 = r_ex2_sub1;
			assign px2 = r_px2;
			assign py2 = r_py2;
			assign close_diff2 = r_close_diff2;

			reg r_v3, r_sp3, r_sp_nan3, r_sp_inf3, r_sx3, r_sz3;
			(* keep = "true", dont_touch = "true" *) reg r_is_close_a, r_is_close_b;
			(* keep = "true", dont_touch = "true" *) reg r_far_c_a, r_far_c_b, r_far_c_c, r_far_c_d, r_far_c_e;
			(* keep = "true", dont_touch = "true" *) reg r_far_norm_a, r_far_norm_b, r_far_norm_c, r_far_norm_d, r_far_norm_e;
			(* keep = "true", dont_touch = "true" *) reg r_sel_fc_a, r_sel_fc_b, r_sel_fc_c;
			(* keep = "true", dont_touch = "true" *) reg r_sel_fn_a, r_sel_fn_b, r_sel_fn_c;
			(* keep = "true", dont_touch = "true" *) reg r_sel_fu_a, r_sel_fu_b, r_sel_fu_c;
			(* keep = "true", dont_touch = "true" *) reg [26:6] r_sum3, r_sum3_b, r_sum3_c;
			reg [1:0] r_fmu_c3;
			reg [2:0] r_fmu_n3;
			reg [3:0] r_fmu_u3;
			reg r_cin6_c3, r_cin6_n3, r_cin6_u3;
			reg [11:6] r_p6_3;
			reg [17:12] r_p12_3;
			reg [23:18] r_p18_3;
			reg [26:24] r_p24_3;
			reg [24:0] r_close_diff3;
			(* keep = "true" *) reg [4:0] r_lz3_a, r_lz3_b;
			reg [7:0] r_ex2_3, r_ex2_p1_3, r_ex2_sub1_3;
			reg r_ex2_le31_3, r_ex2_0_3, r_ex2_1_3, r_ex2_254_3;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_v3 <= 1'b0;
				else
					r_v3 <= v2;
			end

			always @(posedge clk) begin
				r_sp3         <= sp3_w;
				r_sp_nan3     <= sp_nan3_w;
				r_sp_inf3     <= sp_inf3_w;
				r_sx3         <= sx3_w;
				r_sz3         <= sz3_w;
				r_is_close_a  <= is_close3_w;
				r_is_close_b  <= is_close3_w;
				r_far_c_a     <= far_c_w;
				r_far_c_b     <= far_c_w;
				r_far_c_c     <= far_c_w;
				r_far_c_d     <= far_c_w;
				r_far_c_e     <= far_c_w;
				r_far_norm_a  <= sum[26];
				r_far_norm_b  <= sum[26];
				r_far_norm_c  <= sum[26];
				r_far_norm_d  <= sum[26];
				r_far_norm_e  <= sum[26];
				r_sel_fc_a    <= far_c_w;
				r_sel_fc_b    <= far_c_w;
				r_sel_fc_c    <= far_c_w;
				r_sel_fn_a    <= !far_c_w && sum[26];
				r_sel_fn_b    <= !far_c_w && sum[26];
				r_sel_fn_c    <= !far_c_w && sum[26];
				r_sel_fu_a    <= !far_c_w && !sum[26];
				r_sel_fu_b    <= !far_c_w && !sum[26];
				r_sel_fu_c    <= !far_c_w && !sum[26];
				r_sum3        <= sum[26:6];
				r_sum3_b      <= sum[26:6];
				r_sum3_c      <= sum[26:6];
				r_fmu_c3      <= fmu_c_w;
				r_fmu_n3      <= fmu_n_w;
				r_fmu_u3      <= fmu_u_w;
				r_cin6_c3     <= cin6_c_w;
				r_cin6_n3     <= cin6_n_w;
				r_cin6_u3     <= cin6_u_w;
				r_p6_3        <= p6_w;
				r_p12_3       <= p12_w;
				r_p18_3       <= p18_w;
				r_p24_3       <= p24_w;
				r_close_diff3 <= close_diff3_w;
				r_lz3_a       <= lz;
				r_lz3_b       <= lz;
				r_ex2_3       <= ex2_3_w;
				r_ex2_p1_3    <= ex2_p1_3_w;
				r_ex2_sub1_3  <= ex2_sub1_3_w;
				r_ex2_le31_3  <= ex2_le31_w;
				r_ex2_0_3     <= ex2_0_w;
				r_ex2_1_3     <= ex2_1_w;
				r_ex2_254_3   <= ex2_254_w;
			end

			assign v3          = r_v3;
			assign sp3         = r_sp3;
			assign sp_nan3     = r_sp_nan3;
			assign sp_inf3     = r_sp_inf3;
			assign sx3         = r_sx3;
			assign sz3         = r_sz3;
			assign is_close_a  = r_is_close_a;
			assign is_close_b  = r_is_close_b;
			assign far_c_a     = r_far_c_a;
			assign far_c_b     = r_far_c_b;
			assign far_c_c     = r_far_c_c;
			assign far_c_d     = r_far_c_d;
			assign far_c_e     = r_far_c_e;
			assign far_norm_a  = r_far_norm_a;
			assign far_norm_b  = r_far_norm_b;
			assign far_norm_c  = r_far_norm_c;
			assign far_norm_d  = r_far_norm_d;
			assign far_norm_e  = r_far_norm_e;
			assign sel_fc_a    = r_sel_fc_a;
			assign sel_fc_b    = r_sel_fc_b;
			assign sel_fc_c    = r_sel_fc_c;
			assign sel_fn_a    = r_sel_fn_a;
			assign sel_fn_b    = r_sel_fn_b;
			assign sel_fn_c    = r_sel_fn_c;
			assign sel_fu_a    = r_sel_fu_a;
			assign sel_fu_b    = r_sel_fu_b;
			assign sel_fu_c    = r_sel_fu_c;
			assign sum3        = r_sum3;
			assign sum3_b      = r_sum3_b;
			assign sum3_c      = r_sum3_c;
			assign fmu_c3      = r_fmu_c3;
			assign fmu_n3      = r_fmu_n3;
			assign fmu_u3      = r_fmu_u3;
			assign cin6_c3     = r_cin6_c3;
			assign cin6_n3     = r_cin6_n3;
			assign cin6_u3     = r_cin6_u3;
			assign p6_3        = r_p6_3;
			assign p12_3       = r_p12_3;
			assign p18_3       = r_p18_3;
			assign p24_3       = r_p24_3;
			assign close_diff3 = r_close_diff3;
			assign lz3_a       = r_lz3_a;
			assign lz3_b       = r_lz3_b;
			assign ex2_3       = r_ex2_3;
			assign ex2_p1_3    = r_ex2_p1_3;
			assign ex2_sub1_3  = r_ex2_sub1_3;
			assign ex2_le31_3  = r_ex2_le31_3;
			assign ex2_0_3     = r_ex2_0_3;
			assign ex2_1_3     = r_ex2_1_3;
			assign ex2_254_3   = r_ex2_254_3;

			reg r_ovalid;
			(* extract_reset = "no" *) reg [31:0] r_or;

			always @(posedge clk or negedge rst_n) begin
				if (!rst_n)
					r_ovalid <= 1'b0;
				else
					r_ovalid <= o_valid_comb;
			end

			always @(posedge clk) begin
				r_or     <= o_r_comb;
			end

			assign o_valid = r_ovalid;
			assign o_r     = r_or;

			wire unused_lat4 = &{1'b0, unused_bits};
		end
	endgenerate
endmodule

`default_nettype wire
