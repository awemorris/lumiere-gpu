/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_edge: the integer part of the RX setup: snaps the window
 * coordinates to 15.8 fixed point, builds the triangle that the scan
 * converter walks (a triangle, or one of the two sub-triangles of a line's
 * parallelogram or of a point's square), its edges with the top-left rule,
 * its bounding box (the pixels of the whole primitive) ∩ scissor ∩ render
 * target (∩ the viewport's rectangle), its facing and culling, and writes
 * the integer deltas of the triangle that defines the planes back to
 * atg_rx_setup's fp registers as fp32.
 *
 * start runs it; it reads fp registers 31, 32, 34, 35, 37, 38 (the window
 * x, y of vertices 0-2) and 76-79 (the viewport's rectangle x0, x1, y0,
 * y1) through r_ra / r_rd (data the next cycle), and writes DX1, DY1, DX2,
 * DY2, DF, OX, OY to registers 40-46 through r_we / r_wa / r_wd, then
 * pulses done; edges, bbox, front and empty hold until the next start.
 * Line (vertices 0 = a, 1 = b): x-major when |dx| >= |dy|; corners a and b
 * -+ (0, 128) (x-major) or -+ (128, 0) subpixels; sub-triangle 0 (a-, b-,
 * b+), 1 (a-, b+, a+); planes of (a, b, a + one pixel along the minor
 * axis).  Point (vertex 0 = p): corners p -+ (128, 128); sub-triangle 0
 * (p--, p+-, p++), 1 (p--, p++, p-+); planes of (p, p + (1, 0), p + (0,
 * 1)).  Lines and points are front and never culled.  An empty result
 * (zero area, culled, empty bbox) zeroes the edges and the bbox and writes
 * no register.  See spec/gpu/rx/atg_rx_edge.md.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rx_edge (
	input wire clk,
	input wire rst_n,
	input wire start,
	input wire [3:0] mode,	/* [1:0] kind, [2] sub-triangle, [3] raw */
	input wire [31:0] smin,
	input wire [31:0] smax,
	input wire [31:0] rts,
	input wire [2:0] cull,
	input wire vprect,
	input wire [15:0] line_width,
	input wire [15:0] point_size,
	output reg [6:0] r_ra,
	input wire [31:0] r_rd,
	output reg r_we,
	output reg [6:0] r_wa,
	output reg [31:0] r_wd,
	output reg done,
	output wire [287:0] edges,
	output wire [63:0] bbox,
	output reg front,
	output reg empty
);
	wire rst_sync = !rst_n;
	(* max_fanout = 16 *) reg run;
	(* extract_reset = "no", extract_enable = "no", max_fanout = 16 *) reg [36:0] phase_oh;
	(* keep = "true", dont_touch = "true" *) reg [36:0] phase_oh_b, phase_oh_c;
	(* extract_enable = "no", max_fanout = 16 *) reg [3:0] md;
	(* keep = "true", dont_touch = "true", extract_enable = "no" *) reg [63:0] r_is_tri_c;
	(* keep = "true", dont_touch = "true", extract_enable = "no" *) reg [47:0] r_is_line_c;
	(* equivalent_register_removal = "no", extract_enable = "no", max_fanout = 16 *) reg [3:0] r_is_pt_c;
	wire [1:0] kind = md[1:0];
	wire is_tri = r_is_tri_c[0];
	integer k;
	genvar gi, gs;

	wire [55:0] carry_unused;	/* the adders' carries */
	wire [251:0] cmp_d_unused;

	/* Pipelined f2fix stage 1: registered r_rd -> coarse+fine shift -> round prep -> add */
	localparam [23:0] SAT_POS = 24'h3FFFFF;
	localparam [23:0] SAT_NEG = 24'hC00001;

	(* dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg [31:0] r_rd_snap;
	wire [7:0] snap_exp = r_rd_snap[30:23];
	wire [22:0] snap_mant = r_rd_snap[22:0];
`ifdef SYNTH_KINTEX7
	wire [7:0] snap_exp_ovf;
	genvar gsx;
	generate
	for (gsx = 0; gsx < 8; gsx = gsx + 1) begin : g_sn_exp_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sxo (.I0(r_rd_snap[23 + gsx]), .O(snap_exp_ovf[gsx]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] snap_exp_ovf;
	genvar gsx;
	generate
	for (gsx = 0; gsx < 8; gsx = gsx + 1) begin : g_sn_exp_b
		assign snap_exp_ovf[gsx] = r_rd_snap[23 + gsx];
	end
	endgenerate
`else
	wire [7:0] snap_exp_ovf = snap_exp;
`endif
	assign carry_unused[49] = 1'b0;

	/* Exponent subtraction (3 cmos3 for sub14_lo, 4 cmos3 for one-hot sh_hi_oh[7:0]) */
	wire [1:0] sub14_lo = {!(snap_exp[1] ^ snap_exp[0]), snap_exp[0]};
	wire [3:0] sub14_lo_oh = {
		sub14_lo[1]  &  sub14_lo[0],
		sub14_lo[1]  & ~sub14_lo[0],
		~sub14_lo[1] &  sub14_lo[0],
		~sub14_lo[1] & ~sub14_lo[0]
	};
	wire snap_amt5_hi = !snap_exp[7] && !(&snap_exp[3:0]);
	wire sn_g3 = (&snap_exp[3:0]) | (!snap_exp[3] & !snap_exp[2] & !(snap_exp[1] & snap_exp[0]));
	wire sn_g2 = (!snap_exp[3] & !snap_exp[2] & snap_exp[1] & snap_exp[0]) | (!snap_exp[3] & snap_exp[2] & !(snap_exp[1] & snap_exp[0]));
	wire sn_g1 = (!snap_exp[3] & snap_exp[2] & snap_exp[1] & snap_exp[0]) | (snap_exp[3] & !snap_exp[2] & !(snap_exp[1] & snap_exp[0]));
	wire sn_g0 = (snap_exp[3] & !snap_exp[2] & snap_exp[1] & snap_exp[0]) | (snap_exp[3] & snap_exp[2] & !(snap_exp[1] & snap_exp[0]));
	(* keep = "true" *) wire [7:0] sh_hi_oh = {
		 snap_amt5_hi && sn_g3,
		 snap_amt5_hi && sn_g2,
		 snap_amt5_hi && sn_g1,
		 snap_amt5_hi && sn_g0,
		!snap_amt5_hi && sn_g3,
		!snap_amt5_hi && sn_g2,
		!snap_amt5_hi && sn_g1,
		!snap_amt5_hi && sn_g0
	};
	wire [47:0] snap_in = {1'b1, snap_mant, 24'd0};
`ifdef SYNTH_KINTEX7
	wire [2:0] sub14_hi = {snap_amt5_hi, (sn_g3 || sn_g2), (sn_g3 || sn_g1)};
	wire [4:0] snap_amt5 = {sub14_hi, sub14_lo};
	wire snap_c16_s2_sn2, snap_c16_s1_sn1, snap_c16_s0_sn0;
	wire [3:0] snap_c16_s2_sn1, snap_c16_s1_sn0;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc22 (.I0(snap_amt5[4]), .O(snap_c16_s2_sn2));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc11 (.I0(snap_amt5[4]), .O(snap_c16_s1_sn1));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc00 (.I0(snap_amt5[4]), .O(snap_c16_s0_sn0));
	genvar gsc;
	generate
	for (gsc = 0; gsc < 4; gsc = gsc + 1) begin : g_sc16_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc21 (.I0(snap_amt5[4]), .O(snap_c16_s2_sn1[gsc]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc10 (.I0(snap_amt5[4]), .O(snap_c16_s1_sn0[gsc]));
	end
	endgenerate

	wire [15:0] snap_s1_sn2 = snap_c16_s2_sn2 ? 16'd0 : snap_in[47:32];
	wire [31:0] snap_s1_sn1 = {
		snap_c16_s2_sn1[3] ? 4'd0 : snap_in[47:44],
		snap_c16_s2_sn1[2] ? 4'd0 : snap_in[43:40],
		snap_c16_s2_sn1[1] ? 4'd0 : snap_in[39:36],
		snap_c16_s2_sn1[0] ? 4'd0 : snap_in[35:32],
		snap_c16_s1_sn1 ? snap_in[47:32] : snap_in[31:16]
	};
	wire [31:0] snap_s1_sn0 = {
		snap_c16_s1_sn0[3] ? snap_in[47:44] : snap_in[31:28],
		snap_c16_s1_sn0[2] ? snap_in[43:40] : snap_in[27:24],
		snap_c16_s1_sn0[1] ? snap_in[39:36] : snap_in[23:20],
		snap_c16_s1_sn0[0] ? snap_in[35:32] : snap_in[19:16],
		snap_c16_s0_sn0 ? snap_in[31:16] : snap_in[15:0]
	};

	wire [3:0] snap_amt_s2, snap_amt_s1_lo, snap_amt_s1_hi, snap_amt_s0_lo, snap_amt_s0_hi;
	genvar gsa;
	generate
	for (gsa = 0; gsa < 4; gsa = gsa + 1) begin : g_sn_amt_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sa2  (.I0(snap_amt5[gsa]), .O(snap_amt_s2[gsa]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sa1l (.I0(snap_amt5[gsa]), .O(snap_amt_s1_lo[gsa]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sa1h (.I0(snap_amt5[gsa]), .O(snap_amt_s1_hi[gsa]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sa0l (.I0(snap_amt5[gsa]), .O(snap_amt_s0_lo[gsa]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sa0h (.I0(snap_amt5[gsa]), .O(snap_amt_s0_hi[gsa]));
	end
	endgenerate
	wire [23:0] snap_sh1_lo_w, snap_sh1_hi_w;
	am_shift #(.W(24), .SW(4)) u_sn1_lo (.a(snap_s1_sn1[23:0]), .amt(snap_amt_s1_lo), .left(1'b0), .arith(1'b0), .y(snap_sh1_lo_w));
	am_shift #(.W(24), .SW(4)) u_sn1_hi (.a(snap_s1_sn1[31:8]), .amt(snap_amt_s1_hi), .left(1'b0), .arith(1'b0), .y(snap_sh1_hi_w));
	wire [15:0] snap_sh1 = {snap_sh1_hi_w[7:0], snap_sh1_lo_w[7:0]};
	wire [31:0] snap_sh1_full = {16'd0, snap_sh1_hi_w[23:16], snap_sh1_lo_w[23:16]};

	wire [15:0] snap_sh2;
	wire [23:0] snap_sh0_lo_w, snap_sh0_hi_w;
	am_shift #(.W(16), .SW(4)) u_sn2    (.a(snap_s1_sn2),        .amt(snap_amt_s2),    .left(1'b0), .arith(1'b0), .y(snap_sh2));
	am_shift #(.W(24), .SW(4)) u_sn0_lo (.a(snap_s1_sn0[23:0]),  .amt(snap_amt_s0_lo), .left(1'b0), .arith(1'b0), .y(snap_sh0_lo_w));
	am_shift #(.W(24), .SW(4)) u_sn0_hi (.a(snap_s1_sn0[31:8]),  .amt(snap_amt_s0_hi), .left(1'b0), .arith(1'b0), .y(snap_sh0_hi_w));
	wire [31:0] snap_sh0_full = {snap_sh0_hi_w[23:8], snap_sh0_hi_w[7:0], snap_sh0_lo_w[7:0]};

	wire [47:0] snap_shifted = {snap_sh2, snap_sh1, snap_sh0_full[15:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] sub14_hi = {snap_amt5_hi, (sn_g3 || sn_g2), (sn_g3 || sn_g1)};
	wire [4:0] snap_amt5 = {sub14_hi, sub14_lo};
	wire snap_c16_s2_sn2, snap_c16_s1_sn1, snap_c16_s0_sn0;
	wire [3:0] snap_c16_s2_sn1, snap_c16_s1_sn0;
	assign snap_c16_s2_sn2 = snap_amt5[4];
	assign snap_c16_s1_sn1 = snap_amt5[4];
	assign snap_c16_s0_sn0 = snap_amt5[4];
	genvar gsc;
	generate
	for (gsc = 0; gsc < 4; gsc = gsc + 1) begin : g_sc16_b
		assign snap_c16_s2_sn1[gsc] = snap_amt5[4];
		assign snap_c16_s1_sn0[gsc] = snap_amt5[4];
	end
	endgenerate

	wire [15:0] snap_s1_sn2 = snap_c16_s2_sn2 ? 16'd0 : snap_in[47:32];
	wire [31:0] snap_s1_sn1 = {
		snap_c16_s2_sn1[3] ? 4'd0 : snap_in[47:44],
		snap_c16_s2_sn1[2] ? 4'd0 : snap_in[43:40],
		snap_c16_s2_sn1[1] ? 4'd0 : snap_in[39:36],
		snap_c16_s2_sn1[0] ? 4'd0 : snap_in[35:32],
		snap_c16_s1_sn1 ? snap_in[47:32] : snap_in[31:16]
	};
	wire [31:0] snap_s1_sn0 = {
		snap_c16_s1_sn0[3] ? snap_in[47:44] : snap_in[31:28],
		snap_c16_s1_sn0[2] ? snap_in[43:40] : snap_in[27:24],
		snap_c16_s1_sn0[1] ? snap_in[39:36] : snap_in[23:20],
		snap_c16_s1_sn0[0] ? snap_in[35:32] : snap_in[19:16],
		snap_c16_s0_sn0 ? snap_in[31:16] : snap_in[15:0]
	};

	wire [3:0] snap_amt_s2, snap_amt_s1_lo, snap_amt_s1_hi, snap_amt_s0_lo, snap_amt_s0_hi;
	genvar gsa;
	generate
	for (gsa = 0; gsa < 4; gsa = gsa + 1) begin : g_sn_amt_b
		assign snap_amt_s2[gsa] = snap_amt5[gsa];
		assign snap_amt_s1_lo[gsa] = snap_amt5[gsa];
		assign snap_amt_s1_hi[gsa] = snap_amt5[gsa];
		assign snap_amt_s0_lo[gsa] = snap_amt5[gsa];
		assign snap_amt_s0_hi[gsa] = snap_amt5[gsa];
	end
	endgenerate
	wire [23:0] snap_sh1_lo_w, snap_sh1_hi_w;
	am_shift #(.W(24), .SW(4)) u_sn1_lo (.a(snap_s1_sn1[23:0]), .amt(snap_amt_s1_lo), .left(1'b0), .arith(1'b0), .y(snap_sh1_lo_w));
	am_shift #(.W(24), .SW(4)) u_sn1_hi (.a(snap_s1_sn1[31:8]), .amt(snap_amt_s1_hi), .left(1'b0), .arith(1'b0), .y(snap_sh1_hi_w));
	wire [15:0] snap_sh1 = {snap_sh1_hi_w[7:0], snap_sh1_lo_w[7:0]};
	wire [31:0] snap_sh1_full = {16'd0, snap_sh1_hi_w[23:16], snap_sh1_lo_w[23:16]};

	wire [15:0] snap_sh2;
	wire [23:0] snap_sh0_lo_w, snap_sh0_hi_w;
	am_shift #(.W(16), .SW(4)) u_sn2    (.a(snap_s1_sn2),        .amt(snap_amt_s2),    .left(1'b0), .arith(1'b0), .y(snap_sh2));
	am_shift #(.W(24), .SW(4)) u_sn0_lo (.a(snap_s1_sn0[23:0]),  .amt(snap_amt_s0_lo), .left(1'b0), .arith(1'b0), .y(snap_sh0_lo_w));
	am_shift #(.W(24), .SW(4)) u_sn0_hi (.a(snap_s1_sn0[31:8]),  .amt(snap_amt_s0_hi), .left(1'b0), .arith(1'b0), .y(snap_sh0_hi_w));
	wire [31:0] snap_sh0_full = {snap_sh0_hi_w[23:8], snap_sh0_hi_w[7:0], snap_sh0_lo_w[7:0]};

	wire [47:0] snap_shifted = {snap_sh2, snap_sh1, snap_sh0_full[15:0]};
`else
	/* ASIC 15 FO4: 2-bit fine shift (0..3) in parallel with 3-bit 4-bit-step coarse shift (0..28) (<= 9 cmos3) */
	wire [50:0] snap_fine_pad = {3'b000, snap_in};
	wire [47:0] snap_fine_0 = snap_fine_pad[47:0];
	wire [47:0] snap_fine_1 = snap_fine_pad[48:1];
	wire [47:0] snap_fine_2 = snap_fine_pad[49:2];
	wire [47:0] snap_fine_3 = snap_fine_pad[50:3];
	(* keep = "true" *) wire [47:0] snap_fine =
		({48{sub14_lo_oh[0]}} & snap_fine_0) |
		({48{sub14_lo_oh[1]}} & snap_fine_1) |
		({48{sub14_lo_oh[2]}} & snap_fine_2) |
		({48{sub14_lo_oh[3]}} & snap_fine_3);
	(* keep = "true" *) wire [47:0] snap_sh_lo4 =
		({48{sh_hi_oh[0]}} & snap_fine) |
		({48{sh_hi_oh[1]}} & {4'd0,  snap_fine[47:4]}) |
		({48{sh_hi_oh[2]}} & {8'd0,  snap_fine[47:8]}) |
		({48{sh_hi_oh[3]}} & {12'd0, snap_fine[47:12]});
	(* keep = "true" *) wire [47:0] snap_sh_hi4 =
		({48{sh_hi_oh[4]}} & {16'd0, snap_fine[47:16]}) |
		({48{sh_hi_oh[5]}} & {20'd0, snap_fine[47:20]}) |
		({48{sh_hi_oh[6]}} & {24'd0, snap_fine[47:24]}) |
		({48{sh_hi_oh[7]}} & {28'd0, snap_fine[47:28]});
	wire [47:0] snap_shifted = snap_sh_lo4 | snap_sh_hi4;
`endif

	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg [47:0] r_snap_shifted;
	(* extract_enable = "no", max_fanout = 16 *) reg r_snap_sign1, r_snap_ovf1, r_snap_zero_out1;
	(* extract_enable = "no", max_fanout = 16 *) reg [3:0] r_snap_tsn1, r_snap_tsn2;
	(* extract_enable = "no", max_fanout = 16 *) reg r_snap_v1, r_snap_v2;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg [23:0] r_s2_base;
	(* extract_enable = "no", max_fanout = 16 *) reg r_s2_cin, r_s2_sign, r_s2_ovf, r_s2_zero_out;
	(* extract_enable = "no", max_fanout = 16 *) reg [3:0] r_snap_tsn;
	(* extract_enable = "no", max_fanout = 16 *) reg r_snap_v;

	/* Pipelined f2fix stage 2a: sticky/sign pre-register */
	wire [23:0] s2_unrounded = r_snap_shifted[47:24];
	wire s2_guard = r_snap_shifted[23];
	wire s2_lsb = s2_unrounded[0];
	wire s2_sticky_lo = (r_snap_shifted[11:0] != 12'd0);
	wire s2_sticky_hi = (r_snap_shifted[22:12] != 11'd0);
	wire [23:0] s2_base = r_snap_sign1 ? ~s2_unrounded : s2_unrounded;
	wire s2_cin = r_snap_sign1 ^ (s2_guard && (s2_lsb || s2_sticky_lo || s2_sticky_hi));

	/* Pipelined f2fix stage 2b: 24-bit round + saturate from registered s2_base/cin */
	wire [23:0] s2_signed_rnd;
	am_add #(.W(24)) u_s2_rnd (
		.a(r_s2_base),
		.b(24'd0),
		.ci(r_s2_cin),
		.s(s2_signed_rnd),
		.co(carry_unused[50])
	);
	assign carry_unused[51] = 1'b0;

	wire [23:0] snap_out;
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_snap_out
		wire sgn_s, z_s, ov_s;
		assign sgn_s = r_s2_sign;
		assign z_s   = r_s2_zero_out;
		assign ov_s  = r_s2_ovf;
		wire [11:0] sat_s = sgn_s ? SAT_NEG[12 * gs +: 12] : SAT_POS[12 * gs +: 12];
		(* keep = "true" *) wire [11:0] out_s = z_s ? 12'd0 : (ov_s ? sat_s : s2_signed_rnd[12 * gs +: 12]);
		assign snap_out[12 * gs +: 12] = out_s;
	end
	endgenerate

	/* The registers read, in order: 31 32 34 35 37 38, then 76-79 (one-hot phase_oh[8:0]) */
	wire [6:0] rrv =
		phase_oh[0] ? 7'd32 : phase_oh[1] ? 7'd34 : phase_oh[2] ? 7'd35 :
		phase_oh[3] ? 7'd37 : phase_oh[4] ? 7'd38 : phase_oh[5] ? 7'd76 :
		phase_oh[6] ? 7'd77 : phase_oh[7] ? 7'd78 : 7'd79;

	/* Packed snapped coordinates sv[0..9] with 12-bit sliced write enables */
	(* dont_touch = "true", extract_reset = "no" *) reg [239:0] sv_pk;
	wire signed [23:0] sv [0:9];
	wire snap_v_act_raw = !start && run && r_snap_v;
	wire [1:0] snap_v_act;
	assign snap_v_act = {2{snap_v_act_raw}};
	generate
	for (gi = 0; gi < 10; gi = gi + 1) begin : g_sv_reg
		assign sv[gi] = sv_pk[24 * gi +: 24];
		wire sv_we_raw = snap_v_act[gi / 5] && (r_snap_tsn == gi[3:0]);
		wire sv_we_lo, sv_we_hi;
		assign sv_we_lo = sv_we_raw;
		assign sv_we_hi = sv_we_raw;
		always @(posedge clk) begin
			if (sv_we_lo) sv_pk[24 * gi +: 12]      <= snap_out[11:0];
			if (sv_we_hi) sv_pk[24 * gi + 12 +: 12] <= snap_out[23:12];
		end
	end
	endgenerate

	wire signed [23:0] ax = sv[0], ay = sv[1], bx = sv[2], by = sv[3];
	wire signed [23:0] ddx, ddy;
	wire [23:0] neg_ddx, neg_ddy;
	wire [23:0] adx, ady;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg signed [23:0] r_ddx, r_ddy;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg [23:0] r_adx, r_ady;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg signed [23:0] r_hx, r_hy;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg signed [23:0] r_hx_g, r_hy_g, r_hx_g_neg, r_hy_g_neg, r_hy_g_c3, r_hx_g_c4, r_hy_g_c5, r_hx_g_c6;
	(* extract_enable = "no", max_fanout = 16 *) reg r_xmaj;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(24)) u_ddx (.a(bx), .b(ax), .bi(1'b0), .d(ddx), .bo(carry_unused[3]));
	am_sub #(.W(24)) u_ddy (.a(by), .b(ay), .bi(1'b0), .d(ddy), .bo(carry_unused[4]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(24)) u_ddx (.a(bx), .b(ax), .bi(1'b0), .d(ddx), .bo(carry_unused[3]));
	am_sub #(.W(24)) u_ddy (.a(by), .b(ay), .bi(1'b0), .d(ddy), .bo(carry_unused[4]));
`else
	wire [11:0] ddx_lo, ddx_hi0, ddx_hi1, ddy_lo, ddy_hi0, ddy_hi1;
	wire ddx_bo12, ddy_bo12, ddx_u0, ddx_u1, ddy_u0, ddy_u1;
	am_sub #(.W(12)) u_ddx_lo  (.a(bx[11:0]),  .b(ax[11:0]),  .bi(1'b0), .d(ddx_lo),  .bo(ddx_bo12));
	am_sub #(.W(12)) u_ddx_hi0 (.a(bx[23:12]), .b(ax[23:12]), .bi(1'b0), .d(ddx_hi0), .bo(ddx_u0));
	am_sub #(.W(12)) u_ddx_hi1 (.a(bx[23:12]), .b(ax[23:12]), .bi(1'b1), .d(ddx_hi1), .bo(ddx_u1));
	am_sub #(.W(12)) u_ddy_lo  (.a(by[11:0]),  .b(ay[11:0]),  .bi(1'b0), .d(ddy_lo),  .bo(ddy_bo12));
	am_sub #(.W(12)) u_ddy_hi0 (.a(by[23:12]), .b(ay[23:12]), .bi(1'b0), .d(ddy_hi0), .bo(ddy_u0));
	am_sub #(.W(12)) u_ddy_hi1 (.a(by[23:12]), .b(ay[23:12]), .bi(1'b1), .d(ddy_hi1), .bo(ddy_u1));
	assign ddx = {ddx_bo12 ? ddx_hi1 : ddx_hi0, ddx_lo};
	assign ddy = {ddy_bo12 ? ddy_hi1 : ddy_hi0, ddy_lo};
	assign carry_unused[4:3] = {ddx_u0 & ddx_u1, ddy_u0 & ddy_u1};
`endif
`ifdef SYNTH_ASIC
	/* Pure-gate 3->9->24 prefix-OR abs for -r_ddx / -r_ddy (<= 7 cmos3) */
	wire [7:0] ddx_o3 = {
		|r_ddx[23:21], |r_ddx[20:18], |r_ddx[17:15], |r_ddx[14:12],
		|r_ddx[11:9],  |r_ddx[8:6],   |r_ddx[5:3],   |r_ddx[2:0]
	};
	wire [7:0] ddy_o3 = {
		|r_ddy[23:21], |r_ddy[20:18], |r_ddy[17:15], |r_ddy[14:12],
		|r_ddy[11:9],  |r_ddy[8:6],   |r_ddy[5:3],   |r_ddy[2:0]
	};
	wire ddx_o9_0 = |ddx_o3[2:0], ddx_o9_1 = |ddx_o3[5:3];
	wire ddy_o9_0 = |ddy_o3[2:0], ddy_o9_1 = |ddy_o3[5:3];
	wire [23:0] ddx_or_lo = {
		ddx_o9_0 | ddx_o9_1 | ddx_o3[6] | r_ddx[21] | r_ddx[22],
		ddx_o9_0 | ddx_o9_1 | ddx_o3[6] | r_ddx[21],
		ddx_o9_0 | ddx_o9_1 | ddx_o3[6],
		ddx_o9_0 | ddx_o9_1 | r_ddx[18] | r_ddx[19],
		ddx_o9_0 | ddx_o9_1 | r_ddx[18],
		ddx_o9_0 | ddx_o9_1,
		ddx_o9_0 | ddx_o3[3] | ddx_o3[4] | r_ddx[15] | r_ddx[16],
		ddx_o9_0 | ddx_o3[3] | ddx_o3[4] | r_ddx[15],
		ddx_o9_0 | ddx_o3[3] | ddx_o3[4],
		ddx_o9_0 | ddx_o3[3] | r_ddx[12] | r_ddx[13],
		ddx_o9_0 | ddx_o3[3] | r_ddx[12],
		ddx_o9_0 | ddx_o3[3],
		ddx_o9_0 | r_ddx[9] | r_ddx[10],
		ddx_o9_0 | r_ddx[9],
		ddx_o9_0,
		ddx_o3[0] | ddx_o3[1] | r_ddx[6] | r_ddx[7],
		ddx_o3[0] | ddx_o3[1] | r_ddx[6],
		ddx_o3[0] | ddx_o3[1],
		ddx_o3[0] | r_ddx[3] | r_ddx[4],
		ddx_o3[0] | r_ddx[3],
		ddx_o3[0],
		r_ddx[0] | r_ddx[1],
		r_ddx[0],
		1'b0
	};
	wire [23:0] ddy_or_lo = {
		ddy_o9_0 | ddy_o9_1 | ddy_o3[6] | r_ddy[21] | r_ddy[22],
		ddy_o9_0 | ddy_o9_1 | ddy_o3[6] | r_ddy[21],
		ddy_o9_0 | ddy_o9_1 | ddy_o3[6],
		ddy_o9_0 | ddy_o9_1 | r_ddy[18] | r_ddy[19],
		ddy_o9_0 | ddy_o9_1 | r_ddy[18],
		ddy_o9_0 | ddy_o9_1,
		ddy_o9_0 | ddy_o3[3] | ddy_o3[4] | r_ddy[15] | r_ddy[16],
		ddy_o9_0 | ddy_o3[3] | ddy_o3[4] | r_ddy[15],
		ddy_o9_0 | ddy_o3[3] | ddy_o3[4],
		ddy_o9_0 | ddy_o3[3] | r_ddy[12] | r_ddy[13],
		ddy_o9_0 | ddy_o3[3] | r_ddy[12],
		ddy_o9_0 | ddy_o3[3],
		ddy_o9_0 | r_ddy[9] | r_ddy[10],
		ddy_o9_0 | r_ddy[9],
		ddy_o9_0,
		ddy_o3[0] | ddy_o3[1] | r_ddy[6] | r_ddy[7],
		ddy_o3[0] | ddy_o3[1] | r_ddy[6],
		ddy_o3[0] | ddy_o3[1],
		ddy_o3[0] | r_ddy[3] | r_ddy[4],
		ddy_o3[0] | r_ddy[3],
		ddy_o3[0],
		r_ddy[0] | r_ddy[1],
		r_ddy[0],
		1'b0
	};
	assign adx = r_ddx ^ {24{r_ddx[23]}} & ddx_or_lo;
	assign ady = r_ddy ^ {24{r_ddy[23]}} & ddy_or_lo;
	assign carry_unused[6:5] = {ddx_o3[7] & neg_ddx[0], ddy_o3[7] & neg_ddy[0]};
	assign neg_ddx = ~r_ddx;
	assign neg_ddy = ~r_ddy;
`else
	am_sub #(.W(24)) u_adx (.a(24'd0), .b(r_ddx), .bi(1'b0), .d(neg_ddx), .bo(carry_unused[5]));
	am_sub #(.W(24)) u_ady (.a(24'd0), .b(r_ddy), .bi(1'b0), .d(neg_ddy), .bo(carry_unused[6]));
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_abs_dxy
		wire sx_s, sy_s;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sxs (.I0(r_ddx[23]), .O(sx_s));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sys (.I0(r_ddy[23]), .O(sy_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sx_s = r_ddx[23];
		assign sy_s = r_ddy[23];
`else
		assign sx_s = r_ddx[23];
		assign sy_s = r_ddy[23];
`endif
		assign adx[12 * gs +: 12] = sx_s ? neg_ddx[12 * gs +: 12] : r_ddx[12 * gs +: 12];
		assign ady[12 * gs +: 12] = sy_s ? neg_ddy[12 * gs +: 12] : r_ddy[12 * gs +: 12];
	end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
`ifdef MUT_LINE_MAJOR
	wire xmaj = r_adx > r_ady;	/* the TB's mutation check: y-major when equal */
`else
	wire xmaj = r_adx >= r_ady;
`endif
`elsif SYNTH_KINTEX_ULTRASCALE
`ifdef MUT_LINE_MAJOR
	wire xmaj = r_adx > r_ady;	/* the TB's mutation check: y-major when equal */
`else
	wire xmaj = r_adx >= r_ady;
`endif
`else
`ifdef MUT_LINE_MAJOR
	wire xmaj = r_adx > r_ady;
`else
	/* Pure-LUT 24-bit comparator r_adx >= r_ady using 2-bit base cells (<= 7 cmos3) */
	wire [23:0] xm_eq1 = ~(r_adx ^ r_ady);
	wire [11:0] xm_lt2;
	genvar gxm;
	generate
	for (gxm = 0; gxm < 12; gxm = gxm + 1) begin : g_xm_c2
		wire [1:0] xa2 = r_adx[2*gxm +: 2];
		wire [1:0] xb2 = r_ady[2*gxm +: 2];
		assign xm_lt2[gxm] = (~xa2[1] & xb2[1]) | ((~xa2[1] | xb2[1]) & ~xa2[0] & xb2[0]);
	end
	endgenerate
	wire [5:0] xm_lt4;
	wire [5:0] xm_eq4;
	generate
	for (gxm = 0; gxm < 6; gxm = gxm + 1) begin : g_xm_c4
		assign xm_lt4[gxm] = xm_lt2[2*gxm+1] | (xm_eq1[4*gxm+3] & xm_eq1[4*gxm+2] & xm_lt2[2*gxm]);
		assign xm_eq4[gxm] = &xm_eq1[4*gxm +: 4];
	end
	endgenerate
	wire [2:0] xm_lt8, xm_eq8;
	assign xm_lt8[0] = xm_lt4[1] | (xm_eq4[1] & xm_lt4[0]);
	assign xm_eq8[0] = &xm_eq1[7:0];
	assign xm_lt8[1] = xm_lt4[3] | (xm_eq4[3] & xm_lt4[2]);
	assign xm_eq8[1] = &xm_eq1[15:8];
	assign xm_lt8[2] = xm_lt4[5] | (xm_eq4[5] & xm_lt4[4]);
	assign xm_eq8[2] = &xm_eq1[23:16];
	wire xmaj = !(xm_lt8[2] | (xm_eq8[2] & xm_lt8[1]) | (xm_eq8[2] & xm_eq8[1] & xm_lt8[0]));
`endif
`endif
	wire [15:0] eff_line_w = line_width == 16'd0 ? 16'h0100 : line_width;
	wire [15:0] eff_point_s = point_size == 16'd0 ? 16'h0100 : point_size;
	wire signed [23:0] half_line_w = {9'd0, eff_line_w[15:1]};
	wire signed [23:0] half_point_s = {9'd0, eff_point_s[15:1]};

	wire is_line = r_is_line_c[0];
	wire is_line_hx = r_is_line_c[1];
	wire is_line_hy = r_is_line_c[2];
	wire is_line_w  = r_is_line_c[3];
	wire is_line_w2 = r_is_line_c[4];

	/* The corners: a triangle's vertices (and v0 again); a line's a-, a+,
	 * b-, b+; a point's p--, p+-, p++, p-+.  The walked triangle is corners
	 * (0, w1, w2). */
	wire signed [23:0] vx2 = sv[4], vy2 = sv[5];
	wire signed [23:0] hx = r_hx;
	wire signed [23:0] hy = r_hy;
	wire signed [23:0] c0x_n, c0y_n, c1x_n, c1y_n, c2x_n, c2y_n, c3x_n, c3y_n;
`ifdef SYNTH_KINTEX7
	/* Corners r_c0..r_c3: 8-bit sliced dedicated is_tri/is_line FFs (<= 16 loads each) */
	wire [23:0] hx_g_c0 = {r_is_tri_c[20] ? 8'd0 : hx[23:16], r_is_tri_c[19] ? 8'd0 : hx[15:8], r_is_tri_c[18] ? 8'd0 : hx[7:0]};
	wire [23:0] hx_g_c1 = {r_is_tri_c[23] ? 8'd0 : hx[23:16], r_is_tri_c[22] ? 8'd0 : hx[15:8], r_is_tri_c[21] ? 8'd0 : hx[7:0]};
	wire [23:0] b_c2x   = {r_is_tri_c[26] ? 8'd0 : (r_is_line_c[22] ? ~hx[23:16] : hx[23:16]),
	                       r_is_tri_c[25] ? 8'd0 : (r_is_line_c[21] ? ~hx[15:8]  : hx[15:8]),
	                       r_is_tri_c[24] ? 8'd0 : (r_is_line_c[20] ? ~hx[7:0]   : hx[7:0])};
	wire [23:0] b_c3x   = {r_is_tri_c[29] ? 8'd0 : (r_is_line_c[28] ? hx[23:16] : ~hx[23:16]),
	                       r_is_tri_c[28] ? 8'd0 : (r_is_line_c[27] ? hx[15:8]  : ~hx[15:8]),
	                       r_is_tri_c[27] ? 8'd0 : (r_is_line_c[26] ? hx[7:0]   : ~hx[7:0])};
	wire [23:0] hy_g_c0 = {r_is_tri_c[32] ? 8'd0 : hy[23:16], r_is_tri_c[31] ? 8'd0 : hy[15:8], r_is_tri_c[30] ? 8'd0 : hy[7:0]};
	wire [23:0] b_c1y   = {r_is_tri_c[35] ? 8'd0 : (r_is_line_c[19] ? hy[23:16] : ~hy[23:16]),
	                       r_is_tri_c[34] ? 8'd0 : (r_is_line_c[18] ? hy[15:8]  : ~hy[15:8]),
	                       r_is_tri_c[33] ? 8'd0 : (r_is_line_c[17] ? hy[7:0]   : ~hy[7:0])};
	wire [23:0] b_c2y   = {r_is_tri_c[38] ? 8'd0 : (r_is_line_c[25] ? ~hy[23:16] : hy[23:16]),
	                       r_is_tri_c[37] ? 8'd0 : (r_is_line_c[24] ? ~hy[15:8]  : hy[15:8]),
	                       r_is_tri_c[36] ? 8'd0 : (r_is_line_c[23] ? ~hy[7:0]   : hy[7:0])};
	wire [23:0] hy_g_c3 = {r_is_tri_c[41] ? 8'd0 : hy[23:16], r_is_tri_c[40] ? 8'd0 : hy[15:8], r_is_tri_c[39] ? 8'd0 : hy[7:0]};
	wire [23:0] c1_ax   = {r_is_tri_c[44] ? bx[23:16] : ax[23:16], r_is_tri_c[43] ? bx[15:8] : ax[15:8], r_is_tri_c[42] ? bx[7:0] : ax[7:0]};
	wire [23:0] c1_ay   = {r_is_tri_c[47] ? by[23:16] : ay[23:16], r_is_tri_c[46] ? by[15:8] : ay[15:8], r_is_tri_c[45] ? by[7:0] : ay[7:0]};
	wire [23:0] c2_ax   = {r_is_tri_c[50] ? vx2[23:16] : (r_is_line_c[31] ? bx[23:16] : ax[23:16]),
	                       r_is_tri_c[49] ? vx2[15:8]  : (r_is_line_c[30] ? bx[15:8]  : ax[15:8]),
	                       r_is_tri_c[48] ? vx2[7:0]   : (r_is_line_c[29] ? bx[7:0]   : ax[7:0])};
	wire [23:0] c2_ay   = {r_is_tri_c[53] ? vy2[23:16] : (r_is_line_c[34] ? by[23:16] : ay[23:16]),
	                       r_is_tri_c[52] ? vy2[15:8]  : (r_is_line_c[33] ? by[15:8]  : ay[15:8]),
	                       r_is_tri_c[51] ? vy2[7:0]   : (r_is_line_c[32] ? by[7:0]   : ay[7:0])};
	wire [23:0] c3_ax   = {(!r_is_tri_c[56] && r_is_line_c[37]) ? bx[23:16] : ax[23:16],
	                       (!r_is_tri_c[55] && r_is_line_c[36]) ? bx[15:8]  : ax[15:8],
	                       (!r_is_tri_c[54] && r_is_line_c[35]) ? bx[7:0]   : ax[7:0]};
	wire [23:0] c3_ay   = {(!r_is_tri_c[59] && r_is_line_c[40]) ? by[23:16] : ay[23:16],
	                       (!r_is_tri_c[58] && r_is_line_c[39]) ? by[15:8]  : ay[15:8],
	                       (!r_is_tri_c[57] && r_is_line_c[38]) ? by[7:0]   : ay[7:0]};
	am_sub #(.W(24)) u_c0x (.a(ax),    .b(hx_g_c0), .bi(1'b0), .d(c0x_n), .bo(carry_unused[7]));
	am_sub #(.W(24)) u_c0y (.a(ay),    .b(hy_g_c0), .bi(1'b0), .d(c0y_n), .bo(carry_unused[8]));
	am_add #(.W(24)) u_c1x (.a(c1_ax), .b(hx_g_c1), .ci(1'b0), .s(c1x_n), .co(carry_unused[9]));
	am_add #(.W(24)) u_c1y (.a(c1_ay), .b(b_c1y),   .ci(!r_is_tri_c[60] && !r_is_line_c[41]), .s(c1y_n), .co(carry_unused[10]));
	am_add #(.W(24)) u_c2x (.a(c2_ax), .b(b_c2x),   .ci(!r_is_tri_c[61] &&  r_is_line_c[42]), .s(c2x_n), .co(carry_unused[11]));
	am_add #(.W(24)) u_c2y (.a(c2_ay), .b(b_c2y),   .ci(!r_is_tri_c[62] &&  r_is_line_c[43]), .s(c2y_n), .co(carry_unused[12]));
	am_add #(.W(24)) u_c3x (.a(c3_ax), .b(b_c3x),   .ci(!r_is_tri_c[63] && !r_is_line_c[44]), .s(c3x_n), .co(carry_unused[13]));
	am_add #(.W(24)) u_c3y (.a(c3_ay), .b(hy_g_c3), .ci(1'b0), .s(c3y_n), .co(carry_unused[14]));
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Corners r_c0..r_c3: 8-bit sliced dedicated is_tri/is_line FFs (<= 16 loads each) */
	wire [23:0] hx_g_c0 = {r_is_tri_c[20] ? 8'd0 : hx[23:16], r_is_tri_c[19] ? 8'd0 : hx[15:8], r_is_tri_c[18] ? 8'd0 : hx[7:0]};
	wire [23:0] hx_g_c1 = {r_is_tri_c[23] ? 8'd0 : hx[23:16], r_is_tri_c[22] ? 8'd0 : hx[15:8], r_is_tri_c[21] ? 8'd0 : hx[7:0]};
	wire [23:0] b_c2x   = {r_is_tri_c[26] ? 8'd0 : (r_is_line_c[22] ? ~hx[23:16] : hx[23:16]),
	                       r_is_tri_c[25] ? 8'd0 : (r_is_line_c[21] ? ~hx[15:8]  : hx[15:8]),
	                       r_is_tri_c[24] ? 8'd0 : (r_is_line_c[20] ? ~hx[7:0]   : hx[7:0])};
	wire [23:0] b_c3x   = {r_is_tri_c[29] ? 8'd0 : (r_is_line_c[28] ? hx[23:16] : ~hx[23:16]),
	                       r_is_tri_c[28] ? 8'd0 : (r_is_line_c[27] ? hx[15:8]  : ~hx[15:8]),
	                       r_is_tri_c[27] ? 8'd0 : (r_is_line_c[26] ? hx[7:0]   : ~hx[7:0])};
	wire [23:0] hy_g_c0 = {r_is_tri_c[32] ? 8'd0 : hy[23:16], r_is_tri_c[31] ? 8'd0 : hy[15:8], r_is_tri_c[30] ? 8'd0 : hy[7:0]};
	wire [23:0] b_c1y   = {r_is_tri_c[35] ? 8'd0 : (r_is_line_c[19] ? hy[23:16] : ~hy[23:16]),
	                       r_is_tri_c[34] ? 8'd0 : (r_is_line_c[18] ? hy[15:8]  : ~hy[15:8]),
	                       r_is_tri_c[33] ? 8'd0 : (r_is_line_c[17] ? hy[7:0]   : ~hy[7:0])};
	wire [23:0] b_c2y   = {r_is_tri_c[38] ? 8'd0 : (r_is_line_c[25] ? ~hy[23:16] : hy[23:16]),
	                       r_is_tri_c[37] ? 8'd0 : (r_is_line_c[24] ? ~hy[15:8]  : hy[15:8]),
	                       r_is_tri_c[36] ? 8'd0 : (r_is_line_c[23] ? ~hy[7:0]   : hy[7:0])};
	wire [23:0] hy_g_c3 = {r_is_tri_c[41] ? 8'd0 : hy[23:16], r_is_tri_c[40] ? 8'd0 : hy[15:8], r_is_tri_c[39] ? 8'd0 : hy[7:0]};
	wire [23:0] c1_ax   = {r_is_tri_c[44] ? bx[23:16] : ax[23:16], r_is_tri_c[43] ? bx[15:8] : ax[15:8], r_is_tri_c[42] ? bx[7:0] : ax[7:0]};
	wire [23:0] c1_ay   = {r_is_tri_c[47] ? by[23:16] : ay[23:16], r_is_tri_c[46] ? by[15:8] : ay[15:8], r_is_tri_c[45] ? by[7:0] : ay[7:0]};
	wire [23:0] c2_ax   = {r_is_tri_c[50] ? vx2[23:16] : (r_is_line_c[31] ? bx[23:16] : ax[23:16]),
	                       r_is_tri_c[49] ? vx2[15:8]  : (r_is_line_c[30] ? bx[15:8]  : ax[15:8]),
	                       r_is_tri_c[48] ? vx2[7:0]   : (r_is_line_c[29] ? bx[7:0]   : ax[7:0])};
	wire [23:0] c2_ay   = {r_is_tri_c[53] ? vy2[23:16] : (r_is_line_c[34] ? by[23:16] : ay[23:16]),
	                       r_is_tri_c[52] ? vy2[15:8]  : (r_is_line_c[33] ? by[15:8]  : ay[15:8]),
	                       r_is_tri_c[51] ? vy2[7:0]   : (r_is_line_c[32] ? by[7:0]   : ay[7:0])};
	wire [23:0] c3_ax   = {(!r_is_tri_c[56] && r_is_line_c[37]) ? bx[23:16] : ax[23:16],
	                       (!r_is_tri_c[55] && r_is_line_c[36]) ? bx[15:8]  : ax[15:8],
	                       (!r_is_tri_c[54] && r_is_line_c[35]) ? bx[7:0]   : ax[7:0]};
	wire [23:0] c3_ay   = {(!r_is_tri_c[59] && r_is_line_c[40]) ? by[23:16] : ay[23:16],
	                       (!r_is_tri_c[58] && r_is_line_c[39]) ? by[15:8]  : ay[15:8],
	                       (!r_is_tri_c[57] && r_is_line_c[38]) ? by[7:0]   : ay[7:0]};
	am_sub #(.W(24)) u_c0x (.a(ax),    .b(hx_g_c0), .bi(1'b0), .d(c0x_n), .bo(carry_unused[7]));
	am_sub #(.W(24)) u_c0y (.a(ay),    .b(hy_g_c0), .bi(1'b0), .d(c0y_n), .bo(carry_unused[8]));
	am_add #(.W(24)) u_c1x (.a(c1_ax), .b(hx_g_c1), .ci(1'b0), .s(c1x_n), .co(carry_unused[9]));
	am_add #(.W(24)) u_c1y (.a(c1_ay), .b(b_c1y),   .ci(!r_is_tri_c[60] && !r_is_line_c[41]), .s(c1y_n), .co(carry_unused[10]));
	am_add #(.W(24)) u_c2x (.a(c2_ax), .b(b_c2x),   .ci(!r_is_tri_c[61] &&  r_is_line_c[42]), .s(c2x_n), .co(carry_unused[11]));
	am_add #(.W(24)) u_c2y (.a(c2_ay), .b(b_c2y),   .ci(!r_is_tri_c[62] &&  r_is_line_c[43]), .s(c2y_n), .co(carry_unused[12]));
	am_add #(.W(24)) u_c3x (.a(c3_ax), .b(b_c3x),   .ci(!r_is_tri_c[63] && !r_is_line_c[44]), .s(c3x_n), .co(carry_unused[13]));
	am_add #(.W(24)) u_c3y (.a(c3_ay), .b(hy_g_c3), .ci(1'b0), .s(c3y_n), .co(carry_unused[14]));
`else
	/* ASIC 15 FO4: 8+8+8 direct-CLA carry-select corners using pre-gated r_hx_g/r_hy_g & 1-hot mode */
	wire [23:0] cn_a [0:7];
	reg  [23:0] r_cn_a [0:7];
	wire [23:0] cn_b [0:7];
	wire [23:0] cn_res [0:7];
	wire [7:0] cn_sub;
	assign cn_a[0] = ax;
	assign cn_a[1] = ay;
	assign cn_a[2] = ({24{r_is_tri_c[1]}} & bx) | ({24{!r_is_tri_c[1]}} & ax);
	assign cn_a[3] = ({24{r_is_tri_c[1]}} & by) | ({24{!r_is_tri_c[1]}} & ay);
	assign cn_a[4] = ({24{r_is_tri_c[2]}} & vx2) | ({24{r_is_line_c[2]}} & bx) | ({24{r_is_pt_c[2]}} & ax);
	assign cn_a[5] = ({24{r_is_tri_c[2]}} & vy2) | ({24{r_is_line_c[2]}} & by) | ({24{r_is_pt_c[2]}} & ay);
	assign cn_a[6] = ({24{r_is_line_c[3]}} & bx) | ({24{!r_is_line_c[3]}} & ax);
	assign cn_a[7] = ({24{r_is_line_c[3]}} & by) | ({24{!r_is_line_c[3]}} & ay);

	assign cn_b[0] = r_hx_g_neg; assign cn_b[1] = r_hy_g_neg;
	assign cn_b[2] = r_hx_g;     assign cn_b[3] = r_hy_g_c3;
	assign cn_b[4] = r_hx_g_c4;  assign cn_b[5] = r_hy_g_c5;
	assign cn_b[6] = r_hx_g_c6;  assign cn_b[7] = r_hy_g;

	assign cn_sub[0] = 1'b1;
	assign cn_sub[1] = 1'b1;
	assign cn_sub[2] = 1'b0;
	assign cn_sub[3] = r_is_pt_c[1];
	assign cn_sub[4] = r_is_line_c[2];
	assign cn_sub[5] = r_is_line_c[2];
	assign cn_sub[6] = r_is_pt_c[3];
	assign cn_sub[7] = 1'b0;

	function [3:0] edge_cla3;
		input [2:0] a, b;
		input ci;
		reg [2:0] p, g, c;
		begin
			p = a ^ b;
			g = a & b;
			c[0] = ci;
			c[1] = g[0] | (p[0] & ci);
			c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & ci);
			edge_cla3 = {
				g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & ci),
				p ^ c
			};
		end
	endfunction

	generate
	for (gi = 0; gi < 8; gi = gi + 1) begin : g_corner_cs
		always @(posedge clk) begin
			if (run && phase_oh[10])
				r_cn_a[gi] <= cn_a[gi];
		end
`ifdef SYNTH_KINTEX7
		am_addsub #(.W(24)) u_cn (.a(r_cn_a[gi]), .b(cn_b[gi]), .sub(cn_sub[gi]), .s(cn_res[gi]), .co(carry_unused[7 + gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
		am_addsub #(.W(24)) u_cn (.a(r_cn_a[gi]), .b(cn_b[gi]), .sub(cn_sub[gi]), .s(cn_res[gi]), .co(carry_unused[7 + gi]));
`else
		wire [2:0] s0_0 [0:7];
		wire [2:0] s0_1 [0:7];
		wire [7:0] g3_w, p3_w;
		genvar gcn;
		for (gcn = 0; gcn < 8; gcn = gcn + 1) begin : g_cn3
			wire [2:0] ca = r_cn_a[gi][3*gcn +: 3];
			wire [2:0] cb = cn_b[gi][3*gcn +: 3];
			wire [2:0] pg = ca | cb;
			wire [2:0] gg = ca & cb;
			wire [3:0] r0 = edge_cla3(ca, cb, 1'b0);
			wire [3:0] r1 = edge_cla3(ca, cb, 1'b1);
			assign s0_0[gcn] = r0[2:0];
			assign s0_1[gcn] = r1[2:0];
			assign p3_w[gcn] = &pg;
			assign g3_w[gcn] = gg[2] | (pg[2] & gg[1]) | (pg[2] & pg[1] & gg[0]);
		end
		wire g6_0 = g3_w[1] | (p3_w[1] & g3_w[0]);
		wire p6_0 = p3_w[1] & p3_w[0];
		wire g6_1 = g3_w[3] | (p3_w[3] & g3_w[2]);
		wire p6_1 = p3_w[3] & p3_w[2];
		wire g6_2 = g3_w[5] | (p3_w[5] & g3_w[4]);
		wire p6_2 = p3_w[5] & p3_w[4];
		wire g12_0 = g6_1 | (p6_1 & g6_0);
		wire p12_0 = p6_1 & p6_0;
		wire ci_s = cn_sub[gi];
		wire [7:0] cb_sel;
		wire cb_s2 = g6_0 | (p6_0 & ci_s);
		wire cb_s4 = g12_0 | (p12_0 & ci_s);
		wire cb_s6 = g6_2 | (p6_2 & g12_0) | (p6_2 & p12_0 & ci_s);
		assign cb_sel[0] = ci_s;
		assign cb_sel[1] = g3_w[0] | (p3_w[0] & ci_s);
		assign cb_sel[2] = cb_s2;
		assign cb_sel[3] = g3_w[2] | (p3_w[2] & cb_s2);
		assign cb_sel[4] = cb_s4;
		assign cb_sel[5] = g3_w[4] | (p3_w[4] & cb_s4);
		assign cb_sel[6] = cb_s6;
		assign cb_sel[7] = g3_w[6] | (p3_w[6] & cb_s6);
		for (gcn = 0; gcn < 8; gcn = gcn + 1) begin : g_cn_mux
			assign cn_res[gi][3*gcn +: 3] = cb_sel[gcn] ? s0_1[gcn] : s0_0[gcn];
		end
		assign carry_unused[7 + gi] = g3_w[7] & p3_w[7];
`endif
	end
	endgenerate
	assign c0x_n = cn_res[0];
	assign c0y_n = cn_res[1];
	assign c1x_n = cn_res[2];
	assign c1y_n = cn_res[3];
	assign c2x_n = cn_res[4];
	assign c2y_n = cn_res[5];
	assign c3x_n = cn_res[6];
	assign c3y_n = cn_res[7];
`endif

	(* dont_touch = "true", extract_reset = "no" *) reg [191:0] r_c_pk;
	wire signed [23:0] r_c0x = r_c_pk[0   +: 24];
	wire signed [23:0] r_c0y = r_c_pk[24  +: 24];
	wire signed [23:0] r_c1x = r_c_pk[48  +: 24];
	wire signed [23:0] r_c1y = r_c_pk[72  +: 24];
	wire signed [23:0] r_c2x = r_c_pk[96  +: 24];
	wire signed [23:0] r_c2y = r_c_pk[120 +: 24];
	wire signed [23:0] r_c3x = r_c_pk[144 +: 24];
	wire signed [23:0] r_c3y = r_c_pk[168 +: 24];

	wire c9_raw = !start && run && phase_oh[11];
	wire [1:0] c9_mid;
	generate
	for (gi = 0; gi < 2; gi = gi + 1) begin : g_c9_mid
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c9m (.I0(c9_raw), .O(c9_mid[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign c9_mid[gi] = c9_raw;
`else
		assign c9_mid[gi] = c9_raw;
`endif
	end
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_c_half
		localparam integer HI = 12 * gs;
		wire [7:0] we_c;
		for (gi = 0; gi < 8; gi = gi + 1) begin : g_c_buf
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wec (.I0(c9_mid[gs]), .O(we_c[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign we_c[gi] = c9_mid[gs];
`else
			assign we_c[gi] = c9_mid[gs];
`endif
		end
		always @(posedge clk) begin
			if (we_c[0]) r_c_pk[0   + HI +: 12] <= c0x_n[HI +: 12];
			if (we_c[1]) r_c_pk[24  + HI +: 12] <= c0y_n[HI +: 12];
			if (we_c[2]) r_c_pk[48  + HI +: 12] <= c1x_n[HI +: 12];
			if (we_c[3]) r_c_pk[72  + HI +: 12] <= c1y_n[HI +: 12];
			if (we_c[4]) r_c_pk[96  + HI +: 12] <= c2x_n[HI +: 12];
			if (we_c[5]) r_c_pk[120 + HI +: 12] <= c2y_n[HI +: 12];
			if (we_c[6]) r_c_pk[144 + HI +: 12] <= c3x_n[HI +: 12];
			if (we_c[7]) r_c_pk[168 + HI +: 12] <= c3y_n[HI +: 12];
		end
	end
	endgenerate

	wire [1:0] w1 = r_is_tri_c[8] ? 2'd1 : is_line_w  ? (md[2] ? 2'd3 : 2'd2) : (md[2] ? 2'd2 : 2'd1);
	wire [1:0] w2 = r_is_tri_c[9] ? 2'd2 : is_line_w2 ? (md[2] ? 2'd1 : 2'd3) : (md[2] ? 2'd3 : 2'd2);

	/* The walked triangle, the four corners (bbox) and the plane triangle. */
	(* dont_touch = "true", extract_reset = "no" *) reg [71:0] fx_pk;
	(* dont_touch = "true", extract_reset = "no" *) reg [71:0] fy_pk;
	wire signed [23:0] fx [0:2];
	wire signed [23:0] fy [0:2];
	assign fx[0] = fx_pk[0  +: 24];
	assign fx[1] = fx_pk[24 +: 24];
	assign fx[2] = fx_pk[48 +: 24];
	assign fy[0] = fy_pk[0  +: 24];
	assign fy[1] = fy_pk[24 +: 24];
	assign fy[2] = fy_pk[48 +: 24];

	(* dont_touch = "true", extract_reset = "no" *) reg signed [23:0] pdx1, pdy1, pdx2, pdy2, p0x, p0y;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [47:0] pdd;	/* the plane triangle's D (lines and points) */
	wire signed [23:0] fx0 = fx[0], fx1 = fx[1], fx2 = fx[2];
	wire signed [23:0] fy0 = fy[0], fy1 = fy[1], fy2 = fy[2];
	wire signed [23:0] dx1, dy1, dx2, dy2;
	am_sub #(.W(24)) u_dx1 (.a(fx1), .b(fx0), .bi(1'b0), .d(dx1), .bo(carry_unused[15]));
	am_sub #(.W(24)) u_dy1 (.a(fy1), .b(fy0), .bi(1'b0), .d(dy1), .bo(carry_unused[16]));
	am_sub #(.W(24)) u_dx2 (.a(fx2), .b(fx0), .bi(1'b0), .d(dx2), .bo(carry_unused[17]));
	am_sub #(.W(24)) u_dy2 (.a(fy2), .b(fy0), .bi(1'b0), .d(dy2), .bo(carry_unused[18]));
	(* dont_touch = "true", extract_reset = "no" *) reg signed [23:0] r_dx1, r_dy1, r_dx2, r_dy2;

	/* bbox: the corners' pixels, max exclusive, ∩ scissor (when SCISSOR_MAX
	 * != 0) ∩ the viewport's rectangle (vprect, not raw) ∩ RT. */
	function signed [17:0] pix;
		input [15:0] v;	/* a 15.8 value's integer bits */
		pix = {{2{v[15]}}, v};
	endfunction
	wire signed [23:0] q0x = r_c0x, q1x = r_c1x, q2x = r_c2x, q3x = r_c3x;
	wire signed [23:0] q0y = r_c0y, q1y = r_c1y, q2y = r_c2y, q3y = r_c3y;
	wire signed [23:0] s6 = sv[6], s7 = sv[7], s8 = sv[8], s9 = sv[9];
	wire signed [17:0] rx0 = pix(s6[23:8]), rx1 = pix(s7[23:8]);
	wire signed [17:0] ry0 = pix(s8[23:8]), ry1 = pix(s9[23:8]);
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_min01_x, r_max01_x, r_min23_x, r_max23_x;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_min01_y, r_max01_y, r_min23_y, r_max23_y;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_lx, r_ly, r_mx, r_my;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_mx1, r_my1;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_clx, r_cly, r_cmx, r_cmy;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] bx0, by0, bx1, by1;
	wire [17:0] m_x1, m_y1;	/* the largest pixel + 1 (calculated in Stage 2) */
	am_add #(.W(18)) u_m_x1 (.a(r_mx), .b(18'd0), .ci(1'b1), .s(m_x1), .co(carry_unused[19]));
	am_add #(.W(18)) u_m_y1 (.a(r_my), .b(18'd0), .ci(1'b1), .s(m_y1), .co(carry_unused[20]));

	/* Stage 3 of bbox: scissor, viewport, and RT clamping */
	(* max_fanout = 16 *) reg r_scis_en;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_scis_rt_min_x, r_scis_rt_max_x;
	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_scis_rt_min_y, r_scis_rt_max_y;

	(* dont_touch = "true", extract_reset = "no" *) reg signed [17:0] r_win_min_x, r_win_max_x, r_win_min_y, r_win_max_y;
	function lt18_u;
		input [17:0] a;
		input [17:0] b;
		reg [17:0] eq1;
		reg [8:0] lt2;
		reg [2:0] lt6, eq6;
		integer i2;
		begin
			eq1 = ~(a ^ b);
			for (i2 = 0; i2 < 9; i2 = i2 + 1)
				lt2[i2] = (~a[2*i2+1] & b[2*i2+1]) | ((~a[2*i2+1] | b[2*i2+1]) & ~a[2*i2] & b[2*i2]);
			for (i2 = 0; i2 < 3; i2 = i2 + 1) begin
				lt6[i2] = lt2[3*i2+2] | ((&eq1[6*i2+4 +: 2]) & lt2[3*i2+1]) | ((&eq1[6*i2+2 +: 4]) & lt2[3*i2]);
				eq6[i2] = &eq1[6*i2 +: 6];
			end
			lt18_u = lt6[2] | (eq6[2] & lt6[1]) | (eq6[2] & eq6[1] & lt6[0]);
		end
	endfunction
`ifdef SYNTH_KINTEX7
	wire win_mnx_bo, win_mxx_bo, win_mny_bo, win_mxy_bo;
	am_sub #(.W(18)) u_wmnx (.a(r_scis_rt_min_x), .b(rx0), .bi(1'b0), .d(cmp_d_unused[0  +: 18]), .bo(win_mnx_bo));
	am_sub #(.W(18)) u_wmxx (.a(rx1), .b(r_scis_rt_max_x), .bi(1'b0), .d(cmp_d_unused[18 +: 18]), .bo(win_mxx_bo));
	am_sub #(.W(18)) u_wmny (.a(r_scis_rt_min_y), .b(ry0), .bi(1'b0), .d(cmp_d_unused[36 +: 18]), .bo(win_mny_bo));
	am_sub #(.W(18)) u_wmxy (.a(ry1), .b(r_scis_rt_max_y), .bi(1'b0), .d(cmp_d_unused[54 +: 18]), .bo(win_mxy_bo));
	wire sc_mnx_bo, sc_mxx_bo, sc_mny_bo, sc_mxy_bo;
	am_sub #(.W(18)) u_smnx (.a(r_lx),  .b(r_win_min_x), .bi(1'b0), .d(cmp_d_unused[72  +: 18]), .bo(sc_mnx_bo));
	am_sub #(.W(18)) u_smxx (.a(r_win_max_x), .b(r_mx1), .bi(1'b0), .d(cmp_d_unused[90  +: 18]), .bo(sc_mxx_bo));
	am_sub #(.W(18)) u_smny (.a(r_ly),  .b(r_win_min_y), .bi(1'b0), .d(cmp_d_unused[108 +: 18]), .bo(sc_mny_bo));
	am_sub #(.W(18)) u_smxy (.a(r_win_max_y), .b(r_my1), .bi(1'b0), .d(cmp_d_unused[126 +: 18]), .bo(sc_mxy_bo));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire win_mnx_bo, win_mxx_bo, win_mny_bo, win_mxy_bo;
	am_sub #(.W(18)) u_wmnx (.a(r_scis_rt_min_x), .b(rx0), .bi(1'b0), .d(cmp_d_unused[0  +: 18]), .bo(win_mnx_bo));
	am_sub #(.W(18)) u_wmxx (.a(rx1), .b(r_scis_rt_max_x), .bi(1'b0), .d(cmp_d_unused[18 +: 18]), .bo(win_mxx_bo));
	am_sub #(.W(18)) u_wmny (.a(r_scis_rt_min_y), .b(ry0), .bi(1'b0), .d(cmp_d_unused[36 +: 18]), .bo(win_mny_bo));
	am_sub #(.W(18)) u_wmxy (.a(ry1), .b(r_scis_rt_max_y), .bi(1'b0), .d(cmp_d_unused[54 +: 18]), .bo(win_mxy_bo));
	wire sc_mnx_bo, sc_mxx_bo, sc_mny_bo, sc_mxy_bo;
	am_sub #(.W(18)) u_smnx (.a(r_lx),  .b(r_win_min_x), .bi(1'b0), .d(cmp_d_unused[72  +: 18]), .bo(sc_mnx_bo));
	am_sub #(.W(18)) u_smxx (.a(r_win_max_x), .b(r_mx1), .bi(1'b0), .d(cmp_d_unused[90  +: 18]), .bo(sc_mxx_bo));
	am_sub #(.W(18)) u_smny (.a(r_ly),  .b(r_win_min_y), .bi(1'b0), .d(cmp_d_unused[108 +: 18]), .bo(sc_mny_bo));
	am_sub #(.W(18)) u_smxy (.a(r_win_max_y), .b(r_my1), .bi(1'b0), .d(cmp_d_unused[126 +: 18]), .bo(sc_mxy_bo));
`else
	wire win_mnx_bo = lt18_u(r_scis_rt_min_x, rx0);
	wire win_mxx_bo = lt18_u(rx1, r_scis_rt_max_x);
	wire win_mny_bo = lt18_u(r_scis_rt_min_y, ry0);
	wire win_mxy_bo = lt18_u(ry1, r_scis_rt_max_y);
	wire sc_mnx_bo  = lt18_u(r_lx, r_win_min_x);
	wire sc_mxx_bo  = lt18_u(r_win_max_x, r_mx1);
	wire sc_mny_bo  = lt18_u(r_ly, r_win_min_y);
	wire sc_mxy_bo  = lt18_u(r_win_max_y, r_my1);
	assign cmp_d_unused[143:0] = 144'd0;
`endif
	(* keep = "true" *) wire win_min_x_sel = vprect && !md[3] && ((r_scis_rt_min_x[17] != rx0[17]) ? r_scis_rt_min_x[17] : win_mnx_bo);
	(* keep = "true" *) wire win_max_x_sel = vprect && !md[3] && ((rx1[17] != r_scis_rt_max_x[17]) ? rx1[17] : win_mxx_bo);
	(* keep = "true" *) wire win_min_y_sel = vprect && !md[3] && ((r_scis_rt_min_y[17] != ry0[17]) ? r_scis_rt_min_y[17] : win_mny_bo);
	(* keep = "true" *) wire win_max_y_sel = vprect && !md[3] && ((ry1[17] != r_scis_rt_max_y[17]) ? ry1[17] : win_mxy_bo);
	wire [3:0] win_cmp_unused = {win_max_y_sel, win_min_y_sel, win_max_x_sel, win_min_x_sel};
	(* extract_enable = "no", extract_reset = "no" *) reg r_win_mnx_sel, r_win_mny_sel;

	(* keep = "true" *) wire lo_scis_x_sel = (r_lx[17] != r_win_min_x[17]) ? r_lx[17] : sc_mnx_bo;
	(* keep = "true" *) wire hi_scis_x_sel = (r_win_max_x[17] != r_mx1[17]) ? r_win_max_x[17] : sc_mxx_bo;
	(* keep = "true" *) wire lo_scis_y_sel = (r_ly[17] != r_win_min_y[17]) ? r_ly[17] : sc_mny_bo;
	(* keep = "true" *) wire hi_scis_y_sel = (r_win_max_y[17] != r_my1[17]) ? r_win_max_y[17] : sc_mxy_bo;

	/* The sample at the bbox origin, in subpixels. */
	wire signed [24:0] sx = {bx0[16:0], 8'd128};
	wire signed [24:0] sy = {by0[16:0], 8'd128};

	(* dont_touch = "true", extract_reset = "no" *) reg signed [47:0] dd;	/* D = dx1 dy2 - dx2 dy1 (subpixel^2); area2 = -D */
	(* dont_touch = "true", extract_reset = "no" *) reg signed [47:0] r_dx1_dy2;
	wire ccw = dd[47];
	(* dont_touch = "true", extract_reset = "no" *) reg [71:0] eA_pk;
	(* dont_touch = "true", extract_reset = "no" *) reg [71:0] eB_pk;
	(* dont_touch = "true", extract_reset = "no" *) reg [74:0] oX_pk;
	(* dont_touch = "true", extract_reset = "no" *) reg [74:0] oY_pk;
	wire signed [23:0] eA [0:2];
	wire signed [23:0] eB [0:2];
	wire signed [24:0] oX [0:2];
	wire signed [24:0] oY [0:2];
	assign eA[0] = eA_pk[0  +: 24];
	assign eA[1] = eA_pk[24 +: 24];
	assign eA[2] = eA_pk[48 +: 24];
	assign eB[0] = eB_pk[0  +: 24];
	assign eB[1] = eB_pk[24 +: 24];
	assign eB[2] = eB_pk[48 +: 24];
	assign oX[0] = oX_pk[0  +: 25];
	assign oX[1] = oX_pk[25 +: 25];
	assign oX[2] = oX_pk[50 +: 25];
	assign oY[0] = oY_pk[0  +: 25];
	assign oY[1] = oY_pk[25 +: 25];
	assign oY[2] = oY_pk[50 +: 25];

	reg [2:0] tl;
	(* dont_touch = "true", extract_reset = "no" *) reg [143:0] eE_pk;
	wire [47:0] eE [0:2];
	assign eE[0] = eE_pk[0  +: 48];
	assign eE[1] = eE_pk[48 +: 48];
	assign eE[2] = eE_pk[96 +: 48];
	(* dont_touch = "true", extract_reset = "no" *) reg [49:0] acc;

	/* Vertex of the ordered (area2 > 0) triangle's corner i: (0,1,2) or (0,2,1). */
	function [1:0] vtx;
		input [1:0] i;
		input c;
		vtx = c ? i : (i == 2'd0 ? 2'd0 : 2'd3 - i);
	endfunction

	/* Steps 23-29 select DX1 DY1 DX2 DY2 DF OX OY of the plane triangle (OX, OY:
	 * dedicated subtracters calculated at cnt 16-17). */
	(* dont_touch = "true", extract_reset = "no" *) reg [47:0] r_cox_d, r_coy_d;
	wire [47:0] cox_d, coy_d;
	wire sx_s_lo, sx_s_hi, sy_s_lo, sy_s_hi;
	wire p0x_s_lo, p0x_s_hi, p0y_s_lo, p0y_s_hi;
	wire pdx1_s_lo, pdx1_s_hi, pdy1_s_lo, pdy1_s_hi, pdx2_s_lo, pdx2_s_hi, pdy2_s_lo, pdy2_s_hi;
	assign sx_s_lo   = sx[24];
	assign sx_s_hi   = sx[24];
	assign sy_s_lo   = sy[24];
	assign sy_s_hi   = sy[24];
	assign p0x_s_lo  = p0x[23];
	assign p0x_s_hi  = p0x[23];
	assign p0y_s_lo  = p0y[23];
	assign p0y_s_hi  = p0y[23];
	assign pdx1_s_lo = pdx1[23];
	assign pdx1_s_hi = pdx1[23];
	assign pdy1_s_lo = pdy1[23];
	assign pdy1_s_hi = pdy1[23];
	assign pdx2_s_lo = pdx2[23];
	assign pdx2_s_hi = pdx2[23];
	assign pdy2_s_lo = pdy2[23];
	assign pdy2_s_hi = pdy2[23];
	wire [47:0] sx_ext   = {{12{sx_s_hi}},   {11{sx_s_lo}},   sx};
	wire [47:0] sy_ext   = {{12{sy_s_hi}},   {11{sy_s_lo}},   sy};
	wire [47:0] p0x_ext  = {{12{p0x_s_hi}},  {12{p0x_s_lo}},  p0x};
	wire [47:0] p0y_ext  = {{12{p0y_s_hi}},  {12{p0y_s_lo}},  p0y};
	wire [47:0] pdx1_ext = {{12{pdx1_s_hi}}, {12{pdx1_s_lo}}, pdx1};
	wire [47:0] pdy1_ext = {{12{pdy1_s_hi}}, {12{pdy1_s_lo}}, pdy1};
	wire [47:0] pdx2_ext = {{12{pdx2_s_hi}}, {12{pdx2_s_lo}}, pdx2};
	wire [47:0] pdy2_ext = {{12{pdy2_s_hi}}, {12{pdy2_s_lo}}, pdy2};

	wire [25:0] cox_d26, coy_d26;
	am_sub #(.W(26)) u_cox_sub (.a({sx[24], sx}), .b({{2{p0x[23]}}, p0x}), .bi(1'b0), .d(cox_d26), .bo(carry_unused[25]));
	am_sub #(.W(26)) u_coy_sub (.a({sy[24], sy}), .b({{2{p0y[23]}}, p0y}), .bi(1'b0), .d(coy_d26), .bo(carry_unused[47]));
	assign cox_d = {{22{cox_d26[25]}}, cox_d26};
	assign coy_d = {{22{coy_d26[25]}}, coy_d26};
	wire unused_edge_exts = &{1'b0, sx_ext, sy_ext, p0x_ext, p0y_ext};

	/* int2f: a 48-bit signed integer times 2^-sh to fp32, four stages:
	 * Stage 1 (cnt 23-29): absolute value & leading zeros
	 * Stage 2 (cnt 24-30): latch abs & shift amount
	 * Stage 3 (cnt 25-31): barrel shifter and exponent addition
	 * Stage 4 (cnt 26-32): rounding increment, exponent subtract, and pack into fp registers 40-46 */
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg [47:0] r_cabs;
	(* extract_enable = "no", max_fanout = 16 *) reg r_cs;
	(* extract_enable = "no", max_fanout = 16 *) reg [4:0] r_csh;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg [47:0] ca;
	reg [2:0] r_cabs_nz;
	(* extract_enable = "no", max_fanout = 16 *) reg cs;
	(* extract_enable = "no", max_fanout = 16 *) reg [5:0] clz;
	(* extract_enable = "no", max_fanout = 16 *) reg [5:0] r_clz_hi_oh_a, r_clz_hi_oh_b, r_clz_hi_oh_c;
	(* extract_enable = "no", max_fanout = 16 *) reg [3:0] r_clz_lo_oh_a, r_clz_lo_oh_b, r_clz_lo_oh_c;
	(* extract_enable = "no", max_fanout = 16 *) reg r_clz0_a, r_clz0_b, r_clz0_c;
	(* extract_enable = "no", max_fanout = 16 *) reg [4:0] csh;
	(* dont_touch = "true", extract_enable = "no", extract_reset = "no", max_fanout = 16 *) reg [47:0] r_ci;
	(* extract_enable = "no", max_fanout = 16 *) reg [4:0] r_ci_sh;
	(* extract_enable = "no", max_fanout = 16 *) reg r_ci_sgn;

	wire [47:0] neg_ci, cabs;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(48)) u_cabs (.a(48'd0), .b(r_ci), .bi(1'b0), .d(neg_ci), .bo(carry_unused[21]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(48)) u_cabs (.a(48'd0), .b(r_ci), .bi(1'b0), .d(neg_ci), .bo(carry_unused[21]));
`else
	/* Parallel byte-zero carry-free cabs negation (<= 7 cmos3) */
	wire [5:0] nci_zb = {
		(r_ci[47:40] == 8'd0),
		(r_ci[39:32] == 8'd0),
		(r_ci[31:24] == 8'd0),
		(r_ci[23:16] == 8'd0),
		(r_ci[15:8]  == 8'd0),
		(r_ci[7:0]   == 8'd0)
	};
	wire [7:0] nci_s_p1 [0:5];
	wire [5:0] nci_u_co;
	genvar gnc;
	generate
	for (gnc = 0; gnc < 6; gnc = gnc + 1) begin : g_nci8
		wire [7:0] nb = ~r_ci[8*gnc +: 8];
		wire [3:0] l_s = {nb[3] ^ (&nb[2:0]), nb[2] ^ (nb[1] & nb[0]), nb[1] ^ nb[0], ~nb[0]};
		wire l_co = &nb[3:0];
		wire [3:0] h_s1 = {nb[7] ^ (&nb[6:4]), nb[6] ^ (nb[5] & nb[4]), nb[5] ^ nb[4], ~nb[4]};
		assign nci_s_p1[gnc] = {l_co ? h_s1 : nb[7:4], l_s};
		assign nci_u_co[gnc] = &nb[7:4];
	end
	endgenerate
	wire nci_c8  =  nci_zb[0];
	wire nci_c16 =  nci_zb[1] & nci_zb[0];
	wire nci_c24 =  nci_zb[2] & nci_zb[1] & nci_zb[0];
	wire nci_c32 = &nci_zb[3:0];
	wire nci_c40 = &nci_zb[4:0];
	assign neg_ci = {
		nci_c40 ? nci_s_p1[5] : ~r_ci[47:40],
		nci_c32 ? nci_s_p1[4] : ~r_ci[39:32],
		nci_c24 ? nci_s_p1[3] : ~r_ci[31:24],
		nci_c16 ? nci_s_p1[2] : ~r_ci[23:16],
		nci_c8  ? nci_s_p1[1] : ~r_ci[15:8],
		nci_s_p1[0]
	};
	assign carry_unused[21] = (&nci_u_co) & nci_zb[5];
`endif
	generate
	for (gs = 0; gs < 3; gs = gs + 1) begin : g_cabs_sel
		wire ci_sgn_s;
		assign ci_sgn_s = r_ci_sgn;
		assign cabs[16 * gs +: 16] = ci_sgn_s ? neg_ci[16 * gs +: 16] : r_ci[16 * gs +: 16];
	end
	endgenerate

	/* Hierarchical 6x8-bit CLZ for r_cabs[47:0] (shallow 2-level priority tree) */
	wire [5:0] cabs_nz;
	wire [2:0] cabs_lz [0:5];
	generate
	for (gi = 0; gi < 6; gi = gi + 1) begin : g_cabs_clz
		wire [7:0] cb = r_cabs[8 * gi +: 8];
		assign cabs_nz[gi] = |cb;
		assign cabs_lz[gi] =
			cb[7] ? 3'd0 : cb[6] ? 3'd1 : cb[5] ? 3'd2 : cb[4] ? 3'd3 :
			cb[3] ? 3'd4 : cb[2] ? 3'd5 : cb[1] ? 3'd6 : 3'd7;
	end
	endgenerate

	wire cabs_hi_nz = |cabs_nz[5:3];
	wire cabs_lo_nz = |cabs_nz[2:0];
	wire [5:0] cl_hi = cabs_nz[5] ? {3'd0, cabs_lz[5]} : (cabs_nz[4] ? {3'd1, cabs_lz[4]} : {3'd2, cabs_lz[3]});
	wire [5:0] cl_lo = cabs_nz[2] ? {3'd3, cabs_lz[2]} : (cabs_nz[1] ? {3'd4, cabs_lz[1]} : {3'd5, cabs_lz[0]});
	wire [5:0] cl = cabs_hi_nz ? cl_hi : (cabs_lo_nz ? cl_lo : 6'd48);

	/* Stage 3: fast one-hot byte+bit barrel left shift (<= 7 cmos3, fanout <= 16) */
	wire [7:0] ca_b [0:5];
	assign ca_b[0] = ca[7:0];
	assign ca_b[1] = ca[15:8];
	assign ca_b[2] = ca[23:16];
	assign ca_b[3] = ca[31:24];
	assign ca_b[4] = ca[39:32];
	assign ca_b[5] = ca[47:40];

	wire [47:0] ca_sh8;
	genvar gcb;
	generate
	for (gcb = 0; gcb < 6; gcb = gcb + 1) begin : g_cn_bytes
		wire [5:0] h_oh = (gcb < 2) ? r_clz_hi_oh_a : (gcb < 4) ? r_clz_hi_oh_b : r_clz_hi_oh_c;
		wire [7:0] b0 = ca_b[gcb];
		wire [7:0] b1, b2, b3, b4, b5;
		if (gcb >= 1) begin : g_b1 assign b1 = ca_b[gcb - 1]; end else begin : g_b1z assign b1 = 8'd0; end
		if (gcb >= 2) begin : g_b2 assign b2 = ca_b[gcb - 2]; end else begin : g_b2z assign b2 = 8'd0; end
		if (gcb >= 3) begin : g_b3 assign b3 = ca_b[gcb - 3]; end else begin : g_b3z assign b3 = 8'd0; end
		if (gcb >= 4) begin : g_b4 assign b4 = ca_b[gcb - 4]; end else begin : g_b4z assign b4 = 8'd0; end
		if (gcb >= 5) begin : g_b5 assign b5 = ca_b[gcb - 5]; end else begin : g_b5z assign b5 = 8'd0; end
		assign ca_sh8[8 * gcb +: 8] =
			({8{h_oh[0]}} & b0) | ({8{h_oh[1]}} & b1) | ({8{h_oh[2]}} & b2) |
			({8{h_oh[3]}} & b3) | ({8{h_oh[4]}} & b4) | ({8{h_oh[5]}} & b5);
	end
	endgenerate

	wire [47:0] cn_s21;
	wire [47:0] cn;
	generate
	for (gcb = 0; gcb < 48; gcb = gcb + 1) begin : g_cn_bits
		wire [3:0] l_oh = (gcb < 16) ? r_clz_lo_oh_a : (gcb < 32) ? r_clz_lo_oh_b : r_clz_lo_oh_c;
		wire b0_sel = (gcb < 16) ? r_clz0_a : (gcb < 32) ? r_clz0_b : r_clz0_c;
		wire s2, s4, sh6;
		if (gcb >= 2) begin : g_s2 assign s2 = ca_sh8[gcb - 2]; end else begin : g_s2z assign s2 = 1'b0; end
		if (gcb >= 4) begin : g_s4 assign s4 = ca_sh8[gcb - 4]; end else begin : g_s4z assign s4 = 1'b0; end
		if (gcb >= 6) begin : g_s6 assign sh6 = ca_sh8[gcb - 6]; end else begin : g_s6z assign sh6 = 1'b0; end
		assign cn_s21[gcb] = (l_oh[0] & ca_sh8[gcb]) | (l_oh[1] & s2) | (l_oh[2] & s4) | (l_oh[3] & sh6);
		if (gcb >= 1) begin : g_cn_sh1
			assign cn[gcb] = b0_sel ? cn_s21[gcb - 1] : cn_s21[gcb];
		end else begin : g_cn_sh0
			assign cn[gcb] = b0_sel ? 1'b0 : cn_s21[0];
		end
	end
	endgenerate

	/* Fast sticky directly from ca + clz (<= 7 cmos3, decoupled from barrel shifter) */
	function automatic ca_byte_part;
		input [7:0] b;
		input [2:0] s;
		begin
			case (s)
			3'd0: ca_byte_part = |b[6:0];
			3'd1: ca_byte_part = |b[5:0];
			3'd2: ca_byte_part = |b[4:0];
			3'd3: ca_byte_part = |b[3:0];
			3'd4: ca_byte_part = |b[2:0];
			3'd5: ca_byte_part = |b[1:0];
			3'd6: ca_byte_part = b[0];
			default: ca_byte_part = 1'b0;
			endcase
		end
	endfunction
	wire ca_nz_b0 = |ca[7:0];
	wire ca_nz_b1 = |ca[15:8];
	wire ca_stk_fast =
		(r_clz_hi_oh_a[0] & (ca_byte_part(ca[23:16], clz[2:0]) || ca_nz_b1 || ca_nz_b0)) ||
		(r_clz_hi_oh_a[1] & (ca_byte_part(ca[15:8],  clz[2:0]) || ca_nz_b0)) ||
		(r_clz_hi_oh_a[2] & ca_byte_part(ca[7:0],   clz[2:0]));

	wire [8:0] czs;
	am_add #(.W(9)) u_czs (.a({3'd0, clz}), .b({4'd0, csh}), .ci(1'b0), .s(czs),
		.co(carry_unused[23]));

	(* dont_touch = "true", extract_reset = "no" *) reg [47:0] r_cn;
	(* dont_touch = "true", extract_reset = "no" *) reg r_cn_guard;
	(* dont_touch = "true", extract_reset = "no" *) reg r_cn_stk0;
	(* dont_touch = "true", extract_reset = "no" *) reg r_cn_stk1;
	(* max_fanout = 16 *) reg r_cs2;
	reg [8:0] r_czs;
	(* max_fanout = 16 *) reg r_ca_zero;

	/* Stage 4: CARRY4 rounding incrementer with registered sticky/guard bits */
	wire [23:0] cm_in = r_cn[47:24];
	wire cm_inc = r_cn_guard && (r_cn_stk0 || r_cn_stk1 || r_cn[24]);
	wire [24:0] cm;
	am_add #(.W(25)) u_cm (
		.a({1'b0, cm_in}),
		.b(25'd0),
		.ci(cm_inc),
		.s(cm),
		.co(carry_unused[52])
	);

	wire [8:0] ce0, ce1;
	am_sub #(.W(9)) u_ce0 (.a(9'd174), .b(r_czs), .bi(1'b0), .d(ce0), .bo(carry_unused[24]));
	am_sub #(.W(9)) u_ce1 (.a(9'd175), .b(r_czs), .bi(1'b0), .d(ce1), .bo(carry_unused[22]));

	wire caz_lo, caz_hi, cm24_ce, cm24_hi, cm24_lo;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cazl   (.I0(r_ca_zero), .O(caz_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cazh   (.I0(r_ca_zero), .O(caz_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cm24ce (.I0(cm[24]),    .O(cm24_ce));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cm24mh (.I0(cm[24]),    .O(cm24_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cm24ml (.I0(cm[24]),    .O(cm24_lo));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign caz_lo = r_ca_zero;
	assign caz_hi = r_ca_zero;
	assign cm24_ce = cm[24];
	assign cm24_hi = cm[24];
	assign cm24_lo = cm[24];
`else
	assign caz_lo  = r_ca_zero;
	assign caz_hi  = r_ca_zero;
	assign cm24_ce = cm[24];
	assign cm24_hi = cm[24];
	assign cm24_lo = cm[24];
`endif
	wire [7:0] ce = cm24_ce ? ce1[7:0] : ce0[7:0];
	wire [22:0] cm_mant = {cm24_hi ? cm[23:13] : cm[22:12], cm24_lo ? cm[12:1] : cm[11:0]};
	wire [31:0] c_out = {caz_hi ? 16'd0 : {r_cs2, ce[7:0], cm_mant[22:16]},
	                     caz_lo ? 16'd0 : cm_mant[15:0]};

	/* steps 21-26: feed edge ce_e / 2; 25-30: their results;
	 * steps 1-10: the snapped value; the register written (40..46) */
	wire [2:0] ce_e = {2'd0, phase_oh[22] | phase_oh[24] | phase_oh[26]};
	wire [3:0] tsn = {
		phase_oh[9] | phase_oh[10],
		phase_oh[5] | phase_oh[6] | phase_oh[7] | phase_oh[8],
		phase_oh[3] | phase_oh[4] | phase_oh[7] | phase_oh[8],
		phase_oh[2] | phase_oh[4] | phase_oh[6] | phase_oh[8] | phase_oh[10]
	};
	wire [6:0] wa_n = {
		4'b0101,
		phase_oh[34] | phase_oh[35] | phase_oh[36],
		phase_oh[32] | phase_oh[33] | phase_oh[36],
		phase_oh[31] | phase_oh[33] | phase_oh[35]
	};
	wire [1:0] ej = {phase_oh[25] | phase_oh[26], phase_oh[23] | phase_oh[24]};
	wire [1:0] ek = {phase_oh[29] | phase_oh[30], phase_oh[27] | phase_oh[28]};
	assign carry_unused[2] = 1'b0;
	assign carry_unused[1] = 1'b0;
	assign carry_unused[0] = 1'b0;
	assign carry_unused[26] = 1'b0;
	assign carry_unused[27] = 1'b0;
	assign carry_unused[28] = 1'b0;
	assign carry_unused[29] = 1'b0;
	assign carry_unused[30] = 1'b0;
	/* the plane triangle's second delta; D's line form; D = dx1 dy2 - dx2 dy1 */
	wire signed [23:0] vdx2, vdy2;
	wire [47:0] ndy, dd_n;
	wire [24:0] ndy25;
	wire r_ddy_s_ndy;
	assign r_ddy_s_ndy = r_ddy[23];
`ifdef SYNTH_KINTEX7
	am_sub #(.W(24)) u_vdx2 (.a(vx2), .b(ax), .bi(1'b0), .d(vdx2), .bo(carry_unused[31]));
	am_sub #(.W(24)) u_vdy2 (.a(vy2), .b(ay), .bi(1'b0), .d(vdy2), .bo(carry_unused[32]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(24)) u_vdx2 (.a(vx2), .b(ax), .bi(1'b0), .d(vdx2), .bo(carry_unused[31]));
	am_sub #(.W(24)) u_vdy2 (.a(vy2), .b(ay), .bi(1'b0), .d(vdy2), .bo(carry_unused[32]));
`else
	wire [11:0] vdx2_lo, vdx2_hi0, vdx2_hi1, vdy2_lo, vdy2_hi0, vdy2_hi1;
	wire vdx2_bo12, vdy2_bo12, vdx2_u0, vdx2_u1, vdy2_u0, vdy2_u1;
	am_sub #(.W(12)) u_vdx2_lo  (.a(vx2[11:0]),  .b(ax[11:0]),  .bi(1'b0), .d(vdx2_lo),  .bo(vdx2_bo12));
	am_sub #(.W(12)) u_vdx2_hi0 (.a(vx2[23:12]), .b(ax[23:12]), .bi(1'b0), .d(vdx2_hi0), .bo(vdx2_u0));
	am_sub #(.W(12)) u_vdx2_hi1 (.a(vx2[23:12]), .b(ax[23:12]), .bi(1'b1), .d(vdx2_hi1), .bo(vdx2_u1));
	am_sub #(.W(12)) u_vdy2_lo  (.a(vy2[11:0]),  .b(ay[11:0]),  .bi(1'b0), .d(vdy2_lo),  .bo(vdy2_bo12));
	am_sub #(.W(12)) u_vdy2_hi0 (.a(vy2[23:12]), .b(ay[23:12]), .bi(1'b0), .d(vdy2_hi0), .bo(vdy2_u0));
	am_sub #(.W(12)) u_vdy2_hi1 (.a(vy2[23:12]), .b(ay[23:12]), .bi(1'b1), .d(vdy2_hi1), .bo(vdy2_u1));
	assign vdx2 = {vdx2_bo12 ? vdx2_hi1 : vdx2_hi0, vdx2_lo};
	assign vdy2 = {vdy2_bo12 ? vdy2_hi1 : vdy2_hi0, vdy2_lo};
	assign carry_unused[32:31] = {vdx2_u0 & vdx2_u1, vdy2_u0 & vdy2_u1};
`endif
`ifdef SYNTH_KINTEX7
	am_sub #(.W(25)) u_ndy (.a(25'd0), .b({r_ddy_s_ndy, r_ddy}), .bi(1'b0), .d(ndy25),
		.bo(carry_unused[33]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(25)) u_ndy (.a(25'd0), .b({r_ddy_s_ndy, r_ddy}), .bi(1'b0), .d(ndy25),
		.bo(carry_unused[33]));
`else
	/* Pure-LUT 25-bit negation 0 - {r_ddy_s_ndy, r_ddy} via 4->12->25 prefix-OR (<= 7 cmos3) */
	wire [24:0] ndy_in = {r_ddy_s_ndy, r_ddy};
	wire [5:0] ndy_or4 = {|ndy_in[23:20], |ndy_in[19:16], |ndy_in[15:12], |ndy_in[11:8], |ndy_in[7:4], |ndy_in[3:0]};
	wire [1:0] ndy_or12 = {|ndy_or4[5:3], |ndy_or4[2:0]};
	wire [24:0] ndy_or_below;
	assign ndy_or_below[0] = 1'b0;
	genvar gndy;
	generate
	for (gndy = 1; gndy < 24; gndy = gndy + 1) begin : g_ndy_neg
		localparam integer NG4 = gndy / 4;
		localparam integer NR4 = gndy % 4;
		wire n_lo4 = (NR4 == 0) ? 1'b0 : (NR4 == 1) ? ndy_in[4*NG4] : (NR4 == 2) ? (|ndy_in[4*NG4 +: 2]) : (|ndy_in[4*NG4 +: 3]);
		wire n_mid =
			(NG4 == 0) ? 1'b0 :
			(NG4 == 1) ? ndy_or4[0] :
			(NG4 == 2) ? (ndy_or4[1] | ndy_or4[0]) :
			(NG4 == 3) ? ndy_or12[0] :
			(NG4 == 4) ? (ndy_or12[0] | ndy_or4[3]) :
			             (ndy_or12[0] | ndy_or4[4] | ndy_or4[3]);
		assign ndy_or_below[gndy] = n_lo4 | n_mid;
	end
	endgenerate
	assign ndy_or_below[24] = ndy_or12[0] | ndy_or12[1];
	assign ndy25 = ndy_in ^ ndy_or_below;
	assign carry_unused[33] = 1'b0;
`endif
	assign ndy = {{15{ndy25[24]}}, ndy25, 8'd0};

	/* The multiplier: registered operands + 3-stage pipeline (LAT = 3), sliced muxes for fanout <= 13 */
	wire signed [24:0] ma_comb, mb_comb;
	wire [2:0] eA_s23_mp;
`ifdef SYNTH_KINTEX7
	genvar geas;
	generate
	for (geas = 0; geas < 3; geas = geas + 1) begin : g_ea_s23_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_easmp (.I0(eA[geas][23]), .O(eA_s23_mp[geas]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar geas;
	generate
	for (geas = 0; geas < 3; geas = geas + 1) begin : g_ea_s23_b
		assign eA_s23_mp[geas] = eA[geas][23];
	end
	endgenerate
`else
	assign eA_s23_mp = {eA[2][23], eA[1][23], eA[0][23]};
`endif
	wire [24:0] eA_ext [0:2];
	wire [24:0] eB_ext [0:2];
	assign eA_ext[0] = {eA_s23_mp[0], eA[0]};
	assign eA_ext[1] = {eA_s23_mp[1], eA[1]};
	assign eA_ext[2] = {eA_s23_mp[2], eA[2]};
	assign eB_ext[0] = {eB[0][23], eB[0]};
	assign eB_ext[1] = {eB[1][23], eB[1]};
	assign eB_ext[2] = {eB[2][23], eB[2]};
	wire [24:0] dx1_25 = {r_dx1[23], r_dx1};
	wire [24:0] dx2_25 = {r_dx2[23], r_dx2};
	wire [24:0] dy1_25 = {r_dy1[23], r_dy1};
	wire [24:0] dy2_25 = {r_dy2[23], r_dy2};
	generate
	for (gs = 0; gs < 4; gs = gs + 1) begin : g_mp_mux
		localparam integer LO = (gs == 0) ? 0  : (gs == 1) ? 6 : (gs == 2) ? 12 : 18;
		localparam integer W  = (gs == 3) ? 7 : 6;
		(* keep = "true" *) wire s_dx1 = phase_oh_b[14];
		(* keep = "true" *) wire s_dx2 = phase_oh_b[15];
		(* keep = "true" *) wire s_ea0 = phase_oh_b[21];
		(* keep = "true" *) wire s_eb0 = phase_oh_b[22];
		(* keep = "true" *) wire s_ea1 = phase_oh_b[23];
		(* keep = "true" *) wire s_eb1 = phase_oh_b[24];
		(* keep = "true" *) wire s_ea2 = phase_oh_b[25];
		(* keep = "true" *) wire s_eb2 = phase_oh_b[26];
		wire [W-1:0] ma_t0 = ({W{s_dx1}} & dx1_25[LO +: W])    | ({W{s_dx2}} & dx2_25[LO +: W]);
		wire [W-1:0] ma_t1 = ({W{s_ea0}} & eA_ext[0][LO +: W]) | ({W{s_ea1}} & eA_ext[1][LO +: W]);
		wire [W-1:0] ma_t2 = ({W{s_ea2}} & eA_ext[2][LO +: W]) | ({W{s_eb0}} & eB_ext[0][LO +: W]);
		wire [W-1:0] ma_t3 = ({W{s_eb1}} & eB_ext[1][LO +: W]) | ({W{s_eb2}} & eB_ext[2][LO +: W]);
		assign ma_comb[LO +: W] = (ma_t0 | ma_t1) | (ma_t2 | ma_t3);

		wire [W-1:0] mb_t0 = ({W{s_dx1}} & dy2_25[LO +: W]) | ({W{s_dx2}} & dy1_25[LO +: W]);
		wire [W-1:0] mb_t1 = ({W{s_ea0}} & oX[0][LO +: W])  | ({W{s_ea1}} & oX[1][LO +: W]);
		wire [W-1:0] mb_t2 = ({W{s_ea2}} & oX[2][LO +: W])  | ({W{s_eb0}} & oY[0][LO +: W]);
		wire [W-1:0] mb_t3 = ({W{s_eb1}} & oY[1][LO +: W])  | ({W{s_eb2}} & oY[2][LO +: W]);
		assign mb_comb[LO +: W] = (mb_t0 | mb_t1) | (mb_t2 | mb_t3);
	end
	endgenerate

	(* keep = "true", dont_touch = "true", extract_enable = "no", extract_reset = "no" *) reg signed [24:0] r_mp_a, r_mp_a_b, r_mp_a_c, r_mp_a_d, r_mp_a_e, r_mp_a_f, r_mp_b;
`ifdef SYNTH_ASIC
	(* dont_touch = "true" *) reg [11:0] r_mp_bo_p1x, r_mp_bo_p1x_b, r_mp_bo_n1x, r_mp_bo_n1x_b;
	(* dont_touch = "true" *) reg [11:0] r_mp_bo_p2x, r_mp_bo_p2x_b, r_mp_bo_n2x, r_mp_bo_n2x_b;
	reg [12:0] r_mp_bo_neg;
	reg r_mp_12_p, r_mp_12_n;
	wire [26:0] mb_pad_comb = {mb_comb[24], mb_comb, 1'b0};
`endif
	always @(posedge clk) begin
		r_mp_a   <= ma_comb;
		r_mp_a_b <= ma_comb;
		r_mp_a_c <= ma_comb;
		r_mp_a_d <= ma_comb;
		r_mp_a_e <= ma_comb;
		r_mp_a_f <= ma_comb;
		r_mp_b   <= mb_comb;
`ifdef SYNTH_ASIC
		for (k = 0; k < 12; k = k + 1) begin
			r_mp_bo_p1x[k]   <= (mb_pad_comb[2*k] ^ mb_pad_comb[2*k+1]) && !mb_pad_comb[2*k+2];
			r_mp_bo_p1x_b[k] <= (mb_pad_comb[2*k] ^ mb_pad_comb[2*k+1]) && !mb_pad_comb[2*k+2];
			r_mp_bo_n1x[k]   <= (mb_pad_comb[2*k] ^ mb_pad_comb[2*k+1]) &&  mb_pad_comb[2*k+2];
			r_mp_bo_n1x_b[k] <= (mb_pad_comb[2*k] ^ mb_pad_comb[2*k+1]) &&  mb_pad_comb[2*k+2];
			r_mp_bo_p2x[k]   <= !mb_pad_comb[2*k+2] &&  mb_pad_comb[2*k+1] &&  mb_pad_comb[2*k];
			r_mp_bo_p2x_b[k] <= !mb_pad_comb[2*k+2] &&  mb_pad_comb[2*k+1] &&  mb_pad_comb[2*k];
			r_mp_bo_n2x[k]   <=  mb_pad_comb[2*k+2] && !mb_pad_comb[2*k+1] && !mb_pad_comb[2*k];
			r_mp_bo_n2x_b[k] <=  mb_pad_comb[2*k+2] && !mb_pad_comb[2*k+1] && !mb_pad_comb[2*k];
			r_mp_bo_neg[k]   <= mb_pad_comb[2*k+2] && !(mb_pad_comb[2*k+1] && mb_pad_comb[2*k]);
		end
		r_mp_12_p <= !mb_pad_comb[26] &&  mb_pad_comb[24];
		r_mp_12_n <=  mb_pad_comb[26] && !mb_pad_comb[24];
		r_mp_bo_neg[12] <= mb_pad_comb[26] && !mb_pad_comb[24];
`endif
	end

	wire signed [49:0] mp;
`ifdef SYNTH_ASIC
	/* 3-stage Radix-4 Booth 25x25 signed multiplier with sign-folded Booth selects (<= 11 cmos3 Stage 1) */
	wire [12:0] mp_bneg = r_mp_bo_neg;
	wire [49:0] mp_pp [0:12];
	wire [25:0] mp_a1x_0 = {r_mp_a[24],   r_mp_a};
	wire [25:0] mp_a1x_1 = {r_mp_a_b[24], r_mp_a_b};
	wire [25:0] mp_a1x_2 = {r_mp_a_c[24], r_mp_a_c};
	wire [25:0] mp_a1x_3 = {r_mp_a_d[24], r_mp_a_d};
	wire [25:0] mp_a1x_4 = {r_mp_a_e[24], r_mp_a_e};
	wire [25:0] mp_a1x_5 = {r_mp_a_f[24], r_mp_a_f};
	wire [25:0] mp_a2x_0 = {r_mp_a,   1'b0};
	wire [25:0] mp_a2x_1 = {r_mp_a_b, 1'b0};
	wire [25:0] mp_a2x_2 = {r_mp_a_c, 1'b0};
	wire [25:0] mp_a2x_3 = {r_mp_a_d, 1'b0};
	wire [25:0] mp_a2x_4 = {r_mp_a_e, 1'b0};
	wire [25:0] mp_a2x_5 = {r_mp_a_f, 1'b0};
	genvar gmp;
	generate
	for (gmp = 0; gmp < 12; gmp = gmp + 1) begin : g_mp_b
		wire [25:0] cur_a1x = (gmp < 2) ? mp_a1x_0 : (gmp < 4) ? mp_a1x_1 : (gmp < 6) ? mp_a1x_2 : (gmp < 8) ? mp_a1x_3 : (gmp < 10) ? mp_a1x_4 : mp_a1x_5;
		wire [25:0] cur_a2x = (gmp < 2) ? mp_a2x_0 : (gmp < 4) ? mp_a2x_1 : (gmp < 6) ? mp_a2x_2 : (gmp < 8) ? mp_a2x_3 : (gmp < 10) ? mp_a2x_4 : mp_a2x_5;
		wire p1x_lo = r_mp_bo_p1x[gmp], p1x_hi = r_mp_bo_p1x_b[gmp];
		wire n1x_lo = r_mp_bo_n1x[gmp], n1x_hi = r_mp_bo_n1x_b[gmp];
		wire p2x_lo = r_mp_bo_p2x[gmp], p2x_hi = r_mp_bo_p2x_b[gmp];
		wire n2x_lo = r_mp_bo_n2x[gmp], n2x_hi = r_mp_bo_n2x_b[gmp];
		wire [12:0] pp_lo =
			(({13{p1x_lo}} &  cur_a1x[12:0]) | ({13{n1x_lo}} & ~cur_a1x[12:0])) |
			(({13{p2x_lo}} &  cur_a2x[12:0]) | ({13{n2x_lo}} & ~cur_a2x[12:0]));
		wire [12:0] pp_hi =
			(({13{p1x_hi}} &  cur_a1x[25:13]) | ({13{n1x_hi}} & ~cur_a1x[25:13])) |
			(({13{p2x_hi}} &  cur_a2x[25:13]) | ({13{n2x_hi}} & ~cur_a2x[25:13]));
		wire [25:0] u_j   = {pp_hi, pp_lo};
		wire s_j          = u_j[25];
		wire [49:0] row_base;
		if (gmp == 0) begin : g_mb0
			assign row_base = {22'd0, ~s_j, s_j, s_j, u_j[24:0]};
		end else if (gmp < 11) begin : g_mbm
			wire [22 - 2*gmp : 0] pad_hi = {(23 - 2*gmp){1'b0}};
			assign row_base = {pad_hi, 1'b1, ~s_j, u_j[24:0], {(2*gmp){1'b0}}};
		end else begin : g_mb11
			assign row_base = {1'b1, ~s_j, u_j[24:0], 22'd0};
		end
		if (gmp == 0) begin : g_mn0
			assign mp_pp[0] = row_base;
		end else begin : g_mnj
			wire [49:0] neg_ins = {{(50 - (2*gmp - 1)){1'b0}}, mp_bneg[gmp - 1], {(2*gmp - 2){1'b0}}};
			assign mp_pp[gmp] = row_base | neg_ins;
		end
	end
	endgenerate
	wire [25:0] mp12_u = r_mp_12_n ? ~mp_a1x_2 : (r_mp_12_p ? mp_a1x_2 : 26'd0);
	assign mp_pp[12] = {~mp12_u[25], mp12_u[24:0], 1'b0, mp_bneg[11], 22'd0};
	wire [24:0] unused_r_mp_b = r_mp_b;

	/* Stage 1 CSA: 13 -> 9 -> 6 rows with fast replicated r_mp_a */
	wire [49:0] mp_s1_m [0:8];
	generate
	for (gmp = 0; gmp < 4; gmp = gmp + 1) begin : g_mp_s1_32a
		wire [49:0] x0 = mp_pp[3*gmp];
		wire [49:0] x1 = mp_pp[3*gmp + 1];
		wire [49:0] x2 = mp_pp[3*gmp + 2];
		assign mp_s1_m[2*gmp]     = x0 ^ x1 ^ x2;
		assign mp_s1_m[2*gmp + 1] = {((x0[48:0] & x1[48:0]) | (x0[48:0] & x2[48:0]) | (x1[48:0] & x2[48:0])), 1'b0};
	end
	endgenerate
	assign mp_s1_m[8] = mp_pp[12];

	wire [49:0] mp_s1_l1 [0:5];
	generate
	for (gmp = 0; gmp < 3; gmp = gmp + 1) begin : g_mp_s1_32b
		wire [49:0] y0 = mp_s1_m[3*gmp];
		wire [49:0] y1 = mp_s1_m[3*gmp + 1];
		wire [49:0] y2 = mp_s1_m[3*gmp + 2];
		assign mp_s1_l1[2*gmp]     = y0 ^ y1 ^ y2;
		assign mp_s1_l1[2*gmp + 1] = {((y0[48:0] & y1[48:0]) | (y0[48:0] & y2[48:0]) | (y1[48:0] & y2[48:0])), 1'b0};
	end
	endgenerate

	reg [49:0] r_mp_csa1 [0:5];
	integer imp;
	always @(posedge clk) begin
		for (imp = 0; imp < 6; imp = imp + 1)
			r_mp_csa1[imp] <= mp_s1_l1[imp];
	end

	/* Stage 2 CSA: 6 -> 4 (2x 3:2) -> 2 rows (1x 4:2) (<= 10 cmos3 total) */
	wire [49:0] mp_s2_l2_0 = r_mp_csa1[0] ^ r_mp_csa1[1] ^ r_mp_csa1[2];
	wire [49:0] mp_s2_l2_1 = {((r_mp_csa1[0][48:0] & r_mp_csa1[1][48:0]) | (r_mp_csa1[0][48:0] & r_mp_csa1[2][48:0]) | (r_mp_csa1[1][48:0] & r_mp_csa1[2][48:0])), 1'b0};
	wire [49:0] mp_s2_l2_2 = r_mp_csa1[3] ^ r_mp_csa1[4] ^ r_mp_csa1[5];
	wire [49:0] mp_s2_l2_3 = {((r_mp_csa1[3][48:0] & r_mp_csa1[4][48:0]) | (r_mp_csa1[3][48:0] & r_mp_csa1[5][48:0]) | (r_mp_csa1[4][48:0] & r_mp_csa1[5][48:0])), 1'b0};

	wire [49:0] mp42_p0 = mp_s2_l2_0 ^ mp_s2_l2_1;
	wire [49:0] mp42_p1 = mp_s2_l2_2 ^ mp_s2_l2_3;
	wire [49:0] mp42_cout = {((mp42_p0[48:0] & mp_s2_l2_2[48:0]) | (~mp42_p0[48:0] & mp_s2_l2_0[48:0])), 1'b0};
	wire [48:0] mp42_x  = mp42_p0[48:0] ^ mp42_p1[48:0];
	wire [49:0] mp_s2_l3_s = mp42_p0 ^ mp42_p1 ^ mp42_cout;
	wire [49:0] mp_s2_l3_c = {((mp42_x & mp42_cout[48:0]) | (~mp42_x & mp_s2_l2_3[48:0])), 1'b0};

	reg [49:0] r_mp_s2, r_mp_c2;
	always @(posedge clk) begin
		r_mp_s2 <= mp_s2_l3_s;
		r_mp_c2 <= mp_s2_l3_c;
	end

	/* Stage 3: 50-bit Kogge-Stone am_add with ci = mp_bneg[12] */
	wire [49:0] mp_sum_w;
	wire mp_co_unused;
	am_add #(.W(50)) u_mp_fin (.a(r_mp_s2), .b(r_mp_c2), .ci(mp_bneg[12]), .s(mp_sum_w), .co(mp_co_unused));
	reg [49:0] r_mp_p3;
	always @(posedge clk)
		r_mp_p3 <= mp_sum_w;
	assign mp = r_mp_p3;
`else
	am_mul #(.AW(25), .BW(25), .SIGNED(1), .LAT(3)) u_mp (.clk(clk), .en(1'b1), .a(r_mp_a), .b(r_mp_b), .p(mp));
`endif

	am_sub #(.W(48)) u_dd_n (.a(r_dx1_dy2), .b(mp[47:0]), .bi(1'b0), .d(dd_n), .bo(carry_unused[34]));
	/* the edges of the ordered triangle: A = y1 - y0, B = x0 - x1, oX = sx - x0, oY = sy - y0 */
	wire [23:0] eA_n [0:2];
	wire [23:0] eB_n [0:2];
	wire [24:0] oX_n [0:2];
	wire [24:0] oY_n [0:2];
	wire [47:0] eE_n;
	(* keep = "true", dont_touch = "true" *) reg [11:0] r_dd_ccw;
	wire [2:0] ccw_e_a = r_dd_ccw[2:0];
	wire [2:0] ccw_e_b = r_dd_ccw[5:3];
	wire [2:0] ccw_e_c = r_dd_ccw[8:6];

	/* Zero-XOR 6x4-bit borrow lookahead with speculative 4-bit sum (<= 9 cmos3 vs 13) */
	function [24:0] edge_sub25;
		input [24:0] a, b;
		reg [24:0] gb, pb, px;
		reg [11:0] gn2, pn2;
		reg [5:0] g4, p4;
		reg gn8_0, pn8_0, gn8_1, pn8_1, g8_2, p8_2;
		reg [6:0] c4;
		integer m, kb;
		reg [3:0] g4b, p4b, x4b, s0, s1;
		reg c0_1, c0_2, c0_3, c1_1, c1_2, c1_3;
		begin
			gb = (~a) & b;
			pb = (~a) | b;
			px = a ^ b;
			for (kb = 0; kb < 12; kb = kb + 1) begin
				gn2[kb] = !(gb[2*kb+1] | (pb[2*kb+1] & gb[2*kb]));
				pn2[kb] = !(pb[2*kb+1] & pb[2*kb]);
			end
			for (kb = 0; kb < 6; kb = kb + 1) begin
				g4[kb] = !(gn2[2*kb+1] & (pn2[2*kb+1] | gn2[2*kb]));
				p4[kb] = !(pn2[2*kb+1] | pn2[2*kb]);
			end
			c4[0] = 1'b0;
			c4[1] = g4[0];
			gn8_0 = !(g4[1] | (p4[1] & g4[0]));
			pn8_0 = !(p4[1] & p4[0]);
			c4[2] = !gn8_0;
			c4[3] = g4[2] | (p4[2] & g4[1]) | (p4[2] & p4[1] & g4[0]);
			gn8_1 = !(g4[3] | (p4[3] & g4[2]));
			pn8_1 = !(p4[3] & p4[2]);
			c4[4] = !(gn8_1 & (pn8_1 | gn8_0));
			c4[5] = g4[4] | (p4[4] & c4[4]);
			g8_2  = g4[5] | (p4[5] & g4[4]);
			p8_2  = p4[5] & p4[4];
			c4[6] = g8_2 | (p8_2 & c4[4]);
			for (m = 0; m < 6; m = m + 1) begin
				g4b = gb[4*m +: 4];
				p4b = pb[4*m +: 4];
				x4b = px[4*m +: 4];
				c0_1 = g4b[0];
				c0_2 = g4b[1] | (p4b[1] & g4b[0]);
				c0_3 = g4b[2] | (p4b[2] & c0_2);
				s0 = x4b ^ {c0_3, c0_2, c0_1, 1'b0};
				c1_1 = p4b[0];
				c1_2 = g4b[1] | (p4b[1] & p4b[0]);
				c1_3 = g4b[2] | (p4b[2] & c1_2);
				s1 = x4b ^ {c1_3, c1_2, c1_1, 1'b1};
				edge_sub25[4*m +: 4] = c4[m] ? s1 : s0;
			end
			edge_sub25[24] = px[24] ^ c4[6];
		end
	endfunction

	genvar ge;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_e
		localparam [1:0] K0 = ge, K1 = ge == 2 ? 0 : ge + 1;
		wire [23:0] fy_v0, fy_v0_oy, fy_v1, fx_v0, fx_v0_ox, fx_v1;
		for (gs = 0; gs < 2; gs = gs + 1) begin : g_vtx_s
			localparam integer HI = 12 * gs;
`ifdef SYNTH_KINTEX7
			wire cew_ax, cew_ay, cew_bx, cew_by, cew_c, cew_d;
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cewax (.I0(ccw_e_a[ge]),     .O(cew_ax));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ceway (.I0(ccw_e_a[ge]),     .O(cew_ay));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cewbx (.I0(ccw_e_b[ge]),     .O(cew_bx));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cewby (.I0(ccw_e_b[ge]),     .O(cew_by));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cewc  (.I0(ccw_e_c[ge]),     .O(cew_c));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_cewd  (.I0(r_dd_ccw[9 + ge]),.O(cew_d));
			wire [1:0] v0_a    = vtx(K0, cew_ay);
			wire [1:0] v0_a_oy = vtx(K0, cew_by);
			wire [1:0] v1_a    = vtx(K1, cew_c);
			wire [1:0] v0_b    = vtx(K0, cew_ax);
			wire [1:0] v0_b_ox = vtx(K0, cew_bx);
			wire [1:0] v1_b    = vtx(K1, cew_d);
`elsif SYNTH_KINTEX_ULTRASCALE
			wire cew_ax, cew_ay, cew_bx, cew_by, cew_c, cew_d;
			assign cew_ax = ccw_e_a[ge];
			assign cew_ay = ccw_e_a[ge];
			assign cew_bx = ccw_e_b[ge];
			assign cew_by = ccw_e_b[ge];
			assign cew_c = ccw_e_c[ge];
			assign cew_d = r_dd_ccw[9 + ge];
			wire [1:0] v0_a    = vtx(K0, cew_ay);
			wire [1:0] v0_a_oy = vtx(K0, cew_by);
			wire [1:0] v1_a    = vtx(K1, cew_c);
			wire [1:0] v0_b    = vtx(K0, cew_ax);
			wire [1:0] v0_b_ox = vtx(K0, cew_bx);
			wire [1:0] v1_b    = vtx(K1, cew_d);
`else
			wire [1:0] v0_a    = vtx(K0, ccw_e_a[ge]);
			wire [1:0] v0_a_oy = vtx(K0, ccw_e_b[ge]);
			wire [1:0] v1_a    = vtx(K1, ccw_e_c[ge]);
			wire [1:0] v0_b    = vtx(K0, ccw_e_a[ge]);
			wire [1:0] v0_b_ox = vtx(K0, ccw_e_b[ge]);
			wire [1:0] v1_b    = vtx(K1, r_dd_ccw[9 + ge]);
`endif
			assign fy_v0[HI +: 12]    = fy[v0_a][HI +: 12];
			assign fy_v0_oy[HI +: 12] = fy[v0_a_oy][HI +: 12];
			assign fy_v1[HI +: 12]    = fy[v1_a][HI +: 12];
			assign fx_v0[HI +: 12]    = fx[v0_b][HI +: 12];
			assign fx_v0_ox[HI +: 12] = fx[v0_b_ox][HI +: 12];
			assign fx_v1[HI +: 12]    = fx[v1_b][HI +: 12];
		end
`ifdef SYNTH_KINTEX7
		am_sub #(.W(24)) u_eA (.a(fy_v1), .b(fy_v0), .bi(1'b0), .d(eA_n[ge]),
			.bo(carry_unused[35 + 4 * ge]));
		am_sub #(.W(24)) u_eB (.a(fx_v0), .b(fx_v1), .bi(1'b0), .d(eB_n[ge]),
			.bo(carry_unused[36 + 4 * ge]));
		am_sub #(.W(25)) u_oX (.a(sx), .b({fx_v0_ox[23], fx_v0_ox}), .bi(1'b0), .d(oX_n[ge]),
			.bo(carry_unused[37 + 4 * ge]));
		am_sub #(.W(25)) u_oY (.a(sy), .b({fy_v0_oy[23], fy_v0_oy}), .bi(1'b0), .d(oY_n[ge]),
			.bo(carry_unused[38 + 4 * ge]));
`elsif SYNTH_KINTEX_ULTRASCALE
		am_sub #(.W(24)) u_eA (.a(fy_v1), .b(fy_v0), .bi(1'b0), .d(eA_n[ge]),
			.bo(carry_unused[35 + 4 * ge]));
		am_sub #(.W(24)) u_eB (.a(fx_v0), .b(fx_v1), .bi(1'b0), .d(eB_n[ge]),
			.bo(carry_unused[36 + 4 * ge]));
		am_sub #(.W(25)) u_oX (.a(sx), .b({fx_v0_ox[23], fx_v0_ox}), .bi(1'b0), .d(oX_n[ge]),
			.bo(carry_unused[37 + 4 * ge]));
		am_sub #(.W(25)) u_oY (.a(sy), .b({fy_v0_oy[23], fy_v0_oy}), .bi(1'b0), .d(oY_n[ge]),
			.bo(carry_unused[38 + 4 * ge]));
`else
		wire [24:0] ea_sub25 = edge_sub25({1'b0, fy_v1}, {1'b0, fy_v0});
		wire [24:0] eb_sub25 = edge_sub25({1'b0, fx_v0}, {1'b0, fx_v1});
		assign eA_n[ge] = ea_sub25[23:0];
		assign eB_n[ge] = eb_sub25[23:0];
		assign oX_n[ge] = edge_sub25(sx, {fx_v0_ox[23], fx_v0_ox});
		assign oY_n[ge] = edge_sub25(sy, {fy_v0_oy[23], fy_v0_oy});
		assign carry_unused[35 + 4 * ge +: 4] = {2'b00, eb_sub25[24], ea_sub25[24]};
`endif
	end
	endgenerate
	/* E_k = (A_k oX_k - !tl_k) + B_k oY_k */
	wire [47:0] acc_n;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(48)) u_acc_n (.a(mp[47:0]), .b(48'd0), .bi(!tl[ek]), .d(acc_n), .bo(carry_unused[48]));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(48)) u_acc_n (.a(mp[47:0]), .b(48'd0), .bi(!tl[ek]), .d(acc_n), .bo(carry_unused[48]));
`else
	/* 6x 8-bit pure-LUT conditional decrement (- !tl[ek], <= 6 cmos3) */
	wire [5:0] acc_mz;
	wire [47:0] acc_mp_m1;
	wire acc_dec_en = !tl[ek];
	genvar ga_b, ga_i;
	for (ga_b = 0; ga_b < 6; ga_b = ga_b + 1) begin : g_acc_dec
		wire [7:0] ab = mp[8*ga_b +: 8];
		assign acc_mz[ga_b] = (ab == 8'd0);
		assign acc_mp_m1[8*ga_b + 0] = ~ab[0];
		for (ga_i = 1; ga_i < 4; ga_i = ga_i + 1) begin : g_m1_lo
			assign acc_mp_m1[8*ga_b + ga_i] = ab[ga_i] ^ !(|ab[ga_i-1:0]);
		end
		wire ab_or4 = |ab[3:0];
		assign acc_mp_m1[8*ga_b + 4] = ab[4] ^ !ab_or4;
		for (ga_i = 5; ga_i < 8; ga_i = ga_i + 1) begin : g_m1_hi
			assign acc_mp_m1[8*ga_b + ga_i] = ab[ga_i] ^ !(ab_or4 | (|ab[ga_i-1:4]));
		end
		wire ab_bor;
		if (ga_b == 0) begin : g_bor0
			assign ab_bor = acc_dec_en;
		end else begin : g_bork
			assign ab_bor = acc_dec_en & (&acc_mz[ga_b-1:0]);
		end
		assign acc_n[8*ga_b +: 8] = ab_bor ? acc_mp_m1[8*ga_b +: 8] : ab;
	end
	assign carry_unused[48] = 1'b0;
`endif
	am_add #(.W(48)) u_eE_n (.a(acc[47:0]), .b(mp[47:0]), .ci(1'b0), .s(eE_n),
		.co(carry_unused[53]));

	/* Sliced datapath register updates to keep every write-enable and mux select fanout <= 16 */
	wire c_act;
`ifdef SYNTH_KINTEX7
	wire start_cact, empty_eab, empty_bx;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stca (.I0(start), .O(start_cact));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_empe (.I0(empty), .O(empty_eab));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_empb (.I0(empty), .O(empty_bx));
	assign c_act = !start_cact && run;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire start_cact, empty_eab, empty_bx;
	assign start_cact = start;
	assign empty_eab = empty;
	assign empty_bx = empty;
	assign c_act = !start_cact && run;
`else
	wire empty_eab = empty;
	wire empty_bx  = empty;
	assign c_act = !start && run;
`endif

	/* cnt == 2..11: r_snap_shifted (3 x 16-bit) */
	wire we_snap_raw = c_act && r_snap_v1;
	generate
	for (gs = 0; gs < 3; gs = gs + 1) begin : g_snap_reg
		wire we_s;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wesn (.I0(we_snap_raw), .O(we_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign we_s = we_snap_raw;
`else
		assign we_s = we_snap_raw;
`endif
		always @(posedge clk) begin
			if (we_s) r_snap_shifted[16 * gs +: 16] <= snap_shifted[16 * gs +: 16];
		end
	end
	endgenerate

	/* cnt == 1, 2: 2-stage pipelined scissor/viewport calculation */
	wire we_c1_raw   = c_act && phase_oh[1];
	wire we_c2_raw   = c_act && phase_oh[2];
	reg [31:0] r_smin, r_smax, r_rts;
	reg r_sc_minx, r_sc_maxx, r_sc_miny, r_sc_maxy;
	wire [3:0] we_c2_s;
`ifdef SYNTH_KINTEX7
	genvar gc2;
	generate
	for (gc2 = 0; gc2 < 4; gc2 = gc2 + 1) begin : g_wc2_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc2 (.I0(we_c2_raw), .O(we_c2_s[gc2]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gc2;
	generate
	for (gc2 = 0; gc2 < 4; gc2 = gc2 + 1) begin : g_wc2_b
		assign we_c2_s[gc2] = we_c2_raw;
	end
	endgenerate
`else
	assign we_c2_s = {4{we_c2_raw}};
`endif
	always @(posedge clk) begin
		if (we_c1_raw) begin
			r_sc_minx <= r_scis_en && ($signed({2'b00, r_smin[15:0]})  > 18'sd0);
			r_sc_maxx <= r_scis_en && ($signed({2'b00, r_smax[15:0]})  < $signed({2'b00, r_rts[15:0]}));
			r_sc_miny <= r_scis_en && ($signed({2'b00, r_smin[31:16]}) > 18'sd0);
			r_sc_maxy <= r_scis_en && ($signed({2'b00, r_smax[31:16]}) < $signed({2'b00, r_rts[31:16]}));
		end
		if (we_c2_s[0]) r_scis_rt_min_x <= r_sc_minx ? $signed({2'b00, r_smin[15:0]})  : 18'sd0;
		if (we_c2_s[1]) r_scis_rt_max_x <= r_sc_maxx ? $signed({2'b00, r_smax[15:0]})  : $signed({2'b00, r_rts[15:0]});
		if (we_c2_s[2]) r_scis_rt_min_y <= r_sc_miny ? $signed({2'b00, r_smin[31:16]}) : 18'sd0;
		if (we_c2_s[3]) r_scis_rt_max_y <= r_sc_maxy ? $signed({2'b00, r_smax[31:16]}) : $signed({2'b00, r_rts[31:16]});
	end

	/* cnt == 8, 9: r_ddx, r_ddy, r_adx, r_ady (2 x 12-bit slices) */
	wire we_c6_raw = c_act && phase_oh[8];
	wire we_c7_raw = c_act && phase_oh[9];
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_c67_reg
		localparam integer HI = 12 * gs;
		wire wc6_x, wc6_y, wc7_x, wc7_y;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc6x (.I0(we_c6_raw), .O(wc6_x));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc6y (.I0(we_c6_raw), .O(wc6_y));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc7x (.I0(we_c7_raw), .O(wc7_x));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc7y (.I0(we_c7_raw), .O(wc7_y));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign wc6_x = we_c6_raw;
		assign wc6_y = we_c6_raw;
		assign wc7_x = we_c7_raw;
		assign wc7_y = we_c7_raw;
`else
		assign wc6_x = we_c6_raw;
		assign wc6_y = we_c6_raw;
		assign wc7_x = we_c7_raw;
		assign wc7_y = we_c7_raw;
`endif
		always @(posedge clk) begin
			if (wc6_x) r_ddx[HI +: 12] <= ddx[HI +: 12];
			if (wc6_y) r_ddy[HI +: 12] <= ddy[HI +: 12];
			if (wc7_x) r_adx[HI +: 12] <= adx[HI +: 12];
			if (wc7_y) r_ady[HI +: 12] <= ady[HI +: 12];
		end
	end
	endgenerate

	/* cnt == 12: r_min01_x..r_max23_y (2 x 9-bit slices) using Kogge-Stone am_sub comparators */
	wire we_c10_raw = c_act && phase_oh[12];
	wire signed [17:0] p_q0x = pix(q0x[23:8]), p_q1x = pix(q1x[23:8]), p_q2x = pix(q2x[23:8]), p_q3x = pix(q3x[23:8]);
	wire signed [17:0] p_q0y = pix(q0y[23:8]), p_q1y = pix(q1y[23:8]), p_q2y = pix(q2y[23:8]), p_q3y = pix(q3y[23:8]);
	wire mn01x_bo, mx01x_bo, mn23x_bo, mx23x_bo, mn01y_bo, mx01y_bo, mn23y_bo, mx23y_bo;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(18)) u_c_mn01x (.a(p_q0x), .b(p_q1x), .bi(1'b0), .d(cmp_d_unused[144 +: 18]), .bo(mn01x_bo));
	am_sub #(.W(18)) u_c_mx01x (.a(p_q1x), .b(p_q0x), .bi(1'b0), .d(cmp_d_unused[162 +: 18]), .bo(mx01x_bo));
	am_sub #(.W(18)) u_c_mn23x (.a(p_q2x), .b(p_q3x), .bi(1'b0), .d(cmp_d_unused[180 +: 18]), .bo(mn23x_bo));
	am_sub #(.W(18)) u_c_mx23x (.a(p_q3x), .b(p_q2x), .bi(1'b0), .d(cmp_d_unused[198 +: 18]), .bo(mx23x_bo));
	am_sub #(.W(18)) u_c_mn01y (.a(p_q0y), .b(p_q1y), .bi(1'b0), .d(cmp_d_unused[216 +: 18]), .bo(mn01y_bo));
	am_sub #(.W(18)) u_c_mx01y (.a(p_q1y), .b(p_q0y), .bi(1'b0), .d(cmp_d_unused[234 +: 18]), .bo(mx01y_bo));
	am_sub #(.W(18)) u_c_mn23y (.a(p_q2y), .b(p_q3y), .bi(1'b0), .d(), .bo(mn23y_bo));
	am_sub #(.W(18)) u_c_mx23y (.a(p_q3y), .b(p_q2y), .bi(1'b0), .d(), .bo(mx23y_bo));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(18)) u_c_mn01x (.a(p_q0x), .b(p_q1x), .bi(1'b0), .d(cmp_d_unused[144 +: 18]), .bo(mn01x_bo));
	am_sub #(.W(18)) u_c_mx01x (.a(p_q1x), .b(p_q0x), .bi(1'b0), .d(cmp_d_unused[162 +: 18]), .bo(mx01x_bo));
	am_sub #(.W(18)) u_c_mn23x (.a(p_q2x), .b(p_q3x), .bi(1'b0), .d(cmp_d_unused[180 +: 18]), .bo(mn23x_bo));
	am_sub #(.W(18)) u_c_mx23x (.a(p_q3x), .b(p_q2x), .bi(1'b0), .d(cmp_d_unused[198 +: 18]), .bo(mx23x_bo));
	am_sub #(.W(18)) u_c_mn01y (.a(p_q0y), .b(p_q1y), .bi(1'b0), .d(cmp_d_unused[216 +: 18]), .bo(mn01y_bo));
	am_sub #(.W(18)) u_c_mx01y (.a(p_q1y), .b(p_q0y), .bi(1'b0), .d(cmp_d_unused[234 +: 18]), .bo(mx01y_bo));
	am_sub #(.W(18)) u_c_mn23y (.a(p_q2y), .b(p_q3y), .bi(1'b0), .d(), .bo(mn23y_bo));
	am_sub #(.W(18)) u_c_mx23y (.a(p_q3y), .b(p_q2y), .bi(1'b0), .d(), .bo(mx23y_bo));
`else
	assign mn01x_bo = lt17_u(p_q0x[16:0], p_q1x[16:0]);
	assign mx01x_bo = lt17_u(p_q1x[16:0], p_q0x[16:0]);
	assign mn23x_bo = lt17_u(p_q2x[16:0], p_q3x[16:0]);
	assign mx23x_bo = lt17_u(p_q3x[16:0], p_q2x[16:0]);
	assign mn01y_bo = lt17_u(p_q0y[16:0], p_q1y[16:0]);
	assign mx01y_bo = lt17_u(p_q1y[16:0], p_q0y[16:0]);
	assign mn23y_bo = lt17_u(p_q2y[16:0], p_q3y[16:0]);
	assign mx23y_bo = lt17_u(p_q3y[16:0], p_q2y[16:0]);
	assign cmp_d_unused[251:144] = 108'd0;
`endif
	(* keep = "true" *) wire cmp_min01_x = (p_q0x[17] != p_q1x[17]) ? p_q0x[17] : mn01x_bo;
	(* keep = "true" *) wire cmp_max01_x = (p_q1x[17] != p_q0x[17]) ? p_q1x[17] : mx01x_bo;
	(* keep = "true" *) wire cmp_min23_x = (p_q2x[17] != p_q3x[17]) ? p_q2x[17] : mn23x_bo;
	(* keep = "true" *) wire cmp_max23_x = (p_q3x[17] != p_q2x[17]) ? p_q3x[17] : mx23x_bo;
	(* keep = "true" *) wire cmp_min01_y = (p_q0y[17] != p_q1y[17]) ? p_q0y[17] : mn01y_bo;
	(* keep = "true" *) wire cmp_max01_y = (p_q1y[17] != p_q0y[17]) ? p_q1y[17] : mx01y_bo;
	(* keep = "true" *) wire cmp_min23_y = (p_q2y[17] != p_q3y[17]) ? p_q2y[17] : mn23y_bo;
	(* keep = "true" *) wire cmp_max23_y = (p_q3y[17] != p_q2y[17]) ? p_q3y[17] : mx23y_bo;
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_c10_reg
		localparam integer HI = 9 * gs;
		wire wc10_m;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc10m (.I0(we_c10_raw), .O(wc10_m));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign wc10_m = we_c10_raw;
`else
		assign wc10_m = we_c10_raw;
`endif
		wire [7:0] wc10;
		wire s_mn01x, s_mx01x, s_mn23x, s_mx23x, s_mn01y, s_mx01y, s_mn23y, s_mx23y;
		for (gi = 0; gi < 8; gi = gi + 1) begin : g_wc10_b
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc10 (.I0(wc10_m), .O(wc10[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wc10[gi] = wc10_m;
`else
			assign wc10[gi] = wc10_m;
`endif
		end
		assign s_mn01x = cmp_min01_x;
		assign s_mx01x = cmp_max01_x;
		assign s_mn23x = cmp_min23_x;
		assign s_mx23x = cmp_max23_x;
		assign s_mn01y = cmp_min01_y;
		assign s_mx01y = cmp_max01_y;
		assign s_mn23y = cmp_min23_y;
		assign s_mx23y = cmp_max23_y;
		always @(posedge clk) begin
			if (wc10[0]) r_min01_x[HI +: 9] <= s_mn01x ? p_q0x[HI +: 9] : p_q1x[HI +: 9];
			if (wc10[1]) r_max01_x[HI +: 9] <= s_mx01x ? p_q0x[HI +: 9] : p_q1x[HI +: 9];
			if (wc10[2]) r_min23_x[HI +: 9] <= s_mn23x ? p_q2x[HI +: 9] : p_q3x[HI +: 9];
			if (wc10[3]) r_max23_x[HI +: 9] <= s_mx23x ? p_q2x[HI +: 9] : p_q3x[HI +: 9];
			if (wc10[4]) r_min01_y[HI +: 9] <= s_mn01y ? p_q0y[HI +: 9] : p_q1y[HI +: 9];
			if (wc10[5]) r_max01_y[HI +: 9] <= s_mx01y ? p_q0y[HI +: 9] : p_q1y[HI +: 9];
			if (wc10[6]) r_min23_y[HI +: 9] <= s_mn23y ? p_q2y[HI +: 9] : p_q3y[HI +: 9];
			if (wc10[7]) r_max23_y[HI +: 9] <= s_mx23y ? p_q2y[HI +: 9] : p_q3y[HI +: 9];
		end
	end
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_c10_fxy
		localparam integer HI = 12 * gs;
		wire wc10_f_mid;
		wire [5:0] wc10_f;
		wire [1:0] w1x_b, w1y_b, w2x_b, w2y_b;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc10fm (.I0(we_c10_raw), .O(wc10_f_mid));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w1x0 (.I0(w1[0]), .O(w1x_b[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w1x1 (.I0(w1[1]), .O(w1x_b[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w1y0 (.I0(w1[0]), .O(w1y_b[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w1y1 (.I0(w1[1]), .O(w1y_b[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w2x0 (.I0(w2[0]), .O(w2x_b[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w2x1 (.I0(w2[1]), .O(w2x_b[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w2y0 (.I0(w2[0]), .O(w2y_b[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w2y1 (.I0(w2[1]), .O(w2y_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign wc10_f_mid = we_c10_raw;
		assign w1x_b[0] = w1[0];
		assign w1x_b[1] = w1[1];
		assign w1y_b[0] = w1[0];
		assign w1y_b[1] = w1[1];
		assign w2x_b[0] = w2[0];
		assign w2x_b[1] = w2[1];
		assign w2y_b[0] = w2[0];
		assign w2y_b[1] = w2[1];
`else
		assign wc10_f_mid = we_c10_raw;
		assign w1x_b = w1; assign w1y_b = w1;
		assign w2x_b = w2; assign w2y_b = w2;
`endif
		for (gi = 0; gi < 6; gi = gi + 1) begin : g_wc10_fb
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc10f (.I0(wc10_f_mid), .O(wc10_f[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wc10_f[gi] = wc10_f_mid;
`else
			assign wc10_f[gi] = wc10_f_mid;
`endif
		end
		always @(posedge clk) begin
			if (wc10_f[0]) fx_pk[0  + HI +: 12] <= r_c0x[HI +: 12];
			if (wc10_f[1]) fy_pk[0  + HI +: 12] <= r_c0y[HI +: 12];
			if (wc10_f[2]) fx_pk[24 + HI +: 12] <= w1x_b[1] ? (w1x_b[0] ? r_c3x[HI +: 12] : r_c2x[HI +: 12]) : r_c1x[HI +: 12];
			if (wc10_f[3]) fy_pk[24 + HI +: 12] <= w1y_b[1] ? (w1y_b[0] ? r_c3y[HI +: 12] : r_c2y[HI +: 12]) : r_c1y[HI +: 12];
			if (wc10_f[4]) fx_pk[48 + HI +: 12] <= w2x_b[1] ? (w2x_b[0] ? r_c3x[HI +: 12] : r_c2x[HI +: 12]) : r_c1x[HI +: 12];
			if (wc10_f[5]) fy_pk[48 + HI +: 12] <= w2y_b[1] ? (w2y_b[0] ? r_c3y[HI +: 12] : r_c2y[HI +: 12]) : r_c1y[HI +: 12];
		end
	end
	endgenerate

	/* cnt == 13: pdx1..p0y, r_dx1..r_dy2, pdd, r_lx..r_my */
	wire we_c11_raw, we_c11_win;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_wc11r (.I0(c_act), .I1(phase_oh[13]), .O(we_c11_raw));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc11w (.I0(we_c11_raw), .O(we_c11_win));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_wc11r (.I0(c_act), .I1(phase_oh[13]), .O(we_c11_raw));
	assign we_c11_win = we_c11_raw;
`else
	assign we_c11_raw = c_act && phase_oh[13];
	assign we_c11_win = we_c11_raw;
`endif
	wire [2:0] c11_mid;
	generate
	for (gi = 0; gi < 3; gi = gi + 1) begin : g_c11_mid
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_c11m (.I0(we_c11_raw), .O(c11_mid[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign c11_mid[gi] = we_c11_raw;
`else
		assign c11_mid[gi] = we_c11_raw;
`endif
	end
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_c11_fxy
		localparam integer HI = 12 * gs;
		wire [11:0] wc11;
		for (gi = 0; gi < 12; gi = gi + 1) begin : g_wc11_b
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc11 (.I0(c11_mid[gs]), .O(wc11[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wc11[gi] = c11_mid[gs];
`else
			assign wc11[gi] = c11_mid[gs];
`endif
		end
		wire tl_sx, tl_sy, tr_x, tr_y, lnx_s, lny_s;
		assign tl_sx = r_is_tri_c[12 + 4 * gs] || r_is_line_c[11 + 4 * gs];
		assign tl_sy = r_is_tri_c[13 + 4 * gs] || r_is_line_c[12 + 4 * gs];
		assign tr_x  = r_is_tri_c[10 + 4 * gs];
		assign tr_y  = r_is_tri_c[11 + 4 * gs];
		assign lnx_s = r_is_line_c[9  + 4 * gs] && !r_xmaj;
		assign lny_s = r_is_line_c[10 + 4 * gs] && !r_xmaj;
		wire [23:0] c256 = 24'sd256;
		wire [11:0] pdx2_s = tr_x ? vdx2[HI +: 12] : (lnx_s ? c256[HI +: 12] : 12'd0);
		wire [11:0] pdy2_s = tr_y ? vdy2[HI +: 12] : (lny_s ? 12'd0          : c256[HI +: 12]);
		always @(posedge clk) begin
			if (wc11[0])  r_dx1[HI +: 12] <= dx1[HI +: 12];
			if (wc11[1])  r_dy1[HI +: 12] <= dy1[HI +: 12];
			if (wc11[2])  r_dx2[HI +: 12] <= dx2[HI +: 12];
			if (wc11[3])  r_dy2[HI +: 12] <= dy2[HI +: 12];
			if (wc11[6])  pdx1[HI +: 12] <= tl_sx ? r_ddx[HI +: 12] : c256[HI +: 12];
			if (wc11[7])  pdy1[HI +: 12] <= tl_sy ? r_ddy[HI +: 12] : 12'd0;
			if (wc11[8])  pdx2[HI +: 12] <= pdx2_s;
			if (wc11[9])  pdy2[HI +: 12] <= pdy2_s;
			if (wc11[10]) p0x[HI +: 12]  <= ax[HI +: 12];
			if (wc11[11]) p0y[HI +: 12]  <= ay[HI +: 12];
		end
	end
	endgenerate

	/* pdd (3 x 16-bit slices, written at cnt == 11 or cnt == 15 && is_tri) */
	wire ddx_s_pdd;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ddxpdd (.I0(r_ddx[23]), .O(ddx_s_pdd));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ddx_s_pdd = r_ddx[23];
`else
	assign ddx_s_pdd = r_ddx[23];
`endif
	wire [47:0] ddx_pdd_ext = {{16{ddx_s_pdd}}, r_ddx, 8'd0};
	wire [47:0] pdd_c65536  = 48'sd65536;
	wire we_pdd_root;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_wpdd_rt (.I0(c_act), .I1(phase_oh[13]), .O(we_pdd_root));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_wpdd_rt (.I0(c_act), .I1(phase_oh[13]), .O(we_pdd_root));
`else
	assign we_pdd_root = c_act && phase_oh[13];
`endif
	generate
	for (gs = 0; gs < 3; gs = gs + 1) begin : g_pdd_reg
		localparam integer HI = 16 * gs;
		wire we_pdd_s, is_ln_s, xmaj_s;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wpdd (.I0(we_pdd_root), .O(we_pdd_s));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ilns (.I0(is_line),  .O(is_ln_s));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_xmjs (.I0(r_xmaj),   .O(xmaj_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign we_pdd_s = we_pdd_root;
		assign is_ln_s = is_line;
		assign xmaj_s = r_xmaj;
`else
		assign we_pdd_s = we_pdd_root;
		assign is_ln_s  = is_line;
		assign xmaj_s   = r_xmaj;
`endif
		(* keep = "true" *) wire [15:0] pdd_11_s = !is_ln_s ? pdd_c65536[HI +: 16] :
			(xmaj_s ? ddx_pdd_ext[HI +: 16] : ndy[HI +: 16]);
		always @(posedge clk) begin
			if (we_pdd_s)
				pdd[HI +: 16] <= pdd_11_s;
		end
	end
	endgenerate

	/* cnt == 13: r_lx, r_ly, r_mx, r_my (2 x 9-bit slices) */
`ifdef SYNTH_KINTEX7
	wire lx_bo, ly_bo, mx_bo, my_bo;
	am_sub #(.W(18)) u_c_lx (.a(r_min01_x), .b(r_min23_x), .bi(1'b0), .d(), .bo(lx_bo));
	am_sub #(.W(18)) u_c_ly (.a(r_min01_y), .b(r_min23_y), .bi(1'b0), .d(), .bo(ly_bo));
	am_sub #(.W(18)) u_c_mx (.a(r_max23_x), .b(r_max01_x), .bi(1'b0), .d(), .bo(mx_bo));
	am_sub #(.W(18)) u_c_my (.a(r_max23_y), .b(r_max01_y), .bi(1'b0), .d(), .bo(my_bo));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire lx_bo, ly_bo, mx_bo, my_bo;
	am_sub #(.W(18)) u_c_lx (.a(r_min01_x), .b(r_min23_x), .bi(1'b0), .d(), .bo(lx_bo));
	am_sub #(.W(18)) u_c_ly (.a(r_min01_y), .b(r_min23_y), .bi(1'b0), .d(), .bo(ly_bo));
	am_sub #(.W(18)) u_c_mx (.a(r_max23_x), .b(r_max01_x), .bi(1'b0), .d(), .bo(mx_bo));
	am_sub #(.W(18)) u_c_my (.a(r_max23_y), .b(r_max01_y), .bi(1'b0), .d(), .bo(my_bo));
`else
	wire lx_bo = lt18_u(r_min01_x, r_min23_x);
	wire ly_bo = lt18_u(r_min01_y, r_min23_y);
	wire mx_bo = lt18_u(r_max23_x, r_max01_x);
	wire my_bo = lt18_u(r_max23_y, r_max01_y);
`endif
	(* keep = "true" *) wire cmp_lx = (r_min01_x[17] != r_min23_x[17]) ? r_min01_x[17] : lx_bo;
	(* keep = "true" *) wire cmp_ly = (r_min01_y[17] != r_min23_y[17]) ? r_min01_y[17] : ly_bo;
	(* keep = "true" *) wire cmp_mx = (r_max23_x[17] != r_max01_x[17]) ? r_max23_x[17] : mx_bo;
	(* keep = "true" *) wire cmp_my = (r_max23_y[17] != r_max01_y[17]) ? r_max23_y[17] : my_bo;
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_lmxy_reg
		localparam integer HI = 9 * gs;
		wire [3:0] wl11;
		wire s_lx, s_ly, s_mx, s_my;
`ifdef SYNTH_KINTEX7
		for (gi = 0; gi < 4; gi = gi + 1) begin : g_wl11
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wl11 (.I0(c11_mid[2]), .O(wl11[gi]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gi = 0; gi < 4; gi = gi + 1) begin : g_wl11
			assign wl11[gi] = c11_mid[2];
		end
`else
		assign wl11 = {4{c11_mid[2]}};
`endif
		assign s_lx = cmp_lx;
		assign s_ly = cmp_ly;
		assign s_mx = cmp_mx;
		assign s_my = cmp_my;
		always @(posedge clk) begin
			if (wl11[0]) r_lx[HI +: 9] <= s_lx ? r_min01_x[HI +: 9] : r_min23_x[HI +: 9];
			if (wl11[1]) r_ly[HI +: 9] <= s_ly ? r_min01_y[HI +: 9] : r_min23_y[HI +: 9];
			if (wl11[2]) r_mx[HI +: 9] <= s_mx ? r_max01_x[HI +: 9] : r_max23_x[HI +: 9];
			if (wl11[3]) r_my[HI +: 9] <= s_my ? r_max01_y[HI +: 9] : r_max23_y[HI +: 9];
		end
	end
	endgenerate

	/* cnt == 14, 15, 16/21: r_mx1..r_win_max_y, r_clx..r_cmy, bx0..by1 (2 x 9-bit slices) */
	wire we_c12_raw = c_act && phase_oh[14];
	wire we_c13_raw = c_act && phase_oh[15];
	wire we_bx_raw  = c_act && (phase_oh[16] || (phase_oh[21] && empty_bx));
	wire is_c14_raw = phase_oh[16];
	always @(posedge clk) begin
		if (we_c11_win) begin
			r_win_mnx_sel <= win_min_x_sel;
			r_win_mny_sel <= win_min_y_sel;
		end
	end
	generate
	for (gs = 0; gs < 2; gs = gs + 1) begin : g_bbox_pipe
		localparam integer HI = 9 * gs;
		wire [5:0] wc12;
		wire [3:0] wc13, wbx, sc14;
		wire sw_mnx, sw_mxx, sw_mny, sw_mxy;
		wire sl_x, sh_x, sl_y, sh_y;
		for (gi = 0; gi < 6; gi = gi + 1) begin : g_wc12_b
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc12 (.I0(we_c12_raw), .O(wc12[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wc12[gi] = we_c12_raw;
`else
			assign wc12[gi] = we_c12_raw;
`endif
		end
		for (gi = 0; gi < 4; gi = gi + 1) begin : g_wc13_b
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wc13 (.I0(we_c13_raw), .O(wc13[gi]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wbx  (.I0(we_bx_raw),  .O(wbx[gi]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc14 (.I0(is_c14_raw), .O(sc14[gi]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wc13[gi] = we_c13_raw;
			assign wbx[gi] = we_bx_raw;
			assign sc14[gi] = is_c14_raw;
`else
			assign wc13[gi] = we_c13_raw;
			assign wbx[gi]  = we_bx_raw;
			assign sc14[gi] = is_c14_raw;
`endif
		end
		assign sw_mnx = r_win_mnx_sel;
		assign sw_mxx = win_max_x_sel;
		assign sw_mny = r_win_mny_sel;
		assign sw_mxy = win_max_y_sel;
		assign sl_x   = lo_scis_x_sel;
		assign sh_x   = hi_scis_x_sel;
		assign sl_y   = lo_scis_y_sel;
		assign sh_y   = hi_scis_y_sel;
		always @(posedge clk) begin
			if (wc12[0]) r_mx1[HI +: 9]       <= m_x1[HI +: 9];
			if (wc12[1]) r_my1[HI +: 9]       <= m_y1[HI +: 9];
			if (wc12[2]) r_win_min_x[HI +: 9] <= sw_mnx ? rx0[HI +: 9] : r_scis_rt_min_x[HI +: 9];
			if (wc12[3]) r_win_max_x[HI +: 9] <= sw_mxx ? rx1[HI +: 9] : r_scis_rt_max_x[HI +: 9];
			if (wc12[4]) r_win_min_y[HI +: 9] <= sw_mny ? ry0[HI +: 9] : r_scis_rt_min_y[HI +: 9];
			if (wc12[5]) r_win_max_y[HI +: 9] <= sw_mxy ? ry1[HI +: 9] : r_scis_rt_max_y[HI +: 9];
			if (wc13[0]) r_clx[HI +: 9]       <= sl_x ? r_win_min_x[HI +: 9] : r_lx[HI +: 9];
			if (wc13[1]) r_cly[HI +: 9]       <= sl_y ? r_win_min_y[HI +: 9] : r_ly[HI +: 9];
			if (wc13[2]) r_cmx[HI +: 9]       <= sh_x ? r_win_max_x[HI +: 9] : r_mx1[HI +: 9];
			if (wc13[3]) r_cmy[HI +: 9]       <= sh_y ? r_win_max_y[HI +: 9] : r_my1[HI +: 9];
			if (wbx[0])  bx0[HI +: 9]         <= sc14[0] ? r_clx[HI +: 9] : 9'd0;
			if (wbx[1])  by0[HI +: 9]         <= sc14[1] ? r_cly[HI +: 9] : 9'd0;
			if (wbx[2])  bx1[HI +: 9]         <= sc14[2] ? r_cmx[HI +: 9] : 9'd0;
			if (wbx[3])  by1[HI +: 9]         <= sc14[3] ? r_cmy[HI +: 9] : 9'd0;
		end
	end
	endgenerate

	/* dd (3 x 16-bit slices), r_dx1_dy2, r_cox_d, r_coy_d, acc, r_cabs, ca, r_cn (3 x 16-bit slices) */
	wire we_dx1dy2_raw = c_act && phase_oh[18];
	wire we_dd_raw     = c_act && phase_oh[19];
	wire we_cox_raw    = c_act && phase_oh[20];
	wire we_coy_raw    = c_act && phase_oh[21];
	wire we_acc_raw    = c_act && (phase_oh[25] || phase_oh[27] || phase_oh[29]);
	wire we_cabs_raw   = c_act && (|phase_oh[33:27]);
	wire ph_34_28_raw  = (|phase_oh[34:28]);
	wire ph_34_28_wca, ph_34_28_ca, ph_34_28_cb, ph_34_28_cc;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p34w (.I0(ph_34_28_raw), .O(ph_34_28_wca));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p34a (.I0(ph_34_28_raw), .O(ph_34_28_ca));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p34b (.I0(ph_34_28_raw), .O(ph_34_28_cb));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p34c (.I0(ph_34_28_raw), .O(ph_34_28_cc));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ph_34_28_wca = ph_34_28_raw;
	assign ph_34_28_ca = ph_34_28_raw;
	assign ph_34_28_cb = ph_34_28_raw;
	assign ph_34_28_cc = ph_34_28_raw;
`else
	assign ph_34_28_wca = ph_34_28_raw;
	assign ph_34_28_ca  = ph_34_28_raw;
	assign ph_34_28_cb  = ph_34_28_raw;
	assign ph_34_28_cc  = ph_34_28_raw;
`endif
	wire we_ca_raw     = c_act && ph_34_28_wca;
	wire we_cn_raw     = c_act && (|phase_oh[35:29]);
	generate
	for (gs = 0; gs < 3; gs = gs + 1) begin : g_48b_regs
		localparam integer HI = 16 * gs;
		wire wdx1dy2, wdd, wcox, wcoy, wacc, wcabs, wca, wcn;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdx  (.I0(we_dx1dy2_raw), .O(wdx1dy2));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wdd  (.I0(we_dd_raw),     .O(wdd));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wcox (.I0(we_cox_raw),    .O(wcox));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wcoy (.I0(we_coy_raw),    .O(wcoy));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wacc (.I0(we_acc_raw),    .O(wacc));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wcab (.I0(we_cabs_raw),   .O(wcabs));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wca  (.I0(we_ca_raw),     .O(wca));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wcn  (.I0(we_cn_raw),     .O(wcn));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign wdx1dy2 = we_dx1dy2_raw;
		assign wdd = we_dd_raw;
		assign wcox = we_cox_raw;
		assign wcoy = we_coy_raw;
		assign wacc = we_acc_raw;
		assign wcabs = we_cabs_raw;
		assign wca = we_ca_raw;
		assign wcn = we_cn_raw;
`else
		assign wdx1dy2 = we_dx1dy2_raw;
		assign wdd   = we_dd_raw;
		assign wcox  = we_cox_raw;
		assign wcoy  = we_coy_raw;
		assign wacc  = we_acc_raw;
		assign wcabs = we_cabs_raw;
		assign wca   = we_ca_raw;
		assign wcn   = we_cn_raw;
`endif
		always @(posedge clk) begin
			if (wdx1dy2) r_dx1_dy2[HI +: 16] <= mp[HI +: 16];
			if (wdd)     dd[HI +: 16]        <= dd_n[HI +: 16];
			if (wcox)    r_cox_d[HI +: 16]   <= cox_d[HI +: 16];
			if (wcoy)    r_coy_d[HI +: 16]   <= coy_d[HI +: 16];
			if (wacc)    acc[HI +: 16]       <= acc_n[HI +: 16];
			if (wcabs)   r_cabs[HI +: 16]    <= cabs[HI +: 16];
			if (wca) begin
				ca[HI +: 16]   <= r_cabs[HI +: 16];
				r_cabs_nz[gs]  <= (r_cabs[HI +: 16] != 16'd0);
			end
			if (wcn) begin
				r_cn[HI +: 16] <= cn[HI +: 16];
				if (gs == 0) r_cn_stk0 <= ca_stk_fast;
				if (gs == 1) begin
					r_cn_guard <= cn[23];
					r_cn_stk1  <= 1'b0;
				end
			end
		end
	end
	endgenerate
	always @(posedge clk) begin
		if (we_acc_raw) acc[49:48] <= 2'b00;
		if (we_dd_raw)  r_dd_ccw   <= {12{dd_n[47]}};
	end

	/* eA, eB, oX, oY, eE (sliced <= 16-bit write enables) */
	wire we_eab_raw = c_act && (phase_oh_b[20] || (phase_oh_b[21] && empty_eab));
	wire is_c16_eab = phase_oh_b[20];
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_edge_regs
		for (gs = 0; gs < 2; gs = gs + 1) begin : g_eab_s
			localparam integer HI = 12 * gs;
			localparam integer O_LO = (gs == 0) ? 0  : 13;
			localparam integer O_W  = (gs == 0) ? 13 : 12;
			wire wea, web, sc16a, sc16b, wox, woy;
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wea  (.I0(we_eab_raw), .O(wea));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_web  (.I0(we_eab_raw), .O(web));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc16a(.I0(is_c16_eab), .O(sc16a));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sc16b(.I0(is_c16_eab), .O(sc16b));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wox  (.I0(we_cox_raw), .O(wox));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_woy  (.I0(we_cox_raw), .O(woy));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wea = we_eab_raw;
			assign web = we_eab_raw;
			assign sc16a = is_c16_eab;
			assign sc16b = is_c16_eab;
			assign wox = we_cox_raw;
			assign woy = we_cox_raw;
`else
			assign wea   = we_eab_raw;
			assign web   = we_eab_raw;
			assign sc16a = is_c16_eab;
			assign sc16b = is_c16_eab;
			assign wox   = we_cox_raw;
			assign woy   = we_cox_raw;
`endif
			always @(posedge clk) begin
				if (wea) eA_pk[24 * ge + HI +: 12]   <= sc16a ? eA_n[ge][HI +: 12] : 12'd0;
				if (web) eB_pk[24 * ge + HI +: 12]   <= sc16b ? eB_n[ge][HI +: 12] : 12'd0;
				if (wox) oX_pk[25 * ge + O_LO +: O_W] <= oX_n[ge][O_LO +: O_W];
				if (woy) oY_pk[25 * ge + O_LO +: O_W] <= oY_n[ge][O_LO +: O_W];
			end
		end
		for (gs = 0; gs < 3; gs = gs + 1) begin : g_ee_s
			localparam integer HI = 16 * gs;
			wire emp_ee;
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_empee (.I0(empty), .O(emp_ee));
`else
			assign emp_ee = empty;
`endif
			wire wee_raw = run && ((phase_oh_b[21] && emp_ee) ||
				((ge == 0) ? phase_oh[26] : (ge == 1) ? phase_oh[28] : phase_oh[30]));
			wire wee_s;
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wees (.I0(wee_raw), .O(wee_s));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign wee_s = wee_raw;
`else
			assign wee_s = wee_raw;
`endif
			always @(posedge clk) begin
				if (wee_s)
					eE_pk[48 * ge + HI +: 16] <= emp_ee ? 16'd0 : eE_n[HI +: 16];
			end
		end
	end
	endgenerate

	/* r_wd (2 x 16-bit slices) */
	wire we_wd_raw = c_act && (|phase_oh[36:30]);
	wire we_wd_lo, we_wd_hi;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwdl (.I0(we_wd_raw), .O(we_wd_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwdh (.I0(we_wd_raw), .O(we_wd_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign we_wd_lo = we_wd_raw;
	assign we_wd_hi = we_wd_raw;
`else
	assign we_wd_lo = we_wd_raw;
	assign we_wd_hi = we_wd_raw;
`endif
	always @(posedge clk) begin
		if (we_wd_lo) r_wd[15:0]  <= c_out[15:0];
		if (we_wd_hi) r_wd[31:16] <= c_out[31:16];
	end

	/* Pre-computed dd == 0 in 3 x 16-bit chunks for empty check */
	wire [2:0] dd_nz = {dd[47:32] != 16'd0, dd[31:16] != 16'd0, dd[15:0] != 16'd0};

	wire [3:0] xmaj_hx, xmaj_hy;
	wire we_h_raw = run && phase_oh[10];
	wire we_hx_mid, we_hy_mid;
	wire [4:0] we_hx_lo, we_hx_hi, we_hy_lo, we_hy_hi;
	wire hx_isl_nx_l, hx_isl_nx_h, hx_isl_pr_l, hx_isl_pr_h, hx_isl_ng_l, hx_isl_ng_h, hx_isl_c_l, hx_isl_c_h;
	wire hy_isl_nx_l, hy_isl_nx_h, hy_isl_pr_l, hy_isl_pr_h, hy_isl_ng_l, hy_isl_ng_h, hy_isl_c_l, hy_isl_c_h;
	wire hx_ist_pr_l, hx_ist_pr_h, hx_ist_ng_l, hx_ist_ng_h, hx_ist_c4_l, hx_ist_c4_h, hx_ist_c6_l, hx_ist_c6_h;
	wire hy_ist_pr_l, hy_ist_pr_h, hy_ist_ng_l, hy_ist_ng_h, hy_ist_c3_l, hy_ist_c3_h, hy_ist_c5_l, hy_ist_c5_h;
`ifdef SYNTH_KINTEX7
	genvar ghb;
	generate
	for (ghb = 0; ghb < 4; ghb = ghb + 1) begin : g_xm_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_xmhx (.I0(xmaj), .O(xmaj_hx[ghb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_xmhy (.I0(xmaj), .O(xmaj_hy[ghb]));
	end
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whxm (.I0(we_h_raw), .O(we_hx_mid));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whym (.I0(we_h_raw), .O(we_hy_mid));
	for (ghb = 0; ghb < 5; ghb = ghb + 1) begin : g_wh_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whxl (.I0(we_hx_mid), .O(we_hx_lo[ghb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whxh (.I0(we_hx_mid), .O(we_hx_hi[ghb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whyl (.I0(we_hy_mid), .O(we_hy_lo[ghb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_whyh (.I0(we_hy_mid), .O(we_hy_hi[ghb]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilnl (.I0(is_line_hx),    .O(hx_isl_nx_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilnh (.I0(is_line_hx),    .O(hx_isl_nx_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilpl (.I0(is_line_hx),    .O(hx_isl_pr_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilph (.I0(is_line_hx),    .O(hx_isl_pr_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilgl (.I0(is_line_hx),    .O(hx_isl_ng_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilgh (.I0(is_line_hx),    .O(hx_isl_ng_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilcl (.I0(is_line_hx),    .O(hx_isl_c_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxilch (.I0(is_line_hx),    .O(hx_isl_c_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilnl (.I0(is_line_hy),    .O(hy_isl_nx_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilnh (.I0(is_line_hy),    .O(hy_isl_nx_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilpl (.I0(is_line_hy),    .O(hy_isl_pr_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilph (.I0(is_line_hy),    .O(hy_isl_pr_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilgl (.I0(is_line_hy),    .O(hy_isl_ng_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilgh (.I0(is_line_hy),    .O(hy_isl_ng_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilcl (.I0(is_line_hy),    .O(hy_isl_c_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyilch (.I0(is_line_hy),    .O(hy_isl_c_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxistpl(.I0(r_is_tri_c[4]), .O(hx_ist_pr_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxistph(.I0(r_is_tri_c[5]), .O(hx_ist_pr_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxistgl(.I0(r_is_tri_c[4]), .O(hx_ist_ng_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxistgh(.I0(r_is_tri_c[5]), .O(hx_ist_ng_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxist4l(.I0(r_is_tri_c[4]), .O(hx_ist_c4_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxist4h(.I0(r_is_tri_c[5]), .O(hx_ist_c4_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxist6l(.I0(r_is_tri_c[4]), .O(hx_ist_c6_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hxist6h(.I0(r_is_tri_c[5]), .O(hx_ist_c6_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyistpl(.I0(r_is_tri_c[6]), .O(hy_ist_pr_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyistph(.I0(r_is_tri_c[7]), .O(hy_ist_pr_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyistgl(.I0(r_is_tri_c[6]), .O(hy_ist_ng_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyistgh(.I0(r_is_tri_c[7]), .O(hy_ist_ng_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyist3l(.I0(r_is_tri_c[6]), .O(hy_ist_c3_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyist3h(.I0(r_is_tri_c[7]), .O(hy_ist_c3_h));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyist5l(.I0(r_is_tri_c[6]), .O(hy_ist_c5_l));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_hyist5h(.I0(r_is_tri_c[7]), .O(hy_ist_c5_h));
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar ghb;
	generate
	for (ghb = 0; ghb < 4; ghb = ghb + 1) begin : g_xm_b
		assign xmaj_hx[ghb] = xmaj;
		assign xmaj_hy[ghb] = xmaj;
	end
	assign we_hx_mid = we_h_raw;
	assign we_hy_mid = we_h_raw;
	for (ghb = 0; ghb < 5; ghb = ghb + 1) begin : g_wh_b
		assign we_hx_lo[ghb] = we_hx_mid;
		assign we_hx_hi[ghb] = we_hx_mid;
		assign we_hy_lo[ghb] = we_hy_mid;
		assign we_hy_hi[ghb] = we_hy_mid;
	end
	endgenerate
	assign hx_isl_nx_l = is_line_hx;
	assign hx_isl_nx_h = is_line_hx;
	assign hx_isl_pr_l = is_line_hx;
	assign hx_isl_pr_h = is_line_hx;
	assign hx_isl_ng_l = is_line_hx;
	assign hx_isl_ng_h = is_line_hx;
	assign hx_isl_c_l = is_line_hx;
	assign hx_isl_c_h = is_line_hx;
	assign hy_isl_nx_l = is_line_hy;
	assign hy_isl_nx_h = is_line_hy;
	assign hy_isl_pr_l = is_line_hy;
	assign hy_isl_pr_h = is_line_hy;
	assign hy_isl_ng_l = is_line_hy;
	assign hy_isl_ng_h = is_line_hy;
	assign hy_isl_c_l = is_line_hy;
	assign hy_isl_c_h = is_line_hy;
	assign hx_ist_pr_l = r_is_tri_c[4];
	assign hx_ist_pr_h = r_is_tri_c[5];
	assign hx_ist_ng_l = r_is_tri_c[4];
	assign hx_ist_ng_h = r_is_tri_c[5];
	assign hx_ist_c4_l = r_is_tri_c[4];
	assign hx_ist_c4_h = r_is_tri_c[5];
	assign hx_ist_c6_l = r_is_tri_c[4];
	assign hx_ist_c6_h = r_is_tri_c[5];
	assign hy_ist_pr_l = r_is_tri_c[6];
	assign hy_ist_pr_h = r_is_tri_c[7];
	assign hy_ist_ng_l = r_is_tri_c[6];
	assign hy_ist_ng_h = r_is_tri_c[7];
	assign hy_ist_c3_l = r_is_tri_c[6];
	assign hy_ist_c3_h = r_is_tri_c[7];
	assign hy_ist_c5_l = r_is_tri_c[6];
	assign hy_ist_c5_h = r_is_tri_c[7];
`else
	assign xmaj_hx   = {4{xmaj}};
	assign xmaj_hy   = {4{xmaj}};
	assign we_hx_mid = we_h_raw;
	assign we_hy_mid = we_h_raw;
	assign we_hx_lo  = {5{we_h_raw}};
	assign we_hx_hi  = {5{we_h_raw}};
	assign we_hy_lo  = {5{we_h_raw}};
	assign we_hy_hi  = {5{we_h_raw}};
	assign hx_isl_nx_l = is_line_hx; assign hx_isl_nx_h = is_line_hx;
	assign hx_isl_pr_l = is_line_hx; assign hx_isl_pr_h = is_line_hx;
	assign hx_isl_ng_l = is_line_hx; assign hx_isl_ng_h = is_line_hx;
	assign hx_isl_c_l  = is_line_hx; assign hx_isl_c_h  = is_line_hx;
	assign hy_isl_nx_l = is_line_hy; assign hy_isl_nx_h = is_line_hy;
	assign hy_isl_pr_l = is_line_hy; assign hy_isl_pr_h = is_line_hy;
	assign hy_isl_ng_l = is_line_hy; assign hy_isl_ng_h = is_line_hy;
	assign hy_isl_c_l  = is_line_hy; assign hy_isl_c_h  = is_line_hy;
	assign hx_ist_pr_l = r_is_tri_c[4]; assign hx_ist_pr_h = r_is_tri_c[5];
	assign hx_ist_ng_l = r_is_tri_c[4]; assign hx_ist_ng_h = r_is_tri_c[5];
	assign hx_ist_c4_l = r_is_tri_c[4]; assign hx_ist_c4_h = r_is_tri_c[5];
	assign hx_ist_c6_l = r_is_tri_c[4]; assign hx_ist_c6_h = r_is_tri_c[5];
	assign hy_ist_pr_l = r_is_tri_c[6]; assign hy_ist_pr_h = r_is_tri_c[7];
	assign hy_ist_ng_l = r_is_tri_c[6]; assign hy_ist_ng_h = r_is_tri_c[7];
	assign hy_ist_c3_l = r_is_tri_c[6]; assign hy_ist_c3_h = r_is_tri_c[7];
	assign hy_ist_c5_l = r_is_tri_c[6]; assign hy_ist_c5_h = r_is_tri_c[7];
`endif
	wire [23:0] hx_next = {hx_isl_nx_h ? (xmaj_hx[0] ? 12'd0 : half_line_w[23:12]) : half_point_s[23:12],
	                       hx_isl_nx_l ? (xmaj_hx[0] ? 12'd0 : half_line_w[11:0])  : half_point_s[11:0]};
	wire [23:0] hy_next = {hy_isl_nx_h ? (xmaj_hy[0] ? half_line_w[23:12] : 12'd0) : half_point_s[23:12],
	                       hy_isl_nx_l ? (xmaj_hy[0] ? half_line_w[11:0]  : 12'd0) : half_point_s[11:0]};

	wire [11:0] hx_pre_lo = hx_ist_pr_l ? 12'd0   : (hx_isl_pr_l ? half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hx_pre_hi = hx_ist_pr_h ? 12'd0   : (hx_isl_pr_h ? half_line_w[23:12] : half_point_s[23:12]);
	wire [11:0] hx_neg_lo = hx_ist_ng_l ? 12'hFFF : ~(hx_isl_ng_l ? half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hx_neg_hi = hx_ist_ng_h ? 12'hFFF : ~(hx_isl_ng_h ? half_line_w[23:12] : half_point_s[23:12]);
	wire [11:0] hx_c4_lo  = hx_ist_c4_l ? 12'd0 : (r_is_line_c[5] ? ~half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hx_c4_hi  = hx_ist_c4_h ? 12'd0 : (r_is_line_c[6] ? ~half_line_w[23:12] : half_point_s[23:12]);
	wire [11:0] hx_c6_lo  = hx_ist_c6_l ? 12'd0 : (r_is_pt_c[0]   ? ~half_point_s[11:0] : half_line_w[11:0]);
	wire [11:0] hx_c6_hi  = hx_ist_c6_h ? 12'd0 : (r_is_pt_c[0]   ? ~half_point_s[23:12] : half_line_w[23:12]);

	wire [11:0] hy_pre_lo = hy_ist_pr_l ? 12'd0   : (hy_isl_pr_l ? half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hy_pre_hi = hy_ist_pr_h ? 12'd0   : (hy_isl_pr_h ? half_line_w[23:12] : half_point_s[23:12]);
	wire [11:0] hy_neg_lo = hy_ist_ng_l ? 12'hFFF : ~(hy_isl_ng_l ? half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hy_neg_hi = hy_ist_ng_h ? 12'hFFF : ~(hy_isl_ng_h ? half_line_w[23:12] : half_point_s[23:12]);
	wire [11:0] hy_c3_lo  = hy_ist_c3_l ? 12'd0 : (r_is_pt_c[0]   ? ~half_point_s[11:0] : half_line_w[11:0]);
	wire [11:0] hy_c3_hi  = hy_ist_c3_h ? 12'd0 : (r_is_pt_c[0]   ? ~half_point_s[23:12] : half_line_w[23:12]);
	wire [11:0] hy_c5_lo  = hy_ist_c5_l ? 12'd0 : (r_is_line_c[7] ? ~half_line_w[11:0]  : half_point_s[11:0]);
	wire [11:0] hy_c5_hi  = hy_ist_c5_h ? 12'd0 : (r_is_line_c[8] ? ~half_line_w[23:12] : half_point_s[23:12]);

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_xmaj <= 1'b0;
		end else if (we_h_raw) begin
			r_xmaj <= xmaj;
		end
	end

	always @(posedge clk) begin
		if (we_hx_lo[0]) r_hx[11:0]       <= hx_next[11:0];
		if (we_hx_hi[0]) r_hx[23:12]      <= hx_next[23:12];
		if (we_hy_lo[0]) r_hy[11:0]       <= hy_next[11:0];
		if (we_hy_hi[0]) r_hy[23:12]      <= hy_next[23:12];
		if (we_hx_lo[1]) r_hx_g[11:0]     <= (hx_isl_pr_l && xmaj_hx[1]) ? 12'd0   : hx_pre_lo;
		if (we_hx_lo[2]) r_hx_g_neg[11:0] <= (hx_isl_ng_l && xmaj_hx[2]) ? 12'hFFF : hx_neg_lo;
		if (we_hx_lo[3]) r_hx_g_c4[11:0]  <= (hx_isl_c_l  && xmaj_hx[3]) ? (hx_ist_c4_l ? 12'd0 : (r_is_line_c[5] ? 12'hFFF : 12'd0)) : hx_c4_lo;
		if (we_hx_lo[4]) r_hx_g_c6[11:0]  <= (hx_isl_c_l  && xmaj_hx[3]) ? 12'd0   : hx_c6_lo;
		if (we_hx_hi[1]) r_hx_g[23:12]    <= (hx_isl_pr_h && xmaj_hx[1]) ? 12'd0   : hx_pre_hi;
		if (we_hx_hi[2]) r_hx_g_neg[23:12]<= (hx_isl_ng_h && xmaj_hx[2]) ? 12'hFFF : hx_neg_hi;
		if (we_hx_hi[3]) r_hx_g_c4[23:12] <= (hx_isl_c_h  && xmaj_hx[3]) ? (hx_ist_c4_h ? 12'd0 : (r_is_line_c[6] ? 12'hFFF : 12'd0)) : hx_c4_hi;
		if (we_hx_hi[4]) r_hx_g_c6[23:12] <= (hx_isl_c_h  && xmaj_hx[3]) ? 12'd0   : hx_c6_hi;
		if (we_hy_lo[1]) r_hy_g[11:0]     <= (hy_isl_pr_l && !xmaj_hy[1]) ? 12'd0   : hy_pre_lo;
		if (we_hy_lo[2]) r_hy_g_neg[11:0] <= (hy_isl_ng_l && !xmaj_hy[2]) ? 12'hFFF : hy_neg_lo;
		if (we_hy_lo[3]) r_hy_g_c3[11:0]  <= (hy_isl_c_l  && !xmaj_hy[3]) ? 12'd0   : hy_c3_lo;
		if (we_hy_lo[4]) r_hy_g_c5[11:0]  <= (hy_isl_c_l  && !xmaj_hy[3]) ? (hy_ist_c5_l ? 12'd0 : (r_is_line_c[7] ? 12'hFFF : 12'd0)) : hy_c5_lo;
		if (we_hy_hi[1]) r_hy_g[23:12]    <= (hy_isl_pr_h && !xmaj_hy[1]) ? 12'd0   : hy_pre_hi;
		if (we_hy_hi[2]) r_hy_g_neg[23:12]<= (hy_isl_ng_h && !xmaj_hy[2]) ? 12'hFFF : hy_neg_hi;
		if (we_hy_hi[3]) r_hy_g_c3[23:12] <= (hy_isl_c_h  && !xmaj_hy[3]) ? 12'd0   : hy_c3_hi;
		if (we_hy_hi[4]) r_hy_g_c5[23:12] <= (hy_isl_c_h  && !xmaj_hy[3]) ? (hy_ist_c5_h ? 12'd0 : (r_is_line_c[8] ? 12'hFFF : 12'd0)) : hy_c5_hi;
	end

	wire bx_empty_bo, by_empty_bo;
`ifdef SYNTH_KINTEX7
	am_sub #(.W(18)) u_bx_emp (.a(bx0), .b(bx1), .bi(1'b0), .d(), .bo(bx_empty_bo));
	am_sub #(.W(18)) u_by_emp (.a(by0), .b(by1), .bi(1'b0), .d(), .bo(by_empty_bo));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_sub #(.W(18)) u_bx_emp (.a(bx0), .b(bx1), .bi(1'b0), .d(), .bo(bx_empty_bo));
	am_sub #(.W(18)) u_by_emp (.a(by0), .b(by1), .bi(1'b0), .d(), .bo(by_empty_bo));
`else
	function lt17_u;
		input [16:0] a, b;
		reg [7:0] lt2, eq2;
		reg [3:0] lt4, eq4;
		reg [1:0] lt8, eq8;
		reg lt16, eq16;
		integer i2;
		begin
			for (i2 = 0; i2 < 8; i2 = i2 + 1) begin
				lt2[i2] = (~a[2*i2+1] & b[2*i2+1]) | ((~a[2*i2+1] | b[2*i2+1]) & ~a[2*i2] & b[2*i2]);
				eq2[i2] = (a[2*i2+1] ~^ b[2*i2+1]) & (a[2*i2] ~^ b[2*i2]);
			end
			for (i2 = 0; i2 < 4; i2 = i2 + 1) begin
				lt4[i2] = lt2[2*i2+1] | (eq2[2*i2+1] & lt2[2*i2]);
				eq4[i2] = eq2[2*i2+1] & eq2[2*i2];
			end
			lt8[0] = lt4[1] | (eq4[1] & lt4[0]); eq8[0] = eq4[1] & eq4[0];
			lt8[1] = lt4[3] | (eq4[3] & lt4[2]); eq8[1] = eq4[3] & eq4[2];
			lt16   = lt8[1] | (eq8[1] & lt8[0]); eq16   = eq8[1] & eq8[0];
			lt17_u = (!a[16] & b[16]) | ((!a[16] | b[16]) & lt16);
		end
	endfunction
	assign bx_empty_bo = lt17_u(bx0[16:0], bx1[16:0]);
	assign by_empty_bo = lt17_u(by0[16:0], by1[16:0]);
`endif
	wire bx1_le_bx0 = !((bx0[17] != bx1[17]) ? bx0[17] : bx_empty_bo);
	wire by1_le_by0 = !((by0[17] != by1[17]) ? by0[17] : by_empty_bo);

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			front <= 1'b0;
			empty <= 1'b0;
		end else if (run && phase_oh_b[20]) begin
			front <= !is_tri || (cull[2] ? ccw : !ccw);
			empty <= (dd_nz == 3'b000) || (is_tri && cull[0] && (cull[2] ? ccw : !ccw)) ||
				(is_tri && cull[1] && !(cull[2] ? ccw : !ccw)) || bx1_le_bx0 || by1_le_by0;
		end
	end

	/* Pre-register phase_30 value (dd/pdd) and we_ci on cycle 25..31 to cut dd/phase_oh -> r_ci (15 -> <= 9 cmos3) */
	reg [47:0] r_p30_val;
	reg r_p30_sgn, r_we_ci;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_we_ci <= 1'b0;
		end else begin
			r_we_ci <= !start && run && (|phase_oh_b[31:25]);
		end
	end
`ifdef SYNTH_KINTEX7
	wire en_p30_lo, en_p30_hi, en_p30_top;
	wire en_p30_raw = run && phase_oh_b[29];
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ep30l (.I0(en_p30_raw), .O(en_p30_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ep30h (.I0(en_p30_raw), .O(en_p30_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ep30t (.I0(en_p30_raw), .O(en_p30_top));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire en_p30_lo, en_p30_hi, en_p30_top;
	wire en_p30_raw = run && phase_oh_b[29];
	assign en_p30_lo = en_p30_raw;
	assign en_p30_hi = en_p30_raw;
	assign en_p30_top = en_p30_raw;
`else
	wire en_p30_lo  = run && phase_oh_b[29];
	wire en_p30_hi  = en_p30_lo;
	wire en_p30_top = en_p30_lo;
`endif
	always @(posedge clk) begin
		if (en_p30_lo)  r_p30_val[15:0]  <= r_is_tri_c[1] ? dd[15:0]  : pdd[15:0];
		if (en_p30_hi)  r_p30_val[31:16] <= r_is_tri_c[2] ? dd[31:16] : pdd[31:16];
		if (en_p30_top) begin
			r_p30_val[47:32] <= r_is_tri_c[3] ? dd[47:32] : pdd[47:32];
			r_p30_sgn        <= r_is_tri_c[3] ? dd[47]    : pdd[47];
		end
	end

	wire [32:26] phc_ci_0, phc_ci_1, phc_ci_2;
`ifdef SYNTH_KINTEX7
	genvar gci;
	generate
	for (gci = 26; gci <= 32; gci = gci + 1) begin : g_phc_ci
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_phci0 (.I0(phase_oh_c[gci]), .O(phc_ci_0[gci]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_phci1 (.I0(phase_oh_c[gci]), .O(phc_ci_1[gci]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_phci2 (.I0(phase_oh_c[gci]), .O(phc_ci_2[gci]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gci;
	generate
	for (gci = 26; gci <= 32; gci = gci + 1) begin : g_phc_ci
		assign phc_ci_0[gci] = phase_oh_c[gci];
		assign phc_ci_1[gci] = phase_oh_c[gci];
		assign phc_ci_2[gci] = phase_oh_c[gci];
	end
	endgenerate
`else
	assign phc_ci_0 = phase_oh_c[32:26];
	assign phc_ci_1 = phase_oh_c[32:26];
	assign phc_ci_2 = phase_oh_c[32:26];
`endif

	wire [47:0] ci_g0, ci_g1, ci_g2, ci_g3;
	assign ci_g0[15:0]  = ({16{phc_ci_0[26]}} & pdx1_ext[15:0])   | ({16{phc_ci_0[27]}} & pdy1_ext[15:0]);
	assign ci_g0[31:16] = ({16{phc_ci_1[26]}} & pdx1_ext[31:16])  | ({16{phc_ci_1[27]}} & pdy1_ext[31:16]);
	assign ci_g0[47:32] = ({16{phc_ci_2[26]}} & pdx1_ext[47:32])  | ({16{phc_ci_2[27]}} & pdy1_ext[47:32]);
	assign ci_g1[15:0]  = ({16{phc_ci_0[28]}} & pdx2_ext[15:0])   | ({16{phc_ci_0[29]}} & pdy2_ext[15:0]);
	assign ci_g1[31:16] = ({16{phc_ci_1[28]}} & pdx2_ext[31:16])  | ({16{phc_ci_1[29]}} & pdy2_ext[31:16]);
	assign ci_g1[47:32] = ({16{phc_ci_2[28]}} & pdx2_ext[47:32])  | ({16{phc_ci_2[29]}} & pdy2_ext[47:32]);
	assign ci_g2[15:0]  = ({16{phc_ci_0[30]}} & r_p30_val[15:0])  | ({16{phc_ci_0[31]}} & r_cox_d[15:0]);
	assign ci_g2[31:16] = ({16{phc_ci_1[30]}} & r_p30_val[31:16]) | ({16{phc_ci_1[31]}} & r_cox_d[31:16]);
	assign ci_g2[47:32] = ({16{phc_ci_2[30]}} & r_p30_val[47:32]) | ({16{phc_ci_2[31]}} & r_cox_d[47:32]);
	assign ci_g3[15:0]  =  {16{phc_ci_0[32]}} & r_coy_d[15:0];
	assign ci_g3[31:16] =  {16{phc_ci_1[32]}} & r_coy_d[31:16];
	assign ci_g3[47:32] =  {16{phc_ci_2[32]}} & r_coy_d[47:32];
	wire [47:0] ci_next = (ci_g0 | ci_g1) | (ci_g2 | ci_g3);
	wire [4:0] ci_sh_next = phc_ci_2[30] ? 5'd16 : 5'd8;
	wire ci_sgn_g0 = (phc_ci_2[26] & pdx1[23])    | (phc_ci_2[27] & pdy1[23]);
	wire ci_sgn_g1 = (phc_ci_2[28] & pdx2[23])    | (phc_ci_2[29] & pdy2[23]);
	wire ci_sgn_g2 = (phc_ci_2[30] & r_p30_sgn)   | (phc_ci_2[31] & r_cox_d[47]);
	wire ci_sgn_g3 =  phc_ci_2[32] & r_coy_d[47];
	wire ci_sgn_next = (ci_sgn_g0 | ci_sgn_g1) | (ci_sgn_g2 | ci_sgn_g3);

	wire we_ci_s0, we_ci_s1, we_ci_s2, we_ci_ctrl;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_weci0 (.I0(r_we_ci), .O(we_ci_s0));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_weci1 (.I0(r_we_ci), .O(we_ci_s1));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_weci2 (.I0(r_we_ci), .O(we_ci_s2));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wecic (.I0(r_we_ci), .O(we_ci_ctrl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign we_ci_s0 = r_we_ci;
	assign we_ci_s1 = r_we_ci;
	assign we_ci_s2 = r_we_ci;
	assign we_ci_ctrl = r_we_ci;
`else
	assign we_ci_s0 = r_we_ci;
	assign we_ci_s1 = r_we_ci;
	assign we_ci_s2 = r_we_ci;
	assign we_ci_ctrl = r_we_ci;
`endif

	always @(posedge clk) begin
		if (we_ci_s0) r_ci[15:0]  <= ci_next[15:0];
		if (we_ci_s1) r_ci[31:16] <= ci_next[31:16];
		if (we_ci_s2) r_ci[47:32] <= ci_next[47:32];
		if (we_ci_ctrl) begin
			r_ci_sh  <= ci_sh_next;
			r_ci_sgn <= ci_sgn_next;
		end
	end

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_we <= 1'b0;
			r_cs <= 1'b0;
			r_csh <= 5'd0;
			cs <= 1'b0;
			clz <= 6'd0;
			r_clz_hi_oh_a <= 6'd1;
			r_clz_hi_oh_b <= 6'd1;
			r_clz_hi_oh_c <= 6'd1;
			r_clz_lo_oh_a <= 4'd1;
			r_clz_lo_oh_b <= 4'd1;
			r_clz_lo_oh_c <= 4'd1;
			r_clz0_a <= 1'b0;
			r_clz0_b <= 1'b0;
			r_clz0_c <= 1'b0;
			csh <= 5'd0;
			r_cs2 <= 1'b0;
			r_czs <= 9'd0;
			r_ca_zero <= 1'b1;
			r_wa <= 7'd0;
		end else begin
			r_we <= 1'b0;
			if (run) begin
				/* 27-33: int2f stage 1; 28-34: stage 2; 29-35: stage 3; 30-36: stage 4 into fp registers 40-46 */
				if (|phase_oh[33:27]) begin
					r_cs <= r_ci_sgn;
					r_csh <= r_ci_sh;
				end
				if (ph_34_28_ca) begin
					cs <= r_cs;
					clz <= cl;
					r_clz_hi_oh_a <= (|cabs_nz[5:0]) ? {cabs_nz == 6'b000001, ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&~cabs_nz[2]&cabs_nz[1], ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&cabs_nz[2], ~cabs_nz[5]&~cabs_nz[4]&cabs_nz[3], ~cabs_nz[5]&cabs_nz[4], cabs_nz[5]} : 6'd0;
					r_clz_lo_oh_a <= {(cl[2:1] == 2'd3), (cl[2:1] == 2'd2), (cl[2:1] == 2'd1), (cl[2:1] == 2'd0)};
					r_clz0_a <= cl[0];
					csh <= r_csh;
				end
				if (ph_34_28_cb) begin
					r_clz_hi_oh_b <= (|cabs_nz[5:0]) ? {cabs_nz == 6'b000001, ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&~cabs_nz[2]&cabs_nz[1], ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&cabs_nz[2], ~cabs_nz[5]&~cabs_nz[4]&cabs_nz[3], ~cabs_nz[5]&cabs_nz[4], cabs_nz[5]} : 6'd0;
					r_clz_lo_oh_b <= {(cl[2:1] == 2'd3), (cl[2:1] == 2'd2), (cl[2:1] == 2'd1), (cl[2:1] == 2'd0)};
					r_clz0_b <= cl[0];
				end
				if (ph_34_28_cc) begin
					r_clz_hi_oh_c <= (|cabs_nz[5:0]) ? {cabs_nz == 6'b000001, ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&~cabs_nz[2]&cabs_nz[1], ~cabs_nz[5]&~cabs_nz[4]&~cabs_nz[3]&cabs_nz[2], ~cabs_nz[5]&~cabs_nz[4]&cabs_nz[3], ~cabs_nz[5]&cabs_nz[4], cabs_nz[5]} : 6'd0;
					r_clz_lo_oh_c <= {(cl[2:1] == 2'd3), (cl[2:1] == 2'd2), (cl[2:1] == 2'd1), (cl[2:1] == 2'd0)};
					r_clz0_c <= cl[0];
				end
				if (|phase_oh[35:29]) begin
					r_cs2 <= cs;
					r_czs <= czs;
					r_ca_zero <= (r_cabs_nz == 3'b000);
				end
				if (|phase_oh[36:30]) begin
					r_we <= 1'b1;
					r_wa <= wa_n;
				end
			end
		end
	end


	wire s1_snap_ge_141 = snap_exp_ovf[7] && ((snap_exp_ovf[6:4] != 3'd0) || (snap_exp_ovf[3:0] >= 4'd13));
	wire s1_snap_eq_140 = snap_exp_ovf[7] && (snap_exp_ovf[6:4] == 3'd0) && (snap_exp_ovf[3:0] == 4'd12);
	wire s1_snap_mant_h = (&snap_mant[22:17]) && (&snap_mant[16:11]) && (&snap_mant[10:6]) && (&snap_mant[5:1]);
	wire s1_snap_ovf_w  = s1_snap_ge_141 || (s1_snap_eq_140 && s1_snap_mant_h);
	wire s1_snap_lt_118 = !snap_exp_ovf[7] && ((snap_exp_ovf[6:4] != 3'd7) || (snap_exp_ovf[3:0] < 4'd6));
	wire s1_snap_mant_nz = (|snap_mant[22:17]) || (|snap_mant[16:11]) || (|snap_mant[10:5]) || (|snap_mant[4:0]);
	wire s1_snap_zout_w = ((&snap_exp_ovf) && s1_snap_mant_nz) || s1_snap_lt_118;

`ifdef SYNTH_KINTEX7
	wire [3:0] start_mid, run_mid;
	wire [15:0] start_leaf, run_leaf;
	genvar gst;
	generate
	for (gst = 0; gst < 4; gst = gst + 1) begin : g_st_mid
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stm (.I0(start), .O(start_mid[gst]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rnm (.I0(run),   .O(run_mid[gst]));
	end
	for (gst = 0; gst < 16; gst = gst + 1) begin : g_st_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stl (.I0(start_mid[gst / 4]), .O(start_leaf[gst]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rnl (.I0(run_mid[gst / 4]),   .O(run_leaf[gst]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] start_mid, run_mid;
	wire [15:0] start_leaf, run_leaf;
	genvar gst;
	generate
	for (gst = 0; gst < 4; gst = gst + 1) begin : g_st_mid
		assign start_mid[gst] = start;
		assign run_mid[gst] = run;
	end
	for (gst = 0; gst < 16; gst = gst + 1) begin : g_st_leaf
		assign start_leaf[gst] = start_mid[gst / 4];
		assign run_leaf[gst] = run_mid[gst / 4];
	end
	endgenerate
`else
	wire [15:0] start_leaf = {16{start}};
	wire [15:0] run_leaf   = {16{run}};
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			phase_oh[11:0]    <= 12'd0;
			phase_oh_b[11:0]  <= 12'd0;
			phase_oh_c[11:0]  <= 12'd0;
		end else begin
			if (start_leaf[0])  phase_oh[11:0]    <= 12'd1; else if (run_leaf[0])  phase_oh[11:0]    <= {phase_oh[10:0], 1'b0};
			if (start_leaf[3])  phase_oh_b[11:0]  <= 12'd1; else if (run_leaf[3])  phase_oh_b[11:0]  <= {phase_oh_b[10:0], 1'b0};
			if (start_leaf[6])  phase_oh_c[11:0]  <= 12'd1; else if (run_leaf[6])  phase_oh_c[11:0]  <= {phase_oh_c[10:0], 1'b0};
		end
	end
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			phase_oh[23:12]   <= 12'd0;
			phase_oh_b[23:12] <= 12'd0;
			phase_oh_c[23:12] <= 12'd0;
		end else begin
			if (start_leaf[1])  phase_oh[23:12]   <= 12'd0; else if (run_leaf[1])  phase_oh[23:12]   <= phase_oh[22:11];
			if (start_leaf[4])  phase_oh_b[23:12] <= 12'd0; else if (run_leaf[4])  phase_oh_b[23:12] <= phase_oh_b[22:11];
			if (start_leaf[7])  phase_oh_c[23:12] <= 12'd0; else if (run_leaf[7])  phase_oh_c[23:12] <= phase_oh_c[22:11];
		end
	end
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			phase_oh[36:24]   <= 13'd0;
			phase_oh_b[36:24] <= 13'd0;
			phase_oh_c[36:24] <= 13'd0;
		end else begin
			if (start_leaf[2])  phase_oh[36:24]   <= 13'd0; else if (run_leaf[2])  phase_oh[36:24]   <= phase_oh[35:23];
			if (start_leaf[5])  phase_oh_b[36:24] <= 13'd0; else if (run_leaf[5])  phase_oh_b[36:24] <= phase_oh_b[35:23];
			if (start_leaf[8])  phase_oh_c[36:24] <= 13'd0; else if (run_leaf[8])  phase_oh_c[36:24] <= phase_oh_c[35:23];
		end
	end
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_smin <= 32'd0;
			r_smax <= 32'd0;
			r_rts  <= 32'd0;
		end else begin
			if (start_leaf[9])  r_smin[15:0]  <= smin[15:0];
			if (start_leaf[10]) r_smin[31:16] <= smin[31:16];
			if (start_leaf[11]) r_smax[15:0]  <= smax[15:0];
			if (start_leaf[12]) r_smax[31:16] <= smax[31:16];
			if (start_leaf[13]) r_rts[15:0]   <= rts[15:0];
			if (start_leaf[14]) r_rts[31:16]  <= rts[31:16];
		end
	end

`ifdef SYNTH_KINTEX7
	wire start_ctrl_b, start_snap_b, start_mode_b;
	wire [7:0] start_mode_t, mtri;
	wire [5:0] start_mode_l, mline;
	wire run_snap_raw, run_snap_dlo, run_snap_dhi, run_snap_ctl;
	wire run_s2_ctl, run_s2_b0, run_s2_b1, run_s2_b2, run_tl_b;
	wire ph_10_1_lo, ph_10_1_hi, ph_10_1_v;
	wire rsv1_b0, rsv1_b1, rsv2_ctl, rsv2_0, rsv2_1, rsv2_2;
	wire mode_tri_w  = mode[1] || mode[3];
	wire mode_line_w = !(mode[1] || mode[3]) && (mode[1:0] == 2'd1);
	wire mode_pt_w   = !(mode[1] || mode[3]) && (mode[1:0] != 2'd1);
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stctrl (.I0(start_mid[3]),     .O(start_ctrl_b));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stsnap (.I0(start_mid[3]),     .O(start_snap_b));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stmdb  (.I0(start_mid[3]),     .O(start_mode_b));
	genvar gmc;
	generate
	for (gmc = 0; gmc < 8; gmc = gmc + 1) begin : g_st_tri_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stmdt (.I0(start_mid[3]), .O(start_mode_t[gmc]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mtri  (.I0(mode_tri_w),   .O(mtri[gmc]));
	end
	for (gmc = 0; gmc < 6; gmc = gmc + 1) begin : g_st_ln_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stmdl (.I0(start_mid[3]), .O(start_mode_l[gmc]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mlin  (.I0(mode_line_w),  .O(mline[gmc]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h2)) u_rnsnraw(.I0(run_mid[3]), .I1(start_mid[3]), .O(run_snap_raw));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rnsndl (.I0(run_snap_raw),    .O(run_snap_dlo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rnsndh (.I0(run_snap_raw),    .O(run_snap_dhi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rnsnct (.I0(run_snap_raw),    .O(run_snap_ctl));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rns2ct (.I0(run_snap_raw),    .O(run_s2_ctl));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rns2b0 (.I0(run_snap_raw),    .O(run_s2_b0));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rns2b1 (.I0(run_snap_raw),    .O(run_s2_b1));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rns2b2 (.I0(run_snap_raw),    .O(run_s2_b2));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rntlb  (.I0(run_mid[3]),      .O(run_tl_b));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p101l  (.I0(|phase_oh[10:1]), .O(ph_10_1_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p101h  (.I0(|phase_oh[10:1]), .O(ph_10_1_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_p101v  (.I0(|phase_oh[10:1]), .O(ph_10_1_v));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv10  (.I0(r_snap_v1),       .O(rsv1_b0));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv11  (.I0(r_snap_v1),       .O(rsv1_b1));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv2ct (.I0(r_snap_v2),       .O(rsv2_ctl));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv20  (.I0(r_snap_v2),       .O(rsv2_0));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv21  (.I0(r_snap_v2),       .O(rsv2_1));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rsv22  (.I0(r_snap_v2),       .O(rsv2_2));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire start_ctrl_b, start_snap_b, start_mode_b;
	wire [7:0] start_mode_t, mtri;
	wire [5:0] start_mode_l, mline;
	wire run_snap_raw, run_snap_dlo, run_snap_dhi, run_snap_ctl;
	wire run_s2_ctl, run_s2_b0, run_s2_b1, run_s2_b2, run_tl_b;
	wire ph_10_1_lo, ph_10_1_hi, ph_10_1_v;
	wire rsv1_b0, rsv1_b1, rsv2_ctl, rsv2_0, rsv2_1, rsv2_2;
	wire mode_tri_w  = mode[1] || mode[3];
	wire mode_line_w = !(mode[1] || mode[3]) && (mode[1:0] == 2'd1);
	wire mode_pt_w   = !(mode[1] || mode[3]) && (mode[1:0] != 2'd1);
	assign start_ctrl_b = start_mid[3];
	assign start_snap_b = start_mid[3];
	assign start_mode_b = start_mid[3];
	genvar gmc;
	generate
	for (gmc = 0; gmc < 8; gmc = gmc + 1) begin : g_st_tri_b
		assign start_mode_t[gmc] = start_mid[3];
		assign mtri[gmc] = mode_tri_w;
	end
	for (gmc = 0; gmc < 6; gmc = gmc + 1) begin : g_st_ln_b
		assign start_mode_l[gmc] = start_mid[3];
		assign mline[gmc] = mode_line_w;
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h2)) u_rnsnraw(.I0(run_mid[3]), .I1(start_mid[3]), .O(run_snap_raw));
	assign run_snap_dlo = run_snap_raw;
	assign run_snap_dhi = run_snap_raw;
	assign run_snap_ctl = run_snap_raw;
	assign run_s2_ctl = run_snap_raw;
	assign run_s2_b0 = run_snap_raw;
	assign run_s2_b1 = run_snap_raw;
	assign run_s2_b2 = run_snap_raw;
	assign run_tl_b = run_mid[3];
	assign ph_10_1_lo = |phase_oh[10:1];
	assign ph_10_1_hi = |phase_oh[10:1];
	assign ph_10_1_v = |phase_oh[10:1];
	assign rsv1_b0 = r_snap_v1;
	assign rsv1_b1 = r_snap_v1;
	assign rsv2_ctl = r_snap_v2;
	assign rsv2_0 = r_snap_v2;
	assign rsv2_1 = r_snap_v2;
	assign rsv2_2 = r_snap_v2;
`else
	wire mode_tri_w  = mode[1] || mode[3];
	wire mode_line_w = !(mode[1] || mode[3]) && (mode[1:0] == 2'd1);
	wire mode_pt_w   = !(mode[1] || mode[3]) && (mode[1:0] != 2'd1);
	wire start_ctrl_b = start_leaf[15], start_snap_b = start_leaf[15], start_mode_b = start_leaf[15];
	wire [7:0] start_mode_t = {8{start_leaf[15]}};
	wire [5:0] start_mode_l = {6{start_leaf[15]}};
	wire [7:0] mtri  = {8{mode_tri_w}};
	wire [5:0] mline = {6{mode_line_w}};
	wire run_snap_raw = !start_leaf[15] && run_leaf[15];
	wire run_snap_dlo = run_snap_raw, run_snap_dhi = run_snap_raw, run_snap_ctl = run_snap_raw;
	wire run_s2_ctl = run_snap_raw, run_s2_b0 = run_snap_raw, run_s2_b1 = run_snap_raw, run_s2_b2 = run_snap_raw;
	wire run_tl_b = run_leaf[15];
	wire ph_10_1_lo = |phase_oh[10:1], ph_10_1_hi = |phase_oh[10:1], ph_10_1_v = |phase_oh[10:1];
	wire rsv1_b0 = r_snap_v1, rsv1_b1 = r_snap_v1;
	wire rsv2_ctl = r_snap_v2, rsv2_0 = r_snap_v2, rsv2_1 = r_snap_v2, rsv2_2 = r_snap_v2;
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			done <= 1'b0;
			run <= 1'b0;
			r_scis_en <= 1'b0;
			md <= 4'd0;
			r_ra <= 7'd0;
			tl[0] <= 1'b0;
			tl[1] <= 1'b0;
			tl[2] <= 1'b0;
		end else begin
			done <= 1'b0;
			if (start_ctrl_b) begin
				run <= 1'b1;
				r_scis_en <= (smax != 32'd0);
				md <= mode;
				r_ra <= 7'd31;
			end else if (run_tl_b) begin
				if (|phase_oh[8:0])
					r_ra <= rrv;
				if (phase_oh_b[21]) begin
					for (k = 0; k < 3; k = k + 1) begin : g_tl_s
						/* Pure balanced 3-bit OR tree (<= 7 cmos3 vs 14) */
						tl[k] <= (!eA[k][23] && ((|eA[k][22:16]) | (|eA[k][15:8]) | (|eA[k][7:0]))) ||
						         (!(|eA[k][23:16]) && !(|eA[k][15:8]) && !(|eA[k][7:0]) &&
						          !eB[k][23] && ((|eB[k][22:16]) | (|eB[k][15:8]) | (|eB[k][7:0])));
					end
					if (empty) begin
						run <= 1'b0;
						done <= 1'b1;
					end
				end
				if (phase_oh[36]) begin
					run <= 1'b0;
					done <= 1'b1;
				end
			end
		end
	end

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_snap_v1 <= 1'b0;
			r_snap_v2 <= 1'b0;
			r_snap_v <= 1'b0;
		end else begin
			if (start_snap_b) begin
				r_snap_v1 <= 1'b0;
				r_snap_v2 <= 1'b0;
				r_snap_v <= 1'b0;
			end else if (run_snap_ctl) begin
				r_snap_v1 <= ph_10_1_v;
				r_snap_v2 <= rsv1_b0;
				r_snap_v <= rsv2_ctl;
			end
		end
	end

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_rd_snap <= 32'd0;
			r_snap_sign1 <= 1'b0;
			r_snap_ovf1 <= 1'b0;
			r_snap_zero_out1 <= 1'b0;
			r_snap_tsn1 <= 4'd0;
			r_snap_tsn2 <= 4'd0;
			r_s2_base <= 24'd0;
			r_s2_cin <= 1'b0;
			r_s2_sign <= 1'b0;
			r_s2_ovf <= 1'b0;
			r_s2_zero_out <= 1'b0;
			r_snap_tsn <= 4'd0;
		end else begin
			if (run_snap_dlo && ph_10_1_lo)
				r_rd_snap[15:0] <= r_rd[15:0];
			if (run_snap_dhi && ph_10_1_hi)
				r_rd_snap[31:16] <= r_rd[31:16];
			if (run_snap_ctl)
				r_snap_tsn1 <= tsn;
			if (run_s2_ctl && rsv1_b1) begin
				r_snap_sign1 <= r_rd_snap[31];
				r_snap_ovf1 <= s1_snap_ovf_w;
				r_snap_zero_out1 <= s1_snap_zout_w;
				r_snap_tsn2 <= r_snap_tsn1;
			end
			if (run_s2_b0 && rsv2_0)
				r_s2_base[7:0] <= s2_base[7:0];
			if (run_s2_b1 && rsv2_1)
				r_s2_base[15:8] <= s2_base[15:8];
			if (run_s2_b2 && rsv2_2)
				r_s2_base[23:16] <= s2_base[23:16];
			if (run_s2_ctl && rsv2_ctl) begin
				r_s2_cin <= s2_cin;
				r_s2_sign <= r_snap_sign1;
				r_s2_ovf <= r_snap_ovf1;
				r_s2_zero_out <= r_snap_zero_out1;
				r_snap_tsn <= r_snap_tsn2;
			end
		end
	end

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			r_is_tri_c  <= 64'd0;
			r_is_line_c <= 48'd0;
			r_is_pt_c   <= 4'd0;
		end else begin
			for (k = 0; k < 8; k = k + 1) begin : g_tri_c_ld
				if (start_mode_t[k]) r_is_tri_c[8 * k +: 8]  <= {8{mtri[k]}};
			end
			for (k = 0; k < 6; k = k + 1) begin : g_ln_c_ld
				if (start_mode_l[k]) r_is_line_c[8 * k +: 8] <= {8{mline[k]}};
			end
			if (start_mode_b)
				r_is_pt_c <= {4{mode_pt_w}};
		end
	end

	genvar g;
	generate
	for (g = 0; g < 3; g = g + 1) begin : g_edge
		assign edges[96 * g +: 96] = {eE[g], eB[g], eA[g]};
	end
	endgenerate
	assign bbox = {by1[15:0], bx1[15:0], by0[15:0], bx0[15:0]};

	wire unused_ok = &{1'b0, carry_unused, cmp_d_unused, ce_e[2:1], win_cmp_unused, ek,
		acc[49:48], ce0[8], ce1[8], bx0[17:16], by0[17:16], bx1[17:16],
		by1[17:16], adx[23], ady[23], q0x[7:0], q1x[7:0],
		q2x[7:0], q3x[7:0], q0y[7:0], q1y[7:0], q2y[7:0], q3y[7:0], s6[7:0], s7[7:0],
		s8[7:0], s9[7:0], eff_line_w[0], eff_point_s[0], unused_edge_exts, r_is_tri_c, r_is_line_c
`ifdef SYNTH_ASIC
		, unused_r_mp_b
`endif
	};
endmodule

`default_nettype wire
