/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_coarse: Coarse Tile Rasterizer (8x8 pixel tiles).
 *
 * Evaluates the three edge functions across 8x8 pixel tiles within the
 * triangle bounding box. Tests the conservative corner bounds of each tile:
 *   - Trivially Rejected (outside): if for any edge, all corners are further
 *     outside than the maximum MSAA sample offset reach, the entire 8x8 tile
 *     is outside the triangle and skipped in 1 cycle.
 *   - Trivially Accepted (inside): if for all three edges, all corners are
 *     further inside than the maximum MSAA sample offset (e_min > lowm), the
 *     entire 8x8 tile is inside the triangle (tile_trivial = 1).
 *   - Intersecting: boundary tile, evaluated by Fine Rasterizer.
 *
 * Arithmetic operators are prohibited; uses am_add / am_sub library modules.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rx_coarse (
	input wire clk,
	input wire rst_n,

	/* Setup triangle input */
	input wire tri_valid,
	output wire tri_ready,
	input wire [15:0] bbox_min_x,
	input wire [15:0] bbox_min_y,
	input wire [15:0] bbox_max_x,
	input wire [15:0] bbox_max_y,
	input wire [47:0] e0_a8,
	input wire [47:0] e0_b8,
	input wire [47:0] e0_er,
	input wire [47:0] e1_a8,
	input wire [47:0] e1_b8,
	input wire [47:0] e1_er,
	input wire [47:0] e2_a8,
	input wire [47:0] e2_b8,
	input wire [47:0] e2_er,
	input wire [47:0] lowm0,
	input wire [47:0] lowm1,
	input wire [47:0] lowm2,

	/* Coarse tile output to Fine Rasterizer */
	output reg tile_valid,
	input wire tile_ready,
	output reg [15:0] tile_x,
	output reg [15:0] tile_y,
	output wire [47:0] tile_e0,
	output wire [47:0] tile_e1,
	output wire [47:0] tile_e2,
	output reg tile_trivial,
	output reg tile_last,
	output wire busy
);
	localparam [2:0] S_IDLE = 3'd0, S_INIT = 3'd1, S_WALK = 3'd2, S_EVAL = 3'd3, S_STEP = 3'd4, S_WAIT = 3'd5, S_INIT2 = 3'd6;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire coarse_rst_sync;
	LUT1 #(.INIT(2'b01)) u_rst_coarse_inv (.I0(rst_n), .O(coarse_rst_sync));
	(* fsm_encoding = "none" *) reg [2:0] st;
	(* keep = "true", dont_touch = "true" *) reg [2:0] st_c0, st_c1, st_c2, st_c3;
	(* keep = "true", dont_touch = "true" *) wire we_txy_idle_l, we_txy_idle_h;
	LUT1 #(.INIT(2'h2)) u_wtxy_l (.I0((st_c1 == S_IDLE) && tri_valid), .O(we_txy_idle_l));
	LUT1 #(.INIT(2'h2)) u_wtxy_h (.I0((st_c1 == S_IDLE) && tri_valid), .O(we_txy_idle_h));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire coarse_rst_sync;
	LUT1 #(.INIT(2'b01)) u_rst_coarse_inv (.I0(rst_n), .O(coarse_rst_sync));
	(* fsm_encoding = "none" *) reg [2:0] st;
	(* keep = "true", dont_touch = "true" *) reg [2:0] st_c0, st_c1, st_c2, st_c3;
	(* keep = "true", dont_touch = "true" *) wire we_txy_idle_l, we_txy_idle_h;
	assign we_txy_idle_l = (st_c1 == S_IDLE) && tri_valid;
	assign we_txy_idle_h = (st_c1 == S_IDLE) && tri_valid;
`else
	(* keep = "true", dont_touch = "true" *) wire coarse_rst_sync = !rst_n;
	(* fsm_encoding = "none" *) reg [2:0] st;
`endif

	(* extract_reset = "no" *) reg [15:0] bx0, by0, bx1, by1;
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] a8 [0:2];
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] b8 [0:2];
	(* extract_reset = "no", keep = "true", dont_touch = "true" *) reg [11:0] a8_neg_rep [0:2];
	(* extract_reset = "no", keep = "true", dont_touch = "true" *) reg [11:0] b8_neg_rep [0:2];
	(* extract_reset = "no", equivalent_register_removal = "no" *) reg signed [47:0] ab8 [0:2];
	(* extract_reset = "no" *) reg signed [47:0] er [0:2];
	(* extract_reset = "no" *) reg signed [47:0] ec [0:2];
	(* extract_reset = "no" *) reg [47:0] lm [0:2];

	/* Pipeline registers for clean single-stage arithmetic */
	(* extract_reset = "no", max_fanout = 16 *) reg signed [47:0] r_emax [0:2];
	(* extract_reset = "no", max_fanout = 16 *) reg signed [47:0] r_emin [0:2];
	(* extract_reset = "no", max_fanout = 16 *) reg [15:0] r_tx_next, r_ty_next;
	(* extract_reset = "no", max_fanout = 16 *) reg signed [47:0] r_ec_next [0:2];
	(* extract_reset = "no", max_fanout = 16 *) reg signed [47:0] r_er_next [0:2];
	(* max_fanout = 16 *) reg r_row_end;
	(* max_fanout = 16 *) reg r_tile_outside;
	(* max_fanout = 16 *) reg r_is_trivial;
	(* max_fanout = 16 *) reg r_last_tile;

	assign tile_e0 = ec[0];
	assign tile_e1 = ec[1];
	assign tile_e2 = ec[2];

	/* Conservative corner delta offsets: max and min deltas per edge */
	(* extract_reset = "no" *) reg signed [47:0] d_max [0:2];
	(* extract_reset = "no" *) reg signed [47:0] d_min [0:2];

	/* Step deltas: 8 pixels in x (a8 <<< 3), 8 pixels in y (b8 <<< 3) */
	wire signed [47:0] step_x [0:2];
	wire signed [47:0] step_y [0:2];

	genvar ge, gnb;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_step
		assign step_x[ge] = a8[ge] <<< 3;
		assign step_y[ge] = b8[ge] <<< 3;
	end
	endgenerate

	/* Tile evaluation & stepping 48-bit adders: 4-bit CLA + speculative 24+24 under ASIC (<= 11 cmos3) */
	wire signed [47:0] e_max [0:2];
	wire signed [47:0] e_min [0:2];
	wire signed [47:0] ec_next [0:2];
	wire signed [47:0] er_next [0:2];
	wire signed [47:0] init_dmax [0:2];
	wire signed [47:0] init_dmin [0:2];
	wire signed [47:0] init_ab8_comb [0:2];
	wire [5:0] eval_carry_unused;
	wire [5:0] step_carry_unused;
	wire [5:0] init_carry_unused;
	wire [2:0] init_ab8_carry_unused;

	wire [47:0] dx_p [0:2];
	wire [47:0] dx_m [0:2];
	wire [47:0] dy_p [0:2];
	wire [47:0] dy_m [0:2];
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_init_delta_in
		for (gnb = 0; gnb < 12; gnb = gnb + 1) begin : g_dx_4
			assign dx_p[ge][4*gnb +: 4] = a8_neg_rep[ge][gnb] ? 4'd0 : step_x[ge][4*gnb +: 4];
			assign dx_m[ge][4*gnb +: 4] = a8_neg_rep[ge][gnb] ? step_x[ge][4*gnb +: 4] : 4'd0;
			assign dy_p[ge][4*gnb +: 4] = b8_neg_rep[ge][gnb] ? 4'd0 : step_y[ge][4*gnb +: 4];
			assign dy_m[ge][4*gnb +: 4] = b8_neg_rep[ge][gnb] ? step_y[ge][4*gnb +: 4] : 4'd0;
		end
	end
	endgenerate

`ifdef SYNTH_KINTEX7
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_eval
		am_add #(.W(48)) u_emax (.a(ec[ge]), .b(d_max[ge]), .ci(1'b0), .s(e_max[ge]),
			.co(eval_carry_unused[2 * ge]));
		am_add #(.W(48)) u_emin (.a(ec[ge]), .b(d_min[ge]), .ci(1'b0), .s(e_min[ge]),
			.co(eval_carry_unused[2 * ge + 1]));
		am_add #(.W(48)) u_ec_next (.a(ec[ge]), .b(step_x[ge]), .ci(1'b0), .s(ec_next[ge]),
			.co(step_carry_unused[2 * ge]));
		am_add #(.W(48)) u_er_next (.a(er[ge]), .b(step_y[ge]), .ci(1'b0), .s(er_next[ge]),
			.co(step_carry_unused[2 * ge + 1]));
		am_add #(.W(48)) u_dmax (.a(dx_p[ge]), .b(dy_p[ge]), .ci(1'b0), .s(init_dmax[ge]),
			.co(init_carry_unused[2 * ge]));
		am_add #(.W(48)) u_dmin (.a(dx_m[ge]), .b(dy_m[ge]), .ci(1'b0), .s(init_dmin[ge]),
			.co(init_carry_unused[2 * ge + 1]));
		wire signed [47:0] ge_a = (ge == 0) ? e0_a8 : (ge == 1) ? e1_a8 : e2_a8;
		wire signed [47:0] ge_b = (ge == 0) ? e0_b8 : (ge == 1) ? e1_b8 : e2_b8;
		am_add #(.W(48)) u_ab8 (.a(ge_a), .b(ge_b), .ci(1'b0), .s(init_ab8_comb[ge]),
			.co(init_ab8_carry_unused[ge]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_eval
		am_add #(.W(48)) u_emax (.a(ec[ge]), .b(d_max[ge]), .ci(1'b0), .s(e_max[ge]),
			.co(eval_carry_unused[2 * ge]));
		am_add #(.W(48)) u_emin (.a(ec[ge]), .b(d_min[ge]), .ci(1'b0), .s(e_min[ge]),
			.co(eval_carry_unused[2 * ge + 1]));
		am_add #(.W(48)) u_ec_next (.a(ec[ge]), .b(step_x[ge]), .ci(1'b0), .s(ec_next[ge]),
			.co(step_carry_unused[2 * ge]));
		am_add #(.W(48)) u_er_next (.a(er[ge]), .b(step_y[ge]), .ci(1'b0), .s(er_next[ge]),
			.co(step_carry_unused[2 * ge + 1]));
		am_add #(.W(48)) u_dmax (.a(dx_p[ge]), .b(dy_p[ge]), .ci(1'b0), .s(init_dmax[ge]),
			.co(init_carry_unused[2 * ge]));
		am_add #(.W(48)) u_dmin (.a(dx_m[ge]), .b(dy_m[ge]), .ci(1'b0), .s(init_dmin[ge]),
			.co(init_carry_unused[2 * ge + 1]));
		wire signed [47:0] ge_a = (ge == 0) ? e0_a8 : (ge == 1) ? e1_a8 : e2_a8;
		wire signed [47:0] ge_b = (ge == 0) ? e0_b8 : (ge == 1) ? e1_b8 : e2_b8;
		am_add #(.W(48)) u_ab8 (.a(ge_a), .b(ge_b), .ci(1'b0), .s(init_ab8_comb[ge]),
			.co(init_ab8_carry_unused[ge]));
	end
	endgenerate
`else
	(* extract_reset = "no" *) reg [47:0] r_dx_p [0:2];
	(* extract_reset = "no" *) reg [47:0] r_dx_m [0:2];
	(* extract_reset = "no" *) reg [47:0] r_dy_p [0:2];
	(* extract_reset = "no" *) reg [47:0] r_dy_m [0:2];
	wire [47:0] cadd_a [0:20];
	wire [47:0] cadd_b [0:20];
	wire [47:0] cadd_s [0:20];
	wire [20:0] cadd_co_un;
	wire [2:0] ab_co_un;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_cadd_map
		wire signed [47:0] ge_a = (ge == 0) ? e0_a8 : (ge == 1) ? e1_a8 : e2_a8;
		wire signed [47:0] ge_b = (ge == 0) ? e0_b8 : (ge == 1) ? e1_b8 : e2_b8;
		assign cadd_a[ge]      = ec[ge];     assign cadd_b[ge]      = d_max[ge];  assign e_max[ge]         = cadd_s[ge];
		assign cadd_a[3 + ge]  = ec[ge];     assign cadd_b[3 + ge]  = d_min[ge];  assign e_min[ge]         = cadd_s[3 + ge];
		assign cadd_a[6 + ge]  = ec[ge];     assign cadd_b[6 + ge]  = step_x[ge]; assign ec_next[ge]       = cadd_s[6 + ge];
		assign cadd_a[9 + ge]  = er[ge];     assign cadd_b[9 + ge]  = step_y[ge]; assign er_next[ge]       = cadd_s[9 + ge];
		assign cadd_a[12 + ge] = r_dx_p[ge]; assign cadd_b[12 + ge] = r_dy_p[ge]; assign init_dmax[ge]     = cadd_s[12 + ge];
		assign cadd_a[15 + ge] = r_dx_m[ge]; assign cadd_b[15 + ge] = r_dy_m[ge]; assign init_dmin[ge]     = cadd_s[15 + ge];

		/* init_ab8_comb: 3-bit CLA + 9+9+6 over bits [31:8] (<= 9 cmos3) */
		wire [23:0] ab_a = ge_a[31:8];
		wire [23:0] ab_b = ge_b[31:8];
		wire [23:0] ab_g = ab_a & ab_b;
		wire [23:0] ab_p = ab_a | ab_b;
		wire [7:0]  ab_g3, ab_p3, ab_c0, ab_c1;
		wire [2:0]  ab_s0 [0:7];
		wire [2:0]  ab_s1 [0:7];
		genvar gab;
		for (gab = 0; gab < 8; gab = gab + 1) begin : g_ab3
			wire [2:0] gi = ab_g[3*gab +: 3];
			wire [2:0] pi = ab_p[3*gab +: 3];
			assign ab_p3[gab] = &pi;
			assign ab_g3[gab] = gi[2] | (pi[2] & gi[1]) | (pi[2] & pi[1] & gi[0]);
			am_add #(.W(3)) u_ab_s0 (.a(ab_a[3*gab +: 3]), .b(ab_b[3*gab +: 3]), .ci(1'b0), .s(ab_s0[gab]), .co(ab_c0[gab]));
			am_add #(.W(3)) u_ab_s1 (.a(ab_a[3*gab +: 3]), .b(ab_b[3*gab +: 3]), .ci(1'b1), .s(ab_s1[gab]), .co(ab_c1[gab]));
		end
		wire ab_p9_0 = &ab_p3[2:0];
		wire ab_g9_0 = ab_g3[2] | (ab_p3[2] & ab_g3[1]) | (ab_p3[2] & ab_p3[1] & ab_g3[0]);
		wire ab_p9_1 = &ab_p3[5:3];
		wire ab_g9_1 = ab_g3[5] | (ab_p3[5] & ab_g3[4]) | (ab_p3[5] & ab_p3[4] & ab_g3[3]);
		wire ab_p6_2 = &ab_p3[7:6];
		wire ab_g6_2 = ab_g3[7] | (ab_p3[7] & ab_g3[6]);
		wire [7:0] ab_ci;
		wire ab_c18 = ab_g9_1 | (ab_p9_1 & ab_g9_0);
		assign ab_ci[0] = 1'b0;
		assign ab_ci[1] = ab_g3[0];
		assign ab_ci[2] = ab_g3[1] | (ab_p3[1] & ab_g3[0]);
		assign ab_ci[3] = ab_g9_0;
		assign ab_ci[4] = ab_g3[3] | (ab_p3[3] & ab_g9_0);
		assign ab_ci[5] = ab_g3[4] | (ab_p3[4] & ab_g3[3]) | (ab_p3[4] & ab_p3[3] & ab_g9_0);
		assign ab_ci[6] = ab_c18;
		assign ab_ci[7] = ab_g3[6] | (ab_p3[6] & ab_c18);
		wire [23:0] ab_s24;
		for (gab = 0; gab < 8; gab = gab + 1) begin : g_ab_m
			assign ab_s24[3*gab +: 3] = ab_ci[gab] ? ab_s1[gab] : ab_s0[gab];
		end
		wire ab_c24 = ab_g6_2 | (ab_p6_2 & ab_g9_1) | (ab_p6_2 & ab_p9_1 & ab_g9_0);
		wire ab_hi_b = ge_a[32] ^ ge_b[32] ^ ab_c24;
		assign init_ab8_comb[ge] = {{16{ab_hi_b}}, ab_s24, 8'd0};
		assign ab_co_un[ge] = (&ab_c0) & (&ab_c1);
	end
	for (ge = 0; ge < 18; ge = ge + 1) begin : g_cadd48
		wire [47:0] ca = cadd_a[ge];
		wire [47:0] cb = cadd_b[ge];
		wire [47:0] cg = ca & cb;
		wire [47:0] cp = ca | cb;
		wire [23:0] g2, p2;
		wire [1:0]  s2_0 [0:23];
		wire [1:0]  s2_1 [0:23];
		wire [23:0] u2_0, u2_1;
		genvar gb2;
		for (gb2 = 0; gb2 < 24; gb2 = gb2 + 1) begin : g_b2
			assign p2[gb2] = cp[2*gb2+1] & cp[2*gb2];
			assign g2[gb2] = cg[2*gb2+1] | (cp[2*gb2+1] & cg[2*gb2]);
			am_add #(.W(2)) u_s0 (.a(ca[2*gb2 +: 2]), .b(cb[2*gb2 +: 2]), .ci(1'b0), .s(s2_0[gb2]), .co(u2_0[gb2]));
			am_add #(.W(2)) u_s1 (.a(ca[2*gb2 +: 2]), .b(cb[2*gb2 +: 2]), .ci(1'b1), .s(s2_1[gb2]), .co(u2_1[gb2]));
		end
		wire [7:0] g6, p6;
		for (gb2 = 0; gb2 < 8; gb2 = gb2 + 1) begin : g_b6
			assign p6[gb2] = &p2[3*gb2 +: 3];
			assign g6[gb2] = g2[3*gb2+2] | (p2[3*gb2+2] & g2[3*gb2+1]) | (p2[3*gb2+2] & p2[3*gb2+1] & g2[3*gb2]);
		end
		wire [3:0] g12, p12;
		for (gb2 = 0; gb2 < 4; gb2 = gb2 + 1) begin : g_b12
			assign p12[gb2] = p6[2*gb2+1] & p6[2*gb2];
			assign g12[gb2] = g6[2*gb2+1] | (p6[2*gb2+1] & g6[2*gb2]);
		end
		wire c12 = g12[0];
		wire c24 = g12[1] | (p12[1] & g12[0]);

		/* [11:0] direct from g2/g6 */
		wire [5:0] cb_0_11;
		assign cb_0_11[0] = 1'b0;
		assign cb_0_11[1] = g2[0];
		assign cb_0_11[2] = g2[1] | (p2[1] & g2[0]);
		assign cb_0_11[3] = g6[0];
		assign cb_0_11[4] = g2[3] | (p2[3] & g6[0]);
		assign cb_0_11[5] = g2[4] | (p2[4] & g2[3]) | (p2[4] & p2[3] & g6[0]);
		for (gb2 = 0; gb2 < 6; gb2 = gb2 + 1) begin : g_s0
			assign cadd_s[ge][2*gb2 +: 2] = cb_0_11[gb2] ? s2_1[gb2] : s2_0[gb2];
		end

		/* [23:12] speculative on c12 (<= 10 cmos3) */
		wire [11:6] cb_m0, cb_m1;
		assign cb_m0[6]  = 1'b0;
		assign cb_m0[7]  = g2[6];
		assign cb_m0[8]  = g2[7] | (p2[7] & g2[6]);
		assign cb_m0[9]  = g6[2];
		assign cb_m0[10] = g2[9]  | (p2[9]  & g6[2]);
		assign cb_m0[11] = g2[10] | (p2[10] & g2[9]) | (p2[10] & p2[9] & g6[2]);
		assign cb_m1[6]  = 1'b1;
		assign cb_m1[7]  = g2[6] | p2[6];
		assign cb_m1[8]  = g2[7] | (p2[7] & (g2[6] | p2[6]));
		assign cb_m1[9]  = g6[2] | p6[2];
		assign cb_m1[10] = g2[9]  | (p2[9]  & (g6[2] | p6[2]));
		assign cb_m1[11] = g2[10] | (p2[10] & g2[9]) | (p2[10] & p2[9] & (g6[2] | p6[2]));
		wire [23:12] mid_0, mid_1;
		for (gb2 = 6; gb2 < 12; gb2 = gb2 + 1) begin : g_sm
			assign mid_0[2*gb2 +: 2] = cb_m0[gb2] ? s2_1[gb2] : s2_0[gb2];
			assign mid_1[2*gb2 +: 2] = cb_m1[gb2] ? s2_1[gb2] : s2_0[gb2];
		end
		assign cadd_s[ge][23:12] = c12 ? mid_1 : mid_0;

		/* [47:24] speculative on c24 = 0 and c24 = 1 (<= 12 cmos3) */
		wire [23:12] cb_hi0, cb_hi1;
		wire c36_0 = g12[2];
		wire c36_1 = g12[2] | p12[2];
		assign cb_hi0[12] = 1'b0;
		assign cb_hi0[13] = g2[12];
		assign cb_hi0[14] = g2[13] | (p2[13] & g2[12]);
		assign cb_hi0[15] = g6[4];
		assign cb_hi0[16] = g2[15] | (p2[15] & g6[4]);
		assign cb_hi0[17] = g2[16] | (p2[16] & g2[15]) | (p2[16] & p2[15] & g6[4]);
		assign cb_hi0[18] = c36_0;
		assign cb_hi0[19] = g2[18] | (p2[18] & c36_0);
		assign cb_hi0[20] = g2[19] | (p2[19] & g2[18]) | (p2[19] & p2[18] & c36_0);
		wire g22_21 = g2[22] | (p2[22] & g2[21]);
		wire p22_21 = p2[22] & p2[21];
		wire c42_0 = g6[6]  | (p6[6]  & c36_0);
		assign cb_hi0[21] = c42_0;
		assign cb_hi0[22] = g2[21] | (p2[21] & c42_0);
		assign cb_hi0[23] = g22_21 | (p22_21 & c42_0);

		wire c30_1 = g6[4] | p6[4];
		assign cb_hi1[12] = 1'b1;
		assign cb_hi1[13] = g2[12] | p2[12];
		assign cb_hi1[14] = g2[13] | (p2[13] & (g2[12] | p2[12]));
		assign cb_hi1[15] = c30_1;
		assign cb_hi1[16] = g2[15] | (p2[15] & c30_1);
		assign cb_hi1[17] = g2[16] | (p2[16] & g2[15]) | (p2[16] & p2[15] & c30_1);
		assign cb_hi1[18] = c36_1;
		assign cb_hi1[19] = g2[18] | (p2[18] & c36_1);
		assign cb_hi1[20] = g2[19] | (p2[19] & g2[18]) | (p2[19] & p2[18] & c36_1);
		wire c42_1 = g6[6]  | (p6[6]  & c36_1);
		assign cb_hi1[21] = c42_1;
		assign cb_hi1[22] = g2[21] | (p2[21] & c42_1);
		assign cb_hi1[23] = g22_21 | (p22_21 & c42_1);

		wire [47:24] hi_0, hi_1;
		for (gb2 = 12; gb2 < 24; gb2 = gb2 + 1) begin : g_sh
			assign hi_0[2*gb2 +: 2] = cb_hi0[gb2] ? s2_1[gb2] : s2_0[gb2];
			assign hi_1[2*gb2 +: 2] = cb_hi1[gb2] ? s2_1[gb2] : s2_0[gb2];
		end
		assign cadd_s[ge][47:24] = c24 ? hi_1 : hi_0;
		assign cadd_co_un[ge]    = (&u2_0) & (&u2_1) & g12[3] & p12[3];
	end
	endgenerate
	assign eval_carry_unused    = {6{&cadd_co_un}};
	assign step_carry_unused    = 6'd0;
	assign init_carry_unused    = 6'd0;
	assign init_ab8_carry_unused = ab_co_un;
`endif

	/* Any edge where E_max is further outside than lowm reach means all sample points are outside */
	wire [2:0] edge_outside;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_edge_out
		wire [47:0] eo_or = r_emax[ge] | lm[ge];
		wire [15:0] eo_and3;
		genvar geo;
		for (geo = 0; geo < 16; geo = geo + 1) begin : g_eo3
			assign eo_and3[geo] = &eo_or[3*geo +: 3];
		end
		wire [4:0] eo_and9;
		for (geo = 0; geo < 5; geo = geo + 1) begin : g_eo9
			assign eo_and9[geo] = &eo_and3[3*geo +: 3];
		end
		wire eo_27 = &eo_and9[2:0];
		wire eo_21 = eo_and9[4] & eo_and9[3] & eo_and3[15];
		assign edge_outside[ge] = r_emax[ge][47] && !(eo_27 && eo_21);
	end
	endgenerate
	wire tile_outside = |edge_outside;

	/* All edges where E_min > lowm means all sample points in tile are unconditionally inside */
	wire [2:0] inside_bo;
`ifdef SYNTH_KINTEX7
	wire [143:0] inside_d_unused;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_inside_cmp
		am_sub #(.W(48)) u_ins (.a(lm[ge]), .b(r_emin[ge]), .bi(1'b0), .d(inside_d_unused[48 * ge +: 48]), .bo(inside_bo[ge]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [143:0] inside_d_unused;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_inside_cmp
		am_sub #(.W(48)) u_ins (.a(lm[ge]), .b(r_emin[ge]), .bi(1'b0), .d(inside_d_unused[48 * ge +: 48]), .bo(inside_bo[ge]));
	end
	endgenerate
`else
	wire inside_d_unused = 1'b0;
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_inside_cmp_lut
		wire [47:0] cg = (~lm[ge]) & r_emin[ge];
		wire [47:0] cp = (~lm[ge]) | r_emin[ge];
		wire [23:0] g2, p2;
		genvar g2c;
		for (g2c = 0; g2c < 24; g2c = g2c + 1) begin : g_c2
			assign p2[g2c] = cp[2*g2c+1] & cp[2*g2c];
			assign g2[g2c] = cg[2*g2c+1] | (cp[2*g2c+1] & cg[2*g2c]);
		end
		/* 2 -> 4 -> 8 -> 16 -> 48 balanced binary tree (<= 10 cmos3) */
		wire [11:0] g4, p4;
		for (g2c = 0; g2c < 12; g2c = g2c + 1) begin : g_c4
			assign p4[g2c] = p2[2*g2c+1] & p2[2*g2c];
			assign g4[g2c] = g2[2*g2c+1] | (p2[2*g2c+1] & g2[2*g2c]);
		end
		wire [5:0] g8, p8;
		for (g2c = 0; g2c < 6; g2c = g2c + 1) begin : g_c8
			assign p8[g2c] = p4[2*g2c+1] & p4[2*g2c];
			assign g8[g2c] = g4[2*g2c+1] | (p4[2*g2c+1] & g4[2*g2c]);
		end
		wire [2:0] g16, p16;
		for (g2c = 0; g2c < 3; g2c = g2c + 1) begin : g_c16
			assign p16[g2c] = p8[2*g2c+1] & p8[2*g2c];
			assign g16[g2c] = g8[2*g2c+1] | (p8[2*g2c+1] & g8[2*g2c]);
		end
		wire p32_hi = p16[2] & p16[1];
		wire g32_hi = g16[2] | (p16[2] & g16[1]);
		assign inside_bo[ge] = g32_hi | (p32_hi & g16[0]);
		wire unused_ins_p = p16[0];
	end
	endgenerate
`endif
	wire [2:0] in_ok = {lm[2][47] || inside_bo[2], lm[1][47] || inside_bo[1], lm[0][47] || inside_bo[0]};
	wire all_inside = !r_emin[0][47] && !r_emin[1][47] && !r_emin[2][47] && (&in_ok);

	/* Stepping arithmetic for coordinates */
	wire [15:0] tx_next;
	wire [15:0] ty_next;
	wire [1:0] coord_carry_unused;
	am_add #(.W(16)) u_tx_next (.a(tile_x), .b(16'd8), .ci(1'b0), .s(tx_next), .co(coord_carry_unused[0]));
	am_add #(.W(16)) u_ty_next (.a(tile_y), .b(16'd8), .ci(1'b0), .s(ty_next), .co(coord_carry_unused[1]));

	wire row_end_lt, last_tile_lt, tx_lt_bx0, tx_gt_bx1, ty_lt_by0, ty_gt_by1;
`ifdef SYNTH_KINTEX7
	wire [15:0] cmp_d0, cmp_d1, cmp_d2, cmp_d3, cmp_d4, cmp_d5;
	am_sub #(.W(16)) u_re_cmp  (.a(r_tx_next), .b(bx1),       .bi(1'b0), .d(cmp_d0), .bo(row_end_lt));
	am_sub #(.W(16)) u_lt_cmp  (.a(r_ty_next), .b(by1),       .bi(1'b0), .d(cmp_d1), .bo(last_tile_lt));
	am_sub #(.W(16)) u_tbx0    (.a(tile_x),    .b(bx0),       .bi(1'b0), .d(cmp_d2), .bo(tx_lt_bx0));
	am_sub #(.W(16)) u_tbx1    (.a(bx1),       .b(r_tx_next), .bi(1'b0), .d(cmp_d3), .bo(tx_gt_bx1));
	am_sub #(.W(16)) u_tby0    (.a(tile_y),    .b(by0),       .bi(1'b0), .d(cmp_d4), .bo(ty_lt_by0));
	am_sub #(.W(16)) u_tby1    (.a(by1),       .b(r_ty_next), .bi(1'b0), .d(cmp_d5), .bo(ty_gt_by1));
	wire unused_cmp_d = &{1'b0, cmp_d0, cmp_d1, cmp_d2, cmp_d3, cmp_d4, cmp_d5};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [15:0] cmp_d0, cmp_d1, cmp_d2, cmp_d3, cmp_d4, cmp_d5;
	am_sub #(.W(16)) u_re_cmp  (.a(r_tx_next), .b(bx1),       .bi(1'b0), .d(cmp_d0), .bo(row_end_lt));
	am_sub #(.W(16)) u_lt_cmp  (.a(r_ty_next), .b(by1),       .bi(1'b0), .d(cmp_d1), .bo(last_tile_lt));
	am_sub #(.W(16)) u_tbx0    (.a(tile_x),    .b(bx0),       .bi(1'b0), .d(cmp_d2), .bo(tx_lt_bx0));
	am_sub #(.W(16)) u_tbx1    (.a(bx1),       .b(r_tx_next), .bi(1'b0), .d(cmp_d3), .bo(tx_gt_bx1));
	am_sub #(.W(16)) u_tby0    (.a(tile_y),    .b(by0),       .bi(1'b0), .d(cmp_d4), .bo(ty_lt_by0));
	am_sub #(.W(16)) u_tby1    (.a(by1),       .b(r_ty_next), .bi(1'b0), .d(cmp_d5), .bo(ty_gt_by1));
	wire unused_cmp_d = &{1'b0, cmp_d0, cmp_d1, cmp_d2, cmp_d3, cmp_d4, cmp_d5};
`else
	function automatic lt16_u;
		input [15:0] a, b;
		reg [3:0] lt4, eq4;
		reg [1:0] lt8, eq8;
		integer k4;
		begin
			for (k4 = 0; k4 < 4; k4 = k4 + 1) begin
				lt4[k4] = (a[4*k4 +: 4] < b[4*k4 +: 4]);
				eq4[k4] = (a[4*k4 +: 4] == b[4*k4 +: 4]);
			end
			lt8[0] = lt4[1] | (eq4[1] & lt4[0]);
			eq8[0] = eq4[1] & eq4[0];
			lt8[1] = lt4[3] | (eq4[3] & lt4[2]);
			eq8[1] = eq4[3] & eq4[2];
			lt16_u = lt8[1] | (eq8[1] & lt8[0]);
		end
	endfunction
	assign row_end_lt   = lt16_u(r_tx_next, bx1);
	assign last_tile_lt = lt16_u(r_ty_next, by1);
	assign tx_lt_bx0    = lt16_u(tile_x,    bx0);
	assign tx_gt_bx1    = lt16_u(bx1,       r_tx_next);
	assign ty_lt_by0    = lt16_u(tile_y,    by0);
	assign ty_gt_by1    = lt16_u(by1,       r_ty_next);
	wire unused_cmp_d = 1'b0;
`endif
	wire row_end = !row_end_lt;
	wire last_tile = row_end && !last_tile_lt;

	/* Check if tile is fully within bounding box (no scissor/bbox clipping of corners) */
	wire tile_in_bbox = !tx_lt_bx0 && !tx_gt_bx1 && !ty_lt_by0 && !ty_gt_by1;
	wire trivial_fast = !r_emin[0][47] && !r_emin[1][47] && !r_emin[2][47] && tile_in_bbox;
	wire is_trivial = trivial_fast && (&in_ok);

	/* Stepping arithmetic & delta prep handled above via g_cadd48 under !SYNTH_KINTEX7 */

`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire [8:0] bx0_0_leaf;
	(* keep = "true", dont_touch = "true" *) wire [8:0] by0_0_leaf;
	genvar g_b0;
	generate
		for (g_b0 = 0; g_b0 < 9; g_b0 = g_b0 + 1) begin : g_b0_buf
			LUT1 #(.INIT(2'h2)) u_bx0_0 (.I0(bx0[0]), .O(bx0_0_leaf[g_b0]));
			LUT1 #(.INIT(2'h2)) u_by0_0 (.I0(by0[0]), .O(by0_0_leaf[g_b0]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire [8:0] bx0_0_leaf;
	(* keep = "true", dont_touch = "true" *) wire [8:0] by0_0_leaf;
	genvar g_b0;
	generate
		for (g_b0 = 0; g_b0 < 9; g_b0 = g_b0 + 1) begin : g_b0_buf
			assign bx0_0_leaf[g_b0] = bx0[0];
			assign by0_0_leaf[g_b0] = by0[0];
		end
	endgenerate
`endif

	wire signed [47:0] init_e_sub [0:2];
	wire signed [47:0] init_er [0:2];
	wire [2:0] init_er_carry_unused;
	wire [15:0] init_lo_d [0:2];
	wire [2:0]  init_lo_bo;
	wire [15:0] init_m0_w [0:2];
	wire [15:0] init_m1_w [0:2];
	wire [2:0]  init_mbo0_w, init_mbo1_w;
	wire [15:0] init_h0_w [0:2];
	wire [15:0] init_h1_w [0:2];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	(* extract_reset = "no" *) reg [47:0] r_init_e_sub [0:2];
	(* extract_reset = "no" *) reg [15:0] r_init_er_lo [0:2];
	(* extract_reset = "no" *) reg        r_init_bo16  [0:2];
	(* extract_reset = "no" *) reg [15:0] r_init_m0    [0:2];
	(* extract_reset = "no" *) reg [15:0] r_init_m1    [0:2];
	(* extract_reset = "no" *) reg        r_init_mbo0  [0:2];
	(* extract_reset = "no" *) reg        r_init_mbo1  [0:2];
	(* extract_reset = "no" *) reg [15:0] r_init_h0    [0:2];
	(* extract_reset = "no" *) reg [15:0] r_init_h1    [0:2];
`endif
	generate
	for (ge = 0; ge < 3; ge = ge + 1) begin : g_init_er
`ifdef SYNTH_KINTEX7
		wire [47:0] init_e_sub_comb;
		genvar gsk;
		for (gsk = 0; gsk < 3; gsk = gsk + 1) begin : g_sub_sl
			wire bx0_s = bx0_0_leaf[ge * 3 + gsk];
			wire by0_s = by0_0_leaf[ge * 3 + gsk];
			assign init_e_sub_comb[gsk*16 +: 16] = (bx0_s && by0_s) ? ab8[ge][gsk*16 +: 16] :
			                                       bx0_s ? a8[ge][gsk*16 +: 16] :
			                                       by0_s ? b8[ge][gsk*16 +: 16] : 16'd0;
		end
		assign init_e_sub[ge] = init_e_sub_comb;
		assign init_lo_d[ge] = 16'd0;
		assign init_lo_bo[ge] = 1'b0;
		assign init_m0_w[ge] = 16'd0;
		assign init_m1_w[ge] = 16'd0;
		assign init_mbo0_w[ge] = 1'b0;
		assign init_mbo1_w[ge] = 1'b0;
		assign init_h0_w[ge]   = 16'd0;
		assign init_h1_w[ge]   = 16'd0;
		am_sub #(.W(48)) u_init_er (.a(er[ge]), .b(init_e_sub[ge]), .bi(1'b0), .d(init_er[ge]),
			.bo(init_er_carry_unused[ge]));
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [47:0] init_e_sub_comb;
		genvar gsk;
		for (gsk = 0; gsk < 3; gsk = gsk + 1) begin : g_sub_sl
			wire bx0_s = bx0_0_leaf[ge * 3 + gsk];
			wire by0_s = by0_0_leaf[ge * 3 + gsk];
			assign init_e_sub_comb[gsk*16 +: 16] = (bx0_s && by0_s) ? ab8[ge][gsk*16 +: 16] :
			                                       bx0_s ? a8[ge][gsk*16 +: 16] :
			                                       by0_s ? b8[ge][gsk*16 +: 16] : 16'd0;
		end
		assign init_e_sub[ge] = init_e_sub_comb;
		assign init_lo_d[ge] = 16'd0;
		assign init_lo_bo[ge] = 1'b0;
		assign init_m0_w[ge] = 16'd0;
		assign init_m1_w[ge] = 16'd0;
		assign init_mbo0_w[ge] = 1'b0;
		assign init_mbo1_w[ge] = 1'b0;
		assign init_h0_w[ge]   = 16'd0;
		assign init_h1_w[ge]   = 16'd0;
		am_sub #(.W(48)) u_init_er (.a(er[ge]), .b(init_e_sub[ge]), .bi(1'b0), .d(init_er[ge]),
			.bo(init_er_carry_unused[ge]));
`else
		assign init_e_sub[ge] = r_init_e_sub[ge];
		/* Fast 4+4+4+4 CLA for [15:0] (<= 10 cmos3) */
		wire [15:0] il_a = er[ge][15:0];
		wire [15:0] il_b = init_e_sub[ge][15:0];
		wire [15:0] il_bg = (~il_a) & il_b;
		wire [15:0] il_bp = (~il_a) | il_b;
		wire [7:0] il_g2, il_p2, il_u0, il_u1;
		wire [1:0] il_d0 [0:7];
		wire [1:0] il_d1 [0:7];
		genvar gl2;
		for (gl2 = 0; gl2 < 8; gl2 = gl2 + 1) begin : g_il_2b
			assign il_p2[gl2] = il_bp[2*gl2+1] & il_bp[2*gl2];
			assign il_g2[gl2] = il_bg[2*gl2+1] | (il_bp[2*gl2+1] & il_bg[2*gl2]);
			am_sub #(.W(2)) u_il_d0 (.a(il_a[2*gl2 +: 2]), .b(il_b[2*gl2 +: 2]), .bi(1'b0), .d(il_d0[gl2]), .bo(il_u0[gl2]));
			am_sub #(.W(2)) u_il_d1 (.a(il_a[2*gl2 +: 2]), .b(il_b[2*gl2 +: 2]), .bi(1'b1), .d(il_d1[gl2]), .bo(il_u1[gl2]));
		end
		wire [3:0] il_g4, il_p4;
		for (gl2 = 0; gl2 < 4; gl2 = gl2 + 1) begin : g_il_4b
			assign il_p4[gl2] = il_p2[2*gl2+1] & il_p2[2*gl2];
			assign il_g4[gl2] = il_g2[2*gl2+1] | (il_p2[2*gl2+1] & il_g2[2*gl2]);
		end
		wire il_bo8 = il_g4[1] | (il_p4[1] & il_g4[0]);
		wire [7:0] il_bo;
		assign il_bo[0] = 1'b0;
		assign il_bo[1] = il_g2[0];
		assign il_bo[2] = il_g4[0];
		assign il_bo[3] = il_g2[2] | (il_p2[2] & il_g4[0]);
		assign il_bo[4] = 1'b0;
		assign il_bo[5] = il_g2[4];
		assign il_bo[6] = il_g4[2];
		assign il_bo[7] = il_g2[6] | (il_p2[6] & il_g4[2]);
		wire [7:0] il_lo, il_hi0, il_hi1;
		for (gl2 = 0; gl2 < 4; gl2 = gl2 + 1) begin : g_il_ml
			assign il_lo[2*gl2 +: 2] = il_bo[gl2] ? il_d1[gl2] : il_d0[gl2];
		end
		for (gl2 = 4; gl2 < 8; gl2 = gl2 + 1) begin : g_il_mh
			assign il_hi0[2*(gl2-4) +: 2] = il_bo[gl2] ? il_d1[gl2] : il_d0[gl2];
			wire b1_i = (gl2 == 4) ? 1'b1 : (gl2 == 5) ? (il_g2[4] | il_p2[4]) : (gl2 == 6) ? (il_g4[2] | il_p4[2]) : (il_g2[6] | (il_p2[6] & (il_g4[2] | il_p4[2])));
			assign il_hi1[2*(gl2-4) +: 2] = b1_i ? il_d1[gl2] : il_d0[gl2];
		end
		assign init_lo_d[ge]  = {il_bo8 ? il_hi1 : il_hi0, il_lo};
		assign init_lo_bo[ge] = (il_g4[3] | (il_p4[3] & il_g4[2]) | (il_p4[3] & il_p4[2] & il_bo8)) | (1'b0 & ((&il_u0) & (&il_u1)));

		/* Fast 4+4+4+4 CLA/select for [31:16] with bi=0 and bi=1 (<= 10 cmos3) */
		wire [15:0] im_a = er[ge][31:16];
		wire [15:0] im_b = init_e_sub[ge][31:16];
		wire [15:0] im_bg = (~im_a) & im_b;
		wire [15:0] im_bp = (~im_a) | im_b;
		wire [7:0] im_g2, im_p2, im_u0, im_u1;
		wire [1:0] im_d0 [0:7];
		wire [1:0] im_d1 [0:7];
		genvar gm2;
		for (gm2 = 0; gm2 < 8; gm2 = gm2 + 1) begin : g_im_2b
			assign im_p2[gm2] = im_bp[2*gm2+1] & im_bp[2*gm2];
			assign im_g2[gm2] = im_bg[2*gm2+1] | (im_bp[2*gm2+1] & im_bg[2*gm2]);
			am_sub #(.W(2)) u_im_d0 (.a(im_a[2*gm2 +: 2]), .b(im_b[2*gm2 +: 2]), .bi(1'b0), .d(im_d0[gm2]), .bo(im_u0[gm2]));
			am_sub #(.W(2)) u_im_d1 (.a(im_a[2*gm2 +: 2]), .b(im_b[2*gm2 +: 2]), .bi(1'b1), .d(im_d1[gm2]), .bo(im_u1[gm2]));
		end
		wire [3:0] im_g4, im_p4;
		for (gm2 = 0; gm2 < 4; gm2 = gm2 + 1) begin : g_im_4b
			assign im_p4[gm2] = im_p2[2*gm2+1] & im_p2[2*gm2];
			assign im_g4[gm2] = im_g2[2*gm2+1] | (im_p2[2*gm2+1] & im_g2[2*gm2]);
		end
		wire im_b8_0 = im_g4[1] | (im_p4[1] & im_g4[0]);
		wire im_b8_1 = im_g4[1] | (im_p4[1] & (im_g4[0] | im_p4[0]));
		wire im_g8_1 = im_g4[3] | (im_p4[3] & im_g4[2]);
		wire im_p8_1 = im_p4[3] & im_p4[2];
		wire im_b16_0 = im_g8_1 | (im_p8_1 & im_b8_0);
		wire im_b16_1 = im_b16_0 | (im_p8_1 & im_p4[1] & im_p4[0]);
		wire [7:0] im_bo0, im_bo1;
		assign im_bo0[0] = 1'b0;
		assign im_bo0[1] = im_g2[0];
		assign im_bo0[2] = im_g4[0];
		assign im_bo0[3] = im_g2[2] | (im_p2[2] & im_g4[0]);
		assign im_bo0[4] = 1'b0;
		assign im_bo0[5] = im_g2[4];
		assign im_bo0[6] = im_g4[2];
		assign im_bo0[7] = im_g2[6] | (im_p2[6] & im_g4[2]);
		assign im_bo1[0] = 1'b1;
		assign im_bo1[1] = im_g2[0] | im_p2[0];
		assign im_bo1[2] = im_g4[0] | im_p4[0];
		assign im_bo1[3] = im_g2[2] | (im_p2[2] & (im_g4[0] | im_p4[0]));
		assign im_bo1[4] = 1'b1;
		assign im_bo1[5] = im_g2[4] | im_p2[4];
		assign im_bo1[6] = im_g4[2] | im_p4[2];
		assign im_bo1[7] = im_g2[6] | (im_p2[6] & (im_g4[2] | im_p4[2]));
		wire [7:0] im_lo0, im_lo1, im_hi0, im_hi1;
		for (gm2 = 0; gm2 < 4; gm2 = gm2 + 1) begin : g_im_ml
			assign im_lo0[2*gm2 +: 2] = im_bo0[gm2] ? im_d1[gm2] : im_d0[gm2];
			assign im_lo1[2*gm2 +: 2] = im_bo1[gm2] ? im_d1[gm2] : im_d0[gm2];
		end
		for (gm2 = 4; gm2 < 8; gm2 = gm2 + 1) begin : g_im_mh
			assign im_hi0[2*(gm2-4) +: 2] = im_bo0[gm2] ? im_d1[gm2] : im_d0[gm2];
			assign im_hi1[2*(gm2-4) +: 2] = im_bo1[gm2] ? im_d1[gm2] : im_d0[gm2];
		end
		assign init_m0_w[ge]   = {im_b8_0 ? im_hi1 : im_hi0, im_lo0};
		assign init_m1_w[ge]   = {im_b8_1 ? im_hi1 : im_hi0, im_lo1};
		assign init_mbo0_w[ge] = im_b16_0 | (1'b0 & ((&im_u0) & (&im_u1)));
		assign init_mbo1_w[ge] = im_b16_1;

		/* Compute [47:32] speculative init_h0/h1 in S_INIT in parallel with [31:16] (<= 10 cmos3) */
		wire [15:0] ih_a = er[ge][47:32];
		wire [15:0] ih_b = init_e_sub[ge][47:32];
		wire [15:0] ih_bg = (~ih_a) & ih_b;
		wire [15:0] ih_bp = (~ih_a) | ih_b;
		wire [7:0] ih_g2, ih_p2, ih_u0, ih_u1;
		wire [1:0] ih_d0 [0:7];
		wire [1:0] ih_d1 [0:7];
		genvar gb2;
		for (gb2 = 0; gb2 < 8; gb2 = gb2 + 1) begin : g_ih_2b
			assign ih_p2[gb2] = ih_bp[2*gb2+1] & ih_bp[2*gb2];
			assign ih_g2[gb2] = ih_bg[2*gb2+1] | (ih_bp[2*gb2+1] & ih_bg[2*gb2]);
			am_sub #(.W(2)) u_ih_d0 (.a(ih_a[2*gb2 +: 2]), .b(ih_b[2*gb2 +: 2]), .bi(1'b0), .d(ih_d0[gb2]), .bo(ih_u0[gb2]));
			am_sub #(.W(2)) u_ih_d1 (.a(ih_a[2*gb2 +: 2]), .b(ih_b[2*gb2 +: 2]), .bi(1'b1), .d(ih_d1[gb2]), .bo(ih_u1[gb2]));
		end
		wire [3:0] ih_g4, ih_p4;
		for (gb2 = 0; gb2 < 4; gb2 = gb2 + 1) begin : g_ih_4b
			assign ih_p4[gb2] = ih_p2[2*gb2+1] & ih_p2[2*gb2];
			assign ih_g4[gb2] = ih_g2[2*gb2+1] | (ih_p2[2*gb2+1] & ih_g2[2*gb2]);
		end
		wire [7:0] ih_bo0, ih_bo1;
		wire ih_b8_0 = ih_g4[1] | (ih_p4[1] & ih_g4[0]);
		wire ih_b8_1 = ih_g4[1] | (ih_p4[1] & (ih_g4[0] | ih_p4[0]));
		assign ih_bo0[0] = 1'b0;
		assign ih_bo0[1] = ih_g2[0];
		assign ih_bo0[2] = ih_g4[0];
		assign ih_bo0[3] = ih_g2[2] | (ih_p2[2] & ih_g4[0]);
		assign ih_bo1[0] = 1'b1;
		assign ih_bo1[1] = ih_g2[0] | ih_p2[0];
		assign ih_bo1[2] = ih_g4[0] | ih_p4[0];
		assign ih_bo1[3] = ih_g2[2] | (ih_p2[2] & (ih_g4[0] | ih_p4[0]));

		assign ih_bo0[4] = 1'b0;
		assign ih_bo0[5] = ih_g2[4];
		assign ih_bo0[6] = ih_g4[2];
		assign ih_bo0[7] = ih_g2[6] | (ih_p2[6] & ih_g4[2]);
		assign ih_bo1[4] = 1'b1;
		assign ih_bo1[5] = ih_g2[4] | ih_p2[4];
		assign ih_bo1[6] = ih_g4[2] | ih_p4[2];
		assign ih_bo1[7] = ih_g2[6] | (ih_p2[6] & (ih_g4[2] | ih_p4[2]));

		wire [7:0] ih_lo0, ih_lo1, ih_hi0, ih_hi1;
		for (gb2 = 0; gb2 < 4; gb2 = gb2 + 1) begin : g_ih_mux_lo
			assign ih_lo0[2*gb2 +: 2] = ih_bo0[gb2] ? ih_d1[gb2] : ih_d0[gb2];
			assign ih_lo1[2*gb2 +: 2] = ih_bo1[gb2] ? ih_d1[gb2] : ih_d0[gb2];
		end
		for (gb2 = 4; gb2 < 8; gb2 = gb2 + 1) begin : g_ih_mux_hi
			assign ih_hi0[2*(gb2-4) +: 2] = ih_bo0[gb2] ? ih_d1[gb2] : ih_d0[gb2];
			assign ih_hi1[2*(gb2-4) +: 2] = ih_bo1[gb2] ? ih_d1[gb2] : ih_d0[gb2];
		end
		assign init_h0_w[ge] = {ih_b8_0 ? ih_hi1 : ih_hi0, ih_lo0};
		assign init_h1_w[ge] = {ih_b8_1 ? ih_hi1 : ih_hi0, ih_lo1};

		wire bo32_w = r_init_bo16[ge] ? r_init_mbo1[ge] : r_init_mbo0[ge];
		assign init_er[ge] = {bo32_w ? r_init_h1[ge] : r_init_h0[ge], r_init_bo16[ge] ? r_init_m1[ge] : r_init_m0[ge], r_init_er_lo[ge]};
		assign init_er_carry_unused[ge] = (&ih_u0) & (&ih_u1) & ih_g4[3] & ih_p4[3];
`endif
	end
	endgenerate

	integer k;
`ifdef SYNTH_KINTEX7
	assign tri_ready = (st_c1 == S_IDLE);
	assign busy = (st_c1 != S_IDLE);
`elsif SYNTH_KINTEX_ULTRASCALE
	assign tri_ready = (st_c1 == S_IDLE);
	assign busy = (st_c1 != S_IDLE);
`else
	assign tri_ready = (st == S_IDLE);
	assign busy = (st != S_IDLE);
`endif

`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire we_bx0, we_by0, we_bx1, we_by1;
	wire we_bbox_raw = (st_c1 == S_IDLE) && tri_valid;
	LUT1 #(.INIT(2'h2)) u_we_bx0 (.I0(we_bbox_raw), .O(we_bx0));
	LUT1 #(.INIT(2'h2)) u_we_by0 (.I0(we_bbox_raw), .O(we_by0));
	LUT1 #(.INIT(2'h2)) u_we_bx1 (.I0(we_bbox_raw), .O(we_bx1));
	LUT1 #(.INIT(2'h2)) u_we_by1 (.I0(we_bbox_raw), .O(we_by1));
	always @(posedge clk) if (we_bx0) bx0 <= bbox_min_x;
	always @(posedge clk) if (we_by0) by0 <= bbox_min_y;
	always @(posedge clk) if (we_bx1) bx1 <= bbox_max_x;
	always @(posedge clk) if (we_by1) by1 <= bbox_max_y;

	(* keep = "true", dont_touch = "true" *) wire we_tx_next, we_ty_next;
	wire we_txy_raw = (st_c2 == S_WALK);
	LUT1 #(.INIT(2'h2)) u_we_tx (.I0(we_txy_raw), .O(we_tx_next));
	LUT1 #(.INIT(2'h2)) u_we_ty (.I0(we_txy_raw), .O(we_ty_next));
	always @(posedge clk) if (we_tx_next) r_tx_next <= tx_next;
	always @(posedge clk) if (we_ty_next) r_ty_next <= ty_next;

	wire [8:0] we_walk_emax_leaf, we_walk_emin_leaf;
	wire [8:0] we_walk_er_nx_leaf, we_walk_ec_nx_leaf;
	wire [8:0] we_init_dmax_leaf, we_init_dmin_leaf;
	wire [8:0] we_init_er_leaf, we_init_ec_leaf;
	wire [8:0] we_idle_a_leaf, we_idle_b_leaf, we_idle_ab_leaf;
	wire [8:0] we_idle_er_leaf, we_idle_ec_leaf;
	wire [8:0] we_lm_leaf;
	wire [8:0] we_step_er_leaf, we_step_ec_leaf;
	wire [2:0] we_walk_mid, we_init_mid, we_idle_mid, we_step_mid;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwm0 (.I0(st_c2 == S_WALK), .O(we_walk_mid[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwm1 (.I0(st_c2 == S_WALK), .O(we_walk_mid[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwm2 (.I0(st_c2 == S_WALK), .O(we_walk_mid[2]));

	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wim0 (.I0(st_c1 == S_INIT), .O(we_init_mid[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wim1 (.I0(st_c1 == S_INIT), .O(we_init_mid[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wim2 (.I0(st_c1 == S_INIT), .O(we_init_mid[2]));

	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wid0 (.I0((st_c3 == S_IDLE) && tri_valid), .O(we_idle_mid[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wid1 (.I0((st_c3 == S_IDLE) && tri_valid), .O(we_idle_mid[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wid2 (.I0((st_c3 == S_IDLE) && tri_valid), .O(we_idle_mid[2]));

	wire step_advance = ((st_c3 == S_STEP) && r_tile_outside && !r_last_tile) ||
	                    ((st_c3 == S_WAIT) && tile_ready && !r_last_tile);
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wsm0 (.I0(step_advance), .O(we_step_mid[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wsm1 (.I0(step_advance), .O(we_step_mid[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wsm2 (.I0(step_advance), .O(we_step_mid[2]));

	(* dont_touch = "true" *) wire [8:0] r_row_end_b;
	genvar grre;
	generate
		for (grre = 0; grre < 9; grre = grre + 1) begin : g_rre
			LUT1 #(.INIT(2'h2)) u_rre (.I0(r_row_end), .O(r_row_end_b[grre]));
		end
	endgenerate

	genvar gck;
	generate
		for (gck = 0; gck < 9; gck = gck + 1) begin : g_coarse_we_leaf
			localparam integer EK = gck / 3;
			localparam integer SK = gck % 3;
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwl_emx (.I0(we_walk_mid[EK]), .O(we_walk_emax_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwl_emn (.I0(we_walk_mid[EK]), .O(we_walk_emin_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwl_ern (.I0(we_walk_mid[EK]), .O(we_walk_er_nx_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wwl_ecn (.I0(we_walk_mid[EK]), .O(we_walk_ec_nx_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wil_dmx (.I0(we_init_mid[EK]), .O(we_init_dmax_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wil_dmn (.I0(we_init_mid[EK]), .O(we_init_dmin_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wil_er  (.I0(we_init_mid[EK]), .O(we_init_er_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wil_ec  (.I0(we_init_mid[EK]), .O(we_init_ec_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_l_a   (.I0(we_idle_mid[EK]), .O(we_idle_a_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_l_b   (.I0(we_idle_mid[EK]), .O(we_idle_b_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_l_ab  (.I0(we_idle_mid[EK]), .O(we_idle_ab_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_l_er  (.I0(we_idle_mid[EK]), .O(we_idle_er_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_l_ec  (.I0(we_idle_mid[EK]), .O(we_idle_ec_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_w_lm    (.I0(we_idle_mid[EK] || we_init_mid[EK] || we_walk_mid[EK]), .O(we_lm_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wsler   (.I0(we_step_mid[EK]), .O(we_step_er_leaf[gck]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_wslec   (.I0(we_step_mid[EK]), .O(we_step_ec_leaf[gck]));

			wire [15:0] in_a8_k = (EK == 0) ? e0_a8[SK*16 +: 16] : (EK == 1) ? e1_a8[SK*16 +: 16] : e2_a8[SK*16 +: 16];
			wire [15:0] in_b8_k = (EK == 0) ? e0_b8[SK*16 +: 16] : (EK == 1) ? e1_b8[SK*16 +: 16] : e2_b8[SK*16 +: 16];
			wire [15:0] in_er_k = (EK == 0) ? e0_er[SK*16 +: 16] : (EK == 1) ? e1_er[SK*16 +: 16] : e2_er[SK*16 +: 16];
			wire [15:0] in_lm_k = (EK == 0) ? lowm0[SK*16 +: 16] : (EK == 1) ? lowm1[SK*16 +: 16] : lowm2[SK*16 +: 16];

			always @(posedge clk) begin
				if (we_idle_a_leaf[gck])
					a8[EK][SK*16 +: 16]  <= in_a8_k;
				if (we_idle_b_leaf[gck])
					b8[EK][SK*16 +: 16]  <= in_b8_k;
				if (we_idle_ab_leaf[gck])
					ab8[EK][SK*16 +: 16] <= init_ab8_comb[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_init_dmax_leaf[gck])
					d_max[EK][SK*16 +: 16] <= init_dmax[EK][SK*16 +: 16];
				if (we_init_dmin_leaf[gck])
					d_min[EK][SK*16 +: 16] <= init_dmin[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_walk_emax_leaf[gck])
					r_emax[EK][SK*16 +: 16]    <= e_max[EK][SK*16 +: 16];
				if (we_walk_emin_leaf[gck])
					r_emin[EK][SK*16 +: 16]    <= e_min[EK][SK*16 +: 16];
				if (we_walk_ec_nx_leaf[gck])
					r_ec_next[EK][SK*16 +: 16] <= ec_next[EK][SK*16 +: 16];
				if (we_walk_er_nx_leaf[gck])
					r_er_next[EK][SK*16 +: 16] <= er_next[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_lm_leaf[gck]) begin
					lm[EK][SK*16 +: 16] <= in_lm_k;
				end
			end

			always @(posedge clk) begin
				if (we_idle_er_leaf[gck]) begin
					er[EK][SK*16 +: 16] <= in_er_k;
				end else if (we_init_er_leaf[gck]) begin
					er[EK][SK*16 +: 16] <= init_er[EK][SK*16 +: 16];
				end else if (we_step_er_leaf[gck] && r_row_end_b[gck]) begin
					er[EK][SK*16 +: 16] <= r_er_next[EK][SK*16 +: 16];
				end
			end

			always @(posedge clk) begin
				if (we_idle_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= in_er_k;
				end else if (we_init_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= init_er[EK][SK*16 +: 16];
				end else if (we_step_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= r_row_end_b[gck] ? r_er_next[EK][SK*16 +: 16] : r_ec_next[EK][SK*16 +: 16];
				end
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire we_bx0, we_by0, we_bx1, we_by1;
	wire we_bbox_raw = (st_c1 == S_IDLE) && tri_valid;
	assign we_bx0 = we_bbox_raw;
	assign we_by0 = we_bbox_raw;
	assign we_bx1 = we_bbox_raw;
	assign we_by1 = we_bbox_raw;
	always @(posedge clk) if (we_bx0) bx0 <= bbox_min_x;
	always @(posedge clk) if (we_by0) by0 <= bbox_min_y;
	always @(posedge clk) if (we_bx1) bx1 <= bbox_max_x;
	always @(posedge clk) if (we_by1) by1 <= bbox_max_y;

	(* keep = "true", dont_touch = "true" *) wire we_tx_next, we_ty_next;
	wire we_txy_raw = (st_c2 == S_WALK);
	assign we_tx_next = we_txy_raw;
	assign we_ty_next = we_txy_raw;
	always @(posedge clk) if (we_tx_next) r_tx_next <= tx_next;
	always @(posedge clk) if (we_ty_next) r_ty_next <= ty_next;

	wire [8:0] we_walk_emax_leaf, we_walk_emin_leaf;
	wire [8:0] we_walk_er_nx_leaf, we_walk_ec_nx_leaf;
	wire [8:0] we_init_dmax_leaf, we_init_dmin_leaf;
	wire [8:0] we_init_er_leaf, we_init_ec_leaf;
	wire [8:0] we_idle_a_leaf, we_idle_b_leaf, we_idle_ab_leaf;
	wire [8:0] we_idle_er_leaf, we_idle_ec_leaf;
	wire [8:0] we_lm_leaf;
	wire [8:0] we_step_er_leaf, we_step_ec_leaf;
	wire [2:0] we_walk_mid, we_init_mid, we_idle_mid, we_step_mid;
	assign we_walk_mid[0] = st_c2 == S_WALK;
	assign we_walk_mid[1] = st_c2 == S_WALK;
	assign we_walk_mid[2] = st_c2 == S_WALK;

	assign we_init_mid[0] = st_c1 == S_INIT;
	assign we_init_mid[1] = st_c1 == S_INIT;
	assign we_init_mid[2] = st_c1 == S_INIT;

	assign we_idle_mid[0] = (st_c3 == S_IDLE) && tri_valid;
	assign we_idle_mid[1] = (st_c3 == S_IDLE) && tri_valid;
	assign we_idle_mid[2] = (st_c3 == S_IDLE) && tri_valid;

	wire step_advance = ((st_c3 == S_STEP) && r_tile_outside && !r_last_tile) ||
	                    ((st_c3 == S_WAIT) && tile_ready && !r_last_tile);
	assign we_step_mid[0] = step_advance;
	assign we_step_mid[1] = step_advance;
	assign we_step_mid[2] = step_advance;

	(* dont_touch = "true" *) wire [8:0] r_row_end_b;
	genvar grre;
	generate
		for (grre = 0; grre < 9; grre = grre + 1) begin : g_rre
			assign r_row_end_b[grre] = r_row_end;
		end
	endgenerate

	genvar gck;
	generate
		for (gck = 0; gck < 9; gck = gck + 1) begin : g_coarse_we_leaf
			localparam integer EK = gck / 3;
			localparam integer SK = gck % 3;
			assign we_walk_emax_leaf[gck] = we_walk_mid[EK];
			assign we_walk_emin_leaf[gck] = we_walk_mid[EK];
			assign we_walk_er_nx_leaf[gck] = we_walk_mid[EK];
			assign we_walk_ec_nx_leaf[gck] = we_walk_mid[EK];
			assign we_init_dmax_leaf[gck] = we_init_mid[EK];
			assign we_init_dmin_leaf[gck] = we_init_mid[EK];
			assign we_init_er_leaf[gck] = we_init_mid[EK];
			assign we_init_ec_leaf[gck] = we_init_mid[EK];
			assign we_idle_a_leaf[gck] = we_idle_mid[EK];
			assign we_idle_b_leaf[gck] = we_idle_mid[EK];
			assign we_idle_ab_leaf[gck] = we_idle_mid[EK];
			assign we_idle_er_leaf[gck] = we_idle_mid[EK];
			assign we_idle_ec_leaf[gck] = we_idle_mid[EK];
			assign we_lm_leaf[gck] = we_idle_mid[EK] || we_init_mid[EK] || we_walk_mid[EK];
			assign we_step_er_leaf[gck] = we_step_mid[EK];
			assign we_step_ec_leaf[gck] = we_step_mid[EK];

			wire [15:0] in_a8_k = (EK == 0) ? e0_a8[SK*16 +: 16] : (EK == 1) ? e1_a8[SK*16 +: 16] : e2_a8[SK*16 +: 16];
			wire [15:0] in_b8_k = (EK == 0) ? e0_b8[SK*16 +: 16] : (EK == 1) ? e1_b8[SK*16 +: 16] : e2_b8[SK*16 +: 16];
			wire [15:0] in_er_k = (EK == 0) ? e0_er[SK*16 +: 16] : (EK == 1) ? e1_er[SK*16 +: 16] : e2_er[SK*16 +: 16];
			wire [15:0] in_lm_k = (EK == 0) ? lowm0[SK*16 +: 16] : (EK == 1) ? lowm1[SK*16 +: 16] : lowm2[SK*16 +: 16];

			always @(posedge clk) begin
				if (we_idle_a_leaf[gck])
					a8[EK][SK*16 +: 16]  <= in_a8_k;
				if (we_idle_b_leaf[gck])
					b8[EK][SK*16 +: 16]  <= in_b8_k;
				if (we_idle_ab_leaf[gck])
					ab8[EK][SK*16 +: 16] <= init_ab8_comb[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_init_dmax_leaf[gck])
					d_max[EK][SK*16 +: 16] <= init_dmax[EK][SK*16 +: 16];
				if (we_init_dmin_leaf[gck])
					d_min[EK][SK*16 +: 16] <= init_dmin[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_walk_emax_leaf[gck])
					r_emax[EK][SK*16 +: 16]    <= e_max[EK][SK*16 +: 16];
				if (we_walk_emin_leaf[gck])
					r_emin[EK][SK*16 +: 16]    <= e_min[EK][SK*16 +: 16];
				if (we_walk_ec_nx_leaf[gck])
					r_ec_next[EK][SK*16 +: 16] <= ec_next[EK][SK*16 +: 16];
				if (we_walk_er_nx_leaf[gck])
					r_er_next[EK][SK*16 +: 16] <= er_next[EK][SK*16 +: 16];
			end

			always @(posedge clk) begin
				if (we_lm_leaf[gck]) begin
					lm[EK][SK*16 +: 16] <= in_lm_k;
				end
			end

			always @(posedge clk) begin
				if (we_idle_er_leaf[gck]) begin
					er[EK][SK*16 +: 16] <= in_er_k;
				end else if (we_init_er_leaf[gck]) begin
					er[EK][SK*16 +: 16] <= init_er[EK][SK*16 +: 16];
				end else if (we_step_er_leaf[gck] && r_row_end_b[gck]) begin
					er[EK][SK*16 +: 16] <= r_er_next[EK][SK*16 +: 16];
				end
			end

			always @(posedge clk) begin
				if (we_idle_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= in_er_k;
				end else if (we_init_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= init_er[EK][SK*16 +: 16];
				end else if (we_step_ec_leaf[gck]) begin
					ec[EK][SK*16 +: 16] <= r_row_end_b[gck] ? r_er_next[EK][SK*16 +: 16] : r_ec_next[EK][SK*16 +: 16];
				end
			end
		end
	endgenerate
`endif

	reg [2:0] next_st;
	always @(*) begin
		next_st = st;
		case (st)
		S_IDLE: begin
			if (tri_valid) begin
				if (bbox_max_x > bbox_min_x && bbox_max_y > bbox_min_y)
					next_st = S_INIT;
				else
					next_st = S_IDLE;
			end
		end
		S_INIT:  next_st = S_INIT2;
		S_INIT2: next_st = S_WALK;
		S_WALK: next_st = S_EVAL;
		S_EVAL: next_st = S_STEP;
		S_STEP: begin
			if (r_tile_outside) begin
				if (r_last_tile) next_st = S_IDLE;
				else next_st = S_WALK;
			end else begin
				next_st = S_WAIT;
			end
		end
		S_WAIT: begin
			if (tile_ready) begin
				if (r_last_tile) next_st = S_IDLE;
				else next_st = S_WALK;
			end
		end
		default: next_st = S_IDLE;
		endcase
	end

	always @(posedge clk or posedge coarse_rst_sync) begin
		if (coarse_rst_sync) begin
			st <= S_IDLE;
`ifdef SYNTH_KINTEX7
			st_c0 <= S_IDLE;
			st_c1 <= S_IDLE;
			st_c2 <= S_IDLE;
			st_c3 <= S_IDLE;
`elsif SYNTH_KINTEX_ULTRASCALE
			st_c0 <= S_IDLE;
			st_c1 <= S_IDLE;
			st_c2 <= S_IDLE;
			st_c3 <= S_IDLE;
`endif
		end else begin
			st <= next_st;
`ifdef SYNTH_KINTEX7
			st_c0 <= next_st;
			st_c1 <= next_st;
			st_c2 <= next_st;
			st_c3 <= next_st;
`elsif SYNTH_KINTEX_ULTRASCALE
			st_c0 <= next_st;
			st_c1 <= next_st;
			st_c2 <= next_st;
			st_c3 <= next_st;
`endif
		end
	end

	always @(posedge clk or posedge coarse_rst_sync) begin
		if (coarse_rst_sync) begin
			tile_valid <= 1'b0;
			tile_last <= 1'b0;
			tile_trivial <= 1'b0;
			r_row_end <= 1'b0;
			r_tile_outside <= 1'b0;
			r_is_trivial <= 1'b0;
			r_last_tile <= 1'b0;
		end else begin
`ifdef SYNTH_KINTEX7
			if (st_c0 == S_IDLE) begin
				tile_valid <= 1'b0;
				tile_last <= 1'b0;
				if (tri_valid) begin
					if (we_txy_idle_l) tile_x <= {bbox_min_x[15:1], 1'b0};
					if (we_txy_idle_h) tile_y <= {bbox_min_y[15:1], 1'b0};
				end
			end else if (st_c2 == S_EVAL) begin
				r_tile_outside <= tile_outside;
				r_is_trivial <= is_trivial;
				r_last_tile <= last_tile;
				r_row_end <= row_end;
			end else if (st_c3 == S_STEP) begin
				if (r_tile_outside) begin
					if (!r_last_tile) begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
						end else begin
							tile_x <= r_tx_next;
						end
					end
				end else begin
					tile_valid <= 1'b1;
					tile_trivial <= r_is_trivial;
					tile_last <= r_last_tile;
				end
			end else if (st_c3 == S_WAIT) begin
				if (tile_ready) begin
					tile_valid <= 1'b0;
					if (r_last_tile) begin
						tile_last <= 1'b0;
					end else begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
						end else begin
							tile_x <= r_tx_next;
						end
					end
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (st_c0 == S_IDLE) begin
				tile_valid <= 1'b0;
				tile_last <= 1'b0;
				if (tri_valid) begin
					if (we_txy_idle_l) tile_x <= {bbox_min_x[15:1], 1'b0};
					if (we_txy_idle_h) tile_y <= {bbox_min_y[15:1], 1'b0};
				end
			end else if (st_c2 == S_EVAL) begin
				r_tile_outside <= tile_outside;
				r_is_trivial <= is_trivial;
				r_last_tile <= last_tile;
				r_row_end <= row_end;
			end else if (st_c3 == S_STEP) begin
				if (r_tile_outside) begin
					if (!r_last_tile) begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
						end else begin
							tile_x <= r_tx_next;
						end
					end
				end else begin
					tile_valid <= 1'b1;
					tile_trivial <= r_is_trivial;
					tile_last <= r_last_tile;
				end
			end else if (st_c3 == S_WAIT) begin
				if (tile_ready) begin
					tile_valid <= 1'b0;
					if (r_last_tile) begin
						tile_last <= 1'b0;
					end else begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
						end else begin
							tile_x <= r_tx_next;
						end
					end
				end
			end
`else
			case (st)
			S_IDLE: begin
				tile_valid <= 1'b0;
				tile_last <= 1'b0;
				if (tri_valid) begin
					bx0 <= bbox_min_x;
					by0 <= bbox_min_y;
					bx1 <= bbox_max_x;
					by1 <= bbox_max_y;
					r_init_e_sub[0] <= (bbox_min_x[0] && bbox_min_y[0]) ? init_ab8_comb[0] : (bbox_min_x[0] ? e0_a8 : (bbox_min_y[0] ? e0_b8 : 48'd0));
					r_init_e_sub[1] <= (bbox_min_x[0] && bbox_min_y[0]) ? init_ab8_comb[1] : (bbox_min_x[0] ? e1_a8 : (bbox_min_y[0] ? e1_b8 : 48'd0));
					r_init_e_sub[2] <= (bbox_min_x[0] && bbox_min_y[0]) ? init_ab8_comb[2] : (bbox_min_x[0] ? e2_a8 : (bbox_min_y[0] ? e2_b8 : 48'd0));
					/* Align tile origin to bounding box origin */
					tile_x <= {bbox_min_x[15:1], 1'b0};
					tile_y <= {bbox_min_y[15:1], 1'b0};
					a8[0] <= e0_a8; b8[0] <= e0_b8; er[0] <= e0_er; ec[0] <= e0_er;
					a8[1] <= e1_a8; b8[1] <= e1_b8; er[1] <= e1_er; ec[1] <= e1_er;
					a8[2] <= e2_a8; b8[2] <= e2_b8; er[2] <= e2_er; ec[2] <= e2_er;
					a8_neg_rep[0] <= {12{e0_a8[47]}}; b8_neg_rep[0] <= {12{e0_b8[47]}};
					a8_neg_rep[1] <= {12{e1_a8[47]}}; b8_neg_rep[1] <= {12{e1_b8[47]}};
					a8_neg_rep[2] <= {12{e2_a8[47]}}; b8_neg_rep[2] <= {12{e2_b8[47]}};
					ab8[0] <= init_ab8_comb[0];
					ab8[1] <= init_ab8_comb[1];
					ab8[2] <= init_ab8_comb[2];
					lm[0] <= lowm0;
					lm[1] <= lowm1;
					lm[2] <= lowm2;
				end
			end

			S_INIT: begin
				for (k = 0; k < 3; k = k + 1) begin
					r_dx_p[k] <= dx_p[k];
					r_dx_m[k] <= dx_m[k];
					r_dy_p[k] <= dy_p[k];
					r_dy_m[k] <= dy_m[k];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					r_init_er_lo[k]  <= init_lo_d[k];
					r_init_bo16[k]   <= init_lo_bo[k];
					r_init_m0[k]     <= init_m0_w[k];
					r_init_m1[k]     <= init_m1_w[k];
					r_init_mbo0[k]   <= init_mbo0_w[k];
					r_init_mbo1[k]   <= init_mbo1_w[k];
					r_init_h0[k]     <= init_h0_w[k];
					r_init_h1[k]     <= init_h1_w[k];
`endif
				end
				lm[0] <= lowm0;
				lm[1] <= lowm1;
				lm[2] <= lowm2;
			end

			S_INIT2: begin
				for (k = 0; k < 3; k = k + 1) begin
					d_max[k] <= init_dmax[k];
					d_min[k] <= init_dmin[k];
					er[k]    <= init_er[k];
					ec[k]    <= init_er[k];
				end
			end

			S_WALK: begin
				for (k = 0; k < 3; k = k + 1) begin
					r_emax[k]    <= e_max[k];
					r_emin[k]    <= e_min[k];
					r_ec_next[k] <= ec_next[k];
					r_er_next[k] <= er_next[k];
					lm[k]        <= (k == 0) ? lowm0 : (k == 1) ? lowm1 : lowm2;
				end
				r_tx_next <= tx_next;
				r_ty_next <= ty_next;
			end

			S_EVAL: begin
				r_tile_outside <= tile_outside;
				r_is_trivial <= is_trivial;
				r_last_tile <= last_tile;
				r_row_end <= row_end;
			end

			S_STEP: begin
				if (r_tile_outside) begin
					if (!r_last_tile) begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
							for (k = 0; k < 3; k = k + 1) begin
								er[k] <= r_er_next[k];
								ec[k] <= r_er_next[k];
							end
						end else begin
							tile_x <= r_tx_next;
							for (k = 0; k < 3; k = k + 1)
								ec[k] <= r_ec_next[k];
						end
					end
				end else begin
					tile_valid <= 1'b1;
					tile_trivial <= r_is_trivial;
					tile_last <= r_last_tile;
				end
			end

			S_WAIT: begin
				if (tile_ready) begin
					tile_valid <= 1'b0;
					if (r_last_tile) begin
						tile_last <= 1'b0;
					end else begin
						if (r_row_end) begin
							tile_x <= {bx0[15:1], 1'b0};
							tile_y <= r_ty_next;
							for (k = 0; k < 3; k = k + 1) begin
								er[k] <= r_er_next[k];
								ec[k] <= r_er_next[k];
							end
						end else begin
							tile_x <= r_tx_next;
							for (k = 0; k < 3; k = k + 1)
								ec[k] <= r_ec_next[k];
						end
					end
				end
			end
			default: begin end
			endcase
`endif
		end
	end

	wire unused_ok = &{1'b0, eval_carry_unused, coord_carry_unused,
		step_carry_unused, init_carry_unused, init_ab8_carry_unused, init_er_carry_unused,
		inside_d_unused};
endmodule

`default_nettype wire
