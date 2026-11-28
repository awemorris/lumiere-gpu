/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_seg: the segment geometry of the blitter's 2D engine
 * (atg_blt_2d), combinational, in bytes: the length of the segment that
 * starts at pixel px (byte address da) and stays inside one destination
 * line (reversed: from px back to the line's start) and inside one row of
 * a 4x4 tile (a tiled destination or source), at most rem pixels; the
 * bytes back from the rightmost pixel of a reversed segment; the source
 * window (the source byte that maps to byte 0 of the destination line,
 * and the line of the segment's last source byte); the write's byte
 * enables: bytes p0 .. p0 + snu - 1 of the line without the KEEP bytes of
 * each 32-bit pixel.
 * See spec/gpu/blt/atg_blt_seg.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_blt_seg #(
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter BURST = 0,
	parameter USE_SNU_M1_ONLY = 0,
	parameter USE_IN_ROOM_ONLY = 0,
	parameter USE_IN_DA_PX_ONLY = 0,
	parameter USE_ROOM_TILE3_ONLY = 0
) (
	input wire [2:0] psl,	/* log2 of the pixel's bytes */
	input wire [LINE_SHIFT:0] line_pix,	/* LINE_BYTES >> psl */
	input wire rev,
	input wire dtile,
	input wire stile,
	input wire [3:0] keep,
	/* S_SB: the segment's length */
	input wire [LINE_SHIFT-1:0] da,	/* the byte of px in its line */
	input wire [1:0] px,
	input wire [1:0] spx,
	input wire [16:0] rem,
	input wire [LINE_SHIFT:0] in_room,
	input wire use_in_room,
	input wire [LINE_SHIFT:0] in_da_px,
	input wire use_in_da_px,
	input wire [2:0] in_room_tile3,
	input wire use_room_tile3,
	input wire [LINE_SHIFT:0] in_snu_m1,
	input wire use_snu_m1,
	output wire [LINE_SHIFT:0] seg_n,
	output wire seg_last,
	output wire [LINE_SHIFT:0] room,
	/* S_SC: back from the rightmost pixel (reversed) */
	input wire [LINE_SHIFT:0] sn,
	output wire [ADDR_W-1:0] back,
	/* S_SD: the source window */
	input wire [LINE_SHIFT:0] snu,
	input wire [LINE_SHIFT-1:0] dlo,	/* the byte of the first destination byte */
	input wire [ADDR_W-1:0] slo,
	output wire [ADDR_W-1:0] src_base,
	output wire [ADDR_W-LINE_SHIFT-1:0] last_line,	/* the line of the last source byte */
	/* S_WR: the byte enables */
	input wire [LINE_SHIFT-1:0] p0,
	output reg [LINE_BYTES-1:0] be
);
	localparam US = LINE_SHIFT;

	/* verilator lint_off WIDTHTRUNC */
	localparam [US:0] LB_V = LINE_BYTES;
	/* verilator lint_on WIDTHTRUNC */

	wire [US-1:0] p_da = da;
	wire [US:0] da_px, room_r, back_n, p_end, leg_back_n;
	wire [ADDR_W-1:0] src_next, src_last;
	wire [7:0] seg_co_unused;
	assign seg_co_unused[7] = 1'b0;
	assign seg_co_unused[1:0] = 2'b00;
	wire [3:0] cs_co_unused;

	/* Pre-shift da by pixel size (da_px = p_da >> psl) */
	generate
		if (USE_IN_DA_PX_ONLY) begin : g_dapx_in
			assign da_px = in_da_px;
		end else begin : g_dapx_dyn
			assign da_px = use_in_da_px ? in_da_px :
			               (psl == 3'd4) ? {5'b00000, p_da[US-1:4]} :
			               (psl == 3'd3) ? {4'b0000, p_da[US-1:3]}  :
			               (psl == 3'd2) ? {3'b000, p_da[US-1:2]}   :
			               (psl == 3'd1) ? {2'b00, p_da[US-1:1]}    : {1'b0, p_da};
		end
	endgenerate

	/* Line pixels: LINE_BYTES >> psl */
	wire [US:0] line_pixels = line_pix;

	/* Forward: line_pixels - da_px; Reverse: da_px + 1 */
	wire [US:0] room_f;

	assign room_f[0] = da_px[0];
	assign room_r[0] = ~da_px[0];

	genvar g_rf;
	generate
		for (g_rf = 1; g_rf < US; g_rf = g_rf + 1) begin : g_rf_bits
			assign room_f[g_rf] = (|line_pixels[US:g_rf+1]) ?
				(da_px[g_rf] ^ (|da_px[g_rf-1:0])) :
				((da_px[g_rf-1:0] == {g_rf{1'b0}}) & line_pixels[g_rf]);
			assign room_r[g_rf] = da_px[g_rf] ^ (&da_px[g_rf-1:0]);
		end
	endgenerate

	assign room_f[US] = (da_px[US-1:0] == {US{1'b0}}) & line_pixels[US];
	assign room_r[US] = da_px[US] ^ (&da_px[US-1:0]);
	wire [2:0] tile_room_d = {px == 2'b00, px[1] ^ px[0], px[0]};
	wire [2:0] tile_room_s = {spx == 2'b00, spx[1] ^ spx[0], spx[0]};

	wire rem_large = |rem[16:US];
	wire tiled = dtile || stile;

	wire [US:0] fwd_rem = {1'b0, rem[US-1:0]};
	wire [US:0] room_line = rev ? room_r : room_f;
	wire [2:0] room_tile3;
	generate
		if (USE_ROOM_TILE3_ONLY) begin : g_rt3_in
			assign room_tile3 = in_room_tile3;
		end else begin : g_rt3_dyn
			wire [2:0] room_t3 = tile_room_d;
			wire [2:0] room_s3 = tile_room_s;
			wire [2:0] room_tile3_comb = (dtile && stile) ? (room_t3 < room_s3 ? room_t3 : room_s3) :
			                             dtile ? room_t3 : room_s3;
			assign room_tile3 = use_room_tile3 ? in_room_tile3 : room_tile3_comb;
		end
	endgenerate
	/* Parallel forward/reverse tile comparison before rev mux cuts leg_room to <= 10 cmos3. */
	wire take_tile_f = tiled && ((|room_f[US:3]) || (room_tile3 < room_f[2:0]));
	wire take_tile_r = tiled && ((|room_r[US:3]) || (room_tile3 < room_r[2:0]));
	wire [US:0] room_calc_f = {take_tile_f ? {(US-2){1'b0}} : room_f[US:3],
	                           take_tile_f ? room_tile3     : room_f[2:0]};
	wire [US:0] room_calc_r = {take_tile_r ? {(US-2){1'b0}} : room_r[US:3],
	                           take_tile_r ? room_tile3     : room_r[2:0]};
	wire [US:0] room_calc = rev ? room_calc_r : room_calc_f;
	assign room = room_calc;
	wire [US:0] eff_room;
	generate
		if (USE_IN_ROOM_ONLY || BURST) begin : g_eff_in
			assign eff_room = in_room;
		end else begin : g_eff_dyn
			assign eff_room = use_in_room ? in_room : room_calc;
		end
	endgenerate

	wire rem_done = !rem_large && (fwd_rem <= eff_room);
	assign seg_n = rem_done ? fwd_rem : eff_room;
	assign seg_last = rem_done;

	/* The bytes back from the rightmost pixel, and the source window. */
	am_sub #(.W(US + 1)) s_bn (.a(sn), .b({(US + 1){1'b0}}), .bi(1'b1), .d(back_n), .bo(seg_co_unused[2]));
	assign back = (psl == 3'd4) ? {{(ADDR_W - US - 5){1'b0}}, back_n, 4'b0000} :
	              (psl == 3'd3) ? {{(ADDR_W - US - 4){1'b0}}, back_n, 3'b000}  :
	              (psl == 3'd2) ? {{(ADDR_W - US - 3){1'b0}}, back_n, 2'b00}   :
	              (psl == 3'd1) ? {{(ADDR_W - US - 2){1'b0}}, back_n, 1'b0}    :
	                              {{(ADDR_W - US - 1){1'b0}}, back_n};
	wire [7:0] snu_m1_8;
	wire snu_m1_bo;
	generate
		if (USE_SNU_M1_ONLY) begin : g_snum1_in
			assign snu_m1_8 = {{(7-US){1'b0}}, in_snu_m1};
			assign snu_m1_bo = 1'b0;
		end else begin : g_snum1_dyn
			wire [7:0] snu_m1_comb_8;
			am_sub #(.W(8)) s_snum1 (.a({{(7-US){1'b0}}, snu}), .b(8'd1), .bi(1'b0), .d(snu_m1_comb_8), .bo(snu_m1_bo));
			assign snu_m1_8 = use_snu_m1 ? {{(7-US){1'b0}}, in_snu_m1} : snu_m1_comb_8;
		end
	endgenerate
`ifdef SYNTH_KINTEX7
	/* 16+16 carry-select on Kintex-7 keeps CARRY4 Pre<=1, Post<=1 (Rule 1 clean). */
	wire sb_bo_lo, sb_bo_hi0, sb_bo_hi1;
	wire [ADDR_W-17:0] sb_hi0, sb_hi1;
	am_sub #(.W(16))        s_sb_lo  (.a(slo[15:0]),        .b({{(16-US){1'b0}}, dlo}), .bi(1'b0), .d(src_base[15:0]), .bo(sb_bo_lo));
	am_sub #(.W(ADDR_W-16)) s_sb_hi0 (.a(slo[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}),     .bi(1'b0), .d(sb_hi0),         .bo(sb_bo_hi0));
	am_sub #(.W(ADDR_W-16)) s_sb_hi1 (.a(slo[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}),     .bi(1'b1), .d(sb_hi1),         .bo(sb_bo_hi1));
	assign src_base[ADDR_W-1:16] = sb_bo_lo ? sb_hi1 : sb_hi0;
	assign seg_co_unused[3] = sb_bo_lo ? sb_bo_hi1 : sb_bo_hi0;

	wire [15:0] sl_op_lo = {{8{snu_m1_8[7]}}, snu_m1_8};
	wire [ADDR_W-17:0] sl_op_hi = {(ADDR_W-16){snu_m1_8[7]}};
	wire sl_c0, sl_c10, sl_c11;
	wire [ADDR_W-17:0] sl_hi0, sl_hi1;
	am_add #(.W(16))        a_sl_lo  (.a(slo[15:0]),        .b(sl_op_lo), .ci(1'b0), .s(src_last[15:0]), .co(sl_c0));
	am_add #(.W(ADDR_W-16)) a_sl_hi0 (.a(slo[ADDR_W-1:16]), .b(sl_op_hi), .ci(1'b0), .s(sl_hi0),         .co(sl_c10));
	am_add #(.W(ADDR_W-16)) a_sl_hi1 (.a(slo[ADDR_W-1:16]), .b(sl_op_hi), .ci(1'b1), .s(sl_hi1),         .co(sl_c11));
	assign src_last[ADDR_W-1:16] = sl_c0 ? sl_hi1 : sl_hi0;
	assign src_next = src_last;
	assign seg_co_unused[4] = sl_c0 ? sl_c11 : sl_c10;
	wire cs_b3_dummy = sb_bo_hi1 & sl_c11;
`elsif SYNTH_KINTEX_ULTRASCALE
	/* 16+16 carry-select on Kintex-7 keeps CARRY4 Pre<=1, Post<=1 (Rule 1 clean). */
	wire sb_bo_lo, sb_bo_hi0, sb_bo_hi1;
	wire [ADDR_W-17:0] sb_hi0, sb_hi1;
	am_sub #(.W(16))        s_sb_lo  (.a(slo[15:0]),        .b({{(16-US){1'b0}}, dlo}), .bi(1'b0), .d(src_base[15:0]), .bo(sb_bo_lo));
	am_sub #(.W(ADDR_W-16)) s_sb_hi0 (.a(slo[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}),     .bi(1'b0), .d(sb_hi0),         .bo(sb_bo_hi0));
	am_sub #(.W(ADDR_W-16)) s_sb_hi1 (.a(slo[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}),     .bi(1'b1), .d(sb_hi1),         .bo(sb_bo_hi1));
	assign src_base[ADDR_W-1:16] = sb_bo_lo ? sb_hi1 : sb_hi0;
	assign seg_co_unused[3] = sb_bo_lo ? sb_bo_hi1 : sb_bo_hi0;

	wire [15:0] sl_op_lo = {{8{snu_m1_8[7]}}, snu_m1_8};
	wire [ADDR_W-17:0] sl_op_hi = {(ADDR_W-16){snu_m1_8[7]}};
	wire sl_c0, sl_c10, sl_c11;
	wire [ADDR_W-17:0] sl_hi0, sl_hi1;
	am_add #(.W(16))        a_sl_lo  (.a(slo[15:0]),        .b(sl_op_lo), .ci(1'b0), .s(src_last[15:0]), .co(sl_c0));
	am_add #(.W(ADDR_W-16)) a_sl_hi0 (.a(slo[ADDR_W-1:16]), .b(sl_op_hi), .ci(1'b0), .s(sl_hi0),         .co(sl_c10));
	am_add #(.W(ADDR_W-16)) a_sl_hi1 (.a(slo[ADDR_W-1:16]), .b(sl_op_hi), .ci(1'b1), .s(sl_hi1),         .co(sl_c11));
	assign src_last[ADDR_W-1:16] = sl_c0 ? sl_hi1 : sl_hi0;
	assign src_next = src_last;
	assign seg_co_unused[4] = sl_c0 ? sl_c11 : sl_c10;
	wire cs_b3_dummy = sb_bo_hi1 & sl_c11;
`else
	/* ASIC SYNTH_ASIC: single Kogge-Stone am_sub / am_add (<= 10 cmos3). */
	am_sub #(.W(ADDR_W)) s_sb (.a(slo), .b({{(ADDR_W-US){1'b0}}, dlo}), .bi(1'b0), .d(src_base), .bo(seg_co_unused[3]));

	wire [7:0] sl_op_s = USE_SNU_M1_ONLY ? 8'd0 : {8{snu_m1_8[7]}};
	wire [ADDR_W-25:0] sl_op_hi = USE_SNU_M1_ONLY ? {(ADDR_W-24){1'b0}} : {(ADDR_W-24){snu_m1_8[7]}};
	wire [ADDR_W-1:0] sl_op_full = {sl_op_hi, sl_op_s, sl_op_s, snu_m1_8};
	am_add #(.W(ADDR_W)) a_sl (.a(slo), .b(sl_op_full), .ci(1'b0), .s(src_last), .co(seg_co_unused[4]));
	assign src_next = src_last;
	wire cs_b3_dummy = 1'b0;
`endif
	assign seg_co_unused[5] = snu_m1_bo;
	assign last_line = src_last[ADDR_W-1:US];

	/* The end of the write's bytes */
	localparam NUM_BE_GRP = LINE_BYTES / 8;
	integer u;
`ifdef SYNTH_KINTEX7
	wire pe_c0, pe_c10, pe_c11, pe_c20, pe_c21;
	wire [1:0] pe_m0, pe_m1;
	wire [US-4:0] pe_hi0, pe_hi1;
	am_add #(.W(2))    a_pe_0   (.a(p0[1:0]),             .b(snu[1:0]),    .ci(1'b0), .s(p_end[1:0]), .co(pe_c0));
	am_add #(.W(2))    a_pe_10  (.a(p0[3:2]),             .b(snu[3:2]),    .ci(1'b0), .s(pe_m0),      .co(pe_c10));
	am_add #(.W(2))    a_pe_11  (.a(p0[3:2]),             .b(snu[3:2]),    .ci(1'b1), .s(pe_m1),      .co(pe_c11));
	am_add #(.W(US-3)) a_pe_hi0 (.a({1'b0, p0[US-1:4]}),  .b(snu[US:4]),   .ci(1'b0), .s(pe_hi0),     .co(pe_c20));
	am_add #(.W(US-3)) a_pe_hi1 (.a({1'b0, p0[US-1:4]}),  .b(snu[US:4]),   .ci(1'b1), .s(pe_hi1),     .co(pe_c21));
	wire pe_c1 = pe_c10 | (pe_c11 & pe_c0);
	assign p_end[3:2]  = pe_c0 ? pe_m1  : pe_m0;
	assign p_end[US:4] = pe_c1 ? pe_hi1 : pe_hi0;
	assign seg_co_unused[6] = pe_c1 ? pe_c21 : pe_c20;
	assign cs_co_unused = {cs_b3_dummy, pe_c21, pe_c20, 1'b0};

	(* keep = "true" *) wire [US-1:0] p0_0, p0_1, p0_2, p0_3;
	genvar g_p0;
	generate
		for (g_p0 = 0; g_p0 < US; g_p0 = g_p0 + 1) begin : g_p0_bufs
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_p0_0 (.I0(p0[g_p0]), .O(p0_0[g_p0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_p0_1 (.I0(p0[g_p0]), .O(p0_1[g_p0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_p0_2 (.I0(p0[g_p0]), .O(p0_2[g_p0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_p0_3 (.I0(p0[g_p0]), .O(p0_3[g_p0]));
		end
	endgenerate
	wire [NUM_BE_GRP-1:0] pend_hi_gt, pend_hi_eq;
	wire [7:0] pend_lo_gt_0, pend_lo_gt_1, pend_lo_gt_2, pend_lo_gt_3;
	wire [2:0] pend_lo_0, pend_lo_1, pend_lo_2, pend_lo_3;
	genvar gp3;
	generate
		for (gp3 = 0; gp3 < 3; gp3 = gp3 + 1) begin : g_pend_lo_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pl0 (.I0(p_end[gp3]), .O(pend_lo_0[gp3]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pl1 (.I0(p_end[gp3]), .O(pend_lo_1[gp3]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pl2 (.I0(p_end[gp3]), .O(pend_lo_2[gp3]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pl3 (.I0(p_end[gp3]), .O(pend_lo_3[gp3]));
		end
	endgenerate
	genvar gb;
	generate
		for (gb = 0; gb < NUM_BE_GRP; gb = gb + 1) begin : g_pend_hi
			assign pend_hi_gt[gb] = (p_end[US:3] > gb[US-3:0]);
			assign pend_hi_eq[gb] = (p_end[US:3] == gb[US-3:0]);
		end
		for (gb = 0; gb < 7; gb = gb + 1) begin : g_pend_lo
			assign pend_lo_gt_0[gb] = (pend_lo_0 > gb[2:0]);
			assign pend_lo_gt_1[gb] = (pend_lo_1 > gb[2:0]);
			assign pend_lo_gt_2[gb] = (pend_lo_2 > gb[2:0]);
			assign pend_lo_gt_3[gb] = (pend_lo_3 > gb[2:0]);
		end
		assign pend_lo_gt_0[7] = 1'b0;
		assign pend_lo_gt_1[7] = 1'b0;
		assign pend_lo_gt_2[7] = 1'b0;
		assign pend_lo_gt_3[7] = 1'b0;
	endgenerate

	always @(*) begin
		for (u = 0; u < LINE_BYTES; u = u + 1) begin
			if (u < 16)
				be[u] = (u[US:0] >= {1'b0, p0_0} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_0[u & 7]));
			else if (u < 32)
				be[u] = (u[US:0] >= {1'b0, p0_1} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_1[u & 7]));
			else if (u < 48)
				be[u] = (u[US:0] >= {1'b0, p0_2} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_2[u & 7]));
			else
				be[u] = (u[US:0] >= {1'b0, p0_3} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_3[u & 7]));
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire pe_c0, pe_c10, pe_c11, pe_c20, pe_c21;
	wire [1:0] pe_m0, pe_m1;
	wire [US-4:0] pe_hi0, pe_hi1;
	am_add #(.W(2))    a_pe_0   (.a(p0[1:0]),             .b(snu[1:0]),    .ci(1'b0), .s(p_end[1:0]), .co(pe_c0));
	am_add #(.W(2))    a_pe_10  (.a(p0[3:2]),             .b(snu[3:2]),    .ci(1'b0), .s(pe_m0),      .co(pe_c10));
	am_add #(.W(2))    a_pe_11  (.a(p0[3:2]),             .b(snu[3:2]),    .ci(1'b1), .s(pe_m1),      .co(pe_c11));
	am_add #(.W(US-3)) a_pe_hi0 (.a({1'b0, p0[US-1:4]}),  .b(snu[US:4]),   .ci(1'b0), .s(pe_hi0),     .co(pe_c20));
	am_add #(.W(US-3)) a_pe_hi1 (.a({1'b0, p0[US-1:4]}),  .b(snu[US:4]),   .ci(1'b1), .s(pe_hi1),     .co(pe_c21));
	wire pe_c1 = pe_c10 | (pe_c11 & pe_c0);
	assign p_end[3:2]  = pe_c0 ? pe_m1  : pe_m0;
	assign p_end[US:4] = pe_c1 ? pe_hi1 : pe_hi0;
	assign seg_co_unused[6] = pe_c1 ? pe_c21 : pe_c20;
	assign cs_co_unused = {cs_b3_dummy, pe_c21, pe_c20, 1'b0};

	(* keep = "true" *) wire [US-1:0] p0_0, p0_1, p0_2, p0_3;
	genvar g_p0;
	generate
		for (g_p0 = 0; g_p0 < US; g_p0 = g_p0 + 1) begin : g_p0_bufs
			assign p0_0[g_p0] = p0[g_p0];
			assign p0_1[g_p0] = p0[g_p0];
			assign p0_2[g_p0] = p0[g_p0];
			assign p0_3[g_p0] = p0[g_p0];
		end
	endgenerate
	wire [NUM_BE_GRP-1:0] pend_hi_gt, pend_hi_eq;
	wire [7:0] pend_lo_gt_0, pend_lo_gt_1, pend_lo_gt_2, pend_lo_gt_3;
	wire [2:0] pend_lo_0, pend_lo_1, pend_lo_2, pend_lo_3;
	genvar gp3;
	generate
		for (gp3 = 0; gp3 < 3; gp3 = gp3 + 1) begin : g_pend_lo_buf
			assign pend_lo_0[gp3] = p_end[gp3];
			assign pend_lo_1[gp3] = p_end[gp3];
			assign pend_lo_2[gp3] = p_end[gp3];
			assign pend_lo_3[gp3] = p_end[gp3];
		end
	endgenerate
	genvar gb;
	generate
		for (gb = 0; gb < NUM_BE_GRP; gb = gb + 1) begin : g_pend_hi
			assign pend_hi_gt[gb] = (p_end[US:3] > gb[US-3:0]);
			assign pend_hi_eq[gb] = (p_end[US:3] == gb[US-3:0]);
		end
		for (gb = 0; gb < 7; gb = gb + 1) begin : g_pend_lo
			assign pend_lo_gt_0[gb] = (pend_lo_0 > gb[2:0]);
			assign pend_lo_gt_1[gb] = (pend_lo_1 > gb[2:0]);
			assign pend_lo_gt_2[gb] = (pend_lo_2 > gb[2:0]);
			assign pend_lo_gt_3[gb] = (pend_lo_3 > gb[2:0]);
		end
		assign pend_lo_gt_0[7] = 1'b0;
		assign pend_lo_gt_1[7] = 1'b0;
		assign pend_lo_gt_2[7] = 1'b0;
		assign pend_lo_gt_3[7] = 1'b0;
	endgenerate

	always @(*) begin
		for (u = 0; u < LINE_BYTES; u = u + 1) begin
			if (u < 16)
				be[u] = (u[US:0] >= {1'b0, p0_0} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_0[u & 7]));
			else if (u < 32)
				be[u] = (u[US:0] >= {1'b0, p0_1} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_1[u & 7]));
			else if (u < 48)
				be[u] = (u[US:0] >= {1'b0, p0_2} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_2[u & 7]));
			else
				be[u] = (u[US:0] >= {1'b0, p0_3} && !keep[u % 4]) &&
				        (pend_hi_gt[u >> 3] || (pend_hi_eq[u >> 3] && pend_lo_gt_3[u & 7]));
		end
	end
`else
	/* Carry-only constant-K prefix compare for pend_hi0/1_gt (<= 11 cmos3 end-to-end). */
	localparam NUM_BE_GRP2 = LINE_BYTES / 4;
	localparam W_PE_HI     = US - 2;
	wire pe_c0;
	am_add #(.W(2)) a_pe_0 (.a(p0[1:0]), .b(snu[1:0]), .ci(1'b0), .s(p_end[1:0]), .co(pe_c0));
	assign p_end[US:2] = {(US-1){1'b0}};
	assign seg_co_unused[6] = 1'b0;
	assign cs_co_unused = {cs_b3_dummy, 3'b000};

	wire [W_PE_HI-1:0] pe_ab_and = p0[US-1:2] & snu[US-1:2];
	wire [W_PE_HI-1:0] pe_ab_or  = p0[US-1:2] | snu[US-1:2];
	wire [NUM_BE_GRP2-1:0] pend_hi0_gt, pend_hi1_gt, pend_hi0_ge, pend_hi1_ge;
	wire [3:0] pend_lo_gt;
	genvar gb;
	generate
		for (gb = 0; gb < NUM_BE_GRP2; gb = gb + 1) begin : g_pend_hi
			localparam [W_PE_HI-1:0] K_HI = ~gb[W_PE_HI-1:0];
			wire [W_PE_HI-1:0] g_k = (K_HI & pe_ab_or) | ((~K_HI) & pe_ab_and);
			wire [W_PE_HI-1:0] p_k = K_HI | pe_ab_or;
			if (W_PE_HI == 4) begin : g_w4
				wire g32 = g_k[3] | (p_k[3] & g_k[2]);
				wire p32 = p_k[3] & p_k[2];
				wire g10 = g_k[1] | (p_k[1] & g_k[0]);
				wire p10 = p_k[1] & p_k[0];
				wire g30 = g32 | (p32 & g10);
				wire p30 = p32 & p10;
				assign pend_hi0_gt[gb] = snu[US] | g30;
				assign pend_hi1_gt[gb] = snu[US] | g30 | p30;
			end else begin : g_w3
				wire g21 = g_k[2] | (p_k[2] & g_k[1]);
				wire p21 = p_k[2] & p_k[1];
				wire g20 = g21 | (p21 & g_k[0]);
				wire p20 = p21 & p_k[0];
				assign pend_hi0_gt[gb] = snu[US] | g20;
				assign pend_hi1_gt[gb] = snu[US] | g20 | p20;
			end
			if (gb == 0) begin : g_ge0
				assign pend_hi0_ge[0] = 1'b1;
				assign pend_hi1_ge[0] = 1'b1;
			end else begin : g_gen
				assign pend_hi0_ge[gb] = pend_hi0_gt[gb - 1];
				assign pend_hi1_ge[gb] = pend_hi1_gt[gb - 1];
			end
		end
		for (gb = 0; gb < 3; gb = gb + 1) begin : g_pend_lo
			assign pend_lo_gt[gb] = (p_end[1:0] > gb[1:0]);
		end
		assign pend_lo_gt[3] = 1'b0;
	endgenerate

	wire [LINE_BYTES-1:0] be_c0, be_c1, be_ge_p0;
	genvar gu;
	generate
		for (gu = 0; gu < LINE_BYTES; gu = gu + 1) begin : g_be_pre
			localparam integer GRP = gu >> 2;
			localparam integer BIT = gu & 3;
			wire ge_p0_w = (gu == 0) ? (p0 == {US{1'b0}}) :
			               (gu == LINE_BYTES - 1) ? 1'b1 :
			               (gu[US-1:0] >= p0);
			assign be_ge_p0[gu] = ge_p0_w && !keep[gu % 4];
			if (BIT == 3) begin : g_b3
				assign be_c0[gu] = be_ge_p0[gu] && pend_hi0_gt[GRP];
				assign be_c1[gu] = be_ge_p0[gu] && pend_hi1_gt[GRP];
			end else begin : g_b02
				assign be_c0[gu] = be_ge_p0[gu] && (pend_hi0_gt[GRP] || (pend_hi0_ge[GRP] && pend_lo_gt[BIT]));
				assign be_c1[gu] = be_ge_p0[gu] && (pend_hi1_gt[GRP] || (pend_hi1_ge[GRP] && pend_lo_gt[BIT]));
			end
		end
	endgenerate

	always @(*) begin
		for (u = 0; u < LINE_BYTES; u = u + 1)
			be[u] = pe_c0 ? be_c1[u] : be_c0[u];
	end
	wire [US-2:0] p_end_hi_unused = p_end[US:2];
`endif

	wire unused_ok = &{1'b0, src_last[US-1:0], src_next, keep, seg_co_unused, cs_co_unused};
endmodule

`default_nettype wire
