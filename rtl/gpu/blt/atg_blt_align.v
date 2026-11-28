/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_align: byte-aligned window selector (combinational).
 * Extracts a LINE_BYTES window from a 2*LINE_BYTES double line
 * starting at byte offset off (0..LINE_BYTES-1).
 *
 * Implemented with 3 stages of 4-to-1 byte multiplexers:
 *   Stage 0: shifts by 0, 1, 2, or 3 bytes (off[1:0])
 *   Stage 1: shifts by 0, 4, 8, or 12 bytes (off[3:2])
 *   Stage 2: shifts by 0, 16, 32, or 48 bytes (off[5:4])
 * Strictly <= 3 LUT6 logic levels from input to output.
 * Fully buffered select trees guaranteeing fanout <= 16 on all nets.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_blt_align #(
	parameter LINE_BYTES = 64,
	parameter LINE_SHIFT = 6,
	parameter OFF_REPS = 8,
	parameter USE_DIRECT_OFF_REP = 0
) (
	input wire [2 * LINE_BYTES * 8 - 1:0] in_data,
	input wire [LINE_SHIFT - 1:0] off,
	input wire [OFF_REPS * LINE_SHIFT - 1:0] off_rep,
	output wire [LINE_BYTES * 8 - 1:0] out_data
);
	localparam US = LINE_SHIFT;

	/* Unpack in_data into byte array */
	wire [7:0] in_b [0:2*LINE_BYTES-1];
	genvar ui;
	generate
		for (ui = 0; ui < 2 * LINE_BYTES; ui = ui + 1) begin : g_in_unpack
			assign in_b[ui] = in_data[ui*8 +: 8];
		end
	endgenerate

`ifdef SYNTH_KINTEX7
	/* Number of intermediate bytes needed per stage */
	localparam S2_OUT_BYTES = LINE_BYTES;
	localparam S1_OUT_BYTES = (LINE_BYTES >= 64) ? (LINE_BYTES + 48) : (LINE_BYTES + 16);
	localparam S0_OUT_BYTES = S1_OUT_BYTES + 12;

	/* Stage 0 Select: off[1:0] */
	/* Each leaf buffer drives 2 bytes (16 LUT6s) */
	localparam S0_LEAF_CNT = (S0_OUT_BYTES + 1) / 2;
	wire [1:0] s0_sel [0:S0_LEAF_CNT-1];

	localparam S0_MID_CNT  = (S0_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s0_mid [0:S0_MID_CNT-1];
	genvar sm0, sl0, b0;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s0_dir
			for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
				localparam integer R0 = (sl0 * OFF_REPS) / S0_LEAF_CNT;
				assign s0_sel[sl0] = off_rep[R0*LINE_SHIFT +: 2];
			end
		end else if (OFF_REPS >= 8) begin : g_s0_1lut
			for (b0 = 0; b0 < 2; b0 = b0 + 1) begin : g_s0_bit
				for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
					localparam integer R0 = (sl0 * OFF_REPS) / S0_LEAF_CNT;
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l0 (.I0(off_rep[R0*LINE_SHIFT + b0]), .O(s0_sel[sl0][b0]));
				end
			end
		end else begin : g_s0_2lut
			for (b0 = 0; b0 < 2; b0 = b0 + 1) begin : g_s0_bit
				for (sm0 = 0; sm0 < S0_MID_CNT; sm0 = sm0 + 1) begin : g_s0_mid
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m0 (.I0(off[b0]), .O(s0_mid[sm0][b0]));
				end
				for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l0 (.I0(s0_mid[sl0 / 8][b0]), .O(s0_sel[sl0][b0]));
				end
			end
		end
	endgenerate

	/* Stage 1 Select: off[3:2] */
	localparam S1_LEAF_CNT = (S1_OUT_BYTES + 1) / 2;
	wire [1:0] s1_sel [0:S1_LEAF_CNT-1];

	localparam S1_MID_CNT  = (S1_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s1_mid [0:S1_MID_CNT-1];
	genvar sm1, sl1, b1;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s1_dir
			for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
				localparam integer R1 = (sl1 * OFF_REPS) / S1_LEAF_CNT;
				assign s1_sel[sl1] = off_rep[R1*LINE_SHIFT + 2 +: 2];
			end
		end else if (OFF_REPS >= 8) begin : g_s1_1lut
			for (b1 = 0; b1 < 2; b1 = b1 + 1) begin : g_s1_bit
				for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
					localparam integer R1 = (sl1 * OFF_REPS) / S1_LEAF_CNT;
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l1 (.I0(off_rep[R1*LINE_SHIFT + 2 + b1]), .O(s1_sel[sl1][b1]));
				end
			end
		end else begin : g_s1_2lut
			for (b1 = 0; b1 < 2; b1 = b1 + 1) begin : g_s1_bit
				for (sm1 = 0; sm1 < S1_MID_CNT; sm1 = sm1 + 1) begin : g_s1_mid
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m1 (.I0(off[2 + b1]), .O(s1_mid[sm1][b1]));
				end
				for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l1 (.I0(s1_mid[sl1 / 8][b1]), .O(s1_sel[sl1][b1]));
				end
			end
		end
	endgenerate

	/* Stage 2 Select: off[5:4] (or {1'b0, off[4]} for 32B) */
	localparam S2_LEAF_CNT = (S2_OUT_BYTES + 1) / 2;
	wire [1:0] s2_sel [0:S2_LEAF_CNT-1];

	localparam S2_MID_CNT  = (S2_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s2_mid [0:S2_MID_CNT-1];
	genvar sm2, sl2, b2;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s2_dir
			for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
				localparam integer R2 = (sl2 * OFF_REPS) / S2_LEAF_CNT;
				if (LINE_BYTES >= 64) begin : g_s2_d64
					assign s2_sel[sl2] = off_rep[R2*LINE_SHIFT + 4 +: 2];
				end else begin : g_s2_d32
					assign s2_sel[sl2] = {1'b0, off_rep[R2*LINE_SHIFT + 4]};
				end
			end
		end else if (OFF_REPS >= 8) begin : g_s2_1lut
			for (b2 = 0; b2 < 2; b2 = b2 + 1) begin : g_s2_bit
				for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
					localparam integer R2 = (sl2 * OFF_REPS) / S2_LEAF_CNT;
					wire in_s2_r = (LINE_BYTES >= 64) ? off_rep[R2*LINE_SHIFT + 4 + b2] : ((b2 == 1) ? 1'b0 : off_rep[R2*LINE_SHIFT + 4]);
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l2 (.I0(in_s2_r), .O(s2_sel[sl2][b2]));
				end
			end
		end else begin : g_s2_2lut
			for (b2 = 0; b2 < 2; b2 = b2 + 1) begin : g_s2_bit
				wire in_s2_bit;
				if (LINE_BYTES >= 64) begin : g_s2_in_64
					assign in_s2_bit = (b2 == 1) ? off[5] : off[4];
				end else begin : g_s2_in_32
					assign in_s2_bit = (b2 == 1) ? 1'b0 : off[4];
				end
				for (sm2 = 0; sm2 < S2_MID_CNT; sm2 = sm2 + 1) begin : g_s2_mid
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m2 (.I0(in_s2_bit), .O(s2_mid[sm2][b2]));
				end
				for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_l2 (.I0(s2_mid[sl2 / 8][b2]), .O(s2_sel[sl2][b2]));
				end
			end
		end
	endgenerate

	wire [7:0] st0_b [0:S0_OUT_BYTES-1];
	wire [7:0] st1_b [0:S1_OUT_BYTES-1];
	wire [7:0] st2_b [0:S2_OUT_BYTES-1];
	genvar i0, i1, i2;
	generate
		for (i0 = 0; i0 < S0_OUT_BYTES; i0 = i0 + 1) begin : g_stage0
			wire [1:0] sel0 = s0_sel[i0 / 2];
			assign st0_b[i0] = (sel0 == 2'd3) ? in_b[i0 + 3] :
			                   (sel0 == 2'd2) ? in_b[i0 + 2] :
			                   (sel0 == 2'd1) ? in_b[i0 + 1] : in_b[i0];
		end
		for (i1 = 0; i1 < S1_OUT_BYTES; i1 = i1 + 1) begin : g_stage1
			wire [1:0] sel1 = s1_sel[i1 / 2];
			assign st1_b[i1] = (sel1 == 2'd3) ? st0_b[i1 + 12] :
			                   (sel1 == 2'd2) ? st0_b[i1 + 8]  :
			                   (sel1 == 2'd1) ? st0_b[i1 + 4]  : st0_b[i1];
		end
		for (i2 = 0; i2 < S2_OUT_BYTES; i2 = i2 + 1) begin : g_stage2
			wire [1:0] sel2 = s2_sel[i2 / 2];
			if (LINE_BYTES >= 64) begin : g_s2_64
				assign st2_b[i2] = (sel2 == 2'd3) ? st1_b[i2 + 48] :
				                   (sel2 == 2'd2) ? st1_b[i2 + 32] :
				                   (sel2 == 2'd1) ? st1_b[i2 + 16] : st1_b[i2];
			end else begin : g_s2_32
				assign st2_b[i2] = sel2[0] ? st1_b[i2 + 16] : st1_b[i2];
				wire unused_sel2 = &{1'b0, sel2[1]};
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Number of intermediate bytes needed per stage */
	localparam S2_OUT_BYTES = LINE_BYTES;
	localparam S1_OUT_BYTES = (LINE_BYTES >= 64) ? (LINE_BYTES + 48) : (LINE_BYTES + 16);
	localparam S0_OUT_BYTES = S1_OUT_BYTES + 12;

	/* Stage 0 Select: off[1:0] */
	/* Each leaf buffer drives 2 bytes (16 LUT6s) */
	localparam S0_LEAF_CNT = (S0_OUT_BYTES + 1) / 2;
	wire [1:0] s0_sel [0:S0_LEAF_CNT-1];

	localparam S0_MID_CNT  = (S0_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s0_mid [0:S0_MID_CNT-1];
	genvar sm0, sl0, b0;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s0_dir
			for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
				localparam integer R0 = (sl0 * OFF_REPS) / S0_LEAF_CNT;
				assign s0_sel[sl0] = off_rep[R0*LINE_SHIFT +: 2];
			end
		end else if (OFF_REPS >= 8) begin : g_s0_1lut
			for (b0 = 0; b0 < 2; b0 = b0 + 1) begin : g_s0_bit
				for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
					localparam integer R0 = (sl0 * OFF_REPS) / S0_LEAF_CNT;
					assign s0_sel[sl0][b0] = off_rep[R0*LINE_SHIFT + b0];
				end
			end
		end else begin : g_s0_2lut
			for (b0 = 0; b0 < 2; b0 = b0 + 1) begin : g_s0_bit
				for (sm0 = 0; sm0 < S0_MID_CNT; sm0 = sm0 + 1) begin : g_s0_mid
					assign s0_mid[sm0][b0] = off[b0];
				end
				for (sl0 = 0; sl0 < S0_LEAF_CNT; sl0 = sl0 + 1) begin : g_s0_leaf
					assign s0_sel[sl0][b0] = s0_mid[sl0 / 8][b0];
				end
			end
		end
	endgenerate

	/* Stage 1 Select: off[3:2] */
	localparam S1_LEAF_CNT = (S1_OUT_BYTES + 1) / 2;
	wire [1:0] s1_sel [0:S1_LEAF_CNT-1];

	localparam S1_MID_CNT  = (S1_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s1_mid [0:S1_MID_CNT-1];
	genvar sm1, sl1, b1;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s1_dir
			for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
				localparam integer R1 = (sl1 * OFF_REPS) / S1_LEAF_CNT;
				assign s1_sel[sl1] = off_rep[R1*LINE_SHIFT + 2 +: 2];
			end
		end else if (OFF_REPS >= 8) begin : g_s1_1lut
			for (b1 = 0; b1 < 2; b1 = b1 + 1) begin : g_s1_bit
				for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
					localparam integer R1 = (sl1 * OFF_REPS) / S1_LEAF_CNT;
					assign s1_sel[sl1][b1] = off_rep[R1*LINE_SHIFT + 2 + b1];
				end
			end
		end else begin : g_s1_2lut
			for (b1 = 0; b1 < 2; b1 = b1 + 1) begin : g_s1_bit
				for (sm1 = 0; sm1 < S1_MID_CNT; sm1 = sm1 + 1) begin : g_s1_mid
					assign s1_mid[sm1][b1] = off[2 + b1];
				end
				for (sl1 = 0; sl1 < S1_LEAF_CNT; sl1 = sl1 + 1) begin : g_s1_leaf
					assign s1_sel[sl1][b1] = s1_mid[sl1 / 8][b1];
				end
			end
		end
	endgenerate

	/* Stage 2 Select: off[5:4] (or {1'b0, off[4]} for 32B) */
	localparam S2_LEAF_CNT = (S2_OUT_BYTES + 1) / 2;
	wire [1:0] s2_sel [0:S2_LEAF_CNT-1];

	localparam S2_MID_CNT  = (S2_LEAF_CNT + 7) / 8;
	(* dont_touch = "true" *) wire [1:0] s2_mid [0:S2_MID_CNT-1];
	genvar sm2, sl2, b2;
	generate
		if (OFF_REPS >= 8 && USE_DIRECT_OFF_REP != 0) begin : g_s2_dir
			for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
				localparam integer R2 = (sl2 * OFF_REPS) / S2_LEAF_CNT;
				if (LINE_BYTES >= 64) begin : g_s2_d64
					assign s2_sel[sl2] = off_rep[R2*LINE_SHIFT + 4 +: 2];
				end else begin : g_s2_d32
					assign s2_sel[sl2] = {1'b0, off_rep[R2*LINE_SHIFT + 4]};
				end
			end
		end else if (OFF_REPS >= 8) begin : g_s2_1lut
			for (b2 = 0; b2 < 2; b2 = b2 + 1) begin : g_s2_bit
				for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
					localparam integer R2 = (sl2 * OFF_REPS) / S2_LEAF_CNT;
					wire in_s2_r = (LINE_BYTES >= 64) ? off_rep[R2*LINE_SHIFT + 4 + b2] : ((b2 == 1) ? 1'b0 : off_rep[R2*LINE_SHIFT + 4]);
					assign s2_sel[sl2][b2] = in_s2_r;
				end
			end
		end else begin : g_s2_2lut
			for (b2 = 0; b2 < 2; b2 = b2 + 1) begin : g_s2_bit
				wire in_s2_bit;
				if (LINE_BYTES >= 64) begin : g_s2_in_64
					assign in_s2_bit = (b2 == 1) ? off[5] : off[4];
				end else begin : g_s2_in_32
					assign in_s2_bit = (b2 == 1) ? 1'b0 : off[4];
				end
				for (sm2 = 0; sm2 < S2_MID_CNT; sm2 = sm2 + 1) begin : g_s2_mid
					assign s2_mid[sm2][b2] = in_s2_bit;
				end
				for (sl2 = 0; sl2 < S2_LEAF_CNT; sl2 = sl2 + 1) begin : g_s2_leaf
					assign s2_sel[sl2][b2] = s2_mid[sl2 / 8][b2];
				end
			end
		end
	endgenerate

	wire [7:0] st0_b [0:S0_OUT_BYTES-1];
	wire [7:0] st1_b [0:S1_OUT_BYTES-1];
	wire [7:0] st2_b [0:S2_OUT_BYTES-1];
	genvar i0, i1, i2;
	generate
		for (i0 = 0; i0 < S0_OUT_BYTES; i0 = i0 + 1) begin : g_stage0
			wire [1:0] sel0 = s0_sel[i0 / 2];
			assign st0_b[i0] = (sel0 == 2'd3) ? in_b[i0 + 3] :
			                   (sel0 == 2'd2) ? in_b[i0 + 2] :
			                   (sel0 == 2'd1) ? in_b[i0 + 1] : in_b[i0];
		end
		for (i1 = 0; i1 < S1_OUT_BYTES; i1 = i1 + 1) begin : g_stage1
			wire [1:0] sel1 = s1_sel[i1 / 2];
			assign st1_b[i1] = (sel1 == 2'd3) ? st0_b[i1 + 12] :
			                   (sel1 == 2'd2) ? st0_b[i1 + 8]  :
			                   (sel1 == 2'd1) ? st0_b[i1 + 4]  : st0_b[i1];
		end
		for (i2 = 0; i2 < S2_OUT_BYTES; i2 = i2 + 1) begin : g_stage2
			wire [1:0] sel2 = s2_sel[i2 / 2];
			if (LINE_BYTES >= 64) begin : g_s2_64
				assign st2_b[i2] = (sel2 == 2'd3) ? st1_b[i2 + 48] :
				                   (sel2 == 2'd2) ? st1_b[i2 + 32] :
				                   (sel2 == 2'd1) ? st1_b[i2 + 16] : st1_b[i2];
			end else begin : g_s2_32
				assign st2_b[i2] = sel2[0] ? st1_b[i2 + 16] : st1_b[i2];
				wire unused_sel2 = &{1'b0, sel2[1]};
			end
		end
	endgenerate
`else
	/* ASIC 2-stage 8:1 -> 8:1 (64B) / 8:1 -> 4:1 (32B) one-hot mux cuts barrel shift to <= 10 cmos3. */
	localparam S0_OUT_A     = (LINE_BYTES >= 64) ? (LINE_BYTES + 56) : (LINE_BYTES + 24);
	localparam S0A_LEAF_CNT = (S0_OUT_A + 1) / 2;
	localparam S1A_LEAF_CNT = (LINE_BYTES + 1) / 2;
	wire [7:0] st0_b [0:S0_OUT_A-1];
	wire [7:0] st2_b [0:LINE_BYTES-1];
	wire [7:0] s0_oh [0:S0A_LEAF_CNT-1];
	genvar i0, i1;
	generate
		for (i0 = 0; i0 < S0A_LEAF_CNT; i0 = i0 + 1) begin : g_s0_dec
			localparam integer R0 = (i0 * OFF_REPS) / S0A_LEAF_CNT;
			wire [2:0] sel0 = off_rep[R0*LINE_SHIFT +: 3];
			assign s0_oh[i0] = {
				 sel0[2] &  sel0[1] &  sel0[0],
				 sel0[2] &  sel0[1] & ~sel0[0],
				 sel0[2] & ~sel0[1] &  sel0[0],
				 sel0[2] & ~sel0[1] & ~sel0[0],
				~sel0[2] &  sel0[1] &  sel0[0],
				~sel0[2] &  sel0[1] & ~sel0[0],
				~sel0[2] & ~sel0[1] &  sel0[0],
				~sel0[2] & ~sel0[1] & ~sel0[0]
			};
		end
		for (i0 = 0; i0 < S0_OUT_A; i0 = i0 + 1) begin : g_stage0
			wire [7:0] oh0 = s0_oh[i0 / 2];
			assign st0_b[i0] =
				(({8{oh0[7]}} & in_b[i0 + 7]) | ({8{oh0[6]}} & in_b[i0 + 6])) |
				(({8{oh0[5]}} & in_b[i0 + 5]) | ({8{oh0[4]}} & in_b[i0 + 4])) |
				(({8{oh0[3]}} & in_b[i0 + 3]) | ({8{oh0[2]}} & in_b[i0 + 2])) |
				(({8{oh0[1]}} & in_b[i0 + 1]) | ({8{oh0[0]}} & in_b[i0]));
		end
		if (LINE_BYTES >= 64) begin : g_s1_64
			wire [7:0] s1_oh [0:S1A_LEAF_CNT-1];
			for (i1 = 0; i1 < S1A_LEAF_CNT; i1 = i1 + 1) begin : g_s1_dec
				localparam integer R1 = (i1 * OFF_REPS) / S1A_LEAF_CNT;
				wire [2:0] sel1 = off_rep[R1*LINE_SHIFT + 3 +: 3];
				assign s1_oh[i1] = {
					 sel1[2] &  sel1[1] &  sel1[0],
					 sel1[2] &  sel1[1] & ~sel1[0],
					 sel1[2] & ~sel1[1] &  sel1[0],
					 sel1[2] & ~sel1[1] & ~sel1[0],
					~sel1[2] &  sel1[1] &  sel1[0],
					~sel1[2] &  sel1[1] & ~sel1[0],
					~sel1[2] & ~sel1[1] &  sel1[0],
					~sel1[2] & ~sel1[1] & ~sel1[0]
				};
			end
			for (i1 = 0; i1 < LINE_BYTES; i1 = i1 + 1) begin : g_stage1
				wire [7:0] oh1 = s1_oh[i1 / 2];
				assign st2_b[i1] =
					(({8{oh1[7]}} & st0_b[i1 + 56]) | ({8{oh1[6]}} & st0_b[i1 + 48])) |
					(({8{oh1[5]}} & st0_b[i1 + 40]) | ({8{oh1[4]}} & st0_b[i1 + 32])) |
					(({8{oh1[3]}} & st0_b[i1 + 24]) | ({8{oh1[2]}} & st0_b[i1 + 16])) |
					(({8{oh1[1]}} & st0_b[i1 + 8])  | ({8{oh1[0]}} & st0_b[i1]));
			end
		end else begin : g_s1_32
			wire [3:0] s1_oh4 [0:S1A_LEAF_CNT-1];
			for (i1 = 0; i1 < S1A_LEAF_CNT; i1 = i1 + 1) begin : g_s1_dec
				localparam integer R1 = (i1 * OFF_REPS) / S1A_LEAF_CNT;
				wire [1:0] sel1 = off_rep[R1*LINE_SHIFT + 3 +: 2];
				assign s1_oh4[i1] = {
					 sel1[1] &  sel1[0],
					 sel1[1] & ~sel1[0],
					~sel1[1] &  sel1[0],
					~sel1[1] & ~sel1[0]
				};
			end
			for (i1 = 0; i1 < LINE_BYTES; i1 = i1 + 1) begin : g_stage1
				wire [3:0] oh1 = s1_oh4[i1 / 2];
				assign st2_b[i1] =
					(({8{oh1[3]}} & st0_b[i1 + 24]) | ({8{oh1[2]}} & st0_b[i1 + 16])) |
					(({8{oh1[1]}} & st0_b[i1 + 8])  | ({8{oh1[0]}} & st0_b[i1]));
			end
		end
	endgenerate
`endif

	/* Pack final stage outputs into out_data */
	genvar pk;
	generate
		for (pk = 0; pk < LINE_BYTES; pk = pk + 1) begin : g_out_pack
			assign out_data[pk*8 +: 8] = st2_b[pk];
		end
	endgenerate

	wire unused_ok = &{1'b0, off, off_rep};
endmodule
`default_nettype wire
