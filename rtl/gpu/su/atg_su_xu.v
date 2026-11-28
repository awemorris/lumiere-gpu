/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_xu: The SU's shared transcendental unit: rcp, rsq (table + one
 * Newton step), exp2, log2, sin, cos (range reduction + polynomial) and
 * fsqrt; one lane at a time, results as long beats.
 *
 * Takes one lane group at a time (x_ready is 1 only in IDLE) and
 * computes its EXEC lanes one after the other in a sequential engine,
 * then returns the group as one long beat (l_wr 1, l_last = the group's
 * x_last, class x); a group without EXEC lanes returns at once.  The
 * algorithms are integer and exact step for step with the reference model gsi1fp
 * xu() (the model the SU testbench compares with):
 *   rcp    2^50 / m by a 28-step restoring division, RNE with the
 *          remainder as sticky (correctly rounded);
 *   fsqrt  isqrt(m << 26) bit by bit (26 steps), correctly rounded;
 *   rsq    t = isqrt(m << 34) (30 steps), then 2^57 / t (29 steps);
 *   exp2   x in Q8.30, 2^frac as the product of 2^(2^-i) (30 steps, one
 *          multiply each), the integer part in the exponent;
 *   log2   the mantissa squared 30 times (one bit of the fraction each),
 *          the exponent as the integer part (absolute error about 2^-22);
 *   sin, cos  x / (2 pi) in turns (Q0.40, mod 1), the quadrant, then 26
 *          CORDIC rotations; |x| < 2^-12 gives x and 1.0 (absolute error
 *          about 2^-24 on [-pi, pi]).
 * Inputs are flushed (denormals as zeros); results through am_fp_rnd
 * (RNE, flushed).  Constants: include/atg_su_xu_tab.vh.
 *
 * States: IDLE →(group)→ LOAD → STEP (per op) → FIN → (next lane | RET)
 * →(l_ready)→ IDLE.
*/

`timescale 1ns / 1ps
`default_nettype none

module atg_su_xu #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
) (
	input wire clk,
	input wire rst_n,
	/* Issued instruction, one lane group per cycle (from atg_su_opd, shared by all units) */
	input wire x_valid,
	input wire [WARP_W-1:0] x_warp,
	input wire [GRP_W-1:0] x_grp,
	input wire x_last,
	input wire [SBE_W:0] x_sbe,
	input wire [15:0] x_pc,
	input wire [UOP_W-1:0] x_uop,
	input wire [NPHYS*32-1:0] x_a,
	input wire [NPHYS*32-1:0] x_b,
	input wire [NPHYS*32-1:0] x_c,
	input wire [2*NPHYS-1:0] x_mask,
	input wire [31:0] x_info,
	output wire x_ready,
	/* Long result to atg_su_wb (any time; wb may refuse with l_ready = 0) */
	output wire l_valid,
	input wire l_ready,
	output wire [WARP_W-1:0] l_warp,
	output wire [GRP_W-1:0] l_grp,
	output wire [6:0] l_rd,
	output wire [NPHYS*32-1:0] l_data,
	output wire [NPHYS-1:0] l_lanes,
	output wire l_wr,
	output wire l_last,
	output wire [SBE_W:0] l_sbe,
	output wire [2:0] l_cls
);
`include "atg_su_uop.vh"
`include "atg_su_fp.vh"
`include "atg_su_xu_tab.vh"

	localparam DW = NPHYS * 32;
	localparam LW = NPHYS > 1 ? $clog2(NPHYS) : 1;
	localparam [2:0] IDLE = 3'd0, LOAD = 3'd1, STEP = 3'd2, FIN = 3'd3, STORE = 3'd4, RET = 3'd5;

	(* dont_touch = "true" *) wire rst_p = !rst_n;

	(* dont_touch = "true" *) reg [2:0] st, st_cs, st_g, st_d;
	wire [DW-1:0] rx, rq;
	reg [NPHYS-1:0] rmask;
	(* dont_touch = "true" *) reg [2:0] rop;
	(* dont_touch = "true" *) reg [2:0] rop_fin_lo0, rop_fin_lo1, rop_fin_hi0, rop_fin_hi1, rop_fin_ctl;
	(* dont_touch = "true" *) reg [2:0] rop_pipe, rop_we, rop_step;
	(* dont_touch = "true" *) reg rop_we_is_exp2_r, rop_we_is_log2_r, rop_we_is_cordic_r;
	(* dont_touch = "true" *) reg rop_we_is_rcp_r, rop_we_is_rsq_fsq_r, rop_we_is_div_r;
	(* dont_touch = "true" *) reg [2:0] rop_dt, rop_dt_dtr_lo, rop_dt_dtr_hi, rop_dt_exab, rop_dt_tt;
	(* dont_touch = "true" *) reg [2:0] rop_load, rop_load_ctl_lo, rop_load_ctl_hi, rop_load_mb, rop_load_y, rop_load_fr;
	reg [5:0] nsteps_m1_r, nsteps_init_r;
	reg       rop_is_rsq_r;
	reg [6:0] rrd;
	reg [WARP_W-1:0] rwarp;
	reg [GRP_W-1:0] rgrp;
	reg rlast;
	reg [SBE_W:0] rsbe;
	(* dont_touch = "true" *) reg [LW-1:0] li, li_rq_0, li_rq_1;
	(* dont_touch = "true" *) reg [5:0] step;
	(* dont_touch = "true" *) reg [5:0] step_cnt, step_cnt_b, step_cordic, step_tab, step_fbit_lo, step_fbit_hi;
	(* dont_touch = "true" *) reg [5:0] step_tab_atan, step_tab_expk;
	(* dont_touch = "true" *) reg ph;			/* rsq: 0 square root, 1 division */
	/* The engine (registers sliced to bound fanout <= 16). */
	wire [59:0] n;		/* sqrt: the radicand, two bits per step from the top */
	wire [33:0] rem;		/* division and sqrt remainder */
	wire [29:0] root;	/* quotient / root */
	reg stk;		/* rsq: the square root was inexact */
	wire [41:0] y;		/* exp2 / log2 */
	wire [29:0] fr;		/* exp2: the fraction bits; log2: the result bits */
	reg signed [11:0] ex;	/* the exponent of the result's LSB */
	wire signed [35:0] cx, cy;
	wire signed [40:0] z;
	reg qneg;		/* sin/cos: the quadrant's sign */
	(* dont_touch = "true" *) reg sres;		/* a special result is in res */
	reg [31:0] res;

	wire mine = x_valid && x_uop[ATG_UOP_UNIT +: 4] == ATG_UNIT_XU;
	assign x_ready = st == IDLE;

	wire [27:0] carry_unused;	/* the adders' carries and borrows */

	wire [NPHYS-1:0] x_mask_rev;
	wire [NPHYS-1:0] rm_after;
	wire [NPHYS-1:0] rm_after_rev;
	genvar gxr;
	generate
		for (gxr = 0; gxr < NPHYS; gxr = gxr + 1) begin : g_xu_rev
			if (gxr == 0) begin : g_xu_z
				assign rm_after[gxr] = 1'b0;
			end else begin : g_xu_nz
				assign rm_after[gxr] = rmask[gxr] && ({{(32-LW){1'b0}}, li} < gxr);
			end
			assign x_mask_rev[gxr]   = x_mask[NPHYS - 1 - gxr];
			assign rm_after_rev[gxr] = rm_after[NPHYS - 1 - gxr];
		end
	endgenerate
	wire [LW-1:0] first_rev_n, after_rev_n;
	wire first_nz, after_nz;
	am_msb #(.W(NPHYS), .SW(LW)) u_xu_first (.a(x_mask_rev),   .n(first_rev_n), .nz(first_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_xu_after (.a(rm_after_rev), .n(after_rev_n), .nz(after_nz));
	wire [LW:0] first = first_nz ? {1'b0, ~first_rev_n} : NPHYS[LW:0];
	wire [LW:0] after = after_nz ? {1'b0, ~after_rev_n} : NPHYS[LW:0];
	(* keep = "true" *) reg [LW-1:0] after_q_0, after_q_1, after_q_2, after_q_3;
	(* keep = "true" *) reg after_nz_q;
	assign carry_unused[0] = 1'b0;
	/* The lane's input, flushed, and its fields (registered in sub_st 0). */
	wire [31:0] lx_cur_r;
	wire [31:0] lx_raw = atg_su_flush(lx_cur_r);
	reg [31:0] lx;
	reg [23:0] lm_r;
	(* dont_touch = "true" *) reg [3:0] sub_st;
	(* dont_touch = "true" *) reg [3:0] sub_st_load, sub_st_load_b, sub_st_step, sub_st_fin, sub_st_fin_b;
	wire ls = lx[31];
	wire ls_ev, ls_t1, ls_res;
	(* dont_touch = "true" *) reg  ls_ev_0_r, ls_ev_1_r, ls_ev_2_r;
	(* dont_touch = "true" *) reg  ls_t1_0_r, ls_t1_1_r, ls_t1_2_r;
	wire ls_ev_2a, ls_ev_2b, ls_t1_2a, ls_t1_2b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsev   (.I0(lx[31]),    .O(ls_ev));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lst1   (.I0(lx[31]),    .O(ls_t1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsrs   (.I0(lx[31]),    .O(ls_res));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsev2a (.I0(ls_ev_2_r), .O(ls_ev_2a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsev2b (.I0(ls_ev_2_r), .O(ls_ev_2b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lst12a (.I0(ls_t1_2_r), .O(ls_t1_2a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lst12b (.I0(ls_t1_2_r), .O(ls_t1_2b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ls_ev = lx[31];
	assign ls_t1 = lx[31];
	assign ls_res = lx[31];
	assign ls_ev_2a = ls_ev_2_r;
	assign ls_ev_2b = ls_ev_2_r;
	assign ls_t1_2a = ls_t1_2_r;
	assign ls_t1_2b = ls_t1_2_r;
`else
	assign ls_ev    = lx[31];
	assign ls_t1    = lx[31];
	assign ls_res   = lx[31];
	assign ls_ev_2a = ls_ev_2_r;
	assign ls_ev_2b = ls_ev_2_r;
	assign ls_t1_2a = ls_t1_2_r;
	assign ls_t1_2b = ls_t1_2_r;
`endif

	wire [7:0] le_sub, le_cmp;
`ifdef SYNTH_KINTEX7
	genvar gle;
	generate
		for (gle = 0; gle < 8; gle = gle + 1) begin : g_le_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_les (.I0(lx[23+gle]), .O(le_sub[gle]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lec (.I0(lx[23+gle]), .O(le_cmp[gle]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gle;
	generate
		for (gle = 0; gle < 8; gle = gle + 1) begin : g_le_buf
			assign le_sub[gle] = lx[23+gle];
			assign le_cmp[gle] = lx[23+gle];
		end
	endgenerate
`else
	assign le_sub = lx[30:23];
	assign le_cmp = lx[30:23];
`endif

	wire [7:0] le = le_sub;
	wire [23:0] lm = lm_r;
	wire [23:0] lm_lmm, lm_ev0, lm_dtr, lm_y;
	genvar glm;
	generate
		for (glm = 0; glm < 24; glm = glm + 1) begin : g_lm_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lml (.I0(lm_r[glm]), .O(lm_lmm[glm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lme (.I0(lm_r[glm]), .O(lm_ev0[glm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lmd (.I0(lm_r[glm]), .O(lm_dtr[glm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lmy (.I0(lm_r[glm]), .O(lm_y[glm]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign lm_lmm[glm] = lm_r[glm];
			assign lm_ev0[glm] = lm_r[glm];
			assign lm_dtr[glm] = lm_r[glm];
			assign lm_y[glm] = lm_r[glm];
`else
			assign lm_lmm[glm] = lm_r[glm];
			assign lm_ev0[glm] = lm_r[glm];
			assign lm_dtr[glm] = lm_r[glm];
			assign lm_y[glm]   = lm_r[glm];
`endif
		end
	endgenerate
	wire lnan = le_cmp == 8'hFF && lx[22:0] != 0;
	wire linf = le_cmp == 8'hFF && lx[22:0] == 0;
	wire lzero = le_cmp == 8'd0;
	reg lnan_r, linf_r, lzero_r, le_ge135_r, le_lt115_r;
	reg load_go_step_r;
	wire [31:0] load_res_r;
	wire lodd = le[0];	/* e - 150 odd */
	wire lodd_lmm0, lodd_lmm1, lodd_hee;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lod0 (.I0(le[0]), .O(lodd_lmm0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lod1 (.I0(le[0]), .O(lodd_lmm1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lodh (.I0(le[0]), .O(lodd_hee));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lodd_lmm0 = le[0];
	assign lodd_lmm1 = le[0];
	assign lodd_hee = le[0];
`else
	assign lodd_lmm0 = le[0];
	assign lodd_lmm1 = le[0];
	assign lodd_hee  = le[0];
`endif

	wire [24:0] lmm = {
		lodd_lmm1 ? lm_lmm[23:11]          : {1'b0, lm_lmm[23:12]},
		lodd_lmm0 ? {lm_lmm[10:0], 1'b0}  : lm_lmm[11:0]
	};
	/* ee/2 with ee = e - 150 (- 1 when odd). */
	wire [11:0] hee_d, esh, tsh;
	wire [6:0] ersh;
	wire [7:0] trsh;
	reg signed [11:0] hee_r;
	reg [11:0] esh_r, tsh_r;
	reg [6:0] ersh_r;
	reg [7:0] trsh_r;
	reg [6:0] esh_amt_r;
	reg esh_left_r;
	reg [7:0] tsh_amt_r;
	reg tsh_left_r;
	am_sub #(.W(12)) u_hee (.a({4'd0, le}), .b(12'd150), .bi(lodd_hee), .d(hee_d), .bo(carry_unused[1]));
	wire signed [11:0] hee = $signed(hee_d) >>> 1;
	/* exp2: x in Q8.30 (sh = e - 120 <= 14 here). */
	am_sub #(.W(12)) u_esh (.a({4'd0, le}), .b(12'd120), .bi(1'b0), .d(esh), .bo(carry_unused[2]));
	am_sub #(.W(7)) u_ersh (.a(7'd120), .b(le[6:0]), .bi(1'b0), .d(ersh), .bo(carry_unused[3]));
	wire [38:0] ev0, ev;
	wire [38:0] ev0_r, ev_r;
	atg_su_xu_sh #(.W(39), .SW(7)) u_ev0 (.a({15'd0, lm_ev0}), .amt(esh_amt_r),
		.left(esh_left_r), .arith(1'b0), .y(ev0));
	/* 2-cycle registered 13b+13b+13b carry-select for u_ev so ls/ev0_r -> ev_*_r <= 10 cmos3. */
	wire ls_ev_1a, ls_ev_1b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsev1a (.I0(ls_ev_1_r), .O(ls_ev_1a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lsev1b (.I0(ls_ev_1_r), .O(ls_ev_1b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ls_ev_1a = ls_ev_1_r;
	assign ls_ev_1b = ls_ev_1_r;
`else
	assign ls_ev_1a = ls_ev_1_r;
	assign ls_ev_1b = ls_ev_1_r;
`endif
	wire [38:0] ev0_eff = {
		ls_ev_2b  ? ~ev0_r[38:32] : ev0_r[38:32],
		ls_ev_2a  ? ~ev0_r[31:26] : ev0_r[31:26],
		ls_ev_1b  ? ~ev0_r[25:20] : ev0_r[25:20],
		ls_ev_1a  ? ~ev0_r[19:13] : ev0_r[19:13],
		ls_ev_0_r ? ~ev0_r[12:0]  : ev0_r[12:0]
	};
	wire [12:0] ev_lo, ev_m0, ev_m1, ev_hi0, ev_hi1;
	wire        ev_c13, ev_c26_0, ev_c26_1, ev_co0_unused, ev_co1_unused;
	reg  [12:0] ev_lo_r, ev_m0_r, ev_m1_r, ev_hi0_r, ev_hi1_r;
	reg         ev_c13_r, ev_c26_0_r, ev_c26_1_r;
	am_add #(.W(13)) u_ev_lo  (.a(13'd0), .b(ev0_eff[12:0]),  .ci(ls_ev_0_r), .s(ev_lo),  .co(ev_c13));
	am_add #(.W(13)) u_ev_m0  (.a(13'd0), .b(ev0_eff[25:13]), .ci(1'b0),      .s(ev_m0),  .co(ev_c26_0));
	am_add #(.W(13)) u_ev_m1  (.a(13'd0), .b(ev0_eff[25:13]), .ci(1'b1),      .s(ev_m1),  .co(ev_c26_1));
	am_add #(.W(13)) u_ev_hi0 (.a(13'd0), .b(ev0_eff[38:26]), .ci(1'b0),      .s(ev_hi0), .co(ev_co0_unused));
	am_add #(.W(13)) u_ev_hi1 (.a(13'd0), .b(ev0_eff[38:26]), .ci(1'b1),      .s(ev_hi1), .co(ev_co1_unused));
	wire ev_c13_m, ev_c13_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_evc13m (.I0(ev_c13_r), .O(ev_c13_m));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_evc13c (.I0(ev_c13_r), .O(ev_c13_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ev_c13_m = ev_c13_r;
	assign ev_c13_c = ev_c13_r;
`else
	assign ev_c13_m = ev_c13_r;
	assign ev_c13_c = ev_c13_r;
`endif
	wire ev_c26_w = ev_c13_c ? ev_c26_1_r : ev_c26_0_r;
	assign ev = {ev_c26_w ? ev_hi1_r : ev_hi0_r, ev_c13_m ? ev_m1_r : ev_m0_r, ev_lo_r};
	assign carry_unused[6] = &{1'b0, ev_co0_unused, ev_co1_unused};
	/* sin/cos: the turns (split into 4 single-DSP 24x<=17 multiplies). */
	wire [40:0] tp0, tp1, tp2;
	wire [31:0] tp3;
	(* dont_touch = "true" *) reg [40:0] tp0_r, tp1_r, tp2_r;
	(* dont_touch = "true" *) reg [31:0] tp3_r;
	reg [40:0] tp2_rr;
	wire [82:0] tp_s0, tp, tps;
	reg [82:0] tp_s0_r;
	(* dont_touch = "true" *) reg [82:0] tp_r;
	wire [39:0] tps_r;
	am_mul #(.AW(24), .BW(17), .LAT(3)) u_tp0 (.clk(clk), .en(1'b1), .a(lm_r), .b(XU_INV2PI[16:0]), .p(tp0));
	am_mul #(.AW(24), .BW(17), .LAT(3)) u_tp1 (.clk(clk), .en(1'b1), .a(lm_r), .b(XU_INV2PI[33:17]), .p(tp1));
	am_mul #(.AW(24), .BW(17), .LAT(3)) u_tp2 (.clk(clk), .en(1'b1), .a(lm_r), .b(XU_INV2PI[50:34]), .p(tp2));
	am_mul #(.AW(24), .BW(8),  .LAT(3)) u_tp3 (.clk(clk), .en(1'b1), .a(lm_r), .b(XU_INV2PI[58:51]), .p(tp3));
	am_add #(.W(83)) u_tp_s0 (.a({tp3_r, 10'd0, tp0_r}), .b({25'd0, tp1_r, 17'd0}), .ci(1'b0),
		.s(tp_s0), .co(carry_unused[22]));
	am_add #(.W(83)) u_tp_s1 (.a(tp_s0_r), .b({8'd0, tp2_rr, 34'd0}), .ci(1'b0),
		.s(tp), .co(carry_unused[23]));
	am_sub #(.W(12)) u_tsh (.a({4'd0, le}), .b(12'd171), .bi(1'b0), .d(tsh), .bo(carry_unused[4]));
	am_sub #(.W(8)) u_trsh (.a(8'd171), .b(le), .bi(1'b0), .d(trsh), .bo(carry_unused[5]));
	/* The low 40 bits of tp[39:0] << tsh or tp >> -tsh: one shifter. */
	atg_su_xu_sh #(.W(83), .SW(8)) u_tps (.a(tp_r), .amt(tsh_amt_r), .left(tsh_left_r),
		.arith(1'b0), .y(tps));
	(* keep = "true", dont_touch = "true" *) reg t0_zero_a, t0_zero_b, t0_zero_c;
	wire t0_zero_a0, t0_zero_a1, t0_zero_b0, t0_zero_b1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t0za0 (.I0(t0_zero_a), .O(t0_zero_a0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t0za1 (.I0(t0_zero_a), .O(t0_zero_a1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t0zb0 (.I0(t0_zero_b), .O(t0_zero_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t0zb1 (.I0(t0_zero_b), .O(t0_zero_b1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign t0_zero_a0 = t0_zero_a;
	assign t0_zero_a1 = t0_zero_a;
	assign t0_zero_b0 = t0_zero_b;
	assign t0_zero_b1 = t0_zero_b;
`else
	assign t0_zero_a0 = t0_zero_a;
	assign t0_zero_a1 = t0_zero_a;
	assign t0_zero_b0 = t0_zero_b;
	assign t0_zero_b1 = t0_zero_b;
`endif

	wire [39:0] t0 = {
		t0_zero_c  ? 8'd0 : tps_r[39:32],
		t0_zero_b1 ? 8'd0 : tps_r[31:24],
		t0_zero_b0 ? 8'd0 : tps_r[23:16],
		t0_zero_a1 ? 8'd0 : tps_r[15:8],
		t0_zero_a0 ? 8'd0 : tps_r[7:0]
	};
	wire [39:0] t1, tt;
	wire [39:0] t1_r, tt_r;
	/* 2-cycle registered 8b+8b+8b+8b+8b carry-select for u_t1 so tps_r/ls -> t1_*_r <= 11 cmos3. */
	wire ls_t1_1b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lst11b (.I0(ls_t1_1_r), .O(ls_t1_1b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ls_t1_1b = ls_t1_1_r;
`else
	assign ls_t1_1b = ls_t1_1_r;
`endif
	wire [39:0] t0_eff = {
		ls_t1_2b  ? ~t0[39:32] : t0[39:32],
		ls_t1_2a  ? ~t0[31:24] : t0[31:24],
		ls_t1_1b  ? ~t0[23:16] : t0[23:16],
		ls_t1_1_r ? ~t0[15:8]  : t0[15:8],
		ls_t1_0_r ? ~t0[7:0]   : t0[7:0]
	};
	wire [7:0] t1_lo, t1_s8_0, t1_s8_1, t1_s16_0, t1_s16_1, t1_s24_0, t1_s24_1, t1_hi_0, t1_hi_1;
	wire       t1_c8, t1_c16_0, t1_c16_1, t1_c24_0, t1_c24_1, t1_c32_0, t1_c32_1, t1_co0_unused, t1_co1_unused;
	reg  [7:0] t1_lo_r, t1_s8_0_r, t1_s8_1_r, t1_s16_0_r, t1_s16_1_r, t1_s24_0_r, t1_s24_1_r, t1_hi_0_r, t1_hi_1_r;
	reg        t1_c8_r, t1_c16_0_r, t1_c16_1_r, t1_c24_0_r, t1_c24_1_r, t1_c32_0_r, t1_c32_1_r;
	am_add #(.W(8)) u_t1_lo  (.a(8'd0), .b(t0_eff[7:0]),   .ci(ls_t1_0_r), .s(t1_lo),    .co(t1_c8));
	am_add #(.W(8)) u_t1_s80 (.a(8'd0), .b(t0_eff[15:8]),  .ci(1'b0),      .s(t1_s8_0),  .co(t1_c16_0));
	am_add #(.W(8)) u_t1_s81 (.a(8'd0), .b(t0_eff[15:8]),  .ci(1'b1),      .s(t1_s8_1),  .co(t1_c16_1));
	am_add #(.W(8)) u_t1_s10 (.a(8'd0), .b(t0_eff[23:16]), .ci(1'b0),      .s(t1_s16_0), .co(t1_c24_0));
	am_add #(.W(8)) u_t1_s11 (.a(8'd0), .b(t0_eff[23:16]), .ci(1'b1),      .s(t1_s16_1), .co(t1_c24_1));
	am_add #(.W(8)) u_t1_s20 (.a(8'd0), .b(t0_eff[31:24]), .ci(1'b0),      .s(t1_s24_0), .co(t1_c32_0));
	am_add #(.W(8)) u_t1_s21 (.a(8'd0), .b(t0_eff[31:24]), .ci(1'b1),      .s(t1_s24_1), .co(t1_c32_1));
	am_add #(.W(8)) u_t1_hi0 (.a(8'd0), .b(t0_eff[39:32]), .ci(1'b0),      .s(t1_hi_0),  .co(t1_co0_unused));
	am_add #(.W(8)) u_t1_hi1 (.a(8'd0), .b(t0_eff[39:32]), .ci(1'b1),      .s(t1_hi_1),  .co(t1_co1_unused));
	wire t1_c8_m, t1_c8_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t1c8m (.I0(t1_c8_r), .O(t1_c8_m));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t1c8c (.I0(t1_c8_r), .O(t1_c8_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign t1_c8_m = t1_c8_r;
	assign t1_c8_c = t1_c8_r;
`else
	assign t1_c8_m = t1_c8_r;
	assign t1_c8_c = t1_c8_r;
`endif
	wire t1_c16_w = t1_c8_c  ? t1_c16_1_r : t1_c16_0_r;
	wire t1_c24_w = t1_c16_w ? t1_c24_1_r : t1_c24_0_r;
	wire t1_c32_w = t1_c24_w ? t1_c32_1_r : t1_c32_0_r;
	assign t1 = {
		t1_c32_w ? t1_hi_1_r  : t1_hi_0_r,
		t1_c24_w ? t1_s24_1_r : t1_s24_0_r,
		t1_c16_w ? t1_s16_1_r : t1_s16_0_r,
		t1_c8_m  ? t1_s8_1_r  : t1_s8_0_r,
		t1_lo_r
	};
	assign carry_unused[7] = &{1'b0, t1_co0_unused, t1_co1_unused};
	wire [1:0] tt_hi;
	am_add #(.W(2)) u_tt (.a(t1_r[39:38]), .b({1'b0, (rop_dt_tt == ATG_XU_COS[2:0])}), .ci(1'b0),
		.s(tt_hi), .co(carry_unused[8]));
	assign tt = {tt_hi, t1_r[37:0]};
	/* The shared multiplier (exp2: y * K, log2: y * y), split into 6 single-DSP <=24x<=17 multiplies. */
	wire [41:0] mb_r;
	wire [40:0] p00, p01;
	wire [31:0] p02;
	wire [34:0] p10, p11;
	wire [25:0] p12;
	(* dont_touch = "true" *) reg [40:0] p00_r, p01_r;
	(* dont_touch = "true" *) reg [31:0] p02_r;
	(* dont_touch = "true" *) reg [34:0] p10_r, p11_r;
	(* dont_touch = "true" *) reg [25:0] p12_r;
	wire [83:0] psum0, psum1, prod;
	reg [83:0] psum0_r, psum1_r;
	(* dont_touch = "true" *) reg [83:0] prod_r;
	am_mul #(.AW(24), .BW(17), .LAT(3)) u_p00 (.clk(clk), .en(1'b1), .a(y[23:0]),  .b(mb_r[16:0]),  .p(p00));
	am_mul #(.AW(24), .BW(17), .LAT(3)) u_p01 (.clk(clk), .en(1'b1), .a(y[23:0]),  .b(mb_r[33:17]), .p(p01));
	am_mul #(.AW(24), .BW(8),  .LAT(3)) u_p02 (.clk(clk), .en(1'b1), .a(y[23:0]),  .b(mb_r[41:34]), .p(p02));
	am_mul #(.AW(18), .BW(17), .LAT(3)) u_p10 (.clk(clk), .en(1'b1), .a(y[41:24]), .b(mb_r[16:0]),  .p(p10));
	am_mul #(.AW(18), .BW(17), .LAT(3)) u_p11 (.clk(clk), .en(1'b1), .a(y[41:24]), .b(mb_r[33:17]), .p(p11));
	am_mul #(.AW(18), .BW(8),  .LAT(3)) u_p12 (.clk(clk), .en(1'b1), .a(y[41:24]), .b(mb_r[41:34]), .p(p12));
	am_add #(.W(84)) u_ps0 (.a({8'd0, p11_r, p00_r}), .b({p12_r, p01_r, 17'd0}), .ci(1'b0),
		.s(psum0), .co(carry_unused[24]));
	am_add #(.W(84)) u_ps1 (.a({25'd0, p10_r, 24'd0}), .b({18'd0, p02_r, 34'd0}), .ci(1'b0),
		.s(psum1), .co(carry_unused[25]));
	am_add #(.W(84)) u_prod (.a(psum0_r), .b(psum1_r), .ci(1'b0),
		.s(prod), .co(carry_unused[26]));
	/* The division step (rcp, rsq phase 1) and the sqrt step. */
	wire [34:0] dtr, strl;
	wire [33:0] srem = {rem[31:0], n[59:58]};
	reg is_rcp_lo_r, is_rcp_m_r, is_rcp_hi_r;
	wire is_rcp_lo0, is_rcp_lo1, is_rcp_m0, is_rcp_m1, is_rcp_hi0, is_rcp_hi1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ircl0 (.I0(is_rcp_lo_r), .O(is_rcp_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ircl1 (.I0(is_rcp_lo_r), .O(is_rcp_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ircm0 (.I0(is_rcp_m_r),  .O(is_rcp_m0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ircm1 (.I0(is_rcp_m_r),  .O(is_rcp_m1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_irch0 (.I0(is_rcp_hi_r), .O(is_rcp_hi0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_irch1 (.I0(is_rcp_hi_r), .O(is_rcp_hi1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign is_rcp_lo0 = is_rcp_lo_r;
	assign is_rcp_lo1 = is_rcp_lo_r;
	assign is_rcp_m0 = is_rcp_m_r;
	assign is_rcp_m1 = is_rcp_m_r;
	assign is_rcp_hi0 = is_rcp_hi_r;
	assign is_rcp_hi1 = is_rcp_hi_r;
`else
	assign is_rcp_lo0 = is_rcp_lo_r;
	assign is_rcp_lo1 = is_rcp_lo_r;
	assign is_rcp_m0  = is_rcp_m_r;
	assign is_rcp_m1  = is_rcp_m_r;
	assign is_rcp_hi0 = is_rcp_hi_r;
	assign is_rcp_hi1 = is_rcp_hi_r;
`endif

	wire [34:0] dtr_b = {
		is_rcp_hi0 ? 5'd0             : 5'd0,
		is_rcp_hi1 ? 6'd0             : root[29:24],
		is_rcp_m1  ? lm_dtr[23:18]    : root[23:18],
		is_rcp_m0  ? lm_dtr[17:13]    : root[17:13],
		is_rcp_lo1 ? lm_dtr[12:6]     : root[12:6],
		is_rcp_lo0 ? lm_dtr[5:0]      : root[5:0]
	};
	/* 2-cycle registered 13b+11b+11b carry-select for u_dtr so is_rcp/rop -> dtr_*_r <= 10 cmos3. */
	wire [12:0] dtr_lo;
	wire [10:0] dtr_m0, dtr_m1, dtr_hi0, dtr_hi1;
	wire        dtr_b13, dtr_b24_0, dtr_b24_1, dtr_bo0_unused, dtr_bo1_unused;
	reg  [12:0] dtr_lo_r;
	reg  [10:0] dtr_m0_r, dtr_m1_r, dtr_hi0_r, dtr_hi1_r;
	reg         dtr_b13_r, dtr_b24_0_r, dtr_b24_1_r;
	am_sub #(.W(13)) u_dtr_lo  (.a({rem[11:0], 1'b0}), .b(dtr_b[12:0]),  .bi(1'b0), .d(dtr_lo),  .bo(dtr_b13));
	am_sub #(.W(11)) u_dtr_m0  (.a(rem[22:12]),        .b(dtr_b[23:13]), .bi(1'b0), .d(dtr_m0),  .bo(dtr_b24_0));
	am_sub #(.W(11)) u_dtr_m1  (.a(rem[22:12]),        .b(dtr_b[23:13]), .bi(1'b1), .d(dtr_m1),  .bo(dtr_b24_1));
	am_sub #(.W(11)) u_dtr_hi0 (.a(rem[33:23]),        .b(dtr_b[34:24]), .bi(1'b0), .d(dtr_hi0), .bo(dtr_bo0_unused));
	am_sub #(.W(11)) u_dtr_hi1 (.a(rem[33:23]),        .b(dtr_b[34:24]), .bi(1'b1), .d(dtr_hi1), .bo(dtr_bo1_unused));
	wire dtr_b13_m, dtr_b13_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dtrb13m (.I0(dtr_b13_r), .O(dtr_b13_m));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dtrb13c (.I0(dtr_b13_r), .O(dtr_b13_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign dtr_b13_m = dtr_b13_r;
	assign dtr_b13_c = dtr_b13_r;
`else
	assign dtr_b13_m = dtr_b13_r;
	assign dtr_b13_c = dtr_b13_r;
`endif
	wire dtr_b24_w = dtr_b13_c ? dtr_b24_1_r : dtr_b24_0_r;
	assign dtr = {dtr_b24_w ? dtr_hi1_r : dtr_hi0_r, dtr_b13_m ? dtr_m1_r : dtr_m0_r, dtr_lo_r};
	assign carry_unused[9] = &{1'b0, dtr_bo0_unused, dtr_bo1_unused};
	am_sub #(.W(35)) u_strl (.a({1'b0, srem}), .b({3'd0, root, 2'b01}), .bi(1'b0), .d(strl),
		.bo(carry_unused[10]));
	wire [33:0] snext = strl[34] ? srem : strl[33:0];
	/* The step counts, less one. */
	wire [5:0] nsteps_m1 = rop_pipe == ATG_XU_RCP[2:0] ? 6'd27 : rop_pipe == ATG_XU_FSQRT[2:0] ? 6'd25 :
		rop_pipe == ATG_XU_RSQ[2:0] ? (ph_step ? 6'd28 : 6'd29) :
		(rop_pipe == ATG_XU_SIN[2:0] || rop_pipe == ATG_XU_COS[2:0]) ? 6'd25 : 6'd29;
	wire [4:0] step_tab_atn_lo, step_tab_atn_hi;
	wire [4:0] step_tab_exp_lo, step_tab_exp_hi;
	genvar gsta;
	generate
		for (gsta = 0; gsta < 5; gsta = gsta + 1) begin : g_step_tab_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stalo (.I0(step_tab_atan[gsta]), .O(step_tab_atn_lo[gsta]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stahi (.I0(step_tab_atan[gsta]), .O(step_tab_atn_hi[gsta]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stelo (.I0(step_tab_expk[gsta]), .O(step_tab_exp_lo[gsta]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stehi (.I0(step_tab_expk[gsta]), .O(step_tab_exp_hi[gsta]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign step_tab_atn_lo[gsta] = step_tab_atan[gsta];
			assign step_tab_atn_hi[gsta] = step_tab_atan[gsta];
			assign step_tab_exp_lo[gsta] = step_tab_expk[gsta];
			assign step_tab_exp_hi[gsta] = step_tab_expk[gsta];
`else
			assign step_tab_atn_lo[gsta] = step_tab_atan[gsta];
			assign step_tab_atn_hi[gsta] = step_tab_atan[gsta];
			assign step_tab_exp_lo[gsta] = step_tab_expk[gsta];
			assign step_tab_exp_hi[gsta] = step_tab_expk[gsta];
`endif
		end
	endgenerate
	wire [39:0] atn_full_lo = xu_atan(step_tab_atn_lo);
	wire [39:0] atn_full_hi = xu_atan(step_tab_atn_hi);
	wire [39:0] atn = {atn_full_hi[39:20], atn_full_lo[19:0]};
	(* rom_style = "distributed", ram_style = "distributed" *) wire [39:0] atn_r;
	wire [10:0] lue;
	(* dont_touch = "true" *) reg [10:0] lue_r;
	am_sub #(.W(11)) u_lue (.a({3'd0, le}), .b(11'd127), .bi(1'b0), .d(lue), .bo(carry_unused[11]));
	wire [40:0] lv = {lue_r, fr};	/* log2: (e - 127) << 30 | fraction, signed */
	wire [40:0] lva;
	wire [40:0] lva_r;
	/* 2-cycle registered carry-select for u_lva so fr -> lva_lo_r/hi0_r/hi1_r <= 9 cmos3. */
	wire        lv_sub = lv[40];
	wire        lv_sub_lo0, lv_sub_lo1, lv_sub_hi0, lv_sub_hi1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvsl0 (.I0(lv_sub), .O(lv_sub_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvsl1 (.I0(lv_sub), .O(lv_sub_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvsh0 (.I0(lv_sub), .O(lv_sub_hi0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvsh1 (.I0(lv_sub), .O(lv_sub_hi1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lv_sub_lo0 = lv_sub;
	assign lv_sub_lo1 = lv_sub;
	assign lv_sub_hi0 = lv_sub;
	assign lv_sub_hi1 = lv_sub;
`else
	assign lv_sub_lo0 = lv_sub;
	assign lv_sub_lo1 = lv_sub;
	assign lv_sub_hi0 = lv_sub;
	assign lv_sub_hi1 = lv_sub;
`endif
	wire [40:0] lv_eff = {
		lv_sub_hi1 ? ~lv[40:31] : lv[40:31],
		lv_sub_hi0 ? ~lv[30:21] : lv[30:21],
		lv_sub_lo1 ? ~lv[20:11] : lv[20:11],
		lv_sub_lo0 ? ~lv[10:0]  : lv[10:0]
	};
	wire [20:0] lva_lo;
	wire [19:0] lva_hi0, lva_hi1;
	wire        lva_c21, lva_co0_unused, lva_co1_unused;
	reg  [20:0] lva_lo_r;
	reg  [19:0] lva_hi0_r, lva_hi1_r;
	reg         lva_c21_r;
	am_add #(.W(21)) u_lva_lo  (.a(21'd0), .b(lv_eff[20:0]),  .ci(lv_sub), .s(lva_lo),  .co(lva_c21));
	am_add #(.W(20)) u_lva_hi0 (.a(20'd0), .b(lv_eff[40:21]), .ci(1'b0),   .s(lva_hi0), .co(lva_co0_unused));
	am_add #(.W(20)) u_lva_hi1 (.a(20'd0), .b(lv_eff[40:21]), .ci(1'b1),   .s(lva_hi1), .co(lva_co1_unused));
	assign lva = {lva_c21_r ? lva_hi1_r : lva_hi0_r, lva_lo_r};
	assign carry_unused[12] = &{1'b0, lva_co0_unused, lva_co1_unused};
	/* The result of a finished lane (state FIN): an integer v (< 2^41) times 2^e, sign s, to
	 * fp32 (RNE, flushed). */
	reg ns, nst, nz;	/* nz: the result is 0 */
	reg [3:0] fr_q4_r;	/* stage-1 8:1 nibble pre-mux for fr[29 - step] */
	reg fbit_r;		/* stage-2 4:1 registered fr[29 - step] (exp2) */
	reg [40:0] nv;
	reg [11:0] ne;
	reg [5:0] np;
	(* dont_touch = "true" *) reg ns_r, nst_r, nz_r;
	wire [40:0] nv_r;
	(* dont_touch = "true" *) reg [11:0] ne_r;
	(* dont_touch = "true" *) reg [5:0] np_r;
	integer pi;
	reg [9:0] nv_lo0, nv_lo1;
	reg [10:0] nv_hi0;
	reg [9:0] nv_hi1;
	always @(*) begin
		case (rop_fin_lo0)
		ATG_XU_RCP[2:0]:   nv_lo0 = root[9:0];
		ATG_XU_FSQRT[2:0]: nv_lo0 = root[9:0];
		ATG_XU_RSQ[2:0]:   nv_lo0 = n[9:0];
		ATG_XU_EXP2[2:0]:  nv_lo0 = y[9:0];
		ATG_XU_LOG2[2:0]:  nv_lo0 = lva_r[9:0];
		default:           nv_lo0 = cy[9:0];
		endcase
	end
	always @(*) begin
		case (rop_fin_lo1)
		ATG_XU_RCP[2:0]:   nv_lo1 = root[19:10];
		ATG_XU_FSQRT[2:0]: nv_lo1 = root[19:10];
		ATG_XU_RSQ[2:0]:   nv_lo1 = n[19:10];
		ATG_XU_EXP2[2:0]:  nv_lo1 = y[19:10];
		ATG_XU_LOG2[2:0]:  nv_lo1 = lva_r[19:10];
		default:           nv_lo1 = cy[19:10];
		endcase
	end
	always @(*) begin
		case (rop_fin_hi0)
		ATG_XU_RCP[2:0]:   nv_hi0 = {3'd0, root[27:20]};
		ATG_XU_FSQRT[2:0]: nv_hi0 = {5'd0, root[25:20]};
		ATG_XU_RSQ[2:0]:   nv_hi0 = {2'd0, n[28:20]};
		ATG_XU_EXP2[2:0]:  nv_hi0 = y[30:20];
		ATG_XU_LOG2[2:0]:  nv_hi0 = lva_r[30:20];
		default:           nv_hi0 = cy[30:20];
		endcase
	end
	always @(*) begin
		case (rop_fin_hi1)
		ATG_XU_RCP[2:0]:   nv_hi1 = 10'd0;
		ATG_XU_FSQRT[2:0]: nv_hi1 = 10'd0;
		ATG_XU_RSQ[2:0]:   nv_hi1 = 10'd0;
		ATG_XU_EXP2[2:0]:  nv_hi1 = y[40:31];
		ATG_XU_LOG2[2:0]:  nv_hi1 = lva_r[40:31];
		default:           nv_hi1 = {5'd0, cy[35:31]};
		endcase
	end
	wire [40:0] nv_comb = {nv_hi1, nv_hi0, nv_lo1, nv_lo0};
	wire [5:0] np_from_nvr;
	wire [3:0] np_top_idx, np_mid_idx, np_bot_idx;
	wire       np_top_nz, np_mid_nz, np_bot_nz_unused;
	/* 3-way power-of-2 am_msb split replaces W=21+am_add(+20) with zero-gate bit concat. */
	am_msb #(.W(9),  .SW(4)) u_np_msb_top (.a(nv_r[40:32]), .n(np_top_idx), .nz(np_top_nz));
	am_msb #(.W(16), .SW(4)) u_np_msb_mid (.a(nv_r[31:16]), .n(np_mid_idx), .nz(np_mid_nz));
	am_msb #(.W(16), .SW(4)) u_np_msb_bot (.a(nv_r[15:0]),  .n(np_bot_idx), .nz(np_bot_nz_unused));
	assign np_from_nvr = np_top_nz ? {2'b10, np_top_idx} :
		(np_mid_nz ? {2'b01, np_mid_idx} : {2'b00, np_bot_idx});
	wire [5:0] np_comb = np_from_nvr;
	wire unused_np_msb = &{1'b0, np_bot_nz_unused};

	wire [31:0] fr_rev = {
		2'b00, fr[0], fr[1], fr[2], fr[3], fr[4], fr[5], fr[6],
		fr[7], fr[8], fr[9], fr[10], fr[11], fr[12], fr[13], fr[14],
		fr[15], fr[16], fr[17], fr[18], fr[19], fr[20], fr[21], fr[22],
		fr[23], fr[24], fr[25], fr[26], fr[27], fr[28], fr[29]
	};
	reg [3:0] fr_q4_comb;
	always @(*) begin
		case (step_fbit_lo[4:2])
		3'd0: fr_q4_comb = fr_rev[3:0];
		3'd1: fr_q4_comb = fr_rev[7:4];
		3'd2: fr_q4_comb = fr_rev[11:8];
		3'd3: fr_q4_comb = fr_rev[15:12];
		3'd4: fr_q4_comb = fr_rev[19:16];
		3'd5: fr_q4_comb = fr_rev[23:20];
		3'd6: fr_q4_comb = fr_rev[27:24];
		default: fr_q4_comb = fr_rev[31:28];
		endcase
	end

	reg fin_cy_z0_r, fin_cy_z1_r, fin_fr_z_r, fin_rem_nz0_r, fin_rem_nz1_r;
	reg fin_cy_le0_r, fin_lv_z_r, fin_rem_nz_r, fin_rem_or_stk_r;
	always @(*) begin
		nv = nv_comb;
		{ns, nst, nz, ne} = {3'b000, ex};
		case (rop_fin_ctl)
		ATG_XU_RCP[2:0]:   {ns, nst} = {ls_res, fin_rem_nz_r};
		ATG_XU_FSQRT[2:0]: nst = fin_rem_nz_r;
		ATG_XU_RSQ[2:0]:   nst = fin_rem_or_stk_r;
		ATG_XU_EXP2[2:0]:  ;
		ATG_XU_LOG2[2:0]:  {ns, ne, nz} = {lv[40], -12'sd30, fin_lv_z_r};
		default:           {ns, ne, nz} = {qneg, -12'sd34, fin_cy_le0_r};
		endcase
		np = np_comb;
	end
	wire [5:0] nsh;
	reg [5:0] nsh_r;
	wire [40:0] nw;
	wire [40:0] nw_r;
	wire [11:0] ne127, nexp;
	reg [11:0] ne127_r;
	(* dont_touch = "true" *) reg [11:0] nexp_r;
	am_sub #(.W(6)) u_nsh (.a(6'd40), .b(np_r), .bi(1'b0), .d(nsh), .bo(carry_unused[13]));
	atg_su_xu_sh #(.W(41), .SW(6), .DIR(2)) u_nw (.a(nv_r), .amt(nsh_r), .left(1'b1), .arith(1'b0), .y(nw));
	am_add #(.W(12)) u_ne (.a(ne_r), .b(12'd127), .ci(1'b0), .s(ne127), .co(carry_unused[14]));
	am_add #(.W(12)) u_nx (.a(ne127_r), .b({6'd0, np_r}), .ci(1'b0), .s(nexp), .co(carry_unused[15]));

	/* 2-cycle pipelined RNE rounding (Pre=0, Post=0 around CARRY4 in sub_st 5, pure LUTs in sub_st 6) */
	wire [23:0] nr_mp1_d;
	wire [11:0] nr_ep1_d;
	wire nr_ep1_co_unused;
	am_add #(.W(24)) u_nr_mp1 (.a(nw_r[40:17]), .b(24'd0), .ci(1'b1), .s(nr_mp1_d), .co(carry_unused[27]));
	am_add #(.W(12)) u_nr_ep1 (.a(nexp_r),      .b(12'd0), .ci(1'b1), .s(nr_ep1_d), .co(nr_ep1_co_unused));
	wire nr_nst_all_d = (|nw_r[15:0]) || nst_r;
	wire nr_up_d      = nw_r[16] && (nr_nst_all_d || nw_r[17]);
	wire nr_carry_d   = (&nw_r[40:17]) && nr_up_d;

	wire [23:0] nr_mp1_r;
	reg [11:0] nr_ep1_r;
	(* dont_touch = "true" *) reg nr_up_r, nr_carry_r;
	wire [31:0] nr_r;

	wire nr_carry_e, nr_carry_m;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nrce (.I0(nr_carry_r), .O(nr_carry_e));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nrcm (.I0(nr_carry_r), .O(nr_carry_m));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign nr_carry_e = nr_carry_r;
	assign nr_carry_m = nr_carry_r;
`else
	assign nr_carry_e = nr_carry_r;
	assign nr_carry_m = nr_carry_r;
`endif

	wire [11:0] nr_e_final = nr_carry_e ? nr_ep1_r : nexp_r;
	(* keep = "true" *) wire nr_udf_lo = nr_e_final[11] || !(|nr_e_final[10:0]);
	(* keep = "true" *) wire nr_udf_hi = nr_e_final[11] || !(|nr_e_final[10:0]);
	(* keep = "true" *) wire nr_ovf_lo = !nr_e_final[11] && ((|nr_e_final[10:8]) || (&nr_e_final[7:0]));
	(* keep = "true" *) wire nr_ovf_hi = !nr_e_final[11] && ((|nr_e_final[10:8]) || (&nr_e_final[7:0]));
	wire nr_up_a, nr_up_b, nr_udf_b, nr_ovf_b, ns_r_hi, ns_r_lo;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nrupa (.I0(nr_up_r),   .O(nr_up_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nrupb (.I0(nr_up_r),   .O(nr_up_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nudfb (.I0(nr_udf_lo), .O(nr_udf_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_novfb (.I0(nr_ovf_lo), .O(nr_ovf_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nsrhi (.I0(ns_r),      .O(ns_r_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nsrlo (.I0(ns_r),      .O(ns_r_lo));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign nr_up_a = nr_up_r;
	assign nr_up_b = nr_up_r;
	assign nr_udf_b = nr_udf_lo;
	assign nr_ovf_b = nr_ovf_lo;
	assign ns_r_hi = ns_r;
	assign ns_r_lo = ns_r;
`else
	assign nr_up_a  = nr_up_r;
	assign nr_up_b  = nr_up_r;
	assign nr_udf_b = nr_udf_lo;
	assign nr_ovf_b = nr_ovf_lo;
	assign ns_r_hi  = ns_r;
	assign ns_r_lo  = ns_r;
`endif
	wire [22:0] nr_m_final = {
		nr_carry_m ? 7'd0  : (nr_up_a ? nr_mp1_r[22:16] : nw_r[39:33]),
		nr_carry_m ? 16'd0 : (nr_up_b ? nr_mp1_r[15:0]  : nw_r[32:17])
	};
	wire [31:0] nr = {
		nr_ovf_hi ? {ns_r_hi, 8'hFF, 7'd0} : nr_udf_hi ? {ns_r_hi, 15'd0} : {ns_r_lo, nr_e_final[7:0], nr_m_final[22:16]},
		(nr_ovf_b || nr_udf_b) ? 16'd0 : nr_m_final[15:0]
	};
	wire nz_r_lo, nz_r_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nzrlo (.I0(nz_r), .O(nz_r_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nzrhi (.I0(nz_r), .O(nz_r_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign nz_r_lo = nz_r;
	assign nz_r_hi = nz_r;
`else
	assign nz_r_lo = nz_r;
	assign nz_r_hi = nz_r;
`endif

	wire [31:0] fin = {
		nz_r_hi ? 16'd0 : nr_r[31:16],
		nz_r_lo ? 16'd0 : nr_r[15:0]
	};

	/* LOAD: the result's exponent (rcp, rsq, fsqrt, exp2); z of sin/cos. */
	wire [23:0] exab = rop_dt_exab == ATG_XU_RCP[2:0] ? {12'd100, 4'd0, le} :	/* -50 - (e - 150) */
		rop_dt_exab == ATG_XU_RSQ[2:0] ? {-12'sd40, hee_r} : rop_dt_exab == ATG_XU_FSQRT[2:0] ? {hee_r, 12'd13} :
		{{3{ev_r[38]}}, ev_r[38:30], 12'd38};
	reg [23:0] exab_r;
	wire [11:0] ex_l;
	reg [11:0] ex_l_r;
	wire [38:0] zq;
	wire [38:0] zq_r;
	am_sub #(.W(12)) u_ex (.a(exab_r[23:12]), .b(exab_r[11:0]), .bi(1'b0), .d(ex_l), .bo(carry_unused[16]));
	am_sub #(.W(39)) u_zq (.a(39'h4000000000), .b({1'b0, t1_r[37:0]}), .bi(1'b0), .d(zq),
		.bo(carry_unused[17]));

	wire linf_hi, linf_hi_b, linf_lo;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_linf_hi  (.I0(linf_r), .O(linf_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_linf_hib (.I0(linf_r), .O(linf_hi_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_linf_lo  (.I0(linf_r), .O(linf_lo));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign linf_hi = linf_r;
	assign linf_hi_b = linf_r;
	assign linf_lo = linf_r;
`else
	assign linf_hi   = linf_r;
	assign linf_hi_b = linf_r;
	assign linf_lo   = linf_r;
`endif
	/* Precomputed LOAD special condition and result (registered in LOAD sub_st 8) */
	wire [2:0] rop_load_ctl_hi_a, rop_load_ctl_hi_b;
`ifdef SYNTH_KINTEX7
	genvar grlch;
	generate
		for (grlch = 0; grlch < 3; grlch = grlch + 1) begin : g_rlch_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rlcha (.I0(rop_load_ctl_hi[grlch]), .O(rop_load_ctl_hi_a[grlch]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rlchb (.I0(rop_load_ctl_hi[grlch]), .O(rop_load_ctl_hi_b[grlch]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar grlch;
	generate
		for (grlch = 0; grlch < 3; grlch = grlch + 1) begin : g_rlch_buf
			assign rop_load_ctl_hi_a[grlch] = rop_load_ctl_hi[grlch];
			assign rop_load_ctl_hi_b[grlch] = rop_load_ctl_hi[grlch];
		end
	endgenerate
`else
	assign rop_load_ctl_hi_a = rop_load_ctl_hi;
	assign rop_load_ctl_hi_b = rop_load_ctl_hi;
`endif
	reg load_go_step_w;
	reg [15:0] load_res_w_lo;
	reg [7:0] load_res_w_hi_b0, load_res_w_hi_b1;
	always @(*) begin
		load_go_step_w = 1'b0;
		load_res_w_hi_b1 = 8'h7F;
		case (rop_load_ctl_hi_a)
		ATG_XU_RCP[2:0]:
			if (lzero_r || linf_hi)
				load_res_w_hi_b1 = {ls_res, linf_hi ? 7'd0 : 7'h7F};
			else if (!lnan_r)
				load_go_step_w = 1'b1;
		ATG_XU_RSQ[2:0], ATG_XU_FSQRT[2:0]:
			if (lzero_r)
				load_res_w_hi_b1 = rop_load_ctl_hi_a == ATG_XU_RSQ[2:0] ? {ls_res, 7'h7F} : lx[31:24];
			else if (lnan_r || ls_res)
				load_res_w_hi_b1 = 8'h7F;
			else if (linf_hi)
				load_res_w_hi_b1 = rop_load_ctl_hi_a == ATG_XU_RSQ[2:0] ? 8'd0 : lx[31:24];
			else
				load_go_step_w = 1'b1;
		ATG_XU_EXP2[2:0]:
			if (lzero_r)
				load_res_w_hi_b1 = 8'h3F;
			else if (linf_hi || (!lnan_r && le_ge135_r))
				load_res_w_hi_b1 = ls_res ? 8'd0 : 8'h7F;
			else if (!lnan_r)
				load_go_step_w = 1'b1;
		ATG_XU_LOG2[2:0]:
			if (lzero_r)
				load_res_w_hi_b1 = 8'hFF;
			else if (lnan_r || ls_res)
				load_res_w_hi_b1 = 8'h7F;
			else if (linf_hi)
				load_res_w_hi_b1 = lx[31:24];
			else
				load_go_step_w = 1'b1;
		default:
			if (lnan_r || linf_hi)
				load_res_w_hi_b1 = 8'h7F;
			else if (le_lt115_r)
				load_res_w_hi_b1 = rop_load_ctl_hi_a == ATG_XU_SIN[2:0] ? lx[31:24] : 8'h3F;
			else
				load_go_step_w = 1'b1;
		endcase
	end
	always @(*) begin
		load_res_w_hi_b0 = 8'hC0;
		case (rop_load_ctl_hi_b)
		ATG_XU_RCP[2:0]:
			if (lzero_r || linf_hi_b)
				load_res_w_hi_b0 = linf_hi_b ? 8'd0 : 8'h80;
		ATG_XU_RSQ[2:0], ATG_XU_FSQRT[2:0]:
			if (lzero_r)
				load_res_w_hi_b0 = rop_load_ctl_hi_b == ATG_XU_RSQ[2:0] ? 8'h80 : lx[23:16];
			else if (lnan_r || ls_res)
				load_res_w_hi_b0 = 8'hC0;
			else if (linf_hi_b)
				load_res_w_hi_b0 = rop_load_ctl_hi_b == ATG_XU_RSQ[2:0] ? 8'd0 : lx[23:16];
		ATG_XU_EXP2[2:0]:
			if (lzero_r)
				load_res_w_hi_b0 = 8'h80;
			else if (linf_hi_b || (!lnan_r && le_ge135_r))
				load_res_w_hi_b0 = ls_res ? 8'd0 : 8'h80;
		ATG_XU_LOG2[2:0]:
			if (lzero_r)
				load_res_w_hi_b0 = 8'h80;
			else if (lnan_r || ls_res)
				load_res_w_hi_b0 = 8'hC0;
			else if (linf_hi_b)
				load_res_w_hi_b0 = lx[23:16];
		default:
			if (lnan_r || linf_hi_b)
				load_res_w_hi_b0 = 8'hC0;
			else if (le_lt115_r)
				load_res_w_hi_b0 = rop_load_ctl_hi_b == ATG_XU_SIN[2:0] ? lx[23:16] : 8'h80;
		endcase
	end
	wire [15:0] load_res_w_hi = {load_res_w_hi_b1, load_res_w_hi_b0};
	always @(*) begin
		load_res_w_lo = 16'd0;
		case (rop_load_ctl_lo)
		ATG_XU_RCP[2:0]:
			if (lzero_r || linf_lo)
				load_res_w_lo = 16'd0;
		ATG_XU_RSQ[2:0], ATG_XU_FSQRT[2:0]:
			if (lzero_r)
				load_res_w_lo = rop_load_ctl_lo == ATG_XU_RSQ[2:0] ? 16'd0 : lx[15:0];
			else if (lnan_r || ls_res)
				load_res_w_lo = 16'd0;
			else if (linf_lo)
				load_res_w_lo = rop_load_ctl_lo == ATG_XU_RSQ[2:0] ? 16'd0 : lx[15:0];
		ATG_XU_EXP2[2:0]:
			load_res_w_lo = 16'd0;
		ATG_XU_LOG2[2:0]:
			load_res_w_lo = 16'd0;
		default:
			if (lnan_r || linf_lo)
				load_res_w_lo = 16'd0;
			else if (le_lt115_r)
				load_res_w_lo = rop_load_ctl_lo == ATG_XU_SIN[2:0] ? lx[15:0] : 16'd0;
		endcase
	end
	wire [31:0] load_res_w = {load_res_w_hi, load_res_w_lo};

	/* STEP: the step count and the CORDIC rotation. */
	wire [5:0] step1;
	am_add #(.W(6)) u_step (.a(step_cnt), .b(6'd0), .ci(1'b1), .s(step1), .co(carry_unused[18]));
	wire [35:0] cysh, cxsh, cx_n, cy_n;
	wire [35:0] cysh_r, cxsh_r, cx_n_r, cy_n_r;
	wire [40:0] z_n;
	wire [40:0] z_n_r;
	wire [34:0] dtr_r, strl_r;
	wire strl34_snext_0, strl34_snext_1, strl34_snext_2, strl34_snext_stk, strl34_root;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_s34sn0 (.I0(strl_r[34]), .O(strl34_snext_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_s34sn1 (.I0(strl_r[34]), .O(strl34_snext_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_s34sn2 (.I0(strl_r[34]), .O(strl34_snext_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_s34stk (.I0(strl_r[34]), .O(strl34_snext_stk));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_s34rt  (.I0(strl_r[34]), .O(strl34_root));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign strl34_snext_0 = strl_r[34];
	assign strl34_snext_1 = strl_r[34];
	assign strl34_snext_2 = strl_r[34];
	assign strl34_snext_stk = strl_r[34];
	assign strl34_root = strl_r[34];
`else
	assign strl34_snext_0   = strl_r[34];
	assign strl34_snext_1   = strl_r[34];
	assign strl34_snext_2   = strl_r[34];
	assign strl34_snext_stk = strl_r[34];
	assign strl34_root      = strl_r[34];
`endif

	wire [33:0] snext_r = {
		strl34_snext_2 ? srem[33:22] : strl_r[33:22],
		strl34_snext_1 ? srem[21:11] : strl_r[21:11],
		strl34_snext_0 ? srem[10:0]  : strl_r[10:0]
	};
	wire cx35_sh, cy35_sh;
	wire z40_cx0, z40_cx1, z40_cx2, z40_cy0, z40_cy1, z40_cy2, z40_z;
	wire z40_z_lo, z40_z_m0, z40_z_m1, z40_z_hi0, z40_z_hi1;
	(* keep = "true" *) reg z_40_lo_r, z_40_m0_r, z_40_m1_r, z_40_hi0_r, z_40_hi1_r;
	(* keep = "true" *) reg z_40_cx0_r, z_40_cx1_r, z_40_cx2_r, z_40_cy0_r, z_40_cy1_r, z_40_cy2_r;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cx35sh (.I0(cx[35]), .O(cx35_sh));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cy35sh (.I0(cy[35]), .O(cy35_sh));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cx35_sh = cx[35];
	assign cy35_sh = cy[35];
`else
	assign cx35_sh = cx[35];
	assign cy35_sh = cy[35];
`endif
	assign z40_cx0 = !z_40_cx0_r;
	assign z40_cx1 = !z_40_cx1_r;
	assign z40_cx2 = !z_40_cx2_r;
	assign z40_cy0 = z_40_cy0_r;
	assign z40_cy1 = z_40_cy1_r;
	assign z40_cy2 = z_40_cy2_r;
	assign z40_z  = !z[40];
	assign z40_z_lo  = !z_40_lo_r;
	assign z40_z_m0  = !z_40_m0_r;
	assign z40_z_m1  = !z_40_m1_r;
	assign z40_z_hi0 = !z_40_hi0_r;
	assign z40_z_hi1 = !z_40_hi1_r;

	atg_su_xu_sh #(.W(36), .SW(6), .DIR(1)) u_cysh (.a({cy35_sh, cy[34:0]}), .amt(step_cordic), .left(1'b0), .arith(1'b1), .y(cysh));
	atg_su_xu_sh #(.W(36), .SW(6), .DIR(1)) u_cxsh (.a({cx35_sh, cx[34:0]}), .amt(step_cordic), .left(1'b0), .arith(1'b1), .y(cxsh));
	/* 2-cycle registered 12b+12b+12b carry-select for u_cx and u_cy so z[40]/cx -> cxn/cyn <= 10 cmos3. */
	wire [11:0] cxn_lo, cxn_m0, cxn_m1, cxn_hi0, cxn_hi1;
	wire        cxn_c12, cxn_c24_0, cxn_c24_1, cxn_co0_unused, cxn_co1_unused;
	reg  [11:0] cxn_lo_r, cxn_m0_r, cxn_m1_r, cxn_hi0_r, cxn_hi1_r;
	reg         cxn_c12_r, cxn_c24_0_r, cxn_c24_1_r;
	am_add #(.W(12)) u_cx_lo  (.a(cx[11:0]),  .b(z40_cx0 ? ~cysh_r[11:0]  : cysh_r[11:0]),  .ci(z40_cx0), .s(cxn_lo),  .co(cxn_c12));
	am_add #(.W(12)) u_cx_m0  (.a(cx[23:12]), .b(z40_cx1 ? ~cysh_r[23:12] : cysh_r[23:12]), .ci(1'b0),    .s(cxn_m0),  .co(cxn_c24_0));
	am_add #(.W(12)) u_cx_m1  (.a(cx[23:12]), .b(z40_cx1 ? ~cysh_r[23:12] : cysh_r[23:12]), .ci(1'b1),    .s(cxn_m1),  .co(cxn_c24_1));
	am_add #(.W(12)) u_cx_hi0 (.a(cx[35:24]), .b(z40_cx2 ? ~cysh_r[35:24] : cysh_r[35:24]), .ci(1'b0),    .s(cxn_hi0), .co(cxn_co0_unused));
	am_add #(.W(12)) u_cx_hi1 (.a(cx[35:24]), .b(z40_cx2 ? ~cysh_r[35:24] : cysh_r[35:24]), .ci(1'b1),    .s(cxn_hi1), .co(cxn_co1_unused));
	wire cxn_c24_w = cxn_c12_r ? cxn_c24_1_r : cxn_c24_0_r;
	assign cx_n = {cxn_c24_w ? cxn_hi1_r : cxn_hi0_r, cxn_c12_r ? cxn_m1_r : cxn_m0_r, cxn_lo_r};
	assign carry_unused[19] = &{1'b0, cxn_co0_unused, cxn_co1_unused};

	wire [11:0] cyn_lo, cyn_m0, cyn_m1, cyn_hi0, cyn_hi1;
	wire        cyn_c12, cyn_c24_0, cyn_c24_1, cyn_co0_unused, cyn_co1_unused;
	reg  [11:0] cyn_lo_r, cyn_m0_r, cyn_m1_r, cyn_hi0_r, cyn_hi1_r;
	reg         cyn_c12_r, cyn_c24_0_r, cyn_c24_1_r;
	am_add #(.W(12)) u_cy_lo  (.a(cy[11:0]),  .b(z40_cy0 ? ~cxsh_r[11:0]  : cxsh_r[11:0]),  .ci(z40_cy0), .s(cyn_lo),  .co(cyn_c12));
	am_add #(.W(12)) u_cy_m0  (.a(cy[23:12]), .b(z40_cy1 ? ~cxsh_r[23:12] : cxsh_r[23:12]), .ci(1'b0),    .s(cyn_m0),  .co(cyn_c24_0));
	am_add #(.W(12)) u_cy_m1  (.a(cy[23:12]), .b(z40_cy1 ? ~cxsh_r[23:12] : cxsh_r[23:12]), .ci(1'b1),    .s(cyn_m1),  .co(cyn_c24_1));
	am_add #(.W(12)) u_cy_hi0 (.a(cy[35:24]), .b(z40_cy2 ? ~cxsh_r[35:24] : cxsh_r[35:24]), .ci(1'b0),    .s(cyn_hi0), .co(cyn_co0_unused));
	am_add #(.W(12)) u_cy_hi1 (.a(cy[35:24]), .b(z40_cy2 ? ~cxsh_r[35:24] : cxsh_r[35:24]), .ci(1'b1),    .s(cyn_hi1), .co(cyn_co1_unused));
	wire cyn_c24_w = cyn_c12_r ? cyn_c24_1_r : cyn_c24_0_r;
	assign cy_n = {cyn_c24_w ? cyn_hi1_r : cyn_hi0_r, cyn_c12_r ? cyn_m1_r : cyn_m0_r, cyn_lo_r};
	assign carry_unused[20] = &{1'b0, cyn_co0_unused, cyn_co1_unused};
	/* 2-cycle registered 14b+14b+13b carry-select for u_z so z -> zn_*_r <= 11 cmos3. */
	wire [13:0] atn_z_eff_lo  = z40_z_lo  ? ~atn_r[13:0]         : atn_r[13:0];
	wire [13:0] atn_z_eff_m0  = z40_z_m0  ? ~atn_r[27:14]        : atn_r[27:14];
	wire [13:0] atn_z_eff_m1  = z40_z_m1  ? ~atn_r[27:14]        : atn_r[27:14];
	wire [12:0] atn_z_eff_hi0 = z40_z_hi0 ? ~{1'b0, atn_r[39:28]} : {1'b0, atn_r[39:28]};
	wire [12:0] atn_z_eff_hi1 = z40_z_hi1 ? ~{1'b0, atn_r[39:28]} : {1'b0, atn_r[39:28]};
	wire [13:0] zn_lo, zn_m0, zn_m1;
	wire [12:0] zn_hi0, zn_hi1;
	wire        zn_c14, zn_c28_0, zn_c28_1, zn_co0_unused, zn_co1_unused;
	reg  [13:0] zn_lo_r, zn_m0_r, zn_m1_r;
	reg  [12:0] zn_hi0_r, zn_hi1_r;
	reg         zn_c14_r, zn_c28_0_r, zn_c28_1_r;
	am_add #(.W(14)) u_z_lo  (.a(z[13:0]),  .b(atn_z_eff_lo),  .ci(z40_z_lo), .s(zn_lo),  .co(zn_c14));
	am_add #(.W(14)) u_z_m0  (.a(z[27:14]), .b(atn_z_eff_m0),  .ci(1'b0),     .s(zn_m0),  .co(zn_c28_0));
	am_add #(.W(14)) u_z_m1  (.a(z[27:14]), .b(atn_z_eff_m1),  .ci(1'b1),     .s(zn_m1),  .co(zn_c28_1));
	am_add #(.W(13)) u_z_hi0 (.a(z[40:28]), .b(atn_z_eff_hi0), .ci(1'b0),     .s(zn_hi0), .co(zn_co0_unused));
	am_add #(.W(13)) u_z_hi1 (.a(z[40:28]), .b(atn_z_eff_hi1), .ci(1'b1),     .s(zn_hi1), .co(zn_co1_unused));
	wire zn_c14_m, zn_c14_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_znc14m (.I0(zn_c14_r), .O(zn_c14_m));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_znc14c (.I0(zn_c14_r), .O(zn_c14_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign zn_c14_m = zn_c14_r;
	assign zn_c14_c = zn_c14_r;
`else
	assign zn_c14_m = zn_c14_r;
	assign zn_c14_c = zn_c14_r;
`endif
	wire zn_c28_w = zn_c14_c ? zn_c28_1_r : zn_c28_0_r;
	assign z_n = {zn_c28_w ? zn_hi1_r : zn_hi0_r, zn_c14_m ? zn_m1_r : zn_m0_r, zn_lo_r};
	assign carry_unused[21] = &{1'b0, zn_co0_unused, zn_co1_unused};

	wire [3:0] sub_st1;
	wire sub_st_co_unused;
	am_add #(.W(4)) u_sub_st (.a(sub_st), .b(4'd0), .ci(1'b1), .s(sub_st1), .co(sub_st_co_unused));

	/* Replicated per-lane idle register to bound rx_we/rq_clr fanout <= 32 */
	wire idle_mine_ctl = (st == IDLE) && mine;
	(* keep = "true", dont_touch = "true" *) reg [NPHYS-1:0] st_idle_rep;

	wire g_tps_we  = (st_g == LOAD) && (sub_st_load_b == 4'd8);
	wire g_ev0_we  = (st_g == LOAD) && (sub_st_load_b == 4'd3);
	wire g_ev_we   = (st_g == LOAD) && (sub_st_load_b == 4'd5);
	wire g_t1_we   = (st_g == LOAD) && (sub_st_load_b == 4'd10);
	wire g_ttzq_we = (st_g == LOAD) && (sub_st_load_b == 4'd11);
	wire g_stp0_we = (st_g == STEP) && (sub_st_step == 4'd0);
	wire g_stp1_we = (st_g == STEP) && (sub_st_step == 4'd1);
	wire g_stp2_we = (st_g == STEP) && (sub_st_step == 4'd2);
	wire g_fin2_we = (st_g == FIN)  && (sub_st_fin_b == 4'd2);
	wire g_fin3_we = (st_g == FIN)  && (sub_st_fin_b == 4'd3);
	wire g_fin6_we = (st_g == FIN)  && (sub_st_fin_b == 4'd6);
	wire g_tt_we, g_zq_we, g_stp0_a, g_stp0_b, g_stp2_a, g_stp2_b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gttw  (.I0(g_ttzq_we), .O(g_tt_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gzqw  (.I0(g_ttzq_we), .O(g_zq_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gst0a (.I0(g_stp0_we), .O(g_stp0_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gst0b (.I0(g_stp0_we), .O(g_stp0_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gst2a (.I0(g_stp2_we), .O(g_stp2_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gst2b (.I0(g_stp2_we), .O(g_stp2_b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign g_tt_we = g_ttzq_we;
	assign g_zq_we = g_ttzq_we;
	assign g_stp0_a = g_stp0_we;
	assign g_stp0_b = g_stp0_we;
	assign g_stp2_a = g_stp2_we;
	assign g_stp2_b = g_stp2_we;
`else
	assign g_tt_we  = g_ttzq_we;
	assign g_zq_we  = g_ttzq_we;
	assign g_stp0_a = g_stp0_we;
	assign g_stp0_b = g_stp0_we;
	assign g_stp2_a = g_stp2_we;
	assign g_stp2_b = g_stp2_we;
`endif

	genvar gxl, gxh;
	generate
	for (gxl = 0; gxl < NPHYS; gxl = gxl + 1) begin : g_xu_lane_reg
		wire im_lane = st_idle_rep[gxl] && mine;
		wire rq_we_lane;
		wire [LW-1:0] li_rq_sl = (gxl < (NPHYS / 2)) ? li_rq_0 : li_rq_1;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rqwln (.I0((st_cs == STORE) && (li_rq_sl == gxl[LW-1:0])), .O(rq_we_lane));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign rq_we_lane = (st_cs == STORE) && (li_rq_sl == gxl[LW-1:0]);
`else
		assign rq_we_lane = (st == STORE) && (li == gxl[LW-1:0]);
`endif
		for (gxh = 0; gxh < 2; gxh = gxh + 1) begin : g_xu_half
			wire rx_we_lo, rx_we_hi, rq_clr_h, rq_we_h;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rxwel (.I0(im_lane), .O(rx_we_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rxweh (.I0(im_lane), .O(rx_we_hi));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rqclh (.I0(im_lane), .O(rq_clr_h));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rqweh (.I0(rq_we_lane), .O(rq_we_h));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign rx_we_lo = im_lane;
			assign rx_we_hi = im_lane;
			assign rq_clr_h = im_lane;
			assign rq_we_h = rq_we_lane;
`else
			assign rx_we_lo = im_lane;
			assign rx_we_hi = im_lane;
			assign rq_clr_h = im_lane;
			assign rq_we_h  = rq_we_lane;
`endif

			(* shreg_extract = "no" *) reg [15:0] rx_sl, rq_sl;
			always @(posedge clk) begin
				if (rx_we_lo)
					rx_sl[7:0] <= x_a[gxl*32 + gxh*16 +: 8];
				if (rx_we_hi)
					rx_sl[15:8] <= x_a[gxl*32 + gxh*16 + 8 +: 8];
				if (rq_clr_h)
					rq_sl <= 16'd0;
				else if (rq_we_h)
					rq_sl <= res[gxh*16 +: 16];
			end
			assign rx[gxl*32 + gxh*16 +: 16] = rx_sl;
			assign rq[gxl*32 + gxh*16 +: 16] = rq_sl;
		end
	end
	for (gxl = 0; gxl < 3; gxl = gxl + 1) begin : g_tps_sl
		localparam TPS_W = (gxl == 2) ? 8 : 16;
		wire tps_we_sl;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tpswe (.I0(g_tps_we), .O(tps_we_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign tps_we_sl = g_tps_we;
`else
		assign tps_we_sl = g_tps_we;
`endif

		reg [TPS_W-1:0] tps_sl;
		always @(posedge clk)
			if (tps_we_sl)
				tps_sl <= tps[gxl*16 +: TPS_W];
		assign tps_r[gxl*16 +: TPS_W] = tps_sl;
	end
	for (gxl = 0; gxl < 3; gxl = gxl + 1) begin : g_dtr_sl
		localparam DTR_W = (gxl == 2) ? 3 : 16;
		wire dtr_we_sl, strl_we_sl;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dtrwe  (.I0(g_stp1_we), .O(dtr_we_sl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_strlwe (.I0(g_stp0_b),  .O(strl_we_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign dtr_we_sl = g_stp1_we;
		assign strl_we_sl = g_stp0_b;
`else
		assign dtr_we_sl  = g_stp1_we;
		assign strl_we_sl = g_stp0_b;
`endif

		reg [DTR_W-1:0] dtr_sl, strl_sl;
		always @(posedge clk) begin
			if (dtr_we_sl)
				dtr_sl <= dtr[gxl*16 +: DTR_W];
			if (strl_we_sl)
				strl_sl <= strl[gxl*16 +: DTR_W];
		end
		assign dtr_r[gxl*16 +: DTR_W] = dtr_sl;
		assign strl_r[gxl*16 +: DTR_W] = strl_sl;
	end
	for (gxl = 0; gxl < 3; gxl = gxl + 1) begin : g_xu_wide_sl
		localparam W39 = (gxl == 2) ? 7 : 16;
		localparam W40 = (gxl == 2) ? 8 : 16;
		localparam W41 = (gxl == 2) ? 9 : 16;
		localparam W36 = (gxl == 2) ? 4 : 16;
		wire ev0_we_s, ev_we_s, t1_we_s, tt_we_s, zq_we_s;
		wire cysh_we_s, cxsh_we_s, atn_we_s, cxn_we_s, cyn_we_s, zn_we_s;
		wire lva_we_s, nv_we_s, nw_we_s;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ev0we  (.I0(g_ev0_we),  .O(ev0_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_evwe   (.I0(g_ev_we),   .O(ev_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_t1we   (.I0(g_t1_we),   .O(t1_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ttwe   (.I0(g_tt_we),   .O(tt_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_zqwe   (.I0(g_zq_we),   .O(zq_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cyshwe (.I0(g_stp0_a),  .O(cysh_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cxshwe (.I0(g_stp0_a),  .O(cxsh_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atnwe  (.I0(g_stp0_b),  .O(atn_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cxnwe  (.I0(g_stp2_a),  .O(cxn_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cynwe  (.I0(g_stp2_a),  .O(cyn_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_znwe   (.I0(g_stp2_b),  .O(zn_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvawe  (.I0(g_fin2_we), .O(lva_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nvwe   (.I0(g_fin3_we), .O(nv_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwwe   (.I0(g_fin6_we), .O(nw_we_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign ev0_we_s = g_ev0_we;
		assign ev_we_s = g_ev_we;
		assign t1_we_s = g_t1_we;
		assign tt_we_s = g_tt_we;
		assign zq_we_s = g_zq_we;
		assign cysh_we_s = g_stp0_a;
		assign cxsh_we_s = g_stp0_a;
		assign atn_we_s = g_stp0_b;
		assign cxn_we_s = g_stp2_a;
		assign cyn_we_s = g_stp2_a;
		assign zn_we_s = g_stp2_b;
		assign lva_we_s = g_fin2_we;
		assign nv_we_s = g_fin3_we;
		assign nw_we_s = g_fin6_we;
`else
		assign ev0_we_s  = g_ev0_we;
		assign ev_we_s   = g_ev_we;
		assign t1_we_s   = g_t1_we;
		assign tt_we_s   = g_tt_we;
		assign zq_we_s   = g_zq_we;
		assign cysh_we_s = g_stp0_a;
		assign cxsh_we_s = g_stp0_a;
		assign atn_we_s  = g_stp0_b;
		assign cxn_we_s  = g_stp2_a;
		assign cyn_we_s  = g_stp2_a;
		assign zn_we_s   = g_stp2_b;
		assign lva_we_s  = g_fin2_we;
		assign nv_we_s   = g_fin3_we;
		assign nw_we_s   = g_fin6_we;
`endif

		reg [W39-1:0] ev0_sl, ev_sl, zq_sl;
		reg [W40-1:0] t1_sl, tt_sl, atn_sl;
		reg [W36-1:0] cysh_sl, cxsh_sl, cxn_sl, cyn_sl;
		reg [W41-1:0] zn_sl, lva_sl, nv_sl, nw_sl;
		always @(posedge clk) begin
			if (ev0_we_s)  ev0_sl  <= ev0[gxl*16 +: W39];
			if (ev_we_s)   ev_sl   <= ev[gxl*16 +: W39];
			if (t1_we_s)   t1_sl   <= t1[gxl*16 +: W40];
			if (tt_we_s)   tt_sl   <= tt[gxl*16 +: W40];
			if (zq_we_s)   zq_sl   <= zq[gxl*16 +: W39];
			if (cysh_we_s) cysh_sl <= cysh[gxl*16 +: W36];
			if (cxsh_we_s) cxsh_sl <= cxsh[gxl*16 +: W36];
			if (atn_we_s)  atn_sl  <= atn[gxl*16 +: W40];
			if (cxn_we_s)  cxn_sl  <= cx_n[gxl*16 +: W36];
			if (cyn_we_s)  cyn_sl  <= cy_n[gxl*16 +: W36];
			if (zn_we_s)   zn_sl   <= z_n[gxl*16 +: W41];
			if (lva_we_s)  lva_sl  <= lva[gxl*16 +: W41];
			if (nv_we_s)   nv_sl   <= nv[gxl*16 +: W41];
			if (nw_we_s)   nw_sl   <= nw[gxl*16 +: W41];
		end
		assign ev0_r[gxl*16 +: W39]  = ev0_sl;
		assign ev_r[gxl*16 +: W39]   = ev_sl;
		assign t1_r[gxl*16 +: W40]   = t1_sl;
		assign tt_r[gxl*16 +: W40]   = tt_sl;
		assign zq_r[gxl*16 +: W39]   = zq_sl;
		assign cysh_r[gxl*16 +: W36] = cysh_sl;
		assign cxsh_r[gxl*16 +: W36] = cxsh_sl;
		assign atn_r[gxl*16 +: W40]  = atn_sl;
		assign cx_n_r[gxl*16 +: W36] = cxn_sl;
		assign cy_n_r[gxl*16 +: W36] = cyn_sl;
		assign z_n_r[gxl*16 +: W41]  = zn_sl;
		assign lva_r[gxl*16 +: W41]  = lva_sl;
		assign nv_r[gxl*16 +: W41]   = nv_sl;
		assign nw_r[gxl*16 +: W41]   = nw_sl;
	end
	for (gxl = 0; gxl < 2; gxl = gxl + 1) begin : g_xu_nr_sl
		localparam W24 = (gxl == 1) ? 8 : 16;
		wire nrmp1_we_s, nrr_we_s;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nmpwe (.I0((st_g == FIN) && (sub_st_fin_b == 4'd7)), .O(nrmp1_we_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nrrwe (.I0((st_g == FIN) && (sub_st_fin_b == 4'd8)), .O(nrr_we_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign nrmp1_we_s = (st_g == FIN) && (sub_st_fin_b == 4'd7);
		assign nrr_we_s = (st_g == FIN) && (sub_st_fin_b == 4'd8);
`else
		assign nrmp1_we_s = (st_g == FIN) && (sub_st_fin_b == 4'd7);
		assign nrr_we_s   = (st_g == FIN) && (sub_st_fin_b == 4'd8);
`endif

		reg [W24-1:0] nrmp1_sl;
		reg [15:0] nrr_sl;
		always @(posedge clk) begin
			if (nrmp1_we_s) nrmp1_sl <= nr_mp1_d[gxl*16 +: W24];
			if (nrr_we_s)   nrr_sl   <= nr[gxl*16 +: 16];
		end
		assign nr_mp1_r[gxl*16 +: W24] = nrmp1_sl;
		assign nr_r[gxl*16 +: 16]      = nrr_sl;
	end
	endgenerate

	wire lx_we_lo, lx_we_hi, lm_we_lo, lm_we_hi;
	wire sh1_we_a, sh1_we_b, sh1_we_eamt, sh1_we_ersh, sh1_we_trsh1, sh1_we_trsh2, sh1_we_tsh_lo, sh1_we_tsh_hi;
	wire load_commit_raw = (st_g == LOAD) && (sub_st_load >= 4'd12);
	wire load_commit_a, load_commit_b;
	wire load_commit_res;
	wire cordic_load_commit, y_we_load, fr_we_load_lo, fr_we_load_hi;
	wire root_we_load_we, root_we_load_lo, root_we_load_hi, rem_we_load, n_we_load_lo, n_we_load_hi;
	wire load_end_lo, load_end_hi, res_we_0, res_we_1, res_we_2, res_we_3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldca     (.I0(load_commit_raw), .O(load_commit_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldcb     (.I0(load_commit_raw), .O(load_commit_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lxwelo   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd1)), .O(lx_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lxwehi   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd1)), .O(lx_we_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lmwelo   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd1)), .O(lm_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lmwehi   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd1)), .O(lm_we_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wea   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1web   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wee   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_eamt));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wer   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_ersh));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wt1   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_trsh1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wt2   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_trsh2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wtl   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_tsh_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sh1wth   (.I0((st_g == LOAD) && (sub_st_load_b == 4'd2)), .O(sh1_we_tsh_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldcmr    (.I0(load_commit_a), .O(load_commit_res));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cldcm    (.I0(load_commit_a), .O(cordic_load_commit));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_yweld    (.I0(load_commit_a), .O(y_we_load));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frwll    (.I0(load_commit_a), .O(fr_we_load_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frwlh    (.I0(load_commit_a), .O(fr_we_load_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtwlw    (.I0(load_commit_b), .O(root_we_load_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtwll    (.I0(load_commit_b), .O(root_we_load_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtwlh    (.I0(load_commit_b), .O(root_we_load_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwld    (.I0(load_commit_b), .O(rem_we_load));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwll     (.I0(load_commit_b), .O(n_we_load_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwlh     (.I0(load_commit_b), .O(n_we_load_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldelo    (.I0(load_commit_res), .O(load_end_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldehi    (.I0(load_commit_res), .O(load_end_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_reswe0   (.I0((load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres)), .O(res_we_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_reswe1   (.I0((load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres)), .O(res_we_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_reswe2   (.I0((load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres)), .O(res_we_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_reswe3   (.I0((load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres)), .O(res_we_3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign load_commit_a = load_commit_raw;
	assign load_commit_b = load_commit_raw;
	assign lx_we_lo = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lx_we_hi = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lm_we_lo = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lm_we_hi = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign sh1_we_a = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_b = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_eamt = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_ersh = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_trsh1 = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_trsh2 = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_tsh_lo = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_tsh_hi = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign load_commit_res = load_commit_a;
	assign cordic_load_commit = load_commit_a;
	assign y_we_load = load_commit_a;
	assign fr_we_load_lo = load_commit_a;
	assign fr_we_load_hi = load_commit_a;
	assign root_we_load_we = load_commit_b;
	assign root_we_load_lo = load_commit_b;
	assign root_we_load_hi = load_commit_b;
	assign rem_we_load = load_commit_b;
	assign n_we_load_lo = load_commit_b;
	assign n_we_load_hi = load_commit_b;
	assign load_end_lo = load_commit_res;
	assign load_end_hi = load_commit_res;
	assign res_we_0 = (load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_1 = (load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_2 = (load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_3 = (load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
`else
	assign load_commit_a       = load_commit_raw;
	assign load_commit_b       = load_commit_raw;
	assign lx_we_lo    = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lx_we_hi    = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lm_we_lo    = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign lm_we_hi    = (st_g == LOAD) && (sub_st_load_b == 4'd1);
	assign sh1_we_a    = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_b    = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_eamt = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_ersh = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_trsh1= (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_trsh2= (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_tsh_lo = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign sh1_we_tsh_hi = (st_g == LOAD) && (sub_st_load_b == 4'd2);
	assign load_commit_res     = load_commit_a;
	assign cordic_load_commit  = load_commit_a;
	assign y_we_load           = load_commit_a;
	assign fr_we_load_lo       = load_commit_a;
	assign fr_we_load_hi       = load_commit_a;
	assign root_we_load_we     = load_commit_b;
	assign root_we_load_lo     = load_commit_b;
	assign root_we_load_hi     = load_commit_b;
	assign rem_we_load         = load_commit_b;
	assign n_we_load_lo        = load_commit_b;
	assign n_we_load_hi        = load_commit_b;
	assign load_end_lo = load_commit_res;
	assign load_end_hi = load_commit_res;
	assign res_we_0    = (load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_1    = (load_end_lo && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_2    = (load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
	assign res_we_3    = (load_end_hi && !load_go_step_r) || (((st_g == FIN) && (sub_st_fin_b >= 4'd9)) && !sres);
`endif

	wire step_end_b0, step_end_b1, step_end_b2, step_end_b3;
	wire step_last_b0, step_last_b1, step_last_b2, step_load_clr, ce_nexp_r;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb0  (.I0((st_g == STEP) && (sub_st_step >= 4'd7)), .O(step_end_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb1  (.I0((st_g == STEP) && (sub_st_step >= 4'd7)), .O(step_end_b1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb2  (.I0((st_g == STEP) && (sub_st_step >= 4'd7)), .O(step_end_b2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb3  (.I0((st_g == STEP) && (sub_st_step >= 4'd7)), .O(step_end_b3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slb0  (.I0(step_is_last_r), .O(step_last_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slb1  (.I0(step_is_last_r), .O(step_last_b1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slb2  (.I0(step_is_last_r), .O(step_last_b2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slclr (.I0(load_commit_b), .O(step_load_clr));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cenxp (.I0((st_g == FIN) && (sub_st_fin_b == 4'd6)), .O(ce_nexp_r));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign step_end_b0 = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b1 = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b2 = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b3 = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_last_b0 = step_is_last_r;
	assign step_last_b1 = step_is_last_r;
	assign step_last_b2 = step_is_last_r;
	assign step_load_clr = load_commit_b;
	assign ce_nexp_r = (st_g == FIN) && (sub_st_fin_b == 4'd6);
`else
	assign step_end_b0   = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b1   = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b2   = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_end_b3   = (st_g == STEP) && (sub_st_step >= 4'd7);
	assign step_last_b0  = step_is_last_r;
	assign step_last_b1  = step_is_last_r;
	assign step_last_b2  = step_is_last_r;
	assign step_load_clr = load_commit_b;
	assign ce_nexp_r     = (st_g == FIN) && (sub_st_fin_b == 4'd6);
`endif

	reg step_is_last_r;
	wire step_end   = step_end_b0;
	wire step_last  = step_end_b0 && step_last_b0;
	wire unused_nsteps_m1 = &{1'b0, nsteps_m1};
	wire fin_end    = (st_g == FIN)  && (sub_st_fin >= 4'd8);
	wire load_end   = load_end_lo;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p)
			nexp_r <= 12'd0;
		else if (ce_nexp_r)
			nexp_r <= nexp;
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			step_tab_atan <= 6'd0;
		end else if (step_load_clr) begin
			step_tab_atan <= 6'd0;
		end else if (step_end_b1) begin
			step_tab_atan <= step_last_b1 ? 6'd0 : step1;
		end
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			step_tab_expk <= 6'd0;
			step_fbit_lo  <= 6'd0;
			step_fbit_hi  <= 6'd0;
		end else if (step_load_clr) begin
			step_tab_expk <= 6'd0;
			step_fbit_lo  <= 6'd0;
			step_fbit_hi  <= 6'd0;
		end else if (step_end_b2) begin
			step_tab_expk <= step_last_b2 ? 6'd0 : step1;
			step_fbit_lo  <= step_last_b2 ? 6'd0 : step1;
			step_fbit_hi  <= step_last_b2 ? 6'd0 : step1;
		end
	end

	always @(posedge clk) begin
		if (lx_we_lo) lx[15:0]   <= lx_raw[15:0];
		if (lx_we_hi) lx[31:16]  <= lx_raw[31:16];
		if (lm_we_lo) lm_r[15:0] <= lx_raw[15:0];
		if (lm_we_hi) lm_r[23:16] <= {1'b1, lx_raw[22:16]};
		if (sh1_we_a) begin
			hee_r  <= hee;
			lnan_r <= lnan;
			linf_r <= linf;
			lzero_r <= lzero;
			le_ge135_r <= (le_cmp >= 8'd135);
			le_lt115_r <= (le_cmp < 8'd115);
		end
		if (sh1_we_b) esh_r  <= esh;
		if (sh1_we_eamt) begin
			esh_amt_r  <= esh[11] ? ersh : {3'd0, esh[3:0]};
			esh_left_r <= !esh[11];
		end
		if (sh1_we_ersh) ersh_r <= ersh;
		if (sh1_we_trsh1) trsh_r[3:0] <= trsh[3:0];
		if (sh1_we_trsh2) trsh_r[7:4] <= trsh[7:4];
		if (sh1_we_tsh_lo) tsh_r[5:0]  <= tsh[5:0];
		if (sh1_we_tsh_hi) begin
			tsh_r[11:6] <= tsh[11:6];
			tsh_amt_r   <= tsh[11] ? trsh : {2'd0, tsh[5:0]};
			tsh_left_r  <= !tsh[11];
		end
		if (res_we_0) res[7:0]   <= load_end_lo ? load_res_r[7:0]   : fin[7:0];
		if (res_we_1) res[15:8]  <= load_end_lo ? load_res_r[15:8]  : fin[15:8];
		if (res_we_2) res[23:16] <= load_end_hi ? load_res_r[23:16] : fin[23:16];
		if (res_we_3) res[31:24] <= load_end_hi ? load_res_r[31:24] : fin[31:24];
	end

	/* Sliced/buffered datapath registers */
	wire [LW-1:0] first_x_b0, first_x_b1, first_x_b2, first_x_b3;
	wire [LW-1:0] after_x_b0, after_x_b1, after_x_b2, after_x_b3;
	genvar gxb;
	generate
		for (gxb = 0; gxb < LW; gxb = gxb + 1) begin : g_buf_x_sel
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxb0 (.I0(first[gxb]),     .O(first_x_b0[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxb1 (.I0(first[gxb]),     .O(first_x_b1[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxb2 (.I0(first[gxb]),     .O(first_x_b2[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fxb3 (.I0(first[gxb]),     .O(first_x_b3[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_axb0 (.I0(after_q_0[gxb]), .O(after_x_b0[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_axb1 (.I0(after_q_1[gxb]), .O(after_x_b1[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_axb2 (.I0(after_q_2[gxb]), .O(after_x_b2[gxb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_axb3 (.I0(after_q_3[gxb]), .O(after_x_b3[gxb]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign first_x_b0[gxb] = first[gxb];
			assign first_x_b1[gxb] = first[gxb];
			assign first_x_b2[gxb] = first[gxb];
			assign first_x_b3[gxb] = first[gxb];
			assign after_x_b0[gxb] = after_q_0[gxb];
			assign after_x_b1[gxb] = after_q_1[gxb];
			assign after_x_b2[gxb] = after_q_2[gxb];
			assign after_x_b3[gxb] = after_q_3[gxb];
`else
			assign first_x_b0[gxb] = first[gxb];
			assign first_x_b1[gxb] = first[gxb];
			assign first_x_b2[gxb] = first[gxb];
			assign first_x_b3[gxb] = first[gxb];
			assign after_x_b0[gxb] = after_q_0[gxb];
			assign after_x_b1[gxb] = after_q_1[gxb];
			assign after_x_b2[gxb] = after_q_2[gxb];
			assign after_x_b3[gxb] = after_q_3[gxb];
`endif
		end
	endgenerate

	wire cur_lx_load_raw = (st == LOAD) && (sub_st_load == 4'd0);
	wire [LW-1:0] li_lx_0, li_lx_1, li_lx_2, li_lx_3;
	genvar glx;
	generate
		for (glx = 0; glx < LW; glx = glx + 1) begin : g_buf_lilx
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lilx0 (.I0(li[glx]), .O(li_lx_0[glx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lilx1 (.I0(li[glx]), .O(li_lx_1[glx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lilx2 (.I0(li[glx]), .O(li_lx_2[glx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lilx3 (.I0(li[glx]), .O(li_lx_3[glx]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign li_lx_0[glx] = li[glx];
			assign li_lx_1[glx] = li[glx];
			assign li_lx_2[glx] = li[glx];
			assign li_lx_3[glx] = li[glx];
`else
			assign li_lx_0[glx] = li[glx];
			assign li_lx_1[glx] = li[glx];
			assign li_lx_2[glx] = li[glx];
			assign li_lx_3[glx] = li[glx];
`endif
		end
	endgenerate

	wire cur_lx_lo, cur_lx_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_clxlo (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd0)), .O(cur_lx_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_clxhi (.I0((st_g  == LOAD) && (sub_st_load   == 4'd0)), .O(cur_lx_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cur_lx_lo = (st_cs == LOAD) && (sub_st_load_b == 4'd0);
	assign cur_lx_hi = (st_g  == LOAD) && (sub_st_load   == 4'd0);
`else
	assign cur_lx_lo = (st_cs == LOAD) && (sub_st_load_b == 4'd0);
	assign cur_lx_hi = (st_g  == LOAD) && (sub_st_load   == 4'd0);
`endif


	reg [7:0] lx_cur_b0, lx_cur_b1, lx_cur_b2, lx_cur_b3;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			lx_cur_b0 <= 8'd0;
			lx_cur_b1 <= 8'd0;
			lx_cur_b2 <= 8'd0;
			lx_cur_b3 <= 8'd0;
		end else begin
			if (cur_lx_lo) begin
				lx_cur_b0 <= rx[{li_lx_0, 5'd0} +: 8];
				lx_cur_b1 <= rx[{li_lx_1, 5'd8} +: 8];
			end
			if (cur_lx_hi) begin
				lx_cur_b2 <= rx[{li_lx_2, 5'd16} +: 8];
				lx_cur_b3 <= rx[{li_lx_3, 5'd24} +: 8];
			end
		end
	end
	assign lx_cur_r = {lx_cur_b3, lx_cur_b2, lx_cur_b1, lx_cur_b0};

	wire load_res_we_lo, load_res_we_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldrwl (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd10)), .O(load_res_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ldrwh (.I0((st_g  == LOAD) && (sub_st_load   == 4'd10)), .O(load_res_we_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign load_res_we_lo = (st_cs == LOAD) && (sub_st_load_b == 4'd10);
	assign load_res_we_hi = (st_g  == LOAD) && (sub_st_load   == 4'd10);
`else
	assign load_res_we_lo = (st_cs == LOAD) && (sub_st_load_b == 4'd10);
	assign load_res_we_hi = (st_g  == LOAD) && (sub_st_load   == 4'd10);
`endif


	reg [15:0] load_res_lo, load_res_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			load_res_lo <= 16'd0;
			load_res_hi <= 16'd0;
		end else begin
			if (load_res_we_lo) load_res_lo <= load_res_w[15:0];
			if (load_res_we_hi) load_res_hi <= load_res_w[31:16];
		end
	end
	assign load_res_r = {load_res_hi, load_res_lo};

	wire prod_r_81_a0, prod_r_81_a1, prod_r_81_a2;
	wire prod_r_81_b, prod_r_81_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pr81a0 (.I0(prod_r[81]), .O(prod_r_81_a0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pr81a1 (.I0(prod_r[81]), .O(prod_r_81_a1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pr81a2 (.I0(prod_r[81]), .O(prod_r_81_a2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pr81b  (.I0(prod_r[81]), .O(prod_r_81_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pr81c  (.I0(prod_r[81]), .O(prod_r_81_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign prod_r_81_a0 = prod_r[81];
	assign prod_r_81_a1 = prod_r[81];
	assign prod_r_81_a2 = prod_r[81];
	assign prod_r_81_b = prod_r[81];
	assign prod_r_81_c = prod_r[81];
`else
	assign prod_r_81_a0 = prod_r[81];
	assign prod_r_81_a1 = prod_r[81];
	assign prod_r_81_a2 = prod_r[81];
	assign prod_r_81_b  = prod_r[81];
	assign prod_r_81_c  = prod_r[81];
`endif


	wire ph_rem, ph_root, ph_step;
	wire ph_n_s0, ph_n_s1, ph_n_s2, ph_n_s3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phrem  (.I0(ph), .O(ph_rem));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phrt   (.I0(ph), .O(ph_root));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phst   (.I0(ph), .O(ph_step));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phns0  (.I0(ph), .O(ph_n_s0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phns1  (.I0(ph), .O(ph_n_s1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phns2  (.I0(ph), .O(ph_n_s2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_phns3  (.I0(ph), .O(ph_n_s3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ph_rem = ph;
	assign ph_root = ph;
	assign ph_step = ph;
	assign ph_n_s0 = ph;
	assign ph_n_s1 = ph;
	assign ph_n_s2 = ph;
	assign ph_n_s3 = ph;
`else
	assign ph_rem   = ph;
	assign ph_root  = ph;
	assign ph_step  = ph;
	assign ph_n_s0  = ph;
	assign ph_n_s1  = ph;
	assign ph_n_s2  = ph;
	assign ph_n_s3  = ph;
`endif


	/* CORDIC cx, cy, z */
	wire cordic_we_cx_lo, cordic_we_cx_hi;
	wire cordic_we_cy_lo, cordic_we_cy_hi;
	wire cordic_we_z_lo, cordic_we_z_mid, cordic_we_z_hi;
	wire is_load_cx_lo, is_load_cx_hi;
	wire is_load_cy_lo, is_load_cy_hi;
	wire is_load_z_lo, is_load_z_mid, is_load_z_hi;
	wire cordic_step_commit = step_end_b0 && rop_we_is_cordic_r;
	wire cordic_we_raw = cordic_load_commit || cordic_step_commit;

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwexl (.I0(cordic_we_raw), .O(cordic_we_cx_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwexh (.I0(cordic_we_raw), .O(cordic_we_cx_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdweyl (.I0(cordic_we_raw), .O(cordic_we_cy_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdweyh (.I0(cordic_we_raw), .O(cordic_we_cy_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwezl (.I0(cordic_we_raw), .O(cordic_we_z_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwezm (.I0(cordic_we_raw), .O(cordic_we_z_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwezh (.I0(cordic_we_raw), .O(cordic_we_z_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildcxl (.I0(cordic_load_commit), .O(is_load_cx_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildcxh (.I0(cordic_load_commit), .O(is_load_cx_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildcyl (.I0(cordic_load_commit), .O(is_load_cy_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildcyh (.I0(cordic_load_commit), .O(is_load_cy_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildzl  (.I0(cordic_load_commit), .O(is_load_z_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildzm  (.I0(cordic_load_commit), .O(is_load_z_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ildzh  (.I0(cordic_load_commit), .O(is_load_z_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cordic_we_cx_lo = cordic_we_raw;
	assign cordic_we_cx_hi = cordic_we_raw;
	assign cordic_we_cy_lo = cordic_we_raw;
	assign cordic_we_cy_hi = cordic_we_raw;
	assign cordic_we_z_lo = cordic_we_raw;
	assign cordic_we_z_mid = cordic_we_raw;
	assign cordic_we_z_hi = cordic_we_raw;
	assign is_load_cx_lo = cordic_load_commit;
	assign is_load_cx_hi = cordic_load_commit;
	assign is_load_cy_lo = cordic_load_commit;
	assign is_load_cy_hi = cordic_load_commit;
	assign is_load_z_lo = cordic_load_commit;
	assign is_load_z_mid = cordic_load_commit;
	assign is_load_z_hi = cordic_load_commit;
`else
	assign cordic_we_cx_lo = cordic_we_raw;
	assign cordic_we_cx_hi = cordic_we_raw;
	assign cordic_we_cy_lo = cordic_we_raw;
	assign cordic_we_cy_hi = cordic_we_raw;
	assign cordic_we_z_lo  = cordic_we_raw;
	assign cordic_we_z_mid = cordic_we_raw;
	assign cordic_we_z_hi  = cordic_we_raw;
	assign is_load_cx_lo = cordic_load_commit;
	assign is_load_cx_hi = cordic_load_commit;
	assign is_load_cy_lo = cordic_load_commit;
	assign is_load_cy_hi = cordic_load_commit;
	assign is_load_z_lo  = cordic_load_commit;
	assign is_load_z_mid = cordic_load_commit;
	assign is_load_z_hi  = cordic_load_commit;
`endif

	wire tt38_lo, tt38_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tt38l (.I0(tt_r[38]), .O(tt38_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tt38h (.I0(tt_r[38]), .O(tt38_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign tt38_lo = tt_r[38];
	assign tt38_hi = tt_r[38];
`else
	assign tt38_lo = tt_r[38];
	assign tt38_hi = tt_r[38];
`endif

	wire [40:0] z_init = {
		tt38_hi ? {2'b0, zq_r[38:20]} : {3'b0, tt_r[37:20]},
		tt38_lo ? zq_r[19:0]          : tt_r[19:0]
	};
	reg [17:0] cx_lo, cx_hi, cy_lo, cy_hi;
	reg [13:0] z_lo, z_mid;
	reg [12:0] z_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			cx_lo <= 18'd0;
			cx_hi <= 18'd0;
			cy_lo <= 18'd0;
			cy_hi <= 18'd0;
			z_lo  <= 14'd0;
			z_mid <= 14'd0;
			z_hi  <= 13'd0;
			z_40_lo_r  <= 1'b0;
			z_40_m0_r  <= 1'b0;
			z_40_m1_r  <= 1'b0;
			z_40_hi0_r <= 1'b0;
			z_40_hi1_r <= 1'b0;
			z_40_cx0_r <= 1'b0;
			z_40_cx1_r <= 1'b0;
			z_40_cx2_r <= 1'b0;
			z_40_cy0_r <= 1'b0;
			z_40_cy1_r <= 1'b0;
			z_40_cy2_r <= 1'b0;
		end else begin
			if (cordic_we_cx_lo)
				cx_lo <= is_load_cx_lo ? XU_CG[17:0] : cx_n_r[17:0];
			if (cordic_we_cx_hi)
				cx_hi <= is_load_cx_hi ? {1'b0, XU_CG[34:18]} : cx_n_r[35:18];
			if (cordic_we_cy_lo)
				cy_lo <= is_load_cy_lo ? 18'd0 : cy_n_r[17:0];
			if (cordic_we_cy_hi)
				cy_hi <= is_load_cy_hi ? 18'd0 : cy_n_r[35:18];
			if (cordic_we_z_lo)
				z_lo  <= is_load_z_lo ? z_init[13:0] : z_n_r[13:0];
			if (cordic_we_z_mid)
				z_mid <= is_load_z_mid ? z_init[27:14] : z_n_r[27:14];
			if (cordic_we_z_hi) begin
				z_hi       <= is_load_z_hi ? z_init[40:28] : z_n_r[40:28];
				z_40_lo_r  <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_m0_r  <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_m1_r  <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_hi0_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_hi1_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cx0_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cx1_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cx2_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cy0_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cy1_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
				z_40_cy2_r <= is_load_z_hi ? z_init[40]    : z_n_r[40];
			end
		end
	end
	assign cx = {cx_hi, cx_lo};
	assign cy = {cy_hi, cy_lo};
	assign z  = {z_hi, z_mid, z_lo};

	/* Multiplier buffer mb_r */
	wire mb_we_lo, mb_we_mid, mb_we_hi;
	wire is_exp2_lo0, is_exp2_lo1;
	wire is_exp2_mid0, is_exp2_mid1;
	wire is_exp2_hi0, is_exp2_hi1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbwelo (.I0((st == STEP) && (sub_st_step == 4'd0)), .O(mb_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbwem  (.I0((st == STEP) && (sub_st_step == 4'd0)), .O(mb_we_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbwehi (.I0((st == STEP) && (sub_st_step == 4'd0)), .O(mb_we_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2l0  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2l1  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2m0  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_mid0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2m1  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_mid1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2h0  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_hi0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2h1  (.I0(rop_load_mb == ATG_XU_EXP2[2:0]), .O(is_exp2_hi1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign mb_we_lo = (st == STEP) && (sub_st_step == 4'd0);
	assign mb_we_mid = (st == STEP) && (sub_st_step == 4'd0);
	assign mb_we_hi = (st == STEP) && (sub_st_step == 4'd0);
	assign is_exp2_lo0 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_lo1 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_mid0 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_mid1 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_hi0 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_hi1 = rop_load_mb == ATG_XU_EXP2[2:0];
`else
	assign mb_we_lo = (st == STEP) && (sub_st_step == 4'd0);
	assign mb_we_mid = (st == STEP) && (sub_st_step == 4'd0);
	assign mb_we_hi = (st == STEP) && (sub_st_step == 4'd0);
	assign is_exp2_lo0  = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_lo1  = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_mid0 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_mid1 = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_hi0  = rop_load_mb == ATG_XU_EXP2[2:0];
	assign is_exp2_hi1  = rop_load_mb == ATG_XU_EXP2[2:0];
`endif


	wire [39:0] expk_full_lo = xu_expk(step_tab_exp_lo);
	wire [39:0] expk_full_hi = xu_expk(step_tab_exp_hi);
	wire [41:0] expk_val = {2'd0, expk_full_hi[39:20], expk_full_lo[19:0]};
	reg [13:0] mb_r_lo, mb_r_mid, mb_r_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			mb_r_lo  <= 14'd0;
			mb_r_mid <= 14'd0;
			mb_r_hi  <= 14'd0;
		end else begin
			if (mb_we_lo)  mb_r_lo  <= {is_exp2_lo1  ? expk_val[13:7]  : y[13:7],  is_exp2_lo0  ? expk_val[6:0]  : y[6:0]};
			if (mb_we_mid) mb_r_mid <= {is_exp2_mid1 ? expk_val[27:21] : y[27:21], is_exp2_mid0 ? expk_val[20:14] : y[20:14]};
			if (mb_we_hi)  mb_r_hi  <= {is_exp2_hi1  ? expk_val[41:35] : y[41:35], is_exp2_hi0  ? expk_val[34:28] : y[34:28]};
		end
	end
	assign mb_r = {mb_r_hi, mb_r_mid, mb_r_lo};

	/* Accumulator y */
	wire y_we_exp  = step_end_b0 && rop_we_is_exp2_r && fbit_r;
	wire y_we_log  = step_end_b0 && rop_we_is_log2_r;
	wire y_we_raw  = y_we_load || y_we_exp || y_we_log;

	wire y_we_lo, y_we_mid, y_we_hi;
	wire y_we_load_lo, y_we_load_mid, y_we_load_hi;
	wire y_we_exp_lo, y_we_exp_mid, y_we_exp_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywelo   (.I0(y_we_raw),  .O(y_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywemid  (.I0(y_we_raw),  .O(y_we_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywehi   (.I0(y_we_raw),  .O(y_we_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywldl   (.I0(y_we_load), .O(y_we_load_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywldm   (.I0(y_we_load), .O(y_we_load_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywldh   (.I0(y_we_load), .O(y_we_load_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywexl   (.I0(y_we_exp),  .O(y_we_exp_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywexm   (.I0(y_we_exp),  .O(y_we_exp_mid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ywexh   (.I0(y_we_exp),  .O(y_we_exp_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign y_we_lo = y_we_raw;
	assign y_we_mid = y_we_raw;
	assign y_we_hi = y_we_raw;
	assign y_we_load_lo = y_we_load;
	assign y_we_load_mid = y_we_load;
	assign y_we_load_hi = y_we_load;
	assign y_we_exp_lo = y_we_exp;
	assign y_we_exp_mid = y_we_exp;
	assign y_we_exp_hi = y_we_exp;
`else
	assign y_we_lo   = y_we_raw;
	assign y_we_mid  = y_we_raw;
	assign y_we_hi   = y_we_raw;
	assign y_we_load_lo  = y_we_load;
	assign y_we_load_mid = y_we_load;
	assign y_we_load_hi  = y_we_load;
	assign y_we_exp_lo   = y_we_exp;
	assign y_we_exp_mid  = y_we_exp;
	assign y_we_exp_hi   = y_we_exp;
`endif


	wire is_exp2_y_lo, is_exp2_y_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2yl (.I0(rop_load_y == ATG_XU_EXP2[2:0]), .O(is_exp2_y_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2yh (.I0(rop_load_y == ATG_XU_EXP2[2:0]), .O(is_exp2_y_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign is_exp2_y_lo = rop_load_y == ATG_XU_EXP2[2:0];
	assign is_exp2_y_hi = rop_load_y == ATG_XU_EXP2[2:0];
`else
	assign is_exp2_y_lo = rop_load_y == ATG_XU_EXP2[2:0];
	assign is_exp2_y_hi = rop_load_y == ATG_XU_EXP2[2:0];
`endif

	wire [41:0] y_load_val = {
		is_exp2_y_hi ? 21'h20000 : {1'b0, lm_y[23:4]},
		is_exp2_y_lo ? 21'd0     : {lm_y[3:0], 17'd0}
	};
	wire [13:0] y_next_lo = y_we_load_lo ? y_load_val[13:0] :
	                        y_we_exp_lo  ? prod_r[51:38] :
	                        (prod_r_81_a0 ? prod_r[54:41] : prod_r[53:40]);
	wire [13:0] y_next_mid = y_we_load_mid ? y_load_val[27:14] :
	                         y_we_exp_mid  ? prod_r[65:52] :
	                         (prod_r_81_a1 ? prod_r[68:55] : prod_r[67:54]);
	wire [13:0] y_next_hi = y_we_load_hi ? y_load_val[41:28] :
	                        y_we_exp_hi  ? {2'd0, prod_r[77:66]} :
	                        (prod_r_81_a2 ? {1'b0, prod_r[81:69]} : prod_r[81:68]);

	reg [13:0] y_lo, y_mid, y_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			y_lo  <= 14'd0;
			y_mid <= 14'd0;
			y_hi  <= 14'd0;
		end else begin
			if (y_we_lo)  y_lo  <= y_next_lo;
			if (y_we_mid) y_mid <= y_next_mid;
			if (y_we_hi)  y_hi  <= y_next_hi;
		end
	end
	assign y = {y_hi, y_mid, y_lo};

	/* Fraction register fr */
	wire fr_we_step = step_end_b3 && rop_we_is_log2_r;
	wire fr_we_raw = fr_we_load_lo || fr_we_step;

	wire fr_we_lo, fr_we_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frwel (.I0(fr_we_raw), .O(fr_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frweh (.I0(fr_we_raw), .O(fr_we_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign fr_we_lo = fr_we_raw;
	assign fr_we_hi = fr_we_raw;
`else
	assign fr_we_lo = fr_we_raw;
	assign fr_we_hi = fr_we_raw;
`endif


	wire is_exp2_fr_lo, is_exp2_fr_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2fl (.I0(rop_load_fr == ATG_XU_EXP2[2:0]), .O(is_exp2_fr_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ie2fh (.I0(rop_load_fr == ATG_XU_EXP2[2:0]), .O(is_exp2_fr_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign is_exp2_fr_lo = rop_load_fr == ATG_XU_EXP2[2:0];
	assign is_exp2_fr_hi = rop_load_fr == ATG_XU_EXP2[2:0];
`else
	assign is_exp2_fr_lo = rop_load_fr == ATG_XU_EXP2[2:0];
	assign is_exp2_fr_hi = rop_load_fr == ATG_XU_EXP2[2:0];
`endif

	wire [29:0] fr_load_val = {
		is_exp2_fr_hi ? ev_r[29:15] : 15'd0,
		is_exp2_fr_lo ? ev_r[14:0]  : 15'd0
	};
	wire [14:0] fr_next_lo = fr_we_load_lo ? fr_load_val[14:0]  : {fr_lo[13:0], prod_r_81_b};
	wire [14:0] fr_next_hi = fr_we_load_hi ? fr_load_val[29:15] : {fr_hi[13:0], fr_lo[14]};
	reg [14:0] fr_lo, fr_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			fr_lo <= 15'd0;
			fr_hi <= 15'd0;
		end else begin
			if (fr_we_lo) fr_lo <= fr_next_lo;
			if (fr_we_hi) fr_hi <= fr_next_hi;
		end
	end
	assign fr = {fr_hi, fr_lo};

	/* Sqrt/div root */
	wire root_we_step = step_end_b3 && (rop_we_is_rcp_r || (rop_we_is_rsq_fsq_r && !ph_root));
	wire root_we_raw = root_we_load_we || root_we_step;

	wire root_we_lo, root_we_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtwel (.I0(root_we_raw), .O(root_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtweh (.I0(root_we_raw), .O(root_we_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign root_we_lo = root_we_raw;
	assign root_we_hi = root_we_raw;
`else
	assign root_we_lo = root_we_raw;
	assign root_we_hi = root_we_raw;
`endif


	wire dtr34_rem_s0, dtr34_rem_s1, dtr34_rem_s2, dtr34_root;
	wire dtr34_n;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34s0 (.I0(dtr_r[34]), .O(dtr34_rem_s0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34s1 (.I0(dtr_r[34]), .O(dtr34_rem_s1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34s2 (.I0(dtr_r[34]), .O(dtr34_rem_s2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34rt (.I0(dtr_r[34]), .O(dtr34_root));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34n  (.I0(dtr_r[34]), .O(dtr34_n));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign dtr34_rem_s0 = dtr_r[34];
	assign dtr34_rem_s1 = dtr_r[34];
	assign dtr34_rem_s2 = dtr_r[34];
	assign dtr34_root = dtr_r[34];
	assign dtr34_n = dtr_r[34];
`else
	assign dtr34_rem_s0 = dtr_r[34];
	assign dtr34_rem_s1 = dtr_r[34];
	assign dtr34_rem_s2 = dtr_r[34];
	assign dtr34_root   = dtr_r[34];
	assign dtr34_n      = dtr_r[34];
`endif


	wire [14:0] root_next_lo = root_we_load_lo ? 15'd0 : {root[13:0], (rop_step == ATG_XU_RCP[2:0]) ? !dtr34_root : !strl34_root};
	wire [14:0] root_next_hi = root_we_load_hi ? 15'd0 : root[28:14];
	wire [29:0] root_next = {root_next_hi, root_next_lo};
	reg [14:0] root_lo, root_hi;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			root_lo <= 15'd0;
			root_hi <= 15'd0;
		end else begin
			if (root_we_lo) root_lo <= root_next[14:0];
			if (root_we_hi) root_hi <= root_next[29:15];
		end
	end
	assign root = {root_hi, root_lo};

	/* Remainder rem */
	wire rem_we_step = step_end_b3 && rop_we_is_div_r;
	wire rem_we_last = step_last && rop_is_rsq_r && !ph_rem;
	wire rem_we_last_lo, rem_we_last_hi, rem_we_last_raw;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwll  (.I0(rem_we_last), .O(rem_we_last_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwlh  (.I0(rem_we_last), .O(rem_we_last_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwlr  (.I0(rem_we_last), .O(rem_we_last_raw));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign rem_we_last_lo = rem_we_last;
	assign rem_we_last_hi = rem_we_last;
	assign rem_we_last_raw = rem_we_last;
`else
	assign rem_we_last_lo  = rem_we_last;
	assign rem_we_last_hi  = rem_we_last;
	assign rem_we_last_raw = rem_we_last;
`endif

	wire rem_we_raw = rem_we_load || rem_we_step || rem_we_last_raw;

	wire rem_we_s0, rem_we_s1, rem_we_s2;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmws0 (.I0(rem_we_raw), .O(rem_we_s0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmws1 (.I0(rem_we_raw), .O(rem_we_s1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmws2 (.I0(rem_we_raw), .O(rem_we_s2));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign rem_we_s0 = rem_we_raw;
	assign rem_we_s1 = rem_we_raw;
	assign rem_we_s2 = rem_we_raw;
`else
	assign rem_we_s0 = rem_we_raw;
	assign rem_we_s1 = rem_we_raw;
	assign rem_we_s2 = rem_we_raw;
`endif


	wire rem_step_cond = (rop_step == ATG_XU_RCP[2:0]) || ph_rem;
	wire dtr34_rem_lo, dtr34_rem_hi;
	wire rem_cond_lo, rem_cond_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34rl (.I0(dtr_r[34]),    .O(dtr34_rem_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d34rh (.I0(dtr_r[34]),    .O(dtr34_rem_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmcl  (.I0(rem_step_cond), .O(rem_cond_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmch  (.I0(rem_step_cond), .O(rem_cond_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign dtr34_rem_lo = dtr_r[34];
	assign dtr34_rem_hi = dtr_r[34];
	assign rem_cond_lo = rem_step_cond;
	assign rem_cond_hi = rem_step_cond;
`else
	assign dtr34_rem_lo = dtr_r[34];
	assign dtr34_rem_hi = dtr_r[34];
	assign rem_cond_lo  = rem_step_cond;
	assign rem_cond_hi  = rem_step_cond;
`endif

	wire [16:0] rem_step_lo = rem_cond_lo ? (!dtr34_rem_lo ? dtr_r[16:0] : {rem[15:0], 1'b0}) : snext_r[16:0];
	wire [16:0] rem_step_hi = rem_cond_hi ? (!dtr34_rem_hi ? dtr_r[33:17] : rem[32:16]) : snext_r[33:17];
	wire [33:0] rem_step_val = {rem_step_hi, rem_step_lo};
	wire rem_we_load_lo, rem_we_load_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwldl (.I0(rem_we_load), .O(rem_we_load_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rmwldh (.I0(rem_we_load), .O(rem_we_load_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign rem_we_load_lo = rem_we_load;
	assign rem_we_load_hi = rem_we_load;
`else
	assign rem_we_load_lo = rem_we_load;
	assign rem_we_load_hi = rem_we_load;
`endif

	wire [16:0] rem_next_lo = rem_we_load_lo ? ((rop_step == ATG_XU_RCP[2:0]) ? 17'h00000 : 17'd0) :
	                          rem_we_last_lo ? 17'h00000 : rem_step_lo;
	wire [16:0] rem_next_hi = rem_we_load_hi ? ((rop_step == ATG_XU_RCP[2:0]) ? 17'h00020 : 17'd0) :
	                          rem_we_last_hi ? 17'h00800 : rem_step_hi;
	wire [33:0] rem_next = {rem_next_hi, rem_next_lo};

	reg [11:0] rem_s0, rem_s1;
	reg [9:0]  rem_s2;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			rem_s0 <= 12'd0;
			rem_s1 <= 12'd0;
			rem_s2 <= 10'd0;
		end else begin
			if (rem_we_s0) rem_s0 <= rem_next[11:0];
			if (rem_we_s1) rem_s1 <= rem_next[23:12];
			if (rem_we_s2) rem_s2 <= rem_next[33:24];
		end
	end
	assign rem = {rem_s2, rem_s1, rem_s0};

	/* Radicand n */
	wire n_we_step = step_end_b3 && rop_we_is_rsq_fsq_r;
	wire n_we_last = step_last && rop_is_rsq_r && !ph_n_s0;
	wire n_we_last_we, n_we_last_lo0, n_we_last_lo1, n_we_last_hi0, n_we_last_hi1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwlw  (.I0(n_we_last), .O(n_we_last_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwll0 (.I0(n_we_last), .O(n_we_last_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwll1 (.I0(n_we_last), .O(n_we_last_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwlh0 (.I0(n_we_last), .O(n_we_last_hi0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwlh1 (.I0(n_we_last), .O(n_we_last_hi1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign n_we_last_we = n_we_last;
	assign n_we_last_lo0 = n_we_last;
	assign n_we_last_lo1 = n_we_last;
	assign n_we_last_hi0 = n_we_last;
	assign n_we_last_hi1 = n_we_last;
`else
	assign n_we_last_we  = n_we_last;
	assign n_we_last_lo0 = n_we_last;
	assign n_we_last_lo1 = n_we_last;
	assign n_we_last_hi0 = n_we_last;
	assign n_we_last_hi1 = n_we_last;
`endif

	wire n_we_raw_lo = n_we_load_lo || n_we_step || n_we_last_we;
	wire n_we_raw_hi = n_we_load_hi || n_we_step || n_we_last_we;

	wire n_we_s0, n_we_s1, n_we_s2, n_we_s3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nws0 (.I0(n_we_raw_lo), .O(n_we_s0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nws1 (.I0(n_we_raw_lo), .O(n_we_s1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nws2 (.I0(n_we_raw_hi), .O(n_we_s2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nws3 (.I0(n_we_raw_hi), .O(n_we_s3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign n_we_s0 = n_we_raw_lo;
	assign n_we_s1 = n_we_raw_lo;
	assign n_we_s2 = n_we_raw_hi;
	assign n_we_s3 = n_we_raw_hi;
`else
	assign n_we_s0 = n_we_raw_lo;
	assign n_we_s1 = n_we_raw_lo;
	assign n_we_s2 = n_we_raw_hi;
	assign n_we_s3 = n_we_raw_hi;
`endif


	wire [59:0] n_step_val = {
		ph_n_s3 ? n[59:45] : n[57:43],
		ph_n_s2 ? n[44:30] : n[42:28],
		ph_n_s1 ? {n[29], n[27:14]} : n[27:13],
		ph_n_s0 ? {n[13:0], !dtr34_n} : {n[12:0], 2'b00}
	};
	wire [59:0] n_load_val = {1'b0, lmm, 34'd0};
	wire n_we_load_lo0, n_we_load_lo1;
	wire n_we_load_hi0a, n_we_load_hi0b, n_we_load_hi1a, n_we_load_hi1b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld0  (.I0(n_we_load_lo), .O(n_we_load_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld1  (.I0(n_we_load_lo), .O(n_we_load_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld2a (.I0(n_we_load_hi), .O(n_we_load_hi0a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld2b (.I0(n_we_load_hi), .O(n_we_load_hi0b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld3a (.I0(n_we_load_hi), .O(n_we_load_hi1a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nwld3b (.I0(n_we_load_hi), .O(n_we_load_hi1b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign n_we_load_lo0 = n_we_load_lo;
	assign n_we_load_lo1 = n_we_load_lo;
	assign n_we_load_hi0a = n_we_load_hi;
	assign n_we_load_hi0b = n_we_load_hi;
	assign n_we_load_hi1a = n_we_load_hi;
	assign n_we_load_hi1b = n_we_load_hi;
`else
	assign n_we_load_lo0  = n_we_load_lo;
	assign n_we_load_lo1  = n_we_load_lo;
	assign n_we_load_hi0a = n_we_load_hi;
	assign n_we_load_hi0b = n_we_load_hi;
	assign n_we_load_hi1a = n_we_load_hi;
	assign n_we_load_hi1b = n_we_load_hi;
`endif

	wire [29:0] n_next_lo = {
		n_we_load_lo1 ? n_load_val[29:15] : n_we_last_lo1 ? 15'd0 : n_step_val[29:15],
		n_we_load_lo0 ? n_load_val[14:0]  : n_we_last_lo0 ? 15'd0 : n_step_val[14:0]
	};
	wire [29:0] n_next_hi = {
		n_we_load_hi1b ? n_load_val[59:52] : n_we_last_hi1 ? n[59:52] : n_step_val[59:52],
		n_we_load_hi1a ? n_load_val[51:45] : n_we_last_hi1 ? n[51:45] : n_step_val[51:45],
		n_we_load_hi0b ? n_load_val[44:37] : n_we_last_hi0 ? n[44:37] : n_step_val[44:37],
		n_we_load_hi0a ? n_load_val[36:30] : n_we_last_hi0 ? n[36:30] : n_step_val[36:30]
	};
	wire [59:0] n_next = {n_next_hi, n_next_lo};

	reg [14:0] n_s0, n_s1, n_s2, n_s3;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			n_s0 <= 15'd0;
			n_s1 <= 15'd0;
			n_s2 <= 15'd0;
			n_s3 <= 15'd0;
		end else begin
			if (n_we_s0) n_s0 <= n_next[14:0];
			if (n_we_s1) n_s1 <= n_next[29:15];
			if (n_we_s2) n_s2 <= n_next[44:30];
			if (n_we_s3) n_s3 <= n_next[59:45];
		end
	end
	assign n = {n_s3, n_s2, n_s1, n_s0};

	wire idle_mine_rmask;
	wire idle_mine_rop_0, idle_mine_rop_1, idle_mine_rop_2a, idle_mine_rop_2b;
	wire idle_mine_rrd;
	wire idle_mine_warp_grp;
	wire idle_mine_ctl_b = (st_cs == IDLE) && mine;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imrm  (.I0(idle_mine_ctl),   .O(idle_mine_rmask));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imr0  (.I0(idle_mine_ctl),   .O(idle_mine_rop_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imr1  (.I0(idle_mine_ctl),   .O(idle_mine_rop_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imr2a (.I0(idle_mine_ctl_b), .O(idle_mine_rop_2a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imr2b (.I0(idle_mine_ctl_b), .O(idle_mine_rop_2b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imrd  (.I0(idle_mine_ctl_b), .O(idle_mine_rrd));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_imwg  (.I0(idle_mine_ctl_b), .O(idle_mine_warp_grp));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign idle_mine_rmask = idle_mine_ctl;
	assign idle_mine_rop_0 = idle_mine_ctl;
	assign idle_mine_rop_1 = idle_mine_ctl;
	assign idle_mine_rop_2a = idle_mine_ctl_b;
	assign idle_mine_rop_2b = idle_mine_ctl_b;
	assign idle_mine_rrd = idle_mine_ctl_b;
	assign idle_mine_warp_grp = idle_mine_ctl_b;
`else
	assign idle_mine_rmask    = idle_mine_ctl;
	assign idle_mine_rop_0    = idle_mine_ctl;
	assign idle_mine_rop_1    = idle_mine_ctl;
	assign idle_mine_rop_2a   = idle_mine_ctl_b;
	assign idle_mine_rop_2b   = idle_mine_ctl_b;
	assign idle_mine_rrd      = idle_mine_ctl_b;
	assign idle_mine_warp_grp = idle_mine_ctl_b;
`endif
	wire [2:0] x_op_0, x_op_1;
`ifdef SYNTH_KINTEX7
	genvar gxop;
	generate
		for (gxop = 0; gxop < 3; gxop = gxop + 1) begin : g_xopb
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xop0 (.I0(x_uop[ATG_UOP_OP + gxop]), .O(x_op_0[gxop]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_xop1 (.I0(x_uop[ATG_UOP_OP + gxop]), .O(x_op_1[gxop]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gxop;
	generate
		for (gxop = 0; gxop < 3; gxop = gxop + 1) begin : g_xopb
			assign x_op_0[gxop] = x_uop[ATG_UOP_OP + gxop];
			assign x_op_1[gxop] = x_uop[ATG_UOP_OP + gxop];
		end
	endgenerate
`else
	assign x_op_0 = x_uop[ATG_UOP_OP +: 3];
	assign x_op_1 = x_uop[ATG_UOP_OP +: 3];
`endif


	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			rmask <= {NPHYS{1'b0}};
			rop <= 3'd0;
			rop_fin_lo0 <= 3'd0;
			rop_fin_lo1 <= 3'd0;
			rop_fin_hi0 <= 3'd0;
			rop_fin_hi1 <= 3'd0;
			rop_fin_ctl <= 3'd0;
			rop_pipe <= 3'd0;
			rop_dt <= 3'd0;
			rop_dt_dtr_lo <= 3'd0;
			rop_dt_dtr_hi <= 3'd0;
			rop_dt_exab <= 3'd0;
			rop_dt_tt <= 3'd0;
			rop_load <= 3'd0;
			rop_load_ctl_lo <= 3'd0;
			rop_load_ctl_hi <= 3'd0;
			rop_load_mb <= 3'd0;
			rop_load_y <= 3'd0;
			rop_load_fr <= 3'd0;
			rop_we <= 3'd0;
			rop_we_is_exp2_r    <= 1'b0;
			rop_we_is_log2_r    <= 1'b0;
			rop_we_is_cordic_r  <= 1'b0;
			rop_we_is_rcp_r     <= 1'b0;
			rop_we_is_rsq_fsq_r <= 1'b0;
			rop_we_is_div_r     <= 1'b0;
			rop_step <= 3'd0;
			nsteps_init_r <= 6'd29;
			rop_is_rsq_r <= 1'b0;
			is_rcp_lo_r <= 1'b0;
			is_rcp_m_r  <= 1'b0;
			is_rcp_hi_r <= 1'b0;
			rrd <= 7'd0;
			rwarp <= {WARP_W{1'b0}};
			rgrp <= {GRP_W{1'b0}};
			rlast <= 1'b0;
			rsbe <= {(SBE_W+1){1'b0}};
		end else begin
			if (idle_mine_rmask)
				rmask <= x_mask[NPHYS-1:0];
			if (idle_mine_rop_0) begin
				rop <= x_op_0;
				rop_fin_lo0 <= x_op_0;
				rop_fin_lo1 <= x_op_0;
				rop_fin_hi0 <= x_op_0;
				rop_fin_hi1 <= x_op_0;
				rop_fin_ctl <= x_op_0;
				is_rcp_lo_r <= (x_op_0 == ATG_XU_RCP[2:0]);
				is_rcp_m_r  <= (x_op_0 == ATG_XU_RCP[2:0]);
				is_rcp_hi_r <= (x_op_0 == ATG_XU_RCP[2:0]);
			end
			if (idle_mine_rop_1) begin
				rop_pipe <= x_op_0;
				rop_dt <= x_op_0;
				rop_dt_dtr_lo <= x_op_0;
				rop_dt_dtr_hi <= x_op_0;
				rop_dt_exab <= x_op_0;
				rop_dt_tt <= x_op_0;
			end
			if (idle_mine_rop_2a) begin
				rop_load <= x_op_1;
				rop_load_ctl_lo <= x_op_1;
				rop_load_ctl_hi <= x_op_1;
				rop_load_mb <= x_op_1;
				rop_load_y <= x_op_1;
				rop_load_fr <= x_op_1;
				rop_we <= x_op_1;
			end
			if (idle_mine_rop_2b) begin
				rop_we_is_exp2_r    <= (x_op_1 == ATG_XU_EXP2[2:0]);
				rop_we_is_log2_r    <= (x_op_1 == ATG_XU_LOG2[2:0]);
				rop_we_is_cordic_r  <= (x_op_1 == ATG_XU_SIN[2:0]) || (x_op_1 == ATG_XU_COS[2:0]);
				rop_we_is_rcp_r     <= (x_op_1 == ATG_XU_RCP[2:0]);
				rop_we_is_rsq_fsq_r <= (x_op_1 == ATG_XU_RSQ[2:0]) || (x_op_1 == ATG_XU_FSQRT[2:0]);
				rop_we_is_div_r     <= (x_op_1 == ATG_XU_RCP[2:0]) || (x_op_1 == ATG_XU_RSQ[2:0]) || (x_op_1 == ATG_XU_FSQRT[2:0]);
				rop_step <= x_op_1;
				rop_is_rsq_r <= (x_op_1 == ATG_XU_RSQ[2:0]);
				nsteps_init_r <= (x_op_1 == ATG_XU_RCP[2:0]) ? 6'd27 :
					(x_op_1 == ATG_XU_FSQRT[2:0] ||
					 x_op_1 == ATG_XU_SIN[2:0] ||
					 x_op_1 == ATG_XU_COS[2:0]) ? 6'd25 : 6'd29;
			end
			if (idle_mine_rrd) begin
				rrd <= x_uop[ATG_UOP_RD +: 7];
			end
			if (idle_mine_warp_grp) begin
				rwarp <= x_warp;
				rgrp <= x_grp;
				rlast <= x_last;
				rsbe <= x_sbe;
			end
		end
	end

	wire lue_we;
	wire ce_ev_ls, ce_ev_c0, ce_ev_c1, ce_ev_c2, ce_ev_c3, ce_ev_c4;
	wire ce_t1_ls, ce_t1_c0, ce_t1_c1, ce_t1_c2, ce_t1_c3, ce_t1_c4, ce_t1_c5, ce_t1_c6, ce_t1_c7, ce_t1_c8;
	wire ce_dtr_c0, ce_dtr_c1, ce_dtr_c2, ce_dtr_c3, ce_dtr_c4;
	wire ce_zn_c0, ce_zn_c1, ce_zn_c2, ce_zn_c3, ce_zn_c4;
	wire ce_cxn_c0, ce_cxn_c1, ce_cxn_c2, ce_cxn_c3, ce_cxn_c4;
	wire ce_cyn_c0, ce_cyn_c1, ce_cyn_c2, ce_cyn_c3, ce_cyn_c4;
	wire ce_lva_lo0, ce_lva_lo1, ce_lva_hi0, ce_lva_hi1, ce_lva_c21;
	wire st_step1 = (st_cs == STEP) && (sub_st_step == 4'd1);
	wire st_fin1  = (st_cs == FIN)  && (sub_st_fin == 4'd1);
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_luewe (.I0((st_cs == FIN) && (sub_st_fin == 4'd0)), .O(lue_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceevl (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd2)), .O(ce_ev_ls));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceev0 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd4)), .O(ce_ev_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceev1 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd4)), .O(ce_ev_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceev2 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd4)), .O(ce_ev_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceev3 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd4)), .O(ce_ev_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ceev4 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd4)), .O(ce_ev_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet1l (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd8)), .O(ce_t1_ls));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet10 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet11 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet12 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet13 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet14 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet15 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet16 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c6));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet17 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c7));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cet18 (.I0((st_cs == LOAD) && (sub_st_load_b == 4'd9)), .O(ce_t1_c8));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cedt0 (.I0((st_cs == STEP) && (sub_st_step == 4'd0)), .O(ce_dtr_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cedt1 (.I0((st_cs == STEP) && (sub_st_step == 4'd0)), .O(ce_dtr_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cedt2 (.I0((st_cs == STEP) && (sub_st_step == 4'd0)), .O(ce_dtr_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cedt3 (.I0((st_cs == STEP) && (sub_st_step == 4'd0)), .O(ce_dtr_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cedt4 (.I0((st_cs == STEP) && (sub_st_step == 4'd0)), .O(ce_dtr_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cezn0 (.I0(st_step1), .O(ce_zn_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cezn1 (.I0(st_step1), .O(ce_zn_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cezn2 (.I0(st_step1), .O(ce_zn_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cezn3 (.I0(st_step1), .O(ce_zn_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cezn4 (.I0(st_step1), .O(ce_zn_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecx0 (.I0(st_step1), .O(ce_cxn_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecx1 (.I0(st_step1), .O(ce_cxn_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecx2 (.I0(st_step1), .O(ce_cxn_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecx3 (.I0(st_step1), .O(ce_cxn_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecx4 (.I0(st_step1), .O(ce_cxn_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecy0 (.I0(st_step1), .O(ce_cyn_c0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecy1 (.I0(st_step1), .O(ce_cyn_c1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecy2 (.I0(st_step1), .O(ce_cyn_c2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecy3 (.I0(st_step1), .O(ce_cyn_c3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cecy4 (.I0(st_step1), .O(ce_cyn_c4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_celv0 (.I0(st_fin1),  .O(ce_lva_lo0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_celv1 (.I0(st_fin1),  .O(ce_lva_lo1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_celv2 (.I0(st_fin1),  .O(ce_lva_hi0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_celv3 (.I0(st_fin1),  .O(ce_lva_hi1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_celvc (.I0(st_fin1),  .O(ce_lva_c21));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lue_we = (st_cs == FIN) && (sub_st_fin == 4'd0);
	assign ce_ev_ls = (st_cs == LOAD) && (sub_st_load_b == 4'd2);
	assign ce_ev_c0 = (st_cs == LOAD) && (sub_st_load_b == 4'd4);
	assign ce_ev_c1 = (st_cs == LOAD) && (sub_st_load_b == 4'd4);
	assign ce_ev_c2 = (st_cs == LOAD) && (sub_st_load_b == 4'd4);
	assign ce_ev_c3 = (st_cs == LOAD) && (sub_st_load_b == 4'd4);
	assign ce_ev_c4 = (st_cs == LOAD) && (sub_st_load_b == 4'd4);
	assign ce_t1_ls = (st_cs == LOAD) && (sub_st_load_b == 4'd8);
	assign ce_t1_c0 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c1 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c2 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c3 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c4 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c5 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c6 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c7 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_t1_c8 = (st_cs == LOAD) && (sub_st_load_b == 4'd9);
	assign ce_dtr_c0 = (st_cs == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c1 = (st_cs == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c2 = (st_cs == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c3 = (st_cs == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c4 = (st_cs == STEP) && (sub_st_step == 4'd0);
	assign ce_zn_c0 = st_step1;
	assign ce_zn_c1 = st_step1;
	assign ce_zn_c2 = st_step1;
	assign ce_zn_c3 = st_step1;
	assign ce_zn_c4 = st_step1;
	assign ce_cxn_c0 = st_step1;
	assign ce_cxn_c1 = st_step1;
	assign ce_cxn_c2 = st_step1;
	assign ce_cxn_c3 = st_step1;
	assign ce_cxn_c4 = st_step1;
	assign ce_cyn_c0 = st_step1;
	assign ce_cyn_c1 = st_step1;
	assign ce_cyn_c2 = st_step1;
	assign ce_cyn_c3 = st_step1;
	assign ce_cyn_c4 = st_step1;
	assign ce_lva_lo0 = st_fin1;
	assign ce_lva_lo1 = st_fin1;
	assign ce_lva_hi0 = st_fin1;
	assign ce_lva_hi1 = st_fin1;
	assign ce_lva_c21 = st_fin1;
`else
	assign lue_we     = (st == FIN) && (sub_st_fin == 4'd0);
	assign ce_ev_ls   = (st == LOAD) && (sub_st_load == 4'd2);
	assign ce_ev_c0   = (st == LOAD) && (sub_st_load == 4'd4);
	assign ce_ev_c1   = (st == LOAD) && (sub_st_load == 4'd4);
	assign ce_ev_c2   = (st == LOAD) && (sub_st_load == 4'd4);
	assign ce_ev_c3   = (st == LOAD) && (sub_st_load == 4'd4);
	assign ce_ev_c4   = (st == LOAD) && (sub_st_load == 4'd4);
	assign ce_t1_ls   = (st == LOAD) && (sub_st_load == 4'd8);
	assign ce_t1_c0   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c1   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c2   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c3   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c4   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c5   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c6   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c7   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_t1_c8   = (st == LOAD) && (sub_st_load == 4'd9);
	assign ce_dtr_c0  = (st == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c1  = (st == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c2  = (st == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c3  = (st == STEP) && (sub_st_step == 4'd0);
	assign ce_dtr_c4  = (st == STEP) && (sub_st_step == 4'd0);
	assign ce_zn_c0   = st_step1;
	assign ce_zn_c1   = st_step1;
	assign ce_zn_c2   = st_step1;
	assign ce_zn_c3   = st_step1;
	assign ce_zn_c4   = st_step1;
	assign ce_cxn_c0  = st_step1;
	assign ce_cxn_c1  = st_step1;
	assign ce_cxn_c2  = st_step1;
	assign ce_cxn_c3  = st_step1;
	assign ce_cxn_c4  = st_step1;
	assign ce_cyn_c0  = st_step1;
	assign ce_cyn_c1  = st_step1;
	assign ce_cyn_c2  = st_step1;
	assign ce_cyn_c3  = st_step1;
	assign ce_cyn_c4  = st_step1;
	assign ce_lva_lo0 = st_fin1;
	assign ce_lva_lo1 = st_fin1;
	assign ce_lva_hi0 = st_fin1;
	assign ce_lva_hi1 = st_fin1;
	assign ce_lva_c21 = st_fin1;
`endif


	wire tsh_ge40 = !tsh_r[11] && ((|tsh_r[10:6]) || (tsh_r[5] && (tsh_r[4] || tsh_r[3])));
	always @(posedge clk) begin
		if (ce_ev_ls) begin
			ls_ev_0_r <= ls_ev;
			ls_ev_1_r <= ls_ev;
			ls_ev_2_r <= ls_ev;
		end
		if (ce_ev_c0) begin
			ev_lo_r  <= ev_lo;
			ev_c13_r <= ev_c13;
			ev_c26_0_r <= ev_c26_0;
			ev_c26_1_r <= ev_c26_1;
		end
		if (ce_ev_c1) ev_m0_r  <= ev_m0;
		if (ce_ev_c2) ev_m1_r  <= ev_m1;
		if (ce_ev_c3) ev_hi0_r <= ev_hi0;
		if (ce_ev_c4) ev_hi1_r <= ev_hi1;
		if (ce_t1_ls) begin
			ls_t1_0_r <= ls_t1;
			ls_t1_1_r <= ls_t1;
			ls_t1_2_r <= ls_t1;
		end
		if (ce_t1_c0) begin
			t1_lo_r    <= t1_lo;
			t1_c8_r    <= t1_c8;
			t1_c16_0_r <= t1_c16_0;
			t1_c16_1_r <= t1_c16_1;
			t1_c24_0_r <= t1_c24_0;
			t1_c24_1_r <= t1_c24_1;
			t1_c32_0_r <= t1_c32_0;
			t1_c32_1_r <= t1_c32_1;
		end
		if (ce_t1_c1) t1_s8_0_r  <= t1_s8_0;
		if (ce_t1_c2) t1_s8_1_r  <= t1_s8_1;
		if (ce_t1_c3) t1_s16_0_r <= t1_s16_0;
		if (ce_t1_c4) t1_s16_1_r <= t1_s16_1;
		if (ce_t1_c5) t1_s24_0_r <= t1_s24_0;
		if (ce_t1_c6) t1_s24_1_r <= t1_s24_1;
		if (ce_t1_c7) t1_hi_0_r  <= t1_hi_0;
		if (ce_t1_c8) t1_hi_1_r  <= t1_hi_1;
		if (ce_dtr_c0) begin
			dtr_lo_r    <= dtr_lo;
			dtr_b13_r   <= dtr_b13;
			dtr_b24_0_r <= dtr_b24_0;
			dtr_b24_1_r <= dtr_b24_1;
		end
		if (ce_dtr_c1) dtr_m0_r  <= dtr_m0;
		if (ce_dtr_c2) dtr_m1_r  <= dtr_m1;
		if (ce_dtr_c3) dtr_hi0_r <= dtr_hi0;
		if (ce_dtr_c4) dtr_hi1_r <= dtr_hi1;
		if (ce_zn_c0) begin
			zn_lo_r    <= zn_lo;
			zn_c14_r   <= zn_c14;
			zn_c28_0_r <= zn_c28_0;
			zn_c28_1_r <= zn_c28_1;
		end
		if (ce_zn_c1) zn_m0_r  <= zn_m0;
		if (ce_zn_c2) zn_m1_r  <= zn_m1;
		if (ce_zn_c3) zn_hi0_r <= zn_hi0;
		if (ce_zn_c4) zn_hi1_r <= zn_hi1;
		if (ce_cxn_c0) begin
			cxn_lo_r    <= cxn_lo;
			cxn_c12_r   <= cxn_c12;
			cxn_c24_0_r <= cxn_c24_0;
			cxn_c24_1_r <= cxn_c24_1;
		end
		if (ce_cxn_c1) cxn_m0_r  <= cxn_m0;
		if (ce_cxn_c2) cxn_m1_r  <= cxn_m1;
		if (ce_cxn_c3) cxn_hi0_r <= cxn_hi0;
		if (ce_cxn_c4) cxn_hi1_r <= cxn_hi1;
		if (ce_cyn_c0) begin
			cyn_lo_r    <= cyn_lo;
			cyn_c12_r   <= cyn_c12;
			cyn_c24_0_r <= cyn_c24_0;
			cyn_c24_1_r <= cyn_c24_1;
		end
		if (ce_cyn_c1) cyn_m0_r  <= cyn_m0;
		if (ce_cyn_c2) cyn_m1_r  <= cyn_m1;
		if (ce_cyn_c3) cyn_hi0_r <= cyn_hi0;
		if (ce_cyn_c4) cyn_hi1_r <= cyn_hi1;
		if (ce_lva_lo0) lva_lo_r[10:0]  <= lva_lo[10:0];
		if (ce_lva_lo1) lva_lo_r[20:11] <= lva_lo[20:11];
		if (ce_lva_c21) lva_c21_r       <= lva_c21;
		if (ce_lva_hi0) lva_hi0_r       <= lva_hi0;
		if (ce_lva_hi1) lva_hi1_r       <= lva_hi1;
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= IDLE;
			st_cs <= IDLE;
			st_g <= IDLE;
			st_d <= IDLE;
			nsteps_m1_r <= 6'd29;
			st_idle_rep <= {NPHYS{1'b1}};
			sub_st <= 4'd0;
			sub_st_load <= 4'd0;
			sub_st_load_b <= 4'd0;
			sub_st_step <= 4'd0;
			sub_st_fin <= 4'd0;
			sub_st_fin_b <= 4'd0;
			li <= {LW{1'b0}};
			li_rq_0 <= {LW{1'b0}};
			li_rq_1 <= {LW{1'b0}};
			after_q_0 <= {LW{1'b0}};
			after_q_1 <= {LW{1'b0}};
			after_q_2 <= {LW{1'b0}};
			after_q_3 <= {LW{1'b0}};
			after_nz_q <= 1'b0;
			t0_zero_a <= 1'b0;
			t0_zero_b <= 1'b0;
			t0_zero_c <= 1'b0;
			step <= 6'd0;
			step_cnt <= 6'd0;
			step_cnt_b <= 6'd0;
			step_cordic <= 6'd0;
			step_tab <= 6'd0;
			ph <= 1'b0;
			load_go_step_r <= 1'b0;
			fr_q4_r <= 4'd0;
			fbit_r <= 1'b0;
			stk <= 1'b0;
			sres <= 1'b0;
			ex <= 12'sd0;
			exab_r <= 24'd0;
			ex_l_r <= 12'sd0;
			step_is_last_r <= 1'b0;
			qneg <= 1'b0;
			tp0_r <= 41'd0;
			tp1_r <= 41'd0;
			tp2_r <= 41'd0;
			tp3_r <= 32'd0;
			tp2_rr <= 41'd0;
			tp_s0_r <= 83'd0;
			tp_r <= 83'd0;
			p00_r <= 41'd0;
			p01_r <= 41'd0;
			p02_r <= 32'd0;
			p10_r <= 35'd0;
			p11_r <= 35'd0;
			p12_r <= 26'd0;
			psum0_r <= 84'd0;
			psum1_r <= 84'd0;
			prod_r <= 84'd0;
			lue_r <= 11'd0;
			ns_r <= 1'b0;
			nst_r <= 1'b0;
			nz_r <= 1'b0;
			ne_r <= 12'd0;
			np_r <= 6'd0;
			nsh_r <= 6'd0;
			ne127_r <= 12'd0;
			fin_cy_z0_r <= 1'b0;
			fin_cy_z1_r <= 1'b0;
			fin_fr_z_r <= 1'b0;
			fin_rem_nz0_r <= 1'b0;
			fin_rem_nz1_r <= 1'b0;
			fin_cy_le0_r <= 1'b0;
			fin_lv_z_r <= 1'b0;
			fin_rem_nz_r <= 1'b0;
			fin_rem_or_stk_r <= 1'b0;
			nr_ep1_r <= 12'd0;
			nr_up_r <= 1'b0;
			nr_carry_r <= 1'b0;
		end else begin
			tp0_r <= tp0;
			tp1_r <= tp1;
			tp2_r <= tp2;
			tp3_r <= tp3;
			tp2_rr <= tp2_r;
			tp_s0_r <= tp_s0;
			tp_r <= tp;
			p00_r <= p00;
			p01_r <= p01;
			p02_r <= p02;
			p10_r <= p10;
			p11_r <= p11;
			p12_r <= p12;
			psum0_r <= psum0;
			psum1_r <= psum1;
			prod_r <= prod;
			case (st_d)
			IDLE: if (mine) begin
				st_idle_rep <= {NPHYS{1'b0}};
				sub_st <= 4'd0;
				sub_st_load <= 4'd0;
				sub_st_load_b <= 4'd0;
				sub_st_step <= 4'd0;
				sub_st_fin <= 4'd0;
				sub_st_fin_b <= 4'd0;
				if (first == NPHYS[LW:0]) begin
					st <= RET;
					st_cs <= RET;
					st_g <= RET;
					st_d <= RET;
				end else begin
					li <= first[LW-1:0];
					li_rq_0 <= first[LW-1:0];
					li_rq_1 <= first[LW-1:0];
					st <= LOAD;
					st_cs <= LOAD;
					st_g <= LOAD;
					st_d <= LOAD;
				end
			end
			LOAD: begin
				sub_st <= sub_st1;
				sub_st_load <= sub_st1;
				sub_st_load_b <= sub_st1;
				sub_st_step <= sub_st1;
				sub_st_fin <= sub_st1;
				sub_st_fin_b <= sub_st1;
				case (sub_st_load)
				4'd0, 4'd1, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7: ;
				4'd2: begin
					after_nz_q <= after_nz;
					after_q_0 <= ~after_rev_n;
					after_q_1 <= ~after_rev_n;
					after_q_2 <= ~after_rev_n;
					after_q_3 <= ~after_rev_n;
				end
				4'd8: begin
					exab_r <= exab;
					t0_zero_a <= tsh_ge40;
					t0_zero_b <= tsh_ge40;
					t0_zero_c <= tsh_ge40;
				end
				4'd9: begin
					ex_l_r <= ex_l;
				end
				4'd10: ;
				4'd11: begin
					load_go_step_r <= load_go_step_w;
				end
				default: begin
					sub_st <= 4'd0;
					sub_st_load <= 4'd0;
					sub_st_load_b <= 4'd0;
					sub_st_step <= 4'd0;
					sub_st_fin <= 4'd0;
					sub_st_fin_b <= 4'd0;
					step <= 6'd0;
					step_cnt <= 6'd0;
					step_cnt_b <= 6'd0;
					step_is_last_r <= 1'b0;
					step_cordic <= 6'd0;
					step_tab <= 6'd0;
					ph <= 1'b0;
					nsteps_m1_r <= nsteps_init_r;
					stk <= 1'b0;
					sres <= !load_go_step_r;
					st <= load_go_step_r ? STEP : FIN;
					st_cs <= load_go_step_r ? STEP : FIN;
					st_g <= load_go_step_r ? STEP : FIN;
					st_d <= load_go_step_r ? STEP : FIN;
					ex <= ex_l_r;
					qneg <= tt_r[39];
				end
				endcase
			end
			STEP: begin
				sub_st <= sub_st1;
				sub_st_load <= sub_st1;
				sub_st_load_b <= sub_st1;
				sub_st_step <= sub_st1;
				sub_st_fin <= sub_st1;
				sub_st_fin_b <= sub_st1;
				step_is_last_r <= (step_cnt_b == nsteps_m1_r);
				case (sub_st_step)
				4'd0, 4'd1, 4'd2, 4'd3, 4'd6: ;
				4'd4: fr_q4_r <= fr_q4_comb;
				4'd5: fbit_r <= fr_q4_r[step_fbit_lo[1:0]];
				default: begin
					sub_st <= 4'd0;
					sub_st_load <= 4'd0;
					sub_st_load_b <= 4'd0;
					sub_st_step <= 4'd0;
					sub_st_fin <= 4'd0;
					sub_st_fin_b <= 4'd0;
					step          <= step_last ? 6'd0 : step1;
					step_cnt      <= step_last ? 6'd0 : step1;
					step_cnt_b    <= step_last ? 6'd0 : step1;
					step_cordic   <= step_last ? 6'd0 : step1;
					step_tab      <= step_last ? 6'd0 : step1;
					if (step_last) begin
						if (rop_is_rsq_r && !ph_step) begin
							/* 2^57 / t: t = the root (30 bits). */
							ph <= 1'b1;
							nsteps_m1_r <= 6'd28;
							stk <= strl34_snext_stk ? (srem != 34'd0) : (strl_r[33:0] != 34'd0);
						end else begin
							st <= FIN;
							st_cs <= FIN;
							st_g <= FIN;
							st_d <= FIN;
						end
					end
				end
				endcase
			end
			FIN: begin
				sub_st <= sub_st1;
				sub_st_load <= sub_st1;
				sub_st_load_b <= sub_st1;
				sub_st_step <= sub_st1;
				sub_st_fin <= sub_st1;
				sub_st_fin_b <= sub_st1;
				case (sub_st_fin)
				4'd0: begin
					if (lue_we) lue_r <= lue;
					fin_cy_z0_r <= (cy[17:0] == 18'd0);
					fin_cy_z1_r <= (cy[34:18] == 17'd0);
					fin_fr_z_r <= (fr == 30'd0);
					fin_rem_nz0_r <= (rem[16:0] != 17'd0);
					fin_rem_nz1_r <= (rem[33:17] != 17'd0);
				end
				4'd1: begin
					fin_cy_le0_r <= cy[35] || (fin_cy_z0_r && fin_cy_z1_r);
					fin_lv_z_r <= fin_fr_z_r && (lue_r == 11'd0);
					fin_rem_nz_r <= fin_rem_nz0_r || fin_rem_nz1_r;
					fin_rem_or_stk_r <= fin_rem_nz0_r || fin_rem_nz1_r || stk;
				end
				4'd2: ;
				4'd3: begin
					ns_r <= ns;
					nst_r <= nst;
					nz_r <= nz;
					ne_r <= ne;
				end
				4'd4: np_r <= np;
				4'd5: begin
					nsh_r <= nsh;
					ne127_r <= ne127;
				end
				4'd6: ;
				4'd7: begin
					nr_ep1_r <= nr_ep1_d;
					nr_up_r <= nr_up_d;
					nr_carry_r <= nr_carry_d;
				end
				4'd8: ;
				default: begin
					sub_st <= 4'd0;
					sub_st_load <= 4'd0;
					sub_st_load_b <= 4'd0;
					sub_st_step <= 4'd0;
					sub_st_fin <= 4'd0;
					sub_st_fin_b <= 4'd0;
					st <= STORE;
					st_cs <= STORE;
					st_g <= STORE;
					st_d <= STORE;
				end
				endcase
			end
			STORE: begin
				/* The result of FIN is in res: store it, next lane. */
				sub_st <= 4'd0;
				sub_st_load <= 4'd0;
				sub_st_load_b <= 4'd0;
				sub_st_step <= 4'd0;
				sub_st_fin <= 4'd0;
				sub_st_fin_b <= 4'd0;
				if (!after_nz_q) begin
					st <= RET;
					st_cs <= RET;
					st_g <= RET;
					st_d <= RET;
				end else begin
					li <= after_q_0;
					li_rq_0 <= after_q_0;
					li_rq_1 <= after_q_0;
					st <= LOAD;
					st_cs <= LOAD;
					st_g <= LOAD;
					st_d <= LOAD;
				end
			end
			default: if (l_ready) begin
				st <= IDLE;
				st_cs <= IDLE;
				st_g <= IDLE;
				st_d <= IDLE;
				st_idle_rep <= {NPHYS{1'b1}};
			end
			endcase
		end
	end

	wire l_valid_buf;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lvbuf (.I0(st == RET), .O(l_valid_buf));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign l_valid_buf = st == RET;
`else
	assign l_valid_buf = st == RET;
`endif

	assign l_valid = l_valid_buf;
	assign l_warp = rwarp;
	assign l_grp = rgrp;
	assign l_rd = rrd;
	assign l_data = rq;
	assign l_lanes = rmask;
	assign l_wr = 1'b1;
	assign l_last = rlast;
	assign l_sbe = rsbe;
	assign l_cls = ATG_CLS_X;

	wire unused_ok = &{1'b0, x_pc, x_b, x_c, x_mask[2*NPHYS-1:NPHYS], x_info, x_uop,
		snext, prod_r, tp_r[82:40], tps[82:40], y[41], esh_r[10:4], carry_unused[27],
		sub_st_co_unused, step_end, nr_ep1_co_unused, nr_mp1_r[23], nw_r[40], nw_r[15:0],
		unused_np_msb, step_fbit_hi, step_fbit_lo[5], after, (NWARPS != 0)};
endmodule

module atg_su_xu_sh #(
	parameter W = 32,
	parameter SW = 5,
	parameter DIR = 0
) (
	input wire [W-1:0] a,
	input wire [SW-1:0] amt,
	input wire left,
	input wire arith,
	output wire [W-1:0] y
);
`ifdef SYNTH_KINTEX7
	localparam integer NGRP = (W + 7) / 8;
	wire fill_raw = ((DIR == 1) ? arith : (!left && arith)) ? a[W-1] : 1'b0;
	wire [W-1:0] stg [0:SW];
	assign stg[0] = a;
	genvar k, g, b;
	generate
		for (k = 0; k < SW; k = k + 1) begin : g_st
			localparam integer ST = (k == 0) ? 1 : (k == 1) ? 2 : (k == 2) ? 4 : (k == 3) ? 8 :
				(k == 4) ? 16 : (k == 5) ? 32 : (k == 6) ? 64 : 128;
			wire left_stg, fill_stg;
			if (DIR == 0) begin : g_lstg
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lstg (.I0(left), .O(left_stg));
			end else begin : g_nlstg
				assign left_stg = 1'b0;
			end
			if (DIR != 2) begin : g_fstg
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fstg (.I0(fill_raw), .O(fill_stg));
			end else begin : g_nfstg
				assign fill_stg = 1'b0;
			end
			for (g = 0; g < NGRP; g = g + 1) begin : g_grp
				localparam integer LO = g * 8;
				localparam integer HI = ((LO + 8) < W) ? (LO + 8) : W;
				wire amt_g, left_g, fill_g;
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_amg (.I0(amt[k]), .O(amt_g));
				if (DIR == 0) begin : g_lg
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lfg (.I0(left_stg), .O(left_g));
				end else begin : g_nlg
					assign left_g = 1'b0;
				end
				if (DIR != 2) begin : g_fg
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flg (.I0(fill_stg), .O(fill_g));
				end else begin : g_nfg
					assign fill_g = 1'b0;
				end
				for (b = LO; b < HI; b = b + 1) begin : g_bit
					wire lval, rval;
					if (DIR != 1) begin : g_l
						if (b >= ST) begin : g_in
							assign lval = stg[k][b - ST];
						end else begin : g_out
							assign lval = 1'b0;
						end
					end else begin : g_nl
						assign lval = 1'b0;
					end
					if (DIR != 2) begin : g_r
						if ((b + ST) < W) begin : g_in
							assign rval = stg[k][b + ST];
						end else begin : g_out
							assign rval = fill_g;
						end
					end else begin : g_nr
						assign rval = 1'b0;
					end
					if (DIR == 1) begin : g_o1
						assign stg[k + 1][b] = amt_g ? rval : stg[k][b];
					end else if (DIR == 2) begin : g_o2
						assign stg[k + 1][b] = amt_g ? lval : stg[k][b];
					end else begin : g_o0
						assign stg[k + 1][b] = amt_g ? (left_g ? lval : rval) : stg[k][b];
					end
				end
			end
		end
	endgenerate
	assign y = stg[SW];
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam integer NGRP = (W + 7) / 8;
	wire fill_raw = ((DIR == 1) ? arith : (!left && arith)) ? a[W-1] : 1'b0;
	wire [W-1:0] stg [0:SW];
	assign stg[0] = a;
	genvar k, g, b;
	generate
		for (k = 0; k < SW; k = k + 1) begin : g_st
			localparam integer ST = (k == 0) ? 1 : (k == 1) ? 2 : (k == 2) ? 4 : (k == 3) ? 8 :
				(k == 4) ? 16 : (k == 5) ? 32 : (k == 6) ? 64 : 128;
			wire left_stg, fill_stg;
			if (DIR == 0) begin : g_lstg
				assign left_stg = left;
			end else begin : g_nlstg
				assign left_stg = 1'b0;
			end
			if (DIR != 2) begin : g_fstg
				assign fill_stg = fill_raw;
			end else begin : g_nfstg
				assign fill_stg = 1'b0;
			end
			for (g = 0; g < NGRP; g = g + 1) begin : g_grp
				localparam integer LO = g * 8;
				localparam integer HI = ((LO + 8) < W) ? (LO + 8) : W;
				wire amt_g, left_g, fill_g;
				assign amt_g = amt[k];
				if (DIR == 0) begin : g_lg
					assign left_g = left_stg;
				end else begin : g_nlg
					assign left_g = 1'b0;
				end
				if (DIR != 2) begin : g_fg
					assign fill_g = fill_stg;
				end else begin : g_nfg
					assign fill_g = 1'b0;
				end
				for (b = LO; b < HI; b = b + 1) begin : g_bit
					wire lval, rval;
					if (DIR != 1) begin : g_l
						if (b >= ST) begin : g_in
							assign lval = stg[k][b - ST];
						end else begin : g_out
							assign lval = 1'b0;
						end
					end else begin : g_nl
						assign lval = 1'b0;
					end
					if (DIR != 2) begin : g_r
						if ((b + ST) < W) begin : g_in
							assign rval = stg[k][b + ST];
						end else begin : g_out
							assign rval = fill_g;
						end
					end else begin : g_nr
						assign rval = 1'b0;
					end
					if (DIR == 1) begin : g_o1
						assign stg[k + 1][b] = amt_g ? rval : stg[k][b];
					end else if (DIR == 2) begin : g_o2
						assign stg[k + 1][b] = amt_g ? lval : stg[k][b];
					end else begin : g_o0
						assign stg[k + 1][b] = amt_g ? (left_g ? lval : rval) : stg[k][b];
					end
				end
			end
		end
	endgenerate
	assign y = stg[SW];
`else
	am_shift #(.W(W), .SW(SW), .DIR(DIR), .USE_DSP(0)) u_sh (
		.a(a), .amt(amt), .left(left), .arith(arith), .y(y)
	);
`endif
endmodule

`default_nettype wire
