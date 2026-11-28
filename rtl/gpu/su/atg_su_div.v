/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_div: The SU's shared integer divider: div, divu, rem, remu
 * (RISC-V semantics), lane by lane, results as long beats.
 *
 * Takes one lane group at a time (x_ready is 1 only in IDLE), divides
 * its EXEC lanes one after the other with a 32-step restoring divider (34
 * cycles per lane: load, 32 steps, sign fix), then returns the group as
 * one long beat (l_wr 1, l_last = the group's x_last, class x).  A group
 * without EXEC lanes returns at once.  Division by zero gives all ones
 * (div/divu) and the dividend (rem/remu); INT_MIN / -1 gives INT_MIN and
 * remainder 0 (RISC-V).  The reference is the reference model gsi1fp div().
 *
 * States: IDLE →(group)→ DIV (lane loop) → RET →(l_ready)→ IDLE.
*/

`timescale 1ns / 1ps
`default_nettype none

module atg_su_div #(
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

	localparam DW = NPHYS * 32;
	localparam LW = NPHYS > 1 ? $clog2(NPHYS) : 1;
	localparam [1:0] IDLE = 2'd0, DIV = 2'd1, RET = 2'd2;

	(* max_fanout = 12 *) reg [1:0] st;
	(* max_fanout = 12 *) reg [NPHYS-1:0] rm;
	(* max_fanout = 12 *) reg [1:0] rop;
	reg [6:0] rrd;
	reg [WARP_W-1:0] rwarp;
	reg [GRP_W-1:0] rgrp;
	reg rlast;
	reg [SBE_W:0] rsbe;
	(* max_fanout = 12 *) reg [LW-1:0] li;
	(* max_fanout = 12 *) reg [5:0] step;
	/* The divider of the current lane. */
	(* max_fanout = 12 *) reg [31:0] dn, dd, q;
	(* max_fanout = 12 *) reg [31:0] r;
	(* max_fanout = 12 *) reg r32;
	(* max_fanout = 12 *) reg neg_q, neg_r, lb_zero_r;
	wire [31:0] la_cur_r, lb_cur_r;

	(* max_fanout = 12 *) wire mine = x_valid && x_uop[ATG_UOP_UNIT +: 4] == ATG_UNIT_DIV;
	assign x_ready = st == IDLE;

	/* The next lane in the mask at or after lane n (NPHYS when none). */
	wire [NPHYS-1:0] x_mask_rev;
	wire [NPHYS-1:0] rm_after;
	wire [NPHYS-1:0] rm_after_rev;
	genvar gdr;
	generate
		for (gdr = 0; gdr < NPHYS; gdr = gdr + 1) begin : g_div_rev
			if (gdr == 0) begin : g_div_z
				assign rm_after[gdr] = 1'b0;
			end else begin : g_div_nz
				assign rm_after[gdr] = rm[gdr] && ({{(32-LW){1'b0}}, li} < gdr);
			end
			assign x_mask_rev[gdr]   = x_mask[NPHYS - 1 - gdr];
			assign rm_after_rev[gdr] = rm_after[NPHYS - 1 - gdr];
		end
	endgenerate
	wire [LW-1:0] first_rev_n, after_rev_n;
	wire first_nz, after_nz;
	am_msb #(.W(NPHYS), .SW(LW)) u_div_first (.a(x_mask_rev),   .n(first_rev_n), .nz(first_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_div_after (.a(rm_after_rev), .n(after_rev_n), .nz(after_nz));
	wire [LW:0] first = first_nz ? {1'b0, ~first_rev_n} : NPHYS[LW:0];
	wire [LW:0] after = after_nz ? {1'b0, ~after_rev_n} : NPHYS[LW:0];
	(* keep = "true" *) reg [LW-1:0] after_q_a0, after_q_a1, after_q_b0, after_q_b1;
	(* keep = "true" *) reg after_nz_q;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	(* keep = "true" *) reg [NPHYS-1:0] rm_a0, rm_a1, rm_b0, rm_b1;
	(* keep = "true" *) reg [LW-1:0] li_a0, li_a1, li_b0, li_b1;
	wire [NPHYS-1:0] rm_after_a0, rm_after_a1, rm_after_b0, rm_after_b1;
	wire [NPHYS-1:0] rm_after_rev_a0, rm_after_rev_a1, rm_after_rev_b0, rm_after_rev_b1;
	generate
		for (gdr = 0; gdr < NPHYS; gdr = gdr + 1) begin : g_div_rev_rep
			if (gdr == 0) begin : g_div_rep_z
				assign rm_after_a0[gdr] = 1'b0;
				assign rm_after_a1[gdr] = 1'b0;
				assign rm_after_b0[gdr] = 1'b0;
				assign rm_after_b1[gdr] = 1'b0;
			end else begin : g_div_rep_nz
				assign rm_after_a0[gdr] = rm_a0[gdr] && ({{(32-LW){1'b0}}, li_a0} < gdr);
				assign rm_after_a1[gdr] = rm_a1[gdr] && ({{(32-LW){1'b0}}, li_a1} < gdr);
				assign rm_after_b0[gdr] = rm_b0[gdr] && ({{(32-LW){1'b0}}, li_b0} < gdr);
				assign rm_after_b1[gdr] = rm_b1[gdr] && ({{(32-LW){1'b0}}, li_b1} < gdr);
			end
			assign rm_after_rev_a0[gdr] = rm_after_a0[NPHYS - 1 - gdr];
			assign rm_after_rev_a1[gdr] = rm_after_a1[NPHYS - 1 - gdr];
			assign rm_after_rev_b0[gdr] = rm_after_b0[NPHYS - 1 - gdr];
			assign rm_after_rev_b1[gdr] = rm_after_b1[NPHYS - 1 - gdr];
		end
	endgenerate
	wire [LW-1:0] after_n_a0, after_n_a1, after_n_b0, after_n_b1;
	wire after_nz_a0, after_nz_a1, after_nz_b0, after_nz_b1;
	am_msb #(.W(NPHYS), .SW(LW)) u_da_a0 (.a(rm_after_rev_a0), .n(after_n_a0), .nz(after_nz_a0));
	am_msb #(.W(NPHYS), .SW(LW)) u_da_a1 (.a(rm_after_rev_a1), .n(after_n_a1), .nz(after_nz_a1));
	am_msb #(.W(NPHYS), .SW(LW)) u_da_b0 (.a(rm_after_rev_b0), .n(after_n_b0), .nz(after_nz_b0));
	am_msb #(.W(NPHYS), .SW(LW)) u_da_b1 (.a(rm_after_rev_b1), .n(after_n_b1), .nz(after_nz_b1));
	wire [LW-1:0] after_a0 = ~after_n_a0;
	wire [LW-1:0] after_a1 = ~after_n_a1;
	wire [LW-1:0] after_b0 = ~after_n_b0;
	wire [LW-1:0] after_b1 = ~after_n_b1;
	wire unused_da_nz = &{1'b0, after_nz_a0, after_nz_a1, after_nz_b0, after_nz_b1};
`endif
	wire [DW-1:0] ra, rb, rq;
	wire [31:0] la = la_cur_r;
	wire [31:0] lb = lb_cur_r;
	wire sgn = !rop[0];	/* div, rem */

	/*
	 * One 33-bit subtractor, by step: 0 - a at the load (step 0), the
	 * trial r:dn[31] - d of the steps 1-32, 0 - the result at the end; and
	 * 0 - b at the load.
	 */
	reg s_load_r, s_end_r;
	(* dont_touch = "true" *) reg step_ph, step_ph_b, step_ph_c, step_ph_d, step_ph_s;
	wire step_ph_e;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sphe (.I0(step_ph), .O(step_ph_e));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign step_ph_e = step_ph;
`else
	assign step_ph_e = step_ph;
`endif
	wire [3:0] st_step_ph = {st, step_ph_e};
	wire step_ph_done = ((st == DIV) && step_ph_e && !s_load && s_end);
	(* max_fanout = 12 *) wire s_load = s_load_r;
	(* max_fanout = 12 *) wire s_end = s_end_r;
	wire [15:0] d0_lo_w, nb_lo_w, nb_hi0_w, nb_hi1_w;
	wire d0_bo16_w, nb_bo16_w;
	wire [16:0] d0_hi0_w, d0_hi1_w;
	reg [15:0] d0_lo_r, nb_lo_r, nb_hi0_r, nb_hi1_r;
	(* dont_touch = "true" *) reg d0_bo16_r, d0_bo16_r1, d0_bo16_r2, nb_bo16_r;
	reg [16:0] d0_hi0_r, d0_hi1_r;
	wire d0_bo16_0, d0_bo16_1a, d0_bo16_1b, d0_bo16_1c, d0_bo16_1d, nb_bo16_0, nb_bo16_1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d0b0  (.I0(d0_bo16_r),  .O(d0_bo16_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d0b1a (.I0(d0_bo16_r),  .O(d0_bo16_1a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d0b1b (.I0(d0_bo16_r1), .O(d0_bo16_1b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d0b1c (.I0(d0_bo16_r2), .O(d0_bo16_1c));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_d0b1d (.I0(d0_bo16_r2), .O(d0_bo16_1d));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nbb0  (.I0(nb_bo16_r),  .O(nb_bo16_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_nbb1  (.I0(nb_bo16_r),  .O(nb_bo16_1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign d0_bo16_0 = d0_bo16_r;
	assign d0_bo16_1a = d0_bo16_r;
	assign d0_bo16_1b = d0_bo16_r1;
	assign d0_bo16_1c = d0_bo16_r2;
	assign d0_bo16_1d = d0_bo16_r2;
	assign nb_bo16_0 = nb_bo16_r;
	assign nb_bo16_1 = nb_bo16_r;
`else
	assign d0_bo16_0  = d0_bo16_r;
	assign d0_bo16_1a = d0_bo16_r;
	assign d0_bo16_1b = d0_bo16_r1;
	assign d0_bo16_1c = d0_bo16_r2;
	assign d0_bo16_1d = d0_bo16_r2;
	assign nb_bo16_0  = nb_bo16_r;
	assign nb_bo16_1  = nb_bo16_r;
`endif
	wire d0_msb = d0_bo16_1d ? d0_hi1_r[16] : d0_hi0_r[16];
	(* max_fanout = 12 *) wire [32:0] d0 = {
		d0_msb,
		d0_bo16_1c ? d0_hi1_r[15]     : d0_hi0_r[15],
		d0_bo16_1b ? d0_hi1_r[14:12]  : d0_hi0_r[14:12],
		d0_bo16_1a ? d0_hi1_r[11:8]   : d0_hi0_r[11:8],
		d0_bo16_0  ? d0_hi1_r[7:0]    : d0_hi0_r[7:0],
		d0_lo_r
	};
	wire [31:0] neg_b = {
		nb_bo16_1 ? nb_hi1_r[15:8] : nb_hi0_r[15:8],
		nb_bo16_0 ? nb_hi1_r[7:0]  : nb_hi0_r[7:0],
		nb_lo_r
	};
	wire [5:0] step_next;
	wire d0_hi0_bo_unused, d0_hi1_bo_unused, nb_hi0_bo_unused, nb_hi1_bo_unused, sn_co_unused;
	wire nb_bo_unused = &{1'b0, nb_hi0_bo_unused, nb_hi1_bo_unused};
	/* Pipelined 2-cycle 33-bit / 32-bit carry-select subtractors (16b lo + parallel hi0/hi1). */
	wire [32:0] d0_a = s_load || s_end ? 33'd0 : {r[31:0], dn[31]};
	wire [32:0] d0_b = s_load ? {1'b0, la} : s_end ? {1'b0, q} : {1'b0, dd};
	wire [31:0] nb_b = s_load ? lb : r[31:0];
	am_sub #(.W(16)) u_d0_lo  (.a(d0_a[15:0]),  .b(d0_b[15:0]),  .bi(1'b0), .d(d0_lo_w),  .bo(d0_bo16_w));
	am_sub #(.W(17)) u_d0_hi0 (.a(d0_a[32:16]), .b(d0_b[32:16]), .bi(1'b0), .d(d0_hi0_w), .bo(d0_hi0_bo_unused));
	am_sub #(.W(17)) u_d0_hi1 (.a(d0_a[32:16]), .b(d0_b[32:16]), .bi(1'b1), .d(d0_hi1_w), .bo(d0_hi1_bo_unused));
	am_sub #(.W(16)) u_nb_lo  (.a(16'd0),       .b(nb_b[15:0]),  .bi(1'b0), .d(nb_lo_w),  .bo(nb_bo16_w));
	am_sub #(.W(16)) u_nb_hi0 (.a(16'd0),       .b(nb_b[31:16]), .bi(1'b0), .d(nb_hi0_w), .bo(nb_hi0_bo_unused));
	am_sub #(.W(16)) u_nb_hi1 (.a(16'd0),       .b(nb_b[31:16]), .bi(1'b1), .d(nb_hi1_w), .bo(nb_hi1_bo_unused));

	am_add #(.W(6)) u_step (.a(step), .b(6'd0), .ci(1'b1), .s(step_next), .co(sn_co_unused));
	wire trial_pos, trial_pos_lo, trial_pos_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos  (.I0(d0_msb), .O(trial_pos));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos0 (.I0(d0_msb), .O(trial_pos_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos1 (.I0(d0_msb), .O(trial_pos_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos  (.I0(d0_msb), .O(trial_pos));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos0 (.I0(d0_msb), .O(trial_pos_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b01)) u_tpos1 (.I0(d0_msb), .O(trial_pos_hi));
`else
	assign trial_pos    = !d0[32];
	assign trial_pos_lo = !d0[32];
	assign trial_pos_hi = !d0[32];
`endif

	wire [31:0] div_res = lb_zero_r ? (rop[1] ? la : 32'hFFFFFFFF) :
		rop[1] ? (neg_r ? neg_b : r[31:0]) :
		(neg_q ? d0[31:0] : q);

	/* Buffered control and index signals */
	/* Buffered control and index signals */
	wire [LW-1:0] first_a_b0, first_a_b1, first_a_b2, first_a_b3;
	wire [LW-1:0] first_b_b0, first_b_b1, first_b_b2, first_b_b3;
	wire [LW-1:0] after_a_b0, after_a_b1, after_a_b2, after_a_b3;
	wire [LW-1:0] after_b_b0, after_b_b1, after_b_b2, after_b_b3;
	genvar gbf;
	generate
		for (gbf = 0; gbf < LW; gbf = gbf + 1) begin : g_buf_sel
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_ab0 (.I0(first[gbf]), .O(first_a_b0[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_ab1 (.I0(first[gbf]), .O(first_a_b1[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_ab2 (.I0(first[gbf]), .O(first_a_b2[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_ab3 (.I0(first[gbf]), .O(first_a_b3[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_bb0 (.I0(first[gbf]), .O(first_b_b0[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_bb1 (.I0(first[gbf]), .O(first_b_b1[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_bb2 (.I0(first[gbf]), .O(first_b_b2[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_f_bb3 (.I0(first[gbf]), .O(first_b_b3[gbf]));

			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_ab0 (.I0(after_q_a0[gbf]), .O(after_a_b0[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_ab1 (.I0(after_q_a0[gbf]), .O(after_a_b1[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_ab2 (.I0(after_q_a1[gbf]), .O(after_a_b2[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_ab3 (.I0(after_q_a1[gbf]), .O(after_a_b3[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_bb0 (.I0(after_q_b0[gbf]), .O(after_b_b0[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_bb1 (.I0(after_q_b0[gbf]), .O(after_b_b1[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_bb2 (.I0(after_q_b1[gbf]), .O(after_b_b2[gbf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_a_bb3 (.I0(after_q_b1[gbf]), .O(after_b_b3[gbf]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign first_a_b0[gbf] = first[gbf];
			assign first_a_b1[gbf] = first[gbf];
			assign first_a_b2[gbf] = first[gbf];
			assign first_a_b3[gbf] = first[gbf];
			assign first_b_b0[gbf] = first[gbf];
			assign first_b_b1[gbf] = first[gbf];
			assign first_b_b2[gbf] = first[gbf];
			assign first_b_b3[gbf] = first[gbf];

			assign after_a_b0[gbf] = after_q_a0[gbf];
			assign after_a_b1[gbf] = after_q_a0[gbf];
			assign after_a_b2[gbf] = after_q_a1[gbf];
			assign after_a_b3[gbf] = after_q_a1[gbf];
			assign after_b_b0[gbf] = after_q_b0[gbf];
			assign after_b_b1[gbf] = after_q_b0[gbf];
			assign after_b_b2[gbf] = after_q_b1[gbf];
			assign after_b_b3[gbf] = after_q_b1[gbf];
`else
			assign first_a_b0[gbf] = first[gbf];
			assign first_a_b1[gbf] = first[gbf];
			assign first_a_b2[gbf] = first[gbf];
			assign first_a_b3[gbf] = first[gbf];
			assign first_b_b0[gbf] = first[gbf];
			assign first_b_b1[gbf] = first[gbf];
			assign first_b_b2[gbf] = first[gbf];
			assign first_b_b3[gbf] = first[gbf];

			assign after_a_b0[gbf] = after_q_a0[gbf];
			assign after_a_b1[gbf] = after_q_a0[gbf];
			assign after_a_b2[gbf] = after_q_a1[gbf];
			assign after_a_b3[gbf] = after_q_a1[gbf];
			assign after_b_b0[gbf] = after_q_b0[gbf];
			assign after_b_b1[gbf] = after_q_b0[gbf];
			assign after_b_b2[gbf] = after_q_b1[gbf];
			assign after_b_b3[gbf] = after_q_b1[gbf];
`endif
		end
	endgenerate

	wire cur_load_raw = ((st == IDLE) && mine && (first != NPHYS[LW:0])) ||
	                    ((st == DIV) && step_ph && s_end && after_nz_q);
	wire idle_sel_raw = (st == IDLE);

	wire cur_load_ab0, cur_load_ab1, cur_load_ab2, cur_load_ab3;
	wire cur_load_bb0, cur_load_bb1, cur_load_bb2, cur_load_bb3;
	wire idle_sel_ab0, idle_sel_ab1, idle_sel_ab2, idle_sel_ab3;
	wire idle_sel_bb0, idle_sel_bb1, idle_sel_bb2, idle_sel_bb3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_ab0 (.I0(cur_load_raw), .O(cur_load_ab0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_ab1 (.I0(cur_load_raw), .O(cur_load_ab1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_ab2 (.I0(cur_load_raw), .O(cur_load_ab2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_ab3 (.I0(cur_load_raw), .O(cur_load_ab3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_bb0 (.I0(cur_load_raw), .O(cur_load_bb0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_bb1 (.I0(cur_load_raw), .O(cur_load_bb1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_bb2 (.I0(cur_load_raw), .O(cur_load_bb2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_cl_bb3 (.I0(cur_load_raw), .O(cur_load_bb3));

	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_ab0 (.I0(idle_sel_raw), .O(idle_sel_ab0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_ab1 (.I0(idle_sel_raw), .O(idle_sel_ab1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_ab2 (.I0(idle_sel_raw), .O(idle_sel_ab2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_ab3 (.I0(idle_sel_raw), .O(idle_sel_ab3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_bb0 (.I0(idle_sel_raw), .O(idle_sel_bb0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_bb1 (.I0(idle_sel_raw), .O(idle_sel_bb1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_bb2 (.I0(idle_sel_raw), .O(idle_sel_bb2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_is_bb3 (.I0(idle_sel_raw), .O(idle_sel_bb3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cur_load_ab0 = cur_load_raw;
	assign cur_load_ab1 = cur_load_raw;
	assign cur_load_ab2 = cur_load_raw;
	assign cur_load_ab3 = cur_load_raw;
	assign cur_load_bb0 = cur_load_raw;
	assign cur_load_bb1 = cur_load_raw;
	assign cur_load_bb2 = cur_load_raw;
	assign cur_load_bb3 = cur_load_raw;

	assign idle_sel_ab0 = idle_sel_raw;
	assign idle_sel_ab1 = idle_sel_raw;
	assign idle_sel_ab2 = idle_sel_raw;
	assign idle_sel_ab3 = idle_sel_raw;
	assign idle_sel_bb0 = idle_sel_raw;
	assign idle_sel_bb1 = idle_sel_raw;
	assign idle_sel_bb2 = idle_sel_raw;
	assign idle_sel_bb3 = idle_sel_raw;
`else
	assign cur_load_ab0 = cur_load_raw;
	assign cur_load_ab1 = cur_load_raw;
	assign cur_load_ab2 = cur_load_raw;
	assign cur_load_ab3 = cur_load_raw;
	assign cur_load_bb0 = cur_load_raw;
	assign cur_load_bb1 = cur_load_raw;
	assign cur_load_bb2 = cur_load_raw;
	assign cur_load_bb3 = cur_load_raw;

	assign idle_sel_ab0 = idle_sel_raw;
	assign idle_sel_ab1 = idle_sel_raw;
	assign idle_sel_ab2 = idle_sel_raw;
	assign idle_sel_ab3 = idle_sel_raw;
	assign idle_sel_bb0 = idle_sel_raw;
	assign idle_sel_bb1 = idle_sel_raw;
	assign idle_sel_bb2 = idle_sel_raw;
	assign idle_sel_bb3 = idle_sel_raw;
`endif

	reg [7:0] la_cur_b0, la_cur_b1, la_cur_b2, la_cur_b3;
	reg [7:0] lb_cur_b0, lb_cur_b1, lb_cur_b2, lb_cur_b3;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			la_cur_b0 <= 8'd0;
			la_cur_b1 <= 8'd0;
			la_cur_b2 <= 8'd0;
			la_cur_b3 <= 8'd0;
			lb_cur_b0 <= 8'd0;
			lb_cur_b1 <= 8'd0;
			lb_cur_b2 <= 8'd0;
			lb_cur_b3 <= 8'd0;
		end else begin
			if (cur_load_ab0)
				la_cur_b0 <= idle_sel_ab0 ? x_a[{first_a_b0, 5'd0} +: 8] : ra[{after_a_b0, 5'd0} +: 8];
			if (cur_load_ab1)
				la_cur_b1 <= idle_sel_ab1 ? x_a[{first_a_b1, 5'd8} +: 8] : ra[{after_a_b1, 5'd8} +: 8];
			if (cur_load_ab2)
				la_cur_b2 <= idle_sel_ab2 ? x_a[{first_a_b2, 5'd16} +: 8] : ra[{after_a_b2, 5'd16} +: 8];
			if (cur_load_ab3)
				la_cur_b3 <= idle_sel_ab3 ? x_a[{first_a_b3, 5'd24} +: 8] : ra[{after_a_b3, 5'd24} +: 8];

			if (cur_load_bb0)
				lb_cur_b0 <= idle_sel_bb0 ? x_b[{first_b_b0, 5'd0} +: 8] : rb[{after_b_b0, 5'd0} +: 8];
			if (cur_load_bb1)
				lb_cur_b1 <= idle_sel_bb1 ? x_b[{first_b_b1, 5'd8} +: 8] : rb[{after_b_b1, 5'd8} +: 8];
			if (cur_load_bb2)
				lb_cur_b2 <= idle_sel_bb2 ? x_b[{first_b_b2, 5'd16} +: 8] : rb[{after_b_b2, 5'd16} +: 8];
			if (cur_load_bb3)
				lb_cur_b3 <= idle_sel_bb3 ? x_b[{first_b_b3, 5'd24} +: 8] : rb[{after_b_b3, 5'd24} +: 8];
		end
	end
	assign la_cur_r = {la_cur_b3, la_cur_b2, la_cur_b1, la_cur_b0};
	assign lb_cur_r = {lb_cur_b3, lb_cur_b2, lb_cur_b1, lb_cur_b0};

	wire [3:0] idle_mine_g;
	genvar gimg;
	generate
		for (gimg = 0; gimg < 4; gimg = gimg + 1) begin : g_im_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_im (.I0((st == IDLE) && mine), .O(idle_mine_g[gimg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign idle_mine_g[gimg] = (st == IDLE) && mine;
`else
			assign idle_mine_g[gimg] = (st == IDLE) && mine;
`endif
		end
	endgenerate

	genvar gdl, gdh;
	generate
	for (gdl = 0; gdl < NPHYS; gdl = gdl + 1) begin : g_div_lane
		wire idle_mine_sl = idle_mine_g[(gdl * 4) / NPHYS];
		wire rq_we_lane;
		wire step_ph_lane = (gdl < (NPHYS / 2)) ? step_ph_b : step_ph_c;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rq_lane (.I0((st == DIV) && step_ph_lane && s_end && (li == gdl[LW-1:0])), .O(rq_we_lane));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign rq_we_lane = (st == DIV) && step_ph_lane && s_end && (li == gdl[LW-1:0]);
`else
		assign rq_we_lane = (st == DIV) && step_ph && s_end && (li == gdl[LW-1:0]);
`endif
		for (gdh = 0; gdh < 2; gdh = gdh + 1) begin : g_div_half
			wire ra_we_b, rb_we_b, rq_clr_b, rq_we_b;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ra_we  (.I0(idle_mine_sl), .O(ra_we_b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rb_we  (.I0(idle_mine_sl), .O(rb_we_b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rq_clr (.I0(idle_mine_sl), .O(rq_clr_b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rq_we  (.I0(rq_we_lane), .O(rq_we_b));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ra_we_b = idle_mine_sl;
			assign rb_we_b = idle_mine_sl;
			assign rq_clr_b = idle_mine_sl;
			assign rq_we_b = rq_we_lane;
`else
			assign ra_we_b = idle_mine_sl;
			assign rb_we_b = idle_mine_sl;
			assign rq_clr_b = idle_mine_sl;
			assign rq_we_b = rq_we_lane;
`endif
			(* shreg_extract = "no" *) reg [15:0] ra_h_r, rb_h_r, rq_h_r;
			always @(posedge clk) begin
				if (ra_we_b)
					ra_h_r <= x_a[gdl*32 + gdh*16 +: 16];
				if (rb_we_b)
					rb_h_r <= x_b[gdl*32 + gdh*16 +: 16];
				if (rq_clr_b)
					rq_h_r <= 16'd0;
				else if (rq_we_b)
					rq_h_r <= div_res[gdh*16 +: 16];
			end
			assign ra[gdl*32 + gdh*16 +: 16] = ra_h_r;
			assign rb[gdl*32 + gdh*16 +: 16] = rb_h_r;
			assign rq[gdl*32 + gdh*16 +: 16] = rq_h_r;
		end
	end
	wire [1:0] la31_b, lb31_b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_la31_0 (.I0(la[31]), .O(la31_b[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_la31_1 (.I0(la[31]), .O(la31_b[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_lb31_0 (.I0(lb[31]), .O(lb31_b[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_lb31_1 (.I0(lb[31]), .O(lb31_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign la31_b[0] = la[31];
	assign la31_b[1] = la[31];
	assign lb31_b[0] = lb[31];
	assign lb31_b[1] = lb[31];
`else
	assign la31_b[0] = la[31];
	assign la31_b[1] = la[31];
	assign lb31_b[0] = lb[31];
	assign lb31_b[1] = lb[31];
`endif

	for (gdh = 0; gdh < 2; gdh = gdh + 1) begin : g_div_op
		wire sgn_la31_h, sgn_lb31_h;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sla (.I0(sgn && la31_b[gdh]), .O(sgn_la31_h));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_slb (.I0(sgn && lb31_b[gdh]), .O(sgn_lb31_h));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sgn_la31_h = sgn && la31_b[gdh];
		assign sgn_lb31_h = sgn && lb31_b[gdh];
`else
		assign sgn_la31_h = sgn && la31_b[gdh];
		assign sgn_lb31_h = sgn && lb31_b[gdh];
`endif
		wire load_dn, load_dd, load_q, load_r;
		wire step_dn, step_q, step_r;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ldn (.I0((st == DIV) && step_ph_e && s_load), .O(load_dn));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ldd (.I0((st == DIV) && step_ph_e && s_load), .O(load_dd));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_lq  (.I0((st == DIV) && step_ph_e && s_load), .O(load_q));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_lr  (.I0((st == DIV) && step_ph_e && s_load), .O(load_r));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sdn (.I0((st == DIV) && step_ph_s && !s_load && !s_end), .O(step_dn));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sq  (.I0((st == DIV) && step_ph_s && !s_load && !s_end), .O(step_q));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sr  (.I0((st == DIV) && step_ph_s && !s_load && !s_end), .O(step_r));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign load_dn = (st == DIV) && step_ph_e && s_load;
		assign load_dd = (st == DIV) && step_ph_e && s_load;
		assign load_q = (st == DIV) && step_ph_e && s_load;
		assign load_r = (st == DIV) && step_ph_e && s_load;
		assign step_dn = (st == DIV) && step_ph_s && !s_load && !s_end;
		assign step_q = (st == DIV) && step_ph_s && !s_load && !s_end;
		assign step_r = (st == DIV) && step_ph_s && !s_load && !s_end;
`else
		assign load_dn = (st == DIV) && step_ph && s_load;
		assign load_dd = (st == DIV) && step_ph && s_load;
		assign load_q  = (st == DIV) && step_ph && s_load;
		assign load_r  = (st == DIV) && step_ph && s_load;
		assign step_dn = (st == DIV) && step_ph_s && !s_load && !s_end;
		assign step_q  = (st == DIV) && step_ph_s && !s_load && !s_end;
		assign step_r  = (st == DIV) && step_ph_s && !s_load && !s_end;
`endif
		always @(posedge clk) begin
			if (load_dn)
				dn[gdh*16 +: 16] <= sgn_la31_h ? d0[gdh*16 +: 16] : la[gdh*16 +: 16];
			else if (step_dn) begin
				if (gdh == 0)
					dn[15:0] <= {dn[14:0], 1'b0};
				else
					dn[31:16] <= dn[30:15];
			end
		end
		always @(posedge clk) begin
			if (load_dd)
				dd[gdh*16 +: 16] <= sgn_lb31_h ? neg_b[gdh*16 +: 16] : lb[gdh*16 +: 16];
		end
		always @(posedge clk) begin
			if (load_q)
				q[gdh*16 +: 16] <= 16'd0;
			else if (step_q) begin
				if (gdh == 0)
					q[15:0] <= {q[14:0], trial_pos};
				else
					q[31:16] <= q[30:15];
			end
		end
		always @(posedge clk) begin
			if (load_r)
				r[gdh*16 +: 16] <= 16'd0;
			else if (step_r) begin
				if (gdh == 0)
					r[15:0] <= trial_pos_lo ? d0[15:0] : {r[14:0], dn[31]};
				else
					r[31:16] <= trial_pos_hi ? d0[31:16] : r[30:15];
			end
		end
	end
	endgenerate

	wire rst_p = !rst_n;
	wire idle_mine_rm;
	wire idle_mine_ctl1, idle_mine_ctl2;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_im_rm (.I0((st == IDLE) && mine), .O(idle_mine_rm));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_im_c1 (.I0((st == IDLE) && mine), .O(idle_mine_ctl1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_im_c2 (.I0((st == IDLE) && mine), .O(idle_mine_ctl2));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign idle_mine_rm = (st == IDLE) && mine;
	assign idle_mine_ctl1 = (st == IDLE) && mine;
	assign idle_mine_ctl2 = (st == IDLE) && mine;
`else
	assign idle_mine_rm = (st == IDLE) && mine;
	assign idle_mine_ctl1 = (st == IDLE) && mine;
	assign idle_mine_ctl2 = (st == IDLE) && mine;
`endif

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			rm <= {NPHYS{1'b0}};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			rm_a0 <= {NPHYS{1'b0}};
			rm_a1 <= {NPHYS{1'b0}};
			rm_b0 <= {NPHYS{1'b0}};
			rm_b1 <= {NPHYS{1'b0}};
`endif
			rop <= 2'd0;
			rrd <= 7'd0;
			rwarp <= {WARP_W{1'b0}};
			rgrp <= {GRP_W{1'b0}};
			rlast <= 1'b0;
			rsbe <= {(SBE_W+1){1'b0}};
		end else begin
			if (idle_mine_rm) begin
				rm <= x_mask[NPHYS-1:0];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				rm_a0 <= x_mask[NPHYS-1:0];
				rm_a1 <= x_mask[NPHYS-1:0];
				rm_b0 <= x_mask[NPHYS-1:0];
				rm_b1 <= x_mask[NPHYS-1:0];
`endif
			end
			if (idle_mine_ctl1) begin
				rop <= x_uop[ATG_UOP_OP +: 2];
				rrd <= x_uop[ATG_UOP_RD +: 7];
			end
			if (idle_mine_ctl2) begin
				rwarp <= x_warp;
				rgrp <= x_grp;
				rlast <= x_last;
				rsbe <= x_sbe;
			end
		end
	end

	wire [7:0] div_ph0_ce;
	genvar gdph;
	generate
		for (gdph = 0; gdph < 8; gdph = gdph + 1) begin : g_dph_ce
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dph (.I0((st == DIV) && !step_ph_d), .O(div_ph0_ce[gdph]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign div_ph0_ce[gdph] = (st == DIV) && !step_ph_d;
`else
			assign div_ph0_ce[gdph] = (st == DIV) && !step_ph_d;
`endif
		end
	endgenerate

	always @(posedge clk) begin
		if (div_ph0_ce[0]) d0_lo_r       <= d0_lo_w;
		if (div_ph0_ce[1]) d0_hi0_r[8:0] <= d0_hi0_w[8:0];
		if (div_ph0_ce[2]) d0_hi0_r[16:9] <= d0_hi0_w[16:9];
		if (div_ph0_ce[3]) d0_hi1_r[8:0] <= d0_hi1_w[8:0];
		if (div_ph0_ce[4]) d0_hi1_r[16:9] <= d0_hi1_w[16:9];
		if (div_ph0_ce[5]) nb_lo_r       <= nb_lo_w;
		if (div_ph0_ce[6]) nb_hi0_r      <= nb_hi0_w;
		if (div_ph0_ce[7]) nb_hi1_r      <= nb_hi1_w;
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= IDLE;
			li <= {LW{1'b0}};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			li_a0 <= {LW{1'b0}};
			li_a1 <= {LW{1'b0}};
			li_b0 <= {LW{1'b0}};
			li_b1 <= {LW{1'b0}};
`endif
			after_q_a0 <= {LW{1'b0}};
			after_q_a1 <= {LW{1'b0}};
			after_q_b0 <= {LW{1'b0}};
			after_q_b1 <= {LW{1'b0}};
			after_nz_q <= 1'b0;
			step <= 6'd0;
			s_load_r <= 1'b1;
			s_end_r  <= 1'b0;
			step_ph  <= 1'b0;
			step_ph_b <= 1'b0;
			step_ph_c <= 1'b0;
			step_ph_d <= 1'b0;
			step_ph_s <= 1'b0;
			d0_bo16_r <= 1'b0;
			d0_bo16_r1 <= 1'b0;
			d0_bo16_r2 <= 1'b0;
			r32 <= 1'b0;
			neg_q <= 1'b0;
			neg_r <= 1'b0;
			lb_zero_r <= 1'b0;
		end else begin
			case (st)
			IDLE: if (mine) begin
				step <= 6'd0;
				s_load_r <= 1'b1;
				s_end_r  <= 1'b0;
				step_ph  <= 1'b0;
				step_ph_b <= 1'b0;
				step_ph_c <= 1'b0;
				step_ph_d <= 1'b0;
				if (first == NPHYS[LW:0])
					st <= RET;
				else begin
					li <= first[LW-1:0];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					li_a0 <= first[LW-1:0];
					li_a1 <= first[LW-1:0];
					li_b0 <= first[LW-1:0];
					li_b1 <= first[LW-1:0];
`endif
					st <= DIV;
				end
			end
			DIV: begin
				if (!step_ph) begin
					d0_bo16_r <= d0_bo16_w;
					d0_bo16_r1 <= d0_bo16_w;
					d0_bo16_r2 <= d0_bo16_w;
					nb_bo16_r <= nb_bo16_w;
					step_ph   <= 1'b1;
					step_ph_b <= 1'b1;
					step_ph_c <= 1'b1;
					step_ph_d <= 1'b1;
					step_ph_s <= 1'b1;
				end else begin
					step_ph <= 1'b0;
					step_ph_b <= 1'b0;
					step_ph_c <= 1'b0;
					step_ph_d <= 1'b0;
					step_ph_s <= 1'b0;
					if (s_load_r) begin
						/* Load the lane: magnitudes and the signs of the results. */
						neg_q <= sgn && (la[31] ^ lb[31]);
						neg_r <= sgn && la[31];
						lb_zero_r <= (lb == 32'd0);
						r32 <= 1'b0;
						step <= 6'd1;
						s_load_r <= 1'b0;
						s_end_r  <= 1'b0;
					end else if (!s_end_r) begin
						if (step == 6'd1) begin
`ifdef SYNTH_KINTEX7
							after_q_a0 <= ~after_rev_n;
							after_q_a1 <= ~after_rev_n;
							after_q_b0 <= ~after_rev_n;
							after_q_b1 <= ~after_rev_n;
`elsif SYNTH_KINTEX_ULTRASCALE
							after_q_a0 <= ~after_rev_n;
							after_q_a1 <= ~after_rev_n;
							after_q_b0 <= ~after_rev_n;
							after_q_b1 <= ~after_rev_n;
`else
							after_q_a0 <= after_a0;
							after_q_a1 <= after_a1;
							after_q_b0 <= after_b0;
							after_q_b1 <= after_b1;
`endif
							after_nz_q <= after_nz;
						end
						r32 <= trial_pos ? d0[32] : r[31];
						step <= step_next;
						if (step == 6'd32)
							s_end_r <= 1'b1;
					end else begin
						step <= 6'd0;
						s_load_r <= 1'b1;
						s_end_r  <= 1'b0;
						if (!after_nz_q)
							st <= RET;
						else begin
							li <= after_q_a0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
							li_a0 <= after_q_a0;
							li_a1 <= after_q_a1;
							li_b0 <= after_q_b0;
							li_b1 <= after_q_b1;
`endif
							st <= DIV;
						end
					end
				end
			end
			default: if (l_ready)
				st <= IDLE;
			endcase
		end
	end

	wire l_valid_buf;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_l_val (.I0(st == RET), .O(l_valid_buf));
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
	assign l_lanes = rm;
	assign l_wr = 1'b1;
	assign l_last = rlast;
	assign l_sbe = rsbe;
	assign l_cls = ATG_CLS_X;

	wire unused_ok = &{1'b0, x_pc, x_c, x_mask[2*NPHYS-1:NPHYS], x_info, x_uop, r32,
		after, (NWARPS != 0), d0_hi0_bo_unused, d0_hi1_bo_unused};
endmodule

`default_nettype wire
