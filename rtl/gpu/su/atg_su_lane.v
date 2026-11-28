/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_lane: Cross-lane operations: qd (quad differences and swaps for
 * derivatives, design §3.4.1) always; shfl/vote/ballot with SUBGROUP
 * (staging registers, design §3.4.2).  Short results, SHORT_LAT cycles
 * after the group's slot on the bus.
 *
 * qd works inside a lane group (NPHYS >= 4), one group per cycle.
 * shfl/vote/ballot need all 32 lanes: the groups before the last are
 * staged as they pass (x_ready 1); the last group is staged in its first
 * cycle on the bus and then held there (x_ready 0) for G = 32/NPHYS more
 * cycles, in which the results of groups 0..G-1 enter the result pipeline,
 * one per cycle; x_ready is 1 in the last of them.  So group g's result
 * leaves SHORT_LAT cycles after cycle x_end - (G-1) + g, the same as for
 * an instruction whose groups pass one per cycle: atg_su_sb's short
 * interlock and atg_su_wb's one-short-result-per-cycle rule hold, since
 * the bus carries nothing else in those cycles.  2 × G bus cycles.
 * The behaviour: spec/gpu/su/atg_su_lane.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_lane #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter SUBGROUP = 0,	/* shfl/vote/ballot present */
	parameter SHORT_LAT = 6	/* Cycles from x_valid to the short result (one fixed pipeline for classes a and f, design §3.1.1) */
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
	output wire sg_emit_done,
	/* Short result to atg_su_wb (exactly SHORT_LAT cycles after x_valid) */
	output wire s_valid,
	output wire [7:0] s_valid_rep,
	output wire [WARP_W-1:0] s_warp,
	output wire [GRP_W-1:0] s_grp,
	output wire [6:0] s_rd,
	output wire [NPHYS*32-1:0] s_data,
	output wire [NPHYS-1:0] s_lanes
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam G = 32 / NPHYS;
	localparam EW = 1 + WARP_W + GRP_W + 7 + NPHYS + DW;

	(* dont_touch = "true" *) wire rst_p = !rst_n;
	wire [3:0] unit = x_uop[ATG_UOP_UNIT +: 4];
	wire [7:0] op = x_uop[ATG_UOP_OP +: 8];
	wire [4:0] sub = x_uop[ATG_UOP_SUB +: 5];
	wire wrd = x_uop[ATG_UOP_WRD];
	wire mine = x_valid && unit == ATG_UNIT_LANE && op == ATG_LANE_QD && wrd;
	/* A subgroup instruction on the bus (they are ISA faults without SUBGROUP). */
	wire sg = SUBGROUP != 0 && x_valid && unit == ATG_UNIT_LANE &&
		(op == ATG_LANE_SHFL || op == ATG_LANE_VOTE || op == ATG_LANE_BALLOT);

	/* Stage 0 registered input for QD instructions (cuts long path from OPD) */
	(* shreg_extract = "no" *) reg r0_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] r0_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  r0_grp;
	(* shreg_extract = "no" *) reg [6:0]        r0_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  r0_mask;
	(* shreg_extract = "no", max_fanout = 16 *) reg [2:0] r0_sub;
	(* shreg_extract = "no" *) reg [DW-1:0]     r0_xa;
	(* shreg_extract = "no" *) reg [(NPHYS/4)*32-1:0] r0_qa_a, r0_qa_b, r0_qb_a, r0_qb_b;

	wire [16*3-1:0] sub_ctl_p;
	wire [2:0] sub_pipe;
	wire [2:0] sub_ctl_vote;
`ifdef SYNTH_KINTEX7
	wire [2:0] sub_p0, sub_p1, sub_qa;
	genvar gsb_pre;
	generate
		for (gsb_pre = 0; gsb_pre < 3; gsb_pre = gsb_pre + 1) begin : g_sub_pre
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_p0 (.I0(sub[gsb_pre]), .O(sub_p0[gsb_pre]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_p1 (.I0(sub[gsb_pre]), .O(sub_p1[gsb_pre]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_qa (.I0(sub[gsb_pre]), .O(sub_qa[gsb_pre]));
		end
	endgenerate

	genvar gsb, gsp;
	generate
		for (gsp = 0; gsp < 16; gsp = gsp + 1) begin : g_sub_p
			wire [2:0] sub_src = (gsp < 8) ? sub_p0 : sub_p1;
			for (gsb = 0; gsb < 3; gsb = gsb + 1) begin : g_sub_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_ctl (.I0(sub_src[gsb]), .O(sub_ctl_p[gsp*3 + gsb]));
			end
		end
		for (gsb = 0; gsb < 3; gsb = gsb + 1) begin : g_sub_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_pip (.I0(sub_qa[gsb]), .O(sub_pipe[gsb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs_vot (.I0(sub_qa[gsb]), .O(sub_ctl_vote[gsb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] sub_p0, sub_p1, sub_qa;
	genvar gsb_pre;
	generate
		for (gsb_pre = 0; gsb_pre < 3; gsb_pre = gsb_pre + 1) begin : g_sub_pre
			assign sub_p0[gsb_pre] = sub[gsb_pre];
			assign sub_p1[gsb_pre] = sub[gsb_pre];
			assign sub_qa[gsb_pre] = sub[gsb_pre];
		end
	endgenerate

	genvar gsb, gsp;
	generate
		for (gsp = 0; gsp < 16; gsp = gsp + 1) begin : g_sub_p
			wire [2:0] sub_src = (gsp < 8) ? sub_p0 : sub_p1;
			for (gsb = 0; gsb < 3; gsb = gsb + 1) begin : g_sub_b
				assign sub_ctl_p[gsp*3 + gsb] = sub_src[gsb];
			end
		end
		for (gsb = 0; gsb < 3; gsb = gsb + 1) begin : g_sub_buf
			assign sub_pipe[gsb] = sub_qa[gsb];
			assign sub_ctl_vote[gsb] = sub_qa[gsb];
		end
	endgenerate
`else
	assign sub_ctl_p = {16{sub[2:0]}};
	assign sub_pipe = sub[2:0];
	assign sub_ctl_vote = sub[2:0];
`endif

	genvar gqi;
	generate
		wire [1:0] sub0_qa_grp;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qa_a (.I0(sub_qa[0]), .O(sub0_qa_grp[0]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qa_b (.I0(sub_qa[0]), .O(sub0_qa_grp[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sub0_qa_grp[0] = sub_qa[0];
		assign sub0_qa_grp[1] = sub_qa[0];
`else
		assign sub0_qa_grp = {2{sub[0]}};
`endif
		for (gqi = 0; gqi < NPHYS / 4; gqi = gqi + 1) begin : g_qa_mux
			wire sub0_qa_lo, sub0_qa_hi;
			wire sub0_qb_lo, sub0_qb_hi;
			wire sub0_src = sub0_qa_grp[gqi / ((NPHYS / 4 + 1) / 2)];
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qa_lo (.I0(sub0_src), .O(sub0_qa_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qa_hi (.I0(sub0_src), .O(sub0_qa_hi));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qb_lo (.I0(sub0_src), .O(sub0_qb_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bs0_qb_hi (.I0(sub0_src), .O(sub0_qb_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign sub0_qa_lo = sub0_src;
			assign sub0_qa_hi = sub0_src;
			assign sub0_qb_lo = sub0_src;
			assign sub0_qb_hi = sub0_src;
`else
			assign sub0_qa_lo = sub[0];
			assign sub0_qa_hi = sub[0];
			assign sub0_qb_lo = sub[0];
			assign sub0_qb_hi = sub[0];
`endif
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					r0_qa_a[gqi*32 +: 32] <= 32'd0;
					r0_qa_b[gqi*32 +: 32] <= 32'd0;
					r0_qb_a[gqi*32 +: 32] <= 32'd0;
					r0_qb_b[gqi*32 +: 32] <= 32'd0;
				end else begin
					r0_qa_b[gqi*32 +: 32] <= x_a[(4*gqi)*32 +: 32];
					r0_qb_a[gqi*32 +: 32] <= x_a[(4*gqi + 3)*32 +: 32];
					r0_qa_a[gqi*32 +: 16] <= sub0_qa_lo ? x_a[(4*gqi + 2)*32 +: 16] : x_a[(4*gqi + 1)*32 +: 16];
					r0_qa_a[gqi*32+16 +: 16] <= sub0_qa_hi ? x_a[(4*gqi + 2)*32+16 +: 16] : x_a[(4*gqi + 1)*32+16 +: 16];
					r0_qb_b[gqi*32 +: 16] <= sub0_qb_lo ? x_a[(4*gqi + 1)*32 +: 16] : x_a[(4*gqi + 2)*32 +: 16];
					r0_qb_b[gqi*32+16 +: 16] <= sub0_qb_hi ? x_a[(4*gqi + 1)*32+16 +: 16] : x_a[(4*gqi + 2)*32+16 +: 16];
				end
			end
		end
	endgenerate

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			r0_valid <= 1'b0;
			r0_warp  <= {WARP_W{1'b0}};
			r0_grp   <= {GRP_W{1'b0}};
			r0_rd    <= 7'd0;
			r0_mask  <= {NPHYS{1'b0}};
			r0_sub   <= 3'd0;
			r0_xa    <= {DW{1'b0}};
		end else begin
			r0_valid <= mine;
			r0_warp  <= x_warp;
			r0_grp   <= x_grp;
			r0_rd    <= x_uop[ATG_UOP_RD +: 7];
			r0_mask  <= x_mask[NPHYS-1:0];
			r0_sub   <= sub_pipe;
			r0_xa    <= x_a;
		end
	end

	/* Stage 1 to 5 pipeline match for QD control & operands (quad-swap pre-muxed at r0 -> r1) */
	(* shreg_extract = "no" *) reg r1_valid, r2_valid, r3_valid;
	(* shreg_extract = "no", max_fanout = 12 *) reg r4_valid, r5_valid;
	(* shreg_extract = "no", keep = "true" *) reg [7:0] r5_valid_rep, pipe_v_s6_rep;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] r1_warp, r2_warp, r3_warp, r4_warp, r5_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  r1_grp,  r2_grp,  r3_grp,  r4_grp,  r5_grp;
	(* shreg_extract = "no" *) reg [6:0]        r1_rd,   r2_rd,   r3_rd,   r4_rd,   r5_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  r1_mask, r2_mask, r3_mask, r4_mask, r5_mask;
	(* shreg_extract = "no" *) reg [2:0]        r1_sub,  r2_sub,  r3_sub,  r4_sub;
	(* shreg_extract = "no" *) reg [DW-1:0]     r1_xa,   r2_xa,   r3_xa,   r4_xa,   r5_xa;
	(* shreg_extract = "no", keep = "true", equivalent_register_removal = "no", max_fanout = 16 *) reg [NPHYS*2-1:0] r4_use_qa, r4_use_qb, r4_use_xa;
	(* shreg_extract = "no", keep = "true", equivalent_register_removal = "no", max_fanout = 16 *) reg [NPHYS*2-1:0] r5_use_qa, r5_use_qb, r5_use_xa;

	integer qm;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			r1_valid <= 1'b0; r2_valid <= 1'b0; r3_valid <= 1'b0; r4_valid <= 1'b0; r5_valid <= 1'b0;
			r5_valid_rep <= 8'd0;
			r1_warp  <= {WARP_W{1'b0}}; r2_warp <= {WARP_W{1'b0}}; r3_warp <= {WARP_W{1'b0}}; r4_warp <= {WARP_W{1'b0}}; r5_warp <= {WARP_W{1'b0}};
			r1_grp   <= {GRP_W{1'b0}};  r2_grp  <= {GRP_W{1'b0}};  r3_grp  <= {GRP_W{1'b0}};  r4_grp  <= {GRP_W{1'b0}};  r5_grp  <= {GRP_W{1'b0}};
			r1_rd    <= 7'd0;           r2_rd   <= 7'd0;           r3_rd   <= 7'd0;           r4_rd   <= 7'd0;           r5_rd   <= 7'd0;
			r1_mask  <= {NPHYS{1'b0}};  r2_mask <= {NPHYS{1'b0}};  r3_mask <= {NPHYS{1'b0}};  r4_mask <= {NPHYS{1'b0}};  r5_mask <= {NPHYS{1'b0}};
			r1_sub   <= 3'd0;           r2_sub  <= 3'd0;           r3_sub  <= 3'd0;           r4_sub  <= 3'd0;
			r1_xa    <= {DW{1'b0}};     r2_xa   <= {DW{1'b0}};     r3_xa   <= {DW{1'b0}};     r4_xa   <= {DW{1'b0}};     r5_xa   <= {DW{1'b0}};
			r4_use_qa <= {(NPHYS*2){1'b0}};
			r4_use_qb <= {(NPHYS*2){1'b0}};
			r4_use_xa <= {(NPHYS*2){1'b0}};
			r5_use_qa <= {(NPHYS*2){1'b0}};
			r5_use_qb <= {(NPHYS*2){1'b0}};
			r5_use_xa <= {(NPHYS*2){1'b0}};
		end else begin
			r1_valid <= r0_valid; r2_valid <= r1_valid; r3_valid <= r2_valid; r4_valid <= r3_valid; r5_valid <= r4_valid;
			r5_valid_rep <= {8{r4_valid}};
			r1_warp  <= r0_warp;  r2_warp  <= r1_warp;  r3_warp  <= r2_warp;  r4_warp  <= r3_warp;  r5_warp  <= r4_warp;
			r1_grp   <= r0_grp;   r2_grp   <= r1_grp;   r3_grp   <= r2_grp;   r4_grp   <= r3_grp;   r5_grp   <= r4_grp;
			r1_rd    <= r0_rd;    r2_rd    <= r1_rd;    r3_rd    <= r2_rd;    r4_rd    <= r3_rd;    r5_rd    <= r4_rd;
			r1_mask  <= r0_mask;  r2_mask  <= r1_mask;  r3_mask  <= r2_mask;  r4_mask  <= r3_mask;  r5_mask  <= r4_mask;
			r1_sub   <= r0_sub;   r2_sub   <= r1_sub;   r3_sub   <= r2_sub;   r4_sub   <= r3_sub;
			for (qm = 0; qm < NPHYS; qm = qm + 1) begin
				r1_xa[qm*32 +: 32] <= (r0_sub == 3'd2) ? r0_xa[(qm ^ 1)*32 +: 32] :
				                      (r0_sub == 3'd3) ? r0_xa[(qm ^ 2)*32 +: 32] :
				                      (r0_sub == 3'd4) ? r0_xa[(qm ^ 3)*32 +: 32] :
				                      r0_xa[(qm & ~3)*32 +: 32];
				r4_use_qa[qm*2 +: 2] <= {2{r3_valid && ((r3_sub == 3'd0 && (qm & 2) == 0) || (r3_sub == 3'd1 && (qm & 1) == 0))}};
				r4_use_qb[qm*2 +: 2] <= {2{r3_valid && ((r3_sub == 3'd0 && (qm & 2) != 0) || (r3_sub == 3'd1 && (qm & 1) != 0))}};
				r4_use_xa[qm*2 +: 2] <= {2{r3_valid && (r3_sub >= 3'd2)}};
			end
			r2_xa <= r1_xa; r3_xa <= r2_xa; r4_xa <= r3_xa; r5_xa <= r4_xa;
			r5_use_qa <= r4_use_qa;
			r5_use_qb <= r4_use_qb;
			r5_use_xa <= r4_use_xa;
		end
	end

	/* qd: quads inside the group: lane l's partners l^1, l^2, l^3 and l&~3.
	 * The differences x[l|1] - x[l&~1] (sub 0) and x[l|2] - x[l&~2] (sub 1)
	 * take two values per quad, so a quad has two adders: A for lanes 0
	 * and 1 (sub 0) or 0 and 2 (sub 1), B for the other two. */
	wire [DW-1:0] qa_all, qb_all;
	genvar gq, gm;
	generate
		for (gq = 0; gq < NPHYS / 4; gq = gq + 1) begin : g_qd
			wire [31:0] qa, qb;
			wire unused_qa_v, unused_qb_v;
			am_fp_add #(.LAT(5)) u_qa (
				.clk(clk), .rst_n(rst_n), .i_valid(1'b1),
				.i_a(r0_qa_a[gq*32 +: 32]),
				.i_b(r0_qa_b[gq*32 +: 32]), .i_sub(1'b1),
				.o_valid(unused_qa_v), .o_r(qa)
			);
			am_fp_add #(.LAT(5)) u_qb (
				.clk(clk), .rst_n(rst_n), .i_valid(1'b1),
				.i_a(r0_qb_a[gq*32 +: 32]),
				.i_b(r0_qb_b[gq*32 +: 32]), .i_sub(1'b1),
				.o_valid(unused_qb_v), .o_r(qb)
			);
			for (gm = 4*gq; gm < 4*gq + 4; gm = gm + 1) begin : g_l
				assign qa_all[gm*32 +: 32] = qa;
				assign qb_all[gm*32 +: 32] = qb;
			end
		end
	endgenerate

	/* The state machine of the subgroup instructions: IDLE (groups are
	 * staged as they pass; the last one moves to EMIT) and EMIT (group
	 * e_cnt's result enters the pipeline; x_ready in the last). */
	localparam S_IDLE = 1'b0;
	localparam S_EMIT = 1'b1;
	reg st;
	reg [GRP_W-1:0] e_cnt;
	(* dont_touch = "true" *) reg [GRP_W-1:0] e_cnt_b;
	(* keep = "true" *) reg [NPHYS*3*GRP_W-1:0] e_cnt_rep;
	(* keep = "true" *) reg [31:0] st_idle_rep;
	wire e_last = {{(32-GRP_W){1'b0}}, e_cnt_b} == G - 1;
	wire emit = st == S_EMIT;
	wire stage = sg && st == S_IDLE;
	assign sg_emit_done = (SUBGROUP == 0) || (emit && e_last);
	assign x_ready = !(sg && x_last) || (emit && e_last);

	/* Staging, per warp lane: rs1, EXEC, and the shfl source lane (s_ok: it
	 * exists, 0..31). */
	reg [32*32-1:0] s_d;
	reg [31:0] s_ex;
	reg [31:0] s_ok, s_ok_hi_a, s_ok_hi_b, s_need_ok_hi;
	reg [32*5-1:0] s_src;

	/* The source lane of shfl for lane ln of the group on the bus. */
	/*
	 * Lane ln = x_grp NPHYS + l (disjoint bits); ln + rs2 or ln - rs2 (up,
	 * down) in one adder a lane.
	 */
	wire [NPHYS-1:0] c_ok, c_ok_hi_a, c_ok_hi_b, c_need_ok_hi;
	wire [NPHYS*5-1:0] c_src;
	wire [GRP_W-1:0] x_grp_src, x_grp_stg;
`ifdef SYNTH_KINTEX7
	genvar gxg;
	generate
		for (gxg = 0; gxg < GRP_W; gxg = gxg + 1) begin : g_xgrp_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxgrp_src (.I0(x_grp[gxg]), .O(x_grp_src[gxg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bxgrp_stg (.I0(x_grp[gxg]), .O(x_grp_stg[gxg]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gxg;
	generate
		for (gxg = 0; gxg < GRP_W; gxg = gxg + 1) begin : g_xgrp_buf
			assign x_grp_src[gxg] = x_grp[gxg];
			assign x_grp_stg[gxg] = x_grp[gxg];
		end
	endgenerate
`else
	assign x_grp_src = x_grp;
	assign x_grp_stg = x_grp;
`endif
	genvar gl;
	generate
		for (gl = 0; gl < NPHYS; gl = gl + 1) begin : g_src
			/* verilator lint_off WIDTHTRUNC */
			wire [4:0] ln = ({{(32-GRP_W){1'b0}}, x_grp_src} << $clog2(NPHYS)) | gl;
			/* verilator lint_on WIDTHTRUNC */
			wire [31:0] bv = x_b[gl*32 +: 32];
			wire [2:0] sub_ctl_l = (gl / 2 < 16) ? sub_ctl_p[(gl/2)*3 +: 3] : 3'd0;
	wire [4:0] ln_xor, ln_add, ln_sub;
	wire [3:0] sub_ctl_g;
`ifdef SYNTH_KINTEX7
	genvar glnb;
	for (glnb = 0; glnb < 5; glnb = glnb + 1) begin : g_ln_buf
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lnx (.I0(ln[glnb]), .O(ln_xor[glnb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lna (.I0(ln[glnb]), .O(ln_add[glnb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lns (.I0(ln[glnb]), .O(ln_sub[glnb]));
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar glnb;
	for (glnb = 0; glnb < 5; glnb = glnb + 1) begin : g_ln_buf
		assign ln_xor[glnb] = ln[glnb];
		assign ln_add[glnb] = ln[glnb];
		assign ln_sub[glnb] = ln[glnb];
	end
`else
	assign ln_xor = ln; assign ln_add = ln; assign ln_sub = ln;
`endif
			wire [5:0] sum_add, sum_sub;
			wire co_add_unused, bo_sub_unused;
			am_add #(.W(6)) u_sum_add (.a({1'b0, ln_add}), .b({1'b0, bv[4:0]}), .ci(1'b0),
				.s(sum_add), .co(co_add_unused));
			am_sub #(.W(6)) u_sum_sub (.a({1'b0, ln_sub}), .b({1'b0, bv[4:0]}), .bi(1'b0),
				.d(sum_sub), .bo(bo_sub_unused));
			(* keep = "true" *) wire nz0 = |bv[9:5];
			(* keep = "true" *) wire nz1 = |bv[15:10];
			(* keep = "true" *) wire nz2 = |bv[21:16];
			(* keep = "true" *) wire nz3 = |bv[26:22];
			(* keep = "true" *) wire nz4 = |bv[31:27];
			(* keep = "true" *) wire ok_hi = ~(nz0 | nz1 | nz2 | nz3 | nz4);
			/* (local regs: one block's writes do not wake the other lanes' blocks) */
			reg ok;
			reg [4:0] src;
			always @(*) begin
				ok = 1'b1;
				case (sub_ctl_l)
				3'd0: src = bv[4:0];
				3'd2: begin
					src = sum_sub[4:0];
					ok = !sum_sub[5];
				end
				3'd3: begin
					src = sum_add[4:0];
					ok = !sum_add[5];
				end
				3'd4: begin	/* rot: (l + rs2) mod 32, always a lane */
					src = sum_add[4:0];
					ok = 1'b1;
				end
				default: src = ln_xor ^ bv[4:0];	/* 1 xor */
				endcase
			end
			assign c_ok[gl]         = ok;
			assign c_ok_hi_a[gl]    = ~(nz0 | nz1);
			assign c_ok_hi_b[gl]    = ~(nz2 | nz3 | nz4);
			assign c_need_ok_hi[gl] = (sub_ctl_l != 3'd4);
			assign c_src[gl*5 +: 5] = src;
		end
	endgenerate

	/* Results of group e_cnt.  shfl: the source lane's rs1 when the source
	 * exists and is in EXEC, else the lane's own rs1.  ballot: the EXEC
	 * lanes whose rs1 is not 0; vote: any/all of them as a Boolean. */
	reg [31:0] bal;
	integer bl;
	always @(*) begin
		for (bl = 0; bl < 32; bl = bl + 1)
			bal[bl] = s_ex[bl] && (s_d[bl*32 +: 32] != 32'd0);
	end
	wire is_ballot = (op == ATG_LANE_BALLOT);
	wire is_vote   = (op == ATG_LANE_VOTE);
	wire is_shfl   = (op == ATG_LANE_SHFL);
	wire vote_val  = sub_ctl_vote[0] ? (bal == s_ex) : (bal != 32'd0);
	/* shfl writes the EXEC lanes; vote and ballot every lane. */
	wire [31:0] e_base = {{(32-GRP_W){1'b0}}, e_cnt_b} * NPHYS;
	wire [31:0] ex_g;
	wire [GRP_W-1:0] e_cnt_next;
	wire ec_co_unused;
	am_shift #(.W(32), .SW(6), .DIR(1)) u_ex_g (.a(s_ex), .amt(e_base[5:0]), .left(1'b0), .arith(1'b0),
		.y(ex_g));
	wire unused_eb = &{1'b0, e_base[31:6]};
	am_add #(.W(GRP_W)) u_e_cnt (.a(e_cnt_b), .b({GRP_W{1'b0}}), .ci(1'b1), .s(e_cnt_next),
		.co(ec_co_unused));
	wire [NPHYS-1:0] z_lanes = op == ATG_LANE_SHFL ? ex_g[NPHYS-1:0] : {NPHYS{1'b1}};

	wire [GRP_W-1:0] e_cnt_g [0:7];
`ifdef SYNTH_KINTEX7
	genvar gec, geg;
	generate
		for (geg = 0; geg < 8; geg = geg + 1) begin : g_ec_buf
			for (gec = 0; gec < GRP_W; gec = gec + 1) begin : g_ec_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ec (.I0(e_cnt[gec]), .O(e_cnt_g[geg][gec]));
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gec, geg;
	generate
		for (geg = 0; geg < 8; geg = geg + 1) begin : g_ec_buf
			for (gec = 0; gec < GRP_W; gec = gec + 1) begin : g_ec_b
				assign e_cnt_g[geg][gec] = e_cnt[gec];
			end
		end
	endgenerate
`else
	genvar geg;
	generate
		for (geg = 0; geg < 8; geg = geg + 1) begin : g_ec_buf
			assign e_cnt_g[geg] = e_cnt;
		end
	endgenerate
`endif

	/* Stage-0 shuffle pre-lookup -> registered *_d -> Stage-1 s_ex/s_d mux into pipe[1]. */
	wire [NPHYS*2-1:0] shfl_ok_h;
	wire [4:0] shfl_es_h [0:NPHYS*2-1];
	wire [15:0] shfl_self_h [0:NPHYS*2-1];
	genvar gzl, gzh;
	generate
		for (gzl = 0; gzl < NPHYS; gzl = gzl + 1) begin: g_shfl_lane
			wire [GRP_W-1:0] e_cnt_lane = e_cnt_g[gzl >> 2];
			wire [GRP_W-1:0] ec_es, ec_h0, ec_h1;
`ifdef SYNTH_KINTEX7
			genvar gec;
			for (gec = 0; gec < GRP_W; gec = gec + 1) begin : g_ec_lane_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bec_es (.I0(e_cnt_lane[gec]), .O(ec_es[gec]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bec_h0 (.I0(e_cnt_lane[gec]), .O(ec_h0[gec]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bec_h1 (.I0(e_cnt_lane[gec]), .O(ec_h1[gec]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gec;
			for (gec = 0; gec < GRP_W; gec = gec + 1) begin : g_ec_lane_buf
				assign ec_es[gec] = e_cnt_lane[gec];
				assign ec_h0[gec] = e_cnt_lane[gec];
				assign ec_h1[gec] = e_cnt_lane[gec];
			end
`else
			assign ec_es = e_cnt_rep[(gzl*3 + 0)*GRP_W +: GRP_W];
			assign ec_h0 = e_cnt_rep[(gzl*3 + 1)*GRP_W +: GRP_W];
			assign ec_h1 = e_cnt_rep[(gzl*3 + 2)*GRP_W +: GRP_W];
`endif
			/* verilator lint_off WIDTHTRUNC */
			/* verilator lint_off WIDTHEXPAND */
			wire [4:0] el_l_es = ({{(32-GRP_W){1'b0}}, ec_es} << $clog2(NPHYS)) | {27'd0, gzl[4:0]};
			wire [4:0] el_l_h0 = ({{(32-GRP_W){1'b0}}, ec_h0} << $clog2(NPHYS)) | {27'd0, gzl[4:0]};
			wire [4:0] el_l_h1 = ({{(32-GRP_W){1'b0}}, ec_h1} << $clog2(NPHYS)) | {27'd0, gzl[4:0]};
			/* verilator lint_on WIDTHEXPAND */
			/* verilator lint_on WIDTHTRUNC */
			reg [4:0] es_l;
			integer sl_idx;
			always @(*) begin
				es_l = s_src[4:0];
				for (sl_idx = 1; sl_idx < 32; sl_idx = sl_idx + 1) begin
					if (el_l_es == sl_idx[4:0])
						es_l = s_src[sl_idx*5 +: 5];
				end
			end
			wire s_ok_l = s_ok[el_l_es] && (!s_need_ok_hi[el_l_es] || (s_ok_hi_a[el_l_es] && s_ok_hi_b[el_l_es]));

			for (gzh = 0; gzh < 2; gzh = gzh + 1) begin: g_half
				localparam integer H_IDX = gzl*2 + gzh;
				wire [4:0] el_l_h = gzh[0] ? el_l_h1 : el_l_h0;
				assign shfl_ok_h[H_IDX]   = s_ok_l;
				assign shfl_es_h[H_IDX]   = es_l;
				assign shfl_self_h[H_IDX] = s_d[{el_l_h, gzh[0], 4'd0} +: 16];
			end
		end
	endgenerate

	/* Buffered bal per half-lane */
	wire [15:0] bal_h [0:NPHYS*2-1];
	genvar g_kp, g_b16;
	generate
		for (g_kp = 0; g_kp < NPHYS*2; g_kp = g_kp + 1) begin: g_bal_h
			wire [15:0] bal_src = g_kp[0] ? bal[31:16] : bal[15:0];
`ifdef SYNTH_KINTEX7
			for (g_b16 = 0; g_b16 < 16; g_b16 = g_b16 + 1) begin: g_bal_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bal (.I0(bal_src[g_b16]), .O(bal_h[g_kp][g_b16]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			for (g_b16 = 0; g_b16 < 16; g_b16 = g_b16 + 1) begin: g_bal_b
				assign bal_h[g_kp][g_b16] = bal_src[g_b16];
			end
`else
			assign bal_h[g_kp] = bal_src;
`endif
		end
	endgenerate

	(* shreg_extract = "no" *) reg [EW-1:0] pipe [0:SHORT_LAT-1];
	(* keep = "true" *) wire [DW-1:0] pipe_fb;
	(* keep = "true" *) wire [31:0] stg_we_lo, stg_we_hi;
	wire [NPHYS*2-1:0] is_ballot_h_lo, is_ballot_h_hi;
	wire [NPHYS*2-1:0] is_vote_h_lo, is_vote_h_hi;
	wire [NPHYS*2-1:0] vote_val_h_lo, vote_val_h_hi;
	wire [NPHYS*2-1:0] is_shfl_h_lo;
	(* shreg_extract = "no" *) reg [NPHYS*2-1:0] is_ballot_d, is_vote_d, is_shfl_d, shfl_ok_d;
	(* shreg_extract = "no" *) reg [15:0] bal_h_d [0:NPHYS*2-1];
	(* shreg_extract = "no" *) reg [NPHYS*2-1:0] vote_val_d;
	(* shreg_extract = "no" *) reg [4:0] shfl_es_d [0:NPHYS*2-1];
	(* shreg_extract = "no" *) reg [15:0] shfl_self_d [0:NPHYS*2-1];
	wire [15:0] shfl_val_d [0:NPHYS*2-1];
	wire emit_ctrl;
	wire is_ballot_g0, is_ballot_g1;
	wire is_vote_g0, is_vote_g1;
	wire vote_val_g0, vote_val_g1;
	wire is_shfl_g0, is_shfl_g1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_emit_ctrl (.I0(emit), .O(emit_ctrl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bg0       (.I0(is_ballot),  .O(is_ballot_g0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bg1       (.I0(is_ballot),  .O(is_ballot_g1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vt0       (.I0(is_vote),    .O(is_vote_g0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vt1       (.I0(is_vote),    .O(is_vote_g1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vg0       (.I0(vote_val),   .O(vote_val_g0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vg1       (.I0(vote_val),   .O(vote_val_g1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sh0       (.I0(is_shfl),    .O(is_shfl_g0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sh1       (.I0(is_shfl),    .O(is_shfl_g1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign emit_ctrl = emit;
	assign is_ballot_g0 = is_ballot;
	assign is_ballot_g1 = is_ballot;
	assign is_vote_g0 = is_vote;
	assign is_vote_g1 = is_vote;
	assign vote_val_g0 = vote_val;
	assign vote_val_g1 = vote_val;
	assign is_shfl_g0 = is_shfl;
	assign is_shfl_g1 = is_shfl;
`else
	assign emit_ctrl     = emit;
	assign is_ballot_g0  = is_ballot;
	assign is_ballot_g1  = is_ballot;
	assign is_vote_g0    = is_vote;
	assign is_vote_g1    = is_vote;
	assign vote_val_g0   = vote_val;
	assign vote_val_g1   = vote_val;
	assign is_shfl_g0    = is_shfl;
	assign is_shfl_g1    = is_shfl;
`endif

	genvar gp;
	generate
		for (gp = 0; gp < NPHYS * 2; gp = gp + 1) begin : g_r4v
			wire r4_use_xa_lo, r4_use_xa_hi;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_r4x_lo (.I0(r4_use_xa[gp]), .O(r4_use_xa_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_r4x_hi (.I0(r4_use_xa[gp]), .O(r4_use_xa_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign r4_use_xa_lo = r4_use_xa[gp];
			assign r4_use_xa_hi = r4_use_xa[gp];
`else
			assign r4_use_xa_lo = r4_use_xa[gp];
			assign r4_use_xa_hi = r4_use_xa[gp];
`endif
			assign pipe_fb[gp*16 +: 8]     = r4_use_xa_lo ? r4_xa[gp*16 +: 8]     : pipe[SHORT_LAT-2][gp*16 +: 8];
			assign pipe_fb[gp*16 + 8 +: 8] = r4_use_xa_hi ? r4_xa[gp*16 + 8 +: 8] : pipe[SHORT_LAT-2][gp*16 + 8 +: 8];
			wire is_ballot_gp  = (gp < NPHYS) ? is_ballot_g0  : is_ballot_g1;
			wire is_vote_gp    = (gp < NPHYS) ? is_vote_g0    : is_vote_g1;
			wire vote_val_gp   = (gp < NPHYS) ? vote_val_g0   : vote_val_g1;
			wire is_shfl_gp    = (gp < NPHYS) ? is_shfl_g0    : is_shfl_g1;
			wire [4:0] shfl_es_ex;
			wire [4:0] shfl_es_b0a, shfl_es_b0b, shfl_es_b1a, shfl_es_b1b;
			wire [4:0] shfl_es_b2a, shfl_es_b2b, shfl_es_b3a, shfl_es_b3b;
			wire sel_shfl_gp, sel_shfl_gp_0, sel_shfl_gp_1, sel_shfl_gp_2, sel_shfl_gp_3;
`ifdef SYNTH_KINTEX7
			genvar gseb;
			for (gseb = 0; gseb < 5; gseb = gseb + 1) begin : g_se_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seex  (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_ex[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb0a (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b0a[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb0b (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b0b[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb1a (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b1a[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb1b (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b1b[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb2a (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b2a[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb2b (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b2b[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb3a (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b3a[gseb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seb3b (.I0(shfl_es_d[gp][gseb]), .O(shfl_es_b3b[gseb]));
			end
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssg0 (.I0(sel_shfl_gp), .O(sel_shfl_gp_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssg1 (.I0(sel_shfl_gp), .O(sel_shfl_gp_1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssg2 (.I0(sel_shfl_gp), .O(sel_shfl_gp_2));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssg3 (.I0(sel_shfl_gp), .O(sel_shfl_gp_3));
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gseb;
			for (gseb = 0; gseb < 5; gseb = gseb + 1) begin : g_se_buf
				assign shfl_es_ex[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b0a[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b0b[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b1a[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b1b[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b2a[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b2b[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b3a[gseb] = shfl_es_d[gp][gseb];
				assign shfl_es_b3b[gseb] = shfl_es_d[gp][gseb];
			end
			assign sel_shfl_gp_0 = sel_shfl_gp;
			assign sel_shfl_gp_1 = sel_shfl_gp;
			assign sel_shfl_gp_2 = sel_shfl_gp;
			assign sel_shfl_gp_3 = sel_shfl_gp;
`else
			assign shfl_es_ex  = shfl_es_d[gp];
			assign shfl_es_b0a = shfl_es_d[gp];
			assign shfl_es_b0b = shfl_es_d[gp];
			assign shfl_es_b1a = shfl_es_d[gp];
			assign shfl_es_b1b = shfl_es_d[gp];
			assign shfl_es_b2a = shfl_es_d[gp];
			assign shfl_es_b2b = shfl_es_d[gp];
			assign shfl_es_b3a = shfl_es_d[gp];
			assign shfl_es_b3b = shfl_es_d[gp];
			assign sel_shfl_gp_0 = sel_shfl_gp;
			assign sel_shfl_gp_1 = sel_shfl_gp;
			assign sel_shfl_gp_2 = sel_shfl_gp;
			assign sel_shfl_gp_3 = sel_shfl_gp;
`endif
			assign sel_shfl_gp   = shfl_ok_d[gp] && s_ex[shfl_es_ex];
			assign shfl_val_d[gp][1:0]   = sel_shfl_gp_0 ? s_d[{shfl_es_b0a, gp[0], 4'd0} +: 2]  : shfl_self_d[gp][1:0];
			assign shfl_val_d[gp][3:2]   = sel_shfl_gp_0 ? s_d[{shfl_es_b0b, gp[0], 4'd2} +: 2]  : shfl_self_d[gp][3:2];
			assign shfl_val_d[gp][5:4]   = sel_shfl_gp_1 ? s_d[{shfl_es_b1a, gp[0], 4'd4} +: 2]  : shfl_self_d[gp][5:4];
			assign shfl_val_d[gp][7:6]   = sel_shfl_gp_1 ? s_d[{shfl_es_b1b, gp[0], 4'd6} +: 2]  : shfl_self_d[gp][7:6];
			assign shfl_val_d[gp][9:8]   = sel_shfl_gp_2 ? s_d[{shfl_es_b2a, gp[0], 4'd8} +: 2]  : shfl_self_d[gp][9:8];
			assign shfl_val_d[gp][11:10] = sel_shfl_gp_2 ? s_d[{shfl_es_b2b, gp[0], 4'd10} +: 2] : shfl_self_d[gp][11:10];
			assign shfl_val_d[gp][13:12] = sel_shfl_gp_3 ? s_d[{shfl_es_b3a, gp[0], 4'd12} +: 2] : shfl_self_d[gp][13:12];
			assign shfl_val_d[gp][15:14] = sel_shfl_gp_3 ? s_d[{shfl_es_b3b, gp[0], 4'd14} +: 2] : shfl_self_d[gp][15:14];
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bal_lo  (.I0(is_ballot_gp),  .O(is_ballot_h_lo[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bal_hi  (.I0(is_ballot_gp),  .O(is_ballot_h_hi[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vot_lo  (.I0(is_vote_gp),    .O(is_vote_h_lo[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vot_hi  (.I0(is_vote_gp),    .O(is_vote_h_hi[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_val_lo  (.I0(vote_val_gp),   .O(vote_val_h_lo[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_val_hi  (.I0(vote_val_gp),   .O(vote_val_h_hi[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_shf_lo  (.I0(is_shfl_gp),    .O(is_shfl_h_lo[gp]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign is_ballot_h_lo[gp] = is_ballot_gp;
			assign is_ballot_h_hi[gp] = is_ballot_gp;
			assign is_vote_h_lo[gp] = is_vote_gp;
			assign is_vote_h_hi[gp] = is_vote_gp;
			assign vote_val_h_lo[gp] = vote_val_gp;
			assign vote_val_h_hi[gp] = vote_val_gp;
			assign is_shfl_h_lo[gp] = is_shfl_gp;
`else
			assign is_ballot_h_lo[gp] = is_ballot_gp;
			assign is_ballot_h_hi[gp] = is_ballot_gp;
			assign is_vote_h_lo[gp]   = is_vote_gp;
			assign is_vote_h_hi[gp]   = is_vote_gp;
			assign vote_val_h_lo[gp]  = vote_val_gp;
			assign vote_val_h_hi[gp]  = vote_val_gp;
			assign is_shfl_h_lo[gp]   = is_shfl_gp;
`endif
		end
		wire [G-1:0] stage_grp;
		genvar gg;
		for (gg = 0; gg < G; gg = gg + 1) begin : g_stg_grp
			assign stage_grp[gg] = stage && (gg[GRP_W-1:0] == x_grp_stg);
		end
		for (gp = 0; gp < 32; gp = gp + 1) begin : g_stgw
`ifdef SYNTH_KINTEX7
			wire stg_cond = (gp / NPHYS < G) ? stage_grp[gp / NPHYS] : 1'b0;
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stgw_l (.I0(stg_cond), .O(stg_we_lo[gp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stgw_h (.I0(stg_cond), .O(stg_we_hi[gp]));
`elsif SYNTH_KINTEX_ULTRASCALE
			wire stg_cond = (gp / NPHYS < G) ? stage_grp[gp / NPHYS] : 1'b0;
			assign stg_we_lo[gp] = stg_cond;
			assign stg_we_hi[gp] = stg_cond;
`else
			localparam integer GP_G = gp / NPHYS;
			wire stg_cond = (GP_G < G) ? (sg && st_idle_rep[gp] && (GP_G[GRP_W-1:0] == x_grp_stg)) : 1'b0;
			assign stg_we_lo[gp] = stg_cond;
			assign stg_we_hi[gp] = stg_cond;
`endif
		end
	endgenerate

	wire [NPHYS*2-1:0] ib_d_lo, ib_d_hi, iv_d_lo, iv_d_hi, is_d_lo, is_d_hi;
	wire stg_last_st, stg_last_ec0, stg_last_ec1, stg_last_ec2;
	wire emit_st_c, emit_ec0, emit_ec1, emit_ec2;
	wire stg_last_rlo, stg_last_rhi, emit_rlo, emit_rhi;
`ifdef SYNTH_KINTEX7
	genvar gdl;
	generate
		for (gdl = 0; gdl < NPHYS * 2; gdl = gdl + 1) begin : g_d_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ibdl (.I0(is_ballot_d[gdl]), .O(ib_d_lo[gdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ibdh (.I0(is_ballot_d[gdl]), .O(ib_d_hi[gdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ivdl (.I0(is_vote_d[gdl]),   .O(iv_d_lo[gdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ivdh (.I0(is_vote_d[gdl]),   .O(iv_d_hi[gdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_isdl (.I0(is_shfl_d[gdl]),   .O(is_d_lo[gdl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_isdh (.I0(is_shfl_d[gdl]),   .O(is_d_hi[gdl]));
		end
	endgenerate
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slst  (.I0(stage && x_last), .O(stg_last_st));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slrlo (.I0(stage && x_last), .O(stg_last_rlo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slrhi (.I0(stage && x_last), .O(stg_last_rhi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slec0 (.I0(stage && x_last), .O(stg_last_ec0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slec1 (.I0(stage && x_last), .O(stg_last_ec1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slec2 (.I0(stage && x_last), .O(stg_last_ec2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emst  (.I0(emit), .O(emit_st_c));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emrlo (.I0(emit), .O(emit_rlo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emrhi (.I0(emit), .O(emit_rhi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emec0 (.I0(emit), .O(emit_ec0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emec1 (.I0(emit), .O(emit_ec1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_emec2 (.I0(emit), .O(emit_ec2));
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gdl;
	generate
		for (gdl = 0; gdl < NPHYS * 2; gdl = gdl + 1) begin : g_d_buf
			assign ib_d_lo[gdl] = is_ballot_d[gdl];
			assign ib_d_hi[gdl] = is_ballot_d[gdl];
			assign iv_d_lo[gdl] = is_vote_d[gdl];
			assign iv_d_hi[gdl] = is_vote_d[gdl];
			assign is_d_lo[gdl] = is_shfl_d[gdl];
			assign is_d_hi[gdl] = is_shfl_d[gdl];
		end
	endgenerate
	assign stg_last_st = stage && x_last;
	assign stg_last_rlo = stage && x_last;
	assign stg_last_rhi = stage && x_last;
	assign stg_last_ec0 = stage && x_last;
	assign stg_last_ec1 = stage && x_last;
	assign stg_last_ec2 = stage && x_last;
	assign emit_st_c = emit;
	assign emit_rlo = emit;
	assign emit_rhi = emit;
	assign emit_ec0 = emit;
	assign emit_ec1 = emit;
	assign emit_ec2 = emit;
`else
	assign ib_d_lo = is_ballot_d; assign ib_d_hi = is_ballot_d;
	assign iv_d_lo = is_vote_d;   assign iv_d_hi = is_vote_d;
	assign is_d_lo = is_shfl_d;   assign is_d_hi = is_shfl_d;
	assign stg_last_st  = stage && x_last;
	assign stg_last_rlo = stage && x_last;
	assign stg_last_rhi = stage && x_last;
	assign stg_last_ec0 = stage && x_last;
	assign stg_last_ec1 = stage && x_last;
	assign stg_last_ec2 = stage && x_last;
	assign emit_st_c = emit; assign emit_rlo = emit; assign emit_rhi = emit;
	assign emit_ec0  = emit; assign emit_ec1 = emit; assign emit_ec2 = emit;
`endif

	integer k, kp;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= S_IDLE;
			e_cnt <= {GRP_W{1'b0}};
			e_cnt_b <= {GRP_W{1'b0}};
			for (k = 0; k < SHORT_LAT; k = k + 1)
				pipe[k] <= {EW{1'b0}};
			pipe_v_s6_rep <= 8'd0;
			is_ballot_d <= {(NPHYS*2){1'b0}};
			is_vote_d   <= {(NPHYS*2){1'b0}};
			is_shfl_d   <= {(NPHYS*2){1'b0}};
			shfl_ok_d   <= {(NPHYS*2){1'b0}};
			vote_val_d  <= {(NPHYS*2){1'b0}};
			for (kp = 0; kp < NPHYS * 2; kp = kp + 1) begin
				bal_h_d[kp]     <= 16'd0;
				shfl_es_d[kp]   <= 5'd0;
				shfl_self_d[kp] <= 16'd0;
			end
		end else begin
			if (emit_ctrl)
				pipe[0][EW-1:DW] <= {wrd, x_warp, e_cnt_b, x_uop[ATG_UOP_RD +: 7], z_lanes};
			else
				pipe[0][EW-1:DW] <= {(EW-DW){1'b0}};

			pipe[0][DW-1:0] <= {DW{1'b0}};
			for (kp = 0; kp < NPHYS * 2; kp = kp + 1) begin
				is_ballot_d[kp] <= is_ballot_h_lo[kp];
				is_vote_d[kp]   <= is_vote_h_lo[kp];
				is_shfl_d[kp]   <= is_shfl_h_lo[kp];
				shfl_ok_d[kp]   <= shfl_ok_h[kp];
				bal_h_d[kp]     <= bal_h[kp];
				vote_val_d[kp]  <= vote_val_h_lo[kp];
				shfl_es_d[kp]   <= shfl_es_h[kp];
				shfl_self_d[kp] <= shfl_self_h[kp];
			end

			pipe[1][EW-1:DW] <= pipe[0][EW-1:DW];
			for (kp = 0; kp < NPHYS * 2; kp = kp + 1) begin
				pipe[1][kp*16 +: 8] <= ib_d_lo[kp] ? bal_h_d[kp][7:0] :
				                       iv_d_lo[kp] ? {8{vote_val_d[kp]}} :
				                       is_d_lo[kp] ? shfl_val_d[kp][7:0] :
				                       pipe[0][kp*16 +: 8];

				pipe[1][kp*16 + 8 +: 8] <= ib_d_hi[kp] ? bal_h_d[kp][15:8] :
				                          iv_d_hi[kp] ? {8{vote_val_d[kp]}} :
				                          is_d_hi[kp] ? shfl_val_d[kp][15:8] :
				                          pipe[0][kp*16 + 8 +: 8];
			end

			for (k = 2; k < SHORT_LAT; k = k + 1)
				pipe[k] <= pipe[k-1];
			pipe_v_s6_rep <= {8{pipe[SHORT_LAT-2][EW-1]}};

			if (stg_last_st) begin
				st <= S_EMIT;
				e_cnt <= {GRP_W{1'b0}};
			e_cnt_b <= {GRP_W{1'b0}};
			end else if (emit_st_c) begin
				if (e_last) begin
					st <= S_IDLE;
				end else begin
					e_cnt <= e_cnt_next;
					e_cnt_b <= e_cnt_next;
				end
			end
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st_idle_rep[15:0] <= 16'hFFFF;
		end else if (stg_last_rlo) begin
			st_idle_rep[15:0] <= 16'd0;
		end else if (emit_rlo && e_last) begin
			st_idle_rep[15:0] <= 16'hFFFF;
		end
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st_idle_rep[31:16] <= 16'hFFFF;
		end else if (stg_last_rhi) begin
			st_idle_rep[31:16] <= 16'd0;
		end else if (emit_rhi && e_last) begin
			st_idle_rep[31:16] <= 16'hFFFF;
		end
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			e_cnt_rep[0 +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (stg_last_ec0) begin
			e_cnt_rep[0 +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (emit_ec0 && !e_last) begin
			e_cnt_rep[0 +: (NPHYS*GRP_W)] <= {(NPHYS){e_cnt_next}};
		end
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			e_cnt_rep[(NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (stg_last_ec1) begin
			e_cnt_rep[(NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (emit_ec1 && !e_last) begin
			e_cnt_rep[(NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS){e_cnt_next}};
		end
	end
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			e_cnt_rep[(2*NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (stg_last_ec2) begin
			e_cnt_rep[(2*NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS*GRP_W){1'b0}};
		end else if (emit_ec2 && !e_last) begin
			e_cnt_rep[(2*NPHYS*GRP_W) +: (NPHYS*GRP_W)] <= {(NPHYS){e_cnt_next}};
		end
	end

	/* The staging registers (no reset: written before they are read). */
	integer m;
	always @(posedge clk) begin
		for (m = 0; m < 32; m = m + 1) begin
			if (stg_we_lo[m]) begin
				s_d[m*32 +: 16] <= x_a[(m % NPHYS)*32 +: 16];
				s_ex[m] <= x_mask[m % NPHYS];
				s_ok[m] <= c_ok[m % NPHYS];
				s_ok_hi_a[m] <= c_ok_hi_a[m % NPHYS];
				s_ok_hi_b[m] <= c_ok_hi_b[m % NPHYS];
				s_need_ok_hi[m] <= c_need_ok_hi[m % NPHYS];
			end
			if (stg_we_hi[m]) begin
				s_d[m*32 + 16 +: 16] <= x_a[(m % NPHYS)*32 + 16 +: 16];
				s_src[m*5 +: 5] <= c_src[(m % NPHYS)*5 +: 5];
			end
		end
	end

	wire [EW-1:0] o = pipe[SHORT_LAT-1];
	generate
	if (SHORT_LAT > 6) begin : g_lat7
		wire [EW-1:0] o_s6 = pipe[5];
		(* shreg_extract = "no" *) reg r6_valid;
		(* shreg_extract = "no", keep = "true" *) reg [7:0] r6_valid_rep;
		(* shreg_extract = "no" *) reg [WARP_W-1:0] r6_warp;
		(* shreg_extract = "no" *) reg [GRP_W-1:0]  r6_grp;
		(* shreg_extract = "no" *) reg [6:0]        r6_rd;
		(* shreg_extract = "no" *) reg [NPHYS-1:0]  r6_lanes;
		(* shreg_extract = "no" *) reg [DW-1:0]     r6_data;
		integer g_l7;
		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				r6_valid     <= 1'b0;
				r6_valid_rep <= 8'd0;
				r6_warp      <= {WARP_W{1'b0}};
				r6_grp       <= {GRP_W{1'b0}};
				r6_rd        <= 7'd0;
				r6_lanes     <= {NPHYS{1'b0}};
			end else begin
				r6_valid     <= r5_valid ? 1'b1 : o_s6[EW-1];
				r6_valid_rep <= {8{r5_valid ? 1'b1 : o_s6[EW-1]}};
				r6_warp      <= r5_valid ? r5_warp : o_s6[EW-2 -: WARP_W];
				r6_grp       <= r5_valid ? r5_grp  : o_s6[EW-2-WARP_W -: GRP_W];
				r6_rd        <= r5_valid ? r5_rd   : o_s6[EW-2-WARP_W-GRP_W -: 7];
				r6_lanes     <= r5_valid ? r5_mask : o_s6[DW +: NPHYS];
			end
		end
		always @(posedge clk) begin
			for (g_l7 = 0; g_l7 < NPHYS * 2; g_l7 = g_l7 + 1)
				r6_data[g_l7*16 +: 16] <= r5_use_qa[g_l7] ? qa_all[g_l7*16 +: 16] :
				                           r5_use_qb[g_l7] ? qb_all[g_l7*16 +: 16] :
				                           r5_use_xa[g_l7] ? r5_xa[g_l7*16 +: 16]  :
				                           o_s6[g_l7*16 +: 16];
		end
		if (SHORT_LAT == 7) begin : g_out7
			assign s_valid     = r6_valid;
			assign s_valid_rep = r6_valid_rep;
			assign s_warp      = r6_warp;
			assign s_grp       = r6_grp;
			assign s_rd        = r6_rd;
			assign s_lanes     = r6_lanes;
			assign s_data      = r6_data;
		end else begin : g_out_gt7
			localparam integer EXTRA7 = SHORT_LAT - 7;
			(* shreg_extract = "no" *) reg [EXTRA7-1:0]     ex_valid;
			(* shreg_extract = "no", keep = "true" *) reg [7:0] ex_valid_rep [0:EXTRA7-1];
			(* shreg_extract = "no" *) reg [WARP_W-1:0]    ex_warp  [0:EXTRA7-1];
			(* shreg_extract = "no" *) reg [GRP_W-1:0]     ex_grp   [0:EXTRA7-1];
			(* shreg_extract = "no" *) reg [6:0]           ex_rd    [0:EXTRA7-1];
			(* shreg_extract = "no" *) reg [NPHYS-1:0]     ex_lanes [0:EXTRA7-1];
			(* shreg_extract = "no" *) reg [DW-1:0]        ex_data  [0:EXTRA7-1];
			integer ke7;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					ex_valid <= {EXTRA7{1'b0}};
					for (ke7 = 0; ke7 < EXTRA7; ke7 = ke7 + 1) begin
						ex_valid_rep[ke7] <= 8'd0;
						ex_warp[ke7]      <= {WARP_W{1'b0}};
						ex_grp[ke7]       <= {GRP_W{1'b0}};
						ex_rd[ke7]        <= 7'd0;
						ex_lanes[ke7]     <= {NPHYS{1'b0}};
					end
				end else begin
					ex_valid[0]     <= r6_valid;
					ex_valid_rep[0] <= r6_valid_rep;
					ex_warp[0]      <= r6_warp;
					ex_grp[0]       <= r6_grp;
					ex_rd[0]        <= r6_rd;
					ex_lanes[0]     <= r6_lanes;
					for (ke7 = 1; ke7 < EXTRA7; ke7 = ke7 + 1) begin
						ex_valid[ke7]     <= ex_valid[ke7-1];
						ex_valid_rep[ke7] <= ex_valid_rep[ke7-1];
						ex_warp[ke7]      <= ex_warp[ke7-1];
						ex_grp[ke7]       <= ex_grp[ke7-1];
						ex_rd[ke7]        <= ex_rd[ke7-1];
						ex_lanes[ke7]     <= ex_lanes[ke7-1];
					end
				end
			end
			always @(posedge clk) begin
				ex_data[0] <= r6_data;
				for (ke7 = 1; ke7 < EXTRA7; ke7 = ke7 + 1)
					ex_data[ke7] <= ex_data[ke7-1];
			end
			assign s_valid     = ex_valid[EXTRA7-1];
			assign s_valid_rep = ex_valid_rep[EXTRA7-1];
			assign s_warp      = ex_warp[EXTRA7-1];
			assign s_grp       = ex_grp[EXTRA7-1];
			assign s_rd        = ex_rd[EXTRA7-1];
			assign s_lanes     = ex_lanes[EXTRA7-1];
			assign s_data      = ex_data[EXTRA7-1];
		end
		wire unused_o_lat7 = &{1'b0, o, r5_valid_rep, pipe_v_s6_rep};
	end else begin : g_lat6
		assign s_valid = r5_valid ? 1'b1 : o[EW-1];
		assign s_valid_rep = r5_valid_rep | pipe_v_s6_rep;
		assign s_warp  = r5_valid ? r5_warp : o[EW-2 -: WARP_W];
		assign s_grp   = r5_valid ? r5_grp : o[EW-2-WARP_W -: GRP_W];
		assign s_rd    = r5_valid ? r5_rd : o[EW-2-WARP_W-GRP_W -: 7];
		assign s_lanes = r5_valid ? r5_mask : o[DW +: NPHYS];

		genvar g_sd;
		for (g_sd = 0; g_sd < NPHYS * 2; g_sd = g_sd + 1) begin : g_s_data
			assign s_data[g_sd*16 +: 16] = r5_use_qa[g_sd] ? qa_all[g_sd*16 +: 16] :
			                               r5_use_qb[g_sd] ? qb_all[g_sd*16 +: 16] :
			                               r5_use_xa[g_sd] ? r5_xa[g_sd*16 +: 16]  :
			                               o[g_sd*16 +: 16];
		end
	end
	endgenerate

	wire unused_ok = &{1'b0, x_sbe, x_pc, x_c, x_mask[2*NPHYS-1:NPHYS], x_info,
		x_uop, sub[4:3], ex_g, pipe_fb, (NWARPS != 0)};
endmodule


`default_nettype wire
