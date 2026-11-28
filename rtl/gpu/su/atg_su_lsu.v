/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_lsu: Global memory: lw/sw/atom of NPHYS lanes per group through
 * the coalescer (lanes merged into line requests), the SU's D$
 * (atg_su_dcache) and the MC's atomic unit; fence's acquire invalidate.
 *
 * One lane group at a time (x_ready is 1 only in IDLE):
 *   lw   the EXEC lanes' byte addresses rs1 + imm (word aligned: bits 1:0
 *        are ignored) are coalesced: the line of the first pending lane is
 *        read from the D$, every pending lane in that line takes its word,
 *        until no lane is pending; then one beat (l_wr 1, EXEC lanes);
 *   sw   the side-effect lanes are coalesced the same way into line writes
 *        with byte enables (when two lanes write one word the higher lane
 *        wins); each write's response is waited for; then a beat without
 *        data (l_wr 0), so cm falls after the stores are done (design
 *        §3.1.1); sw.x (the uop's sub[0], a store to the warp's scratch
 *        memory) takes the EXEC lanes instead, helper and killed lanes
 *        included;
 *   atom one at_* operation per side-effect lane, in lane order (at_op =
 *        the uop's sub: and or xor xchg add sub smax smin umax umin
 *        cmpxchg), the D$ invalidated first (dc_inv); the old values are
 *        the beat's data when rd is written (l_wr = WRD);
 *   fence (class a, no beat) pulses dc_inv when imm[2] (acquire) is set;
 *        atg_su_sb has waited for cm = 0 before issuing it.
 * A group without lanes returns its beat at once.  dc_rsp_error and
 * at_error give 0 to the lanes concerned (the MC raised the fault).  Every
 * beat carries the group's x_last as l_last, class m and x_sbe.  Requests
 * are in order, so a warp's sw is complete before its next lw is read.
 *
 * States: IDLE →(group)→ LINE (request the next line) →(dc_req_ready)→
 * WAIT →(dc_rsp)→ LINE | BEAT; IDLE →(atom)→ AREQ →(at_ready)→ AWAIT
 * →(at_rsp)→ AREQ | BEAT; BEAT →(l_ready)→ IDLE.
*/

`timescale 1ns / 1ps
`default_nettype none

module atg_su_lsu #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5	/* log2(LINE_BYTES) */
) (
	input wire clk,
	input wire rst_n,
	/* Issued instruction, one lane group per cycle (from atg_su_opd, shared by all units) */
	input wire x_valid,
	input wire [WARP_W-1:0] x_warp,
	input wire [NWARPS*2-1:0] warp_stage,
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
	output wire [2:0] l_cls,
	/* AMB client port (design §4.1) */
	output wire dc_req_valid,
	input wire dc_req_ready,
	output wire dc_req_write,
	output wire [ADDR_W-LINE_SHIFT-1:0] dc_req_line,
	output wire [LINE_BYTES-1:0] dc_req_be,
	output wire [LINE_BYTES*8-1:0] dc_req_wdata,
	output wire [7:0] dc_req_tag,
	input wire dc_rsp_valid,
	input wire dc_rsp_write,
	input wire [7:0] dc_rsp_tag,
	input wire [LINE_BYTES*8-1:0] dc_rsp_rdata,
	input wire dc_rsp_error,
	/* Atomic port (to atg_mc_atom) */
	output wire at_valid,
	output wire [1:0] at_stage,
	input wire at_ready,
	output wire [3:0] at_op,
	output wire [29:0] at_addr,
	output wire [31:0] at_a,
	output wire [31:0] at_b,
	input wire at_rsp_valid,
	input wire [31:0] at_old,
	input wire at_error,
	output wire dc_inv
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam LB = LINE_BYTES * 8;
	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam WPL = LINE_BYTES / 4;
	localparam WW = WPL > 1 ? $clog2(WPL) : 1;
	localparam LW = NPHYS > 1 ? $clog2(NPHYS) : 1;
	localparam [3:0] IDLE = 4'd0, LINE = 4'd1, WAIT = 4'd2, AREQ = 4'd3, AWAIT = 4'd4,
		BEAT = 4'd5, LINE_HIT = 4'd6, LINE_DAT = 4'd7, PREP = 4'd8, WAIT_MUX = 4'd9, LINE_DAT2 = 4'd10,
		LINE_DAT1 = 4'd11;

	wire rst_p = !rst_n;
	wire take_lsu_raw = x_valid && (unit == ATG_UNIT_LSU) && (st == IDLE) && (op != ATG_LSU_FENCE);
	wire [3:0] take_lsu_q;
	wire prep_lsu_raw = (st == PREP);
	wire prep_lsu_h0, prep_lsu_h1;
	wire [3:0] op_ctl_b, op_ctl_c;
`ifdef SYNTH_KINTEX7
	genvar gtl;
	generate
		for (gtl = 0; gtl < 4; gtl = gtl + 1) begin : g_tk_q
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_q (.I0(take_lsu_raw), .O(take_lsu_q[gtl]));
		end
	endgenerate
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pr_lh0 (.I0(prep_lsu_raw), .O(prep_lsu_h0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pr_lh1 (.I0(prep_lsu_raw), .O(prep_lsu_h1));
	for (gtl = 0; gtl < 4; gtl = gtl + 1) begin : g_op_b
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_opb (.I0(x_uop[ATG_UOP_OP + 4 + gtl]), .O(op_ctl_b[gtl]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_opc (.I0(x_uop[ATG_UOP_OP + 4 + gtl]), .O(op_ctl_c[gtl]));
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gtl;
	generate
		for (gtl = 0; gtl < 4; gtl = gtl + 1) begin : g_tk_q
			assign take_lsu_q[gtl] = take_lsu_raw;
		end
	endgenerate
	assign prep_lsu_h0 = prep_lsu_raw;
	assign prep_lsu_h1 = prep_lsu_raw;
	for (gtl = 0; gtl < 4; gtl = gtl + 1) begin : g_op_b
		assign op_ctl_b[gtl] = x_uop[ATG_UOP_OP + 4 + gtl];
		assign op_ctl_c[gtl] = x_uop[ATG_UOP_OP + 4 + gtl];
	end
`else
	assign take_lsu_q = {4{take_lsu_raw}};
	assign prep_lsu_h0 = prep_lsu_raw;
	assign prep_lsu_h1 = prep_lsu_raw;
	assign op_ctl_b    = x_uop[ATG_UOP_OP + 4 +: 4];
	assign op_ctl_c    = x_uop[ATG_UOP_OP + 4 +: 4];
`endif
	(* max_fanout = 12 *) reg [3:0] st;
	(* keep = "true", dont_touch = "true" *) reg [NPHYS-1:0] idle_r, prep_r;
	reg at_valid_r;
	(* dont_touch = "true" *) reg [29:0] at_addr_r;
	(* dont_touch = "true" *) reg [31:0] at_a_r, at_b_r;
	wire [29:0] at_ad_w;
	wire [31:0] at_a_w, at_b_w;
	(* max_fanout = 12 *) reg [31:0] rimm;
	wire [DW-1:0] radr, rdat, rcmp, res;
	(* dont_touch = "true" *) reg [NPHYS-1:0] pend, pend_a, pend_b, pend_be;
	(* keep = "true" *) reg [NPHYS-1:0] pend_clr, pend_at_ad, pend_at_a, pend_at_b;
	(* keep = "true", dont_touch = "true" *) reg [NPHYS-1:0] pend_ll0, pend_ll1, pend_ll2, pend_ll3;
	(* dont_touch = "true" *) reg [NPHYS-1:0] lhit, lhit_a, lhit_b, lhit_be;
	(* max_fanout = 12 *) reg [NPHYS-1:0] lanes, cur;
	(* max_fanout = 12 *) reg rwr, rst_w, ratom;
	(* max_fanout = 12 *) reg [3:0] rsub;
	reg [6:0] rrd;
	reg [WARP_W-1:0] rwarp;
	reg [GRP_W-1:0] rgrp;
	reg rlast;
	reg [SBE_W:0] rsbe;
	localparam LA_LO = LA_W / 2;
	localparam LA_HI = LA_W - LA_LO;
	(* shreg_extract = "no", max_fanout = 12 *) reg [LA_LO-1:0] cline_lo;
	(* shreg_extract = "no", max_fanout = 12 *) reg [LA_HI-1:0] cline_hi;
	(* dont_touch = "true" *) reg [LA_LO-1:0] dc_line_lo_r;
	(* dont_touch = "true" *) reg [LA_HI-1:0] dc_line_hi_r;
	wire [LA_W-1:0] cline = {cline_hi, cline_lo};
	wire [LA_W-1:0] dc_line_r = {dc_line_hi_r, dc_line_lo_r};
	wire [LINE_BYTES-1:0] cbe;
	wire [LB-1:0] cwd;
	wire [LB-1:0] cbe_unused_r;
	reg [LB-1:0] dc_rdata_r;
	localparam integer NG_ERR = (NPHYS >= 4) ? (NPHYS / 4) : 1;
	reg dc_err_r;
	(* shreg_extract = "no", dont_touch = "true" *) reg [NG_ERR-1:0] dc_err_g_r;
	wire dc_rdata_we_raw = (st == WAIT) && dc_rsp_valid;
	wire [3:0] dc_rdata_we_g;
	wire [2*WPL-1:0] dc_rdata_we;
	wire dc_rsp_hit_ctl;
`ifdef SYNTH_KINTEX7
	genvar grdw, grdg;
	generate
		for (grdg = 0; grdg < 4; grdg = grdg + 1) begin : g_dcrd_g
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrw_g (.I0(dc_rdata_we_raw), .O(dc_rdata_we_g[grdg]));
		end
		for (grdw = 0; grdw < 2 * WPL; grdw = grdw + 1) begin : g_dcrd_we
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrw (.I0(dc_rdata_we_g[(grdw * 4) / (2 * WPL)]), .O(dc_rdata_we[grdw]));
		end
	endgenerate
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrhc (.I0(dc_rdata_we_raw), .O(dc_rsp_hit_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar grdw, grdg;
	generate
		for (grdg = 0; grdg < 4; grdg = grdg + 1) begin : g_dcrd_g
			assign dc_rdata_we_g[grdg] = dc_rdata_we_raw;
		end
		for (grdw = 0; grdw < 2 * WPL; grdw = grdw + 1) begin : g_dcrd_we
			assign dc_rdata_we[grdw] = dc_rdata_we_g[(grdw * 4) / (2 * WPL)];
		end
	endgenerate
	assign dc_rsp_hit_ctl = dc_rdata_we_raw;
`else
	assign dc_rdata_we_g  = {4{dc_rdata_we_raw}};
	assign dc_rdata_we    = {(2*WPL){dc_rdata_we_raw}};
	assign dc_rsp_hit_ctl = dc_rdata_we_raw;
`endif
	integer grd;
	always @(posedge clk) begin
		for (grd = 0; grd < 2 * WPL; grd = grd + 1)
			if (dc_rdata_we[grd])
				dc_rdata_r[grd*16 +: 16] <= dc_rsp_rdata[grd*16 +: 16];
	end
	(* max_fanout = 12 *) reg [LW-1:0] ali;
	reg inv_q;

	wire [3:0] unit = x_uop[ATG_UOP_UNIT +: 4];
	(* max_fanout = 12 *) wire [7:0] op = x_uop[ATG_UOP_OP +: 8];
	wire mine = x_valid && unit == ATG_UNIT_LSU;
	wire take;
	wire [3:0] st_ldat_g, st_ldat1_g, st_ldat2_g;
	wire dc_rsp_valid_pend, dc_rsp_valid_pend_0, dc_rsp_valid_pend_1, dc_rsp_valid_pend_2, dc_rsp_valid_pend_3, dc_rsp_valid_pend_4, dc_rsp_valid_pend_5;
	wire st_idle_take_0, st_idle_take_1, st_idle_take_2, st_idle_take_3, st_idle_take_4, st_idle_take_5;
	wire areq_clr_0, areq_clr_1, areq_clr_2, areq_clr_3, areq_clr_4, areq_clr_5;
	wire idle_is_atom_0, idle_is_atom_1;
`ifdef SYNTH_KINTEX7
	genvar gldg;
	generate
		for (gldg = 0; gldg < 4; gldg = gldg + 1) begin : g_st_ldat_g
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sld0 (.I0(st == LINE_DAT),  .O(st_ldat_g[gldg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sld1 (.I0(st == LINE_DAT1), .O(st_ldat1_g[gldg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sld2 (.I0(st == LINE_DAT2), .O(st_ldat2_g[gldg]));
		end
	endgenerate
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_tk_top (.I0(take_lsu_raw), .O(take));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp    (.I0(dc_rdata_we_raw), .O(dc_rsp_valid_pend));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp0   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp1   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp2   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp3   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp4   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_dcrvp5   (.I0(dc_rsp_valid_pend), .O(dc_rsp_valid_pend_5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit0    (.I0(st == IDLE && take), .O(st_idle_take_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit1    (.I0(st == IDLE && take), .O(st_idle_take_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit2    (.I0(st == IDLE && take), .O(st_idle_take_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit3    (.I0(st == IDLE && take), .O(st_idle_take_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit4    (.I0(st == IDLE && take), .O(st_idle_take_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stit5    (.I0(st == IDLE && take), .O(st_idle_take_5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc0     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc1     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc2     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc3     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc4     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aqc5     (.I0(st == AREQ && at_valid_r && at_ready), .O(areq_clr_5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iatm0    (.I0(op == ATG_LSU_ATOM), .O(idle_is_atom_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iatm1    (.I0(op == ATG_LSU_ATOM), .O(idle_is_atom_1));
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gldg;
	generate
		for (gldg = 0; gldg < 4; gldg = gldg + 1) begin : g_st_ldat_g
			assign st_ldat_g[gldg] = st == LINE_DAT;
			assign st_ldat1_g[gldg] = st == LINE_DAT1;
			assign st_ldat2_g[gldg] = st == LINE_DAT2;
		end
	endgenerate
	assign take = take_lsu_raw;
	assign dc_rsp_valid_pend = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_0 = dc_rsp_valid_pend;
	assign dc_rsp_valid_pend_1 = dc_rsp_valid_pend;
	assign dc_rsp_valid_pend_2 = dc_rsp_valid_pend;
	assign dc_rsp_valid_pend_3 = dc_rsp_valid_pend;
	assign dc_rsp_valid_pend_4 = dc_rsp_valid_pend;
	assign dc_rsp_valid_pend_5 = dc_rsp_valid_pend;
	assign st_idle_take_0 = st == IDLE && take;
	assign st_idle_take_1 = st == IDLE && take;
	assign st_idle_take_2 = st == IDLE && take;
	assign st_idle_take_3 = st == IDLE && take;
	assign st_idle_take_4 = st == IDLE && take;
	assign st_idle_take_5 = st == IDLE && take;
	assign areq_clr_0 = st == AREQ && at_valid_r && at_ready;
	assign areq_clr_1 = st == AREQ && at_valid_r && at_ready;
	assign areq_clr_2 = st == AREQ && at_valid_r && at_ready;
	assign areq_clr_3 = st == AREQ && at_valid_r && at_ready;
	assign areq_clr_4 = st == AREQ && at_valid_r && at_ready;
	assign areq_clr_5 = st == AREQ && at_valid_r && at_ready;
	assign idle_is_atom_0 = op == ATG_LSU_ATOM;
	assign idle_is_atom_1 = op == ATG_LSU_ATOM;
`else
	assign st_ldat_g           = {4{st == LINE_DAT}};
	assign st_ldat1_g          = {4{st == LINE_DAT1}};
	assign st_ldat2_g          = {4{st == LINE_DAT2}};
	assign take                = mine && st == IDLE && op != ATG_LSU_FENCE;
	assign dc_rsp_valid_pend   = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_0 = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_1 = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_2 = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_3 = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_4 = dc_rdata_we_raw;
	assign dc_rsp_valid_pend_5 = dc_rdata_we_raw;
	assign st_idle_take_0      = (st == IDLE && take);
	assign st_idle_take_1      = (st == IDLE && take);
	assign st_idle_take_2      = (st == IDLE && take);
	assign st_idle_take_3      = (st == IDLE && take);
	assign st_idle_take_4      = (st == IDLE && take);
	assign st_idle_take_5      = (st == IDLE && take);
	assign areq_clr_0          = (st == AREQ && at_valid_r && at_ready);
	assign areq_clr_1          = (st == AREQ && at_valid_r && at_ready);
	assign areq_clr_2          = (st == AREQ && at_valid_r && at_ready);
	assign areq_clr_3          = (st == AREQ && at_valid_r && at_ready);
	assign areq_clr_4          = (st == AREQ && at_valid_r && at_ready);
	assign areq_clr_5          = (st == AREQ && at_valid_r && at_ready);
	assign idle_is_atom_0      = (op == ATG_LSU_ATOM);
	assign idle_is_atom_1      = (op == ATG_LSU_ATOM);
`endif
	(* max_fanout = 12 *) wire [31:0] imm = x_uop[ATG_UOP_IMM +: 32];
	/* The lanes of the access: EXEC for lw and sw.x, the side-effect lanes otherwise. */
	wire use_exec = ({op_ctl_b, op[3:0]} == ATG_LSU_LW) || (({op_ctl_b, op[3:0]} == ATG_LSU_SW) && x_uop[ATG_UOP_SUB]);
	wire [NPHYS-1:0] amask = use_exec ? x_mask[NPHYS-1:0] : x_mask[2*NPHYS-1:NPHYS];
	assign x_ready = st == IDLE;

	/* The first pending lane and the lanes in its line. */
	wire [NPHYS-1:0] pend_rev, pend_clr_rev;
	genvar gpr;
	generate
		for (gpr = 0; gpr < NPHYS; gpr = gpr + 1) begin : g_pend_rev
			assign pend_rev[gpr]     = pend[NPHYS - 1 - gpr];
			assign pend_clr_rev[gpr] = pend_clr[NPHYS - 1 - gpr];
		end
	endgenerate
	wire [LW-1:0] pend_rev_n, pend_clr_rev_n;
	wire pend_nz, pend_clr_nz;
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_lead     (.a(pend_rev),     .n(pend_rev_n),     .nz(pend_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_lead_clr (.a(pend_clr_rev), .n(pend_clr_rev_n), .nz(pend_clr_nz));
	(* max_fanout = 12 *) wire [LW:0] lead     = pend_nz     ? {1'b0, ~pend_rev_n}     : NPHYS[LW:0];
	(* max_fanout = 12 *) wire [LW:0] lead_clr = pend_clr_nz ? {1'b0, ~pend_clr_rev_n} : NPHYS[LW:0];
	wire [NPHYS-1:0] inl = pend & lhit;
	wire [NPHYS-1:0] line_eq;
	wire [NPHYS-1:0] pend_ll0_rev, pend_ll1_rev, pend_ll2_rev, pend_ll3_rev;
	genvar gpl;
	generate
		for (gpl = 0; gpl < NPHYS; gpl = gpl + 1) begin : g_pend_ll_rev
			assign pend_ll0_rev[gpl] = pend_ll0[NPHYS - 1 - gpl];
			assign pend_ll1_rev[gpl] = pend_ll1[NPHYS - 1 - gpl];
			assign pend_ll2_rev[gpl] = pend_ll2[NPHYS - 1 - gpl];
			assign pend_ll3_rev[gpl] = pend_ll3[NPHYS - 1 - gpl];
		end
	endgenerate
	wire [LW-1:0] pend_ll0_n, pend_ll1_n, pend_ll2_n, pend_ll3_n;
	wire pend_ll0_nz, pend_ll1_nz, pend_ll2_nz, pend_ll3_nz;
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_ll0 (.a(pend_ll0_rev), .n(pend_ll0_n), .nz(pend_ll0_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_ll1 (.a(pend_ll1_rev), .n(pend_ll1_n), .nz(pend_ll1_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_ll2 (.a(pend_ll2_rev), .n(pend_ll2_n), .nz(pend_ll2_nz));
	am_msb #(.W(NPHYS), .SW(LW)) u_lsu_ll3 (.a(pend_ll3_rev), .n(pend_ll3_n), .nz(pend_ll3_nz));
	wire [LW-1:0] lead_ll0 = ~pend_ll0_n;
	wire [LW-1:0] lead_ll1 = ~pend_ll1_n;
	wire [LW-1:0] lead_ll2 = ~pend_ll2_n;
	wire [LW-1:0] lead_ll3 = ~pend_ll3_n;
	wire [NPHYS*64-1:0] radr64;
	genvar ge;
	generate
		for (ge = 0; ge < NPHYS; ge = ge + 1) begin : g_radr64
			assign radr64[ge*64 +: 64] = {32'd0, radr[ge*32 +: 32]};
		end
	endgenerate
	localparam integer LL_Q0 = LA_W / 4;
	localparam integer LL_Q1 = LA_W / 4;
	localparam integer LL_Q2 = LA_W / 4;
	localparam integer LL_Q3 = LA_W - (LL_Q0 + LL_Q1 + LL_Q2);
	wire [LA_W-1:0] lline = {
		radr64[lead_ll3*64 + LINE_SHIFT + LL_Q0 + LL_Q1 + LL_Q2 +: LL_Q3],
		radr64[lead_ll2*64 + LINE_SHIFT + LL_Q0 + LL_Q1         +: LL_Q2],
		radr64[lead_ll1*64 + LINE_SHIFT + LL_Q0                 +: LL_Q1],
		radr64[lead_ll0*64 + LINE_SHIFT                         +: LL_Q0]
	};
	wire unused_ll_nz = &{1'b0, pend_ll0_nz, pend_ll1_nz, pend_ll2_nz, pend_ll3_nz};

	/* The next atomic lane. */
	wire [LW:0] anext = lead;

	/* The lanes' addresses: registered radr + rimm in PREP (avoids opd+adder in IDLE). */
	localparam N_LEQ = (LA_W + 2) / 3;
	wire [DW-1:0] x_adr;
	wire [NPHYS*WW-1:0] rwo_wr_a_bus, rwo_wr_b_bus, rwo_wr_be_bus;

	wire at_err_l [0:NPHYS-1];
	genvar gop_l;
	generate
	for (gop_l = 0; gop_l < NPHYS; gop_l = gop_l + 1) begin : g_buf_opl
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ae (.I0(at_error), .O(at_err_l[gop_l]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign at_err_l[gop_l] = at_error;
`else
		assign at_err_l[gop_l] = at_error;
`endif
	end
	endgenerate

	genvar la, gh, gleq, gl;
	generate
		for (la = 0; la < NPHYS; la = la + 1) begin : g_adr
			wire co_unused;
			am_add #(.W(32)) u_adr (.a(radr[la*32 +: 32]), .b(rimm),
				.ci(1'b0), .s(x_adr[la*32 +: 32]), .co(co_unused));
			(* keep = "true" *) wire [N_LEQ-1:0] leq_g;
			for (gleq = 0; gleq < N_LEQ; gleq = gleq + 1) begin : g_leq
				localparam LB_B = gleq * 3;
				localparam LW_C = (LB_B + 3 <= LA_W) ? 3 : (LA_W - LB_B);
				assign leq_g[gleq] = (radr64[la*64 + LINE_SHIFT + LB_B +: LW_C] == cline[LB_B +: LW_C]);
			end
			(* keep = "true" *) wire leq_lo = &leq_g[N_LEQ/2-1:0];
			(* keep = "true" *) wire leq_hi = &leq_g[N_LEQ-1:N_LEQ/2];
			assign line_eq[la] = leq_lo && leq_hi;

			wire take_lsu_h = take_lsu_q[(la * 4) / NPHYS];
			wire prep_lsu_h = (la < NPHYS/2) ? prep_lsu_h0 : prep_lsu_h1;
			wire prep_wo, prep_adr_h, take_byte_l;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_b  (.I0(take_lsu_h), .O(take_byte_l));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pr_wo (.I0(prep_lsu_h), .O(prep_wo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pr_ad (.I0(prep_lsu_h), .O(prep_adr_h));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign take_byte_l = take_lsu_h;
			assign prep_wo = prep_lsu_h;
			assign prep_adr_h = prep_lsu_h;
`else
			assign take_byte_l = idle_r[la];
			assign prep_wo     = prep_r[la];
			assign prep_adr_h  = prep_r[la];
`endif
			(* dont_touch = "true" *) reg [WW-1:0] rwo_rd_0, rwo_rd_1, rwo_rd_2, rwo_rd_3, rwo_wr_a, rwo_wr_b, rwo_wr_be;
			always @(posedge clk) begin
				if (prep_wo) begin
					rwo_rd_0  <= x_adr[la*32 + 2 +: WW];
					rwo_rd_1  <= x_adr[la*32 + 2 +: WW];
					rwo_rd_2  <= x_adr[la*32 + 2 +: WW];
					rwo_rd_3  <= x_adr[la*32 + 2 +: WW];
					rwo_wr_a  <= x_adr[la*32 + 2 +: WW];
					rwo_wr_b  <= x_adr[la*32 + 2 +: WW];
					rwo_wr_be <= x_adr[la*32 + 2 +: WW];
				end
			end
			assign rwo_wr_a_bus[la*WW +: WW]  = rwo_wr_a;
			assign rwo_wr_b_bus[la*WW +: WW]  = rwo_wr_b;
			assign rwo_wr_be_bus[la*WW +: WW] = rwo_wr_be;

			wire at_err_la0, at_err_la1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ae0 (.I0(at_err_l[la]), .O(at_err_la0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_ae1 (.I0(at_err_l[la]), .O(at_err_la1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign at_err_la0 = at_err_l[la];
			assign at_err_la1 = at_err_l[la];
`else
			assign at_err_la0 = at_err_l[la];
			assign at_err_la1 = at_err_l[la];
`endif

			for (gh = 0; gh < 4; gh = gh + 1) begin : g_lsu_half
				wire [WW-1:0] rwo_rd_h = (gh == 0) ? rwo_rd_0 :
				                         (gh == 1) ? rwo_rd_1 :
				                         (gh == 2) ? rwo_rd_2 : rwo_rd_3;
				reg [7:0] rsel_h;
				integer rwi;
				always @(*) begin
					rsel_h = dc_rdata_r[gh*8 +: 8];
					for (rwi = 1; rwi < WPL; rwi = rwi + 1)
						if (rwo_rd_h == rwi[WW-1:0])
							rsel_h = dc_rdata_r[rwi*32 + gh*8 +: 8];
				end
				(* shreg_extract = "no" *) reg [7:0] radr_h, rdat_h, rcmp_h, res_h;
				wire dc_we_h_raw = (st == WAIT_MUX) && cur[la];
				wire dc_we_h, dc_err_h;
				wire dc_err_src = dc_err_g_r[la / 4];
				wire at_we_h = (st == AWAIT) && at_rsp_valid && (ali == la[LW-1:0]);
				wire at_err_h = (gh < 2) ? at_err_la0 : at_err_la1;
				wire take_rd_h, take_cr_h;
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_trd   (.I0(take_byte_l), .O(take_rd_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_tcr   (.I0(take_byte_l), .O(take_cr_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dcwe  (.I0(dc_we_h_raw), .O(dc_we_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dcerr (.I0(dc_err_src),  .O(dc_err_h));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign take_rd_h = take_byte_l;
				assign take_cr_h = take_byte_l;
				assign dc_we_h = dc_we_h_raw;
				assign dc_err_h = dc_err_src;
`else
				assign take_rd_h = take_byte_l;
				assign take_cr_h = take_byte_l;
				assign dc_we_h   = dc_we_h_raw;
				assign dc_err_h  = dc_err_src;
`endif
				always @(posedge clk) begin
					if (take_rd_h)
						radr_h <= x_a[la*32 + gh*8 +: 8];
					else if (prep_adr_h)
						radr_h <= x_adr[la*32 + gh*8 +: 8];
					if (take_rd_h)
						rdat_h <= x_b[la*32 + gh*8 +: 8];
					if (take_cr_h)
						rcmp_h <= x_c[la*32 + gh*8 +: 8];
					if (take_cr_h) begin
						res_h <= 8'd0;
					end else if (dc_we_h) begin
						res_h <= dc_err_h ? 8'd0 : rsel_h;
					end else if (at_we_h) begin
						res_h <= at_err_h ? 8'd0 : at_old[gh*8 +: 8];
					end
				end
				assign radr[la*32 + gh*8 +: 8] = radr_h;
				assign rdat[la*32 + gh*8 +: 8] = rdat_h;
				assign rcmp[la*32 + gh*8 +: 8] = rcmp_h;
				assign res[la*32 + gh*8 +: 8]  = res_h;
			end
		end
		for (la = 0; la < LB / 16; la = la + 1) begin : g_cwd_sl
			localparam W_IDX = la / 2;
			localparam H_IDX = la % 2;
			wire [NPHYS-1:0] hit_raw;
			for (gl = 0; gl < NPHYS; gl = gl + 1) begin : g_sl_hit
				wire [WW-1:0] wo_g = (H_IDX == 0) ? rwo_wr_a_bus[gl*WW +: WW] : rwo_wr_b_bus[gl*WW +: WW];
				wire lh_g = (H_IDX == 0) ? lhit_a[gl] : lhit_b[gl];
				wire pd_g = (H_IDX == 0) ? pend_a[gl] : pend_b[gl];
				assign hit_raw[gl] = pd_g && lh_g && (wo_g == W_IDX[WW-1:0]);
			end
			(* shreg_extract = "no", dont_touch = "true" *) reg [NPHYS-1:0] sl_hit_0_r, sl_hit_1_r;
			wire sl_hit_we_0, sl_hit_we_1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slhw0 (.I0(st_ldat_g[la / 8]), .O(sl_hit_we_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_slhw1 (.I0(st_ldat_g[la / 8]), .O(sl_hit_we_1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign sl_hit_we_0 = st_ldat_g[la / 8];
			assign sl_hit_we_1 = st_ldat_g[la / 8];
`else
			assign sl_hit_we_0 = (st == LINE_DAT);
			assign sl_hit_we_1 = (st == LINE_DAT);
`endif
			always @(posedge clk) begin
				if (sl_hit_we_0)
					sl_hit_0_r <= hit_raw;
				if (sl_hit_we_1)
					sl_hit_1_r <= hit_raw;
			end
			localparam integer NG_CWD = (NPHYS >= 4) ? (NPHYS / 4) : 1;
			wire [NG_CWD-1:0] grp_any_0, grp_any_1;
			wire [NG_CWD-1:0] grp_win_0, grp_win_1;
			wire [NG_CWD*8-1:0] grp_dat_0, grp_dat_1;
			genvar ggc;
			for (ggc = 0; ggc < NG_CWD; ggc = ggc + 1) begin : g_cwd_grp
				if (NPHYS >= 4) begin : g_4
					localparam integer B = ggc * 4;
					wire [3:0] h_vec0 = sl_hit_0_r[B +: 4];
					wire [3:0] h_vec1 = sl_hit_1_r[B +: 4];
					wire [3:0] oh_0 = {
						h_vec0[3],
						!h_vec0[3] && h_vec0[2],
						!h_vec0[3] && !h_vec0[2] && h_vec0[1],
						!h_vec0[3] && !h_vec0[2] && !h_vec0[1]
					};
					wire [3:0] oh_1 = {
						h_vec1[3],
						!h_vec1[3] && h_vec1[2],
						!h_vec1[3] && !h_vec1[2] && h_vec1[1],
						!h_vec1[3] && !h_vec1[2] && !h_vec1[1]
					};
					assign grp_any_0[ggc] = |h_vec0;
					assign grp_any_1[ggc] = |h_vec1;
					assign grp_dat_0[ggc*8 +: 8] =
						(oh_0[3] ? rdat[(B+3)*32 + H_IDX*16 +: 8] : 8'd0) |
						(oh_0[2] ? rdat[(B+2)*32 + H_IDX*16 +: 8] : 8'd0) |
						(oh_0[1] ? rdat[(B+1)*32 + H_IDX*16 +: 8] : 8'd0) |
						(oh_0[0] ? rdat[(B+0)*32 + H_IDX*16 +: 8] : 8'd0);
					assign grp_dat_1[ggc*8 +: 8] =
						(oh_1[3] ? rdat[(B+3)*32 + H_IDX*16 + 8 +: 8] : 8'd0) |
						(oh_1[2] ? rdat[(B+2)*32 + H_IDX*16 + 8 +: 8] : 8'd0) |
						(oh_1[1] ? rdat[(B+1)*32 + H_IDX*16 + 8 +: 8] : 8'd0) |
						(oh_1[0] ? rdat[(B+0)*32 + H_IDX*16 + 8 +: 8] : 8'd0);
				end else begin : g_lt4
					reg [7:0] d0, d1;
					integer il;
					always @(*) begin
						d0 = 8'd0;
						d1 = 8'd0;
						for (il = 0; il < NPHYS; il = il + 1) begin
							if (sl_hit_0_r[il]) d0 = rdat[il*32 + H_IDX*16 +: 8];
							if (sl_hit_1_r[il]) d1 = rdat[il*32 + H_IDX*16 + 8 +: 8];
						end
					end
					assign grp_any_0[0] = |sl_hit_0_r;
					assign grp_any_1[0] = |sl_hit_1_r;
					assign grp_dat_0[7:0] = d0;
					assign grp_dat_1[7:0] = d1;
				end
				if (ggc == NG_CWD - 1) begin : g_top
					assign grp_win_0[ggc] = grp_any_0[ggc];
					assign grp_win_1[ggc] = grp_any_1[ggc];
				end else begin : g_lo
					assign grp_win_0[ggc] = grp_any_0[ggc] && !(|grp_any_0[NG_CWD-1:ggc+1]);
					assign grp_win_1[ggc] = grp_any_1[ggc] && !(|grp_any_1[NG_CWD-1:ggc+1]);
				end
			end
			(* shreg_extract = "no", dont_touch = "true" *) reg [NG_CWD-1:0]   grp_win_0_r, grp_win_1_r;
			(* shreg_extract = "no", dont_touch = "true" *) reg [NG_CWD*8-1:0] grp_dat_0_r, grp_dat_1_r;
			wire cwd_dat_we_0, cwd_dat_we_1;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwe0 (.I0(st_ldat1_g[la / 8]), .O(cwd_dat_we_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cdwe1 (.I0(st_ldat1_g[la / 8]), .O(cwd_dat_we_1));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign cwd_dat_we_0 = st_ldat1_g[la / 8];
			assign cwd_dat_we_1 = st_ldat1_g[la / 8];
`else
			assign cwd_dat_we_0 = (st == LINE_DAT1);
			assign cwd_dat_we_1 = (st == LINE_DAT1);
`endif
			always @(posedge clk) begin
				if (cwd_dat_we_0) begin
					grp_win_0_r <= grp_win_0;
					grp_dat_0_r <= grp_dat_0;
				end
				if (cwd_dat_we_1) begin
					grp_win_1_r <= grp_win_1;
					grp_dat_1_r <= grp_dat_1;
				end
			end
			reg [7:0] lwd_sl_0, lwd_sl_1;
			integer sl_l;
			always @(*) begin
				lwd_sl_0 = 8'd0;
				lwd_sl_1 = 8'd0;
				for (sl_l = 0; sl_l < NG_CWD; sl_l = sl_l + 1) begin
					if (grp_win_0_r[sl_l])
						lwd_sl_0 = lwd_sl_0 | grp_dat_0_r[sl_l*8 +: 8];
					if (grp_win_1_r[sl_l])
						lwd_sl_1 = lwd_sl_1 | grp_dat_1_r[sl_l*8 +: 8];
				end
			end
			wire [15:0] lwd_sl = {lwd_sl_1, lwd_sl_0};
			(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] cwd_sl;
			wire cwd_we_sl;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_cwdwe (.I0(st_ldat2_g[la / 8]), .O(cwd_we_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign cwd_we_sl = st_ldat2_g[la / 8];
`else
			assign cwd_we_sl = (st == LINE_DAT2);
`endif
			always @(posedge clk)
				if (cwd_we_sl)
					cwd_sl <= lwd_sl;
			assign cwd[la*16 +: 16] = cwd_sl;
		end
		for (la = 0; la < LINE_BYTES / 16; la = la + 1) begin : g_cbe_sl
			wire [15:0] lbe_sl;
			for (gh = 0; gh < 4; gh = gh + 1) begin : g_cbe_w
				localparam W_IDX = la * 4 + gh;
				(* dont_touch = "true" *) wire [NPHYS-1:0] be_hit;
				for (gl = 0; gl < NPHYS; gl = gl + 1) begin : g_be_l
					assign be_hit[gl] = pend_be[gl] && lhit_be[gl] && (rwo_wr_be_bus[gl*WW +: WW] == W_IDX[WW-1:0]);
				end
				(* dont_touch = "true" *) wire w_any_hit = |be_hit;
				assign lbe_sl[gh*4 +: 4] = {4{w_any_hit}};
			end
			(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] lbe_sl_r;
			(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] cbe_sl;
			wire lbe_we_sl, cbe_we_sl, rwr_sl;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_lbewe (.I0(st_ldat_g[la & 3]),  .O(lbe_we_sl));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_cbewe (.I0(st_ldat2_g[la & 3]), .O(cbe_we_sl));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rwr   (.I0(rwr),                  .O(rwr_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign lbe_we_sl = st_ldat_g[la & 3];
			assign cbe_we_sl = st_ldat2_g[la & 3];
			assign rwr_sl = rwr;
`else
			assign lbe_we_sl = (st == LINE_DAT);
			assign cbe_we_sl = (st == LINE_DAT2);
			assign rwr_sl    = rwr;
`endif
			always @(posedge clk) begin
				if (lbe_we_sl)
					lbe_sl_r <= lbe_sl;
				if (cbe_we_sl)
					cbe_sl <= lbe_sl_r;
			end
			assign cbe[la*16 +: 16] = cbe_sl;
			assign dc_req_be[la*16 +: 16] = rwr_sl ? cbe_sl : 16'd0;
		end
	endgenerate

	wire at_cap_ce_raw = ((st == PREP) && (lanes != {NPHYS{1'b0}}) && ratom) || (st == AWAIT);
	wire at_ad_ce_lo, at_ad_ce_hi, at_a_ce_lo, at_a_ce_hi, at_b_ce_b0, at_b_ce_b1, at_b_ce_b2, at_b_ce_b3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atad_lo (.I0(at_cap_ce_raw), .O(at_ad_ce_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atad_hi (.I0(at_cap_ce_raw), .O(at_ad_ce_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ata_lo  (.I0(at_cap_ce_raw), .O(at_a_ce_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ata_hi  (.I0(at_cap_ce_raw), .O(at_a_ce_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atb_b0  (.I0(at_cap_ce_raw), .O(at_b_ce_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atb_b1  (.I0(at_cap_ce_raw), .O(at_b_ce_b1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atb_b2  (.I0(at_cap_ce_raw), .O(at_b_ce_b2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_atb_b3  (.I0(at_cap_ce_raw), .O(at_b_ce_b3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign at_ad_ce_lo = at_cap_ce_raw;
	assign at_ad_ce_hi = at_cap_ce_raw;
	assign at_a_ce_lo = at_cap_ce_raw;
	assign at_a_ce_hi = at_cap_ce_raw;
	assign at_b_ce_b0 = at_cap_ce_raw;
	assign at_b_ce_b1 = at_cap_ce_raw;
	assign at_b_ce_b2 = at_cap_ce_raw;
	assign at_b_ce_b3 = at_cap_ce_raw;
`else
	assign at_ad_ce_lo = at_cap_ce_raw;
	assign at_ad_ce_hi = at_cap_ce_raw;
	assign at_a_ce_lo  = at_cap_ce_raw;
	assign at_a_ce_hi  = at_cap_ce_raw;
	assign at_b_ce_b0  = at_cap_ce_raw;
	assign at_b_ce_b1  = at_cap_ce_raw;
	assign at_b_ce_b2  = at_cap_ce_raw;
	assign at_b_ce_b3  = at_cap_ce_raw;
`endif

	wire lsu_idle_ce_raw = !rst_p && (st == IDLE) && take;
	wire lsu_idle_ce_a, lsu_idle_ce_b, lsu_idle_ce_imm0a, lsu_idle_ce_imm0b, lsu_idle_ce_imm1a, lsu_idle_ce_imm1b;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_licea   (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_liceb   (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_licei0a (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_imm0a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_licei0b (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_imm0b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_licei1a (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_imm1a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_licei1b (.I0(lsu_idle_ce_raw), .O(lsu_idle_ce_imm1b));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign lsu_idle_ce_a = lsu_idle_ce_raw;
	assign lsu_idle_ce_b = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm0a = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm0b = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm1a = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm1b = lsu_idle_ce_raw;
`else
	assign lsu_idle_ce_a     = lsu_idle_ce_raw;
	assign lsu_idle_ce_b     = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm0a = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm0b = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm1a = lsu_idle_ce_raw;
	assign lsu_idle_ce_imm1b = lsu_idle_ce_raw;
`endif
	wire line_ce_raw = !rst_p && (st == LINE) && (lead != NPHYS[LW:0]);
	wire line_ce_0, line_ce_1, line_ce_2, line_ce_3, line_ce_0a, line_ce_3a;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce0  (.O(line_ce_0),  .I0(line_ce_raw));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce0a (.O(line_ce_0a), .I0(line_ce_raw));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce1  (.O(line_ce_1),  .I0(line_ce_raw));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce2  (.O(line_ce_2),  .I0(line_ce_raw));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce3  (.O(line_ce_3),  .I0(line_ce_raw));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lce3a (.O(line_ce_3a), .I0(line_ce_raw));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign line_ce_0 = line_ce_raw;
	assign line_ce_0a = line_ce_raw;
	assign line_ce_1 = line_ce_raw;
	assign line_ce_2 = line_ce_raw;
	assign line_ce_3 = line_ce_raw;
	assign line_ce_3a = line_ce_raw;
`else
	assign line_ce_0  = line_ce_raw;
	assign line_ce_0a = line_ce_raw;
	assign line_ce_1  = line_ce_raw;
	assign line_ce_2  = line_ce_raw;
	assign line_ce_3  = line_ce_raw;
	assign line_ce_3a = line_ce_raw;
`endif
	wire line_hit_ce = !rst_p && (st == LINE_HIT);
	wire line_hit_ce_0, line_hit_ce_1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lhit0 (.O(line_hit_ce_0), .I0(line_hit_ce));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_buf_lhit1 (.O(line_hit_ce_1), .I0(line_hit_ce));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign line_hit_ce_0 = line_hit_ce;
	assign line_hit_ce_1 = line_hit_ce;
`else
	assign line_hit_ce_0 = line_hit_ce;
	assign line_hit_ce_1 = line_hit_ce;
`endif

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend    <= {NPHYS{1'b0}};
			pend_a  <= {NPHYS{1'b0}};
		end else if (st_idle_take_0) begin
			pend    <= amask;
			pend_a  <= amask;
		end else if (dc_rsp_valid_pend_0) begin
			pend    <= pend & ~cur;
			pend_a  <= pend_a & ~cur;
		end else if (areq_clr_0) begin
			pend[lead[LW-1:0]]    <= 1'b0;
			pend_a[lead[LW-1:0]]  <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend_b  <= {NPHYS{1'b0}};
			pend_be <= {NPHYS{1'b0}};
		end else if (st_idle_take_4) begin
			pend_b  <= amask;
			pend_be <= amask;
		end else if (dc_rsp_valid_pend_4) begin
			pend_b  <= pend_b & ~cur;
			pend_be <= pend_be & ~cur;
		end else if (areq_clr_4) begin
			pend_b[lead[LW-1:0]]  <= 1'b0;
			pend_be[lead[LW-1:0]] <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend_clr   <= {NPHYS{1'b0}};
			pend_at_ad <= {NPHYS{1'b0}};
		end else if (st_idle_take_1) begin
			pend_clr   <= amask;
			pend_at_ad <= amask;
		end else if (dc_rsp_valid_pend_1) begin
			pend_clr   <= pend_clr & ~cur;
			pend_at_ad <= pend_at_ad & ~cur;
		end else if (areq_clr_1) begin
			pend_clr[lead_clr[LW-1:0]]   <= 1'b0;
			pend_at_ad[lead_clr[LW-1:0]] <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend_at_a  <= {NPHYS{1'b0}};
			pend_at_b  <= {NPHYS{1'b0}};
		end else if (st_idle_take_5) begin
			pend_at_a  <= amask;
			pend_at_b  <= amask;
		end else if (dc_rsp_valid_pend_5) begin
			pend_at_a  <= pend_at_a & ~cur;
			pend_at_b  <= pend_at_b & ~cur;
		end else if (areq_clr_5) begin
			pend_at_a[lead_clr[LW-1:0]]  <= 1'b0;
			pend_at_b[lead_clr[LW-1:0]]  <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend_ll0 <= {NPHYS{1'b0}};
			pend_ll1 <= {NPHYS{1'b0}};
		end else if (st_idle_take_2) begin
			pend_ll0 <= amask;
			pend_ll1 <= amask;
		end else if (dc_rsp_valid_pend_2) begin
			pend_ll0 <= pend_ll0 & ~cur;
			pend_ll1 <= pend_ll1 & ~cur;
		end else if (areq_clr_2) begin
			pend_ll0[lead_clr[LW-1:0]] <= 1'b0;
			pend_ll1[lead_clr[LW-1:0]] <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			pend_ll2 <= {NPHYS{1'b0}};
			pend_ll3 <= {NPHYS{1'b0}};
		end else if (st_idle_take_3) begin
			pend_ll2 <= amask;
			pend_ll3 <= amask;
		end else if (dc_rsp_valid_pend_3) begin
			pend_ll2 <= pend_ll2 & ~cur;
			pend_ll3 <= pend_ll3 & ~cur;
		end else if (areq_clr_3) begin
			pend_ll2[lead_clr[LW-1:0]] <= 1'b0;
			pend_ll3[lead_clr[LW-1:0]] <= 1'b0;
		end
	end

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= IDLE;
			idle_r <= {NPHYS{1'b1}};
			prep_r <= {NPHYS{1'b0}};
			at_valid_r <= 1'b0;
			at_addr_r <= 30'd0;
			at_a_r <= 32'd0;
			at_b_r <= 32'd0;
			rimm <= 32'd0;
			inv_q <= 1'b0;
			ali <= {LW{1'b0}};
			rwr <= 1'b0;
			ratom <= 1'b0;
			rst_w <= 1'b0;
			rsub <= 4'd0;
			lanes <= {NPHYS{1'b0}};
			rrd <= 7'd0;
			rwarp <= {WARP_W{1'b0}};
			rgrp <= {GRP_W{1'b0}};
			rlast <= 1'b0;
			rsbe <= {(SBE_W+1){1'b0}};
			cline_lo <= {LA_LO{1'b0}};
			cline_hi <= {LA_HI{1'b0}};
			dc_line_lo_r <= {LA_LO{1'b0}};
			dc_line_hi_r <= {LA_HI{1'b0}};
			lhit <= {NPHYS{1'b0}};
			lhit_a <= {NPHYS{1'b0}};
			lhit_b <= {NPHYS{1'b0}};
			lhit_be <= {NPHYS{1'b0}};
			cur <= {NPHYS{1'b0}};
			dc_err_r <= 1'b0;
			dc_err_g_r <= {NG_ERR{1'b0}};
		end else begin
			if (lsu_idle_ce_a) begin
				rwr <= op == ATG_LSU_SW;
				ratom <= op == ATG_LSU_ATOM;
				rst_w <= op == ATG_LSU_LW || (op == ATG_LSU_ATOM && x_uop[ATG_UOP_WRD]);
				rsub <= x_uop[ATG_UOP_SUB +: 4];
				lanes <= amask;
			end
			if (lsu_idle_ce_b) begin
				rrd <= x_uop[ATG_UOP_RD +: 7];
				rwarp <= x_warp;
				rgrp <= x_grp;
				rlast <= x_last;
				rsbe <= x_sbe;
			end
			if (lsu_idle_ce_imm0a)
				rimm[7:0] <= idle_is_atom_0 ? 8'd0 : imm[7:0];
			if (lsu_idle_ce_imm0b)
				rimm[15:8] <= idle_is_atom_0 ? 8'd0 : imm[15:8];
			if (lsu_idle_ce_imm1a)
				rimm[23:16] <= idle_is_atom_1 ? 8'd0 : imm[23:16];
			if (lsu_idle_ce_imm1b)
				rimm[31:24] <= idle_is_atom_1 ? 8'd0 : imm[31:24];
			if (line_ce_0) cline_lo     <= lline[LA_LO-1:0];
			if (line_ce_1) cline_hi     <= lline[LA_W-1:LA_LO];
			if (line_ce_2) dc_line_lo_r <= lline[LA_LO-1:0];
			if (line_ce_3) dc_line_hi_r <= lline[LA_W-1:LA_LO];
			if (line_hit_ce_0) begin
				lhit   <= line_eq;
				lhit_a <= line_eq;
			end
			if (line_hit_ce_1) begin
				lhit_b  <= line_eq;
				lhit_be <= line_eq;
			end

			inv_q <= (mine && st == IDLE && op == ATG_LSU_FENCE && imm[2]) ||
			         (take && (amask != {NPHYS{1'b0}}) && (op == ATG_LSU_ATOM));
			if (at_ad_ce_lo) at_addr_r[14:0]  <= at_ad_w[14:0];
			if (at_ad_ce_hi) at_addr_r[29:15] <= at_ad_w[29:15];
			if (at_a_ce_lo)  at_a_r[15:0]     <= at_a_w[15:0];
			if (at_a_ce_hi)  at_a_r[31:16]    <= at_a_w[31:16];
			if (at_b_ce_b0)  at_b_r[7:0]      <= at_b_w[7:0];
			if (at_b_ce_b1)  at_b_r[15:8]     <= at_b_w[15:8];
			if (at_b_ce_b2)  at_b_r[23:16]    <= at_b_w[23:16];
			if (at_b_ce_b3)  at_b_r[31:24]    <= at_b_w[31:24];
			case (st)
			IDLE: if (take) begin
				idle_r <= {NPHYS{1'b0}};
				prep_r <= {NPHYS{1'b1}};
				st <= PREP;
			end
			PREP: begin
				prep_r <= {NPHYS{1'b0}};
				if (lanes == {NPHYS{1'b0}})
					st <= BEAT;
				else if (ratom) begin
					st         <= AREQ;
					at_valid_r <= pend_nz;
				end else
					st <= LINE;
			end
			LINE: begin
				if (lead == NPHYS[LW:0])
					st <= BEAT;
				else
					st <= LINE_HIT;
			end
			LINE_HIT: begin
				st <= LINE_DAT;
			end
			LINE_DAT: begin
				st <= LINE_DAT1;
			end
			LINE_DAT1: begin
				st <= LINE_DAT2;
			end
			LINE_DAT2: begin
				cur <= inl;
				st <= WAIT;
			end
			WAIT: if (dc_rsp_hit_ctl) begin
				dc_err_r   <= dc_rsp_error;
				dc_err_g_r <= {NG_ERR{dc_rsp_error}};
				st <= WAIT_MUX;
			end
			WAIT_MUX: begin
				st <= LINE;
			end
			AREQ: if (!at_valid_r) begin
				st <= BEAT;
			end else begin
				ali <= lead[LW-1:0];
				if (at_ready) begin
					st         <= AWAIT;
					at_valid_r <= 1'b0;
				end
			end
			AWAIT: begin
				if (at_rsp_valid) begin
					st         <= AREQ;
					at_valid_r <= pend_nz;
				end
			end
			default: if (l_ready) begin
				st     <= IDLE;
				idle_r <= {NPHYS{1'b1}};
			end
			endcase
		end
	end

	/* The D$ request is valid in WAIT until taken (cline holds). */
	reg req_taken;
	always @(posedge clk or posedge rst_p)
		if (rst_p)
			req_taken <= 1'b0;
		else if (st != WAIT)
			req_taken <= 1'b0;
		else if (dc_req_valid && dc_req_ready)
			req_taken <= 1'b1;
	wire [1:0] cur_stage = warp_stage[rwarp * 2 +: 2];
	assign dc_req_valid = st == WAIT && !req_taken;
	assign dc_req_write = rwr;
	assign dc_req_line = dc_line_r;
	assign dc_req_wdata = cwd;
	assign dc_req_tag = {6'd0, cur_stage};
	assign at_stage = cur_stage;
	assign dc_inv = inv_q;

	/* 4-lane group tree for atomic request address and operands (FO <= 32, depth ~11 cmos3) */
	localparam integer NG_AT = (NPHYS >= 4) ? (NPHYS / 4) : 1;
	wire [NG_AT-1:0] at_any_ad, at_any_a, at_any_b;
	wire [NG_AT-1:0] at_win_ad, at_win_a, at_win_b;
	wire [NG_AT*30-1:0] at_gdat_ad;
	wire [NG_AT*32-1:0] at_gdat_a, at_gdat_b;
	genvar gat;
	generate
		for (gat = 0; gat < NG_AT; gat = gat + 1) begin : g_at_grp
			localparam integer B = gat * 4;
			wire [3:0] pad_0, pad_1, pad_2, pad_3, pa_0, pa_1, pa_2, pa_3, pb_0, pb_1, pb_2, pb_3;
			genvar gpb;
			for (gpb = 0; gpb < 4; gpb = gpb + 1) begin : g_pbuf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pad0 (.I0(pend_at_ad[B + gpb]), .O(pad_0[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pad1 (.I0(pend_at_ad[B + gpb]), .O(pad_1[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pad2 (.I0(pend_at_ad[B + gpb]), .O(pad_2[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pad3 (.I0(pend_at_ad[B + gpb]), .O(pad_3[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pa0  (.I0(pend_at_a[B + gpb]),  .O(pa_0[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pa1  (.I0(pend_at_a[B + gpb]),  .O(pa_1[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pa2  (.I0(pend_at_a[B + gpb]),  .O(pa_2[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pa3  (.I0(pend_at_a[B + gpb]),  .O(pa_3[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pb0  (.I0(pend_at_b[B + gpb]),  .O(pb_0[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pb1  (.I0(pend_at_b[B + gpb]),  .O(pb_1[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pb2  (.I0(pend_at_b[B + gpb]),  .O(pb_2[gpb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pb3  (.I0(pend_at_b[B + gpb]),  .O(pb_3[gpb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign pad_0[gpb] = pend_at_ad[B + gpb];
				assign pad_1[gpb] = pend_at_ad[B + gpb];
				assign pad_2[gpb] = pend_at_ad[B + gpb];
				assign pad_3[gpb] = pend_at_ad[B + gpb];
				assign pa_0[gpb] = pend_at_a[B + gpb];
				assign pa_1[gpb] = pend_at_a[B + gpb];
				assign pa_2[gpb] = pend_at_a[B + gpb];
				assign pa_3[gpb] = pend_at_a[B + gpb];
				assign pb_0[gpb] = pend_at_b[B + gpb];
				assign pb_1[gpb] = pend_at_b[B + gpb];
				assign pb_2[gpb] = pend_at_b[B + gpb];
				assign pb_3[gpb] = pend_at_b[B + gpb];
`else
				assign pad_0[gpb] = pend_at_ad[B + gpb]; assign pad_1[gpb] = pend_at_ad[B + gpb];
				assign pad_2[gpb] = pend_at_ad[B + gpb]; assign pad_3[gpb] = pend_at_ad[B + gpb];
				assign pa_0[gpb]  = pend_at_a[B + gpb];  assign pa_1[gpb]  = pend_at_a[B + gpb];
				assign pa_2[gpb]  = pend_at_a[B + gpb];  assign pa_3[gpb]  = pend_at_a[B + gpb];
				assign pb_0[gpb]  = pend_at_b[B + gpb];  assign pb_1[gpb]  = pend_at_b[B + gpb];
				assign pb_2[gpb]  = pend_at_b[B + gpb];  assign pb_3[gpb]  = pend_at_b[B + gpb];
`endif
			end
			assign at_any_ad[gat] = |pad_0;
			assign at_any_a[gat]  = |pa_0;
			assign at_any_b[gat]  = |pb_0;
			assign at_gdat_ad[gat*30 +: 8] =
				pad_0[0] ? radr[(B+0)*32 + 2 +: 8] : pad_0[1] ? radr[(B+1)*32 + 2 +: 8] :
				pad_0[2] ? radr[(B+2)*32 + 2 +: 8] : pad_0[3] ? radr[(B+3)*32 + 2 +: 8] : 8'd0;
			assign at_gdat_ad[gat*30 + 8 +: 7] =
				pad_1[0] ? radr[(B+0)*32 + 10 +: 7] : pad_1[1] ? radr[(B+1)*32 + 10 +: 7] :
				pad_1[2] ? radr[(B+2)*32 + 10 +: 7] : pad_1[3] ? radr[(B+3)*32 + 10 +: 7] : 7'd0;
			assign at_gdat_ad[gat*30 + 15 +: 8] =
				pad_2[0] ? radr[(B+0)*32 + 17 +: 8] : pad_2[1] ? radr[(B+1)*32 + 17 +: 8] :
				pad_2[2] ? radr[(B+2)*32 + 17 +: 8] : pad_2[3] ? radr[(B+3)*32 + 17 +: 8] : 8'd0;
			assign at_gdat_ad[gat*30 + 23 +: 7] =
				pad_3[0] ? radr[(B+0)*32 + 25 +: 7] : pad_3[1] ? radr[(B+1)*32 + 25 +: 7] :
				pad_3[2] ? radr[(B+2)*32 + 25 +: 7] : pad_3[3] ? radr[(B+3)*32 + 25 +: 7] : 7'd0;
			assign at_gdat_a[gat*32 +: 8] =
				pa_0[0] ? rdat[(B+0)*32 +: 8] : pa_0[1] ? rdat[(B+1)*32 +: 8] :
				pa_0[2] ? rdat[(B+2)*32 +: 8] : pa_0[3] ? rdat[(B+3)*32 +: 8] : 8'd0;
			assign at_gdat_a[gat*32 + 8 +: 8] =
				pa_1[0] ? rdat[(B+0)*32 + 8 +: 8] : pa_1[1] ? rdat[(B+1)*32 + 8 +: 8] :
				pa_1[2] ? rdat[(B+2)*32 + 8 +: 8] : pa_1[3] ? rdat[(B+3)*32 + 8 +: 8] : 8'd0;
			assign at_gdat_a[gat*32 + 16 +: 8] =
				pa_2[0] ? rdat[(B+0)*32 + 16 +: 8] : pa_2[1] ? rdat[(B+1)*32 + 16 +: 8] :
				pa_2[2] ? rdat[(B+2)*32 + 16 +: 8] : pa_2[3] ? rdat[(B+3)*32 + 16 +: 8] : 8'd0;
			assign at_gdat_a[gat*32 + 24 +: 8] =
				pa_3[0] ? rdat[(B+0)*32 + 24 +: 8] : pa_3[1] ? rdat[(B+1)*32 + 24 +: 8] :
				pa_3[2] ? rdat[(B+2)*32 + 24 +: 8] : pa_3[3] ? rdat[(B+3)*32 + 24 +: 8] : 8'd0;
			assign at_gdat_b[gat*32 +: 8] =
				pb_0[0] ? rcmp[(B+0)*32 +: 8] : pb_0[1] ? rcmp[(B+1)*32 +: 8] :
				pb_0[2] ? rcmp[(B+2)*32 +: 8] : pb_0[3] ? rcmp[(B+3)*32 +: 8] : 8'd0;
			assign at_gdat_b[gat*32 + 8 +: 8] =
				pb_1[0] ? rcmp[(B+0)*32 + 8 +: 8] : pb_1[1] ? rcmp[(B+1)*32 + 8 +: 8] :
				pb_1[2] ? rcmp[(B+2)*32 + 8 +: 8] : pb_1[3] ? rcmp[(B+3)*32 + 8 +: 8] : 8'd0;
			assign at_gdat_b[gat*32 + 16 +: 8] =
				pb_2[0] ? rcmp[(B+0)*32 + 16 +: 8] : pb_2[1] ? rcmp[(B+1)*32 + 16 +: 8] :
				pb_2[2] ? rcmp[(B+2)*32 + 16 +: 8] : pb_2[3] ? rcmp[(B+3)*32 + 16 +: 8] : 8'd0;
			assign at_gdat_b[gat*32 + 24 +: 8] =
				pb_3[0] ? rcmp[(B+0)*32 + 24 +: 8] : pb_3[1] ? rcmp[(B+1)*32 + 24 +: 8] :
				pb_3[2] ? rcmp[(B+2)*32 + 24 +: 8] : pb_3[3] ? rcmp[(B+3)*32 + 24 +: 8] : 8'd0;
			if (gat == 0) begin : g_at_first
				assign at_win_ad[gat] = at_any_ad[0];
				assign at_win_a[gat]  = at_any_a[0];
				assign at_win_b[gat]  = at_any_b[0];
			end else begin : g_at_rest
				assign at_win_ad[gat] = at_any_ad[gat] && !(|at_any_ad[gat-1:0]);
				assign at_win_a[gat]  = at_any_a[gat]  && !(|at_any_a[gat-1:0]);
				assign at_win_b[gat]  = at_any_b[gat]  && !(|at_any_b[gat-1:0]);
			end
		end
	endgenerate
	wire [NG_AT-1:0] at_win_a_lo, at_win_a_hi, at_win_b_lo, at_win_b_hi;
`ifdef SYNTH_KINTEX7
	genvar gatw;
	generate
		for (gatw = 0; gatw < NG_AT; gatw = gatw + 1) begin : g_at_win_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awal (.I0(at_win_a[gatw]), .O(at_win_a_lo[gatw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awah (.I0(at_win_a[gatw]), .O(at_win_a_hi[gatw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awbl (.I0(at_win_b[gatw]), .O(at_win_b_lo[gatw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awbh (.I0(at_win_b[gatw]), .O(at_win_b_hi[gatw]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gatw;
	generate
		for (gatw = 0; gatw < NG_AT; gatw = gatw + 1) begin : g_at_win_buf
			assign at_win_a_lo[gatw] = at_win_a[gatw];
			assign at_win_a_hi[gatw] = at_win_a[gatw];
			assign at_win_b_lo[gatw] = at_win_b[gatw];
			assign at_win_b_hi[gatw] = at_win_b[gatw];
		end
	endgenerate
`else
	assign at_win_a_lo = at_win_a;
	assign at_win_a_hi = at_win_a;
	assign at_win_b_lo = at_win_b;
	assign at_win_b_hi = at_win_b;
`endif
	reg [29:0] at_ad_comb;
	reg [31:0] at_a_comb, at_b_comb;
	integer iat;
	always @(*) begin
		at_ad_comb = 30'd0;
		at_a_comb  = 32'd0;
		at_b_comb  = 32'd0;
		for (iat = 0; iat < NG_AT; iat = iat + 1) begin
			if (at_win_ad[iat])   at_ad_comb        = at_ad_comb        | at_gdat_ad[iat*30 +: 30];
			if (at_win_a_lo[iat]) at_a_comb[15:0]   = at_a_comb[15:0]   | at_gdat_a[iat*32 +: 16];
			if (at_win_a_hi[iat]) at_a_comb[31:16]  = at_a_comb[31:16]  | at_gdat_a[iat*32 + 16 +: 16];
			if (at_win_b_lo[iat]) at_b_comb[15:0]   = at_b_comb[15:0]   | at_gdat_b[iat*32 +: 16];
			if (at_win_b_hi[iat]) at_b_comb[31:16]  = at_b_comb[31:16]  | at_gdat_b[iat*32 + 16 +: 16];
		end
	end
	assign at_ad_w = at_ad_comb;
	assign at_a_w  = at_a_comb;
	assign at_b_w  = at_b_comb;
	assign at_valid = at_valid_r;
	assign at_op = rsub;
	assign at_addr = at_addr_r;
	assign at_a = at_a_r;
	assign at_b = at_b_r;

	assign l_valid = st == BEAT;
	assign l_warp = rwarp;
	assign l_grp = rgrp;
	assign l_rd = rrd;
	assign l_data = res;
	assign l_lanes = lanes;
	assign l_wr = rst_w;
	assign l_last = rlast;
	assign l_sbe = rsbe;
	assign l_cls = ATG_CLS_M;

	wire unused_ok = &{1'b0, x_pc, x_info, x_uop, imm, dc_rsp_write, dc_rsp_tag, ratom,
		unused_ll_nz, lead_clr[LW], (NWARPS != 0)};
endmodule

`default_nettype wire
