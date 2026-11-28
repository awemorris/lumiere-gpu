/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_opd: operand collection.  Takes an issued instruction from
 * atg_su_sb and sends its 32/NPHYS lane groups on the execution bus x_*,
 * one per cycle while the uop's unit is ready.
 *
 * Two stages per group: A addresses the RF (and the constant bank, rq),
 * B selects each operand (GPR 0-62, rq 63, c0-c31 64-95, the specials
 * 96-127 of design §3.2), applies the n/a modifiers for the float units
 * (FPU, XU, PK; PK on both halves) and drives x_*.  i_valid -> first
 * x_valid is 2 cycles; i_ready while stage A is free or ends this cycle.
 * unit_ready 0 holds x_* (the RF output holds while r_en is 0).  The masks
 * come from q_exec/q_killed/q_info of the warp in stage B.  lidx is lidn
 * and lidy/lidz/prim are 0 in v0 (q_info carries no WG_SIZE; 1-D groups).
 * A uop that reads a pair and rs3 (a texture .c or .3 form: rs1, rs1+1, rs2
 * and rs3 = rs1+2, the .3 form with Dref also rs1+3) needs four or five
 * operands: each of its groups goes out twice, the second pass with rs3 as
 * operand 1 (x_a), rs3+1 as operand 3 (x_c) and TEX op bit 7 set on x_uop;
 * x_last is on the second pass of the last group.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_opd #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter WARP_BASE = 0,	/* The warp operand of slot 0: SU_INDEX x NWARPS, so warp numbers the GPU's resident warps */
	parameter REG_IO = 0	/* 1: RF has registered output q1_pipe/q2_pipe (2-cycle read latency) */
) (
	input wire clk,
	input wire rst_n,
	/* From atg_su_sb */
	input wire i_valid,
	output wire i_ready,
	input wire [WARP_W-1:0] i_warp,
	input wire [15:0] i_pc,
	input wire [UOP_W-1:0] i_uop,
	input wire [SBE_W:0] i_sbe,
	/* Register file (atg_su_rf read port: dual read port) */
	output wire r_en,
	output wire [7:0] r_en_rep,
	output wire [WARP_W-1:0] r_warp,
	output wire [GRP_W-1:0] r_grp,
	output wire [11:0] r_regs,
	input wire [2*NPHYS*32-1:0] r_data,
	/* Constant bank (atg_su_cbank) */
	output wire cr_en,
	output wire [14:0] cr_idx,
	input wire [95:0] cr_data,
	/* Warp state */
	output wire [WARP_W-1:0] a_q_warp,
	output wire [WARP_W*4-1:0] a_q_warp_rep,
	output wire [WARP_W-1:0] q_warp,
	input wire [159:0] q_info,
	input wire [31:0] q_exec,
	input wire [31:0] q_killed,
	/* rq (atg_su_rq) */
	output wire rq_pop,
	input wire [NPHYS*32-1:0] rq_data,
	/* Execution bus (to every unit) */
	output wire x_valid,
	output wire [WARP_W-1:0] x_warp,
	output wire [GRP_W-1:0] x_grp,
	output wire x_last,
	output wire [SBE_W:0] x_sbe,
	output wire [15:0] x_pc,
	output wire [UOP_W-1:0] x_uop,
	output wire [UOP_W-1:0] x_uop_pt,
	output wire [2*UOP_W-1:0] x_uop_rep,
	output wire [NPHYS*32-1:0] x_a,
	output wire [NPHYS*32-1:0] x_b,
	output wire [NPHYS*32-1:0] x_c,
	output wire [2*NPHYS-1:0] x_mask,
	output wire [31:0] x_info,
	input wire [15:0] unit_ready,

	/* Forwarding buses (from ALU, FPU execution pipelines and WB) */
	input wire fwd_alu_ex3_valid,
	input wire [WARP_W-1:0] fwd_alu_ex3_warp,
	input wire [GRP_W-1:0] fwd_alu_ex3_grp,
	input wire [6:0] fwd_alu_ex3_rd,

	input wire fwd_alu_ex4_valid,
	input wire [WARP_W-1:0] fwd_alu_ex4_warp,
	input wire [GRP_W-1:0] fwd_alu_ex4_grp,
	input wire [6:0] fwd_alu_ex4_rd,
	input wire [NPHYS*32-1:0] fwd_alu_ex4_data,
	input wire [NPHYS-1:0] fwd_alu_ex4_lanes,

	input wire fwd_alu_ex5_valid,
	input wire [WARP_W-1:0] fwd_alu_ex5_warp,
	input wire [GRP_W-1:0] fwd_alu_ex5_grp,
	input wire [6:0] fwd_alu_ex5_rd,
	input wire [NPHYS*32-1:0] fwd_alu_ex5_data,
	input wire [NPHYS-1:0] fwd_alu_ex5_lanes,

	input wire fwd_fpu_ex4_valid,
	input wire [WARP_W-1:0] fwd_fpu_ex4_warp,
	input wire [GRP_W-1:0] fwd_fpu_ex4_grp,
	input wire [6:0] fwd_fpu_ex4_rd,
	input wire [NPHYS*32-1:0] fwd_fpu_ex4_data,
	input wire [NPHYS-1:0] fwd_fpu_ex4_lanes,

	input wire fwd_fpu_ex5_valid,
	input wire [WARP_W-1:0] fwd_fpu_ex5_warp,
	input wire [GRP_W-1:0] fwd_fpu_ex5_grp,
	input wire [6:0] fwd_fpu_ex5_rd,
	input wire [NPHYS*32-1:0] fwd_fpu_ex5_data,
	input wire [NPHYS-1:0] fwd_fpu_ex5_lanes,

	input wire fwd_wb_valid,
	input wire [WARP_W-1:0] fwd_wb_warp,
	input wire [GRP_W-1:0] fwd_wb_grp,
	input wire [6:0] fwd_wb_rd,
	input wire [NPHYS*32-1:0] fwd_wb_data,
	input wire [NPHYS-1:0] fwd_wb_lanes,
	output wire x_end_out
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam G = 32 / NPHYS;

	wire rst_p = !rst_n;

	/* Stage A: the instruction whose next group addresses the RF. */
	reg a_act;
	reg [WARP_W-1:0] a_warp;
	reg [15:0] a_pc;
	reg [UOP_W-1:0] a_uop;
	reg [SBE_W:0] a_sbe;
	reg [GRP_W-1:0] a_grp;
	(* dont_touch = "true" *) reg [G-1:0] a_grp_oh_r;
	wire [G-1:0] a_grp_oh_next;
	generate
		if (G > 1) begin : g_agoh_n
			assign a_grp_oh_next = {a_grp_oh_r[G-2:0], a_grp_oh_r[G-1]};
		end else begin : g_agoh_1
			assign a_grp_oh_next = 1'b1;
		end
	endgenerate
	reg a_step;	/* 0 = step 0 (reads rs1, rs2), 1 = step 1 (reads rs3) */
	(* dont_touch = "true" *) reg a_pass_ctl;	/* control/FSM pass */
	(* dont_touch = "true" *) reg a_pass_r1_lo, a_pass_r1_hi;	/* operand r1 mux pass */
	wire a_pass_r1 = a_pass_r1_lo;
	(* dont_touch = "true" *) reg a_pass_out;	/* output uop/rq_pop pass */
	reg a_wait;	/* 1 = waiting for registered RF output (when REG_IO == 1) */
	reg a_wait_last;
	(* keep = "true", dont_touch = "true" *) reg [15:0] a_act_pre_r, a_wait_pre_r;
	(* keep = "true", dont_touch = "true" *) reg [7:0] a_act_ren_r, a_wait_ren_r;
	(* keep = "true", dont_touch = "true" *) reg [WARP_W*4-1:0] a_q_warp_rep_r;
	(* dont_touch = "true" *) reg [WARP_W-1:0] a_rf_warp_r;
	(* dont_touch = "true" *) reg [GRP_W-1:0] a_rf_grp_r;
	(* dont_touch = "true" *) reg [5:0] a_rf_r1_r;
	(* dont_touch = "true" *) reg [5:0] a_rf_r2_r;
	reg [4:0] a_cr_r3_r;
	reg [6:0] a_rs1_p1_r, a_rs3_p1_r;
	/* Stage B: the group on x_*. */
	reg b_valid, b_last;
	reg [3:0] b_unit_r;
	reg b_sg_last_r;
	(* dont_touch = "true" *) reg [13:1] b_rdy_oh_r;
	(* dont_touch = "true" *) reg b_valid_last_r;
	reg [WARP_W-1:0] b_warp;
	reg [15:0] b_pc;
	reg [UOP_W-1:0] b_uop;
	(* keep = "true", equivalent_register_removal = "no" *) reg [UOP_W-1:0] b_uop_pt, b_uop_fp, b_uop_ln;
	reg [SBE_W:0] b_sbe;
	reg [GRP_W-1:0] b_grp;
	reg b_pass;
	reg b_step1;
	reg [6:0] b_n1, b_n2, b_n3;
	reg b_is_gpr1, b_is_gpr2, b_is_gpr3;
	reg b_hit_rf_1, b_hit_rf_2, b_hit_rf_3;
	wire [DW-1:0] hold_r1;
	wire [DW-1:0] hold_r2;

	/* Flat 1-hot unit_ready check for REG_IO != 0 */
	wire [13:1] b_not_rdy_vec = b_rdy_oh_r & ~unit_ready[13:1];
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) wire b_adv_fast = !(|b_not_rdy_vec);
`else
	wire b_adv_fast = !(|b_not_rdy_vec);
`endif
	wire b_adv_fast_r, b_adv_fast_c, b_adv_fast_x, b_adv_fast_q, b_adv_fast_b;
	wire b_adv_fast_b0, b_adv_fast_b1;
	wire a_wait_ctl, a_wait_adv, a_act_ctl, a_act_adv, a_step_ctl;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_r  (.I0(b_adv_fast), .O(b_adv_fast_r));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_c  (.I0(b_adv_fast), .O(b_adv_fast_c));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_x  (.I0(b_adv_fast), .O(b_adv_fast_x));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_q  (.I0(b_adv_fast), .O(b_adv_fast_q));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_b  (.I0(b_adv_fast), .O(b_adv_fast_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_b0 (.I0(b_adv_fast), .O(b_adv_fast_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_baf_b1 (.I0(b_adv_fast), .O(b_adv_fast_b1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awt_c  (.I0(a_wait_pre_r[15]), .O(a_wait_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awt_a  (.I0(a_wait),           .O(a_wait_adv));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aact_c (.I0(a_act_pre_r[15]),  .O(a_act_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aact_a (.I0(a_act),            .O(a_act_adv));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_astp_c (.I0(a_step),           .O(a_step_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign b_adv_fast_r = b_adv_fast;
	assign b_adv_fast_c = b_adv_fast;
	assign b_adv_fast_x = b_adv_fast;
	assign b_adv_fast_q = b_adv_fast;
	assign b_adv_fast_b = b_adv_fast;
	assign b_adv_fast_b0 = b_adv_fast;
	assign b_adv_fast_b1 = b_adv_fast;
	assign a_wait_ctl = a_wait_pre_r[15];
	assign a_wait_adv = a_wait;
	assign a_act_ctl = a_act_pre_r[15];
	assign a_act_adv = a_act;
	assign a_step_ctl = a_step;
`else
	assign b_adv_fast_r  = b_adv_fast;
	assign b_adv_fast_c  = b_adv_fast;
	assign b_adv_fast_x  = b_adv_fast;
	assign b_adv_fast_q  = b_adv_fast;
	assign b_adv_fast_b  = b_adv_fast;
	assign b_adv_fast_b0 = b_adv_fast;
	assign b_adv_fast_b1 = b_adv_fast;
	assign a_wait_ctl    = a_wait_pre_r[15];
	assign a_wait_adv    = a_wait;
	assign a_act_ctl     = a_act_pre_r[15];
	assign a_act_adv     = a_act;
	assign a_step_ctl    = a_step;
`endif
	wire r_en_fast   = a_act_ctl && !a_wait_ctl && b_adv_fast_r;
	wire cr_en_fast  = a_act_ctl && !a_wait_ctl && !a_step_ctl && b_adv_fast_c;
	wire rq_pop_fast = a_act_ctl && !a_wait_ctl && a_uop[ATG_UOP_RQS] && !a_pass_out && !a_step_ctl && b_adv_fast_q;
	wire x_end_fast  = b_valid_last_r && b_adv_fast_x;

	wire [3:0] b_unit = b_uop[ATG_UOP_UNIT +: 4];
	wire b_rdy = (REG_IO != 0) ? b_adv_fast_b : unit_ready[b_unit];
	wire b_adv = (REG_IO != 0) ? b_adv_fast_b : (!b_valid || b_rdy);
	wire b_adv_1 = (REG_IO != 0) ? b_adv_fast_b0 : (!b_valid || b_rdy);
	wire b_adv_2 = (REG_IO != 0) ? b_adv_fast_b1 : (!b_valid || b_rdy);
	assign x_end_out = (REG_IO != 0) ? x_end_fast : (b_valid && b_last && b_rdy);
	wire a_adv = a_act_adv && ((REG_IO != 0) ? a_wait_adv : b_adv);
	wire a_adv_1, a_adv_2, a_adv_3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aadv1 (.I0(a_adv), .O(a_adv_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aadv2 (.I0(a_adv), .O(a_adv_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aadv3 (.I0(a_adv), .O(a_adv_3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign a_adv_1 = a_adv;
	assign a_adv_2 = a_adv;
	assign a_adv_3 = a_adv;
`else
	assign a_adv_1 = a_adv;
	assign a_adv_2 = a_adv;
	assign a_adv_3 = a_adv;
`endif
	wire a_wide = a_uop[ATG_UOP_PAIR] && a_uop[ATG_UOP_USE3];
	wire a_needs_step1 = a_uop[ATG_UOP_USE3] || a_uop[ATG_UOP_PAIR];
	wire a_step_done = !a_needs_step1 || a_step_ctl;
	wire a_pass_done = (!a_wide || a_pass_ctl) && a_step_done;
	wire a_last = ({{(32-GRP_W){1'b0}}, a_grp} == G - 1) && a_pass_done;
	assign i_ready = !a_act_adv || ((REG_IO != 0) ? a_wait_last : (a_adv && a_last));

	/* Operand numbers; operand 3 is rs1+1 for the PAIR instructions;
	 * operands 1 and 3 are rs3 and rs3+1 on a second pass. */
	wire [6:0] a_r1 = {a_pass_r1_hi ? a_uop[ATG_UOP_RS3 + 3 +: 4] : a_uop[ATG_UOP_RS1 + 3 +: 4],
	                   a_pass_r1_lo ? a_uop[ATG_UOP_RS3 +: 3]     : a_uop[ATG_UOP_RS1 +: 3]};
	wire [6:0] a_n1 = a_r1;
	wire [6:0] a_n2 = a_uop[ATG_UOP_RS2 +: 7];
	wire [6:0] i_rs1_p1, i_rs3_p1;
	wire [GRP_W-1:0] a_grp_next;
	wire ir1_co_unused, ir3_co_unused, ag_co_unused;
	am_add #(.W(7)) u_i_rs1p (.a(i_uop[ATG_UOP_RS1 +: 7]), .b(7'd0), .ci(1'b1), .s(i_rs1_p1), .co(ir1_co_unused));
	am_add #(.W(7)) u_i_rs3p (.a(i_uop[ATG_UOP_RS3 +: 7]), .b(7'd0), .ci(1'b1), .s(i_rs3_p1), .co(ir3_co_unused));
	am_add #(.W(GRP_W)) u_a_grp (.a(a_grp), .b({GRP_W{1'b0}}), .ci(1'b1), .s(a_grp_next),
		.co(ag_co_unused));
	wire [6:0] a_n3 = a_uop[ATG_UOP_PAIR] ? (a_pass_r1_lo ? a_rs3_p1_r : a_rs1_p1_r) : a_uop[ATG_UOP_RS3 +: 7];

	wire [6:0] a_rf_r1 = a_step_ctl ? a_n3 : a_r1;
	wire [6:0] a_rf_r2 = a_step_ctl ? 7'd0 : a_n2;

	assign r_en = (REG_IO != 0) ? r_en_fast : (a_act && b_adv);
	genvar grn;
	generate
		for (grn = 0; grn < 8; grn = grn + 1) begin : g_ren_rep
			wire ren_g4_s = a_act_ren_r[grn] && !a_wait_ren_r[grn];
			assign r_en_rep[grn] = (REG_IO != 0) ? (ren_g4_s && b_adv_fast_r) : r_en;
		end
	endgenerate
	assign r_warp = a_rf_warp_r;
	assign r_grp = a_rf_grp_r;
	assign r_regs = {a_rf_r2_r, a_rf_r1_r};
	assign cr_en = (REG_IO != 0) ? cr_en_fast : (a_act && b_adv && (!a_needs_step1 || !a_step));
	wire [4:0] a_n1_cr, a_n2_cr, a_n3_cr;
	wire [4:0] a_n1_opc_0, a_n1_opc_0h, a_n1_opc_1, a_n1_opc_2, a_n1_opc_3;
	wire [4:0] a_n2_opc_0, a_n2_opc_0h, a_n2_opc_1, a_n2_opc_2, a_n2_opc_3;
	wire [4:0] a_n3_opc_0, a_n3_opc_0h, a_n3_opc_1, a_n3_opc_2, a_n3_opc_3;

	assign a_n1_cr    = (REG_IO != 0) ? a_rf_r1_r[4:0] : a_n1[4:0];
	assign a_n2_cr    = (REG_IO != 0) ? a_rf_r2_r[4:0] : a_n2[4:0];
	assign a_n3_cr    = (REG_IO != 0) ? a_cr_r3_r      : a_n3[4:0];
`ifdef SYNTH_KINTEX7
	genvar gopc;
	generate
		for (gopc = 0; gopc < 5; gopc = gopc + 1) begin : g_an_opc
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n1o0  (.I0(a_n1[gopc]), .O(a_n1_opc_0[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n1o0h (.I0(a_n1[gopc]), .O(a_n1_opc_0h[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n1o1  (.I0(a_n1[gopc]), .O(a_n1_opc_1[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n1o2  (.I0(a_n1[gopc]), .O(a_n1_opc_2[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n1o3  (.I0(a_n1[gopc]), .O(a_n1_opc_3[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n2o0  (.I0(a_n2[gopc]), .O(a_n2_opc_0[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n2o0h (.I0(a_n2[gopc]), .O(a_n2_opc_0h[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n2o1  (.I0(a_n2[gopc]), .O(a_n2_opc_1[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n2o2  (.I0(a_n2[gopc]), .O(a_n2_opc_2[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n2o3  (.I0(a_n2[gopc]), .O(a_n2_opc_3[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n3o0  (.I0(a_n3[gopc]), .O(a_n3_opc_0[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n3o0h (.I0(a_n3[gopc]), .O(a_n3_opc_0h[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n3o1  (.I0(a_n3[gopc]), .O(a_n3_opc_1[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n3o2  (.I0(a_n3[gopc]), .O(a_n3_opc_2[gopc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_n3o3  (.I0(a_n3[gopc]), .O(a_n3_opc_3[gopc]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gopc;
	generate
		for (gopc = 0; gopc < 5; gopc = gopc + 1) begin : g_an_opc
			assign a_n1_opc_0[gopc] = a_n1[gopc];
			assign a_n1_opc_0h[gopc] = a_n1[gopc];
			assign a_n1_opc_1[gopc] = a_n1[gopc];
			assign a_n1_opc_2[gopc] = a_n1[gopc];
			assign a_n1_opc_3[gopc] = a_n1[gopc];
			assign a_n2_opc_0[gopc] = a_n2[gopc];
			assign a_n2_opc_0h[gopc] = a_n2[gopc];
			assign a_n2_opc_1[gopc] = a_n2[gopc];
			assign a_n2_opc_2[gopc] = a_n2[gopc];
			assign a_n2_opc_3[gopc] = a_n2[gopc];
			assign a_n3_opc_0[gopc] = a_n3[gopc];
			assign a_n3_opc_0h[gopc] = a_n3[gopc];
			assign a_n3_opc_1[gopc] = a_n3[gopc];
			assign a_n3_opc_2[gopc] = a_n3[gopc];
			assign a_n3_opc_3[gopc] = a_n3[gopc];
		end
	endgenerate
`else
	assign a_n1_opc_0  = a_n1[4:0];
	assign a_n1_opc_0h = a_n1[4:0];
	assign a_n1_opc_1  = a_n1[4:0];
	assign a_n1_opc_2  = a_n1[4:0];
	assign a_n1_opc_3  = a_n1[4:0];
	assign a_n2_opc_0  = a_n2[4:0];
	assign a_n2_opc_0h = a_n2[4:0];
	assign a_n2_opc_1  = a_n2[4:0];
	assign a_n2_opc_2  = a_n2[4:0];
	assign a_n2_opc_3  = a_n2[4:0];
	assign a_n3_opc_0  = a_n3[4:0];
	assign a_n3_opc_0h = a_n3[4:0];
	assign a_n3_opc_1  = a_n3[4:0];
	assign a_n3_opc_2  = a_n3[4:0];
	assign a_n3_opc_3  = a_n3[4:0];
`endif

	assign cr_idx = {a_n3_cr, a_n2_cr, a_n1_cr};
	assign rq_pop = (REG_IO != 0) ? rq_pop_fast : (a_act && b_adv && a_uop[ATG_UOP_RQS] && !a_pass_out && (!a_needs_step1 || !a_step));

	wire a_match_wb_1 = fwd_wb_valid && (a_n1 < 7'd63) && (fwd_wb_rd < 7'd63) &&
	                    (fwd_wb_warp == a_warp) && (fwd_wb_grp == a_grp) && (fwd_wb_rd[5:0] == a_n1[5:0]);
	wire a_match_wb_2 = fwd_wb_valid && (a_n2 < 7'd63) && (fwd_wb_rd < 7'd63) &&
	                    (fwd_wb_warp == a_warp) && (fwd_wb_grp == a_grp) && (fwd_wb_rd[5:0] == a_n2[5:0]);
	wire a_match_wb_3 = fwd_wb_valid && (a_n3 < 7'd63) && (fwd_wb_rd < 7'd63) &&
	                    (fwd_wb_warp == a_warp) && (fwd_wb_grp == a_grp) && (fwd_wb_rd[5:0] == a_n3[5:0]);

	wire a_match_alu_ex4_1 = fwd_alu_ex3_valid && (a_n1 < 7'd63) && (fwd_alu_ex3_rd < 7'd63) &&
	                         (fwd_alu_ex3_warp == a_warp) && (fwd_alu_ex3_grp == a_grp) && (fwd_alu_ex3_rd[5:0] == a_n1[5:0]);
	wire a_match_alu_ex4_2 = fwd_alu_ex3_valid && (a_n2 < 7'd63) && (fwd_alu_ex3_rd < 7'd63) &&
	                         (fwd_alu_ex3_warp == a_warp) && (fwd_alu_ex3_grp == a_grp) && (fwd_alu_ex3_rd[5:0] == a_n2[5:0]);
	wire a_match_alu_ex4_3 = fwd_alu_ex3_valid && (a_n3 < 7'd63) && (fwd_alu_ex3_rd < 7'd63) &&
	                         (fwd_alu_ex3_warp == a_warp) && (fwd_alu_ex3_grp == a_grp) && (fwd_alu_ex3_rd[5:0] == a_n3[5:0]);

	wire a_match_alu_ex5_1 = fwd_alu_ex4_valid && (a_n1 < 7'd63) && (fwd_alu_ex4_rd < 7'd63) &&
	                         (fwd_alu_ex4_warp == a_warp) && (fwd_alu_ex4_grp == a_grp) && (fwd_alu_ex4_rd[5:0] == a_n1[5:0]);
	wire a_match_alu_ex5_2 = fwd_alu_ex4_valid && (a_n2 < 7'd63) && (fwd_alu_ex4_rd < 7'd63) &&
	                         (fwd_alu_ex4_warp == a_warp) && (fwd_alu_ex4_grp == a_grp) && (fwd_alu_ex4_rd[5:0] == a_n2[5:0]);
	wire a_match_alu_ex5_3 = fwd_alu_ex4_valid && (a_n3 < 7'd63) && (fwd_alu_ex4_rd < 7'd63) &&
	                         (fwd_alu_ex4_warp == a_warp) && (fwd_alu_ex4_grp == a_grp) && (fwd_alu_ex4_rd[5:0] == a_n3[5:0]);

	reg b_hit_alu_ex4_1, b_hit_alu_ex4_2, b_hit_alu_ex4_3;
	reg b_hit_alu_ex5_1, b_hit_alu_ex5_2, b_hit_alu_ex5_3;
	reg [159:0] b_q_info;

	/* Stage A precomputation registers for REG_IO != 0 */
	(* dont_touch = "true" *) reg [2:0] a_k1_r, a_k2_r, a_k3_r;
	reg [31:0] a_cval1_r, a_cval2_r, a_cval3_r;
	wire [DW-1:0] a_cl_r;
	reg [NPHYS-1:0] a_cov_g_r;
	reg [31:0] a_info_r;

	/* Buffered q_info[ATG_WI_FRONT] (6 copies, <= 17 loads each) */
	(* dont_touch = "true" *) wire qi_front_1a = q_info[ATG_WI_FRONT];
	(* dont_touch = "true" *) wire qi_front_1b = q_info[ATG_WI_FRONT];
	(* dont_touch = "true" *) wire qi_front_2a = q_info[ATG_WI_FRONT];
	(* dont_touch = "true" *) wire qi_front_2b = q_info[ATG_WI_FRONT];
	(* dont_touch = "true" *) wire qi_front_3a = q_info[ATG_WI_FRONT];
	(* dont_touch = "true" *) wire qi_front_3b = q_info[ATG_WI_FRONT];

	/* Per-slice replicated enables for Stage A precomputation, Stage A->B, and Stage A load */
	wire [3:0] a_pre_k_en;
	wire a_pre_cl_en0 = (REG_IO != 0) && a_act_pre_r[0] && !a_wait_pre_r[0];
	wire a_pre_cl_en1 = (REG_IO != 0) && a_act_pre_r[1] && !a_wait_pre_r[1];
	assign a_pre_k_en[0] = (REG_IO != 0) && a_act_pre_r[2] && !a_wait_pre_r[2];
	assign a_pre_k_en[1] = (REG_IO != 0) && a_act_pre_r[3] && !a_wait_pre_r[3];
	assign a_pre_k_en[2] = (REG_IO != 0) && a_act_pre_r[4] && !a_wait_pre_r[4];
	assign a_pre_k_en[3] = (REG_IO != 0) && a_act_pre_r[5] && !a_wait_pre_r[5];
	wire a_pre_we_0  = (REG_IO != 0) && a_act_pre_r[6]  && !a_wait_pre_r[6];
	wire a_pre_we_1  = (REG_IO != 0) && a_act_pre_r[7]  && !a_wait_pre_r[7];
	wire a_pre_we_2  = (REG_IO != 0) && a_act_pre_r[8]  && !a_wait_pre_r[8];
	wire a_pre_we_3  = (REG_IO != 0) && a_act_pre_r[9]  && !a_wait_pre_r[9];
	wire a_pre_we_4  = (REG_IO != 0) && a_act_pre_r[10] && !a_wait_pre_r[10];
	wire a_pre_we_5  = (REG_IO != 0) && a_act_pre_r[11] && !a_wait_pre_r[11];
	wire a_pre_we_6  = (REG_IO != 0) && a_act_pre_r[12] && !a_wait_pre_r[12];
	wire a_pre_we_7a = (REG_IO != 0) && a_act_pre_r[13] && !a_wait_pre_r[13];
	wire a_pre_we_7b = (REG_IO != 0) && a_act_pre_r[14] && !a_wait_pre_r[14];


	wire a_act_en, a_wait_en, a_step_en;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aact_e (.I0(a_act),  .O(a_act_en));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_awt_e  (.I0(a_wait), .O(a_wait_en));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_astp_e (.I0(a_step), .O(a_step_en));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign a_act_en = a_act;
	assign a_wait_en = a_wait;
	assign a_step_en = a_step;
`else
	assign a_act_en  = a_act;
	assign a_wait_en = a_wait;
	assign a_step_en = a_step;
`endif
	wire [NPHYS-1:0] a_stgb_en_g;
	wire [NPHYS-1:0] a_hold_en_g;
	genvar geg;
	generate
		for (geg = 0; geg < NPHYS; geg = geg + 1) begin : g_en_grp
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sg_en (.I0((REG_IO != 0) ? (a_act_en && a_wait_en && a_step_done) : (b_adv && a_act_en && a_step_done)), .O(a_stgb_en_g[geg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hd_en (.I0(a_step_en && !a_wait_en), .O(a_hold_en_g[geg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign a_stgb_en_g[geg] = (REG_IO != 0) ? (a_act_en && a_wait_en && a_step_done) : (b_adv && a_act_en && a_step_done);
			assign a_hold_en_g[geg] = a_step_en && !a_wait_en;
`else
			assign a_stgb_en_g[geg] = (REG_IO != 0) ? (a_act_en && a_wait_en && a_step_done) : (b_adv && a_act_en && a_step_done);
			assign a_hold_en_g[geg] = a_step_en && !a_wait_en;
`endif
		end
	endgenerate

	wire a_stgb_ctl_raw = (REG_IO != 0) ? (a_act_en && a_wait_en && a_step_done) : (b_adv && a_act_en && a_step_done);
	wire stgb_ctl_0, stgb_ctl_1, stgb_ctl_2;
	wire b_ctl_we_0, b_ctl_we_1, b_ctl_we_2, b_ctl_we_3, b_ctl_we_4, b_ctl_we_5, b_ctl_we_6, b_ctl_we_7;
	wire b_ctl_we_8, b_ctl_we_9, b_ctl_we_9a, b_ctl_we_9b, b_ctl_we_10, b_ctl_we_11, b_ctl_we_12, b_ctl_we_13, b_ctl_we_14;
	wire [8:2] b_ctl_we_pt, b_ctl_we_fp, b_ctl_we_ln;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stgb0  (.I0(a_stgb_ctl_raw), .O(stgb_ctl_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stgb1  (.I0(a_stgb_ctl_raw), .O(stgb_ctl_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_stgb2  (.I0(a_stgb_ctl_raw), .O(stgb_ctl_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe0  (.I0(stgb_ctl_0), .O(b_ctl_we_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe1  (.I0(stgb_ctl_0), .O(b_ctl_we_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe2  (.I0(stgb_ctl_0), .O(b_ctl_we_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe3  (.I0(stgb_ctl_0), .O(b_ctl_we_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe4  (.I0(stgb_ctl_0), .O(b_ctl_we_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe5  (.I0(stgb_ctl_0), .O(b_ctl_we_5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe6  (.I0(stgb_ctl_0), .O(b_ctl_we_6));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe7  (.I0(stgb_ctl_0), .O(b_ctl_we_7));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe8  (.I0(stgb_ctl_0), .O(b_ctl_we_8));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe9  (.I0(stgb_ctl_1), .O(b_ctl_we_9));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe9a (.I0(stgb_ctl_1), .O(b_ctl_we_9a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe9b (.I0(stgb_ctl_1), .O(b_ctl_we_9b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe10 (.I0(stgb_ctl_1), .O(b_ctl_we_10));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe11 (.I0(stgb_ctl_1), .O(b_ctl_we_11));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe12 (.I0(stgb_ctl_1), .O(b_ctl_we_12));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe13 (.I0(stgb_ctl_1), .O(b_ctl_we_13));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcwe14 (.I0(stgb_ctl_1), .O(b_ctl_we_14));
	genvar gbc;
	generate
		for (gbc = 2; gbc <= 8; gbc = gbc + 1) begin : g_bcwe_uop
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcpt (.I0(stgb_ctl_2), .O(b_ctl_we_pt[gbc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcfp (.I0(stgb_ctl_2), .O(b_ctl_we_fp[gbc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bcln (.I0(stgb_ctl_2), .O(b_ctl_we_ln[gbc]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	assign stgb_ctl_0 = a_stgb_ctl_raw;
	assign stgb_ctl_1 = a_stgb_ctl_raw;
	assign stgb_ctl_2 = a_stgb_ctl_raw;
	assign b_ctl_we_0 = stgb_ctl_0;
	assign b_ctl_we_1 = stgb_ctl_0;
	assign b_ctl_we_2 = stgb_ctl_0;
	assign b_ctl_we_3 = stgb_ctl_0;
	assign b_ctl_we_4 = stgb_ctl_0;
	assign b_ctl_we_5 = stgb_ctl_0;
	assign b_ctl_we_6 = stgb_ctl_0;
	assign b_ctl_we_7 = stgb_ctl_0;
	assign b_ctl_we_8 = stgb_ctl_0;
	assign b_ctl_we_9 = stgb_ctl_1;
	assign b_ctl_we_9a = stgb_ctl_1;
	assign b_ctl_we_9b = stgb_ctl_1;
	assign b_ctl_we_10 = stgb_ctl_1;
	assign b_ctl_we_11 = stgb_ctl_1;
	assign b_ctl_we_12 = stgb_ctl_1;
	assign b_ctl_we_13 = stgb_ctl_1;
	assign b_ctl_we_14 = stgb_ctl_1;
	genvar gbc;
	generate
		for (gbc = 2; gbc <= 8; gbc = gbc + 1) begin : g_bcwe_uop
			assign b_ctl_we_pt[gbc] = stgb_ctl_2;
			assign b_ctl_we_fp[gbc] = stgb_ctl_2;
			assign b_ctl_we_ln[gbc] = stgb_ctl_2;
		end
	endgenerate
`else
	assign stgb_ctl_0 = a_stgb_ctl_raw;
	assign stgb_ctl_1 = a_stgb_ctl_raw;
	assign stgb_ctl_2 = a_stgb_ctl_raw;
	assign b_ctl_we_0  = a_stgb_ctl_raw;
	assign b_ctl_we_1  = a_stgb_ctl_raw;
	assign b_ctl_we_2  = a_stgb_ctl_raw;
	assign b_ctl_we_3  = a_stgb_ctl_raw;
	assign b_ctl_we_4  = a_stgb_ctl_raw;
	assign b_ctl_we_5  = a_stgb_ctl_raw;
	assign b_ctl_we_6  = a_stgb_ctl_raw;
	assign b_ctl_we_7  = a_stgb_ctl_raw;
	assign b_ctl_we_8  = a_stgb_ctl_raw;
	assign b_ctl_we_9  = a_stgb_ctl_raw;
	assign b_ctl_we_9a = a_stgb_ctl_raw;
	assign b_ctl_we_9b = a_stgb_ctl_raw;
	assign b_ctl_we_10 = a_stgb_ctl_raw;
	assign b_ctl_we_11 = a_stgb_ctl_raw;
	assign b_ctl_we_12 = a_stgb_ctl_raw;
	assign b_ctl_we_13 = a_stgb_ctl_raw;
	assign b_ctl_we_14 = a_stgb_ctl_raw;
	assign b_ctl_we_pt = {7{a_stgb_ctl_raw}};
	assign b_ctl_we_fp = {7{a_stgb_ctl_raw}};
	assign b_ctl_we_ln = {7{a_stgb_ctl_raw}};
`endif

	wire a_load_we_raw = !a_act || ((REG_IO != 0) ? a_wait_last : (a_adv && a_last));
	wire a_load_we_0, a_load_we_1, a_load_we_2, a_load_we_3, a_load_we_4;
	wire a_load_we_5, a_load_we_6, a_load_we_7, a_load_we_8, a_load_we_8a, a_load_we_9, a_load_we_9a;
	wire [WARP_W-1:0] i_warp_a, i_warp_q, i_warp_rf;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe0  (.I0(a_load_we_raw), .O(a_load_we_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe1  (.I0(a_load_we_raw), .O(a_load_we_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe2  (.I0(a_load_we_raw), .O(a_load_we_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe3  (.I0(a_load_we_raw), .O(a_load_we_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe4  (.I0(a_load_we_raw), .O(a_load_we_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe5  (.I0(a_load_we_raw), .O(a_load_we_5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe6  (.I0(a_load_we_raw), .O(a_load_we_6));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe7  (.I0(a_load_we_raw), .O(a_load_we_7));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe8  (.I0(a_load_we_raw), .O(a_load_we_8));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe8a (.I0(a_load_we_raw), .O(a_load_we_8a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe9  (.I0(a_load_we_raw), .O(a_load_we_9));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_alwe9a (.I0(a_load_we_raw), .O(a_load_we_9a));
	genvar giwb;
	for (giwb = 0; giwb < WARP_W; giwb = giwb + 1) begin : g_iw_buf
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iwa  (.I0(i_warp[giwb]), .O(i_warp_a[giwb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iwq  (.I0(i_warp[giwb]), .O(i_warp_q[giwb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iwrf (.I0(i_warp[giwb]), .O(i_warp_rf[giwb]));
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	assign a_load_we_0 = a_load_we_raw;
	assign a_load_we_1 = a_load_we_raw;
	assign a_load_we_2 = a_load_we_raw;
	assign a_load_we_3 = a_load_we_raw;
	assign a_load_we_4 = a_load_we_raw;
	assign a_load_we_5 = a_load_we_raw;
	assign a_load_we_6 = a_load_we_raw;
	assign a_load_we_7 = a_load_we_raw;
	assign a_load_we_8 = a_load_we_raw;
	assign a_load_we_8a = a_load_we_raw;
	assign a_load_we_9 = a_load_we_raw;
	assign a_load_we_9a = a_load_we_raw;
	genvar giwb;
	for (giwb = 0; giwb < WARP_W; giwb = giwb + 1) begin : g_iw_buf
		assign i_warp_a[giwb] = i_warp[giwb];
		assign i_warp_q[giwb] = i_warp[giwb];
		assign i_warp_rf[giwb] = i_warp[giwb];
	end
`else
	assign a_load_we_0  = a_load_we_raw;
	assign a_load_we_1  = a_load_we_raw;
	assign a_load_we_2  = a_load_we_raw;
	assign a_load_we_3  = a_load_we_raw;
	assign a_load_we_4  = a_load_we_raw;
	assign a_load_we_5  = a_load_we_raw;
	assign a_load_we_6  = a_load_we_raw;
	assign a_load_we_7  = a_load_we_raw;
	assign a_load_we_8  = a_load_we_raw;
	assign a_load_we_8a = a_load_we_raw;
	assign a_load_we_9  = a_load_we_raw;
	assign a_load_we_9a = a_load_we_raw;
	assign i_warp_a     = i_warp;
	assign i_warp_q     = i_warp;
	assign i_warp_rf    = i_warp;
`endif


	wire [DW-1:0] b_early_va, b_early_vb, b_early_vc;
	reg b_use_rf1, b_use_rf2, b_use_rf3;
	(* keep = "true", dont_touch = "true" *) reg [NPHYS*2-1:0] b_use_rf1_rep, b_use_rf2_rep, b_use_rf3_rep;
	reg b_clr31_1, b_inv31_1, b_clr15_1, b_inv15_1;
	reg b_clr31_2, b_inv31_2, b_clr15_2, b_inv15_2;
	reg b_clr31_3, b_inv31_3, b_clr15_3, b_inv15_3;
	reg [2*NPHYS-1:0] b_mask_r;
	reg [31:0] b_info_r;

	wire [4:0] a_grp_base = (G > 1) ? {a_grp, {($clog2(NPHYS)){1'b0}}} : 5'd0;
	wire [4:0] a_grp_base_cl, a_grp_base_e0, a_grp_base_e1;
`ifdef SYNTH_KINTEX7
	genvar ggbb;
	generate
		for (ggbb = 0; ggbb < 5; ggbb = ggbb + 1) begin : g_agb_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_agbc  (.I0(a_grp_base[ggbb]), .O(a_grp_base_cl[ggbb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_agbe0 (.I0(a_grp_base[ggbb]), .O(a_grp_base_e0[ggbb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_agbe1 (.I0(a_grp_base[ggbb]), .O(a_grp_base_e1[ggbb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar ggbb;
	generate
		for (ggbb = 0; ggbb < 5; ggbb = ggbb + 1) begin : g_agb_buf
			assign a_grp_base_cl[ggbb] = a_grp_base[ggbb];
			assign a_grp_base_e0[ggbb] = a_grp_base[ggbb];
			assign a_grp_base_e1[ggbb] = a_grp_base[ggbb];
		end
	endgenerate
`else
	assign a_grp_base_cl = a_grp_base;
	assign a_grp_base_e0 = a_grp_base;
	assign a_grp_base_e1 = a_grp_base;
`endif
	wire [3:0] a_cl_hi0 = {1'b0, q_info[ATG_WI_LID+5 +: 3]};
	wire [3:0] a_cl_hi1;
	wire a_cl_hi_co_unused;
	am_add #(.W(4)) u_acl_hi (.a(a_cl_hi0), .b(4'd1), .ci(1'b0), .s(a_cl_hi1), .co(a_cl_hi_co_unused));
	wire [DW-1:0] a_cl_comb;
	wire [NPHYS-1:0] a_cl_co_unused;
	genvar la;
	generate
		for (la = 0; la < NPHYS; la = la + 1) begin : g_acl
			wire [4:0] a_ln = a_grp_base_cl[4:0] | la[4:0];
			wire [4:0] a_cl_lo;
			wire [8:0] a_cl_s = {a_cl_co_unused[la] ? a_cl_hi1 : a_cl_hi0, a_cl_lo};
			am_add #(.W(5)) u_acl (
				.a(q_info[ATG_WI_LID +: 5]),
				.b(a_ln),
				.ci(1'b0),
				.s(a_cl_lo),
				.co(a_cl_co_unused[la])
			);
			assign a_cl_comb[la*32 +: 32] = {23'd0, a_cl_s};
			(* shreg_extract = "no" *) reg [8:0] a_cl_l;
			wire a_cl_we_l;

			assign a_cl_we_l = (la < (NPHYS/2)) ? a_pre_cl_en0 : a_pre_cl_en1;

			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					a_cl_l <= 9'd0;
				else if (a_cl_we_l)
					a_cl_l <= a_cl_s;
			end
			assign a_cl_r[la*32 +: 32] = {23'd0, a_cl_l};
		end
	endgenerate

	reg [NPHYS-1:0] a_exec_g, a_kill_g, a_cov_g;
	integer agi;
	always @(*) begin
		a_exec_g = {NPHYS{1'b0}};
		a_kill_g = {NPHYS{1'b0}};
		a_cov_g  = {NPHYS{1'b0}};
		for (agi = 0; agi < 32/NPHYS; agi = agi + 1) begin
			a_exec_g = a_exec_g | ({NPHYS{a_grp_oh_r[agi]}} & q_exec[agi*NPHYS +: NPHYS]);
			a_kill_g = a_kill_g | ({NPHYS{a_grp_oh_r[agi]}} & q_killed[agi*NPHYS +: NPHYS]);
			a_cov_g  = a_cov_g  | ({NPHYS{a_grp_oh_r[agi]}} & q_info[ATG_WI_MASK + agi*NPHYS +: NPHYS]);
		end
	end

	wire [3:0] a_unit = a_uop[ATG_UOP_UNIT +: 4];
	wire [7:0] a_op = a_uop[ATG_UOP_OP +: 8];
	wire [6:0] a_mods = a_uop[ATG_UOP_MOD +: 7];
	wire [3:0] a_unit_cr, a_unit_e0, a_unit_e1, a_unit_rdy;
	wire [5:0] a_mods_cr, a_mods_e0, a_mods_e1;
`ifdef SYNTH_KINTEX7
	genvar guab, gmab;
	generate
		for (guab = 0; guab < 4; guab = guab + 1) begin : g_aunit_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aucr  (.I0(a_unit[guab]), .O(a_unit_cr[guab]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aue0  (.I0(a_unit[guab]), .O(a_unit_e0[guab]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aue1  (.I0(a_unit[guab]), .O(a_unit_e1[guab]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aurdy (.I0(a_unit[guab]), .O(a_unit_rdy[guab]));
		end
		for (gmab = 0; gmab < 6; gmab = gmab + 1) begin : g_amods_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_amcr (.I0(a_mods[gmab]), .O(a_mods_cr[gmab]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ame0 (.I0(a_mods[gmab]), .O(a_mods_e0[gmab]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ame1 (.I0(a_mods[gmab]), .O(a_mods_e1[gmab]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar guab, gmab;
	generate
		for (guab = 0; guab < 4; guab = guab + 1) begin : g_aunit_buf
			assign a_unit_cr[guab] = a_unit[guab];
			assign a_unit_e0[guab] = a_unit[guab];
			assign a_unit_e1[guab] = a_unit[guab];
			assign a_unit_rdy[guab] = a_unit[guab];
		end
		for (gmab = 0; gmab < 6; gmab = gmab + 1) begin : g_amods_buf
			assign a_mods_cr[gmab] = a_mods[gmab];
			assign a_mods_e0[gmab] = a_mods[gmab];
			assign a_mods_e1[gmab] = a_mods[gmab];
		end
	endgenerate
`else
	assign a_unit_cr     = a_unit;
	assign a_unit_e0     = a_unit;
	assign a_unit_e1     = a_unit;
	assign a_unit_rdy    = a_unit;
	assign a_mods_cr     = a_mods[5:0];
	assign a_mods_e0     = a_mods[5:0];
	assign a_mods_e1     = a_mods[5:0];
`endif
	wire a_fp_cr = a_unit_cr == ATG_UNIT_FPU || a_unit_cr == ATG_UNIT_XU;
	wire a_pk_cr = a_unit_cr == ATG_UNIT_PK;
	wire a_fp_unit = a_fp_cr;
	wire a_pk_unit = a_pk_cr;
	wire [31:0] cr_fmod1 = fmod(cr_data[31:0],  a_mods_cr[0], a_mods_cr[1], a_fp_cr, a_pk_cr);
	wire [31:0] cr_fmod2 = fmod(cr_data[63:32], a_mods_cr[2], a_mods_cr[3], a_fp_cr, a_pk_cr);
	wire [31:0] cr_fmod3 = fmod(cr_data[95:64], a_mods_cr[4], a_mods_cr[5], a_fp_cr, a_pk_cr);
	wire [31:0] q_fmod1  = fmod(q_exec, a_mods_cr[0], a_mods_cr[1], a_fp_cr, a_pk_cr);
	wire [31:0] q_fmod2  = fmod(q_exec, a_mods_cr[2], a_mods_cr[3], a_fp_cr, a_pk_cr);
	wire [31:0] q_fmod3  = fmod(q_exec, a_mods_cr[4], a_mods_cr[5], a_fp_cr, a_pk_cr);
	wire [31:0] cr_fmod1_g [0:3];
	wire [31:0] cr_fmod2_g [0:3];
	wire [31:0] cr_fmod3_g [0:3];
	wire [31:0] q_fmod1_g  [0:3];
	wire [31:0] q_fmod2_g  [0:3];
	wire [31:0] q_fmod3_g  [0:3];
	genvar gcrg, gcrb;
	generate
		for (gcrg = 0; gcrg < 4; gcrg = gcrg + 1) begin : g_crf_grp
			for (gcrb = 0; gcrb < 32; gcrb = gcrb + 1) begin : g_crf_bit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_crf1 (.I0(cr_fmod1[gcrb]), .O(cr_fmod1_g[gcrg][gcrb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_crf2 (.I0(cr_fmod2[gcrb]), .O(cr_fmod2_g[gcrg][gcrb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_crf3 (.I0(cr_fmod3[gcrb]), .O(cr_fmod3_g[gcrg][gcrb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qfm1 (.I0(q_fmod1[gcrb]),  .O(q_fmod1_g[gcrg][gcrb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qfm2 (.I0(q_fmod2[gcrb]),  .O(q_fmod2_g[gcrg][gcrb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qfm3 (.I0(q_fmod3[gcrb]),  .O(q_fmod3_g[gcrg][gcrb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign cr_fmod1_g[gcrg][gcrb] = cr_fmod1[gcrb];
				assign cr_fmod2_g[gcrg][gcrb] = cr_fmod2[gcrb];
				assign cr_fmod3_g[gcrg][gcrb] = cr_fmod3[gcrb];
				assign q_fmod1_g[gcrg][gcrb] = q_fmod1[gcrb];
				assign q_fmod2_g[gcrg][gcrb] = q_fmod2[gcrb];
				assign q_fmod3_g[gcrg][gcrb] = q_fmod3[gcrb];
`else
				assign cr_fmod1_g[gcrg][gcrb] = cr_fmod1[gcrb];
				assign cr_fmod2_g[gcrg][gcrb] = cr_fmod2[gcrb];
				assign cr_fmod3_g[gcrg][gcrb] = cr_fmod3[gcrb];
				assign q_fmod1_g[gcrg][gcrb]  = q_fmod1[gcrb];
				assign q_fmod2_g[gcrg][gcrb]  = q_fmod2[gcrb];
				assign q_fmod3_g[gcrg][gcrb]  = q_fmod3[gcrb];
`endif
			end
		end
	endgenerate
	(* dont_touch = "true" *) wire [31:0] a_com1 = a_cval1_r;
	(* dont_touch = "true" *) wire [31:0] a_com2 = a_cval2_r;
	(* dont_touch = "true" *) wire [31:0] a_com3 = a_cval3_r;
	wire [DW-1:0] a_early_va_comb, a_early_vb_comb, a_early_vc_comb;

	wire [2:0] a_k1_pre = opk(a_n1, q_info[ATG_WI_KIND +: 2]);
	wire [2:0] a_k2_pre = opk(a_n2, q_info[ATG_WI_KIND +: 2]);
	wire [2:0] a_k3_pre = opk(a_n3, q_info[ATG_WI_KIND +: 2]);

	wire [2:0] a_k1_pre_0, a_k1_pre_1, a_k2_pre_0, a_k2_pre_1, a_k3_pre_0, a_k3_pre_1;
`ifdef SYNTH_KINTEX7
	genvar gkp;
	generate
		for (gkp = 0; gkp < 3; gkp = gkp + 1) begin : g_akpre
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak10 (.I0(a_k1_pre[gkp]), .O(a_k1_pre_0[gkp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak11 (.I0(a_k1_pre[gkp]), .O(a_k1_pre_1[gkp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak20 (.I0(a_k2_pre[gkp]), .O(a_k2_pre_0[gkp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak21 (.I0(a_k2_pre[gkp]), .O(a_k2_pre_1[gkp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak30 (.I0(a_k3_pre[gkp]), .O(a_k3_pre_0[gkp]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ak31 (.I0(a_k3_pre[gkp]), .O(a_k3_pre_1[gkp]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gkp;
	generate
		for (gkp = 0; gkp < 3; gkp = gkp + 1) begin : g_akpre
			assign a_k1_pre_0[gkp] = a_k1_pre[gkp];
			assign a_k1_pre_1[gkp] = a_k1_pre[gkp];
			assign a_k2_pre_0[gkp] = a_k2_pre[gkp];
			assign a_k2_pre_1[gkp] = a_k2_pre[gkp];
			assign a_k3_pre_0[gkp] = a_k3_pre[gkp];
			assign a_k3_pre_1[gkp] = a_k3_pre[gkp];
		end
	endgenerate
`else
	assign a_k1_pre_0 = a_k1_pre;
	assign a_k1_pre_1 = a_k1_pre;
	assign a_k2_pre_0 = a_k2_pre;
	assign a_k2_pre_1 = a_k2_pre;
	assign a_k3_pre_0 = a_k3_pre;
	assign a_k3_pre_1 = a_k3_pre;
`endif


	wire a_i104_1_pre0 = (a_n1 == 7'd104), a_i104_1_pre1 = (a_n1 == 7'd104);
	wire a_i104_2_pre0 = (a_n2 == 7'd104), a_i104_2_pre1 = (a_n2 == 7'd104);
	wire a_i104_3_pre0 = (a_n3 == 7'd104), a_i104_3_pre1 = (a_n3 == 7'd104);
	wire a_ucr1_pre0   = (a_n1[6:5] == 2'b10), a_ucr1_pre1 = (a_n1[6:5] == 2'b10);
	wire a_ucr2_pre0   = (a_n2[6:5] == 2'b10), a_ucr2_pre1 = (a_n2[6:5] == 2'b10);
	wire a_ucr3_pre0   = (a_n3[6:5] == 2'b10), a_ucr3_pre1 = (a_n3[6:5] == 2'b10);
	genvar lb, lbh;
	generate
		for (lb = 0; lb < NPHYS; lb = lb + 1) begin : g_aearly
			wire [4:0] a_ln = ((lb < (NPHYS/2)) ? a_grp_base_e0[4:0] : a_grp_base_e1[4:0]) | lb[4:0];
			wire [3:0] a_unit_l = (lb < (NPHYS/2)) ? a_unit_e0 : a_unit_e1;
			wire [5:0] a_mods_l = (lb < (NPHYS/2)) ? a_mods_e0 : a_mods_e1;
			(* dont_touch = "true" *) wire a_fp_l = (a_unit_l == ATG_UNIT_FPU) || (a_unit_l == ATG_UNIT_XU);
			(* dont_touch = "true" *) wire a_pk_l = (a_unit_l == ATG_UNIT_PK);
			wire [2:0] a_k1_in = (lb < (NPHYS/2)) ? a_k1_pre_0 : a_k1_pre_1;
			wire [2:0] a_k2_in = (lb < (NPHYS/2)) ? a_k2_pre_0 : a_k2_pre_1;
			wire [2:0] a_k3_in = (lb < (NPHYS/2)) ? a_k3_pre_0 : a_k3_pre_1;
			wire a_i104_1_w, a_i104_2_w, a_i104_3_w;
			wire a_ucr1_w, a_ucr2_w, a_ucr3_w;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ai104_1 (.I0((lb < (NPHYS/2)) ? a_i104_1_pre0 : a_i104_1_pre1), .O(a_i104_1_w));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ai104_2 (.I0((lb < (NPHYS/2)) ? a_i104_2_pre0 : a_i104_2_pre1), .O(a_i104_2_w));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ai104_3 (.I0((lb < (NPHYS/2)) ? a_i104_3_pre0 : a_i104_3_pre1), .O(a_i104_3_w));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aucr_1  (.I0((lb < (NPHYS/2)) ? a_ucr1_pre0   : a_ucr1_pre1),   .O(a_ucr1_w));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aucr_2  (.I0((lb < (NPHYS/2)) ? a_ucr2_pre0   : a_ucr2_pre1),   .O(a_ucr2_w));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aucr_3  (.I0((lb < (NPHYS/2)) ? a_ucr3_pre0   : a_ucr3_pre1),   .O(a_ucr3_w));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign a_i104_1_w = (lb < (NPHYS/2)) ? a_i104_1_pre0 : a_i104_1_pre1;
			assign a_i104_2_w = (lb < (NPHYS/2)) ? a_i104_2_pre0 : a_i104_2_pre1;
			assign a_i104_3_w = (lb < (NPHYS/2)) ? a_i104_3_pre0 : a_i104_3_pre1;
			assign a_ucr1_w = (lb < (NPHYS/2)) ? a_ucr1_pre0   : a_ucr1_pre1;
			assign a_ucr2_w = (lb < (NPHYS/2)) ? a_ucr2_pre0   : a_ucr2_pre1;
			assign a_ucr3_w = (lb < (NPHYS/2)) ? a_ucr3_pre0   : a_ucr3_pre1;
`else
			assign a_i104_1_w = (lb < (NPHYS/2)) ? a_i104_1_pre0 : a_i104_1_pre1;
			assign a_i104_2_w = (lb < (NPHYS/2)) ? a_i104_2_pre0 : a_i104_2_pre1;
			assign a_i104_3_w = (lb < (NPHYS/2)) ? a_i104_3_pre0 : a_i104_3_pre1;
			assign a_ucr1_w   = (lb < (NPHYS/2)) ? a_ucr1_pre0   : a_ucr1_pre1;
			assign a_ucr2_w   = (lb < (NPHYS/2)) ? a_ucr2_pre0   : a_ucr2_pre1;
			assign a_ucr3_w   = (lb < (NPHYS/2)) ? a_ucr3_pre0   : a_ucr3_pre1;
`endif
			wire [31:0] a_op1_nk, a_op2_nk, a_op3_nk;
			wire [31:0] a_early_va_nk = fmod(a_op1_nk, a_mods_l[0], a_mods_l[1], a_fp_l, a_pk_l);
			wire [31:0] a_early_vb_nk = fmod(a_op2_nk, a_mods_l[2], a_mods_l[3], a_fp_l, a_pk_l);
			wire [31:0] a_early_vc_nk = fmod(a_op3_nk, a_mods_l[4], a_mods_l[5], a_fp_l, a_pk_l);
			wire a_k31_1 = !((a_fp_l || a_pk_l) && a_mods_l[1]) ^ ((a_fp_l || a_pk_l) && a_mods_l[0]);
			wire a_k15_1 = !(a_pk_l && a_mods_l[1]) ^ (a_pk_l && a_mods_l[0]);
			wire a_k31_2 = !((a_fp_l || a_pk_l) && a_mods_l[3]) ^ ((a_fp_l || a_pk_l) && a_mods_l[2]);
			wire a_k15_2 = !(a_pk_l && a_mods_l[3]) ^ (a_pk_l && a_mods_l[2]);
			wire a_k31_3 = !((a_fp_l || a_pk_l) && a_mods_l[5]) ^ ((a_fp_l || a_pk_l) && a_mods_l[4]);
			wire a_k15_3 = !(a_pk_l && a_mods_l[5]) ^ (a_pk_l && a_mods_l[4]);
			for (lbh = 0; lbh < 2; lbh = lbh + 1) begin : g_b_early_h
				(* dont_touch = "true" *) reg a_s1_1_lo_r, a_s1_2_lo_r, a_s1_3_lo_r, a_s1_5_lo_r, a_s1_0_lo_r, a_s1_q_lo_r, a_s1_cr_lo_r;
				(* dont_touch = "true" *) reg a_s1_1_hi_r, a_s1_2_hi_r, a_s1_3_hi_r, a_s1_5_hi_r, a_s1_0_hi_r, a_s1_q_hi_r, a_s1_cr_hi_r;
				(* dont_touch = "true" *) reg a_s2_1_lo_r, a_s2_2_lo_r, a_s2_3_lo_r, a_s2_5_lo_r, a_s2_0_lo_r, a_s2_q_lo_r, a_s2_cr_lo_r;
				(* dont_touch = "true" *) reg a_s2_1_hi_r, a_s2_2_hi_r, a_s2_3_hi_r, a_s2_5_hi_r, a_s2_0_hi_r, a_s2_q_hi_r, a_s2_cr_hi_r;
				(* dont_touch = "true" *) reg a_s3_2_lo_r, a_s3_3_lo_r, a_s3_5_lo_r, a_s3_0_lo_r, a_s3_q_lo_r, a_s3_cr_lo_r;
				(* dont_touch = "true" *) reg a_s3_2_hi_r, a_s3_3_hi_r, a_s3_5_hi_r, a_s3_0_hi_r, a_s3_q_hi_r, a_s3_cr_hi_r;
				reg a_k1_lo_is4_r, a_k1_hi_is4_r;
				reg a_k2_lo_is4_r, a_k2_hi_is4_r;
				reg a_k3_lo_is4_r, a_k3_hi_is4_r;
				wire a_k_we_raw = a_pre_k_en[(lb * 4) / NPHYS];
				wire a_k_we_lo, a_k_we_hi;
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_akw_lo (.I0(a_k_we_raw), .O(a_k_we_lo));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_akw_hi (.I0(a_k_we_raw), .O(a_k_we_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign a_k_we_lo = a_k_we_raw;
				assign a_k_we_hi = a_k_we_raw;
`else
				assign a_k_we_lo = a_k_we_raw;
				assign a_k_we_hi = a_k_we_raw;
`endif

				always @(posedge clk or posedge rst_p) begin
					if (rst_p) begin
						a_s1_1_lo_r <= 1'b0; a_s1_2_lo_r <= 1'b0; a_s1_3_lo_r <= 1'b0; a_s1_5_lo_r <= 1'b0; a_s1_0_lo_r <= 1'b0; a_s1_q_lo_r <= 1'b0; a_s1_cr_lo_r <= 1'b0;
						a_s1_1_hi_r <= 1'b0; a_s1_2_hi_r <= 1'b0; a_s1_3_hi_r <= 1'b0; a_s1_5_hi_r <= 1'b0; a_s1_0_hi_r <= 1'b0; a_s1_q_hi_r <= 1'b0; a_s1_cr_hi_r <= 1'b0;
						a_s2_1_lo_r <= 1'b0; a_s2_2_lo_r <= 1'b0; a_s2_3_lo_r <= 1'b0; a_s2_5_lo_r <= 1'b0; a_s2_0_lo_r <= 1'b0; a_s2_q_lo_r <= 1'b0; a_s2_cr_lo_r <= 1'b0;
						a_s2_1_hi_r <= 1'b0; a_s2_2_hi_r <= 1'b0; a_s2_3_hi_r <= 1'b0; a_s2_5_hi_r <= 1'b0; a_s2_0_hi_r <= 1'b0; a_s2_q_hi_r <= 1'b0; a_s2_cr_hi_r <= 1'b0;
						a_s3_2_lo_r <= 1'b0; a_s3_3_lo_r <= 1'b0; a_s3_5_lo_r <= 1'b0; a_s3_0_lo_r <= 1'b0; a_s3_q_lo_r <= 1'b0; a_s3_cr_lo_r <= 1'b0;
						a_s3_2_hi_r <= 1'b0; a_s3_3_hi_r <= 1'b0; a_s3_5_hi_r <= 1'b0; a_s3_0_hi_r <= 1'b0; a_s3_q_hi_r <= 1'b0; a_s3_cr_hi_r <= 1'b0;
						a_k1_lo_is4_r <= 1'b0; a_k1_hi_is4_r <= 1'b0;
						a_k2_lo_is4_r <= 1'b0; a_k2_hi_is4_r <= 1'b0;
						a_k3_lo_is4_r <= 1'b0; a_k3_hi_is4_r <= 1'b0;
					end else begin
						if (a_k_we_lo) begin
							a_s1_1_lo_r  <= (a_k1_in == 3'd1);
							a_s1_2_lo_r  <= (a_k1_in == 3'd2);
							a_s1_3_lo_r  <= (a_k1_in == 3'd3);
							a_s1_5_lo_r  <= (a_k1_in == 3'd5);
							a_s1_0_lo_r  <= (a_k1_in == 3'd0) && !a_i104_1_w && !a_ucr1_w;
							a_s1_q_lo_r  <= a_i104_1_w;
							a_s1_cr_lo_r <= a_ucr1_w;
							a_s2_1_lo_r  <= (a_k2_in == 3'd1);
							a_s2_2_lo_r  <= (a_k2_in == 3'd2);
							a_s2_3_lo_r  <= (a_k2_in == 3'd3);
							a_s2_5_lo_r  <= (a_k2_in == 3'd5);
							a_s2_0_lo_r  <= (a_k2_in == 3'd0) && !a_i104_2_w && !a_ucr2_w;
							a_s2_q_lo_r  <= a_i104_2_w;
							a_s2_cr_lo_r <= a_ucr2_w;
							a_s3_2_lo_r  <= (a_k3_in == 3'd2);
							a_s3_3_lo_r  <= (a_k3_in == 3'd3);
							a_s3_5_lo_r  <= (a_k3_in == 3'd5);
							a_s3_0_lo_r  <= (a_k3_in == 3'd0) && !a_i104_3_w && !a_ucr3_w;
							a_s3_q_lo_r  <= a_i104_3_w;
							a_s3_cr_lo_r <= a_ucr3_w;
							a_k1_lo_is4_r <= (a_k1_in == 3'd4);
							a_k2_lo_is4_r <= (a_k2_in == 3'd4);
							a_k3_lo_is4_r <= (a_k3_in == 3'd4);
						end
						if (a_k_we_hi) begin
							a_s1_1_hi_r  <= (a_k1_in == 3'd1);
							a_s1_2_hi_r  <= (a_k1_in == 3'd2);
							a_s1_3_hi_r  <= (a_k1_in == 3'd3);
							a_s1_5_hi_r  <= (a_k1_in == 3'd5);
							a_s1_0_hi_r  <= (a_k1_in == 3'd0) && !a_i104_1_w && !a_ucr1_w;
							a_s1_q_hi_r  <= a_i104_1_w;
							a_s1_cr_hi_r <= a_ucr1_w;
							a_s2_1_hi_r  <= (a_k2_in == 3'd1);
							a_s2_2_hi_r  <= (a_k2_in == 3'd2);
							a_s2_3_hi_r  <= (a_k2_in == 3'd3);
							a_s2_5_hi_r  <= (a_k2_in == 3'd5);
							a_s2_0_hi_r  <= (a_k2_in == 3'd0) && !a_i104_2_w && !a_ucr2_w;
							a_s2_q_hi_r  <= a_i104_2_w;
							a_s2_cr_hi_r <= a_ucr2_w;
							a_s3_2_hi_r  <= (a_k3_in == 3'd2);
							a_s3_3_hi_r  <= (a_k3_in == 3'd3);
							a_s3_5_hi_r  <= (a_k3_in == 3'd5);
							a_s3_0_hi_r  <= (a_k3_in == 3'd0) && !a_i104_3_w && !a_ucr3_w;
							a_s3_q_hi_r  <= a_i104_3_w;
							a_s3_cr_hi_r <= a_ucr3_w;
							a_k1_hi_is4_r <= (a_k1_in == 3'd4);
							a_k2_hi_is4_r <= (a_k2_in == 3'd4);
							a_k3_hi_is4_r <= (a_k3_in == 3'd4);
						end
					end
				end
				wire a_kl1_lo = a_kill_g[lb] && a_k1_lo_is4_r;
				wire a_kl1_hi = a_kill_g[lb] && a_k1_hi_is4_r;
				wire a_kl2_lo = a_kill_g[lb] && a_k2_lo_is4_r;
				wire a_kl2_hi = a_kill_g[lb] && a_k2_hi_is4_r;
				wire a_kl3_lo = a_kill_g[lb] && a_k3_lo_is4_r;
				wire a_kl3_hi = a_kill_g[lb] && a_k3_hi_is4_r;
				wire [15:0] a_ln_h = (lbh == 0) ? {11'd0, a_ln} : 16'd0;
				wire [15:0] a_ln_1, a_ln_2, a_ln_3;
`ifdef SYNTH_KINTEX7
				if (lbh == 0) begin : g_aln_buf
					genvar glnb;
					for (glnb = 0; glnb < 5; glnb = glnb + 1) begin : g_alnb
						(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aln1 (.I0(a_ln[glnb]), .O(a_ln_1[glnb]));
						(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aln2 (.I0(a_ln[glnb]), .O(a_ln_2[glnb]));
						(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_aln3 (.I0(a_ln[glnb]), .O(a_ln_3[glnb]));
					end
					assign a_ln_1[15:5] = 11'd0;
					assign a_ln_2[15:5] = 11'd0;
					assign a_ln_3[15:5] = 11'd0;
				end else begin : g_aln_z
					assign a_ln_1 = 16'd0;
					assign a_ln_2 = 16'd0;
					assign a_ln_3 = 16'd0;
				end
`elsif SYNTH_KINTEX_ULTRASCALE
				if (lbh == 0) begin : g_aln_buf
					genvar glnb;
					for (glnb = 0; glnb < 5; glnb = glnb + 1) begin : g_alnb
						assign a_ln_1[glnb] = a_ln[glnb];
						assign a_ln_2[glnb] = a_ln[glnb];
						assign a_ln_3[glnb] = a_ln[glnb];
					end
					assign a_ln_1[15:5] = 11'd0;
					assign a_ln_2[15:5] = 11'd0;
					assign a_ln_3[15:5] = 11'd0;
				end else begin : g_aln_z
					assign a_ln_1 = 16'd0;
					assign a_ln_2 = 16'd0;
					assign a_ln_3 = 16'd0;
				end
`else
				assign a_ln_1 = a_ln_h;
				assign a_ln_2 = a_ln_h;
				assign a_ln_3 = a_ln_h;
`endif

				wire [7:0] a_op1_lo_pre =
					(a_s1_1_lo_r ? hold_r1[lb*32 + lbh*16 +: 8] : 8'd0) |
					(a_s1_2_lo_r ? rq_data[lb*32 + lbh*16 +: 8] : 8'd0) |
					(a_s1_3_lo_r ? a_ln_1[0 +: 8]               : 8'd0) |
					(a_s1_5_lo_r ? a_cl_r[lb*32 + lbh*16 +: 8]  : 8'd0) |
					(a_s1_0_lo_r ? a_com1[lbh*16 +: 8]          : 8'd0);
				wire [7:0] a_op1_hi_pre =
					(a_s1_1_hi_r ? hold_r1[lb*32 + lbh*16 + 8 +: 8] : 8'd0) |
					(a_s1_2_hi_r ? rq_data[lb*32 + lbh*16 + 8 +: 8] : 8'd0) |
					(a_s1_3_hi_r ? a_ln_1[8 +: 8]                   : 8'd0) |
					(a_s1_5_hi_r ? a_cl_r[lb*32 + lbh*16 + 8 +: 8]  : 8'd0) |
					(a_s1_0_hi_r ? a_com1[lbh*16 + 8 +: 8]          : 8'd0);
				assign a_op1_nk[lbh*16 +: 8]     = a_op1_lo_pre;
				assign a_op1_nk[lbh*16 + 8 +: 8] = a_op1_hi_pre;
				assign a_early_va_comb[lb*32 + lbh*16 +: 8]     = a_s1_cr_lo_r ? cr_fmod1_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_s1_q_lo_r ? q_fmod1_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_kl1_lo ? 8'hFF : a_early_va_nk[lbh*16 +: 8]));
				assign a_early_va_comb[lb*32 + lbh*16 + 8 +: 8] = a_s1_cr_hi_r ? cr_fmod1_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_s1_q_hi_r ? q_fmod1_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_kl1_hi ? {(lbh == 0) ? a_k15_1 : a_k31_1, 7'h7F} : a_early_va_nk[lbh*16 + 8 +: 8]));

				wire [7:0] a_op2_lo_pre =
					(a_s2_1_lo_r ? hold_r2[lb*32 + lbh*16 +: 8] : 8'd0) |
					(a_s2_2_lo_r ? rq_data[lb*32 + lbh*16 +: 8] : 8'd0) |
					(a_s2_3_lo_r ? a_ln_2[0 +: 8]               : 8'd0) |
					(a_s2_5_lo_r ? a_cl_r[lb*32 + lbh*16 +: 8]  : 8'd0) |
					(a_s2_0_lo_r ? a_com2[lbh*16 +: 8]          : 8'd0);
				wire [7:0] a_op2_hi_pre =
					(a_s2_1_hi_r ? hold_r2[lb*32 + lbh*16 + 8 +: 8] : 8'd0) |
					(a_s2_2_hi_r ? rq_data[lb*32 + lbh*16 + 8 +: 8] : 8'd0) |
					(a_s2_3_hi_r ? a_ln_2[8 +: 8]                   : 8'd0) |
					(a_s2_5_hi_r ? a_cl_r[lb*32 + lbh*16 + 8 +: 8]  : 8'd0) |
					(a_s2_0_hi_r ? a_com2[lbh*16 + 8 +: 8]          : 8'd0);
				assign a_op2_nk[lbh*16 +: 8]     = a_op2_lo_pre;
				assign a_op2_nk[lbh*16 + 8 +: 8] = a_op2_hi_pre;
				assign a_early_vb_comb[lb*32 + lbh*16 +: 8]     = a_s2_cr_lo_r ? cr_fmod2_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_s2_q_lo_r ? q_fmod2_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_kl2_lo ? 8'hFF : a_early_vb_nk[lbh*16 +: 8]));
				assign a_early_vb_comb[lb*32 + lbh*16 + 8 +: 8] = a_s2_cr_hi_r ? cr_fmod2_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_s2_q_hi_r ? q_fmod2_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_kl2_hi ? {(lbh == 0) ? a_k15_2 : a_k31_2, 7'h7F} : a_early_vb_nk[lbh*16 + 8 +: 8]));

				wire [7:0] a_op3_lo_pre =
					(a_s3_2_lo_r ? rq_data[lb*32 + lbh*16 +: 8] : 8'd0) |
					(a_s3_3_lo_r ? a_ln_3[0 +: 8]               : 8'd0) |
					(a_s3_5_lo_r ? a_cl_r[lb*32 + lbh*16 +: 8]  : 8'd0) |
					(a_s3_0_lo_r ? a_com3[lbh*16 +: 8]          : 8'd0);
				wire [7:0] a_op3_hi_pre =
					(a_s3_2_hi_r ? rq_data[lb*32 + lbh*16 + 8 +: 8] : 8'd0) |
					(a_s3_3_hi_r ? a_ln_3[8 +: 8]                   : 8'd0) |
					(a_s3_5_hi_r ? a_cl_r[lb*32 + lbh*16 + 8 +: 8]  : 8'd0) |
					(a_s3_0_hi_r ? a_com3[lbh*16 + 8 +: 8]          : 8'd0);
				assign a_op3_nk[lbh*16 +: 8]     = a_op3_lo_pre;
				assign a_op3_nk[lbh*16 + 8 +: 8] = a_op3_hi_pre;
				assign a_early_vc_comb[lb*32 + lbh*16 +: 8]     = a_s3_cr_lo_r ? cr_fmod3_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_s3_q_lo_r ? q_fmod3_g[(lb * 4) / NPHYS][lbh*16 +: 8]     : (a_kl3_lo ? 8'hFF : a_early_vc_nk[lbh*16 +: 8]));
				assign a_early_vc_comb[lb*32 + lbh*16 + 8 +: 8] = a_s3_cr_hi_r ? cr_fmod3_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_s3_q_hi_r ? q_fmod3_g[(lb * 4) / NPHYS][lbh*16 + 8 +: 8] : (a_kl3_hi ? {(lbh == 0) ? a_k15_3 : a_k31_3, 7'h7F} : a_early_vc_nk[lbh*16 + 8 +: 8]));

				(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] b_va_h, b_vb_h, b_vc_h;
				(* shreg_extract = "no", dont_touch = "true" *) reg [15:0] hold_r1_h, hold_r2_h;
				wire b_va_we_h, b_vb_we_h, b_vc_we_h;
				wire hold1_we_h, hold2_we_h;
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bvaw (.I0(a_stgb_en_g[lb]), .O(b_va_we_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bvbw (.I0(a_stgb_en_g[lb]), .O(b_vb_we_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bvcw (.I0(a_stgb_en_g[lb]), .O(b_vc_we_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hd1w (.I0(a_hold_en_g[lb]), .O(hold1_we_h));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hd2w (.I0(a_hold_en_g[lb]), .O(hold2_we_h));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign b_va_we_h = a_stgb_en_g[lb];
				assign b_vb_we_h = a_stgb_en_g[lb];
				assign b_vc_we_h = a_stgb_en_g[lb];
				assign hold1_we_h = a_hold_en_g[lb];
				assign hold2_we_h = a_hold_en_g[lb];
`else
				assign b_va_we_h = a_stgb_en_g[lb];
				assign b_vb_we_h = a_stgb_en_g[lb];
				assign b_vc_we_h = a_stgb_en_g[lb];
				assign hold1_we_h = a_hold_en_g[lb];
				assign hold2_we_h = a_hold_en_g[lb];
`endif

				always @(posedge clk or posedge rst_p) begin
					if (rst_p) begin
						b_va_h <= 16'd0;
						b_vb_h <= 16'd0;
						b_vc_h <= 16'd0;
						hold_r1_h <= 16'd0;
						hold_r2_h <= 16'd0;
					end else begin
						if (b_va_we_h)
							b_va_h <= a_early_va_comb[lb*32 + lbh*16 +: 16];
						if (b_vb_we_h)
							b_vb_h <= a_early_vb_comb[lb*32 + lbh*16 +: 16];
						if (b_vc_we_h)
							b_vc_h <= a_early_vc_comb[lb*32 + lbh*16 +: 16];
						if (hold1_we_h)
							hold_r1_h <= r_data[lb*32 + lbh*16 +: 16];
						if (hold2_we_h)
							hold_r2_h <= r_data[DW + lb*32 + lbh*16 +: 16];
					end
				end
				assign b_early_va[lb*32 + lbh*16 +: 16] = b_va_h;
				assign b_early_vb[lb*32 + lbh*16 +: 16] = b_vb_h;
				assign b_early_vc[lb*32 + lbh*16 +: 16] = b_vc_h;
				assign hold_r1[lb*32 + lbh*16 +: 16] = hold_r1_h;
				assign hold_r2[lb*32 + lbh*16 +: 16] = hold_r2_h;
			end
		end
	endgenerate

	wire [7:0] a_cval1_next_lo_b0_l = opc_spec_lo_b0(a_n1_opc_0,  a_warp, q_info, qi_front_1a);
	wire [7:0] a_cval1_next_lo_b0_h = opc_spec_lo_b0(a_n1_opc_0h, a_warp, q_info, qi_front_1a);
	wire [7:0] a_cval1_next_lo_b0   = {a_cval1_next_lo_b0_h[7:4], a_cval1_next_lo_b0_l[3:0]};
	wire [7:0] a_cval1_next_lo_b1 = opc_spec_lo_b1(a_n1_opc_1, a_warp, q_info, qi_front_1a);
	wire [7:0] a_cval1_next_hi_b0 = opc_spec_hi_b0(a_n1_opc_2, a_warp, q_info, qi_front_1b);
	wire [7:0] a_cval1_next_hi_b1 = opc_spec_hi_b1(a_n1_opc_3, a_warp, q_info, qi_front_1b);
	wire [15:0] a_cval1_next_lo = {a_cval1_next_lo_b1, a_cval1_next_lo_b0};
	wire [15:0] a_cval1_next_hi = {a_cval1_next_hi_b1, a_cval1_next_hi_b0};

	wire [7:0] a_cval2_next_lo_b0_l = opc_spec_lo_b0(a_n2_opc_0,  a_warp, q_info, qi_front_2a);
	wire [7:0] a_cval2_next_lo_b0_h = opc_spec_lo_b0(a_n2_opc_0h, a_warp, q_info, qi_front_2a);
	wire [7:0] a_cval2_next_lo_b0   = {a_cval2_next_lo_b0_h[7:4], a_cval2_next_lo_b0_l[3:0]};
	wire [7:0] a_cval2_next_lo_b1 = opc_spec_lo_b1(a_n2_opc_1, a_warp, q_info, qi_front_2a);
	wire [7:0] a_cval2_next_hi_b0 = opc_spec_hi_b0(a_n2_opc_2, a_warp, q_info, qi_front_2b);
	wire [7:0] a_cval2_next_hi_b1 = opc_spec_hi_b1(a_n2_opc_3, a_warp, q_info, qi_front_2b);
	wire [15:0] a_cval2_next_lo = {a_cval2_next_lo_b1, a_cval2_next_lo_b0};
	wire [15:0] a_cval2_next_hi = {a_cval2_next_hi_b1, a_cval2_next_hi_b0};

	wire [7:0] a_cval3_next_lo_b0_l = opc_spec_lo_b0(a_n3_opc_0,  a_warp, q_info, qi_front_3a);
	wire [7:0] a_cval3_next_lo_b0_h = opc_spec_lo_b0(a_n3_opc_0h, a_warp, q_info, qi_front_3a);
	wire [7:0] a_cval3_next_lo_b0   = {a_cval3_next_lo_b0_h[7:4], a_cval3_next_lo_b0_l[3:0]};
	wire [7:0] a_cval3_next_lo_b1 = opc_spec_lo_b1(a_n3_opc_1, a_warp, q_info, qi_front_3a);
	wire [7:0] a_cval3_next_hi_b0 = opc_spec_hi_b0(a_n3_opc_2, a_warp, q_info, qi_front_3b);
	wire [7:0] a_cval3_next_hi_b1 = opc_spec_hi_b1(a_n3_opc_3, a_warp, q_info, qi_front_3b);
	wire [15:0] a_cval3_next_lo = {a_cval3_next_lo_b1, a_cval3_next_lo_b0};
	wire [15:0] a_cval3_next_hi = {a_cval3_next_hi_b1, a_cval3_next_hi_b0};
	wire [31:0] a_cval1_next = {a_cval1_next_hi, a_cval1_next_lo};
	wire [31:0] a_cval2_next = {a_cval2_next_hi, a_cval2_next_lo};
	wire [31:0] a_cval3_next = {a_cval3_next_hi, a_cval3_next_lo};
	wire [UOP_W-1:0] b_uop_next = (REG_IO != 0) ?
		(a_uop | ({{(UOP_W-1){1'b0}}, a_pass_out} << (ATG_UOP_OP + ATG_TEX_PASS2))) :
		a_uop;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			a_cval1_r <= 32'd0;
			a_cval2_r <= 32'd0;
			a_cval3_r <= 32'd0;
			b_rdy_oh_r <= 13'd0;
			b_valid_last_r <= 1'b0;
			a_act <= 1'b0;
			b_valid <= 1'b0;
			b_unit_r <= 4'd0;
			b_sg_last_r <= 1'b0;
			a_warp <= {WARP_W{1'b0}};
			a_rf_warp_r <= {WARP_W{1'b0}};
			a_pc <= 16'd0;
			a_uop <= {UOP_W{1'b0}};
			a_sbe <= {(SBE_W+1){1'b0}};
			a_grp <= {GRP_W{1'b0}};
			a_grp_oh_r <= {{(G-1){1'b0}}, 1'b1};
			a_rf_grp_r <= {GRP_W{1'b0}};
			a_step <= 1'b0;
			a_pass_ctl <= 1'b0;
			a_pass_r1_lo <= 1'b0;
			a_pass_r1_hi <= 1'b0;
			a_pass_out <= 1'b0;
			a_wait <= 1'b0;
			a_wait_last <= 1'b0;
			a_act_pre_r <= 16'd0;
			a_wait_pre_r <= 16'd0;
			a_act_ren_r <= 8'd0;
			a_wait_ren_r <= 8'd0;
			a_q_warp_rep_r <= {(WARP_W*4){1'b0}};
			a_rf_r1_r <= 6'd0;
			a_rf_r2_r <= 6'd0;
			a_cr_r3_r <= 5'd0;
			a_rs1_p1_r <= 7'd0;
			a_rs3_p1_r <= 7'd0;
			b_pass <= 1'b0;
			b_step1 <= 1'b0;
			b_n1 <= 7'd0;
			b_n2 <= 7'd0;
			b_n3 <= 7'd0;
			b_is_gpr1 <= 1'b0;
			b_is_gpr2 <= 1'b0;
			b_is_gpr3 <= 1'b0;
			b_hit_rf_1 <= 1'b0;
			b_hit_rf_2 <= 1'b0;
			b_hit_rf_3 <= 1'b0;
			b_hit_alu_ex4_1 <= 1'b0;
			b_hit_alu_ex4_2 <= 1'b0;
			b_hit_alu_ex4_3 <= 1'b0;
			b_hit_alu_ex5_1 <= 1'b0;
			b_hit_alu_ex5_2 <= 1'b0;
			b_hit_alu_ex5_3 <= 1'b0;
			b_last <= 1'b0;
			b_warp <= {WARP_W{1'b0}};
			b_q_info <= 160'd0;
			b_pc <= 16'd0;
			b_uop <= {UOP_W{1'b0}};
			b_uop_pt <= {UOP_W{1'b0}};
			b_uop_fp <= {UOP_W{1'b0}};
			b_uop_ln <= {UOP_W{1'b0}};
			b_sbe <= {(SBE_W+1){1'b0}};
			b_grp <= {GRP_W{1'b0}};
			a_k1_r <= 3'd0;
			a_k2_r <= 3'd0;
			a_k3_r <= 3'd0;
			a_cov_g_r <= {NPHYS{1'b0}};
			a_info_r <= 32'd0;
			b_use_rf1 <= 1'b0;
			b_use_rf2 <= 1'b0;
			b_use_rf3 <= 1'b0;
			b_use_rf1_rep <= {(NPHYS*2){1'b0}};
			b_use_rf2_rep <= {(NPHYS*2){1'b0}};
			b_use_rf3_rep <= {(NPHYS*2){1'b0}};
			b_clr31_1 <= 1'b0; b_inv31_1 <= 1'b0; b_clr15_1 <= 1'b0; b_inv15_1 <= 1'b0;
			b_clr31_2 <= 1'b0; b_inv31_2 <= 1'b0; b_clr15_2 <= 1'b0; b_inv15_2 <= 1'b0;
			b_clr31_3 <= 1'b0; b_inv31_3 <= 1'b0; b_clr15_3 <= 1'b0; b_inv15_3 <= 1'b0;
			b_mask_r <= {(2*NPHYS){1'b0}};
			b_info_r <= 32'd0;
		end else begin
			if (a_pre_we_0) a_cval1_r[15:0]  <= a_cval1_next[15:0];
			if (a_pre_we_1) a_cval1_r[31:16] <= a_cval1_next[31:16];
			if (a_pre_we_2) a_cval2_r[15:0]  <= a_cval2_next[15:0];
			if (a_pre_we_3) a_cval2_r[31:16] <= a_cval2_next[31:16];
			if (a_pre_we_4) a_cval3_r[15:0]  <= a_cval3_next[15:0];
			if (a_pre_we_5) a_cval3_r[31:16] <= a_cval3_next[31:16];

			if (b_adv && !a_wait) begin
				b_rdy_oh_r <= 13'd0;
				b_valid_last_r <= 1'b0;
			end else if (b_ctl_we_0) begin
				b_rdy_oh_r <= {
					(a_unit_rdy == 4'd13),
					(a_last && (a_unit_rdy == ATG_UNIT_LANE) && (a_op == ATG_LANE_SHFL || a_op == ATG_LANE_VOTE || a_op == ATG_LANE_BALLOT)),
					(a_unit_rdy == 4'd11),
					(a_unit_rdy == 4'd10),
					(a_unit_rdy == 4'd9),
					(a_unit_rdy == 4'd8),
					(a_unit_rdy == 4'd7),
					(a_unit_rdy == 4'd6),
					(a_unit_rdy == 4'd5),
					(a_unit_rdy == 4'd4),
					(a_unit_rdy == 4'd3),
					(a_unit_rdy == 4'd2),
					(a_unit_rdy == 4'd1)
				};
				b_valid_last_r <= a_last;
			end

			if (a_pre_we_6) begin
				a_k1_r <= opk(a_n1, q_info[ATG_WI_KIND +: 2]);
				a_k2_r <= opk(a_n2, q_info[ATG_WI_KIND +: 2]);
				a_k3_r <= opk(a_n3, q_info[ATG_WI_KIND +: 2]);
			end
			if (a_pre_we_7a) begin
				a_cov_g_r <= a_cov_g;
			end
			if (a_pre_we_7b) begin
				a_info_r <= {13'd0, q_info[ATG_WI_FRONT], q_info[ATG_WI_KIND +: 2],
					q_info[ATG_WI_SEQ +: 16]};
			end
			if (REG_IO == 0 && b_adv && a_adv && a_step_done) begin
				b_q_info <= q_info;
			end
			if (b_ctl_we_1)  b_pc <= a_pc;
			if (b_ctl_we_2)  b_uop[15:0]  <= b_uop_next[15:0];
			if (b_ctl_we_3)  b_uop[31:16] <= b_uop_next[31:16];
			if (b_ctl_we_4)  b_uop[47:32] <= b_uop_next[47:32];
			if (b_ctl_we_5)  b_uop[63:48] <= b_uop_next[63:48];
			if (b_ctl_we_6)  b_uop[79:64] <= b_uop_next[79:64];
			if (b_ctl_we_7)  b_uop[95:80] <= b_uop_next[95:80];
			if (b_ctl_we_pt[2]) b_uop_pt[15:0]  <= b_uop_next[15:0];
			if (b_ctl_we_pt[3]) b_uop_pt[31:16] <= b_uop_next[31:16];
			if (b_ctl_we_pt[4]) b_uop_pt[47:32] <= b_uop_next[47:32];
			if (b_ctl_we_pt[5]) b_uop_pt[63:48] <= b_uop_next[63:48];
			if (b_ctl_we_pt[6]) b_uop_pt[79:64] <= b_uop_next[79:64];
			if (b_ctl_we_pt[7]) b_uop_pt[95:80] <= b_uop_next[95:80];
			if (b_ctl_we_fp[2]) b_uop_fp[15:0]  <= b_uop_next[15:0];
			if (b_ctl_we_fp[3]) b_uop_fp[31:16] <= b_uop_next[31:16];
			if (b_ctl_we_fp[4]) b_uop_fp[47:32] <= b_uop_next[47:32];
			if (b_ctl_we_fp[5]) b_uop_fp[63:48] <= b_uop_next[63:48];
			if (b_ctl_we_fp[6]) b_uop_fp[79:64] <= b_uop_next[79:64];
			if (b_ctl_we_fp[7]) b_uop_fp[95:80] <= b_uop_next[95:80];
			if (b_ctl_we_ln[2]) b_uop_ln[15:0]  <= b_uop_next[15:0];
			if (b_ctl_we_ln[3]) b_uop_ln[31:16] <= b_uop_next[31:16];
			if (b_ctl_we_ln[4]) b_uop_ln[47:32] <= b_uop_next[47:32];
			if (b_ctl_we_ln[5]) b_uop_ln[63:48] <= b_uop_next[63:48];
			if (b_ctl_we_ln[6]) b_uop_ln[79:64] <= b_uop_next[79:64];
			if (b_ctl_we_ln[7]) b_uop_ln[95:80] <= b_uop_next[95:80];
			if (b_ctl_we_pt[8]) b_uop_pt[UOP_W-1:96] <= b_uop_next[UOP_W-1:96];
			if (b_ctl_we_fp[8]) b_uop_fp[UOP_W-1:96] <= b_uop_next[UOP_W-1:96];
			if (b_ctl_we_ln[8]) b_uop_ln[UOP_W-1:96] <= b_uop_next[UOP_W-1:96];
			if (b_ctl_we_8) begin
				b_uop[UOP_W-1:96] <= b_uop_next[UOP_W-1:96];
				b_warp <= a_warp;
				b_grp <= a_grp;
			end
			if (b_ctl_we_9) begin
				b_unit_r <= a_unit;
				b_sg_last_r <= a_last && (a_unit == ATG_UNIT_LANE) &&
					(a_op == ATG_LANE_SHFL || a_op == ATG_LANE_VOTE || a_op == ATG_LANE_BALLOT);
				b_sbe <= a_sbe;
				b_pass <= a_pass_out;
				b_last <= a_last;
				b_step1 <= a_needs_step1;
				b_use_rf1 <= (a_k1_r == 3'd1) && !a_needs_step1;
				b_use_rf2 <= (a_k2_r == 3'd1) && !a_needs_step1;
				b_use_rf3 <= (a_k3_r == 3'd1);
			end
			if (b_ctl_we_9a) begin
				b_use_rf1_rep <= {(NPHYS*2){(a_k1_r == 3'd1) && !a_needs_step1}};
				b_use_rf2_rep[NPHYS-1:0] <= {(NPHYS){(a_k2_r == 3'd1) && !a_needs_step1}};
			end
			if (b_ctl_we_9b) begin
				b_use_rf2_rep[NPHYS*2-1:NPHYS] <= {(NPHYS){(a_k2_r == 3'd1) && !a_needs_step1}};
				b_use_rf3_rep <= {(NPHYS*2){(a_k3_r == 3'd1)}};
			end
			if (b_ctl_we_10) begin
				b_n1 <= a_n1;
				b_n2 <= a_n2;
				b_is_gpr1 <= a_n1 < 7'd63;
				b_is_gpr2 <= a_n2 < 7'd63;
			end
			if (b_ctl_we_11) begin
				b_n3 <= a_n3;
				b_is_gpr3 <= a_n3 < 7'd63;
				b_mask_r[NPHYS-1:0] <= a_exec_g;
			end
			if (b_ctl_we_12) begin
				b_mask_r[2*NPHYS-1:NPHYS] <= a_exec_g & a_cov_g_r & ~a_kill_g;
				b_clr31_1 <= (a_fp_unit || a_pk_unit) && a_mods[1];
				b_inv31_1 <= (a_fp_unit || a_pk_unit) && a_mods[0];
				b_clr15_1 <= a_pk_unit && a_mods[1];
				b_inv15_1 <= a_pk_unit && a_mods[0];
				b_clr31_2 <= (a_fp_unit || a_pk_unit) && a_mods[3];
				b_inv31_2 <= (a_fp_unit || a_pk_unit) && a_mods[2];
			end
			if (b_ctl_we_13) begin
				b_clr15_2 <= a_pk_unit && a_mods[3];
				b_inv15_2 <= a_pk_unit && a_mods[2];
				b_clr31_3 <= (a_fp_unit || a_pk_unit) && a_mods[5];
				b_inv31_3 <= (a_fp_unit || a_pk_unit) && a_mods[4];
				b_clr15_3 <= a_pk_unit && a_mods[5];
				b_inv15_3 <= a_pk_unit && a_mods[4];
			end
			if (b_ctl_we_14) begin
				b_info_r <= a_info_r;
			end
			if (b_adv_1) begin
				if (a_adv_1) begin
					b_valid <= a_step_done;
					if (a_step_done) begin
						b_hit_rf_1 <= (REG_IO != 0) ? 1'b0 : a_match_wb_1;
						b_hit_rf_2 <= (REG_IO != 0) ? 1'b0 : a_match_wb_2;
						b_hit_rf_3 <= (REG_IO != 0) ? 1'b0 : a_match_wb_3;
						b_hit_alu_ex4_1 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex4_1;
						b_hit_alu_ex4_2 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex4_2;
						b_hit_alu_ex4_3 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex4_3;
						b_hit_alu_ex5_1 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex5_1;
						b_hit_alu_ex5_2 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex5_2;
						b_hit_alu_ex5_3 <= (REG_IO != 0) ? 1'b0 : a_match_alu_ex5_3;
					end
				end else begin
					b_valid <= 1'b0;
					b_hit_rf_1 <= 1'b0;
					b_hit_rf_2 <= 1'b0;
					b_hit_rf_3 <= 1'b0;
					b_hit_alu_ex4_1 <= 1'b0;
					b_hit_alu_ex4_2 <= 1'b0;
					b_hit_alu_ex4_3 <= 1'b0;
					b_hit_alu_ex5_1 <= 1'b0;
					b_hit_alu_ex5_2 <= 1'b0;
					b_hit_alu_ex5_3 <= 1'b0;
				end
			end else begin
				b_hit_alu_ex4_1 <= 1'b0;
				b_hit_alu_ex4_2 <= 1'b0;
				b_hit_alu_ex4_3 <= 1'b0;
				b_hit_alu_ex5_1 <= 1'b0;
				b_hit_alu_ex5_2 <= 1'b0;
				b_hit_alu_ex5_3 <= 1'b0;
			end
			if (REG_IO != 0) begin
				if (a_act_pre_r[15] && !a_wait_pre_r[15] && b_adv_2) begin
					a_wait <= 1'b1;
					a_wait_last <= a_last;
					a_wait_pre_r <= 16'hFFFF;
					a_wait_ren_r <= 8'hFF;
				end else if (a_adv_3) begin
					a_wait <= 1'b0;
					a_wait_last <= 1'b0;
					a_wait_pre_r <= 16'd0;
					a_wait_ren_r <= 8'd0;
				end
			end
			if (a_load_we_0) a_pc <= i_pc;
			if (a_load_we_1) a_uop[15:0] <= i_uop[15:0];
			if (a_load_we_2) a_uop[31:16] <= i_uop[31:16];
			if (a_load_we_3) a_uop[47:32] <= i_uop[47:32];
			if (a_load_we_4) a_uop[63:48] <= i_uop[63:48];
			if (a_load_we_5) a_uop[79:64] <= i_uop[79:64];
			if (a_load_we_6) a_uop[95:80] <= i_uop[95:80];
			if (a_load_we_7) begin
				a_uop[UOP_W-1:96] <= i_uop[UOP_W-1:96];
				a_sbe <= i_sbe;
			end
			if (a_load_we_8a) begin
				a_act_pre_r <= {16{i_valid}};
				a_act_ren_r <= {8{i_valid}};
			end
			if (a_load_we_8) begin
				a_warp <= i_warp_a;
				a_q_warp_rep_r <= {4{i_warp_q}};
				a_rf_warp_r <= i_warp_rf;
				a_act <= i_valid;
			end
			if (a_load_we_9) begin
				a_rf_r1_r <= i_uop[ATG_UOP_RS1 +: 6];
				a_rf_r2_r <= i_uop[ATG_UOP_RS2 +: 6];
				a_rs1_p1_r <= i_rs1_p1;
			end
			if (a_load_we_9a) begin
				a_rs3_p1_r <= i_rs3_p1;
				a_cr_r3_r <= i_uop[ATG_UOP_PAIR] ? i_rs1_p1[4:0] : i_uop[ATG_UOP_RS3 +: 5];
			end
			if (a_adv_2) begin
				if (a_needs_step1 && !a_step) begin
					a_step <= 1'b1;
					a_rf_r1_r <= a_n3[5:0];
					a_rf_r2_r <= 6'd0;
				end else begin
					a_step <= 1'b0;
					if (a_wide && !a_pass_ctl) begin
						a_pass_ctl <= 1'b1;
						a_pass_r1_lo <= 1'b1;
						a_pass_r1_hi <= 1'b1;
						a_pass_out <= 1'b1;
						a_rf_r1_r <= a_uop[ATG_UOP_RS3 +: 6];
						a_rf_r2_r <= a_uop[ATG_UOP_RS2 +: 6];
						a_cr_r3_r <= a_uop[ATG_UOP_PAIR] ? a_rs3_p1_r[4:0] : a_uop[ATG_UOP_RS3 +: 5];
					end else begin
						a_pass_ctl <= 1'b0;
						a_pass_r1_lo <= 1'b0;
			a_pass_r1_hi <= 1'b0;
						a_pass_out <= 1'b0;
						if (a_last) begin
							a_grp <= {GRP_W{1'b0}};
							a_grp_oh_r <= {{(G-1){1'b0}}, 1'b1};
							a_rf_grp_r <= {GRP_W{1'b0}};
						end else begin
							a_grp <= a_grp_next;
							a_grp_oh_r <= a_grp_oh_next;
							a_rf_grp_r <= a_grp_next;
							a_rf_r1_r <= a_uop[ATG_UOP_RS1 +: 6];
							a_rf_r2_r <= a_uop[ATG_UOP_RS2 +: 6];
							a_cr_r3_r <= a_uop[ATG_UOP_PAIR] ? a_rs1_p1_r[4:0] : a_uop[ATG_UOP_RS3 +: 5];
						end
					end
				end
			end
		end
	end

	/* Stage B: operand selection. */
	wire [6:0] mods = b_uop[ATG_UOP_MOD +: 7];
	wire fp_unit = b_unit == ATG_UNIT_FPU || b_unit == ATG_UNIT_XU;
	wire pk_unit = b_unit == ATG_UNIT_PK;
	wire [4:0] grp_base = (G > 1) ? {b_grp, {($clog2(NPHYS)){1'b0}}} : 5'd0;

	reg [NPHYS-1:0] exec_g, kill_g, cov_g;
	integer gi;
	always @(*) begin
		exec_g = {NPHYS{1'b0}};
		kill_g = {NPHYS{1'b0}};
		cov_g  = {NPHYS{1'b0}};
		for (gi = 0; gi < 32/NPHYS; gi = gi + 1) begin
			if (b_grp == gi[GRP_W-1:0]) begin
				exec_g = q_exec[gi*NPHYS +: NPHYS];
				kill_g = q_killed[gi*NPHYS +: NPHYS];
				cov_g  = b_q_info[ATG_WI_MASK + gi*NPHYS +: NPHYS];
			end
		end
	end
	wire [WARP_W-1:0] a_q_warp_buf;

	assign a_q_warp_buf = a_warp;

	assign a_q_warp = a_q_warp_buf;
	assign a_q_warp_rep = a_q_warp_rep_r;
	assign q_warp = (REG_IO != 0) ? a_warp : b_warp;

	/* Stage A special-operand precomputation (only n[4:0] for n in 96..127) */
	function [7:0] opc_spec_lo_b0;
		input [4:0] n_lo;
		input [WARP_W-1:0] wn;
		input [159:0] qi;
		input qf_lo;
		reg [15:0] full;
		begin
			case (n_lo)
			5'd0:  full = 16'd0;
			5'd1:  full = 16'h0000;
			5'd2:  full = 16'd1;
			5'd3:  full = 16'h0000;
			5'd4:  full = 16'h0000;
			5'd5:  full = 16'h0000;
			5'd7:  full = WARP_BASE[15:0] | {{(16-WARP_W){1'b0}}, wn};
			5'd10: full = qi[ATG_WI_WGX +: 16];
			5'd11: full = qi[ATG_WI_WGX + 16 +: 16];
			5'd12: full = qi[ATG_WI_WGX + 32 +: 16];
			5'd13, 5'd16: full = {8'd0, qi[ATG_WI_LID +: 8]};
			5'd18: full = {16{qf_lo}};
			5'd19: full = qi[ATG_WI_NWGX +: 16];
			5'd20: full = qi[ATG_WI_NWGX + 16 +: 16];
			5'd21: full = qi[ATG_WI_NWGX + 32 +: 16];
			5'd22: full = 16'h3C00;
			5'd23: full = qi[ATG_WI_MASK +: 16];
			5'd24: full = (qi[ATG_WI_KIND +: 2] == 2'd0) ? {8'd0, qi[ATG_WI_LID +: 8]} : 16'd0;
			default: full = 16'd0;
			endcase
			opc_spec_lo_b0 = full[7:0];
		end
	endfunction

	function [7:0] opc_spec_lo_b1;
		input [4:0] n_lo;
		input [WARP_W-1:0] wn;
		input [159:0] qi;
		input qf_lo;
		reg [15:0] full;
		begin
			case (n_lo)
			5'd0:  full = 16'd0;
			5'd1:  full = 16'h0000;
			5'd2:  full = 16'd1;
			5'd3:  full = 16'h0000;
			5'd4:  full = 16'h0000;
			5'd5:  full = 16'h0000;
			5'd7:  full = WARP_BASE[15:0] | {{(16-WARP_W){1'b0}}, wn};
			5'd10: full = qi[ATG_WI_WGX +: 16];
			5'd11: full = qi[ATG_WI_WGX + 16 +: 16];
			5'd12: full = qi[ATG_WI_WGX + 32 +: 16];
			5'd13, 5'd16: full = {8'd0, qi[ATG_WI_LID +: 8]};
			5'd18: full = {16{qf_lo}};
			5'd19: full = qi[ATG_WI_NWGX +: 16];
			5'd20: full = qi[ATG_WI_NWGX + 16 +: 16];
			5'd21: full = qi[ATG_WI_NWGX + 32 +: 16];
			5'd22: full = 16'h3C00;
			5'd23: full = qi[ATG_WI_MASK +: 16];
			5'd24: full = (qi[ATG_WI_KIND +: 2] == 2'd0) ? {8'd0, qi[ATG_WI_LID +: 8]} : 16'd0;
			default: full = 16'd0;
			endcase
			opc_spec_lo_b1 = full[15:8];
		end
	endfunction

	function [7:0] opc_spec_hi_b0;
		input [4:0] n_lo;
		input [WARP_W-1:0] wn;
		input [159:0] qi;
		input qf_hi;
		reg [15:0] full;
		begin
			case (n_lo)
			5'd0:  full = 16'd0;
			5'd1:  full = 16'h3F80;
			5'd2:  full = 16'd0;
			5'd3:  full = 16'h3F00;
			5'd4:  full = 16'h4000;
			5'd5:  full = 16'hBF80;
			5'd7:  full = WARP_BASE[31:16];
			5'd10, 5'd11, 5'd12, 5'd13, 5'd16: full = 16'd0;
			5'd18: full = {16{qf_hi}};
			5'd19, 5'd20, 5'd21: full = 16'd0;
			5'd22: full = 16'h3C00;
			5'd23: full = qi[ATG_WI_MASK + 16 +: 16];
			5'd24: full = 16'd0;
			default: full = 16'd0;
			endcase
			opc_spec_hi_b0 = full[7:0];
		end
	endfunction

	function [7:0] opc_spec_hi_b1;
		input [4:0] n_lo;
		input [WARP_W-1:0] wn;
		input [159:0] qi;
		input qf_hi;
		reg [15:0] full;
		begin
			case (n_lo)
			5'd0:  full = 16'd0;
			5'd1:  full = 16'h3F80;
			5'd2:  full = 16'd0;
			5'd3:  full = 16'h3F00;
			5'd4:  full = 16'h4000;
			5'd5:  full = 16'hBF80;
			5'd7:  full = WARP_BASE[31:16];
			5'd10, 5'd11, 5'd12, 5'd13, 5'd16: full = 16'd0;
			5'd18: full = {16{qf_hi}};
			5'd19, 5'd20, 5'd21: full = 16'd0;
			5'd22: full = 16'h3C00;
			5'd23: full = qi[ATG_WI_MASK + 16 +: 16];
			5'd24: full = 16'd0;
			default: full = 16'd0;
			endcase
			opc_spec_hi_b1 = full[15:8];
		end
	endfunction

	/* The value of operand n, split in two: the part that is the same in every
	 * lane (constants, warp and workgroup values, the constant word), made once
	 * per operand, and the lane's own part (its GPR, the rq word, its lane
	 * number, its killed bit), chosen per lane.  Everything they read is an
	 * argument (so that always @(*) sees it). */
	function [31:0] opc;
		input [6:0] n;
		input [31:0] cw;
		input [WARP_W-1:0] wn;
		input [159:0] qi;
		input [31:0] qe;
		begin
			if (n < 7'd96)
				opc = cw;
			else begin
				case (n)
				7'd96: opc = 32'd0;
				7'd97: opc = 32'h3F800000;
				7'd98: opc = 32'd1;
				7'd99: opc = 32'h3F000000;
				7'd100: opc = 32'h40000000;
				7'd101: opc = 32'hBF800000;
				7'd103: opc = WARP_BASE[31:0] | {{(32-WARP_W){1'b0}}, wn};
				7'd104: opc = qe;
				7'd106: opc = {16'd0, qi[ATG_WI_WGX +: 16]};
				7'd107: opc = {16'd0, qi[ATG_WI_WGX + 16 +: 16]};
				7'd108: opc = {16'd0, qi[ATG_WI_WGX + 32 +: 16]};
				7'd109, 7'd112: opc = {24'd0, qi[ATG_WI_LID +: 8]};
				7'd114: opc = {32{qi[ATG_WI_FRONT]}};
				7'd115: opc = {16'd0, qi[ATG_WI_NWGX +: 16]};
				7'd116: opc = {16'd0, qi[ATG_WI_NWGX + 16 +: 16]};
				7'd117: opc = {16'd0, qi[ATG_WI_NWGX + 32 +: 16]};
				7'd118: opc = 32'h3C003C00;
				7'd119: opc = qi[ATG_WI_MASK +: 32];
				7'd120: opc = (qi[ATG_WI_KIND +: 2] == 2'd0) ? {24'd0, qi[ATG_WI_LID +: 8]} : 32'd0;
				7'd121: opc = 32'd0;
				default: opc = 32'd0;
				endcase
			end
		end
	endfunction

	/* How the lane combines the common part: 0 = the common part, 1 = the GPR,
	 * 2 = rq, 3 = the lane number, 4 = the killed bit, 5 = common + lane
	 * (the local index). */
	function [2:0] opk;
		input [6:0] n;
		input [1:0] kind;
		begin
			if (n < 7'd63)
				opk = 3'd1;
			else if (n == 7'd63)
				opk = 3'd2;
			else if (n == 7'd102)
				opk = 3'd3;
			else if (n == 7'd105)
				opk = 3'd4;
			else if (n == 7'd109 || n == 7'd112 || (n == 7'd120 && kind == 2'd0))
				opk = 3'd5;
			else
				opk = 3'd0;
		end
	endfunction

	function [31:0] opl;
		input [2:0] k;
		input [31:0] com, gpr, rq;
		input [4:0] lane;
		input kl;
		input [31:0] com_lane;	/* com + lane */
		begin
			case (k)
			3'd1: opl = gpr;
			3'd2: opl = rq;
			3'd3: opl = {27'd0, lane};
			3'd4: opl = {32{kl}};
			3'd5: opl = com_lane;
			default: opl = com;
			endcase
		end
	endfunction

	/* The float modifiers: abs, then negate.
	 * Only bit 31 (for fp and pk) and bit 15 (for pk) are modified.
	 * Bits [30:16] and [14:0] pass straight through with zero logic. */
	function [31:0] fmod;
		input [31:0] v;
		input ng, ab, fp, pk;
		reg s31, s15;
		begin
			s31 = v[31];
			s15 = v[15];
			if (fp) begin
				if (ab)
					s31 = 1'b0;
				if (ng)
					s31 = !s31;
			end else if (pk) begin
				if (ab) begin
					s31 = 1'b0;
					s15 = 1'b0;
				end
				if (ng) begin
					s31 = !s31;
					s15 = !s15;
				end
			end
			fmod = {s31, v[30:16], s15, v[14:0]};
		end
	endfunction

	wire [DW-1:0] va, vb, vc;

	/* Register to catch RF write-read collision (1-cycle delay from fwd_wb) */
	reg [DW-1:0]     fwd_rf_data;
	reg [NPHYS-1:0]  fwd_rf_lanes;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			fwd_rf_data  <= {DW{1'b0}};
			fwd_rf_lanes <= {NPHYS{1'b0}};
		end else begin
			fwd_rf_data  <= fwd_wb_data;
			fwd_rf_lanes <= fwd_wb_lanes;
		end
	end

	/* The common part and the kind of each operand, once per instruction. */
	wire [31:0] com1 = opc(b_n1, cr_data[31:0], b_warp, b_q_info, q_exec);
	wire [31:0] com2 = opc(b_n2, cr_data[63:32], b_warp, b_q_info, q_exec);
	wire [31:0] com3 = opc(b_n3, cr_data[95:64], b_warp, b_q_info, q_exec);
	wire [2:0] k1 = opk(b_n1, b_q_info[ATG_WI_KIND +: 2]);
	wire [2:0] k2 = opk(b_n2, b_q_info[ATG_WI_KIND +: 2]);
	wire [2:0] k3 = opk(b_n3, b_q_info[ATG_WI_KIND +: 2]);

	/* Operand forwarding comparison for GPRs:
	 * - ALU EX4 is registered (s4_data is flip-flop output), safe to forward back-to-back.
	 * - FPU EX4 is combinational (fpu_res has ~21ns logic delay), completely cut.
	 * - Both ALU and FPU forward from EX5 (registered) and WB/RF.
	 */
	wire hit_alu_ex4_1 = (REG_IO == 0) ?
		(fwd_alu_ex4_valid && b_is_gpr1 && (fwd_alu_ex4_rd < 7'd63) && (fwd_alu_ex4_warp == b_warp) && (fwd_alu_ex4_grp == b_grp) && (fwd_alu_ex4_rd[5:0] == b_n1[5:0])) :
		b_hit_alu_ex4_1;
	wire hit_fpu_ex4_1 = 1'b0;
	wire hit_alu_ex5_1 = (REG_IO == 0) ?
		(fwd_alu_ex5_valid && b_is_gpr1 && (fwd_alu_ex5_rd < 7'd63) && (fwd_alu_ex5_warp == b_warp) && (fwd_alu_ex5_grp == b_grp) && (fwd_alu_ex5_rd[5:0] == b_n1[5:0])) :
		b_hit_alu_ex5_1;
	wire hit_fpu_ex5_1 = (REG_IO == 0) ?
		(fwd_fpu_ex5_valid && b_is_gpr1 && (fwd_fpu_ex5_rd < 7'd63) && (fwd_fpu_ex5_warp == b_warp) && (fwd_fpu_ex5_grp == b_grp) && (fwd_fpu_ex5_rd[5:0] == b_n1[5:0])) :
		1'b0;
	wire hit_wb_1      = (REG_IO == 0) ?
		(fwd_wb_valid && b_is_gpr1 && (fwd_wb_rd < 7'd63) && (fwd_wb_warp == b_warp) && (fwd_wb_grp == b_grp) && (fwd_wb_rd[5:0] == b_n1[5:0])) :
		1'b0;
	wire hit_rf_1      = b_hit_rf_1;

	wire hit_alu_ex4_2 = (REG_IO == 0) ?
		(fwd_alu_ex4_valid && b_is_gpr2 && (fwd_alu_ex4_rd < 7'd63) && (fwd_alu_ex4_warp == b_warp) && (fwd_alu_ex4_grp == b_grp) && (fwd_alu_ex4_rd[5:0] == b_n2[5:0])) :
		b_hit_alu_ex4_2;
	wire hit_fpu_ex4_2 = 1'b0;
	wire hit_alu_ex5_2 = (REG_IO == 0) ?
		(fwd_alu_ex5_valid && b_is_gpr2 && (fwd_alu_ex5_rd < 7'd63) && (fwd_alu_ex5_warp == b_warp) && (fwd_alu_ex5_grp == b_grp) && (fwd_alu_ex5_rd[5:0] == b_n2[5:0])) :
		b_hit_alu_ex5_2;
	wire hit_fpu_ex5_2 = (REG_IO == 0) ?
		(fwd_fpu_ex5_valid && b_is_gpr2 && (fwd_fpu_ex5_rd < 7'd63) && (fwd_fpu_ex5_warp == b_warp) && (fwd_fpu_ex5_grp == b_grp) && (fwd_fpu_ex5_rd[5:0] == b_n2[5:0])) :
		1'b0;
	wire hit_wb_2      = (REG_IO == 0) ?
		(fwd_wb_valid && b_is_gpr2 && (fwd_wb_rd < 7'd63) && (fwd_wb_warp == b_warp) && (fwd_wb_grp == b_grp) && (fwd_wb_rd[5:0] == b_n2[5:0])) :
		1'b0;
	wire hit_rf_2      = b_hit_rf_2;

	wire hit_alu_ex4_3 = (REG_IO == 0) ?
		(fwd_alu_ex4_valid && b_is_gpr3 && (fwd_alu_ex4_rd < 7'd63) && (fwd_alu_ex4_warp == b_warp) && (fwd_alu_ex4_grp == b_grp) && (fwd_alu_ex4_rd[5:0] == b_n3[5:0])) :
		b_hit_alu_ex4_3;
	wire hit_fpu_ex4_3 = 1'b0;
	wire hit_alu_ex5_3 = (REG_IO == 0) ?
		(fwd_alu_ex5_valid && b_is_gpr3 && (fwd_alu_ex5_rd < 7'd63) && (fwd_alu_ex5_warp == b_warp) && (fwd_alu_ex5_grp == b_grp) && (fwd_alu_ex5_rd[5:0] == b_n3[5:0])) :
		b_hit_alu_ex5_3;
	wire hit_fpu_ex5_3 = (REG_IO == 0) ?
		(fwd_fpu_ex5_valid && b_is_gpr3 && (fwd_fpu_ex5_rd < 7'd63) && (fwd_fpu_ex5_warp == b_warp) && (fwd_fpu_ex5_grp == b_grp) && (fwd_fpu_ex5_rd[5:0] == b_n3[5:0])) :
		1'b0;
	wire hit_wb_3      = (REG_IO == 0) ?
		(fwd_wb_valid && b_is_gpr3 && (fwd_wb_rd < 7'd63) && (fwd_wb_warp == b_warp) && (fwd_wb_grp == b_grp) && (fwd_wb_rd[5:0] == b_n3[5:0])) :
		1'b0;
	wire hit_rf_3      = b_hit_rf_3;

	/*
	 * Per lane: its number in the warp (the group's base and l: disjoint
	 * bits), local index (LID + ln, 9-bit adder independent of opc).
	 */
	wire [NPHYS-1:0] cl_co_unused;
	genvar l;
	generate
		for (l = 0; l < NPHYS; l = l + 1) begin : g_lane
			wire [4:0] ln = grp_base[4:0] | l[4:0];
			wire [8:0] cl_s;
			am_add #(.W(9)) u_cl (.a({1'b0, b_q_info[ATG_WI_LID +: 8]}), .b({4'd0, ln}), .ci(1'b0),
				.s(cl_s), .co(cl_co_unused[l]));
			wire [31:0] cl = {23'd0, cl_s};

			/* Early operand paths: evaluated in parallel from register sources */
			wire fwd_hit_1 = (hit_alu_ex4_1 && fwd_alu_ex4_lanes[l]) ||
			                 (hit_alu_ex5_1 && fwd_alu_ex5_lanes[l]) ||
			                 (hit_fpu_ex5_1 && fwd_fpu_ex5_lanes[l]) ||
			                 (hit_wb_1      && fwd_wb_lanes[l]) ||
			                 (hit_rf_1      && fwd_rf_lanes[l]);

			wire [31:0] fwd_data_1 = (hit_alu_ex4_1 && fwd_alu_ex4_lanes[l]) ? fwd_alu_ex4_data[l*32 +: 32] :
			                         (hit_alu_ex5_1 && fwd_alu_ex5_lanes[l]) ? fwd_alu_ex5_data[l*32 +: 32] :
			                         (hit_fpu_ex5_1 && fwd_fpu_ex5_lanes[l]) ? fwd_fpu_ex5_data[l*32 +: 32] :
			                         (hit_wb_1      && fwd_wb_lanes[l]) ? fwd_wb_data[l*32 +: 32] :
			                         fwd_rf_data[l*32 +: 32];

			wire use_rf1 = (k1 == 3'd1) && !fwd_hit_1 && !b_step1;
			wire [31:0] early_gpr1 = fwd_hit_1 ? fwd_data_1 : hold_r1[l*32 +: 32];
			wire [31:0] early_op1 = (k1 == 3'd1) ? early_gpr1 : opl(k1, com1, 32'd0, rq_data[l*32 +: 32], ln, kill_g[l], cl);
			wire [31:0] early_va1 = fmod(early_op1, mods[0], mods[1], fp_unit, pk_unit);
			wire [31:0] rf_va1_0 = fmod(r_data[l*32 +: 32], mods[0], mods[1], fp_unit, pk_unit);
			wire [31:0] rf_va1_r = {(r_data[l*32 + 31] & ~b_clr31_1) ^ b_inv31_1, r_data[l*32 + 16 +: 15],
				(r_data[l*32 + 15] & ~b_clr15_1) ^ b_inv15_1, r_data[l*32 +: 15]};
			wire b_use_rf1_lo, b_use_rf1_hi;
			wire b_use_rf2_lo, b_use_rf2_hi;
			wire b_use_rf3_lo, b_use_rf3_hi;

			assign b_use_rf1_lo = b_use_rf1_rep[2*l];
			assign b_use_rf1_hi = b_use_rf1_rep[2*l + 1];
			assign b_use_rf2_lo = b_use_rf2_rep[2*l];
			assign b_use_rf2_hi = b_use_rf2_rep[2*l + 1];
			assign b_use_rf3_lo = b_use_rf3_rep[2*l];
			assign b_use_rf3_hi = b_use_rf3_rep[2*l + 1];

			assign va[l*32 +: 16] = (REG_IO != 0) ?
				(b_use_rf1_lo ? rf_va1_r[15:0] : b_early_va[l*32 +: 16]) :
				(use_rf1 ? rf_va1_0[15:0] : early_va1[15:0]);
			assign va[l*32 + 16 +: 16] = (REG_IO != 0) ?
				(b_use_rf1_hi ? rf_va1_r[31:16] : b_early_va[l*32 + 16 +: 16]) :
				(use_rf1 ? rf_va1_0[31:16] : early_va1[31:16]);

			wire fwd_hit_2 = (hit_alu_ex4_2 && fwd_alu_ex4_lanes[l]) ||
			                 (hit_alu_ex5_2 && fwd_alu_ex5_lanes[l]) ||
			                 (hit_fpu_ex5_2 && fwd_fpu_ex5_lanes[l]) ||
			                 (hit_wb_2      && fwd_wb_lanes[l]) ||
			                 (hit_rf_2      && fwd_rf_lanes[l]);

			wire [31:0] fwd_data_2 = (hit_alu_ex4_2 && fwd_alu_ex4_lanes[l]) ? fwd_alu_ex4_data[l*32 +: 32] :
			                         (hit_alu_ex5_2 && fwd_alu_ex5_lanes[l]) ? fwd_alu_ex5_data[l*32 +: 32] :
			                         (hit_fpu_ex5_2 && fwd_fpu_ex5_lanes[l]) ? fwd_fpu_ex5_data[l*32 +: 32] :
			                         (hit_wb_2      && fwd_wb_lanes[l]) ? fwd_wb_data[l*32 +: 32] :
			                         fwd_rf_data[l*32 +: 32];

			wire use_rf2 = (k2 == 3'd1) && !fwd_hit_2 && !b_step1;
			wire [31:0] early_gpr2 = fwd_hit_2 ? fwd_data_2 : hold_r2[l*32 +: 32];
			wire [31:0] early_op2 = (k2 == 3'd1) ? early_gpr2 : opl(k2, com2, 32'd0, rq_data[l*32 +: 32], ln, kill_g[l], cl);
			wire [31:0] early_vb2 = fmod(early_op2, mods[2], mods[3], fp_unit, pk_unit);
			wire [31:0] rf_vb2_0 = fmod(r_data[DW + l*32 +: 32], mods[2], mods[3], fp_unit, pk_unit);
			wire [31:0] rf_vb2_r = {(r_data[DW + l*32 + 31] & ~b_clr31_2) ^ b_inv31_2, r_data[DW + l*32 + 16 +: 15],
				(r_data[DW + l*32 + 15] & ~b_clr15_2) ^ b_inv15_2, r_data[DW + l*32 +: 15]};
			assign vb[l*32 +: 16] = (REG_IO != 0) ?
				(b_use_rf2_lo ? rf_vb2_r[15:0] : b_early_vb[l*32 +: 16]) :
				(use_rf2 ? rf_vb2_0[15:0] : early_vb2[15:0]);
			assign vb[l*32 + 16 +: 16] = (REG_IO != 0) ?
				(b_use_rf2_hi ? rf_vb2_r[31:16] : b_early_vb[l*32 + 16 +: 16]) :
				(use_rf2 ? rf_vb2_0[31:16] : early_vb2[31:16]);

			wire fwd_hit_3 = (hit_alu_ex4_3 && fwd_alu_ex4_lanes[l]) ||
			                 (hit_alu_ex5_3 && fwd_alu_ex5_lanes[l]) ||
			                 (hit_fpu_ex5_3 && fwd_fpu_ex5_lanes[l]) ||
			                 (hit_wb_3      && fwd_wb_lanes[l]) ||
			                 (hit_rf_3      && fwd_rf_lanes[l]);

			wire [31:0] fwd_data_3 = (hit_alu_ex4_3 && fwd_alu_ex4_lanes[l]) ? fwd_alu_ex4_data[l*32 +: 32] :
			                         (hit_alu_ex5_3 && fwd_alu_ex5_lanes[l]) ? fwd_alu_ex5_data[l*32 +: 32] :
			                         (hit_fpu_ex5_3 && fwd_fpu_ex5_lanes[l]) ? fwd_fpu_ex5_data[l*32 +: 32] :
			                         (hit_wb_3      && fwd_wb_lanes[l]) ? fwd_wb_data[l*32 +: 32] :
			                         fwd_rf_data[l*32 +: 32];

			wire use_rf3 = (k3 == 3'd1) && !fwd_hit_3;
			wire [31:0] early_op3 = (k3 == 3'd1) ? fwd_data_3 : opl(k3, com3, 32'd0, rq_data[l*32 +: 32], ln, kill_g[l], cl);
			wire [31:0] early_vc3 = fmod(early_op3, mods[4], mods[5], fp_unit, pk_unit);
			wire [31:0] rf_vc3_0 = fmod(r_data[0 + l*32 +: 32], mods[4], mods[5], fp_unit, pk_unit);
			wire [31:0] rf_vc3_r = {(r_data[l*32 + 31] & ~b_clr31_3) ^ b_inv31_3, r_data[l*32 + 16 +: 15],
				(r_data[l*32 + 15] & ~b_clr15_3) ^ b_inv15_3, r_data[l*32 +: 15]};
			assign vc[l*32 +: 16] = (REG_IO != 0) ?
				(b_use_rf3_lo ? rf_vc3_r[15:0] : b_early_vc[l*32 +: 16]) :
				(use_rf3 ? rf_vc3_0[15:0] : early_vc3[15:0]);
			assign vc[l*32 + 16 +: 16] = (REG_IO != 0) ?
				(b_use_rf3_hi ? rf_vc3_r[31:16] : b_early_vc[l*32 + 16 +: 16]) :
				(use_rf3 ? rf_vc3_0[31:16] : early_vc3[31:16]);
		end
	endgenerate

	assign x_valid = b_valid;
	assign x_warp = b_warp;
	assign x_grp = b_grp;
	assign x_last = b_last;
	assign x_sbe = b_sbe;
	assign x_pc = b_pc;
	assign x_uop = (REG_IO != 0) ? b_uop : (b_uop | ({{(UOP_W-1){1'b0}}, b_pass} << (ATG_UOP_OP + ATG_TEX_PASS2)));
	assign x_uop_pt = (REG_IO != 0) ? b_uop_pt : (b_uop_pt | ({{(UOP_W-1){1'b0}}, b_pass} << (ATG_UOP_OP + ATG_TEX_PASS2)));
	assign x_uop_rep = {
		(REG_IO != 0) ? b_uop_ln : (b_uop_ln | ({{(UOP_W-1){1'b0}}, b_pass} << (ATG_UOP_OP + ATG_TEX_PASS2))),
		(REG_IO != 0) ? b_uop_fp : (b_uop_fp | ({{(UOP_W-1){1'b0}}, b_pass} << (ATG_UOP_OP + ATG_TEX_PASS2)))
	};
	assign x_a = va;
	assign x_b = vb;
	assign x_c = vc;
	assign x_mask = (REG_IO != 0) ? b_mask_r : {exec_g[NPHYS-1:0] & cov_g[NPHYS-1:0] & ~kill_g[NPHYS-1:0],
		exec_g[NPHYS-1:0]};
	assign x_info = (REG_IO != 0) ? b_info_r : {13'd0, b_q_info[ATG_WI_FRONT], b_q_info[ATG_WI_KIND +: 2],
		b_q_info[ATG_WI_SEQ +: 16]};

	wire unused_fwd_ex4 = &{1'b0,
		fwd_fpu_ex4_valid, fwd_fpu_ex4_warp, fwd_fpu_ex4_grp, fwd_fpu_ex4_rd, fwd_fpu_ex4_data, fwd_fpu_ex4_lanes,
		hit_fpu_ex4_1, hit_fpu_ex4_2, hit_fpu_ex4_3, a_cl_co_unused, a_mods[6]};
	wire unused_ok = &{1'b0, a_n1, a_n2[6], a_n3[6], a_rf_r1, a_rf_r2, a_wait, mods[6], exec_g, kill_g, cov_g, grp_base, b_q_info, unused_fwd_ex4, cl_co_unused, a_cl_hi_co_unused, a_cl_co_unused, b_unit_r, b_sg_last_r, unit_ready, ir1_co_unused, ir3_co_unused, a_act_pre_r[15], a_wait_pre_r[15]};
endmodule

`default_nettype wire
