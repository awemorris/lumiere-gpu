/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_exec: Warp control: EXEC and KILLED per warp, the
 * exec.set/and/or/save and kill instructions, warp-uniform branches
 * (bany/bnone/ball/j/bexnz/bexz), end and bar; tells the scheduler where
 * the warp continues.
 *
 * The behaviour, and where it differs from
 * the design: spec/gpu/su/atg_su_exec.md (§10 Status).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_exec #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
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
	/* Short result to atg_su_wb (exactly SHORT_LAT cycles after x_valid) */
	output wire s_valid,
	output wire [7:0] s_valid_rep,
	output wire [WARP_W-1:0] s_warp,
	output wire [GRP_W-1:0] s_grp,
	output wire [6:0] s_rd,
	output wire [NPHYS*32-1:0] s_data,
	output wire [NPHYS-1:0] s_lanes,
	/* Warp control (to atg_su_sched) */
	output wire wc_valid,
	output wire [WARP_W-1:0] wc_warp,
	output wire [1:0] wc_op,
	output wire [15:0] wc_pc,
	/* EXEC state */
	input wire [WARP_W-1:0] q_warp,
	output wire [31:0] q_exec,
	output wire [31:0] q_killed,
	input wire init_valid,
	input wire [WARP_W-1:0] init_warp,
	input wire [31:0] init_mask
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam EW = 1 + WARP_W + GRP_W + 7 + NPHYS + DW;
	localparam SKID_W = WARP_W + GRP_W + 1 + 16 + UOP_W + DW;
	localparam LAT_REM = (SHORT_LAT > 2) ? (SHORT_LAT - 2) : 1;

	wire rst_p = !rst_n;
	wire [NWARPS*32-1:0] exec_r, kill_r;

	/* Stage 1: skid-registered input boundary decoupling opd/rf from EXEC logic */
	wire s1_valid;
	wire [SKID_W-1:0] s1_data;
	wire x_exec_in = x_valid && (x_uop[ATG_UOP_UNIT +: 4] == ATG_UNIT_EXEC);
	atg_su_skid #(.W(SKID_W), .SL_S(8)) u_exec_skid (
		.clk(clk), .rst_n(rst_n),
		.in_valid(x_exec_in), .in_ready(x_ready),
		.in_data({x_warp, x_grp, x_last, x_pc, x_uop, x_a}),
		.out_valid(s1_valid), .out_ready(1'b1), .out_data(s1_data)
	);

	wire [WARP_W-1:0] s1_warp = s1_data[DW + UOP_W + 16 + 1 + GRP_W +: WARP_W];
	wire [GRP_W-1:0]  s1_grp  = s1_data[DW + UOP_W + 16 + 1 +: GRP_W];
	wire              s1_last = s1_data[DW + UOP_W + 16];
	wire [15:0]       s1_pc   = s1_data[DW + UOP_W +: 16];
	wire [UOP_W-1:0]  s1_uop  = s1_data[DW +: UOP_W];
	wire [DW-1:0]     s1_a    = s1_data[DW-1:0];

	wire [7:0]  s1_op    = s1_uop[ATG_UOP_OP +: 8];
	wire [31:0] s1_imm   = s1_uop[ATG_UOP_IMM +: 32];
	wire [6:0]  s1_rd    = s1_uop[ATG_UOP_RD +: 7];

	/* Stage 2 combinational: per-lane Boolean zero-detect, opcode predecode, PC+1 / target add */
	wire [NPHYS-1:0] s1_bt;
	genvar gl;
	generate
		for (gl = 0; gl < NPHYS; gl = gl + 1) begin : g_bt
			wire bt_hi = |s1_a[gl*32 + 16 +: 16];
			wire bt_lo = |s1_a[gl*32 +: 16];
			assign s1_bt[gl] = bt_hi | bt_lo;
		end
	endgenerate

	wire [15:0] s1_pc_p1, s1_pc_tgt;
	wire [1:0] pc_co_unused;
	am_add #(.W(16)) u_pc_p1  (.a(s1_pc), .b(16'd1),         .ci(1'b0), .s(s1_pc_p1),  .co(pc_co_unused[0]));
	am_add #(.W(16)) u_pc_tgt (.a(s1_pc), .b(s1_imm[15:0]),  .ci(1'b0), .s(s1_pc_tgt), .co(pc_co_unused[1]));

	wire s1_is_br      = (s1_op <= ATG_EXEC_BEXZ);
	wire s1_is_kill    = (s1_op == ATG_EXEC_KILL);
	wire s1_is_exec_op = (s1_op == ATG_EXEC_SET || s1_op == ATG_EXEC_AND || s1_op == ATG_EXEC_OR);
	wire s1_is_save    = (s1_op == ATG_EXEC_SAVE) && s1_uop[ATG_UOP_WRD];
	wire s1_is_wc      = s1_last && (s1_is_br || s1_op == ATG_EXEC_END || s1_op == ATG_EXEC_BAR);
	wire [1:0] s1_wc_op = (s1_op == ATG_EXEC_END) ? 2'd1 : (s1_op == ATG_EXEC_BAR) ? 2'd2 : 2'd0;

	/* Stage 2 registers */
	(* max_fanout = 16 *) reg              s2_valid;
	(* max_fanout = 12 *) reg [WARP_W-1:0] s2_warp;
	(* max_fanout = 12 *) reg [GRP_W-1:0]  s2_grp;
	(* max_fanout = 12 *) reg              s2_last;
	(* max_fanout = 12 *) reg [2:0]        s2_op3;
	(* max_fanout = 12 *) reg [7:0]        s2_op;
	(* max_fanout = 12 *) reg [NPHYS-1:0]  s2_bt;
	(* max_fanout = 12 *) reg              s2_is_br, s2_is_kill, s2_is_exec_op, s2_is_save, s2_is_wc;
	(* max_fanout = 12 *) reg [1:0]        s2_wc_op;
	reg [6:0]                              s2_rd;
	reg [15:0]                             s2_pc_p1, s2_pc_tgt;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s2_valid      <= 1'b0;
			s2_warp       <= {WARP_W{1'b0}};
			s2_grp        <= {GRP_W{1'b0}};
			s2_last       <= 1'b0;
			s2_op3        <= 3'd0;
			s2_op         <= 8'd0;
			s2_bt         <= {NPHYS{1'b0}};
			s2_is_br      <= 1'b0;
			s2_is_kill    <= 1'b0;
			s2_is_exec_op <= 1'b0;
			s2_is_save    <= 1'b0;
			s2_is_wc      <= 1'b0;
			s2_wc_op      <= 2'd0;
			s2_rd         <= 7'd0;
			s2_pc_p1      <= 16'd0;
			s2_pc_tgt     <= 16'd0;
		end else begin
			s2_valid      <= s1_valid;
			if (s1_valid) begin
				s2_warp       <= s1_warp;
				s2_grp        <= s1_grp;
				s2_last       <= s1_last;
				s2_op3        <= s1_op[2:0];
				s2_op         <= s1_op;
				s2_bt         <= s1_bt;
				s2_is_br      <= s1_is_br;
				s2_is_kill    <= s1_is_kill;
				s2_is_exec_op <= s1_is_exec_op;
				s2_is_save    <= s1_is_save;
				s2_is_wc      <= s1_is_wc;
				s2_wc_op      <= s1_wc_op;
				s2_rd         <= s1_rd;
				s2_pc_p1      <= s1_pc_p1;
				s2_pc_tgt     <= s1_pc_tgt;
			end
		end
	end

	/* Stage 3 combinational: EXEC/KILLED mask read, branch decision, mask write data */
	wire [WARP_W-1:0] s2_warp_ex_lo, s2_warp_ex_hi;
	wire [GRP_W-1:0]  s2_grp_ex;
`ifdef SYNTH_KINTEX7
	genvar gxw;
	generate
		for (gxw = 0; gxw < WARP_W; gxw = gxw + 1) begin : g_buf_xww
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_xww_lo (.I0(s2_warp[gxw]), .O(s2_warp_ex_lo[gxw]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_xww_hi (.I0(s2_warp[gxw]), .O(s2_warp_ex_hi[gxw]));
		end
		for (gxw = 0; gxw < GRP_W; gxw = gxw + 1) begin : g_buf_xgw
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_xgw (.I0(s2_grp[gxw]), .O(s2_grp_ex[gxw]));
		end
	endgenerate
`else
	assign s2_warp_ex_lo = s2_warp;
	assign s2_warp_ex_hi = s2_warp;
	assign s2_grp_ex     = s2_grp;
`endif
	(* max_fanout = 12 *) wire [31:0] wexec = {
		exec_r[{s2_warp_ex_hi, 5'd16} +: 16],
		exec_r[{s2_warp_ex_lo, 5'd0}  +: 16]
	};

	(* max_fanout = 12 *) reg [NPHYS-1:0] wexec_g;
	integer gi;
	always @(*) begin
		wexec_g = {NPHYS{1'b0}};
		for (gi = 0; gi < 32/NPHYS; gi = gi + 1) begin
			if (s2_grp_ex == gi[GRP_W-1:0])
				wexec_g = wexec[gi*NPHYS +: NPHYS];
		end
	end

	wire [NPHYS-1:0] em = wexec_g;
	wire [NPHYS-1:0] bt = s2_bt;

	(* max_fanout = 12 *) reg acc_any, acc_all;
	wire any_now = acc_any || |(bt & em);
	wire all_now = acc_all && &(bt | ~em);
	wire br_static_take = (s2_op == ATG_EXEC_J) ||
	                      (s2_op == ATG_EXEC_BEXNZ && wexec != 32'd0) ||
	                      (s2_op == ATG_EXEC_BEXZ && wexec == 32'd0);
	wire wc_take_now = s2_is_br && (br_static_take ||
	                   (s2_op3 == ATG_EXEC_BALL[2:0] && all_now) ||
	                   (s2_op3 == ATG_EXEC_BANY[2:0] && any_now) ||
	                   (s2_op3 == ATG_EXEC_BNONE[2:0] && !any_now));

	reg [DW-1:0] save_d;
	integer l;
	always @(*)
		for (l = 0; l < NPHYS; l = l + 1)
			save_d[l*32 +: 32] = {32{wexec_g[l]}};
	wire s_in = s2_valid && s2_is_save;

	(* max_fanout = 12 *) reg [LAT_REM-1:0] pipe_v;
	(* shreg_extract = "no", keep = "true" *) reg [7:0] pipe_v_rep;
	(* shreg_extract = "no", max_fanout = 12 *) reg [EW-2:0] pipe_d [0:LAT_REM-1];
	(* max_fanout = 12 *) reg wc_v;
	(* max_fanout = 12 *) reg [WARP_W-1:0] wc_w;
	(* max_fanout = 12 *) reg [1:0] wc_o;
	reg [15:0] wc_pc_r;

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) wire wc_ce_ctrl = s2_valid && s2_is_wc;
`else
	wire wc_ce_ctrl = s2_valid && s2_is_wc;
`endif
	integer k;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			wc_v <= 1'b0;
			wc_w <= {WARP_W{1'b0}};
			wc_o <= 2'd0;
			wc_pc_r <= 16'd0;
			for (k = 0; k < LAT_REM; k = k + 1)
				pipe_v[k] <= 1'b0;
			pipe_v_rep <= 8'd0;
		end else begin
			wc_v <= wc_ce_ctrl;
			if (wc_ce_ctrl) begin
				wc_w <= s2_warp;
				wc_o <= s2_wc_op;
				wc_pc_r <= wc_take_now ? s2_pc_tgt : s2_pc_p1;
			end
			pipe_v[0] <= s_in;
			for (k = 1; k < LAT_REM; k = k + 1)
				pipe_v[k] <= pipe_v[k-1];
			pipe_v_rep <= (LAT_REM > 1) ? {8{pipe_v[LAT_REM-2]}} : {8{s_in}};
		end
	end

	integer kd;
	always @(posedge clk) begin
		pipe_d[0] <= {s2_warp, s2_grp, s2_rd, {NPHYS{1'b1}}, save_d};
		for (kd = 1; kd < LAT_REM; kd = kd + 1)
			pipe_d[kd] <= pipe_d[kd-1];
	end

	wire [1:0] op_w [0:NWARPS-1];
	wire [WARP_W-1:0] x_warp_w [0:NWARPS-1];
	wire exec_op_match_w [0:NWARPS-1];
	wire [WARP_W-1:0] q_warp_ex_lo, q_warp_ex_hi, q_warp_kl_lo, q_warp_kl_hi;
	genvar gwb, gbb;
	generate
	for (gwb = 0; gwb < NWARPS; gwb = gwb + 1) begin : g_buf_w
		for (gbb = 0; gbb < 2; gbb = gbb + 1) begin : g_buf_op
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_op (.I0(s2_op[gbb]), .O(op_w[gwb][gbb]));
`else
			assign op_w[gwb][gbb] = s2_op[gbb];
`endif
		end
		for (gbb = 0; gbb < WARP_W; gbb = gbb + 1) begin : g_buf_xw
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_xw (.I0(s2_warp[gbb]), .O(x_warp_w[gwb][gbb]));
`else
			assign x_warp_w[gwb][gbb] = s2_warp[gbb];
`endif
		end
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_eom (.I0(s2_is_exec_op), .O(exec_op_match_w[gwb]));
`else
		assign exec_op_match_w[gwb] = s2_is_exec_op;
`endif
	end
	for (gbb = 0; gbb < WARP_W; gbb = gbb + 1) begin : g_buf_qw
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_qwe_l (.I0(q_warp[gbb]), .O(q_warp_ex_lo[gbb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_qwe_h (.I0(q_warp[gbb]), .O(q_warp_ex_hi[gbb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_qwk_l (.I0(q_warp[gbb]), .O(q_warp_kl_lo[gbb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_qwk_h (.I0(q_warp[gbb]), .O(q_warp_kl_hi[gbb]));
`else
		assign q_warp_ex_lo[gbb] = q_warp[gbb];
		assign q_warp_ex_hi[gbb] = q_warp[gbb];
		assign q_warp_kl_lo[gbb] = q_warp[gbb];
		assign q_warp_kl_hi[gbb] = q_warp[gbb];
`endif
	end
	endgenerate

	genvar gw, gg;
	generate
		for (gw = 0; gw < NWARPS; gw = gw + 1) begin : g_warp_state
			wire w_match = s2_valid && (x_warp_w[gw] == gw[WARP_W-1:0]);
			wire exec_op_match = exec_op_match_w[gw];
			wire [1:0] op_gw = op_w[gw];

			for (gg = 0; gg < 32/NPHYS; gg = gg + 1) begin : g_grp_state
				wire w_init_kl = init_valid && (init_warp == gw[WARP_W-1:0]);
				wire w_init_ex = init_valid && (init_warp == gw[WARP_W-1:0]);
				wire g_match = w_match && (s2_grp == gg[GRP_W-1:0]);
				wire kill_ce = w_init_kl || (g_match && s2_is_kill);
				(* max_fanout = 12 *) reg [NPHYS-1:0] exec_sl_r, kill_sl_r;
				wire [NPHYS-1:0] cur_exec = exec_sl_r;
				wire [NPHYS-1:0] cur_kill = kill_sl_r;
				wire [NPHYS-1:0] kill_d = w_init_kl ? {NPHYS{1'b0}} : (cur_kill | (bt & cur_exec));

				always @(posedge clk or posedge rst_p) begin
					if (rst_p)
						kill_sl_r <= {NPHYS{1'b0}};
					else if (kill_ce)
						kill_sl_r <= kill_d;
				end

				wire exec_ce = w_init_ex || (g_match && exec_op_match);
				reg [NPHYS-1:0] exec_d;
				always @(*) begin
					if (w_init_ex)
						exec_d = init_mask[gg*NPHYS +: NPHYS];
					else case (op_gw)
						ATG_EXEC_SET[1:0]: exec_d = bt;
						ATG_EXEC_AND[1:0]: exec_d = cur_exec & bt;
						default:           exec_d = cur_exec | bt;
					endcase
				end

				always @(posedge clk or posedge rst_p) begin
					if (rst_p)
						exec_sl_r <= {NPHYS{1'b0}};
					else if (exec_ce)
						exec_sl_r <= exec_d;
				end
				assign exec_r[gw*32 + gg*NPHYS +: NPHYS] = exec_sl_r;
				assign kill_r[gw*32 + gg*NPHYS +: NPHYS] = kill_sl_r;
			end
		end
	endgenerate

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			acc_any <= 1'b0;
			acc_all <= 1'b1;
		end else if (s2_valid) begin
			acc_any <= s2_last ? 1'b0 : any_now;
			acc_all <= s2_last ? 1'b1 : all_now;
		end
	end

	assign wc_valid = wc_v;
	assign wc_warp = wc_w;
	assign wc_op = wc_o;
	assign wc_pc = wc_pc_r;
	wire [15:0] q_exec_lo, q_exec_hi, q_kill_lo, q_kill_hi;
	generate
		if (NWARPS == 2) begin : g_qmux2
			assign q_exec_lo = q_warp_ex_lo[0] ? exec_r[1*32 +: 16]    : exec_r[0*32 +: 16];
			assign q_exec_hi = q_warp_ex_hi[0] ? exec_r[1*32+16 +: 16] : exec_r[0*32+16 +: 16];
			assign q_kill_lo = q_warp_kl_lo[0] ? kill_r[1*32 +: 16]    : kill_r[0*32 +: 16];
			assign q_kill_hi = q_warp_kl_hi[0] ? kill_r[1*32+16 +: 16] : kill_r[0*32+16 +: 16];
		end else if (NWARPS == 4) begin : g_qmux4
			wire [15:0] ex_l_0 = q_warp_ex_lo[0] ? exec_r[1*32 +: 16]    : exec_r[0*32 +: 16];
			wire [15:0] ex_l_1 = q_warp_ex_lo[0] ? exec_r[3*32 +: 16]    : exec_r[2*32 +: 16];
			wire [15:0] ex_h_0 = q_warp_ex_hi[0] ? exec_r[1*32+16 +: 16] : exec_r[0*32+16 +: 16];
			wire [15:0] ex_h_1 = q_warp_ex_hi[0] ? exec_r[3*32+16 +: 16] : exec_r[2*32+16 +: 16];
			wire [15:0] kl_l_0 = q_warp_kl_lo[0] ? kill_r[1*32 +: 16]    : kill_r[0*32 +: 16];
			wire [15:0] kl_l_1 = q_warp_kl_lo[0] ? kill_r[3*32 +: 16]    : kill_r[2*32 +: 16];
			wire [15:0] kl_h_0 = q_warp_kl_hi[0] ? kill_r[1*32+16 +: 16] : kill_r[0*32+16 +: 16];
			wire [15:0] kl_h_1 = q_warp_kl_hi[0] ? kill_r[3*32+16 +: 16] : kill_r[2*32+16 +: 16];
			assign q_exec_lo = q_warp_ex_lo[1] ? ex_l_1 : ex_l_0;
			assign q_exec_hi = q_warp_ex_hi[1] ? ex_h_1 : ex_h_0;
			assign q_kill_lo = q_warp_kl_lo[1] ? kl_l_1 : kl_l_0;
			assign q_kill_hi = q_warp_kl_hi[1] ? kl_h_1 : kl_h_0;
		end else begin : g_qmux8
			wire [15:0] ex_l_00 = q_warp_ex_lo[0] ? exec_r[1*32 +: 16]    : exec_r[0*32 +: 16];
			wire [15:0] ex_l_01 = q_warp_ex_lo[0] ? exec_r[3*32 +: 16]    : exec_r[2*32 +: 16];
			wire [15:0] ex_l_10 = q_warp_ex_lo[0] ? exec_r[5*32 +: 16]    : exec_r[4*32 +: 16];
			wire [15:0] ex_l_11 = q_warp_ex_lo[0] ? exec_r[7*32 +: 16]    : exec_r[6*32 +: 16];
			wire [15:0] ex_l_0  = q_warp_ex_lo[1] ? ex_l_01               : ex_l_00;
			wire [15:0] ex_l_1  = q_warp_ex_lo[1] ? ex_l_11               : ex_l_10;
			assign q_exec_lo    = q_warp_ex_lo[2] ? ex_l_1                : ex_l_0;

			wire [15:0] ex_h_00 = q_warp_ex_hi[0] ? exec_r[1*32+16 +: 16] : exec_r[0*32+16 +: 16];
			wire [15:0] ex_h_01 = q_warp_ex_hi[0] ? exec_r[3*32+16 +: 16] : exec_r[2*32+16 +: 16];
			wire [15:0] ex_h_10 = q_warp_ex_hi[0] ? exec_r[5*32+16 +: 16] : exec_r[4*32+16 +: 16];
			wire [15:0] ex_h_11 = q_warp_ex_hi[0] ? exec_r[7*32+16 +: 16] : exec_r[6*32+16 +: 16];
			wire [15:0] ex_h_0  = q_warp_ex_hi[1] ? ex_h_01               : ex_h_00;
			wire [15:0] ex_h_1  = q_warp_ex_hi[1] ? ex_h_11               : ex_h_10;
			assign q_exec_hi    = q_warp_ex_hi[2] ? ex_h_1                : ex_h_0;

			wire [15:0] kl_l_00 = q_warp_kl_lo[0] ? kill_r[1*32 +: 16]    : kill_r[0*32 +: 16];
			wire [15:0] kl_l_01 = q_warp_kl_lo[0] ? kill_r[3*32 +: 16]    : kill_r[2*32 +: 16];
			wire [15:0] kl_l_10 = q_warp_kl_lo[0] ? kill_r[5*32 +: 16]    : kill_r[4*32 +: 16];
			wire [15:0] kl_l_11 = q_warp_kl_lo[0] ? kill_r[7*32 +: 16]    : kill_r[6*32 +: 16];
			wire [15:0] kl_l_0  = q_warp_kl_lo[1] ? kl_l_01               : kl_l_00;
			wire [15:0] kl_l_1  = q_warp_kl_lo[1] ? kl_l_11               : kl_l_10;
			assign q_kill_lo    = q_warp_kl_lo[2] ? kl_l_1                : kl_l_0;

			wire [15:0] kl_h_00 = q_warp_kl_hi[0] ? kill_r[1*32+16 +: 16] : kill_r[0*32+16 +: 16];
			wire [15:0] kl_h_01 = q_warp_kl_hi[0] ? kill_r[3*32+16 +: 16] : kill_r[2*32+16 +: 16];
			wire [15:0] kl_h_10 = q_warp_kl_hi[0] ? kill_r[5*32+16 +: 16] : kill_r[4*32+16 +: 16];
			wire [15:0] kl_h_11 = q_warp_kl_hi[0] ? kill_r[7*32+16 +: 16] : kill_r[6*32+16 +: 16];
			wire [15:0] kl_h_0  = q_warp_kl_hi[1] ? kl_h_01               : kl_h_00;
			wire [15:0] kl_h_1  = q_warp_kl_hi[1] ? kl_h_11               : kl_h_10;
			assign q_kill_hi    = q_warp_kl_hi[2] ? kl_h_1                : kl_h_0;
		end
	endgenerate
	assign q_exec   = {q_exec_hi, q_exec_lo};
	assign q_killed = {q_kill_hi, q_kill_lo};

	wire [EW-2:0] o = pipe_d[LAT_REM-1];
	assign s_valid = pipe_v[LAT_REM-1];
	assign s_valid_rep = pipe_v_rep;
	assign s_warp = o[EW-2 -: WARP_W];
	assign s_grp = o[EW-2-WARP_W -: GRP_W];
	assign s_rd = o[EW-2-WARP_W-GRP_W -: 7];
	assign s_lanes = o[DW +: NPHYS];
	assign s_data = o[DW-1:0];

	wire unused_ok = &{1'b0, x_sbe, x_b, x_c, x_mask, x_info, x_uop,
		s1_imm[31:16], pc_co_unused};
endmodule

`default_nettype wire
