/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_alu: the integer ALU of the SU (spec/gpu/su/atg_su_alu.md), NPHYS
 * lanes: add/sub, shifts, bitwise logic, Boolean compares (all ones/0),
 * Zbb min/max/andn/orn/xnor, sel (bitwise multiplexer), and lui/movi.
 *
 * Real 6-stage physical pipeline (ws001-p010):
 *   Stage 1 (EX1): Operand latching, bitwise logic, operand prep for add/sub and shift
 *   Stage 2 (EX2): 33-bit addition and subtraction (am_addsub)
 *   Stage 3 (EX3): 33-bit barrel shifter (am_shift) and comparison logic
 *   Stage 4 (EX4): Operation result multiplexer
 *   Stage 5 (EX5): Writeback preparation and formatting
 *   Stage 6 (WB):  Output registers driving s_* exactly SHORT_LAT (6) cycles after x_valid
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_alu #(
	parameter NPHYS = 8,
	parameter NWARPS = 4,
	parameter WARP_W = 2,
	parameter GRP_W = 2,
	parameter SBE_W = 2,
	parameter UOP_W = 108,
	parameter SHORT_LAT = 6	/* Cycles from x_valid to the short result */
) (
	input wire clk,
	input wire rst_n,
	input wire en,

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

	/* Forwarding buses (to atg_su_opd) */
	output wire fwd_ex3_valid,
	output wire [WARP_W-1:0] fwd_ex3_warp,
	output wire [GRP_W-1:0] fwd_ex3_grp,
	output wire [6:0] fwd_ex3_rd,

	output wire fwd_ex4_valid,
	output wire [WARP_W-1:0] fwd_ex4_warp,
	output wire [GRP_W-1:0] fwd_ex4_grp,
	output wire [6:0] fwd_ex4_rd,
	output wire [NPHYS*32-1:0] fwd_ex4_data,
	output wire [NPHYS-1:0] fwd_ex4_lanes,

	output wire fwd_ex5_valid,
	output wire [WARP_W-1:0] fwd_ex5_warp,
	output wire [GRP_W-1:0] fwd_ex5_grp,
	output wire [6:0] fwd_ex5_rd,
	output wire [NPHYS*32-1:0] fwd_ex5_data,
	output wire [NPHYS-1:0] fwd_ex5_lanes
);
`include "atg_su_uop.vh"
	localparam DW = NPHYS * 32;

	/* Instruction fields */
	wire rst_p = !rst_n;
	wire [3:0] unit = x_uop[ATG_UOP_UNIT +: 4];
	wire [4:0] op = x_uop[ATG_UOP_OP +: 5];
	wire immb = x_uop[ATG_UOP_OP + ATG_ALU_IMMB];
	wire [3:0] immb_g;
	wire [NPHYS*4-1:0] immb_buf;
	wire [NPHYS-1:0]   immb_amt_buf;
	wire [4:0]         imm_amt_g [0:3];
`ifdef SYNTH_KINTEX7
	genvar g_ib, g_ig;
	generate
		for (g_ig = 0; g_ig < 4; g_ig = g_ig + 1) begin : g_immb_g
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ibg (.I0(immb), .O(immb_g[g_ig]));
			for (g_ib = 0; g_ib < 5; g_ib = g_ib + 1) begin : g_iag
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iag (.I0(imm[g_ib]), .O(imm_amt_g[g_ig][g_ib]));
			end
		end
		for (g_ib = 0; g_ib < NPHYS*4; g_ib = g_ib + 1) begin : g_immb_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ib (.I0(immb_g[g_ib / NPHYS]), .O(immb_buf[g_ib]));
		end
		for (g_ib = 0; g_ib < NPHYS; g_ib = g_ib + 1) begin : g_immba_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_iba (.I0(immb_g[(g_ib * 4) / NPHYS]), .O(immb_amt_buf[g_ib]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_ib, g_ig;
	generate
		for (g_ig = 0; g_ig < 4; g_ig = g_ig + 1) begin : g_immb_g
			assign immb_g[g_ig] = immb;
			for (g_ib = 0; g_ib < 5; g_ib = g_ib + 1) begin : g_iag
				assign imm_amt_g[g_ig][g_ib] = imm[g_ib];
			end
		end
		for (g_ib = 0; g_ib < NPHYS*4; g_ib = g_ib + 1) begin : g_immb_buf
			assign immb_buf[g_ib] = immb_g[g_ib / NPHYS];
		end
		for (g_ib = 0; g_ib < NPHYS; g_ib = g_ib + 1) begin : g_immba_buf
			assign immb_amt_buf[g_ib] = immb_g[(g_ib * 4) / NPHYS];
		end
	endgenerate
`else
	assign immb_g       = {4{immb}};
	assign immb_buf     = {(NPHYS*4){immb}};
	assign immb_amt_buf = {NPHYS{immb}};
	genvar g_ig;
	generate
		for (g_ig = 0; g_ig < 4; g_ig = g_ig + 1) begin : g_imm_g
			assign imm_amt_g[g_ig] = imm[4:0];
		end
	endgenerate
`endif
	wire [31:0] imm = x_uop[ATG_UOP_IMM +: 32];
	wire mine = x_valid && unit == ATG_UNIT_ALU && x_uop[ATG_UOP_WRD];

	function [31:0] rev;
		input [31:0] w;
		integer b;
		begin
			for (b = 0; b < 32; b = b + 1)
				rev[b] = w[31 - b];
		end
	endfunction

	/*
	 * Stage 1 (EX1): Latch inputs, prepare addsub and shift
	 */
	(* shreg_extract = "no" *) reg s1_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s1_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s1_grp;
	(* shreg_extract = "no" *) reg [6:0]        s1_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s1_mask;
	(* shreg_extract = "no", max_fanout = 12 *) reg [4:0]        s1_op;

	(* shreg_extract = "no", max_fanout = 12 *) reg [DW-1:0] s1_la, s1_lb, s1_lb_x;
	(* shreg_extract = "no" *) reg [DW-1:0] s1_lc;
	(* shreg_extract = "no" *) reg [31:0] s1_movi_w;
	(* shreg_extract = "no", dont_touch = "true" *) reg [NPHYS-1:0] s1_is_sll_lo, s1_is_sll_hi;
	(* shreg_extract = "no", max_fanout = 12 *) reg [NPHYS-1:0] s1_sra;
	(* shreg_extract = "no", max_fanout = 12 *) reg [NPHYS*5-1:0] s1_amt;
	(* shreg_extract = "no", keep = "true" *) reg [NPHYS-1:0] s1_sub;
	wire [DW-1:0] s0_lb_sel, s0_lb_x;
	wire s0_is_sub = (op != ATG_ALU_ADD);
	wire [NPHYS*2-1:0] s0_is_sub_b;
	genvar gl1;
	generate
		for (gl1 = 0; gl1 < NPHYS; gl1 = gl1 + 1) begin : g_s0_lb
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssub0 (.I0(s0_is_sub), .O(s0_is_sub_b[gl1*2]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ssub1 (.I0(s0_is_sub), .O(s0_is_sub_b[gl1*2+1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign s0_is_sub_b[gl1*2] = s0_is_sub;
			assign s0_is_sub_b[gl1*2+1] = s0_is_sub;
`else
			assign s0_is_sub_b[gl1*2]   = s0_is_sub;
			assign s0_is_sub_b[gl1*2+1] = s0_is_sub;
`endif
			wire [15:0] b_lo = {
				immb_buf[gl1*4+1] ? imm[15:8] : x_b[gl1*32 + 8 +: 8],
				immb_buf[gl1*4+0] ? imm[7:0]  : x_b[gl1*32 +: 8]
			};
			wire [15:0] b_hi = {
				immb_buf[gl1*4+3] ? imm[31:24] : x_b[gl1*32 + 24 +: 8],
				immb_buf[gl1*4+2] ? imm[23:16] : x_b[gl1*32 + 16 +: 8]
			};
			assign s0_lb_sel[gl1*32 +: 32] = {b_hi, b_lo};
			assign s0_lb_x[gl1*32 +: 16]      = s0_is_sub_b[gl1*2]   ? ~b_lo : b_lo;
			assign s0_lb_x[gl1*32 + 16 +: 16] = s0_is_sub_b[gl1*2+1] ? ~b_hi : b_hi;
		end
	endgenerate

`ifdef __ICARUS__
	wire en_s1 = en && mine;
	wire en_s2 = en && s1_valid;
	wire en_s3 = en && s2_valid;
	wire en_s4 = en && s3_valid;
	wire en_s5 = en && s4_valid;
	wire en_s6 = en && s5_valid;
`else
	wire en_s1 = en;
	wire en_s2 = en;
	wire en_s3 = en;
	wire en_s4 = en;
	wire en_s5 = en;
	wire en_s6 = en;
`endif
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s1_valid <= 1'b0;
			s1_warp  <= {WARP_W{1'b0}};
			s1_grp   <= {GRP_W{1'b0}};
			s1_rd    <= 7'd0;
			s1_mask  <= {NPHYS{1'b0}};
			s1_op    <= 5'd0;
			s1_sub   <= {NPHYS{1'b0}};
		end else if (en) begin
			s1_valid <= mine;
			s1_warp  <= x_warp;
			s1_grp   <= x_grp;
			s1_rd    <= x_uop[ATG_UOP_RD +: 7];
			s1_mask  <= x_mask[NPHYS-1:0];
			s1_op    <= op;
			s1_sub   <= {NPHYS{(op != ATG_ALU_ADD)}};
		end
	end

	integer i1;
	always @(posedge clk) begin
		if (en_s1) begin
			s1_lc     <= x_c;
			s1_movi_w <= imm;
			s1_lb     <= s0_lb_sel;
			s1_lb_x   <= s0_lb_x;
			for (i1 = 0; i1 < NPHYS; i1 = i1 + 1) begin
				s1_la[i1*32 +: 32] <= x_a[i1*32 +: 32];
				s1_is_sll_lo[i1]   <= (op == ATG_ALU_SLL);
				s1_is_sll_hi[i1]   <= (op == ATG_ALU_SLL);
				s1_sra[i1]         <= (op == ATG_ALU_SRA) && x_a[i1*32 + 31];
				s1_amt[i1*5 +: 5]  <= immb_amt_buf[i1] ? imm_amt_g[(i1 * 4) / NPHYS] : x_b[i1*32 +: 5];
			end
		end
	end

	/*
	 * Stage 2 (EX2): 16b + 17b carry-select addition / subtraction and bitwise logic
	 */
	wire [NPHYS*33-1:0] s2_d_comb;
	wire [NPHYS*2-1:0] s2_co_unused;

	genvar l2;
	generate
	for (l2 = 0; l2 < NPHYS; l2 = l2 + 1) begin : g_ex2_addsub
		wire [15:0] d_lo;
		wire        d_lo_co;
		wire [16:0] d_hi0, d_hi1;
		am_add #(.W(16)) u_d_lo (
			.a(s1_la[l2*32 +: 16]),
			.b(s1_lb_x[l2*32 +: 16]),
			.ci(s1_sub[l2]),
			.s(d_lo),
			.co(d_lo_co)
		);
		am_add #(.W(17)) u_d_hi0 (
			.a({1'b0, s1_la[l2*32 + 16 +: 16]}),
			.b({s1_sub[l2], s1_lb_x[l2*32 + 16 +: 16]}),
			.ci(1'b0),
			.s(d_hi0),
			.co(s2_co_unused[l2*2])
		);
		am_add #(.W(17)) u_d_hi1 (
			.a({1'b0, s1_la[l2*32 + 16 +: 16]}),
			.b({s1_sub[l2], s1_lb_x[l2*32 + 16 +: 16]}),
			.ci(1'b1),
			.s(d_hi1),
			.co(s2_co_unused[l2*2 + 1])
		);
		assign s2_d_comb[l2*33 +: 33] = {d_lo_co ? d_hi1 : d_hi0, d_lo};
	end
	endgenerate

	(* shreg_extract = "no" *) reg s2_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s2_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s2_grp;
	(* shreg_extract = "no" *) reg [6:0]        s2_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s2_mask;
	(* shreg_extract = "no", max_fanout = 12 *) reg [4:0]        s2_op;

	(* shreg_extract = "no" *) reg [NPHYS*33-1:0] s2_d;
	(* shreg_extract = "no" *) reg [DW-1:0] s2_la, s2_lb;
	(* shreg_extract = "no" *) reg [DW-1:0] s2_and, s2_or, s2_xor, s2_andn, s2_orn, s2_xnor, s2_sel, s2_movi;
	(* shreg_extract = "no" *) reg [DW-1:0] s2_sv;
	(* shreg_extract = "no", max_fanout = 12 *) reg [NPHYS-1:0] s2_sra;
	(* shreg_extract = "no", max_fanout = 12 *) reg [NPHYS*5-1:0] s2_amt;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s2_valid <= 1'b0;
			s2_warp  <= {WARP_W{1'b0}};
			s2_grp   <= {GRP_W{1'b0}};
			s2_rd    <= 7'd0;
			s2_mask  <= {NPHYS{1'b0}};
			s2_op    <= 5'd0;
		end else if (en) begin
			s2_valid <= s1_valid;
			s2_warp  <= s1_warp;
			s2_grp   <= s1_grp;
			s2_rd    <= s1_rd;
			s2_mask  <= s1_mask;
			s2_op    <= s1_op;
		end
	end

	integer i2;
	always @(posedge clk) begin
		if (en_s2) begin
			s2_d    <= s2_d_comb;
			s2_la   <= s1_la;
			s2_lb   <= s1_lb;
			s2_and  <= s1_la & s1_lb;
			s2_or   <= s1_la | s1_lb;
			s2_xor  <= s1_la ^ s1_lb;
			s2_andn <= s1_la & ~s1_lb;
			s2_orn  <= s1_la | ~s1_lb;
			s2_xnor <= ~(s1_la ^ s1_lb);
			s2_sel  <= (s1_la & s1_lb) | (~s1_la & s1_lc);
			s2_sra  <= s1_sra;
			s2_amt  <= s1_amt;
			for (i2 = 0; i2 < NPHYS; i2 = i2 + 1) begin : blk_s2_sv
				reg [31:0] la_r;
				la_r = rev(s1_la[i2*32 +: 32]);
				s2_movi[i2*32 +: 32]      <= s1_movi_w;
				s2_sv[i2*32 +: 16]        <= s1_is_sll_lo[i2] ? la_r[15:0]  : s1_la[i2*32 +: 16];
				s2_sv[i2*32 + 16 +: 16]   <= s1_is_sll_hi[i2] ? la_r[31:16] : s1_la[i2*32 + 16 +: 16];
			end
		end
	end

	/*
	 * Stage 3 (EX3): 33-bit barrel shifter & comparison
	 */
	wire [NPHYS*33-1:0] s3_sh_comb;

	genvar l3;
	generate
	for (l3 = 0; l3 < NPHYS; l3 = l3 + 1) begin : g_ex3_shift
		am_shift #(.W(33), .SW(5)) u_sh (
			.a({s2_sra[l3], s2_sv[l3*32 +: 32]}),
			.amt(s2_amt[l3*5 +: 5]),
			.left(1'b0),
			.arith(1'b1),
			.y(s3_sh_comb[l3*33 +: 33])
		);
	end
	endgenerate

	(* shreg_extract = "no" *) reg s3_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s3_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s3_grp;
	(* shreg_extract = "no" *) reg [6:0]        s3_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s3_mask;
	(* shreg_extract = "no", keep = "true", max_fanout = 12 *) reg [NPHYS*5-1:0] s3_op_lane;

	(* shreg_extract = "no" *) reg [DW-1:0] s3_sh;
	(* shreg_extract = "no" *) reg [DW-1:0] s3_d;
	(* shreg_extract = "no" *) reg [DW-1:0] s3_la, s3_lb;
	(* shreg_extract = "no", max_fanout = 12 *) reg [NPHYS-1:0] s3_slt, s3_ltu;
	(* shreg_extract = "no" *) reg [DW-1:0] s3_and, s3_or, s3_xor, s3_andn, s3_orn, s3_xnor, s3_sel, s3_movi;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s3_valid <= 1'b0;
			s3_warp  <= {WARP_W{1'b0}};
			s3_grp   <= {GRP_W{1'b0}};
			s3_rd    <= 7'd0;
			s3_mask  <= {NPHYS{1'b0}};
		end else if (en) begin
			s3_valid <= s2_valid;
			s3_warp  <= s2_warp;
			s3_grp   <= s2_grp;
			s3_rd    <= s2_rd;
			s3_mask  <= s2_mask;
		end
	end

	integer i3;
	always @(posedge clk) begin
		if (en_s3) begin
			s3_la    <= s2_la;
			s3_lb    <= s2_lb;
			s3_and   <= s2_and;
			s3_or    <= s2_or;
			s3_xor   <= s2_xor;
			s3_andn  <= s2_andn;
			s3_orn   <= s2_orn;
			s3_xnor  <= s2_xnor;
			s3_sel   <= s2_sel;
			s3_movi  <= s2_movi;
			for (i3 = 0; i3 < NPHYS; i3 = i3 + 1) begin
				s3_op_lane[i3*5 +: 5] <= s2_op;
				s3_sh[i3*32 +: 32] <= s3_sh_comb[i3*33 +: 32];
				s3_d[i3*32 +: 32]  <= s2_d[i3*33 +: 32];
				s3_slt[i3] <= (s2_la[i3*32 + 31] != s2_lb[i3*32 + 31]) ?
					s2_la[i3*32 + 31] : s2_d[i3*33 + 31];
				s3_ltu[i3] <= s2_d[i3*33 + 32];
			end
		end
	end

	/*
	 * Stage 4 (EX4): Operation result multiplexer
	 */
	reg [DW-1:0] s4_res_comb;
	integer i4;
	always @(*) begin
		s4_res_comb = {DW{1'b0}};
		for (i4 = 0; i4 < NPHYS; i4 = i4 + 1) begin
			case (s3_op_lane[i4*5 +: 5])
			ATG_ALU_ADD:  s4_res_comb[i4*32 +: 32] = s3_d[i4*32 +: 32];
			ATG_ALU_SUB:  s4_res_comb[i4*32 +: 32] = s3_d[i4*32 +: 32];
			ATG_ALU_SLL:  s4_res_comb[i4*32 +: 32] = rev(s3_sh[i4*32 +: 32]);
			ATG_ALU_SLT:  s4_res_comb[i4*32 +: 32] = s3_slt[i4] ? 32'hFFFFFFFF : 32'd0;
			ATG_ALU_SLTU: s4_res_comb[i4*32 +: 32] = s3_ltu[i4] ? 32'hFFFFFFFF : 32'd0;
			ATG_ALU_XOR:  s4_res_comb[i4*32 +: 32] = s3_xor[i4*32 +: 32];
			ATG_ALU_SRL:  s4_res_comb[i4*32 +: 32] = s3_sh[i4*32 +: 32];
			ATG_ALU_SRA:  s4_res_comb[i4*32 +: 32] = s3_sh[i4*32 +: 32];
			ATG_ALU_OR:   s4_res_comb[i4*32 +: 32] = s3_or[i4*32 +: 32];
			ATG_ALU_AND:  s4_res_comb[i4*32 +: 32] = s3_and[i4*32 +: 32];
			ATG_ALU_MIN:  s4_res_comb[i4*32 +: 32] = s3_slt[i4] ? s3_la[i4*32 +: 32] : s3_lb[i4*32 +: 32];
			ATG_ALU_MAX:  s4_res_comb[i4*32 +: 32] = s3_slt[i4] ? s3_lb[i4*32 +: 32] : s3_la[i4*32 +: 32];
			ATG_ALU_MINU: s4_res_comb[i4*32 +: 32] = s3_ltu[i4] ? s3_la[i4*32 +: 32] : s3_lb[i4*32 +: 32];
			ATG_ALU_MAXU: s4_res_comb[i4*32 +: 32] = s3_ltu[i4] ? s3_lb[i4*32 +: 32] : s3_la[i4*32 +: 32];
			ATG_ALU_ANDN: s4_res_comb[i4*32 +: 32] = s3_andn[i4*32 +: 32];
			ATG_ALU_ORN:  s4_res_comb[i4*32 +: 32] = s3_orn[i4*32 +: 32];
			ATG_ALU_XNOR: s4_res_comb[i4*32 +: 32] = s3_xnor[i4*32 +: 32];
			ATG_ALU_SEL:  s4_res_comb[i4*32 +: 32] = s3_sel[i4*32 +: 32];
			ATG_ALU_MOVI: s4_res_comb[i4*32 +: 32] = s3_movi[i4*32 +: 32];
			default:      s4_res_comb[i4*32 +: 32] = 32'd0;
			endcase
		end
	end

	(* shreg_extract = "no" *) reg s4_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s4_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s4_grp;
	(* shreg_extract = "no" *) reg [6:0]        s4_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s4_mask;
	(* shreg_extract = "no" *) reg [DW-1:0]     s4_data;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s4_valid <= 1'b0;
			s4_warp  <= {WARP_W{1'b0}};
			s4_grp   <= {GRP_W{1'b0}};
			s4_rd    <= 7'd0;
			s4_mask  <= {NPHYS{1'b0}};
		end else if (en) begin
			s4_valid <= s3_valid;
			s4_warp  <= s3_warp;
			s4_grp   <= s3_grp;
			s4_rd    <= s3_rd;
			s4_mask  <= s3_mask;
		end
	end

	always @(posedge clk) begin
		if (en_s4)
			s4_data  <= s4_res_comb;
	end

	/*
	 * Stage 5 (EX5): Writeback prep and pipeline alignment
	 */
	(* shreg_extract = "no" *) reg s5_valid;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s5_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s5_grp;
	(* shreg_extract = "no" *) reg [6:0]        s5_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s5_mask;
	(* shreg_extract = "no" *) reg [DW-1:0]     s5_data;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s5_valid <= 1'b0;
			s5_warp  <= {WARP_W{1'b0}};
			s5_grp   <= {GRP_W{1'b0}};
			s5_rd    <= 7'd0;
			s5_mask  <= {NPHYS{1'b0}};
		end else if (en) begin
			s5_valid <= s4_valid;
			s5_warp  <= s4_warp;
			s5_grp   <= s4_grp;
			s5_rd    <= s4_rd;
			s5_mask  <= s4_mask;
		end
	end

	always @(posedge clk) begin
		if (en_s5)
			s5_data  <= s4_data;
	end

	/*
	 * Stage 6 (WB): Registered drive to s_* exactly SHORT_LAT (6) cycles after x_valid
	 */
	(* shreg_extract = "no" *) reg s6_valid;
	(* shreg_extract = "no", keep = "true" *) reg [7:0] s6_valid_rep;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] s6_warp;
	(* shreg_extract = "no" *) reg [GRP_W-1:0]  s6_grp;
	(* shreg_extract = "no" *) reg [6:0]        s6_rd;
	(* shreg_extract = "no" *) reg [NPHYS-1:0]  s6_lanes;
	(* shreg_extract = "no" *) reg [DW-1:0]     s6_data;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			s6_valid <= 1'b0;
			s6_valid_rep <= 8'd0;
			s6_warp  <= {WARP_W{1'b0}};
			s6_grp   <= {GRP_W{1'b0}};
			s6_rd    <= 7'd0;
			s6_lanes <= {NPHYS{1'b0}};
		end else if (en) begin
			s6_valid <= s5_valid;
			s6_valid_rep <= {8{s5_valid}};
			s6_warp  <= s5_warp;
			s6_grp   <= s5_grp;
			s6_rd    <= s5_rd;
			s6_lanes <= s5_mask;
		end
	end

	always @(posedge clk) begin
		if (en_s6)
			s6_data  <= s5_data;
	end

	generate
	if (SHORT_LAT > 6) begin : g_lat7
		localparam integer EXTRA = SHORT_LAT - 6;
		(* shreg_extract = "no" *) reg [EXTRA-1:0]      ex_valid;
		(* shreg_extract = "no", keep = "true" *) reg [7:0] ex_valid_rep [0:EXTRA-1];
		(* shreg_extract = "no" *) reg [WARP_W-1:0]     ex_warp  [0:EXTRA-1];
		(* shreg_extract = "no" *) reg [GRP_W-1:0]      ex_grp   [0:EXTRA-1];
		(* shreg_extract = "no" *) reg [6:0]            ex_rd    [0:EXTRA-1];
		(* shreg_extract = "no" *) reg [NPHYS-1:0]      ex_lanes [0:EXTRA-1];
		(* shreg_extract = "no" *) reg [DW-1:0]         ex_data  [0:EXTRA-1];
		integer ke;
		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				ex_valid <= {EXTRA{1'b0}};
				for (ke = 0; ke < EXTRA; ke = ke + 1) begin
					ex_valid_rep[ke] <= 8'd0;
					ex_warp[ke]      <= {WARP_W{1'b0}};
					ex_grp[ke]       <= {GRP_W{1'b0}};
					ex_rd[ke]        <= 7'd0;
					ex_lanes[ke]     <= {NPHYS{1'b0}};
				end
			end else if (en) begin
				ex_valid[0]     <= s6_valid;
				ex_valid_rep[0] <= s6_valid_rep;
				ex_warp[0]      <= s6_warp;
				ex_grp[0]       <= s6_grp;
				ex_rd[0]        <= s6_rd;
				ex_lanes[0]     <= s6_lanes;
				for (ke = 1; ke < EXTRA; ke = ke + 1) begin
					ex_valid[ke]     <= ex_valid[ke-1];
					ex_valid_rep[ke] <= ex_valid_rep[ke-1];
					ex_warp[ke]      <= ex_warp[ke-1];
					ex_grp[ke]       <= ex_grp[ke-1];
					ex_rd[ke]        <= ex_rd[ke-1];
					ex_lanes[ke]     <= ex_lanes[ke-1];
				end
			end
		end
		always @(posedge clk) begin
			if (en) begin
				ex_data[0] <= s6_data;
				for (ke = 1; ke < EXTRA; ke = ke + 1)
					ex_data[ke] <= ex_data[ke-1];
			end
		end
		assign s_valid     = ex_valid[EXTRA-1];
		assign s_valid_rep = ex_valid_rep[EXTRA-1];
		assign s_warp      = ex_warp[EXTRA-1];
		assign s_grp       = ex_grp[EXTRA-1];
		assign s_rd        = ex_rd[EXTRA-1];
		assign s_lanes     = ex_lanes[EXTRA-1];
		assign s_data      = ex_data[EXTRA-1];
	end else begin : g_lat6
		assign s_valid     = s6_valid;
		assign s_valid_rep = s6_valid_rep;
		assign s_warp      = s6_warp;
		assign s_grp       = s6_grp;
		assign s_rd        = s6_rd;
		assign s_lanes     = s6_lanes;
		assign s_data      = s6_data;
	end
	endgenerate
	assign x_ready = 1'b1;

	assign fwd_ex3_valid = s3_valid;
	assign fwd_ex3_warp  = s3_warp;
	assign fwd_ex3_grp   = s3_grp;
	assign fwd_ex3_rd    = s3_rd;

	assign fwd_ex4_valid = s4_valid;
	assign fwd_ex4_warp  = s4_warp;
	assign fwd_ex4_grp   = s4_grp;
	assign fwd_ex4_rd    = s4_rd;
	assign fwd_ex4_data  = s4_data;
	assign fwd_ex4_lanes = s4_mask;

	assign fwd_ex5_valid = s5_valid;
	assign fwd_ex5_warp  = s5_warp;
	assign fwd_ex5_grp   = s5_grp;
	assign fwd_ex5_rd    = s5_rd;
	assign fwd_ex5_data  = s5_data;
	assign fwd_ex5_lanes = s5_mask;

	wire unused_ok = &{1'b0, x_last, x_sbe, x_pc, x_mask[2*NPHYS-1:NPHYS], x_info, x_uop,
		s3_sh_comb[32], s2_co_unused, (NWARPS != 0), (SHORT_LAT != 0)};
endmodule

`default_nettype wire
