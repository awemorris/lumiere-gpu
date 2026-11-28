/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_dec: Decodes a 64-bit GSI1 instruction (design §3.3, §3.4) into
 * the SU micro-op of include/atg_su_uop.vh: unit, operation, operand
 * numbers (7 bits), immediate, modifiers, latency class, destination
 * count, and the ISA-fault flag.
 *
 * d_* -> u_*: one pipeline register with valid/ready (d_ready when the
 * register is empty or u_ready takes it).  An undefined or disallowed
 * instruction gives a uop with only ILL set (unit NONE); atg_su_sb faults
 * on it.  The op codes are those of include/atg_su_uop.vh; mk/gsi1asm.py
 * holds the same table (its --dec-table output is this module's reference).
 * The opcode table is in the children atg_su_dec_tab and atg_su_dec_tabc.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_dec #(
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter UOP_W = 108,	/* Width of the decoded instruction (= ATG_UOP_W of include/atg_su_uop.vh) */
	parameter NREGS = 32,	/* GPRs per thread: 32 or 64 (r0..r62 at most; 63 is rq) */
	parameter PACKED = 0,	/* 16-bit packed arithmetic present (else those opcodes are ISA faults; packh/unpackh always present) */
	parameter SUBGROUP = 0,	/* shfl/vote/ballot present (CAPS[24]) */
	parameter HAS_SA = 0,	/* rq/sactl/qstat/sastat/qtag usable (always 0 in v1) */
	parameter MS4 = 0,	/* texsz.s present (CAPS[29]) */
	parameter PIPE_TAB = 0	/* 1: 2-stage decode pipeline for high-frequency synthesis */
) (
	input wire clk,
	input wire rst_n,
	input wire d_valid,
	output wire d_ready,
	input wire [WARP_W-1:0] d_warp,
	input wire [15:0] d_pc,
	input wire [63:0] d_instr,
	output reg u_valid,
	input wire u_ready,
	output reg [WARP_W-1:0] u_warp,
	output reg [15:0] u_pc,
	output wire [UOP_W-1:0] u_uop
);
`include "atg_su_uop.vh"

	/* Formats: which immediate the instruction has. */
	localparam [2:0] FM_I = 3'd1, FM_S = 3'd2, FM_B = 3'd3, FM_U = 3'd4;

	wire [31:0] d_w0 = d_instr[31:0];
	wire [31:0] d_w1 = d_instr[63:32];
	wire [6:0] d_opc = d_w0[6:0];
	wire [2:0] d_f3_t0, d_f3_t1, d_f3_c0, d_f3_c1, d_f3_c2, d_f3_c3;
	wire [6:0] d_f7_t0, d_f7_t1, d_f7_t2, d_f7_t3, d_f7_c0, d_f7_c1;
	wire [4:0] d_rs2f;
	wire [4:0] d_sub;
`ifdef SYNTH_KINTEX7
	genvar gdb_f;
	generate
		for (gdb_f = 0; gdb_f < 3; gdb_f = gdb_f + 1) begin : g_bf3
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3t0 (.I0(d_w0[12+gdb_f]), .O(d_f3_t0[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3t1 (.I0(d_w0[12+gdb_f]), .O(d_f3_t1[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3c0 (.I0(d_w0[12+gdb_f]), .O(d_f3_c0[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3c1 (.I0(d_w0[12+gdb_f]), .O(d_f3_c1[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3c2 (.I0(d_w0[12+gdb_f]), .O(d_f3_c2[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf3c3 (.I0(d_w0[12+gdb_f]), .O(d_f3_c3[gdb_f]));
		end
		for (gdb_f = 0; gdb_f < 7; gdb_f = gdb_f + 1) begin : g_bf7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7t0 (.I0(d_w0[25+gdb_f]), .O(d_f7_t0[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7t1 (.I0(d_w0[25+gdb_f]), .O(d_f7_t1[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7t2 (.I0(d_w0[25+gdb_f]), .O(d_f7_t2[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7t3 (.I0(d_w0[25+gdb_f]), .O(d_f7_t3[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7c0 (.I0(d_w0[25+gdb_f]), .O(d_f7_c0[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bf7c1 (.I0(d_w0[25+gdb_f]), .O(d_f7_c1[gdb_f]));
		end
		for (gdb_f = 0; gdb_f < 5; gdb_f = gdb_f + 1) begin : g_br2
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_br2 (.I0(d_w0[20+gdb_f]), .O(d_rs2f[gdb_f]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb (.I0(d_w1[17+gdb_f]), .O(d_sub[gdb_f]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gdb_f;
	generate
		for (gdb_f = 0; gdb_f < 3; gdb_f = gdb_f + 1) begin : g_bf3
			assign d_f3_t0[gdb_f] = d_w0[12+gdb_f];
			assign d_f3_t1[gdb_f] = d_w0[12+gdb_f];
			assign d_f3_c0[gdb_f] = d_w0[12+gdb_f];
			assign d_f3_c1[gdb_f] = d_w0[12+gdb_f];
			assign d_f3_c2[gdb_f] = d_w0[12+gdb_f];
			assign d_f3_c3[gdb_f] = d_w0[12+gdb_f];
		end
		for (gdb_f = 0; gdb_f < 7; gdb_f = gdb_f + 1) begin : g_bf7
			assign d_f7_t0[gdb_f] = d_w0[25+gdb_f];
			assign d_f7_t1[gdb_f] = d_w0[25+gdb_f];
			assign d_f7_t2[gdb_f] = d_w0[25+gdb_f];
			assign d_f7_t3[gdb_f] = d_w0[25+gdb_f];
			assign d_f7_c0[gdb_f] = d_w0[25+gdb_f];
			assign d_f7_c1[gdb_f] = d_w0[25+gdb_f];
		end
		for (gdb_f = 0; gdb_f < 5; gdb_f = gdb_f + 1) begin : g_br2
			assign d_rs2f[gdb_f] = d_w0[20+gdb_f];
			assign d_sub[gdb_f] = d_w1[17+gdb_f];
		end
	endgenerate
`else
	assign d_f3_t0 = d_w0[14:12];
	assign d_f3_t1 = d_w0[14:12];
	assign d_f3_c0 = d_w0[14:12];
	assign d_f3_c1 = d_w0[14:12];
	assign d_f3_c2 = d_w0[14:12];
	assign d_f3_c3 = d_w0[14:12];
	assign d_f7_t0 = d_w0[31:25];
	assign d_f7_t1 = d_w0[31:25];
	assign d_f7_t2 = d_w0[31:25];
	assign d_f7_t3 = d_w0[31:25];
	assign d_f7_c0 = d_w0[31:25];
	assign d_f7_c1 = d_w0[31:25];
	assign d_rs2f  = d_w0[24:20];
	assign d_sub   = d_w1[21:17];
`endif
	wire [2:0] d_f3 = d_f3_t0;
	wire [6:0] d_f7 = d_f7_t0;
	wire [6:0] d_rs1 = {d_w1[3:2], d_w0[19:15]};
	wire [31:0] d_hi = {{12{d_w1[31]}}, d_w1[31:22], 10'd0} << 2;

	/* The table lookup: the RISC-V opcodes and custom-0..3. */
	wire [6:0] d_opc_tab_r, d_opc_tab_c, d_opc_ctl;
`ifdef SYNTH_KINTEX7
	genvar g_opc;
	generate
		for (g_opc = 0; g_opc < 7; g_opc = g_opc + 1) begin : g_buf_opc
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bopc_tr(.I0(d_opc[g_opc]), .O(d_opc_tab_r[g_opc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bopc_tc(.I0(d_opc[g_opc]), .O(d_opc_tab_c[g_opc]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bopc_c (.I0(d_opc[g_opc]), .O(d_opc_ctl[g_opc]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_opc;
	generate
		for (g_opc = 0; g_opc < 7; g_opc = g_opc + 1) begin : g_buf_opc
			assign d_opc_tab_r[g_opc] = d_opc[g_opc];
			assign d_opc_tab_c[g_opc] = d_opc[g_opc];
			assign d_opc_ctl[g_opc] = d_opc[g_opc];
		end
	endgenerate
`else
	assign d_opc_tab_r = d_opc;
	assign d_opc_tab_c = d_opc;
	assign d_opc_ctl   = d_opc;
`endif
	wire [28:0] tr, tc;
	wire [28:0] d_tab_out;
	wire [31:0] d_imm_isb;
	wire im_co_unused;
	wire [6:0] d_rs1_2;
	wire r2_co_unused;
	am_add #(.W(7)) u_rs1_2 (.a(d_rs1), .b(7'd2), .ci(1'b0), .s(d_rs1_2), .co(r2_co_unused));

	/* In RISC-V / custom encoding, FM_S only occurs for STORE (0100011), sts (0001011 f3=1),
	 * and exp (0101011 f3=4); FM_B only occurs for BRANCH (1100011). */
	wire d_is_b = (d_opc_ctl == 7'b1100011);
	wire d_is_s = (d_opc_ctl == 7'b0100011) ||
		(d_opc_ctl == 7'b0001011 && d_f3 == 3'd1) ||
		(d_opc_ctl == 7'b0101011 && d_f3 == 3'd4);
	wire d_is_u = (d_opc_ctl == 7'b0110111);
	wire [31:0] d_lo_i = {{20{d_w0[31]}}, d_w0[31:20]};
	wire [31:0] d_lo_s = {{20{d_w0[31]}}, d_w0[31:25], d_w0[11:7]};
	wire [31:0] d_lo_b = {{21{d_w0[31]}}, d_w0[7], d_w0[30:25], d_w0[11:8]};
	wire [31:0] d_imm_a_w = d_is_s ? d_lo_s : d_is_b ? d_lo_b : d_lo_i;

	wire [6:0] d_rd = {d_w1[1:0], d_w0[11:7]};
	wire [6:0] d_rs2 = {d_w1[5:4], d_rs2f};
	wire [6:0] d_rs3f = {d_w1[7:6], d_w0[31:27]};

	/* Operand checks (design §3.4.1: registers beyond NREGS, rq without SA). */
	function src_bad;
		input [6:0] r;
		begin
			if (r == 7'd63)
				src_bad = HAS_SA == 0;
			else
				src_bad = !r[6] && ({25'd0, r} >= NREGS);
		end
	endfunction

	wire d_rs1_lt_60 = (d_rs1 < 7'd60);
	wire d_rs1_lt_61 = (d_rs1 < 7'd61);
	wire d_rs1_lt_62 = (d_rs1 < 7'd62);
	wire d_rs1_bad_t4 = !d_rs1[6] ? ({25'd0, d_rs1} >= NREGS - 3 || !d_rs1_lt_60) :
		(d_rs1 == 7'd93 || d_rs1 == 7'd94 || d_rs1 == 7'd95 || d_rs1 == 7'd125 || d_rs1 == 7'd126 || d_rs1 == 7'd127);
	wire d_rs1_bad_t3 = !d_rs1[6] ? ({25'd0, d_rs1} >= NREGS - 2 || !d_rs1_lt_61) :
		(d_rs1 == 7'd94 || d_rs1 == 7'd95 || d_rs1 == 7'd126 || d_rs1 == 7'd127);
	wire d_pair_bad = !d_rs1[6] ? ({25'd0, d_rs1} >= NREGS - 1 || !d_rs1_lt_62) :
		(d_rs1 == 7'd95 || d_rs1 == 7'd127);
	wire d_src1_bad = src_bad(d_rs1);
	wire d_src2_bad = src_bad(d_rs2);
	wire d_src3_bad = src_bad(d_rs3f);
	wire d_dst_bad_64 = d_rd[6];
	wire d_dst_bad_63 = (d_rd == 7'd63);
	wire d_dst_bad_tex = ({25'd0, d_rd} >= NREGS - 3) || (&d_rd[5:2]);
	wire d_dst_bad_ge = ({25'd0, d_rd} >= NREGS);
	wire d_rqs1 = (d_rs1 == 7'd63);
	wire d_rqs2 = (d_rs2 == 7'd63);
	wire d_rqs3 = (d_rs3f == 7'd63);
	wire d_rqs1_2 = (d_rs1_2 == 7'd63);
	wire d_w0_11_bad = (d_w0[1:0] != 2'b11);

	(* keep = "true" *) reg s0_valid;
	reg [WARP_W-1:0] s0_warp;
	reg [15:0] s0_pc;
	reg [31:0] s0_w0, s0_w1;
	reg [28:0] s0_tab;
	reg [6:0] s0_rs1_2;
	reg [31:0] s0_imm_isb;
	(* dont_touch = "true" *) reg [2:0] s0_not_fmu_r;
	reg s0_rs1_bad_t4_r, s0_rs1_bad_t3_r, s0_pair_bad_r, s0_tthree_r;
	reg s0_src1_bad_r, s0_src2_bad_r, s0_src3_bad_r;
	reg s0_dst_bad_64_r, s0_dst_bad_63_r, s0_dst_bad_tex_r, s0_dst_bad_ge_r;
	reg s0_rqs1_r, s0_rqs2_r, s0_rqs3_r, s0_rqs1_2_r, s0_w0_11_bad_r;

	wire u_adv = !u_valid || u_ready;
	(* dont_touch = "true" *) wire rst_p = !rst_n;

	generate
	if (PIPE_TAB != 0) begin : g_pipe_tab2
		/* Stage 0a: Parallel constant-opc sub-table pre-decodes (<= 9 cmos3) -> p0_* FFs */
		wire [28:0] tr_op, tr_opimm, tr_fp, tr_other;
		wire [28:0] tc_c0, tc_c1, tc_c2, tc_c3;
		wire d_is_op    = (d_opc_ctl == 7'b0110011);
		wire d_is_opimm = (d_opc_ctl == 7'b0010011);
		wire d_is_opfp  = (d_opc_ctl == 7'b1010011);
		wire d_is_c0    = (d_opc_ctl == 7'b0001011);
		wire d_is_c1    = (d_opc_ctl == 7'b0101011);
		wire d_is_c2    = (d_opc_ctl == 7'b1011011);
		wire d_is_c3    = (d_opc_ctl == 7'b1111011);
		wire d_is_other = !d_is_op && !d_is_opimm && !d_is_opfp && !d_is_c0 && !d_is_c1 && !d_is_c2 && !d_is_c3;

		atg_su_dec_tab tab_op (
			.opc(7'b0110011), .f3(d_f3_t0), .f7(d_f7_t0), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tr_op[0]), .fmt(tr_op[3:1]), .unit(tr_op[7:4]), .op(tr_op[15:8]), .cls(tr_op[18:16]),
			.use1(tr_op[19]), .use2(tr_op[20]), .use3(tr_op[21]), .wrd(tr_op[22]), .pair(tr_op[23]),
			.side(tr_op[24]), .need_pk(tr_op[25]), .need_sg(tr_op[26]), .need_sa(tr_op[27]), .tex(tr_op[28])
		);
		atg_su_dec_tab tab_opimm (
			.opc(7'b0010011), .f3(d_f3_t0), .f7(d_f7_t1), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tr_opimm[0]), .fmt(tr_opimm[3:1]), .unit(tr_opimm[7:4]), .op(tr_opimm[15:8]), .cls(tr_opimm[18:16]),
			.use1(tr_opimm[19]), .use2(tr_opimm[20]), .use3(tr_opimm[21]), .wrd(tr_opimm[22]), .pair(tr_opimm[23]),
			.side(tr_opimm[24]), .need_pk(tr_opimm[25]), .need_sg(tr_opimm[26]), .need_sa(tr_opimm[27]), .tex(tr_opimm[28])
		);
		atg_su_dec_tab tab_fp (
			.opc(7'b1010011), .f3(d_f3_t1), .f7(d_f7_t2), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tr_fp[0]), .fmt(tr_fp[3:1]), .unit(tr_fp[7:4]), .op(tr_fp[15:8]), .cls(tr_fp[18:16]),
			.use1(tr_fp[19]), .use2(tr_fp[20]), .use3(tr_fp[21]), .wrd(tr_fp[22]), .pair(tr_fp[23]),
			.side(tr_fp[24]), .need_pk(tr_fp[25]), .need_sg(tr_fp[26]), .need_sa(tr_fp[27]), .tex(tr_fp[28])
		);
		atg_su_dec_tab #(.OTHER_ONLY(1)) tab_other (
			.opc(d_opc_tab_r), .f3(d_f3_t1), .f7(d_f7_t3), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tr_other[0]), .fmt(tr_other[3:1]), .unit(tr_other[7:4]), .op(tr_other[15:8]), .cls(tr_other[18:16]),
			.use1(tr_other[19]), .use2(tr_other[20]), .use3(tr_other[21]), .wrd(tr_other[22]), .pair(tr_other[23]),
			.side(tr_other[24]), .need_pk(tr_other[25]), .need_sg(tr_other[26]), .need_sa(tr_other[27]), .tex(tr_other[28])
		);
		atg_su_dec_tabc tab_c0 (
			.opc(7'b0001011), .f3(d_f3_c0), .f7(d_f7_c0), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tc_c0[0]), .fmt(tc_c0[3:1]), .unit(tc_c0[7:4]), .op(tc_c0[15:8]), .cls(tc_c0[18:16]),
			.use1(tc_c0[19]), .use2(tc_c0[20]), .use3(tc_c0[21]), .wrd(tc_c0[22]), .pair(tc_c0[23]),
			.side(tc_c0[24]), .need_pk(tc_c0[25]), .need_sg(tc_c0[26]), .need_sa(tc_c0[27]), .tex(tc_c0[28])
		);
		atg_su_dec_tabc tab_c1 (
			.opc(7'b0101011), .f3(d_f3_c1), .f7(d_f7_c0), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tc_c1[0]), .fmt(tc_c1[3:1]), .unit(tc_c1[7:4]), .op(tc_c1[15:8]), .cls(tc_c1[18:16]),
			.use1(tc_c1[19]), .use2(tc_c1[20]), .use3(tc_c1[21]), .wrd(tc_c1[22]), .pair(tc_c1[23]),
			.side(tc_c1[24]), .need_pk(tc_c1[25]), .need_sg(tc_c1[26]), .need_sa(tc_c1[27]), .tex(tc_c1[28])
		);
		atg_su_dec_tabc tab_c2 (
			.opc(7'b1011011), .f3(d_f3_c2), .f7(d_f7_c1), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tc_c2[0]), .fmt(tc_c2[3:1]), .unit(tc_c2[7:4]), .op(tc_c2[15:8]), .cls(tc_c2[18:16]),
			.use1(tc_c2[19]), .use2(tc_c2[20]), .use3(tc_c2[21]), .wrd(tc_c2[22]), .pair(tc_c2[23]),
			.side(tc_c2[24]), .need_pk(tc_c2[25]), .need_sg(tc_c2[26]), .need_sa(tc_c2[27]), .tex(tc_c2[28])
		);
		atg_su_dec_tabc tab_c3 (
			.opc(7'b1111011), .f3(d_f3_c3), .f7(d_f7_c1), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tc_c3[0]), .fmt(tc_c3[3:1]), .unit(tc_c3[7:4]), .op(tc_c3[15:8]), .cls(tc_c3[18:16]),
			.use1(tc_c3[19]), .use2(tc_c3[20]), .use3(tc_c3[21]), .wrd(tc_c3[22]), .pair(tc_c3[23]),
			.side(tc_c3[24]), .need_pk(tc_c3[25]), .need_sg(tc_c3[26]), .need_sa(tc_c3[27]), .tex(tc_c3[28])
		);

		(* keep = "true" *) reg p0_valid;
		reg [WARP_W-1:0] p0_warp;
		reg [15:0] p0_pc;
		(* dont_touch = "true" *) reg [31:0] p0_w0, p0_w1;
		reg [6:0]  p0_rs1_2;
		(* dont_touch = "true" *) reg [31:0] p0_imm_a, p0_imm_b;
		reg [2:0]  p0_not_fmu_r;
		reg        p0_rs1_bad_t4_r, p0_rs1_bad_t3_r, p0_pair_bad_r;
		reg        p0_src1_bad_r, p0_src2_bad_r, p0_src3_bad_r;
		reg        p0_dst_bad_64_r, p0_dst_bad_63_r, p0_dst_bad_tex_r, p0_dst_bad_ge_r;
		reg        p0_rqs1_r, p0_rqs2_r, p0_rqs3_r, p0_rqs1_2_r, p0_w0_11_bad_r;
		reg [7:0]  p0_opc_sel;
		reg [28:0] p0_tr_op, p0_tr_opimm, p0_tr_fp, p0_tr_other;
		reg [28:0] p0_tc_c0, p0_tc_c1, p0_tc_c2, p0_tc_c3;

		wire s0_adv = !s0_valid || u_adv;
		wire p0_ready = !p0_valid || s0_adv;
		assign d_ready = p0_ready;
		wire p0_ce = p0_ready && d_valid;
		wire s0_ce = s0_adv && p0_valid;
		wire [19:0] p0_ce_b;
		wire [11:0] s0_ce_b;
		genvar gdce;
		for (gdce = 0; gdce < 20; gdce = gdce + 1) begin : g_p0ce
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pce (.I0(p0_ce), .O(p0_ce_b[gdce]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign p0_ce_b[gdce] = p0_ce;
`else
			assign p0_ce_b[gdce] = p0_ce;
`endif
		end
		for (gdce = 0; gdce < 12; gdce = gdce + 1) begin : g_s0ce
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sce (.I0(s0_ce), .O(s0_ce_b[gdce]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign s0_ce_b[gdce] = s0_ce;
`else
			assign s0_ce_b[gdce] = s0_ce;
`endif
		end

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				p0_valid        <= 1'b0;
				p0_warp         <= {WARP_W{1'b0}};
				p0_pc           <= 16'd0;
				p0_w0           <= 32'd0;
				p0_w1           <= 32'd0;
				p0_rs1_2        <= 7'd0;
				p0_imm_a        <= 32'd0;
				p0_imm_b        <= 32'd0;
				p0_not_fmu_r    <= 3'd0;
				p0_rs1_bad_t4_r <= 1'b0;
				p0_rs1_bad_t3_r <= 1'b0;
				p0_pair_bad_r   <= 1'b0;
				p0_src1_bad_r   <= 1'b0;
				p0_src2_bad_r   <= 1'b0;
				p0_src3_bad_r   <= 1'b0;
				p0_dst_bad_64_r <= 1'b0;
				p0_dst_bad_63_r <= 1'b0;
				p0_dst_bad_tex_r <= 1'b0;
				p0_dst_bad_ge_r <= 1'b0;
				p0_rqs1_r       <= 1'b0;
				p0_rqs2_r       <= 1'b0;
				p0_rqs3_r       <= 1'b0;
				p0_rqs1_2_r     <= 1'b0;
				p0_w0_11_bad_r  <= 1'b0;
				p0_opc_sel      <= 8'd0;
				p0_tr_op        <= 29'd0;
				p0_tr_opimm     <= 29'd0;
				p0_tr_fp        <= 29'd0;
				p0_tr_other     <= 29'd0;
				p0_tc_c0        <= 29'd0;
				p0_tc_c1        <= 29'd0;
				p0_tc_c2        <= 29'd0;
				p0_tc_c3        <= 29'd0;
			end else begin
				if (p0_ready)
					p0_valid <= d_valid;
				if (p0_ce_b[0]) begin
					p0_warp         <= d_warp;
					p0_rs1_2        <= d_rs1_2;
					p0_not_fmu_r    <= {3{!d_is_u}};
				end
				if (p0_ce_b[1])  p0_pc <= d_pc;
				if (p0_ce_b[2])  p0_w0[15:0]  <= d_w0[15:0];
				if (p0_ce_b[3])  p0_w0[31:16] <= d_w0[31:16];
				if (p0_ce_b[4])  p0_w1[15:0]  <= d_w1[15:0];
				if (p0_ce_b[5])  p0_w1[31:16] <= d_w1[31:16];
				if (p0_ce_b[6])  p0_imm_a[15:0]  <= d_imm_a_w[15:0];
				if (p0_ce_b[7])  p0_imm_a[31:16] <= d_imm_a_w[31:16];
				if (p0_ce_b[8])  p0_imm_b[15:0]  <= d_hi[15:0];
				if (p0_ce_b[9])  p0_imm_b[31:16] <= d_hi[31:16];
				if (p0_ce_b[10]) begin
					p0_rs1_bad_t4_r <= d_rs1_bad_t4;
					p0_rs1_bad_t3_r <= d_rs1_bad_t3;
					p0_pair_bad_r   <= d_pair_bad;
					p0_src1_bad_r   <= d_src1_bad;
					p0_src2_bad_r   <= d_src2_bad;
					p0_src3_bad_r   <= d_src3_bad;
					p0_dst_bad_64_r <= d_dst_bad_64;
					p0_dst_bad_63_r <= d_dst_bad_63;
					p0_dst_bad_tex_r <= d_dst_bad_tex;
					p0_dst_bad_ge_r <= d_dst_bad_ge;
					p0_rqs1_r       <= d_rqs1;
					p0_rqs2_r       <= d_rqs2;
					p0_rqs3_r       <= d_rqs3;
					p0_rqs1_2_r     <= d_rqs1_2;
					p0_w0_11_bad_r  <= d_w0_11_bad;
				end
				if (p0_ce_b[11]) begin
					p0_opc_sel      <= {d_is_c3, d_is_c2, d_is_c1, d_is_c0, d_is_other, d_is_opfp, d_is_opimm, d_is_op};
					p0_tr_op[7:0]   <= tr_op[7:0];
				end
				if (p0_ce_b[12]) begin
					p0_tr_op[28:8]  <= tr_op[28:8];
					p0_tr_opimm     <= tr_opimm;
				end
				if (p0_ce_b[13]) p0_tr_fp    <= tr_fp;
				if (p0_ce_b[14]) p0_tc_c0    <= tc_c0;
				if (p0_ce_b[15]) p0_tc_c2    <= tc_c2;
				if (p0_ce_b[16]) p0_tr_other <= tr_other;
				if (p0_ce_b[17]) p0_tc_c1    <= tc_c1;
				if (p0_ce_b[18]) p0_tc_c3    <= tc_c3;
				if (p0_ce_b[19]) ;
			end
		end

		/* Stage 0b: One-hot SOP combine + 32-bit immediate adder (<= 10 cmos3) -> s0_* FFs */
		wire [31:0] p0_imm_isb;
		am_add #(.W(32)) u_imm (.a(p0_imm_a), .b(p0_imm_b), .ci(1'b0), .s(p0_imm_isb), .co(im_co_unused));
		assign d_imm_isb = 32'd0;
		wire [28:0] p0_tab_nxt =
			(p0_opc_sel[0] ? p0_tr_op    : 29'd0) |
			(p0_opc_sel[1] ? p0_tr_opimm : 29'd0) |
			(p0_opc_sel[2] ? p0_tr_fp    : 29'd0) |
			(p0_opc_sel[3] ? p0_tr_other : 29'd0) |
			(p0_opc_sel[4] ? p0_tc_c0    : 29'd0) |
			(p0_opc_sel[5] ? p0_tc_c1    : 29'd0) |
			(p0_opc_sel[6] ? p0_tc_c2    : 29'd0) |
			(p0_opc_sel[7] ? p0_tc_c3    : 29'd0);
		wire p0_tthree_nxt = p0_tab_nxt[28] && p0_w0[30];

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				s0_valid        <= 1'b0;
				s0_warp         <= {WARP_W{1'b0}};
				s0_pc           <= 16'd0;
				s0_w0           <= 32'd0;
				s0_w1           <= 32'd0;
				s0_tab          <= 29'd0;
				s0_rs1_2        <= 7'd0;
				s0_imm_isb      <= 32'd0;
				s0_not_fmu_r    <= 3'd0;
				s0_rs1_bad_t4_r <= 1'b0;
				s0_rs1_bad_t3_r <= 1'b0;
				s0_pair_bad_r   <= 1'b0;
				s0_tthree_r     <= 1'b0;
				s0_src1_bad_r   <= 1'b0;
				s0_src2_bad_r   <= 1'b0;
				s0_src3_bad_r   <= 1'b0;
				s0_dst_bad_64_r <= 1'b0;
				s0_dst_bad_63_r <= 1'b0;
				s0_dst_bad_tex_r <= 1'b0;
				s0_dst_bad_ge_r <= 1'b0;
				s0_rqs1_r       <= 1'b0;
				s0_rqs2_r       <= 1'b0;
				s0_rqs3_r       <= 1'b0;
				s0_rqs1_2_r     <= 1'b0;
				s0_w0_11_bad_r  <= 1'b0;
			end else begin
				if (s0_adv)
					s0_valid <= p0_valid;
				if (s0_ce_b[0]) begin
					s0_warp         <= p0_warp;
					s0_rs1_2        <= p0_rs1_2;
					s0_not_fmu_r    <= p0_not_fmu_r;
				end
				if (s0_ce_b[1])  s0_pc <= p0_pc;
				if (s0_ce_b[2])  s0_w0[15:0]  <= p0_w0[15:0];
				if (s0_ce_b[3])  s0_w0[31:16] <= p0_w0[31:16];
				if (s0_ce_b[4])  s0_w1[15:0]  <= p0_w1[15:0];
				if (s0_ce_b[5])  s0_w1[31:16] <= p0_w1[31:16];
				if (s0_ce_b[6])  begin
					s0_tab[14:0] <= p0_tab_nxt[14:0];
					s0_tthree_r  <= p0_tthree_nxt;
				end
				if (s0_ce_b[7])  s0_tab[28:15] <= p0_tab_nxt[28:15];
				if (s0_ce_b[8])  s0_imm_isb[15:0]  <= p0_imm_isb[15:0];
				if (s0_ce_b[9])  s0_imm_isb[31:16] <= p0_imm_isb[31:16];
				if (s0_ce_b[10]) begin
					s0_rs1_bad_t4_r <= p0_rs1_bad_t4_r;
					s0_rs1_bad_t3_r <= p0_rs1_bad_t3_r;
					s0_pair_bad_r   <= p0_pair_bad_r;
					s0_src1_bad_r   <= p0_src1_bad_r;
					s0_src2_bad_r   <= p0_src2_bad_r;
					s0_src3_bad_r   <= p0_src3_bad_r;
					s0_dst_bad_64_r <= p0_dst_bad_64_r;
					s0_dst_bad_63_r <= p0_dst_bad_63_r;
				end
				if (s0_ce_b[11]) begin
					s0_dst_bad_tex_r <= p0_dst_bad_tex_r;
					s0_dst_bad_ge_r <= p0_dst_bad_ge_r;
					s0_rqs1_r       <= p0_rqs1_r;
					s0_rqs2_r       <= p0_rqs2_r;
					s0_rqs3_r       <= p0_rqs3_r;
					s0_rqs1_2_r     <= p0_rqs1_2_r;
					s0_w0_11_bad_r  <= p0_w0_11_bad_r;
				end
			end
		end
		assign tr = 29'd0;
		assign tc = 29'd0;
		assign d_tab_out = 29'd0;
		wire unused_pt2 = &{1'b0, d_opc_tab_c, d_rd, im_co_unused};
	end else begin : g_comb_tab
		wire custom_lo = d_opc_ctl == 7'b0001011 || d_opc_ctl == 7'b0101011 || d_opc_ctl == 7'b1011011 ||
			d_opc_ctl == 7'b1111011;
		wire custom_hi = d_opc_ctl == 7'b0001011 || d_opc_ctl == 7'b0101011 || d_opc_ctl == 7'b1011011 ||
			d_opc_ctl == 7'b1111011;
		atg_su_dec_tab tab (
			.opc(d_opc_tab_r), .f3(d_f3), .f7(d_f7), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tr[0]), .fmt(tr[3:1]), .unit(tr[7:4]), .op(tr[15:8]), .cls(tr[18:16]),
			.use1(tr[19]), .use2(tr[20]), .use3(tr[21]), .wrd(tr[22]), .pair(tr[23]),
			.side(tr[24]), .need_pk(tr[25]), .need_sg(tr[26]), .need_sa(tr[27]), .tex(tr[28])
		);
		atg_su_dec_tabc tabc (
			.opc(d_opc_tab_c), .f3(d_f3), .f7(d_f7), .rs2f(d_rs2f), .sub(d_sub), .fmt2(d_w0[26:25]),
			.def(tc[0]), .fmt(tc[3:1]), .unit(tc[7:4]), .op(tc[15:8]), .cls(tc[18:16]),
			.use1(tc[19]), .use2(tc[20]), .use3(tc[21]), .wrd(tc[22]), .pair(tc[23]),
			.side(tc[24]), .need_pk(tc[25]), .need_sg(tc[26]), .need_sa(tc[27]), .tex(tc[28])
		);
		wire [28:0] d_tab_comb = {custom_hi ? tc[28:15] : tr[28:15], custom_lo ? tc[14:0] : tr[14:0]};
		wire [2:0] d_fmt = d_tab_comb[3:1];
		assign d_tab_out = d_tab_comb;
		assign d_ready = u_adv;
		wire [31:0] d_imm_a_comb = (d_fmt == FM_S) ? d_lo_s : (d_fmt == FM_B) ? d_lo_b : d_lo_i;
		am_add #(.W(32)) u_imm (.a(d_imm_a_comb), .b(d_hi), .ci(1'b0), .s(d_imm_isb), .co(im_co_unused));
		always @(posedge clk) begin
			s0_valid <= 1'b0;
			s0_warp <= {WARP_W{1'b0}};
			s0_pc <= 16'd0;
			s0_w0 <= 32'd0;
			s0_w1 <= 32'd0;
			s0_tab <= 29'd0;
			s0_rs1_2 <= 7'd0;
			s0_imm_isb <= 32'd0;
			s0_not_fmu_r <= 3'd0;
			s0_rs1_bad_t4_r <= 1'b0;
			s0_rs1_bad_t3_r <= 1'b0;
			s0_pair_bad_r <= 1'b0;
			s0_tthree_r <= 1'b0;
		end
		wire unused_comb_tab = &{1'b0, d_imm_a_w, d_rs1_bad_t4, d_rs1_bad_t3, d_pair_bad, d_is_u, d_rd, d_rs1_2};
	end
	endgenerate

	wire [31:0] w0 = (PIPE_TAB != 0) ? s0_w0 : d_w0;
	wire [31:0] w1 = (PIPE_TAB != 0) ? s0_w1 : d_w1;
	wire [28:0] tab_out = (PIPE_TAB != 0) ? s0_tab : d_tab_out;
	wire [6:0] rs1_2 = (PIPE_TAB != 0) ? s0_rs1_2 : d_rs1_2;
	wire [31:0] imm_isb = (PIPE_TAB != 0) ? s0_imm_isb : d_imm_isb;

	wire [2:0] f3 = w0[14:12];
	wire [6:0] f7 = w0[31:25];
	wire [4:0] sub = w1[21:17];
	wire [6:0] rd = {w1[1:0], w0[11:7]};
	wire [6:0] rs1 = {w1[3:2], w0[19:15]};
	wire [6:0] rs2 = {w1[5:4], w0[24:20]};
	wire [6:0] rs3f = {w1[7:6], w0[31:27]};
	wire [31:0] imm_u = {w0[31:12], w1[13:2]};

	wire def, use1, use2, use3, wrd, pair, side, need_pk, need_sg, need_sa, tex;
	wire [2:0] fmt, cls;
	wire [3:0] unit;
	wire [7:0] op;
	assign {tex, need_sa, need_sg, need_pk, side, pair, wrd, use3, use2, use1, cls, op, unit,
		fmt, def} = tab_out;

	/* Operand checks use pre-decoded Stage-0 flags when PIPE_TAB != 0. */
	wire src1_bad = (PIPE_TAB != 0) ? s0_src1_bad_r : d_src1_bad;
	wire src2_bad = (PIPE_TAB != 0) ? s0_src2_bad_r : d_src2_bad;
	wire src3_bad = (PIPE_TAB != 0) ? s0_src3_bad_r : d_src3_bad;
	wire dst_bad_64  = (PIPE_TAB != 0) ? s0_dst_bad_64_r  : d_dst_bad_64;
	wire dst_bad_63  = (PIPE_TAB != 0) ? s0_dst_bad_63_r  : d_dst_bad_63;
	wire dst_bad_tex = (PIPE_TAB != 0) ? s0_dst_bad_tex_r : d_dst_bad_tex;
	wire dst_bad_ge  = (PIPE_TAB != 0) ? s0_dst_bad_ge_r  : d_dst_bad_ge;
	wire rqs1_w   = (PIPE_TAB != 0) ? s0_rqs1_r   : d_rqs1;
	wire rqs2_w   = (PIPE_TAB != 0) ? s0_rqs2_r   : d_rqs2;
	wire rqs3f_w  = (PIPE_TAB != 0) ? s0_rqs3_r   : d_rqs3;
	wire rqs1_2_w = (PIPE_TAB != 0) ? s0_rqs1_2_r : d_rqs1_2;
	wire w0_11_bad = (PIPE_TAB != 0) ? s0_w0_11_bad_r : d_w0_11_bad;

	/* A texture .c or .3 form reads rs1+2 (Dref or the third coordinate) as
	 * its rs3; .3 with .c also rs1+3 (Dref). */
	wire tthree = (PIPE_TAB != 0) ? s0_tthree_r : (tex && f7[5]);
	wire tcmp = tex && (op[ATG_TEX_CFORM] || tthree);
	wire [6:0] rs3 = tcmp ? rs1_2 : rs3f;
	wire t4 = tthree && op[ATG_TEX_CFORM];
	wire cmp_bad = (PIPE_TAB != 0) ? (tcmp && (t4 ? s0_rs1_bad_t4_r : s0_rs1_bad_t3_r)) :
		(tcmp && (t4 ? d_rs1_bad_t4 : d_rs1_bad_t3));
	/* A texture R form's texel offsets: word1 [25:22], [29:26], [13:10]. */
	wire [31:0] imm_t = {20'd0, w1[13:10], w1[29:26], w1[25:22]};
	wire [2:0] ndst = !wrd ? 3'd0 : tex ? 3'd4 : 3'd1;
	wire pair_bad = (PIPE_TAB != 0) ? (pair && s0_pair_bad_r) : (pair && d_pair_bad);
	wire dst_bad = wrd && (dst_bad_64 ||
		(dst_bad_63 ? (HAS_SA == 0 || cls != ATG_CLS_A || tex) :
		(tex ? dst_bad_tex : dst_bad_ge)));
	/* texsz.s (texsz with op[6] and op[3]) needs MS4. */
	wire need_ms = unit == ATG_UNIT_TEX && op[1:0] == 2'd3 && op[6] && op[3];
	(* keep = "true" *) wire ill = w0_11_bad || !def || (need_pk && PACKED == 0) || (need_ms && MS4 == 0) ||
		(need_sg && SUBGROUP == 0) || (need_sa && HAS_SA == 0) ||
		(use1 && src1_bad) || (use2 && src2_bad) || (use3 && !tcmp && src3_bad) ||
		pair_bad || cmp_bad || dst_bad;
	wire rqs = (use1 && rqs1_w) || (use2 && rqs2_w) || (use3 && (tcmp ? rqs1_2_w : rqs3f_w));
	wire rqd = wrd && dst_bad_63;

	(* keep = "true" *) wire [2:0] fmt_lo = fmt;
	(* keep = "true" *) wire [2:0] fmt_hi = fmt;
	wire not_fmu_0 = (PIPE_TAB != 0) ? s0_not_fmu_r[0] : (fmt != FM_U);
	wire not_fmu_1 = (PIPE_TAB != 0) ? s0_not_fmu_r[1] : (fmt != FM_U);
	wire not_fmu_2 = (PIPE_TAB != 0) ? s0_not_fmu_r[2] : (fmt != FM_U);
	reg [31:0] imm;
	always @(*) begin
		case (fmt_lo)
		FM_I, FM_S, FM_B: imm[15:0] = imm_isb[15:0];
		FM_U: imm[15:0] = imm_u[15:0];
		default: imm[15:0] = tex ? imm_t[15:0] : 16'd0;
		endcase
		case (fmt_hi)
		FM_I, FM_S, FM_B: imm[31:16] = imm_isb[31:16];
		FM_U: imm[31:16] = imm_u[31:16];
		default: imm[31:16] = tex ? imm_t[31:16] : 16'd0;
		endcase
	end

	reg [UOP_W-1:0] uop_raw;
	always @(*) begin
		uop_raw = {UOP_W{1'b0}};
		uop_raw[ATG_UOP_UNIT +: 4] = unit;
		uop_raw[ATG_UOP_OP +: 8] = op;
		uop_raw[ATG_UOP_RD +: 7] = rd;
		uop_raw[ATG_UOP_IMM +: 32] = imm;
		uop_raw[ATG_UOP_CLS +: 3] = cls;
		uop_raw[ATG_UOP_NDST +: 3] = ndst;
		uop_raw[ATG_UOP_USE1] = use1;
		uop_raw[ATG_UOP_USE2] = use2;
		uop_raw[ATG_UOP_USE3] = use3;
		uop_raw[ATG_UOP_WRD] = wrd;
		uop_raw[ATG_UOP_PAIR] = pair;
		uop_raw[ATG_UOP_RQS] = rqs;
		uop_raw[ATG_UOP_RQD] = rqd;
		uop_raw[ATG_UOP_SIDE] = side;
		if (not_fmu_0) begin
			uop_raw[ATG_UOP_RS1 +: 7] = rs1;
			uop_raw[ATG_UOP_RS2 +: 7] = rs2;
		end
		if (not_fmu_1) begin
			uop_raw[ATG_UOP_RS3 +: 7] = rs3;
			uop_raw[ATG_UOP_MOD +: 7] = w1[14:8];
		end
		if (not_fmu_2) begin
			uop_raw[ATG_UOP_HSEL +: 2] = w1[16:15];
			uop_raw[ATG_UOP_SUB +: 5] = {sub[4] | tthree, sub[3:0]};
			uop_raw[ATG_UOP_RM +: 3] = f3;
			uop_raw[ATG_UOP_SAMP_SLOT +: 4] = tex ? w1[9:6] : 4'd0;
		end
	end

	wire [UOP_W-1:0] uop;
	localparam U_SL = (UOP_W + 15) / 16;
	genvar gus;
	generate
		for (gus = 0; gus < U_SL; gus = gus + 1) begin : g_uop_sl
			localparam SL_B = gus * 16;
			localparam SL_W = (SL_B + 16 <= UOP_W) ? 16 : (UOP_W - SL_B);
			(* keep = "true" *) wire ill_sl = ill;
			(* keep = "true" *) wire u_ce_sl = u_adv && ((PIPE_TAB != 0) ? s0_valid : d_valid);
			wire [UOP_W-1:0] ill_vec = ({{(UOP_W-1){1'b0}}, 1'b1} << ATG_UOP_ILL);
			assign uop[SL_B +: SL_W] = ill_sl ? ill_vec[SL_B +: SL_W] : uop_raw[SL_B +: SL_W];
			reg [SL_W-1:0] u_uop_sl_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					u_uop_sl_r <= {SL_W{1'b0}};
				else if (u_ce_sl)
					u_uop_sl_r <= uop[SL_B +: SL_W];
			end
			assign u_uop[SL_B +: SL_W] = u_uop_sl_r;
		end
	endgenerate

	wire stg_valid = (PIPE_TAB != 0) ? s0_valid : d_valid;
	wire [WARP_W-1:0] stg_warp = (PIPE_TAB != 0) ? s0_warp : d_warp;
	wire [15:0] stg_pc = (PIPE_TAB != 0) ? s0_pc : d_pc;
	(* keep = "true" *) wire u_ce_hdr = u_adv && stg_valid;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			u_valid <= 1'b0;
			u_warp <= {WARP_W{1'b0}};
			u_pc <= 16'd0;
		end else begin
			if (u_adv)
				u_valid <= stg_valid;
			if (u_ce_hdr) begin
				u_warp <= stg_warp;
				u_pc <= stg_pc;
			end
		end
	end

	wire unused_ok = &{1'b0, r2_co_unused, im_co_unused, f7[6], f7[4:0]};
endmodule

`default_nettype wire
