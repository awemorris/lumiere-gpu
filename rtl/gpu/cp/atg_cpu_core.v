/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cpu_core: The RV32IM pipeline of the CP's control processor: fetch,
 * decode/register read, execute, writeback (4 stages), Machine mode only,
 * one interrupt line, no CSRs (the control window replaces them).
 *
 * F: i_en with i_addr = pc_f; the word comes back on i_rdata the next
 * cycle and stays there while i_en is 0 (BRAM output register).
 * D: register read of i_rdata's rs1/rs2 (with W's value of this cycle)
 * into E's operand registers, and pre-decode of ALU opcode/immediates
 * into E's control registers (timing at 200 MHz).  E: instruction decode,
 * atg_cpu_alu, branch resolve, the data access (d_valid held with its
 * payload until d_ready) and the atg_cpu_muldiv start; its operands take
 * W's value when W holds the instruction just before (program order).
 * W: the result register and the register file write.  A taken branch,
 * jump, mret or interrupt refetches (2 bubbles).
 *
 * run = 0 stops before the next instruction starts (an access or a
 * multiply/divide already started completes); stopped is then 1.  An
 * interrupt (irq = 1, not in the handler) is taken before an instruction
 * starts: the hidden mepc gets its PC (a WFI's next PC), the core jumps
 * to 0x10, and mret (0x30200073) returns.  WFI waits for irq.  A fault
 * (illegal instruction, CSR/ECALL/EBREAK, misaligned access or jump
 * target, fetch outside 0x0000_0000-0x0001_FFFF) pulses fault with the
 * PC and stops the core until reset.  See spec/gpu/cp/atg_cpu_core.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cpu_core (
	input wire clk,
	input wire rst_n,
	input wire run,
	output wire stopped,
	input wire irq,
	/* Instruction port (IMEM, synchronous BRAM read) */
	output wire i_en,
	output wire [31:0] i_addr,
	input wire [31:0] i_rdata,
	/* Data port (to atg_cpu_win) */
	output wire d_valid,
	output wire d_write,
	output wire [31:0] d_addr,
	output reg [31:0] d_wdata,
	output reg [3:0] d_be,
	input wire d_ready,
	input wire [31:0] d_rdata,
	/* Faults (to atg_cpu_fw) */
	output reg fault,
	output reg [31:0] fault_pc,
	/* Diagnostics */
	output wire [31:0] curr_pc,
	output wire [3:0] curr_mcause,
	output wire [31:0] curr_mepc,
	output wire curr_in_irq,
	output wire curr_faulted
);
	localparam [31:0] IRQ_VECTOR = 32'h0000_0010;
	/* SYSTEM: only these two; CSR*, ECALL, EBREAK fault. */
	localparam [31:0] MRET = 32'h3020_0073;
	localparam [31:0] WFI = 32'h1050_0073;

	localparam [6:0] OPC_LUI = 7'b0110111;
	localparam [6:0] OPC_AUIPC = 7'b0010111;
	localparam [6:0] OPC_JAL = 7'b1101111;
	localparam [6:0] OPC_JALR = 7'b1100111;
	localparam [6:0] OPC_BRANCH = 7'b1100011;
	localparam [6:0] OPC_LOAD = 7'b0000011;
	localparam [6:0] OPC_STORE = 7'b0100011;
	localparam [6:0] OPC_IMM = 7'b0010011;
	localparam [6:0] OPC_OP = 7'b0110011;
	localparam [6:0] OPC_FENCE = 7'b0001111;

	/* atg_cpu_alu op codes. */
	localparam [3:0] A_ADD = 4'd0;
	localparam [3:0] A_SUB = 4'd1;
	localparam [3:0] A_SLL = 4'd2;
	localparam [3:0] A_SLT = 4'd3;
	localparam [3:0] A_SLTU = 4'd4;
	localparam [3:0] A_XOR = 4'd5;
	localparam [3:0] A_SRL = 4'd6;
	localparam [3:0] A_SRA = 4'd7;
	localparam [3:0] A_OR = 4'd8;
	localparam [3:0] A_AND = 4'd9;
	localparam [3:0] A_PASS_B = 4'd10;

	/* Pipeline state. */
	reg [31:0] pc_f;	/* address fetched this cycle */
	/* 32-bit alternating AOI/OAI Kogge-Stone adder (<= 8 cmos3, zero sc8->sc24 ripple). */
	function [32:0] fn_ks_add32;
		input [31:0] a_in;
		input [31:0] b_in;
		input ci_in;
		reg [31:0] x0, g0_n, p0_n, g1, p1, g2_n, p2_n, g3, p3, g4_n, p4_n, g5, sum_w;
		begin
			x0      = a_in ^ b_in;
			g0_n    = ~(a_in & b_in);
			p0_n    = ~(a_in | b_in);
			g0_n[0] = ~((a_in[0] & b_in[0]) | ((a_in[0] | b_in[0]) & ci_in));
			g1      = ~(g0_n & (p0_n | {g0_n[30:0], 1'b1}));
			p1      = ~(p0_n | {p0_n[30:0], 1'b1});
			g2_n    = ~(g1 | (p1 & {g1[29:0], 2'b00}));
			p2_n    = ~(p1 & {p1[29:0], 2'b00});
			g3      = ~(g2_n & (p2_n | {g2_n[27:0], 4'hF}));
			p3      = ~(p2_n | {p2_n[27:0], 4'hF});
			g4_n    = ~(g3 | (p3 & {g3[23:0], 8'h00}));
			p4_n    = ~(p3 & {p3[23:0], 8'h00});
			g5      = ~(g4_n & (p4_n | {g4_n[15:0], 16'hFFFF}));
			sum_w   = x0 ^ {g5[30:0], ci_in};
			fn_ks_add32 = {g5[31], sum_w};
		end
	endfunction

	/* 32-bit alternating AOI/OAI parallel-prefix comparator (a_in < b_in, <= 7 cmos3). */
	function fn_cmp_lt;
		input [31:0] a_in;
		input [31:0] b_in;
		reg [31:0] g0_n, t0_n, g1, t1, g2_n, t2_n, g3, t3, g4_n, t4_n, g5;
		begin
			g0_n = ~(b_in & ~a_in);
			t0_n = ~(b_in | ~a_in);
			g1   = ~(g0_n & (t0_n | {g0_n[30:0], 1'b1}));
			t1   = ~(t0_n | {t0_n[30:0], 1'b1});
			g2_n = ~(g1 | (t1 & {g1[29:0], 2'b00}));
			t2_n = ~(t1 & {t1[29:0], 2'b00});
			g3   = ~(g2_n & (t2_n | {g2_n[27:0], 4'hF}));
			t3   = ~(t2_n | {t2_n[27:0], 4'hF});
			g4_n = ~(g3 | (t3 & {g3[23:0], 8'h00}));
			t4_n = ~(t3 & {t3[23:0], 8'h00});
			g5   = ~(g4_n & (t4_n | {g4_n[15:0], 16'hFFFF}));
			fn_cmp_lt = g5[31];
		end
	endfunction

	wire [32:0] pc_f_res = fn_ks_add32(pc_f, 32'd4, 1'b0);
	wire [31:0] pc_f_4 = pc_f_res[31:0];	/* pc_f + 4 */
	wire pc_f_co = pc_f_res[32];
	(* DONT_TOUCH = "TRUE" *) reg [31:0] pc_d;	/* address of the instruction in D (i_rdata) */
	reg [31:0] pc_d4;
	reg d_vld;
	(* DONT_TOUCH = "TRUE" *) reg [31:0] pc_e;	/* address of the instruction in E */
	reg [31:0] pc_e4;
	reg [4:0] rd_e;
	reg [2:0] f3_e;
	reg rd_nz_e;
	reg is_auipc_e, is_jal_e, is_jalr_e, is_br_e, is_ld_e, is_st_e;
	reg is_m_e, is_mret_e, is_wfi_e, is_fast_e, legal_e, is_mem_e, has_rd_e, pc_out_e;
	reg br_sel_j_e, br_sel_eq_e, br_sel_lt_e, br_sel_ltu_e;
	reg [31:0] ea_b_e, pc_imm_b_e;
	(* DONT_TOUCH = "TRUE" *) reg [31:0] rs1_e, rs2_e;	/* its operands as read in D */
	(* keep = "true" *) reg fwd1_e0, fwd1_e1, fwd1_e2, fwd1_e3;
	(* keep = "true" *) reg fwd2_e0, fwd2_e1, fwd2_e2, fwd2_e3;	/* take them from W instead (decided in D: timing) */
	reg e_vld;
	reg e_busy;	/* its access or multiply/divide has started */
	reg mem_prep_r, mem_req_r, mem_done_r;
	reg d_valid_r, d_write_r;
	reg [31:0] d_addr_r, mem_rdata_r;
	reg [1:0] ea_r;
	reg jump_s1_r, jump_pend_r, br_eq_p_r, br_lt_p_r, br_ltu_p_r;
	reg md_s1_r;
	reg [31:0] jump_tgt_r;
	reg [31:0] op1_r, op2_r;
	(* DONT_TOUCH = "TRUE" *) reg w_wen;	/* W: the last completed instruction writes w_rd */
	(* DONT_TOUCH = "TRUE" *) reg [4:0] w_rd;
	(* DONT_TOUCH = "TRUE" *) reg [31:0] w_val;
	(* DONT_TOUCH = "TRUE" *) reg [3:0] alu_op_e;
	(* DONT_TOUCH = "TRUE" *) reg [31:0] alu_b_imm_e;
	(* DONT_TOUCH = "TRUE" *) reg alu_b_sel_imm_e;
	(* DONT_TOUCH = "TRUE" *) reg is_alu_e;
	reg faulted;
	reg in_irq;
	reg [31:0] mepc;
	reg [3:0] mcause_q;
	/* x1..x31; rf[0] is never read (zero-based: an index needs no subtraction). */
	reg [31:0] rf [0:31];

	/* D: register read, with the value W writes in this cycle. */
	wire [4:0] d_ra = i_rdata[19:15];
	wire [4:0] d_rb = i_rdata[24:20];
	wire [31:0] d_rs1 = (d_ra == 5'd0) ? 32'd0 : (w_wen && w_rd == d_ra) ? w_val : rf[d_ra];
	wire [31:0] d_rs2 = (d_rb == 5'd0) ? 32'd0 : (w_wen && w_rd == d_rb) ? w_val : rf[d_rb];

	/* D: decode of i_rdata for E-stage ALU pre-computation (timing at 200 MHz).
	 * Only ALU instructions (LUI, OP-IMM, OP) need pre-decoding here; loads/stores
	 * use ea_add directly, and JALR computes its target via jalr_t in stage E. */
	wire [6:0] d_opc = i_rdata[6:0];
	wire [2:0] d_f3  = i_rdata[14:12];
	wire [6:0] d_f7  = i_rdata[31:25];

	wire [31:0] d_imm_i = {{20{i_rdata[31]}}, i_rdata[31:20]};
	wire [31:0] d_imm_s = {{20{i_rdata[31]}}, i_rdata[31:25], i_rdata[11:7]};
	wire [31:0] d_imm_b = {{19{i_rdata[31]}}, i_rdata[31], i_rdata[7], i_rdata[30:25], i_rdata[11:8], 1'b0};
	wire [31:0] d_imm_u = {i_rdata[31:12], 12'd0};
	wire [31:0] d_imm_j = {{11{i_rdata[31]}}, i_rdata[31], i_rdata[19:12], i_rdata[20], i_rdata[30:21], 1'b0};

	wire d_opc_op  = d_opc[1:0] == 2'b11;
	wire [4:0] d_op5 = d_opc[6:2];
	wire d_is_lui   = d_opc_op && d_op5 == 5'b01101;
	wire d_is_auipc = d_opc_op && d_op5 == 5'b00101;
	wire d_is_jal   = d_opc_op && d_op5 == 5'b11011;
	wire d_is_jalr  = d_opc_op && d_op5 == 5'b11001 && d_f3 == 3'd0;
	wire d_is_br    = d_opc_op && d_op5 == 5'b11000 && d_f3[2:1] != 2'b01;
	wire d_is_ld    = d_opc_op && d_op5 == 5'b00000 && (d_f3 == 3'd0 || d_f3 == 3'd1 || d_f3 == 3'd2 ||
		d_f3 == 3'd4 || d_f3 == 3'd5);
	wire d_is_st    = d_opc_op && d_op5 == 5'b01000 && (d_f3 == 3'd0 || d_f3 == 3'd1 || d_f3 == 3'd2);
	wire d_f7_0  = (d_f7 == 7'd0);
	wire d_f7_20 = (d_f7 == 7'h20);
	wire d_imm_bad  = (d_f3 == 3'd1 && !d_f7_0) || (d_f3 == 3'd5 && !d_f7_0 && !d_f7_20);
	wire d_is_imm   = d_opc_op && d_op5 == 5'b00100 && !d_imm_bad;
	wire d_is_op    = d_opc_op && d_op5 == 5'b01100 && (d_f7_0 || (d_f7_20 && (d_f3 == 3'd0 || d_f3 == 3'd5)));
	wire d_is_m     = d_opc_op && d_op5 == 5'b01100 && d_f7 == 7'd1;
	wire d_is_fence = d_opc_op && d_op5 == 5'b00011 && d_f3[2:1] == 2'b00;
	wire d_is_mret  = (i_rdata == 32'h3020_0073);
	wire d_is_wfi   = (i_rdata == 32'h1050_0073);
	wire d_leg_a = d_is_lui || d_is_auipc || d_is_jal || d_is_jalr;
	wire d_leg_b = d_is_br || d_is_ld || d_is_st || d_is_fence;
	wire d_leg_c = d_is_imm || d_is_op || d_is_m;
	wire d_leg_s = d_is_mret || d_is_wfi;
	wire d_legal = (d_leg_a || d_leg_b) || (d_leg_c || d_leg_s);
	wire d_is_mem   = d_is_ld || d_is_st;
	wire d_has_rd   = d_is_lui || d_is_auipc || d_is_jal || d_is_jalr || d_is_ld || d_is_imm || d_is_op || d_is_m;
	wire [31:0] d_ea_b     = d_is_st ? d_imm_s : d_imm_i;
	wire [31:0] d_pc_imm_b = d_is_jal ? d_imm_j : (d_is_br ? d_imm_b : d_imm_u);

	wire [31:0] d_alu_b_imm = (d_is_lui || d_is_auipc) ? d_imm_u : d_imm_i;
	wire d_alu_b_sel_imm = d_is_lui || d_is_imm || d_is_auipc;
	wire d_is_alu = d_is_lui || d_is_imm || d_is_op || d_is_auipc;
	wire d_is_fast = !(d_is_mem || d_is_br || d_is_jalr || d_is_jal) && !(d_is_alu || d_is_m || d_is_wfi);

	wire [3:0] d_alu_norm =
		(d_f3 == 3'd0) ? ((d_is_op && d_f7[5]) ? A_SUB : A_ADD) :
		(d_f3 == 3'd1) ? A_SLL :
		(d_f3 == 3'd2) ? A_SLT :
		(d_f3 == 3'd3) ? A_SLTU :
		(d_f3 == 3'd4) ? A_XOR :
		(d_f3 == 3'd5) ? (d_f7[5] ? A_SRA : A_SRL) :
		(d_f3 == 3'd6) ? A_OR : A_AND;
	wire [3:0] d_alu_op = d_is_lui ? A_PASS_B : (d_is_auipc ? A_ADD : d_alu_norm);

	/* E: decode precomputed in D. */
	wire [4:0] rd      = rd_e;
	wire [2:0] f3      = f3_e;
	wire is_auipc      = is_auipc_e;
	wire is_jal        = is_jal_e;
	wire is_jalr       = is_jalr_e;
	wire is_br         = is_br_e;
	wire is_ld         = is_ld_e;
	wire is_st         = is_st_e;
	wire is_m          = is_m_e;
	wire is_mret       = is_mret_e;
	wire is_wfi        = is_wfi_e;
	wire legal         = legal_e;
	wire is_mem        = is_mem_e;
	wire has_rd        = has_rd_e;

	/* E's operands: W holds the instruction just before it (program order). */
	wire [31:0] rs1 = {
		fwd1_e3 ? w_val[31:24] : rs1_e[31:24],
		fwd1_e2 ? w_val[23:16] : rs1_e[23:16],
		fwd1_e1 ? w_val[15:8]  : rs1_e[15:8],
		fwd1_e0 ? w_val[7:0]   : rs1_e[7:0]
	};
	wire [31:0] rs2 = {
		fwd2_e3 ? w_val[31:24] : rs2_e[31:24],
		fwd2_e2 ? w_val[23:16] : rs2_e[23:16],
		fwd2_e1 ? w_val[15:8]  : rs2_e[15:8],
		fwd2_e0 ? w_val[7:0]   : rs2_e[7:0]
	};

	/* ALU: registered operands on Cycle 0 (alu_issue), alu_y_r on Cycle 1 (alu_s1_r), wb on Cycle 2 (alu_s2_r). */
	wire [31:0] alu_b = alu_b_sel_imm_e ? alu_b_imm_e : rs2;
	wire [3:0] alu_op = alu_op_e;
	wire [31:0] alu_y;
	wire alu_eq, alu_lt, alu_ltu;
	reg irq_r;
	reg alu_s1_r, alu_s2_r;
	reg [31:0] alu_a_r, alu_b_r, alu_y_r;

	atg_cpu_alu alu (
		.op(alu_op), .a(alu_a_r), .b(alu_b_r),
		.y(alu_y), .eq(alu_eq), .lt(alu_lt), .ltu(alu_ltu)
	);

	/* Parallel prefix branch comparator on resolved op1_r and op2_r (registered into jump_taken_r). */
	wire br_eq = (op1_r[31:24] == op2_r[31:24]) && (op1_r[23:16] == op2_r[23:16]) &&
		(op1_r[15:8] == op2_r[15:8]) && (op1_r[7:0] == op2_r[7:0]);
	wire br_ltu = fn_cmp_lt(op1_r, op2_r);
	wire br_lt  = fn_cmp_lt({~op1_r[31], op1_r[30:0]}, {~op2_r[31], op2_r[30:0]});
	wire [31:0] br_y = 32'd0;

	/* Fast 4-way branch condition from pre-decoded jump/branch type & polarity-adjusted br_*_p_r flags. */
	wire br_cond_pend = br_sel_j_e | (br_sel_eq_e & br_eq_p_r) |
		(br_sel_lt_e & br_lt_p_r) | (br_sel_ltu_e & br_ltu_p_r);
	wire jump_taken_pend = jump_pend_r && br_cond_pend;

	/*
	 * Fast Kogge-Stone effective address (from op1_r) and branch target.
	 */
	wire [31:0] ea_b = ea_b_e;
	wire [32:0] ea_res = fn_ks_add32(op1_r, ea_b, 1'b0);
	wire [31:0] ea_full = ea_res[31:0];
	wire ea_co = ea_res[32];
	wire [1:0] ea_p = {op1_r[1] ^ ea_b[1] ^ (op1_r[0] & ea_b[0]), op1_r[0] ^ ea_b[0]};

	wire [31:0] pc_imm_b = pc_imm_b_e;
	wire [32:0] pi_res = fn_ks_add32(pc_e, pc_imm_b, 1'b0);
	wire [31:0] pc_imm = pi_res[31:0];
	wire pc_imm_co = pi_res[32];

	wire [31:0] link = pc_e4;
	wire [1:0] pc_imm_lo = {pc_e[1] ^ pc_imm_b[1] ^ (pc_e[0] & pc_imm_b[0]), pc_e[0] ^ pc_imm_b[0]};
	wire [31:0] jalr_t = {ea_full[31:1], 1'b0};
	wire taken = is_mret | jump_taken_pend;
	wire [31:0] target = is_mret ? mepc : jump_tgt_r;
	wire [1:0] target_lo = is_mret ? mepc[1:0] : jump_tgt_r[1:0];

	/* Multiply/divide. */
	wire md_busy, md_done;
	wire [31:0] md_y;

	/* Faults of the instruction in E (memory misalignment checked on mem_prep_r from op1_r). */
	wire mis_mem = mem_prep_r && (f3[1] ? (ea_p[1] | ea_p[0]) : (f3[0] & ea_p[0]));
	wire mis_jump = taken && target_lo != 2'd0;
	wire pc_out = pc_out_e;
	wire bad = !legal || mis_mem || mis_jump || pc_out;

	/* Execution control (flattened: exec and issue signals bypass take_irq). */
	wire irq_cond = irq_r && !in_irq;
	wire take_irq = e_vld && !e_busy && !faulted && run && irq_cond;
	wire exec_nf  = e_vld && (e_busy || (run && !irq_cond));
	wire do_fault = !faulted && ((exec_nf && (!legal || mis_jump || pc_out)) || mis_mem);
	wire [3:0] do_fault_cause = pc_out ? 4'd1 : (!legal ? 4'd2 : (mis_jump ? 4'd0 : (is_ld ? 4'd4 : 4'd6)));
	/* A faulting instruction may still "complete" (it writes nothing and
	 * the core stops the next cycle), so e_done does not wait for the
	 * fault checks (timing). */
	wire issue_base_nf = e_vld && !e_busy && run && !irq_cond && !pc_out;
	wire issue_base = !faulted && issue_base_nf;
	wire md_issue_nf   = issue_base_nf && is_m && !md_s1_r;
	wire mem_issue_nf  = issue_base_nf && is_mem && !mem_prep_r && !mem_req_r && !mem_done_r;
	wire jump_issue_nf = issue_base_nf && (is_br || is_jalr || is_jal) && !jump_s1_r && !jump_pend_r;
	wire alu_issue_nf  = issue_base_nf && is_alu_e && !alu_s1_r && !alu_s2_r;
	wire md_issue   = !faulted && md_issue_nf;
	wire mem_issue  = !faulted && mem_issue_nf;
	wire jump_issue = !faulted && jump_issue_nf;
	wire alu_issue  = !faulted && alu_issue_nf;
	wire fast_done_nofault = e_vld && run && !irq_cond && ((is_wfi && irq_r) | is_fast_e);
	wire e_done_other = mem_done_r | jump_pend_r | alu_s2_r | md_done;
	wire e_done_nofault = e_done_other | fast_done_nofault;
	wire e_done = e_done_other | (!faulted && fast_done_nofault);

	wire exec_ok_nf = e_vld && !e_busy && run;
	wire exec_ok = !faulted && exec_ok_nf;
	wire take_irq_nf = exec_ok_nf && irq_cond;
	wire taken_done_nf = jump_taken_pend || (exec_ok_nf && !irq_cond && is_mret);
	wire taken_done = jump_taken_pend || (exec_ok && !irq_cond && is_mret);
	wire redirect_nf = jump_taken_pend || (exec_ok_nf && (irq_cond ? !e_busy : is_mret));
	wire redirect = jump_taken_pend || (exec_ok && (irq_cond ? !e_busy : is_mret));
	/* D moves into E when E is empty or completes (and nothing is flushed). */
	/* E takes D's instruction when it is empty or completes; a redirect
	 * only marks it invalid (the data registers do not wait for it). */
	wire e_free = !e_vld || e_done;
	wire d_adv_nf = d_vld && (!e_vld || e_done_nofault);
	wire d_adv = !faulted && d_adv_nf;
	wire i_en_nf = !d_vld || !e_vld || e_done_nofault;

	assign i_en = !faulted && i_en_nf;
	assign i_addr = {pc_f[31:2], 2'b00};
	assign stopped = faulted || (!run && !e_busy);

	/* Data port (fully registered via mem_issue so atg_cpu_win receives clean FFs). */
	assign d_valid = d_valid_r;
	assign d_write = d_write_r;
	assign d_addr  = d_addr_r;
	reg [3:0]  d_be_c;
	reg [31:0] d_wdata_c;

	always @(*) begin
		case (f3[1:0])
		2'd0: begin
			d_be_c = ea_p == 2'd0 ? 4'b0001 : ea_p == 2'd1 ? 4'b0010 :
				ea_p == 2'd2 ? 4'b0100 : 4'b1000;
			d_wdata_c = {4{op2_r[7:0]}};
		end
		2'd1: begin
			d_be_c = ea_p[1] ? 4'b1100 : 4'b0011;
			d_wdata_c = {2{op2_r[15:0]}};
		end
		default: begin
			d_be_c = 4'b1111;
			d_wdata_c = op2_r;
		end
		endcase
	end

	/* Split load alignment (mem_req_r && d_ready -> mem_rdata_r) and width/sign decode (mem_done_r -> w_val). */
	wire [31:0] ld_sh_w = ea_r == 2'd0 ? d_rdata : ea_r == 2'd1 ? {8'd0, d_rdata[31:8]} :
		ea_r == 2'd2 ? {16'd0, d_rdata[31:16]} : {24'd0, d_rdata[31:24]};
	reg [31:0] ld_val;

	always @(*) begin
		case (f3)
		3'd0: ld_val = {{24{mem_rdata_r[7]}}, mem_rdata_r[7:0]};
		3'd1: ld_val = {{16{mem_rdata_r[15]}}, mem_rdata_r[15:0]};
		3'd4: ld_val = {24'd0, mem_rdata_r[7:0]};
		3'd5: ld_val = {16'd0, mem_rdata_r[15:0]};
		default: ld_val = mem_rdata_r;
		endcase
	end

	/* Write-back value: split into fast registered ALU path and other sources */
	wire [31:0] other_wb = is_ld ? ld_val : (is_m ? md_y : link);
	wire [31:0] wb = is_alu_e ? alu_y_r : other_wb;

	/* W: the register write (repeated while W holds it: harmless). */
	always @(posedge clk) begin
		if (w_wen)
			rf[w_rd] <= w_val;
	end

	atg_cpu_muldiv muldiv (
		.clk(clk), .rst_n(rst_n), .start(md_s1_r), .op(f3), .a(op1_r), .b(op2_r),
		.busy(md_busy), .done(md_done), .y(md_y)
	);

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pc_f <= 32'd0;
			d_vld <= 1'b0;
			e_vld <= 1'b0;
			w_wen <= 1'b0;
			e_busy <= 1'b0;
			mem_prep_r <= 1'b0;
			mem_req_r <= 1'b0;
			mem_done_r <= 1'b0;
			d_valid_r <= 1'b0;
			d_write_r <= 1'b0;
			d_addr_r <= 32'd0;
			mem_rdata_r <= 32'd0;
			d_wdata <= 32'd0;
			d_be <= 4'd0;
			ea_r <= 2'd0;
			jump_s1_r <= 1'b0;
			jump_pend_r <= 1'b0;
			br_eq_p_r <= 1'b0;
			br_lt_p_r <= 1'b0;
			br_ltu_p_r <= 1'b0;
			md_s1_r <= 1'b0;
			jump_tgt_r <= 32'd0;
			op1_r <= 32'd0;
			op2_r <= 32'd0;
			irq_r <= 1'b0;
			alu_s1_r <= 1'b0;
			alu_s2_r <= 1'b0;
			alu_a_r <= 32'd0;
			alu_b_r <= 32'd0;
			alu_y_r <= 32'd0;
			faulted <= 1'b0;
			in_irq <= 1'b0;
			mepc <= 32'd0;
			mcause_q <= 4'd0;
			fault <= 1'b0;
			fault_pc <= 32'd0;
		end else begin
			fault <= 1'b0;
			if (do_fault) begin
				faulted  <= 1'b1;
				fault    <= 1'b1;
				mcause_q <= do_fault_cause;
			end
			/* Follows E until the fault, then holds (no enable on the
			 * fault checks: timing). */
			if (!faulted)
				fault_pc <= pc_e;
			if (mem_issue || jump_issue || md_issue) begin
				op1_r <= rs1;
				op2_r <= rs2;
			end
			md_s1_r <= md_issue;
			if (mem_issue)
				mem_prep_r <= 1'b1;
			else if (mem_prep_r)
				mem_prep_r <= 1'b0;
			if (mem_prep_r && !mis_mem) begin
				mem_req_r <= 1'b1;
				d_valid_r <= 1'b1;
				d_write_r <= is_st_e;
				d_addr_r  <= ea_full;
				d_be      <= d_be_c;
				d_wdata   <= d_wdata_c;
				ea_r      <= ea_p;
			end else if (mem_req_r && d_ready) begin
				mem_req_r   <= 1'b0;
				d_valid_r   <= 1'b0;
				mem_done_r  <= 1'b1;
				mem_rdata_r <= ld_sh_w;
			end else if (mem_done_r) begin
				mem_done_r <= 1'b0;
			end
			if (jump_issue)
				jump_s1_r <= 1'b1;
			if (jump_s1_r) begin
				jump_s1_r   <= 1'b0;
				jump_pend_r <= 1'b1;
			end else if (jump_pend_r) begin
				jump_pend_r <= 1'b0;
			end
			/* Latched on jump_s1_r cycle from op1_r/op2_r with f3_e[0] polarity folded in. */
			br_eq_p_r  <= br_eq ^ f3_e[0];
			br_lt_p_r  <= br_lt ^ f3_e[0];
			br_ltu_p_r <= br_ltu ^ f3_e[0];
			jump_tgt_r <= is_jalr ? jalr_t : pc_imm;
			alu_y_r <= alu_y;
			if (alu_issue) begin
				alu_s1_r <= 1'b1;
				alu_a_r  <= is_auipc ? pc_e : rs1;
				alu_b_r  <= alu_b;
			end else if (alu_s1_r) begin
				alu_s1_r <= 1'b0;
				alu_s2_r <= 1'b1;
			end else if (alu_s2_r) begin
				alu_s2_r <= 1'b0;
			end
			irq_r <= irq;
			if (faulted) begin
				if (e_done_other)
					e_busy <= 1'b0;
			end else if (e_done_nofault)
				e_busy <= 1'b0;
			else if (mem_issue_nf || md_issue_nf || jump_issue_nf || alu_issue_nf)
				e_busy <= 1'b1;
			if (faulted) begin
				if (e_done_other && is_mret)
					in_irq <= 1'b0;
			end else if (take_irq_nf) begin
				mepc <= is_wfi ? link : pc_e;
				in_irq <= 1'b1;
				mcause_q <= 4'd11;
			end else if (e_done_nofault && is_mret) begin
				in_irq <= 1'b0;
			end
			if (faulted ? e_done_other : e_done_nofault)
				w_wen <= has_rd && rd_nz_e && !bad;
			/* E. */
			if (faulted) begin
				if (e_done_other)
					e_vld <= 1'b0;
			end else if (d_adv_nf) begin
				e_vld <= !redirect_nf;
			end else if (e_done_nofault || take_irq_nf) begin
				e_vld <= 1'b0;
			end
			/* F and D: flattened 1-hot next-PC mux (cuts pc_imm -> target -> redirect -> pc_f chain). */
			if (faulted) begin
				if (jump_taken_pend)
					d_vld <= 1'b0;
			end else if (redirect_nf) begin
				d_vld <= 1'b0;
			end else if (i_en_nf) begin
				d_vld <= 1'b1;
			end

			if (faulted) begin
				if (jump_taken_pend)
					pc_f <= jump_tgt_r;
			end else if (take_irq_nf) begin
				pc_f <= IRQ_VECTOR;
			end else if (taken_done_nf) begin
				pc_f <= is_mret ? mepc : jump_tgt_r;
			end else if (i_en_nf) begin
				pc_f <= pc_f_4;
			end
		end
	end

	/* Data registers of the stages */
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pc_d            <= 32'd0;
			pc_d4           <= 32'd0;
			pc_e            <= 32'd0;
			pc_e4           <= 32'd0;
			rd_e            <= 5'd0;
			f3_e            <= 3'd0;
			rd_nz_e         <= 1'b0;
			is_auipc_e      <= 1'b0;
			is_jal_e        <= 1'b0;
			is_jalr_e       <= 1'b0;
			is_br_e         <= 1'b0;
			is_ld_e         <= 1'b0;
			is_st_e         <= 1'b0;
			is_m_e          <= 1'b0;
			is_mret_e       <= 1'b0;
			is_wfi_e        <= 1'b0;
			is_fast_e       <= 1'b0;
			br_sel_j_e      <= 1'b0;
			br_sel_eq_e     <= 1'b0;
			br_sel_lt_e     <= 1'b0;
			br_sel_ltu_e    <= 1'b0;
			legal_e         <= 1'b0;
			is_mem_e        <= 1'b0;
			has_rd_e        <= 1'b0;
			pc_out_e        <= 1'b0;
			ea_b_e          <= 32'd0;
			pc_imm_b_e      <= 32'd0;
			rs1_e           <= 32'd0;
			rs2_e           <= 32'd0;
			fwd1_e0         <= 1'b0;
			fwd1_e1         <= 1'b0;
			fwd1_e2         <= 1'b0;
			fwd1_e3         <= 1'b0;
			fwd2_e0         <= 1'b0;
			fwd2_e1         <= 1'b0;
			fwd2_e2         <= 1'b0;
			fwd2_e3         <= 1'b0;
			alu_op_e        <= A_ADD;
			alu_b_imm_e     <= 32'd0;
			alu_b_sel_imm_e <= 1'b0;
			is_alu_e        <= 1'b0;
			w_rd            <= 5'd0;
			w_val           <= 32'd0;
		end else begin
			if (i_en) begin
				pc_d  <= pc_f;
				pc_d4 <= pc_f_4;
			end
			if (d_adv) begin
				pc_e            <= pc_d;
				pc_e4           <= pc_d4;
				rd_e            <= i_rdata[11:7];
				f3_e            <= d_f3;
				rd_nz_e         <= i_rdata[11:7] != 5'd0;
				is_auipc_e      <= d_is_auipc;
				is_jal_e        <= d_is_jal;
				is_jalr_e       <= d_is_jalr;
				is_br_e         <= d_is_br;
				is_ld_e         <= d_is_ld;
				is_st_e         <= d_is_st;
				is_m_e          <= d_is_m;
				is_mret_e       <= d_is_mret;
				is_wfi_e        <= d_is_wfi;
				is_fast_e       <= d_is_fast;
				br_sel_j_e      <= d_is_jal || d_is_jalr;
				br_sel_eq_e     <= d_is_br && !d_f3[2];
				br_sel_lt_e     <= d_is_br &&  d_f3[2] && !d_f3[1];
				br_sel_ltu_e    <= d_is_br &&  d_f3[2] &&  d_f3[1];
				legal_e         <= d_legal;
				is_mem_e        <= d_is_mem;
				has_rd_e        <= d_has_rd;
				pc_out_e        <= pc_d[31:17] != 15'd0;
				ea_b_e          <= d_ea_b;
				pc_imm_b_e      <= d_pc_imm_b;
				rs1_e           <= d_rs1;
				rs2_e           <= d_rs2;
				/* What W will hold while this instruction is in E (e_vld avoids e_done on D). */
				fwd1_e0         <= d_ra != 5'd0 && (e_vld ? (has_rd && rd == d_ra) : (w_wen && w_rd == d_ra));
				fwd1_e1         <= d_ra != 5'd0 && (e_vld ? (has_rd && rd == d_ra) : (w_wen && w_rd == d_ra));
				fwd1_e2         <= d_ra != 5'd0 && (e_vld ? (has_rd && rd == d_ra) : (w_wen && w_rd == d_ra));
				fwd1_e3         <= d_ra != 5'd0 && (e_vld ? (has_rd && rd == d_ra) : (w_wen && w_rd == d_ra));
				fwd2_e0         <= d_rb != 5'd0 && (e_vld ? (has_rd && rd == d_rb) : (w_wen && w_rd == d_rb));
				fwd2_e1         <= d_rb != 5'd0 && (e_vld ? (has_rd && rd == d_rb) : (w_wen && w_rd == d_rb));
				fwd2_e2         <= d_rb != 5'd0 && (e_vld ? (has_rd && rd == d_rb) : (w_wen && w_rd == d_rb));
				fwd2_e3         <= d_rb != 5'd0 && (e_vld ? (has_rd && rd == d_rb) : (w_wen && w_rd == d_rb));
				alu_op_e        <= d_alu_op;
				alu_b_imm_e     <= d_alu_b_imm;
				alu_b_sel_imm_e <= d_alu_b_sel_imm;
				is_alu_e        <= d_is_alu;
			end
			if (alu_s2_r) begin
				w_rd  <= rd;
				w_val <= alu_y_r;
			end else if (e_done) begin
				w_rd  <= rd;
				w_val <= other_wb;
			end
		end
	end

	assign curr_pc = faulted ? fault_pc : (e_vld ? pc_e : pc_f);
	assign curr_mcause = mcause_q;
	assign curr_mepc = mepc;
	assign curr_in_irq = in_irq;
	assign curr_faulted = faulted;

	wire unused_ok = &{1'b0, md_busy, alu_eq, alu_lt, alu_ltu, br_y, pc_imm_co, ea_co,
		pc_f_co, target, pc_imm_lo, is_auipc_e};
endmodule

`default_nettype wire
