/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cpu_alu: The RV32I integer ALU and branch comparator of
 * atg_cpu_core, combinational.
 *
 * y = op(a, b): 0 ADD, 1 SUB, 2 SLL, 3 SLT, 4 SLTU, 5 XOR, 6 SRL, 7 SRA,
 * 8 OR, 9 AND, 10 PASS_B; other codes give 0.  Shifts use b[4:0].
 * eq, lt (signed) and ltu (unsigned) compare a with b whatever op is.
 * See spec/gpu/cp/atg_cpu_alu.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cpu_alu (
	input wire [3:0] op,
	input wire [31:0] a,
	input wire [31:0] b,
	output reg [31:0] y,
	output wire eq,
	output wire lt,
	output wire ltu
);
	localparam [3:0] OP_ADD = 4'd0;
	localparam [3:0] OP_SUB = 4'd1;
	localparam [3:0] OP_SLL = 4'd2;
	localparam [3:0] OP_SLT = 4'd3;
	localparam [3:0] OP_SLTU = 4'd4;
	localparam [3:0] OP_XOR = 4'd5;
	localparam [3:0] OP_SRL = 4'd6;
	localparam [3:0] OP_SRA = 4'd7;
	localparam [3:0] OP_OR = 4'd8;
	localparam [3:0] OP_AND = 4'd9;
	localparam [3:0] OP_PASS_B = 4'd10;

	wire [4:0] sh = b[4:0];

	/* Pure-Boolean 32-bit alternating AOI/OAI Kogge-Stone adder (<= 8 cmos3, zero + operators). */
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

	wire [32:0] sum_res  = fn_ks_add32(a,  b, 1'b0);
	wire [32:0] diff_res = fn_ks_add32(a, ~b, 1'b1);
	wire [31:0] sum_w    = sum_res[31:0];
	wire [31:0] diff_w   = diff_res[31:0];
	wire        sum_co   = sum_res[32];
	wire        diff_co  = diff_res[32];

	/* Direct 2-mux cascade shifter (sh[2:0] 3-stage 2:1 -> sh[4:3] 2-stage 2:1, shallowest cmos3). */
	reg [31:0] sl1, sl2, sl4, sl8, sh_left;
	reg [31:0] srl1, srl2, srl4, srl8, sh_srl;
	reg [31:0] sra1, sra2, sra4, sra8, sh_sra;
	integer si;
	always @(*) begin
		for (si = 0; si < 32; si = si + 1) begin
			sl1[si]  = sh[0] ? ((si >= 1) ? a[si-1] : 1'b0) : a[si];
			srl1[si] = sh[0] ? ((si + 1 < 32) ? a[si+1] : 1'b0) : a[si];
			sra1[si] = sh[0] ? ((si + 1 < 32) ? a[si+1] : a[31]) : a[si];
		end
		for (si = 0; si < 32; si = si + 1) begin
			sl2[si]  = sh[1] ? ((si >= 2) ? sl1[si-2] : 1'b0) : sl1[si];
			srl2[si] = sh[1] ? ((si + 2 < 32) ? srl1[si+2] : 1'b0) : srl1[si];
			sra2[si] = sh[1] ? ((si + 2 < 32) ? sra1[si+2] : a[31]) : sra1[si];
		end
		for (si = 0; si < 32; si = si + 1) begin
			sl4[si]  = sh[2] ? ((si >= 4) ? sl2[si-4] : 1'b0) : sl2[si];
			srl4[si] = sh[2] ? ((si + 4 < 32) ? srl2[si+4] : 1'b0) : srl2[si];
			sra4[si] = sh[2] ? ((si + 4 < 32) ? sra2[si+4] : a[31]) : sra2[si];
		end
		for (si = 0; si < 32; si = si + 1) begin
			sl8[si]  = sh[3] ? ((si >= 8) ? sl4[si-8] : 1'b0) : sl4[si];
			srl8[si] = sh[3] ? ((si + 8 < 32) ? srl4[si+8] : 1'b0) : srl4[si];
			sra8[si] = sh[3] ? ((si + 8 < 32) ? sra4[si+8] : a[31]) : sra4[si];
		end
		for (si = 0; si < 32; si = si + 1) begin
			sh_left[si] = sh[4] ? ((si >= 16) ? sl8[si-16] : 1'b0) : sl8[si];
			sh_srl[si]  = sh[4] ? ((si + 16 < 32) ? srl8[si+16] : 1'b0) : srl8[si];
			sh_sra[si]  = sh[4] ? ((si + 16 < 32) ? sra8[si+16] : a[31]) : sra8[si];
		end
	end

	wire [3:0] eq_b = {
		a[31:24] == b[31:24],
		a[23:16] == b[23:16],
		a[15:8]  == b[15:8],
		a[7:0]   == b[7:0]
	};
	assign eq = &eq_b;
	assign ltu = ~diff_co;
	/* Signed: different signs -> a's sign; equal signs -> the unsigned result. */
	assign lt = (a[31] != b[31]) ? a[31] : ltu;

	wire op_add   = (op == OP_ADD);
	wire op_sub   = (op == OP_SUB);
	wire op_sll   = (op == OP_SLL);
	wire op_slt   = (op == OP_SLT);
	wire op_sltu  = (op == OP_SLTU);
	wire op_xor   = (op == OP_XOR);
	wire op_srl   = (op == OP_SRL);
	wire op_sra   = (op == OP_SRA);
	wire op_or    = (op == OP_OR);
	wire op_and   = (op == OP_AND);
	wire op_passb = (op == OP_PASS_B);

	wire [31:0] y_shift = ({32{op_sll}} & sh_left) | ({32{op_srl}} & sh_srl) | ({32{op_sra}} & sh_sra);
	wire [31:0] y_logic = ({32{op_xor}} & (a ^ b)) | ({32{op_or}} & (a | b)) |
	                      ({32{op_and}} & (a & b)) | ({32{op_passb}} & b);
	wire [31:0] y_cmp   = {31'd0, (op_slt & lt) | (op_sltu & ltu)};
	wire [31:0] y_fast  = y_shift | y_logic | y_cmp;
	wire [31:0] y_arith_n = ~(({32{op_add}} & sum_w) | ({32{op_sub}} & diff_w));

	always @(*) begin
		y = ~(y_arith_n & ~y_fast);
	end

	wire unused_ok = &{1'b0, sum_co};
endmodule

`default_nettype wire
