/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cpu_muldiv: The RV32M unit: MUL/MULH/MULHSU/MULHU on a DSP (4
 * cycles) and DIV/DIVU/REM/REMU by a 32-step restoring divider.
 *
 * start (one cycle) takes op (funct3), a and b.  busy is 1 from the
 * cycle after start until the cycle before done; done is a one-cycle
 * pulse in the cycle y is first valid (start + 4 for MUL*, start + 34
 * for DIV/REM*), and y holds until the next start.  A start while busy
 * is ignored.  Division by zero gives quotient all ones and remainder a;
 * INT_MIN / -1 gives INT_MIN and remainder 0 (RISC-V).
 * See spec/gpu/cp/atg_cpu_muldiv.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cpu_muldiv (
	input wire clk,
	input wire rst_n,
	input wire start,
	input wire [2:0] op,
	input wire [31:0] a,
	input wire [31:0] b,
	output wire busy,
	output reg done,
	output reg [31:0] y
);
	localparam [2:0] S_IDLE = 3'd0;
	localparam [2:0] S_MUL  = 3'd1;	/* 17x17 multipliers stage 1 */
	localparam [2:0] S_MUL2 = 3'd2;	/* 17x17 multipliers stage 2 */
	localparam [2:0] S_MUL3 = 3'd3;	/* 17x17 multipliers stage 3 */
	localparam [2:0] S_MSEL = 3'd4;	/* final 48-bit CSA + prefix merge & select */
	localparam [2:0] S_DIV  = 3'd5;	/* 32 steps, then the sign fix */

	reg [2:0] state;
	reg [1:0] op_r;	/* funct3[1:0]: the MUL half or REM */
	/* Multiplier: split into 17-bit signed halves {signed_ext, hi16} and {1'b0, lo16}. */
	reg signed [16:0] ma_hi, ma_lo, mb_hi, mb_lo;
	/* Divider: magnitudes, partial remainder and quotient. */
	reg [31:0] dq;	/* dividend shifting out, quotient shifting in */
	reg [31:0] dd;	/* divisor magnitude */
	reg [31:0] dd_n;	/* ~divisor magnitude for zero-INV S_DIV subtraction */
	reg [32:0] dr;	/* partial remainder */
	reg [5:0] cnt;
	reg neg_q, neg_r, dzero;
	reg [31:0] a_r;

	assign busy = state != S_IDLE;

	wire is_div = op[2];
	/* MULH and MULHSU take a as signed; MULH takes b as signed. */
	wire a_signed = op[1:0] != 2'd3;
	wire b_signed = op[1:0] == 2'd1;
	/* DIV and REM are signed (funct3 4 and 6). */
	wire d_signed = !op[0];

	/* 32-bit OR-prefix mask {0, |x[i-1:0]} via alternating NOR/NAND (<= 6 cmos3): x ^ mask = -x. */
	function [31:0] fn_neg_mask32;
		input [31:0] x;
		reg [31:0] o1_n, o2, o3_n, o4, o5_n;
		begin
			o1_n = ~(x    | {x[30:0],     1'b0});
			o2   = ~(o1_n & {o1_n[29:0],  2'b11});
			o3_n = ~(o2   | {o2[27:0],    4'h0});
			o4   = ~(o3_n & {o3_n[23:0],  8'hFF});
			o5_n = ~(o4   | {o4[15:0],   16'h0000});
			fn_neg_mask32 = {~o5_n[30:0], 1'b0};
		end
	endfunction

	/* Fast 16-bit unsigned comparator a < b (<= 6 cmos3, zero final XOR). */
	function fn_cmp_lt16;
		input [15:0] a_in;
		input [15:0] b_in;
		reg [15:0] g0_n, t0_n, g1, t1, g2_n, t2_n, g3, t3, g4_n;
		begin
			g0_n = ~(b_in & ~a_in);
			t0_n = ~(b_in | ~a_in);
			g1   = ~(g0_n & (t0_n | {g0_n[14:0], 1'b1}));
			t1   = ~(t0_n | {t0_n[14:0], 1'b1});
			g2_n = ~(g1 | (t1 & {g1[13:0], 2'b00}));
			t2_n = ~(t1 & {t1[13:0], 2'b00});
			g3   = ~(g2_n & (t2_n | {g2_n[11:0], 4'hF}));
			t3   = ~(t2_n | {t2_n[11:0], 4'hF});
			g4_n = ~(g3 | (t3 & {g3[7:0], 8'h00}));
			fn_cmp_lt16 = ~g4_n[15];
		end
	endfunction

	/* Pure-Boolean 16-bit alternating AOI/OAI Kogge-Stone adder (<= 7 cmos3, zero + operators). */
	function [16:0] fn_ks_add16;
		input [15:0] a_in;
		input [15:0] b_in;
		input ci_in;
		reg [15:0] x0, g0_n, p0_n, g1, p1, g2_n, p2_n, g3, p3, g4_n, sum_w;
		begin
			x0      = a_in ^ b_in;
			g0_n    = ~(a_in & b_in);
			p0_n    = ~(a_in | b_in);
			g0_n[0] = ~((a_in[0] & b_in[0]) | ((a_in[0] | b_in[0]) & ci_in));
			g1      = ~(g0_n & (p0_n | {g0_n[14:0], 1'b1}));
			p1      = ~(p0_n | {p0_n[14:0], 1'b1});
			g2_n    = ~(g1 | (p1 & {g1[13:0], 2'b00}));
			p2_n    = ~(p1 & {p1[13:0], 2'b00});
			g3      = ~(g2_n & (p2_n | {g2_n[11:0], 4'hF}));
			p3      = ~(p2_n | {p2_n[11:0], 4'hF});
			g4_n    = ~(g3 | (p3 & {g3[7:0], 8'h00}));
			sum_w   = x0 ^ {~g4_n[14:0], ci_in};
			fn_ks_add16 = {~g4_n[15], sum_w};
		end
	endfunction

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

`ifndef SYNTH_ASIC
	/* 48-bit Kogge-Stone adder (a + b) in ~9 cmos3 gates, zero + operators. */
	function [47:0] fn_ks_add48;
		input [47:0] a_in;
		input [47:0] b_in;
		reg [47:0] g0, p0, g1, p1, g2, p2, g3, p3, g4, p4, g5, p5, g6;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g1 = g0 | (p0 & {g0[46:0], 1'b0});
			p1 = p0 & {p0[46:0], 1'b0};
			g2 = g1 | (p1 & {g1[45:0], 2'b0});
			p2 = p1 & {p1[45:0], 2'b0};
			g3 = g2 | (p2 & {g2[43:0], 4'b0});
			p3 = p2 & {p2[43:0], 4'b0};
			g4 = g3 | (p3 & {g3[39:0], 8'b0});
			p4 = p3 & {p3[39:0], 8'b0};
			g5 = g4 | (p4 & {g4[31:0], 16'b0});
			p5 = p4 & {p4[31:0], 16'b0};
			g6 = g5 | (p5 & {g5[15:0], 32'b0});
			fn_ks_add48 = p0 ^ {g6[46:0], 1'b0};
		end
	endfunction
`endif

	wire [31:0] neg_a   = a ^ fn_neg_mask32(a);
	wire [31:0] neg_b   = b ^ fn_neg_mask32(b);
	wire [31:0] dq_res  = dq ^ ({32{neg_q}} & fn_neg_mask32(dq));
	wire [31:0] dr_res  = dr[31:0] ^ ({32{neg_r}} & fn_neg_mask32(dr[31:0]));
	wire [31:0] div_res = op_r[1] ? dr_res : dq_res;
	wire [31:0] a_abs = (d_signed && a[31]) ? neg_a : a;
	wire [31:0] b_abs = (d_signed && b[31]) ? neg_b : b;

	/* One restoring step: 2x16-bit carry-select subtraction with fast 6-gate lo_co (<= 11 cmos3). */
	wire [32:0] trial      = {dr[31:0], dq[31]};
	wire [16:0] sub_lo     = fn_ks_add16(trial[15:0],  dd_n[15:0],  1'b1);
	wire [16:0] sub_hi0    = fn_ks_add16(trial[31:16], dd_n[31:16], 1'b0);
	wire [16:0] sub_hi1    = fn_ks_add16(trial[31:16], dd_n[31:16], 1'b1);
	wire        lo_co      = !fn_cmp_lt16(trial[15:0], dd[15:0]);
	wire        fast_take0 = trial[32] | sub_hi0[16];
	wire        co32       = sub_hi0[16] | (lo_co & sub_hi1[16]);
	wire        take       = fast_take0 | (lo_co & sub_hi1[16]);
	wire        sel_t      = !take;
	wire        sel_0      = fast_take0 && !lo_co;
	wire        sel_1      = lo_co && (fast_take0 || sub_hi1[16]);
	wire [15:0] dr_lo_n    = ({16{take}} & sub_lo[15:0]) | ({16{sel_t}} & trial[15:0]);
	wire [15:0] dr_hi_n    = ({16{sel_t}} & trial[31:16]) | ({16{sel_0}} & sub_hi0[15:0]) | ({16{sel_1}} & sub_hi1[15:0]);
	wire        dr_msb_n   = take ? (trial[32] & co32) : trial[32];
	wire [32:0] dr_step    = {dr_msb_n, dr_hi_n, dr_lo_n};
	wire [5:0] cnt_n;
	wire ar_co_cnt;
	am_add #(.W(6)) cnt_add (.a(cnt), .b(6'd0), .ci(1'b1), .s(cnt_n), .co(ar_co_cnt));

`ifdef SYNTH_ASIC
	/* 33x33 signed product via 5-cycle Radix-4 Booth CSA pipeline (<= 11 cmos3/stage, 5 cycles). */
	(* keep = "true" *) reg [32:0] ma_p0, ma_p1, ma_p2, ma_p3;
	(* keep = "true" *) reg [32:0] ma_n0, ma_n1, ma_n2, ma_n3;
	(* keep = "true" *) reg [16:0] b_p1_lo, b_p1_hi, b_n1_lo, b_n1_hi;
	(* keep = "true" *) reg [16:0] b_p2_lo, b_p2_hi, b_n2_lo, b_n2_hi, b_neg_lo, b_neg_hi;
	reg [63:0] r1_0, r1_1, r1_2, r1_3, r1_4, r1_5, r1_6, r1_7;
	reg [63:0] r2_0, r2_1, r2_2, r2_3;
	reg [63:0] m_s_r, m_c_r;
	reg op_lo_r;

	wire [34:0] b_ext_in = {b_signed && b[31], b_signed && b[31], b, 1'b0};
	wire [16:0] b_p1_c, b_n1_c, b_p2_c, b_n2_c, b_neg_c;
	wire [63:0] pp_n [0:16];
	genvar gk;
	generate
		for (gk = 0; gk < 17; gk = gk + 1) begin : g_booth
			wire b0 = b_ext_in[2*gk];
			wire b1 = b_ext_in[2*gk + 1];
			wire b2 = b_ext_in[2*gk + 2];
			wire b_one_w = b1 ^ b0;
			wire b_two_w = (b2 & ~b1 & ~b0) | (~b2 & b1 & b0);
			wire b_neg_w = b2 & ~(b1 & b0);
			assign b_p1_c[gk]  = b_one_w & ~b_neg_w;
			assign b_n1_c[gk]  = b_one_w &  b_neg_w;
			assign b_p2_c[gk]  = b_two_w & ~b_neg_w;
			assign b_n2_c[gk]  = b_two_w &  b_neg_w;
			assign b_neg_c[gk] = b_neg_w;

			wire [32:0] ma_ps = (gk < 4) ? ma_p0 : (gk < 8) ? ma_p1 : (gk < 12) ? ma_p2 : ma_p3;
			wire [32:0] ma_ns = (gk < 4) ? ma_n0 : (gk < 8) ? ma_n1 : (gk < 12) ? ma_n2 : ma_n3;
			wire [34:0] ma_1p = {{2{ma_ps[32]}}, ma_ps};
			wire [34:0] ma_1n = {{2{ma_ns[32]}}, ma_ns};
			wire [34:0] ma_2p = {ma_ps[32], ma_ps, 1'b0};
			wire [34:0] ma_2n = {ma_ns[32], ma_ns, 1'b1};
			wire [16:0] p1_lo = (gk < 8) ? b_p1_lo : b_p1_hi;
			wire [16:0] n1_lo = (gk < 8) ? b_n1_lo : b_n1_hi;
			wire [16:0] p2_lo = (gk < 8) ? b_p2_lo : b_p2_hi;
			wire [16:0] n2_lo = (gk < 8) ? b_n2_lo : b_n2_hi;
			wire [16:0] p1_hi = (gk < 8) ? b_p1_hi : b_p1_lo;
			wire [16:0] n1_hi = (gk < 8) ? b_n1_hi : b_n1_lo;
			wire [16:0] p2_hi = (gk < 8) ? b_p2_hi : b_p2_lo;
			wire [16:0] n2_hi = (gk < 8) ? b_n2_hi : b_n2_lo;
			/* Active-low Booth mag_n in 3 NAND/NOR levels (eliminates final INV per PP bit). */
			wire [16:0] t0_lo = ~({17{p1_lo[gk]}} & ma_1p[16:0]);
			wire [16:0] t1_lo = ~({17{n1_lo[gk]}} & ma_1n[16:0]);
			wire [16:0] t2_lo = ~({17{p2_lo[gk]}} & ma_2p[16:0]);
			wire [16:0] t3_lo = ~({17{n2_lo[gk]}} & ma_2n[16:0]);
			wire [16:0] mag_lo_n = ~(~(t0_lo & t1_lo) | ~(t2_lo & t3_lo));

			wire [17:0] t0_hi = ~({18{p1_hi[gk]}} & ma_1p[34:17]);
			wire [17:0] t1_hi = ~({18{n1_hi[gk]}} & ma_1n[34:17]);
			wire [17:0] t2_hi = ~({18{p2_hi[gk]}} & ma_2p[34:17]);
			wire [17:0] t3_hi = ~({18{n2_hi[gk]}} & ma_2n[34:17]);
			wire [17:0] mag_hi_n = ~(~(t0_hi & t1_hi) | ~(t2_hi & t3_hi));
			wire [34:0] mag_n = {mag_hi_n, mag_lo_n};
			wire prev_neg_n = (gk > 0) ? ~((gk - 1 < 8) ? b_neg_lo[gk-1] : b_neg_hi[gk-1]) : 1'b1;

			if (gk == 0) begin : g_p0
				assign pp_n[0] = {27'h7FF_FFFF, ~mag_n[34], mag_n[34], mag_n[34], mag_n[33:0]};
			end else if (gk == 1) begin : g_p1
				assign pp_n[1] = {26'h3FF_FFFF, 1'b0, ~mag_n[34], mag_n[33:0], 1'b1, prev_neg_n};
			end else if (gk < 15) begin : g_pmid
				localparam integer SH = 2 * gk;
				localparam integer TOP = 64 - 36 - SH;
				if (TOP > 0) begin : g_top
					assign pp_n[gk] = {{TOP{1'b1}}, 1'b0, ~mag_n[34], mag_n[33:0], 1'b1, prev_neg_n, {(SH-2){1'b1}}};
				end else begin : g_notop
					assign pp_n[gk] = {1'b0, ~mag_n[34], mag_n[33:0], 1'b1, prev_neg_n, {(SH-2){1'b1}}};
				end
			end else if (gk == 15) begin : g_p15
				assign pp_n[15] = {mag_n[33:0], 1'b1, prev_neg_n, 28'hFFF_FFFF};
			end else begin : g_p16
				assign pp_n[16] = {mag_n[31:0], 1'b1, prev_neg_n, 30'h3FFF_FFFF};
			end
		end
	endgenerate

	/* S_MUL -> S_MUL2: 17 inverted rows -> 12 -> 8 inverted rows (sum = -8 - P mod 2^64). */
	wire [63:0] w1a_s0 = pp_n[0]  ^ pp_n[1]  ^ pp_n[2];
	wire [63:0] w1a_c0 = {(pp_n[0][62:0]  & pp_n[1][62:0])  | (pp_n[0][62:0]  & pp_n[2][62:0])  | (pp_n[1][62:0]  & pp_n[2][62:0]),  1'b1};
	wire [63:0] w1a_s1 = pp_n[3]  ^ pp_n[4]  ^ pp_n[5];
	wire [63:0] w1a_c1 = {(pp_n[3][62:0]  & pp_n[4][62:0])  | (pp_n[3][62:0]  & pp_n[5][62:0])  | (pp_n[4][62:0]  & pp_n[5][62:0]),  1'b1};
	wire [63:0] w1a_s2 = pp_n[6]  ^ pp_n[7]  ^ pp_n[8];
	wire [63:0] w1a_c2 = {(pp_n[6][62:0]  & pp_n[7][62:0])  | (pp_n[6][62:0]  & pp_n[8][62:0])  | (pp_n[7][62:0]  & pp_n[8][62:0]),  1'b1};
	wire [63:0] w1a_s3 = pp_n[9]  ^ pp_n[10] ^ pp_n[11];
	wire [63:0] w1a_c3 = {(pp_n[9][62:0]  & pp_n[10][62:0]) | (pp_n[9][62:0]  & pp_n[11][62:0]) | (pp_n[10][62:0] & pp_n[11][62:0]), 1'b1};
	wire [63:0] w1a_s4 = pp_n[12] ^ pp_n[13] ^ pp_n[14];
	wire [63:0] w1a_c4 = {(pp_n[12][62:0] & pp_n[13][62:0]) | (pp_n[12][62:0] & pp_n[14][62:0]) | (pp_n[13][62:0] & pp_n[14][62:0]), 1'b1};

	wire [63:0] r1_n0 = w1a_s0 ^ w1a_c0 ^ w1a_c1;
	wire [63:0] r1_n1 = {(w1a_s0[62:0] & w1a_c0[62:0]) | (w1a_s0[62:0] & w1a_c1[62:0]) | (w1a_c0[62:0] & w1a_c1[62:0]), 1'b1};
	wire [63:0] r1_n2 = w1a_s1 ^ w1a_c2 ^ pp_n[15];
	wire [63:0] r1_n3 = {(w1a_s1[62:0] & w1a_c2[62:0]) | (w1a_s1[62:0] & pp_n[15][62:0]) | (w1a_c2[62:0] & pp_n[15][62:0]), 1'b1};
	wire [63:0] r1_n4 = w1a_s2 ^ w1a_c3 ^ pp_n[16];
	wire [63:0] r1_n5 = {(w1a_s2[62:0] & w1a_c3[62:0]) | (w1a_s2[62:0] & pp_n[16][62:0]) | (w1a_c3[62:0] & pp_n[16][62:0]), 1'b1};
	wire [63:0] r1_n6 = w1a_s3 ^ w1a_s4 ^ w1a_c4;
	wire [63:0] r1_n7 = {(w1a_s3[62:0] & w1a_s4[62:0]) | (w1a_s3[62:0] & w1a_c4[62:0]) | (w1a_s4[62:0] & w1a_c4[62:0]), 1'b1};

	/* S_MUL2 -> S_MUL3: invert the 8 rows once (~r1_0..7 sums to +P mod 2^64), then 8 -> 6 -> 4 rows. */
	wire [63:0] rt_0 = ~r1_0, rt_1 = ~r1_1, rt_2 = ~r1_2, rt_3 = ~r1_3;
	wire [63:0] rt_4 = ~r1_4, rt_5 = ~r1_5, rt_6 = ~r1_6, rt_7 = ~r1_7;
	wire [63:0] w2a_s0 = rt_0 ^ rt_1 ^ rt_2;
	wire [63:0] w2a_c0 = {(rt_0[62:0] & rt_1[62:0]) | (rt_0[62:0] & rt_2[62:0]) | (rt_1[62:0] & rt_2[62:0]), 1'b0};
	wire [63:0] w2a_s1 = rt_3 ^ rt_4 ^ rt_5;
	wire [63:0] w2a_c1 = {(rt_3[62:0] & rt_4[62:0]) | (rt_3[62:0] & rt_5[62:0]) | (rt_4[62:0] & rt_5[62:0]), 1'b0};
	wire [63:0] r2_n0  = w2a_s0 ^ w2a_c0 ^ w2a_s1;
	wire [63:0] r2_n1  = {(w2a_s0[62:0] & w2a_c0[62:0]) | (w2a_s0[62:0] & w2a_s1[62:0]) | (w2a_c0[62:0] & w2a_s1[62:0]), 1'b0};
	wire [63:0] r2_n2  = w2a_c1 ^ rt_6 ^ rt_7;
	wire [63:0] r2_n3  = {(w2a_c1[62:0] & rt_6[62:0])   | (w2a_c1[62:0] & rt_7[62:0])   | (rt_6[62:0]   & rt_7[62:0]),   1'b0};

	/* S_MUL3 -> S_MSEL: 4 rows -> 3 -> 2 rows (2 CSA layers). */
	wire [63:0] w3a_s = r2_0 ^ r2_1 ^ r2_2;
	wire [63:0] w3a_c = {(r2_0[62:0] & r2_1[62:0]) | (r2_0[62:0] & r2_2[62:0]) | (r2_1[62:0] & r2_2[62:0]), 1'b0};
	wire [63:0] ms_n  = w3a_s ^ w3a_c ^ r2_3;
	wire [63:0] mc_n  = {(w3a_s[62:0] & w3a_c[62:0]) | (w3a_s[62:0] & r2_3[62:0]) | (w3a_c[62:0] & r2_3[62:0]), 1'b0};

	/* S_MSEL: 2x32-bit Kogge-Stone carry-select across 64 bits (<= 11 cmos3). */
	wire [32:0] m_lo_w  = fn_ks_add32(m_s_r[31:0],  m_c_r[31:0],  1'b0);
	wire [32:0] m_hi0_w = fn_ks_add32(m_s_r[63:32], m_c_r[63:32], 1'b0);
	wire [32:0] m_hi1_w = fn_ks_add32(m_s_r[63:32], m_c_r[63:32], 1'b1);
	wire [31:0] m_hi_w  = m_lo_w[32] ? m_hi1_w[31:0] : m_hi0_w[31:0];
`else
	wire mul_en = (state == S_MUL || state == S_MUL2 || state == S_MUL3);
	/* 33x33 signed product via four 17x17 signed LAT=3 multipliers (<=9 cmos3 inside, 5 cycles total). */
	wire [33:0] p00, p01, p10, p11;
	am_mul #(.AW(17), .BW(17), .SIGNED(1), .LAT(3)) u_m00 (.clk(clk), .en(mul_en), .a(ma_lo), .b(mb_lo), .p(p00));
	am_mul #(.AW(17), .BW(17), .SIGNED(1), .LAT(3)) u_m01 (.clk(clk), .en(mul_en), .a(ma_lo), .b(mb_hi), .p(p01));
	am_mul #(.AW(17), .BW(17), .SIGNED(1), .LAT(3)) u_m10 (.clk(clk), .en(mul_en), .a(ma_hi), .b(mb_lo), .p(p10));
	am_mul #(.AW(17), .BW(17), .SIGNED(1), .LAT(3)) u_m11 (.clk(clk), .en(mul_en), .a(ma_hi), .b(mb_hi), .p(p11));

	/* S_MSEL: 3:2 CSA + 48-bit Kogge-Stone add on [63:16] directly from p00..p11. */
	wire [47:0] ms_a = {p11[31:0], p00[31:16]};
	wire [47:0] ms_b = {{14{p01[33]}}, p01};
	wire [47:0] ms_c = {{14{p10[33]}}, p10};
	wire [47:0] ms_s = ms_a ^ ms_b ^ ms_c;
	wire [47:0] ms_k = (ms_a & ms_b) | (ms_a & ms_c) | (ms_b & ms_c);
	wire [47:0] p_hi48 = fn_ks_add48(ms_s, {ms_k[46:0], 1'b0});
	wire [63:0] p_prod = {p_hi48, p00[15:0]};
`endif

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_IDLE;
			done  <= 1'b0;
			y     <= 32'd0;
			op_r  <= 2'd0;
			cnt   <= 6'd0;
`ifdef SYNTH_ASIC
			ma_p0   <= 33'd0; ma_p1 <= 33'd0; ma_p2 <= 33'd0; ma_p3 <= 33'd0;
			ma_n0   <= 33'd0; ma_n1 <= 33'd0; ma_n2 <= 33'd0; ma_n3 <= 33'd0;
			b_p1_lo <= 17'd0; b_p1_hi <= 17'd0;
			b_n1_lo <= 17'd0; b_n1_hi <= 17'd0;
			b_p2_lo <= 17'd0; b_p2_hi <= 17'd0;
			b_n2_lo <= 17'd0; b_n2_hi <= 17'd0;
			b_neg_lo <= 17'd0; b_neg_hi <= 17'd0;
			r1_0 <= 64'd0; r1_1 <= 64'd0; r1_2 <= 64'd0; r1_3 <= 64'd0;
			r1_4 <= 64'd0; r1_5 <= 64'd0; r1_6 <= 64'd0; r1_7 <= 64'd0;
			r2_0 <= 64'd0; r2_1 <= 64'd0; r2_2 <= 64'd0; r2_3 <= 64'd0;
			m_s_r <= 64'd0;
			m_c_r <= 64'd0;
			op_lo_r <= 1'b0;
`else
			ma_hi <= 17'sd0;
			ma_lo <= 17'sd0;
			mb_hi <= 17'sd0;
			mb_lo <= 17'sd0;
`endif
			dq    <= 32'd0;
			dd    <= 32'd0;
			dd_n  <= 32'hFFFF_FFFF;
			dr    <= 33'd0;
			neg_q <= 1'b0;
			neg_r <= 1'b0;
			dzero <= 1'b0;
			a_r   <= 32'd0;
		end else begin
			done <= 1'b0;
`ifdef SYNTH_ASIC
			r1_0  <= r1_n0; r1_1  <= r1_n1; r1_2  <= r1_n2; r1_3  <= r1_n3;
			r1_4  <= r1_n4; r1_5  <= r1_n5; r1_6  <= r1_n6; r1_7  <= r1_n7;
			r2_0  <= r2_n0; r2_1  <= r2_n1; r2_2  <= r2_n2; r2_3  <= r2_n3;
			m_s_r <= ms_n;
			m_c_r <= mc_n;
`endif
			case (state)
			S_IDLE: begin
				/* Pre-latch abs operands every S_IDLE cycle to bypass md_start on dq/dd. */
				dq <= a_abs;
				dd <= b_abs;
				dd_n <= ~b_abs;
				if (start) begin
					op_r <= op[1:0];
					a_r <= a;
					if (is_div) begin
						dr <= 33'd0;
						cnt <= 6'd0;
						neg_q <= d_signed && (a[31] != b[31]);
						neg_r <= d_signed && a[31];
						dzero <= (b == 32'd0);
						state <= S_DIV;
					end else begin
`ifdef SYNTH_ASIC
						ma_p0    <= {a_signed && a[31], a};
						ma_p1    <= {a_signed && a[31], a};
						ma_p2    <= {a_signed && a[31], a};
						ma_p3    <= {a_signed && a[31], a};
						ma_n0    <= ~{a_signed && a[31], a};
						ma_n1    <= ~{a_signed && a[31], a};
						ma_n2    <= ~{a_signed && a[31], a};
						ma_n3    <= ~{a_signed && a[31], a};
						b_p1_lo  <= b_p1_c;
						b_p1_hi  <= b_p1_c;
						b_n1_lo  <= b_n1_c;
						b_n1_hi  <= b_n1_c;
						b_p2_lo  <= b_p2_c;
						b_p2_hi  <= b_p2_c;
						b_n2_lo  <= b_n2_c;
						b_n2_hi  <= b_n2_c;
						b_neg_lo <= b_neg_c;
						b_neg_hi <= b_neg_c;
						op_lo_r  <= (op[1:0] == 2'd0);
`else
						ma_hi <= {a_signed && a[31], a[31:16]};
						ma_lo <= {1'b0, a[15:0]};
						mb_hi <= {b_signed && b[31], b[31:16]};
						mb_lo <= {1'b0, b[15:0]};
`endif
						state <= S_MUL;
					end
				end
			end
			S_MUL:
				state <= S_MUL2;
			S_MUL2:
				state <= S_MUL3;
			S_MUL3:
				state <= S_MSEL;
			S_MSEL: begin
`ifdef SYNTH_ASIC
				y <= op_lo_r ? m_lo_w[31:0] : m_hi_w;
`else
				y <= (op_r == 2'd0) ? p_prod[31:0] : p_prod[63:32];
`endif
				done <= 1'b1;
				state <= S_IDLE;
			end
			S_DIV: begin
				dr <= dr_step;
				dq <= {dq[30:0], take};
				if (!cnt[5]) begin
					cnt <= cnt_n;
				end else begin
					/* Sign fix; op_r[1] = 1 is REM/REMU. */
					if (dzero)
						y <= op_r[1] ? a_r : 32'hFFFF_FFFF;
					else
						y <= div_res;
					done <= 1'b1;
					state <= S_IDLE;
				end
			end
				default: begin
					done <= 1'b0;
					state <= S_IDLE;
				end
			endcase
		end
	end

`ifdef SYNTH_ASIC
	wire unused_ok = &{1'b0, dr[32], ar_co_cnt, m_hi0_w[32], m_hi1_w[32], b_neg_hi[16], dd[31:16], sub_lo[16]};
`else
	wire unused_ok = &{1'b0, dr[32], ar_co_cnt, p00[33:32], p11[33:32], ms_k[47], dd[31:16], sub_lo[16]};
`endif
endmodule

`default_nettype wire
