/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tu_frnd: one channel of atg_tu_filter's conversion to fp32 (P12 →
 * P18): the integer p of the source i_sel (0 the sum A: A × 0x0101010101
 * = (A × 257) × 65537 + A × 2^32; 1 the LOD query's magnitude; 2 gather of
 * a D24 texel and 4 a D24 texel: c × (2^48 + 2^24 + 1); 3 gather of a
 * byte b: b × 0x0101010101 × 2^32; 5 RGB565's G: c × sum(2^6i, i < 13);
 * 6 its R or B: c × sum(2^5i, i < 16)), as p × 2^(n + eb − 127) rounded to
 * nearest even, n the msb position, eb 55 (the query 119, 5: 49, 6: 47);
 * 0 for p = 0.
 *
 * Pipelined, no stall: i_rr in cycle 12, i_sel in 13, the other sources
 * in 14; P13-P14 the adders, P15 the leading zeros, P16 p << lz, P17 the
 * rounding; o_rn in cycle 18 (a register).
 * See spec/gpu/tu/atg_tu_filter.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tu_frnd (
	input wire clk,
	input wire rst_n,
	input wire [39:0] i_rr,
	input wire [2:0] i_sel,
	input wire [15:0] i_fx,
	input wire [23:0] i_g24,
	input wire [7:0] i_gb,
	input wire [23:0] i_d24,
	input wire [5:0] i_c6,
	input wire [4:0] i_c5,
	output reg [31:0] o_rn
);
	wire [5:0] p_nc;

	wire [48:0] x;
	wire [64:0] y;
	wire [40:0] p_hi41;
	wire [80:0] p = {8'd0, p_hi41, y14[31:0]};
	reg [48:0] x13;
	reg [39:0] rr13, rr14;
	reg [64:0] y14;
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] sel_s0, sel_s1, sel_s2, sel_s3, sel_s4, sel_s5, sel_s6, sel_s7, sel_s8, sel_s9, sel_nc;

	am_add #(.W(49)) u_x (.a({9'd0, i_rr}), .b({1'b0, i_rr, 8'd0}), .ci(1'b0), .s(x),
		.co(p_nc[0]));
	am_add #(.W(65)) u_y (.a({16'd0, x13}), .b({x13, 16'd0}), .ci(1'b0), .s(y), .co(p_nc[1]));
	am_add #(.W(41)) u_p (.a({8'd0, y14[64:32]}), .b({1'b0, rr14}), .ci(1'b0), .s(p_hi41),
		.co(p_nc[2]));

	always @(posedge clk) begin
		x13 <= x;
		rr13 <= i_rr;
		y14 <= y;
		rr14 <= rr13;
		sel_s0 <= i_sel;
		sel_s1 <= i_sel;
		sel_s2 <= i_sel;
		sel_s3 <= i_sel;
		sel_s4 <= i_sel;
		sel_s5 <= i_sel;
		sel_s6 <= i_sel;
		sel_s7 <= i_sel;
		sel_s8 <= i_sel;
		sel_s9 <= i_sel;
		sel_nc <= i_sel;
	end

	/* P14: the integer */
	wire [80:0] np_fx  = {65'd0, i_fx};
	wire [80:0] np_g24 = {9'd0, i_g24, i_g24, i_g24};
	wire [80:0] np_gb  = {9'd0, {5{i_gb}}, 32'd0};
	wire [80:0] np_d24 = {9'd0, i_d24, i_d24, i_d24};
	wire [80:0] np_c6  = {3'd0, {13{i_c6}}};
	wire [80:0] np_c5  = {1'd0, {16{i_c5}}};

	function [7:0] mux8;
		input [2:0] s;
		input [7:0] p_s, fx_s, g24_s, gb_s, d24_s, c6_s, c5_s;
		case (s)
		3'd1: mux8 = fx_s;
		3'd2: mux8 = g24_s;
		3'd3: mux8 = gb_s;
		3'd4: mux8 = d24_s;
		3'd5: mux8 = c6_s;
		3'd6: mux8 = c5_s;
		default: mux8 = p_s;
		endcase
	endfunction

	reg [80:0] np;
	reg [1:0] ncode;
	always @(*) begin
		ncode = sel_nc == 3'd1 ? 2'd1 : sel_nc == 3'd5 ? 2'd2 : sel_nc == 3'd6 ? 2'd3 : 2'd0;
		np[7:0]   = mux8(sel_s0, p[7:0],   np_fx[7:0],   np_g24[7:0],   np_gb[7:0],   np_d24[7:0],   np_c6[7:0],   np_c5[7:0]);
		np[15:8]  = mux8(sel_s1, p[15:8],  np_fx[15:8],  np_g24[15:8],  np_gb[15:8],  np_d24[15:8],  np_c6[15:8],  np_c5[15:8]);
		np[23:16] = mux8(sel_s2, p[23:16], np_fx[23:16], np_g24[23:16], np_gb[23:16], np_d24[23:16], np_c6[23:16], np_c5[23:16]);
		np[31:24] = mux8(sel_s3, p[31:24], np_fx[31:24], np_g24[31:24], np_gb[31:24], np_d24[31:24], np_c6[31:24], np_c5[31:24]);
		np[39:32] = mux8(sel_s4, p[39:32], np_fx[39:32], np_g24[39:32], np_gb[39:32], np_d24[39:32], np_c6[39:32], np_c5[39:32]);
		np[47:40] = mux8(sel_s5, p[47:40], np_fx[47:40], np_g24[47:40], np_gb[47:40], np_d24[47:40], np_c6[47:40], np_c5[47:40]);
		np[55:48] = mux8(sel_s6, p[55:48], np_fx[55:48], np_g24[55:48], np_gb[55:48], np_d24[55:48], np_c6[55:48], np_c5[55:48]);
		np[63:56] = mux8(sel_s7, p[63:56], np_fx[63:56], np_g24[63:56], np_gb[63:56], np_d24[63:56], np_c6[63:56], np_c5[63:56]);
		np[71:64] = mux8(sel_s8, p[71:64], np_fx[71:64], np_g24[71:64], np_gb[71:64], np_d24[71:64], np_c6[71:64], np_c5[71:64]);
		case (sel_s9)
		3'd1: np[80:72] = np_fx[80:72];
		3'd2: np[80:72] = np_g24[80:72];
		3'd3: np[80:72] = np_gb[80:72];
		3'd4: np[80:72] = np_d24[80:72];
		3'd5: np[80:72] = np_c6[80:72];
		3'd6: np[80:72] = np_c5[80:72];
		default: np[80:72] = p[80:72];
		endcase
	end

	(* extract_reset = "no", shreg_extract = "no", max_fanout = 16 *) reg [80:0] np15, np16;
	(* max_fanout = 16 *) reg [1:0] nc15, nc16;
	(* keep = "true", max_fanout = 16 *) reg [6:0] lz16;
	(* keep = "true", equivalent_register_removal = "no" *) reg [6:0] lz16_ne, lz16_st_lo, lz16_st_hi;
	(* extract_reset = "no" *) reg [13:0] nib_nz16;
	(* max_fanout = 16 *) reg z16, z17;

	wire [83:0] lzv = {np15, 3'b000};
	(* keep = "true" *) wire [13:0] c_nz;
	(* keep = "true" *) wire [6:0] c_pos [0:13];
	(* keep = "true" *) wire [4:0] g_nz;
	(* keep = "true" *) wire [6:0] g_pos [0:4];
	genvar gl;
	generate
	for (gl = 0; gl < 13; gl = gl + 1) begin : g_lzc6
		wire [5:0] ch = lzv[83 - gl*6 -: 6];
		localparam [6:0] P0 = gl * 6;
		localparam [6:0] P1 = gl * 6 + 1;
		localparam [6:0] P2 = gl * 6 + 2;
		localparam [6:0] P3 = gl * 6 + 3;
		localparam [6:0] P4 = gl * 6 + 4;
		localparam [6:0] P5 = gl * 6 + 5;
		assign c_nz[gl]  = |ch;
		assign c_pos[gl] = ch[5] ? P0 : ch[4] ? P1 : ch[3] ? P2 :
		                   ch[2] ? P3 : ch[1] ? P4 : P5;
	end
	assign c_nz[13]  = |np15[2:0];
	assign c_pos[13] = np15[2] ? 7'd78 : np15[1] ? 7'd79 : np15[0] ? 7'd80 : 7'd0;
	for (gl = 0; gl < 4; gl = gl + 1) begin : g_lzc_grp
		assign g_nz[gl]  = c_nz[gl*3] || c_nz[gl*3 + 1] || c_nz[gl*3 + 2];
		assign g_pos[gl] = c_nz[gl*3] ? c_pos[gl*3] : c_nz[gl*3 + 1] ? c_pos[gl*3 + 1] : c_pos[gl*3 + 2];
	end
	assign g_nz[4]  = c_nz[12] || c_nz[13];
	assign g_pos[4] = c_nz[12] ? c_pos[12] : c_pos[13];
	endgenerate
	(* keep = "true" *) wire g_nz01 = g_nz[0] || g_nz[1];
	(* keep = "true" *) wire g_nz23 = g_nz[2] || g_nz[3];
	(* keep = "true" *) wire [6:0] lz_lo01 = g_nz[0] ? g_pos[0] : g_pos[1];
	(* keep = "true" *) wire [6:0] lz_lo23 = g_nz[2] ? g_pos[2] : g_pos[3];
	wire [6:0] lz  = g_nz01 ? lz_lo01 : g_nz23 ? lz_lo23 : g_pos[4];
	wire       z15 = !(g_nz01 || g_nz23 || g_nz[4]);

	integer gi_fr;
	always @(posedge clk) begin
		np15 <= np;
		nc15 <= ncode;
		np16 <= np15;
		nc16 <= nc15;
		lz16 <= lz;
		lz16_ne <= lz;
		lz16_st_lo <= lz;
		lz16_st_hi <= lz;
		for (gi_fr = 0; gi_fr < 14; gi_fr = gi_fr + 1)
			nib_nz16[gi_fr] <= |np15[gi_fr*4 +: 4];
		z16 <= z15;
	end

	/* eb + 80 and eb + 81 of the code */
	wire [7:0] eb80 = nc16 == 2'd1 ? 8'd199 : nc16 == 2'd2 ? 8'd129 : nc16 == 2'd3 ? 8'd127 :
		8'd135;
	wire [7:0] eb81 = nc16 == 2'd1 ? 8'd200 : nc16 == 2'd2 ? 8'd130 : nc16 == 2'd3 ? 8'd128 :
		8'd136;
	wire [80:0] ny;
	wire [7:0] ne0, ne1;

	am_shift #(.W(81), .SW(7)) u_ny (.a(np16), .amt(lz16), .left(1'b1), .arith(1'b0), .y(ny));
	am_sub #(.W(8)) u_ne0 (.a(eb80), .b({1'b0, lz16_ne}), .bi(1'b0), .d(ne0), .bo(p_nc[3]));
	am_sub #(.W(8)) u_ne1 (.a(eb81), .b({1'b0, lz16_ne}), .bi(1'b0), .d(ne1), .bo(p_nc[4]));

	(* keep = "true" *) wire [13:0] b_term, lz_eq;
	(* keep = "true" *) wire [12:0] nib_nz_term;
	generate
	for (gl = 0; gl < 14; gl = gl + 1) begin : g_nst
		localparam [3:0] GINV = 4'd13 - gl[3:0];
		wire [6:0] lz_s = (gl < 7) ? lz16_st_lo : lz16_st_hi;
		assign b_term[gl] = (lz_s[1:0] == 2'd0) ? (|np16[gl*4 +: 4]) :
		                    (lz_s[1:0] == 2'd1) ? (|np16[gl*4 +: 3]) :
		                    (lz_s[1:0] == 2'd2) ? (|np16[gl*4 +: 2]) : np16[gl*4];
		assign lz_eq[gl]  = !lz_s[6] && (lz_s[5:2] == GINV);
		if (gl < 13) begin : g_nz_t
			assign nib_nz_term[gl] = !lz_s[6] && (lz_s[5:2] < GINV) && nib_nz16[gl];
		end
	end
	endgenerate
	(* keep = "true" *) wire [4:0] b_grp;
	assign b_grp[0] = (lz_eq[0]  & b_term[0])  | (lz_eq[1]  & b_term[1])  | (lz_eq[2]  & b_term[2]);
	assign b_grp[1] = (lz_eq[3]  & b_term[3])  | (lz_eq[4]  & b_term[4])  | (lz_eq[5]  & b_term[5]);
	assign b_grp[2] = (lz_eq[6]  & b_term[6])  | (lz_eq[7]  & b_term[7])  | (lz_eq[8]  & b_term[8]);
	assign b_grp[3] = (lz_eq[9]  & b_term[9])  | (lz_eq[10] & b_term[10]) | (lz_eq[11] & b_term[11]);
	assign b_grp[4] = (lz_eq[12] & b_term[12]) | (lz_eq[13] & b_term[13]);
	(* keep = "true" *) wire [2:0] nz_grp = {|nib_nz_term[12:10], |nib_nz_term[9:5], |nib_nz_term[4:0]};
	(* keep = "true" *) wire b_any  = |b_grp;
	(* keep = "true" *) wire nz_any = |nz_grp;

	(* extract_reset = "no", max_fanout = 16 *) reg [24:0] ny17;	/* bits 80:56 */
	reg nst17;
	(* max_fanout = 16 *) reg [7:0] ne0_17, ne1_17;
	always @(posedge clk) begin
		ny17 <= ny[80:56];
		nst17 <= b_any || nz_any;
		z17 <= z16;
		ne0_17 <= ne0;
		ne1_17 <= ne1;
	end

	wire [24:0] m;

	am_add #(.W(25)) u_m (.a({1'b0, ny17[24:1]}), .b(25'd0),
		.ci(ny17[0] && (nst17 || ny17[1])), .s(m), .co(p_nc[5]));

	(* keep = "true" *) wire m24_lo = m[24];
	(* keep = "true" *) wire m24_hi = m[24];
	(* keep = "true" *) wire z17_lo = z17;
	(* keep = "true" *) wire z17_hi = z17;
	wire [31:0] rn_lo_hi = {
		1'b0,
		m24_hi ? ne1_17 : ne0_17,
		m24_hi ? m[23:17] : m[22:16],
		m24_lo ? m[16:1]  : m[15:0]
	};

	always @(posedge clk) begin
		o_rn[15:0]  <= z17_lo ? 16'd0 : rn_lo_hi[15:0];
		o_rn[31:16] <= z17_hi ? 16'd0 : rn_lo_hi[31:16];
	end

	wire unused_ok = &{1'b0, rst_n, p_nc, ny[55:0]};
endmodule

`default_nettype wire
