/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_avg: the integer averages of the blitter's RESOLVE (docs/ghci.md
 * sections 8.2 and 8.5), combinational.  The input holds up to four
 * pixels of four samples each, sample s of pixel p at sample slot 4 p + s,
 * one slot per pixel (ps = 2^psl bytes: slot k at bits 8 ps k); the output
 * holds the pixels' averages, pixel p at bits 8 ps p (as many pixels as
 * fit 128 bits; the rest 0):
 *
 *   fmt 0 UNORM8           every byte an unsigned normalized channel
 *                          (pixels of 1, 2 or 4 bytes)
 *   fmt 1 UNORM565         the fields [15:11], [10:5], [4:0]
 *   fmt 6 UNORM2_10_10_10  the fields [31:30], [29:20], [19:10], [9:0]
 *   fmt 7 UNORM1555        the fields [15], [14:10], [9:5], [4:0]
 *   fmt 2 SAMPLE0 (and 3-5, which atg_blt_favg averages)  sample 0
 *
 * A field's average is the sum of the four samples' codes divided by 4,
 * rounded to the nearest code, ties to even.  See spec/gpu/blt/atg_blt_avg.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_blt_avg_s1_r (
	input wire [2:0] fmt0,
	input wire [2:0] fmt1,
	input wire [2:0] fmt2,
	input wire [2:0] fmt3,
	input wire [2:0] psl0,
	input wire [2:0] psl1,
	input wire [2:0] psl2,
	input wire [2:0] psl3,
	input wire [2:0] psl_s0,
	input wire [511:0] in_s,
	output wire [191:0] csa_s_out,
	output wire [191:0] csa_c_out,
	output wire [127:0] s0_only
);
	function [127:0] sample0_fast;
		input [2:0] pl;
		input [511:0] x;
		reg [127:0] o;
		integer b;
		begin
			o = 128'd0;
			case (pl)
			3'd0: o[31:0] = {x[103:96], x[71:64], x[39:32], x[7:0]};
			3'd1: o[63:0] = {x[207:192], x[143:128], x[79:64], x[15:0]};
			3'd2: o = {x[415:384], x[287:256], x[159:128], x[31:0]};
			3'd3: o = {x[319:256], x[63:0]};
			3'd4: o = x[127:0];
			default: begin
				for (b = 0; b < 16; b = b + 1)
					o[8*b +: 8] = x[8*b +: 8];
			end
			endcase
			sample0_fast = o;
		end
	endfunction

	genvar j, sm;
	generate
	for (j = 0; j < 16; j = j + 1) begin : g_avg
		localparam integer PP = j / 4;
		localparam integer CH = j % 4;
		wire [2:0] p_fmt_l = (PP == 0) ? fmt0 : (PP == 1) ? fmt1 : (PP == 2) ? fmt2 : fmt3;
		wire [2:0] p_psl_l = (PP == 0) ? psl0 : (PP == 1) ? psl1 : (PP == 2) ? psl2 : psl3;
		wire [9:0] v [0:3];

		for (sm = 0; sm < 4; sm = sm + 1) begin : g_s
			localparam integer S_SLOT = 4 * PP + sm;
			wire [7:0] in_b = in_s[8 * S_SLOT +: 8];
			wire [15:0] in_h = in_s[16 * S_SLOT +: 16];
			wire [31:0] in_w = in_s[32 * S_SLOT +: 32];
			reg [9:0] v_val;

			always @(*) begin
				case (p_fmt_l)
				3'd0: begin
					case (p_psl_l)
					3'd0: v_val = (CH == 0) ? {2'd0, in_b} : 10'd0;
					3'd1: v_val = (CH == 0) ? {2'd0, in_h[7:0]} : (CH == 1) ? {2'd0, in_h[15:8]} : 10'd0;
					3'd2: v_val = (CH == 0) ? {2'd0, in_w[7:0]} : (CH == 1) ? {2'd0, in_w[15:8]} : (CH == 2) ? {2'd0, in_w[23:16]} : {2'd0, in_w[31:24]};
					default: v_val = 10'd0;
					endcase
				end
				3'd1: begin
					case (CH)
					0: v_val = {5'd0, in_h[4:0]};
					1: v_val = {4'd0, in_h[10:5]};
					2: v_val = {5'd0, in_h[15:11]};
					default: v_val = 10'd0;
					endcase
				end
				3'd6: begin
					case (CH)
					0: v_val = in_w[9:0];
					1: v_val = in_w[19:10];
					2: v_val = in_w[29:20];
					default: v_val = {8'd0, in_w[31:30]};
					endcase
				end
				3'd7: begin
					case (CH)
					0: v_val = {5'd0, in_h[4:0]};
					1: v_val = {5'd0, in_h[9:5]};
					2: v_val = {5'd0, in_h[14:10]};
					default: v_val = {9'd0, in_h[15]};
					endcase
				end
				default: v_val = 10'd0;
				endcase
			end
			assign v[sm] = v_val;
		end

		wire [11:0] w0 = {2'd0, v[0]};
		wire [11:0] w1 = {2'd0, v[1]};
		wire [11:0] w2 = {2'd0, v[2]};
		wire [11:0] w3 = {2'd0, v[3]};
		wire [11:0] csa1_s = w0 ^ w1 ^ w2;
		wire [11:0] csa1_c = {((w0[10:0] & w1[10:0]) | (w1[10:0] & w2[10:0]) | (w0[10:0] & w2[10:0])), 1'b0};
		wire [11:0] csa2_s = csa1_s ^ csa1_c ^ w3;
		wire [11:0] csa2_c = {((csa1_s[10:0] & csa1_c[10:0]) | (csa1_c[10:0] & w3[10:0]) | (csa1_s[10:0] & w3[10:0])), 1'b0};
		assign csa_s_out[j*12 +: 12] = csa2_s;
		assign csa_c_out[j*12 +: 12] = csa2_c;
	end
	endgenerate

	assign s0_only = sample0_fast(psl_s0, in_s);
endmodule

module atg_blt_avg_s1 (
	input wire [2:0] fmt,
	input wire [2:0] psl,
	input wire [511:0] in_s,
	output wire [191:0] csa_s_out,
	output wire [191:0] csa_c_out,
	output wire [127:0] s0_only
);
	atg_blt_avg_s1_r u_s1_r (
		.fmt0(fmt), .fmt1(fmt), .fmt2(fmt), .fmt3(fmt),
		.psl0(psl), .psl1(psl), .psl2(psl), .psl3(psl), .psl_s0(psl),
		.in_s(in_s),
		.csa_s_out(csa_s_out),
		.csa_c_out(csa_c_out),
		.s0_only(s0_only)
	);
endmodule

module atg_blt_avg_s2a (
	input wire [191:0] csa_s_in,
	input wire [191:0] csa_c_in,
	output wire [63:0] sum_lo_out,
	output wire [15:0] c4_s_out,
	output wire [127:0] hi0_out,
	output wire [127:0] hi1_out
);
	genvar j;
	generate
	for (j = 0; j < 16; j = j + 1) begin : g_avga
		wire [11:0] csa2_s = csa_s_in[j*12 +: 12];
		wire [11:0] csa2_c = csa_c_in[j*12 +: 12];
		wire [1:0] hi_co_unused;
		am_add #(.W(4)) a_slo (.a(csa2_s[3:0]),  .b(csa2_c[3:0]),  .ci(1'b0), .s(sum_lo_out[j*4 +: 4]), .co(c4_s_out[j]));
		am_add #(.W(8)) a_hi0 (.a(csa2_s[11:4]), .b(csa2_c[11:4]), .ci(1'b0), .s(hi0_out[j*8 +: 8]),    .co(hi_co_unused[0]));
		am_add #(.W(8)) a_hi1 (.a(csa2_s[11:4]), .b(csa2_c[11:4]), .ci(1'b1), .s(hi1_out[j*8 +: 8]),    .co(hi_co_unused[1]));
		wire unused_ok = &{1'b0, hi_co_unused};
	end
	endgenerate
endmodule

module atg_blt_avg_s2b (
	input wire [2:0] fmt,
	input wire [2:0] psl,
	input wire [63:0] sum_lo_in,
	input wire [15:0] c4_s_in,
	input wire [127:0] hi0_in,
	input wire [127:0] hi1_in,
	output wire [127:0] avg_only
);
	wire [159:0] q;

	genvar j;
	generate
	for (j = 0; j < 16; j = j + 1) begin : g_avgb
		wire [3:0] sum_lo = sum_lo_in[j*4 +: 4];
		wire c4_s = c4_s_in[j];
		wire [7:0] hi0 = hi0_in[j*8 +: 8];
		wire [7:0] hi1 = hi1_in[j*8 +: 8];
		wire       slo_c1_dummy;
		wire [2:0] p4_32;
		am_add #(.W(3)) a_p4_add (.a({1'b0, sum_lo[3:2]}), .b(3'd1), .ci(1'b0), .s(p4_32), .co(slo_c1_dummy));
		wire c4_p4_exact = c4_s | p4_32[2];
`ifdef MUT_RSV_RNE
		wire round_up = sum_lo[1];
`else
		wire round_up = sum_lo[1] && (sum_lo[0] || sum_lo[2]);
`endif
		wire [1:0] q_lo = round_up ? p4_32[1:0] : sum_lo[3:2];
		wire       c4_sel = round_up ? c4_p4_exact : c4_s;
		assign q[10 * j +: 10] = {c4_sel ? hi1 : hi0, q_lo};

		wire unused_ok = &{1'b0, slo_c1_dummy};
	end
	endgenerate

	wire [9:0] q0_0 = q[10 * 0 +: 10];
	wire [9:0] q0_1 = q[10 * 1 +: 10];
	wire [9:0] q0_2 = q[10 * 2 +: 10];
	wire [9:0] q0_3 = q[10 * 3 +: 10];
	wire [9:0] q1_0 = q[10 * 4 +: 10];
	wire [9:0] q1_1 = q[10 * 5 +: 10];
	wire [9:0] q1_2 = q[10 * 6 +: 10];
	wire [9:0] q1_3 = q[10 * 7 +: 10];
	wire [9:0] q2_0 = q[10 * 8 +: 10];
	wire [9:0] q2_1 = q[10 * 9 +: 10];
	wire [9:0] q2_2 = q[10 * 10 +: 10];
	wire [9:0] q2_3 = q[10 * 11 +: 10];
	wire [9:0] q3_0 = q[10 * 12 +: 10];
	wire [9:0] q3_1 = q[10 * 13 +: 10];
	wire [9:0] q3_2 = q[10 * 14 +: 10];
	wire [9:0] q3_3 = q[10 * 15 +: 10];

	wire [127:0] f6_out = {
		q3_3[1:0], q3_2, q3_1, q3_0,
		q2_3[1:0], q2_2, q2_1, q2_0,
		q1_3[1:0], q1_2, q1_1, q1_0,
		q0_3[1:0], q0_2, q0_1, q0_0
	};
	wire [127:0] f1_out = {
		64'd0,
		q3_2[4:0], q3_1[5:0], q3_0[4:0],
		q2_2[4:0], q2_1[5:0], q2_0[4:0],
		q1_2[4:0], q1_1[5:0], q1_0[4:0],
		q0_2[4:0], q0_1[5:0], q0_0[4:0]
	};
	wire [127:0] f7_out = {
		64'd0,
		q3_3[0], q3_2[4:0], q3_1[4:0], q3_0[4:0],
		q2_3[0], q2_2[4:0], q2_1[4:0], q2_0[4:0],
		q1_3[0], q1_2[4:0], q1_1[4:0], q1_0[4:0],
		q0_3[0], q0_2[4:0], q0_1[4:0], q0_0[4:0]
	};
	wire [127:0] f0_p0 = {96'd0, q3_0[7:0], q2_0[7:0], q1_0[7:0], q0_0[7:0]};
	wire [127:0] f0_p1 = {64'd0, q3_1[7:0], q3_0[7:0], q2_1[7:0], q2_0[7:0], q1_1[7:0], q1_0[7:0], q0_1[7:0], q0_0[7:0]};
	wire [127:0] f0_p2 = {
		q3_3[7:0], q3_2[7:0], q3_1[7:0], q3_0[7:0],
		q2_3[7:0], q2_2[7:0], q2_1[7:0], q2_0[7:0],
		q1_3[7:0], q1_2[7:0], q1_1[7:0], q1_0[7:0],
		q0_3[7:0], q0_2[7:0], q0_1[7:0], q0_0[7:0]
	};

	reg [127:0] avg_core;
	always @(*) begin
		case ({fmt, psl[1:0]})
		5'b000_00: avg_core = f0_p0;
		5'b000_01: avg_core = f0_p1;
		5'b000_10,
		5'b000_11: avg_core = f0_p2;
		5'b001_00, 5'b001_01, 5'b001_10, 5'b001_11: avg_core = f1_out;
		5'b110_00, 5'b110_01, 5'b110_10, 5'b110_11: avg_core = f6_out;
		5'b111_00, 5'b111_01, 5'b111_10, 5'b111_11: avg_core = f7_out;
		default: avg_core = 128'd0;
		endcase
	end
	wire unused_psl2 = psl[2];
	assign avg_only = avg_core;
endmodule

module atg_blt_avg_s2 (
	input wire [2:0] fmt,
	input wire [2:0] psl,
	input wire [191:0] csa_s_in,
	input wire [191:0] csa_c_in,
	output wire [127:0] avg_only
);
	wire [63:0] sum_lo_w;
	wire [15:0] c4_s_w;
	wire [127:0] hi0_w, hi1_w;
	atg_blt_avg_s2a u_s2a (
		.csa_s_in(csa_s_in), .csa_c_in(csa_c_in),
		.sum_lo_out(sum_lo_w), .c4_s_out(c4_s_w), .hi0_out(hi0_w), .hi1_out(hi1_w)
	);
	atg_blt_avg_s2b u_s2b (
		.fmt(fmt), .psl(psl),
		.sum_lo_in(sum_lo_w), .c4_s_in(c4_s_w), .hi0_in(hi0_w), .hi1_in(hi1_w),
		.avg_only(avg_only)
	);
endmodule

module atg_blt_avg (
	input wire [2:0] fmt,
	input wire [2:0] psl,	/* log2 of the pixel's bytes */
	input wire [511:0] in_s,
	output reg [127:0] out_p,
	output wire [127:0] avg_only,
	output wire [127:0] s0_only
);
	wire [191:0] csa_s_w, csa_c_w;
	wire [127:0] avg_w, s0_w;

	atg_blt_avg_s1 u_s1 (
		.fmt(fmt),
		.psl(psl),
		.in_s(in_s),
		.csa_s_out(csa_s_w),
		.csa_c_out(csa_c_w),
		.s0_only(s0_w)
	);

	atg_blt_avg_s2 u_s2 (
		.fmt(fmt),
		.psl(psl),
		.csa_s_in(csa_s_w),
		.csa_c_in(csa_c_w),
		.avg_only(avg_w)
	);

	assign avg_only = avg_w;
	assign s0_only = s0_w;

	always @(*) begin
		if (fmt != 3'd0 && fmt != 3'd1 && fmt != 3'd6 && fmt != 3'd7)
			out_p = s0_w;
		else
			out_p = avg_w;
	end
endmodule

`default_nettype wire
