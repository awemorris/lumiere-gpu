/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_occ: the occlusion counter: counts the samples that pass every
 * per-sample test while ROP_CTRL[0] is 1; the command processor's QUERY
 * micro-op reads and clears it.
 *
 * count <= (clear ? 0 : count) + (inc_valid && ROP_CTRL[0] ? inc_n : 0):
 * a clear and an increment in the same cycle leave the increment.  40
 * bits, wrapping.  ROP_CTRL[0] takes effect on its STATE write.  See
 * spec/gpu/rop/atg_rop_occ.md.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_occ (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	input wire inc_valid,
	input wire [4:0] inc_n,
	input wire clear,
	output wire [39:0] count
);
	`include "ghci_regs.vh"

	reg on;	/* ROP_CTRL[0] */
	reg [15:0] count_lo_r;
	reg [15:0] count_mid_r;
	reg [7:0] count_hi_r;
	assign count = {count_hi_r, count_mid_r, count_lo_r};

`ifdef SYNTH_KINTEX7
	wire [2:0] clear_buf;
	wire [2:0] rst_n_buf;
	genvar gcb;
	generate
		for (gcb = 0; gcb < 3; gcb = gcb + 1) begin : g_occ_bufs
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cbuf (.I0(clear), .O(clear_buf[gcb]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rbuf (.I0(rst_n), .O(rst_n_buf[gcb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] clear_buf;
	wire [2:0] rst_n_buf;
	genvar gcb;
	generate
		for (gcb = 0; gcb < 3; gcb = gcb + 1) begin : g_occ_bufs
			assign clear_buf[gcb] = clear;
			assign rst_n_buf[gcb] = rst_n;
		end
	endgenerate
`else
	wire [2:0] clear_buf = {3{clear}};
	wire [2:0] rst_n_buf = {3{rst_n}};
`endif

	wire inc_en = inc_valid && on;
	wire [4:0] inc_val = inc_en ? inc_n : 5'd0;

	/* the count (from 0 after a clear) + this beat's samples */
	wire [39:0] count_next;
	wire cn_co_unused;
`ifdef SYNTH_KINTEX7
	wire [15:0] count_lo = clear_buf[0] ? 16'd0 : count[15:0];
	wire [15:0] count_mid = clear_buf[1] ? 16'd0 : count[31:16];
	wire [7:0] count_hi = clear_buf[2] ? 8'd0 : count[39:32];
	wire [39:0] count_in = {count_hi, count_mid, count_lo};
	am_add #(.W(40)) u_count (.a(count_in), .b({35'd0, inc_val}),
		.ci(1'b0), .s(count_next), .co(cn_co_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [15:0] count_lo = clear_buf[0] ? 16'd0 : count[15:0];
	wire [15:0] count_mid = clear_buf[1] ? 16'd0 : count[31:16];
	wire [7:0] count_hi = clear_buf[2] ? 8'd0 : count[39:32];
	wire [39:0] count_in = {count_hi, count_mid, count_lo};
	am_add #(.W(40)) u_count (.a(count_in), .b({35'd0, inc_val}),
		.ci(1'b0), .s(count_next), .co(cn_co_unused));
`else
	/* Parallel carry-select incrementer directly on count[39:0], clear mux at output (<= 9 cmos3) */
	wire [4:0] cnt_lo5_clr, cnt_lo5_raw;
	wire cnt_c5;
	am_add #(.W(5)) u_cnt_lo5_c (.a(5'd0),       .b(inc_val), .ci(1'b0), .s(cnt_lo5_clr), .co(cn_co_unused));
	am_add #(.W(5)) u_cnt_lo5_r (.a(count[4:0]), .b(inc_val), .ci(1'b0), .s(cnt_lo5_raw), .co(cnt_c5));
	wire p15_5  = (&count[10:5]) && (&count[15:11]);
	wire p31_lo = (&count[19:16]) && (&count[23:20]);
	wire p31_hi = (&count[27:24]) && (&count[31:28]);
	wire p31_16 = p31_lo && p31_hi;
	wire cnt_c16 = cnt_c5 && p15_5;
	wire cnt_c32 = cnt_c5 && p15_5 && p31_16;
	wire [10:0] cnt_15_5_p1;
	wire [15:0] cnt_31_16_p1;
	wire [7:0]  cnt_39_32_p1;
	wire co15_u, co31_u, co39_u;
	am_add #(.W(11)) u_cnt_p1_a (.a(count[15:5]),  .b(11'd0), .ci(1'b1), .s(cnt_15_5_p1),  .co(co15_u));
	am_add #(.W(16)) u_cnt_p1_b (.a(count[31:16]), .b(16'd0), .ci(1'b1), .s(cnt_31_16_p1), .co(co31_u));
	am_add #(.W(8))  u_cnt_p1_c (.a(count[39:32]), .b(8'd0),  .ci(1'b1), .s(cnt_39_32_p1), .co(co39_u));
	assign count_next[4:0]   = clear_buf[0] ? cnt_lo5_clr : cnt_lo5_raw;
	assign count_next[15:5]  = clear_buf[0] ? 11'd0 : (cnt_c5  ? cnt_15_5_p1  : count[15:5]);
	assign count_next[31:16] = clear_buf[1] ? 16'd0 : (cnt_c16 ? cnt_31_16_p1 : count[31:16]);
	assign count_next[39:32] = clear_buf[2] ? 8'd0  : (cnt_c32 ? cnt_39_32_p1 : count[39:32]);
	wire unused_co_all = co15_u & co31_u & co39_u;
`endif

	always @(posedge clk or negedge rst_n_buf[0]) begin
		if (!rst_n_buf[0]) begin
			on <= 1'b0;
			count_lo_r <= 16'd0;
		end else begin
			if (st_we && st_idx == GHCI_S_ROP_CTRL)
				on <= st_wdata[0];
			count_lo_r <= count_next[15:0];
		end
	end

	always @(posedge clk or negedge rst_n_buf[1]) begin
		if (!rst_n_buf[1]) begin
			count_mid_r <= 16'd0;
		end else begin
			count_mid_r <= count_next[31:16];
		end
	end

	always @(posedge clk or negedge rst_n_buf[2]) begin
		if (!rst_n_buf[2]) begin
			count_hi_r <= 8'd0;
		end else begin
			count_hi_r <= count_next[39:32];
		end
	end

	wire unused_ok = &{1'b0, st_wdata[31:1]};
endmodule

`default_nettype wire
