/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_fence: writes fences and results to memory, polls WAIT_FENCE
 * words, keeps FENCE_LAST and raises the FENCE interrupt; memory through
 * the CP's AMB port.
 *
 * One request at a time (f_ready in IDLE only):
 *   FENCE (op 0): write f_value to the word at f_addr (that word's byte
 *     enables); on the write's response FENCE_LAST = value and, with f_irq,
 *     ev_fence pulses.
 *   RESULT (op 1): write {f_value, f_aux} to the 8 bytes at f_addr.
 *   WAIT_FENCE (op 2): read the word at f_addr until
 *     (int32)(mem - f_value) >= 0, POLL cycles between reads.
 * f_busy is 1 from the request until its last response.  The sequencer
 * has already waited for the blocks; this unit only orders its own write
 * response before FENCE_LAST/ev_fence.  An error response is a fault of
 * kind 1 (aperture or AXI error) with the address; the unit then stops
 * until the core reset (rst_n).  FENCE_LAST (0x070) is also written by
 * the host (the driver restores it after RESET).  Session protection:
 * f_chk and f_slot come with the request (a firmware µop of a protected
 * session that is not SELF) and go with its memory requests (req_chk,
 * req_slot).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_fence #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter POLL = 64	/* WAIT_FENCE: cycles between two reads */
) (
	input wire clk,
	input wire rst_n,
	/* HCR register snoop (from atg_cp_regs) */
	input wire hcr_we,
	input wire [9:0] hcr_widx,
	input wire [31:0] hcr_wdata,
	input wire [31:0] hcr_wmask,
	input wire [9:0] hcr_ridx,
	output wire [31:0] hcr_rdata,
	/* Requests (from atg_cp_seq) */
	input wire f_valid,
	output wire f_ready,
	input wire [1:0] f_op,
	input wire [ADDR_W-1:0] f_addr,
	input wire [31:0] f_value,
	input wire [31:0] f_aux,
	input wire f_irq,
	input wire f_mem,
	input wire f_flush,
	input wire f_64,
	input wire f_chk,
	input wire [5:0] f_slot,
	output wire f_busy,
	/* Events */
	output wire ev_fence,
	output wire [31:0] fence_last,
	/* Fault report to atg_cp_ctl (one source of its `src_*` vectors) */
	output wire fault,
	output wire [3:0] fault_kind,
	output wire [31:0] fault_addr,
	output wire [7:0] fault_tag,
	/* AMB client port (design §4.1) */
	output wire req_valid,
	input wire req_ready,
	output wire req_write,
	output wire [ADDR_W-LINE_SHIFT-1:0] req_line,
	output wire [LINE_BYTES-1:0] req_be,
	output wire [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0] req_tag,
	output reg req_chk,
	output reg [5:0] req_slot,
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	`include "ghci_regs.vh"

	localparam WS = LINE_SHIFT - 2;
	localparam [9:0] I_LAST = 10'd28;

	localparam [2:0] F_IDLE = 3'd0;
	localparam [2:0] F_REQ = 3'd1;	/* request held until ready */
	localparam [2:0] F_RSP = 3'd2;	/* wait for the response */
	localparam [2:0] F_GAP = 3'd3;	/* WAIT_FENCE: before the next read */
	localparam [2:0] F_DEAD = 3'd4;	/* after an error, until reset */
	localparam [2:0] F_CHECK = 3'd5;	/* WAIT_FENCE: pipeline comparison */
	localparam [2:0] F_CHECK2 = 3'd6;
	localparam [2:0] F_NOMEM = 3'd7;	/* FENCE with !f_mem: write last from value */

	(* max_fanout = 16 *) reg [2:0] st;
	(* max_fanout = 16 *) reg [1:0] op;
	reg [ADDR_W-1:0] addr;
	reg [31:0] value;
	reg [31:0] aux;
	reg irq_on;
	(* max_fanout = 16 *) reg [31:0] last;
	(* extract_enable = "no" *) reg [7:0] gap;
	reg f_wide_r;
	reg passed_r;
`ifdef SYNTH_KINTEX7
	reg passed_32_r;
	reg p64_b3_gt_r, p64_b3_eq_r, p64_b2_gt_r, p64_b2_eq_r;
	reg p64_b1_gt_r, p64_b1_eq_r, p64_b0_ge_r;
	wire p64_hi32_gt = p64_b3_gt_r | (p64_b3_eq_r & p64_b2_gt_r);
	wire p64_hi32_eq = p64_b3_eq_r & p64_b2_eq_r;
	wire p64_lo32_ge = p64_b1_gt_r | (p64_b1_eq_r & p64_b0_ge_r);
	wire passed_64_chk = p64_hi32_gt | (p64_hi32_eq & p64_lo32_ge);
	wire passed_chk_w = f_wide_cmp_r ? passed_64_chk : passed_32_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg passed_32_r;
	reg p64_b3_gt_r, p64_b3_eq_r, p64_b2_gt_r, p64_b2_eq_r;
	reg p64_b1_gt_r, p64_b1_eq_r, p64_b0_ge_r;
	wire p64_hi32_gt = p64_b3_gt_r | (p64_b3_eq_r & p64_b2_gt_r);
	wire p64_hi32_eq = p64_b3_eq_r & p64_b2_eq_r;
	wire p64_lo32_ge = p64_b1_gt_r | (p64_b1_eq_r & p64_b0_ge_r);
	wire passed_64_chk = p64_hi32_gt | (p64_hi32_eq & p64_lo32_ge);
	wire passed_chk_w = f_wide_cmp_r ? passed_64_chk : passed_32_r;
`else
	reg passed_32_asic_r, p64_hi_lt_r, p64_hi_eq_r, p64_lo_lt_r;
	wire passed_chk_w = f_wide_cmp_r ? !(p64_hi_lt_r || (p64_hi_eq_r && p64_lo_lt_r)) : passed_32_asic_r;
`endif

`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire fence_rst_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_fence_rst (.I0(rst_n), .O(fence_rst_high));

	wire last_we_host = hcr_we && (hcr_widx == I_LAST);
	wire last_we_rsp = (st == F_RSP) && rsp_valid && !rsp_error && (op == 2'd0);
	wire last_we_nomem = (st == F_NOMEM);
	wire last_we_fence = last_we_rsp || last_we_nomem;
	wire [3:0] last_we_b;
	wire [3:0] last_we_fence_b;
	genvar g_lw;
	generate
		for (g_lw = 0; g_lw < 4; g_lw = g_lw + 1) begin : g_lwe_buf
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_lwe (.I0(last_we_host), .I1(last_we_fence), .O(last_we_b[g_lw]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_lwr (.I0(last_we_fence), .O(last_we_fence_b[g_lw]));
		end
	endgenerate

	wire [1:0] f_op1_b, f_op2_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fop0 (.I0(f_op == 2'd1), .O(f_op1_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fop1 (.I0(f_op == 2'd1), .O(f_op1_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_fop2_0 (.I0(f_op[0]), .I1(f_op[1]), .O(f_op2_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_fop2_1 (.I0(f_op[0]), .I1(f_op[1]), .O(f_op2_b[1]));

	wire [3:0] f64_be_b;
	genvar g_64b;
	generate
		for (g_64b = 0; g_64b < 4; g_64b = g_64b + 1) begin : g_f64_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_f64b (.I0(f_64), .O(f64_be_b[g_64b]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire fence_rst_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_fence_rst (.I0(rst_n), .O(fence_rst_high));

	wire last_we_host = hcr_we && (hcr_widx == I_LAST);
	wire last_we_rsp = (st == F_RSP) && rsp_valid && !rsp_error && (op == 2'd0);
	wire last_we_nomem = (st == F_NOMEM);
	wire last_we_fence = last_we_rsp || last_we_nomem;
	wire [3:0] last_we_b;
	wire [3:0] last_we_fence_b;
	genvar g_lw;
	generate
		for (g_lw = 0; g_lw < 4; g_lw = g_lw + 1) begin : g_lwe_buf
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_lwe (.I0(last_we_host), .I1(last_we_fence), .O(last_we_b[g_lw]));
			assign last_we_fence_b[g_lw] = last_we_fence;
		end
	endgenerate

	wire [1:0] f_op1_b, f_op2_b;
	assign f_op1_b[0] = f_op == 2'd1;
	assign f_op1_b[1] = f_op == 2'd1;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_fop2_0 (.I0(f_op[0]), .I1(f_op[1]), .O(f_op2_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_fop2_1 (.I0(f_op[0]), .I1(f_op[1]), .O(f_op2_b[1]));

	wire [3:0] f64_be_b;
	genvar g_64b;
	generate
		for (g_64b = 0; g_64b < 4; g_64b = g_64b + 1) begin : g_f64_buf
			assign f64_be_b[g_64b] = f_64;
		end
	endgenerate
`endif

	wire mem_word_we = (st == F_RSP) && rsp_valid && !rsp_error && (op == 2'd2);
`ifdef SYNTH_KINTEX7
	wire [3:0] mem_word_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_mw0 (.I0(mem_word_we), .O(mem_word_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_mw1 (.I0(mem_word_we), .O(mem_word_we_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_mw2 (.I0(mem_word_we), .O(mem_word_we_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_mw3 (.I0(mem_word_we), .O(mem_word_we_b[3]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] mem_word_we_b;
	assign mem_word_we_b[0] = mem_word_we;
	assign mem_word_we_b[1] = mem_word_we;
	assign mem_word_we_b[2] = mem_word_we;
	assign mem_word_we_b[3] = mem_word_we;
`endif

	wire [WS-1:0] word = addr[LINE_SHIFT-1:2];
	wire [WS-1:0] f_word_in = f_addr[LINE_SHIFT-1:2];
	wire is_read = op == 2'd2;

`ifdef SYNTH_KINTEX7
	wire [WS-1:0] fword_b0, fword_b1, fword_b2, fword_b3;
	genvar g_fw;
	generate
		for (g_fw = 0; g_fw < WS; g_fw = g_fw + 1) begin : g_fword_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwb0 (.I0(word[g_fw]), .O(fword_b0[g_fw]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwb1 (.I0(word[g_fw]), .O(fword_b1[g_fw]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwb2 (.I0(word[g_fw]), .O(fword_b2[g_fw]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwb3 (.I0(word[g_fw]), .O(fword_b3[g_fw]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [WS-1:0] fword_b0, fword_b1, fword_b2, fword_b3;
	genvar g_fw;
	generate
		for (g_fw = 0; g_fw < WS; g_fw = g_fw + 1) begin : g_fword_buf
			assign fword_b0[g_fw] = word[g_fw];
			assign fword_b1[g_fw] = word[g_fw];
			assign fword_b2[g_fw] = word[g_fw];
			assign fword_b3[g_fw] = word[g_fw];
		end
	endgenerate
`else
	wire [WS-1:0] fword_b0 = word;
	wire [WS-1:0] fword_b1 = word;
	wire [WS-1:0] fword_b2 = word;
	wire [WS-1:0] fword_b3 = word;
`endif

	(* dont_touch = "true" *) reg f_ready_r;
	assign f_ready = f_ready_r;
`ifdef SYNTH_KINTEX7
	assign f_busy = st != F_IDLE && st != F_DEAD;
	reg req_valid_r;
	always @(posedge clk or posedge fence_rst_high) begin
		if (fence_rst_high)
			req_valid_r <= 1'b0;
		else
			req_valid_r <= (st == F_REQ);
	end
	assign req_valid = req_valid_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	assign f_busy = st != F_IDLE && st != F_DEAD;
	reg req_valid_r;
	always @(posedge clk or posedge fence_rst_high) begin
		if (fence_rst_high)
			req_valid_r <= 1'b0;
		else
			req_valid_r <= (st == F_REQ);
	end
	assign req_valid = req_valid_r;
`else
	reg f_busy_r;
	reg req_valid_r;
	reg st_is_rsp_r;
	reg st_is_idle_r;
	reg st_is_req_r;
	assign f_busy = f_busy_r;
	assign req_valid = req_valid_r;
`endif
	assign req_write = !is_read;
	assign req_line = addr[ADDR_W-1:LINE_SHIFT];
	(* max_fanout = 16 *) reg f_wide_wdata_r;
`ifdef SYNTH_KINTEX7
	localparam integer NW_PAIR = LINE_BYTES / 8;
	wire [NW_PAIR*2-1:0] fw_sel_b;
	genvar g_ws, g_wb;
	generate
		for (g_ws = 0; g_ws < NW_PAIR; g_ws = g_ws + 1) begin : g_fws
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fws_lo (.I0(f_wide_wdata_r), .I1(op == 2'd1), .O(fw_sel_b[g_ws*2]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fws_hi (.I0(f_wide_wdata_r), .I1(op == 2'd1), .O(fw_sel_b[g_ws*2 + 1]));
			assign req_wdata[g_ws*64 +: 32] = value;
			assign req_wdata[g_ws*64 + 32 +: 16] = fw_sel_b[g_ws*2]     ? aux[15:0]  : value[15:0];
			assign req_wdata[g_ws*64 + 48 +: 16] = fw_sel_b[g_ws*2 + 1] ? aux[31:16] : value[31:16];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam integer NW_PAIR = LINE_BYTES / 8;
	wire [NW_PAIR*2-1:0] fw_sel_b;
	genvar g_ws, g_wb;
	generate
		for (g_ws = 0; g_ws < NW_PAIR; g_ws = g_ws + 1) begin : g_fws
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fws_lo (.I0(f_wide_wdata_r), .I1(op == 2'd1), .O(fw_sel_b[g_ws*2]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fws_hi (.I0(f_wide_wdata_r), .I1(op == 2'd1), .O(fw_sel_b[g_ws*2 + 1]));
			assign req_wdata[g_ws*64 +: 32] = value;
			assign req_wdata[g_ws*64 + 32 +: 16] = fw_sel_b[g_ws*2]     ? aux[15:0]  : value[15:0];
			assign req_wdata[g_ws*64 + 48 +: 16] = fw_sel_b[g_ws*2 + 1] ? aux[31:16] : value[31:16];
		end
	endgenerate
`else
	assign req_wdata = (f_wide_wdata_r || op == 2'd1) ? {(LINE_BYTES / 8){aux, value}} : {(LINE_BYTES / 4){value}};
`endif
	/* FENCE: the word (or 8-byte pair if f_wide); RESULT: the 8-byte pair (8-byte aligned). */
	reg [LINE_BYTES-1:0] be_word, be_pair;
	integer bw;
	always @(*)
		for (bw = 0; bw < LINE_BYTES / 4; bw = bw + 1) begin
			be_word[bw*4 +: 4] = {{(32 - WS){1'b0}}, f_word_in} == bw ? 4'hF : 4'h0;
			be_pair[bw*4 +: 4] = {{(33 - WS){1'b0}}, f_word_in[WS-1:1]} == bw / 2 ? 4'hF : 4'h0;
		end
	(* max_fanout = 16 *) reg [LINE_BYTES-1:0] req_be_r;
	assign req_be = req_be_r;
	assign req_tag = 8'd0;

	wire [31:0] mem_lo = {
		rsp_rdata[{fword_b3, 2'd3, 3'd0} +: 8],
		rsp_rdata[{fword_b2, 2'd2, 3'd0} +: 8],
		rsp_rdata[{fword_b1, 2'd1, 3'd0} +: 8],
		rsp_rdata[{fword_b0, 5'd0} +: 8]
	};
	wire [31:0] mem_hi = {
		rsp_rdata[{fword_b3[WS-1:1], 1'b1, 2'd3, 3'd0} +: 8],
		rsp_rdata[{fword_b2[WS-1:1], 1'b1, 2'd2, 3'd0} +: 8],
		rsp_rdata[{fword_b1[WS-1:1], 1'b1, 2'd1, 3'd0} +: 8],
		rsp_rdata[{fword_b0[WS-1:1], 1'b1, 5'd0} +: 8]
	};
	wire [63:0] mem_qword = {mem_hi, mem_lo};
	reg [63:0] mem_qword_r;
	(* max_fanout = 16 *) reg f_wide_cmp_r;

	/* 31-bit alternating NAND/NOR parallel-prefix unsigned a < b (<= 7 cmos3, zero am_sub). */
	function fn_cmp_lt31;
		input [30:0] a_in;
		input [30:0] b_in;
		reg [30:0] g0_n, t0_n, g1, t1, g2_n, t2_n, g3, t3, g4_n, t4_n, g5;
		begin
			g0_n = ~(b_in & ~a_in);
			t0_n = ~(b_in | ~a_in);
			g1   = ~(g0_n & (t0_n | {g0_n[29:0], 1'b1}));
			t1   = ~(t0_n | {t0_n[29:0], 1'b1});
			g2_n = ~(g1 | (t1 & {g1[28:0], 2'b00}));
			t2_n = ~(t1 & {t1[28:0], 2'b00});
			g3   = ~(g2_n & (t2_n | {g2_n[26:0], 4'hF}));
			t3   = ~(t2_n | {t2_n[26:0], 4'hF});
			g4_n = ~(g3 | (t3 & {g3[22:0], 8'h00}));
			t4_n = ~(t3 & {t3[22:0], 8'h00});
			g5   = ~(g4_n & (t4_n | {g4_n[14:0], 16'hFFFF}));
			fn_cmp_lt31 = g5[30];
		end
	endfunction

	/* The gap's count - 1. */
	wire [7:0] gap_n;
	wire gap_bo;

	am_sub #(.W(8)) gap_sub (.a(gap), .b(8'd0), .bi(1'b1), .d(gap_n), .bo(gap_bo));
	function fn_ge64;
		input [63:0] a;
		input [63:0] b;
		reg [65:0] av, bv;
		reg [21:0] g1, p1;
		reg [7:0]  g2, p2;
		reg [2:0]  g3, p3;
		reg e0, e1, e2, gt;
		integer j, k, m;
		begin
			av = {2'b00, a};
			bv = {2'b00, b};
			for (j = 0; j < 22; j = j + 1) begin
				e0 = ~(av[3*j+0] ^ bv[3*j+0]);
				e1 = ~(av[3*j+1] ^ bv[3*j+1]);
				e2 = ~(av[3*j+2] ^ bv[3*j+2]);
				p1[j] = e2 & e1 & e0;
				gt = (av[3*j+2] & ~bv[3*j+2]) | (e2 & av[3*j+1] & ~bv[3*j+1]) | (e2 & e1 & av[3*j+0] & ~bv[3*j+0]);
				g1[j] = (j == 0) ? (gt | p1[0]) : gt;
			end
			for (k = 0; k < 7; k = k + 1) begin
				g2[k] = g1[3*k+2] | (p1[3*k+2] & g1[3*k+1]) | (p1[3*k+2] & p1[3*k+1] & g1[3*k]);
				p2[k] = p1[3*k+2] & p1[3*k+1] & p1[3*k];
			end
			g2[7] = g1[21];
			p2[7] = p1[21];
			g3[0] = g2[2] | (p2[2] & g2[1]) | (p2[2] & p2[1] & g2[0]);
			p3[0] = p2[2] & p2[1] & p2[0];
			g3[1] = g2[5] | (p2[5] & g2[4]) | (p2[5] & p2[4] & g2[3]);
			p3[1] = p2[5] & p2[4] & p2[3];
			g3[2] = g2[7] | (p2[7] & g2[6]);
			p3[2] = p2[7] & p2[6];
			fn_ge64 = g3[2] | (p3[2] & g3[1]) | (p3[2] & p3[1] & g3[0]);
		end
	endfunction
	wire lt_lo31   = fn_cmp_lt31(mem_qword_r[30:0], value[30:0]);
	wire passed_32 = !(mem_qword_r[31] ^ value[31] ^ lt_lo31);
`ifdef SYNTH_KINTEX7
	function [1:0] fn_cmp16_k7;
		input [15:0] a;
		input [15:0] b;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, gt_lo, eq_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = gt2[7] | (eq2[7] & gt2[6]);
			eq_hi  = eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			gt_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & gt2[0]);
			eq_lo  = eq2[2] & eq2[1] & eq2[0];
			fn_cmp16_k7[1] = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & gt_lo);
			fn_cmp16_k7[0] = eq_hi & eq_mid & eq_lo;
		end
	endfunction
	wire [1:0] p64_b3_cmp = fn_cmp16_k7({~mem_qword_r[63], mem_qword_r[62:48]}, {~aux[31], aux[30:16]});
	wire [1:0] p64_b2_cmp = fn_cmp16_k7(mem_qword_r[47:32], aux[15:0]);
	wire [1:0] p64_b1_cmp = fn_cmp16_k7(mem_qword_r[31:16], value[31:16]);
	wire [1:0] p64_b0_cmp = fn_cmp16_k7(mem_qword_r[15:0],  value[15:0]);
	wire p64_b3_gt = p64_b3_cmp[1];
	wire p64_b3_eq = p64_b3_cmp[0];
	wire p64_b2_gt = p64_b2_cmp[1];
	wire p64_b2_eq = p64_b2_cmp[0];
	wire p64_b1_gt = p64_b1_cmp[1];
	wire p64_b1_eq = p64_b1_cmp[0];
	wire p64_b0_ge = p64_b0_cmp[1] | p64_b0_cmp[0];
	wire passed_64 = 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
	function [1:0] fn_cmp16_k7;
		input [15:0] a;
		input [15:0] b;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, gt_lo, eq_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = gt2[7] | (eq2[7] & gt2[6]);
			eq_hi  = eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			gt_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & gt2[0]);
			eq_lo  = eq2[2] & eq2[1] & eq2[0];
			fn_cmp16_k7[1] = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & gt_lo);
			fn_cmp16_k7[0] = eq_hi & eq_mid & eq_lo;
		end
	endfunction
	wire [1:0] p64_b3_cmp = fn_cmp16_k7({~mem_qword_r[63], mem_qword_r[62:48]}, {~aux[31], aux[30:16]});
	wire [1:0] p64_b2_cmp = fn_cmp16_k7(mem_qword_r[47:32], aux[15:0]);
	wire [1:0] p64_b1_cmp = fn_cmp16_k7(mem_qword_r[31:16], value[31:16]);
	wire [1:0] p64_b0_cmp = fn_cmp16_k7(mem_qword_r[15:0],  value[15:0]);
	wire p64_b3_gt = p64_b3_cmp[1];
	wire p64_b3_eq = p64_b3_cmp[0];
	wire p64_b2_gt = p64_b2_cmp[1];
	wire p64_b2_eq = p64_b2_cmp[0];
	wire p64_b1_gt = p64_b1_cmp[1];
	wire p64_b1_eq = p64_b1_cmp[0];
	wire p64_b0_ge = p64_b0_cmp[1] | p64_b0_cmp[0];
	wire passed_64 = 1'b0;
`else
	function fn_cmp_lt32;
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
			fn_cmp_lt32 = g5[31];
		end
	endfunction
	wire p64_hi_lt_w = fn_cmp_lt32({~mem_qword_r[63], mem_qword_r[62:32]}, {~aux[31], aux[30:0]});
	wire p64_hi_eq_w = (mem_qword_r[63:32] == aux[31:0]);
	wire p64_lo_lt_w = fn_cmp_lt32(mem_qword_r[31:0], value[31:0]);
	wire passed_64 = 1'b0;
`endif
	wire passed    = f_wide_cmp_r ? passed_64 : passed_32;

`ifdef SYNTH_KINTEX7
	assign fault = st == F_RSP && rsp_valid && rsp_error;
`elsif SYNTH_KINTEX_ULTRASCALE
	assign fault = st == F_RSP && rsp_valid && rsp_error;
`else
	assign fault = st_is_rsp_r && rsp_valid && rsp_error;
`endif
	assign fault_kind = GHCI_FAULT_KIND_FENCE_ERROR;
	assign fault_addr = addr[31:0];
	assign fault_tag = 8'd0;
`ifdef SYNTH_KINTEX7
	wire st_idle_c0, st_idle_c1, st_idle_c2;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c0));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c1));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle2 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c2));
	wire [8:0] f_val_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv0 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv1 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv2 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv3 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv4 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv5 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv6 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv7 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[7]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv8 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[8]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire st_idle_c0, st_idle_c1, st_idle_c2;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c0));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c1));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h01)) u_fidle2 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .O(st_idle_c2));
	wire [8:0] f_val_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv0 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv1 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv2 (.I0(f_valid), .I1(st_idle_c0), .O(f_val_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv3 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv4 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv5 (.I0(f_valid), .I1(st_idle_c1), .O(f_val_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv6 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv7 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[7]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_fv8 (.I0(f_valid), .I1(st_idle_c2), .O(f_val_b[8]));
`endif

	reg ev_fence_pulse;
	assign ev_fence = ((st == F_RSP) && rsp_valid && !rsp_error && (op == 2'd0) && irq_on) || ev_fence_pulse;
	assign fence_last = last;

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg [2:0] next_st_c;
	always @(*) begin
		next_st_c = st;
		case (st)
		F_IDLE:
			if (f_valid)
				next_st_c = ((f_op == 2'd0) && !f_mem) ? F_NOMEM : F_REQ;
		F_NOMEM:
			next_st_c = F_IDLE;
		F_REQ:
			if (req_ready)
				next_st_c = F_RSP;
		F_RSP:
			if (rsp_valid)
				next_st_c = rsp_error ? F_DEAD : (is_read ? F_CHECK : F_IDLE);
		F_CHECK:
			next_st_c = F_CHECK2;
		F_CHECK2:
			next_st_c = passed_chk_w ? F_IDLE : F_GAP;
		F_GAP:
			if (gap <= 8'd1)
				next_st_c = F_REQ;
		default: ;
		endcase
	end
	wire last_we_fence = ((st == F_RSP) && rsp_valid && !rsp_error && (op == 2'd0)) || (st == F_NOMEM);
	wire [31:0] last_host_wval = (last & ~hcr_wmask) | (hcr_wdata & hcr_wmask);
	wire last_host_idx_ok = (hcr_widx[9:5] == 5'd0) && (hcr_widx[4:0] == I_LAST[4:0]);
`endif

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge fence_rst_high) begin
		if (fence_rst_high) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge fence_rst_high) begin
		if (fence_rst_high) begin
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			f_busy_r <= 1'b0;
			req_valid_r <= 1'b0;
			st_is_rsp_r <= 1'b0;
			st_is_idle_r <= 1'b1;
			st_is_req_r <= 1'b0;
`endif
			st <= F_IDLE;
			f_ready_r <= 1'b1;
			op <= 2'd0;
			addr <= {ADDR_W{1'b0}};
			value <= 32'd0;
			aux <= 32'd0;
			irq_on <= 1'b0;
			req_chk <= 1'b0;
			req_slot <= 6'd0;
			last <= 32'd0;
			gap <= 8'd0;
			ev_fence_pulse <= 1'b0;
			f_wide_r <= 1'b0;
			f_wide_wdata_r <= 1'b0;
			f_wide_cmp_r <= 1'b0;
			passed_r <= 1'b0;
`ifdef SYNTH_KINTEX7
			passed_32_r <= 1'b0;
			p64_b3_gt_r <= 1'b0;
			p64_b3_eq_r <= 1'b0;
			p64_b2_gt_r <= 1'b0;
			p64_b2_eq_r <= 1'b0;
			p64_b1_gt_r <= 1'b0;
			p64_b1_eq_r <= 1'b0;
			p64_b0_ge_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
			passed_32_r <= 1'b0;
			p64_b3_gt_r <= 1'b0;
			p64_b3_eq_r <= 1'b0;
			p64_b2_gt_r <= 1'b0;
			p64_b2_eq_r <= 1'b0;
			p64_b1_gt_r <= 1'b0;
			p64_b1_eq_r <= 1'b0;
			p64_b0_ge_r <= 1'b0;
`else
			passed_32_asic_r <= 1'b0;
			p64_hi_lt_r <= 1'b0;
			p64_hi_eq_r <= 1'b0;
			p64_lo_lt_r <= 1'b0;
`endif
			req_be_r <= {LINE_BYTES{1'b0}};
			mem_qword_r <= 64'd0;
		end else begin
`ifdef SYNTH_KINTEX7
			if (last_we_b[0])
				last[7:0] <= last_we_fence_b[0] ? value[7:0] : ((last[7:0] & ~hcr_wmask[7:0]) | (hcr_wdata[7:0] & hcr_wmask[7:0]));
			if (last_we_b[1])
				last[15:8] <= last_we_fence_b[1] ? value[15:8] : ((last[15:8] & ~hcr_wmask[15:8]) | (hcr_wdata[15:8] & hcr_wmask[15:8]));
			if (last_we_b[2])
				last[23:16] <= last_we_fence_b[2] ? value[23:16] : ((last[23:16] & ~hcr_wmask[23:16]) | (hcr_wdata[23:16] & hcr_wmask[23:16]));
			if (last_we_b[3])
				last[31:24] <= last_we_fence_b[3] ? value[31:24] : ((last[31:24] & ~hcr_wmask[31:24]) | (hcr_wdata[31:24] & hcr_wmask[31:24]));
			if (mem_word_we_b[0])
				mem_qword_r[15:0] <= mem_qword[15:0];
			if (mem_word_we_b[1])
				mem_qword_r[31:16] <= mem_qword[31:16];
			if (mem_word_we_b[2])
				mem_qword_r[47:32] <= mem_qword[47:32];
			if (mem_word_we_b[3])
				mem_qword_r[63:48] <= mem_qword[63:48];
`elsif SYNTH_KINTEX_ULTRASCALE
			if (last_we_b[0])
				last[7:0] <= last_we_fence_b[0] ? value[7:0] : ((last[7:0] & ~hcr_wmask[7:0]) | (hcr_wdata[7:0] & hcr_wmask[7:0]));
			if (last_we_b[1])
				last[15:8] <= last_we_fence_b[1] ? value[15:8] : ((last[15:8] & ~hcr_wmask[15:8]) | (hcr_wdata[15:8] & hcr_wmask[15:8]));
			if (last_we_b[2])
				last[23:16] <= last_we_fence_b[2] ? value[23:16] : ((last[23:16] & ~hcr_wmask[23:16]) | (hcr_wdata[23:16] & hcr_wmask[23:16]));
			if (last_we_b[3])
				last[31:24] <= last_we_fence_b[3] ? value[31:24] : ((last[31:24] & ~hcr_wmask[31:24]) | (hcr_wdata[31:24] & hcr_wmask[31:24]));
			if (mem_word_we_b[0])
				mem_qword_r[15:0] <= mem_qword[15:0];
			if (mem_word_we_b[1])
				mem_qword_r[31:16] <= mem_qword[31:16];
			if (mem_word_we_b[2])
				mem_qword_r[47:32] <= mem_qword[47:32];
			if (mem_word_we_b[3])
				mem_qword_r[63:48] <= mem_qword[63:48];
`else
			f_busy_r <= !(st_is_idle_r && !f_valid) && (st != F_NOMEM) &&
				!((st == F_RSP) && rsp_valid && (rsp_error || !is_read)) &&
				!((st == F_CHECK2) && passed_chk_w) && (st != F_DEAD);
			req_valid_r <= (st_is_req_r && !req_ready) ||
				(st_is_idle_r && f_valid && ((f_op != 2'd0) || f_mem)) ||
				((st == F_GAP) && (gap <= 8'd1));
			st_is_rsp_r <= (st_is_req_r && req_ready) || ((st == F_RSP) && !rsp_valid);
			st_is_idle_r <= (st_is_idle_r && !f_valid) || (st == F_NOMEM) ||
				((st == F_RSP) && rsp_valid && !rsp_error && !is_read) ||
				((st == F_CHECK2) && passed_chk_w);
			st_is_req_r <= (st_is_req_r && !req_ready) ||
				(st_is_idle_r && f_valid && ((f_op != 2'd0) || f_mem)) ||
				((st == F_GAP) && (gap <= 8'd1));
			if (last_we_fence)
				last <= value;
			else if (hcr_we && last_host_idx_ok)
				last <= last_host_wval;
			if (mem_word_we)
				mem_qword_r <= mem_qword;
`endif
			ev_fence_pulse <= (st == F_IDLE) && f_valid && (f_op == 2'd0) && !f_mem && f_irq;
`ifdef SYNTH_KINTEX7
			if (f_val_b[7]) begin : g_be_lo
				integer kb;
				for (kb = 0; kb < 2; kb = kb + 1) begin
					req_be_r[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] <= f_op2_b[0] ? {(LINE_BYTES/4){1'b0}} :
						(f64_be_b[kb] || f_op1_b[kb>>1]) ? be_pair[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] :
						be_word[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)];
				end
			end
			if (f_val_b[8]) begin : g_be_hi
				integer kb;
				for (kb = 2; kb < 4; kb = kb + 1) begin
					req_be_r[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] <= f_op2_b[1] ? {(LINE_BYTES/4){1'b0}} :
						(f64_be_b[kb] || f_op1_b[kb>>1]) ? be_pair[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] :
						be_word[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)];
				end
			end
			if (f_val_b[0]) addr[15:0] <= (f_64 || f_op == 2'd1) ? {f_addr[15:3], 3'b000} : {f_addr[15:2], 2'b00};
			if (f_val_b[1]) addr[ADDR_W-1:16] <= {{(ADDR_W-32){1'b0}}, f_addr[31:16]};
			if (f_val_b[2]) value[15:0] <= f_value[15:0];
			if (f_val_b[3]) value[31:16] <= f_value[31:16];
			if (f_val_b[4]) aux[15:0] <= f_aux[15:0];
			if (f_val_b[5]) aux[31:16] <= f_aux[31:16];
`elsif SYNTH_KINTEX_ULTRASCALE
			if (f_val_b[7]) begin : g_be_lo
				integer kb;
				for (kb = 0; kb < 2; kb = kb + 1) begin
					req_be_r[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] <= f_op2_b[0] ? {(LINE_BYTES/4){1'b0}} :
						(f64_be_b[kb] || f_op1_b[kb>>1]) ? be_pair[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] :
						be_word[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)];
				end
			end
			if (f_val_b[8]) begin : g_be_hi
				integer kb;
				for (kb = 2; kb < 4; kb = kb + 1) begin
					req_be_r[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] <= f_op2_b[1] ? {(LINE_BYTES/4){1'b0}} :
						(f64_be_b[kb] || f_op1_b[kb>>1]) ? be_pair[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)] :
						be_word[kb*(LINE_BYTES/4) +: (LINE_BYTES/4)];
				end
			end
			if (f_val_b[0]) addr[15:0] <= (f_64 || f_op == 2'd1) ? {f_addr[15:3], 3'b000} : {f_addr[15:2], 2'b00};
			if (f_val_b[1]) addr[ADDR_W-1:16] <= {{(ADDR_W-32){1'b0}}, f_addr[31:16]};
			if (f_val_b[2]) value[15:0] <= f_value[15:0];
			if (f_val_b[3]) value[31:16] <= f_value[31:16];
			if (f_val_b[4]) aux[15:0] <= f_aux[15:0];
			if (f_val_b[5]) aux[31:16] <= f_aux[31:16];
`endif
`ifdef SYNTH_KINTEX7
			case (st)
`elsif SYNTH_KINTEX_ULTRASCALE
			case (st)
`else
			if (st_is_req_r) begin
				if (req_ready)
					st <= F_RSP;
			end else case (st)
`endif
			F_IDLE: begin
`ifdef SYNTH_KINTEX7
				if (f_val_b[6]) begin
					op <= f_op;
					f_wide_r <= f_64;
					f_wide_wdata_r <= f_64;
					f_wide_cmp_r <= f_64;
					irq_on <= f_irq;
					req_chk <= f_chk;
					req_slot <= f_slot;
					f_ready_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
				if (f_val_b[6]) begin
					op <= f_op;
					f_wide_r <= f_64;
					f_wide_wdata_r <= f_64;
					f_wide_cmp_r <= f_64;
					irq_on <= f_irq;
					req_chk <= f_chk;
					req_slot <= f_slot;
					f_ready_r <= 1'b0;
`else
				if (f_valid) begin
					op <= f_op;
					f_wide_r <= f_64;
					f_wide_wdata_r <= f_64;
					f_wide_cmp_r <= f_64;
					req_be_r <= (f_op == 2'd2) ? {LINE_BYTES{1'b0}} : (f_64 || f_op == 2'd1) ? be_pair : be_word;
					addr <= (f_64 || f_op == 2'd1) ? {f_addr[ADDR_W-1:3], 3'b000} : {f_addr[ADDR_W-1:2], 2'b00};
					value <= f_value;
					aux <= f_aux;
					irq_on <= f_irq;
					req_chk <= f_chk;
					req_slot <= f_slot;
					f_ready_r <= 1'b0;
`endif
					if ((f_op == 2'd0) && !f_mem) begin
						st <= F_NOMEM;
					end else begin
						st <= F_REQ;
					end
				end
			end
			F_NOMEM: begin
				st <= F_IDLE;
				f_ready_r <= 1'b1;
			end
			F_REQ:
				if (req_ready)
					st <= F_RSP;
			F_RSP:
				if (rsp_valid) begin
					if (rsp_error) begin
						st <= F_DEAD;
					end else if (is_read) begin
						st <= F_CHECK;
					end else begin
						st <= F_IDLE;
						f_ready_r <= 1'b1;
					end
				end
			F_CHECK: begin
`ifdef SYNTH_KINTEX7
				passed_32_r <= passed_32;
				p64_b3_gt_r <= p64_b3_gt;
				p64_b3_eq_r <= p64_b3_eq;
				p64_b2_gt_r <= p64_b2_gt;
				p64_b2_eq_r <= p64_b2_eq;
				p64_b1_gt_r <= p64_b1_gt;
				p64_b1_eq_r <= p64_b1_eq;
				p64_b0_ge_r <= p64_b0_ge;
`elsif SYNTH_KINTEX_ULTRASCALE
				passed_32_r <= passed_32;
				p64_b3_gt_r <= p64_b3_gt;
				p64_b3_eq_r <= p64_b3_eq;
				p64_b2_gt_r <= p64_b2_gt;
				p64_b2_eq_r <= p64_b2_eq;
				p64_b1_gt_r <= p64_b1_gt;
				p64_b1_eq_r <= p64_b1_eq;
				p64_b0_ge_r <= p64_b0_ge;
`else
				passed_32_asic_r <= passed_32;
				p64_hi_lt_r <= p64_hi_lt_w;
				p64_hi_eq_r <= p64_hi_eq_w;
				p64_lo_lt_r <= p64_lo_lt_w;
`endif
				st <= F_CHECK2;
			end
			F_CHECK2:
				if (passed_chk_w) begin
					st <= F_IDLE;
					f_ready_r <= 1'b1;
				end else begin
					gap <= POLL[7:0];
					st <= F_GAP;
				end
			F_GAP:
				if (gap <= 8'd1)
					st <= F_REQ;
				else
					gap <= gap_n;
			default:
				;
			endcase
		end
	end

`ifdef SYNTH_KINTEX7
	wire sel_last_pre = (hcr_ridx[5:0] == I_LAST[5:0]);
	wire [1:0] sel_last_b;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000002)) u_sl0 (.I0(sel_last_pre), .I1(hcr_ridx[6]), .I2(hcr_ridx[7]), .I3(hcr_ridx[8]), .I4(hcr_ridx[9]), .O(sel_last_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000002)) u_sl1 (.I0(sel_last_pre), .I1(hcr_ridx[6]), .I2(hcr_ridx[7]), .I3(hcr_ridx[8]), .I4(hcr_ridx[9]), .O(sel_last_b[1]));
	assign hcr_rdata = {sel_last_b[1] ? last[31:16] : 16'd0, sel_last_b[0] ? last[15:0] : 16'd0};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire sel_last_pre = (hcr_ridx[5:0] == I_LAST[5:0]);
	wire [1:0] sel_last_b;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000002)) u_sl0 (.I0(sel_last_pre), .I1(hcr_ridx[6]), .I2(hcr_ridx[7]), .I3(hcr_ridx[8]), .I4(hcr_ridx[9]), .O(sel_last_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000002)) u_sl1 (.I0(sel_last_pre), .I1(hcr_ridx[6]), .I2(hcr_ridx[7]), .I3(hcr_ridx[8]), .I4(hcr_ridx[9]), .O(sel_last_b[1]));
	assign hcr_rdata = {sel_last_b[1] ? last[31:16] : 16'd0, sel_last_b[0] ? last[15:0] : 16'd0};
`else
	assign hcr_rdata = (hcr_ridx == I_LAST) ? last : 32'd0;
`endif

	wire unused_ok = &{1'b0, rsp_write, rsp_tag, gap_bo, f_addr[2:0], hcr_ridx[9:6], f_flush};
endmodule

`default_nettype wire
