/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_port: the AMB port of the blitter's 2D engine (atg_blt_2d): the
 * request register, the outstanding reads and writes, the three line
 * buffers that the read responses fill (by tag: 0 and 1 the source lines,
 * 2 the destination line), and the error flag.
 *
 * issue (taken only while free, i.e. no request is held) loads the
 * request; it is held on req_* until the MC takes it.  rd_idle and wr_idle
 * are 1 when no read, no write is outstanding (the request register
 * included only once taken).  An error response sets fail until clr (the
 * kick).  The tag is {5'd0, write, slot}.  See spec/gpu/blt/atg_blt_port.md.
 */

`timescale 1ns / 1ps
`default_nettype none

`ifdef SYNTH_KINTEX7
(* keep_hierarchy = "yes" *)
`elsif SYNTH_KINTEX_ULTRASCALE
(* keep_hierarchy = "yes" *)
`endif
module atg_blt_port #(
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5	/* log2(LINE_BYTES) */
) (
	input wire clk,
	input wire rst_n,
	input wire clr,
	/* The engine's request */
	input wire issue,
	input wire i_write,
	input wire [ADDR_W-LINE_SHIFT-1:0] i_line,
	input wire [1:0] i_slot,
	input wire [LINE_BYTES-1:0] i_be,
	input wire [LINE_BYTES*8-1:0] i_wdata,
	output reg free,
	(* dont_touch = "true", equivalent_register_removal = "no" *) output reg free_leg,
	(* dont_touch = "true", equivalent_register_removal = "no" *) output reg free_st,
	(* dont_touch = "true", equivalent_register_removal = "no" *) output reg free_fsm,
	output wire rd_idle,
	output wire wr_idle,
`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) output reg fail,
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) output reg fail,
`else
	output reg fail,
`endif
	output reg [LINE_BYTES*8-1:0] buf0,
	output reg [LINE_BYTES*8-1:0] buf1,
	output reg [LINE_BYTES*8-1:0] bufd,
	/* The AMB port */
	output reg req_valid,
	input wire req_ready,
	output reg req_write,
	output reg [ADDR_W-LINE_SHIFT-1:0] req_line,
	(* extract_reset = "no" *) output reg [LINE_BYTES-1:0] req_be,
	output reg [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0] req_tag,
	input wire rsp_valid,
	input wire rsp_inhibit,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	localparam LA_W = ADDR_W - LINE_SHIFT;
	(* max_fanout = 16 *) reg [1:0] slot;
	(* max_fanout = 16 *) reg [3:0] wr_out;
	(* max_fanout = 16 *) reg [1:0] rd_out;
	(* max_fanout = 16 *) reg rd_idle_q, wr_idle_q;
	(* max_fanout = 16 *) wire q_fire = req_valid && req_ready;
	(* keep = "true" *) wire rsp_rd = rsp_valid && !rsp_inhibit && !rsp_write;
	(* keep = "true" *) wire rsp_wr = rsp_valid && !rsp_inhibit && rsp_write;
`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) wire can_issue_ctrl = issue;
	(* max_fanout = 16 *) wire can_issue_lo   = issue;
	(* max_fanout = 16 *) wire can_issue_hi   = issue;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) wire can_issue_ctrl = issue;
	(* max_fanout = 16 *) wire can_issue_lo   = issue;
	(* max_fanout = 16 *) wire can_issue_hi   = issue;
`else
	wire can_issue_ctrl = issue && free;
	wire can_issue_lo = issue && free;
	wire can_issue_hi = issue && free;
`endif

	assign rd_idle = rd_idle_q;
	assign wr_idle = wr_idle_q;
	assign req_tag = {5'd0, req_write, slot};

	/* The writes and the reads outstanding: parallel +1/-1 avoids serial add->sub (cuts ~19->10 cmos3). */
	wire [3:0] wr_p1, wr_m1, wr_next;
	wire [1:0] rd_p1, rd_m1, rd_next;
	wire [3:0] out_co_unused;
	wire wr_inc = q_fire && req_write;
	wire wr_dec = rsp_wr;
	wire rd_inc = q_fire && !req_write;
	wire rd_dec = rsp_rd;

	am_add #(.W(4)) a_wr (.a(wr_out), .b(4'd0), .ci(1'b1), .s(wr_p1), .co(out_co_unused[0]));
	am_sub #(.W(4)) s_wr (.a(wr_out), .b(4'd0), .bi(1'b1), .d(wr_m1), .bo(out_co_unused[1]));
	assign wr_next = (wr_inc == wr_dec) ? wr_out : (wr_inc ? wr_p1 : wr_m1);

	am_add #(.W(2)) a_rd (.a(rd_out), .b(2'd0), .ci(1'b1), .s(rd_p1), .co(out_co_unused[2]));
	am_sub #(.W(2)) s_rd (.a(rd_out), .b(2'd0), .bi(1'b1), .d(rd_m1), .bo(out_co_unused[3]));
	assign rd_next = (rd_inc == rd_dec) ? rd_out : (rd_inc ? rd_p1 : rd_m1);

	wire wr_out_is0 = (wr_out == 4'd0);
	wire wr_out_is1 = (wr_out == 4'd1);
	wire wr_idle_next = wr_inc ? 1'b0 : (wr_dec ? (wr_out_is1 || wr_out_is0) : wr_out_is0);
	wire rd_out_is0 = (rd_out == 2'd0);
	wire rd_out_is1 = (rd_out == 2'd1);
	wire rd_idle_next = rd_inc ? 1'b0 : (rd_dec ? (rd_out_is1 || rd_out_is0) : rd_out_is0);

`ifdef SYNTH_KINTEX7
	localparam NUM_SLICES = (LINE_BYTES * 8) / 16;

	(* keep = "true", dont_touch = "true" *) wire ce_b0_root, ce_b1_root, ce_bd_root;
	LUT5 #(.INIT(32'h00000002)) u_root_b0 (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_b0_root));
	LUT5 #(.INIT(32'h00000020)) u_root_b1 (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_b1_root));
	LUT5 #(.INIT(32'h00002200)) u_root_bd (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_bd_root));

	(* keep = "true", dont_touch = "true" *) wire iss_lo_root, iss_hi_root;
	LUT1 #(.INIT(2'b10)) u_iss_lo (.I0(i_write), .O(iss_lo_root));
	LUT1 #(.INIT(2'b10)) u_iss_hi (.I0(i_write), .O(iss_hi_root));

	genvar si;
	generate
		for (si = 0; si < NUM_SLICES; si = si + 1) begin : g_buf_slice
			(* keep = "true", dont_touch = "true" *) wire ce_b0;
			(* keep = "true", dont_touch = "true" *) wire ce_b1;
			(* keep = "true", dont_touch = "true" *) wire ce_bd;
			(* keep = "true", dont_touch = "true" *) wire ce_iss;

			LUT1 #(.INIT(2'b10)) u_buf_b0 (.I0(ce_b0_root), .O(ce_b0));
			LUT1 #(.INIT(2'b10)) u_buf_b1 (.I0(ce_b1_root), .O(ce_b1));
			LUT1 #(.INIT(2'b10)) u_buf_bd (.I0(ce_bd_root), .O(ce_bd));
			LUT1 #(.INIT(2'b10)) u_buf_iss (.I0(si < 8 ? iss_lo_root : iss_hi_root), .O(ce_iss));

			always @(posedge clk) begin
				if (ce_b0)
					buf0[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_b1)
					buf1[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_bd)
					bufd[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_iss)
					req_wdata[si*16 +: 16] <= i_wdata[si*16 +: 16];
			end
		end
	endgenerate

	(* max_fanout = 16 *) wire can_issue_be0 = issue;
	(* max_fanout = 16 *) wire can_issue_be1 = issue;

	always @(posedge clk) begin
		if (can_issue_be0)
			req_be[LINE_BYTES/2-1:0] <= i_be[LINE_BYTES/2-1:0];
		if (can_issue_be1)
			req_be[LINE_BYTES-1:LINE_BYTES/2] <= i_be[LINE_BYTES-1:LINE_BYTES/2];
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam NUM_SLICES = (LINE_BYTES * 8) / 16;

	(* keep = "true", dont_touch = "true" *) wire ce_b0_root, ce_b1_root, ce_bd_root;
	LUT5 #(.INIT(32'h00000002)) u_root_b0 (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_b0_root));
	LUT5 #(.INIT(32'h00000020)) u_root_b1 (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_b1_root));
	LUT5 #(.INIT(32'h00002200)) u_root_bd (.I0(rsp_valid), .I1(rsp_write), .I2(rsp_tag[0]), .I3(rsp_tag[1]), .I4(rsp_inhibit), .O(ce_bd_root));

	(* keep = "true", dont_touch = "true" *) wire iss_lo_root, iss_hi_root;
	assign iss_lo_root = i_write;
	assign iss_hi_root = i_write;

	genvar si;
	generate
		for (si = 0; si < NUM_SLICES; si = si + 1) begin : g_buf_slice
			(* keep = "true", dont_touch = "true" *) wire ce_b0;
			(* keep = "true", dont_touch = "true" *) wire ce_b1;
			(* keep = "true", dont_touch = "true" *) wire ce_bd;
			(* keep = "true", dont_touch = "true" *) wire ce_iss;

			assign ce_b0 = ce_b0_root;
			assign ce_b1 = ce_b1_root;
			assign ce_bd = ce_bd_root;
			assign ce_iss = si < 8 ? iss_lo_root : iss_hi_root;

			always @(posedge clk) begin
				if (ce_b0)
					buf0[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_b1)
					buf1[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_bd)
					bufd[si*16 +: 16] <= rsp_rdata[si*16 +: 16];
				if (ce_iss)
					req_wdata[si*16 +: 16] <= i_wdata[si*16 +: 16];
			end
		end
	endgenerate

	(* max_fanout = 16 *) wire can_issue_be0 = issue;
	(* max_fanout = 16 *) wire can_issue_be1 = issue;

	always @(posedge clk) begin
		if (can_issue_be0)
			req_be[LINE_BYTES/2-1:0] <= i_be[LINE_BYTES/2-1:0];
		if (can_issue_be1)
			req_be[LINE_BYTES-1:LINE_BYTES/2] <= i_be[LINE_BYTES-1:LINE_BYTES/2];
	end
`endif

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	(* keep = "true" *) reg [7:0] free_wd_rep;
	(* keep = "true" *) reg [1:0] free_be_rep;
	localparam WD_SLICE_BITS = (LINE_BYTES * 8) / 8;
`endif

	always @(posedge clk) begin
		if (can_issue_lo)
			req_line[15:0] <= i_line[15:0];
		if (can_issue_hi)
			req_line[LA_W-1:16] <= i_line[LA_W-1:16];
		if (can_issue_ctrl) begin
			slot <= i_slot;
		end
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		if (issue && free_be_rep[0])
			req_be[LINE_BYTES/2-1:0] <= i_be[LINE_BYTES/2-1:0];
		if (issue && free_be_rep[1])
			req_be[LINE_BYTES-1:LINE_BYTES/2] <= i_be[LINE_BYTES-1:LINE_BYTES/2];
		if (rsp_rd) begin
			case (rsp_tag[1:0])
			2'd0: buf0 <= rsp_rdata;
			2'd1: buf1 <= rsp_rdata;
			default: bufd <= rsp_rdata;
			endcase
		end
`endif
	end

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	genvar gw_sl;
	generate
		for (gw_sl = 0; gw_sl < 8; gw_sl = gw_sl + 1) begin : g_wd_slice
			wire we_wd = issue && free_wd_rep[gw_sl];
			always @(posedge clk) begin
				if (we_wd)
					req_wdata[gw_sl*WD_SLICE_BITS +: WD_SLICE_BITS] <= i_wdata[gw_sl*WD_SLICE_BITS +: WD_SLICE_BITS];
			end
		end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) wire q_fire_free, iss_free, q_fire_rep, iss_rep;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qff (.I0(q_fire),         .O(q_fire_free));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_isf (.I0(can_issue_ctrl), .O(iss_free));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_qfr (.I0(q_fire),         .O(q_fire_rep));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_isr (.I0(can_issue_ctrl), .O(iss_rep));

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			free     <= 1'b1;
			free_leg <= 1'b1;
			free_st  <= 1'b1;
			free_fsm <= 1'b1;
		end else begin
			if (q_fire_free)
				free <= 1'b1;
			if (iss_free)
				free <= 1'b0;
			if (q_fire_rep) begin
				free_leg <= 1'b1;
				free_st  <= 1'b1;
				free_fsm <= 1'b1;
			end
			if (iss_rep) begin
				free_leg <= 1'b0;
				free_st  <= 1'b0;
				free_fsm <= 1'b0;
			end
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) wire q_fire_free, iss_free, q_fire_rep, iss_rep;
	assign q_fire_free = q_fire;
	assign iss_free = can_issue_ctrl;
	assign q_fire_rep = q_fire;
	assign iss_rep = can_issue_ctrl;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			free     <= 1'b1;
			free_leg <= 1'b1;
			free_st  <= 1'b1;
			free_fsm <= 1'b1;
		end else begin
			if (q_fire_free)
				free <= 1'b1;
			if (iss_free)
				free <= 1'b0;
			if (q_fire_rep) begin
				free_leg <= 1'b1;
				free_st  <= 1'b1;
				free_fsm <= 1'b1;
			end
			if (iss_rep) begin
				free_leg <= 1'b0;
				free_st  <= 1'b0;
				free_fsm <= 1'b0;
			end
		end
	end
`endif

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			req_valid <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			free <= 1'b1;
			free_leg <= 1'b1;
			free_st <= 1'b1;
			free_fsm <= 1'b1;
			free_wd_rep <= 8'hFF;
			free_be_rep <= 2'b11;
`endif
			req_write <= 1'b0;
			wr_out <= 4'd0;
			rd_out <= 2'd0;
			wr_idle_q <= 1'b1;
			rd_idle_q <= 1'b1;
			fail <= 1'b0;
		end else begin
			wr_out <= wr_next;
			rd_out <= rd_next;
			wr_idle_q <= wr_idle_next;
			rd_idle_q <= rd_idle_next;
			if (clr)
				fail <= 1'b0;
			else if (rsp_valid && !rsp_inhibit && rsp_error)
				fail <= 1'b1;
			if (q_fire) begin
				req_valid <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				free <= 1'b1;
				free_leg <= 1'b1;
				free_st <= 1'b1;
				free_fsm <= 1'b1;
				free_wd_rep <= 8'hFF;
				free_be_rep <= 2'b11;
`endif
			end
			if (can_issue_ctrl) begin
				req_valid <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				free <= 1'b0;
				free_leg <= 1'b0;
				free_st <= 1'b0;
				free_fsm <= 1'b0;
				free_wd_rep <= 8'd0;
				free_be_rep <= 2'b00;
`endif
				req_write <= i_write;
			end
		end
	end

	wire unused_ok = &{1'b0, rsp_tag[7:2], out_co_unused};
endmodule

`default_nettype wire
