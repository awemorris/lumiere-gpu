/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_dc_dma: The DC's scan-out DMA: reads the framebuffer FB_BASE line by
 * line (FB_STRIDE, FB_SIZE) through the AMB and pushes pixel words into
 * the pixel FIFO towards pclk.  See spec/gpu/dc/atg_dc_dma.md.
 *
 * A frame starts at frame_start (with enable 1): the base, stride, size
 * and format are taken then.  Each row of FB_SIZE.w pixels (4 bytes, or 2
 * for RGB565) is read as ceil(row bytes / LINE_BYTES) lines from
 * base + y * stride; base and stride are LINE_BYTES aligned (the low bits
 * are ignored).  Read responses go to a line FIFO of LFIFO entries; a
 * request is made only when the outstanding requests and the lines
 * waiting fit in it, so the responses (which cannot be held back) always
 * have room.  At most one request every two cycles.  The unpacker sends
 * the pixels of each line in order, one per cycle, as px_data (a 32-bit
 * pixel, or RGB565 in the low 16 bits) with px_valid/px_ready.  An error
 * response gives pixels 0 (the MC raised the fault).
 *
 * frame_start during a frame and enable = 0 abort it: no new request (a
 * request already offered stays until req_ready), the line FIFO and the
 * unpacker are cleared, and the responses still to come are dropped: each
 * request carries the frame's epoch in its tag, and responses of an older
 * epoch are not kept.  px_valid may fall without px_ready at an abort.
 * idle is 1 when no request is offered or outstanding.
 *
 * scan_sum and scan_words are the sum and the number of the pixel words
 * pushed in the last frame (taken at the next frame_start or abort): a
 * check that the scan-out read the framebuffer, without a display.
 *
 * clear (the DC's CTRL.RESET) aborts like enable = 0 and starts no frame;
 * the count of outstanding requests is kept (rst_n is the board reset), so
 * the responses of reads issued before it are still counted and dropped.
 * err pulses for an error response of the current frame (its line is
 * black).  done is 1 when the frame has no line left to request and no
 * pixel left to push (or there is no frame): the cursor may read then.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_dc_dma #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter LFIFO_LOG2 = 5	/* 32 entries (ws001-p038) */
) (
	input wire clk,
	input wire rst_n,
	input wire clear,
	input wire enable,
	input wire frame_start,
	input wire [31:0] fb_base,
	input wire [31:0] fb_stride,
	input wire [31:0] fb_size,
	input wire [1:0] format,
	output wire px_valid,
	input wire px_ready,
	output wire [31:0] px_data,
	output wire idle,
	output wire done,
	output reg err,
	output reg [31:0] scan_sum,
	output reg [31:0] scan_words,
	/* AMB client port (design §4.1) */
	output reg req_valid,
	input wire req_ready,
	output wire req_write,
	output wire [ADDR_W-LINE_SHIFT-1:0] req_line,
	output wire [LINE_BYTES-1:0] req_be,
	output wire [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0] req_tag,
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	localparam LW = LINE_BYTES * 8;
	localparam AW = ADDR_W - LINE_SHIFT;
	localparam [LFIFO_LOG2:0] LFIFO = 1 << LFIFO_LOG2;
	localparam [LFIFO_LOG2:0] LFIFO_M1 = LFIFO - 1;
	localparam integer PPL32_I = LINE_BYTES / 4;	/* pixels in a line */
	localparam integer PPL16_I = LINE_BYTES / 2;
	localparam [5:0] PPL32 = PPL32_I[5:0];
	localparam [5:0] PPL16 = PPL16_I[5:0];
	localparam [5:0] PPL32_LAST = PPL32 - 6'd1;
	localparam [5:0] PPL16_LAST = PPL16 - 6'd1;
	localparam [5:0] PPL32_M2 = PPL32 - 6'd2;
	localparam [5:0] PPL16_M2 = PPL16 - 6'd2;
	localparam integer LB_I = LINE_BYTES - 1;
	localparam RB_HI_W = 18 - LINE_SHIFT;

	assign req_write = 1'b0;
	assign req_be = {LINE_BYTES{1'b0}};
	assign req_wdata = {LW{1'b0}};

	(* keep = "true", equivalent_register_removal = "no" *) reg enable_q, enable_q_ab0, enable_q_ab1;

`ifdef SYNTH_KINTEX7
	wire fs_ab_s, fs_st, fs_iss;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fs_abs (.I0(frame_start), .O(fs_ab_s));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fs_st  (.I0(frame_start), .O(fs_st));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fs_iss (.I0(frame_start), .O(fs_iss));

	wire clr_t0, clr_t1, clr_ab_s, clr_st, clr_iss;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_t0  (.I0(clear), .O(clr_t0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_t1  (.I0(clear), .O(clr_t1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_abs (.I0(clear), .O(clr_ab_s));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_st  (.I0(clear), .O(clr_st));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_iss (.I0(clear), .O(clr_iss));
	wire clr_k, clr_r, clr_ctl0, clr_ctl1, clr_used, clr_eq;
	wire clr_rql, clr_rqh, clr_curl, clr_curh, clr_rowl, clr_rowh, clr_nr, clr_strl, clr_strh, clr_err;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_k    (.I0(clr_t0), .O(clr_k));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_r    (.I0(clr_t0), .O(clr_r));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_ctl0 (.I0(clr_t0), .O(clr_ctl0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_ctl1 (.I0(clr_t0), .O(clr_ctl1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_used (.I0(clr_t0), .O(clr_used));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_eq   (.I0(clr_t0), .O(clr_eq));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_rql  (.I0(clr_t1), .O(clr_rql));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_rqh  (.I0(clr_t1), .O(clr_rqh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_curl (.I0(clr_t1), .O(clr_curl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_curh (.I0(clr_t1), .O(clr_curh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_rowl (.I0(clr_t1), .O(clr_rowl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_rowh (.I0(clr_t1), .O(clr_rowh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_nr   (.I0(clr_t1), .O(clr_nr));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_strl (.I0(clr_t1), .O(clr_strl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_strh (.I0(clr_t1), .O(clr_strh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr_err  (.I0(clr_t1), .O(clr_err));

	wire en_ab_s, en_st, en_iss;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_en_abs (.I0(enable), .O(en_ab_s));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_en_st  (.I0(enable), .O(en_st));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_en_iss (.I0(enable), .O(en_iss));

	wire abort = fs_ab_s || (enable_q_ab1 && !en_ab_s) || clr_ab_s;
	wire start = fs_st && en_st && !clr_st;

	wire start_t0, start_t1;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_t0 (.I0(start), .O(start_t0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_t1 (.I0(start), .O(start_t1));
	wire start_k, start_r, start_ctl0, start_ctl1, start_fnl, start_fnm, start_fhl, start_fhm;
	wire start_curl, start_curh, start_rowl, start_rowh, start_nr, start_strl, start_strh, start_up;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_k    (.I0(start_t0), .O(start_k));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_r    (.I0(start_t0), .O(start_r));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_ctl0 (.I0(start_t0), .O(start_ctl0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_ctl1 (.I0(start_t0), .O(start_ctl1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_fnl  (.I0(start_t0), .O(start_fnl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_fnm  (.I0(start_t0), .O(start_fnm));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_fhl  (.I0(start_t0), .O(start_fhl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_fhm  (.I0(start_t0), .O(start_fhm));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_curl (.I0(start_t1), .O(start_curl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_curh (.I0(start_t1), .O(start_curh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_rowl (.I0(start_t1), .O(start_rowl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_rowh (.I0(start_t1), .O(start_rowh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_nr   (.I0(start_t1), .O(start_nr));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_strl (.I0(start_t1), .O(start_strl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_strh (.I0(start_t1), .O(start_strh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_st_up   (.I0(start_t1), .O(start_up));

	wire abort_t1, abort_slow;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_t1   (.I0(abort), .O(abort_t1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_slow (.I0(abort), .O(abort_slow));
	wire abort_ctl, abort_used, abort_rsp_in, abort_rsp_k0, abort_rsp_k1, abort_rsp_e0, abort_rsp_e1;
	wire abort_lfifo0, abort_lfifo1;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_ctl    (.I0(abort), .O(abort_ctl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_used   (.I0(abort), .O(abort_used));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_rspin  (.I0(abort), .O(abort_rsp_in));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_rspk0  (.I0(abort), .O(abort_rsp_k0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_rspk1  (.I0(abort), .O(abort_rsp_k1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_rspe0  (.I0(abort), .O(abort_rsp_e0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_rspe1  (.I0(abort), .O(abort_rsp_e1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_lf0    (.I0(abort), .O(abort_lfifo0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_lf1    (.I0(abort), .O(abort_lfifo1));

	/* Fast single-LUT4 aborts directly from frame_start/enable_q_ab0/enable/clear (Rule 1 <= 3 LUTs, no shared sub-LUT). */
	wire abort_row_end, abort_act, abort_sum_l, abort_sum_h;
	wire abort_w_l, abort_w_h, abort_x, abort_row, abort_idx, abort_sp;
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_rend (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_row_end));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_act  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_act));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_suml (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sum_l));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_sumh (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sum_h));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_wl   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_w_l));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_wh   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_w_h));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_x    (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_x));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_row  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_row));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_idx  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_idx));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_sp   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sp));

	wire fmt_is16_ll, fmt_is16_lh, fmt_is16_ml, fmt_is16_mh, fmt_is16_n, fmt_is16_c;
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_ll (.I0(format[0]), .I1(format[1]), .O(fmt_is16_ll));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_lh (.I0(format[0]), .I1(format[1]), .O(fmt_is16_lh));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_ml (.I0(format[0]), .I1(format[1]), .O(fmt_is16_ml));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_mh (.I0(format[0]), .I1(format[1]), .O(fmt_is16_mh));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_n  (.I0(format[0]), .I1(format[1]), .O(fmt_is16_n));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_c  (.I0(format[0]), .I1(format[1]), .O(fmt_is16_c));

	wire [15:0] fb_w_c0, fb_w_c1, fb_w_nl0, fb_w_nl1, fb_w_ff, fb_h_c0, fb_h_c1;
	genvar gfb;
	generate
		for (gfb = 0; gfb < 16; gfb = gfb + 1) begin : g_fb_buf
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bw_c0  (.I0(fb_size[gfb]),      .O(fb_w_c0[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bw_c1  (.I0(fb_size[gfb]),      .O(fb_w_c1[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bw_nl0 (.I0(fb_size[gfb]),      .O(fb_w_nl0[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bw_nl1 (.I0(fb_size[gfb]),      .O(fb_w_nl1[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bw_ff  (.I0(fb_size[gfb]),      .O(fb_w_ff[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bh_c0  (.I0(fb_size[16 + gfb]), .O(fb_h_c0[gfb]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_bh_c1  (.I0(fb_size[16 + gfb]), .O(fb_h_c1[gfb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire fs_ab_s, fs_st, fs_iss;
	assign fs_ab_s = frame_start;
	assign fs_st = frame_start;
	assign fs_iss = frame_start;

	wire clr_t0, clr_t1, clr_ab_s, clr_st, clr_iss;
	assign clr_t0 = clear;
	assign clr_t1 = clear;
	assign clr_ab_s = clear;
	assign clr_st = clear;
	assign clr_iss = clear;
	wire clr_k, clr_r, clr_ctl0, clr_ctl1, clr_used, clr_eq;
	wire clr_rql, clr_rqh, clr_curl, clr_curh, clr_rowl, clr_rowh, clr_nr, clr_strl, clr_strh, clr_err;
	assign clr_k = clr_t0;
	assign clr_r = clr_t0;
	assign clr_ctl0 = clr_t0;
	assign clr_ctl1 = clr_t0;
	assign clr_used = clr_t0;
	assign clr_eq = clr_t0;
	assign clr_rql = clr_t1;
	assign clr_rqh = clr_t1;
	assign clr_curl = clr_t1;
	assign clr_curh = clr_t1;
	assign clr_rowl = clr_t1;
	assign clr_rowh = clr_t1;
	assign clr_nr = clr_t1;
	assign clr_strl = clr_t1;
	assign clr_strh = clr_t1;
	assign clr_err = clr_t1;

	wire en_ab_s, en_st, en_iss;
	assign en_ab_s = enable;
	assign en_st = enable;
	assign en_iss = enable;

	wire abort = fs_ab_s || (enable_q_ab1 && !en_ab_s) || clr_ab_s;
	wire start = fs_st && en_st && !clr_st;

	wire start_t0, start_t1;
	assign start_t0 = start;
	assign start_t1 = start;
	wire start_k, start_r, start_ctl0, start_ctl1, start_fnl, start_fnm, start_fhl, start_fhm;
	wire start_curl, start_curh, start_rowl, start_rowh, start_nr, start_strl, start_strh, start_up;
	assign start_k = start_t0;
	assign start_r = start_t0;
	assign start_ctl0 = start_t0;
	assign start_ctl1 = start_t0;
	assign start_fnl = start_t0;
	assign start_fnm = start_t0;
	assign start_fhl = start_t0;
	assign start_fhm = start_t0;
	assign start_curl = start_t1;
	assign start_curh = start_t1;
	assign start_rowl = start_t1;
	assign start_rowh = start_t1;
	assign start_nr = start_t1;
	assign start_strl = start_t1;
	assign start_strh = start_t1;
	assign start_up = start_t1;

	wire abort_t1, abort_slow;
	assign abort_t1 = abort;
	assign abort_slow = abort;
	wire abort_ctl, abort_used, abort_rsp_in, abort_rsp_k0, abort_rsp_k1, abort_rsp_e0, abort_rsp_e1;
	wire abort_lfifo0, abort_lfifo1;
	assign abort_ctl = abort;
	assign abort_used = abort;
	assign abort_rsp_in = abort;
	assign abort_rsp_k0 = abort;
	assign abort_rsp_k1 = abort;
	assign abort_rsp_e0 = abort;
	assign abort_rsp_e1 = abort;
	assign abort_lfifo0 = abort;
	assign abort_lfifo1 = abort;

	/* Fast single-LUT4 aborts directly from frame_start/enable_q_ab0/enable/clear (Rule 1 <= 3 LUTs, no shared sub-LUT). */
	wire abort_row_end, abort_act, abort_sum_l, abort_sum_h;
	wire abort_w_l, abort_w_h, abort_x, abort_row, abort_idx, abort_sp;
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_rend (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_row_end));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_act  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_act));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_suml (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sum_l));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_sumh (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sum_h));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_wl   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_w_l));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_wh   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_w_h));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_x    (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_x));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_row  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_row));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_idx  (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_idx));
	(* keep = "true" *) LUT4 #(.INIT(16'hFFAE)) u_ab_sp   (.I0(frame_start), .I1(enable_q_ab0), .I2(enable), .I3(clear), .O(abort_sp));

	wire fmt_is16_ll, fmt_is16_lh, fmt_is16_ml, fmt_is16_mh, fmt_is16_n, fmt_is16_c;
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_ll (.I0(format[0]), .I1(format[1]), .O(fmt_is16_ll));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_lh (.I0(format[0]), .I1(format[1]), .O(fmt_is16_lh));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_ml (.I0(format[0]), .I1(format[1]), .O(fmt_is16_ml));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_mh (.I0(format[0]), .I1(format[1]), .O(fmt_is16_mh));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_n  (.I0(format[0]), .I1(format[1]), .O(fmt_is16_n));
	(* keep = "true" *) LUT2 #(.INIT(4'b0100)) u_fmt_c  (.I0(format[0]), .I1(format[1]), .O(fmt_is16_c));

	wire [15:0] fb_w_c0, fb_w_c1, fb_w_nl0, fb_w_nl1, fb_w_ff, fb_h_c0, fb_h_c1;
	genvar gfb;
	generate
		for (gfb = 0; gfb < 16; gfb = gfb + 1) begin : g_fb_buf
			assign fb_w_c0[gfb] = fb_size[gfb];
			assign fb_w_c1[gfb] = fb_size[gfb];
			assign fb_w_nl0[gfb] = fb_size[gfb];
			assign fb_w_nl1[gfb] = fb_size[gfb];
			assign fb_w_ff[gfb] = fb_size[gfb];
			assign fb_h_c0[gfb] = fb_size[16 + gfb];
			assign fb_h_c1[gfb] = fb_size[16 + gfb];
		end
	endgenerate
`else
	wire fs_iss = frame_start;
	wire clr_iss = clear, clr_k = clear, clr_r = clear;
	wire clr_ctl0 = clear, clr_ctl1 = clear, clr_used = clear, clr_eq = clear, clr_rql = clear, clr_rqh = clear;
	wire clr_curl = clear, clr_curh = clear, clr_rowl = clear, clr_rowh = clear;
	wire clr_nr = clear, clr_strl = clear, clr_strh = clear, clr_err = clear;
	wire en_iss = enable;
	wire abort = frame_start || (enable_q && !enable) || clear;
	wire start = frame_start && enable && !clear;
	wire start_k = start, start_r = start, start_ctl0 = start, start_ctl1 = start;
	wire start_fnl = start, start_fnm = start, start_fhl = start, start_fhm = start;
	wire start_curl = start, start_curh = start, start_rowl = start, start_rowh = start;
	wire start_nr = start, start_strl = start, start_strh = start, start_up = start;
	wire abort_ctl = abort, abort_used = abort, abort_rsp_in = abort, abort_rsp_k0 = abort, abort_rsp_k1 = abort;
	wire abort_rsp_e0 = abort, abort_rsp_e1 = abort, abort_lfifo0 = abort, abort_lfifo1 = abort;
	wire abort_row_end = abort, abort_act = abort, abort_sum_l = abort, abort_sum_h = abort;
	wire abort_w_l = abort, abort_w_h = abort, abort_x = abort, abort_row = abort, abort_idx = abort, abort_sp = abort;
	wire abort_t1 = abort, abort_slow = abort;
	wire fmt_is16_ll = (format == 2'd2), fmt_is16_lh = (format == 2'd2);
	wire fmt_is16_ml = (format == 2'd2), fmt_is16_mh = (format == 2'd2);
	wire fmt_is16_n = (format == 2'd2), fmt_is16_c = (format == 2'd2);
	wire [15:0] fb_w_c0 = fb_size[15:0], fb_w_c1 = fb_size[15:0];
	wire [15:0] fb_w_nl0 = fb_size[15:0], fb_w_nl1 = fb_size[15:0], fb_w_ff = fb_size[15:0];
	wire [15:0] fb_h_c0 = fb_size[31:16], fb_h_c1 = fb_size[31:16];
`endif
	reg [3:0] epoch;
	reg [3:0] tag_q;
	assign req_tag = {4'd0, tag_q};

	/* The frame's parameters. */
	reg [13:0] f_stride_lo;
	reg [AW-1:14] f_stride_hi;
	wire [AW-1:0] f_stride = {f_stride_hi, f_stride_lo};
	reg [15:0] f_w, f_h;
	(* keep = "true" *) reg f_16, f_16_lo0, f_16_lo1, f_16_hi0, f_16_hi1;
	localparam RB16_W = 17 - LINE_SHIFT;
	localparam RB32_W = 18 - LINE_SHIFT;
	wire [15:0] rb_hi_32 = {{(16 - RB32_W){1'b0}}, fb_w_nl0[15:LINE_SHIFT-2]};
	wire [15:0] rb_hi_16 = {{(16 - RB16_W){1'b0}}, fb_w_nl1[15:LINE_SHIFT-1]};
	wire rb_lo_nz_32 = |fb_w_nl0[LINE_SHIFT-3:0];
	wire rb_lo_nz_16 = |fb_w_nl1[LINE_SHIFT-2:0];
	wire [15:0] calc_nl_last, calc_nl_m2;
	wire nl_is_1;
`ifdef SYNTH_ASIC
	/* Balanced 4-bit nibble subtractors computed in parallel before format/lo_nz mux (<= 10 cmos3). */
	wire [3:0] r32_1_n0, r32_1_n1, r32_1_n2, r32_1_n3;
	wire [3:0] r16_1_n0, r16_1_n1, r16_1_n2, r16_1_n3;
	wire [3:0] r32_2_n0, r32_2_n1, r32_2_n2, r32_2_n3;
	wire [3:0] r16_2_n0, r16_2_n1, r16_2_n2, r16_2_n3;
	wire [15:0] row_co_unused_v;
	am_sub #(.W(4)) s_r32_10 (.a(rb_hi_32[3:0]),   .b(4'd1), .bi(1'b0), .d(r32_1_n0), .bo(row_co_unused_v[0]));
	am_sub #(.W(4)) s_r32_11 (.a(rb_hi_32[7:4]),   .b(4'd1), .bi(1'b0), .d(r32_1_n1), .bo(row_co_unused_v[1]));
	am_sub #(.W(4)) s_r32_12 (.a(rb_hi_32[11:8]),  .b(4'd1), .bi(1'b0), .d(r32_1_n2), .bo(row_co_unused_v[2]));
	am_sub #(.W(4)) s_r32_13 (.a(rb_hi_32[15:12]), .b(4'd1), .bi(1'b0), .d(r32_1_n3), .bo(row_co_unused_v[3]));
	wire r32b1_0 = ~(|rb_hi_32[3:0]);
	wire r32b1_1 = r32b1_0 && ~(|rb_hi_32[7:4]);
	wire r32b1_2 = r32b1_1 && ~(|rb_hi_32[11:8]);
	wire [15:0] rb_hi_32_m1 = {r32b1_2 ? r32_1_n3 : rb_hi_32[15:12], r32b1_1 ? r32_1_n2 : rb_hi_32[11:8], r32b1_0 ? r32_1_n1 : rb_hi_32[7:4], r32_1_n0};

	am_sub #(.W(4)) s_r16_10 (.a(rb_hi_16[3:0]),   .b(4'd1), .bi(1'b0), .d(r16_1_n0), .bo(row_co_unused_v[4]));
	am_sub #(.W(4)) s_r16_11 (.a(rb_hi_16[7:4]),   .b(4'd1), .bi(1'b0), .d(r16_1_n1), .bo(row_co_unused_v[5]));
	am_sub #(.W(4)) s_r16_12 (.a(rb_hi_16[11:8]),  .b(4'd1), .bi(1'b0), .d(r16_1_n2), .bo(row_co_unused_v[6]));
	am_sub #(.W(4)) s_r16_13 (.a(rb_hi_16[15:12]), .b(4'd1), .bi(1'b0), .d(r16_1_n3), .bo(row_co_unused_v[7]));
	wire r16b1_0 = ~(|rb_hi_16[3:0]);
	wire r16b1_1 = r16b1_0 && ~(|rb_hi_16[7:4]);
	wire r16b1_2 = r16b1_1 && ~(|rb_hi_16[11:8]);
	wire [15:0] rb_hi_16_m1 = {r16b1_2 ? r16_1_n3 : rb_hi_16[15:12], r16b1_1 ? r16_1_n2 : rb_hi_16[11:8], r16b1_0 ? r16_1_n1 : rb_hi_16[7:4], r16_1_n0};

	am_sub #(.W(4)) s_r32_20 (.a(rb_hi_32[3:0]),   .b(4'd2), .bi(1'b0), .d(r32_2_n0), .bo(row_co_unused_v[8]));
	am_sub #(.W(4)) s_r32_21 (.a(rb_hi_32[7:4]),   .b(4'd1), .bi(1'b0), .d(r32_2_n1), .bo(row_co_unused_v[9]));
	am_sub #(.W(4)) s_r32_22 (.a(rb_hi_32[11:8]),  .b(4'd1), .bi(1'b0), .d(r32_2_n2), .bo(row_co_unused_v[10]));
	am_sub #(.W(4)) s_r32_23 (.a(rb_hi_32[15:12]), .b(4'd1), .bi(1'b0), .d(r32_2_n3), .bo(row_co_unused_v[11]));
	wire r32b2_0 = ~(|rb_hi_32[3:1]);
	wire r32b2_1 = r32b2_0 && ~(|rb_hi_32[7:4]);
	wire r32b2_2 = r32b2_1 && ~(|rb_hi_32[11:8]);
	wire [15:0] rb_hi_32_m2 = {r32b2_2 ? r32_2_n3 : rb_hi_32[15:12], r32b2_1 ? r32_2_n2 : rb_hi_32[11:8], r32b2_0 ? r32_2_n1 : rb_hi_32[7:4], r32_2_n0};

	am_sub #(.W(4)) s_r16_20 (.a(rb_hi_16[3:0]),   .b(4'd2), .bi(1'b0), .d(r16_2_n0), .bo(row_co_unused_v[12]));
	am_sub #(.W(4)) s_r16_21 (.a(rb_hi_16[7:4]),   .b(4'd1), .bi(1'b0), .d(r16_2_n1), .bo(row_co_unused_v[13]));
	am_sub #(.W(4)) s_r16_22 (.a(rb_hi_16[11:8]),  .b(4'd1), .bi(1'b0), .d(r16_2_n2), .bo(row_co_unused_v[14]));
	am_sub #(.W(4)) s_r16_23 (.a(rb_hi_16[15:12]), .b(4'd1), .bi(1'b0), .d(r16_2_n3), .bo(row_co_unused_v[15]));
	wire r16b2_0 = ~(|rb_hi_16[3:1]);
	wire r16b2_1 = r16b2_0 && ~(|rb_hi_16[7:4]);
	wire r16b2_2 = r16b2_1 && ~(|rb_hi_16[11:8]);
	wire [15:0] rb_hi_16_m2 = {r16b2_2 ? r16_2_n3 : rb_hi_16[15:12], r16b2_1 ? r16_2_n2 : rb_hi_16[11:8], r16b2_0 ? r16_2_n1 : rb_hi_16[7:4], r16_2_n0};

	wire [15:0] nl_last_32 = rb_lo_nz_32 ? rb_hi_32    : rb_hi_32_m1;
	wire [15:0] nl_last_16 = rb_lo_nz_16 ? rb_hi_16    : rb_hi_16_m1;
	wire [15:0] nl_m2_32   = rb_lo_nz_32 ? rb_hi_32_m1 : rb_hi_32_m2;
	wire [15:0] nl_m2_16   = rb_lo_nz_16 ? rb_hi_16_m1 : rb_hi_16_m2;
	assign calc_nl_last = {fmt_is16_lh ? nl_last_16[15:8] : nl_last_32[15:8], fmt_is16_ll ? nl_last_16[7:0] : nl_last_32[7:0]};
	assign calc_nl_m2   = {fmt_is16_mh ? nl_m2_16[15:8]   : nl_m2_32[15:8],   fmt_is16_ml ? nl_m2_16[7:0]   : nl_m2_32[7:0]};
	wire nl_is_1_32 = ~(|rb_hi_32) || (~(|(rb_hi_32 ^ 16'd1)) && !rb_lo_nz_32);
	wire nl_is_1_16 = ~(|rb_hi_16) || (~(|(rb_hi_16 ^ 16'd1)) && !rb_lo_nz_16);
	assign nl_is_1 = fmt_is16_n ? nl_is_1_16 : nl_is_1_32;
`else
	wire [15:0] rb_hi_32_m1, rb_hi_32_m2, rb_hi_16_m1, rb_hi_16_m2;
	wire [15:0] row_co_unused_v;
	am_sub #(.W(16)) s_r32_1 (.a(rb_hi_32), .b(16'd1), .bi(1'b0), .d(rb_hi_32_m1), .bo(row_co_unused_v[0]));
	am_sub #(.W(16)) s_r32_2 (.a(rb_hi_32), .b(16'd2), .bi(1'b0), .d(rb_hi_32_m2), .bo(row_co_unused_v[1]));
	am_sub #(.W(16)) s_r16_1 (.a(rb_hi_16), .b(16'd1), .bi(1'b0), .d(rb_hi_16_m1), .bo(row_co_unused_v[2]));
	am_sub #(.W(16)) s_r16_2 (.a(rb_hi_16), .b(16'd2), .bi(1'b0), .d(rb_hi_16_m2), .bo(row_co_unused_v[3]));
	assign row_co_unused_v[15:4] = 12'd0;
`ifdef SYNTH_KINTEX7
	wire lnz32_ll, lnz32_lh, lnz32_ml, lnz32_mh;
	wire lnz16_ll, lnz16_lh, lnz16_ml, lnz16_mh;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz32_ll (.I0(rb_lo_nz_32), .O(lnz32_ll));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz32_lh (.I0(rb_lo_nz_32), .O(lnz32_lh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz32_ml (.I0(rb_lo_nz_32), .O(lnz32_ml));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz32_mh (.I0(rb_lo_nz_32), .O(lnz32_mh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz16_ll (.I0(rb_lo_nz_16), .O(lnz16_ll));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz16_lh (.I0(rb_lo_nz_16), .O(lnz16_lh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz16_ml (.I0(rb_lo_nz_16), .O(lnz16_ml));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_lnz16_mh (.I0(rb_lo_nz_16), .O(lnz16_mh));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire lnz32_ll, lnz32_lh, lnz32_ml, lnz32_mh;
	wire lnz16_ll, lnz16_lh, lnz16_ml, lnz16_mh;
	assign lnz32_ll = rb_lo_nz_32;
	assign lnz32_lh = rb_lo_nz_32;
	assign lnz32_ml = rb_lo_nz_32;
	assign lnz32_mh = rb_lo_nz_32;
	assign lnz16_ll = rb_lo_nz_16;
	assign lnz16_lh = rb_lo_nz_16;
	assign lnz16_ml = rb_lo_nz_16;
	assign lnz16_mh = rb_lo_nz_16;
`else
	wire lnz32_ll = rb_lo_nz_32, lnz32_lh = rb_lo_nz_32;
	wire lnz32_ml = rb_lo_nz_32, lnz32_mh = rb_lo_nz_32;
	wire lnz16_ll = rb_lo_nz_16, lnz16_lh = rb_lo_nz_16;
	wire lnz16_ml = rb_lo_nz_16, lnz16_mh = rb_lo_nz_16;
`endif
	wire [15:0] nl_last_32 = {lnz32_lh ? rb_hi_32[15:8]    : rb_hi_32_m1[15:8], lnz32_ll ? rb_hi_32[7:0]    : rb_hi_32_m1[7:0]};
	wire [15:0] nl_last_16 = {lnz16_lh ? rb_hi_16[15:8]    : rb_hi_16_m1[15:8], lnz16_ll ? rb_hi_16[7:0]    : rb_hi_16_m1[7:0]};
	wire [15:0] nl_m2_32   = {lnz32_mh ? rb_hi_32_m1[15:8] : rb_hi_32_m2[15:8], lnz32_ml ? rb_hi_32_m1[7:0] : rb_hi_32_m2[7:0]};
	wire [15:0] nl_m2_16   = {lnz16_mh ? rb_hi_16_m1[15:8] : rb_hi_16_m2[15:8], lnz16_ml ? rb_hi_16_m1[7:0] : rb_hi_16_m2[7:0]};
	assign calc_nl_last = {fmt_is16_lh ? nl_last_16[15:8] : nl_last_32[15:8], fmt_is16_ll ? nl_last_16[7:0] : nl_last_32[7:0]};
	assign calc_nl_m2   = {fmt_is16_mh ? nl_m2_16[15:8]   : nl_m2_32[15:8],   fmt_is16_ml ? nl_m2_16[7:0]   : nl_m2_32[7:0]};
	wire nl_is_1_32 = ~(|rb_hi_32) || (~(|(rb_hi_32 ^ 16'd1)) && !rb_lo_nz_32);
	wire nl_is_1_16 = ~(|rb_hi_16) || (~(|(rb_hi_16 ^ 16'd1)) && !rb_lo_nz_16);
	assign nl_is_1 = fmt_is16_n ? nl_is_1_16 : nl_is_1_32;
`endif

	/* Requests. */
	reg rq_active;
	(* extract_reset = "no" *) reg [15:0] rq_k;
	(* extract_reset = "no" *) reg [15:0] rq_row;
	reg k_is_nl_r, row_is_last_r;
	reg [15:0] f_nl;
	reg [15:0] rq_row_line_lo;
	reg [AW-1:16] rq_row_line_hi;
	wire [AW-1:0] rq_row_line = {rq_row_line_hi, rq_row_line_lo};
	(* keep = "true", equivalent_register_removal = "no" *) reg [15:0] rq_next_row_lo, rq_next_row_lo_r;
	reg [AW-1:16] rq_next_row_hi;
	wire [AW-1:0] rq_next_row_r = {rq_next_row_hi, rq_next_row_lo};
	reg [1:0] nrow_st;
	reg nrow_c16_q;

	reg [13:0] rq_cur_lo;
	reg [AW-1:14] rq_cur_hi;
	wire [AW-1:0] rq_cur = {rq_cur_hi, rq_cur_lo};

	reg [13:0] req_line_lo;
	reg [AW-1:14] req_line_hi;
	assign req_line = {req_line_hi, req_line_lo};
	reg [LFIFO_LOG2:0] inflight;
`ifdef SYNTH_KINTEX7
	wire req_valid_iss, req_valid_take, req_valid_idle, req_valid_clrl, req_valid_clrh, req_valid_ctl;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_iss  (.I0(req_valid), .O(req_valid_iss));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_take (.I0(req_valid), .O(req_valid_take));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_idle (.I0(req_valid), .O(req_valid_idle));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_clrl (.I0(req_valid), .O(req_valid_clrl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_clrh (.I0(req_valid), .O(req_valid_clrh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rv_ctl  (.I0(req_valid), .O(req_valid_ctl));
	wire req_ready_take, req_ready_iss;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rr_take (.I0(req_ready), .O(req_ready_take));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rr_iss  (.I0(req_ready), .O(req_ready_iss));
	wire take = req_valid_take && req_ready_take;
	wire take_infl, take_used, take_ctl, take_clrl, take_clrh;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_infl (.I0(take), .O(take_infl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_used (.I0(take), .O(take_used));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_ctl  (.I0(take), .O(take_ctl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_clrl (.I0(take), .O(take_clrl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_tk_clrh (.I0(take), .O(take_clrh));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire req_valid_iss, req_valid_take, req_valid_idle, req_valid_clrl, req_valid_clrh, req_valid_ctl;
	assign req_valid_iss = req_valid;
	assign req_valid_take = req_valid;
	assign req_valid_idle = req_valid;
	assign req_valid_clrl = req_valid;
	assign req_valid_clrh = req_valid;
	assign req_valid_ctl = req_valid;
	wire req_ready_take, req_ready_iss;
	assign req_ready_take = req_ready;
	assign req_ready_iss = req_ready;
	wire take = req_valid_take && req_ready_take;
	wire take_infl, take_used, take_ctl, take_clrl, take_clrh;
	assign take_infl = take;
	assign take_used = take;
	assign take_ctl = take;
	assign take_clrl = take;
	assign take_clrh = take;
`else
	wire req_valid_iss = req_valid, req_valid_idle = req_valid;
	wire req_valid_clrl = req_valid, req_valid_clrh = req_valid, req_valid_ctl = req_valid;
	wire req_ready_iss = req_ready;
	wire take = req_valid && req_ready;
	wire take_infl = take, take_used = take, take_ctl = take, take_clrl = take, take_clrh = take;
`endif
	wire clr_rqline_lo = clr_rql && (!req_valid_clrl || take_clrl);
	wire clr_rqline_hi = clr_rqh && (!req_valid_clrh || take_clrh);
	wire rsp_read = rsp_valid && !rsp_write;
	localparam [LFIFO_LOG2-1:0] LFIFO_M1_LO = LFIFO - 1;
	localparam [LFIFO_LOG2-1:0] LFIFO_M2_LO = LFIFO - 2;
	(* max_fanout = 16 *) reg [LFIFO_LOG2:0] used;
	reg used_ge31_r, used_ge32_r;
	wire [LFIFO_LOG2:0] used_inc, used_dec;
	wire [1:0] u_co_unused;
	am_add #(.W(LFIFO_LOG2 + 1)) a_uinc (.a(used), .b({(LFIFO_LOG2 + 1){1'b0}}), .ci(1'b1), .s(used_inc), .co(u_co_unused[0]));
	am_sub #(.W(LFIFO_LOG2 + 1)) s_udec (.a(used), .b({(LFIFO_LOG2 + 1){1'b0}}), .bi(1'b1), .d(used_dec), .bo(u_co_unused[1]));

`ifdef SYNTH_KINTEX7
	wire rq_active_iss, rq_active_done;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rqa (.I0(rq_active), .O(rq_active_iss));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rqd (.I0(rq_active), .O(rq_active_done));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rq_active_iss, rq_active_done;
	assign rq_active_iss = rq_active;
	assign rq_active_done = rq_active;
`else
	wire rq_active_iss = rq_active;
	wire rq_active_done = rq_active;
`endif
	(* keep = "true", equivalent_register_removal = "no" *) reg nrow_rdy_r, nrow_rdy_nr, nrow_rdy_nr2, nrow_rdy_nhi;
	wire iss_room = req_valid_iss ? (!used_ge31_r && req_ready_iss) : !used_ge32_r;
	wire issue_step = !fs_iss && !clr_iss && en_iss && rq_active_iss && (!k_is_nl_r || nrow_rdy_r) && iss_room;

	assign idle = !req_valid_idle && inflight == {(LFIFO_LOG2 + 1){1'b0}};

	/*
	 * The requests' sums: the lines in flight or waiting, the next counts
	 * (in flight, epoch, row, line of the row, the next row's first line,
	 * the line), and the last line of a row and the last row.
	 */
	wire [LFIFO_LOG2:0] infl_inc, infl_dec;
	wire [LFIFO_LOG2:0] infl_next = (take_infl && !rsp_read) ? infl_inc :
	                                (!take_infl && rsp_read) ? infl_dec : inflight;
	wire [3:0] epoch_inc;
	wire [15:0] rq_row_inc, calc_h_last, calc_h_m2, rq_k_inc;
	(* max_fanout = 16 *) reg [15:0] f_nl_last, f_nl_m2, f_h_last, f_h_m2;
	wire [AW-1:0] rq_cur_inc;
	wire [13:0] rq_co_unused;

	assign rq_co_unused[0] = 1'b0;
	am_add #(.W(LFIFO_LOG2 + 1)) a_infl (.a(inflight), .b({(LFIFO_LOG2 + 1){1'b0}}), .ci(1'b1), .s(infl_inc),
		.co(rq_co_unused[1]));
	am_sub #(.W(LFIFO_LOG2 + 1)) s_infl (.a(inflight), .b({(LFIFO_LOG2 + 1){1'b0}}), .bi(1'b1), .d(infl_dec),
		.bo(rq_co_unused[2]));
	am_add #(.W(4)) a_epoch (.a(epoch), .b(4'd0), .ci(1'b1), .s(epoch_inc), .co(rq_co_unused[3]));
	assign rq_co_unused[4] = 1'b0;

`ifdef SYNTH_ASIC
	/* 4-bit split increments for rq_k, and 8+8 for rq_row and fb_size.h (<= 10 cmos3). */
	wire [7:0] rq_row_lo_inc, rq_row_hi_inc;
	wire [3:0] rq_k_n0, rq_k_n1, rq_k_n2, rq_k_n3;
	wire [7:0] rk_co_unused;
	am_add #(.W(8)) a_row_lo (.a(rq_row[7:0]),  .b(8'd0), .ci(1'b1), .s(rq_row_lo_inc), .co(rq_co_unused[5]));
	am_add #(.W(8)) a_row_hi (.a(rq_row[15:8]), .b(8'd0), .ci(1'b1), .s(rq_row_hi_inc), .co(rq_co_unused[6]));
	assign rq_row_inc = {(&rq_row[7:0]) ? rq_row_hi_inc : rq_row[15:8], rq_row_lo_inc};

	am_add #(.W(4)) a_k_n0 (.a(rq_k[3:0]),   .b(4'd0), .ci(1'b1), .s(rq_k_n0), .co(rk_co_unused[0]));
	am_add #(.W(4)) a_k_n1 (.a(rq_k[7:4]),   .b(4'd0), .ci(1'b1), .s(rq_k_n1), .co(rk_co_unused[1]));
	am_add #(.W(4)) a_k_n2 (.a(rq_k[11:8]),  .b(4'd0), .ci(1'b1), .s(rq_k_n2), .co(rk_co_unused[2]));
	am_add #(.W(4)) a_k_n3 (.a(rq_k[15:12]), .b(4'd0), .ci(1'b1), .s(rq_k_n3), .co(rk_co_unused[3]));
	wire k_g0 = &rq_k[3:0];
	wire k_g1 = &rq_k[7:4];
	wire k_g2 = &rq_k[11:8];
	wire k_all1_0 = k_g0;
	wire k_all1_1 = k_g0 && k_g1;
	wire k_all1_2 = k_all1_1 && k_g2;
	assign rq_k_inc = {
		k_all1_2 ? rq_k_n3 : rq_k[15:12],
		k_all1_1 ? rq_k_n2 : rq_k[11:8],
		k_all1_0 ? rq_k_n1 : rq_k[7:4],
		rq_k_n0
	};

	/* Balanced 4-bit nibble subtractors for fb_size.h -> calc_h_last / calc_h_m2 (<= 9 cmos3). */
	wire [3:0] ch_1_n0, ch_1_n1, ch_1_n2, ch_1_n3, ch_2_n0, ch_2_n1, ch_2_n2, ch_2_n3;
	am_sub #(.W(4)) s_h_n0  (.a(fb_h_c0[3:0]),   .b(4'd1), .bi(1'b0), .d(ch_1_n0), .bo(rk_co_unused[4]));
	am_sub #(.W(4)) s_h_n1  (.a(fb_h_c0[7:4]),   .b(4'd1), .bi(1'b0), .d(ch_1_n1), .bo(rk_co_unused[5]));
	am_sub #(.W(4)) s_h_n2  (.a(fb_h_c0[11:8]),  .b(4'd1), .bi(1'b0), .d(ch_1_n2), .bo(rk_co_unused[6]));
	am_sub #(.W(4)) s_h_n3  (.a(fb_h_c0[15:12]), .b(4'd1), .bi(1'b0), .d(ch_1_n3), .bo(rk_co_unused[7]));
	wire ch1_b0 = ~(|fb_h_c0[3:0]);
	wire ch1_b1 = ch1_b0 && ~(|fb_h_c0[7:4]);
	wire ch1_b2 = ch1_b1 && ~(|fb_h_c0[11:8]);
	assign calc_h_last = {ch1_b2 ? ch_1_n3 : fb_h_c0[15:12], ch1_b1 ? ch_1_n2 : fb_h_c0[11:8], ch1_b0 ? ch_1_n1 : fb_h_c0[7:4], ch_1_n0};

	wire [3:0] chm_co_un;
	am_sub #(.W(4)) s_hm_n0 (.a(fb_h_c1[3:0]),   .b(4'd2), .bi(1'b0), .d(ch_2_n0), .bo(chm_co_un[0]));
	am_sub #(.W(4)) s_hm_n1 (.a(fb_h_c1[7:4]),   .b(4'd1), .bi(1'b0), .d(ch_2_n1), .bo(chm_co_un[1]));
	am_sub #(.W(4)) s_hm_n2 (.a(fb_h_c1[11:8]),  .b(4'd1), .bi(1'b0), .d(ch_2_n2), .bo(chm_co_un[2]));
	am_sub #(.W(4)) s_hm_n3 (.a(fb_h_c1[15:12]), .b(4'd1), .bi(1'b0), .d(ch_2_n3), .bo(chm_co_un[3]));
	wire ch2_b0 = ~(|fb_h_c1[3:1]);
	wire ch2_b1 = ch2_b0 && ~(|fb_h_c1[7:4]);
	wire ch2_b2 = ch2_b1 && ~(|fb_h_c1[11:8]);
	assign calc_h_m2 = {ch2_b2 ? ch_2_n3 : fb_h_c1[15:12], ch2_b1 ? ch_2_n2 : fb_h_c1[11:8], ch2_b0 ? ch_2_n1 : fb_h_c1[7:4], ch_2_n0};
	assign rq_co_unused[7]  = (&rk_co_unused) ^ (&chm_co_un);
	assign rq_co_unused[8]  = 1'b0;
	assign rq_co_unused[9]  = 1'b0;
	assign rq_co_unused[10] = 1'b0;

	/* 2-cycle pipelined 8-bit carry-select adder for rq_next_row_r = rq_row_line + f_stride. */
	wire [7:0] nrow_b0, nrow_b1_0, nrow_b1_1, nrow_b2;
	wire [AW-25:0] nrow_b3_0, nrow_b3_1;
	wire nrow_c8, nrow_c16_0, nrow_c16_1, nrow_c24;
	wire [1:0] nrow_hi_co_un;
	am_add #(.W(8)) a_nrow_0   (.a(rq_row_line[7:0]),     .b(f_stride[7:0]),     .ci(1'b0),       .s(nrow_b0),   .co(nrow_c8));
	am_add #(.W(8)) a_nrow_1_0 (.a(rq_row_line[15:8]),    .b(f_stride[15:8]),    .ci(1'b0),       .s(nrow_b1_0), .co(nrow_c16_0));
	am_add #(.W(8)) a_nrow_1_1 (.a(rq_row_line[15:8]),    .b(f_stride[15:8]),    .ci(1'b1),       .s(nrow_b1_1), .co(nrow_c16_1));
	am_add #(.W(8)) a_nrow_2   (.a(rq_row_line[23:16]),   .b(f_stride[23:16]),   .ci(nrow_c16_q), .s(nrow_b2),   .co(nrow_c24));
	am_add #(.W(AW-24)) a_nrow_3_0 (.a(rq_row_line[AW-1:24]), .b(f_stride[AW-1:24]), .ci(1'b0), .s(nrow_b3_0), .co(nrow_hi_co_un[0]));
	am_add #(.W(AW-24)) a_nrow_3_1 (.a(rq_row_line[AW-1:24]), .b(f_stride[AW-1:24]), .ci(1'b1), .s(nrow_b3_1), .co(nrow_hi_co_un[1]));
	wire [15:0] nrow_lo_w = {nrow_c8 ? nrow_b1_1 : nrow_b1_0, nrow_b0};
	wire nrow_c16_w = nrow_c8 ? nrow_c16_1 : nrow_c16_0;
	wire [AW-1:16] nrow_hi_w = {nrow_c24 ? nrow_b3_1 : nrow_b3_0, nrow_b2};

	/* 8-bit chunks for AW-bit rq_cur increment (<= 10 cmos3). */
	wire [7:0] cur_b0, cur_b1, cur_b2;
	wire [AW-25:0] cur_b3;
	wire [2:0] cur_co_unused;
	am_add #(.W(8))     a_cur_0 (.a(rq_cur[7:0]),     .b(8'd0),            .ci(1'b1), .s(cur_b0), .co(cur_co_unused[0]));
	am_add #(.W(8))     a_cur_1 (.a(rq_cur[15:8]),    .b(8'd0),            .ci(1'b1), .s(cur_b1), .co(cur_co_unused[1]));
	am_add #(.W(8))     a_cur_2 (.a(rq_cur[23:16]),   .b(8'd0),            .ci(1'b1), .s(cur_b2), .co(cur_co_unused[2]));
	am_add #(.W(AW-24)) a_cur_3 (.a(rq_cur[AW-1:24]), .b({(AW-24){1'b0}}), .ci(1'b1), .s(cur_b3), .co(rq_co_unused[11]));
	wire cur_g0 = &rq_cur[7:0];
	wire cur_g1 = &rq_cur[15:8];
	wire cur_g2 = &rq_cur[23:16];
	wire cur_a1 = cur_g0 && cur_g1;
	wire cur_a2 = cur_a1 && cur_g2;
	assign rq_cur_inc = {
		cur_a2 ? cur_b3 : rq_cur[AW-1:24],
		cur_a1 ? cur_b2 : rq_cur[23:16],
		cur_g0 ? cur_b1 : rq_cur[15:8],
		cur_b0
	};
	assign rq_co_unused[12] = &cur_co_unused;
	assign rq_co_unused[13] = &nrow_hi_co_un;
`else
	wire [AW-1:0] nrow_sum_w;
	am_add #(.W(16)) a_row (.a(rq_row),   .b(16'd0),        .ci(1'b1), .s(rq_row_inc),  .co(rq_co_unused[5]));
	am_add #(.W(16)) a_k   (.a(rq_k),     .b(16'd0),        .ci(1'b1), .s(rq_k_inc),    .co(rq_co_unused[6]));
	am_sub #(.W(16)) s_h   (.a(fb_h_c0),  .b(16'd1),        .bi(1'b0), .d(calc_h_last), .bo(rq_co_unused[7]));
	am_sub #(.W(16)) s_hm  (.a(fb_h_c1),  .b(16'd2),        .bi(1'b0), .d(calc_h_m2),   .bo(rq_co_unused[8]));
	am_add #(.W(AW)) a_cur (.a(rq_cur),   .b({AW{1'b0}}),    .ci(1'b1), .s(rq_cur_inc),  .co(rq_co_unused[9]));
	am_add #(.W(AW)) a_nr  (.a(rq_row_line), .b(f_stride),  .ci(1'b0), .s(nrow_sum_w),  .co(rq_co_unused[10]));
	assign rq_co_unused[13:11] = {3{nrow_c16_q}};
`endif

	wire k_is_nl = k_is_nl_r;
`ifdef SYNTH_KINTEX7
	wire k_is_nl_a0, k_is_nl_a1, k_is_nl_b0, k_is_nl_b1;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_knl_a0 (.I0(k_is_nl), .O(k_is_nl_a0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_knl_a1 (.I0(k_is_nl), .O(k_is_nl_a1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_knl_b0 (.I0(k_is_nl), .O(k_is_nl_b0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_knl_b1 (.I0(k_is_nl), .O(k_is_nl_b1));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire k_is_nl_a0, k_is_nl_a1, k_is_nl_b0, k_is_nl_b1;
	assign k_is_nl_a0 = k_is_nl;
	assign k_is_nl_a1 = k_is_nl;
	assign k_is_nl_b0 = k_is_nl;
	assign k_is_nl_b1 = k_is_nl;
`else
	wire k_is_nl_a0 = k_is_nl;
	wire k_is_nl_a1 = k_is_nl;
	wire k_is_nl_b0 = k_is_nl;
	wire k_is_nl_b1 = k_is_nl;
`endif

`ifdef SYNTH_KINTEX7
	wire issue_step_rql, issue_step_rqh, issue_step_curl, issue_step_curh, issue_step_rowl, issue_step_rowh;
	wire issue_step_ctla, issue_step_ctlk, issue_step_ctlr, issue_step_nr;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_rql  (.I0(issue_step), .O(issue_step_rql));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_rqh  (.I0(issue_step), .O(issue_step_rqh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_curl (.I0(issue_step), .O(issue_step_curl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_curh (.I0(issue_step), .O(issue_step_curh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_rowl (.I0(issue_step), .O(issue_step_rowl));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_rowh (.I0(issue_step), .O(issue_step_rowh));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_ctla (.I0(issue_step), .O(issue_step_ctla));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_ctlk (.I0(issue_step), .O(issue_step_ctlk));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_ctlr (.I0(issue_step), .O(issue_step_ctlr));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_is_nr   (.I0(issue_step), .O(issue_step_nr));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire issue_step_rql, issue_step_rqh, issue_step_curl, issue_step_curh, issue_step_rowl, issue_step_rowh;
	wire issue_step_ctla, issue_step_ctlk, issue_step_ctlr, issue_step_nr;
	assign issue_step_rql = issue_step;
	assign issue_step_rqh = issue_step;
	assign issue_step_curl = issue_step;
	assign issue_step_curh = issue_step;
	assign issue_step_rowl = issue_step;
	assign issue_step_rowh = issue_step;
	assign issue_step_ctla = issue_step;
	assign issue_step_ctlk = issue_step;
	assign issue_step_ctlr = issue_step;
	assign issue_step_nr = issue_step;
`else
	wire issue_step_rql = issue_step, issue_step_rqh = issue_step;
	wire issue_step_curl = issue_step, issue_step_curh = issue_step;
	wire issue_step_rowl = issue_step, issue_step_rowh = issue_step;
	wire issue_step_ctla = issue_step;
	wire issue_step_ctlk = issue_step;
	wire issue_step_ctlr = issue_step;
	wire issue_step_nr   = issue_step;
`endif

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rq_k <= 16'd0;
			k_is_nl_r <= 1'b0;
		end else if (clr_k || start_k) begin
			rq_k <= 16'd0;
			k_is_nl_r <= !clr_k && nl_is_1;
		end else if (issue_step_ctlk) begin
			if (k_is_nl_a0) begin
				rq_k <= 16'd0;
				k_is_nl_r <= ~(|f_nl_last);
			end else begin
				rq_k <= rq_k_inc;
				k_is_nl_r <= (rq_k == f_nl_m2);
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rq_row <= 16'd0;
			row_is_last_r <= 1'b0;
		end else if (clr_r) begin
			rq_row <= 16'd0;
			row_is_last_r <= 1'b0;
		end else if (start_r) begin
			rq_row <= 16'd0;
			row_is_last_r <= ~(|(fb_h_c0 ^ 16'd1));
		end else if (issue_step_ctlr && k_is_nl_b0) begin
			rq_row <= rq_row_inc;
			row_is_last_r <= (rq_row == f_h_m2);
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			enable_q     <= 1'b0;
			enable_q_ab0 <= 1'b0;
			enable_q_ab1 <= 1'b0;
		end else if (clr_eq) begin
			enable_q     <= 1'b0;
			enable_q_ab0 <= 1'b0;
			enable_q_ab1 <= 1'b0;
		end else begin
			enable_q     <= enable;
			enable_q_ab0 <= enable;
			enable_q_ab1 <= enable;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			used        <= {(LFIFO_LOG2 + 1){1'b0}};
			used_ge31_r <= 1'b0;
			used_ge32_r <= 1'b0;
		end else if (clr_used || abort_used) begin
			used        <= {(LFIFO_LOG2 + 1){1'b0}};
			used_ge31_r <= 1'b0;
			used_ge32_r <= 1'b0;
		end else begin
			case ({take_used && (tag_q == epoch), l_pop})
			2'b10: begin
				used <= used_inc;
				used_ge31_r <= used_ge31_r || (used[LFIFO_LOG2-1:0] == LFIFO_M2_LO);
				used_ge32_r <= used_ge32_r || (used[LFIFO_LOG2-1:0] == LFIFO_M1_LO);
			end
			2'b01: begin
				used <= used_dec;
				used_ge31_r <= used_ge32_r;
				used_ge32_r <= 1'b0;
			end
			default: ;
			endcase
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_nl_last <= 16'd0;
		else if (start_fnl)
			f_nl_last <= calc_nl_last;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_nl_m2 <= 16'd0;
		else if (start_fnm)
			f_nl_m2 <= calc_nl_m2;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_h_last <= 16'd0;
		else if (start_fhl)
			f_h_last <= calc_h_last;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_h_m2 <= 16'd0;
		else if (start_fhm)
			f_h_m2 <= calc_h_m2;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			epoch <= 4'd0;
			tag_q <= 4'd0;
			rq_active <= 1'b0;
			req_valid <= 1'b0;
			inflight <= {(LFIFO_LOG2 + 1){1'b0}};
			f_nl <= 16'd0;
			f_16 <= 1'b0;
			f_16_lo0 <= 1'b0;
			f_16_lo1 <= 1'b0;
			f_16_hi0 <= 1'b0;
			f_16_hi1 <= 1'b0;
		end else if (clr_ctl0) begin
			epoch <= epoch_inc;
			if (!req_valid_ctl || take_ctl) begin
				tag_q <= 4'd0;
				req_valid <= 1'b0;
			end
			rq_active <= 1'b0;
			inflight <= infl_next;
			f_nl <= 16'd0;
			f_16 <= 1'b0;
			f_16_lo0 <= 1'b0;
			f_16_lo1 <= 1'b0;
			f_16_hi0 <= 1'b0;
			f_16_hi1 <= 1'b0;
		end else begin
			inflight <= infl_next;
			if (abort_ctl) begin
				epoch <= epoch_inc;
				rq_active <= 1'b0;
			end
			if (start_ctl0) begin
				if (take_ctl)
					req_valid <= 1'b0;
				rq_active <= fb_w_ff != 16'd0 && fb_h_c0 != 16'd0;
				f_nl <= 16'd0;
				f_16 <= fmt_is16_c;
				f_16_lo0 <= fmt_is16_c;
				f_16_lo1 <= fmt_is16_c;
				f_16_hi0 <= fmt_is16_c;
				f_16_hi1 <= fmt_is16_c;
			end else if (issue_step_ctla) begin
				/* One line of the frame. */
				req_valid <= 1'b1;
				tag_q <= epoch;
				if (k_is_nl_b0 && row_is_last_r)
					rq_active <= 1'b0;
			end else if (take_ctl) begin
				req_valid <= 1'b0;
			end
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			f_w <= 16'd0;
			f_h <= 16'd0;
		end else if (clr_ctl1) begin
			f_w <= 16'd0;
			f_h <= 16'd0;
		end else if (start_ctl1) begin
			f_w <= fb_w_ff;
			f_h <= fb_h_c0;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			req_line_lo <= 14'd0;
		else if (clr_rqline_lo)
			req_line_lo <= 14'd0;
		else if (issue_step_rql)
			req_line_lo <= rq_cur_lo;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			req_line_hi <= {(AW-14){1'b0}};
		else if (clr_rqline_hi)
			req_line_hi <= {(AW-14){1'b0}};
		else if (issue_step_rqh)
			req_line_hi <= rq_cur_hi;
	end

	wire [63:0] fb_base_ext = {32'd0, fb_base};
	wire [63:0] fb_stride_ext = {32'd0, fb_stride};
	wire [AW-1:0] rq_cur_src_lo = start_curl ? fb_base_ext[LINE_SHIFT +: AW] : rq_next_row_r;
	wire [AW-1:0] rq_cur_src_hi = start_curh ? fb_base_ext[LINE_SHIFT +: AW] : rq_next_row_r;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_cur_lo <= 14'd0;
		else if (clr_curl)
			rq_cur_lo <= 14'd0;
		else if (start_curl || issue_step_curl)
			rq_cur_lo <= (start_curl || k_is_nl_a1) ? rq_cur_src_lo[13:0] : rq_cur_inc[13:0];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_cur_hi <= {(AW-14){1'b0}};
		else if (clr_curh)
			rq_cur_hi <= {(AW-14){1'b0}};
		else if (start_curh || issue_step_curh)
			rq_cur_hi <= (start_curh || k_is_nl_b1) ? rq_cur_src_hi[AW-1:14] : rq_cur_inc[AW-1:14];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_row_line_lo <= 16'd0;
		else if (clr_rowl)
			rq_row_line_lo <= 16'd0;
		else if (start_rowl)
			rq_row_line_lo <= fb_base[LINE_SHIFT +: 16];
		else if (issue_step_rowl && k_is_nl_a1)
			rq_row_line_lo <= rq_next_row_lo_r;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_row_line_hi <= {(AW-16){1'b0}};
		else if (clr_rowh)
			rq_row_line_hi <= {(AW-16){1'b0}};
		else if (start_rowh)
			rq_row_line_hi <= fb_base_ext[LINE_SHIFT + 16 +: AW - 16];
		else if (issue_step_rowh && k_is_nl_b1)
			rq_row_line_hi <= rq_next_row_hi;
	end

`ifdef SYNTH_ASIC
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			nrow_st          <= 2'd0;
			nrow_c16_q       <= 1'b0;
			nrow_rdy_r       <= 1'b0;
			nrow_rdy_nr      <= 1'b0;
			nrow_rdy_nr2     <= 1'b0;
			nrow_rdy_nhi     <= 1'b0;
			rq_next_row_lo   <= 16'd0;
			rq_next_row_lo_r <= 16'd0;
			rq_next_row_hi   <= {(AW-16){1'b0}};
		end else if (clr_nr || start_nr || (issue_step_nr && k_is_nl_a1)) begin
			nrow_st      <= 2'd0;
			nrow_c16_q   <= 1'b0;
			nrow_rdy_r   <= 1'b0;
			nrow_rdy_nr  <= 1'b0;
			nrow_rdy_nr2 <= 1'b0;
			nrow_rdy_nhi <= 1'b0;
		end else if (nrow_st == 2'd0) begin
			rq_next_row_lo   <= nrow_lo_w;
			rq_next_row_lo_r <= nrow_lo_w;
			nrow_c16_q       <= nrow_c16_w;
			nrow_st          <= 2'd1;
		end else if (nrow_st == 2'd1) begin
			rq_next_row_hi <= nrow_hi_w;
			nrow_st        <= 2'd2;
			nrow_rdy_r     <= 1'b1;
			nrow_rdy_nr    <= 1'b1;
			nrow_rdy_nr2   <= 1'b1;
			nrow_rdy_nhi   <= 1'b1;
		end
	end
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			nrow_st      <= 2'd0;
			nrow_c16_q   <= 1'b0;
			nrow_rdy_r   <= 1'b0;
			nrow_rdy_nr  <= 1'b0;
			nrow_rdy_nr2 <= 1'b0;
			nrow_rdy_nhi <= 1'b0;
		end else if (clr_nr || start_nr || (issue_step_nr && k_is_nl_a1)) begin
			nrow_st      <= 2'd0;
			nrow_c16_q   <= 1'b0;
			nrow_rdy_r   <= 1'b0;
			nrow_rdy_nr  <= 1'b0;
			nrow_rdy_nr2 <= 1'b0;
			nrow_rdy_nhi <= 1'b0;
		end else if (!nrow_rdy_r) begin
			nrow_rdy_r   <= 1'b1;
			nrow_rdy_nr  <= 1'b1;
			nrow_rdy_nr2 <= 1'b1;
			nrow_rdy_nhi <= 1'b1;
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_next_row_lo <= 16'd0;
		else if (!nrow_rdy_nr)
			rq_next_row_lo <= nrow_sum_w[15:0];
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_next_row_lo_r <= 16'd0;
		else if (!nrow_rdy_nr2)
			rq_next_row_lo_r <= nrow_sum_w[15:0];
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rq_next_row_hi <= {(AW-16){1'b0}};
		else if (!nrow_rdy_nhi)
			rq_next_row_hi <= nrow_sum_w[AW-1:16];
	end
`endif

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_stride_lo <= 14'd0;
		else if (clr_strl)
			f_stride_lo <= 14'd0;
		else if (start_strl)
			f_stride_lo <= fb_stride[LINE_SHIFT +: 14];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_stride_hi <= {(AW-14){1'b0}};
		else if (clr_strh)
			f_stride_hi <= {(AW-14){1'b0}};
		else if (start_strh)
			f_stride_hi <= fb_stride_ext[LINE_SHIFT + 14 +: AW - 14];
	end

	localparam NUM_L_SLICES = LW / 16;
	localparam NUM_L_GROUPS = 4;
	localparam SLICES_PER_GROUP = NUM_L_SLICES / NUM_L_GROUPS;

	/* Sliced line FIFO (Rule 2 fanout isolation for 512-bit width) */
	wire l_valid;
	wire [LW-1:0] l_data;
	wire l_pop;
	wire l_in_ready = 1'b1;	/* always 1 when a response comes: the requests wait for room */
	wire keep_in = rsp_read && rsp_tag[3:0] == epoch && !abort_rsp_in;
	(* keep = "true", equivalent_register_removal = "no" *) reg rsp_keep_ctl;
	(* keep = "true", equivalent_register_removal = "no" *) reg [NUM_L_SLICES-1:0] rsp_keep_grp;
	(* keep = "true", equivalent_register_removal = "no" *) reg [NUM_L_SLICES-1:0] rsp_err_grp;
	reg rsp_err_ctl;
	reg [LW-1:0] rsp_rdata_q;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rsp_keep_ctl <= 1'b0;
			rsp_err_ctl  <= 1'b0;
		end else if (abort_rsp_in) begin
			rsp_keep_ctl <= 1'b0;
			rsp_err_ctl  <= 1'b0;
		end else begin
			rsp_keep_ctl <= keep_in;
			rsp_err_ctl  <= keep_in && rsp_error;
		end
	end
	localparam integer GRP_LO = (NUM_L_SLICES > 16) ? 16 : NUM_L_SLICES;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rsp_keep_grp[GRP_LO-1:0] <= {GRP_LO{1'b0}};
		else if (abort_rsp_k0)
			rsp_keep_grp[GRP_LO-1:0] <= {GRP_LO{1'b0}};
		else
			rsp_keep_grp[GRP_LO-1:0] <= {GRP_LO{keep_in}};
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			rsp_err_grp[GRP_LO-1:0] <= {GRP_LO{1'b0}};
		else if (abort_rsp_e0)
			rsp_err_grp[GRP_LO-1:0] <= {GRP_LO{1'b0}};
		else
			rsp_err_grp[GRP_LO-1:0] <= {GRP_LO{keep_in && rsp_error}};
	end
	generate
	if (NUM_L_SLICES > 16) begin : g_rsp_hi
		always @(posedge clk or negedge rst_n) begin
			if (!rst_n)
				rsp_keep_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){1'b0}};
			else if (abort_rsp_k1)
				rsp_keep_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){1'b0}};
			else
				rsp_keep_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){keep_in}};
		end
		always @(posedge clk or negedge rst_n) begin
			if (!rst_n)
				rsp_err_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){1'b0}};
			else if (abort_rsp_e1)
				rsp_err_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){1'b0}};
			else
				rsp_err_grp[NUM_L_SLICES-1:16] <= {(NUM_L_SLICES-16){keep_in && rsp_error}};
		end
	end
	endgenerate
	always @(posedge clk) begin
`ifdef SYNTH_ASIC
		rsp_rdata_q <= (keep_in && rsp_error) ? {LW{1'b0}} : rsp_rdata;
`else
		rsp_rdata_q <= rsp_rdata;
`endif
	end
	wire keep = rsp_keep_ctl;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			err <= 1'b0;
		else if (clr_err)
			err <= 1'b0;
		else
			err <= keep_in && rsp_error;
	end

	reg [LFIFO_LOG2:0] l_wp, l_rp, l_rp_p1;
	wire [LFIFO_LOG2:0] l_wp_next, l_rp_p1_next;
	wire [1:0] l_fifo_co_unused;
	am_add #(.W(LFIFO_LOG2 + 1)) a_lwp (.a(l_wp),    .b({(LFIFO_LOG2 + 1){1'b0}}), .ci(1'b1), .s(l_wp_next),    .co(l_fifo_co_unused[0]));
	am_add #(.W(LFIFO_LOG2 + 1)) a_lrp (.a(l_rp_p1), .b({(LFIFO_LOG2 + 1){1'b0}}), .ci(1'b1), .s(l_rp_p1_next), .co(l_fifo_co_unused[1]));

	reg l_valid_r;
	wire l_will_empty = (l_rp_p1 == l_wp);
	assign l_valid = l_valid_r;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			l_wp <= {(LFIFO_LOG2 + 1){1'b0}};
			l_rp <= {(LFIFO_LOG2 + 1){1'b0}};
		end else if (abort_lfifo0) begin
			l_wp <= {(LFIFO_LOG2 + 1){1'b0}};
			l_rp <= {(LFIFO_LOG2 + 1){1'b0}};
		end else begin
			if (keep)
				l_wp <= l_wp_next;
			if (l_pop)
				l_rp <= l_rp_p1;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			l_rp_p1 <= {{LFIFO_LOG2{1'b0}}, 1'b1};
			l_valid_r <= 1'b0;
		end else if (abort_lfifo1) begin
			l_rp_p1 <= {{LFIFO_LOG2{1'b0}}, 1'b1};
			l_valid_r <= 1'b0;
		end else begin
			if (l_pop)
				l_rp_p1 <= l_rp_p1_next;
			case ({keep, l_pop})
			2'b10: l_valid_r <= 1'b1;
			2'b01: l_valid_r <= !l_will_empty;
			default: ;
			endcase
		end
	end

`ifdef SYNTH_KINTEX7
	localparam NUM_BUFS = NUM_L_SLICES * 4;
	wire [NUM_BUFS-1:0] keep_b;
	wire [NUM_BUFS-1:0] l_pop_b;
	wire [NUM_BUFS-1:0] rsp_error_b;

	wire [3:0] keep_tree;
	wire [3:0] l_pop_tree;
	wire [3:0] rsp_error_tree;
	genvar gti;
	generate
		for (gti = 0; gti < 4; gti = gti + 1) begin : g_slice_tree
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_t_k (.I0(keep), .O(keep_tree[gti]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_t_p (.I0(l_pop), .O(l_pop_tree[gti]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_t_e (.I0(rsp_err_ctl), .O(rsp_error_tree[gti]));
		end
	endgenerate

	genvar gki;
	generate
		for (gki = 0; gki < NUM_BUFS; gki = gki + 1) begin : g_slice_bufs
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_k (.I0(keep_tree[gki / 16]), .O(keep_b[gki]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_p (.I0(l_pop_tree[gki / 16]), .O(l_pop_b[gki]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_e (.I0(rsp_error_tree[gki / 16]), .O(rsp_error_b[gki]));
		end
	endgenerate

	wire [LFIFO_LOG2-1:0] l_wp_tree [0:3];
	wire [LFIFO_LOG2-1:0] l_rp_tree [0:3];
	genvar gptr;
	generate
		for (gptr = 0; gptr < 4; gptr = gptr + 1) begin : g_ptr_tree
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_wp0 (.I0(l_wp_next[0]), .O(l_wp_tree[gptr][0]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_wp1 (.I0(l_wp_next[1]), .O(l_wp_tree[gptr][1]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_wp2 (.I0(l_wp_next[2]), .O(l_wp_tree[gptr][2]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_wp3 (.I0(l_wp_next[3]), .O(l_wp_tree[gptr][3]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_wp4 (.I0(l_wp_next[4]), .O(l_wp_tree[gptr][4]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rp0 (.I0(l_rp_p1[0]),   .O(l_rp_tree[gptr][0]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rp1 (.I0(l_rp_p1[1]),   .O(l_rp_tree[gptr][1]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rp2 (.I0(l_rp_p1[2]),   .O(l_rp_tree[gptr][2]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rp3 (.I0(l_rp_p1[3]),   .O(l_rp_tree[gptr][3]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rp4 (.I0(l_rp_p1[4]),   .O(l_rp_tree[gptr][4]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam NUM_BUFS = NUM_L_SLICES * 4;
	wire [NUM_BUFS-1:0] keep_b;
	wire [NUM_BUFS-1:0] l_pop_b;
	wire [NUM_BUFS-1:0] rsp_error_b;

	wire [3:0] keep_tree;
	wire [3:0] l_pop_tree;
	wire [3:0] rsp_error_tree;
	genvar gti;
	generate
		for (gti = 0; gti < 4; gti = gti + 1) begin : g_slice_tree
			assign keep_tree[gti] = keep;
			assign l_pop_tree[gti] = l_pop;
			assign rsp_error_tree[gti] = rsp_err_ctl;
		end
	endgenerate

	genvar gki;
	generate
		for (gki = 0; gki < NUM_BUFS; gki = gki + 1) begin : g_slice_bufs
			assign keep_b[gki] = keep_tree[gki / 16];
			assign l_pop_b[gki] = l_pop_tree[gki / 16];
			assign rsp_error_b[gki] = rsp_error_tree[gki / 16];
		end
	endgenerate

	wire [LFIFO_LOG2-1:0] l_wp_tree [0:3];
	wire [LFIFO_LOG2-1:0] l_rp_tree [0:3];
	genvar gptr;
	generate
		for (gptr = 0; gptr < 4; gptr = gptr + 1) begin : g_ptr_tree
			assign l_wp_tree[gptr][0] = l_wp_next[0];
			assign l_wp_tree[gptr][1] = l_wp_next[1];
			assign l_wp_tree[gptr][2] = l_wp_next[2];
			assign l_wp_tree[gptr][3] = l_wp_next[3];
			assign l_wp_tree[gptr][4] = l_wp_next[4];
			assign l_rp_tree[gptr][0] = l_rp_p1[0];
			assign l_rp_tree[gptr][1] = l_rp_p1[1];
			assign l_rp_tree[gptr][2] = l_rp_p1[2];
			assign l_rp_tree[gptr][3] = l_rp_p1[3];
			assign l_rp_tree[gptr][4] = l_rp_p1[4];
		end
	endgenerate
`else
	(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_wp;
	(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_rp;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			s_wp <= {LFIFO_LOG2{1'b0}};
			s_rp <= {LFIFO_LOG2{1'b0}};
		end else if (abort) begin
			s_wp <= {LFIFO_LOG2{1'b0}};
			s_rp <= {LFIFO_LOG2{1'b0}};
		end else begin
			if (keep)
				s_wp <= l_wp_next[LFIFO_LOG2-1:0];
			if (l_pop)
				s_rp <= l_rp_p1[LFIFO_LOG2-1:0];
		end
	end
`endif

`ifdef SYNTH_KINTEX7
	wire [7:0] abort_lg_tree;
	genvar galg;
	generate
		for (galg = 0; galg < 8; galg = galg + 1) begin : g_ab_lg_tree
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ab_lgt (.I0(abort_t1), .O(abort_lg_tree[galg]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] abort_lg_tree;
	genvar galg;
	generate
		for (galg = 0; galg < 8; galg = galg + 1) begin : g_ab_lg_tree
			assign abort_lg_tree[galg] = abort_t1;
		end
	endgenerate
`endif

	genvar igrp;
	generate
	for (igrp = 0; igrp < NUM_L_SLICES * 4; igrp = igrp + 1) begin : g_l_grp
`ifdef SYNTH_KINTEX7
		wire abort_grp;
		(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ab_grp (.I0(abort_lg_tree[igrp / 16]), .O(abort_grp));
		(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_wp_r;
		(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_rp_r;
		always @(posedge clk or negedge rst_n) begin
			if (!rst_n) begin
				s_wp_r <= {LFIFO_LOG2{1'b0}};
				s_rp_r <= {LFIFO_LOG2{1'b0}};
			end else if (abort_grp) begin
				s_wp_r <= {LFIFO_LOG2{1'b0}};
				s_rp_r <= {LFIFO_LOG2{1'b0}};
			end else begin
				if (keep_b[igrp])
					s_wp_r <= l_wp_tree[igrp / 16];
				if (l_pop_b[igrp])
					s_rp_r <= l_rp_tree[igrp / 16];
			end
		end

		RAM32X1D u_ram0 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 0]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 0])
		);
		RAM32X1D u_ram1 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 1]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 1])
		);
		RAM32X1D u_ram2 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 2]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 2])
		);
		RAM32X1D u_ram3 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 3]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 3])
		);
`elsif SYNTH_KINTEX_ULTRASCALE
		wire abort_grp;
		assign abort_grp = abort_lg_tree[igrp / 16];
		(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_wp_r;
		(* max_fanout = 16 *) reg [LFIFO_LOG2-1:0] s_rp_r;
		always @(posedge clk or negedge rst_n) begin
			if (!rst_n) begin
				s_wp_r <= {LFIFO_LOG2{1'b0}};
				s_rp_r <= {LFIFO_LOG2{1'b0}};
			end else if (abort_grp) begin
				s_wp_r <= {LFIFO_LOG2{1'b0}};
				s_rp_r <= {LFIFO_LOG2{1'b0}};
			end else begin
				if (keep_b[igrp])
					s_wp_r <= l_wp_tree[igrp / 16];
				if (l_pop_b[igrp])
					s_rp_r <= l_rp_tree[igrp / 16];
			end
		end

		RAM32X1D u_ram0 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 0]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 0])
		);
		RAM32X1D u_ram1 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 1]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 1])
		);
		RAM32X1D u_ram2 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 2]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 2])
		);
		RAM32X1D u_ram3 (
			.WCLK(clk), .WE(keep_b[igrp]),
			.D(rsp_error_b[igrp] ? 1'b0 : rsp_rdata_q[igrp*4 + 3]),
			.A0(s_wp_r[0]), .A1(s_wp_r[1]), .A2(s_wp_r[2]), .A3(s_wp_r[3]), .A4(s_wp_r[4]),
			.DPRA0(s_rp_r[0]), .DPRA1(s_rp_r[1]), .DPRA2(s_rp_r[2]), .DPRA3(s_rp_r[3]), .DPRA4(s_rp_r[4]),
			.SPO(), .DPO(l_data[igrp*4 + 3])
		);
`else
		reg [3:0] grp_mem [0:LFIFO-1];
		always @(posedge clk) begin
			if (rsp_keep_grp[igrp / 4])
`ifdef SYNTH_ASIC
				grp_mem[s_wp] <= rsp_rdata_q[igrp*4 +: 4];
`else
				grp_mem[s_wp] <= rsp_err_grp[igrp / 4] ? 4'd0 : rsp_rdata_q[igrp*4 +: 4];
`endif
		end
		assign l_data[igrp*4 +: 4] = grp_mem[s_rp];
`endif
	end
	endgenerate

	wire [LFIFO_LOG2:0] lcount = {(LFIFO_LOG2 + 1){1'b0}};

	/* The unpacker. */
	reg up_active;
	reg [15:0] up_row, up_x;
	(* max_fanout = 16 *) reg [5:0] up_idx;
	wire [5:0] ppl = f_16 ? PPL16 : PPL32;
	wire push = px_valid && px_ready;
	wire [15:0] calc_w_last, calc_w_m2;
	reg [15:0] f_w_last, f_w_m2;
	wire [5:0] ppl_last = f_16 ? PPL16_LAST : PPL32_LAST;
	wire [5:0] ppl_m2   = f_16 ? PPL16_M2   : PPL32_M2;
	wire [7:0] up_co_unused;
`ifdef SYNTH_ASIC
	wire [3:0] wl_n0, wl_n1, wl_n2, wl_n3, wm_n0, wm_n1, wm_n2, wm_n3;
	am_sub #(.W(4)) s_wl_n0 (.a(fb_w_c0[3:0]),   .b(4'd1), .bi(1'b0), .d(wl_n0), .bo(up_co_unused[0]));
	am_sub #(.W(4)) s_wl_n1 (.a(fb_w_c0[7:4]),   .b(4'd1), .bi(1'b0), .d(wl_n1), .bo(up_co_unused[1]));
	am_sub #(.W(4)) s_wl_n2 (.a(fb_w_c0[11:8]),  .b(4'd1), .bi(1'b0), .d(wl_n2), .bo(up_co_unused[2]));
	am_sub #(.W(4)) s_wl_n3 (.a(fb_w_c0[15:12]), .b(4'd1), .bi(1'b0), .d(wl_n3), .bo(up_co_unused[3]));
	wire wl_b0 = ~(|fb_w_c0[3:0]);
	wire wl_b1 = wl_b0 && ~(|fb_w_c0[7:4]);
	wire wl_b2 = wl_b1 && ~(|fb_w_c0[11:8]);
	assign calc_w_last = {wl_b2 ? wl_n3 : fb_w_c0[15:12], wl_b1 ? wl_n2 : fb_w_c0[11:8], wl_b0 ? wl_n1 : fb_w_c0[7:4], wl_n0};

	am_sub #(.W(4)) s_wm_n0 (.a(fb_w_c1[3:0]),   .b(4'd2), .bi(1'b0), .d(wm_n0), .bo(up_co_unused[4]));
	am_sub #(.W(4)) s_wm_n1 (.a(fb_w_c1[7:4]),   .b(4'd1), .bi(1'b0), .d(wm_n1), .bo(up_co_unused[5]));
	am_sub #(.W(4)) s_wm_n2 (.a(fb_w_c1[11:8]),  .b(4'd1), .bi(1'b0), .d(wm_n2), .bo(up_co_unused[6]));
	am_sub #(.W(4)) s_wm_n3 (.a(fb_w_c1[15:12]), .b(4'd1), .bi(1'b0), .d(wm_n3), .bo(up_co_unused[7]));
	wire wm_b0 = ~(|fb_w_c1[3:1]);
	wire wm_b1 = wm_b0 && ~(|fb_w_c1[7:4]);
	wire wm_b2 = wm_b1 && ~(|fb_w_c1[11:8]);
	assign calc_w_m2 = {wm_b2 ? wm_n3 : fb_w_c1[15:12], wm_b1 ? wm_n2 : fb_w_c1[11:8], wm_b0 ? wm_n1 : fb_w_c1[7:4], wm_n0};
`else
	am_sub #(.W(16)) s_w  (.a(fb_w_c0), .b(16'd1), .bi(1'b0), .d(calc_w_last), .bo(up_co_unused[0]));
	am_sub #(.W(16)) s_wm (.a(fb_w_c1), .b(16'd2), .bi(1'b0), .d(calc_w_m2),   .bo(up_co_unused[1]));
	assign up_co_unused[7:2] = 6'd0;
`endif

	reg row_end_r;
	wire [3:0] eq_w_m2 = {
		(up_x[15:12] == f_w_m2[15:12]),
		(up_x[11:8]  == f_w_m2[11:8]),
		(up_x[7:4]   == f_w_m2[7:4]),
		(up_x[3:0]   == f_w_m2[3:0])
	};
	wire next_is_row_end = &eq_w_m2;
	wire zero_w_last = ~(|f_w_last);
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			row_end_r <= 1'b0;
		end else if (abort_row_end) begin
			row_end_r <= ~(|(fb_w_ff ^ 16'd1));
		end else if (push) begin
			if (row_end_r)
				row_end_r <= zero_w_last;
			else
				row_end_r <= next_is_row_end;
		end
	end
	wire row_end = row_end_r;
	wire [15:0] diff_h_last = up_row ^ f_h_last;
	wire row_h_end = ~(|diff_h_last);
	reg ppl_match_r;
	wire ppl_match = ppl_match_r;

`ifdef SYNTH_KINTEX7
	wire row_end_pop, row_end_x, row_end_idx0, row_end_idx1, row_end_row, row_end_act;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rep  (.I0(row_end), .O(row_end_pop));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rex  (.I0(row_end), .O(row_end_x));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rei0 (.I0(row_end), .O(row_end_idx0));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rei1 (.I0(row_end), .O(row_end_idx1));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rer  (.I0(row_end), .O(row_end_row));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rea  (.I0(row_end), .O(row_end_act));
	wire abort_fw_l, abort_fw_m, abort_spx_l, abort_spx_h;
	wire abort_scan_l, abort_scan_h, abort_sw_l, abort_sw_h;
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_fwl  (.I0(abort_slow), .O(abort_fw_l));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_fwm  (.I0(abort_slow), .O(abort_fw_m));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_spxl (.I0(abort_slow), .O(abort_spx_l));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_spxh (.I0(abort_slow), .O(abort_spx_h));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_scl  (.I0(abort_slow), .O(abort_scan_l));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_sch  (.I0(abort_slow), .O(abort_scan_h));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_swl  (.I0(abort_slow), .O(abort_sw_l));
	(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_ab_swh  (.I0(abort_slow), .O(abort_sw_h));
	wire [LINE_SHIFT-2:0] up_idx_d0a, up_idx_d0b, up_idx_d0c, up_idx_d0d;
	wire [LINE_SHIFT-2:0] up_idx_d1a, up_idx_d1b, up_idx_d1c, up_idx_d1d;
	wire [LINE_SHIFT-2:0] up_idx_d2a, up_idx_d2b, up_idx_d3a, up_idx_d3b;
	genvar gui;
	generate
		for (gui = 0; gui < LINE_SHIFT - 1; gui = gui + 1) begin : g_ui_buf
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui0a (.I0(up_idx_m[gui]), .O(up_idx_d0a[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui0b (.I0(up_idx_m[gui]), .O(up_idx_d0b[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui0c (.I0(up_idx_m[gui]), .O(up_idx_d0c[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui0d (.I0(up_idx_m[gui]), .O(up_idx_d0d[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui1a (.I0(up_idx_m[gui]), .O(up_idx_d1a[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui1b (.I0(up_idx_m[gui]), .O(up_idx_d1b[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui1c (.I0(up_idx_m[gui]), .O(up_idx_d1c[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui1d (.I0(up_idx_m[gui]), .O(up_idx_d1d[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui2a (.I0(up_idx[gui]),   .O(up_idx_d2a[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui2b (.I0(up_idx[gui]),   .O(up_idx_d2b[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui3a (.I0(up_idx[gui]),   .O(up_idx_d3a[gui]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_ui3b (.I0(up_idx[gui]),   .O(up_idx_d3b[gui]));
		end
	endgenerate
	(* keep = "true" *) wire px_valid_k7 = up_active && l_valid && !frame_start && (!enable_q_ab0 || enable) && !clear;
	assign px_valid = px_valid_k7;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire row_end_pop, row_end_x, row_end_idx0, row_end_idx1, row_end_row, row_end_act;
	assign row_end_pop = row_end;
	assign row_end_x = row_end;
	assign row_end_idx0 = row_end;
	assign row_end_idx1 = row_end;
	assign row_end_row = row_end;
	assign row_end_act = row_end;
	wire abort_fw_l, abort_fw_m, abort_spx_l, abort_spx_h;
	wire abort_scan_l, abort_scan_h, abort_sw_l, abort_sw_h;
	assign abort_fw_l = abort_slow;
	assign abort_fw_m = abort_slow;
	assign abort_spx_l = abort_slow;
	assign abort_spx_h = abort_slow;
	assign abort_scan_l = abort_slow;
	assign abort_scan_h = abort_slow;
	assign abort_sw_l = abort_slow;
	assign abort_sw_h = abort_slow;
	wire [LINE_SHIFT-2:0] up_idx_d0a, up_idx_d0b, up_idx_d0c, up_idx_d0d;
	wire [LINE_SHIFT-2:0] up_idx_d1a, up_idx_d1b, up_idx_d1c, up_idx_d1d;
	wire [LINE_SHIFT-2:0] up_idx_d2a, up_idx_d2b, up_idx_d3a, up_idx_d3b;
	genvar gui;
	generate
		for (gui = 0; gui < LINE_SHIFT - 1; gui = gui + 1) begin : g_ui_buf
			assign up_idx_d0a[gui] = up_idx_m[gui];
			assign up_idx_d0b[gui] = up_idx_m[gui];
			assign up_idx_d0c[gui] = up_idx_m[gui];
			assign up_idx_d0d[gui] = up_idx_m[gui];
			assign up_idx_d1a[gui] = up_idx_m[gui];
			assign up_idx_d1b[gui] = up_idx_m[gui];
			assign up_idx_d1c[gui] = up_idx_m[gui];
			assign up_idx_d1d[gui] = up_idx_m[gui];
			assign up_idx_d2a[gui] = up_idx[gui];
			assign up_idx_d2b[gui] = up_idx[gui];
			assign up_idx_d3a[gui] = up_idx[gui];
			assign up_idx_d3b[gui] = up_idx[gui];
		end
	endgenerate
	(* keep = "true" *) wire px_valid_k7 = up_active && l_valid && !frame_start && (!enable_q_ab0 || enable) && !clear;
	assign px_valid = px_valid_k7;
`else
	wire row_end_pop = row_end, row_end_x = row_end;
	wire row_end_idx0 = row_end, row_end_idx1 = row_end;
	wire row_end_row = row_end, row_end_act = row_end;
	wire abort_fw_l = abort_slow, abort_fw_m = abort_slow, abort_spx_l = abort_slow, abort_spx_h = abort_slow;
	wire abort_scan_l = abort_slow, abort_scan_h = abort_slow, abort_sw_l = abort_slow, abort_sw_h = abort_slow;
	wire [LINE_SHIFT-2:0] up_idx_d0a = up_idx[LINE_SHIFT-2:0], up_idx_d0b = up_idx[LINE_SHIFT-2:0];
	wire [LINE_SHIFT-2:0] up_idx_d0c = up_idx[LINE_SHIFT-2:0], up_idx_d0d = up_idx[LINE_SHIFT-2:0];
	wire [LINE_SHIFT-2:0] up_idx_d1a = up_idx[LINE_SHIFT-2:0], up_idx_d1b = up_idx[LINE_SHIFT-2:0];
	wire [LINE_SHIFT-2:0] up_idx_d1c = up_idx[LINE_SHIFT-2:0], up_idx_d1d = up_idx[LINE_SHIFT-2:0];
	wire [LINE_SHIFT-2:0] up_idx_d2a = up_idx[LINE_SHIFT-2:0], up_idx_d2b = up_idx[LINE_SHIFT-2:0];
	wire [LINE_SHIFT-2:0] up_idx_d3a = up_idx[LINE_SHIFT-2:0], up_idx_d3b = up_idx[LINE_SHIFT-2:0];
	assign px_valid = up_active && l_valid && !abort;
`endif

	assign l_pop = push && (row_end_pop || ppl_match);
	assign px_data[1:0]   = f_16_lo0 ? l_data[{up_idx_d0a[LINE_SHIFT-2:0], 4'd0}  +: 2] : l_data[{up_idx_d0a[LINE_SHIFT-3:0], 5'd0}  +: 2];
	assign px_data[3:2]   = f_16_lo0 ? l_data[{up_idx_d0b[LINE_SHIFT-2:0], 4'd2}  +: 2] : l_data[{up_idx_d0b[LINE_SHIFT-3:0], 5'd2}  +: 2];
	assign px_data[5:4]   = f_16_lo0 ? l_data[{up_idx_d0c[LINE_SHIFT-2:0], 4'd4}  +: 2] : l_data[{up_idx_d0c[LINE_SHIFT-3:0], 5'd4}  +: 2];
	assign px_data[7:6]   = f_16_lo0 ? l_data[{up_idx_d0d[LINE_SHIFT-2:0], 4'd6}  +: 2] : l_data[{up_idx_d0d[LINE_SHIFT-3:0], 5'd6}  +: 2];
	assign px_data[9:8]   = f_16_lo1 ? l_data[{up_idx_d1a[LINE_SHIFT-2:0], 4'd8}  +: 2] : l_data[{up_idx_d1a[LINE_SHIFT-3:0], 5'd8}  +: 2];
	assign px_data[11:10] = f_16_lo1 ? l_data[{up_idx_d1b[LINE_SHIFT-2:0], 4'd10} +: 2] : l_data[{up_idx_d1b[LINE_SHIFT-3:0], 5'd10} +: 2];
	assign px_data[13:12] = f_16_lo1 ? l_data[{up_idx_d1c[LINE_SHIFT-2:0], 4'd12} +: 2] : l_data[{up_idx_d1c[LINE_SHIFT-3:0], 5'd12} +: 2];
	assign px_data[15:14] = f_16_lo1 ? l_data[{up_idx_d1d[LINE_SHIFT-2:0], 4'd14} +: 2] : l_data[{up_idx_d1d[LINE_SHIFT-3:0], 5'd14} +: 2];
	assign px_data[19:16] = f_16_hi0 ? 4'd0 : l_data[{up_idx_d2a[LINE_SHIFT-3:0], 5'd16} +: 4];
	assign px_data[23:20] = f_16_hi0 ? 4'd0 : l_data[{up_idx_d2b[LINE_SHIFT-3:0], 5'd20} +: 4];
	assign px_data[27:24] = f_16_hi1 ? 4'd0 : l_data[{up_idx_d3a[LINE_SHIFT-3:0], 5'd24} +: 4];
	assign px_data[31:28] = f_16_hi1 ? 4'd0 : l_data[{up_idx_d3b[LINE_SHIFT-3:0], 5'd28} +: 4];

	/* Pipelined pixel accumulator (Rule 1 & Rule 4) */
	reg [31:0] sum_px;
	(* keep = "true" *) reg sum_push, sum_push_lo, sum_push_hi;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum_px[15:0] <= 16'd0;
		end else if (abort_spx_l) begin
			sum_px[15:0] <= 16'd0;
		end else begin
			sum_px[15:0] <= px_data[15:0];
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum_px[31:16] <= 16'd0;
		end else if (abort_spx_h) begin
			sum_px[31:16] <= 16'd0;
		end else begin
			sum_px[31:16] <= px_data[31:16];
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum_push <= 1'b0;
			sum_push_lo <= 1'b0;
			sum_push_hi <= 1'b0;
		end else if (abort_sp) begin
			sum_push <= 1'b0;
			sum_push_lo <= 1'b0;
			sum_push_hi <= 1'b0;
		end else begin
			sum_push <= push;
			sum_push_lo <= push;
			sum_push_hi <= push;
		end
	end

	(* max_fanout = 16 *) reg [31:0] sum_acc, words_acc;
	reg sum_c16_q, words_c16_q;

	wire [31:0] sum_next, words_inc;
	wire [15:0] scan_sum_hi_p1, scan_words_hi_p1;
	wire sum_c16_w, words_c16_w;
	wire [15:0] up_row_inc, up_x_inc;
	wire [5:0] up_idx_inc;
	wire [4:0] acc_co_unused;
`ifdef SYNTH_ASIC
	/* The unpacker's sums: carry-save 8-bit halves (<= 10 cmos3). */
	wire [7:0] sum_hi_0, sum_hi_1;
	wire [7:0] sum_scan_p1_lo, sum_scan_p1_hi;
	wire sum_c8, sum_c24_0, sum_c24_1, sum_c24_w, sum_co_un0, sum_co_un1;
	wire [7:0] sum_top_2_0, sum_top_2_1;
	wire sum_top_co2_0, sum_top_co2_1;
	wire [1:0] sum_p1_co_un;
	am_add #(.W(8)) a_sum_0   (.a(sum_acc[7:0]),   .b(sum_px[7:0]),   .ci(1'b0),      .s(sum_next[7:0]), .co(sum_c8));
	am_add #(.W(8)) a_sum_1_0 (.a(sum_acc[15:8]),  .b(sum_px[15:8]),  .ci(1'b0),      .s(sum_hi_0),       .co(sum_co_un0));
	am_add #(.W(8)) a_sum_1_1 (.a(sum_acc[15:8]),  .b(sum_px[15:8]),  .ci(1'b1),      .s(sum_hi_1),       .co(sum_co_un1));
	assign sum_next[15:8] = sum_c8 ? sum_hi_1 : sum_hi_0;
	assign sum_c16_w      = sum_c8 ? sum_co_un1 : sum_co_un0;

	am_add #(.W(8)) a_sum_2   (.a(sum_acc[23:16]), .b(sum_px[23:16]), .ci(sum_c16_q), .s(sum_next[23:16]), .co(sum_c24_w));
	am_add #(.W(8)) a_sum_3_0 (.a(sum_acc[31:24]), .b(sum_px[31:24]), .ci(1'b0),      .s(sum_top_2_0),     .co(sum_top_co2_0));
	am_add #(.W(8)) a_sum_3_1 (.a(sum_acc[31:24]), .b(sum_px[31:24]), .ci(1'b1),      .s(sum_top_2_1),     .co(sum_top_co2_1));
	assign sum_next[31:24] = sum_c24_w ? sum_top_2_1 : sum_top_2_0;
	assign sum_c24_0 = sum_top_co2_0 ^ sum_top_co2_1;
	assign sum_c24_1 = 1'b0;

	am_add #(.W(8)) a_sum_p1_l (.a(sum_acc[23:16]), .b(8'd0), .ci(1'b1), .s(sum_scan_p1_lo), .co(sum_p1_co_un[0]));
	am_add #(.W(8)) a_sum_p1_h (.a(sum_acc[31:24]), .b(8'd0), .ci(1'b1), .s(sum_scan_p1_hi), .co(sum_p1_co_un[1]));
	assign scan_sum_hi_p1 = {(&sum_acc[23:16]) ? sum_scan_p1_hi : sum_acc[31:24], sum_scan_p1_lo};

	wire [7:0] words_b0_inc, words_b1_inc, words_b2_inc, words_b3_inc;
	wire [7:0] words_p1_lo, words_p1_hi;
	wire [5:0] words_co_unused;
	am_add #(.W(8)) a_w_0 (.a(words_acc[7:0]),   .b(8'd0), .ci(1'b1), .s(words_b0_inc), .co(words_co_unused[0]));
	am_add #(.W(8)) a_w_1 (.a(words_acc[15:8]),  .b(8'd0), .ci(1'b1), .s(words_b1_inc), .co(words_co_unused[1]));
	assign words_inc[15:0] = {(&words_acc[7:0]) ? words_b1_inc : words_acc[15:8], words_b0_inc};
	assign words_c16_w = &words_acc[15:0];
	am_add #(.W(8)) a_w_2 (.a(words_acc[23:16]), .b(8'd0), .ci(1'b1), .s(words_b2_inc), .co(words_co_unused[2]));
	am_add #(.W(8)) a_w_3 (.a(words_acc[31:24]), .b(8'd0), .ci(1'b1), .s(words_b3_inc), .co(words_co_unused[3]));
	assign words_inc[31:16] = {
		(words_c16_q && (&words_acc[23:16])) ? words_b3_inc : words_acc[31:24],
		words_c16_q ? words_b2_inc : words_acc[23:16]
	};
	am_add #(.W(8)) a_wp1_l (.a(words_acc[23:16]), .b(8'd0), .ci(1'b1), .s(words_p1_lo), .co(words_co_unused[4]));
	am_add #(.W(8)) a_wp1_h (.a(words_acc[31:24]), .b(8'd0), .ci(1'b1), .s(words_p1_hi), .co(words_co_unused[5]));
	assign scan_words_hi_p1 = {(&words_acc[23:16]) ? words_p1_hi : words_acc[31:24], words_p1_lo};

	wire [7:0] up_x_lo_inc, up_x_hi_inc;
	am_add #(.W(16)) a_urow (.a(up_row),     .b(16'd0), .ci(1'b1), .s(up_row_inc),  .co(acc_co_unused[0]));
	am_add #(.W(8))  a_ux_l (.a(up_x[7:0]),  .b(8'd0),  .ci(1'b1), .s(up_x_lo_inc), .co(acc_co_unused[1]));
	am_add #(.W(8))  a_ux_h (.a(up_x[15:8]), .b(8'd0),  .ci(1'b1), .s(up_x_hi_inc), .co(acc_co_unused[3]));
	assign up_x_inc = {(&up_x[7:0]) ? up_x_hi_inc : up_x[15:8], up_x_lo_inc};
	am_add #(.W(6))  a_uidx (.a(up_idx),     .b(6'd0),  .ci(1'b1), .s(up_idx_inc),  .co(acc_co_unused[2]));
	assign acc_co_unused[4] = sum_c24_0 ^ sum_c24_1 ^ (&sum_p1_co_un) ^ (&words_co_unused);
	wire unused_dma_sum_top = 1'b0;
`else
	am_add #(.W(32)) a_sum  (.a(sum_acc),   .b(sum_px), .ci(1'b0), .s(sum_next),   .co(acc_co_unused[3]));
	am_add #(.W(32)) a_w    (.a(words_acc), .b(32'd0),  .ci(1'b1), .s(words_inc),  .co(acc_co_unused[4]));
	am_add #(.W(16)) a_urow (.a(up_row),    .b(16'd0),  .ci(1'b1), .s(up_row_inc), .co(acc_co_unused[0]));
	am_add #(.W(16)) a_ux   (.a(up_x),      .b(16'd0),  .ci(1'b1), .s(up_x_inc),   .co(acc_co_unused[1]));
	am_add #(.W(6))  a_uidx (.a(up_idx),    .b(6'd0),   .ci(1'b1), .s(up_idx_inc), .co(acc_co_unused[2]));
	assign sum_c16_w = 1'b0;
	assign words_c16_w = 1'b0;
	assign scan_sum_hi_p1 = sum_acc[31:16];
	assign scan_words_hi_p1 = words_acc[31:16];
	wire unused_dma_sum_top = sum_c16_q ^ words_c16_q;
`endif

	assign done = !rq_active_done && !up_active && !sum_push;

`ifdef SYNTH_KINTEX7
	wire push_w_lo, push_w_hi, push_up0, push_up1, push_idx_lo, push_idx_hi;
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pwl   (.I0(px_valid), .I1(px_ready), .O(push_w_lo));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pwh   (.I0(px_valid), .I1(px_ready), .O(push_w_hi));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pup0  (.I0(px_valid), .I1(px_ready), .O(push_up0));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pup1  (.I0(px_valid), .I1(px_ready), .O(push_up1));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pidxl (.I0(px_valid), .I1(px_ready), .O(push_idx_lo));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pidxh (.I0(px_valid), .I1(px_ready), .O(push_idx_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire push_w_lo, push_w_hi, push_up0, push_up1, push_idx_lo, push_idx_hi;
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pwl   (.I0(px_valid), .I1(px_ready), .O(push_w_lo));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pwh   (.I0(px_valid), .I1(px_ready), .O(push_w_hi));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pup0  (.I0(px_valid), .I1(px_ready), .O(push_up0));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pup1  (.I0(px_valid), .I1(px_ready), .O(push_up1));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pidxl (.I0(px_valid), .I1(px_ready), .O(push_idx_lo));
	(* keep = "true" *) LUT2 #(.INIT(4'h8)) u_buf_pidxh (.I0(px_valid), .I1(px_ready), .O(push_idx_hi));
`else
	wire push_w_lo = push;
	wire push_w_hi = push;
	wire push_up0 = push;
	wire push_up1 = push;
	wire push_idx_lo = push;
	wire push_idx_hi = push;
`endif

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			up_active <= 1'b0;
		else if (abort_act)
			up_active <= start_up && fb_w_ff != 16'd0 && fb_h_c0 != 16'd0;
		else if (push_up1 && row_end_act && row_h_end)
			up_active <= 1'b0;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_w_last <= 16'd0;
		else if (abort_fw_l)
			f_w_last <= calc_w_last;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			f_w_m2 <= 16'd0;
		else if (abort_fw_m)
			f_w_m2 <= calc_w_m2;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			scan_sum[15:0] <= 16'd0;
		else if (abort_scan_l)
			scan_sum[15:0] <= sum_acc[15:0];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			scan_sum[31:16] <= 16'd0;
		else if (abort_scan_h)
			scan_sum[31:16] <= sum_c16_q ? scan_sum_hi_p1 : sum_acc[31:16];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			scan_words[15:0] <= 16'd0;
		else if (abort_sw_l)
			scan_words[15:0] <= words_acc[15:0];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			scan_words[31:16] <= 16'd0;
		else if (abort_sw_h)
			scan_words[31:16] <= words_c16_q ? scan_words_hi_p1 : words_acc[31:16];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			sum_acc[15:0] <= 16'd0;
			sum_c16_q     <= 1'b0;
		end else if (abort_sum_l) begin
			sum_acc[15:0] <= 16'd0;
			sum_c16_q     <= 1'b0;
		end else if (sum_push_lo) begin
			sum_acc[15:0] <= sum_next[15:0];
			sum_c16_q     <= sum_c16_w;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			sum_acc[31:16] <= 16'd0;
		else if (abort_sum_h)
			sum_acc[31:16] <= 16'd0;
		else if (sum_push_hi)
			sum_acc[31:16] <= sum_next[31:16];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			words_acc[15:0] <= 16'd0;
			words_c16_q     <= 1'b0;
		end else if (abort_w_l) begin
			words_acc[15:0] <= 16'd0;
			words_c16_q     <= 1'b0;
		end else if (push_w_lo) begin
			words_acc[15:0] <= words_inc[15:0];
			words_c16_q     <= words_c16_w;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			words_acc[31:16] <= 16'd0;
		else if (abort_w_h)
			words_acc[31:16] <= 16'd0;
		else if (push_w_hi)
			words_acc[31:16] <= words_inc[31:16];
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			up_x <= 16'd0;
		else if (abort_x)
			up_x <= 16'd0;
		else if (push_up0)
			up_x <= row_end_x ? 16'd0 : up_x_inc;
	end

	(* keep = "true", equivalent_register_removal = "no" *) reg [5:0] up_idx_m;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			up_idx[2:0]   <= 3'd0;
			up_idx_m[2:0] <= 3'd0;
			ppl_match_r   <= 1'b0;
		end else if (abort_idx) begin
			up_idx[2:0]   <= 3'd0;
			up_idx_m[2:0] <= 3'd0;
			ppl_match_r   <= 1'b0;
		end else if (push_idx_lo) begin
			up_idx[2:0]   <= row_end_idx0 ? 3'd0 : (ppl_match ? 3'd0 : up_idx_inc[2:0]);
			up_idx_m[2:0] <= row_end_idx0 ? 3'd0 : (ppl_match ? 3'd0 : up_idx_inc[2:0]);
			ppl_match_r   <= (row_end_idx0 || ppl_match) ? 1'b0 : (up_idx == ppl_m2);
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			up_idx[5:3]   <= 3'd0;
			up_idx_m[5:3] <= 3'd0;
		end else if (abort_idx) begin
			up_idx[5:3]   <= 3'd0;
			up_idx_m[5:3] <= 3'd0;
		end else if (push_idx_hi) begin
			up_idx[5:3]   <= row_end_idx1 ? 3'd0 : (ppl_match ? 3'd0 : up_idx_inc[5:3]);
			up_idx_m[5:3] <= row_end_idx1 ? 3'd0 : (ppl_match ? 3'd0 : up_idx_inc[5:3]);
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			up_row <= 16'd0;
		else if (abort_row)
			up_row <= 16'd0;
		else if (push_up1 && row_end_row)
			up_row <= up_row_inc;
	end

	wire unused_ok = &{1'b0, fb_base[LINE_SHIFT-1:0], fb_stride[LINE_SHIFT-1:0],
		rsp_tag[7:4], l_in_ready, up_idx[5:LINE_SHIFT-1], up_idx_m[5:LINE_SHIFT-1], row_co_unused_v, rq_co_unused,
		up_co_unused, acc_co_unused, u_co_unused, rsp_err_ctl, f_nl, f_w, f_h, ppl,
		unused_dma_sum_top, l_fifo_co_unused, abort_rsp_k1, abort_rsp_e1, nrow_c16_q,
		nrow_rdy_nr, nrow_rdy_nr2, nrow_rdy_nhi};
endmodule

`default_nettype wire
