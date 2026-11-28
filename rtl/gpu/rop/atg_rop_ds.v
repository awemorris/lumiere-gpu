/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_ds: ROP depth and stencil: reads the depth/stencil line of the
 * quad (DS_BASE/STRIDE, D32F or D24S8), tests (DEPTH func, STENCIL_*
 * front/back), updates and writes it back, and passes the surviving lanes.
 *
 * Each (lane, sample) entry is tested by atg_rop_dtest (CAPS[30]: depth
 * bounds, the stencil test and ops with the quad's facing, the depth test
 * of D16, D24S8 and D32F; the written bytes from its byte enables).  One
 * quad at a time: per row with covered lanes, read the line, test its
 * entries, write the changed bytes and wait for the write's response, so
 * the next quad reads what this one wrote.  D16 pixels are 2 bytes (sample
 * s of pixel x at byte 2 (S x + s) of the row), the others 4.  Quads with no lane are dropped.  The direct
 * path's quads (q_dbg) get DEBUG_COLOR as fp32 c/255.
 *
 * Multisampling (MS_CTRL[3:0] = 2, docs/ghci.md section 12.11): the
 * samples of a pixel are consecutive words (sample s of pixel x at word
 * 4 x + s of the row), so a quad's row is one line still; the test runs
 * per sample of the row's lanes (8 entries; 2 with one sample), each with
 * its own depth (q_sdepth), and the surviving samples go on (d_smask).
 * Beats (CAPS[30]): a quad comes as one beat per colour attachment
 * (q_beat = {rt, nocol, first}); the first beat is tested and written,
 * the later ones go on with its surviving samples without a memory access
 * (dropped with it).  A beat without colour goes on too (atg_rop counts
 * the first beats' samples for occlusion queries; atg_rop_wb drops it).
 * See spec/gpu/rop/atg_rop_ds.md.
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rop_ds #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter STENCIL = 0,	/* Stencil present (ROP_STENCIL) */
	parameter ENABLE_DEPTH32F = 1,
	parameter ENABLE_DEPTH24_S8 = 1,
	parameter ENABLE_DEPTH16 = 1
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* From atg_rop_order */
	input wire q_valid,
	output wire q_ready,
	input wire [31:0] q_xy,
	input wire [15:0] q_smask,	/* bit 4 l + s: sample s of lane l */
	input wire [511:0] q_color,
	input wire [511:0] q_color1,
	input wire [511:0] q_sdepth,	/* sample s of lane l at 32 (4 l + s) */
	input wire q_dbg,
	input wire q_front,	/* front facing (CAPS[30]; the direct path: 1) */
	input wire [4:0] q_beat,	/* {rt[2:0], nocol, first} */
	/* To atg_rop_wb */
	output reg d_valid,
	input wire d_ready,
	(* max_fanout = 8 *) output reg [31:0] d_xy,
	output reg [15:0] d_smask,
	output reg [511:0] d_color,
	output reg [511:0] d_color1,
	output reg [4:0] d_beat,
	output wire busy,
	/* AMB client port (design §4.1) */
	output reg req_valid,
	input wire req_ready,
	output reg req_write,
	output reg [ADDR_W-LINE_SHIFT-1:0] req_line,
	output reg [LINE_BYTES-1:0] req_be,
	output reg [LINE_BYTES*8-1:0] req_wdata,
	output wire [7:0] req_tag,
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	`include "atg_rop_unorm8.vh"
	`include "ghci_regs.vh"

	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	/* Distributed ROM for unorm8 conversion to avoid BRAM inference */
	(* rom_style = "distributed" *) reg [31:0] rom_unorm8 [0:255];
	integer r_u8_i;
	initial begin
		for (r_u8_i = 0; r_u8_i < 256; r_u8_i = r_u8_i + 1)
			rom_unorm8[r_u8_i] = atg_rop_unorm8(r_u8_i[7:0]);
	end

	localparam WPL = LINE_BYTES / 4;	/* words per line */
	localparam WB = LINE_SHIFT - 2;	/* word index bits */
	integer k;

	/* STATE: DEBUG_COLOR 0x05C, DS_BASE 0x074, DS_STRIDE 0x075, DEPTH 0x076,
	 * STENCIL_FRONT, _OPS, _BACK 0x077-0x079, DEPTH_BOUNDS_MIN, _MAX 0x08B,
	 * 0x08C. */
	(* keep = "true", dont_touch = "true" *) reg [31:0] dbg_color_lo, dbg_color_hi;
	reg [31:0] ds_base, bmin, bmax;
	(* keep = "true", dont_touch = "true" *) reg [31:0] ds_stride_mul, ds_stride_add;
	reg [9:0] depth;
	(* keep = "true", dont_touch = "true" *) reg [9:0] depth_d24_rep;
	(* keep = "true", equivalent_register_removal = "no" *) reg [10:0] d16_buf;
	reg [26:0] sfront, sback;
	reg [24:0] sops;
	(* keep = "true", dont_touch = "true" *) reg [2:0] ms4_rep;

`ifdef SYNTH_KINTEX7
	wire we_sfront_raw = st_we && (st_idx == GHCI_S_STENCIL_FRONT);
	wire we_sback_raw  = st_we && (st_idx == GHCI_S_STENCIL_BACK);
	wire we_sops_raw   = st_we && (st_idx == GHCI_S_STENCIL_OPS);
	wire we_bmin_raw   = st_we && (st_idx == GHCI_S_DEPTH_BOUNDS_MIN);
	wire we_bmax_raw   = st_we && (st_idx == GHCI_S_DEPTH_BOUNDS_MAX);
	wire we_dbg_raw    = st_we && (st_idx == GHCI_S_DEBUG_COLOR);
	wire we_base_raw   = st_we && (st_idx == GHCI_S_DS_BASE);
	wire we_stride_raw = st_we && (st_idx == GHCI_S_DS_STRIDE);
	wire we_depth_raw  = st_we && (st_idx == GHCI_S_DEPTH);
	wire we_ms_raw     = st_we && (st_idx == GHCI_S_MS_CTRL);

	wire [1:0] we_sfront_b, we_sback_b, we_sops_b, we_bmin_b, we_bmax_b, we_base_b, we_depth_b;
	wire [3:0] we_dbg_b, we_stride_b;
	wire we_ms_b;
	genvar g_st_b;
	generate
	for (g_st_b = 0; g_st_b < 2; g_st_b = g_st_b + 1) begin : g_st_we2
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sf (.I0(we_sfront_raw), .O(we_sfront_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sb (.I0(we_sback_raw),  .O(we_sback_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_so (.I0(we_sops_raw),   .O(we_sops_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bn (.I0(we_bmin_raw),   .O(we_bmin_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bx (.I0(we_bmax_raw),   .O(we_bmax_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bs (.I0(we_base_raw),   .O(we_base_b[g_st_b]));
	end
	for (g_st_b = 0; g_st_b < 4; g_st_b = g_st_b + 1) begin : g_st_we4
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_db (.I0(we_dbg_raw),    .O(we_dbg_b[g_st_b]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_st (.I0(we_stride_raw), .O(we_stride_b[g_st_b]));
	end
	endgenerate
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dp0 (.I0(we_depth_raw), .O(we_depth_b[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dp1 (.I0(we_depth_raw), .O(we_depth_b[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ms (.I0(we_ms_raw),    .O(we_ms_b));

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			ms4_rep <= 3'd0;
			dbg_color_lo <= 32'd0;
			dbg_color_hi <= 32'd0;
			ds_base <= 32'd0;
			ds_stride_mul <= 32'd0;
			ds_stride_add <= 32'd0;
			depth <= 10'd0;
			depth_d24_rep <= 10'd0;
			d16_buf <= 11'd0;
			sfront <= 27'd0;
			sback <= 27'd0;
			sops <= 25'd0;
			bmin <= 32'd0;
			bmax <= 32'd0;
		end else begin
			if (we_sfront_b[0]) sfront[13:0]  <= st_wdata[13:0];
			if (we_sfront_b[1]) sfront[26:14] <= st_wdata[26:14];
			if (we_sback_b[0])  sback[13:0]   <= st_wdata[13:0];
			if (we_sback_b[1])  sback[26:14]  <= st_wdata[26:14];
			if (we_sops_b[0])   sops[12:0]    <= st_wdata[12:0];
			if (we_sops_b[1])   sops[24:13]   <= st_wdata[24:13];
			if (we_bmin_b[0])   bmin[15:0]    <= st_wdata[15:0];
			if (we_bmin_b[1])   bmin[31:16]   <= st_wdata[31:16];
			if (we_bmax_b[0])   bmax[15:0]    <= st_wdata[15:0];
			if (we_bmax_b[1])   bmax[31:16]   <= st_wdata[31:16];
			if (we_dbg_b[0])    dbg_color_lo[15:0]  <= st_wdata[15:0];
			if (we_dbg_b[1])    dbg_color_lo[31:16] <= st_wdata[31:16];
			if (we_dbg_b[2])    dbg_color_hi[15:0]  <= st_wdata[15:0];
			if (we_dbg_b[3])    dbg_color_hi[31:16] <= st_wdata[31:16];
			if (we_base_b[0])   ds_base[15:0]  <= st_wdata[15:0];
			if (we_base_b[1])   ds_base[31:16] <= st_wdata[31:16];
			if (we_stride_b[0]) ds_stride_mul[15:0]  <= st_wdata[15:0];
			if (we_stride_b[1]) ds_stride_mul[31:16] <= st_wdata[31:16];
			if (we_stride_b[2]) ds_stride_add[15:0]  <= st_wdata[15:0];
			if (we_stride_b[3]) ds_stride_add[31:16] <= st_wdata[31:16];
			if (we_depth_b[0]) begin
				depth <= st_wdata[9:0];
				d16_buf[5:0] <= {6{st_wdata[7:5] == 3'b100}};
			end
			if (we_depth_b[1]) begin
				depth_d24_rep <= {10{st_wdata[7:5] == 3'b001}};
				d16_buf[10:6] <= {5{st_wdata[7:5] == 3'b100}};
			end
			if (we_ms_b)
				ms4_rep <= {3{st_wdata[3:0] == 4'd2}};
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire we_sfront_raw = st_we && (st_idx == GHCI_S_STENCIL_FRONT);
	wire we_sback_raw  = st_we && (st_idx == GHCI_S_STENCIL_BACK);
	wire we_sops_raw   = st_we && (st_idx == GHCI_S_STENCIL_OPS);
	wire we_bmin_raw   = st_we && (st_idx == GHCI_S_DEPTH_BOUNDS_MIN);
	wire we_bmax_raw   = st_we && (st_idx == GHCI_S_DEPTH_BOUNDS_MAX);
	wire we_dbg_raw    = st_we && (st_idx == GHCI_S_DEBUG_COLOR);
	wire we_base_raw   = st_we && (st_idx == GHCI_S_DS_BASE);
	wire we_stride_raw = st_we && (st_idx == GHCI_S_DS_STRIDE);
	wire we_depth_raw  = st_we && (st_idx == GHCI_S_DEPTH);
	wire we_ms_raw     = st_we && (st_idx == GHCI_S_MS_CTRL);

	wire [1:0] we_sfront_b, we_sback_b, we_sops_b, we_bmin_b, we_bmax_b, we_base_b, we_depth_b;
	wire [3:0] we_dbg_b, we_stride_b;
	wire we_ms_b;
	genvar g_st_b;
	generate
	for (g_st_b = 0; g_st_b < 2; g_st_b = g_st_b + 1) begin : g_st_we2
		assign we_sfront_b[g_st_b] = we_sfront_raw;
		assign we_sback_b[g_st_b] = we_sback_raw;
		assign we_sops_b[g_st_b] = we_sops_raw;
		assign we_bmin_b[g_st_b] = we_bmin_raw;
		assign we_bmax_b[g_st_b] = we_bmax_raw;
		assign we_base_b[g_st_b] = we_base_raw;
	end
	for (g_st_b = 0; g_st_b < 4; g_st_b = g_st_b + 1) begin : g_st_we4
		assign we_dbg_b[g_st_b] = we_dbg_raw;
		assign we_stride_b[g_st_b] = we_stride_raw;
	end
	endgenerate
	assign we_depth_b[0] = we_depth_raw;
	assign we_depth_b[1] = we_depth_raw;
	assign we_ms_b = we_ms_raw;

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			ms4_rep <= 3'd0;
			dbg_color_lo <= 32'd0;
			dbg_color_hi <= 32'd0;
			ds_base <= 32'd0;
			ds_stride_mul <= 32'd0;
			ds_stride_add <= 32'd0;
			depth <= 10'd0;
			depth_d24_rep <= 10'd0;
			d16_buf <= 11'd0;
			sfront <= 27'd0;
			sback <= 27'd0;
			sops <= 25'd0;
			bmin <= 32'd0;
			bmax <= 32'd0;
		end else begin
			if (we_sfront_b[0]) sfront[13:0]  <= st_wdata[13:0];
			if (we_sfront_b[1]) sfront[26:14] <= st_wdata[26:14];
			if (we_sback_b[0])  sback[13:0]   <= st_wdata[13:0];
			if (we_sback_b[1])  sback[26:14]  <= st_wdata[26:14];
			if (we_sops_b[0])   sops[12:0]    <= st_wdata[12:0];
			if (we_sops_b[1])   sops[24:13]   <= st_wdata[24:13];
			if (we_bmin_b[0])   bmin[15:0]    <= st_wdata[15:0];
			if (we_bmin_b[1])   bmin[31:16]   <= st_wdata[31:16];
			if (we_bmax_b[0])   bmax[15:0]    <= st_wdata[15:0];
			if (we_bmax_b[1])   bmax[31:16]   <= st_wdata[31:16];
			if (we_dbg_b[0])    dbg_color_lo[15:0]  <= st_wdata[15:0];
			if (we_dbg_b[1])    dbg_color_lo[31:16] <= st_wdata[31:16];
			if (we_dbg_b[2])    dbg_color_hi[15:0]  <= st_wdata[15:0];
			if (we_dbg_b[3])    dbg_color_hi[31:16] <= st_wdata[31:16];
			if (we_base_b[0])   ds_base[15:0]  <= st_wdata[15:0];
			if (we_base_b[1])   ds_base[31:16] <= st_wdata[31:16];
			if (we_stride_b[0]) ds_stride_mul[15:0]  <= st_wdata[15:0];
			if (we_stride_b[1]) ds_stride_mul[31:16] <= st_wdata[31:16];
			if (we_stride_b[2]) ds_stride_add[15:0]  <= st_wdata[15:0];
			if (we_stride_b[3]) ds_stride_add[31:16] <= st_wdata[31:16];
			if (we_depth_b[0]) begin
				depth <= st_wdata[9:0];
				d16_buf[5:0] <= {6{st_wdata[7:5] == 3'b100}};
			end
			if (we_depth_b[1]) begin
				depth_d24_rep <= {10{st_wdata[7:5] == 3'b001}};
				d16_buf[10:6] <= {5{st_wdata[7:5] == 3'b100}};
			end
			if (we_ms_b)
				ms4_rep <= {3{st_wdata[3:0] == 4'd2}};
		end
	end
`else
	always @(posedge clk or posedge rst_sync)
		if (rst_sync) begin
			ms4_rep <= 3'd0;
			dbg_color_lo <= 32'd0;
			dbg_color_hi <= 32'd0;
			ds_base <= 32'd0;
			ds_stride_mul <= 32'd0;
			ds_stride_add <= 32'd0;
			depth <= 10'd0;
			depth_d24_rep <= 10'd0;
			d16_buf <= 11'd0;
			sfront <= 27'd0;
			sback <= 27'd0;
			sops <= 25'd0;
			bmin <= 32'd0;
			bmax <= 32'd0;
		end else if (st_we) begin
			if (st_idx == GHCI_S_STENCIL_FRONT)
				sfront <= st_wdata[26:0];
			if (st_idx == GHCI_S_STENCIL_BACK)
				sback <= st_wdata[26:0];
			if (st_idx == GHCI_S_STENCIL_OPS)
				sops <= st_wdata[24:0];
			if (st_idx == GHCI_S_DEPTH_BOUNDS_MIN)
				bmin <= st_wdata;
			if (st_idx == GHCI_S_DEPTH_BOUNDS_MAX)
				bmax <= st_wdata;
			if (st_idx == GHCI_S_DEBUG_COLOR) begin
				dbg_color_lo <= st_wdata;
				dbg_color_hi <= st_wdata;
			end
			if (st_idx == GHCI_S_DS_BASE)
				ds_base <= st_wdata;
			if (st_idx == GHCI_S_DS_STRIDE) begin
				ds_stride_mul <= st_wdata;
				ds_stride_add <= st_wdata;
			end
			if (st_idx == GHCI_S_DEPTH) begin
				depth <= st_wdata[9:0];
				depth_d24_rep <= {10{st_wdata[7:5] == 3'b001}};
				d16_buf <= {11{st_wdata[7:5] == 3'b100}};
			end
			if (st_idx == GHCI_S_MS_CTRL)
				ms4_rep <= {3{st_wdata[3:0] == 4'd2}};
		end
`endif
	/* a test reads the line: exactly one format, and a test enabled */
	wire one_fmt = depth[7:5] == 3'b001 || depth[7:5] == 3'b010 || d16_buf[4];
	wire test = one_fmt && (depth[0] || (depth[8] && depth[7:5] == 3'b001) || depth[9]);

	localparam [4:0] S_IDLE = 5'd0, S_ROW_SETUP = 5'd1, S_ROW = 5'd2, S_RD = 5'd3,
		S_TEST0 = 5'd4, S_TEST0A = 5'd13, S_TEST0B = 5'd10, S_TEST1 = 5'd5,
		S_TEST2 = 5'd6, S_TEST2B = 5'd14, S_TEST3 = 5'd7,
		S_WR = 5'd8, S_OUT = 5'd9, S_ROW_SETUP1 = 5'd11, S_ROW_SETUP2 = 5'd12,
		S_TEST_PRE = 5'd15, S_ROW_SETUP3 = 5'd16, S_ROW_SETUP4 = 5'd18;
	(* max_fanout = 16 *) reg [4:0] st;
	(* keep = "true", dont_touch = "true" *) reg [4:0] r_rep;	/* row */
	(* keep = "true", dont_touch = "true" *) reg [31:0] ds_xy_int;
	reg [2:0] e;	/* entry of the row: {lane of the row, sample} */
	reg wait_rsp;
	reg [15:0] mask, pm;
	reg [511:0] z;
	reg [31:0] addr;
	reg [LINE_BYTES*8-1:0] line;
	reg wr_any;
	(* keep = "true", dont_touch = "true" *) reg [4:0] front_rep;

	/* Pipelined per-entry test staging registers */
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] test_stored;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] test_zl;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] test_zun;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [23:0] test_zu24;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [15:0] test_zu16;
	reg [32:0] test_zkey;
	reg [32:0] test_skey;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [7:0] test_s_inc;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [7:0] test_s_dec;
	(* keep = "true", dont_touch = "true" *) reg [LINE_SHIFT-1:0] test_bo_rep [0:3];
	reg [3:0] test_sb;
	(* keep = "true", dont_touch = "true" *) reg [3:0] test_mask_rep;
	reg test_pass;
	(* keep = "true", dont_touch = "true" *) reg [31:0] test_wword_rep [0:1];
	(* keep = "true", dont_touch = "true" *) reg [3:0] test_wbe_rep [0:3];

	(* keep = "true", dont_touch = "true" *) reg [31:0] row_base0, row_base0_rb1;
	reg [31:0] row_base1;

	wire [1:0] lane = {r_rep[4], e[2]};
	wire [3:0] sb = {lane, e[1:0]};	/* the sample's bit */
`ifdef SYNTH_KINTEX7
	wire [3:0] sb_buf [0:7];
	wire [31:0] zl;
	genvar g_sb;
	generate
	for (g_sb = 0; g_sb < 8; g_sb = g_sb + 1) begin : g_buf_sb
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sb0 (.I0(sb[0]), .O(sb_buf[g_sb][0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sb1 (.I0(sb[1]), .O(sb_buf[g_sb][1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sb2 (.I0(sb[2]), .O(sb_buf[g_sb][2]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sb3 (.I0(sb[3]), .O(sb_buf[g_sb][3]));
		assign zl[4 * g_sb +: 4] = z[32 * sb_buf[g_sb] + 4 * g_sb +: 4];
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] sb_buf [0:7];
	wire [31:0] zl;
	genvar g_sb;
	generate
	for (g_sb = 0; g_sb < 8; g_sb = g_sb + 1) begin : g_buf_sb
		assign sb_buf[g_sb][0] = sb[0];
		assign sb_buf[g_sb][1] = sb[1];
		assign sb_buf[g_sb][2] = sb[2];
		assign sb_buf[g_sb][3] = sb[3];
		assign zl[4 * g_sb +: 4] = z[32 * sb_buf[g_sb] + 4 * g_sb +: 4];
	end
	endgenerate
`else
	wire [31:0] zl = z[32 * sb +: 32];
`endif

	/* Stage 0A: Pre-shift subtractors for u24 and u16, and LZC for zun */
	wire [47:0] u24_p;
	wire [39:0] u16_p;
	wire u24_p_bo, u16_p_bo, u24_r_co, u16_r_co;
	am_sub #(.W(48)) u_u24_p (.a({1'b1, test_zl[22:0], 24'd0}), .b({24'd0, 1'b1, test_zl[22:0]}), .bi(1'b0), .d(u24_p),
		.bo(u24_p_bo));
	am_sub #(.W(40)) u_u16_p (.a({1'b1, test_zl[22:0], 16'd0}), .b({16'd0, 1'b1, test_zl[22:0]}), .bi(1'b0), .d(u16_p),
		.bo(u16_p_bo));

	wire [4:0] sub30_w;
	wire sub30_bo_unused;
	am_sub #(.W(5)) u_sub30 (
		.a(5'd30),
		.b(test_zl[27:23]),
		.bi(1'b0),
		.d(sub30_w),
		.bo(sub30_bo_unused)
	);

	(* keep = "true" *) wire [3:0] zl_mnz;
	assign zl_mnz[0] = (test_zl[5:0]   != 6'd0);
	assign zl_mnz[1] = (test_zl[11:6]  != 6'd0);
	assign zl_mnz[2] = (test_zl[17:12] != 6'd0);
	assign zl_mnz[3] = (test_zl[22:18] != 5'd0);
	wire zl_mant_nz = |zl_mnz;
	wire zl_st_w = zl_mant_nz || (test_zl[30:23] != 8'd126);

	reg [47:0] u24_s0a_p;
	reg [39:0] u16_s0a_p;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [4:0] u24_s0a_t;
	reg zl_s0a_st;
	reg [71:0] zun_s0a_r;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [4:0] zun_s0a_lz;
	(* keep = "true", dont_touch = "true" *) reg [3:0] zun_s0a_found;

	wire [47:0] u24_sh;
	wire [39:0] u16_sh;
	am_shift #(.W(48), .SW(5)) u_u24_z (.a(u24_s0a_p), .amt(u24_s0a_t), .left(1'b0), .arith(1'b0), .y(u24_sh));
	am_shift #(.W(40), .SW(5)) u_u16_z (.a(u16_s0a_p), .amt(u24_s0a_t), .left(1'b0), .arith(1'b0), .y(u16_sh));

	reg [23:0] u24_stage0_hi;
	reg u24_stage0_round;
	(* keep = "true", dont_touch = "true" *) reg [2:0] u24_stage0_zero;
	(* keep = "true", dont_touch = "true" *) reg [2:0] u24_stage0_max;

	reg [15:0] u16_stage0_hi;
	reg u16_stage0_round;
	(* keep = "true", dont_touch = "true" *) reg [1:0] u16_stage0_zero;
	(* keep = "true", dont_touch = "true" *) reg [1:0] u16_stage0_max;

	wire [23:0] u24_r;
	am_add #(.W(24)) u_u24_r (.a(u24_stage0_hi), .b(24'd0), .ci(u24_stage0_round), .s(u24_r),
		.co(u24_r_co));
	wire [23:0] u24_final = {
		u24_stage0_zero[2] ? 8'd0 : u24_stage0_max[2] ? 8'hFF : u24_r[23:16],
		u24_stage0_zero[1] ? 8'd0 : u24_stage0_max[1] ? 8'hFF : u24_r[15:8],
		u24_stage0_zero[0] ? 8'd0 : u24_stage0_max[0] ? 8'hFF : u24_r[7:0]
	};

	wire [15:0] u16_r;
	am_add #(.W(16)) u_u16_r (.a(u16_stage0_hi), .b(16'd0), .ci(u16_stage0_round), .s(u16_r),
		.co(u16_r_co));
	wire [15:0] u16_final = {
		u16_stage0_zero[1] ? 8'd0 : u16_stage0_max[1] ? 8'hFF : u16_r[15:8],
		u16_stage0_zero[0] ? 8'd0 : u16_stage0_max[0] ? 8'hFF : u16_r[7:0]
	};

	/* Pipelined zun depth conversion (replicated depth_d24_rep[7:0] keeps fanout <= 10 per FF) */
	wire [79:0] zun_d24_pat = {test_stored[23:0], test_stored[23:0], test_stored[23:0], test_stored[23:16]};
	wire [79:0] zun_d16_pat = {5{test_stored[15:0]}};
	(* keep = "true" *) wire [79:0] zun_r = {
		depth_d24_rep[7] ? zun_d24_pat[79:70] : zun_d16_pat[79:70],
		depth_d24_rep[6] ? zun_d24_pat[69:60] : zun_d16_pat[69:60],
		depth_d24_rep[5] ? zun_d24_pat[59:50] : zun_d16_pat[59:50],
		depth_d24_rep[4] ? zun_d24_pat[49:40] : zun_d16_pat[49:40],
		depth_d24_rep[3] ? zun_d24_pat[39:30] : zun_d16_pat[39:30],
		depth_d24_rep[2] ? zun_d24_pat[29:20] : zun_d16_pat[29:20],
		depth_d24_rep[1] ? zun_d24_pat[19:10] : zun_d16_pat[19:10],
		depth_d24_rep[0] ? zun_d24_pat[9:0]   : zun_d16_pat[9:0]
	};
	/* Compute d24 and d16 LZC in parallel, muxed by depth_d24_rep at the end (<= 10 cmos3) */
	function [4:0] lzc6_fn;
		input [5:0] v;
		input [4:0] base;
		begin
			casez (v)
			6'b1?????: lzc6_fn = base + 5'd0;
			6'b01????: lzc6_fn = base + 5'd1;
			6'b001???: lzc6_fn = base + 5'd2;
			6'b0001??: lzc6_fn = base + 5'd3;
			6'b00001?: lzc6_fn = base + 5'd4;
			default:   lzc6_fn = base + 5'd5;
			endcase
		end
	endfunction
	wire [3:0] d24_gnz = {|test_stored[23:18], |test_stored[17:12], |test_stored[11:6], |test_stored[5:0]};
	wire [4:0] d24_lz_hi = d24_gnz[3] ? lzc6_fn(test_stored[23:18], 5'd0) : lzc6_fn(test_stored[17:12], 5'd6);
	wire [4:0] d24_lz_lo = d24_gnz[1] ? lzc6_fn(test_stored[11:6], 5'd12) : (d24_gnz[0] ? lzc6_fn(test_stored[5:0], 5'd18) : 5'd0);
	wire [4:0] d24_lz    = (d24_gnz[3] || d24_gnz[2]) ? d24_lz_hi : d24_lz_lo;
	wire [2:0] d16_gnz   = {|test_stored[15:10], |test_stored[9:4], |test_stored[3:0]};
	wire [4:0] d16_lz    = d16_gnz[2] ? lzc6_fn(test_stored[15:10], 5'd0) :
	                       (d16_gnz[1] ? lzc6_fn(test_stored[9:4], 5'd6) :
	                       (d16_gnz[0] ? lzc6_fn({test_stored[3:0], 2'b00}, 5'd12) : 5'd0));
	wire zun_found = depth_d24_rep[8] ? (|d24_gnz) : (|d16_gnz);
	wire [4:0] zun_lz = depth_d24_rep[9] ? d24_lz : d16_lz;

	wire [71:0] zun_mn;
	am_shift #(.W(72), .SW(5)) u_zun_mn (.a(zun_s0a_r), .amt(zun_s0a_lz), .left(1'b1), .arith(1'b0), .y(zun_mn));

	wire [7:0] sub126_w;
	wire sub126_bo_unused;
	am_sub #(.W(8)) u_sub126 (
		.a(8'd126),
		.b({3'd0, zun_s0a_lz}),
		.bi(1'b0),
		.d(sub126_w),
		.bo(sub126_bo_unused)
	);

	reg [22:0] zun_s0_mn_hi;
	reg zun_s0_round;
	reg [7:0] zun_s0_e0, zun_s0_e1;
	(* keep = "true", dont_touch = "true" *) reg [3:0] zun_s0_found;

	wire [23:0] zun_q;
	wire zun_carry_unused;
	am_add #(.W(24)) u_zun_q (.a({1'b0, zun_s0_mn_hi}), .b(24'd0), .ci(zun_s0_round), .s(zun_q), .co(zun_carry_unused));
`ifdef SYNTH_KINTEX7
	wire [3:0] zun_q23_b;
	LUT1 #(.INIT(2'h2)) u_zq23_0 (.I0(zun_q[23]), .O(zun_q23_b[0]));
	LUT1 #(.INIT(2'h2)) u_zq23_1 (.I0(zun_q[23]), .O(zun_q23_b[1]));
	LUT1 #(.INIT(2'h2)) u_zq23_2 (.I0(zun_q[23]), .O(zun_q23_b[2]));
	LUT1 #(.INIT(2'h2)) u_zq23_3 (.I0(zun_q[23]), .O(zun_q23_b[3]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] zun_q23_b;
	assign zun_q23_b[0] = zun_q[23];
	assign zun_q23_b[1] = zun_q[23];
	assign zun_q23_b[2] = zun_q[23];
	assign zun_q23_b[3] = zun_q[23];
`else
	wire [3:0] zun_q23_b = {4{zun_q[23]}};
`endif
	wire [7:0] zun_e = zun_q23_b[3] ? zun_s0_e1 : zun_s0_e0;
	wire [22:0] zun_m = {
		zun_q23_b[2] ? 7'd0 : zun_q[22:16],
		zun_q23_b[1] ? 8'd0 : zun_q[15:8],
		zun_q23_b[0] ? 8'd0 : zun_q[7:0]
	};
	wire [31:0] zun_final = {
		!zun_s0_found[3] ? 8'd0 : {1'b0, zun_e[7:1]},
		!zun_s0_found[2] ? 8'd0 : {zun_e[0], zun_m[22:16]},
		!zun_s0_found[1] ? 8'd0 : zun_m[15:8],
		!zun_s0_found[0] ? 8'd0 : zun_m[7:0]
	};

	/* the entry's byte in the line: (S x + s) ps from the row's address */
	wire [2:0] ent = ms4_rep[2] ? e : {2'b00, e[2]};
	wire [LINE_SHIFT-1:0] bo;
	wire bo_co_unused;
	am_add #(.W(LINE_SHIFT)) u_bo (.a(addr[LINE_SHIFT-1:0]),
		.b(d16_buf[4] ? {{(LINE_SHIFT-4){1'b0}}, ent, 1'b0} : {{(LINE_SHIFT-5){1'b0}}, ent, 2'b00}),
		.ci(1'b0), .s(bo), .co(bo_co_unused));
	reg [LINE_SHIFT-1:0] r_bo;
	reg [3:0] r_sb;
	reg r_mask_sb;

	/* The row's address: base + y stride + x S ps; the next entry. */
	wire [31:0] yrs_p3;
	reg [31:0] yrs_r;
`ifdef SYNTH_KINTEX7
	wire [48:0] yrs_k7;
	am_mul #(.AW(17), .BW(32), .LAT(3)) u_yrs (.clk(clk), .en(1'b1), .a({1'b0, ds_xy_int[31:16]}), .b(ds_stride_mul), .p(yrs_k7));
	assign yrs_p3 = yrs_k7[31:0];
	wire unused_yrs_hi = &{1'b0, yrs_k7[48:32]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [48:0] yrs_k7;
	am_mul #(.AW(17), .BW(32), .LAT(3)) u_yrs (.clk(clk), .en(1'b1), .a({1'b0, ds_xy_int[31:16]}), .b(ds_stride_mul), .p(yrs_k7));
	assign yrs_p3 = yrs_k7[31:0];
	wire unused_yrs_hi = &{1'b0, yrs_k7[48:32]};
`else
	/* 3-stage Radix-4 Booth 16u x 32u -> 32u (<= 10 cmos3/stage, LAT=3 matching SYNTH_KINTEX7) */
	wire [18:0] yrs_a_pad = {2'b00, ds_xy_int[31:16], 1'b0};
	wire [7:0] yrs_neg;
	wire [31:0] yrs_pp [0:8];
	genvar gyk;
	generate
	for (gyk = 0; gyk < 8; gyk = gyk + 1) begin : g_yrs_booth
		wire b_m1 = yrs_a_pad[2 * gyk];
		wire b_0  = yrs_a_pad[2 * gyk + 1];
		wire b_p1 = yrs_a_pad[2 * gyk + 2];
		wire bo_s1 = b_m1 ^ b_0;
		wire bo_nz = bo_s1 | (b_p1 ^ b_0);
		wire neg_j = b_p1 && !(b_0 && b_m1);
		assign yrs_neg[gyk] = neg_j;
		wire [31 - 2*gyk : 0] u_w;
		genvar guk;
		for (guk = 0; guk <= 31 - 2*gyk; guk = guk + 1) begin : g_yu
			wire ak = ds_stride_mul[guk] ^ b_p1;
			wire akm1;
			if (guk > 0) begin : g_akm
				assign akm1 = ds_stride_mul[guk - 1] ^ b_p1;
			end else begin : g_akz
				assign akm1 = b_p1;
			end
			assign u_w[guk] = bo_nz & (bo_s1 ? ak : akm1);
		end
		if (gyk == 0) begin : g_yb0
			assign yrs_pp[0] = u_w;
		end else if (gyk == 1) begin : g_yb1
			assign yrs_pp[1] = {u_w, 1'b0, yrs_neg[0]};
		end else begin : g_ybm
			assign yrs_pp[gyk] = {u_w, 1'b0, yrs_neg[gyk - 1], {(2*gyk - 2){1'b0}}};
		end
	end
	assign yrs_pp[8] = {ds_xy_int[31] ? ds_stride_mul[15:0] : 16'd0, 1'b0, yrs_neg[7], 14'd0};

	/* Stage 1 CSA: 9 -> 5 rows via 2x 4:2 compressors (<= 10 cmos3 total incl. Booth) */
	wire [31:0] yrs_s1_w [0:4];
	for (gyk = 0; gyk < 2; gyk = gyk + 1) begin : g_yc42
		wire [31:0] a0 = yrs_pp[4 * gyk];
		wire [31:0] a1 = yrs_pp[4 * gyk + 1];
		wire [31:0] a2 = yrs_pp[4 * gyk + 2];
		wire [31:0] a3 = yrs_pp[4 * gyk + 3];
		wire [31:0] p0 = a0 ^ a1;
		wire [31:0] p1 = a2 ^ a3;
		wire [30:0] co_hi = (p0[30:0] & a2[30:0]) | (~p0[30:0] & a0[30:0]);
		wire [31:0] co = {co_hi, 1'b0};
		wire [30:0] x_hi = p0[30:0] ^ p1[30:0];
		assign yrs_s1_w[2 * gyk]     = p0 ^ p1 ^ co;
		assign yrs_s1_w[2 * gyk + 1] = {(x_hi & co[30:0]) | (~x_hi & a3[30:0]), 1'b0};
	end
	assign yrs_s1_w[4] = yrs_pp[8];
	reg [31:0] yrs_r1 [0:4];
	integer iy1;
	always @(posedge clk) begin
		for (iy1 = 0; iy1 < 5; iy1 = iy1 + 1)
			yrs_r1[iy1] <= yrs_s1_w[iy1];
	end

	/* Stage 2 CSA: 5 -> 4 -> 2 rows via 3:2 + 4:2 compressor (<= 8 cmos3) */
	wire [31:0] yrs_l2_0 = yrs_r1[0] ^ yrs_r1[1] ^ yrs_r1[2];
	wire [31:0] yrs_l2_1 = {((yrs_r1[0][30:0] & yrs_r1[1][30:0]) | (yrs_r1[0][30:0] & yrs_r1[2][30:0]) | (yrs_r1[1][30:0] & yrs_r1[2][30:0])), 1'b0};
	wire [31:0] yrs_p0 = yrs_l2_0 ^ yrs_l2_1;
	wire [31:0] yrs_p1 = yrs_r1[3] ^ yrs_r1[4];
	wire [31:0] yrs_co = {((yrs_p0[30:0] & yrs_r1[3][30:0]) | (~yrs_p0[30:0] & yrs_l2_0[30:0])), 1'b0};
	wire [30:0] yrs_x2 = yrs_p0[30:0] ^ yrs_p1[30:0];
	wire [31:0] yrs_l3_s = yrs_p0 ^ yrs_p1 ^ yrs_co;
	wire [31:0] yrs_l3_c = {(yrs_x2 & yrs_co[30:0]) | (~yrs_x2 & yrs_r1[4][30:0]), 1'b0};
	reg [31:0] yrs_r2_s, yrs_r2_c;
	always @(posedge clk) begin
		yrs_r2_s <= yrs_l3_s;
		yrs_r2_c <= yrs_l3_c;
	end

	/* Stage 3: 32-bit Kogge-Stone am_add */
	wire yrs_co_unused;
	am_add #(.W(32)) u_yrs_fin (.a(yrs_r2_s), .b(yrs_r2_c), .ci(1'b0), .s(yrs_p3), .co(yrs_co_unused));
	wire unused_yrs_hi = yrs_co_unused;
	endgenerate
`endif
	wire [15:0] xo_lo = ms4_rep[0] ? (d16_buf[5] ? {ds_xy_int[12:0], 3'b000} : {ds_xy_int[11:0], 4'b0000}) :
		(d16_buf[5] ? {ds_xy_int[14:0], 1'b0} : {ds_xy_int[13:0], 2'b00});
	wire [15:0] xo_hi = ms4_rep[1] ? (d16_buf[5] ? {13'd0, ds_xy_int[15:13]} : {12'd0, ds_xy_int[15:12]}) :
		(d16_buf[5] ? {15'd0, ds_xy_int[15]} : {14'd0, ds_xy_int[15:14]});
	wire [31:0] xo = {xo_hi, xo_lo};
	reg [31:0] r_xo;
	wire [31:0] row_a, row_base1_comb, cur_row_base, row_addr;
	wire [2:0] e_next = ms4_rep[2] ? {e[2] ^ (e[1] & e[0]), e[1] ^ e[0], ~e[0]} : {~e[2], e[1:0]};
	wire ra_co_unused, rb1_co_unused, rd_co_unused;
	am_add #(.W(32)) u_ra (.a(ds_base), .b(yrs_r), .ci(1'b0), .s(row_a), .co(ra_co_unused));
`ifdef SYNTH_KINTEX7
	am_add #(.W(32)) u_rb1 (.a(row_base0_rb1), .b(ds_stride_add), .ci(1'b0), .s(row_base1_comb), .co(rb1_co_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(32)) u_rb1 (.a(row_base0_rb1), .b(ds_stride_add), .ci(1'b0), .s(row_base1_comb), .co(rb1_co_unused));
`else
	/* 4+4 split carry-select with flat 8-bit CLA for row_base0_rb1 + ds_stride_add (<= 9 cmos3) */
	wire [31:0] rb1_g1 = row_base0_rb1 & ds_stride_add;
	wire [31:0] rb1_p1 = row_base0_rb1 | ds_stride_add;
	wire [7:0] rb1_g4, rb1_p4;
	genvar grb;
	for (grb = 0; grb < 8; grb = grb + 1) begin : g_rb1_gp4
		wire [3:0] g4 = rb1_g1[4*grb +: 4];
		wire [3:0] p4 = rb1_p1[4*grb +: 4];
		assign rb1_p4[grb] = &p4;
		assign rb1_g4[grb] = g4[3] | (p4[3] & g4[2]) | (p4[3] & p4[2] & g4[1]) | (p4[3] & p4[2] & p4[1] & g4[0]);
	end
	wire [2:0] rb1_g8 = {
		rb1_g4[5] | (rb1_p4[5] & rb1_g4[4]),
		rb1_g4[3] | (rb1_p4[3] & rb1_g4[2]),
		rb1_g4[1] | (rb1_p4[1] & rb1_g4[0])
	};
	wire [2:0] rb1_p8 = {rb1_p4[5] & rb1_p4[4], rb1_p4[3] & rb1_p4[2], rb1_p4[1] & rb1_p4[0]};
	wire [3:0] rb1_cin8 = {
		rb1_g8[2] | (rb1_p8[2] & rb1_g8[1]) | (rb1_p8[2] & rb1_p8[1] & rb1_g8[0]),
		rb1_g8[1] | (rb1_p8[1] & rb1_g8[0]),
		rb1_g8[0],
		1'b0
	};
	wire [31:0] rb1_s0, rb1_s1;
	wire [7:0] rb1_c4_s0, rb1_c4_s1, rb1_u_s0, rb1_u_s1;
	for (grb = 0; grb < 4; grb = grb + 1) begin : g_rb1_add
		am_add #(.W(4)) u_bl0 (.a(row_base0_rb1[8*grb +: 4]),   .b(ds_stride_add[8*grb +: 4]),   .ci(1'b0), .s(rb1_s0[8*grb +: 4]),   .co(rb1_c4_s0[grb]));
		am_add #(.W(4)) u_bl1 (.a(row_base0_rb1[8*grb +: 4]),   .b(ds_stride_add[8*grb +: 4]),   .ci(1'b1), .s(rb1_s1[8*grb +: 4]),   .co(rb1_c4_s1[grb]));
		am_add #(.W(4)) u_bh0 (.a(row_base0_rb1[8*grb+4 +: 4]), .b(ds_stride_add[8*grb+4 +: 4]), .ci(1'b0), .s(rb1_s0[8*grb+4 +: 4]), .co(rb1_u_s0[grb]));
		am_add #(.W(4)) u_bh1 (.a(row_base0_rb1[8*grb+4 +: 4]), .b(ds_stride_add[8*grb+4 +: 4]), .ci(1'b1), .s(rb1_s1[8*grb+4 +: 4]), .co(rb1_u_s1[grb]));
		wire c_lo = rb1_cin8[grb];
		wire c_hi = c_lo ? rb1_c4_s1[grb] : rb1_c4_s0[grb];
		assign row_base1_comb[8*grb +: 4]   = c_lo ? rb1_s1[8*grb +: 4]   : rb1_s0[8*grb +: 4];
		assign row_base1_comb[8*grb+4 +: 4] = c_hi ? rb1_s1[8*grb+4 +: 4] : rb1_s0[8*grb+4 +: 4];
	end
	assign rb1_co_unused = (&rb1_u_s0) & (&rb1_u_s1) & rb1_g4[7] & rb1_p4[7] & rb1_g4[6] & rb1_p4[6];
`endif
	assign cur_row_base = {
		r_rep[3] ? row_base1[31:24] : row_base0[31:24],
		r_rep[2] ? row_base1[23:16] : row_base0[23:16],
		r_rep[1] ? row_base1[15:8]  : row_base0[15:8],
		r_rep[0] ? row_base1[7:0]   : row_base0[7:0]
	};
`ifdef SYNTH_KINTEX7
	am_add #(.W(32)) u_rd (.a(cur_row_base), .b(r_xo), .ci(1'b0), .s(row_addr), .co(rd_co_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(32)) u_rd (.a(cur_row_base), .b(r_xo), .ci(1'b0), .s(row_addr), .co(rd_co_unused));
`else
	/* 4+4 split carry-select with flat 8-bit CLA for cur_row_base + r_xo (<= 9 cmos3) */
	wire [31:0] rd_g1 = cur_row_base & r_xo;
	wire [31:0] rd_p1 = cur_row_base | r_xo;
	wire [7:0] rd_g4, rd_p4;
	genvar grd;
	for (grd = 0; grd < 8; grd = grd + 1) begin : g_rd_gp4
		wire [3:0] g4 = rd_g1[4*grd +: 4];
		wire [3:0] p4 = rd_p1[4*grd +: 4];
		assign rd_p4[grd] = &p4;
		assign rd_g4[grd] = g4[3] | (p4[3] & g4[2]) | (p4[3] & p4[2] & g4[1]) | (p4[3] & p4[2] & p4[1] & g4[0]);
	end
	wire [2:0] rd_g8 = {
		rd_g4[5] | (rd_p4[5] & rd_g4[4]),
		rd_g4[3] | (rd_p4[3] & rd_g4[2]),
		rd_g4[1] | (rd_p4[1] & rd_g4[0])
	};
	wire [2:0] rd_p8 = {rd_p4[5] & rd_p4[4], rd_p4[3] & rd_p4[2], rd_p4[1] & rd_p4[0]};
	wire [3:0] rd_cin8 = {
		rd_g8[2] | (rd_p8[2] & rd_g8[1]) | (rd_p8[2] & rd_p8[1] & rd_g8[0]),
		rd_g8[1] | (rd_p8[1] & rd_g8[0]),
		rd_g8[0],
		1'b0
	};
	function [4:0] ds_cla4;
		input [3:0] a, b;
		input ci;
		reg [3:0] p, g, c;
		begin
			p = a ^ b;
			g = a & b;
			c[0] = ci;
			c[1] = g[0] | (p[0] & ci);
			c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & ci);
			c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & ci);
			ds_cla4 = {
				g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (&p & ci),
				p ^ c
			};
		end
	endfunction

	wire [31:0] rd_s0, rd_s1;
	wire [7:0] rd_c4_s0, rd_c4_s1, rd_u_s0, rd_u_s1;
	for (grd = 0; grd < 4; grd = grd + 1) begin : g_rd_add
		wire [4:0] rl0_5 = ds_cla4(cur_row_base[8*grd +: 4],   r_xo[8*grd +: 4],   1'b0);
		wire [4:0] rl1_5 = ds_cla4(cur_row_base[8*grd +: 4],   r_xo[8*grd +: 4],   1'b1);
		wire [4:0] rh0_5 = ds_cla4(cur_row_base[8*grd+4 +: 4], r_xo[8*grd+4 +: 4], 1'b0);
		wire [4:0] rh1_5 = ds_cla4(cur_row_base[8*grd+4 +: 4], r_xo[8*grd+4 +: 4], 1'b1);
		assign rd_s0[8*grd +: 4]   = rl0_5[3:0]; assign rd_c4_s0[grd] = rl0_5[4];
		assign rd_s1[8*grd +: 4]   = rl1_5[3:0]; assign rd_c4_s1[grd] = rl1_5[4];
		assign rd_s0[8*grd+4 +: 4] = rh0_5[3:0]; assign rd_u_s0[grd]  = rh0_5[4];
		assign rd_s1[8*grd+4 +: 4] = rh1_5[3:0]; assign rd_u_s1[grd]  = rh1_5[4];
		wire c_lo = rd_cin8[grd];
		wire c_hi = c_lo ? rd_c4_s1[grd] : rd_c4_s0[grd];
		assign row_addr[8*grd +: 4]   = c_lo ? rd_s1[8*grd +: 4]   : rd_s0[8*grd +: 4];
		assign row_addr[8*grd+4 +: 4] = c_hi ? rd_s1[8*grd+4 +: 4] : rd_s0[8*grd+4 +: 4];
	end
	assign rd_co_unused = (&rd_u_s0) & (&rd_u_s1) & rd_g4[7] & rd_p4[7] & rd_g4[6] & rd_p4[6];
`endif
	/* The entry's 32-bit word of the line and its half (D16), from pre-registered r_bo */
	(* keep = "true" *) wire [31:0] lword;
`ifdef SYNTH_KINTEX7
	wire [LINE_SHIFT-1:2] bo_w_buf [0:7];
	wire [1:0] bo_h_buf;
	genvar g_bo, g_bob;
	generate
	for (g_bo = 0; g_bo < 8; g_bo = g_bo + 1) begin : g_bow
		for (g_bob = 2; g_bob < LINE_SHIFT; g_bob = g_bob + 1) begin : g_bob_bit
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bow (.I0(r_bo[g_bob]), .O(bo_w_buf[g_bo][g_bob]));
		end
		assign lword[4 * g_bo +: 4] = line[32 * bo_w_buf[g_bo] + 4 * g_bo +: 4];
	end
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_boh0 (.I0(r_bo[1]), .O(bo_h_buf[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_boh1 (.I0(r_bo[1]), .O(bo_h_buf[1]));
	endgenerate
	(* keep = "true" *) wire [15:0] stored_lo = {
		d16_buf[8] ? lword[16 * bo_h_buf[1] + 8 +: 8] : lword[15:8],
		d16_buf[6] ? lword[16 * bo_h_buf[0] +: 8]     : lword[7:0]
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [LINE_SHIFT-1:2] bo_w_buf [0:7];
	wire [1:0] bo_h_buf;
	genvar g_bo, g_bob;
	generate
	for (g_bo = 0; g_bo < 8; g_bo = g_bo + 1) begin : g_bow
		for (g_bob = 2; g_bob < LINE_SHIFT; g_bob = g_bob + 1) begin : g_bob_bit
			assign bo_w_buf[g_bo][g_bob] = r_bo[g_bob];
		end
		assign lword[4 * g_bo +: 4] = line[32 * bo_w_buf[g_bo] + 4 * g_bo +: 4];
	end
	assign bo_h_buf[0] = r_bo[1];
	assign bo_h_buf[1] = r_bo[1];
	endgenerate
	(* keep = "true" *) wire [15:0] stored_lo = {
		d16_buf[8] ? lword[16 * bo_h_buf[1] + 8 +: 8] : lword[15:8],
		d16_buf[6] ? lword[16 * bo_h_buf[0] +: 8]     : lword[7:0]
	};
`else
	assign lword = line[32 * r_bo[LINE_SHIFT-1:2] +: 32];
	wire [15:0] stored_lo = d16_buf[6] ? lword[16 * r_bo[1] +: 16] : lword[15:0];
`endif

	/* Pure-LUT keys and stencil +/-1 (0 CARRY4s on stencil, buffered zero-mask on depth keys) */
	wire [32:0] zkey_sm, skey_sm;
	wire zkey_co_unused, skey_co_unused;
	wire [2:0] zl_enz_buf, st_enz_buf;
	wire zl_exp_nz = (test_zl[30:23] != 8'd0);
	wire st_exp_nz = (test_stored[30:23] != 8'd0);
`ifdef SYNTH_KINTEX7
	genvar g_kz;
	generate
	for (g_kz = 0; g_kz < 3; g_kz = g_kz + 1) begin : g_key_enz
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_zenz (.I0(zl_exp_nz), .O(zl_enz_buf[g_kz]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_senz (.I0(st_exp_nz), .O(st_enz_buf[g_kz]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_kz;
	generate
	for (g_kz = 0; g_kz < 3; g_kz = g_kz + 1) begin : g_key_enz
		assign zl_enz_buf[g_kz] = zl_exp_nz;
		assign st_enz_buf[g_kz] = st_exp_nz;
	end
	endgenerate
`else
	assign zl_enz_buf = {3{zl_exp_nz}};
	assign st_enz_buf = {3{st_exp_nz}};
`endif
`ifdef SYNTH_KINTEX7
	am_addsub #(.W(33)) u_zkey_sm (.a(33'd0), .b({2'b00, test_zl[30:0]}), .sub(test_zl[31]), .s(zkey_sm), .co(zkey_co_unused));
	am_addsub #(.W(33)) u_skey_sm (.a(33'd0), .b({2'b00, test_stored[30:0]}), .sub(test_stored[31]), .s(skey_sm), .co(skey_co_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_addsub #(.W(33)) u_zkey_sm (.a(33'd0), .b({2'b00, test_zl[30:0]}), .sub(test_zl[31]), .s(zkey_sm), .co(zkey_co_unused));
	am_addsub #(.W(33)) u_skey_sm (.a(33'd0), .b({2'b00, test_stored[30:0]}), .sub(test_stored[31]), .s(skey_sm), .co(skey_co_unused));
`else
	/* Pure-LUT 3->9->27->33 prefix-OR signed-magnitude conversion (<= 9 cmos3) */
	wire [32:0] z_mag_b = {2'b00, test_zl[30:0]};
	wire [32:0] s_mag_b = {2'b00, test_stored[30:0]};
	wire [32:0] z_or_below, s_or_below;
	wire [10:0] z_or3, s_or3;
	wire [2:0] z_or9, s_or9;
	genvar go3;
	generate
	for (go3 = 0; go3 < 10; go3 = go3 + 1) begin : g_key_or3
		assign z_or3[go3] = |z_mag_b[3 * go3 +: 3];
		assign s_or3[go3] = |s_mag_b[3 * go3 +: 3];
	end
	assign z_or3[10] = z_mag_b[30];
	assign s_or3[10] = s_mag_b[30];
	endgenerate
	assign z_or9[0] = |z_or3[2:0];
	assign z_or9[1] = |z_or3[5:3];
	assign z_or9[2] = |z_or3[8:6];
	assign s_or9[0] = |s_or3[2:0];
	assign s_or9[1] = |s_or3[5:3];
	assign s_or9[2] = |s_or3[8:6];
	wire z_or18 = z_or9[0] | z_or9[1];
	wire s_or18 = s_or9[0] | s_or9[1];
	wire z_or27 = z_or18 | z_or9[2];
	wire s_or27 = s_or18 | s_or9[2];
	genvar gk_bit;
	generate
	for (gk_bit = 0; gk_bit < 33; gk_bit = gk_bit + 1) begin : g_key_sm_bit
		if (gk_bit == 0) begin : gb0
			assign z_or_below[0] = 1'b0;
			assign s_or_below[0] = 1'b0;
		end else begin : gbn
			localparam integer G3 = (gk_bit - 1) / 3;
			localparam integer R3 = (gk_bit - 1) % 3;
			wire z_lo3 = (R3 == 0) ? z_mag_b[3 * G3] : (R3 == 1) ? |z_mag_b[3 * G3 +: 2] : z_or3[G3];
			wire s_lo3 = (R3 == 0) ? s_mag_b[3 * G3] : (R3 == 1) ? |s_mag_b[3 * G3 +: 2] : s_or3[G3];
			wire z_mid =
				(G3 == 0) ? 1'b0 :
				(G3 == 1) ? z_or3[0] :
				(G3 == 2) ? |z_or3[1:0] :
				(G3 == 3) ? z_or9[0] :
				(G3 == 4) ? (z_or9[0] | z_or3[3]) :
				(G3 == 5) ? (z_or9[0] | |z_or3[4:3]) :
				(G3 == 6) ? z_or18 :
				(G3 == 7) ? (z_or18 | z_or3[6]) :
				(G3 == 8) ? (z_or18 | |z_or3[7:6]) :
				(G3 == 9) ? z_or27 :
				(G3 == 10) ? (z_or27 | z_or3[9]) : 1'b0;
			wire s_mid =
				(G3 == 0) ? 1'b0 :
				(G3 == 1) ? s_or3[0] :
				(G3 == 2) ? |s_or3[1:0] :
				(G3 == 3) ? s_or9[0] :
				(G3 == 4) ? (s_or9[0] | s_or3[3]) :
				(G3 == 5) ? (s_or9[0] | |s_or3[4:3]) :
				(G3 == 6) ? s_or18 :
				(G3 == 7) ? (s_or18 | s_or3[6]) :
				(G3 == 8) ? (s_or18 | |s_or3[7:6]) :
				(G3 == 9) ? s_or27 :
				(G3 == 10) ? (s_or27 | s_or3[9]) : 1'b0;
			assign z_or_below[gk_bit] = z_lo3 | z_mid;
			assign s_or_below[gk_bit] = s_lo3 | s_mid;
		end
		assign zkey_sm[gk_bit] = z_mag_b[gk_bit] ^ (test_zl[31]     & z_or_below[gk_bit]);
		assign skey_sm[gk_bit] = s_mag_b[gk_bit] ^ (test_stored[31] & s_or_below[gk_bit]);
	end
	endgenerate
	assign zkey_co_unused = 1'b0;
	assign skey_co_unused = 1'b0;
	reg [32:0] skey_sm_s0_r;
`endif
	wire [32:0] conv_zkey = {
		zl_enz_buf[2] ? zkey_sm[32:22] : 11'd0,
		zl_enz_buf[1] ? zkey_sm[21:11] : 11'd0,
		zl_enz_buf[0] ? zkey_sm[10:0]  : 11'd0
	};
`ifdef SYNTH_KINTEX7
	wire [32:0] conv_skey = {
		st_enz_buf[2] ? skey_sm[32:22] : 11'd0,
		st_enz_buf[1] ? skey_sm[21:11] : 11'd0,
		st_enz_buf[0] ? skey_sm[10:0]  : 11'd0
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [32:0] conv_skey = {
		st_enz_buf[2] ? skey_sm[32:22] : 11'd0,
		st_enz_buf[1] ? skey_sm[21:11] : 11'd0,
		st_enz_buf[0] ? skey_sm[10:0]  : 11'd0
	};
`else
	wire [32:0] conv_skey = {
		st_enz_buf[2] ? skey_sm_s0_r[32:22] : 11'd0,
		st_enz_buf[1] ? skey_sm_s0_r[21:11] : 11'd0,
		st_enz_buf[0] ? skey_sm_s0_r[10:0]  : 11'd0
	};
`endif
	wire [7:0] conv_s_inc, conv_s_dec;
	wire sinc_co_unused, sdec_bo_unused;
`ifdef SYNTH_KINTEX7
	am_add #(.W(8)) u_s_inc (.a(test_stored[31:24]), .b(8'd1), .ci(1'b0), .s(conv_s_inc), .co(sinc_co_unused));
	am_sub #(.W(8)) u_s_dec (.a(test_stored[31:24]), .b(8'd1), .bi(1'b0), .d(conv_s_dec), .bo(sdec_bo_unused));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(8)) u_s_inc (.a(test_stored[31:24]), .b(8'd1), .ci(1'b0), .s(conv_s_inc), .co(sinc_co_unused));
	am_sub #(.W(8)) u_s_dec (.a(test_stored[31:24]), .b(8'd1), .bi(1'b0), .d(conv_s_dec), .bo(sdec_bo_unused));
`else
	/* 2-stage pure-LUT 8-bit +1/-1 for stencil ops (S_TEST0B -> S_TEST1, <= 5 cmos3) */
	wire [7:0] sb_in = test_stored[31:24];
	wire [3:0] sinc_lo = {sb_in[3] ^ (&sb_in[2:0]), sb_in[2] ^ (sb_in[1] & sb_in[0]), sb_in[1] ^ sb_in[0], ~sb_in[0]};
	wire [3:0] sinc_h1 = {sb_in[7] ^ (&sb_in[6:4]), sb_in[6] ^ (sb_in[5] & sb_in[4]), sb_in[5] ^ sb_in[4], ~sb_in[4]};
	wire [3:0] sdec_lo = {sb_in[3] ^ !(|sb_in[2:0]), sb_in[2] ^ !(|sb_in[1:0]), sb_in[1] ^ ~sb_in[0], ~sb_in[0]};
	wire [3:0] sdec_h1 = {sb_in[7] ^ !(|sb_in[6:4]), sb_in[6] ^ !(|sb_in[5:4]), sb_in[5] ^ ~sb_in[4], ~sb_in[4]};
	reg [7:0] sb_s0_inc_r, sb_s0_dec_r, sb_s0_in_r;
	reg sb_s0_lo_all1, sb_s0_lo_all0;
	assign conv_s_inc = {sb_s0_lo_all1 ? sb_s0_inc_r[7:4] : sb_s0_in_r[7:4], sb_s0_inc_r[3:0]};
	assign sinc_co_unused = &sb_in;
	assign conv_s_dec = {sb_s0_lo_all0 ? sb_s0_dec_r[7:4] : sb_s0_in_r[7:4], sb_s0_dec_r[3:0]};
	assign sdec_bo_unused = !(|sb_in);
`endif

	/* 2-stage pure-LUT depth/stencil evaluation (S_TEST2 -> S_TEST2B, 0 CARRY4s, <= 4 LUTs/stage) */
`ifdef MUT_STENCIL_FACE
	wire [4:0] face_sel = ~front_rep;
`else
	wire [4:0] face_sel = front_rep;
`endif
	(* keep = "true", dont_touch = "true" *) reg [7:0] face_ss_ref, face_cm, face_wm;
	(* keep = "true", dont_touch = "true" *) reg [2:0] face_ss_fn;
	(* keep = "true", dont_touch = "true" *) reg [8:0] face_so_lo, face_so_hi;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			face_ss_ref <= 8'd0;
			face_cm     <= 8'd0;
			face_ss_fn  <= 3'd0;
			face_so_lo  <= 9'd0;
			face_so_hi  <= 9'd0;
			face_wm     <= 8'd0;
		end else begin
			face_ss_ref <= face_sel[0] ? sfront[7:0]   : sback[7:0];
			face_cm     <= face_sel[1] ? sfront[15:8]  : sback[15:8];
			face_ss_fn  <= face_sel[1] ? sfront[26:24] : sback[26:24];
			face_so_lo  <= face_sel[2] ? sops[8:0]     : sops[24:16];
			face_so_hi  <= face_sel[3] ? sops[8:0]     : sops[24:16];
			face_wm     <= face_sel[4] ? sfront[23:16] : sback[23:16];
		end
	end

	wire s2_bmin_st_w, s2_bmax_st_w, s2_bmin_zun_w, s2_bmax_zun_w;
	wire s2_lt_32_w, s2_eq_32_w, s2_nan_w, s2_lt_24_w, s2_eq_24_w, s2_lt_16_w, s2_eq_16_w;
	wire s2_s_cmp_w, s2_sch_f_w, s2_sch_dp_w, s2_sch_df_w;
	wire [7:0] s2_swr_f_w, s2_swr_dp_w, s2_swr_df_w;
	atg_rop_ds_eval_s2a #(
		.ENABLE_DEPTH32F(ENABLE_DEPTH32F),
		.ENABLE_DEPTH24_S8(ENABLE_DEPTH24_S8),
		.ENABLE_DEPTH16(ENABLE_DEPTH16)
	) u_eval_s2a (
		.ss_ref(face_ss_ref),
		.cm(face_cm),
		.ss_fn(face_ss_fn),
		.so_lo(face_so_lo),
		.so_hi(face_so_hi),
		.wm(face_wm),
		.bmin(bmin),
		.bmax(bmax),
		.zf(test_zl),
		.stored(test_stored),
		.zun(test_zun),
		.zu24(test_zu24),
		.zu16(test_zu16),
		.zkey(test_zkey),
		.skey(test_skey),
		.s_inc(test_s_inc),
		.s_dec(test_s_dec),
		.bmin_st(s2_bmin_st_w),
		.bmax_st(s2_bmax_st_w),
		.bmin_zun(s2_bmin_zun_w),
		.bmax_zun(s2_bmax_zun_w),
		.lt_32(s2_lt_32_w),
		.eq_32(s2_eq_32_w),
		.nan_32(s2_nan_w),
		.lt_24(s2_lt_24_w),
		.eq_24(s2_eq_24_w),
		.lt_16(s2_lt_16_w),
		.eq_16(s2_eq_16_w),
		.s_cmp(s2_s_cmp_w),
		.swr_fail(s2_swr_f_w),
		.swr_dpass(s2_swr_dp_w),
		.swr_dfail(s2_swr_df_w),
		.sch_fail(s2_sch_f_w),
		.sch_dpass(s2_sch_dp_w),
		.sch_dfail(s2_sch_df_w)
	);

	reg s2_bmin_st, s2_bmax_st, s2_bmin_zun, s2_bmax_zun;
	reg s2_lt_32, s2_eq_32, s2_nan, s2_lt_24, s2_eq_24, s2_lt_16, s2_eq_16;
	reg s2_s_cmp, s2_sch_f, s2_sch_dp, s2_sch_df;
	reg [7:0] s2_swr_f, s2_swr_dp, s2_swr_df;
	reg [31:0] s2_zf;
	reg [23:0] s2_zu24;
	reg [15:0] s2_zu16;

	wire eval_pass;
	wire [31:0] eval_wword;
	wire [3:0] eval_wbe;
	atg_rop_ds_eval_s2b #(
		.ENABLE_DEPTH32F(ENABLE_DEPTH32F),
		.ENABLE_DEPTH24_S8(ENABLE_DEPTH24_S8),
		.ENABLE_DEPTH16(ENABLE_DEPTH16)
	) u_eval_s2b (
		.depth(depth),
		.zf(s2_zf),
		.zu24(s2_zu24),
		.zu16(s2_zu16),
		.bmin_st(s2_bmin_st),
		.bmax_st(s2_bmax_st),
		.bmin_zun(s2_bmin_zun),
		.bmax_zun(s2_bmax_zun),
		.lt_32(s2_lt_32),
		.eq_32(s2_eq_32),
		.nan_32(s2_nan),
		.lt_24(s2_lt_24),
		.eq_24(s2_eq_24),
		.lt_16(s2_lt_16),
		.eq_16(s2_eq_16),
		.s_cmp(s2_s_cmp),
		.swr_fail(s2_swr_f),
		.swr_dpass(s2_swr_dp),
		.swr_dfail(s2_swr_df),
		.sch_fail(s2_sch_f),
		.sch_dpass(s2_sch_dp),
		.sch_dfail(s2_sch_df),
		.pass(eval_pass),
		.wword(eval_wword),
		.wbe(eval_wbe)
	);

	/* Registered debug color expansion (fanout of dbg_color_lo/hi <= 16, fanout of dbgf_r <= 4) */
	wire [31:0] dbgf_ch3_lo = rom_unorm8[dbg_color_lo[31:24]];
	wire [31:0] dbgf_ch3_hi = rom_unorm8[dbg_color_hi[31:24]];
	wire [31:0] dbgf_ch2_lo = rom_unorm8[dbg_color_lo[23:16]];
	wire [31:0] dbgf_ch2_hi = rom_unorm8[dbg_color_hi[23:16]];
	wire [31:0] dbgf_ch1_lo = rom_unorm8[dbg_color_lo[15:8]];
	wire [31:0] dbgf_ch1_hi = rom_unorm8[dbg_color_hi[15:8]];
	wire [31:0] dbgf_ch0_lo = rom_unorm8[dbg_color_lo[7:0]];
	wire [31:0] dbgf_ch0_hi = rom_unorm8[dbg_color_hi[7:0]];
	(* keep = "true", dont_touch = "true" *) reg [127:0] dbgf_r;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			dbgf_r <= 128'd0;
		end else begin
			dbgf_r <= {
				dbgf_ch3_hi[31:16], dbgf_ch3_lo[15:0],
				dbgf_ch2_hi[31:16], dbgf_ch2_lo[15:0],
				dbgf_ch1_hi[31:16], dbgf_ch1_lo[15:0],
				dbgf_ch0_hi[31:16], dbgf_ch0_lo[15:0]
			};
		end
	end

	assign q_ready = st == S_IDLE;
	assign busy = st != S_IDLE || d_valid;
	assign req_tag = 8'd0;

	localparam integer N_L16 = LINE_BYTES / 2;
	wire is_s_test3 = (st == S_TEST3);

	/* Per-byte write enable in S_TEST3 using replicated test_bo_rep, test_wbe_rep, test_mask_rep, d16_buf */
	(* keep = "true" *) wire [LINE_BYTES-1:0] s3_byte_we;
	genvar gk;
	generate
	for (gk = 0; gk < LINE_BYTES; gk = gk + 1) begin : g_s3_bwe
		localparam integer G_Q = (gk * 4) / LINE_BYTES;
		wire [LINE_SHIFT-1:0] bo_q = test_bo_rep[G_Q];
		wire [3:0] wbe_q = test_wbe_rep[G_Q];
		wire m_q = test_mask_rep[G_Q];
		wire d16_q = d16_buf[G_Q];
		wire [LINE_SHIFT-1:2] k_w = gk[LINE_SHIFT-1:2];
		wire k_h = gk[1];
		wire [1:0] b_idx = d16_q ? {1'b0, gk[0]} : gk[1:0];
		assign s3_byte_we[gk] = is_s_test3 && m_q && (k_w == bo_q[LINE_SHIFT-1:2]) &&
			(!d16_q || (k_h == bo_q[1])) && wbe_q[b_idx];
	end
	endgenerate

	wire en_row_addr = (st == S_ROW) && (mask[8 * r_rep[4] +: 8] != 8'd0);
	wire en_req_line = (st == S_RD) && !wait_rsp && !req_valid;
	wire [1:0] en_addr_buf, en_rline_buf;

`ifdef SYNTH_KINTEX7
	wire en_idle_qv = (st == S_IDLE) && q_valid;
	wire en_idle_qf = en_idle_qv && q_beat[0];
	wire en_rd_rsp = (st == S_RD) && wait_rsp && rsp_valid;
	wire [4:0] en_col_mid;
	wire [67:0] en_col_leaf;
	wire [2:0] en_qf_mid;
	wire [32:0] en_qf_leaf;
	wire [1:0] en_rd_mid;
	wire [N_L16-1:0] en_rd_leaf;
	wire [31:0] q_dbg_leaf;
	wire [1:0] q_dbg_mid;
	wire [LINE_BYTES-1:0] s3_bwe_leaf;
	genvar g_cm, g_cl, g_cq;
	generate
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_buf_ar
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_adr (.I0(en_row_addr), .O(en_addr_buf[g_cm]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rln (.I0(en_req_line), .O(en_rline_buf[g_cm]));
	end
	for (g_cm = 0; g_cm < 5; g_cm = g_cm + 1) begin : g_mid_col
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mcol (
			.I0(en_idle_qv), .O(en_col_mid[g_cm])
		);
	end
	for (g_cl = 0; g_cl < 68; g_cl = g_cl + 1) begin : g_leaf_col
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lcol (
			.I0(en_col_mid[g_cl / 14]), .O(en_col_leaf[g_cl])
		);
	end
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_mid_dbg
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mdbg (
			.I0(q_dbg), .O(q_dbg_mid[g_cm])
		);
	end
	for (g_cl = 0; g_cl < 32; g_cl = g_cl + 1) begin : g_leaf_dbg
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ldbg (
			.I0(q_dbg_mid[g_cl / 16]), .O(q_dbg_leaf[g_cl])
		);
	end
	wire [511:0] dbg_quad_col = {4{dbgf_r}};
	for (g_cq = 0; g_cq < 32; g_cq = g_cq + 1) begin : g_reg_dcol
		always @(posedge clk) begin
			if (en_col_leaf[g_cq])
				d_color[16 * g_cq +: 16] <= q_dbg_leaf[g_cq] ?
					dbg_quad_col[16 * g_cq +: 16] : q_color[16 * g_cq +: 16];
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				d_color1[16 * g_cq +: 16] <= 16'd0;
			else if (en_col_leaf[32 + g_cq])
				d_color1[16 * g_cq +: 16] <= q_color1[16 * g_cq +: 16];
		end
	end
	for (g_cq = 0; g_cq < 2; g_cq = g_cq + 1) begin : g_reg_dxy
		always @(posedge clk) begin
			if (en_col_leaf[64 + g_cq])
				d_xy[16 * g_cq +: 16] <= q_xy[16 * g_cq +: 16];
			if (en_col_leaf[66 + g_cq])
				ds_xy_int[16 * g_cq +: 16] <= q_xy[16 * g_cq +: 16];
		end
	end

	/* Buffer en_idle_qf for z[511:0], mask[15:0] */
	for (g_cm = 0; g_cm < 3; g_cm = g_cm + 1) begin : g_mid_qf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mqf (
			.I0(en_idle_qf), .O(en_qf_mid[g_cm])
		);
	end
	for (g_cl = 0; g_cl < 33; g_cl = g_cl + 1) begin : g_leaf_qf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lqf (
			.I0(en_qf_mid[g_cl / 11]), .O(en_qf_leaf[g_cl])
		);
	end
	for (g_cq = 0; g_cq < 32; g_cq = g_cq + 1) begin : g_reg_z
		always @(posedge clk) begin
			if (en_qf_leaf[g_cq])
				z[16 * g_cq +: 16] <= q_sdepth[16 * g_cq +: 16];
		end
	end
	always @(posedge clk) begin
		if (en_qf_leaf[32])
			mask <= q_smask;
	end

	/* Buffer en_rd_rsp for line and req_be */
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_mid_rd
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_mrd (
			.I0(en_rd_rsp), .O(en_rd_mid[g_cm])
		);
	end
	for (g_cl = 0; g_cl < N_L16; g_cl = g_cl + 1) begin : g_leaf_rd
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lrd (
			.I0(en_rd_mid[g_cl / (N_L16 / 2)]), .O(en_rd_leaf[g_cl])
		);
	end
	for (g_cq = 0; g_cq < N_L16; g_cq = g_cq + 1) begin : g_reg_line
		always @(posedge clk) begin
			if (en_rd_leaf[g_cq]) begin
				line[16 * g_cq +: 16] <= rsp_rdata[16 * g_cq +: 16];
				req_be[2 * g_cq +: 2] <= 2'b00;
			end else begin
				if (s3_bwe_leaf[2 * g_cq])
					req_be[2 * g_cq] <= 1'b1;
				if (s3_bwe_leaf[2 * g_cq + 1])
					req_be[2 * g_cq + 1] <= 1'b1;
			end
		end
	end

	for (g_cl = 0; g_cl < LINE_BYTES; g_cl = g_cl + 1) begin : g_s3_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_s3b (.I0(s3_byte_we[g_cl]), .O(s3_bwe_leaf[g_cl]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire en_idle_qv = (st == S_IDLE) && q_valid;
	wire en_idle_qf = en_idle_qv && q_beat[0];
	wire en_rd_rsp = (st == S_RD) && wait_rsp && rsp_valid;
	wire [4:0] en_col_mid;
	wire [67:0] en_col_leaf;
	wire [2:0] en_qf_mid;
	wire [32:0] en_qf_leaf;
	wire [1:0] en_rd_mid;
	wire [N_L16-1:0] en_rd_leaf;
	wire [31:0] q_dbg_leaf;
	wire [1:0] q_dbg_mid;
	wire [LINE_BYTES-1:0] s3_bwe_leaf;
	genvar g_cm, g_cl, g_cq;
	generate
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_buf_ar
		assign en_addr_buf[g_cm] = en_row_addr;
		assign en_rline_buf[g_cm] = en_req_line;
	end
	for (g_cm = 0; g_cm < 5; g_cm = g_cm + 1) begin : g_mid_col
		assign en_col_mid[g_cm] = en_idle_qv;
	end
	for (g_cl = 0; g_cl < 68; g_cl = g_cl + 1) begin : g_leaf_col
		assign en_col_leaf[g_cl] = en_col_mid[g_cl / 14];
	end
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_mid_dbg
		assign q_dbg_mid[g_cm] = q_dbg;
	end
	for (g_cl = 0; g_cl < 32; g_cl = g_cl + 1) begin : g_leaf_dbg
		assign q_dbg_leaf[g_cl] = q_dbg_mid[g_cl / 16];
	end
	wire [511:0] dbg_quad_col = {4{dbgf_r}};
	for (g_cq = 0; g_cq < 32; g_cq = g_cq + 1) begin : g_reg_dcol
		always @(posedge clk) begin
			if (en_col_leaf[g_cq])
				d_color[16 * g_cq +: 16] <= q_dbg_leaf[g_cq] ?
					dbg_quad_col[16 * g_cq +: 16] : q_color[16 * g_cq +: 16];
		end
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync)
				d_color1[16 * g_cq +: 16] <= 16'd0;
			else if (en_col_leaf[32 + g_cq])
				d_color1[16 * g_cq +: 16] <= q_color1[16 * g_cq +: 16];
		end
	end
	for (g_cq = 0; g_cq < 2; g_cq = g_cq + 1) begin : g_reg_dxy
		always @(posedge clk) begin
			if (en_col_leaf[64 + g_cq])
				d_xy[16 * g_cq +: 16] <= q_xy[16 * g_cq +: 16];
			if (en_col_leaf[66 + g_cq])
				ds_xy_int[16 * g_cq +: 16] <= q_xy[16 * g_cq +: 16];
		end
	end

	/* Buffer en_idle_qf for z[511:0], mask[15:0] */
	for (g_cm = 0; g_cm < 3; g_cm = g_cm + 1) begin : g_mid_qf
		assign en_qf_mid[g_cm] = en_idle_qf;
	end
	for (g_cl = 0; g_cl < 33; g_cl = g_cl + 1) begin : g_leaf_qf
		assign en_qf_leaf[g_cl] = en_qf_mid[g_cl / 11];
	end
	for (g_cq = 0; g_cq < 32; g_cq = g_cq + 1) begin : g_reg_z
		always @(posedge clk) begin
			if (en_qf_leaf[g_cq])
				z[16 * g_cq +: 16] <= q_sdepth[16 * g_cq +: 16];
		end
	end
	always @(posedge clk) begin
		if (en_qf_leaf[32])
			mask <= q_smask;
	end

	/* Buffer en_rd_rsp for line and req_be */
	for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_mid_rd
		assign en_rd_mid[g_cm] = en_rd_rsp;
	end
	for (g_cl = 0; g_cl < N_L16; g_cl = g_cl + 1) begin : g_leaf_rd
		assign en_rd_leaf[g_cl] = en_rd_mid[g_cl / (N_L16 / 2)];
	end
	for (g_cq = 0; g_cq < N_L16; g_cq = g_cq + 1) begin : g_reg_line
		always @(posedge clk) begin
			if (en_rd_leaf[g_cq]) begin
				line[16 * g_cq +: 16] <= rsp_rdata[16 * g_cq +: 16];
				req_be[2 * g_cq +: 2] <= 2'b00;
			end else begin
				if (s3_bwe_leaf[2 * g_cq])
					req_be[2 * g_cq] <= 1'b1;
				if (s3_bwe_leaf[2 * g_cq + 1])
					req_be[2 * g_cq + 1] <= 1'b1;
			end
		end
	end

	for (g_cl = 0; g_cl < LINE_BYTES; g_cl = g_cl + 1) begin : g_s3_buf
		assign s3_bwe_leaf[g_cl] = s3_byte_we[g_cl];
	end
	endgenerate
`else
	assign en_addr_buf = {2{en_row_addr}};
	assign en_rline_buf = {2{en_req_line}};
	wire [LINE_BYTES-1:0] s3_bwe_leaf = s3_byte_we;
`endif

	wire u24_zero_w = (test_zl[30:23] == 8'd0) || test_zl[31] ||
	                  (test_zl[30:23] == 8'hFF && zl_mant_nz) || (test_zl[30:23] < 8'd102);
	wire u16_zero_w = (test_zl[30:23] == 8'd0) || test_zl[31] ||
	                  (test_zl[30:23] == 8'hFF && zl_mant_nz) || (test_zl[30:23] < 8'd103);
	wire u_max_w = (test_zl[30:23] >= 8'd127);
	wire [31:0] eval_wword_pre = d16_buf[7] ? {eval_wword[15:0], eval_wword[15:0]} : eval_wword;

	wire [63:0] addr_ext = {32'd0, addr};
	/* Free-running datapath registers for row base setup and S_TEST0..S_TEST2B pipeline */
	always @(posedge clk) begin
		r_xo <= xo;
		yrs_r <= yrs_p3;
		row_base0 <= row_a;
		row_base0_rb1 <= row_a;
		row_base1 <= row_base1_comb;

		if (en_addr_buf[0]) addr[15:0]  <= row_addr[15:0];
		if (en_addr_buf[1]) addr[31:16] <= row_addr[31:16];
		if (en_rline_buf[0]) req_line[13:0] <= addr_ext[LINE_SHIFT + 13 : LINE_SHIFT];
		if (en_rline_buf[1]) req_line[ADDR_W - LINE_SHIFT - 1 : 14] <= addr_ext[ADDR_W - 1 : LINE_SHIFT + 14];

		/* S_TEST_PRE latches */
		r_bo <= bo;
		r_sb <= sb;
		r_mask_sb <= mask[sb];

		/* S_TEST0 latches (inputs stationary throughout S_TEST0..S_TEST3) */
		test_stored[15:0] <= stored_lo;
		if (d16_buf[9])
			test_stored[23:16] <= 8'd0;
		else
			test_stored[23:16] <= lword[23:16];
		if (d16_buf[10])
			test_stored[31:24] <= 8'd0;
		else
			test_stored[31:24] <= lword[31:24];
		test_zl <= zl;
		test_bo_rep[0] <= r_bo;
		test_bo_rep[1] <= r_bo;
		test_bo_rep[2] <= r_bo;
		test_bo_rep[3] <= r_bo;
		test_sb <= r_sb;
		test_mask_rep <= {4{r_mask_sb}};

		/* S_TEST0A latches */
		u24_s0a_p <= u24_p;
		u16_s0a_p <= u16_p;
		u24_s0a_t <= sub30_w;
		zl_s0a_st <= zl_st_w;
		zun_s0a_r <= zun_r[79:8];
		zun_s0a_lz <= zun_lz;
		zun_s0a_found <= {4{zun_found}};

		/* S_TEST0B latches */
		u24_stage0_hi <= u24_sh[47:24];
		u24_stage0_round <= u24_sh[23] && (zl_s0a_st || u24_sh[24]);
		u24_stage0_zero <= {3{u24_zero_w}};
		u24_stage0_max <= {3{u_max_w}};
		u16_stage0_hi <= u16_sh[39:24];
		u16_stage0_round <= u16_sh[23] && (zl_s0a_st || u16_sh[24]);
		u16_stage0_zero <= {2{u16_zero_w}};
		u16_stage0_max <= {2{u_max_w}};
		zun_s0_round <= zun_mn[47];
		zun_s0_found <= zun_s0a_found;
		zun_s0_mn_hi <= zun_mn[70:48];
		zun_s0_e0 <= sub126_w;
		zun_s0_e1 <= {3'b011, ~zun_s0a_lz};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		sb_s0_in_r <= sb_in;
		sb_s0_lo_all1 <= &sb_in[3:0];
		sb_s0_lo_all0 <= !(|sb_in[3:0]);
		sb_s0_inc_r <= {sinc_h1, sinc_lo};
		sb_s0_dec_r <= {sdec_h1, sdec_lo};
		skey_sm_s0_r <= skey_sm;
`endif

		/* S_TEST1 latches */
		test_zun <= zun_final;
		test_zu24 <= u24_final;
		test_zu16 <= u16_final;
		test_zkey <= conv_zkey;
		test_skey <= conv_skey;
		test_s_inc <= conv_s_inc;
		test_s_dec <= conv_s_dec;

		/* S_TEST2 latches */
		s2_bmin_st  <= s2_bmin_st_w;
		s2_bmax_st  <= s2_bmax_st_w;
		s2_bmin_zun <= s2_bmin_zun_w;
		s2_bmax_zun <= s2_bmax_zun_w;
		s2_lt_32    <= s2_lt_32_w;
		s2_eq_32    <= s2_eq_32_w;
		s2_nan      <= s2_nan_w;
		s2_lt_24    <= s2_lt_24_w;
		s2_eq_24    <= s2_eq_24_w;
		s2_lt_16    <= s2_lt_16_w;
		s2_eq_16    <= s2_eq_16_w;
		s2_s_cmp    <= s2_s_cmp_w;
		s2_swr_f    <= s2_swr_f_w;
		s2_swr_dp   <= s2_swr_dp_w;
		s2_swr_df   <= s2_swr_df_w;
		s2_sch_f    <= s2_sch_f_w;
		s2_sch_dp   <= s2_sch_dp_w;
		s2_sch_df   <= s2_sch_df_w;
		s2_zf       <= test_zl;
		s2_zu24     <= test_zu24;
		s2_zu16     <= test_zu16;

		/* S_TEST2B latches */
		test_wword_rep[0] <= eval_wword_pre;
		test_wword_rep[1] <= eval_wword_pre;
		test_pass <= eval_pass;
		test_wbe_rep[0] <= eval_wbe;
		test_wbe_rep[1] <= eval_wbe;
		test_wbe_rep[2] <= eval_wbe;
		test_wbe_rep[3] <= eval_wbe;

		/* S_TEST3 byte writes (each s3_bwe_leaf[k] drives 8 data bits) */
		for (k = 0; k < LINE_BYTES; k = k + 1) begin
			if (s3_bwe_leaf[k])
				req_wdata[8 * k +: 8] <= test_wword_rep[(k * 2) / LINE_BYTES][8 * (k % 4) +: 8];
		end
	end

	/* Rule 1 fix: decouple st[2:1] from d_ready, and compute st[0], st[3], d_valid next-state
	 * via explicit single LUT4s on d_ready (so wb/cached_tag_x0/1 -> d_ready (3 LUTs) + 1 LUT = 4 LUTs) */
	reg [4:0] ds_st_next_nodr;
	reg [1:0] r_row_nz;
	reg r_e_last;
	always @(*) begin
		ds_st_next_nodr = st;
		case (st)
		S_IDLE: begin
			if (q_valid) begin
`ifdef MUT_DS_BEAT
				if (1'b0)
`else
				if (!q_beat[0])
`endif
					ds_st_next_nodr = S_OUT;
				else
					ds_st_next_nodr = (q_smask == 16'd0) ? S_IDLE : (test ? S_ROW_SETUP : S_OUT);
			end else begin
				ds_st_next_nodr = S_IDLE;
			end
		end
		S_ROW_SETUP:  ds_st_next_nodr = S_ROW_SETUP1;
		S_ROW_SETUP1: ds_st_next_nodr = S_ROW_SETUP2;
		S_ROW_SETUP2: ds_st_next_nodr = S_ROW_SETUP3;
		S_ROW_SETUP3: ds_st_next_nodr = S_ROW_SETUP4;
		S_ROW_SETUP4: ds_st_next_nodr = S_ROW;
		S_ROW: begin
			if (r_row_nz[r_rep[4]])
				ds_st_next_nodr = S_RD;
			else if (r_rep[4])
				ds_st_next_nodr = S_OUT;
			else
				ds_st_next_nodr = S_ROW;
		end
		S_RD:       ds_st_next_nodr = (wait_rsp && rsp_valid) ? S_TEST_PRE : S_RD;
		S_TEST_PRE: ds_st_next_nodr = S_TEST0;
		S_TEST0:    ds_st_next_nodr = S_TEST0A;
		S_TEST0A:   ds_st_next_nodr = S_TEST0B;
		S_TEST0B:   ds_st_next_nodr = S_TEST1;
		S_TEST1:    ds_st_next_nodr = S_TEST2;
		S_TEST2:    ds_st_next_nodr = S_TEST2B;
		S_TEST2B:   ds_st_next_nodr = S_TEST3;
		S_TEST3:    ds_st_next_nodr = r_e_last ? S_WR : S_TEST_PRE;
		S_WR: begin
			if (!wr_any || (wait_rsp && rsp_valid))
				ds_st_next_nodr = r_rep[4] ? S_OUT : S_ROW;
			else
				ds_st_next_nodr = S_WR;
		end
		default: /* S_OUT when !d_valid */
			ds_st_next_nodr = (pm == 16'd0) ? S_IDLE : S_OUT;
		endcase
	end

	(* keep = "true" *) wire ds_st0_base = ds_st_next_nodr[0];
	(* keep = "true" *) wire ds_st3_base = ds_st_next_nodr[3];
	(* keep = "true" *) wire ds_dval_set = (st == S_OUT) && (pm != 16'd0);
	wire ds_st0_next, ds_st3_next, ds_dval_next;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_st0 (
		.I0(ds_st0_base), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_st0_next)
	);
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_st3 (
		.I0(ds_st3_base), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_st3_next)
	);
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_dval (
		.I0(ds_dval_set), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_dval_next)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_st0 (
		.I0(ds_st0_base), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_st0_next)
	);
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_st3 (
		.I0(ds_st3_base), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_st3_next)
	);
	(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'h222E)) u_ds_dval (
		.I0(ds_dval_set), .I1(d_valid), .I2(d_ready), .I3(rst_sync), .O(ds_dval_next)
	);
`else
	assign ds_st0_next  = d_valid ? !d_ready : ds_st0_base;
	assign ds_st3_next  = d_valid ? !d_ready : ds_st3_base;
	assign ds_dval_next = d_valid ? !d_ready : ds_dval_set;
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			st      <= 5'd0;
			d_valid <= 1'b0;
		end else begin
			st[0]   <= ds_st0_next;
			st[3]   <= ds_st3_next;
			st[4]   <= ds_st_next_nodr[4];
			st[2:1] <= ds_st_next_nodr[2:1];
			d_valid <= ds_dval_next;
		end
	end

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			d_color1 <= 512'd0;
`endif
			d_beat <= 5'd0;
			req_valid <= 1'b0;
			wait_rsp <= 1'b0;
			pm <= 16'd0;
			front_rep <= 5'd0;
			r_rep <= 5'd0;
			wr_any <= 1'b0;
			e <= 3'd0;
			r_row_nz <= 2'd0;
			r_e_last <= 1'b0;
			req_write <= 1'b0;
			d_smask <= 16'd0;
		end else begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			if (st == S_TEST3)
				req_be <= req_be | s3_bwe_leaf;
`endif
			case (st)
			S_IDLE:
				if (q_valid) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					d_xy <= q_xy;
					ds_xy_int <= q_xy;
					d_color <= q_dbg ? {4{dbgf_r}} : q_color;
					d_color1 <= q_color1;
`endif
					d_beat <= q_beat;
`ifdef MUT_DS_BEAT
					if (1'b1) begin
`else
					if (q_beat[0]) begin
`endif
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						mask <= q_smask;
						r_row_nz <= {|q_smask[15:8], |q_smask[7:0]};
						z <= q_sdepth;
`endif
						pm <= q_smask;
						front_rep <= {5{q_front || q_dbg}};
						r_rep <= 5'd0;
					end
				end
			S_ROW_SETUP:  begin end
			S_ROW_SETUP1: begin end
			S_ROW_SETUP2: begin end
			S_ROW_SETUP3: begin end
			S_ROW_SETUP4: begin end
			S_ROW:
				/* the row's address; skip a row without lanes */
				if (mask[8 * r_rep[4] +: 8] == 8'd0 && !r_rep[4])
					r_rep <= 5'h1F;
			S_RD:
				if (!wait_rsp) begin
					if (!req_valid) begin
						req_valid <= 1'b1;
						req_write <= 1'b0;
					end else if (req_ready) begin
						req_valid <= 1'b0;
						wait_rsp <= 1'b1;
					end
				end else if (rsp_valid) begin
					wait_rsp <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					line <= rsp_rdata;
					req_be <= {LINE_BYTES{1'b0}};
`endif
					wr_any <= 1'b0;
					e <= 3'd0;
					r_e_last <= 1'b0;
				end
			S_TEST_PRE: begin end
			S_TEST0:    begin end
			S_TEST0A: begin end
			S_TEST0B: begin end
			S_TEST1:  begin end
			S_TEST2:  begin end
			S_TEST2B: begin end
			S_TEST3: begin
				if (test_mask_rep[0]) begin
					if (!test_pass)
						pm[test_sb] <= 1'b0;
					if (test_wbe_rep[0] != 4'd0)
						wr_any <= 1'b1;
				end
				e <= e_next;
				r_e_last <= (e_next == (ms4_rep[2] ? 3'd7 : 3'd4));
			end
			S_WR:
				if (!wr_any) begin
					r_rep <= 5'h1F;
				end else if (!wait_rsp) begin
					if (!req_valid) begin
						req_valid <= 1'b1;
						req_write <= 1'b1;
					end else if (req_ready) begin
						req_valid <= 1'b0;
						wait_rsp <= 1'b1;
					end
				end else if (rsp_valid) begin
					wait_rsp <= 1'b0;
					wr_any <= 1'b0;
					r_rep <= 5'h1F;
				end
			default: begin	/* S_OUT */
				if (!d_valid)
					d_smask <= pm;
			end
			endcase
		end
	end

	wire unused_ok = &{1'b0, rsp_write, rsp_tag, rsp_error, STENCIL != 0, WPL != 0, WB != 0, bo[0], r_bo[0], unused_yrs_hi,
		ra_co_unused, rb1_co_unused, rd_co_unused, bo_co_unused,
		sub30_bo_unused, sub126_bo_unused,
		u24_p_bo, u16_p_bo, u24_r_co, u16_r_co, u24_sh[22:0], u16_sh[22:0],
		zun_mn[71], zun_mn[46:0], zun_carry_unused, sinc_co_unused, sdec_bo_unused,
		zkey_co_unused, skey_co_unused, addr[LINE_SHIFT-1:0],
		dbgf_ch3_lo[31:16], dbgf_ch3_hi[15:0], dbgf_ch2_lo[31:16], dbgf_ch2_hi[15:0],
		dbgf_ch1_lo[31:16], dbgf_ch1_hi[15:0], dbgf_ch0_lo[31:16], dbgf_ch0_hi[15:0],
		test_bo_rep[0][0], test_bo_rep[1][0], test_bo_rep[2][0], test_bo_rep[3][0], sops[15:9]};
endmodule

/* Pure-LUT 33-bit signed lt/eq comparator (0 CARRY4s, lt <= 4 LUTs, eq <= 3 LUTs) */
module atg_rop_ds_cmp33 (
	input wire [32:0] a,
	input wire [32:0] b,
	output wire lt,
	output wire eq
);
	/* Flip sign bit [32] so signed comparison becomes unsigned comparison */
	wire [32:0] ua = {~a[32], a[31:0]};
	wire [32:0] ub = {~b[32], b[31:0]};

	(* keep = "true" *) wire [10:0] lt0, eq0;
	genvar i;
	generate
	for (i = 0; i < 11; i = i + 1) begin : g_s0
		wire [2:0] sa = ua[3 * i +: 3];
		wire [2:0] sb = ub[3 * i +: 3];
		assign eq0[i] = (sa == sb);
		assign lt0[i] = (~sa[2] & sb[2]) |
			((sa[2] ~^ sb[2]) & ((~sa[1] & sb[1]) | ((sa[1] ~^ sb[1]) & (~sa[0] & sb[0]))));
	end
	endgenerate

	(* keep = "true" *) wire [3:0] lt1, eq1;
	generate
	for (i = 0; i < 3; i = i + 1) begin : g_s1
		assign eq1[i] = eq0[3 * i + 2] & eq0[3 * i + 1] & eq0[3 * i];
		assign lt1[i] = lt0[3 * i + 2] | (eq0[3 * i + 2] & lt0[3 * i + 1]) |
			(eq0[3 * i + 2] & eq0[3 * i + 1] & lt0[3 * i]);
	end
	endgenerate
	assign eq1[3] = eq0[10] & eq0[9];
	assign lt1[3] = lt0[10] | (eq0[10] & lt0[9]);

	(* keep = "true" *) wire lt01 = lt1[1] | (eq1[1] & lt1[0]);
	assign lt = lt1[3] | (eq1[3] & lt1[2]) | (eq1[3] & eq1[2] & lt01);
	assign eq = &eq1;
endmodule

/* Stage 2A of depth/stencil test: pure-LUT comparators and stencil op pre-select (0 CARRY4s, <= 4 LUTs) */
module atg_rop_ds_eval_s2a #(
	parameter ENABLE_DEPTH32F = 1,
	parameter ENABLE_DEPTH24_S8 = 1,
	parameter ENABLE_DEPTH16 = 1
) (
	input wire [7:0] ss_ref,
	input wire [7:0] cm,
	input wire [2:0] ss_fn,
	input wire [8:0] so_lo,
	input wire [8:0] so_hi,
	input wire [7:0] wm,
	input wire [31:0] bmin,
	input wire [31:0] bmax,
	input wire [31:0] zf,
	input wire [31:0] stored,
	input wire [31:0] zun,
	input wire [23:0] zu24,
	input wire [15:0] zu16,
	input wire [32:0] zkey,
	input wire [32:0] skey,
	input wire [7:0] s_inc,
	input wire [7:0] s_dec,
	output wire bmin_st,
	output wire bmax_st,
	output wire bmin_zun,
	output wire bmax_zun,
	output wire lt_32,
	output wire eq_32,
	output wire nan_32,
	output wire lt_24,
	output wire eq_24,
	output wire lt_16,
	output wire eq_16,
	output wire s_cmp,
	output wire [7:0] swr_fail,
	output wire [7:0] swr_dpass,
	output wire [7:0] swr_dfail,
	output wire sch_fail,
	output wire sch_dpass,
	output wire sch_dfail
);
	/* 1. depth bounds (pure LUT, 0 LUTs before/after 4-LUT atg_rop_ms_fle) */
	generate
	if (ENABLE_DEPTH32F) begin : g_bounds_st
		atg_rop_ms_fle u_bmin_st  (.a(bmin),   .b(stored), .le(bmin_st));
		atg_rop_ms_fle u_bmax_st  (.a(stored), .b(bmax),   .le(bmax_st));
	end else begin : g_no_bounds_st
		assign bmin_st = 1'b1;
		assign bmax_st = 1'b1;
	end
	endgenerate
	atg_rop_ms_fle u_bmin_zun (.a(bmin),   .b(zun),    .le(bmin_zun));
	atg_rop_ms_fle u_bmax_zun (.a(zun),    .b(bmax),   .le(bmax_zun));

	/* 3. depth compare (pure LUT, <= 4 LUTs directly to FFs) */
	generate
	if (ENABLE_DEPTH32F) begin : g_cmp32
		(* keep = "true" *) wire [3:0] zf_mnz, st_mnz;
		assign zf_mnz[0] = (zf[5:0]   != 6'd0);
		assign zf_mnz[1] = (zf[11:6]  != 6'd0);
		assign zf_mnz[2] = (zf[17:12] != 6'd0);
		assign zf_mnz[3] = (zf[22:18] != 5'd0);
		assign st_mnz[0] = (stored[5:0]   != 6'd0);
		assign st_mnz[1] = (stored[11:6]  != 6'd0);
		assign st_mnz[2] = (stored[17:12] != 6'd0);
		assign st_mnz[3] = (stored[22:18] != 5'd0);
		(* keep = "true" *) wire zf_e0 = (zf[30:26]     == 5'h1F);
		(* keep = "true" *) wire zf_e1 = (zf[25:23]     == 3'b111);
		(* keep = "true" *) wire st_e0 = (stored[30:26] == 5'h1F);
		(* keep = "true" *) wire st_e1 = (stored[25:23] == 3'b111);
		(* keep = "true" *) wire zf_nan = zf_e0 & zf_e1 & (|zf_mnz);
		(* keep = "true" *) wire st_nan = st_e0 & st_e1 & (|st_mnz);
		assign nan_32 = zf_nan | st_nan;

		atg_rop_ds_cmp33 u_dc32 (.a(zkey),          .b(skey),                  .lt(lt_32), .eq(eq_32));
	end else begin : g_no_cmp32
		assign nan_32 = 1'b0;
		assign lt_32 = 1'b0;
		assign eq_32 = 1'b0;
	end

	if (ENABLE_DEPTH24_S8) begin : g_cmp24
		atg_rop_ds_cmp33 u_dc24 (.a({9'd0, zu24}),  .b({9'd0, stored[23:0]}),  .lt(lt_24), .eq(eq_24));
	end else begin : g_no_cmp24
		assign lt_24 = 1'b0;
		assign eq_24 = 1'b0;
	end

	if (ENABLE_DEPTH16) begin : g_cmp16
		atg_rop_ds_cmp33 u_dc16 (.a({17'd0, zu16}), .b({17'd0, stored[15:0]}), .lt(lt_16), .eq(eq_16));
	end else begin : g_no_cmp16
		assign lt_16 = 1'b0;
		assign eq_16 = 1'b0;
	end
	endgenerate

	/* 2. stencil compare & pre-masked stencil write words (<= 9 cmos3 total) */
	wire [7:0] s_a = stored[31:24];
	wire [7:0] lhs = ss_ref & cm;
	wire [7:0] rhs = s_a & cm;
	/* Balanced 2->4->8 OAI3/AOI3 stencil comparator (depth 6 to slt/seq, 9 total to s_cmp) */
	(* keep = "true" *) wire [7:0] sc_eq_raw_k0 = ~(ss_ref & s_a);
	(* keep = "true" *) wire [7:0] sc_eq_raw    = ~(sc_eq_raw_k0 & (ss_ref | s_a));
	(* keep = "true" *) wire [7:0] sc_eq_n      = ~(sc_eq_raw | ~cm);
	(* keep = "true" *) wire [7:0] sc_lt_n      = ~((~ss_ref) & cm & s_a);
	(* keep = "true" *) wire [3:0] sc_lt2, sc_en2;
	genvar gsc;
	generate
	for (gsc = 0; gsc < 4; gsc = gsc + 1) begin : g_sc2
		assign sc_lt2[gsc] = !(sc_lt_n[2*gsc+1] & (sc_eq_n[2*gsc+1] | sc_lt_n[2*gsc]));
		assign sc_en2[gsc] = !(sc_eq_n[2*gsc+1] | sc_eq_n[2*gsc]);
	end
	endgenerate
	(* keep = "true" *) wire [1:0] sc_lt4_n = {
		!(sc_lt2[3] | (sc_en2[3] & sc_lt2[2])),
		!(sc_lt2[1] | (sc_en2[1] & sc_lt2[0]))
	};
	(* keep = "true" *) wire [1:0] sc_en4_n = {
		!(sc_en2[3] & sc_en2[2]),
		!(sc_en2[1] & sc_en2[0])
	};
	(* keep = "true" *) wire slt = !(sc_lt4_n[1] & (sc_en4_n[1] | sc_lt4_n[0]));
	(* keep = "true" *) wire seq = !(sc_en4_n[1] | sc_en4_n[0]);
	wire unused_lhs_rhs = &{1'b0, lhs, rhs};

	reg s_cmp_r;
	always @(*) begin
		case (ss_fn)
		3'd0: s_cmp_r = 1'b0;
		3'd1: s_cmp_r = slt;
		3'd2: s_cmp_r = seq;
		3'd3: s_cmp_r = slt || seq;
		3'd4: s_cmp_r = !slt && !seq;
		3'd5: s_cmp_r = !seq;
		3'd6: s_cmp_r = !slt;
		default: s_cmp_r = 1'b1;
		endcase
	end
	assign s_cmp = s_cmp_r;

	function [3:0] get_snew4;
		input [2:0] op;
		input [3:0] sa;
		input [3:0] sref;
		input [3:0] sinc;
		input [3:0] sdec;
		input is_ff;
		input is_z;
		begin
			case (op)
			3'd0: get_snew4 = sa;
			3'd1: get_snew4 = 4'd0;
			3'd2: get_snew4 = sref;
			3'd3: get_snew4 = is_ff ? 4'hF : sinc;
			3'd4: get_snew4 = is_z  ? 4'd0 : sdec;
			3'd5: get_snew4 = ~sa;
`ifdef MUT_STENCIL_WRAP
			3'd6: get_snew4 = is_ff ? 4'hF : sinc;
			default: get_snew4 = is_z ? 4'd0 : sdec;
`else
			3'd6: get_snew4 = sinc;
			default: get_snew4 = sdec;
`endif
			endcase
		end
	endfunction

	wire sa_ff_raw = (s_a == 8'hFF);
	wire sa_z_raw = (s_a == 8'd0);
`ifdef SYNTH_KINTEX7
	wire sa_ff_lo, sa_ff_hi;
	wire sa_z_lo, sa_z_hi;
	wire [7:0] s_inc_f, s_inc_dp, s_inc_df;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_saffl (.I0(sa_ff_raw), .O(sa_ff_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_saffh (.I0(sa_ff_raw), .O(sa_ff_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sazl (.I0(sa_z_raw), .O(sa_z_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sazh (.I0(sa_z_raw), .O(sa_z_hi));
	genvar gsi;
	generate
	for (gsi = 0; gsi < 8; gsi = gsi + 1) begin : g_sinc_b
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sif (.I0(s_inc[gsi]), .O(s_inc_f[gsi]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sidp (.I0(s_inc[gsi]), .O(s_inc_dp[gsi]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_sidf (.I0(s_inc[gsi]), .O(s_inc_df[gsi]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire sa_ff_lo, sa_ff_hi;
	wire sa_z_lo, sa_z_hi;
	wire [7:0] s_inc_f, s_inc_dp, s_inc_df;
	assign sa_ff_lo = sa_ff_raw;
	assign sa_ff_hi = sa_ff_raw;
	assign sa_z_lo = sa_z_raw;
	assign sa_z_hi = sa_z_raw;
	genvar gsi;
	generate
	for (gsi = 0; gsi < 8; gsi = gsi + 1) begin : g_sinc_b
		assign s_inc_f[gsi] = s_inc[gsi];
		assign s_inc_dp[gsi] = s_inc[gsi];
		assign s_inc_df[gsi] = s_inc[gsi];
	end
	endgenerate
`else
	wire sa_ff_lo = sa_ff_raw;
	wire sa_ff_hi = sa_ff_raw;
	wire sa_z_lo  = sa_z_raw;
	wire sa_z_hi  = sa_z_raw;
	wire [7:0] s_inc_f  = s_inc;
	wire [7:0] s_inc_dp = s_inc;
	wire [7:0] s_inc_df = s_inc;
`endif
	wire [7:0] snew_f = {
		get_snew4(so_hi[2:0], s_a[7:4], ss_ref[7:4], s_inc_f[7:4], s_dec[7:4], sa_ff_hi, sa_z_hi),
		get_snew4(so_lo[2:0], s_a[3:0], ss_ref[3:0], s_inc_f[3:0], s_dec[3:0], sa_ff_lo, sa_z_lo)
	};
	wire [7:0] snew_dp = {
		get_snew4(so_hi[5:3], s_a[7:4], ss_ref[7:4], s_inc_dp[7:4], s_dec[7:4], sa_ff_hi, sa_z_hi),
		get_snew4(so_lo[5:3], s_a[3:0], ss_ref[3:0], s_inc_dp[3:0], s_dec[3:0], sa_ff_lo, sa_z_lo)
	};
	wire [7:0] snew_df = {
		get_snew4(so_hi[8:6], s_a[7:4], ss_ref[7:4], s_inc_df[7:4], s_dec[7:4], sa_ff_hi, sa_z_hi),
		get_snew4(so_lo[8:6], s_a[3:0], ss_ref[3:0], s_inc_df[3:0], s_dec[3:0], sa_ff_lo, sa_z_lo)
	};
	assign swr_fail  = (s_a & ~wm) | (snew_f  & wm);
	assign swr_dpass = (s_a & ~wm) | (snew_dp & wm);
	assign swr_dfail = (s_a & ~wm) | (snew_df & wm);

	wire [1:0] sch_f_n, sch_dp_n, sch_df_n;
	wire [3:0] dx_rep_lo = (ss_ref[3:0] ^ s_a[3:0]) & wm[3:0];
	wire [3:0] dx_rep_hi = (ss_ref[7:4] ^ s_a[7:4]) & wm[7:4];
	wire [3:0] dx_zer_lo = s_a[3:0] & wm[3:0];
	wire [3:0] dx_zer_hi = s_a[7:4] & wm[7:4];
	wire [3:0] dx_inv_lo = wm[3:0];
	wire [3:0] dx_inv_hi = wm[7:4];
	wire nz_z_lo   = |dx_zer_lo, nz_z_hi   = |dx_zer_hi;
	wire nz_rep_lo = |dx_rep_lo, nz_rep_hi = |dx_rep_hi;
	wire nz_inv_lo = |dx_inv_lo, nz_inv_hi = |dx_inv_hi;
	wire nz_inc_lo = wm[0] | (wm[1] & s_a[0]) | (wm[2] & s_a[1] & s_a[0]) | (wm[3] & s_a[2] & s_a[1] & s_a[0]);
	wire nz_dec_lo = wm[0] | (wm[1] & !s_a[0]) | (wm[2] & !s_a[1] & !s_a[0]) | (wm[3] & !s_a[2] & !s_a[1] & !s_a[0]);
	wire nz_inc_hi = (&s_a[3:0]) && (wm[4] | (wm[5] & s_a[4]) | (wm[6] & s_a[5] & s_a[4]) | (wm[7] & s_a[6] & s_a[5] & s_a[4]));
	wire nz_dec_hi = (~(|s_a[3:0])) && (wm[4] | (wm[5] & !s_a[4]) | (wm[6] & !s_a[5] & !s_a[4]) | (wm[7] & !s_a[6] & !s_a[5] & !s_a[4]));
	function get_sch4;
		input [2:0] op;
		input nz_z, nz_rep, nz_inc, nz_dec, nz_inv, is_ff, is_z;
		begin
			case (op)
			3'd0: get_sch4 = 1'b0;
			3'd1: get_sch4 = nz_z;
			3'd2: get_sch4 = nz_rep;
			3'd3: get_sch4 = !is_ff && nz_inc;
			3'd4: get_sch4 = !is_z  && nz_dec;
			3'd5: get_sch4 = nz_inv;
`ifdef MUT_STENCIL_WRAP
			3'd6: get_sch4 = !is_ff && nz_inc;
			default: get_sch4 = !is_z && nz_dec;
`else
			3'd6: get_sch4 = nz_inc;
			default: get_sch4 = nz_dec;
`endif
			endcase
		end
	endfunction
	assign sch_f_n[0]  = get_sch4(so_lo[2:0], nz_z_lo, nz_rep_lo, nz_inc_lo, nz_dec_lo, nz_inv_lo, sa_ff_lo, sa_z_lo);
	assign sch_f_n[1]  = get_sch4(so_hi[2:0], nz_z_hi, nz_rep_hi, nz_inc_hi, nz_dec_hi, nz_inv_hi, sa_ff_hi, sa_z_hi);
	assign sch_dp_n[0] = get_sch4(so_lo[5:3], nz_z_lo, nz_rep_lo, nz_inc_lo, nz_dec_lo, nz_inv_lo, sa_ff_lo, sa_z_lo);
	assign sch_dp_n[1] = get_sch4(so_hi[5:3], nz_z_hi, nz_rep_hi, nz_inc_hi, nz_dec_hi, nz_inv_hi, sa_ff_hi, sa_z_hi);
	assign sch_df_n[0] = get_sch4(so_lo[8:6], nz_z_lo, nz_rep_lo, nz_inc_lo, nz_dec_lo, nz_inv_lo, sa_ff_lo, sa_z_lo);
	assign sch_df_n[1] = get_sch4(so_hi[8:6], nz_z_hi, nz_rep_hi, nz_inc_hi, nz_dec_hi, nz_inv_hi, sa_ff_hi, sa_z_hi);
	assign sch_fail  = |sch_f_n;
	assign sch_dpass = |sch_dp_n;
	assign sch_dfail = |sch_df_n;

	wire unused_sops = &{1'b0, zf[31]};
endmodule

/* Stage 2B of depth/stencil test: final boolean combine and write word assembly (<= 4 LUTs, 0 CARRY4s) */
module atg_rop_ds_eval_s2b #(
	parameter ENABLE_DEPTH32F = 1,
	parameter ENABLE_DEPTH24_S8 = 1,
	parameter ENABLE_DEPTH16 = 1
) (
	input wire [9:0] depth,
	input wire [31:0] zf,
	input wire [23:0] zu24,
	input wire [15:0] zu16,
	input wire bmin_st,
	input wire bmax_st,
	input wire bmin_zun,
	input wire bmax_zun,
	input wire lt_32,
	input wire eq_32,
	input wire nan_32,
	input wire lt_24,
	input wire eq_24,
	input wire lt_16,
	input wire eq_16,
	input wire s_cmp,
	input wire [7:0] swr_fail,
	input wire [7:0] swr_dpass,
	input wire [7:0] swr_dfail,
	input wire sch_fail,
	input wire sch_dpass,
	input wire sch_dfail,
	output wire pass,
	output wire [31:0] wword,
	output wire [3:0] wbe
);
	wire d24_raw = (ENABLE_DEPTH24_S8 != 0) && (depth[7:5] == 3'b001);
	wire d32_raw = (ENABLE_DEPTH32F != 0) && (depth[7:5] == 3'b010);
	wire d16 = (ENABLE_DEPTH16 != 0) && (depth[7:5] == 3'b100);
	wire d24_lo, d24_hi, d32_lo, d32_hi;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_d24l (.I0(d24_raw), .O(d24_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_d24h (.I0(d24_raw), .O(d24_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_d32l (.I0(d32_raw), .O(d32_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_d32h (.I0(d32_raw), .O(d32_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign d24_lo = d24_raw;
	assign d24_hi = d24_raw;
	assign d32_lo = d32_raw;
	assign d32_hi = d32_raw;
`else
	assign d24_lo = d24_raw;
	assign d24_hi = d24_raw;
	assign d32_lo = d32_raw;
	assign d32_hi = d32_raw;
`endif
	wire has = d24_lo || d32_lo || d16;

	/* Level 1: format mux for depth lt/eq/nan, depth fn decode, and b_ok (each <= 6 inputs) */
	wire b_ok = !depth[9] || (d32_raw ? (bmin_st && bmax_st) : (bmin_zun && bmax_zun));
	(* keep = "true" *) wire d_lt = d32_raw ? (!nan_32 && lt_32) : (d24_raw ? lt_24 : lt_16);
	(* keep = "true" *) wire d_eq = d32_raw ? (!nan_32 && eq_32) : (d24_raw ? eq_24 : eq_16);

	wire [2:0] fn = depth[4:2];
	wire d_nan = d32_raw && nan_32;
	(* keep = "true" *) wire allow_lt = !depth[0] || (!d_nan && (fn == 3'd1 || fn == 3'd3 || fn == 3'd5 || fn == 3'd7));
	(* keep = "true" *) wire allow_eq = !depth[0] || (!d_nan && (fn == 3'd2 || fn == 3'd3 || fn == 3'd6 || fn == 3'd7));
	(* keep = "true" *) wire allow_gt = !depth[0] || (d_nan ? (fn == 3'd5 || fn == 3'd7) : (fn == 3'd4 || fn == 3'd5 || fn == 3'd6 || fn == 3'd7));

	/* Level 2: d_ok */
	(* keep = "true" *) wire d_ok = d_lt ? allow_lt : (d_eq ? allow_eq : allow_gt);

	wire s_on = depth[8] && d24_lo;
	wire s_diff     = !s_cmp ? sch_fail : (d_ok ? sch_dpass : sch_dfail);
	wire s_ok = !s_on || s_cmp;

	wire live = has && b_ok;
	assign pass = !has ? 1'b1 : (live && s_ok && d_ok);
	wire d_wr = live && s_ok && d_ok && depth[1];
	wire s_ch = live && s_on && s_diff;
	wire [7:0] wword_hi =
		({8{d32_hi}}                     & zf[31:24]) |
		({8{d24_hi && !s_cmp}}           & swr_fail)  |
		({8{d24_hi &&  s_cmp &&  d_ok}}  & swr_dpass) |
		({8{d24_hi &&  s_cmp && !d_ok}}  & swr_dfail);
	assign wword = {
		wword_hi,
		d32_hi ? zf[23:16] : (d24_hi ? zu24[23:16] : 8'd0),
		d32_lo ? zf[15:0]  : (d24_lo ? zu24[15:0]  : zu16)
	};
	assign wbe = d32_lo ? {4{d_wr}} : (d24_lo ? {s_ch, {3{d_wr}}} : {2'b00, {2{d_wr}}});
endmodule

`default_nettype wire
