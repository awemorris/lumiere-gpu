/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_2d: the blitter's 2D engine (atg_blt routes to it the kicks with
 * BLT_OP[2] = 0): FILL, COPY, TILE (linear to 4x4 tiles) and UNTILE of a
 * rectangle through the blitter's AMB port (atg_mc), one blit per kick,
 * with the state of atg_blt_cfg (STATE 0x000-0x00A, 0x00C-0x00E as the CP
 * writes them, docs/ghci.md sections 8.2 and 8.5): pixels of 8, 16, 32,
 * 64 or 128 bits, blend-over of 32-bit FILL and COPY (atg_blt_blend), the
 * reverse direction of FILL and COPY (the bottom row first, right to
 * left), KEEP (32-bit FILL and COPY: the bytes of each pixel not written),
 * and the clip rectangle.  Other combinations are refused with an error
 * of kind OPCODE.
 *
 * Streaming burst DMA architecture (Phase 030):
 * For COPY, TILE, UNTILE and FILL without blend/RAW smear, a 1 KiB ping-pong
 * buffer (Bank 0 / Bank 1, 512B each) and decoupled prefetch walk decouple
 * source burst reads from destination pipeline writes. Source lines are
 * prefetched into the active filling bank, aligned via a single-cycle barrel
 * shifter on read-out, and written as continuous pipeline bursts without
 * per-segment stop-and-wait roundtrip penalties, achieving 1.0+ GB/s throughput.
 * Blend-over and self-overlapping smear operations fall back to the serial
 * verified path.
 */

`timescale 1ns / 1ps
`default_nettype none


module atg_blt_2d #(
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter ENABLE_SCALE = 1,
	parameter ENABLE_BLEND = 1,
	parameter ENABLE_TILE = 1
) (
	input wire clk,
	input wire rst_n,
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	input wire kick,
	output wire busy,
	output wire [3:0] fsm,
	output reg done,
	output reg error,
	output reg [3:0] error_kind,
	output wire req_valid,
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
	`include "ghci_regs.vh"

	localparam LINE_BITS = LINE_BYTES * 8;
	localparam US = LINE_SHIFT;		/* log2 of the bytes of a line */
	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam [2:0] MAX_SEGS_M1 = (LINE_SHIFT == 5) ? 3'd7 : 3'd3;
	localparam [3:0] SLOT_LIMIT1 = (LINE_SHIFT == 5) ? 4'd15 : 4'd7;
	localparam [3:0] SLOT_LIMIT2 = (LINE_SHIFT == 5) ? 4'd14 : 4'd6;

	/* The state and the operation (atg_blt_cfg). */
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire rst_b2d_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'b01)) u_rst_b2d (.I0(rst_n), .O(rst_b2d_high));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire rst_b2d_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'b01)) u_rst_b2d (.I0(rst_n), .O(rst_b2d_high));
`else
	wire rst_b2d_high = !rst_n;
`endif
	wire [1:0] o_op;
	wire o_blend, o_rev, op_ok;
	wire [2:0] o_psl;
	wire [3:0] o_keep;
	wire [16:0] clip_x0, clip_y0, clip_x1, clip_y1, dx, dy;
	wire [31:0] s_src_xy, dbase_u, sbase_u, dstride_u, sstride_u;
	wire [31:0] src_slice_u, dst_slice_u;
	wire [15:0] depth_u;
	wire [LINE_BITS-1:0] fill_line;
	wire c_op2;
	wire [8:0] c_resolve;

	atg_blt_cfg #(
		.LINE_BYTES(LINE_BYTES),
		.ENABLE_BLEND(ENABLE_BLEND),
		.ENABLE_TILE(ENABLE_TILE)
	) cfg (
		.clk(clk), .rst_n(rst_n), .st_we(st_we), .st_idx(st_idx), .st_wdata(st_wdata),
		.op(o_op), .op2(c_op2), .resolve(c_resolve), .blend(o_blend), .rev(o_rev), .psl(o_psl), .keep(o_keep), .op_ok(op_ok),
		.x0(clip_x0), .y0(clip_y0), .x1(clip_x1), .y1(clip_y1), .dx(dx), .dy(dy),
		.src_xy(s_src_xy), .dbase(dbase_u), .sbase(sbase_u), .dstride(dstride_u),
		.sstride(sstride_u), .src_slice(src_slice_u), .dst_slice(dst_slice_u), .depth(depth_u), .fill_line(fill_line)
	);

	/* The mode of the blit in progress, taken at the kick. */
`ifdef SYNTH_KINTEX7
	reg m_copy;		/* reads a source: COPY, TILE, UNTILE */
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg m_copy_a, m_copy_b;
	reg m_blend;
	reg m_rev;
	reg m_dtile, m_stile;	/* the destination (TILE), the source (UNTILE) is tiled */
	reg [2:0] psl;		/* log2 of the pixel's bytes */
	reg [3:0] m_keep;
	reg m_burst;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg m_copy;		/* reads a source: COPY, TILE, UNTILE */
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg m_copy_a, m_copy_b;
	reg m_blend;
	reg m_rev;
	reg m_dtile, m_stile;	/* the destination (TILE), the source (UNTILE) is tiled */
	reg [2:0] psl;		/* log2 of the pixel's bytes */
	reg [3:0] m_keep;
	reg m_burst;
`else
	reg m_copy;		/* reads a source: COPY, TILE, UNTILE */
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg m_copy_a, m_copy_b;
	reg m_blend;
	reg m_rev;
	reg m_dtile, m_stile;	/* the destination (TILE), the source (UNTILE) is tiled */
	reg [2:0] psl;		/* log2 of the pixel's bytes */
	reg [3:0] m_keep;
	reg m_burst;
	(* keep = "true" *) reg [7:0] m_copy_wd_rep;
	(* keep = "true" *) reg [7:0] m_blend_wd_rep;
	(* keep = "true" *) reg m_copy_ctl_r;
`endif

	/* The pixel offset of pixel x in a row (linear or in 4x4 tiles); its bytes are this << psl. */
	function [ADDR_W-1:0] x_pixels;
		input [16:0] x;
		input tiled;
		x_pixels = tiled ? {{(ADDR_W - 19){1'b0}}, x[16:2], 2'b00, x[1:0]} :
			{{(ADDR_W - 17){1'b0}}, x};
	endfunction

	/* A tile row's bytes, 4 << psl, and three of them, 12 << psl, by table. */
	function [ADDR_W-1:0] tile_bytes;
		input [2:0] pl;
		input three;
		reg [ADDR_W-1:0] t;
		begin
			case (pl)
			3'd0: t = 4;
			3'd1: t = 8;
			3'd2: t = 16;
			3'd3: t = 32;
			3'd4: t = 64;
			3'd5: t = 128;
			3'd6: t = 256;
			default: t = 512;
			endcase
			tile_bytes = three ? {t[ADDR_W-2:0], 1'b0} | t : t;
		end
	endfunction

	localparam [4:0] S_IDLE  = 5'd0;
	localparam [4:0] S_CLIP  = 5'd1;	/* size and source origin */
	localparam [4:0] S_START = 5'd2;	/* the first row and pixel */
	localparam [4:0] S_MUL   = 5'd3;	/* row addresses: y * stride */
	localparam [4:0] S_SA    = 5'd4;	/* segment: the pixel's addresses */
	localparam [4:0] S_SB    = 5'd5;	/* segment: compute room */
	localparam [4:0] S_SB1   = 5'd6;	/* segment: compute sn from rem and room */
	localparam [4:0] S_SC    = 5'd7;	/* segment: its first byte */
	localparam [4:0] S_SD    = 5'd8;	/* segment: lines and shift */
	localparam [4:0] S_CRD   = 5'd9;	/* read the source lines */
	localparam [4:0] S_CWAIT = 5'd10;	/* wait for them */
	localparam [4:0] S_DRD   = 5'd11;	/* blend: read the destination line */
	localparam [4:0] S_DWAIT = 5'd12;	/* wait for it */
	localparam [4:0] S_BLEND = 5'd13;	/* blend the line */
	localparam [4:0] S_WR    = 5'd14;	/* queue the segment's write */
	localparam [4:0] S_DRAIN = 5'd15;	/* wait for every response */
	localparam [4:0] S_BURST = 5'd16;	/* decoupled streaming burst DMA */
	localparam [4:0] S_INIT  = 5'd17;	/* burst setup pipeline */
	localparam [4:0] S_START1 = 5'd18;	/* accumulator trigger */
	localparam [4:0] S_INIT1  = 5'd19;	/* burst setup pipeline stage 1 */
	localparam [4:0] S_SA1    = 5'd20;	/* segment: shift da to pixel units */
	localparam [4:0] S_INIT0  = 5'd21;	/* burst setup intermediate stage */

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ctl;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_mem;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_leg;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_pipe;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ns;
	reg r_busy_q;
	reg is_burst;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_idle, r_st_clip, r_st_clip_b, r_st_clip_b_mux0, r_st_clip_b_mux1, r_st_start, r_st_start1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_mul_a, r_st_mul_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_init_a, r_st_init_b, r_st_init0, r_st_init1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_sa, r_st_sa1, r_st_sb, r_st_sb1, r_st_sc, r_st_sd;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_crd, r_st_cwait, r_st_drd, r_st_dwait, r_st_blend, r_st_wr, r_st_drain, r_st_burst;
	wire state_b4_0, state_b4_1;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sb40 (.I0(state_ctl[4]), .O(state_b4_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sb41 (.I0(state_ctl[4]), .O(state_b4_1));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ctl;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_mem;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_leg;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_pipe;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ns;
	reg r_busy_q;
	reg is_burst;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_idle, r_st_clip, r_st_clip_b, r_st_clip_b_mux0, r_st_clip_b_mux1, r_st_start, r_st_start1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_mul_a, r_st_mul_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_init_a, r_st_init_b, r_st_init0, r_st_init1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_sa, r_st_sa1, r_st_sb, r_st_sb1, r_st_sc, r_st_sd;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_crd, r_st_cwait, r_st_drd, r_st_dwait, r_st_blend, r_st_wr, r_st_drain, r_st_burst;
	wire state_b4_0, state_b4_1;
	assign state_b4_0 = state_ctl[4];
	assign state_b4_1 = state_ctl[4];
`else
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ctl;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_mem;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_leg;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_pipe;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [4:0] state_ns;
	reg r_busy_q;
	reg is_burst;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_idle, r_st_clip, r_st_clip_b, r_st_start, r_st_start1;
	wire r_st_clip_b_mux0 = r_st_clip_b;
	wire r_st_clip_b_mux1 = r_st_clip_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_mul_a, r_st_mul_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_init_a, r_st_init_b, r_st_init0, r_st_init1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_sa, r_st_sa1, r_st_sb, r_st_sb1, r_st_sc, r_st_sd;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_st_crd, r_st_cwait, r_st_drd, r_st_dwait, r_st_blend, r_st_wr, r_st_drain, r_st_burst;
	wire state_b4_0 = state_ctl[4];
	wire state_b4_1 = state_ctl[4];
`endif

	/* The rectangle. */
`ifdef SYNTH_KINTEX7
	reg [16:0] x0, y0, x1, y1;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg [16:0] x0, y0, x1, y1;
`else
	reg [16:0] x0, y0, x1, y1;
`endif
	wire empty;
	reg [16:0] width;
	reg [16:0] rows;	/* rows: left after this one */
	reg [16:0] width_m1;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_mr_px, r_mr_px_m, r_mr_px_hi, r_mr_spx, r_mr_spx_m, r_mr_spx_hi;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg [16:0] r_src_m_dx, r_src_m_dy;
`endif
	reg [16:0] sx0, sy0;	/* the source pixel of x0, y0 */
	reg [16:0] px_row, spx_row;	/* the first pixel of every row */

	/* Row walking: the row's y modulo 4 (tiles); the rows' byte addresses
	 * (base + y stride, then + the step a row) in atg_blt_acc. */
	reg [1:0] ydl, ysl;
	reg [16:0] px, spx;	/* the next pixel (rightmost reversed), its source pixel */
	reg [16:0] rem;		/* pixels left in the row */
	reg rows_is_zero;
	reg r_rows_eq_1;

	/* The segment. */
	reg [ADDR_W-1:0] da, sa;	/* byte addresses of px, spx */
	reg [ADDR_W-1:0] r_d_xb_leg, r_s_xb_leg;
	reg [US-1:0] r_dst_row_px, r_d_px_lo;
	reg [16:0] r_px_next, r_spx_next, r_rem_next;
	reg [7:0] r_sb_eq;
	reg [US:0] sn, snu;	/* pixels, bytes */
	reg [US:0] r_leg_room;
`ifdef SYNTH_KINTEX7
	reg [ADDR_W-1:0] dlo, slo;	/* first destination and source bytes */
`elsif SYNTH_KINTEX_ULTRASCALE
	reg [ADDR_W-1:0] dlo, slo;	/* first destination and source bytes */
`else
	reg [ADDR_W-1:0] dlo, slo;	/* first destination and source bytes */
`endif
	reg [LA_W-1:0] seg_line;
	reg [US-1:0] seg_p0;
	reg [US-1:0] src_off;
	(* keep = "true" *) reg [8*US-1:0] src_off_rep;
	reg [LA_W-1:0] src_n0, src_n1;
	reg n0_slot;
`ifdef SYNTH_KINTEX7
	reg rd_first;
	reg rd_second;	/* the first, a needed second source read is to go */
`elsif SYNTH_KINTEX_ULTRASCALE
	reg rd_first;
	reg rd_second;	/* the first, a needed second source read is to go */
`else
	reg rd_first, rd_second;	/* the first, a needed second source read is to go */
`endif

	/*
	 * =========================================================================
	 * Burst Engine Signals & Registers
	 * =========================================================================
	 */
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	/* 1 KiB RAM (32 slots), written without async reset so Check 5 & memory inference stay clean. */
	reg [LINE_BITS-1:0] pp_ram [0:31];
`endif

	/* Bank Metadata (2 Banks, up to 8 segments each, full D-FF registers) */
	(* ram_style = "registers" *) reg [6:0] meta_dline_0_0 [0:7];
	(* ram_style = "registers" *) reg [6:0] meta_dline_0_1 [0:7];
	(* ram_style = "registers" *) reg [6:0] meta_dline_0_2 [0:7];
	(* ram_style = "registers" *) reg [LA_W-22:0] meta_dline_0_3 [0:7];
	(* ram_style = "registers" *) reg [6:0] meta_dline_1_0 [0:7];
	(* ram_style = "registers" *) reg [6:0] meta_dline_1_1 [0:7];
	(* ram_style = "registers" *) reg [6:0] meta_dline_1_2 [0:7];
	(* ram_style = "registers" *) reg [LA_W-22:0] meta_dline_1_3 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_0 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_1 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_2 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_3 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_4 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_5 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_6 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_0_7 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_0 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_1 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_2 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_3 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_4 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_5 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_6 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_be_1_7 [0:7];
	(* ram_style = "registers" *) reg [US-1:0] meta_off_0 [0:7];
	(* ram_style = "registers" *) reg [US-1:0] meta_off_1 [0:7];
	(* ram_style = "registers" *) reg [3:0] meta_slot0_0 [0:7];
	(* ram_style = "registers" *) reg [3:0] meta_slot0_1 [0:7];
	(* ram_style = "registers" *) reg [3:0] meta_slot1_0 [0:7];
	(* ram_style = "registers" *) reg [3:0] meta_slot1_1 [0:7];
	(* ram_style = "registers" *) reg [7:0] meta_n0slot_0;
	(* ram_style = "registers" *) reg [7:0] meta_n0slot_1;
	reg [7:0] bw_seg_onehot;
	reg [3:0] bank_num_segs_0, bank_num_segs_1;
	reg [2:0] bank_last_seg_0, bank_last_seg_1;
	reg [1:0] bank_is_last;
	reg [1:0] bank_active;
	reg [1:0] bank_closed;
	reg [1:0] bank_ready;
	reg [4:0] bank_rd_pending_0, bank_rd_pending_1;
	reg rows_b_zero;

	/* Burst Walk & Prefetch registers */
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg bw_bank, bw_bank_dup;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg bw_bank, bw_bank_dup;
`else
	reg bw_bank;
	wire bw_bank_dup = bw_bank;
`endif
	reg [2:0] bw_seg_idx;
	reg [3:0] bw_slot_cnt;
	reg bw_sub;
	reg bw_done_all;
	reg bw_walk_done;
	reg [16:0] px_b, spx_b, rem_b;
	reg [16:0] rows_b;
	reg [1:0] bank_avail;
	reg [US:0] rem_line_b;
	reg        rem_line_b_ge8;
	reg [US:0] rem_line_row;
	reg [US:0] r_rem_line_init;
	reg        r_rem_line_init_ge8;
	reg [US-1:0] cur_da_row;
	reg [1:0] ydl_b, ysl_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg ydl_b_is_3_lo, ydl_b_is_3_hi_a, ydl_b_is_3_hi_b, ydl_b_is_3_cur;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg ysl_b_is_3_lo, ysl_b_is_3_hi_a, ysl_b_is_3_hi_b, ysl_b_is_3_cur;
	reg [US:0] r_delta_px;
	reg [ADDR_W-1:0] dst_row_b, src_row_b;
	reg [ADDR_W-1:0] r_dst_back, r_src_back;
	reg [ADDR_W-1:0] r_dst_neg, r_src_neg;
	reg [ADDR_W-1:0] r_tile_step, r_tile_back;
	reg [31:0] r_dstride, r_sstride, r_dbase, r_sbase;
	reg r_seg_last;
	reg [ADDR_W-1:0] r_d_first, r_s_first;
	reg [16:0] r_yd_start, r_ys_start;
	reg [ADDR_W-1:0] r_dst_delta_norm, r_dst_delta_back;
	reg [ADDR_W-1:0] r_src_delta_norm, r_src_delta_back;
	reg [2:0] psl_w;
	reg [US:0] r_line_pixels;
	reg [2:0] r_line_cap3;
	reg [US:0] line_pixels_b;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg rem_b_small, rem_b_small_bsl, rem_b_small_ctl, rem_b_small_rs;
	(* dont_touch = "true", equivalent_register_removal = "no" *) reg [US:0] cur_bst_room, cur_bst_room_b;
	reg [US:0] r_init_bst_room;
	reg [US:0] r_room_last;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg [US:0] r_rl_mask_k;
	reg [ADDR_W-1:0] r_b_dst_delta, r_b_src_delta;
	reg r_bd_bk_neg, r_bd_bk_tile, r_bd_bk_str;
	reg r_bd_nm_neg, r_bd_nm_tile, r_bd_nm_str;
`endif
	wire [US-1:0] cur_b_dst_delta_lo = ydl_b_is_3_lo ? r_dst_delta_back[US-1:0] : r_dst_delta_norm[US-1:0];

	/* Burst segment pipeline registers: Walk -> Stage1B(w) -> Stage2A(g) -> Stage2B(g2) -> Skid */
	reg w_valid;
	reg [ADDR_W-1:0] w_dst_row, w_src_row, w_d_xb, w_s_xb;
	reg [US:0] w_seg_n, w_seg_bytes;
	reg [US-1:0] w_back_lo;
	reg w_seg_last, w_rows_b_zero;

	reg g_valid;
	reg [ADDR_W-1:0] g_da_full, g_sa_full;
	reg [US:0] g_seg_n;
	reg [US:0] g_seg_bytes;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg [US:0] g_rev_off;
`endif
	reg [US-1:0] g_back_lo;
	reg g_seg_last;
	reg g_rows_b_zero;

	reg g2_valid;
	reg [LA_W-1:0] g2_seg_line, g2_src_n0, g2_src_n1;
	reg [US:0] g2_seg_n, g2_seg_bytes, g2_p_end;
	reg [US-1:0] g2_dlo_lo, g2_src_off;
	reg g2_two_lines, g2_n0_slot, g2_seg_last, g2_rows_b_zero;

	/* Stage 2 -> Stage 3 (Issue pipeline registers via am_skid) */
	wire p_valid;
	wire [LA_W-1:0] p_dline;
	wire [US:0] p_seg_n;
	wire [US:0] p_seg_bytes;
	wire [US-1:0] p_dlo_lo;
	wire [US-1:0] p_off;
	wire [LA_W-1:0] p_src_n0;
	wire [LA_W-1:0] p_src_n1;
	wire p_two_lines;
	wire p_n0_slot;
	wire p_seg_last;
	wire p_rows_b_zero;

	/* Burst Write & Commit registers */
`ifdef SYNTH_KINTEX7
	reg br_bank;
	reg [2:0] br_seg_idx;
	reg br_active;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg br_bank;
	reg [2:0] br_seg_idx;
	reg br_active;
`else
	reg br_bank;
	reg [2:0] br_seg_idx;
	reg br_active;
`endif
	reg b_draining;
	reg bst_rd_err;
	reg [4:0] burst_wr_out;

	/* Mux inputs to address and segment logic */
	reg r_bw_seg_last, r_bw_slot_ge_lim1, r_bw_slot_ge_lim2;
	wire bw_slot_ge_lim1 = r_bw_slot_ge_lim1;
	wire bw_slot_ge_lim2 = r_bw_slot_ge_lim2;
	wire bw_slot_full_p = p_two_lines ? bw_slot_ge_lim2 : bw_slot_ge_lim1;
	wire bank_close_cond = (p_seg_last && p_rows_b_zero) || r_bw_seg_last || bw_slot_full_p;
	wire [0:0] slot_param_unused;
	assign slot_param_unused[0] = &{1'b0, SLOT_LIMIT1, SLOT_LIMIT2, bw_slot_cnt, bw_seg_idx};
	wire bank_close_fire_0, bank_close_fire_1, bank_close_fire;
`ifdef SYNTH_KINTEX7
	wire [16:0] ib_buf;
	genvar g_ibi;
	generate
		for (g_ibi = 0; g_ibi < 17; g_ibi = g_ibi + 1) begin : g_ib_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ibb (.I0(is_burst), .O(ib_buf[g_ibi]));
		end
	endgenerate
	wire is_burst_port  = ib_buf[0];
	wire is_burst_fsm   = ib_buf[1];
	wire is_burst_walk  = ib_buf[2];
	wire is_burst_bank0 = ib_buf[3];
	wire is_burst_bank1 = ib_buf[4];
	wire is_burst_bk0   = ib_buf[5];
	wire is_burst_bk1   = ib_buf[6];
	wire is_burst_bk2   = ib_buf[7];
	wire is_burst_bk3   = ib_buf[8];
	wire is_burst_meta0 = ib_buf[9];
	wire is_burst_meta1 = ib_buf[10];
	wire is_burst_ctl0  = ib_buf[11];
	wire is_burst_ctl1  = ib_buf[12];
	wire is_burst_ctl2  = ib_buf[13];
	wire is_burst_ctl3  = ib_buf[14];
	wire is_burst_ctl4  = ib_buf[15];
	wire is_burst_mem   = ib_buf[16];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [16:0] ib_buf;
	genvar g_ibi;
	generate
		for (g_ibi = 0; g_ibi < 17; g_ibi = g_ibi + 1) begin : g_ib_buf
			assign ib_buf[g_ibi] = is_burst;
		end
	endgenerate
	wire is_burst_port  = ib_buf[0];
	wire is_burst_fsm   = ib_buf[1];
	wire is_burst_walk  = ib_buf[2];
	wire is_burst_bank0 = ib_buf[3];
	wire is_burst_bank1 = ib_buf[4];
	wire is_burst_bk0   = ib_buf[5];
	wire is_burst_bk1   = ib_buf[6];
	wire is_burst_bk2   = ib_buf[7];
	wire is_burst_bk3   = ib_buf[8];
	wire is_burst_meta0 = ib_buf[9];
	wire is_burst_meta1 = ib_buf[10];
	wire is_burst_ctl0  = ib_buf[11];
	wire is_burst_ctl1  = ib_buf[12];
	wire is_burst_ctl2  = ib_buf[13];
	wire is_burst_ctl3  = ib_buf[14];
	wire is_burst_ctl4  = ib_buf[15];
	wire is_burst_mem   = ib_buf[16];
`else
	wire is_burst_port  = is_burst;
	wire is_burst_fsm   = is_burst;
	wire is_burst_walk  = is_burst;
	wire is_burst_bank0 = is_burst;
	wire is_burst_bank1 = is_burst;
	wire is_burst_bk0   = is_burst;
	wire is_burst_bk1   = is_burst;
	wire is_burst_bk2   = is_burst;
	wire is_burst_bk3   = is_burst;
	wire is_burst_meta0 = is_burst;
	wire is_burst_meta1 = is_burst;
	wire is_burst_ctl0  = is_burst;
	wire is_burst_ctl1  = is_burst;
	wire is_burst_ctl2  = is_burst;
	wire is_burst_ctl3  = is_burst;
	wire is_burst_ctl4  = is_burst;
	wire is_burst_mem   = is_burst;
`endif

	/*
	 * =========================================================================
	 * Segment Geometry & Addresses: Common Combinational Datapath
	 * =========================================================================
	 */
	wire [ADDR_W-1:0] cur_da_full = d_addr;
	wire [ADDR_W-1:0] cur_sa_full = s_addr;

	/* The segment's geometry (atg_blt_seg, combinational). */
	wire [US:0] seg_n = leg_seg_n;
	wire [ADDR_W-1:0] back, src_base_u;
	reg [ADDR_W-1:0] r_leg_back;
	wire [LA_W-1:0] last_line;
	wire [LINE_BYTES-1:0] seg_be;


	/* verilator lint_off WIDTHTRUNC */
	localparam [US:0] LB_V = LINE_BYTES;
	/* verilator lint_on WIDTHTRUNC */

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt0  (.I0(m_dtile), .O(mdt_px_row));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt1a (.I0(m_dtile), .O(mdt_px_b_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt1c (.I0(m_dtile), .O(mdt_px_b_0b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt1b (.I0(m_dtile), .O(mdt_px_b_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt2  (.I0(m_dtile), .O(mdt_px));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt2h (.I0(m_dtile), .O(mdt_px_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt3a (.I0(m_dtile), .O(mdt_bstep0_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt3b (.I0(m_dtile), .O(mdt_bstep0_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt4a (.I0(m_dtile), .O(mdt_bstep1_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt4b (.I0(m_dtile), .O(mdt_bstep1_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt5  (.I0(m_dtile), .O(mdt_step0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt6  (.I0(m_dtile), .O(mdt_step1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt7  (.I0(m_dtile), .O(mdt_ctl_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt7b (.I0(m_dtile), .O(mdt_ctl_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt8  (.I0(m_dtile), .O(mdt_h));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mdt9  (.I0(m_dtile), .O(mdt_acc));

	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst0  (.I0(m_stile), .O(mst_px_row));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst1a (.I0(m_stile), .O(mst_px_b_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst1c (.I0(m_stile), .O(mst_px_b_0b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst1b (.I0(m_stile), .O(mst_px_b_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst2  (.I0(m_stile), .O(mst_px));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst2h (.I0(m_stile), .O(mst_px_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst3a (.I0(m_stile), .O(mst_bstep0_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst3b (.I0(m_stile), .O(mst_bstep0_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst4a (.I0(m_stile), .O(mst_bstep1_a));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst4b (.I0(m_stile), .O(mst_bstep1_b));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst5  (.I0(m_stile), .O(mst_step0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst6  (.I0(m_stile), .O(mst_step1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst7  (.I0(m_stile), .O(mst_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst8  (.I0(m_stile), .O(mst_h));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mst9  (.I0(m_stile), .O(mst_acc));

	wire mdt_px_row, mdt_px_b_0, mdt_px_b_0b, mdt_px_b_1, mdt_px, mdt_px_hi;
	wire mst_px_row, mst_px_b_0, mst_px_b_0b, mst_px_b_1, mst_px, mst_px_hi;
	wire mdt_px_b = mdt_px_b_0b;
	wire mst_px_b = mst_px_b_0b;
	wire mdt_bstep0_a, mdt_bstep0_b, mdt_bstep1_a, mdt_bstep1_b, mdt_step0, mdt_step1;
	wire mst_bstep0_a, mst_bstep0_b, mst_bstep1_a, mst_bstep1_b, mst_step0, mst_step1;
	wire mdt_ctl_a, mdt_ctl_b, mdt_h, mdt_acc;
	wire mdt_ctl = mdt_ctl_a;
	wire mst_ctl, mst_h, mst_acc;

	wire [2:0] psl_dx0, psl_dx1, psl_sx0, psl_sx1, psl_ctl, psl_leg0, psl_leg1, psl_leg1_b, psl_snu, psl_rbv;
	wire [2:0] pslw_dxb0_a, pslw_dxb0_b, pslw_dxb1;
	wire [2:0] pslw_sxb0_a, pslw_sxb0_b, pslw_sxb1;
	wire [2:0] pslw_gbk0, pslw_ctl0, pslw_ctl1;
	genvar gpsl;
	generate
		for (gpsl = 0; gpsl < 3; gpsl = gpsl + 1) begin : g_psl_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdx0 (.I0(psl[gpsl]),   .O(psl_dx0[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdx1 (.I0(psl[gpsl]),   .O(psl_dx1[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_psx0 (.I0(psl[gpsl]),   .O(psl_sx0[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_psx1 (.I0(psl[gpsl]),   .O(psl_sx1[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pctl (.I0(psl[gpsl]),   .O(psl_ctl[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_plg0 (.I0(psl[gpsl]),   .O(psl_leg0[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_plg1 (.I0(psl[gpsl]),   .O(psl_leg1[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_plg1b(.I0(psl[gpsl]),   .O(psl_leg1_b[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_psnu (.I0(psl[gpsl]),   .O(psl_snu[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_prbv (.I0(psl[gpsl]),   .O(psl_rbv[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wd0a (.I0(psl_w[gpsl]), .O(pslw_dxb0_a[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wd0b (.I0(psl_w[gpsl]), .O(pslw_dxb0_b[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wd1  (.I0(psl_w[gpsl]), .O(pslw_dxb1[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ws0a (.I0(psl_w[gpsl]), .O(pslw_sxb0_a[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ws0b (.I0(psl_w[gpsl]), .O(pslw_sxb0_b[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ws1  (.I0(psl_w[gpsl]), .O(pslw_sxb1[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wg0  (.I0(psl_w[gpsl]), .O(pslw_gbk0[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wct0 (.I0(psl_w[gpsl]), .O(pslw_ctl0[gpsl]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wct1 (.I0(psl_w[gpsl]), .O(pslw_ctl1[gpsl]));
		end
	endgenerate
	wire [2:0] psl_dx = psl_dx0;
	wire [2:0] psl_sx = psl_sx0;
	wire [2:0] psl_leg = psl_leg0;
`elsif SYNTH_KINTEX_ULTRASCALE
	assign mdt_px_row = m_dtile;
	assign mdt_px_b_0 = m_dtile;
	assign mdt_px_b_0b = m_dtile;
	assign mdt_px_b_1 = m_dtile;
	assign mdt_px = m_dtile;
	assign mdt_px_hi = m_dtile;
	assign mdt_bstep0_a = m_dtile;
	assign mdt_bstep0_b = m_dtile;
	assign mdt_bstep1_a = m_dtile;
	assign mdt_bstep1_b = m_dtile;
	assign mdt_step0 = m_dtile;
	assign mdt_step1 = m_dtile;
	assign mdt_ctl_a = m_dtile;
	assign mdt_ctl_b = m_dtile;
	assign mdt_h = m_dtile;
	assign mdt_acc = m_dtile;

	assign mst_px_row = m_stile;
	assign mst_px_b_0 = m_stile;
	assign mst_px_b_0b = m_stile;
	assign mst_px_b_1 = m_stile;
	assign mst_px = m_stile;
	assign mst_px_hi = m_stile;
	assign mst_bstep0_a = m_stile;
	assign mst_bstep0_b = m_stile;
	assign mst_bstep1_a = m_stile;
	assign mst_bstep1_b = m_stile;
	assign mst_step0 = m_stile;
	assign mst_step1 = m_stile;
	assign mst_ctl = m_stile;
	assign mst_h = m_stile;
	assign mst_acc = m_stile;

	wire mdt_px_row, mdt_px_b_0, mdt_px_b_0b, mdt_px_b_1, mdt_px, mdt_px_hi;
	wire mst_px_row, mst_px_b_0, mst_px_b_0b, mst_px_b_1, mst_px, mst_px_hi;
	wire mdt_px_b = mdt_px_b_0b;
	wire mst_px_b = mst_px_b_0b;
	wire mdt_bstep0_a, mdt_bstep0_b, mdt_bstep1_a, mdt_bstep1_b, mdt_step0, mdt_step1;
	wire mst_bstep0_a, mst_bstep0_b, mst_bstep1_a, mst_bstep1_b, mst_step0, mst_step1;
	wire mdt_ctl_a, mdt_ctl_b, mdt_h, mdt_acc;
	wire mdt_ctl = mdt_ctl_a;
	wire mst_ctl, mst_h, mst_acc;

	wire [2:0] psl_dx0, psl_dx1, psl_sx0, psl_sx1, psl_ctl, psl_leg0, psl_leg1, psl_leg1_b, psl_snu, psl_rbv;
	wire [2:0] pslw_dxb0_a, pslw_dxb0_b, pslw_dxb1;
	wire [2:0] pslw_sxb0_a, pslw_sxb0_b, pslw_sxb1;
	wire [2:0] pslw_gbk0, pslw_ctl0, pslw_ctl1;
	genvar gpsl;
	generate
		for (gpsl = 0; gpsl < 3; gpsl = gpsl + 1) begin : g_psl_buf
			assign psl_dx0[gpsl] = psl[gpsl];
			assign psl_dx1[gpsl] = psl[gpsl];
			assign psl_sx0[gpsl] = psl[gpsl];
			assign psl_sx1[gpsl] = psl[gpsl];
			assign psl_ctl[gpsl] = psl[gpsl];
			assign psl_leg0[gpsl] = psl[gpsl];
			assign psl_leg1[gpsl] = psl[gpsl];
			assign psl_leg1_b[gpsl] = psl[gpsl];
			assign psl_snu[gpsl] = psl[gpsl];
			assign psl_rbv[gpsl] = psl[gpsl];
			assign pslw_dxb0_a[gpsl] = psl_w[gpsl];
			assign pslw_dxb0_b[gpsl] = psl_w[gpsl];
			assign pslw_dxb1[gpsl] = psl_w[gpsl];
			assign pslw_sxb0_a[gpsl] = psl_w[gpsl];
			assign pslw_sxb0_b[gpsl] = psl_w[gpsl];
			assign pslw_sxb1[gpsl] = psl_w[gpsl];
			assign pslw_gbk0[gpsl] = psl_w[gpsl];
			assign pslw_ctl0[gpsl] = psl_w[gpsl];
			assign pslw_ctl1[gpsl] = psl_w[gpsl];
		end
	endgenerate
	wire [2:0] psl_dx = psl_dx0;
	wire [2:0] psl_sx = psl_sx0;
	wire [2:0] psl_leg = psl_leg0;
`else
	wire mdt_px_row = m_dtile, mdt_px_b_0 = m_dtile, mdt_px_b_0b = m_dtile, mdt_px_b_1 = m_dtile, mdt_px_b = m_dtile, mdt_px = m_dtile, mdt_px_hi = m_dtile;
	wire mst_px_row = m_stile, mst_px_b_0 = m_stile, mst_px_b_0b = m_stile, mst_px_b_1 = m_stile, mst_px_b = m_stile, mst_px = m_stile, mst_px_hi = m_stile;
	wire mdt_bstep0_a = m_dtile, mdt_bstep0_b = m_dtile, mdt_bstep1_a = m_dtile, mdt_bstep1_b = m_dtile, mdt_step0 = m_dtile, mdt_step1 = m_dtile;
	wire mst_bstep0_a = m_stile, mst_bstep0_b = m_stile, mst_bstep1_a = m_stile, mst_bstep1_b = m_stile, mst_step0 = m_stile, mst_step1 = m_stile;
	wire mdt_ctl = m_dtile, mdt_ctl_a = m_dtile, mdt_ctl_b = m_dtile, mdt_h = m_dtile, mdt_acc = m_dtile;
	wire mst_ctl = m_stile, mst_h = m_stile, mst_acc = m_stile;

	wire [2:0] psl_dx = psl, psl_dx0 = psl, psl_dx1 = psl;
	wire [2:0] psl_sx = psl, psl_sx0 = psl, psl_sx1 = psl;
	wire [2:0] psl_ctl = psl, psl_leg = psl, psl_leg0 = psl, psl_leg1 = psl, psl_leg1_b = psl, psl_snu = psl, psl_rbv = psl;
	wire [2:0] pslw_dxb0_a = psl_w, pslw_dxb0_b = psl_w, pslw_dxb1 = psl_w;
	wire [2:0] pslw_sxb0_a = psl_w, pslw_sxb0_b = psl_w, pslw_sxb1 = psl_w;
	wire [2:0] pslw_gbk0 = psl_w, pslw_ctl0 = psl_w, pslw_ctl1 = psl_w;
`endif
	(* equivalent_register_removal = "no" *) reg [2:0] psl_leg_r;

	wire [US:0] calc_line_pixels = (psl_ctl == 3'd4) ? {4'b0000, LB_V[US:4]} :
	                               (psl_ctl == 3'd3) ? {3'b000, LB_V[US:3]}  :
	                               (psl_ctl == 3'd2) ? {2'b00, LB_V[US:2]}   :
	                               (psl_ctl == 3'd1) ? {1'b0, LB_V[US:1]}    : LB_V;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [US:0] calc_rl_mask_k;
	genvar g_crm;
	generate
		for (g_crm = 0; g_crm < US; g_crm = g_crm + 1) begin : g_calc_rlm
			assign calc_rl_mask_k[g_crm] = |calc_line_pixels[US:g_crm+1];
		end
	endgenerate
	assign calc_rl_mask_k[US] = 1'b0;
`endif

	reg [US:0] r_da_px_leg;
	reg [2:0] r_room_tile3;
	(* keep = "true" *) reg [US:0] r_snu_m1;
	wire [US:0] in_da_px_wire = r_da_px_leg;
	wire use_in_da_px_wire = 1'b1;

	/* Legacy FSM Segment Engine (completely isolated from burst) */
	wire [US:0] leg_room;
	wire [US:0] leg_seg_n;
	wire leg_seg_last;
	atg_blt_seg #(
		.ADDR_W(ADDR_W),
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.USE_SNU_M1_ONLY(1),
		.USE_IN_DA_PX_ONLY(1),
		.USE_ROOM_TILE3_ONLY(1)
	) seg (
		.psl(psl_leg_r), .line_pix(r_line_pixels), .rev(m_rev), .dtile(mdt_ctl), .stile(mst_ctl), .keep(m_keep),
		.da(da[US-1:0]), .px(px[1:0]), .spx(spx[1:0]), .rem(rem),
		.in_room(r_leg_room), .use_in_room(r_st_sb1),
		.in_da_px(in_da_px_wire), .use_in_da_px(use_in_da_px_wire),
		.in_room_tile3(r_room_tile3), .use_room_tile3(1'b1),
		.in_snu_m1(r_snu_m1), .use_snu_m1(1'b1),
		.sn(sn), .snu(snu), .dlo(dlo[US-1:0]), .slo(slo),
		.p0(seg_p0), .seg_n(leg_seg_n), .seg_last(leg_seg_last), .room(leg_room), .back(back), .src_base(src_base_u),
		.last_line(last_line), .be(seg_be)
	);
	wire leg_rem_done = (!(|rem[16:US+1])) && (rem[US:0] <= r_leg_room);
	wire [US:0] leg_sn = leg_rem_done ? rem[US:0] : r_leg_room;
	/* Precompute rem_back_val in S_SA / S_SB so rem carry chain is completely isolated */
	wire [US:0] rem_bn;
	wire rem_bn_co;
`ifdef SYNTH_KINTEX7
	wire rmbn_c0, rmbn_h0, rmbn_h1;
	wire [US-3:0] rmbn_hi0, rmbn_hi1;
	am_sub #(.W(3))    s_leg_rm_0  (.a(rem[2:0]),    .b(3'd0),           .bi(1'b1), .d(rem_bn[2:0]), .bo(rmbn_c0));
	am_sub #(.W(US-2)) s_leg_rm_h0 (.a(rem[US:3]),   .b({(US-2){1'b0}}), .bi(1'b0), .d(rmbn_hi0),    .bo(rmbn_h0));
	am_sub #(.W(US-2)) s_leg_rm_h1 (.a(rem[US:3]),   .b({(US-2){1'b0}}), .bi(1'b1), .d(rmbn_hi1),    .bo(rmbn_h1));
	assign rem_bn[US:3] = rmbn_c0 ? rmbn_hi1 : rmbn_hi0;
	assign rem_bn_co = (rmbn_c0 ? rmbn_h1 : rmbn_h0) & rmbn_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rmbn_c0, rmbn_h0, rmbn_h1;
	wire [US-3:0] rmbn_hi0, rmbn_hi1;
	am_sub #(.W(3))    s_leg_rm_0  (.a(rem[2:0]),    .b(3'd0),           .bi(1'b1), .d(rem_bn[2:0]), .bo(rmbn_c0));
	am_sub #(.W(US-2)) s_leg_rm_h0 (.a(rem[US:3]),   .b({(US-2){1'b0}}), .bi(1'b0), .d(rmbn_hi0),    .bo(rmbn_h0));
	am_sub #(.W(US-2)) s_leg_rm_h1 (.a(rem[US:3]),   .b({(US-2){1'b0}}), .bi(1'b1), .d(rmbn_hi1),    .bo(rmbn_h1));
	assign rem_bn[US:3] = rmbn_c0 ? rmbn_hi1 : rmbn_hi0;
	assign rem_bn_co = (rmbn_c0 ? rmbn_h1 : rmbn_h0) & rmbn_h0;
`else
	assign rem_bn[0] = ~rem[0];
	genvar g_lrm;
	generate
		for (g_lrm = 1; g_lrm <= US; g_lrm = g_lrm + 1) begin : g_rem_bn_fast
			assign rem_bn[g_lrm] = rem[g_lrm] ^ (~(|rem[g_lrm-1:0]));
		end
	endgenerate
	assign rem_bn_co = 1'b0;
`endif
	wire [ADDR_W-1:0] rem_back_val = (psl_leg1 == 3'd4) ? {{(ADDR_W - US - 5){1'b0}}, rem_bn, 4'b0000} :
	                                 (psl_leg1 == 3'd3) ? {{(ADDR_W - US - 4){1'b0}}, rem_bn, 3'b000}  :
	                                 (psl_leg1 == 3'd2) ? {{(ADDR_W - US - 3){1'b0}}, rem_bn, 2'b00}   :
	                                 (psl_leg1 == 3'd1) ? {{(ADDR_W - US - 2){1'b0}}, rem_bn, 1'b0}    :
	                                                      {{(ADDR_W - US - 1){1'b0}}, rem_bn};
	reg [ADDR_W-1:0] r_rem_back_val;
	reg r_rem_hi_zero;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_rem_back_val <= {ADDR_W{1'b0}};
			r_rem_hi_zero  <= 1'b0;
		end else if (r_st_sa || r_st_sb) begin
			r_rem_back_val <= rem_back_val;
			r_rem_hi_zero  <= !(|rem[16:US+1]);
		end
	end

	wire leg_cmp_lo    = (rem[2:0] <= r_leg_room[2:0]);
	wire leg_cmp_hi_lt = (rem[US:3] < r_leg_room[US:3]);
	wire leg_cmp_hi_eq = (rem[US:3] == r_leg_room[US:3]);
	wire leg_rem_done_lo, leg_rem_done_hi;
`ifdef SYNTH_KINTEX7
	LUT4 #(.INIT(16'hF800)) u_lrd_lo (
		.I0(leg_cmp_lo), .I1(leg_cmp_hi_eq), .I2(leg_cmp_hi_lt), .I3(r_rem_hi_zero),
		.O(leg_rem_done_lo)
	);
	LUT4 #(.INIT(16'hF800)) u_lrd_hi (
		.I0(leg_cmp_lo), .I1(leg_cmp_hi_eq), .I2(leg_cmp_hi_lt), .I3(r_rem_hi_zero),
		.O(leg_rem_done_hi)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	LUT4 #(.INIT(16'hF800)) u_lrd_lo (
		.I0(leg_cmp_lo), .I1(leg_cmp_hi_eq), .I2(leg_cmp_hi_lt), .I3(r_rem_hi_zero),
		.O(leg_rem_done_lo)
	);
	LUT4 #(.INIT(16'hF800)) u_lrd_hi (
		.I0(leg_cmp_lo), .I1(leg_cmp_hi_eq), .I2(leg_cmp_hi_lt), .I3(r_rem_hi_zero),
		.O(leg_rem_done_hi)
	);
`else
	assign leg_rem_done_lo = r_rem_hi_zero && (leg_cmp_hi_lt || (leg_cmp_hi_eq && leg_cmp_lo));
	assign leg_rem_done_hi = leg_rem_done_lo;
`endif

	wire [US:0] room_bn;
	wire room_bn_co;
`ifdef SYNTH_KINTEX7
	wire rmb_c0, rmb_h0, rmb_h1;
	wire [US-3:0] rmb_hi0, rmb_hi1;
	am_sub #(.W(3))    s_leg_room_0  (.a(r_leg_room[2:0]),  .b(3'd0),           .bi(1'b1), .d(room_bn[2:0]), .bo(rmb_c0));
	am_sub #(.W(US-2)) s_leg_room_h0 (.a(r_leg_room[US:3]), .b({(US-2){1'b0}}), .bi(1'b0), .d(rmb_hi0),     .bo(rmb_h0));
	am_sub #(.W(US-2)) s_leg_room_h1 (.a(r_leg_room[US:3]), .b({(US-2){1'b0}}), .bi(1'b1), .d(rmb_hi1),     .bo(rmb_h1));
	assign room_bn[US:3] = rmb_c0 ? rmb_hi1 : rmb_hi0;
	assign room_bn_co = (rmb_c0 ? rmb_h1 : rmb_h0) & rmb_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rmb_c0, rmb_h0, rmb_h1;
	wire [US-3:0] rmb_hi0, rmb_hi1;
	am_sub #(.W(3))    s_leg_room_0  (.a(r_leg_room[2:0]),  .b(3'd0),           .bi(1'b1), .d(room_bn[2:0]), .bo(rmb_c0));
	am_sub #(.W(US-2)) s_leg_room_h0 (.a(r_leg_room[US:3]), .b({(US-2){1'b0}}), .bi(1'b0), .d(rmb_hi0),     .bo(rmb_h0));
	am_sub #(.W(US-2)) s_leg_room_h1 (.a(r_leg_room[US:3]), .b({(US-2){1'b0}}), .bi(1'b1), .d(rmb_hi1),     .bo(rmb_h1));
	assign room_bn[US:3] = rmb_c0 ? rmb_hi1 : rmb_hi0;
	assign room_bn_co = (rmb_c0 ? rmb_h1 : rmb_h0) & rmb_h0;
`else
	assign room_bn[0] = ~r_leg_room[0];
	generate
		for (g_lrm = 1; g_lrm <= US; g_lrm = g_lrm + 1) begin : g_room_bn_fast
			assign room_bn[g_lrm] = r_leg_room[g_lrm] ^ (~(|r_leg_room[g_lrm-1:0]));
		end
	endgenerate
	assign room_bn_co = 1'b0;
`endif
	wire [ADDR_W-1:0] room_back_val = (psl_rbv == 3'd4) ? {{(ADDR_W - US - 5){1'b0}}, room_bn, 4'b0000} :
	                                  (psl_rbv == 3'd3) ? {{(ADDR_W - US - 4){1'b0}}, room_bn, 3'b000}  :
	                                  (psl_rbv == 3'd2) ? {{(ADDR_W - US - 3){1'b0}}, room_bn, 2'b00}   :
	                                  (psl_rbv == 3'd1) ? {{(ADDR_W - US - 2){1'b0}}, room_bn, 1'b0}    :
	                                                      {{(ADDR_W - US - 1){1'b0}}, room_bn};
	wire [ADDR_W-1:0] leg_back_val = {
		leg_rem_done_hi ? r_rem_back_val[ADDR_W-1:16] : room_back_val[ADDR_W-1:16],
		leg_rem_done_lo ? r_rem_back_val[15:0]        : room_back_val[15:0]
	};

	/*
	 * =================================================================
	 * Burst Engine: 3-Stage Pipeline (Walk -> Geom/Address -> Issue)
	 * =================================================================
	 */
	/* Stage 1: Walk Stage (Computes only bst_seg_n for coordinate walking) */
	wire [ADDR_W-1:0] d_px_row = x_pixels(px_row, mdt_px_row);
	wire [US-1:0] d_xb_row_comb = (psl_ctl == 3'd4) ? {d_px_row[US-5:0], 4'b0000} :
	                              (psl_ctl == 3'd3) ? {d_px_row[US-4:0], 3'b000}  :
	                              (psl_ctl == 3'd2) ? {d_px_row[US-3:0], 2'b00}   :
	                              (psl_ctl == 3'd1) ? {d_px_row[US-2:0], 1'b0}    : d_px_row[US-1:0];
	reg [US-1:0] r_d_xb_row;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			r_d_xb_row <= {US{1'b0}};
		else if (r_st_start1)
			r_d_xb_row <= d_xb_row_comb;
	end
	wire [US-1:0] d_xb_row = r_d_xb_row;
	wire [US-1:0] cur_da_init;
	wire co_da_init;
`ifdef SYNTH_KINTEX7
	am_add #(.W(US)) a_da_init (.a(dst_row[US-1:0]), .b(d_xb_row), .ci(1'b0), .s(cur_da_init), .co(co_da_init));
`elsif SYNTH_KINTEX_ULTRASCALE
	am_add #(.W(US)) a_da_init (.a(dst_row[US-1:0]), .b(d_xb_row), .ci(1'b0), .s(cur_da_init), .co(co_da_init));
`else
	wire dai_c0, dai_h0, dai_h1;
	wire [US-3:0] dai_s0, dai_s1;
	wire dai_c1_w = dst_row[0] & d_xb_row[0];
	assign cur_da_init[0] = dst_row[0] ^ d_xb_row[0];
	assign cur_da_init[1] = dst_row[1] ^ d_xb_row[1] ^ dai_c1_w;
	assign dai_c0 = (dst_row[1] & d_xb_row[1]) | ((dst_row[1] | d_xb_row[1]) & dai_c1_w);
	am_add #(.W(US-2)) a_da_init_h0 (.a(dst_row[US-1:2]), .b(d_xb_row[US-1:2]), .ci(1'b0), .s(dai_s0),           .co(dai_h0));
	am_add #(.W(US-2)) a_da_init_h1 (.a(dst_row[US-1:2]), .b(d_xb_row[US-1:2]), .ci(1'b1), .s(dai_s1),           .co(dai_h1));
	assign cur_da_init[US-1:2] = dai_c0 ? dai_s1 : dai_s0;
	assign co_da_init = dai_c0 ? dai_h1 : dai_h0;
`endif


	wire [US-1:0] cur_da_row_next;
	wire co_da_row;
	am_add #(.W(US)) a_da_row (.a(cur_da_row), .b(cur_b_dst_delta_lo), .ci(1'b0), .s(cur_da_row_next), .co(co_da_row));

	reg [US:0] r_da_px_init;
	wire [US:0] da_px_init = r_da_px_init;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [4:0] psl_ctl_oh = {psl_ctl == 3'd4, psl_ctl == 3'd3, psl_ctl == 3'd2, psl_ctl == 3'd1, psl_ctl == 3'd0};
	wire [US-1:0] dai_full_0 = {dai_s0, cur_da_init[1:0]};
	wire [US-1:0] dai_full_1 = {dai_s1, cur_da_init[1:0]};
	wire [US:0] da_px_init_0 =
		({(US+1){psl_ctl_oh[4]}} & {5'b00000, dai_full_0[US-1:4]}) |
		({(US+1){psl_ctl_oh[3]}} & {4'b0000,  dai_full_0[US-1:3]}) |
		({(US+1){psl_ctl_oh[2]}} & {3'b000,   dai_full_0[US-1:2]}) |
		({(US+1){psl_ctl_oh[1]}} & {2'b00,    dai_full_0[US-1:1]}) |
		({(US+1){psl_ctl_oh[0]}} & {1'b0,     dai_full_0});
	wire [US:0] da_px_init_1 =
		({(US+1){psl_ctl_oh[4]}} & {5'b00000, dai_full_1[US-1:4]}) |
		({(US+1){psl_ctl_oh[3]}} & {4'b0000,  dai_full_1[US-1:3]}) |
		({(US+1){psl_ctl_oh[2]}} & {3'b000,   dai_full_1[US-1:2]}) |
		({(US+1){psl_ctl_oh[1]}} & {2'b00,    dai_full_1[US-1:1]}) |
		({(US+1){psl_ctl_oh[0]}} & {1'b0,     dai_full_1});
	wire [US:0] da_px_init_comb = dai_c0 ? da_px_init_1 : da_px_init_0;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			r_da_px_init <= {(US+1){1'b0}};
		else if ((r_st_mul_a && ds_idle_b && m_burst) || r_st_init_a)
			r_da_px_init <= da_px_init_comb;
	end
`endif
	wire [US:0] rem_line_init_f, rem_line_init_r;
	wire [1:0] rem_line_init_co;
`ifdef SYNTH_KINTEX7
	wire rli_f_c0, rli_f_h0, rli_f_h1, rli_r_c0, rli_r_h0, rli_r_h1;
	wire [US-3:0] rli_f_hi0, rli_f_hi1, rli_r_hi0, rli_r_hi1;
	am_sub #(.W(3))    s_rli_f_l  (.a(r_line_pixels[2:0]),  .b(da_px_init[2:0]),  .bi(1'b0), .d(rem_line_init_f[2:0]), .bo(rli_f_c0));
	am_sub #(.W(US-2)) s_rli_f_h0 (.a(r_line_pixels[US:3]), .b(da_px_init[US:3]), .bi(1'b0), .d(rli_f_hi0),            .bo(rli_f_h0));
	am_sub #(.W(US-2)) s_rli_f_h1 (.a(r_line_pixels[US:3]), .b(da_px_init[US:3]), .bi(1'b1), .d(rli_f_hi1),            .bo(rli_f_h1));
	assign rem_line_init_f[US:3] = rli_f_c0 ? rli_f_hi1 : rli_f_hi0;
	am_add #(.W(3))    a_rli_r_l  (.a(da_px_init[2:0]),     .b(3'd0),             .ci(1'b1), .s(rem_line_init_r[2:0]), .co(rli_r_c0));
	am_add #(.W(US-2)) a_rli_r_h0 (.a(da_px_init[US:3]),    .b({(US-2){1'b0}}),   .ci(1'b0), .s(rli_r_hi0),            .co(rli_r_h0));
	am_add #(.W(US-2)) a_rli_r_h1 (.a(da_px_init[US:3]),    .b({(US-2){1'b0}}),   .ci(1'b1), .s(rli_r_hi1),            .co(rli_r_h1));
	assign rem_line_init_r[US:3] = rli_r_c0 ? rli_r_hi1 : rli_r_hi0;
	assign rem_line_init_co = {(rli_r_c0 ? rli_r_h1 : rli_r_h0), (rli_f_c0 ? rli_f_h1 : rli_f_h0)};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rli_f_c0, rli_f_h0, rli_f_h1, rli_r_c0, rli_r_h0, rli_r_h1;
	wire [US-3:0] rli_f_hi0, rli_f_hi1, rli_r_hi0, rli_r_hi1;
	am_sub #(.W(3))    s_rli_f_l  (.a(r_line_pixels[2:0]),  .b(da_px_init[2:0]),  .bi(1'b0), .d(rem_line_init_f[2:0]), .bo(rli_f_c0));
	am_sub #(.W(US-2)) s_rli_f_h0 (.a(r_line_pixels[US:3]), .b(da_px_init[US:3]), .bi(1'b0), .d(rli_f_hi0),            .bo(rli_f_h0));
	am_sub #(.W(US-2)) s_rli_f_h1 (.a(r_line_pixels[US:3]), .b(da_px_init[US:3]), .bi(1'b1), .d(rli_f_hi1),            .bo(rli_f_h1));
	assign rem_line_init_f[US:3] = rli_f_c0 ? rli_f_hi1 : rli_f_hi0;
	am_add #(.W(3))    a_rli_r_l  (.a(da_px_init[2:0]),     .b(3'd0),             .ci(1'b1), .s(rem_line_init_r[2:0]), .co(rli_r_c0));
	am_add #(.W(US-2)) a_rli_r_h0 (.a(da_px_init[US:3]),    .b({(US-2){1'b0}}),   .ci(1'b0), .s(rli_r_hi0),            .co(rli_r_h0));
	am_add #(.W(US-2)) a_rli_r_h1 (.a(da_px_init[US:3]),    .b({(US-2){1'b0}}),   .ci(1'b1), .s(rli_r_hi1),            .co(rli_r_h1));
	assign rem_line_init_r[US:3] = rli_r_c0 ? rli_r_hi1 : rli_r_hi0;
	assign rem_line_init_co = {(rli_r_c0 ? rli_r_h1 : rli_r_h0), (rli_f_c0 ? rli_f_h1 : rli_f_h0)};
`else
	assign rem_line_init_f[0] = da_px_init[0];
	assign rem_line_init_r[0] = ~da_px_init[0];
	genvar g_rli;
	generate
		for (g_rli = 1; g_rli < US; g_rli = g_rli + 1) begin : g_rli_fast
			assign rem_line_init_f[g_rli] = (|r_line_pixels[US:g_rli+1]) ?
				(da_px_init[g_rli] ^ (|da_px_init[g_rli-1:0])) :
				((da_px_init[g_rli-1:0] == {g_rli{1'b0}}) & r_line_pixels[g_rli]);
			assign rem_line_init_r[g_rli] = da_px_init[g_rli] ^ (&da_px_init[g_rli-1:0]);
		end
	endgenerate
	assign rem_line_init_f[US] = (da_px_init[US-1:0] == {US{1'b0}}) & r_line_pixels[US];
	assign rem_line_init_r[US] = da_px_init[US] ^ (&da_px_init[US-1:0]);
	assign rem_line_init_co = 2'b00;
`endif
	wire [US:0] rem_line_init = mr_walk_rlr_init ? rem_line_init_r : rem_line_init_f;

`ifdef SYNTH_KINTEX7
	reg r_is_init1;
	reg r_mr_rlr_sub;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg r_is_init1;
	reg r_mr_rlr_sub;
`else
	reg r_is_init1;
`endif
	wire [US:0] delta_px = r_delta_px;
	wire [US:0] rlr_curr = rem_line_row;
	wire [US:0] rlr_sum_w;
	wire [0:0] rlr_co_unused;
`ifdef SYNTH_KINTEX7
	/* Carry-free split 3+(US-2) LUT addition (0 CARRY4; negation folded into r_delta_px at S_START1). */
	wire [2:0] rlr_s_lo;
	wire rlr_s_co0;
	wire [US-3:0] rlr_s_hi0, rlr_s_hi1;
	wire [1:0] rlr_cs_unused;
	am_add #(.W(3))    a_rlr_lo  (.a(rlr_curr[2:0]),    .b(delta_px[2:0]),    .ci(1'b0), .s(rlr_s_lo),  .co(rlr_s_co0));
	am_add #(.W(US-2)) a_rlr_hi0 (.a(rlr_curr[US:3]),   .b(delta_px[US:3]),   .ci(1'b0), .s(rlr_s_hi0), .co(rlr_cs_unused[0]));
	am_add #(.W(US-2)) a_rlr_hi1 (.a(rlr_curr[US:3]),   .b(delta_px[US:3]),   .ci(1'b1), .s(rlr_s_hi1), .co(rlr_cs_unused[1]));
	assign rlr_sum_w = {rlr_s_co0 ? rlr_s_hi1 : rlr_s_hi0, rlr_s_lo};
	assign rlr_co_unused[0] = &{1'b0, rlr_cs_unused};
	wire [US:0] rem_line_row_next_0, rem_line_row_next_1;
	assign rem_line_row_next_0[0] = rlr_s_lo[0];
	assign rem_line_row_next_1[0] = rlr_s_lo[0];
	wire [US:0] rlr_cand_0 = {rlr_s_hi0, rlr_s_lo};
	wire [US:0] rlr_cand_1 = {rlr_s_hi1, rlr_s_lo};
	genvar g_rlr;
	generate
		for (g_rlr = 1; g_rlr < US; g_rlr = g_rlr + 1) begin : g_rlr_bits
			wire is_lp_k = line_pixels_b[g_rlr];
			wire lt_lp_k = |line_pixels_b[US:g_rlr+1];
			wire eq_zero_k0 = (rlr_cand_0[g_rlr-1:0] == {g_rlr{1'b0}});
			wire eq_zero_k1 = (rlr_cand_1[g_rlr-1:0] == {g_rlr{1'b0}});
			assign rem_line_row_next_0[g_rlr] = is_lp_k ? eq_zero_k0 : (lt_lp_k ? rlr_cand_0[g_rlr] : 1'b0);
			assign rem_line_row_next_1[g_rlr] = is_lp_k ? eq_zero_k1 : (lt_lp_k ? rlr_cand_1[g_rlr] : 1'b0);
		end
		assign rem_line_row_next_0[US] = line_pixels_b[US] ? (rlr_cand_0[US-1:0] == {US{1'b0}}) : 1'b0;
		assign rem_line_row_next_1[US] = line_pixels_b[US] ? (rlr_cand_1[US-1:0] == {US{1'b0}}) : 1'b0;
	endgenerate
	wire [US:0] rem_line_row_next = rlr_s_co0 ? rem_line_row_next_1 : rem_line_row_next_0;
	wire [US:0] rlr_cand = rlr_cand_0;
	wire rlr_cand_unused = &{1'b0, rlr_cand};
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Carry-free split 3+(US-2) LUT addition (0 CARRY4; negation folded into r_delta_px at S_START1). */
	wire [2:0] rlr_s_lo;
	wire rlr_s_co0;
	wire [US-3:0] rlr_s_hi0, rlr_s_hi1;
	wire [1:0] rlr_cs_unused;
	am_add #(.W(3))    a_rlr_lo  (.a(rlr_curr[2:0]),    .b(delta_px[2:0]),    .ci(1'b0), .s(rlr_s_lo),  .co(rlr_s_co0));
	am_add #(.W(US-2)) a_rlr_hi0 (.a(rlr_curr[US:3]),   .b(delta_px[US:3]),   .ci(1'b0), .s(rlr_s_hi0), .co(rlr_cs_unused[0]));
	am_add #(.W(US-2)) a_rlr_hi1 (.a(rlr_curr[US:3]),   .b(delta_px[US:3]),   .ci(1'b1), .s(rlr_s_hi1), .co(rlr_cs_unused[1]));
	assign rlr_sum_w = {rlr_s_co0 ? rlr_s_hi1 : rlr_s_hi0, rlr_s_lo};
	assign rlr_co_unused[0] = &{1'b0, rlr_cs_unused};
	wire [US:0] rem_line_row_next_0, rem_line_row_next_1;
	assign rem_line_row_next_0[0] = rlr_s_lo[0];
	assign rem_line_row_next_1[0] = rlr_s_lo[0];
	wire [US:0] rlr_cand_0 = {rlr_s_hi0, rlr_s_lo};
	wire [US:0] rlr_cand_1 = {rlr_s_hi1, rlr_s_lo};
	genvar g_rlr;
	generate
		for (g_rlr = 1; g_rlr < US; g_rlr = g_rlr + 1) begin : g_rlr_bits
			wire is_lp_k = line_pixels_b[g_rlr];
			wire lt_lp_k = |line_pixels_b[US:g_rlr+1];
			wire eq_zero_k0 = (rlr_cand_0[g_rlr-1:0] == {g_rlr{1'b0}});
			wire eq_zero_k1 = (rlr_cand_1[g_rlr-1:0] == {g_rlr{1'b0}});
			assign rem_line_row_next_0[g_rlr] = is_lp_k ? eq_zero_k0 : (lt_lp_k ? rlr_cand_0[g_rlr] : 1'b0);
			assign rem_line_row_next_1[g_rlr] = is_lp_k ? eq_zero_k1 : (lt_lp_k ? rlr_cand_1[g_rlr] : 1'b0);
		end
		assign rem_line_row_next_0[US] = line_pixels_b[US] ? (rlr_cand_0[US-1:0] == {US{1'b0}}) : 1'b0;
		assign rem_line_row_next_1[US] = line_pixels_b[US] ? (rlr_cand_1[US-1:0] == {US{1'b0}}) : 1'b0;
	endgenerate
	wire [US:0] rem_line_row_next = rlr_s_co0 ? rem_line_row_next_1 : rem_line_row_next_0;
	wire [US:0] rlr_cand = rlr_cand_0;
	wire rlr_cand_unused = &{1'b0, rlr_cand};
`else
	/* Direct 2-bit lo2 + 3+(US-2) carry-select + parallel carry-free zero/ge4 cuts a_rlr -> rem_line_row / r_room_last to <= 10 cmos3. */
	wire rlr_c1 = rlr_curr[0] & delta_px[0];
	wire [1:0] rlr_s_lo2 = {rlr_curr[1] ^ delta_px[1] ^ rlr_c1, rlr_curr[0] ^ delta_px[0]};
	wire rlr_c2 = (rlr_curr[1] & delta_px[1]) | ((rlr_curr[1] | delta_px[1]) & rlr_c1);
	wire rlr_c3, rlr_h0, rlr_h1;
	wire [2:0] rlr_lo3;
	wire [US-3:0] rlr_hi0, rlr_hi1;
	am_add #(.W(3))    a_rlr_lo3 (.a(rlr_curr[2:0]),  .b(delta_px[2:0]),  .ci(1'b0), .s(rlr_lo3), .co(rlr_c3));
	am_add #(.W(US-2)) a_rlr_hi0 (.a(rlr_curr[US:3]), .b(delta_px[US:3]), .ci(1'b0), .s(rlr_hi0), .co(rlr_h0));
	am_add #(.W(US-2)) a_rlr_hi1 (.a(rlr_curr[US:3]), .b(delta_px[US:3]), .ci(1'b1), .s(rlr_hi1), .co(rlr_h1));
	assign rlr_sum_w = {rlr_c3 ? rlr_hi1 : rlr_hi0, rlr_lo3};
	assign rlr_co_unused[0] = (rlr_c3 ? rlr_h1 : rlr_h0) & rlr_h0;
	wire [US:0] rlr_ab_x = rlr_curr ^ delta_px;
	wire [US:0] rlr_ab_o = rlr_curr | delta_px;
	wire [US:0] rl_eq0_mm;
	assign rl_eq0_mm[0] = rlr_ab_x[0];
	assign rl_eq0_mm[US:1] = rlr_ab_x[US:1] ^ rlr_ab_o[US-1:0];
	wire [US-2:0] rl_hi0_mm = {rlr_ab_x[US:3] ^ rlr_ab_o[US-1:2], rlr_ab_x[2]};
	wire [US:0] rem_line_row_next;
	assign rem_line_row_next[0] = rlr_s_lo2[0];
	assign rem_line_row_next[1] = line_pixels_b[1] ? (~rlr_s_lo2[0]) : (r_rl_mask_k[1] ? rlr_s_lo2[1] : 1'b0);
	assign rem_line_row_next[2] = (line_pixels_b[2] & (rl_eq0_mm[1:0] == 2'd0)) | (r_rl_mask_k[2] & rlr_lo3[2]);
	genvar g_rlr;
	generate
		for (g_rlr = 3; g_rlr < US; g_rlr = g_rlr + 1) begin : g_rlr_bits
			wire is_lp_k = line_pixels_b[g_rlr];
			wire lt_lp_k = r_rl_mask_k[g_rlr];
			wire eq_zero_k = (rl_eq0_mm[g_rlr-1:0] == {g_rlr{1'b0}});
			assign rem_line_row_next[g_rlr] = (is_lp_k & eq_zero_k) | (lt_lp_k & (rlr_c3 ? rlr_hi1[g_rlr-3] : rlr_hi0[g_rlr-3]));
		end
		assign rem_line_row_next[US] = line_pixels_b[US] ? (rl_eq0_mm[US-1:0] == {US{1'b0}}) : 1'b0;
	endgenerate
	wire rlr_cand_unused = 1'b0;
`endif

	/* m_rev buffer tree (shared across FPGA & ASIC for <= 16 loads per leaf; dont_touch per bit prevents Vivado merge) */
	wire [63:0] r_mr_buf;
	wire [3:0] o_rev_buf;
`ifdef SYNTH_KINTEX7
	genvar g_orb;
	generate
		for (g_orb = 0; g_orb < 4; g_orb = g_orb + 1) begin : g_orev_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_orev (.I0(o_rev), .O(o_rev_buf[g_orb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_orb;
	generate
		for (g_orb = 0; g_orb < 4; g_orb = g_orb + 1) begin : g_orev_buf
			assign o_rev_buf[g_orb] = o_rev;
		end
	endgenerate
`else
	assign o_rev_buf[0] = o_rev;
	assign o_rev_buf[1] = o_rev;
	assign o_rev_buf[2] = o_rev;
	assign o_rev_buf[3] = o_rev;
`endif

	genvar g_mri;
	generate
		for (g_mri = 0; g_mri < 64; g_mri = g_mri + 1) begin : g_mr_reg
			(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_mr_bit;
			always @(posedge clk or posedge rst_b2d_high) begin
				if (rst_b2d_high)
					r_mr_bit <= 1'b0;
				else if (ik_mr_buf[g_mri / 16])
					r_mr_bit <= o_rev_buf[g_mri / 16];
			end
			assign r_mr_buf[g_mri] = r_mr_bit;
		end
	endgenerate
`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			r_mr_rlr_sub <= 1'b0;
		else if (ik_cfg_copy)
			r_mr_rlr_sub <= !o_rev;
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			r_mr_rlr_sub <= 1'b0;
		else if (ik_cfg_copy)
			r_mr_rlr_sub <= !o_rev;
	end
`endif
	wire [63:0] mr_buf = r_mr_buf;
	wire mr_geom        = mr_buf[51];
	wire mr_walk        = mr_buf[1];
	wire mr_leg         = mr_buf[2];
	wire mr_leg_a       = mr_buf[49];
	wire mr_leg_b       = mr_buf[50];
	wire mr_walk_px     = mr_buf[3];
	wire mr_walk_px_lo  = mr_buf[52];
	wire mr_walk_spx    = mr_buf[4];
	wire mr_walk_spx_lo = mr_buf[53];
	wire mr_walk_lo     = mr_buf[5];
	wire mr_walk_rlr_init = mr_buf[6];
	wire mr_walk_rlr_step = mr_buf[47];
	wire mr_walk_dt0    = mr_buf[7];
	wire mr_walk_dt1    = mr_buf[8];
	wire mr_walk_dt2    = mr_buf[9];
	wire mr_walk_dt3    = mr_buf[10];
	wire mr_walk_st0    = mr_buf[11];
	wire mr_walk_st1    = mr_buf[12];
	wire mr_dacc_lo     = mr_buf[13];
	wire mr_dacc_hi     = mr_buf[14];
	wire mr_sacc_lo     = mr_buf[15];
	wire mr_sacc_h0     = mr_buf[16];
	wire mr_sacc_h1     = mr_buf[48];
	wire mr_sacc_hi     = mr_buf[54];
	wire mr_geom_dlo    = mr_buf[0];
	wire mr_geom_slo    = mr_buf[18];
	wire mr_geom_lin_d0 = mr_buf[19];
	wire mr_geom_lin_d1 = mr_buf[20];
	wire mr_geom_lin_s0 = mr_buf[21];
	wire mr_geom_lin_s1 = mr_buf[22];
	wire mr_geom_lin_p1_lo = mr_buf[23];
	wire mr_geom_lin_p1_hi = mr_buf[46];
	wire mr_px_row      = mr_buf[24];
	wire mr_spx_row     = mr_buf[25];
	wire mr_dlo_lo      = mr_buf[26];
	wire mr_dlo_hi      = mr_buf[27];
	wire mr_slo_lo      = mr_buf[28];
	wire mr_slo_hi      = mr_buf[29];
	wire mr_yd0         = mr_buf[30];
	wire mr_ys0         = mr_buf[31];
	wire mr_px          = mr_buf[32];
	wire mr_spx         = mr_buf[33];
	wire mr_walk_px_hi  = mr_buf[34];
	wire mr_walk_spx_hi = mr_buf[35];
	wire mr_px_hi       = mr_buf[36];
	wire mr_spx_hi      = mr_buf[37];
	wire mr_dacc_m0     = mr_buf[38];
	wire mr_dacc_m1     = mr_buf[39];
	wire mr_sacc_m0     = mr_buf[40];
	wire mr_sacc_m1     = mr_buf[41];
	wire mr_walk_st2    = mr_buf[42];
	wire mr_walk_st3    = mr_buf[43];
	wire mr_yd1         = mr_buf[44];
	wire mr_ys1         = mr_buf[45];

	/* Burst Segment Geometry */
	wire [3:0] unused_2b_co;
	reg [2:0] tile_room_b;
	wire [US:0] bst_room_t = {{(US - 2){1'b0}}, tile_room_b};
	wire bst_tiled = mdt_ctl_b || mst_ctl;
	wire [US:0] bst_room = cur_bst_room;

`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire cmp_lo_le = (rem_b[2:0] <= cur_bst_room[2:0]);
	wire cmp_lo_lt = (rem_b[2:0] <  cur_bst_room[2:0]);
	(* keep = "true" *) wire cmp_hi_lt = (rem_b[5:3] < cur_bst_room[5:3]);
	(* keep = "true" *) wire cmp_hi_eq = (rem_b[5:3] == cur_bst_room[5:3]);
	wire cmp_top_rem = (US >= 6) ? rem_b[6] : 1'b0;
	wire cmp_top_rm  = (US >= 6) ? cur_bst_room[US] : 1'b0;
	wire cmp_last_le = (!cmp_top_rem && cmp_top_rm) || ((cmp_top_rem == cmp_top_rm) && (cmp_hi_lt || (cmp_hi_eq && cmp_lo_le)));
	wire cmp_last_bo = !((!cmp_top_rem && cmp_top_rm) || ((cmp_top_rem == cmp_top_rm) && (cmp_hi_lt || (cmp_hi_eq && cmp_lo_le))));

	/* Direct 1-LUT6 bst_seg_last leaves (merges u_cmpm+u_bslb -> <= 4 LUTs total, Rule 1 clean). */
	wire [12:0] bsl_buf;
	genvar g_bslb;
	generate
		for (g_bslb = 0; g_bslb < 13; g_bslb = g_bslb + 1) begin : g_bsl_buf
			(* dont_touch = "true" *) LUT6 #(.INIT(64'hECFF00EC00000000)) u_bslb (
				.I0(cmp_lo_le), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
				.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(rem_b_small_bsl),
				.O(bsl_buf[g_bslb])
			);
		end
	endgenerate
	wire bsl_px_hi   = bsl_buf[0];
	wire bsl_px_lo   = bsl_buf[1];
	wire bsl_spx_hi  = bsl_buf[2];
	wire bsl_spx_lo  = bsl_buf[3];
	wire bsl_rem_lo  = bsl_buf[4];
	wire bsl_rem_hi  = bsl_buf[5];
	wire bsl_seg     = bsl_buf[6];
	wire bsl_ctl     = bsl_buf[7];
	wire bsl_room    = bsl_buf[8];
	wire bsl_step0   = bsl_buf[10];
	wire bsl_step1   = bsl_buf[11];
	wire bsl_back    = bsl_buf[12];
	wire bst_seg_last = bsl_buf[7];
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire cmp_lo_le = (rem_b[2:0] <= cur_bst_room[2:0]);
	wire cmp_lo_lt = (rem_b[2:0] <  cur_bst_room[2:0]);
	(* keep = "true" *) wire cmp_hi_lt = (rem_b[5:3] < cur_bst_room[5:3]);
	(* keep = "true" *) wire cmp_hi_eq = (rem_b[5:3] == cur_bst_room[5:3]);
	wire cmp_top_rem = (US >= 6) ? rem_b[6] : 1'b0;
	wire cmp_top_rm  = (US >= 6) ? cur_bst_room[US] : 1'b0;
	wire cmp_last_le = (!cmp_top_rem && cmp_top_rm) || ((cmp_top_rem == cmp_top_rm) && (cmp_hi_lt || (cmp_hi_eq && cmp_lo_le)));
	wire cmp_last_bo = !((!cmp_top_rem && cmp_top_rm) || ((cmp_top_rem == cmp_top_rm) && (cmp_hi_lt || (cmp_hi_eq && cmp_lo_le))));

	/* Direct 1-LUT6 bst_seg_last leaves (merges u_cmpm+u_bslb -> <= 4 LUTs total, Rule 1 clean). */
	wire [12:0] bsl_buf;
	genvar g_bslb;
	generate
		for (g_bslb = 0; g_bslb < 13; g_bslb = g_bslb + 1) begin : g_bsl_buf
			(* dont_touch = "true" *) LUT6 #(.INIT(64'hECFF00EC00000000)) u_bslb (
				.I0(cmp_lo_le), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
				.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(rem_b_small_bsl),
				.O(bsl_buf[g_bslb])
			);
		end
	endgenerate
	wire bsl_px_hi   = bsl_buf[0];
	wire bsl_px_lo   = bsl_buf[1];
	wire bsl_spx_hi  = bsl_buf[2];
	wire bsl_spx_lo  = bsl_buf[3];
	wire bsl_rem_lo  = bsl_buf[4];
	wire bsl_rem_hi  = bsl_buf[5];
	wire bsl_seg     = bsl_buf[6];
	wire bsl_ctl     = bsl_buf[7];
	wire bsl_room    = bsl_buf[8];
	wire bsl_step0   = bsl_buf[10];
	wire bsl_step1   = bsl_buf[11];
	wire bsl_back    = bsl_buf[12];
	wire bst_seg_last = bsl_buf[7];
`else
	wire cmp_g10_le = (!rem_b[1] & cur_bst_room[1]) | (!(rem_b[1] ^ cur_bst_room[1]) & (!rem_b[0] | cur_bst_room[0]));
	wire cmp_g32_lt = (!rem_b[3] & cur_bst_room[3]) | (!(rem_b[3] ^ cur_bst_room[3]) & (!rem_b[2] & cur_bst_room[2]));
	wire cmp_g32_eq = !(rem_b[3] ^ cur_bst_room[3]) & !(rem_b[2] ^ cur_bst_room[2]);
	wire cmp_g54_lt = (!rem_b[5] & cur_bst_room[5]) | (!(rem_b[5] ^ cur_bst_room[5]) & (!rem_b[4] & cur_bst_room[4]));
	wire cmp_g54_eq = !(rem_b[5] ^ cur_bst_room[5]) & !(rem_b[4] ^ cur_bst_room[4]);
	wire cmp_g6_lt, cmp_g6_eq;
	generate
		if (US >= 6) begin : g_cmp_ge6
			assign cmp_g6_lt = (rem_b[US:6] <  cur_bst_room[US:6]);
			assign cmp_g6_eq = (rem_b[US:6] == cur_bst_room[US:6]);
		end else begin : g_cmp_lt6
			assign cmp_g6_lt = 1'b0;
			assign cmp_g6_eq = 1'b1;
		end
	endgenerate
	wire cmp_lo4_le   = cmp_g32_lt | (cmp_g32_eq & cmp_g10_le);
	wire cmp_hi_lt    = cmp_g6_lt  | (cmp_g6_eq  & cmp_g54_lt);
	wire cmp_hi_eq    = cmp_g6_eq  & cmp_g54_eq;
	wire cmp_last_le  = cmp_hi_lt  | (cmp_hi_eq  & cmp_lo4_le);
	wire cmp_last_bo  = !cmp_last_le;
	wire cmp_bsl_g6_lt = rem_b_small_bsl & cmp_g6_lt;
	wire cmp_bsl_g6_eq = rem_b_small_bsl & cmp_g6_eq;
	wire cmp_bsl_hi_lt = cmp_bsl_g6_lt | (cmp_bsl_g6_eq & cmp_g54_lt);
	wire cmp_bsl_hi_eq = cmp_bsl_g6_eq & cmp_g54_eq;
	wire bst_seg_last  = cmp_bsl_hi_lt | (cmp_bsl_hi_eq & cmp_lo4_le);
	wire bsl_px_hi   = bst_seg_last;
	wire bsl_px_lo   = bst_seg_last;
	wire bsl_spx_hi  = bst_seg_last;
	wire bsl_spx_lo  = bst_seg_last;
	wire bsl_rem_lo  = bst_seg_last;
	wire bsl_rem_hi  = bst_seg_last;
	wire bsl_seg     = bst_seg_last;
	wire bsl_ctl     = bst_seg_last;
	wire bsl_room    = bst_seg_last;
	wire bsl_step0   = bst_seg_last;
	wire bsl_step1   = bst_seg_last;
	wire bsl_back    = bst_seg_last;
`endif

	wire [US:0] bst_seg_n = bsl_seg ? rem_b[US:0] : cur_bst_room_b;
	wire [US:0] rem_bytes = (pslw_ctl0 == 3'd4) ? {rem_b[US-4:0], 4'b0000} :
	                        (pslw_ctl0 == 3'd3) ? {rem_b[US-3:0], 3'b000}  :
	                        (pslw_ctl0 == 3'd2) ? {rem_b[US-2:0], 2'b00}   :
	                        (pslw_ctl0 == 3'd1) ? {rem_b[US-1:0], 1'b0}    : rem_b[US:0];
	wire [US:0] room_bytes = (pslw_ctl1 == 3'd4) ? {cur_bst_room_b[US-4:0], 4'b0000} :
	                         (pslw_ctl1 == 3'd3) ? {cur_bst_room_b[US-3:0], 3'b000}  :
	                         (pslw_ctl1 == 3'd2) ? {cur_bst_room_b[US-2:0], 2'b00}   :
	                         (pslw_ctl1 == 3'd1) ? {cur_bst_room_b[US-1:0], 1'b0}    : cur_bst_room_b;
`ifdef SYNTH_KINTEX7
	wire [US:0] bst_seg_bytes;
	genvar g_sb;
	generate
		for (g_sb = 0; g_sb <= US; g_sb = g_sb + 1) begin : g_bst_sb
			LUT3 #(.INIT(8'hCA)) u_bsb (
				.I0(room_bytes[g_sb]),
				.I1(rem_bytes[g_sb]),
				.I2(bsl_seg),
				.O(bst_seg_bytes[g_sb])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US:0] bst_seg_bytes;
	genvar g_sb;
	generate
		for (g_sb = 0; g_sb <= US; g_sb = g_sb + 1) begin : g_bst_sb
			LUT3 #(.INIT(8'hCA)) u_bsb (
				.I0(room_bytes[g_sb]),
				.I1(rem_bytes[g_sb]),
				.I2(bsl_seg),
				.O(bst_seg_bytes[g_sb])
			);
		end
	endgenerate
`else
	wire [US:0] bst_seg_bytes = bsl_seg ? rem_bytes : room_bytes;
`endif

	wire [US:0] rem_back_n, room_back_n;
`ifdef SYNTH_KINTEX7
	wire [US:0] rb_z;
	genvar g_rb;
	generate
		for (g_rb = 0; g_rb <= US; g_rb = g_rb + 1) begin : g_rb_bits
			assign rb_z[g_rb] = ~(|cur_bst_room[g_rb:0]);
		end
	endgenerate
	assign room_back_n[0] = ~cur_bst_room[0];
	generate
		for (g_rb = 1; g_rb <= US; g_rb = g_rb + 1) begin : g_room_bn
			assign room_back_n[g_rb] = cur_bst_room[g_rb] ^ rb_z[g_rb - 1];
		end
	endgenerate

	wire [US:0] mb_z;
	generate
		for (g_rb = 0; g_rb <= US; g_rb = g_rb + 1) begin : g_mb_bits
			assign mb_z[g_rb] = ~(|rem_b[g_rb:0]);
		end
	endgenerate
	assign rem_back_n[0] = ~rem_b[0];
	generate
		for (g_rb = 1; g_rb <= US; g_rb = g_rb + 1) begin : g_rem_bn
			assign rem_back_n[g_rb] = rem_b[g_rb] ^ mb_z[g_rb - 1];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US:0] rb_z;
	genvar g_rb;
	generate
		for (g_rb = 0; g_rb <= US; g_rb = g_rb + 1) begin : g_rb_bits
			assign rb_z[g_rb] = ~(|cur_bst_room[g_rb:0]);
		end
	endgenerate
	assign room_back_n[0] = ~cur_bst_room[0];
	generate
		for (g_rb = 1; g_rb <= US; g_rb = g_rb + 1) begin : g_room_bn
			assign room_back_n[g_rb] = cur_bst_room[g_rb] ^ rb_z[g_rb - 1];
		end
	endgenerate

	wire [US:0] mb_z;
	generate
		for (g_rb = 0; g_rb <= US; g_rb = g_rb + 1) begin : g_mb_bits
			assign mb_z[g_rb] = ~(|rem_b[g_rb:0]);
		end
	endgenerate
	assign rem_back_n[0] = ~rem_b[0];
	generate
		for (g_rb = 1; g_rb <= US; g_rb = g_rb + 1) begin : g_rem_bn
			assign rem_back_n[g_rb] = rem_b[g_rb] ^ mb_z[g_rb - 1];
		end
	endgenerate
`else
	assign room_back_n[0] = ~cur_bst_room[0];
	assign rem_back_n[0]  = ~rem_b[0];
	genvar g_rb;
	generate
		for (g_rb = 1; g_rb <= US; g_rb = g_rb + 1) begin : g_bk_fast
			assign room_back_n[g_rb] = cur_bst_room[g_rb] ^ (~(|cur_bst_room[g_rb-1:0]));
			assign rem_back_n[g_rb]  = rem_b[g_rb]        ^ (~(|rem_b[g_rb-1:0]));
		end
	endgenerate
	wire [4:0] psl_gbk_oh = {pslw_gbk0 == 3'd4, pslw_gbk0 == 3'd3, pslw_gbk0 == 3'd2, pslw_gbk0 == 3'd1, pslw_gbk0 == 3'd0};
`endif
`ifdef SYNTH_KINTEX7
	wire [15:0] rem_back_16 = (pslw_gbk0 == 3'd4) ? {{(11-US){1'b0}}, rem_back_n, 4'b0000} :
	                          (pslw_gbk0 == 3'd3) ? {{(12-US){1'b0}}, rem_back_n, 3'b000}  :
	                          (pslw_gbk0 == 3'd2) ? {{(13-US){1'b0}}, rem_back_n, 2'b00}   :
	                          (pslw_gbk0 == 3'd1) ? {{(14-US){1'b0}}, rem_back_n, 1'b0}    : {{(15-US){1'b0}}, rem_back_n};
	wire [15:0] room_back_16 = (pslw_gbk0 == 3'd4) ? {{(11-US){1'b0}}, room_back_n, 4'b0000} :
	                           (pslw_gbk0 == 3'd3) ? {{(12-US){1'b0}}, room_back_n, 3'b000}  :
	                           (pslw_gbk0 == 3'd2) ? {{(13-US){1'b0}}, room_back_n, 2'b00}   :
	                           (pslw_gbk0 == 3'd1) ? {{(14-US){1'b0}}, room_back_n, 1'b0}    : {{(15-US){1'b0}}, room_back_n};
	wire [US-1:0] bst_back_lo;
	generate
		for (g_sb = 0; g_sb < US; g_sb = g_sb + 1) begin : g_bst_bk
			LUT3 #(.INIT(8'hCA)) u_bbk (
				.I0(room_back_16[g_sb]),
				.I1(rem_back_16[g_sb]),
				.I2(bsl_back),
				.O(bst_back_lo[g_sb])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [15:0] rem_back_16 = (pslw_gbk0 == 3'd4) ? {{(11-US){1'b0}}, rem_back_n, 4'b0000} :
	                          (pslw_gbk0 == 3'd3) ? {{(12-US){1'b0}}, rem_back_n, 3'b000}  :
	                          (pslw_gbk0 == 3'd2) ? {{(13-US){1'b0}}, rem_back_n, 2'b00}   :
	                          (pslw_gbk0 == 3'd1) ? {{(14-US){1'b0}}, rem_back_n, 1'b0}    : {{(15-US){1'b0}}, rem_back_n};
	wire [15:0] room_back_16 = (pslw_gbk0 == 3'd4) ? {{(11-US){1'b0}}, room_back_n, 4'b0000} :
	                           (pslw_gbk0 == 3'd3) ? {{(12-US){1'b0}}, room_back_n, 3'b000}  :
	                           (pslw_gbk0 == 3'd2) ? {{(13-US){1'b0}}, room_back_n, 2'b00}   :
	                           (pslw_gbk0 == 3'd1) ? {{(14-US){1'b0}}, room_back_n, 1'b0}    : {{(15-US){1'b0}}, room_back_n};
	wire [US-1:0] bst_back_lo;
	generate
		for (g_sb = 0; g_sb < US; g_sb = g_sb + 1) begin : g_bst_bk
			LUT3 #(.INIT(8'hCA)) u_bbk (
				.I0(room_back_16[g_sb]),
				.I1(rem_back_16[g_sb]),
				.I2(bsl_back),
				.O(bst_back_lo[g_sb])
			);
		end
	endgenerate
`else
	wire [US-1:0] rem_back_sh =
		({US{psl_gbk_oh[4]}} & {rem_back_n[US-5:0], 4'b0000}) |
		({US{psl_gbk_oh[3]}} & {rem_back_n[US-4:0], 3'b000})  |
		({US{psl_gbk_oh[2]}} & {rem_back_n[US-3:0], 2'b00})   |
		({US{psl_gbk_oh[1]}} & {rem_back_n[US-2:0], 1'b0})    |
		({US{psl_gbk_oh[0]}} & rem_back_n[US-1:0]);
	wire [US-1:0] room_back_sh =
		({US{psl_gbk_oh[4]}} & {room_back_n[US-5:0], 4'b0000}) |
		({US{psl_gbk_oh[3]}} & {room_back_n[US-4:0], 3'b000})  |
		({US{psl_gbk_oh[2]}} & {room_back_n[US-3:0], 2'b00})   |
		({US{psl_gbk_oh[1]}} & {room_back_n[US-2:0], 1'b0})    |
		({US{psl_gbk_oh[0]}} & room_back_n[US-1:0]);
	wire [US-1:0] bst_back_lo = bsl_back ? rem_back_sh : room_back_sh;
	wire back_n_msb_unused = &{1'b0, rem_back_n[US], room_back_n[US]};
`endif

	wire bsl_rline;
`ifdef SYNTH_KINTEX7
	/* Separate 6-input LUT for bsl_rline eliminates u_cmpm level on rem_b -> rem_line_b/ge8. */
	(* dont_touch = "true" *) LUT6 #(.INIT(64'hECFF00EC00000000)) u_bslb_rl (
		.I0(cmp_lo_le), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
		.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(rem_b_small_bsl),
		.O(bsl_rline)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Separate 6-input LUT for bsl_rline eliminates u_cmpm level on rem_b -> rem_line_b/ge8. */
	(* dont_touch = "true" *) LUT6 #(.INIT(64'hECFF00EC00000000)) u_bslb_rl (
		.I0(cmp_lo_le), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
		.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(rem_b_small_bsl),
		.O(bsl_rline)
	);
`else
	assign bsl_rline = bst_seg_last;
`endif

	wire rem_line_eq_lo = (cur_bst_room[2:0]  == rem_line_b[2:0]);
	wire rem_line_eq_hi = (cur_bst_room[US:3] == rem_line_b[US:3]);
	wire rem_line_eq_w  = rem_line_eq_lo && rem_line_eq_hi;
	wire rem_line_row_ge8 = |rem_line_row[US:3];
	wire line_pixels_ge8  = |line_pixels_b[US:3];
	wire rem_line_b_ge8_d;
	wire [US:0] rem_line_b_d;
`ifdef SYNTH_KINTEX7
	wire [US:0] rem_line_sub;
	wire rem_line_bo;
	am_sub #(.W(US + 1)) s_rline (.a(rem_line_b), .b(cur_bst_room), .bi(1'b0), .d(rem_line_sub), .bo(rem_line_bo));
	wire rem_line_sub_ge8 = |rem_line_sub[US:3];
	LUT6 #(.INIT(64'hFFFF0000F780F780)) u_rline_ge8_d (
		.I0(rem_line_eq_lo), .I1(rem_line_eq_hi),
		.I2(line_pixels_ge8), .I3(rem_line_sub_ge8),
		.I4(rem_line_row_ge8), .I5(bsl_rline),
		.O(rem_line_b_ge8_d)
	);
	genvar grl;
	generate
		for (grl = 0; grl <= US; grl = grl + 1) begin : g_rline_d
			LUT6 #(.INIT(64'hFFFF0000F780F780)) u_rline_d (
				.I0(rem_line_eq_lo), .I1(rem_line_eq_hi),
				.I2(line_pixels_b[grl]), .I3(rem_line_sub[grl]),
				.I4(rem_line_row[grl]), .I5(bsl_rline),
				.O(rem_line_b_d[grl])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US:0] rem_line_sub;
	wire rem_line_bo;
	am_sub #(.W(US + 1)) s_rline (.a(rem_line_b), .b(cur_bst_room), .bi(1'b0), .d(rem_line_sub), .bo(rem_line_bo));
	wire rem_line_sub_ge8 = |rem_line_sub[US:3];
	LUT6 #(.INIT(64'hFFFF0000F780F780)) u_rline_ge8_d (
		.I0(rem_line_eq_lo), .I1(rem_line_eq_hi),
		.I2(line_pixels_ge8), .I3(rem_line_sub_ge8),
		.I4(rem_line_row_ge8), .I5(bsl_rline),
		.O(rem_line_b_ge8_d)
	);
	genvar grl;
	generate
		for (grl = 0; grl <= US; grl = grl + 1) begin : g_rline_d
			LUT6 #(.INIT(64'hFFFF0000F780F780)) u_rline_d (
				.I0(rem_line_eq_lo), .I1(rem_line_eq_hi),
				.I2(line_pixels_b[grl]), .I3(rem_line_sub[grl]),
				.I4(rem_line_row[grl]), .I5(bsl_rline),
				.O(rem_line_b_d[grl])
			);
		end
	endgenerate
`else
	wire rl_b0;
	wire [2:0] rem_line_sub_lo_unused;
	wire [US:0] rem_line_sub;
	wire rem_line_bo;
	am_sub #(.W(3))      s_rline_l (.a(rem_line_b[2:0]), .b(cur_bst_room[2:0]), .bi(1'b0), .d(rem_line_sub_lo_unused), .bo(rl_b0));
	am_sub #(.W(US + 1)) s_rline   (.a(rem_line_b),      .b(cur_bst_room),      .bi(1'b0), .d(rem_line_sub),           .bo(rem_line_bo));
	wire [US-3:0] rl_y_inv = ~cur_bst_room[US:3];
	wire [US-3:0] rl_sub1_x = rem_line_b[US:3] ^ rl_y_inv;
	wire [US-3:0] rl_sub1_o = rem_line_b[US:3] | rl_y_inv;
	wire [US-3:0] rl_sub1_mm = {rl_sub1_x[US-3:1] ^ rl_sub1_o[US-4:0], rl_sub1_x[0]};
	wire rem_line_sub_ge8 = rl_b0 ? (|rl_sub1_mm) : (|(rem_line_b[US:3] ^ cur_bst_room[US:3]));
	wire rline_s_row = bsl_rline;
	wire rline_s_eq  = !bsl_rline && rem_line_eq_w;
	wire rline_s_sub = !bsl_rline && !rem_line_eq_w;
	assign rem_line_b_ge8_d = (rline_s_row & rem_line_row_ge8) | (rline_s_eq & line_pixels_ge8) | (rline_s_sub & rem_line_sub_ge8);
	genvar grl;
	generate
		for (grl = 0; grl <= US; grl = grl + 1) begin : g_rline_d
			assign rem_line_b_d[grl] =
				(rline_s_row & rem_line_row[grl]) |
				(rline_s_eq  & line_pixels_b[grl]) |
				(rline_s_sub & rem_line_sub[grl]);
		end
	endgenerate
`endif

	/* Clean 17-bit 5+6+6 Carry-Select Addsub for Coordinates (<= 8 cmos3) */
	wire [16:0] brm_17 = {{(16-US){1'b0}}, cur_bst_room_b};
	/* 1. px_next */
	wire [16:0] px_step;
	wire [16:0] px_op_b = {
		mr_walk_px    ? ~brm_17[16:11] : brm_17[16:11],
		mr_walk_px_lo ? ~brm_17[10:0]  : brm_17[10:0]
	};
`ifdef SYNTH_KINTEX7
	wire bpx_c0, bpx_h0, bpx_h1;
	wire [8:0] bpx_s10, bpx_s11;
	am_add #(.W(8)) a_bpx_lo (.a(px_b[7:0]),  .b(px_op_b[7:0]),  .ci(mr_walk_px_lo), .s(px_step[7:0]), .co(bpx_c0));
	am_add #(.W(9)) a_bpx_h0 (.a(px_b[16:8]), .b(px_op_b[16:8]), .ci(1'b0),          .s(bpx_s10),       .co(bpx_h0));
	am_add #(.W(9)) a_bpx_h1 (.a(px_b[16:8]), .b(px_op_b[16:8]), .ci(1'b1),          .s(bpx_s11),       .co(bpx_h1));
	assign px_step[16:8] = bpx_c0 ? bpx_s11 : bpx_s10;
	wire px_co = (bpx_c0 ? bpx_h1 : bpx_h0) & bpx_h0;
	wire [16:0] px_b_next;
	assign px_b_next[7:0] = bsl_px_lo ? px_row[7:0] : px_step[7:0];
	genvar g_bpx;
	generate
		for (g_bpx = 0; g_bpx < 9; g_bpx = g_bpx + 1) begin : g_bpx_lut
			LUT5 #(.INIT(32'hFF00D8D8)) u_bpx_m (.I0(bpx_s10[g_bpx]), .I1(bpx_s11[g_bpx]), .I2(bpx_c0), .I3(px_row[8 + g_bpx]), .I4(bsl_px_hi), .O(px_b_next[8 + g_bpx]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bpx_c0, bpx_h0, bpx_h1;
	wire [8:0] bpx_s10, bpx_s11;
	am_add #(.W(8)) a_bpx_lo (.a(px_b[7:0]),  .b(px_op_b[7:0]),  .ci(mr_walk_px_lo), .s(px_step[7:0]), .co(bpx_c0));
	am_add #(.W(9)) a_bpx_h0 (.a(px_b[16:8]), .b(px_op_b[16:8]), .ci(1'b0),          .s(bpx_s10),       .co(bpx_h0));
	am_add #(.W(9)) a_bpx_h1 (.a(px_b[16:8]), .b(px_op_b[16:8]), .ci(1'b1),          .s(bpx_s11),       .co(bpx_h1));
	assign px_step[16:8] = bpx_c0 ? bpx_s11 : bpx_s10;
	wire px_co = (bpx_c0 ? bpx_h1 : bpx_h0) & bpx_h0;
	wire [16:0] px_b_next;
	assign px_b_next[7:0] = bsl_px_lo ? px_row[7:0] : px_step[7:0];
	genvar g_bpx;
	generate
		for (g_bpx = 0; g_bpx < 9; g_bpx = g_bpx + 1) begin : g_bpx_lut
			LUT5 #(.INIT(32'hFF00D8D8)) u_bpx_m (.I0(bpx_s10[g_bpx]), .I1(bpx_s11[g_bpx]), .I2(bpx_c0), .I3(px_row[8 + g_bpx]), .I4(bsl_px_hi), .O(px_b_next[8 + g_bpx]));
		end
	endgenerate
`else
	wire bpx_fc8_raw, bpx_rc8_raw;
	wire [7:0] px_fwd_l8, px_rev_l8;
	am_add #(.W(8)) a_bpx_f8 (.a(px_b[7:0]), .b(brm_17[7:0]), .ci(1'b0), .s(px_fwd_l8), .co(bpx_fc8_raw));
	am_sub #(.W(8)) s_bpx_r8 (.a(px_b[7:0]), .b(brm_17[7:0]), .bi(1'b0), .d(px_rev_l8), .bo(bpx_rc8_raw));
	wire bpx_rc8 = (px_b[7:4] < brm_17[7:4]) || ((px_b[7:4] == brm_17[7:4]) && (px_b[3:0] < brm_17[3:0]));
	wire bpx_fc8 = ((~px_b[7:4]) < brm_17[7:4]) || (((~px_b[7:4]) == brm_17[7:4]) && ((~px_b[3:0]) < brm_17[3:0]));
	wire [8:0] px_inc_m, px_dec_m;
	assign px_inc_m[0] = 1'b1;
	assign px_dec_m[0] = 1'b1;
	genvar g_pxh;
	generate
		for (g_pxh = 1; g_pxh < 9; g_pxh = g_pxh + 1) begin : g_px_hi
			assign px_inc_m[g_pxh] = &px_b[8 + g_pxh - 1 : 8];
			assign px_dec_m[g_pxh] = ~(|px_b[8 + g_pxh - 1 : 8]);
		end
	endgenerate
	wire [8:0] px_hi_p1 = px_b[16:8] ^ px_inc_m;
	wire [8:0] px_hi_m1 = px_b[16:8] ^ px_dec_m;
	wire [8:0] px_hi_base = bsl_px_hi ? px_row[16:8] : px_b[16:8];
	wire [8:0] px_hi_c1   = bsl_px_hi ? px_row[16:8] : (mr_walk_px ? px_hi_m1 : px_hi_p1);
	wire bpx_carry = mr_walk_px ? bpx_rc8 : bpx_fc8;
	wire [8:0] px_next_hi = bpx_carry ? px_hi_c1 : px_hi_base;
	assign px_step = {
		(mr_walk_px ? (bpx_rc8 ? px_hi_m1 : px_b[16:8]) : (bpx_fc8 ? px_hi_p1 : px_b[16:8])),
		(mr_walk_px ? px_rev_l8 : px_fwd_l8)
	};
	wire px_co = 1'b0;
	wire px_unused_op = &{1'b0, px_op_b, bpx_fc8_raw, bpx_rc8_raw};
	wire px_use_row_lo = bsl_px_lo;
	wire px_use_rev_lo = !bsl_px_lo && mr_walk_px;
	wire px_use_fwd_lo = !bsl_px_lo && !mr_walk_px;
	wire [16:0] px_b_next = {
		px_next_hi,
		({8{px_use_row_lo}} & px_row[7:0]) | ({8{px_use_rev_lo}} & px_rev_l8) | ({8{px_use_fwd_lo}} & px_fwd_l8)
	};
`endif

	/* 2. spx_next */
	wire [16:0] spx_step;
	wire [16:0] spx_op_b = {
		mr_walk_spx    ? ~brm_17[16:11] : brm_17[16:11],
		mr_walk_spx_lo ? ~brm_17[10:0]  : brm_17[10:0]
	};
`ifdef SYNTH_KINTEX7
	wire bsx_c0, bsx_h0, bsx_h1;
	wire [8:0] bsx_s10, bsx_s11;
	am_add #(.W(8)) a_bspx_lo (.a(spx_b[7:0]),  .b(spx_op_b[7:0]),  .ci(mr_walk_spx_lo), .s(spx_step[7:0]), .co(bsx_c0));
	am_add #(.W(9)) a_bspx_h0 (.a(spx_b[16:8]), .b(spx_op_b[16:8]), .ci(1'b0),           .s(bsx_s10),       .co(bsx_h0));
	am_add #(.W(9)) a_bspx_h1 (.a(spx_b[16:8]), .b(spx_op_b[16:8]), .ci(1'b1),           .s(bsx_s11),       .co(bsx_h1));
	assign spx_step[16:8] = bsx_c0 ? bsx_s11 : bsx_s10;
	wire spx_co = (bsx_c0 ? bsx_h1 : bsx_h0) & bsx_h0;
	wire [16:0] spx_b_next;
	assign spx_b_next[7:0] = bsl_spx_lo ? spx_row[7:0] : spx_step[7:0];
	genvar g_bsx;
	generate
		for (g_bsx = 0; g_bsx < 9; g_bsx = g_bsx + 1) begin : g_bsx_lut
			LUT5 #(.INIT(32'hFF00D8D8)) u_bsx_m (.I0(bsx_s10[g_bsx]), .I1(bsx_s11[g_bsx]), .I2(bsx_c0), .I3(spx_row[8 + g_bsx]), .I4(bsl_spx_hi), .O(spx_b_next[8 + g_bsx]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bsx_c0, bsx_h0, bsx_h1;
	wire [8:0] bsx_s10, bsx_s11;
	am_add #(.W(8)) a_bspx_lo (.a(spx_b[7:0]),  .b(spx_op_b[7:0]),  .ci(mr_walk_spx_lo), .s(spx_step[7:0]), .co(bsx_c0));
	am_add #(.W(9)) a_bspx_h0 (.a(spx_b[16:8]), .b(spx_op_b[16:8]), .ci(1'b0),           .s(bsx_s10),       .co(bsx_h0));
	am_add #(.W(9)) a_bspx_h1 (.a(spx_b[16:8]), .b(spx_op_b[16:8]), .ci(1'b1),           .s(bsx_s11),       .co(bsx_h1));
	assign spx_step[16:8] = bsx_c0 ? bsx_s11 : bsx_s10;
	wire spx_co = (bsx_c0 ? bsx_h1 : bsx_h0) & bsx_h0;
	wire [16:0] spx_b_next;
	assign spx_b_next[7:0] = bsl_spx_lo ? spx_row[7:0] : spx_step[7:0];
	genvar g_bsx;
	generate
		for (g_bsx = 0; g_bsx < 9; g_bsx = g_bsx + 1) begin : g_bsx_lut
			LUT5 #(.INIT(32'hFF00D8D8)) u_bsx_m (.I0(bsx_s10[g_bsx]), .I1(bsx_s11[g_bsx]), .I2(bsx_c0), .I3(spx_row[8 + g_bsx]), .I4(bsl_spx_hi), .O(spx_b_next[8 + g_bsx]));
		end
	endgenerate
`else
	wire bsx_fc8_raw, bsx_rc8_raw;
	wire [7:0] spx_fwd_l8, spx_rev_l8;
	am_add #(.W(8)) a_bspx_f8 (.a(spx_b[7:0]), .b(brm_17[7:0]), .ci(1'b0), .s(spx_fwd_l8), .co(bsx_fc8_raw));
	am_sub #(.W(8)) s_bspx_r8 (.a(spx_b[7:0]), .b(brm_17[7:0]), .bi(1'b0), .d(spx_rev_l8), .bo(bsx_rc8_raw));
	wire bsx_rc8 = (spx_b[7:4] < brm_17[7:4]) || ((spx_b[7:4] == brm_17[7:4]) && (spx_b[3:0] < brm_17[3:0]));
	wire bsx_fc8 = ((~spx_b[7:4]) < brm_17[7:4]) || (((~spx_b[7:4]) == brm_17[7:4]) && ((~spx_b[3:0]) < brm_17[3:0]));
	wire [8:0] spx_inc_m, spx_dec_m;
	assign spx_inc_m[0] = 1'b1;
	assign spx_dec_m[0] = 1'b1;
	genvar g_sxh;
	generate
		for (g_sxh = 1; g_sxh < 9; g_sxh = g_sxh + 1) begin : g_spx_hi
			assign spx_inc_m[g_sxh] = &spx_b[8 + g_sxh - 1 : 8];
			assign spx_dec_m[g_sxh] = ~(|spx_b[8 + g_sxh - 1 : 8]);
		end
	endgenerate
	wire [8:0] spx_hi_p1 = spx_b[16:8] ^ spx_inc_m;
	wire [8:0] spx_hi_m1 = spx_b[16:8] ^ spx_dec_m;
	wire [8:0] spx_hi_base = bsl_spx_hi ? spx_row[16:8] : spx_b[16:8];
	wire [8:0] spx_hi_c1   = bsl_spx_hi ? spx_row[16:8] : (mr_walk_spx ? spx_hi_m1 : spx_hi_p1);
	wire bsx_carry = mr_walk_spx ? bsx_rc8 : bsx_fc8;
	wire [8:0] spx_next_hi = bsx_carry ? spx_hi_c1 : spx_hi_base;
	assign spx_step = {
		(mr_walk_spx ? (bsx_rc8 ? spx_hi_m1 : spx_b[16:8]) : (bsx_fc8 ? spx_hi_p1 : spx_b[16:8])),
		(mr_walk_spx ? spx_rev_l8 : spx_fwd_l8)
	};
	wire spx_co = 1'b0;
	wire spx_unused_op = &{1'b0, spx_op_b, bsx_fc8_raw, bsx_rc8_raw};
	wire spx_use_row_lo = bsl_spx_lo;
	wire spx_use_rev_lo = !bsl_spx_lo && mr_walk_spx;
	wire spx_use_fwd_lo = !bsl_spx_lo && !mr_walk_spx;
	wire [16:0] spx_b_next = {
		spx_next_hi,
		({8{spx_use_row_lo}} & spx_row[7:0]) | ({8{spx_use_rev_lo}} & spx_rev_l8) | ({8{spx_use_fwd_lo}} & spx_fwd_l8)
	};
`endif

	/* Precomputed tile_room for next segment (pure 5-input LUT addsub, zero am_add/sub). */
	assign unused_2b_co = 4'd0;
	wire [1:0] px_2b_next = {
		px_b[1] ^ cur_bst_room[1] ^ ((r_mr_walk_lo0 ? ~px_b[0] : px_b[0]) & cur_bst_room[0]),
		px_b[0] ^ cur_bst_room[0]
	};
	wire [1:0] spx_2b_next = {
		spx_b[1] ^ cur_bst_room[1] ^ ((r_mr_walk_lo1 ? ~spx_b[0] : spx_b[0]) & cur_bst_room[0]),
		spx_b[0] ^ cur_bst_room[0]
	};
`ifdef SYNTH_KINTEX7
	wire [2:0] td_walk = {px_2b_next == 2'b00, px_2b_next[1] ^ px_2b_next[0], px_2b_next[0]};
	wire [2:0] ts_walk = {spx_2b_next == 2'b00, spx_2b_next[1] ^ spx_2b_next[0], spx_2b_next[0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] td_walk = {px_2b_next == 2'b00, px_2b_next[1] ^ px_2b_next[0], px_2b_next[0]};
	wire [2:0] ts_walk = {spx_2b_next == 2'b00, spx_2b_next[1] ^ spx_2b_next[0], spx_2b_next[0]};
`else
	wire [2:0] td_walk = {px_2b_next == 2'b00, (px_b[1] ^ cur_bst_room[1]) ^ (cur_bst_room[0] ? ~r_mr_walk_lo0 : px_b[0]), px_2b_next[0]};
	wire [2:0] ts_walk = {spx_2b_next == 2'b00, (spx_b[1] ^ cur_bst_room[1]) ^ (cur_bst_room[0] ? ~r_mr_walk_lo1 : spx_b[0]), spx_2b_next[0]};
`endif
	wire [2:0] tile_room_walk = mdt_ctl_b ? td_walk : ts_walk;

	wire [2:0] td_init = {px_row[1:0] == 2'b00, px_row[1] ^ px_row[0], px_row[0]};
	wire [2:0] ts_init = {spx_row[1:0] == 2'b00, spx_row[1] ^ spx_row[0], spx_row[0]};
	wire [2:0] tile_room_init = mdt_ctl_b ? td_init : ts_init;
	wire [US:0] bst_room_t_init = {{(US - 2){1'b0}}, tile_room_init};
	wire [US:0] bst_room_init = (bst_tiled && (bst_room_t_init < rem_line_init)) ? bst_room_t_init : rem_line_init;

	wire [2:0] tile_room_step = bsl_room ? tile_room_init : tile_room_walk;

	/* 3. rem_next: Clean 5+6+6 (FPGA) / 4+4+4+5 (ASIC) Carry-Select Subtractor */
	wire [16:0] rem_sub;
`ifdef SYNTH_KINTEX7
	wire brm_b0, brm_b1_0, brm_b1_1, brm_b2_0, brm_b2_1;
	wire [5:0] brm_s1_0, brm_s1_1, brm_s2_0, brm_s2_1;
	am_sub #(.W(5)) s_brem_0  (.a(rem_b[4:0]),   .b(brm_17[4:0]),   .bi(1'b0), .d(rem_sub[4:0]), .bo(brm_b0));
	am_sub #(.W(6)) s_brem_10 (.a(rem_b[10:5]),  .b(brm_17[10:5]),  .bi(1'b0), .d(brm_s1_0),     .bo(brm_b1_0));
	am_sub #(.W(6)) s_brem_11 (.a(rem_b[10:5]),  .b(brm_17[10:5]),  .bi(1'b1), .d(brm_s1_1),     .bo(brm_b1_1));
	am_sub #(.W(6)) s_brem_20 (.a(rem_b[16:11]), .b(brm_17[16:11]), .bi(1'b0), .d(brm_s2_0),     .bo(brm_b2_0));
	am_sub #(.W(6)) s_brem_21 (.a(rem_b[16:11]), .b(brm_17[16:11]), .bi(1'b1), .d(brm_s2_1),     .bo(brm_b2_1));
	wire brm_b1 = brm_b1_0 | (brm_b1_1 & brm_b0);
	assign rem_sub[10:5]  = brm_b0 ? brm_s1_1 : brm_s1_0;
	assign rem_sub[16:11] = brm_b1 ? brm_s2_1 : brm_s2_0;
	wire rem_sub_bo = (brm_b1 ? brm_b2_1 : brm_b2_0);
	wire [16:0] rem_b_next = {bsl_rem_hi ? width[16:8] : rem_sub[16:8], bsl_rem_lo ? width[7:0] : rem_sub[7:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire brm_b0, brm_b1_0, brm_b1_1, brm_b2_0, brm_b2_1;
	wire [5:0] brm_s1_0, brm_s1_1, brm_s2_0, brm_s2_1;
	am_sub #(.W(5)) s_brem_0  (.a(rem_b[4:0]),   .b(brm_17[4:0]),   .bi(1'b0), .d(rem_sub[4:0]), .bo(brm_b0));
	am_sub #(.W(6)) s_brem_10 (.a(rem_b[10:5]),  .b(brm_17[10:5]),  .bi(1'b0), .d(brm_s1_0),     .bo(brm_b1_0));
	am_sub #(.W(6)) s_brem_11 (.a(rem_b[10:5]),  .b(brm_17[10:5]),  .bi(1'b1), .d(brm_s1_1),     .bo(brm_b1_1));
	am_sub #(.W(6)) s_brem_20 (.a(rem_b[16:11]), .b(brm_17[16:11]), .bi(1'b0), .d(brm_s2_0),     .bo(brm_b2_0));
	am_sub #(.W(6)) s_brem_21 (.a(rem_b[16:11]), .b(brm_17[16:11]), .bi(1'b1), .d(brm_s2_1),     .bo(brm_b2_1));
	wire brm_b1 = brm_b1_0 | (brm_b1_1 & brm_b0);
	assign rem_sub[10:5]  = brm_b0 ? brm_s1_1 : brm_s1_0;
	assign rem_sub[16:11] = brm_b1 ? brm_s2_1 : brm_s2_0;
	wire rem_sub_bo = (brm_b1 ? brm_b2_1 : brm_b2_0);
	wire [16:0] rem_b_next = {bsl_rem_hi ? width[16:8] : rem_sub[16:8], bsl_rem_lo ? width[7:0] : rem_sub[7:0]};
`else
	wire brm_b8;
	am_sub #(.W(8)) s_brem_8 (.a(rem_b[7:0]), .b(brm_17[7:0]), .bi(1'b0), .d(rem_sub[7:0]), .bo(brm_b8));
	wire [8:0] rem_dec_m;
	assign rem_dec_m[0] = 1'b1;
	genvar g_rmh;
	generate
		for (g_rmh = 1; g_rmh < 9; g_rmh = g_rmh + 1) begin : g_rem_hi
			assign rem_dec_m[g_rmh] = ~(|rem_b[8 + g_rmh - 1 : 8]);
		end
	endgenerate
	wire [8:0] rem_hi_m1 = rem_b[16:8] ^ rem_dec_m;
	assign rem_sub[16:8] = brm_b8 ? rem_hi_m1 : rem_b[16:8];
	wire rem_sub_bo = 1'b0;
	wire [8:0] rem_hi_0 = bsl_rem_hi ? width[16:8] : rem_b[16:8];
	wire [8:0] rem_hi_1 = bsl_rem_hi ? width[16:8] : rem_hi_m1;
	wire [16:0] rem_b_next = {brm_b8 ? rem_hi_1 : rem_hi_0, bsl_rem_lo ? width[7:0] : rem_sub[7:0]};
`endif

	wire [US:0] bst_room_t_next = {{(US - 2){1'b0}}, tile_room_step};
	wire [US:0] bst_room_next_val = bst_tiled ? bst_room_t_next : (bsl_room ? rem_line_row : line_pixels_b);

	wire rem_b_cand_hi = (rem_b[16:12] == 5'd0);
	wire rem_b_cand_lo = (rem_b[11:US+2] == {(10-US){1'b0}}) && rem_b[US+1];
	wire rem_b_cand = rem_b_cand_hi && rem_b_cand_lo;
`ifdef SYNTH_KINTEX7
	wire rem_borrow_hi;
	LUT6 #(.INIT(64'hECFF00EC00000000)) u_rem_bo_hi (
		.I0(cmp_lo_lt), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
		.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(1'b1),
		.O(rem_borrow_hi)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rem_borrow_hi;
	LUT6 #(.INIT(64'hECFF00EC00000000)) u_rem_bo_hi (
		.I0(cmp_lo_lt), .I1(cmp_hi_lt), .I2(cmp_hi_eq),
		.I3(cmp_top_rem), .I4(cmp_top_rm), .I5(1'b1),
		.O(rem_borrow_hi)
	);
`else
	wire rem_borrow_hi = rem_b[US:0] < cur_bst_room;
`endif
	wire rem_b_small_walk = rem_b_small_rs || (rem_b_cand && rem_borrow_hi);


	/* Stage 1A: Walk Stage (computes shifted byte offset w_d_xb/w_s_xb, registers before a_bda/a_bsa) */
	wire [ADDR_W-1:0] d_px_b_lo0 = x_pixels(px_b, mdt_px_b_0);
	wire [ADDR_W-1:0] d_px_b_lo1 = x_pixels(px_b, mdt_px_b_0b);
	wire [ADDR_W-1:0] d_px_b_lo  = {d_px_b_lo1[ADDR_W-1:8], d_px_b_lo0[7:0]};
	wire [ADDR_W-1:0] d_px_b_hi  = x_pixels(px_b, mdt_px_b_1);
	wire [ADDR_W-1:0] s_px_b_lo0 = x_pixels(spx_b, mst_px_b_0);
	wire [ADDR_W-1:0] s_px_b_lo1 = x_pixels(spx_b, mst_px_b_0b);
	wire [ADDR_W-1:0] s_px_b_lo  = {s_px_b_lo1[ADDR_W-1:8], s_px_b_lo0[7:0]};
	wire [ADDR_W-1:0] s_px_b_hi  = x_pixels(spx_b, mst_px_b_1);
	wire [15:0] d_xb_b_lo = (pslw_dxb0_a == 3'd4) ? {d_px_b_lo[11:0], 4'b0000} :
	                        (pslw_dxb0_a == 3'd3) ? {d_px_b_lo[12:0], 3'b000}  :
	                        (pslw_dxb0_a == 3'd2) ? {d_px_b_lo[13:0], 2'b00}   :
	                        (pslw_dxb0_b == 3'd1) ? {d_px_b_lo[14:0], 1'b0}    : d_px_b_lo[15:0];
	wire [ADDR_W-17:0] d_xb_b_hi = (pslw_dxb1 == 3'd4) ? d_px_b_hi[ADDR_W-5:12] :
	                               (pslw_dxb1 == 3'd3) ? d_px_b_hi[ADDR_W-4:13] :
	                               (pslw_dxb1 == 3'd2) ? d_px_b_hi[ADDR_W-3:14] :
	                               (pslw_dxb1 == 3'd1) ? d_px_b_hi[ADDR_W-2:15] : d_px_b_hi[ADDR_W-1:16];
	wire [ADDR_W-1:0] d_xb_b = {d_xb_b_hi, d_xb_b_lo};

	wire [15:0] s_xb_b_lo = (pslw_sxb0_a == 3'd4) ? {s_px_b_lo[11:0], 4'b0000} :
	                        (pslw_sxb0_a == 3'd3) ? {s_px_b_lo[12:0], 3'b000}  :
	                        (pslw_sxb0_a == 3'd2) ? {s_px_b_lo[13:0], 2'b00}   :
	                        (pslw_sxb0_b == 3'd1) ? {s_px_b_lo[14:0], 1'b0}    : s_px_b_lo[15:0];
	wire [ADDR_W-17:0] s_xb_b_hi = (pslw_sxb1 == 3'd4) ? s_px_b_hi[ADDR_W-5:12] :
	                               (pslw_sxb1 == 3'd3) ? s_px_b_hi[ADDR_W-4:13] :
	                               (pslw_sxb1 == 3'd2) ? s_px_b_hi[ADDR_W-3:14] :
	                               (pslw_sxb1 == 3'd1) ? s_px_b_hi[ADDR_W-2:15] : s_px_b_hi[ADDR_W-1:16];
	wire [ADDR_W-1:0] s_xb_b = {s_xb_b_hi, s_xb_b_lo};

	always @(posedge clk) begin
		if (w_len_da0_a) w_dst_row[15:0]        <= dst_row_b[15:0];
		if (w_len_da1)   w_dst_row[ADDR_W-1:16] <= dst_row_b[ADDR_W-1:16];
		if (w_len_da0_b) w_d_xb[15:0]           <= d_xb_b[15:0];
		if (w_len_da1)   w_d_xb[ADDR_W-1:16]    <= d_xb_b[ADDR_W-1:16];
		if (w_len_sa0_a) w_src_row[15:0]        <= src_row_b[15:0];
		if (w_len_sa1)   w_src_row[ADDR_W-1:16] <= src_row_b[ADDR_W-1:16];
		if (w_len_sa0_b) w_s_xb[15:0]           <= s_xb_b[15:0];
		if (w_len_sa1)   w_s_xb[ADDR_W-1:16]    <= s_xb_b[ADDR_W-1:16];
		if (w_len_ctl) begin
			w_seg_n       <= bst_seg_n;
			w_seg_bytes   <= bst_seg_bytes;
			w_back_lo     <= bst_back_lo;
			w_seg_last    <= bst_seg_last;
			w_rows_b_zero <= rows_b_zero;
		end
	end

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [US:0] w_rev_off;
	wire wro_bo_unused;
	am_sub #(.W(US+1)) s_wro (.a(w_seg_bytes), .b({1'b0, w_back_lo}), .bi(1'b0), .d(w_rev_off), .bo(wro_bo_unused));
`endif

	/* Stage 1B: Address Addition (8+8+8+8 parallel carry-select, <= 10 cmos3, 0 Pre-LUTs) */
	wire [ADDR_W-1:0] g_da_full_next, g_sa_full_next;
	wire [1:0] g_full_co_next;
`ifdef SYNTH_KINTEX7
	wire bda_c0, bda_c10, bda_c11, bsa_c0, bsa_c10, bsa_c11;
	wire [ADDR_W-17:0] bda_s10, bda_s11, bsa_s10, bsa_s11;
	am_add #(.W(16))        a_bda_lo (.a(w_dst_row[15:0]),        .b(w_d_xb[15:0]),        .ci(1'b0), .s(g_da_full_next[15:0]), .co(bda_c0));
	am_add #(.W(ADDR_W-16)) a_bda_h0 (.a(w_dst_row[ADDR_W-1:16]), .b(w_d_xb[ADDR_W-1:16]), .ci(1'b0), .s(bda_s10),             .co(bda_c10));
	am_add #(.W(ADDR_W-16)) a_bda_h1 (.a(w_dst_row[ADDR_W-1:16]), .b(w_d_xb[ADDR_W-1:16]), .ci(1'b1), .s(bda_s11),             .co(bda_c11));
	assign g_da_full_next[ADDR_W-1:16] = bda_c0 ? bda_s11 : bda_s10;
	assign g_full_co_next[0] = (bda_c0 ? bda_c11 : bda_c10) & bda_c10;

	am_add #(.W(16))        a_bsa_lo (.a(w_src_row[15:0]),        .b(w_s_xb[15:0]),        .ci(1'b0), .s(g_sa_full_next[15:0]), .co(bsa_c0));
	am_add #(.W(ADDR_W-16)) a_bsa_h0 (.a(w_src_row[ADDR_W-1:16]), .b(w_s_xb[ADDR_W-1:16]), .ci(1'b0), .s(bsa_s10),             .co(bsa_c10));
	am_add #(.W(ADDR_W-16)) a_bsa_h1 (.a(w_src_row[ADDR_W-1:16]), .b(w_s_xb[ADDR_W-1:16]), .ci(1'b1), .s(bsa_s11),             .co(bsa_c11));
	assign g_sa_full_next[ADDR_W-1:16] = bsa_c0 ? bsa_s11 : bsa_s10;
	assign g_full_co_next[1] = (bsa_c0 ? bsa_c11 : bsa_c10) & bsa_c10;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bda_c0, bda_c10, bda_c11, bsa_c0, bsa_c10, bsa_c11;
	wire [ADDR_W-17:0] bda_s10, bda_s11, bsa_s10, bsa_s11;
	am_add #(.W(16))        a_bda_lo (.a(w_dst_row[15:0]),        .b(w_d_xb[15:0]),        .ci(1'b0), .s(g_da_full_next[15:0]), .co(bda_c0));
	am_add #(.W(ADDR_W-16)) a_bda_h0 (.a(w_dst_row[ADDR_W-1:16]), .b(w_d_xb[ADDR_W-1:16]), .ci(1'b0), .s(bda_s10),             .co(bda_c10));
	am_add #(.W(ADDR_W-16)) a_bda_h1 (.a(w_dst_row[ADDR_W-1:16]), .b(w_d_xb[ADDR_W-1:16]), .ci(1'b1), .s(bda_s11),             .co(bda_c11));
	assign g_da_full_next[ADDR_W-1:16] = bda_c0 ? bda_s11 : bda_s10;
	assign g_full_co_next[0] = (bda_c0 ? bda_c11 : bda_c10) & bda_c10;

	am_add #(.W(16))        a_bsa_lo (.a(w_src_row[15:0]),        .b(w_s_xb[15:0]),        .ci(1'b0), .s(g_sa_full_next[15:0]), .co(bsa_c0));
	am_add #(.W(ADDR_W-16)) a_bsa_h0 (.a(w_src_row[ADDR_W-1:16]), .b(w_s_xb[ADDR_W-1:16]), .ci(1'b0), .s(bsa_s10),             .co(bsa_c10));
	am_add #(.W(ADDR_W-16)) a_bsa_h1 (.a(w_src_row[ADDR_W-1:16]), .b(w_s_xb[ADDR_W-1:16]), .ci(1'b1), .s(bsa_s11),             .co(bsa_c11));
	assign g_sa_full_next[ADDR_W-1:16] = bsa_c0 ? bsa_s11 : bsa_s10;
	assign g_full_co_next[1] = (bsa_c0 ? bsa_c11 : bsa_c10) & bsa_c10;
`else
	am_add #(.W(ADDR_W)) a_bda (.a(w_dst_row), .b(w_d_xb), .ci(1'b0), .s(g_da_full_next), .co(g_full_co_next[0]));
	am_add #(.W(ADDR_W)) a_bsa (.a(w_src_row), .b(w_s_xb), .ci(1'b0), .s(g_sa_full_next), .co(g_full_co_next[1]));
`endif

	always @(posedge clk) begin
		if (g_len_da0) g_da_full[15:0]        <= g_da_full_next[15:0];
		if (g_len_da1) g_da_full[ADDR_W-1:16] <= g_da_full_next[ADDR_W-1:16];
		if (g_len_sa0) g_sa_full[15:0]        <= g_sa_full_next[15:0];
		if (g_len_sa1) g_sa_full[ADDR_W-1:16] <= g_sa_full_next[ADDR_W-1:16];
		if (g_len_ctl) begin
			g_seg_n       <= w_seg_n;
			g_seg_bytes   <= w_seg_bytes;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			g_rev_off     <= w_rev_off;
`endif
			g_back_lo     <= w_back_lo;
			g_seg_last    <= w_seg_last;
			g_rows_b_zero <= w_rows_b_zero;
		end
	end

	/* Stage 2A: Geom / Address Engine (Computed from g_* registers) */
	wire [US-1:0] g_da_lo = g_da_full[US-1:0];
	wire [US-1:0] g_sa_lo = g_sa_full[US-1:0];

	wire [US-1:0] g_dlo_lo_rev;
	wire g_dlo_lo_bo;
`ifdef SYNTH_KINTEX7
	wire [2:0] gd_hi0, gd_hi1;
	wire gd_bo_lo, gd_bo_h0, gd_bo_h1;
	am_sub #(.W(3))    s_dlo_lo_0  (.a(g_da_lo[2:0]),    .b(g_back_lo[2:0]),    .bi(1'b0), .d(g_dlo_lo_rev[2:0]), .bo(gd_bo_lo));
	am_sub #(.W(US-3)) s_dlo_lo_h0 (.a(g_da_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b0), .d(gd_hi0[US-4:0]),    .bo(gd_bo_h0));
	am_sub #(.W(US-3)) s_dlo_lo_h1 (.a(g_da_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b1), .d(gd_hi1[US-4:0]),    .bo(gd_bo_h1));
	assign g_dlo_lo_rev[US-1:3] = gd_bo_lo ? gd_hi1[US-4:0] : gd_hi0[US-4:0];
	assign g_dlo_lo_bo = gd_bo_lo ? gd_bo_h1 : gd_bo_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] gd_hi0, gd_hi1;
	wire gd_bo_lo, gd_bo_h0, gd_bo_h1;
	am_sub #(.W(3))    s_dlo_lo_0  (.a(g_da_lo[2:0]),    .b(g_back_lo[2:0]),    .bi(1'b0), .d(g_dlo_lo_rev[2:0]), .bo(gd_bo_lo));
	am_sub #(.W(US-3)) s_dlo_lo_h0 (.a(g_da_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b0), .d(gd_hi0[US-4:0]),    .bo(gd_bo_h0));
	am_sub #(.W(US-3)) s_dlo_lo_h1 (.a(g_da_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b1), .d(gd_hi1[US-4:0]),    .bo(gd_bo_h1));
	assign g_dlo_lo_rev[US-1:3] = gd_bo_lo ? gd_hi1[US-4:0] : gd_hi0[US-4:0];
	assign g_dlo_lo_bo = gd_bo_lo ? gd_bo_h1 : gd_bo_h0;
`else
	am_sub #(.W(US)) s_dlo_lo (.a(g_da_lo), .b(g_back_lo), .bi(1'b0), .d(g_dlo_lo_rev), .bo(g_dlo_lo_bo));
`endif
	wire [US-1:0] g_dlo_lo = mr_geom_dlo ? g_dlo_lo_rev : g_da_lo;

	wire [US-1:0] g_slo_lo_rev;
	wire g_slo_lo_bo;
`ifdef SYNTH_KINTEX7
	wire [2:0] gs_hi0, gs_hi1;
	wire gs_bo_lo, gs_bo_h0, gs_bo_h1;
	am_sub #(.W(3))    s_slo_lo_0  (.a(g_sa_lo[2:0]),    .b(g_back_lo[2:0]),    .bi(1'b0), .d(g_slo_lo_rev[2:0]), .bo(gs_bo_lo));
	am_sub #(.W(US-3)) s_slo_lo_h0 (.a(g_sa_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b0), .d(gs_hi0[US-4:0]),    .bo(gs_bo_h0));
	am_sub #(.W(US-3)) s_slo_lo_h1 (.a(g_sa_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b1), .d(gs_hi1[US-4:0]),    .bo(gs_bo_h1));
	assign g_slo_lo_rev[US-1:3] = gs_bo_lo ? gs_hi1[US-4:0] : gs_hi0[US-4:0];
	assign g_slo_lo_bo = gs_bo_lo ? gs_bo_h1 : gs_bo_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] gs_hi0, gs_hi1;
	wire gs_bo_lo, gs_bo_h0, gs_bo_h1;
	am_sub #(.W(3))    s_slo_lo_0  (.a(g_sa_lo[2:0]),    .b(g_back_lo[2:0]),    .bi(1'b0), .d(g_slo_lo_rev[2:0]), .bo(gs_bo_lo));
	am_sub #(.W(US-3)) s_slo_lo_h0 (.a(g_sa_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b0), .d(gs_hi0[US-4:0]),    .bo(gs_bo_h0));
	am_sub #(.W(US-3)) s_slo_lo_h1 (.a(g_sa_lo[US-1:3]), .b(g_back_lo[US-1:3]), .bi(1'b1), .d(gs_hi1[US-4:0]),    .bo(gs_bo_h1));
	assign g_slo_lo_rev[US-1:3] = gs_bo_lo ? gs_hi1[US-4:0] : gs_hi0[US-4:0];
	assign g_slo_lo_bo = gs_bo_lo ? gs_bo_h1 : gs_bo_h0;
`else
	am_sub #(.W(US)) s_slo_lo (.a(g_sa_lo), .b(g_back_lo), .bi(1'b0), .d(g_slo_lo_rev), .bo(g_slo_lo_bo));
`endif
	wire [US-1:0] g_slo_lo = mr_geom_slo ? g_slo_lo_rev : g_sa_lo;

	wire [LA_W-1:0] s_line = g_sa_full[ADDR_W-1:US];
	wire [LA_W-1:0] s_line_p1, s_line_m1;
	wire [1:0] s_line_co;
	am_add #(.W(LA_W)) a_s_p1 (.a(s_line), .b({LA_W{1'b0}}), .ci(1'b1), .s(s_line_p1), .co(s_line_co[0]));
	am_sub #(.W(LA_W)) s_s_m1 (.a(s_line), .b({LA_W{1'b0}}), .bi(1'b1), .d(s_line_m1), .bo(s_line_co[1]));

	wire [LA_W-1:0] d_line = g_da_full[ADDR_W-1:US];
	wire [LA_W-1:0] d_line_m1;
	wire d_line_bo;
	am_sub #(.W(LA_W)) s_d_m1 (.a(d_line), .b({LA_W{1'b0}}), .bi(1'b1), .d(d_line_m1), .bo(d_line_bo));

	wire slo_borrow = g_slo_lo_bo;
	wire dlo_borrow = g_dlo_lo_bo;

`ifdef SYNTH_KINTEX7
	wire [1:0] dlo_bor_buf, slo_bor_buf;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_db0 (.I0(dlo_borrow), .O(dlo_bor_buf[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_db1 (.I0(dlo_borrow), .O(dlo_bor_buf[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sb0 (.I0(slo_borrow), .O(slo_bor_buf[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sb1 (.I0(slo_borrow), .O(slo_bor_buf[1]));
	wire dlo_borrow_lo = dlo_bor_buf[0];
	wire dlo_borrow_hi = dlo_bor_buf[1];
	wire slo_borrow_lo = slo_bor_buf[0];
	wire slo_borrow_hi = slo_bor_buf[1];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] dlo_bor_buf, slo_bor_buf;
	assign dlo_bor_buf[0] = dlo_borrow;
	assign dlo_bor_buf[1] = dlo_borrow;
	assign slo_bor_buf[0] = slo_borrow;
	assign slo_bor_buf[1] = slo_borrow;
	wire dlo_borrow_lo = dlo_bor_buf[0];
	wire dlo_borrow_hi = dlo_bor_buf[1];
	wire slo_borrow_lo = slo_bor_buf[0];
	wire slo_borrow_hi = slo_bor_buf[1];
`else
	wire dlo_borrow_lo = dlo_borrow;
	wire dlo_borrow_hi = dlo_borrow;
	wire slo_borrow_lo = slo_borrow;
	wire slo_borrow_hi = slo_borrow;
`endif

	wire [LA_W-1:0] g_seg_line = {
		mr_geom_lin_d1 ? (dlo_borrow_hi ? d_line_m1[LA_W-1:14] : d_line[LA_W-1:14]) : d_line[LA_W-1:14],
		mr_geom_lin_d0 ? (dlo_borrow_lo ? d_line_m1[13:0]       : d_line[13:0])       : d_line[13:0]
	};
	wire [LA_W-1:0] g_src_n0 = {
		mr_geom_lin_s1 ? (slo_borrow_hi ? s_line_m1[LA_W-1:14] : s_line[LA_W-1:14]) : s_line[LA_W-1:14],
		mr_geom_lin_s0 ? (slo_borrow_lo ? s_line_m1[13:0]       : s_line[13:0])       : s_line[13:0]
	};
	wire [LA_W-1:0] g_src_n1 = {
		mr_geom_lin_p1_hi ? s_line[LA_W-1:16] : s_line_p1[LA_W-1:16],
		mr_geom_lin_p1_lo ? s_line[15:0]      : s_line_p1[15:0]
	};

	wire [US:0] g_s_end;
	wire g_send_co;
	am_add #(.W(US + 1)) a_bst_send (.a({1'b0, g_sa_lo}), .b(g_seg_bytes), .ci(1'b0), .s(g_s_end), .co(g_send_co));
	wire g_s_end_gt = g_s_end[US] && (|g_s_end[US-1:0]);
	wire g_two_lines = mr_geom_slo ? slo_borrow : g_s_end_gt;

	/* Parallel forward & reverse g_p_end via 3:2 CSA + fast add (<= 11 cmos3). */
	wire [US:0] g_pe_fwd, g_pe_rev_csa;
	wire [US:0] gpe_ca = {1'b0, g_da_lo};
	wire [US:0] gpe_cb = {1'b1, ~g_back_lo};
	wire [US:0] gpe_cc = g_seg_bytes;
	wire [US:0] gpe_cs_s = gpe_ca ^ gpe_cb ^ gpe_cc;
	wire [US:0] gpe_cs_k = {((gpe_ca[US-1:0] & gpe_cb[US-1:0]) | (gpe_ca[US-1:0] & gpe_cc[US-1:0]) | (gpe_cb[US-1:0] & gpe_cc[US-1:0])), 1'b1};
`ifdef SYNTH_KINTEX7
	wire gpf_c0, gpf_h0, gpf_h1, gpr_c0, gpr_h0, gpr_h1;
	wire [US-2:0] gpf_hi0, gpf_hi1, gpr_hi0, gpr_hi1;
	am_add #(.W(2))    a_gpe_f0  (.a(g_da_lo[1:0]),            .b(g_seg_bytes[1:0]),  .ci(1'b0), .s(g_pe_fwd[1:0]), .co(gpf_c0));
	am_add #(.W(US-1)) a_gpe_fh0 (.a({1'b0, g_da_lo[US-1:2]}), .b(g_seg_bytes[US:2]), .ci(1'b0), .s(gpf_hi0),       .co(gpf_h0));
	am_add #(.W(US-1)) a_gpe_fh1 (.a({1'b0, g_da_lo[US-1:2]}), .b(g_seg_bytes[US:2]), .ci(1'b1), .s(gpf_hi1),       .co(gpf_h1));
	assign g_pe_fwd[US:2] = gpf_c0 ? gpf_hi1 : gpf_hi0;

	am_add #(.W(2))    a_gpe_r0  (.a(gpe_cs_s[1:0]),  .b(gpe_cs_k[1:0]),  .ci(1'b0), .s(g_pe_rev_csa[1:0]), .co(gpr_c0));
	am_add #(.W(US-1)) a_gpe_rh0 (.a(gpe_cs_s[US:2]), .b(gpe_cs_k[US:2]), .ci(1'b0), .s(gpr_hi0),          .co(gpr_h0));
	am_add #(.W(US-1)) a_gpe_rh1 (.a(gpe_cs_s[US:2]), .b(gpe_cs_k[US:2]), .ci(1'b1), .s(gpr_hi1),          .co(gpr_h1));
	assign g_pe_rev_csa[US:2] = gpr_c0 ? gpr_hi1 : gpr_hi0;
	wire g_pend_co = (gpf_c0 ? gpf_h1 : gpf_h0) & (gpr_c0 ? gpr_h1 : gpr_h0);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire gpf_c0, gpf_h0, gpf_h1, gpr_c0, gpr_h0, gpr_h1;
	wire [US-2:0] gpf_hi0, gpf_hi1, gpr_hi0, gpr_hi1;
	am_add #(.W(2))    a_gpe_f0  (.a(g_da_lo[1:0]),            .b(g_seg_bytes[1:0]),  .ci(1'b0), .s(g_pe_fwd[1:0]), .co(gpf_c0));
	am_add #(.W(US-1)) a_gpe_fh0 (.a({1'b0, g_da_lo[US-1:2]}), .b(g_seg_bytes[US:2]), .ci(1'b0), .s(gpf_hi0),       .co(gpf_h0));
	am_add #(.W(US-1)) a_gpe_fh1 (.a({1'b0, g_da_lo[US-1:2]}), .b(g_seg_bytes[US:2]), .ci(1'b1), .s(gpf_hi1),       .co(gpf_h1));
	assign g_pe_fwd[US:2] = gpf_c0 ? gpf_hi1 : gpf_hi0;

	am_add #(.W(2))    a_gpe_r0  (.a(gpe_cs_s[1:0]),  .b(gpe_cs_k[1:0]),  .ci(1'b0), .s(g_pe_rev_csa[1:0]), .co(gpr_c0));
	am_add #(.W(US-1)) a_gpe_rh0 (.a(gpe_cs_s[US:2]), .b(gpe_cs_k[US:2]), .ci(1'b0), .s(gpr_hi0),          .co(gpr_h0));
	am_add #(.W(US-1)) a_gpe_rh1 (.a(gpe_cs_s[US:2]), .b(gpe_cs_k[US:2]), .ci(1'b1), .s(gpr_hi1),          .co(gpr_h1));
	assign g_pe_rev_csa[US:2] = gpr_c0 ? gpr_hi1 : gpr_hi0;
	wire g_pend_co = (gpf_c0 ? gpf_h1 : gpf_h0) & (gpr_c0 ? gpr_h1 : gpr_h0);
`else
	wire gpf_co, gpr_co;
	am_add #(.W(US+1)) a_gpe_f (.a({1'b0, g_da_lo}), .b(g_seg_bytes), .ci(1'b0), .s(g_pe_fwd),     .co(gpf_co));
	am_add #(.W(US+1)) a_gpe_r (.a({1'b0, g_da_lo}), .b(g_rev_off),   .ci(1'b0), .s(g_pe_rev_csa), .co(gpr_co));
	wire g_pend_co = gpf_co & gpr_co & (&{1'b0, gpe_cs_s, gpe_cs_k});
`endif
	wire [US:0] g_p_end = mr_geom_dlo ? g_pe_rev_csa : g_pe_fwd;

	wire [US-1:0] off_fwd;
	wire          bo_fwd;
	am_sub #(.W(US)) s_off_fwd (.a(g_sa_lo), .b(g_da_lo), .bi(1'b0), .d(off_fwd), .bo(bo_fwd));
`ifdef SYNTH_KINTEX7
	wire bo_rev_lo    = (g_slo_lo_rev[2:0] <  g_dlo_lo_rev[2:0]);
	wire bo_rev_hi_lt = (g_slo_lo_rev[US-1:3] <  g_dlo_lo_rev[US-1:3]);
	wire bo_rev_hi_eq = (g_slo_lo_rev[US-1:3] == g_dlo_lo_rev[US-1:3]);
	wire bo_rev = bo_rev_hi_lt || (bo_rev_hi_eq && bo_rev_lo);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bo_rev_lo    = (g_slo_lo_rev[2:0] <  g_dlo_lo_rev[2:0]);
	wire bo_rev_hi_lt = (g_slo_lo_rev[US-1:3] <  g_dlo_lo_rev[US-1:3]);
	wire bo_rev_hi_eq = (g_slo_lo_rev[US-1:3] == g_dlo_lo_rev[US-1:3]);
	wire bo_rev = bo_rev_hi_lt || (bo_rev_hi_eq && bo_rev_lo);
`else
	/* Circular-order identity: ((a - c) mod 2^US) < ((b - c) mod 2^US) in 3 parallel 6b compares (<= 9 cmos3). */
	wire circ_sa_lt_da = bo_fwd;
	wire circ_bk_le_sa = (g_back_lo <= g_sa_lo);
	wire circ_bk_le_da = (g_back_lo <= g_da_lo);
	wire bo_rev = circ_sa_lt_da ? (circ_bk_le_sa || !circ_bk_le_da) : (!circ_bk_le_da && circ_bk_le_sa);
`endif
	wire [US-1:0] g_src_off = off_fwd;
	wire          g_n0_slot = mr_geom_slo ? bo_rev : bo_fwd;

	/* Stage 2B: Pipeline register before u_skid_p (breaks Post-LUT=2 / 5-LUT paths into skid) */
	always @(posedge clk) begin
		if (g2_step_dl0) g2_seg_line[13:0]       <= g_seg_line[13:0];
		if (g2_step_dl1) g2_seg_line[LA_W-1:14]  <= g_seg_line[LA_W-1:14];
		if (g2_step_s00) g2_src_n0[13:0]         <= g_src_n0[13:0];
		if (g2_step_s01) g2_src_n0[LA_W-1:14]    <= g_src_n0[LA_W-1:14];
		if (g2_step_s10) g2_src_n1[13:0]         <= g_src_n1[13:0];
		if (g2_step_s11) g2_src_n1[LA_W-1:14]    <= g_src_n1[LA_W-1:14];
		if (g2_step_ctl_a) begin
			g2_seg_n      <= g_seg_n;
			g2_seg_bytes  <= g_seg_bytes;
			g2_p_end      <= g_p_end;
		end
		if (g2_step_ctl_b) begin
			g2_dlo_lo     <= g_dlo_lo;
			g2_src_off    <= g_src_off;
			g2_two_lines  <= g_two_lines;
			g2_n0_slot    <= g_n0_slot;
			g2_seg_last   <= g_seg_last;
			g2_rows_b_zero <= g_rows_b_zero;
		end
	end

	localparam SKID_W = 3 * LA_W + 3 * (US + 1) + 2 * US + 4;
	wire [SKID_W-1:0] skid_in_data = {
		g2_seg_line, g2_src_n0, g2_src_n1, g2_seg_n, g2_seg_bytes, g2_p_end,
		g2_dlo_lo, g2_src_off, g2_two_lines, g2_n0_slot, g2_seg_last, g2_rows_b_zero
	};
	wire [SKID_W-1:0] skid_out_data;
	wire [US:0] p_p_end;
	assign {
		p_dline, p_src_n0, p_src_n1, p_seg_n, p_seg_bytes, p_p_end,
		p_dlo_lo, p_off, p_two_lines, p_n0_slot, p_seg_last, p_rows_b_zero
	} = skid_out_data;

`ifdef SYNTH_KINTEX7
	wire g2_v_skid, g2_v_ctl;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g2vs (.I0(g2_valid), .O(g2_v_skid));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g2vc (.I0(g2_valid), .O(g2_v_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire g2_v_skid, g2_v_ctl;
	assign g2_v_skid = g2_valid;
	assign g2_v_ctl = g2_valid;
`else
	wire g2_v_skid = g2_valid;
	wire g2_v_ctl  = g2_valid;
`endif
	wire skid_clr = !(state_b4_1 && state_ctl[3:0] == S_BURST[3:0]);
	wire skid_in_ready;
	wire issue_rdy;

	atg_blt_skid #(.W(SKID_W)) u_skid_p (
		.clk(clk),
		.rst_n(rst_n),
		.clr(skid_clr),
		.in_valid(g2_v_skid),
		.in_ready(skid_in_ready),
		.in_data(skid_in_data),
		.out_valid(p_valid),
		.out_ready(issue_rdy),
		.out_data(skid_out_data)
	);

`ifdef SYNTH_KINTEX7
	wire [2:0] pv_buf;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pvb0 (.I0(p_valid), .O(pv_buf[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pvb1 (.I0(p_valid), .O(pv_buf[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pvb2 (.I0(p_valid), .O(pv_buf[2]));
	wire p_valid_fill = pv_buf[0];
	wire p_valid_iss0 = pv_buf[1];
	wire p_valid_iss1 = pv_buf[2];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] pv_buf;
	assign pv_buf[0] = p_valid;
	assign pv_buf[1] = p_valid;
	assign pv_buf[2] = p_valid;
	wire p_valid_fill = pv_buf[0];
	wire p_valid_iss0 = pv_buf[1];
	wire p_valid_iss1 = pv_buf[2];
`else
	wire p_valid_fill = p_valid;
	wire p_valid_iss0 = p_valid;
	wire p_valid_iss1 = p_valid;
`endif

	/* The port: the request, the outstanding requests, the line buffers. */
	wire q_free, q_free_leg, q_free_st, q_free_fsm, rd_idle, wr_idle, fail_raw;
`ifdef SYNTH_KINTEX7
	/* fail_raw buffer tree: 8 leaves, each <= 14 loads */
	wire [7:0] fail_buf;
	genvar g_fbi;
	generate
		for (g_fbi = 0; g_fbi < 8; g_fbi = g_fbi + 1) begin : g_fail_b
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fb (.I0(fail_raw), .O(fail_buf[g_fbi]));
		end
	endgenerate
	wire fail_port   = fail_buf[0];
	wire fail_bst_rd = fail_buf[1];
	wire fail_bst_wr = fail_buf[2];
	wire fail_burst  = fail_buf[3];
	wire fail_fsm0   = fail_buf[4];
	wire fail_fsm1   = fail_buf[5];
	wire fail_fsm2   = fail_buf[6];
	wire fail_fsm3   = fail_buf[7];
	wire fail        = fail_raw;
`elsif SYNTH_KINTEX_ULTRASCALE
	/* fail_raw buffer tree: 8 leaves, each <= 14 loads */
	wire [7:0] fail_buf;
	genvar g_fbi;
	generate
		for (g_fbi = 0; g_fbi < 8; g_fbi = g_fbi + 1) begin : g_fail_b
			assign fail_buf[g_fbi] = fail_raw;
		end
	endgenerate
	wire fail_port   = fail_buf[0];
	wire fail_bst_rd = fail_buf[1];
	wire fail_bst_wr = fail_buf[2];
	wire fail_burst  = fail_buf[3];
	wire fail_fsm0   = fail_buf[4];
	wire fail_fsm1   = fail_buf[5];
	wire fail_fsm2   = fail_buf[6];
	wire fail_fsm3   = fail_buf[7];
	wire fail        = fail_raw;
`else
	wire fail_port   = fail_raw;
	wire fail_bst_rd = fail_raw;
	wire fail_bst_wr = fail_raw;
	wire fail_burst  = fail_raw;
	wire fail_fsm0   = fail_raw;
	wire fail_fsm1   = fail_raw;
	wire fail_fsm2   = fail_raw;
	wire fail_fsm3   = fail_raw;
	wire fail        = fail_raw;
`endif
	wire [LINE_BITS-1:0] buf0, buf1, bufd;

	wire crd_cond = (r_st_crd && q_free_leg && wr_idle && !fail_raw);
	wire crd_c0 = crd_cond;
	wire crd_c1 = crd_cond;
	wire crd_c2 = crd_cond;
	wire rd_go_lo = crd_c0 && (rd_first || rd_second);
	wire rd_go_hi = crd_c1 && (rd_first || rd_second);
	wire rd_go_ctl = crd_c2 && (rd_first || rd_second);
	wire rd_go = rd_go_ctl;
	wire drd_go = r_st_drd && q_free_leg && wr_idle && !fail_raw;
	wire can_wr = q_free_leg && !fail_raw;
	wire wr_go = r_st_wr && can_wr;

	wire leg_req_needed = r_st_wr || (wr_idle && (r_st_drd || (r_st_crd && (rd_first || rd_second))));
	wire leg_issue = q_free_leg && !fail_raw && leg_req_needed;
	wire wr_go_ctl = wr_go;

`ifdef SYNTH_KINTEX7
	wire mc_fsm = m_copy, mc_issue = m_copy, mc_bk0 = m_copy, mc_bk1 = m_copy;
	wire req_rdy_i = req_ready;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire mc_fsm = m_copy, mc_issue = m_copy, mc_bk0 = m_copy, mc_bk1 = m_copy;
	wire req_rdy_i = req_ready;
`else
	wire mc_fsm = m_copy, mc_issue = m_copy_ctl_r, mc_bk0 = m_copy, mc_bk1 = m_copy;
	wire req_rdy_i;
`endif

	/* The copied line: the source window shifted by whole bytes via atg_blt_align. */
	wire [2*LINE_BITS-1:0] both = {buf1, buf0};
	wire [LINE_BITS-1:0] copy_line;

`ifdef SYNTH_KINTEX7
	atg_blt_align #(
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.OFF_REPS(8)
	) leg_align (
		.in_data(both),
		.off(src_off),
		.off_rep(src_off_rep),
		.out_data(copy_line)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	atg_blt_align #(
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.OFF_REPS(8)
	) leg_align (
		.in_data(both),
		.off(src_off),
		.off_rep(src_off_rep),
		.out_data(copy_line)
	);
`else
	(* keep = "true" *) reg [63:0] leg_s0_oh_rep, leg_s1_oh_rep;
	wire [7:0] leg_s0_oh_next = {
		 src_base_u[2] &  src_base_u[1] &  src_base_u[0],
		 src_base_u[2] &  src_base_u[1] & ~src_base_u[0],
		 src_base_u[2] & ~src_base_u[1] &  src_base_u[0],
		 src_base_u[2] & ~src_base_u[1] & ~src_base_u[0],
		~src_base_u[2] &  src_base_u[1] &  src_base_u[0],
		~src_base_u[2] &  src_base_u[1] & ~src_base_u[0],
		~src_base_u[2] & ~src_base_u[1] &  src_base_u[0],
		~src_base_u[2] & ~src_base_u[1] & ~src_base_u[0]
	};
	wire [2:0] leg_sel1_next;
	generate
		if (LINE_BYTES >= 64) begin : g_lsel1_64
			assign leg_sel1_next = src_base_u[5:3];
		end else begin : g_lsel1_32
			assign leg_sel1_next = {1'b0, src_base_u[4:3]};
		end
	endgenerate
	wire [7:0] leg_s1_oh_next = {
		 leg_sel1_next[2] &  leg_sel1_next[1] &  leg_sel1_next[0],
		 leg_sel1_next[2] &  leg_sel1_next[1] & ~leg_sel1_next[0],
		 leg_sel1_next[2] & ~leg_sel1_next[1] &  leg_sel1_next[0],
		 leg_sel1_next[2] & ~leg_sel1_next[1] & ~leg_sel1_next[0],
		~leg_sel1_next[2] &  leg_sel1_next[1] &  leg_sel1_next[0],
		~leg_sel1_next[2] &  leg_sel1_next[1] & ~leg_sel1_next[0],
		~leg_sel1_next[2] & ~leg_sel1_next[1] &  leg_sel1_next[0],
		~leg_sel1_next[2] & ~leg_sel1_next[1] & ~leg_sel1_next[0]
	};
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			leg_s0_oh_rep <= 64'd0;
			leg_s1_oh_rep <= 64'd0;
		end else begin
			if (sd_off0) begin leg_s0_oh_rep[15:0]  <= {2{leg_s0_oh_next}}; leg_s1_oh_rep[15:0]  <= {2{leg_s1_oh_next}}; end
			if (sd_off1) begin leg_s0_oh_rep[31:16] <= {2{leg_s0_oh_next}}; leg_s1_oh_rep[31:16] <= {2{leg_s1_oh_next}}; end
			if (sd_off2) begin leg_s0_oh_rep[47:32] <= {2{leg_s0_oh_next}}; leg_s1_oh_rep[47:32] <= {2{leg_s1_oh_next}}; end
			if (sd_off3) begin leg_s0_oh_rep[63:48] <= {2{leg_s0_oh_next}}; leg_s1_oh_rep[63:48] <= {2{leg_s1_oh_next}}; end
		end
	end
	localparam integer LEG_S0_OUT_A     = (LINE_BYTES >= 64) ? (LINE_BYTES + 56) : (LINE_BYTES + 24);
	localparam integer LEG_S0A_LEAF_CNT = (LEG_S0_OUT_A + 1) / 2;
	localparam integer LEG_S1A_LEAF_CNT = (LINE_BYTES + 1) / 2;
	wire [7:0] leg_in_b [0:2*LINE_BYTES-1];
	wire [7:0] leg_st0_b [0:LEG_S0_OUT_A-1];
	genvar lg_u, lg_0, lg_1;
	generate
		for (lg_u = 0; lg_u < 2 * LINE_BYTES; lg_u = lg_u + 1) begin : g_leg_in
			assign leg_in_b[lg_u] = both[lg_u*8 +: 8];
		end
		for (lg_0 = 0; lg_0 < LEG_S0_OUT_A; lg_0 = lg_0 + 1) begin : g_leg_st0
			localparam integer LR0 = ((lg_0 / 2) * 8) / LEG_S0A_LEAF_CNT;
			wire [7:0] loh0 = leg_s0_oh_rep[LR0*8 +: 8];
			assign leg_st0_b[lg_0] =
				(({8{loh0[7]}} & leg_in_b[lg_0 + 7]) | ({8{loh0[6]}} & leg_in_b[lg_0 + 6])) |
				(({8{loh0[5]}} & leg_in_b[lg_0 + 5]) | ({8{loh0[4]}} & leg_in_b[lg_0 + 4])) |
				(({8{loh0[3]}} & leg_in_b[lg_0 + 3]) | ({8{loh0[2]}} & leg_in_b[lg_0 + 2])) |
				(({8{loh0[1]}} & leg_in_b[lg_0 + 1]) | ({8{loh0[0]}} & leg_in_b[lg_0]));
		end
		if (LINE_BYTES >= 64) begin : g_leg_s1_64
			for (lg_1 = 0; lg_1 < LINE_BYTES; lg_1 = lg_1 + 1) begin : g_leg_st1
				localparam integer LR1 = ((lg_1 / 2) * 8) / LEG_S1A_LEAF_CNT;
				wire [7:0] loh1 = leg_s1_oh_rep[LR1*8 +: 8];
				assign copy_line[lg_1*8 +: 8] =
					(({8{loh1[7]}} & leg_st0_b[lg_1 + 56]) | ({8{loh1[6]}} & leg_st0_b[lg_1 + 48])) |
					(({8{loh1[5]}} & leg_st0_b[lg_1 + 40]) | ({8{loh1[4]}} & leg_st0_b[lg_1 + 32])) |
					(({8{loh1[3]}} & leg_st0_b[lg_1 + 24]) | ({8{loh1[2]}} & leg_st0_b[lg_1 + 16])) |
					(({8{loh1[1]}} & leg_st0_b[lg_1 + 8])  | ({8{loh1[0]}} & leg_st0_b[lg_1]));
			end
		end else begin : g_leg_s1_32
			for (lg_1 = 0; lg_1 < LINE_BYTES; lg_1 = lg_1 + 1) begin : g_leg_st1
				localparam integer LR1 = ((lg_1 / 2) * 8) / LEG_S1A_LEAF_CNT;
				wire [7:0] loh1 = leg_s1_oh_rep[LR1*8 +: 8];
				assign copy_line[lg_1*8 +: 8] =
					(({8{loh1[3]}} & leg_st0_b[lg_1 + 24]) | ({8{loh1[2]}} & leg_st0_b[lg_1 + 16])) |
					(({8{loh1[1]}} & leg_st0_b[lg_1 + 8])  | ({8{loh1[0]}} & leg_st0_b[lg_1]));
			end
		end
	endgenerate
	wire leg_off_unused = &{1'b0, src_off, src_off_rep};
`endif

	/* Pipeline copy_line across S_CWAIT so leg_wdata & r_bl_src see clean FF (0-cycle latency cost). */
	reg [LINE_BITS-1:0] r_copy_line;
`ifdef SYNTH_KINTEX7
	localparam integer NUM_CP_CE = LINE_BITS / 16;
	localparam integer NUM_CP_MID = (NUM_CP_CE + 7) / 8;
	wire [NUM_CP_CE-1:0] cp_ce_buf;
	wire [NUM_CP_MID-1:0] cp_ce_mid;
	genvar g_cpm, g_cpi;
	generate
		for (g_cpm = 0; g_cpm < NUM_CP_MID; g_cpm = g_cpm + 1) begin : g_cp_cem
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cpcm (.I0(r_st_cwait), .O(cp_ce_mid[g_cpm]));
		end
		for (g_cpi = 0; g_cpi < NUM_CP_CE; g_cpi = g_cpi + 1) begin : g_cp_ce
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cpcl (.I0(cp_ce_mid[g_cpi / 8]), .O(cp_ce_buf[g_cpi]));
			always @(posedge clk) begin
				if (cp_ce_buf[g_cpi])
					r_copy_line[g_cpi*16 +: 16] <= copy_line[g_cpi*16 +: 16];
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam integer NUM_CP_CE = LINE_BITS / 16;
	localparam integer NUM_CP_MID = (NUM_CP_CE + 7) / 8;
	wire [NUM_CP_CE-1:0] cp_ce_buf;
	wire [NUM_CP_MID-1:0] cp_ce_mid;
	genvar g_cpm, g_cpi;
	generate
		for (g_cpm = 0; g_cpm < NUM_CP_MID; g_cpm = g_cpm + 1) begin : g_cp_cem
			assign cp_ce_mid[g_cpm] = r_st_cwait;
		end
		for (g_cpi = 0; g_cpi < NUM_CP_CE; g_cpi = g_cpi + 1) begin : g_cp_ce
			assign cp_ce_buf[g_cpi] = cp_ce_mid[g_cpi / 8];
			always @(posedge clk) begin
				if (cp_ce_buf[g_cpi])
					r_copy_line[g_cpi*16 +: 16] <= copy_line[g_cpi*16 +: 16];
			end
		end
	endgenerate
`else
	always @(posedge clk) begin
		if (r_st_cwait)
			r_copy_line <= copy_line;
	end
`endif

	/* Blend-over. */
	wire bl_done;
	wire [LINE_BITS-1:0] bl_out;
	reg bl_go;

`ifdef SYNTH_KINTEX7
	localparam NUM_LEG_BYTE_BUFS = LINE_BYTES;
	wire [LINE_BITS-1:0] bl_src;
	wire [LINE_BITS-1:0] leg_wdata;
	wire [NUM_LEG_BYTE_BUFS-1:0] mc_bl_slice;
	wire [NUM_LEG_BYTE_BUFS-1:0] mb_bl_slice;
	wire [7:0] mc_bl_mid, mb_bl_mid;
	genvar bsm, bsi;
	generate
		for (bsm = 0; bsm < 8; bsm = bsm + 1) begin : g_bl_mid
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mc_m (.I0(m_copy),  .O(mc_bl_mid[bsm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mb_m (.I0(m_blend), .O(mb_bl_mid[bsm]));
		end
		for (bsi = 0; bsi < NUM_LEG_BYTE_BUFS; bsi = bsi + 1) begin : g_bl_src
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mc_s (.I0(mc_bl_mid[bsi / 8]), .O(mc_bl_slice[bsi]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mb_s (.I0(mb_bl_mid[bsi / 8]), .O(mb_bl_slice[bsi]));
			assign bl_src[bsi*8 +: 8] = mc_bl_slice[bsi] ? r_copy_line[bsi*8 +: 8] : fill_line[bsi*8 +: 8];
			assign leg_wdata[bsi*8 +: 8] = mb_bl_slice[bsi] ? bl_out[bsi*8 +: 8] :
				(mc_bl_slice[bsi] ? r_copy_line[bsi*8 +: 8] : fill_line[bsi*8 +: 8]);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam NUM_LEG_BYTE_BUFS = LINE_BYTES;
	wire [LINE_BITS-1:0] bl_src;
	wire [LINE_BITS-1:0] leg_wdata;
	wire [NUM_LEG_BYTE_BUFS-1:0] mc_bl_slice;
	wire [NUM_LEG_BYTE_BUFS-1:0] mb_bl_slice;
	wire [7:0] mc_bl_mid, mb_bl_mid;
	genvar bsm, bsi;
	generate
		for (bsm = 0; bsm < 8; bsm = bsm + 1) begin : g_bl_mid
			assign mc_bl_mid[bsm] = m_copy;
			assign mb_bl_mid[bsm] = m_blend;
		end
		for (bsi = 0; bsi < NUM_LEG_BYTE_BUFS; bsi = bsi + 1) begin : g_bl_src
			assign mc_bl_slice[bsi] = mc_bl_mid[bsi / 8];
			assign mb_bl_slice[bsi] = mb_bl_mid[bsi / 8];
			assign bl_src[bsi*8 +: 8] = mc_bl_slice[bsi] ? r_copy_line[bsi*8 +: 8] : fill_line[bsi*8 +: 8];
			assign leg_wdata[bsi*8 +: 8] = mb_bl_slice[bsi] ? bl_out[bsi*8 +: 8] :
				(mc_bl_slice[bsi] ? r_copy_line[bsi*8 +: 8] : fill_line[bsi*8 +: 8]);
		end
	endgenerate
`else
	localparam NUM_WD_BUFS = LINE_BITS / 16;
	localparam integer WD_SL_STEP = (NUM_WD_BUFS >= 8) ? (NUM_WD_BUFS / 8) : 1;
	wire [LINE_BITS-1:0] bl_src;
	genvar bsi;
	generate
		for (bsi = 0; bsi < NUM_WD_BUFS; bsi = bsi + 1) begin : g_bl_src
			assign bl_src[bsi*16 +: 16] = m_copy_wd_rep[bsi / WD_SL_STEP] ? r_copy_line[bsi*16 +: 16] : fill_line[bsi*16 +: 16];
		end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
	localparam integer NUM_BL_CE = LINE_BITS / 16;
	localparam integer NUM_BL_MID = (NUM_BL_CE + 7) / 8;
	reg [LINE_BITS-1:0] r_bl_src;
	wire ce_bl_raw = (r_st_dwait || r_st_cwait);
	wire [NUM_BL_CE-1:0] bl_ce_buf;
	wire [NUM_BL_MID-1:0] bl_ce_mid;
	genvar g_blm, g_bli;
	generate
		for (g_blm = 0; g_blm < NUM_BL_MID; g_blm = g_blm + 1) begin : g_bl_cem
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_blcm (.I0(ce_bl_raw), .O(bl_ce_mid[g_blm]));
		end
		for (g_bli = 0; g_bli < NUM_BL_CE; g_bli = g_bli + 1) begin : g_bl_ce
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_blcl (.I0(bl_ce_mid[g_bli / 8]), .O(bl_ce_buf[g_bli]));
			always @(posedge clk) begin
				if (bl_ce_buf[g_bli])
					r_bl_src[g_bli*16 +: 16] <= bl_src[g_bli*16 +: 16];
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam integer NUM_BL_CE = LINE_BITS / 16;
	localparam integer NUM_BL_MID = (NUM_BL_CE + 7) / 8;
	reg [LINE_BITS-1:0] r_bl_src;
	wire ce_bl_raw = (r_st_dwait || r_st_cwait);
	wire [NUM_BL_CE-1:0] bl_ce_buf;
	wire [NUM_BL_MID-1:0] bl_ce_mid;
	genvar g_blm, g_bli;
	generate
		for (g_blm = 0; g_blm < NUM_BL_MID; g_blm = g_blm + 1) begin : g_bl_cem
			assign bl_ce_mid[g_blm] = ce_bl_raw;
		end
		for (g_bli = 0; g_bli < NUM_BL_CE; g_bli = g_bli + 1) begin : g_bl_ce
			assign bl_ce_buf[g_bli] = bl_ce_mid[g_bli / 8];
			always @(posedge clk) begin
				if (bl_ce_buf[g_bli])
					r_bl_src[g_bli*16 +: 16] <= bl_src[g_bli*16 +: 16];
			end
		end
	endgenerate
`endif

	generate
	if (ENABLE_BLEND != 0) begin : gen_bl
`ifdef SYNTH_KINTEX7
		atg_blt_blend #(.LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT)) bl (
			.clk(clk), .rst_n(rst_n), .go(bl_go), .src(r_bl_src),
			.dst(bufd), .done(bl_done), .out(bl_out)
		);
`elsif SYNTH_KINTEX_ULTRASCALE
		atg_blt_blend #(.LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT)) bl (
			.clk(clk), .rst_n(rst_n), .go(bl_go), .src(r_bl_src),
			.dst(bufd), .done(bl_done), .out(bl_out)
		);
`else
		atg_blt_blend #(.LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT)) bl (
			.clk(clk), .rst_n(rst_n), .go(bl_go), .src(bl_src),
			.dst(bufd), .done(bl_done), .out(bl_out)
		);
`endif
	end else begin : gen_no_bl
		assign bl_done = 1'b1;
		assign bl_out = {LINE_BITS{1'b0}};
	end
	endgenerate

	/* Legacy port instance */
	wire leg_req_valid, leg_req_write;
	wire [LA_W-1:0] leg_req_line;
	wire [LINE_BYTES-1:0] leg_req_be;
	wire [LINE_BITS-1:0] leg_req_wdata;
	wire [7:0] leg_req_tag;

`ifdef SYNTH_KINTEX7
	wire rd_first_lo, rd_first_hi;
	wire rd_go_line_lo, rd_go_line_hi;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rfl (.I0(rd_first), .O(rd_first_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rfh (.I0(rd_first), .O(rd_first_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rgl (.I0(rd_go_lo), .O(rd_go_line_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rgh (.I0(rd_go_hi), .O(rd_go_line_hi));

	wire [LA_W-1:0] port_i_line = {
		rd_go_line_hi ? (rd_first_hi ? src_n0[LA_W-1:14] : src_n1[LA_W-1:14]) : seg_line[LA_W-1:14],
		rd_go_line_lo ? (rd_first_lo ? src_n0[13:0] : src_n1[13:0]) : seg_line[13:0]
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rd_first_lo, rd_first_hi;
	wire rd_go_line_lo, rd_go_line_hi;
	assign rd_first_lo = rd_first;
	assign rd_first_hi = rd_first;
	assign rd_go_line_lo = rd_go_lo;
	assign rd_go_line_hi = rd_go_hi;

	wire [LA_W-1:0] port_i_line = {
		rd_go_line_hi ? (rd_first_hi ? src_n0[LA_W-1:14] : src_n1[LA_W-1:14]) : seg_line[LA_W-1:14],
		rd_go_line_lo ? (rd_first_lo ? src_n0[13:0] : src_n1[13:0]) : seg_line[13:0]
	};
`else
	wire [LA_W-1:0] port_i_line = {
		rd_go_hi ? (rd_first ? src_n0[LA_W-1:14] : src_n1[LA_W-1:14]) : seg_line[LA_W-1:14],
		rd_go_lo ? (rd_first ? src_n0[13:0] : src_n1[13:0]) : seg_line[13:0]
	};

	wire [LINE_BITS-1:0] leg_wdata;
	genvar lwi;
	generate
		for (lwi = 0; lwi < NUM_WD_BUFS; lwi = lwi + 1) begin : g_leg_wd
			wire l_bl = m_blend_wd_rep[lwi / WD_SL_STEP];
			wire l_cp = m_copy_wd_rep[lwi / WD_SL_STEP] && !l_bl;
			wire [15:0] leg_alt = l_bl ? bl_out[lwi*16 +: 16] : fill_line[lwi*16 +: 16];
			assign leg_wdata[lwi*16 +: 16] = l_cp ? r_copy_line[lwi*16 +: 16] : leg_alt;
		end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
	wire rsp_valid_port, rsp_valid_bst, rsp_valid_fsm;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rv_port (.I0(rsp_valid), .O(rsp_valid_port));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rv_bst  (.I0(rsp_valid), .O(rsp_valid_bst));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rv_fsm  (.I0(rsp_valid), .O(rsp_valid_fsm));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rsp_valid_port, rsp_valid_bst, rsp_valid_fsm;
	assign rsp_valid_port = rsp_valid;
	assign rsp_valid_bst = rsp_valid;
	assign rsp_valid_fsm = rsp_valid;
`else
	wire rsp_valid_port = rsp_valid;
	wire rsp_valid_bst  = rsp_valid;
	wire rsp_valid_fsm  = rsp_valid;
`endif

	atg_blt_port #(.ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT)) port (
		.clk(clk), .rst_n(rst_n), .clr(r_st_idle && kick),
		.issue(leg_issue), .i_write(wr_go_ctl),
		.i_line(port_i_line),
		.i_slot(rd_go ? (rd_first ? {1'b0, n0_slot} : 2'd1) : drd_go ? 2'd2 : 2'd0),
		.i_be(seg_be),
		.i_wdata(leg_wdata),
		.free(q_free), .free_leg(q_free_leg), .free_st(q_free_st), .free_fsm(q_free_fsm), .rd_idle(rd_idle), .wr_idle(wr_idle), .fail(fail_raw),
		.buf0(buf0), .buf1(buf1), .bufd(bufd),
		.req_valid(leg_req_valid), .req_ready(req_rdy_i && !m_burst), .req_write(leg_req_write),
		.req_line(leg_req_line), .req_be(leg_req_be), .req_wdata(leg_req_wdata), .req_tag(leg_req_tag),
		.rsp_valid(rsp_valid_port), .rsp_inhibit(m_burst), .rsp_write(rsp_write), .rsp_tag(rsp_tag),
		.rsp_rdata(rsp_rdata), .rsp_error(rsp_error)
	);

	/*
	 * =========================================================================
	 * Burst Engine Read/Write Data Path & Barrel Shifter
	 * =========================================================================
	 */
	/* Read-out from ping-pong RAM (3-stage pipeline: Stage 0 -> s1 -> s2 -> s3) */
	localparam NUM_RAM_SLICES  = LINE_BITS / 4;
	localparam NUM_RAM_GRPS    = NUM_RAM_SLICES / 8;
	localparam NUM_RADDR_REPS  = NUM_RAM_SLICES / 4;
	localparam PP_S1_OUT_BYTES = (LINE_BYTES >= 64) ? (LINE_BYTES + 48) : (LINE_BYTES + 16);
	localparam PP_S0_OUT_BYTES = PP_S1_OUT_BYTES + 12;
	localparam PP_S0_LEAF_CNT  = (PP_S0_OUT_BYTES + 1) / 2;
	localparam PP_S1_LEAF_CNT  = (PP_S1_OUT_BYTES + 1) / 2;
	localparam PP_S2_LEAF_CNT  = (LINE_BYTES + 1) / 2;

	reg s1_valid, s2_valid, s3_valid;
	reg [LA_W-1:0] s1_line, s2_line, s3_line;
	reg [LINE_BYTES-1:0] s1_be, s2_be, s3_be;
	reg [5:2] s1_off_hi;
	reg [LINE_BITS-1:0] s3_wdata;

`ifdef SYNTH_KINTEX7
	wire br_pipe_rdy_raw;
	wire [9:0]   br_rdy_mid;
	wire [149:0] br_rdy_leaf;
	LUT2 #(.INIT(4'hD)) u_bpr_raw (.I0(s3_valid), .I1(req_ready), .O(br_pipe_rdy_raw));
	genvar g_bpr_m, g_bpr_l;
	generate
		for (g_bpr_m = 0; g_bpr_m < 10; g_bpr_m = g_bpr_m + 1) begin : g_bpr_mid
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bprm (.I0(br_pipe_rdy_raw), .O(br_rdy_mid[g_bpr_m]));
		end
		for (g_bpr_l = 0; g_bpr_l < 150; g_bpr_l = g_bpr_l + 1) begin : g_bpr_leaf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bprl (.I0(br_rdy_mid[g_bpr_l / 15]), .O(br_rdy_leaf[g_bpr_l]));
		end
	endgenerate
	wire br_step_en = !s3_valid || req_ready;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire br_pipe_rdy_raw;
	wire [9:0]   br_rdy_mid;
	wire [149:0] br_rdy_leaf;
	LUT2 #(.INIT(4'hD)) u_bpr_raw (.I0(s3_valid), .I1(req_ready), .O(br_pipe_rdy_raw));
	genvar g_bpr_m, g_bpr_l;
	generate
		for (g_bpr_m = 0; g_bpr_m < 10; g_bpr_m = g_bpr_m + 1) begin : g_bpr_mid
			assign br_rdy_mid[g_bpr_m] = br_pipe_rdy_raw;
		end
		for (g_bpr_l = 0; g_bpr_l < 150; g_bpr_l = g_bpr_l + 1) begin : g_bpr_leaf
			assign br_rdy_leaf[g_bpr_l] = br_rdy_mid[g_bpr_l / 15];
		end
	endgenerate
	wire br_step_en = !s3_valid || req_ready;
`else
	(* keep = "true" *) reg [7:0] s3_v_rep;
	wire [7:0] br_rdy_sl;
	genvar g_brs;
	generate
		for (g_brs = 0; g_brs < 8; g_brs = g_brs + 1) begin : g_br_rdy_sl
			assign br_rdy_sl[g_brs] = !s3_v_rep[g_brs] || req_rdy_i;
		end
	endgenerate
	wire br_pipe_rdy = !s3_valid || req_rdy_i;
	wire br_step_en  = br_pipe_rdy;
`endif

`ifdef SYNTH_KINTEX7
	wire br_bank_root0, br_bank_root1;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbr0 (.I0(br_bank), .O(br_bank_root0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbr1 (.I0(br_bank), .O(br_bank_root1));

	wire [3:0] br_bank_ram;
	wire br_bank_l0_lo, br_bank_l0_hi, br_bank_l1_lo, br_bank_l1_hi, br_bank_l2, br_bank_l3;
	wire br_bank_b0, br_bank_b1, br_bank_b2, br_bank_b3;
	wire br_bank_b4, br_bank_b5, br_bank_b6, br_bank_b7;
	wire br_bank_s0, br_bank_s1, br_bank_m0, br_bank_m1;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbram0 (.I0(br_bank_root1), .O(br_bank_ram[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbram1 (.I0(br_bank_root1), .O(br_bank_ram[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbram2 (.I0(br_bank_root1), .O(br_bank_ram[2]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbram3 (.I0(br_bank_root1), .O(br_bank_ram[3]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl0l (.I0(br_bank_root0), .O(br_bank_l0_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl0h (.I0(br_bank_root0), .O(br_bank_l0_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl1l (.I0(br_bank_root0), .O(br_bank_l1_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl1h (.I0(br_bank_root0), .O(br_bank_l1_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl2  (.I0(br_bank_root0), .O(br_bank_l2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbl3  (.I0(br_bank_root0), .O(br_bank_l3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb0  (.I0(br_bank_root0), .O(br_bank_b0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb1  (.I0(br_bank_root0), .O(br_bank_b1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb2  (.I0(br_bank_root0), .O(br_bank_b2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb3  (.I0(br_bank_root0), .O(br_bank_b3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb4  (.I0(br_bank_root1), .O(br_bank_b4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb5  (.I0(br_bank_root1), .O(br_bank_b5));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb6  (.I0(br_bank_root1), .O(br_bank_b6));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbb7  (.I0(br_bank_root1), .O(br_bank_b7));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbs0  (.I0(br_bank_root1), .O(br_bank_s0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbs1  (.I0(br_bank_root1), .O(br_bank_s1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbm0  (.I0(br_bank_root1), .O(br_bank_m0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bbm1  (.I0(br_bank_root1), .O(br_bank_m1));

	wire [2:0] br_seg_root0, br_seg_root1, br_seg_root2, br_seg_root3;
	wire [2:0] br_seg_idx_l0_lo, br_seg_idx_l0_hi;
	wire [2:0] br_seg_idx_l1_lo, br_seg_idx_l1_hi;
	wire [2:0] br_seg_idx_l2_lo, br_seg_idx_l2_hi;
	wire [2:0] br_seg_idx_l3_lo, br_seg_idx_l3_hi;
	wire [2:0] br_seg_idx_b0_lo, br_seg_idx_b0_hi;
	wire [2:0] br_seg_idx_b1_lo, br_seg_idx_b1_hi;
	wire [2:0] br_seg_idx_b2_lo, br_seg_idx_b2_hi;
	wire [2:0] br_seg_idx_b3_lo, br_seg_idx_b3_hi;
	wire [2:0] br_seg_idx_b4_lo, br_seg_idx_b4_hi;
	wire [2:0] br_seg_idx_b5_lo, br_seg_idx_b5_hi;
	wire [2:0] br_seg_idx_b6_lo, br_seg_idx_b6_hi;
	wire [2:0] br_seg_idx_b7_lo, br_seg_idx_b7_hi;
	wire [2:0] br_seg_idx_s0, br_seg_idx_s1, br_seg_idx_m0, br_seg_idx_m1;
	genvar brb;
	generate
		for (brb = 0; brb < 3; brb = brb + 1) begin : g_br_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsr0 (.I0(br_seg_idx[brb]), .O(br_seg_root0[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsr1 (.I0(br_seg_idx[brb]), .O(br_seg_root1[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsr2 (.I0(br_seg_idx[brb]), .O(br_seg_root2[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsr3 (.I0(br_seg_idx[brb]), .O(br_seg_root3[brb]));

			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl0l (.I0(br_seg_root0[brb]), .O(br_seg_idx_l0_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl0h (.I0(br_seg_root0[brb]), .O(br_seg_idx_l0_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl1l (.I0(br_seg_root0[brb]), .O(br_seg_idx_l1_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl1h (.I0(br_seg_root0[brb]), .O(br_seg_idx_l1_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl2l (.I0(br_seg_root0[brb]), .O(br_seg_idx_l2_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl2h (.I0(br_seg_root0[brb]), .O(br_seg_idx_l2_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl3l (.I0(br_seg_root0[brb]), .O(br_seg_idx_l3_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl3h (.I0(br_seg_root0[brb]), .O(br_seg_idx_l3_hi[brb]));

			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb0l (.I0(br_seg_root1[brb]), .O(br_seg_idx_b0_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb0h (.I0(br_seg_root1[brb]), .O(br_seg_idx_b0_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb1l (.I0(br_seg_root1[brb]), .O(br_seg_idx_b1_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb1h (.I0(br_seg_root1[brb]), .O(br_seg_idx_b1_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb2l (.I0(br_seg_root1[brb]), .O(br_seg_idx_b2_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb2h (.I0(br_seg_root1[brb]), .O(br_seg_idx_b2_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb3l (.I0(br_seg_root1[brb]), .O(br_seg_idx_b3_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb3h (.I0(br_seg_root1[brb]), .O(br_seg_idx_b3_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb4l (.I0(br_seg_root2[brb]), .O(br_seg_idx_b4_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb4h (.I0(br_seg_root2[brb]), .O(br_seg_idx_b4_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb5l (.I0(br_seg_root2[brb]), .O(br_seg_idx_b5_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb5h (.I0(br_seg_root2[brb]), .O(br_seg_idx_b5_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb6l (.I0(br_seg_root2[brb]), .O(br_seg_idx_b6_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb6h (.I0(br_seg_root2[brb]), .O(br_seg_idx_b6_hi[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb7l (.I0(br_seg_root2[brb]), .O(br_seg_idx_b7_lo[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsb7h (.I0(br_seg_root2[brb]), .O(br_seg_idx_b7_hi[brb]));

			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bss0  (.I0(br_seg_root3[brb]), .O(br_seg_idx_s0[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bss1  (.I0(br_seg_root3[brb]), .O(br_seg_idx_s1[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsm0  (.I0(br_seg_root3[brb]), .O(br_seg_idx_m0[brb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsm1  (.I0(br_seg_root3[brb]), .O(br_seg_idx_m1[brb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire br_bank_root0, br_bank_root1;
	assign br_bank_root0 = br_bank;
	assign br_bank_root1 = br_bank;

	wire [3:0] br_bank_ram;
	wire br_bank_l0_lo, br_bank_l0_hi, br_bank_l1_lo, br_bank_l1_hi, br_bank_l2, br_bank_l3;
	wire br_bank_b0, br_bank_b1, br_bank_b2, br_bank_b3;
	wire br_bank_b4, br_bank_b5, br_bank_b6, br_bank_b7;
	wire br_bank_s0, br_bank_s1, br_bank_m0, br_bank_m1;
	assign br_bank_ram[0] = br_bank_root1;
	assign br_bank_ram[1] = br_bank_root1;
	assign br_bank_ram[2] = br_bank_root1;
	assign br_bank_ram[3] = br_bank_root1;
	assign br_bank_l0_lo = br_bank_root0;
	assign br_bank_l0_hi = br_bank_root0;
	assign br_bank_l1_lo = br_bank_root0;
	assign br_bank_l1_hi = br_bank_root0;
	assign br_bank_l2 = br_bank_root0;
	assign br_bank_l3 = br_bank_root0;
	assign br_bank_b0 = br_bank_root0;
	assign br_bank_b1 = br_bank_root0;
	assign br_bank_b2 = br_bank_root0;
	assign br_bank_b3 = br_bank_root0;
	assign br_bank_b4 = br_bank_root1;
	assign br_bank_b5 = br_bank_root1;
	assign br_bank_b6 = br_bank_root1;
	assign br_bank_b7 = br_bank_root1;
	assign br_bank_s0 = br_bank_root1;
	assign br_bank_s1 = br_bank_root1;
	assign br_bank_m0 = br_bank_root1;
	assign br_bank_m1 = br_bank_root1;

	wire [2:0] br_seg_root0, br_seg_root1, br_seg_root2, br_seg_root3;
	wire [2:0] br_seg_idx_l0_lo, br_seg_idx_l0_hi;
	wire [2:0] br_seg_idx_l1_lo, br_seg_idx_l1_hi;
	wire [2:0] br_seg_idx_l2_lo, br_seg_idx_l2_hi;
	wire [2:0] br_seg_idx_l3_lo, br_seg_idx_l3_hi;
	wire [2:0] br_seg_idx_b0_lo, br_seg_idx_b0_hi;
	wire [2:0] br_seg_idx_b1_lo, br_seg_idx_b1_hi;
	wire [2:0] br_seg_idx_b2_lo, br_seg_idx_b2_hi;
	wire [2:0] br_seg_idx_b3_lo, br_seg_idx_b3_hi;
	wire [2:0] br_seg_idx_b4_lo, br_seg_idx_b4_hi;
	wire [2:0] br_seg_idx_b5_lo, br_seg_idx_b5_hi;
	wire [2:0] br_seg_idx_b6_lo, br_seg_idx_b6_hi;
	wire [2:0] br_seg_idx_b7_lo, br_seg_idx_b7_hi;
	wire [2:0] br_seg_idx_s0, br_seg_idx_s1, br_seg_idx_m0, br_seg_idx_m1;
	genvar brb;
	generate
		for (brb = 0; brb < 3; brb = brb + 1) begin : g_br_buf
			assign br_seg_root0[brb] = br_seg_idx[brb];
			assign br_seg_root1[brb] = br_seg_idx[brb];
			assign br_seg_root2[brb] = br_seg_idx[brb];
			assign br_seg_root3[brb] = br_seg_idx[brb];

			assign br_seg_idx_l0_lo[brb] = br_seg_root0[brb];
			assign br_seg_idx_l0_hi[brb] = br_seg_root0[brb];
			assign br_seg_idx_l1_lo[brb] = br_seg_root0[brb];
			assign br_seg_idx_l1_hi[brb] = br_seg_root0[brb];
			assign br_seg_idx_l2_lo[brb] = br_seg_root0[brb];
			assign br_seg_idx_l2_hi[brb] = br_seg_root0[brb];
			assign br_seg_idx_l3_lo[brb] = br_seg_root0[brb];
			assign br_seg_idx_l3_hi[brb] = br_seg_root0[brb];

			assign br_seg_idx_b0_lo[brb] = br_seg_root1[brb];
			assign br_seg_idx_b0_hi[brb] = br_seg_root1[brb];
			assign br_seg_idx_b1_lo[brb] = br_seg_root1[brb];
			assign br_seg_idx_b1_hi[brb] = br_seg_root1[brb];
			assign br_seg_idx_b2_lo[brb] = br_seg_root1[brb];
			assign br_seg_idx_b2_hi[brb] = br_seg_root1[brb];
			assign br_seg_idx_b3_lo[brb] = br_seg_root1[brb];
			assign br_seg_idx_b3_hi[brb] = br_seg_root1[brb];
			assign br_seg_idx_b4_lo[brb] = br_seg_root2[brb];
			assign br_seg_idx_b4_hi[brb] = br_seg_root2[brb];
			assign br_seg_idx_b5_lo[brb] = br_seg_root2[brb];
			assign br_seg_idx_b5_hi[brb] = br_seg_root2[brb];
			assign br_seg_idx_b6_lo[brb] = br_seg_root2[brb];
			assign br_seg_idx_b6_hi[brb] = br_seg_root2[brb];
			assign br_seg_idx_b7_lo[brb] = br_seg_root2[brb];
			assign br_seg_idx_b7_hi[brb] = br_seg_root2[brb];

			assign br_seg_idx_s0[brb] = br_seg_root3[brb];
			assign br_seg_idx_s1[brb] = br_seg_root3[brb];
			assign br_seg_idx_m0[brb] = br_seg_root3[brb];
			assign br_seg_idx_m1[brb] = br_seg_root3[brb];
		end
	endgenerate
`else
	wire [3:0] br_bank_ram = {4{br_bank}};
	wire br_bank_l0_lo = br_bank, br_bank_l0_hi = br_bank, br_bank_l1_lo = br_bank, br_bank_l1_hi = br_bank;
	wire br_bank_l2 = br_bank, br_bank_l3 = br_bank;
	wire br_bank_b0 = br_bank, br_bank_b1 = br_bank, br_bank_b2 = br_bank, br_bank_b3 = br_bank;
	wire br_bank_b4 = br_bank, br_bank_b5 = br_bank, br_bank_b6 = br_bank, br_bank_b7 = br_bank;
	wire br_bank_s0 = br_bank, br_bank_s1 = br_bank, br_bank_m0 = br_bank, br_bank_m1 = br_bank;
	wire [2:0] br_seg_idx_l0_lo = br_seg_idx, br_seg_idx_l0_hi = br_seg_idx;
	wire [2:0] br_seg_idx_l1_lo = br_seg_idx, br_seg_idx_l1_hi = br_seg_idx;
	wire [2:0] br_seg_idx_l2_lo = br_seg_idx, br_seg_idx_l2_hi = br_seg_idx;
	wire [2:0] br_seg_idx_l3_lo = br_seg_idx, br_seg_idx_l3_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b0_lo = br_seg_idx, br_seg_idx_b0_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b1_lo = br_seg_idx, br_seg_idx_b1_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b2_lo = br_seg_idx, br_seg_idx_b2_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b3_lo = br_seg_idx, br_seg_idx_b3_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b4_lo = br_seg_idx, br_seg_idx_b4_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b5_lo = br_seg_idx, br_seg_idx_b5_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b6_lo = br_seg_idx, br_seg_idx_b6_hi = br_seg_idx;
	wire [2:0] br_seg_idx_b7_lo = br_seg_idx, br_seg_idx_b7_hi = br_seg_idx;
	wire [2:0] br_seg_idx_s0 = br_seg_idx, br_seg_idx_s1 = br_seg_idx, br_seg_idx_m0 = br_seg_idx, br_seg_idx_m1 = br_seg_idx;
`endif

	wire [3:0] r_slot0 = br_bank_s0 ? meta_slot0_1[br_seg_idx_s0] : meta_slot0_0[br_seg_idx_s0];
	wire [3:0] r_slot1 = br_bank_s1 ? meta_slot1_1[br_seg_idx_s1] : meta_slot1_0[br_seg_idx_s1];
	wire r_n0slot = br_bank_m0 ? meta_n0slot_1[br_seg_idx_m0] : meta_n0slot_0[br_seg_idx_m0];
	wire [US-1:0] r_off = br_bank_m1 ? meta_off_1[br_seg_idx_m1] : meta_off_0[br_seg_idx_m1];

	wire [3:0] r_lo_slot = r_n0slot ? r_slot1 : r_slot0;
	wire [3:0] r_hi_slot = r_n0slot ? r_slot0 : r_slot1;

`ifdef SYNTH_KINTEX7
	/* Partitioned distributed RAM slices with direct FF-driven read addresses (s1_raddr0/1) */
	wire [LINE_BITS-1:0] pp_sdata0;
	wire [LINE_BITS-1:0] pp_sdata1;

	wire [1:0] r_n0slot_b;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rn0b0 (.I0(r_n0slot), .O(r_n0slot_b[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rn0b1 (.I0(r_n0slot), .O(r_n0slot_b[1]));

	wire [3:0] r_lo_slot_g [0:3];
	wire [3:0] r_hi_slot_g [0:3];

	genvar tk, ab;
	generate
		for (tk = 0; tk < 4; tk = tk + 1) begin : g_ram_top_buf
			for (ab = 0; ab < 4; ab = ab + 1) begin : g_slot_dup
				LUT3 #(.INIT(8'hCA)) u_rlo (.I0(r_slot0[ab]), .I1(r_slot1[ab]), .I2(r_n0slot_b[tk >> 1]), .O(r_lo_slot_g[tk][ab]));
				LUT3 #(.INIT(8'hAC)) u_rhi (.I0(r_slot0[ab]), .I1(r_slot1[ab]), .I2(r_n0slot_b[tk >> 1]), .O(r_hi_slot_g[tk][ab]));
			end
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [4:0] s1_raddr0 [0:NUM_RADDR_REPS-1];
	(* equivalent_register_removal = "no" *) reg [4:0] s1_raddr1 [0:NUM_RADDR_REPS-1];
	genvar rrep;
	generate
		for (rrep = 0; rrep < NUM_RADDR_REPS; rrep = rrep + 1) begin : g_s1_raddr
			localparam TK_IDX = rrep / (NUM_RADDR_REPS / 4);
			always @(posedge clk) begin
				if (br_rdy_leaf[rrep / 2]) begin
					s1_raddr0[rrep] <= {br_bank_ram[TK_IDX], r_lo_slot_g[TK_IDX]};
					s1_raddr1[rrep] <= {br_bank_ram[TK_IDX], r_hi_slot_g[TK_IDX]};
				end
			end
		end
	endgenerate

	localparam integer NUM_WE_SUBGRPS = NUM_RAM_SLICES;
	localparam integer NUM_RAM_GRPS2  = NUM_RAM_SLICES;
	wire [4:0] grp_waddr [0:NUM_RAM_GRPS2-1];
	wire [4:0] rtag_mid  [0:7];
	wire       rspv_mid  [0:7];
	wire       rspw_mid  [0:7];
	wire       rtag7_mid [0:7];
	wire       mbst_mid  [0:7];

	genvar gm, gk;
	generate
		for (gm = 0; gm < 8; gm = gm + 1) begin : g_rtag_mid
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_rtm_b
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rtm (.I0(rsp_tag[ab]), .O(rtag_mid[gm][ab]));
			end
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rvm (.I0(rsp_valid_bst), .O(rspv_mid[gm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rwm (.I0(rsp_write),  .O(rspw_mid[gm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rt7 (.I0(rsp_tag[7]), .O(rtag7_mid[gm]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbm (.I0(m_burst),    .O(mbst_mid[gm]));
		end
		for (gk = 0; gk < NUM_RAM_GRPS2; gk = gk + 1) begin : g_ram_grp_buf
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_addr_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ga (.I0(rtag_mid[gk / (NUM_RAM_GRPS2 / 8)][ab]), .O(grp_waddr[gk][ab]));
			end
		end
	endgenerate

	wire       subgrp_we [0:NUM_WE_SUBGRPS-1];
	wire [4:0] sl_waddr  [0:NUM_RAM_SLICES-1];
	wire       sl_we     [0:NUM_RAM_SLICES-1];

	genvar sg, sl;
	generate
		for (sg = 0; sg < NUM_WE_SUBGRPS; sg = sg + 1) begin : g_we_subgrp
			LUT4 #(.INIT(16'h0200)) u_we (
				.I0(rspv_mid[sg / (NUM_WE_SUBGRPS / 8)]), .I1(rspw_mid[sg / (NUM_WE_SUBGRPS / 8)]),
				.I2(rtag7_mid[sg / (NUM_WE_SUBGRPS / 8)]), .I3(mbst_mid[sg / (NUM_WE_SUBGRPS / 8)]),
				.O(subgrp_we[sg])
			);
		end
		for (sl = 0; sl < NUM_RAM_SLICES; sl = sl + 1) begin : g_pp_ram
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_sl_buf
				assign sl_waddr[sl][ab] = grp_waddr[sl][ab];
			end
			assign sl_we[sl] = subgrp_we[sl];

			(* ram_style = "distributed" *) reg [3:0] ram [0:31];
			always @(posedge clk) begin
				if (sl_we[sl])
					ram[sl_waddr[sl]] <= rsp_rdata[sl*4 +: 4];
			end
			assign pp_sdata0[sl*4 +: 4] = ram[s1_raddr0[sl / 4]];
			assign pp_sdata1[sl*4 +: 4] = ram[s1_raddr1[sl / 4]];
		end
	endgenerate

	/* Stage 1 -> Stage 2 barrel shifter Stage 0 (shift by 0..3 bytes) */
	wire [1:0] r_off_s0_buf [0:3];
	genvar rsb;
	generate
		for (rsb = 0; rsb < 4; rsb = rsb + 1) begin : g_roff_s0_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ro0 (.I0(r_off[0]), .O(r_off_s0_buf[rsb][0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ro1 (.I0(r_off[1]), .O(r_off_s0_buf[rsb][1]));
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [1:0] s1_s0_sel [0:PP_S0_LEAF_CNT-1];
	genvar s0l;
	generate
		for (s0l = 0; s0l < PP_S0_LEAF_CNT; s0l = s0l + 1) begin : g_s1_s0_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[16 + (s0l / 8)])
					s1_s0_sel[s0l] <= r_off_s0_buf[s0l / 16];
			end
		end
	endgenerate

	wire [2*LINE_BITS-1:0] pp_both = {pp_sdata1, pp_sdata0};
	wire [7:0] pp_in_b  [0:2*LINE_BYTES-1];
	wire [7:0] pp_st0_b [0:PP_S0_OUT_BYTES-1];
	reg [7:0] s2_st0_b [0:PP_S0_OUT_BYTES-1];

	genvar pui, pi0;
	generate
		for (pui = 0; pui < 2 * LINE_BYTES; pui = pui + 1) begin : g_pp_unpack
			assign pp_in_b[pui] = pp_both[pui*8 +: 8];
		end
		for (pi0 = 0; pi0 < PP_S0_OUT_BYTES; pi0 = pi0 + 1) begin : g_pp_st0
			wire [1:0] sel0 = s1_s0_sel[pi0 / 2];
			assign pp_st0_b[pi0] = (sel0 == 2'd3) ? pp_in_b[pi0 + 3] :
			                       (sel0 == 2'd2) ? pp_in_b[pi0 + 2] :
			                       (sel0 == 2'd1) ? pp_in_b[pi0 + 1] : pp_in_b[pi0];
			always @(posedge clk) begin
				if (br_rdy_leaf[24 + (pi0 / 2)])
					s2_st0_b[pi0] <= pp_st0_b[pi0];
			end
		end
	endgenerate

	/* Stage 2 -> Stage 3 barrel shifter Stages 1 & 2 (shift by 4..12 and 16..48 bytes) */
	wire [1:0] s1_off_s1_buf [0:3];
	wire [1:0] s1_off_s2_buf [0:1];
	generate
		for (rsb = 0; rsb < 4; rsb = rsb + 1) begin : g_s1_off_s1_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_so10 (.I0(s1_off_hi[2]), .O(s1_off_s1_buf[rsb][0]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_so11 (.I0(s1_off_hi[3]), .O(s1_off_s1_buf[rsb][1]));
		end
		for (rsb = 0; rsb < 2; rsb = rsb + 1) begin : g_s1_off_s2_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_so20 (.I0(s1_off_hi[4]), .O(s1_off_s2_buf[rsb][0]));
			if (LINE_BYTES >= 64) begin : g_s2o1_64
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_so21 (.I0(s1_off_hi[5]), .O(s1_off_s2_buf[rsb][1]));
			end else begin : g_s2o1_32
				assign s1_off_s2_buf[rsb][1] = 1'b0;
			end
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [1:0] s2_s1_sel [0:PP_S1_LEAF_CNT-1];
	(* equivalent_register_removal = "no" *) reg [1:0] s2_s2_sel [0:PP_S2_LEAF_CNT-1];
	genvar s1l, s2l;
	generate
		for (s1l = 0; s1l < PP_S1_LEAF_CNT; s1l = s1l + 1) begin : g_s2_s1_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[86 + (s1l / 8)])
					s2_s1_sel[s1l] <= s1_off_s1_buf[s1l / 16];
			end
		end
		for (s2l = 0; s2l < PP_S2_LEAF_CNT; s2l = s2l + 1) begin : g_s2_s2_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[93 + (s2l / 8)])
					s2_s2_sel[s2l] <= s1_off_s2_buf[s2l / 16];
			end
		end
	endgenerate

	wire [7:0] pp_st1_b [0:PP_S1_OUT_BYTES-1];
	wire [7:0] pp_st2_b [0:LINE_BYTES-1];
	wire [LINE_BITS-1:0] pp_copy_line;
	genvar pi1, pi2;
	generate
		for (pi1 = 0; pi1 < PP_S1_OUT_BYTES; pi1 = pi1 + 1) begin : g_pp_st1
			wire [1:0] sel1 = s2_s1_sel[pi1 / 2];
			assign pp_st1_b[pi1] = (sel1 == 2'd3) ? s2_st0_b[pi1 + 12] :
			                       (sel1 == 2'd2) ? s2_st0_b[pi1 + 8]  :
			                       (sel1 == 2'd1) ? s2_st0_b[pi1 + 4]  : s2_st0_b[pi1];
		end
		for (pi2 = 0; pi2 < LINE_BYTES; pi2 = pi2 + 1) begin : g_pp_st2
			wire [1:0] sel2 = s2_s2_sel[pi2 / 2];
			if (LINE_BYTES >= 64) begin : g_pp_s2_64
				assign pp_st2_b[pi2] = (sel2 == 2'd3) ? pp_st1_b[pi2 + 48] :
				                       (sel2 == 2'd2) ? pp_st1_b[pi2 + 32] :
				                       (sel2 == 2'd1) ? pp_st1_b[pi2 + 16] : pp_st1_b[pi2];
			end else begin : g_pp_s2_32
				assign pp_st2_b[pi2] = sel2[0] ? pp_st1_b[pi2 + 16] : pp_st1_b[pi2];
			end
			assign pp_copy_line[pi2*8 +: 8] = pp_st2_b[pi2];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Partitioned distributed RAM slices with direct FF-driven read addresses (s1_raddr0/1) */
	wire [LINE_BITS-1:0] pp_sdata0;
	wire [LINE_BITS-1:0] pp_sdata1;

	wire [1:0] r_n0slot_b;
	assign r_n0slot_b[0] = r_n0slot;
	assign r_n0slot_b[1] = r_n0slot;

	wire [3:0] r_lo_slot_g [0:3];
	wire [3:0] r_hi_slot_g [0:3];

	genvar tk, ab;
	generate
		for (tk = 0; tk < 4; tk = tk + 1) begin : g_ram_top_buf
			for (ab = 0; ab < 4; ab = ab + 1) begin : g_slot_dup
				LUT3 #(.INIT(8'hCA)) u_rlo (.I0(r_slot0[ab]), .I1(r_slot1[ab]), .I2(r_n0slot_b[tk >> 1]), .O(r_lo_slot_g[tk][ab]));
				LUT3 #(.INIT(8'hAC)) u_rhi (.I0(r_slot0[ab]), .I1(r_slot1[ab]), .I2(r_n0slot_b[tk >> 1]), .O(r_hi_slot_g[tk][ab]));
			end
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [4:0] s1_raddr0 [0:NUM_RADDR_REPS-1];
	(* equivalent_register_removal = "no" *) reg [4:0] s1_raddr1 [0:NUM_RADDR_REPS-1];
	genvar rrep;
	generate
		for (rrep = 0; rrep < NUM_RADDR_REPS; rrep = rrep + 1) begin : g_s1_raddr
			localparam TK_IDX = rrep / (NUM_RADDR_REPS / 4);
			always @(posedge clk) begin
				if (br_rdy_leaf[rrep / 2]) begin
					s1_raddr0[rrep] <= {br_bank_ram[TK_IDX], r_lo_slot_g[TK_IDX]};
					s1_raddr1[rrep] <= {br_bank_ram[TK_IDX], r_hi_slot_g[TK_IDX]};
				end
			end
		end
	endgenerate

	localparam integer NUM_WE_SUBGRPS = NUM_RAM_SLICES;
	localparam integer NUM_RAM_GRPS2  = NUM_RAM_SLICES;
	wire [4:0] grp_waddr [0:NUM_RAM_GRPS2-1];
	wire [4:0] rtag_mid  [0:7];
	wire       rspv_mid  [0:7];
	wire       rspw_mid  [0:7];
	wire       rtag7_mid [0:7];
	wire       mbst_mid  [0:7];

	genvar gm, gk;
	generate
		for (gm = 0; gm < 8; gm = gm + 1) begin : g_rtag_mid
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_rtm_b
				assign rtag_mid[gm][ab] = rsp_tag[ab];
			end
			assign rspv_mid[gm] = rsp_valid_bst;
			assign rspw_mid[gm] = rsp_write;
			assign rtag7_mid[gm] = rsp_tag[7];
			assign mbst_mid[gm] = m_burst;
		end
		for (gk = 0; gk < NUM_RAM_GRPS2; gk = gk + 1) begin : g_ram_grp_buf
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_addr_buf
				assign grp_waddr[gk][ab] = rtag_mid[gk / (NUM_RAM_GRPS2 / 8)][ab];
			end
		end
	endgenerate

	wire       subgrp_we [0:NUM_WE_SUBGRPS-1];
	wire [4:0] sl_waddr  [0:NUM_RAM_SLICES-1];
	wire       sl_we     [0:NUM_RAM_SLICES-1];

	genvar sg, sl;
	generate
		for (sg = 0; sg < NUM_WE_SUBGRPS; sg = sg + 1) begin : g_we_subgrp
			LUT4 #(.INIT(16'h0200)) u_we (
				.I0(rspv_mid[sg / (NUM_WE_SUBGRPS / 8)]), .I1(rspw_mid[sg / (NUM_WE_SUBGRPS / 8)]),
				.I2(rtag7_mid[sg / (NUM_WE_SUBGRPS / 8)]), .I3(mbst_mid[sg / (NUM_WE_SUBGRPS / 8)]),
				.O(subgrp_we[sg])
			);
		end
		for (sl = 0; sl < NUM_RAM_SLICES; sl = sl + 1) begin : g_pp_ram
			for (ab = 0; ab < 5; ab = ab + 1) begin : g_sl_buf
				assign sl_waddr[sl][ab] = grp_waddr[sl][ab];
			end
			assign sl_we[sl] = subgrp_we[sl];

			(* ram_style = "distributed" *) reg [3:0] ram [0:31];
			always @(posedge clk) begin
				if (sl_we[sl])
					ram[sl_waddr[sl]] <= rsp_rdata[sl*4 +: 4];
			end
			assign pp_sdata0[sl*4 +: 4] = ram[s1_raddr0[sl / 4]];
			assign pp_sdata1[sl*4 +: 4] = ram[s1_raddr1[sl / 4]];
		end
	endgenerate

	/* Stage 1 -> Stage 2 barrel shifter Stage 0 (shift by 0..3 bytes) */
	wire [1:0] r_off_s0_buf [0:3];
	genvar rsb;
	generate
		for (rsb = 0; rsb < 4; rsb = rsb + 1) begin : g_roff_s0_buf
			assign r_off_s0_buf[rsb][0] = r_off[0];
			assign r_off_s0_buf[rsb][1] = r_off[1];
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [1:0] s1_s0_sel [0:PP_S0_LEAF_CNT-1];
	genvar s0l;
	generate
		for (s0l = 0; s0l < PP_S0_LEAF_CNT; s0l = s0l + 1) begin : g_s1_s0_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[16 + (s0l / 8)])
					s1_s0_sel[s0l] <= r_off_s0_buf[s0l / 16];
			end
		end
	endgenerate

	wire [2*LINE_BITS-1:0] pp_both = {pp_sdata1, pp_sdata0};
	wire [7:0] pp_in_b  [0:2*LINE_BYTES-1];
	wire [7:0] pp_st0_b [0:PP_S0_OUT_BYTES-1];
	reg [7:0] s2_st0_b [0:PP_S0_OUT_BYTES-1];

	genvar pui, pi0;
	generate
		for (pui = 0; pui < 2 * LINE_BYTES; pui = pui + 1) begin : g_pp_unpack
			assign pp_in_b[pui] = pp_both[pui*8 +: 8];
		end
		for (pi0 = 0; pi0 < PP_S0_OUT_BYTES; pi0 = pi0 + 1) begin : g_pp_st0
			wire [1:0] sel0 = s1_s0_sel[pi0 / 2];
			assign pp_st0_b[pi0] = (sel0 == 2'd3) ? pp_in_b[pi0 + 3] :
			                       (sel0 == 2'd2) ? pp_in_b[pi0 + 2] :
			                       (sel0 == 2'd1) ? pp_in_b[pi0 + 1] : pp_in_b[pi0];
			always @(posedge clk) begin
				if (br_rdy_leaf[24 + (pi0 / 2)])
					s2_st0_b[pi0] <= pp_st0_b[pi0];
			end
		end
	endgenerate

	/* Stage 2 -> Stage 3 barrel shifter Stages 1 & 2 (shift by 4..12 and 16..48 bytes) */
	wire [1:0] s1_off_s1_buf [0:3];
	wire [1:0] s1_off_s2_buf [0:1];
	generate
		for (rsb = 0; rsb < 4; rsb = rsb + 1) begin : g_s1_off_s1_buf
			assign s1_off_s1_buf[rsb][0] = s1_off_hi[2];
			assign s1_off_s1_buf[rsb][1] = s1_off_hi[3];
		end
		for (rsb = 0; rsb < 2; rsb = rsb + 1) begin : g_s1_off_s2_buf
			assign s1_off_s2_buf[rsb][0] = s1_off_hi[4];
			if (LINE_BYTES >= 64) begin : g_s2o1_64
				assign s1_off_s2_buf[rsb][1] = s1_off_hi[5];
			end else begin : g_s2o1_32
				assign s1_off_s2_buf[rsb][1] = 1'b0;
			end
		end
	endgenerate

	(* equivalent_register_removal = "no" *) reg [1:0] s2_s1_sel [0:PP_S1_LEAF_CNT-1];
	(* equivalent_register_removal = "no" *) reg [1:0] s2_s2_sel [0:PP_S2_LEAF_CNT-1];
	genvar s1l, s2l;
	generate
		for (s1l = 0; s1l < PP_S1_LEAF_CNT; s1l = s1l + 1) begin : g_s2_s1_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[86 + (s1l / 8)])
					s2_s1_sel[s1l] <= s1_off_s1_buf[s1l / 16];
			end
		end
		for (s2l = 0; s2l < PP_S2_LEAF_CNT; s2l = s2l + 1) begin : g_s2_s2_sel
			always @(posedge clk) begin
				if (br_rdy_leaf[93 + (s2l / 8)])
					s2_s2_sel[s2l] <= s1_off_s2_buf[s2l / 16];
			end
		end
	endgenerate

	wire [7:0] pp_st1_b [0:PP_S1_OUT_BYTES-1];
	wire [7:0] pp_st2_b [0:LINE_BYTES-1];
	wire [LINE_BITS-1:0] pp_copy_line;
	genvar pi1, pi2;
	generate
		for (pi1 = 0; pi1 < PP_S1_OUT_BYTES; pi1 = pi1 + 1) begin : g_pp_st1
			wire [1:0] sel1 = s2_s1_sel[pi1 / 2];
			assign pp_st1_b[pi1] = (sel1 == 2'd3) ? s2_st0_b[pi1 + 12] :
			                       (sel1 == 2'd2) ? s2_st0_b[pi1 + 8]  :
			                       (sel1 == 2'd1) ? s2_st0_b[pi1 + 4]  : s2_st0_b[pi1];
		end
		for (pi2 = 0; pi2 < LINE_BYTES; pi2 = pi2 + 1) begin : g_pp_st2
			wire [1:0] sel2 = s2_s2_sel[pi2 / 2];
			if (LINE_BYTES >= 64) begin : g_pp_s2_64
				assign pp_st2_b[pi2] = (sel2 == 2'd3) ? pp_st1_b[pi2 + 48] :
				                       (sel2 == 2'd2) ? pp_st1_b[pi2 + 32] :
				                       (sel2 == 2'd1) ? pp_st1_b[pi2 + 16] : pp_st1_b[pi2];
			end else begin : g_pp_s2_32
				assign pp_st2_b[pi2] = sel2[0] ? pp_st1_b[pi2 + 16] : pp_st1_b[pi2];
			end
			assign pp_copy_line[pi2*8 +: 8] = pp_st2_b[pi2];
		end
	endgenerate
`else
	localparam integer PP_SL_BITS = LINE_BITS / 8;
	localparam integer BH_SLICES  = LINE_BITS / 16;
	(* keep = "true" *) reg [4:0] s1_raddr0_sim;
	(* keep = "true" *) reg [4:0] s1_raddr1_sim;
	reg [US-1:0] s1_off_sim;
	(* keep = "true" *) reg [8*US-1:0] s2_off_rep;
	(* keep = "true" *) reg [63:0] s2_s0_oh_rep, s2_s1_oh_rep;
	wire [7:0] s1_s0_oh_next = {
		 s1_off_sim[2] &  s1_off_sim[1] &  s1_off_sim[0],
		 s1_off_sim[2] &  s1_off_sim[1] & ~s1_off_sim[0],
		 s1_off_sim[2] & ~s1_off_sim[1] &  s1_off_sim[0],
		 s1_off_sim[2] & ~s1_off_sim[1] & ~s1_off_sim[0],
		~s1_off_sim[2] &  s1_off_sim[1] &  s1_off_sim[0],
		~s1_off_sim[2] &  s1_off_sim[1] & ~s1_off_sim[0],
		~s1_off_sim[2] & ~s1_off_sim[1] &  s1_off_sim[0],
		~s1_off_sim[2] & ~s1_off_sim[1] & ~s1_off_sim[0]
	};
	wire [2:0] s1_sel1_next;
	generate
		if (LINE_BYTES >= 64) begin : g_psel1_64
			assign s1_sel1_next = s1_off_sim[5:3];
		end else begin : g_psel1_32
			assign s1_sel1_next = {1'b0, s1_off_sim[4:3]};
		end
	endgenerate
	wire [7:0] s1_s1_oh_next = {
		 s1_sel1_next[2] &  s1_sel1_next[1] &  s1_sel1_next[0],
		 s1_sel1_next[2] &  s1_sel1_next[1] & ~s1_sel1_next[0],
		 s1_sel1_next[2] & ~s1_sel1_next[1] &  s1_sel1_next[0],
		 s1_sel1_next[2] & ~s1_sel1_next[1] & ~s1_sel1_next[0],
		~s1_sel1_next[2] &  s1_sel1_next[1] &  s1_sel1_next[0],
		~s1_sel1_next[2] &  s1_sel1_next[1] & ~s1_sel1_next[0],
		~s1_sel1_next[2] & ~s1_sel1_next[1] &  s1_sel1_next[0],
		~s1_sel1_next[2] & ~s1_sel1_next[1] & ~s1_sel1_next[0]
	};
	wire [LINE_BITS-1:0] pp_rdata0 = pp_ram[s1_raddr0_sim];
	wire [LINE_BITS-1:0] pp_rdata1 = pp_ram[s1_raddr1_sim];
	reg [LINE_BITS-1:0] pp_rdata0_q, pp_rdata1_q;
	reg br_hold_v;
	wire [BH_SLICES-1:0] br_hold_v_rep;
	reg [LINE_BITS-1:0] s2_hold_line;
	wire [LINE_BITS-1:0] pp_raw_copy_line;
	always @(posedge clk) begin
		if (br_pipe_rdy) begin
			s1_raddr0_sim <= {br_bank, r_lo_slot};
			s1_raddr1_sim <= {br_bank, r_hi_slot};
			s1_off_sim    <= r_off;
		end
	end
	/* Unconditional capture on pp_rdata*_q binds Yosys memory_dff cleanly (Check 4 = 0), while s2_hold_line captures aligned line on stall. */
	always @(posedge clk) begin
		pp_rdata0_q <= pp_rdata0;
		pp_rdata1_q <= pp_rdata1;
		if (!br_pipe_rdy && !br_hold_v) begin
			s2_hold_line <= pp_raw_copy_line;
		end
	end
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			br_hold_v <= 1'b0;
		end else if (!is_burst_ctl0 || fail_burst || bst_rd_err || br_pipe_rdy) begin
			br_hold_v <= 1'b0;
		end else if (!br_hold_v) begin
			br_hold_v <= 1'b1;
		end
	end
	genvar g_pps;
	generate
		for (g_pps = 0; g_pps < 8; g_pps = g_pps + 1) begin : g_pp_sim
			always @(posedge clk) begin
				if (br_rdy_sl[g_pps]) begin
					s2_off_rep[g_pps*US +: US]  <= s1_off_sim;
					s2_s0_oh_rep[g_pps*8 +: 8] <= s1_s0_oh_next;
					s2_s1_oh_rep[g_pps*8 +: 8] <= s1_s1_oh_next;
				end
			end
		end
	endgenerate
	wire [2*LINE_BITS-1:0] pp_raw_both = {pp_rdata1_q, pp_rdata0_q};
	genvar g_bh;
	generate
		for (g_bh = 0; g_bh < BH_SLICES; g_bh = g_bh + 1) begin : g_pp_hold_mux
			(* keep = "true" *) reg br_hold_v_bit;
			always @(posedge clk or posedge rst_b2d_high) begin
				if (rst_b2d_high)
					br_hold_v_bit <= 1'b0;
				else if (!is_burst_ctl0 || fail_burst || bst_rd_err || br_pipe_rdy)
					br_hold_v_bit <= 1'b0;
				else if (!br_hold_v_bit)
					br_hold_v_bit <= 1'b1;
			end
			assign br_hold_v_rep[g_bh] = br_hold_v_bit;
		end
	endgenerate

	localparam integer PP_S0_OUT_A     = (LINE_BYTES >= 64) ? (LINE_BYTES + 56) : (LINE_BYTES + 24);
	localparam integer PP_S0A_LEAF_CNT = (PP_S0_OUT_A + 1) / 2;
	localparam integer PP_S1A_LEAF_CNT = (LINE_BYTES + 1) / 2;
	wire [7:0] pp_in_b [0:2*LINE_BYTES-1];
	wire [7:0] pp_st0_b [0:PP_S0_OUT_A-1];
	genvar pp_u, pp_0, pp_1;
	generate
		for (pp_u = 0; pp_u < 2 * LINE_BYTES; pp_u = pp_u + 1) begin : g_pp_in
			assign pp_in_b[pp_u] = pp_raw_both[pp_u*8 +: 8];
		end
		for (pp_0 = 0; pp_0 < PP_S0_OUT_A; pp_0 = pp_0 + 1) begin : g_pp_st0
			localparam integer PR0 = ((pp_0 / 2) * 8) / PP_S0A_LEAF_CNT;
			wire [7:0] poh0 = s2_s0_oh_rep[PR0*8 +: 8];
			assign pp_st0_b[pp_0] =
				(({8{poh0[7]}} & pp_in_b[pp_0 + 7]) | ({8{poh0[6]}} & pp_in_b[pp_0 + 6])) |
				(({8{poh0[5]}} & pp_in_b[pp_0 + 5]) | ({8{poh0[4]}} & pp_in_b[pp_0 + 4])) |
				(({8{poh0[3]}} & pp_in_b[pp_0 + 3]) | ({8{poh0[2]}} & pp_in_b[pp_0 + 2])) |
				(({8{poh0[1]}} & pp_in_b[pp_0 + 1]) | ({8{poh0[0]}} & pp_in_b[pp_0]));
		end
		if (LINE_BYTES >= 64) begin : g_pp_s1_64
			for (pp_1 = 0; pp_1 < LINE_BYTES; pp_1 = pp_1 + 1) begin : g_pp_st1
				localparam integer PR1 = ((pp_1 / 2) * 8) / PP_S1A_LEAF_CNT;
				wire [7:0] poh1 = s2_s1_oh_rep[PR1*8 +: 8];
				assign pp_raw_copy_line[pp_1*8 +: 8] =
					(({8{poh1[7]}} & pp_st0_b[pp_1 + 56]) | ({8{poh1[6]}} & pp_st0_b[pp_1 + 48])) |
					(({8{poh1[5]}} & pp_st0_b[pp_1 + 40]) | ({8{poh1[4]}} & pp_st0_b[pp_1 + 32])) |
					(({8{poh1[3]}} & pp_st0_b[pp_1 + 24]) | ({8{poh1[2]}} & pp_st0_b[pp_1 + 16])) |
					(({8{poh1[1]}} & pp_st0_b[pp_1 + 8])  | ({8{poh1[0]}} & pp_st0_b[pp_1]));
			end
		end else begin : g_pp_s1_32
			for (pp_1 = 0; pp_1 < LINE_BYTES; pp_1 = pp_1 + 1) begin : g_pp_st1
				localparam integer PR1 = ((pp_1 / 2) * 8) / PP_S1A_LEAF_CNT;
				wire [7:0] poh1 = s2_s1_oh_rep[PR1*8 +: 8];
				assign pp_raw_copy_line[pp_1*8 +: 8] =
					(({8{poh1[3]}} & pp_st0_b[pp_1 + 24]) | ({8{poh1[2]}} & pp_st0_b[pp_1 + 16])) |
					(({8{poh1[1]}} & pp_st0_b[pp_1 + 8])  | ({8{poh1[0]}} & pp_st0_b[pp_1]));
			end
		end
	endgenerate
	wire s2_off_rep_unused = &{1'b0, s2_off_rep};
`endif

	/* Burst requests to AMB */
	wire [US:0] issue_p_end = p_p_end;

`ifdef SYNTH_KINTEX7
	wire [US-1:0] p_dlo_lo_b0, p_dlo_lo_b1, p_dlo_lo_b2, p_dlo_lo_b3;
	wire [US:0] issue_p_end_b0, issue_p_end_b1, issue_p_end_b2, issue_p_end_b3;
	wire [3:0] m_keep_b0, m_keep_b1, m_keep_b2, m_keep_b3;
	genvar g_ib;
	generate
		for (g_ib = 0; g_ib < US; g_ib = g_ib + 1) begin : g_pdl
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdl0 (.I0(p_dlo_lo[g_ib]), .O(p_dlo_lo_b0[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdl1 (.I0(p_dlo_lo[g_ib]), .O(p_dlo_lo_b1[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdl2 (.I0(p_dlo_lo[g_ib]), .O(p_dlo_lo_b2[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pdl3 (.I0(p_dlo_lo[g_ib]), .O(p_dlo_lo_b3[g_ib]));
		end
		for (g_ib = 0; g_ib <= US; g_ib = g_ib + 1) begin : g_ipe
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ipe0 (.I0(issue_p_end[g_ib]), .O(issue_p_end_b0[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ipe1 (.I0(issue_p_end[g_ib]), .O(issue_p_end_b1[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ipe2 (.I0(issue_p_end[g_ib]), .O(issue_p_end_b2[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ipe3 (.I0(issue_p_end[g_ib]), .O(issue_p_end_b3[g_ib]));
		end
		for (g_ib = 0; g_ib < 4; g_ib = g_ib + 1) begin : g_mkb
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mk0 (.I0(m_keep[g_ib]), .O(m_keep_b0[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mk1 (.I0(m_keep[g_ib]), .O(m_keep_b1[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mk2 (.I0(m_keep[g_ib]), .O(m_keep_b2[g_ib]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mk3 (.I0(m_keep[g_ib]), .O(m_keep_b3[g_ib]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US-1:0] p_dlo_lo_b0, p_dlo_lo_b1, p_dlo_lo_b2, p_dlo_lo_b3;
	wire [US:0] issue_p_end_b0, issue_p_end_b1, issue_p_end_b2, issue_p_end_b3;
	wire [3:0] m_keep_b0, m_keep_b1, m_keep_b2, m_keep_b3;
	genvar g_ib;
	generate
		for (g_ib = 0; g_ib < US; g_ib = g_ib + 1) begin : g_pdl
			assign p_dlo_lo_b0[g_ib] = p_dlo_lo[g_ib];
			assign p_dlo_lo_b1[g_ib] = p_dlo_lo[g_ib];
			assign p_dlo_lo_b2[g_ib] = p_dlo_lo[g_ib];
			assign p_dlo_lo_b3[g_ib] = p_dlo_lo[g_ib];
		end
		for (g_ib = 0; g_ib <= US; g_ib = g_ib + 1) begin : g_ipe
			assign issue_p_end_b0[g_ib] = issue_p_end[g_ib];
			assign issue_p_end_b1[g_ib] = issue_p_end[g_ib];
			assign issue_p_end_b2[g_ib] = issue_p_end[g_ib];
			assign issue_p_end_b3[g_ib] = issue_p_end[g_ib];
		end
		for (g_ib = 0; g_ib < 4; g_ib = g_ib + 1) begin : g_mkb
			assign m_keep_b0[g_ib] = m_keep[g_ib];
			assign m_keep_b1[g_ib] = m_keep[g_ib];
			assign m_keep_b2[g_ib] = m_keep[g_ib];
			assign m_keep_b3[g_ib] = m_keep[g_ib];
		end
	endgenerate
`else
	wire [US-1:0] p_dlo_lo_b0 = p_dlo_lo, p_dlo_lo_b1 = p_dlo_lo, p_dlo_lo_b2 = p_dlo_lo, p_dlo_lo_b3 = p_dlo_lo;
	wire [US:0] issue_p_end_b0 = issue_p_end, issue_p_end_b1 = issue_p_end, issue_p_end_b2 = issue_p_end, issue_p_end_b3 = issue_p_end;
	wire [3:0] m_keep_b0 = m_keep, m_keep_b1 = m_keep, m_keep_b2 = m_keep, m_keep_b3 = m_keep;
`endif

	reg [LINE_BYTES-1:0] issue_be;
	integer ib;
	always @(*) begin
		for (ib = 0; ib < LINE_BYTES; ib = ib + 1) begin
			if (ib < 16)
				issue_be[ib] = ib[US:0] >= {1'b0, p_dlo_lo_b0} && ib[US:0] < issue_p_end_b0 && !m_keep_b0[ib % 4];
			else if (ib < 32)
				issue_be[ib] = ib[US:0] >= {1'b0, p_dlo_lo_b1} && ib[US:0] < issue_p_end_b1 && !m_keep_b1[ib % 4];
			else if (ib < 48)
				issue_be[ib] = ib[US:0] >= {1'b0, p_dlo_lo_b2} && ib[US:0] < issue_p_end_b2 && !m_keep_b2[ib % 4];
			else
				issue_be[ib] = ib[US:0] >= {1'b0, p_dlo_lo_b3} && ib[US:0] < issue_p_end_b3 && !m_keep_b3[ib % 4];
		end
	end

	wire bst_can_fill_p = p_valid_fill && (!bank_active[bw_bank_dup] || !bank_closed[bw_bank_dup]);
	wire bst_rd_need = mc_issue;

`ifdef SYNTH_KINTEX7
	wire [2:0] bws_buf;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bws0 (.I0(bw_sub), .O(bws_buf[0]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bws1 (.I0(bw_sub), .O(bws_buf[1]));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bws2 (.I0(bw_sub), .O(bws_buf[2]));
	wire bw_sub_lo  = bws_buf[0];
	wire bw_sub_hi  = bws_buf[1];
	wire bw_sub_ctl = bws_buf[2];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] bws_buf;
	assign bws_buf[0] = bw_sub;
	assign bws_buf[1] = bw_sub;
	assign bws_buf[2] = bw_sub;
	wire bw_sub_lo  = bws_buf[0];
	wire bw_sub_hi  = bws_buf[1];
	wire bw_sub_ctl = bws_buf[2];
`else
	wire bw_sub_lo  = bw_sub;
	wire bw_sub_hi  = bw_sub;
	wire bw_sub_ctl = bw_sub;
`endif

	wire [LA_W-1:0] bst_rd_line_w = {
		bw_sub_hi ? p_src_n1[LA_W-1:14] : p_src_n0[LA_W-1:14],
		bw_sub_lo ? p_src_n1[13:0] : p_src_n0[13:0]
	};

	reg bst_rd_valid;
	reg [LA_W-1:0] bst_rd_line;
	reg [7:0] bst_rd_tag;

	always @(*) begin
		bst_rd_valid = 1'b0;
		bst_rd_line = bst_rd_line_w;
		bst_rd_tag = {1'b0, 2'd0, bw_bank_dup, bw_sub_ctl ? bw_slot_p1 : bw_slot_cnt};
		if (is_burst_mem && bst_can_fill_p && bst_rd_need && !bst_rd_err) begin
			bst_rd_valid = 1'b1;
		end
	end

	wire [3:0] bst_wr_line_0_lo = br_bank_l0_lo ? meta_dline_1_0[br_seg_idx_l0_lo][3:0] : meta_dline_0_0[br_seg_idx_l0_lo][3:0];
	wire [2:0] bst_wr_line_0_hi = br_bank_l0_hi ? meta_dline_1_0[br_seg_idx_l0_hi][6:4] : meta_dline_0_0[br_seg_idx_l0_hi][6:4];
	wire [6:0] bst_wr_line_0 = {bst_wr_line_0_hi, bst_wr_line_0_lo};

	wire [3:0] bst_wr_line_1_lo = br_bank_l1_lo ? meta_dline_1_1[br_seg_idx_l1_lo][3:0] : meta_dline_0_1[br_seg_idx_l1_lo][3:0];
	wire [2:0] bst_wr_line_1_hi = br_bank_l1_hi ? meta_dline_1_1[br_seg_idx_l1_hi][6:4] : meta_dline_0_1[br_seg_idx_l1_hi][6:4];
	wire [6:0] bst_wr_line_1 = {bst_wr_line_1_hi, bst_wr_line_1_lo};

	wire [3:0] bst_wr_line_2_lo = br_bank_l2 ? meta_dline_1_2[br_seg_idx_l2_lo][3:0] : meta_dline_0_2[br_seg_idx_l2_lo][3:0];
	wire [2:0] bst_wr_line_2_hi = br_bank_l2 ? meta_dline_1_2[br_seg_idx_l2_hi][6:4] : meta_dline_0_2[br_seg_idx_l2_hi][6:4];
	wire [6:0] bst_wr_line_2 = {bst_wr_line_2_hi, bst_wr_line_2_lo};

	wire [LA_W-22:0] bst_wr_line_3;
	generate
		if (LA_W > 25) begin : g_bwl3_hi
			wire [3:0] bst_wr_line_3_lo = br_bank_l3 ? meta_dline_1_3[br_seg_idx_l3_lo][3:0] : meta_dline_0_3[br_seg_idx_l3_lo][3:0];
			wire [LA_W-26:0] bst_wr_line_3_hi = br_bank_l3 ? meta_dline_1_3[br_seg_idx_l3_hi][LA_W-22:4] : meta_dline_0_3[br_seg_idx_l3_hi][LA_W-22:4];
			assign bst_wr_line_3 = {bst_wr_line_3_hi, bst_wr_line_3_lo};
		end else begin : g_bwl3_lo
			assign bst_wr_line_3 = br_bank_l3 ? meta_dline_1_3[br_seg_idx_l3_lo] : meta_dline_0_3[br_seg_idx_l3_lo];
		end
	endgenerate
	wire [LA_W-1:0] bst_wr_line_raw = {bst_wr_line_3, bst_wr_line_2, bst_wr_line_1, bst_wr_line_0};

	wire [3:0] bst_wr_be_0_lo = br_bank_b0 ? meta_be_1_0[br_seg_idx_b0_lo][3:0] : meta_be_0_0[br_seg_idx_b0_lo][3:0];
	wire [3:0] bst_wr_be_0_hi = br_bank_b0 ? meta_be_1_0[br_seg_idx_b0_hi][7:4] : meta_be_0_0[br_seg_idx_b0_hi][7:4];
	wire [7:0] bst_wr_be_0 = {bst_wr_be_0_hi, bst_wr_be_0_lo};

	wire [3:0] bst_wr_be_1_lo = br_bank_b1 ? meta_be_1_1[br_seg_idx_b1_lo][3:0] : meta_be_0_1[br_seg_idx_b1_lo][3:0];
	wire [3:0] bst_wr_be_1_hi = br_bank_b1 ? meta_be_1_1[br_seg_idx_b1_hi][7:4] : meta_be_0_1[br_seg_idx_b1_hi][7:4];
	wire [7:0] bst_wr_be_1 = {bst_wr_be_1_hi, bst_wr_be_1_lo};

	wire [3:0] bst_wr_be_2_lo = br_bank_b2 ? meta_be_1_2[br_seg_idx_b2_lo][3:0] : meta_be_0_2[br_seg_idx_b2_lo][3:0];
	wire [3:0] bst_wr_be_2_hi = br_bank_b2 ? meta_be_1_2[br_seg_idx_b2_hi][7:4] : meta_be_0_2[br_seg_idx_b2_hi][7:4];
	wire [7:0] bst_wr_be_2 = {bst_wr_be_2_hi, bst_wr_be_2_lo};

	wire [3:0] bst_wr_be_3_lo = br_bank_b3 ? meta_be_1_3[br_seg_idx_b3_lo][3:0] : meta_be_0_3[br_seg_idx_b3_lo][3:0];
	wire [3:0] bst_wr_be_3_hi = br_bank_b3 ? meta_be_1_3[br_seg_idx_b3_hi][7:4] : meta_be_0_3[br_seg_idx_b3_hi][7:4];
	wire [7:0] bst_wr_be_3 = {bst_wr_be_3_hi, bst_wr_be_3_lo};

	wire [3:0] bst_wr_be_4_lo = br_bank_b4 ? meta_be_1_4[br_seg_idx_b4_lo][3:0] : meta_be_0_4[br_seg_idx_b4_lo][3:0];
	wire [3:0] bst_wr_be_4_hi = br_bank_b4 ? meta_be_1_4[br_seg_idx_b4_hi][7:4] : meta_be_0_4[br_seg_idx_b4_hi][7:4];
	wire [7:0] bst_wr_be_4 = {bst_wr_be_4_hi, bst_wr_be_4_lo};

	wire [3:0] bst_wr_be_5_lo = br_bank_b5 ? meta_be_1_5[br_seg_idx_b5_lo][3:0] : meta_be_0_5[br_seg_idx_b5_lo][3:0];
	wire [3:0] bst_wr_be_5_hi = br_bank_b5 ? meta_be_1_5[br_seg_idx_b5_hi][7:4] : meta_be_0_5[br_seg_idx_b5_hi][7:4];
	wire [7:0] bst_wr_be_5 = {bst_wr_be_5_hi, bst_wr_be_5_lo};

	wire [3:0] bst_wr_be_6_lo = br_bank_b6 ? meta_be_1_6[br_seg_idx_b6_lo][3:0] : meta_be_0_6[br_seg_idx_b6_lo][3:0];
	wire [3:0] bst_wr_be_6_hi = br_bank_b6 ? meta_be_1_6[br_seg_idx_b6_hi][7:4] : meta_be_0_6[br_seg_idx_b6_hi][7:4];
	wire [7:0] bst_wr_be_6 = {bst_wr_be_6_hi, bst_wr_be_6_lo};

	wire [3:0] bst_wr_be_7_lo = br_bank_b7 ? meta_be_1_7[br_seg_idx_b7_lo][3:0] : meta_be_0_7[br_seg_idx_b7_lo][3:0];
	wire [3:0] bst_wr_be_7_hi = br_bank_b7 ? meta_be_1_7[br_seg_idx_b7_hi][7:4] : meta_be_0_7[br_seg_idx_b7_hi][7:4];
	wire [7:0] bst_wr_be_7 = {bst_wr_be_7_hi, bst_wr_be_7_lo};

	wire [LINE_BYTES-1:0] bst_wr_be_raw;
	generate
		if (LINE_BYTES >= 64) begin : g_be_wr64
			assign bst_wr_be_raw = {bst_wr_be_7, bst_wr_be_6, bst_wr_be_5, bst_wr_be_4,
			                        bst_wr_be_3, bst_wr_be_2, bst_wr_be_1, bst_wr_be_0};
		end else begin : g_be_wr32
			assign bst_wr_be_raw = {bst_wr_be_3, bst_wr_be_2, bst_wr_be_1, bst_wr_be_0};
		end
	endgenerate

`ifdef SYNTH_KINTEX7
	localparam integer NUM_BST_SL_BUFS = LINE_BYTES / 2;
	wire [LINE_BITS-1:0] bst_wr_wdata_raw;
	wire [NUM_BST_SL_BUFS-1:0] mc_slice;
	wire [3:0] mc_mid;
	genvar bwm, bwi;
	generate
		for (bwm = 0; bwm < 4; bwm = bwm + 1) begin : g_bst_mcm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mcm (.I0(m_copy), .O(mc_mid[bwm]));
		end
		for (bwi = 0; bwi < NUM_BST_SL_BUFS; bwi = bwi + 1) begin : g_bst_wd
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mcs (.I0(mc_mid[bwi / 8]), .O(mc_slice[bwi]));
			assign bst_wr_wdata_raw[bwi*16 +: 16] = mc_slice[bwi] ? pp_copy_line[bwi*16 +: 16] : fill_line[bwi*16 +: 16];
		end
	endgenerate

	always @(posedge clk) begin
		if (br_rdy_leaf[95])  s1_line[13:0]        <= bst_wr_line_raw[13:0];
		if (br_rdy_leaf[96]) begin
			s1_line[LA_W-1:14] <= bst_wr_line_raw[LA_W-1:14];
			s1_off_hi          <= {{(6-US){1'b0}}, r_off[US-1:2]};
		end
		if (br_rdy_leaf[101]) s2_line[13:0]        <= s1_line[13:0];
		if (br_rdy_leaf[102]) s2_line[LA_W-1:14]   <= s1_line[LA_W-1:14];
		if (br_rdy_leaf[107]) s3_line[13:0]        <= s2_line[13:0];
		if (br_rdy_leaf[108]) s3_line[LA_W-1:14]   <= s2_line[LA_W-1:14];
	end
	genvar bpi;
	generate
		for (bpi = 0; bpi < LINE_BYTES / 16; bpi = bpi + 1) begin : g_s_be
			always @(posedge clk) begin
				if (br_rdy_leaf[97 + bpi])  s1_be[bpi*16 +: 16] <= bst_wr_be_raw[bpi*16 +: 16];
				if (br_rdy_leaf[103 + bpi]) s2_be[bpi*16 +: 16] <= s1_be[bpi*16 +: 16];
				if (br_rdy_leaf[109 + bpi]) s3_be[bpi*16 +: 16] <= s2_be[bpi*16 +: 16];
			end
		end
		for (bpi = 0; bpi < LINE_BITS / 16; bpi = bpi + 1) begin : g_s3_wd
			always @(posedge clk) begin
				if (br_rdy_leaf[113 + bpi]) s3_wdata[bpi*16 +: 16] <= bst_wr_wdata_raw[bpi*16 +: 16];
			end
		end
	endgenerate
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
		end else if (!is_burst_ctl0 || fail_burst || bst_rd_err) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
		end else if (br_rdy_leaf[145]) begin
			s1_valid <= br_active;
			s2_valid <= s1_valid;
			s3_valid <= s2_valid;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	localparam integer NUM_BST_SL_BUFS = LINE_BYTES / 2;
	wire [LINE_BITS-1:0] bst_wr_wdata_raw;
	wire [NUM_BST_SL_BUFS-1:0] mc_slice;
	wire [3:0] mc_mid;
	genvar bwm, bwi;
	generate
		for (bwm = 0; bwm < 4; bwm = bwm + 1) begin : g_bst_mcm
			assign mc_mid[bwm] = m_copy;
		end
		for (bwi = 0; bwi < NUM_BST_SL_BUFS; bwi = bwi + 1) begin : g_bst_wd
			assign mc_slice[bwi] = mc_mid[bwi / 8];
			assign bst_wr_wdata_raw[bwi*16 +: 16] = mc_slice[bwi] ? pp_copy_line[bwi*16 +: 16] : fill_line[bwi*16 +: 16];
		end
	endgenerate

	always @(posedge clk) begin
		if (br_rdy_leaf[95])  s1_line[13:0]        <= bst_wr_line_raw[13:0];
		if (br_rdy_leaf[96]) begin
			s1_line[LA_W-1:14] <= bst_wr_line_raw[LA_W-1:14];
			s1_off_hi          <= {{(6-US){1'b0}}, r_off[US-1:2]};
		end
		if (br_rdy_leaf[101]) s2_line[13:0]        <= s1_line[13:0];
		if (br_rdy_leaf[102]) s2_line[LA_W-1:14]   <= s1_line[LA_W-1:14];
		if (br_rdy_leaf[107]) s3_line[13:0]        <= s2_line[13:0];
		if (br_rdy_leaf[108]) s3_line[LA_W-1:14]   <= s2_line[LA_W-1:14];
	end
	genvar bpi;
	generate
		for (bpi = 0; bpi < LINE_BYTES / 16; bpi = bpi + 1) begin : g_s_be
			always @(posedge clk) begin
				if (br_rdy_leaf[97 + bpi])  s1_be[bpi*16 +: 16] <= bst_wr_be_raw[bpi*16 +: 16];
				if (br_rdy_leaf[103 + bpi]) s2_be[bpi*16 +: 16] <= s1_be[bpi*16 +: 16];
				if (br_rdy_leaf[109 + bpi]) s3_be[bpi*16 +: 16] <= s2_be[bpi*16 +: 16];
			end
		end
		for (bpi = 0; bpi < LINE_BITS / 16; bpi = bpi + 1) begin : g_s3_wd
			always @(posedge clk) begin
				if (br_rdy_leaf[113 + bpi]) s3_wdata[bpi*16 +: 16] <= bst_wr_wdata_raw[bpi*16 +: 16];
			end
		end
	endgenerate
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
		end else if (!is_burst_ctl0 || fail_burst || bst_rd_err) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
		end else if (br_rdy_leaf[145]) begin
			s1_valid <= br_active;
			s2_valid <= s1_valid;
			s3_valid <= s2_valid;
		end
	end
`else
	wire [LINE_BITS-1:0] bst_wr_wdata_raw;
	genvar bwi;
	generate
		for (bwi = 0; bwi < NUM_WD_BUFS; bwi = bwi + 1) begin : g_bst_wd
			wire mc_w = m_copy_wd_rep[bwi / WD_SL_STEP];
			wire use_pp_raw = mc_w && !br_hold_v_rep[bwi];
			wire [15:0] alt_wd = mc_w ? s2_hold_line[bwi*16 +: 16] : fill_line[bwi*16 +: 16];
			assign bst_wr_wdata_raw[bwi*16 +: 16] = use_pp_raw ? pp_raw_copy_line[bwi*16 +: 16] : alt_wd;
		end
	endgenerate
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
			s3_v_rep <= 8'd0;
		end else if (!is_burst_ctl0 || fail_burst || bst_rd_err) begin
			s1_valid <= 1'b0;
			s2_valid <= 1'b0;
			s3_valid <= 1'b0;
			s3_v_rep <= 8'd0;
		end else if (br_pipe_rdy) begin
			s1_valid <= br_active;
			s2_valid <= s1_valid;
			s3_valid <= s2_valid;
			s3_v_rep <= {8{s2_valid}};
		end
	end
	always @(posedge clk) begin
		if (is_burst_ctl0 && !fail_burst && !bst_rd_err && br_pipe_rdy) begin
			s1_line <= bst_wr_line_raw;
			s2_line <= s1_line;
			s3_line <= s2_line;
			s1_be   <= bst_wr_be_raw;
			s2_be   <= s1_be;
			s3_be   <= s2_be;
		end
	end
	genvar g_s3w;
	generate
		for (g_s3w = 0; g_s3w < 8; g_s3w = g_s3w + 1) begin : g_s3w_sl
			always @(posedge clk) begin
				if (is_burst_ctl0 && !fail_burst && !bst_rd_err && br_rdy_sl[g_s3w])
					s3_wdata[g_s3w*PP_SL_BITS +: PP_SL_BITS] <= bst_wr_wdata_raw[g_s3w*PP_SL_BITS +: PP_SL_BITS];
			end
		end
	endgenerate
`endif
	wire bst_wr_valid = s3_valid;
	wire [LA_W-1:0] bst_wr_line = s3_line;
	wire [LINE_BYTES-1:0] bst_wr_be = s3_be;
	wire [LINE_BITS-1:0] bst_wr_wdata = s3_wdata;
	wire [7:0] bst_wr_tag = 8'h80;

`ifdef SYNTH_KINTEX7
	wire [7:0] bwv_buf;
	genvar g_bwv_i;
	generate
		for (g_bwv_i = 0; g_bwv_i < 8; g_bwv_i = g_bwv_i + 1) begin : g_bwv
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bwv (.I0(bst_wr_valid), .O(bwv_buf[g_bwv_i]));
		end
	endgenerate
	wire bwv_ln0 = bwv_buf[0];
	wire bwv_ln1 = bwv_buf[1];
	wire bwv_ln2 = bwv_buf[2];
	wire bwv_ln3 = bwv_buf[3];
	wire bwv_be0 = bwv_buf[4];
	wire bwv_be1 = bwv_buf[5];
	wire bwv_tag = bwv_buf[6];
	wire bwv_ctl = bwv_buf[7];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] bwv_buf;
	genvar g_bwv_i;
	generate
		for (g_bwv_i = 0; g_bwv_i < 8; g_bwv_i = g_bwv_i + 1) begin : g_bwv
			assign bwv_buf[g_bwv_i] = bst_wr_valid;
		end
	endgenerate
	wire bwv_ln0 = bwv_buf[0];
	wire bwv_ln1 = bwv_buf[1];
	wire bwv_ln2 = bwv_buf[2];
	wire bwv_ln3 = bwv_buf[3];
	wire bwv_be0 = bwv_buf[4];
	wire bwv_be1 = bwv_buf[5];
	wire bwv_tag = bwv_buf[6];
	wire bwv_ctl = bwv_buf[7];
`else
	wire bwv_ln0 = bst_wr_valid;
	wire bwv_ln1 = bst_wr_valid;
	wire bwv_ln2 = bst_wr_valid;
	wire bwv_ln3 = bst_wr_valid;
	wire bwv_be0 = bst_wr_valid;
	wire bwv_be1 = bst_wr_valid;
	wire bwv_tag = bst_wr_valid;
	wire bwv_ctl = bst_wr_valid;
`endif
	/* AMB Arbiter: write takes precedence over read to clear buffer space */
	wire bst_wr_fire = bwv_ctl && req_rdy_i;
	wire bst_rd_can = bst_rd_need && !bwv_ctl;
	wire bst_rd_fire = bst_rd_can && bst_can_fill_p && req_rdy_i;
	wire bst_req_valid = bwv_ctl || bst_rd_valid;
	wire bst_req_write = bwv_ctl;
	wire [LA_W-1:0] bst_req_line = {
		bwv_ln3 ? bst_wr_line[LA_W-1:21] : bst_rd_line[LA_W-1:21],
		bwv_ln2 ? bst_wr_line[20:14]      : bst_rd_line[20:14],
		bwv_ln1 ? bst_wr_line[13:7]       : bst_rd_line[13:7],
		bwv_ln0 ? bst_wr_line[6:0]        : bst_rd_line[6:0]
	};
	wire [LINE_BYTES-1:0] bst_req_be = {
		bwv_be1 ? bst_wr_be[LINE_BYTES-1:16] : { (LINE_BYTES-16){1'b1} },
		bwv_be0 ? bst_wr_be[15:0] : 16'hFFFF
	};
	wire [LINE_BITS-1:0] bst_req_wdata = bst_wr_wdata;
	wire [7:0] bst_req_tag = bwv_tag ? bst_wr_tag : bst_rd_tag;

	wire bst_bank_ready_0 = bank_avail[0];
	wire bst_bank_ready_1 = bank_avail[1];
	wire bst_bank_ready = (bw_bank_dup & bank_avail[1]) | (~bw_bank_dup & bank_avail[0]);
	wire bst_rd_rdy_0 = bst_rd_can && bst_bank_ready_0 && req_rdy_i;
	wire bst_rd_rdy_1 = bst_rd_can && bst_bank_ready_1 && req_rdy_i;
	wire bst_rd_rdy   = bst_rd_can && bst_bank_ready && req_rdy_i;
	wire p_two_line_cond = !p_two_lines || bw_sub;
	wire issue_two_ok = !mc_issue || (!s3_valid && req_rdy_i && p_two_line_cond);
	wire issue_rdy_0 = bank_avail[0] && issue_two_ok;
	wire issue_rdy_1 = bank_avail[1] && issue_two_ok;
	wire issue_fire_0 = p_valid_iss0 && issue_rdy_0;
	wire issue_fire_1 = p_valid_iss0 && issue_rdy_1;

	wire copy_rd_ready = issue_two_ok;
	assign issue_rdy = bst_bank_ready && copy_rd_ready;
	wire issue_fire_nopend = p_valid_iss1 && copy_rd_ready;
	wire issue_fire = issue_fire_nopend && bst_bank_ready;
`ifdef SYNTH_KINTEX7
	wire issue_close_fire = (bank_close_cond && issue_fire_nopend) && bst_bank_ready;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire issue_close_fire = (bank_close_cond && issue_fire_nopend) && bst_bank_ready;
`else
	wire issue_iss_ok = p_valid_iss1 && (!mc_issue || (!s3_valid && req_rdy_i && p_two_line_cond));
	wire close_p_hi = (p_seg_last && p_rows_b_zero) || r_bw_seg_last;
	wire close_cond_opt = close_p_hi || (p_two_lines ? bw_slot_ge_lim2 : bw_slot_ge_lim1);
	wire issue_close_0 = (!bw_bank && bank_avail[0] && issue_iss_ok) && close_cond_opt;
	wire issue_close_1 = ( bw_bank && bank_avail[1] && issue_iss_ok) && close_cond_opt;
	wire issue_close_fire = issue_close_0 || issue_close_1;
`endif
`ifdef SYNTH_KINTEX7
	wire [7:0] if_buf;
	LUT3 #(.INIT(8'h80)) u_if0 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[0]));
	LUT3 #(.INIT(8'h80)) u_if1 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[1]));
	LUT3 #(.INIT(8'h80)) u_if2 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[2]));
	LUT3 #(.INIT(8'h80)) u_if3 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[3]));
	LUT3 #(.INIT(8'h80)) u_if4 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[4]));
	LUT3 #(.INIT(8'h80)) u_if5 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[5]));
	LUT3 #(.INIT(8'h80)) u_if6 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[6]));
	LUT3 #(.INIT(8'h80)) u_if7 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[7]));
	wire issue_fire_m0_e01 = if_buf[0];
	wire issue_fire_m0_e23 = if_buf[1];
	wire issue_fire_m0_e45 = if_buf[2];
	wire issue_fire_m0_e67 = if_buf[3];
	wire issue_fire_m1_e01 = if_buf[4];
	wire issue_fire_m1_e23 = if_buf[5];
	wire issue_fire_m1_e45 = if_buf[6];
	wire issue_fire_m1_e67 = if_buf[7];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] if_buf;
	LUT3 #(.INIT(8'h80)) u_if0 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[0]));
	LUT3 #(.INIT(8'h80)) u_if1 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[1]));
	LUT3 #(.INIT(8'h80)) u_if2 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[2]));
	LUT3 #(.INIT(8'h80)) u_if3 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[3]));
	LUT3 #(.INIT(8'h80)) u_if4 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[4]));
	LUT3 #(.INIT(8'h80)) u_if5 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[5]));
	LUT3 #(.INIT(8'h80)) u_if6 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[6]));
	LUT3 #(.INIT(8'h80)) u_if7 (.I0(p_valid_iss1), .I1(bst_bank_ready), .I2(copy_rd_ready), .O(if_buf[7]));
	wire issue_fire_m0_e01 = if_buf[0];
	wire issue_fire_m0_e23 = if_buf[1];
	wire issue_fire_m0_e45 = if_buf[2];
	wire issue_fire_m0_e67 = if_buf[3];
	wire issue_fire_m1_e01 = if_buf[4];
	wire issue_fire_m1_e23 = if_buf[5];
	wire issue_fire_m1_e45 = if_buf[6];
	wire issue_fire_m1_e67 = if_buf[7];
`else
	wire issue_fire_m0_e01 = issue_fire, issue_fire_m0_e23 = issue_fire;
	wire issue_fire_m0_e45 = issue_fire, issue_fire_m0_e67 = issue_fire;
	wire issue_fire_m1_e01 = issue_fire, issue_fire_m1_e23 = issue_fire;
	wire issue_fire_m1_e45 = issue_fire, issue_fire_m1_e67 = issue_fire;
`endif
	wire issue_fire_m0 = issue_fire_m0_e01;
	wire issue_fire_m1 = issue_fire_m1_e01;

	assign bank_close_fire_0 = issue_fire_0 && bank_close_cond && !bw_bank;
	assign bank_close_fire_1 = issue_fire_1 && bank_close_cond && bw_bank;
	assign bank_close_fire = issue_close_fire;
	wire step_p = skid_in_ready;
	wire g2_ready = !g2_v_ctl || step_p;
	wire g_ready  = !g_valid  || g2_ready;
	wire w_ready  = !w_valid  || g_ready;
	wire g2_step  = g_valid && g2_ready;
	wire g_step   = w_valid && g_ready;
`ifdef SYNTH_KINTEX7
	wire w_ready_ws_a, w_ready_ws_b, w_ready_ctl, w_ready_rs;
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrwsa (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ws_a));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrwsb (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ws_b));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrctl (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ctl));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrrs  (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_rs));

	wire walk_step_ws_a, walk_step_ws_b, walk_step_ctl, walk_step_rs;
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswa (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ws_a), .O(walk_step_ws_a));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswb (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ws_b), .O(walk_step_ws_b));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswc (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ctl),  .O(walk_step_ctl));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswr (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_rs),   .O(walk_step_rs));

	wire walk_step = walk_step_ctl;
	wire row_step = walk_step_ctl && bsl_ctl;
	wire walk_step_a = walk_step_rs;
	wire walk_step_b = walk_step_ctl;

	wire [6:0] wlen_buf;
	wire [4:0] glen_buf;
	wire [7:0] g2s_buf;
	wire [5:0] rst_buf;
	wire [11:0] ws_buf;
	genvar g_bi;
	generate
		for (g_bi = 0; g_bi < 7; g_bi = g_bi + 1) begin : g_buf_wlen
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wb (.I0(walk_step_ctl), .O(wlen_buf[g_bi]));
		end
		for (g_bi = 0; g_bi < 5; g_bi = g_bi + 1) begin : g_buf_glen
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_gb (.I0(g_step), .O(glen_buf[g_bi]));
		end
		for (g_bi = 0; g_bi < 8; g_bi = g_bi + 1) begin : g_buf_g2s
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_g2b (.I0(g2_step), .O(g2s_buf[g_bi]));
		end
		for (g_bi = 0; g_bi < 6; g_bi = g_bi + 1) begin : g_buf_rs
			LUT3 #(.INIT(8'h40)) u_rs (.I0(cmp_last_bo), .I1(rem_b_small_rs), .I2(walk_step_rs), .O(rst_buf[g_bi]));
		end
		for (g_bi = 0; g_bi < 6; g_bi = g_bi + 1) begin : g_buf_wsa
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wsba (.I0(walk_step_ws_a), .O(ws_buf[g_bi]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wsbb (.I0(walk_step_ws_b), .O(ws_buf[g_bi + 6]));
		end
	endgenerate
	wire w_len_da0_a = wlen_buf[0];
	wire w_len_da0_b = wlen_buf[1];
	wire w_len_da1   = wlen_buf[2];
	wire w_len_sa0_a = wlen_buf[3];
	wire w_len_sa0_b = wlen_buf[4];
	wire w_len_sa1   = wlen_buf[5];
	wire w_len_ctl   = wlen_buf[6];

	wire g_len_da0 = glen_buf[0];
	wire g_len_da1 = glen_buf[1];
	wire g_len_sa0 = glen_buf[2];
	wire g_len_sa1 = glen_buf[3];
	wire g_len_ctl = glen_buf[4];

	wire g2_step_dl0   = g2s_buf[0];
	wire g2_step_dl1   = g2s_buf[1];
	wire g2_step_s00   = g2s_buf[2];
	wire g2_step_s01   = g2s_buf[3];
	wire g2_step_s10   = g2s_buf[4];
	wire g2_step_s11   = g2s_buf[5];
	wire g2_step_ctl_a = g2s_buf[6];
	wire g2_step_ctl_b = g2s_buf[7];

	wire rst_dst0 = rst_buf[0];
	wire rst_dst1 = rst_buf[1];
	wire rst_src0 = rst_buf[2];
	wire rst_src1 = rst_buf[3];
	wire rst_ctl0 = rst_buf[4];
	wire rst_ctl1 = rst_buf[5];
	wire rst_row  = 1'b0;

	wire ws_px0  = ws_buf[0];
	wire ws_px1  = ws_buf[1];
	wire ws_spx0 = ws_buf[2];
	wire ws_spx1 = ws_buf[3];
	wire ws_rem0 = ws_buf[4];
	wire ws_rem1 = ws_buf[5];
	wire ws_ctl  = ws_buf[6];
	wire ws_gctl = ws_buf[7];
	wire ws_dxb0 = ws_buf[8];
	wire ws_dxb1 = ws_buf[9];
	wire ws_sxb0 = ws_buf[10];
	wire ws_sxb1 = ws_buf[11];
	wire ws_walk = walk_step;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire w_ready_ws_a, w_ready_ws_b, w_ready_ctl, w_ready_rs;
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrwsa (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ws_a));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrwsb (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ws_b));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrctl (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_ctl));
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hFF7F)) u_wrrs  (.I0(w_valid), .I1(g_valid), .I2(g2_valid), .I3(skid_in_ready), .O(w_ready_rs));

	wire walk_step_ws_a, walk_step_ws_b, walk_step_ctl, walk_step_rs;
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswa (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ws_a), .O(walk_step_ws_a));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswb (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ws_b), .O(walk_step_ws_b));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswc (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_ctl),  .O(walk_step_ctl));
	(* dont_touch = "true" *) LUT3 #(.INIT(8'h40)) u_wswr (.I0(bw_walk_done), .I1(is_burst), .I2(w_ready_rs),   .O(walk_step_rs));

	wire walk_step = walk_step_ctl;
	wire row_step = walk_step_ctl && bsl_ctl;
	wire walk_step_a = walk_step_rs;
	wire walk_step_b = walk_step_ctl;

	wire [6:0] wlen_buf;
	wire [4:0] glen_buf;
	wire [7:0] g2s_buf;
	wire [5:0] rst_buf;
	wire [11:0] ws_buf;
	genvar g_bi;
	generate
		for (g_bi = 0; g_bi < 7; g_bi = g_bi + 1) begin : g_buf_wlen
			assign wlen_buf[g_bi] = walk_step_ctl;
		end
		for (g_bi = 0; g_bi < 5; g_bi = g_bi + 1) begin : g_buf_glen
			assign glen_buf[g_bi] = g_step;
		end
		for (g_bi = 0; g_bi < 8; g_bi = g_bi + 1) begin : g_buf_g2s
			assign g2s_buf[g_bi] = g2_step;
		end
		for (g_bi = 0; g_bi < 6; g_bi = g_bi + 1) begin : g_buf_rs
			LUT3 #(.INIT(8'h40)) u_rs (.I0(cmp_last_bo), .I1(rem_b_small_rs), .I2(walk_step_rs), .O(rst_buf[g_bi]));
		end
		for (g_bi = 0; g_bi < 6; g_bi = g_bi + 1) begin : g_buf_wsa
			assign ws_buf[g_bi] = walk_step_ws_a;
			assign ws_buf[g_bi + 6] = walk_step_ws_b;
		end
	endgenerate
	wire w_len_da0_a = wlen_buf[0];
	wire w_len_da0_b = wlen_buf[1];
	wire w_len_da1   = wlen_buf[2];
	wire w_len_sa0_a = wlen_buf[3];
	wire w_len_sa0_b = wlen_buf[4];
	wire w_len_sa1   = wlen_buf[5];
	wire w_len_ctl   = wlen_buf[6];

	wire g_len_da0 = glen_buf[0];
	wire g_len_da1 = glen_buf[1];
	wire g_len_sa0 = glen_buf[2];
	wire g_len_sa1 = glen_buf[3];
	wire g_len_ctl = glen_buf[4];

	wire g2_step_dl0   = g2s_buf[0];
	wire g2_step_dl1   = g2s_buf[1];
	wire g2_step_s00   = g2s_buf[2];
	wire g2_step_s01   = g2s_buf[3];
	wire g2_step_s10   = g2s_buf[4];
	wire g2_step_s11   = g2s_buf[5];
	wire g2_step_ctl_a = g2s_buf[6];
	wire g2_step_ctl_b = g2s_buf[7];

	wire rst_dst0 = rst_buf[0];
	wire rst_dst1 = rst_buf[1];
	wire rst_src0 = rst_buf[2];
	wire rst_src1 = rst_buf[3];
	wire rst_ctl0 = rst_buf[4];
	wire rst_ctl1 = rst_buf[5];
	wire rst_row  = 1'b0;

	wire ws_px0  = ws_buf[0];
	wire ws_px1  = ws_buf[1];
	wire ws_spx0 = ws_buf[2];
	wire ws_spx1 = ws_buf[3];
	wire ws_rem0 = ws_buf[4];
	wire ws_rem1 = ws_buf[5];
	wire ws_ctl  = ws_buf[6];
	wire ws_gctl = ws_buf[7];
	wire ws_dxb0 = ws_buf[8];
	wire ws_dxb1 = ws_buf[9];
	wire ws_sxb0 = ws_buf[10];
	wire ws_sxb1 = ws_buf[11];
	wire ws_walk = walk_step;
`else
	wire walk_step = is_burst && !bw_walk_done && w_ready;
	wire row_step = walk_step && bsl_ctl;
	wire walk_step_a = walk_step, walk_step_b = walk_step;
	wire w_len_da0_a = walk_step, w_len_da0_b = walk_step, w_len_da1 = walk_step;
	wire w_len_sa0_a = walk_step, w_len_sa0_b = walk_step, w_len_sa1 = walk_step;
	wire w_len_ctl = walk_step;
	wire g_len_da0 = g_step, g_len_da1 = g_step;
	wire g_len_sa0 = g_step, g_len_sa1 = g_step;
	wire g_len_ctl = g_step;
	wire g2_step_dl0 = g2_step, g2_step_dl1 = g2_step;
	wire g2_step_s00 = g2_step, g2_step_s01 = g2_step;
	wire g2_step_s10 = g2_step, g2_step_s11 = g2_step;
	wire g2_step_ctl_a = g2_step, g2_step_ctl_b = g2_step;
	wire rst_dst0 = row_step, rst_dst1 = row_step;
	wire rst_src0 = row_step, rst_src1 = row_step;
	wire rst_ctl0 = row_step, rst_ctl1 = row_step;
	wire rst_row  = row_step;
	wire ws_px0  = walk_step, ws_px1  = walk_step;
	wire ws_spx0 = walk_step, ws_spx1 = walk_step;
	wire ws_rem0 = walk_step, ws_rem1 = walk_step;
	wire ws_walk = walk_step;
	wire ws_gctl = walk_step, ws_ctl  = walk_step;
	wire ws_dxb0 = walk_step, ws_dxb1 = walk_step;
	wire ws_sxb0 = walk_step, ws_sxb1 = walk_step;
`endif

	/* Mux external AMB outputs between burst and legacy port (with outbound am_skid for ASIC) */
`ifdef SYNTH_KINTEX7
	assign req_valid = m_burst ? bst_req_valid : leg_req_valid;
	assign req_write = m_burst ? bst_req_write : leg_req_write;
	assign req_line  = m_burst ? bst_req_line  : leg_req_line;
	assign req_be    = m_burst ? bst_req_be    : leg_req_be;
	assign req_tag   = m_burst ? bst_req_tag   : leg_req_tag;

	localparam NUM_REQ_BYTE_BUFS = LINE_BYTES / 2;
	wire [NUM_REQ_BYTE_BUFS-1:0] mb_slice;
	wire [3:0] mb_mid;
	genvar req_m, req_i;
	generate
		for (req_m = 0; req_m < 4; req_m = req_m + 1) begin : g_req_mbm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbm (.I0(m_burst), .O(mb_mid[req_m]));
		end
		for (req_i = 0; req_i < NUM_REQ_BYTE_BUFS; req_i = req_i + 1) begin : g_req_wd
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mbs (.I0(mb_mid[req_i / 8]), .O(mb_slice[req_i]));
			assign req_wdata[req_i*16 +: 16] = mb_slice[req_i] ? bst_req_wdata[req_i*16 +: 16] : leg_req_wdata[req_i*16 +: 16];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	assign req_valid = m_burst ? bst_req_valid : leg_req_valid;
	assign req_write = m_burst ? bst_req_write : leg_req_write;
	assign req_line  = m_burst ? bst_req_line  : leg_req_line;
	assign req_be    = m_burst ? bst_req_be    : leg_req_be;
	assign req_tag   = m_burst ? bst_req_tag   : leg_req_tag;

	localparam NUM_REQ_BYTE_BUFS = LINE_BYTES / 2;
	wire [NUM_REQ_BYTE_BUFS-1:0] mb_slice;
	wire [3:0] mb_mid;
	genvar req_m, req_i;
	generate
		for (req_m = 0; req_m < 4; req_m = req_m + 1) begin : g_req_mbm
			assign mb_mid[req_m] = m_burst;
		end
		for (req_i = 0; req_i < NUM_REQ_BYTE_BUFS; req_i = req_i + 1) begin : g_req_wd
			assign mb_slice[req_i] = mb_mid[req_i / 8];
			assign req_wdata[req_i*16 +: 16] = mb_slice[req_i] ? bst_req_wdata[req_i*16 +: 16] : leg_req_wdata[req_i*16 +: 16];
		end
	endgenerate
`else
	wire mux_req_valid = m_burst ? bst_req_valid : leg_req_valid;
	wire mux_req_write = m_burst ? bst_req_write : leg_req_write;
	wire [LA_W-1:0] mux_req_line = m_burst ? bst_req_line : leg_req_line;
	wire [LINE_BYTES-1:0] mux_req_be = m_burst ? bst_req_be : leg_req_be;
	wire [7:0] mux_req_tag = m_burst ? bst_req_tag : leg_req_tag;
	wire [LINE_BITS-1:0] mux_req_wdata;
	genvar req_i;
	generate
		for (req_i = 0; req_i < NUM_WD_BUFS; req_i = req_i + 1) begin : g_req_wd
			assign mux_req_wdata[req_i*16 +: 16] = m_burst ? bst_req_wdata[req_i*16 +: 16] : leg_req_wdata[req_i*16 +: 16];
		end
	endgenerate
	localparam integer REQ_SK_W = 1 + LA_W + LINE_BYTES + LINE_BITS + 8;
	am_skid #(.W(REQ_SK_W)) u_req_skid (
		.clk(clk), .rst_n(rst_n),
		.in_valid(mux_req_valid), .in_ready(req_rdy_i),
		.in_data({mux_req_write, mux_req_line, mux_req_be, mux_req_wdata, mux_req_tag}),
		.out_valid(req_valid), .out_ready(req_ready),
		.out_data({req_write, req_line, req_be, req_wdata, req_tag})
	);
`endif

	assign busy = r_busy_q;
	assign fsm = (r_st_burst || r_st_init_b || r_st_init1) ? 4'd15 : state[3:0];

	/* The segment's end: the next pixel, or the next row. */
	wire [16:0] sn17 = {{(16 - US){1'b0}}, sn};
	wire seg_last = r_seg_last;
	wire [63:0] r_dstride_ext = {32'd0, r_dstride};
	wire [63:0] r_sstride_ext = {32'd0, r_sstride};
	wire [ADDR_W-1:0] ext_dstride = r_dstride_ext[ADDR_W-1:0];
	wire [ADDR_W-1:0] ext_sstride = r_sstride_ext[ADDR_W-1:0];
	wire [ADDR_W-1:0] tile_step = tile_bytes(psl_leg0, 1'b0);
	wire [ADDR_W-1:0] tile_back = tile_bytes(psl_leg1_b, 1'b1);
	wire [ADDR_W-1:0] dst_back, src_back, dst_neg, src_neg;
	wire [16:0] y1_last, ys_rev;
	wire [5:0] step_co_unused;
	wire [15:0] cs32_co_unused;

`ifdef SYNTH_KINTEX7
	/* 16+16 carry-select on Kintex-7 keeps CARRY4 Pre<=1, Post<=1 (Rule 1 clean). */
	wire dbk_b0, dbk_b10, dbk_b11, sbk_b0, sbk_b10, sbk_b11;
	wire [ADDR_W-17:0] dbk_d10, dbk_d11, sbk_d10, sbk_d11;
	am_sub #(.W(16))        s_dback_lo (.a(ext_dstride[15:0]),        .b(r_tile_back[15:0]),        .bi(1'b0), .d(dst_back[15:0]), .bo(dbk_b0));
	am_sub #(.W(ADDR_W-16)) s_dback_h0 (.a(ext_dstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b0), .d(dbk_d10),        .bo(dbk_b10));
	am_sub #(.W(ADDR_W-16)) s_dback_h1 (.a(ext_dstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b1), .d(dbk_d11),        .bo(dbk_b11));
	assign dst_back[ADDR_W-1:16] = dbk_b0 ? dbk_d11 : dbk_d10;
	assign step_co_unused[0] = (dbk_b0 ? dbk_b11 : dbk_b10) & dbk_b10;

	am_sub #(.W(16))        s_sback_lo (.a(ext_sstride[15:0]),        .b(r_tile_back[15:0]),        .bi(1'b0), .d(src_back[15:0]), .bo(sbk_b0));
	am_sub #(.W(ADDR_W-16)) s_sback_h0 (.a(ext_sstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b0), .d(sbk_d10),        .bo(sbk_b10));
	am_sub #(.W(ADDR_W-16)) s_sback_h1 (.a(ext_sstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b1), .d(sbk_d11),        .bo(sbk_b11));
	assign src_back[ADDR_W-1:16] = sbk_b0 ? sbk_d11 : sbk_d10;
	assign step_co_unused[1] = (sbk_b0 ? sbk_b11 : sbk_b10) & sbk_b10;

	wire dng_b0, dng_b10, dng_b11, sng_b0, sng_b10, sng_b11;
	wire [ADDR_W-17:0] dng_d10, dng_d11, sng_d10, sng_d11;
	am_sub #(.W(16))        s_dneg_lo (.a(16'd0),              .b(ext_dstride[15:0]),        .bi(1'b0), .d(dst_neg[15:0]), .bo(dng_b0));
	am_sub #(.W(ADDR_W-16)) s_dneg_h0 (.a({(ADDR_W-16){1'b0}}), .b(ext_dstride[ADDR_W-1:16]), .bi(1'b0), .d(dng_d10),        .bo(dng_b10));
	am_sub #(.W(ADDR_W-16)) s_dneg_h1 (.a({(ADDR_W-16){1'b0}}), .b(ext_dstride[ADDR_W-1:16]), .bi(1'b1), .d(dng_d11),        .bo(dng_b11));
	assign dst_neg[ADDR_W-1:16] = dng_b0 ? dng_d11 : dng_d10;
	assign step_co_unused[2] = (dng_b0 ? dng_b11 : dng_b10) & dng_b10;

	am_sub #(.W(16))        s_sneg_lo (.a(16'd0),              .b(ext_sstride[15:0]),        .bi(1'b0), .d(src_neg[15:0]), .bo(sng_b0));
	am_sub #(.W(ADDR_W-16)) s_sneg_h0 (.a({(ADDR_W-16){1'b0}}), .b(ext_sstride[ADDR_W-1:16]), .bi(1'b0), .d(sng_d10),        .bo(sng_b10));
	am_sub #(.W(ADDR_W-16)) s_sneg_h1 (.a({(ADDR_W-16){1'b0}}), .b(ext_sstride[ADDR_W-1:16]), .bi(1'b1), .d(sng_d11),        .bo(sng_b11));
	assign src_neg[ADDR_W-1:16] = sng_b0 ? sng_d11 : sng_d10;
	assign step_co_unused[3] = (sng_b0 ? sng_b11 : sng_b10) & sng_b10;
`elsif SYNTH_KINTEX_ULTRASCALE
	/* 16+16 carry-select on Kintex-7 keeps CARRY4 Pre<=1, Post<=1 (Rule 1 clean). */
	wire dbk_b0, dbk_b10, dbk_b11, sbk_b0, sbk_b10, sbk_b11;
	wire [ADDR_W-17:0] dbk_d10, dbk_d11, sbk_d10, sbk_d11;
	am_sub #(.W(16))        s_dback_lo (.a(ext_dstride[15:0]),        .b(r_tile_back[15:0]),        .bi(1'b0), .d(dst_back[15:0]), .bo(dbk_b0));
	am_sub #(.W(ADDR_W-16)) s_dback_h0 (.a(ext_dstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b0), .d(dbk_d10),        .bo(dbk_b10));
	am_sub #(.W(ADDR_W-16)) s_dback_h1 (.a(ext_dstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b1), .d(dbk_d11),        .bo(dbk_b11));
	assign dst_back[ADDR_W-1:16] = dbk_b0 ? dbk_d11 : dbk_d10;
	assign step_co_unused[0] = (dbk_b0 ? dbk_b11 : dbk_b10) & dbk_b10;

	am_sub #(.W(16))        s_sback_lo (.a(ext_sstride[15:0]),        .b(r_tile_back[15:0]),        .bi(1'b0), .d(src_back[15:0]), .bo(sbk_b0));
	am_sub #(.W(ADDR_W-16)) s_sback_h0 (.a(ext_sstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b0), .d(sbk_d10),        .bo(sbk_b10));
	am_sub #(.W(ADDR_W-16)) s_sback_h1 (.a(ext_sstride[ADDR_W-1:16]), .b(r_tile_back[ADDR_W-1:16]), .bi(1'b1), .d(sbk_d11),        .bo(sbk_b11));
	assign src_back[ADDR_W-1:16] = sbk_b0 ? sbk_d11 : sbk_d10;
	assign step_co_unused[1] = (sbk_b0 ? sbk_b11 : sbk_b10) & sbk_b10;

	wire dng_b0, dng_b10, dng_b11, sng_b0, sng_b10, sng_b11;
	wire [ADDR_W-17:0] dng_d10, dng_d11, sng_d10, sng_d11;
	am_sub #(.W(16))        s_dneg_lo (.a(16'd0),              .b(ext_dstride[15:0]),        .bi(1'b0), .d(dst_neg[15:0]), .bo(dng_b0));
	am_sub #(.W(ADDR_W-16)) s_dneg_h0 (.a({(ADDR_W-16){1'b0}}), .b(ext_dstride[ADDR_W-1:16]), .bi(1'b0), .d(dng_d10),        .bo(dng_b10));
	am_sub #(.W(ADDR_W-16)) s_dneg_h1 (.a({(ADDR_W-16){1'b0}}), .b(ext_dstride[ADDR_W-1:16]), .bi(1'b1), .d(dng_d11),        .bo(dng_b11));
	assign dst_neg[ADDR_W-1:16] = dng_b0 ? dng_d11 : dng_d10;
	assign step_co_unused[2] = (dng_b0 ? dng_b11 : dng_b10) & dng_b10;

	am_sub #(.W(16))        s_sneg_lo (.a(16'd0),              .b(ext_sstride[15:0]),        .bi(1'b0), .d(src_neg[15:0]), .bo(sng_b0));
	am_sub #(.W(ADDR_W-16)) s_sneg_h0 (.a({(ADDR_W-16){1'b0}}), .b(ext_sstride[ADDR_W-1:16]), .bi(1'b0), .d(sng_d10),        .bo(sng_b10));
	am_sub #(.W(ADDR_W-16)) s_sneg_h1 (.a({(ADDR_W-16){1'b0}}), .b(ext_sstride[ADDR_W-1:16]), .bi(1'b1), .d(sng_d11),        .bo(sng_b11));
	assign src_neg[ADDR_W-1:16] = sng_b0 ? sng_d11 : sng_d10;
	assign step_co_unused[3] = (sng_b0 ? sng_b11 : sng_b10) & sng_b10;
`else
	am_sub #(.W(ADDR_W)) s_dback (.a(ext_dstride),          .b(r_tile_back), .bi(1'b0), .d(dst_back), .bo(step_co_unused[0]));
	am_sub #(.W(ADDR_W)) s_sback (.a(ext_sstride),          .b(r_tile_back), .bi(1'b0), .d(src_back), .bo(step_co_unused[1]));
	am_sub #(.W(ADDR_W)) s_dneg  (.a({ADDR_W{1'b0}}),       .b(ext_dstride), .bi(1'b0), .d(dst_neg),  .bo(step_co_unused[2]));
	am_sub #(.W(ADDR_W)) s_sneg  (.a({ADDR_W{1'b0}}),       .b(ext_sstride), .bi(1'b0), .d(src_neg),  .bo(step_co_unused[3]));
`endif

	wire [ADDR_W-1:0] dst_step = {
		mdt_step1 ? (ydl == 2'd3 ? r_dst_back[ADDR_W-1:16] : r_tile_step[ADDR_W-1:16]) : ext_dstride[ADDR_W-1:16],
		mdt_step0 ? (ydl == 2'd3 ? r_dst_back[15:0]        : r_tile_step[15:0])        : ext_dstride[15:0]
	};
	wire [ADDR_W-1:0] src_step = {
		mst_step1 ? (ysl == 2'd3 ? r_src_back[ADDR_W-1:16] : r_tile_step[ADDR_W-1:16]) : ext_sstride[ADDR_W-1:16],
		mst_step0 ? (ysl == 2'd3 ? r_src_back[15:0]        : r_tile_step[15:0])        : ext_sstride[15:0]
	};

	/* S_START: the first row (the last one when reversed). */
`ifdef SYNTH_KINTEX7
	wire y1l_bo, y1l_h0, y1l_h1;
	wire [8:0] y1l_hi0, y1l_hi1;
	am_sub #(.W(8)) s_y1_l  (.a(y1[7:0]),  .b(8'd0), .bi(1'b1), .d(y1_last[7:0]), .bo(y1l_bo));
	am_sub #(.W(9)) s_y1_h0 (.a(y1[16:8]), .b(9'd0), .bi(1'b0), .d(y1l_hi0),      .bo(y1l_h0));
	am_sub #(.W(9)) s_y1_h1 (.a(y1[16:8]), .b(9'd0), .bi(1'b1), .d(y1l_hi1),      .bo(y1l_h1));
	assign y1_last[16:8] = y1l_bo ? y1l_hi1 : y1l_hi0;
	assign step_co_unused[4] = y1l_bo ? y1l_h1 : y1l_h0;

	wire ysr_c0, ysr_h0, ysr_h1;
	wire [8:0] ysr_hi0, ysr_hi1;
	am_add #(.W(8)) a_ys_l  (.a(sy0[7:0]),  .b(rows[7:0]),  .ci(1'b0), .s(ys_rev[7:0]), .co(ysr_c0));
	am_add #(.W(9)) a_ys_h0 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b0), .s(ysr_hi0),     .co(ysr_h0));
	am_add #(.W(9)) a_ys_h1 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b1), .s(ysr_hi1),     .co(ysr_h1));
	assign ys_rev[16:8] = ysr_c0 ? ysr_hi1 : ysr_hi0;
	assign step_co_unused[5] = ysr_c0 ? ysr_h1 : ysr_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire y1l_bo, y1l_h0, y1l_h1;
	wire [8:0] y1l_hi0, y1l_hi1;
	am_sub #(.W(8)) s_y1_l  (.a(y1[7:0]),  .b(8'd0), .bi(1'b1), .d(y1_last[7:0]), .bo(y1l_bo));
	am_sub #(.W(9)) s_y1_h0 (.a(y1[16:8]), .b(9'd0), .bi(1'b0), .d(y1l_hi0),      .bo(y1l_h0));
	am_sub #(.W(9)) s_y1_h1 (.a(y1[16:8]), .b(9'd0), .bi(1'b1), .d(y1l_hi1),      .bo(y1l_h1));
	assign y1_last[16:8] = y1l_bo ? y1l_hi1 : y1l_hi0;
	assign step_co_unused[4] = y1l_bo ? y1l_h1 : y1l_h0;

	wire ysr_c0, ysr_h0, ysr_h1;
	wire [8:0] ysr_hi0, ysr_hi1;
	am_add #(.W(8)) a_ys_l  (.a(sy0[7:0]),  .b(rows[7:0]),  .ci(1'b0), .s(ys_rev[7:0]), .co(ysr_c0));
	am_add #(.W(9)) a_ys_h0 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b0), .s(ysr_hi0),     .co(ysr_h0));
	am_add #(.W(9)) a_ys_h1 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b1), .s(ysr_hi1),     .co(ysr_h1));
	assign ys_rev[16:8] = ysr_c0 ? ysr_hi1 : ysr_hi0;
	assign step_co_unused[5] = ysr_c0 ? ysr_h1 : ysr_h0;
`else
	/* Carry-free y1_last decrement (5 cmos3) + 8+9 a_ys with pre-muxed mr_ys/mst_acc (<= 11 cmos3). */
	assign y1_last[0] = ~y1[0];
	genvar g_y1l;
	generate
		for (g_y1l = 1; g_y1l < 17; g_y1l = g_y1l + 1) begin : g_y1l_fast
			assign y1_last[g_y1l] = y1[g_y1l] ^ (~(|y1[g_y1l-1:0]));
		end
	endgenerate
	assign step_co_unused[4] = 1'b0;

	wire ysr_c0, ysr_h0, ysr_h1;
	wire [8:0] ysr_hi0, ysr_hi1;
	am_add #(.W(8)) a_ys_l  (.a(sy0[7:0]),  .b(rows[7:0]),  .ci(1'b0), .s(ys_rev[7:0]), .co(ysr_c0));
	am_add #(.W(9)) a_ys_h0 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b0), .s(ysr_hi0),     .co(ysr_h0));
	am_add #(.W(9)) a_ys_h1 (.a(sy0[16:8]), .b(rows[16:8]), .ci(1'b1), .s(ysr_hi1),     .co(ysr_h1));
	assign ys_rev[16:8] = ysr_c0 ? ysr_hi1 : ysr_hi0;
	assign step_co_unused[5] = (ysr_c0 ? ysr_h1 : ysr_h0) & ysr_h0;
	wire [7:0] ys_s_lo = mr_ys0 ? ys_rev[7:0] : sy0[7:0];
	wire [16:8] ys_s_hi0 = mr_ys1 ? ysr_hi0 : sy0[16:8];
	wire [16:8] ys_s_hi1 = mr_ys1 ? ysr_hi1 : sy0[16:8];
	wire [16:0] ys_cand0 = mst_acc ? {2'b00, ys_s_hi0, ys_s_lo[7:2]} : {ys_s_hi0, ys_s_lo};
	wire [16:0] ys_cand1 = mst_acc ? {2'b00, ys_s_hi1, ys_s_lo[7:2]} : {ys_s_hi1, ys_s_lo};
	wire [16:0] r_ys_start_d;
	assign r_ys_start_d[5:0]  = ys_cand0[5:0];
	assign r_ys_start_d[16:6] = ysr_c0 ? ys_cand1[16:6] : ys_cand0[16:6];
`endif
	wire [16:0] yd_start = {mr_yd1 ? y1_last[16:8] : y0[16:8], mr_yd0 ? y1_last[7:0] : y0[7:0]};
	wire [16:0] ys_start = {mr_ys1 ? ys_rev[16:8] : sy0[16:8], mr_ys0 ? ys_rev[7:0] : sy0[7:0]};
	wire row_go = r_st_start1;
`ifdef SYNTH_KINTEX7
	wire d_row_go, s_row_go;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_drg (.I0(row_go), .O(d_row_go));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_srg (.I0(row_go), .O(s_row_go));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire d_row_go, s_row_go;
	assign d_row_go = row_go;
	assign s_row_go = row_go;
`else
	wire d_row_go = row_go;
	wire s_row_go = row_go;
`endif
	wire row_add = r_st_wr && wr_go && sl_row;

	wire [ADDR_W-1:0] dst_row, src_row;
	wire d_idle, s_idle;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) wire ds_idle_a, ds_idle_b;
	(* dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_dsia (.I0(d_idle), .I1(s_idle), .O(ds_idle_a));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_dsib (.I0(d_idle), .I1(s_idle), .O(ds_idle_b));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) wire ds_idle_a, ds_idle_b;
	(* dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_dsia (.I0(d_idle), .I1(s_idle), .O(ds_idle_a));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_dsib (.I0(d_idle), .I1(s_idle), .O(ds_idle_b));
`else
	wire ds_idle_a = d_idle && s_idle;
	wire ds_idle_b = ds_idle_a;
`endif

	/* The first row's address within its tile row: (y mod 4) x 4 pixels, in bytes. */
	wire [ADDR_W-1:0] d_first, s_first;
	wire [1:0] first_co_unused;

	wire [7:0] dt_base = mdt_h ? {4'b0000, y0[1:0], 2'b00} : 8'd0;
	wire [7:0] d_in_tile_8_comb = (psl_leg0 == 3'd4) ? {dt_base[3:0], 4'b0000} :
	                              (psl_leg0 == 3'd3) ? {dt_base[4:0], 3'b000}  :
	                              (psl_leg0 == 3'd2) ? {dt_base[5:0], 2'b00}   :
	                              (psl_leg0 == 3'd1) ? {dt_base[6:0], 1'b0}    : dt_base;

	/* Direct 2-bit source tile y0[1:0] computation + flattened psl shift (<= 7 cmos3 in S_CLIP). */
	wire [1:0] c_dy_2b = y0[1:0] ^ dy[1:0];
	wire c_dy_bo0 = ~y0[0] & dy[0];
	wire [1:0] c_dy_lo2 = {y0[1] ^ dy[1] ^ c_dy_bo0, c_dy_2b[0]};
`ifdef SYNTH_KINTEX7
	wire [1:0] c_sy0_lo2;
	wire sy0_2b_co;
	am_add #(.W(2)) a_sy0_2b (.a(s_src_xy[17:16]), .b(c_dy_lo2), .ci(1'b0), .s(c_sy0_lo2), .co(sy0_2b_co));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] c_sy0_lo2;
	wire sy0_2b_co;
	am_add #(.W(2)) a_sy0_2b (.a(s_src_xy[17:16]), .b(c_dy_lo2), .ci(1'b0), .s(c_sy0_lo2), .co(sy0_2b_co));
`else
	wire sy0_2b_co = 1'b0;
	wire [1:0] c_sy0_lo2 = {
		s_src_xy[17] ^ c_dy_lo2[1] ^ (s_src_xy[16] & c_dy_lo2[0]),
		s_src_xy[16] ^ c_dy_lo2[0]
	};
`endif
`ifdef SYNTH_KINTEX7
	wire [7:0] st_base_clip = mst_h ? {4'b0000, c_sy0_lo2, 2'b00} : 8'd0;
	wire [7:0] s_in_tile_8_comb = (psl_leg1 == 3'd4) ? {st_base_clip[3:0], 4'b0000} :
	                              (psl_leg1 == 3'd3) ? {st_base_clip[4:0], 3'b000}  :
	                              (psl_leg1 == 3'd2) ? {st_base_clip[5:0], 2'b00}   :
	                              (psl_leg1 == 3'd1) ? {st_base_clip[6:0], 1'b0}    : st_base_clip;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] st_base_clip = mst_h ? {4'b0000, c_sy0_lo2, 2'b00} : 8'd0;
	wire [7:0] s_in_tile_8_comb = (psl_leg1 == 3'd4) ? {st_base_clip[3:0], 4'b0000} :
	                              (psl_leg1 == 3'd3) ? {st_base_clip[4:0], 3'b000}  :
	                              (psl_leg1 == 3'd2) ? {st_base_clip[5:0], 2'b00}   :
	                              (psl_leg1 == 3'd1) ? {st_base_clip[6:0], 1'b0}    : st_base_clip;
`else
	wire [1:0] s_it_2 = mst_h ? c_sy0_lo2 : 2'd0;
	wire [4:0] psl1_oh = {psl_leg1 == 3'd4, psl_leg1 == 3'd3, psl_leg1 == 3'd2, psl_leg1 == 3'd1, psl_leg1 == 3'd0};
	wire [7:0] s_in_tile_8_comb = {
		(psl1_oh[4] & s_it_2[1]),
		(psl1_oh[4] & s_it_2[0]) | (psl1_oh[3] & s_it_2[1]),
		(psl1_oh[3] & s_it_2[0]) | (psl1_oh[2] & s_it_2[1]),
		(psl1_oh[2] & s_it_2[0]) | (psl1_oh[1] & s_it_2[1]),
		(psl1_oh[1] & s_it_2[0]) | (psl1_oh[0] & s_it_2[1]),
		(psl1_oh[0] & s_it_2[0]),
		2'b00
	};
`endif
	reg [7:0] r_d_in_tile_8, r_s_in_tile_8;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_d_in_tile_8 <= 8'd0;
			r_s_in_tile_8 <= 8'd0;
		end else if (cl_tile8) begin
			r_d_in_tile_8 <= d_in_tile_8_comb;
			r_s_in_tile_8 <= s_in_tile_8_comb;
		end
	end
	wire [7:0] d_in_tile_8 = r_d_in_tile_8;
	wire [7:0] s_in_tile_8 = r_s_in_tile_8;

	wire [63:0] r_dbase_ext = {32'd0, r_dbase};
	wire [63:0] r_sbase_ext = {32'd0, r_sbase};
`ifdef SYNTH_KINTEX7
	wire d_first_co, s_first_co, df_h0_c, df_h1_c, sf_h0_c, sf_h1_c;
	wire [ADDR_W-17:0] df_hi0, df_hi1, sf_hi0, sf_hi1;
	am_add #(.W(16))        a_df_lo  (.a(r_dbase_ext[15:0]),        .b({8'd0, d_in_tile_8}),  .ci(1'b0), .s(d_first[15:0]), .co(d_first_co));
	am_add #(.W(ADDR_W-16)) a_df_hi0 (.a(r_dbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b0), .s(df_hi0),        .co(df_h0_c));
	am_add #(.W(ADDR_W-16)) a_df_hi1 (.a(r_dbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b1), .s(df_hi1),        .co(df_h1_c));
	assign d_first[ADDR_W-1:16] = d_first_co ? df_hi1 : df_hi0;
	assign first_co_unused[0] = d_first_co ? df_h1_c : df_h0_c;

	am_add #(.W(16))        a_sf_lo  (.a(r_sbase_ext[15:0]),        .b({8'd0, s_in_tile_8}),  .ci(1'b0), .s(s_first[15:0]), .co(s_first_co));
	am_add #(.W(ADDR_W-16)) a_sf_hi0 (.a(r_sbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b0), .s(sf_hi0),        .co(sf_h0_c));
	am_add #(.W(ADDR_W-16)) a_sf_hi1 (.a(r_sbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b1), .s(sf_hi1),        .co(sf_h1_c));
	assign s_first[ADDR_W-1:16] = s_first_co ? sf_hi1 : sf_hi0;
	assign first_co_unused[1] = s_first_co ? sf_h1_c : sf_h0_c;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire d_first_co, s_first_co, df_h0_c, df_h1_c, sf_h0_c, sf_h1_c;
	wire [ADDR_W-17:0] df_hi0, df_hi1, sf_hi0, sf_hi1;
	am_add #(.W(16))        a_df_lo  (.a(r_dbase_ext[15:0]),        .b({8'd0, d_in_tile_8}),  .ci(1'b0), .s(d_first[15:0]), .co(d_first_co));
	am_add #(.W(ADDR_W-16)) a_df_hi0 (.a(r_dbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b0), .s(df_hi0),        .co(df_h0_c));
	am_add #(.W(ADDR_W-16)) a_df_hi1 (.a(r_dbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b1), .s(df_hi1),        .co(df_h1_c));
	assign d_first[ADDR_W-1:16] = d_first_co ? df_hi1 : df_hi0;
	assign first_co_unused[0] = d_first_co ? df_h1_c : df_h0_c;

	am_add #(.W(16))        a_sf_lo  (.a(r_sbase_ext[15:0]),        .b({8'd0, s_in_tile_8}),  .ci(1'b0), .s(s_first[15:0]), .co(s_first_co));
	am_add #(.W(ADDR_W-16)) a_sf_hi0 (.a(r_sbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b0), .s(sf_hi0),        .co(sf_h0_c));
	am_add #(.W(ADDR_W-16)) a_sf_hi1 (.a(r_sbase_ext[ADDR_W-1:16]), .b({(ADDR_W-16){1'b0}}), .ci(1'b1), .s(sf_hi1),        .co(sf_h1_c));
	assign s_first[ADDR_W-1:16] = s_first_co ? sf_hi1 : sf_hi0;
	assign first_co_unused[1] = s_first_co ? sf_h1_c : sf_h0_c;
`else
	am_add #(.W(ADDR_W)) a_df (.a(r_dbase_ext[ADDR_W-1:0]), .b({{(ADDR_W-8){1'b0}}, d_in_tile_8}), .ci(1'b0), .s(d_first), .co(first_co_unused[0]));
	am_add #(.W(ADDR_W)) a_sf (.a(r_sbase_ext[ADDR_W-1:0]), .b({{(ADDR_W-8){1'b0}}, s_in_tile_8}), .ci(1'b0), .s(s_first), .co(first_co_unused[1]));
`endif

	reg [ADDR_W-1:0] r_dacc_step, r_sacc_step;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_dacc_step <= {ADDR_W{1'b0}};
			r_sacc_step <= {ADDR_W{1'b0}};
		end else begin
			r_dacc_step <= {mr_dacc_hi ? r_dst_neg[ADDR_W-1:16] : dst_step[ADDR_W-1:16], mr_dacc_m0 ? r_dst_neg[15:8] : dst_step[15:8], mr_dacc_lo ? r_dst_neg[7:0] : dst_step[7:0]};
`ifdef SYNTH_KINTEX7
			r_sacc_step <= {mr_sacc_h1 ? r_src_neg[ADDR_W-1:24] : src_step[ADDR_W-1:24], mr_sacc_h0 ? r_src_neg[23:16] : src_step[23:16], mr_sacc_m0 ? r_src_neg[15:8] : src_step[15:8], mr_sacc_lo ? r_src_neg[7:0] : src_step[7:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
			r_sacc_step <= {mr_sacc_h1 ? r_src_neg[ADDR_W-1:24] : src_step[ADDR_W-1:24], mr_sacc_h0 ? r_src_neg[23:16] : src_step[23:16], mr_sacc_m0 ? r_src_neg[15:8] : src_step[15:8], mr_sacc_lo ? r_src_neg[7:0] : src_step[7:0]};
`else
			r_sacc_step <= {mr_sacc_hi ? r_src_neg[ADDR_W-1:16] : src_step[ADDR_W-1:16], mr_sacc_m0 ? r_src_neg[15:8] : src_step[15:8], mr_sacc_lo ? r_src_neg[7:0] : src_step[7:0]};
`endif
		end
	end

	atg_blt_acc #(.W(ADDR_W)) dacc (
		.clk(clk), .rst_n(rst_n), .go(d_row_go),
		.y(r_yd_start), .s(ext_dstride),
		.b(r_d_first),
		.add(row_add), .d(r_dacc_step), .r(dst_row), .idle(d_idle)
	);
	atg_blt_acc #(.W(ADDR_W)) sacc (
		.clk(clk), .rst_n(rst_n), .go(s_row_go),
		.y(r_ys_start), .s(ext_sstride),
		.b(r_s_first),
		.add(row_add), .d(r_sacc_step), .r(src_row), .idle(s_idle)
	);

	/*
	 * The FSM's sums, each for the state that takes it: the rectangle's
	 * size and source origin (S_CLIP), the rows' first pixels (S_START),
	 * the pixel's addresses (S_SA), the segment's bytes (S_SB), its first
	 * bytes (S_SC), and the next pixel, row and the pixels left (S_WR).
	 */
	wire [16:0] c_width, c_rows, c_dx, c_dy, c_sx0, c_sy0, x1_last, s_end, s_last;
	wire [ADDR_W-1:0] d_px, s_px, d_xb, s_xb, d_addr, s_addr, d_lo, s_lo;
	wire [US:0] seg_bytes;
	wire [1:0] ydl_inc, ysl_inc;
	wire [16:0] rows_dec, px_next, spx_next, rem_next;
	wire [23:0] fsm_co_unused;
	assign fsm_co_unused[23:20] = 4'd0;
	assign fsm_co_unused[7] = 1'b0;
	assign fsm_co_unused[3:2] = 2'b00;

	/* 5+6+6 parallel carry-select 17-bit S_CLIP / legacy arithmetic (cuts ~19->9 cmos3). */
	wire [16:0] unused_x_d;

`ifdef SYNTH_KINTEX7
	wire cw_b0, cw_b1_0, cw_b1_1, cw_b2_0, cw_b2_1;
	wire [5:0] cw_s1_0, cw_s1_1, cw_s2_0, cw_s2_1;
	am_sub #(.W(5)) s_w_0  (.a(x1[4:0]),   .b(x0[4:0]),   .bi(1'b0), .d(c_width[4:0]), .bo(cw_b0));
	am_sub #(.W(6)) s_w_10 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b0), .d(cw_s1_0), .bo(cw_b1_0));
	am_sub #(.W(6)) s_w_11 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b1), .d(cw_s1_1), .bo(cw_b1_1));
	am_sub #(.W(6)) s_w_20 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b0), .d(cw_s2_0), .bo(cw_b2_0));
	am_sub #(.W(6)) s_w_21 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b1), .d(cw_s2_1), .bo(cw_b2_1));
	wire cw_b1 = cw_b1_0 | (cw_b1_1 & cw_b0);
	assign c_width[10:5]  = cw_b0 ? cw_s1_1 : cw_s1_0;
	assign c_width[16:11] = cw_b1 ? cw_s2_1 : cw_s2_0;
	assign fsm_co_unused[0] = (cw_b1 ? cw_b2_1 : cw_b2_0) & cw_b2_0;

	wire xe_b0, xe_b1_0, xe_b1_1, xe_b2_0, xe_b2_1;
	wire [5:0] xe_s1_0, xe_s1_1, xe_s2_0, xe_s2_1;
	am_sub #(.W(5)) s_xempty_0  (.a(x1[4:0]),   .b(x0[4:0]),   .bi(1'b1), .d(unused_x_d[4:0]), .bo(xe_b0));
	am_sub #(.W(6)) s_xempty_10 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b0), .d(xe_s1_0), .bo(xe_b1_0));
	am_sub #(.W(6)) s_xempty_11 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b1), .d(xe_s1_1), .bo(xe_b1_1));
	am_sub #(.W(6)) s_xempty_20 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b0), .d(xe_s2_0), .bo(xe_b2_0));
	am_sub #(.W(6)) s_xempty_21 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b1), .d(xe_s2_1), .bo(xe_b2_1));
	wire xe_b1 = xe_b1_0 | (xe_b1_1 & xe_b0);
	assign unused_x_d[10:5]  = xe_b0 ? xe_s1_1 : xe_s1_0;
	assign unused_x_d[16:11] = xe_b1 ? xe_s2_1 : xe_s2_0;
	assign fsm_co_unused[19] = (xe_b1 ? xe_b2_1 : xe_b2_0) & xe_b2_0;

	wire cr_b0, cr_b1_0, cr_b1_1, cr_b2_0, cr_b2_1;
	wire [5:0] cr_s1_0, cr_s1_1, cr_s2_0, cr_s2_1;
	am_sub #(.W(5)) s_rows_0  (.a(y1[4:0]),   .b(y0[4:0]),   .bi(1'b1), .d(c_rows[4:0]), .bo(cr_b0));
	am_sub #(.W(6)) s_rows_10 (.a(y1[10:5]),  .b(y0[10:5]),  .bi(1'b0), .d(cr_s1_0), .bo(cr_b1_0));
	am_sub #(.W(6)) s_rows_11 (.a(y1[10:5]),  .b(y0[10:5]),  .bi(1'b1), .d(cr_s1_1), .bo(cr_b1_1));
	am_sub #(.W(6)) s_rows_20 (.a(y1[16:11]), .b(y0[16:11]), .bi(1'b0), .d(cr_s2_0), .bo(cr_b2_0));
	am_sub #(.W(6)) s_rows_21 (.a(y1[16:11]), .b(y0[16:11]), .bi(1'b1), .d(cr_s2_1), .bo(cr_b2_1));
	wire cr_b1 = cr_b1_0 | (cr_b1_1 & cr_b0);
	assign c_rows[10:5]  = cr_b0 ? cr_s1_1 : cr_s1_0;
	assign c_rows[16:11] = cr_b1 ? cr_s2_1 : cr_s2_0;
	assign fsm_co_unused[1] = (cr_b1 ? cr_b2_1 : cr_b2_0) & cr_b2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire cw_b0, cw_b1_0, cw_b1_1, cw_b2_0, cw_b2_1;
	wire [5:0] cw_s1_0, cw_s1_1, cw_s2_0, cw_s2_1;
	am_sub #(.W(5)) s_w_0  (.a(x1[4:0]),   .b(x0[4:0]),   .bi(1'b0), .d(c_width[4:0]), .bo(cw_b0));
	am_sub #(.W(6)) s_w_10 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b0), .d(cw_s1_0), .bo(cw_b1_0));
	am_sub #(.W(6)) s_w_11 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b1), .d(cw_s1_1), .bo(cw_b1_1));
	am_sub #(.W(6)) s_w_20 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b0), .d(cw_s2_0), .bo(cw_b2_0));
	am_sub #(.W(6)) s_w_21 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b1), .d(cw_s2_1), .bo(cw_b2_1));
	wire cw_b1 = cw_b1_0 | (cw_b1_1 & cw_b0);
	assign c_width[10:5]  = cw_b0 ? cw_s1_1 : cw_s1_0;
	assign c_width[16:11] = cw_b1 ? cw_s2_1 : cw_s2_0;
	assign fsm_co_unused[0] = (cw_b1 ? cw_b2_1 : cw_b2_0) & cw_b2_0;

	wire xe_b0, xe_b1_0, xe_b1_1, xe_b2_0, xe_b2_1;
	wire [5:0] xe_s1_0, xe_s1_1, xe_s2_0, xe_s2_1;
	am_sub #(.W(5)) s_xempty_0  (.a(x1[4:0]),   .b(x0[4:0]),   .bi(1'b1), .d(unused_x_d[4:0]), .bo(xe_b0));
	am_sub #(.W(6)) s_xempty_10 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b0), .d(xe_s1_0), .bo(xe_b1_0));
	am_sub #(.W(6)) s_xempty_11 (.a(x1[10:5]),  .b(x0[10:5]),  .bi(1'b1), .d(xe_s1_1), .bo(xe_b1_1));
	am_sub #(.W(6)) s_xempty_20 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b0), .d(xe_s2_0), .bo(xe_b2_0));
	am_sub #(.W(6)) s_xempty_21 (.a(x1[16:11]), .b(x0[16:11]), .bi(1'b1), .d(xe_s2_1), .bo(xe_b2_1));
	wire xe_b1 = xe_b1_0 | (xe_b1_1 & xe_b0);
	assign unused_x_d[10:5]  = xe_b0 ? xe_s1_1 : xe_s1_0;
	assign unused_x_d[16:11] = xe_b1 ? xe_s2_1 : xe_s2_0;
	assign fsm_co_unused[19] = (xe_b1 ? xe_b2_1 : xe_b2_0) & xe_b2_0;

	wire cr_b0, cr_b1_0, cr_b1_1, cr_b2_0, cr_b2_1;
	wire [5:0] cr_s1_0, cr_s1_1, cr_s2_0, cr_s2_1;
	am_sub #(.W(5)) s_rows_0  (.a(y1[4:0]),   .b(y0[4:0]),   .bi(1'b1), .d(c_rows[4:0]), .bo(cr_b0));
	am_sub #(.W(6)) s_rows_10 (.a(y1[10:5]),  .b(y0[10:5]),  .bi(1'b0), .d(cr_s1_0), .bo(cr_b1_0));
	am_sub #(.W(6)) s_rows_11 (.a(y1[10:5]),  .b(y0[10:5]),  .bi(1'b1), .d(cr_s1_1), .bo(cr_b1_1));
	am_sub #(.W(6)) s_rows_20 (.a(y1[16:11]), .b(y0[16:11]), .bi(1'b0), .d(cr_s2_0), .bo(cr_b2_0));
	am_sub #(.W(6)) s_rows_21 (.a(y1[16:11]), .b(y0[16:11]), .bi(1'b1), .d(cr_s2_1), .bo(cr_b2_1));
	wire cr_b1 = cr_b1_0 | (cr_b1_1 & cr_b0);
	assign c_rows[10:5]  = cr_b0 ? cr_s1_1 : cr_s1_0;
	assign c_rows[16:11] = cr_b1 ? cr_s2_1 : cr_s2_0;
	assign fsm_co_unused[1] = (cr_b1 ? cr_b2_1 : cr_b2_0) & cr_b2_0;
`else
	wire cw_b0, cw_b1_0, cw_b1_1, cw_b2_0, cw_b2_1, cw_b3_0, cw_b3_1;
	wire [3:0] cw_s1_0, cw_s1_1, cw_s2_0, cw_s2_1;
	wire [4:0] cw_s3_0, cw_s3_1;
	am_sub #(.W(4)) s_w_0  (.a(x1[3:0]),   .b(x0[3:0]),   .bi(1'b0), .d(c_width[3:0]), .bo(cw_b0));
	am_sub #(.W(4)) s_w_10 (.a(x1[7:4]),   .b(x0[7:4]),   .bi(1'b0), .d(cw_s1_0),      .bo(cw_b1_0));
	am_sub #(.W(4)) s_w_11 (.a(x1[7:4]),   .b(x0[7:4]),   .bi(1'b1), .d(cw_s1_1),      .bo(cw_b1_1));
	am_sub #(.W(4)) s_w_20 (.a(x1[11:8]),  .b(x0[11:8]),  .bi(1'b0), .d(cw_s2_0),      .bo(cw_b2_0));
	am_sub #(.W(4)) s_w_21 (.a(x1[11:8]),  .b(x0[11:8]),  .bi(1'b1), .d(cw_s2_1),      .bo(cw_b2_1));
	am_sub #(.W(5)) s_w_30 (.a(x1[16:12]), .b(x0[16:12]), .bi(1'b0), .d(cw_s3_0),      .bo(cw_b3_0));
	am_sub #(.W(5)) s_w_31 (.a(x1[16:12]), .b(x0[16:12]), .bi(1'b1), .d(cw_s3_1),      .bo(cw_b3_1));
	wire cw_b1 = cw_b1_0 | (cw_b1_1 & cw_b0);
	wire cw_b2 = cw_b2_0 | (cw_b2_1 & cw_b1);
	assign c_width[7:4]   = cw_b0 ? cw_s1_1 : cw_s1_0;
	assign c_width[11:8]  = cw_b1 ? cw_s2_1 : cw_s2_0;
	assign c_width[16:12] = cw_b2 ? cw_s3_1 : cw_s3_0;
	assign fsm_co_unused[0] = cw_b2 ? cw_b3_1 : cw_b3_0;

	wire xe_b0, xe_b1_0, xe_b1_1, xe_b2_0, xe_b2_1, xe_b3_0, xe_b3_1;
	wire [3:0] xe_s1_0, xe_s1_1, xe_s2_0, xe_s2_1;
	wire [4:0] xe_s3_0, xe_s3_1;
	am_sub #(.W(4)) s_xempty_0  (.a(x1[3:0]),   .b(x0[3:0]),   .bi(1'b1), .d(unused_x_d[3:0]), .bo(xe_b0));
	am_sub #(.W(4)) s_xempty_10 (.a(x1[7:4]),   .b(x0[7:4]),   .bi(1'b0), .d(xe_s1_0),         .bo(xe_b1_0));
	am_sub #(.W(4)) s_xempty_11 (.a(x1[7:4]),   .b(x0[7:4]),   .bi(1'b1), .d(xe_s1_1),         .bo(xe_b1_1));
	am_sub #(.W(4)) s_xempty_20 (.a(x1[11:8]),  .b(x0[11:8]),  .bi(1'b0), .d(xe_s2_0),         .bo(xe_b2_0));
	am_sub #(.W(4)) s_xempty_21 (.a(x1[11:8]),  .b(x0[11:8]),  .bi(1'b1), .d(xe_s2_1),         .bo(xe_b2_1));
	am_sub #(.W(5)) s_xempty_30 (.a(x1[16:12]), .b(x0[16:12]), .bi(1'b0), .d(xe_s3_0),         .bo(xe_b3_0));
	am_sub #(.W(5)) s_xempty_31 (.a(x1[16:12]), .b(x0[16:12]), .bi(1'b1), .d(xe_s3_1),         .bo(xe_b3_1));
	wire xe_b1 = xe_b1_0 | (xe_b1_1 & xe_b0);
	wire xe_b2 = xe_b2_0 | (xe_b2_1 & xe_b1);
	assign unused_x_d[7:4]   = xe_b0 ? xe_s1_1 : xe_s1_0;
	assign unused_x_d[11:8]  = xe_b1 ? xe_s2_1 : xe_s2_0;
	assign unused_x_d[16:12] = xe_b2 ? xe_s3_1 : xe_s3_0;
	assign fsm_co_unused[19] = xe_b2 ? xe_b3_1 : xe_b3_0;

	wire cr_b0, cr_b1_0, cr_b1_1, cr_b2_0, cr_b2_1, cr_b3_0, cr_b3_1;
	wire [3:0] cr_s1_0, cr_s1_1, cr_s2_0, cr_s2_1;
	wire [4:0] cr_s3_0, cr_s3_1;
	am_sub #(.W(4)) s_rows_0  (.a(y1[3:0]),   .b(y0[3:0]),   .bi(1'b1), .d(c_rows[3:0]), .bo(cr_b0));
	am_sub #(.W(4)) s_rows_10 (.a(y1[7:4]),   .b(y0[7:4]),   .bi(1'b0), .d(cr_s1_0),     .bo(cr_b1_0));
	am_sub #(.W(4)) s_rows_11 (.a(y1[7:4]),   .b(y0[7:4]),   .bi(1'b1), .d(cr_s1_1),     .bo(cr_b1_1));
	am_sub #(.W(4)) s_rows_20 (.a(y1[11:8]),  .b(y0[11:8]),  .bi(1'b0), .d(cr_s2_0),     .bo(cr_b2_0));
	am_sub #(.W(4)) s_rows_21 (.a(y1[11:8]),  .b(y0[11:8]),  .bi(1'b1), .d(cr_s2_1),     .bo(cr_b2_1));
	am_sub #(.W(5)) s_rows_30 (.a(y1[16:12]), .b(y0[16:12]), .bi(1'b0), .d(cr_s3_0),     .bo(cr_b3_0));
	am_sub #(.W(5)) s_rows_31 (.a(y1[16:12]), .b(y0[16:12]), .bi(1'b1), .d(cr_s3_1),     .bo(cr_b3_1));
	wire cr_b1 = cr_b1_0 | (cr_b1_1 & cr_b0);
	wire cr_b2 = cr_b2_0 | (cr_b2_1 & cr_b1);
	assign c_rows[7:4]   = cr_b0 ? cr_s1_1 : cr_s1_0;
	assign c_rows[11:8]  = cr_b1 ? cr_s2_1 : cr_s2_0;
	assign c_rows[16:12] = cr_b2 ? cr_s3_1 : cr_s3_0;
	assign fsm_co_unused[1] = cr_b2 ? cr_b3_1 : cr_b3_0;
`endif

`ifdef SYNTH_KINTEX7
	wire cx_empty = (x1 <= x0);
	wire cy_empty = (y1 <= y0);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire cx_empty = (x1 <= x0);
	wire cy_empty = (y1 <= y0);
`else
	/* Fast 4+4+4+5 chunked prefix comparators (<= 7 cmos3) for cx_empty / cy_empty. */
	wire cx_lt3 = (x1[16:12] <  x0[16:12]), cx_eq3 = (x1[16:12] == x0[16:12]);
	wire cx_lt2 = (x1[11:8]  <  x0[11:8]),  cx_eq2 = (x1[11:8]  == x0[11:8]);
	wire cx_lt1 = (x1[7:4]   <  x0[7:4]),   cx_eq1 = (x1[7:4]   == x0[7:4]);
	wire cx_le0 = (x1[3:0]   <= x0[3:0]);
	wire cx_hi_lt = cx_lt3 || (cx_eq3 && cx_lt2);
	wire cx_hi_eq = cx_eq3 && cx_eq2;
	wire cx_lo_le = cx_lt1 || (cx_eq1 && cx_le0);
	wire cx_empty = cx_hi_lt || (cx_hi_eq && cx_lo_le);

	wire cy_lt3 = (y1[16:12] <  y0[16:12]), cy_eq3 = (y1[16:12] == y0[16:12]);
	wire cy_lt2 = (y1[11:8]  <  y0[11:8]),  cy_eq2 = (y1[11:8]  == y0[11:8]);
	wire cy_lt1 = (y1[7:4]   <  y0[7:4]),   cy_eq1 = (y1[7:4]   == y0[7:4]);
	wire cy_le0 = (y1[3:0]   <= y0[3:0]);
	wire cy_hi_lt = cy_lt3 || (cy_eq3 && cy_lt2);
	wire cy_hi_eq = cy_eq3 && cy_eq2;
	wire cy_lo_le = cy_lt1 || (cy_eq1 && cy_le0);
	wire cy_empty = cy_hi_lt || (cy_hi_eq && cy_lo_le);
`endif
	assign empty = cx_empty || cy_empty;
	assign c_dx = 17'd0;
	assign c_dy = 17'd0;

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [16:0] src_m_dx_w, src_m_dy_w;
	wire [1:0] smd_bo_unused;
	am_sub #(.W(17)) s_src_m_dx (.a({1'b0, s_src_xy[15:0]}),  .b(dx), .bi(1'b0), .d(src_m_dx_w), .bo(smd_bo_unused[0]));
	am_sub #(.W(17)) s_src_m_dy (.a({1'b0, s_src_xy[31:16]}), .b(dy), .bi(1'b0), .d(src_m_dy_w), .bo(smd_bo_unused[1]));
`endif

	/* 3:2 CSA + 5+6+6 (FPGA) / 4+4+4+5 (ASIC) carry-select for c_sx0 and c_sy0. */
	wire [16:0] sx_cs_a = {1'b0, s_src_xy[15:0]};
	wire [16:0] sx_cs_b = x0;
	wire [16:0] sx_cs_c = ~dx;
	wire [16:0] sx_cs_s = sx_cs_a ^ sx_cs_b ^ sx_cs_c;
	wire [16:0] sx_cs_k = {((sx_cs_a[15:0] & sx_cs_b[15:0]) | (sx_cs_a[15:0] & sx_cs_c[15:0]) | (sx_cs_b[15:0] & sx_cs_c[15:0])), 1'b1};
	wire [16:0] sy_cs_a = {1'b0, s_src_xy[31:16]};
	wire [16:0] sy_cs_b = y0;
	wire [16:0] sy_cs_c = ~dy;
	wire [16:0] sy_cs_s = sy_cs_a ^ sy_cs_b ^ sy_cs_c;
	wire [16:0] sy_cs_k = {((sy_cs_a[15:0] & sy_cs_b[15:0]) | (sy_cs_a[15:0] & sy_cs_c[15:0]) | (sy_cs_b[15:0] & sy_cs_c[15:0])), 1'b1};
`ifdef SYNTH_KINTEX7
	wire sx0_c0, sx0_c1_0, sx0_c1_1, sx0_c2_0, sx0_c2_1;
	wire [5:0] sx0_s1_0, sx0_s1_1, sx0_s2_0, sx0_s2_1;
	am_add #(.W(5)) a_sx0_0  (.a(sx_cs_s[4:0]),   .b(sx_cs_k[4:0]),   .ci(1'b0), .s(c_sx0[4:0]), .co(sx0_c0));
	am_add #(.W(6)) a_sx0_10 (.a(sx_cs_s[10:5]),  .b(sx_cs_k[10:5]),  .ci(1'b0), .s(sx0_s1_0), .co(sx0_c1_0));
	am_add #(.W(6)) a_sx0_11 (.a(sx_cs_s[10:5]),  .b(sx_cs_k[10:5]),  .ci(1'b1), .s(sx0_s1_1), .co(sx0_c1_1));
	am_add #(.W(6)) a_sx0_20 (.a(sx_cs_s[16:11]), .b(sx_cs_k[16:11]), .ci(1'b0), .s(sx0_s2_0), .co(sx0_c2_0));
	am_add #(.W(6)) a_sx0_21 (.a(sx_cs_s[16:11]), .b(sx_cs_k[16:11]), .ci(1'b1), .s(sx0_s2_1), .co(sx0_c2_1));
	wire sx0_c1 = sx0_c1_0 | (sx0_c1_1 & sx0_c0);
	assign c_sx0[10:5]  = sx0_c0 ? sx0_s1_1 : sx0_s1_0;
	assign c_sx0[16:11] = sx0_c1 ? sx0_s2_1 : sx0_s2_0;
	assign fsm_co_unused[4] = (sx0_c1 ? sx0_c2_1 : sx0_c2_0) & sx0_c2_0;

	wire sy0_c0, sy0_c1_0, sy0_c1_1, sy0_c2_0, sy0_c2_1;
	wire [5:0] sy0_s1_0, sy0_s1_1, sy0_s2_0, sy0_s2_1;
	am_add #(.W(5)) a_sy0_0  (.a(sy_cs_s[4:0]),   .b(sy_cs_k[4:0]),   .ci(1'b0), .s(c_sy0[4:0]), .co(sy0_c0));
	am_add #(.W(6)) a_sy0_10 (.a(sy_cs_s[10:5]),  .b(sy_cs_k[10:5]),  .ci(1'b0), .s(sy0_s1_0), .co(sy0_c1_0));
	am_add #(.W(6)) a_sy0_11 (.a(sy_cs_s[10:5]),  .b(sy_cs_k[10:5]),  .ci(1'b1), .s(sy0_s1_1), .co(sy0_c1_1));
	am_add #(.W(6)) a_sy0_20 (.a(sy_cs_s[16:11]), .b(sy_cs_k[16:11]), .ci(1'b0), .s(sy0_s2_0), .co(sy0_c2_0));
	am_add #(.W(6)) a_sy0_21 (.a(sy_cs_s[16:11]), .b(sy_cs_k[16:11]), .ci(1'b1), .s(sy0_s2_1), .co(sy0_c2_1));
	wire sy0_c1 = sy0_c1_0 | (sy0_c1_1 & sy0_c0);
	assign c_sy0[10:5]  = sy0_c0 ? sy0_s1_1 : sy0_s1_0;
	assign c_sy0[16:11] = sy0_c1 ? sy0_s2_1 : sy0_s2_0;
	assign fsm_co_unused[5] = (sy0_c1 ? sy0_c2_1 : sy0_c2_0) & sy0_c2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire sx0_c0, sx0_c1_0, sx0_c1_1, sx0_c2_0, sx0_c2_1;
	wire [5:0] sx0_s1_0, sx0_s1_1, sx0_s2_0, sx0_s2_1;
	am_add #(.W(5)) a_sx0_0  (.a(sx_cs_s[4:0]),   .b(sx_cs_k[4:0]),   .ci(1'b0), .s(c_sx0[4:0]), .co(sx0_c0));
	am_add #(.W(6)) a_sx0_10 (.a(sx_cs_s[10:5]),  .b(sx_cs_k[10:5]),  .ci(1'b0), .s(sx0_s1_0), .co(sx0_c1_0));
	am_add #(.W(6)) a_sx0_11 (.a(sx_cs_s[10:5]),  .b(sx_cs_k[10:5]),  .ci(1'b1), .s(sx0_s1_1), .co(sx0_c1_1));
	am_add #(.W(6)) a_sx0_20 (.a(sx_cs_s[16:11]), .b(sx_cs_k[16:11]), .ci(1'b0), .s(sx0_s2_0), .co(sx0_c2_0));
	am_add #(.W(6)) a_sx0_21 (.a(sx_cs_s[16:11]), .b(sx_cs_k[16:11]), .ci(1'b1), .s(sx0_s2_1), .co(sx0_c2_1));
	wire sx0_c1 = sx0_c1_0 | (sx0_c1_1 & sx0_c0);
	assign c_sx0[10:5]  = sx0_c0 ? sx0_s1_1 : sx0_s1_0;
	assign c_sx0[16:11] = sx0_c1 ? sx0_s2_1 : sx0_s2_0;
	assign fsm_co_unused[4] = (sx0_c1 ? sx0_c2_1 : sx0_c2_0) & sx0_c2_0;

	wire sy0_c0, sy0_c1_0, sy0_c1_1, sy0_c2_0, sy0_c2_1;
	wire [5:0] sy0_s1_0, sy0_s1_1, sy0_s2_0, sy0_s2_1;
	am_add #(.W(5)) a_sy0_0  (.a(sy_cs_s[4:0]),   .b(sy_cs_k[4:0]),   .ci(1'b0), .s(c_sy0[4:0]), .co(sy0_c0));
	am_add #(.W(6)) a_sy0_10 (.a(sy_cs_s[10:5]),  .b(sy_cs_k[10:5]),  .ci(1'b0), .s(sy0_s1_0), .co(sy0_c1_0));
	am_add #(.W(6)) a_sy0_11 (.a(sy_cs_s[10:5]),  .b(sy_cs_k[10:5]),  .ci(1'b1), .s(sy0_s1_1), .co(sy0_c1_1));
	am_add #(.W(6)) a_sy0_20 (.a(sy_cs_s[16:11]), .b(sy_cs_k[16:11]), .ci(1'b0), .s(sy0_s2_0), .co(sy0_c2_0));
	am_add #(.W(6)) a_sy0_21 (.a(sy_cs_s[16:11]), .b(sy_cs_k[16:11]), .ci(1'b1), .s(sy0_s2_1), .co(sy0_c2_1));
	wire sy0_c1 = sy0_c1_0 | (sy0_c1_1 & sy0_c0);
	assign c_sy0[10:5]  = sy0_c0 ? sy0_s1_1 : sy0_s1_0;
	assign c_sy0[16:11] = sy0_c1 ? sy0_s2_1 : sy0_s2_0;
	assign fsm_co_unused[5] = (sy0_c1 ? sy0_c2_1 : sy0_c2_0) & sy0_c2_0;
`else
	am_add #(.W(17)) a_sx0 (.a(r_src_m_dx), .b(x0), .ci(1'b0), .s(c_sx0), .co(fsm_co_unused[4]));
	am_add #(.W(17)) a_sy0 (.a(r_src_m_dy), .b(y0), .ci(1'b0), .s(c_sy0), .co(fsm_co_unused[5]));
	wire csa_src_unused = &{1'b0, sx_cs_s, sx_cs_k, sy_cs_s, sy_cs_k};
`endif

`ifdef SYNTH_KINTEX7
	wire x1l_b0, x1l_b1_0, x1l_b1_1, x1l_b2_0, x1l_b2_1;
	wire [5:0] x1l_s1_0, x1l_s1_1, x1l_s2_0, x1l_s2_1;
	am_sub #(.W(5)) s_x1_0  (.a(x1[4:0]),   .b(5'd0), .bi(1'b1), .d(x1_last[4:0]), .bo(x1l_b0));
	am_sub #(.W(6)) s_x1_10 (.a(x1[10:5]),  .b(6'd0), .bi(1'b0), .d(x1l_s1_0), .bo(x1l_b1_0));
	am_sub #(.W(6)) s_x1_11 (.a(x1[10:5]),  .b(6'd0), .bi(1'b1), .d(x1l_s1_1), .bo(x1l_b1_1));
	am_sub #(.W(6)) s_x1_20 (.a(x1[16:11]), .b(6'd0), .bi(1'b0), .d(x1l_s2_0), .bo(x1l_b2_0));
	am_sub #(.W(6)) s_x1_21 (.a(x1[16:11]), .b(6'd0), .bi(1'b1), .d(x1l_s2_1), .bo(x1l_b2_1));
	wire x1l_b1 = x1l_b1_0 | (x1l_b1_1 & x1l_b0);
	assign x1_last[10:5]  = x1l_b0 ? x1l_s1_1 : x1l_s1_0;
	assign x1_last[16:11] = x1l_b1 ? x1l_s2_1 : x1l_s2_0;
	assign fsm_co_unused[6] = (x1l_b1 ? x1l_b2_1 : x1l_b2_0) & x1l_b2_0;

	wire sla_c0, sla_c1_0, sla_c1_1, sla_c2_0, sla_c2_1;
	wire [5:0] sla_s1_0, sla_s1_1, sla_s2_0, sla_s2_1;
	am_add #(.W(5)) a_slast_0  (.a(sx0[4:0]),   .b(width_m1[4:0]),   .ci(1'b0), .s(s_last[4:0]), .co(sla_c0));
	am_add #(.W(6)) a_slast_10 (.a(sx0[10:5]),  .b(width_m1[10:5]),  .ci(1'b0), .s(sla_s1_0), .co(sla_c1_0));
	am_add #(.W(6)) a_slast_11 (.a(sx0[10:5]),  .b(width_m1[10:5]),  .ci(1'b1), .s(sla_s1_1), .co(sla_c1_1));
	am_add #(.W(6)) a_slast_20 (.a(sx0[16:11]), .b(width_m1[16:11]), .ci(1'b0), .s(sla_s2_0), .co(sla_c2_0));
	am_add #(.W(6)) a_slast_21 (.a(sx0[16:11]), .b(width_m1[16:11]), .ci(1'b1), .s(sla_s2_1), .co(sla_c2_1));
	wire sla_c1 = sla_c1_0 | (sla_c1_1 & sla_c0);
	assign s_last[10:5]  = sla_c0 ? sla_s1_1 : sla_s1_0;
	assign s_last[16:11] = sla_c1 ? sla_s2_1 : sla_s2_0;
	assign fsm_co_unused[8] = (sla_c1 ? sla_c2_1 : sla_c2_0) & sla_c2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire x1l_b0, x1l_b1_0, x1l_b1_1, x1l_b2_0, x1l_b2_1;
	wire [5:0] x1l_s1_0, x1l_s1_1, x1l_s2_0, x1l_s2_1;
	am_sub #(.W(5)) s_x1_0  (.a(x1[4:0]),   .b(5'd0), .bi(1'b1), .d(x1_last[4:0]), .bo(x1l_b0));
	am_sub #(.W(6)) s_x1_10 (.a(x1[10:5]),  .b(6'd0), .bi(1'b0), .d(x1l_s1_0), .bo(x1l_b1_0));
	am_sub #(.W(6)) s_x1_11 (.a(x1[10:5]),  .b(6'd0), .bi(1'b1), .d(x1l_s1_1), .bo(x1l_b1_1));
	am_sub #(.W(6)) s_x1_20 (.a(x1[16:11]), .b(6'd0), .bi(1'b0), .d(x1l_s2_0), .bo(x1l_b2_0));
	am_sub #(.W(6)) s_x1_21 (.a(x1[16:11]), .b(6'd0), .bi(1'b1), .d(x1l_s2_1), .bo(x1l_b2_1));
	wire x1l_b1 = x1l_b1_0 | (x1l_b1_1 & x1l_b0);
	assign x1_last[10:5]  = x1l_b0 ? x1l_s1_1 : x1l_s1_0;
	assign x1_last[16:11] = x1l_b1 ? x1l_s2_1 : x1l_s2_0;
	assign fsm_co_unused[6] = (x1l_b1 ? x1l_b2_1 : x1l_b2_0) & x1l_b2_0;

	wire sla_c0, sla_c1_0, sla_c1_1, sla_c2_0, sla_c2_1;
	wire [5:0] sla_s1_0, sla_s1_1, sla_s2_0, sla_s2_1;
	am_add #(.W(5)) a_slast_0  (.a(sx0[4:0]),   .b(width_m1[4:0]),   .ci(1'b0), .s(s_last[4:0]), .co(sla_c0));
	am_add #(.W(6)) a_slast_10 (.a(sx0[10:5]),  .b(width_m1[10:5]),  .ci(1'b0), .s(sla_s1_0), .co(sla_c1_0));
	am_add #(.W(6)) a_slast_11 (.a(sx0[10:5]),  .b(width_m1[10:5]),  .ci(1'b1), .s(sla_s1_1), .co(sla_c1_1));
	am_add #(.W(6)) a_slast_20 (.a(sx0[16:11]), .b(width_m1[16:11]), .ci(1'b0), .s(sla_s2_0), .co(sla_c2_0));
	am_add #(.W(6)) a_slast_21 (.a(sx0[16:11]), .b(width_m1[16:11]), .ci(1'b1), .s(sla_s2_1), .co(sla_c2_1));
	wire sla_c1 = sla_c1_0 | (sla_c1_1 & sla_c0);
	assign s_last[10:5]  = sla_c0 ? sla_s1_1 : sla_s1_0;
	assign s_last[16:11] = sla_c1 ? sla_s2_1 : sla_s2_0;
	assign fsm_co_unused[8] = (sla_c1 ? sla_c2_1 : sla_c2_0) & sla_c2_0;
`else
	assign x1_last[0] = ~x1[0];
	genvar g_x1l;
	generate
		for (g_x1l = 1; g_x1l < 17; g_x1l = g_x1l + 1) begin : g_x1l_fast
			assign x1_last[g_x1l] = x1[g_x1l] ^ (~(|x1[g_x1l-1:0]));
		end
	endgenerate
	assign fsm_co_unused[6] = 1'b0;
	am_add #(.W(17)) a_slast (.a(sx0), .b(width_m1), .ci(1'b0), .s(s_last),  .co(fsm_co_unused[8]));
`endif

	wire [ADDR_W-1:0] d_px_hi_w = x_pixels(px, mdt_px_hi);
	wire [ADDR_W-1:0] s_px_hi_w = x_pixels(spx, mst_px_hi);
	assign d_px = x_pixels(px, mdt_px);
	assign s_px = x_pixels(spx, mst_px);
	wire [ADDR_W-1:0] d_xb_lo_w, s_xb_lo_w, d_xb_hi_w, s_xb_hi_w;
	am_shift #(.W(ADDR_W), .SW(3)) h_dx    (.a(d_px),      .amt(psl_dx),  .left(1'b1), .arith(1'b0), .y(d_xb_lo_w));
	am_shift #(.W(ADDR_W), .SW(3)) h_dx_hi (.a(d_px_hi_w), .amt(psl_dx1), .left(1'b1), .arith(1'b0), .y(d_xb_hi_w));
	am_shift #(.W(ADDR_W), .SW(3)) h_sx    (.a(s_px),      .amt(psl_sx),  .left(1'b1), .arith(1'b0), .y(s_xb_lo_w));
	am_shift #(.W(ADDR_W), .SW(3)) h_sx_hi (.a(s_px_hi_w), .amt(psl_sx1), .left(1'b1), .arith(1'b0), .y(s_xb_hi_w));
	assign d_xb = {d_xb_hi_w[ADDR_W-1:16], d_xb_lo_w[15:0]};
	assign s_xb = {s_xb_hi_w[ADDR_W-1:16], s_xb_lo_w[15:0]};

	/* S_SA1 uses FF-registered r_d_xb_leg / r_s_xb_leg (16+16 on Kintex-7 Rule 1, 8+8+8+8 on ASIC). */
`ifdef SYNTH_KINTEX7
	wire da_c0, da_c10, da_c11, sa_c0, sa_c10, sa_c11;
	wire [ADDR_W-17:0] da_s10, da_s11, sa_s10, sa_s11;
	am_add #(.W(16))        a_da_lo  (.a(dst_row[15:0]),        .b(r_d_xb_leg[15:0]),        .ci(1'b0), .s(d_addr[15:0]), .co(da_c0));
	am_add #(.W(ADDR_W-16)) a_da_h0  (.a(dst_row[ADDR_W-1:16]), .b(r_d_xb_leg[ADDR_W-1:16]), .ci(1'b0), .s(da_s10),       .co(da_c10));
	am_add #(.W(ADDR_W-16)) a_da_h1  (.a(dst_row[ADDR_W-1:16]), .b(r_d_xb_leg[ADDR_W-1:16]), .ci(1'b1), .s(da_s11),       .co(da_c11));
	assign d_addr[ADDR_W-1:16] = da_c0 ? da_s11 : da_s10;
	assign fsm_co_unused[9] = (da_c0 ? da_c11 : da_c10) & da_c10;

	am_add #(.W(16))        a_sa_lo  (.a(src_row[15:0]),        .b(r_s_xb_leg[15:0]),        .ci(1'b0), .s(s_addr[15:0]), .co(sa_c0));
	am_add #(.W(ADDR_W-16)) a_sa_h0  (.a(src_row[ADDR_W-1:16]), .b(r_s_xb_leg[ADDR_W-1:16]), .ci(1'b0), .s(sa_s10),       .co(sa_c10));
	am_add #(.W(ADDR_W-16)) a_sa_h1  (.a(src_row[ADDR_W-1:16]), .b(r_s_xb_leg[ADDR_W-1:16]), .ci(1'b1), .s(sa_s11),       .co(sa_c11));
	assign s_addr[ADDR_W-1:16] = sa_c0 ? sa_s11 : sa_s10;
	assign fsm_co_unused[10] = (sa_c0 ? sa_c11 : sa_c10) & sa_c10;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire da_c0, da_c10, da_c11, sa_c0, sa_c10, sa_c11;
	wire [ADDR_W-17:0] da_s10, da_s11, sa_s10, sa_s11;
	am_add #(.W(16))        a_da_lo  (.a(dst_row[15:0]),        .b(r_d_xb_leg[15:0]),        .ci(1'b0), .s(d_addr[15:0]), .co(da_c0));
	am_add #(.W(ADDR_W-16)) a_da_h0  (.a(dst_row[ADDR_W-1:16]), .b(r_d_xb_leg[ADDR_W-1:16]), .ci(1'b0), .s(da_s10),       .co(da_c10));
	am_add #(.W(ADDR_W-16)) a_da_h1  (.a(dst_row[ADDR_W-1:16]), .b(r_d_xb_leg[ADDR_W-1:16]), .ci(1'b1), .s(da_s11),       .co(da_c11));
	assign d_addr[ADDR_W-1:16] = da_c0 ? da_s11 : da_s10;
	assign fsm_co_unused[9] = (da_c0 ? da_c11 : da_c10) & da_c10;

	am_add #(.W(16))        a_sa_lo  (.a(src_row[15:0]),        .b(r_s_xb_leg[15:0]),        .ci(1'b0), .s(s_addr[15:0]), .co(sa_c0));
	am_add #(.W(ADDR_W-16)) a_sa_h0  (.a(src_row[ADDR_W-1:16]), .b(r_s_xb_leg[ADDR_W-1:16]), .ci(1'b0), .s(sa_s10),       .co(sa_c10));
	am_add #(.W(ADDR_W-16)) a_sa_h1  (.a(src_row[ADDR_W-1:16]), .b(r_s_xb_leg[ADDR_W-1:16]), .ci(1'b1), .s(sa_s11),       .co(sa_c11));
	assign s_addr[ADDR_W-1:16] = sa_c0 ? sa_s11 : sa_s10;
	assign fsm_co_unused[10] = (sa_c0 ? sa_c11 : sa_c10) & sa_c10;
`else
	am_add #(.W(ADDR_W)) a_da (.a(dst_row), .b(r_d_xb_leg), .ci(1'b0), .s(d_addr), .co(fsm_co_unused[9]));
	am_add #(.W(ADDR_W)) a_sa (.a(src_row), .b(r_s_xb_leg), .ci(1'b0), .s(s_addr), .co(fsm_co_unused[10]));
`endif

	/* Algebraic da_px in S_SA1: (dst_row[US-1:0] >> psl) + d_px[US-1:0] masked to (US - psl) bits (0 Pre-LUTs, 1 Post-LUT). */
	wire [US-1:0] dst_row_px_sa = (psl_leg_r == 3'd4) ? {4'b0000, dst_row[US-1:4]} :
	                              (psl_leg_r == 3'd3) ? {3'b000,  dst_row[US-1:3]} :
	                              (psl_leg_r == 3'd2) ? {2'b00,   dst_row[US-1:2]} :
	                              (psl_leg_r == 3'd1) ? {1'b0,    dst_row[US-1:1]} : dst_row[US-1:0];
	wire [US-1:0] da_px_s1_raw;
	wire da_px_s1_co;
	am_add #(.W(US)) a_dapx_s1 (.a(r_dst_row_px), .b(r_d_px_lo), .ci(1'b0), .s(da_px_s1_raw), .co(da_px_s1_co));
	wire [US-1:0] da_px_mask = (psl_leg_r == 3'd4) ? {{4{1'b0}}, {(US-4){1'b1}}} :
	                           (psl_leg_r == 3'd3) ? {{3{1'b0}}, {(US-3){1'b1}}} :
	                           (psl_leg_r == 3'd2) ? {{2{1'b0}}, {(US-2){1'b1}}} :
	                           (psl_leg_r == 3'd1) ? {1'b0,      {(US-1){1'b1}}} : {US{1'b1}};
	wire [US:0] da_px_s1 = {1'b0, da_px_s1_raw & da_px_mask};

	am_shift #(.W(US + 1), .SW(3)) h_sn (.a(seg_n), .amt(psl_leg), .left(1'b1), .arith(1'b0), .y(seg_bytes));

`ifdef SYNTH_KINTEX7
	wire dl_b0, dl_b10, dl_b11, sl_b0, sl_b10, sl_b11;
	wire [ADDR_W-17:0] dl_d10, dl_d11, sl_d10, sl_d11;
	am_sub #(.W(16))        s_dlo_lo (.a(da[15:0]),        .b(r_leg_back[15:0]),        .bi(1'b0), .d(d_lo[15:0]), .bo(dl_b0));
	am_sub #(.W(ADDR_W-16)) s_dlo_h0 (.a(da[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b0), .d(dl_d10),     .bo(dl_b10));
	am_sub #(.W(ADDR_W-16)) s_dlo_h1 (.a(da[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b1), .d(dl_d11),     .bo(dl_b11));
	assign d_lo[ADDR_W-1:16] = dl_b0 ? dl_d11 : dl_d10;
	assign fsm_co_unused[11] = (dl_b0 ? dl_b11 : dl_b10) & dl_b10 & da_px_s1_co;

	am_sub #(.W(16))        s_slo_lo (.a(sa[15:0]),        .b(r_leg_back[15:0]),        .bi(1'b0), .d(s_lo[15:0]), .bo(sl_b0));
	am_sub #(.W(ADDR_W-16)) s_slo_h0 (.a(sa[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b0), .d(sl_d10),     .bo(sl_b10));
	am_sub #(.W(ADDR_W-16)) s_slo_h1 (.a(sa[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b1), .d(sl_d11),     .bo(sl_b11));
	assign s_lo[ADDR_W-1:16] = sl_b0 ? sl_d11 : sl_d10;
	assign fsm_co_unused[12] = (sl_b0 ? sl_b11 : sl_b10) & sl_b10;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire dl_b0, dl_b10, dl_b11, sl_b0, sl_b10, sl_b11;
	wire [ADDR_W-17:0] dl_d10, dl_d11, sl_d10, sl_d11;
	am_sub #(.W(16))        s_dlo_lo (.a(da[15:0]),        .b(r_leg_back[15:0]),        .bi(1'b0), .d(d_lo[15:0]), .bo(dl_b0));
	am_sub #(.W(ADDR_W-16)) s_dlo_h0 (.a(da[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b0), .d(dl_d10),     .bo(dl_b10));
	am_sub #(.W(ADDR_W-16)) s_dlo_h1 (.a(da[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b1), .d(dl_d11),     .bo(dl_b11));
	assign d_lo[ADDR_W-1:16] = dl_b0 ? dl_d11 : dl_d10;
	assign fsm_co_unused[11] = (dl_b0 ? dl_b11 : dl_b10) & dl_b10 & da_px_s1_co;

	am_sub #(.W(16))        s_slo_lo (.a(sa[15:0]),        .b(r_leg_back[15:0]),        .bi(1'b0), .d(s_lo[15:0]), .bo(sl_b0));
	am_sub #(.W(ADDR_W-16)) s_slo_h0 (.a(sa[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b0), .d(sl_d10),     .bo(sl_b10));
	am_sub #(.W(ADDR_W-16)) s_slo_h1 (.a(sa[ADDR_W-1:16]), .b(r_leg_back[ADDR_W-1:16]), .bi(1'b1), .d(sl_d11),     .bo(sl_b11));
	assign s_lo[ADDR_W-1:16] = sl_b0 ? sl_d11 : sl_d10;
	assign fsm_co_unused[12] = (sl_b0 ? sl_b11 : sl_b10) & sl_b10;
`else
	am_sub #(.W(ADDR_W)) s_dlo (.a(da), .b(r_leg_back), .bi(1'b0), .d(d_lo), .bo(fsm_co_unused[11]));
	am_sub #(.W(ADDR_W)) s_slo (.a(sa), .b(r_leg_back), .bi(1'b0), .d(s_lo), .bo(fsm_co_unused[12]));
`endif

	wire [2:0] sa1_t3 = {px[1:0] == 2'b00, px[1] ^ px[0], px[0]};
	wire [2:0] sa1_s3 = {spx[1:0] == 2'b00, spx[1] ^ spx[0], spx[0]};
	wire [2:0] sa1_room_tile3 = (mdt_ctl && mst_ctl) ? ((sa1_t3 < sa1_s3) ? sa1_t3 : sa1_s3) :
	                            mdt_ctl ? sa1_t3 : sa1_s3;
	wire [US:0] sc_snu_next = (psl_snu == 3'd4) ? {sn[US-4:0], 4'b0000} :
	                          (psl_snu == 3'd3) ? {sn[US-3:0], 3'b000}  :
	                          (psl_snu == 3'd2) ? {sn[US-2:0], 2'b00}   :
	                          (psl_snu == 3'd1) ? {sn[US-1:0], 1'b0}    : sn;
	wire [US:0] sc_snu_m1;
	wire sc_snu_m1_bo;
`ifdef SYNTH_KINTEX7
	wire snm_c0, snm_h0, snm_h1;
	wire [US-3:0] snm_hi0, snm_hi1;
	am_sub #(.W(3))    s_sc_snm0  (.a(sc_snu_next[2:0]),  .b(3'd0),           .bi(1'b1), .d(sc_snu_m1[2:0]), .bo(snm_c0));
	am_sub #(.W(US-2)) s_sc_snmh0 (.a(sc_snu_next[US:3]), .b({(US-2){1'b0}}), .bi(1'b0), .d(snm_hi0),        .bo(snm_h0));
	am_sub #(.W(US-2)) s_sc_snmh1 (.a(sc_snu_next[US:3]), .b({(US-2){1'b0}}), .bi(1'b1), .d(snm_hi1),        .bo(snm_h1));
	assign sc_snu_m1[US:3] = snm_c0 ? snm_hi1 : snm_hi0;
	assign sc_snu_m1_bo = (snm_c0 ? snm_h1 : snm_h0) & snm_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire snm_c0, snm_h0, snm_h1;
	wire [US-3:0] snm_hi0, snm_hi1;
	am_sub #(.W(3))    s_sc_snm0  (.a(sc_snu_next[2:0]),  .b(3'd0),           .bi(1'b1), .d(sc_snu_m1[2:0]), .bo(snm_c0));
	am_sub #(.W(US-2)) s_sc_snmh0 (.a(sc_snu_next[US:3]), .b({(US-2){1'b0}}), .bi(1'b0), .d(snm_hi0),        .bo(snm_h0));
	am_sub #(.W(US-2)) s_sc_snmh1 (.a(sc_snu_next[US:3]), .b({(US-2){1'b0}}), .bi(1'b1), .d(snm_hi1),        .bo(snm_h1));
	assign sc_snu_m1[US:3] = snm_c0 ? snm_hi1 : snm_hi0;
	assign sc_snu_m1_bo = (snm_c0 ? snm_h1 : snm_h0) & snm_h0;
`else
	/* (sn << k) - 1 == ((sn - 1) << k) | ((1 << k) - 1): parallel -1 and shift cuts 17->8 cmos3. */
	wire [US:0] sn_m1_w;
	assign sn_m1_w[0] = ~sn[0];
	genvar g_snm;
	generate
		for (g_snm = 1; g_snm <= US; g_snm = g_snm + 1) begin : g_snm_fast
			assign sn_m1_w[g_snm] = sn[g_snm] ^ (~(|sn[g_snm-1:0]));
		end
	endgenerate
	assign sc_snu_m1 = (psl_snu == 3'd4) ? {sn_m1_w[US-4:0], 4'b1111} :
	                   (psl_snu == 3'd3) ? {sn_m1_w[US-3:0], 3'b111}  :
	                   (psl_snu == 3'd2) ? {sn_m1_w[US-2:0], 2'b11}   :
	                   (psl_snu == 3'd1) ? {sn_m1_w[US-1:0], 1'b1}    : sn_m1_w;
	assign sc_snu_m1_bo = 1'b0;
`endif

	wire [LA_W-1:0] dlo_line = dlo[ADDR_W-1:US];
	wire [LA_W-1:0] slo_line = slo[ADDR_W-1:US];
	am_add #(.W(2)) a_ydl (.a(ydl), .b(2'd0), .ci(1'b1), .s(ydl_inc), .co(fsm_co_unused[13]));
	am_add #(.W(2)) a_ysl (.a(ysl), .b(2'd0), .ci(1'b1), .s(ysl_inc), .co(fsm_co_unused[14]));

`ifdef SYNTH_KINTEX7
	wire rd_b0, rd_b1_0, rd_b1_1, rd_b2_0, rd_b2_1;
	wire [5:0] rd_s1_0, rd_s1_1, rd_s2_0, rd_s2_1;
	am_sub #(.W(5)) s_rdec_0  (.a(rows[4:0]),   .b(5'd0), .bi(1'b1), .d(rows_dec[4:0]), .bo(rd_b0));
	am_sub #(.W(6)) s_rdec_10 (.a(rows[10:5]),  .b(6'd0), .bi(1'b0), .d(rd_s1_0), .bo(rd_b1_0));
	am_sub #(.W(6)) s_rdec_11 (.a(rows[10:5]),  .b(6'd0), .bi(1'b1), .d(rd_s1_1), .bo(rd_b1_1));
	am_sub #(.W(6)) s_rdec_20 (.a(rows[16:11]), .b(6'd0), .bi(1'b0), .d(rd_s2_0), .bo(rd_b2_0));
	am_sub #(.W(6)) s_rdec_21 (.a(rows[16:11]), .b(6'd0), .bi(1'b1), .d(rd_s2_1), .bo(rd_b2_1));
	wire rd_b1 = rd_b1_0 | (rd_b1_1 & rd_b0);
	assign rows_dec[10:5]  = rd_b0 ? rd_s1_1 : rd_s1_0;
	assign rows_dec[16:11] = rd_b1 ? rd_s2_1 : rd_s2_0;
	assign fsm_co_unused[15] = (rd_b1 ? rd_b2_1 : rd_b2_0) & rd_b2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rd_b0, rd_b1_0, rd_b1_1, rd_b2_0, rd_b2_1;
	wire [5:0] rd_s1_0, rd_s1_1, rd_s2_0, rd_s2_1;
	am_sub #(.W(5)) s_rdec_0  (.a(rows[4:0]),   .b(5'd0), .bi(1'b1), .d(rows_dec[4:0]), .bo(rd_b0));
	am_sub #(.W(6)) s_rdec_10 (.a(rows[10:5]),  .b(6'd0), .bi(1'b0), .d(rd_s1_0), .bo(rd_b1_0));
	am_sub #(.W(6)) s_rdec_11 (.a(rows[10:5]),  .b(6'd0), .bi(1'b1), .d(rd_s1_1), .bo(rd_b1_1));
	am_sub #(.W(6)) s_rdec_20 (.a(rows[16:11]), .b(6'd0), .bi(1'b0), .d(rd_s2_0), .bo(rd_b2_0));
	am_sub #(.W(6)) s_rdec_21 (.a(rows[16:11]), .b(6'd0), .bi(1'b1), .d(rd_s2_1), .bo(rd_b2_1));
	wire rd_b1 = rd_b1_0 | (rd_b1_1 & rd_b0);
	assign rows_dec[10:5]  = rd_b0 ? rd_s1_1 : rd_s1_0;
	assign rows_dec[16:11] = rd_b1 ? rd_s2_1 : rd_s2_0;
	assign fsm_co_unused[15] = (rd_b1 ? rd_b2_1 : rd_b2_0) & rd_b2_0;
`else
	am_sub #(.W(17)) s_rdec (.a(rows), .b(17'd1), .bi(1'b0), .d(rows_dec), .bo(fsm_co_unused[15]));
`endif

	wire [4:0] sn_px_0  = r_mr_px    ? ~sn17[4:0]   : sn17[4:0];
	wire [5:0] sn_px_1  = r_mr_px_m  ? ~sn17[10:5]  : sn17[10:5];
	wire [5:0] sn_px_2  = r_mr_px_hi ? ~sn17[16:11] : sn17[16:11];
`ifdef SYNTH_KINTEX7
	wire px_c0, px_c1_0, px_c1_1, px_c2_0, px_c2_1;
	wire [5:0] px_s1_0, px_s1_1, px_s2_0, px_s2_1;
	am_add #(.W(5)) as_px_0  (.a(px[4:0]),   .b(sn_px_0), .ci(r_mr_px), .s(px_next[4:0]), .co(px_c0));
	am_add #(.W(6)) as_px_10 (.a(px[10:5]),  .b(sn_px_1), .ci(1'b0),    .s(px_s1_0),      .co(px_c1_0));
	am_add #(.W(6)) as_px_11 (.a(px[10:5]),  .b(sn_px_1), .ci(1'b1),    .s(px_s1_1),      .co(px_c1_1));
	am_add #(.W(6)) as_px_20 (.a(px[16:11]), .b(sn_px_2), .ci(1'b0),    .s(px_s2_0),      .co(px_c2_0));
	am_add #(.W(6)) as_px_21 (.a(px[16:11]), .b(sn_px_2), .ci(1'b1),    .s(px_s2_1),      .co(px_c2_1));
	wire px_c1 = px_c1_0 | (px_c1_1 & px_c0);
	assign px_next[10:5]  = px_c0 ? px_s1_1 : px_s1_0;
	assign px_next[16:11] = px_c1 ? px_s2_1 : px_s2_0;
	assign fsm_co_unused[16] = (px_c1 ? px_c2_1 : px_c2_0) & px_c2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire px_c0, px_c1_0, px_c1_1, px_c2_0, px_c2_1;
	wire [5:0] px_s1_0, px_s1_1, px_s2_0, px_s2_1;
	am_add #(.W(5)) as_px_0  (.a(px[4:0]),   .b(sn_px_0), .ci(r_mr_px), .s(px_next[4:0]), .co(px_c0));
	am_add #(.W(6)) as_px_10 (.a(px[10:5]),  .b(sn_px_1), .ci(1'b0),    .s(px_s1_0),      .co(px_c1_0));
	am_add #(.W(6)) as_px_11 (.a(px[10:5]),  .b(sn_px_1), .ci(1'b1),    .s(px_s1_1),      .co(px_c1_1));
	am_add #(.W(6)) as_px_20 (.a(px[16:11]), .b(sn_px_2), .ci(1'b0),    .s(px_s2_0),      .co(px_c2_0));
	am_add #(.W(6)) as_px_21 (.a(px[16:11]), .b(sn_px_2), .ci(1'b1),    .s(px_s2_1),      .co(px_c2_1));
	wire px_c1 = px_c1_0 | (px_c1_1 & px_c0);
	assign px_next[10:5]  = px_c0 ? px_s1_1 : px_s1_0;
	assign px_next[16:11] = px_c1 ? px_s2_1 : px_s2_0;
	assign fsm_co_unused[16] = (px_c1 ? px_c2_1 : px_c2_0) & px_c2_0;
`else
	wire [16:0] sn_px_17 = {sn_px_2, sn_px_1, sn_px_0};
	am_add #(.W(17)) as_px (.a(px), .b(sn_px_17), .ci(r_mr_px), .s(px_next), .co(fsm_co_unused[16]));
`endif

	wire [4:0] sn_spx_0 = r_mr_spx    ? ~sn17[4:0]   : sn17[4:0];
	wire [5:0] sn_spx_1 = r_mr_spx_m  ? ~sn17[10:5]  : sn17[10:5];
	wire [5:0] sn_spx_2 = r_mr_spx_hi ? ~sn17[16:11] : sn17[16:11];
`ifdef SYNTH_KINTEX7
	wire spx_c0, spx_c1_0, spx_c1_1, spx_c2_0, spx_c2_1;
	wire [5:0] spx_s1_0, spx_s1_1, spx_s2_0, spx_s2_1;
	am_add #(.W(5)) as_spx_0  (.a(spx[4:0]),   .b(sn_spx_0), .ci(r_mr_spx), .s(spx_next[4:0]), .co(spx_c0));
	am_add #(.W(6)) as_spx_10 (.a(spx[10:5]),  .b(sn_spx_1), .ci(1'b0),     .s(spx_s1_0),      .co(spx_c1_0));
	am_add #(.W(6)) as_spx_11 (.a(spx[10:5]),  .b(sn_spx_1), .ci(1'b1),     .s(spx_s1_1),      .co(spx_c1_1));
	am_add #(.W(6)) as_spx_20 (.a(spx[16:11]), .b(sn_spx_2), .ci(1'b0),     .s(spx_s2_0),      .co(spx_c2_0));
	am_add #(.W(6)) as_spx_21 (.a(spx[16:11]), .b(sn_spx_2), .ci(1'b1),     .s(spx_s2_1),      .co(spx_c2_1));
	wire spx_c1 = spx_c1_0 | (spx_c1_1 & spx_c0);
	assign spx_next[10:5]  = spx_c0 ? spx_s1_1 : spx_s1_0;
	assign spx_next[16:11] = spx_c1 ? spx_s2_1 : spx_s2_0;
	assign fsm_co_unused[17] = (spx_c1 ? spx_c2_1 : spx_c2_0) & spx_c2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire spx_c0, spx_c1_0, spx_c1_1, spx_c2_0, spx_c2_1;
	wire [5:0] spx_s1_0, spx_s1_1, spx_s2_0, spx_s2_1;
	am_add #(.W(5)) as_spx_0  (.a(spx[4:0]),   .b(sn_spx_0), .ci(r_mr_spx), .s(spx_next[4:0]), .co(spx_c0));
	am_add #(.W(6)) as_spx_10 (.a(spx[10:5]),  .b(sn_spx_1), .ci(1'b0),     .s(spx_s1_0),      .co(spx_c1_0));
	am_add #(.W(6)) as_spx_11 (.a(spx[10:5]),  .b(sn_spx_1), .ci(1'b1),     .s(spx_s1_1),      .co(spx_c1_1));
	am_add #(.W(6)) as_spx_20 (.a(spx[16:11]), .b(sn_spx_2), .ci(1'b0),     .s(spx_s2_0),      .co(spx_c2_0));
	am_add #(.W(6)) as_spx_21 (.a(spx[16:11]), .b(sn_spx_2), .ci(1'b1),     .s(spx_s2_1),      .co(spx_c2_1));
	wire spx_c1 = spx_c1_0 | (spx_c1_1 & spx_c0);
	assign spx_next[10:5]  = spx_c0 ? spx_s1_1 : spx_s1_0;
	assign spx_next[16:11] = spx_c1 ? spx_s2_1 : spx_s2_0;
	assign fsm_co_unused[17] = (spx_c1 ? spx_c2_1 : spx_c2_0) & spx_c2_0;
`else
	wire [16:0] sn_spx_17 = {sn_spx_2, sn_spx_1, sn_spx_0};
	am_add #(.W(17)) as_spx (.a(spx), .b(sn_spx_17), .ci(r_mr_spx), .s(spx_next), .co(fsm_co_unused[17]));
`endif

`ifdef SYNTH_KINTEX7
	wire rm_b0, rm_b1_0, rm_b1_1, rm_b2_0, rm_b2_1;
	wire [5:0] rm_s1_0, rm_s1_1, rm_s2_0, rm_s2_1;
	am_sub #(.W(5)) s_rem_0  (.a(rem[4:0]),   .b(sn17[4:0]),   .bi(1'b0), .d(rem_next[4:0]), .bo(rm_b0));
	am_sub #(.W(6)) s_rem_10 (.a(rem[10:5]),  .b(sn17[10:5]),  .bi(1'b0), .d(rm_s1_0), .bo(rm_b1_0));
	am_sub #(.W(6)) s_rem_11 (.a(rem[10:5]),  .b(sn17[10:5]),  .bi(1'b1), .d(rm_s1_1), .bo(rm_b1_1));
	am_sub #(.W(6)) s_rem_20 (.a(rem[16:11]), .b(sn17[16:11]), .bi(1'b0), .d(rm_s2_0), .bo(rm_b2_0));
	am_sub #(.W(6)) s_rem_21 (.a(rem[16:11]), .b(sn17[16:11]), .bi(1'b1), .d(rm_s2_1), .bo(rm_b2_1));
	wire rm_b1 = rm_b1_0 | (rm_b1_1 & rm_b0);
	assign rem_next[10:5]  = rm_b0 ? rm_s1_1 : rm_s1_0;
	assign rem_next[16:11] = rm_b1 ? rm_s2_1 : rm_s2_0;
	assign fsm_co_unused[18] = (rm_b1 ? rm_b2_1 : rm_b2_0) & rm_b2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rm_b0, rm_b1_0, rm_b1_1, rm_b2_0, rm_b2_1;
	wire [5:0] rm_s1_0, rm_s1_1, rm_s2_0, rm_s2_1;
	am_sub #(.W(5)) s_rem_0  (.a(rem[4:0]),   .b(sn17[4:0]),   .bi(1'b0), .d(rem_next[4:0]), .bo(rm_b0));
	am_sub #(.W(6)) s_rem_10 (.a(rem[10:5]),  .b(sn17[10:5]),  .bi(1'b0), .d(rm_s1_0), .bo(rm_b1_0));
	am_sub #(.W(6)) s_rem_11 (.a(rem[10:5]),  .b(sn17[10:5]),  .bi(1'b1), .d(rm_s1_1), .bo(rm_b1_1));
	am_sub #(.W(6)) s_rem_20 (.a(rem[16:11]), .b(sn17[16:11]), .bi(1'b0), .d(rm_s2_0), .bo(rm_b2_0));
	am_sub #(.W(6)) s_rem_21 (.a(rem[16:11]), .b(sn17[16:11]), .bi(1'b1), .d(rm_s2_1), .bo(rm_b2_1));
	wire rm_b1 = rm_b1_0 | (rm_b1_1 & rm_b0);
	assign rem_next[10:5]  = rm_b0 ? rm_s1_1 : rm_s1_0;
	assign rem_next[16:11] = rm_b1 ? rm_s2_1 : rm_s2_0;
	assign fsm_co_unused[18] = (rm_b1 ? rm_b2_1 : rm_b2_0) & rm_b2_0;
`else
	am_sub #(.W(17)) s_rem (.a(rem), .b(sn17), .bi(1'b0), .d(rem_next), .bo(fsm_co_unused[18]));
`endif
	assign cs32_co_unused = 16'd0;

	/*
	 * Burst Engine Arithmetic Instances (Guardrails Compliant)
	 */
	wire [16:0] rows_b_dec;
	wire [3:0] bw_seg_cnt_next;
	wire [3:0] br_seg_cnt_next;
	wire [3:0] bw_slot_p1, bw_slot_p2;
	wire [1:0] ydl_b_inc, ysl_b_inc;
	wire [ADDR_W-1:0] dst_row_b_next, src_row_b_next;
	wire [4:0] brdp0_up, brdp0_next, brdp1_up, brdp1_next;
	wire [4:0] bwr_up, bwr_next;
	wire [14:0] bst_co_unused;

`ifdef SYNTH_KINTEX7
	wire rsb_b0, rsb_b1_0, rsb_b1_1, rsb_b2_0, rsb_b2_1;
	wire [5:0] rsb_s1_0, rsb_s1_1, rsb_s2_0, rsb_s2_1;
	am_sub #(.W(5)) s_rowsb_0  (.a(rows_b[4:0]),   .b(5'd0), .bi(1'b1), .d(rows_b_dec[4:0]), .bo(rsb_b0));
	am_sub #(.W(6)) s_rowsb_10 (.a(rows_b[10:5]),  .b(6'd0), .bi(1'b0), .d(rsb_s1_0), .bo(rsb_b1_0));
	am_sub #(.W(6)) s_rowsb_11 (.a(rows_b[10:5]),  .b(6'd0), .bi(1'b1), .d(rsb_s1_1), .bo(rsb_b1_1));
	am_sub #(.W(6)) s_rowsb_20 (.a(rows_b[16:11]), .b(6'd0), .bi(1'b0), .d(rsb_s2_0), .bo(rsb_b2_0));
	am_sub #(.W(6)) s_rowsb_21 (.a(rows_b[16:11]), .b(6'd0), .bi(1'b1), .d(rsb_s2_1), .bo(rsb_b2_1));
	wire rsb_b1 = rsb_b1_0 | (rsb_b1_1 & rsb_b0);
	assign rows_b_dec[10:5]  = rsb_b0 ? rsb_s1_1 : rsb_s1_0;
	assign rows_b_dec[16:11] = rsb_b1 ? rsb_s2_1 : rsb_s2_0;
	assign bst_co_unused[0] = (rsb_b1 ? rsb_b2_1 : rsb_b2_0) & rsb_b2_0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rsb_b0, rsb_b1_0, rsb_b1_1, rsb_b2_0, rsb_b2_1;
	wire [5:0] rsb_s1_0, rsb_s1_1, rsb_s2_0, rsb_s2_1;
	am_sub #(.W(5)) s_rowsb_0  (.a(rows_b[4:0]),   .b(5'd0), .bi(1'b1), .d(rows_b_dec[4:0]), .bo(rsb_b0));
	am_sub #(.W(6)) s_rowsb_10 (.a(rows_b[10:5]),  .b(6'd0), .bi(1'b0), .d(rsb_s1_0), .bo(rsb_b1_0));
	am_sub #(.W(6)) s_rowsb_11 (.a(rows_b[10:5]),  .b(6'd0), .bi(1'b1), .d(rsb_s1_1), .bo(rsb_b1_1));
	am_sub #(.W(6)) s_rowsb_20 (.a(rows_b[16:11]), .b(6'd0), .bi(1'b0), .d(rsb_s2_0), .bo(rsb_b2_0));
	am_sub #(.W(6)) s_rowsb_21 (.a(rows_b[16:11]), .b(6'd0), .bi(1'b1), .d(rsb_s2_1), .bo(rsb_b2_1));
	wire rsb_b1 = rsb_b1_0 | (rsb_b1_1 & rsb_b0);
	assign rows_b_dec[10:5]  = rsb_b0 ? rsb_s1_1 : rsb_s1_0;
	assign rows_b_dec[16:11] = rsb_b1 ? rsb_s2_1 : rsb_s2_0;
	assign bst_co_unused[0] = (rsb_b1 ? rsb_b2_1 : rsb_b2_0) & rsb_b2_0;
`else
	am_sub #(.W(17)) s_rowsb (.a(rows_b), .b(17'd1), .bi(1'b0), .d(rows_b_dec), .bo(bst_co_unused[0]));
`endif
	am_add #(.W(4)) a_bwseg (.a({1'b0, bw_seg_idx}), .b(4'd0), .ci(1'b1), .s(bw_seg_cnt_next), .co(bst_co_unused[1]));
	am_add #(.W(4)) a_bwsp1 (.a(bw_slot_cnt), .b(4'd0), .ci(1'b1), .s(bw_slot_p1), .co(bst_co_unused[2]));
	am_add #(.W(4)) a_bwsp2 (.a(bw_slot_cnt), .b(4'd1), .ci(1'b1), .s(bw_slot_p2), .co(bst_co_unused[3]));
	am_add #(.W(4)) a_brseg (.a({1'b0, br_seg_idx}), .b(4'd0), .ci(1'b1), .s(br_seg_cnt_next), .co(bst_co_unused[4]));
	am_add #(.W(2)) a_ydlb (.a(ydl_b), .b(2'd0), .ci(1'b1), .s(ydl_b_inc), .co(bst_co_unused[5]));
	am_add #(.W(2)) a_yslb (.a(ysl_b), .b(2'd0), .ci(1'b1), .s(ysl_b_inc), .co(bst_co_unused[6]));
	wire ydl_is_3_lo   = ydl_b_is_3_lo;
	wire ydl_is_3_hi_a = ydl_b_is_3_hi_a;
	wire ydl_is_3_hi_b = ydl_b_is_3_hi_b;
	wire ydl_is_3_cur  = ydl_b_is_3_cur;
	wire ysl_is_3_lo   = ysl_b_is_3_lo;
	wire ysl_is_3_hi_a = ysl_b_is_3_hi_a;
	wire ysl_is_3_hi_b = ysl_b_is_3_hi_b;
	wire ysl_is_3_cur  = ysl_b_is_3_cur;
	wire [ADDR_W-1:0] b_dst_step_norm = {
		mdt_bstep1_a ? r_tile_step[ADDR_W-1:16] : ext_dstride[ADDR_W-1:16],
		mdt_bstep0_a ? r_tile_step[15:0]        : ext_dstride[15:0]
	};
	wire [ADDR_W-1:0] b_dst_step_back = {
		mdt_bstep1_b ? r_dst_back[ADDR_W-1:16]  : ext_dstride[ADDR_W-1:16],
		mdt_bstep0_b ? r_dst_back[15:0]         : ext_dstride[15:0]
	};
	wire [ADDR_W-1:0] b_dst_delta_norm = {
		mr_walk_dt1 ? r_dst_neg[ADDR_W-1:16] : b_dst_step_norm[ADDR_W-1:16],
		mr_walk_dt0 ? r_dst_neg[15:0]        : b_dst_step_norm[15:0]
	};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire bd_nm_neg  = mr_walk_dt0;
	wire bd_nm_tile = !mr_walk_dt0 && mdt_bstep0_a;
	wire bd_nm_str  = !mr_walk_dt0 && !mdt_bstep0_a;
`endif
`ifdef SYNTH_KINTEX7
	wire [ADDR_W-1:0] b_dst_delta_back = {
		mr_walk_dt3 ? r_dst_neg[ADDR_W-1:16] : b_dst_step_back[ADDR_W-1:16],
		mr_walk_dt2 ? r_dst_neg[15:0]        : b_dst_step_back[15:0]
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [ADDR_W-1:0] b_dst_delta_back = {
		mr_walk_dt3 ? r_dst_neg[ADDR_W-1:16] : b_dst_step_back[ADDR_W-1:16],
		mr_walk_dt2 ? r_dst_neg[15:0]        : b_dst_step_back[15:0]
	};
`else
	wire bd_bk_neg  = mr_walk_dt2;
	wire bd_bk_tile = !mr_walk_dt2 && mdt_bstep0_b;
	wire bd_bk_str  = !mr_walk_dt2 && !mdt_bstep0_b;
	wire [ADDR_W-1:0] b_dst_delta_back = {
		mr_walk_dt3 ? r_dst_neg[ADDR_W-1:16] : b_dst_step_back[ADDR_W-1:16],
		({16{bd_bk_neg}}  & r_dst_neg[15:0]) |
		({16{bd_bk_tile}} & r_dst_back[15:0]) |
		({16{bd_bk_str}}  & ext_dstride[15:0])
	};
	wire dpx_src_unused = &{1'b0, dpx_src_unused_keep};
`endif
	wire [ADDR_W-1:0] b_dst_delta = {
		ydl_is_3_hi_b ? r_dst_delta_back[ADDR_W-1:24] : r_dst_delta_norm[ADDR_W-1:24],
		ydl_is_3_hi_a ? r_dst_delta_back[23:16]       : r_dst_delta_norm[23:16],
		ydl_is_3_lo   ? r_dst_delta_back[15:0]        : r_dst_delta_norm[15:0]
	};
	wire [ADDR_W-1:0] b_src_step_norm = {
		mst_bstep1_a ? r_tile_step[ADDR_W-1:16] : ext_sstride[ADDR_W-1:16],
		mst_bstep0_a ? r_tile_step[15:0]        : ext_sstride[15:0]
	};
	wire [ADDR_W-1:0] b_src_step_back = {
		mst_bstep1_b ? r_src_back[ADDR_W-1:16]  : ext_sstride[ADDR_W-1:16],
		mst_bstep0_b ? r_src_back[15:0]         : ext_sstride[15:0]
	};
	wire [ADDR_W-1:0] b_src_delta_norm = {
		mr_walk_st1 ? r_src_neg[ADDR_W-1:16] : b_src_step_norm[ADDR_W-1:16],
		mr_walk_st0 ? r_src_neg[15:0]        : b_src_step_norm[15:0]
	};
	wire [ADDR_W-1:0] b_src_delta_back = {
		mr_walk_st3 ? r_src_neg[ADDR_W-1:16] : b_src_step_back[ADDR_W-1:16],
		mr_walk_st2 ? r_src_neg[15:0]        : b_src_step_back[15:0]
	};
	wire [ADDR_W-1:0] b_src_delta = {
		ysl_is_3_hi_b ? r_src_delta_back[ADDR_W-1:24] : r_src_delta_norm[ADDR_W-1:24],
		ysl_is_3_hi_a ? r_src_delta_back[23:16]       : r_src_delta_norm[23:16],
		ysl_is_3_lo   ? r_src_delta_back[15:0]        : r_src_delta_norm[15:0]
	};
	wire [ADDR_W-1:0] b_dst_delta_lo_sum = b_dst_delta;
	wire [ADDR_W-1:0] b_src_delta_lo_sum = b_src_delta;
`ifdef SYNTH_KINTEX7
	wire drb_c0, drb_c10, drb_c11, srb_c0, srb_c10, srb_c11;
	wire [ADDR_W-17:0] drb_s10, drb_s11, srb_s10, srb_s11;
	am_add #(.W(16))        a_drb_lo (.a(dst_row_b[15:0]),        .b(b_dst_delta_lo_sum[15:0]),        .ci(1'b0), .s(dst_row_b_next[15:0]), .co(drb_c0));
	am_add #(.W(ADDR_W-16)) a_drb_h0 (.a(dst_row_b[ADDR_W-1:16]), .b(b_dst_delta_lo_sum[ADDR_W-1:16]), .ci(1'b0), .s(drb_s10),            .co(drb_c10));
	am_add #(.W(ADDR_W-16)) a_drb_h1 (.a(dst_row_b[ADDR_W-1:16]), .b(b_dst_delta_lo_sum[ADDR_W-1:16]), .ci(1'b1), .s(drb_s11),            .co(drb_c11));
	assign dst_row_b_next[ADDR_W-1:16] = drb_c0 ? drb_s11 : drb_s10;
	assign bst_co_unused[7] = (drb_c0 ? drb_c11 : drb_c10) & drb_c10;

	am_add #(.W(16))        a_srb_lo (.a(src_row_b[15:0]),        .b(b_src_delta_lo_sum[15:0]),        .ci(1'b0), .s(src_row_b_next[15:0]), .co(srb_c0));
	am_add #(.W(ADDR_W-16)) a_srb_h0 (.a(src_row_b[ADDR_W-1:16]), .b(b_src_delta_lo_sum[ADDR_W-1:16]), .ci(1'b0), .s(srb_s10),            .co(srb_c10));
	am_add #(.W(ADDR_W-16)) a_srb_h1 (.a(src_row_b[ADDR_W-1:16]), .b(b_src_delta_lo_sum[ADDR_W-1:16]), .ci(1'b1), .s(srb_s11),            .co(srb_c11));
	assign src_row_b_next[ADDR_W-1:16] = srb_c0 ? srb_s11 : srb_s10;
	assign bst_co_unused[8] = (srb_c0 ? srb_c11 : srb_c10) & srb_c10;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire drb_c0, drb_c10, drb_c11, srb_c0, srb_c10, srb_c11;
	wire [ADDR_W-17:0] drb_s10, drb_s11, srb_s10, srb_s11;
	am_add #(.W(16))        a_drb_lo (.a(dst_row_b[15:0]),        .b(b_dst_delta_lo_sum[15:0]),        .ci(1'b0), .s(dst_row_b_next[15:0]), .co(drb_c0));
	am_add #(.W(ADDR_W-16)) a_drb_h0 (.a(dst_row_b[ADDR_W-1:16]), .b(b_dst_delta_lo_sum[ADDR_W-1:16]), .ci(1'b0), .s(drb_s10),            .co(drb_c10));
	am_add #(.W(ADDR_W-16)) a_drb_h1 (.a(dst_row_b[ADDR_W-1:16]), .b(b_dst_delta_lo_sum[ADDR_W-1:16]), .ci(1'b1), .s(drb_s11),            .co(drb_c11));
	assign dst_row_b_next[ADDR_W-1:16] = drb_c0 ? drb_s11 : drb_s10;
	assign bst_co_unused[7] = (drb_c0 ? drb_c11 : drb_c10) & drb_c10;

	am_add #(.W(16))        a_srb_lo (.a(src_row_b[15:0]),        .b(b_src_delta_lo_sum[15:0]),        .ci(1'b0), .s(src_row_b_next[15:0]), .co(srb_c0));
	am_add #(.W(ADDR_W-16)) a_srb_h0 (.a(src_row_b[ADDR_W-1:16]), .b(b_src_delta_lo_sum[ADDR_W-1:16]), .ci(1'b0), .s(srb_s10),            .co(srb_c10));
	am_add #(.W(ADDR_W-16)) a_srb_h1 (.a(src_row_b[ADDR_W-1:16]), .b(b_src_delta_lo_sum[ADDR_W-1:16]), .ci(1'b1), .s(srb_s11),            .co(srb_c11));
	assign src_row_b_next[ADDR_W-1:16] = srb_c0 ? srb_s11 : srb_s10;
	assign bst_co_unused[8] = (srb_c0 ? srb_c11 : srb_c10) & srb_c10;
`else
	am_add #(.W(ADDR_W)) a_drb (.a(dst_row_b), .b(r_b_dst_delta), .ci(1'b0), .s(dst_row_b_next), .co(bst_co_unused[7]));
	am_add #(.W(ADDR_W)) a_srb (.a(src_row_b), .b(r_b_src_delta), .ci(1'b0), .s(src_row_b_next), .co(bst_co_unused[8]));
	wire b_delta_unused = &{1'b0, b_dst_delta_lo_sum, b_src_delta_lo_sum};
`endif

`ifdef SYNTH_KINTEX7
	wire [6:0] bwb_buf;
	genvar bwi_k;
	generate
		for (bwi_k = 0; bwi_k < 7; bwi_k = bwi_k + 1) begin : g_bwb
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bwb (.I0(bw_bank), .O(bwb_buf[bwi_k]));
		end
	endgenerate
	wire bw_bank_meta_e01 = bwb_buf[0];
	wire bw_bank_meta_e23 = bwb_buf[1];
	wire bw_bank_meta_e45 = bwb_buf[2];
	wire bw_bank_meta_e67 = bwb_buf[3];
	wire bw_bank_req      = bwb_buf[4];
	wire bw_bank_br0      = bwb_buf[5];
	wire bw_bank_br1      = bwb_buf[6];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [6:0] bwb_buf;
	genvar bwi_k;
	generate
		for (bwi_k = 0; bwi_k < 7; bwi_k = bwi_k + 1) begin : g_bwb
			assign bwb_buf[bwi_k] = bw_bank;
		end
	endgenerate
	wire bw_bank_meta_e01 = bwb_buf[0];
	wire bw_bank_meta_e23 = bwb_buf[1];
	wire bw_bank_meta_e45 = bwb_buf[2];
	wire bw_bank_meta_e67 = bwb_buf[3];
	wire bw_bank_req      = bwb_buf[4];
	wire bw_bank_br0      = bwb_buf[5];
	wire bw_bank_br1      = bwb_buf[6];
`else
	wire bw_bank_meta_e01 = bw_bank;
	wire bw_bank_meta_e23 = bw_bank;
	wire bw_bank_meta_e45 = bw_bank;
	wire bw_bank_meta_e67 = bw_bank;
	wire bw_bank_req      = bw_bank;
	wire bw_bank_br0      = bw_bank;
	wire bw_bank_br1      = bw_bank;
`endif
	wire bw_bank_meta     = bw_bank_meta_e01;

	wire rsp_rd_bst = rsp_valid_bst && !rsp_write && m_burst && (rsp_tag[7] == 1'b0);
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg pp_wr_en_r;
	reg [4:0] pp_wr_addr_r;
	reg [LINE_BITS-1:0] pp_wr_data_r;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			pp_wr_en_r <= 1'b0;
		else
			pp_wr_en_r <= rsp_rd_bst;
	end
	always @(posedge clk) begin
		pp_wr_addr_r <= rsp_tag[4:0];
		pp_wr_data_r <= rsp_rdata;
		if (pp_wr_en_r)
			pp_ram[pp_wr_addr_r] <= pp_wr_data_r;
	end
`endif
	wire br_seg_is_last_0 = (br_seg_idx == bank_last_seg_0);
	wire br_seg_is_last_1 = (br_seg_idx == bank_last_seg_1);
	wire br_seg_is_last = br_bank ? br_seg_is_last_1 : br_seg_is_last_0;
`ifdef SYNTH_KINTEX7
	wire p_valid_brdp;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pvb_brdp (.I0(p_valid), .O(p_valid_brdp));
	(* keep = "true" *) wire brdp0_pre = (!bank_active[0] || !bank_closed[0]) && mc_issue && !bwv_ctl && !bw_bank_br0;
	(* keep = "true" *) wire brdp1_pre = (!bank_active[1] || !bank_closed[1]) && mc_issue && !bwv_ctl &&  bw_bank_br1;
	(* keep = "true" *) wire brdp0_inc = p_valid_brdp && req_rdy_i && brdp0_pre;
	(* keep = "true" *) wire brdp1_inc = p_valid_brdp && req_rdy_i && brdp1_pre;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire p_valid_brdp;
	assign p_valid_brdp = p_valid;
	(* keep = "true" *) wire brdp0_pre = (!bank_active[0] || !bank_closed[0]) && mc_issue && !bwv_ctl && !bw_bank_br0;
	(* keep = "true" *) wire brdp1_pre = (!bank_active[1] || !bank_closed[1]) && mc_issue && !bwv_ctl &&  bw_bank_br1;
	(* keep = "true" *) wire brdp0_inc = p_valid_brdp && req_rdy_i && brdp0_pre;
	(* keep = "true" *) wire brdp1_inc = p_valid_brdp && req_rdy_i && brdp1_pre;
`else
	wire brdp0_inc = bst_rd_fire && (bw_bank_br0 == 1'b0);
	wire brdp1_inc = bst_rd_fire && (bw_bank_br1 == 1'b1);
`endif
	wire brdp0_dec = rsp_rd_bst && (rsp_tag[4] == 1'b0);
	wire brdp1_dec = rsp_rd_bst && (rsp_tag[4] == 1'b1);
	wire [4:0] brdp0_m1, brdp1_m1;
	am_add #(.W(5)) a_brdp0 (.a(bank_rd_pending_0), .b(5'd0), .ci(1'b1), .s(brdp0_up), .co(bst_co_unused[9]));
	am_sub #(.W(5)) s_brdp0 (.a(bank_rd_pending_0), .b(5'd0), .bi(1'b1), .d(brdp0_m1), .bo(bst_co_unused[10]));
	assign brdp0_next = (brdp0_inc == brdp0_dec) ? bank_rd_pending_0 : (brdp0_inc ? brdp0_up : brdp0_m1);
	am_add #(.W(5)) a_brdp1 (.a(bank_rd_pending_1), .b(5'd0), .ci(1'b1), .s(brdp1_up), .co(bst_co_unused[11]));
	am_sub #(.W(5)) s_brdp1 (.a(bank_rd_pending_1), .b(5'd0), .bi(1'b1), .d(brdp1_m1), .bo(bst_co_unused[12]));
	assign brdp1_next = (brdp1_inc == brdp1_dec) ? bank_rd_pending_1 : (brdp1_inc ? brdp1_up : brdp1_m1);
	wire [4:0] brdp0_d = is_burst_ctl0 ? brdp0_next : 5'd0;
	wire [4:0] brdp1_d = is_burst_ctl0 ? brdp1_next : 5'd0;

	wire brdp0_is_zero = (bank_rd_pending_0[4:1] == 4'd0) &&
	                     (!bank_rd_pending_0[0] || (rsp_rd_bst && (rsp_tag[4] == 1'b0)));
	wire brdp1_is_zero = (bank_rd_pending_1[4:1] == 4'd0) &&
	                     (!bank_rd_pending_1[0] || (rsp_rd_bst && (rsp_tag[4] == 1'b1)));

	wire rsp_wr_bst = rsp_valid_bst && rsp_write && m_burst;
	wire [4:0] bwr_p1, bwr_m1;
	am_add #(.W(5)) a_bwr (.a(burst_wr_out), .b(5'd0), .ci(1'b1), .s(bwr_p1), .co(bst_co_unused[13]));
	am_sub #(.W(5)) s_bwr (.a(burst_wr_out), .b(5'd0), .bi(1'b1), .d(bwr_m1), .bo(bst_co_unused[14]));
	assign bwr_up = bwr_p1;
	assign bwr_next = (bst_wr_fire && !rsp_wr_bst) ? bwr_p1 :
	                  (!bst_wr_fire && rsp_wr_bst) ? bwr_m1 : burst_wr_out;

	reg r_burst_drain_done;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			r_burst_drain_done <= 1'b0;
		else
			r_burst_drain_done <= r_st_burst && b_draining && !s1_valid && !s2_valid && !s3_valid && (burst_wr_out == 5'd0) && !rsp_valid_bst;
	end
	wire burst_drain_done = r_burst_drain_done;

	/* Mode determination at kick */
	wire is_smear = (sbase_u == dbase_u) && !o_rev && ({1'b0, s_src_xy[15:0]} < clip_x0);
	wire op_burst = !o_blend && !is_smear;

`ifdef SYNTH_KINTEX7
	wire [3:0] bsc_b0, bsc_b1;
	wire [3:0] bscp1_b0, bscp1_b1;
	genvar bi;
	generate
		for (bi = 0; bi < 4; bi = bi + 1) begin : g_bsc_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc0  (.I0(bw_slot_cnt[bi]), .O(bsc_b0[bi]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsc1  (.I0(bw_slot_cnt[bi]), .O(bsc_b1[bi]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bscp0 (.I0(bw_slot_p1[bi]),  .O(bscp1_b0[bi]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bscp1 (.I0(bw_slot_p1[bi]),  .O(bscp1_b1[bi]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] bsc_b0, bsc_b1;
	wire [3:0] bscp1_b0, bscp1_b1;
	genvar bi;
	generate
		for (bi = 0; bi < 4; bi = bi + 1) begin : g_bsc_buf
			assign bsc_b0[bi] = bw_slot_cnt[bi];
			assign bsc_b1[bi] = bw_slot_cnt[bi];
			assign bscp1_b0[bi] = bw_slot_p1[bi];
			assign bscp1_b1[bi] = bw_slot_p1[bi];
		end
	endgenerate
`else
	wire [3:0] bsc_b0 = bw_slot_cnt;
	wire [3:0] bsc_b1 = bw_slot_cnt;
	wire [3:0] bscp1_b0 = bw_slot_p1;
	wire [3:0] bscp1_b1 = bw_slot_p1;
`endif

	/* Split metadata writes to ensure clock enable fanouts <= 16 */
	wire [63:0] issue_be_64 = {{(64 - LINE_BYTES){1'b0}}, issue_be};

	genvar mk;
	generate
		for (mk = 0; mk < 8; mk = mk + 1) begin : g_meta_entry
			wire mk_bw_meta = (mk < 2) ? bw_bank_meta_e01 :
			                  (mk < 4) ? bw_bank_meta_e23 :
			                  (mk < 6) ? bw_bank_meta_e45 : bw_bank_meta_e67;
			wire mk_if_m0   = (mk < 2) ? issue_fire_m0_e01 :
			                  (mk < 4) ? issue_fire_m0_e23 :
			                  (mk < 6) ? issue_fire_m0_e45 : issue_fire_m0_e67;
			wire mk_if_m1   = (mk < 2) ? issue_fire_m1_e01 :
			                  (mk < 4) ? issue_fire_m1_e23 :
			                  (mk < 6) ? issue_fire_m1_e45 : issue_fire_m1_e67;

`ifdef SYNTH_KINTEX7
			wire we0_dl0, we0_dl1, we0_be0, we0_be1, we0_ct;
			wire we1_dl0, we1_dl1, we1_be0, we1_be1, we1_ct;
			LUT3 #(.INIT(8'h20)) u_w0_dl0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_dl0));
			LUT3 #(.INIT(8'h20)) u_w0_dl1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_dl1));
			LUT3 #(.INIT(8'h20)) u_w0_be0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_be0));
			LUT3 #(.INIT(8'h20)) u_w0_be1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_be1));
			LUT3 #(.INIT(8'h20)) u_w0_ct  (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_ct));

			LUT3 #(.INIT(8'h80)) u_w1_dl0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_dl0));
			LUT3 #(.INIT(8'h80)) u_w1_dl1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_dl1));
			LUT3 #(.INIT(8'h80)) u_w1_be0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_be0));
			LUT3 #(.INIT(8'h80)) u_w1_be1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_be1));
			LUT3 #(.INIT(8'h80)) u_w1_ct  (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_ct));
`elsif SYNTH_KINTEX_ULTRASCALE
			wire we0_dl0, we0_dl1, we0_be0, we0_be1, we0_ct;
			wire we1_dl0, we1_dl1, we1_be0, we1_be1, we1_ct;
			LUT3 #(.INIT(8'h20)) u_w0_dl0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_dl0));
			LUT3 #(.INIT(8'h20)) u_w0_dl1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_dl1));
			LUT3 #(.INIT(8'h20)) u_w0_be0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_be0));
			LUT3 #(.INIT(8'h20)) u_w0_be1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_be1));
			LUT3 #(.INIT(8'h20)) u_w0_ct  (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m0), .O(we0_ct));

			LUT3 #(.INIT(8'h80)) u_w1_dl0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_dl0));
			LUT3 #(.INIT(8'h80)) u_w1_dl1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_dl1));
			LUT3 #(.INIT(8'h80)) u_w1_be0 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_be0));
			LUT3 #(.INIT(8'h80)) u_w1_be1 (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_be1));
			LUT3 #(.INIT(8'h80)) u_w1_ct  (.I0(bw_seg_onehot[mk]), .I1(mk_bw_meta), .I2(mk_if_m1), .O(we1_ct));
`else
			wire we0_dl0 = mk_if_m0 && !mk_bw_meta && bw_seg_onehot[mk];
			wire we0_dl1 = mk_if_m0 && !mk_bw_meta && bw_seg_onehot[mk];
			wire we0_be0 = mk_if_m0 && !mk_bw_meta && bw_seg_onehot[mk];
			wire we0_be1 = mk_if_m0 && !mk_bw_meta && bw_seg_onehot[mk];
			wire we0_ct  = mk_if_m0 && !mk_bw_meta && bw_seg_onehot[mk];

			wire we1_dl0 = mk_if_m1 && mk_bw_meta && bw_seg_onehot[mk];
			wire we1_dl1 = mk_if_m1 && mk_bw_meta && bw_seg_onehot[mk];
			wire we1_be0 = mk_if_m1 && mk_bw_meta && bw_seg_onehot[mk];
			wire we1_be1 = mk_if_m1 && mk_bw_meta && bw_seg_onehot[mk];
			wire we1_ct  = mk_if_m1 && mk_bw_meta && bw_seg_onehot[mk];
`endif
			always @(posedge clk) begin
				if (we0_dl0) begin
					meta_dline_0_0[mk] <= p_dline[6:0];
					meta_dline_0_1[mk] <= p_dline[13:7];
				end
				if (we0_dl1) begin
					meta_dline_0_2[mk] <= p_dline[20:14];
					meta_dline_0_3[mk] <= p_dline[LA_W-1:21];
				end
				if (we0_be0) begin
					meta_be_0_0[mk] <= issue_be_64[7:0];
					meta_be_0_1[mk] <= issue_be_64[15:8];
				end
				if (we0_be1) begin
					meta_be_0_2[mk] <= issue_be_64[23:16];
					meta_be_0_3[mk] <= issue_be_64[31:24];
					if (LINE_BYTES >= 64) begin
						meta_be_0_4[mk] <= issue_be_64[39:32];
						meta_be_0_5[mk] <= issue_be_64[47:40];
						meta_be_0_6[mk] <= issue_be_64[55:48];
						meta_be_0_7[mk] <= issue_be_64[63:56];
					end
				end
				if (we0_ct) begin
					meta_slot0_0[mk]  <= bsc_b0;
					meta_slot1_0[mk]  <= p_two_lines ? bscp1_b0 : bsc_b0;
					meta_off_0[mk]    <= p_off;
					meta_n0slot_0[mk] <= p_n0_slot;
				end

				if (we1_dl0) begin
					meta_dline_1_0[mk] <= p_dline[6:0];
					meta_dline_1_1[mk] <= p_dline[13:7];
				end
				if (we1_dl1) begin
					meta_dline_1_2[mk] <= p_dline[20:14];
					meta_dline_1_3[mk] <= p_dline[LA_W-1:21];
				end
				if (we1_be0) begin
					meta_be_1_0[mk] <= issue_be_64[7:0];
					meta_be_1_1[mk] <= issue_be_64[15:8];
				end
				if (we1_be1) begin
					meta_be_1_2[mk] <= issue_be_64[23:16];
					meta_be_1_3[mk] <= issue_be_64[31:24];
					if (LINE_BYTES >= 64) begin
						meta_be_1_4[mk] <= issue_be_64[39:32];
						meta_be_1_5[mk] <= issue_be_64[47:40];
						meta_be_1_6[mk] <= issue_be_64[55:48];
						meta_be_1_7[mk] <= issue_be_64[63:56];
					end
				end
				if (we1_ct) begin
					meta_slot0_1[mk]  <= bsc_b1;
					meta_slot1_1[mk]  <= p_two_lines ? bscp1_b1 : bsc_b1;
					meta_off_1[mk]    <= p_off;
					meta_n0slot_1[mk] <= p_n0_slot;
				end
			end
		end
	endgenerate

`ifdef SYNTH_KINTEX7
	/* S_IDLE enable buffers */
	wire ce_kick = r_st_idle && kick;
	wire ce_kick_ok = ce_kick && op_ok;

	wire [15:0] ce_k_buf;
	wire [3:0] ik_mr_buf;
	genvar g_ki;
	generate
		for (g_ki = 0; g_ki < 6; g_ki = g_ki + 1) begin : g_ck0
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ck0 (.I0(ce_kick), .O(ce_k_buf[g_ki]));
		end
		for (g_ki = 6; g_ki < 14; g_ki = g_ki + 1) begin : g_ck1
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ck1 (.I0(ce_kick_ok), .O(ce_k_buf[g_ki]));
		end
		for (g_ki = 14; g_ki < 16; g_ki = g_ki + 1) begin : g_ck2
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ck2 (.I0(ce_kick), .O(ce_k_buf[g_ki]));
		end
		for (g_ki = 0; g_ki < 4; g_ki = g_ki + 1) begin : g_ckm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ckm (.I0(ce_kick), .O(ik_mr_buf[g_ki]));
		end
	endgenerate
	wire ik_cfg_copy   = ce_k_buf[0];
	wire ik_cfg_dtile  = ce_k_buf[1];
	wire ik_cfg_stile  = ce_k_buf[2];
	wire ik_cfg_psl0_0 = ce_k_buf[3];
	wire ik_cfg_psl0_1 = ce_k_buf[14];
	wire ik_cfg_psl0_2 = ce_k_buf[15];
	wire ik_cfg_psl1   = ce_k_buf[4];
	wire ik_cfg_ctl    = ce_k_buf[5];
	wire ik_x0_l = ce_k_buf[6];
	wire ik_x0_h = ce_k_buf[7];
	wire ik_y0_l = ce_k_buf[8];
	wire ik_y0_h = ce_k_buf[9];
	wire ik_x1_l = ce_k_buf[10];
	wire ik_x1_h = ce_k_buf[11];
	wire ik_y1_l = ce_k_buf[12];
	wire ik_y1_h = ce_k_buf[13];

	/* S_SD enable buffers (split sd_off into 4 + sd_ctl1 into 4 leaves for <= 17 loads each) */
	wire ce_sd_raw = r_st_sd;
	wire [14:0] ce_sd_buf;
	genvar g_sdi;
	generate
		for (g_sdi = 0; g_sdi < 15; g_sdi = g_sdi + 1) begin : g_ce_sd
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sd (.I0(ce_sd_raw), .O(ce_sd_buf[g_sdi]));
		end
	endgenerate
	wire sd_sl0  = ce_sd_buf[0];
	wire sd_sl1  = ce_sd_buf[1];
	wire sd_s00  = ce_sd_buf[2];
	wire sd_s01  = ce_sd_buf[3];
	wire sd_s10  = ce_sd_buf[4];
	wire sd_s11  = ce_sd_buf[5];
	wire sd_ctl0 = ce_sd_buf[6];
	wire sd_ctl1 = ce_sd_buf[7];
	wire sd_off0 = ce_sd_buf[8];
	wire sd_off1 = ce_sd_buf[9];
	wire sd_off2 = ce_sd_buf[10];
	wire sd_off3 = ce_sd_buf[11];
	wire sd_rpx  = ce_sd_buf[12];
	wire sd_rspx = ce_sd_buf[13];
	wire sd_rrem = ce_sd_buf[14];

	/* S_WR enable buffers */
	wire [3:0] wg_buf;
	wire [4:0] sl_buf;
	genvar g_wgi;
	generate
		for (g_wgi = 0; g_wgi < 4; g_wgi = g_wgi + 1) begin : g_ce_wg
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_wg (.I0(wr_go), .O(wg_buf[g_wgi]));
		end
		for (g_wgi = 0; g_wgi < 5; g_wgi = g_wgi + 1) begin : g_ce_wsl
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sl (.I0(seg_last), .O(sl_buf[g_wgi]));
		end
	endgenerate
	wire wg_px = wg_buf[0], wg_spx = wg_buf[1], wg_rem = wg_buf[2], wg_ctl = wg_buf[3];
	wire sl_px = sl_buf[0], sl_spx = sl_buf[1], sl_rem = sl_buf[2], sl_ctl = sl_buf[3], sl_row = sl_buf[4];
	wire wg_row = wg_buf[3];

	/* 1. S_CLIP: 15 leaves, each driving <= 17 FFs */
	wire ce_clip_raw = r_st_clip;
	wire [1:0] ce_clip_m;
	wire [14:0] ce_clip_l;
	genvar g_cm, g_cl;
	generate
		for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_ce_cm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cm (.I0(ce_clip_raw), .O(ce_clip_m[g_cm]));
		end
		for (g_cl = 0; g_cl < 15; g_cl = g_cl + 1) begin : g_ce_cl
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cl (.I0(ce_clip_m[g_cl / 8]), .O(ce_clip_l[g_cl]));
		end
	endgenerate
	wire cl_w   = ce_clip_l[0];
	wire cl_wm1 = ce_clip_l[13];
	wire cl_r   = ce_clip_l[1];
	wire cl_sx  = ce_clip_l[2];
	wire cl_sy  = ce_clip_l[3];
	wire cl_stp = ce_clip_l[4];
	wire cl_ds0 = ce_clip_l[5];
	wire cl_ds1 = ce_clip_l[6];
	wire cl_ss0 = ce_clip_l[7];
	wire cl_ss1 = ce_clip_l[8];
	wire cl_db0 = ce_clip_l[9];
	wire cl_db1 = ce_clip_l[10];
	wire cl_sb0 = ce_clip_l[11];
	wire cl_sb1 = ce_clip_l[12];
	wire cl_tile8 = ce_clip_l[14];

	/* 2. S_START: 16 leaves, each driving <= 19 FFs */
	wire ce_start_raw = r_st_start;
	wire [3:0] ce_start_m;
	wire [15:0] ce_start_l;
	genvar g_sm, g_sl;
	generate
		for (g_sm = 0; g_sm < 4; g_sm = g_sm + 1) begin : g_ce_sm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sm (.I0(ce_start_raw), .O(ce_start_m[g_sm]));
		end
		for (g_sl = 0; g_sl < 16; g_sl = g_sl + 1) begin : g_ce_sl
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sl (.I0(ce_start_m[g_sl / 4]), .O(ce_start_l[g_sl]));
		end
	endgenerate
	wire st_px  = ce_start_l[0];
	wire st_spx = ce_start_l[1];
	wire st_db0 = ce_start_l[2];
	wire st_db1 = ce_start_l[3];
	wire st_sb0 = ce_start_l[4];
	wire st_sb1 = ce_start_l[5];
	wire st_dn0 = ce_start_l[6];
	wire st_dn1 = ce_start_l[7];
	wire st_sn0 = ce_start_l[8];
	wire st_sn1 = ce_start_l[9];
	wire st_df0 = ce_start_l[10];
	wire st_df1 = ce_start_l[11];
	wire st_sf0 = ce_start_l[12];
	wire st_sf1 = ce_start_l[13];
	wire st_yd  = ce_start_l[14];
	wire st_ys  = ce_start_l[15];

	/* 3. S_SA / S_SA1: leaves, each driving <= 16 FFs */
	wire ce_sa_raw  = r_st_sa && !fail_fsm0;
	wire ce_sa1_raw = r_st_sa1;
	wire [4:0] ce_sa_l;
	wire [4:0] ce_sa1_l;
	genvar g_sa_i;
	generate
		for (g_sa_i = 0; g_sa_i < 5; g_sa_i = g_sa_i + 1) begin : g_ce_sa
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sa  (.I0(ce_sa_raw),  .O(ce_sa_l[g_sa_i]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sa1 (.I0(ce_sa1_raw), .O(ce_sa1_l[g_sa_i]));
		end
	endgenerate
	wire sa_da0  = ce_sa_l[0];
	wire sa_da1  = ce_sa_l[1];
	wire sa_sa0  = ce_sa_l[2];
	wire sa_sa1  = ce_sa_l[3];
	wire sa_ctl  = ce_sa_l[4];
	wire sa1_da0 = ce_sa1_l[0];
	wire sa1_da1 = ce_sa1_l[1];
	wire sa1_sa0 = ce_sa1_l[2];
	wire sa1_sa1 = ce_sa1_l[3];
	wire sa1_ctl = ce_sa1_l[4];

	/* 4. S_SC: 4 leaves, each driving 16 FFs */
	wire ce_sc_raw = r_st_sc;
	wire [3:0] ce_sc_l;
	genvar g_sc_i;
	generate
		for (g_sc_i = 0; g_sc_i < 4; g_sc_i = g_sc_i + 1) begin : g_ce_sc
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sc (.I0(ce_sc_raw), .O(ce_sc_l[g_sc_i]));
		end
	endgenerate
	wire sc_snu  = r_st_sc;
	wire sc_dlo0 = ce_sc_l[0];
	wire sc_dlo1 = ce_sc_l[1];
	wire sc_slo0 = ce_sc_l[2];
	wire sc_slo1 = ce_sc_l[3];

	/* 5. S_INIT enable buffers */
	wire ce_init_raw = r_st_init_a;
	wire [1:0] ce_init_m;
	wire [7:0] ce_init_l;
	genvar g_inm, g_inl;
	generate
		for (g_inm = 0; g_inm < 2; g_inm = g_inm + 1) begin : g_ce_inm
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_im (.I0(ce_init_raw), .O(ce_init_m[g_inm]));
		end
		for (g_inl = 0; g_inl < 8; g_inl = g_inl + 1) begin : g_ce_inl
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_il (.I0(ce_init_m[g_inl / 4]), .O(ce_init_l[g_inl]));
		end
	endgenerate
	wire in_da0 = 1'b1;
	wire in_da1 = 1'b1;
	wire in_sa0 = 1'b1;
	wire in_sa1 = 1'b1;

`elsif SYNTH_KINTEX_ULTRASCALE
	/* S_IDLE enable buffers */
	wire ce_kick = r_st_idle && kick;
	wire ce_kick_ok = ce_kick && op_ok;

	wire [15:0] ce_k_buf;
	wire [3:0] ik_mr_buf;
	genvar g_ki;
	generate
		for (g_ki = 0; g_ki < 6; g_ki = g_ki + 1) begin : g_ck0
			assign ce_k_buf[g_ki] = ce_kick;
		end
		for (g_ki = 6; g_ki < 14; g_ki = g_ki + 1) begin : g_ck1
			assign ce_k_buf[g_ki] = ce_kick_ok;
		end
		for (g_ki = 14; g_ki < 16; g_ki = g_ki + 1) begin : g_ck2
			assign ce_k_buf[g_ki] = ce_kick;
		end
		for (g_ki = 0; g_ki < 4; g_ki = g_ki + 1) begin : g_ckm
			assign ik_mr_buf[g_ki] = ce_kick;
		end
	endgenerate
	wire ik_cfg_copy   = ce_k_buf[0];
	wire ik_cfg_dtile  = ce_k_buf[1];
	wire ik_cfg_stile  = ce_k_buf[2];
	wire ik_cfg_psl0_0 = ce_k_buf[3];
	wire ik_cfg_psl0_1 = ce_k_buf[14];
	wire ik_cfg_psl0_2 = ce_k_buf[15];
	wire ik_cfg_psl1   = ce_k_buf[4];
	wire ik_cfg_ctl    = ce_k_buf[5];
	wire ik_x0_l = ce_k_buf[6];
	wire ik_x0_h = ce_k_buf[7];
	wire ik_y0_l = ce_k_buf[8];
	wire ik_y0_h = ce_k_buf[9];
	wire ik_x1_l = ce_k_buf[10];
	wire ik_x1_h = ce_k_buf[11];
	wire ik_y1_l = ce_k_buf[12];
	wire ik_y1_h = ce_k_buf[13];

	/* S_SD enable buffers (split sd_off into 4 + sd_ctl1 into 4 leaves for <= 17 loads each) */
	wire ce_sd_raw = r_st_sd;
	wire [14:0] ce_sd_buf;
	genvar g_sdi;
	generate
		for (g_sdi = 0; g_sdi < 15; g_sdi = g_sdi + 1) begin : g_ce_sd
			assign ce_sd_buf[g_sdi] = ce_sd_raw;
		end
	endgenerate
	wire sd_sl0  = ce_sd_buf[0];
	wire sd_sl1  = ce_sd_buf[1];
	wire sd_s00  = ce_sd_buf[2];
	wire sd_s01  = ce_sd_buf[3];
	wire sd_s10  = ce_sd_buf[4];
	wire sd_s11  = ce_sd_buf[5];
	wire sd_ctl0 = ce_sd_buf[6];
	wire sd_ctl1 = ce_sd_buf[7];
	wire sd_off0 = ce_sd_buf[8];
	wire sd_off1 = ce_sd_buf[9];
	wire sd_off2 = ce_sd_buf[10];
	wire sd_off3 = ce_sd_buf[11];
	wire sd_rpx  = ce_sd_buf[12];
	wire sd_rspx = ce_sd_buf[13];
	wire sd_rrem = ce_sd_buf[14];

	/* S_WR enable buffers */
	wire [3:0] wg_buf;
	wire [4:0] sl_buf;
	genvar g_wgi;
	generate
		for (g_wgi = 0; g_wgi < 4; g_wgi = g_wgi + 1) begin : g_ce_wg
			assign wg_buf[g_wgi] = wr_go;
		end
		for (g_wgi = 0; g_wgi < 5; g_wgi = g_wgi + 1) begin : g_ce_wsl
			assign sl_buf[g_wgi] = seg_last;
		end
	endgenerate
	wire wg_px = wg_buf[0], wg_spx = wg_buf[1], wg_rem = wg_buf[2], wg_ctl = wg_buf[3];
	wire sl_px = sl_buf[0], sl_spx = sl_buf[1], sl_rem = sl_buf[2], sl_ctl = sl_buf[3], sl_row = sl_buf[4];
	wire wg_row = wg_buf[3];

	/* 1. S_CLIP: 15 leaves, each driving <= 17 FFs */
	wire ce_clip_raw = r_st_clip;
	wire [1:0] ce_clip_m;
	wire [14:0] ce_clip_l;
	genvar g_cm, g_cl;
	generate
		for (g_cm = 0; g_cm < 2; g_cm = g_cm + 1) begin : g_ce_cm
			assign ce_clip_m[g_cm] = ce_clip_raw;
		end
		for (g_cl = 0; g_cl < 15; g_cl = g_cl + 1) begin : g_ce_cl
			assign ce_clip_l[g_cl] = ce_clip_m[g_cl / 8];
		end
	endgenerate
	wire cl_w   = ce_clip_l[0];
	wire cl_wm1 = ce_clip_l[13];
	wire cl_r   = ce_clip_l[1];
	wire cl_sx  = ce_clip_l[2];
	wire cl_sy  = ce_clip_l[3];
	wire cl_stp = ce_clip_l[4];
	wire cl_ds0 = ce_clip_l[5];
	wire cl_ds1 = ce_clip_l[6];
	wire cl_ss0 = ce_clip_l[7];
	wire cl_ss1 = ce_clip_l[8];
	wire cl_db0 = ce_clip_l[9];
	wire cl_db1 = ce_clip_l[10];
	wire cl_sb0 = ce_clip_l[11];
	wire cl_sb1 = ce_clip_l[12];
	wire cl_tile8 = ce_clip_l[14];

	/* 2. S_START: 16 leaves, each driving <= 19 FFs */
	wire ce_start_raw = r_st_start;
	wire [3:0] ce_start_m;
	wire [15:0] ce_start_l;
	genvar g_sm, g_sl;
	generate
		for (g_sm = 0; g_sm < 4; g_sm = g_sm + 1) begin : g_ce_sm
			assign ce_start_m[g_sm] = ce_start_raw;
		end
		for (g_sl = 0; g_sl < 16; g_sl = g_sl + 1) begin : g_ce_sl
			assign ce_start_l[g_sl] = ce_start_m[g_sl / 4];
		end
	endgenerate
	wire st_px  = ce_start_l[0];
	wire st_spx = ce_start_l[1];
	wire st_db0 = ce_start_l[2];
	wire st_db1 = ce_start_l[3];
	wire st_sb0 = ce_start_l[4];
	wire st_sb1 = ce_start_l[5];
	wire st_dn0 = ce_start_l[6];
	wire st_dn1 = ce_start_l[7];
	wire st_sn0 = ce_start_l[8];
	wire st_sn1 = ce_start_l[9];
	wire st_df0 = ce_start_l[10];
	wire st_df1 = ce_start_l[11];
	wire st_sf0 = ce_start_l[12];
	wire st_sf1 = ce_start_l[13];
	wire st_yd  = ce_start_l[14];
	wire st_ys  = ce_start_l[15];

	/* 3. S_SA / S_SA1: leaves, each driving <= 16 FFs */
	wire ce_sa_raw  = r_st_sa && !fail_fsm0;
	wire ce_sa1_raw = r_st_sa1;
	wire [4:0] ce_sa_l;
	wire [4:0] ce_sa1_l;
	genvar g_sa_i;
	generate
		for (g_sa_i = 0; g_sa_i < 5; g_sa_i = g_sa_i + 1) begin : g_ce_sa
			assign ce_sa_l[g_sa_i] = ce_sa_raw;
			assign ce_sa1_l[g_sa_i] = ce_sa1_raw;
		end
	endgenerate
	wire sa_da0  = ce_sa_l[0];
	wire sa_da1  = ce_sa_l[1];
	wire sa_sa0  = ce_sa_l[2];
	wire sa_sa1  = ce_sa_l[3];
	wire sa_ctl  = ce_sa_l[4];
	wire sa1_da0 = ce_sa1_l[0];
	wire sa1_da1 = ce_sa1_l[1];
	wire sa1_sa0 = ce_sa1_l[2];
	wire sa1_sa1 = ce_sa1_l[3];
	wire sa1_ctl = ce_sa1_l[4];

	/* 4. S_SC: 4 leaves, each driving 16 FFs */
	wire ce_sc_raw = r_st_sc;
	wire [3:0] ce_sc_l;
	genvar g_sc_i;
	generate
		for (g_sc_i = 0; g_sc_i < 4; g_sc_i = g_sc_i + 1) begin : g_ce_sc
			assign ce_sc_l[g_sc_i] = ce_sc_raw;
		end
	endgenerate
	wire sc_snu  = r_st_sc;
	wire sc_dlo0 = ce_sc_l[0];
	wire sc_dlo1 = ce_sc_l[1];
	wire sc_slo0 = ce_sc_l[2];
	wire sc_slo1 = ce_sc_l[3];

	/* 5. S_INIT enable buffers */
	wire ce_init_raw = r_st_init_a;
	wire [1:0] ce_init_m;
	wire [7:0] ce_init_l;
	genvar g_inm, g_inl;
	generate
		for (g_inm = 0; g_inm < 2; g_inm = g_inm + 1) begin : g_ce_inm
			assign ce_init_m[g_inm] = ce_init_raw;
		end
		for (g_inl = 0; g_inl < 8; g_inl = g_inl + 1) begin : g_ce_inl
			assign ce_init_l[g_inl] = ce_init_m[g_inl / 4];
		end
	endgenerate
	wire in_da0 = 1'b1;
	wire in_da1 = 1'b1;
	wire in_sa0 = 1'b1;
	wire in_sa1 = 1'b1;

`else
	wire ce_kick_sim = r_st_idle && kick && op_ok;
	wire ik_cfg_copy = ce_kick_sim, ik_cfg_dtile = ce_kick_sim, ik_cfg_stile = ce_kick_sim;
	wire ik_cfg_psl0_0 = ce_kick_sim, ik_cfg_psl0_1 = ce_kick_sim, ik_cfg_psl0_2 = ce_kick_sim;
	wire ik_cfg_psl1 = ce_kick_sim, ik_cfg_ctl = ce_kick_sim;
	wire ik_x0_l = ce_kick_sim, ik_x0_h = ce_kick_sim, ik_y0_l = ce_kick_sim, ik_y0_h = ce_kick_sim;
	wire ik_x1_l = ce_kick_sim, ik_x1_h = ce_kick_sim, ik_y1_l = ce_kick_sim, ik_y1_h = ce_kick_sim;
	wire [3:0] ik_mr_buf = {4{ce_kick_sim}};
	wire sd_sl0 = r_st_sd, sd_sl1 = r_st_sd, sd_s00 = r_st_sd, sd_s01 = r_st_sd;
	wire sd_s10 = r_st_sd, sd_s11 = r_st_sd, sd_ctl0 = r_st_sd, sd_ctl1 = r_st_sd;
	wire sd_off0 = r_st_sd, sd_off1 = r_st_sd, sd_off2 = r_st_sd, sd_off3 = r_st_sd;
	wire sd_rpx = r_st_sd, sd_rspx = r_st_sd, sd_rrem = r_st_sd;
	wire wg_px = wr_go, wg_spx = wr_go, wg_rem = wr_go, wg_row = wr_go, wg_ctl = wr_go;
	wire sl_row = seg_last, sl_px = seg_last, sl_spx = seg_last, sl_rem = seg_last, sl_ctl = seg_last;
	wire cl_w = r_st_clip, cl_wm1 = r_st_clip, cl_r = r_st_clip, cl_sx = r_st_clip, cl_sy = r_st_clip, cl_stp = r_st_clip, cl_tile8 = r_st_clip;
	wire cl_ds0 = r_st_clip, cl_ds1 = r_st_clip, cl_ss0 = r_st_clip, cl_ss1 = r_st_clip;
	wire cl_db0 = r_st_clip, cl_db1 = r_st_clip, cl_sb0 = r_st_clip, cl_sb1 = r_st_clip;
	wire st_px = r_st_start, st_spx = r_st_start;
	wire st_db0 = r_st_start, st_db1 = r_st_start, st_sb0 = r_st_start, st_sb1 = r_st_start;
	wire st_dn0 = r_st_start, st_dn1 = r_st_start, st_sn0 = r_st_start, st_sn1 = r_st_start;
	wire st_df0 = r_st_start, st_df1 = r_st_start, st_sf0 = r_st_start, st_sf1 = r_st_start, st_yd = r_st_start, st_ys = r_st_start;
	wire sa_da0 = r_st_sa && !fail_fsm0, sa_da1 = r_st_sa && !fail_fsm0;
	wire sa_sa0 = r_st_sa && !fail_fsm0, sa_sa1 = r_st_sa && !fail_fsm0, sa_ctl = r_st_sa && !fail_fsm0;
	wire sa1_da0 = r_st_sa1, sa1_da1 = r_st_sa1, sa1_sa0 = r_st_sa1, sa1_sa1 = r_st_sa1, sa1_ctl = r_st_sa1;
	wire sc_snu = r_st_sc, sc_dlo0 = r_st_sc, sc_dlo1 = r_st_sc, sc_slo0 = r_st_sc, sc_slo1 = r_st_sc;
	wire [7:0] ce_init_l = {8{r_st_init_a}};
	wire in_da0 = r_st_init_a, in_da1 = r_st_init_a, in_sa0 = r_st_init_a, in_sa1 = r_st_init_a;
`endif
	wire mb_pos = 1'b1, mb_row = 1'b1, mb_addr = 1'b1, mb_ctl = 1'b1;

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			x0 <= 17'd0;
			y0 <= 17'd0;
			x1 <= 17'd0;
			y1 <= 17'd0;
		end else begin
			if (ik_x0_l) x0[8:0]  <= clip_x0[8:0];
			if (ik_x0_h) x0[16:9] <= clip_x0[16:9];
			if (ik_y0_l) y0[8:0]  <= clip_y0[8:0];
			if (ik_y0_h) y0[16:9] <= clip_y0[16:9];
			if (ik_x1_l) x1[8:0]  <= clip_x1[8:0];
			if (ik_x1_h) x1[16:9] <= clip_x1[16:9];
			if (ik_y1_l) y1[8:0]  <= clip_y1[8:0];
			if (ik_y1_h) y1[16:9] <= clip_y1[16:9];
		end
	end

`ifdef SYNTH_KINTEX7
	wire is_s_mul = r_st_mul_b;
	wire ce_ydl_init_raw = is_s_mul && ds_idle_b && m_burst;
	wire ce_ydl_init_lo = ce_ydl_init_raw;
	wire ce_ydl_init_hi = ce_ydl_init_raw;
	wire ce_ydl_init = !state_b4_0;
	wire ce_ydl;
	LUT4 #(.INIT(16'hFF40)) u_ce_ydl (
		.I0(cmp_last_bo),
		.I1(rem_b_small),
		.I2(walk_step_a),
		.I3(ce_ydl_init_lo),
		.O(ce_ydl)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire is_s_mul = r_st_mul_b;
	wire ce_ydl_init_raw = is_s_mul && ds_idle_b && m_burst;
	wire ce_ydl_init_lo = ce_ydl_init_raw;
	wire ce_ydl_init_hi = ce_ydl_init_raw;
	wire ce_ydl_init = !state_b4_0;
	wire ce_ydl;
	LUT4 #(.INIT(16'hFF40)) u_ce_ydl (
		.I0(cmp_last_bo),
		.I1(rem_b_small),
		.I2(walk_step_a),
		.I3(ce_ydl_init_lo),
		.O(ce_ydl)
	);
`else
	wire ce_ydl_init = r_st_mul_b && d_idle && s_idle && m_burst;
	wire ce_ydl_init_hi = ce_ydl_init;
	wire ce_ydl_step = r_st_burst && rst_ctl1;
	wire ce_ydl = ce_ydl_init || ce_ydl_step;
`endif
	wire [1:0] ydl_b_next_val = ce_ydl_init ? yd_start[1:0] : ydl_b_inc;
	wire [1:0] ysl_b_next_val = ce_ydl_init ? ys_start[1:0] : ysl_b_inc;

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			ydl_b <= 2'd0;
			ysl_b <= 2'd0;
			ydl_b_is_3_lo   <= 1'b0;
			ydl_b_is_3_hi_a <= 1'b0;
			ydl_b_is_3_hi_b <= 1'b0;
			ydl_b_is_3_cur  <= 1'b0;
			ysl_b_is_3_lo   <= 1'b0;
			ysl_b_is_3_hi_a <= 1'b0;
			ysl_b_is_3_hi_b <= 1'b0;
			ysl_b_is_3_cur  <= 1'b0;
			r_delta_px      <= {(US+1){1'b0}};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_b_dst_delta   <= {ADDR_W{1'b0}};
			r_b_src_delta   <= {ADDR_W{1'b0}};
`endif
		end else begin
			if (ce_ydl) begin
				ydl_b <= ydl_b_next_val;
				ysl_b <= ysl_b_next_val;
				ydl_b_is_3_lo   <= (ydl_b_next_val == 2'd3);
				ydl_b_is_3_hi_a <= (ydl_b_next_val == 2'd3);
				ydl_b_is_3_hi_b <= (ydl_b_next_val == 2'd3);
				ydl_b_is_3_cur  <= (ydl_b_next_val == 2'd3);
				ysl_b_is_3_lo   <= (ysl_b_next_val == 2'd3);
				ysl_b_is_3_hi_a <= (ysl_b_next_val == 2'd3);
				ysl_b_is_3_hi_b <= (ysl_b_next_val == 2'd3);
				ysl_b_is_3_cur  <= (ysl_b_next_val == 2'd3);
				r_delta_px      <= (ydl_b_next_val == 2'd3) ? r_delta_px_back : r_delta_px_norm;
			end
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			if (r_st_init0) begin
				r_b_dst_delta <= ydl_b_is_3_lo ? r_dst_delta_back : r_dst_delta_norm;
				r_b_src_delta <= ysl_b_is_3_lo ? r_src_delta_back : r_src_delta_norm;
			end else if (ce_ydl_step) begin
				r_b_dst_delta <= (ydl_b == 2'd2) ? r_dst_delta_back : r_dst_delta_norm;
				r_b_src_delta <= (ysl_b == 2'd2) ? r_src_delta_back : r_src_delta_norm;
			end
`endif
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			dlo <= {ADDR_W{1'b0}};
			slo <= {ADDR_W{1'b0}};
		end else begin
			if (sc_dlo0) dlo[15:0] <= mr_dlo_lo ? d_lo[15:0] : da[15:0];
			if (sc_dlo1) dlo[ADDR_W-1:16] <= mr_dlo_hi ? d_lo[ADDR_W-1:16] : da[ADDR_W-1:16];
			if (sc_slo0) slo[15:0] <= mr_slo_lo ? s_lo[15:0] : sa[15:0];
			if (sc_slo1) slo[ADDR_W-1:16] <= mr_slo_hi ? s_lo[ADDR_W-1:16] : sa[ADDR_W-1:16];
		end
	end

	(* dont_touch = "true", equivalent_register_removal = "no" *) reg r_mr_walk_lo0, r_mr_walk_lo1;
	reg r_smear_x;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			m_copy <= 1'b0;
			m_copy_a <= 1'b0;
			m_copy_b <= 1'b0;
			m_blend <= 1'b0;
			m_rev <= 1'b0;
			r_mr_px <= 1'b0;
			r_mr_px_m <= 1'b0;
			r_mr_px_hi <= 1'b0;
			r_mr_spx <= 1'b0;
			r_mr_spx_m <= 1'b0;
			r_mr_spx_hi <= 1'b0;
			r_mr_walk_lo0 <= 1'b0;
			r_mr_walk_lo1 <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			m_copy_wd_rep <= 8'd0;
			m_blend_wd_rep <= 8'd0;
			m_copy_ctl_r <= 1'b0;
`endif
		end else if (ik_cfg_copy) begin
			m_copy <= o_op != GHCI_BLT_FILL;
			m_copy_a <= o_op != GHCI_BLT_FILL;
			m_copy_b <= o_op != GHCI_BLT_FILL;
			m_blend <= (ENABLE_BLEND != 0) && o_blend;
			m_rev <= o_rev;
			r_mr_px <= o_rev;
			r_mr_px_m <= o_rev;
			r_mr_px_hi <= o_rev;
			r_mr_spx <= o_rev;
			r_mr_spx_m <= o_rev;
			r_mr_spx_hi <= o_rev;
			r_mr_walk_lo0 <= o_rev;
			r_mr_walk_lo1 <= o_rev;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			m_copy_wd_rep <= {8{o_op != GHCI_BLT_FILL}};
			m_blend_wd_rep <= {8{(ENABLE_BLEND != 0) && o_blend}};
			m_copy_ctl_r <= o_op != GHCI_BLT_FILL;
`endif
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_sb_eq   <= 8'd0;
			r_smear_x <= 1'b0;
		end else if (r_st_start) begin
			r_sb_eq[0] <= (r_dbase[3:0]   == r_sbase[3:0]);
			r_sb_eq[1] <= (r_dbase[7:4]   == r_sbase[7:4]);
			r_sb_eq[2] <= (r_dbase[11:8]  == r_sbase[11:8]);
			r_sb_eq[3] <= (r_dbase[15:12] == r_sbase[15:12]);
			r_sb_eq[4] <= (r_dbase[19:16] == r_sbase[19:16]);
			r_sb_eq[5] <= (r_dbase[23:20] == r_sbase[23:20]);
			r_sb_eq[6] <= (r_dbase[27:24] == r_sbase[27:24]);
			r_sb_eq[7] <= (r_dbase[31:28] == r_sbase[31:28]);
			r_smear_x  <= ({1'b0, s_src_xy[15:0]} < x0);
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			m_dtile <= 1'b0;
		end else if (ik_cfg_dtile) begin
			m_dtile <= (ENABLE_TILE != 0) && (o_op == GHCI_BLT_TILE);
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			m_stile <= 1'b0;
		end else if (ik_cfg_stile) begin
			m_stile <= (ENABLE_TILE != 0) && (o_op == GHCI_BLT_UNTILE);
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			psl <= 3'd2;
		end else begin
			if (ik_cfg_psl0_0) psl[0] <= o_psl[0];
			if (ik_cfg_psl0_1) psl[1] <= o_psl[1];
			if (ik_cfg_psl0_2) psl[2] <= o_psl[2];
		end
	end

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	reg [4:0] r_pslw_oh;
`endif
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			psl_w <= 3'd2;
			psl_leg_r <= 3'd2;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_pslw_oh <= 5'b00100;
`endif
		end else if (ik_cfg_psl1) begin
			psl_w <= o_psl;
			psl_leg_r <= o_psl;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_pslw_oh <= {o_psl == 3'd4, o_psl == 3'd3, o_psl == 3'd2, o_psl == 3'd1, o_psl == 3'd0};
`endif
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			m_keep <= 4'd0;
		end else if (ik_cfg_ctl) begin
			m_keep <= o_keep;
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			m_burst <= 1'b0;
		end else if (ik_cfg_ctl) begin
			m_burst <= !o_blend;
		end else if (r_st_start1) begin
			m_burst <= m_burst && !((&r_sb_eq) && !mr_leg_a && r_smear_x);
		end
	end

	reg [US:0] r_delta_px_norm, r_delta_px_back;
`ifdef SYNTH_KINTEX7
	wire [US:0] dpx_norm_pos = (pslw_ctl0 == 3'd4) ? {5'b00000, b_dst_delta_norm[US-1:4]} :
	                           (pslw_ctl0 == 3'd3) ? {4'b0000, b_dst_delta_norm[US-1:3]}  :
	                           (pslw_ctl0 == 3'd2) ? {3'b000, b_dst_delta_norm[US-1:2]}   :
	                           (pslw_ctl0 == 3'd1) ? {2'b00, b_dst_delta_norm[US-1:1]}    : {1'b0, b_dst_delta_norm[US-1:0]};
	wire [US:0] dpx_back_pos = (pslw_ctl1 == 3'd4) ? {5'b00000, b_dst_delta_back[US-1:4]} :
	                           (pslw_ctl1 == 3'd3) ? {4'b0000, b_dst_delta_back[US-1:3]}  :
	                           (pslw_ctl1 == 3'd2) ? {3'b000, b_dst_delta_back[US-1:2]}   :
	                           (pslw_ctl1 == 3'd1) ? {2'b00, b_dst_delta_back[US-1:1]}    : {1'b0, b_dst_delta_back[US-1:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US:0] dpx_norm_pos = (pslw_ctl0 == 3'd4) ? {5'b00000, b_dst_delta_norm[US-1:4]} :
	                           (pslw_ctl0 == 3'd3) ? {4'b0000, b_dst_delta_norm[US-1:3]}  :
	                           (pslw_ctl0 == 3'd2) ? {3'b000, b_dst_delta_norm[US-1:2]}   :
	                           (pslw_ctl0 == 3'd1) ? {2'b00, b_dst_delta_norm[US-1:1]}    : {1'b0, b_dst_delta_norm[US-1:0]};
	wire [US:0] dpx_back_pos = (pslw_ctl1 == 3'd4) ? {5'b00000, b_dst_delta_back[US-1:4]} :
	                           (pslw_ctl1 == 3'd3) ? {4'b0000, b_dst_delta_back[US-1:3]}  :
	                           (pslw_ctl1 == 3'd2) ? {3'b000, b_dst_delta_back[US-1:2]}   :
	                           (pslw_ctl1 == 3'd1) ? {2'b00, b_dst_delta_back[US-1:1]}    : {1'b0, b_dst_delta_back[US-1:0]};
`else
	wire [4:0] pslw0_oh = r_pslw_oh;
	wire [4:0] pslw1_oh = r_pslw_oh;
	wire [US:0] dpx_neg_p  =
		({(US+1){pslw0_oh[4]}} & {5'b00000, r_dst_neg[US-1:4]}) |
		({(US+1){pslw0_oh[3]}} & {4'b0000,  r_dst_neg[US-1:3]}) |
		({(US+1){pslw0_oh[2]}} & {3'b000,   r_dst_neg[US-1:2]}) |
		({(US+1){pslw0_oh[1]}} & {2'b00,    r_dst_neg[US-1:1]}) |
		({(US+1){pslw0_oh[0]}} & {1'b0,     r_dst_neg[US-1:0]});
	wire [US:0] dpx_tile_p =
		({(US+1){pslw0_oh[4]}} & {5'b00000, r_tile_step[US-1:4]}) |
		({(US+1){pslw0_oh[3]}} & {4'b0000,  r_tile_step[US-1:3]}) |
		({(US+1){pslw0_oh[2]}} & {3'b000,   r_tile_step[US-1:2]}) |
		({(US+1){pslw0_oh[1]}} & {2'b00,    r_tile_step[US-1:1]}) |
		({(US+1){pslw0_oh[0]}} & {1'b0,     r_tile_step[US-1:0]});
	wire [US:0] dpx_str_p  =
		({(US+1){pslw0_oh[4]}} & {5'b00000, ext_dstride[US-1:4]}) |
		({(US+1){pslw0_oh[3]}} & {4'b0000,  ext_dstride[US-1:3]}) |
		({(US+1){pslw0_oh[2]}} & {3'b000,   ext_dstride[US-1:2]}) |
		({(US+1){pslw0_oh[1]}} & {2'b00,    ext_dstride[US-1:1]}) |
		({(US+1){pslw0_oh[0]}} & {1'b0,     ext_dstride[US-1:0]});
	wire [US:0] dpx_back_t_p =
		({(US+1){pslw1_oh[4]}} & {5'b00000, r_dst_back[US-1:4]}) |
		({(US+1){pslw1_oh[3]}} & {4'b0000,  r_dst_back[US-1:3]}) |
		({(US+1){pslw1_oh[2]}} & {3'b000,   r_dst_back[US-1:2]}) |
		({(US+1){pslw1_oh[1]}} & {2'b00,    r_dst_back[US-1:1]}) |
		({(US+1){pslw1_oh[0]}} & {1'b0,     r_dst_back[US-1:0]});
`endif
	wire [US:0] dpx_norm_neg, dpx_back_neg;
	wire [1:0] dpx_neg_co_unused;
`ifdef SYNTH_KINTEX7
	wire dpxn_c0, dpxn_h0, dpxn_h1, dpxb_c0, dpxb_h0, dpxb_h1;
	wire [US-3:0] dpxn_hi0, dpxn_hi1, dpxb_hi0, dpxb_hi1;
	am_add #(.W(3))    a_dpx_nn_0  (.a(~dpx_norm_pos[2:0]),  .b(3'd0),            .ci(1'b1), .s(dpx_norm_neg[2:0]), .co(dpxn_c0));
	am_add #(.W(US-2)) a_dpx_nn_h0 (.a(~dpx_norm_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b0), .s(dpxn_hi0),          .co(dpxn_h0));
	am_add #(.W(US-2)) a_dpx_nn_h1 (.a(~dpx_norm_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b1), .s(dpxn_hi1),          .co(dpxn_h1));
	assign dpx_norm_neg[US:3] = dpxn_c0 ? dpxn_hi1 : dpxn_hi0;
	assign dpx_neg_co_unused[0] = (dpxn_c0 ? dpxn_h1 : dpxn_h0) & dpxn_h0;

	am_add #(.W(3))    a_dpx_bn_0  (.a(~dpx_back_pos[2:0]),  .b(3'd0),            .ci(1'b1), .s(dpx_back_neg[2:0]), .co(dpxb_c0));
	am_add #(.W(US-2)) a_dpx_bn_h0 (.a(~dpx_back_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b0), .s(dpxb_hi0),          .co(dpxb_h0));
	am_add #(.W(US-2)) a_dpx_bn_h1 (.a(~dpx_back_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b1), .s(dpxb_hi1),          .co(dpxb_h1));
	assign dpx_back_neg[US:3] = dpxb_c0 ? dpxb_hi1 : dpxb_hi0;
	assign dpx_neg_co_unused[1] = (dpxb_c0 ? dpxb_h1 : dpxb_h0) & dpxb_h0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire dpxn_c0, dpxn_h0, dpxn_h1, dpxb_c0, dpxb_h0, dpxb_h1;
	wire [US-3:0] dpxn_hi0, dpxn_hi1, dpxb_hi0, dpxb_hi1;
	am_add #(.W(3))    a_dpx_nn_0  (.a(~dpx_norm_pos[2:0]),  .b(3'd0),            .ci(1'b1), .s(dpx_norm_neg[2:0]), .co(dpxn_c0));
	am_add #(.W(US-2)) a_dpx_nn_h0 (.a(~dpx_norm_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b0), .s(dpxn_hi0),          .co(dpxn_h0));
	am_add #(.W(US-2)) a_dpx_nn_h1 (.a(~dpx_norm_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b1), .s(dpxn_hi1),          .co(dpxn_h1));
	assign dpx_norm_neg[US:3] = dpxn_c0 ? dpxn_hi1 : dpxn_hi0;
	assign dpx_neg_co_unused[0] = (dpxn_c0 ? dpxn_h1 : dpxn_h0) & dpxn_h0;

	am_add #(.W(3))    a_dpx_bn_0  (.a(~dpx_back_pos[2:0]),  .b(3'd0),            .ci(1'b1), .s(dpx_back_neg[2:0]), .co(dpxb_c0));
	am_add #(.W(US-2)) a_dpx_bn_h0 (.a(~dpx_back_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b0), .s(dpxb_hi0),          .co(dpxb_h0));
	am_add #(.W(US-2)) a_dpx_bn_h1 (.a(~dpx_back_pos[US:3]), .b({(US-2){1'b0}}),  .ci(1'b1), .s(dpxb_hi1),          .co(dpxb_h1));
	assign dpx_back_neg[US:3] = dpxb_c0 ? dpxb_hi1 : dpxb_hi0;
	assign dpx_neg_co_unused[1] = (dpxb_c0 ? dpxb_h1 : dpxb_h0) & dpxb_h0;
`else
	wire [US:0] dpx_neg_n, dpx_tile_n, dpx_str_n, dpx_back_t_n;
	assign dpx_neg_n[0]    = dpx_neg_p[0];
	assign dpx_tile_n[0]   = dpx_tile_p[0];
	assign dpx_str_n[0]    = dpx_str_p[0];
	assign dpx_back_t_n[0] = dpx_back_t_p[0];
	genvar g_dpx;
	generate
		for (g_dpx = 1; g_dpx <= US; g_dpx = g_dpx + 1) begin : g_dpx_neg_fast
			assign dpx_neg_n[g_dpx]    = dpx_neg_p[g_dpx]    ^ ((|dpx_neg_p[g_dpx-1:0])    & !m_rev);
			assign dpx_tile_n[g_dpx]   = dpx_tile_p[g_dpx]   ^ ((|dpx_tile_p[g_dpx-1:0])   & !m_rev);
			assign dpx_str_n[g_dpx]    = dpx_str_p[g_dpx]    ^ ((|dpx_str_p[g_dpx-1:0])    & !m_rev);
			assign dpx_back_t_n[g_dpx] = dpx_back_t_p[g_dpx] ^ ((|dpx_back_t_p[g_dpx-1:0]) & !m_rev);
		end
	endgenerate
	assign dpx_norm_neg =
		({(US+1){r_bd_nm_neg}}  & dpx_neg_n)  |
		({(US+1){r_bd_nm_tile}} & dpx_tile_n) |
		({(US+1){r_bd_nm_str}}  & dpx_str_n);
	assign dpx_back_neg =
		({(US+1){r_bd_bk_neg}}  & dpx_neg_n)    |
		({(US+1){r_bd_bk_tile}} & dpx_back_t_n) |
		({(US+1){r_bd_bk_str}}  & dpx_str_n);
	wire dpx_src_unused_keep = 1'b0;
	assign dpx_neg_co_unused = 2'b00;
`endif
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_dst_delta_norm <= {ADDR_W{1'b0}};
			r_dst_delta_back <= {ADDR_W{1'b0}};
			r_src_delta_norm <= {ADDR_W{1'b0}};
			r_src_delta_back <= {ADDR_W{1'b0}};
			r_delta_px_norm <= {(US+1){1'b0}};
			r_delta_px_back <= {(US+1){1'b0}};
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_bd_bk_neg  <= 1'b0;
			r_bd_bk_tile <= 1'b0;
			r_bd_bk_str  <= 1'b0;
			r_bd_nm_neg  <= 1'b0;
			r_bd_nm_tile <= 1'b0;
			r_bd_nm_str  <= 1'b0;
`endif
		end else begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			if (r_st_start) begin
				r_bd_bk_neg  <= bd_bk_neg;
				r_bd_bk_tile <= bd_bk_tile;
				r_bd_bk_str  <= bd_bk_str;
				r_bd_nm_neg  <= bd_nm_neg;
				r_bd_nm_tile <= bd_nm_tile;
				r_bd_nm_str  <= bd_nm_str;
			end
`endif
			if (r_st_start1) begin
`ifdef SYNTH_KINTEX7
				r_delta_px_norm <= m_rev ? dpx_norm_pos : dpx_norm_neg;
				r_delta_px_back <= m_rev ? dpx_back_pos : dpx_back_neg;
`elsif SYNTH_KINTEX_ULTRASCALE
				r_delta_px_norm <= m_rev ? dpx_norm_pos : dpx_norm_neg;
				r_delta_px_back <= m_rev ? dpx_back_pos : dpx_back_neg;
`else
				r_delta_px_norm <= dpx_norm_neg;
				r_delta_px_back <= dpx_back_neg;
`endif
			end
			if (ce_init_l[0]) r_dst_delta_norm[15:0] <= b_dst_delta_norm[15:0];
			if (ce_init_l[1]) r_dst_delta_norm[ADDR_W-1:16] <= b_dst_delta_norm[ADDR_W-1:16];
			if (ce_init_l[2]) r_dst_delta_back[15:0] <= b_dst_delta_back[15:0];
			if (ce_init_l[3]) r_dst_delta_back[ADDR_W-1:16] <= b_dst_delta_back[ADDR_W-1:16];
			if (ce_init_l[4]) r_src_delta_norm[15:0] <= b_src_delta_norm[15:0];
			if (ce_init_l[5]) r_src_delta_norm[ADDR_W-1:16] <= b_src_delta_norm[ADDR_W-1:16];
			if (ce_init_l[6]) r_src_delta_back[15:0] <= b_src_delta_back[15:0];
			if (ce_init_l[7]) r_src_delta_back[ADDR_W-1:16] <= b_src_delta_back[ADDR_W-1:16];
		end
	end

	/*
	 * Dedicated Clock Enable and Registers for 'rows'
	 */
	wire ce_rows_s_clip = r_st_clip_b;
	wire ce_rows_s_wr   = r_st_wr && (!fail_fsm3 && wg_ctl && sl_ctl);
	wire ce_rows_raw    = ce_rows_s_clip || ce_rows_s_wr;
	wire [16:0] rows_mux = {
		r_st_clip_b_mux1 ? c_rows[16:9] : rows_dec[16:9],
		r_st_clip_b_mux0 ? c_rows[8:0]  : rows_dec[8:0]
	};
	wire [15:0] c_yeq_x = ~(y1[16:1] ^ y0[16:1]);
	wire [15:0] c_yeq_o = y1[16:1] | (~y0[16:1]);
	wire [14:0] c_yeq_mm = c_yeq_x[15:1] ^ c_yeq_o[14:0];
	(* keep = "true" *) wire [4:0] c_yeq_g = {
		(c_yeq_mm[14:12] == 3'd0),
		(c_yeq_mm[11:9]  == 3'd0),
		(c_yeq_mm[8:6]   == 3'd0),
		(c_yeq_mm[5:3]   == 3'd0),
		(c_yeq_mm[2:0]   == 3'd0) && (c_yeq_x[0] == 1'b0) && (y1[0] == y0[0])
	};
	wire c_rows_eq_1    = (&c_yeq_g);
	wire rows_dec_eq_1  = (rows == 17'd2);
	wire r_rows_eq_1_d  = r_st_clip_b ? c_rows_eq_1 : rows_dec_eq_1;

	wire ce_rows_r_lo = ce_rows_raw;
	wire ce_rows_r_hi = ce_rows_raw;

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rows <= 17'd0;
			r_rows_eq_1 <= 1'b0;
		end else begin
			if (ce_rows_r_lo) rows[7:0]  <= rows_mux[7:0];
			if (ce_rows_r_hi) rows[16:8] <= rows_mux[16:8];
			if (ce_rows_raw)  r_rows_eq_1 <= r_rows_eq_1_d;
		end
	end

	/*
	 * Dedicated Process for 'is_burst'
	 */
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			is_burst <= 1'b0;
		end else begin
			if (r_st_init1) begin
				is_burst <= 1'b1;
			end else if (r_st_burst) begin
				if (fail_burst || burst_drain_done)
					is_burst <= 1'b0;
			end else if (r_st_drain || (r_st_idle && !kick) || (r_st_mul_b && ds_idle_a && !m_burst)) begin
				is_burst <= 1'b0;
			end
		end
	end

	/*
	 * Dedicated Process for 'cur_bst_room' (closed-form 1-LUT min3 + pre-registered r_room_last)
	 */
	wire ce_bst_room = r_st_init1 || (is_burst && ws_walk);
	wire [US:0] r_init_bst_room_w = (bst_tiled && !r_rem_line_init_ge8 && (r_rem_line_init[2:0] < bst_room_t_init[2:0])) ?
		r_rem_line_init : (bst_tiled ? bst_room_t_init : r_rem_line_init);
	wire [US:0] rl_mask_k;
	genvar g_rkm;
	generate
		for (g_rkm = 0; g_rkm < US; g_rkm = g_rkm + 1) begin : g_rkm_bits
			assign rl_mask_k[g_rkm] = |line_pixels_b[US:g_rkm+1];
		end
	endgenerate
	assign rl_mask_k[US] = 1'b0;
`ifdef SYNTH_KINTEX7
	wire [US-3:0] rl_hi0_m = rlr_s_hi0 & rl_mask_k[US:3];
	wire [US-3:0] rl_hi1_m = rlr_s_hi1 & rl_mask_k[US:3];
	wire rl_hi0_nz = |rl_hi0_m;
	wire rl_hi1_nz = |rl_hi1_m;
	wire rl_lo_nz  = |({rlr_s_lo[2] & rl_mask_k[2], rlr_s_lo[1:0]});
	wire rlr_ge4_0 = rl_hi0_nz | (rlr_s_lo[2] & rl_mask_k[2]) | (!rl_lo_nz && !line_pixels_b[1]);
	wire rlr_ge4_1 = rl_hi1_nz | (rlr_s_lo[2] & rl_mask_k[2]) | (!rl_lo_nz && !line_pixels_b[1]);
	wire [2:0] room_last_t3_0, room_last_t3_1;
	assign room_last_t3_0[2] = tile_room_init[2] & rlr_ge4_0;
	assign room_last_t3_0[1] = tile_room_init[2] ? (rlr_ge4_0 ? 1'b0 : rem_line_row_next_0[1]) :
		(rlr_ge4_0 ? tile_room_init[1] : (tile_room_init[1] & rem_line_row_next_0[1]));
	assign room_last_t3_0[0] = tile_room_init[2] ? (rlr_ge4_0 ? 1'b0 : rem_line_row_next_0[0]) :
		(rlr_ge4_0 ? tile_room_init[0] :
		((!tile_room_init[1] & tile_room_init[0]) | (!rem_line_row_next_0[1] & rem_line_row_next_0[0]) |
		(tile_room_init[0] & rem_line_row_next_0[0])));
	assign room_last_t3_1[2] = tile_room_init[2] & rlr_ge4_1;
	assign room_last_t3_1[1] = tile_room_init[2] ? (rlr_ge4_1 ? 1'b0 : rem_line_row_next_1[1]) :
		(rlr_ge4_1 ? tile_room_init[1] : (tile_room_init[1] & rem_line_row_next_1[1]));
	assign room_last_t3_1[0] = tile_room_init[2] ? (rlr_ge4_1 ? 1'b0 : rem_line_row_next_1[0]) :
		(rlr_ge4_1 ? tile_room_init[0] :
		((!tile_room_init[1] & tile_room_init[0]) | (!rem_line_row_next_1[1] & rem_line_row_next_1[0]) |
		(tile_room_init[0] & rem_line_row_next_1[0])));
	wire [US:0] r_room_last_w0 = bst_tiled ? {{(US - 2){1'b0}}, room_last_t3_0} : rem_line_row_next_0;
	wire [US:0] r_room_last_w1 = bst_tiled ? {{(US - 2){1'b0}}, room_last_t3_1} : rem_line_row_next_1;
	wire [US:0] r_room_last_w  = rlr_s_co0 ? r_room_last_w1 : r_room_last_w0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [US-3:0] rl_hi0_m = rlr_s_hi0 & rl_mask_k[US:3];
	wire [US-3:0] rl_hi1_m = rlr_s_hi1 & rl_mask_k[US:3];
	wire rl_hi0_nz = |rl_hi0_m;
	wire rl_hi1_nz = |rl_hi1_m;
	wire rl_lo_nz  = |({rlr_s_lo[2] & rl_mask_k[2], rlr_s_lo[1:0]});
	wire rlr_ge4_0 = rl_hi0_nz | (rlr_s_lo[2] & rl_mask_k[2]) | (!rl_lo_nz && !line_pixels_b[1]);
	wire rlr_ge4_1 = rl_hi1_nz | (rlr_s_lo[2] & rl_mask_k[2]) | (!rl_lo_nz && !line_pixels_b[1]);
	wire [2:0] room_last_t3_0, room_last_t3_1;
	assign room_last_t3_0[2] = tile_room_init[2] & rlr_ge4_0;
	assign room_last_t3_0[1] = tile_room_init[2] ? (rlr_ge4_0 ? 1'b0 : rem_line_row_next_0[1]) :
		(rlr_ge4_0 ? tile_room_init[1] : (tile_room_init[1] & rem_line_row_next_0[1]));
	assign room_last_t3_0[0] = tile_room_init[2] ? (rlr_ge4_0 ? 1'b0 : rem_line_row_next_0[0]) :
		(rlr_ge4_0 ? tile_room_init[0] :
		((!tile_room_init[1] & tile_room_init[0]) | (!rem_line_row_next_0[1] & rem_line_row_next_0[0]) |
		(tile_room_init[0] & rem_line_row_next_0[0])));
	assign room_last_t3_1[2] = tile_room_init[2] & rlr_ge4_1;
	assign room_last_t3_1[1] = tile_room_init[2] ? (rlr_ge4_1 ? 1'b0 : rem_line_row_next_1[1]) :
		(rlr_ge4_1 ? tile_room_init[1] : (tile_room_init[1] & rem_line_row_next_1[1]));
	assign room_last_t3_1[0] = tile_room_init[2] ? (rlr_ge4_1 ? 1'b0 : rem_line_row_next_1[0]) :
		(rlr_ge4_1 ? tile_room_init[0] :
		((!tile_room_init[1] & tile_room_init[0]) | (!rem_line_row_next_1[1] & rem_line_row_next_1[0]) |
		(tile_room_init[0] & rem_line_row_next_1[0])));
	wire [US:0] r_room_last_w0 = bst_tiled ? {{(US - 2){1'b0}}, room_last_t3_0} : rem_line_row_next_0;
	wire [US:0] r_room_last_w1 = bst_tiled ? {{(US - 2){1'b0}}, room_last_t3_1} : rem_line_row_next_1;
	wire [US:0] r_room_last_w  = rlr_s_co0 ? r_room_last_w1 : r_room_last_w0;
`else
	wire rl_hi0_nz = |(rl_hi0_mm & r_rl_mask_k[US:2]);
	wire rl_hi1_nz = |((~rlr_ab_x[US:2]) & r_rl_mask_k[US:2]);
	wire rl_lo2_nz = |rlr_s_lo2;
	wire rl_lo2_force_ge4 = !rl_lo2_nz && !line_pixels_b[1];
	wire rlr_ge4_0 = rl_hi0_nz | rl_lo2_force_ge4;
	wire rlr_ge4_1 = rl_hi1_nz | rl_lo2_force_ge4;
	wire rlr_ge4   = rlr_c2 ? rlr_ge4_1 : rlr_ge4_0;
	wire [1:0] rlr_lo2_tile = {line_pixels_b[1] ? (~rlr_s_lo2[0]) : rlr_s_lo2[1], rlr_s_lo2[0]};
	wire [1:0] rlt_lt4;
	assign rlt_lt4[1] = tile_room_init[2] ? rlr_lo2_tile[1] : (tile_room_init[1] & rlr_lo2_tile[1]);
	assign rlt_lt4[0] = tile_room_init[2] ? rlr_lo2_tile[0] :
		((!tile_room_init[1] & tile_room_init[0]) | (!rlr_lo2_tile[1] & rlr_lo2_tile[0]) |
		(tile_room_init[0] & rlr_lo2_tile[0]));
	wire [1:0] rlt_fallback_lo2 = bst_tiled ? rlt_lt4 : rem_line_row_next[1:0];
	wire rlr_ge4_tiled = bst_tiled && rlr_ge4;
	wire [2:0] room_last_t3 = {tile_room_init[2] & rlr_ge4, rlr_ge4_tiled ? tile_room_init[1:0] : rlt_fallback_lo2};
	wire [US:0] r_room_last_w = {
		bst_tiled ? {{(US - 2){1'b0}}, room_last_t3[2]} : rem_line_row_next[US:2],
		room_last_t3[1:0]
	};
	wire rl_mask_unused = &{1'b0, rl_mask_k};
`endif
	wire [US:0] bst_room_t_walk = {{(US - 2){1'b0}}, tile_room_walk};
	/* Pure 3-bit LUT subtraction (zero CARRY4) for rem_line_cap3 when bst_tiled == 1 (cur_bst_room <= 4). */
	wire [2:0] rl_sub3_w;
	assign rl_sub3_w[0] = rem_line_b[0] ^ cur_bst_room_b[0];
	assign rl_sub3_w[1] = rem_line_b[1] ^ cur_bst_room_b[1] ^ (~rem_line_b[0] & cur_bst_room_b[0]);
	assign rl_sub3_w[2] = rem_line_b[2] ^ cur_bst_room_b[2] ^
		((~rem_line_b[1] & cur_bst_room_b[1]) | ((~(rem_line_b[1] ^ cur_bst_room_b[1])) & (~rem_line_b[0] & cur_bst_room_b[0])));
	wire rl_eq3_w = (rem_line_b[2:0] == cur_bst_room_b[2:0]);
`ifdef SYNTH_KINTEX7
	wire [2:0] rem_line_cap3 = rem_line_b_ge8 ? 3'd4 :
		(rl_eq3_w ? r_line_cap3 : (rl_sub3_w[2] ? 3'd4 : rl_sub3_w));
	wire [2:0] room_next_t3;
	assign room_next_t3[2] = tile_room_walk[2] & rem_line_cap3[2];
	assign room_next_t3[1] = tile_room_walk[2] ? rem_line_cap3[1] :
		(rem_line_cap3[2] ? tile_room_walk[1] : (tile_room_walk[1] & rem_line_cap3[1]));
	assign room_next_t3[0] = tile_room_walk[2] ? rem_line_cap3[0] :
		(rem_line_cap3[2] ? tile_room_walk[0] :
		((!tile_room_walk[1] & tile_room_walk[0]) | (!rem_line_cap3[1] & rem_line_cap3[0]) |
		(tile_room_walk[0] & rem_line_cap3[0])));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] rem_line_cap3 = rem_line_b_ge8 ? 3'd4 :
		(rl_eq3_w ? r_line_cap3 : (rl_sub3_w[2] ? 3'd4 : rl_sub3_w));
	wire [2:0] room_next_t3;
	assign room_next_t3[2] = tile_room_walk[2] & rem_line_cap3[2];
	assign room_next_t3[1] = tile_room_walk[2] ? rem_line_cap3[1] :
		(rem_line_cap3[2] ? tile_room_walk[1] : (tile_room_walk[1] & rem_line_cap3[1]));
	assign room_next_t3[0] = tile_room_walk[2] ? rem_line_cap3[0] :
		(rem_line_cap3[2] ? tile_room_walk[0] :
		((!tile_room_walk[1] & tile_room_walk[0]) | (!rem_line_cap3[1] & rem_line_cap3[0]) |
		(tile_room_walk[0] & rem_line_cap3[0])));
`else
	/* Flatten tile_room_walk and rem_line_cap3 in parallel, then single min3 + single one-hot mux into cur_bst_room. */
	wire rnt_go = !rem_line_b_ge8;
	wire [1:0] rnt_go_cap = {rnt_go & r_line_cap3[1], rnt_go & r_line_cap3[0]};
	wire [1:0] rnt_sub_lt4 = {!rl_sub3_w[2] & rl_sub3_w[1], !rl_sub3_w[2] & rl_sub3_w[0]};
	wire [2:0] rnt_rlc3 = {
		rem_line_b_ge8 | (rl_eq3_w & r_line_cap3[2]) | rl_sub3_w[2],
		(rl_eq3_w & rnt_go_cap[1]) | (rnt_go & rnt_sub_lt4[1]),
		(rl_eq3_w & rnt_go_cap[0]) | (rnt_go & rnt_sub_lt4[0])
	};
	wire [2:0] rnt_tw3 = mdt_ctl_b ? td_walk : ts_walk;
	wire [2:0] rnt_bp1 = {cur_bst_room_b[2] ^ (cur_bst_room_b[1] & cur_bst_room_b[0]), cur_bst_room_b[1] ^ cur_bst_room_b[0], ~cur_bst_room_b[0]};
	wire [2:0] rnt_bp3 = {cur_bst_room_b[2] ^ (cur_bst_room_b[1] | cur_bst_room_b[0]), ~(cur_bst_room_b[1] ^ cur_bst_room_b[0]), ~cur_bst_room_b[0]};
	wire rnt_sub_is1 = (rem_line_b[2:0] == rnt_bp1);
	wire rnt_sub_is3 = (rem_line_b[2:0] == rnt_bp3);
	wire rnt_b_is1 = rnt_go & (rl_eq3_w ? (r_line_cap3 == 3'd1) : rnt_sub_is1);
	wire rnt_b_is3 = rnt_go & (rl_eq3_w ? (r_line_cap3 == 3'd3) : rnt_sub_is3);
	wire rnt_a_is1 = mdt_ctl_b ? (&px_2b_next) : (&spx_2b_next);
	wire rnt_a_ne2 = mdt_ctl_b ? (!px_2b_next[1] | px_2b_next[0]) : (!spx_2b_next[1] | spx_2b_next[0]);
	wire [2:0] room_next_t3;
	assign room_next_t3[2] = rnt_tw3[2] & rnt_rlc3[2];
	assign room_next_t3[1] = (!rnt_b_is1 & rnt_tw3[1]) | (rnt_tw3[2] & rnt_rlc3[1]);
	assign room_next_t3[0] = rnt_a_is1 | rnt_b_is1 | (rnt_rlc3[2] & rnt_tw3[0]) | (rnt_b_is3 & rnt_a_ne2);
	wire rnt_rlc0_unused = &{1'b0, rnt_rlc3[0]};
`endif
	wire [US:0] room_next = bst_tiled ? {{(US - 2){1'b0}}, room_next_t3} : line_pixels_b;
	wire [US:0] room_last = r_room_last;
`ifdef SYNTH_KINTEX7
	wire bsl_room_fast;
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hF800)) u_bsl_room_fast (
		.I0(cmp_lo_le),
		.I1(cmp_hi_eq),
		.I2(cmp_hi_lt),
		.I3(rem_b_small_bsl),
		.O(bsl_room_fast)
	);
	wire [US:0] room_fallback = r_is_init1 ? r_init_bst_room : r_room_last;
	wire [US:0] next_bst_room = (r_is_init1 || bsl_room_fast) ? room_fallback : room_next;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bsl_room_fast;
	(* dont_touch = "true" *) LUT4 #(.INIT(16'hF800)) u_bsl_room_fast (
		.I0(cmp_lo_le),
		.I1(cmp_hi_eq),
		.I2(cmp_hi_lt),
		.I3(rem_b_small_bsl),
		.O(bsl_room_fast)
	);
	wire [US:0] room_fallback = r_is_init1 ? r_init_bst_room : r_room_last;
	wire [US:0] next_bst_room = (r_is_init1 || bsl_room_fast) ? room_fallback : room_next;
`else
	wire bsl_room_fast = bsl_room;
	wire use_fb_room = r_is_init1 || bsl_room_fast;
	wire [US:0] room_fallback = r_is_init1 ? r_init_bst_room : r_room_last;
	wire [US:0] rnt_alt = use_fb_room ? room_fallback : line_pixels_b;
	wire take_tiled = !use_fb_room && bst_tiled;
	wire [US:0] next_bst_room = {
		take_tiled ? {(US - 2){1'b0}} : rnt_alt[US:3],
		take_tiled ? room_next_t3 : rnt_alt[2:0]
	};
	wire rnt_t3_unused = &{1'b0, room_next};
`endif

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			cur_bst_room <= {(US+1){1'b0}};
			cur_bst_room_b <= {(US+1){1'b0}};
		end else if (ce_bst_room) begin
			cur_bst_room <= next_bst_room;
			cur_bst_room_b <= next_bst_room;
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_init_bst_room <= {(US+1){1'b0}};
		end else if (r_st_init0) begin
			r_init_bst_room <= r_init_bst_room_w;
		end
	end

	/*
	 * Dedicated Process for 'r_leg_room'
	 */
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_leg_room <= {(US+1){1'b0}};
		end else if (r_st_sb) begin
			r_leg_room <= leg_room;
		end
	end

	/*
	 * Dedicated Processes for High-Frequency Pipeline (shared FPGA & ASIC)
	 */
`ifdef SYNTH_KINTEX7
	wire ce_rem_line_row;
	LUT4 #(.INIT(16'hFF40)) u_ce_rlr (
		.I0(cmp_last_bo),
		.I1(rem_b_small_ctl),
		.I2(walk_step_a),
		.I3(r_is_init1),
		.O(ce_rem_line_row)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire ce_rem_line_row;
	LUT4 #(.INIT(16'hFF40)) u_ce_rlr (
		.I0(cmp_last_bo),
		.I1(rem_b_small_ctl),
		.I2(walk_step_a),
		.I3(r_is_init1),
		.O(ce_rem_line_row)
	);
`else
	wire ce_rem_line_row = r_is_init1 || (walk_step_a && rem_b_small_ctl && !cmp_last_bo);
`endif
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rem_line_row <= {(US+1){1'b0}};
			r_room_last <= {(US+1){1'b0}};
		end else if (r_st_init0) begin
			rem_line_row <= r_rem_line_init;
		end else if (ce_rem_line_row) begin
			rem_line_row <= rem_line_row_next;
			r_room_last <= r_room_last_w;
		end
	end

	wire ce_cdr_init = ((r_st_mul_a && mb_row) || r_st_init_a);
	wire [US-1:0] cur_da_row_d = ce_cdr_init ? cur_da_init : cur_da_row_next;
`ifdef SYNTH_KINTEX7
	wire ce_cur_da_row;
	LUT5 #(.INIT(32'hFFFF_FF40)) u_ce_cdr (
		.I0(cmp_last_bo),
		.I1(rem_b_small_ctl),
		.I2(walk_step_a),
		.I3(r_is_init1),
		.I4(ce_cdr_init),
		.O(ce_cur_da_row)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire ce_cur_da_row;
	LUT5 #(.INIT(32'hFFFF_FF40)) u_ce_cdr (
		.I0(cmp_last_bo),
		.I1(rem_b_small_ctl),
		.I2(walk_step_a),
		.I3(r_is_init1),
		.I4(ce_cdr_init),
		.O(ce_cur_da_row)
	);
`else
	wire ce_cur_da_row = ce_cdr_init || r_is_init1 || (walk_step_a && rem_b_small_ctl && !cmp_last_bo);
`endif
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			cur_da_row <= {US{1'b0}};
`ifdef SYNTH_KINTEX7
			r_da_px_init <= {(US+1){1'b0}};
`elsif SYNTH_KINTEX_ULTRASCALE
			r_da_px_init <= {(US+1){1'b0}};
`endif
			r_is_init1 <= 1'b0;
		end else begin
			r_is_init1 <= r_st_init0;
			if (ce_cur_da_row) begin
				cur_da_row <= cur_da_row_d;
			end
`ifdef SYNTH_KINTEX7
			if (ce_cdr_init) begin
				r_da_px_init <= (psl_ctl == 3'd4) ? {5'b00000, cur_da_init[US-1:4]} :
				                (psl_ctl == 3'd3) ? {4'b0000, cur_da_init[US-1:3]}  :
				                (psl_ctl == 3'd2) ? {3'b000, cur_da_init[US-1:2]}   :
				                (psl_ctl == 3'd1) ? {2'b00, cur_da_init[US-1:1]}    : {1'b0, cur_da_init};
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (ce_cdr_init) begin
				r_da_px_init <= (psl_ctl == 3'd4) ? {5'b00000, cur_da_init[US-1:4]} :
				                (psl_ctl == 3'd3) ? {4'b0000, cur_da_init[US-1:3]}  :
				                (psl_ctl == 3'd2) ? {3'b000, cur_da_init[US-1:2]}   :
				                (psl_ctl == 3'd1) ? {2'b00, cur_da_init[US-1:1]}    : {1'b0, cur_da_init};
			end
`endif
		end
	end

	wire ce_rows_init = ce_ydl_init_hi && mb_row;
`ifdef SYNTH_KINTEX7
	wire ce_rows_b_lo, ce_rows_b_hi;
	LUT4 #(.INIT(16'hFF40)) u_ce_rows_b_lo (.I0(cmp_last_bo), .I1(rem_b_small_ctl), .I2(walk_step_a), .I3(ce_rows_init), .O(ce_rows_b_lo));
	LUT4 #(.INIT(16'hFF40)) u_ce_rows_b_hi (.I0(cmp_last_bo), .I1(rem_b_small_ctl), .I2(walk_step_a), .I3(ce_rows_init), .O(ce_rows_b_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire ce_rows_b_lo, ce_rows_b_hi;
	LUT4 #(.INIT(16'hFF40)) u_ce_rows_b_lo (.I0(cmp_last_bo), .I1(rem_b_small_ctl), .I2(walk_step_a), .I3(ce_rows_init), .O(ce_rows_b_lo));
	LUT4 #(.INIT(16'hFF40)) u_ce_rows_b_hi (.I0(cmp_last_bo), .I1(rem_b_small_ctl), .I2(walk_step_a), .I3(ce_rows_init), .O(ce_rows_b_hi));
`else
	wire ce_rows_b_lo = ce_rows_init || (walk_step_a && rem_b_small_ctl && !cmp_last_bo);
	wire ce_rows_b_hi = ce_rows_b_lo;
`endif

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rows_b[8:0] <= 9'd0;
		end else if (ce_rows_b_lo) begin
			rows_b[8:0] <= ce_rows_init ? rows[8:0] : rows_b_dec[8:0];
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rows_b[16:9] <= 8'd0;
		end else if (ce_rows_b_hi) begin
			rows_b[16:9] <= ce_rows_init ? rows[16:9] : rows_b_dec[16:9];
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rows_b_zero  <= 1'b0;
		end else if (r_st_init_a) begin
			rows_b_zero  <= rows_is_zero;
		end else if (walk_step_a && bsl_ctl) begin
			rows_b_zero  <= (rows_b == 17'd1);
		end
	end

	wire ce_rbs_init = r_st_mul_a && ds_idle_b && m_burst;
	wire ce_rbs_walk = walk_step_b;
	wire ce_rem_b_small = ce_rbs_init || ce_rbs_walk;
	wire width_small = !(|width[16:US+1]);
`ifdef SYNTH_KINTEX7
	wire rem_b_small_walk_fast;
	LUT6 #(.INIT(64'hFFFFFF0080808080)) u_rbs_walk (
		.I0(rem_b_cand_hi), .I1(rem_b_cand_lo), .I2(rem_borrow_hi),
		.I3(cmp_last_bo), .I4(width_small), .I5(rem_b_small_bsl),
		.O(rem_b_small_walk_fast)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rem_b_small_walk_fast;
	LUT6 #(.INIT(64'hFFFFFF0080808080)) u_rbs_walk (
		.I0(rem_b_cand_hi), .I1(rem_b_cand_lo), .I2(rem_borrow_hi),
		.I3(cmp_last_bo), .I4(width_small), .I5(rem_b_small_bsl),
		.O(rem_b_small_walk_fast)
	);
`else
	wire rem_b_small_walk_fast = rem_b_small_bsl ? (cmp_last_bo ? 1'b1 : width_small) :
	                             (rem_b_cand_hi && rem_b_cand_lo && rem_borrow_hi);
`endif
	wire rem_b_small_next = ce_rbs_init ? width_small : rem_b_small_walk_fast;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			rem_b_small     <= 1'b0;
			rem_b_small_bsl <= 1'b0;
			rem_b_small_ctl <= 1'b0;
			rem_b_small_rs  <= 1'b0;
		end else if (ce_rem_b_small) begin
			rem_b_small     <= rem_b_small_next;
			rem_b_small_bsl <= rem_b_small_next;
			rem_b_small_ctl <= rem_b_small_next;
			rem_b_small_rs  <= rem_b_small_next;
		end
	end

`ifdef SYNTH_KINTEX7

	wire ce_sb1_lo, ce_sb1_hi;
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ce_sb1_lo (.I0(r_st_sb1), .O(ce_sb1_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ce_sb1_hi (.I0(r_st_sb1), .O(ce_sb1_hi));
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_leg_back <= {ADDR_W{1'b0}};
		end else begin
			if (ce_sb1_lo) r_leg_back[15:0] <= leg_back_val[15:0];
			if (ce_sb1_hi) r_leg_back[ADDR_W-1:16] <= leg_back_val[ADDR_W-1:16];
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE

	wire ce_sb1_lo, ce_sb1_hi;
	assign ce_sb1_lo = r_st_sb1;
	assign ce_sb1_hi = r_st_sb1;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			r_leg_back <= {ADDR_W{1'b0}};
		end else begin
			if (ce_sb1_lo) r_leg_back[15:0] <= leg_back_val[15:0];
			if (ce_sb1_hi) r_leg_back[ADDR_W-1:16] <= leg_back_val[ADDR_W-1:16];
		end
	end
`endif

	reg fail_fsm_r;
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high)
			fail_fsm_r <= 1'b0;
		else if (r_st_idle)
			fail_fsm_r <= 1'b0;
		else if (fail_raw)
			fail_fsm_r <= 1'b1;
	end
	wire leg_crd_done = q_free_leg && wr_idle && !rd_first && !rd_second;
	wire fail_drain_state = r_st_sa || r_st_crd || r_st_cwait ||
		r_st_drd || r_st_dwait || r_st_wr ||
		(r_st_burst && !burst_drain_done);
	wire force_drain_fsm = fail_fsm_r && fail_drain_state;

`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [4:0] st_g0_r, st_g1_r, st_g2_r, st_g3_r, st_g4_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [4:0] st_g0_r, st_g1_r, st_g2_r, st_g3_r, st_g4_r;
`else
	reg [4:0] st_g0_r, st_g1_r, st_g2_r, st_g3_r, st_g4_r;
`endif
	always @(*) begin
		case (state_ns[1:0])
		2'd0: st_g0_r = (kick && op_ok) ? S_CLIP : S_IDLE;
		2'd1: st_g0_r = S_IDLE;
		2'd2: st_g0_r = S_START1;
		2'd3: st_g0_r = ds_idle_a ? (m_burst ? S_INIT : S_SA) : S_MUL;
		endcase
		case (state_ns[1:0])
		2'd0: st_g1_r = S_SA1;
		2'd1: st_g1_r = S_SB1;
		2'd2: st_g1_r = S_SC;
		2'd3: st_g1_r = S_SD;
		endcase
		case (state_ns[1:0])
		2'd0: st_g2_r = m_copy_a ? S_CRD : (m_blend ? S_DRD : S_WR);
		2'd1: st_g2_r = leg_crd_done ? S_CWAIT : S_CRD;
		2'd2: st_g2_r = (q_free_fsm && rd_idle) ? (m_blend ? S_DRD : S_WR) : S_CWAIT;
		2'd3: st_g2_r = (q_free_fsm && wr_idle) ? S_DWAIT : S_DRD;
		endcase
		case (state_ns[1:0])
		2'd0: st_g3_r = (q_free_fsm && rd_idle) ? S_BLEND : S_DWAIT;
		2'd1: st_g3_r = bl_done ? S_WR : S_BLEND;
		2'd2: st_g3_r = q_free_fsm ? ((r_seg_last && rows_is_zero) ? S_DRAIN : S_SA) : S_WR;
		2'd3: st_g3_r = (q_free_fsm && wr_idle && rd_idle && !rsp_valid_fsm) ? S_IDLE : S_DRAIN;
		endcase
		case (state_ns[1:0])
		2'd0: st_g4_r = burst_drain_done ? S_IDLE : S_BURST;
		2'd1: st_g4_r = S_INIT0;
		2'd2: st_g4_r = S_MUL;
		2'd3: st_g4_r = S_BURST;
		endcase
	end
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire [4:0] st_g0 = st_g0_r;
	(* keep = "true" *) wire [4:0] st_g1 = st_g1_r;
	(* keep = "true" *) wire [4:0] st_g2 = st_g2_r;
	(* keep = "true" *) wire [4:0] st_g3 = st_g3_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire [4:0] st_g0 = st_g0_r;
	(* keep = "true" *) wire [4:0] st_g1 = st_g1_r;
	(* keep = "true" *) wire [4:0] st_g2 = st_g2_r;
	(* keep = "true" *) wire [4:0] st_g3 = st_g3_r;
`else
	wire [4:0] st_g0 = st_g0_r;
	wire [4:0] st_g1 = st_g1_r;
	wire [4:0] st_g2 = st_g2_r;
	wire [4:0] st_g3 = st_g3_r;
`endif
	wire [4:0] st_g4 = (state_ns[3:2] == 2'b00) ? st_g4_r :
	                   ((state_ns[3:2] == 2'b01) ? (state_ns[1:0] == 2'd0 ? S_SB : S_INIT1) : S_IDLE);
	wire [4:0] st_q01 = (state_ns[3:2] == 2'd0) ? st_g0 :
	                                        (state_ns[3:2] == 2'd1) ? st_g1 :
	                                        (state_ns[3:2] == 2'd2) ? st_g2 : st_g3;
	wire [4:0] state_next_norm = state_ns[4] ? st_g4 : st_q01;
	wire [4:0] state_next_no_clip = force_drain_fsm ? S_DRAIN : state_next_norm;

	reg busy_next_no_clip;
	reg done_next_no_clip;
	always @(*) begin
		busy_next_no_clip = r_busy_q;
		done_next_no_clip = 1'b0;
		if (r_st_idle) begin
			if (kick && op_ok)
				busy_next_no_clip = 1'b1;
		end else if (r_st_clip) begin
			busy_next_no_clip = 1'b0;
			done_next_no_clip = 1'b0;
		end else if (r_st_burst) begin
			if (burst_drain_done) begin
				busy_next_no_clip = 1'b0;
				done_next_no_clip = 1'b1;
			end
		end else if (r_st_drain) begin
			if (q_free_fsm && wr_idle && rd_idle && !rsp_valid_fsm) begin
				busy_next_no_clip = 1'b0;
				if (!fail_fsm3)
					done_next_no_clip = 1'b1;
			end
		end
	end

	wire is_s_clip = r_st_clip;
	wire state_d1_nc = state_next_no_clip[1];
	wire busy_d_nc   = busy_next_no_clip;
	wire done_d_nc   = done_next_no_clip;
`ifdef SYNTH_KINTEX7
	wire state_d1;
	wire busy_d;
	wire done_d;
	LUT6 #(.INIT(64'hFFFF_0000_1F10_1F10)) u_state_d1 (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(st_q01[1]),
		.I4(st_g4[1]),
		.I5(state[4]),
		.O(state_d1)
	);
	LUT5 #(.INIT(32'h1F10_0000)) u_busy_d (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(busy_d_nc),
		.I4(rst_n),
		.O(busy_d)
	);
	LUT5 #(.INIT(32'hEFE0_0000)) u_done_d (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(done_d_nc),
		.I4(rst_n),
		.O(done_d)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire state_d1;
	wire busy_d;
	wire done_d;
	LUT6 #(.INIT(64'hFFFF_0000_1F10_1F10)) u_state_d1 (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(st_q01[1]),
		.I4(st_g4[1]),
		.I5(state[4]),
		.O(state_d1)
	);
	LUT5 #(.INIT(32'h1F10_0000)) u_busy_d (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(busy_d_nc),
		.I4(rst_n),
		.O(busy_d)
	);
	LUT5 #(.INIT(32'hEFE0_0000)) u_done_d (
		.I0(cx_empty),
		.I1(cy_empty),
		.I2(is_s_clip),
		.I3(done_d_nc),
		.I4(rst_n),
		.O(done_d)
	);
`else
	wire state_d1 = is_s_clip ? (!cx_empty && !cy_empty) : state_d1_nc;
	wire busy_d   = is_s_clip ? (!cx_empty && !cy_empty) : busy_d_nc;
	wire done_d   = is_s_clip ? (cx_empty || cy_empty)   : done_d_nc;
`endif
	wire [4:0] st_d = {state_next_no_clip[4:2], state_d1, state_next_no_clip[0]};

	/* Direct 1-bit next-state equations for r_st_* FFs (cuts q_free -> r_st_* from 7 LUTs to <= 3 LUTs) */
	wire r_st_clip_next   = r_st_idle && kick && op_ok;
	wire r_st_start_next  = r_st_clip && !cx_empty && !cy_empty;
	wire r_st_start1_next = r_st_start;
	wire r_st_mul_next    = r_st_start1 || (r_st_mul_a && !ds_idle_a);
	wire r_st_init_next   = r_st_mul_a && ds_idle_b && m_burst;
	wire r_st_init0_next  = r_st_init_a;
	wire r_st_init1_next  = r_st_init0;
	wire r_st_burst_next  = r_st_init1 || (r_st_burst && !burst_drain_done && !fail_fsm_r);
	wire r_st_sa_next     = (r_st_mul_a && ds_idle_a && !m_burst) ||
	                        (r_st_wr && !fail_fsm_r && q_free_st && !(r_seg_last && rows_is_zero));
	wire r_st_sa1_next    = r_st_sa && !fail_fsm_r;
	wire r_st_sb_next     = r_st_sa1;
	wire r_st_sb1_next    = r_st_sb;
	wire r_st_sc_next     = r_st_sb1;
	wire r_st_sd_next     = r_st_sc;
	wire r_st_crd_next    = (r_st_sd && m_copy_a) || (r_st_crd && !fail_fsm_r && !leg_crd_done);
	wire r_st_cwait_next  = (r_st_crd && !fail_fsm_r && leg_crd_done) ||
	                        (r_st_cwait && !fail_fsm_r && !(q_free_st && rd_idle));
	wire r_st_drd_next    = (r_st_sd && !m_copy_a && m_blend) ||
	                        (r_st_cwait && !fail_fsm_r && q_free_st && rd_idle && m_blend) ||
	                        (r_st_drd && !fail_fsm_r && !(q_free_st && wr_idle));
	wire r_st_dwait_next  = (r_st_drd && !fail_fsm_r && q_free_st && wr_idle) ||
	                        (r_st_dwait && !fail_fsm_r && !(q_free_st && rd_idle));
	wire r_st_blend_next  = (r_st_dwait && !fail_fsm_r && q_free_st && rd_idle) || (r_st_blend && !bl_done);
	wire r_st_wr_next     = (r_st_sd && !m_copy_a && !m_blend) ||
	                        (r_st_cwait && !fail_fsm_r && q_free_st && rd_idle && !m_blend) ||
	                        (r_st_blend && bl_done) ||
	                        (r_st_wr && !fail_fsm_r && !q_free_st);
	wire r_st_drain_next  = (r_st_wr && !fail_fsm_r && q_free_st && r_seg_last && rows_is_zero) ||
	                        (r_st_drain && !(q_free_st && wr_idle && rd_idle && !rsp_valid_fsm)) ||
	                        force_drain_fsm;
	wire r_st_idle_other  = (r_st_idle && !(kick && op_ok)) ||
	                        (r_st_burst && burst_drain_done) ||
	                        (r_st_drain && q_free_st && wr_idle && rd_idle && !rsp_valid_fsm);
	wire r_st_idle_next   = r_st_clip ? (cx_empty || cy_empty) : r_st_idle_other;

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			state       <= S_IDLE;
			state_ctl   <= S_IDLE;
			state_mem   <= S_IDLE;
			state_leg   <= S_IDLE;
			state_pipe  <= S_IDLE;
			state_ns    <= S_IDLE;
			r_st_idle   <= 1'b1;
			r_st_clip   <= 1'b0;
			r_st_clip_b <= 1'b0;
`ifdef SYNTH_KINTEX7
			r_st_clip_b_mux0 <= 1'b0;
			r_st_clip_b_mux1 <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
			r_st_clip_b_mux0 <= 1'b0;
			r_st_clip_b_mux1 <= 1'b0;
`endif
			r_st_start  <= 1'b0;
			r_st_start1 <= 1'b0;
			r_st_mul_a  <= 1'b0;
			r_st_mul_b  <= 1'b0;
			r_st_init_a <= 1'b0;
			r_st_init_b <= 1'b0;
			r_st_init0  <= 1'b0;
			r_st_init1  <= 1'b0;
			r_st_sa     <= 1'b0;
			r_st_sa1    <= 1'b0;
			r_st_sb     <= 1'b0;
			r_st_sb1    <= 1'b0;
			r_st_sc     <= 1'b0;
			r_st_sd     <= 1'b0;
			r_st_crd    <= 1'b0;
			r_st_cwait  <= 1'b0;
			r_st_drd    <= 1'b0;
			r_st_dwait  <= 1'b0;
			r_st_blend  <= 1'b0;
			r_st_wr     <= 1'b0;
			r_st_drain  <= 1'b0;
			r_st_burst  <= 1'b0;
			r_busy_q    <= 1'b0;
			done        <= 1'b0;
		end else begin
			state           <= st_d;
			state_ctl       <= st_d;
			state_mem       <= st_d;
			state_leg       <= st_d;
			state_pipe      <= st_d;
			state_ns        <= st_d;
			r_st_idle       <= r_st_idle_next;
			r_st_clip       <= r_st_clip_next;
			r_st_clip_b     <= r_st_clip_next;
`ifdef SYNTH_KINTEX7
			r_st_clip_b_mux0 <= r_st_clip_next;
			r_st_clip_b_mux1 <= r_st_clip_next;
`elsif SYNTH_KINTEX_ULTRASCALE
			r_st_clip_b_mux0 <= r_st_clip_next;
			r_st_clip_b_mux1 <= r_st_clip_next;
`endif
			r_st_start      <= r_st_start_next;
			r_st_start1     <= r_st_start1_next;
			r_st_mul_a      <= r_st_mul_next;
			r_st_mul_b      <= r_st_mul_next;
			r_st_init_a     <= r_st_init_next;
			r_st_init_b     <= r_st_init_next;
			r_st_init0      <= r_st_init0_next;
			r_st_init1      <= r_st_init1_next;
			r_st_sa         <= r_st_sa_next;
			r_st_sa1        <= r_st_sa1_next;
			r_st_sb         <= r_st_sb_next;
			r_st_sb1        <= r_st_sb1_next;
			r_st_sc         <= r_st_sc_next;
			r_st_sd         <= r_st_sd_next;
			r_st_crd        <= r_st_crd_next;
			r_st_cwait      <= r_st_cwait_next;
			r_st_drd        <= r_st_drd_next;
			r_st_dwait      <= r_st_dwait_next;
			r_st_blend      <= r_st_blend_next;
			r_st_wr         <= r_st_wr_next;
			r_st_drain      <= r_st_drain_next;
			r_st_burst      <= r_st_burst_next;
			r_busy_q        <= busy_d;
			done            <= done_d;
		end
	end

	wire ce_ydl_leg = r_st_start || (r_st_wr && !fail_fsm3 && q_free_leg && r_seg_last);
	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			ydl <= 2'd0;
			ysl <= 2'd0;
		end else if (ce_ydl_leg) begin
			ydl <= r_st_wr ? ydl_inc : yd_start[1:0];
			ysl <= r_st_wr ? ysl_inc : ys_start[1:0];
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			error <= 1'b0;
			error_kind <= 4'd0;
			rd_first <= 1'b0;
			rd_second <= 1'b0;
			bl_go <= 1'b0;
			r_seg_last <= 1'b0;
			bw_bank <= 1'b0;
`ifdef SYNTH_KINTEX7
			bw_bank_dup <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
			bw_bank_dup <= 1'b0;
`endif
			bw_seg_idx <= 3'd0;
			bw_seg_onehot <= 8'b00000001;
			bw_slot_cnt <= 4'd0;
			r_bw_seg_last <= 1'b0;
			r_bw_slot_ge_lim1 <= 1'b0;
			r_bw_slot_ge_lim2 <= 1'b0;
			bw_sub <= 1'b0;
			bw_done_all <= 1'b0;
			bw_walk_done <= 1'b0;
			w_valid <= 1'b0;
			g_valid <= 1'b0;
			g2_valid <= 1'b0;
			br_bank <= 1'b0;
			br_seg_idx <= 3'd0;
			br_active <= 1'b0;
			b_draining <= 1'b0;
			bst_rd_err <= 1'b0;
			bank_active <= 2'b00;
			bank_closed <= 2'b00;
			bank_ready <= 2'b00;
			bank_avail <= 2'b11;
			bank_rd_pending_0 <= 5'd0;
			bank_rd_pending_1 <= 5'd0;
			burst_wr_out <= 5'd0;
			px_b <= 17'd0;
			spx_b <= 17'd0;
			rem_b <= 17'd0;
			tile_room_b <= 3'd4;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_leg_back <= {ADDR_W{1'b0}};
`endif
			r_da_px_leg <= {(US+1){1'b0}};
			r_room_tile3 <= 3'd0;
			r_snu_m1 <= {(US+1){1'b0}};
			r_d_xb_leg <= {ADDR_W{1'b0}};
			r_s_xb_leg <= {ADDR_W{1'b0}};
			r_dst_row_px <= {US{1'b0}};
			r_d_px_lo <= {US{1'b0}};
			r_px_next <= 17'd0;
			r_spx_next <= 17'd0;
			r_rem_next <= 17'd0;
			rem_line_b <= {(US+1){1'b0}};
			rem_line_b_ge8 <= 1'b0;
			r_rem_line_init <= {(US+1){1'b0}};
			r_rem_line_init_ge8 <= 1'b0;
			line_pixels_b <= {1'b1, {US{1'b0}}};
			r_line_pixels <= {1'b1, {US{1'b0}}};
			r_line_cap3 <= 3'd4;
			rows_is_zero <= 1'b0;
			dst_row_b <= {ADDR_W{1'b0}};
			src_row_b <= {ADDR_W{1'b0}};
			r_dst_back <= {ADDR_W{1'b0}};
			r_src_back <= {ADDR_W{1'b0}};
			r_dst_neg <= {ADDR_W{1'b0}};
			r_src_neg <= {ADDR_W{1'b0}};
			r_tile_step <= {ADDR_W{1'b0}};
			r_tile_back <= {ADDR_W{1'b0}};
			r_dstride <= 32'd0;
			r_sstride <= 32'd0;
			r_dbase <= 32'd0;
			r_sbase <= 32'd0;
			r_d_first <= {ADDR_W{1'b0}};
			r_s_first <= {ADDR_W{1'b0}};
			r_yd_start <= 17'd0;
			r_ys_start <= 17'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_rl_mask_k <= {(US+1){1'b0}};
`endif
			width <= 17'd0;
			width_m1 <= 17'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_src_m_dx <= 17'd0;
			r_src_m_dy <= 17'd0;
`endif
			sx0 <= 17'd0;
			sy0 <= 17'd0;
			px_row <= 17'd0;
			spx_row <= 17'd0;
			px <= 17'd0;
			spx <= 17'd0;
			rem <= 17'd0;
			sn <= {(US+1){1'b0}};
			snu <= {(US+1){1'b0}};
			seg_line <= {LA_W{1'b0}};
			src_n0 <= {LA_W{1'b0}};
			src_n1 <= {LA_W{1'b0}};
			seg_p0 <= {US{1'b0}};
			src_off <= {US{1'b0}};
			src_off_rep <= {(8*US){1'b0}};
			n0_slot <= 1'b0;
			da <= {ADDR_W{1'b0}};
			sa <= {ADDR_W{1'b0}};
		end else begin
			error <= 1'b0;
			bl_go <= 1'b0;

			/* Track pending read responses in ping-pong RAM */
			bank_rd_pending_0 <= brdp0_d;
			bank_rd_pending_1 <= brdp1_d;
			if (is_burst_ctl0) begin
				burst_wr_out <= bwr_next;
			end else begin
				burst_wr_out <= 5'd0;
			end

			if (m_burst && rsp_rd_bst && rsp_error) begin
				bst_rd_err <= 1'b1;
			end

			if (r_st_idle && kick) begin
				bst_rd_err <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				r_src_m_dx <= src_m_dx_w;
				r_src_m_dy <= src_m_dy_w;
`endif
				if (!op_ok) begin
					error <= 1'b1;
					error_kind <= GHCI_FAULT_KIND_INVALID_OP;
				end
			end

			/* S_CLIP updates (ce_clip_l already gated by r_st_clip) */
			if (cl_w)   width <= c_width;
			if (cl_wm1) width_m1 <= unused_x_d;
			if (cl_r) begin
				r_line_pixels <= calc_line_pixels;
				r_line_cap3 <= (|calc_line_pixels[US:2]) ? 3'd4 : {1'b0, calc_line_pixels[1:0]};
			end
			if (cl_sx)  sx0 <= c_sx0;
			if (cl_sy)  sy0 <= c_sy0;
			if (cl_stp) r_tile_step <= tile_step;
			if (cl_tile8) r_tile_back <= tile_back;
			if (cl_ds0) r_dstride[15:0] <= dstride_u[15:0];
			if (cl_ds1) r_dstride[31:16] <= dstride_u[31:16];
			if (cl_ss0) r_sstride[15:0] <= sstride_u[15:0];
			if (cl_ss1) r_sstride[31:16] <= sstride_u[31:16];
			if (cl_db0) r_dbase[15:0] <= dbase_u[15:0];
			if (cl_db1) r_dbase[31:16] <= dbase_u[31:16];
			if (cl_sb0) r_sbase[15:0] <= sbase_u[15:0];
			if (cl_sb1) r_sbase[31:16] <= sbase_u[31:16];

			/* S_START updates (ce_start_l already gated by r_st_start) */
			if (st_px) begin
				px_row <= mr_px_row ? x1_last : x0;
				rows_is_zero <= (rows == 17'd0);
			end
			if (st_spx) spx_row <= mr_spx_row ? s_last : sx0;
			if (st_yd)  r_yd_start <= mdt_acc ? {2'b00, yd_start[16:2]} : yd_start;
`ifdef SYNTH_KINTEX7
			if (st_ys)  r_ys_start <= mst_acc ? {2'b00, ys_start[16:2]} : ys_start;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (st_ys)  r_ys_start <= mst_acc ? {2'b00, ys_start[16:2]} : ys_start;
`else
			if (st_ys)  r_ys_start <= r_ys_start_d;
`endif
			if (st_df0) r_d_first[15:0] <= d_first[15:0];
			if (st_df1) r_d_first[ADDR_W-1:16] <= d_first[ADDR_W-1:16];
			if (st_sf0) r_s_first[15:0] <= s_first[15:0];
			if (st_sf1) r_s_first[ADDR_W-1:16] <= s_first[ADDR_W-1:16];
			if (st_db0) r_dst_back[15:0] <= dst_back[15:0];
			if (st_db1) r_dst_back[ADDR_W-1:16] <= dst_back[ADDR_W-1:16];
			if (st_sb0) r_src_back[15:0] <= src_back[15:0];
			if (st_sb1) r_src_back[ADDR_W-1:16] <= src_back[ADDR_W-1:16];
			if (st_dn0) r_dst_neg[15:0] <= dst_neg[15:0];
			if (st_dn1) r_dst_neg[ADDR_W-1:16] <= dst_neg[ADDR_W-1:16];
			if (st_sn0) r_src_neg[15:0] <= src_neg[15:0];
			if (st_sn1) r_src_neg[ADDR_W-1:16] <= src_neg[ADDR_W-1:16];

			/* S_MUL updates */
			if (r_st_mul_a && ds_idle_b && m_burst) begin
				if (mb_pos) begin
					px_b <= px_row;
					spx_b <= spx_row;
					rem_b <= width;
					tile_room_b <= tile_room_init;
				end
				if (mb_addr) begin
					dst_row_b <= dst_row;
					src_row_b <= src_row;
					line_pixels_b <= calc_line_pixels;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					r_rl_mask_k <= calc_rl_mask_k;
`endif
				end
			end
			if (r_st_mul_b && ds_idle_a && !m_burst) begin
				px <= px_row;
				spx <= spx_row;
				rem <= width;
			end

			/* S_INIT / S_INIT0 updates */
			if (r_st_init_a) begin
				rem_line_b <= rem_line_init;
				r_rem_line_init <= rem_line_init;
				r_rem_line_init_ge8 <= |rem_line_init[US:3];
				bw_bank <= 1'b0;
`ifdef SYNTH_KINTEX7
				bw_bank_dup <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
				bw_bank_dup <= 1'b0;
`endif
				bw_seg_idx <= 3'd0;
				bw_seg_onehot <= 8'b00000001;
				bw_slot_cnt <= 4'd0;
				r_bw_seg_last <= 1'b0;
				r_bw_slot_ge_lim1 <= 1'b0;
				r_bw_slot_ge_lim2 <= 1'b0;
				bw_sub <= 1'b0;
				bw_done_all <= 1'b0;
				bw_walk_done <= 1'b0;
				w_valid <= 1'b0;
				g_valid <= 1'b0;
				g2_valid <= 1'b0;
			end
			if (r_st_init_b) begin
				br_bank <= 1'b0;
				br_seg_idx <= 3'd0;
				br_active <= 1'b0;
				b_draining <= 1'b0;
				bst_rd_err <= 1'b0;
				bank_active <= 2'b00;
				bank_closed <= 2'b00;
				bank_ready <= 2'b00;
				bank_avail <= 2'b11;
			end
			if (r_st_init0) begin
				rem_line_b_ge8 <= r_rem_line_init_ge8;
			end

			/*
			 * =================================================================
			 * S_BURST: Streaming Burst DMA Engine (isolated from state_case)
			 * =================================================================
			 */
`ifdef SYNTH_KINTEX7
			if (is_burst_bk0 && bank_closed[0] && !bank_ready[0] && (!m_copy_b || brdp0_is_zero))
				bank_ready[0] <= 1'b1;
			if (is_burst_bk1 && bank_closed[1] && !bank_ready[1] && (!m_copy_b || brdp1_is_zero))
				bank_ready[1] <= 1'b1;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (is_burst_bk0 && bank_closed[0] && !bank_ready[0] && (!m_copy_b || brdp0_is_zero))
				bank_ready[0] <= 1'b1;
			if (is_burst_bk1 && bank_closed[1] && !bank_ready[1] && (!m_copy_b || brdp1_is_zero))
				bank_ready[1] <= 1'b1;
`else
			if (is_burst_bk0 && bank_closed[0] && !bank_ready[0] &&
			    (!m_copy_b || (brdp0_is_zero && !(rsp_rd_bst && !rsp_tag[4]) && !(pp_wr_en_r && !pp_wr_addr_r[4]))))
				bank_ready[0] <= 1'b1;
			if (is_burst_bk1 && bank_closed[1] && !bank_ready[1] &&
			    (!m_copy_b || (brdp1_is_zero && !(rsp_rd_bst && rsp_tag[4]) && !(pp_wr_en_r && pp_wr_addr_r[4]))))
				bank_ready[1] <= 1'b1;
`endif

			/* Walk & Geom & Issue Pipelines (ws_* and rst_* already include is_burst) */
			if (ws_px0)  px_b[15:0] <= px_b_next[15:0];
			if (ws_px1)  px_b[16]   <= px_b_next[16];
			if (ws_spx0) spx_b[15:0] <= spx_b_next[15:0];
			if (ws_spx1) spx_b[16]   <= spx_b_next[16];
			if (ws_rem0) rem_b[15:0] <= rem_b_next[15:0];
			if (ws_rem1) rem_b[16]   <= rem_b_next[16];
			if (ws_walk) begin
				rem_line_b <= rem_line_b_d;
				rem_line_b_ge8 <= rem_line_b_ge8_d;
				tile_room_b <= tile_room_step;
			end
			if (rst_dst0) dst_row_b[15:0] <= dst_row_b_next[15:0];
			if (rst_dst1) dst_row_b[ADDR_W-1:16] <= dst_row_b_next[ADDR_W-1:16];
			if (rst_src0) src_row_b[15:0] <= src_row_b_next[15:0];
			if (rst_src1) src_row_b[ADDR_W-1:16] <= src_row_b_next[ADDR_W-1:16];
			if (rst_ctl1 && rows_b_zero)
				bw_walk_done <= 1'b1;

			if (is_burst_ctl1) begin
				if (walk_step_b)
					w_valid <= 1'b1;
				else if (g_ready)
					w_valid <= 1'b0;
				if (g_step)
					g_valid <= 1'b1;
				else if (g2_ready)
					g_valid <= 1'b0;
				if (g2_step)
					g2_valid <= 1'b1;
				else if (step_p)
					g2_valid <= 1'b0;
			end

			/* Issue & Prefetch Stage */
			if (is_burst_ctl2) begin
				if (bst_can_fill_p)
					bank_active[bw_bank] <= 1'b1;

`ifdef SYNTH_KINTEX7
				if (issue_close_fire) begin
					bank_closed[bw_bank] <= 1'b1;
					bank_avail[bw_bank] <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
				if (issue_close_fire) begin
					bank_closed[bw_bank] <= 1'b1;
					bank_avail[bw_bank] <= 1'b0;
`else
				if (issue_close_0) begin
					bank_closed[0] <= 1'b1;
					bank_avail[0]  <= 1'b0;
				end
				if (issue_close_1) begin
					bank_closed[1] <= 1'b1;
					bank_avail[1]  <= 1'b0;
				end
				if (issue_close_fire) begin
`endif
					bw_bank <= ~bw_bank;
`ifdef SYNTH_KINTEX7
					bw_bank_dup <= ~bw_bank_dup;
`elsif SYNTH_KINTEX_ULTRASCALE
					bw_bank_dup <= ~bw_bank_dup;
`endif
					bw_seg_idx <= 3'd0;
					bw_slot_cnt <= 4'd0;
					r_bw_seg_last <= 1'b0;
					r_bw_slot_ge_lim1 <= 1'b0;
					r_bw_slot_ge_lim2 <= 1'b0;
					bw_seg_onehot <= 8'b00000001;
					bw_sub <= 1'b0;
					if (p_seg_last && p_rows_b_zero)
						bw_done_all <= 1'b1;
				end else if (issue_fire) begin
					bw_sub <= 1'b0;
					bw_seg_idx <= bw_seg_cnt_next[2:0];
					bw_slot_cnt <= p_two_lines ? bw_slot_p2 : bw_slot_p1;
					r_bw_seg_last <= (bw_seg_cnt_next[2:0] == MAX_SEGS_M1);
					r_bw_slot_ge_lim1 <= (LINE_SHIFT == 5) ?
						(p_two_lines ? (bw_slot_cnt >= 4'd13) : (bw_slot_cnt >= 4'd14)) :
						(p_two_lines ? (bw_slot_cnt >= 4'd5)  : (bw_slot_cnt >= 4'd6));
					r_bw_slot_ge_lim2 <= (LINE_SHIFT == 5) ?
						(p_two_lines ? (bw_slot_cnt >= 4'd12) : (bw_slot_cnt >= 4'd13)) :
						(p_two_lines ? (bw_slot_cnt >= 4'd4)  : (bw_slot_cnt >= 4'd5));
					bw_seg_onehot <= {bw_seg_onehot[6:0], bw_seg_onehot[7]};
				end else if (p_two_lines && !bw_sub && bst_rd_fire) begin
					bw_sub <= 1'b1;
				end
			end

			/* Commit & Write Channel */
			if (is_burst_ctl3) begin
				if (bst_rd_err) begin
					br_active <= 1'b0;
					b_draining <= 1'b1;
				end else if (!br_active) begin
					if (bank_ready[br_bank]) begin
						br_active <= 1'b1;
						br_seg_idx <= 3'd0;
					end
				end else if (br_step_en) begin
					if (br_seg_is_last) begin
						bank_active[br_bank] <= 1'b0;
						bank_closed[br_bank] <= 1'b0;
						bank_ready[br_bank] <= 1'b0;
						bank_avail[br_bank] <= 1'b1;
						br_bank <= ~br_bank;
						br_seg_idx <= 3'd0;
						if (bank_is_last[br_bank]) begin
							br_active <= 1'b0;
							b_draining <= 1'b1;
						end else begin
							br_active <= bank_ready[~br_bank];
						end
					end else begin
						br_seg_idx <= br_seg_cnt_next[2:0];
					end
				end
			end

			/*
			 * =================================================================
			 * Legacy Sequential Path (isolated from state_case)
			 * =================================================================
			 */
			if (sa_da0) r_d_xb_leg[15:0]        <= d_xb[15:0];
			if (sa_da1) r_d_xb_leg[ADDR_W-1:16] <= d_xb[ADDR_W-1:16];
			if (sa_sa0) r_s_xb_leg[15:0]        <= s_xb[15:0];
			if (sa_sa1) r_s_xb_leg[ADDR_W-1:16] <= s_xb[ADDR_W-1:16];
			if (sa_ctl) begin
				r_dst_row_px <= dst_row_px_sa;
				r_d_px_lo    <= d_px[US-1:0];
			end

			if (sa1_da0) da[15:0]        <= d_addr[15:0];
			if (sa1_da1) da[ADDR_W-1:16] <= d_addr[ADDR_W-1:16];
			if (sa1_sa0) sa[15:0]        <= s_addr[15:0];
			if (sa1_sa1) sa[ADDR_W-1:16] <= s_addr[ADDR_W-1:16];
			if (sa1_ctl) begin
				r_da_px_leg  <= da_px_s1;
				r_room_tile3 <= sa1_room_tile3;
			end

			if (r_st_sb1) begin
				sn <= leg_sn;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				r_leg_back <= leg_back_val;
`endif
				r_seg_last <= leg_rem_done;
			end

			if (sc_snu) begin
				snu      <= sc_snu_next;
				r_snu_m1 <= sc_snu_m1;
			end

			if (sd_sl0)  seg_line[13:0]        <= dlo_line[13:0];
			if (sd_sl1)  seg_line[LA_W-1:14]   <= dlo_line[LA_W-1:14];
			if (sd_s00)  src_n0[13:0]          <= slo_line[13:0];
			if (sd_s01)  src_n0[LA_W-1:14]     <= slo_line[LA_W-1:14];
			if (sd_s10)  src_n1[13:0]          <= last_line[13:0];
			if (sd_s11)  src_n1[LA_W-1:14]     <= last_line[LA_W-1:14];
			if (sd_ctl0) begin
				seg_p0 <= dlo[US-1:0];
				src_off <= src_base_u[US-1:0];
				n0_slot <= slo[US-1:0] < dlo[US-1:0];
				rd_first <= 1'b1;
			end
			if (sd_off0) src_off_rep[2*US-1:0]    <= {2{src_base_u[US-1:0]}};
			if (sd_off1) src_off_rep[4*US-1:2*US] <= {2{src_base_u[US-1:0]}};
			if (sd_off2) src_off_rep[6*US-1:4*US] <= {2{src_base_u[US-1:0]}};
			if (sd_off3) src_off_rep[8*US-1:6*US] <= {2{src_base_u[US-1:0]}};
			if (sd_rpx)  r_px_next  <= px_next;
			if (sd_rspx) r_spx_next <= spx_next;
			if (sd_rrem) r_rem_next <= rem_next;

			if (r_st_crd && !fail_fsm2 && q_free_leg && wr_idle) begin
				if (rd_first) begin
					rd_first <= 1'b0;
					rd_second <= src_n1 != src_n0;
				end else if (rd_second) begin
					rd_second <= 1'b0;
				end
			end

			if (r_st_dwait && !fail_fsm3 && q_free_leg && rd_idle) begin
				bl_go <= 1'b1;
			end

			/* S_WR updates (wg_* already include wr_go = r_st_wr && q_free && !fail_raw) */
			if (wg_ctl && sl_ctl) rows_is_zero <= r_rows_eq_1;
			if (wg_px)  px  <= sl_px  ? px_row  : r_px_next;
			if (wg_spx) spx <= sl_spx ? spx_row : r_spx_next;
			if (wg_rem) rem <= sl_rem ? width   : r_rem_next;

			if (r_st_drain && q_free_fsm && wr_idle && rd_idle && !rsp_valid_fsm && fail_fsm3) begin
				error <= 1'b1;
				error_kind <= GHCI_FAULT_KIND_AXI_ERROR;
			end
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			bank_num_segs_0 <= 4'd0;
			bank_last_seg_0 <= 3'd0;
			bank_is_last[0] <= 1'b0;
		end else if (bank_close_fire_0) begin
			bank_num_segs_0 <= bw_seg_cnt_next;
			bank_last_seg_0 <= bw_seg_idx;
			bank_is_last[0] <= (p_seg_last && p_rows_b_zero);
		end
	end

	always @(posedge clk or posedge rst_b2d_high) begin
		if (rst_b2d_high) begin
			bank_num_segs_1 <= 4'd0;
			bank_last_seg_1 <= 3'd0;
			bank_is_last[1] <= 1'b0;
		end else if (bank_close_fire_1) begin
			bank_num_segs_1 <= bw_seg_cnt_next;
			bank_last_seg_1 <= bw_seg_idx;
			bank_is_last[1] <= (p_seg_last && p_rows_b_zero);
		end
	end

`ifdef SYNTH_KINTEX7
	wire unused_ok = &{1'b0,
		c_op2, c_resolve, step_co_unused, first_co_unused, fsm_co_unused, bst_co_unused, rlr_co_unused, dpx_neg_co_unused,
		cs32_co_unused, c_dx, c_dy, sy0_2b_co, slot_param_unused, sc_snu_m1_bo, rem_bn_co, room_bn_co, rlr_cand_unused,
		src_slice_u, dst_slice_u, depth_u};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire unused_ok = &{1'b0,
		c_op2, c_resolve, step_co_unused, first_co_unused, fsm_co_unused, bst_co_unused, rlr_co_unused, dpx_neg_co_unused,
		cs32_co_unused, c_dx, c_dy, sy0_2b_co, slot_param_unused, sc_snu_m1_bo, rem_bn_co, room_bn_co, rlr_cand_unused,
		src_slice_u, dst_slice_u, depth_u};
`else
	wire unused_ok = &{1'b0,
		c_op2, c_resolve, step_co_unused, first_co_unused, fsm_co_unused, bst_co_unused, rlr_co_unused, dpx_neg_co_unused,
		cs32_co_unused, c_dx, c_dy, sy0_2b_co, slot_param_unused, sc_snu_m1_bo, rem_bn_co, room_bn_co, rlr_cand_unused,
		src_slice_u, dst_slice_u, depth_u};
`endif
endmodule

`default_nettype wire
