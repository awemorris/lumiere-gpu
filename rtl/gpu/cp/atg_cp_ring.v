/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_ring: the multi-engine command ring:
 *   - 3 independent hardware queues (0: Graphics, 1: Compute, 2: Copy/DMA)
 *   - Independent BASE/SIZE/WPTR/RPTR per queue
 *   - Dynamic fetch arbiter (priority + round-robin)
 *   - Hardware preemption (CP_PREEMPT) and context switch
 *   - Inter-queue lightweight semaphore synchronization (CP_SEMA)
 *   - 8-entry circular line prefetch cache for zero-bubble packet delivery
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_ring #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter CLASS_V = 1	/* 1: class V packets go to the CPU; 0: fault kind 2 */
) (
	input wire clk,
	input wire rst_n,
	input wire core_rst_n,
	/* HCR register snoop (from atg_cp_regs) */
	input wire hcr_we,
	input wire [9:0] hcr_widx,
	input wire [31:0] hcr_wdata,
	input wire [31:0] hcr_wmask,
	input wire [9:0] hcr_ridx,
	output wire [31:0] hcr_rdata,
	/* Control (from atg_cp_ctl) */
	input wire run,
	input wire halt,
	input wire step,
	input wire open,
	/* Class N packets (to atg_cp_seq) */
	output wire n_valid,
	input wire n_ready,
	output wire [31:0] n_data,
	output wire n_last,
	/* Class V packets (to atg_cpu) */
	output wire v_valid,
	output wire [31:0] v_addr,
	output wire [15:0] v_dwords,
	output wire [7:0] v_tag,
	input wire v_done,
	/* Dedicated DMA transfer queue (RING_DMA to atg_blt_3d) */
	output wire dma_req_valid,
	input  wire dma_req_ready,
	output wire [31:0] dma_src_base,
	output wire [31:0] dma_dst_base,
	output wire [31:0] dma_src_row_stride,
	output wire [31:0] dma_dst_row_stride,
	output wire [31:0] dma_src_slice_stride,
	output wire [31:0] dma_dst_slice_stride,
	output wire [15:0] dma_copy_w,
	output wire [15:0] dma_copy_h,
	output wire [15:0] dma_copy_d,
	input  wire dma_done,
	output wire dma_empty,
	output wire dma_busy,
	/* Status */
	output wire empty,
	output wire retire,
	output wire busy,
	/* Multi-queue & Preemption status */
	output wire [1:0] queue_id,
	output wire preempt_ev,
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
	input wire rsp_valid,
	input wire rsp_write,
	input wire [7:0] rsp_tag,
	input wire [LINE_BYTES*8-1:0] rsp_rdata,
	input wire rsp_error
);
	`include "ghci_regs.vh"

	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam WS = LINE_SHIFT - 2;

	localparam [9:0] I_BASE0_LO = HCR_IDX_RING_BASE_LO;
	localparam [9:0] I_BASE0_HI = HCR_IDX_RING_BASE_HI;
	localparam [9:0] I_BASE0    = HCR_IDX_RING_BASE_LO;
	localparam [9:0] I_SIZE0    = HCR_IDX_RING_SIZE;
	localparam [9:0] I_WPTR0    = HCR_IDX_RING_WPTR;
	localparam [9:0] I_RPTR0    = HCR_IDX_RING_RPTR;

	/* Legacy multi-ring and DMA MMIO indices removed in GHCI 1.0 */

	localparam [3:0] R_IDLE         = 4'd0;
	localparam [3:0] R_HDR          = 4'd1;	/* fetch the header at cur */
	localparam [3:0] R_DEC          = 4'd12;	/* decode the header */
	localparam [3:0] R_DEC2         = 4'd13;
	localparam [3:0] R_N            = 4'd2;	/* stream a class N packet */
	localparam [3:0] R_V            = 4'd3;	/* a class V packet waits for v_done */
	localparam [3:0] R_ADV          = 4'd4;	/* move RPTR past the packet */
	localparam [3:0] R_REQ          = 4'd5;	/* on-demand line read request */
	localparam [3:0] R_RSP          = 4'd6;	/* on-demand line read response */
	localparam [3:0] R_FAULT        = 4'd7;
	localparam [3:0] R_CTX_SAVE_REQ = 4'd8;	/* preemption context save AMB write req */
	localparam [3:0] R_CTX_SAVE_RSP = 4'd9;	/* preemption context save AMB write rsp */
	localparam [3:0] R_CTX_REST_REQ = 4'd10;	/* preemption context restore AMB read req */
	localparam [3:0] R_CTX_REST_RSP = 4'd11;	/* preemption context restore AMB read rsp */

	function [4:0] decode_ring_size;
		input [31:0] bytes;
		case (bytes)
			32'h0000_1000: decode_ring_size = 5'd12;
			32'h0000_2000: decode_ring_size = 5'd13;
			32'h0000_4000: decode_ring_size = 5'd14;
			32'h0000_8000: decode_ring_size = 5'd15;
			32'h0001_0000: decode_ring_size = 5'd16;
			32'h0002_0000: decode_ring_size = 5'd17;
			32'h0004_0000: decode_ring_size = 5'd18;
			32'h0008_0000: decode_ring_size = 5'd19;
			32'h0010_0000: decode_ring_size = 5'd20;
			32'h0020_0000: decode_ring_size = 5'd21;
			32'h0040_0000: decode_ring_size = 5'd22;
			32'h0080_0000: decode_ring_size = 5'd23;
			32'h0100_0000: decode_ring_size = 5'd24;
			default:       decode_ring_size = 5'd0;
		endcase
	endfunction

	function [31:0] encode_ring_size;
		input [4:0] sz;
		case (sz)
			5'd12: encode_ring_size = 32'h0000_1000;
			5'd13: encode_ring_size = 32'h0000_2000;
			5'd14: encode_ring_size = 32'h0000_4000;
			5'd15: encode_ring_size = 32'h0000_8000;
			5'd16: encode_ring_size = 32'h0001_0000;
			5'd17: encode_ring_size = 32'h0002_0000;
			5'd18: encode_ring_size = 32'h0004_0000;
			5'd19: encode_ring_size = 32'h0008_0000;
			5'd20: encode_ring_size = 32'h0010_0000;
			5'd21: encode_ring_size = 32'h0020_0000;
			5'd22: encode_ring_size = 32'h0040_0000;
			5'd23: encode_ring_size = 32'h0080_0000;
			5'd24: encode_ring_size = 32'h0100_0000;
			default: encode_ring_size = 32'h0000_1000;
		endcase
	endfunction

	localparam [1:0] PF_IDLE = 2'd0;
	localparam [1:0] PF_REQ  = 2'd1;
	localparam [1:0] PF_RSP  = 2'd2;
	localparam [1:0] PF_WR   = 2'd3;
	localparam integer NSL   = LINE_BYTES / 4;

	/*
	 * 3 Independent Hardware Queues
	 */
	reg [63:0] base [0:2];
	(* max_fanout = 16 *) reg [31:0] size0_enc_r;
`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [4:0] size [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_m_lo [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_m_hi [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_n_lo [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_n_hi [0:2];
	(* keep = "true", dont_touch = "true" *) wire ring_core_rst_high;
	(* keep = "true", dont_touch = "true" *) wire ring_rst_high;
	assign ring_core_rst_high = !core_rst_n;
	assign ring_rst_high = !rst_n;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [4:0] size [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_m_lo [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_m_hi [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_n_lo [0:2];
	(* keep = "true", equivalent_register_removal = "no" *) reg [4:0] size_n_hi [0:2];
	(* keep = "true", dont_touch = "true" *) wire ring_core_rst_high;
	(* keep = "true", dont_touch = "true" *) wire ring_rst_high;
	assign ring_core_rst_high = !core_rst_n;
	assign ring_rst_high = !rst_n;
`else
	reg [4:0] size [0:2];
	reg [4:0] size_m_lo [0:2];
	reg [4:0] size_m_hi [0:2];
	reg [4:0] size_n_lo [0:2];
	reg [4:0] size_n_hi [0:2];
`endif
	reg [21:0] wptr [0:2];
	reg [21:0] rptr [0:2];

	/* Dedicated DMA Transfer Queue registers */
	reg [31:0] dma_base;
	(* max_fanout = 16 *) reg [4:0]  dma_size;
	reg [17:0] dma_wptr;
	reg [17:0] dma_rptr;
	(* max_fanout = 16 *) reg [31:0] dma_sem;
	reg [31:0] dma_dst_base_r;
	reg [31:0] dma_src_row_stride_r, dma_dst_row_stride_r;
	reg [31:0] dma_src_slice_stride_r, dma_dst_slice_stride_r;
	reg [15:0] dma_copy_w_r, dma_copy_h_r, dma_copy_d_r;
	reg dma_active;
	reg dma_req_v;

	/* Pure-Boolean alternating NAND/NOR prefix helpers (zero am_add/am_sub). */
	function [7:0] fn_inc8;
		input [7:0] a;
		reg [7:0] s0_n, s1, s2_n;
		begin
			s0_n = ~(a    & {a[6:0],    1'b1});
			s1   = ~(s0_n | {s0_n[5:0], 2'b00});
			s2_n = ~(s1   & {s1[3:0],   4'hF});
			fn_inc8 = a ^ {~s2_n[6:0], 1'b1};
		end
	endfunction

	function [7:0] fn_dec8;
		input [7:0] a;
		begin
			fn_dec8 = ~fn_inc8(~a);
		end
	endfunction

	function [8:0] fn_inc9;
		input [8:0] a;
		reg [8:0] s0_n, s1, s2_n, s3;
		begin
			s0_n = ~(a    & {a[7:0],    1'b1});
			s1   = ~(s0_n | {s0_n[6:0], 2'b00});
			s2_n = ~(s1   & {s1[4:0],   4'hF});
			s3   = ~(s2_n | {s2_n[0],   8'h00});
			fn_inc9 = a ^ {s3[7:0], 1'b1};
		end
	endfunction

	function [15:0] fn_inc16;
		input [15:0] a;
		reg [15:0] s0_n, s1, s2_n, s3;
		begin
			s0_n = ~(a    & {a[14:0],    1'b1});
			s1   = ~(s0_n | {s0_n[13:0], 2'b00});
			s2_n = ~(s1   & {s1[11:0],   4'hF});
			s3   = ~(s2_n | {s2_n[7:0],  8'h00});
			fn_inc16 = a ^ {s3[14:0], 1'b1};
		end
	endfunction

	function [15:0] fn_dec16;
		input [15:0] a;
		begin
			fn_dec16 = ~fn_inc16(~a);
		end
	endfunction

	function [17:0] fn_inc18;
		input [17:0] a;
		reg [17:0] s0_n, s1, s2_n, s3, s4_n;
		begin
			s0_n = ~(a    & {a[16:0],    1'b1});
			s1   = ~(s0_n | {s0_n[15:0], 2'b00});
			s2_n = ~(s1   & {s1[13:0],   4'hF});
			s3   = ~(s2_n | {s2_n[9:0],  8'h00});
			s4_n = ~(s3   & {s3[1:0],    16'hFFFF});
			fn_inc18 = a ^ {~s4_n[16:0], 1'b1};
		end
	endfunction

	function [31:0] fn_inc32;
		input [31:0] a;
		reg [31:0] s0_n, s1, s2_n, s3, s4_n;
		begin
			s0_n = ~(a    & {a[30:0],    1'b1});
			s1   = ~(s0_n | {s0_n[29:0], 2'b00});
			s2_n = ~(s1   & {s1[27:0],   4'hF});
			s3   = ~(s2_n | {s2_n[23:0], 8'h00});
			s4_n = ~(s3   & {s3[15:0],   16'hFFFF});
			fn_inc32 = a ^ {~s4_n[30:0], 1'b1};
		end
	endfunction

	function [7:0] fn_addsub8;
		input [7:0] a_in;
		input [7:0] b_in;
		input sub;
		reg [7:0] b, g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3;
		begin
			b    = b_in ^ {8{sub}};
			g0_n = ~(a_in & b);
			p0_n = ~(a_in | b);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[6:0], ~sub}));
			p1   = ~(p0_n | {p0_n[6:0], 1'b1});
			g2_n = ~(g1   | (p1 & {g1[5:0], sub, 1'b0}));
			p2_n = ~(p1   & {p1[5:0], 2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[3:0], {4{~sub}}}));
			fn_addsub8 = p0 ^ {g3[6:0], sub};
		end
	endfunction

	function [15:0] fn_add15_ci;
		input [14:0] a_in;
		input [14:0] b_in;
		input        ci;
		reg [14:0] g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3, p3, g4_n;
		begin
			g0_n = ~(a_in & b_in);
			p0_n = ~(a_in | b_in);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[13:0], ~ci}));
			p1   = ~(p0_n | {p0_n[13:0], 1'b1});
			g2_n = ~(g1   | (p1   & {g1[12:0],   ci, 1'b0}));
			p2_n = ~(p1   & {p1[12:0],   2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[10:0], {4{~ci}}}));
			p3   = ~(p2_n | {p2_n[10:0], 4'hF});
			g4_n = ~(g3   | (p3   & {g3[6:0],    {8{ci}}}));
			fn_add15_ci = {~g4_n[14], p0 ^ {~g4_n[13:0], ci}};
		end
	endfunction

	function [8:0] fn_add8_ci;
		input [7:0] a_in;
		input [7:0] b_in;
		input       ci;
		reg [7:0] g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3, p3, g4_n;
		begin
			g0_n = ~(a_in & b_in);
			p0_n = ~(a_in | b_in);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[6:0], ~ci}));
			p1   = ~(p0_n | {p0_n[6:0], 1'b1});
			g2_n = ~(g1   | (p1   & {g1[5:0],   ci, 1'b0}));
			p2_n = ~(p1   & {p1[5:0],   2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[3:0], {4{~ci}}}));
			p3   = ~(p2_n | {p2_n[3:0], 4'hF});
			g4_n = ~(g3   | (p3   & {8{ci}}));
			fn_add8_ci = {~g4_n[7], p0 ^ {~g4_n[6:0], ci}};
		end
	endfunction

	function [22:0] fn_add22_sub;
		input [21:0] a_in;
		input [21:0] b_in;
		reg [21:0] g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3, p3, g4_n, p4_n, g5;
		begin
			g0_n = ~(a_in & b_in);
			p0_n = ~(a_in | b_in);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[20:0], 1'b0}));
			p1   = ~(p0_n | {p0_n[20:0], 1'b1});
			g2_n = ~(g1   | (p1   & {g1[19:0],   2'b10}));
			p2_n = ~(p1   & {p1[19:0],   2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[17:0], 4'h0}));
			p3   = ~(p2_n | {p2_n[17:0], 4'hF});
			g4_n = ~(g3   | (p3   & {g3[13:0],   8'hFF}));
			p4_n = ~(p3   & {p3[13:0],   8'h00});
			g5   = ~(g4_n & (p4_n | {g4_n[5:0],  16'h0000}));
			fn_add22_sub = {g5[21], p0 ^ {g5[20:0], 1'b1}};
		end
	endfunction

	function [22:0] fn_add22_co;
		input [21:0] a_in;
		input [21:0] b_in;
		reg [21:0] g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3, p3, g4_n, p4_n, g5;
		begin
			g0_n = ~(a_in & b_in);
			p0_n = ~(a_in | b_in);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[20:0], 1'b1}));
			p1   = ~(p0_n | {p0_n[20:0], 1'b1});
			g2_n = ~(g1   | (p1   & {g1[19:0],   2'b00}));
			p2_n = ~(p1   & {p1[19:0],   2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[17:0], 4'hF}));
			p3   = ~(p2_n | {p2_n[17:0], 4'hF});
			g4_n = ~(g3   | (p3   & {g3[13:0],   8'h00}));
			p4_n = ~(p3   & {p3[13:0],   8'h00});
			g5   = ~(g4_n & (p4_n | {g4_n[5:0],  16'hFFFF}));
			fn_add22_co = {g5[21], p0 ^ {g5[20:0], 1'b0}};
		end
	endfunction

	function [23:0] fn_add23_ci;
		input [22:0] a_in;
		input [22:0] b_in;
		input ci;
		reg [22:0] g0_n, p0_n, p0, g1, p1, g2_n, p2_n, g3, p3, g4_n, p4_n, g5;
		begin
			g0_n = ~(a_in & b_in);
			p0_n = ~(a_in | b_in);
			p0   = g0_n & ~p0_n;
			g1   = ~(g0_n & (p0_n | {g0_n[21:0], ~ci}));
			p1   = ~(p0_n | {p0_n[21:0], 1'b1});
			g2_n = ~(g1   | (p1   & {g1[20:0],   ci, 1'b0}));
			p2_n = ~(p1   & {p1[20:0],   2'b00});
			g3   = ~(g2_n & (p2_n | {g2_n[18:0], {4{~ci}}}));
			p3   = ~(p2_n | {p2_n[18:0], 4'hF});
			g4_n = ~(g3   | (p3   & {g3[14:0],   {8{ci}}}));
			p4_n = ~(p3   & {p3[14:0],   8'h00});
			g5   = ~(g4_n & (p4_n | {g4_n[6:0],  {16{~ci}}}));
			fn_add23_ci = {g5[22], p0 ^ {g5[21:0], ci}};
		end
	endfunction

`ifdef SYNTH_KINTEX7
	function [8:0] fn_add8_k7_c0;
		input [7:0] a;
		input [7:0] b;
		reg c1, c2, c3, g1, p1, c4, c5, g2, p2, c6, c8;
		reg [7:0] s;
		begin
			s[0] = a[0] ^ b[0];
			c1   = a[0] & b[0];
			s[1] = a[1] ^ b[1] ^ c1;
			c2   = (a[1] & b[1]) | ((a[1] | b[1]) & c1);
			s[2] = a[2] ^ b[2] ^ c2;
			c3   = (a[2] & b[2]) | ((a[2] | b[2]) & ((a[1] & b[1]) | ((a[1] | b[1]) & (a[0] & b[0]))));
			g1   = (a[5] & b[5]) | ((a[5] | b[5]) & ((a[4] & b[4]) | ((a[4] | b[4]) & (a[3] & b[3]))));
			p1   = (a[5] | b[5]) & (a[4] | b[4]) & (a[3] | b[3]);
			s[3] = a[3] ^ b[3] ^ c3;
			c4   = (a[3] & b[3]) | ((a[3] | b[3]) & c3);
			c5   = (a[4] & b[4]) | ((a[4] | b[4]) & ((a[3] & b[3]) | ((a[3] | b[3]) & c3)));
			s[4] = a[4] ^ b[4] ^ c4;
			s[5] = a[5] ^ b[5] ^ c5;
			g2   = (a[7] & b[7]) | ((a[7] | b[7]) & (a[6] & b[6]));
			p2   = (a[7] | b[7]) & (a[6] | b[6]);
			c6   = g1 | (p1 & c3);
			s[6] = a[6] ^ b[6] ^ c6;
			s[7] = a[7] ^ b[7] ^ ((a[6] & b[6]) | ((a[6] | b[6]) & c6));
			c8   = g2 | (p2 & g1) | (p2 & p1 & c3);
			fn_add8_k7_c0 = {c8, s};
		end
	endfunction

	function [8:0] fn_add8_k7_c1;
		input [7:0] a;
		input [7:0] b;
		reg c1, c2, c3, g1, p1, c4, c5, g2, p2, c6, c8;
		reg [7:0] s;
		begin
			s[0] = ~(a[0] ^ b[0]);
			c1   = a[0] | b[0];
			s[1] = a[1] ^ b[1] ^ c1;
			c2   = (a[1] & b[1]) | ((a[1] | b[1]) & c1);
			s[2] = a[2] ^ b[2] ^ c2;
			c3   = (a[2] & b[2]) | ((a[2] | b[2]) & ((a[1] & b[1]) | ((a[1] | b[1]) & (a[0] | b[0]))));
			g1   = (a[5] & b[5]) | ((a[5] | b[5]) & ((a[4] & b[4]) | ((a[4] | b[4]) & (a[3] & b[3]))));
			p1   = (a[5] | b[5]) & (a[4] | b[4]) & (a[3] | b[3]);
			s[3] = a[3] ^ b[3] ^ c3;
			c4   = (a[3] & b[3]) | ((a[3] | b[3]) & c3);
			c5   = (a[4] & b[4]) | ((a[4] | b[4]) & ((a[3] & b[3]) | ((a[3] | b[3]) & c3)));
			s[4] = a[4] ^ b[4] ^ c4;
			s[5] = a[5] ^ b[5] ^ c5;
			g2   = (a[7] & b[7]) | ((a[7] | b[7]) & (a[6] & b[6]));
			p2   = (a[7] | b[7]) & (a[6] | b[6]);
			c6   = g1 | (p1 & c3);
			s[6] = a[6] ^ b[6] ^ c6;
			s[7] = a[7] ^ b[7] ^ ((a[6] & b[6]) | ((a[6] | b[6]) & c6));
			c8   = g2 | (p2 & g1) | (p2 & p1 & c3);
			fn_add8_k7_c1 = {c8, s};
		end
	endfunction

	function [8:0] fn_inc9_k7;
		input [8:0] a;
		reg c4;
		begin
			c4 = &a[3:0];
			fn_inc9_k7[0] = ~a[0];
			fn_inc9_k7[1] = a[1] ^ a[0];
			fn_inc9_k7[2] = a[2] ^ (a[1] & a[0]);
			fn_inc9_k7[3] = a[3] ^ (a[2] & a[1] & a[0]);
			fn_inc9_k7[4] = a[4] ^ c4;
			fn_inc9_k7[5] = a[5] ^ (c4 & a[4]);
			fn_inc9_k7[6] = a[6] ^ (c4 & a[5] & a[4]);
			fn_inc9_k7[7] = a[7] ^ (c4 & a[6] & a[5] & a[4]);
			fn_inc9_k7[8] = a[8] ^ (c4 & (&a[7:4]));
		end
	endfunction

	function [15:0] fn_inc16_k7;
		input [15:0] a;
		reg c4, p74, p118, c12;
		begin
			c4   = &a[3:0];
			p74  = &a[7:4];
			p118 = &a[11:8];
			c12  = c4 & p74 & p118;
			fn_inc16_k7[0]  = ~a[0];
			fn_inc16_k7[1]  = a[1] ^ a[0];
			fn_inc16_k7[2]  = a[2] ^ (a[1] & a[0]);
			fn_inc16_k7[3]  = a[3] ^ (a[2] & a[1] & a[0]);
			fn_inc16_k7[4]  = a[4] ^ c4;
			fn_inc16_k7[5]  = a[5] ^ (c4 & a[4]);
			fn_inc16_k7[6]  = a[6] ^ (c4 & a[5] & a[4]);
			fn_inc16_k7[7]  = a[7] ^ (c4 & a[6] & a[5] & a[4]);
			fn_inc16_k7[8]  = a[8] ^ (c4 & p74);
			fn_inc16_k7[9]  = a[9] ^ (c4 & p74 & a[8]);
			fn_inc16_k7[10] = a[10] ^ (c4 & p74 & a[9] & a[8]);
			fn_inc16_k7[11] = a[11] ^ (c4 & p74 & a[10] & a[9] & a[8]);
			fn_inc16_k7[12] = a[12] ^ c12;
			fn_inc16_k7[13] = a[13] ^ (c12 & a[12]);
			fn_inc16_k7[14] = a[14] ^ (c12 & a[13] & a[12]);
			fn_inc16_k7[15] = a[15] ^ (c12 & a[14] & a[13] & a[12]);
		end
	endfunction

	function fn_ge16_en_k7;
		input [15:0] a;
		input [15:0] b;
		input en;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, ge_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = en & (gt2[7] | (eq2[7] & gt2[6]));
			eq_hi  = en & eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			ge_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & (gt2[0] | eq2[0]));
			fn_ge16_en_k7 = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & ge_lo);
		end
	endfunction

	function fn_gt16_en_k7;
		input [15:0] a;
		input [15:0] b;
		input en;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, gt_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = en & (gt2[7] | (eq2[7] & gt2[6]));
			eq_hi  = en & eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			gt_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & gt2[0]);
			fn_gt16_en_k7 = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & gt_lo);
		end
	endfunction

	function fn_ge8_en_k7;
		input [7:0] a;
		input [7:0] b;
		input en;
		reg [3:0] gt2, eq2;
		reg gt_hi, eq_hi, ge_lo;
		integer p;
		begin
			for (p = 0; p < 4; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi = en & (gt2[3] | (eq2[3] & gt2[2]));
			eq_hi = en & eq2[3] & eq2[2];
			ge_lo = gt2[1] | (eq2[1] & (gt2[0] | eq2[0]));
			fn_ge8_en_k7 = gt_hi | (eq_hi & ge_lo);
		end
	endfunction

	function fn_gt8_en_k7;
		input [7:0] a;
		input [7:0] b;
		input en;
		reg [3:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_lo;
		integer p;
		begin
			for (p = 0; p < 4; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi = en & (gt2[3] | (eq2[3] & gt2[2]));
			eq_hi = en & eq2[3] & eq2[2];
			gt_lo = gt2[1] | (eq2[1] & gt2[0]);
			fn_gt8_en_k7 = gt_hi | (eq_hi & gt_lo);
		end
	endfunction
`elsif SYNTH_KINTEX_ULTRASCALE
	function [8:0] fn_add8_k7_c0;
		input [7:0] a;
		input [7:0] b;
		reg c1, c2, c3, g1, p1, c4, c5, g2, p2, c6, c8;
		reg [7:0] s;
		begin
			s[0] = a[0] ^ b[0];
			c1   = a[0] & b[0];
			s[1] = a[1] ^ b[1] ^ c1;
			c2   = (a[1] & b[1]) | ((a[1] | b[1]) & c1);
			s[2] = a[2] ^ b[2] ^ c2;
			c3   = (a[2] & b[2]) | ((a[2] | b[2]) & ((a[1] & b[1]) | ((a[1] | b[1]) & (a[0] & b[0]))));
			g1   = (a[5] & b[5]) | ((a[5] | b[5]) & ((a[4] & b[4]) | ((a[4] | b[4]) & (a[3] & b[3]))));
			p1   = (a[5] | b[5]) & (a[4] | b[4]) & (a[3] | b[3]);
			s[3] = a[3] ^ b[3] ^ c3;
			c4   = (a[3] & b[3]) | ((a[3] | b[3]) & c3);
			c5   = (a[4] & b[4]) | ((a[4] | b[4]) & ((a[3] & b[3]) | ((a[3] | b[3]) & c3)));
			s[4] = a[4] ^ b[4] ^ c4;
			s[5] = a[5] ^ b[5] ^ c5;
			g2   = (a[7] & b[7]) | ((a[7] | b[7]) & (a[6] & b[6]));
			p2   = (a[7] | b[7]) & (a[6] | b[6]);
			c6   = g1 | (p1 & c3);
			s[6] = a[6] ^ b[6] ^ c6;
			s[7] = a[7] ^ b[7] ^ ((a[6] & b[6]) | ((a[6] | b[6]) & c6));
			c8   = g2 | (p2 & g1) | (p2 & p1 & c3);
			fn_add8_k7_c0 = {c8, s};
		end
	endfunction

	function [8:0] fn_add8_k7_c1;
		input [7:0] a;
		input [7:0] b;
		reg c1, c2, c3, g1, p1, c4, c5, g2, p2, c6, c8;
		reg [7:0] s;
		begin
			s[0] = ~(a[0] ^ b[0]);
			c1   = a[0] | b[0];
			s[1] = a[1] ^ b[1] ^ c1;
			c2   = (a[1] & b[1]) | ((a[1] | b[1]) & c1);
			s[2] = a[2] ^ b[2] ^ c2;
			c3   = (a[2] & b[2]) | ((a[2] | b[2]) & ((a[1] & b[1]) | ((a[1] | b[1]) & (a[0] | b[0]))));
			g1   = (a[5] & b[5]) | ((a[5] | b[5]) & ((a[4] & b[4]) | ((a[4] | b[4]) & (a[3] & b[3]))));
			p1   = (a[5] | b[5]) & (a[4] | b[4]) & (a[3] | b[3]);
			s[3] = a[3] ^ b[3] ^ c3;
			c4   = (a[3] & b[3]) | ((a[3] | b[3]) & c3);
			c5   = (a[4] & b[4]) | ((a[4] | b[4]) & ((a[3] & b[3]) | ((a[3] | b[3]) & c3)));
			s[4] = a[4] ^ b[4] ^ c4;
			s[5] = a[5] ^ b[5] ^ c5;
			g2   = (a[7] & b[7]) | ((a[7] | b[7]) & (a[6] & b[6]));
			p2   = (a[7] | b[7]) & (a[6] | b[6]);
			c6   = g1 | (p1 & c3);
			s[6] = a[6] ^ b[6] ^ c6;
			s[7] = a[7] ^ b[7] ^ ((a[6] & b[6]) | ((a[6] | b[6]) & c6));
			c8   = g2 | (p2 & g1) | (p2 & p1 & c3);
			fn_add8_k7_c1 = {c8, s};
		end
	endfunction

	function [8:0] fn_inc9_k7;
		input [8:0] a;
		reg c4;
		begin
			c4 = &a[3:0];
			fn_inc9_k7[0] = ~a[0];
			fn_inc9_k7[1] = a[1] ^ a[0];
			fn_inc9_k7[2] = a[2] ^ (a[1] & a[0]);
			fn_inc9_k7[3] = a[3] ^ (a[2] & a[1] & a[0]);
			fn_inc9_k7[4] = a[4] ^ c4;
			fn_inc9_k7[5] = a[5] ^ (c4 & a[4]);
			fn_inc9_k7[6] = a[6] ^ (c4 & a[5] & a[4]);
			fn_inc9_k7[7] = a[7] ^ (c4 & a[6] & a[5] & a[4]);
			fn_inc9_k7[8] = a[8] ^ (c4 & (&a[7:4]));
		end
	endfunction

	function [15:0] fn_inc16_k7;
		input [15:0] a;
		reg c4, p74, p118, c12;
		begin
			c4   = &a[3:0];
			p74  = &a[7:4];
			p118 = &a[11:8];
			c12  = c4 & p74 & p118;
			fn_inc16_k7[0]  = ~a[0];
			fn_inc16_k7[1]  = a[1] ^ a[0];
			fn_inc16_k7[2]  = a[2] ^ (a[1] & a[0]);
			fn_inc16_k7[3]  = a[3] ^ (a[2] & a[1] & a[0]);
			fn_inc16_k7[4]  = a[4] ^ c4;
			fn_inc16_k7[5]  = a[5] ^ (c4 & a[4]);
			fn_inc16_k7[6]  = a[6] ^ (c4 & a[5] & a[4]);
			fn_inc16_k7[7]  = a[7] ^ (c4 & a[6] & a[5] & a[4]);
			fn_inc16_k7[8]  = a[8] ^ (c4 & p74);
			fn_inc16_k7[9]  = a[9] ^ (c4 & p74 & a[8]);
			fn_inc16_k7[10] = a[10] ^ (c4 & p74 & a[9] & a[8]);
			fn_inc16_k7[11] = a[11] ^ (c4 & p74 & a[10] & a[9] & a[8]);
			fn_inc16_k7[12] = a[12] ^ c12;
			fn_inc16_k7[13] = a[13] ^ (c12 & a[12]);
			fn_inc16_k7[14] = a[14] ^ (c12 & a[13] & a[12]);
			fn_inc16_k7[15] = a[15] ^ (c12 & a[14] & a[13] & a[12]);
		end
	endfunction

	function fn_ge16_en_k7;
		input [15:0] a;
		input [15:0] b;
		input en;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, ge_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = en & (gt2[7] | (eq2[7] & gt2[6]));
			eq_hi  = en & eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			ge_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & (gt2[0] | eq2[0]));
			fn_ge16_en_k7 = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & ge_lo);
		end
	endfunction

	function fn_gt16_en_k7;
		input [15:0] a;
		input [15:0] b;
		input en;
		reg [7:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_mid, eq_mid, gt_lo;
		integer p;
		begin
			for (p = 0; p < 8; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi  = en & (gt2[7] | (eq2[7] & gt2[6]));
			eq_hi  = en & eq2[7] & eq2[6];
			gt_mid = gt2[5] | (eq2[5] & gt2[4]) | (eq2[5] & eq2[4] & gt2[3]);
			eq_mid = eq2[5] & eq2[4] & eq2[3];
			gt_lo  = gt2[2] | (eq2[2] & gt2[1]) | (eq2[2] & eq2[1] & gt2[0]);
			fn_gt16_en_k7 = gt_hi | (eq_hi & gt_mid) | (eq_hi & eq_mid & gt_lo);
		end
	endfunction

	function fn_ge8_en_k7;
		input [7:0] a;
		input [7:0] b;
		input en;
		reg [3:0] gt2, eq2;
		reg gt_hi, eq_hi, ge_lo;
		integer p;
		begin
			for (p = 0; p < 4; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi = en & (gt2[3] | (eq2[3] & gt2[2]));
			eq_hi = en & eq2[3] & eq2[2];
			ge_lo = gt2[1] | (eq2[1] & (gt2[0] | eq2[0]));
			fn_ge8_en_k7 = gt_hi | (eq_hi & ge_lo);
		end
	endfunction

	function fn_gt8_en_k7;
		input [7:0] a;
		input [7:0] b;
		input en;
		reg [3:0] gt2, eq2;
		reg gt_hi, eq_hi, gt_lo;
		integer p;
		begin
			for (p = 0; p < 4; p = p + 1) begin
				gt2[p] = (a[2*p+1] & ~b[2*p+1]) | ((~(a[2*p+1] ^ b[2*p+1])) & a[2*p] & ~b[2*p]);
				eq2[p] = (~(a[2*p+1] ^ b[2*p+1])) & (~(a[2*p] ^ b[2*p]));
			end
			gt_hi = en & (gt2[3] | (eq2[3] & gt2[2]));
			eq_hi = en & eq2[3] & eq2[2];
			gt_lo = gt2[1] | (eq2[1] & gt2[0]);
			fn_gt8_en_k7 = gt_hi | (eq_hi & gt_lo);
		end
	endfunction
`endif

	function fn_cmp_gt8;
		input [7:0] a_in;
		input [7:0] b_in;
		reg [7:0] g0_n, t0_n, g1, t1, g2_n, t2_n, g3;
		begin
			/* a > b <=> b < a */
			g0_n = ~(a_in & ~b_in);
			t0_n = ~(a_in | ~b_in);
			g1   = ~(g0_n & (t0_n | {g0_n[6:0], 1'b1}));
			t1   = ~(t0_n | {t0_n[6:0], 1'b1});
			g2_n = ~(g1   | (t1   & {g1[5:0],   2'b00}));
			t2_n = ~(t1   & {t1[5:0],   2'b00});
			g3   = ~(g2_n & (t2_n | {g2_n[3:0], 4'hF}));
			fn_cmp_gt8 = g3[7];
		end
	endfunction

	function fn_cmp_gt9;
		input [8:0] a_in;
		input [8:0] b_in;
		reg [8:0] g0_n, t0_n, g1, t1, g2_n, t2_n, g3, t3, g4_n;
		begin
			g0_n = ~(a_in & ~b_in);
			t0_n = ~(a_in | ~b_in);
			g1   = ~(g0_n & (t0_n | {g0_n[7:0], 1'b1}));
			t1   = ~(t0_n | {t0_n[7:0], 1'b1});
			g2_n = ~(g1   | (t1   & {g1[6:0],   2'b00}));
			t2_n = ~(t1   & {t1[6:0],   2'b00});
			g3   = ~(g2_n & (t2_n | {g2_n[4:0], 4'hF}));
			t3   = ~(t2_n | {t2_n[4:0], 4'hF});
			g4_n = ~(g3   | (t3   & {g3[0],     8'h00}));
			fn_cmp_gt9 = ~g4_n[8];
		end
	endfunction

	wire [17:0] dma_rptr_next = fn_inc18(dma_rptr);
	wire unused_dr_co = 1'b0;

	wire [31:0] dma_sem_next = fn_inc32(dma_sem);
	wire unused_ds_co = 1'b0;

	assign dma_req_valid        = dma_req_v;
	assign dma_src_base         = dma_base;
	assign dma_dst_base         = dma_dst_base_r;
	assign dma_src_row_stride   = dma_src_row_stride_r;
	assign dma_dst_row_stride   = dma_dst_row_stride_r;
	assign dma_src_slice_stride = dma_src_slice_stride_r;
	assign dma_dst_slice_stride = dma_dst_slice_stride_r;
	assign dma_copy_w           = dma_copy_w_r;
	assign dma_copy_h           = dma_copy_h_r;
	assign dma_copy_d           = dma_copy_d_r;
	assign dma_empty            = (dma_wptr == dma_rptr);
	assign dma_busy             = dma_active || (dma_wptr != dma_rptr);

	wire [4:0] size0_ml = size_m_lo[0];
	wire [4:0] size0_mh = size_m_hi[0];
	wire [4:0] size0_nl = size_n_lo[0];
	wire [4:0] size0_nh = size_n_hi[0];
	wire [4:0] size1_ml = size_m_lo[1];
	wire [4:0] size1_mh = size_m_hi[1];
	wire [4:0] size1_nl = size_n_lo[1];
	wire [4:0] size1_nh = size_n_hi[1];
	wire [4:0] size2_ml = size_m_lo[2];
	wire [4:0] size2_mh = size_m_hi[2];
	wire [4:0] size2_nl = size_n_lo[2];
	wire [4:0] size2_nh = size_n_hi[2];

	reg [21:0] mask0, mask1, mask2;
	reg [22:0] nwords0, nwords1, nwords2;

	integer mi;
	always @(*) begin
		for (mi = 0; mi < 11; mi = mi + 1) begin
			mask0[mi] = {27'd0, size0_ml} > mi + 2;
			mask1[mi] = {27'd0, size1_ml} > mi + 2;
			mask2[mi] = {27'd0, size2_ml} > mi + 2;
		end
		for (mi = 11; mi < 22; mi = mi + 1) begin
			mask0[mi] = {27'd0, size0_mh} > mi + 2;
			mask1[mi] = {27'd0, size1_mh} > mi + 2;
			mask2[mi] = {27'd0, size2_mh} > mi + 2;
		end
		for (mi = 0; mi < 12; mi = mi + 1) begin
			nwords0[mi] = {27'd0, size0_nl} == mi + 2;
			nwords1[mi] = {27'd0, size1_nl} == mi + 2;
			nwords2[mi] = {27'd0, size2_nl} == mi + 2;
		end
		for (mi = 12; mi < 23; mi = mi + 1) begin
			nwords0[mi] = {27'd0, size0_nh} == mi + 2;
			nwords1[mi] = {27'd0, size1_nh} == mi + 2;
			nwords2[mi] = {27'd0, size2_nh} == mi + 2;
		end
	end

	wire [21:0] mask [0:2];
	assign mask[0] = mask0;
	assign mask[1] = mask1;
	assign mask[2] = mask2;

	wire [22:0] nwords [0:2];
	assign nwords[0] = nwords0;
	assign nwords[1] = nwords1;
	assign nwords[2] = nwords2;

	(* max_fanout = 16 *) reg [1:0] active_qid;
	reg [1:0] interrupted_qid;
	reg [21:0] saved_rptr;
	reg [31:0] ctx_base;
	reg ctx_base_z_r;
	reg preempt_pending;
	(* max_fanout = 12, extract_enable = "no" *) reg preempt_active;
	reg preempt_saved;
	reg preempt_auto;
	reg [3:0] prio_mode;
	reg preempt_pulse;
	reg [1:0] last_non_q0;

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) reg [31:0] wdata_a, wdata_b, wdata_c, wdata_d;
	(* dont_touch = "true" *) reg [4:0] dec_sz_r;
	(* dont_touch = "true" *) reg dec_sz_nz_r;
	(* dont_touch = "true" *) reg [8:0] wdata_d1;
	(* dont_touch = "true" *) reg [31:0] wmask_a, wmask_b, wmask_c;
	wire [4:0] hcr_dec_sz_w = decode_ring_size(hcr_wdata);
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			wdata_a  <= 32'd0;
			wdata_b  <= 32'd0;
			wdata_c  <= 32'd0;
			wdata_d  <= 32'd0;
			dec_sz_r <= 5'd0;
			dec_sz_nz_r <= 1'b0;
			wdata_d1 <= 9'd0;
			wmask_a  <= 32'd0;
			wmask_b  <= 32'd0;
			wmask_c  <= 32'd0;
		end else begin
			wdata_a  <= hcr_wdata;
			wdata_b  <= hcr_wdata;
			wdata_c  <= hcr_wdata;
			wdata_d  <= hcr_wdata;
			dec_sz_r <= hcr_dec_sz_w;
			dec_sz_nz_r <= (hcr_dec_sz_w != 5'd0);
			wdata_d1 <= hcr_wdata[8:0];
			wmask_a  <= hcr_wmask;
			wmask_b  <= hcr_wmask;
			wmask_c  <= hcr_wmask;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) reg [31:0] wdata_a, wdata_b, wdata_c, wdata_d;
	(* dont_touch = "true" *) reg [4:0] dec_sz_r;
	(* dont_touch = "true" *) reg dec_sz_nz_r;
	(* dont_touch = "true" *) reg [8:0] wdata_d1;
	(* dont_touch = "true" *) reg [31:0] wmask_a, wmask_b, wmask_c;
	wire [4:0] hcr_dec_sz_w = decode_ring_size(hcr_wdata);
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			wdata_a  <= 32'd0;
			wdata_b  <= 32'd0;
			wdata_c  <= 32'd0;
			wdata_d  <= 32'd0;
			dec_sz_r <= 5'd0;
			dec_sz_nz_r <= 1'b0;
			wdata_d1 <= 9'd0;
			wmask_a  <= 32'd0;
			wmask_b  <= 32'd0;
			wmask_c  <= 32'd0;
		end else begin
			wdata_a  <= hcr_wdata;
			wdata_b  <= hcr_wdata;
			wdata_c  <= hcr_wdata;
			wdata_d  <= hcr_wdata;
			dec_sz_r <= hcr_dec_sz_w;
			dec_sz_nz_r <= (hcr_dec_sz_w != 5'd0);
			wdata_d1 <= hcr_wdata[8:0];
			wmask_a  <= hcr_wmask;
			wmask_b  <= hcr_wmask;
			wmask_c  <= hcr_wmask;
		end
	end
`else
	(* keep = "true" *) reg [31:0] wdata_a, wdata_b, wdata_c, wdata_d;
	reg [4:0] dec_sz_r;
	reg dec_sz_nz_r;
	(* keep = "true" *) reg [8:0]  wdata_d1;
	(* keep = "true" *) reg [31:0] wmask_a, wmask_b, wmask_c;
	wire [4:0] hcr_dec_sz_w = decode_ring_size(hcr_wdata);
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wdata_a  <= 32'd0;
			wdata_b  <= 32'd0;
			wdata_c  <= 32'd0;
			wdata_d  <= 32'd0;
			dec_sz_r <= 5'd0;
			dec_sz_nz_r <= 1'b0;
			wdata_d1 <= 9'd0;
			wmask_a  <= 32'd0;
			wmask_b  <= 32'd0;
			wmask_c  <= 32'd0;
		end else begin
			wdata_a  <= hcr_wdata;
			wdata_b  <= hcr_wdata;
			wdata_c  <= hcr_wdata;
			wdata_d  <= hcr_wdata;
			dec_sz_r <= hcr_dec_sz_w;
			dec_sz_nz_r <= (hcr_dec_sz_w != 5'd0);
			wdata_d1 <= hcr_wdata[8:0];
			wmask_a  <= hcr_wmask;
			wmask_b  <= hcr_wmask;
			wmask_c  <= hcr_wmask;
		end
	end
`endif
	reg [7:0] sema [0:7];
	reg [2:0] queue_sema_id [0:2];
	reg [2:0] queue_sema_wait;

	wire [1:0] sema_cmd = wdata_d1[1:0];
	wire [2:0] sema_w_id = wdata_d1[4:2];
	wire [1:0] sema_w_qid = wdata_d1[6:5];
	wire sema_set_q_wait = wdata_d1[7];
	wire sema_clr_q_wait = wdata_d1[8];

	wire [7:0] sema_cur;
	wire [7:0] sema_op_val = (wdata_d[23:16] == 8'd0) ? 8'd1 : wdata_d[23:16];
	wire [7:0] sema_next;
	wire sema_co;

`ifdef SYNTH_KINTEX7
	genvar g_sc;
	generate
		for (g_sc = 0; g_sc < 8; g_sc = g_sc + 1) begin : g_sema_cur_mux
			wire sc_lo, sc_hi;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_sc_lo (
				.I0(sema[0][g_sc]), .I1(sema[1][g_sc]), .I2(sema[2][g_sc]), .I3(sema[3][g_sc]),
				.I4(wdata_d[2]), .I5(wdata_d[3]), .O(sc_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_sc_hi (
				.I0(sema[4][g_sc]), .I1(sema[5][g_sc]), .I2(sema[6][g_sc]), .I3(sema[7][g_sc]),
				.I4(wdata_d[2]), .I5(wdata_d[3]), .O(sc_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_sc_f7 (
				.I0(sc_lo), .I1(sc_hi), .S(wdata_d[4]), .O(sema_cur[g_sc])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_sc;
	generate
		for (g_sc = 0; g_sc < 8; g_sc = g_sc + 1) begin : g_sema_cur_mux
			wire sc_lo, sc_hi;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_sc_lo (
				.I0(sema[0][g_sc]), .I1(sema[1][g_sc]), .I2(sema[2][g_sc]), .I3(sema[3][g_sc]),
				.I4(wdata_d[2]), .I5(wdata_d[3]), .O(sc_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_sc_hi (
				.I0(sema[4][g_sc]), .I1(sema[5][g_sc]), .I2(sema[6][g_sc]), .I3(sema[7][g_sc]),
				.I4(wdata_d[2]), .I5(wdata_d[3]), .O(sc_hi)
			);
			(* DONT_TOUCH = "TRUE" *) MUXF7 u_sc_f7 (
				.I0(sc_lo), .I1(sc_hi), .S(wdata_d[4]), .O(sema_cur[g_sc])
			);
		end
	endgenerate
`else
	assign sema_cur = sema[wdata_d[4:2]];
`endif

	assign sema_next = fn_addsub8(sema_cur, sema_op_val, wdata_d[1:0] == 2'd2);
	assign sema_co = 1'b0;

	/*
	 * Active Queue Signals & Status
	 */
	wire [2:0] q_empty;
	assign q_empty[0] = (wptr[0] == rptr[0]);
	assign q_empty[1] = (wptr[1] == rptr[1]);
	assign q_empty[2] = (wptr[2] == rptr[2]);

	wire [2:0] has_work;
	assign has_work[0] = !q_empty[0] && !(queue_sema_wait[0] && (sema[queue_sema_id[0]] == 8'd0));
	assign has_work[1] = !q_empty[1] && !(queue_sema_wait[1] && (sema[queue_sema_id[1]] == 8'd0));
	assign has_work[2] = !q_empty[2] && !(queue_sema_wait[2] && (sema[queue_sema_id[2]] == 8'd0));

	(* keep = "true", equivalent_register_removal = "no" *) reg [1:0] aqid_av_r0a, aqid_av_r0b, aqid_av_r1a, aqid_av_r1b, aqid_av_r2, aqid_av_r3;

`ifdef SYNTH_KINTEX7
	wire [1:0] aqid_b [0:11];
	genvar g_aqb;
	generate
		for (g_aqb = 0; g_aqb < 12; g_aqb = g_aqb + 1) begin : g_aqid_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_aqb0 (.I0(active_qid[0]), .O(aqid_b[g_aqb][0]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_aqb1 (.I0(active_qid[1]), .O(aqid_b[g_aqb][1]));
		end
	endgenerate
	wire [21:0] wptr_act   = {wptr[aqid_b[1]][21:11], wptr[aqid_b[0]][10:0]};
	wire [21:0] rptr_act   = {rptr[aqid_b[3]][21:11], rptr[aqid_b[2]][10:0]};
	wire [63:0] base_act   = {32'd0, base[aqid_b[11]][31:24], base[aqid_b[10]][23:16], base[aqid_b[5]][15:8], base[aqid_b[4]][7:0]};
	wire [21:0] mask_act   = {mask[aqid_b[7]][21:11], mask[aqid_b[6]][10:0]};
	wire [22:0] nwords_act = {nwords[aqid_b[9]][22:11], nwords[aqid_b[8]][10:0]};
	wire [21:0] wptr_av_c  = {wptr[aqid_av_r1a][21:16], wptr[aqid_av_r1b][15:11], wptr[aqid_av_r0b][10:6], wptr[aqid_av_r0a][5:0]};
	wire [21:0] mask_av_c  = {mask[aqid_av_r3][21:11], mask[aqid_av_r2][10:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] aqid_b [0:11];
	genvar g_aqb;
	generate
		for (g_aqb = 0; g_aqb < 12; g_aqb = g_aqb + 1) begin : g_aqid_buf
			assign aqid_b[g_aqb][0] = active_qid[0];
			assign aqid_b[g_aqb][1] = active_qid[1];
		end
	endgenerate
	wire [21:0] wptr_act   = {wptr[aqid_b[1]][21:11], wptr[aqid_b[0]][10:0]};
	wire [21:0] rptr_act   = {rptr[aqid_b[3]][21:11], rptr[aqid_b[2]][10:0]};
	wire [63:0] base_act   = {32'd0, base[aqid_b[11]][31:24], base[aqid_b[10]][23:16], base[aqid_b[5]][15:8], base[aqid_b[4]][7:0]};
	wire [21:0] mask_act   = {mask[aqid_b[7]][21:11], mask[aqid_b[6]][10:0]};
	wire [22:0] nwords_act = {nwords[aqid_b[9]][22:11], nwords[aqid_b[8]][10:0]};
	wire [21:0] wptr_av_c  = {wptr[aqid_av_r1a][21:16], wptr[aqid_av_r1b][15:11], wptr[aqid_av_r0b][10:6], wptr[aqid_av_r0a][5:0]};
	wire [21:0] mask_av_c  = {mask[aqid_av_r3][21:11], mask[aqid_av_r2][10:0]};
`else
	wire [21:0] wptr_act   = wptr[active_qid];
	wire [21:0] rptr_act   = rptr[active_qid];
	wire [63:0] base_act   = base[active_qid];
	wire [21:0] mask_act   = mask[active_qid];
	wire [22:0] nwords_act = nwords[active_qid];
`endif

	/*
	 * Arbiter Selection
	 */
`ifdef SYNTH_KINTEX7
	wire [1:0] rr_next1 = (active_qid == 2'd0) ? 2'd1 : ((active_qid == 2'd1) ? 2'd2 : 2'd0);
	wire [1:0] rr_next2 = (active_qid == 2'd0) ? 2'd2 : ((active_qid == 2'd1) ? 2'd0 : 2'd1);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] rr_next1 = (active_qid == 2'd0) ? 2'd1 : ((active_qid == 2'd1) ? 2'd2 : 2'd0);
	wire [1:0] rr_next2 = (active_qid == 2'd0) ? 2'd2 : ((active_qid == 2'd1) ? 2'd0 : 2'd1);
`else
	wire [1:0] rr_next1 = {active_qid[0], ~active_qid[0] & ~active_qid[1]};
	wire [1:0] rr_next2 = {~active_qid[0] & ~active_qid[1], active_qid[1]};
`endif

	reg [1:0] next_qid;
`ifdef SYNTH_KINTEX7
	reg [2:0] has_work_r;
	(* max_fanout = 12, extract_enable = "no" *) reg q0_empty_r;
	wire [2:0] hw_sel = has_work_r;
	wire [1:0] q0_empty_b, preempt_act_b, aqid_nz_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_q0eb0 (.I0(q0_empty_r), .O(q0_empty_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_q0eb1 (.I0(q0_empty_r), .O(q0_empty_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pab0  (.I0(preempt_active), .O(preempt_act_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pab1  (.I0(preempt_active), .O(preempt_act_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_anz0  (.I0(active_qid[0]), .I1(active_qid[1]), .O(aqid_nz_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_anz1  (.I0(active_qid[0]), .I1(active_qid[1]), .O(aqid_nz_b[1]));
	wire q0_empty_sel   = q0_empty_b[0];
	wire q0_empty_sel_b = q0_empty_b[1];
	wire preempt_act_sel = preempt_act_b[1];
`elsif SYNTH_KINTEX_ULTRASCALE
	reg [2:0] has_work_r;
	(* max_fanout = 12, extract_enable = "no" *) reg q0_empty_r;
	wire [2:0] hw_sel = has_work_r;
	wire [1:0] q0_empty_b, preempt_act_b, aqid_nz_b;
	assign q0_empty_b[0] = q0_empty_r;
	assign q0_empty_b[1] = q0_empty_r;
	assign preempt_act_b[0] = preempt_active;
	assign preempt_act_b[1] = preempt_active;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_anz0  (.I0(active_qid[0]), .I1(active_qid[1]), .O(aqid_nz_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_anz1  (.I0(active_qid[0]), .I1(active_qid[1]), .O(aqid_nz_b[1]));
	wire q0_empty_sel   = q0_empty_b[0];
	wire q0_empty_sel_b = q0_empty_b[1];
	wire preempt_act_sel = preempt_act_b[1];
`else
	reg [2:0] has_work_r;
	reg q0_empty_r;
	wire [2:0] hw_sel = has_work_r;
	wire q0_empty_sel    = q0_empty_r;
	wire q0_empty_sel_b  = q0_empty_r;
	wire preempt_act_sel = preempt_active;
	wire [1:0] q0_empty_b    = {2{q0_empty_r}};
	wire [1:0] preempt_act_b = {2{preempt_active}};
	wire [1:0] aqid_nz_b     = {2{active_qid != 2'd0}};
`endif
	reg pm_strict_r, pm_rr_r;
	always @(*) begin
		if (preempt_active) begin
			next_qid = 2'd0;
		end else if (pm_strict_r) begin
			/* Strict Priority: Q0 > Q1 > Q2 */
			next_qid = hw_sel[0] ? 2'd0 :
				hw_sel[1] ? 2'd1 :
				hw_sel[2] ? 2'd2 : active_qid;
		end else if (pm_rr_r) begin
			/* Pure Round Robin */
			next_qid = hw_sel[rr_next1] ? rr_next1 :
				hw_sel[rr_next2] ? rr_next2 : active_qid;
		end else begin
			/* Balanced: Graphics prioritized, Compute & Copy interleaved */
			next_qid = hw_sel[0] ? 2'd0 :
				(last_non_q0 == 2'd1) ? (hw_sel[2] ? 2'd2 : (hw_sel[1] ? 2'd1 : 2'd0)) :
				(hw_sel[1] ? 2'd1 : (hw_sel[2] ? 2'd2 : 2'd0));
		end
	end

	(* max_fanout = 12, extract_enable = "no" *) reg [3:0] st;
	(* dont_touch = "true" *) reg st_is_rsp_r, pf_st_is_rsp_r;
	reg ret_n;	/* the read is for R_N (else R_HDR) */
	(* max_fanout = 12, extract_enable = "no" *) reg [21:0] cur;	/* the header's dword index */
	reg [15:0] n;	/* dwords after the header */
	(* max_fanout = 16, extract_enable = "yes" *) reg [15:0] k;	/* R_N: dword of the packet on n_data (0 = header) */
	reg [7:0] tag;
	(* max_fanout = 16, extract_enable = "no" *) reg [31:0] hdr_data;
	(* max_fanout = 16 *) reg step_pend;
	(* max_fanout = 16 *) reg stale;
	reg h_bad_r, h_novf_r;
	reg h_bad_words_r, h_bad_avail_r;
	reg [1:0] h_class_r;

	/*
	 * 8-entry circular prefetch line buffer (256 bytes = 64 words for 32B lines)
	 */
	reg [7:0] buf_valid;
	(* max_fanout = 16 *) reg [LA_W-1:0] buf_tag [0:7];
	reg [LINE_BYTES*8-1:0] buf_data [0:7];

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) reg [2:0] pf_idx_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_ok_tag0_r, rsp_ok_tag1_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_is_tag0_r, rsp_is_tag1_r, rsp_is_tag2_r, rsp_is_tag3_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_err_tag0_r, rsp_error_r;
	(* extract_reset = "no" *) reg [LINE_BYTES*8-1:0] rsp_rdata_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			rsp_ok_tag0_r  <= 1'b0;
			rsp_ok_tag1_r  <= 1'b0;
			rsp_is_tag0_r  <= 1'b0;
			rsp_is_tag1_r  <= 1'b0;
			rsp_is_tag2_r  <= 1'b0;
			rsp_is_tag3_r  <= 1'b0;
			rsp_err_tag0_r <= 1'b0;
			rsp_error_r    <= 1'b0;
			rsp_rdata_r    <= {(LINE_BYTES*8){1'b0}};
		end else begin
			rsp_ok_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0) && !rsp_error;
			rsp_ok_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1) && !rsp_error;
			rsp_is_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0);
			rsp_is_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1);
			rsp_is_tag2_r  <= rsp_valid && (rsp_tag[1:0] == 2'd2);
			rsp_is_tag3_r  <= rsp_valid && (rsp_tag[1:0] == 2'd3);
			rsp_err_tag0_r <= rsp_valid && (rsp_tag[1:0] == 2'd0) && rsp_error;
			rsp_error_r    <= rsp_error;
			rsp_rdata_r    <= rsp_rdata;
		end
	end
	wire rsp_ok_tag1 = rsp_ok_tag1_r;
	wire rsp_ok_tag0 = rsp_ok_tag0_r;
	wire pf_wr_lo, pf_wr_hi, dm_wr_lo, dm_wr_hi;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pfw_lo (.I0(pf_st_is_rsp_r), .I1(rsp_ok_tag1), .O(pf_wr_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pfw_hi (.I0(pf_st_is_rsp_r), .I1(rsp_ok_tag1), .O(pf_wr_hi));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmw_lo (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmw_hi (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_hi));
	wire [1:0] dm_wr_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmwb0 (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmwb1 (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_b[1]));
	wire [LA_W-1:0] wr_line_mux;
	genvar g_wlm;
	generate
		for (g_wlm = 0; g_wlm < LA_W; g_wlm = g_wlm + 1) begin : g_wlm_gen
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_wlm (
				.I0(pf_line[g_wlm]),
				.I1(want_line[g_wlm]),
				.I2(dm_wr_b[g_wlm >= 14 ? 1 : 0]),
				.O(wr_line_mux[g_wlm])
			);
		end
	endgenerate
	wire [15:0] buf_slice_we [0:7];
	wire [7:0] bv_pf_hit, bv_dm_hit;
	genvar g_bi, g_sl;
	generate
		for (g_bi = 0; g_bi < 8; g_bi = g_bi + 1) begin : g_buf_wr
			wire pf_hit_lo, pf_hit_hi, dm_hit_lo, dm_hit_hi;
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_pfh_lo (
				.I0(pf_idx[0]), .I1(pf_idx[1]), .I2(pf_idx[2]), .I3(rsp_ok_tag1), .I4(pf_st_is_rsp_r), .O(pf_hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_pfh_hi (
				.I0(pf_idx_r[0]), .I1(pf_idx_r[1]), .I2(pf_idx_r[2]), .I3(rsp_ok_tag1), .I4(pf_st_is_rsp_r), .O(pf_hit_hi)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_dmh_lo (
				.I0(pa_widx_lo[0]), .I1(pa_widx_lo[1]), .I2(pa_widx_lo[2]), .I3(rsp_ok_tag0), .I4(st_is_rsp_r), .O(dm_hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_dmh_hi (
				.I0(pa_widx_hi[0]), .I1(pa_widx_hi[1]), .I2(pa_widx_hi[2]), .I3(rsp_ok_tag0), .I4(st_is_rsp_r), .O(dm_hit_hi)
			);
			assign bv_pf_hit[g_bi] = pf_hit_lo;
			assign bv_dm_hit[g_bi] = dm_hit_lo;
			for (g_sl = 0; g_sl < 16; g_sl = g_sl + 1) begin : g_sl_gen
				(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_sl_we (
					.I0(g_sl < 8 ? pf_hit_lo : pf_hit_hi),
					.I1(g_sl < 8 ? dm_hit_lo : dm_hit_hi),
					.O(buf_slice_we[g_bi][g_sl])
				);
				always @(posedge clk) begin
					if (buf_slice_we[g_bi][g_sl])
						buf_data[g_bi][g_sl*16 +: 16] <= rsp_rdata_r[g_sl*16 +: 16];
				end
			end
			wire [1:0] tag_sl_we;
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_tl_we0 (.I0(pf_hit_lo), .I1(dm_hit_lo), .O(tag_sl_we[0]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_tl_we1 (.I0(pf_hit_hi), .I1(dm_hit_hi), .O(tag_sl_we[1]));
			always @(posedge clk) begin
				if (tag_sl_we[0]) buf_tag[g_bi][13:0] <= wr_line_mux[13:0];
				if (tag_sl_we[1]) buf_tag[g_bi][LA_W-1:14] <= wr_line_mux[LA_W-1:14];
			end
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* DONT_TOUCH = "TRUE" *) reg [2:0] pf_idx_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_ok_tag0_r, rsp_ok_tag1_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_is_tag0_r, rsp_is_tag1_r, rsp_is_tag2_r, rsp_is_tag3_r;
	(* DONT_TOUCH = "TRUE" *) reg rsp_err_tag0_r, rsp_error_r;
	(* extract_reset = "no" *) reg [LINE_BYTES*8-1:0] rsp_rdata_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			rsp_ok_tag0_r  <= 1'b0;
			rsp_ok_tag1_r  <= 1'b0;
			rsp_is_tag0_r  <= 1'b0;
			rsp_is_tag1_r  <= 1'b0;
			rsp_is_tag2_r  <= 1'b0;
			rsp_is_tag3_r  <= 1'b0;
			rsp_err_tag0_r <= 1'b0;
			rsp_error_r    <= 1'b0;
			rsp_rdata_r    <= {(LINE_BYTES*8){1'b0}};
		end else begin
			rsp_ok_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0) && !rsp_error;
			rsp_ok_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1) && !rsp_error;
			rsp_is_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0);
			rsp_is_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1);
			rsp_is_tag2_r  <= rsp_valid && (rsp_tag[1:0] == 2'd2);
			rsp_is_tag3_r  <= rsp_valid && (rsp_tag[1:0] == 2'd3);
			rsp_err_tag0_r <= rsp_valid && (rsp_tag[1:0] == 2'd0) && rsp_error;
			rsp_error_r    <= rsp_error;
			rsp_rdata_r    <= rsp_rdata;
		end
	end
	wire rsp_ok_tag1 = rsp_ok_tag1_r;
	wire rsp_ok_tag0 = rsp_ok_tag0_r;
	wire pf_wr_lo, pf_wr_hi, dm_wr_lo, dm_wr_hi;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pfw_lo (.I0(pf_st_is_rsp_r), .I1(rsp_ok_tag1), .O(pf_wr_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pfw_hi (.I0(pf_st_is_rsp_r), .I1(rsp_ok_tag1), .O(pf_wr_hi));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmw_lo (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmw_hi (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_hi));
	wire [1:0] dm_wr_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmwb0 (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_dmwb1 (.I0(st_is_rsp_r), .I1(rsp_ok_tag0), .O(dm_wr_b[1]));
	wire [LA_W-1:0] wr_line_mux;
	genvar g_wlm;
	generate
		for (g_wlm = 0; g_wlm < LA_W; g_wlm = g_wlm + 1) begin : g_wlm_gen
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_wlm (
				.I0(pf_line[g_wlm]),
				.I1(want_line[g_wlm]),
				.I2(dm_wr_b[g_wlm >= 14 ? 1 : 0]),
				.O(wr_line_mux[g_wlm])
			);
		end
	endgenerate
	wire [15:0] buf_slice_we [0:7];
	wire [7:0] bv_pf_hit, bv_dm_hit;
	genvar g_bi, g_sl;
	generate
		for (g_bi = 0; g_bi < 8; g_bi = g_bi + 1) begin : g_buf_wr
			wire pf_hit_lo, pf_hit_hi, dm_hit_lo, dm_hit_hi;
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_pfh_lo (
				.I0(pf_idx[0]), .I1(pf_idx[1]), .I2(pf_idx[2]), .I3(rsp_ok_tag1), .I4(pf_st_is_rsp_r), .O(pf_hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_pfh_hi (
				.I0(pf_idx_r[0]), .I1(pf_idx_r[1]), .I2(pf_idx_r[2]), .I3(rsp_ok_tag1), .I4(pf_st_is_rsp_r), .O(pf_hit_hi)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_dmh_lo (
				.I0(pa_widx_lo[0]), .I1(pa_widx_lo[1]), .I2(pa_widx_lo[2]), .I3(rsp_ok_tag0), .I4(st_is_rsp_r), .O(dm_hit_lo)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000 << g_bi)) u_dmh_hi (
				.I0(pa_widx_hi[0]), .I1(pa_widx_hi[1]), .I2(pa_widx_hi[2]), .I3(rsp_ok_tag0), .I4(st_is_rsp_r), .O(dm_hit_hi)
			);
			assign bv_pf_hit[g_bi] = pf_hit_lo;
			assign bv_dm_hit[g_bi] = dm_hit_lo;
			for (g_sl = 0; g_sl < 16; g_sl = g_sl + 1) begin : g_sl_gen
				(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_sl_we (
					.I0(g_sl < 8 ? pf_hit_lo : pf_hit_hi),
					.I1(g_sl < 8 ? dm_hit_lo : dm_hit_hi),
					.O(buf_slice_we[g_bi][g_sl])
				);
				always @(posedge clk) begin
					if (buf_slice_we[g_bi][g_sl])
						buf_data[g_bi][g_sl*16 +: 16] <= rsp_rdata_r[g_sl*16 +: 16];
				end
			end
			wire [1:0] tag_sl_we;
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_tl_we0 (.I0(pf_hit_lo), .I1(dm_hit_lo), .O(tag_sl_we[0]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_tl_we1 (.I0(pf_hit_hi), .I1(dm_hit_hi), .O(tag_sl_we[1]));
			always @(posedge clk) begin
				if (tag_sl_we[0]) buf_tag[g_bi][13:0] <= wr_line_mux[13:0];
				if (tag_sl_we[1]) buf_tag[g_bi][LA_W-1:14] <= wr_line_mux[LA_W-1:14];
			end
		end
	endgenerate
`else
	reg [2:0] pf_idx_r;
	reg rsp_ok_tag0_r, rsp_ok_tag1_r;
	reg rsp_is_tag0_r, rsp_is_tag1_r, rsp_is_tag2_r, rsp_is_tag3_r;
	reg rsp_err_tag0_r, rsp_error_r;
	reg [LINE_BYTES*8-1:0] rsp_rdata_r;
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n) begin
			rsp_ok_tag0_r  <= 1'b0;
			rsp_ok_tag1_r  <= 1'b0;
			rsp_is_tag0_r  <= 1'b0;
			rsp_is_tag1_r  <= 1'b0;
			rsp_is_tag2_r  <= 1'b0;
			rsp_is_tag3_r  <= 1'b0;
			rsp_err_tag0_r <= 1'b0;
			rsp_error_r    <= 1'b0;
			rsp_rdata_r    <= {(LINE_BYTES*8){1'b0}};
		end else begin
			rsp_ok_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0) && !rsp_error;
			rsp_ok_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1) && !rsp_error;
			rsp_is_tag0_r  <= rsp_valid && (rsp_tag[1:0] == 2'd0);
			rsp_is_tag1_r  <= rsp_valid && (rsp_tag[1:0] == 2'd1);
			rsp_is_tag2_r  <= rsp_valid && (rsp_tag[1:0] == 2'd2);
			rsp_is_tag3_r  <= rsp_valid && (rsp_tag[1:0] == 2'd3);
			rsp_err_tag0_r <= rsp_valid && (rsp_tag[1:0] == 2'd0) && rsp_error;
			rsp_error_r    <= rsp_error;
			rsp_rdata_r    <= rsp_rdata;
		end
	end
	wire rsp_ok_tag1 = rsp_ok_tag1_r;
	wire rsp_ok_tag0 = rsp_ok_tag0_r;
	wire pf_wr_lo = pf_st_is_rsp_r && rsp_ok_tag1;
	wire dm_wr_lo = st_is_rsp_r && rsp_ok_tag0;
	wire [LA_W-1:0] wr_line_mux = dm_wr_lo ? want_line : pf_line;
`endif

	/*
	 * The dword wanted as byte address in memory
	 */
`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [ADDR_W-1:0] pa;
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_word_r [0:31];
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_idx_r [0:7];
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_widx_r [0:1];
	wire [2:0] pa_widx_lo, pa_widx_hi;
	genvar g_pwl;
	generate
		for (g_pwl = 0; g_pwl < 3; g_pwl = g_pwl + 1) begin : g_pwidx_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pwlo (.I0(pa_widx_r[1][g_pwl]), .O(pa_widx_lo[g_pwl]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pwhi (.I0(pa_widx_r[1][g_pwl]), .O(pa_widx_hi[g_pwl]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [ADDR_W-1:0] pa;
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_word_r [0:31];
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_idx_r [0:7];
	(* keep = "true", equivalent_register_removal = "no" *) reg [2:0] pa_widx_r [0:1];
	wire [2:0] pa_widx_lo, pa_widx_hi;
	genvar g_pwl;
	generate
		for (g_pwl = 0; g_pwl < 3; g_pwl = g_pwl + 1) begin : g_pwidx_buf
			assign pa_widx_lo[g_pwl] = pa_widx_r[1][g_pwl];
			assign pa_widx_hi[g_pwl] = pa_widx_r[1][g_pwl];
		end
	endgenerate
`else
	(* max_fanout = 16 *) reg [ADDR_W-1:0] pa;
	reg [WS-1:0] pa_word_r [0:3];
	reg [2:0]    pa_idx_r  [0:3];
	reg [2:0]    pa_widx_r;
	wire [2:0]   pa_widx_lo = pa_widx_r;
`endif
	wire [ADDR_W-1:0] want_addr = pa;
	wire [LA_W-1:0] want_line = want_addr[ADDR_W-1:LINE_SHIFT];
	wire [WS-1:0] want_word = want_addr[LINE_SHIFT-1:2];
	wire [2:0] want_idx = want_line[2:0];

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [7:0] bv_pf_hit, bv_dm_hit;
	genvar g_bi, g_sl;
	generate
		for (g_bi = 0; g_bi < 8; g_bi = g_bi + 1) begin : g_buf_wr
			wire pf_hit_w = pf_wr_lo && (pf_idx == g_bi[2:0]);
			wire dm_hit_w = dm_wr_lo && (pa_widx_lo == g_bi[2:0]);
			assign bv_pf_hit[g_bi] = pf_hit_w;
			assign bv_dm_hit[g_bi] = dm_hit_w;
			for (g_sl = 0; g_sl < NSL; g_sl = g_sl + 1) begin : g_sl_gen
				always @(posedge clk) begin
					if (pf_hit_w || dm_hit_w)
						buf_data[g_bi][g_sl*32 +: 32] <= rsp_rdata_r[g_sl*32 +: 32];
				end
			end
			always @(posedge clk) begin
				if (pf_hit_w || dm_hit_w)
					buf_tag[g_bi] <= wr_line_mux;
			end
		end
	endgenerate
`endif

`ifdef SYNTH_KINTEX7
	wire [2:0] widx_tag0, widx_tag1, widx_tag2, widx_tag3;
	genvar g_wt;
	generate
		for (g_wt = 0; g_wt < 3; g_wt = g_wt + 1) begin : g_widx_tag_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wt0 (.I0(pa_widx_r[0][g_wt]), .O(widx_tag0[g_wt]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wt1 (.I0(pa_widx_r[0][g_wt]), .O(widx_tag1[g_wt]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wt2 (.I0(pa_widx_r[0][g_wt]), .O(widx_tag2[g_wt]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wt3 (.I0(pa_widx_r[0][g_wt]), .O(widx_tag3[g_wt]));
		end
	endgenerate

	reg [LA_W-1:0] cur_tag_r;
	reg cur_valid_r;
	wire [8:0] cm_grp;
	wire cur_hit_lo, cur_hit_hi, cur_hit_comb;
	genvar g_cm;
	generate
		for (g_cm = 0; g_cm < 9; g_cm = g_cm + 1) begin : gen_cm_grp
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_cmg (
				.I0(cur_tag_r[g_cm*3 + 0]), .I1(cur_tag_r[g_cm*3 + 1]), .I2(cur_tag_r[g_cm*3 + 2]),
				.I3(want_line[g_cm*3 + 0]), .I4(want_line[g_cm*3 + 1]), .I5(want_line[g_cm*3 + 2]),
				.O(cm_grp[g_cm])
			);
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_ch_lo (
		.I0(cm_grp[0]), .I1(cm_grp[1]), .I2(cm_grp[2]), .I3(cm_grp[3]), .I4(cm_grp[4]),
		.O(cur_hit_lo)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_ch_hi (
		.I0(cm_grp[5]), .I1(cm_grp[6]), .I2(cm_grp[7]), .I3(cm_grp[8]), .I4(cur_valid_r),
		.O(cur_hit_hi)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_ch_all (
		.I0(cur_hit_lo), .I1(cur_hit_hi), .O(cur_hit_comb)
	);

	wire [31:0] line_word [0:7];
	wire [31:0] want_data;
	genvar g_ln, g_by, g_wb2, g_bt;
	generate
		for (g_ln = 0; g_ln < 8; g_ln = g_ln + 1) begin : gen_line_word
			for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_line_byte
				wire [2:0] w_sel;
				for (g_wb2 = 0; g_wb2 < 3; g_wb2 = g_wb2 + 1) begin : gen_wsel_bit
					assign w_sel[g_wb2] = pa_word_r[g_ln*4 + g_by][g_wb2];
				end
				for (g_bt = 0; g_bt < 8; g_bt = g_bt + 1) begin : gen_lw_bit
					wire lw_lo, lw_hi;
					(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_lw_lo (
						.I0(buf_data[g_ln][0*32 + g_by*8 + g_bt]),
						.I1(buf_data[g_ln][1*32 + g_by*8 + g_bt]),
						.I2(buf_data[g_ln][2*32 + g_by*8 + g_bt]),
						.I3(buf_data[g_ln][3*32 + g_by*8 + g_bt]),
						.I4(w_sel[0]),
						.I5(w_sel[1]),
						.O(lw_lo)
					);
					(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_lw_hi (
						.I0(buf_data[g_ln][4*32 + g_by*8 + g_bt]),
						.I1(buf_data[g_ln][5*32 + g_by*8 + g_bt]),
						.I2(buf_data[g_ln][6*32 + g_by*8 + g_bt]),
						.I3(buf_data[g_ln][7*32 + g_by*8 + g_bt]),
						.I4(w_sel[0]),
						.I5(w_sel[1]),
						.O(lw_hi)
					);
					(* DONT_TOUCH = "TRUE" *) MUXF7 u_lw_f7 (
						.I0(lw_lo),
						.I1(lw_hi),
						.S(w_sel[2]),
						.O(line_word[g_ln][g_by*8 + g_bt])
					);
				end
			end
		end
		for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_want_data
			for (g_bt = 0; g_bt < 8; g_bt = g_bt + 1) begin : gen_wd_bit
				localparam IDX_COPY = g_by*2 + (g_bt >= 4 ? 1 : 0);
				wire wd_lo, wd_hi;
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_wd_lo (
					.I0(line_word[0][g_by*8 + g_bt]),
					.I1(line_word[1][g_by*8 + g_bt]),
					.I2(line_word[2][g_by*8 + g_bt]),
					.I3(line_word[3][g_by*8 + g_bt]),
					.I4(pa_idx_r[IDX_COPY][0]),
					.I5(pa_idx_r[IDX_COPY][1]),
					.O(wd_lo)
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_wd_hi (
					.I0(line_word[4][g_by*8 + g_bt]),
					.I1(line_word[5][g_by*8 + g_bt]),
					.I2(line_word[6][g_by*8 + g_bt]),
					.I3(line_word[7][g_by*8 + g_bt]),
					.I4(pa_idx_r[IDX_COPY][0]),
					.I5(pa_idx_r[IDX_COPY][1]),
					.O(wd_hi)
				);
				(* DONT_TOUCH = "TRUE" *) MUXF7 u_wd_f7 (
					.I0(wd_lo),
					.I1(wd_hi),
					.S(pa_idx_r[IDX_COPY][2]),
					.O(want_data[g_by*8 + g_bt])
				);
			end
		end
	endgenerate
	(* max_fanout = 16 *) reg hit_n_r;
	reg [1:0] hdr_wait;
	wire hit_n = hit_n_r;
	wire hit = hit_n_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] widx_tag0, widx_tag1, widx_tag2, widx_tag3;
	genvar g_wt;
	generate
		for (g_wt = 0; g_wt < 3; g_wt = g_wt + 1) begin : g_widx_tag_buf
			assign widx_tag0[g_wt] = pa_widx_r[0][g_wt];
			assign widx_tag1[g_wt] = pa_widx_r[0][g_wt];
			assign widx_tag2[g_wt] = pa_widx_r[0][g_wt];
			assign widx_tag3[g_wt] = pa_widx_r[0][g_wt];
		end
	endgenerate

	reg [LA_W-1:0] cur_tag_r;
	reg cur_valid_r;
	wire [8:0] cm_grp;
	wire cur_hit_lo, cur_hit_hi, cur_hit_comb;
	genvar g_cm;
	generate
		for (g_cm = 0; g_cm < 9; g_cm = g_cm + 1) begin : gen_cm_grp
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_cmg (
				.I0(cur_tag_r[g_cm*3 + 0]), .I1(cur_tag_r[g_cm*3 + 1]), .I2(cur_tag_r[g_cm*3 + 2]),
				.I3(want_line[g_cm*3 + 0]), .I4(want_line[g_cm*3 + 1]), .I5(want_line[g_cm*3 + 2]),
				.O(cm_grp[g_cm])
			);
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_ch_lo (
		.I0(cm_grp[0]), .I1(cm_grp[1]), .I2(cm_grp[2]), .I3(cm_grp[3]), .I4(cm_grp[4]),
		.O(cur_hit_lo)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_ch_hi (
		.I0(cm_grp[5]), .I1(cm_grp[6]), .I2(cm_grp[7]), .I3(cm_grp[8]), .I4(cur_valid_r),
		.O(cur_hit_hi)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_ch_all (
		.I0(cur_hit_lo), .I1(cur_hit_hi), .O(cur_hit_comb)
	);

	wire [31:0] line_word [0:7];
	wire [31:0] want_data;
	genvar g_ln, g_by, g_wb2, g_bt;
	generate
		for (g_ln = 0; g_ln < 8; g_ln = g_ln + 1) begin : gen_line_word
			for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_line_byte
				wire [2:0] w_sel;
				for (g_wb2 = 0; g_wb2 < 3; g_wb2 = g_wb2 + 1) begin : gen_wsel_bit
					assign w_sel[g_wb2] = pa_word_r[g_ln*4 + g_by][g_wb2];
				end
				for (g_bt = 0; g_bt < 8; g_bt = g_bt + 1) begin : gen_lw_bit
					wire lw_lo, lw_hi;
					(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_lw_lo (
						.I0(buf_data[g_ln][0*32 + g_by*8 + g_bt]),
						.I1(buf_data[g_ln][1*32 + g_by*8 + g_bt]),
						.I2(buf_data[g_ln][2*32 + g_by*8 + g_bt]),
						.I3(buf_data[g_ln][3*32 + g_by*8 + g_bt]),
						.I4(w_sel[0]),
						.I5(w_sel[1]),
						.O(lw_lo)
					);
					(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_lw_hi (
						.I0(buf_data[g_ln][4*32 + g_by*8 + g_bt]),
						.I1(buf_data[g_ln][5*32 + g_by*8 + g_bt]),
						.I2(buf_data[g_ln][6*32 + g_by*8 + g_bt]),
						.I3(buf_data[g_ln][7*32 + g_by*8 + g_bt]),
						.I4(w_sel[0]),
						.I5(w_sel[1]),
						.O(lw_hi)
					);
					(* DONT_TOUCH = "TRUE" *) MUXF7 u_lw_f7 (
						.I0(lw_lo),
						.I1(lw_hi),
						.S(w_sel[2]),
						.O(line_word[g_ln][g_by*8 + g_bt])
					);
				end
			end
		end
		for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_want_data
			for (g_bt = 0; g_bt < 8; g_bt = g_bt + 1) begin : gen_wd_bit
				localparam IDX_COPY = g_by*2 + (g_bt >= 4 ? 1 : 0);
				wire wd_lo, wd_hi;
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_wd_lo (
					.I0(line_word[0][g_by*8 + g_bt]),
					.I1(line_word[1][g_by*8 + g_bt]),
					.I2(line_word[2][g_by*8 + g_bt]),
					.I3(line_word[3][g_by*8 + g_bt]),
					.I4(pa_idx_r[IDX_COPY][0]),
					.I5(pa_idx_r[IDX_COPY][1]),
					.O(wd_lo)
				);
				(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_wd_hi (
					.I0(line_word[4][g_by*8 + g_bt]),
					.I1(line_word[5][g_by*8 + g_bt]),
					.I2(line_word[6][g_by*8 + g_bt]),
					.I3(line_word[7][g_by*8 + g_bt]),
					.I4(pa_idx_r[IDX_COPY][0]),
					.I5(pa_idx_r[IDX_COPY][1]),
					.O(wd_hi)
				);
				(* DONT_TOUCH = "TRUE" *) MUXF7 u_wd_f7 (
					.I0(wd_lo),
					.I1(wd_hi),
					.S(pa_idx_r[IDX_COPY][2]),
					.O(want_data[g_by*8 + g_bt])
				);
			end
		end
	endgenerate
	(* max_fanout = 16 *) reg hit_n_r;
	reg [1:0] hdr_wait;
	wire hit_n = hit_n_r;
	wire hit = hit_n_r;
`else
	reg [LA_W-1:0] cur_tag_r;
	reg cur_valid_r;
	wire cur_hit_comb = cur_valid_r && (cur_tag_r == want_line);

	wire [31:0] line_word [0:7];
	wire [31:0] want_data;
	genvar g_ln, g_by;
	generate
		for (g_ln = 0; g_ln < 8; g_ln = g_ln + 1) begin : gen_line_word
			for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_line_byte
				localparam [1:0] BY_OFF = g_by;
				wire [WS-1:0] w_sel = pa_word_r[g_by];
				assign line_word[g_ln][g_by*8 +: 8] = buf_data[g_ln][{w_sel, BY_OFF, 3'd0} +: 8];
			end
		end
		for (g_by = 0; g_by < 4; g_by = g_by + 1) begin : gen_want_data
			wire [2:0] bidx = pa_idx_r[g_by];
			assign want_data[g_by*8 +: 8] = line_word[bidx][g_by*8 +: 8];
		end
	endgenerate
	reg hit_n_r;
	reg [1:0] hdr_wait;
	wire hit_n = hit_n_r;
	wire hit   = hit_n_r;
`endif

`ifdef SYNTH_KINTEX7
	wire ctx_base_zero = ctx_base_z_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire ctx_base_zero = ctx_base_z_r;
`else
	wire ctx_base_zero = (ctx_base == 32'd0);
`endif
	wire go = (run && !halt) || step_pend;
`ifdef SYNTH_KINTEX7
	wire [1:0] go_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_gob0 (.I0(go), .O(go_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_gob1 (.I0(go), .O(go_b[1]));
	wire go_sel = go;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] go_b;
	assign go_b[0] = go;
	assign go_b[1] = go;
	wire go_sel = go;
`else
	wire go_sel = go;
`endif

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) reg [4:0] hcr_we_cfg_b;
	(* dont_touch = "true" *) reg hcr_we_db, hcr_we_dma, hcr_we_sync;
	(* dont_touch = "true" *) reg [9:0] widx_a, widx_b;
	(* dont_touch = "true" *) reg [2:0] doorbell_r, doorbell_w_r;
	wire hcr_we_lo64 = hcr_we && (hcr_widx[9:6] == 4'd0);
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			hcr_we_cfg_b <= 5'd0;
			hcr_we_db    <= 1'b0;
			hcr_we_dma   <= 1'b0;
			hcr_we_sync  <= 1'b0;
			widx_a       <= 10'd0;
			widx_b       <= 10'd0;
			doorbell_r   <= 3'd0;
			doorbell_w_r <= 3'd0;
		end else begin
			hcr_we_cfg_b[3:0] <= {4{hcr_we_lo64 && open}};
			hcr_we_cfg_b[4]   <= hcr_we_lo64;
			hcr_we_db         <= hcr_we_lo64;
			hcr_we_dma        <= hcr_we_lo64;
			hcr_we_sync       <= hcr_we_lo64;
			widx_a            <= hcr_widx;
			widx_b            <= hcr_widx;
			doorbell_r[0]     <= hcr_we_lo64 && (hcr_widx[5:0] == I_WPTR0[5:0]);
			doorbell_r[1]     <= 1'b0;
			doorbell_r[2]     <= 1'b0;
			doorbell_w_r[0]   <= hcr_we_lo64 && (hcr_widx[5:0] == I_WPTR0[5:0]);
			doorbell_w_r[1]   <= 1'b0;
			doorbell_w_r[2]   <= 1'b0;
		end
	end
	wire [2:0] doorbell = doorbell_r;
	wire [2:0] doorbell_w = doorbell_w_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) reg [4:0] hcr_we_cfg_b;
	(* dont_touch = "true" *) reg hcr_we_db, hcr_we_dma, hcr_we_sync;
	(* dont_touch = "true" *) reg [9:0] widx_a, widx_b;
	(* dont_touch = "true" *) reg [2:0] doorbell_r, doorbell_w_r;
	wire hcr_we_lo64 = hcr_we && (hcr_widx[9:6] == 4'd0);
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			hcr_we_cfg_b <= 5'd0;
			hcr_we_db    <= 1'b0;
			hcr_we_dma   <= 1'b0;
			hcr_we_sync  <= 1'b0;
			widx_a       <= 10'd0;
			widx_b       <= 10'd0;
			doorbell_r   <= 3'd0;
			doorbell_w_r <= 3'd0;
		end else begin
			hcr_we_cfg_b[3:0] <= {4{hcr_we_lo64 && open}};
			hcr_we_cfg_b[4]   <= hcr_we_lo64;
			hcr_we_db         <= hcr_we_lo64;
			hcr_we_dma        <= hcr_we_lo64;
			hcr_we_sync       <= hcr_we_lo64;
			widx_a            <= hcr_widx;
			widx_b            <= hcr_widx;
			doorbell_r[0]     <= hcr_we_lo64 && (hcr_widx[5:0] == I_WPTR0[5:0]);
			doorbell_r[1]     <= 1'b0;
			doorbell_r[2]     <= 1'b0;
			doorbell_w_r[0]   <= hcr_we_lo64 && (hcr_widx[5:0] == I_WPTR0[5:0]);
			doorbell_w_r[1]   <= 1'b0;
			doorbell_w_r[2]   <= 1'b0;
		end
	end
	wire [2:0] doorbell = doorbell_r;
	wire [2:0] doorbell_w = doorbell_w_r;
`else
	(* keep = "true" *) reg hcr_we_cfg;
	(* keep = "true" *) reg [9:0] widx_a, widx_b;
	(* keep = "true" *) reg [2:0] doorbell;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			hcr_we_cfg <= 1'b0;
			widx_a     <= 10'd0;
			widx_b     <= 10'd0;
			doorbell   <= 3'd0;
		end else begin
			hcr_we_cfg <= hcr_we && (hcr_widx[9:6] == 4'd0);
			widx_a     <= hcr_widx;
			widx_b     <= hcr_widx;
			doorbell[0] <= hcr_we && (hcr_widx[9:6] == 4'd0) && (hcr_widx[5:0] == I_WPTR0[5:0]);
			doorbell[1] <= 1'b0;
			doorbell[2] <= 1'b0;
		end
	end
	wire [2:0] doorbell_w = doorbell;
`endif

	wire [21:0] wptr_new0 = ((wptr[0] & ~wmask_c[21:0]) | (wdata_c[21:0] & wmask_c[21:0])) & mask[0];
	wire [21:0] wptr_new1 = ((wptr[1] & ~wmask_c[21:0]) | (wdata_c[21:0] & wmask_c[21:0])) & mask[1];
	wire [21:0] wptr_new2 = ((wptr[2] & ~wmask_c[21:0]) | (wdata_c[21:0] & wmask_c[21:0])) & mask[2];

	/* Header decode (from registered hdr_data) — flat 3-LUT decode */
	wire [7:0] hd_op7 = hdr_data[7:0];
	wire hd_10_15 = (hd_op7[7:4] == 4'h1) && !hd_op7[3] && !(hd_op7[2] & hd_op7[1]);
	wire hd_20_37 = (hd_op7[7:5] == 3'b001) && !(hd_op7[4] & hd_op7[3]);
	wire hd_40_46 = (hd_op7[7:3] == 5'b01000) && !(&hd_op7[2:0]);
	wire hd_50_51 = (hd_op7[7:1] == 7'b0101000);
`ifdef SYNTH_KINTEX7
	wire hd_pa_00 = (hd_op7[7:6] == 2'b00);
	wire hd_pb_010 = (hd_op7[7:5] == 3'b010);
	wire hd_t_low  = hd_pa_00 && ((!hd_op7[5] && hd_op7[4] && !hd_op7[3] && !(hd_op7[2] & hd_op7[1])) || (hd_op7[5] && !(hd_op7[4] & hd_op7[3])));
	wire hd_t_high = hd_pb_010 && ((!hd_op7[4] && !hd_op7[3] && !(&hd_op7[2:0])) || (hd_op7[4] && !hd_op7[3] && !hd_op7[2] && !hd_op7[1]));
	wire hd_nz_lo = |hdr_data[11:8];
	wire hd_nz_hi = |hdr_data[15:12];
	wire is_ghci = (hd_t_low || hd_t_high) && (hd_nz_lo || hd_nz_hi);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire hd_pa_00 = (hd_op7[7:6] == 2'b00);
	wire hd_pb_010 = (hd_op7[7:5] == 3'b010);
	wire hd_t_low  = hd_pa_00 && ((!hd_op7[5] && hd_op7[4] && !hd_op7[3] && !(hd_op7[2] & hd_op7[1])) || (hd_op7[5] && !(hd_op7[4] & hd_op7[3])));
	wire hd_t_high = hd_pb_010 && ((!hd_op7[4] && !hd_op7[3] && !(&hd_op7[2:0])) || (hd_op7[4] && !hd_op7[3] && !hd_op7[2] && !hd_op7[1]));
	wire hd_nz_lo = |hdr_data[11:8];
	wire hd_nz_hi = |hdr_data[15:12];
	wire is_ghci = (hd_t_low || hd_t_high) && (hd_nz_lo || hd_nz_hi);
`else
	wire is_ghci = (hd_10_15 || hd_20_37 || hd_40_46 || hd_50_51) && (|hdr_data[15:8]);
`endif
	wire [7:0] ghci_n = fn_dec8(hdr_data[15:8]);
	wire ghci_n_bo = 1'b0;
`ifdef SYNTH_KINTEX7
	wire [2:0] is_ghci_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_igh0 (.I0(is_ghci), .O(is_ghci_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_igh1 (.I0(is_ghci), .O(is_ghci_b[1]));
	assign is_ghci_b[2] = is_ghci;
	wire [1:0] h_class = is_ghci_b[0] ? 2'd0 : hdr_data[31:30];
	wire [15:0] h_n = is_ghci_b[1] ? {8'd0, ghci_n} : (h_class[1] ? hdr_data[15:0] : {8'd0, hdr_data[7:0]});
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] is_ghci_b;
	assign is_ghci_b[0] = is_ghci;
	assign is_ghci_b[1] = is_ghci;
	assign is_ghci_b[2] = is_ghci;
	wire [1:0] h_class = is_ghci_b[0] ? 2'd0 : hdr_data[31:30];
	wire [15:0] h_n = is_ghci_b[1] ? {8'd0, ghci_n} : (h_class[1] ? hdr_data[15:0] : {8'd0, hdr_data[7:0]});
`else
	wire [1:0] h_class = is_ghci ? 2'd0 : hdr_data[31:30];
	wire [15:0] h_n = is_ghci ? {8'd0, ghci_n} : (h_class[1] ? hdr_data[15:0] : {8'd0, hdr_data[7:0]});
`endif

`ifdef SYNTH_KINTEX7
	reg [21:0] mask_av_r;
	reg [22:0] nwords_act_r;
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			mask_av_r    <= 22'd0;
			nwords_act_r <= 23'd0;
		end else begin
			mask_av_r    <= mask_av_c;
			nwords_act_r <= nwords_act;
		end
	end

	wire [8:0] av_lo8_w   = fn_add8_k7_c1(wptr_av_c[7:0],   ~cur[7:0]);
	wire [8:0] av_mid0_w  = fn_add8_k7_c0(wptr_av_c[15:8],  ~cur[15:8]);
	wire [8:0] av_mid1_w  = fn_add8_k7_c1(wptr_av_c[15:8],  ~cur[15:8]);
	wire [8:0] av_hi0_w   = fn_add8_k7_c0({2'b00, wptr_av_c[21:16]}, {2'b00, ~cur[21:16]});
	wire [8:0] av_hi1_w   = fn_add8_k7_c1({2'b00, wptr_av_c[21:16]}, {2'b00, ~cur[21:16]});

	wire [8:0] rem_lo8_w  = fn_add8_k7_c0(nwords_act_r[7:0],  ~cur[7:0]);
	wire [8:0] rem_mid0_w = fn_add8_k7_c0(nwords_act_r[15:8], ~cur[15:8]);
	wire [8:0] rem_mid1_w = fn_add8_k7_c1(nwords_act_r[15:8], ~cur[15:8]);
	wire [8:0] rem_hi0_w  = fn_add8_k7_c0({1'b0, nwords_act_r[22:16]}, {2'b11, ~cur[21:16]});
	wire [8:0] rem_hi1_w  = fn_add8_k7_c1({1'b0, nwords_act_r[22:16]}, {2'b11, ~cur[21:16]});

	reg [8:0] av_lo8_r, av_mid0_r, av_mid1_r;
	reg [5:0] av_hi0_r, av_hi1_r;
	reg [8:0] rem_lo8_r, rem_mid0_r, rem_mid1_r;
	reg [7:0] rem_hi0_r, rem_hi1_r;
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			av_lo8_r   <= 9'd0;
			av_mid0_r  <= 9'd0;
			av_mid1_r  <= 9'd0;
			av_hi0_r   <= 6'd0;
			av_hi1_r   <= 6'd0;
			rem_lo8_r  <= 9'd0;
			rem_mid0_r <= 9'd0;
			rem_mid1_r <= 9'd0;
			rem_hi0_r  <= 8'd0;
			rem_hi1_r  <= 8'd0;
		end else begin
			av_lo8_r   <= av_lo8_w;
			av_mid0_r  <= av_mid0_w;
			av_mid1_r  <= av_mid1_w;
			av_hi0_r   <= av_hi0_w[5:0];
			av_hi1_r   <= av_hi1_w[5:0];
			rem_lo8_r  <= rem_lo8_w;
			rem_mid0_r <= rem_mid0_w;
			rem_mid1_r <= rem_mid1_w;
			rem_hi0_r  <= rem_hi0_w[7:0];
			rem_hi1_r  <= rem_hi1_w[7:0];
		end
	end

	wire [7:0] av_mid_s = av_lo8_r[8] ? av_mid1_r[7:0] : av_mid0_r[7:0];
	wire       av_c16_s = av_lo8_r[8] ? av_mid1_r[8]   : av_mid0_r[8];
	wire [5:0] av_hi_s  = av_c16_s    ? av_hi1_r       : av_hi0_r;
	wire [21:0] avail_d = 22'd0;
	wire        avail_bo = 1'b0;
	wire [21:0] avail   = {av_hi_s, av_mid_s, av_lo8_r[7:0]} & mask_av_r;

	wire [7:0] rem_mid_s = rem_lo8_r[8] ? rem_mid1_r[7:0] : rem_mid0_r[7:0];
	wire       rem_c16_s = rem_lo8_r[8] ? rem_mid1_r[8]   : rem_mid0_r[8];
	wire [7:0] rem_hi_s  = rem_c16_s    ? rem_hi1_r       : rem_hi0_r;
	wire [22:0] rem_words_s  = {rem_hi_s[6:0], rem_mid_s, rem_lo8_r[7:0]};
	wire        rem_words_bo = rem_hi_s[7];
	wire [8:0]  rem_lo8_p1_s = fn_inc9_k7({1'b0, rem_lo8_r[7:0]});
	wire [22:0] rem_sub_a    = nwords_act_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg [21:0] mask_av_r;
	reg [22:0] nwords_act_r;
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			mask_av_r    <= 22'd0;
			nwords_act_r <= 23'd0;
		end else begin
			mask_av_r    <= mask_av_c;
			nwords_act_r <= nwords_act;
		end
	end

	wire [8:0] av_lo8_w   = fn_add8_k7_c1(wptr_av_c[7:0],   ~cur[7:0]);
	wire [8:0] av_mid0_w  = fn_add8_k7_c0(wptr_av_c[15:8],  ~cur[15:8]);
	wire [8:0] av_mid1_w  = fn_add8_k7_c1(wptr_av_c[15:8],  ~cur[15:8]);
	wire [8:0] av_hi0_w   = fn_add8_k7_c0({2'b00, wptr_av_c[21:16]}, {2'b00, ~cur[21:16]});
	wire [8:0] av_hi1_w   = fn_add8_k7_c1({2'b00, wptr_av_c[21:16]}, {2'b00, ~cur[21:16]});

	wire [8:0] rem_lo8_w  = fn_add8_k7_c0(nwords_act_r[7:0],  ~cur[7:0]);
	wire [8:0] rem_mid0_w = fn_add8_k7_c0(nwords_act_r[15:8], ~cur[15:8]);
	wire [8:0] rem_mid1_w = fn_add8_k7_c1(nwords_act_r[15:8], ~cur[15:8]);
	wire [8:0] rem_hi0_w  = fn_add8_k7_c0({1'b0, nwords_act_r[22:16]}, {2'b11, ~cur[21:16]});
	wire [8:0] rem_hi1_w  = fn_add8_k7_c1({1'b0, nwords_act_r[22:16]}, {2'b11, ~cur[21:16]});

	reg [8:0] av_lo8_r, av_mid0_r, av_mid1_r;
	reg [5:0] av_hi0_r, av_hi1_r;
	reg [8:0] rem_lo8_r, rem_mid0_r, rem_mid1_r;
	reg [7:0] rem_hi0_r, rem_hi1_r;
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
			av_lo8_r   <= 9'd0;
			av_mid0_r  <= 9'd0;
			av_mid1_r  <= 9'd0;
			av_hi0_r   <= 6'd0;
			av_hi1_r   <= 6'd0;
			rem_lo8_r  <= 9'd0;
			rem_mid0_r <= 9'd0;
			rem_mid1_r <= 9'd0;
			rem_hi0_r  <= 8'd0;
			rem_hi1_r  <= 8'd0;
		end else begin
			av_lo8_r   <= av_lo8_w;
			av_mid0_r  <= av_mid0_w;
			av_mid1_r  <= av_mid1_w;
			av_hi0_r   <= av_hi0_w[5:0];
			av_hi1_r   <= av_hi1_w[5:0];
			rem_lo8_r  <= rem_lo8_w;
			rem_mid0_r <= rem_mid0_w;
			rem_mid1_r <= rem_mid1_w;
			rem_hi0_r  <= rem_hi0_w[7:0];
			rem_hi1_r  <= rem_hi1_w[7:0];
		end
	end

	wire [7:0] av_mid_s = av_lo8_r[8] ? av_mid1_r[7:0] : av_mid0_r[7:0];
	wire       av_c16_s = av_lo8_r[8] ? av_mid1_r[8]   : av_mid0_r[8];
	wire [5:0] av_hi_s  = av_c16_s    ? av_hi1_r       : av_hi0_r;
	wire [21:0] avail_d = 22'd0;
	wire        avail_bo = 1'b0;
	wire [21:0] avail   = {av_hi_s, av_mid_s, av_lo8_r[7:0]} & mask_av_r;

	wire [7:0] rem_mid_s = rem_lo8_r[8] ? rem_mid1_r[7:0] : rem_mid0_r[7:0];
	wire       rem_c16_s = rem_lo8_r[8] ? rem_mid1_r[8]   : rem_mid0_r[8];
	wire [7:0] rem_hi_s  = rem_c16_s    ? rem_hi1_r       : rem_hi0_r;
	wire [22:0] rem_words_s  = {rem_hi_s[6:0], rem_mid_s, rem_lo8_r[7:0]};
	wire        rem_words_bo = rem_hi_s[7];
	wire [8:0]  rem_lo8_p1_s = fn_inc9_k7({1'b0, rem_lo8_r[7:0]});
	wire [22:0] rem_sub_a    = nwords_act_r;
`else
	reg [21:0] wptr_act_r;
	reg [21:0] mask_act_r;
	reg [22:0] nwords_act_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			wptr_act_r   <= 22'd0;
			mask_act_r   <= 22'd0;
			nwords_act_r <= 23'd0;
		end else begin
			wptr_act_r   <= wptr_act;
			mask_act_r   <= mask_act;
			nwords_act_r <= nwords_act;
		end
	end

	wire [15:0] av_lo15_w  = fn_add15_ci(wptr_act_r[14:0], ~cur[14:0], 1'b1);
	wire [8:0]  av_hi0_w   = fn_add8_ci({1'b0, wptr_act_r[21:15]}, {1'b0, ~cur[21:15]}, 1'b0);
	wire [8:0]  av_hi1_w   = fn_add8_ci({1'b0, wptr_act_r[21:15]}, {1'b0, ~cur[21:15]}, 1'b1);
	wire [15:0] rem_lo15_w = fn_add15_ci(nwords_act_r[14:0], ~cur[14:0], 1'b0);
	wire [8:0]  rem_hi0_w  = fn_add8_ci(nwords_act_r[22:15], {1'b1, ~cur[21:15]}, 1'b0);
	wire [8:0]  rem_hi1_w  = fn_add8_ci(nwords_act_r[22:15], {1'b1, ~cur[21:15]}, 1'b1);

	reg [15:0] av_lo15_r, rem_lo15_r;
	reg [6:0]  av_hi0_r, av_hi1_r;
	reg [8:0]  rem_hi0_r, rem_hi1_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			av_lo15_r  <= 16'd0;
			av_hi0_r   <= 7'd0;
			av_hi1_r   <= 7'd0;
			rem_lo15_r <= 16'd0;
			rem_hi0_r  <= 9'd0;
			rem_hi1_r  <= 9'd0;
		end else begin
			av_lo15_r  <= av_lo15_w;
			av_hi0_r   <= av_hi0_w[6:0];
			av_hi1_r   <= av_hi1_w[6:0];
			rem_lo15_r <= rem_lo15_w;
			rem_hi0_r  <= rem_hi0_w;
			rem_hi1_r  <= rem_hi1_w;
		end
	end

	wire [21:0] avail = {(av_lo15_r[15] ? av_hi1_r : av_hi0_r), av_lo15_r[14:0]} & mask_act_r;
	wire [22:0] rem_words_s   = {rem_lo15_r[15] ? rem_hi1_r[7:0] : rem_hi0_r[7:0], rem_lo15_r[14:0]};
	wire        rem_words_bo  = ~(rem_lo15_r[15] ? rem_hi1_r[8] : rem_hi0_r[8]);
	wire [8:0]  rem_lo8_p1_s  = fn_inc9({1'b0, rem_lo15_r[7:0]});
	wire [21:0] avail_d = 22'd0;
	wire [22:0] rem_sub_a = nwords_act_r;
	wire        avail_bo = 1'b0;
`endif
	(* max_fanout = 16 *) reg [21:0] avail_r;
	wire [21:0] avail_check = avail_r;
`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high)
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high)
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
`endif
			avail_r <= 22'd0;
		else
			avail_r <= avail;
	end

	(* max_fanout = 16 *) reg [22:0] rem_words_r;
	(* max_fanout = 16 *) reg rem_words_bo_r;
	reg [8:0] rem_lo8_p1_r;
	wire rem_lo8_p1_s_co_unused = 1'b0;
`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
`endif
			rem_words_r    <= 23'd0;
			rem_words_bo_r <= 1'b0;
			rem_lo8_p1_r   <= 9'd1;
		end else begin
			rem_words_r    <= rem_words_s;
			rem_words_bo_r <= rem_words_bo;
			rem_lo8_p1_r   <= rem_lo8_p1_s;
		end
	end

	wire [23:0] h_end_res = fn_add23_ci({1'b0, cur}, {7'd0, h_n}, 1'b1);
	reg  [5:0]  cur_p1_hi_r;
	wire [7:0]  cur_p1_hi_w = fn_inc8({2'b00, cur[21:16]});
`ifdef SYNTH_KINTEX7
	wire [8:0]  pe_lo8     = fn_add8_k7_c1(cur[7:0],  n[7:0]);
	wire [8:0]  pe_hi0     = fn_add8_k7_c0(cur[15:8], n[15:8]);
	wire [8:0]  pe_hi1     = fn_add8_k7_c1(cur[15:8], n[15:8]);
	wire        p_end_c16  = pe_lo8[8] ? pe_hi1[8] : pe_hi0[8];
	wire [21:0] p_end_raw  = {p_end_c16 ? cur_p1_hi_r : cur[21:16], pe_lo8[8] ? pe_hi1[7:0] : pe_hi0[7:0], pe_lo8[7:0]};
	wire [22:0] p_end_s    = {1'b0, p_end_raw};
	wire        p_end_co   = 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [8:0]  pe_lo8     = fn_add8_k7_c1(cur[7:0],  n[7:0]);
	wire [8:0]  pe_hi0     = fn_add8_k7_c0(cur[15:8], n[15:8]);
	wire [8:0]  pe_hi1     = fn_add8_k7_c1(cur[15:8], n[15:8]);
	wire        p_end_c16  = pe_lo8[8] ? pe_hi1[8] : pe_hi0[8];
	wire [21:0] p_end_raw  = {p_end_c16 ? cur_p1_hi_r : cur[21:16], pe_lo8[8] ? pe_hi1[7:0] : pe_hi0[7:0], pe_lo8[7:0]};
	wire [22:0] p_end_s    = {1'b0, p_end_raw};
	wire        p_end_co   = 1'b0;
`else
	wire [15:0] p_end_lo15 = fn_add15_ci(cur[14:0], n[14:0], 1'b1);
	wire        p_end_b15  = cur[15] ^ n[15] ^ p_end_lo15[15];
	wire        p_end_c16  = (cur[15] & n[15]) | ((cur[15] | n[15]) & p_end_lo15[15]);
	wire [22:0] p_end_s    = {1'b0, p_end_c16 ? cur_p1_hi_r : cur[21:16], p_end_b15, p_end_lo15[14:0]};
	wire        p_end_co   = 1'b0;
`endif
	wire [22:0] h_end_s   = h_end_res[22:0];
	wire        h_end_co  = h_end_res[23];
	wire [22:0] h_end = h_end_s;
`ifdef SYNTH_KINTEX7
	wire rem_hi_nz   = |rem_words_r[22:16];
	wire rem_hi8_nz  = |rem_words_r[22:8];
	wire av_hi_nz    = |avail_check[21:16];
	wire av_hi8_nz   = |avail_check[21:8];
	wire h_bad_ghci_w  = fn_gt8_en_k7(hdr_data[15:8], rem_lo8_p1_r[7:0], !rem_hi8_nz && !rem_lo8_p1_r[8]);
	wire h_bad_c2_w    = fn_gt16_en_k7(hdr_data[15:0], rem_words_r[15:0], !rem_hi_nz);
	wire h_bad_lo_w    = fn_gt8_en_k7(hdr_data[7:0],  rem_words_r[7:0],  !rem_hi8_nz);
	wire h_bad_ghci_av = fn_gt8_en_k7(hdr_data[15:8], avail_check[7:0], !av_hi8_nz);
	wire h_bad_c2_av   = fn_ge16_en_k7(hdr_data[15:0], avail_check[15:0], !av_hi_nz);
	wire h_bad_lo_av   = fn_ge8_en_k7(hdr_data[7:0],  avail_check[7:0],  !av_hi8_nz);
	wire h_bad_words   = rem_words_bo_r ||
		(is_ghci ? h_bad_ghci_w : (hdr_data[31] ? h_bad_c2_w : h_bad_lo_w));
	wire h_bad_avail   = is_ghci ? h_bad_ghci_av : (hdr_data[31] ? h_bad_c2_av : h_bad_lo_av);
	wire h_novf_words  = h_bad_words;
	wire h_bad         = h_bad_words || h_bad_avail;
	wire rem_lo8_p1_co_unused = 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire rem_hi_nz   = |rem_words_r[22:16];
	wire rem_hi8_nz  = |rem_words_r[22:8];
	wire av_hi_nz    = |avail_check[21:16];
	wire av_hi8_nz   = |avail_check[21:8];
	wire h_bad_ghci_w  = fn_gt8_en_k7(hdr_data[15:8], rem_lo8_p1_r[7:0], !rem_hi8_nz && !rem_lo8_p1_r[8]);
	wire h_bad_c2_w    = fn_gt16_en_k7(hdr_data[15:0], rem_words_r[15:0], !rem_hi_nz);
	wire h_bad_lo_w    = fn_gt8_en_k7(hdr_data[7:0],  rem_words_r[7:0],  !rem_hi8_nz);
	wire h_bad_ghci_av = fn_gt8_en_k7(hdr_data[15:8], avail_check[7:0], !av_hi8_nz);
	wire h_bad_c2_av   = fn_ge16_en_k7(hdr_data[15:0], avail_check[15:0], !av_hi_nz);
	wire h_bad_lo_av   = fn_ge8_en_k7(hdr_data[7:0],  avail_check[7:0],  !av_hi8_nz);
	wire h_bad_words   = rem_words_bo_r ||
		(is_ghci ? h_bad_ghci_w : (hdr_data[31] ? h_bad_c2_w : h_bad_lo_w));
	wire h_bad_avail   = is_ghci ? h_bad_ghci_av : (hdr_data[31] ? h_bad_c2_av : h_bad_lo_av);
	wire h_novf_words  = h_bad_words;
	wire h_bad         = h_bad_words || h_bad_avail;
	wire rem_lo8_p1_co_unused = 1'b0;
`else
	wire rem_hi_nz   = |rem_words_r[22:16];
	wire rem_hi8_nz  = |rem_words_r[22:8];
	wire av_hi_nz    = |avail_check[21:16];
	wire av_hi8_nz   = |avail_check[21:8];
	wire rem_hi_b_gt = fn_cmp_gt8(hdr_data[15:8], rem_words_r[15:8]);
	wire rem_hi_b_eq = (hdr_data[15:8] == rem_words_r[15:8]);
	wire rem_lo_b_gt = fn_cmp_gt8(hdr_data[7:0],  rem_words_r[7:0]);
	wire av_hi_b_gt  = fn_cmp_gt8(hdr_data[15:8], avail_check[15:8]);
	wire av_hi_b_eq  = (hdr_data[15:8] == avail_check[15:8]);
	wire av_lo_b_gt  = fn_cmp_gt8(hdr_data[7:0],  avail_check[7:0]);
	wire av_lo_b_ge  = av_lo_b_gt || (hdr_data[7:0] == avail_check[7:0]);
	wire h_bad_ghci_w  = !rem_hi8_nz && fn_cmp_gt9({1'b0, hdr_data[15:8]}, rem_lo8_p1_r);
	wire h_bad_c2_w    = !rem_hi_nz  && (rem_hi_b_gt || (rem_hi_b_eq && rem_lo_b_gt));
	wire h_bad_lo_w    = !rem_hi8_nz && rem_lo_b_gt;
	wire h_bad_ghci_av = !av_hi8_nz  && fn_cmp_gt8(hdr_data[15:8], avail_check[7:0]);
	wire h_bad_c2_av   = !av_hi_nz   && (av_hi_b_gt || (av_hi_b_eq && av_lo_b_ge));
	wire h_bad_lo_av   = !av_hi8_nz  && av_lo_b_ge;
	wire h_bad_words   = rem_words_bo_r ||
		(is_ghci ? h_bad_ghci_w : (hdr_data[31] ? h_bad_c2_w : h_bad_lo_w));
	wire h_bad_avail   = is_ghci ? h_bad_ghci_av : (hdr_data[31] ? h_bad_c2_av : h_bad_lo_av);
	wire h_novf_words  = h_bad_words;
	wire h_bad         = h_bad_words || h_bad_avail;
	wire rem_lo8_p1_co_unused = 1'b0;
`endif
	wire h_novf = (h_class == 2'd2) && (CLASS_V == 0);
`ifdef SYNTH_KINTEX7
	wire [21:0] p_end = p_end_s[21:0] & mask_act;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [21:0] p_end = p_end_s[21:0] & mask_act;
`else
	wire [21:0] p_end = p_end_s[21:0] & mask_act_r;
`endif

`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) wire [3:0] st_b0, st_b1;
	genvar g_stb;
	generate
		for (g_stb = 0; g_stb < 4; g_stb = g_stb + 1) begin : g_st_buf
			assign st_b0[g_stb] = st[g_stb];
			assign st_b1[g_stb] = st[g_stb];
		end
	endgenerate
	wire [7:0] st_idle_sel_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis0 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis1 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis2 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis3 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis4 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis5 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis6 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_idle_sel_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis7 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[7]));

	wire [4:0] st_radv_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra0 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra1 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra2 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra3 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra4 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_radv_b[4]));

	wire rptr_adv_q0 = st_radv_b[4] && (active_qid == 2'd0);
	wire rptr_adv_q1 = st_radv_b[4] && (active_qid == 2'd1);
	wire rptr_adv_q2 = st_radv_b[4] && (active_qid == 2'd2);
	wire rptr_rst_en = (st == R_CTX_REST_RSP) && rsp_is_tag3_r;
	wire rptr_save_en = (st == R_CTX_SAVE_RSP) && rsp_is_tag2_r;
	wire imm_rest_en = (st_idle_sel_b[6]) && preempt_active && ctx_base_z_r;
	wire st_is_radv_hw = st_radv_b[4];
	wire hw_rst_all;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFF8)) u_hw_rst (
		.I0(q0_empty_r), .I1(imm_rest_en), .I2(st_is_radv_hw), .I3(ring_core_rst_high),
		.O(hw_rst_all)
	);
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			has_work_r <= 3'd0;
		else if (hw_rst_all)
			has_work_r <= 3'd0;
		else
			has_work_r <= has_work;
	end
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			q0_empty_r <= 1'b0;
		else if (hw_rst_all || doorbell[0])
			q0_empty_r <= 1'b0;
		else
			q0_empty_r <= q_empty[0];
	end
	wire rptr_we_q0, rptr_we_q1, rptr_we_q2;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAAAAAAAAAFEEE)) u_rwe_q0 (
		.I0(rptr_adv_q0), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAAAAAFEEEAAAA)) u_rwe_q1 (
		.I0(rptr_adv_q1), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAFEEEAAAAAAAA)) u_rwe_q2 (
		.I0(rptr_adv_q2), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q2)
	);

	reg [21:0] p_end_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			rptr[0] <= 22'd0;
			rptr[1] <= 22'd0;
			rptr[2] <= 22'd0;
		end else begin
			if (rptr_we_q0) begin
				rptr[0][10:0]  <= st_radv_b[0] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[0][21:11] <= st_radv_b[1] ? p_end_r[21:11] : saved_rptr[21:11];
			end
			if (rptr_we_q1) begin
				rptr[1][10:0]  <= st_radv_b[1] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[1][21:11] <= st_radv_b[2] ? p_end_r[21:11] : saved_rptr[21:11];
			end
			if (rptr_we_q2) begin
				rptr[2][10:0]  <= st_radv_b[2] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[2][21:11] <= st_radv_b[3] ? p_end_r[21:11] : saved_rptr[21:11];
			end
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) wire [3:0] st_b0, st_b1;
	genvar g_stb;
	generate
		for (g_stb = 0; g_stb < 4; g_stb = g_stb + 1) begin : g_st_buf
			assign st_b0[g_stb] = st[g_stb];
			assign st_b1[g_stb] = st[g_stb];
		end
	endgenerate
	wire [7:0] st_idle_sel_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis0 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis1 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis2 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis3 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis4 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis5 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis6 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_idle_sel_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_sis7 (.I0(st_b0[0]), .I1(st_b0[1]), .I2(st_b0[2]), .I3(st_b0[3]), .O(st_idle_sel_b[7]));

	wire [4:0] st_radv_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra0 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra1 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra2 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra3 (.I0(st_b1[0]), .I1(st_b1[1]), .I2(st_b1[2]), .I3(st_b1[3]), .O(st_radv_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_sra4 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_radv_b[4]));

	wire rptr_adv_q0 = st_radv_b[4] && (active_qid == 2'd0);
	wire rptr_adv_q1 = st_radv_b[4] && (active_qid == 2'd1);
	wire rptr_adv_q2 = st_radv_b[4] && (active_qid == 2'd2);
	wire rptr_rst_en = (st == R_CTX_REST_RSP) && rsp_is_tag3_r;
	wire rptr_save_en = (st == R_CTX_SAVE_RSP) && rsp_is_tag2_r;
	wire imm_rest_en = (st_idle_sel_b[6]) && preempt_active && ctx_base_z_r;
	wire st_is_radv_hw = st_radv_b[4];
	wire hw_rst_all;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFF8)) u_hw_rst (
		.I0(q0_empty_r), .I1(imm_rest_en), .I2(st_is_radv_hw), .I3(ring_core_rst_high),
		.O(hw_rst_all)
	);
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			has_work_r <= 3'd0;
		else if (hw_rst_all)
			has_work_r <= 3'd0;
		else
			has_work_r <= has_work;
	end
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			q0_empty_r <= 1'b0;
		else if (hw_rst_all || doorbell[0])
			q0_empty_r <= 1'b0;
		else
			q0_empty_r <= q_empty[0];
	end
	wire rptr_we_q0, rptr_we_q1, rptr_we_q2;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAAAAAAAAAFEEE)) u_rwe_q0 (
		.I0(rptr_adv_q0), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAAAAAFEEEAAAA)) u_rwe_q1 (
		.I0(rptr_adv_q1), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hAAAAFEEEAAAAAAAA)) u_rwe_q2 (
		.I0(rptr_adv_q2), .I1(rptr_rst_en), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(interrupted_qid[0]), .I5(interrupted_qid[1]),
		.O(rptr_we_q2)
	);

	reg [21:0] p_end_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			rptr[0] <= 22'd0;
			rptr[1] <= 22'd0;
			rptr[2] <= 22'd0;
		end else begin
			if (rptr_we_q0) begin
				rptr[0][10:0]  <= st_radv_b[0] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[0][21:11] <= st_radv_b[1] ? p_end_r[21:11] : saved_rptr[21:11];
			end
			if (rptr_we_q1) begin
				rptr[1][10:0]  <= st_radv_b[1] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[1][21:11] <= st_radv_b[2] ? p_end_r[21:11] : saved_rptr[21:11];
			end
			if (rptr_we_q2) begin
				rptr[2][10:0]  <= st_radv_b[2] ? p_end_r[10:0]  : saved_rptr[10:0];
				rptr[2][21:11] <= st_radv_b[3] ? p_end_r[21:11] : saved_rptr[21:11];
			end
		end
	end
`else
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n)
			has_work_r <= 3'd0;
		else if ((st == R_ADV) || (st == R_IDLE && preempt_active && ctx_base_zero && q0_empty_r))
			has_work_r <= 3'd0;
		else
			has_work_r <= has_work;
	end
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n)
			q0_empty_r <= 1'b0;
		else if ((st == R_ADV) || (st == R_IDLE && preempt_active && ctx_base_zero && q0_empty_r) || doorbell[0])
			q0_empty_r <= 1'b0;
		else
			q0_empty_r <= q_empty[0];
	end

	reg [21:0] p_end_r;
	wire [2:0] st_radv_b = {3{st == R_ADV}};
	wire rptr_adv_q0 = (st == R_ADV) && (active_qid == 2'd0);
	wire rptr_adv_q1 = (st == R_ADV) && (active_qid == 2'd1);
	wire rptr_adv_q2 = (st == R_ADV) && (active_qid == 2'd2);
	wire rptr_rst_en = (st == R_CTX_REST_RSP) && rsp_is_tag3_r;
	wire rptr_save_en = (st == R_CTX_SAVE_RSP) && rsp_is_tag2_r;
	wire imm_rest_en = (st == R_IDLE) && preempt_active && ctx_base_zero;
	wire rptr_we_q0 = rptr_adv_q0 || ((rptr_rst_en || (imm_rest_en && q0_empty_r)) && (interrupted_qid == 2'd0));
	wire rptr_we_q1 = rptr_adv_q1 || ((rptr_rst_en || (imm_rest_en && q0_empty_r)) && (interrupted_qid == 2'd1));
	wire rptr_we_q2 = rptr_adv_q2 || ((rptr_rst_en || (imm_rest_en && q0_empty_r)) && (interrupted_qid == 2'd2));
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n) begin
			rptr[0] <= 22'd0;
			rptr[1] <= 22'd0;
			rptr[2] <= 22'd0;
		end else begin
			if (rptr_we_q0) rptr[0] <= st_radv_b[0] ? p_end_r : saved_rptr;
			if (rptr_we_q1) rptr[1] <= st_radv_b[1] ? p_end_r : saved_rptr;
			if (rptr_we_q2) rptr[2] <= st_radv_b[2] ? p_end_r : saved_rptr;
		end
	end
`endif

	function [3:0] fn_inc4;
		input [3:0] v;
		input ci;
		begin
			fn_inc4[0] = v[0] ^ ci;
			fn_inc4[1] = v[1] ^ (ci & v[0]);
			fn_inc4[2] = v[2] ^ (ci & v[0] & v[1]);
			fn_inc4[3] = v[3] ^ (ci & (&v[2:0]));
		end
	endfunction

	function [47:0] fn_inc48;
		input [47:0] v;
		reg [47:0] s0_n, s1, s2_n, s3, s4_n, s5;
		begin
			s0_n = ~(v    & {v[46:0],    1'b1});
			s1   = ~(s0_n | {s0_n[45:0], 2'b00});
			s2_n = ~(s1   & {s1[43:0],   4'hF});
			s3   = ~(s2_n | {s2_n[39:0], 8'h00});
			s4_n = ~(s3   & {s3[31:0],   16'hFFFF});
			s5   = ~(s4_n | {s4_n[15:0], 32'h0000_0000});
			fn_inc48 = v ^ {s5[46:0], 1'b1};
		end
	endfunction

	wire [ADDR_W-1:0] pa_4, pa_hdr;
	wire [15:0] k_n;
	wire pa_4_co = 1'b0;
	wire pa_hdr_co, k_co;

	(* max_fanout = 16 *) reg [63:0] base_act_r;
	(* max_fanout = 16 *) reg [21:0] rptr_act_r;
	(* max_fanout = 16 *) reg [23:0] pa_hdr_lo_r;
	(* dont_touch = "true" *) reg pa_hdr_c2_r0, pa_hdr_c2_r1;
	(* max_fanout = 16, equivalent_register_removal = "no" *) reg [ADDR_W-25:0] base_hi_r, base_hi_inc_r;
	(* max_fanout = 16 *) reg [ADDR_W-1:0] pa_hdr_r;
	reg [1:0] pa_hdr_age;

	reg [47:0] pa_w_pad, base_hi_pad;
	always @(*) begin
		pa_w_pad = 48'd0;
		pa_w_pad[ADDR_W-3:0] = pa[ADDR_W-1:2];
		base_hi_pad = 48'd0;
		base_hi_pad[ADDR_W-25:0] = base_act_r[ADDR_W-1:24];
	end
`ifdef SYNTH_KINTEX7
	wire [15:0] pa_w_lo15  = fn_inc16_k7({1'b0, pa_w_pad[14:0]});
	wire [15:0] pa_w_mid15 = fn_inc16_k7({1'b0, pa_w_pad[29:15]});
	wire [15:0] pa_w_hi15  = fn_inc16_k7({1'b0, pa_w_pad[44:30]});
	wire        pa_w_c30   = pa_w_lo15[15] & pa_w_mid15[15];
	wire [44:0] pa_w_inc45 = {
		pa_w_c30      ? pa_w_hi15[14:0]  : pa_w_pad[44:30],
		pa_w_lo15[15] ? pa_w_mid15[14:0] : pa_w_pad[29:15],
		pa_w_lo15[14:0]
	};
	assign pa_4 = {pa_w_inc45[ADDR_W-3:0], pa[1:0]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [15:0] pa_w_lo15  = fn_inc16_k7({1'b0, pa_w_pad[14:0]});
	wire [15:0] pa_w_mid15 = fn_inc16_k7({1'b0, pa_w_pad[29:15]});
	wire [15:0] pa_w_hi15  = fn_inc16_k7({1'b0, pa_w_pad[44:30]});
	wire        pa_w_c30   = pa_w_lo15[15] & pa_w_mid15[15];
	wire [44:0] pa_w_inc45 = {
		pa_w_c30      ? pa_w_hi15[14:0]  : pa_w_pad[44:30],
		pa_w_lo15[15] ? pa_w_mid15[14:0] : pa_w_pad[29:15],
		pa_w_lo15[14:0]
	};
	assign pa_4 = {pa_w_inc45[ADDR_W-3:0], pa[1:0]};
`else
	wire [47:0] pa_w_inc = fn_inc48(pa_w_pad);
	assign pa_4 = {pa_w_inc[ADDR_W-3:0], pa[1:0]};
`endif

	/* pa_hdr: base_act_r + {rptr_act_r, 2'b00} via 22-bit prefix add + parallel upper increment (registered). */
	wire [22:0] ph_add_res = fn_add22_co(base_act_r[23:2], rptr_act_r);
	wire [7:0] ph_s0   = ph_add_res[7:0];
	wire [7:0] ph_s1_0 = ph_add_res[15:8];
	wire [7:0] ph_s1_1 = ph_add_res[15:8];
	wire [5:0] ph_s2_0 = ph_add_res[21:16];
	wire [5:0] ph_s2_1 = ph_add_res[21:16];
	wire ph_c0   = 1'b0;
	wire ph_c1   = 1'b0;
	wire ph_c2   = ph_add_res[22];
	wire ph_c2_0 = ph_add_res[22];
	wire ph_c2_1 = ph_add_res[22];
	wire [47:0] base_hi_inc = fn_inc48(base_hi_pad);
	wire [ADDR_W-25:0] ph_hi_mux;
	genvar g_ph;
	generate
		for (g_ph = 0; g_ph < (ADDR_W - 24); g_ph = g_ph + 1) begin : g_ph_mux
			assign ph_hi_mux[g_ph] = (g_ph < ((ADDR_W - 24) / 2)) ?
				(pa_hdr_c2_r0 ? base_hi_inc_r[g_ph] : base_hi_r[g_ph]) :
				(pa_hdr_c2_r1 ? base_hi_inc_r[g_ph] : base_hi_r[g_ph]);
		end
	endgenerate
`ifdef SYNTH_KINTEX7
	wire pa_hdr_en0, pa_hdr_en1, pa_hdr_en2;
	wire [3:0] pa_hdr_age0_en;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe0 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en0));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe1 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en1));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe2 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en2));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e0 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e1 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e2 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e3 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[3]));
	wire st_is_idle_pa;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_stipa (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_is_idle_pa));
	wire [3:0] ph_age1_ce, we_sz0_ce;
	wire [31:0] size0_enc_next = encode_ring_size(dec_sz_r);
	genvar g_p1;
	generate
		for (g_p1 = 0; g_p1 < 4; g_p1 = g_p1 + 1) begin : g_ph1_sz0
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0020)) u_ph1e (.I0(st_is_idle_pa), .I1(hcr_we_cfg_b[0]), .I2(pa_hdr_age[0]), .I3(pa_hdr_age[1]), .O(ph_age1_ce[g_p1]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_sz0ce (.I0(we_sz0_b[0]), .I1(dec_sz_nz_r), .O(we_sz0_ce[g_p1]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire pa_hdr_en0, pa_hdr_en1, pa_hdr_en2;
	wire [3:0] pa_hdr_age0_en;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe0 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en0));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe1 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en1));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h4)) u_phe2 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(pa_hdr_en2));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e0 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e1 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e2 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[2]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_ph0e3 (.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .I2(st != R_IDLE), .I3(hcr_we_cfg_b[0]), .O(pa_hdr_age0_en[3]));
	wire st_is_idle_pa;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_stipa (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_is_idle_pa));
	wire [3:0] ph_age1_ce, we_sz0_ce;
	wire [31:0] size0_enc_next = encode_ring_size(dec_sz_r);
	genvar g_p1;
	generate
		for (g_p1 = 0; g_p1 < 4; g_p1 = g_p1 + 1) begin : g_ph1_sz0
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0020)) u_ph1e (.I0(st_is_idle_pa), .I1(hcr_we_cfg_b[0]), .I2(pa_hdr_age[0]), .I3(pa_hdr_age[1]), .O(ph_age1_ce[g_p1]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_sz0ce (.I0(we_sz0_b[0]), .I1(dec_sz_nz_r), .O(we_sz0_ce[g_p1]));
		end
	endgenerate
`endif
	assign pa_hdr = pa_hdr_r;
	assign pa_hdr_co = ph_c2;
`ifdef SYNTH_KINTEX7
	assign k_n = fn_inc16_k7(k);
	wire rn_go_k7 = hit_n_r && n_ready && !k_eq_n;
	wire [1:0] k_ce_b;
	wire [1:0] k_clr_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_kclr0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(k_clr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_kclr1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(k_clr_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00060002)) u_kce0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .I4(rn_go_k7), .O(k_ce_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00060002)) u_kce1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .I4(rn_go_k7), .O(k_ce_b[1]));
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			k <= 16'd0;
		end else begin
			if (k_ce_b[0]) k[7:0]  <= k_clr_b[0] ? 8'd0 : k_n[7:0];
			if (k_ce_b[1]) k[15:8] <= k_clr_b[1] ? 8'd0 : k_n[15:8];
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	assign k_n = fn_inc16_k7(k);
	wire rn_go_k7 = hit_n_r && n_ready && !k_eq_n;
	wire [1:0] k_ce_b;
	wire [1:0] k_clr_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_kclr0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(k_clr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_kclr1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(k_clr_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00060002)) u_kce0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .I4(rn_go_k7), .O(k_ce_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00060002)) u_kce1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .I4(rn_go_k7), .O(k_ce_b[1]));
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			k <= 16'd0;
		end else begin
			if (k_ce_b[0]) k[7:0]  <= k_clr_b[0] ? 8'd0 : k_n[7:0];
			if (k_ce_b[1]) k[15:8] <= k_clr_b[1] ? 8'd0 : k_n[15:8];
		end
	end
`else
	assign k_n = fn_inc16(k);
`endif
	assign k_co = 1'b0;

	/*
	 * Background Prefetch Engine
	 */
	wire [LA_W-1:0] next_line;
	wire unused_nl_co = 1'b0;
	reg [47:0] want_line_pad;
	always @(*) begin
		want_line_pad = 48'd0;
		want_line_pad[LA_W-1:0] = want_line;
	end
	wire [47:0] next_line_inc = fn_inc48(want_line_pad);
	assign next_line = next_line_inc[LA_W-1:0];
`ifdef SYNTH_KINTEX7
	(* max_fanout = 16 *) reg [LA_W-1:0] next_line_r;
	reg [LA_W-1:0] nxt_tag_r;
	reg nxt_valid_r;
	reg next_hit_r;
	reg [2:0] nl_settle_r;

	wire [8:0] nm_grp;
	wire nxt_hit_lo, nxt_hit_hi, nxt_hit_comb;
	genvar g_nm;
	generate
		for (g_nm = 0; g_nm < 9; g_nm = g_nm + 1) begin : gen_nm_grp
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_nmg (
				.I0(nxt_tag_r[g_nm*3 + 0]),   .I1(nxt_tag_r[g_nm*3 + 1]),   .I2(nxt_tag_r[g_nm*3 + 2]),
				.I3(next_line_r[g_nm*3 + 0]), .I4(next_line_r[g_nm*3 + 1]), .I5(next_line_r[g_nm*3 + 2]),
				.O(nm_grp[g_nm])
			);
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_nh_lo (
		.I0(nm_grp[0]), .I1(nm_grp[1]), .I2(nm_grp[2]), .I3(nm_grp[3]), .I4(nm_grp[4]),
		.O(nxt_hit_lo)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_nh_hi (
		.I0(nm_grp[5]), .I1(nm_grp[6]), .I2(nm_grp[7]), .I3(nm_grp[8]), .I4(nxt_valid_r),
		.O(nxt_hit_hi)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_nh_all (
		.I0(nxt_hit_lo), .I1(nxt_hit_hi), .O(nxt_hit_comb)
	);
	wire next_hit = next_hit_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* max_fanout = 16 *) reg [LA_W-1:0] next_line_r;
	reg [LA_W-1:0] nxt_tag_r;
	reg nxt_valid_r;
	reg next_hit_r;
	reg [2:0] nl_settle_r;

	wire [8:0] nm_grp;
	wire nxt_hit_lo, nxt_hit_hi, nxt_hit_comb;
	genvar g_nm;
	generate
		for (g_nm = 0; g_nm < 9; g_nm = g_nm + 1) begin : gen_nm_grp
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h8040201008040201)) u_nmg (
				.I0(nxt_tag_r[g_nm*3 + 0]),   .I1(nxt_tag_r[g_nm*3 + 1]),   .I2(nxt_tag_r[g_nm*3 + 2]),
				.I3(next_line_r[g_nm*3 + 0]), .I4(next_line_r[g_nm*3 + 1]), .I5(next_line_r[g_nm*3 + 2]),
				.O(nm_grp[g_nm])
			);
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_nh_lo (
		.I0(nm_grp[0]), .I1(nm_grp[1]), .I2(nm_grp[2]), .I3(nm_grp[3]), .I4(nm_grp[4]),
		.O(nxt_hit_lo)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h80000000)) u_nh_hi (
		.I0(nm_grp[5]), .I1(nm_grp[6]), .I2(nm_grp[7]), .I3(nm_grp[8]), .I4(nxt_valid_r),
		.O(nxt_hit_hi)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_nh_all (
		.I0(nxt_hit_lo), .I1(nxt_hit_hi), .O(nxt_hit_comb)
	);
	wire next_hit = next_hit_r;
`else
	reg [LA_W-1:0] next_line_r;
	reg [LA_W-1:0] nxt_tag_r;
	reg nxt_valid_r;
	reg next_hit_r;
	reg [2:0] nl_settle_r;
	wire [2:0] next_idx = next_line_r[2:0];
	wire nxt_hit_comb = nxt_valid_r && (nxt_tag_r == next_line_r);
	wire next_hit = next_hit_r;
`endif

	(* max_fanout = 16 *) reg [1:0] pf_st;
	(* max_fanout = 16, extract_enable = "no" *) reg [LA_W-1:0] pf_line;
	wire [2:0] pf_idx = pf_line[2:0];
`ifdef SYNTH_KINTEX7
	wire [2:0] next_idx = next_line_r[2:0];
	wire [2:0] nidx_tag0, nidx_tag1, nidx_tag2, nidx_tag3;
	genvar g_nt;
	generate
		for (g_nt = 0; g_nt < 3; g_nt = g_nt + 1) begin : g_nidx_tag_buf
			assign nidx_tag0[g_nt] = next_idx[g_nt];
			assign nidx_tag1[g_nt] = next_idx[g_nt];
			assign nidx_tag2[g_nt] = next_idx[g_nt];
			assign nidx_tag3[g_nt] = next_idx[g_nt];
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] next_idx = next_line_r[2:0];
	wire [2:0] nidx_tag0, nidx_tag1, nidx_tag2, nidx_tag3;
	genvar g_nt;
	generate
		for (g_nt = 0; g_nt < 3; g_nt = g_nt + 1) begin : g_nidx_tag_buf
			assign nidx_tag0[g_nt] = next_idx[g_nt];
			assign nidx_tag1[g_nt] = next_idx[g_nt];
			assign nidx_tag2[g_nt] = next_idx[g_nt];
			assign nidx_tag3[g_nt] = next_idx[g_nt];
		end
	endgenerate
`endif

	wire ctx_save_req = (st == R_CTX_SAVE_REQ);
	wire ctx_rest_req = (st == R_CTX_REST_REQ);
	wire ctx_req = ctx_save_req || ctx_rest_req;

	// Prefetch next line while executing Class N packet before boundary hit
	(* max_fanout = 16 *) reg k_eq_n_r;
	reg [15:0] n_m1_r;
	wire [15:0] n_m1_s = fn_dec16(n);
	wire k_eq_n = k_eq_n_r;

`ifdef SYNTH_KINTEX7
	wire pf_trigger = (st == R_N) && !k_eq_n && !next_hit_r && !nl_settle_r[0] && (pf_st == PF_IDLE) && !ctx_req && go;
	(* keep = "true", equivalent_register_removal = "no" *) reg pf_trigger_r0, pf_trigger_r1;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			pf_trigger_r0 <= 1'b0;
			pf_trigger_r1 <= 1'b0;
		end else begin
			pf_trigger_r0 <= pf_trigger;
			pf_trigger_r1 <= pf_trigger;
		end
	end
	wire pf_start = pf_trigger_r0;
	wire [1:0] pf_line_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1)) u_plwe0 (.I0(pf_st[0]), .I1(pf_st[1]), .O(pf_line_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1)) u_plwe1 (.I0(pf_st[0]), .I1(pf_st[1]), .O(pf_line_we_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire pf_trigger = (st == R_N) && !k_eq_n && !next_hit_r && !nl_settle_r[0] && (pf_st == PF_IDLE) && !ctx_req && go;
	(* keep = "true", equivalent_register_removal = "no" *) reg pf_trigger_r0, pf_trigger_r1;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			pf_trigger_r0 <= 1'b0;
			pf_trigger_r1 <= 1'b0;
		end else begin
			pf_trigger_r0 <= pf_trigger;
			pf_trigger_r1 <= pf_trigger;
		end
	end
	wire pf_start = pf_trigger_r0;
	wire [1:0] pf_line_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1)) u_plwe0 (.I0(pf_st[0]), .I1(pf_st[1]), .O(pf_line_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1)) u_plwe1 (.I0(pf_st[0]), .I1(pf_st[1]), .O(pf_line_we_b[1]));
`else
	wire pf_trigger = (st == R_N) && !k_eq_n && !next_hit_r && !nl_settle_r[0] && (pf_st == PF_IDLE) && !ctx_req && go;
	reg pf_trigger_r0;
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n)
			pf_trigger_r0 <= 1'b0;
		else
			pf_trigger_r0 <= pf_trigger;
	end
	wire pf_start = pf_trigger_r0;
	wire pa_we_hdr = (st == R_IDLE);
	wire pa_we_adv = (st == R_N) && hit_n_r && n_ready && !k_eq_n;
	wire save_rptr_raw = preempt_pending && (active_qid != 2'd0) && ((st == R_IDLE) || (st == R_ADV));
	wire bvc_db = doorbell[active_qid];
	wire hw_sel_act = (active_qid == 2'd0) ? hw_sel[0] : (active_qid == 2'd1) ? hw_sel[1] : hw_sel[2];
	wire bvc_arb_idle = (st == R_IDLE) && !preempt_pending && !preempt_active && go_sel;
	wire bvc_pre_rsp = (save_rptr_raw && ctx_base_zero) || imm_rest_en || (q0_empty_r && imm_rest_en) || rptr_save_en || rptr_rst_en;
	wire bvc_arb_db = bvc_db || (bvc_arb_idle && !hw_sel_act);
	wire bv_clr_fsm = bvc_arb_db || bvc_pre_rsp;
	wire [7:0] bv_d_next = bv_clr_fsm ? 8'd0 : (buf_valid | bv_pf_hit | bv_dm_hit);
	wire hn_hdr_en = (st == R_HDR) && (hdr_wait == 2'b01);
	wire hn_set_other = (pa_we_adv && (&want_word) && next_hit_r) || dm_wr_lo;
	wire hn_clr_other = (pa_we_adv && (&want_word) && !next_hit_r) || bvc_db;
	wire hit_n_next = hn_hdr_en ? cur_hit_comb : (hn_set_other ? 1'b1 : (hn_clr_other ? 1'b0 : hit_n_r));
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n)
			hit_n_r <= 1'b0;
		else
			hit_n_r <= hit_n_next;
	end
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n) begin
			buf_valid  <= 8'd0;
			next_hit_r <= 1'b0;
		end else begin
			buf_valid  <= bv_d_next;
			next_hit_r <= bv_clr_fsm ? 1'b0 : nxt_hit_comb;
		end
	end
	reg empty_r;
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n)
			empty_r <= 1'b1;
		else if (doorbell[0] || doorbell[1] || doorbell[2])
			empty_r <= 1'b0;
		else
			empty_r <= q_empty[0] && q_empty[1] && q_empty[2];
	end
`endif

	reg f_pulse;
	reg [3:0] f_kind;
	reg [31:0] f_addr;
	reg ret_pulse;

`ifdef SYNTH_KINTEX7
	wire pa_we_hdr;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_pa_hdr_lut (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(pa_we_hdr));
	wire st_is_rn_raw = (st == R_N);
	wire pa_we_adv, pa_we_adv_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0080)) u_pa_adv_a (
		.I0(st_is_rn_raw), .I1(hit_n_r), .I2(n_ready), .I3(k_eq_n), .O(pa_we_adv)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0080)) u_pa_adv_b (
		.I0(st_is_rn_raw), .I1(hit_n_r), .I2(n_ready), .I3(k_eq_n), .O(pa_we_adv_b)
	);
	wire [5:0] pa_we_b;
	wire pa_we_b0_lo, pa_we_b0_hi;
	wire [5:0] pa_we_w;
	wire [1:0] pa_we_i;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0l (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b0_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0h (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b0_hi));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe1 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe2 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe3 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe4 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe5 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[5]));
	genvar g_pw;
	generate
		for (g_pw = 0; g_pw < 6; g_pw = g_pw + 1) begin : g_pawe_w
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pw (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_w[g_pw]));
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pwi0 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_i[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pwi1 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_i[1]));

	wire [5:0] pa_nxt_7_2_raw = st_idle_sel_b[2] ? pa_hdr[7:2] : pa_4[7:2];
	wire [5:0] pa_nxt_7_2, pa_nxt_lo, pa_nxt_hi;
	genvar g_pn;
	generate
		for (g_pn = 0; g_pn < 6; g_pn = g_pn + 1) begin : gen_pa_nxt
			assign pa_nxt_7_2[g_pn] = pa_nxt_7_2_raw[g_pn];
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pnl (.I0(pa_nxt_7_2_raw[g_pn]), .O(pa_nxt_lo[g_pn]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_pnh (.I0(pa_nxt_7_2_raw[g_pn]), .O(pa_nxt_hi[g_pn]));
		end
	endgenerate

	wire f_addr_we_bad;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_fawbad (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(f_addr_we_bad));
	wire f_addr_we_rsp = (st == R_RSP) && rsp_err_tag0_r;
	wire [1:0] fa_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fawe0 (.I0(f_addr_we_bad), .I1(f_addr_we_rsp), .O(fa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fawe1 (.I0(f_addr_we_bad), .I1(f_addr_we_rsp), .O(fa_we_b[1]));

	wire save_rptr_raw, save_rptr_bvc;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000000000000E0)) u_save_rptr_lut (
		.I0(active_qid[0]), .I1(active_qid[1]), .I2(preempt_pending),
		.I3(st[0]), .I4(st[1]), .I5(st[3]),
		.O(save_rptr_raw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000000000000E0)) u_save_rptr_bvc (
		.I0(active_qid[0]), .I1(active_qid[1]), .I2(preempt_pending),
		.I3(st[0]), .I4(st[1]), .I5(st[3]),
		.O(save_rptr_bvc)
	);
	wire bvc_db, bvc_pre_rsp, bvc_arb_0, bvc_arb_1, bvc_arb_hw, bvc_arb_db, bv_clr_fsm;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00F0CCAA)) u_bvc_db (
		.I0(doorbell[0]), .I1(doorbell[1]), .I2(doorbell[2]),
		.I3(active_qid[0]), .I4(active_qid[1]),
		.O(bvc_db)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFF888)) u_bvc_pre_rsp (
		.I0(save_rptr_bvc), .I1(ctx_base_z_r), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(rptr_save_en), .I5(rptr_rst_en),
		.O(bvc_pre_rsp)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_bvc_arb0 (
		.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(bvc_arb_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h11110010)) u_bvc_arb1 (
		.I0(preempt_pending), .I1(preempt_active), .I2(run), .I3(halt), .I4(step_pend),
		.O(bvc_arb_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFE0E3254)) u_bvc_arb_hw (
		.I0(hw_sel[0]), .I1(hw_sel[1]), .I2(hw_sel[2]),
		.I3(active_qid[0]), .I4(active_qid[1]),
		.O(bvc_arb_hw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hEAAA)) u_bvc_arb_db (
		.I0(bvc_db), .I1(bvc_arb_0), .I2(bvc_arb_1), .I3(bvc_arb_hw), .O(bvc_arb_db)
	);
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hFE)) u_bv_clr_fsm (
		.I0(bvc_arb_db), .I1(bvc_pre_rsp), .I2(ring_core_rst_high), .O(bv_clr_fsm)
	);
	wire [7:0] bv_d_next;
	genvar g_bvd;
	generate
		for (g_bvd = 0; g_bvd < 8; g_bvd = g_bvd + 1) begin : g_bv_next
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0000FFFE)) u_bv_d (
				.I0(buf_valid[g_bvd]), .I1(bv_pf_hit[g_bvd]), .I2(bv_dm_hit[g_bvd]),
				.I3(bvc_arb_db), .I4(bvc_pre_rsp), .O(bv_d_next[g_bvd])
			);
		end
	endgenerate

	wire hn_hdr_en = (st == R_HDR) && (hdr_wait == 2'b01);
	wire hn_set_other = (pa_we_adv && (want_word == 3'b111) && next_hit_r) || dm_wr_lo;
	wire hn_clr_other = (pa_we_adv && (want_word == 3'b111) && !next_hit_r) || bvc_db;
	wire hit_n_next;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00008F8F00008F80)) u_hn_lut (
		.I0(cur_hit_lo), .I1(cur_hit_hi), .I2(hn_hdr_en),
		.I3(hn_set_other), .I4(hn_clr_other), .I5(hit_n_r),
		.O(hit_n_next)
	);
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			hit_n_r <= 1'b0;
		else
			hit_n_r <= hit_n_next;
	end

	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			buf_valid  <= 8'd0;
			next_hit_r <= 1'b0;
		end else begin
			buf_valid  <= bv_d_next;
			next_hit_r <= bv_clr_fsm ? 1'b0 : nxt_hit_comb;
		end
	end

	reg empty_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			empty_r <= 1'b1;
		else if (doorbell[0] || doorbell[1] || doorbell[2])
			empty_r <= 1'b0;
		else
			empty_r <= q_empty[0] && q_empty[1] && q_empty[2];
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	wire pa_we_hdr;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_pa_hdr_lut (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(pa_we_hdr));
	wire st_is_rn_raw = (st == R_N);
	wire pa_we_adv, pa_we_adv_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0080)) u_pa_adv_a (
		.I0(st_is_rn_raw), .I1(hit_n_r), .I2(n_ready), .I3(k_eq_n), .O(pa_we_adv)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0080)) u_pa_adv_b (
		.I0(st_is_rn_raw), .I1(hit_n_r), .I2(n_ready), .I3(k_eq_n), .O(pa_we_adv_b)
	);
	wire [5:0] pa_we_b;
	wire pa_we_b0_lo, pa_we_b0_hi;
	wire [5:0] pa_we_w;
	wire [1:0] pa_we_i;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0l (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b0_lo));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe0h (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b0_hi));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe1 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe2 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe3 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe4 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pawe5 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_b[5]));
	genvar g_pw;
	generate
		for (g_pw = 0; g_pw < 6; g_pw = g_pw + 1) begin : g_pawe_w
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pw (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_w[g_pw]));
		end
	endgenerate
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pwi0 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_i[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pwi1 (.I0(pa_we_hdr), .I1(pa_we_adv), .O(pa_we_i[1]));

	wire [5:0] pa_nxt_7_2_raw = st_idle_sel_b[2] ? pa_hdr[7:2] : pa_4[7:2];
	wire [5:0] pa_nxt_7_2, pa_nxt_lo, pa_nxt_hi;
	genvar g_pn;
	generate
		for (g_pn = 0; g_pn < 6; g_pn = g_pn + 1) begin : gen_pa_nxt
			assign pa_nxt_7_2[g_pn] = pa_nxt_7_2_raw[g_pn];
			assign pa_nxt_lo[g_pn] = pa_nxt_7_2_raw[g_pn];
			assign pa_nxt_hi[g_pn] = pa_nxt_7_2_raw[g_pn];
		end
	endgenerate

	wire f_addr_we_bad;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_fawbad (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(f_addr_we_bad));
	wire f_addr_we_rsp = (st == R_RSP) && rsp_err_tag0_r;
	wire [1:0] fa_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fawe0 (.I0(f_addr_we_bad), .I1(f_addr_we_rsp), .O(fa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_fawe1 (.I0(f_addr_we_bad), .I1(f_addr_we_rsp), .O(fa_we_b[1]));

	wire save_rptr_raw, save_rptr_bvc;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000000000000E0)) u_save_rptr_lut (
		.I0(active_qid[0]), .I1(active_qid[1]), .I2(preempt_pending),
		.I3(st[0]), .I4(st[1]), .I5(st[3]),
		.O(save_rptr_raw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000000000000E0)) u_save_rptr_bvc (
		.I0(active_qid[0]), .I1(active_qid[1]), .I2(preempt_pending),
		.I3(st[0]), .I4(st[1]), .I5(st[3]),
		.O(save_rptr_bvc)
	);
	wire bvc_db, bvc_pre_rsp, bvc_arb_0, bvc_arb_1, bvc_arb_hw, bvc_arb_db, bv_clr_fsm;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00F0CCAA)) u_bvc_db (
		.I0(doorbell[0]), .I1(doorbell[1]), .I2(doorbell[2]),
		.I3(active_qid[0]), .I4(active_qid[1]),
		.O(bvc_db)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFF888)) u_bvc_pre_rsp (
		.I0(save_rptr_bvc), .I1(ctx_base_z_r), .I2(imm_rest_en),
		.I3(q0_empty_r), .I4(rptr_save_en), .I5(rptr_rst_en),
		.O(bvc_pre_rsp)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_bvc_arb0 (
		.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(bvc_arb_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h11110010)) u_bvc_arb1 (
		.I0(preempt_pending), .I1(preempt_active), .I2(run), .I3(halt), .I4(step_pend),
		.O(bvc_arb_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFE0E3254)) u_bvc_arb_hw (
		.I0(hw_sel[0]), .I1(hw_sel[1]), .I2(hw_sel[2]),
		.I3(active_qid[0]), .I4(active_qid[1]),
		.O(bvc_arb_hw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hEAAA)) u_bvc_arb_db (
		.I0(bvc_db), .I1(bvc_arb_0), .I2(bvc_arb_1), .I3(bvc_arb_hw), .O(bvc_arb_db)
	);
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hFE)) u_bv_clr_fsm (
		.I0(bvc_arb_db), .I1(bvc_pre_rsp), .I2(ring_core_rst_high), .O(bv_clr_fsm)
	);
	wire [7:0] bv_d_next;
	genvar g_bvd;
	generate
		for (g_bvd = 0; g_bvd < 8; g_bvd = g_bvd + 1) begin : g_bv_next
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0000FFFE)) u_bv_d (
				.I0(buf_valid[g_bvd]), .I1(bv_pf_hit[g_bvd]), .I2(bv_dm_hit[g_bvd]),
				.I3(bvc_arb_db), .I4(bvc_pre_rsp), .O(bv_d_next[g_bvd])
			);
		end
	endgenerate

	wire hn_hdr_en = (st == R_HDR) && (hdr_wait == 2'b01);
	wire hn_set_other = (pa_we_adv && (want_word == 3'b111) && next_hit_r) || dm_wr_lo;
	wire hn_clr_other = (pa_we_adv && (want_word == 3'b111) && !next_hit_r) || bvc_db;
	wire hit_n_next;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00008F8F00008F80)) u_hn_lut (
		.I0(cur_hit_lo), .I1(cur_hit_hi), .I2(hn_hdr_en),
		.I3(hn_set_other), .I4(hn_clr_other), .I5(hit_n_r),
		.O(hit_n_next)
	);
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			hit_n_r <= 1'b0;
		else
			hit_n_r <= hit_n_next;
	end

	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			buf_valid  <= 8'd0;
			next_hit_r <= 1'b0;
		end else begin
			buf_valid  <= bv_d_next;
			next_hit_r <= bv_clr_fsm ? 1'b0 : nxt_hit_comb;
		end
	end

	reg empty_r;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high)
			empty_r <= 1'b1;
		else if (doorbell[0] || doorbell[1] || doorbell[2])
			empty_r <= 1'b0;
		else
			empty_r <= q_empty[0] && q_empty[1] && q_empty[2];
	end
`endif

	assign fault = f_pulse;
	assign fault_kind = f_kind;
	assign fault_addr = f_addr;
	assign fault_tag = tag;
	assign retire = ret_pulse;
	assign empty = empty_r;
	assign busy = (st != R_IDLE && st != R_FAULT) || (pf_st != PF_IDLE) ||
		preempt_pending || preempt_active;
	assign queue_id = active_qid;
	assign preempt_ev = preempt_pulse;

	assign n_valid = (st == R_N) && hit_n;
	assign n_data = want_data;
	assign n_last = k_eq_n_r;

	wire [63:0] pa_4_ext = {{(64-ADDR_W){1'b0}}, pa_4};
	wire [63:0] ctx_base_ext = {32'd0, ctx_base};
	wire [63:0] want_addr_ext = {{(64-ADDR_W){1'b0}}, want_addr};
	assign v_valid = (st == R_V);
	assign v_addr = pa_4_ext[31:0];
	assign v_dwords = n;
	assign v_tag = tag;

	// AMB port multiplexing
	wire demand_req = (st == R_REQ);
	wire [LA_W-1:0] ctx_line = ctx_base_ext[ADDR_W-1:LINE_SHIFT];
	wire [LINE_BYTES*8-1:0] ctx_save_wdata =
		{{(LINE_BYTES*8 > 80 ? LINE_BYTES*8 - 80 : 1){1'b0}}, 10'd0, saved_rptr, 14'd0, interrupted_qid, 32'h43545853};

`ifdef SYNTH_KINTEX7
	wire [1:0] ctx_req_b, demand_req_b, ctx_save_req_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_creqb0 (.I0(ctx_req),      .O(ctx_req_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_creqb1 (.I0(ctx_req),      .O(ctx_req_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_dreqb0 (.I0(demand_req),   .O(demand_req_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_dreqb1 (.I0(demand_req),   .O(demand_req_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_csreq0 (.I0(ctx_save_req), .O(ctx_save_req_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_csreq1 (.I0(ctx_save_req), .O(ctx_save_req_b[1]));
	(* dont_touch = "true" *) reg r_req_valid;
	(* dont_touch = "true", extract_reset = "no" *) reg r_req_write;
	(* dont_touch = "true", extract_reset = "no" *) reg [LA_W-1:0] r_req_line;
	(* dont_touch = "true", extract_reset = "no" *) reg [LINE_BYTES-1:0] r_req_be;
	(* dont_touch = "true", extract_reset = "no" *) reg [7:0] r_req_tag;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			r_req_valid <= 1'b0;
			r_req_write <= 1'b0;
			r_req_line  <= {LA_W{1'b0}};
			r_req_be    <= {LINE_BYTES{1'b0}};
			r_req_tag   <= 8'd0;
		end else begin
			r_req_valid <= ctx_req || demand_req || (pf_st == PF_REQ);
			r_req_write <= ctx_save_req;
			r_req_line  <= {
				ctx_req_b[1] ? ctx_line[LA_W-1:14] : (demand_req_b[1] ? want_line[LA_W-1:14] : pf_line[LA_W-1:14]),
				ctx_req_b[0] ? ctx_line[13:0]      : (demand_req_b[0] ? want_line[13:0]      : pf_line[13:0])
			};
			r_req_be    <= {{(LINE_BYTES-16){ctx_save_req_b[1]}}, {16{ctx_save_req_b[0]}}};
			r_req_tag   <= ctx_save_req ? 8'd2 : (ctx_rest_req ? 8'd3 : (demand_req ? 8'd0 : 8'd1));
		end
	end
	assign req_valid = r_req_valid;
	assign req_write = r_req_write;
	assign req_line  = r_req_line;
	assign req_be    = r_req_be;
	assign req_wdata = ctx_save_wdata;
	assign req_tag   = r_req_tag;
	wire ring_req_ready = req_ready;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] ctx_req_b, demand_req_b, ctx_save_req_b;
	assign ctx_req_b[0] = ctx_req;
	assign ctx_req_b[1] = ctx_req;
	assign demand_req_b[0] = demand_req;
	assign demand_req_b[1] = demand_req;
	assign ctx_save_req_b[0] = ctx_save_req;
	assign ctx_save_req_b[1] = ctx_save_req;
	(* dont_touch = "true" *) reg r_req_valid;
	(* dont_touch = "true", extract_reset = "no" *) reg r_req_write;
	(* dont_touch = "true", extract_reset = "no" *) reg [LA_W-1:0] r_req_line;
	(* dont_touch = "true", extract_reset = "no" *) reg [LINE_BYTES-1:0] r_req_be;
	(* dont_touch = "true", extract_reset = "no" *) reg [7:0] r_req_tag;
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
			r_req_valid <= 1'b0;
			r_req_write <= 1'b0;
			r_req_line  <= {LA_W{1'b0}};
			r_req_be    <= {LINE_BYTES{1'b0}};
			r_req_tag   <= 8'd0;
		end else begin
			r_req_valid <= ctx_req || demand_req || (pf_st == PF_REQ);
			r_req_write <= ctx_save_req;
			r_req_line  <= {
				ctx_req_b[1] ? ctx_line[LA_W-1:14] : (demand_req_b[1] ? want_line[LA_W-1:14] : pf_line[LA_W-1:14]),
				ctx_req_b[0] ? ctx_line[13:0]      : (demand_req_b[0] ? want_line[13:0]      : pf_line[13:0])
			};
			r_req_be    <= {{(LINE_BYTES-16){ctx_save_req_b[1]}}, {16{ctx_save_req_b[0]}}};
			r_req_tag   <= ctx_save_req ? 8'd2 : (ctx_rest_req ? 8'd3 : (demand_req ? 8'd0 : 8'd1));
		end
	end
	assign req_valid = r_req_valid;
	assign req_write = r_req_write;
	assign req_line  = r_req_line;
	assign req_be    = r_req_be;
	assign req_wdata = ctx_save_wdata;
	assign req_tag   = r_req_tag;
	wire ring_req_ready = req_ready;
`else
	wire ring_req_valid_c = ctx_req || demand_req || (pf_st == PF_REQ);
	wire ring_req_ready;
	wire [LA_W-1:0] ring_req_line_c = ctx_req ? ctx_line : (demand_req ? want_line : pf_line);
	wire [LINE_BYTES-1:0] ring_req_be_c = ctx_save_req ? {LINE_BYTES{1'b1}} : {LINE_BYTES{1'b0}};
	wire [LINE_BYTES*8-1:0] ring_req_wdata_c = ctx_save_req ? ctx_save_wdata : {(LINE_BYTES * 8){1'b0}};
	wire [7:0] ring_req_tag_c = ctx_save_req ? 8'd2 : (ctx_rest_req ? 8'd3 : (demand_req ? 8'd0 : 8'd1));
	am_skid #(.WIDTH(1 + LA_W + LINE_BYTES + LINE_BYTES*8 + 8)) u_req_skid (
		.clk(clk), .rst_n(core_rst_n),
		.in_valid(ring_req_valid_c), .in_ready(ring_req_ready),
		.in_data({ctx_save_req, ring_req_line_c, ring_req_be_c, ring_req_wdata_c, ring_req_tag_c}),
		.out_valid(req_valid), .out_ready(req_ready),
		.out_data({req_write, req_line, req_be, req_wdata, req_tag})
	);
`endif

	/* Registers surviving CTRL.RESET */
`ifdef SYNTH_KINTEX7
	wire we_b0lo_raw = hcr_we_cfg_b[0] && (widx_a[5:0] == I_BASE0_LO[5:0] || widx_a[5:0] == I_BASE0[5:0]);
	wire we_b0hi_raw = hcr_we_cfg_b[0] && (widx_a[5:0] == I_BASE0_HI[5:0]);
	wire we_b1_raw   = 1'b0;
	wire we_b2_raw   = 1'b0;
	wire we_dmab_raw = 1'b0;
	wire we_dmad_raw = 1'b0;
	wire we_rs_raw   = 1'b0;
	wire we_ss_raw   = 1'b0;
	wire we_ctxb_raw = 1'b0;
	wire we_dext_raw = 1'b0;

	wire [1:0] we_b0lo_b, we_b0hi_b, we_b1_b, we_b2_b, we_dmab_b, we_dmad_b, we_ctxb_b, we_dext_b;
	wire [3:0] we_rs_b, we_ss_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_web0l0 (.I0(we_b0lo_raw), .O(we_b0lo_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_web0l1 (.I0(we_b0lo_raw), .O(we_b0lo_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_web0h0 (.I0(we_b0hi_raw), .O(we_b0hi_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_web0h1 (.I0(we_b0hi_raw), .O(we_b0hi_b[1]));
	assign we_b1_b[0] = we_b1_raw;
	assign we_b1_b[1] = we_b1_raw;
	assign we_b2_b[0] = we_b2_raw;
	assign we_b2_b[1] = we_b2_raw;
	assign we_dmab_b[0] = we_dmab_raw;
	assign we_dmab_b[1] = we_dmab_raw;
	assign we_dmad_b[0] = we_dmad_raw;
	assign we_dmad_b[1] = we_dmad_raw;
	assign we_rs_b[0] = we_rs_raw;
	assign we_rs_b[1] = we_rs_raw;
	assign we_rs_b[2] = we_rs_raw;
	assign we_rs_b[3] = we_rs_raw;
	assign we_ss_b[0] = we_ss_raw;
	assign we_ss_b[1] = we_ss_raw;
	assign we_ss_b[2] = we_ss_raw;
	assign we_ss_b[3] = we_ss_raw;
	assign we_ctxb_b[0] = we_ctxb_raw;
	assign we_ctxb_b[1] = we_ctxb_raw;
	assign we_dext_b[0] = we_dext_raw;
	assign we_dext_b[1] = we_dext_raw;

	wire dmasem_hcr_we = 1'b0;
	wire dmasem_inc_we = dma_active && dma_done;
	wire [1:0] dmasem_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_dsm0 (.I0(dmasem_hcr_we), .I1(dmasem_inc_we), .O(dmasem_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_dsm1 (.I0(dmasem_hcr_we), .I1(dmasem_inc_we), .O(dmasem_we_b[1]));

	wire st_is_rhdr_raw = (st == R_HDR);
	wire [2:0] st_rhdr_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_strh0 (.I0(st_is_rhdr_raw), .O(st_rhdr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_strh1 (.I0(st_is_rhdr_raw), .O(st_rhdr_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_strh2 (.I0(st_is_rhdr_raw), .O(st_rhdr_b[2]));

	wire st_is_rdec_raw;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_strdec (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_is_rdec_raw));
	wire [1:0] st_rdec_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_strd0 (.I0(st_is_rdec_raw), .O(st_rdec_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_strd1 (.I0(st_is_rdec_raw), .O(st_rdec_b[1]));

	wire [2:0] st_rdec2_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_2 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[2]));

	wire [1:0] save_rptr_we_b;
	assign save_rptr_we_b[0] = save_rptr_raw;
	assign save_rptr_we_b[1] = save_rptr_raw;

	wire [1:0] dma_wptr_we_b;
	assign dma_wptr_we_b[0] = 1'b0;
	assign dma_wptr_we_b[1] = 1'b0;

	wire we_sz0_raw = hcr_we_cfg_b[3] && (widx_b[5:0] == I_SIZE0[5:0]) && wmask_c[0];
	wire we_sz1_raw = 1'b0;
	wire we_sz2_raw = 1'b0;
	wire [1:0] we_sz0_b, we_sz1_b, we_sz2_b;
	assign we_sz0_b[0] = we_sz0_raw;
	assign we_sz0_b[1] = we_sz0_raw;
	assign we_sz1_b[0] = we_sz1_raw;
	assign we_sz1_b[1] = we_sz1_raw;
	assign we_sz2_b[0] = we_sz2_raw;
	assign we_sz2_b[1] = we_sz2_raw;
	wire [4:0] dec_sz_c = decode_ring_size(wdata_c);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire we_b0lo_raw = hcr_we_cfg_b[0] && (widx_a[5:0] == I_BASE0_LO[5:0] || widx_a[5:0] == I_BASE0[5:0]);
	wire we_b0hi_raw = hcr_we_cfg_b[0] && (widx_a[5:0] == I_BASE0_HI[5:0]);
	wire we_b1_raw   = 1'b0;
	wire we_b2_raw   = 1'b0;
	wire we_dmab_raw = 1'b0;
	wire we_dmad_raw = 1'b0;
	wire we_rs_raw   = 1'b0;
	wire we_ss_raw   = 1'b0;
	wire we_ctxb_raw = 1'b0;
	wire we_dext_raw = 1'b0;

	wire [1:0] we_b0lo_b, we_b0hi_b, we_b1_b, we_b2_b, we_dmab_b, we_dmad_b, we_ctxb_b, we_dext_b;
	wire [3:0] we_rs_b, we_ss_b;
	assign we_b0lo_b[0] = we_b0lo_raw;
	assign we_b0lo_b[1] = we_b0lo_raw;
	assign we_b0hi_b[0] = we_b0hi_raw;
	assign we_b0hi_b[1] = we_b0hi_raw;
	assign we_b1_b[0] = we_b1_raw;
	assign we_b1_b[1] = we_b1_raw;
	assign we_b2_b[0] = we_b2_raw;
	assign we_b2_b[1] = we_b2_raw;
	assign we_dmab_b[0] = we_dmab_raw;
	assign we_dmab_b[1] = we_dmab_raw;
	assign we_dmad_b[0] = we_dmad_raw;
	assign we_dmad_b[1] = we_dmad_raw;
	assign we_rs_b[0] = we_rs_raw;
	assign we_rs_b[1] = we_rs_raw;
	assign we_rs_b[2] = we_rs_raw;
	assign we_rs_b[3] = we_rs_raw;
	assign we_ss_b[0] = we_ss_raw;
	assign we_ss_b[1] = we_ss_raw;
	assign we_ss_b[2] = we_ss_raw;
	assign we_ss_b[3] = we_ss_raw;
	assign we_ctxb_b[0] = we_ctxb_raw;
	assign we_ctxb_b[1] = we_ctxb_raw;
	assign we_dext_b[0] = we_dext_raw;
	assign we_dext_b[1] = we_dext_raw;

	wire dmasem_hcr_we = 1'b0;
	wire dmasem_inc_we = dma_active && dma_done;
	wire [1:0] dmasem_we_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_dsm0 (.I0(dmasem_hcr_we), .I1(dmasem_inc_we), .O(dmasem_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_dsm1 (.I0(dmasem_hcr_we), .I1(dmasem_inc_we), .O(dmasem_we_b[1]));

	wire st_is_rhdr_raw = (st == R_HDR);
	wire [2:0] st_rhdr_b;
	assign st_rhdr_b[0] = st_is_rhdr_raw;
	assign st_rhdr_b[1] = st_is_rhdr_raw;
	assign st_rhdr_b[2] = st_is_rhdr_raw;

	wire st_is_rdec_raw;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_strdec (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_is_rdec_raw));
	wire [1:0] st_rdec_b;
	assign st_rdec_b[0] = st_is_rdec_raw;
	assign st_rdec_b[1] = st_is_rdec_raw;

	wire [2:0] st_rdec2_b;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_0 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_1 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2000)) u_srd2_2 (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_rdec2_b[2]));

	wire [1:0] save_rptr_we_b;
	assign save_rptr_we_b[0] = save_rptr_raw;
	assign save_rptr_we_b[1] = save_rptr_raw;

	wire [1:0] dma_wptr_we_b;
	assign dma_wptr_we_b[0] = 1'b0;
	assign dma_wptr_we_b[1] = 1'b0;

	wire we_sz0_raw = hcr_we_cfg_b[3] && (widx_b[5:0] == I_SIZE0[5:0]) && wmask_c[0];
	wire we_sz1_raw = 1'b0;
	wire we_sz2_raw = 1'b0;
	wire [1:0] we_sz0_b, we_sz1_b, we_sz2_b;
	assign we_sz0_b[0] = we_sz0_raw;
	assign we_sz0_b[1] = we_sz0_raw;
	assign we_sz1_b[0] = we_sz1_raw;
	assign we_sz1_b[1] = we_sz1_raw;
	assign we_sz2_b[0] = we_sz2_raw;
	assign we_sz2_b[1] = we_sz2_raw;
	wire [4:0] dec_sz_c = decode_ring_size(wdata_c);
`endif

	integer bi;
`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge ring_rst_high) begin
		if (ring_rst_high) begin
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
`endif
			for (bi = 0; bi < 3; bi = bi + 1) begin
				base[bi] <= 64'd0;
				size[bi] <= 5'd12;
				size_m_lo[bi] <= 5'd12;
				size_m_hi[bi] <= 5'd12;
				size_n_lo[bi] <= 5'd12;
				size_n_hi[bi] <= 5'd12;
			end
			dma_base <= 32'd0;
			dma_size <= 5'd12;
			size0_enc_r <= 32'h0000_1000;
			dma_dst_base_r <= 32'd0;
			dma_src_row_stride_r <= 32'd512;
			dma_dst_row_stride_r <= 32'd512;
			dma_src_slice_stride_r <= 32'd65536;
			dma_dst_slice_stride_r <= 32'd65536;
			dma_copy_w_r <= 16'd512;
			dma_copy_h_r <= 16'd128;
			dma_copy_d_r <= 16'd64;
			ctx_base <= 32'd0;
			ctx_base_z_r <= 1'b1;
			prio_mode <= 4'd0;
			pm_strict_r <= 1'b0;
			pm_rr_r     <= 1'b0;
			preempt_auto <= 1'b0;
		end else begin
			pm_strict_r <= (prio_mode == 4'd1);
			pm_rr_r     <= (prio_mode == 4'd2);
`ifdef SYNTH_KINTEX7
			ctx_base_z_r <= (ctx_base == 32'd0);
			if (we_b0lo_b[0]) base[0][15:0]  <= ((base[0][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b0lo_b[1]) base[0][31:16] <=  (base[0][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);
			if (we_b0hi_b[0]) base[0][47:32] <=  (base[0][47:32] & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0]);
			if (we_b0hi_b[1]) base[0][63:48] <=  (base[0][63:48] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_b1_b[0])   base[1][15:0]  <= ((base[1][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b1_b[1])   base[1][31:16] <=  (base[1][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_b2_b[0])   base[2][15:0]  <= ((base[2][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b2_b[1])   base[2][31:16] <=  (base[2][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_dmab_b[0]) dma_base[15:0]  <= ((dma_base[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0])) & 16'hFFE0;
			if (we_dmab_b[1]) dma_base[31:16] <=  (dma_base[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_dmad_b[0]) dma_dst_base_r[15:0]  <= (dma_dst_base_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_dmad_b[1]) dma_dst_base_r[31:16] <= (dma_dst_base_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_rs_b[0]) dma_src_row_stride_r[15:0]  <= (dma_src_row_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_rs_b[1]) dma_src_row_stride_r[31:16] <= (dma_src_row_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);
			if (we_rs_b[2]) dma_dst_row_stride_r[15:0]  <= (dma_dst_row_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_rs_b[3]) dma_dst_row_stride_r[31:16] <= (dma_dst_row_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_ss_b[0]) dma_src_slice_stride_r[15:0]  <= (dma_src_slice_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_ss_b[1]) dma_src_slice_stride_r[31:16] <= (dma_src_slice_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);
			if (we_ss_b[2]) dma_dst_slice_stride_r[15:0]  <= (dma_dst_slice_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_ss_b[3]) dma_dst_slice_stride_r[31:16] <= (dma_dst_slice_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_ctxb_b[0]) ctx_base[15:0]  <= ((ctx_base[15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_ctxb_b[1]) ctx_base[31:16] <=  (ctx_base[31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_dext_b[0]) dma_copy_w_r <= wdata_b[15:0];
			if (we_dext_b[1]) begin
				dma_copy_h_r <= wdata_b[23:16] != 8'd0 ? {8'd0, wdata_b[23:16]} : 16'd128;
				dma_copy_d_r <= wdata_b[31:24] != 8'd0 ? {8'd0, wdata_b[31:24]} : 16'd64;
			end

			if (we_sz0_ce[0]) size0_enc_r[15:0] <= size0_enc_next[15:0];
			if (we_sz0_ce[1]) size0_enc_r[31:16] <= size0_enc_next[31:16];
			if (we_sz0_ce[2]) begin
				size[0]      <= dec_sz_r;
				size_m_lo[0] <= dec_sz_r;
				size_m_hi[0] <= dec_sz_r;
			end
			if (we_sz0_ce[3]) begin
				size_n_lo[0] <= dec_sz_r;
				size_n_hi[0] <= dec_sz_r;
			end
			if (we_sz1_b[0] && dec_sz_nz_r) begin
				size[1]      <= dec_sz_r;
				size_m_lo[1] <= dec_sz_r;
				size_m_hi[1] <= dec_sz_r;
				size_n_lo[1] <= dec_sz_r;
				size_n_hi[1] <= dec_sz_r;
			end
			if (we_sz2_b[0] && dec_sz_nz_r) begin
				size[2]      <= dec_sz_r;
				size_m_lo[2] <= dec_sz_r;
				size_m_hi[2] <= dec_sz_r;
				size_n_lo[2] <= dec_sz_r;
				size_n_hi[2] <= dec_sz_r;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			ctx_base_z_r <= (ctx_base == 32'd0);
			if (we_b0lo_b[0]) base[0][15:0]  <= ((base[0][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b0lo_b[1]) base[0][31:16] <=  (base[0][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);
			if (we_b0hi_b[0]) base[0][47:32] <=  (base[0][47:32] & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0]);
			if (we_b0hi_b[1]) base[0][63:48] <=  (base[0][63:48] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_b1_b[0])   base[1][15:0]  <= ((base[1][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b1_b[1])   base[1][31:16] <=  (base[1][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_b2_b[0])   base[2][15:0]  <= ((base[2][15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_b2_b[1])   base[2][31:16] <=  (base[2][31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_dmab_b[0]) dma_base[15:0]  <= ((dma_base[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0])) & 16'hFFE0;
			if (we_dmab_b[1]) dma_base[31:16] <=  (dma_base[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_dmad_b[0]) dma_dst_base_r[15:0]  <= (dma_dst_base_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_dmad_b[1]) dma_dst_base_r[31:16] <= (dma_dst_base_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_rs_b[0]) dma_src_row_stride_r[15:0]  <= (dma_src_row_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_rs_b[1]) dma_src_row_stride_r[31:16] <= (dma_src_row_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);
			if (we_rs_b[2]) dma_dst_row_stride_r[15:0]  <= (dma_dst_row_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_rs_b[3]) dma_dst_row_stride_r[31:16] <= (dma_dst_row_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_ss_b[0]) dma_src_slice_stride_r[15:0]  <= (dma_src_slice_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_ss_b[1]) dma_src_slice_stride_r[31:16] <= (dma_src_slice_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);
			if (we_ss_b[2]) dma_dst_slice_stride_r[15:0]  <= (dma_dst_slice_stride_r[15:0]  & ~wmask_b[15:0])  | (wdata_b[15:0]  & wmask_b[15:0]);
			if (we_ss_b[3]) dma_dst_slice_stride_r[31:16] <= (dma_dst_slice_stride_r[31:16] & ~wmask_b[31:16]) | (wdata_b[31:16] & wmask_b[31:16]);

			if (we_ctxb_b[0]) ctx_base[15:0]  <= ((ctx_base[15:0]  & ~wmask_a[15:0])  | (wdata_a[15:0]  & wmask_a[15:0])) & 16'hFFE0;
			if (we_ctxb_b[1]) ctx_base[31:16] <=  (ctx_base[31:16] & ~wmask_a[31:16]) | (wdata_a[31:16] & wmask_a[31:16]);

			if (we_dext_b[0]) dma_copy_w_r <= wdata_b[15:0];
			if (we_dext_b[1]) begin
				dma_copy_h_r <= wdata_b[23:16] != 8'd0 ? {8'd0, wdata_b[23:16]} : 16'd128;
				dma_copy_d_r <= wdata_b[31:24] != 8'd0 ? {8'd0, wdata_b[31:24]} : 16'd64;
			end

			if (we_sz0_ce[0]) size0_enc_r[15:0] <= size0_enc_next[15:0];
			if (we_sz0_ce[1]) size0_enc_r[31:16] <= size0_enc_next[31:16];
			if (we_sz0_ce[2]) begin
				size[0]      <= dec_sz_r;
				size_m_lo[0] <= dec_sz_r;
				size_m_hi[0] <= dec_sz_r;
			end
			if (we_sz0_ce[3]) begin
				size_n_lo[0] <= dec_sz_r;
				size_n_hi[0] <= dec_sz_r;
			end
			if (we_sz1_b[0] && dec_sz_nz_r) begin
				size[1]      <= dec_sz_r;
				size_m_lo[1] <= dec_sz_r;
				size_m_hi[1] <= dec_sz_r;
				size_n_lo[1] <= dec_sz_r;
				size_n_hi[1] <= dec_sz_r;
			end
			if (we_sz2_b[0] && dec_sz_nz_r) begin
				size[2]      <= dec_sz_r;
				size_m_lo[2] <= dec_sz_r;
				size_m_hi[2] <= dec_sz_r;
				size_n_lo[2] <= dec_sz_r;
				size_n_hi[2] <= dec_sz_r;
			end
`else
			if (hcr_we_cfg) begin
				if (open) begin
					if (widx_a == I_BASE0_LO || widx_a == I_BASE0)
						base[0][31:0] <= ((base[0][31:0] & ~wmask_a) | (wdata_a & wmask_a)) & 32'hFFFFFFE0;
					if (widx_a == I_BASE0_HI)
						base[0][63:32] <= (base[0][63:32] & ~wmask_a) | (wdata_a & wmask_a);
					if (widx_b == I_SIZE0 && wmask_c[0] && dec_sz_nz_r) begin
						size[0]      <= dec_sz_r;
						size_m_lo[0] <= dec_sz_r;
						size_m_hi[0] <= dec_sz_r;
						size_n_lo[0] <= dec_sz_r;
						size_n_hi[0] <= dec_sz_r;
					end
				end
			end
`endif
		end
	end

`ifdef SYNTH_KINTEX7
	/* Flat <= 4-LUT FSM next-state computation (zero CARRY4/long chains). */
	wire [3:0] st_d_idle, st_d_hdr, st_d_dec2, st_d_n, st_d_v, st_d_adv, st_d_rsp, st_d_csr, st_d_crr;

	wire idle_save_c = preempt_pending && aqid_nz_b[0] && !ctx_base_zero;
	wire idle_rest_c = !idle_save_c && preempt_act_b[0] && q0_empty_b[1] && !ctx_base_zero;
	wire idle_np, idle_hw, idle_age3, idle_hdr_c;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00FF57FF57FF57)) u_idle_np (
		.I0(preempt_pending), .I1(active_qid[0]), .I2(active_qid[1]),
		.I3(ctx_base_zero), .I4(preempt_act_b[0]), .I5(q0_empty_b[1]),
		.O(idle_np)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEDC3210)) u_idle_hw (
		.I0(active_qid[0]), .I1(active_qid[1]),
		.I2(hw_sel[0]), .I3(hw_sel[1]), .I4(hw_sel[2]),
		.O(idle_hw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_idle_age3 (
		.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(idle_age3)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hF200000000000000)) u_idle_hdr (
		.I0(run), .I1(halt), .I2(step_pend),
		.I3(idle_np), .I4(idle_hw), .I5(idle_age3),
		.O(idle_hdr_c)
	);
	assign st_d_idle = {idle_save_c | idle_rest_c, 1'b0, idle_rest_c, idle_hdr_c};

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_hdr_d3 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h1111)) u_hdr_d2 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[2]));
	assign st_d_hdr[1] = 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFEF)) u_hdr_d0 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[0]));

	assign st_d_dec2[3] = 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFFFEFEFE)) u_d2_d2 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[2]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEFFFFFF)) u_d2_d1 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[1]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEFFFEFE)) u_d2_d0 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[0]));
	assign st_d_n   = !hit_n ? R_REQ : ((n_ready && k_eq_n) ? R_ADV : R_N);
	assign st_d_v   = v_done ? R_ADV : R_V;
	assign st_d_adv = (preempt_pending && aqid_nz_b[1] && !ctx_base_zero) ? R_CTX_SAVE_REQ : R_IDLE;
	assign st_d_rsp = rsp_is_tag0_r ? (rsp_error_r ? R_FAULT : (ret_n ? R_N : R_HDR)) : R_RSP;
	assign st_d_csr = rsp_is_tag2_r ? R_IDLE : R_CTX_SAVE_RSP;
	assign st_d_crr = rsp_is_tag3_r ? R_IDLE : R_CTX_REST_RSP;

	/* Fold ring_req_ready (R_REQ=5, R_CTX_SAVE_REQ=8, R_CTX_REST_REQ=10) directly into state-group LUT6s! */
	wire [3:0] st_fin_c;
	genvar gst;
	generate
		for (gst = 0; gst < 4; gst = gst + 1) begin : g_st_mux
			wire g0_o, g1_o, g2_o, g3_o;
			/* Group 0: 0=R_IDLE, 1=R_HDR, 2=R_N, 3=R_V */
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g0 (
				.I0(st_d_idle[gst]), .I1(st_d_hdr[gst]), .I2(st_d_n[gst]), .I3(st_d_v[gst]),
				.I4(st[0]), .I5(st[1]), .O(g0_o)
			);
			/* Group 1: 4=R_ADV, 5=R_REQ (5->6 on ready), 6=R_RSP, 7=R_FAULT(stay 7) */
			wire req5_bit = (gst == 0) ? ~ring_req_ready : ((gst == 1) ? ring_req_ready : ((gst == 2) ? 1'b1 : 1'b0));
			wire flt7_bit = (gst < 3) ? 1'b1 : 1'b0;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g1 (
				.I0(st_d_adv[gst]), .I1(req5_bit), .I2(st_d_rsp[gst]), .I3(flt7_bit),
				.I4(st[0]), .I5(st[1]), .O(g1_o)
			);
			/* Group 2: 8=SAVE_REQ (8->9 on ready), 9=SAVE_RSP, 10=REST_REQ (10->11 on ready), 11=REST_RSP */
			wire sreq8_bit  = (gst == 0) ? ring_req_ready : ((gst == 3) ? 1'b1 : 1'b0);
			wire rreq10_bit = (gst == 0) ? ring_req_ready : ((gst == 1 || gst == 3) ? 1'b1 : 1'b0);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g2 (
				.I0(sreq8_bit), .I1(st_d_csr[gst]), .I2(rreq10_bit), .I3(st_d_crr[gst]),
				.I4(st[0]), .I5(st[1]), .O(g2_o)
			);
			/* Group 3: 12=R_DEC (->13), 13=R_DEC2, 14/15=st */
			wire dec12_bit = (gst == 0 || gst == 2 || gst == 3) ? 1'b1 : 1'b0;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g3 (
				.I0(dec12_bit), .I1(st_d_dec2[gst]), .I2(st[gst]), .I3(st[gst]),
				.I4(st[0]), .I5(st[1]), .O(g3_o)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_gf (
				.I0(g0_o), .I1(g1_o), .I2(g2_o), .I3(g3_o),
				.I4(st[2]), .I5(st[3]), .O(st_fin_c[gst])
			);
		end
	endgenerate

	wire st_idle_or_adv_aq;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0011)) u_stia_aq (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_idle_or_adv_aq));
	wire aq_save_cond   = preempt_pending && ctx_base_z_r && (active_qid[0] || active_qid[1]);
	wire aq_set_0_rsp   = (st == R_CTX_SAVE_RSP) && rsp_is_tag2_r;
	wire aq_set_0       = (st_idle_or_adv_aq && aq_save_cond) || aq_set_0_rsp;

	wire aq_rest_cond   = preempt_active && q0_empty_r && ctx_base_z_r && (!preempt_pending || (!active_qid[0] && !active_qid[1]));
	wire aq_set_int_rsp = (st == R_CTX_REST_RSP) && rsp_is_tag3_r;
	wire aq_set_int     = (st_idle_sel_b[2] && aq_rest_cond) || aq_set_int_rsp;

	wire aq_go_cond     = (!preempt_pending || (!active_qid[0] && !active_qid[1])) && !(preempt_active && q0_empty_r);
	wire aq_hw_sw_ok    = !hw_sel[active_qid] && (|hw_sel);
	wire aq_set_nxt     = st_idle_sel_b[2] && aq_go_cond && aq_hw_sw_ok && go_sel;
	wire aq_ce          = aq_set_0 || aq_set_int || aq_set_nxt;
	wire [1:0] aq_d     = ({2{~aq_set_0}}) & (aq_set_int ? interrupted_qid : next_qid);
`elsif SYNTH_KINTEX_ULTRASCALE
	/* Flat <= 4-LUT FSM next-state computation (zero CARRY4/long chains). */
	wire [3:0] st_d_idle, st_d_hdr, st_d_dec2, st_d_n, st_d_v, st_d_adv, st_d_rsp, st_d_csr, st_d_crr;

	wire idle_save_c = preempt_pending && aqid_nz_b[0] && !ctx_base_zero;
	wire idle_rest_c = !idle_save_c && preempt_act_b[0] && q0_empty_b[1] && !ctx_base_zero;
	wire idle_np, idle_hw, idle_age3, idle_hdr_c;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00FF57FF57FF57)) u_idle_np (
		.I0(preempt_pending), .I1(active_qid[0]), .I2(active_qid[1]),
		.I3(ctx_base_zero), .I4(preempt_act_b[0]), .I5(q0_empty_b[1]),
		.O(idle_np)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEDC3210)) u_idle_hw (
		.I0(active_qid[0]), .I1(active_qid[1]),
		.I2(hw_sel[0]), .I3(hw_sel[1]), .I4(hw_sel[2]),
		.O(idle_hw)
	);
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_idle_age3 (
		.I0(pa_hdr_age[0]), .I1(pa_hdr_age[1]), .O(idle_age3)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hF200000000000000)) u_idle_hdr (
		.I0(run), .I1(halt), .I2(step_pend),
		.I3(idle_np), .I4(idle_hw), .I5(idle_age3),
		.O(idle_hdr_c)
	);
	assign st_d_idle = {idle_save_c | idle_rest_c, 1'b0, idle_rest_c, idle_hdr_c};

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0010)) u_hdr_d3 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[3]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h1111)) u_hdr_d2 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[2]));
	assign st_d_hdr[1] = 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFEF)) u_hdr_d0 (.I0(hdr_wait[0]), .I1(hdr_wait[1]), .I2(hit_n_r), .I3(stale), .O(st_d_hdr[0]));

	assign st_d_dec2[3] = 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFFFEFEFE)) u_d2_d2 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[2]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEFFFFFF)) u_d2_d1 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[1]));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFEFFFEFE)) u_d2_d0 (.I0(h_bad_words_r), .I1(h_bad_avail_r), .I2(h_novf_r), .I3(h_class_r[0]), .I4(h_class_r[1]), .O(st_d_dec2[0]));
	assign st_d_n   = !hit_n ? R_REQ : ((n_ready && k_eq_n) ? R_ADV : R_N);
	assign st_d_v   = v_done ? R_ADV : R_V;
	assign st_d_adv = (preempt_pending && aqid_nz_b[1] && !ctx_base_zero) ? R_CTX_SAVE_REQ : R_IDLE;
	assign st_d_rsp = rsp_is_tag0_r ? (rsp_error_r ? R_FAULT : (ret_n ? R_N : R_HDR)) : R_RSP;
	assign st_d_csr = rsp_is_tag2_r ? R_IDLE : R_CTX_SAVE_RSP;
	assign st_d_crr = rsp_is_tag3_r ? R_IDLE : R_CTX_REST_RSP;

	/* Fold ring_req_ready (R_REQ=5, R_CTX_SAVE_REQ=8, R_CTX_REST_REQ=10) directly into state-group LUT6s! */
	wire [3:0] st_fin_c;
	genvar gst;
	generate
		for (gst = 0; gst < 4; gst = gst + 1) begin : g_st_mux
			wire g0_o, g1_o, g2_o, g3_o;
			/* Group 0: 0=R_IDLE, 1=R_HDR, 2=R_N, 3=R_V */
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g0 (
				.I0(st_d_idle[gst]), .I1(st_d_hdr[gst]), .I2(st_d_n[gst]), .I3(st_d_v[gst]),
				.I4(st[0]), .I5(st[1]), .O(g0_o)
			);
			/* Group 1: 4=R_ADV, 5=R_REQ (5->6 on ready), 6=R_RSP, 7=R_FAULT(stay 7) */
			wire req5_bit = (gst == 0) ? ~ring_req_ready : ((gst == 1) ? ring_req_ready : ((gst == 2) ? 1'b1 : 1'b0));
			wire flt7_bit = (gst < 3) ? 1'b1 : 1'b0;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g1 (
				.I0(st_d_adv[gst]), .I1(req5_bit), .I2(st_d_rsp[gst]), .I3(flt7_bit),
				.I4(st[0]), .I5(st[1]), .O(g1_o)
			);
			/* Group 2: 8=SAVE_REQ (8->9 on ready), 9=SAVE_RSP, 10=REST_REQ (10->11 on ready), 11=REST_RSP */
			wire sreq8_bit  = (gst == 0) ? ring_req_ready : ((gst == 3) ? 1'b1 : 1'b0);
			wire rreq10_bit = (gst == 0) ? ring_req_ready : ((gst == 1 || gst == 3) ? 1'b1 : 1'b0);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g2 (
				.I0(sreq8_bit), .I1(st_d_csr[gst]), .I2(rreq10_bit), .I3(st_d_crr[gst]),
				.I4(st[0]), .I5(st[1]), .O(g2_o)
			);
			/* Group 3: 12=R_DEC (->13), 13=R_DEC2, 14/15=st */
			wire dec12_bit = (gst == 0 || gst == 2 || gst == 3) ? 1'b1 : 1'b0;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_g3 (
				.I0(dec12_bit), .I1(st_d_dec2[gst]), .I2(st[gst]), .I3(st[gst]),
				.I4(st[0]), .I5(st[1]), .O(g3_o)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_gf (
				.I0(g0_o), .I1(g1_o), .I2(g2_o), .I3(g3_o),
				.I4(st[2]), .I5(st[3]), .O(st_fin_c[gst])
			);
		end
	endgenerate

	wire st_idle_or_adv_aq;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0011)) u_stia_aq (.I0(st[0]), .I1(st[1]), .I2(st[2]), .I3(st[3]), .O(st_idle_or_adv_aq));
	wire aq_save_cond   = preempt_pending && ctx_base_z_r && (active_qid[0] || active_qid[1]);
	wire aq_set_0_rsp   = (st == R_CTX_SAVE_RSP) && rsp_is_tag2_r;
	wire aq_set_0       = (st_idle_or_adv_aq && aq_save_cond) || aq_set_0_rsp;

	wire aq_rest_cond   = preempt_active && q0_empty_r && ctx_base_z_r && (!preempt_pending || (!active_qid[0] && !active_qid[1]));
	wire aq_set_int_rsp = (st == R_CTX_REST_RSP) && rsp_is_tag3_r;
	wire aq_set_int     = (st_idle_sel_b[2] && aq_rest_cond) || aq_set_int_rsp;

	wire aq_go_cond     = (!preempt_pending || (!active_qid[0] && !active_qid[1])) && !(preempt_active && q0_empty_r);
	wire aq_hw_sw_ok    = !hw_sel[active_qid] && (|hw_sel);
	wire aq_set_nxt     = st_idle_sel_b[2] && aq_go_cond && aq_hw_sw_ok && go_sel;
	wire aq_ce          = aq_set_0 || aq_set_int || aq_set_nxt;
	wire [1:0] aq_d     = ({2{~aq_set_0}}) & (aq_set_int ? interrupted_qid : next_qid);
`else
	wire idle_save_c  = preempt_pending && aqid_nz_b[0] && !ctx_base_zero;
	wire idle_rest_c  = !idle_save_c && preempt_act_b[0] && q0_empty_b[1] && !ctx_base_zero;
	wire idle_inst_sw = (preempt_pending && aqid_nz_b[0] && ctx_base_zero) ||
		(preempt_act_b[0] && q0_empty_b[1] && ctx_base_zero);
	wire idle_hdr_c   = go_sel && !idle_save_c && !idle_rest_c && !idle_inst_sw && hw_sel[active_qid] && (pa_hdr_age == 2'd3);
	wire [3:0] st_d_idle = idle_save_c ? R_CTX_SAVE_REQ :
		(idle_rest_c ? R_CTX_REST_REQ : (idle_hdr_c ? R_HDR : R_IDLE));
	wire [3:0] st_d_hdr  = (hdr_wait == 2'b00) ? ((!hit_n_r || stale) ? R_REQ : R_DEC) : R_HDR;
	wire [3:0] st_d_dec2 = (h_bad_words_r || h_bad_avail_r || h_novf_r) ? R_FAULT :
		((h_class_r == 2'd2) ? R_V : ((h_class_r == 2'd3) ? R_ADV : R_N));
	wire [3:0] st_d_n    = !hit_n ? R_REQ : ((n_ready && k_eq_n) ? R_ADV : R_N);
	wire [3:0] st_d_v    = v_done ? R_ADV : R_V;
	wire [3:0] st_d_adv  = (preempt_pending && aqid_nz_b[1] && !ctx_base_zero) ? R_CTX_SAVE_REQ : R_IDLE;
	wire [3:0] st_d_req  = ring_req_ready ? R_RSP : R_REQ;
	wire [3:0] st_d_rsp  = rsp_is_tag0_r ? (rsp_error_r ? R_FAULT : (ret_n ? R_N : R_HDR)) : R_RSP;
	wire [3:0] st_d_csr  = rsp_is_tag2_r ? R_IDLE : R_CTX_SAVE_RSP;
	wire [3:0] st_d_crr  = rsp_is_tag3_r ? R_IDLE : R_CTX_REST_RSP;
	reg [3:0] st_fin_c;
	always @(*) begin
		case (st)
		R_IDLE:         st_fin_c = st_d_idle;
		R_HDR:          st_fin_c = st_d_hdr;
		R_N:            st_fin_c = st_d_n;
		R_V:            st_fin_c = st_d_v;
		R_ADV:          st_fin_c = st_d_adv;
		R_REQ:          st_fin_c = st_d_req;
		R_RSP:          st_fin_c = st_d_rsp;
		R_FAULT:        st_fin_c = R_FAULT;
		R_CTX_SAVE_REQ: st_fin_c = st_d_csr;
		R_CTX_SAVE_RSP: st_fin_c = st_d_csr;
		R_CTX_REST_REQ: st_fin_c = ring_req_ready ? R_CTX_REST_RSP : R_CTX_REST_REQ;
		R_CTX_REST_RSP: st_fin_c = st_d_crr;
		R_DEC:          st_fin_c = R_DEC2;
		R_DEC2:         st_fin_c = st_d_dec2;
		default:        st_fin_c = st;
		endcase
	end
	/* Fix R_CTX_SAVE_REQ transition separately since st_d_csr is for RSP */
	wire [3:0] st_next_final = (st == R_CTX_SAVE_REQ) ? (ring_req_ready ? R_CTX_SAVE_RSP : R_CTX_SAVE_REQ) : st_fin_c;
`endif
`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge ring_core_rst_high) begin
		if (ring_core_rst_high) begin
`else
	always @(posedge clk or negedge core_rst_n) begin
		if (!core_rst_n) begin
`endif
			for (bi = 0; bi < 3; bi = bi + 1) begin
				wptr[bi] <= 22'd0;
				queue_sema_id[bi] <= 3'd0;
			end
			queue_sema_wait <= 3'd0;
			for (bi = 0; bi < 8; bi = bi + 1)
				sema[bi] <= 8'd0;

			dma_wptr   <= 18'd0;
			dma_rptr   <= 18'd0;
			dma_sem    <= 32'd0;
			dma_active <= 1'b0;
			dma_req_v  <= 1'b0;

			active_qid      <= 2'd0;
			aqid_av_r0a     <= 2'd0;
			aqid_av_r0b     <= 2'd0;
			aqid_av_r1a     <= 2'd0;
			aqid_av_r1b     <= 2'd0;
			aqid_av_r2      <= 2'd0;
			aqid_av_r3      <= 2'd0;
			interrupted_qid <= 2'd0;
			saved_rptr      <= 22'd0;
			preempt_pending <= 1'b0;
			preempt_active  <= 1'b0;
			preempt_saved   <= 1'b0;
			preempt_pulse   <= 1'b0;
			last_non_q0     <= 2'd1;

			st        <= R_IDLE;
			st_is_rsp_r <= 1'b0;
			pf_st_is_rsp_r <= 1'b0;
			ret_n     <= 1'b0;
			pa        <= {ADDR_W{1'b0}};
			base_act_r    <= 64'd0;
			rptr_act_r    <= 22'd0;
			pa_hdr_lo_r   <= 24'd0;
			pa_hdr_c2_r0  <= 1'b0;
			pa_hdr_c2_r1  <= 1'b0;
			base_hi_r     <= {(ADDR_W-24){1'b0}};
			base_hi_inc_r <= {(ADDR_W-24){1'b0}};
			pa_hdr_r      <= {ADDR_W{1'b0}};
			pa_hdr_age    <= 2'd0;
			cur       <= 22'd0;
			n         <= 16'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			k         <= 16'd0;
`endif
			tag       <= 8'd0;
			hdr_data  <= 32'd0;
			step_pend <= 1'b0;
			stale     <= 1'b0;
			h_bad_r   <= 1'b0;
			h_bad_words_r <= 1'b0;
			h_bad_avail_r <= 1'b0;
			h_novf_r  <= 1'b0;
			h_class_r <= 2'd0;
			p_end_r     <= 22'd0;
			cur_p1_hi_r <= 6'd0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			for (bi = 0; bi < 4; bi = bi + 1) begin
				pa_word_r[bi] <= {WS{1'b0}};
				pa_idx_r[bi]  <= 3'd0;
			end
			pa_widx_r   <= 3'd0;
			pf_idx_r    <= 3'd0;
			k_eq_n_r    <= 1'b0;
			n_m1_r      <= 16'd0;
			cur_tag_r   <= {LA_W{1'b0}};
			cur_valid_r <= 1'b0;
			hdr_wait    <= 2'b10;
			next_line_r <= {LA_W{1'b0}};
			nxt_tag_r   <= {LA_W{1'b0}};
			nxt_valid_r <= 1'b0;
			nl_settle_r <= 3'b000;
`endif
			pf_st     <= PF_IDLE;
			pf_line   <= {LA_W{1'b0}};
			f_pulse   <= 1'b0;
			f_kind    <= 4'd0;
			f_addr    <= 32'd0;
			ret_pulse <= 1'b0;
`ifdef SYNTH_KINTEX7
			for (bi = 0; bi < 32; bi = bi + 1)
				pa_word_r[bi] <= 3'd0;
			for (bi = 0; bi < 8; bi = bi + 1)
				pa_idx_r[bi] <= 3'd0;
			pa_widx_r[0] <= 3'd0;
			pa_widx_r[1] <= 3'd0;
			pf_idx_r    <= 3'd0;
			k_eq_n_r    <= 1'b0;
			n_m1_r      <= 16'd0;
			cur_tag_r   <= {LA_W{1'b0}};
			cur_valid_r <= 1'b0;
			hdr_wait    <= 2'b10;
			next_line_r <= {LA_W{1'b0}};
			nxt_tag_r   <= {LA_W{1'b0}};
			nxt_valid_r <= 1'b0;
			nl_settle_r <= 3'b000;
`elsif SYNTH_KINTEX_ULTRASCALE
			for (bi = 0; bi < 32; bi = bi + 1)
				pa_word_r[bi] <= 3'd0;
			for (bi = 0; bi < 8; bi = bi + 1)
				pa_idx_r[bi] <= 3'd0;
			pa_widx_r[0] <= 3'd0;
			pa_widx_r[1] <= 3'd0;
			pf_idx_r    <= 3'd0;
			k_eq_n_r    <= 1'b0;
			n_m1_r      <= 16'd0;
			cur_tag_r   <= {LA_W{1'b0}};
			cur_valid_r <= 1'b0;
			hdr_wait    <= 2'b10;
			next_line_r <= {LA_W{1'b0}};
			nxt_tag_r   <= {LA_W{1'b0}};
			nxt_valid_r <= 1'b0;
			nl_settle_r <= 3'b000;
`endif
		end else begin
			f_pulse       <= 1'b0;
			ret_pulse     <= 1'b0;
			preempt_pulse <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			cur_tag_r   <= buf_tag[pa_widx_r];
			cur_valid_r <= buf_valid[pa_widx_r];
			next_line_r <= next_line;
			nxt_tag_r   <= buf_tag[next_idx];
			nxt_valid_r <= buf_valid[next_idx];
			nl_settle_r <= {1'b0, nl_settle_r[2:1]};
			if (st == R_HDR) begin
				hdr_data <= want_data;
				k        <= 16'd0;
			end
			if (st == R_DEC) begin
				n             <= h_n;
				tag           <= hdr_data[23:16];
				cur_p1_hi_r   <= cur_p1_hi_w[5:0];
				h_bad_words_r <= h_bad_words;
				h_bad_avail_r <= h_bad_avail;
				h_novf_r      <= h_novf;
				h_class_r     <= h_class;
			end
			if (st == R_DEC2) begin
				k_eq_n_r <= (n == 16'd0);
				n_m1_r   <= n_m1_s;
				p_end_r  <= p_end;
			end
			if (save_rptr_raw) begin
				interrupted_qid <= active_qid;
				saved_rptr      <= (st == R_IDLE) ? (active_qid[1] ? rptr[2] : rptr[1]) : p_end_r;
			end
			if (st == R_IDLE)
				cur <= rptr_act_r;
			if ((st == R_DEC2) || ((st == R_RSP) && rsp_err_tag0_r))
				f_addr <= want_addr_ext[31:0];
`endif
`ifdef SYNTH_KINTEX7
			cur_tag_r[6:0]       <= buf_tag[widx_tag0][6:0];
			cur_tag_r[13:7]      <= buf_tag[widx_tag1][13:7];
			cur_tag_r[20:14]     <= buf_tag[widx_tag2][20:14];
			cur_tag_r[LA_W-1:21] <= buf_tag[widx_tag3][LA_W-1:21];
			cur_valid_r          <= buf_valid[widx_tag0];
			next_line_r <= next_line;
			nxt_tag_r[6:0]       <= buf_tag[nidx_tag0][6:0];
			nxt_tag_r[13:7]      <= buf_tag[nidx_tag1][13:7];
			nxt_tag_r[20:14]     <= buf_tag[nidx_tag2][20:14];
			nxt_tag_r[LA_W-1:21] <= buf_tag[nidx_tag3][LA_W-1:21];
			nxt_valid_r          <= buf_valid[nidx_tag0];
			nl_settle_r <= {1'b0, nl_settle_r[2:1]};
			if (st_rhdr_b[0]) hdr_data[15:0]  <= want_data[15:0];
			if (st_rhdr_b[1]) hdr_data[31:16] <= want_data[31:16];
			if (st_rdec_b[0]) n <= h_n;
			if (st_rdec_b[1]) begin
				tag           <= hdr_data[23:16];
				cur_p1_hi_r   <= cur_p1_hi_w[5:0];
				h_bad_words_r <= h_bad_words;
				h_bad_avail_r <= h_bad_avail;
				h_novf_r      <= h_novf;
				h_class_r     <= h_class;
			end
			if (st_rdec2_b[0]) begin
				k_eq_n_r <= (n == 16'd0);
				n_m1_r[7:0] <= n_m1_s[7:0];
			end
			if (st_rdec2_b[1]) begin
				n_m1_r[15:8] <= n_m1_s[15:8];
				p_end_r[10:0] <= p_end[10:0];
			end
			if (st_rdec2_b[2]) begin
				p_end_r[21:11] <= p_end[21:11];
			end
			if (save_rptr_we_b[0]) begin
				interrupted_qid <= active_qid;
				saved_rptr[10:0] <= st_idle_sel_b[3] ? rptr[active_qid][10:0] : p_end_r[10:0];
			end
			if (save_rptr_we_b[1]) begin
				saved_rptr[21:11] <= st_idle_sel_b[7] ? rptr[active_qid][21:11] : p_end_r[21:11];
			end
			if (st_idle_sel_b[4]) cur[10:0] <= rptr_act_r[10:0];
			if (st_idle_sel_b[5]) cur[21:11] <= rptr_act_r[21:11];
			if (fa_we_b[0]) f_addr[15:0] <= want_addr[15:0];
			if (fa_we_b[1]) f_addr[31:16] <= want_addr[31:16];
`elsif SYNTH_KINTEX_ULTRASCALE
			cur_tag_r[6:0]       <= buf_tag[widx_tag0][6:0];
			cur_tag_r[13:7]      <= buf_tag[widx_tag1][13:7];
			cur_tag_r[20:14]     <= buf_tag[widx_tag2][20:14];
			cur_tag_r[LA_W-1:21] <= buf_tag[widx_tag3][LA_W-1:21];
			cur_valid_r          <= buf_valid[widx_tag0];
			next_line_r <= next_line;
			nxt_tag_r[6:0]       <= buf_tag[nidx_tag0][6:0];
			nxt_tag_r[13:7]      <= buf_tag[nidx_tag1][13:7];
			nxt_tag_r[20:14]     <= buf_tag[nidx_tag2][20:14];
			nxt_tag_r[LA_W-1:21] <= buf_tag[nidx_tag3][LA_W-1:21];
			nxt_valid_r          <= buf_valid[nidx_tag0];
			nl_settle_r <= {1'b0, nl_settle_r[2:1]};
			if (st_rhdr_b[0]) hdr_data[15:0]  <= want_data[15:0];
			if (st_rhdr_b[1]) hdr_data[31:16] <= want_data[31:16];
			if (st_rdec_b[0]) n <= h_n;
			if (st_rdec_b[1]) begin
				tag           <= hdr_data[23:16];
				cur_p1_hi_r   <= cur_p1_hi_w[5:0];
				h_bad_words_r <= h_bad_words;
				h_bad_avail_r <= h_bad_avail;
				h_novf_r      <= h_novf;
				h_class_r     <= h_class;
			end
			if (st_rdec2_b[0]) begin
				k_eq_n_r <= (n == 16'd0);
				n_m1_r[7:0] <= n_m1_s[7:0];
			end
			if (st_rdec2_b[1]) begin
				n_m1_r[15:8] <= n_m1_s[15:8];
				p_end_r[10:0] <= p_end[10:0];
			end
			if (st_rdec2_b[2]) begin
				p_end_r[21:11] <= p_end[21:11];
			end
			if (save_rptr_we_b[0]) begin
				interrupted_qid <= active_qid;
				saved_rptr[10:0] <= st_idle_sel_b[3] ? rptr[active_qid][10:0] : p_end_r[10:0];
			end
			if (save_rptr_we_b[1]) begin
				saved_rptr[21:11] <= st_idle_sel_b[7] ? rptr[active_qid][21:11] : p_end_r[21:11];
			end
			if (st_idle_sel_b[4]) cur[10:0] <= rptr_act_r[10:0];
			if (st_idle_sel_b[5]) cur[21:11] <= rptr_act_r[21:11];
			if (fa_we_b[0]) f_addr[15:0] <= want_addr[15:0];
			if (fa_we_b[1]) f_addr[31:16] <= want_addr[31:16];
`endif

`ifdef SYNTH_KINTEX7
			if (st != R_IDLE || hcr_we_cfg_b[0]) begin
				pa_hdr_age <= 2'd0;
			end else case (pa_hdr_age)
				2'd0: pa_hdr_age <= 2'd1;
				2'd1: pa_hdr_age <= 2'd2;
				2'd2: if (pa_hdr_en2) pa_hdr_age <= 2'd3;
				default: ;
			endcase
			if (ph_age1_ce[0]) pa_hdr_lo_r[15:0] <= {ph_c0 ? ph_s1_1 : ph_s1_0, ph_s0};
			if (ph_age1_ce[1]) begin
				pa_hdr_lo_r[23:16] <= {ph_c1 ? ph_s2_1 : ph_s2_0, base_act_r[1:0]};
				pa_hdr_c2_r0       <= ph_c2;
				pa_hdr_c2_r1       <= ph_c2;
			end
			if (ph_age1_ce[2]) base_hi_r     <= base_act_r[ADDR_W-1:24];
			if (ph_age1_ce[3]) base_hi_inc_r <= base_hi_inc[ADDR_W-25:0];
			if (st == R_IDLE && !hcr_we_cfg_b[0] && pa_hdr_en0)
				pa_hdr_r[23:0] <= pa_hdr_lo_r;
			if (st == R_IDLE && !hcr_we_cfg_b[0] && pa_hdr_en1)
				pa_hdr_r[ADDR_W-1:24] <= ph_hi_mux;
			if (pa_hdr_age0_en[0]) base_act_r[15:0]  <= base_act[15:0];
			if (pa_hdr_age0_en[1]) base_act_r[31:16] <= base_act[31:16];
			if (pa_hdr_age0_en[2]) base_act_r[47:32] <= base_act[47:32];
			if (pa_hdr_age0_en[3]) begin
				base_act_r[63:48] <= base_act[63:48];
				rptr_act_r        <= rptr_act;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (st != R_IDLE || hcr_we_cfg_b[0]) begin
				pa_hdr_age <= 2'd0;
			end else case (pa_hdr_age)
				2'd0: pa_hdr_age <= 2'd1;
				2'd1: pa_hdr_age <= 2'd2;
				2'd2: if (pa_hdr_en2) pa_hdr_age <= 2'd3;
				default: ;
			endcase
			if (ph_age1_ce[0]) pa_hdr_lo_r[15:0] <= {ph_c0 ? ph_s1_1 : ph_s1_0, ph_s0};
			if (ph_age1_ce[1]) begin
				pa_hdr_lo_r[23:16] <= {ph_c1 ? ph_s2_1 : ph_s2_0, base_act_r[1:0]};
				pa_hdr_c2_r0       <= ph_c2;
				pa_hdr_c2_r1       <= ph_c2;
			end
			if (ph_age1_ce[2]) base_hi_r     <= base_act_r[ADDR_W-1:24];
			if (ph_age1_ce[3]) base_hi_inc_r <= base_hi_inc[ADDR_W-25:0];
			if (st == R_IDLE && !hcr_we_cfg_b[0] && pa_hdr_en0)
				pa_hdr_r[23:0] <= pa_hdr_lo_r;
			if (st == R_IDLE && !hcr_we_cfg_b[0] && pa_hdr_en1)
				pa_hdr_r[ADDR_W-1:24] <= ph_hi_mux;
			if (pa_hdr_age0_en[0]) base_act_r[15:0]  <= base_act[15:0];
			if (pa_hdr_age0_en[1]) base_act_r[31:16] <= base_act[31:16];
			if (pa_hdr_age0_en[2]) base_act_r[47:32] <= base_act[47:32];
			if (pa_hdr_age0_en[3]) begin
				base_act_r[63:48] <= base_act[63:48];
				rptr_act_r        <= rptr_act;
			end
`else
			if (pa_hdr_age == 2'd0) begin
				base_act_r <= base_act;
				rptr_act_r <= rptr_act;
			end
			if (st != R_IDLE || (hcr_we_cfg && open)) begin
				pa_hdr_age <= 2'd0;
			end else begin
				case (pa_hdr_age)
				2'd0: begin
					pa_hdr_age <= 2'd1;
				end
				2'd1: begin
					pa_hdr_lo_r <= {
						ph_c1 ? ph_s2_1 : ph_s2_0,
						ph_c0 ? ph_s1_1 : ph_s1_0,
						ph_s0,
						base_act_r[1:0]
					};
					pa_hdr_c2_r0  <= ph_c2;
					pa_hdr_c2_r1  <= ph_c2;
					base_hi_r     <= base_act_r[ADDR_W-1:24];
					base_hi_inc_r <= base_hi_inc[ADDR_W-25:0];
					pa_hdr_age    <= 2'd2;
				end
				2'd2: begin
					pa_hdr_r   <= {ph_hi_mux, pa_hdr_lo_r};
					pa_hdr_age <= 2'd3;
				end
				default: ;
				endcase
			end
`endif

			/* Doorbells */
			if (doorbell_w[0]) begin
				wptr[0] <= wptr_new0;
				if (active_qid == 2'd0)
					stale <= 1'b1;
			end
			if (doorbell_w[1]) begin
				wptr[1] <= wptr_new1;
				if (active_qid == 2'd1)
					stale <= 1'b1;
			end
			if (doorbell_w[2]) begin
				wptr[2] <= wptr_new2;
				if (active_qid == 2'd2)
					stale <= 1'b1;
			end

`ifdef SYNTH_KINTEX7
			if (dma_wptr_we_b[0])
				dma_wptr[8:0]  <= (dma_wptr[8:0]  & ~wmask_c[8:0])  | (wdata_c[8:0]  & wmask_c[8:0]);
			if (dma_wptr_we_b[1])
				dma_wptr[17:9] <= (dma_wptr[17:9] & ~wmask_c[17:9]) | (wdata_c[17:9] & wmask_c[17:9]);
			if (dmasem_we_b[0])
				dma_sem[15:0] <= dmasem_inc_we ? dma_sem_next[15:0] : ((dma_sem[15:0] & ~wmask_c[15:0]) | (wdata_c[15:0] & wmask_c[15:0]));
			if (dmasem_we_b[1])
				dma_sem[31:16] <= dmasem_inc_we ? dma_sem_next[31:16] : ((dma_sem[31:16] & ~wmask_c[31:16]) | (wdata_c[31:16] & wmask_c[31:16]));
`elsif SYNTH_KINTEX_ULTRASCALE
			if (dma_wptr_we_b[0])
				dma_wptr[8:0]  <= (dma_wptr[8:0]  & ~wmask_c[8:0])  | (wdata_c[8:0]  & wmask_c[8:0]);
			if (dma_wptr_we_b[1])
				dma_wptr[17:9] <= (dma_wptr[17:9] & ~wmask_c[17:9]) | (wdata_c[17:9] & wmask_c[17:9]);
			if (dmasem_we_b[0])
				dma_sem[15:0] <= dmasem_inc_we ? dma_sem_next[15:0] : ((dma_sem[15:0] & ~wmask_c[15:0]) | (wdata_c[15:0] & wmask_c[15:0]));
			if (dmasem_we_b[1])
				dma_sem[31:16] <= dmasem_inc_we ? dma_sem_next[31:16] : ((dma_sem[31:16] & ~wmask_c[31:16]) | (wdata_c[31:16] & wmask_c[31:16]));
`else
			/* Host MMIO DMA writes retired in GHCI 1.0 */
`endif

			if (!dma_active && (dma_wptr != dma_rptr) && go) begin
				dma_active <= 1'b1;
				dma_req_v  <= 1'b1;
			end else if (dma_req_v && dma_req_ready) begin
				dma_req_v  <= 1'b0;
			end

			if (dma_active && dma_done) begin
				dma_active <= 1'b0;
				dma_rptr   <= dma_rptr_next;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				dma_sem    <= dma_sem_next;
`endif
			end

			/* Auto-preempt when Queue 0 receives work during Queue 1/2 execution */
			if (preempt_auto && hw_sel[0] && (active_qid != 2'd0) && !preempt_active)
				preempt_pending <= 1'b1;

			if (step)
				step_pend <= 1'b1;
			else if (run && !halt)
				step_pend <= 1'b0;

			// ---------------------------------------------------------
			// Background Prefetcher FSM
			// ---------------------------------------------------------
			case (pf_st)
			PF_IDLE: begin
				if (pf_start) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					pf_line <= next_line;
`endif
					pf_st   <= PF_REQ;
				end
			end
			PF_REQ: begin
				if (ring_req_ready && !demand_req && !ctx_req) begin
					pf_st <= PF_RSP;
					pf_st_is_rsp_r <= 1'b1;
				end
			end
			PF_RSP: begin
				if (rsp_is_tag1_r) begin
					pf_st <= PF_IDLE;
					pf_st_is_rsp_r <= 1'b0;
				end
			end
			default: pf_st <= PF_IDLE;
			endcase

			// ---------------------------------------------------------
			// Main Command Execution FSM
			// ---------------------------------------------------------
`ifdef SYNTH_KINTEX7
			st <= st_fin_c;
`elsif SYNTH_KINTEX_ULTRASCALE
			st <= st_fin_c;
`else
			st <= st_next_final;
`endif
			case (st)
			R_IDLE: begin
				if (preempt_pending && (active_qid != 2'd0)) begin
					if (ctx_base_zero) begin
						/* Instant shadow context switch */
						preempt_saved   <= 1'b1;
						preempt_pending <= 1'b0;
						preempt_active  <= 1'b1;
						preempt_pulse   <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						active_qid      <= 2'd0;
						aqid_av_r0a     <= 2'd0;
						aqid_av_r0b     <= 2'd0;
						aqid_av_r1a     <= 2'd0;
						aqid_av_r1b     <= 2'd0;
						aqid_av_r2      <= 2'd0;
						aqid_av_r3      <= 2'd0;
`endif
						pa_hdr_age      <= 2'd0;
					end
				end else if (preempt_act_sel && q0_empty_sel_b) begin
					/* High-priority Queue 0 finished: restore interrupted queue */
					if (ctx_base_zero) begin
						preempt_active  <= 1'b0;
						preempt_saved   <= 1'b0;
						preempt_pulse   <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						active_qid      <= interrupted_qid;
						aqid_av_r0a     <= interrupted_qid;
						aqid_av_r0b     <= interrupted_qid;
						aqid_av_r1a     <= interrupted_qid;
						aqid_av_r1b     <= interrupted_qid;
						aqid_av_r2      <= interrupted_qid;
						aqid_av_r3      <= interrupted_qid;
`endif
						pa_hdr_age      <= 2'd0;
					end
				end else if (go_sel) begin
					if (hw_sel[active_qid] && (pa_hdr_age == 2'd3)) begin
						step_pend <= 1'b0;
						hdr_wait  <= 2'b10;
					end else if (!hw_sel[active_qid] && (|hw_sel)) begin
						/* Arbiter switches to next active queue */
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						active_qid  <= next_qid;
						aqid_av_r0a <= next_qid;
						aqid_av_r0b <= next_qid;
						aqid_av_r1a <= next_qid;
						aqid_av_r1b <= next_qid;
						aqid_av_r2  <= next_qid;
						aqid_av_r3  <= next_qid;
`endif
						pa_hdr_age <= 2'd0;
						if (next_qid != 2'd0)
							last_non_q0 <= next_qid;
					end
				end
			end

			R_HDR: begin
				if (hdr_wait == 2'b10) begin
					hdr_wait <= 2'b01;
				end else if (hdr_wait == 2'b01) begin
					hdr_wait <= 2'b00;
				end else begin
					hdr_wait <= 2'b10;
					if (!hit_n_r || stale) begin
						ret_n <= 1'b0;
						stale <= doorbell[active_qid];
					end
				end
			end

			R_DEC: begin
			end

			R_DEC2: begin
				if (h_bad_words_r || h_bad_avail_r || h_novf_r) begin
					f_pulse <= 1'b1;
					f_kind  <= h_novf_r ? GHCI_FAULT_KIND_INVALID_OP : GHCI_FAULT_KIND_BAD_REG;
				end
			end

			R_N: begin
				if (!hit_n) begin
					ret_n <= 1'b1;
				end else if (n_ready) begin
					if (k_eq_n) begin
						;
					end
				end
			end

			R_V: begin
			end

			R_ADV: begin
				ret_pulse        <= 1'b1;
				if (queue_sema_wait[active_qid])
					queue_sema_wait[active_qid] <= 1'b0;
				if (preempt_pending && (active_qid != 2'd0)) begin
					if (ctx_base_zero) begin
						preempt_saved   <= 1'b1;
						preempt_pending <= 1'b0;
						preempt_active  <= 1'b1;
						preempt_pulse   <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
						active_qid      <= 2'd0;
						aqid_av_r0a     <= 2'd0;
						aqid_av_r0b     <= 2'd0;
						aqid_av_r1a     <= 2'd0;
						aqid_av_r1b     <= 2'd0;
						aqid_av_r2      <= 2'd0;
						aqid_av_r3      <= 2'd0;
`endif
					end
				end
			end

			R_REQ: begin
				if (ring_req_ready && demand_req) begin
					st_is_rsp_r <= 1'b1;
				end
			end

			R_RSP: begin
				if (rsp_is_tag0_r) begin
					st_is_rsp_r <= 1'b0;
					if (rsp_error_r) begin
						f_pulse <= 1'b1;
						f_kind  <= GHCI_FAULT_KIND_AXI_ERROR;
					end else begin
						hdr_wait <= 2'b00;
					end
				end
			end

			R_CTX_SAVE_REQ: begin
			end

			R_CTX_SAVE_RSP: begin
				if (rsp_is_tag2_r) begin
					preempt_saved   <= 1'b1;
					preempt_pending <= 1'b0;
					preempt_active  <= 1'b1;
					preempt_pulse   <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					active_qid      <= 2'd0;
					aqid_av_r0a     <= 2'd0;
					aqid_av_r0b     <= 2'd0;
					aqid_av_r1a     <= 2'd0;
					aqid_av_r1b     <= 2'd0;
					aqid_av_r2      <= 2'd0;
					aqid_av_r3      <= 2'd0;
`endif
				end
			end

			R_CTX_REST_REQ: begin
			end

			R_CTX_REST_RSP: begin
				if (rsp_is_tag3_r) begin
					preempt_active        <= 1'b0;
					preempt_saved         <= 1'b0;
					preempt_pulse         <= 1'b1;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
					active_qid            <= interrupted_qid;
					aqid_av_r0a           <= interrupted_qid;
					aqid_av_r0b           <= interrupted_qid;
					aqid_av_r1a           <= interrupted_qid;
					aqid_av_r1b           <= interrupted_qid;
					aqid_av_r2            <= interrupted_qid;
					aqid_av_r3            <= interrupted_qid;
`endif
				end
			end

			default: ;
			endcase

`ifdef SYNTH_KINTEX7
			if (aq_ce) begin
				active_qid  <= aq_d;
				aqid_av_r0a <= aq_d;
				aqid_av_r0b <= aq_d;
				aqid_av_r1a <= aq_d;
				aqid_av_r1b <= aq_d;
				aqid_av_r2  <= aq_d;
				aqid_av_r3  <= aq_d;
			end
			if (pa_we_adv_b) begin
				k_eq_n_r <= (k[15:8] == n_m1_r[15:8]) && (k[7:0] == n_m1_r[7:0]);
				if (want_word == 3'b111)
					nl_settle_r <= 3'b111;
			end
			if (pa_we_b0_lo) pa[7:0]  <= st_idle_sel_b[0] ? pa_hdr[7:0]  : pa_4[7:0];
			if (pa_we_b0_hi) pa[15:8] <= st_idle_sel_b[0] ? pa_hdr[15:8] : pa_4[15:8];
			if (pa_we_b[1]) pa[ADDR_W-1:16] <= st_idle_sel_b[1] ? pa_hdr[ADDR_W-1:16] : pa_4[ADDR_W-1:16];
			if (pa_we_w[0]) for (bi = 0;  bi < 6;  bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[1]) for (bi = 6;  bi < 12; bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[2]) for (bi = 12; bi < 16; bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[3]) for (bi = 16; bi < 22; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_w[4]) for (bi = 22; bi < 27; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_w[5]) for (bi = 27; bi < 32; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_i[0]) begin
				for (bi = 0; bi < 4; bi = bi + 1)
					pa_idx_r[bi] <= pa_nxt_lo[5:3];
				pa_widx_r[0] <= pa_nxt_lo[5:3];
			end
			if (pa_we_i[1]) begin
				for (bi = 4; bi < 8; bi = bi + 1)
					pa_idx_r[bi] <= pa_nxt_hi[5:3];
				pa_widx_r[1] <= pa_nxt_hi[5:3];
			end
			if (pf_line_we_b[0] && pf_start) begin
				pf_line[13:0] <= next_line_r[13:0];
				pf_idx_r      <= next_line_r[2:0];
			end
			if (pf_line_we_b[1] && pf_trigger_r1) pf_line[LA_W-1:14] <= next_line_r[LA_W-1:14];
`elsif SYNTH_KINTEX_ULTRASCALE
			if (aq_ce) begin
				active_qid  <= aq_d;
				aqid_av_r0a <= aq_d;
				aqid_av_r0b <= aq_d;
				aqid_av_r1a <= aq_d;
				aqid_av_r1b <= aq_d;
				aqid_av_r2  <= aq_d;
				aqid_av_r3  <= aq_d;
			end
			if (pa_we_adv_b) begin
				k_eq_n_r <= (k[15:8] == n_m1_r[15:8]) && (k[7:0] == n_m1_r[7:0]);
				if (want_word == 3'b111)
					nl_settle_r <= 3'b111;
			end
			if (pa_we_b0_lo) pa[7:0]  <= st_idle_sel_b[0] ? pa_hdr[7:0]  : pa_4[7:0];
			if (pa_we_b0_hi) pa[15:8] <= st_idle_sel_b[0] ? pa_hdr[15:8] : pa_4[15:8];
			if (pa_we_b[1]) pa[ADDR_W-1:16] <= st_idle_sel_b[1] ? pa_hdr[ADDR_W-1:16] : pa_4[ADDR_W-1:16];
			if (pa_we_w[0]) for (bi = 0;  bi < 6;  bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[1]) for (bi = 6;  bi < 12; bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[2]) for (bi = 12; bi < 16; bi = bi + 1) pa_word_r[bi] <= pa_nxt_lo[2:0];
			if (pa_we_w[3]) for (bi = 16; bi < 22; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_w[4]) for (bi = 22; bi < 27; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_w[5]) for (bi = 27; bi < 32; bi = bi + 1) pa_word_r[bi] <= pa_nxt_hi[2:0];
			if (pa_we_i[0]) begin
				for (bi = 0; bi < 4; bi = bi + 1)
					pa_idx_r[bi] <= pa_nxt_lo[5:3];
				pa_widx_r[0] <= pa_nxt_lo[5:3];
			end
			if (pa_we_i[1]) begin
				for (bi = 4; bi < 8; bi = bi + 1)
					pa_idx_r[bi] <= pa_nxt_hi[5:3];
				pa_widx_r[1] <= pa_nxt_hi[5:3];
			end
			if (pf_line_we_b[0] && pf_start) begin
				pf_line[13:0] <= next_line_r[13:0];
				pf_idx_r      <= next_line_r[2:0];
			end
			if (pf_line_we_b[1] && pf_trigger_r1) pf_line[LA_W-1:14] <= next_line_r[LA_W-1:14];
`else
			if (pa_we_adv) begin
				k        <= k_n;
				k_eq_n_r <= (k[15:8] == n_m1_r[15:8]) && (k[7:0] == n_m1_r[7:0]);
				if (&want_word)
					nl_settle_r <= 3'b111;
			end
			if (pa_we_hdr || pa_we_adv) begin
				pa <= pa_we_hdr ? pa_hdr : pa_4;
				for (bi = 0; bi < 4; bi = bi + 1) begin
					pa_word_r[bi] <= pa_we_hdr ? pa_hdr[LINE_SHIFT-1:2] : pa_4[LINE_SHIFT-1:2];
					pa_idx_r[bi]  <= pa_we_hdr ? pa_hdr[LINE_SHIFT +: 3] : pa_4[LINE_SHIFT +: 3];
				end
				pa_widx_r <= pa_we_hdr ? pa_hdr[LINE_SHIFT +: 3] : pa_4[LINE_SHIFT +: 3];
			end
			if ((pf_st == PF_IDLE) && pf_start) begin
				pf_line  <= next_line_r;
				pf_idx_r <= next_line_r[2:0];
			end
`endif
		end
	end

	wire [1:0] unused_ring_unused = 2'd0;

`ifdef SYNTH_KINTEX7
	wire [1:0] ring_lo_ok, rs_b0lo, rs_b0hi, rs_sz0, rs_wp0, rs_rp0;
	genvar g_rs;
	generate
		for (g_rs = 0; g_rs < 2; g_rs = g_rs + 1) begin : g_ring_rd_sel
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_rlok  (.I0(hcr_ridx[6]), .I1(hcr_ridx[7]), .I2(hcr_ridx[8]), .I3(hcr_ridx[9]), .O(ring_lo_ok[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_BASE0_LO[5:0])) u_rb0l (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_b0lo[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_BASE0_HI[5:0])) u_rb0h (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_b0hi[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_SIZE0[5:0]))    u_rsz0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_sz0[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_WPTR0[5:0]))    u_rwp0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_wp0[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_RPTR0[5:0]))    u_rrp0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_rp0[g_rs]));
		end
	endgenerate
	wire [31:0] rrd_p0 = {
		({16{rs_b0lo[1]}} & base[0][31:16]) | ({16{rs_b0hi[1]}} & base[0][63:48]) | ({16{rs_sz0[1]}} & size0_enc_r[31:16]),
		({16{rs_b0lo[0]}} & base[0][15:0])  | ({16{rs_b0hi[0]}} & base[0][47:32]) | ({16{rs_sz0[0]}} & size0_enc_r[15:0])
	};
	wire [31:0] rrd_p1 = {
		({16{rs_wp0[1]}} & {10'd0, wptr[0][21:16]}) | ({16{rs_rp0[1]}} & {10'd0, rptr[0][21:16]}),
		({16{rs_wp0[0]}} & wptr[0][15:0])           | ({16{rs_rp0[0]}} & rptr[0][15:0])
	};
	assign hcr_rdata = {
		ring_lo_ok[1] ? (rrd_p0[31:16] | rrd_p1[31:16]) : 16'd0,
		ring_lo_ok[0] ? (rrd_p0[15:0]  | rrd_p1[15:0])  : 16'd0
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] ring_lo_ok, rs_b0lo, rs_b0hi, rs_sz0, rs_wp0, rs_rp0;
	genvar g_rs;
	generate
		for (g_rs = 0; g_rs < 2; g_rs = g_rs + 1) begin : g_ring_rd_sel
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_rlok  (.I0(hcr_ridx[6]), .I1(hcr_ridx[7]), .I2(hcr_ridx[8]), .I3(hcr_ridx[9]), .O(ring_lo_ok[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_BASE0_LO[5:0])) u_rb0l (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_b0lo[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_BASE0_HI[5:0])) u_rb0h (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_b0hi[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_SIZE0[5:0]))    u_rsz0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_sz0[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_WPTR0[5:0]))    u_rwp0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_wp0[g_rs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_RPTR0[5:0]))    u_rrp0 (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(rs_rp0[g_rs]));
		end
	endgenerate
	wire [31:0] rrd_p0 = {
		({16{rs_b0lo[1]}} & base[0][31:16]) | ({16{rs_b0hi[1]}} & base[0][63:48]) | ({16{rs_sz0[1]}} & size0_enc_r[31:16]),
		({16{rs_b0lo[0]}} & base[0][15:0])  | ({16{rs_b0hi[0]}} & base[0][47:32]) | ({16{rs_sz0[0]}} & size0_enc_r[15:0])
	};
	wire [31:0] rrd_p1 = {
		({16{rs_wp0[1]}} & {10'd0, wptr[0][21:16]}) | ({16{rs_rp0[1]}} & {10'd0, rptr[0][21:16]}),
		({16{rs_wp0[0]}} & wptr[0][15:0])           | ({16{rs_rp0[0]}} & rptr[0][15:0])
	};
	assign hcr_rdata = {
		ring_lo_ok[1] ? (rrd_p0[31:16] | rrd_p1[31:16]) : 16'd0,
		ring_lo_ok[0] ? (rrd_p0[15:0]  | rrd_p1[15:0])  : 16'd0
	};
`else
	assign hcr_rdata =
		(hcr_ridx == I_BASE0_LO || hcr_ridx == I_BASE0) ? base[0][31:0] :
		(hcr_ridx == I_BASE0_HI) ? base[0][63:32] :
		(hcr_ridx == I_SIZE0) ? encode_ring_size(size[0]) :
		(hcr_ridx == I_WPTR0) ? {10'd0, wptr[0]} :
		(hcr_ridx == I_RPTR0) ? {10'd0, rptr[0]} : 32'd0;
`endif

	wire unused_ok = &{1'b0, rsp_write, rsp_tag[7:2], h_end[18], avail_bo, h_end_co, p_end_co, pa_4_co, want_word,
		pa_hdr_co, k_co, unused_nl_co, sema_co, unused_dr_co, unused_ds_co, dma_size, ghci_n_bo, rem_lo8_p1_co_unused,
		base_act, rem_words_bo, rem_words_bo_r, rem_words_s[18], hit, next_hit,
		h_bad_r, h_bad_words_r, h_bad_avail_r, h_bad,
		base[1], base[2], size[1], size[2], wptr[1], wptr[2], rptr[1], rptr[2],
		dma_base, dma_wptr, dma_rptr, dma_sem, dma_dst_base_r, dma_src_row_stride_r,
		dma_src_slice_stride_r, dma_copy_d_r, dma_copy_h_r, dma_copy_w_r,
		interrupted_qid, preempt_saved, preempt_pending, preempt_active, has_work,
		active_qid, ctx_base, queue_sema_wait, sema[1], sema[0],
		aqid_av_r0a, aqid_av_r0b, aqid_av_r1a, aqid_av_r1b, aqid_av_r2, aqid_av_r3};

endmodule

`default_nettype wire
