/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_blt_rsv_px: the pixel path of the blitter's RESOLVE (atg_blt_rsv,
 * docs/ghci.md section 8.2).  It takes row descriptors and the source
 * lines of those rows in order, averages the four samples of each pixel
 * (atg_blt_avg) and assembles the destination lines, which it hands back
 * to atg_blt_rsv as writes.
 *
 * A row descriptor (rd_*, valid/ready, a FIFO of two) is {off0, nlines,
 * w, x, drow}: the byte offset of the row's first source byte in its first
 * line, the number of source lines of the row, the row's pixels, its first
 * destination x, and the destination row's address (linear: base + y
 * stride; tiled: base + (y / 4) stride + (y % 4) 4 ps).  The source lines
 * (ln_push, ln_data) arrive in the order the descriptors name them; there
 * is no ready: atg_blt_rsv reads a line only when the FIFO (2^DEPTH_LOG2
 * lines, ln_count) has room for it.  Pixels are 2^psl bytes (1 to 16).
 *
 * Per row, the window holds up to two source lines (w0, w1) and the byte
 * pointer of the next sample group.  Each cycle one group goes through: n
 * consecutive destination pixels, n <= 4, n <= max(1, LINE_BYTES / (4 ps))
 * (the group's 4 n ps source bytes fit the two lines; a 16-byte pixel's
 * 64-byte aligned samples fill two 32-byte lines), n <= the row's rest,
 * inside one destination line (linear) or inside one tile row (tiled: 4 -
 * x % 4).  A line is loaded into the window only while the row still has
 * lines to load, so exactly nlines are taken per row; at the row's end the
 * window is emptied.  The group's average (atg_blt_avg the same cycle, or
 * atg_blt_favg for sRGB and the floats, while no other group goes) is
 * registered (stage A) and
 * merged into the assembly line (stage B), which becomes a write
 * (wr_*, valid/ready, held until taken) when a group of another line
 * comes, or when nothing more is queued.  Byte enables cover the written
 * pixels only; rows that share a line merge.
 *
 * stop (a level) empties everything at once, the FIFOs included, and
 * stops the writes; idle is 1 when nothing is queued or held.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_blt_rsv_px #(
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter DEPTH_LOG2 = 4,	/* source line FIFO: 2^DEPTH_LOG2 lines */
	parameter RD_W = 32 + 17 + 17 + 17 + 5	/* ADDR_W + 51 + LINE_SHIFT */
) (
	input wire clk,
	input wire rst_n,
	input wire stop,

	input wire [2:0] psl,	/* log2 of the pixel's bytes */
	input wire [2:0] fmt,
	input wire tiled,

	input wire rd_valid,
	output wire rd_ready,
	input wire [RD_W-1:0] rd_data,

	input wire ln_push,
	input wire [LINE_BYTES*8-1:0] ln_data,
	output wire [DEPTH_LOG2:0] ln_count,
	output wire ln_pop,

	output reg wr_valid,
	input wire wr_ready,
	output reg [ADDR_W-LINE_SHIFT-1:0] wr_line,
	output reg [LINE_BYTES-1:0] wr_be,
	output reg [LINE_BYTES*8-1:0] wr_data,

	output wire idle
);
	localparam LB = LINE_BYTES;
	localparam LBITS = LINE_BYTES * 8;
	localparam LS = LINE_SHIFT;
	localparam LA_W = ADDR_W - LINE_SHIFT;

	/* verilator lint_off WIDTHTRUNC */
	localparam [LS:0] LB_V = LB;
	/* verilator lint_on WIDTHTRUNC */

	wire frst_n = rst_n && !stop;

	/* The row descriptors. */
	wire rq_valid;
	wire rq_pop;
	wire [RD_W-1:0] rq_data;
	wire [1:0] rq_count;

	am_fifo #(.W(RD_W), .DEPTH_LOG2(1)) rq (
		.clk(clk),
		.rst_n(frst_n),
		.in_valid(rd_valid),
		.in_ready(rd_ready),
		.in_data(rd_data),
		.out_valid(rq_valid),
		.out_ready(rq_pop),
		.out_data(rq_data),
		.count(rq_count)
	);

	/* The source lines (1-cycle registered input stage isolates MC tree from lq.mem Check 4). */
	reg ln_push_r;
	reg [LBITS-1:0] ln_data_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			ln_push_r <= 1'b0;
		else if (stop)
			ln_push_r <= 1'b0;
		else
			ln_push_r <= ln_push;
	end
	always @(posedge clk) begin
		ln_data_r <= ln_data;
	end

	wire lq_valid;
	wire lq_pop;
	wire [LBITS-1:0] lq_data;
	wire lq_ready;
	wire [DEPTH_LOG2:0] lq_fifo_count;
	wire lncnt_co_unused;

	am_fifo #(.W(LBITS), .DEPTH_LOG2(DEPTH_LOG2)) lq (
		.clk(clk),
		.rst_n(frst_n),
		.in_valid(ln_push_r),
		.in_ready(lq_ready),
		.in_data(ln_data_r),
		.out_valid(lq_valid),
		.out_ready(lq_pop),
		.out_data(lq_data),
		.count(lq_fifo_count)
	);
	am_add #(.W(DEPTH_LOG2 + 1)) a_lncnt (
		.a(lq_fifo_count),
		.b({{DEPTH_LOG2{1'b0}}, ln_push_r}),
		.ci(1'b0),
		.s(ln_count),
		.co(lncnt_co_unused)
	);

	wire [ADDR_W-1:0] d_drow = rq_data[ADDR_W-1:0];
	wire [16:0] d_x = rq_data[ADDR_W +: 17];
	wire [16:0] d_w = rq_data[ADDR_W + 17 +: 17];
	wire [16:0] d_nl = rq_data[ADDR_W + 34 +: 17];
	wire [LS-1:0] d_off = rq_data[ADDR_W + 51 +: LS];

	/* The row in progress (1-cycle setup_v isolates rq.rp -> daddr_ls_init -> n_geom_r). */
	reg active;
	reg setup_v;
	reg setup_v2;
	reg [16:0] x;
	reg [16:0] rem;
	reg rem_has_high;
	reg rem_le8;
	reg [2:0] r_n;
	reg r_last_group;
	reg [5:0] line_px_m1_r;
	reg [5:0] px_in_line_r;
	reg [5:0] m1_in_line_r;
	reg       m1_ge8_r;
	reg [ADDR_W-1:0] drow;
	reg [16:0] lines_left;
	reg lines_left_nz;
	reg [LBITS-1:0] buf0;
	reg [LBITS-1:0] buf1;
	reg head;
	(* equivalent_register_removal = "no", keep = "true" *) reg [63:0] head_rep;
	localparam integer HSZ = LBITS / 64;
	wire [LBITS-1:0] w0, w1;
	genvar ghs;
	generate
		for (ghs = 0; ghs < 64; ghs = ghs + 1) begin : g_head_sl
			assign w0[ghs*HSZ +: HSZ] = head_rep[ghs] ? buf1[ghs*HSZ +: HSZ] : buf0[ghs*HSZ +: HSZ];
			assign w1[ghs*HSZ +: HSZ] = head_rep[ghs] ? buf0[ghs*HSZ +: HSZ] : buf1[ghs*HSZ +: HSZ];
		end
	endgenerate
	reg v0;
	reg v1;
	reg [LS-1:0] ptr;
	(* equivalent_register_removal = "no", keep = "true" *) reg [32*LS-1:0] ptr_rep;

	/* The group: n pixels at x. */
	wire [2:0] sh4;	/* log2 of 4 ps, modulo 8 as the 3-bit sum was */
	wire [LS:0] nfit;
	wire [ADDR_W-1:0] x_b, daddr;
	wire [2:0] room_t;
	wire [3:0] grp_co_unused;

	am_add #(.W(3)) a_sh4 (.a(psl), .b(3'd2), .ci(1'b0), .s(sh4), .co(grp_co_unused[0]));
	am_shift #(.W(LS + 1), .SW(3)) h_fit (.a(LB_V), .amt(sh4), .left(1'b0), .arith(1'b0), .y(nfit));
	wire [2:0] nmax = nfit == {(LS + 1){1'b0}} ? 3'd1 : nfit >= 4 ? 3'd4 : nfit[2:0];
	reg [2:0] sh4_r, nmax_r, psl_r, n_geom_r;
	/* The pixel's bytes in the row (direct 5-way shift on psl_r <= 4 + 16+16 carry-select add). */
	wire [18:0] x_t_in = tiled ? {x[16:2], 2'b00, x[1:0]} : {2'b00, x};
	wire [22:0] x_t_sh = (psl_r == 3'd0) ? {4'd0, x_t_in} :
		(psl_r == 3'd1) ? {3'd0, x_t_in, 1'b0} :
		(psl_r == 3'd2) ? {2'd0, x_t_in, 2'b00} :
		(psl_r == 3'd3) ? {1'b0, x_t_in, 3'b000} : {x_t_in, 4'b0000};
	assign x_b = {{(ADDR_W - 23){1'b0}}, x_t_sh};
	wire [15:0] da_lo;
	wire [ADDR_W-17:0] da_hi0, da_hi1;
	wire da_c0, da_c10, da_c11;
	am_add #(.W(16))        a_da_lo  (.a(drow[15:0]),        .b(x_b[15:0]),        .ci(1'b0), .s(da_lo),  .co(da_c0));
	am_add #(.W(ADDR_W-16)) a_da_hi0 (.a(drow[ADDR_W-1:16]), .b(x_b[ADDR_W-1:16]), .ci(1'b0), .s(da_hi0), .co(da_c10));
	am_add #(.W(ADDR_W-16)) a_da_hi1 (.a(drow[ADDR_W-1:16]), .b(x_b[ADDR_W-1:16]), .ci(1'b1), .s(da_hi1), .co(da_c11));
	assign daddr = {da_c0 ? da_hi1 : da_hi0, da_lo};
	assign grp_co_unused[1] = da_c0 ? da_c11 : da_c10;
	assign grp_co_unused[2] = 1'b0;
	am_sub #(.W(3)) s_rt (.a(3'd4), .b({1'b0, x[1:0]}), .bi(1'b0), .d(room_t), .bo(grp_co_unused[3]));
	wire [2:0] rem_limit = rem_has_high ? 3'd4 : rem[2:0];

	reg [LS-1:0] daddr_ls_lin_r;

	/* Setup-stage linear byte offset (computed from registered x, drow, psl_r during setup_v). */
	wire [LS-1:0] x_b_lin_setup = (psl_r == 3'd0) ? x[LS-1:0] :
		(psl_r == 3'd1) ? {x[LS-2:0], 1'b0} :
		(psl_r == 3'd2) ? {x[LS-3:0], 2'b00} :
		(psl_r == 3'd3) ? {x[LS-4:0], 3'b000} : {x[LS-5:0], 4'b0000};
	wire [LS-1:0] daddr_ls_init;
	wire dals_init_co;
	am_add #(.W(LS)) a_dals_init (.a(drow[LS-1:0]), .b(x_b_lin_setup), .ci(1'b0), .s(daddr_ls_init), .co(dals_init_co));

	wire [5:0] daddr_ls_init_pad = {{(6 - LS){1'b0}}, daddr_ls_init};
	wire [5:0] daddr_ls_init_px = (psl_r == 3'd0) ? daddr_ls_init_pad :
		(psl_r == 3'd1) ? {1'b0, daddr_ls_init_pad[5:1]} :
		(psl_r == 3'd2) ? {2'd0, daddr_ls_init_pad[5:2]} :
		(psl_r == 3'd3) ? {3'd0, daddr_ls_init_pad[5:3]} : {4'd0, daddr_ls_init_pad[5:4]};
	wire [5:0] line_px_m1_w = (LS >= 6) ? (
		(psl == 3'd0) ? 6'd63 : (psl == 3'd1) ? 6'd31 : (psl == 3'd2) ? 6'd15 : (psl == 3'd3) ? 6'd7 : 6'd3
	) : (
		(psl == 3'd0) ? 6'd31 : (psl == 3'd1) ? 6'd15 : (psl == 3'd2) ? 6'd7  : (psl == 3'd3) ? 6'd3 : 6'd1
	);

	function [2:0] sub4_2b;
		input [1:0] b;
		case (b)
		2'd0: sub4_2b = 3'd4;
		2'd1: sub4_2b = 3'd3;
		2'd2: sub4_2b = 3'd2;
		2'd3: sub4_2b = 3'd1;
		endcase
	endfunction

	/* Carry-free 6-bit increment by 1..4 (zero CARRY4). */
	function [5:0] fn_add6;
		input [5:0] a;
		input [2:0] b;
		reg [1:0] s0, s1, s2;
		reg c2, c4;
		begin
			s0[0] = a[0] ^ b[0];
			s0[1] = a[1] ^ b[1] ^ (a[0] & b[0]);
			c2    = (a[1] & b[1]) | ((a[1] ^ b[1]) & (a[0] & b[0]));
			s1[0] = a[2] ^ b[2] ^ c2;
			s1[1] = a[3] ^ ((a[2] & b[2]) | ((a[2] ^ b[2]) & c2));
			c4    = a[3] & ((a[2] & b[2]) | ((a[2] ^ b[2]) & c2));
			s2[0] = a[4] ^ c4;
			s2[1] = a[5] ^ (a[4] & c4);
			fn_add6 = {s2, s1, s0};
		end
	endfunction

	/* Carry-free 4-bit subtraction by 1..4 (zero CARRY4). */
	function [3:0] fn_sub4;
		input [3:0] a;
		input [2:0] b;
		reg [2:0] d_lo;
		reg bo1, bo2, bo3;
		begin
			d_lo[0] = a[0] ^ b[0];
			bo1     = ~a[0] & b[0];
			d_lo[1] = a[1] ^ b[1] ^ bo1;
			bo2     = (~a[1] & b[1]) | (~(a[1] ^ b[1]) & bo1);
			d_lo[2] = a[2] ^ b[2] ^ bo2;
			bo3     = (~a[2] & b[2]) | (~(a[2] ^ b[2]) & bo2);
			fn_sub4 = {a[3] ^ bo3, d_lo};
		end
	endfunction

	/* Carry-free 6-bit subtraction by 0..63 (for initial line room m1 = lpm1 - px). */
	function [5:0] fn_sub6;
		input [5:0] a;
		input [5:0] b;
		integer si;
		reg [5:0] d;
		reg bo;
		begin
			bo = 1'b0;
			for (si = 0; si < 6; si = si + 1) begin
				d[si] = a[si] ^ b[si] ^ bo;
				bo    = (~a[si] & b[si]) | (~(a[si] ^ b[si]) & bo);
			end
			fn_sub6 = d;
		end
	endfunction

	/* Carry-free 4+4 carry-select 8-bit add for ptr + gb_cur (0 CARRY4, 2 LUTs). */
	function [7:0] fn_add8_ptr;
		input [7:0] a;
		input [7:0] b;
		reg [3:0] s_lo, s_hi0, s_hi1;
		reg [4:0] c_lo, c_hi0, c_hi1;
		integer ai;
		begin
			c_lo[0] = 1'b0;
			c_hi0[0] = 1'b0;
			c_hi1[0] = 1'b1;
			for (ai = 0; ai < 4; ai = ai + 1) begin
				s_lo[ai]    = a[ai] ^ b[ai] ^ c_lo[ai];
				c_lo[ai+1]  = (a[ai] & b[ai]) | ((a[ai] ^ b[ai]) & c_lo[ai]);
				s_hi0[ai]   = a[4+ai] ^ b[4+ai] ^ c_hi0[ai];
				c_hi0[ai+1] = (a[4+ai] & b[4+ai]) | ((a[4+ai] ^ b[4+ai]) & c_hi0[ai]);
				s_hi1[ai]   = a[4+ai] ^ b[4+ai] ^ c_hi1[ai];
				c_hi1[ai+1] = (a[4+ai] & b[4+ai]) | ((a[4+ai] ^ b[4+ai]) & c_hi1[ai]);
			end
			fn_add8_ptr = {c_lo[4] ? s_hi1 : s_hi0, s_lo};
		end
	endfunction

	/* Fast step for m1_in_line (remaining pixels in line - 1) and room_l_next (<= 2 LUTs). */
	/* Parallel k=1..4 line-room / pointer-boundary check so r_n only enters final 1-LUT mux. */
	wire [3:0] need2_k_w;
	wire [3:0] shift_ge_k_w;
	wire [3:0] two_used_k_w;
	wire [2:0] room_l_k_w [1:4];
	wire [2:0] room_t_k_w [1:4];
	wire [2:0] n_geom_k_w [1:4];
	wire [2:0] r_n_next_k_w [1:4];
	wire [3:0] r_last_grp_k_w;
	wire [5:0] m1_next_k_w [1:4];
	wire [3:0] m1_ge8_next_k_w;

	genvar gk_p;
	generate
		for (gk_p = 1; gk_p <= 4; gk_p = gk_p + 1) begin : g_p_k
			wire [7:0] n2l_v = need2_lim_v[(gk_p-1)*8 +: 8];
			wire [7:0] twl_v = two_lim_v[(gk_p-1)*8 +: 8];
			wire [5:0] ptr_6 = {{(6-LS){1'b0}}, ptr};
			wire gt_lo = (ptr_6[2:0] >  n2l_v[2:0]);
			wire ge_lo = (ptr_6[2:0] >= n2l_v[2:0]);
			wire gt_hi = (ptr_6[5:3] >  n2l_v[5:3]);
			wire eq_hi = (ptr_6[5:3] == n2l_v[5:3]);
			assign need2_k_w[gk_p-1]    = need2_gt_lb_r[gk_p-1] || gt_hi || (eq_hi && gt_lo);
			assign shift_ge_k_w[gk_p-1] = need2_ge_lb_r[gk_p-1] || gt_hi || (eq_hi && ge_lo);

			wire tw_ge_lo = (ptr_6[2:0] >= twl_v[2:0]);
			wire tw_gt_hi = (ptr_6[5:3] >  twl_v[5:3]);
			wire tw_eq_hi = (ptr_6[5:3] == twl_v[5:3]);
			assign two_used_k_w[gk_p-1] = two_ge_2lb_r[gk_p-1] ||
				(!twl_v[7] && !twl_v[6] && (tw_gt_hi || (tw_eq_hi && tw_ge_lo)));

			/* Fast constant-k subtraction of m1_in_line_r (k = 1..4) with zero ripple across 6 bits. */
			wire [3:0] m1_sub_k = fn_sub4({1'b0, m1_in_line_r[2:0]}, gk_p[2:0]);
			wire       m1_bo3_k = m1_sub_k[3];
			wire [2:0] m1_hi_k  = {
				m1_in_line_r[5] ^ (m1_bo3_k & ~m1_in_line_r[3] & ~m1_in_line_r[4]),
				m1_in_line_r[4] ^ (m1_bo3_k & ~m1_in_line_r[3]),
				m1_in_line_r[3] ^ m1_bo3_k
			};
			wire       m1_wrap_k = !m1_ge8_r && m1_bo3_k;
			assign m1_next_k_w[gk_p]    = m1_wrap_k ? line_px_m1_r : {m1_hi_k, m1_sub_k[2:0]};
			assign m1_ge8_next_k_w[gk_p-1] = m1_wrap_k ? (|line_px_m1_r[5:3]) : (|m1_hi_k);

			assign room_l_k_w[gk_p] = m1_ge8_r ? 3'd4 :
				(m1_sub_k[3] ? (line_px_m1_r[1] ? 3'd4 : 3'd2) :
				((m1_sub_k[2] || (m1_sub_k[1:0] == 2'd3)) ? 3'd4 :
				 ((m1_sub_k[1:0] == 2'd2) ? 3'd3 :
				  (m1_sub_k[1:0] == 2'd1) ? 3'd2 : 3'd1)));
			assign room_t_k_w[gk_p] = sub4_2b(x_p_k[gk_p]);
			assign n_geom_k_w[gk_p] = tiled ? (room_t_k_w[gk_p] < nmax_r ? room_t_k_w[gk_p] : nmax_r) :
`ifndef MUT_RSV_SPLIT
				(room_l_k_w[gk_p] < nmax_r ? room_l_k_w[gk_p] : nmax_r);
`else
				nmax_r;
`endif
			wire [3:0] rem4_sub_k = fn_sub4(rem[3:0], gk_p[2:0]);
			assign r_n_next_k_w[gk_p] = (!rem_le8 || (rem4_sub_k >= {1'b0, n_geom_k_w[gk_p]})) ?
				n_geom_k_w[gk_p] : rem4_sub_k[2:0];
			assign r_last_grp_k_w[gk_p-1] = rem_le8 && (rem4_sub_k <= {1'b0, n_geom_k_w[gk_p]});
		end
	endgenerate

	wire [5:0] m1_next_w = (r_n == 3'd1) ? m1_next_k_w[1] : (r_n == 3'd2) ? m1_next_k_w[2] :
	                       (r_n == 3'd3) ? m1_next_k_w[3] : m1_next_k_w[4];
	wire m1_ge8_next_w   = (r_n == 3'd1) ? m1_ge8_next_k_w[0] : (r_n == 3'd2) ? m1_ge8_next_k_w[1] :
	                       (r_n == 3'd3) ? m1_ge8_next_k_w[2] : m1_ge8_next_k_w[3];
	wire [2:0] room_l_fast = (r_n == 3'd1) ? room_l_k_w[1] : (r_n == 3'd2) ? room_l_k_w[2] :
	                         (r_n == 3'd3) ? room_l_k_w[3] : room_l_k_w[4];

	reg [2:0] room_l_limit_r;
	wire [2:0] n = r_n;

	wire [6:0] gb_init_k [1:4];
	wire [7:0] need2_lim_w [1:4];
	wire [7:0] two_lim_w [1:4];
	wire [4:0] grp_nb_init_k [1:4];
	reg  [27:0] gb_r_v;
	reg  [31:0] need2_lim_v;
	reg  [31:0] two_lim_v;
	reg  [3:0]  need2_gt_lb_r;
	reg  [3:0]  need2_ge_lb_r;
	reg  [3:0]  two_ge_2lb_r;
	reg  [19:0] grp_nb_r_v;
	wire [6:0] gb_k [1:4];
	wire [4:0] grp_nb_k [1:4];
	wire [3:0] n2l_bo_unused, twl_bo_unused;
	wire [1:0] x_p_k [1:4];
	assign x_p_k[1] = (x[1:0] == 2'd0) ? 2'd1 : (x[1:0] == 2'd1) ? 2'd2 : (x[1:0] == 2'd2) ? 2'd3 : 2'd0;
	assign x_p_k[2] = {~x[1], x[0]};
	assign x_p_k[3] = (x[1:0] == 2'd0) ? 2'd3 : (x[1:0] == 2'd1) ? 2'd0 : (x[1:0] == 2'd2) ? 2'd1 : 2'd2;
	assign x_p_k[4] = x[1:0];

	/* verilator lint_off WIDTH */
	localparam [7:0] LB_8  = LB;
	localparam [7:0] LB2_8 = 2 * LB;
	/* verilator lint_on WIDTH */

	genvar gk;
	generate
		for (gk = 1; gk <= 4; gk = gk + 1) begin : g_spec_k
			am_shift #(.W(7), .SW(3)) h_gb_k (.a(gk[6:0]), .amt(sh4), .left(1'b1), .arith(1'b0), .y(gb_init_k[gk]));
			am_sub #(.W(8)) s_n2l (.a(LB_8),  .b({1'b0, gb_init_k[gk]}), .bi(1'b0), .d(need2_lim_w[gk]), .bo(n2l_bo_unused[gk-1]));
			am_sub #(.W(8)) s_twl (.a(LB2_8), .b({1'b0, gb_init_k[gk]}), .bi(1'b0), .d(two_lim_w[gk]),   .bo(twl_bo_unused[gk-1]));
			am_shift #(.W(5), .SW(3)) h_nb_k (.a(gk[4:0]), .amt(psl), .left(1'b1), .arith(1'b0), .y(grp_nb_init_k[gk]));

			assign gb_k[gk] = gb_r_v[(gk-1)*7 +: 7];
			assign grp_nb_k[gk] = grp_nb_r_v[(gk-1)*5 +: 5];
		end
	endgenerate

	/* Single-path pointer arithmetic using registered r_n; boundary flags use 2-LUT precomputed compares. */
	wire [6:0] gb_cur = (r_n == 3'd1) ? gb_k[1] : (r_n == 3'd2) ? gb_k[2] : (r_n == 3'd3) ? gb_k[3] : gb_k[4];
	wire [7:0] ptr_sum = fn_add8_ptr({{(8-LS){1'b0}}, ptr}, {1'b0, gb_cur});
	wire [LS-1:0] nptr_sel = ptr_sum[LS-1:0];
	wire need2    = (r_n == 3'd1) ? need2_k_w[0]    : (r_n == 3'd2) ? need2_k_w[1]    : (r_n == 3'd3) ? need2_k_w[2]    : need2_k_w[3];
	wire shift_ge = (r_n == 3'd1) ? shift_ge_k_w[0] : (r_n == 3'd2) ? shift_ge_k_w[1] : (r_n == 3'd3) ? shift_ge_k_w[2] : shift_ge_k_w[3];
	wire two_used = (r_n == 3'd1) ? two_used_k_w[0] : (r_n == 3'd2) ? two_used_k_w[1] : (r_n == 3'd3) ? two_used_k_w[2] : two_used_k_w[3];
	wire [1:0] gb_co_unused = 2'd0;

	/* Window extraction via 3-stage 4:1 byte selector (<= 3 LUT6 / ~6 cmos3, FO <= 16). */
	wire [LBITS-1:0] win_align_out;
	atg_blt_align #(
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.OFF_REPS(32)
	) u_win_align (
		.in_data({w1, w0}),
		.off(ptr),
		.off_rep(ptr_rep),
		.out_data(win_align_out)
	);
	wire [511:0] win_512;
	generate
		if (LBITS < 512) begin : g_win_pad
			wire [LBITS-1:0] win_align_hi;
			atg_blt_align #(
				.LINE_BYTES(LINE_BYTES),
				.LINE_SHIFT(LINE_SHIFT),
				.OFF_REPS(32)
			) u_win_align_hi (
				.in_data({{LBITS{1'b0}}, w1}),
				.off(ptr),
				.off_rep(ptr_rep),
				.out_data(win_align_hi)
			);
			assign win_512 = {win_align_hi, win_align_out};
		end else begin : g_win_full
			assign win_512 = win_align_out[511:0];
		end
	endgenerate
	wire [1023:0] win = {512'd0, win_512};

	wire [127:0] avg, favg;
	wire slow = fmt == 3'd3 || fmt == 3'd4 || fmt == 3'd5;	/* sRGB, the floats */
	reg a_pend;	/* a slow group is being averaged */
	wire f_valid, f_ready;

	/* Stage P: Pipeline register between window extraction and average computation */
	reg p_v;
	reg [511:0] p_win;
	reg [LA_W-1:0] p_line;
	reg [LS-1:0] p_off;
	reg [4:0] p_nb;
	(* equivalent_register_removal = "no", keep = "true" *) reg [2:0] p_fmt0, p_fmt1, p_fmt2, p_fmt3;
	(* equivalent_register_removal = "no", keep = "true" *) reg [2:0] p_psl0, p_psl1, p_psl2, p_psl3, p_psl_s0;
	reg p_is_s0;

	wire [191:0] avg_csa_s, avg_csa_c;
	wire [127:0] s0_val;

	atg_blt_avg_s1_r av_s1 (
		.fmt0(p_fmt0), .fmt1(p_fmt1), .fmt2(p_fmt2), .fmt3(p_fmt3),
		.psl0(p_psl0), .psl1(p_psl1), .psl2(p_psl2), .psl3(p_psl3), .psl_s0(p_psl_s0),
		.in_s(p_win),
		.csa_s_out(avg_csa_s),
		.csa_c_out(avg_csa_c),
		.s0_only(s0_val)
	);

	/* Stage P1: Pipeline register between CSA unpack and s2a/s2b boundary */
	reg p1_v;
	reg [LA_W-1:0] p1_line;
	reg [LS-1:0] p1_off;
	reg [4:0] p1_nb;
	reg [2:0] p1_fmt;
	reg [2:0] p1_psl;
	reg p1_is_s0;
	reg [191:0] p1_csa_s;
	reg [191:0] p1_csa_c;
	reg [127:0] p1_s0;

	wire [63:0]  avg_s2a_sum_lo;
	wire [15:0]  avg_s2a_c4_s;
	wire [127:0] avg_s2a_hi0, avg_s2a_hi1;

	atg_blt_avg_s2a av_s2a (
		.csa_s_in(p1_csa_s),
		.csa_c_in(p1_csa_c),
		.sum_lo_out(avg_s2a_sum_lo),
		.c4_s_out(avg_s2a_c4_s),
		.hi0_out(avg_s2a_hi0),
		.hi1_out(avg_s2a_hi1)
	);

	/* Stage P2: Pipeline register between s2a (lo+hi0/1) and s2b (rounding+pack) */
	reg p2_v;
	reg [LA_W-1:0] p2_line;
	reg [LS-1:0] p2_off;
	reg [4:0] p2_nb;
	reg [2:0] p2_fmt;
	reg [2:0] p2_psl;
	reg p2_is_s0;
	reg [63:0] p2_sum_lo;
	reg [15:0] p2_c4_s;
	reg [127:0] p2_hi0, p2_hi1;
	reg [127:0] p2_s0;

	atg_blt_avg_s2b av_s2b (
		.fmt(p2_fmt),
		.psl(p2_psl),
		.sum_lo_in(p2_sum_lo),
		.c4_s_in(p2_c4_s),
		.hi0_in(p2_hi0),
		.hi1_in(p2_hi1),
		.avg_only(avg)
	);

	/* Stage A: the average of the group (pre-computes ones mask), Stage A1: byte shift & merge into Stage B. */
	reg a_v;
	reg [LA_W-1:0] a_line;
	reg [LS-1:0] a_off;
	(* equivalent_register_removal = "no", keep = "true" *) reg [9*LS-1:0] a_off_rep;
	reg [LB-1:0] a_ones_r;
	reg [127:0] a_px_r;
	wire [127:0] a_px = a_px_r;

	reg a1_v;
	reg [LA_W-1:0] a1_line;
	reg [LBITS-1:0] a1_shf;
	reg [LB-1:0] a1_be;

	/* Stage B: the line being assembled. */
	reg b_v;
	reg [LA_W-1:0] b_line;
	reg [LBITS-1:0] b_d;
	reg [LB-1:0] b_be;

	wire wr_free = !wr_valid || wr_ready;
	wire a1_same = b_v && b_line == a1_line;
	wire a1_take = a1_v && (!b_v || a1_same || wr_free);
	wire a1_free = !a1_v || !b_v || wr_free;
	wire a_free = !a_v || a1_free;
	wire p2_free = !p2_v || a_free;
	wire p1_free = !p1_v || p2_free;
	wire p_free = !p_v || p1_free;
	wire pipe_all_v = p_v && p1_v && p2_v && a_v && a1_v && b_v;
	wire fire_base  = active && v0 && !a_pend &&
		(slow ? (!p_v && !p1_v && !p2_v && !a_v && (!a1_v || !b_v || wr_ready)) : (!pipe_all_v || wr_ready));
	wire fire_cond  = fire_base && (!need2 || v1);
	wire fire_ctrl  = fire_cond;

	reg fv_start;
	reg [2:0] fv_n;
	reg [511:0] fv_in_s;

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			fv_start <= 1'b0;
		end else if (stop) begin
			fv_start <= 1'b0;
		end else begin
			fv_start <= fire_ctrl && slow;
		end
	end

	always @(posedge clk) begin
		fv_n <= n;
		fv_in_s <= win_512;
	end

	atg_blt_favg fv (
		.clk(clk), .rst_n(frst_n), .i_valid(fv_start), .i_ready(f_ready), .fmt(fmt), .psl(psl),
		.npx(fv_n), .in_s(fv_in_s), .o_valid(f_valid), .o_ready(1'b1), .out_p(favg)
	);
	wire is_last_group = r_last_group;
	wire row_end = fire_ctrl && is_last_group;

	/* gend less one or two lines (the window's lines the group used up; only one is taken in a cycle). */
	wire shift_win = fire_ctrl && shift_ge && !two_used;
	wire drop_both = fire_ctrl && (two_used || row_end);

	/* The window after this cycle's group (before a load). */
	wire [LS-1:0] nptr = fire_ctrl ? nptr_sel : ptr;

	wire wr_buf0 = lq_pop && (head == v0);
	wire wr_buf1 = lq_pop && (head != v0);

	wire may_load = active && lines_left_nz;
	assign lq_pop = may_load && lq_valid && (!v0 || !v1);
	assign ln_pop = lq_valid && lq_pop;
	assign rq_pop = !active && !setup_v && !setup_v2 && !p_v && !a_pend && rq_valid;

	/* The merge: 8 half-words (16-bit am_shift W=88) + 64b-slot placement keeps a_off_rep FO <= 24. */
	localparam integer NSLOT = LB / 8;
	wire [LBITS-1:0] a_shf_w [0:7];
	genvar gsw, gss;
	generate
		for (gsw = 0; gsw < 8; gsw = gsw + 1) begin : g_shf_w
			wire [LS-1:0] w_off = a_off_rep[(gsw + 1)*LS +: LS];
			wire [87:0] al_w;
			am_shift #(.W(88), .SW(6), .DIR(2)) h_al_w (
				.a({72'd0, a_px[gsw*16 +: 16]}),
				.amt({w_off[2:0], 3'b000}),
				.left(1'b1), .arith(1'b0), .y(al_w)
			);
			for (gss = 0; gss < NSLOT; gss = gss + 1) begin : g_slot
				localparam integer SM1 = (gss > 0) ? (gss - 1) : 0;
				wire hit0 = (w_off[LS-1:3] == gss[LS-4:0]);
				wire hit1 = (gss > 0) && (w_off[LS-1:3] == SM1[LS-4:0]);
				assign a_shf_w[gsw][gss*64 +: 64] =
					(hit0 ? al_w[63:0] : 64'd0) |
					(hit1 ? {40'd0, al_w[87:64]} : 64'd0);
			end
		end
	endgenerate
	wire [LBITS-1:0] a_shf = a_shf_w[0] |
		{a_shf_w[1][LBITS-17:0],  16'd0} |
		{a_shf_w[2][LBITS-33:0],  32'd0} |
		{a_shf_w[3][LBITS-49:0],  48'd0} |
		{a_shf_w[4][LBITS-65:0],  64'd0} |
		{a_shf_w[5][LBITS-81:0],  80'd0} |
		{a_shf_w[6][LBITS-97:0],  96'd0} |
		{a_shf_w[7][LBITS-113:0], 112'd0};
	wire [LB-1:0] p2_ones_w, p_ones_w;
	wire [LB-1:0] a_be;

	genvar ob;
	generate
		for (ob = 0; ob < LB; ob = ob + 1) begin : g_ones
			if (ob < 31) begin : g_lt31
				assign p2_ones_w[ob] = (ob[4:0] < p2_nb);
				assign p_ones_w[ob]  = (ob[4:0] < p_nb);
			end else begin : g_ge31
				assign p2_ones_w[ob] = 1'b0;
				assign p_ones_w[ob]  = 1'b0;
			end
		end
	endgenerate
	am_shift #(.W(LB), .SW(LS)) h_be (.a(a_ones_r), .amt(a_off_rep[0*LS +: LS]), .left(1'b1), .arith(1'b0), .y(a_be));

	/* The row's counts: the lines left, the next pixel and the pixels left; a group's bytes. */
	wire [16:0] ll_dec, x_next, rem_next;
	wire [4:0] grp_nb = (n == 3'd1) ? grp_nb_k[1] : (n == 3'd2) ? grp_nb_k[2] : (n == 3'd3) ? grp_nb_k[3] : grp_nb_k[4];
	wire [2:0] row_co_unused;

	am_sub #(.W(17)) s_ll (.a(lines_left), .b(17'd0), .bi(1'b1), .d(ll_dec), .bo(row_co_unused[0]));
	am_add #(.W(17)) a_x (.a(x), .b({14'd0, n}), .ci(1'b0), .s(x_next), .co(row_co_unused[1]));
	am_sub #(.W(17)) s_rem (.a(rem), .b({14'd0, n}), .bi(1'b0), .d(rem_next), .bo(row_co_unused[2]));

	wire [2:0] room_l_next     = room_l_fast;
	wire [2:0] n_geom_next     = (r_n == 3'd1) ? n_geom_k_w[1] : (r_n == 3'd2) ? n_geom_k_w[2] :
	                             (r_n == 3'd3) ? n_geom_k_w[3] : n_geom_k_w[4];
	wire [2:0] r_n_next        = (r_n == 3'd1) ? r_n_next_k_w[1] : (r_n == 3'd2) ? r_n_next_k_w[2] :
	                             (r_n == 3'd3) ? r_n_next_k_w[3] : r_n_next_k_w[4];
	wire       r_last_grp_next = (r_n == 3'd1) ? r_last_grp_k_w[0] : (r_n == 3'd2) ? r_last_grp_k_w[1] :
	                             (r_n == 3'd3) ? r_last_grp_k_w[2] : r_last_grp_k_w[3];

	wire [5:0] m1_init_w   = fn_sub6(line_px_m1_r, px_in_line_r);
	wire [2:0] room_l_init = (|m1_init_w[5:2]) ? 3'd4 :
		((m1_init_w[1:0] == 2'd3) ? 3'd4 : (m1_init_w[1:0] == 2'd2) ? 3'd3 : (m1_init_w[1:0] == 2'd1) ? 3'd2 : 3'd1);
	wire [2:0] room_t_init = sub4_2b(x[1:0]);
	wire [2:0] n_geom_init = tiled ? (room_t_init < nmax_r ? room_t_init : nmax_r) :
`ifndef MUT_RSV_SPLIT
		(room_l_init < nmax_r ? room_l_init : nmax_r);
`else
		nmax_r;
`endif
	reg [LBITS-1:0] a1_sh;	/* only the group's bytes: the average's other pixels are 0 */
	integer i;

	always @(*) begin
		for (i = 0; i < LB; i = i + 1)
			a1_sh[8 * i +: 8] = a1_be[i] ? a1_shf[8 * i +: 8] : 8'd0;
	end

	wire idle_now = !active && !setup_v && !setup_v2 && !rq_valid && !p_v && !p1_v && !p2_v && !a_v && !a1_v && !a_pend;
	assign idle = idle_now && !b_v && !wr_valid;

	/* Non-reset datapath registers (zero rst_n/stop fanout on Check 5). */
	always @(posedge clk) begin
		if (!stop) begin
			if (wr_buf0)
				buf0 <= lq_data;
			if (wr_buf1)
				buf1 <= lq_data;
		end
		if (a1_take && b_v && !a1_same)
			wr_data <= b_d;
		else if (!a1_take && !a1_v && b_v && idle_now && wr_free)
			wr_data <= b_d;
		if (a1_take)
			b_d <= a1_same ? (b_d | a1_sh) : a1_sh;
		if (p_free)
			p_win <= win_512;
		if (p_v && p1_free) begin
			p1_csa_s <= avg_csa_s;
			p1_csa_c <= avg_csa_c;
			p1_s0 <= s0_val;
		end
		if (p1_v && p2_free) begin
			p2_sum_lo <= avg_s2a_sum_lo;
			p2_c4_s   <= avg_s2a_c4_s;
			p2_hi0    <= avg_s2a_hi0;
			p2_hi1    <= avg_s2a_hi1;
			p2_s0     <= p1_s0;
		end
		if (p2_v && a_free)
			a_px_r <= p2_is_s0 ? p2_s0 : avg;
		else if (a_pend && f_valid)
			a_px_r <= favg;
		if (a_v && a1_free) begin
			a1_shf <= a_shf;
			a1_be  <= a_be;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			active <= 1'b0;
			setup_v <= 1'b0;
			setup_v2 <= 1'b0;
			x <= 17'd0;
			rem <= 17'd0;
			rem_has_high <= 1'b0;
			rem_le8 <= 1'b0;
			r_n <= 3'd1;
			r_last_group <= 1'b0;
			line_px_m1_r <= 6'd0;
			px_in_line_r <= 6'd0;
			m1_in_line_r <= 6'd0;
			m1_ge8_r <= 1'b0;
			drow <= {ADDR_W{1'b0}};
			lines_left <= 17'd0;
			lines_left_nz <= 1'b0;
			daddr_ls_lin_r <= {LS{1'b0}};
			room_l_limit_r <= 3'd4;
			n_geom_r <= 3'd4;
			sh4_r <= 3'd2;
			nmax_r <= 3'd4;
			psl_r <= 3'd0;
			gb_r_v <= 28'd0;
			need2_lim_v <= 32'd0;
			two_lim_v <= 32'd0;
			need2_gt_lb_r <= 4'd0;
			need2_ge_lb_r <= 4'd0;
			two_ge_2lb_r <= 4'd0;
			grp_nb_r_v <= 20'd0;
			head <= 1'b0;
			head_rep <= 64'd0;
			v0 <= 1'b0;
			v1 <= 1'b0;
			ptr <= {LS{1'b0}};
			ptr_rep <= {(32*LS){1'b0}};
			p_v <= 1'b0;
			p_line <= {LA_W{1'b0}};
			p_off <= {LS{1'b0}};
			p_nb <= 5'd0;
			p_fmt0 <= 3'd0;
			p_fmt1 <= 3'd0;
			p_fmt2 <= 3'd0;
			p_fmt3 <= 3'd0;
			p_psl0 <= 3'd0;
			p_psl1 <= 3'd0;
			p_psl2 <= 3'd0;
			p_psl3 <= 3'd0;
			p_psl_s0 <= 3'd0;
			p_is_s0 <= 1'b0;
			p1_v <= 1'b0;
			p1_line <= {LA_W{1'b0}};
			p1_off <= {LS{1'b0}};
			p1_nb <= 5'd0;
			p1_fmt <= 3'd0;
			p1_psl <= 3'd0;
			p1_is_s0 <= 1'b0;
			p2_v <= 1'b0;
			p2_line <= {LA_W{1'b0}};
			p2_off <= {LS{1'b0}};
			p2_nb <= 5'd0;
			p2_fmt <= 3'd0;
			p2_psl <= 3'd0;
			p2_is_s0 <= 1'b0;
			a_v <= 1'b0;
			a_pend <= 1'b0;
			a_line <= {LA_W{1'b0}};
			a_off <= {LS{1'b0}};
			a_off_rep <= {(9*LS){1'b0}};
			a_ones_r <= {LB{1'b0}};
			a1_v <= 1'b0;
			a1_line <= {LA_W{1'b0}};
			b_v <= 1'b0;
			b_line <= {LA_W{1'b0}};
			b_be <= {LB{1'b0}};
			wr_valid <= 1'b0;
			wr_line <= {LA_W{1'b0}};
			wr_be <= {LB{1'b0}};
		end else if (stop) begin
			active <= 1'b0;
			setup_v <= 1'b0;
			setup_v2 <= 1'b0;
			x <= 17'd0;
			rem <= 17'd0;
			rem_has_high <= 1'b0;
			rem_le8 <= 1'b0;
			r_n <= 3'd1;
			r_last_group <= 1'b0;
			line_px_m1_r <= 6'd0;
			px_in_line_r <= 6'd0;
			m1_in_line_r <= 6'd0;
			m1_ge8_r <= 1'b0;
			drow <= {ADDR_W{1'b0}};
			lines_left <= 17'd0;
			lines_left_nz <= 1'b0;
			daddr_ls_lin_r <= {LS{1'b0}};
			room_l_limit_r <= 3'd4;
			n_geom_r <= 3'd4;
			sh4_r <= 3'd2;
			nmax_r <= 3'd4;
			psl_r <= 3'd0;
			gb_r_v <= 28'd0;
			need2_lim_v <= 32'd0;
			two_lim_v <= 32'd0;
			need2_gt_lb_r <= 4'd0;
			need2_ge_lb_r <= 4'd0;
			two_ge_2lb_r <= 4'd0;
			grp_nb_r_v <= 20'd0;
			head <= 1'b0;
			head_rep <= 64'd0;
			v0 <= 1'b0;
			v1 <= 1'b0;
			ptr <= {LS{1'b0}};
			ptr_rep <= {(32*LS){1'b0}};
			p_v <= 1'b0;
			p_line <= {LA_W{1'b0}};
			p_off <= {LS{1'b0}};
			p_nb <= 5'd0;
			p_fmt0 <= 3'd0;
			p_fmt1 <= 3'd0;
			p_fmt2 <= 3'd0;
			p_fmt3 <= 3'd0;
			p_psl0 <= 3'd0;
			p_psl1 <= 3'd0;
			p_psl2 <= 3'd0;
			p_psl3 <= 3'd0;
			p_psl_s0 <= 3'd0;
			p_is_s0 <= 1'b0;
			p1_v <= 1'b0;
			p1_line <= {LA_W{1'b0}};
			p1_off <= {LS{1'b0}};
			p1_nb <= 5'd0;
			p1_fmt <= 3'd0;
			p1_psl <= 3'd0;
			p1_is_s0 <= 1'b0;
			p2_v <= 1'b0;
			p2_line <= {LA_W{1'b0}};
			p2_off <= {LS{1'b0}};
			p2_nb <= 5'd0;
			p2_fmt <= 3'd0;
			p2_psl <= 3'd0;
			p2_is_s0 <= 1'b0;
			a_v <= 1'b0;
			a_pend <= 1'b0;
			a_line <= {LA_W{1'b0}};
			a_off <= {LS{1'b0}};
			a_off_rep <= {(9*LS){1'b0}};
			a_ones_r <= {LB{1'b0}};
			a1_v <= 1'b0;
			a1_line <= {LA_W{1'b0}};
			b_v <= 1'b0;
			b_line <= {LA_W{1'b0}};
			b_be <= {LB{1'b0}};
			wr_valid <= 1'b0;
			wr_line <= {LA_W{1'b0}};
			wr_be <= {LB{1'b0}};
		end else begin
			/* Setup & row control (mutually exclusive: active=0 during rq_pop/setup_v/setup_v2). */
			if (rq_pop) begin
				setup_v <= 1'b1;
				drow <= d_drow;
				sh4_r <= sh4;
				nmax_r <= nmax;
				psl_r <= psl;
				line_px_m1_r <= line_px_m1_w;
				gb_r_v <= {gb_init_k[4], gb_init_k[3], gb_init_k[2], gb_init_k[1]};
				need2_lim_v <= {need2_lim_w[4], need2_lim_w[3], need2_lim_w[2], need2_lim_w[1]};
				two_lim_v   <= {two_lim_w[4], two_lim_w[3], two_lim_w[2], two_lim_w[1]};
				need2_gt_lb_r <= {({1'b0, gb_init_k[4]} > LB_8), ({1'b0, gb_init_k[3]} > LB_8),
					({1'b0, gb_init_k[2]} > LB_8), ({1'b0, gb_init_k[1]} > LB_8)};
				need2_ge_lb_r <= {({1'b0, gb_init_k[4]} >= LB_8), ({1'b0, gb_init_k[3]} >= LB_8),
					({1'b0, gb_init_k[2]} >= LB_8), ({1'b0, gb_init_k[1]} >= LB_8)};
				two_ge_2lb_r  <= {({1'b0, gb_init_k[4]} >= LB2_8), ({1'b0, gb_init_k[3]} >= LB2_8),
					({1'b0, gb_init_k[2]} >= LB2_8), ({1'b0, gb_init_k[1]} >= LB2_8)};
				grp_nb_r_v <= {grp_nb_init_k[4], grp_nb_init_k[3], grp_nb_init_k[2], grp_nb_init_k[1]};
				p_fmt0 <= fmt;
				p_fmt1 <= fmt;
				p_fmt2 <= fmt;
				p_fmt3 <= fmt;
				p_psl0 <= psl;
				p_psl1 <= psl;
				p_psl2 <= psl;
				p_psl3 <= psl;
				p_psl_s0 <= psl;
				p_is_s0 <= (fmt != 3'd0 && fmt != 3'd1 && fmt != 3'd6 && fmt != 3'd7);
			end else if (setup_v) begin
				setup_v <= 1'b0;
			end

			if (setup_v) begin
				setup_v2 <= 1'b1;
				daddr_ls_lin_r <= daddr_ls_init;
				px_in_line_r <= daddr_ls_init_px;
			end else if (setup_v2) begin
				setup_v2 <= 1'b0;
			end

			if (setup_v2)
				active <= 1'b1;
			else if (fire_ctrl && row_end)
				active <= 1'b0;

			if (rq_pop) begin
				x <= d_x;
				rem <= d_w;
				rem_has_high <= |d_w[16:3];
				rem_le8 <= !(|d_w[16:4]) && (!d_w[3] || (d_w[2:0] == 3'd0));
				ptr <= d_off;
				ptr_rep <= {32{d_off}};
			end else if (fire_ctrl) begin
				x <= x_next;
				rem <= rem_next;
				rem_has_high <= |rem_next[16:3];
				rem_le8 <= !(|rem_next[16:4]) && (!rem_next[3] || (rem_next[2:0] == 3'd0));
				ptr <= nptr_sel;
				ptr_rep <= {32{nptr_sel}};
			end

			if (setup_v2) begin
				m1_in_line_r <= m1_init_w;
				m1_ge8_r <= |m1_init_w[5:3];
				room_l_limit_r <= room_l_init;
				n_geom_r <= n_geom_init;
				r_n <= (!(|rem[16:3]) && (rem[2:0] < n_geom_init)) ? rem[2:0] : n_geom_init;
				r_last_group <= !(|rem[16:3]) && (rem[2:0] <= n_geom_init);
			end else if (fire_ctrl) begin
				m1_in_line_r <= m1_next_w;
				m1_ge8_r <= m1_ge8_next_w;
				room_l_limit_r <= room_l_next;
				n_geom_r <= n_geom_next;
				r_n <= r_n_next;
				r_last_group <= r_last_grp_next;
			end

			if (rq_pop) begin
				lines_left <= d_nl;
				lines_left_nz <= (d_nl != 17'd0);
			end else if (lq_pop) begin
				lines_left <= ll_dec;
				lines_left_nz <= (ll_dec != 17'd0);
			end

			if (rq_pop) begin
				head <= 1'b0;
				head_rep <= 64'd0;
				v0 <= 1'b0;
				v1 <= 1'b0;
			end else if (drop_both) begin
				head <= 1'b0;
				head_rep <= 64'd0;
				v0 <= 1'b0;
				v1 <= 1'b0;
			end else if (shift_win) begin
				head <= ~head;
				head_rep <= {64{~head}};
				v0 <= v1 | lq_pop;
				v1 <= 1'b0;
			end else if (lq_pop) begin
				if (!v0)
					v0 <= 1'b1;
				else
					v1 <= 1'b1;
			end

			/* The write being handed over. */
			if (wr_valid && wr_ready)
				wr_valid <= 1'b0;

			/* Stage B. */
			if (a1_take) begin
				if (b_v && !a1_same) begin
					wr_valid <= 1'b1;
					wr_line <= b_line;
					wr_be <= b_be;
				end
				b_v <= 1'b1;
				b_line <= a1_line;
				if (a1_same)
					b_be <= b_be | a1_be;
				else
					b_be <= a1_be;
			end else if (!a1_v && b_v && idle_now && wr_free) begin
				wr_valid <= 1'b1;
				wr_line <= b_line;
				wr_be <= b_be;
				b_v <= 1'b0;
			end

			/* Stage P (capture daddr/grp_nb cleanly whenever Stage P is open and !a_pend). */
			if (p_free && !a_pend) begin
				p_line <= daddr[ADDR_W-1:LS];
				p_off <= daddr[LS-1:0];
				p_nb <= grp_nb;
			end
			if (fire_ctrl && !slow) begin
				p_v <= 1'b1;
			end else if (p1_free) begin
				p_v <= 1'b0;
			end

			/* Stage P1 -> P2 -> Stage A. */
			if (p_v && p1_free) begin
				p1_v <= 1'b1;
				p1_line <= p_line;
				p1_off <= p_off;
				p1_nb <= p_nb;
				p1_fmt <= p_fmt0;
				p1_psl <= p_psl0;
				p1_is_s0 <= p_is_s0;
			end else if (p2_free) begin
				p1_v <= 1'b0;
			end

			if (p1_v && p2_free) begin
				p2_v <= 1'b1;
				p2_line <= p1_line;
				p2_off <= p1_off;
				p2_nb <= p1_nb;
				p2_fmt <= p1_fmt;
				p2_psl <= p1_psl;
				p2_is_s0 <= p1_is_s0;
			end else if (a_free) begin
				p2_v <= 1'b0;
			end

			/* Stage A. */
			if (p2_v && a_free) begin
				a_v <= 1'b1;
				a_line <= p2_line;
				a_off <= p2_off;
				a_off_rep <= {9{p2_off}};
				a_ones_r <= p2_ones_w;
			end else if (a_pend && f_valid) begin
				a_v <= 1'b1;
				a_pend <= 1'b0;
				a_line <= p_line;
				a_off <= p_off;
				a_off_rep <= {9{p_off}};
				a_ones_r <= p_ones_w;
			end else if (a1_free) begin
				a_v <= 1'b0;
			end

			/* Stage A1. */
			if (a_v && a1_free) begin
				a1_v <= 1'b1;
				a1_line <= a_line;
			end else if (a1_take) begin
				a1_v <= 1'b0;
			end

			if (fire_ctrl && slow) begin
				a_pend <= 1'b1;
			end
		end
	end

`ifdef RSV_TRACE
	always @(posedge clk)
		if (fire_ctrl)
			$display("%m t=%0t group daddr %h n %0d ptr %0d v %b%b avg %h win0 %h", $time, daddr,
				n, ptr, v0, v1, avg[63:0], win[127:0]);
`endif

	wire unused_ok = &{1'b0, f_ready, lq_ready, rq_count, win[1023:512],
		a_shf_w[1][LBITS-1:LBITS-16], a_shf_w[2][LBITS-1:LBITS-32], a_shf_w[3][LBITS-1:LBITS-48],
		a_shf_w[4][LBITS-1:LBITS-64], a_shf_w[5][LBITS-1:LBITS-80], a_shf_w[6][LBITS-1:LBITS-96],
		a_shf_w[7][LBITS-1:LBITS-112],
		grp_co_unused, gb_co_unused, row_co_unused, dals_init_co, n2l_bo_unused, twl_bo_unused,
		lncnt_co_unused, need2_lim_v, two_lim_v, need2_gt_lb_r, need2_ge_lb_r, two_ge_2lb_r,
		rem_limit, room_t, room_l_limit_r, daddr_ls_lin_r,
		a_off, sh4_r, n_geom_r, rem_has_high, nptr};
endmodule

`default_nettype wire
