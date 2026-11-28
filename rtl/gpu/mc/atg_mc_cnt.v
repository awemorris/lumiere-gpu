/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_mc_cnt: the MC's request counters (spec/gpu/mc/atg_mc.md), from the
 * clients' handshakes (take: valid and ready of a client's AMB port) and
 * responses:
 *
 *   pend    requests taken by the leaves and not yet granted or refused
 *           (done) at the root: pend_zero
 *   chk     those of them with chk (done_chk): chk_idle
 *   unit    requests of UNIT_MASK's clients without their response yet:
 *           unit_idle
 *   ax      requests the root granted to the AXI master (ax_inc: walks
 *           included) without their response from it yet (ax_dec): ax_zero
 *
 * KEEP_MASK's clients (the DC, atg_mc's ROOT_KEEP) are not counted by pend
 * (take is masked and done_keep marks their done) and the caller leaves
 * them out of ax.  rst_n and flush clear pend, chk and unit; ax only rst_n
 * (it counts the AXI master, which flush does not empty).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_mc_cnt #(
	parameter NCLIENT = 2,
	parameter [NCLIENT-1:0] UNIT_MASK = {NCLIENT{1'b0}},
	parameter [NCLIENT-1:0] KEEP_MASK = {NCLIENT{1'b0}}
) (
	input wire clk,
	input wire rst_n,
	input wire flush,
	input wire [NCLIENT-1:0] take,
	input wire [NCLIENT-1:0] take_chk,
	input wire [NCLIENT-1:0] rsp,
	input wire done,
	input wire done_chk,
	input wire done_keep,	/* done was a KEEP_MASK client's */
	input wire ax_inc,
	input wire ax_dec,
	output wire pend_zero,
	output wire chk_idle,
	output wire unit_idle,
	output wire ax_zero
);
	reg [7:0] pend, chk_n;
	reg [9:0] unit_n;

	/*
	 * The net deltas of this cycle's takes and ends/responses: a chain of
	 * one adder per client for each, folding the decrement into the chain
	 * so each counter updates through a single final adder.
	 */
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire cnt_rst_sync;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_cnt_rst (
		.I0(rst_n), .O(cnt_rst_sync)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire cnt_rst_sync;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_cnt_rst (
		.I0(rst_n), .O(cnt_rst_sync)
	);
`endif

	/*
	 * Pre-compute 8-bit partial population counts and nibble-aggregate vectors
	 * before the stage-1 registers so stage-0 stays <=10 cmos3 even with
	 * leaf_cross.data_phase, and stage-2 merges via 4:2 CSA + Kogge-Stone <=11 cmos3.
	 */
	reg [15:0] tm_in, tcm_in, tu_in, ru_in;
	always @(*) begin
		tm_in  = 16'd0;
		tcm_in = 16'd0;
		tu_in  = 16'd0;
		ru_in  = 16'd0;
		tm_in[NCLIENT-1:0]  = take & ~KEEP_MASK;
		tcm_in[NCLIENT-1:0] = take & take_chk;
		tu_in[NCLIENT-1:0]  = take & UNIT_MASK;
		ru_in[NCLIENT-1:0]  = rsp & UNIT_MASK;
	end

	/* Fast 3:2 full adder and 2-layer 10-bit compressor (<= 6-7 nonINV gates). */
	function [1:0] fn_fa;
		input a, b, c;
		begin
			fn_fa[0] = a ^ b ^ c;
			fn_fa[1] = (a & b) | (b & c) | (a & c);
		end
	endfunction

	/* Compress 16 bits into (s, c, b0) in <= 6-7 nonINV gates.
	 * For NCLIENT<=10 (atg_mc), upper 6 bits are constant 0 -> collapses to 2 FAs! */
	function [7:0] fn_comp16;
		input [15:0] v;
		reg [1:0] p0, p1, p2, p3, p4, l1a, l1b, l2a, l2b;
		begin
			if (NCLIENT <= 10) begin
				p0 = fn_fa(v[1], v[2], v[3]);
				p1 = fn_fa(v[4], v[5], v[6]);
				p2 = fn_fa(v[7], v[8], v[9]);
				l1a = fn_fa(p0[0], p1[0], p2[0]); /* bit0=s0(w1), bit1=k1(w2) */
				l1b = fn_fa(p0[1], p1[1], p2[1]); /* bit0=s1(w2), bit1=k2(w4) */
				/* Pack: {1'b0, b0, k2(w4), k1(w2), 1'b0, 1'b0, s1(w2), s0(w1)} */
				fn_comp16 = {1'b0, v[0], l1b[1], l1a[1], 1'b0, 1'b0, l1b[0], l1a[0]};
			end else begin
				p0 = fn_fa(v[1],  v[2],  v[3]);
				p1 = fn_fa(v[4],  v[5],  v[6]);
				p2 = fn_fa(v[7],  v[8],  v[9]);
				p3 = fn_fa(v[10], v[11], v[12]);
				p4 = fn_fa(v[13], v[14], v[15]);
				l1a = fn_fa(p0[0], p1[0], p2[0]);
				l1b = fn_fa(p3[0], p4[0], 1'b0);
				l2a = fn_fa(p0[1], p1[1], p2[1]);
				l2b = fn_fa(p3[1], p4[1], l1a[1]);
				/* {v[0](w1), l2b[1](w4), l2a[1](w4), l2b[0](w2), l1b[1](w2), l2a[0](w2), l1b[0](w1), l1a[0](w1)} */
				fn_comp16 = {v[0], l2b[1], l2a[1], l2b[0], l1b[1], l2a[0], l1b[0], l1a[0]};
			end
		end
	endfunction

	/* Add s[2:0] + c[2:0] + b0 (max 15) in 4-bit Kogge-Stone (zero + operators). */
	function [3:0] fn_ks4;
		input [3:0] a_in;
		input [3:0] b_in;
		reg [3:0] g0, p0, g1, g2;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g1 = g0 | (p0 & {g0[2:0], 1'b0});
			g2 = g1 | (p0 & {p0[2:0], 1'b0} & {g1[1:0], 2'b0});
			fn_ks4 = p0 ^ {g2[2:0], 1'b0};
		end
	endfunction

	function [3:0] fn_uncomp10;
		input [7:0] cp;
		reg [3:0] sa;
		begin
			if (NCLIENT <= 10) begin
				/* s={0,s1,s0}, c={k2,k1,b0} */
				fn_uncomp10 = fn_ks4({1'b0, cp[2:0]}, {1'b0, cp[5:4], cp[6]});
			end else begin
				sa = fn_ks4({1'b0, cp[5], cp[2], cp[0]}, {1'b0, cp[6], cp[3], cp[1]});
				fn_uncomp10 = fn_ks4(sa, {2'd0, cp[4], cp[7]});
			end
		end
	endfunction

	function [2:0] fn_pop4;
		input [3:0] v;
		reg s0, s1, c0, c1, c2;
		begin
			s0 = v[0] ^ v[1]; c0 = v[0] & v[1];
			s1 = v[2] ^ v[3]; c1 = v[2] & v[3];
			c2 = c0 & c1;
			fn_pop4[0] = s0 ^ s1;
			fn_pop4[2] = c2;
			fn_pop4[1] = ~c2 & (c0 | c1 | (s0 & s1));
		end
	endfunction

	function [4:0] fn_ks5;
		input [4:0] a_in;
		input [4:0] b_in;
		reg [4:0] g0, p0, g1, p1, g2, g3;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g1 = g0 | (p0 & {g0[3:0], 1'b0});
			p1 = p0 & {p0[3:0], 1'b0};
			g2 = g1 | (p1 & {g1[2:0], 2'b0});
			g3 = g2 | (p1 & {p1[2:0], 2'b0} & {g2[0], 4'b0});
			fn_ks5 = p0 ^ {g3[3:0], 1'b0};
		end
	endfunction

	function [4:0] fn_pop16_fast;
		input [15:0] v;
		reg [2:0] q0, q1, q2, q3;
		reg [3:0] s01, s23;
		begin
			q0 = fn_pop4(v[3:0]);   q1 = fn_pop4(v[7:4]);
			q2 = fn_pop4(v[11:8]);  q3 = fn_pop4(v[15:12]);
			s01 = fn_ks4({1'b0, q0}, {1'b0, q1});
			s23 = fn_ks4({1'b0, q2}, {1'b0, q3});
			fn_pop16_fast = fn_ks5({1'b0, s01}, {1'b0, s23});
		end
	endfunction

	wire [7:0] tk_cp_c = fn_comp16(tm_in);
	wire [7:0] ck_cp_c = fn_comp16(tcm_in);
	wire [7:0] ut_cp_c = fn_comp16(tu_in);
	wire [4:0] ur_pop_c = fn_pop16_fast(ru_in);

	function [1:0] nib_ag;
		input [3:0] n;
		begin
			nib_ag[1] = |n;
			nib_ag[0] = (n[0] & n[1]) | (n[2] & n[3]) | ((n[0] | n[1]) & (n[2] | n[3]));
		end
	endfunction

	/* Split fn_z1 across stage-1 FF: nibble aggregates (3 gates) in stage-0, combine (3 gates) in stage-2. */
	function [7:0] fn_nib4;
		input [15:0] v;
		begin
			fn_nib4 = {nib_ag(v[15:12]), nib_ag(v[11:8]), nib_ag(v[7:4]), nib_ag(v[3:0])};
		end
	endfunction

	function [1:0] fn_z1_from_nib;
		input [7:0] g;
		reg [3:0] na, ng;
		reg any_b, gt1_b;
		begin
			na = {g[7], g[5], g[3], g[1]};
			ng = {g[6], g[4], g[2], g[0]};
			any_b = |na;
			gt1_b = (|ng) | (na[0] & na[1]) | (na[2] & na[3]) | ((na[0] | na[1]) & (na[2] | na[3]));
			fn_z1_from_nib = {~any_b, any_b & ~gt1_b};
		end
	endfunction

	/* 8-bit Kogge-Stone adder (a + b) in 6 nonINV gates, zero + operators. */
	function [7:0] fn_ks8;
		input [7:0] a_in;
		input [7:0] b_in;
		reg [7:0] g0, p0, g1, p1, g2, p2, g3;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g1 = g0 | (p0 & {g0[6:0], 1'b0});
			p1 = p0 & {p0[6:0], 1'b0};
			g2 = g1 | (p1 & {g1[5:0], 2'b0});
			p2 = p1 & {p1[5:0], 2'b0};
			g3 = g2 | (p2 & {g2[3:0], 4'b0});
			fn_ks8 = p0 ^ {g3[6:0], 1'b0};
		end
	endfunction

	/* 10-bit Kogge-Stone adder (a + b + ci) in 7 nonINV gates, zero + operators. */
	function [9:0] fn_ks10;
		input [9:0] a_in;
		input [9:0] b_in;
		input ci_in;
		reg [9:0] g0, p0, g1, p1, g2, p2, g3, p3, g4;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g0[0] = g0[0] | (p0[0] & ci_in);
			g1 = g0 | (p0 & {g0[8:0], 1'b0});
			p1 = p0 & {p0[8:0], 1'b0};
			g2 = g1 | (p1 & {g1[7:0], 2'b0});
			p2 = p1 & {p1[7:0], 2'b0};
			g3 = g2 | (p2 & {g2[5:0], 4'b0});
			p3 = p2 & {p2[5:0], 4'b0};
			g4 = g3 | (p3 & {g3[1:0], 8'b0});
			fn_ks10 = p0 ^ {g4[8:0], ci_in};
		end
	endfunction

	/* Parallel (x - 2) in 4 cmos3 gates: bit 0 unchanged, bits [W-1:1] decremented by 1. */
	function [7:0] fn_dec8_m2;
		input [7:0] x;
		reg [6:0] hi, z, s1, s2, s3;
		begin
			hi = x[7:1];
			z  = ~hi;
			s1 = z  & {z[5:0],  1'b1};
			s2 = s1 & {s1[4:0], 2'b11};
			s3 = s2 & {s2[2:0], 4'hF};
			fn_dec8_m2 = {hi ^ {s3[5:0], 1'b1}, x[0]};
		end
	endfunction

	function [9:0] fn_dec10_m2;
		input [9:0] x;
		reg [8:0] hi, z, s1, s2, s3, s4;
		begin
			hi = x[9:1];
			z  = ~hi;
			s1 = z  & {z[7:0],  1'b1};
			s2 = s1 & {s1[6:0], 2'b11};
			s3 = s2 & {s2[4:0], 4'hF};
			s4 = s3 & {s3[0],   8'hFF};
			fn_dec10_m2 = {hi ^ {s4[7:0], 1'b1}, x[0]};
		end
	endfunction

	wire [7:0] tm_nib_c  = fn_nib4(tm_in);
	wire [7:0] tcm_nib_c = fn_nib4(tcm_in);
	wire [7:0] tu_nib_c  = fn_nib4(tu_in);
	wire ru_any_c = |ru_in;

	/* Stage 1: strictly 1-cycle registered compressed counts + activity indicators */
	reg [7:0] tk_cp_r, ck_cp_r, ut_cp_r;
	reg [4:0] n_unit_rsp;
	reg [7:0] tm_nib_r, tcm_nib_r, tu_nib_r;
	reg ru_any_r;
	(* max_fanout = 16 *) reg done_s;
	reg done_chk_s, done_keep_s, ax_inc_s, ax_dec_s;

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else if (flush) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else begin
			tk_cp_r   <= tk_cp_c;  ck_cp_r   <= ck_cp_c;  ut_cp_r   <= ut_cp_c; n_unit_rsp <= ur_pop_c;
			tm_nib_r  <= tm_nib_c; tcm_nib_r <= tcm_nib_c; tu_nib_r <= tu_nib_c;
			ru_any_r  <= ru_any_c;
			done_s    <= done;     done_chk_s <= done_chk; done_keep_s <= done_keep;
		end
	end

	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			ax_inc_s <= 1'b0;
			ax_dec_s <= 1'b0;
		end else begin
			ax_inc_s <= ax_inc;
			ax_dec_s <= ax_dec;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else if (flush) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else begin
			tk_cp_r   <= tk_cp_c;  ck_cp_r   <= ck_cp_c;  ut_cp_r   <= ut_cp_c; n_unit_rsp <= ur_pop_c;
			tm_nib_r  <= tm_nib_c; tcm_nib_r <= tcm_nib_c; tu_nib_r <= tu_nib_c;
			ru_any_r  <= ru_any_c;
			done_s    <= done;     done_chk_s <= done_chk; done_keep_s <= done_keep;
		end
	end

	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			ax_inc_s <= 1'b0;
			ax_dec_s <= 1'b0;
		end else begin
			ax_inc_s <= ax_inc;
			ax_dec_s <= ax_dec;
		end
	end
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else if (flush) begin
			tk_cp_r   <= 8'd0;  ck_cp_r   <= 8'd0;  ut_cp_r   <= 8'd0; n_unit_rsp <= 5'd0;
			tm_nib_r  <= 8'd0;  tcm_nib_r <= 8'd0;  tu_nib_r  <= 8'd0;
			ru_any_r  <= 1'b0;
			done_s    <= 1'b0;  done_chk_s <= 1'b0; done_keep_s <= 1'b0;
		end else begin
			tk_cp_r   <= tk_cp_c;  ck_cp_r   <= ck_cp_c;  ut_cp_r   <= ut_cp_c; n_unit_rsp <= ur_pop_c;
			tm_nib_r  <= tm_nib_c; tcm_nib_r <= tcm_nib_c; tu_nib_r <= tu_nib_c;
			ru_any_r  <= ru_any_c;
			done_s    <= done;     done_chk_s <= done_chk; done_keep_s <= done_keep;
		end
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			ax_inc_s <= 1'b0;
			ax_dec_s <= 1'b0;
		end else begin
			ax_inc_s <= ax_inc;
			ax_dec_s <= ax_dec;
		end
	end
`endif

	/* Stage 2: fast uncompress + parallel decrement/select (<= 11-12 cmos3). */
	function [7:0] fn_dec8_cond;
		input [7:0] x;
		input dec_en;
		reg [7:0] z, s1, s2, s3;
		begin
			z  = ~x;
			s1 = z  & {z[6:0],  1'b1};
			s2 = s1 & {s1[5:0], 2'b11};
			s3 = s2 & {s2[3:0], 4'hF};
			fn_dec8_cond = x ^ ({s3[6:0], 1'b1} & {8{dec_en}});
		end
	endfunction

	function [5:0] fn_ks6_ci;
		input [5:0] a_in;
		input [5:0] b_in;
		input ci_in;
		reg [5:0] g0, p0, g1, p1, g2, p2, g3;
		begin
			p0 = a_in ^ b_in;
			g0 = a_in & b_in;
			g0[0] = g0[0] | (p0[0] & ci_in);
			g1 = g0 | (p0 & {g0[4:0], 1'b0});
			p1 = p0 & {p0[4:0], 1'b0};
			g2 = g1 | (p1 & {g1[3:0], 2'b0});
			p2 = p1 & {p1[3:0], 2'b0};
			g3 = g2 | (p2 & {g2[1:0], 4'b0});
			fn_ks6_ci = p0 ^ {g3[4:0], ci_in};
		end
	endfunction

	function [5:0] fn_inc6;
		input [5:0] x;
		reg [5:0] s1, s2, s3;
		begin
			s1 = x  & {x[4:0],  1'b1};
			s2 = s1 & {s1[3:0], 2'b11};
			s3 = s2 & {s2[1:0], 4'hF};
			fn_inc6 = x ^ {s3[4:0], 1'b1};
		end
	endfunction

	function [5:0] fn_dec6;
		input [5:0] x;
		reg [5:0] z, s1, s2, s3;
		begin
			z  = ~x;
			s1 = z  & {z[4:0],  1'b1};
			s2 = s1 & {s1[3:0], 2'b11};
			s3 = s2 & {s2[1:0], 4'hF};
			fn_dec6 = x ^ {s3[4:0], 1'b1};
		end
	endfunction

	wire dec_t = done_s && !done_keep_s;

	wire [3:0] tk_val = fn_uncomp10(tk_cp_r);
	wire [3:0] ck_val = fn_uncomp10(ck_cp_r);
	wire [7:0] pend_eff = fn_dec8_cond(pend, dec_t);
	wire [7:0] chk_eff  = fn_dec8_cond(chk_n, done_chk_s);
	wire [7:0] pend_n   = fn_ks8(pend_eff, {4'd0, tk_val});
	wire [7:0] chk_nx   = fn_ks8(chk_eff,  {4'd0, ck_val});

	/* unit_nx: low 6b CSA+KS computes [3:0] + carry/borrow {-1,0,+1} into precomputed [9:4] */
	wire [5:0] ut_s_w = (NCLIENT <= 10) ? {4'd0, ut_cp_r[1:0]} : 6'd0;
	wire [5:0] ut_c_w = (NCLIENT <= 10) ? {3'd0, ut_cp_r[5:4], ut_cp_r[6]} : {2'd0, fn_uncomp10(ut_cp_r)};
	wire [5:0] ur_n_w = ~{1'b0, n_unit_rsp};
	wire [5:0] u_a_s  = {2'b00, unit_n[3:0]} ^ ut_s_w ^ ut_c_w;
	wire [5:0] u_a_c  = ({2'b00, unit_n[3:0]} & ut_s_w) | ({2'b00, unit_n[3:0]} & ut_c_w) | (ut_s_w & ut_c_w);
	wire [5:0] u_b_s  = u_a_s ^ {u_a_c[4:0], 1'b0} ^ ur_n_w;
	wire [5:0] u_b_c  = (u_a_s & {u_a_c[4:0], 1'b0}) | (u_a_s & ur_n_w) | ({u_a_c[4:0], 1'b0} & ur_n_w);
	wire [5:0] u_lo_s = fn_ks6_ci(u_b_s, {u_b_c[4:0], 1'b1}, 1'b0);
	wire [5:0] u_hi_p0 = unit_n[9:4];
	wire [5:0] u_hi_p1 = fn_inc6(unit_n[9:4]);
	wire [5:0] u_hi_m1 = fn_dec6(unit_n[9:4]);
	wire [5:0] u_hi_s  = u_lo_s[5] ? u_hi_m1 : (u_lo_s[4] ? u_hi_p1 : u_hi_p0);
	wire [9:0] unit_nx = {u_hi_s, u_lo_s[3:0]};

	/* Wrap-exact O(log W) is1 checks: run in parallel with pend_n/chk_nx/unit_nx. */
	wire [7:0] pend_m2 = fn_ks8(pend, 8'hFE);
	wire [7:0] pend_m3 = fn_ks8(pend, 8'hFD);
	wire [7:0] pend_m2_eff = dec_t ? pend_m3 : pend_m2;
	wire [7:0] chk_m2  = fn_ks8(chk_n, 8'hFE);
	wire [7:0] chk_m3  = fn_ks8(chk_n, 8'hFD);
	wire [7:0] chk_m2_eff = done_chk_s ? chk_m3 : chk_m2;

	wire [9:0] unit_m2 = fn_ks10(unit_n, 10'h3FE, 1'b0);
	wire [9:0] ut_w = {6'd0, fn_uncomp10(ut_cp_r)};
	wire [9:0] ur_w = {5'h1F, ~n_unit_rsp};
	wire [9:0] u1s = unit_m2 ^ ut_w ^ ur_w;
	wire [9:0] u1c = (unit_m2 & ut_w) | (unit_m2 & ur_w) | (ut_w & ur_w);

	reg pend_is0_r, pend_is1_r;
	reg chk_is0_r, chk_is1_r;
	reg unit_is0_r, unit_is1_r;
	reg ax_is0_r, ax_is1_r;

	wire [1:0] tm_z1_r  = fn_z1_from_nib(tm_nib_r);
	wire [1:0] tcm_z1_r = fn_z1_from_nib(tcm_nib_r);
	wire [1:0] tu_z1_r  = fn_z1_from_nib(tu_nib_r);

	wire pend_zero_c = dec_t ? ((pend_is1_r && tm_z1_r[1]) || (pend_is0_r && tm_z1_r[0]))
	                         : (pend_is0_r && tm_z1_r[1]);
	wire chk_idle_c  = done_chk_s ? ((chk_is1_r && tcm_z1_r[1]) || (chk_is0_r && tcm_z1_r[0]))
	                              : (chk_is0_r && tcm_z1_r[1]);
	wire unit_idle_c = ru_any_r ? ((unit_is1_r && tu_z1_r[1]) || (unit_is0_r && tu_z1_r[0]))
	                            : (unit_is0_r && tu_z1_r[1]);
	wire ax_zero_c   = (ax_inc_s == ax_dec_s) ? ax_is0_r : (ax_inc_s ? 1'b0 : ax_is1_r);

	/* (A + B + 1 == 0 mod 2^W) <=> &(A ^ B). */
	wire pend_is0_c = pend_zero_c;
	wire chk_is0_c  = chk_idle_c;
	wire pend_is1_c = (&pend_m2_eff[7:4]) && &(pend_m2_eff[3:0] ^ tk_val);
	wire chk_is1_c  = (&chk_m2_eff[7:4])  && &(chk_m2_eff[3:0]  ^ ck_val);
	wire unit_is0_c = unit_idle_c;
	wire unit_is1_c = &(u1s ^ {u1c[8:0], 1'b1});
	wire [11:0] unused_cs2_co = {
		u1c[9], 11'd0
	};

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else if (flush) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else begin
			pend <= pend_n;
			chk_n <= chk_nx;
			unit_n <= unit_nx;
			pend_is0_r <= pend_is0_c;
			pend_is1_r <= pend_is1_c;
			chk_is0_r  <= chk_is0_c;
			chk_is1_r  <= chk_is1_c;
			unit_is0_r <= unit_is0_c;
			unit_is1_r <= unit_is1_c;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge cnt_rst_sync) begin
		if (cnt_rst_sync) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else if (flush) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else begin
			pend <= pend_n;
			chk_n <= chk_nx;
			unit_n <= unit_nx;
			pend_is0_r <= pend_is0_c;
			pend_is1_r <= pend_is1_c;
			chk_is0_r  <= chk_is0_c;
			chk_is1_r  <= chk_is1_c;
			unit_is0_r <= unit_is0_c;
			unit_is1_r <= unit_is1_c;
		end
	end
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else if (flush) begin
			pend <= 8'd0;
			chk_n <= 8'd0;
			unit_n <= 10'd0;
			pend_is0_r <= 1'b1;
			pend_is1_r <= 1'b0;
			chk_is0_r  <= 1'b1;
			chk_is1_r  <= 1'b0;
			unit_is0_r <= 1'b1;
			unit_is1_r <= 1'b0;
		end else begin
			pend <= pend_n;
			chk_n <= chk_nx;
			unit_n <= unit_nx;
			pend_is0_r <= pend_is0_c;
			pend_is1_r <= pend_is1_c;
			chk_is0_r  <= chk_is0_c;
			chk_is1_r  <= chk_is1_c;
			unit_is0_r <= unit_is0_c;
			unit_is1_r <= unit_is1_c;
		end
	end
`endif

	assign pend_zero = pend_zero_c;
	assign chk_idle  = chk_idle_c;
	assign unit_idle = unit_idle_c;
	assign ax_zero   = ax_zero_c;

	/* ax: precompute +1 and -1 in parallel with ax_inc/ax_dec control logic. */
	reg [7:0] ax_n;
	wire [7:0] ax_p1, ax_m1;
	wire [1:0] ax_co_unused;

	am_add #(.W(8)) ax_add_p1 (.a(ax_n), .b(8'd0), .ci(1'b1), .s(ax_p1), .co(ax_co_unused[0]));
	am_add #(.W(8)) ax_add_m1 (.a(ax_n), .b(8'hFF), .ci(1'b0), .s(ax_m1), .co(ax_co_unused[1]));
	wire [7:0] ax_next = (ax_inc_s == ax_dec_s) ? ax_n : (ax_inc_s ? ax_p1 : ax_m1);

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge cnt_rst_sync)
		if (cnt_rst_sync) begin
			ax_n     <= 8'd0;
			ax_is0_r <= 1'b1;
			ax_is1_r <= 1'b0;
		end else begin
			ax_n     <= ax_next;
			ax_is0_r <= ax_next == 8'd0;
			ax_is1_r <= ax_next == 8'd1;
		end
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge cnt_rst_sync)
		if (cnt_rst_sync) begin
			ax_n     <= 8'd0;
			ax_is0_r <= 1'b1;
			ax_is1_r <= 1'b0;
		end else begin
			ax_n     <= ax_next;
			ax_is0_r <= ax_next == 8'd0;
			ax_is1_r <= ax_next == 8'd1;
		end
`else
	always @(posedge clk or negedge rst_n)
		if (!rst_n) begin
			ax_n     <= 8'd0;
			ax_is0_r <= 1'b1;
			ax_is1_r <= 1'b0;
		end else begin
			ax_n     <= ax_next;
			ax_is0_r <= ax_next == 8'd0;
			ax_is1_r <= ax_next == 8'd1;
		end
`endif

	wire unused_ok = &{1'b0, ax_co_unused, unused_cs2_co};
endmodule

`default_nettype wire
