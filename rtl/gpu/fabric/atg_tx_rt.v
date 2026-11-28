/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_tx_rt: K-way pipelined router connecting SUs to the shared spine TU.
 *
 * Multi-flit texture requests lock per child until tx_last passes.
 * Injects child index [KW-1:0] into p_tx_tag[TBIT +: KW] (at most 7 bits total
 * across all stages for 128 SUs -> leaves low tag bits [8:0] untouched).
 * Return flits (tr_*) are routed back via p_tr_tag[TBIT +: KW] through
 * flop-to-flop am_skid boundaries on both request and response paths.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_tx_rt #(
	parameter K = 4,
	parameter TBIT = 14
) (
	input wire clk,
	input wire rst_n,

	/* Child SU texture ports (packed, child 0 in low bits) */
	input wire [K-1:0] c_tx_valid,
	output wire [K-1:0] c_tx_ready,
	input wire [K*8-1:0] c_tx_op,
	input wire [K*4-1:0] c_tx_slot,
	input wire [K*4-1:0] c_tx_samp_slot,
	input wire [K-1:0] c_tx_vs,
	input wire [K*4-1:0] c_tx_mask,
	input wire [K*640-1:0] c_tx_coord,
	input wire [K*12-1:0] c_tx_off,
	input wire [K*16-1:0] c_tx_tag,
	input wire [K-1:0] c_tx_last,

	output wire [K-1:0] c_tr_valid,
	input wire [K-1:0] c_tr_ready,
	output wire [K*16-1:0] c_tr_tag,
	output wire [K*512-1:0] c_tr_data,
	output wire [K-1:0] c_tr_last,

	/* Parent port toward Spine TU */
	output wire p_tx_valid,
	input wire p_tx_ready,
	output wire [7:0] p_tx_op,
	output wire [3:0] p_tx_slot,
	output wire [3:0] p_tx_samp_slot,
	output wire p_tx_vs,
	output wire [3:0] p_tx_mask,
	output wire [639:0] p_tx_coord,
	output wire [11:0] p_tx_off,
	output wire [15:0] p_tx_tag,
	output wire p_tx_last,

	input wire p_tr_valid,
	output wire p_tr_ready,
	input wire [15:0] p_tr_tag,
	input wire [511:0] p_tr_data,
	input wire p_tr_last
);
	localparam KW    = (K > 2) ? 2 : 1;
	localparam TX_W  = 8 + 4 + 4 + 1 + 4 + 640 + 12 + 16 + 1; /* 690 bits */
	localparam TR_W  = 16 + 512 + 1;                          /* 529 bits */

	wire [K-1:0] sk_v, sk_r;
	wire [K*TX_W-1:0] sk_d_flat;

	genvar gk;
	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_tx_sk
			wire [15:0] raw_tag = c_tx_tag[gk*16 +: 16];
			wire [15:0] stamped_tag;
			if (TBIT == 15) begin : g_topbit
				assign stamped_tag = {gk[0], raw_tag[14:0]};
			end else if (TBIT + KW == 16) begin : g_hi
				assign stamped_tag = {gk[KW-1:0], raw_tag[TBIT-1:0]};
			end else begin : g_mid
				assign stamped_tag = {raw_tag[15:TBIT+KW], gk[KW-1:0], raw_tag[TBIT-1:0]};
			end
			wire [TX_W-1:0] in_vec = {
				c_tx_op[gk*8 +: 8],
				c_tx_slot[gk*4 +: 4],
				c_tx_samp_slot[gk*4 +: 4],
				c_tx_vs[gk],
				c_tx_mask[gk*4 +: 4],
				c_tx_coord[gk*640 +: 640],
				c_tx_off[gk*12 +: 12],
				stamped_tag,
				c_tx_last[gk]
			};
			atg_ex_rt_skid #(.WIDTH(TX_W), .SL_S(10)) u_tx_sk (
				.clk(clk), .rst_n(rst_n),
				.in_valid(c_tx_valid[gk]), .in_ready(c_tx_ready[gk]),
				.in_data(in_vec),
				.out_valid(sk_v[gk]), .out_ready(sk_r[gk]),
				.out_data(sk_d_flat[gk*TX_W +: TX_W])
			);
		end
	endgenerate

	/* Parallel unrotated 2-way RR arbiter over sk_v */
	reg [K-1:0] rr_ge_r;
	wire [K-1:0] sk_v_hi = sk_v & rr_ge_r;
	wire [K-1:0] win_lo_1h, win_hi_1h;
	assign win_lo_1h[0] = sk_v[0];
	assign win_hi_1h[0] = sk_v_hi[0];
	generate
		for (gk = 1; gk < K; gk = gk + 1) begin : g_tx_pe
			assign win_lo_1h[gk] = sk_v[gk] && !(|sk_v[gk-1:0]);
			assign win_hi_1h[gk] = sk_v_hi[gk] && !(|sk_v_hi[gk-1:0]);
		end
	endgenerate
	wire         pick_valid = |sk_v;
	wire         any_hi     = |sk_v_hi;
	wire [K-1:0] pick_oh    = any_hi ? win_hi_1h : win_lo_1h;

	wire [K-1:0] next_rr_ge;
	assign next_rr_ge[0] = pick_oh[K - 1];
	generate
		for (gk = 1; gk < K; gk = gk + 1) begin : g_tx_nrr
			assign next_rr_ge[gk] = pick_oh[K - 1] || (|pick_oh[gk-1:0]);
		end
	endgenerate

	wire out_in_ready;
	wire fire = pick_valid && out_in_ready;

	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_tx_rdy
			assign sk_r[gk] = out_in_ready && pick_oh[gk];
		end
	endgenerate

	localparam integer SL_W = 8;
	localparam integer N_SL = (TX_W + SL_W - 1) / SL_W;
	localparam integer N_M1 = (N_SL + 11) / 12;
	wire [N_M1*K-1:0] sel_m1;
	genvar gm1, gmk;
	generate
		for (gm1 = 0; gm1 < N_M1; gm1 = gm1 + 1) begin : g_tx_m1
			for (gmk = 0; gmk < K; gmk = gmk + 1) begin : g_m1_k
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m1 (.I0(pick_oh[gmk]), .O(sel_m1[gm1*K + gmk]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_m1[gm1*K + gmk] = pick_oh[gmk];
`else
				assign sel_m1[gm1*K + gmk] = pick_oh[gmk];
`endif
			end
		end
	endgenerate
	wire [TX_W-1:0] mux_tx;
	genvar gs;
	generate
		for (gs = 0; gs < N_SL; gs = gs + 1) begin : g_tx_mux
			localparam integer B0 = gs * SL_W;
			localparam integer BW = (B0 + SL_W <= TX_W) ? SL_W : (TX_W - B0);
			(* keep = "true" *) wire [K-1:0] sel_oh_sl;
			for (gk = 0; gk < K; gk = gk + 1) begin : g_sel_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sel (.I0(sel_m1[(gs / 12)*K + gk]), .O(sel_oh_sl[gk]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_oh_sl[gk] = sel_m1[(gs / 12)*K + gk];
`else
				assign sel_oh_sl[gk] = sel_m1[(gs / 12)*K + gk];
`endif
			end
			reg [BW-1:0] sl_v;
			integer mi;
			always @(*) begin
				sl_v = {BW{1'b0}};
				for (mi = 0; mi < K; mi = mi + 1)
					if (sel_oh_sl[mi])
						sl_v = sl_v | sk_d_flat[mi*TX_W + B0 +: BW];
			end
			assign mux_tx[B0 +: BW] = sl_v;
		end
	endgenerate

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			rr_ge_r <= {K{1'b1}};
		end else if (fire) begin
			rr_ge_r <= next_rr_ge;
		end
	end

	atg_ex_rt_skid #(.WIDTH(TX_W), .SL_S(7)) u_tx_out_sk (
		.clk(clk), .rst_n(rst_n),
		.in_valid(pick_valid), .in_ready(out_in_ready),
		.in_data(mux_tx),
		.out_valid(p_tx_valid), .out_ready(p_tx_ready),
		.out_data({p_tx_op, p_tx_slot, p_tx_samp_slot, p_tx_vs, p_tx_mask,
		           p_tx_coord, p_tx_off, p_tx_tag, p_tx_last})
	);

	/* Return channel (TU -> SU) */
	wire pr_v, pr_r;
	wire [TR_W-1:0] pr_d;

	atg_ex_rt_skid #(.WIDTH(TR_W), .SL_S(10)) u_tr_in_sk (
		.clk(clk), .rst_n(rst_n),
		.in_valid(p_tr_valid), .in_ready(p_tr_ready),
		.in_data({p_tr_tag, p_tr_data, p_tr_last}),
		.out_valid(pr_v), .out_ready(pr_r),
		.out_data(pr_d)
	);

	wire [15:0] tr_tag_dec = pr_d[TR_W-1 -: 16];
	wire [KW-1:0] dst = tr_tag_dec[TBIT +: KW];
	wire [K-1:0] cr_in_r;
	assign pr_r = cr_in_r[dst];

	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_tr_sk
			wire [TR_W-1:0] c_out;
			atg_ex_rt_skid #(.WIDTH(TR_W), .SL_S(10)) u_tr_out_sk (
				.clk(clk), .rst_n(rst_n),
				.in_valid(pr_v && (dst == gk[KW-1:0])), .in_ready(cr_in_r[gk]),
				.in_data(pr_d),
				.out_valid(c_tr_valid[gk]), .out_ready(c_tr_ready[gk]),
				.out_data(c_out)
			);
			assign c_tr_tag[gk*16 +: 16]    = c_out[TR_W-1 -: 16];
			assign c_tr_data[gk*512 +: 512] = c_out[512:1];
			assign c_tr_last[gk]            = c_out[0];
		end
	endgenerate

	wire unused_ok = &{1'b0, tr_tag_dec};
endmodule

`default_nettype wire
