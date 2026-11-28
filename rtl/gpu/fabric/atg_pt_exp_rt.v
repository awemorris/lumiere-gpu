/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_pt_exp_rt: K-way pipelined stream router for shared Spine PT exports.
 *
 * Multiplexes single-beat PT export writes and serialized vertex warp-end
 * (wc_end) markers from K SUs/subtrees toward the single shared Spine
 * atg_su_pt instance (Option I, docs/noc_and_mmu_architecture.md §6.1).
 * Uses strict flop-to-flop am_skid boundaries on every child and parent port.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_pt_exp_rt #(
	parameter K = 4,
	parameter W = 277
) (
	input wire clk,
	input wire rst_n,

	input wire [K-1:0] c_valid,
	output wire [K-1:0] c_ready,
	input wire [K*W-1:0] c_data,

	output wire p_valid,
	input wire p_ready,
	output wire [W-1:0] p_data
);
	localparam KW = (K > 2) ? 2 : 1;

	wire [K-1:0] sk_v, sk_r;
	wire [K*W-1:0] sk_d_flat;

	genvar gk;
	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_sk
			atg_ex_rt_skid #(.WIDTH(W), .SL_S(7)) u_sk (
				.clk(clk), .rst_n(rst_n),
				.in_valid(c_valid[gk]), .in_ready(c_ready[gk]),
				.in_data(c_data[gk*W +: W]),
				.out_valid(sk_v[gk]), .out_ready(sk_r[gk]),
				.out_data(sk_d_flat[gk*W +: W])
			);
		end
	endgenerate

	reg [KW-1:0] last_won;
	reg [KW-1:0] pick;
	wire         pick_valid = |sk_v;
	integer      st, ii;

	always @(*) begin
		pick = {KW{1'b0}};
		for (st = K - 1; st >= 0; st = st - 1) begin
			for (ii = 0; ii < K; ii = ii + 1) begin
				if (last_won == ii[KW-1:0] && sk_v[(ii + 1 + st) % K])
					pick = ((ii + 1 + st) % K);
			end
		end
	end

	wire out_in_ready;
	wire fire = pick_valid && out_in_ready;

	generate
		for (gk = 0; gk < K; gk = gk + 1) begin : g_rdy
			assign sk_r[gk] = out_in_ready && (pick == gk[KW-1:0]);
		end
	endgenerate

	localparam integer SL_W = 8;
	localparam integer N_SL = (W + SL_W - 1) / SL_W;
	localparam integer N_M1 = (N_SL + 11) / 12;
	wire [N_M1*KW-1:0] sel_m1;
	genvar gm1, gmb;
	generate
		for (gm1 = 0; gm1 < N_M1; gm1 = gm1 + 1) begin : g_pt_m1
			for (gmb = 0; gmb < KW; gmb = gmb + 1) begin : g_m1_b
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m1 (.I0(pick[gmb]), .O(sel_m1[gm1*KW + gmb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_m1[gm1*KW + gmb] = pick[gmb];
`else
				assign sel_m1[gm1*KW + gmb] = pick[gmb];
`endif
			end
		end
	endgenerate
	wire [W-1:0] mux_data;
	genvar gs, gsb;
	generate
		for (gs = 0; gs < N_SL; gs = gs + 1) begin : g_mux
			localparam integer B0 = gs * SL_W;
			localparam integer BW = (B0 + SL_W <= W) ? SL_W : (W - B0);
			wire [KW-1:0] sel_sl;
			for (gsb = 0; gsb < KW; gsb = gsb + 1) begin : g_sb
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bsl (.I0(sel_m1[(gs / 12)*KW + gsb]), .O(sel_sl[gsb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign sel_sl[gsb] = sel_m1[(gs / 12)*KW + gsb];
`else
				assign sel_sl[gsb] = sel_m1[(gs / 12)*KW + gsb];
`endif
			end
			reg [BW-1:0] sl_v;
			integer mi;
			always @(*) begin
				sl_v = sk_d_flat[B0 +: BW];
				for (mi = 1; mi < K; mi = mi + 1)
					if (sel_sl == mi[KW-1:0])
						sl_v = sk_d_flat[mi*W + B0 +: BW];
			end
			assign mux_data[B0 +: BW] = sl_v;
		end
	endgenerate

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			last_won <= {KW{1'b0}};
		end else if (fire) begin
			last_won <= pick;
		end
	end

	atg_ex_rt_skid #(.WIDTH(W), .SL_S(7)) u_out_sk (
		.clk(clk), .rst_n(rst_n),
		.in_valid(pick_valid), .in_ready(out_in_ready),
		.in_data(mux_data),
		.out_valid(p_valid), .out_ready(p_ready),
		.out_data(p_data)
	);
endmodule

`default_nettype wire
