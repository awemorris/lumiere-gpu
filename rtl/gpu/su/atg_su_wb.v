/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_wb: Write-back arbitration: one RF write port shared by the short
 * pipeline (one result per cycle), the long units' beats, and the VF/RX
 * input writes; pushes rq results; releases scoreboard entries.
 *
 * The behaviour, and where it differs from
 * the design: spec/gpu/su/atg_su_wb.md (§10 Status).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_wb_skid #(
	parameter W = 32,
	parameter WIDTH = W
) (
	input wire clk,
	input wire rst_n,

	input wire in_valid,
	(* keep = "true" *) output reg in_ready,
	input wire [WIDTH-1:0] in_data,

	(* keep = "true" *) output reg out_valid,
	input wire out_ready,
	output wire [WIDTH-1:0] out_data
);
	wire rst_p = !rst_n;
	(* keep = "true" *) reg skid_valid;

	wire skid_load = in_valid && in_ready;

	localparam N_SK_SL = (WIDTH + 3) / 4;
	localparam N_SK_GRP = (N_SK_SL + 7) / 8;
	localparam N_SK_RST = (N_SK_SL + 3) / 4;
	wire out_ready_lo, out_ready_hi, in_valid_lo, in_valid_hi;
	wire rst_p_lo, rst_p_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rdy_lo (.I0(out_ready), .O(out_ready_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rdy_hi (.I0(out_ready), .O(out_ready_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_val_lo (.I0(in_valid),  .O(in_valid_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_val_hi (.I0(in_valid),  .O(in_valid_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rst_lo (.I0(rst_p),     .O(rst_p_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rst_hi (.I0(rst_p),     .O(rst_p_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign out_ready_lo = out_ready;
	assign out_ready_hi = out_ready;
	assign in_valid_lo = in_valid;
	assign in_valid_hi = in_valid;
	assign rst_p_lo = rst_p;
	assign rst_p_hi = rst_p;
`else
	assign out_ready_lo = out_ready;
	assign out_ready_hi = out_ready;
	assign in_valid_lo  = in_valid;
	assign in_valid_hi  = in_valid;
	assign rst_p_lo     = rst_p;
	assign rst_p_hi     = rst_p;
`endif
	wire [N_SK_GRP-1:0] out_ready_g;
	wire [N_SK_GRP-1:0] in_valid_g;
	wire [N_SK_RST-1:0] rst_p_g;
	genvar gkg;
	generate
		for (gkg = 0; gkg < N_SK_GRP; gkg = gkg + 1) begin : g_sk_grp
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rdy (.I0((gkg < (N_SK_GRP / 2)) ? out_ready_lo : out_ready_hi), .O(out_ready_g[gkg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_val (.I0((gkg < (N_SK_GRP / 2)) ? in_valid_lo  : in_valid_hi),  .O(in_valid_g[gkg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign out_ready_g[gkg] = (gkg < (N_SK_GRP / 2)) ? out_ready_lo : out_ready_hi;
			assign in_valid_g[gkg] = (gkg < (N_SK_GRP / 2)) ? in_valid_lo  : in_valid_hi;
`else
			assign out_ready_g[gkg] = out_ready;
			assign in_valid_g[gkg]  = in_valid;
`endif
		end
		for (gkg = 0; gkg < N_SK_RST; gkg = gkg + 1) begin : g_sk_rst
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_rst (.I0((gkg < (N_SK_RST / 2)) ? rst_p_lo : rst_p_hi), .O(rst_p_g[gkg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign rst_p_g[gkg] = (gkg < (N_SK_RST / 2)) ? rst_p_lo : rst_p_hi;
`else
			assign rst_p_g[gkg] = rst_p;
`endif
		end
	endgenerate

	genvar gsk;
	generate
	for (gsk = 0; gsk < N_SK_SL; gsk = gsk + 1) begin : g_sk_sl
		localparam SK_W = (gsk * 4 + 4 <= WIDTH) ? 4 : (WIDTH - gsk * 4);
		wire sl_out_ready, sl_in_valid;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sl_rdy (.I0(out_ready_g[gsk / 8]), .O(sl_out_ready));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sl_val (.I0(in_valid_g[gsk / 8]),  .O(sl_in_valid));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sl_out_ready = out_ready_g[gsk / 8];
		assign sl_in_valid = in_valid_g[gsk / 8];
`else
		assign sl_out_ready = out_ready_g[gsk / 8];
		assign sl_in_valid  = in_valid_g[gsk / 8];
`endif
		wire sl_rst_p = rst_p_g[gsk / 4];
		(* dont_touch = "true" *) reg out_v_sl, skid_v_sl, in_r_sl;
		always @(posedge clk or posedge sl_rst_p) begin
			if (sl_rst_p) begin
				in_r_sl   <= 1'b1;
				out_v_sl  <= 1'b0;
				skid_v_sl <= 1'b0;
			end else begin
				if (skid_v_sl) begin
					if (sl_out_ready) begin
						out_v_sl  <= 1'b1;
						skid_v_sl <= 1'b0;
						in_r_sl   <= 1'b1;
					end
				end else begin
					if (sl_out_ready || !out_v_sl) begin
						out_v_sl <= sl_in_valid;
					end else if (sl_in_valid && in_r_sl) begin
						skid_v_sl <= 1'b1;
						in_r_sl   <= 1'b0;
					end
				end
			end
		end
		wire load_out_s = skid_v_sl ? sl_out_ready : (sl_in_valid && (sl_out_ready || !out_v_sl));
		wire skid_load_s = sl_in_valid && in_r_sl;
		(* shreg_extract = "no", dont_touch = "true" *) reg [SK_W-1:0] out_sl;
		(* shreg_extract = "no", dont_touch = "true" *) reg [SK_W-1:0] skid_sl;
		always @(posedge clk) begin
			if (load_out_s)
				out_sl <= skid_v_sl ? skid_sl : in_data[gsk*4 +: SK_W];
			if (skid_load_s)
				skid_sl <= in_data[gsk*4 +: SK_W];
		end
		assign out_data[gsk*4 +: SK_W] = out_sl;
	end
	endgenerate

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			in_ready   <= 1'b1;
			out_valid  <= 1'b0;
			skid_valid <= 1'b0;
		end else begin
			if (skid_valid) begin
				if (out_ready) begin
					out_valid  <= 1'b1;
					skid_valid <= 1'b0;
					in_ready   <= 1'b1;
				end
			end else begin
				if (out_ready || !out_valid) begin
					out_valid <= in_valid;
				end else if (skid_load) begin
					skid_valid <= 1'b1;
					in_ready   <= 1'b0;
				end
			end
		end
	end
endmodule

module atg_su_wb_fifo #(
	parameter W = 8,
	parameter DEPTH_LOG2 = 2
) (
	input wire clk,
	input wire rst_n,
	input wire in_valid,
	output wire in_ready,
	input wire [W-1:0] in_data,
	output wire out_valid,
	input wire out_ready,
	output wire [W-1:0] out_data,
	output wire [DEPTH_LOG2:0] count
);
	wire rst_p = !rst_n;
	(* max_fanout = 12 *) reg [DEPTH_LOG2:0] wp;
	(* max_fanout = 12 *) reg [DEPTH_LOG2:0] rp;

	wire [DEPTH_LOG2:0] used;
	wire unused_used_bo;
	am_sub #(.W(DEPTH_LOG2 + 1)) u_used (
		.a(wp), .b(rp), .bi(1'b0), .d(used), .bo(unused_used_bo)
	);
	wire in_rdy_w = (wp[DEPTH_LOG2-1:0] != rp[DEPTH_LOG2-1:0]) || (wp[DEPTH_LOG2] == rp[DEPTH_LOG2]);
	(* keep = "true", max_fanout = 12 *) reg r_valid;
	wire push = in_valid && in_rdy_w;
	wire pop = r_valid && out_ready;

	wire [DEPTH_LOG2:0] wp_next;
	wire unused_wp_co;
	am_add #(.W(DEPTH_LOG2 + 1)) u_wp (
		.a(wp), .b({(DEPTH_LOG2 + 1){1'b0}}), .ci(1'b1), .s(wp_next), .co(unused_wp_co)
	);

	wire [DEPTH_LOG2:0] rp_next;
	wire unused_rp_co;
	am_add #(.W(DEPTH_LOG2 + 1)) u_rp (
		.a(rp), .b({(DEPTH_LOG2 + 1){1'b0}}), .ci(1'b1), .s(rp_next), .co(unused_rp_co)
	);

	wire will_empty = (rp_next == wp);
	assign count = used;
	assign in_ready = in_rdy_w;
	assign out_valid = r_valid;

	localparam N_FF_SL = (W + 7) / 8;
	genvar gff;
	generate
	for (gff = 0; gff < N_FF_SL; gff = gff + 1) begin : g_ff_sl
		localparam FF_W = (gff * 8 + 8 <= W) ? 8 : (W - gff * 8);
		(* ram_style = "distributed" *) reg [FF_W-1:0] mem_sl [0:(1 << DEPTH_LOG2) - 1];
		(* dont_touch = "true" *) reg [DEPTH_LOG2-1:0] wp_sl, rp_sl;
		wire push_s, pop_s;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_psh (.I0(in_valid && in_rdy_w), .O(push_s));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_pop (.I0(r_valid && out_ready), .O(pop_s));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign push_s = in_valid && in_rdy_w;
		assign pop_s = r_valid && out_ready;
`else
		assign push_s = in_valid && in_rdy_w;
		assign pop_s  = r_valid && out_ready;
`endif
		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				wp_sl <= {DEPTH_LOG2{1'b0}};
				rp_sl <= {DEPTH_LOG2{1'b0}};
			end else begin
				if (push_s)
					wp_sl <= wp_next[DEPTH_LOG2-1:0];
				if (pop_s)
					rp_sl <= rp_next[DEPTH_LOG2-1:0];
			end
		end
		always @(posedge clk) begin
			if (push_s)
				mem_sl[wp_sl] <= in_data[gff*8 +: FF_W];
		end
		assign out_data[gff*8 +: FF_W] = mem_sl[rp_sl];
	end
	endgenerate

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			wp <= {(DEPTH_LOG2 + 1){1'b0}};
			rp <= {(DEPTH_LOG2 + 1){1'b0}};
			r_valid <= 1'b0;
		end else begin
			if (push)
				wp <= wp_next;
			if (pop)
				rp <= rp_next;
			case ({push, pop})
			2'b10: r_valid <= 1'b1;
			2'b01: r_valid <= !will_empty;
			default: ;
			endcase
		end
	end
endmodule

module atg_su_wb #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter GRP_W = 2,	/* log2(32/NPHYS): the lane-group number of a warp (0 for NPHYS 32 is given as 1 bit) */
	parameter SBE_W = 2,	/* log2(SB_ENTRIES); the value of all ones above the entries is "no entry" (an entry index is SBE_W+1 bits) */
	parameter NS = 6,	/* Short sources (alu, fpu, pk, lane, lda, exec) */
	parameter NL = 6,	/* Long sources (lsu, lds, cbank, tex, xu, div) */
	parameter SHORT_LAT = 6,	/* Cycles from x_valid to the short result (sizes the input FIFOs) */
	parameter REG_LONG = 0,	/* 1: register long inputs via am_skid and pipeline lq/fq before RF mux */
	parameter USE_SV_REP = 0	/* 1: use replicated 8-slice s_valid_rep to bound fanout <= 256 */
) (
	input wire clk,
	input wire rst_n,
	/* Short results (packed, source 0 in the low bits; at most one valid per cycle) */
	input wire [NS-1:0] s_valid,
	input wire [NS*8-1:0] s_valid_rep,
	input wire [NS*WARP_W-1:0] s_warp,
	input wire [NS*GRP_W-1:0] s_grp,
	input wire [NS*7-1:0] s_rd,
	input wire [NS*NPHYS*32-1:0] s_data,
	input wire [NS*NPHYS-1:0] s_lanes,
	/* Long results (packed) */
	input wire [NL-1:0] l_valid,
	output wire [NL-1:0] l_ready,
	input wire [NL*WARP_W-1:0] l_warp,
	input wire [NL*GRP_W-1:0] l_grp,
	input wire [NL*7-1:0] l_rd,
	input wire [NL*NPHYS*32-1:0] l_data,
	input wire [NL*NPHYS-1:0] l_lanes,
	input wire [NL-1:0] l_wr,
	input wire [NL-1:0] l_last,
	input wire [NL*(SBE_W+1)-1:0] l_sbe,
	input wire [NL*3-1:0] l_cls,
	/* Input writes (0 VF, 1 RX: one lane word per cycle) */
	input wire [1:0] in_we,
	input wire [2*WARP_W-1:0] in_warp,
	input wire [11:0] in_reg,
	input wire [9:0] in_lane,
	input wire [63:0] in_data,
	/* Outputs */
	output wire w_en,
	output wire [WARP_W-1:0] w_warp,
	output wire [GRP_W-1:0] w_grp,
	output wire [5:0] w_reg,
	output wire [NPHYS*32-1:0] w_data,
	output wire [NPHYS-1:0] w_lanes,
	output wire rel_valid,
	output wire [WARP_W-1:0] rel_warp,
	output wire [SBE_W:0] rel_sbe,
	output wire [2:0] rel_cls,
	output wire rq_push,
	output wire [NPHYS*32-1:0] rq_data,
	output wire wb_stall
);
`include "atg_su_uop.vh"

	localparam DW = NPHYS * 32;
	localparam G = 32 / NPHYS;
	localparam LP_W = NL > 1 ? $clog2(NL) : 1;
	/* An input FIFO entry: {warp, reg, lane, data}. */
	localparam IW = WARP_W + 6 + 5 + 32;
	/*
	 * The input FIFOs' depth.  A source writes one word a cycle and cannot
	 * wait; a word waits while short results hold the port.  After the first
	 * waiting word raises wb_stall, the short results still to come are at
	 * most those in the units' pipelines (SHORT_LAT), the instruction in
	 * atg_su_opd and the one issued in that cycle (G each; a subgroup
	 * instruction's G more bus cycles carry no result) and the cycles of the
	 * stall's latency (2); source 1 also waits while source 0 has words, so
	 * twice that.  NPHYS 4 (G 8) needs 64; a fixed 16 lost RX writes there.
	 */
	localparam IN_LOG2 = $clog2(2 * (SHORT_LAT + 2 * G + 2));

	wire rst_p = !rst_n;

	/* The short result of this cycle: structured balanced 2-level MUX tree (at most one valid per cycle) */
	wire [NS-1:0] s_valid_sd, s_valid_mux, s_valid_sel;
	genvar gsv;
	generate
	for (gsv = 0; gsv < NS; gsv = gsv + 1) begin : g_buf_sv
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sv_sd  (.I0(s_valid[gsv]), .O(s_valid_sd[gsv]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sv_mux (.I0(s_valid[gsv]), .O(s_valid_mux[gsv]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sv_sel (.I0(s_valid[gsv]), .O(s_valid_sel[gsv]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign s_valid_sd[gsv] = s_valid[gsv];
		assign s_valid_mux[gsv] = s_valid[gsv];
		assign s_valid_sel[gsv] = s_valid[gsv];
`else
		assign s_valid_sd[gsv]  = s_valid[gsv];
		assign s_valid_mux[gsv] = s_valid[gsv];
		assign s_valid_sel[gsv] = s_valid[gsv];
`endif
	end
	endgenerate

	wire v_01_q0, v_01_q1, v_01_q2, v_01_q3, v_01_c;
	wire v_23_q0, v_23_q1, v_23_q2, v_23_q3, v_23_c;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q0 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q0));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q1 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q1));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q2 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q2));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q3 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q3));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_c  (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_c));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q0 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q0));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q1 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q1));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q2 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q2));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q3 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q3));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_c  (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_c));
`elsif SYNTH_KINTEX_ULTRASCALE
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q0 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q0));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q1 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q1));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q2 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q2));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_q3 (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_q3));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v01_c  (.I0(s_valid_mux[0]), .I1(s_valid_mux[1]), .O(v_01_c));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q0 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q0));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q1 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q1));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q2 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q2));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_q3 (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_q3));
	(* dont_touch = "true" *) LUT2 #(.INIT(4'he)) u_b_v23_c  (.I0(s_valid_mux[2]), .I1(s_valid_mux[3]), .O(v_23_c));
`else
	assign v_01_q0 = s_valid_mux[0] || s_valid_mux[1];
	assign v_01_q1 = s_valid_mux[0] || s_valid_mux[1];
	assign v_01_q2 = s_valid_mux[0] || s_valid_mux[1];
	assign v_01_q3 = s_valid_mux[0] || s_valid_mux[1];
	assign v_01_c  = s_valid_mux[0] || s_valid_mux[1];
	assign v_23_q0 = s_valid_mux[2] || s_valid_mux[3];
	assign v_23_q1 = s_valid_mux[2] || s_valid_mux[3];
	assign v_23_q2 = s_valid_mux[2] || s_valid_mux[3];
	assign v_23_q3 = s_valid_mux[2] || s_valid_mux[3];
	assign v_23_c  = s_valid_mux[2] || s_valid_mux[3];
`endif

	wire [DW-1:0] sd;
	genvar gsd;
	generate
	for (gsd = 0; gsd < DW / 16; gsd = gsd + 1) begin : g_sd_sl
		localparam integer SV_SL = (gsd * 8) / (DW / 16);
		(* dont_touch = "true" *) wire sv0_s = USE_SV_REP ? s_valid_rep[0*8 + SV_SL] : s_valid_sd[0];
		(* dont_touch = "true" *) wire sv1_s = USE_SV_REP ? s_valid_rep[1*8 + SV_SL] : s_valid_sd[1];
		(* dont_touch = "true" *) wire sv2_s = USE_SV_REP ? s_valid_rep[2*8 + SV_SL] : s_valid_sd[2];
		(* dont_touch = "true" *) wire sv3_s = USE_SV_REP ? s_valid_rep[3*8 + SV_SL] : s_valid_sd[3];
		(* dont_touch = "true" *) wire sv4_s = USE_SV_REP ? s_valid_rep[4*8 + SV_SL] : s_valid_sd[4];
		wire v01_s = USE_SV_REP ? (sv0_s || sv1_s) : ((gsd < 8) ? v_01_q0 : ((gsd < 16) ? v_01_q1 : ((gsd < 24) ? v_01_q2 : v_01_q3)));
		wire v23_s = USE_SV_REP ? (sv2_s || sv3_s) : ((gsd < 8) ? v_23_q0 : ((gsd < 16) ? v_23_q1 : ((gsd < 24) ? v_23_q2 : v_23_q3)));
		(* dont_touch = "true" *) wire [15:0] sd_01_s = sv0_s ? s_data[0*DW + gsd*16 +: 16] : s_data[1*DW + gsd*16 +: 16];
		(* dont_touch = "true" *) wire [15:0] sd_23_s = sv2_s ? s_data[2*DW + gsd*16 +: 16] : s_data[3*DW + gsd*16 +: 16];
		(* dont_touch = "true" *) wire [15:0] sd_45_s = sv4_s ? s_data[4*DW + gsd*16 +: 16] : s_data[5*DW + gsd*16 +: 16];
		assign sd[gsd*16 +: 16] = v01_s ? sd_01_s : (v23_s ? sd_23_s : sd_45_s);
	end
	endgenerate

	wire [NS-1:0] s_we_u;
	genvar gsu;
	generate
		for (gsu = 0; gsu < NS; gsu = gsu + 1) begin : g_s_we_u
			assign s_we_u[gsu] = (s_rd[gsu*7 +: 7] != 7'd63);
		end
	endgenerate
	/* s_valid is one-hot: balanced 3+3 reduction avoids deep tree on rf.we_r. */
	wire s_we_012 = (s_valid_mux[0] && s_we_u[0]) || (s_valid_mux[1] && s_we_u[1]) || (s_valid_mux[2] && s_we_u[2]);
	wire s_we_345 = (s_valid_mux[3] && s_we_u[3]) || (s_valid_mux[4] && s_we_u[4]) || (s_valid_mux[5] && s_we_u[5]);
	wire s_we_pre = s_we_012 || s_we_345;

	wire [6:0] srd_01 = s_valid_mux[0] ? s_rd[0*7 +: 7] : s_rd[1*7 +: 7];
	wire [6:0] srd_23 = s_valid_mux[2] ? s_rd[2*7 +: 7] : s_rd[3*7 +: 7];
	wire [6:0] srd_45 = s_valid_mux[4] ? s_rd[4*7 +: 7] : s_rd[5*7 +: 7];
	wire [6:0] srd = v_01_c ? srd_01 : (v_23_c ? srd_23 : srd_45);

	wire [WARP_W-1:0] sw_01 = s_valid_mux[0] ? s_warp[0*WARP_W +: WARP_W] : s_warp[1*WARP_W +: WARP_W];
	wire [WARP_W-1:0] sw_23 = s_valid_mux[2] ? s_warp[2*WARP_W +: WARP_W] : s_warp[3*WARP_W +: WARP_W];
	wire [WARP_W-1:0] sw_45 = s_valid_mux[4] ? s_warp[4*WARP_W +: WARP_W] : s_warp[5*WARP_W +: WARP_W];
	wire [WARP_W-1:0] sw = v_01_c ? sw_01 : (v_23_c ? sw_23 : sw_45);

	wire [GRP_W-1:0] sg_01 = s_valid_mux[0] ? s_grp[0*GRP_W +: GRP_W] : s_grp[1*GRP_W +: GRP_W];
	wire [GRP_W-1:0] sg_23 = s_valid_mux[2] ? s_grp[2*GRP_W +: GRP_W] : s_grp[3*GRP_W +: GRP_W];
	wire [GRP_W-1:0] sg_45 = s_valid_mux[4] ? s_grp[4*GRP_W +: GRP_W] : s_grp[5*GRP_W +: GRP_W];
	wire [GRP_W-1:0] sg = v_01_c ? sg_01 : (v_23_c ? sg_23 : sg_45);

	wire [NPHYS-1:0] sl;
	genvar gsl, gss;
	generate
		for (gsl = 0; gsl < NPHYS; gsl = gsl + 1) begin : g_sl_bit
			wire [NS-1:0] sl_v;
			for (gss = 0; gss < NS; gss = gss + 1) begin : g_sl_s
				assign sl_v[gss] = s_valid_mux[gss] && s_lanes[gss*NPHYS + gsl];
			end
			assign sl[gsl] = |sl_v;
		end
	endgenerate

	/* Direct 6-input reduction on s_valid avoids extra LUT1 hop on critical WB stall/arb paths. */
	wire sv = |s_valid;

	/* Input writes: one FIFO per source (VF, RX), since they cannot wait. */
	(* max_fanout = 16 *) wire [1:0] fi_valid;
	wire [1:0] fi_ready;
	(* max_fanout = 16 *) wire [2*IW-1:0] fi_data;
	genvar s;
	generate
		for (s = 0; s < 2; s = s + 1) begin : inq
			wire in_rdy;
			wire [IN_LOG2:0] unused_count;
			atg_su_wb_fifo #(.W(IW), .DEPTH_LOG2(IN_LOG2)) q (
				.clk(clk), .rst_n(rst_n), .in_valid(in_we[s]), .in_ready(in_rdy),
				.in_data({in_warp[s*WARP_W +: WARP_W], in_reg[s*6 +: 6], in_lane[s*5 +: 5],
					in_data[s*32 +: 32]}),
				.out_valid(fi_valid[s]), .out_ready(fi_ready[s]),
				.out_data(fi_data[s*IW +: IW]), .count(unused_count)
			);
`ifndef SYNTHESIS
			always @(posedge clk or negedge rst_n)
				if (!rst_n) begin
				end else if (in_we[s] && !in_rdy)
					$display("ERROR atg_su_wb: input %0d FIFO full, a write lost", s);
`else
			wire unused_rdy = &{1'b0, in_rdy};
`endif
		end
	endgenerate

	wire sv_arb, sv_arb_fq, sv_arb_lq;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sva  (.I0(sv), .O(sv_arb));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_svaf (.I0(sv), .O(sv_arb_fq));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sval (.I0(sv), .O(sv_arb_lq));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign sv_arb = sv;
	assign sv_arb_fq = sv;
	assign sv_arb_lq = sv;
`else
	assign sv_arb    = sv;
	assign sv_arb_fq = sv;
	assign sv_arb_lq = sv;
`endif
	/* Combinational muxes */
	(* max_fanout = 16 *) reg [2:0] wait_n;

	/* Register release and stall outputs to break long combinational path to SB and Sched */
	(* max_fanout = 16 *) reg r_rel_valid;
	(* max_fanout = 16 *) reg [WARP_W-1:0] r_rel_warp;
	(* max_fanout = 16 *) reg [SBE_W:0] r_rel_sbe;
	(* max_fanout = 16 *) reg [2:0] r_rel_cls;
	(* max_fanout = 16 *) reg r_wb_stall;

	wire in_src = !fi_valid[0];
	wire [IW-1:0] fe = fi_data[in_src*IW +: IW];
	wire [4:0] fe_lane = fe[36:32];

	reg [DW-1:0] fe_data;
	reg [NPHYS-1:0] fe_lanes;
	integer l;
	always @(*) begin
		for (l = 0; l < NPHYS; l = l + 1) begin
			fe_data[l*32 +: 32] = fe[31:0];
			fe_lanes[l] = {27'd0, fe_lane} % NPHYS == l;
		end
	end
	wire [31:0] fe_grp = {27'd0, fe_lane} / NPHYS;

	generate
	if (REG_LONG != 0) begin : g_reg_long
		localparam LW = WARP_W + GRP_W + 7 + DW + NPHYS + 1 + 1 + (SBE_W + 1) + 3;
		wire [NL-1:0]           lk_valid;
		wire [NL-1:0]           lk_ready;
		wire [NL*WARP_W-1:0]    lk_warp;
		wire [NL*GRP_W-1:0]     lk_grp;
		wire [NL*7-1:0]         lk_rd;
		wire [NL*DW-1:0]        lk_data;
		wire [NL*NPHYS-1:0]     lk_lanes;
		wire [NL-1:0]           lk_wr;
		wire [NL-1:0]           lk_last;
		wire [NL*(SBE_W+1)-1:0] lk_sbe;
		wire [NL*3-1:0]         lk_cls;

		genvar glk;
		for (glk = 0; glk < NL; glk = glk + 1) begin : g_lskid
			atg_su_wb_skid #(.W(LW)) u_lskid (
				.clk(clk),
				.rst_n(rst_n),
				.in_valid(l_valid[glk]),
				.in_ready(l_ready[glk]),
				.in_data({
					l_warp[glk*WARP_W +: WARP_W],
					l_grp[glk*GRP_W +: GRP_W],
					l_rd[glk*7 +: 7],
					l_data[glk*DW +: DW],
					l_lanes[glk*NPHYS +: NPHYS],
					l_wr[glk],
					l_last[glk],
					l_sbe[glk*(SBE_W+1) +: (SBE_W+1)],
					l_cls[glk*3 +: 3]
				}),
				.out_valid(lk_valid[glk]),
				.out_ready(lk_ready[glk]),
				.out_data({
					lk_warp[glk*WARP_W +: WARP_W],
					lk_grp[glk*GRP_W +: GRP_W],
					lk_rd[glk*7 +: 7],
					lk_data[glk*DW +: DW],
					lk_lanes[glk*NPHYS +: NPHYS],
					lk_wr[glk],
					lk_last[glk],
					lk_sbe[glk*(SBE_W+1) +: (SBE_W+1)],
					lk_cls[glk*3 +: 3]
				})
			);
		end

		/* 1-entry registered input-FIFO output stage (fq_*) */
		(* keep = "true", max_fanout = 16 *) reg fq_valid;
		(* dont_touch = "true" *) reg [DW/16-1:0] fq_valid_sl;
		(* max_fanout = 16 *) reg [WARP_W-1:0] fq_warp;
		(* max_fanout = 16 *) reg [GRP_W-1:0]  fq_grp;
		(* max_fanout = 16 *) reg [5:0]        fq_reg;
		(* max_fanout = 16 *) reg [31:0]       fq_word;
		(* max_fanout = 16 *) reg [NPHYS-1:0]  fq_lanes;

		wire fq_can_load = !fq_valid || !sv_arb_fq;
		wire fq_ld_ctrl, fq_ld_wlo, fq_ld_whi, fq_ld_vsl;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_c  (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_ctrl));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_wl (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_wlo));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_wh (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_whi));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_vs (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_vsl));
`elsif SYNTH_KINTEX_ULTRASCALE
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_c  (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_ctrl));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_wl (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_wlo));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_wh (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_whi));
		(* dont_touch = "true" *) LUT2 #(.INIT(4'h1)) u_b_fq_vs (.I0(fq_valid), .I1(sv_arb_fq), .O(fq_ld_vsl));
`else
		assign fq_ld_ctrl = !fq_valid || !sv_arb_fq;
		assign fq_ld_wlo  = !fq_valid || !sv_arb_fq;
		assign fq_ld_whi  = !fq_valid || !sv_arb_fq;
		assign fq_ld_vsl  = !fq_valid || !sv_arb_fq;
`endif
		assign fi_ready = fq_can_load ? (fi_valid[0] ? 2'b01 : (fi_valid[1] ? 2'b10 : 2'b00)) : 2'b00;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				fq_valid    <= 1'b0;
				fq_valid_sl <= {(DW/16){1'b0}};
				fq_warp     <= {WARP_W{1'b0}};
				fq_grp      <= {GRP_W{1'b0}};
				fq_reg      <= 6'd0;
				fq_lanes    <= {NPHYS{1'b0}};
				fq_word     <= 32'd0;
			end else begin
				if (fq_ld_ctrl) begin
					fq_valid <= |fi_valid;
					fq_warp  <= fe[IW-1 -: WARP_W];
					fq_grp   <= fe_grp[GRP_W-1:0];
					fq_reg   <= fe[42:37];
					fq_lanes <= fe_lanes;
				end
				if (fq_ld_vsl)
					fq_valid_sl <= {(DW/16){|fi_valid}};
				if (fq_ld_wlo)
					fq_word[15:0]  <= fe[15:0];
				if (fq_ld_whi)
					fq_word[31:16] <= fe[31:16];
			end
		end

		wire [DW-1:0] fq_data = {NPHYS{fq_word}};

		/* Parallel unrotated 2-way round-robin arbiter over lk_valid (<= 11 cmos3 total). */
		reg  [NL-1:0] rr_ge_r;
		wire [NL-1:0] lk_hi = lk_valid & rr_ge_r;
		wire [NL-1:0] win_lo_1h, win_hi_1h;
		assign win_lo_1h[0] = lk_valid[0];
		assign win_hi_1h[0] = lk_hi[0];
		genvar g_lk;
		for (g_lk = 1; g_lk < NL; g_lk = g_lk + 1) begin : g_lk_pe
			assign win_lo_1h[g_lk] = lk_valid[g_lk] && !(|lk_valid[g_lk-1:0]);
			assign win_hi_1h[g_lk] = lk_hi[g_lk]    && !(|lk_hi[g_lk-1:0]);
		end
		wire          ksel_v   = |lk_valid;
		wire          ksel_hv  = |lk_hi;
		wire [NL-1:0] lk_s_one = ksel_hv ? win_hi_1h : win_lo_1h;

		wire [NL-1:0] lk_we_in;
		wire [NL-1:0] lk_rq_in;
		genvar gwe;
		for (gwe = 0; gwe < NL; gwe = gwe + 1) begin : g_lk_we
			assign lk_we_in[gwe] = lk_valid[gwe] && lk_wr[gwe] && (lk_rd[gwe*7 +: 7] != 7'd63);
			assign lk_rq_in[gwe] = lk_valid[gwe] && lk_wr[gwe] && (lk_rd[gwe*7 +: 7] == 7'd63);
		end

		reg [WARP_W-1:0] lk_s_warp;
		reg [GRP_W-1:0]  lk_s_grp;
		reg [5:0]        lk_s_rd;
		reg [NPHYS-1:0]  lk_s_lanes;
		reg              lk_s_wr;
		reg              lk_s_we;
		reg              lk_s_rq;
		reg              lk_s_last;
		reg [SBE_W:0]    lk_s_sbe;
		reg [2:0]        lk_s_cls;
		integer mk;
		always @(*) begin
			lk_s_warp  = {WARP_W{1'b0}};
			lk_s_grp   = {GRP_W{1'b0}};
			lk_s_rd    = 6'd0;
			lk_s_lanes = {NPHYS{1'b0}};
			lk_s_wr    = 1'b0;
			lk_s_we    = 1'b0;
			lk_s_rq    = 1'b0;
			lk_s_last  = 1'b0;
			lk_s_sbe   = {(SBE_W+1){1'b0}};
			lk_s_cls   = 3'd0;
			for (mk = 0; mk < NL; mk = mk + 1) begin
				if (lk_s_one[mk]) begin
					lk_s_warp  = lk_s_warp  | lk_warp[mk*WARP_W +: WARP_W];
					lk_s_grp   = lk_s_grp   | lk_grp[mk*GRP_W +: GRP_W];
					lk_s_rd    = lk_s_rd    | lk_rd[mk*7 +: 6];
					lk_s_lanes = lk_s_lanes | lk_lanes[mk*NPHYS +: NPHYS];
					lk_s_wr    = lk_s_wr    | lk_wr[mk];
					lk_s_we    = lk_s_we    | lk_we_in[mk];
					lk_s_rq    = lk_s_rq    | lk_rq_in[mk];
					lk_s_last  = lk_s_last  | lk_last[mk];
					lk_s_sbe   = lk_s_sbe   | lk_sbe[mk*(SBE_W+1) +: (SBE_W+1)];
					lk_s_cls   = lk_s_cls   | lk_cls[mk*3 +: 3];
				end
			end
		end

		/* 2-stage mandatory pipeline (lq1_* -> lq_*) cuts wb.rr -> wb (20 cmos3 -> <=10). */
		(* keep = "true", max_fanout = 16 *) reg lq1_valid;
		wire lq_in_rdy = !lq1_valid;
		reg [WARP_W-1:0] lq1_warp;
		reg [GRP_W-1:0]  lq1_grp;
		reg [5:0]        lq1_reg;
		reg [NPHYS-1:0]  lq1_lanes;
		reg              lq1_wr;
		reg              lq1_we;
		reg              lq1_rq;
		reg              lq1_last;
		reg [SBE_W:0]    lq1_sbe;
		reg [2:0]        lq1_cls;

		(* keep = "true", max_fanout = 12 *) reg lq_valid;
		(* max_fanout = 12 *) reg [WARP_W-1:0] lq_warp;
		(* max_fanout = 12 *) reg [GRP_W-1:0]  lq_grp;
		(* max_fanout = 12 *) reg [5:0]        lq_reg;
		(* max_fanout = 12 *) reg [NPHYS-1:0]  lq_lanes;
		(* keep = "true", max_fanout = 12 *) reg lq_wr;
		(* keep = "true", max_fanout = 12 *) reg lq_we;
		(* keep = "true", max_fanout = 12 *) reg lq_rq;
		(* max_fanout = 12 *) reg              lq_last;
		(* max_fanout = 12 *) reg [SBE_W:0]    lq_sbe;
		(* max_fanout = 12 *) reg [2:0]        lq_cls;

		wire lq_done    = !lq_wr || (!sv_arb_lq && !fq_valid);
		wire lq_head_we = lq_done || !lq_valid;
		wire lq_head_we_ctl, lq_head_we_d0, lq_head_we_d1, lq1_ctl_we0, lq1_ctl_we1, lq1_rr_we;
		wire [DW/16-1:0] lq1_d_we;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq_hwc (.I0(lq_head_we), .O(lq_head_we_ctl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq_hwd0 (.I0(lq_head_we), .O(lq_head_we_d0));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq_hwd1 (.I0(lq_head_we), .O(lq_head_we_d1));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq1_cwe0 (.I0(ksel_v && lq_in_rdy), .O(lq1_ctl_we0));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq1_cwe1 (.I0(ksel_v && lq_in_rdy), .O(lq1_ctl_we1));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq1_rwe (.I0(ksel_v && lq_in_rdy), .O(lq1_rr_we));
		genvar glqwe;
		for (glqwe = 0; glqwe < DW / 16; glqwe = glqwe + 1) begin : g_lq1_dwe
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lq1_dw (.I0(ksel_v && lq_in_rdy), .O(lq1_d_we[glqwe]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		assign lq_head_we_ctl = lq_head_we;
		assign lq_head_we_d0 = lq_head_we;
		assign lq_head_we_d1 = lq_head_we;
		assign lq1_ctl_we0 = ksel_v && lq_in_rdy;
		assign lq1_ctl_we1 = ksel_v && lq_in_rdy;
		assign lq1_rr_we = ksel_v && lq_in_rdy;
		genvar glqwe;
		for (glqwe = 0; glqwe < DW / 16; glqwe = glqwe + 1) begin : g_lq1_dwe
			assign lq1_d_we[glqwe] = ksel_v && lq_in_rdy;
		end
`else
		assign lq_head_we_ctl = lq_head_we;
		assign lq_head_we_d0  = lq_head_we;
		assign lq_head_we_d1  = lq_head_we;
		assign lq1_ctl_we0    = ksel_v && lq_in_rdy;
		assign lq1_ctl_we1    = ksel_v && lq_in_rdy;
		assign lq1_rr_we      = ksel_v && lq_in_rdy;
		assign lq1_d_we       = {(DW/16){ksel_v && lq_in_rdy}};
`endif

		wire [NL-1:0] lk_ready_pre = (ksel_v && lq_in_rdy) ? lk_s_one : {NL{1'b0}};
		genvar glkr;
		for (glkr = 0; glkr < NL; glkr = glkr + 1) begin : g_lkr
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_lkr (.I0(lk_ready_pre[glkr]), .O(lk_ready[glkr]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign lk_ready[glkr] = lk_ready_pre[glkr];
`else
			assign lk_ready[glkr] = lk_ready_pre[glkr];
`endif
		end

		localparam N_KS_GRP = (DW / 16 + 3) / 4;
		wire [NL-1:0] lk_one_grp [0:N_KS_GRP-1];
		genvar gkg;
		for (gkg = 0; gkg < N_KS_GRP; gkg = gkg + 1) begin : g_ksel_grp
			genvar gkb;
			for (gkb = 0; gkb < NL; gkb = gkb + 1) begin : g_ksel_grp_bit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_lk1h_g (.I0(lk_s_one[gkb]), .O(lk_one_grp[gkg][gkb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign lk_one_grp[gkg][gkb] = lk_s_one[gkb];
`else
				assign lk_one_grp[gkg][gkb] = lk_s_one[gkb];
`endif
			end
		end

		genvar glqd, glqh;
		for (glqd = 0; glqd < DW / 16; glqd = glqd + 1) begin : g_lqd_sl
			localparam integer LQ_SL = (glqd * 8) / (DW / 16);
			wire [NS-1:0] sv_lqd_raw = USE_SV_REP ?
				{s_valid_rep[5*8+LQ_SL], s_valid_rep[4*8+LQ_SL], s_valid_rep[3*8+LQ_SL],
				 s_valid_rep[2*8+LQ_SL], s_valid_rep[1*8+LQ_SL], s_valid_rep[0*8+LQ_SL]} :
				s_valid_sel;
			wire [NS-1:0] sv_lqd;
`ifdef SYNTH_KINTEX7
			genvar gsvb;
			for (gsvb = 0; gsvb < NS; gsvb = gsvb + 1) begin : g_sv_buf
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svl (.I0(sv_lqd_raw[gsvb]), .O(sv_lqd[gsvb]));
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			genvar gsvb;
			for (gsvb = 0; gsvb < NS; gsvb = gsvb + 1) begin : g_sv_buf
				assign sv_lqd[gsvb] = sv_lqd_raw[gsvb];
			end
`else
			assign sv_lqd = sv_lqd_raw;
`endif
			(* dont_touch = "true" *) wire lq_ld_s = !lq_valid || !lq_wr || (!sv && !fq_valid_sl[glqd]);
			(* keep = "true", max_fanout = 12 *) wire wb_sel1_w = !(sv_lqd[0] || sv_lqd[1] || sv_lqd[2] || sv_lqd[3]);
			(* keep = "true", max_fanout = 12 *) wire wb_sel0_w = !(sv_lqd[0] || sv_lqd[1]) && (sv_lqd[2] || sv_lqd[3] || !(sv_lqd[4] || sv_lqd[5]));
			(* keep = "true", max_fanout = 12 *) wire wb_sel1_r = !(sv_lqd[0] || sv_lqd[1] || sv_lqd[2] || sv_lqd[3]);
			(* keep = "true", max_fanout = 12 *) wire wb_sel0_r = !(sv_lqd[0] || sv_lqd[1]) && (sv_lqd[2] || sv_lqd[3] || !(sv_lqd[4] || sv_lqd[5]));

			wire [15:0] lk_s_data_sl;
			for (glqh = 0; glqh < 2; glqh = glqh + 1) begin : g_lqd_h
				wire [NL-1:0] lk_one_h;
				genvar ghb;
				for (ghb = 0; ghb < NL; ghb = ghb + 1) begin : g_lk1h_h_bit
`ifdef SYNTH_KINTEX7
					(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_lk1h_h (
						.I0(lk_one_grp[glqd / 4][ghb]),
						.O(lk_one_h[ghb])
					);
`elsif SYNTH_KINTEX_ULTRASCALE
					assign lk_one_h[ghb] = lk_one_grp[glqd / 4][ghb];
`else
					assign lk_one_h[ghb] = lk_one_grp[glqd / 4][ghb];
`endif
				end

				reg [7:0] d_h;
				integer imh;
				always @(*) begin
					d_h = 8'd0;
					for (imh = 0; imh < NL; imh = imh + 1)
						if (lk_one_h[imh])
							d_h = d_h | lk_data[imh*DW + glqd*16 + glqh*8 +: 8];
				end
				assign lk_s_data_sl[glqh*8 +: 8] = d_h;
			end

			(* shreg_extract = "no", max_fanout = 12 *) reg [15:0] lq_d_sl;
			(* shreg_extract = "no" *) reg [15:0] lq1_d_sl;
			always @(posedge clk) begin
				if (lq1_d_we[glqd])
					lq1_d_sl <= lk_s_data_sl;
				if (lq_ld_s && lq1_valid)
					lq_d_sl <= lq1_d_sl;
			end
			(* dont_touch = "true" *) wire [15:0] sd_fl_s = fq_valid_sl[glqd] ? fq_data[glqd*16 +: 16] : lq_d_sl;
			assign w_data[glqd*16 +: 16] = wb_sel1_w ? (wb_sel0_w ? sd_fl_s : g_sd_sl[glqd].sd_45_s) :
				(wb_sel0_w ? g_sd_sl[glqd].sd_23_s : g_sd_sl[glqd].sd_01_s);
			assign rq_data[glqd*16 +: 16] = wb_sel1_r ? (wb_sel0_r ? lq_d_sl : g_sd_sl[glqd].sd_45_s) :
				(wb_sel0_r ? g_sd_sl[glqd].sd_23_s : g_sd_sl[glqd].sd_01_s);
		end

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				lq1_valid     <= 1'b0;
				lq1_warp      <= {WARP_W{1'b0}};
				lq1_grp       <= {GRP_W{1'b0}};
				lq1_reg       <= 6'd0;
				lq1_lanes     <= {NPHYS{1'b0}};
				lq1_wr        <= 1'b0;
				lq1_we        <= 1'b0;
				lq1_rq        <= 1'b0;
				lq1_last      <= 1'b0;
				lq1_sbe       <= {(SBE_W+1){1'b0}};
				lq1_cls       <= 3'd0;
				lq_valid      <= 1'b0;
				lq_warp       <= {WARP_W{1'b0}};
				lq_grp        <= {GRP_W{1'b0}};
				lq_reg        <= 6'd0;
				lq_lanes      <= {NPHYS{1'b0}};
				lq_wr         <= 1'b0;
				lq_we         <= 1'b0;
				lq_rq         <= 1'b0;
				lq_last       <= 1'b0;
				lq_sbe        <= {(SBE_W+1){1'b0}};
				lq_cls        <= 3'd0;
			end else begin
				if (lq1_ctl_we0) begin
					lq1_valid <= 1'b1;
					lq1_warp  <= lk_s_warp;
					lq1_grp   <= lk_s_grp;
					lq1_reg   <= lk_s_rd[5:0];
					lq1_wr    <= lk_s_wr;
					lq1_we    <= lk_s_we;
					lq1_rq    <= lk_s_rq;
					lq1_last  <= lk_s_last;
					lq1_sbe   <= lk_s_sbe;
					lq1_cls   <= lk_s_cls;
				end else if (lq_head_we_ctl && lq1_valid) begin
					lq1_valid <= 1'b0;
				end
				if (lq1_ctl_we1)
					lq1_lanes <= lk_s_lanes;
				if (lq_head_we_d0) begin
					lq_valid <= lq1_valid;
					lq_warp  <= lq1_warp;
					lq_grp   <= lq1_grp;
					lq_reg   <= lq1_reg;
					lq_wr    <= lq1_wr;
					lq_we    <= lq1_we;
					lq_rq    <= lq1_rq;
					lq_last  <= lq1_last;
					lq_sbe   <= lq1_sbe;
					lq_cls   <= lq1_cls;
				end
				if (lq_head_we_d1)
					lq_lanes <= lq1_lanes;
			end
		end

		wire [NL-1:0] next_rr_ge;
		assign next_rr_ge[0] = lk_s_one[NL - 1];
		for (g_lk = 1; g_lk < NL; g_lk = g_lk + 1) begin : g_nrr_ge
			assign next_rr_ge[g_lk] = lk_s_one[NL - 1] || (|lk_s_one[g_lk-1:0]);
		end

		wire [2:0] wait_n_p1;
		am_add #(.W(3)) u_add_wait (
			.a(wait_n),
			.b(3'd1),
			.ci(1'b0),
			.s(wait_n_p1),
			.co()
		);
		wire [2:0] wait_n_next = (&wait_n) ? 3'd7 : wait_n_p1;

		wire l_stalled = lq_valid && lq_wr && (sv_arb_lq || fq_valid);
		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				rr_ge_r <= {NL{1'b1}};
				wait_n  <= 3'd0;
			end else begin
				if (lq1_rr_we)
					rr_ge_r <= next_rr_ge;
				if (l_stalled)
					wait_n <= wait_n_next;
				else
					wait_n <= 3'd0;
			end
		end

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				r_rel_valid <= 1'b0;
				r_rel_warp  <= {WARP_W{1'b0}};
				r_rel_sbe   <= {(SBE_W+1){1'b0}};
				r_rel_cls   <= 3'd0;
				r_wb_stall  <= 1'b0;
			end else begin
				r_rel_valid <= lq_valid && lq_done && lq_last;
				r_rel_warp  <= lq_warp;
				r_rel_sbe   <= lq_sbe;
				r_rel_cls   <= lq_cls;
				r_wb_stall  <= (wait_n > 3'd4) || (|fi_valid) || fq_valid;
			end
		end

		wire sv_ctl, sv_w_m, sv_rq;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svctl (.I0(sv), .O(sv_ctl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svwm  (.I0(sv), .O(sv_w_m));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svrq  (.I0(sv), .O(sv_rq));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sv_ctl = sv;
		assign sv_w_m = sv;
		assign sv_rq = sv;
`else
		assign sv_ctl = sv;
		assign sv_w_m = sv;
		assign sv_rq  = sv;
`endif
		wire s_rq = sv_rq && !s_we_pre;
		wire non_s_we = fq_valid ? 1'b1 : (lq_valid && lq_we);

		assign w_en    = s_we_012 || s_we_345 || (!sv_ctl && non_s_we);
		assign w_warp  = sv_w_m ? sw       : (fq_valid ? fq_warp  : lq_warp);
		assign w_grp   = sv_w_m ? sg       : (fq_valid ? fq_grp   : lq_grp);
		assign w_reg   = sv_w_m ? srd[5:0] : (fq_valid ? fq_reg   : lq_reg);
		assign w_lanes = sv_w_m ? sl       : (fq_valid ? fq_lanes : lq_lanes);
		assign rq_push = sv_rq  ? s_rq     : (!fq_valid && lq_valid && lq_rq);
	end else begin : g_comb_long
		(* max_fanout = 16 *) reg [LP_W-1:0] rr;
		wire port_free = !sv_arb && !(|fi_valid);

		reg lsel_free_v, lsel_free_hv;
		reg [LP_W-1:0] lsel_free, lsel_free_lo;
		reg lsel_busy_v, lsel_busy_hv;
		reg [LP_W-1:0] lsel_busy, lsel_busy_lo;
		integer k;
		always @(*) begin
			lsel_free_v = 1'b0;
			lsel_free_hv = 1'b0;
			lsel_free = {LP_W{1'b0}};
			lsel_free_lo = {LP_W{1'b0}};

			lsel_busy_v = 1'b0;
			lsel_busy_hv = 1'b0;
			lsel_busy = {LP_W{1'b0}};
			lsel_busy_lo = {LP_W{1'b0}};

			for (k = NL - 1; k >= 0; k = k - 1) begin
				if (l_valid[k]) begin
					lsel_free_v = 1'b1;
					lsel_free_lo = k[LP_W-1:0];
					if (k[LP_W-1:0] >= rr) begin
						lsel_free_hv = 1'b1;
						lsel_free = k[LP_W-1:0];
					end
				end
				if (l_valid[k] && !l_wr[k]) begin
					lsel_busy_v = 1'b1;
					lsel_busy_lo = k[LP_W-1:0];
					if (k[LP_W-1:0] >= rr) begin
						lsel_busy_hv = 1'b1;
						lsel_busy = k[LP_W-1:0];
					end
				end
			end
			if (!lsel_free_hv)
				lsel_free = lsel_free_lo;
			if (!lsel_busy_hv)
				lsel_busy = lsel_busy_lo;
		end

		wire lsel_v = port_free ? lsel_free_v : lsel_busy_v;
		wire [LP_W-1:0] lsel = port_free ? lsel_free : lsel_busy;

		reg [WARP_W-1:0] l_s_warp;
		reg [SBE_W:0] l_s_sbe;
		reg [2:0] l_s_cls;
		reg [NL-1:0] l_s_one;
		integer m;
		always @(*) begin
			l_s_warp = l_warp[WARP_W-1:0];
			l_s_sbe = l_sbe[SBE_W:0];
			l_s_cls = l_cls[2:0];
			for (m = 0; m < NL; m = m + 1) begin
				l_s_one[m] = lsel == m[LP_W-1:0];
				if (m > 0 && l_s_one[m]) begin
					l_s_warp = l_warp[m*WARP_W +: WARP_W];
					l_s_sbe = l_sbe[m*(SBE_W+1) +: SBE_W+1];
					l_s_cls = l_cls[m*3 +: 3];
				end
			end
		end
		assign l_ready = lsel_v ? l_s_one : {NL{1'b0}};

		reg [6:0] lrd_free;
		reg [WARP_W-1:0] l_s_warp_free;
		reg [GRP_W-1:0]  l_s_grp_free;
		reg [DW-1:0]     l_s_data_free;
		reg [NPHYS-1:0]  l_s_lanes_free;
		integer mf;
		always @(*) begin
			lrd_free = l_rd[6:0];
			l_s_warp_free = l_warp[WARP_W-1:0];
			l_s_grp_free = l_grp[GRP_W-1:0];
			l_s_data_free = l_data[DW-1:0];
			l_s_lanes_free = l_lanes[NPHYS-1:0];
			for (mf = 1; mf < NL; mf = mf + 1) begin
				if (lsel_free == mf[LP_W-1:0]) begin
					lrd_free = l_rd[mf*7 +: 7];
					l_s_warp_free = l_warp[mf*WARP_W +: WARP_W];
					l_s_grp_free = l_grp[mf*GRP_W +: GRP_W];
					l_s_data_free = l_data[mf*DW +: DW];
					l_s_lanes_free = l_lanes[mf*NPHYS +: NPHYS];
				end
			end
		end

		wire in_go = !sv && (|fi_valid);
		assign fi_ready = in_go ? (in_src ? 2'b10 : 2'b01) : 2'b00;

		wire fi_has = |fi_valid;
		wire lw_free = lsel_free_v && l_wr[lsel_free];
		wire s_rq = sv && !s_we_pre;
		wire l_rq_free = lw_free && (lrd_free == 7'd63);

		wire sv_ctl, sv_w0, sv_w1;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svctl (.I0(sv), .O(sv_ctl));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svw0  (.I0(sv), .O(sv_w0));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_svw1  (.I0(sv), .O(sv_w1));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign sv_ctl = sv;
		assign sv_w0 = sv;
		assign sv_w1 = sv;
`else
		assign sv_ctl = sv;
		assign sv_w0  = sv;
		assign sv_w1  = sv;
`endif
		wire non_s_we = fi_has ? 1'b1 : (lw_free && (lrd_free != 7'd63));
		assign w_en = s_we_012 || s_we_345 || (!sv_ctl && non_s_we);

		assign w_warp  = sv_w0  ? sw       : (fi_has ? fe[IW-1 -: WARP_W] : l_s_warp_free);
		assign w_grp   = sv_w0  ? sg       : (fi_has ? fe_grp[GRP_W-1:0]  : l_s_grp_free);
		assign w_reg   = sv_w0  ? srd[5:0] : (fi_has ? fe[42:37]          : lrd_free[5:0]);
		assign w_data  = sv ? sd           : (fi_has ? fe_data            : l_s_data_free);
		assign w_lanes = sv_w1  ? sl       : (fi_has ? fe_lanes           : l_s_lanes_free);
		assign rq_push = sv_ctl ? s_rq     : (!fi_has && l_rq_free);
		assign rq_data = sv_ctl ? sd       : l_s_data_free;

		wire [LP_W-1:0] lsel_p1;
		am_add #(.W(LP_W)) u_add_lsel (
			.a(lsel),
			.b({{(LP_W-1){1'b0}}, 1'b1}),
			.ci(1'b0),
			.s(lsel_p1),
			.co()
		);
		wire [LP_W-1:0] rr_l_next = ({{(32-LP_W){1'b0}}, lsel} == NL - 1) ? {LP_W{1'b0}} : lsel_p1;

		wire [2:0] wait_nl_p1;
		am_add #(.W(3)) u_add_waitl (
			.a(wait_n),
			.b(3'd1),
			.ci(1'b0),
			.s(wait_nl_p1),
			.co()
		);
		wire [2:0] wait_nl_next = (&wait_n) ? 3'd7 : wait_nl_p1;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				rr <= {LP_W{1'b0}};
				wait_n <= 3'd0;
			end else begin
				if (lsel_v)
					rr <= rr_l_next;
				if (|(l_valid & ~l_ready))
					wait_n <= wait_nl_next;
				else
					wait_n <= 3'd0;
			end
		end

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				r_rel_valid <= 1'b0;
				r_rel_warp  <= {WARP_W{1'b0}};
				r_rel_sbe   <= {(SBE_W+1){1'b0}};
				r_rel_cls   <= 3'd0;
				r_wb_stall  <= 1'b0;
			end else begin
				r_rel_valid <= lsel_v && l_last[lsel];
				r_rel_warp  <= l_s_warp;
				r_rel_sbe   <= l_s_sbe;
				r_rel_cls   <= l_s_cls;
				r_wb_stall  <= wait_n > 3'd4 || (|fi_valid);
			end
		end
	end
	endgenerate

	assign rel_valid = r_rel_valid;
	assign rel_warp = r_rel_warp;
	assign rel_sbe = r_rel_sbe;
	assign rel_cls = r_rel_cls;
	assign wb_stall = r_wb_stall;

`ifndef SYNTHESIS
	/* Two short results in one cycle cannot happen (fixed latency, one issue). */
	reg s_any, s_two;
	integer si;
	always @(*) begin
		s_any = 1'b0;
		s_two = 1'b0;
		for (si = 0; si < NS; si = si + 1)
			if (s_valid[si]) begin
				s_two = s_two || s_any;
				s_any = 1'b1;
			end
	end
	/* verilator lint_off SYNCASYNCNET */
	always @(posedge clk)
		if (rst_n && s_two)
			$display("ERROR atg_su_wb: two short results in one cycle (%b)", s_valid);
	/* verilator lint_on SYNCASYNCNET */
`endif

	wire unused_ok = &{1'b0, fe_grp, (G != 0)};
endmodule

`default_nettype wire
