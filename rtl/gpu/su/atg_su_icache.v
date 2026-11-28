/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_icache: The SU's instruction cache: 2 KiB direct-mapped (256
 * instructions), lines of LINE_BYTES from the AMB, invalidated by
 * LOAD_PROGRAM / INVALIDATE[3].
 *
 * The behaviour, and where it differs from
 * the design: spec/gpu/su/atg_su_icache.md (§10 Status).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_icache #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter IC_BYTES = 2048	/* Size in bytes */
) (
	input wire clk,
	input wire rst_n,
	input wire ic_req,
	output wire ic_ready,
	input wire [31:0] ic_addr,
	input wire [7:0] ic_tag,
	output wire ic_rvalid,
	output wire [63:0] ic_rdata,
	input wire inv,
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
	localparam LB = LINE_BYTES * 8;
	localparam NLINES = IC_BYTES / LINE_BYTES;
	localparam IX_W = $clog2(NLINES);
	localparam OFF_W = $clog2(IC_BYTES);
	localparam TAG_W = ADDR_W - OFF_W;
	localparam WPL = LINE_BYTES / 8;	/* instructions per line */
	localparam WS_W = WPL > 1 ? $clog2(WPL) : 1;
	localparam [1:0] S_IDLE = 2'd0, S_MREQ = 2'd1, S_MWAIT = 2'd2, S_RESP = 2'd3;

	localparam TAG_LO = TAG_W / 2;
	localparam TAG_HI = TAG_W - TAG_LO;
	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam N_SL = LB / 16;
	localparam N_DSL = LB / 64;
	localparam V_SL = (NLINES >= 16) ? (NLINES / 16) : 1;
	localparam V_SW = NLINES / V_SL;
	localparam N_LEQ = (LA_W + 2) / 3;

	wire rst_p = !rst_n;
	(* max_fanout = 12 *) reg [1:0] state;
	(* dont_touch = "true" *) reg [3:0] st_idle_m_r;
	(* dont_touch = "true" *) reg st_mwait_tlo_r, st_mwait_thi_r;
	reg miss_pend_r;
	wire [NLINES-1:0] valid;
	(* max_fanout = 12 *) reg [ADDR_W-LINE_SHIFT-1:0] mline;	/* the line being filled */
	(* dont_touch = "true" *) reg [IX_W-1:0] m_idx_d_r;
	wire [LB-1:0] fill;
	(* max_fanout = 12 *) reg inv_seen;

	wire rsp_v_ctl, rsp_v_d;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rv_ctl (.I0(rsp_valid), .O(rsp_v_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_rv_d   (.I0(rsp_valid), .O(rsp_v_d));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign rsp_v_ctl = rsp_valid;
	assign rsp_v_d = rsp_valid;
`else
	assign rsp_v_ctl = rsp_valid;
	assign rsp_v_d   = rsp_valid;
`endif
	wire st_mwait_nxt = (state == S_MREQ) ? req_ready : ((state == S_MWAIT) ? !(rsp_v_ctl && !rsp_write) : 1'b0);
	wire st_mwait_nxt_d, st_mwait_nxt_f;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_mwn_d (.I0(st_mwait_nxt), .O(st_mwait_nxt_d));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_mwn_f (.I0(st_mwait_nxt), .O(st_mwait_nxt_f));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign st_mwait_nxt_d = st_mwait_nxt;
	assign st_mwait_nxt_f = st_mwait_nxt;
`else
	assign st_mwait_nxt_d = st_mwait_nxt;
	assign st_mwait_nxt_f = st_mwait_nxt;
`endif
	(* dont_touch = "true" *) reg [N_SL-1:0] st_mwait_f_r;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p)
			st_mwait_f_r <= {N_SL{1'b0}};
		else
			st_mwait_f_r <= {N_SL{st_mwait_nxt_f}};
	end

	wire [63:0] ic_addr_tag = {32'd0, ic_addr};
	wire [31:0] ic_addr_ram_lo = ic_addr;
	wire [31:0] ic_addr_ram_hi = ic_addr;

	wire [IX_W-1:0] idx_v  = ic_addr_tag[OFF_W-1:LINE_SHIFT];
	wire [TAG_W-1:0] tag = ic_addr_tag[ADDR_W-1:OFF_W];
	wire [ADDR_W-LINE_SHIFT-1:0] line = ic_addr_tag[ADDR_W-1:LINE_SHIFT];
	wire [WS_W-1:0] wsel = WPL > 1 ? ic_addr_tag[LINE_SHIFT-1:3] : {WS_W{1'b0}};
	wire [NLINES-1:0] line_hit;
	localparam integer N_PAIRS = (NLINES + 1) / 2;
	reg [N_PAIRS-1:0] pair_hit_r;
	reg [LA_W-1:0] last_line_r, cam_line_r1, cam_line_r2;
	reg last_hit_r, cam_v1_r, cam_v2_r, cam_hit_r, fill_match_r;

	wire fast_hit = last_hit_r && (line == last_line_r);
	wire cam_hit  = cam_v2_r && cam_hit_r && (line == cam_line_r2);
	wire hit_idle = st_idle_m_r[0] && (fast_hit || cam_hit);
	/* Pre-register (line == mline) in S_MWAIT to cut CARRY8 comparison from ic_ready */
	wire from_fill = (state == S_RESP) && fill_match_r;
	assign ic_ready = hit_idle || from_fill;
	wire take = ic_req && (hit_idle || from_fill);

	/* The answer, 2 cycles after the request (BRAM rline + dedicated fabric rline_pipe, Rule 4). */
	(* shreg_extract = "no", max_fanout = 12 *) reg rv, rv_pipe, rfill, rfill_pipe;
	(* shreg_extract = "no", max_fanout = 12 *) reg [WS_W-1:0] rsel, rsel_pipe;
	wire [LB-1:0] rline_pipe;
	wire [LB-1:0] fill_pipe;
	assign ic_rvalid = rv_pipe;
	wire [LB-1:0] src;

	wire inv_ctl, inv_d0, inv_d1, inv_v0, inv_v1;
	wire last_line_we_lo, last_line_we_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_inv_ctl (.I0(inv), .O(inv_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_inv_d0  (.I0(inv), .O(inv_d0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_inv_d1  (.I0(inv), .O(inv_d1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_inv_v0  (.I0(inv), .O(inv_v0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_inv_v1  (.I0(inv), .O(inv_v1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_llwlo   (.I0((state == S_RESP) && !inv_seen && !inv_ctl), .O(last_line_we_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_llwhi   (.I0((state == S_RESP) && !inv_seen && !inv_ctl), .O(last_line_we_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign inv_ctl = inv;
	assign inv_d0 = inv;
	assign inv_d1 = inv;
	assign inv_v0 = inv;
	assign inv_v1 = inv;
	assign last_line_we_lo = (state == S_RESP) && !inv_seen && !inv_ctl;
	assign last_line_we_hi = (state == S_RESP) && !inv_seen && !inv_ctl;
`else
	assign inv_ctl = inv;
	assign inv_d0  = inv;
	assign inv_d1  = inv;
	assign inv_v0  = inv;
	assign inv_v1  = inv;
	assign last_line_we_lo = (state == S_RESP) && !inv_seen && !inv_ctl;
	assign last_line_we_hi = (state == S_RESP) && !inv_seen && !inv_ctl;
`endif
	genvar gdsl, gsl, grs, gvs, gvi;
	generate
		for (gdsl = 0; gdsl < N_DSL; gdsl = gdsl + 1) begin : g_data_sl
			(* ram_style = "block" *) reg [63:0] data_sl [0:NLINES-1];
			reg [63:0] rline_sl;
			(* dont_touch = "true" *) reg [63:0] rline_pipe_sl;
			(* dont_touch = "true" *) reg st_mwait_d_sl;
			(* dont_touch = "true" *) reg fill_wr_pend_d_sl;
			(* dont_touch = "true" *) reg [IX_W-1:0] m_idx_d_sl;
			wire data_we_next_sl = st_mwait_d_sl && rsp_valid && !rsp_write && !rsp_error && !inv_seen && !((gdsl < (N_DSL / 2)) ? inv_d0 : inv_d1);
			/* Speculative BRAM read gated only by (ic_req && st_idle_m_r) so tag compare does not feed ENARDEN (Rule 4); rv<=take qualifies use downstream. */
			wire data_re_sl = ic_req && st_idle_m_r[gdsl % 4];
			wire [31:0] ic_addr_ram = (gdsl < 4) ? ic_addr_ram_lo : ic_addr_ram_hi;
			always @(posedge clk) begin
				if (fill_wr_pend_d_sl)
					data_sl[m_idx_d_sl] <= fill[gdsl*64 +: 64];
				if (data_re_sl)
					rline_sl <= data_sl[ic_addr_ram[OFF_W-1:LINE_SHIFT]];
				rline_pipe_sl <= rline_sl;
			end
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					st_mwait_d_sl <= 1'b0;
					fill_wr_pend_d_sl <= 1'b0;
					m_idx_d_sl <= {IX_W{1'b0}};
				end else begin
					/* Track requested index throughout S_IDLE so CE is a pure FF (st_idle_m_r clears on miss). */
					if (st_idle_m_r[gdsl % 4])
						m_idx_d_sl <= ic_addr_ram[OFF_W-1:LINE_SHIFT];
					st_mwait_d_sl <= st_mwait_nxt_d;
					fill_wr_pend_d_sl <= data_we_next_sl;
				end
			end
			assign rline_pipe[gdsl*64 +: 64] = rline_pipe_sl;
		end
		for (gsl = 0; gsl < N_SL; gsl = gsl + 1) begin : g_ic_sl
			(* shreg_extract = "no" *) reg [15:0] fill_sl, fill_pipe_sl;
			(* dont_touch = "true" *) reg rfill_sl;
			wire fill_we_sl_raw = st_mwait_f_r[gsl] && rsp_v_d && !rsp_write;
			wire fill_we_sl;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fwe (.I0(fill_we_sl_raw), .O(fill_we_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign fill_we_sl = fill_we_sl_raw;
`else
			assign fill_we_sl = fill_we_sl_raw;
`endif
			always @(posedge clk) begin
				if (fill_we_sl)
					fill_sl <= rsp_error ? 16'd0 : rsp_rdata[gsl*16 +: 16];
				fill_pipe_sl <= fill_sl;
			end
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					rfill_sl <= 1'b0;
				end else begin
					rfill_sl <= rfill;
				end
			end
			assign fill[gsl*16 +: 16] = fill_sl;
			assign fill_pipe[gsl*16 +: 16] = fill_pipe_sl;
			assign src[gsl*16 +: 16] = rfill_sl ? fill_pipe_sl : rline_pipe[gsl*16 +: 16];
		end
		for (grs = 0; grs < 8; grs = grs + 1) begin : g_rdata_sl
			(* dont_touch = "true" *) reg [WS_W-1:0] rsel_sl;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					rsel_sl <= {WS_W{1'b0}};
				else
					rsel_sl <= rsel;
			end
			wire [63:0] src_w = src[rsel_sl*64 +: 64];
			assign ic_rdata[grs*8 +: 8] = src_w[grs*8 +: 8];
		end
		for (gvs = 0; gvs < V_SL; gvs = gvs + 1) begin : g_val_sl
			wire inv_v_sl;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ivsl (.I0((gvs < (V_SL / 2)) ? inv_v0 : inv_v1), .O(inv_v_sl));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign inv_v_sl = (gvs < (V_SL / 2)) ? inv_v0 : inv_v1;
`else
			assign inv_v_sl = inv;
`endif
			wire [35:0] m_idx_pad = {{(36-IX_W){1'b0}}, mline[IX_W-1:0]};
			(* keep = "true" *) wire v_set_sl = (state == S_MWAIT) && rsp_v_ctl && !rsp_write && !rsp_error && !inv_seen &&
				(m_idx_pad[35:4] == gvs);
			for (gvi = 0; gvi < V_SW; gvi = gvi + 1) begin : g_val_bit
				reg valid_bit;
				always @(posedge clk or posedge rst_p) begin
					if (rst_p)
						valid_bit <= 1'b0;
					else if (inv_v_sl)
						valid_bit <= 1'b0;
					else if (v_set_sl && (m_idx_pad[3:0] == gvi[3:0]))
						valid_bit <= 1'b1;
				end
				assign valid[gvs*V_SW + gvi] = valid_bit;
			end
		end
	endgenerate

	(* keep = "true" *) wire [IX_W-1:0] m_idx_lo = mline[IX_W-1:0];
	(* keep = "true" *) wire [IX_W-1:0] m_idx_hi = mline[IX_W-1:0];
	(* keep = "true", dont_touch = "true" *) reg fill_wr_pend_tlo, fill_wr_pend_thi;
	wire tag_we_lo_next = st_mwait_tlo_r && rsp_valid && !rsp_write && !rsp_error && !inv_seen && !inv_ctl;
	wire tag_we_hi_next = st_mwait_thi_r && rsp_valid && !rsp_write && !rsp_error && !inv_seen && !inv_ctl;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			fill_wr_pend_tlo <= 1'b0;
			fill_wr_pend_thi <= 1'b0;
		end else begin
			fill_wr_pend_tlo <= tag_we_lo_next;
			fill_wr_pend_thi <= tag_we_hi_next;
		end
	end
	localparam N_IC_GRP = (NLINES + 7) / 8;
	wire [IX_W-1:0]  m_idx_lo_g [0:N_IC_GRP-1];
	wire [IX_W-1:0]  m_idx_hi_g [0:N_IC_GRP-1];
	wire [IX_W-1:0]  idx_v_g    [0:N_IC_GRP-1];
	wire [TAG_W-1:0] tag_g      [0:N_IC_GRP-1];
	wire [TAG_W-1:0] mline_g    [0:N_IC_GRP-1];
	wire [N_IC_GRP-1:0] inv_g, tlo_we_g, thi_we_g;
	genvar gln, gicg, gicb;
	generate
		for (gicg = 0; gicg < N_IC_GRP; gicg = gicg + 1) begin : g_ic_mbuf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_invg (.I0(inv_ctl),          .O(inv_g[gicg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tlow (.I0(fill_wr_pend_tlo), .O(tlo_we_g[gicg]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_thiw (.I0(fill_wr_pend_thi), .O(thi_we_g[gicg]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign inv_g[gicg] = inv_ctl;
			assign tlo_we_g[gicg] = fill_wr_pend_tlo;
			assign thi_we_g[gicg] = fill_wr_pend_thi;
`else
			assign inv_g[gicg]    = inv;
			assign tlo_we_g[gicg] = fill_wr_pend_tlo;
			assign thi_we_g[gicg] = fill_wr_pend_thi;
`endif
			for (gicb = 0; gicb < IX_W; gicb = gicb + 1) begin : g_bit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mlo (.I0(m_idx_lo[gicb]), .O(m_idx_lo_g[gicg][gicb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mhi (.I0(m_idx_hi[gicb]), .O(m_idx_hi_g[gicg][gicb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_iv  (.I0(idx_v[gicb]),    .O(idx_v_g[gicg][gicb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign m_idx_lo_g[gicg][gicb] = m_idx_lo[gicb];
				assign m_idx_hi_g[gicg][gicb] = m_idx_hi[gicb];
				assign idx_v_g[gicg][gicb] = idx_v[gicb];
`else
				assign m_idx_lo_g[gicg][gicb] = m_idx_lo[gicb];
				assign m_idx_hi_g[gicg][gicb] = m_idx_hi[gicb];
				assign idx_v_g[gicg][gicb]    = idx_v[gicb];
`endif
			end
			for (gicb = 0; gicb < TAG_W; gicb = gicb + 1) begin : g_tbit
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tg  (.I0(tag[gicb]),           .O(tag_g[gicg][gicb]));
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_mlg (.I0(mline[IX_W + gicb]),  .O(mline_g[gicg][gicb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign tag_g[gicg][gicb] = tag[gicb];
				assign mline_g[gicg][gicb] = mline[IX_W + gicb];
`else
				assign tag_g[gicg][gicb]      = tag[gicb];
				assign mline_g[gicg][gicb]    = mline[IX_W + gicb];
`endif
			end
		end
		for (gln = 0; gln < NLINES; gln = gln + 1) begin : g_cam
			reg [TAG_LO-1:0] tlo_r;
			reg [TAG_HI-1:0] thi_r;
			always @(posedge clk) begin
				if (tlo_we_g[gln / 8] && (m_idx_lo_g[gln / 8] == gln[IX_W-1:0]))
					tlo_r <= mline_g[gln / 8][0 +: TAG_LO];
				if (thi_we_g[gln / 8] && (m_idx_hi_g[gln / 8] == gln[IX_W-1:0]))
					thi_r <= mline_g[gln / 8][TAG_LO +: TAG_HI];
			end
			assign line_hit[gln] = valid[gln] && (idx_v_g[gln / 8] == gln[IX_W-1:0]) &&
				({thi_r, tlo_r} == tag_g[gln / 8]);
		end
	endgenerate

	reg [7:0] m_tag;
	assign req_valid = state == S_MREQ;
	assign req_write = 1'b0;
	assign req_line = mline;
	assign req_be = {LINE_BYTES{1'b0}};
	assign req_wdata = {LB{1'b0}};
	assign req_tag = m_tag;

	wire cam_miss = st_idle_m_r[0] && !inv_ctl && ic_req && cam_v2_r && !cam_hit_r && (line == cam_line_r2);
	wire miss_fire = cam_miss;
	wire miss_0, miss_1, miss_2, miss_3, miss_4, miss_5;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m0 (.I0(miss_fire), .O(miss_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m1 (.I0(miss_fire), .O(miss_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m2 (.I0(miss_fire), .O(miss_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m3 (.I0(miss_fire), .O(miss_3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m4 (.I0(miss_fire), .O(miss_4));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_m5 (.I0(miss_fire), .O(miss_5));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign miss_0 = miss_fire;
	assign miss_1 = miss_fire;
	assign miss_2 = miss_fire;
	assign miss_3 = miss_fire;
	assign miss_4 = miss_fire;
	assign miss_5 = miss_fire;
`else
	assign miss_0 = miss_fire;
	assign miss_1 = miss_fire;
	assign miss_2 = miss_fire;
	assign miss_3 = miss_fire;
	assign miss_4 = miss_fire;
	assign miss_5 = miss_fire;
`endif

	integer ip_i;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			state <= S_IDLE;
			st_idle_m_r <= 4'b1111;
			st_mwait_tlo_r <= 1'b0;
			st_mwait_thi_r <= 1'b0;
			miss_pend_r <= 1'b0;
			pair_hit_r <= {N_PAIRS{1'b0}};
			last_line_r <= {LA_W{1'b0}};
			cam_line_r1 <= {LA_W{1'b0}};
			cam_line_r2 <= {LA_W{1'b0}};
			last_hit_r <= 1'b0;
			cam_v1_r <= 1'b0;
			cam_v2_r <= 1'b0;
			cam_hit_r <= 1'b0;
			fill_match_r <= 1'b0;
			rv <= 1'b0;
			rv_pipe <= 1'b0;
			rfill <= 1'b0;
			rfill_pipe <= 1'b0;
			rsel <= {WS_W{1'b0}};
			rsel_pipe <= {WS_W{1'b0}};
			inv_seen <= 1'b0;
			mline <= {(ADDR_W-LINE_SHIFT){1'b0}};
			m_idx_d_r <= {IX_W{1'b0}};
			m_tag <= 8'd0;
		end else begin
			for (ip_i = 0; ip_i < N_PAIRS; ip_i = ip_i + 1) begin
				if (ip_i * 2 + 1 < NLINES)
					pair_hit_r[ip_i] <= line_hit[ip_i*2] || line_hit[ip_i*2 + 1];
				else
					pair_hit_r[ip_i] <= line_hit[ip_i*2];
			end
			cam_line_r1 <= line;
			cam_v1_r <= !inv_ctl && st_idle_m_r[0] && ic_req;
			cam_hit_r <= !inv_ctl && (|pair_hit_r);
			cam_line_r2 <= cam_line_r1;
			cam_v2_r <= !inv_ctl && st_idle_m_r[0] && cam_v1_r;

			rv <= take;
			rv_pipe <= rv;
			rfill_pipe <= rfill;
			rsel_pipe <= rsel;
			if (take) begin
				rfill <= from_fill;
				rsel <= wsel;
			end
			if (st_idle_m_r[0] && cam_v2_r && cam_hit_r && !inv_ctl) begin
				last_line_r[13:0]      <= cam_line_r2[13:0];
				last_line_r[LA_W-1:14] <= cam_line_r2[LA_W-1:14];
				last_hit_r <= 1'b1;
			end
			if (miss_0)
				mline[8:0] <= line[8:0];
			if (miss_1)
				mline[17:9] <= line[17:9];
			if (miss_2)
				mline[LA_W-1:18] <= line[LA_W-1:18];
			if (miss_3)
				m_idx_d_r <= line[IX_W-1:0];
			if (miss_4)
				m_tag <= ic_tag;
			case (state)
			S_IDLE: begin
				miss_pend_r <= 1'b0;
				fill_match_r <= 1'b0;
				if (miss_5) begin
					last_hit_r <= 1'b0;
					cam_v1_r <= 1'b0;
					cam_v2_r <= 1'b0;
					state <= S_MREQ;
					st_idle_m_r <= 4'b0000;
					inv_seen <= 1'b0;
				end
			end
			S_MREQ: begin
				fill_match_r <= 1'b0;
				if (req_ready) begin
					state <= S_MWAIT;
					st_mwait_tlo_r <= 1'b1;
					st_mwait_thi_r <= 1'b1;
				end
			end
			S_MWAIT: begin
				fill_match_r <= ic_req && (line == mline);
				if (rsp_v_ctl && !rsp_write) begin
					state <= S_RESP;
					st_mwait_tlo_r <= 1'b0;
					st_mwait_thi_r <= 1'b0;
				end
			end
			default: begin	/* S_RESP: one cycle for the waiting request */
				state <= S_IDLE;
				fill_match_r <= 1'b0;
				st_idle_m_r <= 4'b1111;
				if (!inv_seen && !inv_ctl) begin
					last_hit_r <= 1'b1;
				end
			end
			endcase
			if (last_line_we_lo) last_line_r[13:0]      <= mline[13:0];
			if (last_line_we_hi) last_line_r[LA_W-1:14] <= mline[LA_W-1:14];
			if (inv_ctl) begin
				inv_seen <= 1'b1;
				last_hit_r <= 1'b0;
				cam_v1_r <= 1'b0;
				cam_v2_r <= 1'b0;
				miss_pend_r <= 1'b0;
			end
		end
	end

	wire unused_ok = &{1'b0, rsp_tag, ic_addr[2:0], fill, fill_pipe, rfill_pipe, rsel_pipe, m_idx_d_r, (TAG_W == 0), (N_LEQ == 0)};
endmodule

`default_nettype wire
