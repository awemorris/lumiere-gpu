/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_wb: ROP write-back: a small write-combining buffer of
 * render-target lines; reads the destination line when its old pixels are
 * needed, sends the pixels through a blend path, writes lines back; drains
 * when idle.
 *
 * High-frequency 5nm ASIC implementation:
 * - Zero CARRY4s in streaming hit detection, slot offsets, and counters.
 * - 1-LUT6 shift-and-compare per bit for streaming tag hit (t0_eq, t1_eq).
 * - 4-LUT-level streaming b_dst gather via 64-bit chunk pre-select (q64_0, q64_1).
 * - 3-stage pipelined byte-enable decode & pixel merge (pre_wr* -> commit_* -> edat).
 * - All fanouts <= 16 via replicated flip-flops and LUT1 buffer trees.
 */

`timescale 1ns / 1ps
`default_nettype none

`include "atg_config.vh"

`ifndef CFG_ROP_MRT_COUNT
`define CFG_ROP_MRT_COUNT 8
`endif

(* keep_hierarchy = "yes" *)
module atg_rop_wb #(
	parameter ADDR_W = 32,	/* GPU address width */
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter MRT_COUNT = 8,
	parameter ENABLE_MSAA = 1,
	parameter ENABLE_FBLEND = 1,
	parameter ENABLE_PACK = 1
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* From atg_rop_ds */
	input wire d_valid,
	output wire d_ready,
	output wire d_skid_ready,
	output wire [41:0] d_skid_ready_vec,
	input wire [31:0] d_xy,
	input wire [15:0] d_smask,	/* bit 4 l + s: sample s of lane l */
	input wire [511:0] d_color,
	input wire [2:0] d_rt,	/* the beat's attachment (Phase 003: MRT 8) */
	input wire d_nocol,	/* a beat without colour */
	/* atg_rop_blend */
	output wire b_valid,
	output wire [511:0] b_src,
	output wire [127:0] b_dst,
	output wire [3:0] b_mask,
	output wire [2:0] b_rt,
	input wire b_rvalid,
	input wire [127:0] b_pix,
	output wire busy,
	/* AMB client port (design §4.1) */
	(* equivalent_register_removal = "no", max_fanout = 16 *) output reg req_valid,
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
	localparam LA_W = ADDR_W - LINE_SHIFT;
	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	/* Registered input skid buffer to keep module boundary flop-to-flop (Rule 3) */
	wire di_valid, di_nocol;
	wire [31:0] di_xy;
	wire [15:0] di_smask;
	wire [511:0] di_color;
	wire [2:0] di_rt;
	wire di_ready;
	localparam integer D_SKID_W = 32 + 16 + 512 + 3 + 1;
`ifdef SYNTH_KINTEX7
	atg_rop_skid_rep #(.W(D_SKID_W), .R_W(10)) u_d_skid (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(d_valid),
		.in_ready(d_ready),
		.in_data({d_xy, d_smask, d_color, d_rt, d_nocol}),
		.out_valid(di_valid),
		.out_ready_vec(di_ready_rep[57:48]),
		.out_data({di_xy, di_smask, di_color, di_rt, di_nocol})
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	atg_rop_skid_rep #(.W(D_SKID_W), .R_W(10)) u_d_skid (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(d_valid),
		.in_ready(d_ready),
		.in_data({d_xy, d_smask, d_color, d_rt, d_nocol}),
		.out_valid(di_valid),
		.out_ready_vec(di_ready_rep[57:48]),
		.out_data({di_xy, di_smask, di_color, di_rt, di_nocol})
	);
`else
	atg_rop_skid #(.W(D_SKID_W)) u_d_skid (
		.clk(clk),
		.rst_n(rst_n),
		.in_valid(d_valid),
		.in_ready(d_ready),
		.in_data({d_xy, d_smask, d_color, d_rt, d_nocol}),
		.out_valid(di_valid),
		.out_ready(di_ready_rep[0]),
		.out_data({di_xy, di_smask, di_color, di_rt, di_nocol})
	);
`endif

	/* Stage S1: Decouples u_d_skid / u_d1_skid from CAM hazard, streaming gather, and blend dst */
	wire s1_d_ready_w;
	reg s1_v, s1_nocol, s1_sm_zero;
	reg [31:0] s1_xy;
	reg [15:0] s1_smask;
	reg [511:0] s1_color;
	reg [2:0] s1_rt;
	localparam integer N_DREP = 10;
	localparam integer N_DRDY = 130;
	(* keep = "true", dont_touch = "true" *) reg [N_DREP-1:0] s1_v_drep, s1_nocol_drep, s1_sm_zero_drep;
	(* keep = "true", dont_touch = "true" *) reg [N_DREP-1:0] is_idle_drep, pipe_v0_drep, pipe_hi_empty_drep;
	(* keep = "true", dont_touch = "true" *) reg [N_DREP-1:0] line_stream_ok_drep, s1_sh0_drep, s1_sh1_drep;
	(* keep = "true", dont_touch = "true" *) reg [2:0] rt_blend, s1_rt_blend, rt_wpix_b, rt_wpix_s;
	(* keep = "true", dont_touch = "true" *) reg is_idle_rt;
	wire haz_clear_w;
	wire [4:0] haz_clear_rep;
	wire [11:0] haz_cam_full_rep, haz_hv_clear_rep, haz_v16_rep, haz_vrel_or0_rep, haz_vrel_or1_rep;
	wire [11:0] haz_arel_or0_rep, haz_arel_or1_rep, haz_vhit_a_rep;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire [N_DREP-1:0] sgs_nf_vec, no_bypass_vec;
	(* keep = "true", dont_touch = "true" *) wire [N_DRDY-1:0] di_ready_rep;
	genvar gdr;
	generate
	for (gdr = 0; gdr < N_DREP; gdr = gdr + 1) begin : g_di_rdy_pre
		localparam integer H_IDX = (gdr < 5) ? gdr : (gdr + 2);
		(* keep = "true" *) wire sgs_nf_w = is_idle_drep[gdr] && line_stream_ok_drep[gdr] && s1_sh0_drep[gdr] && s1_sh1_drep[gdr] && !haz_cam_full_rep[H_IDX];
		(* keep = "true" *) wire no_bypass_w = s1_v_drep[gdr] && !(is_idle_drep[gdr] && (s1_nocol_drep[gdr] || s1_sm_zero_drep[gdr] || (!pipe_v0_drep[gdr] && pipe_hi_empty_drep[gdr])));
		assign sgs_nf_vec[gdr] = sgs_nf_w;
		assign no_bypass_vec[gdr] = no_bypass_w;
	end
	for (gdr = 0; gdr < N_DRDY; gdr = gdr + 1) begin : g_di_ready_rep
		localparam integer D_IDX = gdr % N_DREP;
		localparam integer H_IDX = (D_IDX < 5) ? D_IDX : (D_IDX + 2);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hDDD5DD55DDDDDD5D)) u_drdy (
			.I0(no_bypass_vec[D_IDX]), .I1(sgs_nf_vec[D_IDX]),
			.I2(haz_vhit_a_rep[H_IDX]), .I3(haz_vrel_or0_rep[H_IDX]),
			.I4(haz_arel_or0_rep[H_IDX]), .I5(haz_v16_rep[H_IDX]),
			.O(di_ready_rep[gdr])
		);
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire [N_DREP-1:0] sgs_nf_vec, no_bypass_vec;
	(* keep = "true", dont_touch = "true" *) wire [N_DRDY-1:0] di_ready_rep;
	genvar gdr;
	generate
	for (gdr = 0; gdr < N_DREP; gdr = gdr + 1) begin : g_di_rdy_pre
		localparam integer H_IDX = (gdr < 5) ? gdr : (gdr + 2);
		(* keep = "true" *) wire sgs_nf_w = is_idle_drep[gdr] && line_stream_ok_drep[gdr] && s1_sh0_drep[gdr] && s1_sh1_drep[gdr] && !haz_cam_full_rep[H_IDX];
		(* keep = "true" *) wire no_bypass_w = s1_v_drep[gdr] && !(is_idle_drep[gdr] && (s1_nocol_drep[gdr] || s1_sm_zero_drep[gdr] || (!pipe_v0_drep[gdr] && pipe_hi_empty_drep[gdr])));
		assign sgs_nf_vec[gdr] = sgs_nf_w;
		assign no_bypass_vec[gdr] = no_bypass_w;
	end
	for (gdr = 0; gdr < N_DRDY; gdr = gdr + 1) begin : g_di_ready_rep
		localparam integer D_IDX = gdr % N_DREP;
		localparam integer H_IDX = (D_IDX < 5) ? D_IDX : (D_IDX + 2);
		(* keep = "true", dont_touch = "true" *) LUT6 #(.INIT(64'hDDD5DD55DDDDDD5D)) u_drdy (
			.I0(no_bypass_vec[D_IDX]), .I1(sgs_nf_vec[D_IDX]),
			.I2(haz_vhit_a_rep[H_IDX]), .I3(haz_vrel_or0_rep[H_IDX]),
			.I4(haz_arel_or0_rep[H_IDX]), .I5(haz_v16_rep[H_IDX]),
			.O(di_ready_rep[gdr])
		);
	end
	endgenerate
`else
	wire [N_DREP-1:0] di_ready_rep;
	genvar gdr;
	generate
	for (gdr = 0; gdr < N_DREP; gdr = gdr + 1) begin : g_di_ready_rep
		localparam integer H_IDX = (gdr < 5) ? gdr : (gdr + 2);
		wire iord_k = is_idle_drep[gdr] && (s1_nocol_drep[gdr] || s1_sm_zero_drep[gdr] || (!pipe_v0_drep[gdr] && pipe_hi_empty_drep[gdr]));
		wire sgs_nf_k = is_idle_drep[gdr] && line_stream_ok_drep[gdr] && s1_sh0_drep[gdr] && s1_sh1_drep[gdr] && !haz_cam_full_rep[H_IDX];
		wire no_bypass_k = s1_v_drep[gdr] && !iord_k;
		wire t_vm_n = !((haz_vrel_or0_rep[H_IDX] | haz_vrel_or1_rep[H_IDX]) & sgs_nf_k);
		wire t_va_n = !((haz_arel_or0_rep[H_IDX] | haz_arel_or1_rep[H_IDX]) & (haz_vhit_a_rep[H_IDX] & sgs_nf_k));
		wire sgs_noa_n = !(sgs_nf_k && !haz_vhit_a_rep[H_IDX]);
		wire u0_rel = !(t_vm_n & t_va_n);
		wire hv_or_nb = !((haz_v16_rep[H_IDX] | sgs_noa_n) & no_bypass_k);
		assign di_ready_rep[gdr] = u0_rel | hv_or_nb;
	end
	endgenerate
`endif
	wire di_ready_fast_w = !s1_v || idle_or_drop;
	assign di_ready = di_ready_fast_w || stream_hit_free;
	wire unused_haz_clr_rep = &{1'b0, haz_clear_rep};

	wire [45:0] s1_ce_leaf;
`ifdef SYNTH_KINTEX7
	assign s1_ce_leaf = di_ready_rep[45:0];
`elsif SYNTH_KINTEX_ULTRASCALE
	assign s1_ce_leaf = di_ready_rep[45:0];
`else
	assign s1_ce_leaf[15:0]  = {16{di_ready_rep[1]}};
	assign s1_ce_leaf[39:16] = {24{di_ready_rep[2]}};
	assign s1_ce_leaf[45:40] = {6{di_ready_rep[3]}};
`endif

	integer i_s1c, i_dr;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			s1_v <= 1'b0;
			s1_v_drep <= {N_DREP{1'b0}};
		end else begin
`ifdef SYNTH_KINTEX7
			if (di_ready_rep[46]) begin
				s1_v <= di_valid;
				s1_v_drep <= {N_DREP{di_valid}};
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (di_ready_rep[46]) begin
				s1_v <= di_valid;
				s1_v_drep <= {N_DREP{di_valid}};
			end
`else
			if (di_ready_rep[3])
				s1_v <= di_valid;
			for (i_dr = 0; i_dr < N_DREP; i_dr = i_dr + 1)
				if (di_ready_rep[i_dr]) s1_v_drep[i_dr] <= di_valid;
`endif
		end
	end
	always @(posedge clk) begin
		for (i_s1c = 0; i_s1c < 16; i_s1c = i_s1c + 1) begin
			if (s1_ce_leaf[i_s1c])
				s1_color[8 * i_s1c +: 8] <= di_color[8 * i_s1c +: 8];
		end
		for (i_s1c = 16; i_s1c < 40; i_s1c = i_s1c + 1) begin
			if (s1_ce_leaf[i_s1c])
				s1_color[128 + 16 * (i_s1c - 16) +: 16] <= di_color[128 + 16 * (i_s1c - 16) +: 16];
		end
		if (s1_ce_leaf[40]) s1_xy[15:0]  <= di_xy[15:0];
		if (s1_ce_leaf[41]) s1_xy[31:16] <= di_xy[31:16];
		if (s1_ce_leaf[42]) s1_smask     <= di_smask;
		if (s1_ce_leaf[43]) begin
			s1_rt      <= di_rt;
			s1_rt_blend <= di_rt;
			s1_nocol   <= di_nocol;
			s1_sm_zero <= (di_smask == 16'd0);
		end
		if (s1_ce_leaf[44])
			s1_nocol_drep <= {N_DREP{di_nocol}};
		if (s1_ce_leaf[45])
			s1_sm_zero_drep <= {N_DREP{(di_smask == 16'd0)}};
	end

	/* The two cache line entries */
	reg ev [0:1];
	reg erd [0:1];
	reg [LA_W-1:0] etag [0:1];
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [LINE_BYTES*8-1:0] edat [0:1];
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [LINE_BYTES-1:0] edirty [0:1];
	wire [LINE_BYTES*8-1:0] edat0 = edat[0], edat1 = edat[1];
	wire [LINE_BYTES-1:0] edirty0 = edirty[0], edirty1 = edirty[1];

	localparam [4:0]
		S_IDLE        = 5'd0,
		S_ROW_WAIT1   = 5'd1,
		S_ROW_WAIT2   = 5'd2,
		S_ROW_WAIT3   = 5'd3,
		S_ROW_WAIT4   = 5'd4,
		S_ROW_WAIT5   = 5'd5,
		S_ROW_PREP    = 5'd6,
		S_ROW         = 5'd7,
		S_LOOK        = 5'd8,
		S_EVW         = 5'd9,
		S_FILL        = 5'd10,
		S_BLEND_PREP1 = 5'd11,
		S_BLEND_PREP2 = 5'd12,
		S_BLEND       = 5'd13,
		S_BWAIT       = 5'd14,
		S_BCOMMIT     = 5'd15,
		S_FLUSH       = 5'd16,
		S_FLW         = 5'd17,
		S_LOOK_PREP   = 5'd18,
		S_FLUSH_PREP  = 5'd19,
		S_ROW_WAIT6   = 5'd20,
		S_ROW_PREP2   = 5'd21;

	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [4:0] st;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg r, r_fsm, r_fill, r_tag, r_req_ln, r_dty_lo, r_dty_hi;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [1:0] j, j_wpix;	/* line of the row (several lines) */
	(* max_fanout = 16 *) reg [1:0] g;	/* pass (one line per row: the sample) */
	reg wait_rsp;
	reg [4:0] idle;

	wire [1:0] j1 = {j[1] ^ j[0], ~j[0]};
	wire [1:0] g1 = {g[1] ^ g[0], ~g[0]};
	wire [4:0] idle1;
	wire idle1_co_unused;
	wire j1_co_unused = 1'b0;
	wire g1_co_unused = 1'b0;
	am_add #(.W(5)) u_inc_idle (.a(idle), .b(5'd1), .ci(1'b0), .s(idle1), .co(idle1_co_unused));

	reg [31:0] xy;
	reg [15:0] mask;
	reg [511:0] color;
	reg [2:0] rt;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] addr;
	reg [LINE_SHIFT-1:0] off [0:1];
	reg [LINE_SHIFT-1:0] off_p1 [0:1];
	reg f_go;
	reg fsm_b_valid;
	(* equivalent_register_removal = "no", max_fanout = 16 *) reg bwait_d1;
	reg r_look_hit;
	reg r_edirty_nz;
	reg [3:0] r_pmask;
	reg r_pmask_z;

	/* Interface wires to/from atg_rop_wpix */
	wire [31:0] p_addr, p_addr1;
	wire [31:0] p_row_base0, p_row_base1, p_stride, p_xoff_j;
	wire [1:0] p_nl, p_np, p_path;
	wire p_multi, p_drop, p_read, p_lw, f_done;
	wire [3:0] p_mask, p_ent;
	wire [511:0] p_res;
	wire [4*LINE_SHIFT-1:0] p_soff;
	wire [4:0] p_ps;
	wire [31:0] p_xoff;
	wire [2:0] p_xsh;
	wire f_take = (st == S_BWAIT) && (r_bl_path == 2'd1) && f_done;

	(* equivalent_register_removal = "no", max_fanout = 16 *) reg [31:0] wb_row_base0, wb_row_base1, cur_row_base_r;
	wire [31:0] wb_row_base1_comb;
	wire wb_rb1_co;
	/* Both inputs are registered FFs -> 1 Pre-CARRY4 LUT, 0 Post-CARRY4 LUTs */
	am_add #(.W(32)) u_wb_rb1 (.a(wb_row_base0), .b(p_stride), .ci(1'b0), .s(wb_row_base1_comb), .co(wb_rb1_co));
	wire [31:0] wb_addr_comb;
	wire wb_addr_co;
	/* Both inputs are registered FFs -> 1 Pre-CARRY4 LUT, 0 Post-CARRY4 LUTs */
	am_add #(.W(32)) u_wb_addr (.a(cur_row_base_r), .b(p_xoff_j), .ci(1'b0), .s(wb_addr_comb), .co(wb_addr_co));

	wire [31:0] p_ps_32 = {27'd0, p_ps};
	wire [LINE_SHIFT-1:0] ext_ps = p_ps_32[LINE_SHIFT-1:0];
	wire [LINE_SHIFT-1:0] off_p1_comb;
	wire off_p1_co_unused;
	am_add #(.W(LINE_SHIFT)) u_off_p1 (
		.a(addr[LINE_SHIFT-1:0]),
		.b(ext_ps),
		.ci(1'b0),
		.s(off_p1_comb),
		.co(off_p1_co_unused)
	);

`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) reg is_idle_reg;
`else
	reg is_idle_reg;
`endif

	/* Line Tracking and Cached Row Base */
	reg line_match_valid;
	reg stream_cfg_ok;
	reg [2:0] line_rt;
	reg [15:0] line_y;
	reg [LA_W-1:0] cached_tag_x0;
	reg [LA_W-1:0] cached_tag_x1;
	reg [31:0] cached_row_base0;
	reg [31:0] cached_row_base1;
	reg [2:0] cached_xsh;
	(* keep = "true", dont_touch = "true" *) reg [1:0] cached_xsh_rep [0:15];
	reg cached_drop;
	reg cached_multi;
	reg [1:0] cached_np;
	reg [1:0] cached_path;
	reg cached_read;
	reg [1:0] cached_ps;
	reg [3:0] cached_lm;

	wire [LA_W-1:0] tag_x0_comb, tag_x1_comb;
	wire tag_x0_co, tag_x1_co;
	/* Both inputs are registered FFs, and outputs go directly to unconditional FFs tag_x0_r/tag_x1_r
	 * -> 1 Pre-CARRY4 LUT, 0 Post-CARRY4 LUTs! */
	wire [63:0] wb_rb0_ext = {32'd0, wb_row_base0};
	wire [63:0] wb_rb1_ext = {32'd0, wb_row_base1};
	wire [63:0] addr_ext = {32'd0, addr};
	am_sub #(.W(LA_W)) u_tag_x0 (.a(etag[0]), .b(wb_rb0_ext[ADDR_W-1:LINE_SHIFT]), .bi(1'b0), .d(tag_x0_comb), .bo(tag_x0_co));
	am_sub #(.W(LA_W)) u_tag_x1 (.a(etag[1]), .b(wb_rb1_ext[ADDR_W-1:LINE_SHIFT]), .bi(1'b0), .d(tag_x1_comb), .bo(tag_x1_co));
	reg [LA_W-1:0] tag_x0_r, tag_x1_r;
	reg rb0_aligned_r, rb1_aligned_r;
	always @(posedge clk) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		if (!is_idle_reg) begin
`endif
			tag_x0_r <= tag_x0_comb;
			tag_x1_r <= tag_x1_comb;
			rb0_aligned_r <= (wb_row_base0[LINE_SHIFT-1:0] == {LINE_SHIFT{1'b0}});
			rb1_aligned_r <= (wb_row_base1[LINE_SHIFT-1:0] == {LINE_SHIFT{1'b0}});
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		end
`endif
	end

	/* Fast line-offset calculation for streaming (1 LUT level, 0 CARRY4s) */
	wire [LINE_SHIFT-1:1] d_xy_loff;
`ifdef SYNTH_KINTEX7
	genvar g_xl;
	generate
	for (g_xl = 1; g_xl < LINE_SHIFT; g_xl = g_xl + 1) begin : g_dxy_loff
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_xl (.I0(s1_xy[g_xl]), .O(d_xy_loff[g_xl]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_xl;
	generate
	for (g_xl = 1; g_xl < LINE_SHIFT; g_xl = g_xl + 1) begin : g_dxy_loff
		assign d_xy_loff[g_xl] = s1_xy[g_xl];
	end
	endgenerate
`else
	assign d_xy_loff = s1_xy[LINE_SHIFT-1:1];
`endif
	wire [LINE_SHIFT-1:0] d_xy_lo_b = {d_xy_loff, s1_xy[0]};
	wire [1:0] xsh_loff = cached_xsh_rep[0];
	(* keep = "true" *) wire [LINE_SHIFT-1:0] str_line_off =
		(xsh_loff == 2'd2) ? {d_xy_lo_b[LINE_SHIFT-3:0], 2'b00} :
		(xsh_loff == 2'd1) ? {d_xy_lo_b[LINE_SHIFT-2:0], 1'b0} :
		d_xy_lo_b[LINE_SHIFT-1:0];

	wire [LINE_SHIFT-1:0] str_x_p1 = {d_xy_loff[LINE_SHIFT-1:1], 1'b1};
	(* keep = "true" *) wire [LINE_SHIFT-1:0] str_line_off_p1 =
		(xsh_loff == 2'd2) ? {str_x_p1[LINE_SHIFT-3:0], 2'b00} :
		(xsh_loff == 2'd1) ? {str_x_p1[LINE_SHIFT-2:0], 1'b0} :
		str_x_p1;

	wire [511:0] p_bsrc;
	wire [127:0] p_bdst;
	reg [511:0] r_bsrc;
	reg [127:0] r_bdst;
	reg [3:0] r_bmask;
	reg [4*LINE_SHIFT-1:0] r_bsoff;
	reg [3:0] r_bent;
	reg [1:0] r_bps;
	reg [3:0] r_blm;

`ifdef SYNTH_KINTEX7
	wire [10:0] idle_grp;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig0  (.I0(is_idle_reg), .O(idle_grp[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig1  (.I0(is_idle_reg), .O(idle_grp[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig2  (.I0(is_idle_reg), .O(idle_grp[2]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig3  (.I0(is_idle_reg), .O(idle_grp[3]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig4  (.I0(is_idle_reg), .O(idle_grp[4]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig5  (.I0(is_idle_reg), .O(idle_grp[5]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig6  (.I0(is_idle_reg), .O(idle_grp[6]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig7  (.I0(is_idle_reg), .O(idle_grp[7]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig8  (.I0(is_idle_reg), .O(idle_grp[8]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig9  (.I0(is_idle_reg), .O(idle_grp[9]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig10 (.I0(is_idle_reg), .O(idle_grp[10]));
	wire [1:0] j_wpix_buf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_jwp0 (.I0(j_wpix[0]), .O(j_wpix_buf[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_jwp1 (.I0(j_wpix[1]), .O(j_wpix_buf[1]));

	wire [31:0] idle_bsrc;
	genvar g_ib;
	generate
	for (g_ib = 0; g_ib < 32; g_ib = g_ib + 1) begin : g_ib_buf
		localparam G_IDX = g_ib / 16;
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ib (.I0(idle_grp[G_IDX]), .O(idle_bsrc[g_ib]));
	end
	endgenerate

	genvar g_bs;
	generate
	for (g_bs = 0; g_bs < 32; g_bs = g_bs + 1) begin : g_bs_mux
		assign b_src[16 * g_bs +: 16] = idle_bsrc[g_bs] ? s1_color[16 * g_bs +: 16] : r_bsrc[16 * g_bs +: 16];
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [10:0] idle_grp;
	assign idle_grp[0] = is_idle_reg;
	assign idle_grp[1] = is_idle_reg;
	assign idle_grp[2] = is_idle_reg;
	assign idle_grp[3] = is_idle_reg;
	assign idle_grp[4] = is_idle_reg;
	assign idle_grp[5] = is_idle_reg;
	assign idle_grp[6] = is_idle_reg;
	assign idle_grp[7] = is_idle_reg;
	assign idle_grp[8] = is_idle_reg;
	assign idle_grp[9] = is_idle_reg;
	assign idle_grp[10] = is_idle_reg;
	wire [1:0] j_wpix_buf;
	assign j_wpix_buf[0] = j_wpix[0];
	assign j_wpix_buf[1] = j_wpix[1];

	wire [31:0] idle_bsrc;
	genvar g_ib;
	generate
	for (g_ib = 0; g_ib < 32; g_ib = g_ib + 1) begin : g_ib_buf
		localparam G_IDX = g_ib / 16;
		assign idle_bsrc[g_ib] = idle_grp[G_IDX];
	end
	endgenerate

	genvar g_bs;
	generate
	for (g_bs = 0; g_bs < 32; g_bs = g_bs + 1) begin : g_bs_mux
		assign b_src[16 * g_bs +: 16] = idle_bsrc[g_bs] ? s1_color[16 * g_bs +: 16] : r_bsrc[16 * g_bs +: 16];
	end
	endgenerate
`else
	assign b_src = is_idle_reg ? s1_color : r_bsrc;
`endif

	/* Split streaming b_dst gather across Cycle 0 (q64_0/1 -> r_q64_0/1) and Cycle 1 (r_q64_0/1 -> b_dst),
	 * matching atg_rop_blend #(.DST_LATE(1)) with zero added latency.
	 */
	localparam integer CHUNK_IDX_W = LINE_SHIFT - 3;
	(* keep = "true" *) wire [CHUNK_IDX_W-1:0] str_c_rep [0:7];
	(* keep = "true" *) wire [63:0] q64_0, q64_1;
	reg [63:0] r_q64_0, r_q64_1;
	reg [2:1] r_str_x21;
	(* keep = "true", dont_touch = "true" *) reg [7:0] pipe_v0_bdst;
	genvar gc, gbit;
	generate
	for (gc = 0; gc < 8; gc = gc + 1) begin : g_str_c
		wire [1:0] xsh_c = cached_xsh_rep[gc];
		assign str_c_rep[gc] =
			xsh_c[1] ? s1_xy[1 +: CHUNK_IDX_W] :
			xsh_c[0] ? s1_xy[2 +: CHUNK_IDX_W] :
			           s1_xy[3 +: CHUNK_IDX_W];
	end
	for (gc = 0; gc < 4; gc = gc + 1) begin : g_q64
		for (gbit = 0; gbit < 16; gbit = gbit + 1) begin : g_qb
			assign q64_0[16 * gc + gbit] = edat0[{str_c_rep[gc],     gc[1:0], gbit[3:0]}];
			assign q64_1[16 * gc + gbit] = edat1[{str_c_rep[4 + gc], gc[1:0], gbit[3:0]}];
		end
	end
	for (gc = 0; gc < 4; gc = gc + 1) begin : g_bdst_slot
		wire [63:0] q64 = (gc >= 2) ? r_q64_1 : r_q64_0;
		wire [1:0] xsh_slo = cached_xsh_rep[8 + gc];
		wire [1:0] xsh_shi = cached_xsh_rep[12 + gc];
		wire str_lo = pipe_v0_bdst[2 * gc + 0];
		wire str_hi = pipe_v0_bdst[2 * gc + 1];
		wire [1:0] x21_lo;
		wire x1_hi;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_x21_l0 (.I0(r_str_x21[1]), .O(x21_lo[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_x21_l1 (.I0(r_str_x21[2]), .O(x21_lo[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_x1_hi  (.I0(r_str_x21[1]), .O(x1_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign x21_lo[0] = r_str_x21[1];
		assign x21_lo[1] = r_str_x21[2];
		assign x1_hi = r_str_x21[1];
`else
		assign x21_lo = r_str_x21;
		assign x1_hi  = r_str_x21[1];
`endif

		for (gbit = 0; gbit < 8; gbit = gbit + 1) begin : g_b0_7
			(* keep = "true" *) wire s_p0 = q64[{x21_lo, gc[0], gbit[2:0]}];
			(* keep = "true" *) wire s_p12 = gc[0] ?
				(xsh_slo[1] ? q64[32 + gbit] : (x21_lo[0] ? q64[48 + gbit] : q64[16 + gbit])) :
				(xsh_slo[1] ? q64[gbit]      : (x21_lo[0] ? q64[32 + gbit] : q64[gbit]));
			assign b_dst[32 * gc + gbit] = str_lo ?
				((xsh_slo == 2'd0) ? s_p0 : s_p12) :
				r_bdst[32 * gc + gbit];
		end
		for (gbit = 8; gbit < 16; gbit = gbit + 1) begin : g_b8_15
			(* keep = "true" *) wire s_p12 = gc[0] ?
				(xsh_slo[1] ? q64[32 + gbit] : (xsh_slo[0] ? (x1_hi ? q64[48 + gbit] : q64[16 + gbit]) : 1'b0)) :
				(xsh_slo[1] ? q64[gbit]      : (xsh_slo[0] ? (x1_hi ? q64[32 + gbit] : q64[gbit])      : 1'b0));
			assign b_dst[32 * gc + gbit] = str_lo ? s_p12 : r_bdst[32 * gc + gbit];
		end
		for (gbit = 16; gbit < 32; gbit = gbit + 1) begin : g_b16_31
			assign b_dst[32 * gc + gbit] = str_hi ?
				(xsh_shi[1] ? q64[(gc[0] ? 32 : 0) + gbit] : 1'b0) :
				r_bdst[32 * gc + gbit];
		end
	end
	endgenerate

	/* wpix is driven exclusively by registered FSM state (0 combinational paths from d_* inputs) */
	atg_rop_wpix #(
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.MRT_COUNT(MRT_COUNT),
		.ENABLE_MSAA(ENABLE_MSAA),
		.ENABLE_FBLEND(ENABLE_FBLEND),
		.ENABLE_PACK(ENABLE_PACK)
	) wpix (
		.clk(clk), .rst_n(rst_n), .st_we(st_we), .st_idx(st_idx), .st_wdata(st_wdata),
		.i_xy(xy), .i_rt(rt), .i_rt_b(rt_wpix_b), .i_rt_s(rt_wpix_s), .i_smask(mask), .i_color(color),
`ifdef SYNTH_KINTEX7
		.i_r(r), .i_j(j_wpix_buf), .i_g(g),
`elsif SYNTH_KINTEX_ULTRASCALE
		.i_r(r), .i_j(j_wpix_buf), .i_g(g),
`else
		.i_r(r), .i_j(j_wpix), .i_g(g),
`endif
		.i_line0(edat0), .i_line1(edat1),
		.i_off0(off[0]), .i_off1(off[1]),
		.i_off0_p1(off_p1[0]), .i_off1_p1(off_p1[1]),
		.i_reg_off0(off[0]), .i_reg_off1(off[1]),
		.i_reg_rt(rt), .i_reg_color(color), .i_reg_mask(mask),
`ifdef SYNTH_KINTEX7
		.b_pix(b_pix), .f_go(f_go), .f_done(f_done), .f_take(f_take), .i_idle(idle_grp[2]),
`elsif SYNTH_KINTEX_ULTRASCALE
		.b_pix(b_pix), .f_go(f_go), .f_done(f_done), .f_take(f_take), .i_idle(idle_grp[2]),
`else
		.b_pix(b_pix), .f_go(f_go), .f_done(f_done), .f_take(f_take), .i_idle(is_idle_reg),
`endif
		.o_addr(p_addr), .o_addr1(p_addr1), .o_nl(p_nl), .o_np(p_np), .o_multi(p_multi), .o_drop(p_drop),
		.o_read(p_read), .o_path(p_path), .o_lw(p_lw), .o_mask(p_mask), .o_bsrc(p_bsrc),
		.o_bdst(p_bdst), .o_res(p_res), .o_ent(p_ent), .o_soff(p_soff), .o_ps(p_ps),
		.o_row_base0(p_row_base0), .o_row_base1(p_row_base1),
		.o_xoff(p_xoff), .o_xsh(p_xsh),
		.o_stride(p_stride), .o_xoff_j(p_xoff_j)
	);

	/* Pure-LUT Cache Hit Detection and Streaming Qualification (pre-registered into s1_sh0/1/2) */
	localparam integer TAG_CMP_BITS = 18 - LINE_SHIFT; /* 13 bits for LINE_SHIFT=5, 12 bits for LINE_SHIFT=6 */
	localparam integer T_HI = TAG_CMP_BITS - 1;
	wire [1:0] xsh_t0 = cached_xsh_rep[1];
	wire [1:0] xsh_t1 = cached_xsh_rep[2];
`ifdef SYNTH_KINTEX7
	wire [31:0] sh_xy_cand;
	assign sh_xy_cand[15:0]  = idle_grp[8] ? di_xy[15:0]  : s1_xy[15:0];
	assign sh_xy_cand[31:16] = idle_grp[9] ? di_xy[31:16] : s1_xy[31:16];
	wire [2:0] sh_rt_cand = idle_grp[9] ? di_rt : s1_rt;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [31:0] sh_xy_cand;
	assign sh_xy_cand[15:0]  = idle_grp[8] ? di_xy[15:0]  : s1_xy[15:0];
	assign sh_xy_cand[31:16] = idle_grp[9] ? di_xy[31:16] : s1_xy[31:16];
	wire [2:0] sh_rt_cand = idle_grp[9] ? di_rt : s1_rt;
`else
	wire [31:0] sh_xy_cand = is_idle_reg ? di_xy : s1_xy;
	wire [2:0] sh_rt_cand = is_idle_reg ? di_rt : s1_rt;
`endif
	(* keep = "true" *) wire [7:0] t01_eq;
	assign t01_eq[0] = ((!xsh_t0[1]) || ((sh_xy_cand[LINE_SHIFT - 2] == cached_tag_x0[0]) && (sh_xy_cand[LINE_SHIFT - 2] == cached_tag_x1[0]))) &&
	                   (sh_xy_cand[31] == line_y[15]);
	assign t01_eq[1] = ((xsh_t1 == 2'd0) || ((sh_xy_cand[LINE_SHIFT - 1] == cached_tag_x0[1]) && (sh_xy_cand[LINE_SHIFT - 1] == cached_tag_x1[1])));
	genvar gt;
	generate
	for (gt = 0; gt < 5; gt = gt + 1) begin : g_tag_eq
		localparam integer X_LO = LINE_SHIFT + 2 * gt;
		localparam integer T_LO = 2 + 2 * gt;
		assign t01_eq[2 + gt] =
			(sh_xy_cand[X_LO +: 2] == cached_tag_x0[T_LO +: 2]) &&
			(sh_xy_cand[X_LO +: 2] == cached_tag_x1[T_LO +: 2]);
	end
	if (LINE_SHIFT == 5) begin : g_teq7_ls5
		assign t01_eq[7] = (sh_xy_cand[15] == cached_tag_x0[12]) && (sh_xy_cand[15] == cached_tag_x1[12]) &&
		                   !sh_xy_cand[0];
	end else begin : g_teq7_ls6
		assign t01_eq[7] = (cached_tag_x0[12] == cached_tag_x1[12]) &&
		                   !sh_xy_cand[0];
	end
	endgenerate

	(* keep = "true" *) wire [4:0] y_eq3;
	assign y_eq3[0] = (sh_xy_cand[18:16] == line_y[2:0]);
	assign y_eq3[1] = (sh_xy_cand[21:19] == line_y[5:3]);
	assign y_eq3[2] = (sh_xy_cand[24:22] == line_y[8:6]);
	assign y_eq3[3] = (sh_xy_cand[27:25] == line_y[11:9]);
	assign y_eq3[4] = (sh_xy_cand[30:28] == line_y[14:12]);

	(* keep = "true" *) wire hit_rt = (sh_rt_cand == line_rt);
	(* keep = "true" *) wire dval_col = s1_v && !s1_nocol;
	reg s1_sh0, s1_sh1;
	wire s1_sh0_nxt = &t01_eq[5:0];
	wire s1_sh1_nxt = (&{t01_eq[7:6], y_eq3[4:0]}) && hit_rt;

	/* Pipeline shift register (11 stages for atg_rop_blend LAT=11 + 1 commit stage) */
	reg [10:0] pipe_v;
	reg pipe_nz_1_r;
	reg [3:0] pipe_id [0:10];
	reg [4*LINE_SHIFT-1:0] pipe_soff [0:9];
	reg [3:0] pipe_mask [0:9];
	reg [3:0] pipe_ent [0:9];
	reg [1:0] pipe_ps [0:9];
	reg [3:0] pipe_lm [0:9];
	localparam integer N_CREP = LINE_BYTES + 4;
	reg commit_v;
	(* keep = "true", dont_touch = "true" *) reg [N_CREP-1:0] commit_v_rep;
	reg commit_is_stream;
	reg [3:0] commit_id;

	(* keep = "true" *) wire pipe_nz_0 = (pipe_v[5:0] != 6'd0);
	reg r_pipe_nz_0;
	reg r_pipe_hi_empty;
	reg r_idle_is16;
	(* keep = "true" *) wire pipe_nz_1 = pipe_nz_1_r;
	wire pipe_busy = pipe_nz_0 || pipe_nz_1;

	(* keep = "true" *) wire pipe_all_empty = !pipe_v[0] && r_pipe_hi_empty;

	/* Flat idle_or_drop without multi-bit pipe_v reduction */
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire idle_or_drop =
		idle_grp[10] && (s1_nocol || s1_sm_zero || pipe_all_empty);
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire idle_or_drop =
		idle_grp[10] && (s1_nocol || s1_sm_zero || pipe_all_empty);
`else
	(* keep = "true" *) wire idle_or_drop =
		is_idle_reg && (s1_nocol || s1_sm_zero || pipe_all_empty);
`endif

	/* Hazard Tracker and Pixel Lock CAM */
	wire alloc_ready;
	wire [3:0] alloc_id;
	wire [2:0] haz_grp_hit;
	reg release_valid;
	reg [3:0] release_id;
	reg [15:0] release_oh;

	/* Flat pre-qualification of stream path decoupled from haz_grp_hit */
	wire haz_vhit_any, haz_cam_full;
	wire [3:0] haz_vhit_id;
	wire unused_wb_haz_wires = &{1'b0, haz_vhit_any, haz_cam_full, haz_vhit_id,
		haz_v16_rep[6:5], haz_hv_clear_rep[11:7], haz_hv_clear_rep[4:0], haz_vrel_or1_rep, haz_arel_or1_rep};
	reg r_line_stream_ok;
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire stream_gate_sh = idle_grp[3] && r_line_stream_ok && s1_sh0 && s1_sh1;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire stream_gate_sh = idle_grp[3] && r_line_stream_ok && s1_sh0 && s1_sh1;
`else
	(* keep = "true" *) wire stream_gate_sh = is_idle_reg && r_line_stream_ok && s1_sh0 && s1_sh1;
`endif
	(* keep = "true" *) wire sh_grp2_cand   = dval_col && !s1_sm_zero;
	wire shf_nf_ok = stream_gate_sh && !haz_cam_full_rep[6];
	wire s_vm_n = !((haz_vrel_or0_rep[6] | haz_vrel_or1_rep[6]) & shf_nf_ok);
	wire s_va_n = !((haz_arel_or0_rep[6] | haz_arel_or1_rep[6]) & (haz_vhit_a_rep[6] & shf_nf_ok));
	wire s_hv_n = !(haz_hv_clear_rep[6] & shf_nf_ok);
	wire stream_hit_free = !(s_vm_n & s_va_n) || !s_hv_n;
	(* keep = "true" *) wire stream_cand_haz = stream_gate_sh && sh_grp2_cand;
`ifdef SYNTH_KINTEX7
	wire stream_cand_ok = idle_grp[5] && r_line_stream_ok && s1_sh0 && s1_sh1 && dval_col && !s1_sm_zero && !haz_cam_full_rep[5];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire stream_cand_ok = idle_grp[5] && r_line_stream_ok && s1_sh0 && s1_sh1 && dval_col && !s1_sm_zero && !haz_cam_full_rep[5];
`else
	wire stream_cand_ok = is_idle_reg && r_line_stream_ok && s1_sh0 && s1_sh1 && dval_col && !s1_sm_zero && !haz_cam_full_rep[5];
`endif
	wire d_vm_n = !((haz_vrel_or0_rep[5] | haz_vrel_or1_rep[5]) & stream_cand_ok);
	wire d_va_n = !((haz_arel_or0_rep[5] | haz_arel_or1_rep[5]) & (haz_vhit_a_rep[5] & stream_cand_ok));
	wire d_hv_n = !(haz_hv_clear_rep[5] & stream_cand_ok);
	wire do_stream = !(d_vm_n & d_va_n) || !d_hv_n;
	wire stream_ok = stream_hit_free;
	wire stream_hit_w = stream_hit_free;
	wire unused_haz_clr_w = haz_clear_w;

	wire next_release_valid = commit_v && commit_is_stream;
	wire [3:0] next_release_id = commit_id;
	wire [15:0] next_release_oh = next_release_valid ? {
		(commit_id == 4'd15), (commit_id == 4'd14), (commit_id == 4'd13), (commit_id == 4'd12),
		(commit_id == 4'd11), (commit_id == 4'd10), (commit_id == 4'd9),  (commit_id == 4'd8),
		(commit_id == 4'd7),  (commit_id == 4'd6),  (commit_id == 4'd5),  (commit_id == 4'd4),
		(commit_id == 4'd3),  (commit_id == 4'd2),  (commit_id == 4'd1),  (commit_id == 4'd0)
	} : 16'd0;

`ifdef SYNTH_KINTEX7
	wire di_rdy_haz_w = di_ready_rep[47];
	wire [28:0] pre_load_vec_w = di_ready_rep[128:100];
`elsif SYNTH_KINTEX_ULTRASCALE
	wire di_rdy_haz_w = di_ready_rep[47];
	wire [28:0] pre_load_vec_w = di_ready_rep[128:100];
`else
	wire di_rdy_haz_w = di_ready_rep[4];
	wire [28:0] pre_load_vec_w = {29{di_ready_rep[4]}};
`endif
	reg [32:0] r_skid_tag;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync)
			r_skid_tag <= 33'd0;
		else if (d_valid && d_ready && !di_rdy_haz_w && di_valid)
			r_skid_tag <= {d_rt, d_xy[31:17], d_xy[15:1]};
	end
	wire di_haz_load_w = !d_ready ? di_rdy_haz_w : (d_valid && (di_rdy_haz_w || !di_valid));
	wire [32:0] di_haz_tag_w = !d_ready ? r_skid_tag : {d_rt, d_xy[31:17], d_xy[15:1]};

	atg_rop_hazard_pipe #(
		.NUM_ENTRIES(16),
		.TAG_W(33)
	) u_hazard (
		.clk(clk),
		.rst_n(rst_n),
		.pre_load(di_rdy_haz_w),
		.pre_load_vec(pre_load_vec_w),
		.pre_tag({di_rt, di_xy[31:17], di_xy[15:1]}),
		.di_load(di_haz_load_w),
		.di_pre_tag(di_haz_tag_w),
		.alloc_valid(stream_cand_haz),
		.alloc_ready(alloc_ready),
		.alloc_tag({s1_rt, s1_xy[31:17], s1_xy[15:1]}),
		.alloc_id(alloc_id),
		.o_grp_hit(haz_grp_hit),
		.release_valid(release_valid),
		.release_id(release_id),
		.release_oh(release_oh),
		.next_release_valid(next_release_valid),
		.next_release_id(next_release_id),
		.next_release_oh(next_release_oh),
		.o_vhit_any(haz_vhit_any),
		.o_vhit_id(haz_vhit_id),
		.o_haz_clear(haz_clear_w),
		.o_haz_clear_rep(haz_clear_rep),
		.o_cam_full(haz_cam_full),
		.o_cam_full_rep(haz_cam_full_rep),
		.o_hv_clear_rep(haz_hv_clear_rep),
		.o_v16_rep(haz_v16_rep),
		.o_vrel_or0_rep(haz_vrel_or0_rep),
		.o_vrel_or1_rep(haz_vrel_or1_rep),
		.o_arel_or0_rep(haz_arel_or0_rep),
		.o_arel_or1_rep(haz_arel_or1_rep),
		.o_vhit_a_rep(haz_vhit_a_rep)
	);

	wire [3:0] s0_mask = {s1_smask[12], s1_smask[8], s1_smask[4], s1_smask[0]};
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire b_valid_w =
		(stream_gate_sh && sh_grp2_cand && !haz_grp_hit[0] && !haz_grp_hit[1]) || fsm_b_valid;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire b_valid_w =
		(stream_gate_sh && sh_grp2_cand && !haz_grp_hit[0] && !haz_grp_hit[1]) || fsm_b_valid;
`else
	(* keep = "true" *) wire b_valid_w = is_idle_reg ? do_stream : fsm_b_valid;
`endif
	assign b_valid = b_valid_w;
`ifdef SYNTH_KINTEX7
	assign b_mask = idle_grp[3] ? s0_mask : r_bmask;
	wire [2:0] eff_rt = idle_grp[4] ? s1_rt : rt;
`elsif SYNTH_KINTEX_ULTRASCALE
	assign b_mask = idle_grp[3] ? s0_mask : r_bmask;
	wire [2:0] eff_rt = idle_grp[4] ? s1_rt : rt;
`else
	assign b_mask = is_idle_reg ? s0_mask : r_bmask;
	wire [2:0] eff_rt = is_idle_rt ? s1_rt_blend : rt_blend;
`endif
`ifdef SYNTH_KINTEX7
	wire [2:0] eff_rt_blend;
	genvar g_rtb;
	generate
	for (g_rtb = 0; g_rtb < 3; g_rtb = g_rtb + 1) begin : g_rt_blend
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rt_b (.I0(eff_rt[g_rtb]), .O(eff_rt_blend[g_rtb]));
	end
	endgenerate
	assign b_rt = eff_rt_blend;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] eff_rt_blend;
	genvar g_rtb;
	generate
	for (g_rtb = 0; g_rtb < 3; g_rtb = g_rtb + 1) begin : g_rt_blend
		assign eff_rt_blend[g_rtb] = eff_rt[g_rtb];
	end
	endgenerate
	assign b_rt = eff_rt_blend;
`else
	assign b_rt = eff_rt;
`endif

	wire p_ready = (r_bl_path == 2'd2 && bwait_d1) || (r_bl_path == 2'd0 && b_rvalid) || f_done;

	/* Pure-LUT LA_W-bit tag equality and dirty check for FSM (0 CARRY4s, <= 3 LUT levels) */
	localparam integer N_HEQ = (LA_W + 2) / 3;
	localparam integer H_CMP_W = N_HEQ * 3;
	wire [63:0] et_pad = {{(64-LA_W){1'b0}}, etag[r_tag]};
	wire [63:0] ad_pad = {{(64-LA_W){1'b0}}, addr_ext[LINE_SHIFT +: LA_W]};
	wire [H_CMP_W-1:0] et_cmp = et_pad[H_CMP_W-1:0];
	wire [H_CMP_W-1:0] ad_cmp = ad_pad[H_CMP_W-1:0];
	(* keep = "true" *) wire [N_HEQ-1:0] hit_eq3;
	genvar gh;
	generate
	for (gh = 0; gh < N_HEQ; gh = gh + 1) begin : g_hit_eq3
		assign hit_eq3[gh] = (et_cmp[3*gh +: 3] == ad_cmp[3*gh +: 3]);
	end
	endgenerate
	(* keep = "true" *) wire hit_g0 = &hit_eq3[N_HEQ/2-1:0];
	(* keep = "true" *) wire hit_g1 = &hit_eq3[N_HEQ-1:N_HEQ/2];
	wire hit = ev[r_tag] && hit_g0 && hit_g1 && (erd[r_tag] || !r_row_read);

	localparam integer N_DGRP = (LINE_BYTES + 5) / 6;
	localparam integer D_HALF = LINE_BYTES / 2;
	wire [LINE_BYTES-1:0] cur_edirty = {
		r_dty_hi ? edirty1[LINE_BYTES-1:D_HALF] : edirty0[LINE_BYTES-1:D_HALF],
		r_dty_lo ? edirty1[D_HALF-1:0]          : edirty0[D_HALF-1:0]
	};
	(* keep = "true" *) wire [N_DGRP-1:0] dirty_or6;
	generate
	for (gh = 0; gh < N_DGRP; gh = gh + 1) begin : g_dirty_or
		localparam integer D_LO = 6 * gh;
		localparam integer D_W = (D_LO + 6 <= LINE_BYTES) ? 6 : (LINE_BYTES - D_LO);
		assign dirty_or6[gh] = |cur_edirty[D_LO +: D_W];
	end
	endgenerate
	wire edirty_nz = |dirty_or6;

	localparam integer N_PRED = LINE_BYTES / 2;
`ifdef SYNTH_KINTEX7
	wire [1:0] p_read_mid;
	wire [2*N_PRED-1:0] p_read_edat;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_prdm0 (.I0(r_row_read), .O(p_read_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_prdm1 (.I0(r_row_read), .O(p_read_mid[1]));
	genvar g_pred;
	generate
	for (g_pred = 0; g_pred < 2 * N_PRED; g_pred = g_pred + 1) begin : g_pred_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_prd (
			.I0(p_read_mid[g_pred / N_PRED]), .O(p_read_edat[g_pred])
		);
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] p_read_mid;
	wire [2*N_PRED-1:0] p_read_edat;
	assign p_read_mid[0] = r_row_read;
	assign p_read_mid[1] = r_row_read;
	genvar g_pred;
	generate
	for (g_pred = 0; g_pred < 2 * N_PRED; g_pred = g_pred + 1) begin : g_pred_buf
		assign p_read_edat[g_pred] = p_read_mid[g_pred / N_PRED];
	end
	endgenerate
`endif

	/* Flat di_ready: parallel pre-qualified stream check + hazard clear (<= 11 cmos3 into skid) */
	assign s1_d_ready_w = idle_or_drop || stream_hit_free;
`ifdef SYNTH_KINTEX7
	assign d_skid_ready = di_ready_rep[129];
	assign d_skid_ready_vec = di_ready_rep[99:58];
`elsif SYNTH_KINTEX_ULTRASCALE
	assign d_skid_ready = di_ready_rep[129];
	assign d_skid_ready_vec = di_ready_rep[99:58];
`else
	assign d_skid_ready = di_ready_rep[4];
	assign d_skid_ready_vec = {42{di_ready_rep[4]}};
`endif
	assign busy = (st != S_IDLE) || ev[0] || ev[1] || pipe_busy || di_valid || s1_v;
	assign req_tag = 8'd0;

	/* ---------------------------------------------------------------------
	 * 3-Stage Commit Pipeline (Stage A: pre_wr* -> Stage B: commit_* -> Stage C: edat)
	 * Shared between streaming (pipe_v[9] -> pipe_v[10] -> commit_v)
	 * and FSM (S_BWAIT !bwait_d1 -> S_BWAIT p_ready -> S_BCOMMIT).
	 * --------------------------------------------------------------------- */
`ifdef SYNTH_KINTEX7
	wire [1:0] pre_in_ps = pipe_v[9] ? pipe_ps[9] : r_bps;
	wire pre_is_u8 = pipe_v[9] || (r_bl_path == 2'd0);
	wire [1:0] pre_u8_sel_in = !pre_is_u8 ? 2'd0 : (pre_in_ps[0] ? 2'd1 : (pre_in_ps[1] ? 2'd2 : 2'd3));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] pre_in_ps = pipe_v[9] ? pipe_ps[9] : r_bps;
	wire pre_is_u8 = pipe_v[9] || (r_bl_path == 2'd0);
	wire [1:0] pre_u8_sel_in = !pre_is_u8 ? 2'd0 : (pre_in_ps[0] ? 2'd1 : (pre_in_ps[1] ? 2'd2 : 2'd3));
`else
	wire [1:0] pre_u8_sel_in = (r_bl_path != 2'd0) ? 2'd0 : (r_bps[0] ? 2'd1 : (r_bps[1] ? 2'd2 : 2'd3));
	wire [1:0] str_u8_sel_in = pipe_ps[8][0] ? 2'd1 : (pipe_ps[8][1] ? 2'd2 : 2'd3);
`endif
	wire [1:0] pre_u8_sel_in_b [0:3];
`ifdef SYNTH_KINTEX7
	genvar g_u8b;
	generate
	for (g_u8b = 0; g_u8b < 4; g_u8b = g_u8b + 1) begin : g_u8_sel_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_u8s0 (.I0(pre_u8_sel_in[0]), .O(pre_u8_sel_in_b[g_u8b][0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_u8s1 (.I0(pre_u8_sel_in[1]), .O(pre_u8_sel_in_b[g_u8b][1]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_u8b;
	generate
	for (g_u8b = 0; g_u8b < 4; g_u8b = g_u8b + 1) begin : g_u8_sel_buf
		assign pre_u8_sel_in_b[g_u8b][0] = pre_u8_sel_in[0];
		assign pre_u8_sel_in_b[g_u8b][1] = pre_u8_sel_in[1];
	end
	endgenerate
`else
	assign pre_u8_sel_in_b[0] = pre_u8_sel_in;
	assign pre_u8_sel_in_b[1] = pre_u8_sel_in;
	assign pre_u8_sel_in_b[2] = pre_u8_sel_in;
	assign pre_u8_sel_in_b[3] = pre_u8_sel_in;
`endif

	/* Replicate pre_in_* into 8 groups of 4 bytes via explicit LUTs so fanouts <= 16 */
	localparam integer N_BGRP = (LINE_BYTES + 3) / 4;
	(* keep = "true", dont_touch = "true" *) wire [3:0] pre_mask_g [0:N_BGRP-1];
	(* keep = "true", dont_touch = "true" *) wire [4*LINE_SHIFT-1:0] pre_soff_g [0:N_BGRP-1];
	(* keep = "true", dont_touch = "true" *) wire [3:0] pre_ent_g [0:N_BGRP-1];
	(* keep = "true", dont_touch = "true" *) wire [3:0] pre_lm_g [0:N_BGRP-1];

`ifdef SYNTH_KINTEX7
	wire [1:0] pv9_mid;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pv9m0 (.I0(pipe_v[9]), .O(pv9_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pv9m1 (.I0(pipe_v[9]), .O(pv9_mid[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] pv9_mid;
	assign pv9_mid[0] = pipe_v[9];
	assign pv9_mid[1] = pipe_v[9];
`endif

	genvar gbg, gb, gbi;
	generate
	for (gbg = 0; gbg < N_BGRP; gbg = gbg + 1) begin : g_pre_grp
		(* keep = "true", dont_touch = "true" *) wire [3:0] lm_g;
`ifdef SYNTH_KINTEX7
		wire pv9_a, pv9_b;
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pva (.I0(pv9_mid[gbg / 4]), .O(pv9_a));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pvb (.I0(pv9_mid[gbg / 4]), .O(pv9_b));
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_pm_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_mk (
				.I0(r_bmask[gbi]), .I1(pipe_mask[9][gbi]), .I2(pv9_a), .O(pre_mask_g[gbg][gbi])
			);
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_en (
				.I0(r_bent[gbi]), .I1(pipe_ent[9][gbi]), .I2(pv9_a), .O(pre_ent_g[gbg][gbi])
			);
		end
		for (gbi = 0; gbi < 4 * LINE_SHIFT; gbi = gbi + 1) begin : g_so_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_so (
				.I0(r_bsoff[gbi]), .I1(pipe_soff[9][gbi]), .I2((gbi < 8) ? pv9_a : pv9_b), .O(pre_soff_g[gbg][gbi])
			);
		end
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_lm_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_lm (
				.I0(r_blm[gbi]), .I1(pipe_lm[9][gbi]), .I2(pv9_b), .O(lm_g[gbi])
			);
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		wire pv9_a, pv9_b;
		assign pv9_a = pv9_mid[gbg / 4];
		assign pv9_b = pv9_mid[gbg / 4];
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_pm_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_mk (
				.I0(r_bmask[gbi]), .I1(pipe_mask[9][gbi]), .I2(pv9_a), .O(pre_mask_g[gbg][gbi])
			);
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_en (
				.I0(r_bent[gbi]), .I1(pipe_ent[9][gbi]), .I2(pv9_a), .O(pre_ent_g[gbg][gbi])
			);
		end
		for (gbi = 0; gbi < 4 * LINE_SHIFT; gbi = gbi + 1) begin : g_so_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_so (
				.I0(r_bsoff[gbi]), .I1(pipe_soff[9][gbi]), .I2((gbi < 8) ? pv9_a : pv9_b), .O(pre_soff_g[gbg][gbi])
			);
		end
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_lm_lut
			(* keep = "true", dont_touch = "true" *) LUT3 #(.INIT(8'hCA)) u_lm (
				.I0(r_blm[gbi]), .I1(pipe_lm[9][gbi]), .I2(pv9_b), .O(lm_g[gbi])
			);
		end
`else
		assign pre_mask_g[gbg] = r_bmask;
		assign pre_soff_g[gbg] = r_bsoff;
		assign pre_ent_g[gbg]  = r_bent;
		assign lm_g = r_blm;
`endif
		assign pre_lm_g[gbg] = lm_g;
	end
	endgenerate

`ifdef SYNTH_KINTEX7
	wire [3:0] pre_lm_byte [0:LINE_BYTES-1];
	generate
	for (gb = 0; gb < LINE_BYTES; gb = gb + 1) begin : g_plm_b
		localparam integer G_B = gb / 4;
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_plmb
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_plmb (.I0(pre_lm_g[G_B][gbi]), .O(pre_lm_byte[gb][gbi]));
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] pre_lm_byte [0:LINE_BYTES-1];
	generate
	for (gb = 0; gb < LINE_BYTES; gb = gb + 1) begin : g_plm_b
		localparam integer G_B = gb / 4;
		for (gbi = 0; gbi < 4; gbi = gbi + 1) begin : g_plmb
			assign pre_lm_byte[gb][gbi] = pre_lm_g[G_B][gbi];
		end
	end
	endgenerate
`endif

	wire [LINE_BYTES-1:0] pre_wr0_en_w, pre_wr1_en_w;
	wire [2*LINE_BYTES-1:0] pre_wr0_sel_w, pre_wr1_sel_w;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	wire [LINE_BYTES-1:0] str_wr0_en_w, str_wr1_en_w;
	wire [2*LINE_BYTES-1:0] str_wr0_sel_w, str_wr1_sel_w;
	reg [LINE_BYTES-1:0] r_str_wr0_en, r_str_wr1_en;
	reg [2*LINE_BYTES-1:0] r_str_wr0_sel, r_str_wr1_sel;
	reg [1:0] r_str_u8_sel;
`endif
	generate
	for (gb = 0; gb < LINE_BYTES; gb = gb + 1) begin : g_pre_dec
		localparam integer G_B = gb / 4;
		localparam [LINE_SHIFT-1:0] B_OFF = gb;
		wire [3:0] mk_g = pre_mask_g[G_B];
		wire [4*LINE_SHIFT-1:0] so_g = pre_soff_g[G_B];
		wire [3:0] en_g = pre_ent_g[G_B];
`ifdef SYNTH_KINTEX7
		wire [3:0] lm_v = pre_lm_byte[gb];
`elsif SYNTH_KINTEX_ULTRASCALE
		wire [3:0] lm_v = pre_lm_byte[gb];
`else
		wire [3:0] lm_v = pre_lm_g[G_B];
`endif

		wire [LINE_SHIFT-1:0] so0 = so_g[0 +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] so1 = so_g[LINE_SHIFT +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] so2 = so_g[2*LINE_SHIFT +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] so3 = so_g[3*LINE_SHIFT +: LINE_SHIFT];

		wire m0 = mk_g[0] && (so0[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (lm_v[0] ||  so0[0]) : (lm_v[0] || !so0[0])) &&
			(B_OFF[1] ? (lm_v[1] ||  so0[1]) : (lm_v[1] || !so0[1])) &&
			(B_OFF[2] ? (lm_v[2] ||  so0[2]) : (lm_v[2] || !so0[2])) &&
			(B_OFF[3] ? (lm_v[3] ||  so0[3]) : (lm_v[3] || !so0[3]));
		wire m1 = mk_g[1] && (so1[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (lm_v[0] ||  so1[0]) : (lm_v[0] || !so1[0])) &&
			(B_OFF[1] ? (lm_v[1] ||  so1[1]) : (lm_v[1] || !so1[1])) &&
			(B_OFF[2] ? (lm_v[2] ||  so1[2]) : (lm_v[2] || !so1[2])) &&
			(B_OFF[3] ? (lm_v[3] ||  so1[3]) : (lm_v[3] || !so1[3]));
		wire m2 = mk_g[2] && (so2[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (lm_v[0] ||  so2[0]) : (lm_v[0] || !so2[0])) &&
			(B_OFF[1] ? (lm_v[1] ||  so2[1]) : (lm_v[1] || !so2[1])) &&
			(B_OFF[2] ? (lm_v[2] ||  so2[2]) : (lm_v[2] || !so2[2])) &&
			(B_OFF[3] ? (lm_v[3] ||  so2[3]) : (lm_v[3] || !so2[3]));
		wire m3 = mk_g[3] && (so3[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (lm_v[0] ||  so3[0]) : (lm_v[0] || !so3[0])) &&
			(B_OFF[1] ? (lm_v[1] ||  so3[1]) : (lm_v[1] || !so3[1])) &&
			(B_OFF[2] ? (lm_v[2] ||  so3[2]) : (lm_v[2] || !so3[2])) &&
			(B_OFF[3] ? (lm_v[3] ||  so3[3]) : (lm_v[3] || !so3[3]));

		wire w0_0 = m0 && !en_g[0];
		wire w0_1 = m1 && !en_g[1];
		wire w0_2 = m2 && !en_g[2];
		wire w0_3 = m3 && !en_g[3];

		wire w1_0 = m0 && en_g[0];
		wire w1_1 = m1 && en_g[1];
		wire w1_2 = m2 && en_g[2];
		wire w1_3 = m3 && en_g[3];

		assign pre_wr0_en_w[gb] = w0_0 | w0_1 | w0_2 | w0_3;
		assign pre_wr1_en_w[gb] = w1_0 | w1_1 | w1_2 | w1_3;

		assign pre_wr0_sel_w[2*gb + 1] = w0_3 || w0_2;
		assign pre_wr0_sel_w[2*gb + 0] = w0_3 || (!w0_2 && w0_1);
		assign pre_wr1_sel_w[2*gb + 1] = w1_3 || w1_2;
		assign pre_wr1_sel_w[2*gb + 0] = w1_3 || (!w1_2 && w1_1);
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
		wire [LINE_SHIFT-1:0] ss0 = pipe_soff[8][0 +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] ss1 = pipe_soff[8][LINE_SHIFT +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] ss2 = pipe_soff[8][2*LINE_SHIFT +: LINE_SHIFT];
		wire [LINE_SHIFT-1:0] ss3 = pipe_soff[8][3*LINE_SHIFT +: LINE_SHIFT];
		wire [3:0] smk = pipe_mask[8];
		wire [3:0] sen = pipe_ent[8];
		wire [3:0] slm = pipe_lm[8];
		wire sm0 = smk[0] && (ss0[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (slm[0] ||  ss0[0]) : (slm[0] || !ss0[0])) &&
			(B_OFF[1] ? (slm[1] ||  ss0[1]) : (slm[1] || !ss0[1])) &&
			(B_OFF[2] ? (slm[2] ||  ss0[2]) : (slm[2] || !ss0[2])) &&
			(B_OFF[3] ? (slm[3] ||  ss0[3]) : (slm[3] || !ss0[3]));
		wire sm1 = smk[1] && (ss1[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (slm[0] ||  ss1[0]) : (slm[0] || !ss1[0])) &&
			(B_OFF[1] ? (slm[1] ||  ss1[1]) : (slm[1] || !ss1[1])) &&
			(B_OFF[2] ? (slm[2] ||  ss1[2]) : (slm[2] || !ss1[2])) &&
			(B_OFF[3] ? (slm[3] ||  ss1[3]) : (slm[3] || !ss1[3]));
		wire sm2 = smk[2] && (ss2[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (slm[0] ||  ss2[0]) : (slm[0] || !ss2[0])) &&
			(B_OFF[1] ? (slm[1] ||  ss2[1]) : (slm[1] || !ss2[1])) &&
			(B_OFF[2] ? (slm[2] ||  ss2[2]) : (slm[2] || !ss2[2])) &&
			(B_OFF[3] ? (slm[3] ||  ss2[3]) : (slm[3] || !ss2[3]));
		wire sm3 = smk[3] && (ss3[LINE_SHIFT-1:4] == B_OFF[LINE_SHIFT-1:4]) &&
			(B_OFF[0] ? (slm[0] ||  ss3[0]) : (slm[0] || !ss3[0])) &&
			(B_OFF[1] ? (slm[1] ||  ss3[1]) : (slm[1] || !ss3[1])) &&
			(B_OFF[2] ? (slm[2] ||  ss3[2]) : (slm[2] || !ss3[2])) &&
			(B_OFF[3] ? (slm[3] ||  ss3[3]) : (slm[3] || !ss3[3]));
		wire sw0_0 = sm0 && !sen[0], sw0_1 = sm1 && !sen[1], sw0_2 = sm2 && !sen[2], sw0_3 = sm3 && !sen[3];
		wire sw1_0 = sm0 &&  sen[0], sw1_1 = sm1 &&  sen[1], sw1_2 = sm2 &&  sen[2], sw1_3 = sm3 &&  sen[3];
		assign str_wr0_en_w[gb] = sw0_0 | sw0_1 | sw0_2 | sw0_3;
		assign str_wr1_en_w[gb] = sw1_0 | sw1_1 | sw1_2 | sw1_3;
		assign str_wr0_sel_w[2*gb + 1] = sw0_3 || sw0_2;
		assign str_wr0_sel_w[2*gb + 0] = sw0_3 || (!sw0_2 && sw0_1);
		assign str_wr1_sel_w[2*gb + 1] = sw1_3 || sw1_2;
		assign str_wr1_sel_w[2*gb + 0] = sw1_3 || (!sw1_2 && sw1_1);
`endif
	end
	endgenerate

	reg [LINE_BYTES-1:0] pre_wr0_en, pre_wr1_en;
	reg [2*LINE_BYTES-1:0] pre_wr0_sel, pre_wr1_sel;
	(* keep = "true", dont_touch = "true" *) reg [1:0] pre_u8_sel_rep [0:31];

	/* Stage B: 2-LUT-level slot byte mux and per-byte cache line mux (fanout <= 16) */
	wire use_stream_res = pipe_v[10] && b_rvalid;
	(* keep = "true" *) wire [127:0] m_raw [0:3];
`ifdef SYNTH_KINTEX7
	generate
	for (gb = 0; gb < 4; gb = gb + 1) begin : g_rep_gen
		genvar gq;
		for (gq = 0; gq < 16; gq = gq + 1) begin : g_m_slice
			wire [1:0] u8_sel = pre_u8_sel_rep[8 * gb + (gq / 2)];
			assign m_raw[gb][8 * gq +: 8] =
				(u8_sel == 2'd3) ? b_pix[32 * gb + 8 * (gq % 4) +: 8] :
				(u8_sel == 2'd2) ? b_pix[32 * gb + 8 * (gq % 2) +: 8] :
				(u8_sel == 2'd1) ? b_pix[32 * gb +: 8] :
				                   p_res[128 * gb + 8 * gq +: 8];
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	generate
	for (gb = 0; gb < 4; gb = gb + 1) begin : g_rep_gen
		genvar gq;
		for (gq = 0; gq < 16; gq = gq + 1) begin : g_m_slice
			wire [1:0] u8_sel = pre_u8_sel_rep[8 * gb + (gq / 2)];
			assign m_raw[gb][8 * gq +: 8] =
				(u8_sel == 2'd3) ? b_pix[32 * gb + 8 * (gq % 4) +: 8] :
				(u8_sel == 2'd2) ? b_pix[32 * gb + 8 * (gq % 2) +: 8] :
				(u8_sel == 2'd1) ? b_pix[32 * gb +: 8] :
				                   p_res[128 * gb + 8 * gq +: 8];
		end
	end
	endgenerate
`else
	generate
	for (gb = 0; gb < 4; gb = gb + 1) begin : g_rep_gen
		wire [1:0] u8_sel = pre_u8_sel_rep[0];
		assign m_raw[gb] =
			(u8_sel == 2'd3) ? {4{b_pix[32 * gb +: 32]}} :
			(u8_sel == 2'd2) ? {8{b_pix[32 * gb +: 16]}} :
			(u8_sel == 2'd1) ? {16{b_pix[32 * gb +: 8]}} :
			                   p_res[128 * gb +: 128];
	end
	endgenerate
`endif

	wire [LINE_BYTES*8-1:0] wdata0_pre_comb, wdata1_pre_comb;
	generate
	for (gb = 0; gb < LINE_BYTES; gb = gb + 1) begin : g_wdata_pre
		wire [1:0] s0 = pre_wr0_sel[2 * gb +: 2];
		wire [1:0] s1 = pre_wr1_sel[2 * gb +: 2];
		assign wdata0_pre_comb[8 * gb +: 8] = m_raw[s0][8 * (gb % 16) +: 8];
		assign wdata1_pre_comb[8 * gb +: 8] = m_raw[s1][8 * (gb % 16) +: 8];
	end
	endgenerate

	reg [LINE_BYTES-1:0] commit_wr0_en, commit_wr1_en;
	reg [LINE_BYTES*8-1:0] commit_wdata0, commit_wdata1;
	(* max_fanout = 16 *) reg [1:0] g_fsm;
	wire [1:0] p_multi_b;
	wire [1:0] p_xsh_b [0:3];
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pmb0 (.I0(p_multi), .O(p_multi_b[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pmb1 (.I0(p_multi), .O(p_multi_b[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px00 (.I0(p_xsh[0]), .O(p_xsh_b[0][0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px01 (.I0(p_xsh[1]), .O(p_xsh_b[0][1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px10 (.I0(p_xsh[0]), .O(p_xsh_b[1][0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px11 (.I0(p_xsh[1]), .O(p_xsh_b[1][1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px20 (.I0(p_xsh[0]), .O(p_xsh_b[2][0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px21 (.I0(p_xsh[1]), .O(p_xsh_b[2][1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px30 (.I0(p_xsh[0]), .O(p_xsh_b[3][0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_px31 (.I0(p_xsh[1]), .O(p_xsh_b[3][1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign p_multi_b[0] = p_multi;
	assign p_multi_b[1] = p_multi;
	assign p_xsh_b[0][0] = p_xsh[0];
	assign p_xsh_b[0][1] = p_xsh[1];
	assign p_xsh_b[1][0] = p_xsh[0];
	assign p_xsh_b[1][1] = p_xsh[1];
	assign p_xsh_b[2][0] = p_xsh[0];
	assign p_xsh_b[2][1] = p_xsh[1];
	assign p_xsh_b[3][0] = p_xsh[0];
	assign p_xsh_b[3][1] = p_xsh[1];
`else
	assign p_multi_b = {2{p_multi}};
	assign p_xsh_b[0] = p_xsh[1:0];
	assign p_xsh_b[1] = p_xsh[1:0];
	assign p_xsh_b[2] = p_xsh[1:0];
	assign p_xsh_b[3] = p_xsh[1:0];
`endif

	task next_row;
		begin
			if (p_multi_b[0] && j != p_nl) begin
				j <= j1;
				j_wpix <= j1;
			end else if (!r) begin
				r <= 1'b1;
				j <= 2'd0;
				j_wpix <= 2'd0;
			end else if (p_multi_b[0]) begin
				r <= 1'b0;
				j <= 2'd0;
				j_wpix <= 2'd0;
				g <= 2'd0;
				g_fsm <= 2'd0;
			end else begin
				g <= 2'd0;
				g_fsm <= 2'd0;
			end
		end
	endtask

	task line_in;
		begin
			if (p_multi_b[0]) begin
				g <= 2'd0;
				g_fsm <= 2'd0;
			end else begin
				r <= 1'b1;
				g <= 2'd0;
				g_fsm <= 2'd0;
			end
		end
	endtask

	task pass_done;
		begin
			if (!p_multi_b[1] && g_fsm != p_np) begin
				g <= g1;
				g_fsm <= g1;
			end else if (!p_multi_b[1]) begin
				r <= 1'b0;
				j <= 2'd0;
				g <= 2'd0;
				g_fsm <= 2'd0;
			end else begin
				next_row;
			end
		end
	endtask

	integer l, b_idx, ps_i;
	wire latch_pre_en = pipe_v[9] || (st == S_BWAIT && !bwait_d1);
	wire latch_commit_en = (pipe_v[10] && b_rvalid) || (st == S_BWAIT && p_ready);

	/* Buffered enables for high-fanout data registers */
	wire fill_done_w = r_st_fill && (!r_row_read || (!req_valid && wait_rsp && rsp_valid));
	wire fill_we0_w = fill_done_w && !r_fill;
	wire fill_we1_w = fill_done_w && r_fill;
	wire req_wr_go_w = (r_st_look && !r_look_hit && r_edirty_nz) || (r_st_flush && r_edirty_nz);
	wire req_rd_go_w = (r_st_look && !r_look_hit && !r_edirty_nz && r_row_read) ||
	                   (r_st_evw && !req_valid && !wait_rsp && r_row_read);
	wire req_go_w = req_wr_go_w || req_rd_go_w;
	wire en_s_blend_go = (st == S_BLEND) && !r_pmask_z;
	/* Rule 1 fix: color/xy/mask are only read when st != S_IDLE, so latch whenever is_idle_reg && s1_v (1 LUT) */
`ifdef SYNTH_KINTEX7
	wire en_s_idle_miss = idle_grp[10] && s1_v;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire en_s_idle_miss = idle_grp[10] && s1_v;
`else
	wire en_s_idle_miss = is_idle_reg && s1_v;
`endif
	wire en_s_row_w = (st == S_ROW);
	wire en_cur_rb_w = (st == S_ROW_WAIT6) || (st == S_ROW_PREP2);
	wire en_bprep2_w = (st == S_BLEND_PREP2) && !r_bl_multi && (g_fsm == 2'd0);
	wire ed_clr_evw_flw = ((st == S_EVW) || (st == S_FLW)) && !req_valid && !wait_rsp;
`ifdef SYNTH_KINTEX7
	wire ed0_clr_w = rst_sync || fill_we0_w || (ed_clr_evw_flw && !r);
	wire ed1_clr_w = rst_sync || fill_we1_w || (ed_clr_evw_flw && r);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire ed0_clr_w = rst_sync || fill_we0_w || (ed_clr_evw_flw && !r);
	wire ed1_clr_w = rst_sync || fill_we1_w || (ed_clr_evw_flw && r);
`else
	wire ed0_clr_w = fill_we0_w || (ed_clr_evw_flw && !r);
	wire ed1_clr_w = fill_we1_w || (ed_clr_evw_flw && r);
`endif

	localparam integer N_BE16  = LINE_BYTES / 16;
	localparam integer N_SEL16 = LINE_BYTES / 8;
	localparam integer N_LCE   = 2 * N_PRED + 2 * N_BE16 + 1;
	localparam integer N_LPE   = 2 * N_BE16 + 2 * N_SEL16 + 4;
	localparam integer N_RGO   = N_BE16 + 2;
	localparam integer N_RWR   = N_BE16 + 2;

	(* dont_touch = "true" *) reg [N_PRED-1:0] r_wdata_ce_r;

	wire [1:0] en_s_row_buf, en_crb_buf, etag0_we_buf, etag1_we_buf;
	wire [1:0] crb_w5_buf, crb_r_buf;
	wire [5:0] en_bp2_leaf;
	wire [N_BE16-1:0] ed0_clr_buf, ed1_clr_buf;

`ifdef SYNTH_KINTEX7
	wire [1:0] fill_we0_mid, fill_we1_mid;
	wire [N_PRED-1:0] fill_we0_leaf, fill_we1_leaf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw0m0 (.I0(fill_we0_w), .O(fill_we0_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw0m1 (.I0(fill_we0_w), .O(fill_we0_mid[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw1m0 (.I0(fill_we1_w), .O(fill_we1_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw1m1 (.I0(fill_we1_w), .O(fill_we1_mid[1]));

	wire [2:0] lce_mid;
	wire [N_LCE-1:0] lce_leaf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lcem0 (.I0(latch_commit_en), .O(lce_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lcem1 (.I0(latch_commit_en), .O(lce_mid[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lcem2 (.I0(latch_commit_en), .O(lce_mid[2]));

	wire [1:0] lpe_mid;
	wire [N_LPE-1:0] lpe_leaf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lpem0 (.I0(latch_pre_en), .O(lpe_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lpem1 (.I0(latch_pre_en), .O(lpe_mid[1]));

	wire [1:0] req_go_mid, r_wdata_mid;
	wire [N_RGO-1:0] req_go_leaf;
	wire [N_PRED-1:0] r_wdata_leaf;
	wire [N_RWR-1:0] req_wr_leaf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rgm0 (.I0(req_go_w), .O(req_go_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rgm1 (.I0(req_go_w), .O(req_go_mid[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rwm0 (.I0(r), .O(r_wdata_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rwm1 (.I0(r), .O(r_wdata_mid[1]));

	wire [2:0] ebl_mid, eid_mid;
	wire [41:0] ebl_leaf;
	wire [35:0] eid_leaf;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eblm0 (.I0(en_s_blend_go), .O(ebl_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eblm1 (.I0(en_s_blend_go), .O(ebl_mid[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eblm2 (.I0(en_s_blend_go), .O(ebl_mid[2]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eidm0 (.I0(en_s_idle_miss), .O(eid_mid[0]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eidm1 (.I0(en_s_idle_miss), .O(eid_mid[1]));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eidm2 (.I0(en_s_idle_miss), .O(eid_mid[2]));

	wire is_w5_raw = (st == S_ROW_WAIT6);
	wire [8:0] str_ce_leaf;
	wire do_stream_drep, do_stream_bdst, do_stream_pv;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dstrd (.I0(do_stream), .O(do_stream_drep));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dstrb (.I0(do_stream), .O(do_stream_bdst));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_dstrp (.I0(do_stream), .O(do_stream_pv));
	for (gb = 0; gb < 9; gb = gb + 1) begin : g_str_ce
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_strce (.I0(do_stream), .O(str_ce_leaf[gb]));
	end
	generate
	for (gb = 0; gb < 2; gb = gb + 1) begin : g_row_ce_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_srow  (.I0(en_s_row_w),  .O(en_s_row_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_crb   (.I0(en_cur_rb_w), .O(en_crb_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_crbw5 (.I0(is_w5_raw),   .O(crb_w5_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_crbr  (.I0(r),           .O(crb_r_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_et0   (.I0(fill_we0_w),  .O(etag0_we_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_et1   (.I0(fill_we1_w),  .O(etag1_we_buf[gb]));
	end
	for (gb = 0; gb < 6; gb = gb + 1) begin : g_bp2_ce_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bp2 (.I0(en_bprep2_w), .O(en_bp2_leaf[gb]));
	end
	for (gb = 0; gb < N_BE16; gb = gb + 1) begin : g_edclr_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_edc0 (.I0(ed0_clr_w), .O(ed0_clr_buf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_edc1 (.I0(ed1_clr_w), .O(ed1_clr_buf[gb]));
	end
	for (gb = 0; gb < N_PRED; gb = gb + 1) begin : g_fill_we_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw0l (.I0(fill_we0_mid[gb / (N_PRED / 2)]), .O(fill_we0_leaf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fw1l (.I0(fill_we1_mid[gb / (N_PRED / 2)]), .O(fill_we1_leaf[gb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rwl  (.I0(r_wdata_mid[gb / (N_PRED / 2)]),  .O(r_wdata_leaf[gb]));
	end
	for (gb = 0; gb < N_LCE; gb = gb + 1) begin : g_lce_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lcel (.I0(lce_mid[(gb * 3) / N_LCE]), .O(lce_leaf[gb]));
	end
	for (gb = 0; gb < N_LPE; gb = gb + 1) begin : g_lpe_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lpel (.I0(lpe_mid[gb / (N_LPE / 2)]), .O(lpe_leaf[gb]));
	end
	for (gb = 0; gb < N_RGO; gb = gb + 1) begin : g_req_go_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rgl (.I0(req_go_mid[gb / (N_RGO / 2)]), .O(req_go_leaf[gb]));
	end
	for (gb = 0; gb < N_RWR; gb = gb + 1) begin : g_req_wr_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rwrl (.I0(req_wr_go_w), .O(req_wr_leaf[gb]));
	end
	for (gb = 0; gb < 42; gb = gb + 1) begin : g_ebl_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ebll (.I0(ebl_mid[gb / 14]), .O(ebl_leaf[gb]));
	end
	for (gb = 0; gb < 36; gb = gb + 1) begin : g_eid_leaf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eidl (.I0(eid_mid[gb / 12]), .O(eid_leaf[gb]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] fill_we0_mid, fill_we1_mid;
	wire [N_PRED-1:0] fill_we0_leaf, fill_we1_leaf;
	assign fill_we0_mid[0] = fill_we0_w;
	assign fill_we0_mid[1] = fill_we0_w;
	assign fill_we1_mid[0] = fill_we1_w;
	assign fill_we1_mid[1] = fill_we1_w;

	wire [2:0] lce_mid;
	wire [N_LCE-1:0] lce_leaf;
	assign lce_mid[0] = latch_commit_en;
	assign lce_mid[1] = latch_commit_en;
	assign lce_mid[2] = latch_commit_en;

	wire [1:0] lpe_mid;
	wire [N_LPE-1:0] lpe_leaf;
	assign lpe_mid[0] = latch_pre_en;
	assign lpe_mid[1] = latch_pre_en;

	wire [1:0] req_go_mid, r_wdata_mid;
	wire [N_RGO-1:0] req_go_leaf;
	wire [N_PRED-1:0] r_wdata_leaf;
	wire [N_RWR-1:0] req_wr_leaf;
	assign req_go_mid[0] = req_go_w;
	assign req_go_mid[1] = req_go_w;
	assign r_wdata_mid[0] = r;
	assign r_wdata_mid[1] = r;

	wire [2:0] ebl_mid, eid_mid;
	wire [41:0] ebl_leaf;
	wire [35:0] eid_leaf;
	assign ebl_mid[0] = en_s_blend_go;
	assign ebl_mid[1] = en_s_blend_go;
	assign ebl_mid[2] = en_s_blend_go;
	assign eid_mid[0] = en_s_idle_miss;
	assign eid_mid[1] = en_s_idle_miss;
	assign eid_mid[2] = en_s_idle_miss;

	wire is_w5_raw = (st == S_ROW_WAIT6);
	wire [8:0] str_ce_leaf;
	wire do_stream_drep, do_stream_bdst, do_stream_pv;
	assign do_stream_drep = do_stream;
	assign do_stream_bdst = do_stream;
	assign do_stream_pv = do_stream;
	for (gb = 0; gb < 9; gb = gb + 1) begin : g_str_ce
		assign str_ce_leaf[gb] = do_stream;
	end
	generate
	for (gb = 0; gb < 2; gb = gb + 1) begin : g_row_ce_buf
		assign en_s_row_buf[gb] = en_s_row_w;
		assign en_crb_buf[gb] = en_cur_rb_w;
		assign crb_w5_buf[gb] = is_w5_raw;
		assign crb_r_buf[gb] = r;
		assign etag0_we_buf[gb] = fill_we0_w;
		assign etag1_we_buf[gb] = fill_we1_w;
	end
	for (gb = 0; gb < 6; gb = gb + 1) begin : g_bp2_ce_buf
		assign en_bp2_leaf[gb] = en_bprep2_w;
	end
	for (gb = 0; gb < N_BE16; gb = gb + 1) begin : g_edclr_buf
		assign ed0_clr_buf[gb] = ed0_clr_w;
		assign ed1_clr_buf[gb] = ed1_clr_w;
	end
	for (gb = 0; gb < N_PRED; gb = gb + 1) begin : g_fill_we_leaf
		assign fill_we0_leaf[gb] = fill_we0_mid[gb / (N_PRED / 2)];
		assign fill_we1_leaf[gb] = fill_we1_mid[gb / (N_PRED / 2)];
		assign r_wdata_leaf[gb] = r_wdata_mid[gb / (N_PRED / 2)];
	end
	for (gb = 0; gb < N_LCE; gb = gb + 1) begin : g_lce_leaf
		assign lce_leaf[gb] = lce_mid[(gb * 3) / N_LCE];
	end
	for (gb = 0; gb < N_LPE; gb = gb + 1) begin : g_lpe_leaf
		assign lpe_leaf[gb] = lpe_mid[gb / (N_LPE / 2)];
	end
	for (gb = 0; gb < N_RGO; gb = gb + 1) begin : g_req_go_leaf
		assign req_go_leaf[gb] = req_go_mid[gb / (N_RGO / 2)];
	end
	for (gb = 0; gb < N_RWR; gb = gb + 1) begin : g_req_wr_leaf
		assign req_wr_leaf[gb] = req_wr_go_w;
	end
	for (gb = 0; gb < 42; gb = gb + 1) begin : g_ebl_leaf
		assign ebl_leaf[gb] = ebl_mid[gb / 14];
	end
	for (gb = 0; gb < 36; gb = gb + 1) begin : g_eid_leaf
		assign eid_leaf[gb] = eid_mid[gb / 12];
	end
	endgenerate
`else
	assign en_s_row_buf = {2{en_s_row_w}};
	assign en_crb_buf = {2{en_cur_rb_w}};
	assign crb_w5_buf = {2{st == S_ROW_WAIT6}};
	wire [8:0] str_ce_leaf = {9{do_stream}};
	wire do_stream_drep = do_stream;
	wire do_stream_bdst = do_stream;
	wire do_stream_pv   = do_stream;
	assign crb_r_buf  = {2{r}};
	assign etag0_we_buf = {2{fill_we0_w}};
	assign etag1_we_buf = {2{fill_we1_w}};
	assign en_bp2_leaf = {6{en_bprep2_w}};
	assign ed0_clr_buf = {N_BE16{ed0_clr_w}};
	assign ed1_clr_buf = {N_BE16{ed1_clr_w}};
	wire [N_PRED-1:0] fill_we0_leaf = {N_PRED{fill_we0_w}};
	wire [N_PRED-1:0] fill_we1_leaf = {N_PRED{fill_we1_w}};
	wire [N_PRED-1:0] r_wdata_leaf  = {N_PRED{r}};
	wire [2:0] lce_mid = {3{latch_commit_en}};
	wire [N_LCE-1:0] lce_leaf = {N_LCE{latch_commit_en}};
	wire [N_LPE-1:0] lpe_leaf = {N_LPE{latch_pre_en}};
	wire [N_RGO-1:0] req_go_leaf = {N_RGO{req_go_w}};
	wire [N_RWR-1:0] req_wr_leaf = {N_RWR{req_wr_go_w}};
	wire [41:0] ebl_leaf = {42{en_s_blend_go}};
	wire [35:0] eid_leaf = {36{en_s_idle_miss}};
`endif

	/* Pre-shifted tag_x0/x1 for S_BLEND_PREP2 latch (so compare time needs 0 xsh muxes on bits T_HI:2) */
	wire tag_hi_bad = (p_xsh_b[2] == 2'd0) ? (|tag_x0_r[T_HI:T_HI-1] | |tag_x1_r[T_HI:T_HI-1]) :
	                  (p_xsh_b[2] == 2'd1) ? (tag_x0_r[T_HI] | tag_x1_r[T_HI]) : 1'b0;
	wire [T_HI:0] tag_x0_sh =
		(p_xsh_b[2] == 2'd2) ? tag_x0_r[T_HI:0] :
		(p_xsh_b[2] == 2'd1) ? {tag_x0_r[T_HI-1:0], 1'b0} :
		                       {tag_x0_r[T_HI-2:0], 2'b00};
	wire [T_HI:0] tag_x1_sh =
		(p_xsh_b[3] == 2'd2) ? tag_x1_r[T_HI:0] :
		(p_xsh_b[3] == 2'd1) ? {tag_x1_r[T_HI-1:0], 1'b0} :
		                       {tag_x1_r[T_HI-2:0], 2'b00};
	wire [12:0] tag_x0_prep = (LINE_SHIFT == 5) ?
		{tag_hi_bad ? 1'b1 : tag_x0_sh[T_HI], tag_x0_sh[11:0]} :
		{tag_hi_bad ? 1'b1 : 1'b0,            tag_x0_sh[11:0]};
	wire [12:0] tag_x1_prep = (LINE_SHIFT == 5) ?
		{tag_hi_bad ? 1'b0 : tag_x1_sh[T_HI], tag_x1_sh[11:0]} :
		{1'b0,                                tag_x1_sh[11:0]};

	/* Free-running / unreset datapath registers (<= 16 FFs per CE leaf, 0 rst_sync fanout) */
	always @(posedge clk) begin
`ifdef SYNTH_KINTEX7
		wb_row_base0 <= p_row_base0;
		wb_row_base1 <= wb_row_base1_comb;
`elsif SYNTH_KINTEX_ULTRASCALE
		wb_row_base0 <= p_row_base0;
		wb_row_base1 <= wb_row_base1_comb;
`else
		if (!is_idle_reg) begin
			wb_row_base0 <= p_row_base0;
			wb_row_base1 <= wb_row_base1_comb;
		end
`endif

		if (en_crb_buf[0]) cur_row_base_r[15:0]  <= crb_w5_buf[0] ? p_row_base0[15:0]  : (crb_r_buf[0] ? wb_row_base1[15:0]  : wb_row_base0[15:0]);
		if (en_crb_buf[1]) cur_row_base_r[31:16] <= crb_w5_buf[1] ? p_row_base0[31:16] : (crb_r_buf[1] ? wb_row_base1[31:16] : wb_row_base0[31:16]);

		if (en_s_row_buf[0]) addr[15:0]  <= wb_addr_comb[15:0];
		if (en_s_row_buf[1]) addr[31:16] <= wb_addr_comb[31:16];

		if (etag0_we_buf[0]) etag[0][15:0]      <= addr_ext[LINE_SHIFT +: 16];
		if (etag0_we_buf[1]) etag[0][LA_W-1:16] <= addr_ext[ADDR_W-1 : LINE_SHIFT+16];
		if (etag1_we_buf[0]) etag[1][15:0]      <= addr_ext[LINE_SHIFT +: 16];
		if (etag1_we_buf[1]) etag[1][LA_W-1:16] <= addr_ext[ADDR_W-1 : LINE_SHIFT+16];

`ifdef SYNTH_KINTEX7
		for (l = 0; l < N_BE16; l = l + 1) begin
			edirty[0][16 * l +: 16] <= ed0_clr_buf[l] ? 16'd0 :
				(edirty[0][16 * l +: 16] | commit_wr0_en[16 * l +: 16]);
			edirty[1][16 * l +: 16] <= ed1_clr_buf[l] ? 16'd0 :
				(edirty[1][16 * l +: 16] | commit_wr1_en[16 * l +: 16]);
		end

		/* Buffered S_IDLE miss latches for color, xy, mask (Rule 1 fix: 3 LUTs to CE) */
		for (l = 0; l < 32; l = l + 1) begin
			if (eid_leaf[l])
				color[16 * l +: 16] <= s1_color[16 * l +: 16];
		end
		if (eid_leaf[32])
			xy[15:0] <= {s1_xy[15:LINE_SHIFT], d_xy_loff, s1_xy[0]};
		if (eid_leaf[33])
			xy[31:16] <= s1_xy[31:16];
		if (eid_leaf[34])
			mask <= s1_smask;
		if (eid_leaf[35]) begin
			rt <= s1_rt;
			rt_blend <= s1_rt_blend;
			rt_wpix_b <= s1_rt_blend;
			rt_wpix_s <= s1_rt_blend;
		end

		for (l = 0; l < 4; l = l + 1) begin
			if (str_ce_leaf[l])
				r_q64_0[16 * l +: 16] <= q64_0[16 * l +: 16];
			if (str_ce_leaf[4 + l])
				r_q64_1[16 * l +: 16] <= q64_1[16 * l +: 16];
		end
		if (str_ce_leaf[8])
			r_str_x21 <= s1_xy[2:1];

		/* Buffered S_BLEND latches for r_bsrc, r_bdst, r_bmask, r_bsoff, r_bent, r_bps */
		for (l = 0; l < 32; l = l + 1) begin
			if (ebl_leaf[l])
				r_bsrc[16 * l +: 16] <= p_bsrc[16 * l +: 16];
		end
		for (l = 0; l < 8; l = l + 1) begin
			if (ebl_leaf[32 + l])
				r_bdst[16 * l +: 16] <= p_bdst[16 * l +: 16];
		end
		if (ebl_leaf[40])
			r_bsoff <= p_soff;
		if (ebl_leaf[41]) begin
			r_bmask <= r_pmask;
			r_bent <= p_ent;
			r_bps <= p_ps[1:0];
			r_blm <= {p_ps[4], |p_ps[4:3], |p_ps[4:2], |p_ps[4:1]};
		end

		/* Stage A -> B: latch decoded byte write enables & slot selects */
		for (l = 0; l < N_BE16; l = l + 1) begin
			if (lpe_leaf[l])
				pre_wr0_en[16 * l +: 16] <= pre_wr0_en_w[16 * l +: 16];
			if (lpe_leaf[N_BE16 + l])
				pre_wr1_en[16 * l +: 16] <= pre_wr1_en_w[16 * l +: 16];
		end
		for (l = 0; l < N_SEL16; l = l + 1) begin
			if (lpe_leaf[2 * N_BE16 + l])
				pre_wr0_sel[16 * l +: 16] <= pre_wr0_sel_w[16 * l +: 16];
			if (lpe_leaf[2 * N_BE16 + N_SEL16 + l])
				pre_wr1_sel[16 * l +: 16] <= pre_wr1_sel_w[16 * l +: 16];
		end
		for (ps_i = 0; ps_i < 32; ps_i = ps_i + 1) begin
			if (lpe_leaf[2 * N_BE16 + 2 * N_SEL16 + (ps_i / 8)])
				pre_u8_sel_rep[ps_i] <= pre_u8_sel_in_b[ps_i / 8];
		end

		/* Stage B -> C: latch muxed new pixel bytes for commit */
		for (l = 0; l < N_PRED; l = l + 1) begin
			if (lce_leaf[l])
				commit_wdata0[16 * l +: 16] <= wdata0_pre_comb[16 * l +: 16];
			if (lce_leaf[N_PRED + l])
				commit_wdata1[16 * l +: 16] <= wdata1_pre_comb[16 * l +: 16];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (l = 0; l < N_BE16; l = l + 1) begin
			edirty[0][16 * l +: 16] <= ed0_clr_buf[l] ? 16'd0 :
				(edirty[0][16 * l +: 16] | commit_wr0_en[16 * l +: 16]);
			edirty[1][16 * l +: 16] <= ed1_clr_buf[l] ? 16'd0 :
				(edirty[1][16 * l +: 16] | commit_wr1_en[16 * l +: 16]);
		end

		/* Buffered S_IDLE miss latches for color, xy, mask (Rule 1 fix: 3 LUTs to CE) */
		for (l = 0; l < 32; l = l + 1) begin
			if (eid_leaf[l])
				color[16 * l +: 16] <= s1_color[16 * l +: 16];
		end
		if (eid_leaf[32])
			xy[15:0] <= {s1_xy[15:LINE_SHIFT], d_xy_loff, s1_xy[0]};
		if (eid_leaf[33])
			xy[31:16] <= s1_xy[31:16];
		if (eid_leaf[34])
			mask <= s1_smask;
		if (eid_leaf[35]) begin
			rt <= s1_rt;
			rt_blend <= s1_rt_blend;
			rt_wpix_b <= s1_rt_blend;
			rt_wpix_s <= s1_rt_blend;
		end

		for (l = 0; l < 4; l = l + 1) begin
			if (str_ce_leaf[l])
				r_q64_0[16 * l +: 16] <= q64_0[16 * l +: 16];
			if (str_ce_leaf[4 + l])
				r_q64_1[16 * l +: 16] <= q64_1[16 * l +: 16];
		end
		if (str_ce_leaf[8])
			r_str_x21 <= s1_xy[2:1];

		/* Buffered S_BLEND latches for r_bsrc, r_bdst, r_bmask, r_bsoff, r_bent, r_bps */
		for (l = 0; l < 32; l = l + 1) begin
			if (ebl_leaf[l])
				r_bsrc[16 * l +: 16] <= p_bsrc[16 * l +: 16];
		end
		for (l = 0; l < 8; l = l + 1) begin
			if (ebl_leaf[32 + l])
				r_bdst[16 * l +: 16] <= p_bdst[16 * l +: 16];
		end
		if (ebl_leaf[40])
			r_bsoff <= p_soff;
		if (ebl_leaf[41]) begin
			r_bmask <= r_pmask;
			r_bent <= p_ent;
			r_bps <= p_ps[1:0];
			r_blm <= {p_ps[4], |p_ps[4:3], |p_ps[4:2], |p_ps[4:1]};
		end

		/* Stage A -> B: latch decoded byte write enables & slot selects */
		for (l = 0; l < N_BE16; l = l + 1) begin
			if (lpe_leaf[l])
				pre_wr0_en[16 * l +: 16] <= pre_wr0_en_w[16 * l +: 16];
			if (lpe_leaf[N_BE16 + l])
				pre_wr1_en[16 * l +: 16] <= pre_wr1_en_w[16 * l +: 16];
		end
		for (l = 0; l < N_SEL16; l = l + 1) begin
			if (lpe_leaf[2 * N_BE16 + l])
				pre_wr0_sel[16 * l +: 16] <= pre_wr0_sel_w[16 * l +: 16];
			if (lpe_leaf[2 * N_BE16 + N_SEL16 + l])
				pre_wr1_sel[16 * l +: 16] <= pre_wr1_sel_w[16 * l +: 16];
		end
		for (ps_i = 0; ps_i < 32; ps_i = ps_i + 1) begin
			if (lpe_leaf[2 * N_BE16 + 2 * N_SEL16 + (ps_i / 8)])
				pre_u8_sel_rep[ps_i] <= pre_u8_sel_in_b[ps_i / 8];
		end

		/* Stage B -> C: latch muxed new pixel bytes for commit */
		for (l = 0; l < N_PRED; l = l + 1) begin
			if (lce_leaf[l])
				commit_wdata0[16 * l +: 16] <= wdata0_pre_comb[16 * l +: 16];
			if (lce_leaf[N_PRED + l])
				commit_wdata1[16 * l +: 16] <= wdata1_pre_comb[16 * l +: 16];
		end
`else
		if (en_s_idle_miss) begin
			color <= s1_color;
			xy <= {s1_xy[31:LINE_SHIFT], d_xy_loff, s1_xy[0]};
			mask <= s1_smask;
			rt <= s1_rt;
			rt_blend <= s1_rt_blend;
			rt_wpix_b <= s1_rt_blend;
			rt_wpix_s <= s1_rt_blend;
		end
		if (do_stream) begin
			r_q64_0 <= q64_0;
			r_q64_1 <= q64_1;
			r_str_x21 <= s1_xy[2:1];
		end

		if (en_s_blend_go) begin
			r_bsrc <= p_bsrc;
			r_bdst <= p_bdst;
			r_bsoff <= p_soff;
			r_bmask <= r_pmask;
			r_bent <= p_ent;
			r_bps <= p_ps[1:0];
			r_blm <= {p_ps[4], |p_ps[4:3], |p_ps[4:2], |p_ps[4:1]};
		end

		if (latch_pre_en) begin
			pre_wr0_en <= pipe_v[9] ? r_str_wr0_en : pre_wr0_en_w;
			pre_wr1_en <= pipe_v[9] ? r_str_wr1_en : pre_wr1_en_w;
			pre_wr0_sel <= pipe_v[9] ? r_str_wr0_sel : pre_wr0_sel_w;
			pre_wr1_sel <= pipe_v[9] ? r_str_wr1_sel : pre_wr1_sel_w;
			pre_u8_sel_rep[0] <= pipe_v[9] ? r_str_u8_sel : pre_u8_sel_in;
		end

		if (latch_commit_en) begin
			commit_wdata0 <= wdata0_pre_comb;
			commit_wdata1 <= wdata1_pre_comb;
		end

		if (pipe_v[8]) begin
			r_str_wr0_en  <= str_wr0_en_w;
			r_str_wr1_en  <= str_wr1_en_w;
			r_str_wr0_sel <= str_wr0_sel_w;
			r_str_wr1_sel <= str_wr1_sel_w;
			r_str_u8_sel  <= str_u8_sel_in;
		end
`endif

		/* Buffered S_BLEND_PREP2 line metadata latches (<= 16 FFs per leaf) */
		if (en_bp2_leaf[0]) begin
			line_rt <= rt;
			cached_drop <= p_drop;
			cached_ps <= p_ps[1:0];
			cached_lm <= {p_ps[4], |p_ps[4:3], |p_ps[4:2], |p_ps[4:1]};
			cached_xsh <= p_xsh;
		end
		if (en_bp2_leaf[1]) begin
			line_y <= xy[31:16];
		end
		if (en_bp2_leaf[2]) begin
			cached_tag_x0 <= {tag_x0_r[LA_W-1:13], tag_x0_prep};
		end
		if (en_bp2_leaf[3]) begin
			cached_tag_x1 <= {tag_x1_r[LA_W-1:13], tag_x1_prep};
		end
		if (en_bp2_leaf[4]) begin
			for (ps_i = 0; ps_i < 8; ps_i = ps_i + 1)
				cached_xsh_rep[ps_i] <= p_xsh_b[0];
		end
		if (en_bp2_leaf[5]) begin
			for (ps_i = 8; ps_i < 16; ps_i = ps_i + 1)
				cached_xsh_rep[ps_i] <= p_xsh_b[1];
		end
	end

`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			edirty[0] <= {LINE_BYTES{1'b0}};
			edirty[1] <= {LINE_BYTES{1'b0}};
		end else begin
			if (ed0_clr_w)
				edirty[0] <= {LINE_BYTES{1'b0}};
			else if (commit_v)
				edirty[0] <= edirty[0] | commit_wr0_en;

			if (ed1_clr_w)
				edirty[1] <= {LINE_BYTES{1'b0}};
			else if (commit_v)
				edirty[1] <= edirty[1] | commit_wr1_en;
		end
	end
`endif

	/* Pre-registered wpix control & FSM conditions + flattened 1-hot FSM next-state */
	reg r_st_row, r_st_blend, r_st_bcommit;
	reg r_st_rw1, r_st_rw2, r_st_rw3, r_st_rw4, r_st_rw5, r_st_rw6, r_st_rp, r_st_rp2;
	reg r_st_lp, r_st_look, r_st_evw, r_st_fill, r_st_bp1, r_st_bp2, r_st_bwait, r_st_flush_prep, r_st_flush, r_st_flw;
	reg r_scfg_tag_ok, r_scfg_ev_ok, r_bl_np_z;
	reg r_j_eq_pnl, r_g_eq_pnp, r_ev_any;
	reg r_row_drop, r_row_lw, r_row_multi, r_row_read;
	reg r_bl_multi;
	reg [1:0] r_bl_path;
`ifdef SYNTH_KINTEX7
	(* keep = "true" *) wire idle_flush_go = idle_grp[6] && !di_valid && !s1_v && !r_pipe_nz_0 && !pipe_nz_1 && r_idle_is16 && r_ev_any;
	(* keep = "true" *) wire st0_miss_cand = idle_grp[5] && s1_v && !s1_nocol && pipe_all_empty && !s1_sm_zero;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true" *) wire idle_flush_go = idle_grp[6] && !di_valid && !s1_v && !r_pipe_nz_0 && !pipe_nz_1 && r_idle_is16 && r_ev_any;
	(* keep = "true" *) wire st0_miss_cand = idle_grp[5] && s1_v && !s1_nocol && pipe_all_empty && !s1_sm_zero;
`else
	(* keep = "true" *) wire idle_flush_go = is_idle_reg && !di_valid && !s1_v && !r_pipe_nz_0 && !pipe_nz_1 && r_idle_is16 && r_ev_any;
	(* keep = "true" *) wire st0_miss_cand = is_idle_reg && s1_v && !s1_nocol && pipe_all_empty && !s1_sm_zero;
`endif

	wire row_go_prep = !r_row_drop && !r_row_lw && ((r_row_multi && !r_j_eq_pnl) || !r_fsm);
	wire row_go_idle = r_row_drop || (!r_row_lw && r_fsm && r_row_multi && r_j_eq_pnl);
	wire row_to_lp   = !r_row_drop && r_row_lw;
	wire row_to_bp1  = !r_row_drop && !r_row_lw && r_fsm && !r_row_multi;
	wire bl_next_pass = !r_bl_multi && !r_g_eq_pnp;
	wire bl_done_idle = (!r_bl_multi && r_g_eq_pnp) || (r_bl_multi && r_j_eq_pnl && r_fsm);
	wire bl_next_row  = r_bl_multi && (!r_j_eq_pnl || !r_fsm);
	wire bl_to_bp1    = bl_next_pass || (!bl_done_idle && !bl_next_row);

	wire evw_done   = !req_valid && !wait_rsp;
	wire fill_done  = !r_row_read || (!req_valid && wait_rsp && rsp_valid);
	wire lf_to_bp1  = r_row_multi || r_fsm;
	wire lf_st_wait = (r_st_look && r_look_hit) || (r_st_fill && fill_done);

	/* Direct 1-hot next-state signals (no 5-bit binary encode -> 4:1 mux -> 5-bit decode chain!) */
	(* keep = "true" *) wire st0_miss = st0_miss_cand && !stream_hit_free;
	wire blend_fin_go = (r_st_blend && r_pmask_z) || r_st_bcommit;
`ifdef SYNTH_KINTEX7
	wire nxt_idle = idle_grp[6] ? (!idle_flush_go && !st0_miss) : (
`elsif SYNTH_KINTEX_ULTRASCALE
	wire nxt_idle = idle_grp[6] ? (!idle_flush_go && !st0_miss) : (
`else
	wire nxt_idle = is_idle_reg ? (!idle_flush_go && !st0_miss) : (
`endif
		(r_st_row     && row_go_idle) ||
		(blend_fin_go && bl_done_idle) ||
		(r_st_flush   && !r_edirty_nz && r_fsm));
	wire nxt_rw1        = st0_miss;
	wire nxt_rw2        = r_st_rw1;
	wire nxt_rw3        = r_st_rw2;
	wire nxt_rw4        = r_st_rw3;
	wire nxt_rw5        = r_st_rw4;
	wire nxt_rw6        = r_st_rw5;
	wire nxt_row        = r_st_rw6 || r_st_rp2;
	wire nxt_rp         = (r_st_row && row_go_prep) || (blend_fin_go && !bl_to_bp1 && bl_next_row) || (lf_st_wait && !lf_to_bp1);
	wire nxt_rp2        = r_st_rp;
	wire nxt_lp         = r_st_row && row_to_lp;
	wire nxt_look       = r_st_lp;
	wire nxt_evw        = (r_st_look && !r_look_hit && r_edirty_nz) || (r_st_evw && !evw_done);
	wire nxt_fill       = (r_st_look && !r_look_hit && !r_edirty_nz) || (r_st_evw && evw_done) || (r_st_fill && !fill_done);
	wire nxt_bp1        = (r_st_row && row_to_bp1) || (blend_fin_go && bl_to_bp1) || (lf_st_wait && lf_to_bp1);
	wire nxt_bp2        = r_st_bp1;
	wire nxt_blend      = r_st_bp2;
	wire nxt_bwait      = (r_st_blend && !r_pmask_z) || (r_st_bwait && !p_ready);
	wire nxt_bcommit    = r_st_bwait && p_ready;
`ifdef SYNTH_KINTEX7
	wire nxt_flush_prep = (idle_grp[6] && idle_flush_go) || (r_st_flush && !r_edirty_nz && !r_fsm) || (r_st_flw && evw_done);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire nxt_flush_prep = (idle_grp[6] && idle_flush_go) || (r_st_flush && !r_edirty_nz && !r_fsm) || (r_st_flw && evw_done);
`else
	wire nxt_flush_prep = (is_idle_reg && idle_flush_go) || (r_st_flush && !r_edirty_nz && !r_fsm) || (r_st_flw && evw_done);
`endif
	wire nxt_flush      = r_st_flush_prep;
	wire nxt_flw        = (r_st_flush && r_edirty_nz) || (r_st_flw && !evw_done);

	wire [4:0] st_next_full = {
		(nxt_flush | nxt_flw) | (nxt_lp | nxt_flush_prep) | (nxt_rw6 | nxt_rp2),
		((nxt_look | nxt_evw) | (nxt_fill | nxt_bp1)) | ((nxt_bp2 | nxt_blend) | (nxt_bwait | nxt_bcommit)),
		((nxt_rw4 | nxt_rw5) | (nxt_rp | nxt_row)) | ((nxt_bp2 | nxt_blend) | (nxt_bwait | nxt_bcommit)) | (nxt_rw6 | nxt_rp2),
		((nxt_rw2 | nxt_rw3) | (nxt_rp | nxt_row)) | ((nxt_fill | nxt_bp1) | (nxt_bwait | nxt_bcommit)) | (nxt_lp | nxt_flush_prep),
		((nxt_rw1 | nxt_rw3) | (nxt_rw5 | nxt_row)) | ((nxt_evw | nxt_bp1) | (nxt_blend | nxt_bcommit)) | (nxt_flw | nxt_flush_prep | nxt_rp2)
	};
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			st <= 5'd0;
			is_idle_reg <= 1'b1;
			is_idle_drep <= {N_DREP{1'b1}};
			is_idle_rt   <= 1'b1;
			r_st_row     <= 1'b0;
			r_st_blend   <= 1'b0;
			r_st_bcommit <= 1'b0;
			r_st_rw1 <= 1'b0; r_st_rw2 <= 1'b0; r_st_rw3 <= 1'b0; r_st_rw4 <= 1'b0;
			r_st_rw5 <= 1'b0; r_st_rw6 <= 1'b0; r_st_rp  <= 1'b0; r_st_rp2 <= 1'b0; r_st_lp  <= 1'b0;
			r_st_look <= 1'b0; r_st_evw <= 1'b0; r_st_fill <= 1'b0; r_st_bp1 <= 1'b0;
			r_st_bp2 <= 1'b0; r_st_bwait <= 1'b0; r_st_flush_prep <= 1'b0; r_st_flush <= 1'b0; r_st_flw <= 1'b0;
			r_j_eq_pnl <= 1'b0;
			r_g_eq_pnp <= 1'b0;
			r_ev_any <= 1'b0;
			r_row_drop <= 1'b0;
			r_row_lw <= 1'b0;
			r_row_multi <= 1'b0;
			r_row_read <= 1'b0;
			r_bl_multi <= 1'b0;
			r_bl_path <= 2'd0;
			r_bl_np_z <= 1'b0;
		end else begin
			st <= st_next_full;
			is_idle_reg <= nxt_idle;
			is_idle_drep <= {N_DREP{nxt_idle}};
			is_idle_rt   <= nxt_idle;
			r_st_row     <= nxt_row;
			r_st_blend   <= nxt_blend;
			r_st_bcommit <= nxt_bcommit;
			r_st_rw1     <= nxt_rw1;
			r_st_rw2     <= nxt_rw2;
			r_st_rw3     <= nxt_rw3;
			r_st_rw4     <= nxt_rw4;
			r_st_rw5     <= nxt_rw5;
			r_st_rw6     <= nxt_rw6;
			r_st_rp      <= nxt_rp;
			r_st_rp2     <= nxt_rp2;
			r_st_lp      <= nxt_lp;
			r_st_look    <= nxt_look;
			r_st_evw     <= nxt_evw;
			r_st_fill    <= nxt_fill;
			r_st_bp1     <= nxt_bp1;
			r_st_bp2     <= nxt_bp2;
			r_st_bwait   <= nxt_bwait;
			r_st_flush_prep <= nxt_flush_prep;
			r_st_flush   <= nxt_flush;
			r_st_flw     <= nxt_flw;
			r_ev_any     <= ev[0] || ev[1];
			if (r_st_rw6 || r_st_rp2) begin
				r_j_eq_pnl  <= (j == p_nl);
				r_row_drop  <= p_drop;
				r_row_lw    <= p_lw;
				r_row_multi <= p_multi;
				r_row_read  <= p_read;
			end
			if (r_st_bp1 || r_st_bp2 || r_st_bwait) begin
				r_j_eq_pnl <= (j == p_nl);
				r_g_eq_pnp <= (g_fsm == p_np);
				r_bl_multi <= p_multi;
				r_bl_path  <= p_path;
				r_bl_np_z  <= (p_np == 2'd0);
			end
		end
	end

	/* Flat parallel next-value logic for r, j, g, ev, erd, idle, r_line_stream_ok (<= 7 cmos3, FO <= 16) */
`ifdef SYNTH_KINTEX7
	wire r_clr0 =
		(idle_grp[10] && !di_valid && !s1_v && !pipe_busy && (idle == 5'd16) && r_ev_any) ||
`elsif SYNTH_KINTEX_ULTRASCALE
	wire r_clr0 =
		(idle_grp[10] && !di_valid && !s1_v && !pipe_busy && (idle == 5'd16) && r_ev_any) ||
`else
	wire r_clr0 =
		(is_idle_reg && !di_valid && !s1_v && !pipe_busy && (idle == 5'd16) && r_ev_any) ||
`endif
		r_st_rw1 ||
		(r_st_row && (r_row_drop || (!r_row_lw && r_fsm && r_row_multi && r_j_eq_pnl))) ||
		(blend_fin_go && ((!r_bl_multi && r_g_eq_pnp) || (r_bl_multi && r_j_eq_pnl && r_fsm))) ||
		(r_st_flush && !r_edirty_nz && r_fsm);
	wire r_set1 =
		(r_st_row && !r_row_drop && !r_row_lw && !(r_row_multi && !r_j_eq_pnl) && !r_fsm) ||
		(r_st_look && r_look_hit && !r_row_multi) ||
		(r_st_fill && fill_done && !r_row_multi) ||
		(blend_fin_go && r_bl_multi && r_j_eq_pnl && !r_fsm) ||
		(r_st_flush && !r_edirty_nz && !r_fsm);
	wire r_nxt = r_set1 || (r_fsm && !r_clr0);

	wire j_clr0 =
		r_st_rw1 ||
		(r_st_row && (r_row_drop || (!r_row_lw && !(r_row_multi && !r_j_eq_pnl)))) ||
		(blend_fin_go && (!r_bl_multi || r_j_eq_pnl)) ||
		(r_st_flush && !r_edirty_nz && r_fsm);
	wire j_inc =
		(r_st_row && !r_row_drop && !r_row_lw && r_row_multi && !r_j_eq_pnl) ||
		(blend_fin_go && r_bl_multi && !r_j_eq_pnl);
	wire [1:0] j_nxt = j_inc ? j1 : (j_clr0 ? 2'd0 : j);

	wire g_clr0 =
		r_st_rw1 ||
		(r_st_row && (r_row_drop || (!r_row_lw && !(r_row_multi && !r_j_eq_pnl) && (r_fsm || !r_row_multi) && r_row_multi))) ||
		(r_st_look && r_look_hit) ||
		(r_st_fill && fill_done) ||
		(blend_fin_go && (!r_bl_multi ? r_g_eq_pnp : (!r_j_eq_pnl ? 1'b0 : r_fsm))) ||
		(r_st_flush && !r_edirty_nz && r_fsm);
	wire g_inc = blend_fin_go && !r_bl_multi && !r_g_eq_pnp;
	wire [1:0] g_nxt = g_inc ? g1 : (g_clr0 ? 2'd0 : g);

	wire tag_x0_hi_z = (tag_x0_r[LA_W-1 : TAG_CMP_BITS] == {(LA_W-TAG_CMP_BITS){1'b0}});
	wire tag_x1_hi_z = (tag_x1_r[LA_W-1 : TAG_CMP_BITS] == {(LA_W-TAG_CMP_BITS){1'b0}});
	wire bp2_stream_ok = r_scfg_tag_ok && r_scfg_ev_ok && !r_bl_multi && r_bl_np_z && (r_bl_path == 2'd0) && tag_x0_hi_z && tag_x1_hi_z;
	reg r_pipe_v_04_nz;

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			fsm_b_valid <= 1'b0;
			f_go <= 1'b0;
			r <= 1'b0; r_fsm <= 1'b0; r_fill <= 1'b0; r_tag <= 1'b0;
			r_req_ln <= 1'b0; r_dty_lo <= 1'b0; r_dty_hi <= 1'b0;
			j <= 2'd0;
			j_wpix <= 2'd0;
			g <= 2'd0;
			g_fsm <= 2'd0;
			req_valid <= 1'b0;
			wait_rsp <= 1'b0;
			ev[0] <= 1'b0;
			ev[1] <= 1'b0;
			erd[0] <= 1'b0;
			erd[1] <= 1'b0;
			line_match_valid <= 1'b0;
			stream_cfg_ok <= 1'b0;
			r_line_stream_ok <= 1'b0;
			line_stream_ok_drep <= {N_DREP{1'b0}};
			off[0] <= {LINE_SHIFT{1'b0}};
			off[1] <= {LINE_SHIFT{1'b0}};
			off_p1[0] <= {LINE_SHIFT{1'b0}};
			off_p1[1] <= {LINE_SHIFT{1'b0}};
			cached_row_base0 <= 32'd0;
			cached_row_base1 <= 32'd0;
			cached_multi <= 1'b0;
			cached_np <= 2'd0;
			cached_path <= 2'd0;
			cached_read <= 1'b0;
			idle <= 5'd0;
			pipe_v <= 11'd0;
			pipe_v0_drep <= {N_DREP{1'b0}};
			r_pipe_nz_0 <= 1'b0;
			r_pipe_v_04_nz <= 1'b0;
			r_pipe_hi_empty <= 1'b1;
			pipe_hi_empty_drep <= {N_DREP{1'b1}};
			r_idle_is16 <= 1'b0;
			pipe_v0_bdst <= 8'd0;
			pipe_nz_1_r <= 1'b0;
			commit_v <= 1'b0;
			commit_v_rep <= {N_CREP{1'b0}};
			commit_is_stream <= 1'b0;
			commit_id <= 4'd0;
			commit_wr0_en <= {LINE_BYTES{1'b0}};
			commit_wr1_en <= {LINE_BYTES{1'b0}};
			bwait_d1 <= 1'b0;
			r_look_hit <= 1'b0;
			r_edirty_nz <= 1'b0;
			r_wdata_ce_r <= {N_PRED{1'b0}};
			r_pmask <= 4'd0;
			r_pmask_z <= 1'b0;
			s1_sh0 <= 1'b0;
			s1_sh1 <= 1'b0;
			s1_sh0_drep <= {N_DREP{1'b0}};
			s1_sh1_drep <= {N_DREP{1'b0}};
			release_valid <= 1'b0;
			release_id <= 4'd0;
			release_oh <= 16'd0;
		end else begin
`ifdef SYNTH_KINTEX7
			if (!idle_grp[7] || di_ready) begin
				s1_sh0 <= s1_sh0_nxt;
				s1_sh1 <= s1_sh1_nxt;
			end
			for (i_dr = 0; i_dr < N_DREP; i_dr = i_dr + 1) begin
				if (!is_idle_drep[i_dr] || ((i_dr < 5) ? di_ready_rep[47] : di_ready_rep[129])) begin
					s1_sh0_drep[i_dr] <= s1_sh0_nxt;
					s1_sh1_drep[i_dr] <= s1_sh1_nxt;
				end
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (!idle_grp[7] || di_ready) begin
				s1_sh0 <= s1_sh0_nxt;
				s1_sh1 <= s1_sh1_nxt;
			end
			for (i_dr = 0; i_dr < N_DREP; i_dr = i_dr + 1) begin
				if (!is_idle_drep[i_dr] || ((i_dr < 5) ? di_ready_rep[47] : di_ready_rep[129])) begin
					s1_sh0_drep[i_dr] <= s1_sh0_nxt;
					s1_sh1_drep[i_dr] <= s1_sh1_nxt;
				end
			end
`else
			if (!is_idle_reg || di_ready) begin
				s1_sh0 <= s1_sh0_nxt;
				s1_sh1 <= s1_sh1_nxt;
			end
			for (i_dr = 0; i_dr < N_DREP; i_dr = i_dr + 1) begin
				if (!is_idle_drep[i_dr] || di_ready_rep[i_dr]) begin
					s1_sh0_drep[i_dr] <= s1_sh0_nxt;
					s1_sh1_drep[i_dr] <= s1_sh1_nxt;
				end
			end
`endif
			fsm_b_valid <= r_st_blend && !r_pmask_z && (r_bl_path == 2'd0);
			f_go        <= r_st_blend && !r_pmask_z && (r_bl_path == 2'd1);
			r_wdata_ce_r <= {N_PRED{r_st_lp || r_st_flush_prep}};
			if (req_valid && req_ready) begin
				req_valid <= 1'b0;
				wait_rsp <= 1'b1;
			end else if (req_go_w) begin
				req_valid <= 1'b1;
			end
			if (req_go_w)
				req_write <= req_wr_go_w;
			if (req_go_leaf[N_BE16 + 0])
				req_line[15:0] <= req_wr_leaf[N_BE16 + 0] ? etag[r_req_ln][15:0] : addr_ext[LINE_SHIFT +: 16];
			if (req_go_leaf[N_BE16 + 1])
				req_line[LA_W-1:16] <= req_wr_leaf[N_BE16 + 1] ? etag[r_req_ln][LA_W-1:16] : addr_ext[ADDR_W-1 : LINE_SHIFT+16];
			for (l = 0; l < N_BE16; l = l + 1) begin
				if (req_go_leaf[l])
					req_be[16 * l +: 16] <= req_wr_leaf[l] ? ((l < (N_BE16/2) ? r_dty_lo : r_dty_hi) ? edirty[1][16 * l +: 16] : edirty[0][16 * l +: 16]) : 16'd0;
			end
			for (l = 0; l < N_PRED; l = l + 1) begin
				if (r_wdata_ce_r[l])
					req_wdata[16 * l +: 16] <= r_wdata_leaf[l] ? edat[1][16 * l +: 16] : edat[0][16 * l +: 16];
			end
			if (rsp_valid)
				wait_rsp <= 1'b0;
			release_valid <= 1'b0;
			release_oh <= 16'd0;

			/* Advance 11-stage streaming shift register */
			pipe_nz_1_r <= (pipe_v[9:5] != 5'd0) || lce_leaf[N_LCE - 1];
			r_pipe_v_04_nz <= do_stream_pv || (pipe_v[3:0] != 4'd0);
			r_pipe_nz_0 <= do_stream_pv || r_pipe_v_04_nz;
			r_pipe_hi_empty <= (pipe_v[9:0] == 10'd0) && !lce_leaf[N_LCE - 1];
			pipe_hi_empty_drep <= {N_DREP{(pipe_v[9:0] == 10'd0) && !lce_leaf[N_LCE - 1]}};
			pipe_v0_drep <= {N_DREP{do_stream_drep}};
			pipe_v0_bdst <= {8{do_stream_bdst}};
`ifdef SYNTH_KINTEX7
			pipe_v[0] <= do_stream_pv;
			for (ps_i = 1; ps_i <= 10; ps_i = ps_i + 1)
				pipe_v[ps_i] <= pipe_v[ps_i - 1];
`elsif SYNTH_KINTEX_ULTRASCALE
			pipe_v[0] <= do_stream_pv;
			for (ps_i = 1; ps_i <= 10; ps_i = ps_i + 1)
				pipe_v[ps_i] <= pipe_v[ps_i - 1];
`else
			if (do_stream || pipe_v != 11'd0)
				pipe_v <= {pipe_v[9:0], do_stream};
`endif

			if (do_stream) begin
				pipe_id[0] <= alloc_id;
				pipe_soff[0] <= {str_line_off_p1, str_line_off, str_line_off_p1, str_line_off};
				pipe_mask[0] <= s0_mask;
				pipe_ent[0] <= 4'b1100;
				pipe_ps[0] <= cached_ps;
				pipe_lm[0] <= cached_lm;
			end
			for (ps_i = 1; ps_i <= 9; ps_i = ps_i + 1) begin
				if (pipe_v[ps_i - 1]) begin
					pipe_id[ps_i] <= pipe_id[ps_i - 1];
					pipe_soff[ps_i] <= pipe_soff[ps_i - 1];
					pipe_mask[ps_i] <= pipe_mask[ps_i - 1];
					pipe_ent[ps_i] <= pipe_ent[ps_i - 1];
					pipe_ps[ps_i] <= pipe_ps[ps_i - 1];
					pipe_lm[ps_i] <= pipe_lm[ps_i - 1];
				end
			end
			if (pipe_v[9])
				pipe_id[10] <= pipe_id[9];

			/* Stage B -> C: pre-gate commit_wr0_en and commit_wr1_en with lce_leaf every cycle */
			commit_v <= lce_leaf[N_LCE - 1];
			if (lce_leaf[N_LCE - 1]) begin
				commit_is_stream <= pipe_v[10] && b_rvalid;
				commit_id <= pipe_id[10];
			end
`ifdef SYNTH_KINTEX7
			for (l = 0; l < N_BE16; l = l + 1) begin
				commit_wr0_en[16 * l +: 16] <= lce_leaf[2 * N_PRED + l] ? pre_wr0_en[16 * l +: 16] : 16'd0;
				commit_wr1_en[16 * l +: 16] <= lce_leaf[2 * N_PRED + N_BE16 + l] ? pre_wr1_en[16 * l +: 16] : 16'd0;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			for (l = 0; l < N_BE16; l = l + 1) begin
				commit_wr0_en[16 * l +: 16] <= lce_leaf[2 * N_PRED + l] ? pre_wr0_en[16 * l +: 16] : 16'd0;
				commit_wr1_en[16 * l +: 16] <= lce_leaf[2 * N_PRED + N_BE16 + l] ? pre_wr1_en[16 * l +: 16] : 16'd0;
			end
`else
			commit_wr0_en <= latch_commit_en ? pre_wr0_en : {LINE_BYTES{1'b0}};
			commit_wr1_en <= latch_commit_en ? pre_wr1_en : {LINE_BYTES{1'b0}};
`endif
			if (commit_v && commit_is_stream) begin
				release_valid <= 1'b1;
				release_id <= commit_id;
				release_oh <= {
					(commit_id == 4'd15), (commit_id == 4'd14), (commit_id == 4'd13), (commit_id == 4'd12),
					(commit_id == 4'd11), (commit_id == 4'd10), (commit_id == 4'd9),  (commit_id == 4'd8),
					(commit_id == 4'd7),  (commit_id == 4'd6),  (commit_id == 4'd5),  (commit_id == 4'd4),
					(commit_id == 4'd3),  (commit_id == 4'd2),  (commit_id == 4'd1),  (commit_id == 4'd0)
				};
			end

			r <= r_nxt; r_fsm <= r_nxt; r_fill <= r_nxt; r_tag <= r_nxt;
			r_req_ln <= r_nxt; r_dty_lo <= r_nxt; r_dty_hi <= r_nxt;
			j <= j_nxt;
			j_wpix <= j_nxt;
			g <= g_nxt;
			g_fsm <= g_nxt;

			if (st_we || r_st_rw1 || r_st_evw || r_st_flush_prep) begin
				line_match_valid <= 1'b0;
				stream_cfg_ok <= 1'b0;
				r_line_stream_ok <= 1'b0;
			line_stream_ok_drep <= {N_DREP{1'b0}};
			end else if (r_st_bp2 && !r_bl_multi && g_fsm == 2'd0) begin
				line_match_valid <= 1'b1;
				stream_cfg_ok <= bp2_stream_ok;
				r_line_stream_ok <= bp2_stream_ok;
				line_stream_ok_drep <= {N_DREP{bp2_stream_ok}};
			end

`ifdef SYNTH_KINTEX7
			if ((idle_grp[7] && (di_valid || s1_v)) || (r_st_flush && !r_edirty_nz && r_fsm)) begin
				idle <= 5'd0;
				r_idle_is16 <= 1'b0;
			end else if (idle_grp[7] && !pipe_busy && (idle != 5'd16)) begin
				idle <= idle1;
				r_idle_is16 <= (idle == 5'd15);
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if ((idle_grp[7] && (di_valid || s1_v)) || (r_st_flush && !r_edirty_nz && r_fsm)) begin
				idle <= 5'd0;
				r_idle_is16 <= 1'b0;
			end else if (idle_grp[7] && !pipe_busy && (idle != 5'd16)) begin
				idle <= idle1;
				r_idle_is16 <= (idle == 5'd15);
			end
`else
			if ((is_idle_reg && (di_valid || s1_v)) || (r_st_flush && !r_edirty_nz && r_fsm)) begin
				idle <= 5'd0;
				r_idle_is16 <= 1'b0;
			end else if (is_idle_reg && !pipe_busy && (idle != 5'd16)) begin
				idle <= idle1;
				r_idle_is16 <= (idle == 5'd15);
			end
`endif

			if (r_st_fill && fill_done) begin
				ev[r_fill] <= 1'b1;
				erd[r_fill] <= r_row_read;
			end else if (r_st_flush && !r_edirty_nz) begin
				ev[r_fill] <= 1'b0;
			end

			if (r_st_lp) begin
				off[r] <= addr[LINE_SHIFT-1:0];
				off_p1[r] <= off_p1_comb;
				r_look_hit <= hit;
				r_edirty_nz <= ev[r_tag] && edirty_nz;
			end
			if (r_st_flush_prep)
				r_edirty_nz <= ev[r_tag] && edirty_nz;

			if (r_st_bp1) begin
				r_scfg_tag_ok <= rb0_aligned_r && rb1_aligned_r;
				r_scfg_ev_ok <= ev[0] && ev[1] &&
					(mask[7:0] != 8'd0) && (mask[15:8] != 8'd0) &&
					(erd[0] || !r_row_read) && (erd[1] || !r_row_read);
			end
			if (r_st_bp2) begin
				r_pmask <= p_mask;
				r_pmask_z <= (p_mask == 4'd0);
			end
			if (r_st_blend && !r_pmask_z)
				bwait_d1 <= 1'b0;
			else if (r_st_bwait)
				bwait_d1 <= 1'b1;
		end
	end

	/* Datapath entry data: free-running without reset */
	integer b_idx_edat;
	always @(posedge clk) begin
`ifdef SYNTH_KINTEX7
		/* Stage C & S_FILL: update edat[0..1] with buffered slice enables */
		for (b_idx_edat = 0; b_idx_edat < LINE_BYTES; b_idx_edat = b_idx_edat + 1) begin
			if (fill_we0_leaf[b_idx_edat / 2])
				edat[0][8 * b_idx_edat +: 8] <= p_read_edat[b_idx_edat / 2] ? rsp_rdata[8 * b_idx_edat +: 8] : 8'd0;
			else if (commit_wr0_en[b_idx_edat])
				edat[0][8 * b_idx_edat +: 8] <= commit_wdata0[8 * b_idx_edat +: 8];

			if (fill_we1_leaf[b_idx_edat / 2])
				edat[1][8 * b_idx_edat +: 8] <= p_read_edat[N_PRED + (b_idx_edat / 2)] ? rsp_rdata[8 * b_idx_edat +: 8] : 8'd0;
			else if (commit_wr1_en[b_idx_edat])
				edat[1][8 * b_idx_edat +: 8] <= commit_wdata1[8 * b_idx_edat +: 8];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		/* Stage C & S_FILL: update edat[0..1] with buffered slice enables */
		for (b_idx_edat = 0; b_idx_edat < LINE_BYTES; b_idx_edat = b_idx_edat + 1) begin
			if (fill_we0_leaf[b_idx_edat / 2])
				edat[0][8 * b_idx_edat +: 8] <= p_read_edat[b_idx_edat / 2] ? rsp_rdata[8 * b_idx_edat +: 8] : 8'd0;
			else if (commit_wr0_en[b_idx_edat])
				edat[0][8 * b_idx_edat +: 8] <= commit_wdata0[8 * b_idx_edat +: 8];

			if (fill_we1_leaf[b_idx_edat / 2])
				edat[1][8 * b_idx_edat +: 8] <= p_read_edat[N_PRED + (b_idx_edat / 2)] ? rsp_rdata[8 * b_idx_edat +: 8] : 8'd0;
			else if (commit_wr1_en[b_idx_edat])
				edat[1][8 * b_idx_edat +: 8] <= commit_wdata1[8 * b_idx_edat +: 8];
		end
`else
		if (fill_we0_w) begin
			edat[0] <= r_row_read ? rsp_rdata : {LINE_BYTES*8{1'b0}};
		end else if (commit_v) begin
			for (b_idx_edat = 0; b_idx_edat < LINE_BYTES; b_idx_edat = b_idx_edat + 1)
				if (commit_wr0_en[b_idx_edat])
					edat[0][8 * b_idx_edat +: 8] <= commit_wdata0[8 * b_idx_edat +: 8];
		end

		if (fill_we1_w) begin
			edat[1] <= r_row_read ? rsp_rdata : {LINE_BYTES*8{1'b0}};
		end else if (commit_v) begin
			for (b_idx_edat = 0; b_idx_edat < LINE_BYTES; b_idx_edat = b_idx_edat + 1)
				if (commit_wr1_en[b_idx_edat])
					edat[1][8 * b_idx_edat +: 8] <= commit_wdata1[8 * b_idx_edat +: 8];
		end
`endif
	end

`ifndef SYNTHESIS
	/* A quad's row on one line stays in it; several lines start a line. */
	always @(posedge clk or negedge rst_n)
		if (!rst_n) begin
		end else if (st == S_ROW && !p_drop && p_lw &&
		    (p_multi ? wb_addr_comb[LINE_SHIFT-1:0] != {LINE_SHIFT{1'b0}} :
		    {{(32-LINE_SHIFT){1'b0}}, wb_addr_comb[LINE_SHIFT-1:0]} + 32'd2 * ({30'd0, p_np} + 32'd1) * {27'd0, p_ps} >
		    LINE_BYTES))
			$display("FAIL atg_rop_wb: attachment %0d row at %h crosses a line", rt, wb_addr_comb);
`endif

	wire unused_ok = &{1'b0, rsp_write, rsp_tag, rsp_error, (MRT_COUNT != 0),
		wb_rb1_co, wb_addr_co, tag_x0_co, tag_x1_co, off_p1_co_unused, j1_co_unused, g1_co_unused, idle1_co_unused, p_addr, p_addr1, p_row_base1, p_xoff,
		alloc_ready, release_id, use_stream_res, commit_v_rep, stream_ok, stream_hit_w, lce_mid, xsh_t0[0],
		p_multi_b, p_read, p_path, p_drop, p_lw,
		cached_drop, cached_row_base0, cached_row_base1, cached_xsh, cached_multi, cached_np, cached_path, cached_read,
		cached_tag_x0[LA_W-1:13], cached_tag_x1[LA_W-1:13]};
endmodule

`default_nettype wire
