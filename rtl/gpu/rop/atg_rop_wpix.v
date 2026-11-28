/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rop_wpix: the pixels of atg_rop_wb: the attachment's state (CAPS[30]
 * RT_FORMATS, RTk_BASE/STRIDE, WRITE_MASK, BLEND, BLEND1-3, the blend
 * constants), the place of each pixel in the lines, the choice of the
 * blend path and the new pixel bytes.  Holds atg_rop_fblend.
 *
 * High-frequency 5nm ASIC implementation:
 * - Zero CARRY4s in state decodes and slot offset calculations.
 * - Registered DSP inputs (yr_r, sel_stride) and output (yrs_r) for u_yrs.
 * - Registered row_base0_r and xoff_j_r (1 Pre-CARRY4 LUT, 0 Post-CARRY4 LUTs).
 * - Pipelined slot gather (slot_o_r -> slot_dst128_r) and Path-2 format packer
 *   so all outputs to atg_rop_wb and atg_rop_fblend are registered (0 feedthrough).
 */

`timescale 1ns / 1ps
`default_nettype none

`include "atg_config.vh"

`ifndef CFG_ROP_MRT_COUNT
`define CFG_ROP_MRT_COUNT 8
`endif

`ifdef CFG_ENABLE_MSAA
`define ATG_DEFAULT_ENABLE_MSAA (`CFG_ENABLE_MSAA != 0)
`else
`define ATG_DEFAULT_ENABLE_MSAA 1
`endif

(* keep_hierarchy = "yes" *)
module atg_rop_wpix #(
	parameter LINE_BYTES = 32,	/* AMB line size: 32 or 64 (KC705) */
	parameter LINE_SHIFT = 5,	/* log2(LINE_BYTES) */
	parameter MRT_COUNT = 8,
	parameter ENABLE_MSAA = `ATG_DEFAULT_ENABLE_MSAA,
	parameter ENABLE_FBLEND = 1,
	parameter ENABLE_PACK = 1
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	/* The beat (atg_rop_wb's registered FSM copy) and the place in it */
	input wire [31:0] i_xy,
	input wire [2:0] i_rt,
	input wire [2:0] i_rt_b,
	input wire [2:0] i_rt_s,
	input wire [15:0] i_smask,	/* bit 4 l + s: sample s of lane l */
	input wire [511:0] i_color,
	input wire i_r,	/* row */
	input wire [1:0] i_j,	/* line of the row (several lines) */
	input wire [1:0] i_g,	/* pass: the sample (one line) */
	/* The two line entries and the quad's first byte in each */
	input wire [LINE_BYTES*8-1:0] i_line0,
	input wire [LINE_BYTES*8-1:0] i_line1,
	input wire [LINE_SHIFT-1:0] i_off0,
	input wire [LINE_SHIFT-1:0] i_off1,
	input wire [LINE_SHIFT-1:0] i_off0_p1,
	input wire [LINE_SHIFT-1:0] i_off1_p1,
	input wire [LINE_SHIFT-1:0] i_reg_off0,
	input wire [LINE_SHIFT-1:0] i_reg_off1,
	/* Registered inputs for fblend */
	input wire [2:0] i_reg_rt,
	input wire [511:0] i_reg_color,
	input wire [15:0] i_reg_mask,
	/* The paths' results */
	input wire [127:0] b_pix,
	input wire f_go,
	output wire f_done,
	input wire f_take,
	input wire i_idle,
	/* To atg_rop_wb */
	output wire [31:0] o_addr,
	output wire [31:0] o_addr1,
	output wire [1:0] o_nl,
	output wire [1:0] o_np,
	output wire o_multi,
	output wire o_drop,
	output wire o_read,
	output wire [1:0] o_path,
	output wire o_lw,
	output wire [3:0] o_mask,
	output wire [511:0] o_bsrc,
	output wire [127:0] o_bdst,
	output wire [511:0] o_res,
	output wire [3:0] o_ent,
	output wire [4*LINE_SHIFT-1:0] o_soff,
	output wire [4:0] o_ps,
	output wire [31:0] o_row_base0,
	output wire [31:0] o_row_base1,
	output wire [31:0] o_xoff,
	output wire [2:0] o_xsh,
	output wire [31:0] o_stride,
	output wire [31:0] o_xoff_j
);
	`include "ghci_regs.vh"
	`include "atg_rop_unorm8.vh"
	`include "atg_fmt_info.vh"

	(* keep = "true", dont_touch = "true" *) wire rst_sync = !rst_n;

	/* Distributed ROM for unorm8 conversion to avoid BRAM inference */
	(* rom_style = "distributed" *) reg [31:0] rom_unorm8 [0:255];
	integer r_u8_i;
	initial begin
		for (r_u8_i = 0; r_u8_i < 256; r_u8_i = r_u8_i + 1)
			rom_unorm8[r_u8_i] = atg_rop_unorm8(r_u8_i[7:0]);
	end

	/* STATE registers (names preserved for testbench inspection) */
	reg [31:0] base [0:7];
	reg [31:0] stride [0:7];
	reg [22:0] bls [0:7];
	reg [31:0] kcf [0:3];
	reg [1:0] fmt0;
	reg [63:0] fmts;
	reg [31:0] kc8;
	reg [31:0] wms;
	reg kc_sel;
	(* max_fanout = 8 *) reg ms4;

	/* 2-stage registered state write broadcast inside wpix so fanouts <= 12 and LUTs <= 2 */
	(* keep = "true", dont_touch = "true" *) reg [31:0] st_wd_b0, st_wd_b1, st_wd_c0, st_wd_c1;
	reg [7:0] we_base_r, we_stride_r, we_bls_r;
	reg [3:0] we_kcf_r;
	reg we_fmt0_r, we_fmts0_r, we_fmts1_r, we_kc8_r, we_rop_ctrl_r, we_wm0_r, we_wm0_hi_r, we_wm1_r, we_ms_r;

`ifdef SYNTH_KINTEX7
	wire wpix_st_clk = clk;
	wire wpix_act_clk = clk;
	wire wpix_p2_clk = clk;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire wpix_st_clk = clk;
	wire wpix_act_clk = clk;
	wire wpix_p2_clk = clk;
`else
	reg [5:0] wpix_st_sr;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync)
			wpix_st_sr <= 6'h3F;
		else if (st_we)
			wpix_st_sr <= 6'h3F;
		else if (wpix_st_sr != 6'd0)
			wpix_st_sr <= {1'b0, wpix_st_sr[5:1]};
	end
	reg wpix_st_en, wpix_act_en, wpix_p2_en;
	always @(negedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			wpix_st_en <= 1'b1;
			wpix_act_en <= 1'b1;
			wpix_p2_en <= 1'b1;
		end else begin
			wpix_st_en <= st_we || (wpix_st_sr != 6'd0);
			wpix_act_en <= !i_idle || st_we || (wpix_st_sr != 6'd0);
			wpix_p2_en <= (!i_idle && (path_c == 2'd2 || path == 2'd2)) || (wpix_st_sr != 6'd0);
		end
	end
	wire wpix_st_clk = clk & wpix_st_en;
	wire wpix_act_clk = clk & wpix_act_en;
	wire wpix_p2_clk = clk & wpix_p2_en;
`endif

	wire [2:0] st_we_b;
	wire [8:0] st_idx_b [0:2];
`ifdef SYNTH_KINTEX7
	genvar g_stb, g_sti;
	generate
	for (g_stb = 0; g_stb < 3; g_stb = g_stb + 1) begin : g_st_we_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stwe (.I0(st_we), .O(st_we_b[g_stb]));
		for (g_sti = 0; g_sti < 9; g_sti = g_sti + 1) begin : g_st_idx_buf
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_stix (.I0(st_idx[g_sti]), .O(st_idx_b[g_stb][g_sti]));
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_stb, g_sti;
	generate
	for (g_stb = 0; g_stb < 3; g_stb = g_stb + 1) begin : g_st_we_buf
		assign st_we_b[g_stb] = st_we;
		for (g_sti = 0; g_sti < 9; g_sti = g_sti + 1) begin : g_st_idx_buf
			assign st_idx_b[g_stb][g_sti] = st_idx[g_sti];
		end
	end
	endgenerate
`else
	assign st_we_b = {3{st_we}};
	assign st_idx_b[0] = st_idx;
	assign st_idx_b[1] = st_idx;
	assign st_idx_b[2] = st_idx;
`endif

	integer i;
	always @(posedge wpix_st_clk) begin
		st_wd_b0 <= st_wdata;
		st_wd_b1 <= st_wdata;
		st_wd_c0 <= st_wdata;
		st_wd_c1 <= st_wdata;
	end
	always @(posedge wpix_st_clk or posedge rst_sync) begin
		if (rst_sync) begin
			we_base_r <= 8'd0;
			we_stride_r <= 8'd0;
			we_bls_r <= 8'd0;
			we_kcf_r <= 4'd0;
			we_fmt0_r <= 1'b0;
			we_fmts0_r <= 1'b0;
			we_fmts1_r <= 1'b0;
			we_kc8_r <= 1'b0;
			we_rop_ctrl_r <= 1'b0;
			we_wm0_r <= 1'b0;
			we_wm0_hi_r <= 1'b0;
			we_wm1_r <= 1'b0;
			we_ms_r <= 1'b0;
		end else begin
			we_base_r[0]   <= st_we_b[0] && (st_idx_b[0] == GHCI_S_RT_BASE);
			we_stride_r[0] <= st_we_b[0] && (st_idx_b[0] == GHCI_S_RT_STRIDE);
			we_base_r[1]   <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd0));
			we_stride_r[1] <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd1));
			we_base_r[2]   <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd2));
			we_stride_r[2] <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd3));
			we_base_r[3]   <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd4));
			we_stride_r[3] <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT1_BASE + 9'd5));
			we_base_r[4]   <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT_EXT + 9'd0));
			we_stride_r[4] <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT_EXT + 9'd1));
			we_base_r[5]   <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT_EXT + 9'd2));
			we_stride_r[5] <= st_we_b[0] && (st_idx_b[0] == (GHCI_S_RT_EXT + 9'd3));
			we_base_r[6]   <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_RT_EXT + 9'd4));
			we_stride_r[6] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_RT_EXT + 9'd5));
			we_base_r[7]   <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_RT_EXT + 9'd6));
			we_stride_r[7] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_RT_EXT + 9'd7));

			we_bls_r[0] <= st_we_b[1] && (st_idx_b[1] == GHCI_S_BLEND);
			we_bls_r[1] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND1 + 9'd0));
			we_bls_r[2] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND1 + 9'd1));
			we_bls_r[3] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND1 + 9'd2));
			we_bls_r[4] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND4 + 9'd0));
			we_bls_r[5] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND4 + 9'd1));
			we_bls_r[6] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND4 + 9'd2));
			we_bls_r[7] <= st_we_b[1] && (st_idx_b[1] == (GHCI_S_BLEND4 + 9'd3));

			we_kcf_r[0] <= st_we_b[2] && (st_idx_b[2] == (GHCI_S_BLEND_CONST_R + 9'd0));
			we_kcf_r[1] <= st_we_b[2] && (st_idx_b[2] == (GHCI_S_BLEND_CONST_R + 9'd1));
			we_kcf_r[2] <= st_we_b[2] && (st_idx_b[2] == (GHCI_S_BLEND_CONST_R + 9'd2));
			we_kcf_r[3] <= st_we_b[2] && (st_idx_b[2] == (GHCI_S_BLEND_CONST_R + 9'd3));

			we_fmt0_r     <= st_we_b[2] && (st_idx_b[2] == GHCI_S_RT_FORMAT);
			we_fmts0_r    <= st_we_b[2] && (st_idx_b[2] == GHCI_S_RT_FORMATS);
			we_fmts1_r    <= st_we_b[2] && (st_idx_b[2] == GHCI_S_RT_FORMATS1 || st_idx_b[2] == 9'h08E);
			we_kc8_r      <= st_we_b[2] && (st_idx_b[2] == GHCI_S_BLEND_CONST);
			we_rop_ctrl_r <= st_we_b[2] && (st_idx_b[2] == GHCI_S_ROP_CTRL);
			we_wm0_r      <= st_we_b[2] && (st_idx_b[2] == GHCI_S_WRITE_MASK);
			we_wm0_hi_r   <= st_we_b[2] && (st_idx_b[2] == GHCI_S_WRITE_MASK) && (st_wdata[31:16] != 16'd0);
			we_wm1_r      <= st_we_b[2] && (st_idx_b[2] == GHCI_S_WRITE_MASK1);
			we_ms_r       <= st_we_b[2] && (st_idx_b[2] == GHCI_S_MS_CTRL);
		end
	end

`ifdef SYNTH_KINTEX7
	genvar g_we;
	generate
	for (g_we = 0; g_we < MRT_COUNT; g_we = g_we + 1) begin : g_st_bs
		wire we_b_lo, we_b_hi, we_s_lo, we_s_hi, we_bl_lo, we_bl_hi;
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_blo (.I0(we_base_r[g_we]), .O(we_b_lo));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_bhi (.I0(we_base_r[g_we]), .O(we_b_hi));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_slo (.I0(we_stride_r[g_we]), .O(we_s_lo));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_shi (.I0(we_stride_r[g_we]), .O(we_s_hi));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_llo (.I0(we_bls_r[g_we]), .O(we_bl_lo));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_lhi (.I0(we_bls_r[g_we]), .O(we_bl_hi));
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				base[g_we] <= 32'd0;
				stride[g_we] <= 32'd0;
				bls[g_we] <= 23'd0;
			end else begin
				if (we_b_lo)  base[g_we][15:0]    <= (g_we < 4) ? st_wd_b0[15:0]  : st_wd_b1[15:0];
				if (we_b_hi)  base[g_we][31:16]   <= (g_we < 4) ? st_wd_b0[31:16] : st_wd_b1[31:16];
				if (we_s_lo)  stride[g_we][15:0]  <= (g_we < 4) ? st_wd_b0[15:0]  : st_wd_b1[15:0];
				if (we_s_hi)  stride[g_we][31:16] <= (g_we < 4) ? st_wd_b0[31:16] : st_wd_b1[31:16];
				if (we_bl_lo) bls[g_we][11:0]     <= st_wd_c0[11:0];
				if (we_bl_hi) bls[g_we][22:12]    <= st_wd_c0[22:12];
			end
		end
	end
	for (g_we = 0; g_we < 4; g_we = g_we + 1) begin : g_st_kcf
		wire we_k_lo, we_k_hi;
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_klo (.I0(we_kcf_r[g_we]), .O(we_k_lo));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_khi (.I0(we_kcf_r[g_we]), .O(we_k_hi));
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				kcf[g_we] <= 32'd0;
			end else begin
				if (we_k_lo) kcf[g_we][15:0]  <= st_wd_c1[15:0];
				if (we_k_hi) kcf[g_we][31:16] <= st_wd_c1[31:16];
			end
		end
	end
	endgenerate

	wire we_fmts0_lo, we_fmts0_hi, we_fmts1_lo, we_fmts1_hi, we_kc8_lo, we_kc8_hi;
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_f0lo (.I0(we_fmts0_r), .O(we_fmts0_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_f0hi (.I0(we_fmts0_r), .O(we_fmts0_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_f1lo (.I0(we_fmts1_r), .O(we_fmts1_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_f1hi (.I0(we_fmts1_r), .O(we_fmts1_hi));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_k8lo (.I0(we_kc8_r),   .O(we_kc8_lo));
	(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_k8hi (.I0(we_kc8_r),   .O(we_kc8_hi));

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			fmt0 <= 2'd0;
			fmts <= 64'd0;
			kc8 <= 32'd0;
			wms <= 32'd0;
			kc_sel <= 1'b0;
			ms4 <= 1'b0;
		end else begin
			if (we_fmt0_r)     fmt0 <= st_wd_c0[1:0];
			if (we_fmts0_lo)   fmts[15:0]  <= st_wd_c0[15:0];
			if (we_fmts0_hi)   fmts[31:16] <= st_wd_c0[31:16];
			if (we_fmts1_lo)   fmts[47:32] <= st_wd_c1[15:0];
			if (we_fmts1_hi)   fmts[63:48] <= st_wd_c1[31:16];
			if (we_kc8_lo)     kc8[15:0]   <= st_wd_c1[15:0];
			if (we_kc8_hi)     kc8[31:16]  <= st_wd_c1[31:16];
			if (we_rop_ctrl_r) kc_sel      <= st_wd_c0[1];
			if (we_wm0_r)      wms[15:0]   <= st_wd_c0[15:0];
			if (we_wm0_hi_r)   wms[31:16]  <= st_wd_c0[31:16];
			else if (we_wm1_r) wms[31:16]  <= st_wd_c1[15:0];
			if (we_ms_r)       ms4         <= (st_wd_c0[3:0] == 4'd2);
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_we;
	generate
	for (g_we = 0; g_we < MRT_COUNT; g_we = g_we + 1) begin : g_st_bs
		wire we_b_lo, we_b_hi, we_s_lo, we_s_hi, we_bl_lo, we_bl_hi;
		assign we_b_lo = we_base_r[g_we];
		assign we_b_hi = we_base_r[g_we];
		assign we_s_lo = we_stride_r[g_we];
		assign we_s_hi = we_stride_r[g_we];
		assign we_bl_lo = we_bls_r[g_we];
		assign we_bl_hi = we_bls_r[g_we];
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				base[g_we] <= 32'd0;
				stride[g_we] <= 32'd0;
				bls[g_we] <= 23'd0;
			end else begin
				if (we_b_lo)  base[g_we][15:0]    <= (g_we < 4) ? st_wd_b0[15:0]  : st_wd_b1[15:0];
				if (we_b_hi)  base[g_we][31:16]   <= (g_we < 4) ? st_wd_b0[31:16] : st_wd_b1[31:16];
				if (we_s_lo)  stride[g_we][15:0]  <= (g_we < 4) ? st_wd_b0[15:0]  : st_wd_b1[15:0];
				if (we_s_hi)  stride[g_we][31:16] <= (g_we < 4) ? st_wd_b0[31:16] : st_wd_b1[31:16];
				if (we_bl_lo) bls[g_we][11:0]     <= st_wd_c0[11:0];
				if (we_bl_hi) bls[g_we][22:12]    <= st_wd_c0[22:12];
			end
		end
	end
	for (g_we = 0; g_we < 4; g_we = g_we + 1) begin : g_st_kcf
		wire we_k_lo, we_k_hi;
		assign we_k_lo = we_kcf_r[g_we];
		assign we_k_hi = we_kcf_r[g_we];
		always @(posedge clk or posedge rst_sync) begin
			if (rst_sync) begin
				kcf[g_we] <= 32'd0;
			end else begin
				if (we_k_lo) kcf[g_we][15:0]  <= st_wd_c1[15:0];
				if (we_k_hi) kcf[g_we][31:16] <= st_wd_c1[31:16];
			end
		end
	end
	endgenerate

	wire we_fmts0_lo, we_fmts0_hi, we_fmts1_lo, we_fmts1_hi, we_kc8_lo, we_kc8_hi;
	assign we_fmts0_lo = we_fmts0_r;
	assign we_fmts0_hi = we_fmts0_r;
	assign we_fmts1_lo = we_fmts1_r;
	assign we_fmts1_hi = we_fmts1_r;
	assign we_kc8_lo = we_kc8_r;
	assign we_kc8_hi = we_kc8_r;

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			fmt0 <= 2'd0;
			fmts <= 64'd0;
			kc8 <= 32'd0;
			wms <= 32'd0;
			kc_sel <= 1'b0;
			ms4 <= 1'b0;
		end else begin
			if (we_fmt0_r)     fmt0 <= st_wd_c0[1:0];
			if (we_fmts0_lo)   fmts[15:0]  <= st_wd_c0[15:0];
			if (we_fmts0_hi)   fmts[31:16] <= st_wd_c0[31:16];
			if (we_fmts1_lo)   fmts[47:32] <= st_wd_c1[15:0];
			if (we_fmts1_hi)   fmts[63:48] <= st_wd_c1[31:16];
			if (we_kc8_lo)     kc8[15:0]   <= st_wd_c1[15:0];
			if (we_kc8_hi)     kc8[31:16]  <= st_wd_c1[31:16];
			if (we_rop_ctrl_r) kc_sel      <= st_wd_c0[1];
			if (we_wm0_r)      wms[15:0]   <= st_wd_c0[15:0];
			if (we_wm0_hi_r)   wms[31:16]  <= st_wd_c0[31:16];
			else if (we_wm1_r) wms[31:16]  <= st_wd_c1[15:0];
			if (we_ms_r)       ms4         <= (st_wd_c0[3:0] == 4'd2);
		end
	end
`else
	/* In simulation, also update base/stride/fmts/wms on the same cycle as st_we
	 * because atg_rop_mrt8_tb checks dut_wb.wpix.base/stride/fmts/wms 1 cycle after st_put. */
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			for (i = 0; i < MRT_COUNT; i = i + 1) begin
				base[i] <= 32'd0;
				stride[i] <= 32'd0;
				bls[i] <= 23'd0;
			end
			for (i = 0; i < 4; i = i + 1)
				kcf[i] <= 32'd0;
			fmt0 <= 2'd0;
			fmts <= 64'd0;
			kc8 <= 32'd0;
			wms <= 32'd0;
			kc_sel <= 1'b0;
			ms4 <= 1'b0;
		end else if (st_we) begin
			if (st_idx == GHCI_S_RT_BASE)              base[0] <= st_wdata;
			if (st_idx == GHCI_S_RT_STRIDE)            stride[0] <= st_wdata;
			if (MRT_COUNT > 1 && st_idx == (GHCI_S_RT1_BASE + 9'd0))    base[1] <= st_wdata;
			if (MRT_COUNT > 1 && st_idx == (GHCI_S_RT1_BASE + 9'd1))    stride[1] <= st_wdata;
			if (MRT_COUNT > 2 && st_idx == (GHCI_S_RT1_BASE + 9'd2))    base[2] <= st_wdata;
			if (MRT_COUNT > 2 && st_idx == (GHCI_S_RT1_BASE + 9'd3))    stride[2] <= st_wdata;
			if (MRT_COUNT > 3 && st_idx == (GHCI_S_RT1_BASE + 9'd4))    base[3] <= st_wdata;
			if (MRT_COUNT > 3 && st_idx == (GHCI_S_RT1_BASE + 9'd5))    stride[3] <= st_wdata;
			if (MRT_COUNT > 4 && st_idx == (GHCI_S_RT_EXT + 9'd0))      base[4] <= st_wdata;
			if (MRT_COUNT > 4 && st_idx == (GHCI_S_RT_EXT + 9'd1))      stride[4] <= st_wdata;
			if (MRT_COUNT > 5 && st_idx == (GHCI_S_RT_EXT + 9'd2))      base[5] <= st_wdata;
			if (MRT_COUNT > 5 && st_idx == (GHCI_S_RT_EXT + 9'd3))      stride[5] <= st_wdata;
			if (MRT_COUNT > 6 && st_idx == (GHCI_S_RT_EXT + 9'd4))      base[6] <= st_wdata;
			if (MRT_COUNT > 6 && st_idx == (GHCI_S_RT_EXT + 9'd5))      stride[6] <= st_wdata;
			if (MRT_COUNT > 7 && st_idx == (GHCI_S_RT_EXT + 9'd6))      base[7] <= st_wdata;
			if (MRT_COUNT > 7 && st_idx == (GHCI_S_RT_EXT + 9'd7))      stride[7] <= st_wdata;
			if (st_idx == GHCI_S_RT_FORMAT)            fmt0 <= st_wdata[1:0];
			if (st_idx == GHCI_S_RT_FORMATS)           fmts[31:0] <= st_wdata;
			if (st_idx == GHCI_S_RT_FORMATS1 || st_idx == 9'h08E) fmts[63:32] <= st_wdata;
			if (st_idx == GHCI_S_BLEND)                bls[0] <= st_wdata[22:0];
			if (MRT_COUNT > 1 && st_idx == (GHCI_S_BLEND1 + 9'd0))      bls[1] <= st_wdata[22:0];
			if (MRT_COUNT > 2 && st_idx == (GHCI_S_BLEND1 + 9'd1))      bls[2] <= st_wdata[22:0];
			if (MRT_COUNT > 3 && st_idx == (GHCI_S_BLEND1 + 9'd2))      bls[3] <= st_wdata[22:0];
			if (MRT_COUNT > 4 && st_idx == (GHCI_S_BLEND4 + 9'd0))      bls[4] <= st_wdata[22:0];
			if (MRT_COUNT > 5 && st_idx == (GHCI_S_BLEND4 + 9'd1))      bls[5] <= st_wdata[22:0];
			if (MRT_COUNT > 6 && st_idx == (GHCI_S_BLEND4 + 9'd2))      bls[6] <= st_wdata[22:0];
			if (MRT_COUNT > 7 && st_idx == (GHCI_S_BLEND4 + 9'd3))      bls[7] <= st_wdata[22:0];
			if (st_idx == GHCI_S_BLEND_CONST)          kc8 <= st_wdata;
			if (st_idx == (GHCI_S_BLEND_CONST_R + 9'd0)) kcf[0] <= st_wdata;
			if (st_idx == (GHCI_S_BLEND_CONST_R + 9'd1)) kcf[1] <= st_wdata;
			if (st_idx == (GHCI_S_BLEND_CONST_R + 9'd2)) kcf[2] <= st_wdata;
			if (st_idx == (GHCI_S_BLEND_CONST_R + 9'd3)) kcf[3] <= st_wdata;
			if (st_idx == GHCI_S_ROP_CTRL)             kc_sel <= st_wdata[1];
			if (st_idx == GHCI_S_WRITE_MASK) begin
				wms[15:0] <= st_wdata[15:0];
				if (st_wdata[31:16] != 16'd0)
					wms[31:16] <= st_wdata[31:16];
			end
			if (st_idx == GHCI_S_WRITE_MASK1)          wms[31:16] <= st_wdata[15:0];
			if (st_idx == GHCI_S_MS_CTRL)              ms4 <= (ENABLE_MSAA != 0) && (st_wdata[3:0] == 4'd2);
		end
	end
`endif

	/* 2-stage registered RT format properties (<= 2 LUT levels per stage, 0 CARRY4s) */
	genvar grt;
	generate
	for (grt = 0; grt < 8; grt = grt + 1) begin : g_cfg
		wire [7:0] fsel_k = fmts[8 * grt +: 8];
		wire [7:0] vk_comb = (grt == 0 && fsel_k == 8'd0) ?
			(fmt0 == 2'd0 ? 8'd44 : fmt0 == 2'd1 ? 8'd37 : 8'd4) : fsel_k;

		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [7:0] r_vk_s0_a;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [7:0] r_vk_s0_b;
		(* keep = "true", dont_touch = "true" *) reg [3:0] r_wm_s0;
		(* keep = "true", dont_touch = "true" *) reg [22:0] r_bl_s0;
		(* keep = "true", dont_touch = "true" *) reg r_ms4_s0;
		always @(posedge wpix_st_clk or posedge rst_sync) begin
			if (rst_sync) begin
				r_vk_s0_a <= 8'd44;
				r_vk_s0_b <= 8'd44;
				r_wm_s0 <= 4'hF;
				r_bl_s0 <= 23'd0;
				r_ms4_s0 <= 1'b0;
			end else begin
				r_vk_s0_a <= vk_comb;
				r_vk_s0_b <= vk_comb;
				r_wm_s0 <= wms[4 * ((MRT_COUNT == 1) ? 0 : grt) +: 4];
				r_bl_s0 <= bls[(MRT_COUNT == 1) ? 0 : grt];
				r_ms4_s0 <= ms4;
			end
		end

		(* rom_style = "distributed", ram_style = "distributed" *) wire [67:0] inf_k = atg_fmt_info(r_vk_s0_a);
		(* rom_style = "distributed", ram_style = "distributed" *) wire [25:0] feat_k = atg_fmt_feat(r_vk_s0_b, 1'b0);
		wire [4:0] ps_k = inf_k[5:1];
		/* ps_k is 1-hot power-of-2 (1,2,4,8,16); flat OR decode cuts psl_k depth to 1 cmos3 */
		wire [2:0] psl_k = {ps_k[4], ps_k[3] | ps_k[2], ps_k[3] | ps_k[1]};
		wire [3:0] cmask_k = inf_k[8:6] >= 3'd4 ? 4'hF : inf_k[8:6] == 3'd3 ? 4'h7 :
			inf_k[8:6] == 3'd2 ? 4'h3 : 4'h1;
		wire p_u8_k = r_vk_s0_b == 8'd44 || r_vk_s0_b == 8'd37 || r_vk_s0_b == 8'd51 || r_vk_s0_b == 8'd4 ||
			r_vk_s0_b == 8'd9 || r_vk_s0_b == 8'd16 || r_vk_s0_b == 8'd8;
		wire p_f32_k = (ENABLE_FBLEND != 0) &&
			(r_vk_s0_b == 8'd43 || r_vk_s0_b == 8'd50 || r_vk_s0_b == 8'd57 || r_vk_s0_b == 8'd64 ||
			 r_vk_s0_b == 8'd76 || r_vk_s0_b == 8'd83 || r_vk_s0_b == 8'd97);
		wire [1:0] path_k = p_u8_k ? 2'd0 : p_f32_k ? 2'd1 : 2'd2;
		wire [3:0] segl_k = r_ms4_s0 ?
			(ps_k[4] ? 4'd7 : ps_k[3] ? 4'd6 : ps_k[2] ? 4'd5 : ps_k[1] ? 4'd4 : 4'd3) :
			(ps_k[4] ? 4'd5 : ps_k[3] ? 4'd4 : ps_k[2] ? 4'd3 : ps_k[1] ? 4'd2 : 4'd1);
		wire multi_k = r_ms4_s0 ?
			((LINE_SHIFT == 5) ? (ps_k[4] | ps_k[3]) : ps_k[4]) : 1'b0;
		wire [3:0] nll_k = (LINE_SHIFT == 5) ?
			(ps_k[4] ? 4'd2 : ps_k[3] ? 4'd1 : 4'd0) :
			(ps_k[4] ? 4'd1 : 4'd0);
		wire [1:0] nl_val_k = (LINE_SHIFT == 5) ?
			((ps_k[4] & r_ms4_s0) ? 2'd3 : (ps_k[3] & r_ms4_s0) ? 2'd1 : 2'd0) :
			((ps_k[4] & r_ms4_s0) ? 2'd1 : 2'd0);
		wire [1:0] np_val_k = (!multi_k && r_ms4_s0) ? 2'd3 : 2'd0;
		wire [2:0] xsh_k = r_ms4_s0 ?
			(ps_k[4] ? 3'd6 : ps_k[3] ? 3'd5 : ps_k[2] ? 3'd4 : ps_k[1] ? 3'd3 : 3'd2) :
			psl_k;
		wire [2:0] el_k = (LINE_SHIFT == 5) ?
			(ps_k[4] ? 3'd1 : ps_k[3] ? 3'd2 : ps_k[2] ? 3'd3 : ps_k[1] ? 3'd4 : 3'd5) :
			(ps_k[4] ? 3'd2 : ps_k[3] ? 3'd3 : ps_k[2] ? 3'd4 : ps_k[1] ? 3'd5 : 3'd6);
		wire [2:0] el_oh_k = (LINE_SHIFT == 5) ?
			{ps_k[3], ps_k[4], 1'b0} :
			{ps_k[4], 2'b00};
		wire [3:0] dst_mask_k = {
			!ps_k[4],
			!(ps_k[4] | ps_k[3]),
			!(ps_k[4] | ps_k[3] | ps_k[2]),
			ps_k[0]
		};
		wire dst_en4_k  = !ps_k[0];
		wire dst_en8_k  = !(ps_k[0] | ps_k[1]);
		wire dst_en16_k = ps_k[4] | ps_k[3];
		wire dst_en64_k = ps_k[4];
		wire [4:0] psl_oh_k = ps_k;
		wire unused_segl_k = &{1'b0, segl_k};

		wire drop_k = !feat_k[7] || ((r_wm_s0 & cmask_k) == 4'd0);
		wire read_k = (r_bl_s0[0] && (p_u8_k || p_f32_k)) || ((r_wm_s0 & cmask_k) != cmask_k);

		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [7:0] r_vk;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [4:0] r_ps;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [2:0] r_psl;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [1:0] r_path;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg r_drop, r_read, r_multi;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [3:0] r_nll;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [1:0] r_nl_val, r_np_val;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [2:0] r_xsh, r_el, r_el_oh;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [3:0] r_wm, r_dst_mask;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg r_dst_en4, r_dst_en8, r_dst_en16, r_dst_en64;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [4:0] r_psl_oh;
		(* rom_style = "distributed", ram_style = "distributed", keep = "true", dont_touch = "true" *) reg [22:0] r_bl;

		always @(posedge wpix_st_clk or posedge rst_sync) begin
			if (rst_sync) begin
				r_vk <= 8'd44;
				r_ps <= 5'd4;
				r_psl <= 3'd2;
				r_path <= 2'd0;
				r_drop <= 1'b0;
				r_read <= 1'b0;
				r_multi <= 1'b0;
				r_nll <= 4'd0;
				r_nl_val <= 2'd0;
				r_np_val <= 2'd0;
				r_xsh <= 3'd2;
				r_el <= 3'd3;
				r_el_oh <= 3'b000;
				r_wm <= 4'hF;
				r_dst_mask <= 4'b1100;
				r_dst_en4 <= 1'b1;
				r_dst_en8 <= 1'b1;
				r_dst_en16 <= 1'b0;
				r_dst_en64 <= 1'b0;
				r_psl_oh <= 5'b00100;
				r_bl <= 23'd0;
			end else begin
				r_vk <= r_vk_s0_b;
				r_ps <= ps_k;
				r_psl <= psl_k;
				r_path <= path_k;
				r_drop <= drop_k;
				r_read <= read_k;
				r_multi <= multi_k;
				r_nll <= nll_k;
				r_nl_val <= nl_val_k;
				r_np_val <= np_val_k;
				r_xsh <= xsh_k;
				r_el <= el_k;
				r_el_oh <= el_oh_k;
				r_wm <= r_wm_s0;
				r_dst_mask <= dst_mask_k;
				r_dst_en4 <= dst_en4_k;
				r_dst_en8 <= dst_en8_k;
				r_dst_en16 <= dst_en16_k;
				r_dst_en64 <= dst_en64_k;
				r_psl_oh <= psl_oh_k;
				r_bl <= r_bl_s0;
			end
		end
	end
	endgenerate

	/* Replicated i_rt buffers so 8:1 muxes have fanout <= 16 (<= 8 bits per 8:1 mux group) */
`ifdef SYNTH_KINTEX7
	wire [2:0] i_rt_m0, i_rt_m1;
	wire [2:0] i_rt_c0a, i_rt_c0b, i_rt_c1, i_rt_c1a, i_rt_c1b, i_rt_c2, i_rt_c3, i_rt_c4, i_rt_c5, i_rt_c6, i_rt_c7;
	wire [2:0] i_rt_b0, i_rt_b1, i_rt_b2, i_rt_b3;
	wire [2:0] i_rt_s0, i_rt_s1, i_rt_s2, i_rt_s3;
	genvar g_rtb;
	generate
	for (g_rtb = 0; g_rtb < 3; g_rtb = g_rtb + 1) begin : g_rt_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtm0 (.I0(i_rt[g_rtb]), .O(i_rt_m0[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtm1 (.I0(i_rt[g_rtb]), .O(i_rt_m1[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc0a (.I0(i_rt_m0[g_rtb]), .O(i_rt_c0a[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc0b (.I0(i_rt_m0[g_rtb]), .O(i_rt_c0b[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc1  (.I0(i_rt_m0[g_rtb]), .O(i_rt_c1[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc1a (.I0(i_rt_m0[g_rtb]), .O(i_rt_c1a[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc1b (.I0(i_rt_m0[g_rtb]), .O(i_rt_c1b[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc2 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c2[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc3 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c3[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc4 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c4[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc5 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c5[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc6 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c6[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtc7 (.I0(i_rt_m0[g_rtb]), .O(i_rt_c7[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtb0 (.I0(i_rt_m1[g_rtb]), .O(i_rt_b0[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtb1 (.I0(i_rt_m1[g_rtb]), .O(i_rt_b1[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtb2 (.I0(i_rt_m1[g_rtb]), .O(i_rt_b2[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rtb3 (.I0(i_rt_m1[g_rtb]), .O(i_rt_b3[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rts0 (.I0(i_rt_m1[g_rtb]), .O(i_rt_s0[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rts1 (.I0(i_rt_m1[g_rtb]), .O(i_rt_s1[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rts2 (.I0(i_rt_m1[g_rtb]), .O(i_rt_s2[g_rtb]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_rts3 (.I0(i_rt_m1[g_rtb]), .O(i_rt_s3[g_rtb]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [2:0] i_rt_m0, i_rt_m1;
	wire [2:0] i_rt_c0a, i_rt_c0b, i_rt_c1, i_rt_c1a, i_rt_c1b, i_rt_c2, i_rt_c3, i_rt_c4, i_rt_c5, i_rt_c6, i_rt_c7;
	wire [2:0] i_rt_b0, i_rt_b1, i_rt_b2, i_rt_b3;
	wire [2:0] i_rt_s0, i_rt_s1, i_rt_s2, i_rt_s3;
	genvar g_rtb;
	generate
	for (g_rtb = 0; g_rtb < 3; g_rtb = g_rtb + 1) begin : g_rt_buf
		assign i_rt_m0[g_rtb] = i_rt[g_rtb];
		assign i_rt_m1[g_rtb] = i_rt[g_rtb];
		assign i_rt_c0a[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c0b[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c1[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c1a[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c1b[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c2[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c3[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c4[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c5[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c6[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_c7[g_rtb] = i_rt_m0[g_rtb];
		assign i_rt_b0[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_b1[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_b2[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_b3[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_s0[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_s1[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_s2[g_rtb] = i_rt_m1[g_rtb];
		assign i_rt_s3[g_rtb] = i_rt_m1[g_rtb];
	end
	endgenerate
`else
	wire [2:0] i_rt_c0a = i_rt, i_rt_c0b = i_rt, i_rt_c1 = i_rt, i_rt_c1a = i_rt, i_rt_c1b = i_rt, i_rt_c2 = i_rt, i_rt_c3 = i_rt;
	wire [2:0] i_rt_c4 = i_rt, i_rt_c5 = i_rt, i_rt_c6 = i_rt, i_rt_c7 = i_rt;
	wire [2:0] i_rt_b0 = i_rt_b, i_rt_b1 = i_rt_b, i_rt_b2 = i_rt_b, i_rt_b3 = i_rt_b;
	wire [2:0] i_rt_s0 = i_rt_s, i_rt_s1 = i_rt_s, i_rt_s2 = i_rt_s, i_rt_s3 = i_rt_s;
`endif

	reg [7:0] vk_c;
	reg [4:0] ps_c;
	reg [2:0] psl_c;
	reg [1:0] path_c;
	reg drop_c, read_c, multi_c;
	reg [3:0] nll_c;
	reg [1:0] nl_val_c, np_val_c;
	reg [2:0] xsh_c, el_c, el_oh_c;
	reg [3:0] wm_c, dst_mask_c;
	reg dst_en4_c, dst_en8_c, dst_en16_c, dst_en64_c;
	reg [4:0] psl_oh_c;
	reg [22:0] bl_c;

	always @(*) begin
		case (i_rt_c0a)
		3'd0: vk_c[3:0] = g_cfg[0].r_vk[3:0];
		3'd1: vk_c[3:0] = g_cfg[1].r_vk[3:0];
		3'd2: vk_c[3:0] = g_cfg[2].r_vk[3:0];
		3'd3: vk_c[3:0] = g_cfg[3].r_vk[3:0];
		3'd4: vk_c[3:0] = g_cfg[4].r_vk[3:0];
		3'd5: vk_c[3:0] = g_cfg[5].r_vk[3:0];
		3'd6: vk_c[3:0] = g_cfg[6].r_vk[3:0];
		default: vk_c[3:0] = g_cfg[7].r_vk[3:0];
		endcase
		case (i_rt_c0b)
		3'd0: vk_c[7:4] = g_cfg[0].r_vk[7:4];
		3'd1: vk_c[7:4] = g_cfg[1].r_vk[7:4];
		3'd2: vk_c[7:4] = g_cfg[2].r_vk[7:4];
		3'd3: vk_c[7:4] = g_cfg[3].r_vk[7:4];
		3'd4: vk_c[7:4] = g_cfg[4].r_vk[7:4];
		3'd5: vk_c[7:4] = g_cfg[5].r_vk[7:4];
		3'd6: vk_c[7:4] = g_cfg[6].r_vk[7:4];
		default: vk_c[7:4] = g_cfg[7].r_vk[7:4];
		endcase
		case (i_rt_c1)
		3'd0: begin ps_c = g_cfg[0].r_ps; psl_c = g_cfg[0].r_psl; end
		3'd1: begin ps_c = g_cfg[1].r_ps; psl_c = g_cfg[1].r_psl; end
		3'd2: begin ps_c = g_cfg[2].r_ps; psl_c = g_cfg[2].r_psl; end
		3'd3: begin ps_c = g_cfg[3].r_ps; psl_c = g_cfg[3].r_psl; end
		3'd4: begin ps_c = g_cfg[4].r_ps; psl_c = g_cfg[4].r_psl; end
		3'd5: begin ps_c = g_cfg[5].r_ps; psl_c = g_cfg[5].r_psl; end
		3'd6: begin ps_c = g_cfg[6].r_ps; psl_c = g_cfg[6].r_psl; end
		default: begin ps_c = g_cfg[7].r_ps; psl_c = g_cfg[7].r_psl; end
		endcase
		case (i_rt_c1a)
		3'd0: psl_oh_c = g_cfg[0].r_psl_oh;
		3'd1: psl_oh_c = g_cfg[1].r_psl_oh;
		3'd2: psl_oh_c = g_cfg[2].r_psl_oh;
		3'd3: psl_oh_c = g_cfg[3].r_psl_oh;
		3'd4: psl_oh_c = g_cfg[4].r_psl_oh;
		3'd5: psl_oh_c = g_cfg[5].r_psl_oh;
		3'd6: psl_oh_c = g_cfg[6].r_psl_oh;
		default: psl_oh_c = g_cfg[7].r_psl_oh;
		endcase
		case (i_rt_c1b)
		3'd0: begin dst_mask_c = g_cfg[0].r_dst_mask; dst_en4_c = g_cfg[0].r_dst_en4; dst_en8_c = g_cfg[0].r_dst_en8; dst_en16_c = g_cfg[0].r_dst_en16; dst_en64_c = g_cfg[0].r_dst_en64; end
		3'd1: begin dst_mask_c = g_cfg[1].r_dst_mask; dst_en4_c = g_cfg[1].r_dst_en4; dst_en8_c = g_cfg[1].r_dst_en8; dst_en16_c = g_cfg[1].r_dst_en16; dst_en64_c = g_cfg[1].r_dst_en64; end
		3'd2: begin dst_mask_c = g_cfg[2].r_dst_mask; dst_en4_c = g_cfg[2].r_dst_en4; dst_en8_c = g_cfg[2].r_dst_en8; dst_en16_c = g_cfg[2].r_dst_en16; dst_en64_c = g_cfg[2].r_dst_en64; end
		3'd3: begin dst_mask_c = g_cfg[3].r_dst_mask; dst_en4_c = g_cfg[3].r_dst_en4; dst_en8_c = g_cfg[3].r_dst_en8; dst_en16_c = g_cfg[3].r_dst_en16; dst_en64_c = g_cfg[3].r_dst_en64; end
		3'd4: begin dst_mask_c = g_cfg[4].r_dst_mask; dst_en4_c = g_cfg[4].r_dst_en4; dst_en8_c = g_cfg[4].r_dst_en8; dst_en16_c = g_cfg[4].r_dst_en16; dst_en64_c = g_cfg[4].r_dst_en64; end
		3'd5: begin dst_mask_c = g_cfg[5].r_dst_mask; dst_en4_c = g_cfg[5].r_dst_en4; dst_en8_c = g_cfg[5].r_dst_en8; dst_en16_c = g_cfg[5].r_dst_en16; dst_en64_c = g_cfg[5].r_dst_en64; end
		3'd6: begin dst_mask_c = g_cfg[6].r_dst_mask; dst_en4_c = g_cfg[6].r_dst_en4; dst_en8_c = g_cfg[6].r_dst_en8; dst_en16_c = g_cfg[6].r_dst_en16; dst_en64_c = g_cfg[6].r_dst_en64; end
		default: begin dst_mask_c = g_cfg[7].r_dst_mask; dst_en4_c = g_cfg[7].r_dst_en4; dst_en8_c = g_cfg[7].r_dst_en8; dst_en16_c = g_cfg[7].r_dst_en16; dst_en64_c = g_cfg[7].r_dst_en64; end
		endcase
		case (i_rt_c2)
		3'd0: begin path_c = g_cfg[0].r_path; drop_c = g_cfg[0].r_drop; read_c = g_cfg[0].r_read; multi_c = g_cfg[0].r_multi; end
		3'd1: begin path_c = g_cfg[1].r_path; drop_c = g_cfg[1].r_drop; read_c = g_cfg[1].r_read; multi_c = g_cfg[1].r_multi; end
		3'd2: begin path_c = g_cfg[2].r_path; drop_c = g_cfg[2].r_drop; read_c = g_cfg[2].r_read; multi_c = g_cfg[2].r_multi; end
		3'd3: begin path_c = g_cfg[3].r_path; drop_c = g_cfg[3].r_drop; read_c = g_cfg[3].r_read; multi_c = g_cfg[3].r_multi; end
		3'd4: begin path_c = g_cfg[4].r_path; drop_c = g_cfg[4].r_drop; read_c = g_cfg[4].r_read; multi_c = g_cfg[4].r_multi; end
		3'd5: begin path_c = g_cfg[5].r_path; drop_c = g_cfg[5].r_drop; read_c = g_cfg[5].r_read; multi_c = g_cfg[5].r_multi; end
		3'd6: begin path_c = g_cfg[6].r_path; drop_c = g_cfg[6].r_drop; read_c = g_cfg[6].r_read; multi_c = g_cfg[6].r_multi; end
		default: begin path_c = g_cfg[7].r_path; drop_c = g_cfg[7].r_drop; read_c = g_cfg[7].r_read; multi_c = g_cfg[7].r_multi; end
		endcase
		case (i_rt_c3)
		3'd0: begin nll_c = g_cfg[0].r_nll; xsh_c = g_cfg[0].r_xsh; nl_val_c = g_cfg[0].r_nl_val; np_val_c = g_cfg[0].r_np_val; end
		3'd1: begin nll_c = g_cfg[1].r_nll; xsh_c = g_cfg[1].r_xsh; nl_val_c = g_cfg[1].r_nl_val; np_val_c = g_cfg[1].r_np_val; end
		3'd2: begin nll_c = g_cfg[2].r_nll; xsh_c = g_cfg[2].r_xsh; nl_val_c = g_cfg[2].r_nl_val; np_val_c = g_cfg[2].r_np_val; end
		3'd3: begin nll_c = g_cfg[3].r_nll; xsh_c = g_cfg[3].r_xsh; nl_val_c = g_cfg[3].r_nl_val; np_val_c = g_cfg[3].r_np_val; end
		3'd4: begin nll_c = g_cfg[4].r_nll; xsh_c = g_cfg[4].r_xsh; nl_val_c = g_cfg[4].r_nl_val; np_val_c = g_cfg[4].r_np_val; end
		3'd5: begin nll_c = g_cfg[5].r_nll; xsh_c = g_cfg[5].r_xsh; nl_val_c = g_cfg[5].r_nl_val; np_val_c = g_cfg[5].r_np_val; end
		3'd6: begin nll_c = g_cfg[6].r_nll; xsh_c = g_cfg[6].r_xsh; nl_val_c = g_cfg[6].r_nl_val; np_val_c = g_cfg[6].r_np_val; end
		default: begin nll_c = g_cfg[7].r_nll; xsh_c = g_cfg[7].r_xsh; nl_val_c = g_cfg[7].r_nl_val; np_val_c = g_cfg[7].r_np_val; end
		endcase
		case (i_rt_c4)
		3'd0: begin el_c = g_cfg[0].r_el; el_oh_c = g_cfg[0].r_el_oh; wm_c = g_cfg[0].r_wm; end
		3'd1: begin el_c = g_cfg[1].r_el; el_oh_c = g_cfg[1].r_el_oh; wm_c = g_cfg[1].r_wm; end
		3'd2: begin el_c = g_cfg[2].r_el; el_oh_c = g_cfg[2].r_el_oh; wm_c = g_cfg[2].r_wm; end
		3'd3: begin el_c = g_cfg[3].r_el; el_oh_c = g_cfg[3].r_el_oh; wm_c = g_cfg[3].r_wm; end
		3'd4: begin el_c = g_cfg[4].r_el; el_oh_c = g_cfg[4].r_el_oh; wm_c = g_cfg[4].r_wm; end
		3'd5: begin el_c = g_cfg[5].r_el; el_oh_c = g_cfg[5].r_el_oh; wm_c = g_cfg[5].r_wm; end
		3'd6: begin el_c = g_cfg[6].r_el; el_oh_c = g_cfg[6].r_el_oh; wm_c = g_cfg[6].r_wm; end
		default: begin el_c = g_cfg[7].r_el; el_oh_c = g_cfg[7].r_el_oh; wm_c = g_cfg[7].r_wm; end
		endcase
		case (i_rt_c5)
		3'd0: bl_c[7:0] = g_cfg[0].r_bl[7:0];
		3'd1: bl_c[7:0] = g_cfg[1].r_bl[7:0];
		3'd2: bl_c[7:0] = g_cfg[2].r_bl[7:0];
		3'd3: bl_c[7:0] = g_cfg[3].r_bl[7:0];
		3'd4: bl_c[7:0] = g_cfg[4].r_bl[7:0];
		3'd5: bl_c[7:0] = g_cfg[5].r_bl[7:0];
		3'd6: bl_c[7:0] = g_cfg[6].r_bl[7:0];
		default: bl_c[7:0] = g_cfg[7].r_bl[7:0];
		endcase
		case (i_rt_c6)
		3'd0: bl_c[15:8] = g_cfg[0].r_bl[15:8];
		3'd1: bl_c[15:8] = g_cfg[1].r_bl[15:8];
		3'd2: bl_c[15:8] = g_cfg[2].r_bl[15:8];
		3'd3: bl_c[15:8] = g_cfg[3].r_bl[15:8];
		3'd4: bl_c[15:8] = g_cfg[4].r_bl[15:8];
		3'd5: bl_c[15:8] = g_cfg[5].r_bl[15:8];
		3'd6: bl_c[15:8] = g_cfg[6].r_bl[15:8];
		default: bl_c[15:8] = g_cfg[7].r_bl[15:8];
		endcase
		case (i_rt_c7)
		3'd0: bl_c[22:16] = g_cfg[0].r_bl[22:16];
		3'd1: bl_c[22:16] = g_cfg[1].r_bl[22:16];
		3'd2: bl_c[22:16] = g_cfg[2].r_bl[22:16];
		3'd3: bl_c[22:16] = g_cfg[3].r_bl[22:16];
		3'd4: bl_c[22:16] = g_cfg[4].r_bl[22:16];
		3'd5: bl_c[22:16] = g_cfg[5].r_bl[22:16];
		3'd6: bl_c[22:16] = g_cfg[6].r_bl[22:16];
		default: bl_c[22:16] = g_cfg[7].r_bl[22:16];
		endcase
	end

	/* Registered selected attachment properties (latched every cycle from i_rt) */
	reg [7:0] vk;
	(* keep = "true", dont_touch = "true" *) reg [7:0] vk_pack_a, vk_pack_b;
	reg [4:0] ps;
	(* keep = "true", dont_touch = "true" *) reg [2:0] psl_slot [0:3];
	(* keep = "true", dont_touch = "true" *) reg [4:0] psl_oh_slot [0:3];
	(* keep = "true", dont_touch = "true" *) reg [LINE_SHIFT-1:0] psl_om_r [0:3];
	reg [1:0] path;
	reg drop, read;
	(* keep = "true", dont_touch = "true" *) reg multi_out;
	(* keep = "true", dont_touch = "true" *) reg [3:0] multi_slot;
	reg [1:0] nl_r, np_r;
	reg [2:0] xsh, el, el_oh_r;
	(* keep = "true", dont_touch = "true" *) reg [2:0] xsh_int_lo, xsh_int_hi;
	(* keep = "true", dont_touch = "true" *) reg [3:0] wm_a, wm_b;
	reg [22:0] bl;
	reg [31:0] sel_base, sel_stride;
	reg [16:0] yr_r;
	reg [15:0] x_r;

	wire [2:0] psl_c_buf [0:3];
`ifdef SYNTH_KINTEX7
	genvar g_pslb, g_pslk;
	generate
	for (g_pslk = 0; g_pslk < 4; g_pslk = g_pslk + 1) begin : g_psl_slot_buf
		for (g_pslb = 0; g_pslb < 3; g_pslb = g_pslb + 1) begin : g_psl_bit_buf
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_psl (
				.I0(psl_c[g_pslb]), .O(psl_c_buf[g_pslk][g_pslb])
			);
		end
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_pslb, g_pslk;
	generate
	for (g_pslk = 0; g_pslk < 4; g_pslk = g_pslk + 1) begin : g_psl_slot_buf
		for (g_pslb = 0; g_pslb < 3; g_pslb = g_pslb + 1) begin : g_psl_bit_buf
			assign psl_c_buf[g_pslk][g_pslb] = psl_c[g_pslb];
		end
	end
	endgenerate
`else
	assign psl_c_buf[0] = psl_c;
	assign psl_c_buf[1] = psl_c;
	assign psl_c_buf[2] = psl_c;
	assign psl_c_buf[3] = psl_c;
`endif

	(* keep = "true", dont_touch = "true" *) reg [1:0] path_rep [0:3];
	(* keep = "true", dont_touch = "true" *) reg [1:0] psl_pk_lo [0:3][0:3];
	(* keep = "true", dont_touch = "true" *) reg [3:0] psl_pk_is16 [0:3];
	(* keep = "true", dont_touch = "true" *) reg [1:0] f32_ps_sel_r [0:3][0:7];

	integer is, is2;
	always @(posedge wpix_act_clk or posedge rst_sync) begin
		if (rst_sync) begin
			vk <= 8'd44;
			vk_pack_a <= 8'd44;
			vk_pack_b <= 8'd44;
			ps <= 5'd4;
			for (is = 0; is < 4; is = is + 1) begin
				psl_slot[is] <= 3'd2;
				psl_oh_slot[is] <= 5'b00100;
				psl_om_r[is] <= {{(LINE_SHIFT-4){1'b0}}, is[1:0], 2'b00};
				path_rep[is] <= 2'd0;
				psl_pk_is16[is] <= 4'd0;
				for (is2 = 0; is2 < 4; is2 = is2 + 1)
					psl_pk_lo[is][is2] <= 2'd2;
				for (is2 = 0; is2 < 8; is2 = is2 + 1)
					f32_ps_sel_r[is][is2] <= 2'd0;
			end
			path <= 2'd0;
			drop <= 1'b0;
			read <= 1'b0;
			multi_out <= 1'b0;
			multi_slot <= 4'd0;
			nl_r <= 2'd0;
			np_r <= 2'd0;
			xsh <= 3'd2;
			xsh_int_lo <= 3'd2;
			xsh_int_hi <= 3'd2;
			el <= 3'd3;
			el_oh_r <= 3'b000;
			wm_a <= 4'hF;
			wm_b <= 4'hF;
			bl <= 23'd0;
			sel_base <= 32'd0;
			sel_stride <= 32'd0;
			yr_r <= 17'd0;
			x_r <= 16'd0;
		end else begin
			vk <= vk_c;
			vk_pack_a <= vk_c;
			vk_pack_b <= vk_c;
			ps <= ps_c;
			for (is = 0; is < 4; is = is + 1) begin
				psl_slot[is] <= psl_c_buf[is];
				psl_oh_slot[is] <= psl_oh_c;
				psl_om_r[is] <= (psl_oh_c[0] ? {{(LINE_SHIFT-2){1'b0}}, is[1:0]} :
				                 psl_oh_c[1] ? {{(LINE_SHIFT-3){1'b0}}, is[1:0], 1'b0} :
				                 psl_oh_c[2] ? {{(LINE_SHIFT-4){1'b0}}, is[1:0], 2'b00} :
				                 psl_oh_c[3] ? {{(LINE_SHIFT-5){1'b0}}, is[1:0], 3'b000} :
				                               {is[LINE_SHIFT-5:0], 4'b0000});
				path_rep[is] <= path_c;
				psl_pk_is16[is] <= {4{psl_slot[is][2]}};
`ifdef SYNTH_KINTEX7
				for (is2 = 0; is2 < 4; is2 = is2 + 1)
					psl_pk_lo[is][is2] <= (psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0];
				for (is2 = 0; is2 < 8; is2 = is2 + 1)
					f32_ps_sel_r[is][is2] <= (path_rep[is] != 2'd1) ? 2'd0 :
						((psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0]);
`elsif SYNTH_KINTEX_ULTRASCALE
				for (is2 = 0; is2 < 4; is2 = is2 + 1)
					psl_pk_lo[is][is2] <= (psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0];
				for (is2 = 0; is2 < 8; is2 = is2 + 1)
					f32_ps_sel_r[is][is2] <= (path_rep[is] != 2'd1) ? 2'd0 :
						((psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0]);
`else
				psl_pk_lo[is][0] <= (psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0];
				f32_ps_sel_r[is][0] <= (path_rep[is] != 2'd1) ? 2'd0 :
					((psl_slot[is] >= 3'd3) ? 2'd3 : psl_slot[is][1:0]);
`endif
			end
			path <= path_c;
			drop <= drop_c;
			read <= read_c;
			multi_out <= multi_c;
			multi_slot <= {4{multi_c}};
			nl_r <= nl_val_c;
			np_r <= np_val_c;
			xsh <= xsh_c;
			xsh_int_lo <= xsh_c;
			xsh_int_hi <= xsh_c;
			el <= el_c;
			el_oh_r <= el_oh_c;
			wm_a <= wm_c;
			wm_b <= wm_c;
			bl <= bl_c;
			sel_base[7:0]   <= base[(MRT_COUNT == 1) ? 0 : i_rt_b0][7:0];
			sel_base[15:8]  <= base[(MRT_COUNT == 1) ? 0 : i_rt_b1][15:8];
			sel_base[23:16] <= base[(MRT_COUNT == 1) ? 0 : i_rt_b2][23:16];
			sel_base[31:24] <= base[(MRT_COUNT == 1) ? 0 : i_rt_b3][31:24];
			sel_stride[7:0]   <= stride[(MRT_COUNT == 1) ? 0 : i_rt_s0][7:0];
			sel_stride[15:8]  <= stride[(MRT_COUNT == 1) ? 0 : i_rt_s1][15:8];
			sel_stride[23:16] <= stride[(MRT_COUNT == 1) ? 0 : i_rt_s2][23:16];
			sel_stride[31:24] <= stride[(MRT_COUNT == 1) ? 0 : i_rt_s3][31:24];
			yr_r <= {1'b0, i_xy[31:16]};
			x_r <= i_xy[15:0];
		end
	end

	assign o_path = path;
	assign o_drop = drop;
	assign o_read = read;
	assign o_ps = ps;
	assign o_multi = multi_out;
	assign o_nl = nl_r;
	assign o_np = np_r;
	assign o_xsh = xsh;
	assign o_stride = sel_stride;

	/* Row base & xoff pipeline:
	 * - u_xoff_lo / u_xoff_hi shift registered x_r by registered xsh_int_lo / xsh_int_hi -> registered xoff_r -> registered xoff_j_r (0 CARRY4s)
	 * - u_yrs multiplies registered yr_r by registered sel_stride -> registered yrs_r (0 Pre/Post-DSP LUTs)
	 * - u_a1 adds registered sel_base + registered yrs_r -> o_row_base0 (1 Pre-CARRY4 LUT, 0 Post-CARRY4 LUTs)
	 */
	wire [11:0] xoff_lo;
	wire [19:0] xoff_hi_full;
	am_shift #(.W(12), .SW(3)) u_xoff_lo (.a(x_r[11:0]), .amt(xsh_int_lo), .left(1'b1), .arith(1'b0), .y(xoff_lo));
	am_shift #(.W(20), .SW(3)) u_xoff_hi (.a({8'd0, x_r[15:4]}), .amt(xsh_int_hi), .left(1'b1), .arith(1'b0), .y(xoff_hi_full));
	wire [31:0] xoff = {8'd0, xoff_hi_full[19:8], xoff_lo};
	wire unused_xoff_hi = &{1'b0, xoff_hi_full[7:0]};

	reg [31:0] xoff_r, xoff_j_r;
	wire [31:0] yrs_p3;
`ifdef SYNTH_KINTEX7
	wire [48:0] yrs_k7;
	am_mul #(.AW(17), .BW(32), .LAT(3)) u_yrs (.clk(wpix_act_clk), .en(1'b1), .a(yr_r), .b(sel_stride), .p(yrs_k7));
	assign yrs_p3 = yrs_k7[31:0];
	wire unused_yrs_hi = &{1'b0, yrs_k7[48:32], yr_r[16]};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [48:0] yrs_k7;
	am_mul #(.AW(17), .BW(32), .LAT(3)) u_yrs (.clk(wpix_act_clk), .en(1'b1), .a(yr_r), .b(sel_stride), .p(yrs_k7));
	assign yrs_p3 = yrs_k7[31:0];
	wire unused_yrs_hi = &{1'b0, yrs_k7[48:32], yr_r[16]};
`else
	/* 3-stage Radix-4 Booth 16u x 32u -> 32u (<= 10 cmos3/stage, LAT=3 matching SYNTH_KINTEX7) */
	wire [18:0] yrs_a_pad = {2'b00, yr_r[15:0], 1'b0};
	wire [7:0] yrs_neg;
	wire [31:0] yrs_pp [0:8];
	genvar gyw;
	generate
	for (gyw = 0; gyw < 8; gyw = gyw + 1) begin : g_wyrs_booth
		wire b_m1 = yrs_a_pad[2 * gyw];
		wire b_0  = yrs_a_pad[2 * gyw + 1];
		wire b_p1 = yrs_a_pad[2 * gyw + 2];
		/* 1-hot Booth decode cuts partial product depth from 7 to 4 cmos3 */
		wire b_p1x = !b_p1 && (b_0 ^ b_m1);
		wire b_p2x = !b_p1 &&  b_0 &&  b_m1;
		wire b_n1x =  b_p1 && (b_0 ^ b_m1);
		wire b_n2x =  b_p1 && !b_0 && !b_m1;
		wire neg_j =  b_p1 && !(b_0 && b_m1);
		assign yrs_neg[gyw] = neg_j;
		wire [31 - 2*gyw : 0] u_w;
		genvar gb_w;
		for (gb_w = 0; gb_w <= 31 - 2*gyw; gb_w = gb_w + 1) begin : g_wu
			wire s0 = sel_stride[gb_w];
			wire sm;
			if (gb_w > 0) begin : g_sm
				assign sm = sel_stride[gb_w - 1];
			end else begin : g_sm0
				assign sm = 1'b0;
			end
			assign u_w[gb_w] = (b_p1x & s0) | (b_p2x & sm) | (b_n1x & ~s0) | (b_n2x & ~sm);
		end
		if (gyw == 0) begin : g_wy0
			assign yrs_pp[0] = u_w;
		end else if (gyw == 1) begin : g_wy1
			assign yrs_pp[1] = {u_w, 1'b0, yrs_neg[0]};
		end else begin : g_wym
			assign yrs_pp[gyw] = {u_w, 1'b0, yrs_neg[gyw - 1], {(2*gyw - 2){1'b0}}};
		end
	end
	assign yrs_pp[8] = {yr_r[15] ? sel_stride[15:0] : 16'd0, 1'b0, yrs_neg[7], 14'd0};

	wire [31:0] wyrs_l1 [0:5];
	for (gyw = 0; gyw < 3; gyw = gyw + 1) begin : g_wyc1
		wire [31:0] x0 = yrs_pp[3 * gyw];
		wire [31:0] x1 = yrs_pp[3 * gyw + 1];
		wire [31:0] x2 = yrs_pp[3 * gyw + 2];
		assign wyrs_l1[2 * gyw]     = x0 ^ x1 ^ x2;
		assign wyrs_l1[2 * gyw + 1] = {((x0[30:0] & x1[30:0]) | (x0[30:0] & x2[30:0]) | (x1[30:0] & x2[30:0])), 1'b0};
	end
	reg [31:0] wyrs_r1 [0:5];
	integer iwy1;
	always @(posedge wpix_act_clk) begin
		for (iwy1 = 0; iwy1 < 6; iwy1 = iwy1 + 1)
			wyrs_r1[iwy1] <= wyrs_l1[iwy1];
	end

	wire [31:0] wyrs_s1_w [0:3];
	for (gyw = 0; gyw < 2; gyw = gyw + 1) begin : g_wyc2
		wire [31:0] x0 = wyrs_r1[3 * gyw];
		wire [31:0] x1 = wyrs_r1[3 * gyw + 1];
		wire [31:0] x2 = wyrs_r1[3 * gyw + 2];
		assign wyrs_s1_w[2 * gyw]     = x0 ^ x1 ^ x2;
		assign wyrs_s1_w[2 * gyw + 1] = {((x0[30:0] & x1[30:0]) | (x0[30:0] & x2[30:0]) | (x1[30:0] & x2[30:0])), 1'b0};
	end
	/* 4 -> 2 using single 4:2 CSA on (wyrs_s1_w[0], wyrs_s1_w[1], wyrs_s1_w[2], wyrs_s1_w[3]) */
	wire [31:0] wyrs_x01 = wyrs_s1_w[0] ^ wyrs_s1_w[1];
	wire [31:0] wyrs_x23 = wyrs_s1_w[2] ^ wyrs_s1_w[3];
	wire [31:0] wyrs_x03 = wyrs_x01 ^ wyrs_x23;
	wire [31:0] wyrs_co  = {((wyrs_s1_w[0][30:0] & wyrs_s1_w[1][30:0]) | (wyrs_s1_w[2][30:0] & (wyrs_s1_w[0][30:0] ^ wyrs_s1_w[1][30:0]))), 1'b0};
	wire [31:0] wyrs_l3_s = wyrs_x03 ^ wyrs_co;
	wire [31:0] wyrs_l3_c = {((wyrs_x03[30:0] & wyrs_co[30:0]) | (~wyrs_x03[30:0] & wyrs_s1_w[3][30:0])), 1'b0};
	reg [31:0] wyrs_r2_s, wyrs_r2_c;
	always @(posedge wpix_act_clk) begin
		wyrs_r2_s <= wyrs_l3_s;
		wyrs_r2_c <= wyrs_l3_c;
	end

	wire wyrs_co_unused;
	am_add #(.W(32)) u_wyrs_fin (.a(wyrs_r2_s), .b(wyrs_r2_c), .ci(1'b0), .s(yrs_p3), .co(wyrs_co_unused));
	wire unused_yrs_hi = wyrs_co_unused || yr_r[16];
	endgenerate
`endif

	reg [31:0] yrs_r;
	reg smask_lw_r;
	always @(posedge wpix_act_clk or posedge rst_sync) begin
		if (rst_sync) begin
			xoff_r <= 32'd0;
			xoff_j_r <= 32'd0;
			yrs_r <= 32'd0;
			smask_lw_r <= 1'b0;
		end else begin
			xoff_r <= xoff;
`ifdef MUT_WPIX_LINE
			xoff_j_r <= xoff_r;
`else
			xoff_j_r <= xoff_r | (multi_slot[0] ? {{(30-LINE_SHIFT){1'b0}}, i_j, {LINE_SHIFT{1'b0}}} : 32'd0);
`endif
			yrs_r <= yrs_p3;
			smask_lw_r <= (i_smask[8 * i_r +: 8] != 8'd0);
		end
	end

	wire [31:0] a1;
	wire a1_co_unused;
	am_add #(.W(32)) u_a1 (.a(sel_base), .b(yrs_r), .ci(1'b0), .s(a1), .co(a1_co_unused));
	assign o_row_base0 = a1;
	assign o_row_base1 = 32'd0;
	assign o_xoff = xoff_r;
	assign o_xoff_j = xoff_j_r;
	assign o_addr = 32'd0;
	assign o_addr1 = 32'd0;

	/* 2-stage registered blend constant kcv (1 LUT level per stage) */
	reg [127:0] kc8_unorm_r, kcv_r;
`ifdef SYNTH_KINTEX7
	wire [7:0] kc_sel_buf;
	genvar g_kcb;
	generate
	for (g_kcb = 0; g_kcb < 8; g_kcb = g_kcb + 1) begin : g_kcsel_buf
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_kcb (.I0(kc_sel), .O(kc_sel_buf[g_kcb]));
	end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [7:0] kc_sel_buf;
	genvar g_kcb;
	generate
	for (g_kcb = 0; g_kcb < 8; g_kcb = g_kcb + 1) begin : g_kcsel_buf
		assign kc_sel_buf[g_kcb] = kc_sel;
	end
	endgenerate
`else
	wire [7:0] kc_sel_buf = {8{kc_sel}};
`endif
	wire [127:0] kcf_vec = {kcf[3], kcf[2], kcf[1], kcf[0]};
	integer ikc;
	always @(posedge wpix_st_clk or posedge rst_sync) begin
		if (rst_sync) begin
			kc8_unorm_r <= 128'd0;
			kcv_r <= 128'd0;
		end else begin
			kc8_unorm_r <= {
				rom_unorm8[kc8[31:24]],
				rom_unorm8[kc8[23:16]],
				rom_unorm8[kc8[15:8]],
				rom_unorm8[kc8[7:0]]
			};
			for (ikc = 0; ikc < 8; ikc = ikc + 1)
				kcv_r[16 * ikc +: 16] <= kc_sel_buf[ikc] ? kcf_vec[16 * ikc +: 16] : kc8_unorm_r[16 * ikc +: 16];
		end
	end

	/* Pure-LUT helper functions (0 CARRY4s) */
	function [LINE_SHIFT-1:0] shift_left_psl;
		input [2:0] val;
		input [2:0] sh;
		case (sh)
		3'd0: shift_left_psl = {{(LINE_SHIFT-3){1'b0}}, val};
		3'd1: shift_left_psl = {{(LINE_SHIFT-4){1'b0}}, val, 1'b0};
		3'd2: shift_left_psl = {{(LINE_SHIFT-5){1'b0}}, val, 2'b00};
		3'd3: shift_left_psl = {val[LINE_SHIFT-4:0], 3'b000};
		default: shift_left_psl = {val[LINE_SHIFT-5:0], 4'b0000};
		endcase
	endfunction

	function [3:0] shift_left_el;
		input [1:0] val;
		input [2:0] sh;
		case (sh)
		3'd0: shift_left_el = {2'b00, val};
		3'd1: shift_left_el = {1'b0, val, 1'b0};
		default: shift_left_el = {val, 2'b00};
		endcase
	endfunction

	/* 2-stage pipelined slot gather (S_BLEND_PREP1 -> S_BLEND_PREP2 -> S_BLEND):
	 * Stage 1 latches slot_o_r, slot_o_rep, slot_en_r, slot_en_rep, slot_mk_r, slot_src_r.
	 * Stage 2 latches slot_dst128_r from i_line0/i_line1.
	 */
	reg [LINE_SHIFT-1:0] slot_o_r [0:3];
	(* keep = "true", dont_touch = "true" *) reg [LINE_SHIFT-1:0] slot_o_rep [0:3][0:7];
	(* keep = "true", dont_touch = "true" *) reg [3:0] slot_o_dst [0:3][0:9];
	(* keep = "true", dont_touch = "true" *) reg [3:0] slot_dst_en4, slot_dst_en4_b;
	(* keep = "true", dont_touch = "true" *) reg [3:0] slot_dst_en8, slot_dst_en8_b;
	(* keep = "true", dont_touch = "true" *) reg [3:0] slot_dst_en16, slot_dst_en16_b;
	reg slot_dst_en64_lo [0:3], slot_dst_en64_hi [0:3];
	reg [3:0] slot_en_r;
	(* keep = "true", dont_touch = "true" *) reg [7:0] slot_en_rep [0:3];
	reg [3:0] slot_mk_r;
	reg [127:0] slot_src_r [0:3];
	(* keep = "true", dont_touch = "true" *) reg [127:0] slot_dst128_r [0:3];
	(* keep = "true", dont_touch = "true" *) reg [63:0] slot_dst_fb [0:3];
	(* keep = "true", dont_touch = "true" *) reg [31:0] slot_dst32_out [0:3];

	wire [3:0] mk_comb;
	genvar k, g_rep;
	generate
	for (k = 0; k < 4; k = k + 1) begin : g_slot
		localparam [1:0] K2 = k[1:0];
		wire [1:0] i_g_k, i_j_k;
		wire [2:0] el_oh_k;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig0 (.I0(i_g[0]), .O(i_g_k[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ig1 (.I0(i_g[1]), .O(i_g_k[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ij0 (.I0(i_j[0]), .O(i_j_k[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ij1 (.I0(i_j[1]), .O(i_j_k[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eo0 (.I0(el_oh_r[0]), .O(el_oh_k[0]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eo1 (.I0(el_oh_r[1]), .O(el_oh_k[1]));
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_eo2 (.I0(el_oh_r[2]), .O(el_oh_k[2]));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign i_g_k[0] = i_g[0];
		assign i_g_k[1] = i_g[1];
		assign i_j_k[0] = i_j[0];
		assign i_j_k[1] = i_j[1];
		assign el_oh_k[0] = el_oh_r[0];
		assign el_oh_k[1] = el_oh_r[1];
		assign el_oh_k[2] = el_oh_r[2];
`else
		assign i_g_k = i_g;
		assign i_j_k = i_j;
		assign el_oh_k = el_oh_r;
`endif
		wire [1:0] nj_lo = el_oh_k[0] ? i_j_k : el_oh_k[1] ? {i_j_k[0], 1'b0} : 2'b00;
		/* Direct flat carry-out / bit-2 of (nj + K2) using pre-decoded 1-hot el_oh_r (<= 3 cmos3) */
		wire n_lo3_0 = nj_lo[0] ^ K2[0];
		wire n_lo3_1 = nj_lo[1] ^ K2[1] ^ (nj_lo[0] & K2[0]);
		wire n_lo3_2 = (el_oh_k[0] & ((K2[1] & (i_j_k[1] | (K2[0] & i_j_k[0]))) | (i_j_k[1] & K2[0] & i_j_k[0]))) |
		               (el_oh_k[1] & (i_j_k[1] ^ (K2[1] & i_j_k[0]))) |
		               (el_oh_k[2] & i_j_k[0]);
		wire [2:0] n_lo3 = {n_lo3_2, n_lo3_1, n_lo3_0};
		wire [1:0] ln = multi_slot[k] ? {i_r, n_lo3_2} : K2;
		/* 1-hot shifted o1s (<= 3 cmos3) + flat 6-bit CLA (<= 5 cmos3) */
		wire [2:0] o1s_val = ms4 ? {k[0], i_g_k} : {2'b00, k[0]};
		wire [4:0] sh_oh_k = psl_oh_slot[k];
		wire [LINE_SHIFT-1:0] o1s;
		assign o1s[0] = sh_oh_k[0] & o1s_val[0];
		assign o1s[1] = (sh_oh_k[0] & o1s_val[1]) | (sh_oh_k[1] & o1s_val[0]);
		assign o1s[2] = (sh_oh_k[0] & o1s_val[2]) | (sh_oh_k[1] & o1s_val[1]) | (sh_oh_k[2] & o1s_val[0]);
		assign o1s[3] = (sh_oh_k[1] & o1s_val[2]) | (sh_oh_k[2] & o1s_val[1]) | (sh_oh_k[3] & o1s_val[0]);
		assign o1s[4] = (sh_oh_k[2] & o1s_val[2]) | (sh_oh_k[3] & o1s_val[1]) | (sh_oh_k[4] & o1s_val[0]);
		if (LINE_SHIFT > 5) begin : g_o1s5
			assign o1s[5] = (sh_oh_k[3] & o1s_val[2]) | (sh_oh_k[4] & o1s_val[1]);
		end
		wire [LINE_SHIFT-1:0] off_base = (k >= 2) ? i_off1 : i_off0;
		wire [LINE_SHIFT-1:0] off_p1_base = (k >= 2) ? i_off1_p1 : i_off0_p1;
		wire [LINE_SHIFT-1:0] om_g = off_base & o1s;
		wire [LINE_SHIFT-1:0] om_prop = off_base | o1s;
		wire [LINE_SHIFT-1:0] om_p = off_base ^ o1s;
		wire [LINE_SHIFT-1:0] om_c;
		assign om_c[0] = 1'b0;
		assign om_c[1] = om_g[0];
		assign om_c[2] = om_g[1] | (om_prop[1] & om_g[0]);
		wire om_g3 = om_g[2] | (om_prop[2] & om_g[1]) | (om_prop[2] & om_prop[1] & om_g[0]);
		assign om_c[3] = om_g3;
		assign om_c[4] = om_g[3] | (om_prop[3] & om_g3);
		if (LINE_SHIFT > 5) begin : g_omc5
			assign om_c[5] = om_g[4] | (om_prop[4] & om_g[3]) | (om_prop[4] & om_prop[3] & om_g3);
		end
		wire [LINE_SHIFT-1:0] o1_ms = om_p ^ om_c;
		wire sel_psl_om = multi_slot[k];
		wire sel_ms4_om = !multi_slot[k] && ms4;
		wire sel_off_om = !multi_slot[k] && !ms4;
		wire [LINE_SHIFT-1:0] off_sel_base = k[0] ? off_p1_base : off_base;
		wire [LINE_SHIFT-1:0] o =
			(sel_psl_om ? psl_om_r[k] : {LINE_SHIFT{1'b0}}) |
			(sel_ms4_om ? o1_ms       : {LINE_SHIFT{1'b0}}) |
			(sel_off_om ? off_sel_base : {LINE_SHIFT{1'b0}});
		wire en_k = multi_slot[k] ? i_r : (k >= 2);
		/* Parallel pre-mux i_smask rows by n_lo3[1:0] while n_lo3_2 resolves (<= 9 cmos3) */
		wire [3:0] sm_col_k = {
			i_smask[{2'd3, n_lo3[1:0]}],
			i_smask[{2'd2, n_lo3[1:0]}],
			i_smask[{2'd1, n_lo3[1:0]}],
			i_smask[{2'd0, n_lo3[1:0]}]
		};
		wire mk_ms_k = (el_oh_k[2] || (k < 2)) && sm_col_k[ln];
		wire mk_k = multi_slot[k] ? mk_ms_k : i_smask[{ln, i_g_k}];
		assign mk_comb[k] = mk_k;

		/* Replicate ln into 8 copies (16 bits of i_color each) via LUT1 buffers so fanout <= 16 */
		wire [127:0] src_k_comb;
		for (g_rep = 0; g_rep < 8; g_rep = g_rep + 1) begin : g_src_mux
			wire [1:0] ln_b;
`ifdef SYNTH_KINTEX7
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ln0 (.I0(ln[0]), .O(ln_b[0]));
			(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_ln1 (.I0(ln[1]), .O(ln_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign ln_b[0] = ln[0];
			assign ln_b[1] = ln[1];
`else
			assign ln_b = ln;
`endif
			assign src_k_comb[16 * g_rep +: 16] = i_color[{ln_b, g_rep[2:0], 4'b0000} +: 16];
		end

		/* Pre-decoded dst_mask_c from Stage -1 config avoids psl_c decode on critical path */
		wire [3:0] o_dst_masked_k = o[3:0] & dst_mask_c;
		wire dst_en4_k  = dst_en4_c;
		wire dst_en8_k  = dst_en8_c;
		wire dst_en16_k = dst_en16_c;
		wire dst_en64_k = dst_en64_c;

		integer ir;
		always @(posedge wpix_act_clk or posedge rst_sync) begin
			if (rst_sync) begin
				slot_o_r[k] <= {LINE_SHIFT{1'b0}};
				for (ir = 0; ir < 8; ir = ir + 1)
					slot_o_rep[k][ir] <= {LINE_SHIFT{1'b0}};
				for (ir = 0; ir < 10; ir = ir + 1)
					slot_o_dst[k][ir] <= 4'd0;
				slot_dst_en4[k] <= 1'b1;
				slot_dst_en4_b[k] <= 1'b1;
				slot_dst_en8[k] <= 1'b1;
				slot_dst_en8_b[k] <= 1'b1;
				slot_dst_en16[k] <= 1'b0;
				slot_dst_en16_b[k] <= 1'b0;
				slot_dst_en64_lo[k] <= 1'b0;
				slot_dst_en64_hi[k] <= 1'b0;
				slot_en_r[k] <= 1'b0;
				slot_en_rep[k] <= 8'd0;
				slot_mk_r[k] <= 1'b0;
				slot_src_r[k] <= 128'd0;
			end else begin
				slot_o_r[k] <= o;
				for (ir = 0; ir < 8; ir = ir + 1)
					slot_o_rep[k][ir] <= o;
				for (ir = 0; ir < 10; ir = ir + 1)
					slot_o_dst[k][ir] <= o_dst_masked_k;
				slot_dst_en4[k] <= dst_en4_k;
				slot_dst_en4_b[k] <= dst_en4_k;
				slot_dst_en8[k] <= dst_en8_k;
				slot_dst_en8_b[k] <= dst_en8_k;
				slot_dst_en16[k] <= dst_en16_k;
				slot_dst_en16_b[k] <= dst_en16_k;
				slot_dst_en64_lo[k] <= dst_en64_k;
				slot_dst_en64_hi[k] <= dst_en64_k;
				slot_en_r[k] <= en_k;
				slot_en_rep[k] <= {8{en_k}};
				slot_mk_r[k] <= mk_k;
				slot_src_r[k] <= src_k_comb;
			end
		end

		/* Stage 2: extract 128-bit old pixel from i_line0 / i_line1 using replicated slot_o_rep and slot_en_rep */
		wire [127:0] blk;
		for (g_rep = 0; g_rep < 8; g_rep = g_rep + 1) begin : g_blk_mux
			wire [LINE_SHIFT-1:4] o_hi = slot_o_rep[k][g_rep][LINE_SHIFT-1:4];
			assign blk[16 * g_rep +: 16] = slot_en_rep[k][g_rep] ?
				i_line1[{o_hi, g_rep[2:0], 4'b0000} +: 16] :
				i_line0[{o_hi, g_rep[2:0], 4'b0000} +: 16];
		end

		wire unused_n = &{1'b0, n_lo3[2], psl_slot[k],
			slot_o_dst[k][4][0], slot_o_dst[k][5][0], slot_o_dst[k][6][1:0], slot_o_dst[k][7][1:0],
			slot_o_dst[k][8][2:0], slot_o_dst[k][9][2:0],
			slot_o_rep[k][0][3:0], slot_o_rep[k][1][3:0], slot_o_rep[k][2][3:0], slot_o_rep[k][3][3:0],
			slot_o_rep[k][4][3:0], slot_o_rep[k][5][3:0], slot_o_rep[k][6][3:0], slot_o_rep[k][7][3:0]};
		wire [127:0] dst_k_comb;
		assign dst_k_comb[1:0]   = blk[{slot_o_dst[k][0][3:0], 3'b000} +: 2];
		assign dst_k_comb[3:2]   = blk[{slot_o_dst[k][1][3:0], 3'b010} +: 2];
		assign dst_k_comb[5:4]   = blk[{slot_o_dst[k][2][3:0], 3'b100} +: 2];
		assign dst_k_comb[7:6]   = blk[{slot_o_dst[k][3][3:0], 3'b110} +: 2];
		assign dst_k_comb[11:8]  = slot_dst_en4[k]    ? blk[{slot_o_dst[k][4][3:1], 4'b1000} +: 4] : 4'd0;
		assign dst_k_comb[15:12] = slot_dst_en4_b[k]  ? blk[{slot_o_dst[k][5][3:1], 4'b1100} +: 4] : 4'd0;
		assign dst_k_comb[23:16] = slot_dst_en8[k]    ? blk[{slot_o_dst[k][6][3:2], 5'b1_0000} +: 8] : 8'd0;
		assign dst_k_comb[31:24] = slot_dst_en8_b[k]  ? blk[{slot_o_dst[k][7][3:2], 5'b1_1000} +: 8] : 8'd0;
		assign dst_k_comb[47:32] = slot_dst_en16[k]   ? blk[{slot_o_dst[k][8][3],   6'b10_0000} +: 16] : 16'd0;
		assign dst_k_comb[63:48] = slot_dst_en16_b[k] ? blk[{slot_o_dst[k][9][3],   6'b11_0000} +: 16] : 16'd0;
`ifdef SYNTH_KINTEX7
		wire dst_rst_64, dst_rst_80, dst_rst_96, dst_rst_112;
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr64  (.I0(rst_sync), .I1(slot_dst_en64_lo[k]), .O(dst_rst_64));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr80  (.I0(rst_sync), .I1(slot_dst_en64_lo[k]), .O(dst_rst_80));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr96  (.I0(rst_sync), .I1(slot_dst_en64_hi[k]), .O(dst_rst_96));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr112 (.I0(rst_sync), .I1(slot_dst_en64_hi[k]), .O(dst_rst_112));
		assign dst_k_comb[127:64] = blk[127:64];

		always @(posedge wpix_act_clk) begin
			if (rst_sync) begin
				slot_dst128_r[k][63:0] <= 64'd0;
				slot_dst_fb[k]         <= 64'd0;
				slot_dst32_out[k]      <= 32'd0;
			end else begin
				slot_dst128_r[k][63:0] <= dst_k_comb[63:0];
				slot_dst_fb[k]         <= dst_k_comb[63:0];
				slot_dst32_out[k]      <= dst_k_comb[31:0];
			end

			if (dst_rst_64)
				slot_dst128_r[k][79:64] <= 16'd0;
			else
				slot_dst128_r[k][79:64] <= dst_k_comb[79:64];

			if (dst_rst_80)
				slot_dst128_r[k][95:80] <= 16'd0;
			else
				slot_dst128_r[k][95:80] <= dst_k_comb[95:80];

			if (dst_rst_96)
				slot_dst128_r[k][111:96] <= 16'd0;
			else
				slot_dst128_r[k][111:96] <= dst_k_comb[111:96];

			if (dst_rst_112)
				slot_dst128_r[k][127:112] <= 16'd0;
			else
				slot_dst128_r[k][127:112] <= dst_k_comb[127:112];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		wire dst_rst_64, dst_rst_80, dst_rst_96, dst_rst_112;
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr64  (.I0(rst_sync), .I1(slot_dst_en64_lo[k]), .O(dst_rst_64));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr80  (.I0(rst_sync), .I1(slot_dst_en64_lo[k]), .O(dst_rst_80));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr96  (.I0(rst_sync), .I1(slot_dst_en64_hi[k]), .O(dst_rst_96));
		(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'hB)) u_dr112 (.I0(rst_sync), .I1(slot_dst_en64_hi[k]), .O(dst_rst_112));
		assign dst_k_comb[127:64] = blk[127:64];

		always @(posedge wpix_act_clk) begin
			if (rst_sync) begin
				slot_dst128_r[k][63:0] <= 64'd0;
				slot_dst_fb[k]         <= 64'd0;
				slot_dst32_out[k]      <= 32'd0;
			end else begin
				slot_dst128_r[k][63:0] <= dst_k_comb[63:0];
				slot_dst_fb[k]         <= dst_k_comb[63:0];
				slot_dst32_out[k]      <= dst_k_comb[31:0];
			end

			if (dst_rst_64)
				slot_dst128_r[k][79:64] <= 16'd0;
			else
				slot_dst128_r[k][79:64] <= dst_k_comb[79:64];

			if (dst_rst_80)
				slot_dst128_r[k][95:80] <= 16'd0;
			else
				slot_dst128_r[k][95:80] <= dst_k_comb[95:80];

			if (dst_rst_96)
				slot_dst128_r[k][111:96] <= 16'd0;
			else
				slot_dst128_r[k][111:96] <= dst_k_comb[111:96];

			if (dst_rst_112)
				slot_dst128_r[k][127:112] <= 16'd0;
			else
				slot_dst128_r[k][127:112] <= dst_k_comb[127:112];
		end
`else
		assign dst_k_comb[127:64] = blk[127:64];

		always @(posedge wpix_act_clk or posedge rst_sync) begin
			if (rst_sync) begin
				slot_dst128_r[k] <= 128'd0;
				slot_dst_fb[k] <= 64'd0;
				slot_dst32_out[k] <= 32'd0;
			end else begin
				slot_dst128_r[k][63:0]   <= dst_k_comb[63:0];
				slot_dst_fb[k]           <= dst_k_comb[63:0];
				slot_dst32_out[k]        <= dst_k_comb[31:0];
				slot_dst128_r[k][79:64]  <= slot_dst_en64_lo[k] ? dst_k_comb[79:64]  : 16'd0;
				slot_dst128_r[k][95:80]  <= slot_dst_en64_lo[k] ? dst_k_comb[95:80]  : 16'd0;
				slot_dst128_r[k][111:96] <= slot_dst_en64_hi[k] ? dst_k_comb[111:96] : 16'd0;
				slot_dst128_r[k][127:112] <= slot_dst_en64_hi[k] ? dst_k_comb[127:112] : 16'd0;
			end
		end
`endif
	end
	endgenerate

	reg slot_mk_nz_r;
	always @(posedge wpix_act_clk or posedge rst_sync) begin
		if (rst_sync) slot_mk_nz_r <= 1'b0;
		else slot_mk_nz_r <= (mk_comb != 4'd0);
	end

	assign o_mask = slot_mk_r;
	assign o_lw = multi_slot[0] ? slot_mk_nz_r : smask_lw_r;
	assign o_ent = slot_en_r;
	assign o_soff = {slot_o_r[3], slot_o_r[2], slot_o_r[1], slot_o_r[0]};
	assign o_bsrc = {slot_src_r[3], slot_src_r[2], slot_src_r[1], slot_src_r[0]};
	assign o_bdst = {slot_dst32_out[3], slot_dst32_out[2], slot_dst32_out[1], slot_dst32_out[0]};

	wire [255:0] f_dst = {slot_dst_fb[3], slot_dst_fb[2], slot_dst_fb[1], slot_dst_fb[0]};
	wire [511:0] f_src = o_bsrc;
	wire [255:0] f_dst_in = f_dst;
	wire [3:0] f_mk = slot_mk_r;
	wire [255:0] f_raw, f_keep;
	wire f_idle, fb_o_valid;
	reg f_done_r;
	wire fb_o_ready = !f_done_r || f_take;
	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync)
			f_done_r <= 1'b0;
		else if (fb_o_ready)
			f_done_r <= fb_o_valid;
	end
	assign f_done = f_done_r;

	wire fb_lat_w = fb_o_valid && fb_o_ready;
	wire [15:0] fb_lat_leaf;
	genvar g_fbl;
	generate
`ifdef SYNTH_KINTEX7
	for (g_fbl = 0; g_fbl < 16; g_fbl = g_fbl + 1) begin : g_fb_lat
		(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_fbl (.I0(fb_lat_w), .O(fb_lat_leaf[g_fbl]));
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	for (g_fbl = 0; g_fbl < 16; g_fbl = g_fbl + 1) begin : g_fb_lat
		assign fb_lat_leaf[g_fbl] = fb_lat_w;
	end
`else
	assign fb_lat_leaf = {16{fb_lat_w}};
`endif
	endgenerate

	generate
	if (ENABLE_FBLEND != 0) begin : g_fblend
`ifdef SYNTH_KINTEX7
		wire fblend_clk = clk;
`elsif SYNTH_KINTEX_ULTRASCALE
		wire fblend_clk = clk;
`else
		reg [4:0] f_sim_cnt;
		wire [4:0] f_sim_cnt_next = {
			f_sim_cnt[4] ^ (f_sim_cnt[3:0] == 4'd0),
			f_sim_cnt[3] ^ (f_sim_cnt[2:0] == 3'd0),
			f_sim_cnt[2] ^ (f_sim_cnt[1:0] == 2'd0),
			f_sim_cnt[1] ^ ~f_sim_cnt[0],
			~f_sim_cnt[0]
		};
		always @(posedge clk or negedge rst_n) begin
			if (!rst_n) begin
				f_sim_cnt <= 5'd3;
			end else if (f_go) begin
				f_sim_cnt <= 5'd31;
			end else if (f_sim_cnt != 5'd0) begin
				f_sim_cnt <= f_sim_cnt_next;
			end
		end
		reg f_icg_en;
		always @(negedge clk or negedge rst_n) begin
			if (!rst_n)
				f_icg_en <= 1'b1;
			else
				f_icg_en <= f_go || (f_sim_cnt != 5'd0);
		end
		wire fblend_clk = clk & f_icg_en;
`endif
		atg_rop_fblend fblend (
			.clk(fblend_clk), .rst_n(rst_n),
			.i_valid(f_go), .i_ready(f_idle),
			.i_lanes(f_mk), .i_src(f_src), .i_dst(f_dst_in),
			.i_fmt(vk), .i_blend(bl), .i_const(kcv_r), .i_wm(wm_a),
			.o_valid(fb_o_valid), .o_ready(fb_o_ready),
			.o_raw(f_raw), .o_keep(f_keep)
		);
	end else begin : g_no_fblend
		assign fb_o_valid = f_go;
		assign f_idle = 1'b1;
		assign f_raw = 256'd0;
		assign f_keep = 256'd0;
	end
	endgenerate

	/* Pipelined Path 2 format packer (24 integer and 32-bit float formats, <= 2 LUT levels per stage) */
	localparam integer N_P2 = 24;
	localparam [191:0] P2F = {8'd109, 8'd108, 8'd107, 8'd103, 8'd102, 8'd101, 8'd100, 8'd99, 8'd98,
		8'd96, 8'd95, 8'd82, 8'd81, 8'd75, 8'd74, 8'd68, 8'd56, 8'd55, 8'd42, 8'd41, 8'd21, 8'd20,
		8'd14, 8'd13};

	function [192*N_P2-1:0] ptab_p2;
		input integer unused_n;
		integer pi, pk, pb;
		reg [67:0] inf;
		reg [25:0] ft;
		reg [5:0] w;
		reg [31:0] m;
		begin
			ptab_p2 = {192*N_P2{1'b0}};
			for (pi = 0; pi < N_P2; pi = pi + 1) begin
				inf = atg_fmt_info(P2F[8 * pi +: 8]);
				ft = atg_fmt_feat(P2F[8 * pi +: 8], 1'b0);
				for (pk = 0; pk < 4; pk = pk + 1) begin
					w = inf[23 + 13 * pk +: 6];
					for (pb = 0; pb < 32; pb = pb + 1)
						m[pb] = pb < w;
					ptab_p2[48 * (4 * pi + pk) +: 48] = {6'd0, ft[7] && pk < inf[8:6], 2'd0, m,
						inf[16 + 13 * pk +: 7]};
				end
			end
		end
	endfunction
	localparam [192*N_P2-1:0] PTAB_P2 = ptab_p2(N_P2);

	/* Stage 1 of Path-2 packer: decode format hit & write-mask per format (replicated per lane & even/odd 16-bit slice) */
	(* keep = "true", dont_touch = "true" *) reg [3:0] p2_hit_wm_wr_b0 [0:N_P2-1];
	(* keep = "true", dont_touch = "true" *) reg [3:0] p2_hit_wm_wr_b1 [0:N_P2-1];
	(* keep = "true", dont_touch = "true" *) reg [3:0] p2_hit_wm_l_b0 [0:3][0:N_P2-1];
	(* keep = "true", dont_touch = "true" *) reg [3:0] p2_hit_wm_l_b1 [0:3][0:N_P2-1];
	(* keep = "true", dont_touch = "true" *) reg [N_P2-1:0] p2_hit_v;
	genvar gi, gg, gc_p2;
	generate
	for (gi = 0; gi < N_P2; gi = gi + 1) begin : g_p2_dec
		localparam [7:0] F = P2F[8 * gi +: 8];
		wire [7:0] vk_cur = (gi < 12) ? vk_pack_a : vk_pack_b;
		wire [3:0] wm_cur = (gi < 12) ? wm_a : wm_b;
		wire hit_i = (vk_cur == F);
		wire [3:0] hit_wm_i;
`ifdef SYNTH_KINTEX7
		for (gc_p2 = 0; gc_p2 < 4; gc_p2 = gc_p2 + 1) begin : g_hwm
			(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_hwm (
				.I0(hit_i), .I1(wm_cur[gc_p2]), .O(hit_wm_i[gc_p2])
			);
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (gc_p2 = 0; gc_p2 < 4; gc_p2 = gc_p2 + 1) begin : g_hwm
			(* keep = "true", dont_touch = "true" *) LUT2 #(.INIT(4'h8)) u_hwm (
				.I0(hit_i), .I1(wm_cur[gc_p2]), .O(hit_wm_i[gc_p2])
			);
		end
`else
		assign hit_wm_i = hit_i ? wm_cur : 4'd0;
`endif
		integer il_p2;
		always @(posedge wpix_p2_clk) begin
			p2_hit_wm_wr_b0[gi] <= hit_wm_i;
			p2_hit_wm_wr_b1[gi] <= hit_wm_i;
			for (il_p2 = 0; il_p2 < 4; il_p2 = il_p2 + 1) begin
				p2_hit_wm_l_b0[il_p2][gi] <= hit_wm_i;
				p2_hit_wm_l_b1[il_p2][gi] <= hit_wm_i;
			end
			p2_hit_v[gi] <= hit_i;
		end
	end
	endgenerate

	/* Shared write-mask expansion across the 24 formats (2 stages: 4 groups of 6 -> final 128-bit wr) */
	wire [127:0] p2_wr_leaf [0:N_P2-1];
	generate
	for (gi = 0; gi < N_P2; gi = gi + 1) begin : g_p2_wr_leaf
		localparam [47:0] E0 = PTAB_P2[48 * (4 * gi) +: 48];
		localparam [47:0] E1 = PTAB_P2[48 * (4 * gi + 1) +: 48];
		localparam [47:0] E2 = PTAB_P2[48 * (4 * gi + 2) +: 48];
		localparam [47:0] E3 = PTAB_P2[48 * (4 * gi + 3) +: 48];
		localparam [6:0] P0 = E0[6:0], P1 = E1[6:0], P2 = E2[6:0], P3 = E3[6:0];
		localparam [127:0] M0 = E0[41] ? ({96'd0, E0[38:7]} << P0) : 128'd0;
		localparam [127:0] M1 = E1[41] ? ({96'd0, E1[38:7]} << P1) : 128'd0;
		localparam [127:0] M2 = E2[41] ? ({96'd0, E2[38:7]} << P2) : 128'd0;
		localparam [127:0] M3 = E3[41] ? ({96'd0, E3[38:7]} << P3) : 128'd0;
		for (gg = 0; gg < 4; gg = gg + 1) begin : g_wr_q
			assign p2_wr_leaf[gi][32 * gg +: 16] =
				(p2_hit_wm_wr_b0[gi][0] ? M0[32 * gg +: 16] : 16'd0) |
				(p2_hit_wm_wr_b0[gi][1] ? M1[32 * gg +: 16] : 16'd0) |
				(p2_hit_wm_wr_b0[gi][2] ? M2[32 * gg +: 16] : 16'd0) |
				(p2_hit_wm_wr_b0[gi][3] ? M3[32 * gg +: 16] : 16'd0);
			assign p2_wr_leaf[gi][32 * gg + 16 +: 16] =
				(p2_hit_wm_wr_b1[gi][0] ? M0[32 * gg + 16 +: 16] : 16'd0) |
				(p2_hit_wm_wr_b1[gi][1] ? M1[32 * gg + 16 +: 16] : 16'd0) |
				(p2_hit_wm_wr_b1[gi][2] ? M2[32 * gg + 16 +: 16] : 16'd0) |
				(p2_hit_wm_wr_b1[gi][3] ? M3[32 * gg + 16 +: 16] : 16'd0);
		end
	end
	endgenerate

	(* keep = "true", dont_touch = "true" *) reg [127:0] p2_wr_grp [0:3];
	(* keep = "true", dont_touch = "true" *) reg [3:0] p2_hit_grp;
	(* keep = "true", dont_touch = "true" *) reg [127:0] p2_wr_r [0:3];
	(* keep = "true", dont_touch = "true" *) reg [7:0] p2_hit_rep [0:3];
	integer ig;
	always @(posedge wpix_p2_clk) begin
		for (ig = 0; ig < 4; ig = ig + 1) begin
			p2_wr_grp[ig] <= p2_wr_leaf[6*ig + 0] | p2_wr_leaf[6*ig + 1] | p2_wr_leaf[6*ig + 2] |
				p2_wr_leaf[6*ig + 3] | p2_wr_leaf[6*ig + 4] | p2_wr_leaf[6*ig + 5];
			p2_hit_grp[ig] <= |p2_hit_v[6*ig +: 6];
		end
	end

	generate
	for (k = 0; k < 4; k = k + 1) begin : g_res
		wire hit_any_k;
		wire [127:0] wr_or_k;
`ifdef SYNTH_KINTEX7
		(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_hany (
			.I0(p2_hit_grp[0]), .I1(p2_hit_grp[1]), .I2(p2_hit_grp[2]), .I3(p2_hit_grp[3]),
			.O(hit_any_k)
		);
		for (gg = 0; gg < 128; gg = gg + 1) begin : g_wror
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_wo (
				.I0(p2_wr_grp[0][gg]), .I1(p2_wr_grp[1][gg]), .I2(p2_wr_grp[2][gg]), .I3(p2_wr_grp[3][gg]),
				.O(wr_or_k[gg])
			);
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_hany (
			.I0(p2_hit_grp[0]), .I1(p2_hit_grp[1]), .I2(p2_hit_grp[2]), .I3(p2_hit_grp[3]),
			.O(hit_any_k)
		);
		for (gg = 0; gg < 128; gg = gg + 1) begin : g_wror
			(* keep = "true", dont_touch = "true" *) LUT4 #(.INIT(16'hFFFE)) u_wo (
				.I0(p2_wr_grp[0][gg]), .I1(p2_wr_grp[1][gg]), .I2(p2_wr_grp[2][gg]), .I3(p2_wr_grp[3][gg]),
				.O(wr_or_k[gg])
			);
		end
`else
		assign hit_any_k = |p2_hit_grp;
		assign wr_or_k = p2_wr_grp[0] | p2_wr_grp[1] | p2_wr_grp[2] | p2_wr_grp[3];
`endif
		always @(posedge wpix_p2_clk) begin
			p2_hit_rep[k] <= {8{hit_any_k}};
			p2_wr_r[k] <= wr_or_k;
		end

		(* keep = "true", dont_touch = "true" *) reg [127:0] pk_raw_r;
		(* keep = "true", dont_touch = "true" *) reg [127:0] pk_merged_r;
		if (ENABLE_PACK != 0) begin : g_pk
`ifdef SYNTH_KINTEX7
			wire [127:0] c_k = slot_src_r[k];
`elsif SYNTH_KINTEX_ULTRASCALE
			wire [127:0] c_k = slot_src_r[k];
`else
			wire [127:0] c_k = (path == 2'd2) ? slot_src_r[k] : 128'd0;
`endif
			wire [127:0] raw_leaf [0:N_P2-1];
			for (gi = 0; gi < N_P2; gi = gi + 1) begin : g_raw_leaf
				localparam [47:0] E0 = PTAB_P2[48 * (4 * gi) +: 48];
				localparam [47:0] E1 = PTAB_P2[48 * (4 * gi + 1) +: 48];
				localparam [47:0] E2 = PTAB_P2[48 * (4 * gi + 2) +: 48];
				localparam [47:0] E3 = PTAB_P2[48 * (4 * gi + 3) +: 48];
				localparam [6:0] P0 = E0[6:0], P1 = E1[6:0], P2 = E2[6:0], P3 = E3[6:0];
				localparam [127:0] S0_MASK = {96'd0, E0[38:7]} << P0;
				localparam [127:0] S1_MASK = {96'd0, E1[38:7]} << P1;
				localparam [127:0] S2_MASK = {96'd0, E2[38:7]} << P2;
				localparam [127:0] S3_MASK = {96'd0, E3[38:7]} << P3;
				wire [127:0] s0_sh = ({96'd0, c_k[31:0]}   << P0) & S0_MASK;
				wire [127:0] s1_sh = ({96'd0, c_k[63:32]}  << P1) & S1_MASK;
				wire [127:0] s2_sh = ({96'd0, c_k[95:64]}  << P2) & S2_MASK;
				wire [127:0] s3_sh = ({96'd0, c_k[127:96]} << P3) & S3_MASK;
				for (gg = 0; gg < 4; gg = gg + 1) begin : g_raw_q
					wire [15:0] r0_b0 = (E0[41] && p2_hit_wm_l_b0[k][gi][0]) ? s0_sh[32 * gg +: 16] : 16'd0;
					wire [15:0] r1_b0 = (E1[41] && p2_hit_wm_l_b0[k][gi][1]) ? s1_sh[32 * gg +: 16] : 16'd0;
					wire [15:0] r2_b0 = (E2[41] && p2_hit_wm_l_b0[k][gi][2]) ? s2_sh[32 * gg +: 16] : 16'd0;
					wire [15:0] r3_b0 = (E3[41] && p2_hit_wm_l_b0[k][gi][3]) ? s3_sh[32 * gg +: 16] : 16'd0;
					wire [15:0] r0_b1 = (E0[41] && p2_hit_wm_l_b1[k][gi][0]) ? s0_sh[32 * gg + 16 +: 16] : 16'd0;
					wire [15:0] r1_b1 = (E1[41] && p2_hit_wm_l_b1[k][gi][1]) ? s1_sh[32 * gg + 16 +: 16] : 16'd0;
					wire [15:0] r2_b1 = (E2[41] && p2_hit_wm_l_b1[k][gi][2]) ? s2_sh[32 * gg + 16 +: 16] : 16'd0;
					wire [15:0] r3_b1 = (E3[41] && p2_hit_wm_l_b1[k][gi][3]) ? s3_sh[32 * gg + 16 +: 16] : 16'd0;
					assign raw_leaf[gi][32 * gg +: 16]      = r0_b0 | r1_b0 | r2_b0 | r3_b0;
					assign raw_leaf[gi][32 * gg + 16 +: 16] = r0_b1 | r1_b1 | r2_b1 | r3_b1;
				end
			end

			(* keep = "true", dont_touch = "true" *) reg [127:0] raw_grp [0:3];
			(* keep = "true", dont_touch = "true" *) wire [127:0] pk_m_comb;
			for (gg = 0; gg < 8; gg = gg + 1) begin : g_pkm_comb
`ifdef MUT_WPIX_KEEP
				assign pk_m_comb[16 * gg +: 16] = pk_raw_r[16 * gg +: 16];
`else
				assign pk_m_comb[16 * gg +: 16] = p2_hit_rep[k][gg] ?
					((pk_raw_r[16 * gg +: 16] & p2_wr_r[k][16 * gg +: 16]) |
					 (slot_dst128_r[k][16 * gg +: 16] & ~p2_wr_r[k][16 * gg +: 16])) :
					16'd0;
`endif
			end
			(* keep = "true", dont_touch = "true" *) wire [63:0] pk_w_lo;
`ifdef SYNTH_KINTEX7
			wire [15:0] pk_m_b1;
			for (gg = 0; gg < 16; gg = gg + 1) begin : g_pk_mb
				(* keep = "true", dont_touch = "true" *) LUT1 #(.INIT(2'h2)) u_pkmb (.I0(pk_m_comb[gg]), .O(pk_m_b1[gg]));
			end
			for (gg = 0; gg < 8; gg = gg + 1) begin : g_pkm_wlo
				wire [1:0] psl_lo_k = psl_pk_lo[k][gg / 2];
				wire [7:0] pk_b0_src = (gg >= 4) ? pk_m_b1[7:0] : pk_m_comb[7:0];
				wire [7:0] pk_b2_src = ((gg >= 4) && ((gg % 4) < 2)) ?
					pk_m_b1[8 * (gg % 2) +: 8] :
					pk_m_comb[8 * (gg % 4) +: 8];
				wire [7:0] pk_b1_src = ((gg % 2) == 0) ? pk_b0_src :
					((gg >= 4) ? pk_m_b1[15:8] : pk_m_comb[15:8]);
				assign pk_w_lo[8 * gg +: 8] =
					(psl_lo_k == 2'd3) ? pk_m_comb[8 * gg +: 8] :
					(psl_lo_k == 2'd2) ? pk_b2_src :
					(psl_lo_k == 2'd1) ? pk_b1_src :
					                     pk_b0_src;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			wire [15:0] pk_m_b1;
			for (gg = 0; gg < 16; gg = gg + 1) begin : g_pk_mb
				assign pk_m_b1[gg] = pk_m_comb[gg];
			end
			for (gg = 0; gg < 8; gg = gg + 1) begin : g_pkm_wlo
				wire [1:0] psl_lo_k = psl_pk_lo[k][gg / 2];
				wire [7:0] pk_b0_src = (gg >= 4) ? pk_m_b1[7:0] : pk_m_comb[7:0];
				wire [7:0] pk_b2_src = ((gg >= 4) && ((gg % 4) < 2)) ?
					pk_m_b1[8 * (gg % 2) +: 8] :
					pk_m_comb[8 * (gg % 4) +: 8];
				wire [7:0] pk_b1_src = ((gg % 2) == 0) ? pk_b0_src :
					((gg >= 4) ? pk_m_b1[15:8] : pk_m_comb[15:8]);
				assign pk_w_lo[8 * gg +: 8] =
					(psl_lo_k == 2'd3) ? pk_m_comb[8 * gg +: 8] :
					(psl_lo_k == 2'd2) ? pk_b2_src :
					(psl_lo_k == 2'd1) ? pk_b1_src :
					                     pk_b0_src;
			end
`else
			wire [1:0] psl_lo_k = psl_pk_lo[k][0];
			assign pk_w_lo =
				(psl_lo_k == 2'd3) ? pk_m_comb[63:0] :
				(psl_lo_k == 2'd2) ? {2{pk_m_comb[31:0]}} :
				(psl_lo_k == 2'd1) ? {4{pk_m_comb[15:0]}} :
				                     {8{pk_m_comb[7:0]}};
`endif
			integer rg;
			always @(posedge wpix_p2_clk) begin
				for (rg = 0; rg < 4; rg = rg + 1)
					raw_grp[rg] <= raw_leaf[6*rg + 0] | raw_leaf[6*rg + 1] | raw_leaf[6*rg + 2] |
						raw_leaf[6*rg + 3] | raw_leaf[6*rg + 4] | raw_leaf[6*rg + 5];
				pk_raw_r <= raw_grp[0] | raw_grp[1] | raw_grp[2] | raw_grp[3];
				pk_merged_r[63:0] <= pk_w_lo;
`ifdef SYNTH_KINTEX7
				for (rg = 0; rg < 4; rg = rg + 1) begin
					pk_merged_r[64 + 16 * rg +: 16] <= psl_pk_is16[k][rg] ?
						pk_m_comb[64 + 16 * rg +: 16] : pk_w_lo[16 * rg +: 16];
				end
`elsif SYNTH_KINTEX_ULTRASCALE
				for (rg = 0; rg < 4; rg = rg + 1) begin
					pk_merged_r[64 + 16 * rg +: 16] <= psl_pk_is16[k][rg] ?
						pk_m_comb[64 + 16 * rg +: 16] : pk_w_lo[16 * rg +: 16];
				end
`else
				pk_merged_r[127:64] <= psl_pk_is16[k][0] ? pk_m_comb[127:64] : pk_w_lo;
`endif
			end
		end else begin : g_no_pk
			always @(posedge clk) begin
				pk_raw_r <= 128'd0;
				pk_merged_r <= 128'd0;
			end
		end

		reg [63:0] fr;
		integer i_fr;
		always @(posedge clk) begin
			for (i_fr = 0; i_fr < 4; i_fr = i_fr + 1) begin
				if (fb_lat_leaf[4 * k + i_fr])
					fr[16 * i_fr +: 16] <= (f_raw[64 * k + 16 * i_fr +: 16] & ~f_keep[64 * k + 16 * i_fr +: 16]) |
						(f_dst[64 * k + 16 * i_fr +: 16] & f_keep[64 * k + 16 * i_fr +: 16]);
			end
		end
		wire [127:0] pk_out = (ENABLE_PACK != 0) ? pk_merged_r : 128'd0;
`ifdef SYNTH_KINTEX7
		for (g_rep = 0; g_rep < 16; g_rep = g_rep + 1) begin : g_res_mux
			wire [1:0] f32_sel = f32_ps_sel_r[k][g_rep / 2];
			assign o_res[128 * k + 8 * g_rep +: 8] =
				(f32_sel == 2'd3) ? fr[8 * (g_rep % 8) +: 8] :
				(f32_sel == 2'd2) ? fr[8 * (g_rep % 4) +: 8] :
				(f32_sel == 2'd1) ? fr[8 * (g_rep % 2) +: 8] :
				                    pk_out[8 * g_rep +: 8];
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		for (g_rep = 0; g_rep < 16; g_rep = g_rep + 1) begin : g_res_mux
			wire [1:0] f32_sel = f32_ps_sel_r[k][g_rep / 2];
			assign o_res[128 * k + 8 * g_rep +: 8] =
				(f32_sel == 2'd3) ? fr[8 * (g_rep % 8) +: 8] :
				(f32_sel == 2'd2) ? fr[8 * (g_rep % 4) +: 8] :
				(f32_sel == 2'd1) ? fr[8 * (g_rep % 2) +: 8] :
				                    pk_out[8 * g_rep +: 8];
		end
`else
		wire [1:0] f32_sel = f32_ps_sel_r[k][0];
		assign o_res[128 * k +: 128] =
			(f32_sel == 2'd3) ? {2{fr}} :
			(f32_sel == 2'd2) ? {4{fr[31:0]}} :
			(f32_sel == 2'd1) ? {8{fr[15:0]}} :
			                    pk_out;
`endif
	end
	endgenerate

	localparam [31:0] LS32 = LINE_SHIFT;
	wire unused_ok = &{1'b0, f_idle, unused_yrs_hi, LS32[31:4], a1_co_unused, bl[22:1], i_idle,
		b_pix, i_reg_off0, i_reg_off1, i_reg_rt, i_reg_color, i_reg_mask, mk_comb};
endmodule

`default_nettype wire
