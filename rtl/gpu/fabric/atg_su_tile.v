/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_tile: Single Shader Unit (SU) physical tile with integrated
 * 2-way local NoC router (atg_rt2) merging D$ (c0) and I$ (c1, EXEC=1)
 * onto one unified NoC port (docs/noc_and_mmu_architecture.md §5.1).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_tile #(
	parameter NPHYS = 8,
	parameter NWARPS = 4,
	parameter WARP_W = 2,
	parameter GRP_W = 2,
	parameter NREGS = 32,
	parameter SB_ENTRIES = 4,
	parameter SBE_W = 2,
	parameter NSU = 1,
	parameter SU_INDEX = 0,
	parameter FRAG_RESERVE = 1,
	parameter PACKED = 0,
	parameter PK_SHARED = 0,
	parameter SUBGROUP = 0,
	parameter HAS_SA = 0,
	parameter MS4 = 0,
	parameter HAS_MESH = 0,
	parameter LDS_BYTES = 4096,
	parameter PT_ENTRIES = 64,
	parameter VARY_WORDS = 16,
	parameter ADDR_W = 32,
	parameter LINE_BYTES = 32,
	parameter LINE_SHIFT = 5,
	parameter MRT_COUNT = 1,
	parameter DUAL_SOURCE = 0,
	parameter SHARED_PT = 0,
	parameter HAS_GFX = 1,
	parameter HAS_TU = 1,
	parameter SU_RW = 1,
	parameter [SU_RW-1:0] ROUTE_D = {SU_RW{1'b0}},
	parameter [SU_RW-1:0] ROUTE_I = {SU_RW{1'b0}},
	parameter RBIT_SU = 10,
	parameter FW = LINE_BYTES * 8,
	parameter RFW = LINE_BYTES * 8 + 10 + SU_RW,
	parameter PT_W = 1 + 3 + GRP_W + 7 + NPHYS + NPHYS * 32
) (
	input wire clk,
	input wire rst_n,
	input wire flush,

	/* Control broadcast (from atg_rep / Spine) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	input wire cs_kick,
	input wire cs_mesh,
	input wire lc_kick,
	input wire [3:0] inv,
	input wire scrub,
	input wire [3:0] session_id,

	/* Vertex warps */
	input wire vl_req,
	output wire vl_gnt,
	output wire [7:0] vl_ptbase,
	output wire vl_avail,
	input wire vl_we,
	input wire [5:0] vl_reg,
	input wire [4:0] vl_lane,
	input wire [31:0] vl_data,
	input wire vl_go,
	input wire [31:0] vl_mask,
	input wire [7:0] vl_go_ptbase,

	/* Fragment warps */
	input wire fl_req,
	output wire fl_gnt,
	output wire fl_avail,
	input wire fl_we,
	input wire [5:0] fl_reg,
	input wire [4:0] fl_lane,
	input wire [31:0] fl_data,
	input wire fl_pw_we,
	input wire [11:0] fl_pw_addr,
	input wire [31:0] fl_pw_data,
	input wire fl_go,
	input wire [31:0] fl_mask,
	input wire [63:0] fl_info,

	/* Local PT (SHARED_PT == 0) */
	input wire pt_rd,
	input wire [7:0] pt_idx,
	input wire [6:0] pt_word,
	output wire [31:0] pt_rdata,
	input wire pt_free,
	input wire [7:0] pt_free_idx,
	output wire [7:0] pt_done,

	/* Shared Spine PT export (SHARED_PT != 0) */
	output wire pt_exp_valid,
	input wire pt_exp_ready,
	output wire [PT_W-1:0] pt_exp_packed,

	/* TU */
	output wire tx_valid,
	input wire tx_ready,
	output wire [7:0] tx_op,
	output wire [3:0] tx_slot,
	output wire [3:0] tx_samp_slot,
	output wire tx_vs,
	output wire [3:0] tx_mask,
	output wire [639:0] tx_coord,
	output wire [11:0] tx_off,
	output wire [15:0] tx_tag,
	output wire tx_last,
	input wire tr_valid,
	output wire tr_ready,
	input wire [15:0] tr_tag,
	input wire [511:0] tr_data,
	input wire tr_last,

	/* ROP */
	output wire ex_valid,
	input wire ex_ready,
	output wire [15:0] ex_seq,
	output wire [31:0] ex_xy,
	output wire [3:0] ex_mask,
	output wire [511:0] ex_color,
	output wire [511:0] ex_color1,
	output wire ex_has_src1,
	output wire [127:0] ex_depth,
	output wire ex_last,
	input wire [15:0] ex_next,
	output wire [154:0] ex_aux,

	/* Meshlet Primitive Export */
	output wire mesh_valid,
	input wire mesh_ready,
	output wire [701:0] mesh_tri,
	output wire mesh_last,
	output wire [15:0] mesh_id,

	/* Unified NoC Flit Port (D$ + I$ merged via local atg_rt2) */
	output wire t_req_valid,
	input wire t_req_ready,
	output wire [FW-1:0] t_req_data,
	output wire t_req_last,
	input wire t_rsp_valid,
	output wire t_rsp_ready,
	input wire [RFW-1:0] t_rsp_data,

	/* Atomics */
	output wire at_valid,
	output wire [1:0] at_stage,
	input wire at_ready,
	output wire [3:0] at_op,
	output wire [29:0] at_addr,
	output wire [31:0] at_a,
	output wire [31:0] at_b,
	input wire at_rsp_valid,
	input wire [31:0] at_old,
	input wire at_error,

	/* Systolic array */
	output wire sa_push_valid,
	input wire sa_push_ready,
	output wire [NPHYS*32-1:0] sa_push_data,
	input wire sa_pop_valid,
	output wire sa_pop_ready,
	input wire [NPHYS*32-1:0] sa_pop_data,
	output wire sa_ctl_valid,
	output wire [31:0] sa_ctl_data,
	input wire [31:0] sa_qstat,
	input wire [31:0] sa_stat,
	input wire [31:0] sa_qtag,

	/* Status */
	output wire busy,
	output wire scrub_busy,
	output wire fault,
	output wire [31:0] fault_addr,
	output wire [3:0] fault_stage,
	output wire [3:0] fsm
);
	localparam LA_W = ADDR_W - LINE_SHIFT;
	localparam LINE_BITS = LINE_BYTES * 8;
	localparam [11:0] SU_ID12 = SU_INDEX;
	localparam [7:0] SU_ID_LO = SU_ID12[7:0];
	localparam [3:0] SU_ID_HI = SU_ID12[11:8];

	wire [WARP_W-1:0] unused_vl_slot, unused_fl_slot;

	wire pt_exp_end;
	wire [2:0] pt_exp_blk;
	wire [GRP_W-1:0] pt_exp_grp;
	wire [6:0] pt_exp_word;
	wire [NPHYS-1:0] pt_exp_mask;
	wire [NPHYS*32-1:0] pt_exp_data;
	assign pt_exp_packed = {pt_exp_end, pt_exp_blk, pt_exp_grp, pt_exp_word, pt_exp_mask, pt_exp_data};

	wire d_req_valid, d_req_ready, d_req_write;
	wire [LA_W-1:0] d_req_line;
	wire [LINE_BYTES-1:0] d_req_be;
	wire [LINE_BITS-1:0] d_req_wdata;
	wire [7:0] d_req_tag;
	wire d_rsp_valid, d_rsp_write, d_rsp_error;
	wire [7:0] d_rsp_tag;
	wire [LINE_BITS-1:0] d_rsp_rdata;

	wire i_req_valid, i_req_ready, i_req_write;
	wire [LA_W-1:0] i_req_line;
	wire [LINE_BYTES-1:0] i_req_be;
	wire [LINE_BITS-1:0] i_req_wdata;
	wire [7:0] i_req_tag;
	wire i_rsp_valid, i_rsp_write, i_rsp_error;
	wire [7:0] i_rsp_tag;
	wire [LINE_BITS-1:0] i_rsp_rdata;

	/* Local tile input isolation buffers so multi-SU broadcasts keep FO <= 16 */
	wire        t_st_we, t_cs_kick, t_cs_mesh, t_lc_kick, t_scrub;
	wire [8:0]  t_st_idx;
	wire [31:0] t_st_wdata;
	wire [3:0]  t_inv;
	wire        t_vl_we, t_vl_go, t_fl_we, t_fl_pw_we, t_fl_go, t_at_error;
	wire [5:0]  t_vl_reg, t_fl_reg;
	wire [4:0]  t_vl_lane, t_fl_lane;
	wire [31:0] t_vl_data, t_vl_mask, t_fl_data, t_fl_pw_data, t_fl_mask, t_at_old;
	wire [7:0]  t_vl_go_ptbase;
	wire [11:0] t_fl_pw_addr;
	wire [63:0] t_fl_info;
	wire [15:0] t_ex_next;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tstwe (.I0(st_we),     .O(t_st_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tcsk  (.I0(cs_kick),   .O(t_cs_kick));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tcsm  (.I0(cs_mesh),   .O(t_cs_mesh));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tlck  (.I0(lc_kick),   .O(t_lc_kick));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tscrb (.I0(scrub),     .O(t_scrub));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvwe  (.I0(vl_we),     .O(t_vl_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvgo  (.I0(vl_go),     .O(t_vl_go));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfwe  (.I0(fl_we),     .O(t_fl_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfpwe (.I0(fl_pw_we),  .O(t_fl_pw_we));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfgo  (.I0(fl_go),     .O(t_fl_go));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tater (.I0(at_error),  .O(t_at_error));
	genvar gtb;
	generate
		for (gtb = 0; gtb < 4; gtb = gtb + 1) begin : g_tb4
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tinv (.I0(inv[gtb]), .O(t_inv[gtb]));
		end
		for (gtb = 0; gtb < 5; gtb = gtb + 1) begin : g_tb5
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvln (.I0(vl_lane[gtb]), .O(t_vl_lane[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfln (.I0(fl_lane[gtb]), .O(t_fl_lane[gtb]));
		end
		for (gtb = 0; gtb < 6; gtb = gtb + 1) begin : g_tb6
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvrg (.I0(vl_reg[gtb]), .O(t_vl_reg[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfrg (.I0(fl_reg[gtb]), .O(t_fl_reg[gtb]));
		end
		for (gtb = 0; gtb < 8; gtb = gtb + 1) begin : g_tb8
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvpb (.I0(vl_go_ptbase[gtb]), .O(t_vl_go_ptbase[gtb]));
		end
		for (gtb = 0; gtb < 9; gtb = gtb + 1) begin : g_tb9
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tsti (.I0(st_idx[gtb]), .O(t_st_idx[gtb]));
		end
		for (gtb = 0; gtb < 12; gtb = gtb + 1) begin : g_tb12
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfpa (.I0(fl_pw_addr[gtb]), .O(t_fl_pw_addr[gtb]));
		end
		for (gtb = 0; gtb < 16; gtb = gtb + 1) begin : g_tb16
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_texn (.I0(ex_next[gtb]), .O(t_ex_next[gtb]));
		end
		for (gtb = 0; gtb < 32; gtb = gtb + 1) begin : g_tb32
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tstw (.I0(st_wdata[gtb]),   .O(t_st_wdata[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvd  (.I0(vl_data[gtb]),    .O(t_vl_data[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tvm  (.I0(vl_mask[gtb]),    .O(t_vl_mask[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfd  (.I0(fl_data[gtb]),    .O(t_fl_data[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfpd (.I0(fl_pw_data[gtb]), .O(t_fl_pw_data[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfm  (.I0(fl_mask[gtb]),    .O(t_fl_mask[gtb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tato (.I0(at_old[gtb]),     .O(t_at_old[gtb]));
		end
		for (gtb = 0; gtb < 64; gtb = gtb + 1) begin : g_tb64
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_tfi (.I0(fl_info[gtb]), .O(t_fl_info[gtb]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	assign t_st_we = st_we;
	assign t_cs_kick = cs_kick;
	assign t_cs_mesh = cs_mesh;
	assign t_lc_kick = lc_kick;
	assign t_scrub = scrub;
	assign t_vl_we = vl_we;
	assign t_vl_go = vl_go;
	assign t_fl_we = fl_we;
	assign t_fl_pw_we = fl_pw_we;
	assign t_fl_go = fl_go;
	assign t_at_error = at_error;
	genvar gtb;
	generate
		for (gtb = 0; gtb < 4; gtb = gtb + 1) begin : g_tb4
			assign t_inv[gtb] = inv[gtb];
		end
		for (gtb = 0; gtb < 5; gtb = gtb + 1) begin : g_tb5
			assign t_vl_lane[gtb] = vl_lane[gtb];
			assign t_fl_lane[gtb] = fl_lane[gtb];
		end
		for (gtb = 0; gtb < 6; gtb = gtb + 1) begin : g_tb6
			assign t_vl_reg[gtb] = vl_reg[gtb];
			assign t_fl_reg[gtb] = fl_reg[gtb];
		end
		for (gtb = 0; gtb < 8; gtb = gtb + 1) begin : g_tb8
			assign t_vl_go_ptbase[gtb] = vl_go_ptbase[gtb];
		end
		for (gtb = 0; gtb < 9; gtb = gtb + 1) begin : g_tb9
			assign t_st_idx[gtb] = st_idx[gtb];
		end
		for (gtb = 0; gtb < 12; gtb = gtb + 1) begin : g_tb12
			assign t_fl_pw_addr[gtb] = fl_pw_addr[gtb];
		end
		for (gtb = 0; gtb < 16; gtb = gtb + 1) begin : g_tb16
			assign t_ex_next[gtb] = ex_next[gtb];
		end
		for (gtb = 0; gtb < 32; gtb = gtb + 1) begin : g_tb32
			assign t_st_wdata[gtb] = st_wdata[gtb];
			assign t_vl_data[gtb] = vl_data[gtb];
			assign t_vl_mask[gtb] = vl_mask[gtb];
			assign t_fl_data[gtb] = fl_data[gtb];
			assign t_fl_pw_data[gtb] = fl_pw_data[gtb];
			assign t_fl_mask[gtb] = fl_mask[gtb];
			assign t_at_old[gtb] = at_old[gtb];
		end
		for (gtb = 0; gtb < 64; gtb = gtb + 1) begin : g_tb64
			assign t_fl_info[gtb] = fl_info[gtb];
		end
	endgenerate
`else
	assign t_st_we        = st_we;
	assign t_st_idx       = st_idx;
	assign t_st_wdata     = st_wdata;
	assign t_cs_kick      = cs_kick;
	assign t_cs_mesh      = cs_mesh;
	assign t_lc_kick      = lc_kick;
	assign t_inv          = inv;
	assign t_scrub        = scrub;
	assign t_vl_we        = vl_we;
	assign t_vl_reg       = vl_reg;
	assign t_vl_lane      = vl_lane;
	assign t_vl_data      = vl_data;
	assign t_vl_go        = vl_go;
	assign t_vl_mask      = vl_mask;
	assign t_vl_go_ptbase = vl_go_ptbase;
	assign t_fl_we        = fl_we;
	assign t_fl_reg       = fl_reg;
	assign t_fl_lane      = fl_lane;
	assign t_fl_data      = fl_data;
	assign t_fl_pw_we     = fl_pw_we;
	assign t_fl_pw_addr   = fl_pw_addr;
	assign t_fl_pw_data   = fl_pw_data;
	assign t_fl_go        = fl_go;
	assign t_fl_mask      = fl_mask;
	assign t_fl_info      = fl_info;
	assign t_ex_next      = ex_next;
	assign t_at_old       = at_old;
	assign t_at_error     = at_error;
`endif

	atg_su #(
		.NPHYS(NPHYS), .NWARPS(NWARPS), .WARP_W(WARP_W), .GRP_W(GRP_W),
		.NREGS(NREGS), .SB_ENTRIES(SB_ENTRIES), .SBE_W(SBE_W), .NSU(NSU),
		.SU_INDEX(SU_INDEX), .FRAG_RESERVE(FRAG_RESERVE), .PACKED(PACKED),
		.PK_SHARED(PK_SHARED), .SUBGROUP(SUBGROUP), .HAS_SA(HAS_SA), .MS4(MS4),
		.HAS_MESH(HAS_MESH), .LDS_BYTES(LDS_BYTES), .PT_ENTRIES(PT_ENTRIES),
		.VARY_WORDS(VARY_WORDS), .ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT), .MRT_COUNT(MRT_COUNT), .DUAL_SOURCE(DUAL_SOURCE),
		.SHARED_PT(SHARED_PT), .HAS_GFX(HAS_GFX), .HAS_TU(HAS_TU)
	) u_su (
		.clk(clk), .rst_n(rst_n),
		.st_we(t_st_we), .st_idx(t_st_idx), .st_wdata(t_st_wdata),
		.cs_kick(t_cs_kick), .cs_mesh(t_cs_mesh), .lc_kick(t_lc_kick), .inv(t_inv),
		.vl_req(vl_req), .vl_gnt(vl_gnt), .vl_slot(unused_vl_slot),
		.vl_ptbase(vl_ptbase), .vl_avail(vl_avail),
		.vl_we(t_vl_we), .vl_reg(t_vl_reg), .vl_lane(t_vl_lane), .vl_data(t_vl_data),
		.vl_go(t_vl_go), .vl_mask(t_vl_mask), .vl_go_ptbase(t_vl_go_ptbase),
		.fl_req(fl_req), .fl_gnt(fl_gnt), .fl_slot(unused_fl_slot),
		.fl_avail(fl_avail),
		.fl_we(t_fl_we), .fl_reg(t_fl_reg), .fl_lane(t_fl_lane), .fl_data(t_fl_data),
		.fl_pw_we(t_fl_pw_we), .fl_pw_addr(t_fl_pw_addr), .fl_pw_data(t_fl_pw_data),
		.fl_go(t_fl_go), .fl_mask(t_fl_mask), .fl_info(t_fl_info),
		.pt_rd(pt_rd), .pt_idx(pt_idx), .pt_word(pt_word), .pt_rdata(pt_rdata),
		.pt_free(pt_free), .pt_free_idx(pt_free_idx), .pt_done(pt_done),
		.pt_exp_valid(pt_exp_valid), .pt_exp_ready(pt_exp_ready),
		.pt_exp_end(pt_exp_end), .pt_exp_blk(pt_exp_blk),
		.pt_exp_grp(pt_exp_grp), .pt_exp_word(pt_exp_word),
		.pt_exp_mask(pt_exp_mask), .pt_exp_data(pt_exp_data),
		.tx_valid(tx_valid), .tx_ready(tx_ready), .tx_op(tx_op),
		.tx_slot(tx_slot), .tx_samp_slot(tx_samp_slot), .tx_vs(tx_vs),
		.tx_mask(tx_mask), .tx_coord(tx_coord), .tx_off(tx_off),
		.tx_tag(tx_tag), .tx_last(tx_last),
		.tr_valid(tr_valid), .tr_ready(tr_ready), .tr_tag(tr_tag),
		.tr_data(tr_data), .tr_last(tr_last),
		.ex_valid(ex_valid), .ex_ready(ex_ready), .ex_seq(ex_seq), .ex_xy(ex_xy),
		.ex_mask(ex_mask), .ex_color(ex_color), .ex_color1(ex_color1),
		.ex_has_src1(ex_has_src1), .ex_depth(ex_depth), .ex_last(ex_last),
		.ex_next(t_ex_next), .ex_aux(ex_aux),
		.mesh_valid(mesh_valid), .mesh_ready(mesh_ready), .mesh_tri(mesh_tri),
		.mesh_last(mesh_last), .mesh_id(mesh_id),
		.d_req_valid(d_req_valid), .d_req_ready(d_req_ready), .d_req_write(d_req_write),
		.d_req_line(d_req_line), .d_req_be(d_req_be), .d_req_wdata(d_req_wdata),
		.d_req_tag(d_req_tag),
		.d_rsp_valid(d_rsp_valid), .d_rsp_write(d_rsp_write), .d_rsp_tag(d_rsp_tag),
		.d_rsp_rdata(d_rsp_rdata), .d_rsp_error(d_rsp_error),
		.i_req_valid(i_req_valid), .i_req_ready(i_req_ready), .i_req_write(i_req_write),
		.i_req_line(i_req_line), .i_req_be(i_req_be), .i_req_wdata(i_req_wdata),
		.i_req_tag(i_req_tag),
		.i_rsp_valid(i_rsp_valid), .i_rsp_write(i_rsp_write), .i_rsp_tag(i_rsp_tag),
		.i_rsp_rdata(i_rsp_rdata), .i_rsp_error(i_rsp_error),
		.at_valid(at_valid), .at_stage(at_stage), .at_ready(at_ready),
		.at_op(at_op), .at_addr(at_addr), .at_a(at_a), .at_b(at_b),
		.at_rsp_valid(at_rsp_valid), .at_old(t_at_old), .at_error(t_at_error),
		.sa_push_valid(sa_push_valid), .sa_push_ready(sa_push_ready),
		.sa_push_data(sa_push_data),
		.sa_pop_valid(sa_pop_valid), .sa_pop_ready(sa_pop_ready),
		.sa_pop_data(sa_pop_data),
		.sa_ctl_valid(sa_ctl_valid), .sa_ctl_data(sa_ctl_data),
		.sa_qstat(sa_qstat), .sa_stat(sa_stat), .sa_qtag(sa_qtag),
		.busy(busy), .fsm(fsm),
		.scrub(t_scrub), .scrub_busy(scrub_busy),
		.fault(fault), .fault_addr(fault_addr), .fault_stage(fault_stage)
	);

	wire d_t_req_v, d_t_req_r, d_t_req_l, d_t_rsp_v, d_t_rsp_r;
	wire [FW-1:0] d_t_req_d;
	wire [RFW-1:0] d_t_rsp_d;

	wire i_t_req_v, i_t_req_r, i_t_req_l, i_t_rsp_v, i_t_rsp_r;
	wire [FW-1:0] i_t_req_d;
	wire [RFW-1:0] i_t_rsp_d;

	am_amb_to_rt #(
		.ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT),
		.VRAM_AW(ADDR_W), .VRAM_BASE({ADDR_W{1'b0}}),
		.RW(SU_RW), .ROUTE(ROUTE_D), .CLIENT(SU_ID_LO), .CLIENT_HI(SU_ID_HI),
		.PRIO(0), .FW(FW), .RFW(RFW)
	) u_d_to_rt (
		.clk(clk), .rst_n(rst_n), .flush(flush), .hold(1'b0),
		.req_valid(d_req_valid), .req_ready(d_req_ready), .req_write(d_req_write),
		.req_line(d_req_line), .req_be(d_req_be), .req_wdata(d_req_wdata),
		.req_tag(d_req_tag), .req_chk(1'b0), .req_tbl(1'b0), .req_slot(6'd0),
		.req_session(session_id), .req_exec_type(d_req_write ? 2'b01 : 2'b00),
		.rsp_valid(d_rsp_valid), .rsp_write(d_rsp_write), .rsp_tag(d_rsp_tag),
		.rsp_rdata(d_rsp_rdata), .rsp_error(d_rsp_error),
		.t_req_valid(d_t_req_v), .t_req_ready(d_t_req_r),
		.t_req_data(d_t_req_d), .t_req_last(d_t_req_l),
		.t_rsp_valid(d_t_rsp_v), .t_rsp_ready(d_t_rsp_r), .t_rsp_data(d_t_rsp_d)
	);

	am_amb_to_rt #(
		.ADDR_W(ADDR_W), .LINE_BYTES(LINE_BYTES), .LINE_SHIFT(LINE_SHIFT),
		.VRAM_AW(ADDR_W), .VRAM_BASE({ADDR_W{1'b0}}),
		.RW(SU_RW), .ROUTE(ROUTE_I), .CLIENT(SU_ID_LO), .CLIENT_HI(SU_ID_HI),
		.PRIO(0), .FW(FW), .RFW(RFW)
	) u_i_to_rt (
		.clk(clk), .rst_n(rst_n), .flush(flush), .hold(1'b0),
		.req_valid(i_req_valid), .req_ready(i_req_ready), .req_write(i_req_write),
		.req_line(i_req_line), .req_be(i_req_be), .req_wdata(i_req_wdata),
		.req_tag(i_req_tag), .req_chk(1'b0), .req_tbl(1'b0), .req_slot(6'd0),
		.req_session(session_id), .req_exec_type(2'b10),
		.rsp_valid(i_rsp_valid), .rsp_write(i_rsp_write), .rsp_tag(i_rsp_tag),
		.rsp_rdata(i_rsp_rdata), .rsp_error(i_rsp_error),
		.t_req_valid(i_t_req_v), .t_req_ready(i_t_req_r),
		.t_req_data(i_t_req_d), .t_req_last(i_t_req_l),
		.t_rsp_valid(i_t_rsp_v), .t_rsp_ready(i_t_rsp_r), .t_rsp_data(i_t_rsp_d)
	);

	wire unused_rt2_idle;
	atg_rt2 #(.FW(FW), .RFW(RFW), .RBIT(RBIT_SU)) u_rt2_su (
		.clk(clk), .rst_n(rst_n), .flush(flush),
		.c0_req_valid(d_t_req_v), .c0_req_ready(d_t_req_r),
		.c0_req_data(d_t_req_d), .c0_req_last(d_t_req_l),
		.c1_req_valid(i_t_req_v), .c1_req_ready(i_t_req_r),
		.c1_req_data(i_t_req_d), .c1_req_last(i_t_req_l),
		.p_req_valid(t_req_valid), .p_req_ready(t_req_ready),
		.p_req_data(t_req_data), .p_req_last(t_req_last),
		.p_rsp_valid(t_rsp_valid), .p_rsp_ready(t_rsp_ready), .p_rsp_data(t_rsp_data),
		.c0_rsp_valid(d_t_rsp_v), .c0_rsp_ready(d_t_rsp_r), .c0_rsp_data(d_t_rsp_d),
		.c1_rsp_valid(i_t_rsp_v), .c1_rsp_ready(i_t_rsp_r), .c1_rsp_data(i_t_rsp_d),
		.rsp_idle(unused_rt2_idle)
	);

	wire unused_ok = &{1'b0, unused_vl_slot, unused_fl_slot, unused_rt2_idle,
		d_t_rsp_r, i_t_rsp_r};
endmodule

`default_nettype wire
