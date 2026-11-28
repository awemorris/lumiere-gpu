/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_sched: The SU's warp scheduler: warp slots and their PCs, slot
 * allocation for vertex warps (VF), fragment warps (RX) and compute
 * workgroups (DISPATCH), round-robin selection of a ready warp for fetch,
 * and the warp-info word for special operands.
 *
 * The behaviour, and where it differs from
 * the design: spec/gpu/su/atg_su_sched.md (§10 Status).
 */

`timescale 1ns / 1ps
`default_nettype none

/* verilator lint_off BLKSEQ */
module atg_su_sched #(
	parameter NPHYS = 8,	/* Physical lanes: 4, 8, 16 or 32 (a warp is 32 lanes, one instruction takes 32/NPHYS cycles) */
	parameter NWARPS = 4,	/* Resident warps (slots): 2, 4 or 8 */
	parameter WARP_W = 2,	/* log2(NWARPS) */
	parameter NSU = 1,	/* Number of SUs (compute workgroups are shared out: this SU takes wg_id % NSU == SU_INDEX) */
	parameter SU_INDEX = 0,	/* This SU's number */
	parameter FRAG_RESERVE = 1,	/* Slots kept for fragment warps (design §3.8; KC705 2) */
	parameter GTO = 1,	/* Scheduling policy: 1 = Greedy-Then-Oldest, 0 = Round-Robin */
	parameter USE_QW_REP = 0,	/* 1: use 4-slice replicated q_warp_rep */
	parameter HAS_GFX = 1,	/* 1: vertex/fragment scheduling enabled; 0: compute-only */
	parameter SHARED_PT = 0	/* 1: shared Spine PT; ignore local pt_ok */
) (
	input wire clk,
	input wire rst_n,
	/* STATE broadcast (from atg_cp_state) */
	input wire st_we,
	input wire [8:0] st_idx,
	input wire [31:0] st_wdata,
	input wire cs_kick,
	input wire cs_mesh,
	output wire cs_is_mesh_act,
	/* Vertex warp allocation (VF, via atg_su ports vl_*) */
	input wire vl_req,
	output wire vl_gnt,
	output wire [WARP_W-1:0] vl_slot,
	output wire vl_avail,
	input wire vl_go,
	input wire [31:0] vl_mask,
	/* Fragment warp allocation (RX, via atg_su ports fl_*) */
	input wire fl_req,
	output wire fl_gnt,
	output wire [WARP_W-1:0] fl_slot,
	output wire fl_avail,
	input wire fl_go,
	input wire [31:0] fl_mask,
	input wire [63:0] fl_info,
	/* Fetch (to atg_su_fetch) */
	output wire f_valid,
	input wire f_ready,
	output wire [WARP_W-1:0] f_warp,
	output wire [15:0] f_pc,
	output wire [31:0] f_addr,
	/* Feedback (from atg_su_sb and atg_su_exec) */
	input wire fb_valid,
	input wire [WARP_W-1:0] fb_warp,
	input wire [1:0] fb_kind,
	input wire [NWARPS-1:0] wake,
	input wire [NWARPS-1:0] warp_ready,
	input wire wc_valid,
	input wire [WARP_W-1:0] wc_warp,
	input wire [1:0] wc_op,
	input wire [15:0] wc_pc,
	/* Warp information */
	input wire [WARP_W-1:0] q_warp,
	input wire [WARP_W*4-1:0] q_warp_rep,
	output wire [159:0] q_info,
	output wire init_valid,
	output wire [WARP_W-1:0] init_warp,
	output wire [31:0] init_mask,
	input wire pt_ok,
	input wire xr_valid,
	input wire [WARP_W-1:0] xr_warp,
	output wire busy
);
`include "atg_su_uop.vh"
`include "ghci_regs.vh"

	localparam G = 32 / NPHYS;
	localparam GAP_W = 3;
	localparam [2:0] S_FREE = 3'd0, S_ALLOC = 3'd1, S_READY = 3'd2, S_FETCH = 3'd3,
		S_WC = 3'd4, S_REPLAY = 3'd5, S_BAR = 3'd6, S_ENDED = 3'd7;
	localparam [1:0] K_VERT = 2'd0, K_FRAG = 2'd1, K_COMP = 2'd2;

	wire rst_p = !rst_n;

	/* STATE: program bases, WG_SIZE, the DISPATCH grid. */
	reg [31:0] vs_base, fs_base, cs_base;
	reg [7:0] wg_x, wg_y, wg_z;
	reg [15:0] grid_x, grid_y, grid_z;
	reg [15:0] grid_x_m1, grid_y_m1, grid_z_m1;
	wire [15:0] st_wdata_m1;
	wire wdata_m1_bo_unused;
	am_sub #(.W(16)) u_wdata_m1 (.a(st_wdata[15:0]), .b(16'd0), .bi(1'b1), .d(st_wdata_m1), .bo(wdata_m1_bo_unused));
	(* keep = "true" *) wire we_vs_lo = st_we && (st_idx == GHCI_S_VS_BASE);
	(* keep = "true" *) wire we_vs_hi = st_we && (st_idx == GHCI_S_VS_BASE);
	(* keep = "true" *) wire we_fs_lo = st_we && (st_idx == GHCI_S_FS_BASE);
	(* keep = "true" *) wire we_fs_hi = st_we && (st_idx == GHCI_S_FS_BASE);
	(* keep = "true" *) wire we_cs_lo = st_we && (st_idx == GHCI_S_CS_BASE);
	(* keep = "true" *) wire we_cs_hi = st_we && (st_idx == GHCI_S_CS_BASE);
	(* keep = "true" *) wire we_wgsz  = st_we && (st_idx == GHCI_S_WG_SIZE);
	(* keep = "true" *) wire we_gx    = st_we && (st_idx == GHCI_S_GRID_X);
	(* keep = "true" *) wire we_gx_m1 = st_we && (st_idx == GHCI_S_GRID_X);
	(* keep = "true" *) wire we_gy    = st_we && (st_idx == GHCI_S_GRID_Y);
	(* keep = "true" *) wire we_gy_m1 = st_we && (st_idx == GHCI_S_GRID_Y);
	(* keep = "true" *) wire we_gz    = st_we && (st_idx == GHCI_S_GRID_Z);
	(* keep = "true" *) wire we_gz_m1 = st_we && (st_idx == GHCI_S_GRID_Z);
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			{vs_base, fs_base, cs_base} <= 96'd0;
			{wg_x, wg_y, wg_z} <= {8'd1, 8'd1, 8'd1};
			{grid_x, grid_y, grid_z} <= {16'd1, 16'd1, 16'd1};
			{grid_x_m1, grid_y_m1, grid_z_m1} <= {16'd0, 16'd0, 16'd0};
		end else begin
			if (we_vs_lo) vs_base[15:0]  <= st_wdata[15:0];
			if (we_vs_hi) vs_base[31:16] <= st_wdata[31:16];
			if (we_fs_lo) fs_base[15:0]  <= st_wdata[15:0];
			if (we_fs_hi) fs_base[31:16] <= st_wdata[31:16];
			if (we_cs_lo) cs_base[15:0]  <= st_wdata[15:0];
			if (we_cs_hi) cs_base[31:16] <= st_wdata[31:16];
			if (we_wgsz)  {wg_z, wg_y, wg_x} <= st_wdata[23:0];
			if (we_gx)    grid_x    <= st_wdata[15:0];
			if (we_gx_m1) grid_x_m1 <= st_wdata_m1;
			if (we_gy)    grid_y    <= st_wdata[15:0];
			if (we_gy_m1) grid_y_m1 <= st_wdata_m1;
			if (we_gz)    grid_z    <= st_wdata[15:0];
			if (we_gz_m1) grid_z_m1 <= st_wdata_m1;
		end
	end

	/* The warp-info word (ATG_WI_* of include/atg_su_uop.vh). */
	function [159:0] wi;
		input [47:0] wg, nwg;
		input [7:0] lid;
		input [1:0] kind;
		input front;
		input [15:0] seq;
		input [31:0] mask;
		wi = {mask, seq, 5'd0, front, kind, lid, nwg, wg};
	endfunction

	/* Per slot. */
	reg [NWARPS*3-1:0] st;
	(* dont_touch = "true" *) reg [NWARPS-1:0] st_free_r, st_free_r_out, st_free_r_pc, st_free_r_bar;
	reg [NWARPS-1:0] st_rdy_r;
	reg [NWARPS-1:0] st_rof_r;
	reg [NWARPS*16-1:0] pc;
	reg [NWARPS*GAP_W-1:0] gap;
	reg [NWARPS-1:0] gap_z_r;
	wire [NWARPS*160-1:0] info;
	reg [NWARPS*4-1:0] tag;	/* workgroup of a compute warp (barriers) */

	/* Compute launch: the next workgroup id, its warps, and whose it is. */
	reg cs_act, cs_is_mesh;
	reg [15:0] id_x, id_y, id_z;
	reg [7:0] rot;	/* linear id % NSU */
	reg [3:0] wg_tag;
	reg [3:0] wk;	/* next warp of the workgroup being launched */
	reg launching;
	(* dont_touch = "true" *) reg launching_vg, launching_st, launching_fa;
	(* dont_touch = "true" *) reg [WARP_W-1:0] free_slot_r_st, free_slot_r_fa;
	/* Compute workgroup size pipelined over 7 cycles (u_lxy LAT=3 + u_lanes LAT=3 + output reg) */
	reg [7:0] wg_z_r1, wg_z_r2, wg_z_r3;
	wire [15:0] lanes_xy;
	wire [23:0] lanes;
	(* dont_touch = "true" *) reg [23:0] lanes_r;
	(* dont_touch = "true" *) reg [4:0]  lanes_r_b;
	am_mul #(.AW(8), .BW(8), .LAT(3)) u_lxy (.clk(clk), .en(1'b1), .a(wg_x), .b(wg_y), .p(lanes_xy));
	am_mul #(.AW(16), .BW(8), .LAT(3)) u_lanes (.clk(clk), .en(1'b1), .a(lanes_xy), .b(wg_z_r3), .p(lanes));

	/* Pure-LUT ceil(lanes_r / 32) clamped to NWARPS (0 CARRY4s, <= 3 LUTs) */
	wire [3:0] q_lanes = lanes_r[8:5];
	wire       rem_nz  = |lanes_r[4:0];
	wire       hi_nz   = |lanes_r[23:9];
	wire [3:0] q_p1    = {q_lanes[3] ^ (&q_lanes[2:0]), q_lanes[2] ^ (&q_lanes[1:0]), q_lanes[1] ^ q_lanes[0], ~q_lanes[0]};
	wire [3:0] nw_cand = rem_nz ? q_p1 : q_lanes;
	wire       nw_ovf  = hi_nz || (rem_nz && (&q_lanes)) || ({28'd0, nw_cand} > NWARPS);
	reg [3:0] nw;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			wg_z_r1   <= 8'd1;
			wg_z_r2   <= 8'd1;
			wg_z_r3   <= 8'd1;
			lanes_r   <= 24'd1;
			lanes_r_b <= 5'd1;
			nw        <= 4'd1;
		end else begin
			wg_z_r1   <= wg_z;
			wg_z_r2   <= wg_z_r1;
			wg_z_r3   <= wg_z_r2;
			lanes_r   <= lanes;
			lanes_r_b <= lanes[4:0];
			nw        <= nw_ovf ? NWARPS[3:0] : nw_cand;
		end
	end
	(* dont_touch = "true" *) reg mine_wg_r;
	wire mine_wg = mine_wg_r;
	wire [15:0] id_x1, id_y1, id_z1;	/* the ids + 1 */
	wire ix_co_unused, iy_co_unused, iz_co_unused;
	am_add #(.W(16)) u_idx1 (.a(id_x), .b(16'd0), .ci(1'b1), .s(id_x1), .co(ix_co_unused));
	am_add #(.W(16)) u_idy1 (.a(id_y), .b(16'd0), .ci(1'b1), .s(id_y1), .co(iy_co_unused));
	am_add #(.W(16)) u_idz1 (.a(id_z), .b(16'd0), .ci(1'b1), .s(id_z1), .co(iz_co_unused));
	wire id_x_last_raw = (id_x == grid_x_m1);
	wire id_x_last, id_x_last_y, id_x_last_z;
	wire id_y_last_raw = (id_y == grid_y_m1);
	wire id_y_last, id_y_last_y, id_y_last_z;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idxl  (.I0(id_x_last_raw), .O(id_x_last));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idxly (.I0(id_x_last_raw), .O(id_x_last_y));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idxlz (.I0(id_x_last_raw), .O(id_x_last_z));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idyl  (.I0(id_y_last_raw), .O(id_y_last));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idyly (.I0(id_y_last_raw), .O(id_y_last_y));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_idylz (.I0(id_y_last_raw), .O(id_y_last_z));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign id_x_last = id_x_last_raw;
	assign id_x_last_y = id_x_last_raw;
	assign id_x_last_z = id_x_last_raw;
	assign id_y_last = id_y_last_raw;
	assign id_y_last_y = id_y_last_raw;
	assign id_y_last_z = id_y_last_raw;
`else
	assign id_x_last   = id_x_last_raw;
	assign id_x_last_y = id_x_last_raw;
	assign id_x_last_z = id_x_last_raw;
	assign id_y_last   = id_y_last_raw;
	assign id_y_last_y = id_y_last_raw;
	assign id_y_last_z = id_y_last_raw;
`endif
	(* keep = "true" *) wire id_z_last = (id_z == grid_z_m1);
	wire last_id = id_x_last && id_y_last && id_z_last;

	/* Free slots directly from registered 1-bit flags st_free_r (0 LUTs!). */
	wire [NWARPS-1:0] slot_free = st_free_r_out;
	wire any_free = |slot_free;
	wire [WARP_W-1:0] free_slot;
	wire [4:0] n_free;
	wire n_free_ge_nw;
	wire n_free_gt_frag;
	generate
		if (NWARPS == 4) begin : g_nfree4
			assign free_slot = slot_free[0] ? 2'd0 : slot_free[1] ? 2'd1 : slot_free[2] ? 2'd2 : 2'd3;
			assign n_free[0] = ^slot_free;
			assign n_free[1] = (slot_free[0] & slot_free[1]) ^ (slot_free[2] & slot_free[3]) ^
			                   ((slot_free[0] ^ slot_free[1]) & (slot_free[2] ^ slot_free[3]));
			assign n_free[2] = &slot_free;
			assign n_free[4:3] = 2'b00;
			(* keep = "true" *) wire [2:0] nf3 = n_free[2:0];
			assign n_free_ge_nw   = (nf3 >= nw[2:0]);
			assign n_free_gt_frag = ({27'd0, n_free} > FRAG_RESERVE);
		end else begin : g_nfree_gen
			am_popcount #(.W(NWARPS), .OW(5)) u_nf_pop (
				.i_vec(slot_free),
				.o_cnt(n_free)
			);
			if (NWARPS == 8) begin : g_fs8
				wire lo_any = |slot_free[3:0];
				wire [1:0] lo_s = slot_free[0] ? 2'd0 : (slot_free[1] ? 2'd1 : (slot_free[2] ? 2'd2 : 2'd3));
				wire [1:0] hi_s = slot_free[4] ? 2'd0 : (slot_free[5] ? 2'd1 : (slot_free[6] ? 2'd2 : 2'd3));
				assign free_slot = lo_any ? {1'b0, lo_s} : {1'b1, hi_s};
			end else begin : g_fs_gen
				reg [WARP_W-1:0] fs_r;
				integer fi;
				always @(*) begin
					fs_r = {WARP_W{1'b0}};
					for (fi = NWARPS - 1; fi >= 0; fi = fi - 1)
						if (slot_free[fi])
							fs_r = fi[WARP_W-1:0];
				end
				assign free_slot = fs_r;
			end
			assign n_free_ge_nw   = ({1'b0, n_free} >= {2'b00, nw});
			assign n_free_gt_frag = ({27'd0, n_free} > FRAG_RESERVE);
		end
	endgenerate

	/* Register slot_free-derived allocation triggers (breaks 27-cmos3 slot_free -> popcount -> alloc loop). */
	reg [WARP_W-1:0] free_slot_r;
	reg any_free_r, n_free_ge_nw_r, n_free_gt_frag_r;
	reg alloc_cool_r;

	/* Grants (one outstanding per source); compute launches hold them off (<= 3 LUTs from FFs). */
	reg vl_wait, fl_wait;
	reg vl_gnt_r, fl_gnt_r;
	reg [WARP_W-1:0] vl_s, fl_s;
	(* keep = "true" *) wire cs_try  = cs_act && !launching && mine_wg && !alloc_cool_r && n_free_ge_nw_r;
	(* keep = "true" *) wire cs_skip_raw = cs_act && !launching && !mine_wg;
	wire cs_skip, cs_skip_ctl;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cssk0 (.I0(cs_skip_raw), .O(cs_skip));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cssk1 (.I0(cs_skip_raw), .O(cs_skip_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cs_skip = cs_skip_raw;
	assign cs_skip_ctl = cs_skip_raw;
`else
	assign cs_skip     = cs_skip_raw;
	assign cs_skip_ctl = cs_skip_raw;
`endif
	wire cs_busy_blk = cs_act && mine_wg && !alloc_cool_r && n_free_ge_nw_r;
	wire fl_req_can = (HAS_GFX != 0) && fl_req && !fl_wait && !alloc_cool_r && any_free_r;
	wire vl_req_can = (HAS_GFX != 0) && vl_req && !vl_wait && (SHARED_PT != 0 ? 1'b1 : pt_ok) && !alloc_cool_r && n_free_gt_frag_r;
	wire fl_g = fl_req_can && !launching_vg && !cs_busy_blk;
	wire vl_g = vl_req_can && !fl_req_can && !launching_vg && !cs_busy_blk;
	wire any_fl_vl_raw = (fl_req_can || vl_req_can) && !launching_vg && !cs_busy_blk;
	wire any_fl_vl_lo, any_fl_vl_hi, fl_g_fa_lo, fl_g_fa_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_afvlo (.I0(any_fl_vl_raw), .O(any_fl_vl_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_afvhi (.I0(any_fl_vl_raw), .O(any_fl_vl_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fgflo (.I0(fl_g),          .O(fl_g_fa_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fgfhi (.I0(fl_g),          .O(fl_g_fa_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign any_fl_vl_lo = any_fl_vl_raw;
	assign any_fl_vl_hi = any_fl_vl_raw;
	assign fl_g_fa_lo = fl_g;
	assign fl_g_fa_hi = fl_g;
`else
	assign any_fl_vl_lo = any_fl_vl_raw;
	assign any_fl_vl_hi = any_fl_vl_raw;
	assign fl_g_fa_lo   = fl_g;
	assign fl_g_fa_hi   = fl_g;
`endif
	assign fl_gnt = fl_gnt_r;
	assign vl_gnt = vl_gnt_r;
	assign fl_slot = fl_s;
	assign vl_slot = vl_s;

	reg vl_avail_r, fl_avail_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			vl_avail_r <= 1'b0;
			fl_avail_r <= 1'b0;
		end else begin
			vl_avail_r <= (HAS_GFX != 0) && !vl_wait && (SHARED_PT != 0 ? 1'b1 : pt_ok) &&
				!alloc_cool_r && n_free_gt_frag_r && !vl_g && !launching_vg && !cs_try;
			fl_avail_r <= (HAS_GFX != 0) && !fl_wait && !alloc_cool_r && any_free_r && !fl_g && !launching_vg && !cs_try;
		end
	end
	assign vl_avail = vl_avail_r;
	assign fl_avail = fl_avail_r;

	/* Initial EXEC: vertex/fragment go, else the next compute warp (0 CARRY4s, fanout <= 16). */
	reg [31:0] lmask;	/* (1 << lanes_r[4:0]) - 1 */
	integer li;
	always @(*) begin
		for (li = 0; li < 16; li = li + 1)
			lmask[li] = li < {27'd0, lanes_r[4:0]};
		for (li = 16; li < 32; li = li + 1)
			lmask[li] = li < {27'd0, lanes_r_b};
	end
	(* keep = "true" *) wire wl_ge32_lo = hi_nz || (q_lanes > wk);
	(* keep = "true" *) wire wl_ge32_hi = hi_nz || (q_lanes > wk);
	(* keep = "true" *) wire [15:0] cmask_lo = wl_ge32_lo ? 16'hFFFF : lmask[15:0];
	(* keep = "true" *) wire [15:0] cmask_hi = wl_ge32_hi ? 16'hFFFF : lmask[31:16];
	wire [31:0] cmask = {cmask_hi, cmask_lo};
	wire vl_go_l, fl_go_l, vl_go_lo, vl_go_hi, fl_go_lo, fl_go_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlgl  (.I0(vl_go), .O(vl_go_l));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flgl  (.I0(fl_go), .O(fl_go_l));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlglo (.I0(vl_go), .O(vl_go_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlghi (.I0(vl_go), .O(vl_go_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flglo (.I0(fl_go), .O(fl_go_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flghi (.I0(fl_go), .O(fl_go_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign vl_go_l = vl_go;
	assign fl_go_l = fl_go;
	assign vl_go_lo = vl_go;
	assign vl_go_hi = vl_go;
	assign fl_go_lo = fl_go;
	assign fl_go_hi = fl_go;
`else
	assign vl_go_l  = vl_go; assign fl_go_l  = fl_go;
	assign vl_go_lo = vl_go; assign vl_go_hi = vl_go;
	assign fl_go_lo = fl_go; assign fl_go_hi = fl_go;
`endif
	(* keep = "true" *) wire cs_launch    = launching_st && !vl_go_l && !fl_go_l && !alloc_cool_r && any_free_r;
	(* dont_touch = "true" *) wire cs_launch_fa = launching_fa && !vl_go_l && !fl_go_l && !alloc_cool_r && any_free_r;
	assign init_valid = vl_go_l || fl_go_l || cs_launch;
	assign init_warp = vl_go_l ? vl_s : fl_go_l ? fl_s : free_slot_r_st;
	assign init_mask = {
		vl_go_hi ? vl_mask[31:16] : (fl_go_hi ? fl_mask[31:16] : cmask_hi),
		vl_go_lo ? vl_mask[15:0]  : (fl_go_lo ? fl_mask[15:0]  : cmask_lo)
	};

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			free_slot_r      <= {WARP_W{1'b0}};
			free_slot_r_st   <= {WARP_W{1'b0}};
			free_slot_r_fa   <= {WARP_W{1'b0}};
			any_free_r       <= 1'b1;
			n_free_ge_nw_r   <= 1'b1;
			n_free_gt_frag_r <= (NWARPS > FRAG_RESERVE);
			alloc_cool_r     <= 1'b0;
		end else begin
			free_slot_r      <= free_slot;
			free_slot_r_st   <= free_slot;
			free_slot_r_fa   <= free_slot;
			any_free_r       <= any_free;
			n_free_ge_nw_r   <= n_free_ge_nw;
			n_free_gt_frag_r <= n_free_gt_frag;
			alloc_cool_r     <= cs_launch || fl_g || vl_g;
		end
	end

	/* Candidates: slots that are READY, out of gap, clean f_addr, and warp_ready from scoreboard */
	wire [NWARPS-1:0] w_faddr_dirty;
	wire [NWARPS-1:0] slot_rdy_nogap = st_rdy_r & gap_z_r;
	wire [NWARPS-1:0] cand = st_rdy_r & gap_z_r & warp_ready & (~w_faddr_dirty);
	genvar gc;

	/* Registered pairwise relative ages for GTO Oldest policy (0 CARRY4s, 0 Pre-LUTs!) */
	reg cmp01_r, cmp02_r, cmp03_r, cmp12_r, cmp13_r, cmp23_r;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			{cmp01_r, cmp02_r, cmp03_r, cmp12_r, cmp13_r, cmp23_r} <= 6'd0;
		end else begin
			if (!cand[1])      cmp01_r <= 1'b0;
			else if (!cand[0]) cmp01_r <= 1'b1;
			if (!cand[2])      cmp02_r <= 1'b0;
			else if (!cand[0]) cmp02_r <= 1'b1;
			if (!cand[3])      cmp03_r <= 1'b0;
			else if (!cand[0]) cmp03_r <= 1'b1;
			if (!cand[2])      cmp12_r <= 1'b0;
			else if (!cand[1]) cmp12_r <= 1'b1;
			if (!cand[3])      cmp13_r <= 1'b0;
			else if (!cand[1]) cmp13_r <= 1'b1;
			if (!cand[3])      cmp23_r <= 1'b0;
			else if (!cand[2]) cmp23_r <= 1'b1;
		end
	end

	/* Selection: GTO (Greedy-Then-Oldest, <= 3 LUTs from FFs) or Round-Robin */
	reg [WARP_W-1:0] rr;
	reg [NWARPS-1:0] rr_ge_r;
	reg [WARP_W-1:0] last_warp;
	reg [NWARPS-1:0] last_warp_oh;
	reg active_valid;
	reg hold_v;
	reg [WARP_W-1:0] hold_w;
	reg [NWARPS-1:0] hold_oh;

	wire [NWARPS-1:0] slot_rdy_or_fetch = st_rof_r;
	wire active_live = active_valid && |(last_warp_oh & cand);
	wire use_fixed   = hold_v || active_live;
	wire w0_ok03 = cand[0] && !(cand[3] && cmp03_r);
	wire w0_ok12 = !(cand[1] && cmp01_r) && !(cand[2] && cmp02_r);
	wire w1_ok13 = cand[1] && !(cand[3] && cmp13_r);
	wire w1_ok02 = !(cand[0] && !cmp01_r) && !(cand[2] && cmp12_r);
	wire w2_ok23 = cand[2] && !(cand[3] && cmp23_r);
	wire w2_ok01 = !(cand[0] && !cmp02_r) && !(cand[1] && !cmp12_r);

	wire w0_win = w0_ok03 && w0_ok12;
	wire w1_win = w1_ok13 && w1_ok02;
	wire w2_win = w2_ok23 && w2_ok01;

	wire [WARP_W-1:0] fsel;
	wire [NWARPS-1:0] fsel_oh;
	wire [NWARPS-1:0] fix_mux_oh;
	wire [NWARPS-1:0] cand_sel_oh;
	wire use_fix_mux;
	wire ufm_pc, ufm_fa0, ufm_fa1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ufm_pc  (.I0(use_fix_mux), .O(ufm_pc));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ufm_fa0 (.I0(use_fix_mux), .O(ufm_fa0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ufm_fa1 (.I0(use_fix_mux), .O(ufm_fa1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign ufm_pc = use_fix_mux;
	assign ufm_fa0 = use_fix_mux;
	assign ufm_fa1 = use_fix_mux;
`else
	assign ufm_pc  = use_fix_mux;
	assign ufm_fa0 = use_fix_mux;
	assign ufm_fa1 = use_fix_mux;
`endif
	wire fsel_v;
	generate
		if (GTO != 0 && NWARPS == 4) begin : g_sel_gto4
			wire hold_v_b0, hold_v_b1, hold_v_b2;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hvb0 (.I0(hold_v), .O(hold_v_b0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hvb1 (.I0(hold_v), .O(hold_v_b1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_hvb2 (.I0(hold_v), .O(hold_v_b2));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign hold_v_b0 = hold_v;
			assign hold_v_b1 = hold_v;
			assign hold_v_b2 = hold_v;
`else
			assign hold_v_b0 = hold_v;
			assign hold_v_b1 = hold_v;
			assign hold_v_b2 = hold_v;
`endif
			wire [3:0] fix_oh = hold_v_b0 ? hold_oh : last_warp_oh;
			wire [WARP_W-1:0] fixed_w = hold_v_b1 ? hold_w : last_warp;
			wire w3_win = cand[3] && !w0_win && !w1_win && !w2_win;
			assign fsel[1] = use_fixed ? fixed_w[1] : (!w0_win && !w1_win);
			assign fsel[0] = use_fixed ? fixed_w[0] : (!w0_win && !w2_win);
			assign fsel_v  = hold_v_b2 || (|cand);
			wire [3:0] gto_oh = {
				w3_win,
				w2_win,
				w1_win,
				w0_win
			};
			assign fix_mux_oh  = fix_oh;
			assign cand_sel_oh = gto_oh;
			assign use_fix_mux = use_fixed;
			assign fsel_oh     = use_fixed ? fix_oh : gto_oh;
		end else begin : g_sel_rr
			/* Parallel unrotated 2-way RR arbiter (breaks barrel-rotate + pe + add chain). */
			wire [NWARPS-1:0] cand_hi = cand & rr_ge_r;
			wire [NWARPS-1:0] win_lo_1h, win_hi_1h;
			assign win_lo_1h[0] = cand[0];
			assign win_hi_1h[0] = cand_hi[0];
			genvar grr, gpb;
			for (grr = 1; grr < NWARPS; grr = grr + 1) begin : g_pe
				assign win_lo_1h[grr] = cand[grr] && !(|cand[grr-1:0]);
				assign win_hi_1h[grr] = cand_hi[grr] && !(|cand_hi[grr-1:0]);
			end
			wire any_hi = |cand_hi;
			wire sel_v_rr = |cand;
			wire [NWARPS-1:0] win_rr_1h = any_hi ? win_hi_1h : win_lo_1h;
			assign fix_mux_oh  = hold_oh;
			assign cand_sel_oh = win_rr_1h;
			assign use_fix_mux = hold_v;
			assign fsel_oh     = hold_v ? hold_oh : win_rr_1h;
			for (gpb = 0; gpb < WARP_W; gpb = gpb + 1) begin : g_bin
				wire [NWARPS-1:0] bmask;
				for (grr = 0; grr < NWARPS; grr = grr + 1) begin : g_bm
					assign bmask[grr] = fsel_oh[grr] && grr[gpb];
				end
				assign fsel[gpb] = |bmask;
			end
			assign fsel_v = hold_v || sel_v_rr;
		end
	endgenerate
	wire [NWARPS-1:0] next_rr_ge;
	assign next_rr_ge[0] = fsel_oh[NWARPS - 1];
	generate
		genvar gnge;
		for (gnge = 1; gnge < NWARPS; gnge = gnge + 1) begin : g_nrr_ge
			assign next_rr_ge[gnge] = fsel_oh[NWARPS - 1] || (|fsel_oh[gnge-1:0]);
		end
	endgenerate
	wire [159:0] q_info_out;
	genvar gqi;
	generate
		for (gqi = 0; gqi < 10; gqi = gqi + 1) begin : g_qinfo_sl
			wire [WARP_W-1:0] q_warp_in = USE_QW_REP ? q_warp_rep[((gqi * 4) / 10) * WARP_W +: WARP_W] : q_warp;
			wire [WARP_W-1:0] q_warp_sl;
			genvar gqb;
			for (gqb = 0; gqb < WARP_W; gqb = gqb + 1) begin : g_qw_buf
`ifdef SYNTH_KINTEX7
				(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_qw (.I0(q_warp_in[gqb]), .O(q_warp_sl[gqb]));
`elsif SYNTH_KINTEX_ULTRASCALE
				assign q_warp_sl[gqb] = q_warp_in[gqb];
`else
				assign q_warp_sl[gqb] = q_warp_in[gqb];
`endif
			end
			wire [NWARPS*16-1:0] q_i_m;
			genvar gy;
			for (gy = 0; gy < NWARPS; gy = gy + 1) begin : g_qi_m
				assign q_i_m[gy*16 +: 16] = (q_warp_sl == gy[WARP_W-1:0]) ? info[gy*160 + gqi*16 +: 16] : 16'd0;
			end
			reg [15:0] q_i_sl;
			integer y;
			always @(*) begin
				q_i_sl = 16'd0;
				for (y = 0; y < NWARPS; y = y + 1)
					q_i_sl = q_i_sl | q_i_m[y*16 +: 16];
			end
			assign q_info_out[gqi*16 +: 16] = q_i_sl;
		end
	endgenerate
	assign q_info = q_info_out;

	/* Precompute and register fetch address per warp independently from selection tournament */
	reg [NWARPS*32-1:0] w_f_addr_r;
	reg [NWARPS*32-1:0] w_f_addr_p8_r;
	reg [NWARPS*32-1:0] w_base_r;
	reg [NWARPS*2-1:0]  w_faddr_dirty_r;
	reg [NWARPS-1:0]    w_fa_cy_r;
	wire [NWARPS-1:0]   w_fa_co_unused;
	genvar g_fa;
	generate
		for (g_fa = 0; g_fa < NWARPS; g_fa = g_fa + 1) begin : g_faddr
			assign w_faddr_dirty[g_fa] = |w_faddr_dirty_r[g_fa*2 +: 2];

			/* +8 incrementer from registered w_f_addr_r (chunked 8b + AND-prefix, <= 8 cmos3) */
			wire [28:0] fa_x = w_f_addr_r[g_fa*32 + 3 +: 29];
			wire [7:0]  p8_s0, p8_s1, p8_s2;
			wire [4:0]  p8_s3;
			wire [3:0]  p8_co_unused;
			am_add #(.W(8)) u_p8_0 (.a(fa_x[7:0]),   .b(8'd0), .ci(1'b1), .s(p8_s0), .co(p8_co_unused[0]));
			am_add #(.W(8)) u_p8_1 (.a(fa_x[15:8]),  .b(8'd0), .ci(1'b1), .s(p8_s1), .co(p8_co_unused[1]));
			am_add #(.W(8)) u_p8_2 (.a(fa_x[23:16]), .b(8'd0), .ci(1'b1), .s(p8_s2), .co(p8_co_unused[2]));
			am_add #(.W(5)) u_p8_3 (.a(fa_x[28:24]), .b(5'd0), .ci(1'b1), .s(p8_s3), .co(p8_co_unused[3]));
			wire p8_c8  = &fa_x[7:0];
			wire p8_c16 = p8_c8 && (&fa_x[15:8]);
			wire p8_c24 = p8_c16 && (&fa_x[23:16]);
			wire [31:0] w_f_addr_p8 = {
				p8_c24 ? p8_s3 : fa_x[28:24],
				p8_c16 ? p8_s2 : fa_x[23:16],
				p8_c8  ? p8_s1 : fa_x[15:8],
				p8_s0,
				w_f_addr_r[g_fa*32 +: 3]
			};

			/* 2-cycle 16b + 16b base + (pc << 3) adder for branch/warp-control updates */
			/* 2-cycle chunked carry-select fetch-address compute (<= 11 cmos3):
			 *   dirty=2: [10:0]  computed via 4+4 carry-select adders ([2:0]=base[2:0]) + cy
			 *   dirty=1: [31:11] computed via 4b+1b mid + 4b carry-select hi0 + 6+6 inc
			 */
			wire [3:0] fa_l0_s, fa_l1_s0, fa_l1_s1;
			wire       fa_l0_c, fa_l1_c0, fa_l1_c1;
			am_add #(.W(4)) u_fa_l0   (.a(w_base_r[g_fa*32 + 3 +: 4]), .b(pc[g_fa*16 +: 4]),     .ci(1'b0), .s(fa_l0_s),  .co(fa_l0_c));
			am_add #(.W(4)) u_fa_l1_0 (.a(w_base_r[g_fa*32 + 7 +: 4]), .b(pc[g_fa*16 + 4 +: 4]), .ci(1'b0), .s(fa_l1_s0), .co(fa_l1_c0));
			am_add #(.W(4)) u_fa_l1_1 (.a(w_base_r[g_fa*32 + 7 +: 4]), .b(pc[g_fa*16 + 4 +: 4]), .ci(1'b1), .s(fa_l1_s1), .co(fa_l1_c1));
			wire [10:0] fa_lo11_s  = {fa_l0_c ? fa_l1_s1 : fa_l1_s0, fa_l0_s, w_base_r[g_fa*32 +: 3]};
			wire        fa_lo11_co = fa_l0_c ? fa_l1_c1 : fa_l1_c0;

			wire [3:0] fa_m0_s;
			wire       fa_m0_co;
			am_add #(.W(4)) u_fa_m0 (.a(w_base_r[g_fa*32 + 11 +: 4]), .b(pc[g_fa*16 + 8 +: 4]), .ci(w_fa_cy_r[g_fa]), .s(fa_m0_s), .co(fa_m0_co));
			wire fa_b15_a  = w_base_r[g_fa*32 + 15];
			wire fa_b15_b  = pc[g_fa*16 + 12];
			wire fa_b15_s  = fa_b15_a ^ fa_b15_b ^ fa_m0_co;
			wire fa_b15_co = (fa_b15_a & fa_b15_b) | ((fa_b15_a ^ fa_b15_b) & fa_m0_co);

			wire [3:0] fa_hi0_s0, fa_hi0_s1;
			wire       fa_hi0_c0, fa_hi0_c1;
			wire [5:0] fa_hi1_lo, fa_hi1_hi;
			wire       fa_hi1_lo_co;
			am_add #(.W(4)) u_fa_hi0_0  (.a(w_base_r[g_fa*32 + 16 +: 4]), .b({1'b0, pc[g_fa*16 + 13 +: 3]}), .ci(1'b0), .s(fa_hi0_s0), .co(fa_hi0_c0));
			am_add #(.W(4)) u_fa_hi0_1  (.a(w_base_r[g_fa*32 + 16 +: 4]), .b({1'b0, pc[g_fa*16 + 13 +: 3]}), .ci(1'b1), .s(fa_hi0_s1), .co(fa_hi0_c1));
			am_add #(.W(6)) u_fa_hi1_lo (.a(w_base_r[g_fa*32 + 20 +: 6]), .b(6'd0), .ci(1'b1), .s(fa_hi1_lo), .co(fa_hi1_lo_co));
			am_add #(.W(6)) u_fa_hi1_hi (.a(w_base_r[g_fa*32 + 26 +: 6]), .b(6'd0), .ci(1'b1), .s(fa_hi1_hi), .co(w_fa_co_unused[g_fa]));
			wire [3:0]  fa_hi0_s  = fa_b15_co ? fa_hi0_s1 : fa_hi0_s0;
			wire        fa_hi0_co = fa_b15_co ? fa_hi0_c1 : fa_hi0_c0;
			wire [11:0] fa_hi_p1  = {(&w_base_r[g_fa*32 + 20 +: 6]) ? fa_hi1_hi : w_base_r[g_fa*32 + 26 +: 6], fa_hi1_lo};
			wire [15:0] fa_hi_s   = {fa_hi0_co ? fa_hi_p1 : w_base_r[g_fa*32 + 20 +: 12], fa_hi0_s};
			wire [4:0]  fa_hi5_s  = {fa_b15_s, fa_m0_s};
			wire unused_p8_co = &{1'b0, p8_co_unused, fa_hi1_lo_co};

			wire fa_fetch_raw = (st[g_fa*3 +: 3] == S_FETCH) && fb_valid && (fb_warp == g_fa[WARP_W-1:0]) && (fb_kind == 2'd0);
			wire fa_wc_raw    = (st[g_fa*3 +: 3] == S_WC) && wc_valid && (wc_warp == g_fa[WARP_W-1:0]);
			wire fa_cs_go     = cs_launch_fa && (free_slot_r_fa == g_fa[WARP_W-1:0]);
			wire fa_fl_go     = (g_fa < (NWARPS/2) ? fl_g_fa_lo : fl_g_fa_hi) && (free_slot_r_fa == g_fa[WARP_W-1:0]);
			wire [1:0] fa_d_cur = w_faddr_dirty_r[g_fa*2 +: 2];

			wire fa_any_raw = (cs_launch_fa || (g_fa < (NWARPS/2) ? any_fl_vl_lo : any_fl_vl_hi)) && (free_slot_r_fa == g_fa[WARP_W-1:0]);
			wire [31:0] fa_go_base;
			wire fa_any_b0, fa_any_b1, fa_any_f0, fa_any_f0b, fa_any_f1, fa_any_ctl;
			wire fa_fetch_f0, fa_fetch_f0b, fa_fetch_f1;
			wire fa_cs_0, fa_cs_1, fa_cs_2, fa_cs_3;
			wire fa_fl_0, fa_fl_1, fa_fl_2, fa_fl_3;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_b0  (.I0(fa_any_raw), .O(fa_any_b0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_b1  (.I0(fa_any_raw), .O(fa_any_b1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_f0  (.I0(fa_any_raw), .O(fa_any_f0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_f0b (.I0(fa_any_raw), .O(fa_any_f0b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_f1  (.I0(fa_any_raw), .O(fa_any_f1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_ctl (.I0(fa_any_raw), .O(fa_any_ctl));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_ft0 (.I0(fa_fetch_raw), .O(fa_fetch_f0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_ft0b(.I0(fa_fetch_raw), .O(fa_fetch_f0b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fa_ft1 (.I0(fa_fetch_raw), .O(fa_fetch_f1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fcs0   (.I0(fa_cs_go),   .O(fa_cs_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fcs1   (.I0(fa_cs_go),   .O(fa_cs_1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fcs2   (.I0(fa_cs_go),   .O(fa_cs_2));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fcs3   (.I0(fa_cs_go),   .O(fa_cs_3));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ffl0   (.I0(fa_fl_go),   .O(fa_fl_0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ffl1   (.I0(fa_fl_go),   .O(fa_fl_1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ffl2   (.I0(fa_fl_go),   .O(fa_fl_2));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ffl3   (.I0(fa_fl_go),   .O(fa_fl_3));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign fa_any_b0 = fa_any_raw;
			assign fa_any_b1 = fa_any_raw;
			assign fa_any_f0 = fa_any_raw;
			assign fa_any_f0b = fa_any_raw;
			assign fa_any_f1 = fa_any_raw;
			assign fa_any_ctl = fa_any_raw;
			assign fa_fetch_f0 = fa_fetch_raw;
			assign fa_fetch_f0b = fa_fetch_raw;
			assign fa_fetch_f1 = fa_fetch_raw;
			assign fa_cs_0 = fa_cs_go;
			assign fa_cs_1 = fa_cs_go;
			assign fa_cs_2 = fa_cs_go;
			assign fa_cs_3 = fa_cs_go;
			assign fa_fl_0 = fa_fl_go;
			assign fa_fl_1 = fa_fl_go;
			assign fa_fl_2 = fa_fl_go;
			assign fa_fl_3 = fa_fl_go;
`else
			assign fa_any_b0  = fa_any_raw; assign fa_any_b1 = fa_any_raw;
			assign fa_any_f0  = fa_any_raw; assign fa_any_f0b = fa_any_raw; assign fa_any_f1 = fa_any_raw;
			assign fa_any_ctl = fa_any_raw;
			assign fa_fetch_f0 = fa_fetch_raw; assign fa_fetch_f0b = fa_fetch_raw; assign fa_fetch_f1 = fa_fetch_raw;
			assign fa_cs_0 = fa_cs_go; assign fa_cs_1 = fa_cs_go;
			assign fa_cs_2 = fa_cs_go; assign fa_cs_3 = fa_cs_go;
			assign fa_fl_0 = fa_fl_go; assign fa_fl_1 = fa_fl_go;
			assign fa_fl_2 = fa_fl_go; assign fa_fl_3 = fa_fl_go;
`endif
			assign fa_go_base[7:0]   = fa_cs_0 ? cs_base[7:0]   : (fa_fl_0 ? fs_base[7:0]   : vs_base[7:0]);
			assign fa_go_base[15:8]  = fa_cs_1 ? cs_base[15:8]  : (fa_fl_1 ? fs_base[15:8]  : vs_base[15:8]);
			assign fa_go_base[23:16] = fa_cs_2 ? cs_base[23:16] : (fa_fl_2 ? fs_base[23:16] : vs_base[23:16]);
			assign fa_go_base[31:24] = fa_cs_3 ? cs_base[31:24] : (fa_fl_3 ? fs_base[31:24] : vs_base[31:24]);

			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					w_f_addr_r[g_fa*32 +: 32]     <= 32'd0;
					w_f_addr_p8_r[g_fa*32 +: 32]  <= 32'd0;
					w_base_r[g_fa*32 +: 32]       <= 32'd0;
					w_faddr_dirty_r[g_fa*2 +: 2]  <= 2'd0;
					w_fa_cy_r[g_fa]               <= 1'b0;
				end else begin
					w_f_addr_p8_r[g_fa*32 +: 32] <= w_f_addr_p8;
					if (fa_any_b0) w_base_r[g_fa*32 +: 16]      <= fa_go_base[15:0];
					if (fa_any_b1) w_base_r[g_fa*32 + 16 +: 16] <= fa_go_base[31:16];
					if (fa_any_f0)  w_f_addr_r[g_fa*32 +: 11]     <= fa_go_base[10:0];
					else if (!fa_wc_raw && (fa_d_cur == 2'd2))
						w_f_addr_r[g_fa*32 +: 11] <= fa_lo11_s;
					else if (!fa_any_ctl && !fa_wc_raw && (fa_d_cur != 2'd2) && (fa_d_cur != 2'd1) && fa_fetch_f0)
						w_f_addr_r[g_fa*32 +: 11] <= w_f_addr_p8_r[g_fa*32 +: 11];
					if (fa_any_f0b) w_f_addr_r[g_fa*32 + 11 +: 5] <= fa_go_base[15:11];
					else if (!fa_wc_raw && (fa_d_cur == 2'd1))
						w_f_addr_r[g_fa*32 + 11 +: 5] <= fa_hi5_s;
					else if (!fa_any_ctl && !fa_wc_raw && (fa_d_cur != 2'd2) && (fa_d_cur != 2'd1) && fa_fetch_f0b)
						w_f_addr_r[g_fa*32 + 11 +: 5] <= w_f_addr_p8_r[g_fa*32 + 11 +: 5];
					if (fa_any_f1)  w_f_addr_r[g_fa*32 + 16 +: 16] <= fa_go_base[31:16];
					else if (!fa_wc_raw && (fa_d_cur == 2'd1))
						w_f_addr_r[g_fa*32 + 16 +: 16] <= fa_hi_s;
					else if (!fa_any_ctl && !fa_wc_raw && (fa_d_cur != 2'd2) && (fa_d_cur != 2'd1) && fa_fetch_f1)
						w_f_addr_r[g_fa*32 + 16 +: 16] <= w_f_addr_p8_r[g_fa*32 + 16 +: 16];
					if (fa_any_ctl) begin
						w_faddr_dirty_r[g_fa*2 +: 2] <= 2'd0;
					end else if (fa_wc_raw) begin
						w_faddr_dirty_r[g_fa*2 +: 2] <= 2'd2;
					end else if (fa_d_cur == 2'd2) begin
						w_fa_cy_r[g_fa]              <= fa_lo11_co;
						w_faddr_dirty_r[g_fa*2 +: 2] <= 2'd1;
					end else if (fa_d_cur == 2'd1) begin
						w_faddr_dirty_r[g_fa*2 +: 2] <= 2'd0;
					end
				end
			end
		end
	endgenerate

	wire [WARP_W-1:0] fsel_out, fsel_pc, fsel_fa_lo, fsel_fa_hi, fsel_st;
	wire fsel_v_out, fsel_v_st;
`ifdef SYNTH_KINTEX7
	genvar g_fsb;
	for (g_fsb = 0; g_fsb < WARP_W; g_fsb = g_fsb + 1) begin : g_fsel_buf
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fso (.I0(fsel[g_fsb]), .O(fsel_out[g_fsb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fsp (.I0(fsel[g_fsb]), .O(fsel_pc[g_fsb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fsl (.I0(fsel[g_fsb]), .O(fsel_fa_lo[g_fsb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fsh (.I0(fsel[g_fsb]), .O(fsel_fa_hi[g_fsb]));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fss (.I0(fsel[g_fsb]), .O(fsel_st[g_fsb]));
	end
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fsv_o (.I0(fsel_v), .O(fsel_v_out));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fsv_s (.I0(fsel_v), .O(fsel_v_st));
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_fsb;
	for (g_fsb = 0; g_fsb < WARP_W; g_fsb = g_fsb + 1) begin : g_fsel_buf
		assign fsel_out[g_fsb] = fsel[g_fsb];
		assign fsel_pc[g_fsb] = fsel[g_fsb];
		assign fsel_fa_lo[g_fsb] = fsel[g_fsb];
		assign fsel_fa_hi[g_fsb] = fsel[g_fsb];
		assign fsel_st[g_fsb] = fsel[g_fsb];
	end
	assign fsel_v_out = fsel_v;
	assign fsel_v_st = fsel_v;
`else
	assign fsel_out   = fsel;
	assign fsel_pc    = fsel;
	assign fsel_fa_lo = fsel;
	assign fsel_fa_hi = fsel;
	assign fsel_st    = fsel;
	assign fsel_v_out = fsel_v;
	assign fsel_v_st  = fsel_v;
`endif

	wire [NWARPS-1:0] fix_mux_pc, fix_mux_fa0, fix_mux_fa1;
	wire [NWARPS-1:0] cand_sel_pc, cand_sel_fa0, cand_sel_fa1;
`ifdef SYNTH_KINTEX7
	genvar gfmx;
	generate
		for (gfmx = 0; gfmx < NWARPS; gfmx = gfmx + 1) begin : g_fmux_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fmx_p (.I0(fix_mux_oh[gfmx]),  .O(fix_mux_pc[gfmx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fmx_0 (.I0(fix_mux_oh[gfmx]),  .O(fix_mux_fa0[gfmx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_fmx_1 (.I0(fix_mux_oh[gfmx]),  .O(fix_mux_fa1[gfmx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csx_p (.I0(cand_sel_oh[gfmx]), .O(cand_sel_pc[gfmx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csx_0 (.I0(cand_sel_oh[gfmx]), .O(cand_sel_fa0[gfmx]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csx_1 (.I0(cand_sel_oh[gfmx]), .O(cand_sel_fa1[gfmx]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar gfmx;
	generate
		for (gfmx = 0; gfmx < NWARPS; gfmx = gfmx + 1) begin : g_fmux_buf
			assign fix_mux_pc[gfmx] = fix_mux_oh[gfmx];
			assign fix_mux_fa0[gfmx] = fix_mux_oh[gfmx];
			assign fix_mux_fa1[gfmx] = fix_mux_oh[gfmx];
			assign cand_sel_pc[gfmx] = cand_sel_oh[gfmx];
			assign cand_sel_fa0[gfmx] = cand_sel_oh[gfmx];
			assign cand_sel_fa1[gfmx] = cand_sel_oh[gfmx];
		end
	endgenerate
`else
	assign fix_mux_pc   = fix_mux_oh;
	assign fix_mux_fa0  = fix_mux_oh;
	assign fix_mux_fa1  = fix_mux_oh;
	assign cand_sel_pc  = cand_sel_oh;
	assign cand_sel_fa0 = cand_sel_oh;
	assign cand_sel_fa1 = cand_sel_oh;
`endif

	assign f_valid = fsel_v_out;
	assign f_warp = fsel_out;
	reg [15:0] fix_pc_oh, cand_pc_oh;
	reg [31:0] fix_addr_oh, cand_addr_oh;
	integer gfo;
	always @(*) begin
		fix_pc_oh    = 16'd0;
		cand_pc_oh   = 16'd0;
		fix_addr_oh  = 32'd0;
		cand_addr_oh = 32'd0;
		for (gfo = 0; gfo < NWARPS; gfo = gfo + 1) begin
			if (fix_mux_pc[gfo])
				fix_pc_oh = fix_pc_oh | pc[gfo*16 +: 16];
			if (fix_mux_fa0[gfo])
				fix_addr_oh[15:0] = fix_addr_oh[15:0] | w_f_addr_r[gfo*32 +: 16];
			if (fix_mux_fa1[gfo])
				fix_addr_oh[31:16] = fix_addr_oh[31:16] | w_f_addr_r[gfo*32 + 16 +: 16];
			if (cand_sel_pc[gfo])
				cand_pc_oh = cand_pc_oh | pc[gfo*16 +: 16];
			if (cand_sel_fa0[gfo])
				cand_addr_oh[15:0] = cand_addr_oh[15:0] | w_f_addr_r[gfo*32 +: 16];
			if (cand_sel_fa1[gfo])
				cand_addr_oh[31:16] = cand_addr_oh[31:16] | w_f_addr_r[gfo*32 + 16 +: 16];
		end
	end
	assign f_pc         = ufm_pc  ? fix_pc_oh         : cand_pc_oh;
	assign f_addr[15:0] = ufm_fa0 ? fix_addr_oh[15:0] : cand_addr_oh[15:0];
	assign f_addr[31:16]= ufm_fa1 ? fix_addr_oh[31:16]: cand_addr_oh[31:16];
	wire unused_fsel_mux = &{1'b0, fsel_pc, fsel_fa_lo, fsel_fa_hi};

	/* Per slot: the gap and pc counted down / up; rr, rot, wk and the tag + 1. */
	/* verilator lint_off WIDTHTRUNC */
	localparam [GAP_W-1:0] GAP_INIT = (G > 0) ? (G - 1) : 0;
	localparam [31:0] NSU_M1 = (NSU > 0) ? (NSU - 1) : 0;
	/* verilator lint_on WIDTHTRUNC */
	wire [NWARPS*GAP_W-1:0] gap_m1;
	wire [NWARPS*16-1:0] pc_p1;
	genvar gw;
	generate
		for (gw = 0; gw < NWARPS; gw = gw + 1) begin : g_w
			wire p_co_unused;
			wire [2:0] g_cur = gap[gw*GAP_W +: GAP_W];
			assign gap_m1[gw*GAP_W +: GAP_W] = {
				g_cur[2] ^ (~|g_cur[1:0]),
				g_cur[1] ^ (~g_cur[0]),
				~g_cur[0]
			};
			am_add #(.W(16)) u_pc (.a(pc[gw*16 +: 16]), .b(16'd0), .ci(1'b1), .s(pc_p1[gw*16 +: 16]),
				.co(p_co_unused));
		end
	endgenerate
	wire [WARP_W-1:0] rr_next;
	generate
		if (WARP_W == 2) begin : g_rr2
			assign rr_next = {fsel[1] ^ fsel[0], ~fsel[0]};
		end else begin : g_rr_gen
			wire rr_co_unused;
			am_add #(.W(WARP_W)) u_rr (.a(fsel), .b({WARP_W{1'b0}}), .ci(1'b1), .s(rr_next), .co(rr_co_unused));
		end
	endgenerate
	wire [8:0] rot1;
	wire rot_co_unused;
	am_add #(.W(9)) u_rot (.a({1'b0, rot}), .b(9'd0), .ci(1'b1), .s(rot1), .co(rot_co_unused));
	reg rot_last_r;
	wire rot_last = rot_last_r;
	wire [3:0] wk1  = {wk[3] ^ (&wk[2:0]), wk[2] ^ (&wk[1:0]), wk[1] ^ wk[0], ~wk[0]};
	wire [3:0] tag1 = {wg_tag[3] ^ (&wg_tag[2:0]), wg_tag[2] ^ (&wg_tag[1:0]), wg_tag[1] ^ wg_tag[0], ~wg_tag[0]};

	/* Barrier: every warp of the workgroup is parked (level-pinned <= 3 LUTs). */
	(* keep = "true" *) wire [NWARPS-1:0] w_bar_blk;
	generate
		for (gc = 0; gc < NWARPS; gc = gc + 1) begin : g_bar_blk
			assign w_bar_blk[gc] = (info[gc*160 + ATG_WI_KIND +: 2] == K_COMP) &&
				!st_free_r_bar[gc] && (st[gc*3 +: 3] != S_BAR) && (st[gc*3 +: 3] != S_ENDED);
		end
	endgenerate
	reg [NWARPS-1:0] bar_go;
	integer a, b;
	always @(*) begin
		for (a = 0; a < NWARPS; a = a + 1) begin
			bar_go[a] = st[(a)*3 +: 3] == S_BAR;
			for (b = 0; b < NWARPS; b = b + 1)
				if (b != a && (tag[(b)*4 +: 4] == tag[(a)*4 +: 4]) && w_bar_blk[b])
					bar_go[a] = 1'b0;
		end
	end

	assign busy = cs_act || launching || (!(&slot_free));
	assign cs_is_mesh_act = cs_is_mesh;

	wire [159:0] wi_vl = wi(48'd0, 48'd0, 8'd0, K_VERT, 1'b0, 16'd0, vl_mask);
	wire [159:0] wi_fl = wi(48'd0, 48'd0, 8'd0, K_FRAG, fl_info[48], fl_info[47:32], fl_info[31:0]);
	wire [159:0] wi_cs = wi({id_z, id_y, id_x}, {grid_z, grid_y, grid_x}, {wk[2:0], 5'd0}, K_COMP, 1'b0, 16'd0, cmask);
	wire vl_go_info_lo, vl_go_info_hi, fl_go_info_lo, fl_go_info_hi;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlgil (.I0(vl_go), .O(vl_go_info_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlgih (.I0(vl_go), .O(vl_go_info_hi));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flgil (.I0(fl_go), .O(fl_go_info_lo));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flgih (.I0(fl_go), .O(fl_go_info_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign vl_go_info_lo = vl_go;
	assign vl_go_info_hi = vl_go;
	assign fl_go_info_lo = fl_go;
	assign fl_go_info_hi = fl_go;
`else
	assign vl_go_info_lo = vl_go; assign vl_go_info_hi = vl_go;
	assign fl_go_info_lo = fl_go; assign fl_go_info_hi = fl_go;
`endif
	genvar gwi, gws;
	generate
	for (gwi = 0; gwi < NWARPS; gwi = gwi + 1) begin : g_info_w
		wire [9:0] vl_we_g;
		wire [9:0] fl_we_g;
		wire [9:0] cs_we_g;
		wire vl_go_i = (gwi < (NWARPS / 2)) ? vl_go_info_lo : vl_go_info_hi;
		wire fl_go_i = (gwi < (NWARPS / 2)) ? fl_go_info_lo : fl_go_info_hi;
		genvar gwe;
		for (gwe = 0; gwe < 10; gwe = gwe + 1) begin : g_we_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_vl (.I0(vl_go_i && (vl_s == gwi[WARP_W-1:0])), .O(vl_we_g[gwe]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_fl (.I0(fl_go_i && (fl_s == gwi[WARP_W-1:0])), .O(fl_we_g[gwe]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_cs (.I0(cs_launch_fa && (free_slot_r_fa == gwi[WARP_W-1:0])), .O(cs_we_g[gwe]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign vl_we_g[gwe] = vl_go_i && (vl_s == gwi[WARP_W-1:0]);
			assign fl_we_g[gwe] = fl_go_i && (fl_s == gwi[WARP_W-1:0]);
			assign cs_we_g[gwe] = cs_launch_fa && (free_slot_r_fa == gwi[WARP_W-1:0]);
`else
			assign vl_we_g[gwe] = vl_go_i && (vl_s == gwi[WARP_W-1:0]);
			assign fl_we_g[gwe] = fl_go_i && (fl_s == gwi[WARP_W-1:0]);
			assign cs_we_g[gwe] = cs_launch_fa && (free_slot_r_fa == gwi[WARP_W-1:0]);
`endif
		end
		for (gws = 0; gws < 20; gws = gws + 1) begin : g_info_sl
			wire vl_we_s = vl_we_g[gws / 2];
			wire fl_we_s = fl_we_g[gws / 2];
			wire cs_we_s = cs_we_g[gws / 2];
			(* shreg_extract = "no" *) reg [7:0] info_sl_r;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					info_sl_r <= 8'd0;
				else if (cs_we_s)
					info_sl_r <= wi_cs[gws*8 +: 8];
				else if (fl_we_s)
					info_sl_r <= wi_fl[gws*8 +: 8];
				else if (vl_we_s)
					info_sl_r <= wi_vl[gws*8 +: 8];
			end
			assign info[gwi*160 + gws*8 +: 8] = info_sl_r;
		end
	end
	endgenerate

	/* Level-pinned state transitions per warp (direct rdy/rof bypasses 3-bit equality) */
	wire [NWARPS-1:0] s_rdy_free;
	wire [NWARPS-1:0] s_alloc_free;
	wire [NWARPS*3-1:0] nxt_st_nonfree;
	wire [NWARPS-1:0] nf_rdy_v, nf_rof_v;
	wire [2:0] wc_nxt_op_w = (wc_op == 2'd3) ? S_FREE : (wc_op == 2'd1) ? S_ENDED : (wc_op == 2'd2) ? S_BAR : S_READY;
	wire       wc_op_is_0  = (wc_op == 2'd0);
	wire       wc_op_is_3  = (wc_op == 2'd3);
	wire [NWARPS-1:0] wc_mine_free_v;
	wire f_ready_0, f_ready_1, f_ready_ctl;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frdy0 (.I0(f_ready), .O(f_ready_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frdy1 (.I0(f_ready), .O(f_ready_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_frdyc (.I0(f_ready), .O(f_ready_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign f_ready_0 = f_ready;
	assign f_ready_1 = f_ready;
	assign f_ready_ctl = f_ready;
`else
	assign f_ready_0   = f_ready;
	assign f_ready_1   = f_ready;
	assign f_ready_ctl = f_ready;
`endif
	wire vl_go_st, fl_go_st, vl_go_ctl, fl_go_ctl;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlgst  (.I0(vl_go), .O(vl_go_st));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flgst  (.I0(fl_go), .O(fl_go_st));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_vlgctl (.I0(vl_go), .O(vl_go_ctl));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_flgctl (.I0(fl_go), .O(fl_go_ctl));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign vl_go_st = vl_go;
	assign fl_go_st = fl_go;
	assign vl_go_ctl = vl_go;
	assign fl_go_ctl = fl_go;
`else
	assign vl_go_st  = vl_go;
	assign fl_go_st  = fl_go;
	assign vl_go_ctl = vl_go;
	assign fl_go_ctl = fl_go;
`endif
	generate
		for (gc = 0; gc < NWARPS; gc = gc + 1) begin : g_nxt_st
			assign s_rdy_free[gc] = (cs_launch && (free_slot_r_st == gc[WARP_W-1:0])) ||
				(vl_go_st && (vl_s == gc[WARP_W-1:0])) || (fl_go_st && (fl_s == gc[WARP_W-1:0]));
			assign s_alloc_free[gc] = !s_rdy_free[gc] && (fl_g || vl_g) && (free_slot_r_st == gc[WARP_W-1:0]);

			wire [2:0] cur_st = st[gc*3 +: 3];
			wire is_vl_or_fl = (vl_go_st && vl_s == gc[WARP_W-1:0]) || (fl_go_st && fl_s == gc[WARP_W-1:0]);
			wire is_fb_mine  = fb_valid && (fb_warp == gc[WARP_W-1:0]);
			wire is_wc_mine  = wc_valid && (wc_warp == gc[WARP_W-1:0]);
			assign wc_mine_free_v[gc] = is_wc_mine && wc_op_is_3;
			wire rdy_from_other =
				((cur_st == S_ALLOC)  && is_vl_or_fl) ||
				((cur_st == S_FETCH)  && is_fb_mine && ((fb_kind == 2'd0) || ((fb_kind != 2'd1) && wake[gc]))) ||
				((cur_st == S_REPLAY) && wake[gc]) ||
				((cur_st == S_WC)     && is_wc_mine && wc_op_is_0) ||
				((cur_st == S_BAR)    && bar_go[gc]);
			assign nf_rdy_v[gc] = st_rdy_r[gc] ? !((gc < (NWARPS/2) ? f_ready_0 : f_ready_1) && fsel_oh[gc]) : rdy_from_other;
			assign nf_rof_v[gc] = st_rdy_r[gc] || ((cur_st == S_FETCH) && !is_fb_mine) || rdy_from_other;

			reg [2:0] nf_st_nonrdy;
			always @(*) begin
				nf_st_nonrdy = cur_st;
				case (cur_st)
				S_ALLOC:
					if (is_vl_or_fl)
						nf_st_nonrdy = S_READY;
				S_FETCH:
					if (is_fb_mine) begin
						if (fb_kind == 2'd0)
							nf_st_nonrdy = S_READY;
						else if (fb_kind == 2'd1)
							nf_st_nonrdy = S_WC;
						else
							nf_st_nonrdy = wake[gc] ? S_READY : S_REPLAY;
					end
				S_REPLAY:
					if (wake[gc])
						nf_st_nonrdy = S_READY;
				S_WC:
					if (is_wc_mine)
						nf_st_nonrdy = wc_nxt_op_w;
				S_BAR:
					if (bar_go[gc])
						nf_st_nonrdy = S_READY;
				S_ENDED:
					if (xr_valid && xr_warp == gc[WARP_W-1:0])
						nf_st_nonrdy = S_FREE;
				default: ;
				endcase
			end
			wire f_go_gc = (gc < (NWARPS/2) ? f_ready_0 : f_ready_1) && fsel_oh[gc];
			assign nxt_st_nonfree[gc*3 + 0] = st_rdy_r[gc] ? f_go_gc : nf_st_nonrdy[0];
			assign nxt_st_nonfree[gc*3 + 1] = st_rdy_r[gc] ? 1'b1    : nf_st_nonrdy[1];
			assign nxt_st_nonfree[gc*3 + 2] = st_rdy_r[gc] ? 1'b0    : nf_st_nonrdy[2];
		end
	endgenerate

	wire cs_adv_id_raw = cs_skip || (cs_launch && (wk1 == nw));
	wire cs_adv_id_x;
	wire cs_adv_id_y;
	wire cs_adv_id_z;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csax (.I0(cs_adv_id_raw), .O(cs_adv_id_x));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csay (.I0(cs_adv_id_raw), .O(cs_adv_id_y));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_csaz (.I0(cs_adv_id_raw), .O(cs_adv_id_z));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cs_adv_id_x = cs_adv_id_raw;
	assign cs_adv_id_y = cs_adv_id_raw;
	assign cs_adv_id_z = cs_adv_id_raw;
`else
	assign cs_adv_id_x = cs_adv_id_raw;
	assign cs_adv_id_y = cs_adv_id_raw;
	assign cs_adv_id_z = cs_adv_id_raw;
`endif

	integer w;
	reg [2:0] nxt_st_w;
	reg nxt_free_w;
	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			st <= {NWARPS{S_FREE}};
			st_free_r     <= {NWARPS{1'b1}};
			st_free_r_out <= {NWARPS{1'b1}};
			st_free_r_pc  <= {NWARPS{1'b1}};
			st_free_r_bar <= {NWARPS{1'b1}};
			st_rdy_r  <= {NWARPS{1'b0}};
			st_rof_r  <= {NWARPS{1'b0}};
			gap <= {(NWARPS*GAP_W){1'b0}};
			gap_z_r <= {NWARPS{1'b1}};
			tag <= {(NWARPS*4){1'b0}};
			{rr, hold_v, hold_w} <= {(2*WARP_W+1){1'b0}};
			rr_ge_r <= {NWARPS{1'b1}};
			hold_oh <= {{(NWARPS-1){1'b0}}, 1'b1};
			last_warp <= {WARP_W{1'b0}};
			last_warp_oh <= {{(NWARPS-1){1'b0}}, 1'b1};
			active_valid <= 1'b0;
			{vl_wait, fl_wait, vl_s, fl_s} <= {(2*WARP_W+2){1'b0}};
			{vl_gnt_r, fl_gnt_r} <= 2'b00;
			{cs_act, cs_is_mesh, launching} <= 3'b000;
			{launching_vg, launching_st, launching_fa} <= 3'b000;
			{id_x, id_y, id_z} <= 48'd0;
			rot <= 8'd0;
			rot_last_r <= (NSU_M1 == 32'd0);
			mine_wg_r <= (SU_INDEX == 32'd0);
			wg_tag <= 4'd0;
			wk <= 4'd0;
		end else begin
			for (w = 0; w < NWARPS; w = w + 1) begin
				if (fsel_v_st && f_ready_ctl && fsel_oh[w]) begin
					gap[w*GAP_W +: GAP_W] <= GAP_INIT;
					gap_z_r[w] <= (GAP_INIT == {GAP_W{1'b0}});
				end else if (!gap_z_r[w]) begin
					gap[(w)*GAP_W +: GAP_W] <= gap_m1[(w)*GAP_W +: GAP_W];
					gap_z_r[w] <= (gap[(w)*GAP_W +: GAP_W] == 3'd1);
				end

				nxt_st_w = st_free_r[w] ? {1'b0, s_rdy_free[w], s_alloc_free[w]} : nxt_st_nonfree[w*3 +: 3];
				nxt_free_w = (st_free_r[w] && !s_rdy_free[w] && !s_alloc_free[w]) ||
					((st[(w)*3 +: 3] == S_WC) && wc_mine_free_v[w]) ||
					((st[(w)*3 +: 3] == S_ENDED) && xr_valid && (xr_warp == w[WARP_W-1:0]));
				st[(w)*3 +: 3] <= nxt_st_w;
				st_free_r[w]     <= nxt_free_w;
				st_free_r_out[w] <= nxt_free_w;
				st_free_r_pc[w]  <= nxt_free_w;
				st_free_r_bar[w] <= nxt_free_w;
				st_rdy_r[w]    <= st_free_r[w] ? s_rdy_free[w] : nf_rdy_v[w];
				st_rof_r[w]    <= st_free_r[w] ? s_rdy_free[w] : nf_rof_v[w];

				if (st_free_r[w])
					tag[(w)*4 +: 4] <= wg_tag;
			end
			hold_v <= fsel_v_st && !f_ready_ctl;
			hold_w <= fsel_st;
			hold_oh <= fsel_oh;
			if (fsel_v_st && f_ready_ctl) begin
				rr <= rr_next;
				rr_ge_r <= next_rr_ge;
				last_warp <= fsel_st;
				last_warp_oh <= fsel_oh;
				active_valid <= 1'b1;
			end
			/* Vertex and fragment slots. */
			vl_gnt_r <= vl_g;
			fl_gnt_r <= fl_g;
			if (vl_g) begin
				vl_s <= free_slot_r;
				vl_wait <= 1'b1;
			end
			if (fl_g) begin
				fl_s <= free_slot_r;
				fl_wait <= 1'b1;
			end
			if (vl_go_ctl)
				vl_wait <= 1'b0;
			if (fl_go_ctl)
				fl_wait <= 1'b0;
			/* Compute. */
			if (cs_kick && !cs_act) begin
				cs_act <= 1'b1;
				cs_is_mesh <= cs_mesh;
				{id_x, id_y, id_z} <= 48'd0;
				rot <= 8'd0;
				rot_last_r <= (NSU_M1 == 32'd0);
				mine_wg_r <= (SU_INDEX == 32'd0);
			end
			if (cs_try) begin
				launching    <= 1'b1;
				launching_vg <= 1'b1;
				launching_st <= 1'b1;
				launching_fa <= 1'b1;
				wk <= 4'd0;
			end
			if (cs_try || cs_skip_ctl) begin
				rot <= rot_last ? 8'd0 : rot1[7:0];
				rot_last_r <= rot_last ? (NSU_M1 == 32'd0) : ({23'd0, rot1} == NSU_M1);
				mine_wg_r <= rot_last ? (SU_INDEX == 32'd0) : ({24'd0, rot1[7:0]} == SU_INDEX);
				if (last_id)
					cs_act <= 1'b0;
			end
			if (cs_adv_id_x) begin
				id_x <= id_x_last ? 16'd0 : id_x1;
			end
			if (cs_adv_id_y && id_x_last_y) begin
				id_y <= id_y_last_y ? 16'd0 : id_y1;
			end
			if (cs_adv_id_z && id_x_last_z && id_y_last_z) begin
				id_z <= id_z1;
			end
			if (cs_launch) begin
				wk <= wk1;
				if (wk1 == nw) begin
					launching    <= 1'b0;
					launching_vg <= 1'b0;
					launching_st <= 1'b0;
					launching_fa <= 1'b0;
					wg_tag <= tag1;
				end
			end
		end
	end

	genvar gwp;
	generate
		for (gwp = 0; gwp < NWARPS; gwp = gwp + 1) begin : g_pc_ctrl
			wire clr_raw   = st_free_r_pc[gwp] || (st[gwp*3 +: 3] == S_ALLOC);
			wire fetch_raw = (st[gwp*3 +: 3] == S_FETCH) && fb_valid && (fb_warp == gwp[WARP_W-1:0]) && (fb_kind == 2'd0);
			wire wc_raw    = (st[gwp*3 +: 3] == S_WC) && wc_valid && (wc_warp == gwp[WARP_W-1:0]);
			wire clr_lo, clr_hi, fetch_lo, fetch_hi, wc_lo, wc_hi;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pclr0 (.I0(clr_raw), .O(clr_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pclr1 (.I0(clr_raw), .O(clr_hi));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pfet0 (.I0(fetch_raw), .O(fetch_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pfet1 (.I0(fetch_raw), .O(fetch_hi));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pwc0  (.I0(wc_raw), .O(wc_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_pwc1  (.I0(wc_raw), .O(wc_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign clr_lo = clr_raw;
			assign clr_hi = clr_raw;
			assign fetch_lo = fetch_raw;
			assign fetch_hi = fetch_raw;
			assign wc_lo = wc_raw;
			assign wc_hi = wc_raw;
`else
			assign clr_lo   = clr_raw;
			assign clr_hi   = clr_raw;
			assign fetch_lo = fetch_raw;
			assign fetch_hi = fetch_raw;
			assign wc_lo    = wc_raw;
			assign wc_hi    = wc_raw;
`endif
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					pc[gwp*16 +: 8]     <= 8'd0;
					pc[gwp*16 + 8 +: 8] <= 8'd0;
				end else begin
					if (clr_lo)
						pc[gwp*16 +: 8] <= 8'd0;
					else if (fetch_lo)
						pc[gwp*16 +: 8] <= pc_p1[gwp*16 +: 8];
					else if (wc_lo)
						pc[gwp*16 +: 8] <= wc_pc[7:0];

					if (clr_hi)
						pc[gwp*16 + 8 +: 8] <= 8'd0;
					else if (fetch_hi)
						pc[gwp*16 + 8 +: 8] <= pc_p1[gwp*16 + 8 +: 8];
					else if (wc_hi)
						pc[gwp*16 + 8 +: 8] <= wc_pc[15:8];
				end
			end
		end
	endgenerate

	wire unused_ok = &{1'b0, fl_info[63:49], wk[3], lanes_r[23:9], rot1[8], slot_rdy_nogap,
		w_fa_co_unused, wdata_m1_bo_unused, ix_co_unused, iy_co_unused, iz_co_unused,
		rot_co_unused, unused_fsel_mux, rr, pt_ok, (NSU != 0)};
endmodule

`default_nettype wire
