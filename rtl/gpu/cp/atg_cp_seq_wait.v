/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_seq_wait: atg_cp_seq's waits, combinational: for the payload
 * dword in hand, whether and where it writes STATE and which busy bits
 * (STATUS order: [1] BLT, [2] VF, [3] RX, [4] ROP, [5] TU, [6] SU) hold
 * it; for the µop, which busy bits it waits for before its act (pre, and
 * pre_f for the fence unit) and after its kick (post).  Bit 0 of the busy
 * bits is the units' memory: requests of the units (and the CPU's DMA)
 * that the MC has not answered (atg_mc's unit_idle).  FENCE, WAIT_IDLE
 * (whatever its flags), RESULT and QUERY wait for it too: a unit's busy
 * may fall before its last response (the TU's) and the CPU's DMA writes
 * are not in a busy bit at all.
 *
 * A block's STATE does not change under its work, nor while work that
 * will read it is still upstream:
 *   BLT range                 the BLT
 *   VF (0x020-0x04F, 0x1C0-0x1DF: ATTR_DST and ATTR_LIMIT)  the VF
 *   RX and ROP ranges         the VF, the SUs, the RX and the ROP: the
 *                             ROP's busy covers only the sequence numbers
 *                             the RX has issued, so a draw still in the VF,
 *                             in a vertex warp or in the RX's setup would
 *                             otherwise see the next draw's state
 *   TU range                  the VF, the SUs, the RX and the TU (texels
 *                             are read by warps a draw upstream starts later)
 *   program range (0x190-0x1AF)  the VF, the SUs and the RX (the SU takes
 *                             VS_BASE, FS_BASE and the rest at a warp's start)
 * LOAD_CONST waits for the VF, the SUs and the RX before its kick (the
 * constants of a draw upstream are read by its warps), and for the SUs
 * after it; DISPATCH and LOAD_CONST payloads wait for the SUs, DRAW_RAW's
 * for the RX.
 *
 * Bit 8 (B_DISP, outside B_ALL and the 8-bit WAIT_IDLE mask) is
 * atg_cp_disp's wait (design §18.4): DISP_SET's register writes wait for
 * its host buffer, FLIP waits for its plane (and with flags[0] COMPLETE
 * for B_ALL first, as FENCE), DISP_SET with flags[0] waits for the
 * controller's idle after its kick.  DISP_SET's pairs go to 0x3C0 +
 * offset / 4 (sidx, set by atg_cp_seq), FLIP's four dwords to 0x3B8-0x3BB.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_seq_wait (
	input wire is_ghci,
	input wire vendor,
	input wire [7:0] op,
	input wire [7:0] flags,
	input wire [7:0] p,
	input wire set_val,
	input wire [9:0] sidx,
	input wire is_draw_raw,
	input wire is_dispatch,
	input wire is_load_const,
	input wire p_lt_3_i,
	input wire p_lt_4_i,
	input wire p_lt_9_i,
	input wire p_eq_0_i,
	input wire is_dbg_mark_i,
	input wire is_flip_i,
	output reg pay_we,
	output reg [9:0] pay_idx,
	output reg [8:0] pay_wait,
	output reg [8:0] pre,
	output reg pre_f,
	output reg [8:0] post
);
	`include "ghci_regs.vh"

	/*
	 * DISPATCH's grid words (0x1E9 is not aligned for an OR of p); RAW
	 * (0x1E0, p < 9) and LC_REGION (0x1EC, p < 3) take p by an OR, their
	 * low bits being 0.
	 */
	localparam [8:0] S_GRID_Y = GHCI_S_GRID_X + 9'd1;
	localparam [8:0] S_GRID_Z = GHCI_S_GRID_X + 9'd2;
	/* verilator lint_off UNUSEDPARAM */
	localparam [7:0] OP_BLIT = 8'd2;
	localparam [7:0] OP_DRAW = 8'd3;
	localparam [7:0] OP_DISPATCH = 8'd4;
	localparam [7:0] OP_FENCE = 8'd5;
	localparam [7:0] OP_WAIT_IDLE = 8'd6;
	localparam [7:0] OP_LOAD_CONST = 8'd7;
	localparam [7:0] OP_RESULT = 8'd13;
	localparam [7:0] OP_QUERY = 8'd14;
	localparam [7:0] OP_DISP_SET = 8'd15;
	localparam [7:0] OP_FLIP = 8'd16;
	localparam [7:0] OP_BARRIER = 8'd20;
	/* verilator lint_on UNUSEDPARAM */
	localparam [7:0] OP_SEMAPHORE_SIGNAL = 8'd21;
	localparam [7:0] OP_SEMAPHORE_WAIT = 8'd22;

	/* GHCI 1.0 Structured Command Opcodes */
	localparam [7:0] CMD_QUERY_CAPS            = 8'h15;
	localparam [7:0] CMD_BIND_PIPELINE         = 8'h20;
	localparam [7:0] CMD_BIND_VERTEX_BUFFERS   = 8'h21;
	localparam [7:0] CMD_BIND_INDEX_BUFFER     = 8'h22;
	localparam [7:0] CMD_BIND_DESCRIPTOR_TABLE = 8'h23;
	localparam [7:0] CMD_SET_VIEWPORT          = 8'h24;
	localparam [7:0] CMD_SET_SCISSOR           = 8'h25;
	localparam [7:0] CMD_SET_DYNAMIC_STATE     = 8'h26;
	localparam [7:0] CMD_DRAW                  = 8'h27;
	localparam [7:0] CMD_DRAW_INDEXED          = 8'h28;
	localparam [7:0] CMD_DRAW_INDIRECT         = 8'h29;
	localparam [7:0] CMD_DRAW_MESH_TASKS       = 8'h2a;
	localparam [7:0] CMD_DISPATCH              = 8'h30;
	localparam [7:0] CMD_BARRIER               = 8'h31;
	localparam [7:0] CMD_EXECUTE_COMMANDS      = 8'h32;
	localparam [7:0] CMD_FENCE                 = 8'h33;

	localparam [7:0] VOP_DRAW_RAW = 8'd0;
	localparam [7:0] VOP_DBG_MARK = 8'd1;

	localparam [7:0] B_BLT = 8'h02;
	localparam [7:0] B_VF = 8'h04;
	localparam [7:0] B_RX = 8'h08;
	localparam [7:0] B_ROP = 8'h10;
	localparam [7:0] B_TU = 8'h20;
	localparam [7:0] B_SU = 8'h40;
`ifdef MUT_SEQ_NOMEM
	localparam [7:0] B_MEM = 8'h00;	/* the earlier rule, for the TB's mutation check */
`else
	localparam [7:0] B_MEM = 8'h01;
`endif
	localparam [7:0] B_ALL = 8'hFE | B_MEM;
	localparam [8:0] B_DISP = 9'h100;
`ifdef MUT_SEQ_DRAWSTATE
	/* the earlier rules, for the TB's mutation check */
	localparam [7:0] B_RXROP = B_RX | B_ROP;
	localparam [7:0] B_ROPW = B_ROP;
	localparam [7:0] B_TUW = B_TU | B_SU | B_RX;
	localparam [7:0] B_PROGW = 8'd0;
	localparam [7:0] B_LCPRE = B_SU;
	localparam VF_1D = 0;	/* 0x1C0-0x1CF only */
`else
	localparam [7:0] B_RXROP = B_VF | B_SU | B_RX | B_ROP;
	localparam [7:0] B_ROPW = B_RXROP;
	localparam [7:0] B_TUW = B_VF | B_SU | B_RX | B_TU;
	localparam [7:0] B_PROGW = B_VF | B_SU | B_RX;
	localparam [7:0] B_LCPRE = B_VF | B_SU | B_RX;
	localparam VF_1D = 1;	/* also ATTR_LIMIT, 0x1D0-0x1DF */
`endif

	wire [8:0] s9 = sidx[8:0];

	wire p_lt_3 = p_lt_3_i;
	wire p_lt_4 = p_lt_4_i;
	wire p_lt_9 = p_lt_9_i;
	wire p_eq_0 = p_eq_0_i;

	wire pay_disp_set = set_val && (sidx[9:8] == 2'b11);
	wire pay_set_std  = set_val && (sidx[9:8] != 2'b11);
	wire pay_raw      = is_draw_raw && p_lt_9;
	wire pay_dbg      = is_dbg_mark_i && p_eq_0;
	wire pay_disp     = is_dispatch && p_lt_3;
	wire pay_lc       = is_load_const && p_lt_3;
	wire pay_flip     = !set_val && is_flip_i && p_lt_4;

	wire [9:0] idx_raw  = {1'b0, GHCI_S_RAW | {5'd0, p[3:0]}};
	wire [9:0] idx_dbg  = {1'b0, GHCI_S_DBG_MARK};
	wire [9:0] idx_disp = {1'b0, p[1:0] == 2'd0 ? GHCI_S_GRID_X : p[1:0] == 2'd1 ? S_GRID_Y : S_GRID_Z};
	wire [9:0] idx_lc   = {1'b0, GHCI_S_LC_REGION | {7'd0, p[1:0]}};
	wire [9:0] idx_flip = {8'b11101110, p[1:0]};

	/* Parallel range checks for s9 (avoids 6-deep comparator chain) */
	wire r_blt  = (s9 < GHCI_S_VF);
	wire r_vf   = (s9 >= GHCI_S_VF)   && (s9 < GHCI_S_RX);
	wire r_rx   = (s9 >= GHCI_S_RX)   && (s9 < GHCI_S_ROP);
	wire r_rop  = (s9 >= GHCI_S_ROP)  && (s9 < GHCI_S_TU);
	wire r_tu   = (s9 >= GHCI_S_TU)   && (s9 < GHCI_S_PROG);
	wire r_prog = (s9 >= GHCI_S_PROG) && (s9 < GHCI_S_SA);
	wire r_attr = (s9 >= GHCI_S_SA)   && (s9[8:5] == GHCI_S_ATTR_DST[8:5]) && (VF_1D != 0 || !s9[4]);
	wire [7:0] wait_set_std = sidx[9] ? B_TUW :
		(({8{r_blt}}  & B_BLT)   |
		 ({8{r_vf}}   & B_VF)    |
		 ({8{r_rx}}   & B_RXROP) |
		 ({8{r_rop}}  & B_ROPW)  |
		 ({8{r_tu}}   & B_TUW)   |
		 ({8{r_prog}} & B_PROGW) |
		 ({8{r_attr}} & B_VF));

	always @(*) begin
		pay_we = set_val || pay_raw || pay_dbg || pay_disp || pay_lc || pay_flip;
		pay_idx =
			({10{pay_disp_set || pay_set_std}} & sidx)     |
			({10{pay_raw}}                     & idx_raw)  |
			({10{pay_dbg}}                     & idx_dbg)  |
			({10{pay_disp}}                    & idx_disp) |
			({10{pay_lc}}                      & idx_lc)   |
			({10{pay_flip}}                    & idx_flip);
		pay_wait =
			({9{pay_disp_set}}                 & B_DISP)              |
			({9{pay_set_std}}                  & {1'b0, wait_set_std})|
			({9{pay_raw}}                      & {1'b0, B_RX})        |
			({9{pay_disp || pay_lc}}           & {1'b0, B_SU});
	end

	wire flags_z = (flags == 8'd0);
	wire [7:0] bar_mask = flags_z ? B_ALL : (flags | B_MEM);

	/* Flat one-hot opcode decodes with is_ghci at final mux to keep pre shallow. */
	wire op_hi_z = !(op[7] | op[6]);
	wire op_ok   = !vendor && op_hi_z;
	wire sop_blit    = op_ok && (op[5:0] == OP_BLIT[5:0]);
	wire sop_draw    = op_ok && (op[5:0] == OP_DRAW[5:0]);
	wire sop_fence   = op_ok && (op[5:0] == OP_FENCE[5:0]);
	wire sop_w_or_b  = op_ok && ((op[5:0] == OP_WAIT_IDLE[5:0]) || (op[5:0] == OP_BARRIER[5:0]));
	wire sop_ssig    = op_ok && (op[5:0] == OP_SEMAPHORE_SIGNAL[5:0]);
	wire sop_swait   = op_ok && (op[5:0] == OP_SEMAPHORE_WAIT[5:0]);
	wire sop_lc      = op_ok && (op[5:0] == OP_LOAD_CONST[5:0]);
	wire sop_res     = op_ok && (op[5:0] == OP_RESULT[5:0]);
	wire sop_qry     = op_ok && (op[5:0] == OP_QUERY[5:0]);
	wire sop_dset    = op_ok && (op[5:0] == OP_DISP_SET[5:0]);
	wire sop_flip    = op_ok && (op[5:0] == OP_FLIP[5:0]);

	wire gop_pipe    = op_ok && (op[5:0] == CMD_BIND_PIPELINE[5:0]);
	wire gop_vb_ib   = op_ok && ((op[5:0] == CMD_BIND_VERTEX_BUFFERS[5:0]) || (op[5:0] == CMD_BIND_INDEX_BUFFER[5:0]));
	wire gop_desc    = op_ok && (op[5:0] == CMD_BIND_DESCRIPTOR_TABLE[5:0]);
	wire gop_rxrop   = op_ok && ((op[5:0] == CMD_SET_VIEWPORT[5:0]) || (op[5:0] == CMD_SET_SCISSOR[5:0]) || (op[5:0] == CMD_SET_DYNAMIC_STATE[5:0]));
	wire gop_draw    = op_ok && ((op[5:0] == CMD_DRAW[5:0]) || (op[5:0] == CMD_DRAW_INDEXED[5:0]) || (op[5:0] == CMD_DRAW_INDIRECT[5:0]));
	wire gop_su      = op_ok && ((op[5:0] == CMD_DISPATCH[5:0]) || (op[5:0] == CMD_DRAW_MESH_TASKS[5:0]));
	wire gop_bar     = op_ok && (op[5:0] == CMD_BARRIER[5:0]);
	wire gop_fence   = op_ok && (op[5:0] == CMD_FENCE[5:0]);

	wire pre_all_std = sop_fence || sop_res || (sop_qry && !flags[0])
`ifndef MUT_DISP_NOWAIT
		|| (sop_flip && flags[0])
`endif
		;
	wire pre_bar_std = sop_w_or_b || sop_ssig;
	wire [7:0] pre_std_lo =
		({8{vendor && op_hi_z && (op[5:0] == VOP_DRAW_RAW[5:0])}} & B_RX) |
		({8{sop_blit}}    & B_BLT) |
		({8{sop_draw}}    & B_VF) |
		({8{sop_lc}}      & B_LCPRE) |
		({8{pre_all_std}} & B_ALL) |
		({8{pre_bar_std}} & bar_mask);
	wire [7:0] pre_ghci_lo =
		({8{gop_vb_ib || gop_draw}} & B_VF) |
		({8{gop_fence}}             & B_ALL) |
		({8{gop_bar}}               & bar_mask) |
		({8{gop_pipe}}              & B_PROGW) |
		({8{gop_desc}}              & B_TUW) |
		({8{gop_rxrop}}             & B_RXROP) |
		({8{gop_su}}                & B_SU);
	wire [7:0] pre_lo = is_ghci ? pre_ghci_lo : pre_std_lo;

	always @(*) begin
		pre   = {!is_ghci && sop_flip, pre_lo};
		pre_f = (!is_ghci && (sop_ssig || sop_swait || sop_res || (sop_qry && !flags[0]))) || (is_ghci && gop_fence);
		post  = (!is_ghci && sop_dset && flags[0]) ? B_DISP :
			{1'b0, ({8{!is_ghci && sop_blit && flags[0]}} & B_BLT) | ({8{!is_ghci && sop_lc}} & B_SU)};
	end
endmodule

`default_nettype wire
