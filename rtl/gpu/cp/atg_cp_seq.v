/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_seq: the class N µop sequencer (design §2.4.3): takes µop
 * packets from the ring or from the firmware (UOP_FIFO), writes STATE,
 * kicks the blocks, waits for idle, and hands fences, results and
 * WAIT_FENCE to atg_cp_fence.
 *
 * Source: fw_* while fw_active (sampled between packets), else n_*.
 * Header [31:30] class (0 standard, 1 vendor), [29:24] op, [23:16] tag,
 * [15:8] flags, [7:0] n.  The payload is taken one dword per cycle:
 * SET_REG writes STATE[index] = value per pair (index >= 0x300: fault 3;
 * a write to a block's range waits while it is busy, whose state must not
 * change under its work: atg_cp_seq_wait has the rules),
 * DISPATCH/LOAD_CONST/DRAW_RAW/DBG_MARK write their payload to the vendor
 * STATE words (DISPATCH and LOAD_CONST wait while the SUs are busy,
 * DRAW_RAW while the RX is), the fence µops keep theirs.  The µop runs:
 *   BLIT: wait for the BLT idle, kick; flags[0]: wait until it is done.
 *   DRAW: wait for the VF idle, inv[1:0] and draw_kick.  DISPATCH: cs_kick.
 *   FENCE: wait until every block is idle, request op 0 (flags[0] = IRQ).
 *   WAIT_IDLE: wait until the blocks of flags (0 = all) are idle.
 *   LOAD_CONST: SUs idle, lc_kick, SUs idle.  LOAD_PROGRAM: inv[3].
 *   WAIT_FENCE: request op 2, wait for !f_busy.  PERF_RESET.  WAIT_VSYNC:
 *   one vsync.  INVALIDATE: inv = flags[3:0].  RESULT: blocks idle and
 *   !f_busy, request op 1.  QUERY (CAPS[30]): as RESULT with the occlusion
 *   counter (occ_count, low word, then the high bits) at addr; flags[0]:
 *   no wait and no write, only occ_clear; flags[1]: occ_clear with the
 *   write's request.  DRAW_RAW: raw_kick.  NOP, SET_REG, DBG_MARK: none.
 *   DISP_SET (HAS_DISP): its pairs write the DISP2 registers through
 *   0x3C0 + offset / 4 (atg_cp_disp); flags[0]: kick K_DISP_IDLE, then
 *   wait for busy bit 8.  FLIP: payload to 0x3B8-0x3BB, waits for busy
 *   bit 8 (the plane; flags[0] also every unit), kick K_FLIP.
 * Kicks are one pulse (kick) with a code (kick_code, K_*): BLIT, DRAW,
 * DISPATCH, LOAD_CONST, DRAW_RAW, PERF_RESET, FLIP, DISP_SET's idle wait.
 * A wait after a kick starts one cycle later (busy rises after the kick).
 * Faults stop the sequencer until the core reset (rst_n): kind 2 for an
 * unknown op, a class 2/3 header, or an op whose unit is absent (UNITS,
 * HAS_VSYNC, HAS_DISP; DISP_SET and FLIP from the firmware); kind 3 for a
 * payload shorter than the op needs, a bad SET_REG index or DISP_SET
 * offset (fault_addr = the dword).  A longer payload is skipped.  The
 * header and payload decoders are atg_cp_seq_dec's.
 * halt stops the sequencer between packets (in HALTING, HALTED, FAULTED, and
 * RESETTING).  abort stops the sequencer immediately (in FAULTED and
 * RESETTING), dropping payload and unit requests.  cmd_active (the watchdog)
 * is 1 from a header to the end of its µop, except while waiting for the
 * firmware's next dword.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_seq #(
	parameter [7:0] UNITS = 8'h03,	/* Present blocks, STATUS busy-bit order */
	parameter HAS_VSYNC = 0,	/* 1: a display's vsync reaches the CP (WAIT_VSYNC) */
	parameter HAS_DISP = 0,	/* 1: a DISP2 controller (DISP_SET, FLIP; atg_cp_disp) */
	parameter HAS_HW_WALKER = 0,
	parameter HAS_MESH = 0,
	parameter HAS_SECONDARY_RING = 0
) (
	input wire clk,
	input wire rst_n,
	/* µop packets from the ring (atg_cp_ring) */
	input wire n_valid,
	output wire n_ready,
	input wire [31:0] n_data,
	/* µop packets from the firmware (atg_cpu UOP_FIFO) */
	input wire fw_valid,
	output wire fw_ready,
	input wire [31:0] fw_data,
	input wire fw_active,
	input wire [6:0] fw_side,	/* session protection: the firmware µop's {SELF, slot} */
	input wire halt,
	input wire seq_abort,
	/* STATE writes (to atg_cp_state) */
	output wire uop_we,
	output wire [9:0] uop_idx,
	output wire [31:0] uop_wdata,
	/* Kicks: a one-cycle pulse with its code (K_*) */
	output wire kick,
	output wire [2:0] kick_code,
	output wire cs_mesh,
	output wire [3:0] inv,
	input wire [8:0] busy_in,	/* [8]: atg_cp_disp's wait (not in B_ALL) */
	input wire vsync,
	input wire l2_flush_done,
	input wire l2_idle,
	/* Fences (to atg_cp_fence) */
	output wire f_valid,
	input wire f_ready,
	output wire [1:0] f_op,
	output wire [63:0] f_addr,
	output wire [31:0] f_value,
	output wire [31:0] f_aux,
	output wire f_irq,
	output wire f_mem,
	output wire f_flush,
	output wire f_64,
	input wire f_busy,
	/* The occlusion counter (atg_rop_occ, QUERY) */
	input wire [39:0] occ_count,
	output wire occ_clear,
	/* Status */
	output wire busy,
	output wire cmd_active,
	output wire [3:0] seq_st,
	/* Fault report to atg_cp_ctl (one source of its `src_*` vectors) */
	output wire fault,
	output wire [3:0] fault_kind,
	output wire [31:0] fault_addr,
	output wire [7:0] fault_tag,
	/* Session protection (atg_cp_prot): the µop's source and side, its unit kick */
	output wire u_fw,
	output wire [6:0] u_side,
	output wire [1:0] u_req,	/* [0] it starts a unit, [1] it uses the shader units */
	input wire u_ok,		/* the units run for its slot: it may start them */
	input wire u_active,
	/* Caps internal read interface (for CMD_QUERY_CAPS) */
	output wire [8:0] caps_r_idx,
	input wire [31:0] caps_r_data,
	input wire caps_fw_stall,
	output reg [3:0] bound_session
);
	`include "ghci_regs.vh"

	/* Standard µops (class 0) and Amaterrace's vendor µops (class 1); the header's
	 * rules are atg_cp_seq_dec's. */
	localparam [7:0] OP_SET_REG = 8'd1;
	localparam [7:0] OP_BLIT = 8'd2;
	localparam [7:0] OP_DRAW = 8'd3;
	localparam [7:0] OP_DISPATCH = 8'd4;
	localparam [7:0] OP_FENCE = 8'd5;
	localparam [7:0] OP_LOAD_CONST = 8'd7;
	localparam [7:0] OP_LOAD_PROGRAM = 8'd8;
	localparam [7:0] OP_WAIT_FENCE = 8'd9;
	localparam [7:0] OP_PERF_RESET = 8'd10;
	localparam [7:0] OP_WAIT_VSYNC = 8'd11;
	localparam [7:0] OP_INVALIDATE = 8'd12;
	localparam [7:0] OP_RESULT = 8'd13;
	localparam [7:0] OP_QUERY = 8'd14;
	localparam [7:0] OP_DISP_SET = 8'd15;
	localparam [7:0] OP_FLIP = 8'd16;
	localparam [7:0] OP_BARRIER = 8'd20;

	/* GHCI 1.0 Structured Command Opcodes */
	/* verilator lint_off UNUSEDPARAM */
	localparam [7:0] CMD_SESSION_OPEN          = 8'h12;
	localparam [7:0] CMD_SESSION_CLOSE         = 8'h13;
	localparam [7:0] CMD_BIND_SESSION          = 8'h14;
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
	localparam [7:0] CMD_WAIT_FENCE            = 8'h34;
	/* verilator lint_on UNUSEDPARAM */

	localparam [7:0] VOP_DRAW_RAW = 8'd0;
	localparam [7:0] VOP_DBG_MARK = 8'd1;

	localparam [2:0] K_BLT = 3'd0;
	localparam [2:0] K_DRAW = 3'd1;
	localparam [2:0] K_CS = 3'd2;
	localparam [2:0] K_LC = 3'd3;
	localparam [2:0] K_RAW = 3'd4;
	localparam [2:0] K_PERF = 3'd5;
	localparam [2:0] K_FLIP = 3'd6;
	localparam [2:0] K_DISP_IDLE = 3'd7;

	localparam [3:0] Q_IDLE = 4'd0;
	localparam [3:0] Q_PAY = 4'd1;	/* take the payload */
	localparam [3:0] Q_EXEC = 4'd2;	/* wait for pre, then act */
	localparam [3:0] Q_POST = 4'd3;	/* after a kick: wait for post */
	localparam [3:0] Q_FREQ = 4'd4;	/* fence request */
	localparam [3:0] Q_FWAIT = 4'd5;	/* WAIT_FENCE: wait for !f_busy */
	localparam [3:0] Q_VSYNC = 4'd6;
	localparam [3:0] Q_DEAD = 4'd7;
	localparam [3:0] Q_DEC = 4'd8;
	localparam [3:0] Q_DEC2 = 4'd9;
	localparam [3:0] Q_BARRIER = 4'd10;
	localparam [3:0] Q_FLUSH = 4'd11;
	localparam [3:0] Q_QCAPS_R0 = 4'd12;
	localparam [3:0] Q_QCAPS_R1 = 4'd13;
	localparam [3:0] Q_QCAPS_REQ = 4'd14;
	localparam [3:0] Q_QCAPS_WAIT = 4'd15;

	(* max_fanout = 12 *) reg [3:0] st;
	(* dont_touch = "true" *) reg [3:0] st_c;
	(* dont_touch = "true" *) reg [3:0] st_c_qc;
	(* dont_touch = "true" *) reg [3:0] st_c_qc_d;
	(* dont_touch = "true" *) reg is_qc_req_fa0_b0_r, is_qc_req_fa0_b0a_r, is_qc_req_fa0_b1_r, is_qc_req_fa0_b2_r, is_qc_req_fa0_b3_r, is_qc_req_fa1_lo_r, is_qc_req_fa1_hi_r, is_qc_req_fv0_r, is_qc_req_fv1_r, is_qc_req_fx0_r, is_qc_req_fx1_r, is_qc_req_fop_r, is_qc_req_f64_r;
	(* dont_touch = "true" *) reg [3:0] st_out_r;
	(* dont_touch = "true" *) reg [3:0] st_g0, st_g1, st_g2, st_g3, st_g4;
	(* DONT_TOUCH = "TRUE" *) reg st_is_idle_r;
	(* DONT_TOUCH = "TRUE" *) reg st_is_dec_r;
	(* DONT_TOUCH = "TRUE" *) reg st_is_exec_r;
	(* max_fanout = 16 *) reg use_fw;
	(* max_fanout = 12 *) reg [31:0] hdr_r;
	(* max_fanout = 16 *) reg is_ghci_hdr_r;
	(* max_fanout = 16 *) reg [7:0] ghci_n_r;
	reg dec_fault_r, dec_short_r, dec_absent_r;
	(* max_fanout = 16 *) reg h_zero_r;
	(* max_fanout = 16 *) reg is_query_r;
	(* max_fanout = 16 *) reg vendor;
	(* max_fanout = 16 *) reg [7:0] op;
	(* max_fanout = 16 *) reg [7:0] flags;
	reg [7:0] tag;
	(* max_fanout = 16 *) reg [6:0] side;
	(* max_fanout = 16 *) reg [7:0] n;
	(* max_fanout = 16 *) reg [7:0] p;	/* payload dwords taken */
	/* Pure-Boolean alternating NAND/NOR prefix incrementers & decrementers (zero am_add/am_sub). */
	function [6:0] fn_inc7;
		input [6:0] a;
		reg [6:0] s0_n, s1, s2_n;
		begin
			s0_n = ~(a    & {a[5:0],    1'b1});
			s1   = ~(s0_n | {s0_n[4:0], 2'b00});
			s2_n = ~(s1   & {s1[2:0],   4'hF});
			fn_inc7 = a ^ {~s2_n[5:0], 1'b1};
		end
	endfunction

	function [7:0] fn_inc8;
		input [7:0] a;
		reg [7:0] s0_n, s1, s2_n;
		begin
			s0_n = ~(a    & {a[6:0],    1'b1});
			s1   = ~(s0_n | {s0_n[5:0], 2'b00});
			s2_n = ~(s1   & {s1[3:0],   4'hF});
			fn_inc8 = a ^ {~s2_n[6:0], 1'b1};
		end
	endfunction

	function [6:0] fn_dec7;
		input [6:0] a;
		reg [6:0] p0, p1_n, b;
		begin
			p0   = ~(a    | {a[5:0],    1'b0});
			p1_n = ~(p0   & {p0[4:0],   2'b11});
			b    = ~(p1_n | {p1_n[2:0], 4'b0000});
			fn_dec7 = a ^ {b[5:0], 1'b1};
		end
	endfunction

	function [7:0] fn_dec8;
		input [7:0] a;
		reg [7:0] p0, p1_n, b;
		begin
			p0   = ~(a    | {a[6:0],    1'b0});
			p1_n = ~(p0   & {p0[5:0],   2'b11});
			b    = ~(p1_n | {p1_n[3:0], 4'b0000});
			fn_dec8 = a ^ {b[6:0], 1'b1};
		end
	endfunction

	function [60:0] fn_inc61;
		input [60:0] v;
		reg [60:0] s0_n, s1, s2_n, s3, s4_n, s5;
		begin
			s0_n = ~(v    & {v[59:0],    1'b1});
			s1   = ~(s0_n | {s0_n[58:0], 2'b00});
			s2_n = ~(s1   & {s1[56:0],   4'hF});
			s3   = ~(s2_n | {s2_n[52:0], 8'h00});
			s4_n = ~(s3   & {s3[44:0],   16'hFFFF});
			s5   = ~(s4_n | {s4_n[28:0], 32'h0000_0000});
			fn_inc61 = v ^ {s5[59:0], 1'b1};
		end
	endfunction

	wire [7:0] p_n = fn_inc8(p);	/* p + 1 */
	wire p_co = 1'b0;
	(* max_fanout = 16 *) reg [9:0] sidx;	/* SET_REG: the index of the pair */
	(* max_fanout = 16 *) reg [31:0] a0, a0_hi, a1, a2;	/* fence payload */
	(* max_fanout = 16 *) reg [31:0] fence_flags;
	(* max_fanout = 16 *) reg [39:0] occ_q;
	reg settle;
	(* max_fanout = 16 *) reg nop_r;
	(* max_fanout = 16 *) reg is_allowed_op_r;


`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire seq_rst_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_seq_rst (.I0(rst_n), .O(seq_rst_high));

	wire [4:0] fw_act_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwa0 (.I0(fw_active), .O(fw_act_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwa1 (.I0(fw_active), .O(fw_act_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwa2 (.I0(fw_active), .O(fw_act_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwa3 (.I0(fw_active), .O(fw_act_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_fwa4 (.I0(fw_active), .O(fw_act_b[4]));

	wire [4:0] use_fw_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ufw0 (.I0(use_fw), .O(use_fw_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ufw1 (.I0(use_fw), .O(use_fw_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ufw2 (.I0(use_fw), .O(use_fw_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ufw3 (.I0(use_fw), .O(use_fw_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ufw4 (.I0(use_fw), .O(use_fw_b[4]));

	wire in_valid;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_in_v (.I0(n_valid), .I1(fw_valid), .I2(use_fw), .O(in_valid));
	(* keep = "true" *) wire [31:0] in_data = {
		use_fw_b[3] ? fw_data[31:24] : n_data[31:24],
		use_fw_b[2] ? fw_data[23:16] : n_data[23:16],
		use_fw_b[1] ? fw_data[15:8]  : n_data[15:8],
		use_fw_b[0] ? fw_data[7:0]   : n_data[7:0]
	};
	(* keep = "true" *) wire [31:0] hdr = {
		fw_act_b[3] ? fw_data[31:24] : n_data[31:24],
		fw_act_b[2] ? fw_data[23:16] : n_data[23:16],
		fw_act_b[1] ? fw_data[15:8]  : n_data[15:8],
		fw_act_b[0] ? fw_data[7:0]   : n_data[7:0]
	};
	wire hdr_valid;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_hdr_v (.I0(n_valid), .I1(fw_valid), .I2(fw_act_b[4]), .O(hdr_valid));
	wire unused_ufw4 = use_fw_b[4];
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire seq_rst_high;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_seq_rst (.I0(rst_n), .O(seq_rst_high));

	wire [4:0] fw_act_b;
	assign fw_act_b[0] = fw_active;
	assign fw_act_b[1] = fw_active;
	assign fw_act_b[2] = fw_active;
	assign fw_act_b[3] = fw_active;
	assign fw_act_b[4] = fw_active;

	wire [4:0] use_fw_b;
	assign use_fw_b[0] = use_fw;
	assign use_fw_b[1] = use_fw;
	assign use_fw_b[2] = use_fw;
	assign use_fw_b[3] = use_fw;
	assign use_fw_b[4] = use_fw;

	wire in_valid;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_in_v (.I0(n_valid), .I1(fw_valid), .I2(use_fw), .O(in_valid));
	(* keep = "true" *) wire [31:0] in_data = {
		use_fw_b[3] ? fw_data[31:24] : n_data[31:24],
		use_fw_b[2] ? fw_data[23:16] : n_data[23:16],
		use_fw_b[1] ? fw_data[15:8]  : n_data[15:8],
		use_fw_b[0] ? fw_data[7:0]   : n_data[7:0]
	};
	(* keep = "true" *) wire [31:0] hdr = {
		fw_act_b[3] ? fw_data[31:24] : n_data[31:24],
		fw_act_b[2] ? fw_data[23:16] : n_data[23:16],
		fw_act_b[1] ? fw_data[15:8]  : n_data[15:8],
		fw_act_b[0] ? fw_data[7:0]   : n_data[7:0]
	};
	wire hdr_valid;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_hdr_v (.I0(n_valid), .I1(fw_valid), .I2(fw_act_b[4]), .O(hdr_valid));
	wire unused_ufw4 = use_fw_b[4];
`else
	wire seq_rst_high = !rst_n;
	wire [4:0] fw_act_b = {5{fw_active}};
	wire [4:0] use_fw_b = {5{use_fw}};
	wire in_valid = use_fw ? fw_valid : n_valid;
	wire [31:0] in_data = use_fw ? fw_data : n_data;
	wire [31:0] hdr = fw_active ? fw_data : n_data;
	wire hdr_valid = fw_active ? fw_valid : n_valid;
`endif

	/* Header and payload decode (combinational). */
	wire is_ghci_hdr_comb = (
		(hdr[7:0] >= 8'h10 && hdr[7:0] <= 8'h15) ||
		(hdr[7:0] >= 8'h20 && hdr[7:0] <= 8'h37) ||
		(hdr[7:0] >= 8'h40 && hdr[7:0] <= 8'h46) ||
		(hdr[7:0] >= 8'h50 && hdr[7:0] <= 8'h51)
	) && (hdr[15:8] >= 8'd1);
	wire [7:0] ghci_n_comb = fn_dec8(hdr[15:8]);
	wire ghci_sub_bo = 1'b0;

	wire is_ghci_hdr = (st_c == Q_DEC) ? is_ghci_hdr_r : is_ghci_hdr_comb;
	wire h_vendor = is_ghci_hdr_r ? 1'b0 : hdr_r[30];
	wire [7:0] h_op = is_ghci_hdr_r ? hdr_r[7:0] : {2'b00, hdr_r[29:24]};
	wire [7:0] h_n = is_ghci_hdr_r ? ghci_n_r : hdr_r[7:0];
	wire [7:0] h_n_m1_hdr  = fn_dec8(hdr_r[7:0]);
	wire [7:0] h_n_m1_ghci = fn_dec8(ghci_n_r);
	wire [7:0] h_n_m1 = is_ghci_hdr_r ? h_n_m1_ghci : h_n_m1_hdr;
	wire n_m1_bo = 1'b0;
	wire [7:0] h_n_m2_hdr  = {fn_dec7(hdr_r[7:1]), hdr_r[0]};
	wire [7:0] h_n_m2_ghci = {fn_dec7(ghci_n_r[7:1]), ghci_n_r[0]};
	wire [7:0] h_n_m2 = is_ghci_hdr_r ? h_n_m2_ghci : h_n_m2_hdr;
	wire n_m2_bo = 1'b0;

	wire dec_hdr_fault, dec_hdr_absent, dec_hdr_short, h_zero, pay_bad, pay_absent;
	atg_cp_seq_dec #(
		.UNITS(UNITS),
		.HAS_VSYNC(HAS_VSYNC),
		.HAS_DISP(HAS_DISP),
		.HAS_HW_WALKER(HAS_HW_WALKER),
		.HAS_MESH(HAS_MESH),
		.HAS_SECONDARY_RING(HAS_SECONDARY_RING)
	) dec (
		.hdr(hdr_r), .hdr_fw(use_fw), .is_ghci_in(is_ghci_hdr_r), .h_known(),
		.h_absent(dec_hdr_absent),
		.h_need(), .h_short(), .h_zero(h_zero),
		.hdr_fault(dec_hdr_fault), .hdr_short(dec_hdr_short),
		.vendor(vendor), .op(op), .p(p), .in_data(in_data),
		.pay_bad(pay_bad),
		.pay_absent(pay_absent)
	);

	(* max_fanout = 16 *) reg is_set_r, is_dset_r, is_fence_cmd_r, is_fc_or_res_r, is_query_caps_r;
	reg is_set_r1, is_dset_r1, is_sopen_r1, is_bsess_r;
	reg is_fop_fence_r, is_fop_res_q_r, is_wait_fence_r;
	(* max_fanout = 16 *) reg is_dispatch_r, is_mesh_r, is_load_const_r, is_draw_raw_r, is_barrier_r;
	assign cs_mesh = is_mesh_r;

	reg [31:0] qc_addr_lo, qc_addr_hi, qc_bytes;
	reg [6:0] qc_total_k_r;
	reg qc_zero_r;
	reg [31:0] qc_cur_addr;
	(* max_fanout = 16 *) reg [31:0] qc_cur_addr_hi;
	(* max_fanout = 16 *) reg [6:0] qc_k;
	(* max_fanout = 16, dont_touch = "true" *) reg [6:0] qc_k_cap;
	reg [31:0] qc_w0, qc_w1;
	reg qc_align_err;

	wire [60:0] qc_addr_inc61 = fn_inc61({qc_cur_addr_hi, qc_cur_addr[31:3]});
	wire [31:0] qc_cur_addr_n = {qc_addr_inc61[28:0], qc_cur_addr[2:0]};
	wire [31:0] qc_cur_addr_hi_n = qc_addr_inc61[60:29];
	wire qc_addr_co = 1'b0;
	wire qc_addr_hi_co = 1'b0;
	wire [6:0] qc_k_n = fn_inc7(qc_k);
	wire qc_k_co = 1'b0;

	wire [6:0] qc_total_k = qc_total_k_r;
	reg qc_last_r;

	reg qc_word_sel_r, qc_r_rdy;
	assign caps_r_idx = {qc_k_cap, qc_word_sel_r};

	/* Payload: what this dword does; the waits (atg_cp_seq_wait). */
	reg is_set_or_dset_r;
	wire is_set = !vendor && op == OP_SET_REG;
	wire is_dset = !vendor && op == OP_DISP_SET;
	wire set_val = is_set_or_dset_r && p[0];
	wire set_bad = pay_bad;
	wire pay_we;
	wire [9:0] pay_idx;
	wire [8:0] pay_wait;	/* busy bits that hold this dword */
	wire [8:0] pre;
	wire pre_f;		/* also wait for !f_busy */
	wire [8:0] post;

	atg_cp_seq_wait wt (
		.is_ghci(is_ghci_hdr_r),
		.vendor(vendor),
		.op(op),
		.flags(flags),
		.p(p),
		.set_val(set_val),
		.sidx(sidx),
		.is_draw_raw(is_draw_raw_r),
		.is_dispatch(is_dispatch_r),
		.is_load_const(is_load_const_r),
		.p_lt_3_i(p_lt_3_r),
		.p_lt_4_i(p_lt_4_r),
		.p_lt_9_i(p_lt_9_r),
		.p_eq_0_i(p_is_0_r),
		.is_dbg_mark_i(is_dbg_mark_r),
		.is_flip_i(is_flip_r),
		.pay_we(pay_we),
		.pay_idx(pay_idx),
		.pay_wait(pay_wait),
		.pre(pre),
		.pre_f(pre_f),
		.post(post)
	);

	/* All s_wait boundaries (0x020, 0x050, 0x090, 0x190, 0x1B0, 0x1C0) are multiples of 0x010,
	 * so s_wait_in is a pure function of 6 bits in_data[9:4] (1 LUT6 per bit). */
	reg f_valid_r, f_flush_r;
	function [7:0] fn_s_wait;
		input [5:0] s_hi;
		reg r_5_26, b_632, b_5, b_4, b_2_4, b_28_29;
		begin
			/* 5..26: (>=5) && (<=26) without comparators: s_hi[5]==0 and in 5..26 */
			r_5_26  = !s_hi[5] && (
				(s_hi[4:3] == 2'b00 && (s_hi[2] && (s_hi[1] || s_hi[0]))) || /* 5..7 */
				(s_hi[4:3] == 2'b01) || /* 8..15 */
				(s_hi[4:3] == 2'b10) || /* 16..23 */
				(s_hi[4:2] == 3'b110 && (s_hi[1:0] != 2'b11)) /* 24..26 */
			);
			b_632   = s_hi[5] || r_5_26;
			b_5     = s_hi[5] || (!s_hi[4] && s_hi[3] && (s_hi[2:0] != 3'd0)) || (s_hi[4] && !s_hi[3]) || (s_hi[4:0] == 5'd24);
			b_4     = !s_hi[5] && !s_hi[4] && ((!s_hi[3] && s_hi[2] && (s_hi[1] || s_hi[0])) || (s_hi[3:0] == 4'd8));
			b_2_4   = (s_hi[5:1] == 5'd1) || (s_hi[5:0] == 6'd4);
			b_28_29 = (s_hi[5:1] == 5'd14);
			fn_s_wait = {1'b0, b_632, b_5, b_4, b_632, b_632 | b_2_4 | b_28_29, (s_hi[5:1] == 5'd0), 1'b0};
		end
	endfunction
	wire [7:0] s_wait_n  = fn_s_wait(n_data[9:4]);
	wire [7:0] s_wait_fw = fn_s_wait(fw_data[9:4]);
	(* keep = "true" *) wire [7:0] s_wait_in = use_fw ? s_wait_fw : s_wait_n;

	wire [8:0] pay_wait_act;
	wire pay_hold = |(pay_wait_act & busy_in);
	wire pay_take = st_is_pay_r && in_valid && !pay_hold && !seq_abort;
	wire hdr_take_fast = st_is_idle_r && !seq_abort && hdr_valid;
	wire hdr_take = hdr_take_fast && !halt;

	(* DONT_TOUCH = "TRUE" *) reg st_is_pay_r;
	reg [7:0] n_m1_r;
	reg [7:0] n_m2_r;
	reg p_is_0_r, p_is_1_r, p_is_2_r, p_is_3_r, p_is_4_r, p_is_last_r;
	reg p_lt_3_r, p_lt_4_r, p_lt_9_r;
	reg is_dbg_mark_r, is_flip_r;

	wire [6:0] hdr_take_b;
	wire [3:0] seq_abort_b;
	(* max_fanout = 16 *) reg [8:0] pay_wait_r;
	wire pay_hold_0, pay_hold_1, pay_hold_2;
	wire [3:0] pay_take_b;

`ifdef SYNTH_KINTEX7
	wire hdr_take_fsm;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0200)) u_htk_fsm (.I0(st_is_idle_r), .I1(halt), .I2(seq_abort_b[2]), .I3(hdr_valid), .O(hdr_take_fsm));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk0 (.I0(hdr_take), .O(hdr_take_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk1 (.I0(hdr_take), .O(hdr_take_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk2 (.I0(hdr_take), .O(hdr_take_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk3 (.I0(hdr_take), .O(hdr_take_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk4 (.I0(hdr_take), .O(hdr_take_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk5 (.I0(hdr_take), .O(hdr_take_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_htk6 (.I0(hdr_take), .O(hdr_take_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sab0 (.I0(seq_abort), .O(seq_abort_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sab1 (.I0(seq_abort), .O(seq_abort_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sab2 (.I0(seq_abort), .O(seq_abort_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sab3 (.I0(seq_abort), .O(seq_abort_b[3]));

	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_ph0 (
		.I0(pay_wait_r[0]), .I1(busy_in[0]), .I2(pay_wait_r[1]), .I3(busy_in[1]), .I4(pay_wait_r[2]), .I5(busy_in[2]),
		.O(pay_hold_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_ph1 (
		.I0(pay_wait_r[3]), .I1(busy_in[3]), .I2(pay_wait_r[4]), .I3(busy_in[4]), .I4(pay_wait_r[5]), .I5(busy_in[5]),
		.O(pay_hold_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_ph2 (
		.I0(pay_wait_r[6]), .I1(busy_in[6]), .I2(pay_wait_r[7]), .I3(busy_in[7]), .I4(pay_wait_r[8]), .I5(busy_in[8]),
		.O(pay_hold_2)
	);

	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000008)) u_ptk0 (
		.I0(st_is_pay_r), .I1(in_valid), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .I5(seq_abort_b[2]), .O(pay_take_b[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000008)) u_ptk1 (
		.I0(st_is_pay_r), .I1(in_valid), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .I5(seq_abort_b[2]), .O(pay_take_b[1])
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000008)) u_ptk2 (
		.I0(st_is_pay_r), .I1(in_valid), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .I5(seq_abort_b[3]), .O(pay_take_b[2])
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000008)) u_ptk3 (
		.I0(st_is_pay_r), .I1(in_valid), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .I5(seq_abort_b[3]), .O(pay_take_b[3])
	);
`else
	wire hdr_take_fsm = hdr_take;
	assign hdr_take_b = {7{hdr_take}};
	assign seq_abort_b = {4{seq_abort}};
	wire [8:0] ph_n = ~(pay_wait_r & busy_in);
	wire ph_g0_n = ~&ph_n[3:1];
	wire ph_g1_n = ~&ph_n[6:4];
	wire ph_g2_n = pay_wait_r[8] & busy_in[8];
	assign pay_hold_0 = ph_g0_n;
	assign pay_hold_1 = ph_g1_n;
	assign pay_hold_2 = ph_g2_n;
	wire pay_hold_n = ~(ph_g0_n | ph_g1_n | ph_g2_n);
	wire pay_hold_any = !pay_hold_n;
	wire pay_pre_go = st_is_pay_r && !seq_abort && in_valid;
	wire pay_go_ok = pay_hold_n && pay_pre_go;
	wire pay_take_fast = st_is_pay_r && pay_hold_n && !seq_abort;
	assign pay_take_b = {4{pay_go_ok}};
`endif

	wire [8:0] pw_d;
	assign pw_d[0] = 1'b0;
	assign pw_d[1] = st_is_pay_r && is_set_r && !p[0] && s_wait_in[1];
	assign pw_d[2] = st_is_pay_r && is_set_r && !p[0] && s_wait_in[2];
	assign pw_d[3] = is_draw_raw_r || (st_is_pay_r && is_set_r && !p[0] && s_wait_in[3]);
	assign pw_d[4] = st_is_pay_r && is_set_r && !p[0] && s_wait_in[4];
	assign pw_d[5] = st_is_pay_r && is_set_r && !p[0] && s_wait_in[5];
	assign pw_d[6] = ((is_dispatch_r || is_load_const_r) && p_is_0_r) || (st_is_pay_r && is_set_r && !p[0] && s_wait_in[6]);
	assign pw_d[7] = 1'b0;
	assign pw_d[8] = st_is_pay_r && is_dset_r && !p[0];

	wire pw_we_b;

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pw_we (
		.I0(st_c == Q_DEC2), .I1(pay_take_b[3]), .O(pw_we_b)
	);
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
			pay_wait_r <= 9'd0;
		end else if (pw_we_b) begin
			pay_wait_r <= pw_d;
		end
	end
`elsif SYNTH_KINTEX_ULTRASCALE
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hE)) u_pw_we (
		.I0(st_c == Q_DEC2), .I1(pay_take_b[3]), .O(pw_we_b)
	);
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
			pay_wait_r <= 9'd0;
		end else if (pw_we_b) begin
			pay_wait_r <= pw_d;
		end
	end
`else
	reg st_is_dec2_r;
	assign pw_we_b = st_is_dec2_r || pay_take_b[3];
	wire [8:0] pw_d_pay = {
		is_dset_r && !p[0],
		1'b0,
		((is_dispatch_r || is_load_const_r) && p_is_0_r) || (is_set_r && !p[0] && s_wait_in[6]),
		is_set_r && !p[0] && s_wait_in[5],
		is_set_r && !p[0] && s_wait_in[4],
		is_draw_raw_r || (is_set_r && !p[0] && s_wait_in[3]),
		is_set_r && !p[0] && s_wait_in[2],
		is_set_r && !p[0] && s_wait_in[1],
		1'b0
	};
	wire [8:0] pw_d_dec2 = {2'b00, (is_dispatch_r || is_load_const_r) && p_is_0_r, 2'b00, is_draw_raw_r, 3'b000};
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
			pay_wait_r <= 9'd0;
		end else if (st_is_dec2_r) begin
			pay_wait_r <= pw_d_dec2;
		end else if (pay_pre_go && pay_hold_n) begin
			pay_wait_r <= pw_d_pay;
		end
	end
`endif
	assign pay_wait_act = pay_wait_r;

	wire idle_n_rdy  = st_is_idle_r && !halt && !seq_abort && !fw_act_b[4];
	wire idle_fw_rdy = st_is_idle_r && !halt && !seq_abort && fw_act_b[4];
	wire pay_n_rdy   = st_is_pay_r && !seq_abort && !use_fw_b[4];
	wire pay_fw_rdy  = st_is_pay_r && !seq_abort && use_fw_b[4];

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hAAAAAAAE)) u_nrdy_lut (
		.I0(idle_n_rdy), .I1(pay_n_rdy), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .O(n_ready)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hAAAAAAAE)) u_fwrdy_lut (
		.I0(idle_fw_rdy), .I1(pay_fw_rdy), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .O(fw_ready)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hAAAAAAAE)) u_nrdy_lut (
		.I0(idle_n_rdy), .I1(pay_n_rdy), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .O(n_ready)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hAAAAAAAE)) u_fwrdy_lut (
		.I0(idle_fw_rdy), .I1(pay_fw_rdy), .I2(pay_hold_0), .I3(pay_hold_1), .I4(pay_hold_2), .O(fw_ready)
	);
`else
	wire st_idle_n_ok  = st_is_idle_r && !seq_abort && !fw_act_b[4];
	wire st_idle_fw_ok = st_is_idle_r && !seq_abort && fw_act_b[4];
	wire pay_n_ok  = st_is_pay_r && !seq_abort && !use_fw_b[4];
	wire pay_fw_ok = st_is_pay_r && !seq_abort && use_fw_b[4];
	wire ph_g01_ok = !(ph_g0_n | ph_g1_n);
	wire ph_n_g2_ok = !(ph_g2_n | !pay_n_ok);
	wire ph_fw_g2_ok = !(ph_g2_n | !pay_fw_ok);
	wire n_idle_rdy_n  = !(st_idle_n_ok  && !halt);
	wire fw_idle_rdy_n = !(st_idle_fw_ok && !halt);
	wire n_pay_rdy_n   = !(ph_g01_ok && ph_n_g2_ok);
	wire fw_pay_rdy_n  = !(ph_g01_ok && ph_fw_g2_ok);
	assign n_ready  = !(n_idle_rdy_n  & n_pay_rdy_n);
	assign fw_ready = !(fw_idle_rdy_n & fw_pay_rdy_n);
`endif

	/* 2-stage payload bad check to keep in_data -> FF <= 3 LUTs and pay -> next_st <= 3 LUTs */
	reg pay_chk_v_r;
	reg pay_chk_p0_r;
	reg pay_set_bad_r;
	reg pay_dset_bad_r;
	reg pay_nz32_r;
`ifdef SYNTH_KINTEX7
	wire [31:0] pchk_in [0:1];
	assign pchk_in[0] = n_data;
	assign pchk_in[1] = fw_data;
	wire [1:0] pchk_nz32, pchk_set_bad, pchk_nz24, pchk_off_bad;
	wire [6:0] pchk_qc_k [0:1];
	genvar g_pc;
	generate
		for (g_pc = 0; g_pc < 2; g_pc = g_pc + 1) begin : g_pchk
			wire [31:0] d = pchk_in[g_pc];
			wire [5:0] z_g;
			wire [3:0] b_g;
			wire off_hi_ok, d_z11_8;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg0 (.I0(d[0]),  .I1(d[1]),  .I2(d[2]),  .I3(d[3]),  .I4(d[4]),  .I5(d[5]),  .O(z_g[0]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg1 (.I0(d[6]),  .I1(d[7]),  .I2(d[8]),  .I3(d[9]),  .I4(d[10]), .I5(d[11]), .O(z_g[1]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg2 (.I0(d[12]), .I1(d[13]), .I2(d[14]), .I3(d[15]), .I4(d[16]), .I5(d[17]), .O(z_g[2]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg3 (.I0(d[18]), .I1(d[19]), .I2(d[20]), .I3(d[21]), .I4(d[22]), .I5(d[23]), .O(z_g[3]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg4 (.I0(d[24]), .I1(d[25]), .I2(d[26]), .I3(d[27]), .I4(d[28]), .I5(d[29]), .O(z_g[4]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1))                 u_zg5 (.I0(d[30]), .I1(d[31]), .O(z_g[5]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h7FFFFFFFFFFFFFFF)) u_nz32 (.I0(z_g[0]), .I1(z_g[1]), .I2(z_g[2]), .I3(z_g[3]), .I4(z_g[4]), .I5(z_g[5]), .O(pchk_nz32[g_pc]));

			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFF8)) u_sb0 (.I0(d[8]),  .I1(d[9]),  .I2(d[10]), .I3(d[11]), .I4(d[12]), .I5(d[13]), .O(b_g[0]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb1 (.I0(d[14]), .I1(d[15]), .I2(d[16]), .I3(d[17]), .I4(d[18]), .I5(d[19]), .O(b_g[1]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb2 (.I0(d[20]), .I1(d[21]), .I2(d[22]), .I3(d[23]), .I4(d[24]), .I5(d[25]), .O(b_g[2]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb3 (.I0(d[26]), .I1(d[27]), .I2(d[28]), .I3(d[29]), .I4(d[30]), .I5(d[31]), .O(b_g[3]));
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFFE))            u_sb  (.I0(b_g[0]), .I1(b_g[1]), .I2(b_g[2]), .I3(b_g[3]), .O(pchk_set_bad[g_pc]));

			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001))            u_z118  (.I0(d[8]), .I1(d[9]), .I2(d[10]), .I3(d[11]), .O(d_z11_8));
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h7FFFFFFF))        u_nz24  (.I0(z_g[2]), .I1(z_g[3]), .I2(z_g[4]), .I3(z_g[5]), .I4(d_z11_8), .O(pchk_nz24[g_pc]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000600005FFC10)) u_offok (.I0(d[2]), .I1(d[3]), .I2(d[4]), .I3(d[5]), .I4(d[6]), .I5(d[7]), .O(off_hi_ok));
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEF))               u_obad  (.I0(d[0]), .I1(d[1]), .I2(off_hi_ok), .O(pchk_off_bad[g_pc]));

			genvar g_kb;
			for (g_kb = 0; g_kb < 5; g_kb = g_kb + 1) begin : g_qk_lo
				(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h40)) u_qk (.I0(pchk_set_bad[g_pc]), .I1(pchk_nz32[g_pc]), .I2(d[g_kb + 3]), .O(pchk_qc_k[g_pc][g_kb]));
			end
			for (g_kb = 5; g_kb < 7; g_kb = g_kb + 1) begin : g_qk_hi
				(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qk (.I0(pchk_set_bad[g_pc]), .I1(pchk_nz32[g_pc]), .I2(d[g_kb + 3]), .O(pchk_qc_k[g_pc][g_kb]));
			end
		end
	endgenerate
	wire pay_nz32_c, qc_zero_c, pay_set_bad_c, pay_dset_bad_c;
	wire [6:0] qc_total_k_c;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_pnz_m (.I0(pchk_nz32[0]),    .I1(pchk_nz32[1]),    .I2(use_fw), .O(pay_nz32_c));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h35)) u_qcz_m (.I0(pchk_nz32[0]),    .I1(pchk_nz32[1]),    .I2(use_fw), .O(qc_zero_c));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_psb_m (.I0(pchk_set_bad[0]), .I1(pchk_set_bad[1]), .I2(use_fw), .O(pay_set_bad_c));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFFF0EEEE)) u_pdb_m (
		.I0(pchk_nz24[0]), .I1(pchk_off_bad[0]), .I2(pchk_nz24[1]), .I3(pchk_off_bad[1]), .I4(use_fw), .O(pay_dset_bad_c)
	);
	genvar g_qm;
	generate
		for (g_qm = 0; g_qm < 7; g_qm = g_qm + 1) begin : g_qck_mux
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_qckm (.I0(pchk_qc_k[0][g_qm]), .I1(pchk_qc_k[1][g_qm]), .I2(use_fw), .O(qc_total_k_c[g_qm]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [31:0] pchk_in [0:1];
	assign pchk_in[0] = n_data;
	assign pchk_in[1] = fw_data;
	wire [1:0] pchk_nz32, pchk_set_bad, pchk_nz24, pchk_off_bad;
	wire [6:0] pchk_qc_k [0:1];
	genvar g_pc;
	generate
		for (g_pc = 0; g_pc < 2; g_pc = g_pc + 1) begin : g_pchk
			wire [31:0] d = pchk_in[g_pc];
			wire [5:0] z_g;
			wire [3:0] b_g;
			wire off_hi_ok, d_z11_8;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg0 (.I0(d[0]),  .I1(d[1]),  .I2(d[2]),  .I3(d[3]),  .I4(d[4]),  .I5(d[5]),  .O(z_g[0]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg1 (.I0(d[6]),  .I1(d[7]),  .I2(d[8]),  .I3(d[9]),  .I4(d[10]), .I5(d[11]), .O(z_g[1]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg2 (.I0(d[12]), .I1(d[13]), .I2(d[14]), .I3(d[15]), .I4(d[16]), .I5(d[17]), .O(z_g[2]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg3 (.I0(d[18]), .I1(d[19]), .I2(d[20]), .I3(d[21]), .I4(d[22]), .I5(d[23]), .O(z_g[3]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001)) u_zg4 (.I0(d[24]), .I1(d[25]), .I2(d[26]), .I3(d[27]), .I4(d[28]), .I5(d[29]), .O(z_g[4]));
			(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h1))                 u_zg5 (.I0(d[30]), .I1(d[31]), .O(z_g[5]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h7FFFFFFFFFFFFFFF)) u_nz32 (.I0(z_g[0]), .I1(z_g[1]), .I2(z_g[2]), .I3(z_g[3]), .I4(z_g[4]), .I5(z_g[5]), .O(pchk_nz32[g_pc]));

			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFF8)) u_sb0 (.I0(d[8]),  .I1(d[9]),  .I2(d[10]), .I3(d[11]), .I4(d[12]), .I5(d[13]), .O(b_g[0]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb1 (.I0(d[14]), .I1(d[15]), .I2(d[16]), .I3(d[17]), .I4(d[18]), .I5(d[19]), .O(b_g[1]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb2 (.I0(d[20]), .I1(d[21]), .I2(d[22]), .I3(d[23]), .I4(d[24]), .I5(d[25]), .O(b_g[2]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFFFFFFFFE)) u_sb3 (.I0(d[26]), .I1(d[27]), .I2(d[28]), .I3(d[29]), .I4(d[30]), .I5(d[31]), .O(b_g[3]));
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hFFFE))            u_sb  (.I0(b_g[0]), .I1(b_g[1]), .I2(b_g[2]), .I3(b_g[3]), .O(pchk_set_bad[g_pc]));

			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001))            u_z118  (.I0(d[8]), .I1(d[9]), .I2(d[10]), .I3(d[11]), .O(d_z11_8));
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h7FFFFFFF))        u_nz24  (.I0(z_g[2]), .I1(z_g[3]), .I2(z_g[4]), .I3(z_g[5]), .I4(d_z11_8), .O(pchk_nz24[g_pc]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h00000600005FFC10)) u_offok (.I0(d[2]), .I1(d[3]), .I2(d[4]), .I3(d[5]), .I4(d[6]), .I5(d[7]), .O(off_hi_ok));
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEF))               u_obad  (.I0(d[0]), .I1(d[1]), .I2(off_hi_ok), .O(pchk_off_bad[g_pc]));

			genvar g_kb;
			for (g_kb = 0; g_kb < 5; g_kb = g_kb + 1) begin : g_qk_lo
				(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h40)) u_qk (.I0(pchk_set_bad[g_pc]), .I1(pchk_nz32[g_pc]), .I2(d[g_kb + 3]), .O(pchk_qc_k[g_pc][g_kb]));
			end
			for (g_kb = 5; g_kb < 7; g_kb = g_kb + 1) begin : g_qk_hi
				(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qk (.I0(pchk_set_bad[g_pc]), .I1(pchk_nz32[g_pc]), .I2(d[g_kb + 3]), .O(pchk_qc_k[g_pc][g_kb]));
			end
		end
	endgenerate
	wire pay_nz32_c, qc_zero_c, pay_set_bad_c, pay_dset_bad_c;
	wire [6:0] qc_total_k_c;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_pnz_m (.I0(pchk_nz32[0]),    .I1(pchk_nz32[1]),    .I2(use_fw), .O(pay_nz32_c));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h35)) u_qcz_m (.I0(pchk_nz32[0]),    .I1(pchk_nz32[1]),    .I2(use_fw), .O(qc_zero_c));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_psb_m (.I0(pchk_set_bad[0]), .I1(pchk_set_bad[1]), .I2(use_fw), .O(pay_set_bad_c));
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hFFF0EEEE)) u_pdb_m (
		.I0(pchk_nz24[0]), .I1(pchk_off_bad[0]), .I2(pchk_nz24[1]), .I3(pchk_off_bad[1]), .I4(use_fw), .O(pay_dset_bad_c)
	);
	genvar g_qm;
	generate
		for (g_qm = 0; g_qm < 7; g_qm = g_qm + 1) begin : g_qck_mux
			(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hCA)) u_qckm (.I0(pchk_qc_k[0][g_qm]), .I1(pchk_qc_k[1][g_qm]), .I2(use_fw), .O(qc_total_k_c[g_qm]));
		end
	endgenerate
`else
	function [9:0] fn_pay_chk;
		input [31:0] d;
		reg z0, z1, z2, z3, z4, z5, z6, z7, z8, z9, z10;
		reg n_a, n_b, n_c, n_d, hi24_lo10_ok, z32_lo_n;
		reg s_a, s_b, s_c, s_hi_ok;
		reg nz32, set_bad, t0, t1, t2, t3a, t3b, off_g0, off_g1, off_ok, dset_bad;
		reg [7:0] lo;
		reg [6:0] qk;
		begin
			/* 3-input NOR3 -> NAND3 -> NOR3 exact trees for cmos3 (<= 4 gates for 32 bits). */
			z0  = !(d[0]  | d[1]  | d[8]);
			z1  = !(d[9]  | d[10] | d[11]);
			z2  = !(d[12] | d[13] | d[14]);
			z3  = !(d[15] | d[16] | d[17]);
			z4  = !(d[18] | d[19] | d[20]);
			z5  = !(d[21] | d[22] | d[23]);
			z6  = !(d[24] | d[25] | d[26]);
			z7  = !(d[27] | d[28] | d[29]);
			z8  = !(d[30] | d[31]);
			z9  = !(d[2]  | d[3]  | d[4]);
			z10 = !(d[5]  | d[6]  | d[7]);
			n_a = ~(z0 & z1 & z2);
			n_b = ~(z3 & z4 & z5);
			n_c = ~(z6 & z7 & z8);
			n_d = ~(z9 & z10);
			hi24_lo10_ok = !(n_a | n_b | n_c);
			z32_lo_n = n_c | n_d;
			nz32 = !(!(n_a | n_b) & !z32_lo_n);

			s_a = !(!(d[10] | d[11] | d[12]) & !(d[13] | d[14] | d[15]) & !(d[16] | d[17] | d[18]));
			s_b = !(!(d[19] | d[20] | d[21]) & !(d[22] | d[23] | d[24]) & !(d[25] | d[26] | d[27]));
			s_c = !(!(d[28] | d[29] | d[30]) & !d[31]);
			s_hi_ok = !(s_a | s_b | s_c);
			set_bad = !s_hi_ok || (d[9] & d[8]);

			lo = d[7:0];
			t0     = !(~(!lo[7] & !lo[6] & !lo[5]) | ~(lo[4] & !lo[3] & !lo[2]));
			t1     = !(~(!lo[7] & !lo[6] &  lo[5]) | (!lo[4] & !lo[3]));
			t2     = !(~(!lo[7] &  lo[6] & !lo[5]) | (lo[4] & lo[2]));
			t3a    = !(~( lo[7] & !lo[6] &  lo[5]) | ~(!lo[4] &  lo[3] & !lo[2]));
			t3b    = !(~( lo[7] & !lo[6] &  lo[5]) | ~(!lo[4] & !lo[3] &  lo[2]));
			off_g0 = !(t0 | t1 | t2);
			off_g1 = !(t3a | t3b);
			off_ok = !(off_g0 & off_g1);
			dset_bad = !(hi24_lo10_ok & off_ok);
			qk = set_bad ? 7'd96 : d[9:3];
			fn_pay_chk = {qk, dset_bad, set_bad, nz32};
		end
	endfunction
	wire [9:0] n_chk_c  = fn_pay_chk(n_data);
	wire [9:0] fw_chk_c = fn_pay_chk(fw_data);
	wire pay_nz32_c     = use_fw ? fw_chk_c[0]   : n_chk_c[0];
	wire pay_set_bad_c  = use_fw ? fw_chk_c[1]   : n_chk_c[1];
	wire pay_dset_bad_c = use_fw ? fw_chk_c[2]   : n_chk_c[2];
	wire qc_zero_c      = !pay_nz32_c;
	wire [6:0] qc_total_k_c = use_fw ? fw_chk_c[9:3] : n_chk_c[9:3];
	wire n_al_err_c  = !(!(n_data[0]  | n_data[1]  | n_data[2])  & !(n_data[3]  | n_data[4]  | n_data[5]));
	wire fw_al_err_c = !(!(fw_data[0] | fw_data[1] | fw_data[2]) & !(fw_data[3] | fw_data[4] | fw_data[5]));
	wire pay_align_bad_c = use_fw ? fw_al_err_c : n_al_err_c;
`endif
	wire qca_ce_lo, qca_ce_hi;
`ifdef SYNTH_KINTEX7
	wire st_exec_qca_lo, st_exec_qca_hi;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_sqcalo (.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_exec_qca_lo));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_sqcahi (.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_exec_qca_hi));
	wire qca_wait_cond = (st_c == Q_QCAPS_WAIT) && !f_busy;
	wire qca_exec_cond_lo = st_exec_qca_lo && !seq_abort_b[0] && !nop_r && is_query_caps_r && !qc_align_err && !qc_zero_r;
	wire qca_exec_cond_hi = st_exec_qca_hi && !seq_abort_b[1] && !nop_r && is_query_caps_r && !qc_align_err && !qc_zero_r;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qcalo (.I0(qca_wait_cond), .I1(qca_exec_cond_lo), .I2(exec_go_b[0]), .O(qca_ce_lo));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qcahi (.I0(qca_wait_cond), .I1(qca_exec_cond_hi), .I2(exec_go_b[0]), .O(qca_ce_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire st_exec_qca_lo, st_exec_qca_hi;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_sqcalo (.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_exec_qca_lo));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_sqcahi (.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_exec_qca_hi));
	wire qca_wait_cond = (st_c == Q_QCAPS_WAIT) && !f_busy;
	wire qca_exec_cond_lo = st_exec_qca_lo && !seq_abort_b[0] && !nop_r && is_query_caps_r && !qc_align_err && !qc_zero_r;
	wire qca_exec_cond_hi = st_exec_qca_hi && !seq_abort_b[1] && !nop_r && is_query_caps_r && !qc_align_err && !qc_zero_r;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qcalo (.I0(qca_wait_cond), .I1(qca_exec_cond_lo), .I2(exec_go_b[0]), .O(qca_ce_lo));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'hEA)) u_qcahi (.I0(qca_wait_cond), .I1(qca_exec_cond_hi), .I2(exec_go_b[0]), .O(qca_ce_hi));
`else
	wire qca_exec_en = (st_c == Q_EXEC) && !seq_abort && !nop_r && is_query_caps_r && !qc_align_err && !qc_zero_r;
	wire qca_wait_en = (st_c == Q_QCAPS_WAIT) && !f_busy;
	wire st_exec_qca_lo = (st_c == Q_EXEC);
	wire st_exec_qca_hi = (st_c == Q_EXEC);
	assign qca_ce_lo = qca_exec_en || qca_wait_en;
	assign qca_ce_hi = qca_exec_en || qca_wait_en;
`endif

`ifdef SYNTH_KINTEX7
	wire bad_idx_comb;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF8880000)) u_bad_idx (
		.I0(pay_set_bad_r), .I1(is_set_r1),
		.I2(pay_dset_bad_r), .I3(is_dset_r1),
		.I4(pay_chk_v_r),
		.O(bad_idx_comb)
	);
	wire bad_abs_raw;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_bad_abs (
		.I0(pay_chk_p0_r), .I1(is_sopen_r1), .I2(pay_nz32_r), .O(bad_abs_raw)
	);
	wire bad_absent_comb = (HAS_HW_WALKER == 0) ? bad_abs_raw : 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire bad_idx_comb;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF8880000)) u_bad_idx (
		.I0(pay_set_bad_r), .I1(is_set_r1),
		.I2(pay_dset_bad_r), .I3(is_dset_r1),
		.I4(pay_chk_v_r),
		.O(bad_idx_comb)
	);
	wire bad_abs_raw;
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_bad_abs (
		.I0(pay_chk_p0_r), .I1(is_sopen_r1), .I2(pay_nz32_r), .O(bad_abs_raw)
	);
	wire bad_absent_comb = (HAS_HW_WALKER == 0) ? bad_abs_raw : 1'b0;
`else
	wire bad_idx_comb = pay_chk_v_r && (
		(is_set_r1 && pay_set_bad_r) ||
		(is_dset_r1 && pay_dset_bad_r)
	);
	wire bad_absent_comb = (HAS_HW_WALKER == 0) && pay_chk_p0_r && is_sopen_r1 && pay_nz32_r;
`endif

	(* max_fanout = 16 *) reg uop_we_r;
	(* max_fanout = 16 *) reg [9:0] uop_idx_r;
	(* max_fanout = 16 *) reg [31:0] uop_wdata_r;
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
			uop_we_r <= 1'b0;
			uop_idx_r <= 10'd0;
			uop_wdata_r <= 32'd0;
			pay_chk_v_r <= 1'b0;
			pay_chk_p0_r <= 1'b0;
			pay_set_bad_r <= 1'b0;
			pay_dset_bad_r <= 1'b0;
			pay_nz32_r <= 1'b0;
		end else begin
`ifdef SYNTH_KINTEX7
			uop_we_r <= pay_take_b[0] && pay_we && !bad_idx_comb && !bad_absent_comb && !nop_r;
`elsif SYNTH_KINTEX_ULTRASCALE
			uop_we_r <= pay_take_b[0] && pay_we && !bad_idx_comb && !bad_absent_comb && !nop_r;
`else
			uop_we_r <= (pay_hold_n && (pay_pre_go && !bad_idx_comb && !bad_absent_comb && !nop_r)) && pay_we;
`endif
			uop_idx_r <= pay_idx;
			uop_wdata_r <= in_data;
`ifdef SYNTH_KINTEX7
			pay_chk_v_r <= pay_take_b[0] && !p[0];
			pay_chk_p0_r <= pay_take_b[0] && p_is_0_r;
`elsif SYNTH_KINTEX_ULTRASCALE
			pay_chk_v_r <= pay_take_b[0] && !p[0];
			pay_chk_p0_r <= pay_take_b[0] && p_is_0_r;
`else
			pay_chk_v_r <= pay_hold_n && (pay_pre_go && !p[0]);
			pay_chk_p0_r <= pay_hold_n && (pay_pre_go && p_is_0_r);
`endif
			pay_set_bad_r <= pay_set_bad_c;
			pay_dset_bad_r <= pay_dset_bad_c;
			pay_nz32_r <= pay_nz32_c;
		end
	end
	assign uop_we = uop_we_r;
	assign uop_idx = uop_idx_r;
	assign uop_wdata = uop_wdata_r;

	/* A µop that starts a unit: BLIT, DRAW, DISPATCH, LOAD_CONST and class 1. */
	reg unit_op_dec_r, unit_op_su_dec_r;
	wire unit_op = unit_op_dec_r;
	reg unit_op_r;
	reg u_req_su_r;
	(* max_fanout = 16 *) reg fence_op_r;
	(* max_fanout = 16 *) reg is_wait_vsync_r;
	(* max_fanout = 16 *) reg [8:0] pre_r;
	(* max_fanout = 16 *) reg pre_f_r;
	(* max_fanout = 16 *) reg [8:0] post_r;
	(* max_fanout = 16 *) reg post_nz_r;
	reg [8:0] post_dec_r;
	reg post_nz_dec_r;
	reg k_on_dec_r;
	reg [2:0] k_code_dec_r;
	reg [3:0] inv_code_dec_r;
	reg exec_done_is_idle_r, exec_done_is_freq_r, exec_done_is_flush_r;
	reg [31:0] f_val_pre_r;
	reg st_exec_rdy_r;
	reg [3:0] exec_done_st_r;
	reg exec_settle_r;
	reg k_valid_r;
	reg [2:0] k_code_r;
	reg [3:0] inv_code_r;
	reg k_raw_dec_r, k_blt_dec_r, k_draw_dec_r, k_cs_dec_r;
	reg k_lc_dec_r, k_perf_dec_r, k_flip_dec_r, k_disp_dec_r;
	reg inv_draw_dec_r, inv_prog_dec_r, inv_inv_dec_r, is_cmd_fence_r;

`ifdef SYNTH_KINTEX7
	wire st_is_exec_ureq;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_ureq_ex (
		.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_is_exec_ureq)
	);
	assign u_req = (seq_abort || nop_r) ? 2'b00 : {st_is_exec_ureq && u_req_su_r, st_is_exec_ureq && unit_op_r};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire st_is_exec_ureq;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0004)) u_ureq_ex (
		.I0(st_c[0]), .I1(st_c[1]), .I2(st_c[2]), .I3(st_c[3]), .O(st_is_exec_ureq)
	);
	assign u_req = (seq_abort || nop_r) ? 2'b00 : {st_is_exec_ureq && u_req_su_r, st_is_exec_ureq && unit_op_r};
`else
	assign u_req = (seq_abort || nop_r) ? 2'b00 : {st_c == Q_EXEC && u_req_su_r, st_c == Q_EXEC && unit_op_r};
`endif
	assign u_fw = use_fw;
	assign u_side = side;

	wire exec_unit_op = unit_op_r;
	wire exec_fence_op = fence_op_r;
	wire exec_vsync_op = is_wait_vsync_r;
	wire [8:0] exec_pre = pre_r;
	wire exec_pre_f = pre_f_r;
	wire [8:0] exec_post = post_r;
	wire exec_post_nz = post_nz_r;

	wire pre_busy_0, pre_busy_1, pre_busy_2, ego_ok;
	wire [1:0] exec_go_b;

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb0 (
		.I0(exec_pre[0]), .I1(busy_in[0]), .I2(exec_pre[1]), .I3(busy_in[1]), .I4(exec_pre[2]), .I5(busy_in[2]),
		.O(pre_busy_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb1 (
		.I0(exec_pre[3]), .I1(busy_in[3]), .I2(exec_pre[4]), .I3(busy_in[4]), .I4(exec_pre[5]), .I5(busy_in[5]),
		.O(pre_busy_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb2 (
		.I0(exec_pre[6]), .I1(busy_in[6]), .I2(exec_pre[7]), .I3(busy_in[7]), .I4(exec_pre[8]), .I5(busy_in[8]),
		.O(pre_busy_2)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h2A2A002A)) u_ego_ok (
		.I0(st_exec_rdy_r), .I1(exec_pre_f), .I2(f_busy), .I3(exec_unit_op), .I4(u_ok),
		.O(ego_ok)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_ego0 (
		.I0(ego_ok), .I1(pre_busy_0), .I2(pre_busy_1), .I3(pre_busy_2),
		.O(exec_go_b[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_ego1 (
		.I0(ego_ok), .I1(pre_busy_0), .I2(pre_busy_1), .I3(pre_busy_2),
		.O(exec_go_b[1])
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb0 (
		.I0(exec_pre[0]), .I1(busy_in[0]), .I2(exec_pre[1]), .I3(busy_in[1]), .I4(exec_pre[2]), .I5(busy_in[2]),
		.O(pre_busy_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb1 (
		.I0(exec_pre[3]), .I1(busy_in[3]), .I2(exec_pre[4]), .I3(busy_in[4]), .I4(exec_pre[5]), .I5(busy_in[5]),
		.O(pre_busy_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFF888F888F888)) u_pb2 (
		.I0(exec_pre[6]), .I1(busy_in[6]), .I2(exec_pre[7]), .I3(busy_in[7]), .I4(exec_pre[8]), .I5(busy_in[8]),
		.O(pre_busy_2)
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h2A2A002A)) u_ego_ok (
		.I0(st_exec_rdy_r), .I1(exec_pre_f), .I2(f_busy), .I3(exec_unit_op), .I4(u_ok),
		.O(ego_ok)
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_ego0 (
		.I0(ego_ok), .I1(pre_busy_0), .I2(pre_busy_1), .I3(pre_busy_2),
		.O(exec_go_b[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0002)) u_ego1 (
		.I0(ego_ok), .I1(pre_busy_0), .I2(pre_busy_1), .I3(pre_busy_2),
		.O(exec_go_b[1])
	);
`else
	assign pre_busy_0 = |(exec_pre[2:0] & busy_in[2:0]);
	assign pre_busy_1 = |(exec_pre[5:3] & busy_in[5:3]);
	assign pre_busy_2 = |(exec_pre[8:6] & busy_in[8:6]);
	wire [8:0] pb_ok_w = ~(exec_pre & busy_in);
	wire pb_g0_n = ~&pb_ok_w[2:0];
	wire pb_g1_n = ~&pb_ok_w[5:3];
	wire pb_g2_n = ~&pb_ok_w[8:6];
	wire pb_all_ok = !(pb_g0_n | pb_g1_n | pb_g2_n);
	wire ego_f_ok_w = !(exec_pre_f && f_busy);
	wire ego_u_ok_w = !exec_unit_op || u_ok;
	assign ego_ok = st_exec_rdy_r && ego_f_ok_w && ego_u_ok_w;
	assign exec_go_b = {2{pb_all_ok && ego_ok}};
`endif
	wire exec_go = exec_go_b[0];
	wire is_op_std = !vendor && exec_go_b[1];

	wire k_raw  = k_raw_dec_r;
	wire k_blt  = k_blt_dec_r;
	wire k_draw = k_draw_dec_r;
	wire k_cs   = k_cs_dec_r;
	wire k_lc   = k_lc_dec_r;
	wire k_perf = k_perf_dec_r;
	wire k_flip = k_flip_dec_r;
	wire k_disp = k_disp_dec_r;
	wire k_on_dec = k_raw | k_blt | k_draw | k_cs | k_lc | k_perf | k_flip | (k_disp & flags[0]);
	wire [2:0] k_code_dec = {
		k_raw  | k_perf | k_flip | k_disp,
		k_cs   | k_lc   | k_flip | k_disp,
		k_draw | k_lc   | k_perf | k_disp
	};
	wire inv_draw = inv_draw_dec_r;
	wire inv_prog = inv_prog_dec_r;
	wire inv_inv  = inv_inv_dec_r;
	wire inv_fnc  = is_cmd_fence_r && fence_flags[2];
	wire [3:0] inv_code_dec =
		({4{inv_draw}} & 4'b0011) |
		({4{inv_prog}} & 4'b1000) |
		({4{inv_inv}}  & flags[3:0]) |
		({4{inv_fnc}}  & 4'b1111);

`ifdef SYNTH_KINTEX7
	assign kick = !seq_abort && !nop_r && exec_go_b[1] && k_valid_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	assign kick = !seq_abort && !nop_r && exec_go_b[1] && k_valid_r;
`else
	wire k_pre_ok_n = !(st_exec_rdy_r && !seq_abort && !nop_r && k_valid_r && ego_f_ok_w && ego_u_ok_w);
	assign kick = !(!(!(pb_g0_n | pb_g1_n) & !(pb_g2_n | k_pre_ok_n)));
`endif
	assign kick_code = k_code_r;
	assign inv = exec_go_b[1] ? inv_code_r : 4'd0;

	wire is_query = !vendor && op == OP_QUERY;
	wire fence_op = !vendor && (op == OP_FENCE || op == OP_WAIT_FENCE || op == OP_RESULT ||
		(op == OP_QUERY && !flags[0]) || op == CMD_FENCE || op == CMD_WAIT_FENCE);
	wire [3:0] is_q_b;
	wire [1:0] is_fc_b;
	wire [1:0] is_fcr_b;

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_iq0 (.I0(is_query_r), .O(is_q_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_iq1 (.I0(is_query_r), .O(is_q_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_iq2 (.I0(is_query_r), .O(is_q_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_iq3 (.I0(is_query_r), .O(is_q_b[3]));

	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ifc0 (.I0(is_fence_cmd_r), .O(is_fc_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ifc1 (.I0(is_fence_cmd_r), .O(is_fc_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ifcr0 (.I0(is_fc_or_res_r), .O(is_fcr_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ifcr1 (.I0(is_fc_or_res_r), .O(is_fcr_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign is_q_b[0] = is_query_r;
	assign is_q_b[1] = is_query_r;
	assign is_q_b[2] = is_query_r;
	assign is_q_b[3] = is_query_r;

	assign is_fc_b[0] = is_fence_cmd_r;
	assign is_fc_b[1] = is_fence_cmd_r;

	assign is_fcr_b[0] = is_fc_or_res_r;
	assign is_fcr_b[1] = is_fc_or_res_r;
`else
	assign is_q_b = {4{is_query_r}};
	assign is_fc_b = {2{is_fence_cmd_r}};
	assign is_fcr_b = {2{is_fc_or_res_r}};
`endif


`ifdef SYNTH_KINTEX7
	wire [3:0] cfws_b;
	wire cfws_w0_lo, cfws_w0_hi, cfws_w1_lo, cfws_w1_hi;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws0 (.I0(caps_fw_stall), .O(cfws_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws1a (.I0(caps_fw_stall), .O(cfws_w0_lo));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws1b (.I0(caps_fw_stall), .O(cfws_w0_hi));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws2 (.I0(caps_fw_stall), .O(cfws_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws3a (.I0(caps_fw_stall), .O(cfws_w1_lo));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_cfws3b (.I0(caps_fw_stall), .O(cfws_w1_hi));
	assign cfws_b[1] = cfws_w0_lo;
	assign cfws_b[3] = cfws_w1_lo;
	wire [2:0] f_ready_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frdy0 (.I0(f_ready), .O(f_ready_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frdy1 (.I0(f_ready), .O(f_ready_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frdy2 (.I0(f_ready), .O(f_ready_b[2]));
	wire [2:0] p_is0_ce_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce0 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce1 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce2 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[2]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [3:0] cfws_b;
	wire cfws_w0_lo, cfws_w0_hi, cfws_w1_lo, cfws_w1_hi;
	assign cfws_b[0] = caps_fw_stall;
	assign cfws_w0_lo = caps_fw_stall;
	assign cfws_w0_hi = caps_fw_stall;
	assign cfws_b[2] = caps_fw_stall;
	assign cfws_w1_lo = caps_fw_stall;
	assign cfws_w1_hi = caps_fw_stall;
	assign cfws_b[1] = cfws_w0_lo;
	assign cfws_b[3] = cfws_w1_lo;
	wire [2:0] f_ready_b;
	assign f_ready_b[0] = f_ready;
	assign f_ready_b[1] = f_ready;
	assign f_ready_b[2] = f_ready;
	wire [2:0] p_is0_ce_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce0 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce1 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_pi0ce2 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(p_is0_ce_b[2]));
`else
	wire [3:0] cfws_b = {4{caps_fw_stall}};
	wire cfws_w0_lo = caps_fw_stall;
	wire cfws_w0_hi = caps_fw_stall;
	wire cfws_w1_lo = caps_fw_stall;
	wire cfws_w1_hi = caps_fw_stall;
	wire [2:0] f_ready_b = {3{f_ready}};
	wire p_is0_ce_pre = pay_pre_go && p_is_0_r;
	wire [2:0] p_is0_ce_b = {3{pay_hold_n && p_is0_ce_pre}};
`endif
`ifdef SYNTH_KINTEX7
	assign occ_clear = (is_op_std && is_query_r && flags[0]) ||
		(st_c == Q_FREQ && f_ready_b[0] && is_query_r && flags[1]);
`elsif SYNTH_KINTEX_ULTRASCALE
	assign occ_clear = (is_op_std && is_query_r && flags[0]) ||
		(st_c == Q_FREQ && f_ready_b[0] && is_query_r && flags[1]);
`else
	reg st_is_freq_r;
	wire occ_q0_ok_n = !(st_exec_rdy_r && !vendor && is_query_r && flags[0] && ego_f_ok_w && ego_u_ok_w);
	wire occ_exec_c  = !(pb_g0_n | pb_g1_n) && !(pb_g2_n | occ_q0_ok_n);
	assign occ_clear = occ_exec_c || (st_is_freq_r && f_ready_b[0] && is_query_r && flags[1]);
`endif
	assign f_valid = !seq_abort && f_valid_r;
	assign f_op = is_fop_fence_r ? 2'd0 : (is_fop_res_q_r || is_qc_req_fop_r) ? 2'd1 : 2'd2;
	assign f_addr = {
		is_qc_req_fa1_hi_r ? qc_cur_addr_hi[31:16] : (is_fc_b[1] ? a0_hi[31:16] : 16'd0),
		is_qc_req_fa1_lo_r ? qc_cur_addr_hi[15:0]  : (is_fc_b[0] ? a0_hi[15:0]  : 16'd0),
		is_qc_req_fa0_b3_r  ? qc_cur_addr[31:24]    : a0[31:24],
		is_qc_req_fa0_b2_r  ? qc_cur_addr[23:16]    : a0[23:16],
		is_qc_req_fa0_b1_r  ? qc_cur_addr[15:8]     : a0[15:8],
		is_qc_req_fa0_b0a_r ? qc_cur_addr[7:5]      : a0[7:5],
		is_qc_req_fa0_b0_r  ? qc_cur_addr[4:0]      : a0[4:0]
	};
`ifdef SYNTH_KINTEX7
	wire [31:0] f_value_w = {
		is_qc_req_fv1_r ? qc_w0[31:16] : (is_q_b[1] ? occ_q[31:16] : a1[31:16]),
		is_qc_req_fv0_r ? qc_w0[15:0]  : (is_q_b[0] ? occ_q[15:0]  : a1[15:0])
	};
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [31:0] f_value_w = {
		is_qc_req_fv1_r ? qc_w0[31:16] : (is_q_b[1] ? occ_q[31:16] : a1[31:16]),
		is_qc_req_fv0_r ? qc_w0[15:0]  : (is_q_b[0] ? occ_q[15:0]  : a1[15:0])
	};
`else
	wire [31:0] f_value_w = {
		is_q_b[1] ? occ_q[31:16] : f_val_pre_r[31:16],
		is_q_b[0] ? occ_q[15:0]  : f_val_pre_r[15:0]
	};
`endif
	assign f_value = f_value_w;
	assign f_aux = {
		is_qc_req_fx1_r ? qc_w1[31:16] : (is_q_b[3] ? 16'd0                : (is_fcr_b[1] ? a2[31:16] : 16'd0)),
		is_qc_req_fx0_r ? qc_w1[15:0]  : (is_q_b[2] ? {8'd0, occ_q[39:32]} : (is_fcr_b[0] ? a2[15:0]  : 16'd0))
	};
	assign f_irq = is_qc_req_fop_r ? 1'b0 : (is_fence_cmd_r ? fence_flags[0] : flags[0]);
	assign f_mem = is_qc_req_fop_r ? 1'b1 : (is_fence_cmd_r ? fence_flags[1] : 1'b1);
	assign f_flush = f_flush_r;
	assign f_64 = is_fence_cmd_r || is_qc_req_f64_r;

	reg f_pulse;
	reg [3:0] f_kind;
	reg [31:0] f_faddr;

	/* Pipelined header decode (Q_DEC -> Q_DEC2) so hdr_r -> FF is <= 3 LUTs */
	wire [3:0] h_lo_dec = hdr_r[3:0];
	wire gh_r1 = (hdr_r[7:4] == 4'h1);
	wire gh_r2 = (hdr_r[7:4] == 4'h2);
	wire gh_r3 = (hdr_r[7:4] == 4'h3);
	wire gh_r4 = (hdr_r[7:4] == 4'h4);
	wire gh_r5 = (hdr_r[7:4] == 4'h5);
	wire gh_abs1 = (h_lo_dec[3:1] == 3'd0) && (HAS_HW_WALKER == 0);
	wire gh_abs2 = (((h_lo_dec == 4'h7) || (h_lo_dec[3:1] == 3'b100)) && !UNITS[2]) ||
		((h_lo_dec == 4'ha) && (!UNITS[6] || HAS_MESH == 0)) ||
		(((h_lo_dec == 4'hd) || (h_lo_dec == 4'he)) && !UNITS[6]);
	wire gh_abs3 = ((h_lo_dec == 4'h0) && !UNITS[6]) ||
		((h_lo_dec == 4'h2) && (HAS_SECONDARY_RING == 0)) ||
		((h_lo_dec[3:1] == 3'b011) && !UNITS[4]);
	wire gh_abs4 = !h_lo_dec[3] && (h_lo_dec[2:0] != 3'd7) && !UNITS[1];
	wire gh_abs5 = (h_lo_dec[3:1] == 3'd0) && (HAS_DISP == 0);
	wire ghci_absent_op = (gh_r1 & gh_abs1) | (gh_r2 & gh_abs2) | (gh_r3 & gh_abs3) | (gh_r4 & gh_abs4) | (gh_r5 & gh_abs5);
	wire gh_bad_other = hdr_r[7] || (hdr_r[6] ? (hdr_r[5] || !hdr_r[4]) : (!hdr_r[5] && !hdr_r[4]));
	wire gh_bad1 = h_lo_dec[3] || (h_lo_dec[2] && h_lo_dec[1]);
	wire gh_bad2 = (h_lo_dec == 4'hf);
	wire gh_bad3 = h_lo_dec[3];
	wire gh_bad4 = h_lo_dec[3] || (h_lo_dec[2] && h_lo_dec[1] && h_lo_dec[0]);
	wire gh_bad5 = |h_lo_dec[3:1];
	wire gh_flt1 = gh_abs1 || gh_bad1;
	wire gh_flt2 = gh_abs2 || gh_bad2;
	wire gh_flt3 = gh_abs3 || gh_bad3;
	wire gh_flt4 = gh_abs4 || gh_bad4;
	wire gh_flt5 = gh_abs5 || gh_bad5;
	wire gh_f1 = ~(gh_r1 & gh_flt1);
	wire gh_f2 = ~(gh_r2 & gh_flt2);
	wire gh_f3 = ~(gh_r3 & gh_flt3);
	wire gh_f4 = ~(gh_r4 & gh_flt4);
	wire gh_f5 = ~(gh_r5 & gh_flt5);
	wire ghci_fault_op = !(!gh_bad_other & (gh_f1 & gh_f2 & gh_f3) & (gh_f4 & gh_f5));

	wire [3:0] gm_r1 = {
		(h_lo_dec == 4'h0) || (h_lo_dec == 4'h1),
		(h_lo_dec == 4'h2) || (h_lo_dec == 4'h5),
		(h_lo_dec == 4'h0) || (h_lo_dec == 4'h3) || (h_lo_dec == 4'h4),
		(h_lo_dec == 4'h5)
	};
	wire [3:0] gm_r2 = {
		h_lo_dec == 4'hb,
		(!h_lo_dec[3] & (h_lo_dec[2] | h_lo_dec[1])) | (h_lo_dec[3] & !h_lo_dec[0] & (!h_lo_dec[2] | h_lo_dec[1])),
		h_lo_dec[0] ? (h_lo_dec[3] & !h_lo_dec[2] & h_lo_dec[1]) : !((!h_lo_dec[2] & h_lo_dec[1]) | (h_lo_dec[3] & h_lo_dec[2] & !h_lo_dec[1])),
		!h_lo_dec[3] & ((h_lo_dec[2] & h_lo_dec[0]) | (!h_lo_dec[1] & !h_lo_dec[0]))
	};
	wire [3:0] gm_r3 = {1'b0, !h_lo_dec[3], (h_lo_dec == 4'h3), (h_lo_dec == 4'h4)};
	wire [3:0] gm_r4 = {
		!h_lo_dec[3] && (h_lo_dec[2:0] != 3'd3) && (h_lo_dec[2:0] != 3'd7),
		(h_lo_dec == 4'h2) || (h_lo_dec == 4'h3),
		(h_lo_dec == 4'h1) || (h_lo_dec == 4'h4) || (h_lo_dec == 4'h5) || (h_lo_dec == 4'h6),
		(h_lo_dec == 4'h1) || (h_lo_dec == 4'h3) || (h_lo_dec == 4'h4)
	};
	wire [3:0] gm_r5 = {(h_lo_dec == 4'h1), 3'b000};
	wire [3:0] gm_t1 = ~({4{gh_r1}} & gm_r1);
	wire [3:0] gm_t2 = ~({4{gh_r2}} & gm_r2);
	wire [3:0] gm_t3 = ~({4{gh_r3}} & gm_r3);
	wire [3:0] gm_t4 = ~({4{gh_r4}} & gm_r4);
	wire [3:0] gm_t5 = ~({4{gh_r5}} & gm_r5);
	wire [3:0] gm_g0 = ~(gm_t1 & gm_t2 & gm_t3);
	wire [3:0] gm_g1 = ~(gm_t4 & gm_t5);
	wire [3:0] ghci_min_n = gm_g0 | gm_g1;

	wire [5:0] s_op_dec = hdr_r[29:24];
	wire s_v_absent = !hdr_r[31] && hdr_r[30] && (s_op_dec == 6'd0) && !UNITS[3];
	wire s_v_fault  = s_v_absent || (hdr_r[30] && (s_op_dec[5:1] != 5'd0));
	wire [4:0] s_c_op = hdr_r[28:24];
	wire s_c_absent_lo =
		((s_c_op == 5'd2) && !UNITS[1]) ||
		((s_c_op == 5'd3) && !UNITS[2]) ||
		(((s_c_op == 5'd4) || (s_c_op == 5'd7)) && !UNITS[6]) ||
		((s_c_op == 5'd11) && (HAS_VSYNC == 0)) ||
		((s_c_op == 5'd14) && !UNITS[4]) ||
		(((s_c_op == 5'd15) || (s_c_op == 5'd16)) && (HAS_DISP == 0));
	wire s_c_good_lo = !s_c_op[4] || !(s_c_op[3] | s_c_op[2] | s_c_op[1] | s_c_op[0]) ||
		(s_c_op[4] && !s_c_op[3] && s_c_op[2] && !(s_c_op[1] && s_c_op[0]));
	wire s_c_bad_lo = !s_c_good_lo;
	wire [2:0] std_min_c_lo3 = {
		s_c_op[4] && !s_c_op[3] && ((!s_c_op[2] && !s_c_op[1] && !s_c_op[0]) || (s_c_op[2] && (s_c_op[1] ^ s_c_op[0]))),
		!s_c_op[4] && ((!s_c_op[3] && s_c_op[2] && (!s_c_op[1] || s_c_op[0])) || (s_c_op[3] && !s_c_op[1] && s_c_op[0])),
		(s_c_op[4] && !s_c_op[3] && s_c_op[2] && !s_c_op[1] && !s_c_op[0]) ||
		(!s_c_op[4] && !s_c_op[3] && s_c_op[2] && !(s_c_op[1] ^ s_c_op[0])) ||
		(!s_c_op[4] &&  s_c_op[3] && ((!s_c_op[2] && !s_c_op[1] && !s_c_op[0]) || (s_c_op[2] && (s_c_op[1] ^ s_c_op[0]))))
	};
	wire [3:0] std_min_c_lo = {1'b0, std_min_c_lo3};
	wire s_c_absent = !hdr_r[31] && !hdr_r[30] && !hdr_r[29] && s_c_absent_lo;
	wire s_c_bad    = hdr_r[29] || s_c_bad_lo;
	wire std_absent_op = hdr_r[30] ? s_v_absent : s_c_absent;
	wire std_fault_op  = hdr_r[31] || (hdr_r[30] ? s_v_fault : (s_c_absent || s_c_bad));
	wire [3:0] std_min_v_w = (hdr_r[29:24] == 6'd0) ? 4'd9 : ((hdr_r[29:24] == 6'd1) ? 4'd1 : 4'd0);
	wire [3:0] std_min_n   = hdr_r[31] ? 4'd0 : (hdr_r[30] ? std_min_v_w : (hdr_r[29] ? 4'd0 : std_min_c_lo));
	wire disp_op_hdr_c = is_ghci_hdr_r ? (hdr_r[7:1] == 7'h28) :
		(!hdr_r[31] && !hdr_r[30] && (hdr_r[29:24] == 6'd15 || hdr_r[29:24] == 6'd16));

	reg h_early_fault_r, h_early_absent_r, h_early_short_r, h_nhi_z_r;
	reg ghci_fault_r, std_fault_r, disp_fw_fault_r;
	reg [3:0] h_min_r, h_nlo_r;
	reg [3:0] ghci_min_r, std_min_r;
	reg dec_short_r2;
`ifdef SYNTH_KINTEX7
	wire dec_short_s = h_early_short_r || (h_nhi_z_r && (h_nlo_r < h_min_r));
	wire dec_fault_s = h_early_fault_r || dec_short_s;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire dec_short_s = h_early_short_r || (h_nhi_z_r && (h_nlo_r < h_min_r));
	wire dec_fault_s = h_early_fault_r || dec_short_s;
`else
	wire ghci_lt_w = (h_nlo_r < ghci_min_r);
	wire std_lt_w  = (h_nlo_r < std_min_r);
	wire dec_short_s = h_early_short_r || (h_nhi_z_r && (is_ghci_hdr_r ? ghci_lt_w : std_lt_w));
	wire dec_fault_s = disp_fw_fault_r || (is_ghci_hdr_r ? ghci_fault_r : std_fault_r) || dec_short_s;
`endif

	(* max_fanout = 16 *) reg dec2_wait_r;
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high)
			dec2_wait_r <= 1'b0;
		else if (st_c == Q_DEC2)
			dec2_wait_r <= !dec2_wait_r;
		else
			dec2_wait_r <= 1'b0;
	end
	wire st_is_dec_raw = (st_c == Q_DEC);
	wire [8:0] st_dec_b;
	wire st_is_dec2_raw = (st_c == Q_DEC2) && !dec2_wait_r;
	wire [2:0] st_dec2_b;
	wire [7:0] h_op_a, h_op_b, h_op_b2;
	wire [1:0] sidx_we_b, f_fa_we_b, a0_we_b, a0_hi_we_b, a1_we_b, a2_we_b, fflags_we_b;

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec0 (.I0(st_is_dec_raw), .O(st_dec_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec1 (.I0(st_is_dec_raw), .O(st_dec_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec2 (.I0(st_is_dec_raw), .O(st_dec_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec3 (.I0(st_is_dec_raw), .O(st_dec_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec4 (.I0(st_is_dec_raw), .O(st_dec_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec5 (.I0(st_is_dec_raw), .O(st_dec_b[5]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec6 (.I0(st_is_dec_raw), .O(st_dec_b[6]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec7 (.I0(st_is_dec_raw), .O(st_dec_b[7]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sdec8 (.I0(st_is_dec_raw), .O(st_dec_b[8]));

	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sd2_0 (.I0(st_is_dec2_raw), .O(st_dec2_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sd2_1 (.I0(st_is_dec2_raw), .O(st_dec2_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sd2_2 (.I0(st_is_dec2_raw), .O(st_dec2_b[2]));

	genvar g_hop;
	generate
		for (g_hop = 0; g_hop < 8; g_hop = g_hop + 1) begin : gen_hop_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_hopa  (.I0(h_op[g_hop]), .O(h_op_a[g_hop]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_hopb  (.I0(h_op[g_hop]), .O(h_op_b[g_hop]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_hopb2 (.I0(h_op[g_hop]), .O(h_op_b2[g_hop]));
		end
	endgenerate

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2220)) u_swe0 (
		.I0(pay_take_b[1]), .I1(p[0]), .I2(is_set_r), .I3(is_dset_r), .O(sidx_we_b[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2220)) u_swe1 (
		.I0(pay_take_b[1]), .I1(p[0]), .I2(is_set_r), .I3(is_dset_r), .O(sidx_we_b[1])
	);

	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h2)) u_fawe0 (.I0(pay_take_b[1]), .I1(p[0]), .O(f_fa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h2)) u_fawe1 (.I0(pay_take_b[1]), .I1(p[0]), .O(f_fa_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_a0we0 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(a0_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_a0we1 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(a0_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_a0hwe0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_1_r), .O(a0_hi_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_a0hwe1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_1_r), .O(a0_hi_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a1we0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_2_r), .I3(p_is_1_r), .O(a1_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a1we1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_2_r), .I3(p_is_1_r), .O(a1_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a2we0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_3_r), .I3(p_is_2_r), .O(a2_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a2we1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_3_r), .I3(p_is_2_r), .O(a2_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_ffwe0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_4_r), .O(fflags_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_ffwe1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_4_r), .O(fflags_we_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign st_dec_b[0] = st_is_dec_raw;
	assign st_dec_b[1] = st_is_dec_raw;
	assign st_dec_b[2] = st_is_dec_raw;
	assign st_dec_b[3] = st_is_dec_raw;
	assign st_dec_b[4] = st_is_dec_raw;
	assign st_dec_b[5] = st_is_dec_raw;
	assign st_dec_b[6] = st_is_dec_raw;
	assign st_dec_b[7] = st_is_dec_raw;
	assign st_dec_b[8] = st_is_dec_raw;

	assign st_dec2_b[0] = st_is_dec2_raw;
	assign st_dec2_b[1] = st_is_dec2_raw;
	assign st_dec2_b[2] = st_is_dec2_raw;

	genvar g_hop;
	generate
		for (g_hop = 0; g_hop < 8; g_hop = g_hop + 1) begin : gen_hop_buf
			assign h_op_a[g_hop] = h_op[g_hop];
			assign h_op_b[g_hop] = h_op[g_hop];
			assign h_op_b2[g_hop] = h_op[g_hop];
		end
	endgenerate

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2220)) u_swe0 (
		.I0(pay_take_b[1]), .I1(p[0]), .I2(is_set_r), .I3(is_dset_r), .O(sidx_we_b[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h2220)) u_swe1 (
		.I0(pay_take_b[1]), .I1(p[0]), .I2(is_set_r), .I3(is_dset_r), .O(sidx_we_b[1])
	);

	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h2)) u_fawe0 (.I0(pay_take_b[1]), .I1(p[0]), .O(f_fa_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h2)) u_fawe1 (.I0(pay_take_b[1]), .I1(p[0]), .O(f_fa_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_a0we0 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(a0_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_a0we1 (.I0(pay_take_b[2]), .I1(p_is_0_r), .O(a0_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_a0hwe0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_1_r), .O(a0_hi_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_a0hwe1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_1_r), .O(a0_hi_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a1we0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_2_r), .I3(p_is_1_r), .O(a1_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a1we1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_2_r), .I3(p_is_1_r), .O(a1_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a2we0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_3_r), .I3(p_is_2_r), .O(a2_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hA280)) u_a2we1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_3_r), .I3(p_is_2_r), .O(a2_we_b[1]));

	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_ffwe0 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_4_r), .O(fflags_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT3 #(.INIT(8'h80)) u_ffwe1 (.I0(pay_take_b[2]), .I1(is_fence_cmd_r), .I2(p_is_4_r), .O(fflags_we_b[1]));
`else
	assign st_dec_b = {9{st_is_dec_r}};
	assign st_dec2_b = {3{st_is_dec2_raw}};
	assign h_op_a = h_op;
	assign h_op_b = h_op;
	assign h_op_b2 = h_op;
	wire sidx_we_pre = pay_pre_go && !p[0] && (is_set_r || is_dset_r);
	wire f_fa_we_pre = pay_pre_go && !p[0];
	wire a0_we_pre   = pay_pre_go && p_is_0_r;
	wire a0_hi_pre   = pay_pre_go && is_fence_cmd_r && p_is_1_r;
	wire a1_we_pre   = pay_pre_go && (is_fence_cmd_r ? p_is_2_r : p_is_1_r);
	wire a2_we_pre   = pay_pre_go && (is_fence_cmd_r ? p_is_3_r : p_is_2_r);
	wire fflags_pre  = pay_pre_go && is_fence_cmd_r && p_is_4_r;
	assign sidx_we_b   = {2{pay_hold_n && sidx_we_pre}};
	assign f_fa_we_b   = {2{pay_hold_n && f_fa_we_pre}};
	assign a0_we_b     = {2{pay_hold_n && a0_we_pre}};
	assign a0_hi_we_b  = {2{pay_hold_n && a0_hi_pre}};
	assign a1_we_b     = {2{pay_hold_n && a1_we_pre}};
	assign a2_we_b     = {2{pay_hold_n && a2_we_pre}};
	assign fflags_we_b = {2{pay_hold_n && fflags_pre}};
`endif

	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
			a0 <= 32'd0;
			a0_hi <= 32'd0;
			a1 <= 32'd0;
			a2 <= 32'd0;
			fence_flags <= 32'd0;
			sidx <= 10'd0;
			f_faddr <= 32'd0;
		end else begin
			if (a0_we_b[0]) a0[15:0] <= in_data[15:0];
			if (a0_we_b[1]) a0[31:16] <= in_data[31:16];
			if (a0_hi_we_b[0]) a0_hi[15:0] <= in_data[15:0];
			if (a0_hi_we_b[1]) a0_hi[31:16] <= in_data[31:16];
			if (a1_we_b[0]) a1[15:0] <= in_data[15:0];
			if (a1_we_b[1]) a1[31:16] <= in_data[31:16];
			if (a2_we_b[0]) a2[15:0] <= in_data[15:0];
			if (a2_we_b[1]) a2[31:16] <= in_data[31:16];
			if (st_dec_b[6]) fence_flags[15:0] <= 16'd0;
			else if (fflags_we_b[0]) fence_flags[15:0] <= in_data[15:0];
			if (st_dec_b[7]) fence_flags[31:16] <= 16'd0;
			else if (fflags_we_b[1]) fence_flags[31:16] <= in_data[31:16];
			if (sidx_we_b[0]) sidx[4:0] <= is_dset_r ? in_data[6:2] : in_data[4:0];
			if (sidx_we_b[1]) sidx[9:5] <= is_dset_r ? {4'b1111, in_data[7]} : in_data[9:5];
			if (f_fa_we_b[0]) f_faddr[15:0] <= in_data[15:0];
			if (f_fa_we_b[1]) f_faddr[31:16] <= in_data[31:16];
		end
	end

	wire occ_we_raw = (st_c == Q_EXEC) && is_query_r;
	wire [2:0] occ_we_b;

`ifdef SYNTH_KINTEX7
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_occwe0 (.I0(occ_we_raw), .O(occ_we_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_occwe1 (.I0(occ_we_raw), .O(occ_we_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_occwe2 (.I0(occ_we_raw), .O(occ_we_b[2]));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign occ_we_b[0] = occ_we_raw;
	assign occ_we_b[1] = occ_we_raw;
	assign occ_we_b[2] = occ_we_raw;
`else
	assign occ_we_b = {3{occ_we_raw}};
`endif

	assign fault = f_pulse;
	assign fault_kind = f_kind;
	assign fault_addr = f_faddr;
	assign fault_tag = tag;
`ifdef SYNTH_KINTEX7
	assign busy = !(st_is_idle_r || (st_c == Q_DEAD));
	assign cmd_active = !(st_is_idle_r || (st_c == Q_DEAD) || (st_is_pay_r && use_fw && !fw_valid));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign busy = !(st_is_idle_r || (st_c == Q_DEAD));
	assign cmd_active = !(st_is_idle_r || (st_c == Q_DEAD) || (st_is_pay_r && use_fw && !fw_valid));
`else
	assign busy = !(st_is_idle_r | st_is_dead_r);
	assign cmd_active = !(st_is_idle_r | st_is_dead_r) && !(st_is_pay_r && use_fw && !fw_valid);
`endif
	assign seq_st = st_out_r;

	reg dec_fault_r2;
`ifdef SYNTH_KINTEX7
	wire pay_done_c;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000200000000)) u_pay_done (
		.I0(in_valid), .I1(pay_hold_0), .I2(pay_hold_1), .I3(pay_hold_2), .I4(seq_abort_b[2]), .I5(p_is_last_r),
		.O(pay_done_c)
	);
	wire [3:0] pay_nx_c;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0EFF0EFF)) u_pnx0 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0F0E0F0E)) u_pnx1 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[1])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0E0E0E0E)) u_pnx2 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[2])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000)) u_pnx3 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[3])
	);
	wire pay_is_idle_c;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF000F000)) u_p_idle (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_is_idle_c)
	);

	wire exec_spec_c = seq_abort_b[3] || nop_r || is_query_caps_r;
	wire [3:0] exec_alt_nx = seq_abort_b[3] ? Q_DEAD :
		nop_r ? Q_IDLE :
		(qc_align_err ? Q_DEAD : (qc_zero_r ? Q_IDLE : Q_QCAPS_R0));
	wire [3:0] exec_nx_c;
	genvar g_exn;
	generate
		for (g_exn = 0; g_exn < 4; g_exn = g_exn + 1) begin : g_exec_nx
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hCA00 | ({16{Q_EXEC[g_exn]}} & 16'h000F))) u_exnx (
				.I0(exec_done_st_r[g_exn]), .I1(exec_alt_nx[g_exn]), .I2(exec_spec_c), .I3(exec_go_b[0]),
				.O(exec_nx_c[g_exn])
			);
		end
	endgenerate
	wire exec_no_abort_idle = (nop_r || is_query_caps_r) ? (nop_r || (!qc_align_err && qc_zero_r)) : (exec_done_st_r == Q_IDLE);
	wire exec_is_idle_c = exec_go_b[0] && !seq_abort_b[3] && exec_no_abort_idle;
	wire exec_is_exec_c = !exec_spec_c && !exec_go_b[0];

	wire post_done_c = !settle && !(|(exec_post & busy_in));
	wire bar_done_c  = !settle && !pre_busy_0 && !pre_busy_1 && !pre_busy_2;
	wire [3:0] idle_nx = hdr_take_fsm ? Q_DEC : Q_IDLE;
	wire [3:0] post_nx = post_done_c ? Q_IDLE : Q_POST;
	wire [3:0] freq_nx = f_ready_b[1] ? (is_wait_fence_r ? Q_FWAIT : Q_IDLE) : Q_FREQ;
	wire [3:0] fwait_nx = !f_busy ? Q_IDLE : Q_FWAIT;
	wire [3:0] vsync_nx = vsync ? Q_IDLE : Q_VSYNC;
	wire [3:0] dec2_nx = dec2_wait_r ? (dec_fault_r2 ? Q_DEAD : (h_zero_r ? (is_barrier_r ? Q_BARRIER : Q_EXEC) : Q_PAY)) : Q_DEC2;
	wire [3:0] bar_nx  = bar_done_c ? Q_IDLE : Q_BARRIER;
	wire [3:0] flush_nx = l2_flush_done ? Q_FREQ : Q_FLUSH;
	wire [3:0] qcr0_nx  = (!cfws_b[0] && qc_r_rdy) ? Q_QCAPS_R1 : Q_QCAPS_R0;
	wire [3:0] qcr1_nx  = (!cfws_b[2] && qc_r_rdy) ? Q_QCAPS_REQ : Q_QCAPS_R1;
	wire [3:0] qcreq_nx = f_ready_b[2] ? Q_QCAPS_WAIT : Q_QCAPS_REQ;
	wire [3:0] qcwait_nx = !f_busy ? (qc_last_r ? Q_IDLE : Q_QCAPS_R0) : Q_QCAPS_WAIT;
	wire st_next_is_qcreq = ((st_c == Q_QCAPS_R1) && !cfws_b[2] && qc_r_rdy) || ((st_c == Q_QCAPS_REQ) && !f_ready_b[2]);

	wire [3:0] st_next;
	wire st_idle_next, st_exec_next;
	genvar g_st;
	generate
		for (g_st = 0; g_st < 4; g_st = g_st + 1) begin : g_st_tree
			wire nx_0, nx_1, nx_2, nx_3, other_nx;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx0 (
				.I0(idle_nx[g_st]), .I1(1'b0), .I2(1'b0), .I3(post_nx[g_st]),
				.I4(st_g0[0]), .I5(st_g0[1]), .O(nx_0)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx1 (
				.I0(freq_nx[g_st]), .I1(fwait_nx[g_st]), .I2(vsync_nx[g_st]), .I3(Q_DEAD[g_st]),
				.I4(st_g1[0]), .I5(st_g1[1]), .O(nx_1)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx2 (
				.I0(Q_DEC2[g_st]), .I1(dec2_nx[g_st]), .I2(bar_nx[g_st]), .I3(flush_nx[g_st]),
				.I4(st_g2[0]), .I5(st_g2[1]), .O(nx_2)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx3 (
				.I0(qcr0_nx[g_st]), .I1(qcr1_nx[g_st]), .I2(qcreq_nx[g_st]), .I3(qcwait_nx[g_st]),
				.I4(st_g3[0]), .I5(st_g3[1]), .O(nx_3)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_oth (
				.I0(nx_0), .I1(nx_1), .I2(nx_2), .I3(nx_3),
				.I4(st_g4[2]), .I5(st_g4[3]), .O(other_nx)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_stnxt (
				.I0(other_nx), .I1(pay_nx_c[g_st]), .I2(exec_nx_c[g_st]),
				.I3(st_is_pay_r), .I4(st_is_exec_r),
				.O(st_next[g_st])
			);
		end
	endgenerate
	wire id_0, id_1, id_2, id_3, id_oth;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id0 (
		.I0(1'b0), .I1(1'b0), .I2(1'b0), .I3(post_done_c),
		.I4(st_g0[0]), .I5(st_g0[1]), .O(id_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id1 (
		.I0(f_ready_b[1] && !is_wait_fence_r), .I1(!f_busy), .I2(vsync), .I3(1'b0),
		.I4(st_g1[0]), .I5(st_g1[1]), .O(id_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id2 (
		.I0(1'b0), .I1(1'b0), .I2(bar_done_c), .I3(1'b0),
		.I4(st_g2[0]), .I5(st_g2[1]), .O(id_2)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id3 (
		.I0(1'b0), .I1(1'b0), .I2(1'b0), .I3(!f_busy && qc_last_r),
		.I4(st_g3[0]), .I5(st_g3[1]), .O(id_3)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_idoth (
		.I0(id_0), .I1(id_1), .I2(id_2), .I3(id_3),
		.I4(st_g4[2]), .I5(st_g4[3]), .O(id_oth)
	);
	wire idle_stay_c = st_is_idle_r && (halt || seq_abort_b[2] || !hdr_valid);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFF0F0CCAA)) u_idnxt (
		.I0(id_oth), .I1(pay_is_idle_c), .I2(exec_is_idle_c),
		.I3(st_is_pay_r), .I4(st_is_exec_r), .I5(idle_stay_c),
		.O(st_idle_next)
	);
	wire ex_from_dec2 = dec2_wait_r && !dec_fault_r2 && h_zero_r && !is_barrier_r;
	wire ex_from_pay;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000100)) u_ex_pay (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(ex_from_pay)
	);
	wire ex_2 = (st_g2[1:0] == 2'b01) ? ex_from_dec2 : 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_exnxt (
		.I0(st_g4[3] && !st_g4[2] ? ex_2 : 1'b0), .I1(ex_from_pay), .I2(exec_is_exec_c),
		.I3(st_is_pay_r), .I4(st_is_exec_r),
		.O(st_exec_next)
	);
`elsif SYNTH_KINTEX_ULTRASCALE
	wire pay_done_c;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000200000000)) u_pay_done (
		.I0(in_valid), .I1(pay_hold_0), .I2(pay_hold_1), .I3(pay_hold_2), .I4(seq_abort_b[2]), .I5(p_is_last_r),
		.O(pay_done_c)
	);
	wire [3:0] pay_nx_c;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0EFF0EFF)) u_pnx0 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[0])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0F0E0F0E)) u_pnx1 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[1])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h0E0E0E0E)) u_pnx2 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[2])
	);
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h01000000)) u_pnx3 (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_nx_c[3])
	);
	wire pay_is_idle_c;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF000F000)) u_p_idle (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(pay_is_idle_c)
	);

	wire exec_spec_c = seq_abort_b[3] || nop_r || is_query_caps_r;
	wire [3:0] exec_alt_nx = seq_abort_b[3] ? Q_DEAD :
		nop_r ? Q_IDLE :
		(qc_align_err ? Q_DEAD : (qc_zero_r ? Q_IDLE : Q_QCAPS_R0));
	wire [3:0] exec_nx_c;
	genvar g_exn;
	generate
		for (g_exn = 0; g_exn < 4; g_exn = g_exn + 1) begin : g_exec_nx
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'hCA00 | ({16{Q_EXEC[g_exn]}} & 16'h000F))) u_exnx (
				.I0(exec_done_st_r[g_exn]), .I1(exec_alt_nx[g_exn]), .I2(exec_spec_c), .I3(exec_go_b[0]),
				.O(exec_nx_c[g_exn])
			);
		end
	endgenerate
	wire exec_no_abort_idle = (nop_r || is_query_caps_r) ? (nop_r || (!qc_align_err && qc_zero_r)) : (exec_done_st_r == Q_IDLE);
	wire exec_is_idle_c = exec_go_b[0] && !seq_abort_b[3] && exec_no_abort_idle;
	wire exec_is_exec_c = !exec_spec_c && !exec_go_b[0];

	wire post_done_c = !settle && !(|(exec_post & busy_in));
	wire bar_done_c  = !settle && !pre_busy_0 && !pre_busy_1 && !pre_busy_2;
	wire [3:0] idle_nx = hdr_take_fsm ? Q_DEC : Q_IDLE;
	wire [3:0] post_nx = post_done_c ? Q_IDLE : Q_POST;
	wire [3:0] freq_nx = f_ready_b[1] ? (is_wait_fence_r ? Q_FWAIT : Q_IDLE) : Q_FREQ;
	wire [3:0] fwait_nx = !f_busy ? Q_IDLE : Q_FWAIT;
	wire [3:0] vsync_nx = vsync ? Q_IDLE : Q_VSYNC;
	wire [3:0] dec2_nx = dec2_wait_r ? (dec_fault_r2 ? Q_DEAD : (h_zero_r ? (is_barrier_r ? Q_BARRIER : Q_EXEC) : Q_PAY)) : Q_DEC2;
	wire [3:0] bar_nx  = bar_done_c ? Q_IDLE : Q_BARRIER;
	wire [3:0] flush_nx = l2_flush_done ? Q_FREQ : Q_FLUSH;
	wire [3:0] qcr0_nx  = (!cfws_b[0] && qc_r_rdy) ? Q_QCAPS_R1 : Q_QCAPS_R0;
	wire [3:0] qcr1_nx  = (!cfws_b[2] && qc_r_rdy) ? Q_QCAPS_REQ : Q_QCAPS_R1;
	wire [3:0] qcreq_nx = f_ready_b[2] ? Q_QCAPS_WAIT : Q_QCAPS_REQ;
	wire [3:0] qcwait_nx = !f_busy ? (qc_last_r ? Q_IDLE : Q_QCAPS_R0) : Q_QCAPS_WAIT;
	wire st_next_is_qcreq = ((st_c == Q_QCAPS_R1) && !cfws_b[2] && qc_r_rdy) || ((st_c == Q_QCAPS_REQ) && !f_ready_b[2]);

	wire [3:0] st_next;
	wire st_idle_next, st_exec_next;
	genvar g_st;
	generate
		for (g_st = 0; g_st < 4; g_st = g_st + 1) begin : g_st_tree
			wire nx_0, nx_1, nx_2, nx_3, other_nx;
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx0 (
				.I0(idle_nx[g_st]), .I1(1'b0), .I2(1'b0), .I3(post_nx[g_st]),
				.I4(st_g0[0]), .I5(st_g0[1]), .O(nx_0)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx1 (
				.I0(freq_nx[g_st]), .I1(fwait_nx[g_st]), .I2(vsync_nx[g_st]), .I3(Q_DEAD[g_st]),
				.I4(st_g1[0]), .I5(st_g1[1]), .O(nx_1)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx2 (
				.I0(Q_DEC2[g_st]), .I1(dec2_nx[g_st]), .I2(bar_nx[g_st]), .I3(flush_nx[g_st]),
				.I4(st_g2[0]), .I5(st_g2[1]), .O(nx_2)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_nx3 (
				.I0(qcr0_nx[g_st]), .I1(qcr1_nx[g_st]), .I2(qcreq_nx[g_st]), .I3(qcwait_nx[g_st]),
				.I4(st_g3[0]), .I5(st_g3[1]), .O(nx_3)
			);
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_oth (
				.I0(nx_0), .I1(nx_1), .I2(nx_2), .I3(nx_3),
				.I4(st_g4[2]), .I5(st_g4[3]), .O(other_nx)
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_stnxt (
				.I0(other_nx), .I1(pay_nx_c[g_st]), .I2(exec_nx_c[g_st]),
				.I3(st_is_pay_r), .I4(st_is_exec_r),
				.O(st_next[g_st])
			);
		end
	endgenerate
	wire id_0, id_1, id_2, id_3, id_oth;
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id0 (
		.I0(1'b0), .I1(1'b0), .I2(1'b0), .I3(post_done_c),
		.I4(st_g0[0]), .I5(st_g0[1]), .O(id_0)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id1 (
		.I0(f_ready_b[1] && !is_wait_fence_r), .I1(!f_busy), .I2(vsync), .I3(1'b0),
		.I4(st_g1[0]), .I5(st_g1[1]), .O(id_1)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id2 (
		.I0(1'b0), .I1(1'b0), .I2(bar_done_c), .I3(1'b0),
		.I4(st_g2[0]), .I5(st_g2[1]), .O(id_2)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_id3 (
		.I0(1'b0), .I1(1'b0), .I2(1'b0), .I3(!f_busy && qc_last_r),
		.I4(st_g3[0]), .I5(st_g3[1]), .O(id_3)
	);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFF00F0F0CCCCAAAA)) u_idoth (
		.I0(id_0), .I1(id_1), .I2(id_2), .I3(id_3),
		.I4(st_g4[2]), .I5(st_g4[3]), .O(id_oth)
	);
	wire idle_stay_c = st_is_idle_r && (halt || seq_abort_b[2] || !hdr_valid);
	(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'hFFFFFFFFF0F0CCAA)) u_idnxt (
		.I0(id_oth), .I1(pay_is_idle_c), .I2(exec_is_idle_c),
		.I3(st_is_pay_r), .I4(st_is_exec_r), .I5(idle_stay_c),
		.O(st_idle_next)
	);
	wire ex_from_dec2 = dec2_wait_r && !dec_fault_r2 && h_zero_r && !is_barrier_r;
	wire ex_from_pay;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'h00000100)) u_ex_pay (
		.I0(bad_idx_comb), .I1(bad_absent_comb), .I2(nop_r), .I3(pay_done_c), .I4(is_barrier_r), .O(ex_from_pay)
	);
	wire ex_2 = (st_g2[1:0] == 2'b01) ? ex_from_dec2 : 1'b0;
	(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_exnxt (
		.I0(st_g4[3] && !st_g4[2] ? ex_2 : 1'b0), .I1(ex_from_pay), .I2(exec_is_exec_c),
		.I3(st_is_pay_r), .I4(st_is_exec_r),
		.O(st_exec_next)
	);
`else
	reg st_is_barrier_r, st_is_post_r, st_is_other_nobar_r;
	reg st_is_fwait_r, st_is_vsync_r, st_is_qcwait_r;
	reg st_is_dead_r, st_is_flush_r, st_is_qcr0_r, st_is_qcr1_r, st_is_qcreq_r;
	wire pay_fault_c = !nop_r && (bad_idx_comb || bad_absent_comb);
	wire [3:0] pay_stall_nx = pay_fault_c ? Q_DEAD : Q_PAY;
	wire [3:0] pay_done_tgt = nop_r ? Q_IDLE : (is_barrier_r ? Q_BARRIER : Q_EXEC);
	wire [3:0] pay_go_nx = pay_fault_c ? Q_DEAD : (p_is_last_r ? pay_done_tgt : Q_PAY);
	wire pay_go_en = pay_pre_go;
	wire pay_idle_pre = pay_pre_go && !pay_fault_c && p_is_last_r && nop_r;
	wire pay_exec_pre = pay_pre_go && !pay_fault_c && p_is_last_r && !nop_r && !is_barrier_r;

	wire exec_spec_c = seq_abort_b[3] || nop_r || is_query_caps_r;
	wire [3:0] exec_alt_nx = seq_abort_b[3] ? Q_DEAD :
		nop_r ? Q_IDLE :
		(qc_align_err ? Q_DEAD : (qc_zero_r ? Q_IDLE : Q_QCAPS_R0));
	wire [3:0] exec_stay_nx = exec_spec_c ? exec_alt_nx : Q_EXEC;
	wire exec_go_en = st_is_exec_r && !exec_spec_c && ego_ok;
	wire exec_idle_alt = !seq_abort_b[3] && (nop_r || (is_query_caps_r && !qc_align_err && qc_zero_r));
	wire exec_idle_pre = st_is_exec_r && !exec_spec_c && ego_ok && exec_done_is_idle_r;

	wire [8:0] po_ok_w = ~(exec_post & busy_in);
	wire po_g0_n = ~&po_ok_w[2:0];
	wire po_g1_n = ~&po_ok_w[5:3];
	wire po_g2_n = ~&po_ok_w[8:6];
	wire post_busy_n = ~(po_g0_n | po_g1_n | po_g2_n);
	wire post_done_c = !settle && post_busy_n;
	wire post_stay_c = st_is_post_r && !(!(settle | po_g0_n) & !(po_g1_n | po_g2_n));
	wire [3:0] idle_nx = hdr_take_fsm ? Q_DEC : Q_IDLE;
	wire [3:0] freq_nx = f_ready_b[1] ? (is_wait_fence_r ? Q_FWAIT : Q_IDLE) : Q_FREQ;
	wire [3:0] fwait_nx = !f_busy ? Q_IDLE : Q_FWAIT;
	wire [3:0] vsync_nx = vsync ? Q_IDLE : Q_VSYNC;
	wire [3:0] dec2_nx = dec2_wait_r ? (dec_fault_r2 ? Q_DEAD : (h_zero_r ? (is_barrier_r ? Q_BARRIER : Q_EXEC) : Q_PAY)) : Q_DEC2;
	wire [3:0] flush_nx = l2_flush_done ? Q_FREQ : Q_FLUSH;
	wire [3:0] qcr0_nx  = (!cfws_b[0] && qc_r_rdy) ? Q_QCAPS_R1 : Q_QCAPS_R0;
	wire [3:0] qcr1_nx  = (!cfws_b[2] && qc_r_rdy) ? Q_QCAPS_REQ : Q_QCAPS_R1;
	wire [3:0] qcreq_nx = f_ready_b[2] ? Q_QCAPS_WAIT : Q_QCAPS_REQ;
	wire [3:0] qcwait_nx = !f_busy ? (qc_last_r ? Q_IDLE : Q_QCAPS_R0) : Q_QCAPS_WAIT;
	wire st_next_is_qcreq = (st_is_qcr1_r && !cfws_b[2] && qc_r_rdy) || (st_is_qcreq_r && !f_ready_b[2]);

	wire [3:0] oth_pa = ({4{st_is_freq_r}}   & freq_nx)   | ({4{st_is_fwait_r}} & fwait_nx);
	wire [3:0] oth_pb = ({4{st_is_vsync_r}}  & vsync_nx)  | ({4{st_is_dead_r}}  & Q_DEAD);
	wire [3:0] oth_pc = ({4{st_is_dec_r}}    & Q_DEC2)    | ({4{st_is_dec2_r}}  & dec2_nx);
	wire [3:0] oth_pd = ({4{st_is_flush_r}}  & flush_nx)  | ({4{st_is_qcr0_r}}  & qcr0_nx);
	wire [3:0] oth_pe = ({4{st_is_qcr1_r}}   & qcr1_nx)   | ({4{st_is_qcreq_r}} & qcreq_nx);
	wire [3:0] oth_pf = ({4{st_is_qcwait_r}} & qcwait_nx);
	wire [3:0] oth_n0 = ~(oth_pa | oth_pb);
	wire [3:0] oth_n1 = ~(oth_pc | oth_pd);
	wire [3:0] oth_n2 = ~(oth_pe | oth_pf);

	wire [3:0] idle_term  = ({4{st_is_idle_r}} & {hdr_take_fsm, 3'b000});
	wire [3:0] post_term  = {2'b00, post_stay_c, post_stay_c};
	wire [3:0] other_term = ~(oth_n0 & oth_n1 & oth_n2);
	wire [3:0] pay_term_0 = ({4{st_is_pay_r}} & pay_stall_nx);
	wire [3:0] pay_term_1 = ({4{st_is_pay_r}} & ((in_valid && !seq_abort) ? pay_go_nx : pay_stall_nx));
	wire [3:0] pay_term   = pay_hold_n ? pay_term_1 : pay_term_0;
	wire [3:0] eb_term_pb0 = ({4{st_is_barrier_r}} & Q_BARRIER) | ({4{st_is_exec_r}} & exec_stay_nx);
	wire [3:0] eb_term_pb1 = ({4{st_is_barrier_r && settle}} & Q_BARRIER) |
		({4{st_is_exec_r}} & (exec_spec_c ? exec_alt_nx : (ego_ok ? exec_done_st_r : Q_EXEC)));
	wire [3:0] eb_term = pb_all_ok ? eb_term_pb1 : eb_term_pb0;
	wire [3:0] st_nx_g0_n = ~((idle_term | post_term) | other_term);
	wire [3:0] st_nx_g1_n = ~(pay_term  | eb_term);
	wire [3:0] st_next = ~(st_nx_g0_n & st_nx_g1_n);

	wire id_oth = (st_is_freq_r && f_ready_b[1] && !is_wait_fence_r) ||
		(st_is_fwait_r && !f_busy) ||
		(st_is_vsync_r && vsync) ||
		(st_is_qcwait_r && !f_busy && qc_last_r);
	wire id_t0_n = !((st_is_idle_r && (!hdr_take_fast || halt)) || (st_is_exec_r && exec_spec_c && exec_idle_alt));
	wire id_t1_n = !(id_oth || (st_is_post_r && !settle && post_busy_n));
	wire id_t2_n = !((pay_idle_pre && pay_hold_n) || ((exec_idle_pre || (st_is_barrier_r && !settle)) && pb_all_ok));
	wire st_idle_next = !(id_t0_n & id_t1_n & id_t2_n);

	wire ex_from_dec2 = st_is_dec2_r &&
		dec2_wait_r && !dec_fault_r2 && h_zero_r && !is_barrier_r;
	wire exec_stay_no_pb = st_is_exec_r && !exec_spec_c;
	wire st_exec_next = ex_from_dec2 || (pay_exec_pre && pay_hold_n) || (exec_stay_no_pb && (!ego_ok || !pb_all_ok));

	wire exec_freq_pre  = st_is_exec_r && !seq_abort && ego_ok && exec_done_is_freq_r;
	wire exec_flush_pre = st_is_exec_r && !seq_abort && ego_ok && exec_done_is_flush_r;
	wire f_valid_nx = st_next_is_qcreq || (st_is_freq_r && !f_ready_b[1]) || (st_is_flush_r && l2_flush_done) || (exec_freq_pre && pb_all_ok);
	wire f_flush_nx = (st_is_flush_r && !l2_flush_done) || (exec_flush_pre && pb_all_ok);
	wire h_n1_hdr_n = !(!(hdr_r[7] | hdr_r[6]) && !(hdr_r[5] | hdr_r[4] | hdr_r[3]) && !(!hdr_r[0] | hdr_r[1] | hdr_r[2]));
	wire h_n1_ghci_n = !(!(ghci_n_r[7] | ghci_n_r[6]) && !(ghci_n_r[5] | ghci_n_r[4] | ghci_n_r[3]) && !(!ghci_n_r[0] | ghci_n_r[1] | ghci_n_r[2]));
	wire h_n1_w = !(is_ghci_hdr_r ? h_n1_ghci_n : h_n1_hdr_n);
	wire p_is_last_nx = st_is_dec_r ? h_n1_w : ((pay_hold_n && pay_pre_go) ? (p == n_m2_r) : p_is_last_r);
`endif

`ifdef SYNTH_KINTEX7
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
`elsif SYNTH_KINTEX_ULTRASCALE
	always @(posedge clk or posedge seq_rst_high) begin
		if (seq_rst_high) begin
`else
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
`endif
			f_valid_r <= 1'b0;
			f_flush_r <= 1'b0;
			st <= Q_IDLE;
			st_c <= Q_IDLE;
			st_c_qc <= Q_IDLE;
			st_c_qc_d <= Q_IDLE;
			is_qc_req_fa0_b0_r <= 1'b0;
			is_qc_req_fa0_b0a_r <= 1'b0;
			is_qc_req_fa0_b1_r <= 1'b0;
			is_qc_req_fa0_b2_r <= 1'b0;
			is_qc_req_fa0_b3_r <= 1'b0;
			is_qc_req_fa1_lo_r <= 1'b0;
			is_qc_req_fa1_hi_r <= 1'b0;
			is_qc_req_fv0_r <= 1'b0;
			is_qc_req_fv1_r <= 1'b0;
			is_qc_req_fx0_r <= 1'b0;
			is_qc_req_fx1_r <= 1'b0;
			is_qc_req_fop_r <= 1'b0;
			is_qc_req_f64_r <= 1'b0;
			st_out_r <= Q_IDLE;
			st_g0 <= Q_IDLE;
			st_g1 <= Q_IDLE;
			st_g2 <= Q_IDLE;
			st_g3 <= Q_IDLE;
			st_g4 <= Q_IDLE;
			st_is_idle_r <= 1'b1;
			st_is_dec_r <= 1'b0;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			st_is_dec2_r <= 1'b0;
			st_is_barrier_r <= 1'b0;
			st_is_post_r <= 1'b0;
			st_is_other_nobar_r <= 1'b0;
			st_is_freq_r <= 1'b0;
			st_is_fwait_r <= 1'b0;
			st_is_vsync_r <= 1'b0;
			st_is_qcwait_r <= 1'b0;
			st_is_dead_r <= 1'b0;
			st_is_flush_r <= 1'b0;
			st_is_qcr0_r <= 1'b0;
			st_is_qcr1_r <= 1'b0;
			st_is_qcreq_r <= 1'b0;
`endif
			st_is_exec_r <= 1'b0;
			dec_fault_r2 <= 1'b0;
			dec_short_r2 <= 1'b0;
			use_fw <= 1'b0;
			hdr_r <= 32'd0;
			is_ghci_hdr_r <= 1'b0;
			ghci_n_r <= 8'd0;
			is_set_r <= 1'b0;
			is_dset_r <= 1'b0;
			is_set_or_dset_r <= 1'b0;
			st_is_pay_r <= 1'b0;
			n_m1_r <= 8'd0;
			n_m2_r <= 8'd0;
			qc_last_r <= 1'b0;
			p_is_0_r <= 1'b0;
			p_is_1_r <= 1'b0;
			p_is_2_r <= 1'b0;
			p_is_3_r <= 1'b0;
			p_is_4_r <= 1'b0;
			p_is_last_r <= 1'b0;
			p_lt_3_r <= 1'b0;
			p_lt_4_r <= 1'b0;
			p_lt_9_r <= 1'b0;
			is_dbg_mark_r <= 1'b0;
			is_flip_r <= 1'b0;
			is_set_r1 <= 1'b0;
			is_dset_r1 <= 1'b0;
			is_sopen_r1 <= 1'b0;
			is_bsess_r <= 1'b0;
			bound_session <= 4'd0;
			is_fop_fence_r <= 1'b0;
			is_fop_res_q_r <= 1'b0;
			is_wait_fence_r <= 1'b0;
			h_early_fault_r <= 1'b0;
			ghci_fault_r <= 1'b0;
			std_fault_r <= 1'b0;
			disp_fw_fault_r <= 1'b0;
			h_early_absent_r <= 1'b0;
			h_early_short_r <= 1'b0;
			h_nhi_z_r <= 1'b0;
			h_min_r <= 4'd0;
			ghci_min_r <= 4'd0;
			std_min_r <= 4'd0;
			h_nlo_r <= 4'd0;
			is_fence_cmd_r <= 1'b0;
			is_fc_or_res_r <= 1'b0;
			is_query_caps_r <= 1'b0;
			qc_addr_lo <= 32'd0;
			qc_addr_hi <= 32'd0;
			qc_bytes <= 32'd0;
			qc_total_k_r <= 7'd0;
			qc_zero_r <= 1'b1;
			qc_cur_addr <= 32'd0;
			qc_cur_addr_hi <= 32'd0;
			qc_k <= 7'd0;
			qc_k_cap <= 7'd0;
			qc_w0 <= 32'd0;
			qc_w1 <= 32'd0;
			qc_word_sel_r <= 1'b0;
			qc_r_rdy <= 1'b0;
			qc_align_err <= 1'b0;
			vendor <= 1'b0;
			op <= 8'd0;
			flags <= 8'd0;
			tag <= 8'd0;
			side <= 7'd0;
			n <= 8'd0;
			p <= 8'd0;
			settle <= 1'b0;
			f_pulse <= 1'b0;
			f_kind <= 4'd0;
			dec_fault_r <= 1'b0;
			dec_short_r <= 1'b0;
			dec_absent_r <= 1'b0;
			h_zero_r <= 1'b0;
			is_query_r <= 1'b0;
			is_dispatch_r <= 1'b0;
			is_mesh_r <= 1'b0;
			is_load_const_r <= 1'b0;
			is_draw_raw_r <= 1'b0;
			is_barrier_r <= 1'b0;
			nop_r <= 1'b0;
			is_allowed_op_r <= 1'b0;
			unit_op_r <= 1'b0;
			u_req_su_r <= 1'b0;
			fence_op_r <= 1'b0;
			is_wait_vsync_r <= 1'b0;
			pre_r <= 9'd0;
			pre_f_r <= 1'b0;
			post_r <= 9'd0;
			post_nz_r <= 1'b0;
			post_dec_r <= 9'd0;
			post_nz_dec_r <= 1'b0;
			k_on_dec_r <= 1'b0;
			k_code_dec_r <= 3'd0;
			inv_code_dec_r <= 4'd0;
			st_exec_rdy_r <= 1'b0;
			exec_done_st_r <= Q_IDLE;
			exec_done_is_idle_r <= 1'b1;
			exec_done_is_freq_r <= 1'b0;
			exec_done_is_flush_r <= 1'b0;
			f_val_pre_r <= 32'd0;
			exec_settle_r <= 1'b0;
			k_valid_r <= 1'b0;
			k_code_r <= 3'd0;
			inv_code_r <= 4'd0;
			unit_op_dec_r <= 1'b0;
			unit_op_su_dec_r <= 1'b0;
			k_raw_dec_r <= 1'b0;
			k_blt_dec_r <= 1'b0;
			k_draw_dec_r <= 1'b0;
			k_cs_dec_r <= 1'b0;
			k_lc_dec_r <= 1'b0;
			k_perf_dec_r <= 1'b0;
			k_flip_dec_r <= 1'b0;
			k_disp_dec_r <= 1'b0;
			inv_draw_dec_r <= 1'b0;
			inv_prog_dec_r <= 1'b0;
			inv_inv_dec_r <= 1'b0;
			is_cmd_fence_r <= 1'b0;
			occ_q <= 40'd0;
		end else begin
			f_pulse <= 1'b0;
			settle <= 1'b0;
`ifdef SYNTH_KINTEX7
			if (hdr_take_b[0]) hdr_r[7:0]   <= hdr[7:0];
			if (hdr_take_b[5]) hdr_r[15:8]  <= hdr[15:8];
			if (hdr_take_b[1]) hdr_r[23:16] <= hdr[23:16];
			if (hdr_take_b[6]) hdr_r[31:24] <= hdr[31:24];
			if (hdr_take_b[2]) begin
				use_fw <= fw_act_b[4];
				side <= fw_act_b[4] ? fw_side : 7'd0;
			end
			if (hdr_take_b[4]) begin
				is_ghci_hdr_r <= is_ghci_hdr_comb;
				ghci_n_r <= ghci_n_comb;
			end
`elsif SYNTH_KINTEX_ULTRASCALE
			if (hdr_take_b[0]) hdr_r[7:0]   <= hdr[7:0];
			if (hdr_take_b[5]) hdr_r[15:8]  <= hdr[15:8];
			if (hdr_take_b[1]) hdr_r[23:16] <= hdr[23:16];
			if (hdr_take_b[6]) hdr_r[31:24] <= hdr[31:24];
			if (hdr_take_b[2]) begin
				use_fw <= fw_act_b[4];
				side <= fw_act_b[4] ? fw_side : 7'd0;
			end
			if (hdr_take_b[4]) begin
				is_ghci_hdr_r <= is_ghci_hdr_comb;
				ghci_n_r <= ghci_n_comb;
			end
`else
			if (st_is_idle_r) begin
				hdr_r[15:0] <= hdr[15:0];
				hdr_r[31:16] <= hdr[31:16];
				use_fw <= fw_act_b[4];
				side <= fw_act_b[4] ? fw_side : 7'd0;
				is_ghci_hdr_r <= is_ghci_hdr_comb;
				ghci_n_r <= ghci_n_comb;
			end
`endif
			if (st_dec_b[0]) begin
				vendor <= h_vendor;
				op <= h_op_a;
				qc_align_err <= 1'b0;
			end
			if (p_is0_ce_b[0]) qc_addr_lo[15:0] <= in_data[15:0];
			if (p_is0_ce_b[1]) qc_addr_lo[31:16] <= in_data[31:16];
			if (p_is0_ce_b[2]) begin
`ifdef SYNTH_KINTEX7
				if (in_data[5:0] != 6'd0) qc_align_err <= 1'b1;
`elsif SYNTH_KINTEX_ULTRASCALE
				if (in_data[5:0] != 6'd0) qc_align_err <= 1'b1;
`else
				if (pay_align_bad_c) qc_align_err <= 1'b1;
`endif
				if (is_bsess_r) bound_session <= in_data[3:0];
			end
`ifdef SYNTH_KINTEX7
			if (pay_take_b[2] && p_is_1_r) qc_addr_hi[15:0]  <= in_data[15:0];
			if (pay_take_b[3] && p_is_1_r) qc_addr_hi[31:16] <= in_data[31:16];
			if (pay_take_b[2] && p_is_2_r) begin
				qc_bytes[15:0] <= in_data[15:0];
				qc_zero_r <= qc_zero_c;
				qc_total_k_r <= qc_total_k_c;
				if (in_data[5:0] != 6'd0) qc_align_err <= 1'b1;
			end
			if (pay_take_b[3] && p_is_2_r) qc_bytes[31:16] <= in_data[31:16];
`elsif SYNTH_KINTEX_ULTRASCALE
			if (pay_take_b[2] && p_is_1_r) qc_addr_hi[15:0]  <= in_data[15:0];
			if (pay_take_b[3] && p_is_1_r) qc_addr_hi[31:16] <= in_data[31:16];
			if (pay_take_b[2] && p_is_2_r) begin
				qc_bytes[15:0] <= in_data[15:0];
				qc_zero_r <= qc_zero_c;
				qc_total_k_r <= qc_total_k_c;
				if (in_data[5:0] != 6'd0) qc_align_err <= 1'b1;
			end
			if (pay_take_b[3] && p_is_2_r) qc_bytes[31:16] <= in_data[31:16];
`else
			if (pay_hold_n && (pay_pre_go && p_is_1_r)) begin
				qc_addr_hi <= in_data;
			end
			if (pay_hold_n && (pay_pre_go && p_is_2_r)) begin
				qc_bytes <= in_data;
				qc_zero_r <= qc_zero_c;
				qc_total_k_r <= qc_total_k_c;
				if (pay_align_bad_c) qc_align_err <= 1'b1;
			end
`endif
			if (st_dec_b[1]) begin
				flags <= (is_ghci_hdr_r ? (hdr_r[7:0] == CMD_FENCE) : (!hdr_r[30] && (hdr_r[29:24] == 6'h33))) ?
					8'd0 : (is_ghci_hdr_r ? hdr_r[31:24] : hdr_r[15:8]);
				tag <= hdr_r[23:16];
			end
			if (st_dec_b[2]) begin
				n <= h_n;
				n_m1_r <= h_n_m1;
				n_m2_r <= h_n_m2;
				h_zero_r <= h_zero;
			end
			if (st_dec_b[3]) begin
				is_set_r <= !h_vendor && (h_op_b == OP_SET_REG);
				is_dset_r <= !h_vendor && (h_op_b == OP_DISP_SET);
				is_set_or_dset_r <= !h_vendor && (h_op_b == OP_SET_REG || h_op_b == OP_DISP_SET);
				is_set_r1 <= !h_vendor && (h_op_b == OP_SET_REG);
				is_dset_r1 <= !h_vendor && (h_op_b == OP_DISP_SET);
				is_sopen_r1 <= !h_vendor && (h_op_b == CMD_SESSION_OPEN);
				is_bsess_r <= !h_vendor && (h_op_b == CMD_BIND_SESSION);
				is_fence_cmd_r <= (h_op_b == CMD_FENCE || h_op_b == CMD_WAIT_FENCE);
				is_fop_fence_r <= is_ghci_hdr_r ? (hdr_r[7:0] == 8'd5 || hdr_r[7:0] == CMD_FENCE) :
					(!hdr_r[30] && (hdr_r[29:24] == 6'd5 || hdr_r[29:24] == 6'h33));
				is_fop_res_q_r <= !h_vendor && (h_op_b == OP_RESULT || h_op_b == OP_QUERY);
				is_wait_fence_r <= !h_vendor && (h_op_b == OP_WAIT_FENCE || h_op_b == CMD_WAIT_FENCE);
			end
			if (st_dec_b[8]) begin
				is_query_caps_r <= !h_vendor && (h_op_b2 == CMD_QUERY_CAPS);
				is_query_r <= !h_vendor && (h_op_b2 == OP_QUERY);
				is_dispatch_r <= !h_vendor && (h_op_b2 == OP_DISPATCH || h_op_b2 == CMD_DISPATCH || h_op_b2 == CMD_DRAW_MESH_TASKS);
				is_mesh_r <= !h_vendor && (h_op_b2 == CMD_DRAW_MESH_TASKS);
				is_load_const_r <= !h_vendor && (h_op_b2 == OP_LOAD_CONST);
				is_draw_raw_r <= h_vendor && (h_op_b2 == VOP_DRAW_RAW);
				is_dbg_mark_r <= h_vendor && (h_op_b2 == VOP_DBG_MARK);
				is_flip_r <= !h_vendor && !is_ghci_hdr_r && (h_op_b2 == OP_FLIP);
				is_barrier_r <= !h_vendor && (h_op_b2 == CMD_BARRIER || h_op_b2 == OP_BARRIER);
				begin : g_dec_uop_inv
					reg [5:0] s_op6;
					reg s_cmd_draw3, s_cmd_draw4, s_cmd_disp, s_std_su3;
					reg s_gh_draw3, s_gh_draw4, s_gh_disp;
					reg g_h0, g_h2, g_h3;
					reg g_uop, g_su, g_invd, s_uop, s_su, s_invd;
					reg s_cs_gh, s_cs_st, s_kd_gh, s_kd_st;
					s_op6 = hdr_r[29:24];
					s_cmd_draw3 = (s_op6[5:4] == 2'b10) && ((s_op6[3:0] == 4'h7) || (s_op6[3:1] == 3'b100));
					s_cmd_draw4 = s_cmd_draw3 || (s_op6 == 6'h2a);
					s_cmd_disp  = (s_op6 == 6'h30);
					s_std_su3   = (s_op6[5:3] == 3'd0) && ((s_op6[2:0] == 3'd3) || (s_op6[2:0] == 3'd4) || (s_op6[2:0] == 3'd7));
					s_su   = hdr_r[30] || s_std_su3 || s_cmd_draw4 || s_cmd_disp;
					s_uop  = s_su || (!hdr_r[30] && (s_op6 == 6'd2));
					s_invd = !hdr_r[30] && ((s_op6 == 6'd3) || s_cmd_draw4);

					g_h0 = !(hdr_r[7] | hdr_r[6] | hdr_r[5] | hdr_r[4]);
					g_h2 = !(hdr_r[7] | hdr_r[6]) &&  hdr_r[5] && !hdr_r[4];
					g_h3 = !(hdr_r[7] | hdr_r[6]) &&  hdr_r[5] &&  hdr_r[4];
					s_gh_draw3 = g_h2 && (!hdr_r[3] ? (hdr_r[2] && hdr_r[1] && hdr_r[0]) : (!hdr_r[2] && !hdr_r[1]));
					s_gh_draw4 = g_h2 && (!hdr_r[3] ? (hdr_r[2] && hdr_r[1] && hdr_r[0]) : (!hdr_r[2] && (!hdr_r[1] || !hdr_r[0])));
					s_gh_disp  = g_h3 && !(hdr_r[3] | hdr_r[2] | hdr_r[1] | hdr_r[0]);
					g_uop  = s_gh_draw4 || s_gh_disp || (g_h0 && !hdr_r[3] && ((!hdr_r[2] && hdr_r[1]) || (hdr_r[2] && (hdr_r[1] == hdr_r[0]))));
					g_su   = s_gh_draw4 || s_gh_disp || (g_h0 && !hdr_r[3] && (hdr_r[2] ? (hdr_r[1] == hdr_r[0]) : (hdr_r[1] && hdr_r[0])));
					g_invd = s_gh_draw4 || (g_h0 && (hdr_r[3:0] == 4'h3));

					s_kd_gh = s_gh_draw3 || (g_h0 && (hdr_r[3:0] == 4'h3));
					s_kd_st = !hdr_r[30] && ((s_op6 == 6'd3) || s_cmd_draw3);
					s_cs_gh = s_gh_disp || (g_h0 && (hdr_r[3:0] == 4'h4)) || (g_h2 && (hdr_r[3:0] == 4'ha));
					s_cs_st = !hdr_r[30] && ((s_op6 == 6'd4) || s_cmd_disp || (s_op6 == 6'h2a));

					unit_op_dec_r    <= is_ghci_hdr_r ? g_uop   : s_uop;
					unit_op_su_dec_r <= is_ghci_hdr_r ? g_su    : s_su;
					inv_draw_dec_r   <= is_ghci_hdr_r ? g_invd  : s_invd;
					k_draw_dec_r     <= is_ghci_hdr_r ? s_kd_gh : s_kd_st;
					k_cs_dec_r       <= is_ghci_hdr_r ? s_cs_gh : s_cs_st;
				end
				k_raw_dec_r  <=  h_vendor && (h_op_b2 == VOP_DRAW_RAW);
				k_blt_dec_r  <= !h_vendor && (h_op_b2 == OP_BLIT);
				k_lc_dec_r   <= !h_vendor && (h_op_b2 == OP_LOAD_CONST);
				k_perf_dec_r <= !h_vendor && (h_op_b2 == OP_PERF_RESET);
				k_flip_dec_r <= !h_vendor && (h_op_b2 == OP_FLIP);
				k_disp_dec_r <= !h_vendor && (h_op_b2 == OP_DISP_SET);
				inv_prog_dec_r <= !h_vendor && (h_op_b2 == OP_LOAD_PROGRAM);
				inv_inv_dec_r  <= !h_vendor && (h_op_b2 == OP_INVALIDATE);
				is_cmd_fence_r <= !h_vendor && (h_op_b2 == CMD_FENCE);
			end
			if (st_dec_b[4]) begin
`ifdef SYNTH_KINTEX7
				h_early_fault_r <= (disp_op_hdr_c && use_fw_b[4]) || (is_ghci_hdr_r ? ghci_fault_op : std_fault_op);
				h_min_r <= is_ghci_hdr_r ? ghci_min_n : std_min_n;
`elsif SYNTH_KINTEX_ULTRASCALE
				h_early_fault_r <= (disp_op_hdr_c && use_fw_b[4]) || (is_ghci_hdr_r ? ghci_fault_op : std_fault_op);
				h_min_r <= is_ghci_hdr_r ? ghci_min_n : std_min_n;
`else
				h_early_fault_r <= 1'b0;
				ghci_fault_r <= ghci_fault_op;
				std_fault_r <= std_fault_op;
				disp_fw_fault_r <= disp_op_hdr_c && use_fw_b[4];
				h_min_r <= 4'd0;
`endif
				h_early_absent_r <= is_ghci_hdr_r ? ghci_absent_op : std_absent_op;
				h_early_short_r <= !is_ghci_hdr_r && !hdr_r[30] && (hdr_r[29:24] == 6'd1 || hdr_r[29:24] == 6'd15) && hdr_r[0];
				ghci_min_r <= ghci_min_n;
				std_min_r <= std_min_n;
				h_nhi_z_r <= is_ghci_hdr_r ? (hdr_r[15:12] == 4'd0) : (hdr_r[7:4] == 4'd0);
				h_nlo_r <= is_ghci_hdr_r ? hdr_r[11:8] : hdr_r[3:0];
			end
			if (st_is_dec_r) begin
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
				p <= 8'd0;
`endif
				p_is_0_r <= 1'b1;
				p_is_1_r <= 1'b0;
				p_is_2_r <= 1'b0;
				p_is_3_r <= 1'b0;
				p_is_4_r <= 1'b0;
`ifdef SYNTH_KINTEX7
				p_is_last_r <= (h_n == 8'd1);
`elsif SYNTH_KINTEX_ULTRASCALE
				p_is_last_r <= (h_n == 8'd1);
`endif
				p_lt_3_r <= 1'b1;
				p_lt_4_r <= 1'b1;
				p_lt_9_r <= 1'b1;
`ifdef SYNTH_KINTEX7
			end else if (pay_take_b[0]) begin
`elsif SYNTH_KINTEX_ULTRASCALE
			end else if (pay_take_b[0]) begin
`else
			end else if (pay_hold_n && pay_pre_go) begin
				p <= p_n;
`endif
				p_is_0_r <= 1'b0;
				p_is_1_r <= p_is_0_r;
				p_is_2_r <= p_is_1_r;
				p_is_3_r <= p_is_2_r;
				p_is_4_r <= p_is_3_r;
`ifdef SYNTH_KINTEX7
				p_is_last_r <= (p == n_m2_r);
`elsif SYNTH_KINTEX_ULTRASCALE
				p_is_last_r <= (p == n_m2_r);
`endif
				p_lt_3_r <= p_is_0_r || p_is_1_r;
				p_lt_4_r <= p_is_0_r || p_is_1_r || p_is_2_r;
				p_lt_9_r <= (p[7:3] == 5'd0);
			end
`ifdef SYNTH_KINTEX7
			if (st_is_dec_r || pay_take_b[0])
				p <= st_is_dec_r ? 8'd0 : p_n;
`elsif SYNTH_KINTEX_ULTRASCALE
			if (st_is_dec_r || pay_take_b[0])
				p <= st_is_dec_r ? 8'd0 : p_n;
`endif
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			p_is_last_r <= p_is_last_nx;
`endif
`ifdef SYNTH_KINTEX7
			if (pay_take_b[0] && is_fence_cmd_r && p_is_4_r) begin
`elsif SYNTH_KINTEX_ULTRASCALE
			if (pay_take_b[0] && is_fence_cmd_r && p_is_4_r) begin
`else
			if (pay_hold_n && (pay_pre_go && is_fence_cmd_r && p_is_4_r)) begin
`endif
				if (in_data[2]) begin
					inv_code_r <= 4'b1111;
					exec_done_st_r <= Q_FLUSH;
					exec_done_is_idle_r <= 1'b0;
					exec_done_is_freq_r <= 1'b0;
					exec_done_is_flush_r <= 1'b1;
					exec_settle_r <= 1'b1;
				end else begin
					inv_code_r <= 4'd0;
					exec_done_st_r <= Q_FREQ;
					exec_done_is_idle_r <= 1'b0;
					exec_done_is_freq_r <= 1'b1;
					exec_done_is_flush_r <= 1'b0;
					exec_settle_r <= 1'b0;
				end
			end
			if (st_dec2_b[0]) begin
				is_fc_or_res_r <= !vendor && (op == CMD_FENCE || op == CMD_WAIT_FENCE || op == OP_RESULT || op == CMD_QUERY_CAPS);
				is_allowed_op_r <= !is_ghci_hdr_r ? (!hdr_r[30] && (hdr_r[29:24] == OP_FENCE[5:0] || hdr_r[29:24] == OP_WAIT_FENCE[5:0])) :
					((hdr_r[7:0] == CMD_FENCE) || (hdr_r[7:0] == CMD_WAIT_FENCE) || (hdr_r[7:0] == CMD_SESSION_CLOSE));
				unit_op_r <= unit_op_dec_r;
				u_req_su_r <= unit_op_su_dec_r;
				fence_op_r <= fence_op;
				is_wait_vsync_r <= !vendor && (op == OP_WAIT_VSYNC);
				k_on_dec_r <= k_on_dec;
				k_code_dec_r <= k_code_dec;
				inv_code_dec_r <= inv_code_dec;
			end
			if (st_dec2_b[1]) begin
				pre_r <= pre;
				pre_f_r <= pre_f;
			end
			if (st_dec2_b[2]) begin
				post_dec_r <= post;
				post_nz_dec_r <= !vendor && !is_ghci_hdr_r &&
					((op == OP_LOAD_CONST) || (flags[0] && ((op == OP_BLIT) || (op == OP_DISP_SET))));
			end
			if ((st_c == Q_DEC2) && dec2_wait_r) begin
				post_r <= post_dec_r;
				post_nz_r <= post_nz_dec_r;
				exec_done_st_r <= fence_op_r ? Q_FREQ :
					is_wait_vsync_r ? Q_VSYNC :
					post_nz_dec_r ? Q_POST : Q_IDLE;
				exec_done_is_idle_r <= !nop_r && !is_query_caps_r && !fence_op_r && !is_wait_vsync_r && !post_nz_dec_r;
				exec_done_is_freq_r <= !nop_r && !is_query_caps_r && fence_op_r;
				exec_done_is_flush_r <= 1'b0;
				exec_settle_r <= !fence_op_r && !is_wait_vsync_r && post_nz_dec_r;
				k_valid_r <= k_on_dec_r;
				k_code_r <= k_code_dec_r;
				inv_code_r <= inv_code_dec_r;
			end
			f_val_pre_r <= ((st_c == Q_QCAPS_R1) || (st_c == Q_QCAPS_REQ)) ? qc_w0 : a1;
			dec_fault_r2 <= dec_fault_s;
			dec_short_r2 <= dec_short_s;
`ifdef SYNTH_KINTEX7
			st_exec_rdy_r <= st_exec_next;
			st_is_dec_r <= 1'b0;
			f_valid_r <= st_next_is_qcreq || ((st_c == Q_FREQ) && !f_ready_b[1]) || ((st_c == Q_FLUSH) && l2_flush_done) ||
				(st_is_exec_r && !seq_abort_b[0] && ego_ok && exec_done_is_freq_r && !pre_busy_0 && !pre_busy_1 && !pre_busy_2);
			f_flush_r <= ((st_c == Q_FLUSH) && !l2_flush_done) ||
				(st_is_exec_r && !seq_abort_b[1] && ego_ok && exec_done_is_flush_r && !pre_busy_0 && !pre_busy_1 && !pre_busy_2);
`elsif SYNTH_KINTEX_ULTRASCALE
			st_exec_rdy_r <= st_exec_next;
			st_is_dec_r <= 1'b0;
			f_valid_r <= st_next_is_qcreq || ((st_c == Q_FREQ) && !f_ready_b[1]) || ((st_c == Q_FLUSH) && l2_flush_done) ||
				(st_is_exec_r && !seq_abort_b[0] && ego_ok && exec_done_is_freq_r && !pre_busy_0 && !pre_busy_1 && !pre_busy_2);
			f_flush_r <= ((st_c == Q_FLUSH) && !l2_flush_done) ||
				(st_is_exec_r && !seq_abort_b[1] && ego_ok && exec_done_is_flush_r && !pre_busy_0 && !pre_busy_1 && !pre_busy_2);
`else
			st_exec_rdy_r <= st_exec_next;
			st_is_dec_r <= hdr_take_fsm;
			st_is_dec2_r <= (st_next == Q_DEC2);
			st_is_freq_r <= (st_next == Q_FREQ);
			st_is_fwait_r <= (st_next == Q_FWAIT);
			st_is_vsync_r <= (st_next == Q_VSYNC);
			st_is_qcwait_r <= (st_next == Q_QCAPS_WAIT);
			st_is_dead_r <= (st_next == Q_DEAD);
			st_is_flush_r <= (st_next == Q_FLUSH);
			st_is_qcr0_r <= (st_next == Q_QCAPS_R0);
			st_is_qcr1_r <= (st_next == Q_QCAPS_R1);
			st_is_qcreq_r <= (st_next == Q_QCAPS_REQ);
			f_valid_r <= f_valid_nx;
			f_flush_r <= f_flush_nx;
			begin : g_pay_settle
				reg pay_bad_w;
				reg dec2_pay_w;
				reg pay_leave_pre;
				reg pay_stay_w;
				reg settle_p_pre;
				reg settle_e_pre;
				reg pay_nxt_w;
				reg bar_nxt_w;
				reg post_nxt_w;
				pay_bad_w  = !nop_r && (bad_absent_comb || bad_idx_comb);
				dec2_pay_w = st_is_dec2_r && dec2_wait_r && !dec_fault_r2 && !h_zero_r;
				pay_leave_pre = in_valid && !seq_abort && p_is_last_r;
				pay_stay_w = st_is_pay_r && !pay_bad_w && !(pay_leave_pre && pay_hold_n);
				pay_nxt_w = dec2_pay_w || pay_stay_w;
				bar_nxt_w = (st_is_dec2_r && dec2_wait_r && !dec_fault_r2 && h_zero_r && is_barrier_r) ||
					(st_is_pay_r && !pay_bad_w && is_barrier_r && pay_leave_pre && pay_hold_n) ||
					(st_is_barrier_r && (settle || !pb_all_ok));
				post_nxt_w = (st_is_exec_r && !exec_spec_c && ego_ok && pb_all_ok && (exec_done_st_r == Q_POST)) || post_stay_c;
				st_is_pay_r <= pay_nxt_w;
				st_is_barrier_r <= bar_nxt_w;
				st_is_post_r <= post_nxt_w;
				st_is_other_nobar_r <= 1'b0;
				settle_p_pre = st_is_pay_r && !nop_r && !(bad_absent_comb || bad_idx_comb) && in_valid && !seq_abort && p_is_last_r;
				settle_e_pre = st_is_exec_r && !seq_abort && !nop_r && !is_query_caps_r && exec_settle_r && ego_ok;
				settle <= (settle_p_pre && pay_hold_n) || (settle_e_pre && pb_all_ok);
			end
`endif
			st <= st_next;
			st_c <= st_next;
			st_c_qc <= st_next;
			st_c_qc_d <= st_next;
			is_qc_req_fa0_b0_r <= st_next_is_qcreq;
			is_qc_req_fa0_b0a_r <= st_next_is_qcreq;
			is_qc_req_fa0_b1_r <= st_next_is_qcreq;
			is_qc_req_fa0_b2_r <= st_next_is_qcreq;
			is_qc_req_fa0_b3_r <= st_next_is_qcreq;
			is_qc_req_fa1_lo_r <= st_next_is_qcreq;
			is_qc_req_fa1_hi_r <= st_next_is_qcreq;
			is_qc_req_fv0_r <= st_next_is_qcreq;
			is_qc_req_fv1_r <= st_next_is_qcreq;
			is_qc_req_fx0_r <= st_next_is_qcreq;
			is_qc_req_fx1_r <= st_next_is_qcreq;
			is_qc_req_fop_r <= st_next_is_qcreq;
			is_qc_req_f64_r <= st_next_is_qcreq;
			st_out_r <= st_next;
			st_g0 <= st_next;
			st_g1 <= st_next;
			st_g2 <= st_next;
			st_g3 <= st_next;
			st_g4 <= st_next;
			st_is_idle_r <= st_idle_next;
			st_is_exec_r <= st_exec_next;
			case (st)
			Q_IDLE: begin
				nop_r <= 1'b0;
`ifdef SYNTH_KINTEX7
				if (hdr_take_b[3]) begin
					st_is_dec_r <= 1'b1;
					st_is_pay_r <= 1'b0;
				end
`elsif SYNTH_KINTEX_ULTRASCALE
				if (hdr_take_b[3]) begin
					st_is_dec_r <= 1'b1;
					st_is_pay_r <= 1'b0;
				end
`endif
			end
			Q_DEC: begin
`ifdef SYNTH_KINTEX7
				st_is_pay_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
				st_is_pay_r <= 1'b0;
`endif
			end
			Q_DEC2: begin
				if (dec2_wait_r) begin
`ifdef SYNTH_KINTEX7
					if (dec_fault_s) begin
						f_pulse <= 1'b1;
						f_kind <= dec_short_s ? GHCI_FAULT_KIND_BAD_REG :
							h_early_absent_r ? GHCI_FAULT_KIND_KICK_ABSENT :
							GHCI_FAULT_KIND_INVALID_OP;
						st_is_pay_r <= 1'b0;
					end else begin
						nop_r <= !u_active && !is_allowed_op_r;
						st_is_pay_r <= !h_zero_r;
					end
`elsif SYNTH_KINTEX_ULTRASCALE
					if (dec_fault_s) begin
						f_pulse <= 1'b1;
						f_kind <= dec_short_s ? GHCI_FAULT_KIND_BAD_REG :
							h_early_absent_r ? GHCI_FAULT_KIND_KICK_ABSENT :
							GHCI_FAULT_KIND_INVALID_OP;
						st_is_pay_r <= 1'b0;
					end else begin
						nop_r <= !u_active && !is_allowed_op_r;
						st_is_pay_r <= !h_zero_r;
					end
`else
					if (dec_fault_r2) begin
						f_pulse <= 1'b1;
						f_kind <= dec_short_r2 ? GHCI_FAULT_KIND_BAD_REG :
							h_early_absent_r ? GHCI_FAULT_KIND_KICK_ABSENT :
							GHCI_FAULT_KIND_INVALID_OP;
					end else begin
						nop_r <= !u_active && !is_allowed_op_r;
					end
`endif
				end
			end
			Q_PAY:
				if (bad_absent_comb && !nop_r) begin
					f_pulse <= 1'b1;
					f_kind <= GHCI_FAULT_KIND_KICK_ABSENT;
`ifdef SYNTH_KINTEX7
					st_is_pay_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
					st_is_pay_r <= 1'b0;
`endif
				end else if (bad_idx_comb && !nop_r) begin
					f_pulse <= 1'b1;
					f_kind <= GHCI_FAULT_KIND_BAD_REG;
`ifdef SYNTH_KINTEX7
					st_is_pay_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
					st_is_pay_r <= 1'b0;
`endif
				end else if (pay_take_b[3] && p_is_last_r) begin
`ifdef SYNTH_KINTEX7
					if (nop_r) begin
						st_is_pay_r <= 1'b0;
						settle <= 1'b0;
					end else begin
						st_is_pay_r <= 1'b0;
						settle <= 1'b1;
					end
`elsif SYNTH_KINTEX_ULTRASCALE
					if (nop_r) begin
						st_is_pay_r <= 1'b0;
						settle <= 1'b0;
					end else begin
						st_is_pay_r <= 1'b0;
						settle <= 1'b1;
					end
`endif
				end
			Q_EXEC: begin
				if (seq_abort_b[3]) begin
`ifdef SYNTH_KINTEX7
					st_is_pay_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
					st_is_pay_r <= 1'b0;
`endif
				end else if (nop_r) begin
`ifdef SYNTH_KINTEX7
					settle <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
					settle <= 1'b0;
`endif
				end else if (is_query_caps_r) begin
					if (qc_align_err) begin
						f_pulse <= 1'b1;
						f_kind <= GHCI_FAULT_KIND_INVALID_OP;
`ifdef SYNTH_KINTEX7
						st_is_pay_r <= 1'b0;
`elsif SYNTH_KINTEX_ULTRASCALE
						st_is_pay_r <= 1'b0;
`endif
					end else if (qc_zero_r) begin
						;
					end else begin
						qc_k <= 7'd0;
						qc_k_cap <= 7'd0;
						qc_cur_addr_hi <= qc_addr_hi;
						qc_word_sel_r <= 1'b0;
						qc_r_rdy <= 1'b0;
					end
				end else if (exec_go) begin
`ifdef SYNTH_KINTEX7
					if (exec_settle_r)
						settle <= 1'b1;
`elsif SYNTH_KINTEX_ULTRASCALE
					if (exec_settle_r)
						settle <= 1'b1;
`endif
				end
			end
			Q_QCAPS_R0: begin
				qc_last_r <= (qc_k_n == qc_total_k);
				if (cfws_b[0]) begin
					qc_r_rdy <= 1'b0;
				end else if (!qc_r_rdy) begin
					qc_r_rdy <= 1'b1;
				end else begin
					if (!cfws_w0_lo) qc_w0[15:0] <= caps_r_data[15:0];
					if (!cfws_w0_hi) qc_w0[31:16] <= caps_r_data[31:16];
					qc_word_sel_r <= 1'b1;
					qc_r_rdy <= 1'b0;
				end
			end
			Q_QCAPS_R1: begin
				if (cfws_b[2]) begin
					qc_r_rdy <= 1'b0;
				end else if (!qc_r_rdy) begin
					qc_r_rdy <= 1'b1;
				end else begin
					if (!cfws_w1_lo) qc_w1[15:0] <= caps_r_data[15:0];
					if (!cfws_w1_hi) qc_w1[31:16] <= caps_r_data[31:16];
					qc_r_rdy <= 1'b0;
				end
			end
			Q_QCAPS_WAIT: begin
				if (!f_busy) begin
					qc_k <= qc_k_n;
					qc_k_cap <= qc_k_n;
					qc_cur_addr_hi <= qc_cur_addr_hi_n;
					if (!qc_last_r) begin
						qc_word_sel_r <= 1'b0;
						qc_r_rdy <= 1'b0;
					end
				end
			end
			default:
				;
			endcase
			if (qca_ce_lo) qc_cur_addr[15:0]  <= st_exec_qca_lo ? qc_addr_lo[15:0]  : qc_cur_addr_n[15:0];
			if (qca_ce_hi) qc_cur_addr[31:16] <= st_exec_qca_hi ? qc_addr_lo[31:16] : qc_cur_addr_n[31:16];

			if (occ_we_b[0]) occ_q[13:0] <= occ_count[13:0];
			if (occ_we_b[1]) occ_q[27:14] <= occ_count[27:14];
			if (occ_we_b[2]) occ_q[39:28] <= occ_count[39:28];
		end
	end

	wire unused_p = &{1'b0, p_co, ghci_sub_bo, n_m1_bo, n_m2_bo, is_ghci_hdr, dec_hdr_fault, dec_hdr_absent, dec_hdr_short, dec_fault_r, dec_short_r, dec_absent_r, set_bad, pay_absent, pay_take, s_wait_in[0], s_wait_in[7], pay_wait, fence_flags[31:3], l2_idle, qc_addr_hi_co, qc_k_co, qca_ce_lo, qca_ce_hi};
endmodule

`default_nettype wire
