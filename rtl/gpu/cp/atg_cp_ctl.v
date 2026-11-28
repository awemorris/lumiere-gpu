/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cp_ctl: the CP's state machine (design §2.6): CTRL (RUN, RESET,
 * HALT, STEP, WDT_EN), STATUS, HALT and quiesce, the
 * RESET order of design §1.3 (with the STATE replay), the watchdog,
 * AXI_STUCK, and FAULT_ADDR/FAULT_INFO from all
 * fault sources.
 *
 * One state register st:
 *   IDLE     →(RUN, !HALT)→ RUNNING; →(HALT)→ HALTING
 *   RUNNING  →(HALT)→ HALTING; →(!RUN and the CP not busy)→ IDLE
 *   HALTING  →(mc_idle and the ring at a packet boundary)→ HALTED (ev_halted);
 *            →(!HALT)→ RUNNING/IDLE
 *   HALTED   →(!HALT)→ RUNNING (RUN) or IDLE; STEP pulses step
 *   any but RESETTING →(a fault)→ FAULTED (first fault recorded, ev_fault)
 *   any →(CTRL.RESET)→ RS_WAIT →(mc_idle)→ RS_PULSE (core_rst_n 0, mc_flush,
 *            RUN/HALT/FAULT cleared) → RS_REPLAY →(!replay_busy)→ IDLE
 * HALT quiesces at once (the blocks freeze at their next memory
 * access): HALTED waits only for memory idle and the ring's
 * packet boundary.  quiesce is 1 in HALTING, HALTED, FAULTED and the RESET
 * order.  The watchdog counts cycles with cmd_active while WDT_EN, except
 * those with wdt_hold (the count pauses); at WDT_CYCLES it is a fault of
 * kind 4 (and ev_wdt).  The hardware never
 * resets itself.
 *
 * Faults, first one wins (priority in one cycle: the MC, this module's own
 * (locked write, watchdog), then src 0..NSRC-1):
 *   MC refusal of client c != 0: FAULT_INFO {0, c, 1, 0, write, c}
 *   src k: {tag, SRC_BLOCK[k], kind, 0, 0, SRC_CLIENT[k]}, FAULT_ADDR addr
 *   watchdog: kind 4, [31:24] wdt_tag (session protection: the session whose
 *   work the units are draining, else 0);
 *   RING_BASE_LO/HI, RING_SIZE, CP_HEAP_* written when running (not IDLE): kind 7 (LOCKED_REG).
 * MC refusals of client 0 (the CP's own port) are reported by the ring,
 * the fence unit or the debug port themselves; those of DC_CLIENT (a
 * DISP2 controller's scan-out reads) are not a device fault: the DC shows
 * black and reports them itself (design §18.2).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cp_ctl #(
	parameter ADDR_W = 32,
	parameter NSRC = 8,	/* Number of fault sources (0 ring, 1 seq, 2 state, 3 fence, 4 SU, 5 CPU, 6 BLT, 7 other blocks, 8 DISP) */
	parameter WDT_RESET = 32'h10000000,	/* WDT_CYCLES after reset (2^28) */
	parameter LINE_SHIFT = 5,	/* log2 of the line size */
	parameter [7:0] UNITS = 8'h03,	/* Present blocks, STATUS busy-bit order (DRAW: 2, DISPATCH: 6) */
	parameter [NSRC*8-1:0] SRC_BLOCK = 64'h00_06_00_02_00_00_00_00,	/* FAULT_INFO[7:4] per source */
	parameter [NSRC*8-1:0] SRC_CLIENT = 64'h00_01_00_00_00_00_00_00,	/* FAULT_INFO[7:0] per source */
	parameter [7:0] DC_CLIENT = 8'hFF,	/* the DC's MC client, whose refusals are not faults (0xFF: none) */
	parameter [31:0] APERTURE_BASE_INIT = 32'd0,
	parameter [31:0] APERTURE_LIMIT_INIT = 32'hFFFFFFFF,
	parameter C_CPU = 2,
	parameter C_VF = 3,
	parameter C_TU = 4,
	parameter C_ROP = 5,
	parameter C_ATOM = 6,
	parameter C_SA = 7,
	parameter C_DC = 8'hFF,
	parameter C_SU = 8,
	parameter HAS_PROT = 0,
	parameter HAS_HW_WALKER = 0,
	parameter VA_BITS = 32
) (
	input wire clk,
	input wire rst_n,
	/* HCR register snoop (from atg_cp_regs) */
	input wire hcr_we,
	input wire [9:0] hcr_widx,
	input wire [31:0] hcr_wdata,
	input wire [31:0] hcr_wmask,
	input wire [9:0] hcr_ridx,
	output wire [31:0] hcr_rdata,
	/* Inputs */
	input wire mc_idle,
	input wire mem_ready,
	input wire [7:0] busy_in,
	input wire ring_empty,
	input wire ring_busy,
	input wire cmd_active,
	input wire wdt_hold,	/* the count pauses (the CPU waits on its FIFO or memory) */
	input wire replay_busy,
	input wire [3:0] seq_st,
	input wire f_busy,
	input wire mc_fault,
	input wire [7:0] mc_fault_client,
	input wire [ADDR_W-1:0] mc_fault_addr,
	input wire mc_fault_write,
	input wire mc_fault_prot,	/* refused for session protection: FAULT_INFO[9] */
	input wire mc_fault_mem,	/* a protection table read failed: kind 5 */
	input wire [7:0] mc_fault_tag,
	input wire [7:0] mc_fault_req_tag,
	input wire mmu_fault,
	input wire [3:0] mmu_fault_kind,
	input wire [7:0] mmu_fault_client,
	input wire [3:0] mmu_fault_session,
	input wire [VA_BITS-1:0] mmu_fault_addr,
	input wire [7:0] wdt_tag,	/* the job tag of a watchdog fault */
	input wire [NSRC-1:0] src_fault,
	input wire [NSRC*4-1:0] src_kind,
	input wire [NSRC*32-1:0] src_addr,
	input wire [NSRC*8-1:0] src_tag,
	input wire [NSRC*4-1:0] src_stage,
	/* Outputs */
	output wire run,
	output wire halt,
	output wire ctrl_irq_global,
	output wire quiesce,
	output wire mc_flush,
	output wire [3:0] mc_flush_rep,
	output wire core_rst_n,
	output wire open,
	output wire [3:0] cp_state,
	output wire [2:0] lc_state,
	output wire lc_halting_o,
	output wire lc_halted_o,
	output wire lc_abort_o,
	output wire [2:0] host_kick,
	output wire step,
	output wire ev_halted,
	output wire ev_fault,
	output wire ev_wdt,
	output wire fault_session_clear,
	output wire [7:0] fault_session_tag,
	output wire [ADDR_W-1:0] aperture_base,
	output wire [ADDR_W-1:0] aperture_limit
);
	`include "ghci_regs.vh"

	localparam [2:0] S_IDLE = 3'd0;
	localparam [2:0] S_RUN = 3'd1;
	localparam [2:0] S_HALTING = 3'd2;
	localparam [2:0] S_HALTED = 3'd3;
	localparam [2:0] S_FAULTED = 3'd4;
	localparam [2:0] S_RS_WAIT = 3'd5;
	localparam [2:0] S_RS_PULSE = 3'd6;
	localparam [2:0] S_RS_REPLAY = 3'd7;

	localparam [9:0] I_STATUS       = HCR_IDX_STATUS;
	localparam [9:0] I_CTRL         = HCR_IDX_CTRL;
	localparam [9:0] I_FAULT_INFO   = HCR_IDX_FAULT_INFO;
	localparam [9:0] I_FAULT_ADDR   = HCR_IDX_FAULT_ADDR_LO;
	localparam [9:0] I_FAULT_ADDR_HI= HCR_IDX_FAULT_ADDR_HI;
	localparam [9:0] I_WDT          = HCR_IDX_WDT_CYCLES;
	localparam [9:0] I_HEAP_BASE    = HCR_IDX_CP_HEAP_BASE;
	localparam [9:0] I_HEAP_LIMIT   = HCR_IDX_CP_HEAP_LIMIT;
	localparam [9:0] I_RING_BASE_LO = HCR_IDX_RING_BASE_LO;
	localparam [9:0] I_RING_BASE_HI = HCR_IDX_RING_BASE_HI;
	localparam [9:0] I_RING_SIZE    = HCR_IDX_RING_SIZE;
	localparam [31:0] AP_MASK = ~((32'd1 << LINE_SHIFT) - 32'd1);

	(* max_fanout = 16 *) reg [2:0] st;
	(* max_fanout = 12 *) reg lc_halting_r;
	reg lc_halted_r, lc_abort_r;
	assign lc_halting_o = lc_halting_r;
	assign lc_halted_o  = lc_halted_r;
	assign lc_abort_o   = lc_abort_r;
	reg ctrl_run;
	(* max_fanout = 16 *) reg ctrl_halt;
	reg ctrl_wdt_en;
	reg irq_global;
	reg [31:0] wdt_cycles;
	reg [31:0] ap_base;
	reg [31:0] ap_limit;
	reg [31:0] heap_base;
	reg [31:0] heap_limit;
	(* max_fanout = 16 *) reg [31:0] fault_addr;
	reg [31:0] fault_addr_hi;
	(* max_fanout = 16 *) reg [31:0] fault_info;
	reg [31:0] wdt_count;
	reg [33:0] stuck_count;
	/* The watchdog's and AXI_STUCK's counts + 1 (alternating NAND/NOR prefix, <= 6 cmos3). */
	function [31:0] fn_inc32;
		input [31:0] x;
		reg [31:0] s1_n, s2, s3_n, s4, s5_n;
		begin
			s1_n = ~(x    & {x[30:0],     1'b1});
			s2   = ~(s1_n | {s1_n[29:0],  2'b00});
			s3_n = ~(s2   & {s2[27:0],    4'hF});
			s4   = ~(s3_n | {s3_n[23:0],  8'h00});
			s5_n = ~(s4   & {s4[15:0],   16'hFFFF});
			fn_inc32 = x ^ {~s5_n[30:0], 1'b1};
		end
	endfunction

	function [33:0] fn_inc34;
		input [33:0] x;
		reg [33:0] s1_n, s2, s3_n, s4, s5_n, s6;
		begin
			s1_n = ~(x    & {x[32:0],     1'b1});
			s2   = ~(s1_n | {s1_n[31:0],  2'b00});
			s3_n = ~(s2   & {s2[29:0],    4'hF});
			s4   = ~(s3_n | {s3_n[25:0],  8'h00});
			s5_n = ~(s4   & {s4[17:0],   16'hFFFF});
			s6   = ~(s5_n | {s5_n[1:0],  32'd0});
			fn_inc34 = x ^ {s6[32:0], 1'b1};
		end
	endfunction

	function fn_cmp_ge32;
		input [31:0] a_in;
		input [31:0] b_in;
		reg [32:0] av, bv;
		reg [10:0] g1, p1;
		reg [3:0]  g2, p2;
		reg g3_0, p3_0, g3_1, p3_1;
		reg e0, e1, e2, gt;
		integer j;
		begin
			av = {1'b0, a_in};
			bv = {1'b0, b_in};
			for (j = 0; j < 11; j = j + 1) begin
				e0 = ~(av[3*j+0] ^ bv[3*j+0]);
				e1 = ~(av[3*j+1] ^ bv[3*j+1]);
				e2 = ~(av[3*j+2] ^ bv[3*j+2]);
				p1[j] = e2 & e1 & e0;
				gt = (av[3*j+2] & ~bv[3*j+2]) | (e2 & av[3*j+1] & ~bv[3*j+1]) | (e2 & e1 & av[3*j+0] & ~bv[3*j+0]);
				g1[j] = (j == 0) ? (gt | p1[0]) : gt;
			end
			g2[0] = g1[2] | (p1[2] & g1[1]) | (p1[2] & p1[1] & g1[0]);
			p2[0] = p1[2] & p1[1] & p1[0];
			g2[1] = g1[5] | (p1[5] & g1[4]) | (p1[5] & p1[4] & g1[3]);
			p2[1] = p1[5] & p1[4] & p1[3];
			g2[2] = g1[8] | (p1[8] & g1[7]) | (p1[8] & p1[7] & g1[6]);
			p2[2] = p1[8] & p1[7] & p1[6];
			g2[3] = g1[10] | (p1[10] & g1[9]);
			p2[3] = p1[10] & p1[9];
			g3_0 = g2[1] | (p2[1] & g2[0]);
			p3_0 = p2[1] & p2[0];
			g3_1 = g2[3] | (p2[3] & g2[2]);
			p3_1 = p2[3] & p2[2];
			fn_cmp_ge32 = g3_1 | (p3_1 & g3_0);
		end
	endfunction
	function fn_cmp_lt32;
		input [31:0] a_in;
		input [31:0] b_in;
		begin
			fn_cmp_lt32 = !fn_cmp_ge32(a_in, b_in);
		end
	endfunction

	wire [31:0] wdt_count_n   = fn_inc32(wdt_count);
	wire [33:0] stuck_count_n = fn_inc34(stuck_count);
	wire wdt_co = 1'b0;
	wire stuck_co = 1'b0;
	reg axi_stuck;

	wire resetting = st[2] && (st[1] || st[0]);
	wire faulted = st == S_FAULTED;
	reg st_crst_n_r;
`ifdef SYNTH_KINTEX7
	(* keep = "true", dont_touch = "true" *) wire rst_ctl_int;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_rst_ctl_inv (.I0(rst_n), .O(rst_ctl_int));

	(* keep = "true", dont_touch = "true" *) wire core_rst_n_sig;
	(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h8AAA)) u_core_rst_n_lut (
		.I0(rst_n), .I1(st[0]), .I2(st[1]), .I3(st[2]), .O(core_rst_n_sig)
	);
	assign core_rst_n = core_rst_n_sig;
`elsif SYNTH_KINTEX_ULTRASCALE
	(* keep = "true", dont_touch = "true" *) wire rst_ctl_int;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h1)) u_rst_ctl_inv (.I0(rst_n), .O(rst_ctl_int));

	/* Keep core_rst_n as a fabric net so opt_design does not insert a BUFGCE before reset syncs. */
	(* keep = "true", dont_touch = "true" *) wire core_rst_n_sig;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'h8)) u_core_rst_n_lut (
		.I0(rst_n), .I1(st_crst_n_r), .O(core_rst_n_sig)
	);
	assign core_rst_n = core_rst_n_sig;
`else
	wire rst_ctl_int = ~rst_n;
	assign core_rst_n = rst_n && st_crst_n_r;
`endif

	(* max_fanout = 16 *) reg [3:0] open_r;
	(* max_fanout = 16 *) reg [3:0] can_fault_r;
	(* DONT_TOUCH = "TRUE" *) reg [2:0] halt_r;
	(* DONT_TOUCH = "TRUE" *) reg st_is_run_r;
	(* keep = "true" *) reg [3:0] st_mcf_rep;
	assign open = open_r[0];
	assign ctrl_irq_global = irq_global;
	wire halt_raw = st == S_HALTING || st == S_HALTED || faulted || resetting;
	assign run = st_is_run_r && ctrl_run;
	wire halt_int = halt_r[0];
	assign halt = halt_r[1];
	assign quiesce = halt_r[2];
	assign mc_flush = st == S_RS_PULSE;
	assign mc_flush_rep = st_mcf_rep;
	/* Micro-architectural CP_STATE (§3.3) — flat seq_st decode + isolate mc_idle to final 2:1 mux */
	wire cp_is_fault = faulted;
	wire cp_is_wait_halt_mc = !faulted && (st == S_HALTING) && !mc_idle;
	wire seq_is_idle    = (seq_st == 4'd0);
	wire seq_is_waitmem = (seq_st == 4'd10) || (seq_st == 4'd11);
	wire seq_is_fwait   = (seq_st == 4'd5);
	wire seq_is_decode  = (seq_st == 4'd8) || (seq_st == 4'd9);
	wire seq_is_exec    = (seq_st == 4'd1) || (seq_st == 4'd2) || (seq_st == 4'd3) ||
		(seq_st == 4'd4) || (seq_st == 4'd6) || (f_busy && !seq_is_fwait);
	wire cp_is_wait_mem_nomc = resetting || seq_is_waitmem;
	wire cp_is_blit = busy_in[1]; /* blt_busy */
	wire cp_is_wait_fence = seq_is_fwait; /* Q_FWAIT strictly */
	wire cp_is_exec = seq_is_exec;
	wire cp_is_decode = seq_is_decode;
	wire halt_ok = mc_idle && (!ring_busy || (seq_is_idle && !f_busy));
	wire cp_is_fetch = (st != S_HALTED) && (ring_busy || (!ring_empty && cmd_active));

	wire n_s_0 = !seq_st[0];
	wire n_s_2 = !seq_st[2];
	wire n_s_3 = !seq_st[3];
	wire s_hi_00 = !(seq_st[3] | seq_st[2]);
	wire s_hi_01 = !(seq_st[3] | n_s_2);
	wire s_hi_10 = !(n_s_3 | seq_st[2]);
	wire s_hi_11 = !(n_s_3 | n_s_2);
	wire s_lo_00 = !(seq_st[1] | seq_st[0]);
	wire s_def_7_12 = !(!s_hi_11 & !(s_hi_01 & seq_st[1] & seq_st[0]));
	wire df0_w = f_busy | cp_is_fetch;
	wire b0_c0_n = !(s_hi_00 & !s_lo_00);
	wire b0_c1_n = !(s_hi_01 & n_s_0);
	wire b0_c2_n = !(s_hi_10 & seq_st[1]);
	wire b0_d0_n = !(df0_w & s_hi_00 & s_lo_00);
	wire b0_d1_n = !(df0_w & s_def_7_12);
	wire b1_c2_n = !(s_hi_10 & !seq_st[1]);
	wire b1_d0_n = !(f_busy & s_hi_00 & s_lo_00);
	wire b1_d1_n = !(f_busy & s_def_7_12);
	wire seq_b2_c = !(!(s_hi_10 & seq_st[1]) & !(s_hi_01 & !seq_st[1] & seq_st[0]));
	wire b0_c_ok  = !(b0_c0_n & b0_c1_n & b0_c2_n);
	wire b0_d_ok  = !(b0_d0_n & b0_d1_n);
	wire b0_n     = !(b0_c_ok | b0_d_ok);
	wire b1_c_ok  = !(b0_c0_n & b0_c1_n & b1_c2_n);
	wire b1_d_ok  = !(b1_d0_n & b1_d1_n);
	wire b1_n     = !(b1_c_ok | b1_d_ok);
	wire cp_nx_b0 = !(!(use_override_st & override_st_val[0]) & !( !(use_override_st | b0_n) ));
	wire cp_nx_b1 = !(!(use_override_st & override_st_val[1]) & !( !(use_override_st | b1_n) ));
	wire cp_nx_b2 = use_override_st ? override_st_val[2] : seq_b2_c;
	wire cp_nx_b3 = use_override_st ? override_st_val[3] : 1'b0;
	wire [3:0] cp_state_nx_c = {cp_nx_b3, cp_nx_b2, cp_nx_b1, cp_nx_b0};
	wire seq_nz_w = !(!(seq_st[3] | seq_st[2]) & !(seq_st[1] | seq_st[0]));
	wire ring_seq_busy_w = ring_busy && (f_busy | seq_nz_w);
	wire seq_f_idle_w = !f_busy && !seq_nz_w;
	wire use_override_st = cp_is_wait_halt_mc || cp_is_fault || resetting || cp_is_blit;
	wire [3:0] override_st_val =
		cp_is_wait_halt_mc ? GHCI_ST_WAIT_MEM :
		cp_is_fault        ? GHCI_ST_FAULT :
		resetting          ? GHCI_ST_WAIT_MEM : GHCI_ST_BLIT;
	wire unused_cp_flags = &{1'b0, cp_is_wait_mem_nomc, cp_is_wait_fence, cp_is_exec, cp_is_decode};
	wire [63:0] mc_fault_addr_ext = {{(64-ADDR_W){1'b0}}, mc_fault_addr};
	wire [63:0] mmu_fault_addr_ext = {{(64-VA_BITS){1'b0}}, mmu_fault_addr};
	wire [63:0] ap_base_ext = {32'd0, ap_base};
	wire [63:0] ap_limit_ext = {(&ap_limit ? 32'hFFFF_FFFF : 32'd0), ap_limit};
	reg [3:0] cp_state_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			cp_state_r <= GHCI_ST_IDLE;
		else
			cp_state_r <= cp_state_nx_c;
	end
	assign cp_state = cp_state_r;
	assign lc_state = resetting ? GHCI_LC_RESETTING : st;
	assign aperture_base = ap_base_ext[ADDR_W-1:0];
	assign aperture_limit = ap_limit_ext[ADDR_W-1:0];

	/* Host writes (balanced 10-bit index decode against I_CTRL). */
	wire idx_m_lo  = (hcr_widx[3:0] == I_CTRL[3:0]);
	wire idx_m_mid = (hcr_widx[6:4] == I_CTRL[6:4]);
	wire idx_m_hi  = (hcr_widx[9:7] == I_CTRL[9:7]);
	wire hcr_idx_zero = idx_m_lo && (idx_m_mid && idx_m_hi);
	wire w0 = hcr_we && hcr_wmask[0];
	wire wr_ctrl = w0 && hcr_idx_zero;
	wire wr_rst_no_idx = w0 && hcr_wdata[GHCI_CTRL_RESET];
	wire wr_reset = hcr_idx_zero && wr_rst_no_idx;
	wire wr_reset_go = hcr_idx_zero && (wr_rst_no_idx && !resetting);
	reg step_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			step_r <= 1'b0;
		else
			step_r <= wr_ctrl && hcr_wdata[GHCI_CTRL_STEP] && (st == S_HALTED);
	end
	assign step = step_r;

	assign host_kick = 3'd0;
	wire kick_absent = 1'b0;
	wire kick_bad = 1'b0;

	wire wr_locked_hi = !(hcr_widx[9] | hcr_widx[8] | hcr_widx[7] | hcr_widx[6] | hcr_widx[5]);
	wire wr_p0_we = hcr_we && wr_locked_hi;
	wire wr_locked_9_11  = !hcr_widx[4] &&  hcr_widx[3] && !hcr_widx[2] && (hcr_widx[1] | hcr_widx[0]);
	wire wr_locked_20_21 =  hcr_widx[4] && !hcr_widx[3] &&  hcr_widx[2] && !hcr_widx[1];
	wire locked_bad = (hcr_we && !open) && (wr_locked_hi && (wr_locked_9_11 || wr_locked_20_21));
	wire wr_fi_lo = (hcr_widx[4:0] == I_FAULT_INFO[4:0]);
	wire wr_fault_info = (hcr_we && wr_locked_hi) && wr_fi_lo;

`ifdef SYNTH_KINTEX7
	reg wdt_fire_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			wdt_fire_r <= 1'b0;
		else
			wdt_fire_r <= ctrl_wdt_en && cmd_active && !fn_cmp_lt32(wdt_count, wdt_cycles) && (st != S_FAULTED);
	end
	wire wdt_fire = wdt_fire_r;
`elsif SYNTH_KINTEX_ULTRASCALE
	reg wdt_fire_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			wdt_fire_r <= 1'b0;
		else
			wdt_fire_r <= ctrl_wdt_en && cmd_active && !fn_cmp_lt32(wdt_count, wdt_cycles) && (st != S_FAULTED);
	end
	wire wdt_fire = wdt_fire_r;
`else
	reg wdt_fire_r;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n)
			wdt_fire_r <= 1'b0;
		else
			wdt_fire_r <= ctrl_wdt_en && cmd_active && !fn_cmp_lt32(wdt_count, wdt_cycles) && (st != S_FAULTED);
	end
	wire wdt_fire = wdt_fire_r;
`endif

	/* The first fault of this cycle. */
	/* The MC's refusals of client 0 (the CP) are its own units' faults, except
	 * those of session protection (a firmware µop's fence or result). */
	wire mc_gpu = mc_fault && (mc_fault_client != 8'd0 || mc_fault_prot || mc_fault_mem) &&
		!(mc_fault_client == DC_CLIENT && !mc_fault_prot && !mc_fault_mem);
`ifdef SYNTH_KINTEX7
	wire [5:0] mmu_flt_b;
	genvar g_mf;
	generate
		for (g_mf = 0; g_mf < 6; g_mf = g_mf + 1) begin : g_mfb
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_mfb (.I0(mmu_fault), .O(mmu_flt_b[g_mf]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [5:0] mmu_flt_b;
	genvar g_mf;
	generate
		for (g_mf = 0; g_mf < 6; g_mf = g_mf + 1) begin : g_mfb
			assign mmu_flt_b[g_mf] = mmu_fault;
		end
	endgenerate
`else
	wire [5:0] mmu_flt_b = {6{mmu_fault}};
`endif
	wire bus_fault = mc_gpu || mmu_flt_b[0];
	wire [7:0] bus_fault_client = mmu_flt_b[0] ? mmu_fault_client : mc_fault_client;
	wire [7:0] bus_fault_tag = mmu_flt_b[1] ? {4'd0, mmu_fault_session} : mc_fault_tag;
	wire [3:0] bus_fault_kind = mmu_flt_b[1] ? mmu_fault_kind :
		(mc_fault_mem ? GHCI_FAULT_KIND_AXI_ERROR :
		(mc_fault_prot ? GHCI_FAULT_KIND_PERM_FAULT : GHCI_FAULT_KIND_PAGE_FAULT));
	wire bus_fault_write = mmu_flt_b[1] ? 1'b0 : mc_fault_write;
	wire [3:0] bus_fault_req_tag = mmu_flt_b[1] ? 4'd0 : mc_fault_req_tag[3:0];
	wire [63:0] bus_fault_addr_ext = {
		mmu_flt_b[5] ? mmu_fault_addr_ext[63:48] : mc_fault_addr_ext[63:48],
		mmu_flt_b[4] ? mmu_fault_addr_ext[47:32] : mc_fault_addr_ext[47:32],
		mmu_flt_b[3] ? mmu_fault_addr_ext[31:16] : mc_fault_addr_ext[31:16],
		mmu_flt_b[2] ? mmu_fault_addr_ext[15:0]  : mc_fault_addr_ext[15:0]
	};

	function [3:0] mc_client_to_unit;
		input [7:0] client;
		reg c0, c1, c_cpu, c_vf, c_tu, c_rop, c_pre_su, c_dc;
		begin
			c0       = (client == 8'd0);
			c1       = (client == 8'd1);
			c_cpu    = !c0 && !c1 && (C_CPU != C_VF) && (client == C_CPU[7:0]);
			c_vf     = !c0 && !c1 && !c_cpu && (C_VF != C_TU) && (client == C_VF[7:0]);
			c_tu     = !c0 && !c1 && !c_cpu && !c_vf && (C_TU != C_ROP) && (client == C_TU[7:0]);
			c_rop    = !c0 && !c1 && !c_cpu && !c_vf && !c_tu && (C_ROP != C_ATOM) && (client == C_ROP[7:0]);
			c_pre_su = ((C_ATOM != C_SA) && (client == C_ATOM[7:0])) ||
			           ((C_SA != C_DC) && (client == C_SA[7:0]));
			c_dc     = !c0 && !c1 && !c_cpu && !c_vf && !c_tu && !c_rop && !c_pre_su &&
			           (DC_CLIENT != 8'hFF) && (client == DC_CLIENT[7:0]);
			mc_client_to_unit = (c0 || c_cpu || c_dc) ? GHCI_FAULT_UNIT_CP :
			                    c1                    ? GHCI_FAULT_UNIT_BLT :
			                    c_vf                  ? GHCI_FAULT_UNIT_VF :
			                    c_tu                  ? GHCI_FAULT_UNIT_TU :
			                    c_rop                 ? GHCI_FAULT_UNIT_ROP :
			                                            GHCI_FAULT_UNIT_SU;
		end
	endfunction
	reg any;
	reg [31:0] n_info;
	reg [31:0] n_addr;
	integer k;

`ifdef SYNTH_KINTEX7
	wire [NSRC-1:0] sf_lo, sf_hi, sf_alo, sf_ahi;
	genvar g_sf;
	generate
		for (g_sf = 0; g_sf < NSRC; g_sf = g_sf + 1) begin : g_sf_buf
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sflo  (.I0(src_fault[g_sf]), .O(sf_lo[g_sf]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sfhi  (.I0(src_fault[g_sf]), .O(sf_hi[g_sf]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sfalo (.I0(src_fault[g_sf]), .O(sf_alo[g_sf]));
			(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_sfahi (.I0(src_fault[g_sf]), .O(sf_ahi[g_sf]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [NSRC-1:0] sf_lo, sf_hi, sf_alo, sf_ahi;
	genvar g_sf;
	generate
		for (g_sf = 0; g_sf < NSRC; g_sf = g_sf + 1) begin : g_sf_buf
			assign sf_lo[g_sf] = src_fault[g_sf];
			assign sf_hi[g_sf] = src_fault[g_sf];
			assign sf_alo[g_sf] = src_fault[g_sf];
			assign sf_ahi[g_sf] = src_fault[g_sf];
		end
	endgenerate
`else
	wire [NSRC-1:0] sf_lo  = src_fault;
	wire [NSRC-1:0] sf_hi  = src_fault;
	wire [NSRC-1:0] sf_alo = src_fault;
	wire [NSRC-1:0] sf_ahi = src_fault;
`endif

	reg f_s0_v_c, f_s1_v_c, f_s2_v_c;
	reg [31:0] f_s0_info_c, f_s1_info_c, f_s2_info_c;
	reg [15:0] f_s0_addr_lo_c, f_s2_addr_lo_c;
	reg [15:0] f_s0_addr_hi_c, f_s2_addr_hi_c;
	reg f_s0_nz_c, f_s1_nz_c, f_s2_nz_c;
	wire [NSRC-1:0] src_nz_w;
	genvar g_snz;
	generate
		for (g_snz = 0; g_snz < NSRC; g_snz = g_snz + 1) begin : g_src_nz
			assign src_nz_w[g_snz] = (|src_tag[g_snz*8 +: 8]) || (|SRC_BLOCK[g_snz*8 +: 4]) ||
				(|src_kind[g_snz*4 +: 4]) || (|src_stage[g_snz*4 +: 4]);
		end
	endgenerate
	wire [15:0] f_s1_addr_lo_c, f_s1_addr_hi_c;
`ifdef SYNTH_KINTEX7
	genvar g_s1a;
	generate
		for (g_s1a = 0; g_s1a < 16; g_s1a = g_s1a + 1) begin : g_s1_addr_mux
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_s1_alo (
				.I0(src_addr[5*32 + g_s1a]),
				.I1(src_addr[4*32 + g_s1a]),
				.I2(src_addr[3*32 + g_s1a]),
				.I3(sf_alo[4]),
				.I4(sf_alo[3]),
				.O(f_s1_addr_lo_c[g_s1a])
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_s1_ahi (
				.I0(src_addr[5*32 + 16 + g_s1a]),
				.I1(src_addr[4*32 + 16 + g_s1a]),
				.I2(src_addr[3*32 + 16 + g_s1a]),
				.I3(sf_ahi[4]),
				.I4(sf_ahi[3]),
				.O(f_s1_addr_hi_c[g_s1a])
			);
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	genvar g_s1a;
	generate
		for (g_s1a = 0; g_s1a < 16; g_s1a = g_s1a + 1) begin : g_s1_addr_mux
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_s1_alo (
				.I0(src_addr[5*32 + g_s1a]),
				.I1(src_addr[4*32 + g_s1a]),
				.I2(src_addr[3*32 + g_s1a]),
				.I3(sf_alo[4]),
				.I4(sf_alo[3]),
				.O(f_s1_addr_lo_c[g_s1a])
			);
			(* DONT_TOUCH = "TRUE" *) LUT5 #(.INIT(32'hF0F0CCAA)) u_s1_ahi (
				.I0(src_addr[5*32 + 16 + g_s1a]),
				.I1(src_addr[4*32 + 16 + g_s1a]),
				.I2(src_addr[3*32 + 16 + g_s1a]),
				.I3(sf_ahi[4]),
				.I4(sf_ahi[3]),
				.O(f_s1_addr_hi_c[g_s1a])
			);
		end
	endgenerate
`else
	assign f_s1_addr_lo_c = sf_alo[3] ? src_addr[3*32 +: 16] : sf_alo[4] ? src_addr[4*32 +: 16] : src_addr[5*32 +: 16];
	assign f_s1_addr_hi_c = sf_ahi[3] ? src_addr[3*32+16 +: 16] : sf_ahi[4] ? src_addr[4*32+16 +: 16] : src_addr[5*32+16 +: 16];
`endif

	always @(*) begin
		f_s0_v_c = 1'b0;
		f_s0_info_c = 32'd0;
		f_s0_addr_lo_c = 16'd0;
		f_s0_addr_hi_c = 16'd0;
		f_s0_nz_c = 1'b0;
		for (k = 2; k >= 0; k = k - 1) begin
			if (sf_lo[k]) begin
				f_s0_v_c = 1'b1;
				f_s0_info_c[15:0] = {src_tag[k * 8 +: 8], SRC_BLOCK[k * 8 +: 4], src_kind[k * 4 +: 4]};
				f_s0_nz_c = src_nz_w[k];
			end
			if (sf_alo[k]) begin
				f_s0_addr_lo_c = src_addr[k * 32 +: 16];
			end
			if (sf_hi[k]) begin
				f_s0_info_c[31:16] = {12'd0, src_stage[k * 4 +: 4]};
			end
			if (sf_ahi[k]) begin
				f_s0_addr_hi_c = src_addr[k * 32 + 16 +: 16];
			end
		end

		f_s1_v_c = 1'b0;
		f_s1_info_c = 32'd0;
		f_s1_nz_c = 1'b0;
		for (k = 5; k >= 3; k = k - 1) begin
			if (sf_lo[k]) begin
				f_s1_v_c = 1'b1;
				f_s1_info_c[15:0] = {src_tag[k * 8 +: 8], SRC_BLOCK[k * 8 +: 4], src_kind[k * 4 +: 4]};
				f_s1_nz_c = src_nz_w[k];
			end
			if (sf_hi[k]) begin
				f_s1_info_c[31:16] = {12'd0, src_stage[k * 4 +: 4]};
			end
		end

		f_s2_v_c = 1'b0;
		f_s2_info_c = 32'd0;
		f_s2_addr_lo_c = 16'd0;
		f_s2_addr_hi_c = 16'd0;
		f_s2_nz_c = 1'b0;
		for (k = NSRC - 1; k >= 6; k = k - 1) begin
			if (sf_lo[k]) begin
				f_s2_v_c = 1'b1;
				f_s2_info_c[15:0] = {src_tag[k * 8 +: 8], SRC_BLOCK[k * 8 +: 4], src_kind[k * 4 +: 4]};
				f_s2_nz_c = src_nz_w[k];
			end
			if (sf_alo[k]) begin
				f_s2_addr_lo_c = src_addr[k * 32 +: 16];
			end
			if (sf_hi[k]) begin
				f_s2_info_c[31:16] = {12'd0, src_stage[k * 4 +: 4]};
			end
			if (sf_ahi[k]) begin
				f_s2_addr_hi_c = src_addr[k * 32 + 16 +: 16];
			end
		end
	end

	(* DONT_TOUCH = "TRUE" *) reg [7:0] f_mc_v_r, f_host_v_r, f_s0_v_r, f_s1_v_r, f_s2_v_r;
	reg f_wdt_only_r;
	reg f_mc_nz_r, f_s0_nz_r, f_s1_nz_r, f_s2_nz_r;
	reg f_mc_tag_nz_r, f_host_tag_nz_r, f_s0_tag_nz_r, f_s1_tag_nz_r, f_s2_tag_nz_r;
	reg [31:0] f_mc_info_r, f_mc_addr_r, f_mc_addr_hi_r;
	reg [3:0] f_host_kind_r;
	reg [7:0] f_host_tag_r;
	reg [31:0] f_s0_info_r, f_s1_info_r, f_s2_info_r;
	reg [31:0] f_s0_addr_r, f_s1_addr_r, f_s2_addr_r;

	wire [31:0] f_mc_info_c = {11'd0, bus_fault_write,
		(mc_client_to_unit(bus_fault_client) == GHCI_FAULT_UNIT_SU ? {2'd0, bus_fault_req_tag[1:0]} : 4'd0),
		bus_fault_tag, mc_client_to_unit(bus_fault_client),
		bus_fault_kind};
	wire [7:0] f_host_tag_c = (kick_bad || kick_absent || locked_bad) ? 8'd0 : wdt_tag;

	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			f_mc_v_r <= 8'd0;
			f_host_v_r <= 8'd0;
			f_s0_v_r <= 8'd0;
			f_s1_v_r <= 8'd0;
			f_s2_v_r <= 8'd0;
			f_wdt_only_r <= 1'b0;
			f_mc_nz_r <= 1'b0;
			f_s0_nz_r <= 1'b0;
			f_s1_nz_r <= 1'b0;
			f_s2_nz_r <= 1'b0;
			f_mc_tag_nz_r <= 1'b0;
			f_host_tag_nz_r <= 1'b0;
			f_s0_tag_nz_r <= 1'b0;
			f_s1_tag_nz_r <= 1'b0;
			f_s2_tag_nz_r <= 1'b0;
			f_mc_info_r <= 32'd0;
			f_mc_addr_r <= 32'd0;
			f_mc_addr_hi_r <= 32'd0;
			f_host_kind_r <= 4'd0;
			f_host_tag_r <= 8'd0;
			f_s0_info_r <= 32'd0;
			f_s1_info_r <= 32'd0;
			f_s2_info_r <= 32'd0;
			f_s0_addr_r <= 32'd0;
			f_s1_addr_r <= 32'd0;
			f_s2_addr_r <= 32'd0;
		end else begin
			f_mc_v_r <= {8{bus_fault}};
			f_mc_info_r <= f_mc_info_c;
			f_mc_nz_r <= |f_mc_info_c;
			f_mc_tag_nz_r <= |f_mc_info_c[15:8];
			f_mc_addr_r <= bus_fault_addr_ext[31:0];
			f_mc_addr_hi_r <= bus_fault_addr_ext[63:32];

			f_host_v_r <= {8{kick_bad || kick_absent || locked_bad || wdt_fire}};
			f_wdt_only_r <= wdt_fire && !bus_fault && !kick_bad && !kick_absent && !locked_bad;
			f_host_kind_r <= kick_absent ? GHCI_FAULT_KIND_KICK_ABSENT :
				kick_bad ? GHCI_FAULT_KIND_INVALID_OP :
				locked_bad ? GHCI_FAULT_KIND_LOCKED_REG : GHCI_FAULT_KIND_WATCHDOG;
			f_host_tag_r <= f_host_tag_c;
			f_host_tag_nz_r <= |f_host_tag_c;

			f_s0_v_r <= {8{f_s0_v_c}};
			f_s0_info_r <= f_s0_info_c;
			f_s0_nz_r <= f_s0_nz_c;
			f_s0_tag_nz_r <= |f_s0_info_c[15:8];
			f_s0_addr_r <= {f_s0_addr_hi_c, f_s0_addr_lo_c};

			f_s1_v_r <= {8{f_s1_v_c}};
			f_s1_info_r <= f_s1_info_c;
			f_s1_nz_r <= f_s1_nz_c;
			f_s1_tag_nz_r <= |f_s1_info_c[15:8];
			f_s1_addr_r <= {f_s1_addr_hi_c, f_s1_addr_lo_c};

			f_s2_v_r <= {8{f_s2_v_c}};
			f_s2_info_r <= f_s2_info_c;
			f_s2_nz_r <= f_s2_nz_c;
			f_s2_tag_nz_r <= |f_s2_info_c[15:8];
			f_s2_addr_r <= {f_s2_addr_hi_c, f_s2_addr_lo_c};
		end
	end

	wire [31:0] f_host_info_w = {12'd0, 4'd0, f_host_tag_r, GHCI_FAULT_UNIT_CP, f_host_kind_r};
	wire n_info_nz_c = f_mc_v_r[0] ? f_mc_nz_r :
		(f_host_v_r[0] || (f_s0_v_r[0] ? f_s0_nz_r : (f_s1_v_r[0] ? f_s1_nz_r : f_s2_nz_r)));

	always @(*) begin
		any = f_mc_v_r[0] || f_host_v_r[0] || f_s0_v_r[0] || f_s1_v_r[0] || f_s2_v_r[0];
		n_info[7:0] =
			f_mc_v_r[0]   ? f_mc_info_r[7:0] :
			f_host_v_r[0] ? f_host_info_w[7:0] :
			f_s0_v_r[0]   ? f_s0_info_r[7:0] :
			f_s1_v_r[0]   ? f_s1_info_r[7:0] : f_s2_info_r[7:0];
		n_info[15:8] =
			f_mc_v_r[1]   ? f_mc_info_r[15:8] :
			f_host_v_r[1] ? f_host_info_w[15:8] :
			f_s0_v_r[1]   ? f_s0_info_r[15:8] :
			f_s1_v_r[1]   ? f_s1_info_r[15:8] : f_s2_info_r[15:8];
		n_info[23:16] =
			f_mc_v_r[2]   ? f_mc_info_r[23:16] :
			f_host_v_r[2] ? f_host_info_w[23:16] :
			f_s0_v_r[2]   ? f_s0_info_r[23:16] :
			f_s1_v_r[2]   ? f_s1_info_r[23:16] : f_s2_info_r[23:16];
		n_info[31:24] =
			f_mc_v_r[3]   ? f_mc_info_r[31:24] :
			f_host_v_r[3] ? f_host_info_w[31:24] :
			f_s0_v_r[3]   ? f_s0_info_r[31:24] :
			f_s1_v_r[3]   ? f_s1_info_r[31:24] : f_s2_info_r[31:24];
		n_addr[7:0] =
			f_mc_v_r[4]   ? f_mc_addr_r[7:0] :
			f_host_v_r[4] ? 8'd0 :
			f_s0_v_r[4]   ? f_s0_addr_r[7:0] :
			f_s1_v_r[4]   ? f_s1_addr_r[7:0] : f_s2_addr_r[7:0];
		n_addr[15:8] =
			f_mc_v_r[5]   ? f_mc_addr_r[15:8] :
			f_host_v_r[5] ? 8'd0 :
			f_s0_v_r[5]   ? f_s0_addr_r[15:8] :
			f_s1_v_r[5]   ? f_s1_addr_r[15:8] : f_s2_addr_r[15:8];
		n_addr[23:16] =
			f_mc_v_r[6]   ? f_mc_addr_r[23:16] :
			f_host_v_r[6] ? 8'd0 :
			f_s0_v_r[6]   ? f_s0_addr_r[23:16] :
			f_s1_v_r[6]   ? f_s1_addr_r[23:16] : f_s2_addr_r[23:16];
		n_addr[31:24] =
			f_mc_v_r[7]   ? f_mc_addr_r[31:24] :
			f_host_v_r[7] ? 8'd0 :
			f_s0_v_r[7]   ? f_s0_addr_r[31:24] :
			f_s1_v_r[7]   ? f_s1_addr_r[31:24] : f_s2_addr_r[31:24];
	end

	reg fault_info_nz_r;
	wire fault_info_uncleared = fault_info_nz_r;
	wire take_fault = any && can_fault_r[0];
	wire take_fault_st = any && can_fault_r[1];
	wire take_fault_info = any && can_fault_r[2] && !fault_info_uncleared;
	wire take_fault_addr = any && can_fault_r[3] && !fault_info_uncleared;
	assign ev_fault = take_fault;
	assign ev_wdt = f_wdt_only_r && take_fault;
	assign ev_halted = st == S_HALTING && ctrl_halt && halt_ok && !take_fault &&
		!wr_reset;
	wire [7:0] win_tag = n_info[15:8];
	wire win_tag_nz_c =
		f_mc_v_r[1]   ? f_mc_tag_nz_r :
		f_host_v_r[1] ? f_host_tag_nz_r :
		f_s0_v_r[1]   ? f_s0_tag_nz_r :
		f_s1_v_r[1]   ? f_s1_tag_nz_r : (f_s2_v_r[1] && f_s2_tag_nz_r);
	wire is_session_fault = ((HAS_PROT != 0) || (HAS_HW_WALKER != 0)) && win_tag_nz_c;
	reg fault_session_clear_r;
	reg [7:0] fault_session_tag_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			fault_session_clear_r <= 1'b0;
			fault_session_tag_r <= 8'd0;
		end else begin
			fault_session_clear_r <= can_fault_r[0] && is_session_fault;
			fault_session_tag_r <= win_tag;
		end
	end
	assign fault_session_clear = fault_session_clear_r;
	assign fault_session_tag = fault_session_tag_r;

	reg wdt_run_r, wdt_step_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			wdt_run_r <= 1'b0;
			wdt_step_r <= 1'b0;
		end else begin
			wdt_run_r <= ctrl_wdt_en && cmd_active && !faulted;
			wdt_step_r <= ctrl_wdt_en && cmd_active && !faulted && !wdt_hold;
		end
	end

	reg mc_idle_stuck_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			mc_idle_stuck_r <= 1'b1;
		else
			mc_idle_stuck_r <= mc_idle;
	end

	wire wr_wdt = hcr_we && (hcr_widx == I_WDT);
	wire wr_heap_b = hcr_we && open_r[1] && (hcr_widx == I_HEAP_BASE);
	wire wr_heap_l = hcr_we && open_r[2] && (hcr_widx == I_HEAP_LIMIT);
	reg stuck_hit_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int)
			stuck_hit_r <= 1'b0;
		else
			stuck_hit_r <= !fn_cmp_lt32(stuck_count[33:2], wdt_cycles);
	end
	wire stuck_en = halt_int && !mc_idle_stuck_r && !stuck_hit_r;
	wire wdt_cnt_clr = !wdt_run_r;
	wire wdt_cnt_en = wdt_step_r;

`ifdef SYNTH_KINTEX7
	wire [5:0] tf_buf;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf0 (.I0(take_fault_info), .O(tf_buf[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf1 (.I0(take_fault_info), .O(tf_buf[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf2 (.I0(take_fault_info), .O(tf_buf[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf3 (.I0(take_fault_info), .O(tf_buf[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf4 (.I0(take_fault_addr), .O(tf_buf[4]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_tf5 (.I0(take_fault_addr), .O(tf_buf[5]));

	(* keep = "true", dont_touch = "true" *) wire [5:0] fault_rst_b;
	wire rs_pulse_c = (st == S_RS_PULSE);
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst0 (.I0(rs_pulse_c), .O(fault_rst_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst1 (.I0(rs_pulse_c), .O(fault_rst_b[1]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst2 (.I0(rs_pulse_c), .O(fault_rst_b[2]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst3 (.I0(rs_pulse_c), .O(fault_rst_b[3]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst4 (.I0(rs_pulse_c), .O(fault_rst_b[4]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_frst5 (.I0(rs_pulse_c), .O(fault_rst_b[5]));

	(* keep = "true", dont_touch = "true" *) wire [1:0] wdt_cnt_rst_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wcr0 (.I0(wdt_cnt_clr), .O(wdt_cnt_rst_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wcr1 (.I0(wdt_cnt_clr), .O(wdt_cnt_rst_b[1]));

	(* keep = "true", dont_touch = "true" *) wire [1:0] stuck_cnt_rst_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hB)) u_scr0 (.I0(mc_idle_stuck_r), .I1(halt_int), .O(stuck_cnt_rst_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hB)) u_scr1 (.I0(mc_idle_stuck_r), .I1(halt_int), .O(stuck_cnt_rst_b[1]));

	wire [1:0] wr_fi_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wfi0 (.I0(wr_fault_info), .O(wr_fi_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wfi1 (.I0(wr_fault_info), .O(wr_fi_b[1]));

	wire [1:0] wr_wdt_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ww0 (.I0(wr_wdt), .O(wr_wdt_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_ww1 (.I0(wr_wdt), .O(wr_wdt_b[1]));

	wire [1:0] wr_heap_b_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_whpb0 (.I0(wr_heap_b), .O(wr_heap_b_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_whpb1 (.I0(wr_heap_b), .O(wr_heap_b_b[1]));

	wire [1:0] wr_heap_l_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_whpl0 (.I0(wr_heap_l), .O(wr_heap_l_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_whpl1 (.I0(wr_heap_l), .O(wr_heap_l_b[1]));

	wire [1:0] stuck_en_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_stuck0 (.I0(stuck_en), .O(stuck_en_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_stuck1 (.I0(stuck_en), .O(stuck_en_b[1]));

	wire [1:0] wdt_cnt_en_b;
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wce0 (.I0(wdt_cnt_en), .O(wdt_cnt_en_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT1 #(.INIT(2'h2)) u_wce1 (.I0(wdt_cnt_en), .O(wdt_cnt_en_b[1]));
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [5:0] tf_buf;
	assign tf_buf[0] = take_fault_info;
	assign tf_buf[1] = take_fault_info;
	assign tf_buf[2] = take_fault_info;
	assign tf_buf[3] = take_fault_info;
	assign tf_buf[4] = take_fault_addr;
	assign tf_buf[5] = take_fault_addr;

	(* keep = "true", dont_touch = "true" *) wire [5:0] fault_rst_b;
	wire rs_pulse_c = (st == S_RS_PULSE);
	assign fault_rst_b[0] = rs_pulse_c;
	assign fault_rst_b[1] = rs_pulse_c;
	assign fault_rst_b[2] = rs_pulse_c;
	assign fault_rst_b[3] = rs_pulse_c;
	assign fault_rst_b[4] = rs_pulse_c;
	assign fault_rst_b[5] = rs_pulse_c;

	(* keep = "true", dont_touch = "true" *) wire [1:0] wdt_cnt_rst_b;
	assign wdt_cnt_rst_b[0] = wdt_cnt_clr;
	assign wdt_cnt_rst_b[1] = wdt_cnt_clr;

	(* keep = "true", dont_touch = "true" *) wire [1:0] stuck_cnt_rst_b;
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hB)) u_scr0 (.I0(mc_idle_stuck_r), .I1(halt_int), .O(stuck_cnt_rst_b[0]));
	(* DONT_TOUCH = "TRUE" *) LUT2 #(.INIT(4'hB)) u_scr1 (.I0(mc_idle_stuck_r), .I1(halt_int), .O(stuck_cnt_rst_b[1]));

	wire [1:0] wr_fi_b;
	assign wr_fi_b[0] = wr_fault_info;
	assign wr_fi_b[1] = wr_fault_info;

	wire [1:0] wr_wdt_b;
	assign wr_wdt_b[0] = wr_wdt;
	assign wr_wdt_b[1] = wr_wdt;

	wire [1:0] wr_heap_b_b;
	assign wr_heap_b_b[0] = wr_heap_b;
	assign wr_heap_b_b[1] = wr_heap_b;

	wire [1:0] wr_heap_l_b;
	assign wr_heap_l_b[0] = wr_heap_l;
	assign wr_heap_l_b[1] = wr_heap_l;

	wire [1:0] stuck_en_b;
	assign stuck_en_b[0] = stuck_en;
	assign stuck_en_b[1] = stuck_en;

	wire [1:0] wdt_cnt_en_b;
	assign wdt_cnt_en_b[0] = wdt_cnt_en;
	assign wdt_cnt_en_b[1] = wdt_cnt_en;
`else
	wire [5:0] tf_buf = {{2{take_fault_addr}}, {4{take_fault_info}}};
	wire [5:0] fault_rst_b = {6{(st == S_RS_PULSE)}};
	wire [1:0] wdt_cnt_rst_b = {2{wdt_cnt_clr}};
	wire [1:0] stuck_cnt_rst_b = {2{!halt_int || mc_idle_stuck_r}};
	wire [1:0] wr_fi_b = {2{wr_fault_info}};
	wire [1:0] wr_wdt_b = {2{wr_wdt}};
	wire [1:0] wr_heap_b_b = {2{wr_heap_b}};
	wire [1:0] wr_heap_l_b = {2{wr_heap_l}};
	wire [1:0] stuck_en_b = {2{stuck_en}};
	wire [1:0] wdt_cnt_en_b = {2{wdt_cnt_en}};
	wire unused_k7_ctl = &{1'b0, k[0]};
`endif

	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			fault_info  <= 32'd0;
			fault_info_nz_r <= 1'b0;
			fault_addr  <= 32'd0;
			fault_addr_hi <= 32'd0;
			wdt_count   <= 32'd0;
			stuck_count <= 34'd0;
		end else begin
			if (fault_rst_b[0])
				fault_info_nz_r <= 1'b0;
			else if (tf_buf[0])
				fault_info_nz_r <= n_info_nz_c;
			else if (wr_fi_b[0])
				fault_info_nz_r <= |(fault_info & ~(hcr_wdata & hcr_wmask));

			if (fault_rst_b[0]) fault_info[7:0] <= 8'd0;
			else if (tf_buf[0]) fault_info[7:0] <= n_info[7:0];
			else if (wr_fi_b[0]) fault_info[7:0] <= fault_info[7:0] & ~(hcr_wdata[7:0] & hcr_wmask[7:0]);

			if (fault_rst_b[1]) fault_info[15:8] <= 8'd0;
			else if (tf_buf[1]) fault_info[15:8] <= n_info[15:8];
			else if (wr_fi_b[0]) fault_info[15:8] <= fault_info[15:8] & ~(hcr_wdata[15:8] & hcr_wmask[15:8]);

			if (fault_rst_b[2]) fault_info[23:16] <= 8'd0;
			else if (tf_buf[2]) fault_info[23:16] <= n_info[23:16];
			else if (wr_fi_b[1]) fault_info[23:16] <= fault_info[23:16] & ~(hcr_wdata[23:16] & hcr_wmask[23:16]);

			if (fault_rst_b[3]) fault_info[31:24] <= 8'd0;
			else if (tf_buf[3]) fault_info[31:24] <= n_info[31:24];
			else if (wr_fi_b[1]) fault_info[31:24] <= fault_info[31:24] & ~(hcr_wdata[31:24] & hcr_wmask[31:24]);

			if (fault_rst_b[4]) fault_addr[15:0] <= 16'd0;
			else if (tf_buf[4]) fault_addr[15:0] <= n_addr[15:0];

			if (fault_rst_b[5]) fault_addr[31:16] <= 16'd0;
			else if (tf_buf[5]) fault_addr[31:16] <= n_addr[31:16];

			if (fault_rst_b[5]) fault_addr_hi <= 32'd0;
			else if (tf_buf[5]) fault_addr_hi <= f_mc_v_r[7] ? f_mc_addr_hi_r : 32'd0;

			if (wdt_cnt_rst_b[0]) wdt_count[15:0] <= 16'd0;
			else if (wdt_cnt_en_b[0]) wdt_count[15:0] <= wdt_count_n[15:0];

			if (wdt_cnt_rst_b[1]) wdt_count[31:16] <= 16'd0;
			else if (wdt_cnt_en_b[1]) wdt_count[31:16] <= wdt_count_n[31:16];

			if (stuck_cnt_rst_b[0]) stuck_count[16:0] <= 17'd0;
			else if (stuck_en_b[0]) stuck_count[16:0] <= stuck_count_n[16:0];

			if (stuck_cnt_rst_b[1]) stuck_count[33:17] <= 17'd0;
			else if (stuck_en_b[1]) stuck_count[33:17] <= stuck_count_n[33:17];
		end
	end

	wire ctl_halt_done  = (lc_halting_r && ctrl_halt && mc_idle) && !ring_seq_busy_w;

	reg [2:0] st_nr;
	reg lc_halting_nr, lc_abort_nr, st_is_run_nr;
	reg [3:0] open_nr, can_fault_nr, mcf_rep_nr;
	reg [2:0] halt_nr;
	reg crst_n_nr;
	always @(*) begin
		st_nr         = st;
		lc_halting_nr = lc_halting_r;
		lc_abort_nr   = lc_abort_r;
		open_nr       = open_r;
		can_fault_nr  = can_fault_r;
		halt_nr       = halt_r;
		st_is_run_nr  = st_is_run_r;
		mcf_rep_nr    = 4'd0;
		crst_n_nr     = 1'b1;
		if (take_fault_st && !is_session_fault) begin
			st_nr         = S_FAULTED;
			lc_halting_nr = 1'b0;
			lc_abort_nr   = 1'b1;
			open_nr       = 4'b0000;
			can_fault_nr  = 4'b0000;
			halt_nr       = 3'b111;
			st_is_run_nr  = 1'b0;
		end else if (ctl_halt_done) begin
			st_nr         = S_HALTED;
			lc_halting_nr = 1'b0;
			lc_abort_nr   = 1'b0;
			open_nr       = 4'b1111;
			halt_nr       = 3'b111;
			st_is_run_nr  = 1'b0;
		end else if (lc_halting_r) begin
			if (!ctrl_halt) begin
				st_nr         = ctrl_run ? S_RUN : S_IDLE;
				lc_halting_nr = 1'b0;
				lc_abort_nr   = 1'b0;
				open_nr       = {4{!ctrl_run}};
				halt_nr       = 3'b000;
				st_is_run_nr  = ctrl_run;
			end else begin
				open_nr       = 4'b0000;
				can_fault_nr  = 4'b1111;
				halt_nr       = 3'b111;
				st_is_run_nr  = 1'b0;
			end
		end else begin
			case (st)
			S_IDLE:
				if (ctrl_halt) begin
					st_nr         = S_HALTING;
					lc_halting_nr = 1'b1;
					lc_abort_nr   = 1'b0;
					open_nr       = 4'b0000;
					halt_nr       = 3'b111;
					st_is_run_nr  = 1'b0;
				end else if (ctrl_run) begin
					st_nr         = S_RUN;
					lc_halting_nr = 1'b0;
					lc_abort_nr   = 1'b0;
					open_nr       = 4'b0000;
					halt_nr       = 3'b000;
					st_is_run_nr  = 1'b1;
				end else begin
					open_nr       = 4'b1111;
					can_fault_nr  = 4'b1111;
					halt_nr       = 3'b000;
					st_is_run_nr  = 1'b0;
				end
			S_RUN:
				if (ctrl_halt) begin
					st_nr         = S_HALTING;
					lc_halting_nr = 1'b1;
					lc_abort_nr   = 1'b0;
					open_nr       = 4'b0000;
					halt_nr       = 3'b111;
					st_is_run_nr  = 1'b0;
				end else if (!ctrl_run && !busy_in[0]) begin
					st_nr         = S_IDLE;
					lc_halting_nr = 1'b0;
					lc_abort_nr   = 1'b0;
					open_nr       = 4'b1111;
					halt_nr       = 3'b000;
					st_is_run_nr  = 1'b0;
				end else begin
					open_nr       = 4'b0000;
					can_fault_nr  = 4'b1111;
					halt_nr       = 3'b000;
					st_is_run_nr  = 1'b1;
				end
			S_HALTED:
				if (!ctrl_halt) begin
					st_nr         = ctrl_run ? S_RUN : S_IDLE;
					lc_halting_nr = 1'b0;
					lc_abort_nr   = 1'b0;
					open_nr       = {4{!ctrl_run}};
					halt_nr       = 3'b000;
					st_is_run_nr  = ctrl_run;
				end else begin
					open_nr       = 4'b1111;
					can_fault_nr  = 4'b1111;
					halt_nr       = 3'b111;
					st_is_run_nr  = 1'b0;
				end
			S_RS_WAIT: begin
				open_nr       = 4'b0000;
				can_fault_nr  = 4'b0000;
				halt_nr       = 3'b111;
				st_is_run_nr  = 1'b0;
				if (mc_idle) begin
					st_nr         = S_RS_PULSE;
					lc_halting_nr = 1'b0;
					lc_abort_nr   = 1'b1;
					mcf_rep_nr    = 4'hF;
					crst_n_nr     = 1'b0;
				end
			end
			S_RS_PULSE: begin
				open_nr       = 4'b0000;
				can_fault_nr  = 4'b0000;
				halt_nr       = 3'b111;
				st_is_run_nr  = 1'b0;
				st_nr         = S_RS_REPLAY;
				lc_halting_nr = 1'b0;
				lc_abort_nr   = 1'b1;
			end
			S_RS_REPLAY: begin
				if (!replay_busy) begin
					st_nr         = S_IDLE;
					lc_halting_nr = 1'b0;
					lc_abort_nr   = 1'b0;
					open_nr       = 4'b1111;
					can_fault_nr  = 4'b1111;
					halt_nr       = 3'b000;
					st_is_run_nr  = 1'b0;
				end else begin
					open_nr       = 4'b0000;
					can_fault_nr  = 4'b0000;
					halt_nr       = 3'b111;
					st_is_run_nr  = 1'b0;
				end
			end
			default: begin
				open_nr       = 4'b0000;
				can_fault_nr  = 4'b0000;
				halt_nr       = 3'b111;
				st_is_run_nr  = 1'b0;
			end
			endcase
		end
	end

	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			st <= S_IDLE;
			lc_halting_r <= 1'b0;
			lc_halted_r  <= 1'b0;
			lc_abort_r   <= 1'b0;
			open_r <= 4'b1111;
			can_fault_r <= 4'b1111;
			halt_r <= 3'b000;
			st_is_run_r <= 1'b0;
			st_mcf_rep <= 4'd0;
			st_crst_n_r <= 1'b1;
			wdt_cycles <= WDT_RESET;
			ap_base <= APERTURE_BASE_INIT;
			ap_limit <= APERTURE_LIMIT_INIT;
			heap_base <= 32'd0;
			heap_limit <= 32'd0;
		end else begin
			/* Registers. */
			if (wr_wdt_b[0]) wdt_cycles[15:0] <= (wdt_cycles[15:0] & ~hcr_wmask[15:0]) | (hcr_wdata[15:0] & hcr_wmask[15:0]);
			if (wr_wdt_b[1]) wdt_cycles[31:16] <= (wdt_cycles[31:16] & ~hcr_wmask[31:16]) | (hcr_wdata[31:16] & hcr_wmask[31:16]);
			if (wr_heap_b_b[0]) heap_base[15:0] <= (heap_base[15:0] & ~hcr_wmask[15:0]) | (hcr_wdata[15:0] & hcr_wmask[15:0]);
			if (wr_heap_b_b[1]) heap_base[31:16] <= (heap_base[31:16] & ~hcr_wmask[31:16]) | (hcr_wdata[31:16] & hcr_wmask[31:16]);
			if (wr_heap_l_b[0]) heap_limit[15:0] <= (heap_limit[15:0] & ~hcr_wmask[15:0]) | (hcr_wdata[15:0] & hcr_wmask[15:0]);
			if (wr_heap_l_b[1]) heap_limit[31:16] <= (heap_limit[31:16] & ~hcr_wmask[31:16]) | (hcr_wdata[31:16] & hcr_wmask[31:16]);

			/* The state (wr_reset_go sits at the final 2:1 mux). */
			st           <= wr_reset_go ? S_RS_WAIT : st_nr;
			lc_halting_r <= wr_reset_go ? 1'b0      : lc_halting_nr;
			lc_abort_r   <= wr_reset_go ? 1'b1      : lc_abort_nr;
			open_r       <= wr_reset_go ? 4'b0000   : open_nr;
			can_fault_r  <= wr_reset_go ? 4'b0000   : can_fault_nr;
			halt_r       <= wr_reset_go ? 3'b111    : halt_nr;
			st_is_run_r  <= wr_reset_go ? 1'b0      : st_is_run_nr;
			st_mcf_rep   <= wr_reset_go ? 4'd0      : mcf_rep_nr;
			st_crst_n_r  <= wr_reset_go ? 1'b1      : crst_n_nr;
			begin : g_halted_next
				reg ctl_no_ab_w;
				reg halt_stay_w;
				reg halt_early_w;
				reg halt_c0_n;
				reg halt_c1_n;
				reg s32_z_w;
				reg s10_z_w;
				ctl_no_ab_w  = !wr_reset_go && !(take_fault_st && !is_session_fault);
				halt_stay_w  = ctrl_halt && (st == S_HALTED);
				halt_early_w = lc_halting_r && ctrl_halt && mc_idle;
				halt_c0_n = !(ctl_no_ab_w && (halt_stay_w || (halt_early_w && !ring_busy)));
				s32_z_w   = !(seq_st[3] | seq_st[2]);
				s10_z_w   = !(seq_st[1] | seq_st[0]);
				halt_c1_n = !((ctl_no_ab_w && halt_early_w && !f_busy) && s32_z_w && s10_z_w);
				lc_halted_r <= !(halt_c0_n & halt_c1_n);
			end
		end
	end

	wire ctl_rs_pulse_clr = (st == S_RS_PULSE) && !(take_fault_st && !is_session_fault);
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			ctrl_run    <= 1'b0;
			ctrl_halt   <= 1'b0;
			ctrl_wdt_en <= 1'b0;
			irq_global  <= 1'b0;
		end else if (ctl_rs_pulse_clr) begin
			ctrl_run    <= 1'b0;
			ctrl_halt   <= 1'b0;
			ctrl_wdt_en <= 1'b0;
			irq_global  <= 1'b0;
		end else if (wr_ctrl) begin
			if (hcr_wdata[GHCI_CTRL_RESET]) begin
				ctrl_run    <= 1'b0;
				ctrl_halt   <= 1'b0;
				ctrl_wdt_en <= 1'b0;
				irq_global  <= 1'b0;
			end else begin
				if (hcr_wmask[GHCI_CTRL_RUN])        ctrl_run    <= hcr_wdata[GHCI_CTRL_RUN];
				if (hcr_wmask[GHCI_CTRL_HALT])       ctrl_halt   <= hcr_wdata[GHCI_CTRL_HALT];
				if (hcr_wmask[GHCI_CTRL_WDT_EN])     ctrl_wdt_en <= hcr_wdata[GHCI_CTRL_WDT_EN];
				if (hcr_wmask[GHCI_CTRL_IRQ_GLOBAL]) irq_global  <= hcr_wdata[GHCI_CTRL_IRQ_GLOBAL];
			end
		end
	end

	wire axi_stuck_clr_w = (st == S_RS_PULSE) && !(wr_reset && !resetting) && !(take_fault_st && !is_session_fault);
	wire axi_stuck_ge_w  = fn_cmp_ge32(stuck_count[33:2], wdt_cycles);
	reg axi_stuck_set_r;
	always @(posedge clk or posedge rst_ctl_int) begin
		if (rst_ctl_int) begin
			axi_stuck       <= 1'b0;
			axi_stuck_set_r <= 1'b0;
		end else begin
			axi_stuck_set_r <= halt_int && !mc_idle_stuck_r && axi_stuck_ge_w;
			if (axi_stuck_clr_w)
				axi_stuck <= 1'b0;
			else if (axi_stuck_set_r)
				axi_stuck <= 1'b1;
		end
	end

	/* Reads. */
	wire [31:0] status = {12'd0, cp_state, 1'b0, quiesce, axi_stuck, mem_ready, ring_empty,
		2'd0, mc_idle, 5'd0, faulted, st == S_HALTED, run && !halt};

`ifdef SYNTH_KINTEX7
	wire [1:0] ctl_lo_ok, sel_ctrl, sel_finfo, sel_faddr, sel_faddh, sel_wdt, sel_hbase, sel_hlim, sel_status;
	genvar g_cs;
	generate
		for (g_cs = 0; g_cs < 2; g_cs = g_cs + 1) begin : g_ctl_sel
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_clok (.I0(hcr_ridx[6]), .I1(hcr_ridx[7]), .I2(hcr_ridx[8]), .I3(hcr_ridx[9]), .O(ctl_lo_ok[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_CTRL[5:0]))          u_sctrl (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_ctrl[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_INFO[5:0]))    u_sfinfo(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_finfo[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_ADDR[5:0]))    u_sfaddr(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_faddr[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_ADDR_HI[5:0])) u_sfaddh(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_faddh[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_WDT[5:0]))           u_swdt  (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_wdt[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_HEAP_BASE[5:0]))     u_shbase(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_hbase[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_HEAP_LIMIT[5:0]))    u_shlim (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_hlim[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_STATUS[5:0]))        u_sstat (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_status[g_cs]));
		end
	endgenerate

	wire [31:0] ctrl_word = {25'd0, irq_global, ctrl_wdt_en, 1'b0, 1'b0, ctrl_halt, ctrl_run, resetting};
	wire [31:0] rd_p0 = {
		({16{sel_ctrl[1]}}  & ctrl_word[31:16])  | ({16{sel_finfo[1]}} & fault_info[31:16]) | ({16{sel_faddr[1]}} & fault_addr[31:16]),
		({16{sel_ctrl[0]}}  & ctrl_word[15:0])   | ({16{sel_finfo[0]}} & fault_info[15:0])  | ({16{sel_faddr[0]}} & fault_addr[15:0])
	};
	wire [31:0] rd_p1 = {
		({16{sel_faddh[1]}} & fault_addr_hi[31:16]) | ({16{sel_wdt[1]}} & wdt_cycles[31:16]) | ({16{sel_hbase[1]}} & heap_base[31:16]),
		({16{sel_faddh[0]}} & fault_addr_hi[15:0])  | ({16{sel_wdt[0]}} & wdt_cycles[15:0])  | ({16{sel_hbase[0]}} & heap_base[15:0])
	};
	wire [31:0] rd_p2 = {
		({16{sel_hlim[1]}}  & heap_limit[31:16]) | ({16{sel_status[1]}} & status[31:16]),
		({16{sel_hlim[0]}}  & heap_limit[15:0])  | ({16{sel_status[0]}} & status[15:0])
	};

	assign hcr_rdata = {
		ctl_lo_ok[1] ? (rd_p0[31:16] | rd_p1[31:16] | rd_p2[31:16]) : 16'd0,
		ctl_lo_ok[0] ? (rd_p0[15:0]  | rd_p1[15:0]  | rd_p2[15:0])  : 16'd0
	};
	wire [31:0] unused_ctl_rd_unused = 32'd0;
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [1:0] ctl_lo_ok, sel_ctrl, sel_finfo, sel_faddr, sel_faddh, sel_wdt, sel_hbase, sel_hlim, sel_status;
	genvar g_cs;
	generate
		for (g_cs = 0; g_cs < 2; g_cs = g_cs + 1) begin : g_ctl_sel
			(* DONT_TOUCH = "TRUE" *) LUT4 #(.INIT(16'h0001)) u_clok (.I0(hcr_ridx[6]), .I1(hcr_ridx[7]), .I2(hcr_ridx[8]), .I3(hcr_ridx[9]), .O(ctl_lo_ok[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_CTRL[5:0]))          u_sctrl (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_ctrl[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_INFO[5:0]))    u_sfinfo(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_finfo[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_ADDR[5:0]))    u_sfaddr(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_faddr[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_FAULT_ADDR_HI[5:0])) u_sfaddh(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_faddh[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_WDT[5:0]))           u_swdt  (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_wdt[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_HEAP_BASE[5:0]))     u_shbase(.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_hbase[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_HEAP_LIMIT[5:0]))    u_shlim (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_hlim[g_cs]));
			(* DONT_TOUCH = "TRUE" *) LUT6 #(.INIT(64'h0000000000000001 << I_STATUS[5:0]))        u_sstat (.I0(hcr_ridx[0]), .I1(hcr_ridx[1]), .I2(hcr_ridx[2]), .I3(hcr_ridx[3]), .I4(hcr_ridx[4]), .I5(hcr_ridx[5]), .O(sel_status[g_cs]));
		end
	endgenerate

	wire [31:0] ctrl_word = {25'd0, irq_global, ctrl_wdt_en, 1'b0, 1'b0, ctrl_halt, ctrl_run, resetting};
	wire [31:0] rd_p0 = {
		({16{sel_ctrl[1]}}  & ctrl_word[31:16])  | ({16{sel_finfo[1]}} & fault_info[31:16]) | ({16{sel_faddr[1]}} & fault_addr[31:16]),
		({16{sel_ctrl[0]}}  & ctrl_word[15:0])   | ({16{sel_finfo[0]}} & fault_info[15:0])  | ({16{sel_faddr[0]}} & fault_addr[15:0])
	};
	wire [31:0] rd_p1 = {
		({16{sel_faddh[1]}} & fault_addr_hi[31:16]) | ({16{sel_wdt[1]}} & wdt_cycles[31:16]) | ({16{sel_hbase[1]}} & heap_base[31:16]),
		({16{sel_faddh[0]}} & fault_addr_hi[15:0])  | ({16{sel_wdt[0]}} & wdt_cycles[15:0])  | ({16{sel_hbase[0]}} & heap_base[15:0])
	};
	wire [31:0] rd_p2 = {
		({16{sel_hlim[1]}}  & heap_limit[31:16]) | ({16{sel_status[1]}} & status[31:16]),
		({16{sel_hlim[0]}}  & heap_limit[15:0])  | ({16{sel_status[0]}} & status[15:0])
	};

	assign hcr_rdata = {
		ctl_lo_ok[1] ? (rd_p0[31:16] | rd_p1[31:16] | rd_p2[31:16]) : 16'd0,
		ctl_lo_ok[0] ? (rd_p0[15:0]  | rd_p1[15:0]  | rd_p2[15:0])  : 16'd0
	};
	wire [31:0] unused_ctl_rd_unused = 32'd0;
`else
	reg [31:0] rd;

	always @(*) begin
		case (hcr_ridx)
		I_CTRL:          rd = {25'd0, irq_global, ctrl_wdt_en, 1'b0, 1'b0, ctrl_halt, ctrl_run, resetting};
		I_FAULT_INFO:    rd = fault_info;
		I_FAULT_ADDR:    rd = fault_addr;
		I_FAULT_ADDR_HI: rd = fault_addr_hi;
		I_WDT:           rd = wdt_cycles;
		I_HEAP_BASE:     rd = heap_base;
		I_HEAP_LIMIT:    rd = heap_limit;
		default:         rd = 32'd0;
		endcase
	end

	assign hcr_rdata = (hcr_ridx == I_STATUS) ? status : rd;
`endif
	wire unused_co = &{1'b0, wdt_co, stuck_co, hcr_ridx[9:6]};
endmodule

`default_nettype wire
