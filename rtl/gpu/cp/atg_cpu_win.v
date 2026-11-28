/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_cpu_win: The CPU's data-bus decoder and control window: routes
 * loads/stores to DMEM or to the window registers (class V packet,
 * UOP_FIFO, STATE read, FENCE_LAST, cycle counter, RESULT, interrupt), and
 * the sub-bus to atg_cpu_dma and atg_cpu_fw.
 *
 * Every access is taken in one cycle and answered (d_ready) the next; a
 * UOP/UOP_LAST write waits there while the FIFO is full (stall = 1).
 * Decode of d_addr: 0x0002_xxxx DMEM (m_* in the cycle it is taken, the
 * load data is m_rdata in the answer cycle); 0xF000_00xx the window, word
 * index d_addr[7:2] (whole words; d_be is ignored), acted on in the
 * answer cycle from registered copies: 0x00-0x0F this module's registers,
 * 0x10-0x3F the sub-bus (one-cycle sb_we/sb_re, read data in that cycle);
 * anything else answers with 0 and drops a store.
 *
 * Class V packet: PKT_INFO[31] = v_valid until DONE is written (then 0
 * until v_valid falls, also while DONE waits for the FIFO); reading
 * PKT_INFO with [31] = 1 takes the packet (its interrupt cause goes
 * away).  DONE pulses v_done once the UOP_FIFO is empty.  fw_active is
 * v_valid until v_done: the sequencer takes µops only while a packet is
 * open, the packet's last ones too.  irq = |(IRQ & IRQ_EN): [0] packet waiting and not taken,
 * [1] fence event (sticky, W1C), [2] halt_req.
 *
 * Session protection: taking the packet (PKT_INFO) latches its slot
 * (v_slot, the CP's SLOT_MAP of v_tag) into pkt_slot, which v_done clears;
 * a write of UOP_SELF (sub-bus 0x32, [0]) marks the next µop packet SELF
 * (the firmware's own wait) until its UOP_LAST.  Each UOP_FIFO entry keeps
 * {SELF, pkt_slot} of its write, given with the FIFO's head on fw_side:
 * a µop written outside a packet has slot 0.
 * See spec/gpu/cp/atg_cpu_win.md.
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_cpu_win #(
	parameter UOP_DEPTH_LOG2 = 4	/* UOP_FIFO depth log2 (16 dwords) */
) (
	input wire clk,
	input wire rst_n,
	/* Core data port */
	input wire d_valid,
	input wire d_write,
	input wire [31:0] d_addr,
	input wire [31:0] d_wdata,
	input wire [3:0] d_be,
	output wire d_ready,
	output reg [31:0] d_rdata,
	/* DMEM data port (atg_cpu_mem) */
	output wire m_en,
	output wire [3:0] m_we,
	output wire [31:0] m_addr,
	output wire [31:0] m_wdata,
	input wire [31:0] m_rdata,
	/* Sub-bus to atg_cpu_dma and atg_cpu_fw */
	output wire sb_we,
	output wire sb_re,
	output wire [5:0] sb_idx,
	output wire [31:0] sb_wdata,
	input wire [31:0] dma_rdata,
	input wire [31:0] fw_rdata,
	/* Class V packet (from atg_cp_ring) */
	input wire v_valid,
	input wire [31:0] v_addr,
	input wire [15:0] v_dwords,
	input wire [7:0] v_tag,
	output reg v_done,
	/* µops to atg_cp_seq */
	output wire fw_valid,
	input wire fw_ready,
	output wire [31:0] fw_data,
	output wire fw_last,
	output wire fw_active,
	output wire [6:0] fw_side,	/* the head's {SELF, slot} */
	/* Session protection */
	input wire [5:0] v_slot,	/* SLOT_MAP[v_tag] */
	output reg [5:0] pkt_slot,	/* the taken packet's slot (0: none) */
	/* Other */
	output wire [9:0] st_ridx,
	input wire [31:0] st_rdata,
	input wire [31:0] fence_last,
	input wire fence_ev,
	input wire halt_req,
	output wire irq,
	output reg ev_result,
	output wire stall
);
	/* Window word indices. */
	localparam [5:0] W_PKT_ADDR = 6'h00;
	localparam [5:0] W_PKT_DWORDS = 6'h01;
	localparam [5:0] W_PKT_INFO = 6'h02;
	localparam [5:0] W_DONE = 6'h03;
	localparam [5:0] W_UOP = 6'h04;
	localparam [5:0] W_UOP_LAST = 6'h05;
	localparam [5:0] W_UOP_SELF = 6'h32;	/* sub-bus, also seen here */
	localparam [5:0] W_UOP_FREE = 6'h06;
	localparam [5:0] W_STATE_IDX = 6'h07;
	localparam [5:0] W_STATE_DATA = 6'h08;
	localparam [5:0] W_FENCE_LAST = 6'h09;
	localparam [5:0] W_CYCLE = 6'h0A;
	localparam [5:0] W_RESULT = 6'h0B;
	localparam [5:0] W_IRQ = 6'h0C;
	localparam [5:0] W_IRQ_EN = 6'h0D;

	localparam [UOP_DEPTH_LOG2:0] DEPTH = 1 << UOP_DEPTH_LOG2;

	/* Access FSM: IDLE takes the access (DMEM is accessed there, so its
	 * data is ready a cycle later) and registers its address and data; ACK
	 * does the window's work from those registers and answers (d_ready),
	 * or waits while a UOP write finds the FIFO full.  Nothing in the
	 * answer path depends on d_addr in the same cycle (timing). */
	localparam [2:0] S_IDLE  = 3'd0;
	localparam [2:0] S_DMEM  = 3'd1;
	localparam [2:0] S_ACK   = 3'd2;
	localparam [2:0] S_SUB   = 3'd3;
	localparam [2:0] S_SUB2  = 3'd4;

	reg [2:0] state;
	reg is_idle_r, ack_r, sub_cap_r, sub_rd_r, sb_we_r, sb_re_r;
	reg [13:0] own_sel_r;
	reg q_dmem, q_own, q_sub, q_write;
	reg [5:0] q_idx;
	(* keep = "true" *) reg [5:0] sb_idx_r;
	reg [31:0] q_wdata;
	reg [31:0] q_addr;
	reg [3:0]  q_be;
	reg [31:0] q_rdata;
	reg [31:0] st_rdata_r;

	wire in_dmem = d_addr[31:16] == 16'h0002;
	wire in_win  = (d_addr[31:16] == 16'hF000) && (d_addr[15:8] == 8'h00);
	wire [5:0] idx = d_addr[7:2];
	wire take = d_valid && is_idle_r;
	wire ack = ack_r;
	wire sub_rd = sub_rd_r;

	/* Registers. */
	reg [9:0] st_idx;
	reg [31:0] cycle;
	wire [31:0] cycle_n;	/* cycle + 1 */
	wire cycle_co;

	/* 4x8b carry-select incrementer for 32-bit cycle counter (avoids 32-bit ripple). */
	wire [7:0] cy_s0, cy_s1_0, cy_s1_1, cy_s2_0, cy_s2_1, cy_s3_0, cy_s3_1;
	wire cy_c0, cy_c1_0, cy_c1_1, cy_c2_0, cy_c2_1, cy_c3_0, cy_c3_1;
	am_add #(.W(8)) cy_a0   (.a(cycle[7:0]),   .b(8'd0), .ci(1'b1), .s(cy_s0),   .co(cy_c0));
	am_add #(.W(8)) cy_a1_0 (.a(cycle[15:8]),  .b(8'd0), .ci(1'b0), .s(cy_s1_0), .co(cy_c1_0));
	am_add #(.W(8)) cy_a1_1 (.a(cycle[15:8]),  .b(8'd0), .ci(1'b1), .s(cy_s1_1), .co(cy_c1_1));
	am_add #(.W(8)) cy_a2_0 (.a(cycle[23:16]), .b(8'd0), .ci(1'b0), .s(cy_s2_0), .co(cy_c2_0));
	am_add #(.W(8)) cy_a2_1 (.a(cycle[23:16]), .b(8'd0), .ci(1'b1), .s(cy_s2_1), .co(cy_c2_1));
	am_add #(.W(8)) cy_a3_0 (.a(cycle[31:24]), .b(8'd0), .ci(1'b0), .s(cy_s3_0), .co(cy_c3_0));
	am_add #(.W(8)) cy_a3_1 (.a(cycle[31:24]), .b(8'd0), .ci(1'b1), .s(cy_s3_1), .co(cy_c3_1));
	wire cy_c1 = cy_c0 ? cy_c1_1 : cy_c1_0;
	wire cy_c2 = cy_c2_0 | (cy_c2_1 & cy_c1_0) | (cy_c2_1 & cy_c1_1 & cy_c0);
	assign cycle_n = {
		cy_c2 ? cy_s3_1 : cy_s3_0,
		cy_c1 ? cy_s2_1 : cy_s2_0,
		cy_c0 ? cy_s1_1 : cy_s1_0,
		cy_s0
	};
	assign cycle_co = cy_c2 ? cy_c3_1 : cy_c3_0;
	reg [2:0] irq_en;
	reg fence_pend;
	reg taken;	/* PKT_INFO read while the packet waited */
	reg done_pend;	/* DONE written, waiting for the FIFO to drain */
	reg done_sent;	/* v_done given, v_valid not yet fallen */

	/* The packet is open until v_done; the firmware sees it until it writes DONE. */
	wire pkt = v_valid && !done_sent;
	wire pkt_info = pkt && !done_pend;
	wire [2:0] cause = {halt_req, fence_pend, pkt && !taken};

	wire own_wr = ack && q_own && q_write;
	wire own_rd = sub_rd && q_own && !q_write;

	reg q_uop_r, q_uop_last_r;
	reg m_en_r;
	reg [3:0] m_we_r;
	reg [31:0] sub_rdata_r;
	wire f_valid;
	wire f_ready;
	wire [UOP_DEPTH_LOG2:0] f_count;

	reg self_next;
	reg uop_full_r;
	reg [UOP_DEPTH_LOG2:0] uop_free_r;
	localparam [UOP_DEPTH_LOG2:0] DEPTH_M1 = DEPTH - 1;
	localparam UDEPTH = 1 << UOP_DEPTH_LOG2;

	/* Discrete FF-based UOP FIFO (eliminates Check 4 $mem_v2 & rp->uop_fifo deep paths). */
	reg [UOP_DEPTH_LOG2-1:0] uop_wp, uop_rp;
	reg [UOP_DEPTH_LOG2:0] uop_cnt;
	reg uop_valid_r;
	wire uop_push_c = q_uop_r && !uop_full_r;
	wire uop_pop_c  = uop_valid_r && fw_ready;
	assign f_valid = uop_valid_r;
	assign f_ready = !uop_full_r;
	assign f_count = uop_cnt;

	wire [UOP_DEPTH_LOG2-1:0] uop_wp_n, uop_rp_n;
	wire [UOP_DEPTH_LOG2:0] uop_cnt_dec, uop_cnt_inc;
	wire unused_uwp_co, unused_urp_co, unused_ucd_bo, unused_uci_co;
	am_add #(.W(UOP_DEPTH_LOG2)) u_uwp_inc (
		.a(uop_wp), .b({UOP_DEPTH_LOG2{1'b0}}), .ci(1'b1), .s(uop_wp_n), .co(unused_uwp_co)
	);
	am_add #(.W(UOP_DEPTH_LOG2)) u_urp_inc (
		.a(uop_rp), .b({UOP_DEPTH_LOG2{1'b0}}), .ci(1'b1), .s(uop_rp_n), .co(unused_urp_co)
	);
	am_sub #(.W(UOP_DEPTH_LOG2 + 1)) u_uc_dec (
		.a(uop_cnt), .b({{UOP_DEPTH_LOG2{1'b0}}, 1'b1}), .bi(1'b0), .d(uop_cnt_dec), .bo(unused_ucd_bo)
	);
	am_add #(.W(UOP_DEPTH_LOG2 + 1)) u_uc_inc (
		.a(uop_cnt), .b({{UOP_DEPTH_LOG2{1'b0}}, 1'b0}), .ci(1'b1), .s(uop_cnt_inc), .co(unused_uci_co)
	);
	wire uop_valid_n = uop_push_c ? 1'b1 :
		(uop_pop_c ? (uop_cnt != {{UOP_DEPTH_LOG2{1'b0}}, 1'b1}) : uop_valid_r);

	wire [UDEPTH*40-1:0] uop_flat;
	genvar gu;
	generate
		for (gu = 0; gu < UDEPTH; gu = gu + 1) begin : g_uop
			reg [39:0] entry;
			always @(posedge clk) begin
				if (uop_push_c && (uop_wp == gu[UOP_DEPTH_LOG2-1:0]))
					entry <= {self_next, pkt_slot, q_uop_last_r, q_wdata};
			end
			assign uop_flat[gu*40 +: 40] = entry;
		end
	endgenerate
	reg [39:0] uop_head_c;
	integer uo;
	always @(*) begin
		uop_head_c = 40'd0;
		for (uo = 0; uo < UDEPTH; uo = uo + 1)
			if (uop_rp == uo[UOP_DEPTH_LOG2-1:0])
				uop_head_c = uop_flat[uo*40 +: 40];
	end
	assign {fw_side, fw_last, fw_data} = uop_head_c;

	wire uop_full_n = uop_full_r ? !uop_pop_c : (uop_push_c && !uop_pop_c && (uop_free_r == {{UOP_DEPTH_LOG2{1'b0}}, 1'b1}));
	wire [UOP_DEPTH_LOG2:0] uop_free_dec, uop_free_inc;
	wire unused_ufd_bo, unused_ufi_co, unused_f_count_ok;
	assign unused_f_count_ok = &{1'b0, f_count, unused_uwp_co, unused_urp_co, unused_ucd_bo, unused_uci_co};
	am_sub #(.W(UOP_DEPTH_LOG2 + 1)) u_uf_dec (
		.a(uop_free_r), .b({{UOP_DEPTH_LOG2{1'b0}}, 1'b1}), .bi(1'b0), .d(uop_free_dec), .bo(unused_ufd_bo)
	);
	am_add #(.W(UOP_DEPTH_LOG2 + 1)) u_uf_inc (
		.a(uop_free_r), .b({{UOP_DEPTH_LOG2{1'b0}}, 1'b0}), .ci(1'b1), .s(uop_free_inc), .co(unused_ufi_co)
	);

	assign fw_valid = f_valid;
	assign fw_active = pkt;
	assign stall = ack_r && q_uop_r && uop_full_r;
	wire unused_f_ready = f_ready;
	assign irq = |(cause & irq_en);
	assign st_ridx = st_idx;
	assign d_ready = ack_r && !(q_uop_r && uop_full_r);

	/* DMEM: registered enable/write-enable from take for Check 4 <= 1 depth. */
	assign m_en = m_en_r;
	assign m_we = m_we_r;
	assign m_addr = {16'd0, q_addr[15:0]};
	assign m_wdata = q_wdata;

	/* Sub-bus: registered sb_we_r, sb_re_r, and dedicated sb_idx_r for low fanout. */
	assign sb_we = sb_we_r;
	assign sb_re = sb_re_r;
	assign sb_idx = sb_idx_r;
	assign sb_wdata = q_wdata;

	wire [31:0] own_t0 = ({32{own_sel_r[0]}}  & v_addr) | ({32{own_sel_r[1]}}  & {16'd0, v_dwords});
	wire [31:0] own_t1 = ({32{own_sel_r[2]}}  & {pkt_info, 23'd0, v_tag}) |
		({32{own_sel_r[6]}}  & {{(31 - UOP_DEPTH_LOG2){1'b0}}, uop_free_r});
	wire [31:0] own_t2 = ({32{own_sel_r[7]}}  & {22'd0, st_idx}) | ({32{own_sel_r[8]}}  & st_rdata_r);
	wire [31:0] own_t3 = ({32{own_sel_r[9]}}  & fence_last)      | ({32{own_sel_r[10]}} & cycle);
	wire [31:0] own_t4 = ({32{own_sel_r[12]}} & {29'd0, cause})   | ({32{own_sel_r[13]}} & {29'd0, irq_en});
	wire [31:0] own_rdata_c = ((own_t0 | own_t1) | (own_t2 | own_t3)) | own_t4;

	always @(*) begin
		if (q_write)
			d_rdata = 32'd0;
		else if (q_dmem)
			d_rdata = m_rdata;
		else
			d_rdata = q_rdata;
	end

	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			q_idx <= 6'd0;
			sb_idx_r <= 6'd0;
			q_wdata <= 32'd0;
			q_write <= 1'b0;
			q_addr <= 32'd0;
			q_be <= 4'd0;
		end else if (take) begin
			q_idx <= idx;
			sb_idx_r <= idx;
			q_wdata <= d_wdata;
			q_write <= d_write;
			if (in_dmem)
				q_addr <= d_addr;
			q_be <= d_be;
		end
	end

	integer oi;
	always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			state <= S_IDLE;
			is_idle_r <= 1'b1;
			ack_r <= 1'b0;
			sub_cap_r <= 1'b0;
			sub_rd_r <= 1'b0;
			sb_we_r <= 1'b0;
			sb_re_r <= 1'b0;
			own_sel_r <= 14'd0;
			q_dmem <= 1'b0;
			q_own <= 1'b0;
			q_sub <= 1'b0;
			q_uop_r <= 1'b0;
			q_uop_last_r <= 1'b0;
			m_en_r <= 1'b0;
			m_we_r <= 4'd0;
			sub_rdata_r <= 32'd0;
			q_rdata <= 32'd0;
			st_rdata_r <= 32'd0;
			st_idx <= 10'd0;
			cycle <= 32'd0;
			irq_en <= 3'd0;
			fence_pend <= 1'b0;
			taken <= 1'b0;
			done_pend <= 1'b0;
			done_sent <= 1'b0;
			v_done <= 1'b0;
			ev_result <= 1'b0;
			pkt_slot <= 6'd0;
			self_next <= 1'b0;
			uop_full_r <= 1'b0;
			uop_free_r <= DEPTH;
			uop_wp <= {UOP_DEPTH_LOG2{1'b0}};
			uop_rp <= {UOP_DEPTH_LOG2{1'b0}};
			uop_cnt <= {(UOP_DEPTH_LOG2 + 1){1'b0}};
			uop_valid_r <= 1'b0;
		end else begin
			uop_full_r <= uop_full_n;
			uop_valid_r <= uop_valid_n;
			if (uop_push_c)
				uop_wp <= uop_wp_n;
			if (uop_pop_c)
				uop_rp <= uop_rp_n;
			if (uop_push_c && !uop_pop_c)
				uop_cnt <= uop_cnt_inc;
			else if (!uop_push_c && uop_pop_c)
				uop_cnt <= uop_cnt_dec;
			if (uop_push_c && !uop_pop_c)
				uop_free_r <= uop_free_dec;
			else if (!uop_push_c && uop_pop_c)
				uop_free_r <= uop_free_inc;
			cycle <= cycle_n;
			st_rdata_r <= st_rdata;
			v_done <= 1'b0;
			ev_result <= 1'b0;
			m_en_r <= 1'b0;
			m_we_r <= 4'd0;
			sb_we_r <= 1'b0;
			sb_re_r <= 1'b0;
			if (take) begin
				state <= in_dmem ? S_DMEM : (!d_write && in_win ? S_SUB : S_ACK);
				is_idle_r <= 1'b0;
				ack_r <= !in_dmem && !(!d_write && in_win);
				sub_cap_r <= !in_dmem && !d_write && in_win;
				sub_rd_r <= 1'b0;
				sb_we_r <= !in_dmem && d_write && in_win && (idx[5] || idx[4]);
				sb_re_r <= !in_dmem && !d_write && in_win && (idx[5] || idx[4]);
				for (oi = 0; oi < 14; oi = oi + 1)
					own_sel_r[oi] <= in_win && !idx[5] && !idx[4] && (idx[3:0] == oi[3:0]);
				q_dmem <= in_dmem;
				m_en_r <= in_dmem;
				m_we_r <= (in_dmem && d_write) ? d_be : 4'd0;
				q_own <= in_win && !idx[5] && !idx[4];
				q_sub <= in_win && (idx[5] || idx[4]);
				q_uop_r <= in_win && d_write && (idx[5:1] == 5'b00010);
				q_uop_last_r <= idx[0];
				if (!in_dmem && !in_win)
					q_rdata <= 32'd0;
			end else if (state == S_DMEM) begin
				state <= S_ACK;
				ack_r <= 1'b1;
			end else if (state == S_SUB) begin
				if (q_sub)
					sub_rdata_r <= dma_rdata | fw_rdata;
				state <= S_SUB2;
				sub_cap_r <= 1'b0;
				sub_rd_r <= 1'b1;
			end else if (sub_rd) begin
				q_rdata <= q_sub ? sub_rdata_r : own_rdata_c;
				state <= S_ACK;
				sub_rd_r <= 1'b0;
				ack_r <= 1'b1;
			end else if (ack && !stall) begin
				q_uop_r <= 1'b0;
				state <= S_IDLE;
				is_idle_r <= 1'b1;
				ack_r <= 1'b0;
			end
			if (ack && q_write && own_sel_r[7])
				st_idx <= q_wdata[9:0];
			if (ack && q_write && own_sel_r[13])
				irq_en <= q_wdata[2:0];
			if (ack && q_write && own_sel_r[11])
				ev_result <= 1'b1;
			if (fence_ev)
				fence_pend <= 1'b1;
			else if (ack && q_write && own_sel_r[12] && q_wdata[1])
				fence_pend <= 1'b0;
			if (sub_rd && !q_write && own_sel_r[2] && pkt_info) begin
				taken <= 1'b1;
				pkt_slot <= v_slot;
			end
			if (sb_we && q_idx == W_UOP_SELF)
				self_next <= q_wdata[0];
			else if (ack && q_uop_r && f_ready && q_uop_last_r)
				self_next <= 1'b0;
			if (ack && q_write && own_sel_r[3] && pkt)
				done_pend <= 1'b1;
			if (done_pend && !f_valid) begin
				v_done <= 1'b1;
				done_pend <= 1'b0;
				taken <= 1'b0;
				done_sent <= 1'b1;
				pkt_slot <= 6'd0;
			end
			if (!v_valid)
				done_sent <= 1'b0;
		end
	end

	wire unused_ok = &{1'b0, d_addr[1:0]};
	wire unused_cycle = &{1'b0, cycle_co};
endmodule

`default_nettype wire
