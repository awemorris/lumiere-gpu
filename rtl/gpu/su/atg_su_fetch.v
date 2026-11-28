/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_fetch: Instruction fetch: takes a (warp, pc, address) from the
 * scheduler, reads the 64-bit instruction from the I$ and hands it to the
 * decoder.
 *
 * The behaviour, and where it differs from
 * the design: spec/gpu/su/atg_su_fetch.md (§10 Status).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_fetch #(
	parameter WARP_W = 2	/* log2(NWARPS) */
) (
	input wire clk,
	input wire rst_n,
	/* From atg_su_sched */
	input wire f_valid,
	output wire f_ready,
	input wire [WARP_W-1:0] f_warp,
	input wire [15:0] f_pc,
	input wire [31:0] f_addr,
	/* I$ port (atg_su_icache) */
	output wire ic_req,
	output wire [WARP_W-1:0] ic_warp,
	input wire ic_ready,
	output wire [31:0] ic_addr,
	input wire ic_rvalid,
	input wire [63:0] ic_rdata,
	/* To atg_su_dec */
	output wire d_valid,
	input wire d_ready,
	output wire [WARP_W-1:0] d_warp,
	output wire [15:0] d_pc,
	output wire [63:0] d_instr
);
	/* Four in-flight fetches in order (depth 4, pointer mod 8):
	 * {warp, pc, addr} buffered at request, issued to I$, instruction
	 * written on ic_rvalid; the head leaves when it has its data.
	 * Force slice registers to prevent Distributed RAM inference. */
	(* ram_style = "registers" *) reg [WARP_W-1:0] q_warp [0:3];
	(* ram_style = "registers" *) reg [15:0] q_pc [0:3];
	(* ram_style = "registers" *) reg [31:0] q_addr [0:3];
	(* max_fanout = 12 *) reg [3:0] q_have;
	(* keep = "true", max_fanout = 12 *) reg [2:0] rp;
	(* keep = "true", max_fanout = 12 *) reg [2:0] wp, fp;	/* request, read and fill pointers (mod 8, index bits [1:0]) */
	(* keep = "true", max_fanout = 12 *) reg [2:0] ip;	/* icache issue pointer */
	(* keep = "true", equivalent_register_removal = "no" *) reg [1:0] hip_addr_lo, hip_addr_hi;
	(* shreg_extract = "no" *) reg [31:0] cur_ic_addr_r;
	(* shreg_extract = "no" *) reg [WARP_W-1:0] cur_ic_warp_r;
	wire [2:0] used;
	wire [2:0] wp_next;
	wire [2:0] fp_next;
	wire [2:0] rp_next;
	wire [2:0] ip_next;
	wire us_bo_unused, wp_co_unused, fp_co_unused, rp_co_unused, ip_co_unused;
	am_sub #(.W(3)) u_used (.a(wp), .b(rp), .bi(1'b0), .d(used), .bo(us_bo_unused));
	am_add #(.W(3)) u_wp (.a(wp), .b(3'd0), .ci(1'b1), .s(wp_next), .co(wp_co_unused));
	am_add #(.W(3)) u_fp (.a(fp), .b(3'd0), .ci(1'b1), .s(fp_next), .co(fp_co_unused));
	am_add #(.W(3)) u_rp (.a(rp), .b(3'd0), .ci(1'b1), .s(rp_next), .co(rp_co_unused));
	am_add #(.W(3)) u_ip (.a(ip), .b(3'd0), .ci(1'b1), .s(ip_next), .co(ip_co_unused));

	wire full = (wp[1:0] == rp[1:0]) && (wp[2] != rp[2]);
	wire room = !full;

	/* Decouple f_ready from ic_ready: f_ready is asserted whenever queue has room */
	wire f_ready_w = room;
	assign f_ready = f_ready_w;
	wire push_w = f_valid && f_ready_w;
	wire [3:0] push_ctl, push_pc_ce, push_alo_ce, push_ahi_ce;
	genvar gpf;
	generate
		for (gpf = 0; gpf < 4; gpf = gpf + 1) begin : g_push_buf
			wire push_lane = push_w && (wp[1:0] == gpf[1:0]);
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pctl (.I0(push_lane), .O(push_ctl[gpf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ppc  (.I0(push_lane), .O(push_pc_ce[gpf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_palo (.I0(push_lane), .O(push_alo_ce[gpf]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pahi (.I0(push_lane), .O(push_ahi_ce[gpf]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign push_ctl[gpf] = push_lane;
			assign push_pc_ce[gpf] = push_lane;
			assign push_alo_ce[gpf] = push_lane;
			assign push_ahi_ce[gpf] = push_lane;
`else
			assign push_ctl[gpf]    = push_lane;
			assign push_pc_ce[gpf]  = push_lane;
			assign push_alo_ce[gpf] = push_lane;
			assign push_ahi_ce[gpf] = push_lane;
`endif
		end
	endgenerate
	wire push_0 = push_ctl[0];
	wire push_1 = push_ctl[1];
	wire push_2 = push_ctl[2];
	wire push_3 = push_ctl[3];

	/* I$ issue interface: driven from registered q_addr/q_warp and ic_req_r */
	reg ic_req_r;
	wire ic_req_w = ic_req_r;
	assign ic_req = ic_req_r;
	wire [1:0] hip = ip[1:0];
	assign ic_warp = cur_ic_warp_r;
	assign ic_addr = cur_ic_addr_r;
	wire ip_advance = ic_req_r && ic_ready;
	wire neq_00 = (wp != ip);
	wire neq_10 = (wp_next != ip);
	wire neq_01 = (wp != ip_next);
	wire row0_ic = push_w ? neq_10 : neq_00;
	wire row1_ic = push_w ? neq_00 : neq_01;
	wire ic_req_nxt = ip_advance ? row1_ic : row0_ic;
	wire has_after_adv = (ip_next != wp);
	wire has_after_adv_0, has_after_adv_1;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_haa0 (.I0(has_after_adv), .O(has_after_adv_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_haa1 (.I0(has_after_adv), .O(has_after_adv_1));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign has_after_adv_0 = has_after_adv;
	assign has_after_adv_1 = has_after_adv;
`else
	assign has_after_adv_0 = has_after_adv;
	assign has_after_adv_1 = has_after_adv;
`endif
	wire cur_we_idle = !ic_req_r && push_w;
	wire cur_we_idle_0, cur_we_idle_1, cur_we_idle_w;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cwi0 (.I0(cur_we_idle), .O(cur_we_idle_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cwi1 (.I0(cur_we_idle), .O(cur_we_idle_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cwiw (.I0(cur_we_idle), .O(cur_we_idle_w));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cur_we_idle_0 = cur_we_idle;
	assign cur_we_idle_1 = cur_we_idle;
	assign cur_we_idle_w = cur_we_idle;
`else
	assign cur_we_idle_0 = cur_we_idle;
	assign cur_we_idle_1 = cur_we_idle;
	assign cur_we_idle_w = cur_we_idle;
`endif
	wire cur_we_adv  = has_after_adv || push_w;
	wire [31:0] cur_addr_adv = {
		has_after_adv_1 ? q_addr[ip_next[1:0]][31:16] : f_addr[31:16],
		has_after_adv_0 ? q_addr[ip_next[1:0]][15:0]  : f_addr[15:0]
	};
	wire [WARP_W-1:0] cur_warp_adv = has_after_adv ? q_warp[ip_next[1:0]] : f_warp;
	wire unused_hip_dup = &{1'b0, hip, hip_addr_lo, hip_addr_hi};

	(* dont_touch = "true" *) wire fill_0 = ic_rvalid && (fp[1:0] == 2'd0);
	(* dont_touch = "true" *) wire fill_1 = ic_rvalid && (fp[1:0] == 2'd1);
	(* dont_touch = "true" *) wire fill_2 = ic_rvalid && (fp[1:0] == 2'd2);
	(* dont_touch = "true" *) wire fill_3 = ic_rvalid && (fp[1:0] == 2'd3);

	wire rst_p = !rst_n;
	wire [1:0] h = rp[1:0];
	reg d_valid_r;
	assign d_valid = d_valid_r;
	assign d_warp = q_warp[h];
	assign d_pc = q_pc[h];
	(* keep = "true", max_fanout = 12 *) wire pop = d_valid_r && d_ready;
	wire unused_used_h = &{1'b0, used};

	wire [2:0] wp_nxt = push_w ? wp_next : wp;
	wire [2:0] rp_nxt = pop    ? rp_next : rp;
	wire [3:0] q_have_nxt = {
		fill_3 ? 1'b1 : (push_3 ? 1'b0 : q_have[3]),
		fill_2 ? 1'b1 : (push_2 ? 1'b0 : q_have[2]),
		fill_1 ? 1'b1 : (push_1 ? 1'b0 : q_have[1]),
		fill_0 ? 1'b1 : (push_0 ? 1'b0 : q_have[0])
	};
	wire q_have_rp_nxt  = q_have_nxt[rp[1:0]];
	wire q_have_rp1_nxt = q_have_nxt[rp_next[1:0]];
	wire d_valid_nxt = (wp_nxt != rp_nxt) && (pop ? q_have_rp1_nxt : q_have_rp_nxt);

	genvar gqs;
	generate
		for (gqs = 0; gqs < 8; gqs = gqs + 1) begin : g_qins_sl
			(* dont_touch = "true" *) reg [1:0] fp_sl;
			(* dont_touch = "true" *) reg [1:0] rp_sl;
			wire fill0_sl = ic_rvalid && (fp_sl == 2'd0);
			wire fill1_sl = ic_rvalid && (fp_sl == 2'd1);
			wire fill2_sl = ic_rvalid && (fp_sl == 2'd2);
			wire fill3_sl = ic_rvalid && (fp_sl == 2'd3);
			(* shreg_extract = "no" *) reg [7:0] q_ins0_r, q_ins1_r, q_ins2_r, q_ins3_r;
			always @(posedge clk) begin
				if (fill0_sl)
					q_ins0_r <= ic_rdata[gqs*8 +: 8];
				if (fill1_sl)
					q_ins1_r <= ic_rdata[gqs*8 +: 8];
				if (fill2_sl)
					q_ins2_r <= ic_rdata[gqs*8 +: 8];
				if (fill3_sl)
					q_ins3_r <= ic_rdata[gqs*8 +: 8];
			end
			always @(posedge clk or posedge rst_p) begin
				if (rst_p) begin
					fp_sl <= 2'd0;
					rp_sl <= 2'd0;
				end else begin
					if (ic_rvalid)
						fp_sl <= fp_next[1:0];
					if (pop)
						rp_sl <= rp_next[1:0];
				end
			end
			assign d_instr[gqs*8 +: 8] = (rp_sl == 2'd3) ? q_ins3_r :
			                             (rp_sl == 2'd2) ? q_ins2_r :
			                             (rp_sl == 2'd1) ? q_ins1_r : q_ins0_r;
		end
	endgenerate

	always @(posedge clk) begin
		if (push_0) q_warp[0] <= f_warp;
		if (push_pc_ce[0])  q_pc[0]          <= f_pc;
		if (push_alo_ce[0]) q_addr[0][15:0]  <= f_addr[15:0];
		if (push_ahi_ce[0]) q_addr[0][31:16] <= f_addr[31:16];
		if (push_1) q_warp[1] <= f_warp;
		if (push_pc_ce[1])  q_pc[1]          <= f_pc;
		if (push_alo_ce[1]) q_addr[1][15:0]  <= f_addr[15:0];
		if (push_ahi_ce[1]) q_addr[1][31:16] <= f_addr[31:16];
		if (push_2) q_warp[2] <= f_warp;
		if (push_pc_ce[2])  q_pc[2]          <= f_pc;
		if (push_alo_ce[2]) q_addr[2][15:0]  <= f_addr[15:0];
		if (push_ahi_ce[2]) q_addr[2][31:16] <= f_addr[31:16];
		if (push_3) q_warp[3] <= f_warp;
		if (push_pc_ce[3])  q_pc[3]          <= f_pc;
		if (push_alo_ce[3]) q_addr[3][15:0]  <= f_addr[15:0];
		if (push_ahi_ce[3]) q_addr[3][31:16] <= f_addr[31:16];
	end

	(* keep = "true" *) wire adv_en_pre = ic_req_w && cur_we_adv;
	(* keep = "true" *) wire cur_addr_ce = cur_we_idle || (adv_en_pre && ic_ready);
	wire cur_addr_ce_0, cur_addr_ce_1, cur_addr_ce_2, cur_addr_ce_3;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cace0 (.I0(cur_addr_ce), .O(cur_addr_ce_0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cace1 (.I0(cur_addr_ce), .O(cur_addr_ce_1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cace2 (.I0(cur_addr_ce), .O(cur_addr_ce_2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cace3 (.I0(cur_addr_ce), .O(cur_addr_ce_3));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign cur_addr_ce_0 = cur_addr_ce;
	assign cur_addr_ce_1 = cur_addr_ce;
	assign cur_addr_ce_2 = cur_addr_ce;
	assign cur_addr_ce_3 = cur_addr_ce;
`else
	assign cur_addr_ce_0 = cur_addr_ce;
	assign cur_addr_ce_1 = cur_addr_ce;
	assign cur_addr_ce_2 = cur_addr_ce;
	assign cur_addr_ce_3 = cur_addr_ce;
`endif
	(* keep = "true" *) wire [31:0] cur_ic_addr_nxt = {
		cur_we_idle_1 ? f_addr[31:16] : cur_addr_adv[31:16],
		cur_we_idle_0 ? f_addr[15:0]  : cur_addr_adv[15:0]
	};
	(* keep = "true" *) wire [WARP_W-1:0] cur_ic_warp_nxt = cur_we_idle_w ? f_warp : cur_warp_adv;

	always @(posedge clk or posedge rst_p) begin
		if (rst_p) begin
			wp <= 3'd0;
			ip <= 3'd0;
			hip_addr_lo <= 2'd0;
			hip_addr_hi <= 2'd0;
			cur_ic_addr_r <= 32'd0;
			cur_ic_warp_r <= {WARP_W{1'b0}};
			rp <= 3'd0;
			fp <= 3'd0;
			q_have <= 4'b0000;
			d_valid_r <= 1'b0;
			ic_req_r <= 1'b0;
		end else begin
			d_valid_r <= d_valid_nxt;
			ic_req_r <= ic_req_nxt;
			if (push_w)
				wp <= wp_next;
			if (ip_advance) begin
				ip <= ip_next;
				hip_addr_lo <= ip_next[1:0];
				hip_addr_hi <= ip_next[1:0];
			end
			if (cur_addr_ce_0) cur_ic_addr_r[11:0]  <= cur_ic_addr_nxt[11:0];
			if (cur_addr_ce_1) cur_ic_addr_r[23:12] <= cur_ic_addr_nxt[23:12];
			if (cur_addr_ce_2) cur_ic_addr_r[31:24] <= cur_ic_addr_nxt[31:24];
			if (cur_addr_ce_3) cur_ic_warp_r        <= cur_ic_warp_nxt;
			if (fill_0)
				q_have[0] <= 1'b1;
			else if (push_0)
				q_have[0] <= 1'b0;
			if (fill_1)
				q_have[1] <= 1'b1;
			else if (push_1)
				q_have[1] <= 1'b0;
			if (fill_2)
				q_have[2] <= 1'b1;
			else if (push_2)
				q_have[2] <= 1'b0;
			if (fill_3)
				q_have[3] <= 1'b1;
			else if (push_3)
				q_have[3] <= 1'b0;
			if (ic_rvalid)
				fp <= fp_next;
			if (pop)
				rp <= rp_next;
		end
	end
endmodule

`default_nettype wire
