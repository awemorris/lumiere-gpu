/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_rx_fdiv: fp32 divider a / b, 1 quotient bit per cycle (fanout <= 16).
 * RNE, denormals flushed, canonical NaN 0x7FC00000.
 * Under !SYNTH_KINTEX7 (ASIC): SRT Radix-2 carry-save divider (<= 11 cmos3).
 */

`timescale 1ns / 1ps
`default_nettype none

(* keep_hierarchy = "yes" *)
module atg_rx_fdiv (
	input wire clk,
	input wire rst_n,
	input wire i_start,
	input wire [31:0] i_a,
	input wire [31:0] i_b,
	output wire busy,
	output reg o_done,
	output reg [31:0] o_r
);
	wire rst_sync = !rst_n;

	wire [7:0] ea = i_a[30:23];
	wire [7:0] eb = i_b[30:23];
	wire ea_ff_lo = &ea[5:0];
	wire ea_z_lo  = (ea[5:0] == 6'd0);
	wire eb_ff_lo = &eb[5:0];
	wire eb_z_lo  = (eb[5:0] == 6'd0);
	wire za = (ea[7:6] == 2'b00) && ea_z_lo;
	wire zb = (eb[7:6] == 2'b00) && eb_z_lo;
	wire ea_ff = (ea[7:6] == 2'b11) && ea_ff_lo;
	wire eb_ff = (eb[7:6] == 2'b11) && eb_ff_lo;
	wire sp_a = za || ea_ff;
	wire sp_b = zb || eb_ff;

	wire ma_nz_0 = (i_a[5:0]   != 6'd0);
	wire ma_nz_1 = (i_a[11:6]  != 6'd0);
	wire ma_nz_2 = (i_a[17:12] != 6'd0);
	wire ma_nz_3 = (i_a[22:18] != 5'd0);
	(* keep = "true" *) wire ma_nz = ma_nz_0 || ma_nz_1 || ma_nz_2 || ma_nz_3;

	wire mb_nz_0 = (i_b[5:0]   != 6'd0);
	wire mb_nz_1 = (i_b[11:6]  != 6'd0);
	wire mb_nz_2 = (i_b[17:12] != 6'd0);
	wire mb_nz_3 = (i_b[22:18] != 5'd0);
	(* keep = "true" *) wire mb_nz = mb_nz_0 || mb_nz_1 || mb_nz_2 || mb_nz_3;

	wire s = i_a[31] ^ i_b[31];
	wire [23:0] ma = {1'b1, i_a[22:0]};
	wire [23:0] mb = {1'b1, i_b[22:0]};

	(* keep = "true", dont_touch = "true" *) reg prep_ctrl_a, prep_ctrl_b, prep_ctrl_c;
	(* keep = "true", dont_touch = "true" *) reg run_ctrl_a, run_ctrl_b, run_ctrl_c, run_q0, run_q1, run_r0, run_r1;
	reg r_last;
	(* keep = "true", dont_touch = "true" *) reg round_s1_ctrl, round_s1_a, round_s1_b, round_s1_c;
	(* keep = "true", dont_touch = "true" *) reg round_s2_ctrl, round_s2_a, round_s2_b;
	reg sgn;
	reg r_sp_any, r_sp_nan, r_sp_exp_ff;
	(* keep = "true", dont_touch = "true" *) reg r_ma_lt_mb_a, r_ma_lt_mb_b;
	(* extract_reset = "no" *) reg [9:0] r_eab;
	(* extract_reset = "no" *) reg [23:0] r_ma;
	(* keep = "true", dont_touch = "true" *) reg r_round_up_a, r_round_up_b;
	(* extract_reset = "no" *) reg [24:0] r_m0_inc;
	(* extract_reset = "no" *) reg [7:0] r_ef_0, r_ef_1;
	(* keep = "true", dont_touch = "true" *) reg r_mz_0_a, r_mz_0_b;
	(* keep = "true", dont_touch = "true" *) reg r_mz_1_a, r_mz_1_b;
	reg [4:0] cnt;
	reg [9:0] e;
	(* extract_reset = "no" *) reg [23:0] d_lo, d_hi;
	(* extract_reset = "no" *) reg [24:0] q_reg;

	assign busy = prep_ctrl_a || run_ctrl_a || round_s1_ctrl || round_s2_ctrl;

	/* ma < mb via am_sub borrow-out */
	wire [22:0] ma_mb_unused;
	wire ma_lt_mb_raw;
	am_sub #(.W(23)) u_ma_lt_mb (.a(i_a[22:0]), .b(i_b[22:0]), .bi(1'b0), .d(ma_mb_unused), .bo(ma_lt_mb_raw));

	/* Exponent difference */
	wire [9:0] eab, e_n;
	wire [4:0] cnt_m1;
	wire eab_bo_unused, en_co_unused, cm_bo_unused;
	am_sub #(.W(10)) u_eab (.a({2'b00, ea}), .b({2'b00, eb}), .bi(1'b0), .d(eab), .bo(eab_bo_unused));
	am_add #(.W(10)) u_e_n (.a(r_eab), .b(10'd126), .ci(!r_ma_lt_mb_a), .s(e_n), .co(en_co_unused));
	am_sub #(.W(5)) u_cnt (.a(cnt), .b(5'd0), .bi(1'b1), .d(cnt_m1), .bo(cm_bo_unused));

	(* keep = "true" *) wire [12:0] rem_init_lo = r_ma_lt_mb_a ? {r_ma[11:0], 1'b0} : r_ma[12:0];
	(* keep = "true" *) wire [24:13] rem_init_hi = r_ma_lt_mb_b ? r_ma[23:12] : {1'b0, r_ma[23:13]};

	/* round_s1 common arithmetic */
	wire [24:0] m0_inc_w;
	wire [9:0] e_p1_w;
	wire m0_co_unused, ep1_co_unused;
	am_add #(.W(25)) u_m0 (.a({1'b0, q_reg[24:1]}), .b(25'd1), .ci(1'b0), .s(m0_inc_w), .co(m0_co_unused));
	am_add #(.W(10)) u_ef (.a(e), .b(10'd1), .ci(1'b0), .s(e_p1_w), .co(ep1_co_unused));

	(* keep = "true" *) wire ovf_0_w = !e[9] && (e[8] || (&e[7:0]));
	(* keep = "true" *) wire udf_0_w = e[9] || (e[8:0] == 9'd0);
	(* keep = "true" *) wire ovf_1_w = !e[9] && (e[8] || (&e[7:1]));
	(* keep = "true" *) wire udf_1_w = e[9] && (e != 10'h3FF);

	wire [7:0] ef_0_w = ovf_0_w ? 8'hFF : (udf_0_w ? 8'h00 : e[7:0]);
	wire [7:0] ef_1_w = ovf_1_w ? 8'hFF : (udf_1_w ? 8'h00 : e_p1_w[7:0]);
	wire mz_0_w = ovf_0_w || udf_0_w;
	wire mz_1_w = ovf_1_w || udf_1_w;

	(* keep = "true" *) wire sp_nan = (ea_ff && (ma_nz || eb_ff)) || (eb_ff && mb_nz) || (za && zb);
	(* keep = "true" *) wire sp_exp_ff = ea_ff || zb || (eb_ff && mb_nz);
	wire sp_any = sp_a || sp_b;
	(* keep = "true" *) wire start_ok_a = i_start && !prep_ctrl_a && !run_ctrl_a && !round_s1_ctrl && !round_s2_ctrl;
	(* keep = "true" *) wire start_ok_b = i_start && !prep_ctrl_b && !run_ctrl_b && !round_s1_ctrl && !round_s2_ctrl;

	wire start_d0, start_d1, start_d2, start_d3;
	wire start_m0, start_m1, start_eab;
`ifdef SYNTH_KINTEX7
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sd0 (.I0(start_ok_a), .O(start_d0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sd1 (.I0(start_ok_a), .O(start_d1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sd2 (.I0(start_ok_b), .O(start_d2));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sd3 (.I0(start_ok_b), .O(start_d3));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sm0 (.I0(start_ok_a), .O(start_m0));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_sm1 (.I0(start_ok_b), .O(start_m1));
	(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_seab (.I0(start_ok_b), .O(start_eab));
`elsif SYNTH_KINTEX_ULTRASCALE
	assign start_d0 = start_ok_a;
	assign start_d1 = start_ok_a;
	assign start_d2 = start_ok_b;
	assign start_d3 = start_ok_b;
	assign start_m0 = start_ok_a;
	assign start_m1 = start_ok_b;
	assign start_eab = start_ok_b;
`else
	assign start_d0 = start_ok_a;
	assign start_d1 = start_ok_a;
	assign start_d2 = start_ok_b;
	assign start_d3 = start_ok_b;
	assign start_m0 = start_ok_a;
	assign start_m1 = start_ok_b;
	assign start_eab = start_ok_b;
`endif

	(* keep = "true" *) wire prep_sp_a = prep_ctrl_a && r_sp_any;
	(* keep = "true" *) wire prep_sp_b = prep_ctrl_b && r_sp_any;

	(* keep = "true" *) wire ce_q0 = run_q0 || prep_ctrl_a;
	(* keep = "true" *) wire ce_q1 = run_q1 || prep_ctrl_b;
	(* keep = "true" *) wire ce_r0 = run_r0 || prep_ctrl_a;
	(* keep = "true" *) wire ce_r1 = run_r1 || prep_ctrl_b;

	wire [31:0] sp_or_val = {!r_sp_nan && sgn, {8{r_sp_exp_ff}}, r_sp_nan, 22'd0};

`ifdef SYNTH_KINTEX7
	(* extract_reset = "no" *) reg [25:0] rem;
	wire [25:0] rem_d_lo, rem_d_hi;
	wire rd_bo_lo, rd_bo_hi;
	am_sub #(.W(26)) u_rem_d_lo (.a(rem), .b({2'b00, d_lo}), .bi(1'b0), .d(rem_d_lo), .bo(rd_bo_lo));
	am_sub #(.W(26)) u_rem_d_hi (.a(rem), .b({2'b00, d_hi}), .bi(1'b0), .d(rem_d_hi), .bo(rd_bo_hi));

	wire [24:0] rem_next_shift;
	assign rem_next_shift[12:0]  = rd_bo_lo ? rem[12:0]  : rem_d_lo[12:0];
	assign rem_next_shift[24:13] = rd_bo_hi ? rem[24:13] : rem_d_hi[24:13];

	wire [24:0] q_n = {q_reg[23:0], !rd_bo_hi};
	wire round_up_w = q_reg[0] && (q_reg[1] || (rem[25:1] != 25'd0));

	(* keep = "true" *) wire use_e1 = r_round_up_b && r_m0_inc[24];
	wire [7:0] s2_e = use_e1 ? r_ef_1 : r_ef_0;
	(* keep = "true" *) wire mz_s2_a = (r_round_up_a && r_m0_inc[24]) ? r_mz_1_a : r_mz_0_a;
	(* keep = "true" *) wire mz_s2_b = use_e1 ? r_mz_1_b : r_mz_0_b;
	wire [22:0] s2_m;
	assign s2_m[15:0]  = mz_s2_a ? 16'd0 : (r_round_up_a ? r_m0_inc[15:0]  : q_reg[16:1]);
	assign s2_m[22:16] = mz_s2_b ? 7'd0  : (r_round_up_b ? r_m0_inc[22:16] : q_reg[23:17]);
	wire [31:0] s2_or_val = {sgn, s2_e, s2_m};

	wire unused_m = &{1'b0, rem_d_lo[25:13], rem_d_hi[12:0], e_p1_w[9:8],
		ma_mb_unused, eab_bo_unused, en_co_unused, cm_bo_unused, m0_co_unused, ep1_co_unused};
`elsif SYNTH_KINTEX_ULTRASCALE
	(* extract_reset = "no" *) reg [25:0] rem;
	wire [25:0] rem_d_lo, rem_d_hi;
	wire rd_bo_lo, rd_bo_hi;
	am_sub #(.W(26)) u_rem_d_lo (.a(rem), .b({2'b00, d_lo}), .bi(1'b0), .d(rem_d_lo), .bo(rd_bo_lo));
	am_sub #(.W(26)) u_rem_d_hi (.a(rem), .b({2'b00, d_hi}), .bi(1'b0), .d(rem_d_hi), .bo(rd_bo_hi));

	wire [24:0] rem_next_shift;
	assign rem_next_shift[12:0]  = rd_bo_lo ? rem[12:0]  : rem_d_lo[12:0];
	assign rem_next_shift[24:13] = rd_bo_hi ? rem[24:13] : rem_d_hi[24:13];

	wire [24:0] q_n = {q_reg[23:0], !rd_bo_hi};
	wire round_up_w = q_reg[0] && (q_reg[1] || (rem[25:1] != 25'd0));

	(* keep = "true" *) wire use_e1 = r_round_up_b && r_m0_inc[24];
	wire [7:0] s2_e = use_e1 ? r_ef_1 : r_ef_0;
	(* keep = "true" *) wire mz_s2_a = (r_round_up_a && r_m0_inc[24]) ? r_mz_1_a : r_mz_0_a;
	(* keep = "true" *) wire mz_s2_b = use_e1 ? r_mz_1_b : r_mz_0_b;
	wire [22:0] s2_m;
	assign s2_m[15:0]  = mz_s2_a ? 16'd0 : (r_round_up_a ? r_m0_inc[15:0]  : q_reg[16:1]);
	assign s2_m[22:16] = mz_s2_b ? 7'd0  : (r_round_up_b ? r_m0_inc[22:16] : q_reg[23:17]);
	wire [31:0] s2_or_val = {sgn, s2_e, s2_m};

	wire unused_m = &{1'b0, rem_d_lo[25:13], rem_d_hi[12:0], e_p1_w[9:8],
		ma_mb_unused, eab_bo_unused, en_co_unused, cm_bo_unused, m0_co_unused, ep1_co_unused};
`else
	/* ASIC 15 FO4: SRT Radix-2 Carry-Save Divider (<= 11 cmos3/stage, same 28-cycle latency) */
	(* extract_reset = "no" *) reg [26:0] rem_s, rem_c;
	(* extract_reset = "no" *) reg [24:0] qm_reg;
	reg r_rem_neg, r_rem_zero, r_rem_neg_d;

	/* Carry-free / 2+3 carry-select q-digit selection + balanced CSA mux (<= 11 cmos3) */
	wire [4:0] vs_a = rem_s[26:22];
	wire [4:0] vs_b = rem_c[26:22];
	wire [4:0] v_na = ~vs_a;
	wire [4:0] v_nb = ~vs_b;
	wire [3:0] vt_g = vs_a[3:0] & vs_b[3:0];
	wire [3:0] vt_p = vs_a[3:0] | vs_b[3:0];
	wire vt_c2 = vt_g[1] | (vt_p[1] & vt_g[0]);
	wire vt_c4_0 = vt_g[3] | (vt_p[3] & vt_g[2]);
	wire vt_c4_1 = vt_g[3] | (vt_p[3] & (vt_g[2] | vt_p[2]));
	wire vt_s4_0 = vs_a[4] ^ vs_b[4] ^ vt_c4_0;
	wire vt_s4_1 = vs_a[4] ^ vs_b[4] ^ vt_c4_1;
	(* keep = "true" *) wire v_top4 = vt_c2 ? vt_s4_1 : vt_s4_0;
	wire [4:0] v_top = {v_top4, 4'd0};
	wire v_top_co_un = 1'b0;

	(* keep = "true" *) wire v_is_m1 = &((vs_a ^ vs_b));
	wire [4:0] vz_chk = {
		(vs_a[4] ^ vs_b[4]) ~^ (vs_a[3] | vs_b[3]),
		(vs_a[3] ^ vs_b[3]) ~^ (vs_a[2] | vs_b[2]),
		(vs_a[2] ^ vs_b[2]) ~^ (vs_a[1] | vs_b[1]),
		(vs_a[1] ^ vs_b[1]) ~^ (vs_a[0] | vs_b[0]),
		!(vs_a[0] ^ vs_b[0])
	};
	wire [4:0] vm2_chk = {
		(v_na[4] ^ v_nb[4]) ~^ (v_na[3] | v_nb[3]),
		(v_na[3] ^ v_nb[3]) ~^ (v_na[2] | v_nb[2]),
		(v_na[2] ^ v_nb[2]) ~^ (v_na[1] | v_nb[1]),
		(v_na[1] ^ v_nb[1]) ~^ (v_na[0] | v_nb[0]),
		!(v_na[0] ^ v_nb[0])
	};
	(* keep = "true" *) wire v_is_0  = (&vz_chk[4:2])  & (&vz_chk[1:0]);
	(* keep = "true" *) wire v_is_m2 = (&vm2_chk[4:2]) & (&vm2_chk[1:0]);

	wire q_plus1  = !v_top4 && !v_is_0;
	wire q_minus1 =  v_top4 && !v_is_m1 && !v_is_m2;
	wire q_zero   = v_is_0 || v_is_m1 || v_is_m2;

	/* Speculative 3:2 CSAs in parallel with q-digit selection (<= 3 cmos3) */
	wire [26:0] d_ext = {3'b000, d_hi[23:12], d_lo[11:0]};
	wire [26:0] dn_ext = ~d_ext;
	wire [26:0] csa_p = rem_s ^ rem_c;
	wire [26:0] csa_g = rem_s & rem_c;

	wire [26:0] s_sub = csa_p ^ dn_ext;
	wire [26:0] k_sub = {csa_g[25:0] | (csa_p[25:0] & dn_ext[25:0]), 1'b1};
	wire [26:0] s_add = csa_p ^ d_ext;
	wire [26:0] k_add = {csa_g[25:0] | (csa_p[25:0] & d_ext[25:0]), 1'b0};
	wire [26:0] s_zer = csa_p;
	wire [26:0] k_zer = {csa_g[25:0], 1'b0};

	(* keep = "true", dont_touch = "true" *) wire [2:0] qp_r0_rep = {3{run_r0 && !v_top4 && !v_is_0}};
	(* keep = "true", dont_touch = "true" *) wire [2:0] qm_r0_rep = {3{run_r0 &&  v_top4 && !v_is_m1 && !v_is_m2}};
	(* keep = "true", dont_touch = "true" *) wire [2:0] qz_r0_rep = {3{run_r0 && (v_is_0 || v_is_m1 || v_is_m2)}};
	(* keep = "true", dont_touch = "true" *) wire [2:0] qp_r1_rep = {3{run_r1 && !v_top4 && !v_is_0}};
	(* keep = "true", dont_touch = "true" *) wire [2:0] qm_r1_rep = {3{run_r1 &&  v_top4 && !v_is_m1 && !v_is_m2}};
	(* keep = "true", dont_touch = "true" *) wire [2:0] qz_r1_rep = {3{run_r1 && (v_is_0 || v_is_m1 || v_is_m2)}};

	(* keep = "true" *) wire [12:0] csa_s_run_lo =
		({{4{qp_r0_rep[2]}}, {4{qp_r0_rep[1]}}, {5{qp_r0_rep[0]}}} & {s_sub[11:0], 1'b0}) |
		({{4{qm_r0_rep[2]}}, {4{qm_r0_rep[1]}}, {5{qm_r0_rep[0]}}} & {s_add[11:0], 1'b0}) |
		({{4{qz_r0_rep[2]}}, {4{qz_r0_rep[1]}}, {5{qz_r0_rep[0]}}} & {s_zer[11:0], 1'b0});
	(* keep = "true" *) wire [12:0] csa_k_run_lo =
		({{4{qp_r0_rep[2]}}, {4{qp_r0_rep[1]}}, {5{qp_r0_rep[0]}}} & {k_sub[11:0], 1'b0}) |
		({{4{qm_r0_rep[2]}}, {4{qm_r0_rep[1]}}, {5{qm_r0_rep[0]}}} & {k_add[11:0], 1'b0}) |
		({{4{qz_r0_rep[2]}}, {4{qz_r0_rep[1]}}, {5{qz_r0_rep[0]}}} & {k_zer[11:0], 1'b0});
	(* keep = "true" *) wire [26:13] csa_s_run_hi =
		({{4{qp_r1_rep[2]}}, {5{qp_r1_rep[1]}}, {5{qp_r1_rep[0]}}} & s_sub[25:12]) |
		({{4{qm_r1_rep[2]}}, {5{qm_r1_rep[1]}}, {5{qm_r1_rep[0]}}} & s_add[25:12]) |
		({{4{qz_r1_rep[2]}}, {5{qz_r1_rep[1]}}, {5{qz_r1_rep[0]}}} & s_zer[25:12]);
	(* keep = "true" *) wire [26:13] csa_k_run_hi =
		({{4{qp_r1_rep[2]}}, {5{qp_r1_rep[1]}}, {5{qp_r1_rep[0]}}} & k_sub[25:12]) |
		({{4{qm_r1_rep[2]}}, {5{qm_r1_rep[1]}}, {5{qm_r1_rep[0]}}} & k_add[25:12]) |
		({{4{qz_r1_rep[2]}}, {5{qz_r1_rep[1]}}, {5{qz_r1_rep[0]}}} & k_zer[25:12]);

	(* keep = "true" *) wire [12:0] rem_s_idle_lo = prep_ctrl_a ? rem_init_lo : rem_s[12:0];
	(* keep = "true" *) wire [12:0] rem_c_idle_lo = prep_ctrl_a ? 13'd0       : rem_c[12:0];
	(* keep = "true" *) wire [26:13] rem_s_idle_hi = prep_ctrl_b ? {2'b00, rem_init_hi} : rem_s[26:13];
	(* keep = "true" *) wire [26:13] rem_c_idle_hi = prep_ctrl_b ? 14'd0                : rem_c[26:13];

	wire [12:0] next_rem_s_lo = csa_s_run_lo | ({13{!run_r0}} & rem_s_idle_lo);
	wire [12:0] next_rem_c_lo = csa_k_run_lo | ({13{!run_r0}} & rem_c_idle_lo);
	wire [26:13] next_rem_s_hi = csa_s_run_hi | ({14{!run_r1}} & rem_s_idle_hi);
	wire [26:13] next_rem_c_hi = csa_k_run_hi | ({14{!run_r1}} & rem_c_idle_hi);
	wire [26:0] csa_s = {25'd0, ce_r0, ce_r1};
	wire [26:0] csa_k = 27'd0;

	/* On-the-fly quotient conversion */
	wire [24:0] q_n  = q_minus1 ? {qm_reg[23:0], 1'b1} : {q_reg[23:0], q_plus1};
	wire [24:0] qm_n = q_plus1  ? {q_reg[23:0],  1'b0} : {qm_reg[23:0], q_zero};

	/* round_s1 final remainder sign via pure-LUT 2->4->8/12/13 parallel CLA (<= 7 cmos3) */
	wire [24:0] fs_g = rem_s[25:1] & rem_c[25:1];
	wire [24:0] fs_p = rem_s[25:1] | rem_c[25:1];
	wire [11:0] fs_g2, fs_p2;
	genvar gfs;
	generate
	for (gfs = 0; gfs < 12; gfs = gfs + 1) begin : g_fsg2
		assign fs_p2[gfs] = fs_p[2*gfs+1] & fs_p[2*gfs];
		assign fs_g2[gfs] = fs_g[2*gfs+1] | (fs_p[2*gfs+1] & fs_g[2*gfs]);
	end
	endgenerate
	wire [5:0] fs_g4, fs_p4;
	generate
	for (gfs = 0; gfs < 6; gfs = gfs + 1) begin : g_fsg4
		assign fs_p4[gfs] = fs_p2[2*gfs+1] & fs_p2[2*gfs];
		assign fs_g4[gfs] = fs_g2[2*gfs+1] | (fs_p2[2*gfs+1] & fs_g2[2*gfs]);
	end
	endgenerate
	wire fs_g12_lo = fs_g4[2] | (fs_p4[2] & fs_g4[1]) | (fs_p4[2] & fs_p4[1] & fs_g4[0]);
	wire fs_p12_lo = fs_p4[2] & fs_p4[1] & fs_p4[0];
	wire fs_g13_hi = fs_g[24] | (fs_p[24] & fs_g4[5]) | (fs_p[24] & fs_p4[5] & fs_g4[4]) | (fs_p[24] & fs_p4[5] & fs_p4[4] & fs_g4[3]);
	wire fs_p13_hi = fs_p[24] & fs_p4[5] & fs_p4[4] & fs_p4[3];
	wire fs_c25 = fs_g13_hi | (fs_p13_hi & fs_g12_lo);
	wire rem_neg_w = rem_s[26] ^ rem_c[26] ^ fs_c25;
	wire [25:0] fin_sum_w = {rem_neg_w, 25'd0};
	wire fin_sum_co_un = fs_p12_lo;

	wire [25:0] rz_eq_bit;
	assign rz_eq_bit[0] = !(rem_s[1] ^ rem_c[1]);
	genvar grz;
	generate
	for (grz = 1; grz < 26; grz = grz + 1) begin : g_rz
		assign rz_eq_bit[grz] = ((rem_s[grz + 1] ^ rem_c[grz + 1]) == (rem_s[grz] | rem_c[grz]));
	end
	endgenerate
	wire [6:0] rz_blk_w = {
		&rz_eq_bit[25:24],
		(&rz_eq_bit[23:22]) & (&rz_eq_bit[21:20]),
		(&rz_eq_bit[19:18]) & (&rz_eq_bit[17:16]),
		(&rz_eq_bit[15:14]) & (&rz_eq_bit[13:12]),
		(&rz_eq_bit[11:10]) & (&rz_eq_bit[9:8]),
		(&rz_eq_bit[7:6])   & (&rz_eq_bit[5:4]),
		(&rz_eq_bit[3:2])   & (&rz_eq_bit[1:0])
	};
	reg [6:0] r_rz_blk;
	wire rem_zero_w = 1'b0;

	(* equivalent_register_removal = "no", preserve = "true" *) reg [11:0] d_lo_rnd, d_hi_rnd;
	wire [25:0] d_s1 = {2'b00, d_hi_rnd, d_lo_rnd};
	wire [25:0] rnd_u_lo = rem_s[26:1] ^ rem_c[26:1];
	wire [25:0] rnd_u    = rnd_u_lo ^ d_s1;
	wire [25:0] rnd_v    = {((rem_s[25:1] & rem_c[25:1]) | (rem_s[25:1] & d_s1[24:0]) | (rem_c[25:1] & d_s1[24:0])), 1'b0};
	wire [25:0] rnd_eq_bit;
	assign rnd_eq_bit[0] = !(rnd_u[0] ^ rnd_v[0]);
	generate
	for (grz = 1; grz < 26; grz = grz + 1) begin : g_rnd
		wire pa = rem_s[grz];
		wire pb = rem_c[grz];
		wire pc = d_s1[grz - 1];
		wire rhs_v0 = !((pa & pb & pc) | (~pa & ~pb & ~pc));
		wire rhs_v1 = !((pa & pb) | ((pa | pb) & pc));
		wire rhs_w  = rnd_v[grz - 1] ? rhs_v1 : rhs_v0;
		assign rnd_eq_bit[grz] = (rnd_u[grz] == rhs_w);
	end
	endgenerate
	reg [25:0] r_rnd_eq;
	wire rem_neg_d_w = 1'b0;
	wire round_up_w = 1'b0;

	/* round_s2 packing: compute both !rem_neg_d == 1 (nz) and == 0 (zero) in parallel with 26-bit AND (<= 7 cmos3) */
	wire [12:0] rnd_p2_w = {
		&r_rnd_eq[25:24], &r_rnd_eq[23:22], &r_rnd_eq[21:20], &r_rnd_eq[19:18],
		&r_rnd_eq[17:16], &r_rnd_eq[15:14], &r_rnd_eq[13:12], &r_rnd_eq[11:10],
		&r_rnd_eq[9:8],   &r_rnd_eq[7:6],   &r_rnd_eq[5:4],   &r_rnd_eq[3:2],   &r_rnd_eq[1:0]
	};
	(* keep = "true" *) wire [6:0] rnd_p4_w = {
		rnd_p2_w[12],
		rnd_p2_w[11] & rnd_p2_w[10],
		rnd_p2_w[9]  & rnd_p2_w[8],
		rnd_p2_w[7]  & rnd_p2_w[6],
		rnd_p2_w[5]  & rnd_p2_w[4],
		rnd_p2_w[3]  & rnd_p2_w[2],
		rnd_p2_w[1]  & rnd_p2_w[0]
	};
	wire rnd_g0_a = rnd_p4_w[6] & rnd_p4_w[5] & rnd_p4_w[4] & rnd_p4_w[3];
	wire rnd_g1_a = rnd_p4_w[2] & rnd_p4_w[1] & rnd_p4_w[0];
	(* keep = "true" *) wire rnd_eq_all_a = rnd_g0_a & rnd_g1_a;
	(* keep = "true" *) wire rnd_eq_all_b = rnd_g0_a & rnd_g1_a;
	(* keep = "true" *) wire rnd_eq_all_c = rnd_g0_a & rnd_g1_a;
	wire rem_zero_s2 = &r_rz_blk;

	wire q0_eff = r_rem_neg ? qm_reg[0] : q_reg[0];
	wire q1_eff = r_rem_neg ? qm_reg[1] : q_reg[1];
	wire r_up_pos = q0_eff && (q1_eff || !rem_zero_s2);
	wire r_up_neg_nz = q0_eff;
	wire r_up_neg_z  = q0_eff && q1_eff;
	wire [30:0] s2_pos_val = {
		(!r_rem_neg && r_up_pos && r_m0_inc[24]) ? r_ef_1 : r_ef_0,
		(!r_rem_neg && r_up_pos && r_m0_inc[24]) ? (r_mz_1_b ? 7'd0 : (r_up_pos ? r_m0_inc[22:16] : q_reg[23:17]))
		                                         : (r_mz_0_b ? 7'd0 : (r_up_pos ? r_m0_inc[22:16] : q_reg[23:17])),
		(!r_rem_neg && r_up_pos && r_m0_inc[24]) ? (r_mz_1_a ? 16'd0 : (r_up_pos ? r_m0_inc[15:0]  : q_reg[16:1]))
		                                         : (r_mz_0_a ? 16'd0 : (r_up_pos ? r_m0_inc[15:0]  : q_reg[16:1]))
	};
	wire [22:0] mant_neg_nz = r_up_neg_nz ? q_reg[23:1] : qm_reg[23:1];
	wire [22:0] mant_neg_z  = r_up_neg_z  ? q_reg[23:1] : qm_reg[23:1];
	wire [30:0] s2_neg_nz = {r_ef_0, r_mz_0_b ? 7'd0 : mant_neg_nz[22:16], r_mz_0_a ? 16'd0 : mant_neg_nz[15:0]};
	wire [30:0] s2_neg_z  = {r_ef_0, r_mz_0_b ? 7'd0 : mant_neg_z[22:16],  r_mz_0_a ? 16'd0 : mant_neg_z[15:0]};
	wire [31:0] s2_or_val;
	assign s2_or_val[31]    = sgn;
	assign s2_or_val[30:23] = !r_rem_neg ? s2_pos_val[30:23] : (rnd_eq_all_a ? s2_neg_z[30:23] : s2_neg_nz[30:23]);
	assign s2_or_val[22:16] = !r_rem_neg ? s2_pos_val[22:16] : (rnd_eq_all_b ? s2_neg_z[22:16] : s2_neg_nz[22:16]);
	assign s2_or_val[15:0]  = !r_rem_neg ? s2_pos_val[15:0]  : (rnd_eq_all_c ? s2_neg_z[15:0]  : s2_neg_nz[15:0]);

	wire unused_m = &{1'b0, v_top_co_un, fin_sum_co_un, fin_sum_w[24:0], csa_s[26], csa_k[26],
		r_round_up_a, r_round_up_b, e_p1_w[9:8], ma_mb_unused, eab_bo_unused, en_co_unused,
		cm_bo_unused, m0_co_unused, ep1_co_unused, r_rem_zero, r_rem_neg_d};
`endif

	always @(posedge clk or posedge rst_sync) begin
		if (rst_sync) begin
			o_done <= 1'b0;
			prep_ctrl_a <= 1'b0;
			prep_ctrl_b <= 1'b0;
			prep_ctrl_c <= 1'b0;
			run_ctrl_a <= 1'b0;
			run_ctrl_b <= 1'b0;
			run_ctrl_c <= 1'b0;
			run_q0 <= 1'b0;
			run_q1 <= 1'b0;
			run_r0 <= 1'b0;
			run_r1 <= 1'b0;
			r_last <= 1'b0;
			round_s1_ctrl <= 1'b0;
			round_s1_a <= 1'b0;
			round_s1_b <= 1'b0;
			round_s1_c <= 1'b0;
			round_s2_ctrl <= 1'b0;
			round_s2_a <= 1'b0;
			round_s2_b <= 1'b0;
		end else begin
			o_done <= (prep_ctrl_b && r_sp_any) || round_s2_ctrl;

			if (start_ok_a) begin
				prep_ctrl_a <= 1'b1;
				prep_ctrl_b <= 1'b1;
				prep_ctrl_c <= 1'b1;
			end else if (prep_ctrl_a) begin
				prep_ctrl_a <= 1'b0;
				prep_ctrl_b <= 1'b0;
				prep_ctrl_c <= 1'b0;
			end

			if (prep_ctrl_a && !r_sp_any) begin
				run_ctrl_a <= 1'b1;
				run_ctrl_b <= 1'b1;
				run_ctrl_c <= 1'b1;
				run_q0 <= 1'b1;
				run_q1 <= 1'b1;
				run_r0 <= 1'b1;
				run_r1 <= 1'b1;
			end else if (run_ctrl_c && r_last) begin
				run_ctrl_a <= 1'b0;
				run_ctrl_b <= 1'b0;
				run_ctrl_c <= 1'b0;
				run_q0 <= 1'b0;
				run_q1 <= 1'b0;
				run_r0 <= 1'b0;
				run_r1 <= 1'b0;
			end

			if (prep_ctrl_b)
				r_last <= 1'b0;
			else if (run_ctrl_c)
				r_last <= (cnt == 5'd1);

			if (run_ctrl_c && r_last) begin
				round_s1_ctrl <= 1'b1;
				round_s1_a <= 1'b1;
				round_s1_b <= 1'b1;
				round_s1_c <= 1'b1;
			end else if (round_s1_ctrl) begin
				round_s1_ctrl <= 1'b0;
				round_s1_a <= 1'b0;
				round_s1_b <= 1'b0;
				round_s1_c <= 1'b0;
			end

			if (round_s1_ctrl) begin
				round_s2_ctrl <= 1'b1;
				round_s2_a <= 1'b1;
				round_s2_b <= 1'b1;
			end else if (round_s2_ctrl) begin
				round_s2_ctrl <= 1'b0;
				round_s2_a <= 1'b0;
				round_s2_b <= 1'b0;
			end
		end
	end

	always @(posedge clk) begin
		if (start_ok_a) begin
			sgn <= s;
			r_sp_any <= sp_any;
			r_sp_nan <= sp_nan;
			r_sp_exp_ff <= sp_exp_ff;
			r_ma_lt_mb_a <= ma_lt_mb_raw;
			r_ma_lt_mb_b <= ma_lt_mb_raw;
		end

		if (round_s1_ctrl) begin
			r_round_up_a <= round_up_w;
			r_round_up_b <= round_up_w;
			r_mz_0_a <= mz_0_w;
			r_mz_0_b <= mz_0_w;
			r_mz_1_a <= mz_1_w;
			r_mz_1_b <= mz_1_w;
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			r_rem_neg   <= rem_neg_w;
			r_rem_zero  <= rem_zero_w;
			r_rem_neg_d <= rem_neg_d_w;
			r_rz_blk    <= rz_blk_w;
			r_rnd_eq    <= rnd_eq_bit;
`endif
		end

		if (start_eab)
			r_eab <= eab;
		if (prep_ctrl_c) begin
			e <= e_n;
			cnt <= 5'd24;
		end else if (run_ctrl_c) begin
			cnt <= cnt_m1;
		end

		if (start_m0)
			r_ma[11:0] <= ma[11:0];
		if (start_m1)
			r_ma[23:12] <= ma[23:12];

		if (start_d0) begin
			d_lo[11:0] <= mb[11:0];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			d_lo_rnd   <= mb[11:0];
`endif
		end
		if (start_d1)
			d_lo[23:12] <= mb[23:12];
		if (start_d2)
			d_hi[11:0] <= mb[11:0];
		if (start_d3) begin
			d_hi[23:12] <= mb[23:12];
`ifdef SYNTH_KINTEX7
`elsif SYNTH_KINTEX_ULTRASCALE
`else
			d_hi_rnd    <= mb[23:12];
`endif
		end

		if (ce_q0)
			q_reg[12:0] <= !run_q0 ? 13'd0 : q_n[12:0];
		if (ce_q1)
			q_reg[24:13] <= !run_q1 ? 12'd0 : q_n[24:13];

`ifdef SYNTH_KINTEX7
		if (ce_r0)
			rem[12:0] <= !run_r0 ? rem_init_lo : {rem_next_shift[11:0], 1'b0};
		if (ce_r1)
			rem[25:13] <= !run_r1 ? {1'b0, rem_init_hi} : rem_next_shift[24:12];
`elsif SYNTH_KINTEX_ULTRASCALE
		if (ce_r0)
			rem[12:0] <= !run_r0 ? rem_init_lo : {rem_next_shift[11:0], 1'b0};
		if (ce_r1)
			rem[25:13] <= !run_r1 ? {1'b0, rem_init_hi} : rem_next_shift[24:12];
`else
		if (ce_q0)
			qm_reg[12:0] <= !run_q0 ? 13'd0 : qm_n[12:0];
		if (ce_q1)
			qm_reg[24:13] <= !run_q1 ? 12'd0 : qm_n[24:13];
		rem_s[12:0]  <= next_rem_s_lo;
		rem_c[12:0]  <= next_rem_c_lo;
		rem_s[26:13] <= next_rem_s_hi;
		rem_c[26:13] <= next_rem_c_hi;
`endif

		if (round_s1_a)
			r_m0_inc[15:0] <= m0_inc_w[15:0];
		if (round_s1_b)
			r_m0_inc[24:16] <= m0_inc_w[24:16];
		if (round_s1_c) begin
			r_ef_0 <= ef_0_w;
			r_ef_1 <= ef_1_w;
		end

		if (prep_sp_a)
			o_r[15:0] <= sp_or_val[15:0];
		else if (round_s2_a)
			o_r[15:0] <= s2_or_val[15:0];

		if (prep_sp_b)
			o_r[31:16] <= sp_or_val[31:16];
		else if (round_s2_b)
			o_r[31:16] <= s2_or_val[31:16];
	end
	wire unused_ok = &{1'b0, unused_m};
endmodule

`default_nettype wire
