/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_su_pk_half: one 16-bit half of a lane of atg_su_pk.
 * Supports LAT == 0 (combinational, for unit testbench) and LAT == 6
 * (6-stage physical pipeline with 0 Pre/Post-DSP LUTs and <= 1 Pre/Post-CARRY
 * LUTs for 1.5 GHz ASIC synthesis).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_su_pk_half #(
	parameter ARITH = 1,	/* the packed arithmetic (atg_su_pk's PACKED) */
	parameter FP = 1,	/* with ARITH: the float operations here (not PK_SHARED) */
	parameter LAT = 0	/* 0 = combinational, 6 = 6-stage pipeline */
) (
	input wire clk,
	input wire rst_n,
	input wire valid_in,
	input wire ovr_en,
	input wire [15:0] ovr_val,
	input wire [3:0] o,
	input wire [15:0] ha,
	input wire [15:0] hb,
	input wire [15:0] hc,
	input wire [2:0] r,
	input wire sat,
	input wire [31:0] xw,
	output wire [15:0] y
);
`include "atg_su_uop.vh"
`include "atg_su_fp16.vh"

	localparam AR = ARITH != 0;
	localparam FL = ARITH != 0 && FP != 0;

	generate
	if (LAT == 0) begin : g_lat0
		wire packh = o == ATG_PK_PACKH[3:0];
		wire is_int = o == ATG_PK_IADD2[3:0] || o == ATG_PK_ISUB2[3:0] || o == ATG_PK_IMUL2[3:0];

		reg [15:0] fy, fz;
		reg fnoc;
		always @(*) begin
			fy = 16'h3C00;
			fz = hb;
			fnoc = 1'b0;
			case (o)
			ATG_PK_FSUB2[3:0]: fz = {!hb[15], hb[14:0]};
			ATG_PK_FMUL2[3:0]: begin
				fy = hb;
				fz = 16'd0;
				fnoc = 1'b1;
			end
			ATG_PK_FFMA2[3:0]: begin
				fy = hb;
				fz = hc;
			end
			default: ;
			endcase
		end

		wire [15:0] fa = atg_su_hflush(ha);
		wire [15:0] fb = atg_su_hflush(fy);
		wire [15:0] fc = atg_su_hflush(fz);
		wire [4:0] ea = fa[14:10];
		wire [4:0] eb = fb[14:10];
		wire [4:0] ec = fc[14:10];
		wire sp = fa[15] ^ fb[15];
		wire sc = fc[15];
		wire rdn = r == 3'd2;
		wire nan_a = ea == 5'h1F && fa[9:0] != 10'd0;
		wire nan_b = eb == 5'h1F && fb[9:0] != 10'd0;
		wire nan_c = !fnoc && ec == 5'h1F && fc[9:0] != 10'd0;
		wire inf_a = ea == 5'h1F && fa[9:0] == 10'd0;
		wire inf_b = eb == 5'h1F && fb[9:0] == 10'd0;
		wire inf_c = !fnoc && ec == 5'h1F && fc[9:0] == 10'd0;
		wire za = ea == 5'd0;
		wire zb = eb == 5'd0;
		wire zc = fnoc || ec == 5'd0;
		wire [10:0] ma = {!za, fa[9:0]};
		wire [10:0] mb = {!zb, fb[9:0]};
		wire [10:0] mc = zc ? 11'd0 : {1'b1, fc[9:0]};

		wire [31:0] prod;
		am_mul #(.AW(16), .BW(16)) u_mul (
			.clk(1'b0), .en(1'b0), .a(is_int ? ha : {5'd0, ma}), .b(is_int ? hb : {5'd0, mb}),
			.p(prod)
		);
		wire [21:0] p = prod[21:0];

		wire [9:0] t;
		wire [9:0] d;
		wire [9:0] kx;
		wire [2:0] co_unused;
		am_add #(.W(10)) u_t (.a({5'd0, ea}), .b({5'd0, eb}), .ci(1'b0), .s(t), .co(co_unused[0]));
		am_sub #(.W(10)) u_d (.a({5'd0, ec}), .b(t), .bi(1'b0), .d(d), .bo(co_unused[1]));
		wire big = $signed(d) >= -10'sd1;
		wire near = $signed(d) >= -10'sd37;
		am_add #(.W(10)) u_k (
			.a(big ? {5'd0, ec} : d), .b(big ? 10'd2 : 10'd37), .ci(1'b0), .s(kx),
			.co(co_unused[2])
		);
		wire [46:0] zal;
		am_shift #(.W(47), .SW(6), .DIR(2)) u_al (
			.a({36'd0, mc}), .amt(kx[5:0]), .left(1'b1), .arith(1'b0), .y(zal)
		);
		wire [36:0] pw = !zc && big ? 37'd1 : {13'd0, p, 2'b00};
		wire [36:0] cw = zc ? 37'd0 : big ? {1'b0, mc, 25'd0} :
			near ? {zal[46:11], zal[10] | (|zal[9:0])} : 37'd1;

		wire eff_sub = !(zc || sp == sc);
		wire swap = eff_sub && pw < cw;
		wire sr = swap ? sc : sp;
		wire [36:0] sum;
		wire sum_co_unused;
		am_addsub #(.W(37)) u_sum (
			.a(is_int ? {21'd0, ha} : swap ? cw : pw), .b(is_int ? {21'd0, hb} : swap ? pw : cw),
			.sub(is_int ? o == ATG_PK_ISUB2[3:0] : eff_sub), .s(sum), .co(sum_co_unused)
		);

		wire [36:0] rev;
		genvar j;
		for (j = 0; j < 37; j = j + 1) begin : g_rev
			assign rev[j] = sum[36 - j];
		end
		reg [5:0] lz;
		integer i;
		always @(*) begin
			lz = 6'd0;
			for (i = 36; i >= 0; i = i - 1)
				if (rev[i])
					lz = i[5:0];
		end
		wire [36:0] nrm;
		am_shift #(.W(37), .SW(6), .DIR(2)) u_nrm (
			.a(sum), .amt(lz), .left(1'b1), .arith(1'b0), .y(nrm)
		);

		wire [31:0] xf = xw[30:23] == 8'd0 ? {xw[31], 31'd0} : xw;

		wire f2h = packh || !FL;
		wire [9:0] er;
		wire er_bo_unused;
		am_sub #(.W(10)) u_er (
			.a(f2h ? {2'd0, xf[30:23]} : !zc && big ? kx : t), .b(f2h ? 10'd112 : {4'd0, lz}),
			.bi(!f2h), .d(er), .bo(er_bo_unused)
		);
		wire [15:0] hr;
		am_fp_hrnd #(.LAT(0)) u_rnd (
			.s(f2h ? xf[31] : sr), .e(er), .m(f2h ? {1'b1, xf[22:13]} : nrm[36:26]),
			.g(f2h ? xf[12] : nrm[25]), .st(f2h ? |xf[11:0] : |nrm[24:0]),
			.rm(f2h ? 3'd0 : r), .h(hr)
		);

		wire [15:0] xh = xf[30:23] == 8'hFF ? (xf[22:0] != 23'd0 ? 16'h7E00 : {xf[31], 5'h1F, 10'd0}) :
			xf[30:23] == 8'd0 ? {xf[31], 15'd0} : hr;

		reg [15:0] fma;
		always @(*) begin
			if (nan_a || nan_b || nan_c || (inf_a && zb) || (inf_b && za))
				fma = 16'h7E00;
			else if (inf_a || inf_b)
				fma = inf_c && sc != sp ? 16'h7E00 : {sp, 5'h1F, 10'd0};
			else if (inf_c)
				fma = fc;
			else if (za || zb)
				fma = !zc ? fc : fnoc ? {sp, 15'd0} : {rdn ? (sp | sc) : (sp & sc), 15'd0};
			else if (sum == 37'd0)
				fma = {rdn, 15'd0};
			else
				fma = hr;
		end

		reg [15:0] ca, cb;
		reg [1:0] cop;
		always @(*) begin
			ca = ha;
			cb = hb;
			case (o)
			ATG_PK_FCMPLT2[3:0]: cop = 2'd1;
			ATG_PK_FCMPGE2[3:0]: begin
				ca = hb;
				cb = ha;
				cop = 2'd2;
			end
			ATG_PK_FCMPEQ2[3:0]: cop = 2'd0;
			default: cop = 2'd3;
			endcase
		end

		reg [15:0] v;
		always @(*)
			case (o)
			ATG_PK_FADD2[3:0], ATG_PK_FSUB2[3:0], ATG_PK_FMUL2[3:0], ATG_PK_FFMA2[3:0]:
				v = FL ? fma : 16'd0;
			ATG_PK_FMIN2[3:0]: v = FL ? atg_su_hminmax(ha, hb, 1'b0) : 16'd0;
			ATG_PK_FMAX2[3:0]: v = FL ? atg_su_hminmax(ha, hb, 1'b1) : 16'd0;
			ATG_PK_IADD2[3:0], ATG_PK_ISUB2[3:0]: v = sum[15:0];
			ATG_PK_IMUL2[3:0]: v = prod[15:0];
			ATG_PK_FCMPLT2[3:0], ATG_PK_FCMPGE2[3:0], ATG_PK_FCMPEQ2[3:0], ATG_PK_FCMPNE2[3:0]:
				v = FL ? {16{atg_su_hcmp(ca, cb, cop)}} : 16'd0;
			default: v = 16'd0;
			endcase

		assign y = !valid_in ? 16'd0 : ovr_en ? ovr_val :
			packh ? xh : !AR ? 16'd0 : sat && o <= ATG_PK_FMAX2[3:0] ? atg_su_hsat(v) : v;

		wire unused_ok = &{1'b0, clk, rst_n, prod[31:22], sum[36:16], kx[9:6], er_bo_unused, sum_co_unused,
			co_unused};
	end else if (!AR) begin : g_lat6_noarith
		wire rst_p = !rst_n;
		(* shreg_extract = "no" *) reg        s1_valid, s2_valid, s3_valid, s4_valid, s5_valid, s6_valid;
		(* shreg_extract = "no" *) reg        s1_ovr_en, s2_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s1_ovr_val, s2_ovr_val;
		(* shreg_extract = "no" *) reg        s1_packh, s2_packh;
		(* shreg_extract = "no" *) reg [31:0] s1_xw, s2_xw;
		(* shreg_extract = "no" *) reg [9:0]  s2_er_p;
		(* shreg_extract = "no" *) reg [15:0] s3_res, s4_res, s5_res, s6_res;

		wire [9:0] s1_er_p;
		wire       s1_erp_bo_unused;
		am_sub #(.W(10)) u_s1_erp (.a({2'd0, s1_xw[30:23]}), .b(10'd112), .bi(1'b0), .d(s1_er_p), .bo(s1_erp_bo_unused));

		wire        s2_p_st    = |s2_xw[11:0];
		wire        s2_p_up    = s2_xw[12] && (s2_p_st || s2_xw[13]);
		wire        s2_p_lo1   = &s2_xw[17:13];
		wire        s2_p_hi1   = &s2_xw[22:18];
		wire        s2_p_carry = s2_p_lo1 && s2_p_hi1 && s2_p_up;
		wire [9:0]  s2_p_mp1;
		wire [4:0]  s2_p_ep1;
		wire [1:0]  s2_p_inc_co_unused;
		am_add #(.W(10)) u_s2_p_mp1 (.a(s2_xw[22:13]), .b(10'd1), .ci(1'b0), .s(s2_p_mp1), .co(s2_p_inc_co_unused[0]));
		am_add #(.W(5))  u_s2_p_ep1 (.a(s2_er_p[4:0]), .b(5'd1),  .ci(1'b0), .s(s2_p_ep1), .co(s2_p_inc_co_unused[1]));
		wire [9:0]  s2_p_mr    = s2_p_carry ? 10'd0 : (s2_p_up ? s2_p_mp1 : s2_xw[22:13]);
		wire [4:0]  s2_p_exp   = s2_p_carry ? s2_p_ep1 : s2_er_p[4:0];
		wire        s2_p_ovf0  = !s2_er_p[9] && ((|s2_er_p[8:5]) || (&s2_er_p[4:0]));
		wire        s2_p_ovf1  = !s2_er_p[9] && ((|s2_er_p[8:5]) || (&s2_er_p[4:1]));
		wire        s2_p_udf0  = s2_er_p[9] || (s2_er_p[8:0] == 9'd0);
		wire        s2_p_udf1  = s2_er_p[9];
		wire        s2_p_ovf   = s2_p_carry ? s2_p_ovf1 : s2_p_ovf0;
		wire        s2_p_udf   = s2_p_carry ? s2_p_udf1 : s2_p_udf0;
		wire [15:0] s2_hr_p    = s2_p_ovf ? {s2_xw[31], 5'h1F, 10'd0} :
		                         s2_p_udf ? {s2_xw[31], 15'd0} : {s2_xw[31], s2_p_exp, s2_p_mr};
		wire [15:0] s2_xh      = (s2_xw[30:23] == 8'hFF) ? ((s2_xw[22:0] != 23'd0) ? 16'h7E00 : {s2_xw[31], 5'h1F, 10'd0}) :
		                         (s2_xw[30:23] == 8'd0)  ? {s2_xw[31], 15'd0} : s2_hr_p;
		wire [15:0] s2_res     = s2_ovr_en ? s2_ovr_val : s2_packh ? s2_xh : 16'd0;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				s1_valid   <= 1'b0;
				s2_valid   <= 1'b0;
				s3_valid   <= 1'b0;
				s4_valid   <= 1'b0;
				s5_valid   <= 1'b0;
				s6_valid   <= 1'b0;
				s6_res     <= 16'd0;
			end else begin
				s1_valid <= valid_in;
				s2_valid <= s1_valid;
				s3_valid <= s2_valid;
				s4_valid <= s3_valid;
				s5_valid <= s4_valid;
				s6_valid <= s5_valid;
				s6_res   <= s5_valid ? s5_res : 16'd0;
			end
		end
		always @(posedge clk) begin
			if (valid_in) begin
				s1_ovr_en  <= ovr_en;
				s1_ovr_val <= ovr_val;
				s1_packh   <= (o == ATG_PK_PACKH[3:0]);
				s1_xw      <= xw;
			end
			if (s1_valid) begin
				s2_ovr_en  <= s1_ovr_en;
				s2_ovr_val <= s1_ovr_val;
				s2_packh   <= s1_packh;
				s2_xw      <= s1_xw;
				s2_er_p    <= s1_er_p;
			end
			if (s2_valid)
				s3_res <= s2_res;
			if (s3_valid)
				s4_res <= s3_res;
			if (s4_valid)
				s5_res <= s4_res;
		end

		assign y = s6_res;
		wire unused_noarith = &{1'b0, ha, hb, hc, r, sat, o, s6_valid, s1_erp_bo_unused};
	end else begin : g_lat6
		wire rst_p = !rst_n;
		/* =====================================================================
		 * Stage 1 (EX1): Latch inputs and dedicated multiplier operands (0 Pre-DSP LUTs)
		 * ===================================================================== */
		wire [3:0] o_mux_y, o_mux_z, o_is_int_a, o_is_int_b, o_pipe;
		genvar go;
		for (go = 0; go < 4; go = go + 1) begin : g_o_buf
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bo_y  (.I0(o[go]), .O(o_mux_y[go]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bo_z  (.I0(o[go]), .O(o_mux_z[go]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bo_ia (.I0(o[go]), .O(o_is_int_a[go]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bo_ib (.I0(o[go]), .O(o_is_int_b[go]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_bo_p  (.I0(o[go]), .O(o_pipe[go]));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign o_mux_y[go] = o[go];
			assign o_mux_z[go] = o[go];
			assign o_is_int_a[go] = o[go];
			assign o_is_int_b[go] = o[go];
			assign o_pipe[go] = o[go];
`else
			assign o_mux_y[go]    = o[go];
			assign o_mux_z[go]    = o[go];
			assign o_is_int_a[go] = o[go];
			assign o_is_int_b[go] = o[go];
			assign o_pipe[go]     = o[go];
`endif
		end

		wire s0_is_int_a = o_is_int_a == ATG_PK_IADD2[3:0] || o_is_int_a == ATG_PK_ISUB2[3:0] || o_is_int_a == ATG_PK_IMUL2[3:0];
		wire s0_is_int_b = o_is_int_b == ATG_PK_IADD2[3:0] || o_is_int_b == ATG_PK_ISUB2[3:0] || o_is_int_b == ATG_PK_IMUL2[3:0];
		(* max_fanout = 12 *) wire s0_use_hb = (o_mux_y == ATG_PK_FMUL2[3:0] || o_mux_y == ATG_PK_FFMA2[3:0]);
		wire s0_fnoc = (o_mux_y == ATG_PK_FMUL2[3:0]);
		wire s0_z_hc = (o_mux_z == ATG_PK_FFMA2[3:0]);

		wire [15:0] s0_fa = atg_su_hflush(ha);
		wire [15:0] s0_fb = s0_use_hb ? hb : 16'h3C00;
		wire [15:0] s0_fc = (o_mux_z == ATG_PK_FFMA2[3:0]) ? hc :
		                    (o_mux_z == ATG_PK_FSUB2[3:0]) ? {!hb[15], hb[14:0]} :
		                    (o_mux_z == ATG_PK_FMUL2[3:0]) ? 16'd0 : hb;
		wire ha_ez  = ha[14:10] == 5'd0;
		wire ha_e1f = ha[14:10] == 5'h1F;
		wire ha_mnz = ha[9:0]   != 10'd0;
		wire hb_ez  = hb[14:10] == 5'd0;
		wire hb_e1f = hb[14:10] == 5'h1F;
		wire hb_mnz = hb[9:0]   != 10'd0;
		wire hc_ez  = hc[14:10] == 5'd0;
		wire hc_e1f = hc[14:10] == 5'h1F;
		wire hc_mnz = hc[9:0]   != 10'd0;
		wire        s0_nan_a = ha_e1f && ha_mnz;
		wire        s0_nan_b = s0_use_hb && hb_e1f && hb_mnz;
		wire        s0_nan_c = !s0_fnoc && (s0_z_hc ? (hc_e1f && hc_mnz) : (hb_e1f && hb_mnz));
		wire        s0_inf_a = ha_e1f && !ha_mnz;
		wire        s0_inf_b = s0_use_hb && hb_e1f && !hb_mnz;
		wire        s0_inf_c = !s0_fnoc && (s0_z_hc ? (hc_e1f && !hc_mnz) : (hb_e1f && !hb_mnz));
		wire        s0_za_f  = ha_ez;
		wire        s0_zb_f  = s0_use_hb && hb_ez;
		wire        s0_zc_f  = s0_fnoc || (s0_z_hc ? hc_ez : hb_ez);
		wire [10:0] s0_ma = {!ha_ez, ha_ez ? 10'd0 : ha[9:0]};
		wire [10:0] s0_mb = {!s0_zb_f, (s0_use_hb && !hb_ez) ? hb[9:0] : 10'd0};

		(* shreg_extract = "no" *) reg        s1_valid;
		(* shreg_extract = "no" *) reg        s1_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s1_ovr_val;
		(* shreg_extract = "no" *) reg [3:0]  s1_o;
		(* shreg_extract = "no" *) reg        s1_is_isub;
		(* shreg_extract = "no" *) reg [15:0] s1_ha, s1_hb, s1_hc;
		(* shreg_extract = "no" *) reg [15:0] s1_fa, s1_fb, s1_fc;
		(* shreg_extract = "no" *) reg        s1_fnoc;
		(* shreg_extract = "no" *) reg        s1_nan_a, s1_nan_b, s1_nan_c;
		(* shreg_extract = "no" *) reg        s1_inf_a, s1_inf_b, s1_inf_c;
		(* shreg_extract = "no" *) reg        s1_za, s1_zb, s1_zc;
		(* shreg_extract = "no" *) reg [2:0]  s1_r;
		(* shreg_extract = "no" *) reg        s1_sat;
		(* shreg_extract = "no" *) reg [31:0] s1_xw;
		(* shreg_extract = "no", keep = "true" *) reg [15:0] s1_mul_a;
		(* shreg_extract = "no", keep = "true" *) reg [15:0] s1_mul_b;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p)
				s1_valid <= 1'b0;
			else
				s1_valid <= valid_in;
		end
		always @(posedge clk) begin
			s1_ovr_en  <= ovr_en;
			s1_ovr_val <= ovr_val;
			s1_o       <= o_pipe;
			s1_is_isub <= (o_pipe == ATG_PK_ISUB2[3:0]);
			s1_ha      <= ha;
			s1_hb      <= hb;
			s1_hc      <= hc;
			s1_fa      <= s0_fa;
			s1_fb      <= s0_fb;
			s1_fc      <= s0_fc;
			s1_fnoc    <= s0_fnoc;
			s1_nan_a   <= s0_nan_a; s1_nan_b <= s0_nan_b; s1_nan_c <= s0_nan_c;
			s1_inf_a   <= s0_inf_a; s1_inf_b <= s0_inf_b; s1_inf_c <= s0_inf_c;
			s1_za      <= s0_za_f;  s1_zb    <= s0_zb_f;  s1_zc    <= s0_zc_f;
			s1_r       <= r;
			s1_sat     <= sat;
			s1_xw      <= xw;
			s1_mul_a   <= s0_is_int_a ? ha : {5'd0, s0_ma};
			s1_mul_b   <= s0_is_int_b ? hb : {5'd0, s0_mb};
		end

		/* 16x16 Multiplier: LAT=3 CSA-split under SYNTH_ASIC, LAT=2 + fabric s4_prod under Kintex-7 */
		wire [31:0] prod_mul;
`ifdef SYNTH_ASIC
		am_mul #(.AW(16), .BW(16), .LAT(3)) u_mul (
			.clk(clk), .en(1'b1), .a(s1_mul_a), .b(s1_mul_b), .p(prod_mul)
		);
		wire [31:0] s4_prod = prod_mul;
`else
		am_mul #(.AW(16), .BW(16), .LAT(2)) u_mul (
			.clk(clk), .en(1'b1), .a(s1_mul_a), .b(s1_mul_b), .p(prod_mul)
		);

		/* Stage 4 fabric register directly after DSP PREG (0 Post-DSP LUTs) */
		(* dont_touch = "true" *) reg [31:0] s4_prod;
		always @(posedge clk)
			s4_prod <= prod_mul;
`endif

		/* =====================================================================
		 * Stage 1 -> Stage 2: Parallel small adders & operand decode
		 * ===================================================================== */
		wire [4:0]  s1_ea = s1_fa[14:10];
		wire [4:0]  s1_eb = s1_fb[14:10];
		wire [4:0]  s1_ec = s1_fc[14:10];
		wire        s1_sp = s1_fa[15] ^ s1_fb[15];
		wire        s1_sc = s1_fc[15];
		wire        s1_rdn = s1_r == 3'd2;
		wire [10:0] s1_mc = s1_zc ? 11'd0 : {1'b1, s1_fc[9:0]};

		wire        s1_spec_nan = s1_nan_a || s1_nan_b || s1_nan_c || (s1_inf_a && s1_zb) || (s1_inf_b && s1_za);
		wire        s1_spec_inf_ab = (s1_inf_a || s1_inf_b) && !s1_spec_nan;
		wire        s1_spec_inf_c = s1_inf_c && !s1_spec_nan && !s1_spec_inf_ab;
		wire        s1_spec_zero = (s1_za || s1_zb) && !s1_spec_nan && !s1_spec_inf_ab && !s1_spec_inf_c;
		wire        s1_spec_hit = s1_spec_nan || s1_spec_inf_ab || s1_spec_inf_c || s1_spec_zero;

		wire s1_spec_nan_lo, s1_spec_nan_hi;
		wire s1_spec_inf_ab_lo, s1_spec_inf_ab_hi;
		wire s1_spec_inf_c_lo, s1_spec_inf_c_hi;
		wire s1_zc_lo, s1_zc_hi;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_snan_l (.I0(s1_spec_nan),    .O(s1_spec_nan_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_snan_h (.I0(s1_spec_nan),    .O(s1_spec_nan_hi));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sinf_l (.I0(s1_spec_inf_ab), .O(s1_spec_inf_ab_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sinf_h (.I0(s1_spec_inf_ab), .O(s1_spec_inf_ab_hi));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sc_l   (.I0(s1_spec_inf_c),  .O(s1_spec_inf_c_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_sc_h   (.I0(s1_spec_inf_c),  .O(s1_spec_inf_c_hi));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_zc_l   (.I0(s1_zc),          .O(s1_zc_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_zc_h   (.I0(s1_zc),          .O(s1_zc_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign s1_spec_nan_lo = s1_spec_nan;
		assign s1_spec_nan_hi = s1_spec_nan;
		assign s1_spec_inf_ab_lo = s1_spec_inf_ab;
		assign s1_spec_inf_ab_hi = s1_spec_inf_ab;
		assign s1_spec_inf_c_lo = s1_spec_inf_c;
		assign s1_spec_inf_c_hi = s1_spec_inf_c;
		assign s1_zc_lo = s1_zc;
		assign s1_zc_hi = s1_zc;
`else
		assign s1_spec_nan_lo    = s1_spec_nan;
		assign s1_spec_nan_hi    = s1_spec_nan;
		assign s1_spec_inf_ab_lo = s1_spec_inf_ab;
		assign s1_spec_inf_ab_hi = s1_spec_inf_ab;
		assign s1_spec_inf_c_lo  = s1_spec_inf_c;
		assign s1_spec_inf_c_hi  = s1_spec_inf_c;
		assign s1_zc_lo          = s1_zc;
		assign s1_zc_hi          = s1_zc;
`endif
		wire [7:0] s1_spec_res_lo =
			s1_spec_nan_lo ? 8'h00 :
			s1_spec_inf_ab_lo ? 8'h00 :
			s1_spec_inf_c_lo ? s1_fc[7:0] :
			(!s1_zc_lo ? s1_fc[7:0] : 8'd0);

		wire [7:0] s1_spec_res_hi =
			s1_spec_nan_hi ? 8'h7E :
			s1_spec_inf_ab_hi ? (s1_inf_c && s1_sc != s1_sp ? 8'h7E : {s1_sp, 5'h1F, 2'd0}) :
			s1_spec_inf_c_hi ? s1_fc[15:8] :
			(!s1_zc_hi ? s1_fc[15:8] : s1_fnoc ? {s1_sp, 7'd0} : {s1_rdn ? (s1_sp | s1_sc) : (s1_sp & s1_sc), 7'd0});

		wire [15:0] s1_spec_res = {s1_spec_res_hi, s1_spec_res_lo};

		wire [9:0] s1_t, s1_ec_p25, s1_ec_p37, s1_ec_p2, s1_er_p;
		wire [15:0] s1_isum;
		wire [6:0] s1_co_unused;
		am_add #(.W(10)) u_s1_t   (.a({5'd0, s1_ea}), .b({5'd0, s1_eb}), .ci(1'b0), .s(s1_t),      .co(s1_co_unused[0]));
		am_add #(.W(10)) u_s1_c25 (.a({5'd0, s1_ec}), .b(10'd25),        .ci(1'b0), .s(s1_ec_p25), .co(s1_co_unused[1]));
		am_add #(.W(10)) u_s1_c37 (.a({5'd0, s1_ec}), .b(10'd37),        .ci(1'b0), .s(s1_ec_p37), .co(s1_co_unused[2]));
		am_add #(.W(10)) u_s1_c2  (.a({5'd0, s1_ec}), .b(10'd2),         .ci(1'b0), .s(s1_ec_p2),  .co(s1_co_unused[3]));
		am_addsub #(.W(16)) u_s1_iadd (.a(s1_ha), .b(s1_hb), .sub(s1_is_isub), .s(s1_isum), .co(s1_co_unused[4]));
		am_sub #(.W(10)) u_s1_erp (.a({2'd0, s1_xw[30:23]}), .b(10'd112), .bi(1'b0), .d(s1_er_p), .bo(s1_co_unused[5]));

		/* Precompute PACKH FP32->FP16 rounding & special checks in Stage 1 so s2_xw -> s3_xh <= 8 cmos3. */
		wire        s1_p_st    = |s1_xw[11:0];
		wire        s1_p_up    = s1_xw[12] && (s1_p_st || s1_xw[13]);
		wire        s1_p_carry = (&s1_xw[17:13]) && (&s1_xw[22:18]) && s1_p_up;
		wire [9:0]  s1_p_mp1;
		am_add #(.W(10)) u_s1_p_mp1 (.a(s1_xw[22:13]), .b(10'd1), .ci(1'b0), .s(s1_p_mp1), .co(s1_co_unused[6]));
		wire        s1_xw_ff   = (s1_xw[30:23] == 8'hFF);
		wire        s1_xw_z    = (s1_xw[30:23] == 8'd0);
		wire        s1_xw_nz   = (s1_xw[22:0] != 23'd0);

		wire [15:0] s1_haf = s1_fa;
		wire [15:0] s1_hbf = atg_su_hflush(s1_hb);
		wire        s1_hb_z = s1_hb[14:10] == 5'd0;
		wire        s1_hb_nan = s1_hb[14:10] == 5'h1F && s1_hb[9:0] != 10'd0;
		wire [14:0] s1_ab_diff_unused, s1_ba_diff_unused;
		wire        s1_ab_lt_raw, s1_ba_lt_raw;
		am_sub #(.W(15)) u_s1_ab_lt (.a(s1_ha[14:0]), .b(s1_hb[14:0]), .bi(1'b0), .d(s1_ab_diff_unused), .bo(s1_ab_lt_raw));
		am_sub #(.W(15)) u_s1_ba_lt (.a(s1_hb[14:0]), .b(s1_ha[14:0]), .bi(1'b0), .d(s1_ba_diff_unused), .bo(s1_ba_lt_raw));

		/* --- Stage 2 registers --- */
		(* shreg_extract = "no" *) reg        s2_valid;
		(* shreg_extract = "no" *) reg        s2_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s2_ovr_val;
		(* shreg_extract = "no" *) reg [3:0]  s2_o;
		(* shreg_extract = "no" *) reg [2:0]  s2_r;
		(* shreg_extract = "no" *) reg        s2_sat;
		(* shreg_extract = "no" *) reg [31:0] s2_xw;
		(* shreg_extract = "no" *) reg        s2_p_up_r, s2_p_carry_r, s2_xw_ff_r, s2_xw_z_r, s2_xw_nz_r;
		(* shreg_extract = "no" *) reg [9:0]  s2_p_mp1_r;
		(* shreg_extract = "no" *) reg [10:0] s2_mc;
		(* shreg_extract = "no" *) reg        s2_sp, s2_sc, s2_rdn, s2_zc;
		(* shreg_extract = "no" *) reg        s2_spec_hit;
		(* shreg_extract = "no" *) reg [15:0] s2_spec_res;
		(* shreg_extract = "no" *) reg [9:0]  s2_t, s2_ec_p25, s2_ec_p37, s2_ec_p2, s2_er_p;
		(* shreg_extract = "no" *) reg [15:0] s2_isum;
		(* shreg_extract = "no" *) reg [15:0] s2_haf, s2_hbf;
		(* shreg_extract = "no" *) reg        s2_za, s2_hb_z, s2_nan_a;
		(* shreg_extract = "no", max_fanout = 12 *) reg s2_hb_nan;
		(* shreg_extract = "no" *) reg        s2_ab_lt_raw, s2_ba_lt_raw;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p)
				s2_valid <= 1'b0;
			else
				s2_valid <= s1_valid;
		end
		always @(posedge clk) begin
			s2_ovr_en    <= s1_ovr_en;
			s2_ovr_val   <= s1_ovr_val;
			s2_o         <= s1_o;
			s2_r         <= s1_r;
			s2_sat       <= s1_sat;
			s2_xw        <= s1_xw;
			s2_p_up_r    <= s1_p_up;
			s2_p_carry_r <= s1_p_carry;
			s2_p_mp1_r   <= s1_p_mp1;
			s2_xw_ff_r   <= s1_xw_ff;
			s2_xw_z_r    <= s1_xw_z;
			s2_xw_nz_r   <= s1_xw_nz;
			s2_mc        <= s1_mc;
			s2_sp        <= s1_sp;
			s2_sc        <= s1_sc;
			s2_rdn       <= s1_rdn;
			s2_zc        <= s1_zc;
			s2_spec_hit  <= s1_spec_hit;
			s2_spec_res  <= s1_spec_res;
			s2_t         <= s1_t;
			s2_ec_p25    <= s1_ec_p25;
			s2_ec_p37    <= s1_ec_p37;
			s2_ec_p2     <= s1_ec_p2;
			s2_er_p      <= s1_er_p;
			s2_isum      <= s1_isum;
			s2_haf       <= s1_haf;
			s2_hbf       <= s1_hbf;
			s2_za        <= s1_za;
			s2_hb_z      <= s1_hb_z;
			s2_nan_a     <= s1_nan_a;
			s2_hb_nan    <= s1_hb_nan;
			s2_ab_lt_raw <= s1_ab_lt_raw;
			s2_ba_lt_raw <= s1_ba_lt_raw;
		end

		/* =====================================================================
		 * Stage 2 -> Stage 3: Exponent differences, packh rounding, & minmax/cmp
		 * ===================================================================== */
		wire s2_both_z = s2_za && s2_hb_z;
		wire s2_mag_lt = s2_ab_lt_raw && !s2_both_z;
		wire s2_mag_gt = s2_ba_lt_raw && !s2_both_z;
		wire s2_mag_eq = !s2_mag_lt && !s2_mag_gt;
		wire s2_lt_ab  = (s2_haf[15] && !s2_hbf[15]) ||
		                 (!s2_haf[15] && !s2_hbf[15] && s2_mag_lt) ||
		                 (s2_haf[15] && s2_hbf[15] && s2_mag_gt);
		wire s2_lt_ba  = (s2_hbf[15] && !s2_haf[15]) ||
		                 (!s2_hbf[15] && !s2_haf[15] && s2_mag_gt) ||
		                 (s2_hbf[15] && s2_haf[15] && s2_mag_lt);
		wire s2_is_max = (s2_o == ATG_PK_FMAX2[3:0]);
		wire [15:0] s2_fminmax = (s2_nan_a && s2_hb_nan) ? 16'h7E00 :
		                         s2_nan_a ? s2_hbf :
		                         s2_hb_nan ? s2_haf :
		                         s2_is_max ? (s2_lt_ab ? s2_hbf : s2_haf) :
		                                     (s2_lt_ba ? s2_hbf : s2_haf);
		wire s2_un = s2_nan_a || s2_hb_nan;
		wire s2_eq = s2_both_z || ((s2_haf[15] == s2_hbf[15]) && s2_mag_eq);
		wire s2_lt = !s2_eq && s2_lt_ab;
		wire s2_gt = !s2_eq && s2_lt_ba;
		reg  s2_cmp_bit;
		always @(*) begin
			case (s2_o)
			ATG_PK_FCMPLT2[3:0]: s2_cmp_bit = !s2_un && s2_lt;
			ATG_PK_FCMPGE2[3:0]: s2_cmp_bit = !s2_un && (s2_gt || s2_eq);
			ATG_PK_FCMPEQ2[3:0]: s2_cmp_bit = !s2_un && s2_eq;
			default:             s2_cmp_bit = s2_un || !s2_eq;
			endcase
		end
		wire [15:0] s2_fcmp = {16{s2_cmp_bit}};
		wire [9:0] s2_k, s2_dp37, s2_eb_c, s2_eb_t;
		wire [3:0] s2_bo_unused;
		am_sub #(.W(10)) u_s2_k    (.a(s2_ec_p25), .b(s2_t),     .bi(1'b0), .d(s2_k),    .bo(s2_bo_unused[0]));
		am_sub #(.W(10)) u_s2_dp37 (.a(s2_ec_p37), .b(s2_t),     .bi(1'b0), .d(s2_dp37), .bo(s2_bo_unused[1]));
		am_sub #(.W(10)) u_s2_ebc  (.a(s2_ec_p2),  .b(10'd37),   .bi(1'b0), .d(s2_eb_c), .bo(s2_bo_unused[2]));
		am_sub #(.W(10)) u_s2_ebt  (.a(s2_t),      .b(10'd37),   .bi(1'b0), .d(s2_eb_t), .bo(s2_bo_unused[3]));

		/* Pure-LUT FP32 -> FP16 RNE rounder (0 CARRY4s on mantissa) */
		wire [4:0]  s2_p_ep1;
		wire        s2_p_inc_co_unused;
		am_add #(.W(5))  u_s2_p_ep1 (.a(s2_er_p[4:0]), .b(5'd1),  .ci(1'b0), .s(s2_p_ep1), .co(s2_p_inc_co_unused));
		wire [9:0]  s2_p_mr    = s2_p_carry_r ? 10'd0 : (s2_p_up_r ? s2_p_mp1_r : s2_xw[22:13]);
		wire [4:0]  s2_p_exp   = s2_p_carry_r ? s2_p_ep1 : s2_er_p[4:0];
		wire        s2_p_ovf0  = !s2_er_p[9] && ((|s2_er_p[8:5]) || (&s2_er_p[4:0]));
		wire        s2_p_ovf1  = !s2_er_p[9] && ((|s2_er_p[8:5]) || (&s2_er_p[4:1]));
		wire        s2_p_udf0  = s2_er_p[9] || (s2_er_p[8:0] == 9'd0);
		wire        s2_p_udf1  = s2_er_p[9];
		wire        s2_p_ovf   = s2_p_carry_r ? s2_p_ovf1 : s2_p_ovf0;
		wire        s2_p_udf   = s2_p_carry_r ? s2_p_udf1 : s2_p_udf0;
		wire [15:0] s2_hr_p    = s2_p_ovf ? {s2_xw[31], 5'h1F, 10'd0} :
		                         s2_p_udf ? {s2_xw[31], 15'd0} : {s2_xw[31], s2_p_exp, s2_p_mr};
		wire [15:0] s2_xh      = s2_xw_ff_r ? (s2_xw_nz_r ? 16'h7E00 : {s2_xw[31], 5'h1F, 10'd0}) :
		                         s2_xw_z_r  ? {s2_xw[31], 15'd0} : s2_hr_p;

		/* Precompute Stage 3 0..3 shift in Stage 2 so s3_dp37 -> s4_cw <= 11 cmos3. */
		reg [13:0] s2_al_s1_w;
		always @(*) begin
			case (s2_dp37[1:0])
			2'd0: s2_al_s1_w = {3'd0, s2_mc};
			2'd1: s2_al_s1_w = {2'd0, s2_mc, 1'b0};
			2'd2: s2_al_s1_w = {1'd0, s2_mc, 2'd0};
			2'd3: s2_al_s1_w = {s2_mc, 3'd0};
			endcase
		end

		/* --- Stage 3 registers --- */
		(* shreg_extract = "no" *) reg        s3_valid;
		(* shreg_extract = "no" *) reg        s3_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s3_ovr_val;
		(* dont_touch = "true" *) reg [3:0]  s3_o_pipe, s3_o_mux_lo, s3_o_mux_hi;
		(* shreg_extract = "no" *) reg [2:0]  s3_r;
		(* shreg_extract = "no" *) reg        s3_sat;
		(* shreg_extract = "no" *) reg [13:0] s3_al_s1_r;
		(* dont_touch = "true" *) reg [10:0] s3_mc_cw;
		(* shreg_extract = "no" *) reg        s3_sp, s3_sc, s3_rdn, s3_zc;
		(* shreg_extract = "no" *) reg        s3_spec_hit;
		(* shreg_extract = "no" *) reg [15:0] s3_spec_res;
		(* shreg_extract = "no" *) reg [9:0]  s3_k, s3_dp37;
		(* shreg_extract = "no" *) reg [6:0]  s3_eb_c, s3_eb_t;
		(* shreg_extract = "no" *) reg [15:0] s3_xh, s3_isum, s3_fminmax, s3_fcmp;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p)
				s3_valid <= 1'b0;
			else
				s3_valid <= s2_valid;
		end
		always @(posedge clk) begin
			s3_ovr_en   <= s2_ovr_en;
			s3_ovr_val  <= s2_ovr_val;
			s3_o_pipe   <= s2_o;
			s3_o_mux_lo <= s2_o;
			s3_o_mux_hi <= s2_o;
			s3_r        <= s2_r;
			s3_sat      <= s2_sat;
			s3_al_s1_r  <= s2_al_s1_w;
			s3_mc_cw    <= s2_mc;
			s3_sp       <= s2_sp;
			s3_sc       <= s2_sc;
			s3_rdn      <= s2_rdn;
			s3_zc       <= s2_zc;
			s3_spec_hit <= s2_spec_hit;
			s3_spec_res <= s2_spec_res;
			s3_k        <= s2_k;
			s3_dp37     <= s2_dp37;
			s3_eb_c     <= s2_eb_c[6:0];
			s3_eb_t     <= s2_eb_t[6:0];
			s3_xh       <= s2_xh;
			s3_isum     <= s2_isum;
			s3_fminmax  <= s2_fminmax;
			s3_fcmp     <= s2_fcmp;
		end

		/* =====================================================================
		 * Stage 3 -> Stage 4: Addend shift (u_al) & non-mul result pre-mux
		 * ===================================================================== */
		wire s3_big  = $signed(s3_k) > 10'sd23;
		wire s3_near = $signed(s3_k) >= -10'sd12;
		/* Constant 2-stage left-shift from registered s3_al_s1_r by s3_dp37[5:2] (keeps FO <= 14 per bit). */
		wire [13:0] s3_al_s1 = s3_al_s1_r;
		wire [1:0] s3_dp37_23_lo, s3_dp37_23_hi;
		wire [1:0] s3_dp37_45_s0, s3_dp37_45_s1, s3_dp37_45_s2, s3_dp37_45_s3;
`ifdef SYNTH_KINTEX7
		genvar gshb;
		for (gshb = 0; gshb < 2; gshb = gshb + 1) begin : g_sh_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp23l (.I0(s3_dp37[2 + gshb]), .O(s3_dp37_23_lo[gshb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp23h (.I0(s3_dp37[2 + gshb]), .O(s3_dp37_23_hi[gshb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp450 (.I0(s3_dp37[4 + gshb]), .O(s3_dp37_45_s0[gshb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp451 (.I0(s3_dp37[4 + gshb]), .O(s3_dp37_45_s1[gshb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp452 (.I0(s3_dp37[4 + gshb]), .O(s3_dp37_45_s2[gshb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_dp453 (.I0(s3_dp37[4 + gshb]), .O(s3_dp37_45_s3[gshb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		genvar gshb;
		for (gshb = 0; gshb < 2; gshb = gshb + 1) begin : g_sh_buf
			assign s3_dp37_23_lo[gshb] = s3_dp37[2 + gshb];
			assign s3_dp37_23_hi[gshb] = s3_dp37[2 + gshb];
			assign s3_dp37_45_s0[gshb] = s3_dp37[4 + gshb];
			assign s3_dp37_45_s1[gshb] = s3_dp37[4 + gshb];
			assign s3_dp37_45_s2[gshb] = s3_dp37[4 + gshb];
			assign s3_dp37_45_s3[gshb] = s3_dp37[4 + gshb];
		end
`else
		assign s3_dp37_23_lo = s3_dp37[3:2];
		assign s3_dp37_23_hi = s3_dp37[3:2];
		assign s3_dp37_45_s0 = s3_dp37[5:4];
		assign s3_dp37_45_s1 = s3_dp37[5:4];
		assign s3_dp37_45_s2 = s3_dp37[5:4];
		assign s3_dp37_45_s3 = s3_dp37[5:4];
`endif
		wire [25:0] s3_al_s2_0 = {12'd0, s3_al_s1};
		wire [25:0] s3_al_s2_1 = {8'd0,  s3_al_s1, 4'd0};
		wire [25:0] s3_al_s2_2 = {4'd0,  s3_al_s1, 8'd0};
		wire [25:0] s3_al_s2_3 = {s3_al_s1, 12'd0};
		wire [12:0] s3_al_s2_lo =
			(s3_dp37_23_lo == 2'd0) ? s3_al_s2_0[12:0] :
			(s3_dp37_23_lo == 2'd1) ? s3_al_s2_1[12:0] :
			(s3_dp37_23_lo == 2'd2) ? s3_al_s2_2[12:0] : s3_al_s2_3[12:0];
		wire [12:0] s3_al_s2_hi =
			(s3_dp37_23_hi == 2'd0) ? s3_al_s2_0[25:13] :
			(s3_dp37_23_hi == 2'd1) ? s3_al_s2_1[25:13] :
			(s3_dp37_23_hi == 2'd2) ? s3_al_s2_2[25:13] : s3_al_s2_3[25:13];
		wire [25:0] s3_al_s2 = {s3_al_s2_hi, s3_al_s2_lo};
		wire [46:0] s3_al_z0 = {21'd0, s3_al_s2};
		wire [46:0] s3_al_z1 = {5'd0,  s3_al_s2, 16'd0};
		wire [46:0] s3_al_z2 = {s3_al_s2[14:0], 32'd0};
		wire [11:0] s3_zal_0 = (s3_dp37_45_s0 == 2'd0) ? s3_al_z0[11:0]  : 12'd0;
		wire [11:0] s3_zal_1 = (s3_dp37_45_s1 == 2'd0) ? s3_al_z0[23:12] : (s3_dp37_45_s1 == 2'd1) ? s3_al_z1[23:12] : 12'd0;
		wire [11:0] s3_zal_2 = (s3_dp37_45_s2 == 2'd0) ? s3_al_z0[35:24] : (s3_dp37_45_s2 == 2'd1) ? s3_al_z1[35:24] : (s3_dp37_45_s2 == 2'd2) ? s3_al_z2[35:24] : 12'd0;
		wire [10:0] s3_zal_3 = (s3_dp37_45_s3 == 2'd1) ? s3_al_z1[46:36] : (s3_dp37_45_s3 == 2'd2) ? s3_al_z2[46:36] : 11'd0;
		wire [46:0] s3_zal = {s3_zal_3, s3_zal_2, s3_zal_1, s3_zal_0};
		wire s3_eff_sub_d = !(s3_zc || (s3_sp == s3_sc));
		wire s3_pw_sel_d  = !s3_zc && s3_big;
		/* Precompute sticky bit of s3_zal[10:0] in parallel with s3_dp37[5:2] so s3_dp37 -> s4_cw[0] <= 9 cmos3. */
		wire s3_al_st0 = (s3_dp37_23_lo == 2'd0) ? (|s3_al_s1[10:0]) :
		                 (s3_dp37_23_lo == 2'd1) ? (|s3_al_s1[6:0])  :
		                 (s3_dp37_23_lo == 2'd2) ? (|s3_al_s1[2:0])  : 1'b0;
		wire s3_zal_cw0 = (s3_dp37_45_s0 == 2'd0) && s3_al_st0;
		wire [36:0] s3_zal_cw = {s3_zal[46:11], s3_zal_cw0};
		wire [36:0] s3_big_cw = {1'b0, s3_mc_cw, 25'd0};
		wire [6:0]  s3_eb = (!s3_zc && s3_big) ? s3_eb_c : s3_eb_t;

		reg [7:0] s3_other_lo, s3_other_hi;
		always @(*) begin
			case (s3_o_mux_lo)
			ATG_PK_PACKH[3:0]: s3_other_lo = s3_xh[7:0];
			ATG_PK_FMIN2[3:0], ATG_PK_FMAX2[3:0]: s3_other_lo = FL ? s3_fminmax[7:0] : 8'd0;
			ATG_PK_IADD2[3:0], ATG_PK_ISUB2[3:0]: s3_other_lo = AR ? s3_isum[7:0] : 8'd0;
			ATG_PK_FCMPLT2[3:0], ATG_PK_FCMPGE2[3:0], ATG_PK_FCMPEQ2[3:0], ATG_PK_FCMPNE2[3:0]:
				s3_other_lo = FL ? s3_fcmp[7:0] : 8'd0;
			default: s3_other_lo = 8'd0;
			endcase
		end
		always @(*) begin
			case (s3_o_mux_hi)
			ATG_PK_PACKH[3:0]: s3_other_hi = s3_xh[15:8];
			ATG_PK_FMIN2[3:0], ATG_PK_FMAX2[3:0]: s3_other_hi = FL ? s3_fminmax[15:8] : 8'd0;
			ATG_PK_IADD2[3:0], ATG_PK_ISUB2[3:0]: s3_other_hi = AR ? s3_isum[15:8] : 8'd0;
			ATG_PK_FCMPLT2[3:0], ATG_PK_FCMPGE2[3:0], ATG_PK_FCMPEQ2[3:0], ATG_PK_FCMPNE2[3:0]:
				s3_other_hi = FL ? s3_fcmp[15:8] : 8'd0;
			default: s3_other_hi = 8'd0;
			endcase
		end
		wire [15:0] s3_other_pre = {s3_other_hi, s3_other_lo};

		/* --- Stage 4 registers (in sync with s4_prod) --- */
		(* shreg_extract = "no" *) reg        s4_valid;
		(* shreg_extract = "no" *) reg        s4_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s4_ovr_val;
		(* shreg_extract = "no" *) reg [3:0]  s4_o;
		(* shreg_extract = "no" *) reg [2:0]  s4_r;
		(* shreg_extract = "no" *) reg        s4_sat;
		(* shreg_extract = "no" *) reg [36:0] s4_cw;
		(* shreg_extract = "no" *) reg [6:0]  s4_eb;
		(* shreg_extract = "no" *) reg        s4_big, s4_sp, s4_sc, s4_rdn, s4_zc;
		(* dont_touch = "true", shreg_extract = "no" *) reg s4_eff_sub_p_r, s4_eff_sub_c0_r, s4_eff_sub_c1_r;
		(* dont_touch = "true", shreg_extract = "no" *) reg s4_eff_sub_hi0_r, s4_eff_sub_hi1_r, s4_eff_sub_ci_r;
		(* dont_touch = "true", shreg_extract = "no" *) reg s4_pw_sel_pc0_r, s4_pw_sel_pc0_b_r, s4_pw_sel_pc1_r;
		(* dont_touch = "true", shreg_extract = "no" *) reg s4_pw_sel_cp0_r, s4_pw_sel_cp0_b_r, s4_pw_sel_cp1_r;
		(* shreg_extract = "no" *) reg        s4_spec_hit;
		(* shreg_extract = "no" *) reg [15:0] s4_spec_res;
		(* shreg_extract = "no" *) reg [15:0] s4_other_pre;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p)
				s4_valid <= 1'b0;
			else
				s4_valid <= s3_valid;
		end
		always @(posedge clk) begin
			s4_ovr_en        <= s3_ovr_en;
			s4_ovr_val       <= s3_ovr_val;
			s4_o             <= s3_o_pipe;
			s4_r             <= s3_r;
			s4_sat           <= s3_sat;
			s4_eb            <= s3_eb;
			s4_big           <= s3_big;
			s4_sp            <= s3_sp;
			s4_sc            <= s3_sc;
			s4_rdn           <= s3_rdn;
			s4_zc            <= s3_zc;
			s4_eff_sub_p_r   <= s3_eff_sub_d;
			s4_eff_sub_c0_r  <= s3_eff_sub_d;
			s4_eff_sub_c1_r  <= s3_eff_sub_d;
			s4_eff_sub_hi0_r <= s3_eff_sub_d;
			s4_eff_sub_hi1_r <= s3_eff_sub_d;
			s4_eff_sub_ci_r  <= s3_eff_sub_d;
			s4_pw_sel_pc0_r  <= s3_pw_sel_d;
			s4_pw_sel_pc0_b_r<= s3_pw_sel_d;
			s4_pw_sel_pc1_r  <= s3_pw_sel_d;
			s4_pw_sel_cp0_r  <= s3_pw_sel_d;
			s4_pw_sel_cp0_b_r<= s3_pw_sel_d;
			s4_pw_sel_cp1_r  <= s3_pw_sel_d;
			s4_spec_hit      <= s3_spec_hit;
			s4_spec_res      <= s3_sat ? atg_su_hsat(s3_spec_res) : s3_spec_res;
			s4_other_pre     <= s3_other_pre;
		end
		genvar gcw4;
		for (gcw4 = 0; gcw4 < 3; gcw4 = gcw4 + 1) begin : g_s4_cw
			localparam CW4_W = (gcw4 == 2) ? 5 : 16;
			wire clr_s_raw = s3_zc;
			wire clr_s;
			wire s3_big_s, s3_near_s;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_clr (.I0(clr_s_raw),   .O(clr_s));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_bg  (.I0(s3_big),      .O(s3_big_s));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nr  (.I0(s3_near),     .O(s3_near_s));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign clr_s = clr_s_raw;
			assign s3_big_s = s3_big;
			assign s3_near_s = s3_near;
`else
			assign clr_s = clr_s_raw;
			assign s3_big_s = s3_big;
			assign s3_near_s = s3_near;
`endif
			wire [CW4_W-1:0] cw_in = s3_big_s ? s3_big_cw[gcw4*16 +: CW4_W] :
				s3_near_s ? s3_zal_cw[gcw4*16 +: CW4_W] :
				(gcw4 == 0 ? {{(CW4_W-1){1'b0}}, 1'b1} : {CW4_W{1'b0}});
			always @(posedge clk) begin
				if (clr_s)
					s4_cw[gcw4*16 +: CW4_W] <= {CW4_W{1'b0}};
				else
					s4_cw[gcw4*16 +: CW4_W] <= cw_in;
			end
		end

		/* =====================================================================
		 * Stage 4 -> Stage 5: Speculative parallel 37-bit FMA add/sub
		 * ===================================================================== */
		wire s4_eff_sub = s4_eff_sub_p_r;
		wire s4_pc0_lo, s4_pc0_hi, s4_pc1_lo, s4_pc1_hi;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pc0_lo (.I0(s4_pw_sel_pc0_r), .O(s4_pc0_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pc0_hi (.I0(s4_pw_sel_pc0_r), .O(s4_pc0_hi));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pc1_lo (.I0(s4_pw_sel_pc1_r), .O(s4_pc1_lo));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pc1_hi (.I0(s4_pw_sel_pc1_r), .O(s4_pc1_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign s4_pc0_lo = s4_pw_sel_pc0_r;
		assign s4_pc0_hi = s4_pw_sel_pc0_r;
		assign s4_pc1_lo = s4_pw_sel_pc1_r;
		assign s4_pc1_hi = s4_pw_sel_pc1_r;
`else
		assign s4_pc0_lo = s4_pw_sel_pc0_r;
		assign s4_pc0_hi = s4_pw_sel_pc0_r;
		assign s4_pc1_lo = s4_pw_sel_pc1_r;
		assign s4_pc1_hi = s4_pw_sel_pc1_r;
`endif
		wire [21:0] s4_prod_m_pc = {
			s4_pc1_hi ? 5'd0 : s4_prod[21:17],
			s4_pc1_lo ? 6'd0 : s4_prod[16:11],
			s4_pc0_hi ? 5'd0 : s4_prod[10:6],
			s4_pc0_lo ? 6'd0 : s4_prod[5:0]
		};
		wire [36:0] s4_pw_pc = {13'd0, s4_prod_m_pc, 1'b0, s4_pw_sel_pc0_b_r};

		wire [21:0] s4_prod_m_cp = {s4_pw_sel_cp1_r ? 11'd0 : s4_prod[21:11], s4_pw_sel_cp0_r ? 11'd0 : s4_prod[10:0]};
		wire [36:0] s4_pw_cp = {13'd0, s4_prod_m_cp, 1'b0, s4_pw_sel_cp0_b_r};

		wire [36:0] s4_sum_nxt;
		wire        s4_sr_nxt;
		wire        s4_bo_cp_unused;

		/* Precompute block exponent bases (+0, +16, +32) and their +1 upper bits */
		wire [2:0] s4_eb_hi0 = s4_eb[6:4];
		wire [2:0] s4_eb_hi1;
		wire [2:0] s4_eb_hi2 = {s4_eb[6] ^ s4_eb[5], ~s4_eb[5], s4_eb[4]};
		wire [2:0] s4_eb_hi3;
		wire [3:0] s4_eb_neg;
		wire [2:0] s4_eb_co_unused;
		am_add #(.W(3)) u_s4_eb_hi1 (.a(s4_eb[6:4]), .b(3'd1), .ci(1'b0), .s(s4_eb_hi1), .co(s4_eb_co_unused[0]));
		am_add #(.W(3)) u_s4_eb_hi3 (.a(s4_eb_hi2),  .b(3'd1), .ci(1'b0), .s(s4_eb_hi3), .co(s4_eb_co_unused[1]));
		am_sub #(.W(4)) u_s4_eb_neg (.a(4'd0),        .b(s4_eb[3:0]), .bi(1'b0), .d(s4_eb_neg), .bo(s4_eb_co_unused[2]));

		wire s4_is_fma = FL && (s4_o == ATG_PK_FADD2[3:0] || s4_o == ATG_PK_FSUB2[3:0] ||
		                        s4_o == ATG_PK_FMUL2[3:0] || s4_o == ATG_PK_FFMA2[3:0]);
		wire s4_is_fminmax = FL && (s4_o == ATG_PK_FMIN2[3:0] || s4_o == ATG_PK_FMAX2[3:0]);
		wire [15:0] s4_other_sat = (s4_is_fminmax && s4_sat) ? atg_su_hsat(s4_other_pre) : s4_other_pre;
		wire [15:0] s4_other = (s4_o == ATG_PK_IMUL2[3:0]) ? (AR ? s4_prod[15:0] : 16'd0) : s4_other_sat;

		/* --- Stage 5 registers --- */
		(* shreg_extract = "no" *) reg        s5_valid;
		(* shreg_extract = "no" *) reg        s5_ovr_en;
		(* shreg_extract = "no" *) reg [15:0] s5_ovr_val;
		(* shreg_extract = "no" *) reg        s5_is_fma;
		(* shreg_extract = "no" *) reg [2:0]  s5_r;
		(* shreg_extract = "no" *) reg        s5_sat;
		(* shreg_extract = "no" *) reg [36:0] s5_sum;
		(* shreg_extract = "no" *) reg        s5_sr, s5_rdn;
		(* shreg_extract = "no" *) reg [3:0]  s5_eb_lo, s5_eb_neg;
		(* shreg_extract = "no" *) reg [2:0]  s5_eb_hi0, s5_eb_hi1, s5_eb_hi2, s5_eb_hi3;
		(* shreg_extract = "no" *) reg        s5_spec_hit;
		(* shreg_extract = "no" *) reg [15:0] s5_spec_res;
		(* shreg_extract = "no" *) reg [15:0] s5_other;

		if (LAT >= 7) begin : g_s45_split
			wire [36:0] s4_cw_eff_w = {
				s4_eff_sub_hi1_r ? ~s4_cw[36:28] : s4_cw[36:28],
				s4_eff_sub_hi0_r ? ~s4_cw[27:19] : s4_cw[27:19],
				s4_eff_sub_c1_r  ? ~s4_cw[18:10] : s4_cw[18:10],
				s4_eff_sub_c0_r  ? ~s4_cw[9:0]   : s4_cw[9:0]
			};
			/* 10b + 9b carry-select deferred to Stage 4.5 so both Stage 4 and 4.5 stay <= 11 cmos3. */
			wire [9:0] s4_pc_l0, s4_cp_l0;
			wire [8:0] s4_pc_h0, s4_pc_h1, s4_cp_h0, s4_cp_h1;
			wire       s4_pc_c10, s4_cp_c10, s4_pc_ch0, s4_pc_ch1, s4_cp_ch0, s4_cp_ch1;
			am_add #(.W(10)) u_s4_pc_l0 (.a(s4_pw_pc[9:0]),  .b(s4_cw_eff_w[9:0]),   .ci(s4_eff_sub_ci_r), .s(s4_pc_l0), .co(s4_pc_c10));
			am_add #(.W(9))  u_s4_pc_h0 (.a(s4_pw_pc[18:10]),.b(s4_cw_eff_w[18:10]), .ci(1'b0),            .s(s4_pc_h0), .co(s4_pc_ch0));
			am_add #(.W(9))  u_s4_pc_h1 (.a(s4_pw_pc[18:10]),.b(s4_cw_eff_w[18:10]), .ci(1'b1),            .s(s4_pc_h1), .co(s4_pc_ch1));
			am_add #(.W(10)) u_s4_cp_l0 (.a(s4_cw[9:0]),     .b(~s4_pw_cp[9:0]),    .ci(1'b1),            .s(s4_cp_l0), .co(s4_cp_c10));
			am_add #(.W(9))  u_s4_cp_h0 (.a(s4_cw[18:10]),   .b(~s4_pw_cp[18:10]),  .ci(1'b0),            .s(s4_cp_h0), .co(s4_cp_ch0));
			am_add #(.W(9))  u_s4_cp_h1 (.a(s4_cw[18:10]),   .b(~s4_pw_cp[18:10]),  .ci(1'b1),            .s(s4_cp_h1), .co(s4_cp_ch1));

			(* shreg_extract = "no" *) reg        s45_valid, s45_ovr_en, s45_is_fma, s45_sat;
			(* dont_touch = "true", shreg_extract = "no" *) reg s45_eff_sub_lo0_r, s45_eff_sub_lo1_r, s45_eff_sub_hi0_r, s45_eff_sub_hi1_r, s45_eff_sub_sr_r;
			(* shreg_extract = "no" *) reg        s45_sp, s45_sc, s45_rdn, s45_spec_hit;
			(* shreg_extract = "no" *) reg        s45_pc_c10, s45_pc_ch0, s45_pc_ch1, s45_cp_c10, s45_cp_ch0, s45_cp_ch1;
			(* shreg_extract = "no" *) reg [2:0]  s45_r, s45_eb_hi0, s45_eb_hi1, s45_eb_hi2, s45_eb_hi3;
			(* shreg_extract = "no" *) reg [3:0]  s45_eb_lo, s45_eb_neg;
			(* shreg_extract = "no" *) reg [15:0] s45_ovr_val, s45_spec_res, s45_other;
			(* shreg_extract = "no" *) reg [9:0]  s45_pc_l0, s45_cp_l0;
			(* shreg_extract = "no" *) reg [8:0]  s45_pc_h0, s45_pc_h1, s45_cp_h0, s45_cp_h1;
			(* shreg_extract = "no" *) reg [4:0]  s45_pw_pc_hi, s45_pw_cp_hi;
			(* shreg_extract = "no" *) reg [17:0] s45_cw_eff_hi, s45_cw_hi;

			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					s45_valid <= 1'b0;
				else
					s45_valid <= s4_valid;
			end
			always @(posedge clk) begin
				s45_ovr_en        <= s4_ovr_en;
				s45_ovr_val       <= s4_ovr_val;
				s45_is_fma        <= s4_is_fma;
				s45_r             <= s4_r;
				s45_sat           <= s4_sat;
				s45_eff_sub_lo0_r <= s4_eff_sub;
				s45_eff_sub_lo1_r <= s4_eff_sub;
				s45_eff_sub_hi0_r <= s4_eff_sub;
				s45_eff_sub_hi1_r <= s4_eff_sub;
				s45_eff_sub_sr_r  <= s4_eff_sub;
				s45_sp            <= s4_sp;
				s45_sc            <= s4_sc;
				s45_rdn           <= s4_rdn;
				s45_eb_lo         <= s4_eb[3:0];
				s45_eb_neg        <= s4_eb_neg;
				s45_eb_hi0        <= s4_eb_hi0;
				s45_eb_hi1        <= s4_eb_hi1;
				s45_eb_hi2        <= s4_eb_hi2;
				s45_eb_hi3        <= s4_eb_hi3;
				s45_spec_hit      <= s4_spec_hit;
				s45_spec_res      <= s4_spec_res;
				s45_other         <= s4_other;
				s45_pc_l0         <= s4_pc_l0;
				s45_pc_h0         <= s4_pc_h0;
				s45_pc_h1         <= s4_pc_h1;
				s45_pc_c10        <= s4_pc_c10;
				s45_pc_ch0        <= s4_pc_ch0;
				s45_pc_ch1        <= s4_pc_ch1;
				s45_cp_l0         <= s4_cp_l0;
				s45_cp_h0         <= s4_cp_h0;
				s45_cp_h1         <= s4_cp_h1;
				s45_cp_c10        <= s4_cp_c10;
				s45_cp_ch0        <= s4_cp_ch0;
				s45_cp_ch1        <= s4_cp_ch1;
				s45_pw_pc_hi      <= s4_pw_pc[23:19];
				s45_cw_eff_hi     <= s4_cw_eff_w[36:19];
				s45_pw_cp_hi      <= s4_pw_cp[23:19];
				s45_cw_hi         <= s4_cw[36:19];
			end

			wire [18:0] s45_pc_lo    = {s45_pc_c10 ? s45_pc_h1 : s45_pc_h0, s45_pc_l0};
			wire        s45_pc_lo_co = s45_pc_c10 ? s45_pc_ch1 : s45_pc_ch0;
			wire [18:0] s45_cp_lo    = {s45_cp_c10 ? s45_cp_h1 : s45_cp_h0, s45_cp_l0};
			wire        s45_cp_lo_co = s45_cp_c10 ? s45_cp_ch1 : s45_cp_ch0;

			/* High 18 bits: pw_pc[36:24] == 13'd0 and ~pw_cp[36:24] == 13'h1FFF, so only 5b add + 13b inc/dec. */
			wire [4:0]  s45_pc_hi_l5, s45_cp_hi_l5;
			wire        s45_pc_c24, s45_cp_c24, s45_pc_u13_co, s45_cp_u13_bo_unused;
			wire [12:0] s45_pc_u13_p1, s45_cp_u13_m1;
			am_add #(.W(5))  u_s45_pc_h5  (.a(s45_pw_pc_hi),         .b(s45_cw_eff_hi[4:0]), .ci(s45_pc_lo_co), .s(s45_pc_hi_l5), .co(s45_pc_c24));
			am_add #(.W(13)) u_s45_pc_u13 (.a(s45_cw_eff_hi[17:5]),  .b(13'd0),              .ci(1'b1),         .s(s45_pc_u13_p1),.co(s45_pc_u13_co));
			am_add #(.W(5))  u_s45_cp_h5  (.a(s45_cw_hi[4:0]),       .b(~s45_pw_cp_hi),      .ci(s45_cp_lo_co), .s(s45_cp_hi_l5), .co(s45_cp_c24));
			am_sub #(.W(13)) u_s45_cp_u13 (.a(s45_cw_hi[17:5]),      .b(13'd1),              .bi(1'b0),         .d(s45_cp_u13_m1),.bo(s45_cp_u13_bo_unused));
			wire s45_pc_c24_a, s45_pc_c24_b, s45_cp_c24_a, s45_cp_c24_b;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pcc24a (.I0(s45_pc_c24), .O(s45_pc_c24_a));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_pcc24b (.I0(s45_pc_c24), .O(s45_pc_c24_b));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cpc24a (.I0(s45_cp_c24), .O(s45_cp_c24_a));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_cpc24b (.I0(s45_cp_c24), .O(s45_cp_c24_b));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign s45_pc_c24_a = s45_pc_c24;
			assign s45_pc_c24_b = s45_pc_c24;
			assign s45_cp_c24_a = s45_cp_c24;
			assign s45_cp_c24_b = s45_cp_c24;
`else
			assign s45_pc_c24_a = s45_pc_c24;
			assign s45_pc_c24_b = s45_pc_c24;
			assign s45_cp_c24_a = s45_cp_c24;
			assign s45_cp_c24_b = s45_cp_c24;
`endif
			wire [17:0] s45_pc_hi = {
				s45_pc_c24_b ? s45_pc_u13_p1[12:6] : s45_cw_eff_hi[17:11],
				s45_pc_c24_a ? s45_pc_u13_p1[5:0]  : s45_cw_eff_hi[10:5],
				s45_pc_hi_l5
			};
			wire [17:0] s45_cp_hi = {
				s45_cp_c24_b ? s45_cw_hi[17:11] : s45_cp_u13_m1[12:6],
				s45_cp_c24_a ? s45_cw_hi[10:5]  : s45_cp_u13_m1[5:0],
				s45_cp_hi_l5
			};
			wire        s45_co_pc = s45_pc_c24 && s45_pc_u13_co;
			wire        s45_hi_unused = &{1'b0, s45_cp_u13_bo_unused};
			wire co_pc_lo0, co_pc_lo1, co_pc_hi0, co_pc_hi1, co_pc_sr;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_copc_l0 (.I0(s45_co_pc), .O(co_pc_lo0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_copc_l1 (.I0(s45_co_pc), .O(co_pc_lo1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_copc_h0 (.I0(s45_co_pc), .O(co_pc_hi0));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_copc_h1 (.I0(s45_co_pc), .O(co_pc_hi1));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_copc_sr (.I0(s45_co_pc), .O(co_pc_sr));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign co_pc_lo0 = s45_co_pc;
			assign co_pc_lo1 = s45_co_pc;
			assign co_pc_hi0 = s45_co_pc;
			assign co_pc_hi1 = s45_co_pc;
			assign co_pc_sr = s45_co_pc;
`else
			assign co_pc_lo0 = s45_co_pc; assign co_pc_lo1 = s45_co_pc;
			assign co_pc_hi0 = s45_co_pc; assign co_pc_hi1 = s45_co_pc;
			assign co_pc_sr  = s45_co_pc;
`endif
			wire s45_swap_lo0 = s45_eff_sub_lo0_r && !co_pc_lo0;
			wire s45_swap_lo1 = s45_eff_sub_lo1_r && !co_pc_lo1;
			wire s45_swap_hi0 = s45_eff_sub_hi0_r && !co_pc_hi0;
			wire s45_swap_hi1 = s45_eff_sub_hi1_r && !co_pc_hi1;
			wire s45_swap_sr  = s45_eff_sub_sr_r  && !co_pc_sr;
			wire [36:0] s45_sum  = {
				s45_swap_hi1 ? s45_cp_hi[17:9] : s45_pc_hi[17:9],
				s45_swap_hi0 ? s45_cp_hi[8:0]  : s45_pc_hi[8:0],
				s45_swap_lo1 ? s45_cp_lo[18:9] : s45_pc_lo[18:9],
				s45_swap_lo0 ? s45_cp_lo[8:0]  : s45_pc_lo[8:0]
			};
			wire        s45_sr   = s45_swap_sr ? s45_sc : s45_sp;

			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					s5_valid <= 1'b0;
				else
					s5_valid <= s45_valid;
			end
			always @(posedge clk) begin
				s5_ovr_en   <= s45_ovr_en;
				s5_ovr_val  <= s45_ovr_val;
				s5_is_fma   <= s45_is_fma;
				s5_r        <= s45_r;
				s5_sat      <= s45_sat;
				s5_sum      <= s45_sum;
				s5_sr       <= s45_sr;
				s5_rdn      <= s45_rdn;
				s5_eb_lo    <= s45_eb_lo;
				s5_eb_neg   <= s45_eb_neg;
				s5_eb_hi0   <= s45_eb_hi0;
				s5_eb_hi1   <= s45_eb_hi1;
				s5_eb_hi2   <= s45_eb_hi2;
				s5_eb_hi3   <= s45_eb_hi3;
				s5_spec_hit <= s45_spec_hit;
				s5_spec_res <= s45_spec_res;
				s5_other    <= s45_other;
			end
			assign s4_sum_nxt = 37'd0;
			assign s4_sr_nxt  = 1'b0;
		end else begin : g_s45_direct
			wire [36:0] s4_sum_pc, s4_sum_cp;
			wire s4_co_pc;

			am_addsub #(.W(37)) u_s4_sum_pc (
				.a(s4_pw_pc), .b(s4_cw), .sub(s4_eff_sub), .s(s4_sum_pc), .co(s4_co_pc)
			);
			am_sub #(.W(37)) u_s4_sum_cp (
				.a(s4_cw), .b(s4_pw_cp), .bi(1'b0), .d(s4_sum_cp), .bo(s4_bo_cp_unused)
			);

			wire s4_swap = s4_eff_sub && !s4_co_pc;
			wire s4_swap_lo, s4_swap_hi;
`ifdef SYNTH_KINTEX7
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sw0 (.I0(s4_swap), .O(s4_swap_lo));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_b_sw1 (.I0(s4_swap), .O(s4_swap_hi));
`elsif SYNTH_KINTEX_ULTRASCALE
			assign s4_swap_lo = s4_swap;
			assign s4_swap_hi = s4_swap;
`else
			assign s4_swap_lo = s4_swap;
			assign s4_swap_hi = s4_swap;
`endif
			assign s4_sum_nxt = {
				s4_swap_hi ? s4_sum_cp[36:19] : s4_sum_pc[36:19],
				s4_swap_lo ? s4_sum_cp[18:0]  : s4_sum_pc[18:0]
			};
			assign s4_sr_nxt = s4_swap ? s4_sc : s4_sp;

			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					s5_valid <= 1'b0;
				else
					s5_valid <= s4_valid;
			end
			always @(posedge clk) begin
				s5_ovr_en   <= s4_ovr_en;
				s5_ovr_val  <= s4_ovr_val;
				s5_is_fma   <= s4_is_fma;
				s5_r        <= s4_r;
				s5_sat      <= s4_sat;
				s5_sum      <= s4_sum_nxt;
				s5_sr       <= s4_sr_nxt;
				s5_rdn      <= s4_rdn;
				s5_eb_lo    <= s4_eb[3:0];
				s5_eb_neg   <= s4_eb_neg;
				s5_eb_hi0   <= s4_eb_hi0;
				s5_eb_hi1   <= s4_eb_hi1;
				s5_eb_hi2   <= s4_eb_hi2;
				s5_eb_hi3   <= s4_eb_hi3;
				s5_spec_hit <= s4_spec_hit;
				s5_spec_res <= s4_spec_res;
				s5_other    <= s4_other;
			end
		end

		/* =====================================================================
		 * Stage 5 -> Stage 6: Level-pinned <= 4 LUT LOD, parallel shift, 1-bit
		 *                     sticky, and Carry-Select exponent add (0 CARRY4s)
		 * ===================================================================== */
		/* --- Level 1 (1 LUT6 per wire from Stage 5 FFs) --- */
		(* keep = "true" *) wire       nz_b2   = |s5_sum[36:32];
		(* keep = "true" *) wire       nz_b2_s = |s5_sum[36:32];
		(* keep = "true" *) wire [3:0] msb_b2  = {
			1'b0,
			s5_sum[36],
			!s5_sum[36] && (|s5_sum[35:34]),
			!s5_sum[36] && (s5_sum[35] || (!s5_sum[34] && s5_sum[33]))
		};
		wire [1:0] s5_eb_b2, s5_eb_b1_0, s5_eb_b1_1, s5_eb_b0_0, s5_eb_b0_1;
		wire [1:0] s5_eb_b2_hi;
`ifdef SYNTH_KINTEX7
		genvar geb;
		for (geb = 0; geb < 2; geb = geb + 1) begin : g_eb_buf
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb2   (.I0(s5_eb_lo[geb]),   .O(s5_eb_b2[geb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb2h  (.I0(s5_eb_lo[2+geb]), .O(s5_eb_b2_hi[geb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb1_0 (.I0(s5_eb_lo[geb]),   .O(s5_eb_b1_0[geb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb1_1 (.I0(s5_eb_lo[geb]),   .O(s5_eb_b1_1[geb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb0_0 (.I0(s5_eb_lo[geb]),   .O(s5_eb_b0_0[geb]));
			(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_ebb0_1 (.I0(s5_eb_lo[geb]),   .O(s5_eb_b0_1[geb]));
		end
`elsif SYNTH_KINTEX_ULTRASCALE
		genvar geb;
		for (geb = 0; geb < 2; geb = geb + 1) begin : g_eb_buf
			assign s5_eb_b2[geb] = s5_eb_lo[geb];
			assign s5_eb_b2_hi[geb] = s5_eb_lo[2+geb];
			assign s5_eb_b1_0[geb] = s5_eb_lo[geb];
			assign s5_eb_b1_1[geb] = s5_eb_lo[geb];
			assign s5_eb_b0_0[geb] = s5_eb_lo[geb];
			assign s5_eb_b0_1[geb] = s5_eb_lo[geb];
		end
`else
		assign s5_eb_b2    = s5_eb_lo[1:0];
		assign s5_eb_b2_hi = s5_eb_lo[3:2];
		assign s5_eb_b1_0  = s5_eb_lo[1:0];
		assign s5_eb_b1_1  = s5_eb_lo[1:0];
		assign s5_eb_b0_0  = s5_eb_lo[1:0];
		assign s5_eb_b0_1  = s5_eb_lo[1:0];
`endif
		(* keep = "true" *) wire b2_g32  = s5_eb_b2_hi[1] && s5_eb_b2_hi[0] && s5_sum[36];
		(* keep = "true" *) wire b2_p32  = s5_eb_b2_hi[1] ? (s5_eb_b2_hi[0] || s5_sum[36]) : (s5_eb_b2_hi[0] && s5_sum[36]);
		(* keep = "true" *) wire b2_c2_0, b2_c2_1;
		atg_su_pk_nib_c2 u_b2_c2_0 ({!s5_sum[36] && s5_sum[35], !s5_sum[36] && s5_sum[34], !s5_sum[36] && s5_sum[33]}, s5_eb_b2, 1'b0, b2_c2_0);
		atg_su_pk_nib_c2 u_b2_c2_1 ({!s5_sum[36] && s5_sum[35], !s5_sum[36] && s5_sum[34], !s5_sum[36] && s5_sum[33]}, s5_eb_b2, 1'b1, b2_c2_1);

		(* keep = "true" *) wire b1_h3   = |s5_sum[31:28], b1_h2   = |s5_sum[27:24], b1_h1 = |s5_sum[23:20], b1_h0 = |s5_sum[19:16];
		(* keep = "true" *) wire b1_h3_s = |s5_sum[31:28], b1_h2_s = |s5_sum[27:24];
		(* keep = "true" *) wire b1_m1_3 = |s5_sum[31:30], b1_m0_3 = s5_sum[31] || (!s5_sum[30] && s5_sum[29]);
		(* keep = "true" *) wire b1_m1_2 = |s5_sum[27:26], b1_m0_2 = s5_sum[27] || (!s5_sum[26] && s5_sum[25]);
		(* keep = "true" *) wire b1_m1_1 = |s5_sum[23:22], b1_m0_1 = s5_sum[23] || (!s5_sum[22] && s5_sum[21]);
		(* keep = "true" *) wire b1_m1_0 = |s5_sum[19:18], b1_m0_0 = s5_sum[19] || (!s5_sum[18] && s5_sum[17]);
		(* keep = "true" *) wire b1_c2_0_3, b1_c2_0_2, b1_c2_0_1, b1_c2_0_0;
		(* keep = "true" *) wire b1_c2_1_3, b1_c2_1_2, b1_c2_1_1, b1_c2_1_0;
		atg_su_pk_nib_c2 u_b1_c2_0_3 (s5_sum[31:29], s5_eb_b1_0, 1'b0, b1_c2_0_3);
		atg_su_pk_nib_c2 u_b1_c2_0_2 (s5_sum[27:25], s5_eb_b1_0, 1'b0, b1_c2_0_2);
		atg_su_pk_nib_c2 u_b1_c2_0_1 (s5_sum[23:21], s5_eb_b1_0, 1'b0, b1_c2_0_1);
		atg_su_pk_nib_c2 u_b1_c2_0_0 (s5_sum[19:17], s5_eb_b1_0, 1'b0, b1_c2_0_0);
		atg_su_pk_nib_c2 u_b1_c2_1_3 (s5_sum[31:29], s5_eb_b1_1, 1'b1, b1_c2_1_3);
		atg_su_pk_nib_c2 u_b1_c2_1_2 (s5_sum[27:25], s5_eb_b1_1, 1'b1, b1_c2_1_2);
		atg_su_pk_nib_c2 u_b1_c2_1_1 (s5_sum[23:21], s5_eb_b1_1, 1'b1, b1_c2_1_1);
		atg_su_pk_nib_c2 u_b1_c2_1_0 (s5_sum[19:17], s5_eb_b1_1, 1'b1, b1_c2_1_0);

		(* keep = "true" *) wire b0_h3 = |s5_sum[15:12], b0_h2 = |s5_sum[11:8], b0_h1 = |s5_sum[7:4], b0_h0 = |s5_sum[3:0];
		(* keep = "true" *) wire b0_m1_3 = |s5_sum[15:14], b0_m0_3 = s5_sum[15] || (!s5_sum[14] && s5_sum[13]);
		(* keep = "true" *) wire b0_m1_2 = |s5_sum[11:10], b0_m0_2 = s5_sum[11] || (!s5_sum[10] && s5_sum[9]);
		(* keep = "true" *) wire b0_m1_1 = |s5_sum[7:6],   b0_m0_1 = s5_sum[7]  || (!s5_sum[6]  && s5_sum[5]);
		(* keep = "true" *) wire b0_m1_0 = |s5_sum[3:2],   b0_m0_0 = s5_sum[3]  || (!s5_sum[2]  && s5_sum[1]);
		(* keep = "true" *) wire b0_c2_0_3, b0_c2_0_2, b0_c2_0_1, b0_c2_0_0;
		(* keep = "true" *) wire b0_c2_1_3, b0_c2_1_2, b0_c2_1_1, b0_c2_1_0;
		atg_su_pk_nib_c2 u_b0_c2_0_3 (s5_sum[15:13], s5_eb_b0_0, 1'b0, b0_c2_0_3);
		atg_su_pk_nib_c2 u_b0_c2_0_2 (s5_sum[11:9],  s5_eb_b0_0, 1'b0, b0_c2_0_2);
		atg_su_pk_nib_c2 u_b0_c2_0_1 (s5_sum[7:5],   s5_eb_b0_0, 1'b0, b0_c2_0_1);
		atg_su_pk_nib_c2 u_b0_c2_0_0 (s5_sum[3:1],   s5_eb_b0_0, 1'b0, b0_c2_0_0);
		atg_su_pk_nib_c2 u_b0_c2_1_3 (s5_sum[15:13], s5_eb_b0_1, 1'b1, b0_c2_1_3);
		atg_su_pk_nib_c2 u_b0_c2_1_2 (s5_sum[11:9],  s5_eb_b0_1, 1'b1, b0_c2_1_2);
		atg_su_pk_nib_c2 u_b0_c2_1_1 (s5_sum[7:5],   s5_eb_b0_1, 1'b1, b0_c2_1_1);
		atg_su_pk_nib_c2 u_b0_c2_1_0 (s5_sum[3:1],   s5_eb_b0_1, 1'b1, b0_c2_1_0);

		(* keep = "true" *) wire st2_p0, st1_p3, st1_p2, st1_p1, st1_p0, st0_p0;
		atg_su_pk_st_part6 u_st2_p0 (s5_sum[35:33], s5_sum[23:21], st2_p0);
		(* keep = "true" *) wire st2_p1 = s5_sum[36] && (|s5_sum[24:21]);
		atg_su_pk_st_part6 u_st1_p3 (s5_sum[31:29], s5_sum[19:17], st1_p3);
		atg_su_pk_st_part6 u_st1_p2 (s5_sum[27:25], s5_sum[15:13], st1_p2);
		atg_su_pk_st_part6 u_st1_p1 (s5_sum[23:21], s5_sum[11:9],  st1_p1);
		atg_su_pk_st_part6 u_st1_p0 (s5_sum[19:17], s5_sum[7:5],   st1_p0);
		atg_su_pk_st_part6 u_st0_p0 (s5_sum[14:12], s5_sum[2:0],   st0_p0);
		(* keep = "true" *) wire st0_p1 = s5_sum[15] && (|s5_sum[3:0]);
		(* keep = "true" *) wire or_4_0   = |s5_sum[4:0];
		(* keep = "true" *) wire or_8_5   = |s5_sum[8:5];
		(* keep = "true" *) wire or_12_9  = |s5_sum[12:9];
		(* keep = "true" *) wire or_16_13 = |s5_sum[16:13];
		(* keep = "true" *) wire or_20_17 = |s5_sum[20:17];
		(* keep = "true" *) wire s5_norm_base_ok = s5_valid && !s5_ovr_en && s5_is_fma && !s5_spec_hit && !(s5_sat && s5_sr);

		(* keep = "true" *) wire       nz_b1_raw = b1_h3 || b1_h2 || b1_h1 || b1_h0;
		(* keep = "true" *) wire       nz_b0     = b0_h3 || b0_h2 || b0_h1 || b0_h0;
		(* keep = "true" *) wire [3:0] msb_b1 = {
			b1_h3 || b1_h2,
			b1_h3 || (!b1_h2 && b1_h1),
			b1_h3 ? b1_m1_3 : b1_h2 ? b1_m1_2 : b1_h1 ? b1_m1_1 : b1_m1_0,
			b1_h3 ? b1_m0_3 : b1_h2 ? b1_m0_2 : b1_h1 ? b1_m0_1 : b1_m0_0
		};
		(* keep = "true" *) wire [3:0] msb_b0 = {
			b0_h3 || b0_h2,
			b0_h3 || (!b0_h2 && b0_h1),
			b0_h3 ? b0_m1_3 : b0_h2 ? b0_m1_2 : b0_h1 ? b0_m1_1 : b0_m1_0,
			b0_h3 ? b0_m0_3 : b0_h2 ? b0_m0_2 : b0_h1 ? b0_m0_1 : b0_m0_0
		};
		(* keep = "true" *) wire b1_c2_0 = b1_h3 ? b1_c2_0_3 : b1_h2 ? b1_c2_0_2 : b1_h1 ? b1_c2_0_1 : b1_c2_0_0;
		(* keep = "true" *) wire b1_c2_1 = b1_h3 ? b1_c2_1_3 : b1_h2 ? b1_c2_1_2 : b1_h1 ? b1_c2_1_1 : b1_c2_1_0;
		(* keep = "true" *) wire b0_c2_0 = b0_h3 ? b0_c2_0_3 : b0_h2 ? b0_c2_0_2 : b0_h1 ? b0_c2_0_1 : b0_c2_0_0;
		(* keep = "true" *) wire b0_c2_1 = b0_h3 ? b0_c2_1_3 : b0_h2 ? b0_c2_1_2 : b0_h1 ? b0_c2_1_1 : b0_c2_1_0;

		wire b1_m3 = b1_h3 || b1_h2, b1_m2 = b1_h3 || (!b1_h2 && b1_h1);
		wire b0_m3 = b0_h3 || b0_h2, b0_m2 = b0_h3 || (!b0_h2 && b0_h1);
		(* keep = "true" *) wire b1_g32 = (s5_eb_lo[3] && b1_m3) || ((s5_eb_lo[3] || b1_m3) && s5_eb_lo[2] && b1_m2);
		(* keep = "true" *) wire b1_p32 = (s5_eb_lo[3] || b1_m3) && (s5_eb_lo[2] || b1_m2);
		(* keep = "true" *) wire b0_g32 = (s5_eb_lo[3] && b0_m3) || ((s5_eb_lo[3] || b0_m3) && s5_eb_lo[2] && b0_m2);
		(* keep = "true" *) wire b0_p32 = (s5_eb_lo[3] || b0_m3) && (s5_eb_lo[2] || b0_m2);

		(* keep = "true" *) wire st_b2    = st2_p1 || st2_p0 || or_20_17 || or_16_13 || or_12_9 || or_8_5;
		(* keep = "true" *) wire st_b1_32 = b1_h3 ? (st1_p3 || or_16_13 || or_12_9 || or_8_5) : (st1_p2 || or_12_9 || or_8_5);
		(* keep = "true" *) wire st_b1_10 = b1_h1 ? (st1_p1 || or_8_5) : st1_p0;
		(* keep = "true" *) wire st_b0    = st0_p1 || st0_p0;

		wire [25:0] win0_pad = {s5_sum[14:0], 11'd0};
		(* keep = "true" *) wire [10:0] w2_s2;
		(* keep = "true" *) wire [13:0] w1_s2, w0_s2;
		genvar gs;
		for (gs = 0; gs < 11; gs = gs + 1) begin : g_w2_s2
			assign w2_s2[gs] = msb_b2[1] ? (msb_b2[0] ? s5_sum[24 + gs] : s5_sum[23 + gs]) :
			                               (msb_b2[0] ? s5_sum[22 + gs] : s5_sum[21 + gs]);
		end
		for (gs = 0; gs < 14; gs = gs + 1) begin : g_w10_s2
			assign w1_s2[gs] = b1_h3_s ? s5_sum[17 + gs]   : b1_h2_s ? s5_sum[13 + gs]  : b1_h1 ? s5_sum[9 + gs]  : s5_sum[5 + gs];
			assign w0_s2[gs] = b0_h3   ? win0_pad[12 + gs] : b0_h2   ? win0_pad[8 + gs] : b0_h1 ? win0_pad[4 + gs] : win0_pad[gs];
		end

		/* Optional Stage 5.5 registers (LAT >= 8) splitting LOD/shift into <= 9 cmos3 halves */
		wire        p6_valid, p6_ovr_en, p6_is_fma, p6_sat, p6_sr, p6_rdn, p6_spec_hit;
		wire [15:0] p6_ovr_val, p6_spec_res, p6_other;
		wire [2:0]  p6_r, p6_eb_hi0, p6_eb_hi1, p6_eb_hi2, p6_eb_hi3;
		wire [3:0]  p6_eb_lo, p6_eb_neg;
		wire        p6_norm_base_ok, p6_nz_b2, p6_nz_b2_s, p6_nz_b1_raw, p6_nz_b0;
		wire [3:0]  p6_msb_b2, p6_msb_b1, p6_msb_b0;
		wire        p6_b2_c2_0, p6_b2_c2_1, p6_b2_g32, p6_b2_p32;
		wire        p6_b1_c2_0, p6_b1_c2_1, p6_b1_g32, p6_b1_p32, p6_b1_h3_s, p6_b1_h2_s, p6_b1_h1, p6_b1_h0;
		wire        p6_b0_c2_0, p6_b0_c2_1, p6_b0_g32, p6_b0_p32;
		wire        p6_st_b2, p6_st_b1_32, p6_st_b1_10, p6_st_b0, p6_or_4_0;
		wire [10:0] p6_w2_s2;
		wire [13:0] p6_w1_s2, p6_w0_s2;
		wire [36:25] p6_sum_hi;

		if (LAT >= 8) begin : g_s55_split
			reg        s55_valid, s55_ovr_en, s55_is_fma, s55_sat, s55_sr, s55_rdn, s55_spec_hit;
			reg [15:0] s55_ovr_val, s55_spec_res, s55_other;
			reg [2:0]  s55_r, s55_eb_hi0, s55_eb_hi1, s55_eb_hi2, s55_eb_hi3;
			reg [3:0]  s55_eb_lo, s55_eb_neg;
			reg        s55_norm_base_ok, s55_nz_b2, s55_nz_b2_s, s55_nz_b1_raw, s55_nz_b0;
			reg [3:0]  s55_msb_b2, s55_msb_b1, s55_msb_b0;
			reg        s55_b2_c2_0, s55_b2_c2_1, s55_b2_g32, s55_b2_p32;
			reg        s55_b1_c2_0, s55_b1_c2_1, s55_b1_g32, s55_b1_p32;
			reg        s55_b1_h3_s, s55_b1_h2_s, s55_b1_h1, s55_b1_h0;
			reg        s55_b0_c2_0, s55_b0_c2_1, s55_b0_g32, s55_b0_p32;
			reg        s55_st_b2, s55_st_b1_32, s55_st_b1_10, s55_st_b0, s55_or_4_0;
			reg [10:0] s55_w2_s2;
			reg [13:0] s55_w1_s2, s55_w0_s2;
			reg [36:25] s55_sum_hi;
			always @(posedge clk or posedge rst_p) begin
				if (rst_p)
					s55_valid <= 1'b0;
				else
					s55_valid <= s5_valid;
			end
			always @(posedge clk) begin
				s55_ovr_en       <= s5_ovr_en;
				s55_is_fma       <= s5_is_fma;
				s55_sat          <= s5_sat;
				s55_sr           <= s5_sr;
				s55_rdn          <= s5_rdn;
				s55_spec_hit     <= s5_spec_hit;
				s55_ovr_val      <= s5_ovr_val;
				s55_spec_res     <= s5_spec_res;
				s55_other        <= s5_other;
				s55_r            <= s5_r;
				s55_eb_hi0       <= s5_eb_hi0;
				s55_eb_hi1       <= s5_eb_hi1;
				s55_eb_hi2       <= s5_eb_hi2;
				s55_eb_hi3       <= s5_eb_hi3;
				s55_eb_lo        <= s5_eb_lo;
				s55_eb_neg       <= s5_eb_neg;
				s55_norm_base_ok <= s5_norm_base_ok;
				s55_nz_b2        <= nz_b2;
				s55_nz_b2_s      <= nz_b2_s;
				s55_nz_b1_raw    <= nz_b1_raw;
				s55_nz_b0        <= nz_b0;
				s55_msb_b2       <= msb_b2;
				s55_msb_b1       <= msb_b1;
				s55_msb_b0       <= msb_b0;
				s55_b2_c2_0      <= b2_c2_0;
				s55_b2_c2_1      <= b2_c2_1;
				s55_b2_g32       <= b2_g32;
				s55_b2_p32       <= b2_p32;
				s55_b1_c2_0      <= b1_c2_0;
				s55_b1_c2_1      <= b1_c2_1;
				s55_b1_g32       <= b1_g32;
				s55_b1_p32       <= b1_p32;
				s55_b1_h3_s      <= b1_h3_s;
				s55_b1_h2_s      <= b1_h2_s;
				s55_b1_h1        <= b1_h1;
				s55_b1_h0        <= b1_h0;
				s55_b0_c2_0      <= b0_c2_0;
				s55_b0_c2_1      <= b0_c2_1;
				s55_b0_g32       <= b0_g32;
				s55_b0_p32       <= b0_p32;
				s55_st_b2        <= st_b2;
				s55_st_b1_32     <= st_b1_32;
				s55_st_b1_10     <= st_b1_10;
				s55_st_b0        <= st_b0;
				s55_or_4_0       <= or_4_0;
				s55_w2_s2        <= w2_s2;
				s55_w1_s2        <= w1_s2;
				s55_w0_s2        <= w0_s2;
				s55_sum_hi       <= s5_sum[36:25];
			end
			assign p6_valid        = s55_valid;
			assign p6_ovr_en       = s55_ovr_en;
			assign p6_is_fma       = s55_is_fma;
			assign p6_sat          = s55_sat;
			assign p6_sr           = s55_sr;
			assign p6_rdn          = s55_rdn;
			assign p6_spec_hit     = s55_spec_hit;
			assign p6_ovr_val      = s55_ovr_val;
			assign p6_spec_res     = s55_spec_res;
			assign p6_other        = s55_other;
			assign p6_r            = s55_r;
			assign p6_eb_hi0       = s55_eb_hi0;
			assign p6_eb_hi1       = s55_eb_hi1;
			assign p6_eb_hi2       = s55_eb_hi2;
			assign p6_eb_hi3       = s55_eb_hi3;
			assign p6_eb_lo        = s55_eb_lo;
			assign p6_eb_neg       = s55_eb_neg;
			assign p6_norm_base_ok = s55_norm_base_ok;
			assign p6_nz_b2        = s55_nz_b2;
			assign p6_nz_b2_s      = s55_nz_b2_s;
			assign p6_nz_b1_raw    = s55_nz_b1_raw;
			assign p6_nz_b0        = s55_nz_b0;
			assign p6_msb_b2       = s55_msb_b2;
			assign p6_msb_b1       = s55_msb_b1;
			assign p6_msb_b0       = s55_msb_b0;
			assign p6_b2_c2_0      = s55_b2_c2_0;
			assign p6_b2_c2_1      = s55_b2_c2_1;
			assign p6_b2_g32       = s55_b2_g32;
			assign p6_b2_p32       = s55_b2_p32;
			assign p6_b1_c2_0      = s55_b1_c2_0;
			assign p6_b1_c2_1      = s55_b1_c2_1;
			assign p6_b1_g32       = s55_b1_g32;
			assign p6_b1_p32       = s55_b1_p32;
			assign p6_b1_h3_s      = s55_b1_h3_s;
			assign p6_b1_h2_s      = s55_b1_h2_s;
			assign p6_b1_h1        = s55_b1_h1;
			assign p6_b1_h0        = s55_b1_h0;
			assign p6_b0_c2_0      = s55_b0_c2_0;
			assign p6_b0_c2_1      = s55_b0_c2_1;
			assign p6_b0_g32       = s55_b0_g32;
			assign p6_b0_p32       = s55_b0_p32;
			assign p6_st_b2        = s55_st_b2;
			assign p6_st_b1_32     = s55_st_b1_32;
			assign p6_st_b1_10     = s55_st_b1_10;
			assign p6_st_b0        = s55_st_b0;
			assign p6_or_4_0       = s55_or_4_0;
			assign p6_w2_s2        = s55_w2_s2;
			assign p6_w1_s2        = s55_w1_s2;
			assign p6_w0_s2        = s55_w0_s2;
			assign p6_sum_hi       = s55_sum_hi;
		end else begin : g_s55_direct
			assign p6_valid        = s5_valid;
			assign p6_ovr_en       = s5_ovr_en;
			assign p6_is_fma       = s5_is_fma;
			assign p6_sat          = s5_sat;
			assign p6_sr           = s5_sr;
			assign p6_rdn          = s5_rdn;
			assign p6_spec_hit     = s5_spec_hit;
			assign p6_ovr_val      = s5_ovr_val;
			assign p6_spec_res     = s5_spec_res;
			assign p6_other        = s5_other;
			assign p6_r            = s5_r;
			assign p6_eb_hi0       = s5_eb_hi0;
			assign p6_eb_hi1       = s5_eb_hi1;
			assign p6_eb_hi2       = s5_eb_hi2;
			assign p6_eb_hi3       = s5_eb_hi3;
			assign p6_eb_lo        = s5_eb_lo;
			assign p6_eb_neg       = s5_eb_neg;
			assign p6_norm_base_ok = s5_norm_base_ok;
			assign p6_nz_b2        = nz_b2;
			assign p6_nz_b2_s      = nz_b2_s;
			assign p6_nz_b1_raw    = nz_b1_raw;
			assign p6_nz_b0        = nz_b0;
			assign p6_msb_b2       = msb_b2;
			assign p6_msb_b1       = msb_b1;
			assign p6_msb_b0       = msb_b0;
			assign p6_b2_c2_0      = b2_c2_0;
			assign p6_b2_c2_1      = b2_c2_1;
			assign p6_b2_g32       = b2_g32;
			assign p6_b2_p32       = b2_p32;
			assign p6_b1_c2_0      = b1_c2_0;
			assign p6_b1_c2_1      = b1_c2_1;
			assign p6_b1_g32       = b1_g32;
			assign p6_b1_p32       = b1_p32;
			assign p6_b1_h3_s      = b1_h3_s;
			assign p6_b1_h2_s      = b1_h2_s;
			assign p6_b1_h1        = b1_h1;
			assign p6_b1_h0        = b1_h0;
			assign p6_b0_c2_0      = b0_c2_0;
			assign p6_b0_c2_1      = b0_c2_1;
			assign p6_b0_g32       = b0_g32;
			assign p6_b0_p32       = b0_p32;
			assign p6_st_b2        = st_b2;
			assign p6_st_b1_32     = st_b1_32;
			assign p6_st_b1_10     = st_b1_10;
			assign p6_st_b0        = st_b0;
			assign p6_or_4_0       = or_4_0;
			assign p6_w2_s2        = w2_s2;
			assign p6_w1_s2        = w1_s2;
			assign p6_w0_s2        = w0_s2;
			assign p6_sum_hi       = s5_sum[36:25];
		end

		wire nz_b1_a, nz_b1_b, nz_b1_c;
		wire nz_b2_sa, nz_b2_sb, nz_b2_sc;
`ifdef SYNTH_KINTEX7
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb1_a (.I0(p6_nz_b1_raw), .O(nz_b1_a));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb1_b (.I0(p6_nz_b1_raw), .O(nz_b1_b));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb1_c (.I0(p6_nz_b1_raw), .O(nz_b1_c));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb2_a (.I0(p6_nz_b2_s),   .O(nz_b2_sa));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb2_b (.I0(p6_nz_b2_s),   .O(nz_b2_sb));
		(* dont_touch = "true" *) LUT1 #(.INIT(2'b10)) u_buf_nzb2_c (.I0(p6_nz_b2_s),   .O(nz_b2_sc));
`elsif SYNTH_KINTEX_ULTRASCALE
		assign nz_b1_a = p6_nz_b1_raw;
		assign nz_b1_b = p6_nz_b1_raw;
		assign nz_b1_c = p6_nz_b1_raw;
		assign nz_b2_sa = p6_nz_b2_s;
		assign nz_b2_sb = p6_nz_b2_s;
		assign nz_b2_sc = p6_nz_b2_s;
`else
		assign nz_b1_a = p6_nz_b1_raw;
		assign nz_b1_b = p6_nz_b1_raw;
		assign nz_b1_c = p6_nz_b1_raw;
		assign nz_b2_sa = p6_nz_b2_s;
		assign nz_b2_sb = p6_nz_b2_s;
		assign nz_b2_sc = p6_nz_b2_s;
`endif

		/* --- Level 3 (1 LUT6 per wire from Stage 5.5 FFs) --- */
		(* keep = "true" *) wire [10:0] w2_sh, w1_sh, w0_sh;
		for (gs = 0; gs < 11; gs = gs + 1) begin : g_w_sh
			assign w2_sh[gs] = p6_sum_hi[36] ? p6_sum_hi[25 + gs] : p6_w2_s2[gs];
			assign w1_sh[gs] = p6_msb_b1[1] ? (p6_msb_b1[0] ? p6_w1_s2[gs + 3] : p6_w1_s2[gs + 2]) :
			                                  (p6_msb_b1[0] ? p6_w1_s2[gs + 1] : p6_w1_s2[gs]);
			assign w0_sh[gs] = p6_msb_b0[1] ? (p6_msb_b0[0] ? p6_w0_s2[gs + 3] : p6_w0_s2[gs + 2]) :
			                                  (p6_msb_b0[0] ? p6_w0_s2[gs + 1] : p6_w0_s2[gs]);
		end

		(* keep = "true" *) wire [3:0] s5_sub_msb   = p6_nz_b2 ? p6_msb_b2 : nz_b1_a ? p6_msb_b1 : p6_msb_b0;
		(* keep = "true" *) wire [3:0] s5_msb_x_eb  = (p6_nz_b2 ? p6_msb_b2 : nz_b1_a ? p6_msb_b1 : p6_msb_b0) ^ p6_eb_lo;
		(* keep = "true" *) wire [3:0] s5_msb_x_neg = (p6_nz_b2 ? p6_msb_b2 : nz_b1_a ? p6_msb_b1 : p6_msb_b0) ^ p6_eb_neg;
		(* keep = "true" *) wire       s5_c2_0      = p6_nz_b2 ? p6_b2_c2_0 : nz_b1_b ? p6_b1_c2_0 : p6_b0_c2_0;
		(* keep = "true" *) wire       s5_c2_1      = p6_nz_b2 ? p6_b2_c2_1 : nz_b1_b ? p6_b1_c2_1 : p6_b0_c2_1;
		(* keep = "true" *) wire       s5_g32       = p6_nz_b2 ? p6_b2_g32  : nz_b1_b ? p6_b1_g32  : p6_b0_g32;
		(* keep = "true" *) wire       s5_p32       = p6_nz_b2 ? p6_b2_p32  : nz_b1_b ? p6_b1_p32  : p6_b0_p32;
		(* keep = "true" *) wire [2:0] s5_eb_hi     = nz_b2_sa ? p6_eb_hi2 : nz_b1_b ? p6_eb_hi1 : p6_eb_hi0;
		(* keep = "true" *) wire [2:0] s5_eb_hi_p1  = nz_b2_sa ? p6_eb_hi3 : nz_b1_b ? p6_eb_hi2 : p6_eb_hi1;
		(* keep = "true" *) wire       s5_norm_ok   = p6_norm_base_ok && (nz_b2_sa || nz_b1_b || p6_nz_b0);
		(* keep = "true" *) wire       s5_use_norm  = p6_valid && !p6_ovr_en && p6_is_fma && !p6_spec_hit && (nz_b2_sa || nz_b1_b || p6_nz_b0);

		/* --- Level 4 (1 LUT6 per wire from Level 3, directly into Stage 6 FFs) --- */
		wire [10:0] s5_mg = {
			nz_b2_sc ? w2_sh[10:6] : (nz_b1_c ? w1_sh[10:6] : w0_sh[10:6]),
			nz_b2_sb ? w2_sh[5:0]  : (nz_b1_c ? w1_sh[5:0]  : w0_sh[5:0])
		};
		wire [9:0]  s5_m  = s5_mg[10:1];
		wire        s5_g  = s5_mg[0];
		wire        s5_st = nz_b2_sa ? (p6_st_b2 || p6_or_4_0) :
		                    (p6_b1_h3_s || p6_b1_h2_s) ? (p6_st_b1_32 || p6_or_4_0) :
		                    (p6_b1_h1 || p6_b1_h0) ? (p6_st_b1_10 || p6_or_4_0) : p6_st_b0;

		wire s5_c1_0 = p6_eb_lo[0] && s5_sub_msb[0];
		wire s5_c1_1 = p6_eb_lo[0] || s5_sub_msb[0];
		wire s5_c3_0 = (p6_eb_lo[2] && s5_sub_msb[2]) || ((p6_eb_lo[2] ^ s5_sub_msb[2]) && s5_c2_0);
		wire s5_c3_1 = (p6_eb_lo[2] && s5_sub_msb[2]) || ((p6_eb_lo[2] ^ s5_sub_msb[2]) && s5_c2_1);
		wire s5_c4_0 = s5_g32 || (s5_p32 && s5_c2_0);
		wire s5_c4_1 = s5_g32 || (s5_p32 && s5_c2_1);

		wire [5:0] s5_er = {
			s5_c4_0 ? s5_eb_hi_p1[1:0] : s5_eb_hi[1:0],
			p6_eb_lo[3] ^ s5_sub_msb[3] ^ s5_c3_0,
			p6_eb_lo[2] ^ s5_sub_msb[2] ^ s5_c2_0,
			p6_eb_lo[1] ^ s5_sub_msb[1] ^ s5_c1_0,
			p6_eb_lo[0] ^ s5_sub_msb[0]
		};
		wire [5:0] s5_er_p1 = {
			s5_c4_1 ? s5_eb_hi_p1[1:0] : s5_eb_hi[1:0],
			p6_eb_lo[3] ^ s5_sub_msb[3] ^ s5_c3_1,
			p6_eb_lo[2] ^ s5_sub_msb[2] ^ s5_c2_1,
			p6_eb_lo[1] ^ s5_sub_msb[1] ^ s5_c1_1,
			~(p6_eb_lo[0] ^ s5_sub_msb[0])
		};
		wire s5_er0_en = s5_norm_ok && !(s5_c4_0 ? s5_eb_hi_p1[2] : s5_eb_hi[2]);
		wire s5_er1_en = s5_norm_ok && !(s5_c4_1 ? s5_eb_hi_p1[2] : s5_eb_hi[2]);
		wire s5_er_lo_all1 = &s5_msb_x_eb;
		wire s5_er_lo_zero = ~|s5_msb_x_neg;

		wire [1:0] s5_rm_mode =
			(p6_r == 3'd1 || (p6_r == 3'd2 && !p6_sr) || (p6_r == 3'd3 && p6_sr)) ? 2'd0 :
			((p6_r == 3'd2 && p6_sr) || (p6_r == 3'd3 && !p6_sr)) ? 2'd1 :
			(p6_r == 3'd4) ? 2'd2 : 2'd3;

		wire [15:0] s5_non_norm_raw = !p6_valid ? 16'd0 :
			p6_ovr_en ? p6_ovr_val :
			!p6_is_fma ? p6_other :
			p6_spec_hit ? p6_spec_res :
			(p6_sat ? 16'd0 : {p6_rdn, 15'd0});

		/* --- Stage 6 registers --- */
		(* dont_touch = "true", shreg_extract = "no" *) reg [1:0]  s6_rm_mode, s6_rm_mode_b;
		(* shreg_extract = "no" *) reg        s6_sat;
		(* shreg_extract = "no" *) reg [9:0]  s6_m;
		(* shreg_extract = "no" *) reg [5:0]  s6_er, s6_er_p1;
		(* shreg_extract = "no" *) reg        s6_er0_en, s6_er1_en;
		(* shreg_extract = "no" *) reg        s6_er_lo_all1, s6_er_lo_zero;
		(* shreg_extract = "no" *) reg        s6_trunc_ovf_en;
		(* shreg_extract = "no" *) reg        s6_g, s6_st;
		(* shreg_extract = "no" *) reg        s6_y15;
		(* shreg_extract = "no" *) reg [14:0] s6_non_norm;

		always @(posedge clk or posedge rst_p) begin
			if (rst_p) begin
				s6_er0_en       <= 1'b0;
				s6_er1_en       <= 1'b0;
				s6_trunc_ovf_en <= 1'b0;
				s6_y15          <= 1'b0;
				s6_non_norm     <= 15'd0;
			end else begin
				s6_er0_en       <= s5_er0_en;
				s6_er1_en       <= s5_er1_en;
				s6_trunc_ovf_en <= s5_er0_en && !p6_sat && (s5_rm_mode == 2'd0);
				s6_y15          <= s5_use_norm ? (p6_sat ? 1'b0 : p6_sr) : s5_non_norm_raw[15];
				s6_non_norm     <= s5_use_norm ? 15'd0 : s5_non_norm_raw[14:0];
			end
		end
		always @(posedge clk) begin
			s6_rm_mode    <= s5_rm_mode;
			s6_rm_mode_b  <= s5_rm_mode;
			s6_sat        <= s5_sat;
			s6_m          <= s5_m;
			s6_er         <= s5_er;
			s6_er_p1      <= s5_er_p1;
			s6_er_lo_all1 <= s5_er_lo_all1;
			s6_er_lo_zero <= s5_er_lo_zero;
			s6_g          <= s5_g;
			s6_st         <= s5_st;
		end

		/* =====================================================================
		 * Stage 6: Level-pinned 2-LUT rounding and output mux (0 CARRY4s)
		 * ===================================================================== */
		/* --- Level 1 (each wire <= 6 Stage-6 FF inputs -> 1 LUT6) --- */
		(* keep = "true" *) wire s6_up_l1 =
			(s6_rm_mode == 2'd0) ? 1'b0 :
			(s6_rm_mode == 2'd1) ? (s6_g || s6_st) :
			(s6_rm_mode == 2'd2) ? s6_g :
			                       (s6_g && (s6_st || s6_m[0]));
		(* keep = "true" *) wire s6_c1_l1 = s6_m[0] && (
			(s6_rm_mode_b == 2'd0) ? 1'b0 :
			(s6_rm_mode_b == 2'd1) ? (s6_g || s6_st) :
			(s6_rm_mode_b == 2'd2) ? s6_g :
			                         s6_g
		);
		(* keep = "true" *) wire s6_m31_all1_l1 = &s6_m[3:1];
		(* keep = "true" *) wire s6_m41_all1_l1 = &s6_m[4:1];
		(* keep = "true" *) wire s6_m51_all1_l1 = &s6_m[5:1];
		(* keep = "true" *) wire s6_m61_all1_l1 = &s6_m[6:1];
		(* keep = "true" *) wire s6_m87_all1_l1 = &s6_m[8:7];
		(* keep = "true" *) wire s6_m97_all1_l1 = &s6_m[9:7];

		(* keep = "true" *) wire s6_m_pass_l1 = s6_er0_en && !s6_er[5] &&
			(s6_sat ? (!s6_er[4] && !s6_er_lo_zero && !s6_er_lo_all1) :
			          (s6_er[4] ? !s6_er_lo_all1 : !s6_er_lo_zero));
		(* keep = "true" *) wire s6_ovf_trunc_l1 = s6_trunc_ovf_en && (s6_er[5] || (s6_er[4] && s6_er_lo_all1));

		(* keep = "true" *) wire [9:0] s6_m_fb_l1;
		for (gs = 0; gs < 10; gs = gs + 1) begin : g_m_fb_l1
			assign s6_m_fb_l1[gs] = s6_non_norm[gs] || (s6_trunc_ovf_en && (s6_er[5] || (s6_er[4] && s6_er_lo_all1)));
		end

		(* keep = "true" *) wire [14:10] s6_h0_l1, s6_h1_l1;
		assign s6_h0_l1[14] = s6_non_norm[14] || (s6_er0_en && !s6_sat && (s6_er[5] || s6_er[4]));
		assign s6_h1_l1[14] = s6_non_norm[14] || (s6_er1_en && !s6_sat && (s6_er_p1[5] || s6_er_p1[4]));
		for (gs = 0; gs < 4; gs = gs + 1) begin : g_h_l1
			assign s6_h0_l1[10 + gs] = s6_non_norm[10 + gs] || (s6_er0_en && (s6_er[5] || (s6_sat && s6_er[4]) || s6_er[gs]));
			assign s6_h1_l1[10 + gs] = s6_non_norm[10 + gs] || (s6_er1_en && (s6_er_p1[5] || (s6_sat && s6_er_p1[4]) || s6_er_p1[gs]));
		end

		/* --- Level 2 (each output bit <= 6 Level-1 inputs -> 1 LUT6) --- */
		wire [15:0] s6_y_nxt;
		assign s6_y_nxt[0]  = s6_m_pass_l1 ? (s6_m[0] ^ s6_up_l1) : s6_m_fb_l1[0];
		assign s6_y_nxt[1]  = s6_m_pass_l1 ? (s6_m[1] ^ s6_c1_l1) : s6_m_fb_l1[1];
		assign s6_y_nxt[2]  = s6_m_pass_l1 ? (s6_m[2] ^ (s6_c1_l1 && s6_m[1])) : s6_m_fb_l1[2];
		assign s6_y_nxt[3]  = s6_m_pass_l1 ? (s6_m[3] ^ (s6_c1_l1 && (&s6_m[2:1]))) : s6_m_fb_l1[3];
		assign s6_y_nxt[4]  = s6_m_pass_l1 ? (s6_m[4] ^ (s6_c1_l1 && s6_m31_all1_l1)) : s6_m_fb_l1[4];
		assign s6_y_nxt[5]  = s6_m_pass_l1 ? (s6_m[5] ^ (s6_c1_l1 && s6_m41_all1_l1)) : s6_m_fb_l1[5];
		assign s6_y_nxt[6]  = s6_m_pass_l1 ? (s6_m[6] ^ (s6_c1_l1 && s6_m51_all1_l1)) : s6_m_fb_l1[6];
		assign s6_y_nxt[7]  = s6_m_pass_l1 ? (s6_m[7] ^ (s6_c1_l1 && s6_m61_all1_l1)) : s6_m_fb_l1[7];
		assign s6_y_nxt[8]  = s6_m_pass_l1 ? (s6_m[8] ^ (s6_c1_l1 && s6_m61_all1_l1 && s6_m[7])) : s6_m_fb_l1[8];
		assign s6_y_nxt[9]  = s6_m_pass_l1 ? (s6_m[9] ^ (s6_c1_l1 && s6_m61_all1_l1 && s6_m87_all1_l1)) : s6_m_fb_l1[9];
		assign s6_y_nxt[10] = (s6_c1_l1 && s6_m61_all1_l1 && s6_m97_all1_l1) ? s6_h1_l1[10] : (!s6_ovf_trunc_l1 && s6_h0_l1[10]);
		assign s6_y_nxt[11] = (s6_c1_l1 && s6_m61_all1_l1 && s6_m97_all1_l1) ? s6_h1_l1[11] : s6_h0_l1[11];
		assign s6_y_nxt[12] = (s6_c1_l1 && s6_m61_all1_l1 && s6_m97_all1_l1) ? s6_h1_l1[12] : s6_h0_l1[12];
		assign s6_y_nxt[13] = (s6_c1_l1 && s6_m61_all1_l1 && s6_m97_all1_l1) ? s6_h1_l1[13] : s6_h0_l1[13];
		assign s6_y_nxt[14] = (s6_c1_l1 && s6_m61_all1_l1 && s6_m97_all1_l1) ? s6_h1_l1[14] : s6_h0_l1[14];
		assign s6_y_nxt[15] = s6_y15;

		if (LAT >= 9) begin : g_s7_out
			(* shreg_extract = "no" *) reg [15:0] s7_y;
			always @(posedge clk)
				s7_y <= s6_y_nxt;
			assign y = s7_y;
		end else begin : g_s6_out
			assign y = s6_y_nxt;
		end

		wire unused_lat6 = &{1'b0, s4_prod[31:22], s1_co_unused, s1_ab_diff_unused, s1_ba_diff_unused,
			s2_bo_unused, s2_eb_c[9:7], s2_eb_t[9:7], s3_dp37[9:6], s4_bo_cp_unused};
	end
	endgenerate
endmodule

module atg_su_pk_st_part6 (
	input wire [2:0] h31,
	input wire [2:0] l20,
	output wire st
);
	assign st = (h31[2] && (|l20[2:0])) || (h31[1] && (|l20[1:0])) || (h31[0] && l20[0]);
endmodule

module atg_su_pk_nib_c2 (
	input wire [2:0] x31,
	input wire [1:0] eb10,
	input wire ci,
	output wire co
);
	wire m1 = |x31[2:1];
	wire m0 = x31[2] || (!x31[1] && x31[0]);
	wire c1 = ci ? (eb10[0] || m0) : (eb10[0] && m0);
	assign co = (eb10[1] && m1) || ((eb10[1] ^ m1) && c1);
endmodule

`default_nettype wire
