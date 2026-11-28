/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_dc_tmds: DVI/HDMI TMDS encoding (8b/10b with DC balance, control
 * symbols in blanking) of the three channels; the 10-bit symbols are
 * serialised by OSERDESE2 in the board top (vendor primitive).  See
 * spec/gpu/dc/atg_dc_tmds.md.
 *
 * The DVI 1.0 algorithm per channel (Values referenced: DVI 1.0 §3.2.3,
 * the encoder's flow chart): stage 1 makes the transition-minimised q_m
 * (XOR or XNOR chain by the number of ones), stage 2 chooses to invert it
 * by the running disparity cnt and adds bit 9.  Outside DE the channel
 * sends the control symbol of {C1, C0} and cnt is 0: channel 0 (blue)
 * carries {vs, hs}, channels 1 and 2 send {0, 0}.  Symbols are LSB first:
 * tmds[9:0] is channel 0, [19:10] channel 1 (green), [29:20] channel 2
 * (red).  Five cycles of latency (pipelined for <= 12 cmos3 / 15 FO4).
 */

`timescale 1ns / 1ps
`default_nettype none

module atg_dc_tmds (
	input wire clk,
	input wire rst_n,
	input wire [23:0] rgb,
	input wire de,
	input wire hs,
	input wire vs,
	output wire [29:0] tmds
);
	(* keep = "true" *) wire rst_high = !rst_n;

	/* The number of ones of a nibble (pure logic, no ROM or arithmetic). */
	function [2:0] ones4;
		input [3:0] n;
		reg p0, g0, p1, g1;
		begin
			p0 = n[0] ^ n[1];
			g0 = n[0] & n[1];
			p1 = n[2] ^ n[3];
			g1 = n[2] & n[3];
			ones4[0] = p0 ^ p1;
			ones4[1] = (g0 ^ g1) ^ (p0 & p1);
			ones4[2] = g0 & g1;
		end
	endfunction

	reg [7:0] s1a_qm_xor [0:2];
	reg       s1a_xnor   [0:2];
	reg [1:0] s1a_ctrl   [0:2];
	reg       s1a_de;

	reg [8:0] s1b_qm     [0:2];
	reg [2:0] s1b_n_even [0:2];
	reg [2:0] s1b_n_odd  [0:2];
	reg [1:0] s1b_ctrl   [0:2];
	reg       s1b_de;

	reg [8:0] s1c_qm     [0:2];
	reg [3:0] s1c_n1     [0:2];
	reg [1:0] s1c_ctrl   [0:2];
	reg       s1c_de;

	reg [8:0] s1_qm      [0:2];
	reg       s1_n1_eq4  [0:2];
	reg       s1_n1_gt4  [0:2];
	reg       s1_n1_lt4  [0:2];
	reg [3:0] s1_dh_bal  [0:2];
	reg [3:0] s1_dh_same [0:2];
	reg [3:0] s1_dh_diff [0:2];
	reg [1:0] s1_ctrl    [0:2];
	(* max_fanout = 16 *) reg s1_de;

`ifdef SYNTH_KINTEX7
	wire [23:0] rgb_a, rgb_b;
	genvar gi;
	generate
		for (gi = 0; gi < 24; gi = gi + 1) begin : g_rgb_buf
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_a (.I0(rgb[gi]), .O(rgb_a[gi]));
			(* keep = "true" *) LUT1 #(.INIT(2'b10)) u_buf_b (.I0(rgb[gi]), .O(rgb_b[gi]));
		end
	endgenerate
`elsif SYNTH_KINTEX_ULTRASCALE
	wire [23:0] rgb_a, rgb_b;
	genvar gi;
	generate
		for (gi = 0; gi < 24; gi = gi + 1) begin : g_rgb_buf
			assign rgb_a[gi] = rgb[gi];
			assign rgb_b[gi] = rgb[gi];
		end
	endgenerate
`else
	wire [23:0] rgb_a = rgb;
	wire [23:0] rgb_b = rgb;
`endif

	always @(posedge clk or posedge rst_high) begin
		if (rst_high) begin
			s1a_de <= 1'b0;
			s1a_ctrl[0] <= 2'd0;
			s1a_ctrl[1] <= 2'd0;
			s1a_ctrl[2] <= 2'd0;
			s1b_de <= 1'b0;
			s1b_ctrl[0] <= 2'd0;
			s1b_ctrl[1] <= 2'd0;
			s1b_ctrl[2] <= 2'd0;
			s1c_de <= 1'b0;
			s1c_ctrl[0] <= 2'd0;
			s1c_ctrl[1] <= 2'd0;
			s1c_ctrl[2] <= 2'd0;
			s1_de <= 1'b0;
			s1_ctrl[0] <= 2'd0;
			s1_ctrl[1] <= 2'd0;
			s1_ctrl[2] <= 2'd0;
		end else begin
			s1a_de <= de;
			s1a_ctrl[0] <= {vs, hs};
			s1a_ctrl[1] <= 2'd0;
			s1a_ctrl[2] <= 2'd0;
			s1b_de <= s1a_de;
			s1b_ctrl[0] <= s1a_ctrl[0];
			s1b_ctrl[1] <= s1a_ctrl[1];
			s1b_ctrl[2] <= s1a_ctrl[2];
			s1c_de <= s1b_de;
			s1c_ctrl[0] <= s1b_ctrl[0];
			s1c_ctrl[1] <= s1b_ctrl[1];
			s1c_ctrl[2] <= s1b_ctrl[2];
			s1_de <= s1c_de;
			s1_ctrl[0] <= s1c_ctrl[0];
			s1_ctrl[1] <= s1c_ctrl[1];
			s1_ctrl[2] <= s1c_ctrl[2];
		end
	end

	genvar c;
	generate
	for (c = 0; c < 3; c = c + 1) begin : g_ch
		/* Stage 1a: xnor_ decision and balanced <= 3-level XOR tree for qm_xor. */
		wire [7:0] d_a = rgb_a[8*c +: 8];
		wire [7:0] d_b = rgb_b[8*c +: 8];
		wire [1:0] d_c0_3 = {
			(d_a[1] & d_a[2]) | (d_a[1] & d_a[3]) | (d_a[2] & d_a[3]),
			d_a[1] ^ d_a[2] ^ d_a[3]
		};
		wire [2:0] d_c1 = ones4(d_a[7:4]);
		wire xnor_ = (d_c1 >= 3'd4) ||
			(d_c1 == 3'd3 && d_c0_3 >= 2'd1) ||
			(d_c1 == 3'd2 && d_c0_3 >= 2'd2) ||
			(d_c1 == 3'd1 && d_c0_3 == 2'd3);

		wire px_01 = d_b[0] ^ d_b[1];
		wire px_23 = d_b[2] ^ d_b[3];
		wire px_45 = d_b[4] ^ d_b[5];
		wire px_67 = d_b[6] ^ d_b[7];
		wire px_03 = px_01 ^ px_23;
		wire [7:0] qm_xor;
		assign qm_xor[0] = d_b[0];
		assign qm_xor[1] = px_01;
		assign qm_xor[2] = px_01 ^ d_b[2];
		assign qm_xor[3] = px_03;
		assign qm_xor[4] = px_03 ^ d_b[4];
		assign qm_xor[5] = px_03 ^ px_45;
		assign qm_xor[6] = px_03 ^ px_45 ^ d_b[6];
		assign qm_xor[7] = px_03 ^ (px_45 ^ px_67);

		always @(posedge clk or posedge rst_high) begin
			if (rst_high) begin
				s1a_qm_xor[c] <= 8'd0;
				s1a_xnor[c]   <= 1'b0;
			end else begin
				s1a_qm_xor[c] <= qm_xor;
				s1a_xnor[c]   <= xnor_;
			end
		end

		/* Stage 1b: apply xnor_ to form next_qm and count even/odd nibble ones. */
		wire [8:0] next_qm = {
			!s1a_xnor[c],
			s1a_qm_xor[c][7] ^ s1a_xnor[c],
			s1a_qm_xor[c][6],
			s1a_qm_xor[c][5] ^ s1a_xnor[c],
			s1a_qm_xor[c][4],
			s1a_qm_xor[c][3] ^ s1a_xnor[c],
			s1a_qm_xor[c][2],
			s1a_qm_xor[c][1] ^ s1a_xnor[c],
			s1a_qm_xor[c][0]
		};
		wire [2:0] next_n_even = ones4({next_qm[6], next_qm[4], next_qm[2], next_qm[0]});
		wire [2:0] next_n_odd  = ones4({next_qm[7], next_qm[5], next_qm[3], next_qm[1]});

		always @(posedge clk or posedge rst_high) begin
			if (rst_high) begin
				s1b_qm[c]     <= 9'd0;
				s1b_n_even[c] <= 3'd0;
				s1b_n_odd[c]  <= 3'd0;
			end else begin
				s1b_qm[c]     <= next_qm;
				s1b_n_even[c] <= next_n_even;
				s1b_n_odd[c]  <= next_n_odd;
			end
		end

		/* Stage 1c: 4-bit add n1 = n_even + n_odd. */
		wire [3:0] next_n1;
		wire n1_co_unused;
		am_add #(.W(4)) a_n1 (
			.a({1'b0, s1b_n_even[c]}), .b({1'b0, s1b_n_odd[c]}), .ci(1'b0),
			.s(next_n1), .co(n1_co_unused)
		);

		always @(posedge clk or posedge rst_high) begin
			if (rst_high) begin
				s1c_qm[c] <= 9'd0;
				s1c_n1[c] <= 4'd0;
			end else begin
				s1c_qm[c] <= s1b_qm[c];
				s1c_n1[c] <= next_n1;
			end
		end

		/* Stage 1d: precompute n1 flags and the three candidate half-deltas. */
		wire [3:0] c_n1 = s1c_n1[c];
		wire c_q8 = s1c_qm[c][8];
		wire c_eq4 = (c_n1 == 4'd4);
		wire c_gt4 = c_n1[3] || (c_n1[2] && (|c_n1[1:0]));
		wire c_lt4 = !c_n1[3] && !c_n1[2];

		wire [3:0] dh_pos, dh_pos_m1, dh_neg, dh_neg_p1;
		wire [3:0] dh_unused;
		am_sub #(.W(4)) s_dh_p  (.a(c_n1), .b(4'd4), .bi(1'b0), .d(dh_pos),    .bo(dh_unused[0]));
		am_sub #(.W(4)) s_dh_pm (.a(c_n1), .b(4'd5), .bi(1'b0), .d(dh_pos_m1), .bo(dh_unused[1]));
		am_sub #(.W(4)) s_dh_n  (.a(4'd4), .b(c_n1), .bi(1'b0), .d(dh_neg),    .bo(dh_unused[2]));
		am_sub #(.W(4)) s_dh_np (.a(4'd5), .b(c_n1), .bi(1'b0), .d(dh_neg_p1), .bo(dh_unused[3]));

		always @(posedge clk or posedge rst_high) begin
			if (rst_high) begin
				s1_qm[c]      <= 9'd0;
				s1_n1_eq4[c]  <= 1'b0;
				s1_n1_gt4[c]  <= 1'b0;
				s1_n1_lt4[c]  <= 1'b0;
				s1_dh_bal[c]  <= 4'd0;
				s1_dh_same[c] <= 4'd0;
				s1_dh_diff[c] <= 4'd0;
			end else begin
				s1_qm[c]      <= s1c_qm[c];
				s1_n1_eq4[c]  <= c_eq4;
				s1_n1_gt4[c]  <= c_gt4;
				s1_n1_lt4[c]  <= c_lt4;
				s1_dh_bal[c]  <= c_q8 ? dh_pos    : dh_neg;
				s1_dh_same[c] <= c_q8 ? dh_neg_p1 : dh_neg;
				s1_dh_diff[c] <= c_q8 ? dh_pos    : dh_pos_m1;
			end
		end

		/* Stage 2: running disparity cnt update with 3 parallel 4-bit adders + mux. */
		wire [8:0] q = s1_qm[c];
		reg [4:0] cnt;	/* running disparity, signed, even */
		reg [9:0] out;
		wire cnt_zero  = ~(|cnt[4:1]);
		wire balanced  = cnt_zero || s1_n1_eq4[c];
		wire same_sign = !cnt_zero && ((!cnt[4] && s1_n1_gt4[c]) || (cnt[4] && s1_n1_lt4[c]));

		wire [3:0] cnt_bal_h, cnt_same_h, cnt_diff_h;
		wire [2:0] cnt_co_unused;
		am_add #(.W(4)) a_cnt_bal  (.a(cnt[4:1]), .b(s1_dh_bal[c]),  .ci(1'b0), .s(cnt_bal_h),  .co(cnt_co_unused[0]));
		am_add #(.W(4)) a_cnt_same (.a(cnt[4:1]), .b(s1_dh_same[c]), .ci(1'b0), .s(cnt_same_h), .co(cnt_co_unused[1]));
		am_add #(.W(4)) a_cnt_diff (.a(cnt[4:1]), .b(s1_dh_diff[c]), .ci(1'b0), .s(cnt_diff_h), .co(cnt_co_unused[2]));

		wire [4:0] cnt_next = {
			balanced  ? cnt_bal_h :
			same_sign ? cnt_same_h :
			            cnt_diff_h,
			1'b0
		};
		assign tmds[10*c +: 10] = out;

		always @(posedge clk or posedge rst_high) begin
			if (rst_high) begin
				cnt <= 5'd0;
				out <= 10'd0;
			end else if (!s1_de) begin
				cnt <= 5'd0;
				case (s1_ctrl[c])
				2'b00: out <= 10'b1101010100;
				2'b01: out <= 10'b0010101011;
				2'b10: out <= 10'b0101010100;
				default: out <= 10'b1010101011;
				endcase
			end else if (balanced) begin
				out <= {!q[8], q[8], q[8] ? q[7:0] : ~q[7:0]};
				cnt <= cnt_next;
			end else if (same_sign) begin
				out <= {1'b1, q[8], ~q[7:0]};
				cnt <= cnt_next;
			end else begin
				out <= {1'b0, q[8], q[7:0]};
				cnt <= cnt_next;
			end
		end

		wire unused_ok = &{1'b0, n1_co_unused, dh_unused, cnt_co_unused};
	end
	endgenerate
endmodule

`default_nettype wire
