/*
 * Amaterrace GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg_zcu106_top: Top-level board support package for the AMD / Xilinx
 * Zynq UltraScale+ ZCU106 Evaluation Kit (xczu7ev-ffvc1156-2-e).
 *
 * Architectural Features:
 *   - Core Logic Clock: 400.0 MHz (period 2.500 ns) synthesized from PS pl_clk0
 *     (100.0 MHz) via UltraScale+ MMCME4_BASE (atg_zcu106_clk).
 *   - Memory Architecture: Unified Memory Architecture (UMA) via Zynq MPSoC
 *     High-Performance slave port S_AXI_HP0_FPD (128-bit @ clk_axi 200 MHz),
 *     bridged from clk_gpu (400 MHz) through atg_zcu106_axi_cc.
 *   - Host Interface: M_AXI_HPM0_FPD (64-bit @ clk_axi 200 MHz) bridged into
 *     clk_gpu via am_axi_cdc.
 *   - Parameterized via include/atg_config.vh (e.g. config/gfx-fpga-test-minimal.mk).
 */

`timescale 1ns / 1ps
`default_nettype none

`include "atg_config.vh"

`ifndef CFG_SU_NPHYS
`define CFG_SU_NPHYS 4
`endif
`ifndef CFG_SU_NWARPS
`define CFG_SU_NWARPS 4
`endif
`ifndef CFG_SU_GRP_W
`define CFG_SU_GRP_W 3
`endif
`ifndef CFG_SU_NREGS
`define CFG_SU_NREGS 32
`endif
`ifndef CFG_SU_LDS_KB
`define CFG_SU_LDS_KB 1
`endif
`ifndef CFG_MC_L2_CACHE_KB
`define CFG_MC_L2_CACHE_KB 4
`endif
`ifndef CFG_MC_L2_WAYS
`define CFG_MC_L2_WAYS 2
`endif

module atg_zcu106_top #(
	parameter S_ID_W = 16,		/* PS M_AXI_HPM0_FPD AxID width */
	parameter S_LEN_W = 8,		/* AXI4 burst length width */
	parameter DATA_W = 128,		/* S_AXI_HP0_FPD native 128-bit data bus */
	parameter LINE_BYTES = 64,	/* 64-byte cache line */
	parameter LINE_SHIFT = 6,	/* log2(LINE_BYTES) */
	parameter NSU = 1
) (
	output wire [15:0] hdmi_d,
	output wire hdmi_de,
	output wire hdmi_hs,
	output wire hdmi_vs,
	output wire hdmi_clk,
	inout wire iic_scl,
	inout wire iic_sda,
	output wire [7:0] led
);
	/* ------------------------------------------------------------------ */
	/* 1. Zynq UltraScale+ PS & MMCM Clocks                               */
	/* ------------------------------------------------------------------ */
	wire pl_clk0;
	wire pl_resetn0;
	wire clk_gpu, rst_n_gpu;
	wire clk_axi, rst_n_axi;
	wire pclk, prst_n;
	wire mmcm_locked;

	atg_zcu106_clk #(
		.CLKIN_PERIOD(10.000),		/* 100 MHz pl_clk0 */
		.CLKFBOUT_MULT(12.000),		/* VCO = 1200 MHz */
		.CLKOUT0_DIV(3.000),		/* 400 MHz clk_gpu */
		.CLKOUT1_DIV(6),		/* 200 MHz clk_axi */
		.CLKOUT2_DIV(16)		/* 75 MHz pclk */
	) u_clk (
		.clk_in(pl_clk0),
		.rst_in(!pl_resetn0),
		.clk_gpu(clk_gpu),
		.rst_n_gpu(rst_n_gpu),
		.clk_axi(clk_axi),
		.rst_n_axi(rst_n_axi),
		.pclk(pclk),
		.prst_n(prst_n),
		.locked(mmcm_locked)
	);

	/* ------------------------------------------------------------------ */
	/* 2. PS AXI Interfaces (clk_axi) & CDC Wires                         */
	/* ------------------------------------------------------------------ */
	wire [39:0] ps_awaddr, ps_araddr;
	wire [S_ID_W-1:0] ps_awid, ps_bid, ps_arid, ps_rid;
	wire [7:0] ps_awlen, ps_arlen;
	wire ps_awvalid, ps_awready;
	wire [63:0] ps_wdata, ps_rdata;
	wire [7:0] ps_wstrb;
	wire ps_wlast, ps_wvalid, ps_wready;
	wire [1:0] ps_bresp, ps_rresp;
	wire ps_bvalid, ps_bready;
	wire ps_arvalid, ps_arready, ps_rlast, ps_rvalid, ps_rready;

	wire [16:0] gpu_s_awaddr, gpu_s_araddr;
	wire [S_ID_W-1:0] gpu_s_awid, gpu_s_bid, gpu_s_arid, gpu_s_rid;
	wire [S_LEN_W-1:0] gpu_s_arlen;
	wire gpu_s_awvalid, gpu_s_awready;
	wire [63:0] gpu_s_wdata, gpu_s_rdata;
	wire [7:0] gpu_s_wstrb;
	wire gpu_s_wlast, gpu_s_wvalid, gpu_s_wready;
	wire [1:0] gpu_s_bresp, gpu_s_rresp;
	wire gpu_s_bvalid, gpu_s_bready;
	wire gpu_s_arvalid, gpu_s_arready, gpu_s_rlast, gpu_s_rvalid, gpu_s_rready;

	wire gpu_m_awid, gpu_m_bid, gpu_m_arid, gpu_m_rid;
	wire [63:0] gpu_m_awaddr, gpu_m_araddr;
	wire [7:0] gpu_m_awlen, gpu_m_arlen;
	wire [2:0] gpu_m_awsize, gpu_m_arsize, gpu_m_awprot, gpu_m_arprot;
	wire [1:0] gpu_m_awburst, gpu_m_arburst, gpu_m_bresp, gpu_m_rresp;
	wire [3:0] gpu_m_awcache, gpu_m_arcache;
	wire gpu_m_awvalid, gpu_m_awready, gpu_m_wlast, gpu_m_wvalid, gpu_m_wready;
	wire gpu_m_bvalid, gpu_m_bready, gpu_m_arvalid, gpu_m_arready;
	wire gpu_m_rlast, gpu_m_rvalid, gpu_m_rready;
	wire [DATA_W-1:0] gpu_m_wdata, gpu_m_rdata;
	wire [DATA_W/8-1:0] gpu_m_wstrb;

	wire cc_m_awid, cc_m_arid;
	wire [5:0] cc_m_bid6, cc_m_rid6;
	wire [48:0] cc_m_awaddr, cc_m_araddr;
	wire [7:0] cc_m_awlen, cc_m_arlen;
	wire [2:0] cc_m_awsize, cc_m_arsize, cc_m_awprot, cc_m_arprot;
	wire [1:0] cc_m_awburst, cc_m_arburst, cc_m_bresp, cc_m_rresp;
	wire [3:0] cc_m_awcache, cc_m_arcache, cc_m_awqos, cc_m_arqos;
	wire cc_m_awlock, cc_m_arlock;
	wire cc_m_awvalid, cc_m_awready, cc_m_wlast, cc_m_wvalid, cc_m_wready;
	wire cc_m_bvalid, cc_m_bready, cc_m_arvalid, cc_m_arready;
	wire cc_m_rlast, cc_m_rvalid, cc_m_rready;
	wire [DATA_W-1:0] cc_m_wdata, cc_m_rdata;
	wire [DATA_W/8-1:0] cc_m_wstrb;

	wire gpu_irq, pl_ps_irq0;
	wire [29:0] tmds;
	wire [23:0] px_rgb;
	wire scl_t, sda_t;

	atg_zcu106_ps u_ps (
		.pl_clk0(pl_clk0),
		.pl_resetn0(pl_resetn0),
		.pl_ps_irq0(pl_ps_irq0),

		.maxihpm0_fpd_aclk(clk_axi),
		.maxigp0_awid(ps_awid),
		.maxigp0_awaddr(ps_awaddr),
		.maxigp0_awlen(ps_awlen),
		.maxigp0_awsize(),
		.maxigp0_awburst(),
		.maxigp0_awlock(),
		.maxigp0_awcache(),
		.maxigp0_awprot(),
		.maxigp0_awvalid(ps_awvalid),
		.maxigp0_awuser(),
		.maxigp0_awready(ps_awready),
		.maxigp0_wdata(ps_wdata),
		.maxigp0_wstrb(ps_wstrb),
		.maxigp0_wlast(ps_wlast),
		.maxigp0_wvalid(ps_wvalid),
		.maxigp0_wready(ps_wready),
		.maxigp0_bid(ps_bid),
		.maxigp0_bresp(ps_bresp),
		.maxigp0_bvalid(ps_bvalid),
		.maxigp0_bready(ps_bready),
		.maxigp0_arid(ps_arid),
		.maxigp0_araddr(ps_araddr),
		.maxigp0_arlen(ps_arlen),
		.maxigp0_arsize(),
		.maxigp0_arburst(),
		.maxigp0_arlock(),
		.maxigp0_arcache(),
		.maxigp0_arprot(),
		.maxigp0_arvalid(ps_arvalid),
		.maxigp0_aruser(),
		.maxigp0_arready(ps_arready),
		.maxigp0_rid(ps_rid),
		.maxigp0_rdata(ps_rdata),
		.maxigp0_rresp(ps_rresp),
		.maxigp0_rlast(ps_rlast),
		.maxigp0_rvalid(ps_rvalid),
		.maxigp0_rready(ps_rready),
		.maxigp0_awqos(),
		.maxigp0_arqos(),

		.saxihp0_fpd_aclk(clk_axi),
		.saxigp2_aruser(1'b0),
		.saxigp2_awuser(1'b0),
		.saxigp2_awid({5'd0, cc_m_awid}),
		.saxigp2_awaddr(cc_m_awaddr),
		.saxigp2_awlen(cc_m_awlen),
		.saxigp2_awsize(cc_m_awsize),
		.saxigp2_awburst(cc_m_awburst),
		.saxigp2_awlock(cc_m_awlock),
		.saxigp2_awcache(cc_m_awcache),
		.saxigp2_awprot(cc_m_awprot),
		.saxigp2_awvalid(cc_m_awvalid),
		.saxigp2_awready(cc_m_awready),
		.saxigp2_wdata(cc_m_wdata),
		.saxigp2_wstrb(cc_m_wstrb),
		.saxigp2_wlast(cc_m_wlast),
		.saxigp2_wvalid(cc_m_wvalid),
		.saxigp2_wready(cc_m_wready),
		.saxigp2_bid(cc_m_bid6),
		.saxigp2_bresp(cc_m_bresp),
		.saxigp2_bvalid(cc_m_bvalid),
		.saxigp2_bready(cc_m_bready),
		.saxigp2_arid({5'd0, cc_m_arid}),
		.saxigp2_araddr(cc_m_araddr),
		.saxigp2_arlen(cc_m_arlen),
		.saxigp2_arsize(cc_m_arsize),
		.saxigp2_arburst(cc_m_arburst),
		.saxigp2_arlock(cc_m_arlock),
		.saxigp2_arcache(cc_m_arcache),
		.saxigp2_arprot(cc_m_arprot),
		.saxigp2_arvalid(cc_m_arvalid),
		.saxigp2_arready(cc_m_arready),
		.saxigp2_rid(cc_m_rid6),
		.saxigp2_rdata(cc_m_rdata),
		.saxigp2_rresp(cc_m_rresp),
		.saxigp2_rlast(cc_m_rlast),
		.saxigp2_rvalid(cc_m_rvalid),
		.saxigp2_rready(cc_m_rready),
		.saxigp2_awqos(cc_m_awqos),
		.saxigp2_arqos(cc_m_arqos)
	);

	/* ------------------------------------------------------------------ */
	/* 3. MMIO BAR0 CDC (clk_axi -> clk_gpu) & DMA Clock Converter        */
	/* ------------------------------------------------------------------ */
	am_axi_cdc #(
		.A_W(17),
		.ID_W(S_ID_W),
		.LEN_W(S_LEN_W),
		.D_W(64),
		.DEPTH_LOG2(2)
	) u_mmio_cdc (
		.s_clk(clk_axi),
		.s_rst_n(rst_n_axi),
		.s_awaddr(ps_awaddr[16:0]),
		.s_awid(ps_awid),
		.s_awvalid(ps_awvalid),
		.s_awready(ps_awready),
		.s_wdata(ps_wdata),
		.s_wstrb(ps_wstrb),
		.s_wlast(ps_wlast),
		.s_wvalid(ps_wvalid),
		.s_wready(ps_wready),
		.s_bid(ps_bid),
		.s_bresp(ps_bresp),
		.s_bvalid(ps_bvalid),
		.s_bready(ps_bready),
		.s_araddr(ps_araddr[16:0]),
		.s_arid(ps_arid),
		.s_arlen(ps_arlen),
		.s_arvalid(ps_arvalid),
		.s_arready(ps_arready),
		.s_rdata(ps_rdata),
		.s_rid(ps_rid),
		.s_rresp(ps_rresp),
		.s_rlast(ps_rlast),
		.s_rvalid(ps_rvalid),
		.s_rready(ps_rready),

		.m_clk(clk_gpu),
		.m_rst_n(rst_n_gpu),
		.m_awaddr(gpu_s_awaddr),
		.m_awid(gpu_s_awid),
		.m_awvalid(gpu_s_awvalid),
		.m_awready(gpu_s_awready),
		.m_wdata(gpu_s_wdata),
		.m_wstrb(gpu_s_wstrb),
		.m_wlast(gpu_s_wlast),
		.m_wvalid(gpu_s_wvalid),
		.m_wready(gpu_s_wready),
		.m_bid(gpu_s_bid),
		.m_bresp(gpu_s_bresp),
		.m_bvalid(gpu_s_bvalid),
		.m_bready(gpu_s_bready),
		.m_araddr(gpu_s_araddr),
		.m_arid(gpu_s_arid),
		.m_arlen(gpu_s_arlen),
		.m_arvalid(gpu_s_arvalid),
		.m_arready(gpu_s_arready),
		.m_rdata(gpu_s_rdata),
		.m_rid(gpu_s_rid),
		.m_rresp(gpu_s_rresp),
		.m_rlast(gpu_s_rlast),
		.m_rvalid(gpu_s_rvalid),
		.m_rready(gpu_s_rready)
	);

	atg_zcu106_axi_cc u_mem_cc (
		.s_axi_aclk(clk_gpu),
		.s_axi_aresetn(rst_n_gpu),
		.s_axi_awid(gpu_m_awid),
		.s_axi_awaddr(gpu_m_awaddr[48:0]),
		.s_axi_awlen(gpu_m_awlen),
		.s_axi_awsize(gpu_m_awsize),
		.s_axi_awburst(gpu_m_awburst),
		.s_axi_awlock(1'b0),
		.s_axi_awcache(gpu_m_awcache),
		.s_axi_awprot(gpu_m_awprot),
		.s_axi_awregion(4'd0),
		.s_axi_awqos(4'd0),
		.s_axi_awvalid(gpu_m_awvalid),
		.s_axi_awready(gpu_m_awready),
		.s_axi_wdata(gpu_m_wdata),
		.s_axi_wstrb(gpu_m_wstrb),
		.s_axi_wlast(gpu_m_wlast),
		.s_axi_wvalid(gpu_m_wvalid),
		.s_axi_wready(gpu_m_wready),
		.s_axi_bid(gpu_m_bid),
		.s_axi_bresp(gpu_m_bresp),
		.s_axi_bvalid(gpu_m_bvalid),
		.s_axi_bready(gpu_m_bready),
		.s_axi_arid(gpu_m_arid),
		.s_axi_araddr(gpu_m_araddr[48:0]),
		.s_axi_arlen(gpu_m_arlen),
		.s_axi_arsize(gpu_m_arsize),
		.s_axi_arburst(gpu_m_arburst),
		.s_axi_arlock(1'b0),
		.s_axi_arcache(gpu_m_arcache),
		.s_axi_arprot(gpu_m_arprot),
		.s_axi_arregion(4'd0),
		.s_axi_arqos(4'd0),
		.s_axi_arvalid(gpu_m_arvalid),
		.s_axi_arready(gpu_m_arready),
		.s_axi_rid(gpu_m_rid),
		.s_axi_rdata(gpu_m_rdata),
		.s_axi_rresp(gpu_m_rresp),
		.s_axi_rlast(gpu_m_rlast),
		.s_axi_rvalid(gpu_m_rvalid),
		.s_axi_rready(gpu_m_rready),

		.m_axi_aclk(clk_axi),
		.m_axi_aresetn(rst_n_axi),
		.m_axi_awid(cc_m_awid),
		.m_axi_awaddr(cc_m_awaddr),
		.m_axi_awlen(cc_m_awlen),
		.m_axi_awsize(cc_m_awsize),
		.m_axi_awburst(cc_m_awburst),
		.m_axi_awlock(cc_m_awlock),
		.m_axi_awcache(cc_m_awcache),
		.m_axi_awprot(cc_m_awprot),
		.m_axi_awregion(),
		.m_axi_awqos(cc_m_awqos),
		.m_axi_awvalid(cc_m_awvalid),
		.m_axi_awready(cc_m_awready),
		.m_axi_wdata(cc_m_wdata),
		.m_axi_wstrb(cc_m_wstrb),
		.m_axi_wlast(cc_m_wlast),
		.m_axi_wvalid(cc_m_wvalid),
		.m_axi_wready(cc_m_wready),
		.m_axi_bid(cc_m_bid6[0]),
		.m_axi_bresp(cc_m_bresp),
		.m_axi_bvalid(cc_m_bvalid),
		.m_axi_bready(cc_m_bready),
		.m_axi_arid(cc_m_arid),
		.m_axi_araddr(cc_m_araddr),
		.m_axi_arlen(cc_m_arlen),
		.m_axi_arsize(cc_m_arsize),
		.m_axi_arburst(cc_m_arburst),
		.m_axi_arlock(cc_m_arlock),
		.m_axi_arcache(cc_m_arcache),
		.m_axi_arprot(cc_m_arprot),
		.m_axi_arregion(),
		.m_axi_arqos(cc_m_arqos),
		.m_axi_arvalid(cc_m_arvalid),
		.m_axi_arready(cc_m_arready),
		.m_axi_rid(cc_m_rid6[0]),
		.m_axi_rdata(cc_m_rdata),
		.m_axi_rresp(cc_m_rresp),
		.m_axi_rlast(cc_m_rlast),
		.m_axi_rvalid(cc_m_rvalid),
		.m_axi_rready(cc_m_rready)
	);

	/* ------------------------------------------------------------------ */
	/* 4. Lumiere GPU Core Instance (atg_top, configured via atg_config)  */
	/* ------------------------------------------------------------------ */
	atg_top #(
		.GCLK_HZ(32'd400000000),
		.S_ID_W(S_ID_W),
		.S_LEN_W(S_LEN_W),
		.LINE_BYTES(LINE_BYTES),
		.LINE_SHIFT(LINE_SHIFT),
		.DATA_W(DATA_W),
		.LOCAL_BASE(32'd0),
		.LOCAL_BYTES(32'd0),
		.WINDOW_BYTES(32'd0),
		.HOST_LIMIT(32'd0),
		.HAS_CPU(`ifdef CFG_CP_USE_RV32_CORE (`CFG_CP_USE_RV32_CORE != 0) `else 0 `endif),
		.NSU(`ifdef CFG_NSU `CFG_NSU `else NSU `endif),
		.USE_NOC_FABRIC(`ifdef CFG_USE_NOC_FABRIC (`CFG_USE_NOC_FABRIC != 0) `else 0 `endif),
		.HAS_VF(`ifdef CFG_HAS_VF (`CFG_HAS_VF != 0) `else 1 `endif),
		.HAS_RX(`ifdef CFG_HAS_RX (`CFG_HAS_RX != 0) `else 1 `endif),
		.HAS_TU(`ifdef CFG_HAS_TU (`CFG_HAS_TU != 0) `else 1 `endif),
		.HAS_ROP(`ifdef CFG_HAS_ROP (`CFG_HAS_ROP != 0) `else 1 `endif),
		.HAS_BLT(`ifdef CFG_HAS_BLT (`CFG_HAS_BLT != 0) `else 0 `endif),
		.HAS_SA(0),
		.EXT_CLIENT(0),
		.HAS_JTAG(0),
		.HAS_DISP(`ifdef CFG_HAS_DISP (`CFG_HAS_DISP != 0) `else 1 `endif),
		.DC_HAS_I2C(1),
`ifdef CFG_DC_ENABLE_CURSOR
		.DC_HAS_CURSOR(`CFG_DC_ENABLE_CURSOR != 0),
`else
		.DC_HAS_CURSOR(0),
`endif
		.DC_FIFO_LOG2(11),
		.DC_PX_FMT(1),
		.ROP_STENCIL(1),
		.SU_NPHYS(`CFG_SU_NPHYS),
		.SU_NWARPS(`CFG_SU_NWARPS),
		.SU_WARP_W($clog2(`CFG_SU_NWARPS)),
		.SU_GRP_W(`CFG_SU_GRP_W),
		.SU_NREGS(`CFG_SU_NREGS),
		.SU_SB_ENTRIES(4),
		.SU_SBE_W(2),
`ifdef CFG_ENABLE_FP16
		.SU_PACKED(1),
`else
		.SU_PACKED(0),
`endif
		.SU_PK_SHARED(0),
`ifdef CFG_ENABLE_SUBGROUP
		.SU_SUBGROUP(1),
`else
		.SU_SUBGROUP(0),
`endif
		.SU_FRAG_RESERVE(1),
		.SU_LDS_BYTES(`CFG_SU_LDS_KB * 1024),
		.SU_VARY_WORDS(16),
`ifdef CFG_MC_ENABLE_PROT
		.HAS_PROT(`CFG_MC_ENABLE_PROT),
`else
		.HAS_PROT(0),
`endif
		.HAS_L2(`CFG_MC_L2_CACHE_KB != 0 ? 1 : 0),
		.L2_WAYS(`CFG_MC_L2_WAYS),
		.L2_SET_W(`CFG_MC_L2_CACHE_KB != 0 ? $clog2(((`CFG_MC_L2_CACHE_KB != 0 ? `CFG_MC_L2_CACHE_KB : 16) * 1024) / (64 * `CFG_MC_L2_WAYS)) : 8)
	) gpu (
		.clk(clk_gpu),
		.rst_n(rst_n_gpu),
		.irq(gpu_irq),
		.mem_ready(mmcm_locked),
		.win_en(),

		.s_awaddr(gpu_s_awaddr),
		.s_awid(gpu_s_awid),
		.s_awvalid(gpu_s_awvalid),
		.s_awready(gpu_s_awready),
		.s_wdata(gpu_s_wdata),
		.s_wstrb(gpu_s_wstrb),
		.s_wlast(gpu_s_wlast),
		.s_wvalid(gpu_s_wvalid),
		.s_wready(gpu_s_wready),
		.s_bid(gpu_s_bid),
		.s_bresp(gpu_s_bresp),
		.s_bvalid(gpu_s_bvalid),
		.s_bready(gpu_s_bready),
		.s_araddr(gpu_s_araddr),
		.s_arid(gpu_s_arid),
		.s_arlen(gpu_s_arlen),
		.s_arvalid(gpu_s_arvalid),
		.s_arready(gpu_s_arready),
		.s_rdata(gpu_s_rdata),
		.s_rid(gpu_s_rid),
		.s_rresp(gpu_s_rresp),
		.s_rlast(gpu_s_rlast),
		.s_rvalid(gpu_s_rvalid),
		.s_rready(gpu_s_rready),

		.m_awid(gpu_m_awid),
		.m_awaddr(gpu_m_awaddr),
		.m_awlen(gpu_m_awlen),
		.m_awsize(gpu_m_awsize),
		.m_awburst(gpu_m_awburst),
		.m_awcache(gpu_m_awcache),
		.m_awprot(gpu_m_awprot),
		.m_awvalid(gpu_m_awvalid),
		.m_awready(gpu_m_awready),
		.m_wdata(gpu_m_wdata),
		.m_wstrb(gpu_m_wstrb),
		.m_wlast(gpu_m_wlast),
		.m_wvalid(gpu_m_wvalid),
		.m_wready(gpu_m_wready),
		.m_bid(gpu_m_bid),
		.m_bresp(gpu_m_bresp),
		.m_bvalid(gpu_m_bvalid),
		.m_bready(gpu_m_bready),
		.m_arid(gpu_m_arid),
		.m_araddr(gpu_m_araddr),
		.m_arlen(gpu_m_arlen),
		.m_arsize(gpu_m_arsize),
		.m_arburst(gpu_m_arburst),
		.m_arcache(gpu_m_arcache),
		.m_arprot(gpu_m_arprot),
		.m_arvalid(gpu_m_arvalid),
		.m_arready(gpu_m_arready),
		.m_rid(gpu_m_rid),
		.m_rdata(gpu_m_rdata),
		.m_rresp(gpu_m_rresp),
		.m_rlast(gpu_m_rlast),
		.m_rvalid(gpu_m_rvalid),
		.m_rready(gpu_m_rready),

		.xr_wr(),
		.xr_waddr(),
		.xr_wdata(),
		.xr_wstrb(),
		.xr_rd(),
		.xr_raddr(),
		.xr_rdata(32'd0),
		.x_req_valid(1'b0),
		.x_req_ready(),
		.x_req_write(1'b0),
		.x_req_line({(32-LINE_SHIFT){1'b0}}),
		.x_req_be({LINE_BYTES{1'b0}}),
		.x_req_wdata({(LINE_BYTES*8){1'b0}}),
		.x_req_tag(8'd0),
		.x_rsp_valid(),
		.x_rsp_write(),
		.x_rsp_tag(),
		.x_rsp_rdata(),
		.x_rsp_error(),
		.x_vsync(1'b0),

		.j_wr(1'b0),
		.j_waddr(17'd0),
		.j_wdata(32'd0),
		.j_wstrb(4'd0),
		.j_rd(1'b0),
		.j_raddr(17'd0),
		.j_rdata(),
		.j_busy(),

		.pclk(pclk),
		.prst_n(prst_n),
		.pclk_sel(),
		.tmds(tmds),
		.px_rgb(px_rgb),
		.px_de(hdmi_de),
		.px_hs(hdmi_hs),
		.px_vs(hdmi_vs),
		.scl_t(scl_t),
		.scl_i(iic_scl),
		.sda_t(sda_t),
		.sda_i(iic_sda)
	);

	/* ------------------------------------------------------------------ */
	/* 5. Display Parallel Output, I2C, Interrupt Sync & LEDs             */
	/* ------------------------------------------------------------------ */
	assign iic_scl = scl_t ? 1'bz : 1'b0;
	assign iic_sda = sda_t ? 1'bz : 1'b0;
	assign hdmi_d  = px_rgb[15:0];

	ODDRE1 #(.SIM_DEVICE("ULTRASCALE_PLUS")) hdmi_clk_out (
		.Q(hdmi_clk), .C(pclk), .D1(1'b0), .D2(1'b1), .SR(1'b0)
	);

	am_sync u_sync_irq (
		.clk(clk_axi),
		.rst_n(rst_n_axi),
		.d(gpu_irq),
		.q(pl_ps_irq0)
	);

	assign led[0] = mmcm_locked;
	assign led[1] = rst_n_gpu;
	assign led[2] = gpu_irq;
	assign led[3] = ps_awvalid || ps_arvalid;
	assign led[4] = gpu_m_awvalid || gpu_m_arvalid;
	assign led[5] = ps_wvalid || ps_rvalid;
	assign led[6] = gpu_m_wvalid || gpu_m_rvalid;
	assign led[7] = 1'b0;

	wire unused_ok = &{1'b0, tmds, px_rgb[23:16], ps_awaddr[39:17], ps_araddr[39:17],
		gpu_m_awaddr[63:49], gpu_m_araddr[63:49], cc_m_bid6[5:1], cc_m_rid6[5:1]};

endmodule

`default_nettype wire
