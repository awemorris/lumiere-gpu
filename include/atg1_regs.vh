/*
 * Lumiere GPU
 * Copyright (C) 2026 Awe Morris
 *
 * SPDX-License-Identifier: Zlib
 */

/*
 * atg1_regs.vh: Lumiere GPU (ATG1) vendor CPFW control-path window
 * definitions (0x170..0x17F and 0x1A0..0x1BF), moved from ghci_regs.vh
 * because CPFW register contents are vendor-defined (ghci.md §4).
 */

/* verilator lint_off UNUSEDPARAM */

localparam [16:0] GHCI_CPFW_CTRL    = 17'h00170;
localparam [16:0] GHCI_CPFW_STATUS  = 17'h00174;
localparam [16:0] GHCI_CPFW_VERSION = 17'h00178;
localparam [16:0] GHCI_CPFW_IBASE   = 17'h001A0;
localparam [16:0] GHCI_CPFW_IWORDS  = 17'h001A4;
localparam [16:0] GHCI_CPFW_DBASE   = 17'h001A8;
localparam [16:0] GHCI_CPFW_DWORDS  = 17'h001AC;
localparam [16:0] GHCI_CPFW_MBOX0   = 17'h001B0;

/* CPFW_CTRL and CPFW_STATUS bits. */
localparam GHCI_CPFW_CTRL_LOAD    = 0;
localparam GHCI_CPFW_CTRL_RUN     = 1;
localparam GHCI_CPFW_CTRL_HALT    = 2;
localparam GHCI_CPFW_STATUS_READY   = 0;
localparam GHCI_CPFW_STATUS_FAULT   = 1;
localparam GHCI_CPFW_STATUS_ALIVE   = 2;
localparam GHCI_CPFW_STATUS_STOPPED = 3;	/* the core is stopped or held in reset */
localparam GHCI_CPFW_STATUS_LOADING = 4;

/* verilator lint_on UNUSEDPARAM */
