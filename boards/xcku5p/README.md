# HamGeek XCKU5P Board Support

Target device: `xcku5p-ffvb676-2-i` (Kintex UltraScale+ KU5P, -2 industrial).

## Hardware Spec Targeted
- **DDR4**: 2x `MT40A256M16GE-075E` (32-bit DQ bus, 2666 MT/s, 4:1 PHY = 256-bit AXI4 UI at 333.3 MHz, 1 GiB).
- **PCIe**: FMC PCIe 3.0 x4 (`xdma` in `AXI_Bridge` mode, 128-bit AXI, BAR0 128 KiB regs, BAR2 64 MiB local window, 1 MSI vector).
- **Display**: Parallel 16-bit YCbCr 4:2:2 + syncs (`ODDRE1` pixel clock) for external ADV7511 / FMC HDMI carrier, plus I2C.
- **Clocks**: 200 MHz differential oscillator (`MMCME4_BASE` -> 400 MHz `gclk` default, 25 / 75 MHz `pclk`, 200 MHz DDR4 system clock).

## Current Status
- Verilator `--lint-only` clean against `sim/lint/xilinx_stubs.v`.
- Vivado P&R for `xcku5p` requires Kintex UltraScale+ device data installed in Vivado and carrier-board `PACKAGE_PIN` assignments filled in `atg_xcku5p_top.xdc` (currently left unassigned with explicit placeholders; never guessed).
