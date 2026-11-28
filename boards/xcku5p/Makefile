# HamGeek XCKU5P Vivado non-project build wrapper
# Copyright (C) 2026 Awe Morris; SPDX-License-Identifier: Zlib

ROOT         := $(abspath ../..)
TOP          ?= atg_xcku5p_top
CABLE        ?= digilent
FTDI_SERIAL  ?=
LOADER        = openFPGALoader -c $(CABLE) $(if $(FTDI_SERIAL),--ftdi-serial $(FTDI_SERIAL))

.PHONY: all ip prog

all:
	$(ROOT)/mk/vivado-build.sh xcku5p $(TOP)

ip:
	$(ROOT)/mk/vivado-cmd.sh -mode batch -nojournal -log $(ROOT)/build/vivado/xcku5p_ip/ip.log \
		-source $(ROOT)/boards/xcku5p/ip.tcl -tclargs $(ROOT)/build/vivado/xcku5p_ip

prog:
	$(LOADER) $(ROOT)/build/vivado/$(TOP)/$(TOP).bit
