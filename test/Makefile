# cocotb + Icarus Verilog flow for the TTPE RTL.
#
#   make                              memory subsystem tests against the PDK SRAM models
#   make TARGET=core                  core (Execution Unit + FIFOs + memory) lockstep tests
#   make TARGET=top                   chip level (tt_um_ttpe) tests, strictly through the pins
#   make TARGET=usb                   USB low-speed bit slicer (ttpe_usb_rx) vs sw/usb_ls.py, per cycle
#   make regress                      lint-all, then all three targets with the PDK models and
#                                     then with the stand-in model
#   make USE_PDK_SRAM=0 [TARGET=...]  use the stand-in macro model in ../sim instead
#   make lint [TARGET=...]            iverilog elaboration with -Wall (no simulation)
#   make lint-all                     lint for mem, core and top, both SRAM models
#   GATES=yes make                    gate-level test of tt_submission netlist (used by the gl_test CI job)
#   make clean
#
# Variables
#   TARGET        mem | core | top | usb      (default mem)
#   USE_PDK_SRAM  1 = vendor models in test/ihp_sram (default), 0 = ../sim stand-in
#
# Each (TARGET, SRAM model) pair has its own build directory and results file, so runs
# never overwrite each other.
#
# Requires: iverilog >= 11 (SystemVerilog packages, always_ff/always_comb), cocotb 1.9+ or 2.x.
# The core and top targets also need Python 3 and ../sw (assembler, ISS and device models; the
# testbenches add ../sw to sys.path themselves, so no PYTHONPATH is needed).

SIM            ?= icarus
TOPLEVEL_LANG   = verilog
TARGET         ?= mem
USE_PDK_SRAM   ?= 1

# Gate-level mode (CI job gl_test): the chip-level pin tests run on tb.v around the netlist that the
# GDS action copies to test/gate_level_netlist.v. Results go to results.xml (what the action reads).
ifeq ($(GATES),yes)
  override TARGET       := top
  override USE_PDK_SRAM := 1
endif

.DEFAULT_GOAL  := sim

RTL_DIR         = $(CURDIR)/../src
SIM_DIR         = $(CURDIR)/../sim
PDK_DIR         = $(CURDIR)/ihp_sram

# ---- SRAM macro model -------------------------------------------------------------------
# The vendor models need -DSIM -DFUNCTIONAL (see the Ken Pettit walkthrough). -DSIM also
# removes the synthesis black box (rtl/sram_512x32_bb.v, which is never listed here).
ifeq ($(USE_PDK_SRAM),1)
  SRAM_SRC  = $(PDK_DIR)/RM_IHPSG13_1P_core_behavioral_bm_bist.v \
              $(PDK_DIR)/RM_IHPSG13_1P_512x32_c2_bm_bist.v
  SRAM_TAG  = pdk
  SRAM_ARGS = -DFUNCTIONAL
else
  SRAM_SRC  = $(SIM_DIR)/RM_IHPSG13_1P_512x32_c2_bm_bist_sim.v
  SRAM_TAG  = stub
  SRAM_ARGS =
endif

# ---- Design under test ------------------------------------------------------------------
ifeq ($(TARGET),mem)
  TOPLEVEL        = ttpe_mem_subsys
  TEST_MODULE     = test_ttpe_mem
  VERILOG_SOURCES = $(RTL_DIR)/ttpe_sram_ctrl.sv \
                    $(RTL_DIR)/ttpe_mem_subsys.sv \
                    $(SRAM_SRC)
else ifeq ($(TARGET),core)
  TOPLEVEL        = ttpe_core
  TEST_MODULE     = test_ttpe_core
  # the package must be compiled before the modules that import it
  VERILOG_SOURCES = $(RTL_DIR)/ttpe_isa_pkg.sv \
                    $(RTL_DIR)/ttpe_sram_ctrl.sv \
                    $(RTL_DIR)/ttpe_mem_subsys.sv \
                    $(RTL_DIR)/ttpe_input_sync.sv \
                    $(RTL_DIR)/ttpe_fifo.sv \
                    $(RTL_DIR)/ttpe_usb_rx.sv \
                    $(RTL_DIR)/ttpe_exec.sv \
                    $(RTL_DIR)/ttpe_core.sv \
                    $(SRAM_SRC)
else ifeq ($(TARGET),usb)
  TOPLEVEL        = ttpe_usb_rx
  TEST_MODULE     = test_ttpe_usb
  VERILOG_SOURCES = $(RTL_DIR)/ttpe_usb_rx.sv
else ifeq ($(TARGET),top)
  TEST_MODULE     = test_ttpe_top
  ifeq ($(GATES),yes)
    TOPLEVEL        = tb
    CELLS_DIR       = $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref
    VERILOG_SOURCES = $(CELLS_DIR)/sg13cmos5l_io/verilog/sg13cmos5l_io.v \
                      $(CELLS_DIR)/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v \
                      $(CELLS_DIR)/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v \
                      $(CURDIR)/gate_level_netlist.v \
                      $(SRAM_SRC) \
                      $(CURDIR)/tb.v
  else
    TOPLEVEL        = tt_um_ttpe
    VERILOG_SOURCES = $(RTL_DIR)/ttpe_isa_pkg.sv \
                      $(RTL_DIR)/ttpe_sram_ctrl.sv \
                      $(RTL_DIR)/ttpe_mem_subsys.sv \
                      $(RTL_DIR)/ttpe_input_sync.sv \
                      $(RTL_DIR)/ttpe_fifo.sv \
                      $(RTL_DIR)/ttpe_usb_rx.sv \
                      $(RTL_DIR)/ttpe_exec.sv \
                      $(RTL_DIR)/ttpe_core.sv \
                      $(RTL_DIR)/ttpe_reset_sync.sv \
                      $(RTL_DIR)/ttpe_spi_phy.sv \
                      $(RTL_DIR)/ttpe_host_spi.sv \
                      $(RTL_DIR)/tt_um_ttpe.sv \
                      $(SRAM_SRC)
  endif
else
  $(error TARGET must be mem, core, top or usb, got '$(TARGET)')
endif

COMPILE_ARGS   += -g2012 -Wall -Wno-timescale -DSIM $(SRAM_ARGS)
ifeq ($(GATES),yes)
  COMPILE_ARGS += -DGL_TEST -DFUNCTIONAL
endif

ifeq ($(GATES),yes)
  SIM_BUILD            = sim_build_gl
else
  SIM_BUILD            = sim_build_$(TARGET)_$(SRAM_TAG)
endif
ifeq ($(GATES),yes)
  COCOTB_RESULTS_FILE := results.xml
else
  COCOTB_RESULTS_FILE  = results_$(TARGET)_$(SRAM_TAG).xml
endif
export COCOTB_RESULTS_FILE

# cocotb 1.x selects tests with MODULE, cocotb 2.x with COCOTB_TEST_MODULES
COCOTB_MAJOR := $(shell cocotb-config --version 2>/dev/null | cut -d. -f1)
ifeq ($(COCOTB_MAJOR),1)
  MODULE = $(TEST_MODULE)
else
  COCOTB_TEST_MODULES = $(TEST_MODULE)
endif

include $(shell cocotb-config --makefiles)/Makefile.sim

.PHONY: lint lint-all regress synth-check clean-all

SYNTH_SRC = $(RTL_DIR)/ttpe_isa_pkg.sv $(RTL_DIR)/ttpe_sram_ctrl.sv $(RTL_DIR)/ttpe_mem_subsys.sv \
            $(RTL_DIR)/ttpe_input_sync.sv $(RTL_DIR)/ttpe_fifo.sv $(RTL_DIR)/ttpe_usb_rx.sv $(RTL_DIR)/ttpe_exec.sv \
            $(RTL_DIR)/ttpe_core.sv $(RTL_DIR)/ttpe_reset_sync.sv $(RTL_DIR)/ttpe_spi_phy.sv \
            $(RTL_DIR)/ttpe_host_spi.sv $(RTL_DIR)/tt_um_ttpe.sv $(RTL_DIR)/sram_512x32_bb.v

lint:
	iverilog -g2012 -Wall -Wno-timescale -DSIM $(SRAM_ARGS) -s $(TOPLEVEL) -o lint_$(TARGET).vvp $(VERILOG_SOURCES)
	rm -f lint_$(TARGET).vvp

lint-all:
	$(MAKE) TARGET=mem  USE_PDK_SRAM=1 lint
	$(MAKE) TARGET=core USE_PDK_SRAM=1 lint
	$(MAKE) TARGET=top  USE_PDK_SRAM=1 lint
	$(MAKE) TARGET=usb  lint
	$(MAKE) TARGET=mem  USE_PDK_SRAM=0 lint
	$(MAKE) TARGET=core USE_PDK_SRAM=0 lint
	$(MAKE) TARGET=top  USE_PDK_SRAM=0 lint

regress: lint-all
	$(MAKE) TARGET=mem  USE_PDK_SRAM=1 sim
	$(MAKE) TARGET=core USE_PDK_SRAM=1 sim
	$(MAKE) TARGET=top  USE_PDK_SRAM=1 sim
	$(MAKE) TARGET=usb  sim
	$(MAKE) TARGET=mem  USE_PDK_SRAM=0 sim
	$(MAKE) TARGET=core USE_PDK_SRAM=0 sim
	$(MAKE) TARGET=top  USE_PDK_SRAM=0 sim

# EXPERIMENTAL and not yet run by the author: parse + elaborate the synthesisable files with
# yosys (black-box SRAM stub, no -DSIM), top = tt_um_ttpe. Whether the LibreLane yosys build
# accepts the package import is something to confirm; if not, flatten the package constants.
synth-check:
	yosys -q -p "read_verilog -sv $(SYNTH_SRC); hierarchy -check -top tt_um_ttpe; proc; opt_clean; stat"

clean-all: clean
	rm -rf sim_build_* results_*.xml results.xml lint_*.vvp __pycache__

# ---- gate-level CI: make the results file pass the gl_test action's check ----------------------
# The action runs `! grep failure results.xml`. cocotb 2.1 writes failures="0" into a passing run,
# which that grep matches. Remove the zero counters; real failures still appear as <failure> elements.
ifeq ($(GATES),yes)
.DEFAULT_GOAL := gl
.PHONY: gl
gl: sim
	sed -i -E 's/ (failures|errors|skipped)="0"//g' $(COCOTB_RESULTS_FILE)
endif
