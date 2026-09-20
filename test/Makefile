# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
FST ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog
SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = project.v sram_wrapper.v write_pointer.v

# The SRAM is a hard macro: it is a black box to synthesis, so its
# behavioural model must be supplied to the simulator in both RTL and
# gate-level runs (the netlist only carries an instance of it).
# The PDK cell and macro models select a clean behavioural view under
# `ifdef FUNCTIONAL. Without it they fall back to a timing-check view
# driven by delayed_* signals that iverilog never drives, so every
# output reads X. Needed for RTL as well as gate-level.
COMPILE_ARGS    += -DFUNCTIONAL

MACRO_DIR = $(PWD)/../macro/RM_IHPSG13_1P_1024x8_c2_bm_bist
# the wrapper model instantiates SRAM_1P_behavioral_bm_bist, which the
# PDK keeps in a separate core file - both are needed to elaborate
VERILOG_SOURCES += $(MACRO_DIR)/RM_IHPSG13_1P_core_behavioral_bm_bist.v
VERILOG_SOURCES += $(MACRO_DIR)/RM_IHPSG13_1P_1024x8_c2_bm_bist.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DSIM
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_io/verilog/sg13cmos5l_io.v
# sg13cmos5l_stdcell.v instantiates primitives (ihp_dff_r, ihp_mux2, ...) that
# live in sg13cmos5l_udp.v; without it any design with flops fails to elaborate.
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_udp.v
VERILOG_SOURCES += $(PDK_ROOT)/ihp-sg13cmos5l/libs.ref/sg13cmos5l_stdcell/verilog/sg13cmos5l_stdcell.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
