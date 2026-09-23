# Timing constraints for tt_um_ofek9993_protoemu (IHP sg13cmos5l).
#
# One source of truth for the clock: the period comes from CLOCK_PERIOD in
# src/config.json (20 ns = 50 MHz, matching clock_hz in info.yaml).

set clk_period $::env(CLOCK_PERIOD)
create_clock -name clk -period $clk_period [get_ports {clk}]

# Clock quality. The clock reaches the chip from the demo board through the
# Tiny Tapeout mux; its jitter is unknown, so keep a margin: 0.5 ns (2.5% of
# the period) for setup, 0.1 ns for hold.
set_clock_uncertainty -setup 0.5 [get_clocks {clk}]
set_clock_uncertainty -hold  0.1 [get_clocks {clk}]
set_clock_transition 0.15 [get_clocks {clk}]

# I/O: 20% of the period on each side. Every protocol input and the config
# port go through 2-flop synchronisers and every output is a flop, so these
# are ordinary register-to-pad paths.
set io_delay [expr {$clk_period * 0.2}]
set_input_delay  $io_delay -clock [get_clocks {clk}] [all_inputs -no_clocks]
set_output_delay $io_delay -clock [get_clocks {clk}] [all_outputs]

set_driving_cell -lib_cell sg13cmos5l_buf_4 -pin {X} [all_inputs]
set_load 0.006 [all_outputs]

# On-chip variation margin.
set_timing_derate -early 0.95
set_timing_derate -late  1.05

set_max_fanout 10 [current_design]
