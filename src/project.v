/*
 * Copyright (c) 2026 ofek9993
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// Protocol emulator - Jane Street ASIC competition entry.
//
// Three layers (see ARCHITECTURE.md):
//   * a programmable state-map CONTROLLER (pemu_ctrl.v): the host loads a
//     state table - the protocol - and the controller runs it alone:
//     chip-select, START/STOP, ACK checks, retries, timeouts;
//   * a DATAPATH of 2 timers + 2 shifters wired by configuration, after
//     NXP's FlexIO (AN5034 UART, AN5133 I2C, AN12780 SPI) - bits at a
//     steady rate;
//   * a PIN LAYER (pemu_pins.v): synchronisers, glitch filter, registered
//     outputs, safe reset levels.
// UART, SPI and I2C are tables plus register values, not fixed logic.
//
//   ui_in[0]     CFG_SCK   config shift clock
//   ui_in[1]     CFG_SDI   config data in, MSB first
//   ui_in[2]     CFG_CS    low while shifting, rising edge commits
//   ui_in[6:3]   protocol inputs   (logical pins 8..11; UART RX = ui_in[3])
//   uio[7:0]     protocol pins     (logical pins 0..7, in / out / open-drain)
//   uo_out[0]    CFG_SDO   config read-back
//   uo_out[1]    READY     the controller has finished
//   uo_out[7:4]  protocol outputs  (logical pins 12..15; UART TX = uo_out[4])
//
// A config write is 16 bits: {addr[7:0], data[7:0]}.
module tt_um_ofek9993_protoemu (
    input  wire [7:0] ui_in,    // Dedicated inputs  - config port + protocol inputs
    output wire [7:0] uo_out,   // Dedicated outputs - SDO, READY, protocol outputs
    input  wire [7:0] uio_in,   // IOs: Input path   - protocol pins
    output wire [7:0] uio_out,  // IOs: Output path  - protocol pins
    output wire [7:0] uio_oe,   // IOs: Enable path  - per-pin drive
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Reset synchroniser: reset ASSERTS immediately (asynchronously), but is
  // RELEASED only on a clock edge, two flops after rst_n goes high. rst_n
  // comes from outside the chip at an arbitrary moment; released raw, flops
  // could leave reset on different clock edges and start out of step.
  reg [1:0] rst_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b00;
    else        rst_sync <= {rst_sync[0], 1'b1};
  end
  wire rst_n_sync = rst_sync[1];

  pemu_core core (
      .clk     (clk),
      .rst_n   (rst_n_sync),
      .ui_in   (ui_in),
      .uo_out  (uo_out),
      .uio_in  (uio_in),
      .uio_out (uio_out),
      .uio_oe  (uio_oe)
  );

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

endmodule

`default_nettype wire
