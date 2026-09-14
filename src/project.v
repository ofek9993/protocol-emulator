/*
 * Copyright (c) 2026 ofek9993
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// 8x8 multiply-accumulate, used as a workstation stress test.
//   - the multiplier array gives deep combinational logic
//   - the 16-bit accumulator gives the clock tree real sequential load
//   - uo_out exposes acc[15:8], so every partial product reaches an
//     observable output and none of the multiplier can be optimised away
//
// Reset is ASYNCHRONOUS: it maps directly onto the RESET_B pin of the
// sg13cmos5l_dfrbpq_1 flop, so the accumulator leaves X immediately at
// gate level rather than waiting for a clock edge.
module tt_um_ofek9993_protoemu (
    input  wire [7:0] ui_in,    // Dedicated inputs  - operand A
    output wire [7:0] uo_out,   // Dedicated outputs - accumulator [15:8]
    input  wire [7:0] uio_in,   // IOs: Input path   - operand B
    output wire [7:0] uio_out,  // IOs: Output path  - unused
    output wire [7:0] uio_oe,   // IOs: Enable path  - all inputs
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  wire [15:0] product = ui_in * uio_in;
  reg  [15:0] acc;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) acc <= 16'd0;
    else        acc <= acc + product;
  end

  assign uo_out  = acc[15:8];
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;  // uio used as input (operand B)

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

endmodule
