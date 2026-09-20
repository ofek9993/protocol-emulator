/*
 * Copyright (c) 2026 ofek9993
 * SPDX-License-Identifier: Apache-2.0
 *
 * Auto-incrementing write pointer.
 *
 * Exists so the design has real sequential logic of its own alongside the
 * hard macro: the SRAM gets written as a stream (no address needed from the
 * host) while reads are random-access. That also gives the clock tree flops
 * to balance outside the macro.
 */

`default_nettype none

module write_pointer #(
    parameter WIDTH = 10
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             clear,   // synchronous rewind to zero
    input  wire             step,    // advance by one
    output wire [WIDTH-1:0] addr
);

  reg [WIDTH-1:0] ptr;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)     ptr <= {WIDTH{1'b0}};
    else if (clear) ptr <= {WIDTH{1'b0}};
    else if (step)  ptr <= ptr + 1'b1;
  end

  assign addr = ptr;

endmodule
