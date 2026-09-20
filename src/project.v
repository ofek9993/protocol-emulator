/*
 * Copyright (c) 2026 ofek9993
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

// SRAM scratchpad - workstation bring-up for the IHP hard macro.
//
// Purpose is to prove the full flow works with a hard macro in it, not to be
// the final protocol emulator. Three pieces:
//
//   sram_wrapper   the 1024x8 IHP SRAM macro (a pre-built block)
//   write_pointer  auto-incrementing write address (ordinary flops)
//   this file      pin mapping and the read/write mux
//
// Writes are streamed: assert we and the data lands at the next free address,
// pointer auto-increments. Reads are random-access from the address on uio.
// Reads and writes share one SRAM port, so writing wins when both are asked.
//
//   ui_in [7:0]  write data
//   uio_in[0]    write enable        (stores ui_in, advances the pointer)
//   uio_in[1]    clear write pointer (rewind to address 0)
//   uio_in[7:2]  read address, 6 bits -> the low 64 words
//   uo_out[7:0]  data read back
module tt_um_ofek9993_protoemu (
    input  wire [7:0] ui_in,    // Dedicated inputs  - write data
    output wire [7:0] uo_out,   // Dedicated outputs - SRAM read data
    input  wire [7:0] uio_in,   // IOs: Input path   - control + read address
    output wire [7:0] uio_out,  // IOs: Output path  - unused
    output wire [7:0] uio_oe,   // IOs: Enable path  - all inputs
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  wire       we        = uio_in[0];
  wire       clear_ptr = uio_in[1];
  wire [5:0] rd_addr   = uio_in[7:2];

  wire [9:0] wr_addr;

  write_pointer #(.WIDTH(10)) wptr (
      .clk   (clk),
      .rst_n (rst_n),
      .clear (clear_ptr),
      .step  (we),          // advance only on an actual write
      .addr  (wr_addr)
  );

  // One port, so the write address wins while we is asserted.
  wire [9:0] addr = we ? wr_addr : {4'b0, rd_addr};

  sram_wrapper mem (
      .clk  (clk),
      .en   (1'b1),         // memory always enabled
      .we   (we),
      .addr (addr),
      .din  (ui_in),
      .dout (uo_out)
  );

  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;   // uio is input-only here

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, 1'b0};

endmodule
