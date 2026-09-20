/*
 * Copyright (c) 2026 ofek9993
 * SPDX-License-Identifier: Apache-2.0
 *
 * Thin wrapper around the IHP 1024x8 single-port SRAM hard macro.
 *
 * The macro is a pre-built block, not synthesisable logic: LibreLane is told
 * about it through the MACROS entry in config.json (LEF for its footprint,
 * GDS for the layout, Liberty for timing) and treats this instance as a
 * black box. Everything this module does is name the pins sensibly and tie
 * off the BIST (built-in self test) interface, which is factory test
 * machinery we do not drive. Leaving those inputs floating would propagate X
 * through the memory.
 */

`default_nettype none

module sram_wrapper (
    input  wire        clk,
    input  wire        en,        // memory enable - must be high to read or write
    input  wire        we,        // write enable
    input  wire [9:0]  addr,      // 1024 words
    input  wire [7:0]  din,
    output wire [7:0]  dout
);

  RM_IHPSG13_1P_1024x8_c2_bm_bist sram (
      .A_CLK  (clk),
      .A_MEN  (en),
      .A_WEN  (we),
      .A_REN  (~we),      // read whenever we are not writing
      .A_ADDR (addr),
      .A_DIN  (din),
      .A_DOUT (dout),
      .A_BM   (8'hFF),    // bit mask: every bit writable
      .A_DLY  (1'b0),     // no extra output delay

      // BIST interface - unused, tied inactive
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR (10'b0),
      .A_BIST_DIN  (8'b0),
      .A_BIST_BM   (8'b0)
  );

endmodule
