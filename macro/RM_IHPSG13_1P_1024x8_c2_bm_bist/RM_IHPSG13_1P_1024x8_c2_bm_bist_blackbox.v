/*
 * Black-box declaration of the IHP 1024x8 SRAM for synthesis.
 *
 * The PDK's own model cannot be given to Yosys: it carries a specify block
 * with negative-timing-check syntax ($setuphold with elided arguments,
 * "...,,,A_CLK_DELAY, A_MEN_DELAY") which Yosys cannot parse. It is also the
 * wrong thing to synthesise - the macro is a pre-built block, so synthesis
 * only needs its interface, while its physical shape comes from the LEF and
 * its timing from the Liberty files.
 *
 * The full behavioural model is still used for simulation; test/Makefile
 * includes it separately for both RTL and gate-level runs.
 *
 * Ports copied from RM_IHPSG13_1P_1024x8_c2_bm_bist.v.
 */

(* blackbox *)
module RM_IHPSG13_1P_1024x8_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire [9:0]  A_ADDR,
    input  wire [7:0]  A_DIN,
    input  wire        A_DLY,
    output wire [7:0]  A_DOUT,
    input  wire [7:0]  A_BM,
    input  wire        A_BIST_CLK,
    input  wire        A_BIST_EN,
    input  wire        A_BIST_MEN,
    input  wire        A_BIST_WEN,
    input  wire        A_BIST_REN,
    input  wire [9:0]  A_BIST_ADDR,
    input  wire [7:0]  A_BIST_DIN,
    input  wire [7:0]  A_BIST_BM
);
endmodule
