<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

8x8 multiply-accumulate. On each rising clk, acc <= acc + (ui_in * uio_in).
uo_out exposes acc[15:8]. Asynchronous active-low reset clears the accumulator.

## How to test

Drive operand A on ui_in and operand B on uio_in, clock, read acc[15:8] on uo_out.
See test/test.py: reset, known values, 16-bit wrap, 100 random pairs, mid-stream reset.

## External hardware

List external hardware used in your project (e.g. PMOD, LED display, etc), if any
