<!---
This file is used to generate your project datasheet.
-->

## How it works

An 8x8 unsigned multiply-accumulate unit, used as a workstation stress test for
the Jane Street protocol emulator ASIC competition flow.

On every rising clock edge the design computes `ui_in * uio_in` and adds the
16-bit product into a 16-bit accumulator:

```
acc <= acc + (ui_in * uio_in)
```

`uo_out` exposes the **high** byte of the accumulator (`acc[15:8]`). Reading the
high byte rather than the low byte means every partial product in the multiplier
array affects an observable output, so synthesis cannot optimise any of the
multiplier away.

Driving `rst_n` low clears the accumulator.

This is deliberately deeper logic than a simple adder: the multiplier array gives
several nanoseconds of combinational delay, and the 16-bit accumulator gives the
clock tree real sequential load to balance.

## How to test

Drive operands on `ui_in` (A) and `uio_in` (B), pulse the clock, and read the
accumulator high byte on `uo_out`.

The cocotb testbench in `test/` covers reset behaviour, hand-checked values
including the full-range `255*255` case, accumulator wrapping past 16 bits, a
100-sample random stream checked against a Python reference model, and reset
part-way through a stream.

```
cd test && make -B
```

## External hardware

None.
