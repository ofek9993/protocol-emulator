<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

Entry for the Jane Street protocol emulator ASIC competition, targeting IHP's 130nm CMOS5L
process via Tiny Tapeout.

The goal is a small programmable core with an instruction set built for driving and sampling
pins and counting cycles, so that serial protocols such as UART, SPI and I2C can be emulated in
software rather than hardwired into dedicated logic. The design is inspired by the PIO state
machines on the RP2040 and the PRU cores on TI's Sitara parts.

**Current status: pipeline bring-up.** The design in this repository is still the template
placeholder — an 8-bit adder that drives `uo_out` with `ui_in + uio_in`. It exists to validate
the RTL-to-GDS flow end to end on this process before real design work begins.

## How to test

Run the cocotb testbench in `test/`:

```
make -B
```

For the placeholder design, the test drives `ui_in` and `uio_in` and checks that `uo_out` equals
their sum.

## External hardware

None.
