<!---
This file is used to generate your project datasheet.
-->

## How it works

An SRAM scratchpad built around the IHP `RM_IHPSG13_1P_1024x8_c2_bm_bist`
hard macro (1024 words x 8 bits), used to bring the hard-macro flow up on
CMOS5L.

Three pieces:

- `sram_wrapper.v` wraps the foundry macro and ties off its BIST interface
- `write_pointer.v` is an auto-incrementing write address counter
- `project.v` maps the pins and muxes the single SRAM port between the
  streaming write address and the random-access read address

Writes are streamed: assert `WE` and the byte on `ui_in` lands at the next
free address, with the pointer advancing automatically. Reads are random
access from the 6-bit address on `uio_in[7:2]`, covering the low 64 words.
`CLR_PTR` rewinds the write pointer to zero. The macro does a synchronous
read, so data appears one clock after the address is presented.

The macro measures 146.88 x 336.46 um, which is taller than any x1 or x2
tile, so the project occupies 3x4 tiles.

## How to test

Drive write data on `ui_in`, pulse `WE` (`uio_in[0]`) to store a byte, then
read it back by placing its address on `uio_in[7:2]` with `WE` low and
reading `uo_out`.

The cocotb suite in `test/` covers write-then-read-back, pointer
auto-increment, pointer clear, reset behaviour, and 64 random bytes checked
against a Python reference model.

```
cd test && make -B
```

## External hardware

None.
