<!---
This file is used to generate your project datasheet.
-->

## How it works

A programmable protocol emulator: the chip speaks UART, SPI or I2C (one at a
time), and the protocol is **loaded**, not built in.

Three layers:

- **Controller** - a programmable state machine. The host loads a state table
  (16 rows); each row fires actions on entry (start a frame, set a pin, move
  a byte, load a counter or delay) and watches up to three exits plus a
  timeout. It runs chip-select, START/STOP, ACK/NACK checks, repeated START,
  clock stretching and timeouts by itself.
- **Datapath** - 2 timers + 2 shifters wired by configuration. A timer makes
  the bit clock, a shifter moves bits in or out on it (with start/stop bits,
  MSB/LSB first, open drain or push-pull).
- **Pin layer** - 2-flop synchronisers and a 3-sample glitch filter on every
  protocol input, registered outputs, safe reset levels, and an optional
  output hold delay on one pad (I2C SDA hold time).

The host talks to the chip through a 3-wire serial config port on
`ui_in[2:0]`: 16-bit words `{address, data}`, MSB first, one SCK pulse per
bit, committed when CS rises. Results are read back on `uo_out[0]` (SDO);
`uo_out[1]` (READY) goes high when the controller finishes.

Main registers: `0x00` GCTL (enable, filter off), `0x01` GOUT (what SDO
returns), `0x02` CCTL (run / go / clear / flush), `0x03` push a byte to send,
`0x04` pop a received byte, `0x05` CSEL, `0x06` PHOLD, `0x08-0x0B` controller
pins, `0x10+8n` timer n, `0x30+8n` shifter n, `0x60-0x7F` the controller's
table and settings.

## How to test

With the demo board's RP2040 as the host, bit-bang the config port on
`ui_in[2:0]` and read SDO on `uo_out[0]`.

Simplest check, a UART transmitter at 115200 baud (50 MHz clock): write
`0x10=0x57, 0x11=216, 0x12=19, 0x13=0x00, 0x30=0x83, 0x31=0xBC, 0x00=0x01`,
then write a byte to `0x34`. It comes out on `uo_out[4]`, which is wired to
the RP2040's UART RX.

Pins follow the Tiny Tapeout standard: UART RX `ui_in[3]` / TX `uo_out[4]`;
SPI `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK; I2C `uio[2]` SCL,
`uio[3]` SDA (open drain - add pull-ups).

The cocotb tests in `test/` configure the chip only through its pins: reset
levels, register read-back, UART transmit and receive. Run them with
`cd test && make`.

## External hardware

None for UART and SPI. I2C needs pull-up resistors on SCL and SDA.
