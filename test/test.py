# SPDX-FileCopyrightText: © 2026 ofek9993
# SPDX-License-Identifier: Apache-2.0
"""Smoke tests for the protocol emulator, through its real pins only.

Runs on the RTL (`make`) and, in CI, on the gate-level netlist after
synthesis (`make GATES=yes`), so it never looks inside the chip.

    ui_in[0] CFG_SCK   ui_in[1] CFG_SDI   ui_in[2] CFG_CS (idles high)
    ui_in[3] UART RX   uo_out[0] CFG_SDO  uo_out[4] UART TX

A config write is 16 bits {addr, data}, MSB first, one SCK pulse per bit,
committed when CS rises. Read-back: the byte selected by GOUT (0x01) is
shifted out on SDO, MSB first, during the next word.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

UBIT = 434                      # clocks per UART bit: 115200 baud at 50 MHz

# register addresses
GCTL, GOUT = 0x00, 0x01
T0, T1, S0, S1 = 0x10, 0x18, 0x30, 0x38          # +0 CTL, +1 CMPL, +2 CMPH, +3 PIN, +4 POL / BUF
OUT_RDATA1, OUT_STATUS, OUT_CSTAT = 1, 4, 9


class Chip:
    """The host side: drives the config pins and the UART RX pin."""

    def __init__(self, dut):
        self.dut = dut
        self.cfg = 0b100            # {CS, SDI, SCK}
        self.rx = 1                 # UART RX line, idle high
        self._put()

    def _put(self):
        self.dut.ui_in.value = (self.rx << 3) | self.cfg

    def bit(self, sig, i):
        v = sig.value
        assert v.is_resolvable, f"{sig._name} is X/Z: {v!s}"
        return (int(v) >> i) & 1

    async def clk(self, n):
        await ClockCycles(self.dut.clk, n)
        await FallingEdge(self.dut.clk)          # drive on the falling edge

    async def word(self, addr, data, nbits=16):
        """One config word; returns the byte read back on SDO meanwhile."""
        w, got = (addr << 8) | data, 0
        self.cfg &= ~0b100; self._put(); await self.clk(3)          # CS low
        for k in range(15, 15 - nbits, -1):
            self.cfg = (self.cfg & ~0b010) | (((w >> k) & 1) << 1); self._put(); await self.clk(3)
            if k >= 8:
                got = (got << 1) | self.bit(self.dut.uo_out, 0)     # sample SDO as SCK rises
            self.cfg |= 0b001; self._put(); await self.clk(3)       # SCK rise
            self.cfg &= ~0b001; self._put(); await self.clk(3)
        self.cfg |= 0b100; self._put(); await self.clk(4)           # CS rise: commit
        return got

    async def write(self, addr, data):
        await self.word(addr, data)

    async def read(self, sel):
        await self.word(GOUT, sel)
        return await self.word(0xFF, 0x00, nbits=8)                 # 8 bits: too short to commit

    async def uart_decode(self, timeout=20 * UBIT):
        """Independent UART receiver on uo_out[4]: finds the start bit, samples mid-bit."""
        for _ in range(timeout):
            await RisingEdge(self.dut.clk)
            if self.bit(self.dut.uo_out, 4) == 0:
                break
        else:
            raise AssertionError("no start bit on uo_out[4]")
        await ClockCycles(self.dut.clk, UBIT // 2)
        assert self.bit(self.dut.uo_out, 4) == 0, "start bit not low at mid-bit"
        b = 0
        for k in range(8):
            await ClockCycles(self.dut.clk, UBIT)
            b |= self.bit(self.dut.uo_out, 4) << k
        await ClockCycles(self.dut.clk, UBIT)
        assert self.bit(self.dut.uo_out, 4) == 1, "stop bit not high"
        return b

    async def uart_send(self, b):
        """Independent UART transmitter on ui_in[3]."""
        for level in [0] + [(b >> k) & 1 for k in range(8)] + [1, 1]:
            self.rx = level; self._put()
            await self.clk(UBIT)


async def start(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())       # 50 MHz
    dut.ena.value = 1
    dut.uio_in.value = 0xFF
    chip = Chip(dut)
    dut.rst_n.value = 0
    await chip.clk(10)
    dut.rst_n.value = 1
    await chip.clk(10)
    return chip


@cocotb.test()
async def test_reset_levels(dut):
    """Safe pin levels in and out of reset: outputs high, bidirectional pads released."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.ena.value = 1
    dut.uio_in.value = 0xFF
    chip = Chip(dut)
    dut.rst_n.value = 0
    await chip.clk(5)
    assert (int(dut.uo_out.value) >> 4) == 0xF, "in reset: uo_out[7:4] must be high (UART TX idle)"
    assert int(dut.uio_oe.value) == 0, "in reset: bidirectional pads must be released"
    dut.rst_n.value = 1
    await chip.clk(20)
    assert (int(dut.uo_out.value) >> 4) == 0xF, "after reset: uo_out[7:4] must stay high"
    assert int(dut.uio_oe.value) == 0, "after reset: bidirectional pads must stay released"
    assert int(dut.uo_out.value) & 0b11 == 0, "after reset: SDO and READY low"


@cocotb.test()
async def test_readback(dut):
    """The config port and SDO read-back return known values."""
    chip = await start(dut)
    st = await chip.read(OUT_STATUS)       # both shifters idle: TX buffers empty
    assert st == 0x03, f"status read {st:#04x}, expected 0x03"
    cs = await chip.read(OUT_CSTAT)        # controller idle, RX FIFO empty
    assert cs == 0x04, f"controller status read {cs:#04x}, expected 0x04"


@cocotb.test()
async def test_uart_tx(dut):
    """Configure a UART transmitter (timer 0 + shifter 0) and decode what it sends."""
    chip = await start(dut)
    for a, d in [(T0 + 0, 0x57),          # baud mode, start on trigger, stop on compare, idle high
                 (T0 + 1, 216),           # half bit - 1
                 (T0 + 2, 19),            # 10 bits: start + 8 + stop
                 (T0 + 3, 0x00),          # trigger = shifter 0, no pin
                 (S0 + 0, 0x83),          # transmit, timer 0, LSB first, push-pull
                 (S0 + 1, 0xBC),          # start 0, stop 1, pin 12 = uo_out[4]
                 (GCTL, 0x01)]:
        await chip.write(a, d)
    for sent in (0x55, 0xA3):
        dec = cocotb.start_soon(chip.uart_decode())
        await chip.write(S0 + 4, sent)    # SHBUF: send it
        got = await dec
        assert got == sent, f"UART TX: sent {sent:#04x}, decoded {got:#04x}"
        await chip.clk(2 * UBIT)


@cocotb.test()
async def test_uart_rx(dut):
    """Configure a UART receiver (timer 1 + shifter 1), send it a byte, read it back over SDO."""
    chip = await start(dut)
    for a, d in [(T1 + 0, 0x74),          # baud mode, start on a pin edge, stop on compare
                 (T1 + 1, 216),
                 (T1 + 2, 19),
                 (T1 + 3, 0x20),          # watch pin 8 = ui_in[3]
                 (T1 + 4, 0x01),          # invert: start on the FALLING edge
                 (S1 + 0, 0x50),          # receive, timer 1
                 (S1 + 1, 0xB8),          # start 0, stop 1, pin 8
                 (GCTL, 0x01)]:
        await chip.write(a, d)
    for sent in (0xC4, 0x3B):
        await chip.uart_send(sent)
        got = await chip.read(OUT_RDATA1)
        assert got == sent, f"UART RX: sent {sent:#04x}, read back {got:#04x}"
