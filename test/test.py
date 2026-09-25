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

The suite, one short test per feature (the long sweeps, 256-value runs,
teeth and mutations live in the local benches - too slow for gate level):
  A  the chip itself    reset levels, register read-back, read-back codes after reset
  B  UART               TX, RX, back to back, 9600 baud, framing error + break
  C  SPI                a flash's read ID, all four modes + LSB first
  D  I2C                write / read, dead bus, no device, clock stretching,
                        bus-free time (B32), bus clear (B35), long filter (B36)
  E  the whole chip     datapath off releases the pins (B34), protocol
                        switching without reset (B33), reset mid-transfer
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

    async def word(self, addr, data, nbits=16, strict=True):
        """One config word; returns the byte read back on SDO meanwhile.
        strict = False: the caller does not use that byte, so an undefined SDO
        bit is fine (e.g. the head of an EMPTY RX FIFO - AUDIT B39)."""
        w, got = (addr << 8) | data, 0
        self.cfg &= ~0b100; self._put(); await self.clk(3)          # CS low
        for k in range(15, 15 - nbits, -1):
            self.cfg = (self.cfg & ~0b010) | (((w >> k) & 1) << 1); self._put(); await self.clk(3)
            if k >= 8:                                              # sample SDO as SCK rises
                v = self.dut.uo_out.value
                got = (got << 1) | (self.bit(self.dut.uo_out, 0) if strict or v.is_resolvable else 0)
            self.cfg |= 0b001; self._put(); await self.clk(3)       # SCK rise
            self.cfg &= ~0b001; self._put(); await self.clk(3)
        self.cfg |= 0b100; self._put(); await self.clk(4)           # CS rise: commit
        return got

    async def write(self, addr, data):
        await self.word(addr, data, strict=False)

    async def read(self, sel):
        await self.word(GOUT, sel, strict=False)
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


# ======================================================================
# The controller, through its real pins: SPI and I2C run as loaded tables
# ======================================================================
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CCTL, HTX, HRXPOP, CSEL, PHOLD = 0x02, 0x03, 0x04, 0x05, 0x06
CPMAP0, CPMAP1 = 0x08, 0x09
OUT_HRX = 8


def program(name):
    """The state table exactly as the chip loads it: one {addr, data} word per line
    (test/programs/, written by programs/build.py - the same files the RTL benches load)."""
    with open(os.path.join(HERE, "programs", "prog_%s.hex" % name)) as f:
        return [(int(l, 16) >> 8, int(l, 16) & 0xFF) for l in f if l.strip()]


class Board:
    """The wires outside the chip. Every clock: each uio wire is the AND of
    what the chip drives (only where uio_oe = 1) and what the device drives,
    with a pull-up where nobody drives - which is exactly how open drain works.
    The device model sees the wires and may change what it drives."""

    def __init__(self, dut, device):
        self.dut, self.device = dut, device
        self.en, self.val = 0, 0xFF          # what the device drives (per bit)
        self.pad = 0xFF
        cocotb.start_soon(self._run())

    def _levels(self):
        oe, out = self.dut.uio_oe.value, self.dut.uio_out.value
        oe = int(oe) if oe.is_resolvable else 0
        out = int(out) if out.is_resolvable else 0xFF
        chip = (out & oe) | (~oe & 0xFF)     # released pads read high
        dev = (self.val & self.en) | (~self.en & 0xFF)
        return chip & dev

    def drive(self, bit, level):             # device side: None = release
        if level is None:
            self.en &= ~(1 << bit)
        else:
            self.en |= 1 << bit
            self.val = (self.val & ~(1 << bit)) | (level << bit)

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.pad = self._levels()
            self.device.step(self, self.pad)
            self.dut.uio_in.value = self._levels()


class SpiFlash:
    """SPI mode 0 on CS uio[0], MOSI uio[1], MISO uio[2], SCK uio[3].
    Answers 0x9F (READ JEDEC ID) with EF 40 18."""

    def __init__(self):
        self.pcs, self.psck, self.act, self.started = 1, 0, False, False
        self.rx, self.sh, self.k, self.j, self.out = [], 0, 0, 0, 0xFF
        self.clocks_without_cs = 0

    def step(self, b, pad):
        cs, mosi, sck = pad & 1, (pad >> 1) & 1, (pad >> 3) & 1
        if not self.started:                                        # the first look is the starting point
            self.pcs, self.psck, self.started = cs, sck, True
            return
        if self.pcs and not cs:
            self.act, self.k, self.j, self.sh, self.out = True, 0, 0, 0, 0xFF
            b.drive(2, self.out >> 7)
        elif not self.pcs and cs:
            self.act = False
            b.drive(2, None)
        if not self.psck and sck:                                   # rise: sample MOSI
            if not self.act:
                self.clocks_without_cs += 1
            else:
                self.sh = ((self.sh << 1) | mosi) & 0xFF
                self.k += 1
                if self.k == 8:
                    self.rx.append(self.sh)
                    n = len(self.rx) - 1
                    self.out = [0xEF, 0x40, 0x18][n] if self.rx[0] == 0x9F and n < 3 else 0xFF
                    self.k, self.sh = 0, 0
        if self.psck and not sck and self.act:                      # fall: next MISO bit
            self.j = (self.j + 1) % 8
            b.drive(2, (self.out >> (7 - self.j)) & 1)
        self.pcs, self.psck = cs, sck


class I2cEeprom:
    """A spec-behaved I2C EEPROM at 0x50 on SCL uio[2], SDA uio[3] (open drain):
    detects START / STOP, ACKs its address, takes a register pointer, stores
    written bytes, returns data, honours the master's ACK / NACK."""

    def __init__(self):
        self.mem = [0xFF] * 256
        self.mode, self.bits, self.sh, self.rw, self.first = "idle", 0, 0, 0, False
        self.ptr, self.cur, self.j, self.mack = 0, 0, 0, 1
        self.ps, self.pd = 1, 1
        self.starts = self.stops = 0
        self.master_acks = []
        self.hold_scl = False           # True: after its next ACK it holds SCL low for ever (a dead bus)
        self.stretch, self.hold, self.stretches = 0, 0, 0   # > 0: hold SCL low that many clocks after each ACK

    def step(self, b, pad):
        scl, sda = (pad >> 2) & 1, (pad >> 3) & 1

        def sda_out(v):                                             # 1 = release, 0 = pull low
            b.drive(3, None if v else 0)

        def stretch():                                              # clock stretching after an ACK
            if self.stretch:
                b.drive(2, 0)
                self.hold = self.stretch
                self.stretches += 1

        if self.hold:
            self.hold -= 1
            if self.hold == 0:
                b.drive(2, None)

        if self.ps and scl:                                         # SDA moving while SCL high
            if self.pd and not sda:
                self.starts += 1
                self.mode, self.bits, self.sh = "addr", 0, 0
                sda_out(1)
            elif not self.pd and sda:
                self.stops += 1
                self.mode = "idle"
                sda_out(1)
        if not self.ps and scl:                                     # SCL rise: sample
            if self.mode in ("addr", "wr"):
                self.sh = ((self.sh << 1) | sda) & 0xFF
                self.bits += 1
            elif self.mode == "rack":
                self.mack = sda
        if self.ps and not scl:                                     # SCL fall: act
            m = self.mode
            if m == "addr" and self.bits == 8:
                if self.sh >> 1 == 0x50:
                    self.rw, self.mode = self.sh & 1, "aack"
                    sda_out(0)
                else:
                    self.mode = "idle"
            elif m == "aack":
                sda_out(1)
                stretch()
                if self.hold_scl:
                    b.drive(2, 0)                                   # a dead bus: SCL held low for ever
                if not self.rw:
                    self.mode, self.bits, self.sh, self.first = "wr", 0, 0, True
                else:
                    self.mode, self.j, self.cur = "rd", 0, self.mem[self.ptr]
                    sda_out(self.cur >> 7)
            elif m == "wr" and self.bits == 8:
                if self.first:
                    self.ptr, self.first = self.sh, False
                else:
                    self.mem[self.ptr] = self.sh
                    self.ptr = (self.ptr + 1) & 0xFF
                sda_out(0)
                self.mode = "wack"
            elif m == "wack":
                sda_out(1)
                stretch()
                self.mode, self.bits, self.sh = "wr", 0, 0
            elif m == "rd":
                self.j += 1
                if self.j < 8:
                    sda_out((self.cur >> (7 - self.j)) & 1)
                else:
                    sda_out(1)
                    self.mode = "rack"
            elif m == "rack":
                self.master_acks.append(self.mack)
                if self.mack == 0:
                    self.ptr = (self.ptr + 1) & 0xFF
                    self.cur = self.mem[self.ptr]
                    self.j, self.mode = 0, "rd"
                    sda_out(self.cur >> 7)
                else:
                    self.mode = "done"
        self.ps, self.pd = scl, sda


async def wait_ready(chip, timeout):
    """The controller's DONE comes out on the READY pin, uo_out[1]."""
    for _ in range(timeout):
        await RisingEdge(chip.dut.clk)
        if chip.bit(chip.dut.uo_out, 1):
            return
    raise AssertionError("READY (uo_out[1]) never went high")


async def pop(chip):
    """Read the oldest received byte over SDO and drop it, in one config word."""
    await chip.word(GOUT, OUT_HRX, strict=False)
    return await chip.word(HRXPOP, 0x00)


@cocotb.test()
async def test_spi_flash_via_controller(dut):
    """The controller runs SPI by itself: CS, 4 bytes out, 4 bytes in, READY."""
    chip = await start(dut)
    flash = SpiFlash()
    Board(dut, flash)
    for a, d in [(T0 + 0, 0x54),          # baud mode, CPOL 0 (SCK idles low)
                 (T0 + 1, 9), (T0 + 2, 15),   # SCK half period 10 clocks, 8 cycles
                 (T0 + 3, 0x0F),          # SCK on uio[3], push-pull
                 (T0 + 4, 0x04),          # the controller starts each frame
                 (S0 + 0, 0x8F),          # TX, MSB first, changes on SCK fall
                 (S0 + 1, 0x01),          # MOSI on uio[1]
                 (S1 + 0, 0x44),          # RX, samples on SCK rise
                 (S1 + 1, 0x02),          # MISO on uio[2]
                 (CSEL, 0x04),            # controller: TX = shifter 0, RX = shifter 1
                 (CPMAP0, 0x30),          # controller pin 0 = CS on uio[0], push-pull
                 (GCTL, 0x01)]:
        await chip.write(a, d)
    for a, d in program("spi"):           # load the table
        await chip.write(a, d)
    for b in (0x9F, 0x00, 0x00, 0x00):
        await chip.write(HTX, b)
    await chip.write(CCTL, 0x03)          # run + GO
    await wait_ready(chip, 20000)
    got = [await pop(chip) for _ in range(4)]
    assert flash.rx == [0x9F, 0x00, 0x00, 0x00], f"flash received {[hex(x) for x in flash.rx]}"
    assert got == [0xFF, 0xEF, 0x40, 0x18], f"host read {[hex(x) for x in got]} over SDO"
    assert flash.clocks_without_cs == 0, "SCK toggled while CS was high"
    assert int(dut.uio_out.value) & 1 == 1, "CS is not back high at the end"


@cocotb.test()
async def test_i2c_eeprom_via_controller(dut):
    """The controller runs I2C by itself: open drain, START / repeated START / STOP,
    ACKs, a write then a read, with the SDA hold delay (PHOLD) switched on."""
    chip = await start(dut)
    ee = I2cEeprom()
    Board(dut, ee)
    for a, d in [(T0 + 0, 0x55),          # baud mode, SCL idles high
                 (T0 + 1, 39), (T0 + 2, 17),  # SCL half period 40 clocks, 9 clocks per byte
                 (T0 + 3, 0x09),          # SCL on uio[2], open drain
                 (T0 + 4, 0x3C),          # controller-started, startlow, park, waitpin
                 (S0 + 0, 0x8D),          # SDA out: TX, MSB first, changes on SCL fall, open drain
                 (S0 + 1, 0x33),          # 8 bits + released ACK slot, on uio[3]
                 (S1 + 0, 0x44),          # SDA in: RX, samples on SCL rise
                 (S1 + 1, 0x23),          # last bit = the ACK, on uio[3]
                 (CSEL, 0x04),
                 (CPMAP0, 0x02),          # controller pin 0 READS SCL
                 (CPMAP1, 0x13),          # controller pin 1 = SDA, open drain (START / STOP)
                 (PHOLD, 0xF3),           # SDA changes reach the pad 15 clocks late (hold time)
                 (GCTL, 0x01)]:
        await chip.write(a, d)
    for a, d in program("i2c"):
        await chip.write(a, d)
    for a, d in [(0x60, 60), (0x61, 0),   # D1: START / STOP hold = 60 clocks
                 (0x66, 0x20), (0x67, 0x4E)]:   # timeout = 20000 clocks
        await chip.write(a, d)

    # write: START, 0xA0 (0x50 + W), pointer 0x10, 0xC3, 0x5A, STOP
    await chip.write(0x6B, 3)             # live counter: 3 bytes after the address
    await chip.write(0x69, 0)             # no read
    for b in (0xA0, 0x10, 0xC3, 0x5A):
        await chip.write(HTX, b)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 50000)
    # READY rises as the controller ENTERS its last row; with PHOLD the STOP's
    # SDA rise reaches the wire 15 clocks later - let the bus settle first
    await chip.clk(100)
    assert ee.mem[0x10:0x12] == [0xC3, 0x5A], f"EEPROM holds {[hex(x) for x in ee.mem[0x10:0x12]]}"
    assert (ee.starts, ee.stops) == (1, 1), f"write: {ee.starts} START(s), {ee.stops} STOP(s)"

    # read: START, 0xA0, pointer 0x10, repeated START, 0xA1 (0x50 + R), 2 bytes, STOP
    await chip.write(0x6B, 1)             # 1 byte after the address (the pointer)
    await chip.write(0x69, 2)             # read 2
    for b in (0xA0, 0x10, 0xA1):
        await chip.write(HTX, b)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 50000)
    await chip.clk(100)
    got = [await pop(chip) for _ in range(2)]
    assert got == [0xC3, 0x5A], f"host read {[hex(x) for x in got]} over SDO"
    assert (ee.starts, ee.stops) == (3, 2), f"total {ee.starts} STARTs, {ee.stops} STOPs (want 3, 2)"
    assert ee.master_acks == [0, 1], f"master ACK/NACK {ee.master_acks}: want ACK then NACK"
    assert (int(dut.uio_oe.value) >> 2) & 0b11 == 0, "SCL / SDA still driven at the end"


# ---------------------------------------------------------------- the B27 / B28 / B29 fixes
# These load the SAME datapath configurations the local Verilog benches run
# (test/datapath_configs/<name>.hex, written by programs/datapath_configs.py),
# after the program - the documented load order.

def config(name):
    """A datapath configuration, one {addr, data} word per line."""
    with open(os.path.join(HERE, "datapath_configs", name + ".hex")) as f:
        return [(int(l, 16) >> 8, int(l, 16) & 0xFF) for l in f if l.strip()]


async def load(chip, prog, cfg):
    for a, d in program(prog) + config(cfg):
        await chip.write(a, d)


async def send_frames(chip, data, bit):
    """Independent UART transmitter on ui_in[3]: 8N1 frames BACK TO BACK -
    the next start bit follows the stop bit with no idle time at all."""
    for b in data:
        for level in [0] + [(b >> k) & 1 for k in range(8)] + [1]:
            chip.rx = level; chip._put()
            await chip.clk(bit)


async def decode_frame(chip, bit, timeout):
    """Independent UART receiver on uo_out[4] at `bit` clocks per bit; also
    measures how long the start bit really is - as the time the line stays
    low, so the byte MUST have bit 0 = 1 (else start + d0 look like one)."""
    for _ in range(timeout):
        await RisingEdge(chip.dut.clk)
        if chip.bit(chip.dut.uo_out, 4) == 0:
            break
    else:
        raise AssertionError("no start bit on uo_out[4]")
    width = 0
    while chip.bit(chip.dut.uo_out, 4) == 0 and width < 4 * bit:
        await RisingEdge(chip.dut.clk)
        width += 1
    await ClockCycles(chip.dut.clk, bit // 2)                   # at the end of the start bit: half a bit on = the middle of d0
    b = 0
    for k in range(8):
        b |= chip.bit(chip.dut.uo_out, 4) << k
        await ClockCycles(chip.dut.clk, bit)
    assert chip.bit(chip.dut.uo_out, 4) == 1, "stop bit not high"
    return b, width


@cocotb.test()
async def test_uart_back_to_back_via_controller(dut):
    """B29: at 1 Mbaud a peer sends 4 frames with NO gap between them (how real
    UARTs send). The receiver must be free again by the middle of each stop
    bit to catch the next start bit - all 4 must arrive, in order."""
    chip = await start(dut)
    await load(chip, "uart", "uart_1m_8n1")
    await chip.write(CCTL, 0x01)              # RUN: the table serves both directions
    await chip.clk(200)
    sent = [0xA0, 0xD9, 0x7E, 0x4D]
    await send_frames(chip, sent, 50)         # 1 Mbaud = 50 clocks a bit
    chip.rx = 1; chip._put()
    await chip.clk(200)
    got = [await pop(chip) for _ in range(4)]
    assert got == sent, f"sent {[hex(x) for x in sent]} back to back, host read {[hex(x) for x in got]}"
    assert await chip.read(10) & 0b10 == 0, "RX overflow flagged"


@cocotb.test()
async def test_uart_9600_via_controller(dut):
    """B28: 9600 baud through the timers' prescaler (TIMPRE). The chip's byte is
    decoded at the REAL 9600 baud (5208 clocks a bit) and its start bit
    measured; a byte sent at the real rate arrives."""
    chip = await start(dut)
    await load(chip, "uart", "uart_9600_8n1")
    await chip.write(CCTL, 0x01)
    await chip.clk(200)
    dec = cocotb.start_soon(decode_frame(chip, 5208, 30000))
    await chip.write(HTX, 0xA5)               # bit 0 = 1: the start bit can be measured
    b, width = await dec
    assert b == 0xA5, f"decoded 0x{b:02x} at 9600 baud"
    assert abs(width - 5208) <= 30, f"start bit {width} clocks, 9600 baud is 5208"
    await send_frames(chip, [0xA3], 5208)
    chip.rx = 1; chip._put()
    await chip.clk(2000)
    got = await pop(chip)
    assert got == 0xA3, f"host read 0x{got:02x}, the peer sent 0xa3 at 9600 baud"


@cocotb.test()
async def test_i2c_dead_bus_via_controller(dut):
    """B27: the EEPROM holds SCL low for ever. The controller must time out AND
    let go of the bus - neither SCL nor SDA driven by us - and nothing may
    happen when the slave lets go. After clear + flush (no reset, no reload)
    a normal write and read-back work."""
    chip = await start(dut)
    ee = I2cEeprom()
    board = Board(dut, ee)
    await load(chip, "i2c", "i2c_test_fast")
    ee.hold_scl = True
    await chip.write(0x6B, 2)                 # 2 bytes after the address
    await chip.write(0x69, 0)
    for b in (0xA0, 0x10, 0xC3):
        await chip.write(HTX, b)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 20000)
    st = await chip.read(OUT_CSTAT)
    assert st & 0b10, f"status 0x{st:02x}: no timeout reported"
    oe = int(dut.uio_oe.value)
    assert (oe >> 2) & 0b11 == 0, f"after the timeout our chip still drives SCL / SDA (uio_oe = 0x{oe:02x})"
    ee.hold_scl = False
    board.drive(2, None)                      # the slave lets go
    ee.mode = "idle"
    await chip.clk(500)
    oe = int(dut.uio_oe.value)
    assert (oe >> 2) & 1 == 0, "SCL driven again after the slave let go (a parked frame)"
    await chip.write(CCTL, 0x0D)              # clear flags + flush: the recovery a NACK needs too
    await chip.write(0x6B, 3)                 # write C3 5A at 0x10: 3 bytes after the address
    await chip.write(0x69, 0)
    for b in (0xA0, 0x10, 0xC3, 0x5A):
        await chip.write(HTX, b)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 20000)
    await chip.clk(100)
    assert ee.mem[0x10:0x12] == [0xC3, 0x5A], f"after recovery the EEPROM holds {[hex(x) for x in ee.mem[0x10:0x12]]}"


# ======================================================================
# The wider suite: every block once, every protocol, every error path and
# every fix (B31-B36), each short - it also runs on the gate-level netlist,
# where synthesis mistakes and flops without a reset show up.
# Pass / fail is decided from the pins only, like everything above.
# ======================================================================
from cocotb.triggers import Timer

OUT_OVERRUN, OUT_FRAMEERR, OUT_TIMACTIVE, OUT_CERR, OUT_CCNT = 5, 6, 7, 10, 11


class SpiEcho:
    """An SPI slave in any mode, either bit order, on CS uio[0], MOSI uio[1],
    MISO uio[2], SCK uio[3]. Byte k of a transaction is answered with the
    inverse of byte k-1 it received; byte 0 with 0x5A - so one transfer checks
    MOSI (what it received) and MISO (what the host read back)."""

    def __init__(self, mode=0, msb=True):
        self.mode, self.msb = mode, msb
        self.started, self.act = False, False
        self.rx, self.sh, self.bits, self.ptr, self.base = [], 0, 0, 0, 0
        self.pcs = self.psck = 1

    def step(self, b, pad):
        cs, mosi, sck = pad & 1, (pad >> 1) & 1, (pad >> 3) & 1
        if not self.started:
            self.pcs, self.psck, self.started = cs, sck, True
            return
        rise, fall = (not self.psck and sck), (self.psck and not sck)
        samp = rise if self.mode in (0, 3) else fall
        chg = fall if self.mode in (0, 3) else rise
        if self.pcs and not cs:                                     # selected
            self.act, self.bits, self.base = True, 0, len(self.rx)
            self.ptr = -1 if self.mode in (1, 3) else 0             # CPHA 0: bit 7 out before the first edge
        elif not self.pcs and cs:
            self.act = False
        if self.act and not cs:
            if samp:
                self.sh = ((self.sh << 1) | mosi) & 0xFF if self.msb else (mosi << 7) | (self.sh >> 1)
                self.bits += 1
                if self.bits == 8:
                    self.rx.append(self.sh)
                    self.bits = 0
            if chg:
                self.ptr += 1
        if self.act and self.ptr >= 0:
            k = self.ptr // 8
            rb = 0x5A if k == 0 else (~self.rx[self.base + k - 1]) & 0xFF
            b.drive(2, (rb >> (7 - self.ptr % 8)) & 1 if self.msb else (rb >> (self.ptr % 8)) & 1)
        elif self.act:
            b.drive(2, 1)
        else:
            b.drive(2, None)                                        # deselected: MISO released
        self.pcs, self.psck = cs, sck


class Devices:
    """Several devices on one board; only the active one takes part."""

    def __init__(self, **devs):
        self.devs, self.active = devs, None

    def use(self, board, name):
        board.en = 0                                                # the old device lets go of every pin
        self.active = name

    def step(self, b, pad):
        if self.active:
            self.devs[self.active].step(b, pad)


def expected_echo(sent):
    return [0x5A] + [(~x) & 0xFF for x in sent[:-1]]


async def switch_to(chip, prog, cfg):
    """What a host does to talk to another device: controller stopped,
    datapath off, the program, the configuration, run + clear + flush."""
    await chip.write(CCTL, 0x00)
    await chip.write(GCTL, 0x00)
    await load(chip, prog, cfg)
    await chip.write(CCTL, 0x0D)


async def spi_txn(chip, data):
    """One chip-select window of up to 4 bytes; returns what came back."""
    await chip.write(0x68, len(data))                               # cntA: bytes in this transaction
    for x in data:
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 20000)
    return [await pop(chip) for _ in data]


async def i2c_write_read(chip, ptr, data):
    """Write data at ptr, then read it back (pointer, repeated START, read)."""
    await chip.write(0x6B, len(data) + 1)
    await chip.write(0x69, 0)
    for x in [0xA0, ptr] + data:
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 60000)
    await chip.clk(100)
    await chip.write(0x6B, 1)
    await chip.write(0x69, len(data))
    for x in (0xA0, ptr, 0xA1):
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 60000)
    await chip.clk(100)
    return [await pop(chip) for _ in data]


# ---------------------------------------------------------------- A · the chip itself
@cocotb.test()
async def test_readback_codes_after_reset(dut):
    """Every read-back code gives its reset value (a flop without a reset would
    read as X at gate level), and the live counter reads back what was written."""
    chip = await start(dut)
    want = {0: 0x00, 1: 0x00, OUT_STATUS: 0x03, OUT_OVERRUN: 0x00, OUT_FRAMEERR: 0x00,
            OUT_TIMACTIVE: 0x00, OUT_CSTAT: 0x04, OUT_CERR: 0x00, OUT_CCNT: 0x00}
    for code, v in want.items():
        got = await chip.read(code)
        assert got == v, f"read-back code {code}: {got:#04x} after reset, expected {v:#04x}"
    for v in (0xA5, 0x3C):
        await chip.write(0x6B, v)
        got = await chip.read(OUT_CCNT)
        assert got == v, f"live counter: wrote {v:#04x}, read {got:#04x}"


# ---------------------------------------------------------------- B · UART errors
@cocotb.test()
async def test_uart_framing_error_and_break(dut):
    """A bad stop bit is flagged and its data still delivered; CCTL[2] clears the
    flag WHILE our TX is sending, and the TX byte is unharmed (B31); a break
    (the line low for 25 bits) gives exactly one 0x00, flagged."""
    chip = await start(dut)
    await load(chip, "uart", "uart_1m_8n1")
    await chip.write(CCTL, 0x01)
    await chip.clk(200)
    for level in [0] + [(0x5C >> k) & 1 for k in range(8)] + [0]:   # stop bit 0 = a framing error
        chip.rx = level; chip._put(); await chip.clk(50)
    chip.rx = 1; chip._put(); await chip.clk(300)
    assert await chip.read(OUT_FRAMEERR) & 0b10, "a bad stop bit was not flagged"
    assert await pop(chip) == 0x5C, "the data bits of the bad frame did not arrive"
    dec = cocotb.start_soon(decode_frame(chip, 50, 5000))
    await chip.write(HTX, 0xA5)                                     # our TX is busy ...
    await chip.write(CCTL, 0x05)                                    # ... while the flag is cleared
    b, _ = await dec
    assert b == 0xA5, f"TX byte during the clear decoded as {b:#04x}"
    assert await chip.read(OUT_FRAMEERR) & 0b10 == 0, "CCTL[2] did not clear the framing error"
    chip.rx = 0; chip._put(); await chip.clk(25 * 50)               # a BREAK
    chip.rx = 1; chip._put(); await chip.clk(300)
    assert await chip.read(OUT_FRAMEERR) & 0b10, "a break was not flagged"
    assert await pop(chip) == 0x00, "a break must deliver 0x00"
    assert await chip.read(OUT_CSTAT) & 0b100, "a break delivered more than one byte"


# ---------------------------------------------------------------- C · SPI, every mode
@cocotb.test()
async def test_spi_all_modes(dut):
    """The SPI program loaded once; modes 0-3 and LSB first switched in as
    configurations without a reset; 3 bytes each way per configuration."""
    chip = await start(dut)
    board = Board(dut, Devices())
    for a, d in program("spi"):
        await chip.write(a, d)
    for name, mode, msb in [("spi_mode0_2m5", 0, True), ("spi_mode1_2m5", 1, True), ("spi_mode2_2m5", 2, True),
                            ("spi_mode3_2m5", 3, True), ("spi_mode0_2m5_lsb", 0, False)]:
        slave = SpiEcho(mode, msb)
        board.en = 0
        board.device = slave
        await chip.write(GCTL, 0x00)
        for a, d in config(name):
            await chip.write(a, d)
        await chip.clk(50)
        sent = [0x9F, 0x3C, 0xC5]
        got = await spi_txn(chip, sent)
        assert slave.rx == sent, f"{name}: the slave received {[hex(x) for x in slave.rx]}"
        assert got == expected_echo(sent), f"{name}: the host read {[hex(x) for x in got]}"
        assert int(dut.uio_out.value) & 1 and int(dut.uio_oe.value) & 1, f"{name}: CS not back high"


# ---------------------------------------------------------------- D · I2C error paths and fixes
@cocotb.test()
async def test_i2c_no_device(dut):
    """An address nobody answers: START, NACK, STOP - done, no timeout, nothing stored."""
    chip = await start(dut)
    ee = I2cEeprom()
    Board(dut, ee)
    await load(chip, "i2c", "i2c_test_fast")
    await chip.write(0x6B, 2)
    await chip.write(0x69, 0)
    for x in (0xA2, 0x10, 0x77):                                    # 0x51 + W: not the EEPROM
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    await wait_ready(chip, 20000)
    await chip.clk(100)
    st = await chip.read(OUT_CSTAT)
    assert st & 0b10 == 0, "a NACK must not look like a timeout"
    assert (ee.starts, ee.stops) == (1, 1), f"{ee.starts} START(s), {ee.stops} STOP(s): want one of each"
    assert ee.mem[0x10] == 0xFF, "something was stored"
    await chip.write(CCTL, 0x0D)                                    # the host throws the rest away


@cocotb.test()
async def test_i2c_clock_stretching(dut):
    """The EEPROM holds SCL low after every ACK; the master must wait, not run on."""
    chip = await start(dut)
    ee = I2cEeprom()
    ee.stretch = 37
    Board(dut, ee)
    await load(chip, "i2c", "i2c_test_fast")
    got = await i2c_write_read(chip, 0x20, [0x61, 0x9E])
    assert ee.mem[0x20:0x22] == [0x61, 0x9E], f"stored {[hex(x) for x in ee.mem[0x20:0x22]]}"
    assert got == [0x61, 0x9E], f"read back {[hex(x) for x in got]}"
    assert ee.stretches > 4, f"only {ee.stretches} stretches happened"


@cocotb.test()
async def test_i2c_bus_free_time(dut):
    """B32: the host queues the next transaction while the first still runs;
    between the STOP and the next START the bus must stay free for Standard
    mode's tBUF, 4.7 us = 235 clocks - kept by the chip, not by the host."""
    chip = await start(dut)
    ee = I2cEeprom()
    ee.mem[0x20], ee.mem[0x30] = 0x11, 0x22
    board = Board(dut, ee)
    await load(chip, "i2c", "i2c_standard_100k")
    gaps, stop_at = [], [None]

    async def watch():                                              # STOP / START on the wires
        ps, pd, n = 1, 1, 0
        while True:
            await RisingEdge(dut.clk)
            n += 1
            scl, sda = (board.pad >> 2) & 1, (board.pad >> 3) & 1
            if ps and scl and not pd and sda:
                stop_at[0] = n
            if ps and scl and pd and not sda and stop_at[0] is not None:
                gaps.append(n - stop_at[0])
                stop_at[0] = None
            ps, pd = scl, sda
    cocotb.start_soon(watch())
    q = [0xA0, 0x20, 0xA1, 0xA0, 0x30, 0xA1]
    await chip.write(0x6B, 1)
    await chip.write(0x69, 1)
    for x in q[:3]:
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    # a quick host: the status stays selected, so each poll is one short word
    await chip.word(GOUT, OUT_CSTAT, strict=False)
    sent, queued = 3, False
    while not queued:
        st = await chip.word(0xFF, 0x00, nbits=8)
        if not st & 0b100:                                          # the first byte is in: queue the next
            await chip.write(0x6B, 1)                               # transaction AT ONCE - GO while the
            await chip.write(CCTL, 0x03)                            # chip is still making its STOP
            queued = True
        elif sent < 6 and not st & 0b1000:
            await chip.write(HTX, q[sent])
            sent += 1
    while sent < 6:
        await chip.write(HTX, q[sent])
        sent += 1
    got = []
    for _ in range(400):                                            # both bytes, as they arrive
        if len(got) == 2:
            break
        if not await chip.read(OUT_CSTAT) & 0b100:
            got.append(await pop(chip))
    await chip.clk(500)
    assert got == [0x11, 0x22], f"read {[hex(x) for x in got]}"
    assert gaps and min(gaps) >= 235, f"bus free time {gaps} clocks, Standard mode needs >= 235"


@cocotb.test()
async def test_i2c_bus_clear(dut):
    """B35: a slave holds SDA low in the middle of a read byte (as after our
    reset). The host's bus clear - the _busclear configuration, 9 SCL clocks
    with SDA released - frees it, and the next transaction is exact."""
    chip = await start(dut)
    ee = I2cEeprom()
    board = Board(dut, ee)
    await load(chip, "i2c", "i2c_test_fast")
    ee.mode, ee.cur, ee.j, ee.ps, ee.pd = "rd", 0x00, 2, 1, 0       # stuck inside a read byte
    board.drive(3, 0)
    await chip.clk(20)
    assert (board.pad >> 3) & 1 == 0, "SDA is not held low"
    await chip.write(GCTL, 0x00)
    for a, d in config("i2c_test_fast_busclear"):
        await chip.write(a, d)
    await chip.write(S0 + 4, 0xFF)                                  # SHBUF: fire the 9 clocks
    await chip.clk(400)
    assert (board.pad >> 3) & 1 == 1, "SDA still low after the bus clear"
    await chip.write(GCTL, 0x00)
    for a, d in config("i2c_test_fast"):
        await chip.write(a, d)
    await chip.write(CCTL, 0x0D)
    got = await i2c_write_read(chip, 0x40, [0xB4, 0x4B])
    assert got == [0xB4, 0x4B], f"after the bus clear: read {[hex(x) for x in got]}"


@cocotb.test()
async def test_long_input_filter(dut):
    """B36: with GCTL[2] a level needs 4 equal samples; a 3-clock pulse (what a
    40-59 ns spike looks like) must not reach the chip. Seen on UART RX: the
    long filter keeps the receiver's timer asleep; the normal filter (the
    control) lets the same pulse start it."""
    chip = await start(dut)
    await load(chip, "uart", "uart_1m_8n1")
    await chip.write(CCTL, 0x01)
    for gctl, starts in ((0x05, False), (0x01, True)):
        await chip.write(GCTL, gctl)
        await chip.clk(100)
        chip.rx = 0; chip._put(); await chip.clk(3)                 # a 3-sample pulse
        chip.rx = 1; chip._put()
        active = bool(await chip.read(OUT_TIMACTIVE) & 0b10)
        assert active == starts, (f"GCTL {gctl:#04x}: a 3-clock pulse "
                                  f"{'did not start' if starts else 'started'} the receiver")
        await chip.clk(700)
    assert await chip.read(OUT_CSTAT) & 0b100, "a pulse delivered a byte"


# ---------------------------------------------------------------- E · the chip as a whole
@cocotb.test()
async def test_datapath_off_releases_pins(dut):
    """B34: with the datapath off, a shifter set to drive pin 5 and a timer set
    to drive pin 6 drive nothing; switched on, they do (the control); the
    controller's own pin line (pin 7) keeps its pin either way."""
    chip = await start(dut)
    for a, d in [(S0 + 0, 0x83), (S0 + 1, 0x05), (T1 + 3, 0x1B), (0x0A, 0x37)]:
        await chip.write(a, d)
    oe = int(dut.uio_oe.value)
    assert (oe >> 5) & 0b11 == 0, f"datapath off, but pins 5/6 are driven (uio_oe {oe:#04x})"
    assert (oe >> 7) & 1, "the controller's pin line lost its pin"
    await chip.write(GCTL, 0x01)
    oe = int(dut.uio_oe.value)
    assert (oe >> 5) & 0b11 == 0b11, f"datapath on, but pins 5/6 are not driven (uio_oe {oe:#04x})"
    await chip.write(GCTL, 0x00)
    assert (int(dut.uio_oe.value) >> 5) & 0b11 == 0, "datapath off again, pins 5/6 still driven"


@cocotb.test()
async def test_protocol_switch_without_reset(dut):
    """B33: one chip, no reset: UART -> I2C -> SPI -> UART, each with its own
    program and configuration. Nothing of the previous protocol may stay: after
    I2C its SDA hold delay would slow SPI's SCK, after SPI its CS would stay
    driven during UART."""
    chip = await start(dut)
    spi, ee = SpiEcho(0), I2cEeprom()
    devs = Devices(spi=spi, i2c=ee)
    board = Board(dut, devs)

    async def uart_leg(tag):
        devs.use(board, None)
        await switch_to(chip, "uart", "uart_1m_8n1")
        await chip.clk(200)
        dec = cocotb.start_soon(decode_frame(chip, 50, 5000))
        await chip.write(HTX, 0xC3)
        b, _ = await dec
        assert b == 0xC3, f"{tag}: UART TX decoded {b:#04x}"
        await send_frames(chip, [0x3D], 50)
        chip.rx = 1; chip._put(); await chip.clk(200)
        assert await pop(chip) == 0x3D, f"{tag}: UART RX byte wrong"
        assert int(dut.uio_oe.value) == 0, f"{tag}: a uio pin is driven during UART"

    await uart_leg("UART first")
    devs.use(board, "i2c")
    await switch_to(chip, "i2c", "i2c_standard_100k")               # its SDA hold delay is on pin 3 ...
    got = await i2c_write_read(chip, 0x50, [0x5E, 0xA1])
    assert got == [0x5E, 0xA1], f"I2C after UART: read {[hex(x) for x in got]}"
    devs.use(board, "spi")
    await switch_to(chip, "spi", "spi_mode0_2m5")                   # ... which is SPI's SCK
    sent = [0x12, 0xEE, 0x81]
    got = await spi_txn(chip, sent)
    assert spi.rx == sent and got == expected_echo(sent), f"SPI after I2C: slave {spi.rx}, host {got}"
    await uart_leg("UART after SPI")                                # SPI's CS line must not stay driven


@cocotb.test()
async def test_reset_mid_transfer(dut):
    """rst_n falls in the middle of an SPI transaction, between clock edges:
    1 ns later every pin is safe (pads released, outputs high, SDO / READY
    low). After program + configuration again, the next transaction is exact."""
    chip = await start(dut)
    slave = SpiEcho(0)
    Board(dut, slave)
    await load(chip, "spi", "spi_mode0_2m5")
    await chip.write(0x68, 4)
    for x in (0x01, 0x02, 0x03, 0x04):
        await chip.write(HTX, x)
    await chip.write(CCTL, 0x03)
    await chip.clk(300)                                             # in the middle of byte 2
    await RisingEdge(dut.clk)
    await Timer(3, unit="ns")
    dut.rst_n.value = 0
    await Timer(1, unit="ns")
    oe, uo = int(dut.uio_oe.value), int(dut.uo_out.value)
    assert oe == 0, f"1 ns after reset: bidirectional pads still driven (uio_oe {oe:#04x})"
    assert uo >> 4 == 0xF and uo & 0b11 == 0, f"1 ns after reset: uo_out {uo:#04x}"
    await chip.clk(5)
    dut.rst_n.value = 1
    await chip.clk(10)
    await load(chip, "spi", "spi_mode0_2m5")
    sent = [0x6B, 0x94]
    got = await spi_txn(chip, sent)
    assert got == expected_echo(sent), f"after the reset the host read {[hex(x) for x in got]}"
