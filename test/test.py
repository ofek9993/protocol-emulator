# SPDX-FileCopyrightText: © 2026 ofek9993
# SPDX-License-Identifier: Apache-2.0
"""SRAM scratchpad tests.

Interface under test:
    ui_in[7:0]   write data
    uio_in[0]    write enable (stores ui_in, advances the write pointer)
    uio_in[1]    clear write pointer
    uio_in[7:2]  read address (6 bits -> low 64 words)
    uo_out[7:0]  read data

The IHP macro does a synchronous read: address is presented, and on the
clock edge A_DOUT takes the value. So a read costs one clock, same as a
write, and both are sampled mid-cycle on the following falling edge.
"""

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge


def as_int(sig):
    """Read a signal, naming it clearly if it is X/Z rather than a number."""
    v = sig.value
    if not v.is_resolvable:
        raise AssertionError(f"signal is not resolvable (X/Z): {v!s}")
    return int(v)


def ctrl(we=0, clear=0, raddr=0):
    """Pack the uio_in control word."""
    return (we & 1) | ((clear & 1) << 1) | ((raddr & 0x3F) << 2)


async def reset(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def write(dut, data):
    """Stream one byte to the next free address."""
    dut.ui_in.value = data
    dut.uio_in.value = ctrl(we=1)
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    dut.uio_in.value = ctrl(we=0)


async def read(dut, addr):
    """Random-access read; returns the byte at addr."""
    dut.uio_in.value = ctrl(we=0, raddr=addr)
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    return as_int(dut.uo_out)


async def clear_pointer(dut):
    dut.uio_in.value = ctrl(clear=1)
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    dut.uio_in.value = ctrl()


@cocotb.test()
async def test_write_then_read_back(dut):
    """The simplest thing that must work: store bytes, read them back."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    data = [0x00, 0xA5, 0xFF, 0x5A, 0x01]
    for d in data:
        await write(dut, d)

    for addr, expected in enumerate(data):
        got = await read(dut, addr)
        assert got == expected, (
            f"addr {addr}: wrote {expected:#04x}, read back {got:#04x}"
        )

    dut._log.info(f"stored and verified {len(data)} bytes")


@cocotb.test()
async def test_pointer_auto_increments(dut):
    """Each write must land at the next address, not overwrite the last."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    for i in range(8):
        await write(dut, 0x10 + i)

    for i in range(8):
        got = await read(dut, i)
        assert got == 0x10 + i, (
            f"addr {i}: expected {0x10 + i:#04x}, got {got:#04x} "
            "(write pointer did not advance correctly)"
        )


@cocotb.test()
async def test_clear_pointer_rewinds(dut):
    """Clearing the pointer must send the next write back to address 0."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    for d in (0x11, 0x22, 0x33):
        await write(dut, d)

    await clear_pointer(dut)
    await write(dut, 0x99)

    got = await read(dut, 0)
    assert got == 0x99, f"addr 0 should have been overwritten, got {got:#04x}"
    # the byte after it must be untouched
    got = await read(dut, 1)
    assert got == 0x22, f"addr 1 should still hold 0x22, got {got:#04x}"


@cocotb.test()
async def test_random_stream(dut):
    """Random data against a Python reference model of the memory."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    random.seed(7)
    model = [random.randint(0, 255) for _ in range(64)]

    for d in model:
        await write(dut, d)

    for addr in random.sample(range(64), 24):
        got = await read(dut, addr)
        assert got == model[addr], (
            f"addr {addr}: model says {model[addr]:#04x}, SRAM says {got:#04x}"
        )

    dut._log.info("64 bytes written, 24 random reads verified")


@cocotb.test()
async def test_reset_clears_pointer(dut):
    """Reset must rewind the write pointer (memory contents may persist)."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    for d in (0xDE, 0xAD):
        await write(dut, d)

    await reset(dut)
    await write(dut, 0xBE)

    got = await read(dut, 0)
    assert got == 0xBE, (
        f"after reset the next write should land at address 0, got {got:#04x}"
    )
