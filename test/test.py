# SPDX-FileCopyrightText: © 2026 ofek9993
# SPDX-License-Identifier: Apache-2.0

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

MASK16 = 0xFFFF


def as_int(sig):
    """Read a signal, giving a clear message if it is X/Z.

    At gate level an unresolved value shows up as X rather than a wrong
    number, and the default error ("Can't convert LogicArray to int") does
    not say which signal or when.
    """
    v = sig.value
    if not v.is_resolvable:
        raise AssertionError(f"signal is not resolvable (X/Z): {v!s}")
    return int(v)


async def reset(dut):
    """Assert the asynchronous reset and release it away from a clock edge."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 3)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def mac_step(dut, a, b, acc):
    """Drive one operand pair, clock it in, return the expected accumulator.

    Inputs are driven and sampled mid-cycle (on the falling edge after the
    capturing rising edge). That leaves half a clock period for combinational
    logic to settle, so the same testbench works at RTL (zero delay) and at
    gate level (real cell and wire delays).
    """
    dut.ui_in.value = a
    dut.uio_in.value = b
    await ClockCycles(dut.clk, 1)
    await FallingEdge(dut.clk)
    acc = (acc + a * b) & MASK16
    got = as_int(dut.uo_out)
    assert got == (acc >> 8), (
        f"a={a} b={b}: expected acc[15:8]={acc >> 8:#04x} "
        f"(acc={acc:#06x}), got {got:#04x}"
    )
    return acc


@cocotb.test()
async def test_reset_clears_accumulator(dut):
    """After reset the accumulator, and therefore uo_out, must be zero."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)
    assert as_int(dut.uo_out) == 0, "accumulator not cleared by reset"


@cocotb.test()
async def test_known_values(dut):
    """Hand-checked cases, including the widest partial-product path."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    acc = 0
    for a, b in [(0, 0), (1, 1), (16, 16), (200, 200), (255, 255), (0, 0)]:
        acc = await mac_step(dut, a, b, acc)

    dut._log.info(f"accumulator after known values: {acc:#06x}")


@cocotb.test()
async def test_accumulate_wraps(dut):
    """Accumulate past 16 bits to confirm the carry chain wraps cleanly."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    acc = 0
    for _ in range(40):
        acc = await mac_step(dut, 255, 255, acc)

    assert acc != 0, "accumulator should have wrapped by now"
    dut._log.info(f"accumulator after wrapping: {acc:#06x}")


@cocotb.test()
async def test_random_stream(dut):
    """Random operand stream checked against a Python reference model."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    random.seed(1234)
    acc = 0
    for _ in range(100):
        acc = await mac_step(dut, random.randint(0, 255), random.randint(0, 255), acc)

    dut._log.info(f"accumulator after random stream: {acc:#06x}")


@cocotb.test()
async def test_reset_mid_stream(dut):
    """Reset must clear the accumulator even once it holds a value."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    acc = 0
    for _ in range(5):
        acc = await mac_step(dut, 123, 45, acc)
    assert acc != 0

    await reset(dut)
    assert as_int(dut.uo_out) == 0, "reset did not clear the accumulator"

    await mac_step(dut, 10, 10, 0)
