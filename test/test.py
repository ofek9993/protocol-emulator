# SPDX-FileCopyrightText: © 2026 ofek9993
# SPDX-License-Identifier: Apache-2.0

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

MASK16 = 0xFFFF


async def reset(dut):
    """Hold rst_n low long enough to clear the accumulator."""
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 1)
    await Timer(1, unit="ns")


async def mac_step(dut, a, b, acc):
    """Drive one operand pair, step the clock, return the expected accumulator.

    The DUT registers acc <= acc + a*b on the rising edge, so after the edge
    uo_out shows the HIGH byte of the updated accumulator.
    """
    dut.ui_in.value = a
    dut.uio_in.value = b
    await ClockCycles(dut.clk, 1)
    # ClockCycles returns exactly at the rising edge; give the non-blocking
    # assignment a delta to propagate to uo_out before sampling.
    await Timer(1, unit="ns")
    acc = (acc + a * b) & MASK16
    assert dut.uo_out.value == (acc >> 8), (
        f"a={a} b={b}: expected acc[15:8]={acc >> 8:#04x} "
        f"(acc={acc:#06x}), got {int(dut.uo_out.value):#04x}"
    )
    return acc


@cocotb.test()
async def test_reset_clears_accumulator(dut):
    """After reset the accumulator, and therefore uo_out, must be zero."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)
    assert dut.uo_out.value == 0, f"acc not cleared, uo_out={int(dut.uo_out.value)}"


@cocotb.test()
async def test_known_values(dut):
    """A few hand-checked cases, including the widest partial-product path."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    acc = 0
    # 200*200 = 40000 -> exercises nearly the full 16-bit range in one step
    for a, b in [(0, 0), (1, 1), (16, 16), (200, 200), (255, 255), (0, 0)]:
        acc = await mac_step(dut, a, b, acc)

    dut._log.info(f"accumulator after known values: {acc:#06x}")


@cocotb.test()
async def test_accumulate_wraps(dut):
    """Keep accumulating past 16 bits to confirm the carry chain wraps cleanly."""
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
    """Reset must clear the accumulator even after it has been loaded."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    await reset(dut)

    acc = 0
    for _ in range(5):
        acc = await mac_step(dut, 123, 45, acc)
    assert acc != 0

    await reset(dut)
    assert dut.uo_out.value == 0, "reset did not clear the accumulator"

    # and it must keep working afterwards
    acc = 0
    acc = await mac_step(dut, 10, 10, acc)
