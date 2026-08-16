import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

from golden_model import SCHEMES, butterfly, to_mont

# uio_in bit map
SH = 1 << 0
START = 1 << 1
SCHEME_SHIFT = 3
# uio_out bit map
DONE = 1 << 5
BUSY = 1 << 6


def bytes_of(x):
    return [(x >> 16) & 0xFF, (x >> 8) & 0xFF, x & 0xFF]


async def tick(dut, n=1):
    """Advance n clock edges and let the results settle before sampling.

    Reading a signal in the same delta as the rising edge returns its
    pre-edge value, so every edge is followed by a short settle delay. The
    delay is well inside the 35 ns period and also covers the UNIT_DELAY
    annotations used in gate-level simulation.
    """
    await ClockCycles(dut.clk, n)
    await Timer(2, unit="ns")


async def reset(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await tick(dut, 2)


async def run_butterfly(dut, a, b, w, sel):
    """Load a, b, w; run; return (u, v)."""
    _, q, k = SCHEMES[sel]
    ctrl = sel << SCHEME_SHIFT

    # ---- load 9 bytes: in -> wreg -> breg -> areg
    stream = bytes_of(a) + bytes_of(b) + bytes_of(to_mont(w, q, k))
    for byte in stream:
        dut.ui_in.value = byte
        dut.uio_in.value = ctrl | SH
        await tick(dut)
    dut.uio_in.value = ctrl
    dut.ui_in.value = 0

    # ---- launch
    dut.uio_in.value = ctrl | START
    await tick(dut)
    dut.uio_in.value = ctrl

    # ---- wait for done (bounded, so a hang fails instead of spinning)
    for _ in range(64):
        await tick(dut)
        if int(dut.uio_out.value) & DONE:
            break
    else:
        raise AssertionError(f"done never asserted for sel={sel}")

    # ---- unload 6 bytes: u[23:16..0] then v[23:16..0]
    out = []
    for i in range(6):
        out.append(int(dut.uo_out.value))
        if i < 5:
            dut.uio_in.value = ctrl | SH
            await tick(dut)
            dut.uio_in.value = ctrl

    u = (out[0] << 16) | (out[1] << 8) | out[2]
    v = (out[3] << 16) | (out[4] << 8) | out[5]
    return u, v


@cocotb.test()
async def test_reset_and_idle(dut):
    """Reset clears the datapath and the unit reports neither busy nor done."""
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)
    assert int(dut.uo_out.value) == 0
    status = int(dut.uio_out.value)
    assert status & BUSY == 0, "busy asserted after reset"
    assert status & DONE == 0, "done asserted after reset"
    assert int(dut.uio_oe.value) == 0b1110_0000


@cocotb.test()
async def test_known_vectors(dut):
    """Hand-checked corner vectors for each scheme."""
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)

    for sel in (0, 1, 2):
        name, q, _ = SCHEMES[sel]
        vectors = [
            (0, 0, 0),
            (1, 1, 1),
            (0, q - 1, q - 1),
            (q - 1, q - 1, q - 1),
            (q - 1, 1, 1),
            (1, q - 1, 1),
            ((q - 1) // 2, 2, 3),
        ]
        for a, b, w in vectors:
            eu, ev, et = butterfly(a, b, w, sel)
            u, v = await run_butterfly(dut, a, b, w, sel)
            assert (u, v) == (eu, ev), (
                f"{name} a={a} b={b} w={w}: got u={u} v={v}, want u={eu} v={ev} (t={et})"
            )
            await reset(dut)


@cocotb.test()
async def test_randomised_all_schemes(dut):
    """Randomised sweep across all three moduli."""
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)
    rng = random.Random(0xC0FFEE)

    for sel in (0, 1, 2):
        name, q, _ = SCHEMES[sel]
        for _ in range(25):
            a = rng.randrange(q)
            b = rng.randrange(q)
            w = rng.randrange(q)
            eu, ev, _ = butterfly(a, b, w, sel)
            u, v = await run_butterfly(dut, a, b, w, sel)
            assert (u, v) == (eu, ev), (
                f"{name} a={a} b={b} w={w}: got ({u},{v}), want ({eu},{ev})"
            )
            await reset(dut)


@cocotb.test()
async def test_scheme_switch_without_reset(dut):
    """Crypto-agility: switch modulus back to back with no reset in between.

    This is the property the design exists to demonstrate, so it is checked
    without the reset that the other tests use between vectors.
    """
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)
    rng = random.Random(1234)

    for sel in [0, 2, 1, 2, 0, 1, 0, 2]:
        name, q, _ = SCHEMES[sel]
        a, b, w = (rng.randrange(q) for _ in range(3))
        eu, ev, _ = butterfly(a, b, w, sel)
        u, v = await run_butterfly(dut, a, b, w, sel)
        assert (u, v) == (eu, ev), (
            f"{name} after switch: got ({u},{v}), want ({eu},{ev})"
        )


@cocotb.test()
async def test_latency_matches_model(dut):
    """Cycle count from start to done must be k+5 for each scheme."""
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)

    for sel in (0, 1, 2):
        name, q, k = SCHEMES[sel]
        ctrl = sel << SCHEME_SHIFT
        for byte in bytes_of(7) + bytes_of(11) + bytes_of(to_mont(13, q, k)):
            dut.ui_in.value = byte
            dut.uio_in.value = ctrl | SH
            await tick(dut)
        dut.ui_in.value = 0
        dut.uio_in.value = ctrl | START
        await tick(dut)
        dut.uio_in.value = ctrl

        n = 0
        while not (int(dut.uio_out.value) & DONE):
            await tick(dut)
            n += 1
            assert n < 64, f"{name} never finished"
        assert n == k + 5, f"{name}: {n} cycles, expected {k + 5}"
        await reset(dut)


@cocotb.test()
async def test_no_x_on_outputs(dut):
    """No output may be X or Z at any point during a full transaction."""
    cocotb.start_soon(Clock(dut.clk, 35, unit="ns").start())
    await reset(dut)

    async def check():
        for sig in (dut.uo_out, dut.uio_out, dut.uio_oe):
            assert sig.value.is_resolvable, f"{sig._name} is X/Z"

    ctrl = 2 << SCHEME_SHIFT
    q, k = SCHEMES[2][1], SCHEMES[2][2]
    for byte in bytes_of(q - 1) + bytes_of(q - 2) + bytes_of(to_mont(q - 3, q, k)):
        dut.ui_in.value = byte
        dut.uio_in.value = ctrl | SH
        await tick(dut)
        await check()
    dut.uio_in.value = ctrl | START
    await tick(dut)
    dut.uio_in.value = ctrl
    for _ in range(40):
        await tick(dut)
        await check()
