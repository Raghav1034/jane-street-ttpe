"""
cocotb testbench for ttpe_core: Execution Unit + synchroniser + host FIFOs + memory subsystem.

The RTL is run in lockstep with the golden model sw/ttpe_iss.py (see ttpe_lockstep.py for the
exact list of per-cycle comparisons). All stimulus (programs, external devices, host behaviour)
comes from ttpe_scenarios.py, which has directed tests, the four example programs (uart_tx,
uart_rx, spi_master, i2c_master) and seeded random programs.

Cycle convention: cycle k starts at rising clock edge k; inputs applied just after edge k are
sampled at edge k+1; flop outputs are read just after edge k, combinational outputs at the
falling edge in the middle of cycle k. 50 MHz clock (20 ns). All checks are integer cycle
comparisons.

Run:   make TARGET=core                       (all tests, PDK SRAM models)
       make TARGET=core USE_PDK_SRAM=0        (stand-in SRAM model with extra protocol checks)
Subset: cocotb 2.x  COCOTB_TEST_FILTER=fuzz   |  cocotb 1.x  TESTCASE=test_fuzz_3
"""

import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sw"))

import ttpe_scenarios as scn                 # noqa: E402
import ttpe_world as W                       # noqa: E402
from ttpe_lockstep import (DefaultPolicy, Lockstep, LockstepError)  # noqa: E402

CLK_NS = 20

INPUTS = dict(
    rst_n=1, uio_in=0xFF, ui_in=0xFF,
    host_req=0, host_we=0, host_addr=0, host_wdata=0, host_wstrb=0xF,
    host_resume=0, host_pc_load=0, host_pc_val=0, host_halt=0, host_core_reset=0,
    tx_push=0, tx_wdata=0, tx_fifo_flush=0, rx_pop=0, rx_fifo_flush=0,
)


# ------------------------------------------------------------------ helpers
def val(sig):
    """Integer value of a signal; raises on X/Z so an unknown never passes silently."""
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError("%s is X/Z (%s)" % (getattr(sig, "_name", "signal"), sig.value))


async def reset(dut):
    for k, v in INPUTS.items():
        getattr(dut, k).value = v
    dut.rst_n.value = 0
    await Timer(1, "ns")
    cocotb.start_soon(Clock(dut.clk, CLK_NS, "ns").start())
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


async def idle_cycles(dut, n, **ov):
    """n cycles with default inputs (plus overrides); returns nothing."""
    for _ in range(n):
        await RisingEdge(dut.clk)
        for k, v in {**INPUTS, **ov}.items():
            getattr(dut, k).value = v
        await FallingEdge(dut.clk)


async def host_write_image(dut, words):
    """Write every word through the host port (one per cycle) and check the handshake."""
    for a, w in enumerate(words):
        await RisingEdge(dut.clk)
        for k, v in {**INPUTS, "host_req": 1, "host_we": 1, "host_addr": a,
                     "host_wdata": w, "host_wstrb": 0xF}.items():
            getattr(dut, k).value = v
        await FallingEdge(dut.clk)
        assert val(dut.host_gnt) == 1, "host write %d not granted" % a
    await idle_cycles(dut, 4)


class CocoDut:
    """Adapter between ttpe_lockstep.Lockstep and the cocotb handles of ttpe_core."""

    STATE = (("oe", "uio_oe"), ("out", "uio_out"), ("uo", "uo_out"), ("sr", "dbg_sr"),
             ("c0", "dbg_c0"), ("c1", "dbg_c1"), ("halted", "halted"), ("illegal", "illegal"),
             ("tx_count", "tx_count"), ("rx_count", "rx_count"), ("tx_full", "tx_full"),
             ("tx_empty", "tx_empty"), ("rx_full", "rx_full"), ("rx_empty", "rx_empty"),
             ("rx_overrun", "rx_overrun_cnt"), ("rx_rdata", "rx_rdata"), ("mem_free", "mem_free"))
    COMB = (("accept", "dbg_accept"), ("commit", "dbg_commit"), ("pc", "dbg_pc"),
            ("instr", "dbg_instr"), ("pc_load", "dbg_pc_load"), ("host_gnt", "host_gnt"),
            ("host_wdone", "host_wdone"), ("host_rvalid", "host_rvalid"),
            ("host_rdata", "host_rdata"))

    def __init__(self, dut):
        self.dut = dut
        self._state = [(k, getattr(dut, n)) for k, n in self.STATE]
        self._comb = [(k, getattr(dut, n)) for k, n in self.COMB]
        self._in = {k: getattr(dut, k) for k in INPUTS if k != "rst_n"}

    async def next_cycle(self):
        await RisingEdge(self.dut.clk)
        await Timer(1, "ns")                       # flop outputs of the new cycle have settled

    def state(self):
        return {k: val(h) for k, h in self._state}

    def drive(self, d):
        for k, v in d.items():
            self._in[k].value = v

    async def mid_cycle(self):
        await FallingEdge(self.dut.clk)

    def comb(self):
        return {k: val(h) for k, h in self._comb}


async def run_scenario(dut, sc):
    await reset(dut)
    await host_write_image(dut, sc.words)
    ls = Lockstep(CocoDut(dut), sc.words, sc.world, sc.policy, sc.name)
    await ls.run(sc.cycles)
    ls.finish()
    if sc.check:
        sc.check(ls)
    dut._log.info("%s: %d cycles, %d accepts, %d commits, %d halts, %d host RX bytes",
                  sc.name, sc.cycles, len(ls.rtl_accepts), len(ls.rtl_commits), len(ls.halts),
                  len(ls.rx_got))
    return ls


# ------------------------------------------------------------------ basic tests
@cocotb.test()
async def test_reset_state(dut):
    """Reset state: halted, memory free, FIFOs empty, pads idle, nothing runs by itself."""
    await reset(dut)
    for _ in range(30):
        await RisingEdge(dut.clk)
        await Timer(1, "ns")
        assert val(dut.halted) == 1 and val(dut.mem_free) == 1
        assert val(dut.illegal) == 0
        assert val(dut.tx_empty) == 1 and val(dut.rx_empty) == 1
        assert val(dut.tx_full) == 0 and val(dut.rx_full) == 0
        assert val(dut.tx_count) == 0 and val(dut.rx_count) == 0 and val(dut.rx_overrun_cnt) == 0
        assert val(dut.uio_oe) == 0 and val(dut.uio_out) == 0 and val(dut.uo_out) == 0
        assert val(dut.dbg_sr) == 0 and val(dut.dbg_c0) == 0 and val(dut.dbg_c1) == 0
        assert val(dut.dbg_accept) == 0 and val(dut.dbg_commit) == 0
    # the SRAM macro tie-offs required by the PDK model
    assert val(dut.u_mem.u_ctrl.A_DLY) == 1
    for n in ("A_BIST_CLK", "A_BIST_EN", "A_BIST_MEN", "A_BIST_WEN", "A_BIST_REN",
              "A_BIST_ADDR", "A_BIST_DIN", "A_BIST_BM"):
        assert val(getattr(dut.u_mem.u_ctrl, n)) == 0, n


@cocotb.test()
async def test_load_and_readback_random_image(dut):
    """Random image through the host port, read back (pipelined), also with byte strobes."""
    await reset(dut)
    rng = random.Random(99)
    image = [rng.randrange(1 << 32) for _ in range(512)]
    await host_write_image(dut, image)
    # byte-strobe merges on 16 words
    for a in range(16):
        st = rng.randrange(16)
        nw = rng.randrange(1 << 32)
        await RisingEdge(dut.clk)
        for k, v in {**INPUTS, "host_req": 1, "host_we": 1, "host_addr": a,
                     "host_wdata": nw, "host_wstrb": st}.items():
            getattr(dut, k).value = v
        await FallingEdge(dut.clk)
        w = image[a]
        for b in range(4):
            if (st >> b) & 1:
                w = (w & ~(0xFF << (8 * b))) | (nw & (0xFF << (8 * b)))
        image[a] = w
    await idle_cycles(dut, 3)
    got = {}
    pending = {}
    for k in range(512 + 3):
        await RisingEdge(dut.clk)
        ov = dict(host_req=1, host_we=0, host_addr=k) if k < 512 else {}
        for n, v in {**INPUTS, **ov}.items():
            getattr(dut, n).value = v
        await FallingEdge(dut.clk)
        if k < 512:
            assert val(dut.host_gnt) == 1
        if val(dut.host_rvalid):
            a = k - 2
            got[a] = val(dut.host_rdata)
    assert [got[a] for a in range(512)] == image


@cocotb.test()
async def test_engine_detects_mismatch(dut):
    """Harness self-test: corrupting the reference model must be reported at once."""
    src = "LDI sr, 0x5A @2\nNOP @5\nPINS high uio0 @2\nHALT"
    words = scn.pad512(scn.asm_text(src))
    await reset(dut)
    await host_write_image(dut, words)
    ls = Lockstep(CocoDut(dut), words, W.BusWorld(), DefaultPolicy(), "selftest")
    await ls.run(12)
    ls.iss.sr ^= 0x10
    try:
        await ls.run(3)
    except LockstepError as e:
        assert "SR" in str(e), e
    else:
        raise AssertionError("lockstep engine did not notice a corrupted reference")


# ------------------------------------------------------------------ scenario registry
def _register(name, make):
    async def _t(dut):
        await run_scenario(dut, make())
    _t.__name__ = _t.__qualname__ = name
    _t.__doc__ = "lockstep scenario " + name
    globals()[name] = cocotb.test()(_t)


for _i, _sc in enumerate(scn.directed()):
    _register("test_directed_%s" % _sc.name, lambda i=_i: scn.directed()[i])

for _i, _sc in enumerate(scn.examples()):
    _register("test_example_%s" % _sc.name, lambda i=_i: scn.examples()[i])

for _i in range(12):
    _register("test_fuzz_%d" % _i, lambda i=_i: scn.scn_fuzz(i))
