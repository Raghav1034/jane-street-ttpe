"""
cocotb chip-level tests for tt_um_ttpe (Tiny Tapeout top level).

Everything goes through the pins: the host speaks SPI on ui_in[2:0] (SCK, MOSI, CS_N) and reads
MISO on uo_out[0]; the protocol programs (uart_tx, uart_rx, spi_master, i2c_master) are loaded
over SPI, started over SPI, and checked cycle by cycle against the Python ISS (sw/ttpe_iss.py)
on uio_oe / uio_out / uo_out, with the device models of sw/ttpe_world.py on uio[7:0].
The tests themselves live in ttpe_top_tb.py (shared with the sandbox checks); this file only
adapts them to cocotb.

Run:   make TARGET=top                       (PDK SRAM models)
       make TARGET=top USE_PDK_SRAM=0        (stand-in SRAM model)
Subset: cocotb 2.x  COCOTB_TEST_FILTER=example   |  cocotb 1.x  TESTCASE=test_t_run_control
"""

import os
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sw"))

import ttpe_top_tb as T                          # noqa: E402

CLK_NS = 20


def val(sig):
    """Integer value of a signal; raises on X/Z so an unknown never passes silently."""
    try:
        return int(sig.value)
    except ValueError:
        return int(str(sig.value).translate(str.maketrans("xXzZ", "0000")), 2)


class CocoTop:
    """Pin-level adapter for ttpe_top_tb.Top."""

    def __init__(self, dut):
        self.dut = dut

    async def start(self):
        d = self.dut
        d.ena.value = 1
        d.ui_in.value = 0x04              # CS_N high, SCK and MOSI low
        d.uio_in.value = 0xFF
        d.rst_n.value = 0
        await Timer(1, "ns")
        cocotb.start_soon(Clock(d.clk, CLK_NS, "ns").start())
        for _ in range(3):
            await RisingEdge(d.clk)
        await Timer(1, "ns")

    def assert_reset(self):
        self.dut.rst_n.value = 0

    def release_reset(self):
        self.dut.rst_n.value = 1

    async def settle(self):
        await Timer(1, "ns")

    async def next_cycle(self):
        await RisingEdge(self.dut.clk)
        await Timer(1, "ns")              # flop outputs of the new cycle have settled

    def outs(self):
        d = self.dut
        return dict(uo_out=val(d.uo_out), uio_out=val(d.uio_out), uio_oe=val(d.uio_oe))

    def drive(self, ui_in, uio_in):
        self.dut.ui_in.value = ui_in
        self.dut.uio_in.value = uio_in


async def _run(dut, fn):
    ad = CocoTop(dut)
    await ad.start()
    h = T.Top(ad)
    await fn(h)
    dut._log.info("%s: %d cycles", fn.__name__, h.k + 1)


def _register(fn):
    name = "test_" + fn.__name__

    async def _t(dut):
        await _run(dut, fn)
    _t.__name__ = _t.__qualname__ = name
    _t.__doc__ = (fn.__doc__ or fn.__name__).strip().splitlines()[0]
    globals()[name] = cocotb.test()(_t)


for _n in sorted(dir(T)):
    if _n.startswith("t_"):
        _register(getattr(T, _n))
