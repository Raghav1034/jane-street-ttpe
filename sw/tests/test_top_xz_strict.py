"""Unit tests for post-reset X/Z strictness in test/test_ttpe_top.py (WP2b).

Imports the real test/test_ttpe_top.py with lightweight stdlib stubs for cocotb and
ttpe_top_tb so the checks run in pure Python without requiring a simulator DLL.
"""

import asyncio
import importlib.util
import os
import sys
import types
import unittest
import warnings
from unittest import mock

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
TOP_TEST_PATH = os.path.join(REPO_ROOT, "test", "test_ttpe_top.py")


class _FakeBin:
    def __init__(self, bits):
        self._bits = bits

    def __int__(self):
        return int(self._bits, 2)

    def __str__(self):
        return self._bits


class _FakeSig:
    def __init__(self, name, bits="00000000"):
        self._name = name
        self.value = _FakeBin(bits)

    def __str__(self):
        warnings.warn("str(handle) deprecated", DeprecationWarning, stacklevel=2)
        return self._name


class _FakeDut:
    def __init__(self):
        self.clk = _FakeSig("clk", "0")
        self.rst_n = _FakeSig("rst_n", "0")
        self.ena = _FakeSig("ena", "0")
        self.ui_in = _FakeSig("ui_in", "00000000")
        self.uio_in = _FakeSig("uio_in", "00000000")
        self.uo_out = _FakeSig("uo_out", "00000000")
        self.uio_out = _FakeSig("uio_out", "00000000")
        self.uio_oe = _FakeSig("uio_oe", "00000000")


def _load_top_module():
    cocotb_mod = types.ModuleType("cocotb")
    cocotb_mod.test = lambda: (lambda fn: fn)
    cocotb_mod.start_soon = lambda coro: coro

    clock_mod = types.ModuleType("cocotb.clock")

    class _Clock:
        def __init__(self, sig, period, units="ns"):
            pass

        def start(self):
            return None

    clock_mod.Clock = _Clock

    triggers_mod = types.ModuleType("cocotb.triggers")

    async def _noop(*_args, **_kwargs):
        return None

    triggers_mod.RisingEdge = _noop
    triggers_mod.Timer = _noop

    tb_mod = types.ModuleType("ttpe_top_tb")

    stubs = {
        "cocotb": cocotb_mod,
        "cocotb.clock": clock_mod,
        "cocotb.triggers": triggers_mod,
        "ttpe_top_tb": tb_mod,
    }
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location("_ttpe_top_under_test", TOP_TEST_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod


class TestTopXZStrict(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert os.path.exists(TOP_TEST_PATH), f"missing {TOP_TEST_PATH}"
        cls.top = _load_top_module()

    def test_val_clean_binary(self):
        for bits, expected in (("00000000", 0x00), ("10100101", 0xA5), ("11111111", 0xFF)):
            sig = _FakeSig("uo_out", bits)
            self.assertEqual(self.top.val(sig, strict=True), expected)
            self.assertEqual(self.top.val(sig, strict=False), expected)
            self.assertEqual(self.top.val(sig), expected)

    def test_val_strict_raises_on_x_z_and_mixed(self):
        bad_vectors = ("0000000X", "x0000000", "0000000Z", "z1010101", "01x1z0X1")
        for bits in bad_vectors:
            sig = _FakeSig("uo_out", bits)
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                with self.assertRaisesRegex(ValueError, r"uo_out contains X/Z after reset"):
                    self.top.val(sig, strict=True)
                with self.assertRaises(ValueError):
                    self.top.val(sig)
            dep = [w for w in caught if issubclass(w.category, DeprecationWarning)]
            self.assertEqual(dep, [], f"unexpected DeprecationWarning for {bits}")

    def test_val_nonstrict_masks_xz(self):
        sig = _FakeSig("uo_out", "01x1z0X1")
        self.assertEqual(self.top.val(sig, strict=False), 0b01010001)
        self.assertEqual(self.top.val(_FakeSig("uio_oe", "ZZZZZZZZ"), strict=False), 0)
        self.assertEqual(self.top.val(_FakeSig("uio_out", "XXXXXXXX"), strict=False), 0)

    def test_cocotb_adapter_post_reset_gating(self):
        dut = _FakeDut()
        ad = self.top.CocoTop(dut)

        # Pre-reset and during active reset (start): X/Z must NOT raise
        dut.uo_out.value = _FakeBin("0000000X")
        dut.uio_out.value = _FakeBin("0000Z000")
        dut.uio_oe.value = _FakeBin("x000000z")
        self.assertEqual(ad.outs(), {"uo_out": 0, "uio_out": 0, "uio_oe": 0})

        asyncio.run(ad.start())
        self.assertEqual(ad.outs(), {"uo_out": 0, "uio_out": 0, "uio_oe": 0})

        # After release_reset(): clean binary passes, X/Z on ANY output bus raises
        ad.release_reset()
        dut.uo_out.value = _FakeBin("01010101")
        dut.uio_out.value = _FakeBin("10101010")
        dut.uio_oe.value = _FakeBin("11110000")
        self.assertEqual(ad.outs(), {"uo_out": 0x55, "uio_out": 0xAA, "uio_oe": 0xF0})

        for bus in ("uo_out", "uio_out", "uio_oe"):
            for bad in ("0000000X", "0000000Z", "01x1z0X1"):
                dut.uo_out.value = _FakeBin("00000000")
                dut.uio_out.value = _FakeBin("00000000")
                dut.uio_oe.value = _FakeBin("00000000")
                getattr(dut, bus).value = _FakeBin(bad)
                with self.assertRaisesRegex(ValueError, rf"{bus} contains X/Z after reset"):
                    ad.outs()

        # Re-asserting reset disables strictness again until next release_reset()
        ad.assert_reset()
        dut.uo_out.value = _FakeBin("XXXXXXXX")
        dut.uio_out.value = _FakeBin("00000000")
        dut.uio_oe.value = _FakeBin("ZZZZZZZZ")
        self.assertEqual(ad.outs(), {"uo_out": 0, "uio_out": 0, "uio_oe": 0})


if __name__ == "__main__":
    unittest.main()
