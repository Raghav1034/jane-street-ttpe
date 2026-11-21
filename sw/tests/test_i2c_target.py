"""Independent I2C master model and cycle-exact ISS tests for sw/examples/i2c_target.asm (WP4a).

The master model below is written independently from sw/examples/i2c_master.asm and checks
NXP UM10204 Table 10/11 timing bounds directly on the open-drain bus:
  - Open-drain discipline (target never drives SCL or SDA high)
  - Target internal SDA hold after SCL fall (t_HD;DAT >= 300 ns = 15 cycles at 50 MHz)
  - Target ACK/data valid delay (t_VD;ACK, t_VD;DAT <= 3.45 us [172 cyc] @ 100 kHz; 0.9 us [45 cyc] @ 400 kHz)
  - Target data setup before SCL rise (t_SU;DAT >= 250 ns [13 cyc] @ 100 kHz; 100 ns [5 cyc] @ 400 kHz)
  - No illegal target SDA changes while SCL is high
  - Clock stretching support (and strict non-stretching mode when TX FIFO is preloaded)
  - Optional bus rise time (t_r) and per-edge timing jitter
"""

import os
import random
import sys
import unittest

SW_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SW_DIR not in sys.path:
    sys.path.insert(0, SW_DIR)

import ttpe_asm as asm
import ttpe_iss as iss

ASM_PATH = os.path.join(SW_DIR, "examples", "i2c_target.asm")
SCL_BIT = 2   # uio2
SDA_BIT = 3   # uio3
ADDR_TOG_BIT = 5  # uo5
END_TOG_BIT = 6   # uo6


# UM10204 Table 10/11 timing specs in 20 ns cycles (50 MHz)
SPEC_100K = dict(
    name="100kHz",
    t_low=235,        # 4.7 us min
    t_high=200,       # 4.0 us min
    t_hd_sta=200,     # 4.0 us min
    t_su_sta=235,     # 4.7 us min
    t_su_sto=200,     # 4.0 us min
    t_buf=235,        # 4.7 us min
    t_su_dat_min=13,  # 250 ns min (12.5 -> 13 cycles)
    t_hd_dat_min=15,  # 300 ns internal hold (UM10204 Note)
    t_vd_dat_max=172, # 3.45 us max
    t_vd_ack_max=172, # 3.45 us max
)

SPEC_400K = dict(
    name="400kHz",
    t_low=65,         # 1.3 us min
    t_high=30,        # 0.6 us min
    t_hd_sta=30,      # 0.6 us min
    t_su_sta=30,      # 0.6 us min
    t_su_sto=30,      # 0.6 us min
    t_buf=65,         # 1.3 us min
    t_su_dat_min=5,   # 100 ns min
    t_hd_dat_min=15,  # 300 ns internal hold (UM10204 Note)
    t_vd_dat_max=45,  # 0.9 us max
    t_vd_ack_max=45,  # 0.9 us max
)


def assemble_target(**defs):
    with open(ASM_PATH, encoding="utf-8") as f:
        src = f.read()
    return asm.assemble(src, defines=defs).words


class IndependentI2CMaster:
    """Cycle-by-cycle open-drain I2C master + UM10204 target timing monitor.

    Transaction list format:
      ("W", addr7, [b0, b1, ...], stop_bool)   -> write bytes; stop_bool=False ends for Sr
      ("R", addr7, n_bytes,       stop_bool)   -> read n_bytes; stop_bool=False ends for Sr
    """

    def __init__(self, txns, spec=SPEC_100K, t_r=0, jitter=0, seed=1,
                 allow_stretch=True, master_hd_dat=0, idle_lead=20):
        self.txns = list(txns)
        self.spec = dict(spec)
        self.t_r = t_r
        self.jitter = jitter
        self.rng = random.Random(seed)
        self.allow_stretch = allow_stretch
        self.master_hd_dat = master_hd_dat

        # Open-drain master outputs (False = pull low, True = float/release)
        self.m_scl = True
        self.m_sda = True

        # Rise-time filter state for SCL and SDA
        self._scl_rel_cnt = t_r
        self._sda_rel_cnt = t_r
        self.bus_scl = 1
        self.bus_sda = 1
        self.prev_bus_scl = 1
        self.prev_bus_sda = 1

        # Target-side timing monitor state
        self.violations = []
        self.results = []
        self.stretch_events = 0
        self.max_stretch_cycles = 0
        self._cur_stretch = 0
        self._scl_fall_cyc = None
        self._target_sda_change_cyc = None
        self._target_prev_drive_low = False
        self._target_owns_sda = False
        self._check_vd_kind = None  # "ACK" or "DAT"
        self._was_stretched = False
        self._vd_checked = False

        # Transaction generator coroutine
        self._gen = self._run_all(idle_lead)
        self.done = False
        self._cycle = 0
        self._started = False

    def _jit(self, base):
        if self.jitter <= 0:
            return base
        return base + self.rng.randint(0, self.jitter)

    def _hold_cycles(self, n):
        for _ in range(max(0, n)):
            yield

    def _release_scl_and_wait(self):
        self.m_scl = True
        self._cur_stretch = 0
        stretched = False
        while True:
            yield
            if self.bus_scl == 1:
                break
            # If master released SCL and rise time elapsed, target is stretching
            if self._scl_rel_cnt >= self.t_r:
                if not self.allow_stretch:
                    self.violations.append(
                        f"cyc {self._cycle}: target stretched SCL when allow_stretch=False"
                    )
                    return
                stretched = True
                self._cur_stretch += 1
        if stretched:
            self.stretch_events += 1
            if self._cur_stretch > self.max_stretch_cycles:
                self.max_stretch_cycles = self._cur_stretch

    def _clock_write_bit(self, bit, allow_hd=True):
        # SCL is already low. Wait master_hd_dat, drive SDA, finish t_LOW, raise SCL, hold t_HIGH, drop SCL.
        s = self.spec
        hd = min(self.master_hd_dat, s["t_low"] - 1) if allow_hd else 0
        yield from self._hold_cycles(hd)
        self.m_sda = bool(bit)
        yield from self._hold_cycles(self._jit(s["t_low"]) - hd)
        yield from self._release_scl_and_wait()
        yield from self._hold_cycles(self._jit(s["t_high"]) - 1)
        self.m_scl = False

    def _clock_read_bit(self, kind="DAT"):
        # SCL is already low; release SDA so target drives bit/ACK.
        s = self.spec
        self._target_owns_sda = True
        self._check_vd_kind = kind
        self._vd_checked = False
        self._was_stretched = False
        self.m_sda = True
        yield from self._hold_cycles(self._jit(s["t_low"]))
        yield from self._release_scl_and_wait()
        sampled = self.bus_sda
        # Check t_SU;DAT against last target SDA transition (if target drove/changed SDA)
        if self._target_sda_change_cyc is not None and self._scl_fall_cyc is not None:
            if self._target_sda_change_cyc >= self._scl_fall_cyc:
                su = self._cycle - self._target_sda_change_cyc
                if su < s["t_su_dat_min"]:
                    self.violations.append(
                        f"cyc {self._cycle}: {kind} t_SU;DAT={su} cyc < {s['t_su_dat_min']}"
                    )
        yield from self._hold_cycles(self._jit(s["t_high"]) - 1)
        self._target_owns_sda = False
        self._check_vd_kind = None
        self.m_scl = False
        return sampled

    def _run_all(self, idle_lead):
        s = self.spec
        self.m_scl = True
        self.m_sda = True
        yield from self._hold_cycles(idle_lead)

        bus_active = False
        for op, addr7, arg, do_stop in self.txns:
            if not bus_active:
                # START: SCL=1, SDA 1->0, hold t_HD;STA, then SCL->0
                self.m_scl = True
                self.m_sda = True
                yield from self._hold_cycles(2)
                self.m_sda = False
                yield from self._hold_cycles(self._jit(s["t_hd_sta"]))
                self.m_scl = False
                bus_active = True
            else:
                # Repeated START (Sr): SCL is low -> release SDA -> t_LOW -> release SCL -> t_SU;STA -> SDA 1->0 -> t_HD;STA -> SCL->0
                self.m_sda = True
                yield from self._hold_cycles(self._jit(s["t_low"]))
                yield from self._release_scl_and_wait()
                yield from self._hold_cycles(self._jit(s["t_su_sta"]))
                self.m_sda = False
                yield from self._hold_cycles(self._jit(s["t_hd_sta"]))
                self.m_scl = False

            rw = 1 if op == "R" else 0
            addr_byte = ((addr7 & 0x7F) << 1) | rw
            for i in range(8):
                yield from self._clock_write_bit((addr_byte >> (7 - i)) & 1)
            ack_bit = yield from self._clock_read_bit(kind="ACK")
            addr_acked = (ack_bit == 0)

            if not addr_acked:
                self.results.append((op, addr7, False, []))
            elif op == "W":
                acks = []
                for b in arg:
                    for i in range(8):
                        yield from self._clock_write_bit((b >> (7 - i)) & 1)
                    ab = yield from self._clock_read_bit(kind="ACK")
                    acks.append(ab == 0)
                self.results.append(("W", addr7, True, acks))
            elif op == "R":
                got = []
                n = arg
                for bi in range(n):
                    val = 0
                    for i in range(8):
                        bit = yield from self._clock_read_bit(kind="DAT")
                        val = (val << 1) | bit
                    got.append(val)
                    # Master drives ACK (0) for all bytes except last (NACK = 1)
                    is_last = (bi == n - 1)
                    yield from self._clock_write_bit(1 if is_last else 0)
                self.results.append(("R", addr7, True, got))

            if do_stop:
                # STOP: SCL low -> pull SDA low -> t_LOW -> release SCL -> t_SU;STO -> release SDA -> t_BUF
                yield from self._hold_cycles(min(self.master_hd_dat, s["t_low"] - 1))
                self.m_sda = False
                yield from self._hold_cycles(self._jit(s["t_low"]) - min(self.master_hd_dat, s["t_low"] - 1))
                yield from self._release_scl_and_wait()
                yield from self._hold_cycles(self._jit(s["t_su_sto"]))
                self.m_sda = True
                yield from self._hold_cycles(self._jit(s["t_buf"]))
                bus_active = False

        # Trailing idle cycles so target completes STOP handling
        yield from self._hold_cycles(40)
        self.done = True
        while True:
            yield

    def step_bus(self, cycle, uio_oe, uio_out):
        self._cycle = cycle
        # Check open-drain discipline on SCL and SDA
        t_scl_oe = (uio_oe >> SCL_BIT) & 1
        t_scl_out = (uio_out >> SCL_BIT) & 1
        t_sda_oe = (uio_oe >> SDA_BIT) & 1
        t_sda_out = (uio_out >> SDA_BIT) & 1
        if t_scl_oe and t_scl_out:
            self.violations.append(f"cyc {cycle}: target drove SCL push-pull HIGH")
        if t_sda_oe and t_sda_out:
            self.violations.append(f"cyc {cycle}: target drove SDA push-pull HIGH")

        t_scl_low = bool(t_scl_oe and not t_scl_out)
        t_sda_low = bool(t_sda_oe and not t_sda_out)

        # Advance master coroutine one cycle
        if not self._started:
            self._started = True
        next(self._gen)

        # Recompute wired-AND bus levels with optional rise-time t_r
        raw_scl_low = (not self.m_scl) or t_scl_low
        raw_sda_low = (not self.m_sda) or t_sda_low

        if raw_scl_low:
            self._scl_rel_cnt = 0
            new_scl = 0
        else:
            if self._scl_rel_cnt < self.t_r:
                self._scl_rel_cnt += 1
                new_scl = 0
            else:
                new_scl = 1

        if raw_sda_low:
            self._sda_rel_cnt = 0
            new_sda = 0
        else:
            if self._sda_rel_cnt < self.t_r:
                self._sda_rel_cnt += 1
                new_sda = 0
            else:
                new_sda = 1

        self.prev_bus_scl, self.prev_bus_sda = self.bus_scl, self.bus_sda
        self.bus_scl, self.bus_sda = new_scl, new_sda

        if self.prev_bus_scl == 1 and self.bus_scl == 0:
            self._scl_fall_cyc = cycle
            self._was_stretched = False

        if t_scl_low and self.m_scl:
            self._was_stretched = True

        # Monitor target SDA transitions for t_HD;DAT, t_VD;ACK, t_VD;DAT, and glitch during SCL=1
        if t_sda_low != self._target_prev_drive_low:
            self._target_prev_drive_low = t_sda_low
            self._target_sda_change_cyc = cycle
            if self.bus_scl == 1 and self._target_owns_sda:
                self.violations.append(f"cyc {cycle}: target changed SDA while SCL=1")
            if self._scl_fall_cyc is not None:
                dt = cycle - self._scl_fall_cyc
                if dt < self.spec["t_hd_dat_min"]:
                    self.violations.append(
                        f"cyc {cycle}: target changed SDA at SCL_fall+{dt} < t_HD;DAT={self.spec['t_hd_dat_min']}"
                    )
                if self._check_vd_kind == "ACK" and t_sda_low and not self._was_stretched:
                    if dt > self.spec["t_vd_ack_max"]:
                        self.violations.append(
                            f"cyc {cycle}: t_VD;ACK={dt} > {self.spec['t_vd_ack_max']}"
                        )
                if self._check_vd_kind == "DAT" and not self._was_stretched:
                    if dt > self.spec["t_vd_dat_max"]:
                        self.violations.append(
                            f"cyc {cycle}: t_VD;DAT={dt} > {self.spec['t_vd_dat_max']}"
                        )

        uio_in = 0xFF
        if self.bus_scl == 0:
            uio_in &= ~(1 << SCL_BIT)
        if self.bus_sda == 0:
            uio_in &= ~(1 << SDA_BIT)
        return uio_in


    def pads(self, cycle, uio_oe, uio_out, _uo_out):
        return self.step_bus(cycle, uio_oe, uio_out), 0xFF


def run_i2c_target(txns, spec=SPEC_100K, host_tx=(), defs=None,
                   t_r=0, jitter=0, seed=1, allow_stretch=True,
                   master_hd_dat=0, max_cycles=250000):
    words = assemble_target(**(defs or {}))
    cpu = iss.Ttpe(words, host_tx=list(host_tx))
    master = IndependentI2CMaster(
        txns, spec=spec, t_r=t_r, jitter=jitter, seed=seed,
        allow_stretch=allow_stretch, master_hd_dat=master_hd_dat,
    )
    addr_toggles = 0
    end_toggles = 0
    prev_uo = 0
    for _ in range(max_cycles):
        cpu.step(master)
        if ((cpu.uo_out ^ prev_uo) >> ADDR_TOG_BIT) & 1:
            addr_toggles += 1
        if ((cpu.uo_out ^ prev_uo) >> END_TOG_BIT) & 1:
            end_toggles += 1
        prev_uo = cpu.uo_out
        if master.done:
            break
    assert master.done, f"master did not complete within {max_cycles} cycles"
    rx_bytes = [b for _, b in cpu.rx]
    return cpu, rx_bytes, master, addr_toggles, end_toggles


class TestI2CTarget(unittest.TestCase):
    def test_100k_write_and_read_spec_minimums(self):
        txns = [
            ("W", 0x50, [0x01, 0xA5, 0x00, 0xFF, 0x5A], True),
            ("R", 0x50, 4, True),
        ]
        cpu, rx, master, addr_tog, end_tog = run_i2c_target(
            txns, spec=SPEC_100K, host_tx=[0xDE, 0xAD, 0xBE, 0xEF],
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results[0], ("W", 0x50, True, [True] * 5))
        self.assertEqual(master.results[1], ("R", 0x50, True, [0xDE, 0xAD, 0xBE, 0xEF]))
        self.assertEqual(rx, [0x01, 0xA5, 0x00, 0xFF, 0x5A])
        self.assertEqual(cpu.rx_overrun, 0)
        self.assertEqual(addr_tog, 2)
        self.assertEqual(end_tog, 2)

    def test_100k_repeated_start_register_read(self):
        # Classic S A+W REG Sr A+R d0 d1 P, plus Sr back into Write and STOP
        txns = [
            ("W", 0x50, [0x42], False),
            ("R", 0x50, 2, False),
            ("W", 0x50, [0x99, 0x11], True),
        ]
        _, rx, master, addr_tog, end_tog = run_i2c_target(
            txns, spec=SPEC_100K, host_tx=[0x12, 0x34],
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [
            ("W", 0x50, True, [True]),
            ("R", 0x50, True, [0x12, 0x34]),
            ("W", 0x50, True, [True, True]),
        ])
        self.assertEqual(rx, [0x42, 0x99, 0x11])
        self.assertEqual(addr_tog, 3)
        self.assertEqual(end_tog, 3)

    def test_100k_address_mismatch_nack_and_multi_target_bus(self):
        # Traffic to other targets (0x51, 0x20), then Sr directly to our target 0x50
        txns = [
            ("W", 0x51, [0xAA], True),
            ("R", 0x20, 1, False),
            ("W", 0x50, [0x7E], True),
        ]
        _, rx, master, addr_tog, end_tog = run_i2c_target(txns, spec=SPEC_100K)
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [
            ("W", 0x51, False, []),
            ("R", 0x20, False, []),
            ("W", 0x50, True, [True]),
        ])
        self.assertEqual(rx, [0x7E])
        self.assertEqual(addr_tog, 1)
        self.assertEqual(end_tog, 1)

    def test_100k_clock_stretching_on_late_tx_fifo(self):
        # Host supplies Read bytes late (after master already released SCL -> target stretches SCL)
        txns = [("R", 0x50, 2, True)]
        # Address byte + ACK takes ~9 * 435 = ~4000 cycles; schedule byte 0 at 6500, byte 1 at 12500
        _, _, master, _, _ = run_i2c_target(
            txns, spec=SPEC_100K, host_tx=[(6500, 0xC3), (12500, 0x5A)],
            allow_stretch=True,
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [("R", 0x50, True, [0xC3, 0x5A])])
        self.assertGreaterEqual(master.stretch_events, 2)
        self.assertGreater(master.max_stretch_cycles, 100)

    def test_100k_preloaded_tx_fifo_needs_no_stretch(self):
        # Master with allow_stretch=False succeeds when TX FIFO is preloaded
        txns = [("R", 0x50, 3, True)]
        _, _, master, _, _ = run_i2c_target(
            txns, spec=SPEC_100K, host_tx=[0x11, 0x22, 0x33], allow_stretch=False,
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [("R", 0x50, True, [0x11, 0x22, 0x33])])
        self.assertEqual(master.stretch_events, 0)

    def test_100k_with_rise_time_and_jitter(self):
        # Model 500 ns rise time (25 cycles) + random jitter up to 10 cycles + t_HD;DAT=0
        txns = [
            ("W", 0x50, [0x00, 0xFF, 0x80, 0x01], False),
            ("R", 0x50, 3, True),
        ]
        _, rx, master, _, _ = run_i2c_target(
            txns, spec=SPEC_100K, host_tx=[0xFE, 0x01, 0x80],
            t_r=25, jitter=10, seed=7, master_hd_dat=0,
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results[0], ("W", 0x50, True, [True] * 4))
        self.assertEqual(master.results[1], ("R", 0x50, True, [0xFE, 0x01, 0x80]))
        self.assertEqual(rx, [0x00, 0xFF, 0x80, 0x01])

    def test_100k_rx_fifo_overflow_documented_behaviour(self):
        # 10-byte write into 8-deep RX FIFO: target ACKs all 10, first 8 kept, rx_overrun == 2
        payload = list(range(10))
        txns = [("W", 0x50, payload, True)]
        cpu, rx, master, _, _ = run_i2c_target(txns, spec=SPEC_100K)
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [("W", 0x50, True, [True] * 10)])
        self.assertEqual(rx, payload[:8])
        self.assertEqual(cpu.rx_overrun, 2)

    def test_400k_model_feasible_parameterized_build(self):
        # 400 kHz Fast-mode (-D TSU_DAT=6): model-feasible at spec mins with 0-ns master t_HD;DAT,
        # Sr, STOP, and late-TX clock stretching
        txns = [
            ("W", 0x2A, [0x00, 0xFF, 0x55], False),
            ("R", 0x2A, 2, True),
            ("W", 0x2A, [0xA0], True),
        ]
        _, rx, master, addr_tog, end_tog = run_i2c_target(
            txns, spec=SPEC_400K, defs=dict(ADDR=0x2A, TSU_DAT=6),
            host_tx=[(5000, 0x81), (8000, 0x7E)], jitter=3, seed=11, master_hd_dat=0,
        )
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [
            ("W", 0x2A, True, [True, True, True]),
            ("R", 0x2A, True, [0x81, 0x7E]),
            ("W", 0x2A, True, [True]),
        ])
        self.assertEqual(rx, [0x00, 0xFF, 0x55, 0xA0])
        self.assertGreaterEqual(master.stretch_events, 1)
        self.assertEqual(addr_tog, 3)
        self.assertEqual(end_tog, 3)

    def test_zero_byte_write_then_sr_and_stop(self):
        # Address write immediately followed by Sr (0 data bytes) and address write followed by STOP
        txns = [
            ("W", 0x50, [], False),
            ("W", 0x50, [], True),
        ]
        _, rx, master, addr_tog, end_tog = run_i2c_target(txns, spec=SPEC_100K)
        self.assertEqual(master.violations, [])
        self.assertEqual(master.results, [
            ("W", 0x50, True, []),
            ("W", 0x50, True, []),
        ])
        self.assertEqual(rx, [])
        self.assertEqual(addr_tog, 2)
        self.assertEqual(end_tog, 2)

    def test_negative_controls_timing_checker_catches_violations(self):
        # 1) Using 400 kHz TSU_DAT=6 (120 ns) under 100 kHz spec (250 ns min) with clock stretch MUST fail t_SU;DAT
        #    (0xAA has MSB=1 so SDA transitions 0->1 from address ACK 6 cycles before SCL release)
        _, _, m_su, _, _ = run_i2c_target(
            [("R", 0x50, 1, True)], spec=SPEC_100K, defs=dict(TSU_DAT=6),
            host_tx=[(6500, 0xAA)], allow_stretch=True,
        )
        self.assertTrue(any("t_SU;DAT" in v for v in m_su.violations), m_su.violations)

        # 2) Reducing THD_DAT=12 (240 ns < 300 ns UM10204 Note) MUST fail t_HD;DAT
        _, _, m_hd, _, _ = run_i2c_target(
            [("W", 0x50, [0x55], True)], spec=SPEC_100K, defs=dict(THD_DAT=12),
        )
        self.assertTrue(any("t_HD;DAT" in v for v in m_hd.violations), m_hd.violations)

        # 3) Late TX FIFO with allow_stretch=False MUST flag stretch violation
        _, _, m_st, _, _ = run_i2c_target(
            [("R", 0x50, 1, True)], spec=SPEC_100K, host_tx=[(6500, 0x55)], allow_stretch=False,
        )
        self.assertTrue(any("stretched SCL" in v for v in m_st.violations), m_st.violations)

        # 4) Tightening t_vd_ack_max to 20 cycles (< 27 cycles address-match ACK) MUST flag t_VD;ACK
        tight = dict(SPEC_400K, t_vd_ack_max=20)
        _, _, m_vd, _, _ = run_i2c_target(
            [("W", 0x50, [0x55], True)], spec=tight, defs=dict(TSU_DAT=6),
        )
        self.assertTrue(any("t_VD;ACK" in v for v in m_vd.violations), m_vd.violations)


if __name__ == "__main__":
    unittest.main()
