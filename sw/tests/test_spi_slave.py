"""Independent SPI master model and cycle-exact ISS tests for sw/examples/spi_slave.asm (WP4b).

The master model below is written independently from sw/examples/spi_master.asm and
sw/ttpe_world.py:SpiSlave, checking de-facto Motorola SPI timing bounds (Modes 0..3)
directly on the pins (uio0=CS_N, uio1=MOSI, uio2=MISO, uio3=SCK, uo6=END):
  - Slave never drives CS_N, MOSI, or SCK (uio_oe & 0b1011 == 0)
  - MISO tri-state (uio_oe[2] == 0) whenever CS_N == 1 (within t_dis_max after CS_N rise)
  - MISO actively driven and stable for >= miso_setup_min cycles before every sampling edge
  - MISO held stable for >= miso_hold_min cycles after every sampling edge
  - Full-duplex MOSI/MISO data integrity across all 4 modes (CPOL/CPHA), default 1.0 MHz (H=25),
    documented ceilings (H=12 CPHA=0; H=20 CPHA=1), jitter, 9-phase poll sweep, and negative controls
  - Optional two-ISS cross-test (spi_master.asm <-> spi_slave.asm) across all 4 modes
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

ASM_PATH = os.path.join(SW_DIR, "examples", "spi_slave.asm")
MASTER_ASM_PATH = os.path.join(SW_DIR, "examples", "spi_master.asm")

CS_BIT = 0    # uio0
MOSI_BIT = 1  # uio1
MISO_BIT = 2  # uio2
SCK_BIT = 3   # uio3
END_BIT = 6   # uo6


def assemble_slave(**defs):
    with open(ASM_PATH, encoding="utf-8") as f:
        return asm.assemble(f.read(), defines=defs).words


def assemble_master(**defs):
    with open(MASTER_ASM_PATH, encoding="utf-8") as f:
        return asm.assemble(f.read(), defines=defs).words


class IndependentSPIMaster:
    """Cycle-by-cycle independent SPI master + slave timing monitor (Modes 0..3).

    frames: list of byte lists to transmit on MOSI (one CS_N low pulse per frame;
            empty list [] sends a CS_N low pulse with 0 SCK clocks).
    Default half_period H=25 corresponds to 1.0 MHz SCK at 50 MHz (20 ns/cycle).
    """

    def __init__(self, frames, cpol=0, cpha=0, half_period=25,
                 t_lead=25, t_lag=25, t_cs_hi=40,
                 jitter=0, seed=1, miso_setup_min=2, miso_hold_min=2,
                 t_dis_max=None, idle_lead=30):
        self.frames = [list(fr) for fr in frames]
        self.cpol = int(bool(cpol))
        self.cpha = int(bool(cpha))
        self.half_period = half_period
        self.t_lead = t_lead
        self.t_lag = t_lag
        self.t_cs_hi = t_cs_hi
        self.jitter = jitter
        self.rng = random.Random(seed)
        self.miso_setup_min = miso_setup_min
        self.miso_hold_min = miso_hold_min
        # Default Hi-Z bound matches documented t_dis per CPHA (6 cyc CPHA=0; 17 cyc CPHA=1 polled)
        if t_dis_max is None:
            self.t_dis_max = 6 if self.cpha == 0 else 17
        else:
            self.t_dis_max = t_dis_max

        # Master outputs
        self.m_cs = 1
        self.m_mosi = 0
        self.m_sck = self.cpol

        # Monitor state
        self.violations = []
        self.rx_frames = []
        self.min_miso_setup = None
        self.max_hiz_latency = 0
        self._cycle = 0
        self._prev_miso_oe = 0
        self._prev_miso_val = None
        self._miso_change_cyc = None
        self._last_sample_cyc = None
        self._cs_rise_cyc = None
        self._last_trail_cyc = None
        self._hiz_recorded = True
        self._last_miso_oe = 0
        self._last_miso_out = 0
        self.done = False
        self._gen = self._run_all(idle_lead)

    def _jit(self, base):
        if self.jitter <= 0:
            return base
        return base + self.rng.randint(0, self.jitter)

    def _hold(self, n):
        for _ in range(max(0, n)):
            yield

    def _sample_miso(self, byte_idx, bit_idx):
        # Verify MISO is actively driven and met setup time before this sampling edge
        if not self._last_miso_oe:
            self.violations.append(
                f"cyc {self._cycle}: MISO floating (uio_oe[2]==0) at sample edge byte={byte_idx} bit={7-bit_idx}"
            )
            return 1
        if self._miso_change_cyc is not None:
            su = self._cycle - self._miso_change_cyc
            if self.min_miso_setup is None or su < self.min_miso_setup:
                self.min_miso_setup = su
            if su < self.miso_setup_min:
                self.violations.append(
                    f"cyc {self._cycle}: MISO setup={su} cyc < {self.miso_setup_min} (byte={byte_idx} bit={7-bit_idx})"
                )
        self._last_sample_cyc = self._cycle
        return self._last_miso_out

    def _run_all(self, idle_lead):
        self.m_cs = 1
        self.m_sck = self.cpol
        self.m_mosi = 0
        yield from self._hold(idle_lead)

        lead_lvl = 1 - self.cpol
        trail_lvl = self.cpol

        for fr in self.frames:
            # Assert CS_N low
            self.m_cs = 0
            self._cs_rise_cyc = None
            self._hiz_recorded = True
            self._last_sample_cyc = None
            if self.cpha == 0 and fr:
                # CPHA=0: MOSI bit 7 valid before first leading edge
                self.m_mosi = (fr[0] >> 7) & 1
            yield from self._hold(self.t_lead)

            rx_bytes = []
            for bi, tx_b in enumerate(fr):
                rx_b = 0
                for bit_i in range(8):
                    mosi_bit = (tx_b >> (7 - bit_i)) & 1
                    if self.cpha == 0:
                        # Drive MOSI before leading edge, sample MISO on leading edge
                        self.m_mosi = mosi_bit
                        self.m_sck = lead_lvl
                        yield  # leading edge cycle: slave sees edge in +2, master samples MISO now
                        miso_bit = self._sample_miso(bi, bit_i)
                        rx_b = (rx_b << 1) | miso_bit
                        yield from self._hold(self._jit(self.half_period) - 1)
                        # Trailing edge: slave shifts next MISO bit; master prepares next MOSI bit
                        self.m_sck = trail_lvl
                        self._last_trail_cyc = self._cycle
                        if bit_i < 7:
                            self.m_mosi = (tx_b >> (6 - bit_i)) & 1
                        elif bi + 1 < len(fr):
                            self.m_mosi = (fr[bi + 1] >> 7) & 1
                        yield from self._hold(self._jit(self.half_period))
                    else:
                        # CPHA=1: Leading edge shifts MOSI and MISO; trailing edge samples
                        self.m_mosi = mosi_bit
                        self.m_sck = lead_lvl
                        yield from self._hold(self._jit(self.half_period))
                        self.m_sck = trail_lvl
                        self._last_trail_cyc = self._cycle
                        yield  # trailing edge cycle: master samples MISO now
                        miso_bit = self._sample_miso(bi, bit_i)
                        rx_b = (rx_b << 1) | miso_bit
                        yield from self._hold(self._jit(self.half_period) - 1)
                rx_bytes.append(rx_b)

            self.rx_frames.append(rx_bytes)
            yield from self._hold(self.t_lag)
            self.m_cs = 1
            self.m_sck = self.cpol
            self._cs_rise_cyc = self._cycle
            self._hiz_recorded = False
            yield from self._hold(self.t_cs_hi)

        # Trailing idle so last frame_end completes
        yield from self._hold(40)
        self.done = True
        while True:
            yield

    def step_bus(self, cycle, uio_oe, uio_out):
        self._cycle = cycle
        # Slave must NEVER drive CS_N, MOSI, or SCK
        bad_oe = uio_oe & ((1 << CS_BIT) | (1 << MOSI_BIT) | (1 << SCK_BIT))
        if bad_oe:
            self.violations.append(f"cyc {cycle}: slave drove input pin(s) uio_oe=0x{uio_oe:02X}")

        miso_oe = (uio_oe >> MISO_BIT) & 1
        miso_out = (uio_out >> MISO_BIT) & 1
        self._last_miso_oe = miso_oe
        self._last_miso_out = miso_out

        # Track MISO transitions (enable or level change) for setup and hold checks
        miso_changed = (miso_oe != self._prev_miso_oe) or (miso_oe and miso_out != self._prev_miso_val)
        if miso_changed:
            if self._last_sample_cyc is not None and self.m_cs == 0:
                hd = cycle - self._last_sample_cyc
                if hd < self.miso_hold_min:
                    self.violations.append(
                        f"cyc {cycle}: MISO hold={hd} cyc < {self.miso_hold_min} after sample at {self._last_sample_cyc}"
                    )
            self._miso_change_cyc = cycle
            self._prev_miso_oe = miso_oe
            self._prev_miso_val = miso_out if miso_oe else None

        # Check MISO tri-state latency after CS_N rises
        if self.m_cs == 1:
            if not self._hiz_recorded:
                if miso_oe == 0:
                    lat = cycle - self._cs_rise_cyc
                    if lat > self.max_hiz_latency:
                        self.max_hiz_latency = lat
                    self._hiz_recorded = True
                else:
                    # Allow up to t_dis_max after CS_N rise, or up to (24 if cpha else 12) after T_T7 if t_lag=0
                    lat_cs = cycle - self._cs_rise_cyc
                    lat_tr = (cycle - self._last_trail_cyc) if self._last_trail_cyc is not None else lat_cs
                    bound_tr = (24 if self.cpha else 12)
                    if lat_cs > self.t_dis_max and lat_tr > bound_tr:
                        self.violations.append(
                            f"cyc {cycle}: MISO still driven {lat_cs} cyc after CS_N rise (limit {self.t_dis_max})"
                        )
            elif miso_oe == 1:
                self.violations.append(f"cyc {cycle}: MISO driven while CS_N idle high")

        next(self._gen)

        uio_in = 0xFF
        if self.m_cs == 0:
            uio_in &= ~(1 << CS_BIT)
        if self.m_mosi == 0:
            uio_in &= ~(1 << MOSI_BIT)
        if self.m_sck == 0:
            uio_in &= ~(1 << SCK_BIT)
        if miso_oe and miso_out == 0:
            uio_in &= ~(1 << MISO_BIT)
        return uio_in

    def pads(self, cycle, uio_oe, uio_out, _uo_out):
        return self.step_bus(cycle, uio_oe, uio_out), 0xFF


def run_spi_slave(frames, host_tx, cpol=0, cpha=0, half_period=25,
                  t_lead=25, t_lag=25, t_cs_hi=40, jitter=0, seed=1,
                  miso_setup_min=2, miso_hold_min=2, t_dis_max=None, max_cycles=120000):
    words = assemble_slave(CPOL=cpol, CPHA=cpha)
    cpu = iss.Ttpe(words, host_tx=list(host_tx))
    master = IndependentSPIMaster(
        frames, cpol=cpol, cpha=cpha, half_period=half_period,
        t_lead=t_lead, t_lag=t_lag, t_cs_hi=t_cs_hi, jitter=jitter, seed=seed,
        miso_setup_min=miso_setup_min, miso_hold_min=miso_hold_min, t_dis_max=t_dis_max,
    )
    end_toggles = 0
    prev_uo = 0
    for _ in range(max_cycles):
        cpu.step(master)
        if ((cpu.uo_out ^ prev_uo) >> END_BIT) & 1:
            end_toggles += 1
        prev_uo = cpu.uo_out
        if master.done:
            break
    assert master.done, f"SPI master did not complete within {max_cycles} cycles"
    rx_bytes = [b for _, b in cpu.rx]
    return cpu, rx_bytes, master, end_toggles


def run_spi_cross(cpol, cpha, master_frames, slave_tx_frames, half_period=25, max_cycles=80000):
    """Step two ttpe_iss.Ttpe instances (spi_master.asm <-> spi_slave.asm) on shared pins."""
    # spi_master host TX format: [n, d0..d(n-1)] per frame
    m_tx = []
    for fr in master_frames:
        m_tx += [len(fr)] + list(fr)
    # spi_slave host TX format: CPHA=0 uses [n, d0..d(n-1)], CPHA=1 uses raw [d0..d(n-1)]
    s_tx = []
    for fr in slave_tx_frames:
        s_tx += ([len(fr)] + list(fr)) if cpha == 0 else list(fr)

    m_cpu = iss.Ttpe(assemble_master(CPOL=cpol, CPHA=cpha, H=half_period), host_tx=m_tx)
    s_cpu = iss.Ttpe(assemble_slave(CPOL=cpol, CPHA=cpha), host_tx=s_tx)

    class _Wire:
        def __init__(self):
            self.uio = 0xFF

        def pads(self, _cyc, _oe, _out, _uo):
            return self.uio, 0xFF

    wire = _Wire()
    contentions = []
    for cyc in range(max_cycles):
        # Check no two chips drive the same pin simultaneously
        both = m_cpu.uio_oe & s_cpu.uio_oe
        if both:
            contentions.append(f"cyc {cyc}: pin contention mask=0x{both:02X}")
        # Pull-up default 1; whichever chip enables output drives its level
        pad = 0xFF
        for bit in range(8):
            if (m_cpu.uio_oe >> bit) & 1:
                if not ((m_cpu.uio_out >> bit) & 1):
                    pad &= ~(1 << bit)
            elif (s_cpu.uio_oe >> bit) & 1:
                if not ((s_cpu.uio_out >> bit) & 1):
                    pad &= ~(1 << bit)
        wire.uio = pad
        m_cpu.step(wire)
        s_cpu.step(wire)
    return dict(
        master_rx=[b for _, b in m_cpu.rx],
        slave_rx=[b for _, b in s_cpu.rx],
        contentions=contentions,
        master_illegal=m_cpu.illegal,
        slave_illegal=s_cpu.illegal,
    )


class TestSPISlave(unittest.TestCase):
    def test_mode0_default_1mhz_multi_frame(self):
        # Mode 0 (CPOL=0, CPHA=0) at default 1.0 MHz (H=25): frames of length 1, 4, 2
        mosi = [[0xA5], [0x00, 0xFF, 0x81, 0x7E], [0x12, 0x34]]
        miso = [[0x5A], [0xDE, 0xAD, 0xBE, 0xEF], [0xC3, 0x0F]]
        # CPHA=0 host TX uses [n, d0..d(n-1)] per frame (schedule 3rd frame after 1st drains TX FIFO)
        host_tx = [1, 0x5A, 4, 0xDE, 0xAD, 0xBE, 0xEF, (1000, 2), (1010, 0xC3), (1020, 0x0F)]
        cpu, rx, m, end_tog = run_spi_slave(
            mosi, host_tx, cpol=0, cpha=0, half_period=25, miso_setup_min=10, miso_hold_min=5,
        )
        self.assertEqual(m.violations, [])
        self.assertEqual(m.rx_frames, miso)
        self.assertEqual(rx, [b for fr in mosi for b in fr])
        self.assertEqual(cpu.rx_overrun, 0)
        self.assertEqual(end_tog, 3)
        self.assertLessEqual(m.max_hiz_latency, 6)

    def test_mode1_default_1mhz_variable_length(self):
        # Mode 1 (CPOL=0, CPHA=1) at default 1.0 MHz (H=25): variable-length incl. 0-byte CS pulse
        mosi = [[], [0xA5], [0x11, 0x22, 0x33], [0x80, 0x01]]
        miso = [[], [0x5A], [0xFE, 0xDC, 0xBA], [0x7F, 0x80]]
        host_tx = [0x5A, 0xFE, 0xDC, 0xBA, 0x7F, 0x80]
        cpu, rx, m, end_tog = run_spi_slave(
            mosi, host_tx, cpol=0, cpha=1, half_period=25, miso_setup_min=6, miso_hold_min=5,
        )
        self.assertEqual(m.violations, [])
        self.assertEqual(m.rx_frames, miso)
        self.assertEqual(rx, [b for fr in mosi for b in fr])
        self.assertEqual(cpu.rx_overrun, 0)
        self.assertEqual(end_tog, 4)
        self.assertLessEqual(m.max_hiz_latency, 17)

    def test_mode2_default_1mhz_multi_frame(self):
        # Mode 2 (CPOL=1, CPHA=0) at default 1.0 MHz (H=25): frames of length 1, 3, 2
        mosi = [[0x96], [0x01, 0x80, 0xFF], [0xAA, 0x55]]
        miso = [[0x69], [0xF0, 0x0F, 0x55], [0x33, 0xCC]]
        host_tx = [1, 0x69, 3, 0xF0, 0x0F, 0x55, (1000, 2), (1010, 0x33), (1020, 0xCC)]
        cpu, rx, m, end_tog = run_spi_slave(
            mosi, host_tx, cpol=1, cpha=0, half_period=25, miso_setup_min=10, miso_hold_min=5,
        )
        self.assertEqual(m.violations, [])
        self.assertEqual(m.rx_frames, miso)
        self.assertEqual(rx, [b for fr in mosi for b in fr])
        self.assertEqual(cpu.rx_overrun, 0)
        self.assertEqual(end_tog, 3)
        self.assertLessEqual(m.max_hiz_latency, 6)

    def test_mode3_default_1mhz_variable_length(self):
        # Mode 3 (CPOL=1, CPHA=1) at default 1.0 MHz (H=25): variable-length incl. 0-byte CS pulse
        mosi = [[0x0F, 0xF0, 0xA5, 0x5A], [], [0xC3, 0x3C]]
        miso = [[0x12, 0x34, 0x56, 0x78], [], [0x9A, 0xBC]]
        host_tx = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
        cpu, rx, m, end_tog = run_spi_slave(
            mosi, host_tx, cpol=1, cpha=1, half_period=25, miso_setup_min=6, miso_hold_min=5,
        )
        self.assertEqual(m.violations, [])
        self.assertEqual(m.rx_frames, miso)
        self.assertEqual(rx, [b for fr in mosi for b in fr])
        self.assertEqual(cpu.rx_overrun, 0)
        self.assertEqual(end_tog, 3)
        self.assertLessEqual(m.max_hiz_latency, 17)

    def test_cpha0_ceiling_2_08mhz_and_jitter(self):
        # Modes 0 & 2 at documented zero-pad-delay ceiling H=12 (2.083 MHz), t_lead=8 (160 ns),
        # t_lag=0 (tests T_T7+12 Hi-Z), plus H=14 with random jitter 0..6
        for cpol in (0, 1):
            mosi = [[0xA5, 0x5A, 0x00, 0xFF]]
            miso = [[0xC3, 0x3C, 0x81, 0x7E]]
            host_tx = [4, 0xC3, 0x3C, 0x81, 0x7E]
            _, rx, m, end_tog = run_spi_slave(
                mosi, host_tx, cpol=cpol, cpha=0, half_period=12,
                t_lead=8, t_lag=0, t_cs_hi=36, miso_setup_min=2, miso_hold_min=2,
            )
            self.assertEqual(m.violations, [], f"cpol={cpol}")
            self.assertEqual(m.rx_frames, miso, f"cpol={cpol}")
            self.assertEqual(rx, mosi[0], f"cpol={cpol}")
            self.assertEqual(end_tog, 1, f"cpol={cpol}")

            # With jitter at H=14
            _, rx_j, m_j, _ = run_spi_slave(
                mosi, host_tx, cpol=cpol, cpha=0, half_period=14,
                t_lead=10, t_lag=10, jitter=6, seed=17 + cpol, miso_setup_min=4,
            )
            self.assertEqual(m_j.violations, [], f"jitter cpol={cpol}")
            self.assertEqual(m_j.rx_frames, miso, f"jitter cpol={cpol}")
            self.assertEqual(rx_j, mosi[0], f"jitter cpol={cpol}")

    def test_cpha1_ceiling_1_25mhz_all_9_poll_phases(self):
        # Modes 1 & 3 at documented ceiling H=20 (1.250 MHz, 60 ns setup) sweeping all 9 phases
        # of the 9-cycle CS_N/SCK poll loop, plus t_lag=0 Hi-Z check
        for cpol in (0, 1):
            for phase in range(9):
                mosi = [[0xAA, 0x55, 0x81]]
                miso = [[0x55, 0xAA, 0x7E]]
                _, rx, m, end_tog = run_spi_slave(
                    mosi, miso[0], cpol=cpol, cpha=1, half_period=20,
                    t_lead=6 + phase, t_lag=0, t_cs_hi=40,
                    miso_setup_min=3, miso_hold_min=2,
                )
                self.assertEqual(m.violations, [], f"cpol={cpol} phase={phase}")
                self.assertEqual(m.rx_frames, miso, f"cpol={cpol} phase={phase}")
                self.assertEqual(rx, mosi[0], f"cpol={cpol} phase={phase}")
                self.assertEqual(end_tog, 1, f"cpol={cpol} phase={phase}")

    def test_rx_fifo_overflow_documented_behaviour(self):
        # 10-byte Mode 3 frame into 8-deep RX FIFO: first 8 RX kept, rx_overrun == 2,
        # all 10 TX bytes still returned on MISO (TX topped up mid-frame)
        mosi = [list(range(0x10, 0x1A))]
        miso = [list(range(0xA0, 0xAA))]
        host_tx = miso[0][:8] + [(1800, miso[0][8]), (2200, miso[0][9])]
        cpu, rx, m, _ = run_spi_slave(
            mosi, host_tx, cpol=1, cpha=1, half_period=25, miso_setup_min=6,
        )
        self.assertEqual(m.violations, [])
        self.assertEqual(m.rx_frames, miso)
        self.assertEqual(rx, mosi[0][:8])
        self.assertEqual(cpu.rx_overrun, 2)

    def test_negative_controls_timing_checker_catches_violations(self):
        # 1) Mode 0 with t_lead=4 (< 6 cyc t_a floor): MISO still floating at 1st rising edge;
        #    and t_lead=5 gives 0-cycle setup violation
        _, _, m_lead4, _ = run_spi_slave(
            [[0xAA]], [1, 0x55], cpol=0, cpha=0, half_period=25, t_lead=4, miso_setup_min=1,
        )
        self.assertTrue(any("floating" in v for v in m_lead4.violations), m_lead4.violations)

        _, _, m_lead5, _ = run_spi_slave(
            [[0xAA]], [1, 0x55], cpol=0, cpha=0, half_period=25, t_lead=5, miso_setup_min=1,
        )
        self.assertTrue(any("MISO setup=0" in v for v in m_lead5.violations), m_lead5.violations)

        # 2) Mode 0 below H=12 ceiling (H=10): MUST fail data or timing
        _, rx_fast0, m_fast0, _ = run_spi_slave(
            [[0xA5, 0x5A, 0xC3]], [3, 0x12, 0x34, 0x56],
            cpol=0, cpha=0, half_period=10, t_lead=10, miso_setup_min=2,
        )
        self.assertTrue(
            bool(m_fast0.violations) or m_fast0.rx_frames != [[0x12, 0x34, 0x56]] or rx_fast0 != [0xA5, 0x5A, 0xC3],
            "H=10 in CPHA=0 should fail",
        )

        # 3) Mode 3 below H=17 hard floor (H=15): MUST fail timing or data across poll phases
        failed_fast3 = False
        for phase in range(9):
            _, rx_f, m_f, _ = run_spi_slave(
                [[0xA5, 0x5A]], [0x81, 0x7E],
                cpol=1, cpha=1, half_period=15, t_lead=6 + phase, miso_setup_min=1,
            )
            if m_f.violations or m_f.rx_frames != [[0x81, 0x7E]] or rx_f != [0xA5, 0x5A]:
                failed_fast3 = True
                break
        self.assertTrue(failed_fast3, "H=15 in CPHA=1 should fail")

        # 4) Mode 3 at H=20 demanding miso_setup_min=6 (> 3 cyc worst-case setup) MUST flag setup violation
        caught_su = False
        for phase in range(9):
            _, _, m_su, _ = run_spi_slave(
                [[0xAA]], [0x55], cpol=1, cpha=1, half_period=20, t_lead=6 + phase, miso_setup_min=6,
            )
            if any("MISO setup" in v for v in m_su.violations):
                caught_su = True
                break
        self.assertTrue(caught_su, "miso_setup_min=6 at H=20 CPHA=1 should trigger setup violation")

        # 5) Mode 3 polled build demanding t_dis_max=10 (< 17 cyc worst-case Hi-Z) MUST flag slow Hi-Z
        caught_hiz = False
        for phase in range(9):
            _, _, m_hz, _ = run_spi_slave(
                [[0xAA]], [0x55], cpol=1, cpha=1, half_period=25,
                t_lead=10, t_lag=25 + phase, t_dis_max=10,
            )
            if any("still driven" in v for v in m_hz.violations):
                caught_hiz = True
                break
        self.assertTrue(caught_hiz, "t_dis_max=10 in CPHA=1 polled build should trigger Hi-Z violation")

    def test_cross_spi_master_vs_spi_slave_all_4_modes(self):
        # Two-ISS cross-test (spi_master.asm <-> spi_slave.asm) at 1.0 MHz (H=25) across all 4 modes.
        # Note: spi_master.asm asserts CS_N 11 cycles before first bit (t_lead=11 >= 6 CPHA=0, >= 3 CPHA=1).
        for cpol in (0, 1):
            for cpha in (0, 1):
                m_frames = [[0xA5, 0x0F, 0xF0], [0x81, 0x7E]]
                s_frames = [[0xC3, 0x5A, 0x01], [0xDE, 0xAD]]
                res = run_spi_cross(cpol, cpha, m_frames, s_frames, half_period=25, max_cycles=25000)
                self.assertEqual(res["contentions"], [], f"mode ({cpol},{cpha})")
                self.assertFalse(res["master_illegal"], f"mode ({cpol},{cpha})")
                self.assertFalse(res["slave_illegal"], f"mode ({cpol},{cpha})")
                self.assertEqual(res["master_rx"], [b for fr in s_frames for b in fr], f"mode ({cpol},{cpha})")
                self.assertEqual(res["slave_rx"], [b for fr in m_frames for b in fr], f"mode ({cpol},{cpha})")


if __name__ == "__main__":
    unittest.main()
