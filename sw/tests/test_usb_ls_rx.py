"""
Phase D: low-speed USB receive on the cycle-accurate ISS, using the REFERENCE MODEL of the proposed
ttpe_usb_rx primitive (sw/usb_ls.py).  The primitive does not exist in RTL yet - these tests validate
the model, the software loop (examples/usb_rx.asm) and the cycle budget, not any hardware.

Run:  python3 -m unittest discover -s sw/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import ttpe_asm as asm      # noqa: E402
import ttpe_iss as iss      # noqa: E402
import usb_ls as U          # noqa: E402

EX_DIR = os.path.join(os.path.dirname(__file__), "..", "examples")
WAIT_PCS, WAIT_D = (1, 7), 2

PKTS = [U.token("SETUP", 0, 0),
        U.data_packet("DATA0", [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]),
        [U.pid_byte(U.PID["ACK"])],
        U.data_packet("DATA1", [0xFF] * 8),          # worst case for bit stuffing
        U.token("IN", 0x7F, 0xF)]


def program(src=None):
    if src is None:
        with open(os.path.join(EX_DIR, "usb_rx.asm")) as f:
            src = f.read()
    return asm.assemble(src).image()


class UsbWorld:
    """D+ = uio6, D- = uio7 on the pads; ui4..ui7 = TICK/DATA/STOP/ERR of the primitive model.
    The primitive output is visible to the program in cycle j, i.e. it is applied to the pad (ui)
    side two cycles earlier so that it comes out of the engine's 2-flop synchroniser in cycle j."""

    def __init__(self, dp, dm):
        self.dp, self.dm = dp, dm
        self.y = U.usb_rx_model(U.sync2(dp, 0), U.sync2(dm, 1))
        self.n = len(dp)

    def pads(self, k, oe, out, uo):
        if k < self.n:
            uio = 0x3F | (self.dp[k] << 6) | (self.dm[k] << 7)
        else:
            uio = 0xBF
        j = k + 2
        ui = 0
        if j < self.n:
            t, d, s, e = (self.y[i][j] for i in range(4))
            ui = (t << 4) | (d << 5) | (s << 6) | (e << 7)
        return uio, ui


def run_usb(dp, dm, words=None, extra=300):
    t = iss.Ttpe(words or program(), rx_depth=10 ** 6)
    w = UsbWorld(dp, dm)
    t.run_cycles(w, len(dp) + extra)
    return t, w


def flags(t, bit):
    """cycles at which output pin uo[bit] changed"""
    out, prev = [], 0
    for eff, _oe, _o, uo in t.pin_events:
        b = (uo >> bit) & 1
        if b != prev:
            out.append(eff)
            prev = b
    return out


def min_wait_slack(t):
    """min over WAIT commits of (commit cycle - first cycle the WAIT is armed). >= 0 means no tick was missed."""
    best = None
    for (cc, pc, _w), (ac, apc) in zip(t.commits, t.accepts):
        assert pc == apc
        if pc in WAIT_PCS:
            s = cc - (ac + WAIT_D + 1)
            best = s if best is None else min(best, s)
    return best


def min_data_tick_gap(w):
    tk = [k for k, v in enumerate(w.y[0]) if v]
    return min(b - a for a, b in zip(tk, tk[1:]) if b - a < 100)    # gaps inside packets


class ModelOnly(unittest.TestCase):
    def test_vectors(self):
        self.assertEqual(U.token("SETUP", 0, 0), [0x2D, 0x00, 0x10])      # well-known SETUP addr0/ep0
        self.assertEqual(U.token("SETUP", 1, 0), [0x2D, 0x01, 0xE8])

    def test_crc16_residual(self):
        for payload in ([], [1, 2, 3], list(range(40))):
            pk = U.data_packet("DATA0", payload)
            crc = 0xFFFF
            for byte in pk[1:]:
                for i in range(8):
                    b = (byte >> i) & 1
                    crc = (crc >> 1) ^ 0xA001 if (crc ^ b) & 1 else crc >> 1
            self.assertEqual(crc, 0xB001)

    def test_decode_sweep(self):
        exp = [U.bits_of(p) for p in PKTS]
        for ppm in (-30000, -15000, 0, 15000, 30000):
            for jit in (0, 2):
                for skew in (0, 3):
                    for seed in range(3):
                        dp, dm = U.waveform(PKTS, ppm=ppm, jitter=jit, skew=skew, seed=seed)
                        r = U.packets_from_ticks(*U.usb_rx_model(U.sync2(dp, 0), U.sync2(dm, 1)))
                        self.assertEqual([b for b, _ in r], exp, (ppm, jit, skew, seed))
                        self.assertFalse(any(f for _, f in r))

    def test_missing_eop_is_reported_as_error(self):
        # last data cell followed by idle J (no SE0): a run of "1"s -> stuff violation -> stop+err
        lv, _ = U.encode_levels([0x2D, 0x00, 0x10])
        dp, dm = [0] * 300, [1] * 300
        for l in lv:
            dp += [l] * 33
            dm += [1 - l] * 33
        dp += [0] * 600
        dm += [1] * 600
        r = U.packets_from_ticks(*U.usb_rx_model(U.sync2(dp, 0), U.sync2(dm, 1)))
        self.assertEqual(len(r), 1)
        self.assertTrue(r[0][1])
        self.assertEqual(r[0][0][:24], U.bits_of([0x2D, 0x00, 0x10]))

    def test_short_se0_is_not_decoded_as_eop(self):
        dp, dm = U.waveform(PKTS[:1], eop_cycles=10)
        r = U.packets_from_ticks(*U.usb_rx_model(U.sync2(dp, 0), U.sync2(dm, 1)))
        self.assertNotEqual([b for b, _ in r], [U.bits_of(PKTS[0])])

    def test_stuffing_actually_exercised(self):
        _, st = U.encode_levels(PKTS[3])
        self.assertGreaterEqual(len(st), 10)


class OnEngine(unittest.TestCase):
    def check(self, ppm=0, jit=0, skew=0, seed=1):
        dp, dm = U.waveform(PKTS, ppm=ppm, jitter=jit, skew=skew, seed=seed)
        t, w = run_usb(dp, dm)
        exp = [b for p in PKTS for b in p]
        self.assertEqual([b for _, b in t.rx], exp, (ppm, jit, skew, seed))
        self.assertEqual(t.rx_overrun, 0)
        self.assertEqual(len(flags(t, 6)), len(PKTS))        # one packet-end toggle per packet
        self.assertEqual(flags(t, 5), [])                    # none flagged bad
        self.assertFalse(t.illegal)
        self.assertGreaterEqual(min_wait_slack(t), min_data_tick_gap(w) - 18)   # byte path: WAIT armed 18 after TICK
        self.assertGreaterEqual(min_wait_slack(t), 0)
        return t

    def test_bytes_match_clean(self):
        t = self.check()
        self.assertGreaterEqual(min_wait_slack(t), 14)      # ideal clock: tick gap >= 32

    def test_rate_error_jitter_skew(self):
        for ppm in (-15000, 15000):
            for jit, skew in ((0, 0), (1, 2), (2, 3)):
                for seed in range(3):
                    self.check(ppm, jit, skew, seed)

    def test_packet_end_flags(self):
        dp, dm = U.waveform(PKTS)
        t, _ = run_usb(dp, dm)
        self.assertEqual(len(flags(t, 6)), len(PKTS))        # uo6 toggles once per packet
        self.assertEqual(flags(t, 5), [])                    # no error flag on good traffic

    def test_stuff_error_sets_flag_and_recovers(self):
        lv, st = U.encode_levels([0xFF, 0x00])
        lv[st[0]] = lv[st[0] - 1]                            # remove the stuffed transition
        dp, dm = [0] * 300, [1] * 300
        for l in lv:
            dp += [l] * 33
            dm += [1 - l] * 33
        dp += [0] * 100
        dm += [0] * 50 + [1] * 50
        good = U.waveform([PKTS[0]], lead=300)
        dp += good[0]
        dm += good[1]
        t, _ = run_usb(dp, dm)
        self.assertEqual(len(flags(t, 5)), 1)                # bad packet flagged
        self.assertEqual([b for _, b in t.rx][-3:], PKTS[0])  # next packet decodes

    def test_partial_byte_flagged(self):
        # 3 good bytes then 5 extra bits then EOP -> aligned = false -> error flag
        pk = [0x2D, 0x00, 0x10, 0x15]
        lv, _ = U.encode_levels(pk)
        lv = lv[:-3]                                        # drop the last 3 bit cells
        dp, dm = [0] * 300, [1] * 300
        for l in lv:
            dp += [l] * 33
            dm += [1 - l] * 33
        dp += [0] * 100
        dm += [0] * 50 + [1] * 50
        dp += [0] * 300
        dm += [1] * 300
        t, _ = run_usb(dp, dm)
        self.assertEqual(len(flags(t, 5)), 1)
        self.assertEqual([b for _, b in t.rx], pk[:3])

    def test_negative_control_slow_loop_loses_data(self):
        with open(os.path.join(EX_DIR, "usb_rx.asm")) as f:
            src = f.read().replace("JHI ui6, pkt_end @2", "JHI ui6, pkt_end @30")
        dp, dm = U.waveform(PKTS)
        t, _ = run_usb(dp, dm, words=program(src))
        exp = [b for p in PKTS for b in p]
        self.assertNotEqual([b for _, b in t.rx], exp)       # the test has teeth


class SoftwareOnlyCapture(unittest.TestCase):
    """What the EXISTING engine can do without new hardware: fixed-phase raw capture + host decode."""

    def capture(self, pkt, ppm, jit=0, seed=1):
        dp, dm = U.waveform([pkt], ppm=ppm, jitter=jit, seed=seed)
        t = iss.Ttpe(asm.assemble(U.gen_raw_capture_asm()).image(), rx_depth=10 ** 6)
        t.run_cycles(UsbPadsOnly(dp, dm), len(dp) + 100)
        bits = U.decode_raw_capture([b for _, b in t.rx])
        exp = U.bits_of(pkt)
        return bits is not None and bits[:len(exp)] == exp

    def test_handshake_and_token_at_plus_minus_1p5_percent(self):
        for pkt in ([0xD2], U.token("SETUP", 0, 0)):
            for ppm in (-15000, 0, 15000):
                self.assertTrue(self.capture(pkt, ppm), (pkt, ppm))

    def test_data_packet_ok_only_for_small_rate_error(self):
        pkt = PKTS[1]
        for ppm in (-5000, 0, 5000):
            self.assertTrue(self.capture(pkt, ppm), ppm)
        for ppm in (-15000, 15000):                      # documented limitation: phase drifts out of the cell
            self.assertFalse(self.capture(pkt, ppm), ppm)


class IssIntegration(unittest.TestCase):
    """ISS with the primitive wired into the synchroniser (Ttpe.usb_en, the model of the RTL integration)
    must behave exactly like the external-injection harness used above."""

    def run_integrated(self, dp, dm):
        t = iss.Ttpe(program(), rx_depth=10 ** 6)
        t.usb_en = True
        t.run_cycles(UsbPadsOnly(dp, dm), len(dp) + 300)
        return t

    def test_same_commits_pins_and_bytes(self):
        for ppm, jit, skew, seed in ((0, 0, 0, 1), (-15000, 1, 2, 2), (15000, 2, 3, 3)):
            dp, dm = U.waveform(PKTS, ppm=ppm, jitter=jit, skew=skew, seed=seed)
            a, _ = run_usb(dp, dm)
            b = self.run_integrated(dp, dm)
            self.assertEqual(a.rx, b.rx)
            self.assertEqual(a.commits, b.commits)
            self.assertEqual(a.pin_events, b.pin_events)
            self.assertEqual([x for _, x in b.rx], [x for p in PKTS for x in p])

    def test_disabled_is_transparent(self):
        # usb_en = False: the physical ui_in[7:4] reach the program unchanged
        src = "WAIT rise ui4 @2\nPINS toggle uo1 @2\nWAIT fall ui6 @2\nPINS toggle uo2 @2\nHALT"
        prog = asm.assemble(src).image()

        class W:
            def pads(self, k, oe, out, uo):
                return 0xFF, (0x10 if 20 <= k < 40 else 0) | (0x40 if k < 60 else 0)

        t = iss.Ttpe(prog)
        t.run(W(), 400)
        self.assertEqual([e[3] & 6 for e in t.pin_events][-2:], [2, 6])


class UsbPadsOnly:
    def __init__(self, dp, dm):
        self.dp, self.dm = dp, dm

    def pads(self, k, oe, out, uo):
        return ((0x3F | (self.dp[k] << 6) | (self.dm[k] << 7)) if k < len(self.dp) else 0xBF), 0


if __name__ == "__main__":
    unittest.main()
