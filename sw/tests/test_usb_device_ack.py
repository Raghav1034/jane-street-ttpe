"""
Autonomous low-speed USB device handshake (examples/usb_device_ack.asm) on the cycle-accurate ISS with the
ttpe_usb_rx primitive wired in (Ttpe.usb_en) and a CLOSED-LOOP bus: the scripted host drives D+/D-, the chip
drives them while it transmits, and the slicer sees the chip's own transmission through the pads.

Checked: every host byte reaches the RX FIFO; DATA0/DATA1 are answered with ACK and IN with NAK, other
packets and bad packets with nothing; the response waveform equals usb_ls.tx_expected cycle for cycle;
turnaround (end of host SE0 -> first K edge of the response) is 140 cycles on every path and for every SE0
width of 62..75 cycles; the next packet is received correctly right after the response.

TURNAROUND WINDOW (relayed, NOT verified against the specification text by us): 2 .. 6.5 bit times at the
device = 67 .. 216 cycles at 50 MHz.  The reference point for "end of EOP" is ambiguous (end of SE0 or end of
the 1-bit J that follows), so both readings are asserted: T and T - 33 must lie inside the window.
The program is a turnaround demonstrator: it does not check CRC, address, endpoint or data toggle.

Run:  python3 -m unittest discover -s sw/tests -v
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import ttpe_asm as asm      # noqa: E402
import ttpe_iss as iss      # noqa: E402
import ttpe_world as W      # noqa: E402
import usb_ls as U          # noqa: E402
import test_usb_ls_rx as RX  # noqa: E402  (wave_from_levels, flags)

EX = os.path.join(HERE, "..", "examples", "usb_device_ack.asm")
ACK, NAK = [0xD2], [0x5A]
WINDOW = (67, 216)                         # 2 .. 6.5 bit times in cycles (relayed)
NOMINAL = 140


def program(**defs):
    with open(EX) as f:
        return asm.assemble(f.read(), defines=defs).image()


def expected_response(pkt):
    """What the program must answer for a clean host packet (by PID only)."""
    return {0xC3: ACK, 0x4B: ACK, 0x69: NAK}.get(pkt[0])


def run(dp, dm, extra=400, **defs):
    t = iss.Ttpe(program(**defs), rx_depth=10 ** 6)
    t.usb_en = True
    w = W.UsbHostDevBus(dp, dm)
    t.run_cycles(w, len(dp) + extra)
    return t, w


def host_waveform(packets, eop=66, gap=None, ppm=0, jitter=0, skew=0, seed=1):
    gap = gap or 1000
    return U.waveform(packets, eop_cycles=eop, gap_cycles=gap, ppm=ppm, jitter=jitter, skew=skew, seed=seed)


def host_packet_ends(dp, dm):
    """[(se0_start, j_return)] for every host SE0."""
    out = []
    for k in range(1, len(dp)):
        if dp[k] == 0 and dm[k] == 0 and not (dp[k - 1] == 0 and dm[k - 1] == 0):
            j = next(i for i in range(k, len(dm)) if dm[i] == 1)
            if j - k >= 40:                      # real EOP; shorter SE0 runs are skew glitches inside a packet
                out.append((k, j))
    return out


def first_k_after(w, j, span=700):
    p = w.levels(6)
    return next((i for i in range(j, min(j + span, len(p))) if p[i] == 1), None)


def check_response(tc, w, j, resp):
    """Response waveform after J-return j: exact match, or silence when resp is None."""
    p, m = w.levels(6), w.levels(7)
    k = first_k_after(w, j)
    if resp is None:
        tc.assertIsNone(k, "unexpected response")
        return None
    tc.assertIsNotNone(k, "no response")
    ep, em = U.tx_expected(resp)
    tc.assertEqual(p[k:k + len(ep)], ep)
    tc.assertEqual(m[k:k + len(em)], em)
    return k - j


PKTS = [U.data_packet("DATA0", [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00]),
        U.token("IN", 3, 1),
        U.token("SETUP", 0, 0),
        U.data_packet("DATA1", [0xFF] * 8),          # worst case bit stuffing
        [U.pid_byte(U.PID["ACK"])],
        U.token("OUT", 5, 2)]


class Handshake(unittest.TestCase):
    def test_sizes(self):
        with open(EX) as f:
            self.assertLess(sum(asm.assemble(f.read()).used), 128)

    def test_answers_and_bytes(self):
        dp, dm = host_waveform(PKTS)
        t, w = run(dp, dm)
        self.assertEqual([b for _, b in t.rx], [b for p in PKTS for b in p])
        self.assertEqual(t.rx_overrun, 0)
        self.assertFalse(t.illegal)
        ends = host_packet_ends(dp, dm)
        self.assertEqual(len(ends), len(PKTS))
        for (se0, j), pkt in zip(ends, PKTS):
            check_response(self, w, j, expected_response(pkt))
        self.assertEqual(len(RX.flags(t, 6)), len(PKTS), "one packet-end toggle per HOST packet, none for ours")
        self.assertEqual(RX.flags(t, 5), [])

    def test_turnaround_is_140_on_every_path_and_inside_the_window(self):
        for eop in range(62, 76):
            dp, dm = host_waveform(PKTS[:4], eop=eop)
            t, w = run(dp, dm)
            ends = host_packet_ends(dp, dm)
            ts = [check_response(self, w, j, expected_response(p)) for (_, j), p in zip(ends, PKTS[:4])]
            ts = [x for x in ts if x is not None]
            self.assertEqual(len(ts), 3)
            for x in ts:
                self.assertEqual(x, NOMINAL, eop)
                self.assertGreaterEqual(x, WINDOW[0])
                self.assertLessEqual(x, WINDOW[1])
                self.assertGreaterEqual(x - 33, WINDOW[0])      # other reading: from the end of the J bit
                self.assertLessEqual(x - 33, WINDOW[1])

    def test_host_rate_error_and_jitter(self):
        for ppm in (-15000, 15000):
            for jit, skew, seed in ((0, 0, 1), (1, 2, 2)):
                dp, dm = host_waveform(PKTS, ppm=ppm, jitter=jit, skew=skew, seed=seed)
                t, w = run(dp, dm)
                self.assertEqual([b for _, b in t.rx], [b for p in PKTS for b in p], (ppm, jit, skew))
                for (_, j), pkt in zip(host_packet_ends(dp, dm), PKTS):
                    check_response(self, w, j, expected_response(pkt))

    def test_next_packet_right_after_the_response(self):
        """Host SYNC begins 20 cycles after the chip releases the bus: the program is back in receive."""
        resp_len = len(U.tx_expected(ACK)[0])
        gap = NOMINAL + resp_len + 20
        pk = [PKTS[0], PKTS[3], PKTS[0]]
        dp, dm = host_waveform(pk, gap=gap)
        t, w = run(dp, dm)
        self.assertEqual([b for _, b in t.rx], [b for p in pk for b in p])
        for (_, j), p in zip(host_packet_ends(dp, dm), pk):
            check_response(self, w, j, expected_response(p))

    def test_response_edge_cases_override_constants(self):
        """PAD_* are the only turnaround knobs: shifting one moves that path and nothing else."""
        dp, dm = host_waveform(PKTS[:2])
        t, w = run(dp, dm, PAD_D0=86)
        ts = [check_response(self, w, j, expected_response(p)) for (_, j), p in zip(host_packet_ends(dp, dm), PKTS[:2])]
        self.assertEqual(ts, [NOMINAL + 10, NOMINAL])


class BadPackets(unittest.TestCase):
    @staticmethod
    def levels_of(pkt):
        lv, st = U.encode_levels(pkt)
        return lv, st

    def wave(self, level_lists, eop=66):
        return RX.wave_from_levels(["".join(map(str, lv)) for lv in level_lists], eop=eop, gap=1000)

    def run_mixed(self, bad_levels):
        good = self.levels_of(PKTS[0])[0]
        dp, dm = self.wave([bad_levels, good, bad_levels, good])
        t, w = run(dp, dm)
        ends = host_packet_ends(dp, dm)
        self.assertEqual(len(ends), 4)
        for i, (_, j) in enumerate(ends):
            check_response(self, w, j, None if i % 2 == 0 else ACK)
        self.assertFalse(t.illegal)
        return t

    def test_stuff_violation_is_flagged_and_not_acked(self):
        lv, st = self.levels_of(U.data_packet("DATA0", [0xFF] * 8))
        lv = list(lv)
        lv[st[0]] = lv[st[0] - 1]                       # remove the stuffed transition
        t = self.run_mixed(lv)
        self.assertEqual(len(RX.flags(t, 5)), 2)        # two bad packets flagged on uo5 (toggle count)

    def test_stuff_violation_exactly_at_a_byte_boundary_is_caught_by_the_err_flag(self):
        """0xFC ends in six 1s, so the stuffed 0 follows the byte; with the transition removed the packet
        ends with c1 == 8 (no partial byte): only the ERR input can reject it."""
        lv, st = self.levels_of(U.data_packet("DATA0", [0xFC, 0x01]))
        lv = list(lv)
        self.assertEqual((st[0] - 8) % 8, 0, "stuffed bit sits on a byte boundary")
        lv[st[0]] = lv[st[0] - 1]
        t = self.run_mixed(lv)
        self.assertEqual(len(RX.flags(t, 5)), 2)

    def test_partial_last_byte_is_flagged_and_not_acked(self):
        lv, _ = self.levels_of(PKTS[0])
        t = self.run_mixed(list(lv[:-3]))               # last byte cut after 5 bits
        self.assertEqual(len(RX.flags(t, 5)), 2)

    def test_empty_packet_gets_no_response(self):
        sync_only = [1, 0, 1, 0, 1, 0, 1, 1]
        good = self.levels_of(PKTS[0])[0]
        dp, dm = self.wave([sync_only, good])
        t, w = run(dp, dm)
        ends = host_packet_ends(dp, dm)
        check_response(self, w, ends[0][1], None)
        check_response(self, w, ends[1][1], ACK)


if __name__ == "__main__":
    unittest.main()
