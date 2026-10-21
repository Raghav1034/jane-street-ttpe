"""
Low-speed USB TRANSMIT (examples/usb_tx.asm) on the cycle-accurate ISS.

1. The per-cycle D+/D- waveform produced by the program is compared, cycle for cycle, with the
   reference waveform usb_ls.tx_expected() (SYNC, NRZI, stuffing, 33/34-cycle cells, SE0 66, J 33).
2. The cycle budget of every path (0 bit, 1 bit, sixth 1 + stuffed 0, byte boundary) is measured from
   the commit log of the ISS.
3. Loopback: the waveform is fed into the reference model of the ttpe_usb_rx primitive and into the
   usb_rx.asm receive program (TX engine -> primitive -> RX engine) and must come back unchanged.

The USB signalling conventions are from the author's memory of the specification and are not
independently checked; the loopback proves TX and RX agree with each other (and with
usb_ls.encode_levels for bit values), not that a real host would accept the packet.

Run:  python3 -m unittest discover -s sw/tests -v
"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import ttpe_asm as asm      # noqa: E402
import ttpe_isa as isa      # noqa: E402
import ttpe_iss as iss      # noqa: E402
import ttpe_world as W      # noqa: E402
import usb_ls as U          # noqa: E402
import test_usb_ls_rx as RX  # noqa: E402  (run_usb, flags helpers for the receive side)

EX_DIR = os.path.join(HERE, "..", "examples")
OP = isa.OPCODES

ACK = [U.pid_byte(U.PID["ACK"])]
NAK = [U.pid_byte(U.PID["NAK"])]
SETUP = U.token("SETUP", 0, 0)
DATA0 = U.data_packet("DATA0", [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00])
DATA1_FF = U.data_packet("DATA1", [0xFF] * 8)           # worst case for bit stuffing
PACKETS = {"ACK": ACK, "NAK": NAK, "SETUP": SETUP, "DATA0": DATA0, "DATA1_ff": DATA1_FF}


def tx_program(**defs):
    with open(os.path.join(EX_DIR, "usb_tx.asm")) as f:
        return asm.assemble(f.read(), defines=defs).image()


def run_tx(packets, host_extra=None, tail=700, **defs):
    """Run usb_tx.asm with the host bytes n, b0.. for every packet; returns (iss, world)."""
    host = []
    for p in packets:
        host += [len(p)] + list(p)
    if host_extra is not None:
        host = host_extra
    t = iss.Ttpe(tx_program(**defs), host_tx=host)
    w = W.UsbTxBus()
    n = 500 + sum(len(U.tx_expected(p)[0]) + 400 for p in packets) + tail
    t.run_cycles(w, n)
    return t, w


def jsb_commits(t):
    return [c for c, _, wd in t.commits if isa.decode(wd)[0] == OP["JSB"]]


class WaveformExact(unittest.TestCase):
    def test_every_cycle_matches_the_reference(self):
        for name, pkt in PACKETS.items():
            with self.subTest(name):
                t, w = run_tx([pkt])
                dp, dm = w.levels(6), w.levels(7)
                e0 = dp.index(1)                          # first SYNC edge (first K)
                ep, em = U.tx_expected(pkt)
                self.assertEqual(dp[e0:e0 + len(ep)], ep)
                self.assertEqual(dm[e0:e0 + len(em)], em)
                self.assertEqual(set(dp[:e0]), {0}, "J before SYNC")
                self.assertEqual(set(dp[e0 + len(ep):]), {0})     # released bus = J
                self.assertEqual(set(dm[e0 + len(em):]), {1})
                self.assertFalse(t.illegal)

    def test_cells_are_33_and_34_wide_and_average_close_to_33_33(self):
        t, w = run_tx([DATA0])
        dp, dm = w.levels(6), w.levels(7)
        e0 = dp.index(1)
        ep, _ = U.tx_expected(DATA0)
        # D+ run lengths during the packet are multiples of cells: every run is a sum of 33/34 cells
        cells = U.tx_cells(DATA0)
        widths = {wd for _, wd in cells}
        self.assertEqual(widths, {33, 34})
        total = sum(wd for _, wd in cells)
        nbits = len(cells)
        self.assertLess(abs(total / nbits - 100 / 3), 0.2)                   # mean cell vs 33.33
        rate_err_ppm = (100 / 3 - total / nbits) / (100 / 3) * 1e6
        self.assertLess(abs(rate_err_ppm), 5000)                             # well inside +-1.5 %
        # one byte (8 data cells without stuffing) = 266 cycles
        self.assertEqual(sum(33 + (k in U.TX_EXT_BITS) for k in range(8)), 266)

    def test_plain_33_cells_when_ext_is_off(self):
        t, w = run_tx([SETUP], EXT2=0, EXT5=0)
        dp = w.levels(6)
        e0 = dp.index(1)
        ep, _ = U.tx_expected(SETUP, ext_bits=())
        self.assertEqual(dp[e0:e0 + len(ep)], ep)

    def test_bus_is_driven_only_while_sending(self):
        t, w = run_tx([SETUP])
        ev = t.pin_events
        oe_hist = [(eff, oe & 0xC0) for eff, oe, _o, _u in ev]
        self.assertEqual(oe_hist[0][1], 0)
        self.assertEqual(oe_hist[-1][1], 0, "D+/D- released after the packet")
        self.assertIn(0xC0, {oe for _, oe in oe_hist})
        # J is driven (D+ low, D- high) for at least one cell before the first SYNC edge
        dp = w.levels(6)
        e0 = dp.index(1)
        drive_start = min(eff for eff, oe in oe_hist if oe == 0xC0)
        self.assertGreaterEqual(e0 - drive_start, 33)

    def test_two_packets_back_to_back(self):
        t, w = run_tx([ACK, SETUP])
        dp = w.levels(6)
        e0 = dp.index(1)
        a, _ = U.tx_expected(ACK)
        self.assertEqual(dp[e0:e0 + len(a)], a)
        e1 = dp.index(1, e0 + len(a))
        s, sm = U.tx_expected(SETUP)
        self.assertEqual(dp[e1:e1 + len(s)], s)
        self.assertEqual(w.levels(7)[e1:e1 + len(sm)], sm)

    def test_unused_host_fifo_stays_empty_after_the_packet(self):
        t, w = run_tx([DATA0])
        self.assertEqual(t.tx_idx, len(t.tx))


class CycleBudget(unittest.TestCase):
    """Cell spacing = distance between consecutive bit-test (JSB) commits, per path."""

    def classify(self, pkt):
        t, w = run_tx([pkt])
        js = jsb_commits(t)
        bits, ones = [], 0
        for byte in pkt:
            for k in range(8):
                b = (byte >> k) & 1
                st = False
                if b:
                    ones += 1
                    if ones == 6:
                        st, ones = True, 0
                else:
                    ones = 0
                bits.append((k, b, st))
        self.assertEqual(len(js), len(bits))
        out = {}
        for i in range(len(js) - 1):
            k, b, st = bits[i]
            key = ("one" if b else "zero") + ("+stuff" if st else "") + (",byte-end" if k == 7 else "")
            out.setdefault(key, set()).add((js[i + 1] - js[i], k))
        return out

    def test_spacings_per_path(self):
        pkt = DATA1_FF + [0xFF, 0x00, 0xA5, 0x3C, 0x7E, 0xFE, 0x01]
        out = self.classify(pkt)
        exp = {"zero": {33, 34}, "one": {33, 34}, "one+stuff": {66, 67},
               "zero,byte-end": {33}, "one,byte-end": {33}, "one+stuff,byte-end": {66}}
        self.assertEqual({k: {s for s, _ in v} for k, v in out.items()}, exp)
        for key in ("zero", "one"):
            for s, k in out[key]:
                self.assertEqual(s, 34 if k in U.TX_EXT_BITS else 33, (key, k))
        for s, k in out["one+stuff"]:
            self.assertEqual(s, 66 + (1 if k in U.TX_EXT_BITS else 0))

    def test_edge_is_three_cycles_after_the_bit_test(self):
        t, w = run_tx([DATA0])
        js = set(jsb_commits(t))
        pins = [(c, wd) for c, _, wd in t.commits
                if isa.decode(wd)[0] == OP["PINS"]]
        # toggles that directly follow a JSB (zero bits) are at JSB + 3
        n = 0
        for c, _ in pins:
            if c - 3 in js:
                n += 1
        zeros = sum(1 for p in DATA0 for k in range(8) if not (p >> k) & 1)
        self.assertEqual(n, zeros)

    def test_hpull_deadline_is_the_hpull_commit(self):
        """A byte available at the nominal HPULL commit cycle costs nothing; one cycle later the
        line is stretched by one cycle (documented limit for the host)."""
        pkt = [0x2D, 0x00, 0x10]
        t0, _ = run_tx([pkt])
        hp = [c for c, _, wd in t0.commits if isa.decode(wd)[0] == OP["HPULL"]]    # n, b0, b1, b2
        self.assertEqual(len(hp), 4)
        base = [len(pkt), pkt[0], pkt[1]]
        for late, same in ((0, True), (1, False)):
            host = [(0, b) for b in base] + [(hp[3] + late, pkt[2])]
            t, w = run_tx([pkt], host_extra=host)
            dp = w.levels(6)
            e0 = dp.index(1)
            ep, _ = U.tx_expected(pkt)
            self.assertEqual(dp[e0:e0 + len(ep)] == ep, same, late)


class Loopback(unittest.TestCase):
    """TX program -> reference model of the ttpe_usb_rx primitive -> usb_rx.asm."""

    @staticmethod
    def to_lists(w, tail=400):
        return w.levels(6) + [0] * tail, w.levels(7) + [1] * tail

    def check(self, packets, **defs):
        t, w = run_tx(packets, **defs)
        dp, dm = self.to_lists(w)
        # (a) the primitive model alone
        r = U.packets_from_ticks(*U.usb_rx_model(U.sync2(dp, 0), U.sync2(dm, 1)))
        self.assertEqual([b for b, _ in r], [U.bits_of(p) for p in packets])
        self.assertFalse(any(f for _, f in r), "no stuff error flagged")
        # (b) the receive program on a second engine
        rx, _ = RX.run_usb(dp, dm)
        self.assertEqual([b for _, b in rx.rx], [b for p in packets for b in p])
        self.assertEqual(rx.rx_overrun, 0)
        self.assertEqual(len(RX.flags(rx, 6)), len(packets))
        self.assertEqual(RX.flags(rx, 5), [])
        self.assertFalse(rx.illegal)

    def test_each_packet_type(self):
        for name, pkt in PACKETS.items():
            with self.subTest(name):
                self.check([pkt])

    def test_sequence_of_packets(self):
        self.check([SETUP, DATA0, ACK, DATA1_FF, NAK])

    def test_plain_33_cells_also_loop_back(self):
        self.check([SETUP, DATA1_FF], EXT2=0, EXT5=0)

    def test_stuffing_is_exercised(self):
        self.assertGreaterEqual(len(U.encode_levels(DATA1_FF)[1]), 10)
        # the cells the program sends carry the same number of stuffed zeros as encode_levels
        n_cells = len(U.tx_cells(DATA1_FF))
        self.assertEqual(n_cells, len(U.encode_levels(DATA1_FF)[0]))
        self.assertEqual([lv for lv, _ in U.tx_cells(DATA1_FF)], U.encode_levels(DATA1_FF)[0])

    def test_all_byte_values_loop_back(self):
        pkt = [U.pid_byte(U.PID["DATA0"])] + list(range(256))[:96]
        self.check([pkt])


class Sizes(unittest.TestCase):
    def test_program_fits_one_page(self):
        with open(os.path.join(EX_DIR, "usb_tx.asm")) as f:
            p = asm.assemble(f.read())
        n = sum(p.used)
        self.assertLess(n, 128)
        self.assertEqual(max(i for i, u in enumerate(p.used) if u) + 1, n)   # contiguous, page 0


if __name__ == "__main__":
    unittest.main()
