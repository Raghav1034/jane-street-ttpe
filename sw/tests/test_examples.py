"""
Protocol-level tests for sw/examples/*.asm, run on the cycle-accurate ISS against the
device models in ttpe_world.py.

Run:  python3 -m unittest discover -s sw/tests -v

Cycle conventions (see docs/ISA.md):
  * the pad level "during cycle k" is what BusWorld.levels()/edges() report;
  * a PINS/SHOUT commit in cycle C changes the pad in cycle C+1;
  * an input pad change in cycle X is first seen by an instruction committing in X+2.
All timing assertions are integer cycle arithmetic (a 50 MHz clock = 20 ns per cycle).
"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import ttpe_asm as asm      # noqa: E402
import ttpe_isa as isa      # noqa: E402
import ttpe_iss as iss      # noqa: E402
import ttpe_world as W      # noqa: E402

EX_DIR = os.path.join(os.path.dirname(__file__), "..", "examples")
CLK_NS = 20


def load(name, **defs):
    with open(os.path.join(EX_DIR, name)) as f:
        return asm.assemble(f.read(), defines=defs)


def runs(levels):
    """Run-length encoding: [(level, start_cycle, length), ...]."""
    out, s = [], 0
    for k in range(1, len(levels) + 1):
        if k == len(levels) or levels[k] != levels[s]:
            out.append((levels[s], s, k - s))
            s = k
    return out


def sample_commits(t):
    """Commit cycles of every SAMPLE instruction, in order."""
    return [c for c, _, w in t.commits if isa.decode(w)[0] == isa.OPCODES["SAMPLE"]]


# ====================================================================== UART TX
class UartTx(unittest.TestCase):
    def run_tx(self, bit, host_tx, cycles):
        t = iss.Ttpe(load("uart_tx.asm", BIT=bit).image(), host_tx=host_tx)
        w = W.BusWorld()
        t.run_cycles(w, cycles)
        return t, w

    def test_back_to_back_frames_are_bit_exact(self):
        data = [0x55, 0xA5, 0x00, 0xFF, 0x01, 0x80]
        for bit in (434, 100, 50, 20, 16):
            with self.subTest(bit=bit):
                t, w = self.run_tx(bit, data, 10 * bit * len(data) + 200)
                frames = W.decode_uart(w.levels(1), bit)
                self.assertEqual([f[1] for f in frames], data)
                self.assertTrue(all(f[2] for f in frames), "stop bit must be high")
                starts = [f[0] for f in frames]
                # one stop bit: start-to-start spacing is exactly 10 bit times
                self.assertEqual([b - a for a, b in zip(starts, starts[1:])],
                                 [10 * bit] * (len(data) - 1))
                # every line transition lies exactly on a bit boundary of its frame
                for c, _ in w.edges(1):
                    s = max(x for x in starts if x <= c)
                    self.assertEqual((c - s) % bit, 0, "edge at %d off the bit grid" % c)
                self.assertFalse(t.illegal)

    def test_start_bit_latency_from_host_byte(self):
        """HPULL commits when the byte arrives (X); the start bit is on the pad at X+7."""
        for x in (500, 501, 777):
            with self.subTest(x=x):
                t, w = self.run_tx(434, [(x, 0x3C)], x + 10 * 434 + 100)
                frames = W.decode_uart(w.levels(1), 434)
                self.assertEqual([(f[0], f[1]) for f in frames], [(x + 7, 0x3C)])

    def test_idle_line_is_high(self):
        t, w = self.run_tx(434, [], 3000)
        self.assertEqual(set(w.levels(1)), {1})

    def test_gap_between_frames_when_host_is_slow(self):
        bit = 100
        t, w = self.run_tx(bit, [(300, 0xAA), (5000, 0x55)], 7000)
        frames = W.decode_uart(w.levels(1), bit)
        self.assertEqual([(f[0], f[1]) for f in frames], [(307, 0xAA), (5007, 0x55)])

    def test_bit_parameter_floor(self):
        """Programs with a delta below 2 are rejected by the assembler's timing lint."""
        load("uart_tx.asm", BIT=16)
        with self.assertRaises(asm.AsmError):
            load("uart_tx.asm", BIT=8)


# ====================================================================== UART RX
class UartRx(unittest.TestCase):
    def run_rx(self, bit, frames, cycles, src_bit=None, rx_depth=8):
        t = iss.Ttpe(load("uart_rx.asm", BIT=bit).image(), rx_depth=rx_depth)
        w = W.BusWorld([W.UartSource(2, src_bit or bit, frames)])
        t.run_cycles(w, cycles)
        return t

    def test_sampling_instants_are_bit_centres(self):
        bit = 434
        for start in (1000, 1001, 1002, 1233):
            with self.subTest(start=start):
                t = self.run_rx(bit, [(start, 0xA6)], start + 12 * bit)
                self.assertEqual([b for _, b in t.rx], [0xA6])
                got = sample_commits(t)[:8]
                # SAMPLE committing in cycle e reads the pad of cycle e-2.
                want = [start + bit + bit // 2 + k * bit + 2 for k in range(8)]
                self.assertEqual(got, want)

    def test_streams_at_nominal_and_mismatched_baud(self):
        rng = random.Random(1234)
        data = [rng.randrange(256) for _ in range(24)]
        for bit, src in ((434, 434), (434, 425), (434, 443), (100, 100), (100, 98), (100, 102)):
            with self.subTest(rx_bit=bit, src_bit=src):
                frames = [(1000 + i * 10 * src, b) for i, b in enumerate(data)]
                t = self.run_rx(bit, frames, 1000 + (len(data) * 10 + 2) * src,
                                src_bit=src, rx_depth=64)
                self.assertEqual([b for _, b in t.rx], data)

    def test_idle_line_produces_nothing(self):
        t = self.run_rx(434, [], 5000)
        self.assertEqual(t.rx, [])

    def test_overrun_is_counted_and_oldest_bytes_kept(self):
        data = list(range(0x30, 0x30 + 12))
        frames = [(1000 + i * 10 * 434, b) for i, b in enumerate(data)]
        t = self.run_rx(434, frames, 1000 + 13 * 10 * 434, rx_depth=8)
        self.assertEqual([b for _, b in t.rx], data[:8])
        self.assertEqual(t.rx_overrun, 4)


# ====================================================================== SPI master
def spi_cycles(nbytes, h, margin=400):
    return 200 + nbytes * (16 * h + 200) + margin


class SpiMaster(unittest.TestCase):
    def xfer(self, cpol, cpha, h, mosi, miso, host_tx=None):
        t = iss.Ttpe(load("spi_master.asm", CPOL=cpol, CPHA=cpha, H=h).image(),
                     host_tx=host_tx if host_tx is not None else [len(mosi)] + list(mosi))
        slave = W.SpiSlave(cpol, cpha, tx_bytes=miso)
        w = W.BusWorld([slave])
        t.run_cycles(w, spi_cycles(len(mosi), h))
        return t, w, slave

    def test_all_modes_data_integrity(self):
        mosi = [0xA5, 0x0F, 0xF0, 0x00, 0xFF, 0x81]
        miso = [0xC3, 0x5A, 0x01, 0x80, 0x00, 0xFF]
        for cpol in (0, 1):
            for cpha in (0, 1):
                for h in (24, 100):
                    with self.subTest(cpol=cpol, cpha=cpha, h=h):
                        t, w, s = self.xfer(cpol, cpha, h, mosi, miso)
                        self.assertEqual(s.rx, mosi)
                        self.assertEqual([b for _, b in t.rx], miso)
                        self.assertEqual(len(s.cs_cycles), 1)
                        self.assertFalse(t.illegal)

    def test_sck_half_periods_are_exactly_h(self):
        mosi = [0xA5, 0x3C, 0xFF]
        for cpol in (0, 1):
            for cpha in (0, 1):
                for h in (24, 50):
                    with self.subTest(cpol=cpol, cpha=cpha, h=h):
                        t, w, s = self.xfer(cpol, cpha, h, mosi, [0] * 3)
                        cs_fall = [c for c, lv in w.edges(0) if lv == 0][0]
                        e = [c for c, _ in w.edges(3) if c > cs_fall]
                        self.assertEqual(len(e), 16 * len(mosi))
                        for byte in range(len(mosi)):
                            g = e[16 * byte: 16 * byte + 16]
                            self.assertEqual([b - a for a, b in zip(g, g[1:])], [h] * 15)

    def test_minimum_half_period_works_and_below_is_rejected(self):
        mosi = [0x96, 0x69]
        miso = [0x3C, 0xC3]
        for cpha, hmin in ((0, 12), (1, 9)):
            for cpol in (0, 1):
                with self.subTest(cpol=cpol, cpha=cpha, h=hmin):
                    t, w, s = self.xfer(cpol, cpha, hmin, mosi, miso)
                    self.assertEqual(s.rx, mosi)
                    self.assertEqual([b for _, b in t.rx], miso)
                    cs_fall = [c for c, lv in w.edges(0) if lv == 0][0]
                    e = [c for c, _ in w.edges(3) if c > cs_fall]
                    self.assertEqual([b - a for a, b in zip(e[:16], e[1:16])], [hmin] * 15)
            with self.assertRaises(asm.AsmError):
                load("spi_master.asm", CPOL=0, CPHA=cpha, H=hmin - 1)

    def test_two_back_to_back_transfers(self):
        t = iss.Ttpe(load("spi_master.asm", H=24).image(), host_tx=[2, 0x12, 0x34, 1, 0x56])
        s = W.SpiSlave(0, 0, tx_bytes=[0xAA, 0xBB, 0xCC])
        w = W.BusWorld([s])
        t.run_cycles(w, 6000)
        self.assertEqual(s.transfers, [[0x12, 0x34], [0x56]])
        self.assertEqual(len(s.cs_cycles), 2)
        self.assertEqual([b for _, b in t.rx], [0xAA, 0xBB, 0xCC])

    def test_cs_idle_high_and_released_after_transfer(self):
        t, w, s = self.xfer(0, 0, 24, [0x55], [0xAA])
        cs = w.levels(0)
        self.assertEqual(cs[0], 1)
        self.assertEqual(cs[-1], 1)
        self.assertEqual(sum(1 for lv, _, _ in runs(cs) if lv == 0), 1)


# ====================================================================== I2C master
CONFIGS = ((250, 250, "100 kHz standard mode", 4700, 4000),
           (75, 50, "400 kHz fast mode", 1300, 600))


class I2cMaster(unittest.TestCase):
    def run_i2c(self, host_tx, slave, tl, th, cycles):
        t = iss.Ttpe(load("i2c_master.asm", TL=tl, TH=th).image(), host_tx=host_tx)
        w = W.BusWorld([slave])
        t.run_cycles(w, cycles)
        return t, w

    @staticmethod
    def scl_pulses(w):
        r = runs(w.levels(2))
        lows = [l for lv, _, l in r if lv == 0]
        highs = [l for lv, _, l in r if lv == 1]
        return r, lows, highs

    def test_write_transaction_and_exact_scl_timing(self):
        data = [0x11, 0x22]
        for tl, th, name, tlow_min_ns, thigh_min_ns in CONFIGS:
            with self.subTest(name):
                slave = W.I2cSlave(0x50)
                t, w = self.run_i2c([0xA0, len(data)] + data, slave, tl, th, 60 * (tl + th) + 3000)
                self.assertEqual(slave.written, [data])
                self.assertEqual((slave.starts, slave.stops), (1, 1))
                self.assertEqual(slave.addr_seen, [(0x50, 0, True)])
                self.assertEqual(t.uo_out & 0x80, 0, "no error flag")
                self.assertFalse(t.illegal)

                r, lows, highs = self.scl_pulses(w)
                nbytes = 1 + len(data)                       # address + data
                self.assertEqual(len(lows), 9 * nbytes + 1)  # 9 clocks/byte + the STOP low
                # inside the transaction every SCL high phase is exactly TH ...
                self.assertEqual(highs[1:-1], [th] * (9 * nbytes))
                # ... and every low phase is >= TL, exactly TL except where the program
                # runs extra instructions between bytes (first pulse of bytes 2.., STOP)
                self.assertTrue(all(l >= tl for l in lows))
                longer = {i for i, l in enumerate(lows) if l != tl}
                self.assertEqual(longer, {9 * i for i in range(1, nbytes + 1)})
                # SCL frequency is an exact integer
                self.assertEqual(50_000_000 // (tl + th), 100_000 if tl == 250 else 400_000)
                # I2C minimum low/high times, integer nanoseconds
                self.assertGreaterEqual(tl * CLK_NS, tlow_min_ns)
                self.assertGreaterEqual(th * CLK_NS, thigh_min_ns)

    def test_start_and_stop_conditions(self):
        for tl, th, name, _, _ in CONFIGS:
            with self.subTest(name):
                slave = W.I2cSlave(0x50)
                t, w = self.run_i2c([0xA0, 1, 0x7E], slave, tl, th, 40 * (tl + th) + 2000)
                sda = runs(w.levels(3))
                r, lows, highs = self.scl_pulses(w)
                scl_fall0 = [s for lv, s, _ in r if lv == 0][0]
                sda_fall0 = [s for lv, s, _ in sda if lv == 0][0]
                self.assertEqual(scl_fall0 - sda_fall0, th, "START hold = TH")
                scl_rise_last = [s for lv, s, _ in r if lv == 1][-1]
                sda_rise_last = [s for lv, s, _ in sda if lv == 1][-1]
                self.assertEqual(sda_rise_last - scl_rise_last, th, "STOP set-up = TH")

    def test_read_transaction(self):
        for tl, th, name, _, _ in CONFIGS:
            with self.subTest(name):
                slave = W.I2cSlave(0x50, read_bytes=[0xDE, 0xAD, 0xBE])
                t, w = self.run_i2c([0xA1, 3], slave, tl, th, 60 * (tl + th) + 3000)
                self.assertEqual([b for _, b in t.rx], [0xDE, 0xAD, 0xBE])
                self.assertEqual(slave.nacked_reads, 1, "last byte is NACKed")
                self.assertEqual((slave.starts, slave.stops), (1, 1))
                self.assertEqual(t.uo_out & 0x80, 0)
                _, lows, highs = self.scl_pulses(w)
                self.assertEqual(highs[1:-1], [th] * (9 * 4))    # address + 3 data bytes
                self.assertTrue(all(l >= tl for l in lows))

    def test_address_nack_sets_error_flag_and_stops(self):
        slave = W.I2cSlave(0x50)
        t, w = self.run_i2c([0xA2], slave, 250, 250, 40 * 500 + 3000)   # address 0x51: nobody home
        self.assertEqual(slave.addr_seen, [(0x51, 0, False)])
        self.assertEqual(slave.stops, 1)
        self.assertEqual(slave.written, [[]])
        self.assertEqual(t.uo_out & 0x80, 0x80)
        self.assertEqual(set(w.levels(2)[-100:]), {1})
        self.assertEqual(set(w.levels(3)[-100:]), {1})

    def test_error_flag_clears_on_next_command(self):
        slave = W.I2cSlave(0x50)
        t, w = self.run_i2c([0xA2, (30000, 0xA0), (30000, 1), (30000, 0x42)],
                            slave, 250, 250, 70000)
        self.assertEqual(slave.written, [[], [0x42]])
        self.assertEqual(t.uo_out & 0x80, 0)

    def test_clock_stretching(self):
        slave = W.I2cSlave(0x50, stretch=900)
        data = [0x11, 0x22, 0x33]
        t, w = self.run_i2c([0xA0, 3] + data, slave, 250, 250, 80 * 500 + 3000)
        self.assertEqual(slave.written, [data])
        self.assertEqual(slave.stops, 1)
        _, lows, highs = self.scl_pulses(w)
        self.assertGreaterEqual(max(lows), 900, "slave really held SCL low")
        # the master waits for the real rising edge: high phases are still exactly TH
        self.assertEqual(highs[1:-1], [250] * (9 * 4))

    def test_two_transactions_back_to_back(self):
        slave = W.I2cSlave(0x50, read_bytes=[0x99])
        t, w = self.run_i2c([0xA0, 1, 0x01, 0xA1, 1], slave, 75, 50, 200000 // 4)
        self.assertEqual(slave.written, [[0x01], []])
        self.assertEqual([b for _, b in t.rx], [0x99])
        self.assertEqual((slave.starts, slave.stops), (2, 2))


# ====================================================================== all examples
class AllExamples(unittest.TestCase):
    def test_program_sizes(self):
        sizes = {fn: sum(load(fn).used) for fn in sorted(os.listdir(EX_DIR)) if fn.endswith(".asm")}
        self.assertEqual(set(sizes), {"uart_tx.asm", "uart_rx.asm", "spi_master.asm", "i2c_master.asm",
                                    "usb_rx.asm"})   # usb_rx: needs the proposed ttpe_usb_rx primitive
        for fn, n in sizes.items():
            self.assertLess(n, 128, "%s uses %d words" % (fn, n))


if __name__ == "__main__":
    unittest.main()
