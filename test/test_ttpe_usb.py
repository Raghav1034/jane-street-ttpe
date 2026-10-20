"""
cocotb testbench for ttpe_usb_rx (low-speed USB bit slicer), TARGET=usb.

The RTL is compared cycle by cycle with sw/usb_ls.py:UsbRx (the model that the ISS uses and that
docs/USB_LS_RX.md specifies). In addition the RTL outputs are decoded back into packets and compared
with the transmitted bits, so a model that merely agrees with a wrong RTL cannot pass the functional
tests.

Cycle convention (same as test_ttpe_core.py): cycle k starts at rising clock edge k; inputs are
applied just after edge k; the combinational output usb_d is read at the falling edge of cycle k.
usb_d of cycle k is the value the synchroniser flops s2[15:12] take at edge k+1, i.e. the value the
program sees in cycle k+1 (so tick[k+1] = usb_d[0] of cycle k, as in usb_ls.usb_rx_model).

Run:   make TARGET=usb
Subset: cocotb 2.x  COCOTB_TEST_FILTER=fuzz   |  cocotb 1.x  TESTCASE=test_fuzz
"""

import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "sw"))

import usb_ls as U                            # noqa: E402

CLK_NS = 20


def val(sig):
    try:
        return int(sig.value)
    except ValueError:
        raise AssertionError("%s is X/Z (%s)" % (getattr(sig, "_name", "signal"), sig.value))


class Bench:
    def __init__(self, dut):
        self.dut = dut
        self.model = U.UsbRx()
        self.k = 0
        self.tick, self.data, self.stop, self.err = [0], [0], [0], [0]   # value visible in cycle k+1 at index k+1
        self.prev_dp = 0
        self.log = []

    async def start(self):
        d = self.dut
        d.en.value, d.dp_q.value, d.dp_prev_q.value, d.dm_q.value = 0, 0, 0, 1
        d.rst_n.value = 0
        await Timer(1, "ns")
        cocotb.start_soon(Clock(d.clk, CLK_NS, "ns").start())
        for _ in range(4):
            await RisingEdge(d.clk)
        d.rst_n.value = 1

    async def restart(self):
        """Back to the reset state (new model, empty logs) WITHOUT starting another clock driver."""
        d = self.dut
        await RisingEdge(d.clk)
        d.en.value, d.dp_q.value, d.dp_prev_q.value, d.dm_q.value = 0, 0, 0, 1
        d.rst_n.value = 0
        for _ in range(3):
            await RisingEdge(d.clk)
        d.rst_n.value = 1
        self.model = U.UsbRx()
        self.k = 0
        self.tick, self.data, self.stop, self.err = [0], [0], [0], [0]
        self.prev_dp = 0

    async def cycle(self, en, dp, dm, rst=False):
        """One cycle: apply inputs, compare usb_d with the model, record the outputs."""
        d = self.dut
        await RisingEdge(d.clk)
        d.rst_n.value = 0 if rst else 1
        d.en.value, d.dp_q.value, d.dp_prev_q.value, d.dm_q.value = int(en), dp, self.prev_dp, dm
        await FallingEdge(d.clk)
        got = val(d.usb_d)
        if rst:
            self.model = U.UsbRx()                   # asynchronous reset: state cleared inside the cycle
            exp = (0, 0, 0, 0)
        else:
            exp = self.model.step(en, dp, self.prev_dp, dm)
        if not en and not rst:
            exp_cmp = None                           # usb_d is ignored by the synchroniser while en = 0
        else:
            exp_cmp = exp[0] | exp[1] << 1 | exp[2] << 2 | exp[3] << 3
        if exp_cmp is not None and got != exp_cmp:
            raise AssertionError("cycle %d: usb_d RTL 0b%s != model 0b%s (en=%d dp=%d prev=%d dm=%d)" %
                                 (self.k, format(got, "04b"), format(exp_cmp, "04b"), en, dp, self.prev_dp, dm))
        self.tick.append(got & 1 if en else 0)
        self.data.append((got >> 1) & 1 if en else 0)
        self.stop.append((got >> 2) & 1 if en else 0)
        self.err.append((got >> 3) & 1 if en else 0)
        self.prev_dp = dp
        self.k += 1

    async def run(self, dp, dm, en=None, rst=None):
        for k in range(len(dp)):
            await self.cycle(1 if en is None else en[k], dp[k], dm[k], bool(rst and rst[k]))

    def packets(self):
        n = len(self.tick)
        return U.packets_from_ticks(self.tick[:n], self.data[:n], self.stop[:n], self.err[:n])


def synced(dp, dm, tail=300):
    """pad waveform -> what the primitive sees (2-flop synchroniser delay), padded with idle"""
    return dp + [0] * tail, dm + [1] * tail


PK = [U.token("SETUP", 0, 0),
      U.data_packet("DATA0", [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00]),
      [U.pid_byte(U.PID["ACK"])]]


async def run_and_decode(b, pkts, **kw):
    """Fresh reset, transmit `pkts`, run the primitive on the synchronised waveform."""
    await b.restart()
    dp, dm = U.waveform(pkts, **kw)
    dp, dm = synced(U.sync2(dp, 0), U.sync2(dm, 1))
    await b.run(dp, dm)
    return b


# ------------------------------------------------------------------ tests
@cocotb.test()
async def test_reset_and_idle(dut):
    """After reset and in idle (J) the outputs stay 0; with en = 0 and noise the state stays IDLE."""
    b = Bench(dut)
    await b.start()
    await b.run([0] * 80, [1] * 80)
    assert not any(b.tick) and not any(b.stop) and not any(b.err)
    rnd = random.Random(1)
    n = 400
    await b.run([rnd.randrange(2) for _ in range(n)], [rnd.randrange(2) for _ in range(n)], en=[0] * n)
    await b.run([0] * 60, [1] * 60)                  # enabled again in idle: nothing happens
    assert not any(b.tick)


@cocotb.test()
async def test_clean_packets(dut):
    """Token, data and handshake packets decode to exactly the transmitted bits (RTL outputs only)."""
    b = Bench(dut)
    await b.start()
    await run_and_decode(b, PK)
    r = b.packets()
    assert [bits for bits, _ in r] == [U.bits_of(p) for p in PK], [len(x) for x, _ in r]
    assert not any(e for _, e in r)


@cocotb.test()
async def test_rate_jitter_skew(dut):
    """+-1.5 % rate error, edge jitter and D-/D+ skew, several seeds."""
    exp = [U.bits_of(p) for p in PK[:2]]
    b = Bench(dut)
    await b.start()
    for ppm in (-15000, 0, 15000):
        for jit, skew in ((0, 0), (1, 2), (2, 3)):
            for seed in (1, 2):
                await run_and_decode(b, PK[:2], ppm=ppm, jitter=jit, skew=skew, seed=seed)
                r = b.packets()
                assert [bits for bits, _ in r] == exp and not any(e for _, e in r), (ppm, jit, skew, seed)


@cocotb.test()
async def test_stuffing_worst_case(dut):
    """All-ones payload: a stuffed 0 after every six 1s, no TICK for the stuffed cells."""
    pk = [U.data_packet("DATA1", [0xFF] * 8)]
    b = Bench(dut)
    await b.start()
    await run_and_decode(b, pk)
    r = b.packets()
    assert [bits for bits, _ in r] == [U.bits_of(pk[0])]


@cocotb.test()
async def test_errors_and_edge_cases(dut):
    """Stuff violation, lost EOP, short SE0, minimum EOP, truncated packet, idle glitches."""
    # stuff violation: STOP + ERR after the 6 good bits, then the next packet still decodes
    lv, st = U.encode_levels([0xFF, 0x00])
    lv[st[0]] = lv[st[0] - 1]
    dp, dm = [0] * 300, [1] * 300
    for l in lv:
        dp += [l] * 33
        dm += [1 - l] * 33
    dp += [0] * 100
    dm += [0] * 50 + [1] * 50
    good = U.waveform([PK[0]])
    b = Bench(dut)
    await b.start()
    d2, m2 = dp + good[0], dm + good[1]
    d2, m2 = synced(U.sync2(d2, 0), U.sync2(m2, 1))
    await b.run(d2, m2)
    r = b.packets()
    assert [(len(x), e) for x, e in r] == [(6, True), (24, False)], [(len(x), e) for x, e in r]
    assert r[1][0] == U.bits_of(PK[0])
    # lost EOP: idle J after the last data bit decodes as a run of 1s and ends as a stuff violation
    lv, _ = U.encode_levels(PK[0])
    dp, dm = [0] * 300, [1] * 300
    for l in lv:
        dp += [l] * 33
        dm += [1 - l] * 33
    dp += [0] * 600
    dm += [1] * 600
    await b.restart()
    d2, m2 = synced(U.sync2(dp, 0), U.sync2(dm, 1))
    await b.run(d2, m2)
    r = b.packets()
    assert len(r) == 1 and r[0][1] and r[0][0][:24] == U.bits_of(PK[0])
    # SE0 width: 10 cycles is not an EOP; 20 and more were decoded in every case swept (the 11..19 range
    # depends on the phase of the last cell, so it is deliberately not asserted)
    for eop, ok in ((10, False), (20, True), (24, True), (66, True)):
        await run_and_decode(b, PK[:1], eop_cycles=eop)
        r = b.packets()
        assert (len(r) == 1 and r[0][0] == U.bits_of(PK[0]) and not r[0][1]) == ok, (eop, [(len(x), e) for x, e in r])
    # idle glitches on D+ (1, 5 and 20 cycles) are ignored; the next packet decodes
    dp, dm = U.waveform(PK[:1], lead=900)
    for pos, w in ((100, 1), (300, 5), (500, 20)):
        for k in range(w):
            dp[pos + k] = 1
    await b.restart()
    d2, m2 = synced(U.sync2(dp, 0), U.sync2(dm, 1))
    await b.run(d2, m2)
    assert [x for x, _ in b.packets()] == [U.bits_of(PK[0])]


@cocotb.test()
async def test_enable_toggle(dut):
    """en low in the middle of a packet clears the state; en high again in idle decodes the next one."""
    dp, dm = U.waveform(PK[:2])
    dp, dm = synced(U.sync2(dp, 0), U.sync2(dm, 1))
    n = len(dp)
    en = [1] * n
    for k in range(700, 760):
        en[k] = 0
    b = Bench(dut)
    await b.start()
    await b.run(dp, dm, en=en)
    r = b.packets()
    # Disabling mid-packet emits no STOP: the bits already delivered stay in front of the next packet,
    # which then comes out whole at the end (documented limitation: change en only while the bus is idle).
    assert r and not r[-1][1] and r[-1][0][-88:] == U.bits_of(PK[1]), [(len(x), e) for x, e in r]
    assert len(r[-1][0]) > 88


@cocotb.test()
async def test_async_reset_mid_packet(dut):
    """rst_n low in the middle of a packet clears the primitive at once; the next packet decodes."""
    dp, dm = U.waveform(PK[:2])
    dp, dm = synced(U.sync2(dp, 0), U.sync2(dm, 1))
    n = len(dp)
    rst = [0] * n
    for k in range(650, 654):
        rst[k] = 1
    b = Bench(dut)
    await b.start()
    await b.run(dp, dm, rst=rst)
    r = b.packets()
    # same as the enable test: no STOP for the aborted packet, the next packet is whole at the end
    assert r and not r[-1][1] and r[-1][0][-88:] == U.bits_of(PK[1])


def random_stream(rng, n_pkts):
    """Random mixture of good packets, corrupted packets, lost EOPs, short SE0s and noise bursts."""
    dp, dm = [], []
    for _ in range(n_pkts):
        kind = rng.choice(["good", "good", "good", "stuff", "noeop", "short", "trunc", "noise"])
        payload = [rng.choice([0x00, 0xFF, rng.randrange(256)]) for _ in range(rng.randrange(0, 9))]
        pid = rng.choice(list(U.PID))
        pkt = U.data_packet(pid, payload) if rng.randrange(2) else [U.pid_byte(U.PID[pid])]
        kw = dict(ppm=rng.randint(-15000, 15000), jitter=rng.randint(0, 2), skew=rng.randint(0, 3),
                  seed=rng.randrange(1 << 30), gap_cycles=rng.choice([0, 30, 200, 600]), lead=rng.choice([1, 40, 300]),
                  eop_cycles=rng.choice([20, 30, 50, 66]))
        if kind == "noeop":
            kw["eop_cycles"] = 0
        elif kind == "short":
            kw["eop_cycles"] = rng.randint(1, 19)
        p, m = U.waveform([pkt], **kw)
        if kind == "stuff":
            lv, st = U.encode_levels([0xFF, 0x00, 0xFF])
            lv[st[0]] = lv[st[0] - 1]
            p, m = [0] * 100, [1] * 100
            for l in lv:
                p += [l] * 33
                m += [1 - l] * 33
            p += [0] * 80
            m += [0] * 50 + [1] * 30
        elif kind == "trunc":
            cut = rng.randrange(40, len(p) - 80) if len(p) > 130 else len(p)
            p, m = p[:cut], m[:cut]
        elif kind == "noise":
            L = rng.randrange(20, 400)
            p = [rng.randrange(2) if rng.randrange(3) == 0 else (p[-1] if p else 0) for _ in range(L)]
            m = [rng.randrange(2) if rng.randrange(3) == 0 else 1 for _ in range(L)]
        for _ in range(rng.choice([0, 0, 1, 3])):           # sprinkle glitches
            if len(p) > 10:
                pos, w = rng.randrange(len(p) - 4), rng.randint(1, 3)
                arr = p if rng.randrange(2) else m
                for k in range(pos, pos + w):
                    arr[k] ^= 1
        dp += p
        dm += m
    return dp, dm


@cocotb.test()
async def test_fuzz(dut):
    """Random streams with random enable gaps: RTL == model on every cycle."""
    total = 0
    b = Bench(dut)
    await b.start()
    for seed in range(4):
        rng = random.Random(1000 + seed)
        dp, dm = random_stream(rng, 14)
        dp, dm = synced(U.sync2(dp, 0), U.sync2(dm, 1))
        n = len(dp)
        en = [1] * n
        for _ in range(rng.choice([0, 2, 4])):
            s = rng.randrange(n)
            for k in range(s, min(n, s + rng.randrange(1, 300))):
                en[k] = 0
        await b.restart()
        await b.run(dp, dm, en=en)
        total += n
    dut._log.info("fuzz: %d cycles compared", total)
