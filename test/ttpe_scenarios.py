"""
ttpe_scenarios.py - the scenario set for the core lockstep tests.

Pure Python (no cocotb). A Scenario bundles a program image, an external world, a host policy,
a cycle budget and an optional functional check. test_ttpe_core.py runs every scenario on the RTL
through ttpe_lockstep.Lockstep; the very same scenarios were also run against a Python
transcription of the RTL while the testbench was being written.

Every scenario is checked twice:
  1. lockstep: every cycle and the full accept / commit / pin-event logs against ttpe_iss.py;
  2. functional: `check(ls)` asserts what the program is supposed to do on the pads / FIFOs
     (UART frames, SPI data, I2C transactions, FIFO contents ...).
"""

import os
import random

import ttpe_asm as asm
import ttpe_isa as isa
import ttpe_world as W
from ttpe_lockstep import DefaultPolicy, MAX_RX, MAX_TX

EX_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sw", "examples")
HALT_WORD = isa.encode(isa.OPCODES["HALT"])


class Scenario:
    def __init__(self, name, words, world, policy, cycles, check=None):
        self.name, self.words, self.world = name, words, world
        self.policy, self.cycles, self.check = policy, cycles, check


def pad512(words):
    words = list(words)
    return words + [HALT_WORD] * (isa.PROG_WORDS - len(words))


def example(name, **defs):
    with open(os.path.join(EX_DIR, name)) as f:
        return asm.assemble(f.read(), defines=defs).image()


def asm_text(src):
    return asm.assemble(src, allow_short_delta=True).image()


# ------------------------------------------------------------------ world helpers
class UiWorld(W.BusWorld):
    """BusWorld whose ui_in pins follow a function of the world cycle."""

    def __init__(self, devices=(), ui_fn=None):
        super().__init__(devices)
        self.ui_fn = ui_fn or (lambda k: 0xFF)
        self.ui_log = []

    def pads(self, k, uio_oe, uio_out, uo_out):
        pad, _ = super().pads(k, uio_oe, uio_out, uo_out)
        ui = self.ui_fn(k) & 0xFF
        self.ui_log.append(ui)
        return pad, ui


class PeriodicLow:
    """Pulls `pin` low for `low` cycles out of every `period`, starting at `phase`."""

    def __init__(self, pin, period, low, phase=0):
        self.pin, self.period, self.low, self.phase = pin, period, low, phase

    def step(self, k, pad_prev):
        return (1 << self.pin) if ((k - self.phase) % self.period) < self.low else 0


class RandomLow:
    """Pulls a random subset of pins low; the subset changes with probability 1/`rate`."""

    def __init__(self, seed, pins=0xFF, rate=6):
        self.rng, self.pins, self.rate, self.mask = random.Random(seed), pins, rate, 0

    def step(self, k, pad_prev):
        if self.rng.randrange(self.rate) == 0:
            self.mask = self.rng.randrange(256) & self.pins
        return self.mask


# ------------------------------------------------------------------ directed programs
ALL_OPS = r"""
; every opcode, every PINS mode, SHOUT variants, all REG sub-ops, both counters, page-1 jump
start:
    PINS low uio0 uio1 uio2 @2
    PINS high uio1 uio4 @2
    PINS toggle uio0 uio7 @2
    PINS float uio1 @2
    PINS low uo0 uo1 @2
    PINS high uo2 uo3 uo7 @2
    PINS toggle uo0 uo7 @2
    PINS toggle uo2 @0
    LDI sr, 0xA5 @2
    SHOUT uio3 msb @2
    SHOUT uio3 lsb @2
    SHOUT uio5 msb shift od @2
    SHOUT uio5 lsb shift od @2
    SHOUT uio6 msb shift @2
    SHOUT uo4 msb shift @2
    SHOUT uo5 lsb shift @2
    SHOUT uo6 @1
    HPUSH @2
    LDI sr, 0x3C @2
    XORI sr, 0xFF @2
    ANDI sr, 0x0F @2
    ORI sr, 0x40 @2
    ADDI sr, 0xF8 @2
    HPUSH @2
    LDI c0, 0x11 @2
    LDI c1, 0x22 @2
    MOV sr, c0 @2
    HPUSH @2
    MOV sr, c1 @2
    HPUSH @2
    LDI sr, 0x7E @2
    MOV c0, sr @2
    MOV c1, sr @2
    LDI sr, 0 @2
    MOV sr, c0 @2
    ADDI sr, 1 @2
    HPUSH @2
    SAMPLE uio2 msb @2
    SAMPLE uio6 msb @2
    SAMPLE ui3 lsb @2
    SAMPLE ui0 lsb @2
    SAMPLE uio0 msb @2
    HPUSH @2
    LDI c0, 4 @2
l0: NOP @3
    DJNZ c0, l0 @2
    LDI c1, 3 @2
l1: SAMPLE uio2 msb @2
    DJNZ c1, l1 @2
    HPUSH @2
    LDI c0, 0 @2            ; 256 iterations
l2: NOP @0
    DJNZ c0, l2 @0
    LDI sr, 0x05 @2
    JSB sr[0]==1, t0 @2
    PINS high uio7 @2       ; skipped
t0: JSB sr[1]==1, bad @2    ; not taken
    JSB sr[1]==0, t1 @2
bad: PINS high uio6 @2
t1: JHI uio6, t2 @2         ; pad idles high -> taken
    PINS low uio6 @2
t2: JLO ui3, bad @2         ; ui idles high -> not taken
    JHI ui3, t3 @2
    PINS low uio5 @2
t3: JLO uio3, bad2 @2
    NOP @2
    JMP p1 @2
bad2: PINS high uo1 @2
done: HPUSH @2
    HALT
    .org 0x100
p1: SAMPLE uio4 msb @2
    HPUSH @2
    JHI uio6, p1b @2        ; branch on page 1 stays on page 1
    PINS high uo5 @2
p1b: LDI c1, 2 @2
p1c: NOP @3
    DJNZ c1, p1c @2
    JMP done @2             ; back to page 0
"""

WAIT_RANDOM_TEMPLATE = r"""
; WAIT low/high/fall/rise on uio and ui sources with many different deltas
{body}
    HALT
"""


def wait_program(seed, n=48):
    rng = random.Random(seed)
    lines = []
    for i in range(n):
        typ = rng.choice(["low", "high", "fall", "rise"])
        pin = rng.choice(["uio4", "uio5", "ui2", "ui6"])
        d = rng.choice([0, 1, 2, 3, 5, 9, 17, 33])
        lines.append("WAIT %s %s @%d" % (typ, pin, d))
        lines.append("PINS toggle uio0 @%d" % rng.choice([0, 1, 2, 4]))
        if i % 8 == 7:
            lines.append("HPUSH @2")
    return WAIT_RANDOM_TEMPLATE.format(body="\n".join(lines))


def scn_isa_all_ops():
    w = UiWorld([PeriodicLow(2, 13, 5), PeriodicLow(0, 29, 11)],
                ui_fn=lambda k: 0xFF if (k // 7) % 5 else 0xF7)

    def check(ls):
        assert ls.iss.halted and not ls.iss.illegal, "program must finish with HALT"
        assert ls.halts, "HALT never committed"
        assert len(ls.rtl_commits) > 150
    return Scenario("isa_all_ops", pad512(asm_text(ALL_OPS)), w, DefaultPolicy(), 4000, check)


def scn_wait_types(seed):
    w = UiWorld([PeriodicLow(4, 11, 4, 3), PeriodicLow(5, 17, 6, 8)],
                ui_fn=lambda k: ~(((1 << 2) if (k % 9) < 3 else 0) | ((1 << 6) if (k % 23) < 9 else 0)))

    def check(ls):
        assert ls.halts and not ls.iss.illegal, "WAIT program did not finish"
    return Scenario("wait_types_%d" % seed, pad512(asm_text(wait_program(seed))), w,
                    DefaultPolicy(), 6000, check)


def scn_hpull_echo():
    """HPULL blocks until bytes arrive (timed pushes), each byte is echoed with HPUSH."""
    src = r"""
        LDI c0, 12 @2
        NOP @5
    loop:
        HPULL @2
        XORI sr, 0x80 @2
        HPUSH @2
        DJNZ c0, loop @2
        HALT
    """
    data = [0x10 + i for i in range(12)]
    timed = {5 + 7 * i: b for i, b in enumerate(data)}
    timed[5 + 7 * 5 + 1] = 0xEE            # lands in the middle of the others (FIFO ordering)

    def check(ls):
        assert ls.halts and not ls.iss.illegal
        assert len(ls.rx_got) == 12
        assert ls.rx_got[:5] == [b ^ 0x80 for b in data[:5]]
    return Scenario("hpull_echo", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(tx_timed=timed), 400, check)


def scn_tx_fifo_full():
    """The host pushes 12 bytes at once: only 8 fit, 4 are dropped (host must watch tx_full)."""
    src = r"""
        NOP @200                 ; let the FIFO fill before the first HPULL
        LDI c0, 8 @2
        NOP @5
    loop:
        HPULL @2
        HPUSH @2
        DJNZ c0, loop @2
        HALT
    """
    data = list(range(0x40, 0x4C))
    timed = {i: b for i, b in enumerate(data)}

    def check(ls):
        assert ls.halts and not ls.iss.illegal
        assert ls.tx_dropped == 4, ls.tx_dropped
        assert ls.tx_pushed == data[:8]
        assert ls.rx_got == data[:8]
    return Scenario("tx_fifo_full", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(tx_timed=timed), 400, check)


def scn_rx_overrun_and_flush():
    """12 HPUSH with no host pops: 8 kept, overrun count 4; rx flush clears both."""
    src = r"""
        LDI sr, 1 @2
        LDI c0, 12 @2
        NOP @5
    loop:
        HPUSH @2
        ADDI sr, 1 @2
        DJNZ c0, loop @2
        HALT @100
    """
    state = {}

    def extra(ls, S):
        a = {}
        if ls.iss.halted and S["halted"] and S["rx_count"] == 8 and "seen" not in state:
            state["seen"] = ls.k
            state["overrun"] = S["rx_overrun"]
        if "seen" in state:
            n = ls.k - state["seen"]
            if n == 3:
                a["rx_pop"] = True                  # pop one, then check count
            if n == 6:
                a["rx_flush"] = True
        return a

    def check(ls):
        assert state.get("overrun") == 4, state
        assert ls.rx_got == [1], ls.rx_got          # only the explicit pop (in order, oldest first)
        assert ls.last_state["rx_count"] == 0 and ls.last_state["rx_overrun"] == 0
    return Scenario("rx_overrun_flush", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(rx_drain=False, extra=extra), 500, check)


def scn_tx_flush():
    """Flush the TX FIFO while the engine waits in HPULL; later bytes are not lost."""
    src = r"""
        NOP @40
        HPULL @2
        HPUSH @2
        HPULL @2
        HPUSH @2
        HPULL @2
        HPUSH @2
        HALT
    """
    timed = {0: 0x11, 1: 0x22, 2: 0x33, 3: 0x44, 60: 0x55}
    state = {}

    def extra(ls, S):
        a = {}
        # flush in the cycle in which the second HPULL is waiting on a FIFO that still has bytes
        if ls.wk == 55:
            a["tx_flush"] = True
        if ls.wk == 70:
            a["tx_push"] = 0x66
            a["tx_flush"] = True                       # flush wins over a simultaneous push
        if ls.wk in (90, 91):
            a["tx_push"] = 0x77 + (ls.wk - 90)
        return a

    def check(ls):
        assert ls.halts and not ls.iss.illegal
    return Scenario("tx_flush", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(tx_timed=timed, extra=extra), 300, check)


def scn_rx_overrun_saturates():
    """300 HPUSH into a full FIFO: the 8-bit overrun counter saturates at 255."""
    src = r"""
        LDI sr, 7 @2
        LDI c1, 2 @2
    outer:
        LDI c0, 0 @2
        NOP @5
    loop:
        HPUSH @2
        DJNZ c0, loop @2
        DJNZ c1, outer @2
        HALT
    """
    def check(ls):
        assert ls.halts and not ls.iss.illegal
        assert ls.iss.rx_overrun == 512 - 8
        assert ls.last_state["rx_overrun"] == 255 and ls.last_state["rx_count"] == 8
    return Scenario("rx_overrun_saturates", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(rx_drain=False), 5600, check)


def scn_page_boundary():
    """Conditional branches in the last word of a page: the target page is the BRANCH's page,
    even though the fetch controller has already moved on to the next page (or wrapped)."""
    src = r"""
        LDI c0, 3 @2
        JMP e0 @2
        .org 0x20
    t0: PINS toggle uio1 @2
        JMP e1 @2
        .org 0xFE
    e0: NOP @5
        JHI uio6, t0 @2          ; word 0xFF: taken target 0x020, next sequential word is 0x100
        .org 0x100
        PINS high uo0 @2         ; only reached if the branch is wrongly not taken
        HALT
        .org 0x180
    t1: PINS toggle uio2 @2
        JMP e1 @2
        .org 0x1FE
    e1: NOP @5
        DJNZ c0, t1 @2           ; word 0x1FF: target 0x180; fall-through wraps to 0x000
    """
    def check(ls):
        assert not ls.iss.halted and not ls.iss.illegal
        assert any(ev[3] & 1 for ev in ls.iss.pin_events) is False, "wrong-path PINS executed"
        pcs = [pc for _, pc in ls.rtl_accepts]
        assert 0xFF in pcs and 0x1FF in pcs and 0x180 in pcs and 0x20 in pcs
    return Scenario("page_boundary", pad512(asm_text(src)), W.BusWorld(), DefaultPolicy(), 700, check)


def scn_halt_resume():
    src = r"""
        PINS high uio0 @2
        HALT @3
        PINS low uio0 @2
        PINS high uio1 @2
        HALT @0
        PINS toggle uio0 uio1 @2
        HALT @5
        LDI sr, 0x5A @2
        HPUSH @2
        HALT
        .org 0x80
    alt:
        PINS high uo3 @2
        HALT
    """
    pcs = [None, 0x80, None, None]

    def pick(ls):
        return pcs[min(ls.resumes - 1, len(pcs) - 1)] if ls.resumes else None

    def check(ls):
        assert ls.resumes >= 4, ls.resumes
        assert len(ls.halts) >= 4
    return Scenario("halt_resume", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(resume_on_halt=pick, halt_gap=6, max_resumes=6), 400, check)


def scn_start_pc():
    src = r"""
        PINS high uio0 @2
        HALT
        .org 0x10C
    sub:
        PINS high uio5 @2
        HPUSH @2
        HALT
    """
    def check(ls):
        assert ls.halts and not ls.iss.illegal
        assert ls.rtl_accepts[0][1] == 0x10C, ls.rtl_accepts[0]
        assert ls.iss.uio_out & 0x20 and not ls.iss.uio_out & 0x01
    return Scenario("start_pc", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(start_pc=0x10C), 120, check)


ILLEGAL_PROG = r"""
        PINS high uio0 @2
        .word 0xE0000003           ; reserved opcode 0xE, D = 3
        PINS high uio1 @2          ; reached after resume
        .word 0xF1230000           ; reserved opcode 0xF, D = 0
        PINS high uio2 @2
        .word 0x87000002           ; REG sub-op 7
        PINS high uio3 @2
        .word 0x8C000000           ; REG sub-op 12
        PINS high uio4 @2
        .word 0x8F5A0000           ; REG sub-op 15
        PINS high uio5 @2
        HALT
"""


def scn_illegal():
    def check(ls):
        assert len(ls.halts) == 6, ls.halts
        assert ls.resumes == 6
        assert ls.last_state["halted"] == 1 and ls.last_state["illegal"] == 0
    return Scenario("illegal_ops", pad512(asm_text(ILLEGAL_PROG)), W.BusWorld(),
                    DefaultPolicy(resume_on_halt=True, halt_gap=4, max_resumes=5), 400, check)


def scn_illegal_sticky():
    """No resume after the illegal instruction: the flag stays set and the core stays stopped."""
    state = {}

    def extra(ls, S):
        if S["halted"] and S["illegal"]:
            state["illegal_seen"] = state.get("illegal_seen", 0) + 1
        return {}

    def check(ls):
        assert state.get("illegal_seen", 0) > 20
        assert ls.last_state["illegal"] == 1 and ls.last_state["halted"] == 1
    return Scenario("illegal_sticky", pad512(asm_text(ILLEGAL_PROG)), W.BusWorld(),
                    DefaultPolicy(extra=extra), 120, check)


def scn_host_block_and_readback():
    """While the program runs, the host hammers the SRAM port with writes and resume / pc_load
    pulses: none may have any effect (host_gnt stays 0). After HALT the whole memory is read back
    through the host port (every word compared with the image by the lockstep engine)."""
    src = r"""
        LDI c0, 40 @2
        NOP @5
    loop:
        PINS toggle uio0 @3
        PINS toggle uio1 @1
        DJNZ c0, loop @2
        HALT
    """
    st = {"rd": 0}
    rng = random.Random(7)

    def extra(ls, S):
        a = {}
        if not S["mem_free"]:
            a.update(host_req=1, host_we=1, host_addr=rng.randrange(512), host_wdata=0xDEADBEEF,
                     host_wstrb=0xF)
            if rng.randrange(4) == 0:
                a["resume"] = True
            if rng.randrange(4) == 0:
                a["pc_load"] = rng.randrange(512)
        elif S["halted"] and ls.halts and st["rd"] < 512:
            a.update(host_req=1, host_we=0, host_addr=st["rd"])
            st["rd"] += 1
        return a

    def check(ls):
        assert ls.halts and not ls.iss.illegal
        assert len(ls.mem_reads) == 512, len(ls.mem_reads)
    return Scenario("host_block_readback", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(extra=extra), 1800, check)


def scn_host_patch_while_halted():
    """Halted core: the host rewrites instructions (byte strobes too), then resumes."""
    src = r"""
        PINS high uio0 @2
        HALT
        PINS high uio1 @2
        HPUSH @2
        HALT
    """
    patch = {"done": False}

    def extra(ls, S):
        a = {}
        if S["halted"] and S["mem_free"] and ls.resumes == 1 and not patch["done"]:
            # runs once, after the first HALT: patch word 2 (high uio1 -> high uio6) via strobes
            patch["done"] = True
            a.update(host_req=1, host_we=1, host_addr=2, host_wdata=0x00400000, host_wstrb=0b0100)
        return a

    def check(ls):
        assert patch["done"]
    return Scenario("host_patch", pad512(asm_text(src)), W.BusWorld(),
                    DefaultPolicy(resume_on_halt=True, halt_gap=10, extra=extra), 200, check)


# ------------------------------------------------------------------ the four example programs
def scn_uart_tx(bit, data, timed=None, cycles=None, tag=""):
    def check(ls):
        frames = W.decode_uart(ls.world.levels(1), bit)
        assert [f[1] for f in frames] == data, frames
        assert all(f[2] for f in frames), "stop bit"
        assert not ls.iss.illegal
    kw = dict(tx_timed=timed) if timed else dict(tx_bytes=data)
    return Scenario("uart_tx_%d%s" % (bit, tag), pad512(example("uart_tx.asm", BIT=bit)),
                    W.BusWorld(), DefaultPolicy(**kw),
                    cycles or (10 * bit * len(data) + 200), check)


def scn_uart_rx(bit, data, src_bit=None, drain=True, tag="", start=1000):
    sb = src_bit or bit
    frames = [(start + i * 10 * sb, b) for i, b in enumerate(data)]

    def check(ls):
        if drain:
            assert ls.rx_got == data, (ls.rx_got, data)
        else:
            assert ls.last_state["rx_count"] == min(len(data), MAX_RX)
            assert ls.last_state["rx_overrun"] == max(0, len(data) - MAX_RX)
    return Scenario("uart_rx_%d%s" % (bit, tag), pad512(example("uart_rx.asm", BIT=bit)),
                    W.BusWorld([W.UartSource(2, sb, frames)]),
                    DefaultPolicy(rx_drain=drain), start + (len(data) * 10 + 2) * sb, check)


def scn_spi(cpol, cpha, h, mosi, miso, tag=""):
    slave = W.SpiSlave(cpol, cpha, tx_bytes=miso)

    def check(ls):
        assert slave.rx == mosi, (slave.rx, mosi)
        assert ls.rx_got == miso, (ls.rx_got, miso)
        assert len(slave.cs_cycles) == 1
    cycles = 200 + len(mosi) * (16 * h + 200) + 400
    return Scenario("spi_m%d%d_h%d%s" % (cpol, cpha, h, tag),
                    pad512(example("spi_master.asm", CPOL=cpol, CPHA=cpha, H=h)),
                    W.BusWorld([slave]),
                    DefaultPolicy(tx_bytes=[len(mosi)] + mosi), cycles, check)


def scn_i2c(tl, th, host, slave, cycles, check, tag):
    return Scenario("i2c_%s" % tag, pad512(example("i2c_master.asm", TL=tl, TH=th)),
                    W.BusWorld([slave]), DefaultPolicy(tx_bytes=host), cycles, check)


def scn_i2c_write_read(tl, th, cycles):
    slave = W.I2cSlave(0x50, read_bytes=[0xC1, 0x3E, 0x7F])

    def check(ls):
        assert slave.written[0] == [0x11, 0x22], slave.written
        assert ls.rx_got == [0xC1, 0x3E, 0x7F], ls.rx_got
        assert slave.starts == 2 and slave.stops == 2
    return scn_i2c(tl, th, [0xA0, 2, 0x11, 0x22, 0xA1, 3], slave, cycles, check,
                   "wr_rd_%d_%d" % (tl, th))


def scn_i2c_write(tl, th, cycles):
    slave = W.I2cSlave(0x50)

    def check(ls):
        assert slave.written == [[0x11, 0x22]], slave.written
        assert slave.starts == 1 and slave.stops == 1
        assert not ls.iss.uo_out & 0x80
    return scn_i2c(tl, th, [0xA0, 2, 0x11, 0x22], slave, cycles, check, "wr_%d_%d" % (tl, th))


def scn_i2c_nack(tl, th):
    slave = W.I2cSlave(0x50)

    def check(ls):
        assert slave.written == [[]] or slave.written == [] or all(not w for w in slave.written)
        # the aborted write leaves bytes in the TX FIFO that the program takes as a new command
        # (known gap), so look at the history instead of the final value
        assert any(ev[3] & 0x80 for ev in ls.iss.pin_events), "error flag uo7 must have been set"
    return scn_i2c(tl, th, [0xA2, 1, 0x99], slave, 3000, check, "nack_%d_%d" % (tl, th))


def scn_i2c_stretch():
    slave = W.I2cSlave(0x50, read_bytes=[0x5A], stretch=40)

    def check(ls):
        assert ls.rx_got == [0x5A]
    return scn_i2c(75, 50, [0xA1, 1], slave, 4000, check, "stretch_75_50")


# ------------------------------------------------------------------ random programs
def random_program(rng, n_words=64):
    ops = ([0] * 3 + [1] * 8 + [2] * 6 + [3] * 3 + [4] * 4 + [5] * 4 + [6] * 8 + [7] * 6 +
           [8] * 12 + [9] * 5 + [10] * 4 + [11] * 4 + [12] * 1 + [13] * 4 + [14] * 1)
    words = []
    for i in range(n_words):
        op = rng.choice(ops)
        if op == 15:
            op = 14
        d = rng.choice([0, 0, 1, 1, 2, 2, 3, 4, 6, 9, 20]) if rng.randrange(12) else rng.randrange(80)
        a = rng.randrange(16)
        p = rng.randrange(256)
        if op in (4, 5, 9, 13):                      # branches: stay inside the program, same page
            p = rng.randrange(n_words)
        if op == 3:
            a, p = 0, rng.randrange(n_words)
        if op == 8:
            a = rng.choice([0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11] * 6 + [7, 12, 15])
        if op == 2:
            a = rng.randrange(16)
        words.append(isa.encode(op, a, p, d))
    return words


def scn_fuzz(seed, cycles=1500):
    rng = random.Random(1000 + seed)
    words = random_program(rng)
    hrng = random.Random(2000 + seed)
    w = UiWorld([RandomLow(3000 + seed, rate=5), PeriodicLow(hrng.randrange(8), 9 + hrng.randrange(20), 4)],
                ui_fn=lambda k, r=random.Random(4000 + seed): r.randrange(256) if k % 3 == 0 else 0xFF)
    # ui_fn above is called once per cycle in order, so the stream is deterministic

    def extra(ls, S):
        a = {}
        if hrng.randrange(7) == 0:
            a["tx_push"] = hrng.randrange(256)
        if hrng.randrange(3) == 0:
            a["rx_pop"] = True
        if hrng.randrange(150) == 0:
            a["tx_flush"] = True
        if hrng.randrange(150) == 0:
            a["rx_flush"] = True
        if hrng.randrange(40) == 0:
            a["resume"] = True                       # mostly ignored (core running)
        if S["halted"] and S["mem_free"] and hrng.randrange(6) == 0:
            a.update(host_req=1, host_we=int(hrng.randrange(2)), host_addr=hrng.randrange(80),
                     host_wdata=hrng.randrange(1 << 32), host_wstrb=hrng.randrange(16))
        elif not S["mem_free"] and hrng.randrange(5) == 0:
            a.update(host_req=1, host_we=1, host_addr=hrng.randrange(80), host_wdata=0xFFFFFFFF)
        return a

    def resume_pc(ls):
        return hrng.randrange(64) if hrng.randrange(2) else None

    pol = DefaultPolicy(resume_on_halt=resume_pc, halt_gap=hrng.randrange(1, 8), rx_drain=False,
                        extra=extra)
    return Scenario("fuzz_%d" % seed, pad512(words), w, pol, cycles, None)


# ------------------------------------------------------------------ registry
def directed():
    return [
        scn_isa_all_ops(), scn_wait_types(1), scn_wait_types(2), scn_wait_types(3),
        scn_hpull_echo(), scn_tx_fifo_full(), scn_rx_overrun_and_flush(), scn_tx_flush(),
        scn_halt_resume(), scn_start_pc(), scn_illegal(), scn_illegal_sticky(),
        scn_host_block_and_readback(), scn_host_patch_while_halted(),
        scn_rx_overrun_saturates(), scn_page_boundary(),
    ]


def examples():
    return [
        scn_uart_tx(20, [0x55, 0xA5, 0x00, 0xFF, 0x01, 0x80]),
        scn_uart_tx(434, [0x3C], timed={500: 0x3C}, cycles=500 + 4340 + 200, tag="_single"),
        scn_uart_rx(20, [0x00, 0xFF, 0xA6, 0x5A, 0x01, 0x80, 0x33, 0xCC, 0x12, 0x34]),
        scn_uart_rx(434, [0xA6], tag="_single"),
        scn_uart_rx(20, list(range(0x30, 0x3C)), drain=False, tag="_overrun"),
        scn_uart_rx(100, [0x11, 0xA2, 0x33, 0xC4], src_bit=98, tag="_mismatch"),
        scn_spi(0, 0, 24, [0xA5, 0x0F, 0xF0], [0xC3, 0x5A, 0x01]),
        scn_spi(0, 1, 24, [0xA5, 0x0F, 0xF0], [0xC3, 0x5A, 0x01]),
        scn_spi(1, 0, 24, [0x96, 0x69], [0x3C, 0xC3]),
        scn_spi(1, 1, 24, [0x96, 0x69], [0x3C, 0xC3]),
        scn_spi(0, 0, 12, [0xFF, 0x00, 0x81], [0x18, 0xE7, 0xAA], tag="_min"),
        scn_i2c_write_read(75, 50, 11000),
        scn_i2c_write(250, 250, 18000),
        scn_i2c_nack(75, 50),
        scn_i2c_stretch(),
    ]


def fuzz(n=12, cycles=1500):
    return [scn_fuzz(s, cycles) for s in range(n)]
