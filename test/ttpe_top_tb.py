"""
Chip-level testbench library for tt_um_ttpe. Pure Python + async/await; it talks to the design
only through an adapter that exposes the Tiny Tapeout pins:

    adapter.assert_reset() / adapter.release_reset()      rst_n pin (asynchronous)
    await adapter.settle()                                lets a pin write take effect (no clock edge)
    await adapter.next_cycle()                            advance to the start of the next cycle
    adapter.outs() -> {"uo_out", "uio_out", "uio_oe"}     pin values of the current cycle
    adapter.drive(ui_in, uio_in)                          inputs of the current cycle

Cycle convention (same as ttpe_lockstep.py): cycle k starts at rising clock edge k; flop outputs
are read just after the edge; inputs driven in cycle k are sampled at the edge that ends it.

Everything here is strictly pin-level: the host talks SPI on ui_in[2:0] (SCK, MOSI, CS_N) and
reads MISO on uo_out[0]; protocol devices live on uio[7:0] (sw/ttpe_world.py models).

Exact reference for running programs: the Python ISS (sw/ttpe_iss.py). A Monitor compares the
pins uio_oe / uio_out / uo_out of every cycle with the ISS; ISS cycle 0 is the cycle after the
host_resume pulse, which is `rise + 4` for a CTRL write whose last SCK rising edge is first
driven in cycle `rise` (SCK/MOSI synchroniser: 2 flops, rx_valid flop: +1, resume pulse in
rise + 3, first running cycle rise + 4).

uo_out[0] carries MISO while the synchronised CS_N is low (pad cs_n of cycle k - 2); the
Monitor skips uo_out[0] in exactly those cycles.
"""

import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "sw"))

import ttpe_iss as iss_mod                      # noqa: E402
import ttpe_isa as isa                          # noqa: E402
import ttpe_scenarios as scn                    # noqa: E402
import ttpe_world as W                          # noqa: E402
from ttpe_lockstep import FixedWorld            # noqa: E402

# ---------------------------------------------------------------- register map
R_ID, R_STATUS, R_CTRL, R_PC_LO, R_PC_HI, R_FIFO_CNT, R_RX_OVR, R_SCRATCH = 0, 1, 2, 3, 4, 5, 6, 7
R_SA_LO, R_SA_HI, R_ERR = 8, 9, 0x0A
R_TX_PUSH, R_RX_POP, R_SRAM = 0x10, 0x11, 0x12
ID_VALUE = 0x54

CTRL_RESUME, CTRL_PC_LOAD, CTRL_TX_FLUSH, CTRL_RX_FLUSH, CTRL_HALT, CTRL_CORE_RESET = \
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20

ST_HALTED, ST_ILLEGAL, ST_MEM_FREE, ST_TX_EMPTY, ST_TX_FULL, ST_RX_EMPTY, ST_RX_FULL, ST_OVR = \
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80

ERR_SRAM_BUSY, ERR_BAD_CMD, ERR_TX_DROP = 1, 2, 4

HALT_WORD = isa.encode(isa.OPCODES["HALT"])


def trim(words, pad=8):
    """Program image without the HALT padding, plus `pad` HALT words behind it."""
    n = len(words)
    while n > 0 and words[n - 1] == HALT_WORD:
        n -= 1
    return list(words[:n]) + [HALT_WORD] * pad


class TopError(AssertionError):
    pass


class Monitor:
    """Cycle-exact pin comparison of the DUT against the ISS."""

    def __init__(self, h, words, world, host_tx=(), window=1000, name="prog"):
        self.h, self.world, self.window, self.name = h, world, window, name
        self.iss = iss_mod.Ttpe(scn.pad512(words), host_tx=list(host_tx))
        self.S = None
        self.fixed = FixedWorld()
        self.compared = 0
        self.last_k = None

    def set_start(self, k):
        self.S = k

    def add_tx(self, avail_k, byte):
        """The byte becomes visible to HPULL in DUT cycle avail_k."""
        assert self.S is not None and avail_k >= self.S + self.iss.cycle, "tx byte in the past"
        self.iss.tx.append((avail_k - self.S, byte))

    def hook(self, k, o):
        if self.S is None or k < self.S or k >= self.S + self.window:
            return None
        t = self.iss
        if t.cycle != k - self.S:
            raise TopError("[%s] monitor out of step: ISS %d vs DUT %d" % (self.name, t.cycle, k - self.S))

        def bad(what, dut, ref):
            raise TopError("[%s] DUT cycle %d (ISS %d): %s: DUT 0x%02X != ISS 0x%02X" %
                           (self.name, k, t.cycle, what, dut, ref))
        if o["uio_oe"] != t.uio_oe:
            bad("uio_oe", o["uio_oe"], t.uio_oe)
        if o["uio_out"] != t.uio_out:
            bad("uio_out", o["uio_out"], t.uio_out)
        mask = 0xFF if self.h.spi_sel(k) == 0 else 0xFE
        if (o["uo_out"] & mask) != (t.uo_out & mask):
            bad("uo_out (mask 0x%02X)" % mask, o["uo_out"] & mask, t.uo_out & mask)
        pad_uio, pad_ui = self.world.pads(t.cycle, t.uio_oe, t.uio_out, t.uo_out)
        self.fixed.uio, self.fixed.ui = pad_uio & 0xFF, pad_ui & 0xFF
        t.step(self.fixed)
        self.compared += 1
        self.last_k = k
        return pad_uio & 0xFF, pad_ui & 0xFF

    def done(self):
        return self.S is not None and self.compared >= self.window


class Top:
    def __init__(self, adapter, seed=1):
        self.a = adapter
        self.k = -1
        self.hook = None                  # callable(k, outs) -> (uio_in, ui_in) or None
        self.cs_hist = []                 # pad cs_n value per cycle
        self.rng = random.Random(seed)
        self.last_outs = None
        self.mon = None

    # ------------------------------------------------------------ cycle level
    def spi_sel(self, k):
        return int(k >= 2 and self.cs_hist[k - 2] == 0)

    async def step(self, sck=0, mosi=0, cs_n=1):
        await self.a.next_cycle()
        self.k += 1
        o = self.a.outs()
        uio, ui_hi = 0xFF, 0xFF
        if self.mon is not None:
            r = self.mon.hook(self.k, o)
            if r is not None:
                uio, ui_hi = r
        ui = (ui_hi & 0xF8) | ((cs_n & 1) << 2) | ((mosi & 1) << 1) | (sck & 1)
        self.cs_hist.append(cs_n & 1)
        self.a.drive(ui, uio)
        self.last_outs = o
        return o

    async def idle(self, n):
        for _ in range(n):
            await self.step()

    async def reset(self, low=3):
        self.a.assert_reset()
        await self.idle(low)
        self.a.release_reset()
        await self.idle(6)

    # ------------------------------------------------------------ SPI master
    async def xfer(self, tx, half=8, setup=8, hold=8, gap=8, extra=0, nbits_last=8, on_rise=None,
                   mosi_early=False):
        """One SPI frame (mode 0, MSB first). Returns (rx, rises): rx[i] is the byte clocked in
        on MISO while tx[i] was clocked out (rx[0] = status byte); rises[i] is the cycle in
        which SCK was first driven high for bit 0 of byte i. `extra` adds 0..extra random cycles
        to every half period; `nbits_last` < 8 aborts the last byte early; on_rise = {byte index:
        callback(cycle)} is called in the cycle of that byte's last rising edge; mosi_early makes
        MOSI change one clock cycle after every SCK rising edge instead of at the falling edge
        (digital hold of one clock cycle)."""
        rng = self.rng
        rx, rises = [], []

        def hp():
            return half + (rng.randrange(extra + 1) if extra else 0)

        first = True
        first_bit = (tx[0] >> 7) & 1
        for _ in range(setup):
            await self.step(0, first_bit, 0)
        for bi, byte in enumerate(tx):
            v = 0
            nb = nbits_last if bi == len(tx) - 1 else 8
            for i in range(nb):
                bit = (byte >> (7 - i)) & 1
                if not first:
                    for _ in range(hp()):
                        await self.step(0, bit, 0)
                first = False
                n = hp()
                o = await self.step(1, bit, 0)
                v = (v << 1) | (o["uo_out"] & 1)
                if i == 7:
                    rises.append(self.k)
                    if on_rise and bi in on_rise:
                        on_rise[bi](self.k)
                for j in range(n - 1):
                    nxt = bit
                    if mosi_early:
                        nxt = (byte >> (6 - i)) & 1 if i < 7 else (tx[bi + 1] >> 7) & 1 if bi + 1 < len(tx) else 0
                    await self.step(1, nxt, 0)
            rx.append(v)
        for _ in range(hold):
            await self.step(0, 0, 0)
        for _ in range(gap):
            await self.step(0, 0, 1)
        return rx, rises

    # ------------------------------------------------------------ register helpers
    async def wr(self, addr, *data, **kw):
        rx, rises = await self.xfer([addr & 0x7F] + list(data), **kw)
        return rx, rises

    async def rd(self, addr, n=1, **kw):
        rx, _ = await self.xfer([0x80 | addr] + [0] * n, **kw)
        return rx[1:]

    async def rd1(self, addr, **kw):
        return (await self.rd(addr, 1, **kw))[0]

    async def status(self, **kw):
        rx, _ = await self.xfer([0x80 | R_STATUS], **kw)
        return rx[0]

    async def wait_status(self, mask, value, frames=40):
        for _ in range(frames):
            s = await self.status()
            if (s & mask) == value:
                return s
        raise TopError("status never reached 0x%02X/0x%02X (last 0x%02X)" % (value, mask, s))

    async def set_pc(self, pc):
        await self.wr(R_PC_LO, pc & 0xFF)
        await self.wr(R_PC_HI, (pc >> 8) & 1)

    async def set_saddr(self, a):
        await self.wr(R_SA_LO, a & 0xFF)
        await self.wr(R_SA_HI, (a >> 8) & 1)

    async def load_words(self, words, base=0, **kw):
        await self.set_saddr(base)
        data = []
        for w in words:
            data += [(w >> (8 * i)) & 0xFF for i in range(4)]
        await self.wr(R_SRAM, *data, **kw)

    async def read_words(self, n, base=0, **kw):
        await self.set_saddr(base)
        b = await self.rd(R_SRAM, 4 * n, **kw)
        return [b[4 * i] | (b[4 * i + 1] << 8) | (b[4 * i + 2] << 16) | (b[4 * i + 3] << 24)
                for i in range(n)]

    async def push_tx(self, data, **kw):
        return await self.wr(R_TX_PUSH, *data, **kw)

    async def pop_rx(self, n, **kw):
        return await self.rd(R_RX_POP, n, **kw)

    async def drain_rx(self):
        cnt = (await self.rd1(R_FIFO_CNT)) >> 4
        return list(await self.pop_rx(cnt)) if cnt else []

    async def start(self, pc, mon=None, extra_ctrl=0, **kw):
        """Load the PC and resume. Returns the cycle of the resume pulse's last SCK rise."""
        await self.set_pc(pc)
        cb = {1: (lambda k: mon.set_start(k + 4))} if mon else None
        rx, rises = await self.xfer([R_CTRL, CTRL_PC_LOAD | CTRL_RESUME | extra_ctrl],
                                    on_rise=cb, **kw)
        return rises[1]

    async def run_until(self, k_end):
        while self.k < k_end:
            await self.step()


# ================================================================== tests
async def t_reset_and_registers(h):
    await h.reset()
    o = h.last_outs
    assert o["uio_oe"] == 0 and o["uio_out"] == 0 and o["uo_out"] == 0, o
    rx, _ = await h.xfer([0x80 | R_ID, 0])
    assert rx[0] == ST_HALTED | ST_MEM_FREE | ST_TX_EMPTY | ST_RX_EMPTY, "status 0x%02X" % rx[0]
    assert rx[1] == ID_VALUE, "ID 0x%02X" % rx[1]
    # reset values of the readable registers
    for reg, want in ((R_STATUS, 0x2D), (R_CTRL, 0), (R_PC_LO, 0), (R_PC_HI, 0), (R_FIFO_CNT, 0),
                      (R_RX_OVR, 0), (R_SCRATCH, 0), (R_SA_LO, 0), (R_SA_HI, 0), (R_ERR, 0)):
        got = await h.rd1(reg)
        assert got == want, "reg 0x%02X reset value 0x%02X != 0x%02X" % (reg, got, want)
    # scratch: write / read, every payload byte is a write (last wins), reads repeat
    await h.wr(R_SCRATCH, 0xA5)
    assert await h.rd(R_SCRATCH, 3) == [0xA5] * 3
    await h.wr(R_SCRATCH, 0x11, 0x22, 0x33)
    assert await h.rd1(R_SCRATCH) == 0x33
    # 9 bit registers
    await h.wr(R_PC_LO, 0xFE)
    await h.wr(R_PC_HI, 0xFF)
    assert await h.rd1(R_PC_LO) == 0xFE and await h.rd1(R_PC_HI) == 1
    await h.wr(R_SA_LO, 0x34)
    await h.wr(R_SA_HI, 0x01)
    assert await h.rd1(R_SA_LO) == 0x34 and await h.rd1(R_SA_HI) == 1
    # an aborted write frame (cut in the middle of the payload byte) changes nothing
    await h.xfer([R_SCRATCH, 0xFF], nbits_last=5)
    assert await h.rd1(R_SCRATCH) == 0x33
    # a frame that is cut in the middle of the command byte is ignored and the next frame works
    await h.xfer([0x80 | R_ID], nbits_last=3)
    assert await h.rd1(R_ID) == ID_VALUE
    # rejected commands: unmapped address, wrong direction
    assert await h.rd1(R_ERR) == 0
    assert await h.rd(0x40, 2) == [0, 0]                  # read of an unmapped register
    assert await h.rd1(R_ERR) == ERR_BAD_CMD
    await h.wr(R_ERR, ERR_BAD_CMD)                        # write 1 to clear
    assert await h.rd1(R_ERR) == 0
    await h.wr(R_ID, 0xFF)                                # write to a read-only register
    await h.wr(0x41, 0xFF)                                # write to an unmapped register
    assert await h.rd(R_TX_PUSH, 1) == [0]                # read of a write-only register
    assert await h.rd1(R_ERR) == ERR_BAD_CMD
    assert await h.rd1(R_ID) == ID_VALUE and await h.rd1(R_SCRATCH) == 0x33
    await h.wr(R_ERR, 0x07)
    assert await h.rd1(R_ERR) == 0
    # status byte as the first MISO byte of every frame equals the STATUS register
    rx, _ = await h.xfer([0x80 | R_STATUS, 0])
    assert rx[0] == rx[1] == 0x2D


async def t_spi_timing(h):
    """Different SPI clock periods, jitter and CS timing; the SRAM window stresses the prefetch."""
    await h.reset()
    rng = random.Random(5)
    words = [rng.randrange(1 << 32) for _ in range(12)]
    await h.load_words(words)
    for half, extra, setup, hold, gap in ((8, 0, 8, 8, 8), (8, 3, 8, 8, 8), (9, 1, 4, 8, 4),
                                          (16, 0, 8, 8, 8), (8, 0, 4, 8, 4), (11, 5, 6, 3, 5)):
        v = rng.randrange(256)
        kw = dict(half=half, extra=extra, setup=setup, hold=hold, gap=gap)
        await h.wr(R_SCRATCH, v, **kw)
        assert await h.rd1(R_SCRATCH, **kw) == v, kw
        await h.set_saddr(0)
        b = await h.rd(R_SRAM, 4 * len(words), **kw)
        got = [b[4 * i] | (b[4 * i + 1] << 8) | (b[4 * i + 2] << 16) | (b[4 * i + 3] << 24)
               for i in range(len(words))]
        assert got == words, kw
        assert await h.rd1(R_ID, **kw) == ID_VALUE
    # MOSI changing one clock after the SCK rising edge (1 cycle hold) is sampled correctly
    for half in (8, 10):
        await h.wr(R_SCRATCH, 0x96, half=half, mosi_early=True)
        assert await h.rd1(R_SCRATCH) == 0x96
        await h.wr(R_SCRATCH, 0x69, 0x5A, half=half, mosi_early=True)
        assert await h.rd1(R_SCRATCH) == 0x5A
    assert await h.rd1(R_ERR) == 0


async def t_sram_window(h):
    await h.reset()
    rng = random.Random(77)
    # burst load, burst readback (pipelined prefetch), single word reads
    img = [rng.randrange(1 << 32) for _ in range(40)]
    await h.load_words(img)
    assert await h.rd1(R_SA_LO) == 40 and await h.rd1(R_SA_HI) == 0      # auto increment
    assert await h.read_words(40) == img
    assert await h.rd1(R_SA_LO) == 40
    for a in (0, 7, 39):
        assert await h.read_words(1, base=a) == [img[a]]
    # an incomplete word at the end of a frame is discarded
    await h.set_saddr(5)
    await h.wr(R_SRAM, 0xDE, 0xAD, 0xBE)
    assert await h.rd1(R_SA_LO) == 5
    assert await h.read_words(1, base=5) == [img[5]]
    # a word that was started and cut in the middle of a byte is not written either
    await h.set_saddr(6)
    await h.xfer([R_SRAM, 1, 2, 3, 4], nbits_last=6)
    assert await h.read_words(1, base=6) == [img[6]]
    # reading 3 bytes does not advance the address, reading 5 does by one word
    await h.set_saddr(10)
    b = await h.rd(R_SRAM, 3)
    assert b == [(img[10] >> (8 * i)) & 0xFF for i in range(3)]
    assert await h.rd1(R_SA_LO) == 10
    b = await h.rd(R_SRAM, 5)
    assert b == [(img[10] >> (8 * i)) & 0xFF for i in range(4)] + [img[11] & 0xFF]
    assert await h.rd1(R_SA_LO) == 11
    # a command byte only (no payload) must not advance anything either
    await h.set_saddr(20)
    await h.xfer([0x80 | R_SRAM])
    assert await h.rd1(R_SA_LO) == 20
    # address wrap 511 -> 0
    edge = [0x11223344, 0x55667788, 0x99AABBCC]
    await h.load_words(edge[:2], base=511)
    assert await h.rd1(R_SA_LO) == 1 and await h.rd1(R_SA_HI) == 0
    assert await h.read_words(1, base=511) == [edge[0]]
    assert await h.read_words(1, base=0) == [edge[1]]
    await h.set_saddr(510)
    b = await h.rd(R_SRAM, 12)
    w = [b[4 * i] | (b[4 * i + 1] << 8) | (b[4 * i + 2] << 16) | (b[4 * i + 3] << 24) for i in range(3)]
    assert w[1] == edge[0] and w[2] == edge[1], w
    assert await h.rd1(R_ERR) == 0


async def t_fifo_host_side(h):
    await h.reset()
    # TX FIFO: push, count, full, drop with ERR, flush
    await h.push_tx([1, 2, 3, 4, 5])
    st = await h.status()
    assert not st & ST_TX_EMPTY and not st & ST_TX_FULL
    assert (await h.rd1(R_FIFO_CNT)) & 0x0F == 5
    await h.push_tx([6, 7, 8])
    st = await h.status()
    assert st & ST_TX_FULL and (await h.rd1(R_FIFO_CNT)) & 0x0F == 8
    assert await h.rd1(R_ERR) == 0
    await h.push_tx([9])                                   # dropped
    assert await h.rd1(R_ERR) == ERR_TX_DROP
    assert (await h.rd1(R_FIFO_CNT)) & 0x0F == 8
    await h.wr(R_CTRL, CTRL_TX_FLUSH)
    st = await h.status()
    assert st & ST_TX_EMPTY and not st & ST_TX_FULL
    assert (await h.rd1(R_FIFO_CNT)) & 0x0F == 0
    await h.wr(R_ERR, ERR_TX_DROP)
    # echo program: TX bytes go through HPULL / HPUSH into the RX FIFO
    src = """
        LDI c0, 6 @2
        NOP @5
    loop:
        HPULL @2
        XORI sr, 0x80 @2
        HPUSH @2
        DJNZ c0, loop @2
        HALT
    """
    words = trim(scn.asm_text(src))
    await h.load_words(words)
    data = [0x10, 0x20, 0x30, 0x7F, 0x00, 0xFF]
    await h.push_tx(data)
    await h.start(0)
    await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert not (await h.status()) & ST_ILLEGAL
    assert (await h.rd1(R_FIFO_CNT)) == 0x60               # 6 in RX, 0 in TX
    # reading exactly one byte pops exactly one
    assert await h.pop_rx(1) == [data[0] ^ 0x80]
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 5
    # command byte only: nothing is popped
    await h.xfer([0x80 | R_RX_POP])
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 5
    # a byte counts as consumed once the master has sampled its MSB (one SCK pulse is enough)
    rx, _ = await h.xfer([0x80 | R_RX_POP, 0], nbits_last=1)
    assert rx[1] == (data[1] ^ 0x80) >> 7
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 4
    data = [data[0], None] + data[2:]
    # three more bytes in order, then the rest
    assert await h.pop_rx(2) == [b ^ 0x80 for b in data[2:4]]
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 2
    # reading past the end returns 0x00 without popping or flagging anything
    assert await h.pop_rx(4) == [data[4] ^ 0x80, data[5] ^ 0x80, 0, 0]
    st = await h.status()
    assert st & ST_RX_EMPTY and (await h.rd1(R_FIFO_CNT)) == 0
    assert await h.rd1(R_ERR) == 0


async def t_rx_overrun(h):
    await h.reset()
    n = 11
    src = "\n".join(["LDI sr, 0x%02X @2\nHPUSH @2" % (0x40 + i) for i in range(n)]) + "\nHALT"
    await h.load_words(trim(scn.asm_text(src)))
    await h.start(0)
    await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    st = await h.status()
    assert st & ST_RX_FULL and st & ST_OVR and not st & ST_RX_EMPTY
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 8
    assert await h.rd1(R_RX_OVR) == n - 8
    assert await h.pop_rx(8) == [0x40 + i for i in range(8)]
    assert await h.rd1(R_RX_OVR) == n - 8                  # popping does not clear it
    await h.wr(R_CTRL, CTRL_RX_FLUSH)
    st = await h.status()
    assert not st & ST_OVR and st & ST_RX_EMPTY
    assert await h.rd1(R_RX_OVR) == 0


async def t_run_control(h):
    await h.reset()
    # illegal opcode: sticky illegal + halt, cleared by resume, PC points behind it
    tail = scn.asm_text("LDI sr, 0x5A @2\nHPUSH @2\nHALT")[:3]
    words = trim([0xF0000000] + tail)
    await h.load_words(words)
    await h.start(0)
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert st & ST_ILLEGAL, "illegal flag"
    assert (await h.rd1(R_FIFO_CNT)) >> 4 == 0
    await h.wr(R_CTRL, CTRL_RESUME)                        # continue behind the illegal word
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE | ST_ILLEGAL, ST_HALTED | ST_MEM_FREE)
    assert await h.pop_rx(1) == [0x5A]
    # resume while the core is running is ignored; resume also acts only when halted+mem_free
    await h.wr(R_CTRL, CTRL_RX_FLUSH)

    # soft halt of an endless loop that toggles uio0 (exact pins until the halt takes effect)
    src = """
        PINS high uio0 @2
    loop:
        PINS toggle uio0 @20
        JMP loop @20
    """
    words = trim(scn.asm_text(src))
    await h.load_words(words)
    mon = Monitor(h, words, W.BusWorld(), window=10 ** 9, name="toggle")
    h.mon = mon
    await h.start(0, mon)
    await h.idle(300)
    st = await h.status()
    assert not st & ST_HALTED and not st & ST_MEM_FREE
    # SRAM window while running: rejected, reads 0, flagged, the program is not disturbed
    assert await h.rd(R_SRAM, 4) == [0, 0, 0, 0]
    assert await h.rd1(R_ERR) == ERR_SRAM_BUSY
    await h.wr(R_SRAM, 1, 2, 3, 4)
    await h.wr(R_ERR, ERR_SRAM_BUSY)
    assert await h.rd1(R_ERR) == 0
    # halt: an instruction committing in the halt cycle completes, then nothing more happens
    # the halt pulse is in cycle c + 3; an instruction committing there completes, so the pins
    # of cycle c + 4 still follow the ISS, nothing commits afterwards
    def stop_compare(c):
        mon.window = (c + 5) - mon.S
    _, rises = await h.wr(R_CTRL, CTRL_HALT, on_rise={1: stop_compare})
    c = rises[1]
    await h.run_until(c + 6)
    pins_at_halt = (h.last_outs["uio_oe"], h.last_outs["uio_out"])
    assert mon.compared == mon.window, (mon.compared, mon.window)
    h.mon = None
    for _ in range(150):
        o = await h.step()
        assert (o["uio_oe"], o["uio_out"]) == pins_at_halt, "pins changed after the halt"
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert not st & ST_ILLEGAL and pins_at_halt[0] == 1
    assert await h.read_words(1) == [words[0]]             # the SRAM window works again

    # halt wins over resume in the same write; resume without halted state does nothing
    await h.set_pc(0)
    await h.wr(R_CTRL, CTRL_PC_LOAD | CTRL_RESUME | CTRL_HALT)
    await h.idle(40)
    assert (await h.status()) & ST_HALTED
    # core_reset while halted releases the pins that the halted program left driven
    assert (await h.status()) & ST_HALTED
    await h.wr(R_CTRL, CTRL_CORE_RESET)
    await h.idle(20)
    o = h.last_outs
    assert o["uio_oe"] == 0 and o["uio_out"] == 0 and o["uo_out"] == 0, o
    # core_reset: pins released, SR cleared, illegal cleared
    words2 = trim(scn.asm_text("LDI sr, 0xA5 @2\nPINS high uio3 @2\nPINS high uo7 @2\nWAIT low uio5 @2\nHALT"))
    await h.load_words(words2)
    await h.start(0)
    await h.idle(100)
    o = h.last_outs
    assert o["uio_oe"] == 0x08 and o["uio_out"] == 0x08 and (o["uo_out"] & 0x80), o
    await h.wr(R_CTRL, CTRL_CORE_RESET)                     # stops the WAIT that never completes
    await h.idle(20)
    o = h.last_outs
    assert o["uio_oe"] == 0 and o["uio_out"] == 0 and o["uo_out"] == 0, o
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert not st & ST_ILLEGAL
    # after the reset the registers are really zero: HPUSH pushes SR = 0
    await h.load_words(trim(scn.asm_text("HPUSH @2\nHALT")))
    await h.start(0)
    await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert await h.pop_rx(1) == [0x00]
    # core_reset also clears the illegal flag
    await h.load_words([0xE0000000])
    await h.start(0)
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert st & ST_ILLEGAL
    await h.wr(R_CTRL, CTRL_CORE_RESET)
    assert not (await h.status()) & ST_ILLEGAL
    # soft halt of a core that waits forever
    await h.load_words(trim(scn.asm_text("WAIT low uio3 @2\nHALT")))
    await h.start(0)
    await h.idle(100)
    assert not (await h.status()) & ST_HALTED
    await h.wr(R_CTRL, CTRL_HALT)
    st = await h.wait_status(ST_HALTED | ST_MEM_FREE, ST_HALTED | ST_MEM_FREE)
    assert not st & ST_ILLEGAL


async def t_async_reset(h):
    await h.reset()
    src = "PINS high uio2 @2\nPINS high uo6 @2\nWAIT low uio7 @2\nHALT"
    words = trim(scn.asm_text(src))
    await h.load_words(words)
    await h.push_tx([1, 2, 3])
    await h.start(0)
    await h.idle(60)
    o = h.last_outs
    assert o["uio_oe"] == 4 and o["uio_out"] == 4 and (o["uo_out"] & 0x40), o
    # asynchronous assertion: outputs are released without waiting for a clock edge
    h.a.assert_reset()
    await h.a.settle()                      # no clock edge, only simulation time for the write
    o = h.a.outs()
    assert o["uio_oe"] == 0 and o["uio_out"] == 0 and (o["uo_out"] & 0xFE) == 0, o
    await h.idle(4)
    h.a.release_reset()
    await h.idle(6)
    assert (await h.status()) == ST_HALTED | ST_MEM_FREE | ST_TX_EMPTY | ST_RX_EMPTY
    assert await h.rd1(R_ID) == ID_VALUE
    # the SRAM content survives a reset
    assert await h.read_words(len(words)) == words


async def _example_run(h, name, words, world, tx, window, post, rx_drain=False, verify=False):
    await h.reset()
    await h.load_words(trim(words))
    if verify:
        assert await h.read_words(len(trim(words))) == trim(words), "readback after load"
    if tx:
        await h.push_tx(tx)
    mon = Monitor(h, words, world, host_tx=list(tx), window=window, name=name)
    h.mon = mon
    await h.start(0, mon)
    got = []
    while not mon.done():
        if rx_drain:
            got += await h.drain_rx()
        else:
            await h.step()
    h.mon = None
    got += await h.drain_rx() if rx_drain else []
    await post(mon, got)
    return mon


async def t_example_uart_tx(h):
    data = [0x55, 0xA5, 0x00, 0xFF, 0x01, 0x80]
    world = W.BusWorld()
    sc = scn.scn_uart_tx(20, data)

    async def post(mon, got):
        frames = W.decode_uart(world.levels(1), 20)
        assert [f[1] for f in frames] == data, frames
        assert all(f[2] for f in frames)
        assert not (await h.status()) & ST_ILLEGAL
        assert (await h.status()) & ST_TX_EMPTY
    await _example_run(h, "uart_tx", sc.words, world, data, sc.cycles, post, verify=True)


async def t_example_uart_rx(h):
    bit, data = 100, [0x00, 0xFF, 0xA6, 0x5A]
    frames = [(1000 + i * 10 * bit, b) for i, b in enumerate(data)]
    world = W.BusWorld([W.UartSource(2, bit, frames)])
    words = scn.example("uart_rx.asm", BIT=bit)

    async def post(mon, got):
        assert got == data, got
        assert await h.rd1(R_RX_OVR) == 0
    await _example_run(h, "uart_rx", words, world, [], 1000 + (len(data) * 10 + 2) * bit, post,
                       rx_drain=True)


async def t_example_uart_rx_overrun(h):
    bit, data = 20, list(range(0x30, 0x3C))
    frames = [(1000 + i * 10 * bit, b) for i, b in enumerate(data)]
    world = W.BusWorld([W.UartSource(2, bit, frames)])
    words = scn.example("uart_rx.asm", BIT=bit)

    async def post(mon, got):
        st = await h.status()
        assert st & ST_RX_FULL and st & ST_OVR
        assert await h.rd1(R_RX_OVR) == len(data) - 8
        assert await h.pop_rx(8) == data[:8]
        await h.wr(R_CTRL, CTRL_RX_FLUSH)
        assert await h.rd1(R_RX_OVR) == 0 and not (await h.status()) & ST_OVR
    await _example_run(h, "uart_rx_overrun", words, world, [], 1000 + (len(data) * 10 + 2) * bit, post)


async def t_example_spi_master(h):
    mosi, miso = [0xA5, 0x0F, 0xF0], [0xC3, 0x5A, 0x01]
    slave = W.SpiSlave(0, 0, tx_bytes=miso)
    sc = scn.scn_spi(0, 0, 24, mosi, miso)
    tx = [len(mosi)] + mosi

    async def post(mon, got):
        assert slave.rx == mosi, slave.rx
        assert len(slave.cs_cycles) == 1
        assert await h.drain_rx() == miso
    await _example_run(h, "spi_master", sc.words, W.BusWorld([slave]), tx, sc.cycles, post)


async def t_example_i2c_master(h):
    slave = W.I2cSlave(0x50, read_bytes=[0xC1, 0x3E, 0x7F])
    sc = scn.scn_i2c_write_read(75, 50, 11000)
    tx = [0xA0, 2, 0x11, 0x22, 0xA1, 3]

    async def post(mon, got):
        assert slave.written[0] == [0x11, 0x22], slave.written
        assert slave.starts == 2 and slave.stops == 2
        assert await h.drain_rx() == [0xC1, 0x3E, 0x7F]
    await _example_run(h, "i2c_master", sc.words, W.BusWorld([slave]), tx, sc.cycles, post)


async def t_spi_during_run(h):
    """Host traffic (status polls, TX pushes, FIFO reads, rejected SRAM access) while the UART
    transmitter runs: the uio pins must stay cycle exact, uo_out[0] shows MISO only during
    frames."""
    data = [0x55, 0xA5, 0x00, 0xFF, 0x01, 0x80]
    world = W.BusWorld()
    sc = scn.scn_uart_tx(20, data)
    await h.reset()
    words = trim(sc.words)
    await h.load_words(words)
    await h.push_tx(data[:3])
    mon = Monitor(h, sc.words, world, host_tx=data[:3], window=7000, name="spi_during_run")
    h.mon = mon
    await h.start(0, mon)
    # push the remaining bytes one by one while polling; the push lands in the TX FIFO one
    # cycle after the resume-style pulse (rise + 3), i.e. HPULL can see it from rise + 4
    for b in data[3:]:
        assert (await h.rd1(R_ID)) == ID_VALUE
        _, rises = await h.wr(R_TX_PUSH, b, on_rise={1: lambda k, b=b: mon.add_tx(k + 4, b)})
        await h.status()
        assert await h.rd(R_SRAM, 2) == [0, 0]
        await h.wr(R_ERR, ERR_SRAM_BUSY)
    await h.run_until(mon.S + mon.window + 2)
    h.mon = None
    frames = W.decode_uart(world.levels(1), 20)
    assert [f[1] for f in frames] == data, frames
    assert await h.rd1(R_ERR) == 0
