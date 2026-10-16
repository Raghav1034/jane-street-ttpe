"""
ttpe_lockstep.py - cycle-by-cycle lockstep checker: RTL (via an adapter) vs sw/ttpe_iss.py.

The engine is plain Python and knows nothing about cocotb. It talks to the design through a
small adapter (see `DutIface`), so the same code runs under cocotb on the real RTL and, in the
sandbox that has no simulator, against a Python transcription of the RTL.

Cycle convention (same as test_ttpe_mem.py)
-------------------------------------------
Cycle k starts at rising clock edge k. Inputs for cycle k are applied just after edge k and are
sampled by the flops at edge k+1. `state()` is read just after edge k (flop outputs of cycle k),
`comb()` is read at the falling edge in the middle of cycle k (flop outputs plus combinational
logic fed by the cycle-k inputs).

What is compared
----------------
Every cycle, before the ISS is stepped:
    uio_oe, uio_out, uo_out, SR, C0, C1, halted, illegal,
    TX/RX FIFO counts and full/empty flags, RX overrun count, RX head byte.
Every cycle, after the ISS step:
    instr_accept  <-> ISS accept (and the accepted PC),
    commit        <-> ISS commit (and the committed PC / word),
    the fetched word equals the memory image at the accepted PC,
    host_gnt == host_req & mem_free, host write/read acknowledgements and read data.
At the end of a run:
    full accept list, commit list and pin-event list equal the ISS lists.

Time bases
----------
`wk`      world time: counts every cycle from the first cycle in which the core runs (devices
          in ttpe_world.py see this number, so UART frames etc. are scheduled in it).
ISS time  `iss.cycle`: stops while the core is halted, restarts at host_resume. All log
          entries (accepts, commits, pin events) are in ISS time, so they are directly
          comparable with ISS lists.
"""

import ttpe_isa as isa

CLK_NS = 20
MAX_TX = 8
MAX_RX = 8


class LockstepError(AssertionError):
    pass


class FixedWorld:
    """World handed to the ISS: returns the pads that were already applied to the RTL."""

    def __init__(self):
        self.uio, self.ui = 0xFF, 0xFF

    def pads(self, k, uio_oe, uio_out, uo_out):
        return self.uio, self.ui


class DutIface:
    """Adapter interface (documentation only). Implementations: cocotb (test_ttpe_core.py)
    and the Python RTL transcription used for sandbox self-checks."""

    async def next_cycle(self):
        """Advance to just after the next rising edge (flop outputs settled)."""

    def state(self):
        """dict: oe out uo sr c0 c1 halted illegal tx_count rx_count tx_full tx_empty
        rx_full rx_empty rx_overrun rx_rdata mem_free  (ints; raise on X)."""

    def drive(self, d):
        """Apply inputs for this cycle: uio_in ui_in host_req host_we host_addr host_wdata
        host_wstrb host_resume host_pc_load host_pc_val tx_push tx_wdata tx_fifo_flush
        rx_pop rx_fifo_flush."""

    async def mid_cycle(self):
        """Advance to the falling edge of this cycle."""

    def comb(self):
        """dict: accept commit pc instr pc_load host_gnt host_wdone host_rvalid host_rdata"""


class DefaultPolicy:
    """Host behaviour. Called every cycle with the engine and the flop state of that cycle.

    resume_at_start : pulse host_resume as soon as the core is halted and the memory is free
                      (once, at the beginning).
    resume_on_halt  : pulse host_resume `halt_gap` cycles after every halt (None = never); may be
                      a callable(ls) returning a start PC or None. max_resumes limits how often.
    start_pc        : PC loaded together with the first resume (None = keep).
    tx_bytes        : bytes pushed as soon as the TX FIFO has room (starting at world cycle
                      tx_start).
    tx_timed        : [(wk, byte)] pushes at an exact world cycle (dropped if the FIFO is full).
    rx_drain        : pop the RX FIFO whenever it is not empty.
    """

    def __init__(self, resume_at_start=True, resume_on_halt=None, halt_gap=8, start_pc=None,
                 tx_bytes=(), tx_start=0, tx_timed=(), rx_drain=True, extra=None,
                 max_resumes=None):
        self.resume_at_start = resume_at_start
        self.resume_on_halt = resume_on_halt
        self.halt_gap = halt_gap
        self.start_pc = start_pc
        self.tx_bytes = list(tx_bytes)
        self.tx_start = tx_start
        self.tx_timed = dict(tx_timed)
        self.rx_drain = rx_drain
        self.extra = extra          # callable(ls, S) -> dict, merged last
        self.max_resumes = max_resumes   # limit for resume_on_halt resumes (None = no limit)
        self._n_resumes = 0
        self._started = False
        self._halt_seen = None

    def __call__(self, ls, S):
        a = {}
        if S["halted"] and S["mem_free"]:
            if not self._started and self.resume_at_start:
                a["resume"] = True
                if self.start_pc is not None:
                    a["pc_load"] = self.start_pc
                self._started = True
            elif (self._started and self.resume_on_halt is not None and
                  (self.max_resumes is None or self._n_resumes < self.max_resumes)):
                if self._halt_seen is None:
                    self._halt_seen = ls.k
                if ls.k - self._halt_seen >= self.halt_gap:
                    a["resume"] = True
                    if callable(self.resume_on_halt):
                        pc = self.resume_on_halt(ls)
                        if pc is not None:
                            a["pc_load"] = pc
                    self._halt_seen = None
                    self._n_resumes += 1
        if not S["halted"]:
            self._halt_seen = None
        wk = ls.wk                       # stays 0 until the first running cycle
        running_clock = ls.started or not S["halted"]
        if running_clock and wk in self.tx_timed:
            a["tx_push"] = self.tx_timed[wk]
        elif self.tx_bytes and wk >= self.tx_start and S["tx_count"] < MAX_TX:
            a["tx_push"] = self.tx_bytes.pop(0)
        if self.rx_drain and not S["rx_empty"]:
            a["rx_pop"] = True
        if self.extra:
            a.update(self.extra(ls, S) or {})
        return a


class Lockstep:
    def __init__(self, dut, words, world, policy=None, name=""):
        import ttpe_iss as iss_mod
        self.d = dut
        self.world = world
        self.policy = policy or DefaultPolicy()
        self.name = name
        words = list(words)
        if len(words) != isa.PROG_WORDS:
            words += [isa.encode(isa.OPCODES["HALT"])] * (isa.PROG_WORDS - len(words))
        self.iss = iss_mod.Ttpe(words)
        self.iss.halted = True              # reset state: halted until host_resume
        self.fixed = FixedWorld()
        self.k = 0                          # RTL cycle counter since the first lockstep cycle
        self.wk = 0                         # world time (see module doc)
        self.started = False
        # RTL-side logs (ISS time)
        self.rtl_accepts = []
        self.rtl_commits = []
        self.rtl_pin_events = [(0, 0, 0, 0)]
        self.rx_got = []                    # bytes popped by the host, in order
        self.rx_got_all = []                # (iss_cycle, byte)
        self.tx_pushed = []                 # bytes accepted by the TX FIFO
        self.tx_dropped = 0
        self.resumes = 0
        self.last_state = None
        self.halts = []                     # ISS cycles at which HALT/illegal commits occurred
        self.mem_reads = []                 # (addr, data) host read results
        self._cur = None                    # (pc, word) of the instruction held by the RTL
        self._rd_expect = {}                # cycle -> (addr, data)
        self._wd_expect = set()
        self._last_pins = (0, 0, 0)

    # ------------------------------------------------------------------ helpers
    def fail(self, msg):
        raise LockstepError("[%s] RTL cycle %d (world %d, ISS %d): %s" %
                            (self.name, self.k, self.wk, self.iss.cycle, msg))

    def eq(self, what, rtl, ref):
        if rtl != ref:
            self.fail("%s: RTL %r != ISS %r" % (what, rtl, ref))

    def _check_pre(self, S):
        t = self.iss
        self.eq("uio_oe", S["oe"], t.uio_oe)
        self.eq("uio_out", S["out"], t.uio_out)
        self.eq("uo_out", S["uo"], t.uo_out)
        self.eq("SR", S["sr"], t.sr)
        self.eq("C0", S["c0"], t.c0)
        self.eq("C1", S["c1"], t.c1)
        self.eq("halted", bool(S["halted"]), t.halted)
        self.eq("illegal", bool(S["illegal"]), t.illegal)
        txn = len(t.tx) - t.tx_idx
        self.eq("tx_count", S["tx_count"], txn)
        self.eq("tx_full", bool(S["tx_full"]), txn == MAX_TX)
        self.eq("tx_empty", bool(S["tx_empty"]), txn == 0)
        self.eq("rx_count", S["rx_count"], len(t.rx))
        self.eq("rx_full", bool(S["rx_full"]), len(t.rx) == MAX_RX)
        self.eq("rx_empty", bool(S["rx_empty"]), len(t.rx) == 0)
        self.eq("rx_overrun", S["rx_overrun"], min(t.rx_overrun, 255))
        if t.rx:
            self.eq("rx head byte", S["rx_rdata"], t.rx[0][1])
        pins = (S["oe"], S["out"], S["uo"])
        if pins != self._last_pins:
            self.rtl_pin_events.append((t.cycle,) + pins)
            self._last_pins = pins

    # ------------------------------------------------------------------ one cycle
    async def cycle(self):
        d, t = self.d, self.iss
        await d.next_cycle()
        S = d.state()
        self.last_state = S
        self._check_pre(S)
        act = self.policy(self, S) or {}

        # ---- pads
        if self.started or not S["halted"]:
            self.started = True
            pad_uio, pad_ui = self.world.pads(self.wk, S["oe"], S["out"], S["uo"])
            self.wk += 1
        else:
            pad_uio, pad_ui = 0xFF, getattr(self.world, "ui", 0xFF)
        self.fixed.uio, self.fixed.ui = pad_uio & 0xFF, pad_ui & 0xFF

        ctl_ok = bool(S["halted"]) and bool(S["mem_free"])
        resume = bool(act.get("resume"))
        pc_ld = act.get("pc_load")
        inp = dict(
            uio_in=pad_uio & 0xFF, ui_in=pad_ui & 0xFF,
            host_req=int(bool(act.get("host_req"))), host_we=int(bool(act.get("host_we"))),
            host_addr=act.get("host_addr", 0), host_wdata=act.get("host_wdata", 0),
            host_wstrb=act.get("host_wstrb", 0xF),
            host_resume=int(resume), host_pc_load=int(pc_ld is not None),
            host_pc_val=(pc_ld or 0) & 0x1FF,
            tx_push=int(act.get("tx_push") is not None), tx_wdata=(act.get("tx_push") or 0) & 0xFF,
            tx_fifo_flush=int(bool(act.get("tx_flush"))),
            rx_pop=int(bool(act.get("rx_pop"))),
            rx_fifo_flush=int(bool(act.get("rx_flush"))),
        )
        d.drive(inp)
        await d.mid_cycle()
        C = d.comb()

        # ---- host SRAM port expectations
        gnt_exp = bool(act.get("host_req")) and bool(S["mem_free"])
        self.eq("host_gnt", bool(C["host_gnt"]), gnt_exp)
        self.eq("host_wdone", bool(C["host_wdone"]), self.k in self._wd_expect)
        self._wd_expect.discard(self.k)
        if self.k in self._rd_expect:
            addr, data = self._rd_expect.pop(self.k)
            self.eq("host_rvalid", bool(C["host_rvalid"]), True)
            self.eq("host_rdata[%d]" % addr, C["host_rdata"], data)
            self.mem_reads.append((addr, data))
        else:
            self.eq("host_rvalid", bool(C["host_rvalid"]), False)

        # ---- ISS step
        iss_cycle = t.cycle
        stepped = not t.halted
        if stepped:
            na, nc = len(t.accepts), len(t.commits)
            t.step(self.fixed)
            new_acc, new_com = t.accepts[na:], t.commits[nc:]
        else:
            new_acc, new_com = [], []

        # commit first (it concerns the instruction held BEFORE this cycle's accept)
        self.eq("commit", bool(C["commit"]), bool(new_com))
        if new_com:
            cyc, pc, word = new_com[0]
            if self._cur is None:
                self.fail("RTL commit with nothing held")
            self.eq("commit pc", self._cur[0], pc)
            self.eq("commit word", self._cur[1], word)
            self.rtl_commits.append((iss_cycle, self._cur[0], self._cur[1]))
            self._cur = None
            if t.halted:
                self.halts.append(iss_cycle)
        self.eq("accept", bool(C["accept"]), bool(new_acc))
        if new_acc:
            cyc, pc = new_acc[0]
            self.eq("accept pc", C["pc"], pc)
            self.eq("fetched word", C["instr"], t.mem[pc])
            self.rtl_accepts.append((iss_cycle, C["pc"]))
            self._cur = (C["pc"], C["instr"])

        # ---- host effects on the ISS side (order: after the ISS step, see ttpe_fifo.sv)
        txf = bool(act.get("tx_flush"))
        if act.get("tx_push") is not None:
            if txf or S["tx_full"]:
                self.tx_dropped += 1
            else:
                t.tx.append((t.cycle, act["tx_push"] & 0xFF))
                self.tx_pushed.append(act["tx_push"] & 0xFF)
        if txf:
            del t.tx[t.tx_idx:]
        if bool(act.get("rx_flush")):
            t.rx.clear()
            t.rx_overrun = 0
        elif act.get("rx_pop") and not S["rx_empty"]:
            cyc, byte = t.rx.pop(0)
            self.eq("rx byte popped", S["rx_rdata"], byte)
            self.rx_got.append(byte)
            self.rx_got_all.append((iss_cycle, byte))

        # host SRAM access (only granted while the memory is free)
        if gnt_exp:
            a = act.get("host_addr", 0) & 0x1FF
            if act.get("host_we"):
                w = t.mem[a]
                st = act.get("host_wstrb", 0xF)
                nw = act.get("host_wdata", 0)
                for b in range(4):
                    if (st >> b) & 1:
                        w = (w & ~(0xFF << (8 * b))) | (nw & (0xFF << (8 * b)))
                t.mem[a] = w & 0xFFFFFFFF
                self._wd_expect.add(self.k + 1)
            else:
                self._rd_expect[self.k + 2] = (a, t.mem[a])

        if ctl_ok and resume:
            self.resumes += 1
            t.resume(pc_ld & 0x1FF if pc_ld is not None else None)
        elif ctl_ok and pc_ld is not None:
            t.fetch_pc = pc_ld & 0x1FF
        self.k += 1

    async def run(self, n):
        for _ in range(n):
            await self.cycle()

    async def run_until(self, pred, max_cycles):
        for _ in range(max_cycles):
            await self.cycle()
            if pred(self):
                return True
        return False

    # ------------------------------------------------------------------ final check
    def finish(self):
        """Compare the complete logs; returns the ISS for further assertions."""
        t = self.iss
        self.eq("accept list length", len(self.rtl_accepts), len(t.accepts))
        if self.rtl_accepts != t.accepts:
            for i, (a, b) in enumerate(zip(self.rtl_accepts, t.accepts)):
                if a != b:
                    self.fail("accept #%d RTL %r != ISS %r" % (i, a, b))
        self.eq("commit list length", len(self.rtl_commits), len(t.commits))
        if self.rtl_commits != t.commits:
            for i, (a, b) in enumerate(zip(self.rtl_commits, t.commits)):
                if a != b:
                    self.fail("commit #%d RTL %r != ISS %r" % (i, a, b))
        self.eq("pin events", self.rtl_pin_events, t.pin_events)
        return t
