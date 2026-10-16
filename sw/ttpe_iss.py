#!/usr/bin/env python3
"""
ttpe_iss.py - cycle-accurate instruction-set simulator for TTPE ISA v0.2.

This is the GOLDEN TIMING MODEL for the Execution Unit RTL.  It models the fetch
controller (ttpe_sram_ctrl) as verified by the cocotb suite:

    * core_run asserted in cycle 0           -> first instruction valid in cycle 3
    * instruction accepted in cycle X        -> next sequential instruction valid in X+3
    * pc_load asserted in cycle X (jump)     -> target valid in X+3
    * valid instructions are held until accepted

Execution-unit rules (these are SPEC, the RTL must match them):
    * The unit accepts instruction N (loading it into CUR) in cycle A, in which the
      fetch controller presents it AND (CUR is empty OR CUR commits in this very cycle).
      It never accepts in a cycle in which it asserts pc_load or halts.
    * Countdown: in cycles A+1 ... the delta counter is checked; the action COMMITS in
      the first cycle >= A+1+delta in which the instruction's own condition holds
      (WAIT condition true, HPULL data available).  => commit(N) = A + 1 + delta
      for unconditional instructions.
    * Commit in cycle C updates registers at the edge ending C; the new pin state is
      visible in cycle C+1.
    * Inputs seen by an instruction committing in cycle C are 2-FF synchronised pad
      values: sync2(C) = pad(C-2).  Edge detection compares sync2(C) with sync2(C-1).

Everything is integer cycle math.  No floats anywhere.
"""

import ttpe_isa as isa

OP = isa.OPCODES


class IssError(Exception):
    pass


class PullWorld:
    """Trivial world: pad = driven value, or `pull` bit when not driven; ui = `ui`."""

    def __init__(self, pull=0x00, ui=0x00):
        self.pull, self.ui = pull, ui

    def pads(self, cycle, uio_oe, uio_out, uo_out):
        uio = (uio_out & uio_oe) | (self.pull & ~uio_oe & 0xFF)
        return uio & 0xFF, self.ui & 0xFF


class Ttpe:
    def __init__(self, words, start_pc=0, host_tx=None, rx_depth=8):
        if len(words) != isa.PROG_WORDS:
            words = list(words) + [isa.encode(OP["HALT"])] * (isa.PROG_WORDS - len(words))
        self.mem = list(words)
        # host -> engine bytes: list of ints (available from cycle 0) or (cycle, byte)
        self.tx = []
        for e in (host_tx or []):
            self.tx.append(e if isinstance(e, tuple) else (0, e))
        self.tx.sort(key=lambda t: t[0])
        self.tx_idx = 0
        self.rx = []                # (cycle, byte) pushed by HPUSH
        self.rx_depth = rx_depth
        self.rx_overrun = 0

        # architectural state
        self.sr = self.c0 = self.c1 = 0
        self.uio_oe = self.uio_out = self.uo_out = 0
        self.illegal = False
        self.halted = False

        # pipeline model
        self.fetch_pc = start_pc & 0x1FF
        self.fetch_valid_at = isa.FETCH_LATENCY       # core_run asserted in cycle 0
        self.cur = None
        self.cycle = 0

        # input synchroniser (reset to 0 like the RTL flops)
        self.s1 = 0
        self.s2 = 0
        self.s2_prev = 0

        # observability (used by tests and later as RTL reference)
        self.commits = []           # (commit_cycle, pc, word)
        self.accepts = []           # (accept_cycle, pc)
        self.pin_events = [(0, 0, 0, 0)]   # (effective_cycle, uio_oe, uio_out, uo_out)

    # ----------------------------------------------------------- helpers
    def _log_pins(self, eff_cycle):
        ev = (eff_cycle, self.uio_oe, self.uio_out, self.uo_out)
        if ev[1:] != self.pin_events[-1][1:]:
            self.pin_events.append(ev)

    def _tx_available(self, k):
        return self.tx_idx < len(self.tx) and self.tx[self.tx_idx][0] <= k

    # ------------------------------------------------------------ commit
    def _commit(self, c, k):
        """Try to commit instruction dict c in cycle k.
        Returns (done, pc_load_target_or_None)."""
        op, a, p = c["op"], c["a"], c["p"]
        s2, prev = self.s2, self.s2_prev

        if not isa.is_legal(op, a, p):
            self.illegal = True
            self.halted = True
            return True, None

        n = isa.OP_NAMES[op]
        if n == "NOP":
            return True, None

        if n == "PINS":
            mode, bank = a & 3, (a >> 3) & 1
            if bank == 0:
                if mode == isa.PINS_LOW:
                    self.uio_oe |= p
                    self.uio_out &= ~p & 0xFF
                elif mode == isa.PINS_HIGH:
                    self.uio_oe |= p
                    self.uio_out |= p
                elif mode == isa.PINS_FLOAT:
                    self.uio_oe &= ~p & 0xFF
                else:
                    self.uio_oe |= p
                    self.uio_out ^= p
            else:
                if mode == isa.PINS_LOW:
                    self.uo_out &= ~p & 0xFF
                elif mode == isa.PINS_HIGH:
                    self.uo_out |= p
                elif mode == isa.PINS_TOGGLE:
                    self.uo_out ^= p
                # float on uo bank: no effect
            self._log_pins(k + 1)
            return True, None

        if n == "WAIT":
            cur_b = (s2 >> a) & 1
            prev_b = (prev >> a) & 1
            t = p & 3
            ok = ((t == isa.WAIT_LOW and cur_b == 0) or
                  (t == isa.WAIT_HIGH and cur_b == 1) or
                  (t == isa.WAIT_FALL and prev_b == 1 and cur_b == 0) or
                  (t == isa.WAIT_RISE and prev_b == 0 and cur_b == 1))
            return ok, None

        if n == "JMP":
            return True, ((a & 1) << 8) | p

        if n in ("JHI", "JLO"):
            b = (s2 >> a) & 1
            taken = (b == 1) if n == "JHI" else (b == 0)
            return True, ((c["pc"] & 0x100) | p) if taken else None

        if n == "SHOUT":
            msb = p & isa.SHOUT_MSB
            bit = ((self.sr >> 7) & 1) if msb else (self.sr & 1)
            if a < 8:
                m = 1 << a
                if p & isa.SHOUT_OD:
                    self.uio_out &= ~m & 0xFF
                    if bit:
                        self.uio_oe &= ~m & 0xFF
                    else:
                        self.uio_oe |= m
                else:
                    self.uio_oe |= m
                    self.uio_out = (self.uio_out | m) if bit else (self.uio_out & ~m & 0xFF)
            else:
                m = 1 << (a - 8)
                self.uo_out = (self.uo_out | m) if bit else (self.uo_out & ~m & 0xFF)
            if p & isa.SHOUT_SHIFT:
                self.sr = ((self.sr << 1) & 0xFF) if msb else (self.sr >> 1)
            self._log_pins(k + 1)
            return True, None

        if n == "SAMPLE":
            b = (s2 >> a) & 1
            if p & isa.SAMPLE_MSB:
                self.sr = ((self.sr << 1) | b) & 0xFF
            else:
                self.sr = (self.sr >> 1) | (b << 7)
            return True, None

        if n == "REG":
            sub = isa.REG_NAMES[a]
            if sub == "LDSR":
                self.sr = p
            elif sub == "LDC0":
                self.c0 = p
            elif sub == "LDC1":
                self.c1 = p
            elif sub == "MVC0":
                self.c0 = self.sr
            elif sub == "MVC1":
                self.c1 = self.sr
            elif sub == "MVSR0":
                self.sr = self.c0
            elif sub == "MVSR1":
                self.sr = self.c1
            elif sub == "XORI":
                self.sr ^= p
            elif sub == "ANDI":
                self.sr &= p
            elif sub == "ORI":
                self.sr |= p
            elif sub == "ADDI":
                self.sr = (self.sr + p) & 0xFF
            return True, None

        if n == "DJNZ":
            if a & 1:
                self.c1 = (self.c1 - 1) & 0xFF
                nz = self.c1 != 0
            else:
                self.c0 = (self.c0 - 1) & 0xFF
                nz = self.c0 != 0
            return True, ((c["pc"] & 0x100) | p) if nz else None

        if n == "JSB":
            bit = (self.sr >> (a & 7)) & 1
            taken = bit == ((a >> 3) & 1)
            return True, ((c["pc"] & 0x100) | p) if taken else None

        if n == "HPUSH":
            if len(self.rx) < self.rx_depth:
                self.rx.append((k, self.sr))
            else:
                self.rx_overrun += 1
            return True, None

        if n == "HPULL":
            if not self._tx_available(k):
                return False, None
            self.sr = self.tx[self.tx_idx][1] & 0xFF
            self.tx_idx += 1
            return True, None

        if n == "HALT":
            self.halted = True
            return True, None

        raise IssError("unreachable opcode %r" % n)   # pragma: no cover

    # --------------------------------------------------------------- run
    def step(self, world):
        k = self.cycle
        pad_uio, pad_ui = world.pads(k, self.uio_oe, self.uio_out, self.uo_out)
        committing = False
        pc_load = None

        if self.cur is not None:
            c = self.cur
            if c["timer"] > 0:
                c["timer"] -= 1
            else:
                done, tgt = self._commit(c, k)
                if done:
                    committing = True
                    pc_load = tgt
                    self.commits.append((k, c["pc"], c["word"]))
                    self.cur = None

        if pc_load is not None:
            self.fetch_pc = pc_load & 0x1FF
            self.fetch_valid_at = k + isa.FETCH_LATENCY
        elif not self.halted and (self.cur is None or committing) \
                and self.fetch_valid_at <= k:
            word = self.mem[self.fetch_pc]
            op, a, p, d = isa.decode(word)
            self.cur = dict(pc=self.fetch_pc, word=word, op=op, a=a, p=p, timer=d)
            self.accepts.append((k, self.fetch_pc))
            self.fetch_pc = (self.fetch_pc + 1) & 0x1FF
            self.fetch_valid_at = k + isa.FETCH_LATENCY

        # synchroniser update (end of cycle k)
        self.s2_prev = self.s2
        self.s2 = self.s1
        self.s1 = (pad_uio & 0xFF) | ((pad_ui & 0xFF) << 8)
        self.cycle = k + 1

    def run(self, world=None, max_cycles=100000):
        """Run until HALT commits or max_cycles elapse. Returns self."""
        world = world or PullWorld()
        end = self.cycle + max_cycles
        while self.cycle < end and not self.halted:
            self.step(world)
        return self

    def run_cycles(self, world, n):
        """Run exactly n more cycles (even past HALT is not allowed)."""
        for _ in range(n):
            if self.halted:
                break
            self.step(world)
        return self

    def resume(self, start_pc=None):
        """Host restarts the core (optionally loading a new PC). Pipeline restarts like
        core_run 0->1: first instruction valid 3 cycles later."""
        self.halted = False
        self.illegal = False
        self.cur = None
        if start_pc is not None:
            self.fetch_pc = start_pc & 0x1FF
        self.fetch_valid_at = self.cycle + isa.FETCH_LATENCY

    # convenience for tests
    def commit_cycle_of(self, pc, nth=0):
        hits = [c for c, p_, _ in self.commits if p_ == pc]
        return hits[nth]
