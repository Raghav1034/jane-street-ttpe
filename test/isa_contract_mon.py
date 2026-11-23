"""
isa_contract_mon.py - Independent ISA v0.2 contract monitor derived solely from docs/ISA.md.

Zero imports from sw/ttpe_iss.py or sw/ttpe_isa.py: all opcode bitfields, timing rules,
synchroniser latencies, and next-cycle architectural state checks are transcribed directly
from docs/ISA.md sections 1-4.

Hooked into test/ttpe_lockstep.py so every TARGET=core lockstep scenario checks the RTL
directly against the written ISA specification on every clock cycle, independently of the ISS.

Contract rules checked (see docs/ISA.md):
  C1  Fetch / Accept / PC progression:
        - No accept or commit while halted.
        - Accept only when execution unit is empty or committing in the same cycle.
        - Never accept in a cycle that redirects (pc_load) or halts (HALT / illegal).
        - First accept after host_resume (while halted & mem_free) lands at k_resume + 4.
        - Sequential fetch latency: next accept at max(prev_accept + 3, prev_commit).
        - Taken branch redirect latency: target accept at branch_commit + 3.
        - Sequential PC wraps 9-bit (prev_pc + 1) & 0x1FF; JMP targets any page
          (((a & 1) << 8) | p); conditional branches target same 256-word page
          ((branch_pc & 0x100) | p).
  C2  Commit timing & spacing:
        - Never commit with no instruction held.
        - Unconditional instructions commit at exact cycle A + 1 + D.
        - Sequential D(N) >= 2 -> commit(N+1) == commit(N) + D(N+1) + 1 (unconditional N+1).
        - Sequential D(N) < 2  -> commit(N+1) == commit(N) + (2 - D(N)) + D(N+1) + 1.
        - Taken branch         -> commit(target) == commit(branch) + D(target) + 4 (3-cycle penalty).
        - Blocking WAIT / HPULL commit at first cycle >= A + 1 + D where condition holds.
  C3  Architectural state & pad output timing (docs/ISA.md sections 2 & 3 rule 3):
        - Registers and pin outputs update at the edge ending the commit cycle
          (visible on cycle commit + 1); unchanged on non-commit cycles.
        - Exact next-cycle uio_oe, uio_out, uo_out, SR, C0, C1, halted, illegal checks
          for NOP, PINS, WAIT, JMP, JHI, JLO, SHOUT, SAMPLE, REG, DJNZ, HPUSH, HPULL,
          HALT, JSB, and illegal opcodes/sub-ops.
  C4  2-FF input synchroniser timing (docs/ISA.md section 4):
        - Independent 2-FF + prev-cycle synchroniser clocked on every cycle from raw pads.
        - Pad change in cycle X is first observed in cycle X + 2 (WAIT/SAMPLE/JHI/JLO).
        - Virtual USB source pins ui4..ui7 (sources 12..15) are bypassed while usb_en == 1.
"""


class IsaContractError(AssertionError):
    """Raised when an RTL (or trace) step violates docs/ISA.md."""


# Opcode encodings from docs/ISA.md section 2 (4-bit OP = word[31:28])
OP_NOP    = 0x0
OP_PINS   = 0x1
OP_WAIT   = 0x2
OP_JMP    = 0x3
OP_JHI    = 0x4
OP_JLO    = 0x5
OP_SHOUT  = 0x6
OP_SAMPLE = 0x7
OP_REG    = 0x8
OP_DJNZ   = 0x9
OP_HPUSH  = 0xA
OP_HPULL  = 0xB
OP_HALT   = 0xC
OP_JSB    = 0xD

# Valid REG sub-ops from docs/ISA.md section 2 (A = word[27:24])
VALID_REG_SUBOPS = frozenset((0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11))


def decode_word(word):
    """Decode 32-bit instruction word per docs/ISA.md section 2."""
    w = word & 0xFFFFFFFF
    return (w >> 28) & 0xF, (w >> 24) & 0xF, (w >> 16) & 0xFF, w & 0xFFFF


def is_illegal_word(word):
    """True if opcode or REG sub-op is reserved/illegal per docs/ISA.md section 2."""
    op, a, _, _ = decode_word(word)
    if op in (0xE, 0xF):
        return True
    if op == OP_REG and a not in VALID_REG_SUBOPS:
        return True
    return False


class IsaContractMonitor:
    """Cycle-by-cycle passive monitor of docs/ISA.md contracts.

    Call sequence per RTL cycle k:
      1. pre_cycle(k, S)           -- flop outputs settled just after rising edge k
      2. post_cycle(k, S, inp, C)  -- inputs applied in cycle k + mid-cycle comb outputs C
    """

    def __init__(self, name="", init_sync=0x0000):
        self.name = name
        # Independent 2-FF input synchroniser + 1-cycle edge-detect history (docs/ISA.md §4)
        # Bit i = source pin i (0..7 = uio_in[0..7], 8..15 = ui_in[0..7]).
        init_sync &= 0xFFFF
        self._s1 = init_sync
        self._s2 = init_sync
        self._s3 = init_sync

        # Held instruction: (accept_cycle, pc, word) or None
        self._held = None
        # Expected next accept: (cycle, expected_pc, kind) or None
        # kind in ("resume", "seq", "branch")
        self._exp_accept = None
        # Context of previous commit for spacing consequence checks (docs/ISA.md §3 table)
        self._prev_commit_info = None
        # Host PC tracking for resume after HALT / reset / host_pc_load
        self._resume_pc = 0
        # Deferred next-cycle state expectations checked at pre_cycle(k + 1, S_next)
        self._exp_next = None

        # Telemetry
        self.cycles_checked = 0
        self.accepts_checked = 0
        self.commits_checked = 0

    def _fail(self, k, rule, msg):
        raise IsaContractError("[%s] cyc %d (%s): %s" % (self.name, k, rule, msg))

    def _src_bit(self, pin):
        return (self._s2 >> pin) & 1

    def _wait_cond(self, pin, wtype):
        cur = (self._s2 >> pin) & 1
        prv = (self._s3 >> pin) & 1
        if wtype == 0:    # low
            return cur == 0
        if wtype == 1:    # high
            return cur == 1
        if wtype == 2:    # fall
            return prv == 1 and cur == 0
        return prv == 0 and cur == 1  # rise (3)

    @staticmethod
    def _is_virtual_usb_pin(pin, usb_en):
        return bool(usb_en) and 12 <= pin <= 15

    # ------------------------------------------------------------------ step 1: pre-edge state
    def pre_cycle(self, k, S):
        """Verify cycle-k flop outputs S against expectations from cycle k-1 (Rule C3)."""
        if self._exp_next is None:
            return
        exp = self._exp_next
        self._exp_next = None
        for key, want in exp.items():
            if key == "_src" or want is None:
                continue
            got = S[key]
            if key in ("halted", "illegal"):
                got = int(bool(got))
                want = int(bool(want))
            if got != want:
                self._fail(
                    k, "C3-state",
                    "next-cycle %s mismatch: got 0x%X (%d), expected 0x%X (%d) after %s"
                    % (key, got, got, want, want, exp.get("_src", "cycle %d" % (k - 1))),
                )

    # ------------------------------------------------------------------ step 2: comb + inputs
    def post_cycle(self, k, S, inp, C):
        """Check cycle-k accept/commit/pc_load rules (C1, C2, C4) and schedule C3 for k+1."""
        self.cycles_checked += 1
        halted = bool(S["halted"])
        mem_free = bool(S["mem_free"])
        accept = bool(C["accept"])
        commit = bool(C["commit"])
        pc_load = bool(C["pc_load"])
        usb_en = bool(inp.get("usb_en", 0))
        host_resume = bool(inp.get("host_resume", 0))
        host_pc_load = bool(inp.get("host_pc_load", 0))
        host_pc_val = int(inp.get("host_pc_val", 0)) & 0x1FF
        host_halt = bool(inp.get("host_halt", 0))
        host_core_reset = bool(inp.get("host_core_reset", 0))

        # --- C1: no activity while halted
        if halted:
            if accept:
                self._fail(k, "C1-halted", "accept asserted while core is halted")
            if commit:
                self._fail(k, "C1-halted", "commit asserted while core is halted")
            if pc_load:
                self._fail(k, "C1-halted", "pc_load asserted while core is halted")

        # --- Default next-cycle expectation: architectural state unchanged unless commit/host ctl
        exp = {
            "oe": S["oe"],
            "out": S["out"],
            "uo": S["uo"],
            "sr": S["sr"],
            "c0": S["c0"],
            "c1": S["c1"],
            "halted": int(halted),
            "illegal": int(bool(S["illegal"])),
            "_src": "no-commit at cyc %d" % k,
        }

        committing_halt_or_ill = False
        committing_redirect = False

        # --- C2 & C3: evaluate held instruction and commit timing
        if self._held is None:
            if commit:
                self._fail(k, "C2-commit", "commit asserted with no instruction held")
            if pc_load:
                self._fail(k, "C1-pcload", "pc_load asserted with no instruction held")
        else:
            acc_k, cur_pc, cur_word = self._held
            op, a, p, d = decode_word(cur_word)
            due = acc_k + 1 + d
            illegal_w = is_illegal_word(cur_word)

            if illegal_w or op not in (OP_WAIT, OP_HPULL):
                # Unconditional instruction: must commit at exact cycle A + 1 + D
                if k < due and commit:
                    self._fail(
                        k, "C2-early",
                        "unconditional op=0x%X (A=%d, D=%d) committed early at %d (expected %d)"
                        % (op, acc_k, d, k, due),
                    )
                if k == due and not commit:
                    self._fail(
                        k, "C2-late",
                        "unconditional op=0x%X (A=%d, D=%d) failed to commit at expected %d"
                        % (op, acc_k, d, due),
                    )
                if k > due:
                    self._fail(
                        k, "C2-missed",
                        "unconditional op=0x%X (A=%d, D=%d) missed commit cycle %d"
                        % (op, acc_k, d, due),
                    )
            else:
                # Blocking instruction (WAIT or HPULL): first cycle >= A + 1 + D with condition true
                if k < due and commit:
                    self._fail(
                        k, "C2-early",
                        "blocking op=0x%X (A=%d, D=%d) committed before delta elapsed (%d < %d)"
                        % (op, acc_k, d, k, due),
                    )
                if k >= due:
                    if op == OP_HPULL:
                        cond = not bool(S["tx_empty"])
                        if cond and not commit:
                            self._fail(
                                k, "C2-hpull",
                                "HPULL (A=%d, D=%d) did not commit when TX FIFO non-empty"
                                % (acc_k, d),
                            )
                        if not cond and commit:
                            self._fail(
                                k, "C2-hpull",
                                "HPULL (A=%d, D=%d) committed while TX FIFO empty"
                                % (acc_k, d),
                            )
                    else:  # OP_WAIT
                        pin = a
                        wtype = p & 0x3
                        if not self._is_virtual_usb_pin(pin, usb_en):
                            cond = self._wait_cond(pin, wtype)
                            if cond and not commit:
                                self._fail(
                                    k, "C4-wait",
                                    "WAIT pin=%d type=%d (A=%d, D=%d) did not commit when sync condition held (s2=%d s3=%d)"
                                    % (pin, wtype, acc_k, d, (self._s2 >> pin) & 1, (self._s3 >> pin) & 1),
                                )
                            if not cond and commit:
                                self._fail(
                                    k, "C4-wait",
                                    "WAIT pin=%d type=%d (A=%d, D=%d) committed while sync condition false (s2=%d s3=%d)"
                                    % (pin, wtype, acc_k, d, (self._s2 >> pin) & 1, (self._s3 >> pin) & 1),
                                )

            # Check pc_load & compute exact C3 next-cycle state if committing
            if not commit:
                if pc_load:
                    self._fail(k, "C1-pcload", "pc_load asserted on non-commit cycle")
            else:
                self.commits_checked += 1
                exp["_src"] = "commit pc=0x%03X op=0x%X word=0x%08X at cyc %d" % (cur_pc, op, cur_word, k)

                # Consequence checks from docs/ISA.md §3 table (against previous commit)
                if self._prev_commit_info is not None and not illegal_w and op not in (OP_WAIT, OP_HPULL):
                    prev_kind, prev_c, prev_d = self._prev_commit_info
                    if prev_kind == "branch":
                        want_c = prev_c + d + 4
                        if k != want_c:
                            self._fail(
                                k, "C2-branch-penalty",
                                "taken-branch target committed at %d, expected %d (prev_c=%d + D=%d + 4)"
                                % (k, want_c, prev_c, d),
                            )
                    elif prev_kind == "seq_uncond":
                        lat = max(0, 2 - prev_d)
                        want_c = prev_c + lat + d + 1
                        if k != want_c:
                            self._fail(
                                k, "C2-spacing",
                                "sequential commit at %d, expected %d (prev_c=%d, prev_D=%d, D=%d)"
                                % (k, want_c, prev_c, prev_d, d),
                            )

                # Decode exact architectural effect per docs/ISA.md §2
                want_pc_load = False
                branch_target = None

                if illegal_w:
                    exp["halted"] = 1
                    exp["illegal"] = 1
                    committing_halt_or_ill = True
                    self._resume_pc = (cur_pc + 1) & 0x1FF
                elif op == OP_NOP:
                    pass
                elif op == OP_PINS:
                    bank = (a >> 3) & 1
                    mode = a & 3
                    mask = p
                    if bank == 0:  # uio
                        oe, out = S["oe"], S["out"]
                        if mode == 0:    # low
                            out &= ~mask
                            oe |= mask
                        elif mode == 1:  # high
                            out |= mask
                            oe |= mask
                        elif mode == 2:  # float: clears OE, keeps latch
                            oe &= ~mask
                        else:            # toggle: toggles latch, enables OE
                            out ^= mask
                            oe |= mask
                        exp["oe"] = oe & 0xFF
                        exp["out"] = out & 0xFF
                    else:  # uo (float is no-op)
                        uo = S["uo"]
                        if mode == 0:
                            uo &= ~mask
                        elif mode == 1:
                            uo |= mask
                        elif mode == 3:
                            uo ^= mask
                        exp["uo"] = uo & 0xFF
                elif op == OP_WAIT:
                    pass
                elif op == OP_JMP:
                    want_pc_load = True
                    branch_target = ((a & 1) << 8) | p
                elif op == OP_JHI:
                    if not self._is_virtual_usb_pin(a, usb_en):
                        want_pc_load = (self._src_bit(a) == 1)
                    else:
                        want_pc_load = pc_load  # virtual USB pin not derived from physical pad
                    branch_target = (cur_pc & 0x100) | p
                elif op == OP_JLO:
                    if not self._is_virtual_usb_pin(a, usb_en):
                        want_pc_load = (self._src_bit(a) == 0)
                    else:
                        want_pc_load = pc_load
                    branch_target = (cur_pc & 0x100) | p
                elif op == OP_SHOUT:
                    msb = bool(p & 1)
                    shift = bool(p & 2)
                    od = bool(p & 4)
                    bit = ((S["sr"] >> 7) & 1) if msb else (S["sr"] & 1)
                    if a < 8:
                        m = 1 << a
                        oe, out = S["oe"], S["out"]
                        if od:
                            out &= ~m
                            if bit == 0:
                                oe |= m
                            else:
                                oe &= ~m
                        else:
                            oe |= m
                            if bit:
                                out |= m
                            else:
                                out &= ~m
                        exp["oe"] = oe & 0xFF
                        exp["out"] = out & 0xFF
                    else:
                        m = 1 << (a - 8)
                        uo = S["uo"]
                        if bit:
                            uo |= m
                        else:
                            uo &= ~m
                        exp["uo"] = uo & 0xFF
                    if shift:
                        exp["sr"] = ((S["sr"] << 1) & 0xFF) if msb else (S["sr"] >> 1)
                elif op == OP_SAMPLE:
                    msb = bool(p & 1)
                    if not self._is_virtual_usb_pin(a, usb_en):
                        bit = self._src_bit(a)
                        exp["sr"] = (((S["sr"] << 1) | bit) & 0xFF) if msb else ((bit << 7) | (S["sr"] >> 1))
                    else:
                        exp["sr"] = None  # virtual USB DATA pin not checked against physical pad
                elif op == OP_REG:
                    sr, c0, c1 = S["sr"], S["c0"], S["c1"]
                    if a == 0:
                        exp["sr"] = p
                    elif a == 1:
                        exp["c0"] = p
                    elif a == 2:
                        exp["c1"] = p
                    elif a == 3:
                        exp["c0"] = sr
                    elif a == 4:
                        exp["c1"] = sr
                    elif a == 5:
                        exp["sr"] = c0
                    elif a == 6:
                        exp["sr"] = c1
                    elif a == 8:
                        exp["sr"] = (sr ^ p) & 0xFF
                    elif a == 9:
                        exp["sr"] = (sr & p) & 0xFF
                    elif a == 10:
                        exp["sr"] = (sr | p) & 0xFF
                    elif a == 11:
                        exp["sr"] = (sr + p) & 0xFF
                elif op == OP_DJNZ:
                    idx = a & 1
                    if idx == 0:
                        nv = (S["c0"] - 1) & 0xFF
                        exp["c0"] = nv
                    else:
                        nv = (S["c1"] - 1) & 0xFF
                        exp["c1"] = nv
                    want_pc_load = (nv != 0)
                    branch_target = (cur_pc & 0x100) | p
                elif op == OP_HPUSH:
                    # Overrun rule (docs/ISA.md §2): evaluated against start-of-cycle rx_full;
                    # drops and increments (saturating at 255) even if rx_pop=1 in the same cycle.
                    # rx_fifo_flush overrides HPUSH and clears rx_overrun to 0 at the cycle edge.
                    if bool(inp.get("rx_fifo_flush", 0)):
                        exp["rx_overrun"] = 0
                    elif bool(S["rx_full"]):
                        exp["rx_overrun"] = min(255, S["rx_overrun"] + 1)
                    else:
                        exp["rx_overrun"] = S["rx_overrun"]
                elif op == OP_HPULL:
                    exp["sr"] = None  # populated from TX FIFO head (not exposed in S)
                elif op == OP_HALT:
                    exp["halted"] = 1
                    committing_halt_or_ill = True
                    self._resume_pc = (cur_pc + 1) & 0x1FF
                elif op == OP_JSB:
                    bit_idx = a & 7
                    pol = (a >> 3) & 1
                    want_pc_load = (((S["sr"] >> bit_idx) & 1) == pol)
                    branch_target = (cur_pc & 0x100) | p

                if pc_load != want_pc_load:
                    self._fail(
                        k, "C1-pcload",
                        "pc_load=%d != expected %d for op=0x%X (pc=0x%03X, a=0x%X, p=0x%02X)"
                        % (int(pc_load), int(want_pc_load), op, cur_pc, a, p),
                    )

                self._held = None
                if want_pc_load:
                    committing_redirect = True
                    self._exp_accept = (k + 3, branch_target, "branch")
                    self._prev_commit_info = ("branch", k, d)
                elif committing_halt_or_ill:
                    self._exp_accept = None
                    self._prev_commit_info = None
                else:
                    # Sequential next instruction ready at max(acc_k + 3, k)
                    nxt_acc = max(acc_k + 3, k)
                    self._exp_accept = (nxt_acc, (cur_pc + 1) & 0x1FF, "seq")
                    if op in (OP_WAIT, OP_HPULL):
                        self._prev_commit_info = ("seq_block", k, d)
                    else:
                        self._prev_commit_info = ("seq_uncond", k, d)

        # --- C1: check accept rules (after commit frees the unit)
        if accept:
            if committing_redirect or committing_halt_or_ill:
                self._fail(
                    k, "C1-accept-redirect",
                    "accept asserted in a cycle that %s"
                    % ("redirected (pc_load)" if committing_redirect else "halted"),
                )
            if self._held is not None:
                self._fail(k, "C1-accept-busy", "accept asserted while unit busy and not committing")
            if self._exp_accept is None:
                self._fail(k, "C1-accept-unexpected", "unexpected accept at pc=0x%03X" % C["pc"])
            exp_k, exp_pc, kind = self._exp_accept
            if k != exp_k:
                self._fail(
                    k, "C1-accept-cycle",
                    "%s accept at cyc %d, expected cyc %d" % (kind, k, exp_k),
                )
            if (C["pc"] & 0x1FF) != exp_pc:
                self._fail(
                    k, "C1-accept-pc",
                    "%s accept pc=0x%03X, expected 0x%03X" % (kind, C["pc"] & 0x1FF, exp_pc),
                )
            self._held = (k, C["pc"] & 0x1FF, C["instr"] & 0xFFFFFFFF)
            self._exp_accept = None
            self.accepts_checked += 1
        else:
            if self._exp_accept is not None:
                exp_k, exp_pc, kind = self._exp_accept
                if k >= exp_k:
                    self._fail(
                        k, "C1-accept-missed",
                        "expected %s accept at cyc %d (pc=0x%03X) did not occur"
                        % (kind, exp_k, exp_pc),
                    )

        # --- Host run-control transitions (only while halted & mem_free, except halt/core_reset)
        if host_core_reset or host_halt:
            self._held = None
            self._exp_accept = None
            self._prev_commit_info = None
            exp["halted"] = 1
            if host_core_reset:
                exp["oe"] = 0
                exp["uo"] = 0
                exp["sr"] = 0
                exp["c0"] = 0
                exp["c1"] = 0
                exp["illegal"] = 0
        elif halted and mem_free:
            if host_pc_load:
                self._resume_pc = host_pc_val
            if host_resume:
                exp["halted"] = 0
                exp["illegal"] = 0
                exp["_src"] = "host_resume at cyc %d" % k
                # core_run rises in cycle k+1 -> first instruction valid 3 cycles later at k+4
                self._exp_accept = (k + 4, self._resume_pc, "resume")
                self._prev_commit_info = None

        self._exp_next = exp

        # --- C4: advance independent 2-FF input synchroniser at the rising edge ending cycle k
        raw_pads = (int(inp.get("uio_in", 0xFF)) & 0xFF) | ((int(inp.get("ui_in", 0xFF)) & 0xFF) << 8)
        self._s3 = self._s2
        self._s2 = self._s1
        self._s1 = raw_pads
