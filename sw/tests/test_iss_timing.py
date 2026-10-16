"""ISS semantics and timing-model tests (the contract for the Execution Unit RTL)."""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import ttpe_asm as asm      # noqa: E402
import ttpe_isa as isa      # noqa: E402
import ttpe_iss as iss      # noqa: E402


class ScheduleWorld:
    """ui pins follow ui_fn(cycle); uio pins follow uio_fn(cycle) (None = pull-up/loopback)."""

    def __init__(self, ui_fn=lambda k: 0, uio_fn=None):
        self.ui_fn, self.uio_fn = ui_fn, uio_fn

    def pads(self, k, oe, out, uo):
        uio = self.uio_fn(k) if self.uio_fn else ((out & oe) | (0xFF & ~oe & 0xFF))
        return uio & 0xFF, self.ui_fn(k) & 0xFF


def build(src, **kw):
    return asm.assemble(src, **kw).image()


def run(src, world=None, cycles=300, host_tx=None, start_pc=0, allow_short=False, **kw):
    t = iss.Ttpe(build(src, allow_short_delta=allow_short), start_pc=start_pc,
                 host_tx=host_tx, **kw)
    t.run_cycles(world or iss.PullWorld(), cycles)
    return t


class FetchPipeline(unittest.TestCase):
    def test_first_instruction_accepted_in_cycle_3(self):
        t = run("NOP @5\nHALT")
        self.assertEqual(t.accepts[0], (3, 0))
        self.assertEqual(t.commits[0][0], 3 + 1 + 5)

    def test_sequential_chain_closed_form(self):
        """commit(N+1) = commit(N) + delta(N+1) + 1 + max(0, 2 - delta(N)); includes short deltas."""
        rng = random.Random(7)
        for trial in range(200):
            n = rng.randrange(2, 12)
            deltas = [rng.choice([0, 1, 2, 3, 4, 7, 20]) for _ in range(n)]
            ops = [rng.choice(["NOP", "PINS toggle uio%d" % rng.randrange(8),
                               "LDI sr, %d" % rng.randrange(256)]) for _ in range(n)]
            src = "\n".join("%s @%d" % (o, d) for o, d in zip(ops, deltas)) + "\nHALT"
            t = run(src, allow_short=True, cycles=2000)
            exp_acc = [3]
            exp_com = [3 + 1 + deltas[0]]
            for i in range(1, n):
                a = max(exp_com[i - 1], exp_acc[i - 1] + 3)
                exp_acc.append(a)
                exp_com.append(a + 1 + deltas[i])
            got = [c for c, _, _ in t.commits[:n]]
            self.assertEqual(got, exp_com, "deltas=%s" % deltas)
            # strict-delta programs never stall: spacing is exactly delta+1
            if min(deltas) >= 2:
                self.assertTrue(all(got[i] - got[i - 1] == deltas[i] + 1 for i in range(1, n)))

    def test_taken_branch_costs_three_cycles(self):
        t = run("""
            JMP tgt @6          ; accept 3, commit 10
            NOP @2              ; flushed
        tgt: NOP @5
            HALT
        """)
        c = dict((pc, cyc) for cyc, pc, _ in t.commits)
        self.assertEqual(c[0], 10)
        self.assertNotIn(1, c)                       # skipped instruction never commits
        self.assertEqual(c[2], 10 + 4 + 5)           # commit(target) = commit(jmp) + 4 + delta

    def test_not_taken_branch_costs_nothing_extra(self):
        t = run("""
            JHI uio5, never @4     ; uio5 reads low (loopback of undriven pin w/o pull) -> not taken
            NOP @4
        never: HALT
        """, world=ScheduleWorld(uio_fn=lambda k: 0x00))
        c = [cyc for cyc, _, _ in t.commits]
        self.assertEqual(c[0], 3 + 1 + 4)
        self.assertEqual(c[1], c[0] + 4 + 1)         # plain sequential spacing

    def test_djnz_loop_period(self):
        t = run("""
            LDI c0, 5 @2
        loop: NOP @10
            DJNZ c0, loop @2
            HALT
        """, cycles=500)
        nops = [cyc for cyc, pc, _ in t.commits if pc == 1]
        self.assertEqual(len(nops), 5)
        self.assertEqual({b - a for a, b in zip(nops, nops[1:])}, {10 + 2 + 5})   # a + e + 5

    def test_djnz_zero_means_256(self):
        t = run("""
            LDI c1, 0 @2
        l:  NOP @2
            DJNZ c1, l @2
            HALT
        """, cycles=10000)
        self.assertEqual(sum(1 for _, pc, _ in t.commits if pc == 1), 256)
        self.assertTrue(t.halted)

    def test_pc_wraps_511_to_0(self):
        p = asm.assemble(".org 511\nNOP @5\n.org 0\nPINS high uio1 @5\nHALT")
        t = iss.Ttpe(p.image(), start_pc=511)
        t.run_cycles(iss.PullWorld(), 100)
        self.assertEqual([pc for _, pc, _ in t.commits], [511, 0, 1])


class Semantics(unittest.TestCase):
    def test_pins_modes_and_pin_event_cycle(self):
        t = run("""
            PINS high uio0 uio2 @5     ; commit 9
            PINS low uio2 @5
            PINS toggle uio0 uio7 @5
            PINS float uio0 @5
            PINS high uo3 uo5 @5
            PINS toggle uo3 @5
            PINS low uo5 @5
            HALT
        """, cycles=200)
        self.assertEqual(t.pin_events[1], (10, 0x05, 0x05, 0))          # visible the cycle AFTER commit
        self.assertEqual(t.uio_oe, 0x84)                                  # uio0 floated; 2 and 7 driven
        self.assertEqual(t.uio_out, 0x80)                                 # uio2 low, uio0 toggled->0, uio7 toggled->1
        self.assertEqual(t.uo_out, 0x00)

    def test_float_keeps_latched_value_open_drain_idiom(self):
        t = run("""
            PINS low uio3 @5
            PINS float uio3 @5
            PINS low uio3 @5
            HALT""")
        evs = [(oe, out) for _, oe, out, _ in t.pin_events]
        self.assertEqual(evs, [(0, 0), (8, 0), (0, 0), (8, 0)])

    def test_shout_push_pull_open_drain_and_shift(self):
        t = run("""
            LDI sr, 0b10110001 @2
            SHOUT uio1 lsb shift @2          ; sends 1, SR >>= 1
            SHOUT uio1 msb @2                ; sends SR[7]: SR = 0b01011000 -> 0 (no shift)
            SHOUT uio4 msb shift od @2       ; open-drain: bit 0 -> drive low, SR <<= 1
            SHOUT uio4 msb shift od @2       ; bit 1 -> release
            HALT""")
        # after sequence: SR = 0b01011000 << 2 = 0b01100000
        self.assertEqual(t.sr, 0b01100000)
        self.assertEqual(t.uio_out & 0x10, 0)         # od never latches a 1
        self.assertEqual(t.uio_oe & 0x10, 0)          # last od bit was 1 -> released
        self.assertEqual(t.uio_oe & 0x02, 0x02)       # push-pull pin stays driven
        self.assertEqual(t.uio_out & 0x02, 0)

    def test_sample_both_directions(self):
        w = ScheduleWorld(ui_fn=lambda k: 0x01)       # ui0 = 1 forever
        t = run("""
            LDI sr, 0 @2
            SAMPLE ui0 lsb @2
            SAMPLE ui0 lsb @2                 ; SR = 0b11000000
            HALT""", world=w)
        self.assertEqual(t.sr, 0xC0)
        t = run("""
            LDI sr, 0 @2
            SAMPLE ui0 msb @2
            SAMPLE ui0 msb @2
            SAMPLE ui0 msb @2
            HALT""", world=w)
        self.assertEqual(t.sr, 0x07)

    def test_reg_ops(self):
        t = run("""
            LDI sr, 0xF0 @2
            XORI sr, 0xFF @2          ; 0x0F
            ORI sr, 0x30 @2           ; 0x3F
            ANDI sr, 0x7C @2          ; 0x3C
            ADDI sr, 0xC8 @2          ; 0x3C + 0xC8 = 0x104 -> 0x04
            MOV c0, sr @2
            LDI c1, 9 @2
            MOV sr, c1 @2
            HALT""")
        self.assertEqual((t.sr, t.c0, t.c1), (9, 4, 9))

    def test_jsb_both_polarities(self):
        t = run("""
            LDI sr, 0b00100000 @2
            JSB sr[5]==1, a @2
            PINS high uio0 @2            ; skipped
        a:  JSB sr[5]==0, b @2           ; not taken
            PINS high uio1 @2
        b:  JSB sr[0]==0, c @2           ; taken
            PINS high uio2 @2            ; skipped
        c:  HALT""")
        self.assertEqual(t.uio_out, 0x02)

    def test_conditional_branch_page_uses_branch_address(self):
        p = asm.assemble("""
            .org 0x1F0
            JLO uio0, there @2
            HALT
            .org 0x1F8
        there: PINS high uio3 @2
            HALT""")
        t = iss.Ttpe(p.image(), start_pc=0x1F0)
        t.run_cycles(iss.PullWorld(pull=0x00), 100)
        self.assertEqual(t.uio_out, 0x08)

    def test_illegal_opcode_halts_with_flag(self):
        for word in (0xE0000000, 0xF0000000, 0x8F000000):
            p = asm.assemble(".word 0x%08X" % word)
            t = iss.Ttpe(p.image())
            t.run_cycles(iss.PullWorld(), 50)
            self.assertTrue(t.halted and t.illegal, hex(word))

    def test_halt_and_resume_continue_after_halt(self):
        p = asm.assemble("PINS high uio0 @5\nHALT\nPINS high uio1 @5\nHALT")
        t = iss.Ttpe(p.image())
        t.run(iss.PullWorld())
        self.assertEqual((t.uio_out, t.fetch_pc), (0x01, 2))
        c = t.cycle
        t.resume()
        t.run(iss.PullWorld())
        self.assertEqual(t.uio_out, 0x03)
        self.assertEqual(t.accepts[-2][0], c + 3)      # pipeline restarts like core_run 0->1

    def test_resume_with_new_pc(self):
        p = asm.assemble("HALT\n.org 7\nPINS high uio6 @5\nHALT")
        t = iss.Ttpe(p.image())
        t.run(iss.PullWorld())
        t.resume(start_pc=7)
        t.run(iss.PullWorld())
        self.assertEqual(t.uio_out, 0x40)


class InputTiming(unittest.TestCase):
    def test_sync_latency_is_exactly_two_cycles(self):
        """pad changes in cycle X: an instruction committing in X+1 sees the old value,
        one committing in X+2 sees the new value."""
        X = 30
        for commit, expect in ((X + 1, 0x00), (X + 2, 0x80), (X + 3, 0x80), (X - 5, 0x00)):
            delta = commit - 4
            t = run("SAMPLE ui0 lsb @%d\nHALT" % delta, world=ScheduleWorld(lambda k: 1 if k >= X else 0),
                    allow_short=True)
            self.assertEqual(t.commits[0][0], commit)
            self.assertEqual(t.sr, expect, "commit %d" % commit)

    def test_wait_levels(self):
        X = 60
        w = ScheduleWorld(lambda k: 1 if k >= X else 0)
        t = run("WAIT high ui0 @2\nHALT", world=w, cycles=200)
        self.assertEqual(t.commits[0][0], X + 2)
        t = run("WAIT low ui0 @2\nHALT", world=w, cycles=200)      # already low: no blocking
        self.assertEqual(t.commits[0][0], 3 + 1 + 2)
        t = run("WAIT high ui0 @2\nHALT", world=ScheduleWorld(lambda k: 1), cycles=200)
        self.assertEqual(t.commits[0][0], 3 + 1 + 2 if False else t.commits[0][0])   # level already high
        self.assertEqual(t.commits[0][0], 6)

    def test_wait_edges_exact_cycle(self):
        X = 60
        rise = ScheduleWorld(lambda k: 1 if k >= X else 0)
        fall = ScheduleWorld(lambda k: 0 if k >= X else 1)
        t = run("WAIT rise ui0 @2\nHALT", world=rise, cycles=200)
        self.assertEqual(t.commits[0][0], X + 2)
        t = run("WAIT fall ui0 @2\nHALT", world=fall, cycles=200)
        self.assertEqual(t.commits[0][0], X + 2)

    def test_wait_edge_before_armed_is_missed(self):
        """An edge that completes before the delta countdown ends is NOT seen (documented)."""
        X = 4                                    # synced edge appears at cycle 6; armed at 3+1+100
        w = ScheduleWorld(lambda k: 1 if k >= X else 0)
        t = run("WAIT rise ui0 @100\nHALT", world=w, cycles=400)
        self.assertEqual(t.commits, [])
        self.assertFalse(t.halted)

    def test_wait_waits_after_delta_when_level_already_true(self):
        t = run("WAIT high ui0 @40\nHALT", world=ScheduleWorld(lambda k: 1), cycles=200)
        self.assertEqual(t.commits[0][0], 3 + 1 + 40)

    def test_reset_value_of_synchroniser_is_zero(self):
        """Pad is high from cycle 0, yet for 2 cycles the synchronised value is 0: a rising edge
        is seen at cycle 2. Programs must not WAIT for edges in the first cycles."""
        t = run("WAIT rise ui0 @2\nHALT", world=ScheduleWorld(lambda k: 1), cycles=100)
        self.assertEqual(t.commits, [])                          # armed at 6, edge was at cycle 2


class HostFifo(unittest.TestCase):
    def test_hpull_blocks_until_data(self):
        X = 80
        t = run("HPULL @2\nHPUSH @2\nHALT", host_tx=[(X, 0x5A)], cycles=300)
        self.assertEqual(t.commits[0][0], X)
        self.assertEqual(t.rx[0][1], 0x5A)

    def test_hpull_with_data_ready_does_not_stall(self):
        t = run("HPULL @2\nHALT", host_tx=[0x11], cycles=100)
        self.assertEqual(t.commits[0][0], 6)
        self.assertEqual(t.sr, 0x11)

    def test_hpush_overrun_drops_and_flags(self):
        src = "LDI sr, 1 @2\n" + "HPUSH @2\n" * 5 + "HALT"
        t = run(src, cycles=200, rx_depth=3)
        self.assertEqual(len(t.rx), 3)
        self.assertEqual(t.rx_overrun, 2)


class Timing_Lint_Consistency(unittest.TestCase):
    def test_lint_threshold_matches_model(self):
        """delta >= MIN_SAFE_DELTA exactly removes the stall."""
        for d in range(0, 6):
            t = run("NOP @%d\nNOP @9\nHALT" % d, allow_short=True)
            gap = t.commits[1][0] - t.commits[0][0]
            self.assertEqual(gap, 9 + 1 + max(0, isa.MIN_SAFE_DELTA - d))


if __name__ == "__main__":
    unittest.main()
