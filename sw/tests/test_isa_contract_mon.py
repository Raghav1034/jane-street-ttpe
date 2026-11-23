"""Unit tests and negative controls for test/isa_contract_mon.py (V1).

Validates the independent docs/ISA.md contract monitor in two ways:
  1. Positive coverage: records full cycle traces from all core lockstep scenarios
     (directed, examples, USB, and fuzz) and asserts IsaContractMonitor accepts every
     cycle with zero false positives.
  2. Negative controls: mutates single timing or architectural rules in recorded traces
     and asserts IsaContractMonitor raises IsaContractError naming the violated rule:
       - N1: +1 cycle late commit on an unconditional instruction (C2-late)
       - N2: -1 cycle early commit on an unconditional instruction (C2-early)
       - N3: 2-cycle instead of 3-cycle taken-branch penalty (C1-accept-cycle)
       - N4: pin output changes at commit+2 instead of commit+1 (C3-state)
       - N5: input synchroniser 1-cycle instead of 2-cycle latency on SAMPLE (C3-state)
       - N6: input synchroniser premature WAIT rise on 1-cycle latency (C4-wait)
       - N7: accept asserted in the same cycle as a taken branch pc_load (C1-accept-redirect)
       - N8: PINS float wrongly clears uio_out latch (C3-state)
       - N9: HPULL commits while TX FIFO is empty (C2-hpull)
"""

import asyncio
import copy
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SW_DIR = os.path.join(ROOT, "sw")
TEST_DIR = os.path.join(ROOT, "test")
for p in (SW_DIR, TEST_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import ttpe_asm as asm
import ttpe_isa as isa
import ttpe_iss as iss_mod
import ttpe_scenarios as scn
import ttpe_world as W
from isa_contract_mon import IsaContractError, IsaContractMonitor
from ttpe_lockstep import DefaultPolicy, FixedWorld, Lockstep, MAX_RX, MAX_TX


class TraceRecorderDut:
    """Pure-Python ISS-backed DutIface that records (k, S, inp, C) tuples for monitor tests.

    Used strictly to produce realistic cycle traces for positive sanity checks and
    mutated-trace negative controls in environments without an HDL simulator.
    """

    def __init__(self, words):
        words = list(words)
        if len(words) != isa.PROG_WORDS:
            words += [isa.encode(isa.OPCODES["HALT"])] * (isa.PROG_WORDS - len(words))
        self.t = iss_mod.Ttpe(words)
        self.t.halted = True
        self.fixed = FixedWorld()
        self.trace = []
        self._k = -1
        self._cur_S = None
        self._cur_inp = None
        self._rd_expect = {}
        self._wd_expect = set()
        self._pending_state = None

    def _snapshot(self):
        t = self.t
        txn = len(t.tx) - t.tx_idx
        return dict(
            oe=t.uio_oe, out=t.uio_out, uo=t.uo_out,
            sr=t.sr, c0=t.c0, c1=t.c1,
            halted=int(t.halted), illegal=int(t.illegal),
            tx_count=txn, rx_count=len(t.rx),
            tx_full=int(txn == MAX_TX), tx_empty=int(txn == 0),
            rx_full=int(len(t.rx) == MAX_RX), rx_empty=int(len(t.rx) == 0),
            rx_overrun=min(t.rx_overrun, 255),
            rx_rdata=t.rx[0][1] if t.rx else 0,
            mem_free=int(t.halted),
        )

    async def next_cycle(self):
        self._k += 1
        self._cur_S = self._snapshot()

    def state(self):
        return dict(self._cur_S)

    def drive(self, d):
        self._cur_inp = dict(d)

    async def mid_cycle(self):
        pass

    def comb(self):
        t = self.t
        inp = self._cur_inp
        S = self._cur_S
        k = self._k

        # Advance synchroniser even while halted (matching physical RTL ttpe_input_sync)
        self.fixed.uio = inp["uio_in"] & 0xFF
        self.fixed.ui = inp["ui_in"] & 0xFF
        t.usb_en = bool(inp.get("usb_en", 0))

        # Capture comb events from stepping ISS
        host_gnt = int(bool(inp["host_req"]) and bool(S["mem_free"]))
        host_wdone = int(k in self._wd_expect)
        self._wd_expect.discard(k)
        if k in self._rd_expect:
            _, rdata = self._rd_expect.pop(k)
            host_rvalid, host_rdata = 1, rdata
        else:
            host_rvalid, host_rdata = 0, 0

        if not t.halted:
            na, nc = len(t.accepts), len(t.commits)
            prev_pc_load_cyc = None
            t.step(self.fixed)
            acc = t.accepts[na:]
            com = t.commits[nc:]
            accept = int(bool(acc))
            commit = int(bool(com))
            acc_pc = acc[0][1] if acc else 0
            acc_instr = t.mem[acc_pc] if acc else 0
            # pc_load is asserted when a branch took effect on this commit
            pc_load = 0
            if com:
                _, cpc, cw = com[0]
                op = (cw >> 28) & 0xF
                if op in (3, 4, 5, 9, 13) and not t.halted and not acc:
                    pc_load = int(t.fetch_valid_at == (t.cycle - 1) + 3)
        else:
            accept = commit = acc_pc = acc_instr = pc_load = 0

        # Host FIFO and SRAM side effects (matching Lockstep order)
        txf = bool(inp["tx_fifo_flush"])
        if inp["tx_push"] and not txf and not S["tx_full"]:
            t.tx.append((t.cycle, inp["tx_wdata"] & 0xFF))
        if txf:
            del t.tx[t.tx_idx:]
        if inp["rx_fifo_flush"]:
            t.rx.clear()
            t.rx_overrun = 0
        elif inp["rx_pop"] and not S["rx_empty"]:
            t.rx.pop(0)

        if host_gnt:
            a = inp["host_addr"] & 0x1FF
            if inp["host_we"]:
                w = t.mem[a]
                st = inp["host_wstrb"]
                nw = inp["host_wdata"]
                for b in range(4):
                    if (st >> b) & 1:
                        w = (w & ~(0xFF << (8 * b))) | (nw & (0xFF << (8 * b)))
                t.mem[a] = w & 0xFFFFFFFF
                self._wd_expect.add(k + 1)
            else:
                self._rd_expect[k + 2] = (a, t.mem[a])

        if S["halted"] and S["mem_free"]:
            if inp["host_resume"]:
                t.resume(inp["host_pc_val"] & 0x1FF if inp["host_pc_load"] else None)
            elif inp["host_pc_load"]:
                t.fetch_pc = inp["host_pc_val"] & 0x1FF

        C = dict(
            accept=accept, commit=commit, pc=acc_pc, instr=acc_instr, pc_load=pc_load,
            host_gnt=host_gnt, host_wdone=host_wdone,
            host_rvalid=host_rvalid, host_rdata=host_rdata,
        )
        self.trace.append((k, dict(S), dict(inp), dict(C)))
        return C


def record_program(src, world=None, policy=None, cycles=120, usb_en=False):
    words = scn.pad512(asm.assemble(src, allow_short_delta=True).image())
    dut = TraceRecorderDut(words)
    ls = Lockstep(dut, words, world or W.BusWorld(), policy or DefaultPolicy(),
                  name="rec", usb_en=usb_en)
    asyncio.run(ls.run(cycles))
    ls.finish()
    return dut.trace


def replay_trace(trace, name="replay"):
    mon = IsaContractMonitor(name=name)
    for k, S, inp, C in trace:
        mon.pre_cycle(k, S)
        mon.post_cycle(k, S, inp, C)
    return mon


class TestIsaContractMonitor(unittest.TestCase):
    def test_positive_all_directed_scenarios(self):
        """Every directed scenario passes Lockstep + live IsaContractMonitor with 0 errors."""
        for sc in scn.directed():
            dut = TraceRecorderDut(sc.words)
            ls = Lockstep(dut, sc.words, sc.world, sc.policy, sc.name, usb_en=sc.usb_en)
            asyncio.run(ls.run(sc.cycles))
            ls.finish()
            if sc.check:
                sc.check(ls)
            self.assertGreater(ls.isa_mon.commits_checked, 0, sc.name)

    def test_positive_examples_and_fuzz(self):
        """Selected protocol examples (UART, SPI, I2C M/T, USB) + 4 fuzz seeds pass live monitor."""
        subset = [
            scn.scn_uart_tx(20, [0x55, 0xA5]),
            scn.scn_spi(0, 0, 24, [0xA5, 0x0F], [0xC3, 0x5A]),
            scn.scn_i2c_rstart(75, 50, 9500),
            scn.scn_i2c_target_400k(),
            scn.scn_usb_packets("clean", 0),
            scn.scn_usb_tx("ack", [[0xD2]]),
        ] + [scn.scn_fuzz(s) for s in range(4)]
        for sc in subset:
            dut = TraceRecorderDut(sc.words)
            ls = Lockstep(dut, sc.words, sc.world, sc.policy, sc.name, usb_en=sc.usb_en)
            asyncio.run(ls.run(sc.cycles))
            ls.finish()
            if sc.check:
                sc.check(ls)
            self.assertGreater(ls.isa_mon.commits_checked, 0, sc.name)

    def test_neg_N1_late_commit_unconditional(self):
        """Negative control N1: shift an unconditional commit by +1 cycle -> C2-late fires."""
        tr = record_program("LDI sr, 0x5A @2\nPINS high uio0 @4\nHALT\n", cycles=40)
        replay_trace(tr, "N1-clean")
        mut = copy.deepcopy(tr)
        # Find second commit (PINS high uio0 @4) and delay it by 1 cycle
        commits = [i for i, (_, _, _, C) in enumerate(mut) if C["commit"]]
        c_idx = commits[1]
        mut[c_idx][3]["commit"] = 0
        mut[c_idx + 1][3]["commit"] = 1
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N1-mut")
        self.assertIn("C2-late", str(cm.exception))

    def test_neg_N2_early_commit_unconditional(self):
        """Negative control N2: fire an unconditional commit 1 cycle early -> C2-early fires."""
        tr = record_program("LDI sr, 0x5A @2\nPINS high uio0 @5\nHALT\n", cycles=40)
        replay_trace(tr, "N2-clean")
        mut = copy.deepcopy(tr)
        commits = [i for i, (_, _, _, C) in enumerate(mut) if C["commit"]]
        c_idx = commits[1]
        mut[c_idx - 1][3]["commit"] = 1
        mut[c_idx][3]["commit"] = 0
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N2-mut")
        self.assertIn("C2-early", str(cm.exception))

    def test_neg_N3_wrong_branch_penalty(self):
        """Negative control N3: 2-cycle instead of 3-cycle taken-branch penalty -> C1-accept-cycle fires."""
        tr = record_program("JMP tgt @2\nNOP @2\ntgt: PINS high uio0 @2\nHALT\n", cycles=40)
        replay_trace(tr, "N3-clean")
        mut = copy.deepcopy(tr)
        # Find the accept of tgt (2nd accept) and move it 1 cycle earlier (penalty 2 instead of 3)
        accepts = [i for i, (_, _, _, C) in enumerate(mut) if C["accept"]]
        a_idx = accepts[1]
        mut[a_idx - 1][3]["accept"] = 1
        mut[a_idx - 1][3]["pc"] = mut[a_idx][3]["pc"]
        mut[a_idx - 1][3]["instr"] = mut[a_idx][3]["instr"]
        mut[a_idx][3]["accept"] = 0
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N3-mut")
        self.assertIn("C1-accept-cycle", str(cm.exception))

    def test_neg_N4_pin_change_wrong_cycle(self):
        """Negative control N4: pad output updates at commit+2 instead of commit+1 -> C3-state fires."""
        tr = record_program("PINS high uio0 @3\nNOP @5\nHALT\n", cycles=40)
        replay_trace(tr, "N4-clean")
        mut = copy.deepcopy(tr)
        c_idx = next(i for i, (_, _, _, C) in enumerate(mut) if C["commit"])
        # Delay uio_out/uio_oe update by 1 cycle
        mut[c_idx + 1][1]["oe"] = mut[c_idx][1]["oe"]
        mut[c_idx + 1][1]["out"] = mut[c_idx][1]["out"]
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N4-mut")
        self.assertIn("C3-state", str(cm.exception))

    def test_neg_N5_wrong_sync_latency_sample(self):
        """Negative control N5: SAMPLE observes 1-cycle-old pad instead of 2-cycle-old -> C3-state fires."""
        # Pulse ui3 low for 1 cycle right before SAMPLE commits so X-1 != X-2
        src = "NOP @4\nSAMPLE ui3 lsb @2\nHALT\n"
        # First accept at k=4, NOP commits at k=9, SAMPLE accepted at k=9, commits at k=12.
        # Pad at k=10 is seen by SAMPLE at k=12 (2-FF sync). Make ui3=0 at k=10, ui3=1 at k=11.
        w = scn.UiWorld(ui_fn=lambda wk: 0xF7 if wk == 9 else 0xFF)
        tr = record_program(src, world=w, cycles=30)
        replay_trace(tr, "N5-clean")
        mut = copy.deepcopy(tr)
        commits = [i for i, (_, _, _, C) in enumerate(mut) if C["commit"]]
        s_idx = commits[1]
        # Corrupt sampled SR bit (as if 1-FF sync saw ui3=1 instead of 0)
        mut[s_idx + 1][1]["sr"] ^= 0x80
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N5-mut")
        self.assertIn("C3-state", str(cm.exception))

    def test_neg_N6_premature_wait_rise(self):
        """Negative control N6: WAIT rise commits 1 cycle early (1-FF sync bug) -> C4-wait fires."""
        src = "WAIT rise ui3 @2\nHALT\n"
        # ui3 low until wk=14, rises at wk=15 (RTL k=16) -> 2-FF sync sees rise at k=18
        w = scn.UiWorld(ui_fn=lambda wk: 0xF7 if wk < 15 else 0xFF)
        tr = record_program(src, world=w, cycles=40)
        replay_trace(tr, "N6-clean")
        mut = copy.deepcopy(tr)
        w_idx = next(i for i, (_, _, _, C) in enumerate(mut) if C["commit"])
        # Move WAIT commit 1 cycle earlier (as if 1-FF sync)
        mut[w_idx - 1][3]["commit"] = 1
        mut[w_idx][3]["commit"] = 0
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N6-mut")
        self.assertIn("C4-wait", str(cm.exception))

    def test_neg_N7_accept_during_redirect(self):
        """Negative control N7: accept asserted on taken-branch pc_load cycle -> C1-accept-redirect fires."""
        tr = record_program("JMP tgt @2\nNOP @2\ntgt: HALT\n", cycles=30)
        replay_trace(tr, "N7-clean")
        mut = copy.deepcopy(tr)
        j_idx = next(i for i, (_, _, _, C) in enumerate(mut) if C["pc_load"])
        mut[j_idx][3]["accept"] = 1
        mut[j_idx][3]["pc"] = 1
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N7-mut")
        self.assertIn("C1-accept-redirect", str(cm.exception))

    def test_neg_N8_pins_float_corrupts_latch(self):
        """Negative control N8: PINS float clears uio_out latch instead of keeping it -> C3-state fires."""
        tr = record_program("PINS high uio2 @2\nPINS float uio2 @2\nHALT\n", cycles=30)
        replay_trace(tr, "N8-clean")
        mut = copy.deepcopy(tr)
        commits = [i for i, (_, _, _, C) in enumerate(mut) if C["commit"]]
        f_idx = commits[1]
        mut[f_idx + 1][1]["out"] &= ~(1 << 2)
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N8-mut")
        self.assertIn("C3-state", str(cm.exception))

    def test_neg_N9_hpull_commits_while_empty(self):
        """Negative control N9: HPULL commits while TX FIFO empty -> C2-hpull fires."""
        tr = record_program("HPULL @2\nHALT\n", policy=DefaultPolicy(tx_timed={20: 0xAB}), cycles=40)
        replay_trace(tr, "N9-clean")
        mut = copy.deepcopy(tr)
        h_idx = next(i for i, (_, _, _, C) in enumerate(mut) if C["commit"])
        # Commit 1 cycle earlier while tx_empty is still 1
        mut[h_idx - 1][3]["commit"] = 1
        mut[h_idx][3]["commit"] = 0
        with self.assertRaises(IsaContractError) as cm:
            replay_trace(mut, "N9-mut")
        self.assertIn("C2-hpull", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
