#!/usr/bin/env python3
"""
Mutation harness for the software side of TTPE: example firmware (usb_tx, usb_rx, usb_device_ack, i2c_master
repeated START) and the Python reference model of the USB bit slicer (usb_ls.UsbRx, the line-by-line
specification of src/ttpe_usb_rx.sv).

Place this file at sw/mutants_fw.py.

For every mutant the whole sw/ tree is copied to a temporary directory, ONE exact text edit is applied
(the edit must match exactly once, otherwise the mutant is reported as STALE), the result is checked to
still assemble / compile (otherwise INVALID, not counted as a kill), and the complete unit-test suite is run
against the copy (`python3 -I -m unittest discover -s sw/tests -f`).  KILLED = at least one test fails;
SURVIVED = the whole suite still passes.  A survivor is either a test gap or an equivalent mutant; the
ones argued to be equivalent carry the argument in `equiv` and are reported separately.

The baseline (no edit) must pass first, otherwise nothing is reported.
Nothing in the repository is modified.

Usage (from the repository root or from sw/):
    python3 sw/mutants_fw.py                 run every mutant, print a table, exit 1 if any non-equivalent
                                             valid mutant survived
    python3 sw/mutants_fw.py --list          list mutants only
    python3 sw/mutants_fw.py --only T1,D3    run selected ids
    python3 sw/mutants_fw.py -j 8 --json r.json
The RTL counterpart (needs Icarus Verilog and cocotb) does not exist yet.

First run (author's sandbox, 93 mutants, against the 138-test suite): 76 killed, 17 survived:
P1-P7, P9, P11, P15, P20, P21 (slicer model: the unit tests are tolerant to +-1 cycle timing faults),
T9, X4, D10, I9, I14.  Reproduced identically on Windows host (Python 3.14.3, 2026-11-19): 93 valid,
76 killed, 17 survived (81.7 %), 0 STALE, 0 INVALID.  Working hypotheses, NOT yet confirmed: T9 is equivalent
for every valid PID (a valid PID has at most four leading 1s, so counting five or six ones at the start is
indistinguishable; killable with an invalid first byte); I14 is equivalent (the NACK flag is already low
whenever `hold` is reached); the rest are test gaps (no exact-timing test of the slicer model, no
ERR-at-byte-boundary test for usb_rx.asm, no [good packet, empty packet] order test for usb_device_ack.asm,
no clock stretch at the repeated-START SCL rise).
"""
import argparse
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))     # .../sw

# (id, file relative to sw/, exact old text, new text, description, equivalence argument or None)
M = []


def mut(mid, rel, old, new, desc, equiv=None):
    M.append((mid, rel, old, new, desc, equiv))


# ------------------------------------------------------------------ USB slicer reference model (usb_ls.UsbRx)
U = "usb_ls.py"
mut("P1", U, "SAMPLE_AT = 16", "SAMPLE_AT = 15", "slicer samples one cycle early")
mut("P2", U, "SAMPLE_AT = 16", "SAMPLE_AT = 17", "slicer samples one cycle late")
mut("P3", U, "CELL = (33, 33, 34)", "CELL = (33, 33, 33)", "no long cell: rollover drifts against a 33.33-cycle bit")
mut("P4", U, "CELL = (33, 33, 34)", "CELL = (32, 33, 34)", "cell 0 one cycle short")
mut("P5", U, "CELL = (33, 33, 34)", "CELL = (33, 33, 35)", "long cell one cycle too long")
mut("P6", U, "IDLE_CELLS = 8", "IDLE_CELLS = 6", "LOCKOUT ends after 198 instead of 264 quiet cycles")
mut("P7", U, "self.quiet >= IDLE_CELLS * 33", "self.quiet > IDLE_CELLS * 33", "LOCKOUT threshold off by one (>)")
mut("P8", U, "bit = 1 if lv == self.prev else 0", "bit = 0 if lv == self.prev else 1", "NRZI inverted")
mut("P9", U, "                if self.smp:                  # previous cell already sampled: this edge starts the next cell",
    "                if True:                      # previous cell already sampled: this edge starts the next cell",
    "missing smp gating: every edge advances the cell index")
mut("P10", U, "                    if se0:\n                        self.s_q, t_next, self.st = 1, 1, \"IDLE\"",
    "                    if False:\n                        self.s_q, t_next, self.st = 1, 1, \"IDLE\"", "SE0 not recognised as EOP in RUN")
mut("P11", U, "if lv != SYNC_LEVELS[self.ncell] or se0:", "if lv != SYNC_LEVELS[self.ncell]:", "SE0 ignored while matching SYNC")
mut("P12", U, "if self.ones == 6:", "if self.ones == 5:", "stuffing after five 1s")
mut("P13", U, "if self.ones == 6:", "if self.ones == 7:", "stuffing after seven 1s")
mut("P14", U, "SYNC_LEVELS = (1, 0, 1, 0, 1, 0, 1, 1)", "SYNC_LEVELS = (1, 0, 1, 0, 1, 0, 1, 0)", "SYNC pattern ends in 0")
mut("P15", U, "            if edge and dp == 1:\n                self.st, self.cnt",
    "            if edge:\n                self.st, self.cnt", "falling D+ edge also starts SYNC")
mut("P16", U, "self.ones = self.ones + 1 if bit else 0", "self.ones = self.ones + 1 if bit else 1",
    "ones counter restarts at 1 after a 0")
mut("P17", U, "self.d_q, t_next = bit, 1", "self.d_q, t_next = 1 - bit, 1", "DATA inverted")
mut("P18", U, "                            if bit:                        # stuff violation",
    "                            if not bit:                    # stuff violation", "stuff violation test inverted")
mut("P19", U, "self.s_q = self.r_q = t_next = 1", "self.s_q = t_next = 1", "stuff violation does not raise ERR")
mut("P20", U, "                    self.idx, self.smp = (self.idx + 1) % 3, False",
    "                    self.idx, self.smp = (self.idx + 1) % 2, False", "cell index wraps at 2 after an edge")
mut("P21", U, "        if not en:\n            self.st = \"IDLE\"", "        if False:\n            self.st = \"IDLE\"", "enable ignored")
mut("P22", U, "                        self.ncell += 1", "                        self.ncell += 2", "SYNC cell counter steps by 2")

# ------------------------------------------------------------------ usb_tx.asm
X = "examples/usb_tx.asm"
mut("T1", X, ".default EXT2, 1", ".default EXT2, 0", "no extra cycle after bit 2 (rate wrong by 0.25 %)")
mut("T2", X, ".default EXT5, 1", ".default EXT5, 0", "no extra cycle after bit 5")
mut("T3", X, r"LDI c0, 6 @26+\ext", r"LDI c0, 5 @26+\ext", "0 bit reloads the ones counter with 5")
mut("T4", X, r"DJNZ c0, bit\kn @23+\ext", r"DJNZ c0, bit\kn @22+\ext", "1-bit cell one cycle short")
mut("T5", X, "PINS toggle uio6 uio7 @8 ", "PINS toggle uio6 @8 ", "stuffed 0 toggles only D+")
mut("T6", X, r"JMP bit\kn @20", r"JMP bit\kn @21", "stuffed cell one cycle long")
mut("T7", X, "LDI c0, 7 @5", "LDI c0, 6 @5", "SYNC has 6 toggles")
mut("T8", X, "PINS toggle uio6 uio7 @26         ; SYNC", "PINS toggle uio6 uio7 @25         ; SYNC", "SYNC cell 32 cycles")
mut("T9", X, "LDI c0, 6 @2                      ; ones countdown for the data",
    "LDI c0, 5 @2                      ; ones countdown for the data", "first data byte starts with ones count 5")
mut("T10", X, "NOP @41", "NOP @40", "last SYNC cell one cycle short")
mut("T11", X, "HPULL @11", "HPULL @10", "byte fetch slot one cycle short")
mut("T12", X, "PINS low uio6 uio7 @20            ; EOP", "PINS low uio6 uio7 @19            ; EOP", "EOP SE0 starts one cycle early")
mut("T13", X, "PINS high uio7 @65", "PINS high uio7 @64", "SE0 one cycle short")
mut("T14", X, "PINS float uio6 uio7 @32", "NOP @32", "bus never released")
mut("T15", X, "DJNZ c1, next_byte @2", "NOP @2", "byte counter ignored: only one byte per packet")
mut("T16", X, "    HOT 3, 4, 0", "    HOT 3, 4, 1", "bit 3 cell stretched")
mut("T17", X, "    COLD 6, 7, 0", "    COLD 6, 7, 1", "1-bit 6 cell stretched")
mut("T18", X, "    HOT 2, 3, EXT2", "    HOT 2, 3, 0", "bit 2 (0 path) loses its extra cycle")
mut("T19", X, "    COLD 5, 6, EXT5", "    COLD 5, 6, 0", "bit 5 (1 path) loses its extra cycle")
mut("T20", X, r"JSB sr[\k]==1, one\k @2", r"JSB sr[\k]==0, one\k @2", "bit polarity inverted (bits 0..6)")
mut("T21", X, "JSB sr[7]==1, one7 @2", "JSB sr[7]==0, one7 @2", "bit 7 polarity inverted")
mut("T22", X, "DJNZ c0, ob @5", "DJNZ c0, ob @6", "bit 7 one-path one cycle long")
mut("T23", X, "    HOT 5, 6, EXT5", "    HOT 5, 6, 0", "bit 5 (0 path) loses its extra cycle")
mut("T24", X, "    COLD 2, 3, EXT2", "    COLD 2, 3, 0", "bit 2 (1 path) loses its extra cycle")

# ------------------------------------------------------------------ usb_rx.asm
X = "examples/usb_rx.asm"
mut("X1", X, "SAMPLE ui5 lsb @2             ; SR <- {DATA, SR[7:1]}", "SAMPLE ui5 msb @2             ; SR <- {DATA, SR[7:1]}",
    "bits shifted in MSB first")
mut("X2", X, "    LDI c1, 8 @2                  ; bits left in the current byte", "    LDI c1, 7 @2                  ; bits left in the current byte",
    "7 bits per first byte")
mut("X3", X, "    HPUSH @2                      ; one full byte", "    NOP @2                        ; one full byte", "bytes never pushed")
mut("X4", X, "    JHI ui7, bad @2               ; stuff error", "    NOP @2                        ; stuff error", "ERR ignored")
mut("X5", X, "    JSB sr[3]==1, ok @2           ; c1 == 8 only when no partial byte is pending",
    "    JSB sr[2]==1, ok @2           ; c1 == 8 only when no partial byte is pending", "partial-byte test on the wrong bit")
mut("X6", X, "bad:\n    PINS toggle uo5 @2", "bad:\n    NOP @2", "bad-packet flag never toggled")
mut("X7", X, "ok:\n    PINS toggle uo6 @2", "ok:\n    NOP @2", "packet-end flag never toggled")

# ------------------------------------------------------------------ usb_device_ack.asm
X = "examples/usb_device_ack.asm"
mut("D1", X, ".default PAD_D0, 76", ".default PAD_D0, 75", "DATA0 turnaround 139 instead of 140")
mut("D2", X, ".default PAD_IN, 46", ".default PAD_IN, 47", "IN turnaround 141 instead of 140")
mut("D3", X, "XORI sr, 0xC3", "XORI sr, 0xC2", "DATA0 PID mismatch")
mut("D4", X, "XORI sr, 0x4B", "XORI sr, 0x4A", "DATA1 PID mismatch")
mut("D5", X, "XORI sr, 0x69", "XORI sr, 0x68", "IN PID mismatch")
mut("D6", X, "LDI c1, 0xD2 @PAD_D0", "LDI c1, 0xD3 @PAD_D0", "wrong ACK PID after DATA0")
mut("D7", X, "LDI c1, 0x5A @PAD_IN", "LDI c1, 0x5B @PAD_IN", "wrong NAK PID")
mut("D8", X, "    ADDI sr, 1 @2\n    MOV c1, sr @2\n    DJNZ c1, try_d1 @2", "    ADDI sr, 2 @2\n    MOV c1, sr @2\n    DJNZ c1, try_d1 @2",
    "DATA0 compare off by one")
mut("D9", X, "    MOV c0, sr @2                 ; ... and remembered for the response", "    NOP @2                        ; ... and remembered for the response",
    "PID never remembered")
mut("D10", X, "    LDI c0, 0 @2                  ; no PID yet (0x00 is never a valid PID byte)",
    "    NOP @2                        ; no PID yet (0x00 is never a valid PID byte)", "stale PID survives into an empty packet")
mut("D11", X, "    JHI ui7, bad @2               ; stuff violation", "    NOP @2                        ; stuff violation", "ERR ignored")
mut("D12", X, "    JSB sr[3]==1, good @2         ; c1 == 8 only when no partial byte is pending",
    "    JMP good @2                   ; c1 == 8 only when no partial byte is pending", "partial byte not rejected")
mut("D13", X, "    WAIT high uio7 @2             ; host EOP over: D- back to J", "    NOP @2                        ; host EOP over: D- back to J",
    "no wait for the end of the host EOP")
mut("D14", X, "bad:\n    PINS toggle uo5 @2\n", "bad:\n    NOP @2\n", "bad-packet flag never toggled")
mut("D15", X, "good:\n    PINS toggle uo6 @2\n", "good:\n    NOP @2\n", "packet-end flag not toggled for good packets")
mut("D16", X, "JMP bit0 @47", "JMP bit0 @46", "first data edge one cycle early")
mut("D17", X, "    MOV sr, c1 @2                 ; the response byte", "    NOP @2                        ; the response byte",
    "response byte not loaded")
mut("D18", X, "    DJNZ c1, start @2             ; not a PID we answer", "    NOP @2                        ; not a PID we answer",
    "every other PID is NAKed")
mut("D19", X, "    HPUSH @2                      ; PID to the host", "    NOP @2                        ; PID to the host", "PID byte not pushed")
mut("D20", X, "    PINS float uio6 uio7 @32      ; release after one J bit time", "    NOP @32                       ; release after one J bit time",
    "bus not released after the response")
mut("D21", X, "    DJNZ c1, try_d1 @2", "    DJNZ c1, try_in @2", "DATA1 test skipped")
mut("D22", X, "    DJNZ c1, try_in @2\n    LDI c1, 0xD2 @PAD_D1", "    DJNZ c1, start @2\n    LDI c1, 0xD2 @PAD_D1",
    "IN test skipped after DATA1")
mut("D23", X, "    LDI c1, 8 @2\n    WAIT rise ui4 @2              ; first bit of byte 1", "    LDI c1, 7 @2\n    WAIT rise ui4 @2              ; first bit of byte 1",
    "second byte has 7 bits")
mut("D24", X, ".default PAD_D1, 58", ".default PAD_D1, 57", "DATA1 turnaround 139 instead of 140")

# ------------------------------------------------------------------ i2c_master.asm (repeated START)
X = "examples/i2c_master.asm"
mut("I1", X, "    JSB sr[7]==1, wr_sr @2", "    JSB sr[6]==1, wr_sr @2", "write: Sr flag read from bit 6")
mut("I2", X, "    JSB sr[7]==1, rd_sr @2", "    JSB sr[6]==1, rd_sr @2", "read: Sr flag read from bit 6")
mut("I3", X, "wr_sr:\n    ANDI sr, 0x7F @2", "wr_sr:\n    NOP @2", "write: flag bit left in the count")
mut("I4", X, "rd_sr:\n    ANDI sr, 0x7F @2", "rd_sr:\n    NOP @2", "read: flag bit left in the count")
mut("I5", X, ".default RS_LOW, TL-19", ".default RS_LOW, TL-40", "SCL low time before Sr far too short")
mut("I6", X, ".default RS_LOW, TL-19", ".default RS_LOW, TL-20", "SCL low time before Sr one cycle short")
mut("I7", X, ".default RS_SU, TH-10", ".default RS_SU, TH-30", "tSU;STA of Sr far too short")
mut("I8", X, ".default RS_SU, TH-10", ".default RS_SU, TH-11", "tSU;STA of Sr one cycle short")
mut("I9", X, "    WAIT high uio2 @2             ; clock stretching\n    JMP do_start @RS_SU",
    "    NOP @2                        ; clock stretching\n    JMP do_start @RS_SU", "no clock-stretch wait at the Sr SCL rise")
mut("I10", X, "hold:\n    HPULL @2", "hold:\n    NOP @2", "Sr does not fetch the next address byte")
mut("I11", X, "WRITE_BODY wl_h, hold", "WRITE_BODY wl_h, stop", "write with Sr flag ends with STOP")
mut("I12", X, "READ_BODY rl_h, am_h, hold", "READ_BODY rl_h, am_h, stop", "read with Sr flag ends with STOP")
mut("I13", X, "    JMP do_start @RS_SU", "    JMP cmd @RS_SU", "Sr re-enters at cmd (eats a second byte)")
mut("I14", X, "    PINS low uo7 @2               ; clear error flag\n    MOV c1, sr @2\n    PINS float uio2 @RS_LOW",
    "    NOP @2                        ; clear error flag\n    MOV c1, sr @2\n    PINS float uio2 @RS_LOW", "Sr does not clear the NACK flag",
    equiv="CFG-equivalent: uo7 is cleared in init: and cmd:, set high only in nack: which always exits via stop: -> cmd:; hold: is reachable only from ACKed WRITE_BODY/READ_BODY, so uo7 is already 0 on every path into hold:")
mut("I15", X, "    PINS float uio2 @RS_LOW       ; SCL rises; SDA is already released (high)",
    "    PINS float uio3 @RS_LOW       ; SCL rises; SDA is already released (high)", "Sr releases SDA instead of raising SCL")
mut("I16", X, "    MOV c1, sr @2\n    PINS float uio2 @RS_LOW", "    NOP @2\n    PINS float uio2 @RS_LOW", "Sr loses the R/W bit (c1 stale)")


def apply(tmp_sw, rel, old, new):
    path = os.path.join(tmp_sw, rel)
    with open(path, newline="") as f:
        s = f.read()
    n = s.count(old)
    if n != 1:
        return "STALE (old text matches %d times)" % n
    with open(path, "w", newline="") as f:
        f.write(s.replace(old, new))
    return None


def check_valid(tmp_sw, rel):
    """Mutated source must still compile / assemble."""
    path = os.path.join(tmp_sw, rel)
    if rel.endswith(".py"):
        code = "import py_compile,sys; py_compile.compile(sys.argv[1], doraise=True)"
        args = [sys.executable, "-I", "-c", code, path]
    else:
        code = ("import sys; sys.path.insert(0, sys.argv[1]); import ttpe_asm as a; "
                "a.assemble(open(sys.argv[2]).read())")
        args = [sys.executable, "-I", "-c", code, tmp_sw, path]
    r = subprocess.run(args, capture_output=True, text=True)
    return None if r.returncode == 0 else (r.stderr.strip().splitlines() or ["error"])[-1]


def run_suite(tmp):
    r = subprocess.run([sys.executable, "-I", "-m", "unittest", "discover", "-s", os.path.join(tmp, "sw", "tests"), "-f"],
                       cwd=tmp, capture_output=True, text=True, timeout=900)
    first = None
    m = re.search(r"^(?:FAIL|ERROR): (\S+) \(([^)]*)\)", r.stderr, re.M)
    if m:
        first = m.group(2)
    return r.returncode, first, r.stderr


def copy_sw(tmp):
    shutil.copytree(HERE, os.path.join(tmp, "sw"),
                    ignore=shutil.ignore_patterns("__pycache__", "build", "*.pyc", "*.zip"))
    sv_pkg = os.path.join(HERE, "..", "src", "ttpe_isa_pkg.sv")
    if os.path.exists(sv_pkg):
        os.makedirs(os.path.join(tmp, "src"), exist_ok=True)
        shutil.copy2(sv_pkg, os.path.join(tmp, "src", "ttpe_isa_pkg.sv"))


def run_one(item):
    mid, rel, old, new, desc, equiv = item
    tmp = tempfile.mkdtemp(prefix="ttpe_mut_")
    try:
        copy_sw(tmp)
        err = apply(os.path.join(tmp, "sw"), rel, old, new)
        if err:
            return mid, rel, desc, "STALE", err, equiv
        bad = check_valid(os.path.join(tmp, "sw"), rel)
        if bad:
            return mid, rel, desc, "INVALID", bad, equiv
        rc, first, _ = run_suite(tmp)
        return mid, rel, desc, ("SURVIVED" if rc == 0 else "KILLED"), first or "", equiv
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def run_suite_base():
    tmp = tempfile.mkdtemp(prefix="ttpe_mut_base_")
    try:
        copy_sw(tmp)
        return run_suite(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--only", help="comma-separated mutant ids")
    ap.add_argument("-j", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--json", help="write the result table as JSON")
    a = ap.parse_args()
    items = list(M)
    if a.only:
        ids = set(a.only.split(","))
        items = [m for m in items if m[0] in ids]
    if a.list:
        for mid, rel, _o, _n, desc, equiv in items:
            print("%-4s %-28s %s%s" % (mid, rel, desc, "   [equivalent: %s]" % equiv if equiv else ""))
        print(len(items), "mutants")
        return 0
    t0 = time.time()
    print("baseline ...", end=" ", flush=True)
    rc, _first, err = run_suite_base()
    if rc != 0:
        print("FAILED - fix the unit tests first\n" + err[-2000:])
        return 2
    print("ok (%.1f s)" % (time.time() - t0))
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.j) as ex:
        res = list(ex.map(run_one, items))
    w = max(len(r[1]) for r in res)
    counts = {}
    for mid, rel, desc, status, info, equiv in res:
        shown = status if not (status == "SURVIVED" and equiv) else "SURVIVED(equivalent)"
        counts[shown] = counts.get(shown, 0) + 1
        print("%-4s %-*s %-20s %s%s" % (mid, w, rel, shown, desc, "  <- " + info if info and status != "KILLED" else ""))
    valid = [r for r in res if r[3] in ("KILLED", "SURVIVED")]
    killed = sum(1 for r in valid if r[3] == "KILLED")
    surv = [r for r in valid if r[3] == "SURVIVED" and not r[5]]
    equiv = [r for r in valid if r[3] == "SURVIVED" and r[5]]
    print("\n%d mutants: %s" % (len(res), ", ".join("%s %d" % kv for kv in sorted(counts.items()))))
    print("valid mutants %d, killed %d, survived %d (+%d argued equivalent): kill rate over non-equivalent mutants %.1f %%"
          % (len(valid), killed, len(surv), len(equiv), 100.0 * killed / max(1, killed + len(surv))))
    for r in equiv:
        print("  equivalent %s: %s" % (r[0], r[5]))
    if a.json:
        with open(a.json, "w") as f:
            json.dump([dict(id=r[0], file=r[1], desc=r[2], status=r[3], info=r[4], equiv=r[5]) for r in res], f, indent=1)
    bad = [r for r in res if r[3] in ("STALE", "INVALID")]
    return 1 if (surv or bad) else 0


if __name__ == "__main__":
    sys.exit(main())
