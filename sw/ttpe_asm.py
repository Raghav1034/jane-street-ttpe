#!/usr/bin/env python3
"""
ttpe_asm.py - assembler for the TTPE ISA v0.2.  Pure Python 3, no dependencies.

Usage:
    python3 ttpe_asm.py prog.asm -o prog.hex          # $readmemh image, 512 lines
    python3 ttpe_asm.py prog.asm --bin prog.bin       # little-endian 32-bit words
    python3 ttpe_asm.py prog.asm --mpy prog.py        # MicroPython list for the RP2350 host
    python3 ttpe_asm.py prog.asm --lst                # listing to stdout
    options: --allow-short-delta   accept delta < 2 (successor is then late, see ISA.md)
             -D NAME=VALUE         predefine a symbol (like .equ)

Syntax summary (full reference: docs/ISA.md):
    label:                         labels end with ':'
    ; comment   // comment
    .equ NAME, expr                constants (must be defined before use in .org/.if/DELAY)
    .org expr                      set the location counter
    .word expr                     raw 32-bit word
    .default NAME, expr            like .equ but keeps a value given with -D
    .if expr / .else / .endif      conditional assembly (nesting allowed)
    .macro NAME p1 p2 ... / .endm  text macros; use \\p1 for a parameter, \\@ for a unique id
    <mnemonic> operands [@ delta]  '@ N' = wait N cycles BEFORE the action
    DELAY n                        pseudo: NOP chain whose commit spacing is exactly n cycles

Pins: uio0..uio7 (bidirectional), ui0..ui7 (inputs), uo0..uo7 (outputs); 'uio[3]' also accepted.
"""

import argparse
import ast
import operator
import re
import sys

import ttpe_isa as isa

HALT_WORD = isa.encode(isa.OPCODES["HALT"])


class AsmError(Exception):
    def __init__(self, msg, lineno=None, text=None):
        self.msg, self.lineno, self.text = msg, lineno, text
        loc = "line %d: " % lineno if lineno else ""
        super().__init__(loc + msg + ("\n    " + text.strip() if text else ""))


# ------------------------------------------------------------ expressions
_BIN = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
        ast.FloorDiv: operator.floordiv, ast.Div: operator.floordiv,
        ast.Mod: operator.mod, ast.LShift: operator.lshift,
        ast.RShift: operator.rshift, ast.BitAnd: operator.and_,
        ast.BitOr: operator.or_, ast.BitXor: operator.xor}
_UN = {ast.USub: operator.neg, ast.UAdd: operator.pos, ast.Invert: operator.invert}
_CMP = {ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt,
        ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge}


def eval_expr(text, symbols):
    """Integer-only expression evaluator (no calls, no attributes)."""
    try:
        tree = ast.parse(text.strip(), mode="eval")
    except SyntaxError:
        raise AsmError("bad expression '%s'" % text.strip())

    def ev(n):
        if isinstance(n, ast.Expression):
            return ev(n.body)
        if isinstance(n, ast.Constant) and isinstance(n.value, int) \
                and not isinstance(n.value, bool):
            return n.value
        if isinstance(n, ast.Name):
            if n.id not in symbols:
                raise AsmError("undefined symbol '%s'" % n.id)
            return symbols[n.id]
        if isinstance(n, ast.BinOp) and type(n.op) in _BIN:
            r = ev(n.right)
            if type(n.op) in (ast.FloorDiv, ast.Div, ast.Mod) and r == 0:
                raise AsmError("division by zero in '%s'" % text.strip())
            return _BIN[type(n.op)](ev(n.left), r)
        if isinstance(n, ast.UnaryOp) and type(n.op) in _UN:
            return _UN[type(n.op)](ev(n.operand))
        if isinstance(n, ast.Compare) and len(n.ops) == 1 and type(n.ops[0]) in _CMP:
            return int(_CMP[type(n.ops[0])](ev(n.left), ev(n.comparators[0])))
        raise AsmError("unsupported expression '%s'" % text.strip())

    return ev(tree)


# ----------------------------------------------------------------- program
class Program:
    def __init__(self):
        self.words = [HALT_WORD] * isa.PROG_WORDS   # unused memory = HALT
        self.used = [False] * isa.PROG_WORDS
        self.symbols = {}
        self.listing = []          # (addr, word, lineno, source text)
        self.warnings = []

    def image(self):
        return list(self.words)

    def to_hex(self):
        return "\n".join("%08X" % w for w in self.words) + "\n"

    def to_bin(self):
        return b"".join(w.to_bytes(4, "little") for w in self.words)

    def to_mpy(self, name="PROGRAM"):
        last = max([i for i, u in enumerate(self.used) if u] or [0])
        body = ",\n".join("    0x%08X" % w for w in self.words[:last + 1])
        return "%s = [\n%s,\n]\n" % (name, body)

    def to_listing(self):
        out = []
        for addr, word, lineno, text in self.listing:
            out.append("%03X  %08X  %-34s ; L%-4d %s" %
                       (addr, word, isa.disasm(word, addr), lineno, text.strip()))
        return "\n".join(out)



# ------------------------------------------------------------ macro preprocessor
def _preprocess(source):
    """Expand .macro/.endm. Returns a list of (lineno, text) with macros expanded.
    Errors point at the invocation line."""
    macros = {}
    out = []
    lines = list(enumerate(source.splitlines(), 1))
    i = 0
    uid = [0]

    def strip_comment(t):
        return re.split(r";|//", t, maxsplit=1)[0].strip()

    # pass A: collect definitions
    body_lines = []
    while i < len(lines):
        lineno, raw = lines[i]
        t = strip_comment(raw)
        parts = t.split(None, 1)
        if parts and parts[0].lower() == ".macro":
            if len(parts) < 2:
                raise AsmError(".macro needs a name", lineno, raw)
            toks = parts[1].replace(",", " ").split()
            name, params = toks[0].lower(), toks[1:]
            j, body = i + 1, []
            while j < len(lines):
                tj = strip_comment(lines[j][1])
                if tj.lower() == ".endm":
                    break
                body.append(lines[j][1])
                j += 1
            else:
                raise AsmError("missing .endm for macro '%s'" % name, lineno, raw)
            if name in macros:
                raise AsmError("macro '%s' defined twice" % name, lineno, raw)
            macros[name] = (params, body)
            i = j + 1
            continue
        body_lines.append((lineno, raw))
        i += 1

    # pass B: expand invocations (recursively)
    def expand(lineno, raw, depth):
        if depth > 16:
            raise AsmError("macro nesting too deep (recursive macro?)", lineno, raw)
        t = strip_comment(raw)
        label = ""
        m = re.match(r"^((?:[A-Za-z_]\w*\s*:\s*)*)(.*)$", t)
        if m:
            label, t = m.group(1), m.group(2).strip()
        parts = t.split(None, 1)
        if not parts or parts[0].lower() not in macros:
            return [(lineno, raw)]
        params, body = macros[parts[0].lower()]
        args = [a.strip() for a in parts[1].split(",")] if len(parts) > 1 else []
        if len(args) != len(params):
            raise AsmError("macro '%s' takes %d argument(s), got %d"
                           % (parts[0], len(params), len(args)), lineno, raw)
        uid[0] += 1
        res = [(lineno, label)] if label.strip() else []
        for b in body:
            for p, a in sorted(zip(params, args), key=lambda x: -len(x[0])):
                b = re.sub(r"\\" + re.escape(p) + r"\b", lambda _m, a=a: a, b)
            b = b.replace("\\@", "_m%d" % uid[0])
            res.extend(expand(lineno, b, depth + 1))
        return res

    for lineno, raw in body_lines:
        out.extend(expand(lineno, raw, 0))
    return out

# ---------------------------------------------------------------- assembler
_PIN_RE = re.compile(r"^(uio|ui|uo)\s*(?:\[\s*(\d+)\s*\]|(\d+))$", re.I)
_JSB_RE = re.compile(r"^sr\s*\[\s*(\d+)\s*\]\s*==\s*([01])\s*,\s*(.+)$", re.I)


def _pin(tok, kind):
    """kind: 'src' (uio/ui), 'snk' (uio/uo) or 'any'. Returns (bank_name, n, index)."""
    m = _PIN_RE.match(tok.strip())
    if not m:
        raise AsmError("expected a pin like uio3 / ui5 / uo2, got '%s'" % tok.strip())
    bank = m.group(1).lower()
    n = int(m.group(2) if m.group(2) is not None else m.group(3))
    if n > 7:
        raise AsmError("pin number out of range: %s" % tok.strip())
    if kind == "src" and bank == "uo":
        raise AsmError("'%s' is an output-only pin and cannot be read" % tok.strip())
    if kind == "snk" and bank == "ui":
        raise AsmError("'%s' is an input-only pin and cannot be driven" % tok.strip())
    idx = n if bank == "uio" else 8 + n
    return bank, n, idx


def _split_ops(s):
    return [t.strip() for t in s.split(",")] if s.strip() else []


class _Item:
    __slots__ = ("lineno", "text", "addr", "kind", "mn", "ops", "delta", "n_words")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))


def assemble(source, allow_short_delta=False, defines=None):
    prog = Program()
    sym = dict(defines or {})
    labels = {}
    items = []

    # ---------------- pass 1: parse, conditional assembly, addresses
    loc = 0
    cond_stack = []          # entries: [active_now, taken_any, parent_active]

    def active():
        return all(c[0] for c in cond_stack)

    for lineno, raw in _preprocess(source):
        text = raw
        try:
            line = re.split(r";|//", raw, maxsplit=1)[0].strip()
            if not line:
                continue
            m = re.match(r"^([A-Za-z_]\w*)\s*:\s*(.*)$", line)
            while m:
                if active():
                    if m.group(1) in labels or m.group(1) in sym:
                        raise AsmError("duplicate symbol '%s'" % m.group(1))
                    labels[m.group(1)] = loc
                line = m.group(2).strip()
                m = re.match(r"^([A-Za-z_]\w*)\s*:\s*(.*)$", line)
            if not line:
                continue

            parts = line.split(None, 1)
            mn = parts[0]
            rest = parts[1] if len(parts) > 1 else ""

            # ---- conditional assembly
            if mn.lower() == ".if":
                if active():
                    val = bool(eval_expr(rest, {**sym}))
                    cond_stack.append([val, val])
                else:
                    cond_stack.append([False, True])   # nested in dead block
                continue
            if mn.lower() == ".else":
                if not cond_stack:
                    raise AsmError(".else without .if")
                top = cond_stack[-1]
                top[0] = not top[1] if all(c[0] for c in cond_stack[:-1]) else False
                top[1] = True
                continue
            if mn.lower() == ".endif":
                if not cond_stack:
                    raise AsmError(".endif without .if")
                cond_stack.pop()
                continue
            if not active():
                continue

            # ---- directives
            if mn.lower() == ".equ":
                nm, _, ex = rest.partition(",")
                nm = nm.strip()
                if not re.match(r"^[A-Za-z_]\w*$", nm):
                    raise AsmError("bad .equ name '%s'" % nm)
                if nm in sym or nm in labels:
                    raise AsmError("duplicate symbol '%s'" % nm)
                sym[nm] = eval_expr(ex, {**sym})
                continue
            if mn.lower() == ".default":
                nm, _, ex = rest.partition(",")
                nm = nm.strip()
                if not re.match(r"^[A-Za-z_]\w*$", nm):
                    raise AsmError("bad .default name '%s'" % nm)
                if nm not in sym:
                    if nm in labels:
                        raise AsmError("duplicate symbol '%s'" % nm)
                    sym[nm] = eval_expr(ex, {**sym})
                continue
            if mn.lower() == ".org":
                loc = eval_expr(rest, {**sym})
                if not 0 <= loc < isa.PROG_WORDS:
                    raise AsmError(".org out of range")
                continue

            # ---- split off '@ delta'
            delta = None
            if "@" in rest:
                rest, _, delta = rest.rpartition("@")
                rest = rest.strip()
                delta = delta.strip()
                if not delta:
                    raise AsmError("missing delta expression after '@'")

            mnu = mn.upper()
            if mnu == ".WORD":
                items.append(_Item(lineno=lineno, text=text, addr=loc, kind="word",
                                   mn=mnu, ops=rest, delta=None, n_words=1))
                loc += 1
            elif mnu == "DELAY":
                n = eval_expr(rest, {**sym})
                if n < 1:
                    raise AsmError("DELAY needs n >= 1")
                k = -(-n // (isa.MAX_DELTA + 1))
                base, rem = divmod(n, k)
                chunks = [base + 1] * rem + [base] * (k - rem)
                for c in chunks:
                    items.append(_Item(lineno=lineno, text=text, addr=loc,
                                       kind="instr", mn="NOP", ops="",
                                       delta=str(c - 1), n_words=1))
                    loc += 1
            else:
                items.append(_Item(lineno=lineno, text=text, addr=loc, kind="instr",
                                   mn=mnu, ops=rest, delta=delta, n_words=1))
                loc += 1
            if loc > isa.PROG_WORDS:
                raise AsmError("program exceeds %d words" % isa.PROG_WORDS)
        except AsmError as e:
            raise AsmError(e.msg, lineno, text)

    if cond_stack:
        raise AsmError("missing .endif")
    allsym = {**sym, **labels}
    prog.symbols = dict(allsym)

    # ---------------- pass 2: encode
    for it in items:
        try:
            if it.kind == "word":
                word = eval_expr(it.ops, allsym) & 0xFFFFFFFF
            else:
                word = _encode_instr(it, allsym, allow_short_delta)
            if prog.used[it.addr]:
                raise AsmError("address 0x%03X written twice" % it.addr)
            prog.words[it.addr] = word
            prog.used[it.addr] = True
            prog.listing.append((it.addr, word, it.lineno, it.text))
        except AsmError as e:
            raise AsmError(e.msg, it.lineno, it.text)
    prog.listing.sort()
    return prog


def _target(expr, syms, addr, same_page):
    t = eval_expr(expr, syms)
    if not 0 <= t < isa.PROG_WORDS:
        raise AsmError("jump target 0x%X out of range 0..0x1FF" % t)
    if same_page and (t >> 8) != (addr >> 8):
        raise AsmError(
            "conditional branch from 0x%03X cannot reach 0x%03X (different 256-word "
            "page); branch to a local 'JMP' trampoline instead" % (addr, t))
    return t


_REG_ALIASES = {
    ("LDI", "sr"): "LDSR", ("LDI", "c0"): "LDC0", ("LDI", "c1"): "LDC1",
}
_MOV = {("c0", "sr"): "MVC0", ("c1", "sr"): "MVC1",
        ("sr", "c0"): "MVSR0", ("sr", "c1"): "MVSR1"}


def _imm8(expr, syms):
    v = eval_expr(expr, syms)
    if not -128 <= v <= 255:
        raise AsmError("immediate %d does not fit in 8 bits" % v)
    return v & 0xFF


def _encode_instr(it, syms, allow_short):
    mn, rest = it.mn, it.ops
    d = eval_expr(it.delta, syms) if it.delta is not None else 0
    if not 0 <= d <= isa.MAX_DELTA:
        raise AsmError("delta %d out of range 0..65535" % d)
    if d < isa.MIN_SAFE_DELTA and mn not in ("JMP", "HALT") and not allow_short:
        raise AsmError(
            "TIMING: delta %d < %d makes the NEXT instruction late by %d cycle(s) "
            "(fetch latency is %d cycles). Use delta >= %d, or --allow-short-delta."
            % (d, isa.MIN_SAFE_DELTA, isa.MIN_SAFE_DELTA - d, isa.FETCH_LATENCY,
               isa.MIN_SAFE_DELTA))
    ops = _split_ops(rest)

    def need(n):
        if len(ops) != n:
            raise AsmError("%s takes %d operand(s), got %d" % (mn, n, len(ops)))

    OP = isa.OPCODES
    if mn in ("NOP", "HPUSH", "HPULL", "HALT"):
        need(0)
        return isa.encode(OP[mn], 0, 0, d)

    if mn == "PINS":
        toks = rest.replace(",", " ").split()
        if len(toks) < 2:
            raise AsmError("PINS needs a mode and at least one pin")
        mode = toks[0].lower()
        if mode not in isa.PINS_MODES:
            raise AsmError("PINS mode must be low/high/float/toggle, got '%s'" % toks[0])
        banks, mask = set(), 0
        for t in toks[1:]:
            bank, n, _ = _pin(t, "snk")
            banks.add(bank)
            mask |= 1 << n
        if len(banks) != 1:
            raise AsmError("PINS pins must all be in the same bank (uio or uo)")
        bank = banks.pop()
        if bank == "uo" and mode == "float":
            raise AsmError("uo pins have no output enable; 'float' is not allowed")
        a = (1 << 3 if bank == "uo" else 0) | isa.PINS_MODES[mode]
        return isa.encode(OP["PINS"], a, mask, d)

    if mn == "WAIT":
        toks = rest.replace(",", " ").split()
        if len(toks) != 2 or toks[0].lower() not in isa.WAIT_TYPES:
            raise AsmError("usage: WAIT low|high|fall|rise <pin>")
        _, _, idx = _pin(toks[1], "src")
        return isa.encode(OP["WAIT"], idx, isa.WAIT_TYPES[toks[0].lower()], d)

    if mn == "JMP":
        need(1)
        t = _target(ops[0], syms, it.addr, same_page=False)
        return isa.encode(OP["JMP"], (t >> 8) & 1, t & 0xFF, d)

    if mn in ("JHI", "JLO"):
        need(2)
        _, _, idx = _pin(ops[0], "src")
        t = _target(ops[1], syms, it.addr, same_page=True)
        return isa.encode(OP[mn], idx, t & 0xFF, d)

    if mn == "SHOUT":
        toks = rest.replace(",", " ").split()
        if not toks:
            raise AsmError("usage: SHOUT <pin> [lsb|msb] [shift] [od]")
        bank, n, idx = _pin(toks[0], "snk")
        p = 0
        for f in (t.lower() for t in toks[1:]):
            if f == "msb":
                p |= isa.SHOUT_MSB
            elif f == "lsb":
                pass
            elif f == "shift":
                p |= isa.SHOUT_SHIFT
            elif f == "od":
                if bank == "uo":
                    raise AsmError("open-drain needs a bidirectional (uio) pin")
                p |= isa.SHOUT_OD
            else:
                raise AsmError("unknown SHOUT flag '%s'" % f)
        return isa.encode(OP["SHOUT"], idx, p, d)

    if mn == "SAMPLE":
        toks = rest.replace(",", " ").split()
        if not toks or len(toks) > 2:
            raise AsmError("usage: SAMPLE <pin> [lsb|msb]")
        _, _, idx = _pin(toks[0], "src")
        p = 0
        if len(toks) == 2:
            if toks[1].lower() == "msb":
                p = isa.SAMPLE_MSB
            elif toks[1].lower() != "lsb":
                raise AsmError("SAMPLE flag must be lsb or msb")
        return isa.encode(OP["SAMPLE"], idx, p, d)

    if mn == "LDI":
        need(2)
        key = ("LDI", ops[0].lower())
        if key not in _REG_ALIASES:
            raise AsmError("LDI destination must be sr, c0 or c1")
        return isa.encode(OP["REG"], isa.REG_SUBOPS[_REG_ALIASES[key]],
                          _imm8(ops[1], syms), d)

    if mn == "MOV":
        need(2)
        key = (ops[0].lower(), ops[1].lower())
        if key not in _MOV:
            raise AsmError("MOV must be one of: c0,sr  c1,sr  sr,c0  sr,c1")
        return isa.encode(OP["REG"], isa.REG_SUBOPS[_MOV[key]], 0, d)

    if mn in ("XORI", "ANDI", "ORI", "ADDI"):
        need(2)
        if ops[0].lower() != "sr":
            raise AsmError("%s operates on sr" % mn)
        return isa.encode(OP["REG"], isa.REG_SUBOPS[mn], _imm8(ops[1], syms), d)

    if mn == "DJNZ":
        need(2)
        if ops[0].lower() not in ("c0", "c1"):
            raise AsmError("DJNZ counter must be c0 or c1")
        t = _target(ops[1], syms, it.addr, same_page=True)
        return isa.encode(OP["DJNZ"], int(ops[0][1]), t & 0xFF, d)

    if mn == "JSB":
        m = _JSB_RE.match(rest.strip())
        if not m:
            raise AsmError("usage: JSB sr[bit]==0|1, target")
        bit, val = int(m.group(1)), int(m.group(2))
        if bit > 7:
            raise AsmError("sr bit index must be 0..7")
        t = _target(m.group(3), syms, it.addr, same_page=True)
        return isa.encode(OP["JSB"], (val << 3) | bit, t & 0xFF, d)

    raise AsmError("unknown mnemonic '%s'" % mn)


# ---------------------------------------------------------------------- CLI
def main(argv=None):
    ap = argparse.ArgumentParser(description="TTPE assembler (ISA v0.2)")
    ap.add_argument("source")
    ap.add_argument("-o", "--hex", help="write $readmemh image")
    ap.add_argument("--bin", help="write little-endian binary image")
    ap.add_argument("--mpy", help="write MicroPython list")
    ap.add_argument("--lst", action="store_true", help="print listing")
    ap.add_argument("--allow-short-delta", action="store_true")
    ap.add_argument("-D", action="append", default=[], metavar="NAME=VALUE")
    a = ap.parse_args(argv)
    defs = {}
    for d in a.D:
        k, _, v = d.partition("=")
        defs[k] = int(v, 0)
    try:
        with open(a.source) as f:
            prog = assemble(f.read(), a.allow_short_delta, defs)
    except AsmError as e:
        print("error:", e, file=sys.stderr)
        return 1
    if a.hex:
        open(a.hex, "w").write(prog.to_hex())
    if a.bin:
        open(a.bin, "wb").write(prog.to_bin())
    if a.mpy:
        open(a.mpy, "w").write(prog.to_mpy())
    if a.lst or not (a.hex or a.bin or a.mpy):
        print(prog.to_listing())
    n = sum(prog.used)
    print("; %d / %d words used" % (n, isa.PROG_WORDS), file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
