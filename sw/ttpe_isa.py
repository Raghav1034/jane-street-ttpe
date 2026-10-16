#!/usr/bin/env python3
"""
ttpe_isa.py - TTPE instruction set definition, v0.2 (DRAFT, not frozen).

Single source of truth shared by the assembler (ttpe_asm.py), the cycle-accurate
instruction-set simulator (ttpe_iss.py) and the SystemVerilog package generator
(`python3 ttpe_isa.py --gen-sv > ../rtl/ttpe_isa_pkg.sv`).  Keep ALL encoding
knowledge here; the RTL must import the generated package, never re-type values.

Word layout (fixed by the project brief):
    [31:28] OP   4-bit opcode
    [27:24] A    4-bit field (pin select / mode / sub-op / counter select)
    [23:16] P    8-bit payload (mask / immediate / jump target low 8 bits)
    [15:0]  D    16-bit time delta: cycles to wait before the action
"""

import sys

# ------------------------------------------------------------------ opcodes
OPCODES = {
    "NOP":    0x0,
    "PINS":   0x1,
    "WAIT":   0x2,
    "JMP":    0x3,
    "JHI":    0x4,
    "JLO":    0x5,
    "SHOUT":  0x6,
    "SAMPLE": 0x7,
    "REG":    0x8,
    "DJNZ":   0x9,
    "HPUSH":  0xA,
    "HPULL":  0xB,
    "HALT":   0xC,
    "JSB":    0xD,
    # 0xE, 0xF reserved
}
OP_NAMES = {v: k for k, v in OPCODES.items()}

# PINS: A[3] = bank (0 = uio, 1 = uo_out), A[1:0] = mode, A[2] reserved (0)
PINS_LOW, PINS_HIGH, PINS_FLOAT, PINS_TOGGLE = 0, 1, 2, 3
PINS_MODES = {"low": PINS_LOW, "high": PINS_HIGH,
              "float": PINS_FLOAT, "toggle": PINS_TOGGLE}

# WAIT: A = source (0-7 uio_in, 8-15 ui_in), P[1:0] = type
WAIT_LOW, WAIT_HIGH, WAIT_FALL, WAIT_RISE = 0, 1, 2, 3
WAIT_TYPES = {"low": WAIT_LOW, "high": WAIT_HIGH,
              "fall": WAIT_FALL, "rise": WAIT_RISE}

# SHOUT: A = sink (0-7 uio, 8-15 uo_out), P[0] = msb-first, P[1] = shift after,
#        P[2] = open-drain (uio only)
SHOUT_MSB, SHOUT_SHIFT, SHOUT_OD = 0x01, 0x02, 0x04
# SAMPLE: A = source, P[0] = msb-first (shift left, insert at bit 0);
#         clear = lsb-first (shift right, insert at bit 7)
SAMPLE_MSB = 0x01

# REG sub-ops in A
REG_SUBOPS = {
    "LDSR": 0,   # SR <- imm
    "LDC0": 1,   # C0 <- imm
    "LDC1": 2,   # C1 <- imm
    "MVC0": 3,   # C0 <- SR
    "MVC1": 4,   # C1 <- SR
    "MVSR0": 5,  # SR <- C0
    "MVSR1": 6,  # SR <- C1
    "XORI": 8,   # SR <- SR ^ imm
    "ANDI": 9,   # SR <- SR & imm
    "ORI": 10,   # SR <- SR | imm
    "ADDI": 11,  # SR <- SR + imm (mod 256)
}
REG_NAMES = {v: k for k, v in REG_SUBOPS.items()}

# Pin index spaces
N_UIO, N_UI, N_UO = 8, 8, 8
SRC_UIO_BASE, SRC_UI_BASE = 0, 8      # input source index
SNK_UIO_BASE, SNK_UO_BASE = 0, 8      # output sink index

# Timing constants derived from the VERIFIED fetch controller (ttpe_sram_ctrl)
FETCH_LATENCY = 3     # accept (or pc_load) at cycle X -> next instr valid at X+3
MIN_SAFE_DELTA = 2    # delta below this makes the successor late (see ISA.md)
MAX_DELTA = 0xFFFF
PC_BITS = 9
PROG_WORDS = 1 << PC_BITS


def encode(op, a=0, p=0, d=0):
    assert 0 <= op <= 15 and 0 <= a <= 15 and 0 <= p <= 255 and 0 <= d <= MAX_DELTA
    return (op << 28) | (a << 24) | (p << 16) | d


def decode(word):
    return ((word >> 28) & 0xF, (word >> 24) & 0xF, (word >> 16) & 0xFF,
            word & 0xFFFF)


def is_legal(op, a, p):
    """False => hardware treats the word as HALT and sets the ILLEGAL flag."""
    if op not in OP_NAMES:
        return False
    if OP_NAMES[op] == "REG" and a not in REG_NAMES:
        return False
    return True


def disasm(word, addr=None):
    op, a, p, d = decode(word)
    if not is_legal(op, a, p):
        return "ILLEGAL 0x%08X" % word
    n = OP_NAMES[op]

    def src(i):
        return ("uio%d" % i) if i < 8 else ("ui%d" % (i - 8))

    def snk(i):
        return ("uio%d" % i) if i < 8 else ("uo%d" % (i - 8))

    def mask_pins(bank):
        pre = "uo" if bank else "uio"
        return " ".join("%s%d" % (pre, i) for i in range(8) if (p >> i) & 1)

    page = ((addr >> 8) & 1) << 8 if addr is not None else 0
    if n in ("NOP", "HPUSH", "HPULL", "HALT"):
        s = n
    elif n == "PINS":
        s = "PINS %s %s" % ({v: k for k, v in PINS_MODES.items()}[a & 3],
                            mask_pins((a >> 3) & 1))
    elif n == "WAIT":
        s = "WAIT %s %s" % ({v: k for k, v in WAIT_TYPES.items()}[p & 3], src(a))
    elif n == "JMP":
        s = "JMP 0x%03X" % (((a & 1) << 8) | p)
    elif n in ("JHI", "JLO"):
        s = "%s %s, 0x%03X" % (n, src(a), page | p)
    elif n == "SHOUT":
        f = ["msb" if p & SHOUT_MSB else "lsb"]
        if p & SHOUT_SHIFT:
            f.append("shift")
        if p & SHOUT_OD:
            f.append("od")
        s = "SHOUT %s %s" % (snk(a), " ".join(f))
    elif n == "SAMPLE":
        s = "SAMPLE %s %s" % (src(a), "msb" if p & SAMPLE_MSB else "lsb")
    elif n == "REG":
        sub = REG_NAMES[a]
        s = {"LDSR": "LDI sr, 0x%02X" % p, "LDC0": "LDI c0, 0x%02X" % p,
             "LDC1": "LDI c1, 0x%02X" % p, "MVC0": "MOV c0, sr",
             "MVC1": "MOV c1, sr", "MVSR0": "MOV sr, c0", "MVSR1": "MOV sr, c1",
             "XORI": "XORI sr, 0x%02X" % p, "ANDI": "ANDI sr, 0x%02X" % p,
             "ORI": "ORI sr, 0x%02X" % p, "ADDI": "ADDI sr, 0x%02X" % p}[sub]
    elif n == "DJNZ":
        s = "DJNZ c%d, 0x%03X" % (a & 1, page | p)
    elif n == "JSB":
        s = "JSB sr[%d]==%d, 0x%03X" % (a & 7, (a >> 3) & 1, page | p)
    else:  # pragma: no cover
        s = n
    return s + (" @%d" % d if d else "")


def gen_sv():
    """Emit the SystemVerilog package. Do not edit the output by hand."""
    L = []
    w = L.append
    w("// GENERATED by sw/ttpe_isa.py --gen-sv  -- DO NOT EDIT BY HAND")
    w("// TTPE ISA v0.2 (draft)")
    w("package ttpe_isa_pkg;")
    w("")
    w("  localparam int PC_W        = %d;" % PC_BITS)
    w("  localparam int FETCH_LAT   = %d;  // accept/pc_load -> valid_instruction" % FETCH_LATENCY)
    w("")
    for name, v in OPCODES.items():
        w("  localparam logic [3:0] OP_%-6s = 4'h%X;" % (name, v))
    w("")
    for name, v in PINS_MODES.items():
        w("  localparam logic [1:0] PINS_%-6s = 2'd%d;" % (name.upper(), v))
    for name, v in WAIT_TYPES.items():
        w("  localparam logic [1:0] WAIT_%-4s = 2'd%d;" % (name.upper(), v))
    w("")
    w("  localparam int SHOUT_MSB_BIT   = 0;")
    w("  localparam int SHOUT_SHIFT_BIT = 1;")
    w("  localparam int SHOUT_OD_BIT    = 2;")
    w("  localparam int SAMPLE_MSB_BIT  = 0;")
    w("")
    for name, v in REG_SUBOPS.items():
        w("  localparam logic [3:0] REG_%-5s = 4'd%d;" % (name, v))
    w("")
    w("  // Instruction field slices")
    w("  function automatic logic [3:0]  f_op (input logic [31:0] i); return i[31:28]; endfunction")
    w("  function automatic logic [3:0]  f_a  (input logic [31:0] i); return i[27:24]; endfunction")
    w("  function automatic logic [7:0]  f_p  (input logic [31:0] i); return i[23:16]; endfunction")
    w("  function automatic logic [15:0] f_d  (input logic [31:0] i); return i[15:0];  endfunction")
    w("")
    w("endpackage")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    if "--gen-sv" in sys.argv:
        sys.stdout.write(gen_sv())
    else:
        print(__doc__)
