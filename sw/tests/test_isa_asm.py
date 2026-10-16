"""Assembler / ISA encoding tests.  Run:  python3 -m unittest discover -s sw/tests -v"""
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import ttpe_asm as asm      # noqa: E402
import ttpe_isa as isa      # noqa: E402

EX_DIR = os.path.join(os.path.dirname(__file__), "..", "examples")


def one(line, **kw):
    return asm.assemble(line, **kw).words[0]


class Encoding(unittest.TestCase):
    def test_encode_decode_roundtrip(self):
        rng = random.Random(1)
        for _ in range(2000):
            op, a, p, d = rng.randrange(16), rng.randrange(16), rng.randrange(256), rng.randrange(65536)
            self.assertEqual(isa.decode(isa.encode(op, a, p, d)), (op, a, p, d))

    def test_field_positions(self):
        self.assertEqual(isa.encode(0xA, 0xB, 0xCD, 0xEF12), 0xABCDEF12)

    def test_instruction_table(self):
        # (source, expected word)   delta 5 everywhere unless stated
        T = [
            ("NOP @5",                          0x00000005),
            ("PINS low uio3 @5",                0x10080005),
            ("PINS high uio0 uio7 @5",          0x11810005),
            ("PINS float uio2 uio3 @5",         0x120C0005),
            ("PINS toggle uio[4] @5",           0x13100005),
            ("PINS high uo7 @5",                0x19800005),
            ("WAIT low uio2 @5",                0x22000005),
            ("WAIT high ui1 @5",                0x29010005),
            ("WAIT fall uio0 @5",               0x20020005),
            ("WAIT rise ui7 @5",                0x2F030005),
            ("JMP 0x1A5 @5",                    0x31A50005),
            ("JMP 0x0A5 @5",                    0x30A50005),
            ("SHOUT uio3 @5",                   0x63000005),
            ("SHOUT uio3 msb shift od @5",      0x63070005),
            ("SHOUT uo2 lsb shift @5",          0x6A020005),
            ("SAMPLE uio2 lsb @5",              0x72000005),
            ("SAMPLE ui3 msb @5",               0x7B010005),
            ("LDI sr, 0x5A @5",                 0x805A0005),
            ("LDI c0, 7 @5",                    0x81070005),
            ("LDI c1, 255 @5",                  0x82FF0005),
            ("MOV c0, sr @5",                   0x83000005),
            ("MOV c1, sr @5",                   0x84000005),
            ("MOV sr, c0 @5",                   0x85000005),
            ("MOV sr, c1 @5",                   0x86000005),
            ("XORI sr, 0x0F @5",                0x880F0005),
            ("ANDI sr, 0x0F @5",                0x890F0005),
            ("ORI sr, 0x0F @5",                 0x8A0F0005),
            ("ADDI sr, -1 @5",                  0x8BFF0005),
            ("HPUSH @5",                        0xA0000005),
            ("HPULL @5",                        0xB0000005),
            ("HALT",                            0xC0000000),
        ]
        for src, want in T:
            with self.subTest(src):
                self.assertEqual(one(src), want, "%s -> %08X" % (src, one(src)))

    def test_branch_encodings(self):
        p = asm.assemble("""
            .org 0x010
        l:  JHI uio2, l @5
            JLO ui3, l @5
            DJNZ c1, l @5
            JSB sr[5]==1, l @5
            JSB sr[0]==0, l @5
        """)
        self.assertEqual(p.words[0x10], 0x42100005)
        self.assertEqual(p.words[0x11], 0x5B100005)
        self.assertEqual(p.words[0x12], 0x91100005)
        self.assertEqual(p.words[0x13], 0xDD100005)
        self.assertEqual(p.words[0x14], 0xD0100005)

    def test_unused_memory_is_halt(self):
        p = asm.assemble("NOP @5")
        self.assertTrue(all(w == 0xC0000000 for w in p.words[1:]))


class Errors(unittest.TestCase):
    def bad(self, src, frag, **kw):
        with self.assertRaises(asm.AsmError) as cm:
            asm.assemble(src, **kw)
        self.assertIn(frag, str(cm.exception))

    def test_short_delta_rejected(self):
        self.bad("PINS high uio0 @1", "TIMING")
        self.bad("PINS high uio0", "TIMING")          # delta 0
        self.assertEqual(one("PINS high uio0 @1", allow_short_delta=True), 0x11010001)

    def test_jmp_and_halt_exempt(self):
        asm.assemble("l: JMP l\nHALT")

    def test_branch_page(self):
        self.bad(".org 0\nJHI uio0, far @2\n.org 0x100\nfar: NOP @5", "different 256-word page")
        asm.assemble(".org 0\nJMP far @2\n.org 0x100\nfar: NOP @5")     # JMP can reach

    def test_pin_errors(self):
        self.bad("PINS float uo1 @5", "no output enable")
        self.bad("PINS high uio1 uo2 @5", "same bank")
        self.bad("WAIT high uo1 @5", "output-only")
        self.bad("SHOUT ui1 @5", "input-only")
        self.bad("SHOUT uo1 od @5", "open-drain")
        self.bad("PINS high uio9 @5", "out of range")

    def test_misc_errors(self):
        self.bad("FOO @5", "unknown mnemonic")
        self.bad("a: NOP @5\na: NOP @5", "duplicate symbol")
        self.bad("JMP nowhere @2", "undefined symbol")
        self.bad("LDI sr, 300 @5", "8 bits")
        self.bad("NOP @70000", "out of range")
        self.bad("JMP 0x200 @2", "out of range")
        self.bad(".if 1\nNOP @5", "missing .endif")
        self.bad("NOP @5 @6", "operand")

    def test_error_reports_line_number(self):
        with self.assertRaises(asm.AsmError) as cm:
            asm.assemble("NOP @5\nNOP @5\nBOGUS\n")
        self.assertEqual(cm.exception.lineno, 3)


class Directives(unittest.TestCase):
    def test_equ_default_and_define(self):
        src = ".default X, 7\nNOP @X"
        self.assertEqual(one(src), 7)
        self.assertEqual(one(src, defines={"X": 9}), 9)

    def test_if_else_nesting(self):
        src = """
        .equ A, 1
        .equ B, 0
        .if A
          .if B
            NOP @10
          .else
            NOP @11
          .endif
        .else
          NOP @12
        .endif
        .if B
          NOP @13
        .else
          NOP @14
        .endif
        """
        p = asm.assemble(src)
        self.assertEqual([w & 0xFFFF for w in p.words[:2]], [11, 14])
        self.assertTrue(p.words[2] == 0xC0000000)

    def test_expressions(self):
        self.assertEqual(one("NOP @(3*434)/2-4"), 3 * 434 // 2 - 4)
        self.assertEqual(one("NOP @0x10 | 3"), 0x13)
        self.assertEqual(one("NOP @(1<<8) + 2"), 258)

    def test_org_and_word(self):
        p = asm.assemble(".org 5\n.word 0xDEADBEEF\n.org 2\nNOP @5")
        self.assertEqual(p.words[5], 0xDEADBEEF)
        self.assertEqual(p.words[2], 5)

    def test_overlap_detected(self):
        with self.assertRaises(asm.AsmError):
            asm.assemble("NOP @5\n.org 0\nNOP @5")

    def test_macro_params_and_unique_labels(self):
        src = r"""
        .macro LOOP2 pin
        l\@: PINS toggle \pin @5
             DJNZ c0, l\@ @2
        .endm
            LOOP2 uio1
            LOOP2 uio2
        """
        p = asm.assemble(src)
        self.assertEqual(p.words[0], one("PINS toggle uio1 @5"))
        self.assertEqual(p.words[1], one(".org 0\nDJNZ c0, 0 @2"))
        self.assertEqual(p.words[3] & 0xFF0000, 0x00000000 | (2 << 16))   # second loop targets word 2

    def test_macro_errors(self):
        with self.assertRaises(asm.AsmError):
            asm.assemble(".macro M a\nNOP @5\n.endm\nM")
        with self.assertRaises(asm.AsmError):
            asm.assemble(".macro M\nM\n.endm\nM")           # recursion

    def test_delay_pseudo(self):
        self.assertEqual(one("DELAY 3"), 2)                  # spacing 3 == NOP @2
        self.assertEqual(one("DELAY 65536"), 65535)
        p = asm.assemble("DELAY 65539")
        spacing = sum((w & 0xFFFF) + 1 for w in p.words[:2])
        self.assertEqual(spacing, 65539)
        self.assertTrue(all((w & 0xFFFF) >= 2 for w in p.words[:2]))
        p = asm.assemble("DELAY 200000")
        n = sum(1 for u in p.used if u)
        self.assertEqual(n, 4)
        self.assertEqual(sum((w & 0xFFFF) + 1 for w in p.words[:4]), 200000)
        with self.assertRaises(asm.AsmError):
            asm.assemble("DELAY 2")


class Outputs(unittest.TestCase):
    def test_hex_bin_mpy(self):
        p = asm.assemble("NOP @5\nHALT")
        hexlines = p.to_hex().split()
        self.assertEqual(len(hexlines), 512)
        self.assertEqual(hexlines[0], "00000005")
        b = p.to_bin()
        self.assertEqual(len(b), 2048)
        self.assertEqual(b[:4], bytes([5, 0, 0, 0]))
        self.assertIn("0xC0000000", p.to_mpy())


class ExamplesAssemble(unittest.TestCase):
    def test_examples_assemble_and_disasm_roundtrip(self):
        for fn in sorted(os.listdir(EX_DIR)):
            if not fn.endswith(".asm"):
                continue
            with self.subTest(fn):
                with open(os.path.join(EX_DIR, fn)) as f:
                    p = asm.assemble(f.read())
                self.assertLess(sum(p.used), 512)
                lines = []
                for a, w in enumerate(p.words):
                    if p.used[a]:
                        lines.append(".org %d\n%s" % (a, isa.disasm(w, a)))
                # JMP/DJNZ targets are printed as numbers; delta 0/1 never occurs (lint)
                p2 = asm.assemble("\n".join(lines))
                self.assertEqual(p.words, p2.words)

    def test_generated_sv_package_is_current(self):
        pkg = os.path.join(os.path.dirname(__file__), "..", "..", "rtl", "ttpe_isa_pkg.sv")
        if os.path.exists(pkg):
            with open(pkg) as f:
                text = f.read()
            self.assertEqual(text, isa.gen_sv(),
                             "rtl/ttpe_isa_pkg.sv is stale: run  python3 sw/ttpe_isa.py --gen-sv > rtl/ttpe_isa_pkg.sv")


if __name__ == "__main__":
    unittest.main()
