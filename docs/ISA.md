# TTPE Instruction Set and Timing Model, v0.2 (draft)

Status: draft for review. Not frozen until the Execution Unit RTL has been cross-checked against the
reference model (`sw/ttpe_iss.py`). Source of truth for all encodings: `sw/ttpe_isa.py`.
The SystemVerilog package `rtl/ttpe_isa_pkg.sv` is generated from it
(`python3 sw/ttpe_isa.py --gen-sv > rtl/ttpe_isa_pkg.sv`); a test fails if it goes stale.

Every timing statement below is checked cycle-exactly by `sw/tests` (71 tests). Cycle numbers are
integers; one cycle is 20 ns at 50 MHz.

## 1. Architectural state

| State | Width | Notes |
|---|---|---|
| PC | 9 | Word address into the 512 x 32 SRAM. Wraps 511 -> 0. Unused words should hold `HALT` (the assembler fills them). |
| SR | 8 | Shift/data register. Full-duplex shifting uses this single register. |
| C0, C1 | 8 + 8 | Loop counters. |
| uio_out, uio_oe | 8 + 8 | Output latch and output enable of the 8 bidirectional pins. |
| uo_out | 8 | The 8 dedicated outputs. |
| Input synchroniser | 2 x 16 | 2-flop synchroniser for `uio_in[7:0]` and `ui_in[7:0]`. Resets to 0. |
| Host TX FIFO | bytes | Host to engine. Read by `HPULL`. |
| Host RX FIFO | bytes | Engine to host. Written by `HPUSH`. Reference model depth 8. |
| Flags | | `halted`, `illegal` (sticky), RX-overrun count. |

All state resets to 0. Pin numbering: **sources** 0-7 = `uio_in[0..7]`, 8-15 = `ui_in[0..7]`;
**sinks** 0-7 = `uio[0..7]`, 8-15 = `uo_out[0..7]`.

## 2. Instruction word

```
 31    28 27   24 23      16 15              0
+--------+-------+----------+-----------------+
|   OP   |   A   |    P     |   D (delta)     |
+--------+-------+----------+-----------------+
```

`D` = number of cycles to wait **before** the action. `A` and `P` meaning depends on `OP`.

| OP | Mnemonic | A | P | Action |
|---|---|---|---|---|
| 0 | `NOP` | - | - | Nothing. Consumes time. |
| 1 | `PINS` | [3] bank (0 uio, 1 uo), [1:0] mode: 0 low, 1 high, 2 float, 3 toggle | pin mask (bit i = pin i of the bank) | Changes all masked pins in **one** cycle. low/high/toggle also enable the output. `float` clears OE and keeps the latched value (open-drain idiom). `float` on `uo` is a no-op (assembler rejects it). |
| 2 | `WAIT` | source pin (0-15) | [1:0] 0 low, 1 high, 2 fall, 3 rise | Blocks until the condition holds (section 4). |
| 3 | `JMP` | [0] = target bit 8 | target[7:0] | Unconditional jump, any page. |
| 4 | `JHI` | source pin | target[7:0] | Jump if pin is 1. Target is in the **same 256-word page** as the branch. |
| 5 | `JLO` | source pin | target[7:0] | Jump if pin is 0. Same page rule. |
| 6 | `SHOUT` | sink pin (0-15) | [0] msb-first, [1] shift after, [2] open-drain (uio only) | Drives `SR[7]` (msb) or `SR[0]` (lsb) onto the pin. Open-drain: latch <- 0, OE <- not bit. Optional shift (fill 0). |
| 7 | `SAMPLE` | source pin | [0] msb-first | msb-first: `SR <- {SR[6:0], pin}`. lsb-first: `SR <- {pin, SR[7:1]}`. |
| 8 | `REG` | sub-op (below) | immediate | Register operations. |
| 9 | `DJNZ` | [0] counter (C0/C1) | target[7:0] | `C <- C-1` (8-bit wrap); jump if the new value is non-zero. Same page rule. `LDI C,0` therefore loops 256 times. |
| A | `HPUSH` | - | - | Push SR to the host RX FIFO. If full: byte dropped, overrun count incremented. Never blocks. |
| B | `HPULL` | - | - | Blocks until the host TX FIFO has a byte, then `SR <- byte`. |
| C | `HALT` | - | - | Stop the core. Host can resume; execution continues after the `HALT`. |
| D | `JSB` | [3] polarity v, [2:0] bit n | target[7:0] | Jump if `SR[n] == v`. Same page rule. |
| E, F | reserved | | | Treated as illegal. |

`REG` sub-ops (A): 0 `LDI sr`, 1 `LDI c0`, 2 `LDI c1`, 3 `MOV c0,sr`, 4 `MOV c1,sr`, 5 `MOV sr,c0`,
6 `MOV sr,c1`, 8 `XORI sr`, 9 `ANDI sr`, 10 `ORI sr`, 11 `ADDI sr` (mod 256). Other values are illegal.

**Illegal instruction** (reserved opcode or sub-op): the core halts and sets the sticky `illegal` flag.
Reserved bits in `A`/`P` are ignored by the reference model and are always emitted as 0 by the assembler.

## 3. Execution timing (the contract for the Execution Unit RTL)

The fetch controller (`rtl/ttpe_sram_ctrl.sv`, verified in simulation) presents instructions with these facts:

* `core_run` asserted in cycle 0 -> first instruction valid in cycle 3.
* Instruction accepted in cycle X -> next sequential instruction valid in cycle X+3.
* Jump (`pc_load`) asserted in cycle X -> target valid in cycle X+3.

Execution Unit rules:

1. **Accept.** Instruction N is accepted in cycle A when the controller presents it **and** the unit is
   empty or its current instruction commits in this very cycle. Never in a cycle that redirects or halts.
2. **Commit.** `commit(N) = A + 1 + D` for unconditional instructions. `WAIT` and `HPULL` commit in the first
   cycle >= `A + 1 + D` in which their condition holds (edge or data available).
3. Registers update at the edge ending the commit cycle. A pin change is therefore on the pad in cycle **commit + 1**.
4. The next instruction is accepted in the commit cycle of the previous one.

Consequences (all tested):

| Situation | Rule |
|---|---|
| Sequential, `D >= 2` | `commit(N+1) = commit(N) + D(N+1) + 1`. Fetch latency is fully hidden. |
| Sequential, `D(N) < 2` | The successor is late: add `2 - D(N)` cycles. The assembler rejects `D < 2` unless `--allow-short-delta`. |
| Minimum spacing | 3 cycles (60 ns). Use a single `PINS` with a mask to change several pins in one cycle. |
| Taken branch (`JMP`, `JHI`/`JLO`, `DJNZ`, `JSB`) | `commit(target) = commit(branch) + D(target) + 4`, i.e. a **3-cycle penalty**. `JMP` and `HALT` are exempt from the `D >= 2` rule because their successor is flushed anyway. |
| Not-taken branch | No penalty. |
| Maximum spacing | `D = 65535` -> 65536 cycles = 1.31072 ms. The `DELAY n` pseudo-instruction chains `NOP`s for longer delays. |
| Resume after `HALT` | Pipeline restarts like `core_run` 0 -> 1: next instruction accepted 3 cycles after resume. |

Worked example: `PINS high uio0 @5` as the first instruction: accepted in cycle 3, commit 9, pad changes in cycle 10.

**Loop idiom.** A taken `DJNZ @2` costs 3 (its own spacing) + 3 (penalty) = 6 cycles. The first pass enters
the loop by fall-through, so put 6 cycles of other instructions (e.g. `NOP @5`) before the loop head to make
pass 1 identical to later passes. All four examples use this.

## 4. Input and output timing

* Input synchroniser: a pad change in cycle X is first seen by an instruction committing in cycle **X+2**
  (an instruction committing in X+1 still sees the old value).
* `WAIT low/high` is level-sensitive. `WAIT fall/rise` compares the synchronised value in the commit cycle with
  the previous cycle. **An edge that completes before the delta countdown ends is missed.**
* The synchroniser resets to 0, so a pad that is high at reset produces a rising edge at cycle 2. Do not `WAIT rise`
  in the first few cycles.
* `SAMPLE` committing in cycle e reads the pad as it was in cycle e-2.
* Pad and board delays are not included: the pad characterisation for this process is not yet available.

## 5. Assembler (`sw/ttpe_asm.py`, Python 3 standard library only)

```
python3 sw/ttpe_asm.py prog.asm -o prog.hex        # $readmemh image, 512 lines
python3 sw/ttpe_asm.py prog.asm --bin prog.bin     # little-endian 32-bit words
python3 sw/ttpe_asm.py prog.asm --mpy prog.py      # MicroPython list for the RP2350 host
python3 sw/ttpe_asm.py prog.asm --lst              # listing
options: -D NAME=VALUE   --allow-short-delta
```

Syntax: `label:`, `; comment`, `<mnemonic> operands @ delta`, `.equ`, `.default`, `.org`, `.word`,
`.if/.else/.endif`, `.macro/.endm` (`\p` parameter, `\@` unique id), `DELAY n`. Delta and immediates accept
integer expressions. Pins are `uio0..7`, `ui0..7`, `uo0..7` (source pins 12..15 = `ui4..7` can be virtual USB pins when host register MODE[0] is set, see USB_LS_RX.md; the ISA itself is unchanged). Conditional branches to another 256-word page
are an assembly error (use a local `JMP`).

## 6. Example programs (`sw/examples`)

All parameters are assembler `-D` values; every claim below is asserted in `sw/tests/test_examples.py`.

| Program | Parameters | Verified behaviour |
|---|---|---|
| `uart_tx.asm` | `BIT` cycles per bit (default 434 = 115200 baud) | Every bit boundary exactly `BIT` cycles apart; back-to-back frames exactly `10*BIT` apart (1 stop bit); start bit on the pad exactly 7 cycles after the host byte arrives; `BIT` 16 to 434 tested, `BIT` 8 rejected. |
| `uart_rx.asm` | `BIT` (even) | Samples exactly at bit centres; 24-byte streams decoded at nominal rate and with about 2% baud mismatch; overrun counted. No framing, parity or stop-bit check. |
| `spi_master.asm` | `CPOL`, `CPHA`, `H` half-period | Modes 0-3 data-exact; SCK half periods exactly `H` inside a byte; `H >= 12` (CPHA 0) or `>= 9` (CPHA 1), which is up to 2.08 / 2.78 MHz at 50 MHz. |
| `i2c_master.asm` | `TL`, `TH` (SCL low/high cycles), `DSH` | Write, read (last byte NACKed), address NACK -> error flag `uo7` + STOP, clock stretching. SCL high exactly `TH`, low exactly `TL` inside a byte (longer between bytes). 100 kHz (`TL=TH=250`) and 400 kHz (`TL=75, TH=50`) meet the I2C minimum low/high times. |

Known limitations: I2C has **no repeated START** (so the usual "write register pointer, then read" sequence is
not yet possible), no multi-master arbitration and no 10-bit addressing; an aborted I2C write leaves its remaining
bytes in the host TX FIFO until a flush control exists.

## 7. Verification status

* 71 unit and protocol tests, standard library only: `python3 -m unittest discover -s sw/tests -v`.
* Fault injection: shortening the modelled fetch latency, a one-cycle-late UART sample, an off-by-one SPI sample
  delta and a missing synchroniser stage were each caught by the tests.
* **Not yet done:** the reference model has not been compared against RTL, because the Execution Unit RTL does not
  exist yet. Until it does, this document and the model are a specification, not a verified implementation.

## 8. Open items

1. Host register map (run/halt/reset PC, FIFO status and flush, flags) and FIFO depths for the RTL.
2. Final pin plan: which pins the host SPI slave uses, and which of the 8 inputs / 8 outputs remain for programs.
3. Repeated START for I2C, and a decision on CRC / bit-stuffing / NRZI support for the stretch protocols
   (opcodes E-F are free; `JSB` allows software implementations at 3-cycle granularity).
4. Execution Unit RTL, then cross-check against the model cycle by cycle in cocotb.
5. Synthesis to get real area and timing numbers; none are claimed here.
