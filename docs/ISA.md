# TTPE Instruction Set and Timing Model, v0.2 (draft)

Status: draft for review. Not frozen until the Execution Unit RTL has been cross-checked against the
reference model (`sw/ttpe_iss.py`). Source of truth for all encodings: `sw/ttpe_isa.py`.
The SystemVerilog package `src/ttpe_isa_pkg.sv` is generated from it
(`python3 sw/ttpe_isa.py --gen-sv > src/ttpe_isa_pkg.sv`); `sw/tests/test_isa_asm.py` checks it.

Every timing statement below is checked cycle-exactly by `sw/tests` (186 tests, `python -I -m unittest discover -s sw/tests -f`). Cycle numbers are
integers; one cycle is 20 ns at 50 MHz.

## 1. Architectural state

| State | Width | Notes |
|---|---|---|
| PC | 9 | Word address (`0x000..0x1FF`) into the 512 x 32 SRAM. Sequential execution crosses page boundaries (`0x0FF -> 0x100`) freely and wraps `0x1FF -> 0x000`. Unused words should hold `HALT` (the assembler fills them). |
| SR | 8 | Shift/data register. Full-duplex shifting uses this single register. |
| C0, C1 | 8 + 8 | Loop counters. |
| uio_out, uio_oe | 8 + 8 | Output latch and output enable of the 8 bidirectional pins. |
| uo_out | 8 | The 8 dedicated outputs. |
| Input synchroniser | 2 x 16 | 2-flop synchroniser (`s1`, `s2`) plus 1-cycle history (`s3`) for `uio_in[7:0]` and `ui_in[7:0]`. Resets to 0 and free-runs every clock cycle even while `halted = 1` (section 4). |
| Host TX FIFO | 8 x 8 | Host to engine (depth 8, registered start-of-cycle `empty`/`full`). Read by `HPULL`. |
| Host RX FIFO | 8 x 8 | Engine to host (depth 8, registered start-of-cycle `empty`/`full`). Written by `HPUSH`. |
| Flags | | `halted` (resets to **1** in RTL), `illegal` (sticky; cleared on `host_resume` or `core_reset`), `rx_overrun` (8-bit saturating counter, clamped at `255`, cleared by `rx_fifo_flush`). |

On hardware reset (`rst_n`), all architectural state resets to `0` **except `halted = 1`** (the core stays halted until `host_resume` is pulsed while `halted && mem_free`). *Standalone-ISS modelling note:* `sw/ttpe_iss.py` intentionally models execution starting at `core_run = 1` (`halted = False`, cycle `0`) and freezes its internal synchroniser and USB bit slicer while `halted = True`; the RTL behaviour (`halted = 1` at reset, free-running synchroniser/USB slicer) is the specification truth (see section 4 for why halted-cycle synchroniser differences are unobservable to ISA v0.2 programs).

Pin numbering: **sources** 0-7 = `uio_in[0..7]`, 8-15 = `ui_in[0..7]`; **sinks** 0-7 = `uio[0..7]`, 8-15 = `uo_out[0..7]`.

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
| 1 | `PINS` | [3] bank (0 uio, 1 uo), [1:0] mode: 0 low, 1 high, 2 float, 3 toggle (`A[2]` ignored) | pin mask (bit i = pin i of the bank) | Changes all masked pins in **one** cycle. `low`/`high`/`toggle` also set `uio_oe` = 1 on `uio`. `float` clears `uio_oe` and keeps `uio_out` unchanged (open-drain idiom). `float` on `uo` is a no-op (assembler rejects it). |
| 2 | `WAIT` | source pin (0-15) | [1:0] 0 low, 1 high, 2 fall, 3 rise (`P[7:2]` ignored) | Blocks until the condition holds (section 4). |
| 3 | `JMP` | [0] = target bit 8 (`A[3:1]` ignored) | target[7:0] | Unconditional jump to `{A[0], P}` (any page). |
| 4 | `JHI` | source pin (0-15) | target[7:0] | Jump to `{branch_pc[8], P}` if synchronised pin is 1. Uses the **branch instruction's own** address bit 8 (even at `0x0FF` or `0x1FF`, where prefetch PC has already incremented). |
| 5 | `JLO` | source pin (0-15) | target[7:0] | Jump to `{branch_pc[8], P}` if synchronised pin is 0. Same branch-page rule. |
| 6 | `SHOUT` | sink pin (0-15) | [0] msb-first, [1] shift after, [2] open-drain (uio only); `P[7:3]` ignored | Extracts `bit = P[0] ? SR[7] : SR[0]`. On `uio[0..7]` (`A < 8`): push-pull (`P[2] = 0`) sets `uio_oe[A] <- 1` and `uio_out[A] <- bit`; open-drain (`P[2] = 1`) sets `uio_out[A] <- 0` and `uio_oe[A] <- ~bit`. On `uo_out[0..7]` (`A >= 8`): `P[2]` is ignored and `uo_out[A-8] <- bit` (assembler rejects `od` on `uo`). If `P[1] = 1`, shifts `SR` after reading `bit` (`{SR[6:0], 0}` if `P[0] = 1`, `{0, SR[7:1]}` if `P[0] = 0`). |
| 7 | `SAMPLE` | source pin (0-15) | [0] msb-first (`P[7:1]` ignored) | msb-first: `SR <- {SR[6:0], pin}`. lsb-first: `SR <- {pin, SR[7:1]}`. |
| 8 | `REG` | sub-op (below) | immediate | Register operations. |
| 9 | `DJNZ` | [0] counter (0=`C0`, 1=`C1`; `A[3:1]` ignored) | target[7:0] | `C <- (C - 1) & 0xFF`; jump to `{branch_pc[8], P}` if the new value is non-zero. Same branch-page rule. `LDI C,0` therefore loops 256 times. |
| A | `HPUSH` | - (ignored) | - (ignored) | Push `SR` to the 8-deep host RX FIFO. Never blocks. Evaluated against start-of-cycle `rx_full`: if `rx_full = 1`, the byte is dropped and `rx_overrun` increments (saturating at `255`) **even if the host asserts `rx_pop = 1` in the same cycle**. `rx_fifo_flush` overrides `HPUSH` and `rx_pop`, clearing the FIFO and `rx_overrun` at the cycle edge (RTL: `src/ttpe_fifo.sv:L61,L73-L79`, wired at `src/ttpe_core.sv:L191-L204`). |
| B | `HPULL` | - (ignored) | - (ignored) | Blocks until start-of-cycle `tx_empty = 0`, then `SR <- tx_rdata` and pops the TX FIFO. A host `tx_push` in cycle `X` is first visible to `HPULL` in cycle `X + 1`. If `tx_fifo_flush` occurs in the same cycle an `HPULL` commits, `HPULL` still reads the pre-flush head byte and the FIFO is empty next cycle. |
| C | `HALT` | - (ignored) | - (ignored) | Stop the core (`halted <- 1`). Host can resume; unless overridden by `host_pc_load`, execution continues at `(halt_pc + 1) & 0x1FF`. |
| D | `JSB` | [3] polarity `v`, [2:0] bit `n` | target[7:0] | Jump to `{branch_pc[8], P}` if `SR[n] == v`. Same branch-page rule. |
| E, F | reserved | | | Treated as illegal. |

`REG` sub-ops (A): 0 `LDI sr`, 1 `LDI c0`, 2 `LDI c1`, 3 `MOV c0,sr`, 4 `MOV c1,sr`, 5 `MOV sr,c0`,
6 `MOV sr,c1`, 8 `XORI sr`, 9 `ANDI sr`, 10 `ORI sr`, 11 `ADDI sr` (mod 256). Sub-ops `7` and `12..15` are illegal.

**Illegal instruction** (reserved opcode `0xE`/`0xF` or illegal `REG` sub-op): executes its `D` countdown like an unconditional instruction and commits at `A + 1 + D`, halting the core (`halted <- 1`) and setting sticky `illegal <- 1`. A subsequent `host_resume` (while `halted && mem_free`) clears both `halted` and `illegal` and continues at `(illegal_pc + 1) & 0x1FF` unless `host_pc_load` loads a different address.
Reserved bits in `A`/`P` (as noted per opcode above) are ignored by hardware and the reference model and are always emitted as 0 by the assembler.

## 3. Execution timing (the contract for the Execution Unit RTL)

The fetch controller (`src/ttpe_sram_ctrl.sv`, verified in simulation) presents instructions with these facts:

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
| Sequential accept (general) | `accept(N+1) = max(accept(N) + 3, commit(N))`. |
| Sequential, `D(N) >= 2` | `commit(N+1) = commit(N) + D(N+1) + 1` (for unconditional `N+1`). Fetch latency is fully hidden. |
| Sequential, `D(N) < 2` | For an **unconditional** predecessor `N`, `accept(N+1)` is delayed by `2 - D(N)` cycles (`commit(N+1) = commit(N) + (2 - D(N)) + D(N+1) + 1`). For a **blocking** predecessor `N` (`WAIT`/`HPULL`), the delay is `max(0, accept(N) + 3 - commit(N))` — zero if `N` waited until `commit(N) >= accept(N) + 3`. The assembler rejects `D < 2` unless `--allow-short-delta`. |
| Minimum spacing | 3 cycles (60 ns). Use a single `PINS` with a mask to change several pins in one cycle. |
| Taken branch (`JMP`, `JHI`/`JLO`, `DJNZ`, `JSB`) | `accept(target) = commit(branch) + 3`; `commit(target) = commit(branch) + D(target) + 4` (for unconditional `target`), i.e. a **3-cycle penalty**. `JMP` and `HALT` are exempt from the `D >= 2` rule because their successor is flushed anyway. |
| Not-taken branch | No penalty (follows sequential accept rule). |
| Maximum spacing | `D = 65535` -> 65536 cycles = 1.31072 ms. The `DELAY n` pseudo-instruction chains `NOP`s for longer delays. |
| Host `resume` / `pc_load` | `host_resume` and `host_pc_load` are honoured only while `halted && mem_free` (RTL: `src/ttpe_core.sv:L134-L141,L159,L231`). Pulsing `host_resume` in cycle `k_r` clears `halted` and `illegal` on cycle `k_r + 1` (`core_run` rises) and the first instruction is accepted in cycle `k_r + 4` (3 cycles after `core_run` rises) at `host_pc_val` if loaded, else `(last_halt_or_illegal_pc + 1) & 0x1FF` (or `0` after reset). |

Worked example: `PINS high uio0 @5` as the first instruction: accepted in cycle 3, commit 9, pad changes in cycle 10.

**Loop idiom.** A taken `DJNZ @2` costs 3 (its own spacing) + 3 (penalty) = 6 cycles. The first pass enters
the loop by fall-through, so put 6 cycles of other instructions (e.g. `NOP @5`) before the loop head to make
pass 1 identical to later passes. All four examples use this.

## 4. Input and output timing

* Input synchroniser: a pad change in cycle `X` is first seen by an instruction committing in cycle **`X + 2`** (`sync2(c) = pad(c - 2)`; an instruction committing in `X + 1` still sees the old value).
* `WAIT low/high` is level-sensitive on `sync2(c)`. For a `WAIT` accepted in cycle `A` with delta `D`, condition checks start on cycle `E = A + 1 + D` (inclusive) and repeat on every cycle `c >= E` until true. `WAIT fall/rise` compares `sync2(c) = pad(c - 2)` against `sync2(c - 1) = pad(c - 3)`: a pad transition at cycle `E - 2` (visible on `sync2` at cycle `E`, including `c = A + 1` when `D = 0`) **is caught** on cycle `E`; any pad transition at cycle `<= E - 3` (already visible on `sync2` at `<= E - 1`) is **missed**.
* The synchroniser resets to `0` on `rst_n` and **free-runs every clock cycle even while `halted = 1`**. Because the earliest instruction commit after `host_resume` (cycle `k_r`) is cycle `k_r + 5` (`accept` at `k_r + 4`, `D = 0`), three active cycles (`k_r + 1..k_r + 3`) always flush `s1`/`s2`/`s3` before any program reads them; no post-reset cycle-2 rising edge is ever observable after resume, and standalone ISS (`halted = False` at cycle `0`) produces identical instruction-visible values.
* Virtual USB source pins: when host register `MODE[0] = 1` (`usb_en = 1`, `docs/USB_LS_RX.md`), source pins `12..15` (`ui_in[7:4]`) load the synchronous `ttpe_usb_rx` outputs `{ERR, STOP, DATA, TICK}` into `s2[15:12]` instead of the physical pads, bypassing the physical pad-to-`s2` path on those four sources (sources `0..11` are unaffected). Note that in RTL `ttpe_usb_rx` free-runs while halted, whereas standalone ISS freezes it while halted; change `MODE[0]` only while the USB bus is idle (`J`).
* `SAMPLE` committing in cycle `e` reads `sync2(e)` (the physical pad as it was in cycle `e - 2`, or the virtual USB pin on sources `12..15` when `MODE[0] = 1`).
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

All parameters are assembler `-D` values; every claim below is asserted in `sw/tests/test_examples.py`, `sw/tests/test_i2c_target.py`, and `sw/tests/test_spi_slave.py`.

| Program | Parameters | Verified behaviour |
|---|---|---|
| `uart_tx.asm` | `BIT` cycles per bit (default 434 = 115200 baud) | Every bit boundary exactly `BIT` cycles apart; back-to-back frames exactly `10*BIT` apart (1 stop bit); start bit on the pad exactly 7 cycles after the host byte arrives; `BIT` 16 to 434 tested, `BIT` 8 rejected. |
| `uart_rx.asm` | `BIT` (even) | Samples exactly at bit centres; 24-byte streams decoded at nominal rate and with about 2% baud mismatch; overrun counted. No framing, parity or stop-bit check. |
| `spi_master.asm` | `CPOL`, `CPHA`, `H` half-period | Modes 0-3 data-exact; SCK half periods exactly `H` inside a byte; `H >= 12` (CPHA 0) or `>= 9` (CPHA 1), which is up to 2.08 / 2.78 MHz at 50 MHz. |
| `spi_slave.asm` | `CPOL`, `CPHA` | Full-duplex SPI slave (38 words, zero RTL changes), Modes 0-3 data-exact in Python (`1.0 MHz` default `H=25`, zero-pad-delay ceilings `2.083 MHz` CPHA=0 / `1.250 MHz` CPHA=1); Mode 0 and Mode 3 verified in RTL lockstep (`b7143b9`). CPHA=0 uses length-prefixed TX FIFO `[n, d0..d(n-1)]`; CPHA=1 is variable-length framed by `CS_N`. |
| `i2c_master.asm` | `TL`, `TH` (SCL low/high cycles), `DSH` | Write, read (last byte NACKed), address NACK -> error flag `uo7` + STOP, clock stretching. **Repeated START:** count byte `n[7] = 1` ends the transaction without STOP (bus held: SCL low, SDA released) and the next command byte starts with a repeated START (SCL low >= `TL`, tSU;STA = tHD;STA = exactly `TH`); `n[6:0]` is the byte count (0 = 256), so counts 128..255 are no longer expressible. Register read = host bytes `A0 81 REG A1 n`. SCL high exactly `TH`, low exactly `TL` inside a byte (longer between bytes). 100 kHz (`TL=TH=250`) and 400 kHz (`TL=75, TH=50`) meet the I2C minimum low/high times. |
| `i2c_target.asm` | `ADDR` (7-bit, default `0x50`), `THD_DAT` (default `15`), `TSU_DAT` (default `65` for 100 kHz, `22` for 400 kHz) | I2C target (76 words, zero RTL changes): write, read with clock stretching, address mismatch NACK, repeated START (`Sr`), open-drain only; verified in two-ISS cross-tests with `i2c_master.asm` and 100 kHz / 400 kHz RTL lockstep. |
| `usb_rx.asm` | none (needs MODE[0] = 1: `ttpe_usb_rx` virtual pins ui4..ui7) | Hybrid low-speed USB receive, see `USB_LS_RX.md`. |
| `usb_tx.asm` | `EXT2`, `EXT5` (extra cycle after bit 2 / 5, default 1) | Low-speed USB transmit on D+ = `uio6`, D- = `uio7`, software only: host pushes `n, PID, payload/CRC...`; SYNC, NRZI (`PINS toggle` on 0 bits), bit stuffing (`C0` countdown), EOP (SE0 66 cycles, J 33, release). Cells 33/34 cycles (266 per byte), waveform checked cycle for cycle against `usb_ls.tx_expected`, loopback through `ttpe_usb_rx` model and `usb_rx.asm`. 72 words. CRC must be supplied by the host; FIFO must stay ahead of the wire (HPULL deadline = its commit cycle). |
| `usb_device_ack.asm` | `EXT2`, `EXT5`, `PAD_D0`/`PAD_D1`/`PAD_IN` (turnaround padding per path, defaults 76/58/46) | Autonomous USB handshake (needs `MODE[0]` = 1): receive as `usb_rx.asm` (PID kept in `C0`), `WAIT high uio7` for the end of the host SE0, answer DATA0/DATA1 with ACK (0xD2) and IN with NAK (0x5A) using the `usb_tx.asm` byte loop; first K edge 140 cycles after the end of the host SE0 on every path. No CRC/address/toggle check; bad packets (stuff error, partial byte) get no response. 123 words. |

Known limitations: I2C has no multi-master arbitration, no 10-bit addressing, and no 50 ns input spike filter; `i2c_master.asm` `n` values 128..255 mean "count `n & 0x7F` with repeated START" (the count byte is no longer a plain 0..255 count); an aborted I2C write leaves its remaining bytes in the host TX FIFO until the host pulses `CTRL.tx_flush` (register `0x02` bit 2); `spi_slave.asm` requires preloaded TX FIFO before `CS_N` falls and has up to 17-cycle `CS_N`->Hi-Z release in CPHA=1.

## 7. Verification status

* 186 unit, protocol, and ISA-contract-monitor tests (including USB receive/transmit, exact slicer timing, autonomous ACK, I2C master/target repeated START, SPI master/slave Modes 0-3, `IsaContractMonitor` rules + negative controls N1-N10, and WP2(b) post-reset X/Z strictness), standard library only: `python3 -I -m unittest discover -s sw/tests -f`.
* Fault injection: firmware/model mutation harness `sw/mutants_fw.py` (`126` killed + `1` argued equivalent [`I14`] of `127`, last verified at `824c1be` before `test_isa_contract_mon.py` was added).
* The Execution Unit RTL is compared cycle by cycle with this model in cocotb lockstep with a live `IsaContractMonitor` (`make -C test TARGET=core`:
  70/70 tests = 66 scenarios + 4 harness tests, including `test_isa_monitor_negative_control`, 5 `usb_dev_*` scenarios, I2C target 100k/400k, and SPI slave Mode 0/3), plus memory (`TARGET=mem`, 18), USB slicer (`TARGET=usb`, 8), and chip-pin (`TARGET=top`, 16 with WP2(b) strict post-reset X/Z checking on all 24 output pins and explicit SRAM test initialization, no masking) — 112 PDK-SRAM cocotb tests total (298 primary tests with Python, plus 104 stub-SRAM cocotb tests), CI run on `b7143b9` (not re-run locally), and functional gate-level simulation (`gl_test`, `-DGL_TEST -DFUNCTIONAL`, zero-delay / not SDF-timed, 16/16) plus 8x4 GDS (`ERROR_ON_ILLEGAL_OVERLAPS=1`, `0` illegal overlaps, `0` KLayout DRC, LVS clear) and precheck in CI on `b7143b9` (not re-run locally).
  Nothing has run on silicon.

## 8. Open items

1. Replay a real captured low-speed USB trace (all USB signalling conventions are from the author's memory).
2. Compare the physical-flow numbers with the baseline run (Max Slew/Max Cap warnings, hold buffers; see `PHASE_E_REPORT.md` 1.7).
3. Board-level bring-up: pull-ups, series resistors, 3.3 V levels (see `docs/info.md`); USB thresholds are unverified.
4. USB: CRC generation, turnaround timing and automatic responses remain host/firmware work.
5. 10BASE-T Ethernet and USB full-speed are out of reach at 50 MHz without dedicated hardware.
