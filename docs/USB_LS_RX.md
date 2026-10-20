# Low-speed USB receive on the TTPE: cycle budget and `ttpe_usb_rx` primitive (Phase D, proposal)

Status: **RTL written (`src/ttpe_usb_rx.sv`) and integrated, but not yet simulated.** The authoring session had no
HDL simulator, so the RTL and its cocotb tests (`test/test_ttpe_usb.py`, the `usb_*` scenarios in
`test/ttpe_scenarios.py`, the `t_usb_*` chip-level tests) have never been run against a real simulator. Everything
called "measured" below was produced by the Python reference model (`sw/usb_ls.py`) running against the
cycle-accurate ISS (`sw/ttpe_iss.py`): `python3 -m unittest discover -s sw/tests`. Nothing was synthesised, placed
or measured on silicon. ISA v0.2 is unchanged; with MODE.usb_en = 0 (reset value) the design behaves as before.

## 1. The budget

50 MHz clock, 1.5 Mbit/s: one bit cell is 100/3 = **33.33 cycles**, realised as 33/33/34. The engine commits at
best one instruction every 3 cycles (D = 2), so a cell holds at most **11 instruction slots**, and a taken
branch costs 3 extra cycles (`commit(target) = commit(branch) + D(target) + 4`, ISA.md section 3).

Work that has to happen per bit, and who can do it:

| Per-bit work | Software on today's engine | Why |
|---|---|---|
| Wait for SYNC start (D+ rise) | **yes** (`WAIT rise uio6`) | one instruction, blocks until the edge |
| Sample at a fixed offset after an edge | **yes, exact** | `SAMPLE @D` is cycle-exact; cell gaps 33/34/33 are just different `D` values |
| Push raw samples to the host | **yes** | `HPUSH` every 8th sample, 3 slots, never blocks |
| NRZI decode, bit unstuffing, SYNC check, CRC | **host** (after capture) | done in Python in `decode_raw_capture`; engine does not need to |
| Per-edge phase resync | **no** | `WAIT` has no timeout, so a run of 1s (no edges) would hang it; polling with `JHI/JLO` loops is 6 cycles per poll (taken branch, ISA.md section 3) = 18 % of a cell |
| EOP (SE0) detection | possible but costs a second `SAMPLE` per cell on `uio7` | not exercised in this phase |
| Data-dependent decision tree per bit (stuff counter, NRZI state) | **not demonstrated** | I did not build a software-only decoder; no claim either way beyond the blockers above |

### Measured: software-only fixed-phase capture (`gen_raw_capture_asm`, 146 words, ISS)

The engine samples D+ once per cell from the SYNC edge on, with no feedback. Host does NRZI/unstuff.

| Packet | Exact decode at 0 ppm and +-15 000 ppm (+-1.5 %) | First failing rate error (500 ppm steps, 1 seed, no jitter) |
|---|---|---|
| handshake, 1 byte | yes | 32 000 ppm |
| token, 3 bytes | yes (only 1 000 ppm of margin) | 16 000 ppm |
| data, 11 bytes (about 100 cells) | **no** | 5 500 ppm |

The phase drifts by cells x rate error; the sample point leaves the +-16 cycle window after about 100 cells at
0.5 %. So software-only is enough for tokens and handshakes, **not** for data packets at the +-1.5 % that
low-speed USB requires (tolerance figure from memory, not checked against the spec).

## 2. Partition with the primitive

Per-edge resync is the part software cannot do, and it is cheap in hardware, so the primitive takes **everything
that needs a fixed-latency decision per cell** and presents the engine with four event/level pins.
Software keeps framing, byte assembly, FIFO traffic and protocol responses.

| Engine cycles, per data bit | Instruction (examples/usb_rx.asm) | Commit cycle (t = TICK cycle) |
|---|---|---|
| wait for the bit | `WAIT rise ui4 @2` | t |
| end-of-packet check | `JHI ui6, pkt_end @2` | t+3 |
| shift the bit in | `SAMPLE ui5 lsb @2` | t+6 |
| count | `DJNZ c1, bit_loop @2` | t+9 |
| next `WAIT` armed | taken-branch penalty | t+15 |

Next TICK arrives t+33/34 at nominal clock: **slack 18**. Per byte the fall-through is
`HPUSH` t+12, `LDI c1,8` t+15, then the first bit of the next byte is an unrolled copy of the loop head
(no taken `JMP`): `WAIT` armed t+18, **slack 15**. Engine utilisation is 12 of 33 cycles on bit cells.

Measured with the real ISS (min over 4 seeds per rate; `slack = TICK cycle - first cycle the WAIT is armed`).
Each row gives the range over the three rate errors -1.5 %, 0, +1.5 %. The tests assert
`slack >= min_tick_gap - 18` and `slack >= 0` in every configuration they run, and `slack >= 14` on an ideal clock;
the table itself came from a scratch script that is not committed.

| jitter (+-cycles) | D-/D+ skew (cycles) | min TICK gap | min WAIT slack | bytes exact |
|---|---|---|---|---|
| 0 | 0 | 30..33 | 13..15 | yes |
| 1 | 2 | 29..31 | 11..13 | yes |
| 2 | 3 | 27..29 | 9..11 | yes |
| 4 | 3 | 24..26 | 6..9 | yes |

Jitter of 4 cycles is 80 ns peak, 12 % of a cell; what real hosts and cables give is **unverified**. The TICK gap
drops below 33 even without jitter at -1.5 %, because the free-running 33/33/34 pattern is re-anchored by each
edge (up to 7 cells without an edge, so up to 3 cycles of drift).

Negative control (`test_negative_control_slow_loop_loses_data`): delaying the loop by 28 cycles makes the pushed
bytes wrong, so the test does detect a missed TICK.

## 3. The primitive: `ttpe_usb_rx`

### 3.1 Interface (synchronous, one clock, no new clock domain)

| Port | Dir | Width | Meaning |
|---|---|---|---|
| `clk`, `rst_n` | in | 1 | existing clock; `rst_n` is the synchronised reset (async assert as everywhere else) |
| `en` | in | 1 | host-controlled enable (3.3). `en = 0`: state held in IDLE, engine pins pass through untouched |
| `dp_q` | in | 1 | `s2[6]` = uio6 after the 2-flop synchroniser (D+) |
| `dp_prev_q` | in | 1 | `s3[6]` = D+ one cycle earlier (`sync_prev_q[6]` of `ttpe_input_sync`; no new flop) |
| `dm_q` | in | 1 | `s2[7]` = uio7 synchronised (D-); used only for SE0 |
| `usb_d` | out | 4 | next value for `s2[15:12]` = `{err, stop, data, tick}` (combinational from the inputs and internal state; the primitive keeps its own data/stop/err hold flops so a stale pad value can never leak in when `en` rises) |

Integration (the only change to existing RTL besides one host register): in `ttpe_input_sync`, the D input of
the four `s2` flops for bits 15:12 becomes `usb_en ? usb_d : s1_q[15:12]` (ports `usb_en`, `usb_d` of
`ttpe_input_sync`; `ttpe_exec` instantiates `ttpe_usb_rx` and `ttpe_core`/`tt_um_ttpe` route `usb_en`). Then:

* the primitive output is, by construction, **in the same domain as a synchronised pad**: a value computed
  during cycle k is seen by instructions committing in cycle k+1, and `WAIT rise ui4` compares it with the value
  of cycle k (flop `s3`), exactly like any other pin;
* no mux is added on the path `s2 -> 16:1 source mux -> WAIT/JHI/JLO -> commit` (the mux sits in front of the
  flops), so the documented critical path is not lengthened (**unverified by STA**);
* with `en = 0` the circuit behaves bit-for-bit as before.

### 3.2 Virtual pins (valid only while `en = 1`; they replace the physical inputs ui_in[7:4], source pins 12..15)

| Source pin | Name | Meaning |
|---|---|---|
| ui4 | TICK | 1-cycle pulse: one per decoded data bit (NRZI decoded, stuffed bits removed) and one at the packet end |
| ui5 | DATA | last decoded bit, held until the next TICK |
| ui6 | STOP | set at the packet-end TICK (EOP or stuff error); stays set until the next SYNC is recognised |
| ui7 | ERR | set together with STOP after a bit-stuff violation; cleared with STOP |

A stuffed bit produces no TICK. Pin plan (`HOST_SPI.md`): ui_in[2:0] stay SPI; ui_in[7:4] are free program
inputs today, so enabling the primitive costs those four physical inputs.

### 3.3 Enable

Host register **0x0B `MODE`** (implemented in `ttpe_host_spi.sv`, listed in HOST_SPI.md): bit 0 `usb_en`, R/W,
bits 7:1 read 0 and ignore writes, reset value 0, cleared by the chip reset only (not by `core_reset`, so the host can
configure it before starting a program). No ISA change. Alternative if programs must switch it themselves:
`REG` sub-op 12, which is illegal today; that would be an ISA v0.3 change (assembler, ISS, `ttpe_exec.sv`, ISA.md,
regression). Not done.

**Change `usb_en` only while the bus is idle.** Disabling or resetting the primitive in the middle of a packet emits
no STOP: the bits already delivered stay in front of the next packet and the program's bit counter is misaligned
(tested in `test_enable_toggle` / `test_async_reset_mid_packet`). In the lockstep tests the ISS only advances while
the core runs while the RTL primitive free-runs, so those tests also change `usb_en` only while running.

### 3.4 Behaviour (the reference model `usb_rx_model` is the specification)

| Item | Definition |
|---|---|
| Pins | D+ = uio6, D- = uio7, idle J = (D+ 0, D- 1), K = (D+ 1, D- 0), SE0 = both 0 |
| SYNC start | IDLE and a rising edge of `dp_q` (J to K) |
| Cell clock | counter of the cell, restarted by every `dp_q` edge; free-running rollover 33, 33, 34 (sums to 100 cycles per 3 cells) |
| Sample | when the cell counter reaches **16**, once per cell |
| Resync rule | an edge after the cell was sampled starts the next cell (advance the 33/33/34 index); an edge before the sample belongs to the current cell (no advance) |
| SYNC check | the first 8 sampled levels must be K J K J K J K K (D+ = 1 0 1 0 1 0 1 1), none may be SE0; otherwise go to LOCKOUT silently (no event) |
| NRZI | bit = 1 if the sampled D+ equals the previous sampled D+, else 0; previous level starts as the last SYNC level |
| Unstuff | after six consecutive 1s the next cell is the stuffed 0: no TICK; if it decodes as 1: stuff violation |
| EOP | SE0 (D+ = 0 and D- = 0) at a sample point after SYNC: TICK with STOP, back to IDLE. In the model sweeps SE0 of 20 cycles (400 ns) or more was decoded in every case; 10 cycles never; 11..19 depends on the phase of the last cell (swept 10..100) |
| Stuff violation | TICK with STOP and ERR, then LOCKOUT |
| Missing EOP | line idle J decodes as a run of 1s, so it ends as a stuff violation after 7 cells (tested) |
| LOCKOUT | ignores the bus until 264 cycles (8 cells) pass with no `dp_q` edge, then IDLE; this is how the receiver skips the rest of a damaged packet |
| Outputs | all four are registered; TICK is a single cycle; STOP/ERR are cleared when a new SYNC start is recognised |

State counted from the model: cell counter 6 b, index 2 b, FSM 2 b, SYNC cell counter 3 b, ones counter 3 b,
previous level 1 b, sampled flag 1 b, LOCKOUT counter 9 b + data/stop/err hold 3 b = **30 flops**.
Gate count and area: **not synthesised, no estimate worth quoting.**

Known limitations of this definition: data comes from D+ only (D- is used just for SE0), so there is no
differential noise immunity; CRC is not checked in hardware (host or program does it; the CRC5/CRC16 generators in
`usb_ls.py` match the well-known SETUP vectors `2D 00 10` and `2D 01 E8` and give the constant CRC16 residual
`0xB001`); a bad SYNC is silent; the RX FIFO is 8 deep and `HPUSH` never blocks, so an 11-byte data packet
(one byte per 267 cycles) needs the host to drain it while the packet is arriving.

## 4. Software loop

`sw/examples/usb_rx.asm` (18 words). It pushes every byte to the RX FIFO, LSB first as received (PID first), and on
packet end toggles `uo6` (every packet) and additionally `uo5` when the packet was bad (ERR, or a last byte that
is not complete: `c1 != 8` tested with `MOV sr,c1` and `JSB sr[3]==1`). It does not run on the current RTL
(no primitive). In-band framing is not possible with byte FIFO entries, which is why the end flag is on a pin.

## 5. RTL change set (written, unsimulated)

| File | Change |
|---|---|
| `src/ttpe_usb_rx.sv` | new; transcription of `usb_ls.UsbRx` |
| `src/ttpe_input_sync.sv` | ports `usb_en`, `usb_d`; 4-bit mux in front of the `s2[15:12]` D inputs |
| `src/ttpe_exec.sv` | port `usb_en`; instantiates `ttpe_usb_rx` on `s2[6]`, `s3[6]`, `s2[7]` |
| `src/ttpe_core.sv`, `src/tt_um_ttpe.sv` | route `usb_en` |
| `src/ttpe_host_spi.sv` | register 0x0B `MODE` |
| `info.yaml`, `test/Makefile` | `ttpe_usb_rx.sv` in the source lists; `TARGET=usb` |
| `sw/ttpe_iss.py` | `Ttpe.usb_en` (+ `UsbRx`) in the synchroniser update; default off = unchanged |
| `test/` | `test_ttpe_usb.py`, `usb_*` scenarios, `usb_en` in the lockstep, `t_usb_*` chip-level tests |

Run (needs iverilog >= 11 and cocotb): `make -C test TARGET=usb`, `make -C test TARGET=core`,
`make -C test TARGET=top`, or everything with `make -C test regress`. A mismatch between RTL and model is reported with
the cycle number and both 4-bit values (`TARGET=usb`) or by the lockstep checker (`TARGET=core`).
The physical flow (LibreLane, precheck, GL test) has to be re-run: 30 flops and a small comparator tree should be
small next to the existing design, but that is a statement about size, not a measurement.

## 6. Not verified, do not rely on

* USB specification numbers: tolerance +-1.5 %, EOP width, jitter budget, turnaround timing, receiver
  thresholds. The figures used here are from memory.
* Electrical feasibility of USB low-speed levels on the Tiny Tapeout/IHP pads, and the external 1.5 kOhm pull-up
  and series resistors. No analysis was done.
* Any transmit path (NRZI encoder/stuffer for TX) and device-side response timing. This phase is receive only.
* The RTL itself: never compiled or simulated by the author (no HDL tools were available). Timing closure with the
  extra mux, area, and power: no synthesis was run.
* Behaviour with real hosts or captured traces; only generated waveforms (ppm, jitter, D-/D+ skew, lost EOP, glitches)
  were used. The generator and the model were written by the same author, so shared misreadings of the
  spec would not be caught; the independent checks are the CRC vectors and the packet decode through the engine.
