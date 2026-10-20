# Phase E: physical-design safety audit, risk register, protocol matrix (USB low-speed RX)

Author's note on evidence: "RTL sim" below means Icarus 12 + cocotb 2.1 results **reported by the user** (79 tests:
8 `TARGET=usb` + 55 `TARGET=core` + 16 `TARGET=top`; the author's independent count of the registered tests is also 79, so
no new test was silently skipped). The author never ran Verilog. Nothing in this file is a synthesis, STA, layout or
silicon result unless it says so.

## 0. Bottom line

* The USB block is **simulation-verified against its own specification, which the same author wrote from memory**.
  It has not touched synthesis, a real USB signal, or any independent USB implementation.
* Physical-design risk of the *block itself* looks small (about 30 flops, no macro interface, no floorplan change).
  That is an engineering estimate, not a measurement; the flow decides.
* The real silicon risks are **outside the RTL**: electrical interface, signalling conventions that no test could
  falsify, the 8-byte RX FIFO against a slow host, and the absence of any transmit path.
* Recommendation: run the physical flow now (cheap, objective), compare against the last good baseline, and in parallel
  replay one **real captured low-speed trace** through `TARGET=usb`. Do not call Phase D "fully verified" before that.

## 1. Physical safety audit

### 1.1 Area (6x4 tiles, `PL_TARGET_DENSITY_PCT` 60, `FP_SIZING` absolute)

| Item | Fact or estimate |
|---|---|
| New state | 30 flops, counted from `ttpe_usb_rx.sv` (6+2+2+1+3+3+1+9+3) plus 1 flop (`mode_usb_q`) in `ttpe_host_spi.sv` and 4 mux cells in front of `s2[15:12]` |
| Combinational logic | not measured. Order of a few hundred gates (6-bit incrementer and compares, 9-bit LOCKOUT counter, small FSM). **Estimate** |
| Relative size | the existing design has several hundred flops (synchroniser 48, FIFOs, host SPI, exec) plus the SRAM macro; +30 flops is plausibly in the 5 to 10 % range of flops and small next to the macro (obstruction rectangle in `config.json` is 416.6 x 192.5 um = 0.080 mm2). **Estimate from RTL counting, not synthesis** |
| Die / core | fixed by the tile count (`FP_SIZING: absolute`): adding cells cannot grow the die, it can only raise utilisation. If the placer cannot meet density there is no knob except removing logic |
| What would be a red flag | cell count or area up by more than about 15 % over baseline, or placement/global-route overflow that was not there before (author's threshold, not a TT rule) |

### 1.2 SRAM macro and PDN

* No new signal touches `u_core.u_mem.u_sram`; the macro location, the Metal2-4 obstruction rectangle, the 2 um halo,
  `FP_PDN_VOFFSET 16.84` and `src/pdn_cfg.tcl` are unchanged. Stripe/column alignment depends on die and macro geometry,
  not on cell count, so the earlier precheck fix should not regress. **Reasoned, not re-run.**
* New cells sit on the same standard-cell rails. Congestion near the macro edge is the only plausible interaction;
  `GRT_ALLOW_CONGESTION 1` masks, rather than removes, that risk. Check the global-route congestion report.

### 1.3 Timing at 50 MHz (`CLOCK_PERIOD 20`)

* New register-to-register paths: `s2[6]/s3[6]/s2[7]` -> edge detect -> cell counter increment and compare -> sample
  decision -> next-state / `usb_d` -> flops (including the `s2[15:12]` D mux). Roughly 20 gate levels by inspection;
  at 130 nm that is a few ns typical and maybe 2x in the slow corner, inside 20 ns but **not measured**. Look at WNS in
  all three corners the config lists (typ 1.20 V 25 C, fast 1.32 V, slow 1.08 V 125 C), setup and hold.
* The existing critical path (`s2` -> 16:1 source mux -> WAIT/JHI/JLO -> commit -> pc_load) now has extra fan-out on
  `s2[6]`, `s2[7]` and `s2[15:12]` consumers; that is a small load increase, not a new gate on the path.
* **Deviation from a stated rule:** `ttpe_input_sync.sv` documents that nothing may sit between `s1_q` and `s2_q`.
  For bits 15:12 there is now a 2:1 mux there. The mux delay (well under 1 ns) comes out of the roughly 19 ns of
  metastability resolution time, so MTBF is practically unchanged, but the rule is broken on four inputs and the placer
  must still keep `s1_q`/`s2_q` of those bits close. Accepted knowingly; flag it if a reviewer holds the rule strictly.
* `usb_en` is a same-clock flop output, no clock-domain crossing. Asynchronous reset flops follow the existing
  `rst_sync_n` scheme (recovery/removal checked by the same STA).

### 1.4 Synthesis and lint exposure (unknown until the flow runs)

* `always_comb` reads variables assigned earlier in the same block (`cnt_d`, `quiet_d`); all outputs have defaults, so no
  latch is expected. Yosys/Verilator may still warn on widths; `RUN_LINTER: 1` is on in the config.
* Gate-level `gl_test` runs the same cocotb tests on the netlist: this is the first real check that synthesis kept the
  behaviour. It includes `t_usb_rx_e2e`.
* `make -C test synth-check` (yosys parse/elaborate) exists in the Makefile but its own comment says it has never been
  run by the author; treat the first run as an experiment.

### 1.5 Pass criteria for the gate (proposal)

CI green on the branch: synthesis, placement, CTS, routing, KLayout DRC, LVS, precheck, `gl_test`, viewer. Then compare with
the last good baseline (commit `d8bffd4`): cell count and area, utilisation, setup and hold WNS/TNS per corner, route
congestion, DRC count. Send those numbers back; the claims in 1.1 to 1.3 stay "estimate" until then.

## 2. Risk register

Severity and likelihood are the author's judgement. "Not proven by the cocotb tests" is the point of the table.

| # | Risk | Why the tests do not cover it | Sev | How to close it |
|---|---|---|---|---|
| R1 | **Signalling conventions wrong.** J/K polarity for low speed, SYNC = K J K J K J K K, NRZI (0 = transition), stuff after six 1s, LSB-first, EOP = SE0 then J, idle J = D- high | Generator, model, RTL and tests all come from one author's memory of the spec. Only CRC5/CRC16 vectors are independently known. A wrong polarity passes all 79 tests and fails on first real packet | **High** | Replay a real captured low-speed trace (keyboard or mouse on a logic analyser) through `TARGET=usb`; compare against an independent decoder |
| R2 | **Analog / pad electricals.** Input thresholds of the TT/IHP pad versus USB levels (low speed needs roughly VIL 0.8 V, VIH 2.0 V at the receiver, 3.3 V signalling), pad rise-time handling, ESD, 5 V tolerance | Digital simulation drives ideal 0/1 | **High** | Pad datasheet check, bench measurement, series resistors and clamp on the board; do not connect VBUS-level signals |
| R3 | **Slow edges and chatter.** Low-speed edges are slow (tens to hundreds of ns). A non-Schmitt or noisy input can produce multiple transitions near the threshold, each one re-anchors the bit clock | Tests inject 1 to 3 cycle glitches and up to +-4 cycles of jitter, not realistic threshold chatter or crosstalk | Med | Logic-analyser capture on the real board; add chatter models if captures show it |
| R4 | **D+/D- pull-ups and roles.** A low-speed device needs a 1.5 kOhm pull-up on D- to 3.3 V; a host has 15 kOhm pull-downs. The chip has neither. A floating bus looks like noise or SE0 | Model assumes a clean J idle | **High** if used as a device, Med as a passive listener | External resistors; decide host/device/sniffer role first |
| R5 | **Clock frequency hard-wired.** Cell pattern 33/33/34 assumes exactly 50 MHz | Tests sweep +-1.5 % (model sweeps went to about +-6 %) around exactly 50 MHz | Med | Confirm the board clock; any other frequency needs different constants |
| R6 | **Rate drift beyond the tested range.** Host +-1.5 % (spec figure from memory) | Beyond about +-6 % the model fails; no test at larger errors | Low | Real clocks (crystal) are far inside; covered by R1 capture |
| R7 | **No transmit path.** No ACK/NAK/STALL, no data stage, no NRZI/stuffing encoder, no turnaround timing, no EOP generation | Nothing to test; RX only | **High** for any device role | Out of scope for Phase D; software TX is plausible at 3-cycle granularity but was never analysed or built |
| R8 | **RX FIFO and host speed.** 8-deep FIFO, `HPUSH` never blocks. An 11-byte packet delivers a byte every 267 cycles (5.3 us); overrun after about 48 us if the host does not drain | The cocotb host drains over SPI with ideal timing. A MicroPython host on an RP2350 has unknown latency | **High** for continuous traffic | Measure real host SPI round-trip; budget a drain strategy (early polling, burst reads) or a deeper buffer |
| R9 | **Physical synthesis unverified.** No area, STA, power, congestion or gate-level numbers for the block | RTL sim only | Med | Section 1.5 |
| R10 | **Partial packets after enable/reset.** Changing `usb_en` or resetting mid-packet leaves stray bits and no STOP | Documented and asserted as behaviour, not fixed | Low | Procedure: change `usb_en` only while idle |
| R11 | **Silent bad SYNC and false SYNC.** A corrupted SYNC is dropped without any event; a noise burst that looks like SYNC starts a bogus decode | Fuzz checks RTL equals model, not that dropping is acceptable for the application | Med | Decide whether a "bad SYNC" event is needed |
| R12 | **Single author, single model.** RTL was written as a line-by-line transcription of the model, so a model bug is reproduced in RTL and invisible to RTL-vs-model tests | Passes by construction. The independent checks are: RTL outputs decoded back to packets, the pre-existing ISS-vs-RTL lockstep for the engine, CRC vectors | **High** | Independent implementation or captured traces (R1) |
| R13 | **Not covered at all:** LS keep-alive (bare EOP) handled by absence of SYNC but not tested explicitly; hub/pre-amble traffic; reset/suspend/resume signalling; inter-packet gaps shorter than assumed; X/Z injection; power-up bus state | Not in the test plan | Low to Med | Add cases when a capture exists |
| R14 | **Architecture and rules.** The block is protocol-specific hardware inside a competition entry whose rules this session did not re-read | Not a technical test | Unknown | Check the competition rules on hard-coded protocol logic; the block is off by default and costs 30 flops |
| R15 | **Pin conflict.** With `usb_en = 1`, `ui_in[7:4]` are no longer readable; uio6/uio7 are ordinary bidirectional pins that a program could drive | No protection in hardware | Low | Software discipline; pin plan documented in `HOST_SPI.md` |

## 3. Protocol support matrix

"Tested" = RTL simulation of the engine at the pad pins (user-run), with the author's own device models.

| Protocol | Status | What exists | What it is not |
|---|---|---|---|
| UART TX/RX, 8N1 | **Implemented and tested** | `uart_tx.asm`, `uart_rx.asm`; bit times 20, 100, 434 cycles, rate mismatch, overrun | no parity/2 stop bits/flow control |
| SPI master, modes 0-3 | **Implemented and tested** | `spi_master.asm`, 4 modes, half period 12 and 24 cycles | no slave role |
| I2C master | **Implemented and tested** | `i2c_master.asm`: write, read, NACK, clock stretching | no repeated START, no multi-master, no slave |
| USB low-speed RX | **Implemented in RTL, simulation-tested only** | `ttpe_usb_rx.sv` + `usb_rx.asm` (hybrid: hardware bit slicer, software framing); synthetic waveforms with rate error, jitter, skew, stuff errors, lost EOP | not run on real USB, no CRC check, no PID decode, FIFO-limited (R8), electrically unverified (R2, R4) |
| USB low-speed TX / device response | **Theoretically possible, not implemented** | none | software NRZI + stuffing at 3-cycle granularity looks plausible; turnaround timing never analysed |
| USB full-speed (12 Mb/s) | **Not feasible** with this architecture | none | 4.17 cycles per bit at 50 MHz is below the 3-cycle instruction spacing plus resync needs; would need a dedicated SIE |
| 10BASE-T Ethernet | **Not feasible** | none | 10 Mb/s Manchester = 5 cycles per bit and 2.5 per half-bit; needs dedicated decoder, CRC32 and analog PHY/magnetics |

Note on wording: USB low-speed RX is **hybrid, not purely software-driven**: the per-bit work (resync, NRZI, unstuff,
SYNC/EOP) is in hardware; software does framing, byte assembly and FIFO traffic.

## 4. Completion report

Files changed or added (21; extract the zip at the repo root, nothing is committed by the author):

```
 M docs/HOST_SPI.md  docs/ISA.md  info.yaml
 M src/tt_um_ttpe.sv  src/ttpe_core.sv  src/ttpe_exec.sv  src/ttpe_host_spi.sv  src/ttpe_input_sync.sv
 M sw/tests/test_examples.py  sw/ttpe_iss.py
 M test/Makefile  test/test_ttpe_core.py  test/ttpe_lockstep.py  test/ttpe_scenarios.py  test/ttpe_top_tb.py
?? docs/USB_LS_RX.md  src/ttpe_usb_rx.sv  sw/examples/usb_rx.asm
?? sw/tests/test_usb_ls_rx.py  sw/usb_ls.py  test/test_ttpe_usb.py
?? docs/PHASE_E_REPORT.md   (this file)
```

State: RTL integrated and simulated by the user (79/79); Python suite 87/87 (author); ISA v0.2 unchanged; with
`MODE.usb_en = 0` the design is the previous design plus 30 idle flops. Open: physical flow, gate-level test, real-trace
replay, host drain budget, electrical interface.

Suggested review focus if the source is read before the flow runs (about 15 minutes): `ttpe_usb_rx.sv` lines for the
cell-counter/`smp_q` update and the sample block; the `s2_d` mux in `ttpe_input_sync.sv`; the `MODE` write path in
`ttpe_host_spi.sv`.
