# TTPE Execution Unit, FIFOs and core wrapper (RTL v0.2)

ISA v0.2 is frozen (`docs/ISA.md`, `sw/ttpe_isa.py`, `rtl/ttpe_isa_pkg.sv`). This file documents the RTL that
implements it. Nothing here changes the ISA.

## Files

| File | Role |
|---|---|
| `rtl/ttpe_input_sync.sv` | 2-FF synchroniser for `{ui_in, uio_in}` plus one delay stage for edge detection |
| `rtl/ttpe_fifo.sv` | 8 x 8 synchronous FIFO, first-word-fall-through, flush, saturating drop counter |
| `rtl/ttpe_exec.sv` | Execution Unit (accept / delta countdown / commit / branch / pins / registers) |
| `rtl/ttpe_core.sv` | `ttpe_mem_subsys` + `ttpe_exec` + TX/RX FIFOs + host run control |
| `rtl/sram_512x32_bb.v` | Synthesis black box of the SRAM macro, active only without `-DSIM` |
| `rtl/ttpe_reset_sync.sv`, `rtl/ttpe_spi_phy.sv`, `rtl/ttpe_host_spi.sv`, `rtl/tt_um_ttpe.sv` | Chip level: reset synchroniser, host SPI slave, register map, Tiny Tapeout top. See `docs/HOST_SPI.md` |

## Timing contract implemented (identical to `sw/ttpe_iss.py`)

* accept in cycle A: `valid_instruction` and (unit empty or committing) and no `pc_load` and no halt
* commit in A + 1 + D; `WAIT` and `HPULL` commit in the first cycle >= A + 1 + D where their condition holds
* registers, pins and FIFO operations update at the edge ending the commit cycle (pad changes at commit + 1)
* taken branch: `pc_load` in the commit cycle (3-cycle penalty comes from the fetch controller)
* inputs: `sync(C) = pad(C-2)`; edges compare `sync(C)` with `sync(C-1)`

## Host interface of `ttpe_core`

All inputs are sampled at the edge that ends the cycle in which they are applied.

| Group | Signals | Behaviour |
|---|---|---|
| SRAM | `host_req/we/addr/wdata/wstrb`, `host_gnt/wdone/rvalid/rdata` | Same protocol as `ttpe_sram_ctrl`. Granted only while `mem_free = 1` (fetch controller idle), so fetch timing is deterministic while a program runs. |
| Run control | `host_resume`, `host_pc_load` + `host_pc_val` | `host_resume` is a 1-cycle pulse: clears `halted` and `illegal`; the first instruction is accepted 3 cycles after the cycle following the pulse. `host_pc_load` loads the fetch PC (may be in the same cycle as the resume pulse). Both are ignored unless `halted && mem_free`. After `HALT` or an illegal instruction the PC points at the next instruction. |
| TX FIFO (read by `HPULL`) | `tx_push`, `tx_wdata`, `tx_fifo_flush`; `tx_full`, `tx_empty`, `tx_count` | Push into a full FIFO is dropped (check `tx_full`), even if `HPULL` pops in the same cycle. Flush wins over push. |
| RX FIFO (written by `HPUSH`) | `rx_pop`, `rx_rdata`, `rx_fifo_flush`; `rx_full`, `rx_empty`, `rx_count`, `rx_overrun_cnt` | `rx_rdata` is the head byte while `!rx_empty`. `HPUSH` into a full FIFO is dropped and counted in `rx_overrun_cnt` (8 bit, saturates at 255), even if the host pops in the same cycle. `rx_fifo_flush` empties the FIFO and clears the counter and wins over `HPUSH`. |
| Stop | `host_halt`, `host_core_reset` | 1-cycle pulses, work any time, win over `host_resume`. `host_halt`: nothing new is accepted, an instruction committing in that cycle completes, the held instruction is discarded, SR/C0/C1/pins are kept, the fetch PC is undefined (load one before resuming). `host_core_reset`: the same plus SR, C0, C1, pin registers and `illegal` cleared. FIFOs and synchroniser are untouched. Not part of the ISA. |
| Status | `halted`, `illegal`, `mem_free` | `illegal` is sticky until `host_resume`. |
| Debug | `dbg_*` | Testbench visibility; leave open at the chip top. |

### Differences from what the reference model assumes (none are visible to a program)

1. **Reset state is halted.** The ISS starts running at cycle 0. In RTL the host loads the program and pulses `host_resume`; ISS cycle 0 is the cycle after the pulse.
2. **The synchroniser free-runs from reset.** The ISS starts it at 0 when the core starts. The first instruction can commit no earlier than cycle 4, by which time both hold the same real pad history, so no program can tell them apart. In RTL a pad that is high at reset does not create the spurious rising edge that `docs/ISA.md` section 4 warns about.
3. The ISS has no flush and no host pop. The testbench models both on the ISS side (see `test/ttpe_lockstep.py`).

## Verification status (read this before trusting it)

| What | Status |
|---|---|
| `test/ttpe_lockstep.py` + `test/ttpe_scenarios.py` against a Python transcription of the RTL | 43 scenarios pass in the authoring sandbox. This checks the logic and the testbench, **not the Verilog**. 28 injected faults in the transcription (accept gating, off-by-one delta, WAIT edge swap, synchroniser depth, FIFO full/flush, page bit, host gating, ...) are all caught; the one survivor is a deliberate equivalent mutant. |
| `test/test_ttpe_core.py` on the real RTL under Icarus + cocotb | **Not run by the author** (no simulator in the sandbox). Run `make regress` in `test/`. |
| `sw/tests` | 71 tests pass (also with `-W error`). |
| Synthesis, area, timing | Not run. No numbers are claimed. |

`ttpe_exec.sv` no longer uses constant bit-selects inside `always_comb` (named wires instead), which
removes the Icarus "constant selects in always_* processes" messages. The `test_ttpe_core.py` tests tie
the new `host_halt` / `host_core_reset` ports to 0; they are exercised at chip level
(`test/test_ttpe_top.py`, `t_run_control`).

## Running

```
cd test
make regress                       # mem + core, PDK models then stand-in model
make TARGET=core                   # core only, PDK models
make TARGET=core USE_PDK_SRAM=0    # core only, stand-in macro
make lint TARGET=core              # iverilog -Wall elaboration only
```
Subset: cocotb 2.x `COCOTB_TEST_FILTER=fuzz`, cocotb 1.x `TESTCASE=test_fuzz_3`.
The scenarios total about 120 000 simulated cycles.

## Open items

* (done, see `docs/HOST_SPI.md`) host SPI slave, top-level wrapper, pin muxing.
* I2C repeated START (firmware only).
* `config.json`, PDN, macro placement, synthesis numbers.
* Optional area savings once numbers exist: reset-less FIFO storage, a single `prev` flop instead of the 16-bit delay stage.
