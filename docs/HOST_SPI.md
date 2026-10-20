# TTPE chip top level and host SPI interface (RTL v0.2)

`tt_um_ttpe` is the Tiny Tapeout top level. A host (for example an RP2350 running MicroPython) talks to
the chip over a 3-wire SPI slave on three of the dedicated input pins. Through it the host loads the
program into the SRAM macro, starts / stops the Execution Unit, feeds the TX FIFO (data for `HPULL`)
and drains the RX FIFO (data from `HPUSH`). The ISA is untouched (v0.2 frozen).

## Pin assignment

| Pin | Direction | Function |
|---|---|---|
| `clk` | in | single clock, 50 MHz target |
| `rst_n` | in | active low, asynchronous assert, synchronous release (`ttpe_reset_sync`, 2 flops) |
| `ena` | in | unused |
| `uio[7:0]` | bidir | 100 % Execution Unit pins: `uio_out` / `uio_oe` driven by the program, `uio_in` feeds the synchroniser (source pins 0..7) |
| `ui_in[0]` | in | **SPI SCK** (also synchroniser source pin 8) |
| `ui_in[1]` | in | **SPI MOSI** (also source pin 9) |
| `ui_in[2]` | in | **SPI CS_N** (also source pin 10) |
| `ui_in[7:3]` | in | program inputs (source pins 11..15). With `MODE[0] = 1` (register 0x0B) source pins 12..15 (`ui_in[7:4]`) read the low-speed USB bit slicer instead of the pads: ui4 TICK, ui5 DATA, ui6 STOP, ui7 ERR (docs/USB_LS_RX.md); D+ / D- are uio6 / uio7 |
| `uo_out[0]` | out | **SPI MISO while CS_N is low** (synchronised), otherwise the program's sink pin 8 (`uo0`) |
| `uo_out[7:1]` | out | program outputs (sink pins 9..15) |

Consequences:

* A program that drives `uo0` does not own that pin during a host SPI frame; `uo_out[0]` returns to the
  program's value after the frame (two clock cycles after CS_N goes high).
* The program can still *read* `ui_in[2:0]` as source pins 8..10 (it sees the SPI traffic). Programs
  that need no extra inputs ignore them.
* All 8 `uio` pins are free for protocols; none is used by the host interface.

The SRAM macro instance path in the hierarchy is `u_core.u_mem.u_sram` (needed for the macro placement
config).

## SPI frame

Mode 0 (CPOL = 0, CPHA = 0), MSB first, 8 bit bytes, CS_N low for the whole frame.

```
CS_N   ‾\_______________________________________________________/‾
SCK    ___/‾\_/‾\_ ... 8 clocks ... _/‾\_ ...  8 clocks ... _____
MOSI   [ command byte               ][ payload byte 0 ][ payload byte 1 ] ...
MISO   [ STATUS byte                ][ response 0     ][ response 1     ] ...
```

* **Command byte** = `{rw, addr[6:0]}`. `rw = 1` read, `rw = 0` write.
* **MISO during the command byte** is always the STATUS byte (no extra command needed to poll status).
* **MISO during payload byte k** is the read data for that byte (reads), or `0x00` (writes and rejected
  commands). There is **no turnaround or dummy byte**: to read n bytes clock out the command byte plus
  n payload bytes (send anything on MOSI); the response is in bytes 1..n of the received frame.
* **Every payload byte is a separate access.** Writing two bytes to `SCRATCH` writes twice (last wins);
  the stream registers (`TX_PUSH`, `RX_POP`, `SRAM`) use this to move any number of bytes in one frame.
* A frame that is cut in the middle of a byte is discarded (the next CS_N falling edge resets the byte
  counter); no write takes place for the incomplete byte.

### Timing limits (clk cycles; clk = 50 MHz gives 20 ns)

All SPI inputs are sampled by the 50 MHz clock through a 2-flop synchroniser, so the limits are in clock
cycles. They are derived from the RTL cycle counts; pad / board delays are *not* included.

| Item | Limit | Why |
|---|---|---|
| SCK half period | >= 8 cycles (SCK <= 3.125 MHz at 50 MHz) | after the 8th rising edge the protocol layer needs 5 cycles to have the first response byte (SRAM prefetch) ready; the byte is loaded at the next falling edge, at least 7 cycles later |
| CS_N low to first SCK rising edge | >= 4 cycles arithmetic minimum, **use >= 8 (160 ns)** | MISO shows the status bit 7 three edges after CS_N falls |
| CS_N high between frames | >= 4 cycles | |
| MOSI | stable around the rising edge; may change one clock cycle after it | MOSI is synchronised with the same latency as SCK, so the bit value of the cycle in which SCK is first sampled high is used |
| MISO | sample on the SCK rising edge | MISO changes only after falling edges |

The chip-level tests run: half period 8 (also with 0..3 extra random cycles per half period), 9, 10, 11 +
jitter, 16; CS setup 4, 6, 8; MOSI changing one cycle after the rising edge. If the chip clock is slower
than 50 MHz, scale the SCK limit accordingly.

## STATUS byte

`[0]` halted, `[1]` illegal, `[2]` mem_free, `[3]` tx_empty, `[4]` tx_full, `[5]` rx_empty,
`[6]` rx_full, `[7]` rx overrun counter != 0.

Reset value `0x2D` (halted, mem_free, tx_empty, rx_empty).
`mem_free` becomes 1 a few cycles after `halted` (the fetch controller has to go idle); the SRAM window is
only usable while it is 1. Any later frame is slow enough that `halted = 1` implies `mem_free = 1`, but
poll `STATUS` after a program-initiated `HALT` instead of assuming it.

## Register map

| Addr | Name | Access | Description |
|---|---|---|---|
| 0x00 | ID | R | constant `0x54` |
| 0x01 | STATUS | R | see above |
| 0x02 | CTRL | W (R = 0) | write pulses: bit 0 **resume**, bit 1 **pc_load** (loads `PC`), bit 2 **tx_flush**, bit 3 **rx_flush**, bit 4 **halt**, bit 5 **core_reset** |
| 0x03 | PC_LO | R/W | holding register for the start PC, bits 7:0 |
| 0x04 | PC_HI | R/W | bit 0 = PC bit 8 |
| 0x05 | FIFO_CNT | R | `{rx_count[3:0], tx_count[3:0]}` |
| 0x06 | RX_OVR | R | rx overrun counter, 8 bit, saturating; cleared by `rx_flush` |
| 0x07 | SCRATCH | R/W | 8 bit scratch register (link test) |
| 0x08 | SADDR_LO | R/W | SRAM word address, bits 7:0 |
| 0x09 | SADDR_HI | R/W | bit 0 = word address bit 8 |
| 0x0A | ERR | R/W1C | bit 0 SRAM window used while the core runs, bit 1 rejected command, bit 2 TX_PUSH byte dropped (TX FIFO full). Write 1 to clear |
| 0x0B | MODE | R/W | bit 0 `usb_en`: connect the USB low-speed bit slicer to source pins 12..15. Bits 7:1 read 0, writes ignored. Reset 0; cleared by the chip reset only, not by `CTRL.core_reset`. Change it only while the USB bus is idle |
| 0x10 | TX_PUSH | W | stream: every payload byte is pushed into the TX FIFO (source of `HPULL`) |
| 0x11 | RX_POP | R | stream: every payload byte is the RX FIFO head (destination of `HPUSH`); `0x00` and no pop when empty |
| 0x12 | SRAM | R/W | stream window onto the SRAM at `SADDR` (32 bit words, **LSB byte first**, `SADDR` auto-increments after every 4th byte, 9 bit wrap) |

Unmapped addresses and wrong directions (write to a read-only register, read of a write-only one) set
`ERR[1]`, return `0x00` and ignore the write data.

### CTRL semantics

* `resume` and `pc_load` only act while `halted && mem_free` (core rule). After `HALT` or an illegal
  instruction the core's PC already points at the next instruction, so a plain `resume` continues.
* `pc_load` loads the `PC` holding register into the fetch PC. `pc_load | resume` in one write starts a
  program at that address.
* `halt` works at any time. Nothing new is accepted after the cycle in which the pulse is applied; an
  instruction that commits in that cycle completes normally; the held instruction is discarded.
  `SR`, `C0`, `C1` and the pin state are kept (the pins keep their last driven values). **The fetch PC is
  undefined afterwards: write `PC` and use `pc_load` before the next `resume`.**
* `core_reset` is `halt` plus: `SR`, `C0`, `C1` cleared, all `uio` pins released (`uio_oe = 0`),
  `uo_out = 0`, `illegal` cleared. FIFOs are not touched: add `tx_flush` / `rx_flush` bits (`0x3C` =
  halt + core_reset + both flushes).
* `halt` / `core_reset` win over `resume` in the same write. `core_reset` also works while halted
  (use it to release pins a stopped program left driven).
* The chip `rst_n` is only needed for power-up; the host can always recover with `CTRL = 0x3C`.

### Stream semantics

* **Read bytes are consumed when the master has sampled their MSB** (first SCK rising edge of the byte).
  A byte that was prepared but never clocked is not consumed. So reading exactly one byte from `RX_POP`
  pops exactly one, a command byte alone pops nothing, and reading 3 bytes of an SRAM word does not
  advance `SADDR`.
* `RX_POP` returns `0x00` for an empty FIFO, which is indistinguishable from a data byte `0x00`: read
  `FIFO_CNT` first and pop exactly that many bytes.
* `SRAM` reads prefetch the word at `SADDR` when the command byte arrives and the next word when the 4th
  byte of a word is consumed. `SADDR` is incremented when the 4th byte of a word has been consumed
  (reads) or written (writes).
* `SRAM` writes assemble 4 payload bytes (LSB first) and write the word, byte strobes `0xF`, on the 4th
  byte; an incomplete word at the end of a frame is discarded.
* The SRAM window only works while `mem_free = 1`. While the core runs, reads return 0 and writes are
  dropped, `ERR[0]` is set and the running program is not disturbed.
* `TX_PUSH` into a full TX FIFO drops the byte and sets `ERR[2]` (check `tx_full` / `FIFO_CNT` when
  streaming more than 8 bytes).

## Recipes

Load and run a program (image = list of 32 bit words from `sw/ttpe_asm.py`):

```
write SADDR_LO/HI = 0
write SRAM  <bytes of all words, LSB first, one frame>     (optional: read back and compare)
write TX_PUSH <up to 8 bytes the program will HPULL>
write PC_LO/HI = 0
write CTRL = 0x03                                          (pc_load | resume)
```

Stop an endless program and start another: `CTRL = 0x3C`, wait until `STATUS & 0x05 == 0x05`, load the
new image, write PC, `CTRL = 0x03`.

Cycle relationship for deterministic experiments: if the last SCK rising edge of the CTRL byte is
sampled as high first in clock cycle `c` (cycle in which the pad value is applied), the `resume` pulse is
in cycle `c + 3` and ISS cycle 0 of the program is clock cycle `c + 4`. The chip-level tests use this to
compare the pins with the ISS cycle by cycle.

## MicroPython host example (RP2350)

Pin numbers are placeholders: use the pins you wired to `ui_in[0]` (SCK), `ui_in[1]` (MOSI), `ui_in[2]`
(CS_N) and `uo_out[0]` (MISO). 1 MHz is far below the 3.125 MHz limit.

```python
from machine import Pin, SPI
import time

CS   = Pin(17, Pin.OUT, value=1)
SPI0 = SPI(0, baudrate=1_000_000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB,
           sck=Pin(18), mosi=Pin(19), miso=Pin(16))

ID, STATUS, CTRL, PC_LO, PC_HI, FIFO_CNT, RX_OVR, SCRATCH = 0, 1, 2, 3, 4, 5, 6, 7
SA_LO, SA_HI, ERR, TX_PUSH, RX_POP, SRAM = 8, 9, 0x0A, 0x10, 0x11, 0x12

def xfer(tx):
    rx = bytearray(len(tx))
    CS(0)
    SPI0.write_readinto(bytes(tx), rx)
    CS(1)
    return rx                       # rx[0] = STATUS, rx[i] = response to tx[i]

def wr(addr, *data):
    xfer([addr] + list(data))

def rd(addr, n=1):
    return xfer([0x80 | addr] + [0] * n)[1:]

def status():
    return xfer([0x80 | STATUS])[0]

def load(words, base=0):
    wr(SA_LO, base & 0xFF); wr(SA_HI, base >> 8)
    data = bytearray()
    for w in words:
        data += w.to_bytes(4, "little")
    wr(SRAM, *data)                 # one frame; split into chunks if memory is tight

def read_back(n, base=0):
    wr(SA_LO, base & 0xFF); wr(SA_HI, base >> 8)
    b = rd(SRAM, 4 * n)
    return [int.from_bytes(b[4 * i:4 * i + 4], "little") for i in range(n)]

def start(pc=0):
    wr(PC_LO, pc & 0xFF); wr(PC_HI, pc >> 8)
    wr(CTRL, 0x03)                  # pc_load | resume

def stop():
    wr(CTRL, 0x3C)                  # core_reset | halt | flush both FIFOs
    while status() & 0x05 != 0x05:  # halted and mem_free
        pass

def drain_rx():
    n = rd(FIFO_CNT)[0] >> 4
    return list(rd(RX_POP, n)) if n else []

assert rd(ID)[0] == 0x54
```

Producing the image on a PC (`prog.bin` is then copied to the RP2350 and read with `open("prog.bin", "rb")`):

```python
import sys, struct
sys.path.insert(0, "sw")
import ttpe_asm as asm
words = asm.assemble(open("sw/examples/uart_tx.asm").read(), defines={"BIT": 434}).image()
open("prog.bin", "wb").write(struct.pack("<%dI" % len(words), *words))
```

## Verification status

| What | Status |
|---|---|
| `test/test_ttpe_top.py` (13 chip-level tests, pins only) on the real RTL under Icarus + cocotb | **Not run by the author** (no simulator in the authoring sandbox). Run `make TARGET=top` in `test/`. |
| Same test code (`test/ttpe_top_tb.py`) against a line-by-line Python transcription of the new RTL (`ttpe_spi_phy`, `ttpe_host_spi`, `ttpe_reset_sync`, `tt_um_ttpe` glue, halt / core_reset) | 13 / 13 pass. 25 injected faults in the transcription were run against it (FIFO pop timing, address auto-increment, error flags, halt / core_reset effects, MISO mux, MOSI alignment, status bits, prefetch, ...): 21 are caught, 4 survive and are equivalent at the pins (halt gating of `instr_accept`, a redundant `widx` reset at cs_fall, the `PC_HI` mask above bit 0, and the `ld_real` race below, which needs an HPUSH inside a 7-cycle window). This checks the logic and the testbench, **not the Verilog text**. |
| Core lockstep (`test_ttpe_core.py`) against the transcription after the halt / core_reset change | 43 / 43 pass (the new ports are tied to 0 there). |
| Synthesis, area, timing, gate-level simulation | Not run. No numbers are claimed. |

## Not covered / assumptions to confirm

* The register map, the `0x54` ID and the top module name `tt_um_ttpe` are design choices of this
  repository, not competition requirements; the Tiny Tapeout flow needs `info.yaml` `top_module` to match.
* The reset synchroniser's 2-cycle release delay is not observable at the pins and is not tested beyond
  "everything works after release".
* The `ld_real` race: if an `HPUSH` lands in the RX FIFO between the moment `RX_POP` prepares a byte (FIFO
  empty, byte = 0x00) and the master sampling it, that byte is returned as 0x00 and not popped. Read
  `FIFO_CNT` first.
* SPI pad timing (input/output delays) is not modelled; the limits above are clock-cycle arithmetic.
