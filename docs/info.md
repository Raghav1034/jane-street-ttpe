<!---
Tiny Tapeout datasheet for the TTPE project. Written for the final submission; claims are limited to what
is verified (see "Verification status"). Wiring values in the board section are recommendations, not
measured on hardware.
--->

## How it works

TTPE (Time-Triggered Protocol Engine) is a small programmable I/O engine in the spirit of the RP2040 PIO or a
TI PRU. A program of up to 512 32-bit instructions lives in an on-chip 512x32 SRAM (IHP macro
`RM_IHPSG13_1P_512x32_c2_bm_bist`) and is loaded by a host over SPI. Every instruction carries a 16-bit delta:
it takes effect exactly `delta + 1` cycles after the previous one (minimum delta 2 for back-to-back execution),
so pin changes land on exact clock cycles. New protocols are firmware, not new silicon.

* **Clock:** 50 MHz target (20 ns cycle). The timing constants of the examples assume exactly 50 MHz.
* **ISA v0.2:** 4-bit opcode, 14 defined (`NOP PINS WAIT JMP JHI JLO SHOUT SAMPLE REG DJNZ HPUSH HPULL HALT JSB`), 2 reserved (trapped as illegal).
  Registers: 8-bit `SR`, two 8-bit counters `C0`, `C1`. 16 inputs through a 2-flop synchroniser, 16 outputs.
  A taken branch costs 3 extra cycles. Full reference: `docs/ISA.md`.
* **Program pins:** source pins 0..7 = `uio[7:0]`, 8..15 = `ui_in[7:0]`; sink pins 0..7 = `uio[7:0]`, 8..15 = `uo_out[7:0]`.
* **Host link:** SPI mode 0 slave on `ui_in[0]` SCK, `ui_in[1]` MOSI, `ui_in[2]` CS_N, `uo_out[0]` MISO (while CS_N
  is low). Command byte = `{rw, addr[6:0]}`, `rw = 1` reads; MISO during the command byte is the STATUS byte.
  SCK up to 3.125 MHz at 50 MHz. Details: `docs/HOST_SPI.md`.

### Host register map

| Addr | Name | Access | Meaning |
|---|---|---|---|
| 0x00 | ID | R | `0x54` |
| 0x01 | STATUS | R | `[0]` halted, `[1]` illegal, `[2]` mem_free, `[3]` tx_empty, `[4]` tx_full, `[5]` rx_empty, `[6]` rx_full, `[7]` rx overrun |
| 0x02 | CTRL | W | `[0]` resume, `[1]` pc_load, `[2]` tx_flush, `[3]` rx_flush, `[4]` halt, `[5]` core_reset |
| 0x03/0x04 | PC_LO/HI | R/W | start PC |
| 0x05 | FIFO_CNT | R | `{rx_count, tx_count}` |
| 0x06 | RX_OVR | R | saturating RX overrun counter |
| 0x07 | SCRATCH | R/W | link test |
| 0x08/0x09 | SADDR_LO/HI | R/W | SRAM word address |
| 0x0A | ERR | R/W1C | sticky error flags |
| 0x0B | MODE | R/W | `[0]` `usb_en`: low-speed USB bit slicer drives `ui_in[7:4]`; reset 0 |
| 0x10 | TX_PUSH | W | stream into the 8-deep TX FIFO (source of `HPULL`) |
| 0x11 | RX_POP | R | stream out of the 8-deep RX FIFO (destination of `HPUSH`) |
| 0x12 | SRAM | R/W | stream window onto the SRAM, little-endian words, `SADDR` auto-increments |

### Protocols (programs in `sw/examples`)

| Program | Pins | Notes |
|---|---|---|
| `uart_tx.asm`, `uart_rx.asm` | TXD `uio[1]`, RXD `uio[2]` | 8N1; tested at 20, 100 and 434 cycles per bit (115200 baud = 434) |
| `spi_master.asm` | CS `uio[0]`, MOSI `uio[1]`, MISO `uio[2]`, SCK `uio[3]` | modes 0-3, up to 2.78 MHz |
| `i2c_master.asm` | SCL `uio[2]`, SDA `uio[3]` (open drain) | 100 / 400 kHz, clock stretching, NACK -> `uo[7]` + STOP, repeated START via `n[7]` |
| `usb_rx.asm` | D+ `uio[6]`, D- `uio[7]`; `ui_in[7:4]` = TICK/DATA/STOP/ERR; `uo[6]` packet end, `uo[5]` bad packet | needs `MODE[0] = 1`; hardware bit slicer `ttpe_usb_rx` (30 flops) + software framing |
| `usb_tx.asm` | D+ `uio[6]`, D- `uio[7]` | software only; host supplies PID, payload and CRC |

I2C register read: push `A0 81 REG A1 n` into the TX FIFO (write address, count with Sr flag, register, read
address, read count); the data arrives in the RX FIFO. The count byte `n` is `n[7]` = repeated-START flag and `n[6:0]`
= count (0 = 256). USB packets for `usb_tx.asm`: push `n` then `n` bytes (e.g. ACK = `01 D2`).

## How to test

1. Reset the chip (`rst_n` low then high) with the 50 MHz clock running. Flops do not start at 0 on silicon; the
   core stays halted until the host starts it. If the project was deselected on the shuttle mux it was powered
   down: reset again and **reload the SRAM**.
2. Read register `0x00` (expect `0x54`) and `0x01` (expect `0x2D` after reset).
3. Assemble a program (`make -C sw images`, produces `build/<name>.hex`, `.bin`, `.py`), halt the core, write
   the words to the `SRAM` window from `SADDR = 0`, write `PC_LO/HI`, then write `CTRL = 0x03` (pc_load | resume).
4. Push TX bytes to `0x10`, read RX bytes from `0x11` (use `FIFO_CNT` to know how many).

A MicroPython driver for the RP2350 on the demo board is `sw/host_demo_rp2350.py` (command layer tested on CPython
against a model of the register file; the board layer is untested).

## External hardware

All 24 pins are 3.3 V CMOS. There are no analog pads, no internal pull-ups and no protection: everything below goes
on a Pmod or breadboard and **has not been measured on hardware**.

| Protocol | Wiring |
|---|---|
| Host link | RP2350 GPIO to `ui_in[0..2]` (SCK, MOSI, CS_N) and from `uo_out[0]` (MISO) |
| UART | `uio[1]` TXD, `uio[2]` RXD, 3.3 V levels (use a transceiver for RS-232) |
| SPI | `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK |
| I2C | `uio[2]` SCL, `uio[3]` SDA, each with an external **4.7 kOhm pull-up to 3.3 V** (the program only pulls low or releases) |
| Low-speed USB (device role) | D+ `uio[6]`, D- `uio[7]`; **1.5 kOhm pull-up from D- to 3.3 V**; **27 to 33 Ohm series resistors** in both data lines; never connect VBUS (5 V) to a pin |

USB caveats: the receiver thresholds of the pads against USB low-speed levels were not analysed, the series-resistor
and pull-up values are typical figures from memory (check the USB specification and your transceiver design guide),
and the program idles D+ low / D- high only while it drives them. The TX FIFO must stay ahead of the wire
(one byte every ~267 cycles). The RX FIFO is 8 deep: a long packet needs a host that drains it while the packet arrives.

## Verification status

Simulation only; nothing was measured on silicon.

* Python model (ISS) with 121 unit and protocol tests; the Execution Unit RTL runs cycle-for-cycle against it in
  cocotb lockstep (`make -C test TARGET=core`), plus chip-pin tests (`TARGET=top`) and the USB slicer against its
  model (`TARGET=usb`); gate-level simulation and the physical flow (DRC, LVS, precheck) run in CI.
* The USB signalling conventions (SYNC, NRZI, stuffing, EOP, tolerances) come from the author's memory of the
  specification and were checked only against the author's own model, never against a real host or a captured trace.
* Known open items: the CI log shows Max Slew / Max Cap warnings and 874 inserted hold buffers (not compared to a
  baseline); see `docs/PHASE_E_REPORT.md`.
