# TTPE Pinout Map and External Hardware Guide

> **Verification boundary:** All pin assignments match the RTL (`src/tt_um_ttpe.sv`, `src/ttpe_core.sv`, `src/ttpe_usb_rx.sv`) and `info.yaml`. All external component values and electrical notes below are **design recommendations not measured on silicon** — IHP SG13CMOS5L pad electricals are uncharacterized (Competition FAQ, *Design -> How fast can the I/O go?*).

---

## 1. Chip Top-Level Pin Map (`tt_um_ttpe`)

Tiny Tapeout exposes **24 digital I/O pins** (3.3 V CMOS on the ETR demo board), plus `clk`, `rst_n`, and `ena` (`src/tt_um_ttpe.sv:L23-32`, `info.yaml:L36-62`).

| Pin | Dir | Reset State | ISA Pin ID | Primary Function | Alternate / Muxed Function |
|---|---|---|---|---|---|
| `clk` | In | — | — | System clock (`50 MHz` nominal, `20 ns` period; `info.yaml:L12`) | — |
| `rst_n` | In | Active-low | — | Async-assert, sync-release reset (`src/ttpe_reset_sync.sv`) | — |
| `ena` | In | — | — | Unused (`_unused` sink in `src/tt_um_ttpe.sv:L148`) | Shuttle power-gate: deselection loses all state and SRAM contents; reload required |
| `ui_in[0]` | In | — | Source `8` (`ui0`) | **Host SPI `SCK`** (Mode 0, <= `3.125 MHz` @ `50 MHz`) | Readable by program via 2-FF sync |
| `ui_in[1]` | In | — | Source `9` (`ui1`) | **Host SPI `MOSI`** | Readable by program via 2-FF sync |
| `ui_in[2]` | In | — | Source `10` (`ui2`) | **Host SPI `CS_N`** (active low) | Readable by program via 2-FF sync |
| `ui_in[3]` | In | — | Source `11` (`ui3`) | General-purpose program input | Recommended TT demo-board UART RX (if firmware remapped) |
| `ui_in[4]` | In | — | Source `12` (`ui4`) | General-purpose program input (`MODE[0]=0`) | Virtual USB `TICK` when `MODE[0]=1` (`src/ttpe_usb_rx.sv:L11-20`) |
| `ui_in[5]` | In | — | Source `13` (`ui5`) | General-purpose program input (`MODE[0]=0`) | Virtual USB `DATA` when `MODE[0]=1` |
| `ui_in[6]` | In | — | Source `14` (`ui6`) | General-purpose program input (`MODE[0]=0`) | Virtual USB `STOP` when `MODE[0]=1` |
| `ui_in[7]` | In | — | Source `15` (`ui7`) | General-purpose program input (`MODE[0]=0`) | Virtual USB `ERR` when `MODE[0]=1` |
| `uo_out[0]` | Out | `0` | Sink `8` (`uo0`) | **Host SPI `MISO`** while `CS_N=0` (`src/tt_um_ttpe.sv:L145`) | Program output `uo0` while `CS_N=1` (2 cycles after `CS_N` rises) |
| `uo_out[1..3]` | Out | `0` | Sink `9..11` (`uo1..3`) | General-purpose program outputs (push-pull only, no OE) | — |
| `uo_out[4]` | Out | `0` | Sink `12` (`uo4`) | General-purpose program output (push-pull only) | Recommended TT demo-board UART TX (if firmware remapped) |
| `uo_out[5]` | Out | `0` | Sink `13` (`uo5`) | General-purpose program output | `usb_rx` / `usb_device_ack`: bad-packet toggle |
| `uo_out[6]` | Out | `0` | Sink `14` (`uo6`) | General-purpose program output | `usb_rx` / `usb_device_ack`: packet-end toggle; `spi_slave`: frame-end toggle |
| `uo_out[7]` | Out | `0` | Sink `15` (`uo7`) | General-purpose program output | `i2c_master`: NACK error flag |
| `uio[0]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `0` (`uio0`) | Program bidirectional pin 0 | SPI `CS` |
| `uio[1]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `1` (`uio1`) | Program bidirectional pin 1 | UART `TXD` / SPI `MOSI` |
| `uio[2]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `2` (`uio2`) | Program bidirectional pin 2 | UART `RXD` / SPI `MISO` / I2C `SCL` |
| `uio[3]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `3` (`uio3`) | Program bidirectional pin 3 | SPI `SCK` / I2C `SDA` |
| `uio[4..5]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `4..5` (`uio4..5`) | Program bidirectional pins 4..5 | Spare (e.g., optional USB soft-connect pull-up control) |
| `uio[6]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `6` (`uio6`) | Program bidirectional pin 6 | Low-speed USB `D+` (hardwired into `ttpe_usb_rx` when `MODE[0]=1`) |
| `uio[7]` | Bidir | Hi-Z (`oe=0`, `out=0`) | Src/Snk `7` (`uio7`) | Program bidirectional pin 7 | Low-speed USB `D-` (hardwired into `ttpe_usb_rx` when `MODE[0]=1`) |

**Key architectural constraints on pin selection:**
1. All 8 `uio[7:0]` pins are 100% free of the host SPI link so protocols run concurrently with host FIFO streaming.
2. Only `uio[7:0]` support tri-state (`PINS float`) and open-drain (`SHOUT od`); `uo_out[7:0]` are permanently driven push-pull (`sw/ttpe_asm.py:L428-429,465-466`).
3. `sw/ttpe_asm.py` parses `-D NAME=VALUE` as integer expressions only (`sw/ttpe_asm.py:L538-540`), not pin-name strings. Remapping pins in existing `.asm` files is a zero-RTL firmware edit (replacing pin literals, or guarding alternate pinouts behind `.if <FLAG>` so `-D <FLAG>=1` selects them).

---

## 2. Protocol-to-Pin Comparison vs Tiny Tapeout Recommended Pinouts

Reference: Tiny Tapeout Recommended Pinouts (`https://tinytapeout.com/specs/pinouts/`).

| Protocol | TTPE Example(s) | TTPE Pins | Tiny Tapeout Recommended Pinout | Match / Deviation | Remap Cost & Notes |
|---|---|---|---|---|---|
| **Host SPI** (control & SRAM load) | RTL (`src/tt_um_ttpe.sv`) | `ui_in[0]` SCK, `ui_in[1]` MOSI, `ui_in[2]` CS_N, `uo_out[0]` MISO (`CS_N=0`) | Pmod SPI: `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK | **Intentional deviation** | **Fixed in RTL.** Keeps `uio[7:0]` 100% available for emulated protocols while the RP2350 host streams FIFOs. |
| **UART** (8N1) | `sw/examples/uart_tx.asm`, `sw/examples/uart_rx.asm` | `uio[1]` TXD, `uio[2]` RXD | **Pmod UART (top row):** `uio[1]` TXD, `uio[2]` RXD (optional `uio[0]` CTS, `uio[3]` RTS).<br>**Demo-board UART-to-USB:** Option A `ui_in[3]` RX / `uo_out[4]` TX; Option B `ui_in[1]` RX / `uo_out[0]` TX. | **Matches Pmod UART top row.**<br>**Deviates from Demo-board UART-to-USB.** | **Zero RTL change (firmware-only).** Option A (`ui_in[3]` / `uo_out[4]`) is conflict-free: changing `uio1 -> uo4` and `uio2 -> ui3` (or wrapping in `.if DEMO_UART` for `-D DEMO_UART=1`) is free. Option B (`ui_in[1]` / `uo_out[0]`) **collides** with Host SPI `MOSI`/`MISO` and cannot share the board with live host SPI. |
| **SPI Master** (Modes 0–3) | `sw/examples/spi_master.asm` | `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK | **Pmod SPI (top row) / SPI RAM:** `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK | **Exact match** | **Zero RTL change (firmware-only).** Outputs (`CS`, `MOSI`, `SCK`) may also move to `uo_out[1..7]` and `MISO` to `ui_in[3..7]` if needed; current `.asm` uses literal `uio0..3`. |
| **SPI Slave** (Modes 0–3) | `sw/examples/spi_slave.asm` *(on `phase2/wp4b-spi-slave`, pending lockstep merge)* | `uio[0]` CS_N, `uio[1]` MOSI, `uio[2]` MISO (Hi-Z when `CS_N=1`), `uio[3]` SCK; `uo[6]` frame-end toggle | **Pmod SPI (top row):** `uio[0]` CS, `uio[1]` MOSI, `uio[2]` MISO, `uio[3]` SCK | **Exact match** (plus optional `uo[6]` debug pin) | **Zero RTL change (firmware-only).** `MISO` **must** remain on `uio[0..7]` because Hi-Z release requires `PINS float` (`uio_oe=0`). |
| **I2C Master** (100/400 kHz) | `sw/examples/i2c_master.asm` | `uio[2]` SCL, `uio[3]` SDA (open-drain); `uo[7]` NACK error flag | **Pmod I2C (top row):** `uio[2]` SCL, `uio[3]` SDA (optional `uio[0]` INT, `uio[1]` RESET) | **Exact match** (optional INT/RESET unused; `uo[7]` debug flag added) | **Zero RTL change (firmware-only).** `SCL` and `SDA` **must** remain on `uio[0..7]` for open-drain (`PINS float` / `SHOUT od`). |
| **I2C Target** (100/400 kHz) | `sw/examples/i2c_target.asm` | `uio[2]` SCL, `uio[3]` SDA (open-drain) | **Pmod I2C (top row):** `uio[2]` SCL, `uio[3]` SDA | **Exact match** | **Zero RTL change (firmware-only).** `SCL` and `SDA` **must** remain on `uio[0..7]` for open-drain and clock stretching. |
| **Low-Speed USB** (1.5 Mbit/s, device role) | `sw/examples/usb_rx.asm`, `sw/examples/usb_tx.asm`, `sw/examples/usb_device_ack.asm` | `uio[6]` D+, `uio[7]` D-; virtual `ui_in[4..7]` = TICK/DATA/STOP/ERR (`MODE[0]=1`); `uo[5]` bad-packet, `uo[6]` packet-end | **None official** on `tinytapeout.com/specs/pinouts/`. Community USB CDC projects typically use `uio[0..2]` (`D+`, `D-`, pull-up enable). | **No official standard; avoids `uio[0..3]` conflict** | **`usb_rx` / `usb_device_ack`: NOT firmware-remappable** (`src/ttpe_usb_rx.sv` hardwires `dp_q=s2[6]`, `dm_q=s2[7]`, virtual pins `s2[15:12]`). **`usb_tx.asm` (software-only):** firmware-remappable to any `uio[0..7]` pair (`PINS float` needs `uio`). |
| **10BASE-T Ethernet** (10 Mbit/s) | *None* | — | None | **Out of scope** | See Section 4 below for architectural, bandwidth, and electrical reasons. |

---

## 3. External Hardware Requirements per Protocol

All external circuits go on a Pmod or breadboard connected to the Tiny Tapeout ETR demo board (`https://tinytapeout.com/specs/pcb-etr/`). **None of the electrical values below have been measured on CMOS5L silicon.**

### 3.1 Common Board Baseline
* **Logic level:** 3.3 V CMOS digital I/O on all 24 pins (`ui_in`, `uo_out`, `uio`). No analog pads, no internal pull-ups, no 5 V tolerance.
* **Uncharacterized I/O timing:** Tiny Tapeout estimates ~10 ns round-trip through pads/mux and ~66 MHz max output frequency (unmeasured estimates; Competition FAQ, *Design -> How fast can the I/O go?*). External wiring capacitance should be kept short.
* **Power & reset:** `rst_n` must be asserted after selection/power-up. Deselecting the project on the shuttle mux powers it down and erases SRAM (`docs/info.md:L59-61`).

### 3.2 Per-Protocol External Components

| Protocol | Required External Parts | Wiring & Electrical Notes |
|---|---|---|
| **Host SPI** | None (on ETR demoboard) | Driven directly by the onboard RP2350 GPIO (`ui_in[0..2]`, `uo_out[0]`) at 3.3 V. Keep SCK <= `3.125 MHz` at `50 MHz` chip clock (`1 MHz` recommended in `docs/HOST_SPI.md:L163`). |
| **UART** | None for 3.3 V TTL; RS-232 transceiver (e.g., MAX3232) for RS-232 | `uio[1]` TXD, `uio[2]` RXD at 3.3 V CMOS. Connect GND between TTPE and USB-UART adapter. Never connect 5 V TTL directly. If remapped to `ui_in[3]` / `uo_out[4]`, the ETR demoboard's RP2350 bridges UART over its USB-C port with no external adapter. |
| **SPI (Master & Slave)** | Optional `10 kΩ` pull-up on `CS_N` (`uio[0]`) to 3.3 V | Direct 3.3 V CMOS (`uio[0..3]`). A weak pull-up on `CS_N` keeps an external SPI slave deselected while TTPE is held in reset (`uio_oe=0`). Keep wiring short for multi-MHz SCK; in `spi_slave.asm` CPHA=1 modes, polled `t_dis` can reach `17` cycles (`340 ns` @ `50 MHz`), so multi-slave buses sharing MISO must wait before selecting another slave. |
| **I2C (Master & Target)** | **Two external pull-up resistors** on `uio[2]` (SCL) and `uio[3]` (SDA) to **3.3 V** | TTPE drives open-drain only (`PINS low` / `PINS float` / `SHOUT od`).<br>* **100 kHz Standard-mode:** `4.7 kΩ` to 3.3 V is typical.<br>* **400 kHz Fast-mode:** `2.2 kΩ` (or `1.8–2.2 kΩ`) to 3.3 V is recommended `[INFERENCE]` to keep RC rise time $t_r \le 300\text{ ns}$ (`i2c_target.asm` `-D TSU_DAT=22` budgets worst-case UM10204 `t_r = 300 ns`).<br>* Use a bidirectional I2C level shifter (e.g., BSS138) if interfacing with a 5 V I2C bus. |
| **Low-Speed USB (1.5 Mbit/s, Device Role)** | * **`1.5 kΩ` (±5%) pull-up on `D-` (`uio[7]`)** to 3.3 V (USB 2.0 §7.1.5 Low-Speed identification)<br>* **Series resistors** (`27–33 Ω` standard; `68 Ω` used in TT community breakouts) in both `D+` (`uio[6]`) and `D-` (`uio[7]`)<br>* USB connector breakout (GND, D+, D- only) | * **Pull-up polarity:** Low-Speed USB pulls **`D-`** high (unlike Full-Speed USB, which pulls `D+` high). For software-controlled attach (soft-connect), drive the `1.5 kΩ` resistor from a spare pin (e.g., `uio[5]` or `uo_out[1]`) rather than hard-tying to 3.3 V; per USB 2.0 §7.1.5, a self-powered device must not back-feed `D-` when host `VBUS` is off.<br>* **5 V safety:** **Never connect USB 5 V `VBUS`** to any Tiny Tapeout pin.<br>* **Electrical caveats (unverified):** CMOS5L `uio[6..7]` are single-ended 3.3 V digital pads, not a USB differential receiver (±200 mV sensitivity across 0.8–2.5 V common-mode) and do not guarantee USB LS `75–300 ns` slew-rate control. Treat a passive resistor breakout as a lab-bench demonstrator only; full USB 2.0 electrical compliance requires an external USB 1.1 PHY/transceiver. |

### 3.3 Tiny Tapeout USB Pmod Options (Research Findings)
* **Official Tiny Tapeout store / specs:** No raw USB `D+`/`D-` Pmod is listed on `https://tinytapeout.com/specs/pinouts/` or stocked on the Tiny Tapeout store (official Pmods are Audio, Tiny VGA, Dual 7-segment, QSPI Flash/PSRAM, Gamepad, and Simon Says). The ETR demoboard's USB-C port connects to the RP2350 only (power, MicroPython REPL, and UART-to-USB bridge).
* **Community Tiny Tapeout USB breakouts:** Prior Tiny Tapeout USB projects (e.g., Uri Shaked's TT04–SKY26 USB CDC device, Maximo Balestrini's dual USB CDC) use a passive USB-A/C breakout wired through `68 Ω` series resistors and a `1.5 kΩ` pull-up resistor. Those designs target Full-Speed USB (pull-up on **`D+`** at 48 MHz) on `uio[0..2]`, so adapting one for TTPE requires wiring to `uio[6]` (`D+`) and `uio[7]` (`D-`) and moving the `1.5 kΩ` pull-up to **`D-`**.
* **FPGA ecosystem passive USB Pmods:** Open-source FPGA soft-USB Pmods (e.g., OpenTitan USBdev Pmod, 1BitSquared/iCEBreaker passive USB breakouts) provide a USB receptacle, series resistors, and a switched `1.5 kΩ` pull-up on standard Pmod headers; jumpering to `uio[6..7]` with the pull-up on `D-` fits TTPE's pinout.

---

## 4. 10BASE-T Ethernet: Explicitly Out of Scope

The Competition FAQ lists **10 Mbit Ethernet** alongside low-speed USB as a stretch goal (*Design -> Which protocols, and how fast?*). TTPE declares 10BASE-T **out of scope** for three quantitative reasons (cycle-budget and bandwidth figures are architectural calculations; pad slew/latency margins are `[INFERENCE]` pending CMOS5L characterization):

1. **Line rate exceeds the ISA timing floor:** 10BASE-T uses 10 Mbit/s Manchester encoding = **20 Mbaud** (one transition every **50 ns = 2.5 cycles** at `50 MHz`, or `3.0` cycles at `60 MHz`). ISA v0.2's minimum back-to-back instruction spacing is **3 cycles** (`D >= 2`, `60 ns` at `50 MHz`) and every taken branch costs **6 cycles** (`3 + 3` penalty, `docs/ISA.md` §3). Software bit-banging cannot perform Manchester edge recovery, decoding, or transmission at 20 Mbaud; adding a fixed-function Ethernet SERDES/MAC would contradict the competition's programmable-engine goal.
2. **Frame size and host bandwidth mismatch:** A minimum Ethernet frame (64-byte frame + 8-byte preamble/SFD = 72 bytes) exceeds TTPE's 8-byte TX/RX FIFOs (`src/ttpe_fifo.sv`) by 9×. Meanwhile, the host SPI link tops out at `3.125 MHz` (~`0.39 MB/s` payload ceiling at `50 MHz`, `docs/HOST_SPI.md:L62`), which is **3.2× slower** than 10 Mbit/s wire rate (`1.25 MB/s`), preventing continuous host streaming without adding a dedicated packet SRAM buffer and hardware CRC32 engine.
3. **Analog / differential electrical interface:** 10BASE-T requires ±2.5 V differential drive over 100 Ω twisted pair through an isolation pulse transformer (magnetics) or an external MII/RMII PHY. Tiny Tapeout provides only 3.3 V single-ended digital pads.
