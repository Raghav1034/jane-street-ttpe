# Clean-Room Instruction-Set Simulator (ISS) Task Specification

## Objective

Implement a standalone, cycle-accurate Python 3 Instruction-Set Simulator (ISS) for the Time-Triggered Protocol Engine (TTPE) **strictly from the attached ISA specification** (using only the Python 3 standard library).

Do **not** assume or invent features outside the ISA specification. Every cycle number, bitfield mask, timing penalty, and register update must follow the written specification exactly.

---

## Scope

* **In scope:** All 14 defined ISA v0.2 opcodes (`NOP`, `PINS`, `WAIT`, `JMP`, `JHI`, `JLO`, `SHOUT`, `SAMPLE`, `REG` with all 11 legal sub-ops, `DJNZ`, `HPUSH`, `HPULL`, `HALT`, `JSB`), illegal opcodes (`0xE`, `0xF`) and illegal `REG` sub-ops (`7`, `12..15`), the 9-bit PC fetch pipeline (`FETCH_LATENCY = 3`), the 16-bit delta timer, the 2-FF input synchroniser + 1-cycle edge history, and the 8-deep TX and RX FIFOs (including 8-bit saturating `rx_overrun`).
* **Out of scope:** Host SPI serial framing, external SRAM macro bus signals, and the optional low-speed USB bit slicer (`MODE[0]` stays `0`).

---

## Required Python Interface

Provide a single Python 3 file implementing the function below:

```python
def simulate(
    words,
    pad_fn,
    host_tx=None,
    max_cycles=10000,
    start_pc=0,
    start_halted=False,
    host_ops=None,
):
    ...
```

### Arguments

1. **`words`** (`Sequence[int]`):
   * Up to 512 32-bit unsigned instruction words (`0x00000000..0xFFFFFFFF`).
   * If shorter than 512 words, pad remaining entries up to 512 with `0xC0000000` (`HALT @0`).
2. **`pad_fn(cycle, uio_oe, uio_out, uo_out) -> (uio_in, ui_in)`**:
   * Called once per simulated cycle `k = 0, 1, 2, ...` with the start-of-cycle driven pin register values (`uio_oe`, `uio_out`, `uo_out`, each an 8-bit unsigned int `0..255`).
   * Returns `(uio_in, ui_in)`, each an 8-bit unsigned int `0..255`, representing the external pad levels applied during cycle `k`.
3. **`host_tx`** (`Sequence[int | tuple[int, int]] | None`, default `None`):
   * Optional pre-scheduled host-to-engine TX FIFO bytes: each element is either a plain byte `b` (equivalent to `(0, b)`) or a tuple `(avail_cycle, byte)`.
   * Entries are consumed in FIFO order; a tuple `(avail_cycle, byte)` is available for `HPULL` on any cycle `k >= avail_cycle`.
   * May be left `None` when bytes are instead pushed via `host_ops`.
4. **`max_cycles`** (`int`, default `10000`):
   * Run cycles `k = 0, 1, ..., max_cycles - 1`.
   * Stop early after completing a cycle in which `halted` becomes `True`, **unless** `host_ops` has a pending entry at a later cycle `> k` (in which case keep stepping while halted until `max_cycles` or no future `host_ops` remain).
5. **`start_pc`** (`int`, default `0`):
   * Initial 9-bit fetch PC (`start_pc & 0x1FF`).
6. **`start_halted`** (`bool`, default `False`):
   * `False`: core starts running at cycle `0` (`core_run = 1`), so the first instruction is accepted at cycle `3`.
   * `True`: core starts halted at cycle `0` (`halted = True`) and waits for `host_ops[k]["resume"]`.
7. **`host_ops`** (`dict[int, dict] | None`, default `None`):
   * Optional per-cycle host actions evaluated at the rising edge ending cycle `k` (after any cycle-`k` instruction commit):
     * `"tx_push": int` — push `byte & 0xFF` into the 8-deep TX FIFO if `tx_count < 8` and not `"tx_flush"`; visible to `HPULL` starting at cycle `k + 1`.
     * `"tx_flush": bool` — clear TX FIFO at the edge ending cycle `k` (overrides `"tx_push"`; a same-cycle `HPULL` commit still reads the pre-flush head byte).
     * `"rx_pop": bool` — pop head byte from RX FIFO at the edge ending cycle `k` if `rx_count > 0` and not `"rx_flush"` (does **not** prevent a same-cycle `HPUSH` from dropping and incrementing `rx_overrun` when start-of-cycle `rx_count == 8`).
     * `"rx_flush": bool` — clear RX FIFO and reset `rx_overrun = 0` at the edge ending cycle `k` (overrides same-cycle `HPUSH`).
     * `"pc_load": int` — honoured only while start-of-cycle `halted` is `True`; loads `fetch_pc = val & 0x1FF`.
     * `"resume": bool` — honoured only while start-of-cycle `halted` is `True`; clears `halted = False` and `illegal = False` at the edge ending cycle `k`, so the first instruction is accepted at cycle `k + 4`.

---

## Cycle Conventions

* Time is measured in integer cycles `k = 0, 1, 2, ...`.
* At the start of cycle `k`, all registers (`sr`, `c0`, `c1`, `uio_oe`, `uio_out`, `uo_out`, `halted`, `illegal`, `rx_overrun`, FIFO contents, and synchroniser flops `s1`, `s2`, `s3`) hold their start-of-cycle values.
* An instruction that commits in cycle `k` reads start-of-cycle registers and `s2`/`s3`, and updates registers at the rising clock edge ending cycle `k` (new values are visible at the start of cycle `k + 1`).
* The 2-FF input synchroniser + 1-cycle edge history starts at `s1 = s2 = s3 = 0` at cycle `0` and updates at the end of every cycle `k`:
  ```text
  raw_pads = (uio_in & 0xFF) | ((ui_in & 0xFF) << 8)
  s3_next  = s2
  s2_next  = s1
  s1_next  = raw_pads
  ```
  (Note: if `start_halted=False`, both free-running and halted-frozen synchronisers are identical until the first `HALT`; if `start_halted=True`, free-running every cycle matches the RTL specification.)

---

## Return Value Format

`simulate(...)` must return a `dict` with exactly these five keys:

```python
{
    "accepts":    [(cycle, pc), ...],
    "commits":    [(cycle, pc, word), ...],
    "pin_events": [(effective_cycle, uio_oe, uio_out, uo_out), ...],
    "rx":         [(commit_cycle, byte), ...],
    "final": {
        "cycles":     int,   # total cycles stepped (last cycle index + 1)
        "sr":         int,   # 0..255
        "c0":         int,   # 0..255
        "c1":         int,   # 0..255
        "uio_oe":     int,   # 0..255
        "uio_out":    int,   # 0..255
        "uo_out":     int,   # 0..255
        "halted":     bool,
        "illegal":    bool,
        "rx_overrun": int,   # 0..255 (saturating)
        "tx_count":   int,   # remaining unpopped bytes in TX FIFO (0..8)
        "rx_count":   int,   # remaining unpopped bytes in RX FIFO (0..8)
    },
}
```

* **`pin_events`** starts with `[(0, 0, 0, 0)]` and appends `(k + 1, uio_oe_next, uio_out_next, uo_out_next)` whenever a commit at cycle `k` changes `(uio_oe, uio_out, uo_out)`.
* **`rx`** records `(k, sr)` for every `HPUSH` committing at cycle `k` that actually enters the RX FIFO (i.e., start-of-cycle `rx_count < 8` and no same-cycle `rx_flush`).

---

## Worked Example Traces

### Example 1 — Basic Delta Timing and Pin Effective Cycle
* **Program:**
  * `0x000`: `0x11010005` (`PINS high uio0 @5`)
  * `0x001`: `0xC0000002` (`HALT @2`)
* **Inputs:** `pad_fn = lambda k, oe, out, uo: (0xFF, 0xFF)`, `start_halted=False`.
* **Expected output:**
  * `accepts`: `[(3, 0), (9, 1)]`
  * `commits`: `[(9, 0, 0x11010005), (12, 1, 0xC0000002)]`
  * `pin_events`: `[(0, 0, 0, 0), (10, 1, 1, 0)]`
  * `final`: `cycles=13, uio_oe=1, uio_out=1, uo_out=0, halted=True, illegal=False`

### Example 2 — Taken Branch (3-Cycle Penalty) and Fall-Through
* **Program:**
  * `0x000`: `0x81020002` (`LDI c0, 2 @2`)
  * `0x001`: `0x00000002` (`NOP @2`)
  * `0x002`: `0x90010002` (`DJNZ c0, 0x001 @2`)
  * `0x003`: `0xC0000000` (`HALT @0`)
* **Inputs:** `pad_fn = lambda k, oe, out, uo: (0xFF, 0xFF)`, `start_halted=False`.
* **Expected output:**
  * `accepts`: `[(3, 0), (6, 1), (9, 2), (15, 1), (18, 2), (21, 3)]`
  * `commits`: `[(6, 0, 0x81020002), (9, 1, 0x00000002), (12, 2, 0x90010002), (18, 1, 0x00000002), (21, 2, 0x90010002), (22, 3, 0xC0000000)]`
  * `final`: `cycles=23, c0=0, halted=True, illegal=False`

### Example 3 — `WAIT rise @0`, 2-FF Input Synchroniser, `SAMPLE`, and `HPUSH`
* **Program:**
  * `0x000`: `0x2B030000` (`WAIT rise ui3 @0`)
  * `0x001`: `0x7B000002` (`SAMPLE ui3 lsb @2`)
  * `0x002`: `0xA0000002` (`HPUSH @2`)
  * `0x003`: `0xC0000000` (`HALT @0`)
* **Inputs:** `pad_fn = lambda k, oe, out, uo: (0xFF, 0xF7 if k < 7 else 0xFF)` (`ui3` rises `0 -> 1` at cycle `7`), `start_halted=False`.
* **Expected output:**
  * `accepts`: `[(3, 0), (9, 1), (12, 2), (15, 3)]`
  * `commits`: `[(9, 0, 0x2B030000), (12, 1, 0x7B000002), (15, 2, 0xA0000002), (16, 3, 0xC0000000)]`
  * `rx`: `[(15, 0x80)]`
  * `final`: `cycles=17, sr=0x80, rx_count=1, rx_overrun=0, halted=True, illegal=False`
