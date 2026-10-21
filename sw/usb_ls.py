"""
usb_ls.py - low-speed USB (1.5 Mbit/s) waveform generator and cycle-exact reference model of the
proposed `ttpe_usb_rx` bit-slicer primitive (docs/USB_LS_RX.md).  Python standard library only.

Clock: 50 MHz (20 ns).  One low-speed bit is 100/3 = 33.33 cycles.

Everything here is a *model of a proposal*: there is no RTL for the primitive yet.

Line states (low speed): J (idle) = D+ 0, D- 1;  K = D+ 1, D- 0;  SE0 = both 0.
NRZI: 0 = line toggles, 1 = line stays.  Bytes are sent LSB first.  After six consecutive 1s a 0 is
stuffed.  SYNC = KJKJKJKK.  EOP = SE0 for more than a bit time, then J.
"""
import random

BIT_NUM, BIT_DEN = 100, 3            # 33.33 cycles per bit

# ---------------------------------------------------------------- protocol helpers
PID = dict(OUT=0x1, IN=0x9, SOF=0x5, SETUP=0xD, DATA0=0x3, DATA1=0xB, ACK=0x2, NAK=0xA, STALL=0xE)


def pid_byte(p):
    return ((~p & 0xF) << 4) | p


def crc5(value11):
    """USB CRC5 over the 11 bits (addr[6:0], endp[3:0]), LSB first (reflected form, poly 0x05 -> 0x14).
    Returns the 5-bit field as it is packed into bits [7:3] of the second token byte."""
    crc = 0x1F
    for i in range(11):
        b = (value11 >> i) & 1
        crc = (crc >> 1) ^ 0x14 if (crc ^ b) & 1 else crc >> 1
    return (~crc) & 0x1F


def crc16(data):
    """USB CRC16 (reflected, poly 0x8005 -> 0xA001); sent low byte first."""
    crc = 0xFFFF
    for byte in data:
        for i in range(8):
            b = (byte >> i) & 1
            crc = (crc >> 1) ^ 0xA001 if (crc ^ b) & 1 else crc >> 1
    return (~crc) & 0xFFFF


def token(pid, addr, endp):
    v = (addr & 0x7F) | ((endp & 0xF) << 7)
    c = crc5(v)
    return [pid_byte(PID[pid]), v & 0xFF, ((v >> 8) & 0x7) | (c << 3)]


def data_packet(pid, payload):
    c = crc16(payload)
    return [pid_byte(PID[pid])] + list(payload) + [c & 0xFF, c >> 8]


# ---------------------------------------------------------------- waveform generator
def bits_of(byte_list):
    out = []
    for byte in byte_list:
        out += [(byte >> i) & 1 for i in range(8)]
    return out


def encode_levels(byte_list):
    """SYNC + bytes -> list of line levels (1 = K, 0 = J), one per bit cell, NRZI + bit stuffing.
    Returns (levels, stuffed_positions)."""
    levels, level, ones = [], 0, 0           # idle level is J = 0
    stuffed = []

    def put(bit):
        nonlocal level
        if bit == 0:
            level ^= 1
        levels.append(level)

    for bit in [0, 0, 0, 0, 0, 0, 0, 1]:      # SYNC
        put(bit)
    for bit in bits_of(byte_list):
        put(bit)
        if bit:
            ones += 1
            if ones == 6:
                stuffed.append(len(levels))
                put(0)
                ones = 0
        else:
            ones = 0
    return levels, stuffed


def waveform(packets, gap_cycles=400, eop_cycles=50, ppm=0, jitter=0, skew=0, seed=1, lead=300):
    """Per-cycle (dp, dm) lists.  Host bit period = 33.33 cycles * (1 + ppm*1e-6).
    jitter: each bit boundary moves by a random integer in [-jitter, jitter] cycles.
    skew:   D- edges lag D+ edges by a random integer in [0, skew] cycles (transient SE0/SE1).
    eop_cycles: SE0 duration (about 1.25 .. 1.5 bit times is 42 .. 50 cycles)."""
    rnd = random.Random(seed)
    dp, dm = [0] * lead, [1] * lead           # J idle
    for pkt in packets:
        levels, _ = encode_levels(pkt)
        t0 = len(dp)
        period = BIT_NUM / BIT_DEN * (1 + ppm * 1e-6)
        bounds = [t0 + round(i * period) + (rnd.randint(-jitter, jitter) if i else 0)
                  for i in range(len(levels) + 1)]
        end = bounds[-1]
        dpw = [0] * (end - t0)
        dmw = [1] * (end - t0)
        prev = 0
        for i, lv in enumerate(levels):
            s, e = bounds[i] - t0, bounds[i + 1] - t0
            for k in range(s, e):
                dpw[k] = lv
            sk = rnd.randint(0, skew) if (lv != prev and skew) else 0
            for k in range(s, e):
                dmw[k] = (1 - lv) if k - s >= sk else (1 - prev)
            prev = lv
        dp += dpw
        dm += dmw
        dp += [0] * eop_cycles
        dm += [0] * eop_cycles               # SE0
        dp += [0] * gap_cycles
        dm += [1] * gap_cycles               # J
    return dp, dm


# ---------------------------------------------------------------- transmitter (usb_tx.asm) reference
TX_EXT_BITS = (2, 5)      # data bits whose cell is 34 cycles wide in usb_tx.asm (EXT2/EXT5 = 1)


def tx_cells(byte_list, ext_bits=TX_EXT_BITS):
    """Cells sent by usb_tx.asm: list of (level, width_cycles), level 1 = K.  SYNC = 8 cells of 33;
    a data bit k (LSB first) is 33 wide (+1 if k in ext_bits); a stuffed 0 is 33 wide and follows the
    sixth consecutive 1 immediately (also at the very end of the packet).  Bit stuffing starts counting
    after SYNC (same convention as encode_levels)."""
    cells, level, ones = [], 0, 0

    def put(bit, width):
        nonlocal level
        if bit == 0:
            level ^= 1
        cells.append((level, width))

    for bit in (0, 0, 0, 0, 0, 0, 0, 1):
        put(bit, 33)
    for byte in byte_list:
        for k in range(8):
            bit = (byte >> k) & 1
            put(bit, 33 + (1 if k in ext_bits else 0))
            if bit:
                ones += 1
                if ones == 6:
                    put(0, 33)
                    ones = 0
            else:
                ones = 0
    return cells


def tx_expected(byte_list, ext_bits=TX_EXT_BITS, se0=66, j=33):
    """Per-cycle (dp, dm) the transmitter must produce, starting at the first SYNC edge (first K):
    cells, SE0 for `se0` cycles, J for `j` cycles.  Afterwards the bus is released = J."""
    dp, dm = [], []
    for lv, w in tx_cells(byte_list, ext_bits):
        dp += [lv] * w
        dm += [1 - lv] * w
    dp += [0] * (se0 + j)
    dm += [0] * se0 + [1] * j
    return dp, dm


# ---------------------------------------------------------------- primitive reference model
CELL = (33, 33, 34)       # cycles per cell, three-cell pattern = 100 cycles = 3 bits
SAMPLE_AT = 16            # sample when the cycle counter of the cell reaches this value
IDLE_CELLS = 8            # LOCKOUT ends after this many cells without a D+ edge
SYNC_LEVELS = (1, 0, 1, 0, 1, 0, 1, 1)


class UsbRx:
    """Stepwise form of the primitive; mirrors src/ttpe_usb_rx.sv line by line.

    step(en, dp, dp_prev, dm) is called once per cycle with the synchronised pad values of THAT
    cycle (dp = s2[6], dp_prev = s3[6], dm = s2[7]). It returns (tick, data, stop, err): the values
    that the synchroniser flops s2[12], s2[13], s2[14], s2[15] take at the end of the cycle, i.e. the
    values seen by the program in the NEXT cycle. en = 0 holds the state machine in IDLE with
    data/stop/err cleared (the returned value is then ignored by the synchroniser)."""

    def __init__(self):
        self.st = "IDLE"
        self.cnt = self.idx = self.ncell = self.ones = 0
        self.smp = False
        self.prev = 0
        self.d_q = self.s_q = self.r_q = 0
        self.quiet = 0

    def step(self, en, dp, dp_prev, dm):
        if not en:
            self.st = "IDLE"
            self.d_q = self.s_q = self.r_q = 0
            return 0, 0, 0, 0
        edge = dp != dp_prev
        t_next = 0
        if self.st == "IDLE":
            if edge and dp == 1:
                self.st, self.cnt, self.idx, self.ncell, self.ones, self.smp = "SYNC", 0, 0, 0, 0, False
                self.s_q = self.r_q = 0
        elif self.st == "LOCKOUT":
            self.quiet = 0 if edge else self.quiet + 1
            if self.quiet >= IDLE_CELLS * 33:
                self.st = "IDLE"
        else:
            if edge:
                if self.smp:                  # previous cell already sampled: this edge starts the next cell
                    self.idx, self.smp = (self.idx + 1) % 3, False
                self.cnt = 0
            else:
                self.cnt += 1
                if self.cnt == CELL[self.idx]:    # free-running rollover
                    self.cnt, self.idx, self.smp = 0, (self.idx + 1) % 3, False
            if self.cnt == SAMPLE_AT and not edge:
                self.smp = True
                lv, se0 = dp, (dp == 0 and dm == 0)
                if self.st == "SYNC":
                    if lv != SYNC_LEVELS[self.ncell] or se0:
                        self.st, self.quiet = "LOCKOUT", 0
                    else:
                        self.ncell += 1
                        if self.ncell == 8:
                            self.st, self.prev, self.ones = "RUN", lv, 0
                else:
                    if se0:
                        self.s_q, t_next, self.st = 1, 1, "IDLE"
                    else:
                        bit = 1 if lv == self.prev else 0
                        self.prev = lv
                        if self.ones == 6:                 # this cell must be the stuffed 0
                            self.ones = 0
                            if bit:                        # stuff violation
                                self.s_q = self.r_q = t_next = 1
                                self.st, self.quiet = "LOCKOUT", 0
                        else:
                            self.ones = self.ones + 1 if bit else 0
                            self.d_q, t_next = bit, 1
        return t_next, self.d_q, self.s_q, self.r_q


def usb_rx_model(dp_s, dm_s):
    """dp_s, dm_s: *synchronised* D+/D- (pad delayed by the 2-flop synchroniser), one entry per cycle.
    Returns four per-cycle lists (tick, data, stop, err): the value visible DURING cycle k.
      tick : 1-cycle pulse, one per received (unstuffed) data bit and one at packet end
      data : last decoded bit (held until the next tick)
      stop : set at the packet-end tick (EOP or error), held until the next SYNC is detected
      err  : set with stop when the packet ended in a bit-stuff violation, held likewise
    A stuffed bit produces no tick.  (Whole-array wrapper around UsbRx, enable always on.)"""
    n = len(dp_s)
    tick, data, stop, err = [0] * (n + 1), [0] * (n + 1), [0] * (n + 1), [0] * (n + 1)
    u = UsbRx()
    for k in range(n):
        prev = dp_s[k - 1] if k > 0 else dp_s[0]      # no edge in cycle 0
        tick[k + 1], data[k + 1], stop[k + 1], err[k + 1] = u.step(True, dp_s[k], prev, dm_s[k])
    return tick[:n], data[:n], stop[:n], err[:n]


def sync2(x, fill):
    """2-flop synchroniser: value during cycle k = pad value during cycle k-2."""
    return [fill, fill] + list(x[:-2])


def packets_from_ticks(tick, data, stop, err):
    """Independent decode of the primitive's outputs: list of (bit_list, err_flag) per packet."""
    bits, out = [], []
    for k in range(len(tick)):
        if tick[k]:
            if stop[k]:
                out.append((bits, bool(err[k])))
                bits = []
            else:
                bits.append(data[k])
    return out


# ---------------------------------------------------------------- software-only fixed-phase capture
def gen_raw_capture_asm(cells=128, off=16, pin="uio6"):
    """Assembly for a straight-line raw D+ capture that needs NO new hardware.  WAIT for the first SYNC edge,
    then one SAMPLE per bit cell at commit w + off + round(i*100/3) (gaps 33/34/33 cycles), HPUSH after every
    8 samples.  NRZI, unstuffing, EOP and resync are NOT done on the engine (the host does NRZI/unstuff)."""
    lines = ["WAIT rise %s @2" % pin]
    t = 0
    for i in range(cells):
        c = off + round(i * 100 / 3)
        lines.append("SAMPLE %s lsb @%d" % (pin, c - t - 1))
        t = c
        if i % 8 == 7:
            lines.append("HPUSH @2")
            t += 3
    lines.append("HALT")
    return "\n".join(lines)


def decode_raw_capture(raw_bytes):
    """Host-side decode of the raw capture: check SYNC, NRZI-decode, unstuff.  Returns bit list or None."""
    lv = []
    for b in raw_bytes:
        lv += [(b >> i) & 1 for i in range(8)]
    if tuple(lv[:8]) != SYNC_LEVELS:
        return None
    prev, ones, bits = lv[7], 0, []
    for l in lv[8:]:
        bit = 1 if l == prev else 0
        prev = l
        if ones == 6:
            ones = 0
            continue
        ones = ones + 1 if bit else 0
        bits.append(bit)
    return bits
