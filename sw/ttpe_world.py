#!/usr/bin/env python3
"""
ttpe_world.py - external-world models for the TTPE ISS (and, later, for cocotb).

A World answers `pads(cycle, uio_oe, uio_out, uo_out) -> (uio_in, ui_in)`.
All lines are modelled as a wired-AND bus with pull-ups: a pin reads 0 if the TTPE
drives it low (oe & ~out) or if an attached device pulls it low; otherwise 1.
(Push-pull high is therefore indistinguishable from released; contention is not modelled.)

Devices see the pad values of the PREVIOUS cycle (one cycle of reaction delay), so
there are no zero-delay loops.  Everything is integer cycle math.
"""


class BusWorld:
    def __init__(self, devices=(), ui=0xFF):
        self.devices = list(devices)
        self.ui = ui
        self.last_pad = 0xFF
        self.pad_log = []            # pad_log[k] = uio pad value during cycle k

    def pads(self, k, uio_oe, uio_out, uo_out):
        low = uio_oe & ~uio_out & 0xFF
        for d in self.devices:
            low |= d.step(k, self.last_pad)
        pad = ~low & 0xFF
        self.last_pad = pad
        self.pad_log.append(pad)
        return pad, self.ui

    # --- waveform helpers
    def levels(self, pin):
        return [(p >> pin) & 1 for p in self.pad_log]

    def edges(self, pin):
        """[(cycle, new_level), ...] for every transition of a pad."""
        lv = self.levels(pin)
        return [(k, lv[k]) for k in range(1, len(lv)) if lv[k] != lv[k - 1]]


# -------------------------------------------------------------------- UART
class UartSource:
    """Drives 8N1 frames (LSB first) onto one pin. frames = [(start_cycle, byte)]."""

    def __init__(self, pin, bit_cycles, frames):
        self.pin, self.bit, self.frames = pin, bit_cycles, list(frames)

    def level(self, k):
        for s, b in self.frames:
            rel = k - s
            if 0 <= rel < 10 * self.bit:
                i = rel // self.bit
                if i == 0:
                    return 0
                if i == 9:
                    return 1
                return (b >> (i - 1)) & 1
        return 1

    def step(self, k, pad_prev):
        """Returns the mask of pins this device pulls LOW in cycle k."""
        return (1 << self.pin) if self.level(k) == 0 else 0


def decode_uart(levels, bit_cycles, start=0):
    """Decode 8N1 frames from a per-cycle level list.
    Returns [(start_cycle, byte, stop_ok)], sampling at the centre of every bit."""
    out = []
    k = max(start, 1)
    n = len(levels)
    while k < n:
        if levels[k - 1] == 1 and levels[k] == 0:
            s = k
            if s + 10 * bit_cycles > n:
                break
            byte = 0
            for i in range(8):
                byte |= levels[s + bit_cycles * (i + 1) + bit_cycles // 2] << i
            stop_ok = levels[s + bit_cycles * 9 + bit_cycles // 2] == 1
            out.append((s, byte, bool(stop_ok)))
            k = s + 9 * bit_cycles + bit_cycles // 2
        else:
            k += 1
    return out


# --------------------------------------------------------------------- SPI
class SpiSlave:
    """SPI slave, any mode, MSB first. Pins are uio indices. tx_bytes are returned on
    MISO (0xFF once exhausted). Received bytes are collected in .rx (per transfer)."""

    def __init__(self, cpol, cpha, tx_bytes=(), cs=0, mosi=1, miso=2, sck=3):
        self.cpol, self.cpha = cpol, cpha
        self.cs, self.mosi, self.miso, self.sck = cs, mosi, miso, sck
        self.tx = list(tx_bytes)
        self.tx_pos = 0
        self.rx = []
        self.transfers = []
        self.active = False
        self.prev_cs, self.prev_sck = 1, cpol
        self.miso_low = False
        self.rx_shift = 0
        self.rx_bits = 0
        self.tx_cur = 0xFF
        self.tx_i = 0
        self._loaded_real = False      # tx_cur came from tx_bytes (not the 0xFF filler)
        self.bits_since_load = 0       # bits clocked since tx_cur was loaded
        self.cs_cycles = []        # (fall_cycle, rise_cycle)
        self._cs_fall = None

    def _next_tx(self):
        self._loaded_real = self.tx_pos < len(self.tx)
        if self._loaded_real:
            self.tx_cur = self.tx[self.tx_pos]
            self.tx_pos += 1
        else:
            self.tx_cur = 0xFF
        self.tx_i = 0
        self.bits_since_load = 0

    def _present(self):
        bit = (self.tx_cur >> (7 - self.tx_i)) & 1
        self.miso_low = (bit == 0)

    def step(self, k, pad_prev):
        cs = (pad_prev >> self.cs) & 1
        sck = (pad_prev >> self.sck) & 1
        mosi = (pad_prev >> self.mosi) & 1

        if self.prev_cs == 1 and cs == 0:          # CS asserted
            self.active = True
            self._cs_fall = k
            self.rx_shift, self.rx_bits = 0, 0
            self.transfers.append([])
            self._next_tx()
            if self.cpha == 0:
                self._present()
        elif self.prev_cs == 0 and cs == 1:        # CS released
            self.active = False
            self.miso_low = False
            self.cs_cycles.append((self._cs_fall, k))
            # CPHA=0 preloads the next byte after the last trailing edge; if no bit of it was
            # ever clocked out, hand it back so the next transfer starts with it.
            if self.bits_since_load == 0 and self._loaded_real:
                self.tx_pos -= 1
                self._loaded_real = False

        lead = (self.prev_sck == self.cpol) and (sck != self.cpol)
        trail = (self.prev_sck != self.cpol) and (sck == self.cpol)

        if self.active:
            if self.cpha == 0:
                if lead:
                    self._sample(mosi)
                if trail:
                    self.tx_i += 1
                    if self.tx_i == 8:
                        self._next_tx()
                    self._present()
            else:
                if lead:
                    if self.tx_i == 8:
                        self._next_tx()
                    self._present()
                    self.tx_i += 1
                if trail:
                    self._sample(mosi)

        self.prev_cs, self.prev_sck = cs, sck
        return (1 << self.miso) if (self.active and self.miso_low) else 0

    def _sample(self, mosi):
        self.rx_shift = ((self.rx_shift << 1) | mosi) & 0xFF
        self.rx_bits += 1
        self.bits_since_load += 1
        if self.rx_bits == 8:
            self.rx.append(self.rx_shift)
            self.transfers[-1].append(self.rx_shift)
            self.rx_bits = 0


# --------------------------------------------------------------------- I2C
class I2cSlave:
    """7-bit I2C slave. Writes are collected in .written (list of bytes per transaction);
    reads return read_bytes cyclically. Optional clock stretch after the address ACK."""

    def __init__(self, addr, read_bytes=(0xFF,), stretch=0, scl=2, sda=3):
        self.addr, self.read_bytes, self.stretch = addr, list(read_bytes), stretch
        self.scl, self.sda = scl, sda
        self.state = "idle"
        self.bitcnt = 0
        self.shreg = 0
        self.rw = 0
        self.sda_low = False
        self.stretch_left = 0
        self.prev_scl, self.prev_sda = 1, 1
        self.master_ack = False
        self.rd_pos = 0
        self.cur = 0xFF
        self.written = []
        self.addr_seen = []        # (addr7, rw, matched)
        self.starts = 0
        self.stops = 0
        self.nacked_reads = 0

    def _load_read(self):
        self.cur = self.read_bytes[self.rd_pos % len(self.read_bytes)]
        self.rd_pos += 1

    def step(self, k, pad_prev):
        scl = (pad_prev >> self.scl) & 1
        sda = (pad_prev >> self.sda) & 1
        ps, pd = self.prev_scl, self.prev_sda

        if ps == 1 and scl == 1 and pd == 1 and sda == 0:       # START
            self.state, self.bitcnt, self.shreg = "addr", 0, 0
            self.sda_low = False
            self.starts += 1
            self.written.append([])
        elif ps == 1 and scl == 1 and pd == 0 and sda == 1:     # STOP
            self.state = "idle"
            self.sda_low = False
            self.stops += 1
        else:
            rise = ps == 0 and scl == 1
            fall = ps == 1 and scl == 0
            st = self.state
            if rise:
                if st in ("addr", "wdata"):
                    self.shreg = ((self.shreg << 1) | sda) & 0xFF
                    self.bitcnt += 1
                elif st == "rdata":
                    self.bitcnt += 1
                elif st == "rack":
                    self.master_ack = (sda == 0)
            elif fall:
                if st == "addr" and self.bitcnt == 8:
                    a7, rw = self.shreg >> 1, self.shreg & 1
                    ok = (a7 == self.addr)
                    self.addr_seen.append((a7, rw, ok))
                    if ok:
                        self.rw = rw
                        self.sda_low = True
                        self.state = "addr_ack"
                    else:
                        self.state = "idle"
                elif st == "addr_ack":
                    self.sda_low = False
                    self.stretch_left = self.stretch
                    self.bitcnt = 0
                    if self.rw == 0:
                        self.state = "wdata"
                    else:
                        self.state = "rdata"
                        self._load_read()
                        self.sda_low = ((self.cur >> 7) & 1) == 0
                elif st == "wdata" and self.bitcnt == 8:
                    self.written[-1].append(self.shreg)
                    self.sda_low = True
                    self.state = "wack"
                elif st == "wack":
                    self.sda_low = False
                    self.state = "wdata"
                    self.bitcnt = 0
                elif st == "rdata":
                    if self.bitcnt < 8:
                        self.sda_low = ((self.cur >> (7 - self.bitcnt)) & 1) == 0
                    else:
                        self.sda_low = False
                        self.state = "rack"
                elif st == "rack":
                    if self.master_ack:
                        self._load_read()
                        self.bitcnt = 0
                        self.state = "rdata"
                        self.sda_low = ((self.cur >> 7) & 1) == 0
                    else:
                        self.nacked_reads += 1
                        self.state = "idle"
                        self.sda_low = False

        self.prev_scl, self.prev_sda = scl, sda
        low = 0
        if self.sda_low:
            low |= 1 << self.sda
        if self.stretch_left > 0:
            low |= 1 << self.scl
            self.stretch_left -= 1
        return low


class I2cRegSlave(I2cSlave):
    """I2C slave with a register file and pointer (typical sensor/EEPROM behaviour):
    the first byte written after the address sets the pointer, further written bytes are
    stored at ptr++, reads return mem[ptr++].  A repeated START does not reset the pointer,
    so [S A+W REG Sr A+R data P] reads from REG.  .events logs 'S', 'P' in bus order and
    .starts_while_active counts a START seen before a STOP (= repeated START)."""

    def __init__(self, addr, mem=None, **kw):
        super().__init__(addr, **kw)
        self.mem = list(mem) if mem is not None else list(range(256))
        self.ptr = 0
        self.events = []
        self.starts_while_active = 0
        self._bus_active = False

    def _load_read(self):
        self.cur = self.mem[self.ptr % len(self.mem)]
        self.ptr += 1
        self.rd_pos += 1

    def step(self, k, pad_prev):
        s0, p0, w0 = self.starts, self.stops, sum(len(x) for x in self.written)
        low = super().step(k, pad_prev)
        if self.starts != s0:
            self.events.append("Sr" if self._bus_active else "S")
            if self._bus_active:
                self.starts_while_active += 1
            self._bus_active = True
        if self.stops != p0:
            self.events.append("P")
            self._bus_active = False
        if sum(len(x) for x in self.written) != w0:
            b, n = self.written[-1][-1], len(self.written[-1])
            if n == 1:
                self.ptr = b
            else:
                self.mem[self.ptr % len(self.mem)] = b
                self.ptr += 1
        return low


class UsbTxBus(BusWorld):
    """Low-speed USB bus seen by a transmitter on D+ = uio6, D- = uio7: D- has the device pull-up
    (released reads 1), D+ has the host pull-down (released reads 0), so a released bus is J.
    pad_log / levels() / edges() give the per-cycle line levels."""

    def pads(self, k, uio_oe, uio_out, uo_out):
        pad, ui = super().pads(k, uio_oe, uio_out, uo_out)
        pad = (pad & ~0x40 & 0xFF) | (uio_oe & uio_out & 0x40)
        self.last_pad = pad
        self.pad_log[-1] = pad
        return pad, ui


class UsbHostDevBus(BusWorld):
    """Low-speed USB bus with a scripted host (per-cycle D+/D- lists, J beyond the lists) and the chip as
    the device on D+ = uio6, D- = uio7.  While the chip drives a line (uio_oe set) the chip's value wins,
    otherwise the host's value is on the wire (host idle = J = D+ 0, D- 1), so a released chip reads back
    exactly what the host or the pull resistors put there, including the chip's own transmission.
    uio0..5 behave as in BusWorld without devices.  pad_log / levels() give the per-cycle line levels."""

    def __init__(self, dp, dm, ui=0xFF):
        super().__init__([], ui)
        self.dp, self.dm = dp, dm

    def pads(self, k, uio_oe, uio_out, uo_out):
        hp, hm = (self.dp[k], self.dm[k]) if k < len(self.dp) else (0, 1)
        p = (uio_out >> 6) & 1 if uio_oe & 0x40 else hp
        m = (uio_out >> 7) & 1 if uio_oe & 0x80 else hm
        pad = (~(uio_oe & ~uio_out) & 0x3F) | (p << 6) | (m << 7)
        self.last_pad = pad
        self.pad_log.append(pad)
        return pad, self.ui
