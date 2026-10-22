"""
host_demo_rp2350.py - drive the TTPE chip from the RP2350 (MicroPython) of the Tiny Tapeout demo board.

What is verified and what is not
  * `TtpeHost` (command bytes, register numbers, SRAM load order, FIFO flow control) follows
    docs/HOST_SPI.md and is unit-tested on CPython against a small model of the register file
    (sw/tests/test_host_demo.py).  It has NOT been run against the chip or the board.
  * `SdkSpi` is the thin board layer.  The Tiny Tapeout SDK calls in it (DemoBoard, shuttle enable,
    input/output pin objects, clock) are written from memory of the SDK and are UNTESTED: check
    them against the SDK version on your board.  `PinSpi` is the same bit-banged SPI on plain
    machine.Pin objects, for custom wiring.

Chip pins used by the host link (docs/HOST_SPI.md):  ui_in[0] = SCK, ui_in[1] = MOSI,
ui_in[2] = CS_N, uo_out[0] = MISO.  SPI mode 0, MSB first.  A bit-banged link from MicroPython is
far slower than the 3.125 MHz limit, which is fine.  When the project is deselected on the shuttle
mux the chip loses power: after every (re)selection reset it and load the program again.

Typical use on the board (copy build/<name>.py from `make -C sw images` next to this file):

    from host_demo_rp2350 import *
    host = TtpeHost(SdkSpi().xfer)
    host.reset_and_check()
    demo_i2c_register_read(host, 0x50, 0x05, 3)
"""
import struct
import time

R_ID, R_STATUS, R_CTRL, R_PC_LO, R_PC_HI, R_FIFO_CNT, R_RX_OVR, R_SCRATCH = 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
R_SADDR_LO, R_SADDR_HI, R_ERR, R_MODE = 0x08, 0x09, 0x0A, 0x0B
R_TX_PUSH, R_RX_POP, R_SRAM = 0x10, 0x11, 0x12

CTRL_RESUME, CTRL_PC_LOAD, CTRL_TX_FLUSH, CTRL_RX_FLUSH, CTRL_HALT, CTRL_CORE_RESET = 0x01, 0x02, 0x04, 0x08, 0x10, 0x20
TX_DEPTH = 8
CHIP_ID = 0x54


def _sleep_ms(ms):
    try:
        time.sleep_ms(ms)           # MicroPython
    except AttributeError:
        time.sleep(ms / 1000)       # CPython (tests)


class TtpeHost:
    """Register-level driver. `xfer(tx_bytes) -> rx_bytes` clocks one SPI frame (CS_N low for the
    whole call) and returns as many bytes as were sent; rx[0] is the STATUS byte."""

    def __init__(self, xfer):
        self.xfer = xfer
        self.status = 0

    # --- raw register access (command byte = {rw, addr[6:0]}, rw = 1 reads)
    def write(self, addr, data):
        data = bytes(data) if not isinstance(data, int) else bytes([data])
        rx = self.xfer(bytes([addr & 0x7F]) + data)
        self.status = rx[0]

    def read(self, addr, n=1):
        rx = self.xfer(bytes([0x80 | (addr & 0x7F)]) + bytes(n))     # no dummy byte, data = rx[1:]
        self.status = rx[0]
        return bytes(rx[1:])

    # --- link and run control
    def check_id(self):
        got = self.read(R_ID)[0]
        if got != CHIP_ID:
            raise RuntimeError("TTPE ID 0x%02X, expected 0x%02X" % (got, CHIP_ID))

    def reset_and_check(self, reset_pin=None):
        """Optionally pulse the board's rst_n (reset_pin(0) then (1)), then check the ID and halt state."""
        if reset_pin is not None:
            reset_pin(0)
            _sleep_ms(2)
            reset_pin(1)
            _sleep_ms(2)
        self.check_id()
        self.read(R_STATUS)
        if not self.status & 0x01:
            raise RuntimeError("core is running after reset (STATUS=0x%02X)" % self.status)

    def load(self, words, verify=True):
        """Write 32-bit instruction words from SRAM address 0 (core must be halted)."""
        self.write(R_SADDR_LO, 0)
        self.write(R_SADDR_HI, 0)
        payload = b"".join(struct.pack("<I", w & 0xFFFFFFFF) for w in words)   # LSB byte first
        self.write(R_SRAM, payload)
        if verify:
            self.write(R_SADDR_LO, 0)
            self.write(R_SADDR_HI, 0)
            back = self.read(R_SRAM, len(payload))
            if back != payload:
                raise RuntimeError("SRAM readback mismatch")
        if self.read(R_ERR)[0]:
            raise RuntimeError("ERR flags set: 0x%02X" % self.read(R_ERR)[0])

    def set_usb(self, on):
        """MODE[0]: connect the low-speed USB bit slicer to ui_in[7:4] (change only while the bus is idle)."""
        self.write(R_MODE, 1 if on else 0)

    def start(self, pc=0):
        self.write(R_PC_LO, pc & 0xFF)
        self.write(R_PC_HI, (pc >> 8) & 1)
        self.write(R_CTRL, CTRL_PC_LOAD | CTRL_RESUME)

    def halt(self):
        self.write(R_CTRL, CTRL_HALT)

    # --- FIFOs
    def counts(self):
        v = self.read(R_FIFO_CNT)[0]
        return v >> 4, v & 0x0F          # (rx_count, tx_count)

    def push(self, data, timeout_ms=2000):
        """Push bytes into the TX FIFO (8 deep), waiting for the engine to pull when it is full."""
        data = bytes(data)
        waited = 0
        while data:
            space = TX_DEPTH - self.counts()[1]
            if space > 0:
                self.write(R_TX_PUSH, data[:space])
                data = data[space:]
            else:
                if waited >= timeout_ms:
                    raise RuntimeError("TX FIFO stayed full")
                _sleep_ms(1)
                waited += 1

    def drain(self, expect=None, timeout_ms=2000):
        """Pop RX FIFO bytes. Returns after `expect` bytes (or after the FIFO stays empty for timeout_ms)."""
        out = b""
        waited = 0
        while expect is None or len(out) < expect:
            n = self.counts()[0]
            if n:
                out += self.read(R_RX_POP, n)
                waited = 0
            else:
                if waited >= timeout_ms:
                    break
                _sleep_ms(1)
                waited += 1
        return out

    def overruns(self):
        return self.read(R_RX_OVR)[0]


# ----------------------------------------------------------------------------- board layers
class PinSpi:
    """Bit-banged SPI mode 0 on four pin callables/objects: set_sck(v), set_mosi(v), set_cs(v), get_miso()."""

    def __init__(self, set_sck, set_mosi, set_cs, get_miso):
        self.sck, self.mosi, self.cs, self.miso = set_sck, set_mosi, set_cs, get_miso
        self.sck(0)
        self.cs(1)

    def xfer(self, tx):
        rx = bytearray(len(tx))
        self.cs(0)
        for i, byte in enumerate(tx):
            v = 0
            for b in range(7, -1, -1):
                self.mosi((byte >> b) & 1)
                v = (v << 1) | (1 if self.miso() else 0)   # MISO changes only after falling edges: stable here
                self.sck(1)                       # the chip samples MOSI on this rising edge
                self.sck(0)
            rx[i] = v
        self.cs(1)
        return bytes(rx)


class SdkSpi(PinSpi):
    """Tiny Tapeout demo-board layer (UNTESTED, check against your SDK version): enables the project and
    maps ui_in[0..2] / uo_out[0] to the SPI pins. 50 MHz is the design target; set the clock with the SDK."""

    def __init__(self, project="tt_um_ttpe", clock_hz=None):
        from ttboard.demoboard import DemoBoard      # only exists on the board
        tt = DemoBoard.get()
        getattr(tt.shuttle, project).enable()
        if clock_hz:
            tt.clock_project_PWM(clock_hz)

        def setter(pin):
            return lambda v: pin(1 if v else 0)

        self.tt = tt
        self.reset_pin = lambda v: tt.reset_project(not v) if hasattr(tt, "reset_project") else None
        super().__init__(setter(tt.in0), setter(tt.in1), setter(tt.in2), lambda: tt.out0())


# ----------------------------------------------------------------------------- demos
def run_program(host, words, pc=0):
    host.halt()
    host.load(words)
    host.start(pc)


def demo_uart_tx(host, words, text=b"Hi"):
    """words = uart_tx image (build/uart_tx.py PROGRAM). TXD = uio[1], 115200 baud 8N1."""
    run_program(host, words)
    host.push(text)


def demo_i2c_register_read(host, words, addr7, reg, n):
    """words = i2c_master image. S A+W reg Sr A+R n data P. SCL = uio[2], SDA = uio[3], needs 4.7 kOhm pull-ups.
    n must be 1..127 (n[7] is the repeated-START flag)."""
    run_program(host, words)
    host.push([(addr7 << 1) | 0, 0x81, reg, (addr7 << 1) | 1, n])
    return host.drain(expect=n)


def demo_usb_send(host, words, packet):
    """words = usb_tx image. packet = finished bytes: PID, payload, CRC (e.g. ACK = [0xD2]).
    D+ = uio[6], D- = uio[7]; needs the external pull-up/series resistors (docs/info.md)."""
    run_program(host, words)
    host.push([len(packet)] + list(packet))


def demo_usb_receive(host, words, seconds=1.0):
    """words = usb_rx image. Enables MODE[0], returns the bytes received (PID first) -- the RX FIFO is only
    8 deep, so a long packet overruns unless this loop keeps up (host.overruns() tells)."""
    host.halt()
    host.set_usb(True)
    host.load(words)
    host.start(0)
    out = b""
    for _ in range(int(seconds * 200)):
        out += host.drain(timeout_ms=0)
        _sleep_ms(5)
    return out
