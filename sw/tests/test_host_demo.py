"""
Unit tests for sw/host_demo_rp2350.py on CPython, against a small model of the host register file
written from docs/HOST_SPI.md.  This checks the driver's command bytes, SRAM word order and FIFO flow
control against the DOCUMENT; it does not touch the RTL or the board.

Run:  python3 -m unittest discover -s sw/tests -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import host_demo_rp2350 as H  # noqa: E402


class FakeChip:
    """Register file per docs/HOST_SPI.md: every payload byte is a separate access, MISO of the command
    byte is STATUS, writes answer 0x00, SRAM stream is little-endian words with SADDR auto-increment."""

    def __init__(self, tx_pull_every=0):
        self.sram = [0] * 512
        self.saddr = 0
        self.sbyte = 0
        self.tx, self.rx = [], []
        self.regs = {H.R_SCRATCH: 0, H.R_MODE: 0, H.R_PC_LO: 0, H.R_PC_HI: 0}
        self.ctrl_log, self.pushed, self.err = [], [], 0
        self.halted = True
        self.tx_pull_every = tx_pull_every      # >0: the engine pulls one TX byte every N FIFO_CNT reads
        self._cnt_reads = 0

    def status(self):
        return (self.halted | 0x04 | (0x08 if not self.tx else 0) | (0x10 if len(self.tx) == 8 else 0) |
                (0x20 if not self.rx else 0)) & 0xFF

    def xfer(self, tx):
        rx = bytearray(len(tx))
        rw, addr = tx[0] >> 7, tx[0] & 0x7F
        rx[0] = self.status()
        for k, b in enumerate(tx[1:], start=1):
            rx[k] = self.access(rw, addr, b)
        return bytes(rx)

    def access(self, rw, addr, b):
        if addr == H.R_ID and rw:
            return 0x54
        if addr == H.R_STATUS and rw:
            return self.status()
        if addr == H.R_CTRL and not rw:
            self.ctrl_log.append(b)
            if b & H.CTRL_HALT:
                self.halted = True
            if b & H.CTRL_RESUME:
                self.halted = False
            return 0
        if addr in (H.R_PC_LO, H.R_PC_HI, H.R_SCRATCH, H.R_MODE):
            if rw:
                return self.regs[addr]
            self.regs[addr] = b & (1 if addr in (H.R_PC_HI, H.R_MODE) else 0xFF)
            return 0
        if addr == H.R_FIFO_CNT and rw:
            self._cnt_reads += 1
            if self.tx_pull_every and self.tx and self._cnt_reads % self.tx_pull_every == 0:
                self.tx.pop(0)
            return (len(self.rx) << 4) | len(self.tx)
        if addr == H.R_ERR and rw:
            return self.err
        if addr == H.R_SADDR_LO and not rw:
            self.saddr = (self.saddr & 0x100) | b
            self.sbyte = 0
            return 0
        if addr == H.R_SADDR_HI and not rw:
            self.saddr = (self.saddr & 0xFF) | ((b & 1) << 8)
            self.sbyte = 0
            return 0
        if addr == H.R_TX_PUSH and not rw:
            if len(self.tx) < 8:
                self.tx.append(b)
                self.pushed.append(b)
            else:
                self.err |= 4
            return 0
        if addr == H.R_RX_POP and rw:
            return self.rx.pop(0) if self.rx else 0
        if addr == H.R_SRAM:
            w = self.saddr % 512
            if rw:
                v = (self.sram[w] >> (8 * self.sbyte)) & 0xFF
            else:
                self.sram[w] = (self.sram[w] & ~(0xFF << (8 * self.sbyte))) | (b << (8 * self.sbyte))
                v = 0
            self.sbyte += 1
            if self.sbyte == 4:
                self.sbyte, self.saddr = 0, (self.saddr + 1) % 512
            return v
        self.err |= 2
        return 0


class HostDriver(unittest.TestCase):
    def setUp(self):
        self.chip = FakeChip()
        self.host = H.TtpeHost(self.chip.xfer)

    def test_id_and_reset_state(self):
        self.host.reset_and_check()
        self.chip.halted = False
        with self.assertRaises(RuntimeError):
            self.host.reset_and_check()

    def test_wrong_id_is_reported(self):
        chip = FakeChip()
        orig = chip.access
        chip.access = lambda rw, a, b: 0x55 if (a == H.R_ID and rw) else orig(rw, a, b)
        with self.assertRaises(RuntimeError):
            H.TtpeHost(chip.xfer).check_id()

    def test_program_load_is_little_endian_and_verified(self):
        words = [0xB0000002, 0x84000002, 0x12345678, 0xFFFFFFFF] + list(range(100, 200))
        self.host.load(words)
        self.assertEqual(self.chip.sram[:len(words)], words)

    def test_load_detects_a_bad_readback(self):
        orig = self.chip.access

        def flaky(rw, a, b):
            v = orig(rw, a, b)
            return v ^ 1 if (rw and a == H.R_SRAM) else v
        self.chip.access = flaky
        with self.assertRaises(RuntimeError):
            self.host.load([1, 2, 3])

    def test_start_loads_pc_then_pulses_pc_load_and_resume(self):
        self.host.start(0x105)
        self.assertEqual((self.chip.regs[H.R_PC_LO], self.chip.regs[H.R_PC_HI]), (0x05, 1))
        self.assertEqual(self.chip.ctrl_log, [0x03])
        self.assertFalse(self.chip.halted)
        self.host.halt()
        self.assertTrue(self.chip.halted)

    def test_mode_register(self):
        self.host.set_usb(True)
        self.assertEqual(self.chip.regs[H.R_MODE], 1)
        self.host.set_usb(False)
        self.assertEqual(self.chip.regs[H.R_MODE], 0)

    def test_push_respects_the_8_byte_fifo(self):
        chip = FakeChip(tx_pull_every=2)
        host = H.TtpeHost(chip.xfer)
        data = bytes(range(1, 40))
        host.push(data)
        self.assertEqual(bytes(chip.pushed), data)
        self.assertEqual(chip.err & 4, 0, "never pushed into a full FIFO")

    def test_push_times_out_when_the_engine_never_pulls(self):
        with self.assertRaises(RuntimeError):
            self.host.push(bytes(20), timeout_ms=5)

    def test_drain(self):
        self.chip.rx = [0xDE, 0xAD, 0xBE]
        self.assertEqual(self.host.drain(expect=3), b"\xDE\xAD\xBE")
        self.assertEqual(self.host.drain(timeout_ms=0), b"")

    def test_command_bytes(self):
        seen = []
        orig = self.chip.xfer
        self.host.xfer = lambda tx: (seen.append(bytes(tx)), orig(tx))[1]
        self.host.read(H.R_ID)
        self.host.write(H.R_SCRATCH, 0xA5)
        self.assertEqual(seen[0][0], 0x80 | H.R_ID)
        self.assertEqual(seen[1], bytes([H.R_SCRATCH, 0xA5]))

    def test_demo_payloads(self):
        self.host.halt()
        H.demo_i2c_register_read.__doc__                    # documented limit n <= 127
        chip = FakeChip(tx_pull_every=1)
        host = H.TtpeHost(chip.xfer)
        chip.rx = [0x53, 0x5A, 0x61]
        out = H.demo_i2c_register_read(host, [0x00000000], 0x50, 0x05, 3)
        self.assertEqual(out, b"\x53\x5A\x61")
        self.assertEqual(bytes(chip.pushed), bytes([0xA0, 0x81, 0x05, 0xA1, 3]))
        chip = FakeChip(tx_pull_every=1)
        H.demo_usb_send(H.TtpeHost(chip.xfer), [0], [0xD2])
        self.assertEqual(bytes(chip.pushed), bytes([1, 0xD2]))


class PinSpiBitBang(unittest.TestCase):
    def test_frame_roundtrip_through_a_bit_level_slave(self):
        """Bit-level mode-0 slave: samples MOSI on SCK rising edges, shifts a response byte out MSB first."""
        class Slave:
            def __init__(self):
                self.sck = 0
                self.cs = 1
                self.mosi = 0
                self.bits = []
                self.rx = []
                self.out = 0

            def set_sck(self, v):
                if v and not self.sck and not self.cs:
                    self.bits.append(self.mosi)
                    if len(self.bits) % 8 == 0:
                        self.rx.append(int("".join(map(str, self.bits[-8:])), 2))
                if not v and self.sck:
                    pass
                self.sck = v

            def set_cs(self, v):
                self.cs = v

            def set_mosi(self, v):
                self.mosi = v

            def miso(self):
                k = len(self.bits)
                return (0xA5 >> (7 - (k % 8))) & 1 if not self.cs else 0

        s = Slave()
        spi = H.PinSpi(s.set_sck, s.set_mosi, s.set_cs, lambda: s.miso())
        rx = spi.xfer(bytes([0x80, 0x00, 0xFF]))
        self.assertEqual(s.rx, [0x80, 0x00, 0xFF])
        self.assertEqual(rx, bytes([0xA5, 0xA5, 0xA5]))


if __name__ == "__main__":
    unittest.main()
