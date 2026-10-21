; usb_tx.asm - low-speed USB (1.5 Mbit/s) packet transmitter, software only (no RTL support needed).
; D+ = uio6, D- = uio7, both driven push-pull while a packet is on the wire, released afterwards
; (a real bus needs the device's 1.5 kOhm pull-up on D-, the host's pull-downs and series resistors;
; none of that exists on the chip).  J idle = D+ low, D- high.  K = D+ high, D- low.
;
; Host protocol (TX FIFO bytes):   n, b0 ... b(n-1)        n = 1..255 (0 = 256)
;   b0 is the PID byte (already formed, e.g. ACK = 0xD2, NAK = 0x5A, SETUP = 0x2D); the program
;   adds SYNC, bit stuffing, NRZI and EOP.  CRC5/CRC16 must be appended by the HOST (there are only
;   SR, C0, C1 - a CRC16 does not fit).  Bytes go out LSB first.
;   The host must keep the TX FIFO ahead of the wire: one byte is consumed every ~267 cycles (about
;   5.3 us); a byte pull that finds the FIFO empty STALLS the line (corrupt packet).  Stream the
;   packet in one SPI burst; an 11-byte DATA packet needs the refill to start before the 8th byte.
;
; Wire sequence:  [J driven >= 1 cell]  SYNC (7 transitions, then one K cell)  data cells
;                 SE0 for 66 cycles (2 bit times)  J for 33 cycles (1 bit time)  released.
;
; Registers: SR = byte being sent (tested bit by bit with JSB sr[k], so no shifting is needed:
;            the bit loop is unrolled 8 times), C0 = ones countdown (reload 6; reaches 0 after the
;            sixth consecutive 1 -> stuffed 0), C1 = bytes left.
;
; Timing (ISA rules: sequential spacing = delta+1, taken branch target spacing = delta+4):
;   Every cell is a fixed number of cycles between consecutive bit-test JSB commits (J):
;       zero bit : JSB J, PINS toggle J+3 (edge, pad changes J+4), LDI J+30, next JSB J+33
;       one  bit : JSB J, DJNZ c0 J+27 (taken), next JSB J+33
;       sixth one: DJNZ falls through at J+27; stuffed-0 toggle at J+36, next JSB J+66
;   The edge is always 3 cycles after the cell's JSB commit, so edge-to-edge spacing is exactly the
;   sum of the cell widths.  Cell width = 33, except after bit 2 and bit 5 of each byte (34): the
;   pattern 33,33,34,33,33,34,33,33 = 266 cycles per byte (ideal 8 x 33.33 = 266.67, -0.25 %).
;   Byte boundary (bit 7): DJNZ c1 at J+15, HPULL at J+30, next JSB J+33.
;
.default EXT2, 1             ; extra cycle after bit 2 (34-cycle cell); 0 = plain 33
.default EXT5, 1             ; extra cycle after bit 5

.macro HOT k kn ext          ; bit k, 0 <= k <= 6: test, edge on 0 (falls into bit k+1)
bit\k:
    JSB sr[\k]==1, one\k @2
    PINS toggle uio6 uio7 @2          ; 0 bit: NRZI transition, J+3
    LDI c0, 6 @26+\ext                ; ones countdown restarts; next JSB at J+33(+ext)
.endm

.macro COLD k kn ext         ; 1 bit: count ones, stuff a 0 after the sixth
one\k:
    DJNZ c0, bit\kn @23+\ext          ; taken: next JSB at J+33(+ext)
    PINS toggle uio6 uio7 @8          ; sixth 1: stuffed 0 = transition at J+36(+ext)
    LDI c0, 6 @2
    JMP bit\kn @20                    ; next JSB at J+66(+ext)
.endm

start:                               ; bus released
    HPULL @2                          ; n
    MOV c1, sr @2
    PINS low uio6 @2                  ; drive J: D+ low (D- still pulled up) ...
    PINS high uio7 @2                 ; ... then D- high
    LDI c0, 7 @5                      ; J has been driven for 33 cycles (one cell) before the first edge
sync:
    PINS toggle uio6 uio7 @26         ; SYNC edges K J K J K J K, 33 cycles apart
    DJNZ c0, sync @2
    LDI c0, 6 @2                      ; ones countdown for the data
    NOP @41                           ; 8th SYNC cell (K held), then the first byte is pulled
next_byte:
    HPULL @11                         ; SR <- next byte (blocks if the FIFO is empty)
    HOT 0, 1, 0
    HOT 1, 2, 0
    HOT 2, 3, EXT2
    HOT 3, 4, 0
    HOT 4, 5, 0
    HOT 5, 6, EXT5
    HOT 6, 7, 0
bit7:                                ; last bit of the byte: continue at ob (byte end)
    JSB sr[7]==1, one7 @2
    PINS toggle uio6 uio7 @2          ; J+3
    LDI c0, 6 @2
    JMP ob @2                         ; ob: DJNZ c1 commits at J+15
    COLD 0, 1, 0
    COLD 1, 2, 0
    COLD 2, 3, EXT2
    COLD 3, 4, 0
    COLD 4, 5, 0
    COLD 5, 6, EXT5
    COLD 6, 7, 0
one7:
    DJNZ c0, ob @5                    ; J+9; taken: ob at J+15
    PINS toggle uio6 uio7 @26         ; stuffed 0 at J+36
    LDI c0, 6 @2
    JMP ob @2                         ; ob at J+48
ob:
    DJNZ c1, next_byte @2             ; more bytes: HPULL J+30 (J+63 after a stuffed 0), next JSB 3 later
    PINS low uio6 uio7 @20            ; EOP: SE0 = where the next cell's edge would be (J+36 / J+69)
    PINS high uio7 @65                ; J after 66 cycles of SE0 (D+ stays low)
    PINS float uio6 uio7 @32          ; release after one J bit time
    JMP start @2
