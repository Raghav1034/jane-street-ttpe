; i2c_master.asm - I2C master (single master, 7-bit addressing, clock stretching supported).
; Tiny Tapeout I2C pinout: SCL = uio2, SDA = uio3.  Both lines are used open-drain:
; 'PINS low' pulls the line down, 'PINS float' releases it to the external pull-up.
; (A real bus needs external pull-ups on a Pmod; the chip has no internal ones.)
;
; Parameters (assemble with -D TL=75 -D TH=50 ...), all in 20 ns clock cycles at 50 MHz:
;   TL   SCL low time.   TH   SCL high time.   SCL period = TL + TH.
;   DSH  extra SDA hold after SCL falls: SDA changes DSH+7 cycles after the SCL fall edge.
;
;   standard mode  100 kHz:  TL = 250, TH = 250  (5.0 us / 5.0 us; spec min 4.7 / 4.0)  [default]
;   fast mode      400 kHz:  TL = 75,  TH = 50   (1.5 us / 1.0 us; spec min 1.3 / 0.6)
;
;   Valid range: TL >= DSH + 14 (so every delta is >= 2) and TH >= 27.
;
; Timing guarantee (checked cycle-exactly in tests/test_examples.py):
;   * every SCL pulse inside a byte, including the ACK clock of a write, is exactly TH high
;     (if the slave does not stretch) and exactly TL low (data bits and the write ACK clock);
;   * the first SCL pulse of a byte that follows another byte, and the master-ACK clock of a
;     read, may be LONGER low (never shorter) because the program executes extra instructions
;     in that gap;
;   * START hold (SDA fall -> SCL fall) = TH, STOP set-up (SCL rise -> SDA rise) = TH,
;     bus-free time after STOP = TL.
;
; Host protocol (FIFO bytes):
;   write:  (addr<<1)|0,  n,  d0 ... d(n-1)         n[6:0] = byte count, 0 means 256
;   read :  (addr<<1)|1,  n                         engine pushes n bytes into the RX FIFO
;   n[7] = 0: the transaction ends with a STOP (bus released).
;   n[7] = 1: the transaction ends WITHOUT a STOP: SCL stays low, SDA released, the bus is held.
;             The next command byte then starts with a REPEATED START (Sr) instead of a START:
;                 write-register-pointer + read:   A0 81 REG   A1 n      (S A+W REG Sr A+R data P)
;   Compatibility: counts 1..127 and 256 (n = 0, or n = 0x80 with Sr) work as before.  n values
;   128..255 used to mean 128..255 bytes with a STOP; now n[7] selects Sr (count = n & 0x7F).
;   Reads ACK every byte but the last (NACK), then a STOP (n[7] = 0) or the held bus (n[7] = 1).
;   Sr timing: SCL low >= TL before the rising edge, SDA released while SCL is low, tSU;STA =
;   SCL rise -> SDA fall = exactly TH, tHD;STA = SDA fall -> SCL fall = TH (same START macro).
;   While the bus is held the engine waits in HPULL; the host must send the next command (or
;   any command, e.g. a dummy one ending in STOP) - there is no timeout.
;   If a slave NACKs an address or data byte, the transaction ends with a STOP and the
;   error flag uo7 goes high (cleared when the next command byte arrives).
;   KNOWN GAP: the unread remainder of an aborted write is still in the host TX FIFO; the host
;   must flush it (planned TX_FLUSH control register) before sending the next command.
;
; Registers used: SR = byte being sent / received, C0 = bit counter, C1 = byte counter.
.default TL, 250
.default TH, 250
.default DSH, 14
.default RS_LOW, TL-19       ; Sr: delay before SCL rises.  Fastest path (next command already in the FIFO)
                              ; gives SCL low = TL exactly (21 cycles of fixed overhead, measured on the ISS)
.default RS_SU, TH-10         ; Sr: delay that makes tSU;STA = SCL rise -> SDA fall = exactly TH (10 fixed, measured)

; ---- Timing derivation (ISA rules: sequential spacing = delta+1, taken branch +3) ---------
;  data bit loop:  SHOUT -> PINS float(SCL up) -> WAIT -> PINS low(SCL down) -> DJNZ(taken)
;    SCL high = 3 (WAIT) + (TH-4) + 1                      = TH
;    SCL low  = 3 (DJNZ) + 3 (taken) + (DSH+1) + (TL-DSH-8) + 1 = TL
;  The first bit enters the loop by fall-through, so LDI + NOP @2 (3 + 3 cycles) stand in
;  for the DJNZ + taken-branch cycles of later bits.

.macro START
    PINS low uio3 @2              ; SDA falls while SCL is high = START
    PINS low uio2 @TH-1           ; SCL falls TH cycles later (tHD;STA)
.endm

.macro SEND_BYTE                  ; SCL low on entry and on exit; SR sent MSB first
    LDI c0, 8 @2
    NOP @2                        ; stands in for the DJNZ + taken-branch cycles (see above)
sb\@:
    SHOUT uio3 msb shift od @DSH  ; open-drain: bit 0 -> pull low, bit 1 -> release
    PINS float uio2 @TL-DSH-8     ; SCL rises
    WAIT high uio2 @2             ; clock stretching: block until the line really is high
    PINS low uio2 @TH-4           ; SCL high time = TH, then SCL low
    DJNZ c0, sb\@ @2
.endm

.macro GET_ACK nack_label         ; jump to nack_label if the slave does not pull SDA low
    PINS float uio3 @DSH          ; release SDA for the slave (DJNZ fell through: no penalty)
    PINS float uio2 @TL-DSH-5     ; ACK clock rises: SCL low = 3 + (DSH+1) + (TL-DSH-5) + 1 = TL
    WAIT high uio2 @2
    JHI uio3, \nack_label @20     ; SDA still high => NACK; sampled 3+21 cycles after SCL rose
    PINS low uio2 @TH-25          ; SCL high time = 3 + 21 + (TH-25) + 1 = TH
.endm

.macro READ_BYTE                  ; SCL low on entry and exit; result in SR
    PINS float uio3 @2            ; master releases SDA
    LDI c0, 8 @2
rb\@:
    PINS float uio2 @TL-7         ; SCL high (loop: low time = 3 + 3 + (TL-7) + 1 = TL)
    WAIT high uio2 @2
    SAMPLE uio3 msb @TH/2-2       ; sample SDA in the middle of the high phase
    PINS low uio2 @TH-5-(TH/2-2)  ; SCL high time = 3 + (TH/2-1) + (TH-4-TH/2+2) = TH
    DJNZ c0, rb\@ @2
.endm

.macro CLOCK_ACK_BIT dlow         ; one SCL pulse for the master's ACK/NACK bit, SDA already set
    PINS float uio2 @\dlow        ; SCL rises (low time >= TL, see the two call sites)
    WAIT high uio2 @2
    PINS low uio2 @TH-4           ; SCL high time = TH
.endm

init:
    PINS float uio2 uio3 @2       ; bus idle
    PINS low uo7 @2               ; error flag low
    NOP @TL-1                     ; bus-free time before the first START
cmd:
    HPULL @2                      ; (addr<<1)|rw
    PINS low uo7 @2               ; clear error flag
    MOV c1, sr @2                 ; remember rw in C1
do_start:                         ; also entered from `hold` after the Sr set-up delay
    START
    SEND_BYTE
    GET_ACK nack
    MOV sr, c1 @2
    JSB sr[0]==1, do_read @2      ; branch on the R/W bit

; ------------------------------------------------------------------ write transaction
.macro WRITE_BODY lp exit         ; SR/C1 = count; byte loop, then jump to exit (lp: unique label)
\lp:
    HPULL @2                      ; data byte
    SEND_BYTE
    GET_ACK nack
    DJNZ c1, \lp @2
    JMP \exit @2
.endm
    HPULL @2                      ; n
    JSB sr[7]==1, wr_sr @2        ; n[7]: end with Sr instead of STOP
    MOV c1, sr @2
    WRITE_BODY wl_s, stop
wr_sr:
    ANDI sr, 0x7F @2
    MOV c1, sr @2                 ; count 0 -> 256 (DJNZ wraps), as for n = 0
    WRITE_BODY wl_h, hold

; ------------------------------------------------------------------- read transaction
.macro READ_BODY lp ak exit        ; lp, ak: unique labels (nested macros break \@)
\lp:
    READ_BYTE
    HPUSH @2
    DJNZ c1, \ak @2              ; more bytes to come -> ACK, otherwise fall through (NACK)
    PINS float uio3 @DSH          ; NACK = leave SDA high   (low time >= 3+3+3 + (DSH+1) + dlow+1)
    CLOCK_ACK_BIT TL-DSH-11
    JMP \exit @2
\ak:
    PINS low uio3 @DSH            ; ACK = pull SDA low       (low time >= 3+3+3+3 + (DSH+1) + dlow+1)
    CLOCK_ACK_BIT TL-DSH-14
    JMP \lp @2
.endm
do_read:
    HPULL @2                      ; n
    JSB sr[7]==1, rd_sr @2
    MOV c1, sr @2
    READ_BODY rl_s, am_s, stop
rd_sr:
    ANDI sr, 0x7F @2
    MOV c1, sr @2
    READ_BODY rl_h, am_h, hold

; ------------------------------------------- bus held (n[7] = 1): next command = repeated START
; Entry: SCL low, SDA released.  The next command byte is fetched first (the bus stays held),
; then SCL rises (low time >= TL), SDA stays high, and after exactly TH the START macro pulls SDA low.
hold:
    HPULL @2                      ; (addr<<1)|rw of the next transaction
    PINS low uo7 @2               ; clear error flag
    MOV c1, sr @2
    PINS float uio2 @RS_LOW       ; SCL rises; SDA is already released (high)
    WAIT high uio2 @2             ; clock stretching
    JMP do_start @RS_SU           ; -> START macro: SDA falls TH after SCL rose, SCL TH later

; --------------------------------------------------------------------------- STOP
nack:
    PINS high uo7 @2              ; raise the error flag
    PINS low uio2 @TH             ; SCL was high during the NACK bit: bring it low first
stop:
    PINS low uio3 @DSH            ; SDA low (SCL low)
    PINS float uio2 @TL           ; SCL high
    WAIT high uio2 @2
    PINS float uio3 @TH-4         ; SDA rises while SCL high = STOP (set-up = TH)
    NOP @TL                       ; bus-free time before the next START
    JMP cmd @2
