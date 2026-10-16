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
;   write:  (addr<<1)|0,  n,  d0 ... d(n-1)         n = 0 means 256
;   read :  (addr<<1)|1,  n                         engine pushes n bytes into the RX FIFO
;   Reads ACK every byte but the last (NACK), then a STOP is generated.
;   If a slave NACKs an address or data byte, the transaction ends with a STOP and the
;   error flag uo7 goes high (cleared when the next command byte arrives).
;   KNOWN GAP: the unread remainder of an aborted write is still in the host TX FIFO; the host
;   must flush it (planned TX_FLUSH control register) before sending the next command.
;
; Registers used: SR = byte being sent / received, C0 = bit counter, C1 = byte counter.
.default TL, 250
.default TH, 250
.default DSH, 14

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
    START
    SEND_BYTE
    GET_ACK nack
    MOV sr, c1 @2
    JSB sr[0]==1, do_read @2      ; branch on the R/W bit

; ------------------------------------------------------------------ write transaction
    HPULL @2                      ; n
    MOV c1, sr @2
wr_loop:
    HPULL @2                      ; data byte
    SEND_BYTE
    GET_ACK nack
    DJNZ c1, wr_loop @2
    JMP stop @2

; ------------------------------------------------------------------- read transaction
do_read:
    HPULL @2                      ; n
    MOV c1, sr @2
rd_loop:
    READ_BYTE
    HPUSH @2
    DJNZ c1, ack_more @2          ; more bytes to come -> ACK, otherwise fall through (NACK)
    PINS float uio3 @DSH          ; NACK = leave SDA high   (low time >= 3+3+3 + (DSH+1) + dlow+1)
    CLOCK_ACK_BIT TL-DSH-11
    JMP stop @2
ack_more:
    PINS low uio3 @DSH            ; ACK = pull SDA low       (low time >= 3+3+3+3 + (DSH+1) + dlow+1)
    CLOCK_ACK_BIT TL-DSH-14
    JMP rd_loop @2

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
