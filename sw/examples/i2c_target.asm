; i2c_target.asm - I2C target (slave) for TTPE, zero RTL changes (50 MHz default clock).
;
; Pins (matching sw/examples/i2c_master.asm):
;   uio2  SCL   open-drain (target only pulls low for clock stretching, or floats)
;   uio3  SDA   open-drain (target only pulls low for ACK / data 0, or floats)
;   uo5   ADDR  toggled whenever a START/Sr matches our 7-bit address (debug / framing)
;   uo6   END   toggled when a matched transaction ends on STOP or Repeated START (Sr)
;
; Host protocol:
;   Write (master -> target): received payload bytes are pushed to the host RX FIFO (HPUSH).
;     Hardware limit: ISA v0.2 HPUSH never blocks and cannot read rx_full, so if the 8-deep
;     RX FIFO overflows, hardware drops the byte and increments RX_OVR (STATUS.ovr=1) while
;     the target still ACKs on the wire. The host must drain RX within 8 bytes (720 us at
;     100 kHz, 180 us at 400 kHz).
;   Read  (target -> master): before each transmitted byte the target pulls SCL low (standard
;     I2C clock stretching), executes HPULL from the host TX FIFO, drives bit 7 onto SDA,
;     waits TSU_DAT cycles, and releases SCL. If the master does not support clock stretching,
;     the host must preload the TX FIFO before the Read byte boundary so HPULL does not stall
;     SCL past t_LOW.
;
; Parameters (assemble with -D ADDR=0x50 -D THD_DAT=15 -D TSU_DAT=65):
;   ADDR     7-bit target address (default 0x50; matches byte[7:1] == ADDR).
;   THD_DAT  internal SDA hold cycles after SCL falling edge (default 15 = 300 ns at 50 MHz;
;            must be >= 12 so @THD_DAT-10 >= 2). Required by NXP UM10204 Table 10/11 Note:
;            "A device must internally provide a hold time of at least 300 ns for the SDA
;            signal (referred to the VIH(min) of the SCL signal) to bridge the undefined
;            region of the falling edge of SCL."
;   TSU_DAT  data setup cycles from target SDA release to SCL release on TX byte-boundary
;            stretch (instruction delta @TSU_DAT-1; must satisfy THD_DAT + TSU_DAT < t_LOW
;            so preloaded reads do not stretch):
;            * 100 kHz Standard-mode default: TSU_DAT=65 (1300 ns; @64). Covers worst-case
;              UM10204 SDA rise time t_r=1000 ns (50 cyc) + t_SU;DAT min 250 ns (13 cyc)
;              with +50 ns margin even at t_r(SCL)=0; THD_DAT+TSU_DAT = 80 cyc << 235 t_LOW.
;            * 400 kHz Fast-mode: assemble with -D TSU_DAT=22 (440 ns; @21). Covers worst-case
;              t_r=300 ns (15 cyc) + t_SU;DAT min 100 ns (5 cyc) with +40 ns margin even at
;              t_r(SCL)=0; THD_DAT+TSU_DAT = 37 cyc < 65 t_LOW.
;
; Known hardware limits:
;   * No input spike filter: TTPE uses a 2-flop synchroniser (no 50 ns Fast-mode t_SP filter).
;   * Default build is 100 kHz Standard-mode. 400 kHz Fast-mode (-D TSU_DAT=22) is model-feasible
;     in cycle arithmetic, NOT proven on a real bus (worst-case 300 ns bus t_f consumes most
;     of the 360 ns address-match t_VD;ACK margin). 1 MHz Fast-mode Plus (Fm+) is infeasible.
;   * Sr / STOP polling runs on bit 0 of every data byte (all legal I2C byte boundaries) and
;     after a master NACK on Read; illegal mid-byte Sr/STOP on bits 1..7 is not polled.

.default ADDR,    0x50
.default THD_DAT, 15            ; 300 ns internal SDA hold after SCL fall (UM10204 Note; >= 12)
.default TSU_DAT, 65            ; 1300 ns SDA setup before SCL release (100 kHz >= 1000+250 ns; 400 kHz: 22)

start:
    PINS float uio2 uio3 @2     ; release SCL and SDA (idle bus)

; Wait for START: SDA falling edge while SCL is high.
; WAIT commits at X+2 (pad fell at X); JLO commits at X+5 (reads pad_SCL(X+3)).
; If SCL == 0 (normal data edge), taken branch re-arms WAIT at X+11 (9-cycle blind window).
wait_start:
    WAIT fall uio3 @2
    JLO  uio2, wait_start @2

; Entry after START or Repeated START (Sr) while SCL is still high.
got_start:
    WAIT low uio2 @2            ; wait for SCL fall ending START hold (t_HD;STA)
    LDI  c0, 8 @2               ; 8 bits: 7-bit address + R/W

; Receive 8-bit address + R/W byte (MSB first).
rx_addr_bit:
    WAIT rise uio2 @2           ; SCL rise at T_r -> commits T_r+2
    SAMPLE uio3 msb @2          ; commits T_r+5 (samples pad_SDA(T_r+3))
    WAIT low uio2 @2            ; armed T_r+8, commits T_f+2
    DJNZ c0, rx_addr_bit @2     ; taken: next WAIT rise armed T_f+11; fall-through: T_f+5

; Address match (after bit-7 SCL fall at T_f):
;   MOV c1,sr (T_f+8) -> ANDI (T_f+11) -> XORI (T_f+14) -> ADDI (T_f+17) ->
;   MOV c0,sr (T_f+20) -> DJNZ miss (T_f+23) -> PINS low sda (T_f+26, pad LOW at T_f+27 = 540 ns).
    MOV  c1, sr @2              ; save full byte (c1[0] = R/W)
    ANDI sr, 0xFE @2            ; mask 7-bit address
    XORI sr, (ADDR << 1) @2     ; zero iff address matches
    ADDI sr, 1 @2               ; 1 iff match -> DJNZ falls through
    MOV  c0, sr @2
    DJNZ c0, wait_start @2      ; mismatch: leave SDA floated (NACK), wait for next START/Sr

; Address matched: drive ACK (SDA low), toggle uo5, dispatch on R/W after 9th SCL rise.
    PINS low uio3 @2            ; ACK on pad at T_f+27 (540 ns)
    PINS toggle uo5 @2          ; signal matched address
    MOV  sr, c1 @2              ; restore {addr, rw}
    WAIT rise uio2 @2           ; 9th clock rising edge (T_r+2)
    JSB  sr[0]==1, tx_byte @2   ; R/W == 1 -> Read (tx_byte WAIT low armed at T_r+11)

; Write (R/W == 0): wait for 9th SCL fall, hold SDA for THD_DAT cycles, then release ACK.
    WAIT low uio2 @2            ; 9th SCL fall at T_f -> commits T_f+2
    PINS float uio3 @THD_DAT-4  ; commits T_f+THD_DAT-1 -> pad floats at T_f+THD_DAT (300 ns)

; Receive data bytes from master until Repeated START (Sr) or STOP (P).
; Bit 0 polls SCL and SDA during SCL==1 to detect Sr (SDA 1->0) or STOP (SDA 0->1).
rx_byte:
    WAIT rise uio2 @2           ; bit 0 SCL rise at T_r -> commits T_r+2
    SAMPLE uio3 msb @2          ; commits T_r+5 (SR[0] = initial SDA level)
    JSB  sr[0]==1, rx_hi_poll @2

; Initial SDA == 0: watch for SCL fall (normal bit 0) or SDA rise while SCL==1 (STOP).
rx_lo_poll:
    JLO  uio2, rx_b0_done @2    ; SCL fell -> normal data bit
    JLO  uio3, rx_lo_poll @2    ; SDA still 0 -> keep polling (9 cycles/iter)
    JLO  uio2, rx_b0_done @2    ; confirm SCL still high (prevents false STOP at t_HD;DAT=0)
    PINS toggle uo6 @2          ; STOP confirmed: end of matched transaction
    JMP  wait_start @2

; Initial SDA == 1: watch for SCL fall (normal bit 0) or SDA fall while SCL==1 (Repeated START).
rx_hi_poll:
    JLO  uio2, rx_b0_done @2    ; SCL fell -> normal data bit
    JHI  uio3, rx_hi_poll @2    ; SDA still 1 -> keep polling (9 cycles/iter)
    JLO  uio2, rx_b0_done @2    ; confirm SCL still high (prevents false Sr at t_HD;DAT=0)
    PINS toggle uo6 @2          ; Repeated START confirmed: end previous transfer, start next
    JMP  got_start @2

rx_b0_done:
    LDI  c0, 7 @2               ; 7 remaining bits (bits 1..7)

rx_data_bit:
    WAIT rise uio2 @2
    SAMPLE uio3 msb @2
    WAIT low uio2 @2
    DJNZ c0, rx_data_bit @2

; Received full byte: push to RX FIFO (non-blocking), drive ACK after THD_DAT hold, release.
    HPUSH @2                    ; T_f+8
    PINS low uio3 @THD_DAT-10   ; T_f+THD_DAT-1 -> pad LOW at T_f+THD_DAT (300 ns)
    WAIT rise uio2 @2           ; 9th clock rising edge
    WAIT low uio2 @2            ; 9th clock falling edge
    PINS float uio3 @THD_DAT-4  ; pad floats at T_f+THD_DAT (300 ns)
    JMP  rx_byte @2

; Read (R/W == 1): stretch SCL low after 9th SCL fall, HPULL byte, drive bit 7, wait TSU_DAT, release SCL.
tx_byte:
    WAIT low uio2 @2            ; 9th SCL fall at T_f -> commits T_f+2
    PINS low uio2 @2            ; stretch SCL on pad at T_f+6 (120 ns)
    HPULL @2                    ; blocks safely while SCL stretched; T_f+8 if TX FIFO ready
    SHOUT uio3 msb shift od @THD_DAT-10 ; pad SDA valid at >= T_f+THD_DAT (300 ns)
    PINS float uio2 @TSU_DAT-1  ; release SCL exactly TSU_DAT cycles after SDA valid on pad
    WAIT high uio2 @2           ; wait for SCL rise (level-sensitive, safe for any bus t_r)
    LDI  c0, 7 @2               ; 7 remaining bits (bits 6..0)

tx_bit:
    WAIT low uio2 @2            ; SCL fall at T_f -> commits T_f+2
    SHOUT uio3 msb shift od @THD_DAT-4 ; commits T_f+THD_DAT-1 -> pad SDA valid at T_f+THD_DAT (300 ns)
    WAIT rise uio2 @2           ; armed T_f+THD_DAT+2 -> commits T_r+2
    DJNZ c0, tx_bit @2          ; taken: next WAIT low armed T_r+11

; 9th clock of Read: release SDA after bit-0 SCL fall, sample master ACK (0) / NACK (1).
    WAIT low uio2 @2            ; bit-0 SCL fall at T_f -> commits T_f+2
    PINS float uio3 @THD_DAT-4  ; release SDA at T_f+THD_DAT (300 ns)
    WAIT rise uio2 @2           ; 9th SCL rise at T_r -> commits T_r+2
    JLO  uio3, tx_byte @2       ; master ACK (SDA==0) -> next TX byte (WAIT low armed T_r+11)

; Master NACK (SDA==1): wait for 9th SCL fall, then detect STOP or Repeated START (Sr).
    WAIT low uio2 @2
    WAIT rise uio2 @2
    JHI  uio3, tx_end_hi @2

tx_end_lo:
    JLO  uio2, tx_end_abort @2  ; unexpected SCL fall -> abort to idle
    JLO  uio3, tx_end_lo @2     ; wait for SDA rise (STOP)
    JLO  uio2, tx_end_abort @2  ; confirm SCL still high
    PINS toggle uo6 @2          ; STOP confirmed
    JMP  wait_start @2

tx_end_hi:
    JLO  uio2, tx_end_abort @2  ; unexpected SCL fall -> abort to idle
    JHI  uio3, tx_end_hi @2     ; wait for SDA fall (Repeated START)
    JLO  uio2, tx_end_abort @2  ; confirm SCL still high
    PINS toggle uo6 @2          ; Sr confirmed
    JMP  got_start @2

tx_end_abort:
    PINS toggle uo6 @2
    JMP  wait_start @2
