; spi_slave.asm - Full-duplex SPI slave for TTPE, zero RTL changes (50 MHz default clock).
;
; De-facto Motorola SPI mode convention (CPOL / CPHA, MSB first):
;   Mode 0 (CPOL=0, CPHA=0): SCK idle 0, sample on rise (leading),  shift on fall (trailing)
;   Mode 1 (CPOL=0, CPHA=1): SCK idle 0, shift on rise (leading),   sample on fall (trailing)
;   Mode 2 (CPOL=1, CPHA=0): SCK idle 1, sample on fall (leading),  shift on rise (trailing)
;   Mode 3 (CPOL=1, CPHA=1): SCK idle 1, shift on fall (leading),   sample on rise (trailing)
;
; Pins (matching sw/examples/spi_master.asm):
;   uio0  CS_N  active-low chip select (input)
;   uio1  MOSI  master-out slave-in (input)
;   uio2  MISO  master-in slave-out (Hi-Z via PINS float when CS_N=1; push-pull when CS_N=0)
;   uio3  SCK   serial clock (input)
;   uo6   END   toggled when a frame ends and MISO returns to Hi-Z (debug / framing)
;
; Build-time parameters (assemble with -D CPOL=0/1 -D CPHA=0/1):
;   CPOL  SCK idle polarity (default 0)
;   CPHA  clock phase (default 0)
;
; Host TX/RX FIFO protocol (50 MHz clock, 20 ns/cycle; TX FIFO MUST be preloaded before CS_N):
;   CPHA == 0 (Modes 0 & 2):
;     TX FIFO: [n, d0, d1, ..., d(n-1)]   (n = byte count 1..255; 0 means 256)
;     RX FIFO: [r0, r1, ..., r(n-1)]
;     Why length-prefixed: CPHA=0 must present byte k+1 bit 7 on the trailing SCK edge of byte k
;     (while CS_N is still low). Because ISA v0.2 HPULL blocks with no tx_empty pin, variable-length
;     exact-N-byte full-duplex with clean MISO Hi-Z on CS_N rise is infeasible in CPHA=0.
;   CPHA == 1 (Modes 1 & 3):
;     TX FIFO: [d0, d1, ..., d(N-1)]      (exact N bytes; variable-length framed by CS_N!)
;     RX FIFO: [r0, r1, ..., r(N-1)]
;     HPULL executes only after seeing the leading SCK edge of each byte, so CS_N rising ends the
;     frame cleanly without consuming an extra TX byte.
;
; Operating speed & zero-pad-delay ceilings at 50 MHz (H = SCK half-period in 20 ns cycles):
;   Recommended default SCK: 1.0 MHz (H = 25 cycles = 500 ns) across all 4 modes.
;   Zero-pad-delay model ceilings (no pad/board delay modeled):
;     * CPHA=0 (Modes 0 & 2): H >= 12 (240 ns -> 2.083 MHz max);
;         CS_N fall -> MISO bit 7 valid t_a = 6 cyc (120 ns), master needs t_lead >= 6 + t_su(M);
;         CS_N rise -> MISO Hi-Z t_dis = 6 cyc (120 ns), or 12 cyc (240 ns) after last SCK edge if t_lag=0;
;         inter-frame CS_N high time t_cs_hi >= 24 cyc (480 ns) when next frame is preloaded.
;     * CPHA=1 (Modes 1 & 3): H >= 20 (400 ns -> 1.250 MHz max with 60 ns MISO setup; H=17 hard floor);
;         CS_N fall -> 1st SCK lead t_lead >= 3 cyc (60 ns);
;         CS_N rise -> MISO Hi-Z t_dis up to 17 cyc (340 ns) in the 9-cycle CS_N/SCK poll loop
;         (up to 24 cyc = 480 ns after last SCK edge if t_lag=0) -- multi-slave bus limitation;
;         inter-frame CS_N high time t_cs_hi >= 26 cyc (520 ns) worst case.
;
; Documented infeasible cases (ISA v0.2, zero RTL changes):
;   * Variable-length exact-N-byte CPHA=0 full-duplex with clean MISO Hi-Z on CS_N rise.
;   * Unpreloaded TX FIFO or same-byte command-response (no clock stretching; HPULL stalls & misses SCK).
;   * Mid-byte CS_N abort on bits 1..7 (WAIT monitors SCK only).
;   * Frames > 8 bytes without timely host TX/RX FIFO service (8-deep FIFOs; RX drops with RX_OVR).

.default CPOL, 0
.default CPHA, 0

.macro WAIT_LEAD
.if CPOL == 0
    WAIT rise uio3 @2
.else
    WAIT fall uio3 @2
.endif
.endm

.macro WAIT_TRAIL
.if CPOL == 0
    WAIT fall uio3 @2
.else
    WAIT rise uio3 @2
.endif
.endm

.macro JMP_SCK_IDLE target
.if CPOL == 0
    JLO  uio3, \target @2
.else
    JHI  uio3, \target @2
.endif
.endm

start:
    PINS float uio2 @2          ; MISO Hi-Z while idle

.if CPHA == 0
; =============================================================================
; CPHA == 0 (Modes 0 & 2): sample on leading edge, shift on trailing edge.
; Every half-period path is bounded by H_min = 12 cycles (240 ns -> 2.083 MHz).
; =============================================================================
idle:
    HPULL @2                    ; frame byte count n (1..255; 0 = 256)
    MOV  c1, sr @2              ; c1 = remaining bytes in frame
    HPULL @2                    ; preload byte 0 into SR before CS_N falls
    WAIT low uio0 @2            ; CS_N falls at T_cs -> commits T_cs+2
    SHOUT uio2 msb @2           ; commits T_cs+5 -> MISO bit 7 valid on pad at T_cs+6 (120 ns)

byte_loop:                      ; Entry: MISO already holds bit 7 of current byte
    ; Bit 7 (MSB): LDI c0, 5 overlaps into H_1 (9 <= 12)
    WAIT_LEAD                   ; T_L0 -> commits T_L0+2
    SAMPLE uio1 msb @2          ; commits T_L0+5 (reads pad_MOSI(T_L0+3)); SR[7] <- bit 6
    LDI  c0, 5 @2               ; commits T_L0+8 (bits 6, 5, 4, 3, 2)
    WAIT_TRAIL                  ; armed T_L0+11 (H_1 >= 9) -> commits T_T0+2
    SHOUT uio2 msb @2           ; commits T_T0+5 -> MISO bit 6 on pad at T_T0+6 (120 ns)

    ; Bits 6..2 (5 iterations via DJNZ c0; H_1 >= 6, H_2 >= 12)
bit_loop:
    WAIT_LEAD                   ; T_Li -> commits T_Li+2
    SAMPLE uio1 msb @2          ; commits T_Li+5
    WAIT_TRAIL                  ; armed T_Li+8 (H_1 >= 6) -> commits T_Ti+2
    SHOUT uio2 msb @2           ; commits T_Ti+5 -> MISO pad T_Ti+6 (120 ns)
    DJNZ c0, bit_loop @2        ; taken (+3): next WAIT_LEAD armed T_Ti+14 (H_2 >= 12)

    ; Bit 1: decrement byte counter c1 inside H_1 before bit 0
    WAIT_LEAD                   ; T_L6 -> commits T_L6+2
    SAMPLE uio1 msb @2          ; commits T_L6+5
    DJNZ c1, b1_more @2         ; commits T_L6+8; taken if more bytes remain, fall-through if last byte

    ; Bit 1 trailing + Bit 0 (LAST byte of frame: no HPULL, float MISO on CS_N rise)
    WAIT_TRAIL                  ; armed T_L6+11 (H_1 >= 9) -> commits T_T6+2
    SHOUT uio2 msb @2           ; commits T_T6+5 -> MISO bit 0 on pad T_T6+6
    WAIT_LEAD                   ; armed T_T6+8 (H_2 >= 6) -> commits T_L7+2
    SAMPLE uio1 msb @2          ; commits T_L7+5 -> SR = complete last RX byte
    HPUSH @2                    ; commits T_L7+8
    WAIT_TRAIL                  ; armed T_L7+11 (H_1 >= 9) -> commits T_T7+2
    WAIT high uio0 @2           ; armed T_T7+5 -> catches CS_N rise at T_cs_hi -> commits T_cs_hi+2
    PINS float uio2 @2          ; commits T_cs_hi+5 -> MISO Hi-Z on pad at T_cs_hi+6 (120 ns)
    PINS toggle uo6 @2          ; frame-end marker
    JMP  idle @2                ; next frame (MISO already floated)

    ; Bit 1 trailing + Bit 0 (MORE bytes remain: overlap HPUSH+HPULL in bit 0 H_1)
b1_more:
    WAIT_TRAIL                  ; taken (+3): armed T_L6+14 (H_1 >= 12) -> commits T_T6+2
    SHOUT uio2 msb @2           ; commits T_T6+5 -> MISO bit 0 on pad T_T6+6
    WAIT_LEAD                   ; armed T_T6+8 (H_2 >= 6) -> commits T_L7+2
    SAMPLE uio1 msb @2          ; commits T_L7+5 -> SR = complete RX byte k
    HPUSH @2                    ; commits T_L7+8
    HPULL @2                    ; commits T_L7+11 -> SR = TX byte k+1 (must be preloaded)
    WAIT_TRAIL                  ; armed T_L7+14 (H_1 >= 12) -> commits T_T7+2
    SHOUT uio2 msb @2           ; commits T_T7+5 -> MISO byte k+1 bit 7 on pad T_T7+6 (120 ns)
    JMP  byte_loop @2           ; taken (+3): byte_loop WAIT_LEAD armed T_T7+14 (H_2 >= 12)

.else
; =============================================================================
; CPHA == 1 (Modes 1 & 3): shift on leading edge, sample on trailing edge.
; Variable-length exact-N-byte full-duplex framed by CS_N via 9-cycle poll loop.
; Hard floor H_min = 17 cyc (0 setup); H >= 20 cyc (400 ns -> 1.250 MHz, 60 ns setup).
; =============================================================================
idle:
    WAIT low uio0 @2            ; CS_N falls at T_cs -> commits T_cs+2

; 9-cycle CS_N / SCK poll loop: detects CS_N rise (frame end) or SCK active level (next byte).
; Worst-case SCK lead detection <= T_L0+10; worst-case CS_N rise -> MISO Hi-Z = T_cs_hi+17 (340 ns).
poll_byte:
    JHI  uio0, frame_end @2     ; CS_N high -> release MISO & finish frame
    JMP_SCK_IDLE poll_byte      ; SCK still at idle polarity -> keep polling

    ; Bit 7 (MSB): HPULL runs only after SCK leading edge is seen (exact N bytes consumed)
    HPULL @2                    ; commits <= T_L0+13
    SHOUT uio2 msb @2           ; commits <= T_L0+16 -> MISO bit 7 on pad <= T_L0+17 (340 ns)
    WAIT_TRAIL                  ; armed <= T_L0+19 -> catches T_T0 >= T_L0+17
    SAMPLE uio1 msb @2          ; commits T_T0+5 (reads pad_MOSI(T_T0+3))
    LDI  c0, 6 @2               ; commits T_T0+8 (bits 6, 5, 4, 3, 2, 1)

    ; Bits 6..1 (6 iterations via DJNZ c0; H_1 >= 6, H_2 >= 12)
bit_loop:
    WAIT_LEAD                   ; T_Li -> commits T_Li+2
    SHOUT uio2 msb @2           ; commits T_Li+5 -> MISO pad T_Li+6 (120 ns)
    WAIT_TRAIL                  ; armed T_Li+8 (H_1 >= 6) -> commits T_Ti+2
    SAMPLE uio1 msb @2          ; commits T_Ti+5 (reads pad_MOSI(T_Ti+3))
    DJNZ c0, bit_loop @2        ; taken (+3): next WAIT_LEAD armed T_Ti+14 (H_2 >= 12)

    ; Bit 0 (LSB): push RX byte and return to CS_N / SCK poll
    WAIT_LEAD                   ; T_L7 -> commits T_L7+2
    SHOUT uio2 msb @2           ; commits T_L7+5 -> MISO bit 0 on pad T_L7+6 (120 ns)
    WAIT_TRAIL                  ; armed T_L7+8 (H_1 >= 6) -> commits T_T7+2
    SAMPLE uio1 msb @2          ; commits T_T7+5 -> SR = complete RX byte
    HPUSH @2                    ; commits T_T7+8
    JMP  poll_byte @2           ; taken (+3): JHI uio0 at T_T7+17, JMP_SCK_IDLE at T_T7+20

frame_end:
    PINS float uio2 @2          ; MISO Hi-Z (<= T_cs_hi+17 = 340 ns; <= T_T7+24 if t_lag=0)
    PINS toggle uo6 @2          ; frame-end marker
    JMP  idle @2
.endif
