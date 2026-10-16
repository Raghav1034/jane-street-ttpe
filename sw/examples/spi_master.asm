; spi_master.asm - SPI master, MSB first, all four modes.
; Tiny Tapeout SPI pinout: CS = uio0, MOSI = uio1, MISO = uio2, SCK = uio3.
;
; Build-time parameters (assemble with -D CPOL=1 -D CPHA=1 -D H=40 ...):
;   CPOL  idle level of SCK (0/1)           CPHA  0 = sample on leading edge, 1 = trailing
;   H     SCK half-period in cycles. SCK frequency = 50 MHz / (2*H).
;         Minimum: H >= 12 for CPHA=0, H >= 9 for CPHA=1 (3-cycle instruction floor).
;
; Host protocol (FIFO bytes):  n, d0, d1, ... d(n-1)      (n = 0 means 256 bytes)
;   The engine asserts CS, exchanges n bytes and pushes the n bytes read from MISO into the
;   RX FIFO, then releases CS and waits for the next transfer.
;
; Per-bit timing (derived from the ISA timing model; every instruction's delta is chosen so
; that BOTH half periods are exactly H cycles, including the DJNZ loop overhead):
;   CPHA=0:  SHOUT, edge, SAMPLE, edge, DJNZ     high phase = c+d+2 = H, low phase = a+b+e+6 = H
;   CPHA=1:  edge, SHOUT, SAMPLE, edge, DJNZ     high phase = a+c+d+3 = H, low phase = e+b+5 = H
.default CPOL, 0
.default CPHA, 0
.default H, 24

.if CPHA == 0
  .equ DA, 2            ; SHOUT delta
  .equ DB, H-10         ; leading-edge delta (MOSI set-up time = DB+1 cycles)
  .equ DC, H-4          ; SAMPLE delta
.else
  .equ DA, 2
  .equ DB, H-7
  .equ DC, H-7
.endif

start:
    PINS high uio0 @2             ; CS idle high
.if CPOL
    PINS high uio3 @2             ; SCK idle high
.else
    PINS low uio3 @2              ; SCK idle low
.endif
xfer:
    HPULL @2                      ; n
    MOV c1, sr @2
    PINS low uio0 @10             ; assert CS, 11 cycles before the first bit
byte_loop:
    HPULL @2                      ; byte to send
    LDI c0, 8 @2
bit_loop:
.if CPHA == 0
    SHOUT uio1 msb @DA            ; MOSI <- SR[7]
    PINS toggle uio3 @DB          ; leading edge
    SAMPLE uio2 msb @DC           ; shift MISO into SR (SR is the full-duplex shift register)
    PINS toggle uio3 @2           ; trailing edge
.else
    PINS toggle uio3 @DB          ; leading edge
    SHOUT uio1 msb @DA            ; MOSI <- SR[7]
    SAMPLE uio2 msb @DC
    PINS toggle uio3 @2           ; trailing edge
.endif
    DJNZ c0, bit_loop @2
    HPUSH @2                      ; received byte -> host
    DJNZ c1, byte_loop @2
    PINS high uio0 @10            ; release CS
    JMP xfer @2
