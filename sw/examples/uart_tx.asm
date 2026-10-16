; uart_tx.asm - UART transmitter, 8N1, LSB first.  TXD = uio1 (Tiny Tapeout UART pinout).
;
; Host protocol: every byte the host writes into the TX FIFO is sent as one frame.
; The program blocks on HPULL while the FIFO is empty (line idles high).
;
;   BIT = clock cycles per bit  (50 MHz / 115200 = 434.03 -> 434, error 0.007 %)
;
; Timing (exact, derived from the ISA timing model, verified in tests/test_examples.py):
;   every bit boundary is exactly BIT cycles after the previous one; if the next byte is
;   already queued, its start bit begins exactly BIT cycles after the end of the previous
;   stop bit's first cycle, i.e. a 1-bit stop time.
.default BIT, 434

start:
    PINS high uio1 @2             ; idle high (push-pull)
next_byte:
    HPULL @2                      ; SR <- next byte (blocks until available)
    LDI c0, 8 @2
    PINS low uio1 @2              ; START bit begins here (commit cycle S0)
    NOP @5                        ; first SHOUT must land exactly BIT cycles after S0
bit_loop:
    SHOUT uio1 lsb shift @BIT-7   ; data bit, LSB first; loop overhead is folded into the delta
    DJNZ c0, bit_loop @2
    PINS high uio1 @BIT-4         ; STOP bit begins exactly BIT cycles after the last data bit
    JMP next_byte @BIT-13         ; keep the stop bit >= BIT cycles before the next start bit
