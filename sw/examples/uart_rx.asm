; uart_rx.asm - UART receiver, 8N1, LSB first.  RXD = uio2 (Tiny Tapeout UART pinout).
;
; Every received byte is pushed into the host RX FIFO (HPUSH never blocks; if the host does
; not drain the FIFO in time the byte is dropped and the OVERRUN status flag is set).
;
; Sampling point: each bit is sampled 1.5/1/1/... bit times after the START edge was
; SEEN, i.e. in the middle of the bit.  The 2-flop input synchroniser delays the edge by
; 2 cycles and the sample reads the synchronised value (also 2 cycles old); these two
; delays cancel, so the sample lands exactly on the bit centre (BIT must be even).
.default BIT, 434

rx_loop:
    WAIT fall uio2 @2             ; commit cycle W = start-edge cycle + 2 (sync latency)
    LDI c0, 7 @2                  ; 7 more samples after the first one
    SAMPLE uio2 lsb @3*BIT/2-4    ; bit 0, centre of the bit
    NOP @5                        ; align the loop (see uart_tx.asm for the arithmetic)
bit_loop:
    SAMPLE uio2 lsb @BIT-7        ; bits 1..7
    DJNZ c0, bit_loop @2
    HPUSH @2                      ; hand the byte to the host
    WAIT high uio2 @2             ; wait for the stop bit / idle line
    JMP rx_loop @2
