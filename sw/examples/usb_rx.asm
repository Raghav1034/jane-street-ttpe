; usb_rx.asm - low-speed USB (1.5 Mbit/s) packet receiver for the PROPOSED ttpe_usb_rx
; bit-slicer primitive (docs/USB_LS_RX.md).  NOT runnable on the current RTL: it needs the
; primitive and its four virtual source pins, which exist only in sw/usb_ls.py (reference model).
;
; Virtual source pins (valid only while USBEN is set; they replace ui4..ui7):
;   ui4 TICK  1-cycle pulse per decoded data bit (NRZI decoded, stuffed bits removed) and one at packet end
;   ui5 DATA  last decoded bit, held for one bit time
;   ui6 STOP  set at the packet-end tick (EOP or bit-stuff error); held until the next SYNC
;   ui7 ERR   set together with STOP when the packet ended in a bit-stuff violation
; Packet-end flags toggled on output pins:  uo6 = every packet end,  uo5 = bad packet
; (error, or a last byte that is not complete).  Bytes are pushed to the host RX FIFO LSB-first
; as received (PID first).  The FIFO is 8 deep and HPUSH never blocks: the host must drain it.
;
; Per bit: WAIT commits at t (the TICK cycle), JHI t+3, SAMPLE t+6, DJNZ t+9, next WAIT
; armed at t+15 (3-cycle branch penalty), next TICK at t+33/t+34 -> 18 cycles of slack.
; Per byte: DJNZ falls through at t+9, HPUSH t+12, LDI t+15, then the first bit of the next byte
; is an unrolled copy of the loop head (no taken JMP): WAIT armed at t+18 -> 15 cycles of slack.

start:
    LDI c1, 8 @2                  ; bits left in the current byte
bit_loop:
    WAIT rise ui4 @2              ; next TICK
    JHI ui6, pkt_end @2           ; STOP -> end of packet
    SAMPLE ui5 lsb @2             ; SR <- {DATA, SR[7:1]}
    DJNZ c1, bit_loop @2
    HPUSH @2                      ; one full byte
    LDI c1, 8 @2
    WAIT rise ui4 @2              ; first bit of the next byte: unrolled copy of bit_loop,
    JHI ui6, pkt_end @2           ; avoids a taken JMP (3 cycles) on the byte path
    SAMPLE ui5 lsb @2
    DJNZ c1, bit_loop @2          ; c1 is 7 here, so this branch is always taken
    JMP bit_loop @2               ; never reached (defensive)

pkt_end:
    JHI ui7, bad @2               ; stuff error
    MOV sr, c1 @2
    JSB sr[3]==1, ok @2           ; c1 == 8 only when no partial byte is pending
bad:
    PINS toggle uo5 @2
ok:
    PINS toggle uo6 @2
    JMP start @2
