; usb_device_ack.asm - autonomous low-speed USB device handshake: receive a packet with the ttpe_usb_rx
; slicer (MODE[0] = 1), push every byte to the host RX FIFO, wait for the end of the host's EOP, and answer
;     DATA0 / DATA1 -> ACK (0xD2)        IN -> NAK (0x5A)        anything else, bad packet -> no response
; entirely on the engine, no host involvement in the turnaround.  Combines usb_rx.asm (receive) and the
; transmit half of usb_tx.asm (byte loop copied; the response byte is loaded from a register instead of
; being pulled from the TX FIFO).
;
; THIS IS A TURNAROUND DEMONSTRATOR, NOT A COMPLIANT DEVICE: it does not check CRC5/CRC16 (no 16-bit
; register), the device address or endpoint, the data toggle, or the order of token and data; a data
; packet with a bad CRC but a clean bit stream is still ACKed.  Only a bit-stuff violation or a partial last
; byte (flagged on uo5, no response) is rejected.  A real device needs at least the CRC check.
;
; Pins: D+ = uio6, D- = uio7 (push-pull only while transmitting); ui4..ui7 = TICK/DATA/STOP/ERR of the
; slicer; uo6 toggles at every packet end, uo5 toggles for a bad packet (as in usb_rx.asm).
;
; Turnaround: `WAIT high uio7` synchronises to the return of D- to J, i.e. the end of the host's SE0, which
; arrives 34..47 cycles after pkt_end commits for a 62..75-cycle host SE0.  The response SYNC (first K
; edge) then starts a fixed number of cycles later; PAD_* equalise the three response paths.
; The slicer sees the engine's own transmission on the pads; its tick/stop pulses fall inside the
; transmission, while the program is not waiting for them, and the program returns to `start` only after
; the pins are released (usb_device_ack tests check one uo6 toggle per HOST packet).
;
; Registers: c0 = PID byte of the packet being received (0 = none yet), c1 = bit counter; in the response
; c1 = response PID, c0 = SYNC counter / ones countdown, sr = byte being sent.
.default EXT2, 1
.default EXT5, 1
.default PAD_D0, 76
.default PAD_D1, 58
.default PAD_IN, 46

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

start:
    LDI c0, 0 @2                  ; no PID yet (0x00 is never a valid PID byte)
    LDI c1, 8 @2
pid_loop:
    WAIT rise ui4 @2
    JHI ui6, pkt_end @2
    SAMPLE ui5 lsb @2
    DJNZ c1, pid_loop @2
    HPUSH @2                      ; PID to the host
    MOV c0, sr @2                 ; ... and remembered for the response
    LDI c1, 8 @2
    WAIT rise ui4 @2              ; first bit of byte 1: unrolled copy of bit_loop (no taken JMP)
    JHI ui6, pkt_end @2
    SAMPLE ui5 lsb @2
    DJNZ c1, bit_loop @2          ; c1 is 7 here: always taken
    JMP bit_loop @2               ; never reached
bit_loop:
    WAIT rise ui4 @2
    JHI ui6, pkt_end @2
    SAMPLE ui5 lsb @2
    DJNZ c1, bit_loop @2
    HPUSH @2
    LDI c1, 8 @2
    WAIT rise ui4 @2
    JHI ui6, pkt_end @2
    SAMPLE ui5 lsb @2
    DJNZ c1, bit_loop @2
    JMP bit_loop @2

pkt_end:
    JHI ui7, bad @2               ; stuff violation
    MOV sr, c1 @2
    JSB sr[3]==1, good @2         ; c1 == 8 only when no partial byte is pending
bad:
    PINS toggle uo5 @2
    PINS toggle uo6 @2
    JMP start @2                  ; no response
good:
    PINS toggle uo6 @2
    WAIT high uio7 @2             ; host EOP over: D- back to J
    MOV sr, c0 @2
    XORI sr, 0xC3 @2              ; DATA0?  (sr ^ PID) + 1 == 1 only on a match
    ADDI sr, 1 @2
    MOV c1, sr @2
    DJNZ c1, try_d1 @2
    LDI c1, 0xD2 @PAD_D0          ; ACK
    JMP tx_start @2
try_d1:
    MOV sr, c0 @2
    XORI sr, 0x4B @2              ; DATA1?
    ADDI sr, 1 @2
    MOV c1, sr @2
    DJNZ c1, try_in @2
    LDI c1, 0xD2 @PAD_D1          ; ACK
    JMP tx_start @2
try_in:
    MOV sr, c0 @2
    XORI sr, 0x69 @2              ; IN?
    ADDI sr, 1 @2
    MOV c1, sr @2
    DJNZ c1, start @2             ; not a PID we answer
    LDI c1, 0x5A @PAD_IN          ; NAK

tx_start:                         ; c1 = response PID byte; bus is J (the host released it)
    PINS low uio6 @2
    PINS high uio7 @2
    LDI c0, 7 @5
sync:
    PINS toggle uio6 uio7 @26     ; SYNC edges K J K J K J K, 33 cycles apart
    DJNZ c0, sync @2
    LDI c0, 6 @2                  ; ones countdown for the data
    MOV sr, c1 @2                 ; the response byte
    JMP bit0 @47                  ; same cell timing as usb_tx.asm (HPULL slot)

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
    JMP ob @2                         ; ob: NOP commits at J+15
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
    NOP @2                        ; byte end (usb_tx.asm: DJNZ c1 here)
    PINS low uio6 uio7 @20        ; EOP: SE0
    PINS high uio7 @65            ; J after 66 cycles of SE0
    PINS float uio6 uio7 @32      ; release after one J bit time
    JMP start @2
