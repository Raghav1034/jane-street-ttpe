//==============================================================================
// ttpe_usb_rx.sv - low-speed USB (1.5 Mbit/s) bit slicer for the TTPE (proposal, Phase D)
//
// Cycle-exact transcription of usb_rx_model() in sw/usb_ls.py; that function and docs/USB_LS_RX.md
// are the specification. Any change here must be made in the model first.
//
// Inputs are the SYNCHRONISED pad values (the engine's s2 / s3 flops, see ttpe_input_sync):
//   dp_q      = s2[6]  D+ (uio6) in cycle C = pad(C-2)
//   dp_prev_q = s3[6]  D+ one cycle earlier
//   dm_q      = s2[7]  D- (uio7), used only for SE0
// Output usb_d[3:0] = {ERR, STOP, DATA, TICK} is the NEXT value of the virtual source pins
// ui7..ui4 (= source pins 15..12). ttpe_input_sync loads it into the D input of s2[15:12]
// when `en` is high, so a value computed in cycle C is seen by instructions committing in
// cycle C+1 and WAIT edges compare it with the value of cycle C through s3, exactly like a pad.
//
//   TICK (ui4) 1-cycle pulse per decoded data bit (NRZI decoded, stuffed bits removed) and one at
//              the packet end. A stuffed bit produces no TICK.
//   DATA (ui5) last decoded bit, held until the next TICK.
//   STOP (ui6) set at the packet-end TICK (EOP or stuff error); held until the next SYNC start.
//   ERR  (ui7) set together with STOP after a bit-stuff violation; cleared with STOP.
//
// en = 0: the state machine is held in IDLE with DATA/STOP/ERR = 0 and usb_d is ignored by the
// synchroniser, so the block has no effect on any program.
//
// Behaviour (see the model for the exact definition):
//   IDLE   : a rising edge of D+ (J to K) starts SYNC; the cell counter restarts there.
//   cell   : counter restarted by every D+ edge, free-running rollover after 33, 33, 34 cycles
//            (index 0,1,2); an edge after the cell was sampled advances the index.
//   sample : once per cell when the counter reaches 16.
//   SYNC   : the first 8 sampled levels must be 1 0 1 0 1 0 1 1 and not SE0, otherwise LOCKOUT
//            (silent). After the 8th the state is RUN with prev = last SYNC level.
//   RUN    : SE0 -> TICK+STOP, IDLE.  Otherwise bit = (D+ == prev); after six 1s the next cell is
//            the stuffed 0 (no TICK; a 1 there is a stuff violation: TICK+STOP+ERR, LOCKOUT).
//   LOCKOUT: ignores the bus until 264 cycles without a D+ edge, then IDLE.
//
// State: st 2, cnt 6, idx 2, smp 1, ncell 3, ones 3, prev 1, quiet 9, data/stop/err 3 = 30 flops.
// Async-assert active-low reset on every flop, like the rest of the design.
//
// NOT simulated or synthesised at the time of writing (no HDL tools were available in the
// authoring session): run `make -C test TARGET=usb` (test/test_ttpe_usb.py) first.
//==============================================================================

`default_nettype none

module ttpe_usb_rx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        dp_q,
    input  wire        dp_prev_q,
    input  wire        dm_q,
    output wire [3:0]  usb_d
);

    localparam logic [1:0] S_IDLE = 2'd0;
    localparam logic [1:0] S_SYNC = 2'd1;
    localparam logic [1:0] S_RUN  = 2'd2;
    localparam logic [1:0] S_LOCK = 2'd3;

    localparam logic [5:0] CELL_SHORT = 6'd33;   // cells 0 and 1 of the 33/33/34 pattern
    localparam logic [5:0] CELL_LONG  = 6'd34;   // cell 2
    localparam logic [5:0] SAMPLE_AT  = 6'd16;
    localparam logic [8:0] LOCK_CYC   = 9'd264;  // 8 cells x 33 cycles without a D+ edge

    //--------------------------------------------------------------------------
    // State
    //--------------------------------------------------------------------------
    logic [1:0] st_q;
    logic [5:0] cnt_q;
    logic [1:0] idx_q;
    logic       smp_q;
    logic [2:0] ncell_q;
    logic [2:0] ones_q;
    logic       prev_q;
    logic [8:0] quiet_q;
    logic       data_q, stop_q, err_q;

    //--------------------------------------------------------------------------
    // Next state
    //--------------------------------------------------------------------------
    logic [1:0] st_d;
    logic [5:0] cnt_d;
    logic [1:0] idx_d;
    logic       smp_d;
    logic [2:0] ncell_d;
    logic [2:0] ones_d;
    logic       prev_d;
    logic [8:0] quiet_d;
    logic       data_d, stop_d, err_d, tick_d;

    wire        edge_w     = dp_q ^ dp_prev_q;
    wire        se0_w      = ~dp_q & ~dm_q;
    wire [5:0]  cnt_inc_w  = cnt_q + 6'd1;
    wire [1:0]  idx_inc_w  = (idx_q == 2'd2) ? 2'd0 : (idx_q + 2'd1);
    wire [5:0]  cell_len_w = (idx_q == 2'd2) ? CELL_LONG : CELL_SHORT;
    wire [8:0]  quiet_inc_w = quiet_q + 9'd1;
    wire        ncell0_w   = ncell_q[0];
    wire        ncell_last_w = (ncell_q == 3'd7);
    wire        sync_exp_w = ncell_last_w ? 1'b1 : ~ncell0_w;   // 1 0 1 0 1 0 1 1
    wire        bit_w      = ~(dp_q ^ prev_q);                  // NRZI: 1 = no transition
    wire        ones_six_w = (ones_q == 3'd6);
    wire [2:0]  ones_inc_w = ones_q + 3'd1;

    always_comb begin
        st_d    = st_q;
        cnt_d   = cnt_q;
        idx_d   = idx_q;
        smp_d   = smp_q;
        ncell_d = ncell_q;
        ones_d  = ones_q;
        prev_d  = prev_q;
        quiet_d = quiet_q;
        data_d  = data_q;
        stop_d  = stop_q;
        err_d   = err_q;
        tick_d  = 1'b0;

        if (!en) begin
            st_d   = S_IDLE;
            data_d = 1'b0;
            stop_d = 1'b0;
            err_d  = 1'b0;
        end else if (st_q == S_IDLE) begin
            if (edge_w & dp_q) begin
                st_d    = S_SYNC;
                cnt_d   = 6'd0;
                idx_d   = 2'd0;
                ncell_d = 3'd0;
                ones_d  = 3'd0;
                smp_d   = 1'b0;
                stop_d  = 1'b0;
                err_d   = 1'b0;
            end
        end else if (st_q == S_LOCK) begin
            quiet_d = edge_w ? 9'd0 : quiet_inc_w;
            if (quiet_d >= LOCK_CYC) st_d = S_IDLE;
        end else begin
            // S_SYNC or S_RUN: advance the cell counter
            if (edge_w) begin
                if (smp_q) begin
                    idx_d = idx_inc_w;
                    smp_d = 1'b0;
                end
                cnt_d = 6'd0;
            end else if (cnt_inc_w == cell_len_w) begin
                cnt_d = 6'd0;
                idx_d = idx_inc_w;
                smp_d = 1'b0;
            end else begin
                cnt_d = cnt_inc_w;
            end

            // sample point (an edge makes cnt_d zero, so no extra edge term is needed)
            if (cnt_d == SAMPLE_AT) begin
                smp_d = 1'b1;
                if (st_q == S_SYNC) begin
                    if ((dp_q != sync_exp_w) | se0_w) begin
                        st_d    = S_LOCK;
                        quiet_d = 9'd0;
                    end else begin
                        ncell_d = ncell_q + 3'd1;
                        if (ncell_last_w) begin
                            st_d   = S_RUN;
                            prev_d = dp_q;
                            ones_d = 3'd0;
                        end
                    end
                end else begin
                    if (se0_w) begin
                        stop_d = 1'b1;
                        tick_d = 1'b1;
                        st_d   = S_IDLE;
                    end else begin
                        prev_d = dp_q;
                        if (ones_six_w) begin
                            ones_d = 3'd0;
                            if (bit_w) begin          // stuff violation
                                stop_d  = 1'b1;
                                err_d   = 1'b1;
                                tick_d  = 1'b1;
                                st_d    = S_LOCK;
                                quiet_d = 9'd0;
                            end
                        end else begin
                            ones_d = bit_w ? ones_inc_w : 3'd0;
                            data_d = bit_w;
                            tick_d = 1'b1;
                        end
                    end
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st_q    <= S_IDLE;
            cnt_q   <= 6'd0;
            idx_q   <= 2'd0;
            smp_q   <= 1'b0;
            ncell_q <= 3'd0;
            ones_q  <= 3'd0;
            prev_q  <= 1'b0;
            quiet_q <= 9'd0;
            data_q  <= 1'b0;
            stop_q  <= 1'b0;
            err_q   <= 1'b0;
        end else begin
            st_q    <= st_d;
            cnt_q   <= cnt_d;
            idx_q   <= idx_d;
            smp_q   <= smp_d;
            ncell_q <= ncell_d;
            ones_q  <= ones_d;
            prev_q  <= prev_d;
            quiet_q <= quiet_d;
            data_q  <= data_d;
            stop_q  <= stop_d;
            err_q   <= err_d;
        end
    end

    // {ERR, STOP, DATA, TICK} -> s2[15:12] = ui7..ui4
    assign usb_d = {err_d, stop_d, data_d, tick_d};

endmodule

`default_nettype wire
