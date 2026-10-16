//==============================================================================
// ttpe_exec.sv - TTPE Execution Unit (ISA v0.2, frozen)
//
// Implements the timing contract of docs/ISA.md section 3 and sw/ttpe_iss.py exactly:
//
//   accept : in cycle A the unit takes the instruction presented by the fetch controller
//            iff valid_instruction && (unit empty || current instruction commits this
//            cycle) && no redirect (pc_load) && no halt (HALT / illegal commit, or already
//            halted).
//   commit : cycle A + 1 + D for unconditional instructions; WAIT and HPULL commit in the
//            first cycle >= A + 1 + D in which their condition holds.
//   effect : registers, pins and FIFO operations take effect at the clock edge ending the
//            commit cycle, so a pad changes in cycle commit + 1.
//   branch : pc_load is asserted combinationally in the commit cycle of a taken branch.
//   inputs : instructions read 2-FF synchronised pad values (ttpe_input_sync):
//            sync(C) = pad(C-2); WAIT edges compare sync(C) with sync(C-1).
//
// Reset state: halted (core_run low, fetch controller idle). The host loads the program,
// then pulses `resume`. All other state resets to 0, like the ISS.
//
// Host stop controls (not part of the ISA, no effect on program-visible timing while unused)
//   halt_req   : in the cycle it is applied nothing new is accepted; an instruction that
//                commits in that same cycle completes normally (registers, pins, FIFO ops);
//                at the end of the cycle the unit is halted and the held instruction is
//                discarded. Architectural state (SR, C0, C1, pins) is kept. The fetch PC is
//                undefined afterwards: the host must load a PC before the next resume.
//                Wins over resume in the same cycle. No effect on `illegal`.
//   core_reset : like halt_req and additionally clears SR, C0, C1, the pin registers
//                (all uio pins released, uo_out = 0) and `illegal`. FIFOs and the
//                synchroniser are not touched (flush the FIFOs through their own ports).
//
// Pin sources 0-7 = uio_in[0..7], 8-15 = ui_in[0..7]  (pad_in = {ui_in, uio_in}).
// Sinks 0-7 = uio[0..7], 8-15 = uo_out[0..7].
//
// PHYSICAL NOTES
//   * Longest paths: sync_q -> 16:1 source mux -> WAIT/JHI/JLO decision -> commit ->
//     (instr_accept, pc_load -> fetch controller state/PC flops) and
//     commit -> register enable. A handful of gates; far inside 20 ns in 130 nm.
//   * timer_q is a 16-bit down counter loaded with D at accept; zero detect feeds commit.
//   * Async-assert active-low reset on every flop. rst_n must be synchronised at the top.
//==============================================================================

`default_nettype none

module ttpe_exec (
    input  wire         clk,
    input  wire         rst_n,

    // pads (before synchronisation) and driven values
    input  wire [15:0]  pad_in,          // {ui_in[7:0], uio_in[7:0]}
    output wire [7:0]   uio_out,
    output wire [7:0]   uio_oe,
    output wire [7:0]   uo_out,

    // fetch controller side
    input  wire         valid_instruction,
    input  wire [31:0]  instruction,
    input  wire [8:0]   instr_pc,
    output wire         instr_accept,
    output wire         pc_load,
    output wire [8:0]   pc_load_val,

    // host FIFOs
    input  wire         tx_empty,
    input  wire [7:0]   tx_rdata,
    output wire         tx_pop,
    output wire         rx_push,
    output wire [7:0]   rx_wdata,

    // run control / status
    input  wire         resume,          // 1-cycle pulse; only honoured while halted
    input  wire         halt_req,        // 1-cycle pulse: soft halt (stops after this cycle)
    input  wire         core_reset,      // 1-cycle pulse: soft halt + clear architectural state
    output wire         halted,
    output wire         illegal,

    // debug / verification visibility (leave unconnected at chip top)
    output wire         dbg_commit,
    output wire [7:0]   dbg_sr,
    output wire [7:0]   dbg_c0,
    output wire [7:0]   dbg_c1
);

    import ttpe_isa_pkg::*;

    //--------------------------------------------------------------------------
    // State
    //--------------------------------------------------------------------------
    logic        cur_v_q;               // an instruction is held
    logic [3:0]  cur_op_q;
    logic [3:0]  cur_a_q;
    logic [7:0]  cur_p_q;
    logic        cur_pg_q;              // bit 8 of the held instruction's address
    logic [15:0] timer_q;               // remaining delta cycles

    logic [7:0]  sr_q, c0_q, c1_q;
    logic [7:0]  uio_oe_q, uio_out_q, uo_q;
    logic        halted_q, illegal_q;

    //--------------------------------------------------------------------------
    // Input synchroniser
    //--------------------------------------------------------------------------
    wire [15:0] s2, s2p;

    ttpe_input_sync #(.W(16)) u_sync (
        .clk         (clk),
        .rst_n       (rst_n),
        .pad_in      (pad_in),
        .sync_q      (s2),
        .sync_prev_q (s2p)
    );

    wire src_b  = s2 [cur_a_q];
    wire prev_b = s2p[cur_a_q];

    //--------------------------------------------------------------------------
    // Field slices as named wires (constant selects stay out of always_comb, which
    // Icarus Verilog does not fully support)
    //--------------------------------------------------------------------------
    wire        a3_w        = cur_a_q[3];
    wire        a0_w        = cur_a_q[0];
    wire [1:0]  a_lo2_w     = cur_a_q[1:0];
    wire [2:0]  a_lo3_w     = cur_a_q[2:0];
    wire [1:0]  p_lo2_w     = cur_p_q[1:0];
    wire        shout_msb_w   = cur_p_q[SHOUT_MSB_BIT];
    wire        shout_shift_w = cur_p_q[SHOUT_SHIFT_BIT];
    wire        shout_od_w    = cur_p_q[SHOUT_OD_BIT];
    wire        sample_msb_w  = cur_p_q[SAMPLE_MSB_BIT];
    wire        sr7_w       = sr_q[7];
    wire        sr0_w       = sr_q[0];
    wire [7:0]  sr_shl_w    = {sr_q[6:0], 1'b0};
    wire [7:0]  sr_shr_w    = {1'b0, sr_q[7:1]};
    wire [7:0]  smp_shl_w   = {sr_q[6:0], src_b};
    wire [7:0]  smp_shr_w   = {src_b, sr_q[7:1]};
    wire        sr_jsb_w    = sr_q[a_lo3_w];
    wire [7:0]  mask_w      = 8'd1 << a_lo3_w;

    //--------------------------------------------------------------------------
    // Legality (matches ttpe_isa.is_legal): opcodes 0xE/0xF and REG sub-ops
    // 7 and 12..15 are illegal.
    //--------------------------------------------------------------------------
    wire reg_ok = (cur_a_q <= REG_MVSR1) || ((cur_a_q >= REG_XORI) && (cur_a_q <= REG_ADDI));
    wire legal  = (cur_op_q <= OP_JSB) && ((cur_op_q != OP_REG) || reg_ok);

    //--------------------------------------------------------------------------
    // Commit decision
    //--------------------------------------------------------------------------
    logic cond_ok;

    always_comb begin
        cond_ok = 1'b1;
        if (legal) begin
            case (cur_op_q)
                OP_WAIT: begin
                    case (p_lo2_w)
                        WAIT_LOW:  cond_ok = ~src_b;
                        WAIT_HIGH: cond_ok =  src_b;
                        WAIT_FALL: cond_ok =  prev_b & ~src_b;
                        default:   cond_ok = ~prev_b &  src_b;   // WAIT_RISE
                    endcase
                end
                OP_HPULL: cond_ok = ~tx_empty;
                default:  cond_ok = 1'b1;
            endcase
        end
    end

    wire timer_zero = (timer_q == 16'd0);
    wire commit     = cur_v_q & timer_zero & cond_ok;

    //--------------------------------------------------------------------------
    // Commit effects (combinational "next value" of every architectural register)
    //--------------------------------------------------------------------------
    logic [7:0] sr_d, c0_d, c1_d, oe_d, out_d, uo_d;
    logic       taken, halt_c, ill_c;
    logic       tx_pop_c, rx_push_c;
    logic [8:0] tgt;
    logic       sbit;
    logic [7:0] m;

    always_comb begin
        sr_d  = sr_q;
        c0_d  = c0_q;
        c1_d  = c1_q;
        oe_d  = uio_oe_q;
        out_d = uio_out_q;
        uo_d  = uo_q;
        taken = 1'b0;
        halt_c = 1'b0;
        ill_c  = 1'b0;
        tx_pop_c  = 1'b0;
        rx_push_c = 1'b0;
        tgt   = {cur_pg_q, cur_p_q};
        sbit  = shout_msb_w ? sr7_w : sr0_w;
        m     = mask_w;

        if (commit) begin
            if (!legal) begin
                ill_c  = 1'b1;
                halt_c = 1'b1;
            end else begin
                case (cur_op_q)
                    OP_NOP: ;

                    OP_PINS: begin
                        if (!a3_w) begin
                            case (a_lo2_w)
                                PINS_LOW: begin
                                    oe_d  = uio_oe_q  |  cur_p_q;
                                    out_d = uio_out_q & ~cur_p_q;
                                end
                                PINS_HIGH: begin
                                    oe_d  = uio_oe_q  | cur_p_q;
                                    out_d = uio_out_q | cur_p_q;
                                end
                                PINS_FLOAT: begin
                                    oe_d  = uio_oe_q & ~cur_p_q;
                                end
                                default: begin   // PINS_TOGGLE
                                    oe_d  = uio_oe_q  | cur_p_q;
                                    out_d = uio_out_q ^ cur_p_q;
                                end
                            endcase
                        end else begin
                            case (a_lo2_w)
                                PINS_LOW:    uo_d = uo_q & ~cur_p_q;
                                PINS_HIGH:   uo_d = uo_q |  cur_p_q;
                                PINS_TOGGLE: uo_d = uo_q ^  cur_p_q;
                                default:     ;   // float on the dedicated outputs: no effect
                            endcase
                        end
                    end

                    OP_WAIT: ;   // condition handled in cond_ok, no state change

                    OP_JMP: begin
                        taken = 1'b1;
                        tgt   = {a0_w, cur_p_q};
                    end

                    OP_JHI: taken =  src_b;
                    OP_JLO: taken = ~src_b;

                    OP_SHOUT: begin
                        if (!a3_w) begin
                            if (shout_od_w) begin
                                out_d = uio_out_q & ~m;
                                oe_d  = sbit ? (uio_oe_q & ~m) : (uio_oe_q | m);
                            end else begin
                                oe_d  = uio_oe_q | m;
                                out_d = sbit ? (uio_out_q | m) : (uio_out_q & ~m);
                            end
                        end else begin
                            uo_d = sbit ? (uo_q | m) : (uo_q & ~m);
                        end
                        if (shout_shift_w) begin
                            sr_d = shout_msb_w ? sr_shl_w : sr_shr_w;
                        end
                    end

                    OP_SAMPLE: begin
                        sr_d = sample_msb_w ? smp_shl_w : smp_shr_w;
                    end

                    OP_REG: begin
                        case (cur_a_q)
                            REG_LDSR:  sr_d = cur_p_q;
                            REG_LDC0:  c0_d = cur_p_q;
                            REG_LDC1:  c1_d = cur_p_q;
                            REG_MVC0:  c0_d = sr_q;
                            REG_MVC1:  c1_d = sr_q;
                            REG_MVSR0: sr_d = c0_q;
                            REG_MVSR1: sr_d = c1_q;
                            REG_XORI:  sr_d = sr_q ^ cur_p_q;
                            REG_ANDI:  sr_d = sr_q & cur_p_q;
                            REG_ORI:   sr_d = sr_q | cur_p_q;
                            REG_ADDI:  sr_d = sr_q + cur_p_q;
                            default:   ;   // unreachable: legal() excludes the rest
                        endcase
                    end

                    OP_DJNZ: begin
                        if (a0_w) begin
                            c1_d  = c1_q - 8'd1;
                            taken = (c1_q != 8'd1);
                        end else begin
                            c0_d  = c0_q - 8'd1;
                            taken = (c0_q != 8'd1);
                        end
                    end

                    OP_HPUSH: rx_push_c = 1'b1;

                    OP_HPULL: begin
                        tx_pop_c = 1'b1;
                        sr_d     = tx_rdata;
                    end

                    OP_HALT: halt_c = 1'b1;

                    OP_JSB: taken = (sr_jsb_w == a3_w);

                    default: ;   // unreachable: legal() excludes 0xE/0xF
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // Fetch handshake
    //--------------------------------------------------------------------------
    wire can_take = ~cur_v_q | commit;

    assign pc_load      = commit & taken;
    assign pc_load_val  = tgt;
    assign instr_accept = valid_instruction & can_take & ~pc_load & ~halt_c & ~halted_q
                        & ~halt_req & ~core_reset;

    assign tx_pop   = tx_pop_c;
    assign rx_push  = rx_push_c;
    assign rx_wdata = sr_q;

    //--------------------------------------------------------------------------
    // Sequential logic
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cur_v_q   <= 1'b0;
            cur_op_q  <= 4'd0;
            cur_a_q   <= 4'd0;
            cur_p_q   <= 8'd0;
            cur_pg_q  <= 1'b0;
            timer_q   <= 16'd0;
            sr_q      <= 8'd0;
            c0_q      <= 8'd0;
            c1_q      <= 8'd0;
            uio_oe_q  <= 8'd0;
            uio_out_q <= 8'd0;
            uo_q      <= 8'd0;
            halted_q  <= 1'b1;
            illegal_q <= 1'b0;
        end else begin
            // current-instruction register and delta countdown
            if (instr_accept) begin
                cur_v_q  <= 1'b1;
                cur_op_q <= instruction[31:28];
                cur_a_q  <= instruction[27:24];
                cur_p_q  <= instruction[23:16];
                cur_pg_q <= instr_pc[8];
                timer_q  <= instruction[15:0];
            end else if (commit) begin
                cur_v_q  <= 1'b0;
            end else if (cur_v_q && !timer_zero) begin
                timer_q  <= timer_q - 16'd1;
            end

            // architectural registers (the *_d values equal the old value unless committing)
            sr_q      <= sr_d;
            c0_q      <= c0_d;
            c1_q      <= c1_d;
            uio_oe_q  <= oe_d;
            uio_out_q <= out_d;
            uo_q      <= uo_d;

            // run state. resume is only meaningful while halted, where nothing can commit.
            if (resume) begin
                halted_q  <= 1'b0;
                illegal_q <= 1'b0;
            end else begin
                if (halt_c) halted_q  <= 1'b1;
                if (ill_c)  illegal_q <= 1'b1;
            end

            // host stop controls override everything above (including resume)
            if (halt_req | core_reset) begin
                halted_q <= 1'b1;
                cur_v_q  <= 1'b0;
            end
            if (core_reset) begin
                sr_q      <= 8'd0;
                c0_q      <= 8'd0;
                c1_q      <= 8'd0;
                uio_oe_q  <= 8'd0;
                uio_out_q <= 8'd0;
                uo_q      <= 8'd0;
                illegal_q <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Outputs
    //--------------------------------------------------------------------------
    assign uio_out = uio_out_q;
    assign uio_oe  = uio_oe_q;
    assign uo_out  = uo_q;
    assign halted  = halted_q;
    assign illegal = illegal_q;

    assign dbg_commit = commit;
    assign dbg_sr     = sr_q;
    assign dbg_c0     = c0_q;
    assign dbg_c1     = c1_q;

endmodule

`default_nettype wire
