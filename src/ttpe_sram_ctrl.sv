//==============================================================================
// ttpe_sram_ctrl.sv
//
// TTPE SRAM memory controller + instruction fetch pipeline.
//
// Target   : IHP SG13G2 130nm via Tiny Tapeout, single clock domain (~50 MHz)
// Macro    : RM_IHPSG13_1P_512x32_c2_bm_bist  (512 x 32, single port)
//
//------------------------------------------------------------------------------
// WHAT THIS MODULE DOES
//   1. Arbitrates the single SRAM port between
//        Priority 1 : SPI host interface  (read / byte-masked write)
//        Priority 2 : TTPE core PC fetch  (read only)
//   2. Tracks the 1-cycle SRAM read latency for both requesters and captures
//      A_DOUT into a dedicated register (never consumed combinationally).
//   3. Runs the fetch FSM  HALT -> FETCH -> WAIT_MEM -> EXEC  and presents
//      the fetched word to the Execution Unit with `valid_instruction`.
//
//------------------------------------------------------------------------------
// CYCLE-LEVEL TIMING CONTRACT  (cycle T = the cycle a request is GRANTED)
//
//   cycle T   : request granted; A_MEN/A_REN/A_ADDR driven (from flops through
//               a small mux). The macro samples them at the rising edge that
//               ENDS cycle T.
//   cycle T+1 : A_DOUT is valid (macro read latency = 1 cycle). It is captured
//               by a controller flop at the edge that ENDS cycle T+1.
//   cycle T+2 : captured data is visible on host_rdata + host_rvalid
//               (host read) or on `instruction` + valid_instruction (fetch).
//
//   => grant-to-data latency is exactly 2 cycles for both requesters.
//   => Host reads are fully pipelined (1 per cycle).
//   => host write: host_wdone pulses in cycle T+1.
//   => Fetch: Execution Unit accepts in cycle A (valid_instruction &
//      instr_accept). The next instruction is valid in cycle A+3 at the
//      earliest (FETCH at A+1, WAIT_MEM at A+2, EXEC at A+3), with no host
//      interference. The fetch of instruction N+1 therefore overlaps the
//      time-delta countdown of instruction N inside the Execution Unit.
//
//------------------------------------------------------------------------------
// ASSUMPTIONS THAT MUST BE VERIFIED AGAINST THE PDK MACRO FILES
//   (the project brief does not specify them, so they are NOT invented here)
//   [A1] Macro has a bit-mask input A_BM[31:0]  (the "bm" in the macro name).
//        Polarity is selected by BM_WRITE_LEVEL (1 = bit is written when the
//        mask bit is 1). Check the macro .v / datasheet and set accordingly.
//   [A2] Macro control inputs A_MEN / A_REN / A_WEN are active-high.
//   [A3] "1-cycle read latency" is interpreted as: read sampled at edge N,
//        A_DOUT stable for capture at edge N+1.
//   [A4] BIST port names are as listed below; all are tied to 0 here.
//
//------------------------------------------------------------------------------
// PHYSICAL NOTES
//   * All state and data registers have an ASYNCHRONOUS-assert active-low
//     reset. rst_n must already be synchronised to `clk` (async assert,
//     sync de-assert) at the chip top: this import ttpe_isa_pkg::*;
module does not contain the
//     reset synchroniser.
//   * Macro inputs come from flops through a single mux level (host_gnt
//     select). At 20 ns period this is far inside budget; no extra output
//     register stage is needed for macro setup timing.
//   * A_DOUT is only ever sampled by flops (instr_q, host_rdata_q). It is
//     never used combinationally, so the macro access time is the only path
//     constraint on it.
//   * valid_instruction and core_halted are direct flop outputs (one-hot
//     state bits) so they are glitch-free for the downstream block.
//   * A_CLK is a plain feed-through of clk; CTS must treat the macro clock
//     pin as a normal sink of the same clock tree.
//   * Data registers (instr_q, host_rdata_q) are reset as required by the
//     project rules. If area becomes critical, they are the first candidates
//     for removing reset (their content is never used while valid is low).
//
//------------------------------------------------------------------------------
// BEHAVIOURAL CORNER CASES (all covered by test/test_ttpe_mem.py)
//   * pc_load   : highest priority. Loads PC, aborts any in-flight fetch and
//                 drops valid_instruction. Works in every state, including
//                 HALT (that is how the host sets the start address).
//   * core_run=0: returns to HALT from any state; in-flight read data is
//                 discarded. If an instruction is accepted in the same cycle,
//                 it IS consumed (PC still increments), so resume does not
//                 re-execute it.
//   * instr_accept is ignored unless the FSM is in EXEC.
//   * PC is 9 bits and wraps from 511 to 0.
//   * host_block=1 stops host grants (determinism guard: top level should
//     drive it while the core is running unless host access is wanted).
//     A blocked host must keep host_req asserted and retry; host_gnt is the
//     handshake.
//==============================================================================

`default_nettype none

module ttpe_sram_ctrl #(
    // [A1] Level of an A_BM bit that makes the macro WRITE that bit.
    parameter [0:0] BM_WRITE_LEVEL = 1'b1
) (
    input  wire         clk,
    input  wire         rst_n,            // active low, already synchronised

    //--------------------------------------------------------------------------
    // Host port (Priority 1). Transfer happens in a cycle where
    // host_req && host_gnt. Payload must be stable while host_req is high.
    //--------------------------------------------------------------------------
    input  wire         host_req,
    input  wire         host_we,          // 1 = write, 0 = read
    input  wire [8:0]   host_addr,        // word address
    input  wire [31:0]  host_wdata,
    input  wire [3:0]   host_wstrb,       // byte strobes, [0] = bits 7:0
    input  wire         host_block,       // 1 = host may not access the SRAM
    output wire         host_gnt,         // comb: host_req & ~host_block
    output wire         host_wdone,       // pulse, 1 cycle after a granted write
    output wire         host_rvalid,      // pulse, 2 cycles after a granted read
    output wire [31:0]  host_rdata,       // valid while host_rvalid

    //--------------------------------------------------------------------------
    // TTPE core port (Priority 2)
    //--------------------------------------------------------------------------
    input  wire         core_run,         // 1 = execute, 0 = halt
    input  wire         pc_load,          // 1-cycle: load PC with pc_load_val
    input  wire [8:0]   pc_load_val,
    input  wire         instr_accept,     // Exec Unit takes the presented word
    output wire         valid_instruction,// registered, high in EXEC state
    output wire [31:0]  instruction,      // stable while valid_instruction
    output wire [8:0]   instr_pc,         // address of `instruction`
    output wire         core_fetch_stall, // core wanted the SRAM but host won
    output wire         core_halted,      // registered, high in HALT state

    //--------------------------------------------------------------------------
    // IHP SRAM macro interface
    //--------------------------------------------------------------------------
    output wire         A_CLK,
    output wire         A_MEN,
    output wire         A_WEN,
    output wire         A_REN,
    output wire [8:0]   A_ADDR,
    output wire [31:0]  A_DIN,
    output wire [31:0]  A_BM,
    input  wire [31:0]  A_DOUT,
    output wire         A_DLY,
    output wire         A_BIST_CLK,
    output wire         A_BIST_EN,
    output wire         A_BIST_MEN,
    output wire         A_BIST_WEN,
    output wire         A_BIST_REN,
    output wire [8:0]   A_BIST_ADDR,
    output wire [31:0]  A_BIST_DIN,
    output wire [31:0]  A_BIST_BM
);

    //==========================================================================
    // Fetch FSM encoding (explicit one-hot; each state bit is a flop output)
    //==========================================================================
    localparam [3:0] ST_HALT     = 4'b0001;  // core stopped, host owns SRAM
    localparam [3:0] ST_FETCH    = 4'b0010;  // requesting mem[pc]
    localparam [3:0] ST_WAIT_MEM = 4'b0100;  // read in flight, A_DOUT valid now
    localparam [3:0] ST_EXEC     = 4'b1000;  // instruction presented

    reg [3:0]  state_q, state_d;
    reg [8:0]  pc_q;
    reg [31:0] instr_q;

    // Host read/write tracking (pipeline of the grant)
    reg        host_rd_pend_q;   // T+1 : host read data is on A_DOUT
    reg        host_rvalid_q;    // T+2 : host_rdata_q holds the data
    reg        host_wdone_q;     // T+1 : write has been committed to the macro
    reg [31:0] host_rdata_q;

    //==========================================================================
    // Arbiter (combinational, driven only from flops and module inputs)
    //==========================================================================
    wire core_req     = state_q[1];                  // ST_FETCH
    wire host_gnt_w   = host_req & ~host_block;      // Priority 1
    wire core_gnt     = core_req & ~host_gnt_w;      // Priority 2

    wire host_rd_gnt  = host_gnt_w & ~host_we;
    wire host_wr_gnt  = host_gnt_w &  host_we;

    assign host_gnt         = host_gnt_w;
    assign core_fetch_stall = core_req & host_gnt_w;

    //==========================================================================
    // SRAM port drive
    //==========================================================================
    // Byte strobes -> bit mask. Mask is zero (inactive) when not writing.
    wire [31:0] wr_mask = host_wr_gnt
                        ? { {8{host_wstrb[3]}}, {8{host_wstrb[2]}},
                            {8{host_wstrb[1]}}, {8{host_wstrb[0]}} }
                        : 32'h0000_0000;

    assign A_CLK  = clk;
    assign A_MEN  = host_gnt_w | core_gnt;
    assign A_REN  = host_rd_gnt | core_gnt;
    assign A_WEN  = host_wr_gnt;
    assign A_ADDR = host_gnt_w ? host_addr : pc_q;
    assign A_DIN  = host_wdata;
    assign A_BM   = BM_WRITE_LEVEL ? wr_mask : ~wr_mask;

    // Required tie-offs
    assign A_DLY       = 1'b1;
    assign A_BIST_CLK  = 1'b0;
    assign A_BIST_EN   = 1'b0;
    assign A_BIST_MEN  = 1'b0;
    assign A_BIST_WEN  = 1'b0;
    assign A_BIST_REN  = 1'b0;
    assign A_BIST_ADDR = 9'd0;
    assign A_BIST_DIN  = 32'd0;
    assign A_BIST_BM   = 32'd0;

    //==========================================================================
    // Fetch FSM next-state logic
    //
    // Priority inside every state: !core_run (halt) > pc_load (flush) > normal.
    // A fetch granted in the same cycle as a flush/halt is harmless: the macro
    // read happens, but the FSM never enters WAIT_MEM, so the data is dropped.
    //==========================================================================
    always_comb begin
        state_d = state_q;
        case (state_q)
            ST_HALT: begin
                if (core_run) state_d = ST_FETCH;
            end

            ST_FETCH: begin
                if      (!core_run) state_d = ST_HALT;
                else if (pc_load)   state_d = ST_FETCH;     // new PC, retry
                else if (core_gnt)  state_d = ST_WAIT_MEM;  // read issued
                else                state_d = ST_FETCH;     // lost to host
            end

            ST_WAIT_MEM: begin
                if      (!core_run) state_d = ST_HALT;
                else if (pc_load)   state_d = ST_FETCH;     // discard data
                else                state_d = ST_EXEC;      // data captured now
            end

            ST_EXEC: begin
                if      (!core_run)     state_d = ST_HALT;
                else if (pc_load)       state_d = ST_FETCH; // jump
                else if (instr_accept)  state_d = ST_FETCH; // prefetch next
                else                    state_d = ST_EXEC;  // hold
            end

            default: state_d = ST_HALT;  // illegal-state recovery
        endcase
    end

    //==========================================================================
    // Sequential logic. Async-assert active-low reset on every register.
    //==========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q        <= ST_HALT;
            pc_q           <= 9'd0;
            instr_q        <= 32'd0;
            host_rd_pend_q <= 1'b0;
            host_rvalid_q  <= 1'b0;
            host_wdone_q   <= 1'b0;
            host_rdata_q   <= 32'd0;
        end else begin
            state_q <= state_d;

            // PC: load beats increment. An accepted instruction is consumed
            // even if core_run drops in the same cycle.
            if (pc_load)
                pc_q <= pc_load_val;
            else if (state_q == ST_EXEC && instr_accept)
                pc_q <= pc_q + 9'd1;

            // Core read data: A_DOUT is valid during WAIT_MEM. Captured
            // unconditionally in that state; only presented if the FSM then
            // reaches EXEC.
            if (state_q == ST_WAIT_MEM)
                instr_q <= A_DOUT;

            // Host read pipeline
            host_rd_pend_q <= host_rd_gnt;
            host_rvalid_q  <= host_rd_pend_q;
            if (host_rd_pend_q)
                host_rdata_q <= A_DOUT;

            // Host write completion
            host_wdone_q <= host_wr_gnt;
        end
    end

    //==========================================================================
    // Output assignments
    //==========================================================================
    assign valid_instruction = state_q[3];   // ST_EXEC (flop output)
    assign core_halted       = state_q[0];   // ST_HALT (flop output)
    assign instruction       = instr_q;
    assign instr_pc          = pc_q;

    assign host_rvalid       = host_rvalid_q;
    assign host_rdata        = host_rdata_q;
    assign host_wdone        = host_wdone_q;

endmodule

`default_nettype wire
