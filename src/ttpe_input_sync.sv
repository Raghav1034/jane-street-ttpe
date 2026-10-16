//==============================================================================
// ttpe_input_sync.sv
//
// 2-FF input synchroniser plus one extra delay stage for edge detection.
//
//   cycle X     : pad value present at pad_in
//   cycle X+1   : s1_q      = pad(X)          (first flop, may be metastable)
//   cycle X+2   : sync_q    = pad(X)          (second flop, used by the Execution Unit)
//   cycle X+3   : sync_prev = pad(X)          (previous value of sync_q, for edge detection)
//
// So sync_q(C) = pad(C-2) and sync_prev_q(C) = sync_q(C-1), exactly as in sw/ttpe_iss.py.
//
// PHYSICAL NOTES
//   * s1_q -> sync_q must be placed adjacent and must have no logic between them
//     (set a dont_touch / max-delay constraint in the flow if the tool inserts buffers).
//   * Only sync_q and sync_prev_q may be used by downstream logic. s1_q is never exposed.
//   * Async-assert active-low reset, all flops reset to 0 (same as the ISS).
//   * The synchroniser free-runs from reset. The ISS assumes it starts at 0 when the core
//     starts; the two are indistinguishable to programs because the first instruction can
//     commit no earlier than cycle 4 (see docs/EXEC_UNIT.md).
//==============================================================================

`default_nettype none

module ttpe_input_sync #(
    parameter int W = 16
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire [W-1:0] pad_in,
    output wire [W-1:0] sync_q,
    output wire [W-1:0] sync_prev_q
);

        logic [W-1:0] s1_q, s2_q, s3_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_q <= '0;
            s2_q <= '0;
            s3_q <= '0;
        end else begin
            s1_q <= pad_in;
            s2_q <= s1_q;
            s3_q <= s2_q;
        end
    end

    assign sync_q      = s2_q;
    assign sync_prev_q = s3_q;

endmodule

`default_nettype wire
