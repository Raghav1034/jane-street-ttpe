//==============================================================================
// ttpe_reset_sync.sv - reset synchroniser: asynchronous assert, synchronous release.
//
// rst_n_out falls immediately (no clock needed) when rst_n_in falls and rises two clock
// edges after rst_n_in rises. Every other import ttpe_isa_pkg::*;
module in the design assumes its rst_n is already
// synchronised to clk, so this is instantiated once at the chip top.
//
// The two flops reset to 0, so the reset state is defined without a clock.
//==============================================================================

`default_nettype none

module ttpe_reset_sync (
    input  wire clk,
    input  wire rst_n_in,
    output wire rst_n_out
);

    logic q1_q, q2_q;

    always_ff @(posedge clk or negedge rst_n_in) begin
        if (!rst_n_in) begin
            q1_q <= 1'b0;
            q2_q <= 1'b0;
        end else begin
            q1_q <= 1'b1;
            q2_q <= q1_q;
        end
    end

    assign rst_n_out = q2_q;

endmodule

`default_nettype wire
