//==============================================================================
// RM_IHPSG13_1P_512x32_c2_bm_bist_sim.v
//
// SIMULATION-ONLY behavioural stand-in for the IHP SRAM hard macro.
// NOT for synthesis. It is NOT the vendor model: it implements only the
// behaviour the controller relies on, as described in the project brief:
//   * 512 x 32, single port, synchronous
//   * read  : A_ADDR/A_REN/A_MEN sampled at posedge A_CLK, A_DOUT updates
//             right after that edge (so it is capturable at the next edge)
//   * write : bit-masked, a bit is written where A_BM is 1   [assumption A1]
// Before sign-off, re-run the testbench against the PDK-provided Verilog model.
//
// Extra checking added for verification:
//   * A_DOUT is driven to X in every cycle that is not a read, so any
//     controller path that consumes A_DOUT at the wrong time shows up as X.
//   * Violation counters (read by cocotb):
//       tie_violations : A_DLY != 1 or any BIST input != 0 at a clock edge
//       rw_violations  : A_REN and A_WEN asserted together
//       x_violations   : X/Z on A_MEN/A_REN/A_WEN, or on A_ADDR during access
//==============================================================================

`timescale 1ns/1ps

module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire [8:0]  A_ADDR,
    input  wire [31:0] A_DIN,
    input  wire [31:0] A_BM,
    output reg  [31:0] A_DOUT,
    input  wire        A_DLY,
    input  wire        A_BIST_CLK,
    input  wire        A_BIST_EN,
    input  wire        A_BIST_MEN,
    input  wire        A_BIST_WEN,
    input  wire        A_BIST_REN,
    input  wire [8:0]  A_BIST_ADDR,
    input  wire [31:0] A_BIST_DIN,
    input  wire [31:0] A_BIST_BM
);

    reg [31:0] mem [0:511];

    integer tie_violations;
    integer rw_violations;
    integer x_violations;

    initial begin
        tie_violations = 0;
        rw_violations  = 0;
        x_violations   = 0;
        A_DOUT         = {32{1'bx}};
    end

    always @(posedge A_CLK) begin
        // Tie-off check
        if (A_DLY !== 1'b1 ||
            A_BIST_CLK !== 1'b0 || A_BIST_EN  !== 1'b0 ||
            A_BIST_MEN !== 1'b0 || A_BIST_WEN !== 1'b0 || A_BIST_REN !== 1'b0 ||
            A_BIST_ADDR !== 9'd0 || A_BIST_DIN !== 32'd0 || A_BIST_BM !== 32'd0)
            tie_violations = tie_violations + 1;

        // Unknown control check
        if ((^{A_MEN, A_REN, A_WEN}) === 1'bx)
            x_violations = x_violations + 1;

        if (A_MEN === 1'b1) begin
            if (A_WEN === 1'b1 && A_REN === 1'b1)
                rw_violations = rw_violations + 1;

            if ((A_WEN === 1'b1 || A_REN === 1'b1) && ((^A_ADDR) === 1'bx))
                x_violations = x_violations + 1;

            if (A_WEN === 1'b1)
                mem[A_ADDR] <= (mem[A_ADDR] & ~A_BM) | (A_DIN & A_BM);

            if (A_REN === 1'b1)
                A_DOUT <= mem[A_ADDR];
            else
                A_DOUT <= {32{1'bx}};
        end else begin
            A_DOUT <= {32{1'bx}};
        end
    end

endmodule
