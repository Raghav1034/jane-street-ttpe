// sram_512x32_bb.v
//
// Black-box declaration of the IHP SRAM hard macro for SYNTHESIS (yosys / LibreLane).
// The real macro is supplied to the flow through the macro configuration (LEF, GDS,
// liberty). This stub only tells the synthesiser the port list.
//
// SIMULATION: compile with -DSIM. The stub then disappears and the behavioural models are
// used instead:
//   test/ihp_sram/RM_IHPSG13_1P_core_behavioral_bm_bist.v
//   test/ihp_sram/RM_IHPSG13_1P_512x32_c2_bm_bist.v      (compile with -DSIM -DFUNCTIONAL)
//   sim/RM_IHPSG13_1P_512x32_c2_bm_bist_sim.v            (stand-in with extra checks)
// Never put this file and a behavioural model in the same simulation.
//
// Port directions follow rtl/ttpe_mem_subsys.sv (the only place the macro is instantiated).

`ifndef SIM
(* blackbox *)
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  wire        A_CLK,
    input  wire        A_MEN,
    input  wire        A_WEN,
    input  wire        A_REN,
    input  wire [8:0]  A_ADDR,
    input  wire [31:0] A_DIN,
    input  wire [31:0] A_BM,
    output wire [31:0] A_DOUT,
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
endmodule
`endif
