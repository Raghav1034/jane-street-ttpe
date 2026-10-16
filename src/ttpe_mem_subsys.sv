//==============================================================================
// ttpe_mem_subsys.sv
//
// Memory subsystem wrapper: ttpe_sram_ctrl + the IHP SRAM hard macro.
//
// This is the only place the macro is instantiated, so if the macro port list
// differs from the one used here (see assumptions [A1]-[A4] in
// ttpe_sram_ctrl.sv), this is the only file that needs to change.
//
// SYNTHESIS / PD FLOW:
//   * Do NOT add sim/RM_IHPSG13_1P_512x32_c2_bm_bist_sim.v to the synthesis
//     file list. In the flow the macro is a black box supplied through the
//     LibreLane macro configuration (Verilog stub, LEF, GDS, liberty).
//   * Instance name u_sram is the instance the macro placement / PDN /
//     routing-obstruction settings in config.json must refer to.
//==============================================================================

`default_nettype none

module ttpe_mem_subsys #(
    parameter [0:0] BM_WRITE_LEVEL = 1'b1
) (
    input  wire         clk,
    input  wire         rst_n,

    // Host port
    input  wire         host_req,
    input  wire         host_we,
    input  wire [8:0]   host_addr,
    input  wire [31:0]  host_wdata,
    input  wire [3:0]   host_wstrb,
    input  wire         host_block,
    output wire         host_gnt,
    output wire         host_wdone,
    output wire         host_rvalid,
    output wire [31:0]  host_rdata,

    // Core port
    input  wire         core_run,
    input  wire         pc_load,
    input  wire [8:0]   pc_load_val,
    input  wire         instr_accept,
    output wire         valid_instruction,
    output wire [31:0]  instruction,
    output wire [8:0]   instr_pc,
    output wire         core_fetch_stall,
    output wire         core_halted
);

    wire        a_clk;
    wire        a_men, a_wen, a_ren;
    wire [8:0]  a_addr;
    wire [31:0] a_din, a_bm, a_dout;
    wire        a_dly;
    wire        a_bist_clk, a_bist_en, a_bist_men, a_bist_wen, a_bist_ren;
    wire [8:0]  a_bist_addr;
    wire [31:0] a_bist_din, a_bist_bm;

    ttpe_sram_ctrl #(
        .BM_WRITE_LEVEL (BM_WRITE_LEVEL)
    ) u_ctrl (
        .clk               (clk),
        .rst_n             (rst_n),
        .host_req          (host_req),
        .host_we           (host_we),
        .host_addr         (host_addr),
        .host_wdata        (host_wdata),
        .host_wstrb        (host_wstrb),
        .host_block        (host_block),
        .host_gnt          (host_gnt),
        .host_wdone        (host_wdone),
        .host_rvalid       (host_rvalid),
        .host_rdata        (host_rdata),
        .core_run          (core_run),
        .pc_load           (pc_load),
        .pc_load_val       (pc_load_val),
        .instr_accept      (instr_accept),
        .valid_instruction (valid_instruction),
        .instruction       (instruction),
        .instr_pc          (instr_pc),
        .core_fetch_stall  (core_fetch_stall),
        .core_halted       (core_halted),
        .A_CLK             (a_clk),
        .A_MEN             (a_men),
        .A_WEN             (a_wen),
        .A_REN             (a_ren),
        .A_ADDR            (a_addr),
        .A_DIN             (a_din),
        .A_BM              (a_bm),
        .A_DOUT            (a_dout),
        .A_DLY             (a_dly),
        .A_BIST_CLK        (a_bist_clk),
        .A_BIST_EN         (a_bist_en),
        .A_BIST_MEN        (a_bist_men),
        .A_BIST_WEN        (a_bist_wen),
        .A_BIST_REN        (a_bist_ren),
        .A_BIST_ADDR       (a_bist_addr),
        .A_BIST_DIN        (a_bist_din),
        .A_BIST_BM         (a_bist_bm)
    );

    RM_IHPSG13_1P_512x32_c2_bm_bist u_sram (
        .A_CLK       (a_clk),
        .A_MEN       (a_men),
        .A_WEN       (a_wen),
        .A_REN       (a_ren),
        .A_ADDR      (a_addr),
        .A_DIN       (a_din),
        .A_BM        (a_bm),
        .A_DOUT      (a_dout),
        .A_DLY       (a_dly),
        .A_BIST_CLK  (a_bist_clk),
        .A_BIST_EN   (a_bist_en),
        .A_BIST_MEN  (a_bist_men),
        .A_BIST_WEN  (a_bist_wen),
        .A_BIST_REN  (a_bist_ren),
        .A_BIST_ADDR (a_bist_addr),
        .A_BIST_DIN  (a_bist_din),
        .A_BIST_BM   (a_bist_bm)
    );

endmodule

`default_nettype wire
