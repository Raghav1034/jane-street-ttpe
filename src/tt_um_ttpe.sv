//==============================================================================
// tt_um_ttpe.sv - Tiny Tapeout top level of the Time-Triggered Protocol Engine (TTPE)
//
// Pin plan
//   uio[7:0]   Execution Unit pins (uio_out / uio_oe driven by the program, uio_in feeds the
//              2-FF synchroniser): 100 % available to the program.
//   ui_in[7:0] Synchroniser inputs for the program (source pins 8..15). ui_in[2:0] are in
//              addition the host SPI slave:  ui_in[0] = SCK, ui_in[1] = MOSI, ui_in[2] = CS_N.
//   uo_out[7:0] Execution Unit outputs (sink pins 8..15). uo_out[0] carries the host SPI MISO
//              while CS_N is low (synchronised) and the program's uo0 otherwise. A program
//              that drives uo0 therefore does not own that pin during a host SPI frame.
//   ena        unused (the design has no power-gating handshake).
//
// Reset: rst_n is synchronised here (async assert, sync release) and every block below
// receives the synchronised reset. The core starts halted; the host loads the program through
// SPI and writes CTRL.resume (see docs/HOST_SPI.md).
//
// The SRAM macro instance path is u_core.u_mem.u_sram (relevant for the physical config).
//==============================================================================

`default_nettype none

module tt_um_ttpe (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);

        import ttpe_isa_pkg::*;
wire rst_sync_n;

    ttpe_reset_sync u_rst (
        .clk       (clk),
        .rst_n_in  (rst_n),
        .rst_n_out (rst_sync_n)
    );

    // host SPI <-> core
    wire        miso, spi_sel;
    wire        host_req, host_we, host_gnt_unused, host_wdone_unused, host_rvalid;
    wire [8:0]  host_addr;
    wire [31:0] host_wdata, host_rdata;
    wire [3:0]  host_wstrb;
    wire        host_resume, host_pc_load, host_halt, host_core_reset;
    wire [8:0]  host_pc_val;
    wire        tx_push, tx_fifo_flush, rx_pop, rx_fifo_flush;
    wire [7:0]  tx_wdata, rx_rdata, rx_overrun_cnt;
    wire        tx_full, tx_empty, rx_full, rx_empty, halted, illegal, mem_free;
    wire [3:0]  tx_count, rx_count;
    wire [7:0]  core_uo;

    ttpe_core u_core (
        .clk           (clk),
        .rst_n         (rst_sync_n),
        .uio_in        (uio_in),
        .ui_in         (ui_in),
        .uio_out       (uio_out),
        .uio_oe        (uio_oe),
        .uo_out        (core_uo),
        .host_req      (host_req),
        .host_we       (host_we),
        .host_addr     (host_addr),
        .host_wdata    (host_wdata),
        .host_wstrb    (host_wstrb),
        .host_gnt      (host_gnt_unused),
        .host_wdone    (host_wdone_unused),
        .host_rvalid   (host_rvalid),
        .host_rdata    (host_rdata),
        .host_resume   (host_resume),
        .host_pc_load  (host_pc_load),
        .host_pc_val   (host_pc_val),
        .host_halt     (host_halt),
        .host_core_reset(host_core_reset),
        .tx_push       (tx_push),
        .tx_wdata      (tx_wdata),
        .tx_fifo_flush (tx_fifo_flush),
        .rx_pop        (rx_pop),
        .rx_rdata      (rx_rdata),
        .rx_fifo_flush (rx_fifo_flush),
        .tx_full       (tx_full),
        .tx_empty      (tx_empty),
        .rx_full       (rx_full),
        .rx_empty      (rx_empty),
        .tx_count      (tx_count),
        .rx_count      (rx_count),
        .rx_overrun_cnt(rx_overrun_cnt),
        .halted        (halted),
        .illegal       (illegal),
        .mem_free      (mem_free),
        .dbg_accept    (),
        .dbg_commit    (),
        .dbg_pc        (),
        .dbg_instr     (),
        .dbg_pc_load   (),
        .dbg_sr        (),
        .dbg_c0        (),
        .dbg_c1        ()
    );

    ttpe_host_spi u_spi (
        .clk            (clk),
        .rst_n          (rst_sync_n),
        .sck_i          (ui_in[0]),
        .mosi_i         (ui_in[1]),
        .cs_n_i         (ui_in[2]),
        .miso_o         (miso),
        .spi_sel        (spi_sel),
        .host_req       (host_req),
        .host_we        (host_we),
        .host_addr      (host_addr),
        .host_wdata     (host_wdata),
        .host_wstrb     (host_wstrb),
        .host_rvalid    (host_rvalid),
        .host_rdata     (host_rdata),
        .host_resume    (host_resume),
        .host_pc_load   (host_pc_load),
        .host_pc_val    (host_pc_val),
        .host_halt      (host_halt),
        .host_core_reset(host_core_reset),
        .tx_push        (tx_push),
        .tx_wdata       (tx_wdata),
        .tx_fifo_flush  (tx_fifo_flush),
        .rx_pop         (rx_pop),
        .rx_rdata       (rx_rdata),
        .rx_fifo_flush  (rx_fifo_flush),
        .tx_full        (tx_full),
        .tx_empty       (tx_empty),
        .rx_full        (rx_full),
        .rx_empty       (rx_empty),
        .tx_count       (tx_count),
        .rx_count       (rx_count),
        .rx_overrun_cnt (rx_overrun_cnt),
        .halted         (halted),
        .illegal        (illegal),
        .mem_free       (mem_free)
    );

    assign uo_out = {core_uo[7:1], spi_sel ? miso : core_uo[0]};

    // ena and the SRAM handshake outputs that the SPI layer does not need
    wire _unused = &{1'b0, ena, host_gnt_unused, host_wdone_unused};

endmodule

`default_nettype wire
