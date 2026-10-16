//==============================================================================
// ttpe_core.sv - TTPE core: memory subsystem + Execution Unit + host FIFOs
//
// Pin plan (top level, later):
//   uio[7:0]  : 100 % owned by the Execution Unit (uio_out / uio_oe) and the 2-FF
//               synchroniser (uio_in -> source pins 0..7).
//   ui_in[7:0]: feed the synchroniser (source pins 8..15). The top level may additionally
//               tap ui_in[2:0] for the host SPI slave.
//   uo_out[7:0]: driven by the Execution Unit (sink pins 8..15). The top level may mux
//               uo_out[0] with the host SPI MISO.
//
// HOST INTERFACE (all signals synchronous to clk; inputs are sampled at the edge that
// ends the cycle in which they are applied)
//
//   SRAM access  (host_req/we/addr/wdata/wstrb, host_gnt/wdone/rvalid/rdata)
//       Same protocol as ttpe_sram_ctrl. The host is granted only while the fetch
//       controller is idle (mem_free = 1). host_block = ~mem_free, so SRAM fetch timing is
//       fully deterministic while a program runs. Keep host_req asserted until host_gnt.
//
//   Run control
//       host_resume  : 1-cycle pulse. Clears halted/illegal; core_run is high from the
//                      next cycle and the first instruction is accepted 3 cycles later.
//                      Ignored unless halted && mem_free.
//       host_pc_load : with host_pc_val, loads the fetch PC. Ignored unless halted &&
//                      mem_free. May be asserted in the same cycle as host_resume.
//       (After HALT or an illegal instruction the PC points to the next instruction, so
//        a plain host_resume continues after the HALT.)
//       host_halt    : 1-cycle pulse, works at any time. Soft halt of a running program:
//                      nothing new is accepted, an instruction committing in the same cycle
//                      completes, the held instruction is discarded, SR/C0/C1/pins are kept.
//                      The fetch PC is undefined afterwards: load a PC before host_resume.
//                      Wins over host_resume in the same cycle.
//       host_core_reset: 1-cycle pulse. Like host_halt, and also clears SR, C0, C1, releases
//                      all uio pins, clears uo_out and the illegal flag. FIFOs are not touched
//                      (use tx_fifo_flush / rx_fifo_flush).
//       The host SRAM port becomes usable (mem_free = 1) a few cycles after a halt, when
//       the fetch controller has returned to idle; poll mem_free / halted.
//
//   TX FIFO (host -> engine, read by HPULL), 8 x 8 bit
//       tx_push + tx_wdata : push one byte. Dropped if tx_full (check tx_full first).
//       tx_fifo_flush      : synchronous flush, has priority over tx_push.
//   RX FIFO (engine -> host, written by HPUSH), 8 x 8 bit, first-word-fall-through
//       rx_rdata           : head byte, valid while !rx_empty
//       rx_pop             : consume the head byte (the byte seen in that cycle)
//       rx_fifo_flush      : flush and clear rx_overrun_cnt; has priority over HPUSH
//       HPUSH into a full FIFO is dropped and counted in rx_overrun_cnt (8 bit, saturating)
//
//   Status: tx_full tx_empty rx_full rx_empty tx_count rx_count rx_overrun_cnt halted
//           illegal mem_free
//
// dbg_* outputs exist for the lockstep testbench and must be left open at the chip top.
//==============================================================================

`default_nettype none

module ttpe_core #(
    parameter [0:0] BM_WRITE_LEVEL = 1'b1
) (
    input  wire         clk,
    input  wire         rst_n,

    // pads
    input  wire [7:0]   uio_in,
    input  wire [7:0]   ui_in,
    output wire [7:0]   uio_out,
    output wire [7:0]   uio_oe,
    output wire [7:0]   uo_out,

    // host SRAM port
    input  wire         host_req,
    input  wire         host_we,
    input  wire [8:0]   host_addr,
    input  wire [31:0]  host_wdata,
    input  wire [3:0]   host_wstrb,
    output wire         host_gnt,
    output wire         host_wdone,
    output wire         host_rvalid,
    output wire [31:0]  host_rdata,

    // host run control
    input  wire         host_resume,
    input  wire         host_pc_load,
    input  wire [8:0]   host_pc_val,
    input  wire         host_halt,
    input  wire         host_core_reset,

    // host FIFO ports
    input  wire         tx_push,
    input  wire [7:0]   tx_wdata,
    input  wire         tx_fifo_flush,
    input  wire         rx_pop,
    output wire [7:0]   rx_rdata,
    input  wire         rx_fifo_flush,

    // status
    output wire         tx_full,
    output wire         tx_empty,
    output wire         rx_full,
    output wire         rx_empty,
    output wire [3:0]   tx_count,
    output wire [3:0]   rx_count,
    output wire [7:0]   rx_overrun_cnt,
    output wire         halted,
    output wire         illegal,
    output wire         mem_free,

    // debug (testbench visibility)
    output wire         dbg_accept,
    output wire         dbg_commit,
    output wire [8:0]   dbg_pc,
    output wire [31:0]  dbg_instr,
    output wire         dbg_pc_load,
    output wire [7:0]   dbg_sr,
    output wire [7:0]   dbg_c0,
    output wire [7:0]   dbg_c1
);

    //--------------------------------------------------------------------------
    // Fetch controller + SRAM macro
    //--------------------------------------------------------------------------
    wire        valid_instruction;
    wire [31:0] instruction;
    wire [8:0]  instr_pc;
    wire        instr_accept;
    wire        core_fetch_stall_unused;
    wire        mem_halted;

    wire        x_pc_load;
    wire [8:0]  x_pc_load_val;

    wire        ctl_ok = halted & mem_halted;          // host may use run control
    wire        h_resume  = host_resume  & ctl_ok;
    wire        h_pc_load = host_pc_load & ctl_ok;

    wire        pc_load     = x_pc_load | h_pc_load;
    wire [8:0]  pc_load_val = x_pc_load ? x_pc_load_val : host_pc_val;

    assign mem_free = mem_halted;

    ttpe_mem_subsys #(
        .BM_WRITE_LEVEL (BM_WRITE_LEVEL)
    ) u_mem (
        .clk               (clk),
        .rst_n             (rst_n),
        .host_req          (host_req),
        .host_we           (host_we),
        .host_addr         (host_addr),
        .host_wdata        (host_wdata),
        .host_wstrb        (host_wstrb),
        .host_block        (~mem_halted),
        .host_gnt          (host_gnt),
        .host_wdone        (host_wdone),
        .host_rvalid       (host_rvalid),
        .host_rdata        (host_rdata),
        .core_run          (~halted),
        .pc_load           (pc_load),
        .pc_load_val       (pc_load_val),
        .instr_accept      (instr_accept),
        .valid_instruction (valid_instruction),
        .instruction       (instruction),
        .instr_pc          (instr_pc),
        .core_fetch_stall  (core_fetch_stall_unused),
        .core_halted       (mem_halted)
    );

    //--------------------------------------------------------------------------
    // FIFOs
    //--------------------------------------------------------------------------
    wire        x_tx_pop, x_rx_push;
    wire [7:0]  x_tx_rdata, x_rx_wdata;
    wire [3:0]  tx_cnt_w, rx_cnt_w;

    ttpe_fifo #(.W(8), .AW(3)) u_tx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .flush    (tx_fifo_flush),
        .push     (tx_push),
        .wdata    (tx_wdata),
        .pop      (x_tx_pop),
        .rdata    (x_tx_rdata),
        .full     (tx_full),
        .empty    (tx_empty),
        .count    (tx_cnt_w),
        .drop     (),
        .drop_cnt ()
    );

    ttpe_fifo #(.W(8), .AW(3)) u_rx_fifo (
        .clk      (clk),
        .rst_n    (rst_n),
        .flush    (rx_fifo_flush),
        .push     (x_rx_push),
        .wdata    (x_rx_wdata),
        .pop      (rx_pop),
        .rdata    (rx_rdata),
        .full     (rx_full),
        .empty    (rx_empty),
        .count    (rx_cnt_w),
        .drop     (),
        .drop_cnt (rx_overrun_cnt)
    );

    assign tx_count = tx_cnt_w;
    assign rx_count = rx_cnt_w;

    //--------------------------------------------------------------------------
    // Execution Unit
    //--------------------------------------------------------------------------
    ttpe_exec u_exec (
        .clk               (clk),
        .rst_n             (rst_n),
        .pad_in            ({ui_in, uio_in}),
        .uio_out           (uio_out),
        .uio_oe            (uio_oe),
        .uo_out            (uo_out),
        .valid_instruction (valid_instruction),
        .instruction       (instruction),
        .instr_pc          (instr_pc),
        .instr_accept      (instr_accept),
        .pc_load           (x_pc_load),
        .pc_load_val       (x_pc_load_val),
        .tx_empty          (tx_empty),
        .tx_rdata          (x_tx_rdata),
        .tx_pop            (x_tx_pop),
        .rx_push           (x_rx_push),
        .rx_wdata          (x_rx_wdata),
        .resume            (h_resume),
        .halt_req          (host_halt),
        .core_reset        (host_core_reset),
        .halted            (halted),
        .illegal           (illegal),
        .dbg_commit        (dbg_commit),
        .dbg_sr            (dbg_sr),
        .dbg_c0            (dbg_c0),
        .dbg_c1            (dbg_c1)
    );

    assign dbg_accept  = instr_accept;
    assign dbg_pc      = instr_pc;
    assign dbg_instr   = instruction;
    assign dbg_pc_load = x_pc_load;

endmodule

`default_nettype wire
