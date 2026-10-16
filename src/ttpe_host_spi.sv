//==============================================================================
// ttpe_host_spi.sv - Host SPI slave (mode 0) with register map for the TTPE core.
//
// Frame:  CS_N low | command byte | payload byte(s) | CS_N high
//   command byte = { rw, addr[6:0] }   rw = 1: read, rw = 0: write
//   MISO while the command byte is clocked in: STATUS byte (always, no extra command needed)
//   MISO during payload byte k (k = 0, 1, ...): the register / stream data for that byte
//   (reads), 0x00 (writes and rejected commands).
//   MOSI payload byte k: write data. Every payload byte is a separate write access, so a
//   frame can carry any number of bytes (that is what makes the stream windows work).
//
// STATUS byte  (also register 0x01)
//   [0] halted   [1] illegal   [2] mem_free   [3] tx_empty   [4] tx_full
//   [5] rx_empty [6] rx_full   [7] rx_overrun_cnt != 0
//
// Register map (R = readable, W = writable)
//   0x00 R    ID         constant 0x54
//   0x01 R    STATUS
//   0x02 R/W  CTRL       write: [0] resume  [1] pc_load (uses PC)  [2] tx_flush  [3] rx_flush
//                        [4] halt (soft halt of a running program)
//                        [5] core_reset (halt + clear SR/C0/C1/pins/illegal)
//                        (pulses, 0 = no action). A read returns 0x00.
//                        resume / pc_load only act while halted and mem_free (core rule);
//                        halt and core_reset work at any time and win over resume.
//                        After halt / core_reset the PC is undefined: write PC, then
//                        CTRL = pc_load, then CTRL = resume (or both bits in one write).
//   0x03 R/W  PC_LO      holding register for the start PC, bits [7:0]
//   0x04 R/W  PC_HI      bit 0 = PC bit 8
//   0x05 R    FIFO_CNT   { rx_count[3:0], tx_count[3:0] }
//   0x06 R    RX_OVR     rx overrun counter (8 bit, saturating; rx_flush clears it)
//   0x07 R/W  SCRATCH    8 bit scratch register (bring-up / link test)
//   0x08 R/W  SADDR_LO   SRAM word address, bits [7:0]
//   0x09 R/W  SADDR_HI   bit 0 = word address bit 8
//   0x0A R/W  ERR        sticky error flags, write 1 to clear:
//                        [0] SRAM window used while the core is running (write dropped /
//                            read returned 0), [1] rejected command (unmapped address or
//                            wrong direction), [2] TX_PUSH byte dropped because the TX FIFO
//                            was full
//   0x10 W    TX_PUSH    stream: each payload byte is pushed into the TX FIFO (HPULL source)
//   0x11 R    RX_POP     stream: each payload byte is the head of the RX FIFO (HPUSH
//                        destination); 0x00 and no pop if the FIFO is empty
//   0x12 R/W  SRAM       stream window onto the SRAM at SADDR, 32 bit words, LSB byte first,
//                        auto-increment of SADDR after every 4th byte (9 bit wrap).
//                        Only usable while mem_free = 1 (core halted).
//
// Stream semantics
//   * A stream byte on a read is *consumed* when the master has sampled its MSB (first SCK
//     rising edge of that byte). A byte that was loaded into the shifter but never clocked
//     (the master stopped after the previous byte) is not consumed: no FIFO pop, no address
//     increment. The same word is returned by the next frame.
//   * SRAM reads are prefetched: the word at SADDR is read when the command byte arrives,
//     and the next word is read when the 4th byte of a word is consumed.
//   * SRAM writes assemble 4 payload bytes (LSB first) and write the word with byte strobes
//     0xF on the 4th byte; an incomplete word at the end of a frame is discarded.
//   * Unmapped addresses and wrong directions set ERR[1], read as 0x00 and ignore writes.
//
// Limits (see ttpe_spi_phy.sv): mode 0 only; SCK half period >= 8 clk cycles; CS_N high >= 4
// cycles between frames; first SCK rising edge >= 4 cycles (use 8) after CS_N falls.
//==============================================================================

`default_nettype none

import ttpe_isa_pkg::*;
module ttpe_host_spi (
    input  wire         clk,
    input  wire         rst_n,

    // pads
    input  wire         sck_i,
    input  wire         mosi_i,
    input  wire         cs_n_i,
    output wire         miso_o,
    output wire         spi_sel,          // synchronised CS_N low (frame in progress)

    // core host SRAM port
    output wire         host_req,
    output wire         host_we,
    output wire [8:0]   host_addr,
    output wire [31:0]  host_wdata,
    output wire [3:0]   host_wstrb,
    input  wire         host_rvalid,
    input  wire [31:0]  host_rdata,

    // core run control
    output wire         host_resume,
    output wire         host_pc_load,
    output wire [8:0]   host_pc_val,
    output wire         host_halt,
    output wire         host_core_reset,

    // core FIFO ports
    output wire         tx_push,
    output wire [7:0]   tx_wdata,
    output wire         tx_fifo_flush,
    output wire         rx_pop,
    input  wire [7:0]   rx_rdata,
    output wire         rx_fifo_flush,

    // core status
    input  wire         tx_full,
    input  wire         tx_empty,
    input  wire         rx_full,
    input  wire         rx_empty,
    input  wire [3:0]   tx_count,
    input  wire [3:0]   rx_count,
    input  wire [7:0]   rx_overrun_cnt,
    input  wire         halted,
    input  wire         illegal,
    input  wire         mem_free
);

    localparam [6:0] R_ID      = 7'h00;
    localparam [6:0] R_STATUS  = 7'h01;
    localparam [6:0] R_CTRL    = 7'h02;
    localparam [6:0] R_PC_LO   = 7'h03;
    localparam [6:0] R_PC_HI   = 7'h04;
    localparam [6:0] R_FIFO_CNT= 7'h05;
    localparam [6:0] R_RX_OVR  = 7'h06;
    localparam [6:0] R_SCRATCH = 7'h07;
    localparam [6:0] R_SA_LO   = 7'h08;
    localparam [6:0] R_SA_HI   = 7'h09;
    localparam [6:0] R_ERR     = 7'h0A;
    localparam [6:0] R_TX_PUSH = 7'h10;
    localparam [6:0] R_RX_POP  = 7'h11;
    localparam [6:0] R_SRAM    = 7'h12;

    localparam [7:0] ID_VALUE  = 8'h54;

    //--------------------------------------------------------------------------
    // PHY
    //--------------------------------------------------------------------------
    wire       rx_valid;
    wire [7:0] rx_byte;
    wire       cs_fall, tx_load, rise_first;
    wire [7:0] status_byte;
    wire [7:0] tx_next;

    ttpe_spi_phy u_phy (
        .clk          (clk),
        .rst_n        (rst_n),
        .sck_i        (sck_i),
        .mosi_i       (mosi_i),
        .cs_n_i       (cs_n_i),
        .status_byte  (status_byte),
        .tx_byte_next (tx_next),
        .miso_o       (miso_o),
        .cs_active    (spi_sel),
        .cs_fall      (cs_fall),
        .rx_valid     (rx_valid),
        .rx_byte      (rx_byte),
        .tx_load      (tx_load),
        .rise_first   (rise_first)
    );

    wire ovr_nz = |rx_overrun_cnt;
    assign status_byte = {ovr_nz, rx_full, rx_empty, tx_full, tx_empty, mem_free, illegal, halted};

    //--------------------------------------------------------------------------
    // State
    //--------------------------------------------------------------------------
    logic        have_cmd_q, rw_q, ok_q;
    logic [6:0]  addr_q;
    logic [1:0]  widx_q;
    logic [23:0] wbuf_q;
    logic        ld_pend_q, ld_real_q;

    logic [8:0]  pc_q, sa_q, ha_q;
    logic [7:0]  scratch_q;
    logic [2:0]  err_q;

    logic [31:0] rd_word_q, wdata_q;
    logic        req_q, req_we_q;

    //--------------------------------------------------------------------------
    // Command decode
    //--------------------------------------------------------------------------
    wire        cmd_rw   = rx_byte[7];
    wire [6:0]  cmd_addr = rx_byte[6:0];

    wire        rd_sup = (cmd_addr == R_ID)      || (cmd_addr == R_STATUS)  ||
                         (cmd_addr == R_CTRL)    || (cmd_addr == R_PC_LO)   ||
                         (cmd_addr == R_PC_HI)   || (cmd_addr == R_FIFO_CNT)||
                         (cmd_addr == R_RX_OVR)  || (cmd_addr == R_SCRATCH) ||
                         (cmd_addr == R_SA_LO)   || (cmd_addr == R_SA_HI)   ||
                         (cmd_addr == R_ERR)     || (cmd_addr == R_RX_POP)  ||
                         (cmd_addr == R_SRAM);
    wire        wr_sup = (cmd_addr == R_CTRL)    || (cmd_addr == R_PC_LO)   ||
                         (cmd_addr == R_PC_HI)   || (cmd_addr == R_SCRATCH) ||
                         (cmd_addr == R_SA_LO)   || (cmd_addr == R_SA_HI)   ||
                         (cmd_addr == R_ERR)     || (cmd_addr == R_TX_PUSH) ||
                         (cmd_addr == R_SRAM);
    wire        cmd_ok   = cmd_rw ? rd_sup : wr_sup;

    wire dec_cmd  = rx_valid & ~have_cmd_q & ~cs_fall;
    wire dec_data = rx_valid &  have_cmd_q & ~cs_fall & ~rw_q & ok_q;
    wire consume  = rise_first & ld_pend_q;

    wire is_sram   = (addr_q == R_SRAM);
    wire widx_last = (widx_q == 2'd3);

    wire sram_rd_start = dec_cmd & cmd_ok & cmd_rw & (cmd_addr == R_SRAM);
    wire sram_rd_next  = consume & rw_q & ok_q & is_sram & widx_last;
    wire sram_wr       = dec_data & is_sram & widx_last;

    wire [2:0] err_set = { tx_push & tx_full,
                           dec_cmd & ~cmd_ok,
                           (sram_rd_start | sram_rd_next | sram_wr) & ~mem_free };
    wire [2:0] rx_byte_lo3 = rx_byte[2:0];
    wire [2:0] err_clr = (dec_data & (addr_q == R_ERR)) ? rx_byte_lo3 : 3'b000;

    //--------------------------------------------------------------------------
    // Outputs towards the core
    //--------------------------------------------------------------------------
    wire ctrl_wr = dec_data & (addr_q == R_CTRL);

    assign host_resume   = ctrl_wr & rx_byte[0];
    assign host_pc_load  = ctrl_wr & rx_byte[1];
    assign tx_fifo_flush = ctrl_wr & rx_byte[2];
    assign rx_fifo_flush = ctrl_wr & rx_byte[3];
    assign host_halt     = ctrl_wr & rx_byte[4];
    assign host_core_reset = ctrl_wr & rx_byte[5];
    assign host_pc_val   = pc_q;

    assign tx_push  = dec_data & (addr_q == R_TX_PUSH);
    assign tx_wdata = rx_byte;

    assign rx_pop   = consume & rw_q & ok_q & (addr_q == R_RX_POP) & ld_real_q;

    assign host_req   = req_q;
    assign host_we    = req_we_q;
    assign host_addr  = ha_q;
    assign host_wdata = wdata_q;
    assign host_wstrb = 4'hF;

    //--------------------------------------------------------------------------
    // MISO data for the next byte
    //--------------------------------------------------------------------------
    wire [7:0] pc_lo_w      = pc_q[7:0];
    wire [7:0] pc_hi_w      = {7'b0, pc_q[8]};
    wire [7:0] sa_lo_w      = sa_q[7:0];
    wire [7:0] sa_hi_w      = {7'b0, sa_q[8]};
    wire [7:0] err_w        = {5'b0, err_q};
    wire [7:0] cnt_w        = {rx_count, tx_count};
    wire [7:0] rxpop_w      = rx_empty ? 8'h00 : rx_rdata;
    wire [4:0] sh_w         = {widx_q, 3'b000};
    wire [31:0] rd_shift_w  = rd_word_q >> sh_w;
    wire [7:0] sram_byte_w  = rd_shift_w[7:0];

    logic [7:0] rd_mux;
    always_comb begin
        case (addr_q)
            R_ID:       rd_mux = ID_VALUE;
            R_STATUS:   rd_mux = status_byte;
            R_PC_LO:    rd_mux = pc_lo_w;
            R_PC_HI:    rd_mux = pc_hi_w;
            R_FIFO_CNT: rd_mux = cnt_w;
            R_RX_OVR:   rd_mux = rx_overrun_cnt;
            R_SCRATCH:  rd_mux = scratch_q;
            R_SA_LO:    rd_mux = sa_lo_w;
            R_SA_HI:    rd_mux = sa_hi_w;
            R_ERR:      rd_mux = err_w;
            R_RX_POP:   rd_mux = rxpop_w;
            R_SRAM:     rd_mux = sram_byte_w;
            default:    rd_mux = 8'h00;
        endcase
    end

    assign tx_next = (have_cmd_q & rw_q & ok_q) ? rd_mux : 8'h00;

    //--------------------------------------------------------------------------
    // Sequential logic
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            have_cmd_q <= 1'b0;
            rw_q       <= 1'b0;
            ok_q       <= 1'b0;
            addr_q     <= 7'd0;
            widx_q     <= 2'd0;
            wbuf_q     <= 24'd0;
            ld_pend_q  <= 1'b0;
            ld_real_q  <= 1'b0;
            pc_q       <= 9'd0;
            sa_q       <= 9'd0;
            ha_q       <= 9'd0;
            scratch_q  <= 8'd0;
            err_q      <= 3'd0;
            rd_word_q  <= 32'd0;
            wdata_q    <= 32'd0;
            req_q      <= 1'b0;
            req_we_q   <= 1'b0;
        end else begin
            req_q <= 1'b0;
            err_q <= (err_q & ~err_clr) | err_set;
            if (host_rvalid) rd_word_q <= host_rdata;

            if (cs_fall) begin
                have_cmd_q <= 1'b0;
                ld_pend_q  <= 1'b0;
                widx_q     <= 2'd0;
            end else begin
                if (tx_load) begin
                    ld_pend_q <= 1'b1;
                    ld_real_q <= ~rx_empty;
                end

                // consumption of a loaded read byte (master sampled its MSB)
                if (consume) begin
                    ld_pend_q <= 1'b0;
                    if (rw_q & ok_q & is_sram) begin
                        widx_q <= widx_q + 2'd1;
                        if (widx_last) begin
                            sa_q <= sa_q + 9'd1;
                            if (mem_free) begin
                                req_q    <= 1'b1;
                                req_we_q <= 1'b0;
                                ha_q     <= sa_q + 9'd1;
                            end else begin
                                rd_word_q <= 32'd0;
                            end
                        end
                    end
                end

                // command byte
                if (dec_cmd) begin
                    have_cmd_q <= 1'b1;
                    rw_q       <= cmd_rw;
                    addr_q     <= cmd_addr;
                    ok_q       <= cmd_ok;
                    widx_q     <= 2'd0;
                    if (cmd_ok & cmd_rw & (cmd_addr == R_SRAM)) begin
                        if (mem_free) begin
                            req_q    <= 1'b1;
                            req_we_q <= 1'b0;
                            ha_q     <= sa_q;
                        end else begin
                            rd_word_q <= 32'd0;
                        end
                    end
                end

                // payload byte of a write
                if (dec_data) begin
                    case (addr_q)
                        R_PC_LO:   pc_q[7:0]      <= rx_byte;
                        R_PC_HI:   pc_q[8]        <= rx_byte[0];
                        R_SCRATCH: scratch_q      <= rx_byte;
                        R_SA_LO:   sa_q[7:0]      <= rx_byte;
                        R_SA_HI:   sa_q[8]        <= rx_byte[0];
                        R_SRAM: begin
                            widx_q <= widx_q + 2'd1;
                            case (widx_q)
                                2'd0:    wbuf_q[7:0]   <= rx_byte;
                                2'd1:    wbuf_q[15:8]  <= rx_byte;
                                2'd2:    wbuf_q[23:16] <= rx_byte;
                                default: begin
                                    if (mem_free) begin
                                        req_q    <= 1'b1;
                                        req_we_q <= 1'b1;
                                        wdata_q  <= {rx_byte, wbuf_q};
                                        ha_q     <= sa_q;
                                        sa_q     <= sa_q + 9'd1;
                                    end
                                end
                            endcase
                        end
                        default: ;
                    endcase
                end
            end
        end
    end

endmodule

`default_nettype wire
