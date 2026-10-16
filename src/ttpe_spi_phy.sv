//==============================================================================
// ttpe_spi_phy.sv - SPI slave byte engine, mode 0 (CPOL=0, CPHA=0), MSB first.
//
// Everything runs in the clk domain. sck / mosi / cs_n are asynchronous inputs and go through
// a 3-flop chain each (two synchroniser flops plus one delay flop for edge detection), so the
// SPI clock is only ever *sampled*, never used as a clock. All three signals see the same
// latency, so their relative timing at the master is preserved.
//
//   cs_active  = cs_n low (synchronised)
//   cs_fall    : 1-cycle pulse, the cycle in which cs_active becomes 1
//   sck rise   : MOSI is sampled, rx shift register advances
//   sck fall   : MISO is advanced (shift) or, at a byte boundary, reloaded
//
// MISO content
//   * at cs_fall the shifter is loaded with `status_byte`, so the first byte the master clocks
//     in (while it sends the command byte) is the status byte,
//   * at every later byte boundary (the falling edge after the 8th, 16th ... rising edge) the
//     shifter is loaded with `tx_byte_next`; `tx_load` is high in exactly that cycle so the
//     protocol layer can advance FIFO pointers / word indices.
//
// rx_valid is a 1-cycle pulse (a flop) one cycle after the rising edge that completed a byte;
// rx_byte is valid in that cycle.
//
// rise_first is a combinational pulse in the cycle in which the synchronised SCK rises for
// bit 7 of a byte. The protocol layer uses it to know that the master has sampled the MSB of
// the byte that was loaded, i.e. that the byte is consumed.
//
// TIMING LIMITS (cycles of clk, H = SCK half period in clk cycles; pad delays not included)
//   * CS_N low to first SCK rising edge: the status byte is on MISO 3 clk edges after CS_N
//     falls at the latest, so >= 4 cycles is the arithmetic minimum; use >= 8 (160 ns).
//   * H >= 8 (SCK <= clk/16 = 3.125 MHz at 50 MHz): the protocol layer needs 5 cycles after
//     the rising edge of a command byte to have the first response byte ready and the load
//     happens at the next falling edge, at least H-1 = 7 cycles after the rising edge.
//   * CS_N high between frames: >= 4 cycles.
// Only mode 0 is supported (SCK idles low). Bytes are 8 bits; a frame that is cut off in the
// middle of a byte is discarded by the next cs_fall.
//
// Async-assert active-low reset on every flop; chains reset to the idle line state
// (cs_n high, sck low, mosi low).
//==============================================================================

`default_nettype none

import ttpe_isa_pkg::*;
module ttpe_spi_phy (
    input  wire       clk,
    input  wire       rst_n,

    input  wire       sck_i,
    input  wire       mosi_i,
    input  wire       cs_n_i,

    input  wire [7:0] status_byte,    // loaded into the MISO shifter at cs_fall
    input  wire [7:0] tx_byte_next,   // loaded at every later byte boundary
    output wire       miso_o,

    output wire       cs_active,
    output wire       cs_fall,
    output wire       rx_valid,
    output wire [7:0] rx_byte,
    output wire       tx_load,
    output wire       rise_first      // SCK rising edge that carries bit 7 of a byte (comb)
);

        //--------------------------------------------------------------------------
    // Synchronisers
    //--------------------------------------------------------------------------
    logic sck_s1_q, sck_s2_q, sck_s3_q;
    logic cs_s1_q,  cs_s2_q,  cs_s3_q;
    logic mosi_s1_q, mosi_s2_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_s1_q  <= 1'b0;
            sck_s2_q  <= 1'b0;
            sck_s3_q  <= 1'b0;
            cs_s1_q   <= 1'b1;
            cs_s2_q   <= 1'b1;
            cs_s3_q   <= 1'b1;
            mosi_s1_q <= 1'b0;
            mosi_s2_q <= 1'b0;
        end else begin
            sck_s1_q  <= sck_i;
            sck_s2_q  <= sck_s1_q;
            sck_s3_q  <= sck_s2_q;
            cs_s1_q   <= cs_n_i;
            cs_s2_q   <= cs_s1_q;
            cs_s3_q   <= cs_s2_q;
            mosi_s1_q <= mosi_i;
            mosi_s2_q <= mosi_s1_q;
        end
    end

    assign cs_active = ~cs_s2_q;
    assign cs_fall   = cs_s3_q & ~cs_s2_q;

    wire sck_rise = ~sck_s3_q &  sck_s2_q;
    wire sck_fall =  sck_s3_q & ~sck_s2_q;

    //--------------------------------------------------------------------------
    // Shifters
    //--------------------------------------------------------------------------
    logic [2:0] bit_cnt_q;
    logic [7:0] rx_shift_q, tx_shift_q, rx_byte_q;
    logic       rx_valid_q;

    wire do_rise  = sck_rise & cs_active & ~cs_fall;
    wire do_fall  = sck_fall & cs_active;
    wire boundary = do_fall & (bit_cnt_q == 3'd0);

    wire [7:0] rx_next = {rx_shift_q[6:0], mosi_s2_q};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt_q  <= 3'd0;
            rx_shift_q <= 8'd0;
            tx_shift_q <= 8'd0;
            rx_byte_q  <= 8'd0;
            rx_valid_q <= 1'b0;
        end else if (cs_fall) begin
            bit_cnt_q  <= 3'd0;
            tx_shift_q <= status_byte;
            rx_valid_q <= 1'b0;
        end else begin
            rx_valid_q <= 1'b0;
            if (do_rise) begin
                rx_shift_q <= rx_next;
                bit_cnt_q  <= bit_cnt_q + 3'd1;
                if (bit_cnt_q == 3'd7) begin
                    rx_byte_q  <= rx_next;
                    rx_valid_q <= 1'b1;
                end
            end
            if (do_fall) begin
                if (bit_cnt_q == 3'd0) tx_shift_q <= tx_byte_next;
                else                   tx_shift_q <= {tx_shift_q[6:0], 1'b0};
            end
        end
    end

    assign miso_o   = tx_shift_q[7];
    assign rx_valid = rx_valid_q;
    assign rx_byte  = rx_byte_q;
    assign tx_load  = boundary;
    assign rise_first = do_rise & (bit_cnt_q == 3'd0);

endmodule

`default_nettype wire
