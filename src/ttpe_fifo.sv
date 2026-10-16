//==============================================================================
// ttpe_fifo.sv
//
// Synchronous FIFO, first-word-fall-through, 2**AW entries of W bits (default 8 x 8).
//
// Timing (all outputs come from flops; rdata is a mux of flops):
//   push in cycle X  -> count/empty/rdata reflect it in cycle X+1
//   pop  in cycle X  -> the head is consumed at the end of cycle X; rdata in cycle X is
//                       the byte being popped
//   full / empty are the values at the START of the cycle:
//     * a push while full is dropped, EVEN IF a pop happens in the same cycle
//       (simple and predictable; it matches the reference model),
//     * a pop while empty is ignored.
//   flush has priority over push and pop: the FIFO is empty (and overrun counter is 0) in
//   the next cycle. A pop in the flush cycle still sees valid rdata.
//
// drop      : high in a cycle in which a push is dropped because the FIFO is full
//             (not in a flush cycle).
// drop_cnt  : saturating count of dropped pushes since reset / last flush.
//
// Async-assert active-low reset on every flop (storage included, per the project rule
// "explicit resets everywhere"; the storage reset can be removed later if area requires).
//==============================================================================

`default_nettype none

module ttpe_fifo #(
    parameter int W  = 8,
    parameter int AW = 3
) (
    input  wire          clk,
    input  wire          rst_n,

    input  wire          flush,
    input  wire          push,
    input  wire [W-1:0]  wdata,
    input  wire          pop,

    output wire [W-1:0]  rdata,
    output wire          full,
    output wire          empty,
    output wire [AW:0]   count,
    output wire          drop,
    output wire [7:0]    drop_cnt
);

        localparam int DEPTH = 1 << AW;

    logic [DEPTH*W-1:0] mem_q;
    logic [AW-1:0]      wr_q, rd_q;
    logic [AW:0]        cnt_q;
    logic [7:0]         drop_q;

    // count never exceeds DEPTH = 2**AW, so bit AW set <=> count == DEPTH
    assign full  = cnt_q[AW];
    assign empty = (cnt_q == '0);

    wire do_push = push & ~full;
    wire do_pop  = pop  & ~empty;

    assign drop     = push & full & ~flush;
    assign rdata    = mem_q[rd_q*W +: W];
    assign count    = cnt_q;
    assign drop_cnt = drop_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_q  <= '0;
            wr_q   <= '0;
            rd_q   <= '0;
            cnt_q  <= '0;
            drop_q <= 8'd0;
        end else if (flush) begin
            // storage is left as is: it is unreachable while empty
            wr_q   <= '0;
            rd_q   <= '0;
            cnt_q  <= '0;
            drop_q <= 8'd0;
        end else begin
            if (do_push) begin
                mem_q[wr_q*W +: W] <= wdata;
                wr_q               <= wr_q + 1'b1;
            end
            if (do_pop) begin
                rd_q <= rd_q + 1'b1;
            end
            case ({do_push, do_pop})
                2'b10:   cnt_q <= cnt_q + 1'b1;
                2'b01:   cnt_q <= cnt_q - 1'b1;
                default: cnt_q <= cnt_q;
            endcase
            if (drop && drop_q != 8'hFF) begin
                drop_q <= drop_q + 8'd1;
            end
        end
    end

endmodule

`default_nettype wire
