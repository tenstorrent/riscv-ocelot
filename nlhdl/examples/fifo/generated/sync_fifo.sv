/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

// -----------------------------------------------------------------------------
// sync_fifo — single-clock (synchronous) first-in first-out queue.
// Write and read share one clock domain. Words are read back in write order.
// Output is registered (not show-ahead): rd_data is valid the cycle after a
// successful read.
//
// Generated from: fifo.nlhdl.sv
// -----------------------------------------------------------------------------

`default_nettype none

module sync_fifo #(
    // WIDTH — bit width of each data word (spec: legal range 1..1024).
    parameter int WIDTH              = 32,
    // DEPTH — number of entries. Rounded up to the next power of two below so
    // that pointer wrap-around is free.
    parameter int DEPTH              = 16,
    // Threshold flags: assert when free/used entries fall at or below these.
    parameter int ALMOST_FULL_THRESH  = 2,
    parameter int ALMOST_EMPTY_THRESH = 2
) (
    // Single clock domain: rising edge, synchronous active-low reset.
    input  wire                     clk,
    input  wire                     rst_n,

    // Write (enqueue) interface.
    input  wire                     wr_en,
    input  wire [WIDTH-1:0]         wr_data,
    output wire                     full,
    output wire                     almost_full,

    // Read (dequeue) interface.
    input  wire                     rd_en,
    output logic [WIDTH-1:0]        rd_data,
    output wire                     empty,
    output wire                     almost_empty,

    // Status: current occupancy.
    output wire [$clog2(DEPTH+1)-1:0] count
);

    // Effective depth rounded up to a power of two (spec: round storage up).
    localparam int DEPTH_EFF = 1 << $clog2(DEPTH);
    localparam int ADDR_W    = $clog2(DEPTH_EFF);
    // wrap bit: extra MSB on each pointer distinguishes full from empty
    localparam int PTR_W     = ADDR_W + 1;

    logic [WIDTH-1:0] mem [DEPTH_EFF];

    // Pointers carry one extra MSB (the wrap bit) beyond the address field.
    logic [PTR_W-1:0] wr_ptr;
    logic [PTR_W-1:0] rd_ptr;

    wire [ADDR_W-1:0] wr_addr = wr_ptr[ADDR_W-1:0];
    wire [ADDR_W-1:0] rd_addr = rd_ptr[ADDR_W-1:0];

    // Empty: pointers fully equal. Full: addresses equal, wrap bits differ.
    assign empty = (wr_ptr == rd_ptr);
    assign full  = (wr_addr == rd_addr) && (wr_ptr[ADDR_W] != rd_ptr[ADDR_W]);

    // overflow/underflow guard: these conditions must never corrupt state
    wire do_write = wr_en && !full;   // writes ignored while full
    wire do_read  = rd_en && !empty;  // reads ignored while empty

    // Occupancy = wr_ptr - rd_ptr in modulo-2^PTR_W arithmetic (0..DEPTH_EFF).
    assign count        = ($clog2(DEPTH+1))'(wr_ptr - rd_ptr);
    assign almost_full  = ((DEPTH_EFF - count) <= ALMOST_FULL_THRESH);
    assign almost_empty = (count <= ALMOST_EMPTY_THRESH);

    // Write port: store on a successful write, then advance the write pointer.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (do_write) begin
            mem[wr_addr] <= wr_data;
            wr_ptr       <= wr_ptr + 1'b1;
        end
    end

    // Read port: registered output, valid the cycle after a successful read.
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            rd_ptr  <= '0;
            rd_data <= '0;
        end else if (do_read) begin
            rd_data <= mem[rd_addr];
            rd_ptr  <= rd_ptr + 1'b1;
        end
    end

    // A simultaneous read and write is allowed: both pointers advance and count
    // is unchanged, handled naturally by the two independent always_ff blocks.

endmodule

`default_nettype wire
