// =============================================================================
// Module      : input_unit
// Project     : NEXUS NoC Router
// Description : Per-port input buffer. Receives packets from an upstream
//               source (a neighboring router's output, or the local core)
//               via a valid/ready handshake, stores them in a small
//               synchronous FIFO, and presents the head-of-FIFO packet to
//               the routing/arbitration logic with its own valid/ready
//               handshake on the read side.
//
//               This is a single-clock-domain FIFO (the whole router runs
//               on one clock) -- the dual-pointer / Gray-code synchronizer
//               technique from the async FIFO project is NOT needed here
//               since there is no clock-domain crossing within one router.
//               The buffering/pointer-management skill carries over even
//               though the CDC-specific logic does not.
// =============================================================================

`timescale 1ns/1ps

module input_unit #(
    parameter int PKT_WIDTH  = 24,   // dest_x(2)+dest_y(2)+src_x(2)+src_y(2)+payload(16)
    parameter int FIFO_DEPTH = 4
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // ---- Write side: upstream sender -> this input unit ----
    input  logic                  in_valid,
    output logic                  in_ready,
    input  logic [PKT_WIDTH-1:0]  in_data,

    // ---- Read side: this input unit -> routing/arbitration logic ----
    output logic                  out_valid,
    input  logic                  out_ready,
    output logic [PKT_WIDTH-1:0]  out_data,

    // ---- Status (debug / assertions) ----
    output logic                  fifo_full,
    output logic                  fifo_empty
);

    // -------------------------------------------------------------------------
    // Pointer width: enough bits to index FIFO_DEPTH entries, +1 extra bit
    // for the classic "one extra bit" full/empty distinction technique.
    // -------------------------------------------------------------------------
    localparam int PTR_W = $clog2(FIFO_DEPTH) + 1;

    logic [PTR_W-1:0] wr_ptr, rd_ptr;
    logic [PKT_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    // -------------------------------------------------------------------------
    // Full/Empty Detection
    // Empty: pointers equal (all bits, including the extra MSB)
    // Full:  pointers equal in index bits but differ in the extra MSB
    //        (write pointer has wrapped exactly one more time than read)
    // -------------------------------------------------------------------------
    wire [PTR_W-2:0] wr_idx = wr_ptr[PTR_W-2:0];
    wire [PTR_W-2:0] rd_idx = rd_ptr[PTR_W-2:0];

    assign fifo_empty = (wr_ptr == rd_ptr);
    assign fifo_full  = (wr_idx == rd_idx) && (wr_ptr[PTR_W-1] != rd_ptr[PTR_W-1]);

    // -------------------------------------------------------------------------
    // Handshake Outputs
    // -------------------------------------------------------------------------
    assign in_ready  = !fifo_full;
    assign out_valid = !fifo_empty;

    // -------------------------------------------------------------------------
    // Write Logic
    // -------------------------------------------------------------------------
    wire wr_en = in_valid && in_ready;   // successful write handshake this cycle

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (wr_en) begin
            mem[wr_idx] <= in_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Read Logic
    // -------------------------------------------------------------------------
    wire rd_en = out_valid && out_ready; // successful read handshake this cycle

    assign out_data = mem[rd_idx];        // combinational read of head entry

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_ptr <= '0;
        end else if (rd_en) begin
            rd_ptr <= rd_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Assertions
    // -------------------------------------------------------------------------
    // pragma translate_off
    `ifdef ASSERTIONS_ON

    // Never write when full
    property no_write_when_full;
        @(posedge clk) disable iff (!rst_n)
        fifo_full |-> !wr_en;
    endproperty
    assert property (no_write_when_full)
        else $error("[INPUT_UNIT] Write attempted while FIFO full");

    // Never read when empty
    property no_read_when_empty;
        @(posedge clk) disable iff (!rst_n)
        fifo_empty |-> !rd_en;
    endproperty
    assert property (no_read_when_empty)
        else $error("[INPUT_UNIT] Read attempted while FIFO empty");

    // valid must not deassert before a successful handshake (standard
    // valid/ready stability rule)
    property valid_stable_until_handshake;
        @(posedge clk) disable iff (!rst_n)
        (out_valid && !out_ready) |=> out_valid;
    endproperty
    assert property (valid_stable_until_handshake)
        else $error("[INPUT_UNIT] out_valid dropped without handshake");

    `endif
    // pragma translate_on

endmodule : input_unit

