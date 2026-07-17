// =============================================================================
// Module      : output_unit
// Project     : NEXUS NoC Router
// Description : Per-port output stage. Takes the packet selected for this
//               output by the crossbar (already qualified by the
//               registered grant -- see crossbar.sv) and pushes it into a
//               small depth-2 FIFO, which drains out to the downstream
//               receiver (a neighboring router's input_unit, or the local
//               core) using the standard valid/ready handshake.
//
// DESIGN NOTE : An earlier version of this module used a single
//               skid-register and required a bespoke "busy" feedback
//               signal into the arbiter to avoid overwriting unaccepted
//               data. That was replaced with this depth-2 FIFO approach
//               (see NOC_PROJECT_SPEC.md section 6B) because it reuses
//               the FIFO's existing `in_ready` signal as the natural
//               backpressure indicator -- the arbiter simply ANDs this
//               output's `in_ready` into its grant eligibility for that
//               output, instead of needing a new dedicated busy wire.
//               This is the same FIFO structure as input_unit.sv, reused
//               here on the output side.
// =============================================================================

`timescale 1ns/1ps

module output_unit #(
    parameter int PKT_WIDTH  = 24,
    parameter int FIFO_DEPTH = 2
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // ---- From crossbar (write side of internal FIFO) ----
    input  logic                  xbar_valid_strobe,   // crossbar says: data below is for me this cycle
    input  logic [PKT_WIDTH-1:0]  xbar_data,
    output logic                  xbar_can_accept,     // fed back to this output's arbiter_unit grant eligibility

    // ---- To downstream receiver (next router's input_unit, or local core) ----
    output logic                  out_valid,
    input  logic                  out_ready,
    output logic [PKT_WIDTH-1:0]  out_data
);

    localparam int PTR_W = $clog2(FIFO_DEPTH) + 1;

    logic [PTR_W-1:0] wr_ptr, rd_ptr;
    logic [PKT_WIDTH-1:0] mem [0:FIFO_DEPTH-1];

    wire [PTR_W-2:0] wr_idx = wr_ptr[PTR_W-2:0];
    wire [PTR_W-2:0] rd_idx = rd_ptr[PTR_W-2:0];

    logic fifo_full, fifo_empty;
    assign fifo_empty = (wr_ptr == rd_ptr);
    assign fifo_full  = (wr_idx == rd_idx) && (wr_ptr[PTR_W-1] != rd_ptr[PTR_W-1]);

    // -------------------------------------------------------------------------
    // Write side: driven by the crossbar. xbar_can_accept is this FIFO's
    // in_ready, exposed under a name that makes its purpose at this point
    // in the pipeline clear (it is what the arbiter checks before granting).
    // -------------------------------------------------------------------------
    assign xbar_can_accept = !fifo_full;

    wire wr_en = xbar_valid_strobe && xbar_can_accept;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
        end else if (wr_en) begin
            mem[wr_idx] <= xbar_data;
            wr_ptr      <= wr_ptr + 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // Read side: standard valid/ready toward the downstream receiver.
    // -------------------------------------------------------------------------
    assign out_valid = !fifo_empty;
    assign out_data  = mem[rd_idx];

    wire rd_en = out_valid && out_ready;

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

    // The crossbar must never push into a full output FIFO -- this is the
    // exact condition xbar_can_accept exists to prevent; if this ever
    // fires, the arbiter-to-output wiring at the noc_router.sv top level
    // has a bug (an output was granted despite in_ready being low).
    property no_write_when_full;
        @(posedge clk) disable iff (!rst_n)
        fifo_full |-> !wr_en;
    endproperty
    assert property (no_write_when_full)
        else $error("[OUTPUT_UNIT] Crossbar wrote into a full output FIFO");

    // valid must not deassert before a successful handshake
    property valid_stable_until_handshake;
        @(posedge clk) disable iff (!rst_n)
        (out_valid && !out_ready) |=> out_valid;
    endproperty
    assert property (valid_stable_until_handshake)
        else $error("[OUTPUT_UNIT] out_valid dropped without handshake");

    `endif
    // pragma translate_on

endmodule : output_unit

