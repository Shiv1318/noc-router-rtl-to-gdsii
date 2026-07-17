// =============================================================================
// Module      : crossbar
// Project     : NEXUS NoC Router
// Description : The 5x5 interconnect fabric. Based on the (registered)
//               grant vectors from all 5 per-output arbiters, connects the
//               correct input_unit's packet data to the correct
//               output_unit.
//
// PIPELINING  : The grant signals coming into this module are registered
//               HERE, at the crossbar's input, before driving the output
//               mux (see NOC_PROJECT_SPEC.md section 6A) -- only the
//               control (grant) is registered, the datapath mux itself
//               stays combinational.
//
// PORT STYLE  : Every port is an individually-named signal (in_data_n,
//               in_data_s, ... grant_n[NUM_PORTS-1:0], ...) rather than an
//               unpacked array, as a deliberate Yosys-synthesis-safety
//               choice made before any synthesis attempt (see
//               NOC_PROJECT_SPEC.md section 6C). Internally, the module
//               still uses arrays/loops for compactness -- only the
//               module port list itself avoids unpacked-array typing.
// =============================================================================

`timescale 1ns/1ps

module crossbar
  
#(
    parameter int PKT_WIDTH = 24,
    parameter int NUM_PORTS = 5   // N, S, E, W, L
)(
    input  logic                  clk,
    input  logic                  rst_n,

    // ---- Packet data from each input_unit (combinational, head-of-FIFO) ----
    input  logic [PKT_WIDTH-1:0]  in_data_n,
    input  logic [PKT_WIDTH-1:0]  in_data_s,
    input  logic [PKT_WIDTH-1:0]  in_data_e,
    input  logic [PKT_WIDTH-1:0]  in_data_w,
    input  logic [PKT_WIDTH-1:0]  in_data_l,

    // ---- Grant vectors, ONE PER OUTPUT PORT, each NUM_PORTS bits wide:
    //      grant_X[i] = 1 means input i is granted to output X this cycle.
    //      Combinational, straight from the 5 arbiter_unit instances --
    //      this module registers them internally. ----
    input  logic [NUM_PORTS-1:0]  grant_n,
    input  logic [NUM_PORTS-1:0]  grant_s,
    input  logic [NUM_PORTS-1:0]  grant_e,
    input  logic [NUM_PORTS-1:0]  grant_w,
    input  logic [NUM_PORTS-1:0]  grant_l,

    // ---- Data driven out to each output_unit, plus a "this data is
    //      valid this cycle" strobe per output, both reflecting the
    //      REGISTERED grant (delayed one cycle from grant_X above) ----
    output logic [PKT_WIDTH-1:0]  out_data_n,
    output logic [PKT_WIDTH-1:0]  out_data_s,
    output logic [PKT_WIDTH-1:0]  out_data_e,
    output logic [PKT_WIDTH-1:0]  out_data_w,
    output logic [PKT_WIDTH-1:0]  out_data_l,

    output logic                  out_valid_strobe_n,
    output logic                  out_valid_strobe_s,
    output logic                  out_valid_strobe_e,
    output logic                  out_valid_strobe_w,
    output logic                  out_valid_strobe_l
);

    // -------------------------------------------------------------------------
    // Internally, pack the named ports into arrays purely for compact
    // looped logic. These are ordinary internal signals, not ports, so
    // they are not subject to the port-typing concern in section 6C.
    // Index convention matches noc_pkg::port_e: N=0, S=1, E=2, W=3, L=4.
    // -------------------------------------------------------------------------
    logic [PKT_WIDTH-1:0] in_data [NUM_PORTS];
    logic [NUM_PORTS-1:0] grant_in [NUM_PORTS];   // grant_in[o] = grant_<o> vector
    logic [PKT_WIDTH-1:0] out_data [NUM_PORTS];
    logic [NUM_PORTS-1:0] out_strobe;

    always_comb begin
        in_data[0] = in_data_n; in_data[1] = in_data_s; in_data[2] = in_data_e;
        in_data[3] = in_data_w; in_data[4] = in_data_l;

        grant_in[0] = grant_n; grant_in[1] = grant_s; grant_in[2] = grant_e;
        grant_in[3] = grant_w; grant_in[4] = grant_l;
    end

    assign out_data_n = out_data[0];
    assign out_data_s = out_data[1];
    assign out_data_e = out_data[2];
    assign out_data_w = out_data[3];
    assign out_data_l = out_data[4];

    assign out_valid_strobe_n = out_strobe[0];
    assign out_valid_strobe_s = out_strobe[1];
    assign out_valid_strobe_e = out_strobe[2];
    assign out_valid_strobe_w = out_strobe[3];
    assign out_valid_strobe_l = out_strobe[4];

    // -------------------------------------------------------------------------
    // grant_q is COMBINATIONAL (same cycle as grant_in), not registered.
    // This must stay same-cycle as the FIFO pop in input_unit (which is
    // driven directly off the unregistered arbiter grant in noc_router.sv),
    // otherwise the crossbar reads stale/popped FIFO data one cycle late.
    // -------------------------------------------------------------------------
    logic [NUM_PORTS-1:0] grant_q [NUM_PORTS];

    always_comb begin
        for (int o = 0; o < NUM_PORTS; o++) begin
            grant_q[o] = grant_in[o];
        end
    end

    // -------------------------------------------------------------------------
    // Combinational Mux: for each output port, select the input_unit data
    // whose bit is set in that output's grant vector. Since arbiter_unit
    // guarantees one-hot-or-zero grants, at most one input is selected.
    // -------------------------------------------------------------------------
    always_comb begin
        for (int o = 0; o < NUM_PORTS; o++) begin
            out_data[o]   = '0;
            out_strobe[o] = 1'b0;
            for (int i = 0; i < NUM_PORTS; i++) begin
                if (grant_q[o][i]) begin
                    out_data[o]   = in_data[i];
                    out_strobe[o] = 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Assertions
    // -------------------------------------------------------------------------
    // pragma translate_off
    `ifdef ASSERTIONS_ON

    // No-duplication check: the same input's data must not be presented
    // as valid on two different outputs in the same cycle.
    genvar go1, go2;
    generate
        for (go1 = 0; go1 < NUM_PORTS; go1++) begin : g_dup_outer
            for (go2 = go1+1; go2 < NUM_PORTS; go2++) begin : g_dup_inner
                property no_input_double_granted;
                    @(posedge clk) disable iff (!rst_n)
                    !(|(grant_q[go1] & grant_q[go2]));
                endproperty
                assert property (no_input_double_granted)
                    else $error("[CROSSBAR] Same input granted to two outputs simultaneously (%0d,%0d)", go1, go2);
            end
        end
    endgenerate

    `endif
    // pragma translate_on

endmodule : crossbar


