// =============================================================================
// Module      : arbiter_unit
// Project     : NEXUS NoC Router
// Description : Round-robin arbiter for ONE output port. Up to 5 input
//               units (N, S, E, W, L) may simultaneously request the same
//               output port in the same cycle; this arbiter grants exactly
//               one of them per cycle and rotates priority afterward so
//               that no single input can starve the others (fairness).
//
//               One instance of this module is used per output port -- the
//               top-level router instantiates 5 of these (one per N/S/E/W/L
//               output), each looking at the subset of the 5 input units'
//               requests that target it.
// =============================================================================

`timescale 1ns/1ps

module arbiter_unit #(
    parameter int NUM_REQ = 5
)(
    input  logic                  clk,
    input  logic                  rst_n,

    input  logic [NUM_REQ-1:0]    req,      // one bit per input unit requesting this output
    input  logic                  out_can_accept, // this output's FIFO has room (output_unit.xbar_can_accept) -- see NOC_PROJECT_SPEC.md 6B
    output logic [NUM_REQ-1:0]    grant     // one-hot grant, at most one bit set
);

    // Gate all requests by whether the destination output FIFO has room.
    // If it doesn't, no grant is issued at all this cycle -- requesters
    // simply remain pending and are re-evaluated next cycle, exactly like
    // any other un-granted request.
    wire [NUM_REQ-1:0] req_gated = out_can_accept ? req : '0;

    // -------------------------------------------------------------------------
    // Priority pointer: points to the requester that currently has top
    // priority. After a grant, the pointer advances to one past the
    // granted requester, so that requester drops to lowest priority next
    // time -- this is the core round-robin fairness mechanism.
    // -------------------------------------------------------------------------
    logic [$clog2(NUM_REQ)-1:0] priority_ptr;

    // -------------------------------------------------------------------------
    // Rotate the request vector so that the current top-priority requester
    // appears at bit 0, find the lowest set bit (highest rotated priority),
    // then rotate the result back to the original bit positions.
    // -------------------------------------------------------------------------
    logic [NUM_REQ-1:0] req_rot;
    logic [NUM_REQ-1:0] grant_rot;
    logic [NUM_REQ-1:0] grant_comb;
    logic                any_req;

    always_comb begin
        // Rotate req_gated right by priority_ptr (so priority_ptr's bit lands at bit 0)
        req_rot = '0;
        for (int i = 0; i < NUM_REQ; i++) begin
            req_rot[i] = req_gated[(i + priority_ptr) % NUM_REQ];
        end

        // Priority-encode: pick the lowest set bit in the rotated vector
        grant_rot = '0;
        any_req   = 1'b0;
        for (int i = 0; i < NUM_REQ; i++) begin
            if (req_rot[i] && !any_req) begin
                grant_rot[i] = 1'b1;
                any_req      = 1'b1;
            end
        end

        // Rotate the grant back to original bit positions
        grant_comb = '0;
        for (int i = 0; i < NUM_REQ; i++) begin
            grant_comb[(i + priority_ptr) % NUM_REQ] = grant_rot[i];
        end
    end

    assign grant = grant_comb;

    // -------------------------------------------------------------------------
    // Priority Pointer Update
    // Advances to (granted_index + 1) so the just-served requester becomes
    // lowest priority next cycle. If nobody was granted, pointer holds.
    // -------------------------------------------------------------------------
    logic [$clog2(NUM_REQ)-1:0] granted_index;
    logic                        granted_index_valid;

    always_comb begin
        granted_index       = '0;
        granted_index_valid = 1'b0;
        for (int i = 0; i < NUM_REQ; i++) begin
            if (grant_comb[i]) begin
                granted_index       = i[$clog2(NUM_REQ)-1:0];
                granted_index_valid = 1'b1;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            priority_ptr <= '0;
        end else if (granted_index_valid) begin
            priority_ptr <= (granted_index + 1'b1) % NUM_REQ;
        end
    end

    // -------------------------------------------------------------------------
    // Assertions
    // -------------------------------------------------------------------------
    // pragma translate_off
    `ifdef ASSERTIONS_ON

    // Grant must be one-hot or all-zero (never more than one bit set)
    property grant_onehot_or_zero;
        @(posedge clk) disable iff (!rst_n)
        $onehot0(grant);
    endproperty
    assert property (grant_onehot_or_zero)
        else $error("[ARBITER_UNIT] More than one grant asserted simultaneously");

    // If there is at least one ELIGIBLE request (gated by output FIFO
    // having room), there must be exactly one grant
    property req_implies_grant;
        @(posedge clk) disable iff (!rst_n)
        (req_gated != '0) |-> (grant != '0);
    endproperty
    assert property (req_implies_grant)
        else $error("[ARBITER_UNIT] Eligible requests pending but no grant issued");

    // A granted requester's bit must actually have been requesting
    property grant_implies_req;
        @(posedge clk) disable iff (!rst_n)
        (grant != '0) |-> ((grant & req_gated) == grant);
    endproperty
    assert property (grant_implies_req)
        else $error("[ARBITER_UNIT] Granted a requester that did not request");

    // Never grant when the output cannot accept (belt-and-suspenders
    // check on top of the req_gated mechanism above)
    property no_grant_when_output_full;
        @(posedge clk) disable iff (!rst_n)
        !out_can_accept |-> (grant == '0);
    endproperty
    assert property (no_grant_when_output_full)
        else $error("[ARBITER_UNIT] Granted while output FIFO could not accept");

    `endif
    // pragma translate_on

endmodule : arbiter_unit

