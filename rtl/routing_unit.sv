// =============================================================================
// Module      : routing_unit
// Project     : NEXUS NoC Router
// Description : Combinational XY (dimension-order) routing decision logic.
// =============================================================================
`timescale 1ns/1ps
module routing_unit
#(
    parameter int COORD_W = 2
)(
    input  logic [COORD_W-1:0] dest_x,
    input  logic [COORD_W-1:0] dest_y,
    input  logic [COORD_W-1:0] my_x,
    input  logic [COORD_W-1:0] my_y,
    input  logic               pkt_valid,
    output logic [2:0]         req_port,
    output logic               req_valid
);
    localparam logic [2:0] PORT_N = 3'd0;
    localparam logic [2:0] PORT_S = 3'd1;
    localparam logic [2:0] PORT_E = 3'd2;
    localparam logic [2:0] PORT_W = 3'd3;
    localparam logic [2:0] PORT_L = 3'd4;

    always_comb begin
        req_port  = PORT_L;
        req_valid = pkt_valid;
        if (!pkt_valid) begin
            req_valid = 1'b0;
        end
        else if (dest_x > my_x) begin
            req_port = PORT_E;
        end
        else if (dest_x < my_x) begin
            req_port = PORT_W;
        end
        else if (dest_y > my_y) begin
            req_port = PORT_S;
        end
        else if (dest_y < my_y) begin
            req_port = PORT_N;
        end
        else begin
            req_port = PORT_L;
        end
    end
endmodule : routing_unit
