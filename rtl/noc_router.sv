// =============================================================================
// Module      : noc_router
// Project     : NEXUS NoC Router
// Description : Top-level single router. Instantiates and wires together:
//                 - 5x input_unit   (one per port: N, S, E, W, L)
//                 - 5x routing_unit (one per input, decides target output)
//                 - 5x arbiter_unit (one per OUTPUT, arbitrates among the
//                                    up-to-5 inputs that may request it)
//                 - 1x crossbar     (moves granted packet data to outputs)
//                 - 5x output_unit  (one per port, depth-2 FIFO + handshake
//                                    to the next hop)
//
//               MY_X / MY_Y identify this router's position in the mesh
//               and are fed into every routing_unit instance so the same
//               RTL module works unmodified at every mesh position (see
//               NOC_PROJECT_SPEC.md section 5).
//
// PORT STYLE  : This module's own external ports are individually named
//               (n_in_data, s_in_data, ... not an unpacked array) per
//               NOC_PROJECT_SPEC.md section 6C. Internal wiring between
//               sub-modules uses arrays (req_matrix, grant_matrix, etc.)
//               purely for compact generate-loop logic -- these are
//               internal signals, not ports, so the section 6C concern
//               does not apply to them.
// =============================================================================

`timescale 1ns/1ps

module noc_router
  
#(
    parameter int MY_X        = 0,
    parameter int MY_Y        = 0,
    parameter int COORD_W     = 2,
    parameter int PAYLOAD_W   = 16,
    parameter int PKT_WIDTH   = 2*COORD_W*2 + PAYLOAD_W,  // dest_x+dest_y+src_x+src_y+payload
    parameter int IN_FIFO_DEPTH  = 4,
    parameter int OUT_FIFO_DEPTH = 2
)(
    input  logic clk,
    input  logic rst_n,

    // ---- Port N ----
    input  logic                 n_in_valid,
    output logic                 n_in_ready,
    input  logic [PKT_WIDTH-1:0] n_in_data,
    output logic                 n_out_valid,
    input  logic                 n_out_ready,
    output logic [PKT_WIDTH-1:0] n_out_data,

    // ---- Port S ----
    input  logic                 s_in_valid,
    output logic                 s_in_ready,
    input  logic [PKT_WIDTH-1:0] s_in_data,
    output logic                 s_out_valid,
    input  logic                 s_out_ready,
    output logic [PKT_WIDTH-1:0] s_out_data,

    // ---- Port E ----
    input  logic                 e_in_valid,
    output logic                 e_in_ready,
    input  logic [PKT_WIDTH-1:0] e_in_data,
    output logic                 e_out_valid,
    input  logic                 e_out_ready,
    output logic [PKT_WIDTH-1:0] e_out_data,

    // ---- Port W ----
    input  logic                 w_in_valid,
    output logic                 w_in_ready,
    input  logic [PKT_WIDTH-1:0] w_in_data,
    output logic                 w_out_valid,
    input  logic                 w_out_ready,
    output logic [PKT_WIDTH-1:0] w_out_data,

    // ---- Port L (Local: connects to a core/traffic source-sink) ----
    input  logic                 l_in_valid,
    output logic                 l_in_ready,
    input  logic [PKT_WIDTH-1:0] l_in_data,
    output logic                 l_out_valid,
    input  logic                 l_out_ready,
    output logic [PKT_WIDTH-1:0] l_out_data
);

    localparam int NUM_PORTS = 5;   // index convention: N=0, S=1, E=2, W=3, L=4

    // -------------------------------------------------------------------------
    // Packet field extraction helper. Field layout (MSB to LSB), matching
    // NOC_PROJECT_SPEC.md section 2:
    //   [dest_x(COORD_W)][dest_y(COORD_W)][src_x(COORD_W)][src_y(COORD_W)][payload(PAYLOAD_W)]
    // -------------------------------------------------------------------------
    function automatic logic [COORD_W-1:0] pkt_dest_x(input logic [PKT_WIDTH-1:0] pkt);
        pkt_dest_x = pkt[PKT_WIDTH-1 -: COORD_W];
    endfunction

    function automatic logic [COORD_W-1:0] pkt_dest_y(input logic [PKT_WIDTH-1:0] pkt);
        pkt_dest_y = pkt[PKT_WIDTH-1-COORD_W -: COORD_W];
    endfunction

    // -------------------------------------------------------------------------
    // Internal per-port arrays (NOT module ports -- safe per section 6C).
    // -------------------------------------------------------------------------
    logic                  iu_in_valid   [NUM_PORTS];
    logic                  iu_in_ready   [NUM_PORTS];
    logic [PKT_WIDTH-1:0]  iu_in_data    [NUM_PORTS];
    logic                  iu_out_valid  [NUM_PORTS];  // head-of-FIFO valid (pkt_valid for routing)
    logic                  iu_out_ready  [NUM_PORTS];  // "this input was granted this cycle"
    logic [PKT_WIDTH-1:0]  iu_out_data   [NUM_PORTS];

    logic [2:0]            ru_req_port   [NUM_PORTS];
    logic                  ru_req_valid  [NUM_PORTS];

    logic [NUM_PORTS-1:0]  req_matrix    [NUM_PORTS];   // req_matrix[o][i]
    logic [NUM_PORTS-1:0]  grant_matrix  [NUM_PORTS];    // grant_matrix[o][i]
    logic                  out_can_accept[NUM_PORTS];

    logic                  ou_xbar_valid_strobe [NUM_PORTS];
    logic [PKT_WIDTH-1:0]  ou_xbar_data         [NUM_PORTS];
    logic                  ou_out_valid         [NUM_PORTS];
    logic                  ou_out_ready         [NUM_PORTS];
    logic [PKT_WIDTH-1:0]  ou_out_data          [NUM_PORTS];

    // -------------------------------------------------------------------------
    // Build the request matrix: req_matrix[o][i] = 1 iff input i's
    // requested port equals o AND that input actually has a valid packet.
    // -------------------------------------------------------------------------
    always_comb begin
        for (int o = 0; o < NUM_PORTS; o++) begin
            for (int i = 0; i < NUM_PORTS; i++) begin
                req_matrix[o][i] = ru_req_valid[i] && (int'(ru_req_port[i]) == o);
            end
        end
    end

    // -------------------------------------------------------------------------
    // iu_out_ready[i] is high iff input i was granted by whichever single
    // output it requested this cycle (routing_unit requests exactly one
    // output per cycle by construction).
    // -------------------------------------------------------------------------
    always_comb begin
        for (int i = 0; i < NUM_PORTS; i++) begin
            iu_out_ready[i] = ru_req_valid[i] && grant_matrix[int'(ru_req_port[i])][i];
        end
    end

    // -------------------------------------------------------------------------
    // Generate: 5x input_unit + 5x routing_unit (one pair per port)
    // -------------------------------------------------------------------------
    genvar gp;
    generate
        for (gp = 0; gp < NUM_PORTS; gp++) begin : g_input_side

            input_unit #(
                .PKT_WIDTH  (PKT_WIDTH),
                .FIFO_DEPTH (IN_FIFO_DEPTH)
            ) u_input_unit (
                .clk       (clk),
                .rst_n     (rst_n),
                .in_valid  (iu_in_valid[gp]),
                .in_ready  (iu_in_ready[gp]),
                .in_data   (iu_in_data[gp]),
                .out_valid (iu_out_valid[gp]),
                .out_ready (iu_out_ready[gp]),
                .out_data  (iu_out_data[gp]),
                .fifo_full (),
                .fifo_empty()
            );

            routing_unit #(
                .COORD_W (COORD_W)
            ) u_routing_unit (
                .dest_x    (pkt_dest_x(iu_out_data[gp])),
                .dest_y    (pkt_dest_y(iu_out_data[gp])),
                .my_x      (COORD_W'(MY_X)),
                .my_y      (COORD_W'(MY_Y)),
                .pkt_valid (iu_out_valid[gp]),
                .req_port  (ru_req_port[gp]),
                .req_valid (ru_req_valid[gp])
            );

        end
    endgenerate

    // -------------------------------------------------------------------------
    // Generate: 5x arbiter_unit (one per OUTPUT port o), each looking at
    // column o of the request matrix.
    // -------------------------------------------------------------------------
    generate
        for (gp = 0; gp < NUM_PORTS; gp++) begin : g_arbiters

            arbiter_unit #(
                .NUM_REQ (NUM_PORTS)
            ) u_arbiter_unit (
                .clk            (clk),
                .rst_n          (rst_n),
                .req            (req_matrix[gp]),
                .out_can_accept (out_can_accept[gp]),
                .grant          (grant_matrix[gp])
            );

        end
    endgenerate

    // -------------------------------------------------------------------------
    // Crossbar: single instance. Its ports are individually named (section
    // 6C), so each is connected explicitly here rather than via array
    // port mapping.
    // -------------------------------------------------------------------------
    crossbar #(
        .PKT_WIDTH (PKT_WIDTH),
        .NUM_PORTS (NUM_PORTS)
    ) u_crossbar (
        .clk   (clk),
        .rst_n (rst_n),

        .in_data_n (iu_out_data[0]),
        .in_data_s (iu_out_data[1]),
        .in_data_e (iu_out_data[2]),
        .in_data_w (iu_out_data[3]),
        .in_data_l (iu_out_data[4]),

        .grant_n (grant_matrix[0]),
        .grant_s (grant_matrix[1]),
        .grant_e (grant_matrix[2]),
        .grant_w (grant_matrix[3]),
        .grant_l (grant_matrix[4]),

        .out_data_n (ou_xbar_data[0]),
        .out_data_s (ou_xbar_data[1]),
        .out_data_e (ou_xbar_data[2]),
        .out_data_w (ou_xbar_data[3]),
        .out_data_l (ou_xbar_data[4]),

        .out_valid_strobe_n (ou_xbar_valid_strobe[0]),
        .out_valid_strobe_s (ou_xbar_valid_strobe[1]),
        .out_valid_strobe_e (ou_xbar_valid_strobe[2]),
        .out_valid_strobe_w (ou_xbar_valid_strobe[3]),
        .out_valid_strobe_l (ou_xbar_valid_strobe[4])
    );

    // -------------------------------------------------------------------------
    // Generate: 5x output_unit
    // -------------------------------------------------------------------------
    generate
        for (gp = 0; gp < NUM_PORTS; gp++) begin : g_output_side

            output_unit #(
                .PKT_WIDTH  (PKT_WIDTH),
                .FIFO_DEPTH (OUT_FIFO_DEPTH)
            ) u_output_unit (
                .clk               (clk),
                .rst_n             (rst_n),
                .xbar_valid_strobe (ou_xbar_valid_strobe[gp]),
                .xbar_data         (ou_xbar_data[gp]),
                .xbar_can_accept   (out_can_accept[gp]),
                .out_valid         (ou_out_valid[gp]),
                .out_ready         (ou_out_ready[gp]),
                .out_data          (ou_out_data[gp])
            );

        end
    endgenerate

    // -------------------------------------------------------------------------
    // Top-level port <-> internal-array wiring.
    // Index mapping (must match noc_pkg::port_e): N=0, S=1, E=2, W=3, L=4
    // -------------------------------------------------------------------------

    // Port N (index 0)
    assign iu_in_valid[0]  = n_in_valid;
    assign n_in_ready       = iu_in_ready[0];
    assign iu_in_data[0]   = n_in_data;
    assign n_out_valid      = ou_out_valid[0];
    assign ou_out_ready[0] = n_out_ready;
    assign n_out_data       = ou_out_data[0];

    // Port S (index 1)
    assign iu_in_valid[1]  = s_in_valid;
    assign s_in_ready       = iu_in_ready[1];
    assign iu_in_data[1]   = s_in_data;
    assign s_out_valid      = ou_out_valid[1];
    assign ou_out_ready[1] = s_out_ready;
    assign s_out_data       = ou_out_data[1];

    // Port E (index 2)
    assign iu_in_valid[2]  = e_in_valid;
    assign e_in_ready       = iu_in_ready[2];
    assign iu_in_data[2]   = e_in_data;
    assign e_out_valid      = ou_out_valid[2];
    assign ou_out_ready[2] = e_out_ready;
    assign e_out_data       = ou_out_data[2];

    // Port W (index 3)
    assign iu_in_valid[3]  = w_in_valid;
    assign w_in_ready       = iu_in_ready[3];
    assign iu_in_data[3]   = w_in_data;
    assign w_out_valid      = ou_out_valid[3];
    assign ou_out_ready[3] = w_out_ready;
    assign w_out_data       = ou_out_data[3];

    // Port L (index 4)
    assign iu_in_valid[4]  = l_in_valid;
    assign l_in_ready       = iu_in_ready[4];
    assign iu_in_data[4]   = l_in_data;
    assign l_out_valid      = ou_out_valid[4];
    assign ou_out_ready[4] = l_out_ready;
    assign l_out_data       = ou_out_data[4];

    // -------------------------------------------------------------------------
    // Assertions
    // -------------------------------------------------------------------------
    // pragma translate_off
    `ifdef ASSERTIONS_ON

    // Each input must request at most one output per cycle.
    genvar gi;
    generate
        for (gi = 0; gi < NUM_PORTS; gi++) begin : g_single_req_check
            property single_output_request;
                @(posedge clk) disable iff (!rst_n)
                $onehot0({req_matrix[0][gi], req_matrix[1][gi], req_matrix[2][gi],
                          req_matrix[3][gi], req_matrix[4][gi]});
            endproperty
            assert property (single_output_request)
                else $error("[NOC_ROUTER] Input %0d requested more than one output simultaneously", gi);
        end
    endgenerate

    `endif
    // pragma translate_on

endmodule : noc_router

