// =============================================================================
// File        : noc_defs.sv
// Project     : NEXUS NoC Router
// Description : Plain `localparam` port-index constants -- replaces the
//               earlier noc_pkg::port_e enum approach. Some synthesis
//               front-ends (Yosys's Verilog-2005-based reader, even with
//               -sv) do not reliably support `module x import pkg::*; #(...)`
//               combined headers or typedef'd enums used as port/array
//               types. Plain constants + plain logic vectors are universally
//               portable across simulation (Verilator) and synthesis
//               (Yosys) tools, so this header replaces noc_pkg.sv.
//
//               Include this file (do NOT `import`) in every module that
//               previously did `import noc_pkg::*;`. Anywhere `port_e` was
//               used as a signal type, use `logic [2:0]` instead and the
//               PORT_* constants below for comparisons/assignments.
// =============================================================================
`ifndef NOC_DEFS_SV
`define NOC_DEFS_SV

localparam int PORT_N = 0;
localparam int PORT_S = 1;
localparam int PORT_E = 2;
localparam int PORT_W = 3;
localparam int PORT_L = 4;

`endif
