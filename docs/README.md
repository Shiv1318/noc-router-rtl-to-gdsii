# NoC Router: RTL to GDSII Implementation

A complete **RTL-to-GDSII implementation** of a **5-Port Network-on-Chip (NoC) Router** using **SystemVerilog** and the **Qflow ASIC Design Flow**.

---

## Project Overview

This project demonstrates the complete ASIC implementation flow of a Network-on-Chip (NoC) Router, starting from RTL design and ending with the final GDSII layout.

The project includes:

- RTL Design in SystemVerilog
- Functional Verification
- Logic Synthesis
- Physical Design
- Static Timing Analysis
- Final GDSII Layout Generation

---

## RTL to GDSII Design Flow

```
RTL Design (SystemVerilog)
        │
        ▼
Functional Verification
        │
        ▼
Logic Synthesis (Yosys)
        │
        ▼
Technology Mapping
        │
        ▼
Floorplanning
        │
        ▼
Placement (GrayWolf)
        │
        ▼
Routing (Qrouter)
        │
        ▼
Static Timing Analysis (STA)
        │
        ▼
DRC & LVS Verification
        │
        ▼
GDSII Generation (Magic VLSI)
```

---

## Repository Structure



## Features

- 5-Port Network-on-Chip Router
- Modular SystemVerilog RTL Design
- Functional Verification using Testbench
- Complete RTL-to-GDSII ASIC Flow
- Physical Design Reports
- Final GDSII Layout

---

## Tools Used

- SystemVerilog
- Verilator
- GTKWave
- Yosys
- Qflow
- GrayWolf
- Qrouter
- Magic VLSI

---

## Generated Outputs

- RTL Netlist
- DEF
- LEF
- SPEF
- SDF
- SPICE Netlist
- GDSII Layout
- Timing Reports
- Routing Reports

---

## Author

**Shiv Kumar**

B.Tech in Electronics & Communication Engineering (VLSI)

Jaypee Institute of Information Technology, Noida
