`timescale 1ns / 1ps

// Cluster FPGA top level (Digilent Cmod S7). Port names match
// pcb/make_schematic_multi_fpga.py's schematic net names exactly -- do not
// rename without updating the schematic generator too.
//
// Pipeline: SPOKE_CLK (now 6.144 MHz, 2x the mics' own PDM rate -- see
// clk_reset.v) -> clk_reset (buffering, POR, PDM_CLK=clk/2 divider) ->
// 12x pdm_line_sync -> 12x cic_decimator_shared (each one line's L+R
// channels, time-multiplexed on one shared arithmetic path -- see that
// module's header comment) -> 24x fir_compensator -> spoke_framer ->
// SPOKE_D0-5/SPOKE_STROBE. See fpga/cluster/SPOKE_FRAMING.md for the output
// protocol and fpga/cluster/golden/fir_design.py for the CIC->FIR
// fixed-point convention (top 24 of the CIC's 31 output bits feed the FIR).
//
// This replaces the original 24x fully-parallel pdm_line_demux/cic_decimator
// instantiation (still present as standalone, individually-tested modules --
// pdm_line_demux.v/cic_decimator.v/tb_*.v -- but no longer used here) after
// synthesis showed the parallel design needed 19,233 LUTs against the
// XC7S25's 14,600 available; sharing the CIC's adder/subtractor logic
// between each line's L/R pair (which already arrive time-interleaved on
// the same physical PDM_Dxx wire) cuts that back to fit without adding a
// PLL or changing any decimation math -- see cic_decimator_shared.v.
//
// Channel numbering: physical line `li` (0..11) -> channels 2*li (L, SEL=GND,
// falling-edge capture) and 2*li+1 (R, SEL=+1.8V, rising-edge capture) --
// same "data_line*2 + {L=0,R=1}" convention used in
// test/phase4/data_line_assignment.csv and referenced throughout
// SPOKE_FRAMING.md.
//
// SPOKE_D<n> <-> spoke_framer's spoke_d[5:0] bit mapping (this module's own
// convention, not specified upstream): SPOKE_D0 = bit 0 (LSB) .. SPOKE_D5 =
// bit 5 (MSB).
module cluster_top (
    input  wire SPOKE_CLK,
    input  wire FPGA_RESET_N,
    output wire PDM_CLK,
    input  wire PDM_D00, PDM_D01, PDM_D02, PDM_D03,
    input  wire PDM_D04, PDM_D05, PDM_D06, PDM_D07,
    input  wire PDM_D08, PDM_D09, PDM_D10, PDM_D11,
    output wire SPOKE_D0, SPOKE_D1, SPOKE_D2,
    output wire SPOKE_D3, SPOKE_D4, SPOKE_D5,
    output wire SPOKE_STROBE,
    output wire SPOKE_ALIVE
);
    localparam integer N_LINES   = 12;
    localparam integer N_CH      = 24;
    localparam integer CIC_WIDTH = 31; // Hogenauer bound: STAGES=5, R=64, IN_WIDTH=1
    localparam integer FIR_WIDTH = 24;

    wire clk, rst, pdm_phase;
    clk_reset clk_reset_inst (
        .spoke_clk(SPOKE_CLK),
        .fpga_reset_n(FPGA_RESET_N),
        .clk(clk),
        .pdm_clk(PDM_CLK),
        .pdm_phase(pdm_phase),
        .rst(rst),
        .spoke_alive(SPOKE_ALIVE)
    );

    wire [N_LINES-1:0] pdm_d;
    assign pdm_d[0]  = PDM_D00;
    assign pdm_d[1]  = PDM_D01;
    assign pdm_d[2]  = PDM_D02;
    assign pdm_d[3]  = PDM_D03;
    assign pdm_d[4]  = PDM_D04;
    assign pdm_d[5]  = PDM_D05;
    assign pdm_d[6]  = PDM_D06;
    assign pdm_d[7]  = PDM_D07;
    assign pdm_d[8]  = PDM_D08;
    assign pdm_d[9]  = PDM_D09;
    assign pdm_d[10] = PDM_D10;
    assign pdm_d[11] = PDM_D11;

    wire [N_LINES-1:0] pdm_bit, pdm_line_phase;

    wire [N_CH*CIC_WIDTH-1:0] cic_out_flat;
    wire [N_CH-1:0] cic_valid; // channel c = 2*line + {0=L,1=R}

    genvar li;

    generate
        for (li = 0; li < N_LINES; li = li + 1) begin : g_pdm
            pdm_line_sync u_pdm (
                .clk(clk), .rst(rst),
                .pdm_phase(pdm_phase), .pdm_d(pdm_d[li]),
                .bit_r(pdm_bit[li]), .phase_r(pdm_line_phase[li])
            );

            cic_decimator_shared #(.STAGES(5), .R(64), .IN_WIDTH(1)) u_cic (
                .clk(clk), .rst(rst),
                .bit_in(pdm_bit[li]), .phase(pdm_line_phase[li]),
                .data_out_l(cic_out_flat[CIC_WIDTH*(2*li)   +: CIC_WIDTH]),
                .valid_l   (cic_valid[2*li]),
                .data_out_r(cic_out_flat[CIC_WIDTH*(2*li+1) +: CIC_WIDTH]),
                .valid_r   (cic_valid[2*li+1])
            );
        end
    endgenerate

    wire [N_CH*FIR_WIDTH-1:0] fir_out_flat;
    wire [N_CH-1:0] fir_valid;

    genvar c;
    generate
        for (c = 0; c < N_CH; c = c + 1) begin : g_fir
            fir_compensator #(.COEFF_MEM_FILE("../vectors/fir_coeffs.mem")) u_fir (
                .clk(clk), .rst(rst),
                .valid_in(cic_valid[c]),
                // top 24 of the CIC's 31 output bits -- see fir_design.py's
                // module docstring for the fixed-point convention.
                .data_in(cic_out_flat[CIC_WIDTH*c + 7 +: FIR_WIDTH]),
                .valid_out(fir_valid[c]),
                .data_out(fir_out_flat[FIR_WIDTH*c +: FIR_WIDTH])
            );
        end
    endgenerate

    // Unlike the old fully-parallel design, cic_valid pulses are NOT all
    // simultaneous any more: every line's L channel (even index) completes
    // its window together on one clk cycle, and every line's R channel (odd
    // index) completes together on the *next* clk cycle (they're
    // time-multiplexed on cic_decimator_shared's one shared arithmetic path,
    // driven by the same global `phase` toggle -- see that module). Each
    // fir_compensator is an independent fixed-latency state machine, so that
    // 1-cycle stagger carries straight through to fir_valid: all even
    // (L) channels' valid_out pulse together, then all odd (R) channels'
    // valid_out pulse together 1 cycle later. frame_start must key off an
    // *odd* channel (the later group) so every channel's fir_out_flat data
    // (held stable since its own valid_out, not just during the pulse) is
    // already settled by the time spoke_framer samples it.
    wire [5:0] spoke_d_int;
    spoke_framer u_framer (
        .clk(clk), .rst(rst),
        .frame_start(fir_valid[1]),
        .ch_data_flat(fir_out_flat),
        .spoke_d(spoke_d_int),
        .spoke_strobe(SPOKE_STROBE)
    );

    assign SPOKE_D0 = spoke_d_int[0];
    assign SPOKE_D1 = spoke_d_int[1];
    assign SPOKE_D2 = spoke_d_int[2];
    assign SPOKE_D3 = spoke_d_int[3];
    assign SPOKE_D4 = spoke_d_int[4];
    assign SPOKE_D5 = spoke_d_int[5];
endmodule
