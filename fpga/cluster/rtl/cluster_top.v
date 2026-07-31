`timescale 1ns / 1ps

// Cluster FPGA top level (Digilent Cmod S7). Port names match
// pcb/make_schematic_multi_fpga.py's schematic net names exactly -- do not
// rename without updating the schematic generator too.
//
// Pipeline: SPOKE_CLK -> clk_reset (buffering + POR) -> 12x pdm_line_demux
// (24 channels) -> 24x cic_decimator -> 24x fir_compensator -> spoke_framer
// -> SPOKE_D0-5/SPOKE_STROBE. See fpga/cluster/SPOKE_FRAMING.md for the
// output protocol and fpga/cluster/golden/fir_design.py for the CIC->FIR
// fixed-point convention (top 24 of the CIC's 31 output bits feed the FIR).
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
    output wire PDM_CLK,
    input  wire PDM_D00, PDM_D01, PDM_D02, PDM_D03,
    input  wire PDM_D04, PDM_D05, PDM_D06, PDM_D07,
    input  wire PDM_D08, PDM_D09, PDM_D10, PDM_D11,
    output wire SPOKE_D0, SPOKE_D1, SPOKE_D2,
    output wire SPOKE_D3, SPOKE_D4, SPOKE_D5,
    output wire SPOKE_STROBE
);
    localparam integer N_LINES   = 12;
    localparam integer N_CH      = 24;
    localparam integer CIC_WIDTH = 31; // Hogenauer bound: STAGES=5, R=64, IN_WIDTH=1
    localparam integer FIR_WIDTH = 24;

    wire clk, rst;
    clk_reset clk_reset_inst (
        .spoke_clk(SPOKE_CLK),
        .clk(clk),
        .pdm_clk(PDM_CLK),
        .rst(rst)
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

    wire [N_CH-1:0] pdm_ch; // channel c = 2*line + {0=L,1=R}

    genvar li, c;

    generate
        for (li = 0; li < N_LINES; li = li + 1) begin : g_pdm
            pdm_line_demux u_pdm (
                .clk(clk), .rst(rst), .pdm_d(pdm_d[li]),
                .ch_l(pdm_ch[2*li]), .ch_r(pdm_ch[2*li+1])
            );
        end
    endgenerate

    wire [N_CH*CIC_WIDTH-1:0] cic_out_flat;
    wire [N_CH-1:0] cic_valid;

    generate
        for (c = 0; c < N_CH; c = c + 1) begin : g_cic
            cic_decimator #(.STAGES(5), .R(64), .IN_WIDTH(1)) u_cic (
                .clk(clk), .rst(rst), .data_in(pdm_ch[c]),
                .data_out(cic_out_flat[CIC_WIDTH*c +: CIC_WIDTH]),
                .valid(cic_valid[c])
            );
        end
    endgenerate

    wire [N_CH*FIR_WIDTH-1:0] fir_out_flat;
    wire [N_CH-1:0] fir_valid;

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

    // All 24 FIR engines are identical state machines triggered by the same
    // simultaneous cic_valid pulses (one shared clk, no per-channel skew), so
    // any single channel's valid_out can drive the framer's frame_start.
    wire [5:0] spoke_d_int;
    spoke_framer u_framer (
        .clk(clk), .rst(rst),
        .frame_start(fir_valid[0]),
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
