`timescale 1ns / 1ps

// Stage 1 feasibility spike for the single-FPGA (ALINX AC7200 module,
// XC7A200T) hub design -- see the project's plan file, "Single-FPGA hub
// (ALINX AC7200 module): RTL feasibility staging plan". Goal: an honest
// post-place-and-route LUT/register/DSP/control-set number for a
// 96-channel, fully-parallel (no CIC sharing) CIC+FIR pipeline on
// xc7a200tfbg484-1 -- nothing else. NOT a real board design: no clock
// generation (clk_reset.v-style PDM_CLK divider/POR), no GbE/host
// interface (that's Stage 2, gated on this result), no spoke framing
// (this is a single chip, no inter-chip hop exists in this architecture).
//
// Directly reuses fpga/cluster/rtl/pdm_line_demux.v, cic_decimator.v, and
// fir_compensator.v UNMODIFIED (confirmed standalone/chip-agnostic, already
// individually tested in fpga/cluster/sim/) -- referenced by relative path
// from build_spike.tcl's source list, not copied, so there's only one copy
// of each to maintain.
//
// 48 physical PDM_Dxx lines -> pdm_line_demux (L/R per line, matching the
// mic array board's real 48-signal PDM_D00-D47 I/O budget, see
// pcb/SCHEMATIC_NOTES.md) -> 96 logical channels -> cic_decimator (fully
// parallel, STAGES=5/R=64/IN_WIDTH=1, no sharing) -> fir_compensator
// (NTAPS=32/DATA_WIDTH=24/COEFF_WIDTH=18/ACC_WIDTH=48, same coefficient ROM
// file as the cluster's already-derived FIR design, loaded for real so
// synthesis can't quietly constant-fold a zero-coefficient multiply and
// understate the true LUT/DSP cost).
//
// Output side deliberately does NOT expose all 96 channels' 24-bit outputs
// as top-level ports: xc7a200tfbg484-1 has only 285 available IOBs, and
// 96*24 = 2,304 raw output bits alone would blow that budget for a reason
// that has nothing to do with the actual question being asked here. Instead
// every fir_compensator's data_out/valid_out is XOR-reduced into a narrow
// "keep-alive" bus -- meaningless as a real signal, but enough to give every
// instance's logic a real sink so opt_design can't trim it as dead. This
// reduction network is spike-only scaffolding, not real RTL -- Stage 2's
// real packetizer replaces it entirely.
module single_fpga_top_spike (
    input  wire        clk,
    input  wire        rst,
    input  wire [47:0] pdm_d,          // PDM_D00 (bit 0) .. PDM_D47 (bit 47)
    output wire [31:0] checksum_out,
    output wire        valid_any_out
);
    localparam integer N_LINES    = 48;
    localparam integer N_CH       = 96;
    localparam integer CIC_WIDTH  = 31; // IN_WIDTH(1) + STAGES(5)*clog2(R=64)(6) -- Hogenauer bound
    localparam integer FIR_WIDTH  = 24;

    wire [N_LINES-1:0] ch_l, ch_r;

    genvar li;
    generate
        for (li = 0; li < N_LINES; li = li + 1) begin : g_pdm
            pdm_line_demux u_demux (
                .clk(clk), .rst(rst),
                .pdm_d(pdm_d[li]),
                .ch_l(ch_l[li]), .ch_r(ch_r[li])
            );
        end
    endgenerate

    wire [N_CH*CIC_WIDTH-1:0] cic_out_flat;
    wire [N_CH-1:0]           cic_valid;

    genvar c;
    generate
        for (c = 0; c < N_LINES; c = c + 1) begin : g_cic
            // channel 2*c = L (even), channel 2*c+1 = R (odd) -- same
            // "line*2 + {L=0,R=1}" convention as fpga/cluster/rtl/cluster_top.v.
            cic_decimator #(.STAGES(5), .R(64), .IN_WIDTH(1)) u_cic_l (
                .clk(clk), .rst(rst),
                .data_in(ch_l[c]),
                .data_out(cic_out_flat[CIC_WIDTH*(2*c) +: CIC_WIDTH]),
                .valid(cic_valid[2*c])
            );
            cic_decimator #(.STAGES(5), .R(64), .IN_WIDTH(1)) u_cic_r (
                .clk(clk), .rst(rst),
                .data_in(ch_r[c]),
                .data_out(cic_out_flat[CIC_WIDTH*(2*c+1) +: CIC_WIDTH]),
                .valid(cic_valid[2*c+1])
            );
        end
    endgenerate

    wire [N_CH*FIR_WIDTH-1:0] fir_out_flat;
    wire [N_CH-1:0]           fir_valid;

    generate
        for (c = 0; c < N_CH; c = c + 1) begin : g_fir
            fir_compensator #(
                .COEFF_MEM_FILE("../../cluster/vectors/fir_coeffs.mem")
            ) u_fir (
                .clk(clk), .rst(rst),
                .valid_in(cic_valid[c]),
                // top 24 of the CIC's 31 output bits -- same convention as
                // cluster_top.v (see fir_design.py's fixed-point docstring).
                .data_in(cic_out_flat[CIC_WIDTH*c + 7 +: FIR_WIDTH]),
                .valid_out(fir_valid[c]),
                .data_out(fir_out_flat[FIR_WIDTH*c +: FIR_WIDTH])
            );
        end
    endgenerate

    // Spike-only output reduction -- see module header comment. XOR-reduces
    // EVERY channel's data_out (zero-extended to 32b) and valid_out into one
    // combinational word each cycle, so every one of the 96 fir_compensator
    // instances feeds a real sink and none can be trimmed as dead by
    // opt_design -- then registers that word into an accumulating checksum
    // so there's a real clocked sink too, not just a wide combinational cone.
    reg [31:0] checksum_comb;
    integer k;
    always @(*) begin
        checksum_comb = 32'd0;
        for (k = 0; k < N_CH; k = k + 1)
            checksum_comb = checksum_comb ^ {7'd0, fir_valid[k], fir_out_flat[FIR_WIDTH*k +: FIR_WIDTH]};
    end

    reg [31:0] checksum_r;
    reg        valid_any_r;
    always @(posedge clk) begin
        if (rst) begin
            checksum_r  <= 32'd0;
            valid_any_r <= 1'b0;
        end else begin
            checksum_r  <= checksum_r ^ checksum_comb;
            valid_any_r <= |fir_valid;
        end
    end
    assign checksum_out  = checksum_r;
    assign valid_any_out = valid_any_r;
endmodule
