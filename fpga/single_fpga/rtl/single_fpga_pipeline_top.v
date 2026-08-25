`timescale 1ns / 1ps

// Real (non-spike) 96-channel CIC+FIR pipeline + GbE packetizer top-level.
// Combines single_fpga_top_spike.v's already-passed Stage 1 pipeline wiring
// (unmodified fpga/multi_fpga/cluster/rtl/pdm_line_demux.v, cic_decimator.v,
// fir_compensator.v, reused by relative path, same as the spike) with
// gbe_packetizer.v's tx_axis_* output -- the real, buildable checkpoint this
// project's Stage 2 work stands on. single_fpga_top_spike.v itself is left
// untouched (it's the Stage 1 utilization-only checkpoint, kept for its own
// historical record -- see the plan file).
//
// Still NOT a complete board top: no clock generation (PDM_CLK divider/POR,
// see fpga/multi_fpga/cluster/rtl/clk_reset.v for the pattern this will eventually
// follow), no eth_mac_1g_rgmii.v instantiation (tx_clk/tx_rst are inputs
// here, meant to be driven by that MAC's own outputs once it's wired in),
// no RGMII PHY interface. That real board-top integration is later work;
// this module's job is just "pipeline output correctly reaches the
// packetizer's AXI-Stream input," verified by tb_gbe_pipeline.v.
module single_fpga_pipeline_top #(
    parameter [47:0] DST_MAC  = 48'h000000000000,
    parameter [47:0] SRC_MAC  = 48'h000000000000,
    parameter [31:0] SRC_IP   = 32'hC0A80002,
    parameter [31:0] DST_IP   = 32'hC0A80001,
    parameter [15:0] SRC_PORT = 16'd50000,
    parameter [15:0] DST_PORT = 16'd50000
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [47:0] pdm_d,

    input  wire        tx_clk,
    input  wire        tx_rst,
    output wire [7:0]  tx_axis_tdata,
    output wire        tx_axis_tvalid,
    input  wire        tx_axis_tready,
    output wire        tx_axis_tlast,
    output wire        tx_axis_tuser
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
            // "line*2 + {L=0,R=1}" convention as single_fpga_top_spike.v /
            // fpga/multi_fpga/cluster/rtl/cluster_top.v.
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
                .COEFF_MEM_FILE("../../multi_fpga/cluster/vectors/fir_coeffs.mem")
            ) u_fir (
                .clk(clk), .rst(rst),
                .valid_in(cic_valid[c]),
                // top 24 of the CIC's 31 output bits -- same convention as
                // single_fpga_top_spike.v / cluster_top.v (see fir_design.py's
                // fixed-point docstring).
                .data_in(cic_out_flat[CIC_WIDTH*c + 7 +: FIR_WIDTH]),
                .valid_out(fir_valid[c]),
                .data_out(fir_out_flat[FIR_WIDTH*c +: FIR_WIDTH])
            );
        end
    endgenerate

    gbe_packetizer #(
        .DST_MAC(DST_MAC), .SRC_MAC(SRC_MAC),
        .SRC_IP(SRC_IP), .DST_IP(DST_IP),
        .SRC_PORT(SRC_PORT), .DST_PORT(DST_PORT),
        .N_CH(N_CH), .FIR_WIDTH(FIR_WIDTH)
    ) u_packetizer (
        .clk(clk), .rst(rst),
        .fir_valid_in(fir_valid), .fir_data_in(fir_out_flat),
        .tx_clk(tx_clk), .tx_rst(tx_rst),
        .tx_axis_tdata(tx_axis_tdata), .tx_axis_tvalid(tx_axis_tvalid),
        .tx_axis_tready(tx_axis_tready), .tx_axis_tlast(tx_axis_tlast),
        .tx_axis_tuser(tx_axis_tuser)
    );
endmodule
