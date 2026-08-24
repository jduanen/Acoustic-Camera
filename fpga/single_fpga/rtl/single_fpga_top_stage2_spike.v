`timescale 1ns / 1ps

// Stage 2 feasibility spike: combines the real 96-channel CIC+FIR pipeline
// (single_fpga_pipeline_top.v, itself wrapping gbe_packetizer.v) with the
// vendored 1G RGMII MAC (third_party/verilog-ethernet/eth_mac_1g_rgmii.v),
// targeted at real post-place-and-route utilization -- nothing else. Goal:
// an honest number for "pipeline + packetizer + MAC" total, to compare
// against Stage 1's own 43.52% baseline. See the project's plan file,
// "Single-FPGA hub (ALINX AC7200 module)" section, Stage 2.
//
// NOT a real board top, same disclaimer as single_fpga_top_spike.v: no
// PDM_CLK generation/POR sequencing, no XDC/pin constraints (Vivado's
// placer auto-assigns package pins to unconstrained ports -- standard,
// low-risk technique for a utilization-only build, same as Stage 1), and
// no real RGMII clock architecture -- `rgmii_rx_*`/`gtx_clk`/`gtx_rst` are
// plain top-level input ports with no real PHY or oscillator behind them
// for this build.
//
// USE_CLK90="FALSE" (real, already-supported vendored-MAC parameter, not a
// hack -- rgmii_phy_if.v's own generate/mux just selects `clk` instead of
// `clk90` for the RGMII TX ODDR when this is FALSE, so `gtx_clk90` goes
// genuinely unused). Chosen specifically to avoid needing a real MMCM/PLL
// instance to generate a 90-degree-shifted clock -- the plan file flagged
// this exact TRUE-vs-FALSE choice as unresolved pending "building the
// actual top-level integration"; for a resource-only spike with no timing
// closure being attempted at all, FALSE is the lower-risk choice (no new
// clock-generation primitive to reason about) and doesn't foreclose
// choosing TRUE later for the real board top once real RGMII timing
// closure is being attempted for real, gated on this spike's own
// utilization result same as Stage 1 was.
module single_fpga_top_stage2_spike (
    input  wire        clk,       // pipeline/PDM-rate clock (~3.072MHz on real board)
    input  wire        rst,
    input  wire [47:0] pdm_d,

    input  wire        gtx_clk,   // MAC TX reference clock (125MHz GMII rate on real board)
    input  wire        gtx_rst,

    input  wire        rgmii_rx_clk,
    input  wire [3:0]  rgmii_rxd,
    input  wire        rgmii_rx_ctl,
    output wire        rgmii_tx_clk,
    output wire [3:0]  rgmii_txd,
    output wire        rgmii_tx_ctl
);
    wire        mac_tx_clk, mac_tx_rst;
    wire [7:0]  tx_axis_tdata;
    wire        tx_axis_tvalid, tx_axis_tready, tx_axis_tlast, tx_axis_tuser;

    single_fpga_pipeline_top #(
        .DST_MAC(48'hAABBCCDDEEFF), .SRC_MAC(48'h001122334455),
        .SRC_IP(32'hC0A80002),      .DST_IP(32'hC0A80001),
        .SRC_PORT(16'd50000),       .DST_PORT(16'd50000)
    ) u_pipeline (
        .clk(clk), .rst(rst), .pdm_d(pdm_d),
        .tx_clk(mac_tx_clk), .tx_rst(mac_tx_rst),
        .tx_axis_tdata(tx_axis_tdata), .tx_axis_tvalid(tx_axis_tvalid),
        .tx_axis_tready(tx_axis_tready), .tx_axis_tlast(tx_axis_tlast),
        .tx_axis_tuser(tx_axis_tuser)
    );

    eth_mac_1g_rgmii #(
        .TARGET("XILINX"),
        .IODDR_STYLE("IODDR"),
        .CLOCK_INPUT_STYLE("BUFR"),
        .USE_CLK90("FALSE")
    ) u_mac (
        .gtx_clk(gtx_clk), .gtx_clk90(1'b0), .gtx_rst(gtx_rst),
        .rx_clk(), .rx_rst(),
        .tx_clk(mac_tx_clk), .tx_rst(mac_tx_rst),

        .tx_axis_tdata(tx_axis_tdata), .tx_axis_tvalid(tx_axis_tvalid),
        .tx_axis_tready(tx_axis_tready), .tx_axis_tlast(tx_axis_tlast),
        .tx_axis_tuser(tx_axis_tuser),

        .rx_axis_tdata(), .rx_axis_tvalid(), .rx_axis_tlast(), .rx_axis_tuser(),

        .rgmii_rx_clk(rgmii_rx_clk), .rgmii_rxd(rgmii_rxd), .rgmii_rx_ctl(rgmii_rx_ctl),
        .rgmii_tx_clk(rgmii_tx_clk), .rgmii_txd(rgmii_txd), .rgmii_tx_ctl(rgmii_tx_ctl),

        .tx_error_underflow(), .rx_error_bad_frame(), .rx_error_bad_fcs(), .speed(),

        .cfg_ifg(8'd12), .cfg_tx_enable(1'b1), .cfg_rx_enable(1'b1)
    );
endmodule
