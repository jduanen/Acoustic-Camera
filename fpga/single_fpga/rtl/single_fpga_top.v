`timescale 1ns / 1ps

// Single-FPGA board top (ALINX AC7200 on pcb/single_fpga/front_end), the
// module xdc/single_fpga_top.xdc constrains. Ports are exactly that XDC's:
//   TCXO_CLK   in   12.288 MHz from mic_array Y1 (CON1.21, ball Y13)
//   PDM_CLK    out  3.072 MHz mic clock, TCXO/4, 50 % duty (CON1.25, AB11)
//   LED_R/G/B  out  status LED, active high (CON1.32-34, W14/AB13/Y14)
//   DATA[47:0] in   mic data lines, DATA[n] = net DATA_nn (CON3, banks 15/16)
//   SYS_CLK_P/N in  AC7200's own 200 MHz oscillator (R4/T4, bank 34, 1.5 V)
//   RGMII_*         to the DP83867 (front_end U3): TX in bank 16, RX in bank 15
//   PHY_RESET_N out DP83867 RESET_N (CON1.28)
//
// Clocks, power-on reset, the status LED, DATA sampling (pdm_sampler.v), the
// 96-channel CIC+FIR+packetizer pipeline (single_fpga_pipeline_top.v) and the
// vendored 1G RGMII MAC (third_party/verilog-ethernet/eth_mac_1g_rgmii.v)
// sending the packetizer's UDP frames to the PHY. TX only: received frames
// are dropped, but the MAC still needs RGMII_RXC -- it sets its own speed
// from RXC's rate, and with no RXC it falls back to 10 Mb/s.
//
// RGMII timing follows the board's DP83867 straps (pcb/SCHEMATIC_NOTES.md
// "GbE PHY, RGMII and Y1"): TX and RX clock skew both strap mode 1, so the PHY
// adds 2 ns to each clock itself. TXC therefore leaves the FPGA edge-aligned
// with its data (USE_CLK90 "FALSE": forwarded from gtx_clk by an ODDR like
// the data), and RXC arrives centred in its data, captured with no FPGA-side
// delay (BUFIO/BUFR, which needs RXC and all RX inputs in one bank -- bank
// 15). A 90-degree FPGA-side TXC shift, with the PHY's TX delay strapped off,
// left ~20 ps of margin across corners, which is why R5/R6 are now unfitted.
//
// PHY_RESET_N is held low for PHY_RST_CYCLES TCXO cycles (~10.7 ms) after
// configuration, well over the DP83867's 1 us minimum. Its straps re-latch
// on release, with this FPGA's RX inputs high-Z rather than pulled the way
// they may be during configuration. MDC/MDIO aren't driven: the PHY runs
// from its straps (autoneg on, advertising 10/100/1000). The packetizer only
// keeps up at 1000 Mb/s -- at 10/100 the MAC would back-pressure it into
// overrunning its ping-pong buffer -- so a link that negotiates lower will
// send corrupt data until MDIO is used to advertise 1000 only.
//
// The status LED's streaming indication comes from pkt_toggle, flipped in
// the MAC's tx_clk domain on the last byte of each frame accepted.
//
// TCXO_CLK's ball isn't clock-capable, so the BUFG is reached over fabric
// routing (CLOCK_DEDICATED_ROUTE FALSE in the XDC) and the divide is a plain
// counter off that BUFG -- no MMCM/PLL from this pin (see the XDC comment).
// The divider free-runs, ungated by rst, for the same reason as
// fpga/multi_fpga/cluster/rtl/clk_reset.v's: its phase after reset must not
// depend on how long POR held.
//
// Pipeline clock: the pipeline wants one L/R pair per clk at the PDM rate
// (cic_decimator.v takes a bit every clk; fir_compensator.v counts on 64 clks
// per output). pipe_clk is TCXO_CLK through a BUFGCE enabled one cycle in
// four, so it rises once per PDM period on the TCXO edge where pdm_div goes
// 2->3 -- two cycles after pdm_sampler updates pdm_l/pdm_r (on 0->1), with
// both domains on the same TCXO edges. STA sees pipe_clk as TCXO_CLK itself,
// so every path is timed at 81.38 ns, not the 325.52 ns it really has.
module single_fpga_top #(
    parameter integer POR_CYCLES     = 16,
    parameter integer PHY_RST_CYCLES = 1 << 17
) (
    input  wire        TCXO_CLK,
    output wire        PDM_CLK,
    output wire        LED_R,
    output wire        LED_G,
    output wire        LED_B,
    input  wire [47:0] DATA,

    input  wire        SYS_CLK_P,
    input  wire        SYS_CLK_N,
    output wire        RGMII_TXC,
    output wire [3:0]  RGMII_TXD,
    output wire        RGMII_TX_CTL,
    input  wire        RGMII_RXC,
    input  wire [3:0]  RGMII_RXD,
    input  wire        RGMII_RX_CTL,
    output wire        PHY_RESET_N
);
    wire clk;
    BUFG bufg_inst (.I(TCXO_CLK), .O(clk));

    reg [1:0] pdm_div = 2'd0;
    always @(posedge clk)
        pdm_div <= pdm_div + 2'd1;
    assign PDM_CLK = pdm_div[1];

    reg pipe_ce = 1'b0;                 // high for the cycle before pdm_div 2->3
    always @(posedge clk)
        pipe_ce <= (pdm_div == 2'd1);
    wire pipe_clk;
    BUFGCE #(.SIM_DEVICE("7SERIES")) bufgce_pipe (.I(TCXO_CLK), .CE(pipe_ce), .O(pipe_clk));

    reg [$clog2(POR_CYCLES+1)-1:0] por_cnt = 0;
    reg                            rst     = 1'b1;
    always @(posedge clk) begin
        if (por_cnt < POR_CYCLES)
            por_cnt <= por_cnt + 1'b1;
        rst <= (por_cnt < POR_CYCLES);
    end

    // The pipeline's reset, re-timed to change only on the TCXO edge where
    // pdm_div goes 0->1 -- the same edge as pdm_sampler's outputs -- so every
    // clk -> pipe_clk path changes two cycles from any pipe_clk edge and sits
    // under the XDC's sampler -> pipeline multicycle. Taken straight from rst,
    // the BUFG/BUFGCE skew failed hold on some placements.
    reg pipe_rst = 1'b1;
    always @(posedge clk)
        if (pdm_div == 2'd0)
            pipe_rst <= rst;

    wire [47:0] pdm_l, pdm_r;
    wire        pdm_valid;              // pipe_clk's enable already encodes it
    pdm_sampler #(.N_LINES(48)) u_pdm_sampler (
        .clk(clk), .rst(rst),
        .pdm_div(pdm_div), .data(DATA),
        .pdm_l(pdm_l), .pdm_r(pdm_r), .pdm_valid(pdm_valid)
    );

    // ---- 125 MHz RGMII clock from the 200 MHz SYS_CLK ----
    // VCO 1000 MHz (x5); 125 MHz = /8.
    wire sys_clk, mmcm_fb, gtx_clk_mmcm, mmcm_locked;
    IBUFDS ibufds_sys (.I(SYS_CLK_P), .IB(SYS_CLK_N), .O(sys_clk));
    MMCME2_BASE #(
        .CLKIN1_PERIOD(5.000),
        .DIVCLK_DIVIDE(1),
        .CLKFBOUT_MULT_F(5.000),
        .CLKOUT0_DIVIDE_F(8.000)
    ) mmcm_gtx (
        .CLKIN1(sys_clk), .CLKFBIN(mmcm_fb), .CLKFBOUT(mmcm_fb),
        .CLKOUT0(gtx_clk_mmcm), .CLKOUT1(),
        .CLKOUT0B(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
        .CLKOUT4(), .CLKOUT5(), .CLKOUT6(), .CLKFBOUTB(),
        .LOCKED(mmcm_locked), .PWRDWN(1'b0), .RST(1'b0)
    );
    wire gtx_clk;
    BUFG bufg_gtx (.I(gtx_clk_mmcm), .O(gtx_clk));

    // gtx_rst: asserted while the MMCM is unlocked, released synchronously.
    reg [3:0] gtx_rst_sync = 4'hf;
    always @(posedge gtx_clk or negedge mmcm_locked)
        if (!mmcm_locked) gtx_rst_sync <= 4'hf;
        else              gtx_rst_sync <= {gtx_rst_sync[2:0], 1'b0};
    wire gtx_rst = gtx_rst_sync[3];

    // ---- PHY reset ----
    reg [$clog2(PHY_RST_CYCLES+1)-1:0] phy_rst_cnt = 0;
    reg                                phy_reset_n = 1'b0;
    always @(posedge clk) begin
        if (phy_rst_cnt < PHY_RST_CYCLES)
            phy_rst_cnt <= phy_rst_cnt + 1'b1;
        phy_reset_n <= (phy_rst_cnt == PHY_RST_CYCLES);
    end
    assign PHY_RESET_N = phy_reset_n;

    wire       tx_clk, tx_rst, tx_axis_tready;
    wire [7:0] tx_axis_tdata;
    wire       tx_axis_tvalid, tx_axis_tlast, tx_axis_tuser;

    // Addresses are the same placeholders tb_gbe_pipeline.v and the Stage 2
    // spike use -- set the host's MAC/IP before connecting to a real network.
    single_fpga_pipeline_top #(
        .DST_MAC(48'hAABBCCDDEEFF), .SRC_MAC(48'h001122334455),
        .SRC_IP(32'hC0A80002),      .DST_IP(32'hC0A80001),
        .SRC_PORT(16'd50000),       .DST_PORT(16'd50000)
    ) u_pipeline (
        .clk(pipe_clk), .rst(pipe_rst), .ch_l(pdm_l), .ch_r(pdm_r),
        .tx_clk(tx_clk), .tx_rst(tx_rst),
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
        .tx_clk(tx_clk), .tx_rst(tx_rst),

        .tx_axis_tdata(tx_axis_tdata), .tx_axis_tvalid(tx_axis_tvalid),
        .tx_axis_tready(tx_axis_tready), .tx_axis_tlast(tx_axis_tlast),
        .tx_axis_tuser(tx_axis_tuser),

        .rx_axis_tdata(), .rx_axis_tvalid(), .rx_axis_tlast(), .rx_axis_tuser(),

        .rgmii_rx_clk(RGMII_RXC), .rgmii_rxd(RGMII_RXD), .rgmii_rx_ctl(RGMII_RX_CTL),
        .rgmii_tx_clk(RGMII_TXC), .rgmii_txd(RGMII_TXD), .rgmii_tx_ctl(RGMII_TX_CTL),

        .tx_error_underflow(), .rx_error_bad_frame(), .rx_error_bad_fcs(), .speed(),

        .cfg_ifg(8'd12), .cfg_tx_enable(1'b1), .cfg_rx_enable(1'b0)
    );

    reg pkt_toggle = 1'b0;              // tx_clk domain; status_led syncs it
    always @(posedge tx_clk)
        if (tx_axis_tvalid && tx_axis_tready && tx_axis_tlast)
            pkt_toggle <= ~pkt_toggle;

    status_led u_status_led (
        .clk(clk), .rst(rst),
        .pkt_toggle(pkt_toggle),
        .led_r(LED_R), .led_g(LED_G), .led_b(LED_B)
    );
endmodule
