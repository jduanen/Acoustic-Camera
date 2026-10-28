`timescale 1ns / 1ps

// Single-FPGA board top (ALINX AC7200 on pcb/single_fpga/front_end), the
// module xdc/single_fpga_top.xdc constrains. Ports are exactly that XDC's:
//   TCXO_CLK   in   12.288 MHz from mic_array Y1 (CON1.21, ball Y13)
//   PDM_CLK    out  3.072 MHz mic clock, TCXO/4, 50 % duty (CON1.25, AB11)
//   LED_R/G/B  out  status LED, active high (CON1.32-34, W14/AB13/Y14)
//   DATA[47:0] in   mic data lines, DATA[n] = net DATA_nn (CON3, bank 15)
//
// Clocks, power-on reset, the status LED, DATA sampling (pdm_sampler.v) and
// the 96-channel CIC+FIR+packetizer pipeline (single_fpga_pipeline_top.v).
// The RGMII MAC isn't instantiated yet: its pins have no XDC constraints and
// its 125 MHz clocks (from the module's 200 MHz SYS_CLK via an MMCM) don't
// exist. Until it is, the packetizer's MAC side is tied off (tx_clk stopped,
// held in reset, never ready), so nothing the pipeline computes reaches a pin
// and synthesis trims all of it; tb_single_fpga_top.v checks it end to end
// instead. When the MAC goes in, drive tx_clk/tx_rst/tx_axis_tready from it,
// and flip pkt_toggle in the tx_clk domain on each tx_axis_tvalid & tready &
// tlast so the LED shows streaming -- until then it's tied low and the LED
// goes YELLOW -> GREEN only.
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
    parameter integer POR_CYCLES = 16
) (
    input  wire TCXO_CLK,
    output wire PDM_CLK,
    output wire LED_R,
    output wire LED_G,
    output wire LED_B,
    input  wire [47:0] DATA
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
    BUFGCE bufgce_pipe (.I(TCXO_CLK), .CE(pipe_ce), .O(pipe_clk));

    reg [$clog2(POR_CYCLES+1)-1:0] por_cnt = 0;
    reg                            rst     = 1'b1;
    always @(posedge clk) begin
        if (por_cnt < POR_CYCLES)
            por_cnt <= por_cnt + 1'b1;
        rst <= (por_cnt < POR_CYCLES);
    end

    wire [47:0] pdm_l, pdm_r;
    wire        pdm_valid;              // pipe_clk's enable already encodes it
    pdm_sampler #(.N_LINES(48)) u_pdm_sampler (
        .clk(clk), .rst(rst),
        .pdm_div(pdm_div), .data(DATA),
        .pdm_l(pdm_l), .pdm_r(pdm_r), .pdm_valid(pdm_valid)
    );

    // MAC side tied off until eth_mac_1g_rgmii.v is in -- see header.
    wire       tx_clk = 1'b0, tx_rst = 1'b1, tx_axis_tready = 1'b0;
    wire [7:0] tx_axis_tdata;
    wire       tx_axis_tvalid, tx_axis_tlast, tx_axis_tuser;

    // Addresses are the same placeholders tb_gbe_pipeline.v and the Stage 2
    // spike use; set real ones with the MAC.
    single_fpga_pipeline_top #(
        .DST_MAC(48'hAABBCCDDEEFF), .SRC_MAC(48'h001122334455),
        .SRC_IP(32'hC0A80002),      .DST_IP(32'hC0A80001),
        .SRC_PORT(16'd50000),       .DST_PORT(16'd50000)
    ) u_pipeline (
        .clk(pipe_clk), .rst(rst), .ch_l(pdm_l), .ch_r(pdm_r),
        .tx_clk(tx_clk), .tx_rst(tx_rst),
        .tx_axis_tdata(tx_axis_tdata), .tx_axis_tvalid(tx_axis_tvalid),
        .tx_axis_tready(tx_axis_tready), .tx_axis_tlast(tx_axis_tlast),
        .tx_axis_tuser(tx_axis_tuser)
    );

    wire pkt_toggle = 1'b0; // no MAC yet -- see header

    status_led u_status_led (
        .clk(clk), .rst(rst),
        .pkt_toggle(pkt_toggle),
        .led_r(LED_R), .led_g(LED_G), .led_b(LED_B)
    );
endmodule
