`timescale 1ns / 1ps

// Single-FPGA board top (ALINX AC7200 on pcb/single_fpga/front_end), the
// module xdc/single_fpga_top.xdc constrains. Ports are exactly that XDC's:
//   TCXO_CLK   in   12.288 MHz from mic_array Y1 (CON1.21, ball Y13)
//   PDM_CLK    out  3.072 MHz mic clock, TCXO/4, 50 % duty (CON1.25, AB11)
//   LED_R/G/B  out  status LED, active high (CON1.32-34, W14/AB13/Y14)
//   DATA[47:0] in   mic data lines, DATA[n] = net DATA_nn (CON3, bank 15)
//
// Currently clocks, power-on reset, the status LED and DATA sampling
// (pdm_sampler.v: one L and one R bit per line per PDM period, with a
// pdm_valid strobe). Nothing consumes the samples yet, so synthesis trims
// the sampler: the pipeline (single_fpga_pipeline_top.v) needs a clock-enable
// path to run on pdm_valid (its pdm_line_demux/cic_decimator assume clk =
// PDM_CLK), the RGMII pins have no XDC constraints, and the MAC's
// tx_clk/tx_rst aren't generated. When they are, drive pkt_toggle
// from the tx_clk domain (flip it on each tx_axis_tvalid & tready & tlast)
// so the LED shows streaming -- until then it is tied low and the LED goes
// YELLOW -> GREEN only.
//
// TCXO_CLK's ball isn't clock-capable, so the BUFG is reached over fabric
// routing (CLOCK_DEDICATED_ROUTE FALSE in the XDC) and the divide is a plain
// counter off that BUFG -- no MMCM/PLL from this pin (see the XDC comment).
// The divider free-runs, ungated by rst, for the same reason as
// fpga/multi_fpga/cluster/rtl/clk_reset.v's: its phase after reset must not
// depend on how long POR held.
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

    reg [$clog2(POR_CYCLES+1)-1:0] por_cnt = 0;
    reg                            rst     = 1'b1;
    always @(posedge clk) begin
        if (por_cnt < POR_CYCLES)
            por_cnt <= por_cnt + 1'b1;
        rst <= (por_cnt < POR_CYCLES);
    end

    wire [47:0] pdm_l, pdm_r;
    wire        pdm_valid;
    pdm_sampler #(.N_LINES(48)) u_pdm_sampler (
        .clk(clk), .rst(rst),
        .pdm_div(pdm_div), .data(DATA),
        .pdm_l(pdm_l), .pdm_r(pdm_r), .pdm_valid(pdm_valid)
    );

    wire pkt_toggle = 1'b0; // no packetizer/MAC yet -- see header

    status_led u_status_led (
        .clk(clk), .rst(rst),
        .pkt_toggle(pkt_toggle),
        .led_r(LED_R), .led_g(LED_G), .led_b(LED_B)
    );
endmodule
