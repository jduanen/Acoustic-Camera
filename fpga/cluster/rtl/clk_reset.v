`timescale 1ns / 1ps

// Buffers the received SPOKE_CLK (forwarded from the hub, 3.072 MHz) into
// the cluster's single internal clock domain (no local PLL -- see
// PHASE4.md's "hub-and-spoke, single shared clock domain" architecture),
// echoes it back out as PDM_CLK for the local mics, and generates the
// cluster's internal reset. Reset is held while either the local POR
// counter hasn't finished or the hub-driven FPGA_RESET_N (schematic net,
// active-low) is asserted -- see hub.kicad_sch/cluster_0N.kicad_sch's
// FPGA_RESET_N/SPOKE_ALIVE nets and fpga/hub/rtl/reset_seq.v for the other
// end of this handshake. spoke_alive tells the hub "POR is done and I'm
// safely parked in reset" so it knows it's safe to release fpga_reset_n.
module clk_reset #(
    parameter integer POR_CYCLES = 16
) (
    input  wire spoke_clk,
    input  wire fpga_reset_n,   // active-low, from hub via FPGA_RESET_N
    output wire clk,
    output wire pdm_clk,
    output reg  rst = 1'b1,
    output wire spoke_alive     // high once POR is done AND parked in
                                 // externally-held reset
);
    wire clk_ibuf;

    IBUF ibuf_inst (.I(spoke_clk), .O(clk_ibuf));
    BUFG bufg_inst (.I(clk_ibuf), .O(clk));
    OBUF obuf_inst (.I(clk), .O(pdm_clk));

    // 2-FF synchronizer: fpga_reset_n is driven by a different FPGA (the
    // hub) -- asynchronous to this device's clk despite ultimately sharing
    // the same TCXO, since the round trip through both devices' own
    // IBUF/BUFG/OBUF pipelines gives it arbitrary skew.
    reg fpga_rst_n_meta = 1'b0, fpga_rst_n_sync = 1'b0;
    always @(posedge clk) begin
        fpga_rst_n_meta <= fpga_reset_n;
        fpga_rst_n_sync <= fpga_rst_n_meta;
    end

    reg [$clog2(POR_CYCLES+1)-1:0] por_cnt  = 0;
    reg                            por_done = 1'b0;

    always @(posedge clk) begin
        if (por_cnt < POR_CYCLES) begin
            por_cnt <= por_cnt + 1'b1;
        end else begin
            por_done <= 1'b1;
        end
        rst <= (por_cnt < POR_CYCLES) | ~fpga_rst_n_sync;
    end

    assign spoke_alive = por_done & ~fpga_rst_n_sync;
endmodule
