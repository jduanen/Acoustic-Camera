`timescale 1ns / 1ps

// Buffers the received SPOKE_CLK (forwarded from the hub, 3.072 MHz) into
// the cluster's single internal clock domain (no local PLL -- see
// PHASE4.md's "hub-and-spoke, single shared clock domain" architecture),
// echoes it back out as PDM_CLK for the local mics, and generates an
// internal power-on reset. The schematic's exact port list
// (pcb/make_schematic_multi_fpga.py) has no reset pin at all, so this is the
// only reset source in the design.
module clk_reset #(
    parameter integer POR_CYCLES = 16
) (
    input  wire spoke_clk,
    output wire clk,
    output wire pdm_clk,
    output reg  rst = 1'b1
);
    wire clk_ibuf;

    IBUF ibuf_inst (.I(spoke_clk), .O(clk_ibuf));
    BUFG bufg_inst (.I(clk_ibuf), .O(clk));
    OBUF obuf_inst (.I(clk), .O(pdm_clk));

    reg [$clog2(POR_CYCLES+1)-1:0] por_cnt = 0;

    always @(posedge clk) begin
        if (por_cnt < POR_CYCLES) begin
            por_cnt <= por_cnt + 1'b1;
            rst     <= 1'b1;
        end else begin
            rst <= 1'b0;
        end
    end
endmodule
