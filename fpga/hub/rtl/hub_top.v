`timescale 1ns / 1ps

// Hub FPGA (Cmod A7-35T) top level. Currently implements only the power-up
// reset-sequencing handshake + RGB LED status (see reset_seq.v) -- the rest
// of the hub's responsibilities (spoke deframing, USB FIFO bridge -- see
// fpga/README.md) are future additions to this same top module.
//
// Port names match pcb/multi_fpga/hub.kicad_sch's global-label net names
// exactly (TCXO_CLK, FPGA_RESET_N, SPOKEn_ALIVE), same convention as
// fpga/cluster/rtl/cluster_top.v -- do not rename without updating the
// schematic. led0_r/g/b have no schematic net (Cmod A7's on-board RGB LED
// LD0 is wired directly to dedicated FPGA pins inside the module, not
// brought out through any connector), so they follow Digilent's own
// out-of-box-demo naming instead (see XilinxProjects/s7_hw/.../top.v).
module hub_top (
    input  wire TCXO_CLK,       // 12.288 MHz HCMOS TCXO (Y1)
    output wire FPGA_RESET_N,   // -> all 4 SpokeBus connectors' pin 11
    input  wire SPOKE0_ALIVE, SPOKE1_ALIVE, SPOKE2_ALIVE, SPOKE3_ALIVE,
    output wire led0_r, led0_g, led0_b // on-board RGB LED (LD0)
);
    wire clk_ibuf, clk;
    IBUF ibuf_inst (.I(TCXO_CLK), .O(clk_ibuf));
    BUFG bufg_inst (.I(clk_ibuf), .O(clk));

    reset_seq u_reset_seq (
        .clk(clk),
        .spoke_alive_i({SPOKE3_ALIVE, SPOKE2_ALIVE, SPOKE1_ALIVE, SPOKE0_ALIVE}),
        .spoke_reset_n(FPGA_RESET_N),
        .led_r_n(led0_r), .led_g_n(led0_g), .led_b_n(led0_b)
    );
endmodule
