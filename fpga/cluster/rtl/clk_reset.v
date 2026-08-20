`timescale 1ns / 1ps

// Buffers the received SPOKE_CLK (forwarded from the hub, now 6.144 MHz --
// see cluster_top.v's header comment on the shared-CIC resource-sharing
// architecture) into the cluster's single internal clock domain (still no
// local PLL/MMCM -- see PHASE4.md's "hub-and-spoke, single shared clock
// domain" architecture; this is a plain divide-by-2 toggle flip-flop, not a
// clock synthesizer), derives PDM_CLK = clk/2 (3.072 MHz, what the mics
// actually need) for the local mics, and generates the cluster's internal
// reset. Reset is held while either the local POR counter hasn't finished
// or the hub-driven FPGA_RESET_N (schematic net, active-low) is asserted --
// see hub.kicad_sch/cluster_0N.kicad_sch's FPGA_RESET_N/SPOKE_ALIVE nets and
// fpga/hub/rtl/reset_seq.v for the other end of this handshake. spoke_alive
// tells the hub "POR is done and I'm safely parked in reset" so it knows
// it's safe to release fpga_reset_n.
//
// pdm_phase is pdm_clk_r itself (pre-OBUF, combinational passthrough -- NOT
// an extra registered copy), broadcast to every pdm_line_sync/
// cic_decimator_shared instance. Each of those registers pdm_phase and
// pdm_d at the same edge, so both land one cycle behind their respective
// sources together -- if pdm_phase were instead pre-registered here, that
// alignment would be off by a cycle. See pdm_line_sync.v.
//
// pdm_clk_r free-runs from a fixed initial value, NOT gated by rst: gating
// it would make its phase (which edge -- rise or fall -- comes first after
// reset releases) depend on exactly how many clk cycles POR happened to
// hold for, an unpredictable parity nothing downstream should have to
// track. Harmless to leave ungated -- pdm_line_sync/cic_decimator_shared's
// own state is still properly held at 0 through rst regardless of what
// PDM_CLK/pdm_phase are doing -- and it matches the old design, where
// PDM_CLK was an un-gated passthrough of the single shared clock too.
module clk_reset #(
    parameter integer POR_CYCLES = 16
) (
    input  wire spoke_clk,
    input  wire fpga_reset_n,   // active-low, from hub via FPGA_RESET_N
    output wire clk,
    output wire pdm_clk,
    output wire pdm_phase,
    output reg  rst = 1'b1,
    output wire spoke_alive     // high once POR is done AND parked in
                                 // externally-held reset
);
    wire clk_ibuf;

    IBUF ibuf_inst (.I(spoke_clk), .O(clk_ibuf));
    BUFG bufg_inst (.I(clk_ibuf), .O(clk));

    reg pdm_clk_r = 1'b0;
    always @(posedge clk)
        pdm_clk_r <= ~pdm_clk_r;
    OBUF obuf_inst (.I(pdm_clk_r), .O(pdm_clk));
    assign pdm_phase = pdm_clk_r;

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
