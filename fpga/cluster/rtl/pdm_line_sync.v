`timescale 1ns / 1ps

// One physical PDM_Dxx line carries 2 IM72D128 mics: SEL=GND drives the line
// on PDM_CLK's falling edge (L/even channel, stable through the following
// low phase), SEL=+1.8V drives it on the rising edge (R/odd channel, stable
// through the following high phase) -- see pcb/multi_fpga/SCHEMATIC_NOTES.md
// and the IM72D128 datasheet Sec 3.5.
//
// Replaces pdm_line_demux.v now that clk runs at 2x PDM_CLK's rate (see
// clk_reset.v/cluster_top.v's header comments on the shared-CIC resource-
// sharing architecture): instead of capturing both edges into two held
// registers, this just registers pdm_d and clk_reset's pdm_phase together,
// once per (fast) clk cycle, and lets cic_decimator_shared use phase_r to
// route bit_r to the right channel's state. pdm_phase must be the
// combinational (pre-OBUF) pdm_clk_r, not a registered copy of it -- see
// clk_reset.v's header comment for why that alignment matters.
//
// phase_r convention: 0 = bit_r belongs to the L (falling-edge) channel,
// 1 = R (rising-edge) channel -- matches pdm_phase's own PDM_CLK-level
// meaning directly, no inversion.
module pdm_line_sync (
    input  wire clk,
    input  wire rst,
    input  wire pdm_phase,
    input  wire pdm_d,
    output reg  bit_r,
    output reg  phase_r
);
    always @(posedge clk) begin
        if (rst) begin
            bit_r   <= 1'b0;
            phase_r <= 1'b0;
        end else begin
            bit_r   <= pdm_d;
            phase_r <= pdm_phase;
        end
    end
endmodule
