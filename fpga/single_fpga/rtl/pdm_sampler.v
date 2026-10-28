`timescale 1ns / 1ps

// Samples the 48 mic DATA lines in the TCXO_CLK domain (12.288 MHz = 4x
// PDM_CLK) and splits each into its L and R channel bits, one pair per PDM
// period.
//
// Each line carries two IM72D128s (pcb/SCHEMATIC_NOTES.md "Data Line
// Assignment"; IM72D128 datasheet Table 8): L (SEL=GND) drives after PDM_CLK
// falls and goes high-Z after it rises; R (SEL=VDD) the reverse. The mic
// drives 40-80 ns (tDD) after its clock edge and lets go 5-30 ns (tHZ) after
// the opposite one, so a channel's bit is stable at the FPGA from ~89 ns
// after the PDM_CLK edge that requested it until ~8 ns after the next one
// (budget in xdc/single_fpga_top.xdc).
//
// The one TCXO edge inside that window is two cycles after the requesting
// PDM_CLK edge -- the same edge that launches the next PDM_CLK edge:
//   pdm_div 1->2 (PDM_CLK rises):  captures L, requested by the last fall
//   pdm_div 3->0 (PDM_CLK falls):  captures R, requested by the last rise
// pdm_div is single_fpga_top's free-running divider (PDM_CLK = pdm_div[1]).
// Every DATA bit is registered every cycle in its IOB, alongside the pdm_div
// value it was captured against, and only the two edges above are kept. The
// XDC's set_multicycle_path tells STA the same thing.
//
// Outputs update together once per PDM period: pdm_valid pulses for one clk
// cycle when pdm_l/pdm_r hold that period's L bit (captured two cycles
// earlier) and R bit.
module pdm_sampler #(
    parameter integer N_LINES = 48
) (
    input  wire               clk,
    input  wire               rst,
    input  wire [1:0]         pdm_div,
    input  wire [N_LINES-1:0] data,
    output reg  [N_LINES-1:0] pdm_l = 0,
    output reg  [N_LINES-1:0] pdm_r = 0,
    output reg                pdm_valid = 1'b0
);
    (* IOB = "TRUE" *) reg [N_LINES-1:0] data_iob;
    reg [1:0]         div_r = 2'd0;
    reg [N_LINES-1:0] l_hold = 0;

    always @(posedge clk) begin
        data_iob <= data;
        div_r    <= pdm_div;           // the value data_iob was captured against
    end

    always @(posedge clk) begin
        pdm_valid <= 1'b0;
        if (rst) begin
            l_hold <= 0;
            pdm_l  <= 0;
            pdm_r  <= 0;
        end else if (div_r == 2'd1) begin
            l_hold <= data_iob;
        end else if (div_r == 2'd3) begin
            pdm_l     <= l_hold;
            pdm_r     <= data_iob;
            pdm_valid <= 1'b1;
        end
    end
endmodule
