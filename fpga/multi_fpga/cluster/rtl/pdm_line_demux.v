`timescale 1ns / 1ps

// One physical PDM_Dxx line carries 2 IM72D128 mics: SEL=GND drives the line
// on the falling clock edge (L/even channel), SEL=+1.8V drives it on the
// rising edge (R/odd channel) -- see pcb/multi_fpga/SCHEMATIC_NOTES.md and
// the IM72D128 datasheet Sec 3.5. This module just captures both edges.
//
// Known characteristic (not a bug): cic_decimator (downstream, posedge-
// triggered) reads ch_r on the very same edge this module updates it on --
// per standard nonblocking-assignment semantics, a downstream posedge
// register can never observe an upstream posedge register's new value on
// that same edge, so cic_decimator's R-channel accumulation effectively runs
// one sample behind its L-channel accumulation (which doesn't have this
// issue, since ch_l has already been stable for half a cycle by the time
// cic_decimator's posedge reads it). This is ordinary flip-flop-to-flip-flop
// timing, not a simulation artifact -- real hardware behaves identically.
// See fpga/cluster/golden/gen_pdm_stimulus.py's golden_channel_pipeline()
// for how the golden model accounts for it.
module pdm_line_demux (
    input  wire clk,
    input  wire rst,
    input  wire pdm_d,
    output reg  ch_l,
    output reg  ch_r
);
    always @(negedge clk) begin
        if (rst)
            ch_l <= 1'b0;
        else
            ch_l <= pdm_d;
    end

    always @(posedge clk) begin
        if (rst)
            ch_r <= 1'b0;
        else
            ch_r <= pdm_d;
    end
endmodule
