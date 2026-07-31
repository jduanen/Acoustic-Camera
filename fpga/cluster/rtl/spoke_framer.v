`timescale 1ns / 1ps

// Frames 24 channels' 24-bit PCM samples onto the 6-bit DDR spoke bus, per
// the v1 protocol in fpga/cluster/SPOKE_FRAMING.md. On frame_start, latches
// all 24 channels (must be valid that same cycle) and begins transmitting:
// channel c's 24-bit sample as 4 MSB-first 6-bit nibbles over 2 clock
// periods (nibble on each of the rising and falling edge = DDR), channels in
// ascending order 0..23 (48 cycles busy), then 16 idle cycles (driven low)
// to fill out the 64-cycle CIC/FIR sample period.
//
// SPOKE_STROBE pulses for exactly the 1 cycle that carries channel 0's first
// nibble -- which is the cycle *after* frame_start's own pulse, not the same
// one. frame_start is itself a registered, one-cycle-wide pulse from another
// module (fir_compensator) sharing this same clock, so this module's own
// posedge-triggered logic cannot observe it until the following edge (a
// downstream flip-flop can never sample an upstream flip-flop's output on
// the very edge that output changes -- ordinary, correct synchronous
// behavior, not a simulation artifact). Everything here is therefore purely
// registered off frame_start -- no combinational "same-cycle" shortcut for
// cyc_eff/data -- since a combinational path reacting one cycle earlier than
// this module's own registered state just produces two overlapping "cycle
// 0"s (this was a real, once-shipped bug: see cluster_top.v's integration
// testbench, which is what caught it -- the standalone testbench here didn't,
// because it drives frame_start with generous margin before the sampling
// edge, sidestepping the same-edge relationship entirely).
module spoke_framer (
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_start,           // pulse; all 24 channels valid this same cycle
    input  wire [24*24-1:0] ch_data_flat,      // channel c = ch_data_flat[24*c +: 24]
    output wire [5:0]  spoke_d,
    output wire        spoke_strobe
);
    localparam integer N_CH         = 24;
    localparam integer DATA_WIDTH   = 24;
    localparam integer BUSY_CYCLES  = 48; // N_CH * (DATA_WIDTH/6) / 2
    localparam integer FRAME_CYCLES = 64; // CIC/FIR sample period

    reg [5:0] cyc_r;
    reg [24*24-1:0] latched_flat;

    // Set once the first real frame_start has ever fired, never cleared
    // again -- cyc_r only free-runs (and can wrap through 0) once armed, so
    // it can never produce a spurious cyc_r==0 by wrapping from whatever
    // value it happened to hold before the first real frame.
    reg armed;

    always @(posedge clk) begin
        if (rst) begin
            armed        <= 1'b0;
            cyc_r        <= 6'd0;
            latched_flat <= {(24*24){1'b0}};
        end else if (frame_start) begin
            armed        <= 1'b1;
            cyc_r        <= 6'd0;
            latched_flat <= ch_data_flat;
        end else if (armed) begin
            cyc_r <= cyc_r + 6'd1;
        end
    end

    wire [4:0] c_idx      = cyc_r[5:1];         // channel = cyc_r / 2
    wire       chunk_pair = cyc_r[0];            // 0: nibbles 0/1, 1: nibbles 2/3
    wire       busy       = armed && (cyc_r < BUSY_CYCLES);
    wire [23:0] cur_ch_data = latched_flat[24*c_idx +: 24];

    reg [5:0] d_rise, d_fall;
    always @(*) begin
        if (busy) begin
            if (!chunk_pair) begin
                d_rise = cur_ch_data[23:18];
                d_fall = cur_ch_data[17:12];
            end else begin
                d_rise = cur_ch_data[11:6];
                d_fall = cur_ch_data[5:0];
            end
        end else begin
            d_rise = 6'd0;
            d_fall = 6'd0;
        end
    end

    assign spoke_d      = clk ? d_rise : d_fall;
    assign spoke_strobe = armed && (cyc_r == 6'd0);
endmodule
