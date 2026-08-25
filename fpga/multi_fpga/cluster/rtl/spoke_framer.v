`timescale 1ns / 1ps

// Frames N_CH channels' 24-bit PCM samples onto the 6-bit DDR spoke bus, per
// the v1 protocol in fpga/cluster/SPOKE_FRAMING.md (N_CH=24 there; a bigger
// N_CH is a Mark II exploration, see the project's plan file). On
// frame_start, latches all N_CH channels (must be valid that same cycle) and
// begins transmitting: channel c's 24-bit sample as 4 MSB-first 6-bit
// nibbles over 2 clock periods (nibble on each of the rising and falling
// edge = DDR), channels in ascending order 0..N_CH-1 (BUSY_CYCLES =
// N_CH*2 cycles busy, 48 at N_CH=24), then idle (driven low) until the next
// frame_start.
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
module spoke_framer #(
    parameter integer N_CH = 24
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        frame_start,           // pulse; all N_CH channels valid this same cycle
    input  wire [N_CH*24-1:0] ch_data_flat,    // channel c = ch_data_flat[24*c +: 24]
    output wire [5:0]  spoke_d,
    output wire        spoke_strobe
);
    localparam integer DATA_WIDTH   = 24;
    localparam integer BUSY_CYCLES  = N_CH * (DATA_WIDTH/6) / 2;
    localparam integer FRAME_CYCLES = 64; // CIC/FIR sample period
    // cyc_r must count up to (and hold at) BUSY_CYCLES -- $clog2(BUSY_CYCLES+1)
    // sizes it correctly for whatever N_CH this instance was built with (was a
    // hardcoded 6'd.../reg [5:0], which silently wrapped instead of holding
    // once N_CH grew enough to push BUSY_CYCLES past 63 -- see cyc_r below).
    localparam integer CYC_W        = $clog2(BUSY_CYCLES + 1);

    reg [CYC_W-1:0] cyc_r;
    reg [N_CH*24-1:0] latched_flat;

    // Set once the first real frame_start has ever fired, never cleared
    // again -- cyc_r only free-runs (and can wrap through 0) once armed, so
    // it can never produce a spurious cyc_r==0 by wrapping from whatever
    // value it happened to hold before the first real frame.
    reg armed;

    always @(posedge clk) begin
        if (rst) begin
            armed        <= 1'b0;
            cyc_r        <= {CYC_W{1'b0}};
            latched_flat <= {(N_CH*24){1'b0}};
        end else if (frame_start) begin
            armed        <= 1'b1;
            cyc_r        <= {CYC_W{1'b0}};
            latched_flat <= ch_data_flat;
        end else if (armed && cyc_r < BUSY_CYCLES) begin
            cyc_r <= cyc_r + 1'b1;
            // else: hold at BUSY_CYCLES (idle) until the next frame_start
            // resets it -- NOT a free-running counter, deliberately: this
            // module doesn't assume any particular caller period beyond
            // "frame_start arrives at least BUSY_CYCLES apart" (previously
            // it free-ran and relied on frame_start always beating cyc_r's
            // own 6-bit wraparound back to 0, i.e. arriving within 64
            // cycles -- true for the original fully-parallel cluster_top.v,
            // but not for the current shared-CIC one, whose ~128-cycle gap
            // between frame_start pulses let cyc_r wrap and refire
            // spoke_strobe on stale latched_flat mid-frame, corrupting
            // every frame after the first).
        end
    end

    wire [CYC_W-2:0] c_idx = cyc_r[CYC_W-1:1];   // channel = cyc_r / 2
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
    assign spoke_strobe = armed && (cyc_r == {CYC_W{1'b0}});
endmodule
