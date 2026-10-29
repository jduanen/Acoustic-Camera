`timescale 1ns / 1ps

// Front-end board status LED (D1, common-anode RGB on +5V, each cathode
// switched by a 2N7002 -- see pcb/SCHEMATIC_NOTES.md "Status LED"). The
// outputs drive the FET gates, so they are ACTIVE HIGH (1 = colour on),
// unlike the multi_fpga boards' directly-driven active-low LEDs.
//
// Same YELLOW(resetting)/GREEN(running) convention as
// fpga/multi_fpga/cluster/rtl/clk_reset.v and hub/rtl/reset_seq.v, plus a
// streaming indicator:
//   rst high                          -> YELLOW (R+G)
//   running, no recent packet         -> GREEN
//   running, packet within HOLD_CYCLES -> GREEN/BLUE alternating every
//                                        2^BLINK_BIT clk cycles
// Packets go out at 9,600/s (GBE_FRAMING.md), far faster than the eye can follow, so each one
// just re-arms the HOLD_CYCLES window instead of producing its own flash;
// the LED reverts to solid GREEN once packets stop for that long. RED alone
// is left free for a future fault indication (nothing detects one yet).
//
// pkt_toggle flips once per packet in the MAC's tx_clk domain (async to
// clk), so it is passed through a 2-FF synchronizer and edge-detected here
// -- the same toggle-crossing gbe_packetizer.v uses for its bank handoff.
//
// Defaults are for clk = 12.288 MHz (TCXO): 2^21 cycles = 171 ms per blink
// phase, 2^22 cycles = 341 ms hold.
module status_led #(
    parameter integer BLINK_BIT   = 21,
    parameter integer HOLD_CYCLES = 1 << 22
) (
    input  wire clk,
    input  wire rst,
    input  wire pkt_toggle,
    output wire led_r,
    output wire led_g,
    output wire led_b
);
    (* ASYNC_REG = "TRUE" *) reg [2:0] pkt_sync = 3'b000;
    always @(posedge clk)
        pkt_sync <= {pkt_sync[1:0], pkt_toggle};
    wire pkt_seen = pkt_sync[2] ^ pkt_sync[1];

    reg [BLINK_BIT:0]                blink_cnt = 0;
    reg [$clog2(HOLD_CYCLES+1)-1:0] hold_cnt  = 0;

    always @(posedge clk) begin
        blink_cnt <= blink_cnt + 1'b1;
        if (rst)
            hold_cnt <= 0;
        else if (pkt_seen)
            hold_cnt <= HOLD_CYCLES;
        else if (hold_cnt != 0)
            hold_cnt <= hold_cnt - 1'b1;
    end

    wire streaming = (hold_cnt != 0);
    wire blue_ph   = streaming & blink_cnt[BLINK_BIT];

    assign led_r = rst;
    assign led_g = rst | ~blue_ph;
    assign led_b = ~rst & blue_ph;
endmodule
