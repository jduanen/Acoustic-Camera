`timescale 1ns / 1ps

// Power-up reset-sequencing handshake, hub side (master). Holds all 4
// clusters in reset (FPGA_RESET_N low) and shows YELLOW until SPOKES_ALIVE
// confirms every cluster is alive and parked in its own (externally-held)
// reset; then releases reset and shows GREEN. One-shot by design -- no path
// back to S_RESETTING (matches the described power-up behavior; a later
// re-arm/watchdog feature is not part of this).
//
// spoke_alive_i is a single wired-AND bus signal, not 4 independent bits:
// each cluster's clk_reset.v drives its own SPOKE_ALIVE pin tri-state (Hi-Z
// once ready, low otherwise) onto one shared net with one external pull-up
// on the hub PCB, so the net itself reads high only once all 4 release it --
// the AND-of-4 that used to happen here in RTL now happens for free in the
// wire. See clk_reset.v's header comment for the driver side.
module reset_seq #(
    parameter integer MIN_RESET_CYCLES = 16 // mirrors clk_reset.v's POR_CYCLES
) (
    input  wire clk,
    input  wire spoke_alive_i,
    output reg  spoke_reset_n = 1'b0, // active-low, drives shared FPGA_RESET_N
    output wire led_r_n,              // Cmod A7 on-board RGB LED (LD0),
    output wire led_g_n,              // common-anode -> active-low
    output wire led_b_n
);
    // spoke_alive_i comes from 4 different FPGAs via a shared external net --
    // 2-FF synchronizer, same pattern as clk_reset.v's fpga_reset_n
    // synchronizer.
    reg alive_meta = 1'b0, alive_sync = 1'b0;
    always @(posedge clk) begin
        alive_meta <= spoke_alive_i;
        alive_sync <= alive_meta;
    end
    wire all_alive = alive_sync;

    localparam S_RESETTING = 1'b0, S_RUNNING = 1'b1;
    reg state = S_RESETTING;
    reg [$clog2(MIN_RESET_CYCLES+1)-1:0] hold_cnt = 0;

    always @(posedge clk) begin
        case (state)
            S_RESETTING: begin
                spoke_reset_n <= 1'b0;
                if (hold_cnt < MIN_RESET_CYCLES)
                    hold_cnt <= hold_cnt + 1'b1;
                else if (all_alive)
                    state <= S_RUNNING;
            end
            S_RUNNING: spoke_reset_n <= 1'b1;
        endcase
    end

    // YELLOW (R+G) while resetting, GREEN (G only) once running, B unused.
    assign led_r_n = (state == S_RESETTING) ? 1'b0 : 1'b1;
    assign led_g_n = 1'b0;
    assign led_b_n = 1'b1;
endmodule
