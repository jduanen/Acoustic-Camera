`timescale 1ns / 1ps

// Dedicated per-channel FIR compensation MAC engine. One instance per audio
// channel (24 total in cluster_top.v) -- see fpga/cluster/golden/fir_design.py
// for the fixed-point convention (24-bit unsigned data, 18-bit signed Q1.17
// coefficients, 48-bit accumulator matching a DSP48E1's native width) and
// fir_bitexact() for the bit-exact reference this mirrors.
//
// Completes its NTAPS-tap MAC sequence in NTAPS cycles, starting the cycle
// after valid_in -- with NTAPS=32 and one valid_in pulse every 64 cycles
// (the CIC's decimation rate), that's 32 of 64 cycles used, 2x margin, no
// PLL/second clock domain needed (see PHASE4.md's clock architecture).
module fir_compensator #(
    parameter integer NTAPS       = 32,
    parameter integer DATA_WIDTH  = 24,
    parameter integer COEFF_WIDTH = 18,
    parameter integer ACC_WIDTH   = 48,
    parameter integer FRAC_BITS   = COEFF_WIDTH - 1,
    parameter         COEFF_MEM_FILE = ""
) (
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   valid_in,
    input  wire [DATA_WIDTH-1:0]  data_in,
    output reg                    valid_out,
    output reg  [DATA_WIDTH-1:0]  data_out
);
    reg signed [COEFF_WIDTH-1:0] coeff_rom [0:NTAPS-1];
    initial begin
        if (COEFF_MEM_FILE != "")
            $readmemh(COEFF_MEM_FILE, coeff_rom);
    end

    reg [DATA_WIDTH-1:0] shift_reg [0:NTAPS-1];
    reg signed [ACC_WIDTH-1:0] acc;
    reg [5:0] tap_idx;
    reg       running;

    integer i;

    wire signed [ACC_WIDTH-1:0] product = $signed({1'b0, shift_reg[tap_idx]}) * coeff_rom[tap_idx];
    wire signed [ACC_WIDTH-1:0] next_acc = acc + product;

    always @(posedge clk) begin
        if (rst) begin
            valid_out <= 1'b0;
            data_out  <= {DATA_WIDTH{1'b0}};
            running   <= 1'b0;
            tap_idx   <= 6'd0;
            acc       <= {ACC_WIDTH{1'b0}};
            for (i = 0; i < NTAPS; i = i + 1)
                shift_reg[i] <= {DATA_WIDTH{1'b0}};
        end else begin
            valid_out <= 1'b0; // default; pulses exactly 1 cycle on MAC completion

            if (valid_in) begin
                // New sample: shift it in (newest at index 0, matching
                // fir_bitexact()'s "shift_reg = [x] + shift_reg[:-1]") and
                // (re)start the MAC pass. If a previous pass hasn't finished
                // yet (shouldn't happen given the 32-of-64-cycle budget),
                // this restarts it -- acceptable since it can't occur in
                // correctly-timed use, not worth extra logic to guard against.
                shift_reg[0] <= data_in;
                for (i = 1; i < NTAPS; i = i + 1)
                    shift_reg[i] <= shift_reg[i-1];
                tap_idx <= 6'd0;
                acc     <= {ACC_WIDTH{1'b0}};
                running <= 1'b1;
            end else if (running) begin
                acc <= next_acc;
                if (tap_idx == NTAPS-1) begin
                    running   <= 1'b0;
                    valid_out <= 1'b1;
                    data_out  <= next_acc >>> FRAC_BITS;
                end else begin
                    tap_idx <= tap_idx + 6'd1;
                end
            end
        end
    end
endmodule
