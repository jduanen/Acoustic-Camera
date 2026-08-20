`timescale 1ns / 1ps

// 2-channel time-multiplexed CIC decimator: one integrator/comb *arithmetic*
// chain shared between the L and R channels of a single PDM_Dxx line,
// selected each clk cycle by `phase` (from pdm_line_sync.v: 0=L, 1=R) --
// each channel keeps its own independent state (integ/comb_prev/samp_cnt,
// indexed [0]=L/[1]=R) so the two channels' decimation math is completely
// independent, only the adder/subtractor *logic* is reused between them.
// This is what makes 12 of these (one per physical PDM line) equivalent to
// the 24 fully-parallel cic_decimator.v instances cluster_top.v used to
// instantiate, at half the LUT cost -- see cluster_top.v's header comment.
//
// Both channels still see exactly one new input bit every R samples of
// their own, at the same 3.072 MHz-equivalent rate as before (an L bit
// arrives every other clk cycle, on phase=0 cycles; R every other cycle, on
// phase=1) -- the underlying math is unchanged from cic_decimator.v/
// cic_golden.py's cic_bitexact(), only the hardware scheduling differs, so
// that golden model remains the correct bit-exact reference for each
// channel's output stream independently (see tb_cic_decimator_shared.v).
module cic_decimator_shared #(
    parameter integer STAGES   = 5,
    parameter integer R        = 64,
    parameter integer IN_WIDTH = 1,
    parameter integer WIDTH    = IN_WIDTH + STAGES * $clog2(R)
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 bit_in,   // this cycle's captured PDM bit
    input  wire                 phase,    // 0 = bit_in is L's, 1 = R's
    output reg  [WIDTH-1:0]     data_out_l,
    output reg                  valid_l,
    output reg  [WIDTH-1:0]     data_out_r,
    output reg                  valid_r
);
    // Stage-0 integrator register is narrowed to STAGE0_WIDTH (19 bits, not
    // the full 31-bit Hogenauer bound every other stage needs) -- the only
    // stage that has any lossless slack. This was NOT derived from a
    // formula (a hand-derived per-stage growth-bound guess was tried first
    // and failed catastrophically when checked); it's the result of an
    // exhaustive empirical search (binary-searching each of the 10 stages'
    // widths independently against ~5.1M samples of random/adversarial/
    // worst-case input patterns, requiring bit-exact match to the
    // infinite-precision ideal reference at every width) that found stage 0
    // is the *only* one of the 10 stages (5 integrators + 5 combs) with any
    // margin at all -- see the CIC-sharing/LUT-budget conversation this
    // module was built in for the search itself. STAGE0_WIDTH=19 is
    // specific to STAGES=5/R=64/IN_WIDTH=1 (this module's only real
    // instantiation, in cluster_top.v) -- the assertion below catches
    // silent misuse if those parameters ever change without re-deriving it.
    localparam integer STAGE0_WIDTH = 19;
    initial if (!(STAGES == 5 && R == 64 && IN_WIDTH == 1)) begin
        $display("FATAL: cic_decimator_shared's STAGE0_WIDTH=19 optimization is only verified for STAGES=5,R=64,IN_WIDTH=1 -- re-run the empirical width search before using other parameters");
        $finish;
    end

    reg [STAGE0_WIDTH-1:0] integ0     [0:1];
    reg [WIDTH-1:0]        integ      [0:1][1:STAGES-1];
    reg [WIDTH-1:0]        comb_prev  [0:1][0:STAGES-1];
    reg [31:0]             samp_cnt   [0:1];

    integer s;
    reg [WIDTH-1:0] y;
    reg [WIDTH-1:0] prev;

    always @(posedge clk) begin
        if (rst) begin
            integ0[0] <= {STAGE0_WIDTH{1'b0}};
            integ0[1] <= {STAGE0_WIDTH{1'b0}};
            for (s = 1; s < STAGES; s = s + 1) begin
                integ[0][s]     <= {WIDTH{1'b0}};
                integ[1][s]     <= {WIDTH{1'b0}};
            end
            for (s = 0; s < STAGES; s = s + 1) begin
                comb_prev[0][s] <= {WIDTH{1'b0}};
                comb_prev[1][s] <= {WIDTH{1'b0}};
            end
            samp_cnt[0] <= 32'd0;
            samp_cnt[1] <= 32'd0;
            valid_l    <= 1'b0;
            valid_r    <= 1'b0;
            data_out_l <= {WIDTH{1'b0}};
            data_out_r <= {WIDTH{1'b0}};
        end else begin
            // Default-clear both every cycle; only the channel selected by
            // `phase` can override its own flag back to 1 below, and only
            // on the one cycle its own window completes -- keeps each pulse
            // exactly 1 clk cycle wide regardless of which channel's turn
            // it is (see fir_compensator.v: a 2-cycle-wide valid_in would
            // look like two separate samples arriving).
            valid_l <= 1'b0;
            valid_r <= 1'b0;

            // Integrator chain for whichever channel owns this cycle's bit
            // -- blocking assigns so stage s+1 sees this same cycle's
            // stage-s update, same as cic_decimator.v/cic_golden.py.
            // integ0 (narrower) zero-extends to WIDTH automatically when
            // added into integ[phase][1], exactly matching the verified
            // golden-model semantics (mask-to-STAGE0_WIDTH then feed the
            // masked value into a full-width accumulator).
            integ0[phase] = integ0[phase] + bit_in;
            integ[phase][1] = integ[phase][1] + integ0[phase];
            for (s = 2; s < STAGES; s = s + 1)
                integ[phase][s] = integ[phase][s] + integ[phase][s-1];

            if (samp_cnt[phase] == R - 1) begin
                samp_cnt[phase] <= 32'd0;

                y = integ[phase][STAGES-1];
                for (s = 0; s < STAGES; s = s + 1) begin
                    prev             = comb_prev[phase][s];
                    comb_prev[phase][s] = y;
                    y                = y - prev;
                end

                if (phase == 1'b0) begin
                    data_out_l <= y;
                    valid_l    <= 1'b1;
                end else begin
                    data_out_r <= y;
                    valid_r    <= 1'b1;
                end
            end else begin
                samp_cnt[phase] <= samp_cnt[phase] + 32'd1;
            end
        end
    end
endmodule
