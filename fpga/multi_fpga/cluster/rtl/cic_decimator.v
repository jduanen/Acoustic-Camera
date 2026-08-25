`timescale 1ns / 1ps

// N-stage CIC decimator (differential delay M=1), decimating an IN_WIDTH-bit
// input stream by R. Mirrors fpga/multi_fpga/cluster/golden/cic_golden.py's
// cic_bitexact() exactly -- see that file's docstring and Hogenauer (1981)
// for the register-growth bound WIDTH implements.
module cic_decimator #(
    parameter integer STAGES   = 5,
    parameter integer R        = 64,
    parameter integer IN_WIDTH = 1,
    parameter integer WIDTH    = IN_WIDTH + STAGES * $clog2(R)
) (
    input  wire                 clk,
    input  wire                 rst,
    input  wire [IN_WIDTH-1:0]  data_in,
    output reg  [WIDTH-1:0]     data_out,
    output reg                  valid
);
    reg [WIDTH-1:0] integ     [0:STAGES-1];
    reg [WIDTH-1:0] comb_prev [0:STAGES-1];
    reg [31:0]      samp_cnt;

    integer s;
    reg [WIDTH-1:0] y;
    reg [WIDTH-1:0] prev;

    always @(posedge clk) begin
        if (rst) begin
            for (s = 0; s < STAGES; s = s + 1) begin
                integ[s]     <= {WIDTH{1'b0}};
                comb_prev[s] <= {WIDTH{1'b0}};
            end
            samp_cnt <= 32'd0;
            valid    <= 1'b0;
            data_out <= {WIDTH{1'b0}};
        end else begin
            // Integrator chain: every input sample, mirrors cic_golden.py's
            // per-bit loop exactly (blocking assigns so stage s+1 sees this
            // same cycle's stage-s update, same as the Python model's
            // sequential "x = integ[s]" chaining).
            integ[0] = integ[0] + data_in;
            for (s = 1; s < STAGES; s = s + 1)
                integ[s] = integ[s] + integ[s-1];

            if (samp_cnt == R - 1) begin
                samp_cnt <= 32'd0;

                // Comb chain, decimated rate: combinationally chained within
                // this same cycle against this cycle's integrator output,
                // matching cic_golden.py's "y = x; for s: y = y - comb_prev[s]".
                // NOTE (deferred, sim-only phase): a real synthesis pass will
                // likely need to pipeline this across extra cycles for timing
                // closure -- correctness over timing here.
                y = integ[STAGES-1];
                for (s = 0; s < STAGES; s = s + 1) begin
                    prev         = comb_prev[s];
                    comb_prev[s] = y;
                    y            = y - prev;
                end
                data_out <= y;
                valid    <= 1'b1;
            end else begin
                samp_cnt <= samp_cnt + 32'd1;
                valid    <= 1'b0;
            end
        end
    end
endmodule
