`timescale 1ns / 1ps

// Bit-exact check of fir_compensator.v against fpga/multi_fpga/cluster/golden/fir_design.py's
// fir_bitexact(), driven at the real 64-cycle sample period (matching the
// CIC's decimation rate) so the 32-cycle MAC completion budget is exercised
// for real, not just claimed. Vectors from gen_vectors.py's
// gen_fir_test_vectors() / fir_design.py's gen_fir_vectors().
module tb_fir_compensator;
    localparam integer N            = 400;
    localparam integer DATA_WIDTH   = 24;
    localparam integer PERIOD_CYCLES = 64; // CIC decimation rate this engine is budgeted against

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg [DATA_WIDTH-1:0] mem_in  [0:N-1];
    reg [DATA_WIDTH-1:0] mem_exp [0:N-1];

    reg valid_in = 0;
    reg [DATA_WIDTH-1:0] data_in = 0;
    wire valid_out;
    wire [DATA_WIDTH-1:0] data_out;

    fir_compensator #(.COEFF_MEM_FILE("../vectors/fir_coeffs.mem")) dut (
        .clk(clk), .rst(rst),
        .valid_in(valid_in), .data_in(data_in),
        .valid_out(valid_out), .data_out(data_out)
    );

    integer errors = 0;
    integer idx;
    integer out_idx = 0;
    integer cycles_since_valid;

    initial begin
        $readmemh("../vectors/fir_test_input.mem", mem_in);
        $readmemh("../vectors/fir_test_expected.mem", mem_exp);
    end

    // cycle-budget counter + output capture/compare
    always @(posedge clk) begin
        if (rst) begin
            cycles_since_valid <= 0;
        end else begin
            if (valid_in)
                cycles_since_valid <= 1;
            else
                cycles_since_valid <= cycles_since_valid + 1;

            #1;
            if (valid_out) begin
                if (cycles_since_valid > PERIOD_CYCLES) begin
                    $display("FAIL: MAC completion took >%0d cycles (took %0d)",
                             PERIOD_CYCLES, cycles_since_valid);
                    errors = errors + 1;
                end
                if (out_idx < N) begin
                    if (data_out !== mem_exp[out_idx]) begin
                        $display("FAIL: fir out[%0d] got %0d expected %0d",
                                 out_idx, data_out, mem_exp[out_idx]);
                        errors = errors + 1;
                    end
                    out_idx = out_idx + 1;
                end
            end
        end
    end

    initial begin
        rst = 1;
        valid_in = 0;
        data_in = 0;
        repeat (4) @(negedge clk);
        rst = 0;

        for (idx = 0; idx < N; idx = idx + 1) begin
            @(negedge clk);
            valid_in = 1;
            data_in  = mem_in[idx];
            @(negedge clk);
            valid_in = 0;
            data_in  = 0;
            repeat (PERIOD_CYCLES - 2) @(negedge clk);
        end

        repeat (50) @(posedge clk); // let the final MAC pass finish

        if (out_idx !== N) begin
            $display("FAIL: only %0d/%0d outputs produced", out_idx, N);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("PASS: tb_fir_compensator, %0d outputs bit-exact, all within %0d-cycle budget",
                      N, PERIOD_CYCLES);
            $finish;
        end else begin
            $display("FAIL: tb_fir_compensator, %0d errors", errors);
            $finish;
        end
    end
endmodule
