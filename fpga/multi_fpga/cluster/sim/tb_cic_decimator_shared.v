`timescale 1ns / 1ps

// Bit-exact check of cic_decimator_shared.v against the same
// fpga/multi_fpga/cluster/golden/cic_golden.py cic_bitexact() used by tb_cic_decimator.v
// -- the shared engine's math is unchanged (see cic_decimator_shared.v's
// header comment), so this drives two independent bit streams through the
// L/R-multiplexed interface (phase toggling every clk cycle) and checks each
// channel's output against its own single-channel golden reference
// independently. Vectors from gen_vectors.py's gen_cic_shared_vectors().
module tb_cic_decimator_shared;
    localparam integer N_IN  = 32000;
    localparam integer N_OUT = 500;
    localparam integer WIDTH = 1 + 5 * 6; // IN_WIDTH=1 + STAGES*ceil(log2(R)) = 31

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg  mem_l_in  [0:N_IN-1];
    reg  mem_r_in  [0:N_IN-1];
    reg  [WIDTH-1:0] mem_l_exp [0:N_OUT-1];
    reg  [WIDTH-1:0] mem_r_exp [0:N_OUT-1];

    reg [31:0] idx_l = 0, idx_r = 0;
    reg [31:0] out_idx_l = 0, out_idx_r = 0;

    // phase toggles every cycle: 0=L's turn, 1=R's turn -- mirrors
    // pdm_line_sync.v's phase_r feeding this module in cluster_top.v.
    reg phase = 1'b0;
    wire bit_in = phase ? (idx_r < N_IN ? mem_r_in[idx_r] : 1'b0)
                         : (idx_l < N_IN ? mem_l_in[idx_l] : 1'b0);

    wire [WIDTH-1:0] data_out_l, data_out_r;
    wire valid_l, valid_r;

    cic_decimator_shared #(.STAGES(5), .R(64), .IN_WIDTH(1)) dut (
        .clk(clk), .rst(rst), .bit_in(bit_in), .phase(phase),
        .data_out_l(data_out_l), .valid_l(valid_l),
        .data_out_r(data_out_r), .valid_r(valid_r)
    );

    integer errors = 0;

    initial begin
        $readmemb("../vectors/cic_shared_l_input.mem", mem_l_in);
        $readmemb("../vectors/cic_shared_r_input.mem", mem_r_in);
        $readmemh("../vectors/cic_shared_l_expected.mem", mem_l_exp);
        $readmemh("../vectors/cic_shared_r_expected.mem", mem_r_exp);
    end

    initial begin
        if (dut.WIDTH !== WIDTH) begin
            $display("FAIL: dut.WIDTH=%0d expected %0d", dut.WIDTH, WIDTH);
            errors = errors + 1;
        end
    end

    // phase toggle + per-channel input index advance (each channel's index
    // only moves on its own phase cycle, one bit per its own turn)
    always @(posedge clk) begin
        if (rst) begin
            phase <= 1'b0;
            idx_l <= 0;
            idx_r <= 0;
        end else begin
            phase <= ~phase;
            if (!phase && idx_l < N_IN) idx_l <= idx_l + 1;
            if ( phase && idx_r < N_IN) idx_r <= idx_r + 1;
        end
    end

    // output capture/compare, every cycle
    always @(posedge clk) begin
        if (!rst) begin
            #1;
            if (valid_l) begin
                if (out_idx_l < N_OUT) begin
                    if (data_out_l !== mem_l_exp[out_idx_l]) begin
                        $display("FAIL: L[%0d] got %0d expected %0d",
                                 out_idx_l, data_out_l, mem_l_exp[out_idx_l]);
                        errors = errors + 1;
                    end
                    out_idx_l <= out_idx_l + 1;
                end
            end
            if (valid_r) begin
                if (out_idx_r < N_OUT) begin
                    if (data_out_r !== mem_r_exp[out_idx_r]) begin
                        $display("FAIL: R[%0d] got %0d expected %0d",
                                 out_idx_r, data_out_r, mem_r_exp[out_idx_r]);
                        errors = errors + 1;
                    end
                    out_idx_r <= out_idx_r + 1;
                end
            end
            // valid_l and valid_r must never both be high the same cycle --
            // only one channel can complete its window on any given clk
            // edge (they're phase-interleaved, not truly simultaneous).
            if (valid_l && valid_r) begin
                $display("FAIL: valid_l and valid_r both high same cycle at t=%0t", $time);
                errors = errors + 1;
            end
        end
    end

    initial begin
        rst = 1;
        repeat (4) @(negedge clk);
        rst = 0;

        // 2 clk cycles per input bit (L+R interleaved) for each of N_IN bits,
        // plus margin.
        repeat (2 * N_IN + 100) @(posedge clk);
        #1;

        if (out_idx_l !== N_OUT) begin
            $display("FAIL: L channel produced %0d/%0d expected outputs", out_idx_l, N_OUT);
            errors = errors + 1;
        end
        if (out_idx_r !== N_OUT) begin
            $display("FAIL: R channel produced %0d/%0d expected outputs", out_idx_r, N_OUT);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("PASS: tb_cic_decimator_shared, L(%0d) + R(%0d) outputs all bit-exact, time-multiplexed",
                      N_OUT, N_OUT);
            $finish;
        end else begin
            $display("FAIL: tb_cic_decimator_shared, %0d errors", errors);
            $finish;
        end
    end
endmodule
