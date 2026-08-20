`timescale 1ns / 1ps

module tb_pdm_line_sync;
    reg clk = 0;
    reg rst = 1;
    reg pdm_d = 0;
    reg pdm_phase = 0;
    wire bit_r, phase_r;

    pdm_line_sync dut (
        .clk(clk),
        .rst(rst),
        .pdm_phase(pdm_phase),
        .pdm_d(pdm_d),
        .bit_r(bit_r),
        .phase_r(phase_r)
    );

    always #5 clk = ~clk; // period/frequency irrelevant to this functional check

    integer errors = 0;
    integer i;
    integer n_checks = 0;

    initial begin
        // Hold reset across several edges while driving pdm_d/pdm_phase=1,
        // to prove reset actually overrides capture.
        rst = 1;
        pdm_d = 1;
        pdm_phase = 1;
        repeat (4) @(negedge clk);
        #1;
        if (bit_r !== 1'b0) begin
            $display("FAIL: bit_r not held at 0 during reset (got %b)", bit_r);
            errors = errors + 1;
        end
        if (phase_r !== 1'b0) begin
            $display("FAIL: phase_r not held at 0 during reset (got %b)", phase_r);
            errors = errors + 1;
        end

        rst = 0;
        @(negedge clk); // let the reset-deassertion edge settle

        // pdm_d/pdm_phase only change right after an edge and hold constant
        // until the next, so "current value" at any edge is exactly what
        // that edge should capture -- bit_r/phase_r must both equal the
        // *same* prior-cycle (pdm_d, pdm_phase) pair, one cycle later.
        for (i = 0; i < 1000; i = i + 1) begin
            @(posedge clk);
            #1;
            n_checks = n_checks + 1;
            if (bit_r !== pdm_d) begin
                $display("FAIL: bit_r mismatch at t=%0t: got %b expected %b", $time, bit_r, pdm_d);
                errors = errors + 1;
            end
            if (phase_r !== pdm_phase) begin
                $display("FAIL: phase_r mismatch at t=%0t: got %b expected %b", $time, phase_r, pdm_phase);
                errors = errors + 1;
            end
            pdm_d = $random;
            pdm_phase = $random;
        end

        if (errors == 0) begin
            $display("PASS: tb_pdm_line_sync, 0 mismatches over %0d edge checks", n_checks);
            $finish;
        end else begin
            $display("FAIL: tb_pdm_line_sync, %0d/%0d mismatches", errors, n_checks);
            $finish;
        end
    end
endmodule
