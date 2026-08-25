`timescale 1ns / 1ps

module tb_pdm_line_demux;
    reg clk = 0;
    reg rst = 1;
    reg pdm_d = 0;
    wire ch_l, ch_r;

    pdm_line_demux dut (
        .clk(clk),
        .rst(rst),
        .pdm_d(pdm_d),
        .ch_l(ch_l),
        .ch_r(ch_r)
    );

    always #5 clk = ~clk; // period/frequency irrelevant to this functional check

    integer errors = 0;
    integer i;
    integer n_checks = 0;

    initial begin
        // Hold reset across several edges while driving pdm_d=1, to prove
        // reset actually overrides capture rather than coincidentally
        // matching a 0 input.
        rst = 1;
        pdm_d = 1;
        repeat (4) @(negedge clk);
        #1;
        if (ch_l !== 1'b0) begin
            $display("FAIL: ch_l not held at 0 during reset (got %b)", ch_l);
            errors = errors + 1;
        end
        if (ch_r !== 1'b0) begin
            $display("FAIL: ch_r not held at 0 during reset (got %b)", ch_r);
            errors = errors + 1;
        end

        rst = 0;
        pdm_d = 0;
        @(negedge clk); // let the reset-deassertion edge settle

        // pdm_d is only ever changed right after an edge and held constant
        // until the next edge, so "current value of pdm_d" at any edge is
        // exactly the value that edge should capture.
        for (i = 0; i < 500; i = i + 1) begin
            @(posedge clk);
            #1;
            n_checks = n_checks + 1;
            if (ch_r !== pdm_d) begin
                $display("FAIL: ch_r mismatch at t=%0t: got %b expected %b", $time, ch_r, pdm_d);
                errors = errors + 1;
            end
            pdm_d = $random;

            @(negedge clk);
            #1;
            n_checks = n_checks + 1;
            if (ch_l !== pdm_d) begin
                $display("FAIL: ch_l mismatch at t=%0t: got %b expected %b", $time, ch_l, pdm_d);
                errors = errors + 1;
            end
            pdm_d = $random;
        end

        if (errors == 0) begin
            $display("PASS: tb_pdm_line_demux, 0 mismatches over %0d edge checks", n_checks);
            $finish;
        end else begin
            $display("FAIL: tb_pdm_line_demux, %0d/%0d mismatches", errors, n_checks);
            $finish;
        end
    end
endmodule
