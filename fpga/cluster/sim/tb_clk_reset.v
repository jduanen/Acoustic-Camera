`timescale 1ns / 1ps

// Checks clk_reset.v's clock passthrough (IBUF/BUFG/OBUF chain, needs
// Xilinx's unisim simulation models + glbl -- see the Makefile's sim-clk
// target) and its power-on-reset timing. Reset-hold length only needs to be
// "about POR_CYCLES", not bit-exact against a golden model like the other
// modules -- so this checks a safe lower/upper bound rather than an exact
// edge count.
module tb_clk_reset;
    localparam integer POR_CYCLES = 16;

    reg spoke_clk = 0;
    always #5 spoke_clk = ~spoke_clk;

    wire clk, pdm_clk, rst;

    clk_reset #(.POR_CYCLES(POR_CYCLES)) dut (
        .spoke_clk(spoke_clk),
        .clk(clk),
        .pdm_clk(pdm_clk),
        .rst(rst)
    );

    integer errors = 0;
    integer i;

    initial begin
        if (rst !== 1'b1) begin
            $display("FAIL: rst not high at startup (got %b)", rst);
            errors = errors + 1;
        end

        // clock passthrough on the first few edges: pdm_clk must track clk,
        // clk must track spoke_clk. Kept short (well under POR_CYCLES) so it
        // doesn't itself consume the POR window before the checks below run.
        for (i = 0; i < 3; i = i + 1) begin
            @(posedge spoke_clk);
            #1;
            if (clk !== 1'b1) begin
                $display("FAIL: clk did not rise with spoke_clk (iter %0d)", i);
                errors = errors + 1;
            end
            if (pdm_clk !== clk) begin
                $display("FAIL: pdm_clk (%b) != clk (%b) (iter %0d)", pdm_clk, clk, i);
                errors = errors + 1;
            end
        end

        // reset must still be held partway through the POR window
        repeat (POR_CYCLES / 2 - 3) @(posedge clk);
        #1;
        if (rst !== 1'b1) begin
            $display("FAIL: rst dropped before POR_CYCLES/2 edges elapsed");
            errors = errors + 1;
        end

        // ... and must have deasserted well after the POR window
        repeat (POR_CYCLES) @(posedge clk);
        #1;
        if (rst !== 1'b0) begin
            $display("FAIL: rst still asserted well after POR_CYCLES edges");
            errors = errors + 1;
        end

        // and stay deasserted, with clock passthrough still holding
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            #1;
            if (rst !== 1'b0) begin
                $display("FAIL: rst reasserted spuriously post-POR (iter %0d)", i);
                errors = errors + 1;
            end
            if (pdm_clk !== clk) begin
                $display("FAIL: pdm_clk (%b) != clk (%b) post-POR (iter %0d)", pdm_clk, clk, i);
                errors = errors + 1;
            end
        end

        if (errors == 0) begin
            $display("PASS: tb_clk_reset, clock passthrough + POR timing verified");
            $finish;
        end else begin
            $display("FAIL: tb_clk_reset, %0d errors", errors);
            $finish;
        end
    end
endmodule
