`timescale 1ns / 1ps

// Checks clk_reset.v's clock chain (IBUF/BUFG/OBUF + the PDM_CLK = clk/2
// divider, needs Xilinx's unisim simulation models + glbl -- see the
// Makefile's sim-clk target), its power-on-reset timing, and the
// fpga_reset_n/spoke_alive handshake with the hub (see
// fpga/hub/rtl/reset_seq.v). Reset-hold length only needs to be "about
// POR_CYCLES", not bit-exact against a golden model like the other modules
// -- so this checks a safe lower/upper bound rather than an exact edge
// count.
module tb_clk_reset;
    localparam integer POR_CYCLES = 16;

    reg spoke_clk    = 0;
    reg fpga_reset_n = 1'b0; // hub holds cluster in reset at power-up
    always #5 spoke_clk = ~spoke_clk;

    wire clk, pdm_clk, pdm_phase, rst, spoke_alive;

    clk_reset #(.POR_CYCLES(POR_CYCLES)) dut (
        .spoke_clk(spoke_clk),
        .fpga_reset_n(fpga_reset_n),
        .clk(clk),
        .pdm_clk(pdm_clk),
        .pdm_phase(pdm_phase),
        .rst(rst),
        .spoke_alive(spoke_alive)
    );

    // Independent model of the expected PDM_CLK = clk/2 divider (free-running
    // from a fixed phase, NOT gated by rst -- see clk_reset.v's header
    // comment on why), so the checks below aren't just re-checking the DUT's
    // own formula against itself.
    reg expected_pdm_clk = 1'b0;
    always @(posedge clk)
        expected_pdm_clk <= ~expected_pdm_clk;

    integer errors = 0;
    integer i;

    initial begin
        // let time-0 initial-value assignments across modules settle before
        // sampling anything -- cross-module ordering of `reg x = val;`
        // initializers at time 0 isn't guaranteed, so spoke_alive's
        // continuous assign (fed by two such regs) can read 'x' if sampled
        // in the same instant.
        #1;
        if (rst !== 1'b1) begin
            $display("FAIL: rst not high at startup (got %b)", rst);
            errors = errors + 1;
        end
        if (spoke_alive !== 1'b0) begin
            $display("FAIL: spoke_alive not low at startup (got %b)", spoke_alive);
            errors = errors + 1;
        end

        // clk must track spoke_clk (IBUF/BUFG passthrough); pdm_clk keeps
        // free-running through reset too (see clk_reset.v), checked against
        // the independent expected_pdm_clk model. Kept short (well under
        // POR_CYCLES) so it doesn't itself consume the POR window before the
        // checks below run.
        for (i = 0; i < 3; i = i + 1) begin
            @(posedge spoke_clk);
            #1;
            if (clk !== 1'b1) begin
                $display("FAIL: clk did not rise with spoke_clk (iter %0d)", i);
                errors = errors + 1;
            end
            if (pdm_clk !== expected_pdm_clk) begin
                $display("FAIL: pdm_clk (%b) != expected clk/2 divider (%b) during reset (iter %0d)",
                          pdm_clk, expected_pdm_clk, i);
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
        if (spoke_alive !== 1'b0) begin
            $display("FAIL: spoke_alive high before POR finished");
            errors = errors + 1;
        end

        // ... POR finishes, but fpga_reset_n is still held low (hub hasn't
        // seen all 4 clusters yet): rst must STAY asserted (now held by the
        // external pin, not POR) and spoke_alive must assert -- "POR done,
        // parked in reset, waiting on the hub".
        repeat (POR_CYCLES) @(posedge clk);
        #1;
        if (rst !== 1'b1) begin
            $display("FAIL: rst dropped while fpga_reset_n still held low");
            errors = errors + 1;
        end
        if (spoke_alive !== 1'b1) begin
            $display("FAIL: spoke_alive not asserted once POR done + still in external reset");
            errors = errors + 1;
        end

        // hub releases fpga_reset_n -- after synchronizer latency, rst must
        // drop and spoke_alive must drop too (it only means "alive AND
        // parked in reset", not a running heartbeat).
        fpga_reset_n = 1'b1;
        repeat (8) @(posedge clk); // 2-FF sync + margin
        #1;
        if (rst !== 1'b0) begin
            $display("FAIL: rst still asserted after fpga_reset_n released");
            errors = errors + 1;
        end
        if (spoke_alive !== 1'b0) begin
            $display("FAIL: spoke_alive still asserted after leaving reset");
            errors = errors + 1;
        end

        // and stay deasserted, with pdm_clk now toggling at clk/2 (checked
        // against the independent expected_pdm_clk model above, not the
        // DUT's own formula) and pdm_phase tracking pdm_clk (same signal,
        // pre-OBUF)
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            #1;
            if (rst !== 1'b0) begin
                $display("FAIL: rst reasserted spuriously post-POR (iter %0d)", i);
                errors = errors + 1;
            end
            if (spoke_alive !== 1'b0) begin
                $display("FAIL: spoke_alive reasserted spuriously while running (iter %0d)", i);
                errors = errors + 1;
            end
            if (pdm_clk !== expected_pdm_clk) begin
                $display("FAIL: pdm_clk (%b) != expected clk/2 divider (%b) post-POR (iter %0d)",
                          pdm_clk, expected_pdm_clk, i);
                errors = errors + 1;
            end
            if (pdm_phase !== pdm_clk) begin
                $display("FAIL: pdm_phase (%b) != pdm_clk (%b) post-POR (iter %0d)", pdm_phase, pdm_clk, i);
                errors = errors + 1;
            end
        end

        // fpga_reset_n dropping again later (e.g. a second hub-initiated
        // sequence) must reassert rst and spoke_alive must eventually track
        // it again -- since POR only ever runs once, spoke_alive should
        // reassert immediately (por_done stays latched high).
        fpga_reset_n = 1'b0;
        repeat (8) @(posedge clk);
        #1;
        if (rst !== 1'b1) begin
            $display("FAIL: rst did not reassert on a later fpga_reset_n low pulse");
            errors = errors + 1;
        end
        if (spoke_alive !== 1'b1) begin
            $display("FAIL: spoke_alive did not track a later fpga_reset_n low pulse (por_done should stay latched)");
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("PASS: tb_clk_reset, clk passthrough + PDM_CLK/2 divider + POR timing + fpga_reset_n/spoke_alive handshake verified");
            $finish;
        end else begin
            $display("FAIL: tb_clk_reset, %0d errors", errors);
            $finish;
        end
    end
endmodule
