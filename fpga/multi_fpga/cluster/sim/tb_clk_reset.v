`timescale 1ns / 1ps

// Checks clk_reset.v's clock chain (IBUF/BUFG/OBUF + the PDM_CLK = clk/2
// divider, needs Xilinx's unisim simulation models + glbl -- see the
// Makefile's sim-clk target), its power-on-reset timing, the
// fpga_reset_n/spoke_alive handshake with the hub (see
// fpga/hub/rtl/reset_seq.v -- spoke_alive is checked as a tri-state signal:
// 1'bz when "ready", not 1'b1, since it's now a wired-AND drive; an
// external pull-up, not modeled here, is what resolves that to a real
// logic 1 on the shared bus), and the led_r_n/led_g_n/led_b_n health
// indicator (YELLOW while rst is asserted, GREEN once running -- checked
// alongside every rst assertion via check_led()). Reset-hold length only
// needs to be "about POR_CYCLES", not bit-exact against a golden model like
// the other modules -- so this checks a safe lower/upper bound rather than
// an exact edge count.
module tb_clk_reset;
    localparam integer POR_CYCLES = 16;

    reg spoke_clk    = 0;
    reg fpga_reset_n = 1'b0; // hub holds cluster in reset at power-up
    always #5 spoke_clk = ~spoke_clk;

    wire clk, pdm_clk, pdm_phase, rst, spoke_alive;
    wire led_r_n, led_g_n, led_b_n;

    clk_reset #(.POR_CYCLES(POR_CYCLES)) dut (
        .spoke_clk(spoke_clk),
        .fpga_reset_n(fpga_reset_n),
        .clk(clk),
        .pdm_clk(pdm_clk),
        .pdm_phase(pdm_phase),
        .rst(rst),
        .spoke_alive(spoke_alive),
        .led_r_n(led_r_n), .led_g_n(led_g_n), .led_b_n(led_b_n)
    );

    integer errors = 0;
    integer i;

    // led_r_n/led_g_n/led_b_n track rst combinationally (YELLOW while
    // resetting, GREEN once running) -- checked alongside every rst
    // assertion below rather than as a separate pass.
    task check_led(input [255:0] label);
        begin
            if (led_r_n !== (rst ? 1'b0 : 1'b1)) begin
                $display("FAIL: %0s: led_r_n (%b) != expected (rst=%b)", label, led_r_n, rst);
                errors = errors + 1;
            end
            if (led_g_n !== 1'b0) begin
                $display("FAIL: %0s: led_g_n not held low", label);
                errors = errors + 1;
            end
            if (led_b_n !== 1'b1) begin
                $display("FAIL: %0s: led_b_n not held high", label);
                errors = errors + 1;
            end
        end
    endtask

    // Independent model of the expected PDM_CLK = clk/2 divider (free-running
    // from a fixed phase, NOT gated by rst -- see clk_reset.v's header
    // comment on why), so the checks below aren't just re-checking the DUT's
    // own formula against itself.
    reg expected_pdm_clk = 1'b0;
    always @(posedge clk)
        expected_pdm_clk <= ~expected_pdm_clk;

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
        check_led("at startup");

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
        check_led("partway through POR");

        // ... POR finishes, but fpga_reset_n is still held low (hub hasn't
        // seen all 4 clusters yet): rst must STAY asserted (now held by the
        // external pin, not POR) and spoke_alive must release to Hi-Z (tri-
        // state "ready" drive -- see clk_reset.v's header comment; an
        // external pull-up, not modeled here, is what turns this into a
        // logic 1 on the shared wired-AND net) -- "POR done, parked in
        // reset, waiting on the hub".
        repeat (POR_CYCLES) @(posedge clk);
        #1;
        if (rst !== 1'b1) begin
            $display("FAIL: rst dropped while fpga_reset_n still held low");
            errors = errors + 1;
        end
        if (spoke_alive !== 1'bz) begin
            $display("FAIL: spoke_alive not released to Hi-Z once POR done + still in external reset");
            errors = errors + 1;
        end
        check_led("POR done, parked in external reset");

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
        check_led("just after fpga_reset_n released");

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
            check_led("running (post-POR loop)");
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
        if (spoke_alive !== 1'bz) begin
            $display("FAIL: spoke_alive did not release to Hi-Z on a later fpga_reset_n low pulse (por_done should stay latched)");
            errors = errors + 1;
        end
        check_led("reasserted on a later fpga_reset_n low pulse");

        if (errors == 0) begin
            $display("PASS: tb_clk_reset, clk passthrough + PDM_CLK/2 divider + POR timing + fpga_reset_n/spoke_alive handshake + LED health indicator verified");
            $finish;
        end else begin
            $display("FAIL: tb_clk_reset, %0d errors", errors);
            $finish;
        end
    end
endmodule
