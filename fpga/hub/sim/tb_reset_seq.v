`timescale 1ns / 1ps

// Checks reset_seq.v's power-up sequencing: starts asserted (YELLOW), stays
// asserted until all 4 spoke_alive_i bits are seen high (through the 2-FF
// synchronizer + MIN_RESET_CYCLES hold), then releases (GREEN) and never
// reverts even if the alive bits later drop -- one-shot by design, matching
// the described power-up behavior.
module tb_reset_seq;
    localparam integer MIN_RESET_CYCLES = 16;

    reg clk = 0;
    always #5 clk = ~clk;

    reg [3:0] spoke_alive_i = 4'b0000;
    wire spoke_reset_n, led_r_n, led_g_n, led_b_n;

    reset_seq #(.MIN_RESET_CYCLES(MIN_RESET_CYCLES)) dut (
        .clk(clk),
        .spoke_alive_i(spoke_alive_i),
        .spoke_reset_n(spoke_reset_n),
        .led_r_n(led_r_n), .led_g_n(led_g_n), .led_b_n(led_b_n)
    );

    integer errors = 0;
    integer i;

    task check_yellow_asserted(input [255:0] label);
        begin
            if (spoke_reset_n !== 1'b0) begin
                $display("FAIL: %0s: spoke_reset_n not asserted (got %b)", label, spoke_reset_n);
                errors = errors + 1;
            end
            if (led_r_n !== 1'b0 || led_g_n !== 1'b0) begin
                $display("FAIL: %0s: LED not YELLOW (led_r_n=%b led_g_n=%b)", label, led_r_n, led_g_n);
                errors = errors + 1;
            end
        end
    endtask

    task check_green_released(input [255:0] label);
        begin
            if (spoke_reset_n !== 1'b1) begin
                $display("FAIL: %0s: spoke_reset_n not released (got %b)", label, spoke_reset_n);
                errors = errors + 1;
            end
            if (led_r_n !== 1'b1 || led_g_n !== 1'b0) begin
                $display("FAIL: %0s: LED not GREEN (led_r_n=%b led_g_n=%b)", label, led_r_n, led_g_n);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        // let time-0 initial-value assignments settle before sampling --
        // see tb_clk_reset.v's identical note for why.
        #1;
        check_yellow_asserted("at startup");

        // 3 of 4 alive -- must stay asserted indefinitely
        spoke_alive_i = 4'b0111;
        repeat (MIN_RESET_CYCLES + 20) @(posedge clk);
        #1;
        check_yellow_asserted("with only 3/4 spokes alive");

        // 4th spoke goes alive -- must release after synchronizer + hold time
        spoke_alive_i = 4'b1111;
        repeat (8) @(posedge clk); // 2-FF sync + margin, hold_cnt already maxed
        #1;
        check_green_released("after all 4 spokes alive");

        // dropping alive bits afterward must NOT revert (one-shot sequencer)
        spoke_alive_i = 4'b0000;
        repeat (20) @(posedge clk);
        #1;
        check_green_released("after alive bits later drop (must not revert)");

        if (errors == 0) begin
            $display("PASS: tb_reset_seq, power-up sequencing + one-shot behavior verified");
            $finish;
        end else begin
            $display("FAIL: tb_reset_seq, %0d errors", errors);
            $finish;
        end
    end
endmodule
