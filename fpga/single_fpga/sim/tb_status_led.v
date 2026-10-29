`timescale 1ns / 1ps

// Checks status_led.v: YELLOW while rst, solid GREEN once running with no
// packets, GREEN/BLUE alternating while pkt_toggle keeps flipping (from an
// async clock, through the synchronizer), back to solid GREEN HOLD_CYCLES
// after the last packet, solid RED while fault is set (even while packets
// are flowing), and YELLOW again if rst re-asserts, fault or not. Small
// BLINK_BIT/HOLD_CYCLES so the run is short.
module tb_status_led;
    localparam integer BLINK_BIT   = 3;            // 8-cycle blink phases
    localparam integer HOLD_CYCLES = 64;

    reg clk = 0;
    always #5 clk = ~clk;
    reg tx_clk = 0;
    always #4 tx_clk = ~tx_clk;                    // unrelated packet clock

    reg rst = 1'b1;
    reg pkt_toggle = 1'b0;
    reg fault = 1'b0;
    reg pkts_on = 1'b0;
    integer tx_cnt = 0;
    always @(posedge tx_clk) begin
        tx_cnt <= tx_cnt + 1;
        if (pkts_on && tx_cnt % 20 == 0) pkt_toggle <= ~pkt_toggle;
    end

    wire led_r, led_g, led_b;
    status_led #(.BLINK_BIT(BLINK_BIT), .HOLD_CYCLES(HOLD_CYCLES)) dut (
        .clk(clk), .rst(rst), .pkt_toggle(pkt_toggle), .fault(fault),
        .led_r(led_r), .led_g(led_g), .led_b(led_b)
    );

    integer errors = 0;
    integer i, n_green, n_blue;

    task expect_rgb(input r, input g, input b, input [255:0] label);
        begin
            if ({led_r, led_g, led_b} !== {r, g, b}) begin
                $display("FAIL: %0s: RGB=%b%b%b, expected %b%b%b", label, led_r, led_g, led_b, r, g, b);
                errors = errors + 1;
            end
        end
    endtask

    // Sample n cycles; count GREEN-only and BLUE-only cycles, flag any other colour.
    task sample(input integer n, input [255:0] label);
        begin
            n_green = 0; n_blue = 0;
            for (i = 0; i < n; i = i + 1) begin
                @(posedge clk); #1;
                if ({led_r, led_g, led_b} === 3'b010) n_green = n_green + 1;
                else if ({led_r, led_g, led_b} === 3'b001) n_blue = n_blue + 1;
                else begin
                    $display("FAIL: %0s: unexpected RGB=%b%b%b", label, led_r, led_g, led_b);
                    errors = errors + 1;
                end
            end
        end
    endtask

    initial begin
        #1;
        expect_rgb(1, 1, 0, "at startup (rst)");
        repeat (10) @(posedge clk); #1;
        expect_rgb(1, 1, 0, "held in rst");

        rst = 1'b0;
        sample(4 * (1 << (BLINK_BIT + 1)), "running, no packets");
        if (n_blue != 0) begin
            $display("FAIL: blue seen with no packets (%0d cycles)", n_blue);
            errors = errors + 1;
        end

        pkts_on = 1'b1;
        repeat (10) @(posedge clk);                 // first edge through the sync
        sample(4 * (1 << (BLINK_BIT + 1)), "streaming");
        if (n_blue < 3 * (1 << BLINK_BIT) || n_green < 3 * (1 << BLINK_BIT)) begin
            $display("FAIL: streaming didn't alternate (green %0d, blue %0d)", n_green, n_blue);
            errors = errors + 1;
        end

        pkts_on = 1'b0;
        repeat (HOLD_CYCLES + 40) @(posedge clk);   // last edge + sync + hold
        sample(4 * (1 << (BLINK_BIT + 1)), "after packets stop");
        if (n_blue != 0) begin
            $display("FAIL: still blinking %0d cycles after packets stopped", n_blue);
            errors = errors + 1;
        end

        fault = 1'b1; pkts_on = 1'b1;
        repeat (10) @(posedge clk);
        for (i = 0; i < 4 * (1 << (BLINK_BIT + 1)); i = i + 1) begin
            @(posedge clk); #1;
            if ({led_r, led_g, led_b} !== 3'b100) begin
                if (errors < 20) $display("FAIL: fault while streaming: RGB=%b%b%b, expected 100", led_r, led_g, led_b);
                errors = errors + 1;
            end
        end

        rst = 1'b1;
        @(posedge clk); #1;
        expect_rgb(1, 1, 0, "rst re-asserted, fault set");
        fault = 1'b0; pkts_on = 1'b0;
        @(posedge clk); #1;
        expect_rgb(1, 1, 0, "rst re-asserted");

        if (errors == 0) begin
            $display("PASS: tb_status_led");
        end else begin
            $display("FAIL: tb_status_led (%0d errors)", errors);
        end
        $finish;
    end
endmodule
