`timescale 1ns / 1ps

// Checks mdio_init.v at its default (board) parameters against mdio_phy_model.v
// in three PHY behaviours, each in its own bench:
//   normal         -> two writes, one read, ANAR reads back 0x0001, fault = 0
//   absent         -> MDIO floats high, read returns 0xFFFF,        fault = 1
//   ignores writes -> ANAR still the default 0x01E1,                fault = 1
// In every bench: the frames the model decodes are exactly write ANAR = 0x0001,
// write BMCR = 0x1340, read ANAR, at PHY address 0; nothing is driven until
// >= 195 us after reset releases (DP83867 post-reset MDIO delay); MDC is
// within IEEE 802.3's 2.5 MHz (high and low >= 160 ns, period >= 400 ns); the
// bits this end drives never change within 10 ns of an MDC rising edge
// (DP83867 setup/hold); this end and the PHY never drive MDIO at once; and
// MDIO is released afterwards with done set.
module mdio_bench #(
    parameter integer MODE = 0,
    parameter         EXPECT_FAULT = 1'b0
) (
    output reg finished = 1'b0,
    output reg [31:0] errors = 0
);
    reg clk = 1'b0;
    always #40.690 clk = ~clk;                       // 12.288 MHz

    reg rst = 1'b1;
    wire mdc, mdio_o, mdio_oe, done, fault;
    wire mdio = mdio_oe ? mdio_o : 1'bz;
    pullup (mdio);                                   // board R8

    mdio_init dut (.clk(clk), .rst(rst), .mdc(mdc), .mdio_o(mdio_o), .mdio_oe(mdio_oe),
                   .mdio_i(mdio), .done(done), .fault(fault));
    mdio_phy_model #(.ADDR(5'd0), .MODE(MODE)) phy (.mdc(mdc), .mdio(mdio));

    // ---- what the model saw: frame-by-frame ----
    integer n_frames = 0;
    always @(posedge mdc) #1 if (phy.idx == -1 && phy.ones == 0 && (phy.writes + phy.reads) != n_frames) begin
        case (n_frames)
            0: if (!(phy.op == 2'b01 && phy.rg == 5'h04 && phy.wbits[15:0] == 16'h0001)) begin
                   $display("FAIL[mode %0d]: frame 0 not write ANAR=0x0001 (op=%b reg=%h data=%h)", MODE, phy.op, phy.rg, phy.wbits[15:0]);
                   errors = errors + 1;
               end
            1: if (!(phy.op == 2'b01 && phy.rg == 5'h00 && phy.wbits[15:0] == 16'h1340)) begin
                   $display("FAIL[mode %0d]: frame 1 not write BMCR=0x1340 (op=%b reg=%h data=%h)", MODE, phy.op, phy.rg, phy.wbits[15:0]);
                   errors = errors + 1;
               end
            2: if (!(phy.op == 2'b10 && phy.rg == 5'h04)) begin
                   $display("FAIL[mode %0d]: frame 2 not read ANAR (op=%b reg=%h)", MODE, phy.op, phy.rg);
                   errors = errors + 1;
               end
            default: begin
                   $display("FAIL[mode %0d]: unexpected frame %0d", MODE, n_frames);
                   errors = errors + 1;
               end
        endcase
        n_frames = phy.writes + phy.reads;
    end

    // ---- timing and bus ownership ----
    realtime t_rst_off = 0, t_rise = -1e9, t_fall = -1e9, t_drv = -1e9;
    reg driven_seen = 1'b0;
    always @(posedge mdio_oe) if (!driven_seen) begin
        driven_seen = 1'b1;
        if ($realtime - t_rst_off < 195_000) begin
            $display("FAIL[mode %0d]: MDIO driven %.1f us after reset release (< 195 us)", MODE, ($realtime - t_rst_off) / 1000);
            errors = errors + 1;
        end
    end
    always @(mdio_o or mdio_oe) if (mdio_oe) begin    // this end's own transitions
        if ($realtime - t_rise < 10.0) begin
            $display("FAIL[mode %0d]: MDIO driven %.1f ns after MDC rose (< 10 ns hold)", MODE, $realtime - t_rise);
            errors = errors + 1;
        end
        t_drv = $realtime;
    end
    always @(posedge mdc) begin
        if (mdio_oe && $realtime - t_drv < 10.0) begin
            $display("FAIL[mode %0d]: MDC rose %.1f ns after MDIO driven (< 10 ns setup)", MODE, $realtime - t_drv);
            errors = errors + 1;
        end
        if ($realtime - t_fall < 160.0 || $realtime - t_rise < 400.0) begin
            $display("FAIL[mode %0d]: MDC low %.1f ns / period %.1f ns", MODE, $realtime - t_fall, $realtime - t_rise);
            errors = errors + 1;
        end
        t_rise = $realtime;
    end
    always @(negedge mdc) begin
        if ($realtime - t_rise < 160.0) begin
            $display("FAIL[mode %0d]: MDC high %.1f ns", MODE, $realtime - t_rise);
            errors = errors + 1;
        end
        t_fall = $realtime;
    end
    always @(mdio_oe or phy.drv) if (mdio_oe === 1'b1 && phy.drv === 1'b1) begin
        $display("FAIL[mode %0d]: MDIO driven by both ends", MODE);
        errors = errors + 1;
    end

    initial begin
        repeat (5) @(posedge clk);
        #1 rst = 1'b0; t_rst_off = $realtime;
        wait (done);
        repeat (200) @(posedge clk);
        if (mdio_oe !== 1'b0 || mdio !== 1'b1) begin
            $display("FAIL[mode %0d]: MDIO not released after done (oe=%b mdio=%b)", MODE, mdio_oe, mdio);
            errors = errors + 1;
        end
        if (phy.writes != 2 || phy.reads != 1) begin
            $display("FAIL[mode %0d]: model saw %0d writes, %0d reads (want 2, 1)", MODE, phy.writes, phy.reads);
            errors = errors + 1;
        end
        if (fault !== EXPECT_FAULT) begin
            $display("FAIL[mode %0d]: fault=%b, expected %b (read back %04h)", MODE, fault, EXPECT_FAULT, dut.rd_data);
            errors = errors + 1;
        end
        $display("mode %0d: read back %04h, fault %b, %0d errors", MODE, dut.rd_data, fault, errors);
        finished = 1'b1;
    end
endmodule

module tb_mdio_init;
    wire f0, f1, f2;
    wire [31:0] e0, e1, e2;
    mdio_bench #(.MODE(0), .EXPECT_FAULT(1'b0)) normal  (.finished(f0), .errors(e0));
    mdio_bench #(.MODE(1), .EXPECT_FAULT(1'b1)) absent  (.finished(f1), .errors(e1));
    mdio_bench #(.MODE(2), .EXPECT_FAULT(1'b1)) ignores (.finished(f2), .errors(e2));

    initial begin
        wait (f0 && f1 && f2);
        if (e0 + e1 + e2 == 0) $display("PASS: tb_mdio_init");
        else                   $display("FAIL: tb_mdio_init (%0d errors)", e0 + e1 + e2);
        $finish;
    end
    initial begin
        #5_000_000;
        $display("FAIL: tb_mdio_init timed out");
        $finish;
    end
endmodule
