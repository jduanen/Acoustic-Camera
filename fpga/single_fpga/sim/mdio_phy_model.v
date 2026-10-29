`timescale 1ns / 1ps

// Behavioural DP83867 MDIO slave (IEEE 802.3 clause 22) for tb_mdio_init.v and
// tb_single_fpga_top.v. Samples MDIO on MDC rising edges; on a write to ADDR it
// stores the register, on a read it drives the turnaround 0 and the 16 data
// bits, each changing TCO after an MDC rising edge (DP83867 MDC-to-MDIO output
// delay is 0-10 ns), and releases MDIO after the last one. Resets to the
// strapped defaults this design cares about: BMCR 0x1140, ANAR 0x01E1.
//   MODE 0  normal
//   MODE 1  absent: never drives MDIO (reads see the pull-up)
//   MODE 2  ignores writes (reads return the defaults)
// writes/reads count completed frames addressed to ADDR.
module mdio_phy_model #(
    parameter [4:0]   ADDR = 5'd0,
    parameter integer MODE = 0,
    parameter real    TCO  = 5.0
) (
    input  wire mdc,
    inout  wire mdio
);
    reg [15:0] regs [0:31];
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) regs[i] = 16'h0000;
        regs[0] = 16'h1140;
        regs[4] = 16'h01E1;
    end

    reg drv = 1'b0, drv_val = 1'b1;
    assign mdio = drv ? drv_val : 1'bz;

    integer ones = 0, idx = -1, writes = 0, reads = 0;
    reg [1:0]  op;
    reg [4:0]  phy, rg;
    reg [17:0] wbits;
    reg [15:0] rdata;
    reg        prev = 1'b1;

    always @(posedge mdc) begin
        if (idx < 0) begin
            // >= 32 preamble ones, then the 01 start code (idx 32, 33)
            if (mdio === 1'b1 && prev === 1'b0 && ones >= 32) idx = 34;
            else if (mdio === 1'b1) ones = ones + 1;
            else if (ones < 32) ones = 0;
        end else begin
            if (idx < 36)       op  = {op[0], mdio};
            else if (idx < 41)  phy = {phy[3:0], mdio};
            else if (idx < 46)  rg  = {rg[3:0], mdio};
            if (op == 2'b01 && idx >= 46) wbits = {wbits[16:0], mdio};

            if (op == 2'b10 && phy == ADDR && MODE != 1) begin
                // read: drive the turnaround 0 for bit 47, then D15..D0 for 48..63
                if (idx == 46) begin
                    rdata = regs[rg];
                    drv_val <= #(TCO) 1'b0; drv <= #(TCO) 1'b1;
                end else if (idx >= 47 && idx <= 62) begin
                    drv_val <= #(TCO) rdata[62 - idx];
                end else if (idx == 63) begin
                    drv <= #(TCO) 1'b0;
                end
            end

            if (idx == 63) begin
                if (phy == ADDR && op == 2'b01) begin
                    writes = writes + 1;
                    if (MODE != 2) regs[rg] = wbits[15:0];
                end
                if (phy == ADDR && op == 2'b10) reads = reads + 1;
                idx = -1; ones = 0;
            end else begin
                idx = idx + 1;
            end
        end
        prev = mdio;
    end
endmodule
