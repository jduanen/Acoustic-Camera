`timescale 1ns / 1ps

// One-shot DP83867 configuration over MDIO (IEEE 802.3 clause 22): advertise
// 1000 Mb/s only, restart autonegotiation, then read the advertisement back.
// The packetizer only keeps up at 1000 Mb/s -- at 10/100 the MAC back-pressures
// it into overrunning its ping-pong buffer -- and the board's straps advertise
// 10/100/1000.
//
//   write reg 0x04 ANAR = 0x0001   selector only: clears 10/100 HD/FD (default 0x01E1)
//   write reg 0x00 BMCR = 0x1340   autoneg enable + restart, 1000 Mb/s, full duplex
//   read  reg 0x04 ANAR            must return 0x0001, else `fault`
// The 1000BASE-T control register (0x09) already advertises 1000 by default.
// A partner that can't do 1000 Mb/s then gets no link, rather than a link the
// design can't keep up with. The read-back catches a PHY that ignored the write
// (it still reads 0x01E1) or doesn't answer at all (MDIO floats high: 0xFFFF);
// `fault` latches until rst and drives the status LED red.
//
// Starts WAIT_CYCLES clk cycles after rst falls (rst = PHY held in reset), to
// cover the DP83867's 195 us post-reset delay before MDIO access. MDC is clk /
// (2*MDC_HALF). This end changes MDIO on MDC's falling edge, so a bit is stable
// for half a period either side of the PHY's rising-edge sample (DP83867: 10 ns
// setup and hold). In the read, this end drives the preamble through the
// register address (bits 0-45), then releases MDIO for the turnaround; the PHY
// drives each data bit at most 10 ns after an MDC rising edge, and it is
// sampled as MDC next rises (mdio_i registered on the clk edge that raises
// MDC), ~650 ns later. MDIO is released (high-Z, board pull-up R8) whenever
// this end isn't driving it.
module mdio_init #(
    parameter [4:0]   PHY_ADDR    = 5'd0,        // RX_D0/RX_D2 unstrapped
    parameter integer MDC_HALF    = 4,           // 12.288 MHz / 8 = 1.536 MHz
    parameter integer WAIT_CYCLES = 1 << 13      // ~667 us at 12.288 MHz
) (
    input  wire clk,
    input  wire rst,
    output reg  mdc     = 1'b0,
    output reg  mdio_o  = 1'b1,
    output reg  mdio_oe = 1'b0,
    input  wire mdio_i,
    output reg  done    = 1'b0,
    output reg  fault   = 1'b0
);
    localparam integer N_FRAMES  = 3;            // write ANAR, write BMCR, read ANAR
    localparam [15:0]  ANAR_1000 = 16'h0001;

    // preamble (32 x 1), start 01, opcode (01 write / 10 read), PHYAD, REGAD,
    // turnaround (10 for a write; released for a read), data
    function [63:0] frame(input integer i);
        case (i)
            0:       frame = {32'hFFFF_FFFF, 2'b01, 2'b01, PHY_ADDR, 5'h04, 2'b10, ANAR_1000};
            1:       frame = {32'hFFFF_FFFF, 2'b01, 2'b01, PHY_ADDR, 5'h00, 2'b10, 16'h1340};
            default: frame = {32'hFFFF_FFFF, 2'b01, 2'b10, PHY_ADDR, 5'h04, 18'h3FFFF};
        endcase
    endfunction
    localparam integer READ_FRAME = 2;
    localparam integer READ_DRIVE = 46;          // bits this end drives in a read

    reg [$clog2(WAIT_CYCLES+1)-1:0] wait_cnt = 0;
    reg [$clog2(MDC_HALF)-1:0]      div      = 0;
    reg [6:0]                       bit_idx  = 0;    // bits presented so far in this frame
    reg [1:0]                       frm_idx  = 0;
    reg [63:0]                      shreg    = 0;
    reg [15:0]                      rd_data  = 0;
    reg                             busy     = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            wait_cnt <= 0; div <= 0; bit_idx <= 0; frm_idx <= 0;
            busy <= 1'b0; done <= 1'b0; fault <= 1'b0;
            mdc <= 1'b0; mdio_o <= 1'b1; mdio_oe <= 1'b0;
        end else if (!busy && !done) begin
            if (wait_cnt < WAIT_CYCLES) begin
                wait_cnt <= wait_cnt + 1'b1;
            end else begin
                busy <= 1'b1;
                shreg <= frame(0);
                bit_idx <= 0;
            end
        end else if (busy) begin
            if (div != MDC_HALF - 1) begin
                div <= div + 1'b1;
            end else begin
                div <= 0;
                if (mdc) begin
                    // falling edge: present the next bit (or the gap after a frame)
                    mdc <= 1'b0;
                    if (bit_idx < 64) begin
                        mdio_oe <= (frm_idx != READ_FRAME) || (bit_idx < READ_DRIVE);
                        mdio_o  <= shreg[63];
                        shreg   <= {shreg[62:0], 1'b0};
                        bit_idx <= bit_idx + 1'b1;
                    end else begin
                        mdio_oe <= 1'b0;                 // release between/after frames
                        mdio_o  <= 1'b1;
                        if (frm_idx == N_FRAMES - 1) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            fault <= (rd_data != ANAR_1000);
                        end else begin
                            frm_idx <= frm_idx + 1'b1;
                            shreg   <= frame(frm_idx + 1);
                            bit_idx <= 0;
                        end
                    end
                end else begin
                    // rising edge: the PHY samples what this end drives; in the
                    // read's data phase (bits 48-63) sample what the PHY drives
                    mdc <= 1'b1;
                    if (frm_idx == READ_FRAME && bit_idx >= 49 && bit_idx <= 64)
                        rd_data <= {rd_data[14:0], mdio_i};
                end
            end
        end
    end
endmodule
