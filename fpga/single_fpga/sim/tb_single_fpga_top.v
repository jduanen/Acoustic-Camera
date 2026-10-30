`timescale 1ns / 1ps

// End-to-end check of the board top from its pins: TCXO_CLK in, PDM_CLK out
// to 48 modelled mic pairs, their DATA back in through pdm_sampler.v, the
// real 96-channel CIC+FIR (single_fpga_pipeline_top.v) on the gated
// pipe_clk, the packetizer, and the RGMII MAC out to RGMII_TXD/TX_CTL/TXC.
// Two checks against golden/gen_vectors.py's gen_board_top_vectors():
//   - each channel's FIR outputs (tapped hierarchically), bit-exact against
//     board_top_expected_channels.mem;
//   - the frames on the RGMII pins, decoded from TXC's two edges back into
//     bytes: 7 x 0x55 + 0xD5 preamble/SFD, the body bit-exact against
//     board_top_expected_packets.mem, and a correct Ethernet FCS.
// RX/ARP: frames are driven onto RGMII_RXD/RX_CTL the way the PHY sends them
// (RXC shifted 2 ns, so centred in each nibble): A, an ARP request for the
// board's IP (broadcast); B, one for another IP; C, A again with a corrupted
// FCS; D, a unicast request from a second host, sent while the first UDP
// frame is on the wire. Exactly two ARP replies must come out on TX -- A's
// before any UDP frame, D's right after UDP frame 0 (not cutting into it) --
// each bit-exact (padded by the MAC to 60 bytes) with a good FCS.
// SYS_CLK is a 200 MHz differential source and RGMII_RXC an idle 125 MHz
// clock (the MAC sets its speed from RXC's rate). PHY_RESET_N must stay low
// for PHY_RST_CYCLES (shortened here) and then go high, and the MDIO PHY
// configuration must reach the pins: a PHY register model (mdio_phy_model.v)
// on MDC/MDIO must see the two writes and the read-back, mdio_init must
// finish without a fault, and the status LED must never show red. (Frame
// contents and MDIO timing are checked in detail in tb_mdio_init.v.)
//
// Mic model as in tb_pdm_sampler.v, at the slow datasheet/board corner: each
// mic drives TDD after its PDM_CLK edge (L after the fall, R after the rise),
// releases THZ after the opposite edge, and an undriven line reads X. The
// streams are pipeline_pdm_bits.mem's (channel 2n = line n's L, 2n+1 = R),
// the same stimulus tb_gbe_pipeline.v uses.
//
// Alignment (TCXO edges e = 0,1,2..., pdm_div = e mod 4 just before edge e):
// POR releases after edge 16; pipe_rst, which only updates on pdm_div == 0
// edges, follows at edge 20, so pipe_clk's first post-reset edge is 22. The
// mics start streaming at the first PDM_CLK fall after POR releases (launched
// at edge 19), and the R bit after it at edge 21. pdm_sampler captures those
// at edges 21 and 23 and presents them at edge 24; pipe_clk takes them at
// edge 26. Edge 22 sees the pair requested before streaming, zero -- so every
// channel's CIC input is one zero followed by the stream, the `lead = 1` in
// gen_board_top_vectors().
module tb_single_fpga_top;
    localparam integer N_CH      = 96;
    localparam integer N_LINES   = 48;
    localparam integer N_FRAMES  = 17;   // = gen_vectors.py's N_FRAMES
    localparam integer N_SAMPLES = N_FRAMES * 64;
    localparam integer FIR_W     = 24;

    reg tcxo = 1'b0;
    always #40.690 tcxo = ~tcxo;         // 12.288 MHz

    wire pdm_clk, led_r, led_g, led_b;
    wire [N_LINES-1:0] data;

    localparam integer PHY_RST_CYCLES = 64;
    reg [3:0] rxd_d = 4'h0;
    reg       rxctl_d = 1'b0;

    reg sys_clk = 1'b0;
    always #2.5 sys_clk = ~sys_clk;     // 200 MHz
    reg rxc = 1'b0;
    always #4.0 rxc = ~rxc;             // 125 MHz, idle link

    wire       txc, tx_ctl, phy_reset_n;
    wire [3:0] txd;

    wire mdc, mdio;
    pullup (mdio);                      // board R8
    mdio_phy_model #(.ADDR(5'd0), .MODE(0)) phy (.mdc(mdc), .mdio(mdio));

    single_fpga_top #(.PHY_RST_CYCLES(PHY_RST_CYCLES), .MDIO_WAIT(64)) dut (
        .TCXO_CLK(tcxo), .PDM_CLK(pdm_clk),
        .LED_R(led_r), .LED_G(led_g), .LED_B(led_b),
        .DATA(data),
        .SYS_CLK_P(sys_clk), .SYS_CLK_N(~sys_clk),
        .RGMII_TXC(txc), .RGMII_TXD(txd), .RGMII_TX_CTL(tx_ctl),
        .RGMII_RXC(rxc), .RGMII_RXD(rxd_d), .RGMII_RX_CTL(rxctl_d),
        .PHY_RESET_N(phy_reset_n),
        .MDC(mdc), .MDIO(mdio)
    );

    reg red_seen = 1'b0;
    always @(posedge tcxo) if (led_r === 1'b1 && led_g === 1'b0) red_seen = 1'b1;

    // ---- mics ----
    localparam real CLK_DLY = 6.2, TDD = 80.0, THZ = 30.0, RET_DLY = 1.9;

    reg pdm_mem [0:N_CH*N_SAMPLES-1];
    initial $readmemb("../vectors/pipeline_pdm_bits.mem", pdm_mem);

    wire mic_clk;
    assign #(CLK_DLY) mic_clk = pdm_clk;

    reg [N_LINES-1:0] line = {N_LINES{1'bx}};
    reg [N_LINES-1:0] l_bit, r_bit;
    reg     l_on = 1'b0, r_on = 1'b0;
    integer l_idx = 0, r_idx = 0, n;

    always @(negedge mic_clk) begin                  // L requested, R released
        if (!dut.rst) l_on = 1'b1;
        for (n = 0; n < N_LINES; n = n + 1)
            l_bit[n] = (l_on && l_idx < N_SAMPLES) ? pdm_mem[(2*n) * N_SAMPLES + l_idx] : 1'b0;
        if (l_on) l_idx = l_idx + 1;
        line <= #(THZ) {N_LINES{1'bx}};
        line <= #(TDD) l_bit;
    end
    always @(posedge mic_clk) begin                  // R requested, L released
        if (l_on) r_on = 1'b1;
        for (n = 0; n < N_LINES; n = n + 1)
            r_bit[n] = (r_on && r_idx < N_SAMPLES) ? pdm_mem[(2*n+1) * N_SAMPLES + r_idx] : 1'b0;
        if (r_on) r_idx = r_idx + 1;
        line <= #(THZ) {N_LINES{1'bx}};
        line <= #(TDD) r_bit;
    end

    assign #(RET_DLY) data = line;

    // ---- check ----
    reg [FIR_W-1:0] exp_mem [0:N_FRAMES*N_CH-1];
    initial $readmemh("../vectors/board_top_expected_channels.mem", exp_mem);

    integer frame [0:N_CH-1];
    integer c, errors = 0, checked = 0;
    initial for (c = 0; c < N_CH; c = c + 1) frame[c] = 0;
    reg fir_done = 1'b0;

    always @(posedge dut.pipe_clk) begin
        for (c = 0; c < N_CH; c = c + 1) begin
            if (dut.u_pipeline.fir_valid[c] && frame[c] < N_FRAMES) begin
                if (dut.u_pipeline.fir_out_flat[FIR_W*c +: FIR_W] !== exp_mem[frame[c]*N_CH + c]) begin
                    if (errors < 20)
                        $display("FAIL: ch %0d frame %0d: got %06h expected %06h", c, frame[c],
                                 dut.u_pipeline.fir_out_flat[FIR_W*c +: FIR_W], exp_mem[frame[c]*N_CH + c]);
                    errors = errors + 1;
                end
                frame[c] = frame[c] + 1;
                checked = checked + 1;
            end
        end
        if (checked == N_FRAMES * N_CH && !fir_done) begin
            fir_done = 1'b1;
            $display("FIR: %0d channels x %0d frames checked, %0d wrong", N_CH, N_FRAMES, errors);
        end
    end

    // ---- PHY reset ----
    integer tcxo_edges = 0;
    always @(posedge tcxo) tcxo_edges = tcxo_edges + 1;
    reg phy_rst_seen_low = 1'b0;
    always @(posedge tcxo) begin
        if (tcxo_edges < PHY_RST_CYCLES && phy_reset_n !== 1'b0) begin
            $display("FAIL: PHY_RESET_N=%b at TCXO edge %0d, before PHY_RST_CYCLES", phy_reset_n, tcxo_edges);
            errors = errors + 1;
        end
        if (tcxo_edges == PHY_RST_CYCLES + 4 && phy_reset_n !== 1'b1) begin
            $display("FAIL: PHY_RESET_N=%b after PHY_RST_CYCLES", phy_reset_n);
            errors = errors + 1;
        end
    end

    // ---- RGMII TX decode ----
    localparam integer PKT_BYTES = 1494;              // = gbe_packetizer's TOTAL_PKT_BYTES
    localparam integer N_PACKETS = N_FRAMES / 5;
    reg [7:0] pkt_exp [0:N_PACKETS*PKT_BYTES-1];
    initial $readmemh("../vectors/board_top_expected_packets.mem", pkt_exp);

    reg [7:0] fbuf [0:2047];                          // one frame, preamble to FCS
    integer   flen = 0, n_pkts = 0, k;
    reg [3:0] lo;
    reg       en_r = 1'b0, en_lo;

    function [31:0] crc32_byte(input [31:0] crc, input [7:0] b);
        integer i;
        begin
            crc32_byte = crc ^ b;
            for (i = 0; i < 8; i = i + 1)
                crc32_byte = crc32_byte[0] ? (crc32_byte >> 1) ^ 32'hEDB88320 : crc32_byte >> 1;
        end
    endfunction

    // ARP replies expected on TX, in order (requests A and D)
    localparam [47:0] BOARD_MAC = 48'h001122334455;
    localparam [31:0] BOARD_IP  = 32'hC0A80002;
    localparam integer N_ARP = 2;
    reg [47:0] arp_sha [0:N_ARP-1];
    reg [31:0] arp_spa [0:N_ARP-1];
    initial begin
        arp_sha[0] = 48'h020000000001; arp_spa[0] = 32'hC0A80001;
        arp_sha[1] = 48'h020000000002; arp_spa[1] = 32'hC0A80005;
    end
    integer n_arp = 0, n_udp = 0, udp_at_arp1 = -1;

    function [7:0] arp_reply_byte(input integer a, input integer i);
        reg [47:0] sha; reg [31:0] spa;
        begin
            sha = arp_sha[a]; spa = arp_spa[a];
            if      (i < 6)  arp_reply_byte = sha >> (8 * (5 - i));
            else if (i < 12) arp_reply_byte = BOARD_MAC >> (8 * (11 - i));
            else if (i < 22) arp_reply_byte = (i == 12) ? 8'h08 : (i == 13) ? 8'h06 : (i == 14) ? 8'h00 :
                                              (i == 15) ? 8'h01 : (i == 16) ? 8'h08 : (i == 17) ? 8'h00 :
                                              (i == 18) ? 8'h06 : (i == 19) ? 8'h04 : (i == 20) ? 8'h00 : 8'h02;
            else if (i < 28) arp_reply_byte = BOARD_MAC >> (8 * (27 - i));
            else if (i < 32) arp_reply_byte = BOARD_IP >> (8 * (31 - i));
            else if (i < 38) arp_reply_byte = sha >> (8 * (37 - i));
            else if (i < 42) arp_reply_byte = spa >> (8 * (41 - i));
            else             arp_reply_byte = 8'h00;           // MAC padding to 60
        end
    endfunction

    task check_frame;
        reg [31:0] crc, fcs;
        integer e0, body, nf;
        reg [7:0] want;
        begin
            e0 = errors;
            body = flen - 12;                                 // minus preamble/SFD and FCS
            for (k = 0; k < 7; k = k + 1)
                if (fbuf[k] !== 8'h55) errors = errors + 1;
            if (fbuf[7] !== 8'hD5) errors = errors + 1;
            if (errors != e0) $display("FAIL: TX frame preamble/SFD wrong");
            crc = 32'hFFFFFFFF;
            for (k = 0; k < body && k < 2036; k = k + 1) crc = crc32_byte(crc, fbuf[8+k]);
            fcs = {fbuf[8+body+3], fbuf[8+body+2], fbuf[8+body+1], fbuf[8+body]};
            if (fcs !== ~crc) begin
                $display("FAIL: TX frame FCS %08h, expected %08h", fcs, ~crc);
                errors = errors + 1;
            end
            if ({fbuf[8+12], fbuf[8+13]} == 16'h0806) begin
                nf = n_arp;
                if (n_arp >= N_ARP) begin
                    $display("FAIL: unexpected ARP reply %0d", n_arp);
                    errors = errors + 1;
                end else if (body != 60) begin
                    $display("FAIL: ARP reply %0d body is %0d bytes, expected 60", n_arp, body);
                    errors = errors + 1;
                end else begin
                    for (k = 0; k < 60; k = k + 1) begin
                        want = arp_reply_byte(n_arp, k);
                        if (fbuf[8+k] !== want) begin
                            if (errors - e0 < 10)
                                $display("FAIL: ARP reply %0d byte %0d got %02h expected %02h", n_arp, k, fbuf[8+k], want);
                            errors = errors + 1;
                        end
                    end
                end
                if (n_arp == 0 && n_udp != 0) begin
                    $display("FAIL: ARP reply A came after %0d UDP frames, expected before any", n_udp);
                    errors = errors + 1;
                end
                if (n_arp == 1) udp_at_arp1 = n_udp;
                $display("RGMII: ARP reply %0d, %0d bytes, %0s", n_arp, flen, errors == e0 ? "ok" : "BAD");
                n_arp = n_arp + 1;
            end else begin
                nf = n_udp;
                if (body != PKT_BYTES) begin
                    $display("FAIL: UDP frame %0d body is %0d bytes, expected %0d", n_udp, body, PKT_BYTES);
                    errors = errors + 1;
                end else if (n_udp >= N_PACKETS) begin
                    $display("FAIL: unexpected UDP frame %0d beyond %0d", n_udp, N_PACKETS);
                    errors = errors + 1;
                end else begin
                    for (k = 0; k < PKT_BYTES; k = k + 1)
                        if (fbuf[8+k] !== pkt_exp[n_udp*PKT_BYTES + k]) begin
                            if (errors - e0 < 10)
                                $display("FAIL: UDP frame %0d byte %0d got %02h expected %02h",
                                         n_udp, k, fbuf[8+k], pkt_exp[n_udp*PKT_BYTES + k]);
                            errors = errors + 1;
                        end
                end
                $display("RGMII: UDP frame %0d, %0d bytes, %0s", n_udp, flen, errors == e0 ? "ok" : "BAD");
                n_udp = n_udp + 1;
            end
            n_pkts = n_pkts + 1;
        end
    endtask

    // ---- RGMII RX: frames into the MAC, as the PHY sends them ----
    reg [7:0] rbuf [0:127];
    integer   rlen = 0;

    task rx_byte(input [7:0] b);                      // data centred on RXC (2 ns shift)
        begin
            @(negedge rxc); #2.0; rxd_d = b[3:0]; rxctl_d = 1'b1;
            @(posedge rxc); #2.0; rxd_d = b[7:4]; rxctl_d = 1'b1;
        end
    endtask

    task rx_send(input bad_fcs);
        reg [31:0] crc;
        integer i;
        begin
            crc = 32'hFFFFFFFF;
            for (i = 0; i < rlen; i = i + 1) crc = crc32_byte(crc, rbuf[i]);
            crc = ~crc ^ (bad_fcs ? 32'h00000001 : 32'h0);
            for (i = 0; i < 7; i = i + 1) rx_byte(8'h55);
            rx_byte(8'hD5);
            for (i = 0; i < rlen; i = i + 1) rx_byte(rbuf[i]);
            for (i = 0; i < 4; i = i + 1) rx_byte(crc >> (8 * i));
            @(negedge rxc); #2.0; rxd_d = 4'h0; rxctl_d = 1'b0;
            repeat (24) @(posedge rxc);               // inter-frame gap
        end
    endtask

    task build_arp_request(input [47:0] dst, input [47:0] sha, input [31:0] spa, input [31:0] tpa);
        integer i;
        begin
            for (i = 0; i < 60; i = i + 1) rbuf[i] = 8'h00;   // includes the 18-byte pad
            for (i = 0; i < 6; i = i + 1) begin
                rbuf[i]      = dst >> (8 * (5 - i));
                rbuf[6 + i]  = sha >> (8 * (5 - i));
                rbuf[22 + i] = sha >> (8 * (5 - i));
            end
            rbuf[12] = 8'h08; rbuf[13] = 8'h06; rbuf[15] = 8'h01; rbuf[16] = 8'h08;
            rbuf[18] = 8'h06; rbuf[19] = 8'h04; rbuf[21] = 8'h01;
            for (i = 0; i < 4; i = i + 1) begin
                rbuf[28 + i] = spa >> (8 * (3 - i));
                rbuf[38 + i] = tpa >> (8 * (3 - i));
            end
            rlen = 60;
        end
    endtask

    initial begin
        #30_000;
        build_arp_request(48'hFFFFFFFFFFFF, arp_sha[0], arp_spa[0], BOARD_IP);      rx_send(1'b0); // A
        build_arp_request(48'hFFFFFFFFFFFF, arp_sha[0], arp_spa[0], 32'hC0A80063);  rx_send(1'b0); // B
        #5_000;
        build_arp_request(48'hFFFFFFFFFFFF, arp_sha[0], arp_spa[0], BOARD_IP);      rx_send(1'b1); // C
        wait (n_udp == 0 && en_r && flen > 200);                                     // inside UDP frame 0
        build_arp_request(BOARD_MAC, arp_sha[1], arp_spa[1], BOARD_IP);             rx_send(1'b0); // D
    end

    // TX pin timing: the PHY delays TXC by 2 ns internally (TX skew strap mode
    // 1), so TXC must leave the FPGA edge-aligned with its data -- RGMII TskewT,
    // every data change within 0.5 ns of a TXC edge. A data change too far from
    // the last edge is held pending and must be matched by the next one. This
    // also catches a 90-degree-shifted TXC, which decoding alone can't see in a
    // zero-delay sim.
    realtime t_data = 0, t_txc = 0;
    reg pend = 1'b0;
    always @(txd or tx_ctl) begin
        if ($realtime - t_txc > 0.5) pend = 1'b1;
        t_data = $realtime;
    end
    always @(txc) begin
        if (pend && $realtime - t_data > 0.5 && $realtime > 1000) begin
            if (errors < 20) $display("FAIL: RGMII TX data changed %.2f ns from any TXC edge (> 0.5)", $realtime - t_data);
            errors = errors + 1;
        end
        pend = 1'b0;
        t_txc = $realtime;
    end

    // Decode as the PHY does: sample 2 ns (its internal TXC delay) after each
    // TXC edge -- rising = low nibble + TX_EN, falling = high nibble + TX_EN^TX_ER.
    always @(posedge txc) begin
        #2.0;
        lo = txd; en_lo = tx_ctl;
        if (!en_lo && en_r) begin
            check_frame;
            flen = 0;
        end
        en_r = en_lo;
    end
    always @(negedge txc) begin
        #2.0;
        if (en_lo) begin
            if (tx_ctl !== en_lo) begin
                $display("FAIL: TX_ER asserted in frame %0d", n_pkts);
                errors = errors + 1;
            end
            if (flen < 2048) fbuf[flen] = {txd, lo};
            flen = flen + 1;
        end
    end

    always @(posedge tcxo) if (fir_done && n_udp == N_PACKETS) begin
        if (n_arp != N_ARP || udp_at_arp1 != 1) begin
            $display("FAIL: %0d ARP replies (want %0d); reply D after %0d UDP frames (want 1)", n_arp, N_ARP, udp_at_arp1);
            errors = errors + 1;
        end
        if (dut.mdio_done !== 1'b1 || dut.mdio_fault !== 1'b0 || phy.writes != 2 || phy.reads != 1
            || phy.regs[4] !== 16'h0001 || red_seen) begin
            $display("FAIL: MDIO config: done=%b fault=%b, PHY saw %0d writes %0d reads, ANAR=%04h, red LED %0s",
                     dut.mdio_done, dut.mdio_fault, phy.writes, phy.reads, phy.regs[4], red_seen ? "seen" : "not seen");
            errors = errors + 1;
        end
        if (errors == 0)
            $display("PASS: tb_single_fpga_top, %0d channels x %0d frames, %0d UDP frames and %0d ARP replies bit-exact",
                     N_CH, N_FRAMES, N_PACKETS, N_ARP);
        else
            $display("FAIL: tb_single_fpga_top, %0d errors", errors);
        $finish;
    end

    initial begin
        #800_000;
        $display("FAIL: tb_single_fpga_top timed out, %0d/%0d outputs checked, %0d/%0d UDP frames, %0d/%0d ARP replies",
                 checked, N_FRAMES * N_CH, n_udp, N_PACKETS, n_arp, N_ARP);
        $finish;
    end
endmodule
