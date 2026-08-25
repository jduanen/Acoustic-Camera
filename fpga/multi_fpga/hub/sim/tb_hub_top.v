`timescale 1ns / 1ps

// Integration test: hub_top.v driving all 4 spokes with independently
// distinct frames (a different fpga/multi_fpga/cluster/vectors frame per spoke) to
// catch cross-spoke channel mixups a single shared tone would hide -- same
// reasoning gen_pdm_stimulus.py documents for the cluster's per-channel
// tones. Reuses fpga/multi_fpga/cluster/vectors/framer_channels.mem /
// framer_expected.mem (the same vectors tb_spoke_deframer.v checks a single
// spoke against) -- spoke_deframer.v's correctness doesn't depend on the
// golden vectors' FRAME_CYCLES=64 idle-cycle count (it re-syncs from every
// STROBE edge, see its own header comment), so no new vectors are needed
// here either.
//
// hub_top.v itself generates SPOKE_CLK -- this testbench drives TCXO_CLK and
// derives its own per-spoke drive timing from the DUT's own SPOKE_CLK
// output, mirroring how a real cluster board (clk_reset.v) receives it,
// rather than assuming any particular phase relationship to TCXO_CLK.
//
// ch_data_flat/valid aren't top-level hub_top.v ports -- reached here via
// hierarchical reference into the DUT, simulation-only.
//
// USB side: drives USB_CLKOUT (independent 60MHz clock, genuinely
// asynchronous to tcxo_clk -- no attempt made here to phase-lock them,
// proving the CDC doesn't need it) and USB_TXE_N held low (FIFO always
// ready), then checks the first spoke's full 76-byte record against
// fpga/multi_fpga/USB_FRAMING.md's format -- spoke0 is served first since all 4
// spokes' valid pulses land at effectively the same time here and
// usb_framer.v's arbiter starts at spoke0 after reset (see its own header
// comment). Byte-level framing itself is already checked bit-exactly by
// tb_usb_framer.v against usb_framer_golden.py; this only needs to prove
// hub_top.v wires the 4 deframers into usb_framer.v correctly end-to-end.
module tb_hub_top;
    localparam integer N_CH         = 24;
    localparam integer DATA_WIDTH   = 24;
    localparam integer FRAME_CYCLES = 64;
    localparam integer RECORD_LEN   = 76;

    reg tcxo_clk = 0;
    always #(81.380/2) tcxo_clk = ~tcxo_clk; // 12.288 MHz, matches hub_top.xdc's create_clock

    reg usb_clkout = 0;
    always #(16.667/2) usb_clkout = ~usb_clkout; // 60 MHz, FT232H's CLKOUT
    reg usb_txe_n = 1'b1;

    reg spokes_alive = 0; // wired-AND net driven by 4 clusters -- see reset_seq.v

    wire spoke_clk, fpga_reset_n, led0_r, led0_g, led0_b;

    reg [5:0] rise0 = 0, fall0 = 0; reg strobe0 = 0;
    reg [5:0] rise1 = 0, fall1 = 0; reg strobe1 = 0;
    reg [5:0] rise2 = 0, fall2 = 0; reg strobe2 = 0;
    reg [5:0] rise3 = 0, fall3 = 0; reg strobe3 = 0;

    // #1 delay on each mux: see tb_spoke_deframer.v's identical declaration
    // for why -- without it, the mux's own switch races the DUT's
    // same-edge posedge/negedge captures.
    wire [5:0] spoke_d0; assign #1 spoke_d0 = spoke_clk ? rise0 : fall0;
    wire [5:0] spoke_d1; assign #1 spoke_d1 = spoke_clk ? rise1 : fall1;
    wire [5:0] spoke_d2; assign #1 spoke_d2 = spoke_clk ? rise2 : fall2;
    wire [5:0] spoke_d3; assign #1 spoke_d3 = spoke_clk ? rise3 : fall3;

    wire [7:0] usb_d;
    wire       usb_wr_n;

    hub_top dut (
        .TCXO_CLK(tcxo_clk),
        .FPGA_RESET_N(fpga_reset_n),
        .SPOKES_ALIVE(spokes_alive),
        .led0_r(led0_r), .led0_g(led0_g), .led0_b(led0_b),
        .SPOKE_CLK(spoke_clk),
        .SPOKE0_D0(spoke_d0[0]), .SPOKE0_D1(spoke_d0[1]), .SPOKE0_D2(spoke_d0[2]),
        .SPOKE0_D3(spoke_d0[3]), .SPOKE0_D4(spoke_d0[4]), .SPOKE0_D5(spoke_d0[5]),
        .SPOKE0_STROBE(strobe0),
        .SPOKE1_D0(spoke_d1[0]), .SPOKE1_D1(spoke_d1[1]), .SPOKE1_D2(spoke_d1[2]),
        .SPOKE1_D3(spoke_d1[3]), .SPOKE1_D4(spoke_d1[4]), .SPOKE1_D5(spoke_d1[5]),
        .SPOKE1_STROBE(strobe1),
        .SPOKE2_D0(spoke_d2[0]), .SPOKE2_D1(spoke_d2[1]), .SPOKE2_D2(spoke_d2[2]),
        .SPOKE2_D3(spoke_d2[3]), .SPOKE2_D4(spoke_d2[4]), .SPOKE2_D5(spoke_d2[5]),
        .SPOKE2_STROBE(strobe2),
        .SPOKE3_D0(spoke_d3[0]), .SPOKE3_D1(spoke_d3[1]), .SPOKE3_D2(spoke_d3[2]),
        .SPOKE3_D3(spoke_d3[3]), .SPOKE3_D4(spoke_d3[4]), .SPOKE3_D5(spoke_d3[5]),
        .SPOKE3_STROBE(strobe3),
        .USB_CLKOUT(usb_clkout),
        .USB_D0(usb_d[0]), .USB_D1(usb_d[1]), .USB_D2(usb_d[2]), .USB_D3(usb_d[3]),
        .USB_D4(usb_d[4]), .USB_D5(usb_d[5]), .USB_D6(usb_d[6]), .USB_D7(usb_d[7]),
        .USB_WR_N(usb_wr_n), .USB_TXE_N(usb_txe_n)
    );

    // Captures one byte per active write cycle (usb_wr_n low), same convention as
    // tb_usb_framer.v's own byte-capture block.
    reg [7:0] usb_cap [0:RECORD_LEN-1];
    integer   usb_cap_idx = 0;
    always @(posedge usb_clkout) begin
        if (!usb_wr_n) begin
            if (usb_cap_idx < RECORD_LEN) usb_cap[usb_cap_idx] <= usb_d;
            usb_cap_idx <= usb_cap_idx + 1;
        end
    end

    reg [DATA_WIDTH-1:0] mem_channels [0:6*N_CH-1];
    reg [15:0]            mem_expected [0:6*FRAME_CYCLES-1]; // {strobe,fall[5:0],rise[5:0]}

    initial begin
        $readmemh("../../cluster/vectors/framer_channels.mem", mem_channels);
        $readmemh("../../cluster/vectors/framer_expected.mem", mem_expected);
    end

    // Each spoke is assigned a different frame (0..3) so a cross-spoke
    // channel mixup shows up as a mismatch rather than coincidentally
    // matching.
    localparam integer SPOKE0_FRAME = 0;
    localparam integer SPOKE1_FRAME = 1;
    localparam integer SPOKE2_FRAME = 2;
    localparam integer SPOKE3_FRAME = 3;

    integer errors = 0;
    integer c;
    reg [DATA_WIDTH-1:0] got0 [0:N_CH-1];
    reg [DATA_WIDTH-1:0] got1 [0:N_CH-1];
    reg [DATA_WIDTH-1:0] got2 [0:N_CH-1];
    reg [DATA_WIDTH-1:0] got3 [0:N_CH-1];
    integer valid0_seen = 0, valid1_seen = 0, valid2_seen = 0, valid3_seen = 0;

    always @(posedge spoke_clk) begin
        if (dut.spoke0_valid) begin
            for (c = 0; c < N_CH; c = c + 1) got0[c] = dut.spoke0_ch_data_flat[24*c +: 24];
            valid0_seen = valid0_seen + 1;
        end
        if (dut.spoke1_valid) begin
            for (c = 0; c < N_CH; c = c + 1) got1[c] = dut.spoke1_ch_data_flat[24*c +: 24];
            valid1_seen = valid1_seen + 1;
        end
        if (dut.spoke2_valid) begin
            for (c = 0; c < N_CH; c = c + 1) got2[c] = dut.spoke2_ch_data_flat[24*c +: 24];
            valid2_seen = valid2_seen + 1;
        end
        if (dut.spoke3_valid) begin
            for (c = 0; c < N_CH; c = c + 1) got3[c] = dut.spoke3_ch_data_flat[24*c +: 24];
            valid3_seen = valid3_seen + 1;
        end
    end

    integer cyc;
    reg [15:0] exp0, exp1, exp2, exp3;

    initial begin
        // Bring SPOKES_ALIVE up (models all 4 clusters' wired-AND net
        // resolving high) so reset_seq.v releases FPGA_RESET_N and the
        // sclk-domain reset (rst_sclk) clears -- deframers hold in reset
        // until then, matching real power-up sequencing.
        usb_txe_n = 1'b0; // FIFO always ready
        repeat (8) @(posedge tcxo_clk);
        spokes_alive = 1;

        // Wait for FPGA_RESET_N to release, then a few more SPOKE_CLK edges
        // for the cross-domain synchronizer to settle.
        wait (fpga_reset_n == 1'b1);
        repeat (4) @(posedge spoke_clk);

        for (cyc = 0; cyc < FRAME_CYCLES; cyc = cyc + 1) begin
            exp0 = mem_expected[SPOKE0_FRAME*FRAME_CYCLES + cyc];
            exp1 = mem_expected[SPOKE1_FRAME*FRAME_CYCLES + cyc];
            exp2 = mem_expected[SPOKE2_FRAME*FRAME_CYCLES + cyc];
            exp3 = mem_expected[SPOKE3_FRAME*FRAME_CYCLES + cyc];

            rise0 = exp0[5:0]; strobe0 = exp0[12];
            rise1 = exp1[5:0]; strobe1 = exp1[12];
            rise2 = exp2[5:0]; strobe2 = exp2[12];
            rise3 = exp3[5:0]; strobe3 = exp3[12];
            @(posedge spoke_clk);
            #1;
            fall0 = exp0[11:6];
            fall1 = exp1[11:6];
            fall2 = exp2[11:6];
            fall3 = exp3[11:6];
            @(negedge spoke_clk);
            #1;
        end

        // Let the last busy edge's handoff (BUSY_CYCLES+1) complete.
        repeat (4) @(posedge spoke_clk);

        if (valid0_seen != 1) begin $display("FAIL: spoke0 valid pulses = %0d, expected 1", valid0_seen); errors = errors + 1; end
        if (valid1_seen != 1) begin $display("FAIL: spoke1 valid pulses = %0d, expected 1", valid1_seen); errors = errors + 1; end
        if (valid2_seen != 1) begin $display("FAIL: spoke2 valid pulses = %0d, expected 1", valid2_seen); errors = errors + 1; end
        if (valid3_seen != 1) begin $display("FAIL: spoke3 valid pulses = %0d, expected 1", valid3_seen); errors = errors + 1; end

        for (c = 0; c < N_CH; c = c + 1) begin
            if (got0[c] !== mem_channels[SPOKE0_FRAME*N_CH + c]) begin
                $display("FAIL: spoke0 ch %0d got %h expected %h", c, got0[c], mem_channels[SPOKE0_FRAME*N_CH + c]);
                errors = errors + 1;
            end
            if (got1[c] !== mem_channels[SPOKE1_FRAME*N_CH + c]) begin
                $display("FAIL: spoke1 ch %0d got %h expected %h", c, got1[c], mem_channels[SPOKE1_FRAME*N_CH + c]);
                errors = errors + 1;
            end
            if (got2[c] !== mem_channels[SPOKE2_FRAME*N_CH + c]) begin
                $display("FAIL: spoke2 ch %0d got %h expected %h", c, got2[c], mem_channels[SPOKE2_FRAME*N_CH + c]);
                errors = errors + 1;
            end
            if (got3[c] !== mem_channels[SPOKE3_FRAME*N_CH + c]) begin
                $display("FAIL: spoke3 ch %0d got %h expected %h", c, got3[c], mem_channels[SPOKE3_FRAME*N_CH + c]);
                errors = errors + 1;
            end
        end

        // USB side: wait for spoke0's record (first served, see header comment) to fully
        // drain, then check it against USB_FRAMING.md's format -- sync bytes, spoke_id=0,
        // seq_num=0 (first-ever request), 24 channels' worth of MSB-first payload bytes
        // matching SPOKE0_FRAME's own channel values. usb_cap[] only ever captures its
        // first RECORD_LEN bytes (see the capture block above), so it still holds this
        // first record's bytes even though, by this point in the test, all 4 spokes'
        // records have likely already finished draining (this loop's own settle time
        // dwarfs one record's ~1.3us serialization time) -- hence >= here, not ==: a
        // transient-equality wait would already have been missed.
        wait (usb_cap_idx >= RECORD_LEN);
        @(posedge usb_clkout);
        if (usb_cap[0] !== 8'hA5 || usb_cap[1] !== 8'h5A) begin
            $display("FAIL: USB record sync bytes = %02h %02h, expected a5 5a", usb_cap[0], usb_cap[1]);
            errors = errors + 1;
        end
        if (usb_cap[2] !== 8'd0) begin
            $display("FAIL: USB record spoke_id = %0d, expected 0", usb_cap[2]);
            errors = errors + 1;
        end
        if (usb_cap[3] !== 8'd0) begin
            $display("FAIL: USB record seq_num = %0d, expected 0", usb_cap[3]);
            errors = errors + 1;
        end
        for (c = 0; c < N_CH; c = c + 1) begin
            if ({usb_cap[4+3*c], usb_cap[5+3*c], usb_cap[6+3*c]} !== mem_channels[SPOKE0_FRAME*N_CH + c]) begin
                $display("FAIL: USB record ch %0d got %h expected %h", c,
                         {usb_cap[4+3*c], usb_cap[5+3*c], usb_cap[6+3*c]}, mem_channels[SPOKE0_FRAME*N_CH + c]);
                errors = errors + 1;
            end
        end

        if (errors == 0) begin
            $display("PASS: tb_hub_top, 4 spokes x %0d channels all bit-exact, no cross-spoke mixups, USB record end-to-end OK", N_CH);
            $finish;
        end else begin
            $display("FAIL: tb_hub_top, %0d errors", errors);
            $finish;
        end
    end
endmodule
