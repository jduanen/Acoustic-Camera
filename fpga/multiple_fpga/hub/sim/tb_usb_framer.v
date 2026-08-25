`timescale 1ns / 1ps

// Checks usb_framer.v against fpga/hub/golden/usb_framer_golden.py's pack_record() and
// against the arbitration/drop behavior documented in fpga/USB_FRAMING.md.
//
// Drives sclk and usb_clk at their real, genuinely different, non-commensurate frequencies
// (6.144 MHz / 60 MHz) -- proving the CDC survives real asynchrony, not just same-domain
// timing coincidences a simpler testbench clock choice might accidentally hide.
//
// Phase A (bit-exact): one spoke's frame at a time, sequentially, against the checked-in
// fpga/hub/vectors/usb_channels.mem / usb_expected.mem (8 records, spoke_id cycling
// 0,1,2,3,0,1,2,3 -- see gen_usb_vectors.py). Toggles usb_txe_n mid-record on record 3 to
// exercise backpressure stalling; the byte-capture logic below only samples on active write
// cycles, so a correct pass here also proves stalling doesn't corrupt or skip bytes.
//
// Phase B (structural, not vector-based -- Phase A already proves byte-level framing is
// correct, so this only needs to prove the two behaviors that require multiple spokes
// contending at once): (1) simultaneous requests from two spokes are served in ascending
// rotating-priority order, and (2) a second request for the same spoke arriving before the
// first has been served overwrites it (drop-stale, no queuing) rather than sending both.
module tb_usb_framer;
    localparam integer N_CH        = 24;
    localparam integer DATA_WIDTH  = 24;
    localparam integer RECORD_LEN  = 76;
    localparam integer N_RECORDS   = 8;

    reg sclk = 0;
    always #(162.76/2) sclk = ~sclk; // 6.144 MHz, matches hub_top.v's sclk

    reg usb_clk = 0;
    always #(16.667/2) usb_clk = ~usb_clk; // 60 MHz, FT232H's CLKOUT

    reg rst_sclk = 1;
    reg rst_usb  = 1;

    reg [24*24-1:0] spoke0_ch_data_flat = 0, spoke1_ch_data_flat = 0,
                     spoke2_ch_data_flat = 0, spoke3_ch_data_flat = 0;
    reg spoke0_valid = 0, spoke1_valid = 0, spoke2_valid = 0, spoke3_valid = 0;

    wire [7:0] usb_d;
    wire       usb_wr_n;
    reg        usb_txe_n = 1'b1;

    usb_framer dut (
        .sclk(sclk), .rst_sclk(rst_sclk),
        .usb_clk(usb_clk), .rst_usb(rst_usb),
        .spoke0_ch_data_flat(spoke0_ch_data_flat), .spoke1_ch_data_flat(spoke1_ch_data_flat),
        .spoke2_ch_data_flat(spoke2_ch_data_flat), .spoke3_ch_data_flat(spoke3_ch_data_flat),
        .spoke0_valid(spoke0_valid), .spoke1_valid(spoke1_valid),
        .spoke2_valid(spoke2_valid), .spoke3_valid(spoke3_valid),
        .usb_d(usb_d), .usb_wr_n(usb_wr_n), .usb_txe_n(usb_txe_n)
    );

    reg [DATA_WIDTH-1:0] mem_channels [0:N_RECORDS*N_CH-1];
    reg [7:0]            mem_expected [0:N_RECORDS*RECORD_LEN-1];

    initial begin
        $readmemh("../vectors/usb_channels.mem", mem_channels);
        $readmemh("../vectors/usb_expected.mem", mem_expected);
    end

    // Captures one byte per active write cycle (usb_wr_n low), same-edge sampling
    // convention as tb_hub_top.v's `always @(posedge spoke_clk) if (dut.spokeN_valid)`.
    reg [7:0] cap_bytes [0:RECORD_LEN-1];
    integer   cap_idx = 0;
    always @(posedge usb_clk) begin
        if (!usb_wr_n) begin
            if (cap_idx < RECORD_LEN) cap_bytes[cap_idx] <= usb_d;
            cap_idx <= cap_idx + 1;
        end
    end

    integer errors = 0;
    integer i, c, k;
    integer base_ch, base_rec;
    reg [24*24-1:0] rec_flat;

    // automatic: Phase B.1 calls this twice concurrently (fork/join) -- a non-automatic
    // (static) task would have both calls share one copy of `spoke`/`data`, corrupting
    // whichever call's inputs execute second.
    //
    // valid is set/cleared only during sclk's stable low phase (@(negedge sclk)), never at
    // the same instant as the posedge the DUT's own always @(posedge sclk) block samples it
    // on -- same testbench-vs-DUT same-edge race avoided by tb_spoke_deframer.v/tb_hub_top.v
    // elsewhere in this project (see their own header comments).
    task automatic drive_spoke(input integer spoke, input [24*24-1:0] data);
        begin
            case (spoke)
                0: spoke0_ch_data_flat = data;
                1: spoke1_ch_data_flat = data;
                2: spoke2_ch_data_flat = data;
                3: spoke3_ch_data_flat = data;
            endcase
            @(negedge sclk);
            case (spoke)
                0: spoke0_valid = 1'b1;
                1: spoke1_valid = 1'b1;
                2: spoke2_valid = 1'b1;
                3: spoke3_valid = 1'b1;
            endcase
            @(posedge sclk); // DUT samples valid=1 here
            @(negedge sclk);
            spoke0_valid = 1'b0; spoke1_valid = 1'b0; spoke2_valid = 1'b0; spoke3_valid = 1'b0;
        end
    endtask

    initial begin
        spoke0_valid = 0; spoke1_valid = 0; spoke2_valid = 0; spoke3_valid = 0;
        usb_txe_n = 1'b1;
        repeat (4) @(posedge sclk);
        repeat (4) @(posedge usb_clk);
        rst_sclk = 0;
        rst_usb  = 0;
        repeat (4) @(posedge usb_clk);

        // --- Phase A: sequential, bit-exact vs golden vectors ---
        for (i = 0; i < N_RECORDS; i = i + 1) begin
            base_ch = i * N_CH;
            for (c = 0; c < N_CH; c = c + 1) rec_flat[24*c +: 24] = mem_channels[base_ch + c];

            cap_idx = 0;
            usb_txe_n = 1'b0;
            drive_spoke(i % 4, rec_flat);

            if (i == 3) begin
                // Exercise TXE# backpressure mid-record.
                repeat (40) @(posedge usb_clk);
                usb_txe_n = 1'b1;
                repeat (10) @(posedge usb_clk);
                usb_txe_n = 1'b0;
            end

            wait (cap_idx == RECORD_LEN);
            @(posedge usb_clk);

            base_rec = i * RECORD_LEN;
            for (k = 0; k < RECORD_LEN; k = k + 1) begin
                if (cap_bytes[k] !== mem_expected[base_rec + k]) begin
                    $display("FAIL: record %0d byte %0d got %02h expected %02h",
                             i, k, cap_bytes[k], mem_expected[base_rec + k]);
                    errors = errors + 1;
                end
            end
            repeat (4) @(posedge usb_clk);
        end
        // Phase A's 8 records (spoke_id 0,1,2,3,0,1,2,3) leave last_served == 3, so Phase B's
        // rotating priority starts back at spoke 0 -- matches the expected order below.

        // --- Phase B.1: simultaneous requests served in ascending rotating-priority order ---
        usb_txe_n = 1'b0;
        cap_idx = 0;
        fork
            drive_spoke(0, {N_CH{24'hAAAAAA}});
            drive_spoke(2, {N_CH{24'h555555}});
        join
        wait (cap_idx == RECORD_LEN);
        @(posedge usb_clk);
        if (cap_bytes[2] !== 8'd0) begin
            $display("FAIL: Phase B.1 first record spoke_id = %0d, expected 0", cap_bytes[2]);
            errors = errors + 1;
        end
        cap_idx = 0;
        wait (cap_idx == RECORD_LEN);
        @(posedge usb_clk);
        if (cap_bytes[2] !== 8'd2) begin
            $display("FAIL: Phase B.1 second record spoke_id = %0d, expected 2", cap_bytes[2]);
            errors = errors + 1;
        end
        repeat (4) @(posedge usb_clk);

        // --- Phase B.2: a second request for a spoke still waiting to be picked up
        // overwrites the first (drop-stale), rather than both being queued. Needs another
        // spoke's record to be occupying the arbiter (tx_busy) throughout, stalled on
        // usb_txe_n -- otherwise the very first spoke1 request would be picked up (moved out
        // of req_pending, into tx_shift) before the second one can ever arrive, since the
        // CDC+arbiter pickup latency (a handful of usb_clk cycles) is far shorter than the
        // testbench can drive two separate requests apart (over a sclk cycle each).
        usb_txe_n = 1'b1; // hold the FIFO busy so nothing ever drains
        drive_spoke(0, {N_CH{24'h999999}});   // occupies tx_busy, stuck (TXE# stays high)
        repeat (20) @(posedge usb_clk);        // let spoke0's request be picked up
        drive_spoke(1, {N_CH{24'h111111}});   // 1st spoke1 request -- queues in req_pending
        repeat (20) @(posedge usb_clk);        // still stuck behind spoke0; not yet servable
        drive_spoke(1, {N_CH{24'h222222}});   // 2nd spoke1 request -- overwrites the 1st

        cap_idx = 0;
        usb_txe_n = 1'b0; // release -- spoke0's record drains first, then spoke1's
        wait (cap_idx == RECORD_LEN);
        @(posedge usb_clk);
        if (cap_bytes[2] !== 8'd0) begin
            $display("FAIL: Phase B.2 first record spoke_id = %0d, expected 0", cap_bytes[2]);
            errors = errors + 1;
        end
        cap_idx = 0;
        wait (cap_idx == RECORD_LEN);
        @(posedge usb_clk);
        if (cap_bytes[2] !== 8'd1) begin
            $display("FAIL: Phase B.2 second record spoke_id = %0d, expected 1", cap_bytes[2]);
            errors = errors + 1;
        end
        if (cap_bytes[4] !== 8'h22) begin
            $display("FAIL: Phase B.2 got stale sample (byte4=%02h), expected newer 0x22..",
                     cap_bytes[4]);
            errors = errors + 1;
        end
        repeat (100) @(posedge usb_clk); // nothing more should ever be sent
        if (cap_idx != RECORD_LEN) begin
            $display("FAIL: Phase B.2 sent a third (queued, stale) record -- expected exactly two total, cap_idx=%0d",
                     cap_idx);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("PASS: tb_usb_framer, %0d records bit-exact, arbitration order and drop-stale both correct",
                      N_RECORDS);
            $finish;
        end else begin
            $display("FAIL: tb_usb_framer, %0d errors", errors);
            $finish;
        end
    end
endmodule
