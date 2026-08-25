`timescale 1ns / 1ps

// End-to-end integration test: drives a synthetic multi-tone PDM stream on
// all 96 channels (48 physical PDM_Dxx lines, L/R interleaved per line,
// same convention as fpga/multi_fpga/cluster/sim/tb_cluster_top.v) into
// single_fpga_pipeline_top.v, captures the tx_axis_* byte stream it produces
// (real pdm_line_demux.v/cic_decimator.v/fir_compensator.v -> real
// gbe_packetizer.v, no synthetic fir_valid_in/fir_data_in stimulus like the
// standalone gbe_packetizer.v check done earlier), and compares against
// fpga/single_fpga/golden/gen_vectors.py's expected packet bytes (built from
// fpga/multi_fpga/cluster/golden's already-independently-verified CIC/FIR golden models
// + gbe_packetizer_golden.py) -- bit-exact. This is the integration check
// flagged as still needed after the standalone FSM check: confirms the
// "all 96 fir_valid pulses land on the same cycle" assumption
// gbe_packetizer.v's header comment documents, since here fir_valid_in comes
// from 96 *real* fir_compensator instances, not a testbench-forced constant.
//
// Two independent, deliberately mismatched clock periods for clk (pipeline)
// and tx_clk (MAC side) -- genuinely exercises the ping-pong CDC in
// gbe_packetizer.v, not same-domain timing.
//
// Vectors from fpga/single_fpga/golden/gen_vectors.py's gen_gbe_pipeline_vectors():
//   pipeline_pdm_bits.mem        -- N_CH * N_SAMPLES lines, channel-major, 1 bit/line
//   pipeline_expected_packets.mem -- N_PACKETS * TOTAL_PKT_BYTES lines, one hex byte/line
// These localparams must stay in sync with gen_vectors.py's N_FRAMES.
module tb_gbe_pipeline;
    localparam integer N_CH             = 96;
    localparam integer N_LINES          = 48;
    localparam integer N_FRAMES         = 17;
    localparam integer R                = 64;
    localparam integer N_SAMPLES        = N_FRAMES * R; // 1088
    localparam integer FRAMES_PER_PKT   = 5;
    localparam integer TOTAL_PKT_BYTES  = 1494;
    localparam integer N_PACKETS        = N_FRAMES / FRAMES_PER_PKT; // 3

    // clk's period matters here, not just its existence: gbe_packetizer.v's
    // ping-pong CDC assumes tx_clk drains one packet (TOTAL_PKT_BYTES cycles)
    // well before clk produces the next batch (FRAMES_PER_PKT*R cycles) --
    // see GBE_FRAMING.md's ~8.7x margin note. clk is set to the real PDM
    // rate this design targets (~3.072 MHz) and tx_clk to the real 125 MHz
    // GMII rate, reproducing that same ~8.7x margin here -- an earlier
    // version of this testbench used a much faster, arbitrarily-chosen clk
    // (10ns) alongside tx_clk, inverting the real ratio (production faster
    // than consumption) and corrupting the last batch's tail channels via a
    // genuine write-while-still-draining overrun. Not an RTL bug -- a
    // testbench clock-choice bug once already caught by comparing against
    // the golden model, worth keeping this comment so it isn't reintroduced.
    reg clk = 0;
    always #163 clk = ~clk; // ~326ns period, ~3.07 MHz -- pipeline/PDM domain

    reg tx_clk = 0;
    always #3.973 tx_clk = ~tx_clk; // ~7.95ns period, ~125 MHz GMII rate -- MAC domain

    reg rst    = 1'b1;
    reg tx_rst = 1'b1;
    reg tx_ready = 1'b1; // no backpressure -- functional check only

    reg [N_LINES-1:0] pdm_d;

    wire [7:0] tdata;
    wire       tvalid, tlast, tuser;

    single_fpga_pipeline_top #(
        .DST_MAC(48'hAABBCCDDEEFF), .SRC_MAC(48'h001122334455),
        .SRC_IP(32'hC0A80002),      .DST_IP(32'hC0A80001),
        .SRC_PORT(16'd50000),       .DST_PORT(16'd50000)
    ) dut (
        .clk(clk), .rst(rst), .pdm_d(pdm_d),
        .tx_clk(tx_clk), .tx_rst(tx_rst),
        .tx_axis_tdata(tdata), .tx_axis_tvalid(tvalid), .tx_axis_tready(tx_ready),
        .tx_axis_tlast(tlast), .tx_axis_tuser(tuser)
    );

    initial begin
        // Must span several full clk periods (326ns) so clk's own posedge
        // always block actually samples rst=1 at least once -- an earlier
        // version held rst for only 103ns, shorter than one clk period,
        // so clk's first posedge (t=163ns) already saw rst=0 and every
        // clk-domain register (frame_idx, sample_counter, cic_decimator's
        // samp_cnt, ...) stayed at simulation's default X forever. Purely a
        // testbench bug (reset pulse too short for the realistic, slow PDM
        // clock rate chosen above), not an RTL bug -- caught by X's showing
        // up on internal signals, not by the golden-model comparison
        // (which can't distinguish "stuck at X" from "wrong value" any
        // better than any other mismatch), worth this comment so a future
        // clk-period change doesn't silently reintroduce it.
        #1000 rst    = 1'b0; // off the 326ns clk grid -- no same-timestep race with an edge
        #1000 tx_rst = 1'b0; // off the 8ns tx_clk grid too
    end

    // channel c's PDM bits: pdm_mem[c*N_SAMPLES + sample_idx]
    reg pdm_mem [0:N_CH*N_SAMPLES-1];
    initial $readmemb("../vectors/pipeline_pdm_bits.mem", pdm_mem);

    function l_bit(input integer line, input integer idx);
        l_bit = pdm_mem[(2*line) * N_SAMPLES + idx]; // channel 2*line = L
    endfunction
    function r_bit(input integer line, input integer idx);
        r_bit = pdm_mem[(2*line + 1) * N_SAMPLES + idx]; // channel 2*line+1 = R
    endfunction

    // Stimulus driving: pdm_line_demux.v captures pdm_d on BOTH edges of clk
    // (DDR) -- ch_l on negedge, ch_r on posedge -- so pdm_d must already be
    // SETTLED to the value a given edge should capture *before* that edge
    // arrives, i.e. driven during the *preceding* half-period, not the one
    // that starts at the capturing edge itself. Concretely: the bit ch_l's
    // upcoming negedge should capture must be driven right after the *prior*
    // posedge (so it's stable through the whole posedge-to-negedge half-
    // period); the bit ch_r's upcoming posedge should capture must be driven
    // right after the *prior* negedge. An earlier version of this testbench
    // had this backwards -- driving l_bit right after negedge (so it became
    // whatever the *following posedge*, i.e. ch_r, captured) and r_bit right
    // after posedge (captured by the following negedge, ch_l) -- a
    // consistent L/R swap with a half-sample lag, independent of whether the
    // update used blocking or nonblocking assignment (tried both; identical,
    // still-wrong result either way, which is what exposed that the timing-
    // style guess was never the actual issue). Caught by comparing real
    // captured RTL values against the golden model across many (source
    // channel, shift) hypotheses until the pattern -- channel 2k's real data
    // exactly matching pdm_bits[2k+1] shifted, not pdm_bits[2k] -- pointed at
    // the swap directly; not something inspection alone would have found.
    // tb_pdm_line_demux.v (this module's own unit testbench, already
    // validated) uses this same "drive right after the edge whose value
    // you're *not* about to capture" pattern, confirming it's correct.
    //
    // Sample-0 alignment across reset: pdm_d is driven from index 0 for as
    // long as rst is asserted (the index only advances once !rst), so
    // whichever edge is first captured for real after release is guaranteed
    // to be sample 0 -- same reasoning as tb_cluster_top.v's own reactive
    // driving, simpler here since rst is this testbench's own directly-
    // driven signal, not routed through an internal POR sequencer.
    integer li;
    integer cur_l_idx = 0, cur_r_idx = 0;

    always @(posedge clk) begin
        // prepares the bit the *next negedge* (ch_l) will capture
        for (li = 0; li < N_LINES; li = li + 1) pdm_d[li] <= l_bit(li, cur_l_idx);
        if (!rst && cur_l_idx + 1 < N_SAMPLES) cur_l_idx <= cur_l_idx + 1;
    end
    always @(negedge clk) begin
        // prepares the bit the *next posedge* (ch_r) will capture
        for (li = 0; li < N_LINES; li = li + 1) pdm_d[li] <= r_bit(li, cur_r_idx);
        if (!rst && cur_r_idx + 1 < N_SAMPLES) cur_r_idx <= cur_r_idx + 1;
    end

    // frame f, byte b of packet f/TOTAL_PKT_BYTES: exp_mem[f*TOTAL_PKT_BYTES + b]
    reg [7:0] exp_mem [0:N_PACKETS*TOTAL_PKT_BYTES-1];
    initial $readmemh("../vectors/pipeline_expected_packets.mem", exp_mem);

    integer errors = 0;
    integer pkt_byte_idx = 0;

    always @(posedge tx_clk) begin
        if (tvalid && tx_ready) begin
            if (pkt_byte_idx < N_PACKETS*TOTAL_PKT_BYTES) begin
                if (tdata !== exp_mem[pkt_byte_idx]) begin
                    $display("FAIL: byte %0d got %02h expected %02h",
                             pkt_byte_idx, tdata, exp_mem[pkt_byte_idx]);
                    errors = errors + 1;
                end
                if (((pkt_byte_idx % TOTAL_PKT_BYTES) == TOTAL_PKT_BYTES-1) !== tlast) begin
                    $display("FAIL: tlast mismatch at byte %0d (tlast=%b, expected %0d)",
                             pkt_byte_idx, tlast, (pkt_byte_idx % TOTAL_PKT_BYTES) == TOTAL_PKT_BYTES-1);
                    errors = errors + 1;
                end
                if (tuser !== 1'b0) begin
                    $display("FAIL: tuser asserted at byte %0d (expected always 0)", pkt_byte_idx);
                    errors = errors + 1;
                end
            end else begin
                $display("FAIL: unexpected extra byte %0d beyond %0d expected",
                         pkt_byte_idx, N_PACKETS*TOTAL_PKT_BYTES);
                errors = errors + 1;
            end

            pkt_byte_idx = pkt_byte_idx + 1;

            if (pkt_byte_idx == N_PACKETS*TOTAL_PKT_BYTES) begin
                if (errors == 0) begin
                    $display("PASS: tb_gbe_pipeline, %0d packets x %0d bytes all bit-exact",
                             N_PACKETS, TOTAL_PKT_BYTES);
                end else begin
                    $display("FAIL: tb_gbe_pipeline, %0d errors", errors);
                end
                $finish;
            end
        end
    end

    // Safety net only -- normal completion is the $finish above, triggered
    // the instant the last expected byte is captured.
    initial begin
        #5_000_000;
        $display("FAIL: tb_gbe_pipeline timed out, only %0d/%0d bytes received",
                  pkt_byte_idx, N_PACKETS*TOTAL_PKT_BYTES);
        $finish;
    end
endmodule
