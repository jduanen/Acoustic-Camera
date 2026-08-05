`timescale 1ns / 1ps

// End-to-end integration test: drives a synthetic multi-tone PDM stream on
// all 24 channels (12 physical PDM_Dxx lines, L/R interleaved per line) into
// cluster_top.v, captures the framed SPOKE_D0-5/SPOKE_STROBE output, deframes
// it per fpga/cluster/SPOKE_FRAMING.md, and compares against
// fpga/cluster/golden/gen_pdm_stimulus.py's golden_channel_pipeline() output
// (itself built from the already-independently-verified cic_bitexact() and
// fir_bitexact() golden models) -- bit-exact. This is the "done" criterion
// for the whole cluster pipeline: every module already has its own
// standalone testbench: this one instead exists to catch integration/wiring
// bugs (channel order, bit truncation, port mapping) those can't see.
//
// Vectors from gen_vectors.py's gen_cluster_top_vectors():
//   top_pdm_bits.mem  -- N_CH * N_SAMPLES lines, channel-major, 1 bit/line
//   top_expected.mem  -- N_FRAMES * N_CH lines, frame-major, 24-bit hex PCM
// These localparams must stay in sync with gen_vectors.py's TOP_N_FRAMES /
// gen_pdm_stimulus.py's gen_multitone_pdm() defaults.
module tb_cluster_top;
    localparam integer N_CH       = 24;
    localparam integer N_LINES    = 12;
    localparam integer N_FRAMES   = 20;
    localparam integer R          = 64;
    localparam integer N_SAMPLES  = N_FRAMES * R; // 1280
    localparam integer DATA_WIDTH = 24;

    reg SPOKE_CLK = 0;
    always #5 SPOKE_CLK = ~SPOKE_CLK;

    // Tied high (never asserted) for this test -- FPGA_RESET_N/SPOKE_ALIVE's
    // own handshake behavior is covered by tb_clk_reset.v; this integration
    // test only needs to confirm the existing data path is unaffected by the
    // new pass-through ports.
    reg FPGA_RESET_N = 1'b1;

    reg [N_LINES-1:0] line_d;
    wire PDM_CLK;
    wire [5:0] spoke_d_bus;
    wire SPOKE_STROBE;
    wire SPOKE_ALIVE;

    cluster_top dut (
        .SPOKE_CLK(SPOKE_CLK),
        .FPGA_RESET_N(FPGA_RESET_N),
        .PDM_CLK(PDM_CLK),
        .PDM_D00(line_d[0]),  .PDM_D01(line_d[1]),  .PDM_D02(line_d[2]),  .PDM_D03(line_d[3]),
        .PDM_D04(line_d[4]),  .PDM_D05(line_d[5]),  .PDM_D06(line_d[6]),  .PDM_D07(line_d[7]),
        .PDM_D08(line_d[8]),  .PDM_D09(line_d[9]),  .PDM_D10(line_d[10]), .PDM_D11(line_d[11]),
        .SPOKE_D0(spoke_d_bus[0]), .SPOKE_D1(spoke_d_bus[1]), .SPOKE_D2(spoke_d_bus[2]),
        .SPOKE_D3(spoke_d_bus[3]), .SPOKE_D4(spoke_d_bus[4]), .SPOKE_D5(spoke_d_bus[5]),
        .SPOKE_STROBE(SPOKE_STROBE),
        .SPOKE_ALIVE(SPOKE_ALIVE)
    );

    // channel c's PDM bits: pdm_mem[c*N_SAMPLES + sample_idx]
    reg pdm_mem [0:N_CH*N_SAMPLES-1];
    // frame f, channel c's expected PCM: exp_mem[f*N_CH + c]
    reg [DATA_WIDTH-1:0] exp_mem [0:N_FRAMES*N_CH-1];

    initial begin
        $readmemb("../vectors/top_pdm_bits.mem", pdm_mem);
        $readmemh("../vectors/top_expected.mem", exp_mem);
    end

    function l_bit(input integer line, input integer idx);
        l_bit = pdm_mem[(2*line) * N_SAMPLES + idx]; // channel 2*line = L
    endfunction
    function r_bit(input integer line, input integer idx);
        r_bit = pdm_mem[(2*line + 1) * N_SAMPLES + idx]; // channel 2*line+1 = R
    endfunction

    integer li;
    integer sample_idx;

    // Stimulus: interleave each line's L/R bits onto the shared wire, one new
    // bit per half-cycle, matching pdm_line_demux.v's capture convention
    // (negedge -> L, posedge -> R) -- see tb_pdm_line_demux.v for the same
    // pattern on a single line.
    //
    // line_d is set to sample 0's L values immediately, then dut.rst
    // (hierarchical probe -- testbench-only reference into cluster_top's
    // internal reset, not part of its port list) is watched directly rather
    // than guessing a fixed settle-cycle count. rst updates on a *posedge*
    // (clk_reset.v's POR counter); pdm_line_demux's ch_l capture is
    // *negedge*-triggered, a different edge type, so the very first negedge
    // after rst's transition already sees the settled rst=0 -- no extra
    // margin cycles needed (an earlier version of this held sample 0 for
    // several extra negedges "for safety," which instead made the CIC
    // accumulate several redundant repeats of sample 0 before ever reaching
    // sample 1, corrupting its entire first 64-sample window).
    initial begin
        for (li = 0; li < N_LINES; li = li + 1) line_d[li] = l_bit(li, 0);
        @(negedge dut.rst);
        @(negedge SPOKE_CLK); // captures L[0] for real -- no extra margin

        for (sample_idx = 0; sample_idx < N_SAMPLES; sample_idx = sample_idx + 1) begin
            for (li = 0; li < N_LINES; li = li + 1) line_d[li] = r_bit(li, sample_idx);
            @(posedge SPOKE_CLK); // captures R[sample_idx]

            if (sample_idx + 1 < N_SAMPLES) begin
                for (li = 0; li < N_LINES; li = li + 1) line_d[li] = l_bit(li, sample_idx + 1);
            end
            @(negedge SPOKE_CLK); // captures L[sample_idx+1]
        end
    end

    // Capture + deframe: wait for each SPOKE_STROBE, sample both DDR halves
    // for 48 busy cycles, reconstruct 24 channel values per
    // fpga/cluster/SPOKE_FRAMING.md, compare against golden.
    localparam integer BUSY_CYCLES = 48;

    reg [DATA_WIDTH-1:0] got [0:N_CH-1];
    integer frame_num, cyc, c_idx, chunk_pair;
    reg [5:0] rise_v, fall_v;
    integer errors = 0;

    initial begin
        @(posedge SPOKE_STROBE); // first frame's cycle 0 (rising half already valid)

        for (frame_num = 0; frame_num < N_FRAMES; frame_num = frame_num + 1) begin
            if (frame_num != 0) @(posedge SPOKE_STROBE);

            for (cyc = 0; cyc < BUSY_CYCLES; cyc = cyc + 1) begin
                if (cyc != 0) @(posedge SPOKE_CLK);
                #1;
                rise_v = spoke_d_bus;
                @(negedge SPOKE_CLK);
                #1;
                fall_v = spoke_d_bus;

                c_idx      = cyc / 2;
                chunk_pair = cyc % 2;
                if (chunk_pair == 0) begin
                    got[c_idx][23:18] = rise_v;
                    got[c_idx][17:12] = fall_v;
                end else begin
                    got[c_idx][11:6] = rise_v;
                    got[c_idx][5:0]  = fall_v;
                end
            end

            for (c_idx = 0; c_idx < N_CH; c_idx = c_idx + 1) begin
                if (got[c_idx] !== exp_mem[frame_num*N_CH + c_idx]) begin
                    $display("FAIL: frame %0d ch %0d got %h expected %h",
                             frame_num, c_idx, got[c_idx], exp_mem[frame_num*N_CH + c_idx]);
                    errors = errors + 1;
                end
            end
        end

        if (errors == 0) begin
            $display("PASS: tb_cluster_top, %0d frames x %0d channels all bit-exact", N_FRAMES, N_CH);
            $finish;
        end else begin
            $display("FAIL: tb_cluster_top, %0d errors", errors);
            $finish;
        end
    end
endmodule
