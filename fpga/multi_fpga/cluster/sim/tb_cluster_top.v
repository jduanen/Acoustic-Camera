`timescale 1ns / 1ps

// End-to-end integration test: drives a synthetic multi-tone PDM stream on
// all 24 channels (12 physical PDM_Dxx lines, L/R interleaved per line) into
// cluster_top.v, captures the framed SPOKE_D0-5/SPOKE_STROBE output, deframes
// it per fpga/multi_fpga/cluster/SPOKE_FRAMING.md, and compares against
// fpga/multi_fpga/cluster/golden/gen_pdm_stimulus.py's golden_channel_pipeline() output
// (itself built from the already-independently-verified cic_bitexact() and
// fir_bitexact() golden models) -- bit-exact. This is the "done" criterion
// for the whole cluster pipeline: every module already has its own
// standalone testbench: this one instead exists to catch integration/wiring
// bugs (channel order, bit truncation, port mapping) those can't see. Also
// confirms the LED health indicator (clk_reset.v) reads steady GREEN while
// running -- the reset-transition (YELLOW->GREEN) behavior itself is
// tb_clk_reset.v's job.
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
    wire led0_r, led0_g, led0_b;

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
        .SPOKE_ALIVE(SPOKE_ALIVE),
        .led0_r(led0_r), .led0_g(led0_g), .led0_b(led0_b)
    );

    integer errors = 0;

    // GREEN = led0_r high, led0_g low, led0_b high (active-low, common-anode
    // -- see clk_reset.v). This test never asserts FPGA_RESET_N, so once
    // SPOKE_STROBE is running (checked at both ends of the frame loop below)
    // the LED must already be steady GREEN -- the reset-transition behavior
    // itself (YELLOW->GREEN) is tb_clk_reset.v's job, not duplicated here.
    task check_led_green(input [255:0] label);
        begin
            if (led0_r !== 1'b1 || led0_g !== 1'b0 || led0_b !== 1'b1) begin
                $display("FAIL: %0s: LED not GREEN (led0_r=%b led0_g=%b led0_b=%b)",
                         label, led0_r, led0_g, led0_b);
                errors = errors + 1;
            end
        end
    endtask

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

    // Stimulus: interleave each line's L/R bits onto the shared wire, one new
    // bit per PDM_CLK half-period, matching the real IM72D128 mics' own
    // convention (falling edge -> L, rising edge -> R). Driven off PDM_CLK
    // itself, NOT SPOKE_CLK -- since clk_reset.v now derives PDM_CLK = clk/2
    // (SPOKE_CLK runs 2x the mics' actual rate so cic_decimator_shared can
    // time-multiplex L/R -- see cluster_top.v's header comment), the mics
    // only ever see PDM_CLK's edges, so that's what this testbench (standing
    // in for the mics) must watch, exactly like real hardware would.
    //
    // PDM_CLK now free-runs from t=0, independent of reset (see clk_reset.v)
    // -- exactly like the real mics, which don't know or care about the
    // FPGA's internal reset state either. So this drives line_d reactively,
    // keyed off PDM_CLK's own edges, for the *entire* run, not just starting
    // after some observed reset-release margin: cur_l_idx/cur_r_idx stay
    // pinned at 0 (repeatedly redriving sample 0's L/R values) for as long as
    // dut.rst reads 1, and only start advancing once it reads 0 -- so
    // whichever phase (L or R) happens to be first captured for real after
    // reset releases (this depends on POR_CYCLES' parity relative to
    // PDM_CLK's free-running phase, not fixed) is guaranteed to be sample 0,
    // matching gen_pdm_stimulus.py's golden model. An earlier version of this
    // assumed L is always first and pre-set line_d once before reset released
    // instead of continuously -- on this design that assumption is wrong
    // about half the time depending on POR_CYCLES' parity, and produced a
    // single mismatched channel/phase sample right at reset release that the
    // CIC's integrators then carried forward and amplified for the rest of
    // the run (small in frame 0, badly diverged well before frame 19).
    integer cur_l_idx = 0, cur_r_idx = 0;

    always @(negedge PDM_CLK) begin
        for (li = 0; li < N_LINES; li = li + 1) line_d[li] <= l_bit(li, cur_l_idx);
        if (!dut.rst && cur_l_idx + 1 < N_SAMPLES) cur_l_idx <= cur_l_idx + 1;
    end
    always @(posedge PDM_CLK) begin
        for (li = 0; li < N_LINES; li = li + 1) line_d[li] <= r_bit(li, cur_r_idx);
        if (!dut.rst && cur_r_idx + 1 < N_SAMPLES) cur_r_idx <= cur_r_idx + 1;
    end

    // Capture + deframe: wait for each SPOKE_STROBE, sample both DDR halves
    // for BUSY_CYCLES busy cycles, reconstruct N_CH channel values per
    // fpga/multi_fpga/cluster/SPOKE_FRAMING.md's protocol, compare against golden.
    localparam integer BUSY_CYCLES = N_CH * (DATA_WIDTH/6) / 2; // matches spoke_framer_golden.py

    reg [DATA_WIDTH-1:0] got [0:N_CH-1];
    integer frame_num, cyc, c_idx, chunk_pair;
    reg [5:0] rise_v, fall_v;

    initial begin
        @(posedge SPOKE_STROBE); // first frame's cycle 0 (rising half already valid)
        check_led_green("running, before frame loop");

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
        check_led_green("running, after frame loop");

        if (errors == 0) begin
            $display("PASS: tb_cluster_top, %0d frames x %0d channels all bit-exact, LED steady GREEN throughout", N_FRAMES, N_CH);
            $finish;
        end else begin
            $display("FAIL: tb_cluster_top, %0d errors", errors);
            $finish;
        end
    end
endmodule
