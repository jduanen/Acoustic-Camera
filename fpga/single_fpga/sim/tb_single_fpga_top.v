`timescale 1ns / 1ps

// End-to-end check of the board top from its pins: TCXO_CLK in, PDM_CLK out
// to 48 modelled mic pairs, their DATA back in through pdm_sampler.v, and the
// real 96-channel CIC+FIR (single_fpga_pipeline_top.v) on the gated pipe_clk.
// Each channel's FIR outputs are compared bit-exact against
// golden/gen_vectors.py's gen_board_top_vectors() (board_top_expected_
// channels.mem). The packetizer's MAC side is tied off in the board top, so
// the comparison taps the pipeline's fir_valid/fir_out_flat hierarchically.
//
// Mic model as in tb_pdm_sampler.v, at the slow datasheet/board corner: each
// mic drives TDD after its PDM_CLK edge (L after the fall, R after the rise),
// releases THZ after the opposite edge, and an undriven line reads X. The
// streams are pipeline_pdm_bits.mem's (channel 2n = line n's L, 2n+1 = R),
// the same stimulus tb_gbe_pipeline.v uses.
//
// Alignment (TCXO edges e = 0,1,2..., pdm_div = e mod 4 just before edge e):
// POR releases after edge 16. The mics start streaming at the first PDM_CLK
// fall after that (launched at edge 19), and the R bit after it at edge 21.
// pdm_sampler captures those at edges 21 and 23 and presents them at edge 24;
// pipe_clk takes them at edge 26. pipe_clk's earlier post-reset edges, 18 and
// 22, see the sampler's reset-cleared outputs and the pair requested before
// streaming, both zero -- so every channel's CIC input is two zeros followed
// by the stream, the `lead = 2` in gen_board_top_vectors().
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

    single_fpga_top dut (
        .TCXO_CLK(tcxo), .PDM_CLK(pdm_clk),
        .LED_R(led_r), .LED_G(led_g), .LED_B(led_b),
        .DATA(data)
    );

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
        if (checked == N_FRAMES * N_CH) begin
            if (errors == 0)
                $display("PASS: tb_single_fpga_top, %0d channels x %0d frames bit-exact", N_CH, N_FRAMES);
            else
                $display("FAIL: tb_single_fpga_top, %0d of %0d outputs wrong", errors, checked);
            $finish;
        end
    end

    initial begin
        #500_000;
        $display("FAIL: tb_single_fpga_top timed out, %0d/%0d outputs checked", checked, N_FRAMES * N_CH);
        $finish;
    end
endmodule
