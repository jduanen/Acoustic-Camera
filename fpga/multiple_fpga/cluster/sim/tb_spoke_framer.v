`timescale 1ns / 1ps

// Bit-exact check of spoke_framer.v against fpga/cluster/golden/spoke_framer_golden.py's
// spoke_frame(), across several test frames (all-zero, all-max, ramp, random)
// from fpga/cluster/golden/gen_vectors.py's gen_framer_vectors(). Checks both
// DDR halves (rising/falling edge) of spoke_d and spoke_strobe every cycle.
module tb_spoke_framer;
    localparam integer N_FRAMES     = 6;
    localparam integer N_CH         = 24;
    localparam integer DATA_WIDTH   = 24;
    localparam integer FRAME_CYCLES = 64;
    localparam integer BUSY_CYCLES  = N_CH * (DATA_WIDTH/6) / 2; // matches spoke_framer_golden.py

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg [DATA_WIDTH-1:0] mem_channels [0:N_FRAMES*N_CH-1];
    reg [15:0]            mem_expected [0:N_FRAMES*FRAME_CYCLES-1]; // {strobe,fall[5:0],rise[5:0]}

    reg frame_start = 0;
    reg [N_CH*24-1:0] ch_data_flat = 0;
    wire [5:0] spoke_d;
    wire spoke_strobe;

    spoke_framer #(.N_CH(N_CH)) dut (
        .clk(clk), .rst(rst),
        .frame_start(frame_start),
        .ch_data_flat(ch_data_flat),
        .spoke_d(spoke_d),
        .spoke_strobe(spoke_strobe)
    );

    integer errors = 0;
    integer f, c, cyc;
    integer base_ch, base_cyc;
    reg [15:0] exp_packed;
    reg [5:0] exp_rise, exp_fall;
    reg exp_strobe;

    initial begin
        $readmemh("../vectors/framer_channels.mem", mem_channels);
        $readmemh("../vectors/framer_expected.mem", mem_expected);
    end

    initial begin
        rst = 1;
        frame_start = 0;
        ch_data_flat = 0;
        repeat (4) @(negedge clk);
        rst = 0;
        @(negedge clk);

        // Regression check: spoke_strobe must stay low through the gap
        // between reset release and the first real frame_start pulse -- a
        // real bug here once made cyc_r's reset value collide with the
        // legitimate frame_start-cycle value, causing a spurious strobe
        // right after reset (caught by tb_cluster_top.v, not originally by
        // this testbench, since it always drove frame_start immediately).
        for (c = 0; c < 20; c = c + 1) begin
            @(posedge clk);
            #1;
            if (spoke_strobe !== 1'b0) begin
                $display("FAIL: spoke_strobe spuriously high %0d cycles after reset release, before any frame_start", c);
                errors = errors + 1;
            end
        end

        for (f = 0; f < N_FRAMES; f = f + 1) begin
            base_ch = f * N_CH;
            for (c = 0; c < N_CH; c = c + 1)
                ch_data_flat[24*c +: 24] = mem_channels[base_ch + c];

            @(negedge clk);
            frame_start = 1; // held high through this whole cycle's period (see below)

            base_cyc = f * FRAME_CYCLES;
            for (cyc = 0; cyc < FRAME_CYCLES; cyc = cyc + 1) begin
                exp_packed = mem_expected[base_cyc + cyc];
                exp_rise   = exp_packed[5:0];
                exp_fall   = exp_packed[11:6];
                exp_strobe = exp_packed[12];

                @(posedge clk);
                #1;
                if (spoke_d !== exp_rise) begin
                    $display("FAIL: frame %0d cyc %0d rise got %0d expected %0d", f, cyc, spoke_d, exp_rise);
                    errors = errors + 1;
                end
                if (spoke_strobe !== exp_strobe) begin
                    $display("FAIL: frame %0d cyc %0d strobe got %0d expected %0d", f, cyc, spoke_strobe, exp_strobe);
                    errors = errors + 1;
                end

                @(negedge clk);
                #1;
                if (spoke_d !== exp_fall) begin
                    $display("FAIL: frame %0d cyc %0d fall got %0d expected %0d", f, cyc, spoke_d, exp_fall);
                    errors = errors + 1;
                end
                // Clear frame_start only *after* checking this cycle's falling
                // edge -- it must stay asserted for the whole cycle-0 period
                // (both DDR halves), not just be sampled at the posedge, since
                // cyc_eff is a level-sensitive (not edge-registered) mux in
                // spoke_framer.v.
                if (cyc == 0) frame_start = 0;
            end
        end

        // Regression check for the cyc_r bit-width bug (see spoke_framer.v's
        // CYC_W comment): with cyc_r sized too small for BUSY_CYCLES, it
        // would silently wrap back to 0 instead of holding, spuriously
        // re-asserting spoke_strobe on stale latched_flat mid-idle. Run well
        // past BUSY_CYCLES with no further frame_start and confirm
        // spoke_strobe never re-fires.
        for (c = 0; c < 2 * BUSY_CYCLES; c = c + 1) begin
            @(posedge clk);
            #1;
            if (spoke_strobe !== 1'b0) begin
                $display("FAIL: spoke_strobe spuriously high %0d cycles after the last frame's BUSY_CYCLES window, with no new frame_start (cyc_r wraparound?)", c);
                errors = errors + 1;
            end
        end

        if (errors == 0) begin
            $display("PASS: tb_spoke_framer, %0d frames x %0d cycles all bit-exact", N_FRAMES, FRAME_CYCLES);
            $finish;
        end else begin
            $display("FAIL: tb_spoke_framer, %0d errors", errors);
            $finish;
        end
    end
endmodule
