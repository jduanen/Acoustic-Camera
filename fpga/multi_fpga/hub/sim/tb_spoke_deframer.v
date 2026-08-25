`timescale 1ns / 1ps

// Bit-exact check of spoke_deframer.v against fpga/multi_fpga/cluster/golden/
// spoke_framer_golden.py's spoke_frame()/deframe() round trip, reusing the
// SAME vectors tb_spoke_framer.v checks the transmit side against
// (fpga/multi_fpga/cluster/vectors/framer_channels.mem / framer_expected.mem) -- this
// testbench plays the cluster's role (driving spoke_d/spoke_strobe exactly
// as a real spoke_framer would, per the {strobe,fall,rise} packed vectors)
// and checks spoke_deframer.v correctly reconstructs the original channel
// values.
module tb_spoke_deframer;
    localparam integer N_FRAMES     = 6;
    localparam integer N_CH         = 24;
    localparam integer DATA_WIDTH   = 24;
    localparam integer FRAME_CYCLES = 64;

    reg clk = 0;
    reg rst = 1;
    always #5 clk = ~clk;

    reg [DATA_WIDTH-1:0] mem_channels [0:N_FRAMES*N_CH-1];
    reg [15:0]            mem_expected [0:N_FRAMES*FRAME_CYCLES-1]; // {strobe,fall[5:0],rise[5:0]}

    reg [5:0] rise_v = 0, fall_v = 0;
    reg       strobe_v = 0;
    // mirrors spoke_framer.v's own drive convention (a plain level mux on
    // clk), but with a #1 delay -- mirrors tb_pdm_line_demux.v's established
    // precedent of holding a driven value stable through the *whole* half
    // period and only changing it 1ns after the edge that starts that half,
    // never *at* the edge itself. Without the delay, spoke_d's mux output
    // switches at the exact same simulation instant as clk's edge -- the
    // same instant the DUT's own posedge/negedge-triggered captures read
    // spoke_d -- an LRM-unspecified-order race between the continuous
    // assignment's recompute and the always block's same-edge RHS read
    // (unlike tb_pdm_line_demux.v's plain reg, this signal's value is
    // itself clk-level-dependent, so the delay has to live on the mux
    // rather than on when rise_v/fall_v are set).
    wire [5:0] spoke_d;
    assign #1 spoke_d = clk ? rise_v : fall_v;

    wire [24*24-1:0] ch_data_flat;
    wire valid;

    spoke_deframer dut (
        .clk(clk), .rst(rst),
        .spoke_d(spoke_d),
        .spoke_strobe(strobe_v),
        .ch_data_flat(ch_data_flat),
        .valid(valid)
    );

    integer errors = 0;
    integer f, c, cyc;
    integer base_ch, base_cyc;
    reg [15:0] exp_packed;
    integer frames_checked = 0;

    initial begin
        $readmemh("../../cluster/vectors/framer_channels.mem", mem_channels);
        $readmemh("../../cluster/vectors/framer_expected.mem", mem_expected);
    end

    // Capture ch_data_flat/valid whenever it pulses, compare against the
    // frame currently being driven.
    reg [DATA_WIDTH-1:0] got [0:N_CH-1];
    always @(posedge clk) begin
        if (!rst && valid) begin
            for (c = 0; c < N_CH; c = c + 1)
                got[c] = ch_data_flat[24*c +: 24];
            frames_checked = frames_checked + 1;
        end
    end

    initial begin
        rst = 1;
        strobe_v = 0;
        rise_v = 0;
        fall_v = 0;
        repeat (4) @(negedge clk);
        rst = 0;
        @(negedge clk);

        for (f = 0; f < N_FRAMES; f = f + 1) begin
            base_cyc = f * FRAME_CYCLES;
            for (cyc = 0; cyc < FRAME_CYCLES; cyc = cyc + 1) begin
                exp_packed = mem_expected[base_cyc + cyc];
                // rise_v/strobe_v are set while clk is still low (so the mux
                // -- which only looks at rise_v when clk=1 -- isn't disturbed
                // mid-low-phase); fall_v is set only after the following
                // posedge has settled (so the mux isn't disturbed
                // mid-high-phase either). This keeps spoke_d showing a given
                // cycle's rise value for that whole cycle's high phase and
                // its fall value for that whole cycle's low phase, which the
                // #1 delay on spoke_d's own mux (see its declaration above)
                // then additionally protects at the exact edge instants
                // themselves.
                rise_v   = exp_packed[5:0];
                strobe_v = exp_packed[12];
                @(posedge clk);
                #1;
                fall_v = exp_packed[11:6];
                @(negedge clk);
                #1;
            end

            // by now spoke_deframer.v should have pulsed valid with this
            // frame's reconstructed channels
            base_ch = f * N_CH;
            if (frames_checked != f + 1) begin
                $display("FAIL: frame %0d never produced a valid pulse", f);
                errors = errors + 1;
            end else begin
                for (c = 0; c < N_CH; c = c + 1) begin
                    if (got[c] !== mem_channels[base_ch + c]) begin
                        $display("FAIL: frame %0d ch %0d got %h expected %h",
                                 f, c, got[c], mem_channels[base_ch + c]);
                        errors = errors + 1;
                    end
                end
            end
        end

        if (errors == 0) begin
            $display("PASS: tb_spoke_deframer, %0d frames x %0d channels all bit-exact", N_FRAMES, N_CH);
            $finish;
        end else begin
            $display("FAIL: tb_spoke_deframer, %0d errors", errors);
            $finish;
        end
    end
endmodule
