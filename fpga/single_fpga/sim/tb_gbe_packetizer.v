`timescale 1ns / 1ps

// Standalone unit test of gbe_packetizer.v alone -- synthetic fir_valid_in/
// fir_data_in stimulus (deterministic, distinct per frame/channel; not
// audio-like), no real CIC/FIR pipeline involved. Complements
// tb_gbe_pipeline.v's end-to-end coverage the same way tb_fir_compensator.v
// complements tb_cluster_top.v: isolates this module's own logic (ping-pong
// CDC, header/checksum construction, byte serialization) from pipeline
// integration concerns, matching this project's convention of one dedicated
// testbench per RTL module. Vectors from
// fpga/single_fpga/golden/gen_vectors.py's gen_gbe_packetizer_vectors().
module tb_gbe_packetizer;
    localparam integer N_CH            = 96;
    localparam integer N_FRAMES        = 22;
    localparam integer FRAMES_PER_PKT  = 5;
    localparam integer TOTAL_PKT_BYTES = 1494;
    localparam integer N_PACKETS       = N_FRAMES / FRAMES_PER_PKT; // 4

    reg clk = 0;
    always #163 clk = ~clk; // matches tb_gbe_pipeline.v's PDM-rate choice

    reg tx_clk = 0;
    always #3.973 tx_clk = ~tx_clk; // ~125 MHz GMII rate

    reg rst = 1'b1;
    reg tx_rst = 1'b1;
    reg tx_ready = 1'b1; // no backpressure -- functional check only

    reg [N_CH-1:0]        fir_valid_in = 0;
    reg [N_CH*24-1:0]     fir_data_in  = 0;

    wire [7:0] tdata;
    wire       tvalid, tlast, tuser;

    gbe_packetizer #(
        .DST_MAC(48'hAABBCCDDEEFF), .SRC_MAC(48'h001122334455),
        .SRC_IP(32'hC0A80002),      .DST_IP(32'hC0A80001),
        .SRC_PORT(16'd50000),       .DST_PORT(16'd50000)
    ) dut (
        .clk(clk), .rst(rst),
        .fir_valid_in(fir_valid_in), .fir_data_in(fir_data_in),
        .tx_clk(tx_clk), .tx_rst(tx_rst),
        .tx_axis_tdata(tdata), .tx_axis_tvalid(tvalid), .tx_axis_tready(tx_ready),
        .tx_axis_tlast(tlast), .tx_axis_tuser(tuser)
    );

    reg [23:0] stim_mem [0:N_FRAMES*N_CH-1];
    reg [7:0]  exp_mem  [0:N_PACKETS*TOTAL_PKT_BYTES-1];
    initial begin
        $readmemh("../vectors/gbe_packetizer_stimulus.mem", stim_mem);
        $readmemh("../vectors/gbe_packetizer_expected.mem", exp_mem);
    end

    integer frame_i, ch;
    initial begin
        #1000 rst    = 1'b0;
        #1000 tx_rst = 1'b0;
        #500;
        for (frame_i = 0; frame_i < N_FRAMES; frame_i = frame_i + 1) begin
            @(posedge clk); #1;
            for (ch = 0; ch < N_CH; ch = ch + 1)
                fir_data_in[24*ch +: 24] = stim_mem[frame_i*N_CH + ch];
            fir_valid_in = {N_CH{1'b1}};
            @(posedge clk); #1;
            fir_valid_in = 0;
            // idle gap between frames -- real magnitude doesn't matter here
            // (no real pipeline timing to match), just needs to comfortably
            // exceed one packet's drain time so production never outruns
            // consumption (see GBE_FRAMING.md's margin note).
            repeat (100) @(posedge clk);
        end
    end

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
                    $display("FAIL: tlast mismatch at byte %0d", pkt_byte_idx);
                    errors = errors + 1;
                end
            end else begin
                $display("FAIL: unexpected extra byte %0d beyond %0d expected",
                         pkt_byte_idx, N_PACKETS*TOTAL_PKT_BYTES);
                errors = errors + 1;
            end

            pkt_byte_idx = pkt_byte_idx + 1;

            if (pkt_byte_idx == N_PACKETS*TOTAL_PKT_BYTES) begin
                if (errors == 0)
                    $display("PASS: tb_gbe_packetizer, %0d packets x %0d bytes all bit-exact",
                             N_PACKETS, TOTAL_PKT_BYTES);
                else
                    $display("FAIL: tb_gbe_packetizer, %0d errors", errors);
                $finish;
            end
        end
    end

    initial begin
        #3_000_000;
        $display("FAIL: tb_gbe_packetizer timed out, only %0d/%0d bytes received",
                  pkt_byte_idx, N_PACKETS*TOTAL_PKT_BYTES);
        $finish;
    end
endmodule
