`timescale 1ns / 1ps

// Packetization FSM connecting the single-FPGA hub's 96-channel CIC+FIR pipeline
// (single_fpga_top_spike.v's reused pdm_line_demux/cic_decimator/fir_compensator chain)
// to the vendored MAC's (third_party/verilog-ethernet/eth_mac_1g_rgmii.v) tx_axis_*
// AXI-Stream TX interface. Byte layout, rate budget, and design rationale are documented
// in full in ../GBE_FRAMING.md -- read that first; this header only summarizes what's
// implemented here.
//
// Two clock domains:
//   - clk     : the pipeline's own clock. fir_data_in/fir_valid_in are sampled here.
//   - tx_clk  : eth_mac_1g_rgmii.v's own tx_clk output (GMII-side, genuinely async to
//               clk -- no fixed phase relationship). tx_axis_* is driven here.
// Crossing technique: a ping-pong (2-bank) payload memory written one frame at a time in
// the clk domain, handed off to the tx_clk domain via a toggle-flip-flop + 2-FF-synchronizer
// + edge-detect once per completed 5-frame batch -- see GBE_FRAMING.md's "Clock domain
// crossing" section for the margin analysis (~8.7x) backing why a single ping-pong pair is
// enough, no deeper queuing needed.
//
// Header (Ethernet+IP+UDP, 42 bytes) is byte-identical on every packet -- every field is a
// build-time constant (fixed addresses, fixed length, DF=1/Identification=0 so it never
// fragments). Built once at elaboration time as FIXED_HDR below, including the IPv4 header
// checksum (computed by ip_checksum(), a function over this module's parameters -- stays
// correct automatically if SRC_IP/DST_IP ever change, no runtime checksum logic exists).
//
// FCS, preamble/SFD, and short-frame padding are NOT handled here -- confirmed from
// axis_gmii_tx.v's own state machine (STATE_PREAMBLE/STATE_FCS/STATE_PAD) that the MAC
// does all three automatically from a raw tx_axis_tdata frame-body byte stream. This
// module supplies only the destination-MAC-through-payload bytes.
module gbe_packetizer #(
    parameter [47:0] DST_MAC  = 48'h000000000000,
    parameter [47:0] SRC_MAC  = 48'h000000000000,
    parameter [31:0] SRC_IP   = 32'hC0A80002, // 192.168.0.2
    parameter [31:0] DST_IP   = 32'hC0A80001, // 192.168.0.1
    parameter [15:0] SRC_PORT = 16'd50000,
    parameter [15:0] DST_PORT = 16'd50000,
    parameter integer N_CH          = 96,
    parameter integer FIR_WIDTH     = 24,
    parameter integer FRAMES_PER_PKT = 5
) (
    // Pipeline-side (clk domain)
    input  wire                          clk,
    input  wire                          rst,
    input  wire [N_CH-1:0]               fir_valid_in,
    input  wire [N_CH*FIR_WIDTH-1:0]     fir_data_in,

    // MAC-side (tx_clk domain) -- eth_mac_1g_rgmii.v's tx_axis_* port set
    input  wire                          tx_clk,
    input  wire                          tx_rst,
    output wire [7:0]                    tx_axis_tdata,
    output wire                          tx_axis_tvalid,
    input  wire                          tx_axis_tready,
    output wire                          tx_axis_tlast,
    output wire                          tx_axis_tuser
);

    localparam integer PAYLOAD_BYTES     = N_CH * 3;                         // 288
    localparam integer HDR_FIELD_BYTES   = 4 + 8;                            // seq_num + timestamp = 12
    localparam integer PKT_PAYLOAD_BYTES = HDR_FIELD_BYTES + FRAMES_PER_PKT*PAYLOAD_BYTES; // 1452
    localparam integer ETH_HDR_BYTES     = 42;                               // Eth(14)+IP(20)+UDP(8)
    localparam integer TOTAL_PKT_BYTES   = ETH_HDR_BYTES + PKT_PAYLOAD_BYTES; // 1494
    localparam integer BYTE_CNT_W        = $clog2(TOTAL_PKT_BYTES);
    localparam integer FRAME_IDX_W       = $clog2(FRAMES_PER_PKT);

    // ------------------------------------------------------------------
    // Fixed 42-byte header, built once at elaboration time. See
    // GBE_FRAMING.md's "Byte layout" table for the field-by-field spec.
    // ------------------------------------------------------------------
    localparam [7:0]  IP_VER_IHL    = 8'h45;
    localparam [7:0]  IP_DSCP_ECN   = 8'h00;
    localparam [15:0] IP_TOTAL_LEN  = 20 + 8 + PKT_PAYLOAD_BYTES; // 1480, constant every packet
    localparam [15:0] IP_ID         = 16'h0000;                  // unused: DF=1, never fragmented
    localparam [15:0] IP_FLAGS_FRAG = 16'h4000;                  // DF=1, offset 0
    localparam [7:0]  IP_TTL        = 8'd64;
    localparam [7:0]  IP_PROTO      = 8'd17;                     // UDP
    localparam [15:0] UDP_LEN       = 8 + PKT_PAYLOAD_BYTES;     // 1460, constant every packet

    // Standard IPv4 header checksum: ones'-complement sum of all 16-bit header words
    // (checksum field itself = 0 while summing), carries folded back in, then complemented.
    function [15:0] ip_checksum;
        input [15:0] w0, w1, w2, w3, w4, w5, w6, w7, w8, w9;
        reg [31:0] sum;
        begin
            sum = w0 + w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9;
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]}; // fold carry
            sum = {16'd0, sum[15:0]} + {16'd0, sum[31:16]}; // fold again (2nd carry, if any)
            ip_checksum = ~sum[15:0];
        end
    endfunction

    localparam [15:0] IP_HDR_CHECKSUM = ip_checksum(
        {IP_VER_IHL, IP_DSCP_ECN}, IP_TOTAL_LEN, IP_ID, IP_FLAGS_FRAG,
        {IP_TTL, IP_PROTO}, 16'h0000, SRC_IP[31:16], SRC_IP[15:0], DST_IP[31:16], DST_IP[15:0]
    );

    // Concatenation order = wire byte order (first element = first byte transmitted, at
    // this vector's MSB side) -- read out below via FIXED_HDR[(ETH_HDR_BYTES-1-idx)*8 +: 8].
    localparam [8*ETH_HDR_BYTES-1:0] FIXED_HDR = {
        DST_MAC, SRC_MAC, 16'h0800,
        IP_VER_IHL, IP_DSCP_ECN, IP_TOTAL_LEN, IP_ID, IP_FLAGS_FRAG, IP_TTL, IP_PROTO,
        IP_HDR_CHECKSUM, SRC_IP, DST_IP,
        SRC_PORT, DST_PORT, UDP_LEN, 16'h0000
    };

    // ------------------------------------------------------------------
    // clk domain: frame collector. Reorders each frame's 96 channels into
    // MSB-first-per-channel byte order (fir_data_in is LSB-first internally,
    // matching single_fpga_top_spike.v's [FIR_WIDTH*c +: FIR_WIDTH] convention),
    // then writes one frame (288 bytes) per fir valid pulse into the current
    // write bank at pkt_buf[wr_bank][byte_idx], byte_idx counting up from 0.
    // ------------------------------------------------------------------
    wire frame_valid = &fir_valid_in; // fully parallel, identical latency per channel -- see
                                       // GBE_FRAMING.md's "real architectural advantage" note

    wire [PAYLOAD_BYTES*8-1:0] frame_bytes_msb_first;
    genvar ch;
    generate
        for (ch = 0; ch < N_CH; ch = ch + 1) begin : g_reorder
            assign frame_bytes_msb_first[8*(3*ch+0) +: 8] = fir_data_in[FIR_WIDTH*ch+23 -: 8]; // MSB byte
            assign frame_bytes_msb_first[8*(3*ch+1) +: 8] = fir_data_in[FIR_WIDTH*ch+15 -: 8]; // mid byte
            assign frame_bytes_msb_first[8*(3*ch+2) +: 8] = fir_data_in[FIR_WIDTH*ch+7  -: 8]; // LSB byte
        end
    endgenerate

    reg [8*PKT_PAYLOAD_BYTES-1:0] pkt_buf [0:1]; // ping-pong payload memory (byte idx0 = low bit offset)
    reg [FRAME_IDX_W-1:0] frame_idx;
    reg                   wr_bank;
    reg [63:0]            sample_counter;   // free-running, increments once per frame (48kHz)
    reg [63:0]            batch_ts_r;       // timestamp of frame 0 of the batch being filled
    reg [31:0]            pkt_seq;
    reg                   pkt_ready_toggle;

    // Built up, then written to pkt_buf[wr_bank] with a single nonblocking
    // assignment per cycle (below) -- an earlier version issued several
    // separate nonblocking partial-select writes to the same dynamically-
    // indexed pkt_buf[wr_bank] row (payload write + up to 12 header-byte
    // writes) in one always block invocation; on the batch-closing cycle
    // xsim did not merge all of them correctly, silently dropping some of
    // that cycle's payload bytes (caught by the bit-exact pipeline
    // testbench comparison, not by inspection). One write per row per
    // cycle sidesteps that entirely and is the more conventional pattern
    // for indexed-memory writes besides.
    reg [8*PKT_PAYLOAD_BYTES-1:0] next_bank_val;

    always @(posedge clk) begin
        if (rst) begin
            frame_idx        <= {FRAME_IDX_W{1'b0}};
            wr_bank           <= 1'b0;
            sample_counter    <= 64'd0;
            batch_ts_r        <= 64'd0;
            pkt_seq           <= 32'd0;
            pkt_ready_toggle  <= 1'b0;
        end else if (frame_valid) begin
            sample_counter <= sample_counter + 64'd1;

            if (frame_idx == {FRAME_IDX_W{1'b0}})
                batch_ts_r <= sample_counter;

            // this frame's 288 bytes, at byte offset HDR_FIELD_BYTES + frame_idx*PAYLOAD_BYTES
            next_bank_val = pkt_buf[wr_bank];
            next_bank_val[(HDR_FIELD_BYTES + frame_idx*PAYLOAD_BYTES)*8 +: PAYLOAD_BYTES*8]
                = frame_bytes_msb_first;

            if (frame_idx == FRAMES_PER_PKT-1) begin
                // batch complete: stamp seq_num/timestamp into this bank's first 12 bytes,
                // hand it off to the tx_clk domain, and start filling the other bank.
                next_bank_val[0  +: 8] = pkt_seq[31:24];
                next_bank_val[8  +: 8] = pkt_seq[23:16];
                next_bank_val[16 +: 8] = pkt_seq[15:8];
                next_bank_val[24 +: 8] = pkt_seq[7:0];
                next_bank_val[32 +: 8] = batch_ts_r[63:56];
                next_bank_val[40 +: 8] = batch_ts_r[55:48];
                next_bank_val[48 +: 8] = batch_ts_r[47:40];
                next_bank_val[56 +: 8] = batch_ts_r[39:32];
                next_bank_val[64 +: 8] = batch_ts_r[31:24];
                next_bank_val[72 +: 8] = batch_ts_r[23:16];
                next_bank_val[80 +: 8] = batch_ts_r[15:8];
                next_bank_val[88 +: 8] = batch_ts_r[7:0];

                pkt_seq          <= pkt_seq + 32'd1;
                pkt_ready_toggle <= ~pkt_ready_toggle;
                wr_bank          <= ~wr_bank;
                frame_idx        <= {FRAME_IDX_W{1'b0}};
            end else begin
                frame_idx <= frame_idx + 1'b1;
            end

            pkt_buf[wr_bank] <= next_bank_val; // wr_bank here is still this cycle's (pre-toggle) value
        end
    end

    // ------------------------------------------------------------------
    // tx_clk domain: 2-FF synchronizer + edge detect on pkt_ready_toggle, then
    // serialize one full TOTAL_PKT_BYTES-byte packet (header, then payload) onto
    // tx_axis_*, respecting tx_axis_tready backpressure. rd_bank tracks wr_bank's
    // alternation implicitly (one produce per batch, one consume per detected
    // pulse, consumption always finishes well before the next batch closes --
    // see GBE_FRAMING.md's ~8.7x margin note), so the bank index itself is never
    // transferred across the crossing, only the toggle.
    // ------------------------------------------------------------------
    reg [2:0] tog_sync;
    wire      pkt_pulse = tog_sync[2] ^ tog_sync[1];

    reg                     pkt_pending;
    reg                     rd_bank;
    reg                     tx_active;
    reg [BYTE_CNT_W-1:0]    byte_cnt;

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            tog_sync <= 3'b0;
        end else begin
            tog_sync <= {tog_sync[1:0], pkt_ready_toggle};
        end
    end

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            pkt_pending <= 1'b0;
            rd_bank     <= 1'b0;
            tx_active   <= 1'b0;
            byte_cnt    <= {BYTE_CNT_W{1'b0}};
        end else begin
            if (pkt_pulse)
                pkt_pending <= 1'b1;

            if (!tx_active) begin
                if (pkt_pending) begin
                    tx_active   <= 1'b1;
                    pkt_pending <= 1'b0;
                    byte_cnt    <= {BYTE_CNT_W{1'b0}};
                end
            end else if (tx_axis_tready) begin
                if (byte_cnt == TOTAL_PKT_BYTES-1) begin
                    tx_active <= 1'b0;
                    rd_bank   <= ~rd_bank;
                end else begin
                    byte_cnt <= byte_cnt + 1'b1;
                end
            end
        end
    end

    assign tx_axis_tvalid = tx_active;
    assign tx_axis_tlast  = tx_active && (byte_cnt == TOTAL_PKT_BYTES-1);
    assign tx_axis_tuser  = 1'b0; // no frame-error signaling -- nothing generates one here

    assign tx_axis_tdata = (byte_cnt < ETH_HDR_BYTES)
        ? FIXED_HDR[(ETH_HDR_BYTES-1-byte_cnt)*8 +: 8]
        : pkt_buf[rd_bank][(byte_cnt-ETH_HDR_BYTES)*8 +: 8];

endmodule
