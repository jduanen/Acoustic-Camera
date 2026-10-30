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
// Payload storage is one block RAM (pkt_mem, three 24-bit channel samples per 72-bit
// word), written over 32 cycles per frame -- see "Payload memory" below. Before that it
// was a register array (pkt_ch_mem[bank][slot], one channel per address, all 96 written
// in the cycle fir_valid_in pulsed), which no RAM can take: ~23K flip-flops. Before
// *that* it was a flat bit-vector sliced at a dynamic bit offset.
// An earlier version used the latter (`pkt_buf[wr_bank][(...)*8 +: 2304] <= ...`) -- it
// simulated correctly, but a real placed build showed it cost ~46K LUTs on
// xc7a200tfbg484-1 (see the project's plan file, Stage 2 utilization section): Vivado
// can't map "insert a wide chunk at a runtime bit offset within a huge vector" onto a real
// RAM write port, so it fell back to a wide barrel-shifter/mux network built from
// distributed-RAM primitives, and `report_methodology`'s own `ULMTCS-2` flagged the
// resulting control-set count as requiring reduction. Proper array indexing (address is
// still runtime-computed, but selects a whole memory word rather than an arbitrary bit
// range) is the pattern synthesis tools map to real RAM efficiently, regardless of the
// address being dynamic -- confirmed by a real re-placed build after this rewrite.
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

    localparam integer HDR_FIELD_BYTES   = 4 + 8;                            // seq_num + timestamp = 12
    localparam integer ETH_HDR_BYTES     = 42;                               // Eth(14)+IP(20)+UDP(8)
    localparam integer HDR_TOTAL_BYTES   = ETH_HDR_BYTES + HDR_FIELD_BYTES;  // 54
    localparam integer PAYLOAD_CH_TOTAL  = FRAMES_PER_PKT * N_CH;            // 480 (channel slots/bank)
    localparam integer PKT_PAYLOAD_BYTES = HDR_FIELD_BYTES + PAYLOAD_CH_TOTAL*3; // 1452
    localparam integer TOTAL_PKT_BYTES   = HDR_TOTAL_BYTES + PAYLOAD_CH_TOTAL*3; // 1494
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
    // Payload memory: one block RAM, simple dual port -- written in the clk
    // domain, read in tx_clk. Three channels per 72-bit word (ch 3k in
    // [71:48], 3k+1 in [47:24], 3k+2 in [23:0]), so a bank's 5 frames x 96
    // channels are 160 words; each bank gets 256 addresses (bank = address
    // bit 8), 512 x 72 in all -- exactly one RAMB36 in SDP mode. Channel slot
    // s = frame*N_CH + ch lives in word s/3 = frame*32 + ch/3, position s%3.
    //
    // A RAM can't take all N_CH channels in the one cycle fir_valid_in pulses,
    // so a frame is written as WORDS_PER_FRAME (32) words over the following
    // 32 clk cycles. That relies on fir_data_in holding still until the next
    // frame -- fir_compensator holds data_out between valids, and frames are
    // 64 clks apart in the pipeline (100 in tb_gbe_packetizer.v). The batch is
    // stamped and handed to tx_clk only once its last frame's words are in.
    // ------------------------------------------------------------------
    localparam integer CH_PER_WORD     = 3;
    localparam integer WORDS_PER_FRAME = N_CH / CH_PER_WORD;          // 32
    localparam integer WORDS_PER_BANK  = FRAMES_PER_PKT * WORDS_PER_FRAME; // 160
    localparam integer WORD_W          = CH_PER_WORD * FIR_WIDTH;     // 72
    localparam integer WADDR_W         = 8;                           // within a bank
    initial if (N_CH % CH_PER_WORD != 0 || WORDS_PER_BANK > (1 << WADDR_W)) begin
        $display("FATAL: gbe_packetizer payload RAM layout needs N_CH %% 3 == 0 and <= 256 words per bank");
        $finish;
    end

    wire frame_valid = &fir_valid_in; // all N_CH channels' samples valid together

    (* ram_style = "block" *) reg [WORD_W-1:0] pkt_mem [0:(2 << WADDR_W)-1];
    reg [31:0]           pkt_seq_r [0:1]; // this bank's stamped seq_num, latched at batch close
    reg [63:0]           pkt_ts_r  [0:1]; // this bank's stamped timestamp, latched at batch close

    reg [FRAME_IDX_W-1:0] frame_idx;
    reg                   wr_bank;
    reg                   wr_active;
    reg [$clog2(WORDS_PER_FRAME)-1:0] wr_word;
    reg [63:0]            sample_counter;   // free-running, increments once per frame (48kHz)
    reg [63:0]            batch_ts_r;       // timestamp of frame 0 of the batch being filled
    reg [31:0]            pkt_seq;
    reg                   pkt_ready_toggle;

    wire [WADDR_W-1:0] wr_addr = frame_idx * WORDS_PER_FRAME + wr_word;
    wire [WORD_W-1:0]  wr_data = {fir_data_in[FIR_WIDTH*(CH_PER_WORD*wr_word)     +: FIR_WIDTH],
                                  fir_data_in[FIR_WIDTH*(CH_PER_WORD*wr_word + 1) +: FIR_WIDTH],
                                  fir_data_in[FIR_WIDTH*(CH_PER_WORD*wr_word + 2) +: FIR_WIDTH]};

    always @(posedge clk)
        if (wr_active)
            pkt_mem[{wr_bank, wr_addr}] <= wr_data;

    always @(posedge clk) begin
        if (rst) begin
            frame_idx        <= {FRAME_IDX_W{1'b0}};
            wr_bank           <= 1'b0;
            wr_active         <= 1'b0;
            wr_word           <= 0;
            sample_counter    <= 64'd0;
            batch_ts_r        <= 64'd0;
            pkt_seq           <= 32'd0;
            pkt_ready_toggle  <= 1'b0;
        end else begin
            if (frame_valid && !wr_active) begin
                sample_counter <= sample_counter + 64'd1;
                if (frame_idx == {FRAME_IDX_W{1'b0}})
                    batch_ts_r <= sample_counter;
                wr_active <= 1'b1;
                wr_word   <= 0;
            end else if (wr_active) begin
                if (wr_word != WORDS_PER_FRAME - 1) begin
                    wr_word <= wr_word + 1'b1;
                end else begin
                    wr_active <= 1'b0;
                    if (frame_idx == FRAMES_PER_PKT-1) begin
                        // batch complete: stamp this bank's seq_num/timestamp, hand it
                        // off to the tx_clk domain, and start filling the other bank.
                        pkt_seq_r[wr_bank] <= pkt_seq;
                        pkt_ts_r[wr_bank]  <= batch_ts_r;

                        pkt_seq          <= pkt_seq + 32'd1;
                        pkt_ready_toggle <= ~pkt_ready_toggle;
                        wr_bank          <= ~wr_bank;
                        frame_idx        <= {FRAME_IDX_W{1'b0}};
                    end else begin
                        frame_idx <= frame_idx + 1'b1;
                    end
                end
            end
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
    //
    // byte_cnt free-runs 0..TOTAL_PKT_BYTES-1 across the whole packet (drives
    // tlast); word_idx/ch_in_word/byte_in_ch track position within the payload
    // region only (0..WORDS_PER_BANK-1 words x 3 channels x 3 bytes), avoiding a
    // runtime divide-by-3 to recover a word/channel/byte address from a flat
    // byte offset. The RAM is addressed with word_nx, the word the *next*
    // cycle needs, so its one-cycle read latency is absorbed; tready low just
    // holds the address.
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] tog_sync;
    wire      pkt_pulse = tog_sync[2] ^ tog_sync[1];

    reg                     pkt_pending;
    reg                     rd_bank;
    reg                     tx_active;
    reg [BYTE_CNT_W-1:0]    byte_cnt;
    reg [WADDR_W-1:0]       word_idx;
    reg [1:0]               ch_in_word;
    reg [1:0]               byte_in_ch;

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            tog_sync <= 3'b0;
        end else begin
            tog_sync <= {tog_sync[1:0], pkt_ready_toggle};
        end
    end

    wire advance    = tx_active && tx_axis_tready;
    wire in_payload = (byte_cnt >= HDR_TOTAL_BYTES);
    wire [WADDR_W-1:0] word_nx =
        (advance && in_payload && byte_in_ch == 2'd2 && ch_in_word == 2'd2) ? word_idx + 1'b1 :
        (!tx_active)                                                       ? {WADDR_W{1'b0}} :
                                                                             word_idx;

    reg [WORD_W-1:0] rd_word;
    always @(posedge tx_clk)
        rd_word <= pkt_mem[{rd_bank, word_nx}];

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            pkt_pending <= 1'b0;
            rd_bank     <= 1'b0;
            tx_active   <= 1'b0;
            byte_cnt    <= {BYTE_CNT_W{1'b0}};
            word_idx    <= {WADDR_W{1'b0}};
            ch_in_word  <= 2'd0;
            byte_in_ch  <= 2'd0;
        end else begin
            if (pkt_pulse)
                pkt_pending <= 1'b1;
            word_idx <= word_nx;

            if (!tx_active) begin
                ch_in_word <= 2'd0;
                byte_in_ch <= 2'd0;
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
                    if (in_payload) begin
                        if (byte_in_ch == 2'd2) begin
                            byte_in_ch <= 2'd0;
                            ch_in_word <= (ch_in_word == 2'd2) ? 2'd0 : ch_in_word + 1'b1;
                        end else begin
                            byte_in_ch <= byte_in_ch + 1'b1;
                        end
                    end
                    byte_cnt <= byte_cnt + 1'b1;
                end
            end
        end
    end

    assign tx_axis_tvalid = tx_active;
    assign tx_axis_tlast  = tx_active && (byte_cnt == TOTAL_PKT_BYTES-1);
    assign tx_axis_tuser  = 1'b0; // no frame-error signaling -- nothing generates one here

    wire [3:0] hdr_field_idx = byte_cnt - ETH_HDR_BYTES; // valid only when HDR_TOTAL_BYTES>byte_cnt>=ETH_HDR_BYTES
    wire [7:0] hdr_field_byte = (hdr_field_idx < 4)
        ? pkt_seq_r[rd_bank][(3 - hdr_field_idx)*8 +: 8]
        : pkt_ts_r[rd_bank][(11 - hdr_field_idx)*8 +: 8];

    wire [FIR_WIDTH-1:0] cur_ch_val = (ch_in_word == 2'd0) ? rd_word[71:48] :
                                      (ch_in_word == 2'd1) ? rd_word[47:24] :
                                                             rd_word[23:0];
    wire [7:0] payload_byte = (byte_in_ch == 2'd0) ? cur_ch_val[23:16] :
                               (byte_in_ch == 2'd1) ? cur_ch_val[15:8]  :
                                                       cur_ch_val[7:0];

    assign tx_axis_tdata =
        (byte_cnt < ETH_HDR_BYTES)   ? FIXED_HDR[(ETH_HDR_BYTES-1-byte_cnt)*8 +: 8] :
        (byte_cnt < HDR_TOTAL_BYTES) ? hdr_field_byte :
                                        payload_byte;

endmodule
