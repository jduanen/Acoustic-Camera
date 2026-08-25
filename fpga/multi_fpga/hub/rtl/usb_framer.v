`timescale 1ns / 1ps

// Packs the 4 spokes' reassembled channel data (from spoke_deframer.v x4 in hub_top.v) into
// per-spoke 76-byte records and serializes them onto the FT245 synchronous FIFO write
// interface. See fpga/USB_FRAMING.md for the protocol this implements.
//
// Two clock domains: `sclk` (spoke_deframer.v's own clock -- see hub_top.v) and `usb_clk`
// (from the FT232H's own CLKOUT, genuinely asynchronous to sclk -- no fixed phase
// relationship, unlike sclk itself which is generated from clk). rst_sclk/rst_usb are
// already-synchronized resets in their respective domains (2-FF synchronized from
// spoke_reset_n in hub_top.v, same pattern as rst_sclk's own generation there) -- this
// module does no reset synchronization of its own.
//
// Per-spoke CDC: a toggle flip-flop in the sclk domain flips on each spokeN_valid pulse;
// the usb_clk domain recovers a one-cycle request pulse via a 2-FF synchronizer + edge
// detect (same synchronizer pattern as reset_seq.v's alive_meta/alive_sync, applied to a
// toggle bit instead of a level). chN_data_flat itself needs no per-bit synchronizer --
// spoke_deframer.v holds it stable until its own next valid pulse, so once the synchronized
// request pulse says "safe to read," the data underneath it already is (see
// USB_FRAMING.md's "Clock domain crossing" section for the full reasoning/margin numbers).
//
// Arbitration + serialization (usb_clk domain, single always block below): a rotating-
// priority arbiter (last-served spoke lowest priority next time) picks among pending
// requests, latches that spoke's held data into a 608-bit shift register (8'hA5, 8'h5A,
// spoke_id, seq_num, then the 576-bit payload reordered so channel 0's bytes shift out
// first -- see the PAYLOAD_PACK generate block below), then shifts one byte out per usb_clk
// cycle while usb_txe_n is low, stalling (holding usb_wr_n deasserted) while it's high.
//
// Backpressure/drop: if a fresh request for spoke i arrives while its previous request is
// still pending (not yet picked by the arbiter), hold_i is simply overwritten with the new
// data and req_pending[i] stays set -- the stale, never-sent copy is silently dropped, not
// queued (documented in USB_FRAMING.md; the host detects the gap via seq_num). The "set"
// (req0-3 block) is ordered after the "clear on serve-start" (arbiter block) below so that a
// same-cycle coincidence -- a fresh request landing on the exact spoke just picked for
// service this cycle -- correctly leaves req_pending set for the fresh data rather than
// losing track of it (last nonblocking assignment to the same bit in one always block wins).
module usb_framer (
    input  wire        sclk,
    input  wire        rst_sclk,
    input  wire        usb_clk,
    input  wire        rst_usb,

    input  wire [24*24-1:0] spoke0_ch_data_flat, spoke1_ch_data_flat,
                             spoke2_ch_data_flat, spoke3_ch_data_flat,
    input  wire              spoke0_valid, spoke1_valid, spoke2_valid, spoke3_valid,

    output reg  [7:0]  usb_d,
    output reg          usb_wr_n,
    input  wire          usb_txe_n
);
    localparam integer N_CH        = 24;
    localparam integer DATA_WIDTH  = 24;
    localparam integer RECORD_LEN  = 76; // 4 header + 72 payload bytes

    // --- Per-spoke sclk-domain toggle + usb_clk-domain 2-FF sync/edge-detect ---
    reg tgl0_sclk = 1'b0, tgl1_sclk = 1'b0, tgl2_sclk = 1'b0, tgl3_sclk = 1'b0;
    always @(posedge sclk) begin
        if (rst_sclk) begin
            tgl0_sclk <= 1'b0; tgl1_sclk <= 1'b0; tgl2_sclk <= 1'b0; tgl3_sclk <= 1'b0;
        end else begin
            if (spoke0_valid) tgl0_sclk <= ~tgl0_sclk;
            if (spoke1_valid) tgl1_sclk <= ~tgl1_sclk;
            if (spoke2_valid) tgl2_sclk <= ~tgl2_sclk;
            if (spoke3_valid) tgl3_sclk <= ~tgl3_sclk;
        end
    end

    reg tgl0_meta=1'b0, tgl0_sync=1'b0, tgl0_sync_d=1'b0;
    reg tgl1_meta=1'b0, tgl1_sync=1'b0, tgl1_sync_d=1'b0;
    reg tgl2_meta=1'b0, tgl2_sync=1'b0, tgl2_sync_d=1'b0;
    reg tgl3_meta=1'b0, tgl3_sync=1'b0, tgl3_sync_d=1'b0;
    always @(posedge usb_clk) begin
        if (rst_usb) begin
            tgl0_meta<=1'b0; tgl0_sync<=1'b0; tgl0_sync_d<=1'b0;
            tgl1_meta<=1'b0; tgl1_sync<=1'b0; tgl1_sync_d<=1'b0;
            tgl2_meta<=1'b0; tgl2_sync<=1'b0; tgl2_sync_d<=1'b0;
            tgl3_meta<=1'b0; tgl3_sync<=1'b0; tgl3_sync_d<=1'b0;
        end else begin
            tgl0_meta<=tgl0_sclk; tgl0_sync<=tgl0_meta; tgl0_sync_d<=tgl0_sync;
            tgl1_meta<=tgl1_sclk; tgl1_sync<=tgl1_meta; tgl1_sync_d<=tgl1_sync;
            tgl2_meta<=tgl2_sclk; tgl2_sync<=tgl2_meta; tgl2_sync_d<=tgl2_sync;
            tgl3_meta<=tgl3_sclk; tgl3_sync<=tgl3_meta; tgl3_sync_d<=tgl3_sync;
        end
    end
    wire req0 = tgl0_sync ^ tgl0_sync_d;
    wire req1 = tgl1_sync ^ tgl1_sync_d;
    wire req2 = tgl2_sync ^ tgl2_sync_d;
    wire req3 = tgl3_sync ^ tgl3_sync_d;

    // --- usb_clk-domain per-spoke holding registers, sequence counters, pending flags ---
    reg [24*24-1:0] hold0=0, hold1=0, hold2=0, hold3=0;
    reg [7:0]       seq0=0, seq1=0, seq2=0, seq3=0;
    reg [3:0]       req_pending = 4'b0;

    // --- Rotating-priority arbiter: last_served's successor gets first look ---
    reg [1:0] last_served = 2'd3; // so the first pick after reset starts at spoke0

    wire [1:0] rot1 = last_served + 2'd1;
    wire [1:0] rot2 = last_served + 2'd2;
    wire [1:0] rot3 = last_served + 2'd3;
    wire       sel_valid = |req_pending;
    wire [1:0] sel_spoke = req_pending[rot1] ? rot1 :
                            req_pending[rot2] ? rot2 :
                            req_pending[rot3] ? rot3 :
                            last_served; // only remaining possibility if sel_valid

    wire [24*24-1:0] sel_hold = (sel_spoke == 2'd0) ? hold0 :
                                 (sel_spoke == 2'd1) ? hold1 :
                                 (sel_spoke == 2'd2) ? hold2 : hold3;
    wire [7:0]       sel_seq  = (sel_spoke == 2'd0) ? seq0 :
                                 (sel_spoke == 2'd1) ? seq1 :
                                 (sel_spoke == 2'd2) ? seq2 : seq3;

    // Reorders sel_hold (channel 0 at bits [23:0], per spoke_deframer.v's ch_data_flat
    // convention) into transmission order: channel 0's bytes shift out of tx_shift first,
    // so channel 0 must land at tx_shift's MSB end.
    wire [24*24-1:0] sel_payload;
    genvar gi;
    generate
        for (gi = 0; gi < N_CH; gi = gi + 1) begin : PAYLOAD_PACK
            assign sel_payload[24*24-1 - 24*gi -: 24] = sel_hold[24*gi +: 24];
        end
    endgenerate

    reg [RECORD_LEN*8-1:0] tx_shift = 0;
    reg [6:0]              byte_cnt = 7'd0;
    reg                    tx_busy  = 1'b0;

    always @(posedge usb_clk) begin
        usb_wr_n <= 1'b1; // default deasserted; overridden below on an active write cycle

        if (rst_usb) begin
            hold0 <= 0; hold1 <= 0; hold2 <= 0; hold3 <= 0;
            seq0  <= 0; seq1  <= 0; seq2  <= 0; seq3  <= 0;
            req_pending <= 4'b0;
            last_served <= 2'd3;
            tx_busy  <= 1'b0;
            byte_cnt <= 7'd0;
            usb_d    <= 8'd0;
        end else begin
            // Start serializing the next selected record. Uses sel_hold/sel_seq/sel_payload
            // as they stood *before* this cycle's req0-3 updates below (registered reads),
            // so a same-cycle coincidence with a fresh request for sel_spoke is handled by
            // ordering (see req_pending[sel_spoke] here vs. the req_pending[N]<=1 sets below).
            if (!tx_busy && sel_valid) begin
                tx_shift    <= {8'hA5, 8'h5A, {6'd0, sel_spoke}, sel_seq, sel_payload};
                byte_cnt    <= 7'd76; // RECORD_LEN
                tx_busy     <= 1'b1;
                last_served <= sel_spoke;
                req_pending[sel_spoke] <= 1'b0;
                case (sel_spoke)
                    2'd0: seq0 <= seq0 + 8'd1;
                    2'd1: seq1 <= seq1 + 8'd1;
                    2'd2: seq2 <= seq2 + 8'd1;
                    2'd3: seq3 <= seq3 + 8'd1;
                endcase
            end else if (tx_busy && !usb_txe_n) begin
                usb_d    <= tx_shift[RECORD_LEN*8-1 -: 8];
                usb_wr_n <= 1'b0;
                tx_shift <= tx_shift << 8;
                byte_cnt <= byte_cnt - 7'd1;
                if (byte_cnt == 7'd1) tx_busy <= 1'b0;
            end

            // Latch fresh data whenever a request pulse arrives -- ordered after the arbiter
            // block above so a same-cycle "just started serving this spoke" + "fresh request
            // for that same spoke" coincidence leaves req_pending set (see header comment).
            if (req0) begin hold0 <= spoke0_ch_data_flat; req_pending[0] <= 1'b1; end
            if (req1) begin hold1 <= spoke1_ch_data_flat; req_pending[1] <= 1'b1; end
            if (req2) begin hold2 <= spoke2_ch_data_flat; req_pending[2] <= 1'b1; end
            if (req3) begin hold3 <= spoke3_ch_data_flat; req_pending[3] <= 1'b1; end
        end
    end
endmodule
