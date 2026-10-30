`timescale 1ns / 1ps

// Answers ARP requests for MY_IP with MY_MAC, so a host can find the board
// without a static ARP entry, and merges the replies into the UDP stream.
//
// RX side (rx_clk, from eth_mac_1g_rgmii's rx_axis_*; the MAC has already
// stripped the FCS and flags a bad frame with tuser on tlast): a frame is a
// request for us if it is addressed to the broadcast address or MY_MAC, has
// EtherType 0x0806, is Ethernet/IPv4 ARP (htype 1, ptype 0x0800, hlen 6,
// plen 4), has opcode 1 (request), targets MY_IP, is longer than 42 bytes, and
// ends with tuser = 0. Its sender MAC/IP are latched and handed to the TX
// side with a toggle handshake; until the TX side acknowledges, further
// requests are dropped (a host retries ARP, and replies take ~1 us).
//
// TX side (tx_clk): builds the 42-byte reply (the MAC pads it to the 60-byte
// minimum) -- dst = requester's MAC, src = MY_MAC, opcode 2, sender = us,
// target = the requester -- and arbitrates it with the packetizer's stream
// at frame boundaries only: whichever source starts a frame owns tx_axis_*
// until its tlast, and a pending reply goes first. The packetizer just sees
// tready low for the ~0.5 us a reply takes, well inside its ping-pong margin
// (GBE_FRAMING.md, ~8.7x).
module arp_responder #(
    parameter [47:0] MY_MAC = 48'h000000000000,
    parameter [31:0] MY_IP  = 32'h00000000
) (
    // MAC RX (rx_clk domain)
    input  wire        rx_clk,
    input  wire        rx_rst,
    input  wire [7:0]  rx_tdata,
    input  wire        rx_tvalid,
    input  wire        rx_tlast,
    input  wire        rx_tuser,

    // tx_clk domain: packetizer in, MAC TX out
    input  wire        tx_clk,
    input  wire        tx_rst,
    input  wire [7:0]  s_tdata,
    input  wire        s_tvalid,
    output wire        s_tready,
    input  wire        s_tlast,
    input  wire        s_tuser,
    output wire [7:0]  m_tdata,
    output wire        m_tvalid,
    input  wire        m_tready,
    output wire        m_tlast,
    output wire        m_tuser
);
    // ---------------- RX: parse ----------------
    reg [5:0]  rx_idx = 0;              // byte index, saturates at 42
    reg        rx_ok  = 1'b1;           // nothing seen yet rules this frame out
    reg        rx_bcast = 1'b1, rx_mine = 1'b1;
    reg [47:0] rx_sha = 0;
    reg [31:0] rx_spa = 0;

    // byte b of a field held MSB-first in a vector
    function [7:0] fb(input [47:0] v, input integer nbytes, input integer b);
        fb = v >> (8 * (nbytes - 1 - b));
    endfunction

    // handshake (req_tog: rx -> tx, ack_tog: tx -> rx)
    reg        req_tog = 1'b0;
    reg [47:0] req_sha = 0;             // held stable while req_tog != ack
    reg [31:0] req_spa = 0;
    wire       ack_tog;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_sync = 2'b00;
    always @(posedge rx_clk) ack_sync <= {ack_sync[0], ack_tog};
    wire rx_busy = (req_tog != ack_sync[1]);

    always @(posedge rx_clk) begin
        if (rx_rst) begin
            rx_idx <= 0; rx_ok <= 1'b1; rx_bcast <= 1'b1; rx_mine <= 1'b1;
        end else if (rx_tvalid) begin
            case (rx_idx)
                0, 1, 2, 3, 4, 5: begin
                    if (rx_tdata != 8'hFF)                     rx_bcast <= 1'b0;
                    if (rx_tdata != fb(MY_MAC, 6, rx_idx))     rx_mine  <= 1'b0;
                end
                12: if (rx_tdata != 8'h08) rx_ok <= 1'b0;      // EtherType 0x0806
                13: if (rx_tdata != 8'h06) rx_ok <= 1'b0;
                14: if (rx_tdata != 8'h00) rx_ok <= 1'b0;      // htype 1
                15: if (rx_tdata != 8'h01) rx_ok <= 1'b0;
                16: if (rx_tdata != 8'h08) rx_ok <= 1'b0;      // ptype 0x0800
                17: if (rx_tdata != 8'h00) rx_ok <= 1'b0;
                18: if (rx_tdata != 8'h06) rx_ok <= 1'b0;      // hlen
                19: if (rx_tdata != 8'h04) rx_ok <= 1'b0;      // plen
                20: if (rx_tdata != 8'h00) rx_ok <= 1'b0;      // opcode 1 = request
                21: if (rx_tdata != 8'h01) rx_ok <= 1'b0;
                22, 23, 24, 25, 26, 27: rx_sha <= {rx_sha[39:0], rx_tdata};
                28, 29, 30, 31:         rx_spa <= {rx_spa[23:0], rx_tdata};
                38, 39, 40, 41: if (rx_tdata != fb(MY_IP, 4, rx_idx - 38)) rx_ok <= 1'b0;
                default: ;
            endcase
            if (rx_idx != 42) rx_idx <= rx_idx + 1'b1;

            if (rx_tlast) begin
                // tlast past byte 41 (every frame on the wire is >= 60 bytes before
                // the FCS), so all of the checks above have landed in rx_ok
                if (rx_ok && (rx_bcast || rx_mine) && !rx_tuser && rx_idx == 42 && !rx_busy) begin
                    req_sha <= rx_sha;
                    req_spa <= rx_spa;
                    req_tog <= ~req_tog;
                end
                rx_idx <= 0; rx_ok <= 1'b1; rx_bcast <= 1'b1; rx_mine <= 1'b1;
            end
        end
    end

    // ---------------- TX: reply + arbitration ----------------
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_sync = 3'b000;
    always @(posedge tx_clk) req_sync <= {req_sync[1:0], req_tog};
    wire req_seen = req_sync[2] ^ req_sync[1];

    reg        ack_r = 1'b0;
    assign     ack_tog = ack_r;
    reg        arp_pending = 1'b0;
    reg [47:0] tha = 0;
    reg [31:0] tpa = 0;

    localparam [1:0] OWN_NONE = 2'd0, OWN_UDP = 2'd1, OWN_ARP = 2'd2;
    reg [1:0] own = OWN_NONE;
    reg [5:0] arp_idx = 0;

    function [7:0] arp_byte(input [5:0] i);
        case (i)
            0, 1, 2, 3, 4, 5:       arp_byte = fb(tha, 6, i);          // dst = requester
            6, 7, 8, 9, 10, 11:     arp_byte = fb(MY_MAC, 6, i - 6);   // src
            12: arp_byte = 8'h08;  13: arp_byte = 8'h06;               // EtherType ARP
            14: arp_byte = 8'h00;  15: arp_byte = 8'h01;               // htype 1
            16: arp_byte = 8'h08;  17: arp_byte = 8'h00;               // ptype IPv4
            18: arp_byte = 8'h06;  19: arp_byte = 8'h04;               // hlen, plen
            20: arp_byte = 8'h00;  21: arp_byte = 8'h02;               // opcode 2 = reply
            22, 23, 24, 25, 26, 27: arp_byte = fb(MY_MAC, 6, i - 22);  // sender = us
            28, 29, 30, 31:         arp_byte = fb(MY_IP, 4, i - 28);
            32, 33, 34, 35, 36, 37: arp_byte = fb(tha, 6, i - 32);     // target = requester
            default:                arp_byte = fb(tpa, 4, i - 38);     // 38..41
        endcase
    endfunction

    always @(posedge tx_clk) begin
        if (tx_rst) begin
            own <= OWN_NONE; arp_idx <= 0; arp_pending <= 1'b0;
            ack_r <= req_sync[1];           // nothing outstanding after a reset
        end else begin
            if (req_seen && !arp_pending) begin
                tha <= req_sha; tpa <= req_spa; arp_pending <= 1'b1;
            end
            case (own)
                OWN_NONE:
                    if (arp_pending)   begin own <= OWN_ARP; arp_idx <= 0; end
                    else if (s_tvalid) own <= OWN_UDP;
                OWN_UDP:
                    if (s_tvalid && m_tready && s_tlast) own <= OWN_NONE;
                OWN_ARP:
                    if (m_tready) begin
                        if (arp_idx == 6'd41) begin
                            own <= OWN_NONE; arp_pending <= 1'b0; ack_r <= ~ack_r;
                        end else begin
                            arp_idx <= arp_idx + 1'b1;
                        end
                    end
                default: own <= OWN_NONE;
            endcase
        end
    end

    assign m_tdata  = (own == OWN_ARP) ? arp_byte(arp_idx) : s_tdata;
    assign m_tvalid = (own == OWN_ARP) || (own == OWN_UDP && s_tvalid);
    assign m_tlast  = (own == OWN_ARP) ? (arp_idx == 6'd41) : s_tlast;
    assign m_tuser  = (own == OWN_ARP) ? 1'b0 : s_tuser;
    assign s_tready = (own == OWN_UDP) && m_tready;
endmodule
