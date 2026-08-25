`timescale 1ns / 1ps

// Hub-side deframer for one spoke -- inverse of fpga/multi_fpga/cluster/rtl/spoke_framer.v.
// One instance per spoke (4x in hub_top.v). See fpga/multi_fpga/SPOKE_FRAMING.md for the
// protocol this implements.
//
// DDR capture: spoke_framer.v drives `assign spoke_d = clk ? d_rise : d_fall`
// -- a plain combinational mux on clk's *level* (not an ODDR primitive), so
// d_rise is valid on spoke_d for the whole high phase of a cycle, d_fall for
// the whole low phase. Capturing d_rise at the negedge that ends the high
// phase (mirrors pdm_line_demux.v's dual-edge technique) preserves it across
// the low phase that follows; d_fall doesn't need its own register, since
// spoke_d still combinationally shows it at the *next* posedge (the low
// phase hasn't ended, from spoke_d's perspective, until the following
// negedge) -- so rise_r and spoke_d together give one cycle's complete data
// at the posedge that ends that cycle.
//
// Lock: re-derives cycle-0 alignment from every spoke_strobe rising edge
// rather than assuming a specific inter-frame idle-cycle count -- see
// SPOKE_FRAMING.md's history (spoke_framer.v's own cyc_r needed the same
// "saturate, don't free-run" fix once already, when the frame period changed
// from 64 to 128 cycles under it). cyc_r counts 0..BUSY_CYCLES-1 while
// writing captured nibbles into acc_flat, then holds at BUSY_CYCLES for
// exactly one more cycle to hand the completed frame off to ch_data_flat
// (acc_flat's last write, from the previous edge, has settled into acc_flat
// by then -- doing the handoff on the *same* edge as that last write would
// read acc_flat's pre-write value and drop the last nibble, per ordinary
// nonblocking-assignment semantics), then sits at BUSY_CYCLES+1 through the
// rest of the idle cycles until the next spoke_strobe edge.
module spoke_deframer (
    input  wire              clk,
    input  wire              rst,
    input  wire [5:0]        spoke_d,
    input  wire              spoke_strobe,
    output reg  [24*24-1:0]  ch_data_flat,   // channel c = ch_data_flat[24*c +: 24]
    output reg                valid           // pulses 1 cycle once a frame completes
);
    localparam integer N_CH        = 24;
    localparam integer DATA_WIDTH  = 24;
    localparam integer BUSY_CYCLES = 48;

    reg [5:0] rise_r;
    always @(negedge clk) rise_r <= spoke_d;

    reg strobe_d;
    wire strobe_rise = spoke_strobe && !strobe_d;

    reg [5:0] cyc_r;
    reg       armed;
    reg [N_CH*DATA_WIDTH-1:0] acc_flat;

    wire [4:0] c_idx      = cyc_r[5:1];         // channel = cyc_r / 2
    wire       chunk_pair = cyc_r[0];            // 0: nibbles 0/1, 1: nibbles 2/3
    wire       busy       = armed && (cyc_r < BUSY_CYCLES);

    always @(posedge clk) begin
        strobe_d <= spoke_strobe;
        valid    <= 1'b0; // default; pulses exactly 1 cycle on frame completion

        if (rst) begin
            armed        <= 1'b0;
            cyc_r        <= 6'd0;
            ch_data_flat <= {(N_CH*DATA_WIDTH){1'b0}};
            acc_flat     <= {(N_CH*DATA_WIDTH){1'b0}};
        end else if (strobe_rise) begin
            armed <= 1'b1;
            cyc_r <= 6'd0;
        end else if (busy) begin
            // rise_r/spoke_d here are this cycle's complete DDR data (see
            // header comment) -- inverse of spoke_framer.v's d_rise/d_fall
            // construction.
            if (!chunk_pair) begin
                acc_flat[24*c_idx + 18 +: 6] <= rise_r;   // bits [23:18]
                acc_flat[24*c_idx + 12 +: 6] <= spoke_d;  // bits [17:12]
            end else begin
                acc_flat[24*c_idx + 6  +: 6] <= rise_r;   // bits [11:6]
                acc_flat[24*c_idx + 0  +: 6] <= spoke_d;  // bits [5:0]
            end
            cyc_r <= cyc_r + 6'd1;
        end else if (armed && cyc_r == BUSY_CYCLES) begin
            ch_data_flat <= acc_flat;
            valid        <= 1'b1;
            cyc_r        <= cyc_r + 6'd1; // -> BUSY_CYCLES+1, holds there (saturates)
        end
    end
endmodule
