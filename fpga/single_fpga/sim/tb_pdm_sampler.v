`timescale 1ns / 1ps

// Checks pdm_sampler.v against a behavioural model of the 48 mic pairs. PDM_CLK
// comes from a TCXO/4 divider like single_fpga_top's and reaches the "mics"
// CLK_DLY later. Each mic drives its next random bit TDD after its edge (L
// after the fall, R after the rise) and releases the line THZ after the
// opposite edge; an undriven line reads X, so a capture outside a mic's
// window shows up as a mismatch, not a silently held value. RET_DLY models
// the return trace.
//
// Run twice over the datasheet/board extremes: slow (latest possible data,
// tests setup) and fast (earliest release of the old data, tests hold).
// Every pdm_valid must deliver exactly the L bit and R bit that period's
// mics sent.
module tb_pdm_sampler;
    localparam integer N   = 48;
    localparam real    TCK = 81.380;               // 12.288 MHz

    reg clk = 0;
    always #(TCK/2) clk = ~clk;

    reg  [1:0] pdm_div = 2'd0;
    always @(posedge clk) pdm_div <= pdm_div + 2'd1;
    wire pdm_clk = pdm_div[1];

    real CLK_DLY, TDD, THZ, RET_DLY;                // set per run below

    // Mic model: what each mic sent, in order, and the line's driven value.
    reg  [N-1:0] l_bit, r_bit;                      // current bits being driven
    reg  [N-1:0] line;
    reg  [N-1:0] l_sent [0:4095];
    reg  [N-1:0] r_sent [0:4095];
    integer l_n = 0, r_n = 0;

    wire mic_clk;
    assign #(CLK_DLY) mic_clk = pdm_clk;

    always @(negedge mic_clk) begin                 // L requested; R released
        l_bit = {$random, $random};
        l_sent[l_n] = l_bit; l_n = l_n + 1;
        line <= #(THZ) {N{1'bx}};
        line <= #(TDD) l_bit;
    end
    always @(posedge mic_clk) begin                 // R requested; L released
        r_bit = {$random, $random};
        r_sent[r_n] = r_bit; r_n = r_n + 1;
        line <= #(THZ) {N{1'bx}};
        line <= #(TDD) r_bit;
    end

    wire [N-1:0] data;
    assign #(RET_DLY) data = line;

    reg rst = 1'b1;
    wire [N-1:0] pdm_l, pdm_r;
    wire pdm_valid;
    pdm_sampler #(.N_LINES(N)) dut (
        .clk(clk), .rst(rst), .pdm_div(pdm_div), .data(data),
        .pdm_l(pdm_l), .pdm_r(pdm_r), .pdm_valid(pdm_valid)
    );

    integer errors = 0, checked = 0;
    reg checking = 1'b0;

    // When pdm_valid is seen, R was captured on the edge that launched the
    // latest fall and the latest R request (the rise before it) is its bit;
    // that fall has since reached the mics, so the paired L -- requested by
    // the fall before the R's rise -- is the second-latest L request.
    always @(posedge clk) if (checking && pdm_valid) begin
        if (pdm_r !== r_sent[r_n-1] || pdm_l !== l_sent[l_n-2]) begin
            $display("FAIL: t=%0t: L got %h want %h, R got %h want %h",
                     $time, pdm_l, l_sent[l_n-2], pdm_r, r_sent[r_n-1]);
            errors = errors + 1;
        end
        checked = checked + 1;
    end

    integer run;
    initial begin
        for (run = 0; run < 2; run = run + 1) begin
            if (run == 0) begin                     // slow: data as late as possible
                CLK_DLY = 6.2; TDD = 80.0; THZ = 30.0; RET_DLY = 1.9;
            end else begin                          // fast: old data gone earliest
                CLK_DLY = 3.1; TDD = 40.0; THZ = 5.0;  RET_DLY = 0.0;
            end
            rst = 1'b1; checking = 1'b0;
            l_n = 0; r_n = 0;
            line = {N{1'bx}};
            @(posedge clk); @(posedge clk);
            rst = 1'b0;
            repeat (4) @(posedge pdm_valid);        // let the model fill in
            @(negedge clk);
            checking = 1'b1;
            repeat (200) @(posedge pdm_valid);
            @(negedge clk);
            checking = 1'b0;
            $display("run %0d (%0s): %0d periods checked so far", run, run == 0 ? "slow" : "fast", checked);
        end

        if (errors == 0 && checked >= 400)
            $display("PASS: tb_pdm_sampler");
        else
            $display("FAIL: tb_pdm_sampler (%0d errors, %0d checked)", errors, checked);
        $finish;
    end
endmodule
