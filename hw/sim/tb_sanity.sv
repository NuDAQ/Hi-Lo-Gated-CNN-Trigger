`timescale 1ns / 10ps

// =============================================================================
// Copyright 2026 Albert L. Cheung @ University of California, Irvine
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

// =============================================================================

// Absolute paths are injected by the shell script into:
//   hw/sim/sanity_data/sanity_paths.svh
// Defines provided:
//   `CHUNK_SIG0  `CHUNK_SIG1  `CHUNK_SIG2  `CHUNK_NOISE0
//   `WAVE_CSV    `RESULTS_TXT
`include "sanity_data/sanity_paths.svh"

module tb_sanity;

    parameter real ADC_CLK_PERIOD = 32.0;    // ns  (31.25 MHz)
    parameter real CNN_CLK_PERIOD =  5.0;    // ns  (200   MHz)

    parameter logic [11:0] P_THRESH       = 12'd300;  // ADC counts (~4.7σ)
    parameter logic [ 4:0] P_HILO_WINDOW  = 5'd10;
    parameter logic [ 5:0] P_COINC_WINDOW = 6'd20;
    parameter logic [ 3:0] P_BIN_THR      = 4'd1;     // single-channel, easiest

    parameter logic signed [16:0] P_CNN_THRESH = 17'sd128;

    parameter int N_PRIME = 10;

    parameter real TIMEOUT_NS = 600_000.0;   // 600 µs

    parameter int CNN_SCORE_THRESH = 128;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk_adc = 0;
    reg  clk_cnn = 0;
    reg  rst;
    reg  data_str;

    // ADC channels: adc_ch[channel][sample_in_batch]
    reg [11:0] adc_ch [0:3][0:15];

    wire [767:0] adc_data4_flat;
    genvar gi, gj;
    generate
        for (gi = 0; gi < 4; gi++) begin : gen_ch
            for (gj = 0; gj < 16; gj++) begin : gen_s
                assign adc_data4_flat[767 - (gi*16 + gj)*12 -: 12] = adc_ch[gi][gj];
            end
        end
    endgenerate

    wire        l0_pre_trig;
    wire [31:0] cnn_out_data;
    wire        cnn_out_valid;
    reg         cnn_out_ready;
    wire        l1_cnn_trig;
    wire        chunk_overflow;

    HILO_CNN_TRIGGER_TB_WRAP uut (
        .CLK_ADC        (clk_adc),
        .CLK_CNN        (clk_cnn),
        .RST            (rst),
        .DATA_STR       (data_str),
        .ADC_DATA4_FLAT (adc_data4_flat),
        .THRESH         (P_THRESH),
        .HILO_WINDOW    (P_HILO_WINDOW),
        .COINC_WINDOW   (P_COINC_WINDOW),
        .BIN_THR        (P_BIN_THR),
        .L0_PRE_TRIG    (l0_pre_trig),
        .CNN_OUT_DATA   (cnn_out_data),
        .CNN_OUT_VALID  (cnn_out_valid),
        .CNN_OUT_READY  (cnn_out_ready),
        .CNN_THRESH     (P_CNN_THRESH),
        .L1_CNN_TRIG    (l1_cnn_trig),
        .CHUNK_OVERFLOW (chunk_overflow)
    );

    // -------------------------------------------------------------------------
    // Clocks
    // -------------------------------------------------------------------------
    always #(ADC_CLK_PERIOD / 2.0) clk_adc = ~clk_adc;
    always #(CNN_CLK_PERIOD / 2.0) clk_cnn = ~clk_cnn;

    logic [63:0] chunk_mem [0:3][0:255];

    integer f_wave, f_results;

    function automatic logic [11:0] extract_ch;
        input logic [63:0] word;
        input int          ch;
        logic [15:0] slot16;
        begin
            slot16 = word[ch*16 +: 16];
            extract_ch = slot16[11:0];   // lower 12 bits = ADC value
        end
    endfunction

    task automatic drive_event_batch;
        input int ev_id;
        input int batch;
        integer s;
        logic [63:0] word;
        begin
            for (s = 0; s < 16; s++) begin
                word = chunk_mem[ev_id][batch * 16 + s];
                adc_ch[0][s] = extract_ch(word, 0);
                adc_ch[1][s] = extract_ch(word, 1);
                adc_ch[2][s] = extract_ch(word, 2);
                adc_ch[3][s] = extract_ch(word, 3);
            end
        end
    endtask

    task automatic drive_zero_batch;
        integer s, ch;
        begin
            for (ch = 0; ch < 4; ch++)
                for (s = 0; s < 16; s++)
                    adc_ch[ch][s] = 12'h000;
        end
    endtask

    logic        l0_cap_valid = 0;   // set on first posedge of l0_pre_trig
    real         l0_cap_time  = -1.0;

    logic [31:0] cnn_cap_data  = 0;
    logic        cnn_cap_valid = 0;   // set on first CNN output handshake

    // L0 capture — runs independently of the main initial block
    always @(posedge l0_pre_trig) begin
        if (!l0_cap_valid) begin
            l0_cap_valid = 1;           // blocking: visible in same time step
            l0_cap_time  = $realtime;
            $display("  [%0t] ** L0_PRE_TRIG captured (async) **", $time);
        end
    end

    // CNN output capture — on CNN clock
    always @(posedge clk_cnn) begin
        if (cnn_out_valid && cnn_out_ready && !cnn_cap_valid) begin
            cnn_cap_data  = cnn_out_data;
            cnn_cap_valid = 1;
            $display("  [%0t] ** CNN_OUT captured: 0x%08h  score=%.4f **",
                     $time, cnn_out_data,
                     $itor($signed(cnn_out_data[16:0])) / 256.0);
        end
    end

    // L1 CNN trigger capture — fires 1 CLK_CNN cycle after CNN_OUT handshake
    logic l1_cap_valid = 0;

    always @(posedge clk_cnn) begin
        if (l1_cnn_trig && !l1_cap_valid) begin
            l1_cap_valid = 1;
            $display("  [%0t] ** L1_CNN_TRIG captured (score > %.4f) **",
                     $time, $itor($signed(P_CNN_THRESH)) / 256.0);
        end
    end

    // -------------------------------------------------------------------------
    // Main test
    // -------------------------------------------------------------------------
    string chunk_type [0:3] = '{"sig", "sig", "sig", "noise"};
    integer pass_count, fail_count;
    real    ev_start_time_ns;

    initial begin : main_test
        int ev, b, s;
        int pass;
        real cnn_score;

        $readmemh(`CHUNK_SIG0,   chunk_mem[0]);
        $readmemh(`CHUNK_SIG1,   chunk_mem[1]);
        $readmemh(`CHUNK_SIG2,   chunk_mem[2]);
        $readmemh(`CHUNK_NOISE0, chunk_mem[3]);

        if (chunk_mem[0][0] === 64'bx) begin
            $display("[ERROR] Failed to load chunk_sig0.hex.");
            $display("        Run: python3 scripts/prepare_sanity_chunks.py");
            $finish;
        end

        f_wave    = $fopen(`WAVE_CSV,    "w");
        f_results = $fopen(`RESULTS_TXT, "w");
        if (f_wave == 0 || f_results == 0) begin
            $display("[ERROR] Cannot open output log files — check sanity_data/ is writable.");
            $finish;
        end
        $fwrite(f_wave,    "# ev_id,sample_idx,ch0,ch1,ch2,ch3\n");
        $fwrite(f_results, "# ev_id,type,l0_fired,l0_time_ns,ev_start_ns,cnn_fired,cnn_raw_hex,cnn_score_float,l1_cnn_trig,pass\n");

        rst           = 1;
        data_str      = 0;
        cnn_out_ready = 1;
        drive_zero_batch();
        repeat(20) @(posedge clk_adc);
        rst      = 0;
        repeat(5) @(posedge clk_adc);
        data_str = 1;
        $display("[%0t] Reset released. Starting sanity test.", $time);
        $display("  THRESH=%0d  HILO_WIN=%0d  COINC_WIN=%0d  BIN_THR=%0d  N_PRIME=%0d  CNN_THRESH=%0d (score>%.4f)",
                 P_THRESH, P_HILO_WINDOW, P_COINC_WINDOW, P_BIN_THR, N_PRIME,
                 $signed(P_CNN_THRESH), $itor($signed(P_CNN_THRESH)) / 256.0);

        pass_count = 0;
        fail_count = 0;

        for (ev = 0; ev < 4; ev++) begin
            $display("\n=== Event %0d / 4  [%s] ===", ev, chunk_type[ev]);

            l0_cap_valid = 0;
            l0_cap_time  = -1.0;
            cnn_cap_valid = 0;
            cnn_cap_data  = 0;
            l1_cap_valid  = 0;

            for (b = 0; b < N_PRIME; b++) begin
                @(negedge clk_adc);
                drive_zero_batch();
                @(posedge clk_adc);
            end

            ev_start_time_ns = $realtime;

            for (b = 0; b < 16; b++) begin
                @(negedge clk_adc);
                drive_event_batch(ev, b);
                for (s = 0; s < 16; s++)
                    $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                            ev, b*16+s,
                            $signed(adc_ch[0][s]), $signed(adc_ch[1][s]),
                            $signed(adc_ch[2][s]), $signed(adc_ch[3][s]));
                @(posedge clk_adc);
            end

            @(negedge clk_adc);
            drive_zero_batch();

            fork : wait_fork
                begin : wait_branch
                    wait(l0_cap_valid);   // returns immediately if already set
                    $display("  [%0t] L0 captured at %.0f ns  (%.0f ns after ev start)",
                             $time, l0_cap_time, l0_cap_time - ev_start_time_ns);
                    wait(cnn_cap_valid);  // then wait for CNN result
                end
                begin : timeout_branch
                    #(TIMEOUT_NS);
                    if (!l0_cap_valid)
                        $display("  [%0t] TIMEOUT — L0 did not fire within %.0f µs.",
                                 $time, TIMEOUT_NS / 1000.0);
                    else
                        $display("  [%0t] TIMEOUT — CNN result not received within %.0f µs.",
                                 $time, TIMEOUT_NS / 1000.0);
                end
            join_any
            disable wait_fork;

            repeat(200) @(posedge clk_cnn);

            cnn_score = $itor($signed(cnn_cap_data[16:0])) / 256.0;

            if (chunk_type[ev] == "sig") begin
                pass = l0_cap_valid && l1_cap_valid;
                $display("  Signal check: l0=%0d  l1=%0d  score=%.4f  -> %s",
                         l0_cap_valid, l1_cap_valid, cnn_score, pass ? "PASS" : "FAIL");
            end else begin
                if (!l0_cap_valid) begin
                    pass = 1;
                    $display("  Noise check:  l0=0 (Hi-Lo did not fire)  -> PASS");
                end else if (!cnn_cap_valid) begin
                    pass = 0;
                    $display("  Noise check:  l0=1  cnn=TIMEOUT  -> FAIL");
                end else begin
                    pass = !l1_cap_valid;
                    $display("  Noise check:  l0=1  l1=%0d  score=%.4f (L1 must not fire)  -> %s",
                             l1_cap_valid, cnn_score, pass ? "PASS" : "FAIL");
                end
            end

            if (pass) pass_count++;
            else       fail_count++;

            $fwrite(f_results,
                    "%0d,%s,%0d,%.1f,%.1f,%0d,0x%08h,%.6f,%0d,%0d\n",
                    ev, chunk_type[ev],
                    l0_cap_valid, l0_cap_time, ev_start_time_ns,
                    cnn_cap_valid, cnn_cap_data, cnn_score,
                    l1_cap_valid, pass);
        end

        $display("\n========================================");
        $display("  SANITY TEST COMPLETE");
        $display("  Passed: %0d / 4", pass_count);
        $display("  Failed: %0d / 4", fail_count);
        $display("  CHUNK_OVERFLOW: %0d", chunk_overflow);
        $display("========================================\n");

        $fclose(f_wave);
        $fclose(f_results);

        $display(fail_count == 0 ? "[RESULT] ALL PASS" :
                 "[RESULT] %0d FAILURES — check sanity_results.txt", fail_count);
        $finish;
    end

endmodule
