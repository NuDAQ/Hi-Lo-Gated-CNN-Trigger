`timescale 1ns / 10ps

// =============================================================================
// Copyright 2026 Albert L. Cheung @ University of California, Irvine
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// tb_thermal.sv — HILO_CNN_TRIGGER testbench driven by thermal noise data
//
// Copyright 2026 Albert L. Cheung @ University of California, Irvine
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// =============================================================================

`include "thermal_data/thermal_paths.svh"

module tb_thermal;

    parameter real ADC_CLK_PERIOD = 16.0;   // ns — 62.5 MHz
    parameter real CNN_CLK_PERIOD =  5.0;   // ns — 200 MHz

    parameter logic [11:0] P_THRESH       = 12'd192;  // 3sigma × 64  
    parameter logic [ 4:0] P_HILO_WINDOW  = 5'd5;     // 5 samples
    parameter logic [ 5:0] P_COINC_WINDOW = 6'd30;    // 30 samples 
    parameter logic [ 3:0] P_BIN_THR      = 4'd2;     // >=2 channels in coincidence

    parameter logic signed [16:0] P_CNN_THRESH = 17'sd128;

    parameter int N_CHUNKS_CAPTURE = 3;

    parameter real TIMEOUT_NS = 2_000_000.0;   // 2 ms

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk_adc = 0;
    reg  clk_cnn = 0;
    reg  rst;
    reg  data_str = 0;

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
    reg         cnn_out_ready = 1;
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

    always #(ADC_CLK_PERIOD / 2.0) clk_adc = ~clk_adc;
    always #(CNN_CLK_PERIOD / 2.0) clk_cnn = ~clk_cnn;

    reg [11:0] ring_sv [0:11][0:3][0:15];  // [slot][ch][sample], 12 slots
    reg [11:0] post_sv [0:4] [0:3][0:15];  // [post_batch][ch][sample], 5 slots

    logic        cnn_cap_valid = 0;
    logic [31:0] cnn_cap_data  = 0;

    always @(posedge clk_cnn) begin
        if (cnn_out_valid && cnn_out_ready && !cnn_cap_valid) begin
            cnn_cap_data  <= cnn_out_data;
            cnn_cap_valid <= 1;
            begin
                automatic real _s = $itor($signed(cnn_out_data[16:0])) / 256.0;
                $display("  [%0t] CNN_OUT captured: 0x%08h  score=%.4f  prob=%.4f",
                         $time, cnn_out_data, _s, 1.0 / (1.0 + $exp(-_s)));
            end
        end
    end

    logic l1_cap_valid = 0;

    always @(posedge clk_cnn) begin
        if (l1_cnn_trig && !l1_cap_valid) begin
            l1_cap_valid <= 1;
            $display("  [%0t] L1_CNN_TRIG fired (score > %.4f)",
                     $time, $itor($signed(P_CNN_THRESH)) / 256.0);
        end
    end

    integer f_wave, f_results, stim_fd;
    integer trig_count;
    integer overflow_cnt = 0;

    always @(posedge chunk_overflow) overflow_cnt++;

    initial begin : main_stim
        int c, s, p, slot;

        f_wave    = $fopen(`CHUNK_WAVE_CSV,  "w");
        f_results = $fopen(`CNN_RESULTS_TXT, "w");
        if (f_wave == 0 || f_results == 0) begin
            $display("[ERROR] Cannot open output files. Check thermal_data/ is writable.");
            $finish;
        end
        $fwrite(f_wave,
            "# chunk_id,sample_idx,ch0,ch1,ch2,ch3\n");
        $fwrite(f_results,
            "# chunk_id,l0_time_ns,cnn_fired,cnn_raw_hex,cnn_score_float,l1_cnn_trig,chunk_overflow\n");

        stim_fd = $fopen(`STIMULUS_TXT, "r");
        if (stim_fd == 0) begin
            $display("[ERROR] Cannot open stimulus file: `STIMULUS_TXT");
            $display("        Run: python3 scripts/prepare_thermal_sim.py --data-dir <path>");
            $finish;
        end

        rst      = 1;
        data_str = 0;
        for (c = 0; c < 4; c++)
            for (s = 0; s < 16; s++)
                adc_ch[c][s] = 12'h000;
        repeat(10) @(posedge clk_adc);
        rst = 0;
        repeat(3) @(posedge clk_adc);

        $display("[%0t] Reset released. Streaming thermal noise stimulus.", $time);
        $display("  THRESH=%0d  HILO_WIN=%0d  COINC_WIN=%0d  BIN_THR=%0d  CNN_THRESH=%0d (score>%.4f)",
                 P_THRESH, P_HILO_WINDOW, P_COINC_WINDOW, P_BIN_THR,
                 $signed(P_CNN_THRESH), $itor($signed(P_CNN_THRESH)) / 256.0);

        trig_count = 0;

        while (!$feof(stim_fd) && trig_count < N_CHUNKS_CAPTURE) begin

            for (c = 0; c < 4; c++)
                for (s = 0; s < 16; s++)
                    void'($fscanf(stim_fd, "%d", adc_ch[c][s]));

            for (slot = 11; slot > 0; slot--)
                for (c = 0; c < 4; c++)
                    for (s = 0; s < 16; s++)
                        ring_sv[slot][c][s] = ring_sv[slot-1][c][s];
            for (c = 0; c < 4; c++)
                for (s = 0; s < 16; s++)
                    ring_sv[0][c][s] = adc_ch[c][s];

            data_str = 1;
            @(posedge clk_adc);

            if (l0_pre_trig) begin
                real l0_time;
                l0_time = $realtime;
                $display("\n=== [%0t] L0_PRE_TRIG — chunk %0d / %0d ===",
                         $time, trig_count, N_CHUNKS_CAPTURE);

                cnn_cap_valid = 0;
                cnn_cap_data  = 0;
                l1_cap_valid  = 0;

                if (!$feof(stim_fd)) begin
                    for (c = 0; c < 4; c++)
                        for (s = 0; s < 16; s++)
                            void'($fscanf(stim_fd, "%d", adc_ch[c][s]));
                    for (slot = 11; slot > 0; slot--)
                        for (c = 0; c < 4; c++)
                            for (s = 0; s < 16; s++)
                                ring_sv[slot][c][s] = ring_sv[slot-1][c][s];
                    for (c = 0; c < 4; c++)
                        for (s = 0; s < 16; s++)
                            ring_sv[0][c][s] = adc_ch[c][s];
                end
                @(posedge clk_adc);  // hardware: ADC_IDLE→ADC_POST at this edge

                for (p = 0; p < 5; p++) begin
                    if (!$feof(stim_fd)) begin
                        for (c = 0; c < 4; c++)
                            for (s = 0; s < 16; s++)
                                void'($fscanf(stim_fd, "%d", adc_ch[c][s]));
                        // Store first 4 post batches for logging; 5th just drives HW
                        if (p < 4)
                            for (c = 0; c < 4; c++)
                                for (s = 0; s < 16; s++)
                                    post_sv[p][c][s] = adc_ch[c][s];
                    end
                    @(posedge clk_adc);
                end
                data_str = 0;

                for (slot = 11; slot >= 4; slot--) begin
                    for (s = 0; s < 16; s++) begin
                        automatic int samp_idx = (11 - slot) * 16 + s;
                        $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                                trig_count, samp_idx,
                                $signed(ring_sv[slot][0][s]),
                                $signed(ring_sv[slot][1][s]),
                                $signed(ring_sv[slot][2][s]),
                                $signed(ring_sv[slot][3][s]));
                    end
                end
                for (slot = 3; slot >= 0; slot--) begin
                    for (s = 0; s < 16; s++) begin
                        automatic int samp_idx = 128 + (3 - slot) * 16 + s;
                        $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                                trig_count, samp_idx,
                                $signed(ring_sv[slot][0][s]),
                                $signed(ring_sv[slot][1][s]),
                                $signed(ring_sv[slot][2][s]),
                                $signed(ring_sv[slot][3][s]));
                    end
                end
                for (p = 0; p < 4; p++) begin
                    for (s = 0; s < 16; s++) begin
                        automatic int samp_idx = 192 + p * 16 + s;
                        $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                                trig_count, samp_idx,
                                $signed(post_sv[p][0][s]),
                                $signed(post_sv[p][1][s]),
                                $signed(post_sv[p][2][s]),
                                $signed(post_sv[p][3][s]));
                    end
                end
                $fflush(f_wave);

                fork : cnn_wait
                    begin : cnn_branch
                        wait(cnn_cap_valid);
                    end
                    begin : timeout_branch
                        #(TIMEOUT_NS);
                        $display("  [%0t] TIMEOUT — CNN did not respond within %.0f µs.",
                                 $time, TIMEOUT_NS / 1000.0);
                    end
                join_any
                disable cnn_wait;

                begin
                    automatic real cnn_score =
                        $itor($signed(cnn_cap_data[16:0])) / 256.0;
                    automatic real cnn_prob =
                        cnn_cap_valid ? 1.0 / (1.0 + $exp(-cnn_score)) : 0.0;
                    $fwrite(f_results, "%0d,%.1f,%0d,0x%08h,%.6f,%0d,%0d\n",
                            trig_count, l0_time,
                            cnn_cap_valid ? 1 : 0,
                            cnn_cap_data, cnn_score,
                            l1_cap_valid ? 1 : 0,
                            chunk_overflow);
                    $fflush(f_results);
                    $display("  CNN prob  = %.4f  (cnn_fired=%0d)",
                             cnn_prob, cnn_cap_valid ? 1 : 0);
                end

                trig_count++;

                // Resume streaming (re-assert data_str)
                if (trig_count < N_CHUNKS_CAPTURE)
                    data_str = 1;
            end
        end

        data_str = 0;
        $fclose(stim_fd);
        $fclose(f_wave);
        $fclose(f_results);

        repeat(500) @(posedge clk_cnn);

        $display("\n[%0t] Thermal sim complete.", $time);
        $display("  Chunks captured : %0d / %0d", trig_count, N_CHUNKS_CAPTURE);
        $display("  CHUNK_OVERFLOW  : %0d", overflow_cnt);
        $finish;
    end

endmodule
