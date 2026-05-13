`timescale 1ns / 10ps

// =============================================================================
// tb_thermal.sv — HILO_CNN_TRIGGER testbench driven by thermal noise data
//
// Data source: hilo-trigger thermal noise dataset (σ-unit .npy files),
// pre-converted to ADC-count stimulus by scripts/prepare_thermal_sim.py.
//
// Stimulus format (stimulus.txt):
//   One line per 16-sample batch.
//   Each line: 64 space-separated integers — ch0×16, ch1×16, ch2×16, ch3×16.
//   Events are concatenated; no inter-event resets (continuous stream).
//
// Behaviour:
//   Batches are streamed one per CLK_ADC cycle (DATA_STR=1 every cycle).
//   A 8-slot software ring buffer mirrors CNN_CHUNK_CAPTURE's ring_buf.
//   When L0_PRE_TRIG asserts, the testbench:
//     1. Snapshots the 8-slot ring (pre-trigger, 128 samples).
//     2. Drives 8 more batches as post-trigger (128 samples).
//     3. Writes the 256-sample chunk waveform to CHUNK_WAVE_CSV.
//     4. Waits for CNN_OUT_VALID (or timeout) and logs the score.
//   After N_CHUNKS_CAPTURE chunks the simulation terminates.
//
// Thermal noise events are NOT neutrino signals; CNN scores will be negative
// (score ≤ 0.5 after normalisation).  The testbench does not require a
// positive score — it merely records whatever the CNN outputs.
//
// Absolute paths are injected by run_thermal_sim.sh into:
//   hw/sim/thermal_data/thermal_paths.svh
// Defines provided:
//   `STIMULUS_TXT   `CHUNK_WAVE_CSV   `CNN_RESULTS_TXT
//
// Run via: bash scripts/run_thermal_sim.sh [options]
// =============================================================================

`include "thermal_data/thermal_paths.svh"

module tb_thermal;

    // -------------------------------------------------------------------------
    // Clock parameters — CLK_ADC = 62.5 MHz (16-sample batches from 1 GHz ADC)
    // -------------------------------------------------------------------------
    parameter real ADC_CLK_PERIOD = 16.0;   // ns — 62.5 MHz
    parameter real CNN_CLK_PERIOD =  5.0;   // ns — 200 MHz

    // -------------------------------------------------------------------------
    // Hi-Lo configuration
    // Match hilo-trigger's cosim defaults; P_THRESH overridden from stim_meta.txt
    // at elaboration time or left at 3σ default (192 counts when σ=64).
    // -------------------------------------------------------------------------
    parameter logic [11:0] P_THRESH       = 12'd192;  // 3σ × 64  (update from stim_meta)
    parameter logic [ 4:0] P_HILO_WINDOW  = 5'd5;     // 5 samples
    parameter logic [ 5:0] P_COINC_WINDOW = 6'd16;    // 16 samples (= N_SAMPLES, max effective)
    parameter logic [ 3:0] P_BIN_THR      = 4'd1;     // single-channel threshold

    // Stop simulation after this many L0-triggered CNN outputs
    parameter int N_CHUNKS_CAPTURE = 3;

    // Per-chunk CNN timeout (ns).  CNN inference ~25 µs @ 200 MHz.
    parameter real TIMEOUT_NS = 2_000_000.0;   // 2 ms

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk_adc = 0;
    reg  clk_cnn = 0;
    reg  rst;
    reg  data_str = 0;

    // ADC data: 4 ch × 16 samples × 12-bit
    reg [11:0] adc_ch [0:3][0:15];

    // Flat 768-bit vector: 4 ch × 16 samples × 12 bits, MSB-first, ch0 at low end
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
    wire        chunk_overflow;

    // -------------------------------------------------------------------------
    // DUT instantiation
    // -------------------------------------------------------------------------
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
        .CHUNK_OVERFLOW (chunk_overflow)
    );

    always #(ADC_CLK_PERIOD / 2.0) clk_adc = ~clk_adc;
    always #(CNN_CLK_PERIOD / 2.0) clk_cnn = ~clk_cnn;

    // -------------------------------------------------------------------------
    // Software ring buffer mirror — 8 slots, matches CNN_CHUNK_CAPTURE ring_buf
    //   ring_sv[0] = most recent batch  (= ring_buf(0) in VHDL)
    //   ring_sv[7] = oldest batch       (= ring_buf(7) in VHDL)
    // Post-trigger capture buffer
    // -------------------------------------------------------------------------
    reg [11:0] ring_sv [0:7][0:3][0:15];  // [slot][ch][sample]
    reg [11:0] post_sv [0:7][0:3][0:15];  // [post_batch][ch][sample]

    // -------------------------------------------------------------------------
    // Async CNN output capture
    // Separate always block so CNN results are captured even if they arrive
    // during post-trigger batch driving or timeout waiting.
    // -------------------------------------------------------------------------
    logic        cnn_cap_valid = 0;
    logic [31:0] cnn_cap_data  = 0;

    always @(posedge clk_cnn) begin
        if (cnn_out_valid && cnn_out_ready && !cnn_cap_valid) begin
            cnn_cap_data  <= cnn_out_data;
            cnn_cap_valid <= 1;
            $display("  [%0t] CNN_OUT captured: 0x%08h  score=%.4f",
                     $time, cnn_out_data,
                     $itor($signed(cnn_out_data[16:0])) / 256.0);
        end
    end

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    integer f_wave, f_results, stim_fd;
    integer trig_count;
    integer overflow_cnt = 0;

    always @(posedge chunk_overflow) overflow_cnt++;

    initial begin : main_stim
        int c, s, p, slot;

        // ------------------------------------------------------------------
        // Open output files
        // ------------------------------------------------------------------
        f_wave    = $fopen(`CHUNK_WAVE_CSV,  "w");
        f_results = $fopen(`CNN_RESULTS_TXT, "w");
        if (f_wave == 0 || f_results == 0) begin
            $display("[ERROR] Cannot open output files. Check thermal_data/ is writable.");
            $finish;
        end
        $fwrite(f_wave,
            "# chunk_id,sample_idx,ch0,ch1,ch2,ch3\n");
        $fwrite(f_results,
            "# chunk_id,l0_time_ns,cnn_fired,cnn_raw_hex,cnn_score_float,chunk_overflow\n");

        // ------------------------------------------------------------------
        // Open stimulus
        // ------------------------------------------------------------------
        stim_fd = $fopen(`STIMULUS_TXT, "r");
        if (stim_fd == 0) begin
            $display("[ERROR] Cannot open stimulus file: `STIMULUS_TXT");
            $display("        Run: python3 scripts/prepare_thermal_sim.py --data-dir <path>");
            $finish;
        end

        // ------------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------------
        rst      = 1;
        data_str = 0;
        for (c = 0; c < 4; c++)
            for (s = 0; s < 16; s++)
                adc_ch[c][s] = 12'h000;
        repeat(10) @(posedge clk_adc);
        rst = 0;
        repeat(3) @(posedge clk_adc);

        $display("[%0t] Reset released. Streaming thermal noise stimulus.", $time);
        $display("  THRESH=%0d  HILO_WIN=%0d  COINC_WIN=%0d  BIN_THR=%0d",
                 P_THRESH, P_HILO_WINDOW, P_COINC_WINDOW, P_BIN_THR);

        // ------------------------------------------------------------------
        // Main streaming loop
        // Drive one batch per CLK_ADC cycle until N_CHUNKS_CAPTURE triggers.
        // ------------------------------------------------------------------
        trig_count = 0;

        while (!$feof(stim_fd) && trig_count < N_CHUNKS_CAPTURE) begin

            // --- Read next batch (64 integers: ch0×16, ch1×16, ch2×16, ch3×16) ---
            for (c = 0; c < 4; c++)
                for (s = 0; s < 16; s++)
                    void'($fscanf(stim_fd, "%d", adc_ch[c][s]));

            // --- Update software ring mirror (shift oldest out, newest in) ---
            for (slot = 7; slot > 0; slot--)
                for (c = 0; c < 4; c++)
                    for (s = 0; s < 16; s++)
                        ring_sv[slot][c][s] = ring_sv[slot-1][c][s];
            for (c = 0; c < 4; c++)
                for (s = 0; s < 16; s++)
                    ring_sv[0][c][s] = adc_ch[c][s];

            // --- Drive one clock cycle ---
            data_str = 1;
            @(posedge clk_adc);
            // Data_str stays 1; we keep it high for the next batch too (continuous stream).

            // --- Did L0 fire? (registered VHDL output, stable after posedge) ---
            if (l0_pre_trig) begin
                real l0_time;
                l0_time = $realtime;
                $display("\n=== [%0t] L0_PRE_TRIG — chunk %0d / %0d ===",
                         $time, trig_count, N_CHUNKS_CAPTURE);

                // Reset CNN capture flag for this chunk
                cnn_cap_valid = 0;
                cnn_cap_data  = 0;

                // ----------------------------------------------------------
                // Capture 8 post-trigger batches.
                // The hardware CNN_CHUNK_CAPTURE FSM is now in ADC_POST,
                // waiting for 8 DATA_STR pulses.
                // ----------------------------------------------------------
                for (p = 0; p < 8; p++) begin
                    if (!$feof(stim_fd)) begin
                        for (c = 0; c < 4; c++)
                            for (s = 0; s < 16; s++)
                                void'($fscanf(stim_fd, "%d", adc_ch[c][s]));
                        // Store post batch in software mirror
                        for (c = 0; c < 4; c++)
                            for (s = 0; s < 16; s++)
                                post_sv[p][c][s] = adc_ch[c][s];
                    end
                    @(posedge clk_adc);
                end
                data_str = 0;

                // ----------------------------------------------------------
                // Write waveform CSV:
                //   Pre-trigger:  ring_sv[7] (oldest) .. ring_sv[0] (newest)
                //                 = sample indices 0 .. 127
                //   Post-trigger: post_sv[0] .. post_sv[7]
                //                 = sample indices 128 .. 255
                // This mirrors the BRAM write order in CNN_CHUNK_CAPTURE.
                // ----------------------------------------------------------
                for (slot = 7; slot >= 0; slot--) begin
                    for (s = 0; s < 16; s++) begin
                        automatic int samp_idx = (7 - slot) * 16 + s;
                        $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                                trig_count, samp_idx,
                                $signed(ring_sv[slot][0][s]),
                                $signed(ring_sv[slot][1][s]),
                                $signed(ring_sv[slot][2][s]),
                                $signed(ring_sv[slot][3][s]));
                    end
                end
                for (p = 0; p < 8; p++) begin
                    for (s = 0; s < 16; s++) begin
                        automatic int samp_idx = 128 + p * 16 + s;
                        $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                                trig_count, samp_idx,
                                $signed(post_sv[p][0][s]),
                                $signed(post_sv[p][1][s]),
                                $signed(post_sv[p][2][s]),
                                $signed(post_sv[p][3][s]));
                    end
                end
                $fflush(f_wave);

                // ----------------------------------------------------------
                // Wait for CNN output or timeout
                // ----------------------------------------------------------
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
                    $fwrite(f_results, "%0d,%.1f,%0d,0x%08h,%.6f,%0d\n",
                            trig_count, l0_time,
                            cnn_cap_valid ? 1 : 0,
                            cnn_cap_data, cnn_score,
                            chunk_overflow);
                    $fflush(f_results);
                    $display("  CNN score = %.4f  (cnn_fired=%0d)",
                             cnn_score, cnn_cap_valid ? 1 : 0);
                end

                trig_count++;

                // Resume streaming (re-assert data_str)
                if (trig_count < N_CHUNKS_CAPTURE)
                    data_str = 1;
            end
        end

        // ------------------------------------------------------------------
        // Wrap up
        // ------------------------------------------------------------------
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
