`timescale 1ns / 10ps

// =============================================================================
// tb_sanity.sv — Functional sanity-check for HILO_CNN_TRIGGER
//
// Tests 4 pre-computed chunks (3 signal, 1 noise) and verifies:
//   Signal: L0_PRE_TRIG must fire  AND  CNN score > 0.5
//   Noise:  L0_PRE_TRIG must NOT fire
//
// Data files:  hw/sim/sanity_data/chunk_sig{0,1,2}.hex
//              hw/sim/sanity_data/chunk_noise0.hex
//   Each file: 256 lines of 64-bit hex.
//   Line k represents timestep k.
//   Bit layout: [ch3(16)] [ch2(16)] [ch1(16)] [ch0(16)]
//   Lower 12 bits of each 16-bit slot = signed 12-bit ADC value.
//
// Output files (for plot_sanity_results.py):
//   hw/sim/sanity_data/sanity_wave.csv    — ADC waveforms (256 samples / event)
//   hw/sim/sanity_data/sanity_results.txt — per-event trigger / CNN result
//
// Clocks:
//   CLK_ADC = 31.25 MHz (32 ns period)  — ADC batch processing
//   CLK_CNN = 200   MHz ( 5 ns period)  — CNN inference
//
// Before the event data, N_PRIME dummy batches (all zeros) are fed to prime
// the ring buffer. After N_PRIME + 8 batches per event the testbench waits
// for CNN completion (or TIMEOUT_US).
//
// Run via scripts/run_sanity_sim.sh (generates sanity_paths.svh first).
// =============================================================================

// Absolute paths are injected by the shell script into:
//   hw/sim/sanity_data/sanity_paths.svh
// Defines provided:
//   `CHUNK_SIG0  `CHUNK_SIG1  `CHUNK_SIG2  `CHUNK_NOISE0
//   `WAVE_CSV    `RESULTS_TXT
`include "sanity_data/sanity_paths.svh"

module tb_sanity;

    // -------------------------------------------------------------------------
    // Parameters — override via +define+ on the simulator command line
    // -------------------------------------------------------------------------
    parameter real ADC_CLK_PERIOD = 32.0;    // ns  (31.25 MHz)
    parameter real CNN_CLK_PERIOD =  5.0;    // ns  (200   MHz)

    // Hi-Lo trigger configuration (must match prepare_sanity_chunks.py --thresh)
    parameter logic [11:0] P_THRESH       = 12'd300;  // ADC counts (~4.7σ)
    parameter logic [ 4:0] P_HILO_WINDOW  = 5'd10;
    parameter logic [ 5:0] P_COINC_WINDOW = 6'd20;
    parameter logic [ 3:0] P_BIN_THR      = 4'd1;     // single-channel, easiest

    // Number of dummy (zero) batches to feed before each event chunk.
    // This primes the 4-batch ring buffer inside CNN_CHUNK_CAPTURE.
    parameter int N_PRIME = 6;

    // Per-event timeout (ns).  CNN inference typically takes ~25 µs.
    parameter real TIMEOUT_NS = 600_000.0;   // 600 µs

    // CNN decision threshold: score = $signed(output_data[16:0]) / 256.0 > 0.5
    // In raw integer: $signed(output_data[16:0]) > 128
    parameter int CNN_SCORE_THRESH = 128;

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk_adc = 0;
    reg  clk_cnn = 0;
    reg  rst;
    reg  data_str;

    // ADC channels: adc_ch[channel][sample_in_batch]
    reg [11:0] adc_ch [0:3][0:31];

    // Flat 1536-bit vector: 4 ch × 32 samples × 12 bits (MSB-first, ch0 lowest)
    wire [1535:0] adc_data4_flat;
    genvar gi, gj;
    generate
        for (gi = 0; gi < 4; gi++) begin : gen_ch
            for (gj = 0; gj < 32; gj++) begin : gen_s
                assign adc_data4_flat[1535 - (gi*32 + gj)*12 -: 12] = adc_ch[gi][gj];
            end
        end
    endgenerate

    wire        l0_pre_trig;
    wire [31:0] cnn_out_data;
    wire        cnn_out_valid;
    reg         cnn_out_ready;
    wire        chunk_overflow;

    // -------------------------------------------------------------------------
    // DUT — mixed-language bridge to HILO_CNN_TRIGGER (VHDL)
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

    // -------------------------------------------------------------------------
    // Clocks
    // -------------------------------------------------------------------------
    always #(ADC_CLK_PERIOD / 2.0) clk_adc = ~clk_adc;
    always #(CNN_CLK_PERIOD / 2.0) clk_cnn = ~clk_cnn;

    // -------------------------------------------------------------------------
    // Chunk memory: 4 events × 256 timesteps × 64-bit
    // -------------------------------------------------------------------------
    logic [63:0] chunk_mem [0:3][0:255];

    // -------------------------------------------------------------------------
    // Log file handles
    // -------------------------------------------------------------------------
    integer f_wave, f_results;

    // -------------------------------------------------------------------------
    // Helper: extract signed 12-bit ADC value for channel 'ch' from 64-bit word
    // Bit layout: ch0=[11:0], ch1=[27:16], ch2=[43:32], ch3=[59:48]
    // -------------------------------------------------------------------------
    function automatic logic [11:0] extract_ch;
        input logic [63:0] word;
        input int          ch;
        logic [15:0] slot16;
        begin
            slot16 = word[ch*16 +: 16];
            extract_ch = slot16[11:0];   // lower 12 bits = ADC value
        end
    endfunction

    // -------------------------------------------------------------------------
    // Helper: drive one batch from chunk memory (called at negedge clk_adc)
    // ev_id : which of the 4 events (0..3)
    // batch : which 32-sample batch within the event (0..7)
    // -------------------------------------------------------------------------
    task automatic drive_event_batch;
        input int ev_id;
        input int batch;
        integer s;
        logic [63:0] word;
        begin
            for (s = 0; s < 32; s++) begin
                word = chunk_mem[ev_id][batch * 32 + s];
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
                for (s = 0; s < 32; s++)
                    adc_ch[ch][s] = 12'h000;
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test
    // -------------------------------------------------------------------------
    // Chunk types for logging (0-2 = signal, 3 = noise)
    string chunk_type [0:3] = '{"sig", "sig", "sig", "noise"};

    integer pass_count, fail_count;
    real    ev_start_time_ns;
    integer l0_fired;
    real    l0_time_ns;
    integer cnn_fired;
    logic [31:0] cnn_raw;

    initial begin : main_test
        int ev, b, s, ch;
        string fpath;

        // ------------------------------------------------------------------
        // Load chunk hex files  (paths from sanity_paths.svh)
        // ------------------------------------------------------------------
        $readmemh(`CHUNK_SIG0,   chunk_mem[0]);
        $readmemh(`CHUNK_SIG1,   chunk_mem[1]);
        $readmemh(`CHUNK_SIG2,   chunk_mem[2]);
        $readmemh(`CHUNK_NOISE0, chunk_mem[3]);

        // Verify load
        if (chunk_mem[0][0] === 64'bx) begin
            $display("[ERROR] Failed to load chunk_sig0.hex.");
            $display("        Run: python3 scripts/prepare_sanity_chunks.py");
            $finish;
        end

        // ------------------------------------------------------------------
        // Open output log files
        // ------------------------------------------------------------------
        f_wave    = $fopen(`WAVE_CSV,    "w");
        f_results = $fopen(`RESULTS_TXT, "w");
        if (f_wave == 0 || f_results == 0) begin
            $display("[ERROR] Cannot open output log files.");
            $display("        Check that sanity_data/ directory exists and is writable.");
            $finish;
        end
        $fwrite(f_wave, "# ev_id,sample_idx,ch0,ch1,ch2,ch3\n");
        $fwrite(f_results,
                "# ev_id,type,l0_fired,l0_time_ns,ev_start_ns,"
                "cnn_fired,cnn_raw_hex,cnn_score_float,pass\n");

        // ------------------------------------------------------------------
        // Reset
        // ------------------------------------------------------------------
        rst           = 1;
        data_str      = 0;
        cnn_out_ready = 1;
        drive_zero_batch();
        repeat(20) @(posedge clk_adc);
        rst      = 0;
        repeat(5) @(posedge clk_adc);
        data_str = 1;
        $display("[%0t] Reset released. Starting sanity test.", $time);
        $display("  THRESH=%0d  HILO_WIN=%0d  COINC_WIN=%0d  BIN_THR=%0d  N_PRIME=%0d",
                 P_THRESH, P_HILO_WINDOW, P_COINC_WINDOW, P_BIN_THR, N_PRIME);

        pass_count = 0;
        fail_count = 0;

        // ------------------------------------------------------------------
        // Event loop
        // ------------------------------------------------------------------
        for (ev = 0; ev < 4; ev++) begin
            $display("\n=== Event %0d / 4  [%s] ===", ev, chunk_type[ev]);

            // --- Prime ring buffer with N_PRIME zero batches ---
            for (b = 0; b < N_PRIME; b++) begin
                @(negedge clk_adc);
                drive_zero_batch();
                @(posedge clk_adc);
            end

            // --- Inject 8 event batches and record waveform ---
            l0_fired = 0;
            cnn_fired = 0;
            l0_time_ns = -1.0;
            cnn_raw = 32'hx;

            ev_start_time_ns = $realtime;

            // Log all 256 event samples
            for (b = 0; b < 8; b++) begin
                @(negedge clk_adc);
                drive_event_batch(ev, b);
                // Log the 32 samples of this batch
                for (s = 0; s < 32; s++) begin
                    $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                            ev, b*32+s,
                            $signed(adc_ch[0][s]), $signed(adc_ch[1][s]),
                            $signed(adc_ch[2][s]), $signed(adc_ch[3][s]));
                end
                @(posedge clk_adc);
            end

            // Return to zero batches while waiting for results
            @(negedge clk_adc);
            drive_zero_batch();

            // --- Wait for L0 trigger and CNN result, with timeout ---
            fork : ev_fork
                // Branch 1: monitor L0_PRE_TRIG and CNN output
                begin : monitor_thread
                    // Wait for L0 trigger (anywhere after event data started)
                    @(posedge l0_pre_trig);
                    l0_fired   = 1;
                    l0_time_ns = $realtime;
                    $display("  [%0t] L0_PRE_TRIG fired  (%.0f ns after ev start)",
                             $time, l0_time_ns - ev_start_time_ns);

                    // Wait for CNN output
                    @(posedge cnn_out_valid);
                    cnn_fired = 1;
                    cnn_raw   = cnn_out_data;
                    $display("  [%0t] CNN_OUT_VALID  raw=0x%08h  score=%.4f",
                             $time, cnn_raw,
                             $itor($signed(cnn_raw[16:0])) / 256.0);
                end

                // Branch 2: timeout watchdog
                begin : timeout_thread
                    #(TIMEOUT_NS);
                    if (!l0_fired)
                        $display("  [%0t] TIMEOUT — L0 did not fire within %.0f µs.",
                                 $time, TIMEOUT_NS/1000.0);
                    else if (!cnn_fired)
                        $display("  [%0t] TIMEOUT — CNN result not received within %.0f µs.",
                                 $time, TIMEOUT_NS/1000.0);
                end
            join_any
            disable ev_fork;

            // Continue feeding zeros while CNN processes (important for next event)
            repeat(20) @(posedge clk_adc);

            // --- Pass/fail evaluation ---
            begin
                int pass;
                real cnn_score;
                cnn_score = $itor($signed(cnn_raw[16:0])) / 256.0;

                if (chunk_type[ev] == "sig") begin
                    // Signal: L0 must fire AND CNN must score > 0.5
                    pass = l0_fired && cnn_fired && ($signed(cnn_raw[16:0]) > CNN_SCORE_THRESH);
                    $display("  Signal check: l0_fired=%0d  cnn_fired=%0d  cnn_score=%.4f  -> %s",
                             l0_fired, cnn_fired, cnn_score, pass ? "PASS" : "FAIL");
                end else begin
                    // Noise: L0 must NOT fire
                    pass = !l0_fired;
                    $display("  Noise check:  l0_fired=%0d  -> %s",
                             l0_fired, pass ? "PASS" : "FAIL");
                end

                if (pass) pass_count++;
                else       fail_count++;

                // Log result
                $fwrite(f_results,
                        "%0d,%s,%0d,%.1f,%.1f,%0d,0x%08h,%.6f,%0d\n",
                        ev, chunk_type[ev],
                        l0_fired, l0_time_ns, ev_start_time_ns,
                        cnn_fired, cnn_raw, cnn_score, pass);
            end
        end

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n========================================");
        $display("  SANITY TEST COMPLETE");
        $display("  Passed: %0d / 4", pass_count);
        $display("  Failed: %0d / 4", fail_count);
        $display("  CHUNK_OVERFLOW at any point: %0d", chunk_overflow);
        $display("========================================\n");

        $fclose(f_wave);
        $fclose(f_results);

        if (fail_count == 0)
            $display("[RESULT] ALL PASS");
        else
            $display("[RESULT] %0d FAILURES — check sanity_results.txt and sanity_wave.csv", fail_count);

        $finish;
    end

endmodule
