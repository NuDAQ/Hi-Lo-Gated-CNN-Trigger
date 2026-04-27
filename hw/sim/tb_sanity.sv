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

    // CNN decision threshold: score = $signed(output_data[16:0]) / 256.0
    // Signal   pass: score > +0.5  →  raw integer > +128
    // Noise    pass: score ≤ +0.5  →  raw integer ≤ +128
    // (ARIANNA thermal noise can trigger Hi-Lo; the CNN is the discriminator)
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
    // Async capture registers
    // These always blocks run concurrently with the main initial block,
    // so they correctly capture L0 / CNN results even if they fire during
    // the ADC batch driving loop (before the main block's fork starts).
    // -------------------------------------------------------------------------
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

        // ------------------------------------------------------------------
        // Load chunk hex files  (paths from sanity_paths.svh)
        // ------------------------------------------------------------------
        $readmemh(`CHUNK_SIG0,   chunk_mem[0]);
        $readmemh(`CHUNK_SIG1,   chunk_mem[1]);
        $readmemh(`CHUNK_SIG2,   chunk_mem[2]);
        $readmemh(`CHUNK_NOISE0, chunk_mem[3]);

        if (chunk_mem[0][0] === 64'bx) begin
            $display("[ERROR] Failed to load chunk_sig0.hex.");
            $display("        Run: python3 scripts/prepare_sanity_chunks.py");
            $finish;
        end

        // ------------------------------------------------------------------
        // Open log files
        // ------------------------------------------------------------------
        f_wave    = $fopen(`WAVE_CSV,    "w");
        f_results = $fopen(`RESULTS_TXT, "w");
        if (f_wave == 0 || f_results == 0) begin
            $display("[ERROR] Cannot open output log files — check sanity_data/ is writable.");
            $finish;
        end
        $fwrite(f_wave,    "# ev_id,sample_idx,ch0,ch1,ch2,ch3\n");
        $fwrite(f_results, "# ev_id,type,l0_fired,l0_time_ns,ev_start_ns,cnn_fired,cnn_raw_hex,cnn_score_float,pass\n");

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

            // --- Clear async capture flags BEFORE driving any data ---
            // (The always blocks above will set them when events occur.)
            l0_cap_valid = 0;
            l0_cap_time  = -1.0;
            cnn_cap_valid = 0;
            cnn_cap_data  = 0;

            // --- Prime ring buffer with N_PRIME zero batches ---
            for (b = 0; b < N_PRIME; b++) begin
                @(negedge clk_adc);
                drive_zero_batch();
                @(posedge clk_adc);
            end

            // --- Drive 8 event batches (L0 may fire DURING this loop) ---
            ev_start_time_ns = $realtime;

            for (b = 0; b < 8; b++) begin
                @(negedge clk_adc);
                drive_event_batch(ev, b);
                for (s = 0; s < 32; s++)
                    $fwrite(f_wave, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                            ev, b*32+s,
                            $signed(adc_ch[0][s]), $signed(adc_ch[1][s]),
                            $signed(adc_ch[2][s]), $signed(adc_ch[3][s]));
                @(posedge clk_adc);
            end

            // Keep driving zeros while we wait for CNN to respond
            @(negedge clk_adc);
            drive_zero_batch();

            // --- Wait for L0 + CNN result, or timeout ---
            // Use wait() (level-sensitive), NOT @(posedge ...) (edge-sensitive).
            // This correctly handles the case where L0 already fired during
            // batch driving above.
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

            // Drain any remaining zeros and let handshakes settle before next event
            repeat(200) @(posedge clk_cnn);

            // --- Pass/fail evaluation ---
            cnn_score = $itor($signed(cnn_cap_data[16:0])) / 256.0;

            if (chunk_type[ev] == "sig") begin
                pass = l0_cap_valid && cnn_cap_valid &&
                       ($signed(cnn_cap_data[16:0]) > CNN_SCORE_THRESH);
                $display("  Signal check: l0=%0d  cnn=%0d  score=%.4f  -> %s",
                         l0_cap_valid, cnn_cap_valid, cnn_score, pass ? "PASS" : "FAIL");
            end else begin
                if (!l0_cap_valid) begin
                    pass = 1;
                    $display("  Noise check:  l0=0 (Hi-Lo did not fire)  -> PASS");
                end else if (!cnn_cap_valid) begin
                    pass = 0;
                    $display("  Noise check:  l0=1  cnn=TIMEOUT  -> FAIL");
                end else begin
                    pass = ($signed(cnn_cap_data[16:0]) <= CNN_SCORE_THRESH);
                    $display("  Noise check:  l0=1  score=%.4f (must be ≤0.5)  -> %s",
                             cnn_score, pass ? "PASS" : "FAIL");
                end
            end

            if (pass) pass_count++;
            else       fail_count++;

            $fwrite(f_results,
                    "%0d,%s,%0d,%.1f,%.1f,%0d,0x%08h,%.6f,%0d\n",
                    ev, chunk_type[ev],
                    l0_cap_valid, l0_cap_time, ev_start_time_ns,
                    cnn_cap_valid, cnn_cap_data, cnn_score, pass);
        end

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
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
