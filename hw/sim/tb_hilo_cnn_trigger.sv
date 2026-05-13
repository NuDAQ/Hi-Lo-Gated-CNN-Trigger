`timescale 1ns / 10ps

// =============================================================================
// tb_hilo_cnn_trigger.sv
//
// SystemVerilog testbench for HILO_CNN_TRIGGER (via the mixed-language wrapper
// HILO_CNN_TRIGGER_TB_WRAP).
//
// Clock parameters:
//   CLK_ADC : 31.25 MHz  (32 ns period) — 1 GHz ADC / 32 samples per batch
//   CLK_CNN : 200  MHz  ( 5 ns period)
//
// ADC interface: 4 channels × 16 samples × 12-bit (two's complement)
// Flat vector   : 4 × 16 × 12 = 768 bits, MSB-first packing
//   flat[767 - (ch*16+s)*12 -: 12] ↔ adc_data4_type(ch)(s)
//
// Test strategy:
//   Background: algorithmic Gaussian noise, amplitude ~σ (always below THRESH)
//   Injection:  20 test events — even indices inject a guaranteed signal event
//               (read from pre-computed hex files), odd indices inject random.
//               Signal is superimposed on the background noise baseline.
//   Logging:    three text files (waveforms, L0 triggers, CNN results)
//               readable by ROOT analysis scripts.
// =============================================================================

module tb_hilo_cnn_trigger;

    // -------------------------------------------------------------------------
    // Clock and timing
    // -------------------------------------------------------------------------
    parameter real ADC_CLK_PERIOD = 32.0;   // ns — 31.25 MHz
    parameter real CNN_CLK_PERIOD =  5.0;   // ns — 200 MHz

    // Hi-Lo trigger configuration
    parameter logic [11:0] P_THRESH       = 12'h6A4; // 1700 ADC counts
    parameter logic [ 4:0] P_HILO_WINDOW  = 5'd10;   // 10-sample Hi-Lo window
    parameter logic [ 5:0] P_COINC_WINDOW = 6'd20;   // 20-sample coincidence
    parameter logic [ 3:0] P_BIN_THR      = 4'd2;    // ≥2 channels

    // -------------------------------------------------------------------------
    // DUT signals
    // -------------------------------------------------------------------------
    reg  clk_adc     = 0;
    reg  clk_cnn     = 0;
    reg  rst;
    reg  data_str;

    // Per-channel, per-sample ADC values (12-bit two's complement)
    reg [11:0] adc_ch [0:3][0:15];

    // Flat packed vector for the VHDL wrapper port
    // Packing: flat[767 - (ch*16+s)*12 -: 12] = adc_ch[ch][s]
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
    wire        chunk_overflow;

    // -------------------------------------------------------------------------
    // Simulation logging
    // -------------------------------------------------------------------------
    integer f_wave, f_trig, f_cnn;
    integer cur_test_num  = -1;
    integer cur_event_idx = -1;
    integer cur_label     = -1;

    // Overflow counter (informational)
    integer overflow_cnt = 0;
    always @(posedge chunk_overflow) overflow_cnt++;

    // -------------------------------------------------------------------------
    // Event memory (1000 events × 256 timesteps × 4 ch × 12-bit = 48-bit line)
    // -------------------------------------------------------------------------
    reg [47:0] event_mem [0:255999];   // 1000 events × 256 timesteps
    reg [31:0] label_mem [0:999];

    // Pool of signal-event indices (label == 1)
    integer sig_pool [0:999];
    integer n_sigs = 0;

    initial begin
        $readmemh("/home/work1/Works/AI-Trigger-System/data/real_events.hex", event_mem);
        $readmemh("/home/work1/Works/AI-Trigger-System/data/real_labels.hex", label_mem);

        n_sigs = 0;
        for (int i = 0; i < 1000; i++) begin
            if (label_mem[i] == 32'h1) begin
                sig_pool[n_sigs] = i;
                n_sigs++;
            end
        end
        $display("[%0t] Pre-scan: %0d signal events found.", $time, n_sigs);
    end

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

    // Clocks
    always #(ADC_CLK_PERIOD / 2.0) clk_adc = ~clk_adc;
    always #(CNN_CLK_PERIOD / 2.0) clk_cnn = ~clk_cnn;

    // -------------------------------------------------------------------------
    // Background noise driver (runs when not injecting an event)
    // Amplitude stays below THRESH to prevent false triggers.
    // -------------------------------------------------------------------------
    reg injecting_event = 0;

    always @(posedge clk_adc) begin
        if (data_str && !injecting_event) begin
            for (int c = 0; c < 4; c++) begin
                for (int s = 0; s < 16; s++) begin
                    // Noise centred at 0, amplitude < THRESH (1700)
                    adc_ch[c][s] <= 12'(($urandom % 256) - 128);
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // L0 trigger logging
    // -------------------------------------------------------------------------
    always @(posedge l0_pre_trig) begin
        $fwrite(f_trig, "%0d %0d %0d %0d\n",
                $time, cur_test_num, cur_event_idx, cur_label);
        $fflush(f_trig);
    end

    // -------------------------------------------------------------------------
    // CNN output logging
    // -------------------------------------------------------------------------
    always @(posedge clk_cnn) begin
        if (cnn_out_valid && cnn_out_ready) begin
            $fwrite(f_cnn, "%0d %0d %0d %0d %0d\n",
                    cur_test_num, cur_event_idx, cur_label,
                    cnn_out_data, $time);
            $fflush(f_cnn);
        end
    end

    // -------------------------------------------------------------------------
    // Main stimulus
    // -------------------------------------------------------------------------
    initial begin
        $display("[%0t] HILO_CNN_TRIGGER testbench starting.", $time);

        f_wave = $fopen("/home/work1/Works/AI-Trigger-System/data/analysis/sim_waveforms.txt",  "w");
        f_trig = $fopen("/home/work1/Works/AI-Trigger-System/data/analysis/sim_triggers.txt",   "w");
        f_cnn  = $fopen("/home/work1/Works/AI-Trigger-System/data/analysis/sim_cnn_results.txt","w");
        $fwrite(f_wave, "# test_num event_idx label time_ps sample ch0 ch1 ch2 ch3\n");
        $fwrite(f_trig, "# time_ps test_num event_idx label\n");
        $fwrite(f_cnn,  "# test_num event_idx label cnn_score time_ps\n");

        // Initialise
        rst           = 1;
        data_str      = 0;
        cnn_out_ready = 1;
        injecting_event = 0;
        for (int c = 0; c < 4; c++)
            for (int s = 0; s < 16; s++)
                adc_ch[c][s] = 12'h000;

        repeat(20) @(posedge clk_adc);
        rst      = 0;
        repeat(5)  @(posedge clk_adc);

        $display("[%0t] Reset released — streaming background noise.", $time);
        data_str = 1;
        repeat(200) @(posedge clk_adc);   // let ring buffer fill

        // -----------------------------------------------------------------
        // Dynamic injection loop: 20 test events.
        // Even test indices → guaranteed signal; odd → random pick.
        // -----------------------------------------------------------------
        for (int test_ev = 0; test_ev < 20; test_ev++) begin
            automatic integer delay_cycles;
            automatic integer rand_event_idx;
            automatic integer mem_base;
            automatic reg [47:0] mem_line;
            automatic reg [11:0] pure_sig;
            automatic reg [11:0] noise;

            // 1. Random idle gap
            delay_cycles = 200 + ($urandom % 800);
            repeat(delay_cycles) @(posedge clk_adc);

            // 2. Choose event
            if (test_ev % 2 == 0 && n_sigs > 0)
                rand_event_idx = sig_pool[(test_ev / 2) % n_sigs];
            else
                rand_event_idx = $urandom % 1000;

            cur_test_num  = test_ev;
            cur_event_idx = rand_event_idx;
            cur_label     = label_mem[rand_event_idx];

            $display("===================================================");
            $display("[%0t] INJECT event #%0d  label=%0d",
                     $time, rand_event_idx, cur_label);
            $display("===================================================");

            // 3. Inject: 16 batches × 16 samples = 256 timesteps
            injecting_event = 1;
            for (int batch = 0; batch < 16; batch++) begin
                @(posedge clk_adc);
                for (int s = 0; s < 16; s++) begin
                    mem_base = rand_event_idx * 256 + batch * 16 + s;
                    mem_line = event_mem[mem_base];

                    for (int c = 0; c < 4; c++) begin
                        // Extract 12-bit two's complement signal for channel c
                        case (c)
                            0: pure_sig = mem_line[11: 0];
                            1: pure_sig = mem_line[23:12];
                            2: pure_sig = mem_line[35:24];
                            3: pure_sig = mem_line[47:36];
                        endcase

                        // Small noise floor (well below THRESH)
                        noise = 12'(($urandom % 64) - 32);
                        adc_ch[c][s] <= pure_sig + noise;
                    end

                    // Waveform log (one row per ADC sample)
                    $fwrite(f_wave, "%0d %0d %0d %0d %0d %0d %0d %0d %0d\n",
                            cur_test_num, cur_event_idx, cur_label, $time,
                            batch * 16 + s,
                            adc_ch[0][s], adc_ch[1][s],
                            adc_ch[2][s], adc_ch[3][s]);
                end
            end
            @(posedge clk_adc);
            injecting_event = 0;

            // 4. Wait for CNN result or timeout
            fork
                begin
                    wait (cnn_out_valid == 1);
                    $display("[%0t] CNN result: 0x%08h", $time, cnn_out_data);
                    wait (cnn_out_valid == 0);
                end
                begin
                    #500000;  // 500 µs timeout
                    $display("[%0t] TIMEOUT — no CNN output.", $time);
                end
            join_any
            disable fork;
        end

        repeat(500) @(posedge clk_cnn);
        $display("[%0t] All events injected. Overflow count: %0d",
                 $time, overflow_cnt);

        $fclose(f_wave);
        $fclose(f_trig);
        $fclose(f_cnn);
        $finish;
    end

endmodule
