-- Copyright 2026 Albert L. Cheung @ University of California, Irvine
-- SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

library ieee;
use ieee.std_logic_1164.all;
use work.pre_trigger_pkg.all;

-- ----------------------------------------------------------------------------
-- HILO_CNN_TRIGGER
--
-- Top-level structural composition of the Hi-Lo Gated CNN Trigger.
-- Instantiates:
--   1. PRE_TRIGGER       (hilo-trigger dep) — L0 bipolar pre-trigger
--   2. CNN_CHUNK_CAPTURE (this repo)        — pre/post ring buffer + ping-pong
--   3. WRAPPER_TOP       (cnn-core-wrapper) — HLS CNN inference core
--
-- Clock domains:
--   CLK_ADC : ADC batching clock  (62.5 MHz typical, 1 GHz / 16 samples)
--   CLK_CNN : CNN inference clock (200 MHz typical)
--
-- For multi-antenna-pair systems, instantiate this module twice at a higher
-- level, routing 4 channels per instance.
-- ----------------------------------------------------------------------------

entity HILO_CNN_TRIGGER is
    generic (
        -- CLK_ADC frequency in Hz.  Used to:
        --   1. Calibrate the 50 µs rate-monitor window in CNN_CHUNK_CAPTURE.
        --   2. Size nothing else — only the blanking timer depends on it.
        -- Default: 62.5 MHz (1 GHz ADC / 16 samples per batch).
        CLK_ADC_HZ  : integer := 62_500_000;

        -- Depth of the elastic ADC batch FIFO (number of 16-sample batches).
        -- Absorbs rate mismatches when CLK_ADC > ADC batch delivery rate.
        -- 8 batches = 128 samples of headroom, well above any jitter margin.
        FIFO_DEPTH  : integer := 8
    );
    port (
        CLK_ADC        : in  std_logic;
        CLK_CNN        : in  std_logic;
        -- Active-high synchronous reset (shared; re-timed to CLK_CNN internally)
        RST            : in  std_logic;

        -- ADC data interface (CLK_ADC domain)
        DATA_STR       : in  std_logic;
        ADC_DATA4      : in  adc_data4_type;       -- 4 ch × 16 samples × 12-bit

        -- Hi-Lo trigger configuration (CLK_ADC domain, static during operation)
        THRESH         : in  std_logic_vector(11 downto 0);
        HILO_WINDOW    : in  std_logic_vector( 4 downto 0);  -- max 16 samples
        COINC_WINDOW   : in  std_logic_vector( 5 downto 0);  -- max 32 samples
        BIN_THR        : in  std_logic_vector( 3 downto 0);  -- min active channels

        -- Trigger outputs
        L0_PRE_TRIG    : out std_logic;  -- real-time L0 decision (CLK_ADC domain)

        -- CNN result stream (CLK_CNN domain, AXI-S)
        CNN_OUT_DATA   : out std_logic_vector(31 downto 0);
        CNN_OUT_VALID  : out std_logic;
        CNN_OUT_READY  : in  std_logic;

        -- Status (CLK_ADC domain)
        -- Sticky; set when a trigger is dropped because the circular queue is
        -- full (outside blanking). Cleared by RST only.
        CHUNK_OVERFLOW : out std_logic;
        -- High while rate-based L0 blanking is active (noise suppression).
        L0_BLANKING    : out std_logic
    );
end HILO_CNN_TRIGGER;

architecture structural of HILO_CNN_TRIGGER is

    -- WRAPPER_TOP is a Verilog module; declare as component for mixed-language
    component WRAPPER_TOP
        generic (
            INPUT_WIDTH   : integer := 64;
            OUTPUT_WIDTH  : integer := 32;
            NUM_TIMESTEPS : integer := 256;
            NUM_CHANNELS  : integer := 4
        );
        port (
            clk          : in  std_logic;
            rst_n        : in  std_logic;
            start        : in  std_logic;
            done         : out std_logic;
            idle         : out std_logic;
            ready        : out std_logic;
            input_data   : in  std_logic_vector(63 downto 0);
            input_valid  : in  std_logic;
            input_ready  : out std_logic;
            output_data  : out std_logic_vector(31 downto 0);
            output_valid : out std_logic;
            output_ready : in  std_logic
        );
    end component;

    -- Buffered ADC signals from the elastic FIFO (CLK_ADC domain)
    signal data_str_buf  : std_logic;
    signal adc_data4_buf : adc_data4_type;

    signal pre_trig_int  : std_logic;

    -- CNN handshake (CLK_CNN domain)
    signal rst_n_cnn     : std_logic;
    signal cnn_start     : std_logic;
    signal cnn_done      : std_logic;
    signal cnn_idle      : std_logic;
    signal cnn_ready     : std_logic;
    signal cnn_in_data   : std_logic_vector(63 downto 0);
    signal cnn_in_valid  : std_logic;
    signal cnn_in_ready  : std_logic;

begin

    L0_PRE_TRIG <= pre_trig_int;

    -- -------------------------------------------------------------------------
    -- Elastic FIFO: absorbs rate mismatch when CLK_ADC > ADC batch rate.
    -- All downstream logic uses data_str_buf / adc_data4_buf.
    -- -------------------------------------------------------------------------
    u_ADC_STREAM_FIFO : entity work.ADC_STREAM_FIFO
        generic map (
            DEPTH => FIFO_DEPTH
        )
        port map (
            CLK_ADC      => CLK_ADC,
            RST          => RST,
            DATA_STR_IN  => DATA_STR,
            ADC_DATA4_IN => ADC_DATA4,
            DATA_STR_OUT => data_str_buf,
            ADC_DATA4_OUT => adc_data4_buf,
            OVERFLOW     => open,
            EMPTY        => open
        );

    -- -------------------------------------------------------------------------
    u_PRE_TRIGGER : entity work.PRE_TRIGGER
        port map (
            CLK          => CLK_ADC,
            RESET        => RST,
            DATA_STR     => data_str_buf,
            ADC_DATA4    => adc_data4_buf,
            THRESH       => THRESH,
            HILO_WINDOW  => HILO_WINDOW,
            COINC_WINDOW => COINC_WINDOW,
            BIN_THR      => BIN_THR,
            PRE_TRIG     => pre_trig_int
        );

    -- -------------------------------------------------------------------------
    u_CNN_CHUNK_CAPTURE : entity work.CNN_CHUNK_CAPTURE
        generic map (
            CLK_ADC_HZ => CLK_ADC_HZ
        )
        port map (
            CLK_ADC        => CLK_ADC,
            RST            => RST,
            DATA_STR       => data_str_buf,
            ADC_DATA4      => adc_data4_buf,
            L0_PRE_TRIG    => pre_trig_int,
            CLK_CNN        => CLK_CNN,
            RST_N_CNN      => rst_n_cnn,
            CNN_START      => cnn_start,
            CNN_DONE       => cnn_done,
            CNN_IDLE       => cnn_idle,
            CNN_READY      => cnn_ready,
            CNN_IN_DATA    => cnn_in_data,
            CNN_IN_VALID   => cnn_in_valid,
            CNN_IN_READY   => cnn_in_ready,
            CHUNK_OVERFLOW => CHUNK_OVERFLOW,
            L0_BLANKING    => L0_BLANKING
        );

    -- -------------------------------------------------------------------------
    u_WRAPPER_TOP : WRAPPER_TOP
        port map (
            clk          => CLK_CNN,
            rst_n        => rst_n_cnn,
            start        => cnn_start,
            done         => cnn_done,
            idle         => cnn_idle,
            ready        => cnn_ready,
            input_data   => cnn_in_data,
            input_valid  => cnn_in_valid,
            input_ready  => cnn_in_ready,
            output_data  => CNN_OUT_DATA,
            output_valid => CNN_OUT_VALID,
            output_ready => CNN_OUT_READY
        );

end structural;
