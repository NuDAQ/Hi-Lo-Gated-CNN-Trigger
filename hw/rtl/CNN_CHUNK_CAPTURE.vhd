-- Copyright 2026 Albert L. Cheung @ University of California, Irvine
-- SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pre_trigger_pkg.all;

-- ----------------------------------------------------------------------------
-- CNN_CHUNK_CAPTURE
--
-- Captures a 256-sample chunk (128 pre + 128 post) around an L0 trigger and
-- streams it to the CNN inference core (WRAPPER_TOP) via AXI-Stream.
--
-- Clock domains:
--   CLK_ADC : write side — ring buffer, post-capture FSM, BRAM write port
--   CLK_CNN : read side  — BRAM read port, CNN stream FSM
--
-- 64-bit word format (one ADC sample across 4 channels):
--   [63:48] ch3  [47:32] ch2  [31:16] ch1  [15:0] ch0
--   Each channel: 4-bit zero pad + 12-bit ADC value
--
-- N_BUF-deep circular queue:
--   BRAM N_BUF*256 × 64-bit: Buffer i occupies addr i*256 .. i*256+255
--   ADC writes at wr_ptr, CNN reads at rd_ptr; both advance mod N_BUF.
--   CDC: N_BUF-bit-wide 4-phase set/clear handshake via 2-FF synchronizers.
--
-- Rate-based L0 blanking (CLK_ADC domain):
--   A fixed WINDOW_CYCLES window counts ALL raw L0 pulses (even during
--   blanking, so the FSM detects when noise truly subsides).
--   • Enter BLANKING when rate_cnt ≥ HI_THRESH at window end.
--   • Exit  BLANKING when rate_cnt ≤ LO_THRESH AND queue is empty.
--   During blanking the ADC FSM silently discards L0 pulses without setting
--   CHUNK_OVERFLOW (intentional discard ≠ resource overflow).
--
-- CHUNK_OVERFLOW (sticky, CLK_ADC):
--   Set only when a non-blanking L0 fires but wr_ptr's buffer slot is still
--   occupied (true resource overflow). Cleared only by RST.
-- ----------------------------------------------------------------------------

entity CNN_CHUNK_CAPTURE is
    generic (
        -- CLK_ADC frequency in Hz, used to derive the rate-monitor window so
        -- that the 50 µs blanking window stays calibrated regardless of the
        -- actual CLK_ADC rate.  Default matches the nominal 62.5 MHz.
        CLK_ADC_HZ : integer := 62_500_000
    );
    port (
        -- CLK_ADC domain
        CLK_ADC      : in  std_logic;
        RST          : in  std_logic;  -- active-high, sync to CLK_ADC
        DATA_STR     : in  std_logic;
        ADC_DATA4    : in  adc_data4_type;
        L0_PRE_TRIG  : in  std_logic;

        -- CLK_CNN domain
        CLK_CNN      : in  std_logic;
        RST_N_CNN    : out std_logic;  -- active-low reset, synced to CLK_CNN
        CNN_START    : out std_logic;
        CNN_DONE     : in  std_logic;
        CNN_IDLE     : in  std_logic;
        CNN_READY    : in  std_logic;
        CNN_IN_DATA  : out std_logic_vector(63 downto 0);
        CNN_IN_VALID : out std_logic;
        CNN_IN_READY : in  std_logic;

        -- Status (CLK_ADC domain)
        CHUNK_OVERFLOW : out std_logic;
        L0_BLANKING    : out std_logic   -- high while noise blanking is active
    );
end CNN_CHUNK_CAPTURE;

architecture rtl of CNN_CHUNK_CAPTURE is

    -- =========================================================================
    -- Configurable parameters — edit these constants, no interface change needed
    -- =========================================================================
    constant N_BUF         : integer := 12;    -- circular queue depth (buffers)
    -- Rate-monitor window: 50 µs, derived from CLK_ADC_HZ generic.
    -- At 62.5 MHz → 3125 cycles; at 125 MHz → 6250 cycles; etc.
    constant WINDOW_CYCLES : integer := CLK_ADC_HZ / 20_000;
    constant HI_THRESH     : integer := 10;    -- enter blanking: ≥10 L0/window (1 per 5 µs)
    constant LO_THRESH     : integer := 3;     -- exit  blanking: ≤3  L0/window (<1 per 15 µs)

    -- =========================================================================
    -- Types
    -- =========================================================================
    type batch_t is array(0 to 15) of std_logic_vector(63 downto 0);
    type ring_t  is array(0 to 7)  of batch_t;

    -- =========================================================================
    -- Pre/post sample buffers (CLK_ADC domain)
    -- =========================================================================
    signal ring_buf : ring_t := (others => (others => (others => '0')));
    signal post_buf : ring_t := (others => (others => (others => '0')));

    -- =========================================================================
    -- Circular-queue BRAM: N_BUF*256 × 64-bit
    -- Buffer i: addr i*256 .. i*256+255
    -- True-dual-port inferred: Port A = CLK_ADC (write), Port B = CLK_CNN (read)
    -- =========================================================================
    type bram_t is array(0 to N_BUF*256-1) of std_logic_vector(63 downto 0);
    signal bram : bram_t;
    attribute ram_style        : string;
    attribute ram_style of bram : signal is "block";

    -- BRAM write port (CLK_ADC)
    signal wr_en   : std_logic                     := '0';
    signal wr_addr : unsigned(11 downto 0)         := (others => '0');
    signal wr_data : std_logic_vector(63 downto 0) := (others => '0');

    -- =========================================================================
    -- CDC signals — 4-phase set/clear handshake, N_BUF bits wide
    --
    -- Protocol for buffer i:
    --   ADC sets   buf_written_adc(i) after finishing a BRAM write
    --   CDC sync → buf_written_cnn(i)  (2 CLK_CNN cycles latency)
    --   CNN sets   buf_ack_cnn(i)      after CNN_DONE
    --   CDC sync → buf_ack_adc(i)      (2 CLK_ADC cycles latency)
    --   ADC clears buf_written_adc(i)  on seeing buf_ack_adc(i)='1'
    --   CDC sync → buf_written_cnn(i)='0'
    --   CNN clears buf_ack_cnn(i)      on seeing buf_written_cnn(i)='0'
    -- =========================================================================
    signal buf_written_adc : std_logic_vector(N_BUF-1 downto 0) := (others => '0');
    signal buf_written_s1  : std_logic_vector(N_BUF-1 downto 0) := (others => '0');
    signal buf_written_cnn : std_logic_vector(N_BUF-1 downto 0) := (others => '0');

    signal buf_ack_cnn : std_logic_vector(N_BUF-1 downto 0) := (others => '0');
    signal buf_ack_s1  : std_logic_vector(N_BUF-1 downto 0) := (others => '0');
    signal buf_ack_adc : std_logic_vector(N_BUF-1 downto 0) := (others => '0');

    -- RST synchronizer (ADC → CNN domain)
    signal rst_s1  : std_logic := '1';
    signal rst_cnn : std_logic := '1';

    -- =========================================================================
    -- Rate monitor + blanking (CLK_ADC domain)
    -- =========================================================================
    signal rate_cnt      : integer range 0 to 31             := 0;
    signal win_timer     : integer range 0 to WINDOW_CYCLES-1 := 0;
    signal l0_blanking_i : std_logic := '0';  -- internal blanking flag
    signal buf_any_written : std_logic;       -- OR of all buf_written_adc bits
    signal fifo_empty    : std_logic;

    -- =========================================================================
    -- ADC-domain FSM
    -- =========================================================================
    type adc_fsm_t is (ADC_IDLE, ADC_POST, ADC_WRITE);
    signal adc_state : adc_fsm_t := ADC_IDLE;

    signal post_cnt  : integer range 0 to 7  := 0;
    signal batch_cnt : integer range 0 to 15 := 0;
    signal samp_cnt  : integer range 0 to 15 := 0;
    signal wr_ptr    : integer range 0 to N_BUF-1 := 0;

    -- =========================================================================
    -- CNN-domain FSM
    -- =========================================================================
    type cnn_fsm_t is (CC_IDLE, CC_STREAM, CC_WAIT_DONE, CC_ACK);
    signal cnn_state     : cnn_fsm_t    := CC_IDLE;

    signal cnn_base_addr : unsigned(11 downto 0) := (others => '0');
    signal stream_ptr    : unsigned(7 downto 0)  := (others => '0');
    signal cnn_buf_id    : integer range 0 to N_BUF-1 := 0;
    signal rd_ptr        : integer range 0 to N_BUF-1 := 0;

    -- =========================================================================
    -- Helper: pack one ADC sample (4 channels) into 64 bits
    -- =========================================================================
    function pack_sample(d : adc_data4_type; s : integer)
        return std_logic_vector is
        variable w : std_logic_vector(63 downto 0);
    begin
        w(63 downto 48) := x"0" & d(3)(s);
        w(47 downto 32) := x"0" & d(2)(s);
        w(31 downto 16) := x"0" & d(1)(s);
        w(15 downto  0) := x"0" & d(0)(s);
        return w;
    end function;

begin

    L0_BLANKING <= l0_blanking_i;

    -- =========================================================================
    -- RST → CNN domain: 2-FF synchronizer
    -- =========================================================================
    process(CLK_CNN)
    begin
        if rising_edge(CLK_CNN) then
            rst_s1  <= RST;
            rst_cnn <= rst_s1;
        end if;
    end process;
    RST_N_CNN <= not rst_cnn;

    -- =========================================================================
    -- BRAM Port A — write (CLK_ADC)
    -- =========================================================================
    process(CLK_ADC)
    begin
        if rising_edge(CLK_ADC) then
            if wr_en = '1' then
                bram(to_integer(wr_addr)) <= wr_data;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- CDC: buf_written (ADC → CNN), 2-FF synchronizer
    -- =========================================================================
    process(CLK_CNN)
    begin
        if rising_edge(CLK_CNN) then
            if rst_cnn = '1' then
                buf_written_s1  <= (others => '0');
                buf_written_cnn <= (others => '0');
            else
                buf_written_s1  <= buf_written_adc;
                buf_written_cnn <= buf_written_s1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- CDC: buf_ack (CNN → ADC), 2-FF synchronizer
    -- =========================================================================
    process(CLK_ADC)
    begin
        if rising_edge(CLK_ADC) then
            if RST = '1' then
                buf_ack_s1  <= (others => '0');
                buf_ack_adc <= (others => '0');
            else
                buf_ack_s1  <= buf_ack_cnn;
                buf_ack_adc <= buf_ack_s1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- fifo_empty: OR-reduce buf_written_adc; also require ADC FSM to be idle
    -- (a buffer being written is not yet flagged but the queue is not empty)
    -- =========================================================================
    process(buf_written_adc)
        variable v : std_logic;
    begin
        v := '0';
        for i in 0 to N_BUF-1 loop
            v := v or buf_written_adc(i);
        end loop;
        buf_any_written <= v;
    end process;

    fifo_empty <= '1' when buf_any_written = '0' and adc_state = ADC_IDLE else '0';

    -- =========================================================================
    -- Rate monitor + Blanking FSM (CLK_ADC domain)
    --
    -- Counts every raw L0_PRE_TRIG in a rolling WINDOW_CYCLES window,
    -- regardless of blanking state.  Decisions are made at window boundaries
    -- so the window also acts as a natural hold-off: blanking cannot re-enter
    -- or re-exit more than once per window.
    -- =========================================================================
    process(CLK_ADC)
    begin
        if rising_edge(CLK_ADC) then
            if RST = '1' then
                rate_cnt      <= 0;
                win_timer     <= 0;
                l0_blanking_i <= '0';
            else
                -- Saturating count — avoids integer overflow for extreme noise
                if L0_PRE_TRIG = '1' and rate_cnt < 31 then
                    rate_cnt <= rate_cnt + 1;
                end if;

                if win_timer = WINDOW_CYCLES - 1 then
                    win_timer <= 0;
                    -- Evaluate threshold at window boundary, then clear counter
                    if l0_blanking_i = '0' then
                        if rate_cnt >= HI_THRESH then
                            l0_blanking_i <= '1';
                        end if;
                    else
                        -- Exit only when rate is low AND queue has drained
                        if rate_cnt <= LO_THRESH and fifo_empty = '1' then
                            l0_blanking_i <= '0';
                        end if;
                    end if;
                    rate_cnt <= 0;
                else
                    win_timer <= win_timer + 1;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- ADC-domain FSM
    -- =========================================================================
    process(CLK_ADC)
    begin
        if rising_edge(CLK_ADC) then
            if RST = '1' then
                adc_state       <= ADC_IDLE;
                post_cnt        <= 0;
                batch_cnt       <= 0;
                samp_cnt        <= 0;
                wr_ptr          <= 0;
                wr_en           <= '0';
                wr_addr         <= (others => '0');
                wr_data         <= (others => '0');
                buf_written_adc <= (others => '0');
                CHUNK_OVERFLOW  <= '0';
                ring_buf        <= (others => (others => (others => '0')));
                post_buf        <= (others => (others => (others => '0')));
            else
                wr_en <= '0';

                -- 4-phase handshake: ADC clears buf_written when it sees ack
                for i in 0 to N_BUF-1 loop
                    if buf_ack_adc(i) = '1' and buf_written_adc(i) = '1' then
                        buf_written_adc(i) <= '0';
                    end if;
                end loop;

                case adc_state is

                    -- ----------------------------------------------------------
                    -- ADC_IDLE: continuously roll the pre-trigger ring buffer.
                    -- On L0: if blanking, discard silently; otherwise claim
                    -- wr_ptr slot (or flag overflow if it is still occupied).
                    -- ----------------------------------------------------------
                    when ADC_IDLE =>
                        if DATA_STR = '1' then
                            ring_buf(7) <= ring_buf(6);
                            ring_buf(6) <= ring_buf(5);
                            ring_buf(5) <= ring_buf(4);
                            ring_buf(4) <= ring_buf(3);
                            ring_buf(3) <= ring_buf(2);
                            ring_buf(2) <= ring_buf(1);
                            ring_buf(1) <= ring_buf(0);
                            for s in 0 to 15 loop
                                ring_buf(0)(s) <= pack_sample(ADC_DATA4, s);
                            end loop;
                        end if;

                        if L0_PRE_TRIG = '1' then
                            if l0_blanking_i = '0' then
                                if buf_written_adc(wr_ptr) = '0' and
                                   buf_ack_adc(wr_ptr) = '0' then
                                    post_cnt  <= 0;
                                    adc_state <= ADC_POST;
                                else
                                    -- wr_ptr slot still occupied: true overflow
                                    CHUNK_OVERFLOW <= '1';
                                end if;
                            end if;
                            -- l0_blanking_i = '1': intentional discard, no flag
                        end if;

                    -- ----------------------------------------------------------
                    -- ADC_POST: capture 8 post-trigger batches (128 samples).
                    -- Ring buffer is frozen; new L0 pulses are silently dropped
                    -- (blanking should be active before this becomes an issue).
                    -- ----------------------------------------------------------
                    when ADC_POST =>
                        if DATA_STR = '1' then
                            for s in 0 to 15 loop
                                post_buf(post_cnt)(s) <= pack_sample(ADC_DATA4, s);
                            end loop;

                            if post_cnt = 7 then
                                batch_cnt <= 0;
                                samp_cnt  <= 0;
                                adc_state <= ADC_WRITE;
                            else
                                post_cnt <= post_cnt + 1;
                            end if;
                        end if;

                    -- ----------------------------------------------------------
                    -- ADC_WRITE: stream 16 batches (256 words) into BRAM buffer
                    -- wr_ptr at full CLK_ADC rate.
                    --
                    -- Write order (oldest sample first):
                    --   batch 0-7  → ring_buf[7..0]  (128 pre-trigger samples)
                    --   batch 8-15 → post_buf[0..7]  (128 post-trigger samples)
                    -- ----------------------------------------------------------
                    when ADC_WRITE =>
                        wr_en <= '1';

                        wr_addr <= to_unsigned(
                            wr_ptr * 256 + batch_cnt * 16 + samp_cnt, 12);

                        if batch_cnt < 8 then
                            wr_data <= ring_buf(7 - batch_cnt)(samp_cnt);
                        else
                            wr_data <= post_buf(batch_cnt - 8)(samp_cnt);
                        end if;

                        if samp_cnt = 15 then
                            samp_cnt <= 0;
                            if batch_cnt = 15 then
                                wr_en <= '0';
                                buf_written_adc(wr_ptr) <= '1';
                                -- Advance write pointer (mod N_BUF)
                                if wr_ptr = N_BUF - 1 then
                                    wr_ptr <= 0;
                                else
                                    wr_ptr <= wr_ptr + 1;
                                end if;
                                adc_state <= ADC_IDLE;
                            else
                                batch_cnt <= batch_cnt + 1;
                            end if;
                        else
                            samp_cnt <= samp_cnt + 1;
                        end if;

                end case;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- DIAGNOSTIC concurrent processes — print key signal transitions.
    -- These appear as NOTE messages in the xsim log.  Remove before production.
    -- =========================================================================
    process(CNN_IDLE)
    begin
        report "[CNN_CHUNK] CNN_IDLE = " & std_logic'image(CNN_IDLE) severity note;
    end process;

    process(buf_written_cnn)
    begin
        report "[CNN_CHUNK] buf_written_cnn = "
               & std_logic'image(buf_written_cnn(0))
               & std_logic'image(buf_written_cnn(1)) severity note;
    end process;

    -- =========================================================================
    -- CNN-domain FSM
    --
    -- Processes buffers in FIFO order: rd_ptr follows wr_ptr mod N_BUF.
    -- CC_IDLE waits until buf_written_cnn(rd_ptr) = '1'.  If the ADC is
    -- still filling that slot, CC_IDLE simply waits — the queue preserves order.
    -- =========================================================================
    process(CLK_CNN)
    begin
        if rising_edge(CLK_CNN) then
            if rst_cnn = '1' then
                cnn_state     <= CC_IDLE;
                CNN_START     <= '0';
                CNN_IN_VALID  <= '0';
                CNN_IN_DATA   <= (others => '0');
                buf_ack_cnn   <= (others => '0');
                stream_ptr    <= (others => '0');
                cnn_buf_id    <= 0;
                cnn_base_addr <= (others => '0');
                rd_ptr        <= 0;
            else
                -- ap_ctrl_hs: hold CNN_START until CNN_READY fires
                if CNN_READY = '1' then
                    CNN_START <= '0';
                end if;

                -- 4-phase handshake: CNN clears ack once ADC has cleared
                -- buf_written (visible as buf_written_cnn going low)
                for i in 0 to N_BUF-1 loop
                    if buf_written_cnn(i) = '0' then
                        buf_ack_cnn(i) <= '0';
                    end if;
                end loop;

                case cnn_state is

                    -- ----------------------------------------------------------
                    -- CC_IDLE: wait for rd_ptr's buffer to be marked written.
                    -- Guard buf_ack_cnn(rd_ptr)='0' prevents re-triggering on
                    -- the same buffer before the ADC has cleared buf_written.
                    -- ----------------------------------------------------------
                    when CC_IDLE =>
                        CNN_IN_VALID <= '0';

                        if buf_written_cnn(rd_ptr) = '1' and
                           buf_ack_cnn(rd_ptr) = '0' then
                            cnn_buf_id    <= rd_ptr;
                            cnn_base_addr <= to_unsigned(rd_ptr * 256, 12);
                            CNN_START     <= '1';
                            CNN_IN_VALID  <= '1';
                            CNN_IN_DATA   <= bram(rd_ptr * 256);  -- word 0
                            stream_ptr    <= (others => '0');
                            cnn_state     <= CC_STREAM;
                        end if;

                    -- ----------------------------------------------------------
                    -- CC_STREAM: feed 256 × 64-bit words to CNN via AXI-S.
                    -- CNN_IN_VALID stays '1' continuously (no bubbles).
                    -- CNN_IN_DATA is pre-registered: at entry we present word 0;
                    -- on each READY we pre-load word N+1 for the next cycle.
                    -- ----------------------------------------------------------
                    when CC_STREAM =>
                        CNN_IN_VALID <= '1';

                        if CNN_IN_READY = '1' then
                            if stream_ptr = 255 then
                                CNN_IN_VALID <= '0';
                                cnn_state    <= CC_WAIT_DONE;
                            else
                                stream_ptr  <= stream_ptr + 1;
                                CNN_IN_DATA <= bram(to_integer(
                                                  cnn_base_addr
                                                  + resize(stream_ptr + 1, 12)));
                            end if;
                        end if;

                    -- ----------------------------------------------------------
                    when CC_WAIT_DONE =>
                        CNN_IN_VALID <= '0';
                        if CNN_DONE = '1' then
                            cnn_state <= CC_ACK;
                        end if;

                    -- ----------------------------------------------------------
                    -- CC_ACK: set ack, advance rd_ptr, return to CC_IDLE.
                    -- buf_ack_cnn stays set until buf_written_cnn goes low
                    -- (ADC has seen the ack and cleared its side).
                    -- ----------------------------------------------------------
                    when CC_ACK =>
                        CNN_IN_VALID <= '0';
                        buf_ack_cnn(cnn_buf_id) <= '1';
                        if rd_ptr = N_BUF - 1 then
                            rd_ptr <= 0;
                        else
                            rd_ptr <= rd_ptr + 1;
                        end if;
                        cnn_state <= CC_IDLE;

                end case;
            end if;
        end if;
    end process;

end rtl;
