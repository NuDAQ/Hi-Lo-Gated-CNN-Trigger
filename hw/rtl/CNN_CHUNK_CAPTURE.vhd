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
-- Ping-pong buffer:
--   BRAM 512 × 64-bit:  Buffer 0 = addr 0–255, Buffer 1 = addr 256–511
--   CDC: 4-phase set/clear handshake via 2-FF synchronizers
--
-- CHUNK_OVERFLOW (sticky, CLK_ADC):
--   Set when L0_PRE_TRIG fires but no buffer is free or capture is in
--   progress. Cleared only by RST. Connect to a monitoring/ILA port.
-- ----------------------------------------------------------------------------

entity CNN_CHUNK_CAPTURE is
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
        CHUNK_OVERFLOW : out std_logic
    );
end CNN_CHUNK_CAPTURE;

architecture rtl of CNN_CHUNK_CAPTURE is

    -- =========================================================================
    -- Types
    -- =========================================================================
    -- One batch: 32 packed 64-bit samples
    type batch_t is array(0 to 31) of std_logic_vector(63 downto 0);
    -- Four-batch ring / post buffer
    type ring_t  is array(0 to 3)  of batch_t;

    -- =========================================================================
    -- Ring buffer (4 most-recent pre-trigger batches, CLK_ADC domain)
    -- Shifts on every DATA_STR pulse while in ADC_IDLE.
    -- Frozen (stops shifting) as soon as a trigger is accepted.
    -- =========================================================================
    signal ring_buf : ring_t := (others => (others => (others => '0')));

    -- =========================================================================
    -- Post-trigger capture buffer (CLK_ADC domain)
    -- Filled with 4 batches that follow the trigger.
    -- =========================================================================
    signal post_buf : ring_t := (others => (others => (others => '0')));

    -- =========================================================================
    -- Ping-pong BRAM: 512 × 64-bit
    -- Buffer 0: addr   0–255   Buffer 1: addr 256–511
    -- True-dual-port inferred: Port A on CLK_ADC, Port B on CLK_CNN.
    -- Vivado inference attribute: "block"
    -- =========================================================================
    type bram_t is array(0 to 511) of std_logic_vector(63 downto 0);
    signal bram    : bram_t;
    attribute ram_style        : string;
    attribute ram_style of bram : signal is "block";

    -- BRAM write port (CLK_ADC)
    signal wr_en   : std_logic                    := '0';
    signal wr_addr : unsigned(8 downto 0)         := (others => '0');
    signal wr_data : std_logic_vector(63 downto 0) := (others => '0');

    -- BRAM read port (CLK_CNN), 1-cycle synchronous latency
    signal rd_addr : unsigned(8 downto 0)         := (others => '0');
    signal rd_data : std_logic_vector(63 downto 0);

    -- =========================================================================
    -- CDC signals — 4-phase set/clear handshake
    --
    -- Protocol (buffer N):
    --   ADC sets   buf_written_adc(N) = '1'  after finishing a BRAM write
    --   CDC sync → buf_written_cnn(N)         (2 CLK_CNN cycles latency)
    --   CNN sets   buf_ack_cnn(N)    = '1'  after CNN_DONE
    --   CDC sync → buf_ack_adc(N)            (2 CLK_ADC cycles latency)
    --   ADC clears buf_written_adc(N) = '0'  on seeing buf_ack_adc(N)='1'
    --   CDC sync → buf_written_cnn(N)='0'
    --   CNN clears buf_ack_cnn(N)    = '0'  on seeing buf_written_cnn(N)='0'
    -- =========================================================================
    signal buf_written_adc : std_logic_vector(1 downto 0) := (others => '0');
    signal buf_written_s1  : std_logic_vector(1 downto 0) := (others => '0');
    signal buf_written_cnn : std_logic_vector(1 downto 0) := (others => '0');

    signal buf_ack_cnn : std_logic_vector(1 downto 0) := (others => '0');
    signal buf_ack_s1  : std_logic_vector(1 downto 0) := (others => '0');
    signal buf_ack_adc : std_logic_vector(1 downto 0) := (others => '0');

    -- RST synchronizer (ADC → CNN domain)
    -- Initialized to '1' so RST_N_CNN starts at '0' (reset asserted) before
    -- any CLK_CNN edges.  '0' would make RST_N_CNN='1' at t=0, meaning
    -- cnn_core runs unreset and enters an undefined state before RST fires.
    signal rst_s1  : std_logic := '1';
    signal rst_cnn : std_logic := '1';

    -- =========================================================================
    -- ADC-domain FSM
    -- =========================================================================
    type adc_fsm_t is (ADC_IDLE, ADC_POST, ADC_WRITE);
    signal adc_state : adc_fsm_t := ADC_IDLE;

    signal post_cnt  : integer range 0 to 3  := 0;  -- post batches captured
    signal batch_cnt : integer range 0 to 7  := 0;  -- BRAM write: batch index
    signal samp_cnt  : integer range 0 to 31 := 0;  -- BRAM write: sample index
    signal write_sel : std_logic             := '0'; -- target ping-pong buffer

    -- =========================================================================
    -- CNN-domain FSM
    -- =========================================================================
    type cnn_fsm_t is (CC_IDLE, CC_STREAM, CC_WAIT_DONE, CC_ACK);
    signal cnn_state     : cnn_fsm_t    := CC_IDLE;

    signal cnn_base_addr : unsigned(8 downto 0) := (others => '0'); -- 0 or 256
    signal stream_ptr    : unsigned(7 downto 0) := (others => '0'); -- word 0..255
    signal cnn_buf_id    : integer range 0 to 1 := 0;
    -- rd_addr / rd_data / cnn_waiting removed: BRAM is now read directly inside
    -- the clocked CNN FSM process (no separate read process needed).

    -- =========================================================================
    -- Helper: pack one ADC sample (4 channels) into 64 bits
    -- Layout: [63:48]=ch3 [47:32]=ch2 [31:16]=ch1 [15:0]=ch0
    -- Each channel: 4-bit zero-pad || 12-bit ADC (two's complement)
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
    -- BRAM read is now performed directly inside the CNN FSM process below.
    -- Direct indexing in a clocked process gives exactly the same 1-cycle
    -- registered semantics as the old separate read process, but allows the
    -- CNN FSM to issue the read AND present valid data in consecutive cycles
    -- without a cnn_waiting bubble — matching tb_stream.sv's behaviour.

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
                write_sel       <= '0';
                wr_en           <= '0';
                wr_addr         <= (others => '0');
                wr_data         <= (others => '0');
                buf_written_adc <= (others => '0');
                CHUNK_OVERFLOW  <= '0';
                ring_buf        <= (others => (others => (others => '0')));
                post_buf        <= (others => (others => (others => '0')));
            else
                wr_en <= '0';  -- default: no BRAM write

                -- 4-phase handshake: ADC clears buf_written when it sees ack.
                -- Guard with buf_written='1' to prevent clearing a new write.
                if buf_ack_adc(0) = '1' and buf_written_adc(0) = '1' then
                    buf_written_adc(0) <= '0';
                end if;
                if buf_ack_adc(1) = '1' and buf_written_adc(1) = '1' then
                    buf_written_adc(1) <= '0';
                end if;

                case adc_state is

                    -- ----------------------------------------------------------
                    -- ADC_IDLE: continuously roll the ring buffer; watch for L0.
                    -- ----------------------------------------------------------
                    when ADC_IDLE =>
                        if DATA_STR = '1' then
                            -- Shift ring: [3]←[2]←[1]←[0]←current batch
                            ring_buf(3) <= ring_buf(2);
                            ring_buf(2) <= ring_buf(1);
                            ring_buf(1) <= ring_buf(0);
                            for s in 0 to 31 loop
                                ring_buf(0)(s) <= pack_sample(ADC_DATA4, s);
                            end loop;
                        end if;

                        if L0_PRE_TRIG = '1' then
                            -- A buffer is "free" only when both written AND ack
                            -- flags are clear (full 4-phase handshake complete).
                            if buf_written_adc(0) = '0' and buf_ack_adc(0) = '0' then
                                write_sel <= '0';
                                post_cnt  <= 0;
                                adc_state <= ADC_POST;
                            elsif buf_written_adc(1) = '0' and buf_ack_adc(1) = '0' then
                                write_sel <= '1';
                                post_cnt  <= 0;
                                adc_state <= ADC_POST;
                            else
                                -- Both buffers occupied — trigger dropped.
                                CHUNK_OVERFLOW <= '1';
                            end if;
                        end if;

                    -- ----------------------------------------------------------
                    -- ADC_POST: capture 4 post-trigger batches.
                    -- Ring buffer is now frozen (we left ADC_IDLE).
                    -- New triggers during this phase are dropped (overflow).
                    -- ----------------------------------------------------------
                    when ADC_POST =>
                        if L0_PRE_TRIG = '1' then
                            CHUNK_OVERFLOW <= '1';
                        end if;

                        if DATA_STR = '1' then
                            for s in 0 to 31 loop
                                post_buf(post_cnt)(s) <= pack_sample(ADC_DATA4, s);
                            end loop;

                            if post_cnt = 3 then
                                batch_cnt <= 0;
                                samp_cnt  <= 0;
                                adc_state <= ADC_WRITE;
                            else
                                post_cnt <= post_cnt + 1;
                            end if;
                        end if;

                    -- ----------------------------------------------------------
                    -- ADC_WRITE: stream 8 batches (256 words) into the selected
                    -- BRAM buffer at full CLK_ADC rate (no throttle needed).
                    --
                    -- Write order (chronological, oldest first):
                    --   batch 0-3 → ring_buf[3..0]  (128 pre-trigger samples)
                    --   batch 4-7 → post_buf[0..3]  (128 post-trigger samples)
                    -- ----------------------------------------------------------
                    when ADC_WRITE =>
                        if L0_PRE_TRIG = '1' then
                            CHUNK_OVERFLOW <= '1';
                        end if;

                        wr_en <= '1';

                        -- Address: {write_sel, batch_cnt[2:0], samp_cnt[4:0]}
                        if write_sel = '0' then
                            wr_addr <= to_unsigned(batch_cnt * 32 + samp_cnt, 9);
                        else
                            wr_addr <= to_unsigned(256 + batch_cnt * 32 + samp_cnt, 9);
                        end if;

                        -- Source: first 4 batches from ring (oldest→newest),
                        --         next 4 from post capture.
                        if batch_cnt < 4 then
                            wr_data <= ring_buf(3 - batch_cnt)(samp_cnt);
                        else
                            wr_data <= post_buf(batch_cnt - 4)(samp_cnt);
                        end if;

                        -- Advance sample then batch counters
                        if samp_cnt = 31 then
                            samp_cnt <= 0;
                            if batch_cnt = 7 then
                                wr_en <= '0';
                                -- Signal CNN domain: buffer is ready
                                if write_sel = '0' then
                                    buf_written_adc(0) <= '1';
                                else
                                    buf_written_adc(1) <= '1';
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
            else
                -- ap_ctrl_hs protocol: hold CNN_START (ap_start) high until
                -- CNN_READY (ap_ready) fires to confirm the CNN has accepted
                -- the start.  Do NOT use a default '0' here — that would clear
                -- it every cycle and the HLS core would never register it.
                if CNN_READY = '1' then
                    CNN_START <= '0';
                end if;

                -- 4-phase handshake: CNN clears its ack once the ADC has
                -- cleared buf_written (visible via buf_written_cnn going low).
                if buf_written_cnn(0) = '0' then
                    buf_ack_cnn(0) <= '0';
                end if;
                if buf_written_cnn(1) = '0' then
                    buf_ack_cnn(1) <= '0';
                end if;

                case cnn_state is

                    -- ----------------------------------------------------------
                    -- CC_IDLE: wait for a buffer to be ready.
                    -- Prefer buffer 0. Require CNN core to be idle.
                    -- Issue first BRAM read on entry to CC_STREAM.
                    -- ----------------------------------------------------------
                    -- ----------------------------------------------------------
                    -- CC_IDLE: wait for a buffer to be ready, then start the CNN
                    -- and present the first word simultaneously — exactly matching
                    -- tb_stream.sv which raises start and input_valid together.
                    -- ----------------------------------------------------------
                    when CC_IDLE =>
                        CNN_IN_VALID <= '0';

                        -- Guard: only start when the 4-phase ack handshake for
                        -- this buffer is fully idle (buf_ack_cnn='0').
                        -- Without this, CC_IDLE would re-trigger on the SAME
                        -- buffer immediately after CC_ACK sets buf_ack_cnn='1',
                        -- before the ADC domain has had time to clear
                        -- buf_written_adc via CDC — causing a duplicate inference.
                        if buf_written_cnn(0) = '1' and buf_ack_cnn(0) = '0' then
                            cnn_buf_id    <= 0;
                            cnn_base_addr <= (others => '0');
                            CNN_START     <= '1';
                            CNN_IN_VALID  <= '1';
                            CNN_IN_DATA   <= bram(0);   -- word 0 of buffer 0
                            stream_ptr    <= (others => '0');
                            cnn_state     <= CC_STREAM;

                        elsif buf_written_cnn(1) = '1' and buf_ack_cnn(1) = '0' then
                            cnn_buf_id    <= 1;
                            cnn_base_addr <= to_unsigned(256, 9);
                            CNN_START     <= '1';
                            CNN_IN_VALID  <= '1';
                            CNN_IN_DATA   <= bram(256); -- word 0 of buffer 1
                            stream_ptr    <= (others => '0');
                            cnn_state     <= CC_STREAM;
                        end if;

                    -- ----------------------------------------------------------
                    -- CC_STREAM: feed 256 × 64-bit words to the CNN via AXI-S.
                    --
                    -- Matches tb_stream.sv exactly:
                    --   • CNN_IN_VALID stays '1' for all 256 words (no bubbles).
                    --   • CNN_IN_DATA is pre-registered in the clocked process:
                    --       At CC_IDLE we assign CNN_IN_DATA = bram(base+0).
                    --       After READY='1' on word N we assign bram(base+N+1),
                    --       which appears on CNN_IN_DATA the next cycle. ✓
                    --   • CNN_START is held until CNN_READY fires (ap_ctrl_hs).
                    -- Throughput: 1 word per CLK_CNN cycle (200 MHz effective).
                    -- Back-pressure: CNN_IN_VALID held; CNN_IN_DATA unchanged.
                    -- ----------------------------------------------------------
                    when CC_STREAM =>
                        CNN_IN_VALID <= '1';

                        if CNN_IN_READY = '1' then
                            if stream_ptr = 255 then
                                CNN_IN_VALID <= '0';
                                cnn_state    <= CC_WAIT_DONE;
                            else
                                stream_ptr  <= stream_ptr + 1;
                                -- Pre-register the NEXT word so it is ready on
                                -- the following clock cycle — no bubble.
                                CNN_IN_DATA <= bram(to_integer(
                                                  cnn_base_addr
                                                  + resize(stream_ptr + 1, 9)));
                            end if;
                        end if;
                        -- CNN_IN_READY='0': hold VALID='1', DATA unchanged.

                    -- ----------------------------------------------------------
                    when CC_WAIT_DONE =>
                        CNN_IN_VALID <= '0';
                        if CNN_DONE = '1' then
                            cnn_state <= CC_ACK;
                        end if;

                    -- ----------------------------------------------------------
                    -- CC_ACK: release the buffer (4-phase handshake step 3).
                    -- buf_ack_cnn stays set until the ADC domain clears
                    -- buf_written_adc, which feeds back via buf_written_cnn.
                    -- ----------------------------------------------------------
                    when CC_ACK =>
                        CNN_IN_VALID <= '0';
                        if cnn_buf_id = 0 then
                            buf_ack_cnn(0) <= '1';
                        else
                            buf_ack_cnn(1) <= '1';
                        end if;
                        cnn_state <= CC_IDLE;

                end case;
            end if;
        end if;
    end process;

end rtl;
