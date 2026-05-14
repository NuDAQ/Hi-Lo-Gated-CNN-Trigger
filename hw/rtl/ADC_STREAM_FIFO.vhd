-- Copyright 2026 Albert L. Cheung @ University of California, Irvine
-- SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.pre_trigger_pkg.all;

-- ----------------------------------------------------------------------------
-- ADC_STREAM_FIFO
-- ----------------------------------------------------------------------------

entity ADC_STREAM_FIFO is
    generic (
        DEPTH : integer := 8   -- number of 16-sample batches to buffer (≥ 2)
    );
    port (
        CLK_ADC      : in  std_logic;
        RST          : in  std_logic;   -- active-high synchronous

        -- From ADC
        DATA_STR_IN  : in  std_logic;
        ADC_DATA4_IN : in  adc_data4_type;

        -- To trigger chain (PRE_TRIGGER / CNN_CHUNK_CAPTURE)
        DATA_STR_OUT  : out std_logic;
        ADC_DATA4_OUT : out adc_data4_type;

        -- Status (CLK_ADC domain)
        OVERFLOW : out std_logic;  -- sticky: batch dropped (FIFO full). RST clears.
        EMPTY    : out std_logic
    );
end ADC_STREAM_FIFO;

architecture rtl of ADC_STREAM_FIFO is

    constant WORD_W : integer := 4 * 16 * 12;  -- 768 bits per batch

    type fifo_mem_t is array(0 to DEPTH-1) of std_logic_vector(WORD_W-1 downto 0);
    signal mem : fifo_mem_t := (others => (others => '0'));

    signal wr_ptr : integer range 0 to DEPTH-1 := 0;
    signal rd_ptr : integer range 0 to DEPTH-1 := 0;
    signal count  : integer range 0 to DEPTH   := 0;

    function pack_adc(d : adc_data4_type) return std_logic_vector is
        variable flat : std_logic_vector(WORD_W-1 downto 0);
    begin
        for ch in 0 to 3 loop
            for s in 0 to 15 loop
                flat(WORD_W-1  - (ch*16+s)*12 downto
                     WORD_W-12 - (ch*16+s)*12) := d(ch)(s);
            end loop;
        end loop;
        return flat;
    end function;

    function unpack_adc(flat : std_logic_vector(WORD_W-1 downto 0))
        return adc_data4_type is
        variable d : adc_data4_type;
    begin
        for ch in 0 to 3 loop
            for s in 0 to 15 loop
                d(ch)(s) := flat(WORD_W-1  - (ch*16+s)*12 downto
                                 WORD_W-12 - (ch*16+s)*12);
            end loop;
        end loop;
        return d;
    end function;

begin

    EMPTY <= '1' when count = 0 else '0';

    process(CLK_ADC)
        variable do_push : boolean;
        variable do_pop  : boolean;
    begin
        if rising_edge(CLK_ADC) then
            if RST = '1' then
                wr_ptr        <= 0;
                rd_ptr        <= 0;
                count         <= 0;
                DATA_STR_OUT  <= '0';
                ADC_DATA4_OUT <= (others => (others => (others => '0')));
                OVERFLOW      <= '0';
            else
                do_push := (DATA_STR_IN = '1') and (count < DEPTH);
                do_pop  := (count > 0);

                DATA_STR_OUT <= '0';

                -- Push: write incoming batch into FIFO
                if do_push then
                    mem(wr_ptr) <= pack_adc(ADC_DATA4_IN);
                    if wr_ptr = DEPTH - 1 then wr_ptr <= 0;
                    else                        wr_ptr <= wr_ptr + 1;
                    end if;
                end if;

                -- Overflow: incoming batch but FIFO is full
                if DATA_STR_IN = '1' and count = DEPTH then
                    OVERFLOW <= '1';
                end if;

                -- Pop: output oldest batch, one per CLK_ADC cycle
                if do_pop then
                    ADC_DATA4_OUT <= unpack_adc(mem(rd_ptr));
                    DATA_STR_OUT  <= '1';
                    if rd_ptr = DEPTH - 1 then rd_ptr <= 0;
                    else                        rd_ptr <= rd_ptr + 1;
                    end if;
                end if;

                -- Update occupancy counter
                if    do_push and not do_pop then count <= count + 1;
                elsif do_pop  and not do_push then count <= count - 1;
                end if;
            end if;
        end if;
    end process;

end rtl;
