-- =============================================================================
-- HILO_CNN_TRIGGER_TB_WRAP.vhd
-- Simulation-only mixed-language bridge for tb_hilo_cnn_trigger.sv.
--
-- Vivado xsim cannot bind a SystemVerilog port directly to a VHDL
-- user-defined composite type (adc_data4_type).  This wrapper accepts a
-- flat std_logic_vector and unpacks it into the composite type.
--
-- Flat-vector packing convention (must match the SV generate block):
--   adc_flat[767 - (ch*16 + s)*12  -: 12]  ↔  adc_data4_type(ch)(s)
--   Total width: 4 channels × 16 samples × 12 bits = 768 bits
-- =============================================================================

library ieee;
use ieee.std_logic_1164.all;
use work.pre_trigger_pkg.all;

entity HILO_CNN_TRIGGER_TB_WRAP is
    port (
        CLK_ADC        : in  std_logic;
        CLK_CNN        : in  std_logic;
        RST            : in  std_logic;
        DATA_STR       : in  std_logic;
        -- Flat replacement for adc_data4_type (4 ch × 16 samples × 12 bits)
        ADC_DATA4_FLAT : in  std_logic_vector(767 downto 0);
        THRESH         : in  std_logic_vector(11 downto 0);
        HILO_WINDOW    : in  std_logic_vector( 4 downto 0);
        COINC_WINDOW   : in  std_logic_vector( 5 downto 0);
        BIN_THR        : in  std_logic_vector( 3 downto 0);
        L0_PRE_TRIG    : out std_logic;
        CNN_OUT_DATA   : out std_logic_vector(31 downto 0);
        CNN_OUT_VALID  : out std_logic;
        CNN_OUT_READY  : in  std_logic;
        CHUNK_OVERFLOW : out std_logic
    );
end entity HILO_CNN_TRIGGER_TB_WRAP;

architecture rtl of HILO_CNN_TRIGGER_TB_WRAP is
    signal adc_internal : adc_data4_type;
begin

    -- Unpack flat vector → 2-D array.
    -- Outer (ch): 0 to 3 — high-channel bits first in the flat vector.
    -- Inner (s):  0 to 31 — high-sample bits first within each channel.
    gen_ch : for ch in 0 to 3 generate
        gen_s : for s in 0 to 15 generate
            adc_internal(ch)(s) <=
                ADC_DATA4_FLAT(767 - (ch * 16 + s) * 12 downto
                               756 - (ch * 16 + s) * 12);
        end generate gen_s;
    end generate gen_ch;

    u_dut : entity work.HILO_CNN_TRIGGER
        port map (
            CLK_ADC        => CLK_ADC,
            CLK_CNN        => CLK_CNN,
            RST            => RST,
            DATA_STR       => DATA_STR,
            ADC_DATA4      => adc_internal,
            THRESH         => THRESH,
            HILO_WINDOW    => HILO_WINDOW,
            COINC_WINDOW   => COINC_WINDOW,
            BIN_THR        => BIN_THR,
            L0_PRE_TRIG    => L0_PRE_TRIG,
            CNN_OUT_DATA   => CNN_OUT_DATA,
            CNN_OUT_VALID  => CNN_OUT_VALID,
            CNN_OUT_READY  => CNN_OUT_READY,
            CHUNK_OVERFLOW => CHUNK_OVERFLOW
        );

end architecture rtl;
