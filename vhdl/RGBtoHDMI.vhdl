----------------------------------------------------------------------------------
-- Engineer:            David Banks & Ian Bradbury
--
-- Create Date:         15/7/2018
-- Module Name:         RGBtoHDMI CPLD
-- Project Name:        RGBtoHDMI
-- Target Devices:      XC9572XL
--
-- Version:             1.0
--
----------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity RGBtoHDMI is
    Generic (
        SupportAnalog : boolean := false
    );
    Port (
        -- From RGB Connector
        R3_I:      in    std_logic;
        G3_I:      in    std_logic;
        B3_I:      in    std_logic;
        R2_I:      in    std_logic;
        G2_I:      in    std_logic;
        B2_I:      in    std_logic;
        R1_I:      in    std_logic;
        G1_I:      in    std_logic;
        B1_I:      in    std_logic;
        R0_I:      in    std_logic;
        G0_I:      in    std_logic;
        B0_I:      in    std_logic;

        csync_I:  in    std_logic;
        vsync_I:  in    std_logic;
        analog:    inout std_logic;

        -- From Pi
        clk:       in    std_logic;
        sp_clk:    in    std_logic;
        sp_clken:  in    std_logic;
        sp_data:   in    std_logic;

        -- To PI GPIO
        quad:      out   std_logic_vector(11 downto 0);
        psync:     out   std_logic;
        csync:     out   std_logic;

        -- User interface
        version:   in    std_logic
    );
end RGBtoHDMI;

architecture Behavorial of RGBtoHDMI is

    -- Version number: Design_Major_Minor
    -- Design: 0 = BBC CPLD
    --         1 = Alternative CPLD
    --         2 = Atom CPLD
    --         3 = six bit CPLD (if required);
    --         4 = RGB CPLD (TTL)
    --         C = RGB CPLD (Analog)

    constant VERSION_NUM_BBC        : std_logic_vector(11 downto 0) := x"068";
    constant VERSION_NUM_RGB_TTL    : std_logic_vector(11 downto 0) := x"479";
    constant VERSION_NUM_RGB_ANALOG : std_logic_vector(11 downto 0) := x"C79";

    signal shift_R  : std_logic_vector(3 downto 0);
    signal shift_G  : std_logic_vector(3 downto 0);
    signal shift_B  : std_logic_vector(3 downto 0);

    signal csync1   : std_logic;
    signal csync2   : std_logic;
    signal last     : std_logic;

    signal csync_counter : unsigned(1 downto 0);

    -- The sampling counter runs at 96MHz
    -- - In modes 0..6 it is 6x  the pixel clock
    -- - In mode 7 it is 8x the pixel clock
    --
    -- It serves several purposes:
    -- 1. Counts the 12us between the rising edge of nCSYNC and the first pixel
    -- 2. Counts within each pixel (bits 0, 1, 2)
    -- 3. Counts counts pixels within a quad pixel (bits 3 and 4)
    -- 4. Handles double buffering of alternative quad pixels (bit 5)
    --
    -- At the moment we don't count pixels with the line, the Pi does that

    subtype counter_type is unsigned(7 downto 0);

    signal counter  : counter_type;

    -- Pipelined offset mux output
    signal offset   : std_logic_vector(2 downto 0);

    -- Index to cycle through offsets A..F
    signal index    : std_logic_vector(2 downto 0);

    -- Sample pixel on next clock; pipelined to reduce the number of product terms
    signal sample   : std_logic;

    -- New sample available, toggle psync on next cycle
    signal toggle   : std_logic;

    -- RGB Input Mux
    signal old_mux  : std_logic;
    signal mux_sync : std_logic;

    signal R        : std_logic;
    signal G        : std_logic;
    signal B        : std_logic;

    signal clamp_int    : std_logic;
    signal clamp_enable : std_logic;

    -- Sample point register;
    --
    -- In Mode 7 each pixel lasts 8 clocks (96MHz / 12MHz). The original
    -- pixel clock is a regenerated 6Mhz clock, and both edges are used.
    -- Due to the way it is generated, there are three distinct phases,
    -- each with different rising/falling edge speeds, hence six sampling
    -- points are used.
    --
    -- In Modes 0..6 each pixel lasts 6 clocks (96MHz / 16MHz). The original
    -- pixel clock is a clean 16Mhz clock, so only one sample point is needed.
    -- To achieve this, all six values are set to be the same. This minimises
    -- the logic in the CPLD.
	 
    -- Sampling points
    constant INIT_SAMPLING_POINTS : std_logic_vector(23 downto 0) := "000000011011011011011011";
	 
    signal sp_reg   : std_logic_vector(23 downto 0) := INIT_SAMPLING_POINTS;

    -- Break out of sp_reg
    signal offset_A : std_logic_vector(2 downto 0);
    signal offset_B : std_logic_vector(2 downto 0);
    signal offset_C : std_logic_vector(2 downto 0);
    signal offset_D : std_logic_vector(2 downto 0);
    signal offset_E : std_logic_vector(2 downto 0);
    signal offset_F : std_logic_vector(2 downto 0);
    signal half     : std_logic;
    signal delay    : unsigned(1 downto 0);
    signal rate     : std_logic_vector(1 downto 0);
    signal divider  : std_logic;

begin
    old_mux <= B0_I when not(SupportAnalog) else '0';   -- B0_I used to be mux_in from Pi GPIO

    R <= R2_I when old_mux = '1' else R3_I;
    G <= G2_I when old_mux = '1' else G3_I;
    B <= B2_I when old_mux = '1' else B3_I;

    offset_A <= sp_reg(2 downto 0);
    offset_B <= sp_reg(5 downto 3);
    offset_C <= sp_reg(8 downto 6);
    offset_D <= sp_reg(11 downto 9);
    offset_E <= sp_reg(14 downto 12);
    offset_F <= sp_reg(17 downto 15);
    half     <= sp_reg(18);
    delay    <= unsigned(sp_reg(20 downto 19));
    rate     <= sp_reg(22 downto 21);
    divider  <= sp_reg(23);

    -- Shift the bits in LSB first
    process(sp_clk)
    begin
        if rising_edge(sp_clk) then
            if sp_clken = '1' then
                sp_reg <= sp_data & sp_reg(sp_reg'left downto sp_reg'right + 1);
            end if;
        end if;
    end process;

    process(clk)
    begin
        if rising_edge(clk) then

            -- synchronize CSYNC to the sampling clock
            csync1 <= csync_I;

            -- De-glitch CSYNC
            --    csync1 is the possibly glitchy input
            --    csync2 is the filtered output
            if csync1 = csync2 then
                -- output same as input, reset the counter
                csync_counter <= to_unsigned(0, csync_counter'length);
            else
                -- output different to input
                csync_counter <= csync_counter + 1;
                -- if the difference lasts for N-1 cycles, update the output
                if csync_counter = 3 then
                    csync2 <= csync1;
                end if;
            end if;

            -- Counter is used to find sampling point for first pixel
            last <= csync2;
            -- reset counter on the rising edge of csync
            if last = '0' and csync2 = '1' then
                if rate(1) = '1' then
                    counter(7 downto 3) <= "10" & delay & "0";
                else
                    counter(7 downto 3) <= "110" & delay;
                end if;
                if half = '1' then
                    counter(2 downto 0) <= "000";
                elsif divider = '1' then
                    counter(2 downto 0) <= "100";
                else
                    counter(2 downto 0) <= "011";
                end if;
            elsif divider = '1' or counter(2 downto 0) /= 5 then
                if counter(counter'left) = '1' then
                    counter <= counter + 1;
                else
                    counter(counter'left - 1 downto 0) <= counter(counter'left - 1 downto 0) + 1;
                end if;
            else
                if counter(counter'left) = '1' then
                    counter <= counter + 3;
                else
                    counter(counter'left - 1 downto 0) <= counter(counter'left - 1 downto 0) + 3;
                end if;
            end if;

            -- Sample point offset index
            if counter(counter'left) = '1' then
                index <= "000";
            else
                -- so index offset changes at the same time counter wraps 7->0
                -- so index offset changes at the same time counter wraps ->0
                if (divider = '0' and counter(2 downto 0) = 4) or (divider = '1' and counter(2 downto 0) = 6) then
                    case index is
                        when "000" =>
                            index <= "001";
                        when "001" =>
                            index <= "010";
                        when "010" =>
                            index <= "011";
                        when "011" =>
                            index <= "100";
                        when "100" =>
                            index <= "101";
                        when others =>
                            index <= "000";
                    end case;
                end if;
            end if;

            -- Sample point offset
            case index is
                when "000" =>
                    offset <= offset_B;
                when "001" =>
                    offset <= offset_C;
                when "010" =>
                    offset <= offset_D;
                when "011" =>
                    offset <= offset_E;
                when "100" =>
                    offset <= offset_F;
                when others =>
                    offset <= offset_A;
            end case;

            -- sample/shift control
            if counter(counter'left) = '0' and counter(2 downto 0) = unsigned(offset) and (rate(1) = '0' or rate(0) = counter(3)) then
                sample <= '1';
            else
                sample <= '0';
            end if;

            -- R Sample/shift register
            if sample = '1' then
                if rate = "01" and sp_data = '0' then
                    shift_R <= R2_I & R3_I & shift_R(3 downto 2);     -- 6 bpp
                elsif rate = "00" and sp_data = '1' then
                    shift_R <= R1_I & G2_I & B3_I & B3_I;             -- 9 bpp
                elsif rate /= "00" and sp_data = '1' then
                    shift_R <= R1_I & G2_I & B3_I & B0_I;             -- 12 bpp
                else
                    shift_R <= R3_I & shift_R(3 downto 1);            -- 3 bpp
                end if;
            end if;

            -- G Sample/shift register
            if sample = '1' then
                if rate = "01" and sp_data = '0' then
                    shift_G <= G2_I & G3_I & shift_G(3 downto 2);     -- 6 bpp
                elsif rate = "00" and sp_data = '1'  then
                    shift_G <= R2_I & G3_I & G3_I & B1_I;             -- 9 bpp
                elsif rate /= "00" and sp_data = '1' then
                    shift_G <= R2_I & G3_I & G0_I & B1_I;             -- 12 bpp
                else
                    shift_G <= G3_I & shift_G(3 downto 1);            -- 3 bpp
                end if;
            end if;

            -- B Sample/shift register
            if sample = '1' then
                if rate = "01" and sp_data = '0' then
                    shift_B <= B2_I & B3_I & shift_B(3 downto 2);     -- 6 bpp
                elsif rate = "00" and sp_data = '1' then
                    shift_B <= R3_I & R3_I & vsync_I & B2_I;          -- 9 bpp with G1 on vsync_I
                elsif rate /= "00" and sp_data = '1' then
                    shift_B <= R3_I & R0_I & G1_I & B2_I;             -- 12 bpp
                else
                    shift_B <= B3_I & shift_B(3 downto 1);            -- 3 bpp
                end if;
            end if;

            -- Pipeline when to update the quad
            if counter(counter'left) = '0' and (
                (rate = "00" and sp_data = '0' and counter(4 downto 0) = 0)  or      -- 3bpp
                (rate = "01" and sp_data = '0' and counter(3 downto 0) = 0)  or      -- 6bpp
                (rate = "00" and sp_data = '1' and counter(2 downto 0) = 0)  or      -- 9bpp
                (rate = "01" and sp_data = '1' and counter(2 downto 0) = 0)  or      -- 12bpp
                (rate = "10" and sp_data = '0' and counter(5 downto 0) = 0)  or      -- subsample even 3bpp
                (rate = "11" and sp_data = '0' and counter(5 downto 0) = 8)  or      -- subsample odd  3bpp
                (rate = "10" and sp_data = '1' and counter(3 downto 0) = 0)  or      -- subsample even 12bpp
                (rate = "11" and sp_data = '1' and counter(3 downto 0) = 8)) then    -- subsample odd  12bpp
                -- toggle is asserted in cycle 1
                toggle <= '1';
            else
                toggle <= '0';
            end if;

            -- Output quad register
            if version = '0' then
                if SupportAnalog then
                    if analog = '1' then
                        quad <= VERSION_NUM_RGB_ANALOG;
                    else
                        quad <= VERSION_NUM_RGB_TTL;
                    end if;
                else
                    quad <= VERSION_NUM_BBC;
                end if;
            elsif counter(counter'left) = '1' then
                quad <= (others => '0');
            elsif toggle = '1' then
                -- quad changes at the start of cycle 2
                quad(11) <= shift_B(3);
                quad(10) <= shift_G(3);
                quad(9)  <= shift_R(3);
                quad(8)  <= shift_B(2);
                quad(7)  <= shift_G(2);
                quad(6)  <= shift_R(2);
                quad(5)  <= shift_B(1);
                quad(4)  <= shift_G(1);
                quad(3)  <= shift_R(1);
                quad(2)  <= shift_B(0);
                quad(1)  <= shift_G(0);
                quad(0)  <= shift_R(0);
            end if;

            -- Output a skewed version of psync
            if counter(counter'left) = '1' then
                psync <= '0';
            elsif counter(2 downto 0) = 2 then -- comparing with N gives N-1 cycles of skew
                if rate = "00" and sp_data = '0' then
                    psync <= counter(5); -- 3bpp
                elsif rate = "01" and sp_data = '0' then
                    psync <= counter(4); -- 6bpp
                elsif rate = "00" and sp_data = '1' then
                    psync <= counter(3); -- 9bpp one edge for every pixel
                elsif rate = "01" and sp_data = '1' then
                    psync <= counter(3); -- 12 bpp one edge for every pixel
                elsif sp_data = '0' and rate(0) = counter(5) then
                    psync <= counter(6); -- subsample
                elsif sp_data = '1' and rate(0) = counter(3) then
                    psync <= counter(4); -- subsample
                end if;
            end if;
        end if;
    end process;

    csync <= csync2; -- output the registered version to save a macro-cell

    analog_additions: if SupportAnalog generate
         -- csync2 is cleaned but delayed so OR with csync1 to remove delay on trailing edge of sync pulse
         -- spdata is overloaded as clamp on/off
        clamp_int <= not(csync1 or csync2) and sp_data;
        analog <= 'Z' when version = '0' else clamp_int;
    end generate;

end Behavorial;
