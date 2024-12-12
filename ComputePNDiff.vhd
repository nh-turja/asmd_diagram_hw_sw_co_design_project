library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.all;

library work;
use work.DataTypes_pkg.all;


entity ComputePNDiff is
   port( 
      Clk: in  std_logic;
      RESET: in std_logic;
      start: in std_logic;
      ready: out std_logic;
      PNL_BRAM_addr: out std_logic_vector(PNL_BRAM_ADDR_SIZE_NB-1 downto 0);
      PNL_BRAM_din: out std_logic_vector(PNL_BRAM_DBITS_WIDTH_NB-1 downto 0);
      PNL_BRAM_dout: in std_logic_vector(PNL_BRAM_DBITS_WIDTH_NB-1 downto 0);
      PNL_BRAM_we: out std_logic_vector(0 to 0)
      );
end ComputePNDiff;


architecture beh of ComputePNDiff is


   type state_type is (idle, get_lower_BRAM, store_lower_BRAM, get_upper_BRAM, store_upper_BRAM, 
			do_subtraction, store_in_PNDiff, check_loop_bound);
  
   signal state_reg, state_next: state_type;

   signal ready_reg, ready_next: std_logic;

   -- For selecting between PN or PN diffs portion of memory during memory accesses
   signal do_PN_diff_addr: std_logic_vector(1 downto 0);
   
   -- Address registers for the PNs and PN diff portions of memory
   signal PN_lower_addr_reg, PN_lower_addr_next: unsigned(PNL_BRAM_ADDR_SIZE_NB-1 downto 0);
   signal PN_upper_addr_reg, PN_upper_addr_next: unsigned(PNL_BRAM_ADDR_SIZE_NB-1 downto 0);
   signal diff_addr_reg, diff_addr_next: unsigned(PNL_BRAM_ADDR_SIZE_NB-1 downto 0);
   
--For loop initialization (init_reg=1 means base address)
   signal init_reg,init_next: std_logic;   
-- The registers used to store the upper and lower pn values being read in
-- we need them to be 16 bits to match
   signal lower_value_reg, lower_value_next: signed(15 downto 0);
   signal upper_value_reg, upper_value_next: signed(15 downto 0);
--for storing the difference before writeback 
   signal diff_value_reg, diff_value_next: signed(15 downto 0);
   
   begin
   
-- =============================================================================================
-- State and register logic
-- =============================================================================================
   process(Clk, RESET)
      begin
      if ( RESET = '1' ) then
         state_reg         <= idle;
         ready_reg         <= '1';
         PN_upper_addr_reg <= (others => '0');
         PN_lower_addr_reg <= (others => '0');
         diff_addr_reg     <= (others => '0');
         lower_value_reg   <= (others => '0');
         upper_value_reg   <= (others => '0');
		 diff_value_reg    <= (others => '0');
		 init_reg		   <= '1';
      elsif ( Clk'event and Clk = '1' ) then
         state_reg           <= state_next;
         ready_reg           <= ready_next;
         PN_lower_addr_reg   <= PN_lower_addr_next;
         PN_upper_addr_reg   <= PN_upper_addr_next;
         diff_addr_reg       <= diff_addr_next;
         lower_value_reg     <= lower_value_next;
         upper_value_reg     <= upper_value_next;
		 diff_value_reg		 <= diff_value_next;
		 init_reg			 <= init_next;
      end if; 
   end process;

   
   process (state_reg, start, ready_reg, PN_lower_addr_reg, PN_upper_addr_reg, 
   diff_addr_reg, diff_value_reg, init_reg, PNL_BRAM_dout, lower_value_reg, upper_value_reg)
      begin

         state_next <= state_reg;
         ready_next <= ready_reg;
	  
         PN_upper_addr_next <= PN_upper_addr_reg;
         PN_lower_addr_next <= PN_lower_addr_reg;
         diff_addr_next     <= diff_addr_reg;

         lower_value_next     <= lower_value_reg;
         upper_value_next     <= upper_value_reg;
		 diff_value_next 	  <= diff_value_reg;
		 init_next 			  <= init_reg;
-- Default value is 0 -- used during memory initialization
		 PNL_BRAM_din <= (others=>'0');
         PNL_BRAM_we <= "0";
	  
	 -- Select the PN_lower_addr_reg by default 	 
	 do_PN_diff_addr <= "00";
	  
	 case state_reg is

-- =====================
         when idle =>
            ready_next <= '1';
			
            if ( start = '1' ) then
               ready_next <= '0';
			   diff_value_next  <= (others=>'0');
			   
			   -- Allow diff_addr to drive PNL_BRAM
			   init_next <= '1';
               state_next <= get_upper_BRAM;

            end if;
-- =====================

		 when get_upper_BRAM =>
			do_PN_diff_addr <= "10";
			PNL_BRAM_we <= "0";
			if(init_reg ='1') then
				PN_upper_addr_next <= to_unsigned(PN_UPPER_BRAM_BASE, PNL_BRAM_ADDR_SIZE_NB);
				init_next <= '0';
			else
				PN_upper_addr_next <= PN_upper_addr_reg + 1;
				init_next <= '0';
				
			end if;
			state_next <= store_upper_BRAM;
			
-- =====================
		 when store_upper_BRAM =>
			do_PN_diff_addr <= "10";
			upper_value_next <= signed(PNL_BRAM_dout);
			
			if(init_reg = '1') then
				init_next <= '1';
			else 
				init_next <= '0';
			end if;
			state_next <= get_lower_BRAM;
-- =====================
		 when get_lower_BRAM =>
			do_PN_diff_addr <= "01";
			if(init_reg ='1') then
				PN_lower_addr_next <= to_unsigned(PN_BRAM_BASE, PNL_BRAM_ADDR_SIZE_NB);
				init_next <= '0';
			else
				PN_lower_addr_next <= PN_lower_addr_reg + 1;
				init_next <= '0';
				
			end if;
			state_next <= store_lower_BRAM;
-- =====================
		 when store_lower_BRAM =>
			do_PN_diff_addr <= "01";
			lower_value_next <= signed(PNL_BRAM_dout);
			
			if(init_reg = '1') then
				init_next <= '1';
			else 
				init_next <= '0';
			end if;
			state_next <= do_subtraction;
-- =====================
		 when do_subtraction =>
			diff_value_next<= upper_value_reg - lower_value_reg;
			if(init_reg = '1') then 
				init_next <= '1';
			else
				init_next <= '0';
			end if;
			state_next <= store_in_PNDiff;
-- =====================
		 when store_in_PNDiff => 
			do_PN_diff_addr <= "11";
			PNL_BRAM_we <= "1";
			if(init_reg = '1') then
				diff_addr_next <= to_unsigned(DIFF_BRAM_BASE, PNL_BRAM_ADDR_SIZE_NB);
				init_next <= '0';
			else
				diff_addr_next <= diff_addr_reg + 1; 
				init_next <= '0';
			end if;
			state_next <= check_loop_bound; 

-- =====================
		when check_loop_bound => 
			PNL_BRAM_we <= "1";
			do_PN_diff_addr <= "11";
			PNL_BRAM_din <= std_logic_vector(signed(diff_value_reg));
			if(PN_upper_addr_reg = PN_UPPER_LIMIT - 1) then
				init_next <= '1';
				state_next <= idle;
			else
				init_next <= '0';
				state_next <= get_lower_BRAM;
			end if; 
         
			
	end case;
  end process;
  
  -- Using _reg here (not the look-ahead _next value).
   with do_PN_diff_addr select
      PNL_BRAM_addr <= std_logic_vector(PN_lower_addr_reg) when "01",
                       std_logic_vector(PN_upper_addr_reg) when "10",
                       std_logic_vector(diff_addr_reg) when others;


   ready <= ready_reg;

  

end beh;









