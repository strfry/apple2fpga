-------------------------------------------------------------------------------
--
-- Papilio Pro top-level module, adapted from
--
-- DE2 top-level module for the Apple ][
--
-- Stephen A. Edwards, Columbia University, sedwards@cs.columbia.edu
--
-- From an original by Terasic Technology, Inc.
-- (DE2_TOP.v, part of the DE2 system board CD supplied by Altera)
--
-------------------------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Papilio_Top is

  port (
    -- Clocks
    
    CLOCK_50 : in std_logic;                       -- 50 MHz

    -- Buttons and switches
	 
    RESET : in std_logic;	 
    KEY : in std_logic_vector(3 downto 0);         -- Push buttons
    --SW : in std_logic_vector(17 downto 0);         -- DPDT switches

    -- LED displays

--    HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6, HEX7 -- 7-segment displays
--    : out unsigned(6 downto 0);
--    LEDG : out std_logic_vector(8 downto 0);       -- Green LEDs
--    LEDR : out std_logic_vector(17 downto 0);      -- Red LEDs

    -- RS-232 interface

--    UART_TXD : out std_logic;                      -- UART transmitter   
--    UART_RXD : in std_logic;                       -- UART receiver

    -- IRDA interface

    -- IRDA_TXD : out std_logic;                      -- IRDA Transmitter
--    IRDA_RXD : in std_logic;                       -- IRDA Receiver


    -- Taken from Hamster SD-RAM Controller:
	 
   -- Signals to/from the SDRAM chip
   DRAM_ADDR   : OUT   STD_LOGIC_VECTOR (12 downto 0);
   DRAM_BA      : OUT   STD_LOGIC_VECTOR (1 downto 0);
   DRAM_CAS_N   : OUT   STD_LOGIC;
   DRAM_CKE      : OUT   STD_LOGIC;
   DRAM_CLK      : OUT   STD_LOGIC;
   DRAM_CS_N   : OUT   STD_LOGIC;
   DRAM_DQ      : INOUT STD_LOGIC_VECTOR(15 downto 0);
   DRAM_DQM      : OUT   STD_LOGIC_VECTOR(1 downto 0);
   DRAM_RAS_N   : OUT   STD_LOGIC;
   DRAM_WE_N    : OUT   STD_LOGIC;

    -- FLASH
    
--    FL_DQ : inout std_logic_vector(7 downto 0);      -- Data bus
--    FL_ADDR : out std_logic_vector(21 downto 0);  -- Address bus
--    FL_WE_N,                                         -- Write Enable
--    FL_RST_N,                                        -- RESET
--    FL_OE_N,                                         -- Output Enable
--    FL_CE_N : out std_logic;                         -- Chip Enable

--    -- 16 X 2 LCD Module
--    
--    LCD_ON,                     -- Power ON/OFF
--    LCD_BLON,                   -- Back Light ON/OFF
--    LCD_RW,                     -- Read/Write Select, 0 = Write, 1 = Read
--    LCD_EN,                     -- Enable
--    LCD_RS : out std_logic;     -- Command/Data Select, 0 = Command, 1 = Data
--    LCD_DATA : inout std_logic_vector(7 downto 0); -- Data bus 8 bits

--    -- SD card interface
--    
--    SD_DAT : in std_logic;      -- SD Card Data      SD pin 7 "DAT 0/DataOut"
--    SD_DAT3 : out std_logic;    -- SD Card Data 3    SD pin 1 "DAT 3/nCS"
--    SD_CMD : out std_logic;     -- SD Card Command   SD pin 2 "CMD/DataIn"
--    SD_CLK : out std_logic;     -- SD Card Clock     SD pin 5 "CLK"

--    -- USB JTAG link
--    
--    TDI,                        -- CPLD -> FPGA (data in)
--    TCK,                        -- CPLD -> FPGA (clk)
--    TCS : in std_logic;         -- CPLD -> FPGA (CS)
--    TDO : out std_logic;        -- FPGA -> CPLD (data out)


    -- PS/2 ports A and B

    PS2A_DAT,                    -- Data
    PS2A_CLK : in std_logic;     -- Clock
    PS2B_DAT,                    -- Data
    PS2B_CLK : in std_logic;     -- Clock

    -- VGA output
    
--    VGA_CLK,                                            -- Clock
    VGA_HS,                                             -- H_SYNC
    VGA_VS,                                             -- V_SYNC
--    VGA_BLANK,                                          -- BLANK
    VGA_SYNC : out std_logic;                           -- SYNC
    VGA_R : out unsigned(2 downto 0);                   -- Red[2:0]
    VGA_G : out unsigned(2 downto 0);                   -- Green[2:0]
    VGA_B : out unsigned(1 downto 0);                   -- Blue[1:0]

    
    -- Audio Output
    
	 AUDIO_L,
    AUDIO_R : out std_logic                          -- RESET
    
    );
  
end Papilio_Top;

architecture datapath of Papilio_Top is

  signal CLK_14M, CLK_2M, PRE_PHASE_ZERO : std_logic;
  signal IO_SELECT, DEVICE_SELECT : std_logic_vector(7 downto 0);
  signal ADDR : unsigned(15 downto 0);
  signal D, PD : unsigned(7 downto 0);

  signal ram_we : std_logic;
  signal VIDEO, HBL, VBL, LD194 : std_logic;
  signal COLOR_LINE : std_logic;
  signal COLOR_LINE_CONTROL : std_logic;
  signal GAMEPORT : std_logic_vector(7 downto 0);
  signal cpu_pc : unsigned(15 downto 0);

  signal K : unsigned(7 downto 0);
  signal read_key : std_logic;

  signal flash_clk : unsigned(22 downto 0);

  signal track : unsigned(5 downto 0);
  signal trackmsb : unsigned(3 downto 0);
  signal D1_ACTIVE, D2_ACTIVE : std_logic;
  signal track_addr : unsigned(13 downto 0);
  signal TRACK_RAM_ADDR : unsigned(13 downto 0);
  signal tra : unsigned(15 downto 0);
  signal TRACK_RAM_DI : unsigned(7 downto 0);
  signal TRACK_RAM_WE : std_logic;

  signal CS_N, MOSI, MISO, SCLK : std_logic;
  
  type bram_type is array (0 to 4095) of unsigned(7 downto 0);
  signal BRAM : bram_type;
  
  signal BRAM_WE : std_logic;
  signal BRAM_DQ : unsigned(7 downto 0);
  signal BRAM_ADDR : unsigned(15 downto 0);

begin

  -- In the Apple ][, this was a 555 timer
  flash_clkgen : process (CLK_14M)
  begin
    if rising_edge(CLK_14M) then
      flash_clk <= flash_clk + 1;
    end if;     
  end process;
  
  bram_4k : process (CLK_14M)
  variable address_4k : integer;
  begin
	 address_4k := to_integer(unsigned(BRAM_ADDR(11 downto 0)));
    if rising_edge(CLK_14M) then
		if BRAM_WE = '1' then
			BRAM(address_4k) <= D;
		end if;
		
		BRAM_DQ <= BRAM(address_4k);
		
	 end if;
  end process;


--  pll : entity work.kbd_intf port map (
--    clk_in1   => CLOCK_50,
--    clk_out1  => CLK_14M
--    );

  -- Paddle buttons
  GAMEPORT <=  "0000" & (not KEY(3 downto 0)) & "0";

  --COLOR_LINE_CONTROL <= COLOR_LINE and SW(17);  -- Color or B&W mode
  
  core : entity work.apple2 port map (
    CLK_14M        => CLK_14M,
    CLK_2M         => CLK_2M,
    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
    FLASH_CLK      => flash_clk(22),
    RESET          => RESET,
    ADDR           => ADDR,
    ram_addr       => BRAM_ADDR(15 downto 0),
    D              => D,
    ram_do         => BRAM_DQ(7 downto 0),
    PD             => PD,
    ram_we         => ram_we,
    VIDEO          => VIDEO,
    COLOR_LINE     => COLOR_LINE,
    HBL            => HBL,
    VBL            => VBL,
    LD194          => LD194,
    K              => K,
    read_key       => read_key,
--    AN             => LEDR(3 downto 0),
    GAMEPORT       => GAMEPORT,
    IO_SELECT      => IO_SELECT,
    DEVICE_SELECT  => DEVICE_SELECT,
    pcDebugOut     => cpu_pc
--    speaker        => LEDG(0)
    );

--  vga : entity work.vga_controller port map (
--    CLK_14M    => CLK_14M,
--    VIDEO      => VIDEO,
--    COLOR_LINE => COLOR_LINE_CONTROL,
--    HBL        => HBL,
--    VBL        => VBL,
--    LD194      => LD194,
--    VGA_CLK    => VGA_CLK,
--    VGA_HS     => VGA_HS,
--    VGA_VS     => VGA_VS,
--    VGA_BLANK  => VGA_BLANK,
--    VGA_R      => VGA_R,
--    VGA_G      => VGA_G,
--    VGA_B      => VGA_B
--    );
--
--  VGA_SYNC <= '0';

  keyboard : entity work.kbd_intf port map (
    PS2_Clk  => PS2A_CLK,
    PS2_Data => PS2A_DAT,
    CLK_14M  => CLK_14M,
    RESET    => RESET,
    read_kb  => read_key,
    K => K
    );

--  disk : entity work.disk_ii port map (
--    CLK_14M        => CLK_14M,
--    CLK_2M         => CLK_2M,
--    PRE_PHASE_ZERO => PRE_PHASE_ZERO,
--    IO_SELECT      => IO_SELECT(6),
--    DEVICE_SELECT  => DEVICE_SELECT(6),
--    RESET          => RESET,
--    A              => ADDR,
--    D_IN           => D,
--    D_OUT          => PD,
--    TRACK          => TRACK,
--    TRACK_ADDR     => TRACK_ADDR,
--    D1_ACTIVE      => D1_ACTIVE,
--    D2_ACTIVE      => D2_ACTIVE,
--    ram_write_addr => TRACK_RAM_ADDR,
--    ram_di         => TRACK_RAM_DI,
--    ram_we         => TRACK_RAM_WE
--    );

--  sdcard_interface : entity work.spi_controller port map (
--    CLK_14M        => CLK_14M,
--    RESET          => RESET,
--
--    CS_N           => CS_N,
--    MOSI           => MOSI,
--    MISO           => MISO,
--    SCLK           => SCLK,
--    
--    track          => TRACK,
--    
--    ram_write_addr => TRACK_RAM_ADDR,
--    ram_di         => TRACK_RAM_DI,
--    ram_we         => TRACK_RAM_WE
--    );
--
--  SD_DAT3 <= CS_N;
--  SD_CMD  <= MOSI;
--  MISO    <= SD_DAT;
--  SD_CLK  <= SCLK;

--  digit0 : entity work.hex7seg port map (cpu_pc( 3 downto  0), HEX0);
--  digit1 : entity work.hex7seg port map (cpu_pc( 7 downto  4), HEX1);
--  digit2 : entity work.hex7seg port map (cpu_pc(11 downto  8), HEX2);
--  digit3 : entity work.hex7seg port map (cpu_pc(15 downto 12), HEX3);



--  SRAM_DQ(7 downto 0) <= D when ram_we = '1' else (others => 'Z');
--  SRAM_ADDR(17) <= '0';
--  SRAM_ADDR(16) <= '0';
--  SRAM_UB_N <= '1';
--  SRAM_LB_N <= '0';
--  SRAM_CE_N <= '0';
--  SRAM_WE_N <= not ram_we;
--  SRAM_OE_N <= ram_we;

  
--
--  UART_TXD <= '0';
--  DRAM_ADDR <= (others => '0');
--  DRAM_LDQM <= '0';
--  DRAM_UDQM <= '0';
--  DRAM_WE_N <= '1';
--  DRAM_CAS_N <= '1';
--  DRAM_RAS_N <= '1';
--  DRAM_CS_N <= '1';
--  DRAM_BA_0 <= '0';
--  DRAM_BA_1 <= '0';
--  DRAM_CLK <= '0';
--  DRAM_CKE <= '0';
---- Set all bidirectional ports to tri-state
--  DRAM_DQ     <= (others => 'Z');

end datapath;
