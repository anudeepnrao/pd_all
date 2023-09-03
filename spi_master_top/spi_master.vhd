library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_unsigned.all;

--================================================================================================================
-- SYNTHESIS CONSIDERATIONS
-- ========================
-- There are several output ports that are used to simulate and verify the core operation. 
-- Do not map any signals to the unused ports, and the synthesis tool will remove the related interfacing
-- circuitry. 
-- The same is valid for the transmit and receive ports. If the receive ports are not mapped, the
-- synthesis tool will remove the receive logic from the generated circuitry.
-- Alternatively, you can remove these ports and related circuitry once the core is verified and
-- integrated to your circuit.
--================================================================================================================

entity spi_master is
    Generic (   
        N : positive := 32;                                             -- 32bit serial word length is default
        CPOL : std_logic := '0';                                        -- SPI mode selection (mode 0 default)
        CPHA : std_logic := '0';                                        -- CPOL = clock polarity, CPHA = clock phase.
        PREFETCH : positive := 2;                                       -- prefetch lookahead cycles
        SPI_2X_CLK_DIV : positive := 5);                                -- for a 100MHz sclk_i, yields a 10MHz SCK
    Port (  
        sclk_i : in std_logic := 'X';                                   -- high-speed serial interface system clock
        pclk_i : in std_logic := 'X';                                   -- high-speed parallel interface system clock
        rst_i : in std_logic := 'X';                                    -- reset core
        ---- serial interface ----
        spi_ssel_o : out std_logic;                                     -- spi bus slave select line
        spi_sck_o : out std_logic;                                      -- spi bus sck
        spi_mosi_o : out std_logic;                                     -- spi bus mosi output
        spi_miso_i : in std_logic := 'X';                               -- spi bus spi_miso_i input
        ---- parallel interface ----
        di_req_o : out std_logic;                                       -- preload lookahead data request line
        di_i : in  std_logic_vector (N-1 downto 0) := (others => 'X');  -- parallel data in (clocked on rising spi_clk after last bit)
        wren_i : in std_logic := 'X';                                   -- user data write enable, starts transmission when interface is idle
        wr_ack_o : out std_logic;                                       -- write acknowledge
        do_valid_o : out std_logic;                                     -- do_o data valid signal, valid during one spi_clk rising edge.
        do_o : out  std_logic_vector (N-1 downto 0);                    -- parallel output (clocked on rising spi_clk after last bit)
        --- debug ports: can be removed or left unconnected for the application circuit ---
        sck_ena_o : out std_logic;                                      -- debug: internal sck enable signal
        sck_ena_ce_o : out std_logic;                                   -- debug: internal sck clock enable signal
        do_transfer_o : out std_logic;                                  -- debug: internal transfer driver
        wren_o : out std_logic;                                         -- debug: internal state of the wren_i pulse stretcher
        rx_bit_reg_o : out std_logic;                                   -- debug: internal rx bit
        state_dbg_o : out std_logic_vector (3 downto 0);                -- debug: internal state register
        core_clk_o : out std_logic;
        core_n_clk_o : out std_logic;
        core_ce_o : out std_logic;
        core_n_ce_o : out std_logic;
        sh_reg_dbg_o : out std_logic_vector (N-1 downto 0)              -- debug: internal shift register
    );                      
end spi_master;

--================================================================================================================
-- this architecture is a pipelined register-transfer description.
-- all signals are clocked at the rising edge of the system clock 'sclk_i'.
--================================================================================================================
architecture rtl of spi_master is
    -- core clocks, generated from 'sclk_i': initialized at GSR to differential values
    signal core_clk     : std_logic := '0';     -- continuous core clock, positive logic
    signal core_n_clk   : std_logic := '1';     -- continuous core clock, negative logic
    signal core_ce      : std_logic := '0';     -- core clock enable, positive logic
    signal core_n_ce    : std_logic := '1';     -- core clock enable, negative logic
    -- spi bus clock, generated from the CPOL selected core clock polarity
    signal spi_2x_ce    : std_logic := '1';     -- spi_2x clock enable
    signal spi_clk      : std_logic := '0';     -- spi bus output clock
    signal spi_clk_reg  : std_logic;            -- output pipeline delay for spi sck (do NOT global initialize)
    -- core fsm clock enables
    signal fsm_ce       : std_logic := '1';     -- fsm clock enable
    signal sck_ena_ce   : std_logic := '1';     -- SCK clock enable
    signal samp_ce      : std_logic := '1';     -- data sampling clock enable
    --
    -- GLOBAL RESET: 
    --      all signals are initialized to zero at GSR (global set/reset) by giving explicit
    --      initialization values at declaration. This is needed for all Xilinx FPGAs, and 
    --      especially for the Spartan-6 and newer CLB architectures, where a async reset can
    --      reduce the usability of the slice registers, due to the need to share the control 
    --      set (RESET/PRESET, CLOCK ENABLE and CLOCK) by all 8 registers in a slice.
    --      By using GSR for the initialization, and reducing async RESET local init to the bare
    --      essential, the model achieves better LUT/FF packing and CLB usability.
    --
    -- internal state signals for register and combinatorial stages
    signal state_next : natural range N+1 downto 0 := 0;
    signal state_reg : natural range N+1 downto 0 := 0;
    -- shifter signals for register and combinatorial stages
    signal sh_next : std_logic_vector (N-1 downto 0);
    signal sh_reg : std_logic_vector (N-1 downto 0);
    -- input bit sampled buffer
    signal rx_bit_reg : std_logic := '0';
    -- buffered di_i data signals for register and combinatorial stages
    signal di_reg : std_logic_vector (N-1 downto 0);
    -- internal wren_i stretcher for fsm combinatorial stage
    signal wren : std_logic;
    signal wr_ack_next : std_logic := '0';
    signal wr_ack_reg : std_logic := '0';
    -- internal SSEL enable control signals
    signal ssel_ena_next : std_logic := '0';
    signal ssel_ena_reg : std_logic := '0';
    -- internal SCK enable control signals
    signal sck_ena_next : std_logic;
    signal sck_ena_reg : std_logic;
    -- buffered do_o data signals for register and combinatorial stages
    signal do_buffer_next : std_logic_vector (N-1 downto 0);
    signal do_buffer_reg : std_logic_vector (N-1 downto 0);
    -- internal signal to flag transfer to do_buffer_reg
    signal do_transfer_next : std_logic := '0';
    signal do_transfer_reg : std_logic := '0';
    -- internal input data request signal 
    signal di_req_next : std_logic := '0';
    signal di_req_reg : std_logic := '0';
    -- cross-clock do_transfer_reg -> do_valid_o_reg pipeline
    signal do_valid_A : std_logic := '0';
    signal do_valid_B : std_logic := '0';
    signal do_valid_C : std_logic := '0';
    signal do_valid_D : std_logic := '0';
    signal do_valid_next : std_logic := '0';
    signal do_valid_o_reg : std_logic := '0';
    -- cross-clock di_req_reg -> di_req_o_reg pipeline
    signal di_req_o_A : std_logic := '0';
    signal di_req_o_B : std_logic := '0';
    signal di_req_o_C : std_logic := '0';
    signal di_req_o_D : std_logic := '0';
    signal di_req_o_next : std_logic := '1';
    signal di_req_o_reg : std_logic := '1';
begin
    --=============================================================================================
    --  GENERICS CONSTRAINTS CHECKING
    --=============================================================================================
    -- minimum word width is 8 bits
    assert N >= 8
    report "Generic parameter 'N' (shift register size) needs to be 8 bits minimum"
    severity FAILURE;
    -- minimum prefetch lookahead check
    assert PREFETCH >= 1
    report "Generic parameter 'PREFETCH' (lookahead count) needs to be 1 minimum"
    severity FAILURE;
    -- maximum prefetch lookahead check
    assert PREFETCH <= N-5
    report "Generic parameter 'PREFETCH' (lookahead count) out of range, needs to be N-5 maximum"
    severity FAILURE;
    -- SPI_2X_CLK_DIV clock divider value must not be zero
    assert SPI_2X_CLK_DIV > 0
    report "Generic parameter 'SPI_2X_CLK_DIV' must not be zero"
    severity FAILURE;

    --=============================================================================================
    --  CLOCK GENERATION
    --=============================================================================================
    -- In order to preserve global clocking resources, the core clocking scheme is completely based 
    -- on using clock enables to process the serial high-speed clock at lower rates for the core fsm,
    -- the spi clock generator and the input sampling clock.
    -- The clock generation block derives 2 continuous antiphase signals from the 2x spi base clock 
    -- for the core clocking.
    -- The 2 clock phases are generated by separate and synchronous FFs, and should have only 
    -- differential interconnect delay skew.
    -- Clock enable signals are generated with the same phase as the 2 core clocks, and these clock 
    -- enables are used to control clocking of all internal synchronous circuitry. 
    -- The clock enable phase is selected for serial input sampling, fsm clocking, and spi SCK output, 
    -- based on the configuration of CPOL and CPHA.
    -- Each phase is selected so that all the registers can be clocked with a rising edge on all SPI
    -- modes, by a single high-speed global clock, preserving clock resources and clock to data skew.
    -----------------------------------------------------------------------------------------------
    -- generate the 2x spi base clock enable from the serial high-speed input clock
    spi_2x_ce_gen_proc: process (sclk_i) is
        variable clk_cnt : integer range SPI_2X_CLK_DIV-1 downto 0 := 0;
    begin
        if sclk_i'event and sclk_i = '1' then
            if clk_cnt = SPI_2X_CLK_DIV-1 then
                spi_2x_ce <= '1';
                clk_cnt := 0;
            else
                spi_2x_ce <= '0';
                clk_cnt := clk_cnt + 1;
            end if;
        end if;
    end process spi_2x_ce_gen_proc;
    -----------------------------------------------------------------------------------------------
    -- generate the core antiphase clocks and clock enables from the 2x base CE.
    core_clock_gen_proc : process (sclk_i) is
    begin
        if sclk_i'event and sclk_i = '1' then
            if spi_2x_ce = '1' then
                -- generate the 2 antiphase core clocks
                core_clk <= core_n_clk;
                core_n_clk <= not core_n_clk;
                -- generate the 2 phase core clock enables
                core_ce <= core_n_clk;
                core_n_ce <= not core_n_clk;
            else
                core_ce <= '0';
                core_n_ce <= '0';
            end if;
        end if;
    end process core_clock_gen_proc;

    --=============================================================================================
    --  GENERATE BLOCKS
    --=============================================================================================
    -- spi clk generator: generate spi_clk from core_clk depending on CPOL
    spi_sck_cpol_0_proc: if CPOL = '0' generate
    begin
        spi_clk <= core_clk;            -- for CPOL=0, spi clk has idle LOW
    end generate;
    
    spi_sck_cpol_1_proc: if CPOL = '1' generate
    begin
        spi_clk <= core_n_clk;          -- for CPOL=1, spi clk has idle HIGH
    end generate;
    -----------------------------------------------------------------------------------------------
    -- Sampling clock enable generation: generate 'samp_ce' from 'core_ce' or 'core_n_ce' depending on CPHA
    -- always sample data at the half-cycle of the fsm update cell
    samp_ce_cpha_0_proc: if CPHA = '0' generate
    begin
        samp_ce <= core_ce;
    end generate;
        
    samp_ce_cpha_1_proc: if CPHA = '1' generate
    begin
        samp_ce <= core_n_ce;
    end generate;
    -----------------------------------------------------------------------------------------------
    -- FSM clock enable generation: generate 'fsm_ce' from core_ce or core_n_ce depending on CPHA
    fsm_ce_cpha_0_proc: if CPHA = '0' generate
    begin
        fsm_ce <= core_n_ce;            -- for CPHA=0, latch registers at rising edge of negative core clock enable
    end generate;
    
    fsm_ce_cpha_1_proc: if CPHA = '1' generate
    begin
        fsm_ce <= core_ce;              -- for CPHA=1, latch registers at rising edge of positive core clock enable
    end generate;
    -----------------------------------------------------------------------------------------------
    -- sck enable control: control sck advance phase for CPHA='1' relative to fsm clock
    sck_ena_ce <= core_n_ce;            -- for CPHA=1, SCK is advanced one-half cycle
    
    --=============================================================================================
    --  REGISTERED INPUTS
    --=============================================================================================
    -- rx bit flop: capture rx bit after SAMPLE edge of sck
    rx_bit_proc : process (sclk_i, spi_miso_i) is
    begin
        if sclk_i'event and sclk_i = '1' then
            if samp_ce = '1' then
                rx_bit_reg <= spi_miso_i;
            end if;
        end if;
    end process rx_bit_proc;

    --=============================================================================================
    --  CROSS-CLOCK PIPELINE TRANSFER LOGIC
    --=============================================================================================
    -- do_valid_o and di_req_o strobe output logic
    -- this is a delayed pulse generator with a ripple-transfer FFD pipeline, that generates a 
    -- fixed-length delayed pulse for the output flags, at the parallel clock domain
    out_transfer_proc : process ( pclk_i, do_transfer_reg, di_req_reg, 
                                  do_valid_A, do_valid_B, do_valid_D, 
                                  di_req_o_A, di_req_o_B, di_req_o_D ) is
    begin
        if pclk_i'event and pclk_i = '1' then               -- clock at parallel port clock
            -- do_transfer_reg -> do_valid_o_reg
            do_valid_A <= do_transfer_reg;                  -- the input signal must be at least 2 clocks long
            do_valid_B <= do_valid_A;                       -- feed it to a ripple chain of FFDs
            do_valid_C <= do_valid_B;
            do_valid_D <= do_valid_C;
            do_valid_o_reg <= do_valid_next;                -- registered output pulse
            --------------------------------
            -- di_req_reg -> di_req_o_reg
            di_req_o_A <= di_req_reg;                       -- the input signal must be at least 2 clocks long
            di_req_o_B <= di_req_o_A;                       -- feed it to a ripple chain of FFDs
            di_req_o_C <= di_req_o_B;                           
            di_req_o_D <= di_req_o_C;                           
            di_req_o_reg <= di_req_o_next;                  -- registered output pulse
        end if;
        -- generate a 2-clocks pulse at the 3rd clock cycle
        do_valid_next <= do_valid_A and do_valid_B and not do_valid_D;
        di_req_o_next <= di_req_o_A and di_req_o_B and not di_req_o_D;
    end process out_transfer_proc;
    -- parallel load input registers: data register and write enable
    in_transfer_proc: process ( pclk_i, wren_i, wr_ack_reg ) is
    begin
        -- registered data input, input register with clock enable
        if pclk_i'event and pclk_i = '1' then
            if wren_i = '1' then
                di_reg <= di_i;                             -- parallel data input buffer register
            end if;
        end  if;
        -- stretch wren pulse to be detected by spi fsm (ffd with sync preset and sync reset)
        if pclk_i'event and pclk_i = '1' then
            if wren_i = '1' then                            -- wren_i is the sync preset for wren
                wren <= '1';
            elsif wr_ack_reg = '1' then                     -- wr_ack is the sync reset for wren
                wren <= '0';
            end if;
        end  if;
    end process in_transfer_proc;

    --=============================================================================================
    --  REGISTER TRANSFER PROCESSES
    --=============================================================================================
    -- fsm state and data registers: synchronous to the spi base reference clock
    core_reg_proc : process (sclk_i) is
    begin
        -- FF registers clocked on rising edge and cleared on sync rst_i
        if sclk_i'event and sclk_i = '1' then
            if rst_i = '1' then                             -- sync reset
                state_reg <= 0;                             -- only provide local reset for the state machine
            elsif fsm_ce = '1' then                         -- fsm_ce is clock enable for the fsm
                state_reg <= state_next;                    -- state register
            end if;
        end if;
        -- FF registers clocked synchronous to the fsm state
        if sclk_i'event and sclk_i = '1' then
            if fsm_ce = '1' then
                sh_reg <= sh_next;                          -- shift register
                ssel_ena_reg <= ssel_ena_next;              -- spi select enable
                do_buffer_reg <= do_buffer_next;            -- registered output data buffer 
                do_transfer_reg <= do_transfer_next;        -- output data transferred to buffer
                di_req_reg <= di_req_next;                  -- input data request
                wr_ack_reg <= wr_ack_next;                  -- write acknowledge for data load synchronization
            end if;
        end if;
        -- FF registers clocked one-half cycle earlier than the fsm state
        if sclk_i'event and sclk_i = '1' then
            if sck_ena_ce = '1' then
                sck_ena_reg <= sck_ena_next;                -- spi clock enable: look ahead logic
            end if;
        end if;
    end process core_reg_proc;

    --=============================================================================================
    --  COMBINATORIAL LOGIC PROCESSES
    --=============================================================================================
    -- state and datapath combinatorial logic
    core_combi_proc : process ( sh_reg, state_reg, rx_bit_reg, ssel_ena_reg, sck_ena_reg, do_buffer_reg, 
                                do_transfer_reg, wr_ack_reg, di_req_reg, di_reg, wren ) is
    begin
        sh_next <= sh_reg;                                              -- all output signals are assigned to (avoid latches)
        ssel_ena_next <= ssel_ena_reg;                                  -- controls the slave select line
        sck_ena_next <= sck_ena_reg;                                    -- controls the clock enable of spi sck line
        do_buffer_next <= do_buffer_reg;                                -- output data buffer
        do_transfer_next <= do_transfer_reg;                            -- output data flag
        wr_ack_next <= wr_ack_reg;                                      -- write acknowledge
        di_req_next <= di_req_reg;                                      -- prefetch data request
        spi_mosi_o <= sh_reg(N-1);                                      -- default to avoid latch inference
        state_next <= state_reg;                                        -- next state 
        case state_reg is
        
            when (N+1) =>                                               -- this state is to enable SSEL before SCK
                spi_mosi_o <= sh_reg(N-1);                              -- shift out tx bit from the MSb
                ssel_ena_next <= '1';                                   -- tx in progress: will assert SSEL
                sck_ena_next <= '1';                                    -- enable SCK on next cycle (stays off on first SSEL clock cycle)
                di_req_next <= '0';                                     -- prefetch data request: deassert when shifting data
                wr_ack_next <= '0';                                     -- remove write acknowledge for all but the load stages
                state_next <= state_reg - 1;                            -- update next state at each sck pulse
                
            when (N) =>                                                 -- deassert 'di_rdy' and stretch do_valid
                spi_mosi_o <= sh_reg(N-1);                              -- shift out tx bit from the MSb
                di_req_next <= '0';                                     -- prefetch data request: deassert when shifting data
                sh_next(N-1 downto 1) <= sh_reg(N-2 downto 0);          -- shift inner bits
                sh_next(0) <= rx_bit_reg;                               -- shift in rx bit into LSb
                wr_ack_next <= '0';                                     -- remove write acknowledge for all but the load stages
                state_next <= state_reg - 1;                            -- update next state at each sck pulse
                
            when (N-1) downto (PREFETCH+3) =>                           -- remove 'do_transfer' and shift bits
                spi_mosi_o <= sh_reg(N-1);                              -- shift out tx bit from the MSb
                di_req_next <= '0';                                     -- prefetch data request: deassert when shifting data
                do_transfer_next <= '0';                                -- reset 'do_valid' transfer signal
                sh_next(N-1 downto 1) <= sh_reg(N-2 downto 0);          -- shift inner bits
                sh_next(0) <= rx_bit_reg;                               -- shift in rx bit into LSb
                wr_ack_next <= '0';                                     -- remove write acknowledge for all but the load stages
                state_next <= state_reg - 1;                            -- update next state at each sck pulse
                
            when (PREFETCH+2) downto 2 =>                               -- raise prefetch 'di_req_o' signal
                spi_mosi_o <= sh_reg(N-1);                              -- shift out tx bit from the MSb
                di_req_next <= '1';                                     -- request data in advance to allow for pipeline delays
                sh_next(N-1 downto 1) <= sh_reg(N-2 downto 0);          -- shift inner bits
                sh_next(0) <= rx_bit_reg;                               -- shift in rx bit into LSb
                wr_ack_next <= '0';                                     -- remove write acknowledge for all but the load stages
                state_next <= state_reg - 1;                            -- update next state at each sck pulse
                
            when 1 =>                                                   -- transfer rx data to do_buffer and restart if new data is written
                spi_mosi_o <= sh_reg(N-1);                              -- shift out tx bit from the MSb
                di_req_next <= '1';                                     -- request data in advance to allow for pipeline delays
                do_buffer_next(N-1 downto 1) <= sh_reg(N-2 downto 0);   -- shift rx data directly into rx buffer
                do_buffer_next(0) <= rx_bit_reg;                        -- shift last rx bit into rx buffer
                do_transfer_next <= '1';                                -- signal transfer to do_buffer
                if wren = '1' then                                      -- load tx register if valid data present at di_i
                    state_next <= N;                                  	-- next state is top bit of new data
                    sh_next <= di_reg;                                  -- load parallel data from di_reg into shifter
                    sck_ena_next <= '1';                                -- SCK enabled
                    wr_ack_next <= '1';                                 -- acknowledge data in transfer
                else
                    sck_ena_next <= '0';                                -- SCK disabled: tx empty, no data to send
                    wr_ack_next <= '0';                                 -- remove write acknowledge for all but the load stages
                    state_next <= state_reg - 1;                        -- update next state at each sck pulse
                end if;
                
            when 0 =>                                                   -- idle state: start and end of transmission
                di_req_next <= '1';                                     -- will request data if shifter empty
                sck_ena_next <= '0';                                    -- SCK disabled: tx empty, no data to send
                if wren = '1' then                                      -- load tx register if valid data present at di_i
                    spi_mosi_o <= di_reg(N-1);                          -- special case: shift out first tx bit from the MSb (look ahead)
                    ssel_ena_next <= '1';                               -- enable interface SSEL
                    state_next <= N+1;                                  -- start from idle: let one cycle for SSEL settling
                    sh_next <= di_reg;                                  -- load bits from di_reg into shifter
                    wr_ack_next <= '1';                                 -- acknowledge data in transfer
                else
                    spi_mosi_o <= sh_reg(N-1);                          -- shift out tx bit from the MSb
                    ssel_ena_next <= '0';                               -- deassert SSEL: interface is idle
                    wr_ack_next <= '0';                                 -- remove write acknowledge for all but the load stages
                    state_next <= 0;                                    -- when idle, keep this state
                end if;
                
            when others =>
                state_next <= 0;                                        -- state 0 is safe state
        end case; 
    end process core_combi_proc;

    --=============================================================================================
    --  OUTPUT LOGIC PROCESSES
    --=============================================================================================
    -- data output processes
    spi_ssel_o_proc:    spi_ssel_o <= not ssel_ena_reg;                 -- active-low slave select line 
    do_o_proc:          do_o <= do_buffer_reg;                          -- parallel data out
    do_valid_o_proc:    do_valid_o <= do_valid_o_reg;                   -- data out valid
    di_req_o_proc:      di_req_o <= di_req_o_reg;                       -- input data request for next cycle
    wr_ack_o_proc:      wr_ack_o <= wr_ack_reg;                         -- write acknowledge
    -----------------------------------------------------------------------------------------------
    -- SCK out logic: pipeline phase compensation for the SCK line
    -----------------------------------------------------------------------------------------------
    -- This is a MUX with an output register. 
    -- The register gives us a pipeline delay for the SCK line, pairing with the state machine moore 
    -- output pipeline delay for the MOSI line, and thus enabling higher SCK frequency. 
    spi_sck_o_gen_proc : process (sclk_i, sck_ena_reg, spi_clk, spi_clk_reg) is
    begin
        if sclk_i'event and sclk_i = '1' then
            if sck_ena_reg = '1' then
                spi_clk_reg <= spi_clk;                                 -- copy the selected clock polarity
            else
                spi_clk_reg <= CPOL;                                    -- when clock disabled, set to idle polarity
            end if;
        end if;
        spi_sck_o <= spi_clk_reg;                                       -- connect register to output
    end process spi_sck_o_gen_proc;

    --=============================================================================================
    --  DEBUG LOGIC PROCESSES
    --=============================================================================================
    -- these signals are useful for verification, and can be deleted after debug.
    do_transfer_proc:   do_transfer_o <= do_transfer_reg;
    state_dbg_proc:     state_dbg_o <= std_logic_vector(to_unsigned(state_reg, 4));
    rx_bit_reg_proc:    rx_bit_reg_o <= rx_bit_reg;
    wren_o_proc:        wren_o <= wren;
    sh_reg_dbg_proc:    sh_reg_dbg_o <= sh_reg;
    core_clk_o_proc:    core_clk_o <= core_clk;
    core_n_clk_o_proc:  core_n_clk_o <= core_n_clk;
    core_ce_o_proc:     core_ce_o <= core_ce;
    core_n_ce_o_proc:   core_n_ce_o <= core_n_ce;
    sck_ena_o_proc:     sck_ena_o <= sck_ena_reg;
    sck_ena_ce_o_proc:  sck_ena_ce_o <= sck_ena_ce;

end architecture rtl;
