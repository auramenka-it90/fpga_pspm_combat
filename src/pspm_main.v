`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: pspm_main (Top-Level)
// 
// DESCRIPTION:
//  High-Performance Mixed-Signal Processing System for Motor/Motion Control.
//  Implements a dual-plane architecture (Control Plane + Data Plane) over a 
//  shared physical SPI bus using two independent Chip Selects (NSS_P, NSS_D).
//  Includes UART/RS-485 hardware multiplexer for DD4 and DD5 transceivers.
//
//  Subsystems:
//   - DA36: Resolver ADC (Synchronous lock-in demodulation).
//   - DA21: Elevation ADC (4x AC Channels synchronized to UON23).
//   - DA22: Azimuth ADC   (4x AC Channels synchronized to UON12).
//   - DA45: Telemetry ADC (4x DC Channels with dynamic decimation).
//   - 10x AD5543 multiplying DACs with atomic DMA latch.
//   - Dual Comparator Watchdog (UON12 / UON23).
//   - MultiBoot IPROG Reconfiguration (Image 2 at 1 MB Flash offset).
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module pspm_main (
    // --- System Clock Input ---
    input  wire                         clk_60mHz,       

    // --- External Flash Lockout Line (Hardware protection for shared SPI) ---
    output wire                         w25q128_nss,

    // =========================================================================
    // SHARED SPI BUS (STM32 <-> FPGA)
    // =========================================================================
    input  wire                         spi_stm32_sck,   
    input  wire                         spi_stm32_mosi,  
    output wire                         spi_stm32_miso,  
    
    // --- Chip Selects ---
    input  wire                         spi_stm32_nss_p, // CS for Polling / Config (PB0)
    input  wire                         spi_stm32_nss_d, // CS for DMA / Burst (PB2)

    // --- Global Hardware Interrupt ---
    output wire                         fpga_2_stm32_interrupt_N, 

    // =========================================================================
    // UART / RS-485 MULTIPLEXER INTERFACE
    // =========================================================================
    input  wire                         usart2_fpga_tx,  // TX data from STM32 PA2
    output wire                         usart2_fpga_rx,  // RX data to STM32 PA3

    // --- RS-485 / RS-422 Channel 1 (Transceiver DD4) ---
    output wire                         tx2,             // TX output to DD4 (DI)
    input  wire                         rx2,             // RX input from DD4 (RO)
    output wire                         de2,             // Transmitter Driver Enable for DD4 (DE/RE_N)

    // --- RS-485 / RS-422 Channel 2 (Transceiver DD5) ---
    output wire                         tx3,             // TX output to DD5 (DI)
    input  wire                         rx3,             // RX input from DD5 (RO)
    output wire                         de3,             // Transmitter Driver Enable for DD5 (DE/RE_N)
    
    // --- External Test Points & LEDs ---
    output wire [8:5]                   tp,              
    output wire [2:0]                   led,             
    
    // =========================================================================
    // PHYSICAL PINS: COMPARATORS, ADCs, DACs
    // =========================================================================
    // Comparators (3-Phase Zero-Crossing)
    input  wire                         UON12,           
    input  wire                         UON23,           
    
    // DA36: Resolver ADC (AD7367)
    output wire                         res_cnvst_n, 
    output wire                         res_cs_n, 
    output wire                         res_sclk, 
    output wire                         res_addr, 
    input  wire                         res_busy, 
    input  wire                         res_dout_a, 
    input  wire                         res_dout_b,

    // DA21: Elevation ADC (AD7367 - 4x AC)
    output wire                         da21_cnvst_n, 
    output wire                         da21_cs_n, 
    output wire                         da21_sclk, 
    output wire                         da21_addr, 
    input  wire                         da21_busy, 
    input  wire                         da21_dout_a, 
    input  wire                         da21_dout_b,

    // DA22: Azimuth ADC (AD7367 - 4x AC)
    output wire                         da22_cnvst_n, 
    output wire                         da22_cs_n, 
    output wire                         da22_sclk, 
    output wire                         da22_addr, 
    input  wire                         da22_busy, 
    input  wire                         da22_dout_a, 
    input  wire                         da22_dout_b,

    // DA45: Telemetry ADC (AD7367 - 4x DC)
    output wire                         da45_cnvst_n, 
    output wire                         da45_cs_n, 
    output wire                         da45_sclk, 
    output wire                         da45_addr, 
    input  wire                         da45_busy, 
    input  wire                         da45_dout_a, 
    input  wire                         da45_dout_b,

    // DACs (10x AD5543)
    output wire [10:1]                  dac_cs_n, 
    output wire                         dac_clk,  
    output wire                         dac_sdi
);

    // =========================================================================
    // HARDWARE FLASH LOCKOUT
    // Keep external SPI Flash in disabled/standby mode to prevent MISO contention.
    // =========================================================================
    assign w25q128_nss = 1'b1;

    // =========================================================================
    // SYSTEM CLOCK & RESET
    // =========================================================================
    wire clk; 
    wire rst; 
    wire tick_1khz;

    system_clk_rst u_sys_clk_rst (
        .ext_clk_60  (clk_60mHz),
        .clk_100     (clk),
        .rst_sync    (rst),
        .tick_1khz   (tick_1khz)
    );

    // =========================================================================
    // INTERNAL 400 Hz 3-PHASE GENERATOR (BIST / Self-Test)
    // =========================================================================
    wire gen_uon12;
    wire gen_uon23;

    bist_generator #(
        .SYS_CLK_FREQ (`_D_CLK_),        // 100 MHz
        .TARGET_FREQ  (400)              // 400 Hz
    ) u_bist_gen (
        .clk          (clk),
        .rst          (rst),
        .gen_uon12    (gen_uon12),
        .gen_uon23    (gen_uon23)
    );

    // =========================================================================
    // CONTROL PLANE: SPI POLLING BRIDGE (Uses NSS_P)
    // =========================================================================
    wire [`_D_S_ADDR_WIDTH_-1:0] cfg_addr_raw;
    wire [`_D_DATA_WIDTH_-1:0]   cfg_data_to_fpga;
    wire [`_D_DATA_WIDTH_-1:0]   cfg_data_from_fpga;
    wire                         cfg_wr_strobe;
    wire                         cfg_rd_active;
    wire                         cfg_miso_internal;

    spi_stm32_fpga_bridge u_spi_config (
        .clk       (clk),          
        .rst       (rst),          
        .spi_in    (spi_stm32_mosi),  
        .sck_in    (spi_stm32_sck),   
        .nss_in    (spi_stm32_nss_p), 
        .spi_out   (cfg_miso_internal), 
        .addr      (cfg_addr_raw),
        .dout      (cfg_data_to_fpga), 
        .din       (cfg_data_from_fpga),
        .wr_strobe (cfg_wr_strobe),
        .rd        (cfg_rd_active),      
        .busy      () 
    );

    // --- Address and Device Decoding ---
    wire [`_D_S_DEV_ADDR_WIDTH_-1:0]  addr_dev_s  = cfg_addr_raw[`_D_S_DEV_HI_:`_D_S_DEV_LO_];
    wire [`_D_S_CHIP_ADDR_WIDTH_-1:0] addr_chip_s = cfg_addr_raw[`_D_S_CHIP_HI_:`_D_S_CHIP_LO_];

    // Array of strobes and read buses per peripheral device
    wire [`_D_S_NUM_OF_DEV_:1] wr_dev_s;
    wire [`_D_S_NUM_OF_DEV_:1] rd_dev_s;
    wire [`_D_DATA_WIDTH_-1:0] data_rd_dev_s [1:`_D_S_NUM_OF_DEV_];

    genvar j;
    generate
        for (j = 1; j <= `_D_S_NUM_OF_DEV_; j = j + 1) begin: SPI_GEN
            assign rd_dev_s[j] = (addr_dev_s == j) ? cfg_rd_active : 1'b0;
            assign wr_dev_s[j] = (addr_dev_s == j) ? cfg_wr_strobe : 1'b0;
        end
    endgenerate

    // Multi-Device SPI Read multiplexer
    assign cfg_data_from_fpga = 
        (addr_dev_s == `_D_S_DEBUG_ID_)     ? data_rd_dev_s[`_D_S_DEBUG_ID_]     :
        (addr_dev_s == `_D_S_INT_CTRL_ID_)  ? data_rd_dev_s[`_D_S_INT_CTRL_ID_]  : 
        (addr_dev_s == `_D_S_DAC_ID_)       ? data_rd_dev_s[`_D_S_DAC_ID_]       : 
        (addr_dev_s == `_D_S_RESOLVER_ID_)  ? data_rd_dev_s[`_D_S_RESOLVER_ID_]  : 
        (addr_dev_s == `_D_S_COMP_ID_)      ? data_rd_dev_s[`_D_S_COMP_ID_]      : 
        (addr_dev_s == `_D_S_ADC_BLOCK_ID_) ? data_rd_dev_s[`_D_S_ADC_BLOCK_ID_] : 
        {`_D_DATA_WIDTH_{1'b0}};

    // =========================================================================
    // DATA PLANE: SPI DMA BURST BRIDGE (Uses NSS_D)
    // =========================================================================
    wire [(16 * `_D_DMA_DAC_WORDS_)-1:0] dma_data_from_stm32; // 160 bits (10 DACs)
    wire [(16 * `_D_DMA_WORDS_)-1:0]     dma_data_to_stm32;   // 528 bits (33 words)
    
    wire dma_wr_strobe; 
    wire dma_rd_strobe; 
    wire dma_miso_internal;

    spi_stm32_multi_word_bridge #(
        .READ_WORDS (`_D_DMA_WORDS_),     // 33 words (528 bits)
        .WRITE_WORDS(`_D_DMA_DAC_WORDS_)  // 10 words (160 bits)
    ) u_spi_dma (
        .clk       (clk),       
        .rst       (rst),       
        .spi_in    (spi_stm32_mosi),  
        .sck_in    (spi_stm32_sck),   
        .nss_in    (spi_stm32_nss_d), 
        .spi_out   (dma_miso_internal),   
        .dout      (dma_data_from_stm32),      
        .din       (dma_data_to_stm32),       
        .wr_strobe (dma_wr_strobe), 
        .rd_strobe (dma_rd_strobe), 
        .busy      ()       
    );

    // =========================================================================
    // MISO ARBITRATION (Top-Level Tri-State)
    // =========================================================================
    assign spi_stm32_miso = (!spi_stm32_nss_p) ? cfg_miso_internal :
                            (!spi_stm32_nss_d) ? dma_miso_internal : 
                            1'bZ;

    // =========================================================================
    // CONTROL PLANE: DEBUG MODULE (Device ID = 1)
    // =========================================================================
    wire [`_D_DATA_WIDTH_-1:0] debug_misc_out; 
    wire [14:0]                debug_tp_mux_out;
    wire                       reconfig_trigger;

    debug_module #(
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_),
        .DATA_WIDTH(`_D_DATA_WIDTH_)
    ) u_debug_module (
        .clk              (clk),
        .rst              (rst),
        .cpu_addr         (addr_chip_s),
        .cpu_di           (cfg_data_to_fpga),
        .cpu_wr           (wr_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_rd           (rd_dev_s[`_D_S_DEBUG_ID_]),
        .cpu_do           (data_rd_dev_s[`_D_S_DEBUG_ID_]),
        .misc_out         (debug_misc_out),
        .tp_mux_out       (debug_tp_mux_out),
        .reconfig_trigger (reconfig_trigger)
    );

    // =========================================================================
    // MULTIBOOT / IPROG RECONFIGURATION CONTROLLER
    // =========================================================================
    multiboot_icap #(
        .IMAGE2_ADDR (24'h100000) // 1 MB Flash Offset (0x0010_0000)
    ) u_multiboot (
        .clk     (clk),
        .rst     (rst),
        .trigger (reconfig_trigger)
    );
	
    assign led = debug_misc_out[2:0];

    // =========================================================================
    // UART / RS-485 HARDWARE MULTIPLEXER (Controlled by debug_misc_out[3])
    // =========================================================================
    wire uart_mux_sel = debug_misc_out[3];

    assign tx2 = (uart_mux_sel == 1'b0) ? usart2_fpga_tx : 1'b1;
    assign tx3 = (uart_mux_sel == 1'b1) ? usart2_fpga_tx : 1'b1;

    assign usart2_fpga_rx = (uart_mux_sel == 1'b0) ? rx2 : rx3;

    assign de2 = (uart_mux_sel == 1'b0) ? 1'b1 : 1'b0;
    assign de3 = (uart_mux_sel == 1'b1) ? 1'b1 : 1'b0;

    // =========================================================================
    // COMPARATOR & WATCHDOG BLOCK (Device ID = 5)
    // =========================================================================
    wire uon12_selected = debug_misc_out[5] ? gen_uon12 : UON12;
    wire uon23_selected = debug_misc_out[5] ? gen_uon23 : UON23;

    wire comp_uon12_clean;
    wire comp_uon23_clean;
    wire [31:0] comp_dma_payload; // 2 words
    wire irq_freq_err;

    comparator_block #(
        .ADR_WIDTH    (`_D_S_CHIP_ADDR_WIDTH_),
        .DATA_WIDTH   (`_D_DATA_WIDTH_)
    ) u_comp_block (
        .clk          (clk),
        .rst          (rst),
        .cpu_addr     (addr_chip_s),
        .cpu_di       (cfg_data_to_fpga),
        .cpu_wr       (wr_dev_s[`_D_S_COMP_ID_]),
        .cpu_rd       (rd_dev_s[`_D_S_COMP_ID_]),
        .cpu_do       (data_rd_dev_s[`_D_S_COMP_ID_]),
        .comp_dma_out (comp_dma_payload),
        .uon12_in     (uon12_selected),
        .uon23_in     (uon23_selected),
        .uon12_clean  (comp_uon12_clean),
        .uon23_clean  (comp_uon23_clean),
        .irq_freq_err (irq_freq_err)
    );

    // Hardware Multiplexer for Resolver Reference Phase: Bit [4] (0=UON12, 1=UON23)
    wire res_comp_mux = debug_misc_out[4] ? comp_uon23_clean : comp_uon12_clean;

    // =========================================================================
    // CONTROL PLANE: INTERRUPT CONTROLLER (Device ID = 2)
    // =========================================================================
    wire irq_adc12_ready;
    
    wire [1:0] hardware_irq_bus = {
        irq_freq_err,               // IRQ [1]: Frequency Watchdog Error
        irq_adc12_ready             // IRQ [0]: ADC Block Data Ready (UON23)
    };

    interrupt_controller #( 
        .ADR_WIDTH (`_D_S_CHIP_ADDR_WIDTH_),
        .IRQ_LINES (2)              // Exactly 2 active lines!
    ) u_ic_inst (
        .clk        (clk), 
        .rst        (rst), 
        .cpu_addr   (addr_chip_s),
        .cpu_di     (cfg_data_to_fpga),
        .cpu_wr     (wr_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_rd     (rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .cpu_do     (data_rd_dev_s[`_D_S_INT_CTRL_ID_]),
        .irq_inputs (hardware_irq_bus), 
        .irq_out_N  (fpga_2_stm32_interrupt_N) 
    );

    // =========================================================================
    // DATA PLANE: ADC SUBSYSTEM (DA21 4AC, DA22 4AC, DA45 4DC) (Device ID = 6)
    // =========================================================================
    wire [351:0] adc_dma_payload; // 22 words

    adc_block #(
        .ADR_WIDTH    (`_D_S_CHIP_ADDR_WIDTH_),
        .DATA_WIDTH   (`_D_DATA_WIDTH_),
        .SAMPLE_TICKS (`_D_RES_SAMPLE_TICKS_)
    ) u_adc_subsystem (
        .clk             (clk),
        .rst             (rst),
        
        .cpu_addr        (addr_chip_s),
        .cpu_di          (cfg_data_to_fpga),
        .cpu_wr          (wr_dev_s[`_D_S_ADC_BLOCK_ID_]),
        .cpu_rd          (rd_dev_s[`_D_S_ADC_BLOCK_ID_]),
        .cpu_do          (data_rd_dev_s[`_D_S_ADC_BLOCK_ID_]),
        
        .adc_data_out    (adc_dma_payload),
        .irq_adc12_ready (irq_adc12_ready),
        .dma_ack         (dma_rd_strobe),
        
        .comp_uon12      (comp_uon12_clean), // Synced to Phase 1-2
        .comp_uon23      (comp_uon23_clean), // Synced to Phase 2-3
        
        .da21_cnvst_n    (da21_cnvst_n), 
        .da21_cs_n       (da21_cs_n), 
        .da21_sclk       (da21_sclk), 
        .da21_addr       (da21_addr), 
        .da21_range0     (),
        .da21_range1     (),
        .da21_busy       (da21_busy), 
        .da21_dout_a     (da21_dout_a), 
        .da21_dout_b     (da21_dout_b),	
        
        .da22_cnvst_n    (da22_cnvst_n), 
        .da22_cs_n       (da22_cs_n), 
        .da22_sclk       (da22_sclk), 
        .da22_addr       (da22_addr), 
        .da22_range0     (),
        .da22_range1     (),
        .da22_busy       (da22_busy), 
        .da22_dout_a     (da22_dout_a), 
        .da22_dout_b     (da22_dout_b),	
        
        .da45_cnvst_n    (da45_cnvst_n), 
        .da45_cs_n       (da45_cs_n), 
        .da45_sclk       (da45_sclk), 
        .da45_addr       (da45_addr), 
        .da45_range0     (),
        .da45_range1     (),
        .da45_busy       (da45_busy), 
        .da45_dout_a     (da45_dout_a), 
        .da45_dout_b     (da45_dout_b)
    );

    // =========================================================================
    // DATA PLANE: RESOLVER SUBSYSTEM (DA36) (Device ID = 4)
    // =========================================================================
    wire [143:0] res_dma_payload; // 9 words
    
    resolver_block #(
        .ADR_WIDTH    (`_D_S_CHIP_ADDR_WIDTH_),
        .DATA_WIDTH   (`_D_DATA_WIDTH_),
        .SAMPLE_TICKS (`_D_RES_SAMPLE_TICKS_)
    ) u_resolver_block (
        .clk          (clk),
        .rst          (rst),
        
        .cpu_addr     (addr_chip_s),
        .cpu_di       (cfg_data_to_fpga),
        .cpu_wr       (wr_dev_s[`_D_S_RESOLVER_ID_]),
        .cpu_rd       (rd_dev_s[`_D_S_RESOLVER_ID_]),
        .cpu_do       (data_rd_dev_s[`_D_S_RESOLVER_ID_]),
        
        .res_data_out (res_dma_payload),
        .data_ready   (), 
        .dma_ack      (dma_rd_strobe),
        
        .comp_in      (res_comp_mux), 
        .cnvst_n      (res_cnvst_n), 
        .cs_n         (res_cs_n), 
        .sclk         (res_sclk), 
        .addr         (res_addr), 
        .busy         (res_busy), 
        .dout_a       (res_dout_a), 
        .dout_b       (res_dout_b),
        .range0       (),
        .range1       ()
    );

    // =========================================================================
    // DATA PLANE: DAC SUBSYSTEM (10x AD5543) (Device ID = 3)
    // =========================================================================
    dac_ad5543_block #(
        .ADR_WIDTH    (`_D_S_CHIP_ADDR_WIDTH_)
    ) u_dac_block (
        .clk          (clk),
        .rst          (rst),
        
        .cpu_addr     (addr_chip_s),
        .cpu_di       (cfg_data_to_fpga),
        .cpu_wr       (wr_dev_s[`_D_S_DAC_ID_]),
        .cpu_rd       (rd_dev_s[`_D_S_DAC_ID_]),
        .cpu_do       (data_rd_dev_s[`_D_S_DAC_ID_]),
        
        .hard_data_in (dma_data_from_stm32[159:0]), 
        .hard_trigger (dma_wr_strobe), 
        
        .dac_cs_n     (dac_cs_n),
        .dac_clk      (dac_clk),
        .dac_sdi      (dac_sdi)
    );

    // =========================================================================
    // DMA READ BUS PACKING (528 bits = 33 words)
    // Format: [527:496] Comp Payload (2 words)
    //         [495:144] ADC Payload  (22 words)
    //         [143:0]   Res Payload  (9 words)
    // =========================================================================
    assign dma_data_to_stm32 = {comp_dma_payload, adc_dma_payload, res_dma_payload};

    // =========================================================================
    // TEST POINTS MULTIPLEXER (Controlled by Debug Module)
    // =========================================================================
    wire [31:0] tp_signals;
    
    reg clk_monitor_div2;
    always @(posedge clk or posedge rst) begin
        if (rst)
            clk_monitor_div2 <= 1'b0;
        else
            clk_monitor_div2 <= ~clk_monitor_div2;
    end

    assign tp_signals[0]  = 1'b0;               // Default (GND)
    assign tp_signals[1]  = clk_monitor_div2;   // Divided Clock (50 MHz monitor)
    assign tp_signals[2]  = tick_1khz;
    assign tp_signals[3]  = !fpga_2_stm32_interrupt_N;
    
    // --- Comparators & Internal Generator ---
    assign tp_signals[4]  = UON12;
    assign tp_signals[5]  = comp_uon12_clean;
    assign tp_signals[6]  = UON23;
    assign tp_signals[7]  = comp_uon23_clean;
    assign tp_signals[8]  = gen_uon12;          // Internal 400Hz Gen Phase 1-2
    
    // --- SPI & DMA ---
    assign tp_signals[9]  = spi_stm32_nss_p;
    assign tp_signals[10] = spi_stm32_nss_d;
    assign tp_signals[11] = dma_wr_strobe;
    assign tp_signals[12] = dma_rd_strobe;
    
    // --- SPI Device Strobes ---
    assign tp_signals[13] = wr_dev_s[`_D_S_DEBUG_ID_];
    assign tp_signals[14] = rd_dev_s[`_D_S_DEBUG_ID_];
    assign tp_signals[15] = wr_dev_s[`_D_S_ADC_BLOCK_ID_];
    assign tp_signals[16] = rd_dev_s[`_D_S_ADC_BLOCK_ID_];
    assign tp_signals[17] = wr_dev_s[`_D_S_RESOLVER_ID_];
    
    // --- ADC Subsystem ---
    assign tp_signals[18] = da21_cnvst_n;
    assign tp_signals[19] = da22_cnvst_n;
    assign tp_signals[20] = da45_cnvst_n;
    assign tp_signals[21] = irq_adc12_ready;
    
    // --- Resolver ---
    assign tp_signals[22] = res_cnvst_n;
    assign tp_signals[23] = gen_uon23;          // Internal 400Hz Gen Phase 2-3
    
    // --- DAC ---
    assign tp_signals[24] = dac_cs_n[1];
    assign tp_signals[25] = dac_clk;
    
    // --- UART / RS-485 Diagnostics ---
    assign tp_signals[26] = usart2_fpga_tx;
    assign tp_signals[27] = usart2_fpga_rx;
    assign tp_signals[28] = de2;
    assign tp_signals[29] = de3;
    
    assign tp_signals[31:30] = 2'd0;

    // Dynamic Routing to Physical Pins
    assign tp[5] = tp_signals[ debug_tp_mux_out[4:0]   ];
    assign tp[6] = tp_signals[ debug_tp_mux_out[9:5]   ];
    assign tp[7] = tp_signals[ debug_tp_mux_out[14:10] ];
    assign tp[8] = tick_1khz;
	
endmodule