// =============================================================================
// GLOBAL SYSTEM DEFINITIONS
// =============================================================================
`define     _D_CLK_                 100000000   // System Clock: 100 MHz
`define     _D_DIV_1kHz_            ((`_D_CLK_/1000) - 1)
`define     _D_DATA_WIDTH_          16          // Standard Data Bus Width

// =============================================================================
// SPI BUS CONFIGURATION (Control Plane - Polling)
// =============================================================================
`define     _D_S_ADDR_WIDTH_        10          
`define     _D_S_DEV_ADDR_WIDTH_    4           
`define     _D_S_CHIP_ADDR_WIDTH_   6           

`define     _D_S_DEV_HI_            9
`define     _D_S_DEV_LO_            6
`define     _D_S_CHIP_HI_           5
`define     _D_S_CHIP_LO_           0

// Number of active SPI modules & Device IDs
`define     _D_S_NUM_OF_DEV_        6           // Total devices on Polling bus
`define     _D_S_DEBUG_ID_          1           // ID 1: SPI Debug Module
`define     _D_S_INT_CTRL_ID_       2           // ID 2: SPI Interrupt Controller
`define     _D_S_DAC_ID_            3           // ID 3: DAC Controller (10x AD5543)
`define     _D_S_RESOLVER_ID_       4           // ID 4: Resolver Controller (DA36)
`define     _D_S_COMP_ID_           5           // ID 5: Comparator & Watchdog Block
`define     _D_S_ADC_BLOCK_ID_      6           // ID 6: ADC Subsystem (DA21 4AC, DA22 4AC, DA45 4DC)

// =============================================================================
// DMA & INTERRUPT CONFIGURATION (Data Plane)
// =============================================================================
// 9 (Resolver) + 22 (ADC Subsystem: 9 DA21 + 9 DA22 + 4 DA45) + 2 (Comp Watchdog) = 33 words (528 bits)
`define     _D_DMA_WORDS_           33          // 33 words for Read DMA Burst
`define     _D_DMA_DAC_WORDS_       10          // 10 words for Write DMA Burst (DACs)     
`define     _D_IRQ_LINES_           2

// =============================================================================
// SIGNAL PROCESSING CONFIGURATION
// =============================================================================
`define     _D_RES_SAMPLE_TICKS_    1000        // 100 kHz Sample Rate (1000 ticks of 100 MHz)
`define     _D_COMP_CHATTER_TICKS_  100         // 1.0 us verification window
`define     _D_COMP_BLANKING_TICKS_ 3000        // 30.0 us blanking window