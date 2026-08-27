`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: adc_block (Device ID = 6)
// 
// DESCRIPTION:
//  Combined High-Precision ADC Subsystem for:
//   - DA21: Elevation Subsystem (4x AC Channels synchronized to UON23).
//   - DA22: Azimuth Subsystem   (4x AC Channels synchronized to UON12).
//   - DA45: Telemetry Subsystem (4x DC Channels with dynamic decimation).
//
//  Total DMA Payload: 22 words (352 bits)
//   - DA21: 9 words (1 sample count + 4x 32-bit AC lock-in integrals)
//   - DA22: 9 words (1 sample count + 4x 32-bit AC lock-in integrals)
//   - DA45: 4 words (4x 16-bit DC decimated channels)
//
//  SMART AUTO-CLEAR: 'dma_ack' input automatically clears valid and IRQ flags
//  when the STM32 finishes reading the DMA burst.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX9-TQG144)
//  Toolchain:      Aldec Active-HDL 9.2 / ISE 14.7 / XST
//  All comments in pure ASCII English.
// =============================================================================

module adc_block #(
    parameter ADR_WIDTH    = `_D_S_CHIP_ADDR_WIDTH_, // 6 bits
    parameter DATA_WIDTH   = `_D_DATA_WIDTH_,         // 16 bits
    parameter SAMPLE_TICKS = 1000                     // 100 kHz Sample Rate at 100 MHz clock
)(
    input  wire                 clk,         
    input  wire                 rst,         

    // --- Polling Interface (Control Plane) ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [DATA_WIDTH-1:0]cpu_di,
    input  wire                 cpu_wr,
    input  wire                 cpu_rd,
    output wire [DATA_WIDTH-1:0]cpu_do,

    // --- DMA Interface (Data Plane - 22 words = 352 bits) ---
    output wire [351:0]         adc_data_out,
    output wire                 irq_adc12_ready, // Master 400 Hz Data Ready IRQ
    input  wire                 dma_ack, 

    // --- Synchronization Reference Inputs ---
    input  wire                 comp_uon12,      // Phase 1-2 Zero-Crossing
    input  wire                 comp_uon23,      // Phase 2-3 Zero-Crossing

    // --- Physical ADC Pins: DA21 (Elevation 4x AC) ---
    output wire                 da21_cnvst_n, 
    output wire                 da21_cs_n, 
    output wire                 da21_sclk, 
    output wire                 da21_addr, 
    output wire                 da21_range0, 
    output wire                 da21_range1,
    input  wire                 da21_busy, 
    input  wire                 da21_dout_a, 
    input  wire                 da21_dout_b,

    // --- Physical ADC Pins: DA22 (Azimuth 4x AC) ---
    output wire                 da22_cnvst_n, 
    output wire                 da22_cs_n, 
    output wire                 da22_sclk, 
    output wire                 da22_addr, 
    output wire                 da22_range0, 
    output wire                 da22_range1,
    input  wire                 da22_busy, 
    input  wire                 da22_dout_a, 
    input  wire                 da22_dout_b,

    // --- Physical ADC Pins: DA45 (Telemetry 4x DC) ---
    output wire                 da45_cnvst_n, 
    output wire                 da45_cs_n, 
    output wire                 da45_sclk, 
    output wire                 da45_addr, 
    output wire                 da45_range0, 
    output wire                 da45_range1,
    input  wire                 da45_busy, 
    input  wire                 da45_dout_a, 
    input  wire                 da45_dout_b
);

    wire rst_n = ~rst;

    // =========================================================================
    // 1. CONTROL REGISTERS & SMART AUTO-CLEAR
    // =========================================================================
    reg       reg_irq_en_12;
    reg [4:0] shift_da45;

    reg       hw_valid_da21;
    reg       hw_valid_da22;
    reg       hw_valid_da45;

    wire wr_ctrl = (cpu_addr == 6'h00) && cpu_wr;
    wire wr_deci = (cpu_addr == 6'h18) && cpu_wr;
    
    // Clear flags via CPU Polling OR automatically on DMA burst completion
    wire soft_irq_ack = (wr_ctrl && cpu_di[DATA_WIDTH-1]) || dma_ack;

    // =========================================================================
    // 2. DA21: ELEVATION SUBSYSTEM (4x AC Channels Synced to UON23)
    // =========================================================================
    wire [31:0] c_da21_ac1, c_da21_ac2, c_da21_ac3, c_da21_ac4;
    wire [15:0] c_da21_cnt;
    wire        c_da21_rdy;

    adc_processor_4ac #( 
        .RANGE1_CFG   (1'b0),
        .RANGE0_CFG   (1'b0),
        .SAMPLE_TICKS (SAMPLE_TICKS) 
    ) u_da21 (
        .clk         (clk), 
        .rst_n       (rst_n), 
        .comp_in     (comp_uon23), // Synced to Phase 2-3
        .cnvst_n     (da21_cnvst_n), 
        .cs_n        (da21_cs_n), 
        .sclk        (da21_sclk), 
        .addr        (da21_addr), 
        .range0      (da21_range0), 
        .range1      (da21_range1), 
        .busy        (da21_busy), 
        .dout_a      (da21_dout_a), 
        .dout_b      (da21_dout_b),
        .acc_ch1     (c_da21_ac1), 
        .acc_ch2     (c_da21_ac2), 
        .acc_ch3     (c_da21_ac3), 
        .acc_ch4     (c_da21_ac4), 
        .cnt_samples (c_da21_cnt), 
        .ready_ac    (c_da21_rdy)
    );

    reg [31:0] sh_da21_ac1, sh_da21_ac2, sh_da21_ac3, sh_da21_ac4;
    reg [15:0] sh_da21_cnt;

    // =========================================================================
    // 3. DA22: AZIMUTH SUBSYSTEM (4x AC Channels Synced to UON12)
    // =========================================================================
    wire [31:0] c_da22_ac1, c_da22_ac2, c_da22_ac3, c_da22_ac4;
    wire [15:0] c_da22_cnt;
    wire        c_da22_rdy;

    adc_processor_4ac #( 
        .RANGE1_CFG   (1'b0),
        .RANGE0_CFG   (1'b0),
        .SAMPLE_TICKS (SAMPLE_TICKS) 
    ) u_da22 (
        .clk         (clk), 
        .rst_n       (rst_n), 
        .comp_in     (comp_uon12), // Synced to Phase 1-2
        .cnvst_n     (da22_cnvst_n), 
        .cs_n        (da22_cs_n), 
        .sclk        (da22_sclk), 
        .addr        (da22_addr), 
        .range0      (da22_range0), 
        .range1      (da22_range1), 
        .busy        (da22_busy), 
        .dout_a      (da22_dout_a), 
        .dout_b      (da22_dout_b),
        .acc_ch1     (c_da22_ac1), 
        .acc_ch2     (c_da22_ac2), 
        .acc_ch3     (c_da22_ac3), 
        .acc_ch4     (c_da22_ac4), 
        .cnt_samples (c_da22_cnt), 
        .ready_ac    (c_da22_rdy)
    );

    reg [31:0] sh_da22_ac1, sh_da22_ac2, sh_da22_ac3, sh_da22_ac4;
    reg [15:0] sh_da22_cnt;

    // =========================================================================
    // 4. DA45: TELEMETRY SUBSYSTEM (4x DC Channels)
    // =========================================================================
    wire [15:0] c_da45_dc1, c_da45_dc2, c_da45_dc3, c_da45_dc4;
    wire        c_da45_rdy;

    adc_processor_4dc #( 
        .RANGE1_CFG   (1'b0),
        .RANGE0_CFG   (1'b0),
        .SAMPLE_TICKS (SAMPLE_TICKS) 
    ) u_da45 (
        .clk         (clk), 
        .rst_n       (rst_n),
        .shift_sel   (shift_da45),
        .cnvst_n     (da45_cnvst_n), 
        .cs_n        (da45_cs_n), 
        .sclk        (da45_sclk), 
        .addr        (da45_addr), 
        .range0      (da45_range0), 
        .range1      (da45_range1), 
        .busy        (da45_busy), 
        .dout_a      (da45_dout_a), 
        .dout_b      (da45_dout_b),
        .acc_ch1     (c_da45_dc1), 
        .acc_ch2     (c_da45_dc2), 
        .acc_ch3     (c_da45_dc3), 
        .acc_ch4     (c_da45_dc4),
        .ready_dc    (c_da45_rdy)
    );

    reg [15:0] sh_da45_dc1, sh_da45_dc2, sh_da45_dc3, sh_da45_dc4;

    // =========================================================================
    // 5. SHADOW REGISTERS LATCHING LOGIC
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            reg_irq_en_12 <= 1'b0;
            shift_da45    <= 5'd7; // Default: 2^7 = 128 samples

            hw_valid_da21 <= 1'b0; 
            hw_valid_da22 <= 1'b0; 
            hw_valid_da45 <= 1'b0;
            
            sh_da21_cnt <= 16'd0; sh_da21_ac1 <= 32'd0; sh_da21_ac2 <= 32'd0; sh_da21_ac3 <= 32'd0; sh_da21_ac4 <= 32'd0;
            sh_da22_cnt <= 16'd0; sh_da22_ac1 <= 32'd0; sh_da22_ac2 <= 32'd0; sh_da22_ac3 <= 32'd0; sh_da22_ac4 <= 32'd0;
            sh_da45_dc1 <= 16'd0; sh_da45_dc2 <= 16'd0; sh_da45_dc3 <= 16'd0; sh_da45_dc4 <= 16'd0;
        end else begin
            if (wr_ctrl) begin
                reg_irq_en_12 <= cpu_di[14];
            end

            if (wr_deci) begin
                shift_da45 <= cpu_di[4:0];
            end

            if (soft_irq_ack) begin
                hw_valid_da21 <= 1'b0; 
                hw_valid_da22 <= 1'b0; 
                hw_valid_da45 <= 1'b0;
            end
            
            // Latch DA21 (Elevation 4AC)
            if (c_da21_rdy) begin
                sh_da21_cnt   <= c_da21_cnt;
                sh_da21_ac1   <= c_da21_ac1; 
                sh_da21_ac2   <= c_da21_ac2; 
                sh_da21_ac3   <= c_da21_ac3; 
                sh_da21_ac4   <= c_da21_ac4; 
                hw_valid_da21 <= 1'b1;
            end

            // Latch DA22 (Azimuth 4AC)
            if (c_da22_rdy) begin
                sh_da22_cnt   <= c_da22_cnt;
                sh_da22_ac1   <= c_da22_ac1; 
                sh_da22_ac2   <= c_da22_ac2; 
                sh_da22_ac3   <= c_da22_ac3; 
                sh_da22_ac4   <= c_da22_ac4; 
                hw_valid_da22 <= 1'b1;
            end

            // Latch DA45 (Telemetry 4DC)
            if (c_da45_rdy) begin
                sh_da45_dc1   <= c_da45_dc1; 
                sh_da45_dc2   <= c_da45_dc2;
                sh_da45_dc3   <= c_da45_dc3; 
                sh_da45_dc4   <= c_da45_dc4;
                hw_valid_da45 <= 1'b1;
            end
        end
    end

    // Master 400 Hz Data Ready IRQ driven by DA21 (UON23)
    assign irq_adc12_ready = hw_valid_da21 & reg_irq_en_12;

    // =========================================================================
    // 6. POLLING BUS READ DECODING
    // =========================================================================
    wire rd_ctrl        = (cpu_addr == 6'h00) && cpu_rd;
    wire rd_status      = (cpu_addr == 6'h01) && cpu_rd;

    // DA21 4AC (0x02 .. 0x0A)
    wire rd_da21_cnt    = (cpu_addr == 6'h02) && cpu_rd;
    wire rd_da21_ac1_l  = (cpu_addr == 6'h03) && cpu_rd;
    wire rd_da21_ac1_h  = (cpu_addr == 6'h04) && cpu_rd;
    wire rd_da21_ac2_l  = (cpu_addr == 6'h05) && cpu_rd;
    wire rd_da21_ac2_h  = (cpu_addr == 6'h06) && cpu_rd;
    wire rd_da21_ac3_l  = (cpu_addr == 6'h07) && cpu_rd;
    wire rd_da21_ac3_h  = (cpu_addr == 6'h08) && cpu_rd;
    wire rd_da21_ac4_l  = (cpu_addr == 6'h09) && cpu_rd;
    wire rd_da21_ac4_h  = (cpu_addr == 6'h0A) && cpu_rd;

    // DA22 4AC (0x0B .. 0x13)
    wire rd_da22_cnt    = (cpu_addr == 6'h0B) && cpu_rd;
    wire rd_da22_ac1_l  = (cpu_addr == 6'h0C) && cpu_rd;
    wire rd_da22_ac1_h  = (cpu_addr == 6'h0D) && cpu_rd;
    wire rd_da22_ac2_l  = (cpu_addr == 6'h0E) && cpu_rd;
    wire rd_da22_ac2_h  = (cpu_addr == 6'h0F) && cpu_rd;
    wire rd_da22_ac3_l  = (cpu_addr == 6'h10) && cpu_rd;
    wire rd_da22_ac3_h  = (cpu_addr == 6'h11) && cpu_rd;
    wire rd_da22_ac4_l  = (cpu_addr == 6'h12) && cpu_rd;
    wire rd_da22_ac4_h  = (cpu_addr == 6'h13) && cpu_rd;

    // DA45 4DC (0x14 .. 0x17)
    wire rd_da45_dc1    = (cpu_addr == 6'h14) && cpu_rd;
    wire rd_da45_dc2    = (cpu_addr == 6'h15) && cpu_rd;
    wire rd_da45_dc3    = (cpu_addr == 6'h16) && cpu_rd;
    wire rd_da45_dc4    = (cpu_addr == 6'h17) && cpu_rd;
    wire rd_deci        = (cpu_addr == 6'h18) && cpu_rd;

    wire [DATA_WIDTH-1:0] status_reg = { {(DATA_WIDTH-3){1'b0}}, hw_valid_da45, hw_valid_da22, hw_valid_da21 };
    wire [DATA_WIDTH-1:0] ctrl_reg   = { 1'b0, reg_irq_en_12, {(DATA_WIDTH-2){1'b0}} };
    wire [DATA_WIDTH-1:0] deci_reg   = { {(DATA_WIDTH-5){1'b0}}, shift_da45 };

    assign cpu_do = 
        rd_ctrl       ? ctrl_reg           :
        rd_status     ? status_reg         :
        // DA21
        rd_da21_cnt   ? sh_da21_cnt        :
        rd_da21_ac1_l ? sh_da21_ac1[15:0]  :
        rd_da21_ac1_h ? sh_da21_ac1[31:16] :
        rd_da21_ac2_l ? sh_da21_ac2[15:0]  :
        rd_da21_ac2_h ? sh_da21_ac2[31:16] :
        rd_da21_ac3_l ? sh_da21_ac3[15:0]  :
        rd_da21_ac3_h ? sh_da21_ac3[31:16] :
        rd_da21_ac4_l ? sh_da21_ac4[15:0]  :
        rd_da21_ac4_h ? sh_da21_ac4[31:16] :
        // DA22
        rd_da22_cnt   ? sh_da22_cnt        :
        rd_da22_ac1_l ? sh_da22_ac1[15:0]  :
        rd_da22_ac1_h ? sh_da22_ac1[31:16] :
        rd_da22_ac2_l ? sh_da22_ac2[15:0]  :
        rd_da22_ac2_h ? sh_da22_ac2[31:16] :
        rd_da22_ac3_l ? sh_da22_ac3[15:0]  :
        rd_da22_ac3_h ? sh_da22_ac3[31:16] :
        rd_da22_ac4_l ? sh_da22_ac4[15:0]  :
        rd_da22_ac4_h ? sh_da22_ac4[31:16] :
        // DA45
        rd_da45_dc1   ? sh_da45_dc1        :
        rd_da45_dc2   ? sh_da45_dc2        :
        rd_da45_dc3   ? sh_da45_dc3        :
        rd_da45_dc4   ? sh_da45_dc4        :
        rd_deci       ? deci_reg           :
        {DATA_WIDTH{1'b0}};

    // =========================================================================
    // 7. DMA FLAT BUS PACKING (22 words = 352 bits)
    // =========================================================================
    assign adc_data_out = {
        // DA21 Elevation (9 words: 4x 32-bit AC + 1x 16-bit Count)
        sh_da21_ac4, sh_da21_ac3, sh_da21_ac2, sh_da21_ac1, sh_da21_cnt,
        // DA22 Azimuth   (9 words: 4x 32-bit AC + 1x 16-bit Count)
        sh_da22_ac4, sh_da22_ac3, sh_da22_ac2, sh_da22_ac1, sh_da22_cnt,
        // DA45 Telemetry (4 words: 4x 16-bit DC)
        sh_da45_dc4, sh_da45_dc3, sh_da45_dc2, sh_da45_dc1
    };

endmodule