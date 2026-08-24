`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: adc_block (Device ID = 6)
// 
// DESCRIPTION:
//  Combined ADC Subsystem for DA21 (Elevation), DA22 (Azimuth), and DA45 (Telemetry).
//  - DA21 (Hybrid AC/DC) is synchronized to UON23. Generates the main Master IRQ!
//  - DA22 (Hybrid AC/DC) is synchronized to UON12. Updates shadow regs silently.
//  - DA45 (4x DC) runs on internal decimation timer (N=7 -> 128 samples = 1.28 ms).
//
//  SMART AUTO-CLEAR: The 'dma_ack' input automatically clears the IRQ flags 
//  when the STM32 finishes reading the DMA burst, eliminating SPI overhead.
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX9-TQG144)
//  Toolchain:      ISE 14.7 / XST
//  All comments in ASCII English.
// =============================================================================

module adc_block #(
    parameter ADR_WIDTH    = `_D_S_CHIP_ADDR_WIDTH_,
    parameter DATA_WIDTH   = `_D_DATA_WIDTH_,
    parameter SAMPLE_TICKS = 1000
)(
    input  wire                 clk,         
    input  wire                 rst,         

    // --- Polling Interface ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [DATA_WIDTH-1:0]cpu_di,
    input  wire                 cpu_wr,
    input  wire                 cpu_rd,
    output wire [DATA_WIDTH-1:0]cpu_do,

    // --- DMA Interface ---
    output wire [287:0]         adc_data_out,
    output wire                 irq_adc12_ready, // Master 400 Hz Data Ready IRQ
    input  wire                 dma_ack, 

    // --- Synchronization Inputs ---
    input  wire                 comp_uon12,
    input  wire                 comp_uon23,

    // --- Physical ADC Pins: DA21 (Elevation) ---
    output wire                 da21_cnvst_n, 
    output wire                 da21_cs_n, 
    output wire                 da21_sclk, 
    output wire                 da21_addr, 
    output wire                 da21_range0, 
    output wire                 da21_range1,
    input  wire                 da21_busy, 
    input  wire                 da21_dout_a, 
    input  wire                 da21_dout_b,

    // --- Physical ADC Pins: DA22 (Azimuth) ---
    output wire                 da22_cnvst_n, 
    output wire                 da22_cs_n, 
    output wire                 da22_sclk, 
    output wire                 da22_addr, 
    output wire                 da22_range0, 
    output wire                 da22_range1,
    input  wire                 da22_busy, 
    input  wire                 da22_dout_a, 
    input  wire                 da22_dout_b,

    // --- Physical ADC Pins: DA45 (Telemetry) ---
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
    // 1. CONTROL REGISTERS & FLAGS
    // =========================================================================
    reg reg_ac_sel_da21;
    reg reg_ac_sel_da22;
    reg reg_irq_en_12;

    reg [4:0] shift_da21;
    reg [4:0] shift_da22;
    reg [4:0] shift_da45;

    reg hw_valid_da21;
    reg hw_valid_da22;
    reg hw_valid_da45;

    wire wr_ctrl = (cpu_addr == 6'h00) && cpu_wr;
    wire wr_deci = (cpu_addr == 6'h14) && cpu_wr;
    
    // SMART AUTO-CLEAR: Clear flags via SPI Polling OR automatically via DMA ACK
    wire soft_irq_ack = (wr_ctrl && cpu_di[DATA_WIDTH-1]) || dma_ack;

    // =========================================================================
    // 2. DA21: ELEVATION (Hybrid AC/DC synced to UON23)
    // =========================================================================
    wire [31:0] c_da21_ac1;
    wire [31:0] c_da21_ac2;
    wire [15:0] c_da21_cnt;
    wire [15:0] c_da21_dc1;
    wire [15:0] c_da21_dc2;
    wire        c_da21_rdy_ac;
    wire        c_da21_rdy_dc;

    adc_processor_2ac_2dc #( 
        .SAMPLE_TICKS(SAMPLE_TICKS) 
    ) u_da21 (
        .clk        (clk), 
        .rst_n      (rst_n), 
        .ac_sel     (reg_ac_sel_da21), 
        .comp_in    (comp_uon23),
        .shift_sel  (shift_da21),
        .cnvst_n    (da21_cnvst_n), 
        .cs_n       (da21_cs_n), 
        .sclk       (da21_sclk), 
        .addr       (da21_addr), 
        .range0     (da21_range0), 
        .range1     (da21_range1), 
        .busy       (da21_busy), 
        .dout_a     (da21_dout_a), 
        .dout_b     (da21_dout_b),
        .acc_ac_1   (c_da21_ac1), 
        .acc_ac_2   (c_da21_ac2), 
        .cnt_ac     (c_da21_cnt), 
        .ready_ac   (c_da21_rdy_ac),
        .acc_dc_1   (c_da21_dc1), 
        .acc_dc_2   (c_da21_dc2), 
        .ready_dc   (c_da21_rdy_dc)
    );

    reg [31:0] sh_da21_ac1;
    reg [31:0] sh_da21_ac2;
    reg [15:0] sh_da21_cnt;
    reg [15:0] sh_da21_dc1;
    reg [15:0] sh_da21_dc2;

    // =========================================================================
    // 3. DA22: AZIMUTH (Hybrid AC/DC synced to UON12)
    // =========================================================================
    wire [31:0] c_da22_ac1;
    wire [31:0] c_da22_ac2;
    wire [15:0] c_da22_cnt;
    wire [15:0] c_da22_dc1;
    wire [15:0] c_da22_dc2;
    wire        c_da22_rdy_ac;
    wire        c_da22_rdy_dc;

    adc_processor_2ac_2dc #( 
        .SAMPLE_TICKS(SAMPLE_TICKS) 
    ) u_da22 (
        .clk        (clk), 
        .rst_n      (rst_n), 
        .ac_sel     (reg_ac_sel_da22), 
        .comp_in    (comp_uon12),
        .shift_sel  (shift_da22),
        .cnvst_n    (da22_cnvst_n), 
        .cs_n       (da22_cs_n), 
        .sclk       (da22_sclk), 
        .addr       (da22_addr), 
        .range0     (da22_range0), 
        .range1     (da22_range1), 
        .busy       (da22_busy), 
        .dout_a     (da22_dout_a), 
        .dout_b     (da22_dout_b),
        .acc_ac_1   (c_da22_ac1), 
        .acc_ac_2   (c_da22_ac2), 
        .cnt_ac     (c_da22_cnt), 
        .ready_ac   (c_da22_rdy_ac),
        .acc_dc_1   (c_da22_dc1), 
        .acc_dc_2   (c_da22_dc2), 
        .ready_dc   (c_da22_rdy_dc)
    );

    reg [31:0] sh_da22_ac1;
    reg [31:0] sh_da22_ac2;
    reg [15:0] sh_da22_cnt;
    reg [15:0] sh_da22_dc1;
    reg [15:0] sh_da22_dc2;

    // =========================================================================
    // 4. DA45: TELEMETRY (4x DC synced to internal timer)
    // =========================================================================
    wire [15:0] c_da45_dc1;
    wire [15:0] c_da45_dc2;
    wire [15:0] c_da45_dc3;
    wire [15:0] c_da45_dc4;
    wire        c_da45_rdy_dc;

    adc_processor_4dc #( 
        .SAMPLE_TICKS(SAMPLE_TICKS) 
    ) u_da45 (
        .clk        (clk), 
        .rst_n      (rst_n),
        .shift_sel  (shift_da45),
        .cnvst_n    (da45_cnvst_n), 
        .cs_n       (da45_cs_n), 
        .sclk       (da45_sclk), 
        .addr       (da45_addr), 
        .range0     (da45_range0), 
        .range1     (da45_range1), 
        .busy       (da45_busy), 
        .dout_a     (da45_dout_a), 
        .dout_b     (da45_dout_b),
        .acc_ch1    (c_da45_dc1), 
        .acc_ch2    (c_da45_dc2), 
        .acc_ch3    (c_da45_dc3), 
        .acc_ch4    (c_da45_dc4),
        .ready_dc   (c_da45_rdy_dc)
    );

    reg [15:0] sh_da45_dc1;
    reg [15:0] sh_da45_dc2;
    reg [15:0] sh_da45_dc3;
    reg [15:0] sh_da45_dc4;

    // =========================================================================
    // 5. SHADOW REGISTERS UPDATE LOGIC
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            reg_ac_sel_da21 <= 1'b0; 
            reg_ac_sel_da22 <= 1'b0; 
            reg_irq_en_12   <= 1'b0;
            
            shift_da21 <= 5'd6;  
            shift_da22 <= 5'd6;  
            shift_da45 <= 5'd12; 

            hw_valid_da21 <= 1'b0; 
            hw_valid_da22 <= 1'b0; 
            hw_valid_da45 <= 1'b0;
            
            sh_da21_ac1 <= 0; sh_da21_ac2 <= 0; sh_da21_cnt <= 0; sh_da21_dc1 <= 0; sh_da21_dc2 <= 0;
            sh_da22_ac1 <= 0; sh_da22_ac2 <= 0; sh_da22_cnt <= 0; sh_da22_dc1 <= 0; sh_da22_dc2 <= 0;
            sh_da45_dc1 <= 0; sh_da45_dc2 <= 0; sh_da45_dc3 <= 0; sh_da45_dc4 <= 0;
        end else begin
            if (wr_ctrl) begin
                reg_ac_sel_da21 <= cpu_di[0];
                reg_ac_sel_da22 <= cpu_di[1];
                reg_irq_en_12   <= cpu_di[14];
            end

            if (wr_deci) begin
                shift_da21 <= cpu_di[4:0];
                shift_da22 <= cpu_di[9:5];
                shift_da45 <= cpu_di[14:10];
            end

            if (soft_irq_ack) begin
                hw_valid_da21 <= 1'b0; 
                hw_valid_da22 <= 1'b0; 
                hw_valid_da45 <= 1'b0;
            end
            
            if (c_da21_rdy_ac) begin
                sh_da21_ac1   <= c_da21_ac1; 
                sh_da21_ac2   <= c_da21_ac2; 
                sh_da21_cnt   <= c_da21_cnt;
                hw_valid_da21 <= 1'b1;
            end
            if (c_da21_rdy_dc) begin
                sh_da21_dc1 <= c_da21_dc1; 
                sh_da21_dc2 <= c_da21_dc2;
            end

            if (c_da22_rdy_ac) begin
                sh_da22_ac1   <= c_da22_ac1; 
                sh_da22_ac2   <= c_da22_ac2; 
                sh_da22_cnt   <= c_da22_cnt;
                hw_valid_da22 <= 1'b1;
            end
            if (c_da22_rdy_dc) begin
                sh_da22_dc1 <= c_da22_dc1; 
                sh_da22_dc2 <= c_da22_dc2;
            end

            if (c_da45_rdy_dc) begin
                sh_da45_dc1   <= c_da45_dc1; 
                sh_da45_dc2   <= c_da45_dc2;
                sh_da45_dc3   <= c_da45_dc3; 
                sh_da45_dc4   <= c_da45_dc4;
                hw_valid_da45 <= 1'b1;
            end
        end
    end

    // Master 400 Hz Data Ready IRQ: now driven by DA21 (synced to UON23)
    assign irq_adc12_ready = hw_valid_da21 & reg_irq_en_12;

    // =========================================================================
    // 6. POLLING READ MULTIPLEXER
    // =========================================================================
    wire rd_ctrl        = (cpu_addr == 6'h00) && cpu_rd;
    wire rd_status      = (cpu_addr == 6'h01) && cpu_rd;
    wire rd_da21_cnt    = (cpu_addr == 6'h02) && cpu_rd;
    wire rd_da21_ac1_l  = (cpu_addr == 6'h03) && cpu_rd;
    wire rd_da21_ac1_h  = (cpu_addr == 6'h04) && cpu_rd;
    wire rd_da21_ac2_l  = (cpu_addr == 6'h05) && cpu_rd;
    wire rd_da21_ac2_h  = (cpu_addr == 6'h06) && cpu_rd;
    wire rd_da21_dc1    = (cpu_addr == 6'h07) && cpu_rd;
    wire rd_da21_dc2    = (cpu_addr == 6'h08) && cpu_rd;
    wire rd_da22_cnt    = (cpu_addr == 6'h09) && cpu_rd;
    wire rd_da22_ac1_l  = (cpu_addr == 6'h0A) && cpu_rd;
    wire rd_da22_ac1_h  = (cpu_addr == 6'h0B) && cpu_rd;
    wire rd_da22_ac2_l  = (cpu_addr == 6'h0C) && cpu_rd;
    wire rd_da22_ac2_h  = (cpu_addr == 6'h0D) && cpu_rd;
    wire rd_da22_dc1    = (cpu_addr == 6'h0E) && cpu_rd;
    wire rd_da22_dc2    = (cpu_addr == 6'h0F) && cpu_rd;
    wire rd_da45_dc1    = (cpu_addr == 6'h10) && cpu_rd;
    wire rd_da45_dc2    = (cpu_addr == 6'h11) && cpu_rd;
    wire rd_da45_dc3    = (cpu_addr == 6'h12) && cpu_rd;
    wire rd_da45_dc4    = (cpu_addr == 6'h13) && cpu_rd;
    wire rd_deci        = (cpu_addr == 6'h14) && cpu_rd;

    wire [DATA_WIDTH-1:0] status_reg = { {(DATA_WIDTH-3){1'b0}}, hw_valid_da45, hw_valid_da22, hw_valid_da21 };
    wire [DATA_WIDTH-1:0] ctrl_reg   = { 1'b0, reg_irq_en_12, {(DATA_WIDTH-4){1'b0}}, reg_ac_sel_da22, reg_ac_sel_da21 };
    wire [DATA_WIDTH-1:0] deci_reg   = { 1'b0, shift_da45, shift_da22, shift_da21 };

    assign cpu_do = 
        rd_ctrl       ? ctrl_reg           :
        rd_status     ? status_reg         :
        rd_da21_cnt   ? sh_da21_cnt        :
        rd_da21_ac1_l ? sh_da21_ac1[15:0]  :
        rd_da21_ac1_h ? sh_da21_ac1[31:16] :
        rd_da21_ac2_l ? sh_da21_ac2[15:0]  :
        rd_da21_ac2_h ? sh_da21_ac2[31:16] :
        rd_da21_dc1   ? sh_da21_dc1        :
        rd_da21_dc2   ? sh_da21_dc2        :
        rd_da22_cnt   ? sh_da22_cnt        :
        rd_da22_ac1_l ? sh_da22_ac1[15:0]  :
        rd_da22_ac1_h ? sh_da22_ac1[31:16] :
        rd_da22_ac2_l ? sh_da22_ac2[15:0]  :
        rd_da22_ac2_h ? sh_da22_ac2[31:16] :
        rd_da22_dc1   ? sh_da22_dc1        :
        rd_da22_dc2   ? sh_da22_dc2        :
        rd_da45_dc1   ? sh_da45_dc1        :
        rd_da45_dc2   ? sh_da45_dc2        :
        rd_da45_dc3   ? sh_da45_dc3        :
        rd_da45_dc4   ? sh_da45_dc4        :
        rd_deci       ? deci_reg           :
        {DATA_WIDTH{1'b0}};

    // =========================================================================
    // 7. DMA FLAT BUS PACKING (288 bits = 18 words)
    // =========================================================================
    assign adc_data_out = {
        // DA21 (7 words)
        sh_da21_dc2, sh_da21_dc1, sh_da21_ac2, sh_da21_ac1, sh_da21_cnt,
        // DA22 (7 words)
        sh_da22_dc2, sh_da22_dc1, sh_da22_ac2, sh_da22_ac1, sh_da22_cnt,
        // DA45 (4 words)
        sh_da45_dc4, sh_da45_dc3, sh_da45_dc2, sh_da45_dc1
    };

endmodule