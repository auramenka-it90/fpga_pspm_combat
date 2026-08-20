`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE:       resolver_block (Device ID = 4)
// DESCRIPTION:  Resolver-to-Digital Conversion (RDC) Subsystem Wrapper.
//
// FUNCTIONALITY:
//  Interfaces with external AD7367 ADC (DA36) to sample Resolver Sine and Cosine
//  channels. Performs continuous synchronous lock-in integration over a 360-degree
//  carrier cycle (400 Hz) and outputs the results to STM32 via Polling and DMA.
//
//  SMART AUTO-CLEAR: Reading the DMA burst automatically clears the hardware
//  data-ready and overrun flags, saving STM32 SPI communication overhead.
//
// =============================================================================
//                          SPI CPU REGISTER ADDRESS MAP
// =============================================================================
// 0x00 | REG_CTRL          | [RW] B15: IRQ_ACK (W1C), B14: IRQ_EN, B[7:0]: SEL_MATRIX
// 0x01 | REG_STATUS        | [RO] B2: ADC_BUSY, B1: OVERRUN_ERR, B0: DATA_VALID
// 0x02 | REG_SAMPLES_CNT   | [RO] 16-bit Sample Count per 400Hz cycle (~250 samples)
// 0x03 | REG_SIN_ROLL_L    | [RO] 32-bit Integrated Sine Roll (Low Word)
// 0x04 | REG_SIN_ROLL_H    | [RO] 32-bit Integrated Sine Roll (High Word)
// 0x05 | REG_COS_ROLL_L    | [RO] 32-bit Integrated Cosine Roll (Low Word)
// 0x06 | REG_COS_ROLL_H    | [RO] 32-bit Integrated Cosine Roll (High Word)
// 0x07 | REG_SIN_PITCH_L   | [RO] 32-bit Integrated Sine Pitch (Low Word)
// 0x08 | REG_SIN_PITCH_H   | [RO] 32-bit Integrated Sine Pitch (High Word)
// 0x09 | REG_COS_PITCH_L   | [RO] 32-bit Integrated Cosine Pitch (Low Word)
// 0x0A | REG_COS_PITCH_H   | [RO] 32-bit Integrated Cosine Pitch (High Word)
// =============================================================================
//                             DMA FLAT BUS MAPPING
// =============================================================================
// Total Width: 144 bits (9 words).
// Word Order: [0] SAMPLES_CNT, [1:2] COS_PITCH, [3:4] SIN_PITCH, [5:6] COS_ROLL, [7:8] SIN_ROLL
// =============================================================================

module resolver_block #(
    parameter ADR_WIDTH    = `_D_S_CHIP_ADDR_WIDTH_,
    parameter DATA_WIDTH   = `_D_DATA_WIDTH_,
    parameter SAMPLE_TICKS = 1000
)(
    // --- System Clock & Reset ---
    input  wire                 clk,         // System Clock (100 MHz)
    input  wire                 rst,         // Synchronous Reset (Active High)

    // --- Polling Interface (Control Plane) ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [DATA_WIDTH-1:0]cpu_di,
    input  wire                 cpu_wr,
    input  wire                 cpu_rd,
    output wire [DATA_WIDTH-1:0]cpu_do,

    // --- DMA Interface (Data Plane) ---
    output wire [143:0]         res_data_out,
    output wire                 data_ready,  
    input  wire                 dma_ack, 

    // --- Physical ADC Pins (DA36) ---
    input  wire                 comp_in,     // 400 Hz Reference phase input
    output wire                 cnvst_n, 
    output wire                 cs_n, 
    output wire                 sclk, 
    output wire                 addr, 
    output wire                 range0, 
    output wire                 range1, 
    input  wire                 busy, 
    input  wire                 dout_a, 
    input  wire                 dout_b
);

    wire rst_n = ~rst;

    // Suppress unused input bits warning
    wire _unused_cpu_di = &{1'b0, cpu_di[13:8], 1'b0};

    // =========================================================================
    // 1. REGISTER MAP OFFSETS
    // =========================================================================
    localparam P_OFF_CTRL        = 0; 
    localparam P_OFF_STATUS      = 1; 
    localparam P_OFF_SAMPLES     = 2; 
    localparam P_OFF_SIN_ROLL_L  = 3; 
    localparam P_OFF_SIN_ROLL_H  = 4;
    localparam P_OFF_COS_ROLL_L  = 5;
    localparam P_OFF_COS_ROLL_H  = 6;
    localparam P_OFF_SIN_PITCH_L = 7;
    localparam P_OFF_SIN_PITCH_H = 8;
    localparam P_OFF_COS_PITCH_L = 9;
    localparam P_OFF_COS_PITCH_H = 10;

    reg [7:0] reg_sel;
    reg       reg_irq_en;
    reg       hw_data_valid;
    reg       hw_overrun_err;

    // =========================================================================
    // 2. CORE RDC IP CORE INSTANTIATION
    // =========================================================================
    wire [31:0] core_sin_roll, core_cos_roll, core_sin_pitch, core_cos_pitch;
    wire [15:0] core_cnt_samples;
    wire        core_ready_res;

    adc_processor_resolver #(
        .RANGE1_CFG   (1'b0),
        .RANGE0_CFG   (1'b0),
        .SAMPLE_TICKS (SAMPLE_TICKS) 
    ) u_core (
        .clk            (clk),
        .rst_n          (rst_n),
        .sel            (reg_sel),
        .comp_in        (comp_in),
        .cnvst_n        (cnvst_n), 
        .cs_n           (cs_n), 
        .sclk           (sclk), 
        .addr           (addr), 
        .range0         (range0), 
        .range1         (range1), 
        .busy           (busy), 
        .dout_a         (dout_a), 
        .dout_b         (dout_b),
        .acc_sin_roll   (core_sin_roll), 
        .acc_cos_roll   (core_cos_roll),
        .acc_sin_pitch  (core_sin_pitch), 
        .acc_cos_pitch  (core_cos_pitch),
        .cnt_samples    (core_cnt_samples), 
        .ready_res      (core_ready_res)
    );

    // =========================================================================
    // 3. SHADOW REGISTERS & SMART AUTO-CLEAR
    // =========================================================================
    reg [31:0] shadow_sin_roll, shadow_cos_roll, shadow_sin_pitch, shadow_cos_pitch;
    reg [15:0] shadow_cnt_samples;

    wire wr_ctrl      = (cpu_addr == P_OFF_CTRL) && cpu_wr;
    wire soft_irq_ack = (wr_ctrl && cpu_di[DATA_WIDTH-1]) || dma_ack; 

    always @(posedge clk) begin
        if (rst) begin
            reg_sel            <= 8'd0;
            reg_irq_en         <= 1'b0;
            shadow_sin_roll    <= 32'd0; 
            shadow_cos_roll    <= 32'd0;
            shadow_sin_pitch   <= 32'd0; 
            shadow_cos_pitch   <= 32'd0;
            shadow_cnt_samples <= 16'd0;
            hw_data_valid      <= 1'b0;
            hw_overrun_err     <= 1'b0;
        end else begin
            // --- Write Control Register ---
            if (wr_ctrl) begin
                reg_sel    <= cpu_di[7:0];
                reg_irq_en <= cpu_di[14];
            end

            // --- Acknowledge / Clear Flags ---
            if (soft_irq_ack) begin
                hw_data_valid  <= 1'b0;
                hw_overrun_err <= 1'b0;
            end
            
            // --- Atomic Latch from Resolver Core ---
            if (core_ready_res) begin
                shadow_sin_roll    <= core_sin_roll;
                shadow_cos_roll    <= core_cos_roll;
                shadow_sin_pitch   <= core_sin_pitch;
                shadow_cos_pitch   <= core_cos_pitch;
                shadow_cnt_samples <= core_cnt_samples;
                
                if (hw_data_valid && !soft_irq_ack) begin
                    hw_overrun_err <= 1'b1;
                end
                hw_data_valid <= 1'b1;
            end
        end
    end

    assign data_ready = hw_data_valid & reg_irq_en;

    // =========================================================================
    // 4. POLLING BUS READ DECODING
    // =========================================================================
    wire rd_ctrl        = (cpu_addr == P_OFF_CTRL)        && cpu_rd;
    wire rd_status      = (cpu_addr == P_OFF_STATUS)      && cpu_rd;
    wire rd_samples     = (cpu_addr == P_OFF_SAMPLES)     && cpu_rd;
    wire rd_sin_roll_l  = (cpu_addr == P_OFF_SIN_ROLL_L)  && cpu_rd;
    wire rd_sin_roll_h  = (cpu_addr == P_OFF_SIN_ROLL_H)  && cpu_rd;
    wire rd_cos_roll_l  = (cpu_addr == P_OFF_COS_ROLL_L)  && cpu_rd;
    wire rd_cos_roll_h  = (cpu_addr == P_OFF_COS_ROLL_H)  && cpu_rd;
    wire rd_sin_pitch_l = (cpu_addr == P_OFF_SIN_PITCH_L) && cpu_rd;
    wire rd_sin_pitch_h = (cpu_addr == P_OFF_SIN_PITCH_H) && cpu_rd;
    wire rd_cos_pitch_l = (cpu_addr == P_OFF_COS_PITCH_L) && cpu_rd;
    wire rd_cos_pitch_h = (cpu_addr == P_OFF_COS_PITCH_H) && cpu_rd;

    wire [DATA_WIDTH-1:0] status_reg = { {(DATA_WIDTH-3){1'b0}}, busy, hw_overrun_err, hw_data_valid };
    wire [DATA_WIDTH-1:0] ctrl_reg   = { 1'b0, reg_irq_en, {(DATA_WIDTH-10){1'b0}}, reg_sel };

    assign cpu_do = 
        rd_ctrl        ? ctrl_reg                :
        rd_status      ? status_reg              :
        rd_samples     ? shadow_cnt_samples      :
        rd_sin_roll_l  ? shadow_sin_roll[15:0]   :
        rd_sin_roll_h  ? shadow_sin_roll[31:16]  :
        rd_cos_roll_l  ? shadow_cos_roll[15:0]   :
        rd_cos_roll_h  ? shadow_cos_roll[31:16]  :
        rd_sin_pitch_l ? shadow_sin_pitch[15:0]  :
        rd_sin_pitch_h ? shadow_sin_pitch[31:16] :
        rd_cos_pitch_l ? shadow_cos_pitch[15:0]  :
        rd_cos_pitch_h ? shadow_cos_pitch[31:16] :
        {DATA_WIDTH{1'b0}};

    // =========================================================================
    // 5. DMA FLAT BUS PACKING (144 bits = 9 words)
    // =========================================================================
    assign res_data_out = {
        shadow_cnt_samples, // Word 8 (Sample count)
        shadow_cos_pitch,   // Words 7..6 (32-bit Cos Pitch)
        shadow_sin_pitch,   // Words 5..4 (32-bit Sin Pitch)
        shadow_cos_roll,    // Words 3..2 (32-bit Cos Roll)
        shadow_sin_roll     // Words 1..0 (32-bit Sin Roll)
    };

endmodule