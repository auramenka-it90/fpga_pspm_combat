`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: comparator_block (Device ID = 5)
// 
// DESCRIPTION:
//  Dual Comparator Debounce Filter & Frequency Watchdog.
//  Processes raw asynchronous signals from hardware zero-crossing comparators 
//  (UON12, UON23). Applies a dynamic 2-phase filter (Fast Verification + 
//  Blanking Lockout). Continuously measures the period of the cleaned signals.
//  Period data is exposed via Polling and DMA.
//
// =============================================================================
//                                TIMEBASE NOTES
// =============================================================================
//  - Debounce Timers (VERIFY & BLANKING): 
//      Driven by system clock (100 MHz). 1 LSB = 10 ns.
//  - Period Timers (MIN_PER, MAX_PER, PER_12, PER_23): 
//      Driven by internal prescaler (10 MHz). 1 LSB = 0.1 us.
//
// =============================================================================
//                          SPI CPU REGISTER ADDRESS MAP
// =============================================================================
// 0x00 | REG_CTRL      | [RW] B15: IRQ_ACK, B14: IRQ_EN, B0: EN_MEASUREMENT
// 0x01 | REG_STATUS    | [RO] B3: ERR_23, B2: ERR_12, B1: STATE_23, B0: STATE_12
// 0x02 | REG_VERIFY    | [RW] Fast Verification (1 LSB = 10ns. Def: 100 = 1us)
// 0x03 | REG_BLANKING  | [RW] Blanking Lockout  (1 LSB = 10ns. Def: 3000 = 30us)
// 0x04 | REG_MIN_PER   | [RW] Min Period (Max Freq 425Hz). Def: 23529 (2.35 ms)
// 0x05 | REG_MAX_PER   | [RW] Max Period (Min Freq 375Hz). Def: 26666 (2.66 ms)
// 0x06 | REG_PER_12    | [RO] Measured Period UON12 (1 LSB = 0.1 us)
// 0x07 | REG_PER_23    | [RO] Measured Period UON23 (1 LSB = 0.1 us)
// =============================================================================
//                             DMA FLAT BUS MAPPING
// =============================================================================
// Total: 32 bits (2 words). 
// Order: [0] PER_12, [1] PER_23
// =============================================================================

module comparator_block #(
    parameter ADR_WIDTH  = `_D_S_CHIP_ADDR_WIDTH_,
    parameter DATA_WIDTH = `_D_DATA_WIDTH_
)(
    input  wire                 clk,         
    input  wire                 rst,         

    // --- Polling Interface ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [DATA_WIDTH-1:0]cpu_di,
    input  wire                 cpu_wr,
    input  wire                 cpu_rd,
    output wire [DATA_WIDTH-1:0]cpu_do,

    // --- DMA Interface (32 bits = 2 words) ---
    output wire [31:0]          comp_dma_out,

    // --- Physical Inputs ---
    input  wire                 uon12_in,
    input  wire                 uon23_in,

    // --- Clean Outputs ---
    output wire                 uon12_clean,
    output wire                 uon23_clean,

    // --- Interrupt ---
    output wire                 irq_freq_err
);

    // =========================================================================
    // 1. REGISTER MAP & DEFAULTS
    // =========================================================================
    localparam P_OFF_CTRL      = 0;
    localparam P_OFF_STATUS    = 1;
    localparam P_OFF_VERIFY    = 2;
    localparam P_OFF_BLANKING  = 3;
    localparam P_OFF_MIN_PER   = 4;
    localparam P_OFF_MAX_PER   = 5;
    localparam P_OFF_PER_12    = 6;
    localparam P_OFF_PER_23    = 7;

    reg        reg_en_meas;
    reg        reg_irq_en;
    reg [15:0] reg_verify;
    reg [15:0] reg_blanking;
    reg [15:0] reg_min_per;
    reg [15:0] reg_max_per;

    // =========================================================================
    // 2. PRESCALER (100 MHz -> 10 MHz / 0.1 us tick)
    // =========================================================================
    reg [3:0] prescaler;
    wire tick_0_1us = (prescaler == 4'd9);

    always @(posedge clk) begin
        if (rst) prescaler <= 4'd0;
        else if (tick_0_1us) prescaler <= 4'd0;
        else prescaler <= prescaler + 1'b1;
    end

    // =========================================================================
    // 3. DEBOUNCE INSTANCES (Run at 100 MHz for max precision)
    // =========================================================================
    comp_debounce_dyn u_deb_12 (
        .clk            (clk),
        .rst            (rst),
        .verify_limit   (reg_verify),
        .blanking_limit (reg_blanking),
        .async_in       (uon12_in),
        .clean_out      (uon12_clean)
    );

    comp_debounce_dyn u_deb_23 (
        .clk            (clk),
        .rst            (rst),
        .verify_limit   (reg_verify),
        .blanking_limit (reg_blanking),
        .async_in       (uon23_in),
        .clean_out      (uon23_clean)
    );

    // =========================================================================
    // 4. PERIOD MEASUREMENT & WATCHDOG (Runs on 0.1 us ticks)
    // =========================================================================
    reg [15:0] cnt_12, cnt_23;
    reg [15:0] period_12, period_23;
    reg err_12, err_23;

    // Edge detectors for clean signals (100 MHz resolution)
    reg uon12_d, uon23_d;
    always @(posedge clk) begin
        uon12_d <= uon12_clean;
        uon23_d <= uon23_clean;
    end
    wire edge_12 = uon12_clean & ~uon12_d;
    wire edge_23 = uon23_clean & ~uon23_d;

    wire wr_ctrl = (cpu_addr == P_OFF_CTRL) && cpu_wr;
    wire irq_ack = wr_ctrl && cpu_di[DATA_WIDTH-1];

    always @(posedge clk) begin
        if (rst) begin
            reg_en_meas  <= 1'b1;
            reg_irq_en   <= 1'b0;
            reg_verify   <= 16'd100;       // 1 us
            reg_blanking <= 16'd3000;      // 30 us
            reg_min_per  <= 16'd23529;     // 425 Hz (2.35 ms)
            reg_max_per  <= 16'd26666;     // 375 Hz (2.66 ms)
            
            cnt_12 <= 16'd0; cnt_23 <= 16'd0;
            period_12 <= 16'd0; period_23 <= 16'd0;
            err_12 <= 1'b0; err_23 <= 1'b0;
        end else begin
            // --- SPI Writes ---
            if (wr_ctrl) begin
                reg_en_meas <= cpu_di[0];
                reg_irq_en  <= cpu_di[14];
            end
            if (cpu_wr && cpu_addr == P_OFF_VERIFY)   reg_verify   <= cpu_di;
            if (cpu_wr && cpu_addr == P_OFF_BLANKING) reg_blanking <= cpu_di;
            if (cpu_wr && cpu_addr == P_OFF_MIN_PER)  reg_min_per  <= cpu_di;
            if (cpu_wr && cpu_addr == P_OFF_MAX_PER)  reg_max_per  <= cpu_di;

            // --- Clear Errors ---
            if (irq_ack) begin
                err_12 <= 1'b0;
                err_23 <= 1'b0;
            end

            // --- Measurement Logic UON12 ---
            if (reg_en_meas) begin
                if (edge_12) begin
                    period_12 <= cnt_12;
                    cnt_12    <= 16'd0;
                    // Check bounds on complete cycle
                    if (cnt_12 < reg_min_per || cnt_12 > reg_max_per) err_12 <= 1'b1;
                end else if (tick_0_1us) begin
                    // Timeout protection (Signal lost or frequency too low)
                    if (cnt_12 > reg_max_per) err_12 <= 1'b1;
                    else cnt_12 <= cnt_12 + 1'b1;
                end
            end

            // --- Measurement Logic UON23 ---
            if (reg_en_meas) begin
                if (edge_23) begin
                    period_23 <= cnt_23;
                    cnt_23    <= 16'd0;
                    if (cnt_23 < reg_min_per || cnt_23 > reg_max_per) err_23 <= 1'b1;
                end else if (tick_0_1us) begin
                    if (cnt_23 > reg_max_per) err_23 <= 1'b1;
                    else cnt_23 <= cnt_23 + 1'b1;
                end
            end
        end
    end

    assign irq_freq_err = (err_12 | err_23) & reg_irq_en;

    // =========================================================================
    // 5. BUS INTERFACE DECODING
    // =========================================================================
    wire rd_ctrl      = (cpu_addr == P_OFF_CTRL)      && cpu_rd;
    wire rd_status    = (cpu_addr == P_OFF_STATUS)    && cpu_rd;
    wire rd_verify    = (cpu_addr == P_OFF_VERIFY)    && cpu_rd;
    wire rd_blanking  = (cpu_addr == P_OFF_BLANKING)  && cpu_rd;
    wire rd_min_per   = (cpu_addr == P_OFF_MIN_PER)   && cpu_rd;
    wire rd_max_per   = (cpu_addr == P_OFF_MAX_PER)   && cpu_rd;
    wire rd_per_12    = (cpu_addr == P_OFF_PER_12)    && cpu_rd;
    wire rd_per_23    = (cpu_addr == P_OFF_PER_23)    && cpu_rd;

    wire [DATA_WIDTH-1:0] status_reg = { {(DATA_WIDTH-4){1'b0}}, err_23, err_12, uon23_clean, uon12_clean };
    wire [DATA_WIDTH-1:0] ctrl_reg   = { 1'b0, reg_irq_en, {(DATA_WIDTH-3){1'b0}}, reg_en_meas };

    assign cpu_do = 
        rd_ctrl      ? ctrl_reg     :
        rd_status    ? status_reg   :
        rd_verify    ? reg_verify   :
        rd_blanking  ? reg_blanking :
        rd_min_per   ? reg_min_per  :
        rd_max_per   ? reg_max_per  :
        rd_per_12    ? period_12    :
        rd_per_23    ? period_23    :
        {DATA_WIDTH{1'b0}};

    // =========================================================================
    // 6. DMA FLAT BUS PACKING (32 bits = 2 words)
    // =========================================================================
    assign comp_dma_out = {
        period_23, 
        period_12
    };

endmodule