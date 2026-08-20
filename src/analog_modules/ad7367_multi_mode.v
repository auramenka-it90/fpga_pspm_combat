`timescale 1ns / 1ps

// =============================================================================
// MODULE:       ad7367_multi_mode
// DESCRIPTION:  Hardware Controller for Analog Devices AD7367 Simultaneous ADC.
//
// KEY FEATURES:
//  - High-Speed SPI Mode 00 Controller: Generates 25.0 MHz SCLK from 100 MHz clock.
//  - Automatic Bipolar Sign Extension: Converts raw 14-bit two's complement ADC
//    codes into standard signed 16-bit integers (int16_t).
//  - Anti-Metastability Guard: 2-stage synchronizer on the asynchronous 'busy' pin.
//  - Auto-Reset Hardware Watchdog: Recovers from disconnected cables or stuck ADC.
//  - Strict Timing Margins:
//      * t_CNVST Pulse Width:  30 ns (Datasheet requires min 10 ns) -> +200% margin
//      * t_CONV Busy Timeout:  2.55 us (Datasheet max is 1.60 us)   -> Safe watchdog
//      * t_CS Setup to SCLK:   40 ns (Datasheet requires min 10 ns) -> +300% margin
// =============================================================================

module ad7367_multi_mode (
    // System Interface
    input  wire        clk,          // System Clock (100 MHz, 10ns period)
    input  wire        rst_n,        // Synchronous Reset (Active Low)
    input  wire        start,        // Single-cycle pulse to trigger conversion
    input  wire [1:0]  mode,         // Channel mode (00: All channels A1/B1/A2/B2)

    // ADC Range Configuration
    input  wire        cfg_range0,   // Range 0 configuration level
    input  wire        cfg_range1,   // Range 1 configuration level

    // ADC Physical Pins
    output reg         cnvst_n,      // Conversion Start Pulse (Active Low)
    output reg         cs_n,         // Chip Select Output (Active Low)
    output wire        sclk,         // Serial Shift Clock (25.0 MHz)
    output reg         addr,         // Channel Address Line (0: Channel 1, 1: Channel 2)
    output wire        range0,       // Range 0 Pin Output
    output wire        range1,       // Range 1 Pin Output
    input  wire        busy,         // Asynchronous conversion status input
    input  wire        dout_a,       // Serial Data Output A
    input  wire        dout_b,       // Serial Data Output B

    // Parallel Output Bus (Signed 14-bit data extended to 16-bit int16_t)
    output reg [15:0]  va1,          // Channel A1 voltage sample
    output reg [15:0]  vb1,          // Channel B1 voltage sample
    output reg [15:0]  va2,          // Channel A2 voltage sample
    output reg [15:0]  vb2,          // Channel B2 voltage sample

    // Status Flags
    output reg         ready,        // 1-cycle strobe indicating all data is latched
    output wire        module_busy,  // High while ADC conversion/readout is in progress
    output wire        err_timeout   // High if ADC hardware fails to respond (Watchdog)
);

    // Direct range pass-through routing
    assign range0 = cfg_range0;
    assign range1 = cfg_range1;

    // Suppress unused mode input warning (Mode is fixed to 2'b00)
    wire _unused_mode = &{1'b0, mode, 1'b0};

    // State Machine Definitions (Explicit binary encoding for XST optimization)
    localparam [3:0] 
        ST_IDLE         = 4'd0,
        ST_CNVST        = 4'd1,
        ST_WAIT_BUSY_H  = 4'd2,
        ST_WAIT_BUSY_L  = 4'd3,
        ST_CS_SETUP     = 4'd4, 
        ST_READ         = 4'd5,
        ST_STORE        = 4'd6,
        ST_CHECK_NEXT   = 4'd7,
        ST_DONE         = 4'd8,
        ST_HW_RESET     = 4'd9,
        ST_HW_RECOVER   = 4'd10;

    reg [3:0]  state;
    reg        conv_step; // 0: Channel 1 (A1/B1), 1: Channel 2 (A2/B2)
    reg [7:0]  timer;
    reg [3:0]  bit_cnt;
    reg [1:0]  sclk_cnt;
    reg [13:0] shift_reg_a;
    reg [13:0] shift_reg_b;

    // =========================================================================
    // 1. 2-STAGE ANTI-METASTABILITY SYNCHRONIZER FOR BUSY LINE
    // =========================================================================
    reg busy_sync1;
    reg busy_sync_safe;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy_sync1     <= 1'b0;
            busy_sync_safe <= 1'b0;
        end else begin
            busy_sync1     <= busy;          // First flip-flop: samples asynchronous pad
            busy_sync_safe <= busy_sync1;    // Second flip-flop: delivers clean signal to FSM
        end
    end

    // SCLK Generation: 25.0 MHz (100MHz / 4), strictly tied to 0 outside READ state
    assign sclk        = (state == ST_READ) ? sclk_cnt[1] : 1'b0;
    assign module_busy = (state != ST_IDLE);
    assign err_timeout = (state == ST_HW_RECOVER);

    // =========================================================================
    // 2. MAIN ADC FINITE STATE MACHINE (FSM)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            cnvst_n      <= 1'b1;
            cs_n         <= 1'b1;
            addr         <= 1'b0;
            va1          <= 16'd0;
            vb1          <= 16'd0;
            va2          <= 16'd0;
            vb2          <= 16'd0;
            ready        <= 1'b0;
            conv_step    <= 1'b0;
            timer        <= 8'd0;
            bit_cnt      <= 4'd0;
            sclk_cnt     <= 2'd0;
            shift_reg_a  <= 14'd0;
            shift_reg_b  <= 14'd0;
        end else begin
            ready <= 1'b0; // Default single-cycle strobe
            
            case (state)
                ST_IDLE: begin
                    addr <= 1'b0; 
                    if (start) begin
                        conv_step <= 1'b0;
                        addr      <= 1'b1; // Setup address for first channel conversion
                        timer     <= 8'd0;
                        state     <= ST_CNVST;
                    end
                end

                // Generate 30 ns Conversion Start Pulse
                ST_CNVST: begin
                    cnvst_n <= 1'b0;
                    if (timer == 8'd2) begin 
                        cnvst_n <= 1'b1;
                        timer   <= 8'd0;
                        state   <= ST_WAIT_BUSY_H;
                    end else timer <= timer + 1'b1;
                end

                // Wait for ADC to assert BUSY High (Timeout = 1.28 us)
                ST_WAIT_BUSY_H: begin
                    if (busy_sync_safe) begin
                        timer <= 8'd0;
                        state <= ST_WAIT_BUSY_L;
                    end else if (timer == 8'd128) state <= ST_HW_RESET; 
                    else timer <= timer + 1'b1;
                end

                // Wait for ADC to complete conversion and drop BUSY Low (Timeout = 2.55 us)
                ST_WAIT_BUSY_L: begin
                    if (!busy_sync_safe) begin
                        cs_n  <= 1'b0; // Assert CS
                        timer <= 8'd0;
                        state <= ST_CS_SETUP;
                    end else if (timer == 8'd255) state <= ST_HW_RESET; 
                    else timer <= timer + 1'b1;
                end

                // CS to SCLK setup time delay (40 ns)
                ST_CS_SETUP: begin
                    if (timer == 8'd3) begin 
                        timer    <= 8'd0;
                        bit_cnt  <= 4'd13; // 14 serial bits (13 down to 0)
                        sclk_cnt <= 2'd0;
                        state    <= ST_READ;
                    end else timer <= timer + 1'b1;
                end

                // Readout Data at 25 MHz (Sampling on SCLK rising edge)
                ST_READ: begin
                    sclk_cnt <= sclk_cnt + 1'b1;
                    
                    if (sclk_cnt == 2'd2) begin
                        shift_reg_a <= {shift_reg_a[12:0], dout_a};
                        shift_reg_b <= {shift_reg_b[12:0], dout_b};
                    end
                    
                    if (sclk_cnt == 2'd3) begin
                        if (bit_cnt == 4'd0) state <= ST_STORE;
                        else bit_cnt <= bit_cnt - 1'b1;
                    end
                end

                // Latch conversion results with automatic sign extension (int16_t)
                ST_STORE: begin
                    cs_n <= 1'b1; // Deassert CS
                    if (conv_step == 1'b0) begin
                        va1 <= {{2{shift_reg_a[13]}}, shift_reg_a};
                        vb1 <= {{2{shift_reg_b[13]}}, shift_reg_b};
                    end else begin
                        va2 <= {{2{shift_reg_a[13]}}, shift_reg_a};
                        vb2 <= {{2{shift_reg_b[13]}}, shift_reg_b};
                    end
                    state <= ST_CHECK_NEXT;
                end

                // Check if Channel 2 sequence needs to be performed
                ST_CHECK_NEXT: begin
                    if (conv_step == 1'b0) begin
                        conv_step <= 1'b1;
                        addr      <= 1'b0; // Setup address for Channel 2
                        state     <= ST_CNVST;
                    end else state <= ST_DONE;
                end

                ST_DONE: begin
                    ready <= 1'b1; // Pulse ready flag for 1 clock cycle
                    state <= ST_IDLE;
                end

                // Hardware timeout recovery sequence
                ST_HW_RESET: begin
                    cs_n <= 1'b1; cnvst_n <= 1'b1; addr <= 1'b1;
                    if (timer == 8'd16) begin timer <= 8'd0; state <= ST_HW_RECOVER; end 
                    else timer <= timer + 1'b1;
                end
                
                ST_HW_RECOVER: begin
                    addr <= 1'b0;
                    if (timer == 8'd16) begin state <= ST_IDLE; end 
                    else timer <= timer + 1'b1;
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
endmodule