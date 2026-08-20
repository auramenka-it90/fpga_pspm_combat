`timescale 1ns / 1ps

// =============================================================================
// MODULE:       bist_generator
// DESCRIPTION:  Built-In Self-Test (BIST) 3-Phase 400 Hz Reference Generator.
//
// FUNCTIONALITY:
//  Generates two 50% duty-cycle square waves with an exact 120.0-degree phase 
//  shift at 400 Hz to simulate zero-crossing comparators (UON12, UON23).
//  Used for factory automated test equipment (ATE) and on-board BIST diagnosis.
//
// MATHEMATICAL CONSTANTS (at 100 MHz System Clock):
//  - Total Period:   100,000,000 / 400 = 250,000 clock cycles (2.500 ms)
//  - Half Period:    250,000 / 2       = 125,000 clock cycles (1.250 ms, 50% Duty)
//  - 120-Deg Phase:  250,000 / 3       = 83,333 clock cycles (0.8333 ms, 119.9995 deg)
//
// SYNTHESIS NOTE:
//  Uses the 'f_clog2' function to dynamically calculate the minimum necessary
//  counter bit-width (18 bits), eliminating XST trimming warnings (Xst:1710).
//
//  Target Silicon: Xilinx Spartan-6 (XC6SLX...)
//  Toolchain:      ISE 14.7 / XST
//  All comments in ASCII English.
// =============================================================================

module bist_generator #(
    parameter SYS_CLK_FREQ = 100000000, // System Clock Frequency in Hz (100 MHz)
    parameter TARGET_FREQ  = 400        // Target 3-Phase Frequency in Hz (400 Hz)
)(
    input  wire                 clk,       // System Clock (100 MHz)
    input  wire                 rst,       // Synchronous Reset (Active High)
    
    output reg                  gen_uon12, // Phase 1-2 Simulated Output (0 deg reference)
    output reg                  gen_uon23  // Phase 2-3 Simulated Output (120 deg shifted)
);

    // =========================================================================
    // CONSTANT BIT-WIDTH CALCULATION FUNCTION (ISE 14.7 / Verilog-2001)
    // =========================================================================
    function integer f_clog2;
        input integer depth;
        integer temp;
        begin
            temp = depth;
            for (f_clog2 = 0; temp > 0; f_clog2 = f_clog2 + 1)
                temp = temp >> 1;
        end
    endfunction

    // =========================================================================
    // CONSTANT CALCULATION (Compile-time constants)
    // =========================================================================
    localparam integer PERIOD     = SYS_CLK_FREQ / TARGET_FREQ; // 250,000 ticks
    localparam integer HALF_PER   = PERIOD / 2;                 // 125,000 ticks (50% duty cycle)
    localparam integer PHASE_120  = PERIOD / 3;                 // 83,333 ticks (120 deg shift)

    // Dynamic bit-width calculation: exactly 18 bits for 250,000 ticks (0 warnings)
    localparam integer CNT_WIDTH  = (f_clog2(PERIOD - 1) > 0) ? f_clog2(PERIOD - 1) : 1;

    reg [CNT_WIDTH-1:0] gen_cnt;

    // =========================================================================
    // GENERATOR SEQUENTIAL LOGIC
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            gen_cnt   <= {CNT_WIDTH{1'b0}};
            gen_uon12 <= 1'b0;
            gen_uon23 <= 1'b0;
        end else begin
            // Main Period Counter: Rolls over at PERIOD - 1
            if (gen_cnt >= (PERIOD - 1)) begin
                gen_cnt <= {CNT_WIDTH{1'b0}};
            end else begin
                gen_cnt <= gen_cnt + 1'b1;
            end

            // Phase 1-2: Logic High during first half of period (0 to 180 degrees)
            gen_uon12 <= (gen_cnt < HALF_PER);

            // Phase 2-3: Logic High between 120 and 300 degrees
            gen_uon23 <= (gen_cnt >= PHASE_120) && (gen_cnt < (PHASE_120 + HALF_PER));
        end
    end

endmodule