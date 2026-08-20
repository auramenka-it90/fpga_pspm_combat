`timescale 1ns / 1ps

// =============================================================================
// MODULE: comp_debounce_dyn
// DESCRIPTION: Fast-Response Blanking Debounce Filter with Dynamic Limits.
//              Designed specifically for Zero-Crossing Comparators.
//
// PHASE 1 (Verification): Waits for the signal to be stable for 'verify_limit' 
//                         ticks before switching the output. Minimizes phase delay.
// PHASE 2 (Blanking):     After switching, ignores all input changes for 
//                         'blanking_limit' ticks to filter out high-frequency noise.
// =============================================================================

module comp_debounce_dyn (
    input  wire        clk,            // System clock (e.g., 100 MHz)
    input  wire        rst,            // Synchronous reset (Active High)
    
    // --- Dynamic Configuration ---
    input  wire [15:0] verify_limit,   // Fast verification window (e.g., 100 = 1 us)
    input  wire [15:0] blanking_limit, // Post-trigger lockout window (e.g., 3000 = 30 us)
    
    // --- I/O ---
    input  wire        async_in,       // Raw asynchronous input from physical comparator
    output reg         clean_out       // Fast, debounced, and blanked clean output
);

    // =========================================================================
    // 1. METASTABILITY GUARD (2-Stage Synchronizer)
    // =========================================================================
    // The ASYNC_REG attribute forces the synthesis tool (ISE/Vivado) to place 
    // these two flip-flops in the same slice, minimizing routing delay and 
    // maximizing MTBF (Mean Time Between Failures).
    (* ASYNC_REG = "TRUE" *) reg [1:0] sync_regs;

    always @(posedge clk) begin
        if (rst) begin
            sync_regs <= 2'b00;
        end else begin
            sync_regs <= {sync_regs[0], async_in};
        end
    end
    
    wire sync_in = sync_regs[1];

    // =========================================================================
    // 2. TIMERS & STATE LOGIC
    // =========================================================================
    reg [15:0] verify_timer;
    reg [15:0] blanking_timer;
    reg        blanking_active;

    always @(posedge clk) begin
        if (rst) begin
            clean_out       <= 1'b0;
            verify_timer    <= 16'd0;
            blanking_timer  <= 16'd0;
            blanking_active <= 1'b0;
        end else begin
            
            // --- PHASE 2: Blanking Lockout Timer ---
            // If blanking is active, ignore 'sync_in' and just count down the lockout time.
            if (blanking_active) begin
                if (blanking_timer < blanking_limit - 1) begin
                    blanking_timer <= blanking_timer + 1'b1;
                end else begin
                    blanking_active <= 1'b0; // Blanking window expired, ready for next edge
                end
            end

            // --- PHASE 1: Fast Verification Logic ---
            // Only active when we are NOT in the blanking window.
            if (!blanking_active) begin
                if (sync_in != clean_out) begin
                    // Signal changed: start verification timer
                    if (verify_timer < verify_limit - 1) begin
                        verify_timer <= verify_timer + 1'b1;
                    end else begin
                        // Signal verified: Switch output immediately!
                        clean_out       <= sync_in; 
                        verify_timer    <= 16'd0;
                        
                        // Enter blanking lockout window to ignore subsequent noise
                        blanking_active <= 1'b1;    
                        blanking_timer  <= 16'd0;
                    end
                end else begin
                    // Signal bounced back to original state: reset verification timer
                    verify_timer <= 16'd0;
                end
            end else begin
                // Keep verify timer reset while in blanking mode
                verify_timer <= 16'd0;
            end
            
        end
    end

endmodule