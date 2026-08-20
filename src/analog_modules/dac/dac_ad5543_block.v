`timescale 1ns / 1ps

// =============================================================================
// Module: dac_ad5543_block
// Description: SPI wrapper for controlling up to 10 AD5543 DACs.
// Features: 
//   - Manual mode (direct STM32 writes via SPI).
//   - Hardware Triggered DMA mode (one sweep per external trigger pulse).
//   - Bipolar mode support (2's complement to offset binary conversion).
//   - Shared SPI bus topology (1 shared CLK/SDI, 10 individual CS).
// =============================================================================

module dac_ad5543_block #(
    parameter ADR_WIDTH = 6
)(
    input  wire                 clk,
    input  wire                 rst,
    
    // --- SPI Bridge Bus (STM32 FSMC/FMC Interface) ---
    input  wire [ADR_WIDTH-1:0] cpu_addr,
    input  wire [15:0]          cpu_di,
    input  wire                 cpu_wr,
    input  wire                 cpu_rd,
    output reg  [15:0]          cpu_do,
    
    // --- Hardware Auto Mode Inputs ---
    // 10x 16-bit words (DAC1 = [15:0], ..., DAC10 = [159:144])
    input  wire [159:0]         hard_data_in, 
    input  wire                 hard_trigger, // External pulse to start one sweep
    
    // --- Physical DAC Pins (Shared Bus Topology) ---
    output wire [10:1]          dac_cs_n, // 10 Individual Chip Selects
    output wire                 dac_clk,  // Shared Serial Clock
    output wire                 dac_sdi   // Shared Serial Data
);

    // =========================================================================
    // REGISTER ADDRESS MAP
    // =========================================================================
    localparam [ADR_WIDTH-1:0] REG_DAC_DATA_MIN = 6'd1;  // 0x01: DAC 1 Data
    localparam [ADR_WIDTH-1:0] REG_DAC_DATA_MAX = 6'd10; // 0x0A: DAC 10 Data
    localparam [ADR_WIDTH-1:0] REG_BIPOLAR_CFG  = 6'd11; // 0x0B: Bipolar Config (Bits 9:0)
    localparam [ADR_WIDTH-1:0] REG_STATUS       = 6'd12; // 0x0C: Status (Bit 0 = ready)
    localparam [ADR_WIDTH-1:0] REG_HARD_CTRL    = 6'd13; // 0x0D: Hard Mode Ctrl (Bit 15 = EN, Bits 9:0 = Mask)

    // =========================================================================
    // INTERNAL REGISTERS & SIGNALS
    // =========================================================================
    reg  [15:0] dac_data_mem [1:10]; // STM32 readback memory
    reg  [10:1] bipolar_cfg_reg;     // Bipolar mode mask (1 bit per DAC)
    
    reg         hard_control_en;     // Global Auto-Mode Enable flag
    reg  [10:1] hard_dac_sel;        // Auto-Mode Enable Mask (1 bit per DAC)

    // STM32 Request Latch (Buffer)
    reg         stm32_req_pending;
    reg  [3:0]  stm32_req_dac;
    reg  [15:0] stm32_req_data;

    // Hardware Scanner State
    reg         hard_scan_active;    // 1 = currently performing a triggered sweep
    reg  [3:0]  scan_idx;            // Cycles from 1 to 10
    
    // Trigger Edge Detector
    reg         hard_trigger_d;
    wire        hard_trigger_pulse = hard_trigger & ~hard_trigger_d;

    // Core DAC FSM Signals
    reg  [3:0]  active_dac_sel;      // Target DAC index for FSM (1 to 10)
    reg         dac_start;           // Trigger pulse for FSM
    reg  [15:0] dac_tx_data;         // Data latched for FSM
    
    wire        core_cs_n;
    wire        dac_ready;
    
    // Dynamically fetch bipolar configuration for the currently transmitting DAC
    wire        active_bipolar = (active_dac_sel >= 1 && active_dac_sel <= 10) ? bipolar_cfg_reg[active_dac_sel] : 1'b0;

    // =========================================================================
    // CHIP SELECT DEMULTIPLEXER
    // =========================================================================
    genvar i;
    generate
        for (i = 1; i <= 10; i = i + 1) begin : CS_DEMUX
            assign dac_cs_n[i] = (active_dac_sel == i) ? core_cs_n : 1'b1;
        end
    endgenerate

    // =========================================================================
    // REGISTER WRITE LOGIC & STM32 REQUEST BUFFERING
    // =========================================================================
    integer j;
    always @(posedge clk) begin
        if (rst) begin
            bipolar_cfg_reg   <= 10'd0;
            hard_control_en   <= 1'b0;
            hard_dac_sel      <= 10'd0;
            stm32_req_pending <= 1'b0;
            stm32_req_dac     <= 4'd0;
            stm32_req_data    <= 16'd0;
            
            for (j = 1; j <= 10; j = j + 1) begin
                dac_data_mem[j] <= 16'd0;
            end
        end else begin
            // 1. Latch incoming STM32 Commands
            if (cpu_wr) begin
                if (cpu_addr >= REG_DAC_DATA_MIN && cpu_addr <= REG_DAC_DATA_MAX) begin
                    stm32_req_pending           <= 1'b1;
                    stm32_req_dac               <= cpu_addr[3:0];
                    stm32_req_data              <= cpu_di;
                    dac_data_mem[cpu_addr[3:0]] <= cpu_di; 
                end
                else if (cpu_addr == REG_BIPOLAR_CFG) begin
                    bipolar_cfg_reg[10:1] <= cpu_di[9:0];
                end
                else if (cpu_addr == REG_HARD_CTRL) begin 
                    hard_control_en    <= cpu_di[15];
                    hard_dac_sel[10:1] <= cpu_di[9:0];
                end
            end
            
            // 2. Clear STM32 Request once the Arbiter accepts it
            if (dac_start && stm32_req_pending && dac_ready) begin
                stm32_req_pending <= 1'b0;
            end
        end
    end

    // =========================================================================
    // BUS ARBITER & HARDWARE TRIGGERED SCANNER
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            dac_start        <= 1'b0;
            active_dac_sel   <= 4'd0;
            dac_tx_data      <= 16'd0;
            scan_idx         <= 4'd1;
            hard_scan_active <= 1'b0;
            hard_trigger_d   <= 1'b0;
        end else begin
            // Default state for start pulse
            dac_start      <= 1'b0; 
            hard_trigger_d <= hard_trigger;
            
            // 1. Capture the start pulse (Edge Detected)
            if (hard_control_en && hard_trigger_pulse) begin
                hard_scan_active <= 1'b1;
                scan_idx         <= 4'd1; 
            end
            
            // 2. Clear active flag if Auto Mode is disabled by STM32 mid-sweep
            if (!hard_control_en) begin
                hard_scan_active <= 1'b0;
            end

            // 3. Arbiter Execution
            if (dac_ready && !dac_start) begin
                
                if (stm32_req_pending) begin
                    // ---------------------------------------------------------
                    // PRIORITY 1: STM32 Manual Write Request
                    // ---------------------------------------------------------
                    active_dac_sel <= stm32_req_dac;
                    dac_tx_data    <= stm32_req_data;
                    dac_start      <= 1'b1;
                end 
                else if (hard_scan_active) begin
                    // ---------------------------------------------------------
                    // PRIORITY 2: Hardware Triggered Scanner (One Sweep Mode)
                    // ---------------------------------------------------------
                    if (hard_dac_sel[scan_idx]) begin
                        active_dac_sel <= scan_idx;
                        dac_start      <= 1'b1;
                        
                        case (scan_idx)
                            4'd1:  dac_tx_data <= hard_data_in[15:0];
                            4'd2:  dac_tx_data <= hard_data_in[31:16];
                            4'd3:  dac_tx_data <= hard_data_in[47:32];
                            4'd4:  dac_tx_data <= hard_data_in[63:48];
                            4'd5:  dac_tx_data <= hard_data_in[79:64];
                            4'd6:  dac_tx_data <= hard_data_in[95:80];
                            4'd7:  dac_tx_data <= hard_data_in[111:96];
                            4'd8:  dac_tx_data <= hard_data_in[127:112];
                            4'd9:  dac_tx_data <= hard_data_in[143:128];
                            4'd10: dac_tx_data <= hard_data_in[159:144];
                            default: dac_tx_data <= 16'd0;
                        endcase
                    end
                    
                    // Advance scanner index or finish the sweep
                    if (scan_idx == 4'd10) begin
                        hard_scan_active <= 1'b0; // Sweep complete
                    end else begin
                        scan_idx <= scan_idx + 1'b1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // REGISTER READ LOGIC (FPGA -> STM32)
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            cpu_do <= 16'd0;
        end else if (cpu_rd) begin
            if (cpu_addr >= REG_DAC_DATA_MIN && cpu_addr <= REG_DAC_DATA_MAX) begin
                cpu_do <= dac_data_mem[cpu_addr[3:0]];
            end
            else if (cpu_addr == REG_BIPOLAR_CFG) begin
                cpu_do <= {6'd0, bipolar_cfg_reg[10:1]};
            end
            else if (cpu_addr == REG_STATUS) begin
                cpu_do <= {14'd0, stm32_req_pending, dac_ready};
            end
            else if (cpu_addr == REG_HARD_CTRL) begin
                cpu_do <= {hard_control_en, 5'd0, hard_dac_sel[10:1]};
            end
            else begin
                cpu_do <= 16'h0000;
            end
        end
    end

    // =========================================================================
    // CORE DAC SPI CONTROLLER (Instantiation)
    // =========================================================================
    ad5543_dac_ctrl #(
        .CLK_DIV(5) 
    ) u_ad5543_ctrl (
        .clk         (clk),
        .rst         (rst),
        .start       (dac_start),
        .data_in     (dac_tx_data),
        .bipolar_mode(active_bipolar),
        .dac_cs_n    (core_cs_n), 
        .dac_clk     (dac_clk),   
        .dac_sdi     (dac_sdi),   
        .ready       (dac_ready)
    );

endmodule