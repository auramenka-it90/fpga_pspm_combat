`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: debug_module (Device ID = 1)
// DESCRIPTION: System Debug & Configuration Module
// =============================================================================
//                          SPI CPU REGISTER ADDRESS MAP
// =============================================================================
// 0x00 | REG_FEED_BACK | [RW] Scratchpad register for SPI link testing
// 0x01 | REG_CONST     | [RO] Always reads 0xDEAD (Alive check)
// 0x02 | REG_MISC      | [RW] Misc Control (LEDs, Resolver Mux, etc.)
//                      |      Bits [2:0] : Status LEDs
//                      |      Bit  [4]   : Resolver Comparator MUX (0=UON12, 1=UON23)	
//                      |      Bit  [5]   : BIST 400Hz Gen Enable (0=Ext Pins, 1=Internal 400Hz Gen)
// 0x03 | REG_TP_MUX    | [RW] Test Point Multiplexer Control
//                      |      Bits [4:0]   : Select signal for TP5
//                      |      Bits [9:5]   : Select signal for TP6
//                      |      Bits [14:10] : Select signal for TP7
// =============================================================================

module debug_module #(
    parameter ADR_WIDTH  = `_D_S_CHIP_ADDR_WIDTH_, 
    parameter DATA_WIDTH = `_D_DATA_WIDTH_        
)(
    input  wire                  clk,
    input  wire                  rst,
    
    // --- Polling Interface ---
    input  wire [ADR_WIDTH-1:0]  cpu_addr,
    input  wire [DATA_WIDTH-1:0] cpu_di,
    input  wire                  cpu_wr, 
    input  wire                  cpu_rd, 
    output wire [DATA_WIDTH-1:0] cpu_do,
    
    // --- Internal Control Outputs ---
    output wire [DATA_WIDTH-1:0] misc_out,
    output wire [14:0]           tp_mux_out
);

    // =========================================================================
    // 1. REGISTER MAP
    // =========================================================================
    localparam P_OFF_FEED_BACK = 0; 
    localparam P_OFF_CONST     = 1;
    localparam P_OFF_MISC      = 2;
    localparam P_OFF_TP_MUX    = 3;

    localparam [DATA_WIDTH-1:0] P_CONST_VAL = 16'hDEAD;

    reg [DATA_WIDTH-1:0] fb_reg;
    reg [DATA_WIDTH-1:0] misc_reg;
    reg [14:0]           tp_mux_reg;

    // =========================================================================
    // 2. REGISTER LOGIC
    // =========================================================================
    wire wr_fb     = (cpu_addr == P_OFF_FEED_BACK) && cpu_wr;
    wire wr_misc   = (cpu_addr == P_OFF_MISC)      && cpu_wr;
    wire wr_tp_mux = (cpu_addr == P_OFF_TP_MUX)    && cpu_wr;

    always @(posedge clk) begin
        if (rst) begin
            fb_reg     <= {DATA_WIDTH{1'b0}};
            misc_reg   <= {DATA_WIDTH{1'b0}};
            tp_mux_reg <= 15'd0; // Default: All TPs output signal 0
        end else begin
            if (wr_fb)     fb_reg     <= cpu_di;
            if (wr_misc)   misc_reg   <= cpu_di;
            if (wr_tp_mux) tp_mux_reg <= cpu_di[14:0];
        end
    end

    assign misc_out   = misc_reg; 
    assign tp_mux_out = tp_mux_reg;

    // =========================================================================
    // 3. BUS INTERFACE DECODING
    // =========================================================================
    wire rd_fb     = (cpu_addr == P_OFF_FEED_BACK) && cpu_rd;
    wire rd_const  = (cpu_addr == P_OFF_CONST)     && cpu_rd;
    wire rd_misc   = (cpu_addr == P_OFF_MISC)      && cpu_rd;
    wire rd_tp_mux = (cpu_addr == P_OFF_TP_MUX)    && cpu_rd;

    assign cpu_do = 
        rd_fb     ? fb_reg      :
        rd_const  ? P_CONST_VAL :
        rd_misc   ? misc_reg    :
        rd_tp_mux ? {1'b0, tp_mux_reg} :
        {DATA_WIDTH{1'b0}};

endmodule