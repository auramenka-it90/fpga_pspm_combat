`include "define.v"
`timescale 1ns / 1ps

// =============================================================================
// MODULE: debug_module (Device ID = 1)
// DESCRIPTION: System Debug, Configuration & MultiBoot Control Module
// =============================================================================
//                          SPI CPU REGISTER ADDRESS MAP
// =============================================================================
// 0x00 | REG_FEED_BACK | [RW] Scratchpad register for SPI link testing
// 0x01 | REG_CONST     | [RO] Always reads 0xDEAD (Alive check)
// 0x02 | REG_MISC      | [RW] Misc Control (LEDs, UART MUX, Resolver Mux, BIST)
//                      |      Bits [2:0] : Status LEDs
//                      |      Bit  [3]   : RS-485 MUX (0=DD4, 1=DD5)
//                      |      Bit  [4]   : Resolver Phase MUX (0=UON12, 1=UON23)	
//                      |      Bit  [5]   : BIST 400Hz Gen Enable (0=Ext, 1=BIST)
// 0x03 | REG_TP_MUX    | [RW] Test Point Multiplexer Control (TP5, TP6, TP7)
// 0x04 | REG_RECONFIG  | [WO] MultiBoot Reconfig Trigger (Write 0xAA55 to Reboot)
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
    output wire [14:0]           tp_mux_out,
    output reg                   reconfig_trigger // 1-cycle pulse to trigger IPROG
);

    // =========================================================================
    // 1. REGISTER MAP OFFSETS
    // =========================================================================
    localparam P_OFF_FEED_BACK = 0; 
    localparam P_OFF_CONST     = 1;
    localparam P_OFF_MISC      = 2;
    localparam P_OFF_TP_MUX    = 3;
    localparam P_OFF_RECONFIG  = 4;

    localparam [DATA_WIDTH-1:0] P_CONST_VAL   = 16'hDEAD;
    localparam [DATA_WIDTH-1:0] P_RECONFIG_KEY = 16'hAA55; // Safety Magic Key

    reg [DATA_WIDTH-1:0] fb_reg;
    reg [DATA_WIDTH-1:0] misc_reg;
    reg [14:0]           tp_mux_reg;

    // =========================================================================
    // 2. REGISTER WRITE LOGIC
    // =========================================================================
    wire wr_fb       = (cpu_addr == P_OFF_FEED_BACK) && cpu_wr;
    wire wr_misc     = (cpu_addr == P_OFF_MISC)      && cpu_wr;
    wire wr_tp_mux   = (cpu_addr == P_OFF_TP_MUX)    && cpu_wr;
    wire wr_reconfig = (cpu_addr == P_OFF_RECONFIG)  && cpu_wr && (cpu_di == P_RECONFIG_KEY);

    always @(posedge clk) begin
        if (rst) begin
            fb_reg           <= {DATA_WIDTH{1'b0}};
            misc_reg         <= {DATA_WIDTH{1'b0}};
            tp_mux_reg       <= 15'd0; // Default: All TPs output signal 0
            reconfig_trigger <= 1'b0;
        end else begin
            reconfig_trigger <= 1'b0; // Default: 1-cycle pulse

            if (wr_fb)       fb_reg           <= cpu_di;
            if (wr_misc)     misc_reg         <= cpu_di;
            if (wr_tp_mux)   tp_mux_reg       <= cpu_di[14:0];
            if (wr_reconfig) reconfig_trigger <= 1'b1; // Trigger IPROG reboot!
        end
    end

    assign misc_out   = misc_reg; 
    assign tp_mux_out = tp_mux_reg;

    // =========================================================================
    // 3. BUS INTERFACE READ DECODING
    // =========================================================================
    wire rd_fb     = (cpu_addr == P_OFF_FEED_BACK) && cpu_rd;
    wire rd_const  = (cpu_addr == P_OFF_CONST)     && cpu_rd;
    wire rd_misc   = (cpu_addr == P_OFF_MISC)      && cpu_rd;
    wire rd_tp_mux = (cpu_addr == P_OFF_TP_MUX)    && cpu_rd;

    assign cpu_do = 
        rd_fb     ? fb_reg             :
        rd_const  ? P_CONST_VAL        :
        rd_misc   ? misc_reg           :
        rd_tp_mux ? {1'b0, tp_mux_reg} :
        {DATA_WIDTH{1'b0}};

endmodule