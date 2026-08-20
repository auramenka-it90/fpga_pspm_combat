`timescale 1ns / 1ps

// =============================================================================
// MODULE:       spi_stm32_fpga_bridge
// DESCRIPTION:  Synchronous 16-bit Single-Register SPI Polling Bridge.
//
// FUNCTIONALITY:
//  - SPI Mode 00 Slave interface using NSS_P (PB0).
//  - 32-bit transaction frame:
//      * Word 1 (Bits 0..15):  [15]=R/W, [9:6]=Device ID, [5:0]=Register Offset
//      * Word 2 (Bits 16..31): 16-bit Data Read/Write payload
//  - Zero-Latency Read Latching on bit 15.
// =============================================================================

module spi_stm32_fpga_bridge (
    input  wire        clk,       // System clock (100 MHz)
    input  wire        rst,       // Synchronous reset (Active High)
    
    // --- SPI Interface ---
    input  wire        spi_in,    // MOSI
    input  wire        sck_in,    // SCK
    input  wire        nss_in,    // NSS_P (Active Low)
    output reg         spi_out,   // MISO
    
    // --- Local Bus Interface ---
    output reg  [9:0]  addr,      // 10-bit Register Address ([9:6] Dev, [5:0] Reg)
    output reg  [15:0] dout,      // Data to FPGA (Write)
    input  wire [15:0] din,       // Data from FPGA (Read)
    output reg         wr_strobe, // 1-cycle write strobe
    output reg         rd,        // 1-cycle read strobe
    output wire        busy       // High while CS is active
);

    // =========================================================================
    // 1. SYNCHRONIZERS & EDGE DETECTORS
    // =========================================================================
    reg [2:0] sck_sync;
    reg [2:0] nss_sync;
    reg [1:0] mosi_sync;

    always @(posedge clk) begin
        if (rst) begin
            sck_sync  <= 3'b000;
            nss_sync  <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sck_sync  <= {sck_sync[1:0], sck_in};
            nss_sync  <= {nss_sync[1:0], nss_in};
            mosi_sync <= {mosi_sync[0],  spi_in};
        end
    end

    wire sck_rising  = (sck_sync[1:0] == 2'b01);
    wire sck_falling = (sck_sync[1:0] == 2'b10);
    wire nss_active  = ~nss_sync[1];

    assign busy = nss_active;

    // =========================================================================
    // 2. SPI POLLING FSM & SHIFT REGISTERS
    // =========================================================================
    reg [5:0]  bit_cnt;
    reg [14:0] rx_shifter; // Exactly 15 bits: eliminates unused node warnings
    reg [15:0] tx_shifter;
    reg        is_write;

    always @(posedge clk) begin
        if (rst) begin
            bit_cnt    <= 6'd0;
            rx_shifter <= 15'd0;
            tx_shifter <= 16'd0;
            addr       <= 10'd0;
            dout       <= 16'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            is_write   <= 1'b0;
            spi_out    <= 1'b0;
        end else if (!nss_active) begin
            bit_cnt    <= 6'd0;
            rx_shifter <= 15'd0;
            wr_strobe  <= 1'b0;
            rd         <= 1'b0;
            spi_out    <= 1'b0; 
        end else begin
            wr_strobe <= 1'b0;
            rd        <= 1'b0;

            // Zero-latency read latch
            if (rd) begin
                tx_shifter <= din;
            end

            // MOSI Sampling on SCK Rising Edge
            if (sck_rising) begin
                bit_cnt    <= bit_cnt + 1'b1;
                rx_shifter <= {rx_shifter[13:0], mosi_sync[1]};
                
                // Decode Command & Address at Bit 15
                if (bit_cnt == 6'd15) begin
                    is_write <= rx_shifter[14];
                    addr     <= {rx_shifter[8:0], mosi_sync[1]};
                    
                    if (rx_shifter[14] == 1'b0) begin
                        rd <= 1'b1; // Trigger read pulse
                    end
                end

                // Execute Write at Bit 31
                if (bit_cnt == 6'd31) begin
                    if (is_write) begin
                        dout      <= {rx_shifter[14:0], mosi_sync[1]};
                        wr_strobe <= 1'b1;
                    end
                end
            end

            // MISO Shift out on SCK Falling Edge
            if (sck_falling) begin
                if (bit_cnt == 6'd16) begin
                    if (!is_write) begin
                        spi_out    <= tx_shifter[15];
                        tx_shifter <= {tx_shifter[14:0], 1'b0};
                    end else begin
                        spi_out    <= 1'b0;
                    end
                end else if (bit_cnt > 6'd16) begin
                    if (!is_write) begin
                        spi_out    <= tx_shifter[15];
                        tx_shifter <= {tx_shifter[14:0], 1'b0};
                    end else begin
                        spi_out    <= 1'b0;
                    end
                end
            end
        end
    end

endmodule