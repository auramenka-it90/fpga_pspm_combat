`timescale 1ns / 1ps

// =============================================================================
// MODULE:       spi_stm32_multi_word_bridge
// DESCRIPTION:  Multi-Word Asymmetric Burst SPI DMA Streaming Bridge.
//
// FUNCTIONALITY:
//  - Implements high-speed continuous streaming over SPI Mode 00 using NSS_D.
//  - Asymmetric buffer sizing:
//      * READ_WORDS  (29 words = 464 bits) transferred to STM32 in one DMA burst.
//      * WRITE_WORDS (10 words = 160 bits) transferred from STM32 for 10x DACs.
//  - Atomic Write Latching: On the rising edge of NSS_D, all received DAC data
//    is latched into the 'dout' register and 'wr_strobe' pulses for 1 cycle.
// =============================================================================

module spi_stm32_multi_word_bridge #(
    parameter READ_WORDS  = 29, // 29 words (464 bits) DMA Read Stream
    parameter WRITE_WORDS = 10  // 10 words (160 bits) DMA Write Stream
)(
    input  wire                               clk,       // System Clock (100 MHz)
    input  wire                               rst,       // Synchronous Reset (Active High)
    
    // --- SPI Physical Interface ---
    input  wire                               spi_in,    // MOSI
    input  wire                               sck_in,    // SCK
    input  wire                               nss_in,    // NSS_D (Active Low)
    output reg                                spi_out,   // MISO
    
    // --- Flat Bus Interface ---
    output reg  [(16*WRITE_WORDS)-1:0]        dout,      // 160-bit DAC data to FPGA
    input  wire [(16*READ_WORDS)-1:0]         din,       // 464-bit Telemetry from FPGA
    output reg                                wr_strobe, // 1-cycle write completion pulse
    output reg                                rd_strobe, // 1-cycle read latch pulse
    output wire                               busy       // High while transaction is active
);

    // =========================================================================
    // 1. SYNCHRONIZERS & EDGE DETECTORS (3-FF Metastability Guard)
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
    wire nss_falling = (nss_sync[1:0] == 2'b10); // Start of DMA transaction
    wire nss_rising  = (nss_sync[1:0] == 2'b01); // End of DMA transaction

    assign busy = nss_active;

    // =========================================================================
    // 2. STATE REGISTERS
    // =========================================================================
    reg [8:0]                  bit_cnt;
    reg [14:0]                 rx_word_shifter;
    reg [(16*WRITE_WORDS)-1:0] rx_burst_buffer;
    reg [(16*READ_WORDS)-1:0]  tx_shifter_burst;
    reg [4:0]                  burst_len;
    reg                        is_write;
    reg                        is_read;
    reg [3:0]                  word_idx; // Exactly 4 bits for 10 words (0..10)

    integer w;

    // =========================================================================
    // 3. MAIN DMA STREAMING STATE MACHINE
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            bit_cnt          <= 9'd0;
            rx_word_shifter  <= 15'd0;
            rx_burst_buffer  <= {(16*WRITE_WORDS){1'b0}};
            tx_shifter_burst <= {(16*READ_WORDS){1'b0}};
            dout             <= {(16*WRITE_WORDS){1'b0}};
            wr_strobe        <= 1'b0;
            rd_strobe        <= 1'b0;
            is_write         <= 1'b0;
            is_read          <= 1'b0;
            burst_len        <= 5'd0;
            word_idx         <= 4'd0;
            spi_out          <= 1'b0;
        end else begin
            wr_strobe <= 1'b0;
            rd_strobe <= 1'b0;

            // --- 1. START TRANSACTION (Reset indices on CS falling edge) ---
            if (nss_falling) begin
                bit_cnt         <= 9'd0;
                rx_word_shifter <= 15'd0;
                word_idx        <= 4'd0;
                spi_out         <= 1'b0;
            end

            // --- 2. ACTIVE TRANSFER ---
            if (nss_active) begin
                
                // Sample MOSI on SCK rising edge
                if (sck_rising) begin
                    if (bit_cnt < (16 + (READ_WORDS << 4))) bit_cnt <= bit_cnt + 1'b1;
                    
                    rx_word_shifter <= {rx_word_shifter[13:0], mosi_sync[1]};

                    // Decode Command Word at bit 15
                    if (bit_cnt == 9'd15) begin
                        is_write  <= rx_word_shifter[14];
                        is_read   <= (!rx_word_shifter[14]) || rx_word_shifter[13];
                        burst_len <= (rx_word_shifter[3:0] == 4'd0 && mosi_sync[1] == 1'b0) ? 5'd1 : {rx_word_shifter[3:0], mosi_sync[1]};
                        
                        // Pulse read strobe to latch parallel data into shifter
                        if ((!rx_word_shifter[14]) || rx_word_shifter[13]) rd_strobe <= 1'b1;
                    end
                    else if (bit_cnt > 9'd15 && (bit_cnt[3:0] == 4'd15) && word_idx < WRITE_WORDS) begin
                        rx_burst_buffer[word_idx*16 +: 16] <= {rx_word_shifter[14:0], mosi_sync[1]};
                        word_idx <= word_idx + 1'b1;
                    end
                end

                // Latch parallel telemetry data into serial shifter
                if (rd_strobe) begin
                    for (w = 0; w < READ_WORDS; w = w + 1)
                        tx_shifter_burst[(READ_WORDS-w)*16 - 1 -: 16] <= din[w*16 +: 16];
                end

                // Shift MISO data out on SCK falling edge
                if (sck_falling && bit_cnt >= 9'd16 && is_read) begin
                    spi_out          <= tx_shifter_burst[(16*READ_WORDS)-1];
                    tx_shifter_burst <= {tx_shifter_burst[(16*READ_WORDS)-2 : 0], 1'b0};
                end
            end

            // --- 3. END OF TRANSACTION (Atomic Write Execution) ---
            if (nss_rising) begin
                if (is_write && (bit_cnt >= {4'd0, burst_len, 4'd0} + 9'd16)) begin
                    dout      <= rx_burst_buffer;
                    wr_strobe <= 1'b1; // Trigger atomic DAC update
                end
                spi_out <= 1'b0;
            end
        end
    end
endmodule