`timescale 1ns / 1ps

module ad5543_dac_ctrl #(
    // Clock divider parameter.
    // SPI Clock Period = System Clock Period * (CLK_DIV * 2)
    // Example: 100MHz sys_clk, CLK_DIV=5 -> 10MHz SPI Clock
    parameter CLK_DIV = 5 
)(
    input  wire        clk,          // System clock
    input  wire        rst,          // Synchronous reset (active high)
    input  wire        start,        // Pulse '1' to start data transmission
    input  wire [15:0] data_in,      // 16-bit data to send to the DAC
    input  wire        bipolar_mode, // 1 = 2's complement (flip MSB), 0 = straight binary

    output reg         dac_cs_n,     // Chip Select (active low)
    output reg         dac_clk,      // Serial Clock
    output wire        dac_sdi,      // Serial Data Input for the DAC
    output reg         ready         // Ready flag (1 = ready to accept new data)
);

    // --- Manual function for bit width calculation (ISE 14.7 fix) ---
    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    // Max counter value is in CS_WAIT state: (CLK_DIV * 2) - 1
    localparam integer MAX_DELAY  = (CLK_DIV * 2) - 1;
    localparam integer DELAY_BITS = (f_clog2(MAX_DELAY) > 0) ? f_clog2(MAX_DELAY) : 1;

    // FSM States
    localparam [2:0] 
        IDLE    = 3'd0,
        START   = 3'd1,
        HIGH    = 3'd2,
        LOW     = 3'd3,
        DONE    = 3'd4,
        CS_WAIT = 3'd5; // Guarantees tCSW (CS High Time)

    reg [2:0]            state;
    reg [15:0]           shift_reg;
    reg [3:0]            bit_cnt;   // 0 to 15 requires only 4 bits
    reg [DELAY_BITS-1:0] delay_cnt; 

    // SDI is always the MSB of the shift register
    assign dac_sdi = shift_reg[15];

    always @(posedge clk) begin
        if (rst) begin
            state     <= IDLE;
            dac_cs_n  <= 1'b1;
            dac_clk   <= 1'b0;
            ready     <= 1'b1;
            shift_reg <= 16'd0;
            bit_cnt   <= 4'd0;
            delay_cnt <= {DELAY_BITS{1'b0}};
        end else begin
            case (state)
                IDLE: begin
                    dac_cs_n  <= 1'b1;
                    dac_clk   <= 1'b0;
                    delay_cnt <= {DELAY_BITS{1'b0}};
                    
                    if (start) begin
                        // BIPOLAR MODE LOGIC: Convert 2's complement to Offset Binary
                        if (bipolar_mode)
                            shift_reg <= {~data_in[15], data_in[14:0]};
                        else
                            shift_reg <= data_in;
                            
                        bit_cnt   <= 4'd15;   
                        ready     <= 1'b0;    
                        state     <= START;
                    end else begin
                        ready     <= 1'b1;
                    end
                end

                START: begin
                    dac_cs_n <= 1'b0; // Assert CS
                    
                    if (delay_cnt == CLK_DIV - 1) begin
                        delay_cnt <= {DELAY_BITS{1'b0}};
                        state     <= HIGH;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                HIGH: begin
                    dac_clk <= 1'b1; // Rising edge (DAC samples data here)
                    
                    if (delay_cnt == CLK_DIV - 1) begin
                        delay_cnt <= {DELAY_BITS{1'b0}};
                        state     <= LOW;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                LOW: begin
                    dac_clk <= 1'b0; // Falling edge (Safe to change data)
                    
                    if (delay_cnt == CLK_DIV - 1) begin
                        delay_cnt <= {DELAY_BITS{1'b0}};
                        
                        if (bit_cnt == 4'd0) begin
                            state <= DONE; 
                        end else begin
                            bit_cnt   <= bit_cnt - 1'b1;
                            shift_reg <= {shift_reg[14:0], 1'b0}; // Shift data
                            state     <= HIGH;
                        end
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                DONE: begin
                    dac_cs_n <= 1'b1; // Deassert CS
                    state    <= CS_WAIT;
                end
                
                // Guarantee minimum CS High time (tCSW) before next transaction
                CS_WAIT: begin
                    if (delay_cnt == MAX_DELAY) begin
                        delay_cnt <= {DELAY_BITS{1'b0}};
                        state     <= IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule