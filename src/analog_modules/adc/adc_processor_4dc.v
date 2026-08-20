`timescale 1ns / 1ps
// ==============================================================================
// MODULE: adc_processor_4dc (DYNAMIC & FOOLPROOF)
// ==============================================================================

module adc_processor_4dc #(
    parameter RANGE1_CFG       = 1'b0,
    parameter RANGE0_CFG       = 1'b0,
    parameter integer SAMPLE_TICKS = 1000
)(
    input  wire        clk,          
    input  wire        rst_n,        
    input  wire [4:0]  shift_sel,    

    // ADC Physical Interface
    output wire        cnvst_n, cs_n, sclk, addr, range0, range1,
    input  wire        busy, dout_a, dout_b,

    // Interface for STM32
    output reg [15:0]  acc_ch1, 
    output reg [15:0]  acc_ch2, 
    output reg [15:0]  acc_ch3, 
    output reg [15:0]  acc_ch4,
    output reg         ready_dc      
);

    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    localparam integer TIMER_WIDTH = (f_clog2(SAMPLE_TICKS - 1) > 0) ? f_clog2(SAMPLE_TICKS - 1) : 1;

    wire [4:0]  safe_shift     = (shift_sel > 5'd16) ? 5'd16 : shift_sel;
    wire [15:0] decimation_max = (16'd1 << safe_shift) - 16'd1;

    reg [TIMER_WIDTH-1:0] sampling_timer; 
    reg                   start_pulse;

    always @(posedge clk) begin
        if (!rst_n) begin
            sampling_timer <= {TIMER_WIDTH{1'b0}};
            start_pulse    <= 1'b0;
        end else begin
            if (sampling_timer >= (SAMPLE_TICKS - 1)) begin
                sampling_timer <= {TIMER_WIDTH{1'b0}};
                start_pulse    <= 1'b1;
            end else begin
                sampling_timer <= sampling_timer + 1'b1;
                start_pulse    <= 1'b0;
            end
        end
    end

    wire [15:0] raw_a1, raw_b1, raw_a2, raw_b2;
    wire        ready_pulse;

    ad7367_multi_mode u_adc (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start_pulse),
        .mode        (2'b00),
        .cfg_range0  (RANGE0_CFG),
        .cfg_range1  (RANGE1_CFG),
        .cnvst_n     (cnvst_n),
        .cs_n        (cs_n),
        .sclk        (sclk),
        .addr        (addr),
        .range0      (range0),
        .range1      (range1),
        .busy        (busy),        
        .dout_a      (dout_a),
        .dout_b      (dout_b),
        .va1         (raw_a1),
        .vb1         (raw_b1),
        .va2         (raw_a2),
        .vb2         (raw_b2),
        .ready       (ready_pulse),
        .module_busy (),
        .err_timeout ()
    );

    wire [31:0] ext_ch1 = {{16{raw_a1[15]}}, raw_a1};
    wire [31:0] ext_ch2 = {{16{raw_b1[15]}}, raw_b1};
    wire [31:0] ext_ch3 = {{16{raw_a2[15]}}, raw_a2};
    wire [31:0] ext_ch4 = {{16{raw_b2[15]}}, raw_b2};

    reg [15:0] decimation_cnt; 
    reg [31:0] int_ch1, int_ch2, int_ch3, int_ch4;

    wire [31:0] final_ch1 = int_ch1 + ext_ch1;
    wire [31:0] final_ch2 = int_ch2 + ext_ch2;
    wire [31:0] final_ch3 = int_ch3 + ext_ch3;
    wire [31:0] final_ch4 = int_ch4 + ext_ch4;

    always @(posedge clk) begin
        if (!rst_n) begin
            decimation_cnt <= 16'd0;
            int_ch1  <= 32'd0; int_ch2  <= 32'd0; int_ch3  <= 32'd0; int_ch4  <= 32'd0;
            acc_ch1  <= 16'd0; acc_ch2  <= 16'd0; acc_ch3  <= 16'd0; acc_ch4  <= 16'd0;
            ready_dc <= 1'b0;
        end else begin
            ready_dc <= 1'b0; 
            
            if (ready_pulse) begin
                if (decimation_cnt >= decimation_max) begin
                    acc_ch1  <= final_ch1[safe_shift +: 16];
                    acc_ch2  <= final_ch2[safe_shift +: 16];
                    acc_ch3  <= final_ch3[safe_shift +: 16];
                    acc_ch4  <= final_ch4[safe_shift +: 16];
                    ready_dc <= 1'b1; 
                    
                    int_ch1  <= 32'd0; int_ch2  <= 32'd0; 
                    int_ch3  <= 32'd0; int_ch4  <= 32'd0;
                    decimation_cnt <= 16'd0;
                end else begin
                    int_ch1  <= int_ch1 + ext_ch1;
                    int_ch2  <= int_ch2 + ext_ch2;
                    int_ch3  <= int_ch3 + ext_ch3;
                    int_ch4  <= int_ch4 + ext_ch4;
                    decimation_cnt <= decimation_cnt + 1'b1;
                end
            end
        end
    end

endmodule