`timescale 1ns / 1ps
// ==============================================================================
// MODULE: adc_processor_resolver (PARAMETERIZED & DYNAMIC ROUTING)
// ==============================================================================

module adc_processor_resolver #(
    parameter RANGE1_CFG       = 1'b0,
    parameter RANGE0_CFG       = 1'b0,
    parameter integer SAMPLE_TICKS = 1000
)(
    // --- System Clock and Reset ---
    input  wire         clk,          
    input  wire         rst_n,        
    
    // --- Dynamic Routing Matrix Control ---
    input  wire [7:0]   sel,

    // --- Common Reference Input (400 Hz) ---
    input  wire         comp_in,      
    
    // --- ADC Physical Interface ---
    output wire         cnvst_n, 
    output wire         cs_n, 
    output wire         sclk, 
    output wire         addr, 
    output wire         range0,
    output wire         range1,
    input  wire         busy, 
    input  wire         dout_a, 
    input  wire         dout_b,
    
    // --- Interface for STM32 ---
    output reg  [31:0]  acc_sin_roll, 
    output reg  [31:0]  acc_cos_roll,
    output reg  [31:0]  acc_sin_pitch, 
    output reg  [31:0]  acc_cos_pitch,
    output reg  [15:0]  cnt_samples,
    output reg          ready_res
);

    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    localparam integer TIMER_WIDTH = (f_clog2(SAMPLE_TICKS - 1) > 0) ? f_clog2(SAMPLE_TICKS - 1) : 1;

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

    wire [15:0] v_sin_roll  = (sel[1:0] == 2'b00) ? raw_a1 :
                              (sel[1:0] == 2'b01) ? raw_b1 :
                              (sel[1:0] == 2'b10) ? raw_a2 : raw_b2;

    wire [15:0] v_sin_pitch = (sel[3:2] == 2'b00) ? raw_a1 :
                              (sel[3:2] == 2'b01) ? raw_b1 :
                              (sel[3:2] == 2'b10) ? raw_a2 : raw_b2;

    wire [15:0] v_cos_roll  = (sel[5:4] == 2'b00) ? raw_a1 :
                              (sel[5:4] == 2'b01) ? raw_b1 :
                              (sel[5:4] == 2'b10) ? raw_a2 : raw_b2;

    wire [15:0] v_cos_pitch = (sel[7:6] == 2'b00) ? raw_a1 :
                              (sel[7:6] == 2'b01) ? raw_b1 :
                              (sel[7:6] == 2'b10) ? raw_a2 : raw_b2;

    wire [31:0] ext_sin_roll  = {{16{v_sin_roll[15]}},  v_sin_roll};
    wire [31:0] ext_sin_pitch = {{16{v_sin_pitch[15]}}, v_sin_pitch};
    wire [31:0] ext_cos_roll  = {{16{v_cos_roll[15]}},  v_cos_roll};
    wire [31:0] ext_cos_pitch = {{16{v_cos_pitch[15]}}, v_cos_pitch};

    reg comp_d;
    always @(posedge clk) begin
        if (!rst_n) comp_d <= 1'b0;
        else        comp_d <= comp_in;
    end
    
    wire comp_edge = comp_in && !comp_d; 

    reg [31:0] int_sin_roll, int_sin_pitch;
    reg [31:0] int_cos_roll, int_cos_pitch;
    reg [15:0] cnt_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            int_sin_roll  <= 32'd0; int_sin_pitch <= 32'd0; 
            int_cos_roll  <= 32'd0; int_cos_pitch <= 32'd0;
            cnt_int       <= 16'd0;
            
            acc_sin_roll  <= 32'd0; acc_sin_pitch <= 32'd0; 
            acc_cos_roll  <= 32'd0; acc_cos_pitch <= 32'd0;
            cnt_samples   <= 16'd0;
            ready_res     <= 1'b0;
        end else begin
            ready_res <= 1'b0; 
            
            if (comp_edge) begin
                acc_sin_roll  <= int_sin_roll;  
                acc_sin_pitch <= int_sin_pitch;
                acc_cos_roll  <= int_cos_roll;  
                acc_cos_pitch <= int_cos_pitch;
                cnt_samples   <= cnt_int;
                ready_res     <= 1'b1;  
                
                int_sin_roll  <= 32'd0; 
                int_sin_pitch <= 32'd0; 
                int_cos_roll  <= 32'd0; 
                int_cos_pitch <= 32'd0;
                cnt_int       <= 16'd0;
            end else if (ready_pulse) begin
                if (comp_in) begin
                    int_sin_roll  <= int_sin_roll  + ext_sin_roll;
                    int_sin_pitch <= int_sin_pitch + ext_sin_pitch;
                    int_cos_roll  <= int_cos_roll  + ext_cos_roll;
                    int_cos_pitch <= int_cos_pitch + ext_cos_pitch;
                end else begin
                    int_sin_roll  <= int_sin_roll  - ext_sin_roll;
                    int_sin_pitch <= int_sin_pitch - ext_sin_pitch;
                    int_cos_roll  <= int_cos_roll  - ext_cos_roll;
                    int_cos_pitch <= int_cos_pitch - ext_cos_pitch;
                end
                cnt_int <= cnt_int + 1'b1;
            end
        end
    end
endmodule