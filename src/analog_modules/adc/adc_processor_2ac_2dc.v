`timescale 1ns / 1ps
// ==============================================================================
// MODULE: adc_processor_2ac_2dc (DYNAMIC ROUTING & SHIFT)
// ==============================================================================

module adc_processor_2ac_2dc #(
    parameter RANGE1_CFG       = 1'b0,
    parameter RANGE0_CFG       = 1'b0,
    parameter integer SAMPLE_TICKS = 1000
)(
    // --- System Clock and Reset ---
    input  wire         clk,
    input  wire         rst_n,

    // --- Dynamic Control ---
    input  wire         ac_sel,       
    input  wire         comp_in,      
    input  wire [4:0]   shift_sel,    

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
    output reg  [31:0]  acc_ac_1, 
    output reg  [31:0]  acc_ac_2, 
    output reg  [15:0]  cnt_ac,
    output reg          ready_ac,

    output reg  [15:0]  acc_dc_1, 
    output reg  [15:0]  acc_dc_2, 
    output reg          ready_dc
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

    wire [15:0] v_ac_1 = (ac_sel == 1'b0) ? raw_a1 : raw_a2;
    wire [15:0] v_ac_2 = (ac_sel == 1'b0) ? raw_b1 : raw_b2;
    
    wire [15:0] v_dc_1 = (ac_sel == 1'b0) ? raw_a2 : raw_a1;
    wire [15:0] v_dc_2 = (ac_sel == 1'b0) ? raw_b2 : raw_b1;

    wire [31:0] ext_ac_1 = {{16{v_ac_1[15]}}, v_ac_1};
    wire [31:0] ext_ac_2 = {{16{v_ac_2[15]}}, v_ac_2};
    wire [31:0] ext_dc_1 = {{16{v_dc_1[15]}}, v_dc_1};
    wire [31:0] ext_dc_2 = {{16{v_dc_2[15]}}, v_dc_2};

    reg comp_d;
    always @(posedge clk) begin
        if (!rst_n) comp_d <= 1'b0;
        else        comp_d <= comp_in;
    end
    
    wire comp_edge = comp_in && !comp_d; 

    reg [31:0] acc_ac1_int, acc_ac2_int;
    reg [15:0] cnt_ac_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            acc_ac1_int <= 32'd0; acc_ac2_int <= 32'd0; cnt_ac_int <= 16'd0;
            acc_ac_1    <= 32'd0; acc_ac_2    <= 32'd0; cnt_ac     <= 16'd0;
            ready_ac    <= 1'b0;
        end else begin
            ready_ac <= 1'b0; 
            
            if (comp_edge) begin
                acc_ac_1 <= acc_ac1_int; 
                acc_ac_2 <= acc_ac2_int; 
                cnt_ac   <= cnt_ac_int;
                ready_ac <= 1'b1;  
                
                acc_ac1_int <= 32'd0; 
                acc_ac2_int <= 32'd0; 
                cnt_ac_int  <= 16'd0;
            end else if (ready_pulse) begin
                if (comp_in) begin
                    acc_ac1_int <= acc_ac1_int + ext_ac_1; 
                    acc_ac2_int <= acc_ac2_int + ext_ac_2;
                end else begin
                    acc_ac1_int <= acc_ac1_int - ext_ac_1; 
                    acc_ac2_int <= acc_ac2_int - ext_ac_2;
                end
                cnt_ac_int <= cnt_ac_int + 1'b1;
            end
        end
    end

    reg [15:0] decimation_cnt; 
    reg [31:0] acc_dc1_int, acc_dc2_int;

    wire [31:0] final_dc_1 = acc_dc1_int + ext_dc_1;
    wire [31:0] final_dc_2 = acc_dc2_int + ext_dc_2;

    always @(posedge clk) begin
        if (!rst_n) begin
            decimation_cnt <= 16'd0;
            acc_dc1_int <= 32'd0; acc_dc2_int <= 32'd0;
            acc_dc_1    <= 16'd0; acc_dc_2    <= 16'd0; 
            ready_dc    <= 1'b0;
        end else begin
            ready_dc <= 1'b0; 
            
            if (ready_pulse) begin
                if (decimation_cnt >= decimation_max) begin
                    acc_dc_1 <= final_dc_1[safe_shift +: 16]; 
                    acc_dc_2 <= final_dc_2[safe_shift +: 16];
                    ready_dc <= 1'b1; 
                    
                    acc_dc1_int    <= 32'd0; 
                    acc_dc2_int    <= 32'd0; 
                    decimation_cnt <= 16'd0;
                end else begin
                    acc_dc1_int    <= acc_dc1_int + ext_dc_1; 
                    acc_dc2_int    <= acc_dc2_int + ext_dc_2;
                    decimation_cnt <= decimation_cnt + 1'b1;
                end
            end
        end
    end

endmodule