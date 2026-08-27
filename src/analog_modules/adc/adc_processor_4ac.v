`timescale 1ns / 1ps
// ==============================================================================
// MODULE: adc_processor_4ac
// DESCRIPTION: Synchronous demodulator (Lock-in Amplifier) for 4 AC channels.
//              Accumulates data over a full 360-degree carrier cycle.
//              Optimized with 'f_clog2' to eliminate Xst:1710 trimming warnings.
// ==============================================================================

module adc_processor_4ac #(
    parameter RANGE1_CFG   = 1'b0,
    parameter RANGE0_CFG   = 1'b0,
    parameter SAMPLE_TICKS = 1000
)(
    // --- System Clock and Reset ---
    input  wire         clk,
    input  wire         rst_n,

    // --- Common Reference Input ---
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
    output reg  [31:0]  acc_ch1, 
    output reg  [31:0]  acc_ch2, 
    output reg  [31:0]  acc_ch3, 
    output reg  [31:0]  acc_ch4,
    output reg  [15:0]  cnt_samples,
    output reg          ready_ac
);

    // =========================================================
    // 1. Bit-width calculation function (Eliminates Xst:1710)
    // =========================================================
    function integer f_clog2;
        input integer value;
        begin
            for (f_clog2 = 0; value > 0; f_clog2 = f_clog2 + 1)
                value = value >> 1;
        end
    endfunction

    localparam integer TIMER_WIDTH = (f_clog2(SAMPLE_TICKS - 1) > 0) ? f_clog2(SAMPLE_TICKS - 1) : 1;

    // =========================================================
    // 2. Sampling Timer (Exact 10-bit counter)
    // =========================================================
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

    // =========================================================
    // 3. ADC Controller Instance
    // =========================================================
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

    // =========================================================
    // 4. Signal Routing & Extension
    // =========================================================
    wire [31:0] ext_ch1 = {{16{raw_a1[15]}}, raw_a1};
    wire [31:0] ext_ch2 = {{16{raw_b1[15]}}, raw_b1};
    wire [31:0] ext_ch3 = {{16{raw_a2[15]}}, raw_a2};
    wire [31:0] ext_ch4 = {{16{raw_b2[15]}}, raw_b2};

    // Synchronous Edge Detector
    reg comp_d;
    always @(posedge clk) begin
        if (!rst_n) comp_d <= 1'b0;
        else        comp_d <= comp_in;
    end
    
    wire comp_edge = comp_in && !comp_d; 

    // =========================================================
    // 5. Engine: Quad Synchronous Demodulator
    // =========================================================
    reg [31:0] int_ch1, int_ch2, int_ch3, int_ch4;
    reg [15:0] cnt_int;

    always @(posedge clk) begin
        if (!rst_n) begin
            int_ch1 <= 32'd0; int_ch2 <= 32'd0; 
            int_ch3 <= 32'd0; int_ch4 <= 32'd0;
            cnt_int <= 16'd0;
            
            acc_ch1 <= 32'd0; acc_ch2 <= 32'd0; 
            acc_ch3 <= 32'd0; acc_ch4 <= 32'd0;
            cnt_samples <= 16'd0;
            ready_ac <= 1'b0;
        end else begin
            ready_ac <= 1'b0; 
            
            if (comp_edge) begin
                acc_ch1 <= int_ch1; acc_ch2 <= int_ch2;
                acc_ch3 <= int_ch3; acc_ch4 <= int_ch4;
                cnt_samples <= cnt_int;
                ready_ac <= 1'b1;  
                
                int_ch1 <= 32'd0; int_ch2 <= 32'd0; 
                int_ch3 <= 32'd0; int_ch4 <= 32'd0;
                cnt_int <= 16'd0;
            end else if (ready_pulse) begin
                if (comp_in) begin
                    int_ch1 <= int_ch1 + ext_ch1;
                    int_ch2 <= int_ch2 + ext_ch2;
                    int_ch3 <= int_ch3 + ext_ch3;
                    int_ch4 <= int_ch4 + ext_ch4;
                end else begin
                    int_ch1 <= int_ch1 - ext_ch1;
                    int_ch2 <= int_ch2 - ext_ch2;
                    int_ch3 <= int_ch3 - ext_ch3;
                    int_ch4 <= int_ch4 - ext_ch4;
                end
                cnt_int <= cnt_int + 1'b1;
            end
        end
    end

endmodule