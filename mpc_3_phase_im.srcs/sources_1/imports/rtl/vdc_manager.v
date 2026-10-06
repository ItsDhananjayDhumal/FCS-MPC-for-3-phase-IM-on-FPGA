`timescale 1ns / 1ps
`include "mpc_params.vh"

module vdc_manager #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
) (
    input  wire clk,
    input  wire rst_n,
    input  wire [7:0] sw,
    output reg  signed [DATA_WIDTH-1:0] vdc_q,
    output reg  signed [DATA_WIDTH-1:0] v1_alpha_out,
    output reg  signed [DATA_WIDTH-1:0] v2_alpha_out,
    output reg  signed [DATA_WIDTH-1:0] v2_beta_out,
    output reg  vdc_valid
);

    localparam signed [DATA_WIDTH-1:0] TWO_THIRDS = `TWO_THIRDS;
    localparam signed [DATA_WIDTH-1:0] ONE_THIRD  = `ONE_THIRD;
    localparam signed [DATA_WIDTH-1:0] INV_SQRT3  = `INV_SQRT3;
    localparam VDC_DEFAULT_INT = `VDC_DEFAULT_INT;

    reg [7:0] sw_sync_1, sw_sync_2;
    reg [7:0] sw_stable;
    reg [19:0] debounce_cnt;
    reg startup_done;
    reg sw_valid;

    wire [15:0] vdc_integer = sw_stable[7:4] * 16'd40 + sw_stable[3:0] * 16'd3;
    reg [15:0] vdc_int_reg;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];

    always @(posedge clk) begin
        mul_result_full <= mul_a * mul_b;
    end

    reg [3:0] step;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sw_sync_1 <= 0;
            sw_sync_2 <= 0;
            sw_stable <= 8'h00; 
            startup_done <= 1'b0;
            sw_valid <= 1'b0;
            debounce_cnt <= 0;
            
            vdc_int_reg <= VDC_DEFAULT_INT;
            vdc_q <= (VDC_DEFAULT_INT << FRAC_BITS);
            v1_alpha_out <= 0;
            v2_alpha_out <= 0;
            v2_beta_out <= 0;
            
            vdc_valid <= 0;
            step <= 0;
            mul_a <= 0;
            mul_b <= 0;
        end else begin
            
            sw_sync_1 <= sw;
            sw_sync_2 <= sw_sync_1;
            
            vdc_valid <= 0; 

            if (sw_sync_2 != sw_stable) begin
                if (debounce_cnt == 0) begin
                    
                    debounce_cnt <= 20'd1000000;
                end else begin
                    debounce_cnt <= debounce_cnt - 1;
                    if (debounce_cnt == 1) begin
                        sw_stable <= sw_sync_2;
                        sw_valid <= 1'b1;
                    end
                end
            end else begin
                debounce_cnt <= 0;
            end

            case (step)
                0: begin
                    if (!startup_done) begin
                        // First pass: use default Vdc
                        vdc_int_reg <= VDC_DEFAULT_INT;
                        vdc_q <= (VDC_DEFAULT_INT << FRAC_BITS);
                        startup_done <= 1'b1;
                        step <= 1;
                    end else if (sw_valid && (vdc_integer != vdc_int_reg)) begin
                        // Only update after debounce has completed at least once
                        vdc_int_reg <= vdc_integer;
                        vdc_q <= (vdc_integer << FRAC_BITS);
                        step <= 1;
                    end
                end
                1: begin
                    
                    mul_a <= vdc_q;
                    mul_b <= TWO_THIRDS;
                    step <= 2;
                end
                2: begin
                    
                    step <= 3;
                end
                3: begin
                    v1_alpha_out <= mul_result;
                    
                    mul_a <= vdc_q;
                    mul_b <= ONE_THIRD;
                    step <= 4;
                end
                4: begin
                    
                    step <= 5;
                end
                5: begin
                    v2_alpha_out <= mul_result;
                    
                    mul_a <= vdc_q;
                    mul_b <= INV_SQRT3;
                    step <= 6;
                end
                6: begin
                    
                    step <= 7;
                end
                7: begin
                    v2_beta_out <= mul_result;
                    vdc_valid <= 1'b1; 
                    step <= 0;
                end
                default: step <= 0;
            endcase
        end
    end
endmodule
