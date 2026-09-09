`timescale 1ns / 1ps
`include "mpc_params.vh"

module clarke_transform #(
    parameter ADC_BITS = 12,
    parameter DATA_WIDTH = 32,
    parameter FRAC_BITS = 20
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire [ADC_BITS-1:0] ia_raw,
    input  wire [ADC_BITS-1:0] ib_raw,
    output reg  signed [DATA_WIDTH-1:0] i_alpha,
    output reg  signed [DATA_WIDTH-1:0] i_beta,
    output reg  done
);

    
    
    localparam signed [DATA_WIDTH-1:0] INV_SQRT3 = 32'sd605510; 

    `ifndef ADC_OFFSET
        `define ADC_OFFSET 2048
    `endif
    `ifndef ADC_SCALE
        `define ADC_SCALE 32'sd10485 
    `endif
    
    reg [3:0] step;

    reg signed [DATA_WIDTH-1:0] mul_a;
    reg signed [DATA_WIDTH-1:0] mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
    
    always @(posedge clk) begin
        mul_result_full <= mul_a * mul_b;
    end
    
    reg signed [DATA_WIDTH-1:0] ia_signed, ib_signed;
    reg signed [DATA_WIDTH-1:0] ia, ib;
    reg signed [DATA_WIDTH-1:0] temp;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step <= 0;
            i_alpha <= 0;
            i_beta <= 0;
            done <= 0;
            mul_a <= 0;
            mul_b <= 0;
            ia_signed <= 0;
            ib_signed <= 0;
            ia <= 0;
            ib <= 0;
            temp <= 0;
        end else begin
            done <= 0; 
            
            case (step)
                0: begin
                    if (start) begin
                        
                        ia_signed <= $signed({1'b0, ia_raw}) - $signed(ADC_OFFSET);
                        ib_signed <= $signed({1'b0, ib_raw}) - $signed(ADC_OFFSET);
                        step <= 1;
                    end
                end
                1: begin
                    
                    mul_a <= ia_signed;
                    mul_b <= ADC_SCALE;
                    step <= 2;
                end
                2: begin
                    
                    step <= 3;
                end
                3: begin
                    ia <= mul_result; 
                    
                    mul_a <= ib_signed;
                    mul_b <= ADC_SCALE;
                    step <= 4;
                end
                4: begin
                    
                    step <= 5;
                end
                5: begin
                    ib <= mul_result; 
                    
                    i_alpha <= ia;
                    step <= 6;
                end
                6: begin
                    
                    temp <= ia + (ib <<< 1);
                    step <= 7;
                end
                7: begin
                    
                    mul_a <= temp;
                    mul_b <= INV_SQRT3;
                    step <= 8;
                end
                8: begin
                    
                    step <= 9;
                end
                9: begin
                    i_beta <= mul_result; 
                    done <= 1'b1;         
                    step <= 0;
                end
                default: step <= 0;
            endcase
        end
    end
endmodule
