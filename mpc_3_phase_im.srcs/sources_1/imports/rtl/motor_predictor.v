`timescale 1ns/1ps
`include "mpc_params.vh"

module motor_predictor #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_WIDTH-1:0] i_alpha,
    input  wire signed [DATA_WIDTH-1:0] i_beta,
    input  wire signed [DATA_WIDTH-1:0] psi_r_alpha,
    input  wire signed [DATA_WIDTH-1:0] psi_r_beta,
    input  wire signed [DATA_WIDTH-1:0] wr_psi_alpha,
    input  wire signed [DATA_WIDTH-1:0] wr_psi_beta,
    input  wire signed [DATA_WIDTH-1:0] vs_alpha,
    input  wire signed [DATA_WIDTH-1:0] vs_beta,
    output reg  signed [DATA_WIDTH-1:0] is_alpha_pred,
    output reg  signed [DATA_WIDTH-1:0] is_beta_pred,
    output reg  signed [DATA_WIDTH-1:0] psi_r_alpha_pred,
    output reg  signed [DATA_WIDTH-1:0] psi_r_beta_pred,
    output reg  done
);

    localparam signed [DATA_WIDTH-1:0] C11  = `C11;
    localparam signed [DATA_WIDTH-1:0] C12  = `C12;
    localparam signed [DATA_WIDTH-1:0] C13  = `C13;
    localparam signed [DATA_WIDTH-1:0] D1   = `D1;
    localparam signed [DATA_WIDTH-1:0] E21  = `E21;
    localparam signed [DATA_WIDTH-1:0] E22  = `E22;
    localparam signed [DATA_WIDTH-1:0] TS_Q = `TS_Q;

    reg [4:0] step;
    reg busy;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];

    reg signed [DATA_WIDTH-1:0] t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mul_result_full <= 0;
        end else begin
            mul_result_full <= mul_a * mul_b;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step <= 0;
            busy <= 0;
            done <= 0;
            mul_a <= 0;
            mul_b <= 0;
            is_alpha_pred <= 0;
            is_beta_pred <= 0;
            psi_r_alpha_pred <= 0;
            psi_r_beta_pred <= 0;
            t1 <= 0; t2 <= 0; t3 <= 0; t4 <= 0;
            t5 <= 0; t6 <= 0; t7 <= 0; t8 <= 0;
            t9 <= 0; t10 <= 0; t11 <= 0; t12 <= 0;
            t13 <= 0; t14 <= 0;
        end else begin
            done <= 0; 
            
            if (!busy) begin
                if (start) begin
                    busy <= 1;
                    step <= 0;
                    
                    mul_a <= C11;
                    mul_b <= i_alpha;
                end
            end else begin
                step <= step + 1;
                case (step)
                    0: begin
                        
                    end
                    1: begin
                        t1 <= mul_result;
                        mul_a <= C12;
                        mul_b <= psi_r_alpha;
                    end
                    2: begin
                        
                    end
                    3: begin
                        t2 <= mul_result;
                        mul_a <= C13;
                        mul_b <= wr_psi_beta;
                    end
                    4: begin
                        
                    end
                    5: begin
                        t3 <= mul_result;
                        mul_a <= D1;
                        mul_b <= vs_alpha;
                    end
                    6: begin
                        
                    end
                    7: begin
                        t4 <= mul_result;
                        is_alpha_pred <= t1 + t2 + t3 + mul_result;
                        
                        mul_a <= C11;
                        mul_b <= i_beta;
                    end
                    8: begin
                        
                    end
                    9: begin
                        t5 <= mul_result;
                        mul_a <= C13;
                        mul_b <= wr_psi_alpha;
                    end
                    10: begin
                        
                    end
                    11: begin
                        t6 <= mul_result;
                        mul_a <= C12;
                        mul_b <= psi_r_beta;
                    end
                    12: begin
                        
                    end
                    13: begin
                        t7 <= mul_result;
                        mul_a <= D1;
                        mul_b <= vs_beta;
                    end
                    14: begin
                        
                    end
                    15: begin
                        t8 <= mul_result;
                        is_beta_pred <= t5 - t6 + t7 + mul_result;
                        
                        mul_a <= E21;
                        mul_b <= i_alpha;
                    end
                    16: begin
                        
                    end
                    17: begin
                        t9 <= mul_result;
                        mul_a <= E22;
                        mul_b <= psi_r_alpha;
                    end
                    18: begin
                        
                    end
                    19: begin
                        t10 <= mul_result;
                        mul_a <= TS_Q;
                        mul_b <= wr_psi_beta;
                    end
                    20: begin
                        
                    end
                    21: begin
                        t11 <= mul_result;
                        psi_r_alpha_pred <= t9 + t10 - mul_result;
                        
                        mul_a <= E21;
                        mul_b <= i_beta;
                    end
                    22: begin
                        
                    end
                    23: begin
                        t12 <= mul_result;
                        mul_a <= TS_Q;
                        mul_b <= wr_psi_alpha;
                    end
                    24: begin
                        
                    end
                    25: begin
                        t13 <= mul_result;
                        mul_a <= E22;
                        mul_b <= psi_r_beta;
                    end
                    26: begin
                        
                    end
                    27: begin
                        t14 <= mul_result;
                        psi_r_beta_pred <= t12 + t13 + mul_result;
                    end
                    28: begin
                        done <= 1;
                        busy <= 0;
                    end
                    default: begin
                        busy <= 0; 
                    end
                endcase
            end
        end
    end
endmodule
