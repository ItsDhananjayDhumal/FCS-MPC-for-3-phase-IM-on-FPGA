`timescale 1ns/1ps
`include "mpc_params.vh"

module cost_evaluator #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS 
)(
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_WIDTH-1:0] is_alpha_pred,
    input  wire signed [DATA_WIDTH-1:0] is_beta_pred,
    input  wire signed [DATA_WIDTH-1:0] psi_r_alpha_pred,
    input  wire signed [DATA_WIDTH-1:0] psi_r_beta_pred,
    input  wire signed [DATA_WIDTH-1:0] te_ref_in,
    output reg  signed [DATA_WIDTH-1:0] cost,
    output reg  cost_overflow,
    output reg  done
);

    localparam signed [DATA_WIDTH-1:0] KT = 32'sd3055130;
    localparam signed [DATA_WIDTH-1:0] TE_REF_DEFAULT = 32'sd5242880;
    localparam signed [DATA_WIDTH-1:0] PSI_REF_SQ = 32'sd966367;
    localparam signed [DATA_WIDTH-1:0] LAMBDA_T = 32'sd1048576;
    localparam signed [DATA_WIDTH-1:0] LAMBDA_PSI = 32'sd104857600;

    reg [4:0] step;
    reg busy;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];

    reg signed [DATA_WIDTH-1:0] cross1, cross2, cross_diff, te_pred, torque_err, torque_err_sq;
    reg signed [DATA_WIDTH-1:0] psi_a_sq, psi_b_sq, psi_sq, flux_err, flux_err_sq;
    reg signed [DATA_WIDTH-1:0] cost_t, cost_f;

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
            cost <= 0;
            cost_overflow <= 0;
            mul_a <= 0; mul_b <= 0;
            cross1 <= 0; cross2 <= 0; cross_diff <= 0;
            te_pred <= 0; torque_err <= 0; torque_err_sq <= 0;
            psi_a_sq <= 0; psi_b_sq <= 0; psi_sq <= 0;
            flux_err <= 0; flux_err_sq <= 0;
            cost_t <= 0; cost_f <= 0;
        end else begin
            done <= 0; 
            
            if (!busy) begin
                if (start) begin
                    busy <= 1;
                    step <= 0;
                    
                    mul_a <= psi_r_alpha_pred;
                    mul_b <= is_beta_pred;
                end
            end else begin
                step <= step + 1;
                case (step)
                    0: begin 
                        // Wait for multiplier
                    end
                    1: begin
                        cross1 <= mul_result;
                        mul_a <= psi_r_beta_pred;
                        mul_b <= is_alpha_pred;
                    end
                    2: begin
                        // Wait for multiplier
                    end
                    3: begin
                        cross2 <= mul_result;
                        cross_diff <= cross1 - mul_result;
                    end
                    4: begin
                        mul_a <= KT;
                        mul_b <= cross_diff; 
                    end
                    5: begin
                        // Wait for multiplier
                    end
                    6: begin
                        te_pred <= mul_result;
                    end
                    7: begin
                        torque_err <= te_ref_in - te_pred;
                    end
                    8: begin
                        mul_a <= torque_err;
                        mul_b <= torque_err;
                    end
                    9: begin
                        // Wait for multiplier
                    end
                    10: begin
                        torque_err_sq <= mul_result;
                        mul_a <= psi_r_alpha_pred;
                        mul_b <= psi_r_alpha_pred;
                    end
                    11: begin
                        // Wait for multiplier
                    end
                    12: begin
                        psi_a_sq <= mul_result;
                        mul_a <= psi_r_beta_pred;
                        mul_b <= psi_r_beta_pred;
                    end
                    13: begin
                        // Wait for multiplier
                    end
                    14: begin
                        psi_b_sq <= mul_result;
                        psi_sq <= psi_a_sq + mul_result;
                    end
                    15: begin
                        flux_err <= PSI_REF_SQ - psi_sq;
                    end
                    16: begin
                        mul_a <= flux_err;
                        mul_b <= flux_err;
                    end
                    17: begin
                        // Wait for multiplier
                    end
                    18: begin
                        flux_err_sq <= mul_result;
                        mul_a <= LAMBDA_T;
                        mul_b <= torque_err_sq;
                    end
                    19: begin
                        // Wait for multiplier
                    end
                    20: begin
                        cost_t <= mul_result;
                        mul_a <= LAMBDA_PSI;
                        mul_b <= flux_err_sq;
                    end
                    21: begin
                        // Wait for multiplier
                    end
                    22: begin
                        cost_f <= mul_result;

                        if ((cost_t > 0 && mul_result > 0 && (cost_t + mul_result) < 0) || 
                            cost_t > 32'sd1048576000 || mul_result > 32'sd1048576000) begin
                            cost <= 32'sd2147483647; 
                            cost_overflow <= 1;
                        end else begin
                            cost <= cost_t + mul_result;
                            cost_overflow <= 0;
                        end
                    end
                    23: begin
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