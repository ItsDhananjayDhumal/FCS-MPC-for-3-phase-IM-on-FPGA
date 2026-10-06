`timescale 1ns/1ps
`include "mpc_params.vh"

// =============================================================================
// FCS-MPC Cost Function Evaluator for 3-Phase Induction Motor Drive
//
// Cost function definition:
//   cost = lambda_t * (te_ref - te_pred)^2 + lambda_psi * (psi_ref_sq - psi_pred_sq)^2
//
// Key Design Considerations:
// 1. Squared flux comparison: avoids square root / CORDIC, saving logic & latency.
// 2. Weighting ratio: lambda_psi is 100x lambda_t to balance torque (Nm) and flux (Wb^2) scales.
// 3. Multiplier time-multiplexing: single 32-bit DSP multiplier shared across 25 steps.
// 4. Comprehensive End-to-End Overflow Protection:
//    - Every multiplication checked via upper sign-extension bits [63:52] vs bit 51.
//    - Every addition and subtraction checked via 33-bit sign comparison.
//    - Saturating arithmetic prevents 32-bit two's complement wrap-around at all steps.
//    - Any overflow sets cost_overflow flag and clamps total cost to INT32_MAX.
// =============================================================================

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

    localparam signed [DATA_WIDTH-1:0] KT             = `KT;
    localparam signed [DATA_WIDTH-1:0] TE_REF_DEFAULT = `TE_REF_DEFAULT;
    localparam signed [DATA_WIDTH-1:0] PSI_REF_SQ     = `PSI_REF_SQ;
    localparam signed [DATA_WIDTH-1:0] LAMBDA_T       = `LAMBDA_T;
    localparam signed [DATA_WIDTH-1:0] LAMBDA_PSI     = `LAMBDA_PSI;

    localparam signed [DATA_WIDTH-1:0] INT_MAX = {1'b0, {(DATA_WIDTH-1){1'b1}}}; // +2,147,483,647
    localparam signed [DATA_WIDTH-1:0] INT_MIN = {1'b1, {(DATA_WIDTH-1){1'b0}}}; // -2,147,483,648

    reg [4:0] step;
    reg busy;
    reg overflow_latched;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];

    // Multiplier overflow detection and symmetric saturation clamping
    wire mul_overflow = (mul_result_full[2*DATA_WIDTH-1:DATA_WIDTH+FRAC_BITS] != 
                        {(DATA_WIDTH-FRAC_BITS){mul_result_full[DATA_WIDTH+FRAC_BITS-1]}});
    wire signed [DATA_WIDTH-1:0] mul_sat_val = mul_result_full[2*DATA_WIDTH-1] ? INT_MIN : INT_MAX;
    wire signed [DATA_WIDTH-1:0] mul_saturated = mul_overflow ? mul_sat_val : mul_result;

    // Saturating signed addition function (33-bit sign-extension)
    function signed [DATA_WIDTH-1:0] sat_add;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
        reg signed [DATA_WIDTH:0] sum;
        begin
            sum = {a[DATA_WIDTH-1], a} + {b[DATA_WIDTH-1], b};
            if (sum[DATA_WIDTH] != sum[DATA_WIDTH-1]) begin
                sat_add = sum[DATA_WIDTH] ? INT_MIN : INT_MAX;
            end else begin
                sat_add = sum[DATA_WIDTH-1:0];
            end
        end
    endfunction

    // Saturating signed subtraction function (33-bit sign-extension)
    function signed [DATA_WIDTH-1:0] sat_sub;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
        reg signed [DATA_WIDTH:0] diff;
        begin
            diff = {a[DATA_WIDTH-1], a} - {b[DATA_WIDTH-1], b};
            if (diff[DATA_WIDTH] != diff[DATA_WIDTH-1]) begin
                sat_sub = diff[DATA_WIDTH] ? INT_MIN : INT_MAX;
            end else begin
                sat_sub = diff[DATA_WIDTH-1:0];
            end
        end
    endfunction

    // Helper functions to check if add/sub overflowed
    function is_add_ovf;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
        reg signed [DATA_WIDTH:0] sum;
        begin
            sum = {a[DATA_WIDTH-1], a} + {b[DATA_WIDTH-1], b};
            is_add_ovf = (sum[DATA_WIDTH] != sum[DATA_WIDTH-1]);
        end
    endfunction

    function is_sub_ovf;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
        reg signed [DATA_WIDTH:0] diff;
        begin
            diff = {a[DATA_WIDTH-1], a} - {b[DATA_WIDTH-1], b};
            is_sub_ovf = (diff[DATA_WIDTH] != diff[DATA_WIDTH-1]);
        end
    endfunction

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
            overflow_latched <= 0;
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
                    overflow_latched <= 0;
                    
                    mul_a <= psi_r_alpha_pred;
                    mul_b <= is_beta_pred;
                end
            end else begin
                step <= step + 1;
                case (step)
                    0: begin 
                        // Wait for multiplier pipeline
                    end
                    1: begin
                        cross1 <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                        mul_a <= psi_r_beta_pred;
                        mul_b <= is_alpha_pred;
                    end
                    2: begin
                        // Wait for multiplier pipeline
                    end
                    3: begin
                        cross2 <= mul_saturated;
                        cross_diff <= sat_sub(cross1, mul_saturated);
                        if (mul_overflow || is_sub_ovf(cross1, mul_saturated))
                            overflow_latched <= 1'b1;
                    end
                    4: begin
                        mul_a <= KT;
                        mul_b <= cross_diff; 
                    end
                    5: begin
                        // Wait for multiplier pipeline
                    end
                    6: begin
                        te_pred <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                    end
                    7: begin
                        torque_err <= sat_sub(te_ref_in, te_pred);
                        if (is_sub_ovf(te_ref_in, te_pred))
                            overflow_latched <= 1'b1;
                    end
                    8: begin
                        mul_a <= torque_err;
                        mul_b <= torque_err;
                    end
                    9: begin
                        // Wait for multiplier pipeline
                    end
                    10: begin
                        torque_err_sq <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                        mul_a <= psi_r_alpha_pred;
                        mul_b <= psi_r_alpha_pred;
                    end
                    11: begin
                        // Wait for multiplier pipeline
                    end
                    12: begin
                        psi_a_sq <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                        mul_a <= psi_r_beta_pred;
                        mul_b <= psi_r_beta_pred;
                    end
                    13: begin
                        // Wait for multiplier pipeline
                    end
                    14: begin
                        psi_b_sq <= mul_saturated;
                        psi_sq <= sat_add(psi_a_sq, mul_saturated);
                        if (mul_overflow || is_add_ovf(psi_a_sq, mul_saturated))
                            overflow_latched <= 1'b1;
                    end
                    15: begin
                        flux_err <= sat_sub(PSI_REF_SQ, psi_sq);
                        if (is_sub_ovf(PSI_REF_SQ, psi_sq))
                            overflow_latched <= 1'b1;
                    end
                    16: begin
                        mul_a <= flux_err;
                        mul_b <= flux_err;
                    end
                    17: begin
                        // Wait for multiplier pipeline
                    end
                    18: begin
                        flux_err_sq <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                        mul_a <= LAMBDA_T;
                        mul_b <= torque_err_sq;
                    end
                    19: begin
                        // Wait for multiplier pipeline
                    end
                    20: begin
                        cost_t <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                        mul_a <= LAMBDA_PSI;
                        mul_b <= flux_err_sq;
                    end
                    21: begin
                        // Wait for multiplier pipeline
                    end
                    22: begin
                        cost_f <= mul_saturated;
                        if (mul_overflow) overflow_latched <= 1'b1;
                    end
                    23: begin
                        // Saturating final addition and comprehensive overflow flag check
                        if (overflow_latched || mul_overflow || is_add_ovf(cost_t, cost_f) || 
                            (cost_t == INT_MAX) || (cost_f == INT_MAX)) begin
                            cost <= INT_MAX;
                            cost_overflow <= 1'b1;
                        end else begin
                            cost <= cost_t + cost_f;
                            cost_overflow <= 1'b0;
                        end
                    end
                    24: begin
                        done <= 1'b1;
                        busy <= 1'b0;
                    end
                    default: begin
                        busy <= 1'b0; 
                    end
                endcase
            end
        end
    end
endmodule