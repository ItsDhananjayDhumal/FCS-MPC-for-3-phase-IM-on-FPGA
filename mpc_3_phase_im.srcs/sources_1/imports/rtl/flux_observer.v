`timescale 1ns / 1ps
`include "mpc_params.vh"

module flux_observer #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire signed [DATA_WIDTH-1:0] i_alpha,
    input  wire signed [DATA_WIDTH-1:0] i_beta,
    input  wire signed [DATA_WIDTH-1:0] speed_elec,
    output reg  signed [DATA_WIDTH-1:0] psi_r_alpha,
    output reg  signed [DATA_WIDTH-1:0] psi_r_beta,
    output reg  signed [DATA_WIDTH-1:0] wr_psi_alpha,
    output reg  signed [DATA_WIDTH-1:0] wr_psi_beta,
    output reg  done
);

    localparam signed [DATA_WIDTH-1:0] E21  = `E21;
    localparam signed [DATA_WIDTH-1:0] E22  = `E22;
    localparam signed [DATA_WIDTH-1:0] TS_Q = `TS_Q;

    reg signed [DATA_WIDTH-1:0] psi_a_reg, psi_b_reg;

    // Clamp speed to prevent Forward Euler instability above sync speed
    reg signed [DATA_WIDTH-1:0] speed_clamped;
    always @(*) begin
        if (speed_elec > `SPEED_LIMIT)
            speed_clamped = `SPEED_LIMIT;
        else if (speed_elec < -`SPEED_LIMIT)
            speed_clamped = -`SPEED_LIMIT;
        else
            speed_clamped = speed_elec;
    end

    reg signed [DATA_WIDTH-1:0] term1, term2, term3;
    reg signed [DATA_WIDTH-1:0] term4, term5, term6;
    reg signed [DATA_WIDTH-1:0] psi_a_new, psi_b_new;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];

    always @(posedge clk) begin
        mul_result_full <= mul_a * mul_b;
    end

    reg [4:0] step;

    // E21 = Ts * (Lm / Tau_r)
    // E22 = 1 - (Ts / Tau_r)
    // Tau_r = Lr / Rr
    // TS_Q = Ts

    // term 1 = E21 * i_alpha
    // term 2 = E22 * psi_a_reg
    // term 3 = TS_Q * wr_psi_beta
    
    // term 4 = E21 * i_beta
    // term 5 = E22 * psi_b_reg
    // term 6 = TS_Q * wr_psi_alpha

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step <= 0;
            psi_a_reg <= 0;
            psi_b_reg <= 0;
            psi_r_alpha <= 0;
            psi_r_beta <= 0;
            wr_psi_alpha <= 0;
            wr_psi_beta <= 0;
            done <= 0;
            mul_a <= 0;
            mul_b <= 0;
            term1 <= 0; term2 <= 0; term3 <= 0;
            term4 <= 0; term5 <= 0; term6 <= 0;
            psi_a_new <= 0;
            psi_b_new <= 0;
        end else begin
            done <= 0; 
            
            case (step)
                0: begin
                    if (start) begin
                        // Use clamped speed for stability
                        mul_a <= speed_clamped;
                        mul_b <= psi_b_reg;
                        step <= 1;
                    end
                end
                1: begin
                    
                    step <= 2;
                end
                2: begin
                    wr_psi_beta <= mul_result; 

                    mul_a <= speed_clamped;
                    mul_b <= psi_a_reg;
                    step <= 3;
                end
                3: begin
                    
                    step <= 4;
                end
                4: begin
                    wr_psi_alpha <= mul_result; 

                    mul_a <= E21;
                    mul_b <= i_alpha;
                    step <= 5;
                end
                5: step <= 6;
                6: begin
                    term1 <= mul_result; 

                    mul_a <= E22;
                    mul_b <= psi_a_reg;
                    step <= 7;
                end
                7: step <= 8;
                8: begin
                    term2 <= mul_result; 

                    mul_a <= TS_Q;
                    mul_b <= wr_psi_beta;
                    step <= 9;
                end
                9: step <= 10;
                10: begin
                    term3 <= mul_result; 

                    mul_a <= E21;
                    mul_b <= i_beta;
                    step <= 11;
                end
                11: begin
                    
                    psi_a_new <= term1 + term2 - term3;
                    
                    step <= 12;
                end
                12: begin
                    term4 <= mul_result; 

                    mul_a <= TS_Q;
                    mul_b <= wr_psi_alpha;
                    step <= 13;
                end
                13: step <= 14;
                14: begin
                    term5 <= mul_result; 

                    mul_a <= E22;
                    mul_b <= psi_b_reg;
                    step <= 15;
                end
                15: step <= 16;
                16: begin
                    term6 <= mul_result; 
                    step <= 17;
                end
                17: begin
                    
                    psi_b_new <= term4 + term5 + term6;
                    step <= 18;
                end
                18: begin
                    
                    psi_a_reg <= psi_a_new;
                    psi_b_reg <= psi_b_new;
                    
                    psi_r_alpha <= psi_a_new;
                    psi_r_beta <= psi_b_new;
                    
                    done <= 1'b1;
                    step <= 0;
                end
                default: step <= 0;
            endcase
        end
    end
endmodule
