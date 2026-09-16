`timescale 1ns / 1ps
`include "mpc_params.vh"

module speed_pi #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
) (
    input  wire clk,
    input  wire rst_n,
    input  wire sample_tick,    
    input  wire signed [DATA_WIDTH-1:0] speed_ref,
    input  wire signed [DATA_WIDTH-1:0] speed_fb,
    output reg  signed [DATA_WIDTH-1:0] te_ref
);

    reg signed [DATA_WIDTH-1:0] err;
    reg signed [DATA_WIDTH-1:0] err_integ;

    reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
    reg signed [2*DATA_WIDTH-1:0] mul_result_full;
    wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
    
    always @(posedge clk) begin
        mul_result_full <= mul_a * mul_b;
    end

    reg [2:0] state;
    reg signed [DATA_WIDTH-1:0] p_term;
    reg signed [DATA_WIDTH-1:0] i_term;
    reg signed [DATA_WIDTH-1:0] unclipped_te;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
            err <= 0;
            err_integ <= 0;
            te_ref <= 0;
            mul_a <= 0;
            mul_b <= 0;
            p_term <= 0;
            i_term <= 0;
            unclipped_te <= 0;
        end else begin
            case (state)
                0: begin
                    if (sample_tick) begin
                        err <= speed_ref - speed_fb;
                        state <= 1;
                    end
                end
                1: begin

                    err_integ <= err_integ + err;

                    mul_a <= `PI_KP;
                    mul_b <= err; 
                    
                    state <= 2;
                end
                2: begin
                    mul_a <= `PI_KP;
                    mul_b <= err;
                    state <= 3;
                end
                3: begin
                    
                    state <= 4;
                end
                4: begin
                    p_term <= mul_result;
                    
                    mul_a <= `PI_KI;
                    mul_b <= err_integ;
                    state <= 5;
                end
                5: begin
                    
                    state <= 6;
                end
                6: begin
                    i_term <= mul_result;
                    state <= 7;
                end
                7: begin
                    unclipped_te <= p_term + i_term;
                    state <= 8;
                end
                8: begin
                    
                    if (unclipped_te > `TE_MAX) begin
                        te_ref <= `TE_MAX;
                    end else if (unclipped_te < `TE_MIN) begin
                        te_ref <= `TE_MIN;
                    end else begin
                        te_ref <= unclipped_te;
                    end
                    state <= 0;
                end
            endcase
        end
    end

endmodule
