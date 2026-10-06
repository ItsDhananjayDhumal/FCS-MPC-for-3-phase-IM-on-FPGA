`timescale 1ns / 1ps

`include "mpc_params.vh"

module encoder_reader #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
    // parameter ENCODER_CPR = `ENCODER_CPR,
    // parameter POLE_PAIRS = `POLE_PAIRS,
    // parameter SWITCHING_FREQ = `SWITCHING_FREQ
) (
    input  wire clk,
    input  wire rst_n,
    input  wire enc_a,
    input  wire enc_b,
    input  wire enc_z,
    input  wire sample_tick,
    output reg  signed [31:0] position,
    output reg  signed [DATA_WIDTH-1:0] speed_elec,
    output reg  speed_valid
);

    parameter SPEED_SCALE = `SPEED_SCALE;

    reg [2:0] a_sync, b_sync, z_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 3'b000;
            b_sync <= 3'b000;
            z_sync <= 3'b000;
        end else begin
            a_sync <= {a_sync[1:0], enc_a};
            b_sync <= {b_sync[1:0], enc_b};
            z_sync <= {z_sync[1:0], enc_z};
        end
    end

    wire a_in = a_sync[1];
    wire b_in = b_sync[1];
    wire z_in = z_sync[1];
    wire a_prev = a_sync[2];
    wire b_prev = b_sync[2];
    wire z_prev = z_sync[2];
    wire z_rise = z_in && !z_prev;

    wire [3:0] quad_state = {a_prev, b_prev, a_in, b_in};
    reg [1:0] move; 

    always @(*) begin
        case (quad_state)
            4'b0001, 4'b0111, 4'b1110, 4'b1000: move = 2'd1; 
            4'b0010, 4'b1011, 4'b1101, 4'b0100: move = 2'd2; 
            default: move = 2'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            position <= 32'sd0;
        end else if (1'b0) begin  // Z-index reset disabled for speed estimation
            position <= 32'sd0;
        end else if (move == 2'd1) begin
            position <= position + 32'sd1;
        end else if (move == 2'd2) begin
            position <= position - 32'sd1;
        end
    end

    reg signed [31:0] prev_position;
    reg signed [DATA_WIDTH-1:0] speed_elec_prev;
    reg [2:0] calc_state;

    reg signed [31:0] delta_reg;
    reg signed [2*DATA_WIDTH-1:0] raw_mult_reg;
    reg signed [2*DATA_WIDTH-1:0] filt_term1_reg;
    reg signed [2*DATA_WIDTH-1:0] filt_term2_reg;

    wire signed [DATA_WIDTH-1:0] speed_raw = raw_mult_reg[DATA_WIDTH-1 : 0];
    wire signed [DATA_WIDTH-1:0] t1_trunc = filt_term1_reg[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
    wire signed [DATA_WIDTH-1:0] t2_trunc = filt_term2_reg[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_position <= 32'sd0;
            speed_elec <= 0;
            speed_elec_prev <= 0;
            speed_valid <= 1'b0;
            calc_state <= 3'd0;
            delta_reg <= 0;
            raw_mult_reg <= 0;
            filt_term1_reg <= 0;
            filt_term2_reg <= 0;
        end else begin
            speed_valid <= 1'b0; 
            case (calc_state)
                3'd0: begin
                    if (sample_tick) begin
                        delta_reg <= position - prev_position;
                        prev_position <= position; 
                        calc_state <= 3'd1;
                    end
                end
                3'd1: begin
                    
                    raw_mult_reg <= delta_reg * SPEED_SCALE;
                    calc_state <= 3'd2;
                end
                3'd2: begin
                    
                    filt_term1_reg <= `SPEED_ALPHA * speed_raw;
                    filt_term2_reg <= `SPEED_ONE_MINUS_ALPHA * speed_elec_prev;
                    calc_state <= 3'd3;
                end
                3'd3: begin
                    
                    speed_elec <= t1_trunc + t2_trunc;
                    speed_elec_prev <= t1_trunc + t2_trunc;
                    speed_valid <= 1'b1;
                    calc_state <= 3'd0;
                end
                default: calc_state <= 3'd0;
            endcase
        end
    end

endmodule
