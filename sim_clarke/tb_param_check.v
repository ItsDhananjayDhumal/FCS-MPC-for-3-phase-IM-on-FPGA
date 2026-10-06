`timescale 1ns/1ps
module tb_param_check;
    localparam NUM_POLES = 4;
    localparam SWITCHING_FREQ = 10000;
    localparam ENCODER_CPR = 10000;
    // As written by user:
    parameter SPEED_SCALE_USER = 3294199 * NUM_POLES * SWITCHING_FREQ / ENCODER_CPR;
    // With 64-bit precision:
    parameter SPEED_SCALE_64   = 64'd3294199 * NUM_POLES * SWITCHING_FREQ / ENCODER_CPR;
    initial begin
        $display("SPEED_SCALE_USER = %0d", SPEED_SCALE_USER);
        $display("SPEED_SCALE_64   = %0d", SPEED_SCALE_64);
        $finish;
    end
endmodule
