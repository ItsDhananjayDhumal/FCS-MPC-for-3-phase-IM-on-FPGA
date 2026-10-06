`timescale 1ns/1ps
`include "C:/Users/Dhananjay Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_params.vh"
module tb_test;
    parameter SPEED_SCALE = 3294199 * `NUM_POLES * `SWITCHING_FREQ / `ENCODER_CPR;
    parameter SPEED_SCALE_64 = 64'd3294199 * `NUM_POLES * `SWITCHING_FREQ / `ENCODER_CPR;
    initial begin
        $display("SPEED_SCALE = %0d", SPEED_SCALE);
        $display("SPEED_SCALE_64 = %0d", SPEED_SCALE_64);
        $finish;
    end
endmodule
