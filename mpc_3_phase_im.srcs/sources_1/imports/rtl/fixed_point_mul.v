`timescale 1ns / 1ps
`include "mpc_params.vh"

module fixed_point_mul #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS  = `FRAC_BITS
)(
    input  wire                          clk,
    input  wire signed [DATA_WIDTH-1:0]  a,
    input  wire signed [DATA_WIDTH-1:0]  b,
    output wire signed [DATA_WIDTH-1:0]  result
);

    reg signed [2*DATA_WIDTH-1:0] product_reg;
    always @(posedge clk)
        product_reg <= a * b;

    
    reg signed [DATA_WIDTH-1:0] result_reg;
    always @(posedge clk)
        result_reg <= product_reg[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];

    assign result = result_reg;


endmodule
