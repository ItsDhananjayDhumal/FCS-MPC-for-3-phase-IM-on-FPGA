`timescale 1ns / 1ps

module fixed_point_mul #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_BITS  = 20
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
