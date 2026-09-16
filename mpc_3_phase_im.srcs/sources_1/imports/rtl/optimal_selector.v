`timescale 1ns / 1ps

module optimal_selector #(
    parameter DATA_WIDTH = `DATA_WIDTH 
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          reset_search, 
    input  wire signed [DATA_WIDTH-1:0]  cost,         
    input  wire [2:0]                    switch_state,  
    input  wire                          cost_valid,    
    output reg  [2:0]                    opt_switch_state, 
    output reg                           done           
);

    `include "mpc_params.vh"

    reg signed [DATA_WIDTH-1:0] min_cost;
    reg [3:0] eval_count; 

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            min_cost         <= {1'b0, {(DATA_WIDTH-1){1'b1}}}; 
            opt_switch_state <= 3'b000;
            eval_count       <= 4'd0;
            done             <= 1'b0;
        end else begin
            done <= 1'b0; 

            if (reset_search) begin
                
                min_cost         <= {1'b0, {(DATA_WIDTH-1){1'b1}}}; 
                opt_switch_state <= 3'b000;
                eval_count       <= 4'd0;
            end else if (cost_valid) begin
                
                if (cost < min_cost) begin
                    min_cost         <= cost;
                    opt_switch_state <= switch_state;
                end
                
                if (eval_count == `NUM_VECTORS - 1) begin
                    done <= 1'b1; 
                end
                eval_count <= eval_count + 4'd1;
            end
        end
    end

endmodule
