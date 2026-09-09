`timescale 1ns / 1ps

module voltage_lut #(
    parameter DATA_WIDTH = 32
)(
    input  wire [2:0]                         vec_idx,     
    input  wire signed [DATA_WIDTH-1:0]       v1_alpha,    
    input  wire signed [DATA_WIDTH-1:0]       v2_alpha,    
    input  wire signed [DATA_WIDTH-1:0]       v2_beta,     
    output reg  signed [DATA_WIDTH-1:0]       vs_alpha,    
    output reg  signed [DATA_WIDTH-1:0]       vs_beta,     
    output reg  [2:0]                         switch_state 
);

    

    

    

    

    always @(*) begin
        case (vec_idx)
            3'd0: begin vs_alpha =  {DATA_WIDTH{1'b0}}; vs_beta =  {DATA_WIDTH{1'b0}}; switch_state = 3'b000; end
            3'd1: begin vs_alpha =  v1_alpha;            vs_beta =  {DATA_WIDTH{1'b0}}; switch_state = 3'b100; end
            3'd2: begin vs_alpha =  v2_alpha;            vs_beta =  v2_beta;             switch_state = 3'b110; end
            3'd3: begin vs_alpha = -v2_alpha;            vs_beta =  v2_beta;             switch_state = 3'b010; end
            3'd4: begin vs_alpha = -v1_alpha;            vs_beta =  {DATA_WIDTH{1'b0}}; switch_state = 3'b011; end
            3'd5: begin vs_alpha = -v2_alpha;            vs_beta = -v2_beta;             switch_state = 3'b001; end
            3'd6: begin vs_alpha =  v2_alpha;            vs_beta = -v2_beta;             switch_state = 3'b101; end
            3'd7: begin vs_alpha =  {DATA_WIDTH{1'b0}}; vs_beta =  {DATA_WIDTH{1'b0}}; switch_state = 3'b111; end
            default: begin vs_alpha = 0; vs_beta = 0; switch_state = 3'b000; end
        endcase
    end

endmodule
