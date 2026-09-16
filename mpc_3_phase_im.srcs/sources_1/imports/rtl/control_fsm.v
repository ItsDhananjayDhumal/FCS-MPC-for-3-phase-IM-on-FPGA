`timescale 1ns / 1ps

module control_fsm #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS  = `FRAC_BITS
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          enable,         

    output reg                           adc_start,
    input  wire                          adc_done,
    input  wire [11:0]                   adc_data_ch0,
    input  wire [11:0]                   adc_data_ch1,

    output reg                           sample_tick,    
    input  wire signed [DATA_WIDTH-1:0]  speed_elec,
    input  wire                          speed_valid,

    output reg                           clarke_start,
    output reg  [11:0]                   clarke_ia_raw,
    output reg  [11:0]                   clarke_ib_raw,
    input  wire                          clarke_done,
    input  wire signed [DATA_WIDTH-1:0]  i_alpha,
    input  wire signed [DATA_WIDTH-1:0]  i_beta,

    output reg                           flux_start,
    input  wire                          flux_done,
    input  wire signed [DATA_WIDTH-1:0]  psi_r_alpha,
    input  wire signed [DATA_WIDTH-1:0]  psi_r_beta,
    input  wire signed [DATA_WIDTH-1:0]  wr_psi_alpha,
    input  wire signed [DATA_WIDTH-1:0]  wr_psi_beta,

    output reg  [2:0]                    vec_idx,
    input  wire signed [DATA_WIDTH-1:0]  vs_alpha,
    input  wire signed [DATA_WIDTH-1:0]  vs_beta,
    input  wire [2:0]                    lut_switch_state,

    output reg                           pred_start,
    input  wire                          pred_done,
    input  wire signed [DATA_WIDTH-1:0]  is_alpha_pred,
    input  wire signed [DATA_WIDTH-1:0]  is_beta_pred,
    input  wire signed [DATA_WIDTH-1:0]  psi_r_alpha_pred,
    input  wire signed [DATA_WIDTH-1:0]  psi_r_beta_pred,

    output reg                           cost_start,
    input  wire                          cost_done,
    input  wire signed [DATA_WIDTH-1:0]  cost_value,

    output reg                           sel_reset,
    output reg                           sel_cost_valid,
    input  wire                          sel_done,
    input  wire [2:0]                    opt_switch_state,

    output reg                           gate_update,
    output reg  [2:0]                    gate_switch_state,

    output reg  [4:0]                    fsm_state_out,  
    output reg                           heartbeat        
);

    `include "mpc_params.vh"

    
    
    reg [15:0] ts_counter;
    wire ts_tick = (ts_counter == `TS_COUNTER_MAX - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ts_counter <= 16'd0;
        else if (ts_counter >= `TS_COUNTER_MAX - 1)
            ts_counter <= 16'd0;
        else
            ts_counter <= ts_counter + 16'd1;
    end

    
    
    reg [12:0] hb_prescaler;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_prescaler <= 0;
            heartbeat <= 0;
        end else if (ts_tick) begin
            if (hb_prescaler >= 13'd4999) begin 
                hb_prescaler <= 0;
                heartbeat <= ~heartbeat;
            end else begin
                hb_prescaler <= hb_prescaler + 1;
            end
        end
    end

    
    
    localparam [4:0]
        S_IDLE        = 5'd0,
        S_WAIT_TS     = 5'd1,
        S_START_ADC   = 5'd2,
        S_WAIT_ADC    = 5'd3,
        S_CLARKE      = 5'd4,
        S_WAIT_CLARKE = 5'd5,
        S_FLUX        = 5'd6,
        S_WAIT_FLUX   = 5'd7,
        S_MPC_INIT    = 5'd8,
        S_MPC_PREDICT = 5'd9,
        S_WAIT_PRED   = 5'd10,
        S_MPC_COST    = 5'd11,
        S_WAIT_COST   = 5'd12,
        S_MPC_NEXT    = 5'd13,
        S_APPLY       = 5'd14,
        S_DISABLED    = 5'd15,
        S_WAIT_SEL    = 5'd16,
        S_ERROR       = 5'd17;

    reg [4:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            adc_start        <= 1'b0;
            sample_tick      <= 1'b0;
            clarke_start     <= 1'b0;
            clarke_ia_raw    <= 12'd0;
            clarke_ib_raw    <= 12'd0;
            flux_start       <= 1'b0;
            vec_idx          <= 3'd0;
            pred_start       <= 1'b0;
            cost_start       <= 1'b0;
            sel_reset        <= 1'b0;
            sel_cost_valid   <= 1'b0;
            gate_update      <= 1'b0;
            gate_switch_state <= 3'b000;
        end else if (ts_counter == 16'd9900 && state != S_WAIT_TS && state != S_IDLE && state != S_DISABLED && state != S_ERROR) begin
            state <= S_ERROR;
        end else begin
            
            adc_start      <= 1'b0;
            sample_tick    <= 1'b0;
            clarke_start   <= 1'b0;
            flux_start     <= 1'b0;
            pred_start     <= 1'b0;
            cost_start     <= 1'b0;
            sel_reset      <= 1'b0;
            sel_cost_valid <= 1'b0;
            gate_update    <= 1'b0;

            case (state)
                
                S_IDLE: begin
                    if (enable)
                        state <= S_WAIT_TS;
                    else
                        state <= S_DISABLED;
                end

                S_DISABLED: begin
                    
                    gate_switch_state <= 3'b000;
                    gate_update       <= 1'b1;
                    if (enable)
                        state <= S_WAIT_TS;
                end

                S_WAIT_TS: begin
                    if (!enable) begin
                        state <= S_DISABLED;
                    end else if (ts_tick) begin
                        sample_tick <= 1'b1; 
                        state <= S_START_ADC;
                    end
                end

                S_START_ADC: begin
                    adc_start <= 1'b1;
                    state <= S_WAIT_ADC;
                end

                S_WAIT_ADC: begin
                    if (adc_done) begin
                        clarke_ia_raw <= adc_data_ch0;
                        clarke_ib_raw <= adc_data_ch1;

                        if (adc_data_ch0 > `MAX_CURRENT_RAW || adc_data_ch0 < `MIN_CURRENT_RAW ||
                            adc_data_ch1 > `MAX_CURRENT_RAW || adc_data_ch1 < `MIN_CURRENT_RAW) begin
                            state <= S_ERROR; 
                        end else begin
                            state <= S_CLARKE;
                        end
                    end
                end

                S_CLARKE: begin
                    clarke_start <= 1'b1;
                    state <= S_WAIT_CLARKE;
                end

                S_WAIT_CLARKE: begin
                    if (clarke_done) begin
                        state <= S_FLUX;
                    end
                end

                S_FLUX: begin
                    flux_start <= 1'b1;
                    state <= S_WAIT_FLUX;
                end

                S_WAIT_FLUX: begin
                    if (flux_done) begin
                        state <= S_MPC_INIT;
                    end
                end

                S_MPC_INIT: begin
                    
                    sel_reset <= 1'b1;
                    vec_idx   <= 3'd0;
                    state     <= S_MPC_PREDICT;
                end

                S_MPC_PREDICT: begin

                    pred_start <= 1'b1;
                    state <= S_WAIT_PRED;
                end

                S_WAIT_PRED: begin
                    if (pred_done) begin
                        state <= S_MPC_COST;
                    end
                end

                S_MPC_COST: begin
                    
                    cost_start <= 1'b1;
                    state <= S_WAIT_COST;
                end

                S_WAIT_COST: begin
                    if (cost_done) begin
                        sel_cost_valid <= 1'b1;
                        state <= S_WAIT_SEL;
                    end
                end

                                
                S_WAIT_SEL: begin
                    state <= S_MPC_NEXT;
                end

                S_ERROR: begin
                    gate_switch_state <= 3'b000;
                    gate_update       <= 1'b1;
                    if (!enable) state <= S_IDLE; 
                end

                S_MPC_NEXT: begin
                    if (sel_done) begin
                        
                        state <= S_APPLY;
                    end else begin
                        
                        vec_idx <= vec_idx + 3'd1;
                        state   <= S_MPC_PREDICT;
                    end
                end

                S_APPLY: begin
                    
                    gate_switch_state <= opt_switch_state;
                    gate_update       <= 1'b1;
                    state <= S_WAIT_TS; 
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    always @(*) fsm_state_out = state;

endmodule
