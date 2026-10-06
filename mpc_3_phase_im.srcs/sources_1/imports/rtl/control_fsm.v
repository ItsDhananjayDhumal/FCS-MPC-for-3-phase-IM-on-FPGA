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

    output reg  signed [12:0]            adc_offset,     // Auto-tare calibrated ADC zero offset
    output reg                           cal_done,       // Calibration complete flag

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
        S_ERROR       = 5'd17,
        // Auto-tare calibration states
        S_CAL_WARMUP  = 5'd18,
        S_CAL_ADC     = 5'd19,
        S_CAL_WAIT    = 5'd20,
        S_CAL_ACCUM   = 5'd21,
        S_CAL_DONE    = 5'd22,
        // Pre-magnetization states
        S_MAG_WAIT_TS = 5'd23,
        S_MAG_APPLY   = 5'd24,
        S_MAG_ADC     = 5'd25,
        S_MAG_WAIT_ADC= 5'd26,
        S_MAG_CHECK   = 5'd27;

    reg [4:0] state;

    reg [12:0] hb_prescaler;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_prescaler <= 0;
            heartbeat <= 0;
        end else if (state == S_ERROR) begin
            heartbeat <= 1'b1;  // Solid ON = fault indicator
        end else if (ts_tick) begin
            if (hb_prescaler >= 13'd4999) begin 
                hb_prescaler <= 0;
                heartbeat <= ~heartbeat;
            end else begin
                hb_prescaler <= hb_prescaler + 1;
            end
        end
    end

    // Auto-tare calibration registers
    reg [28:0] warmup_cnt;           // 5-second warmup counter
    reg [8:0]  cal_sample_cnt;       // sample counter (0-255)
    reg [8:0]  cal_valid_cnt;        // count of non-outlier samples
    reg signed [23:0] cal_accum_ch0; // accumulator for ch0 (enough for 256*4095)
    reg signed [23:0] cal_accum_ch1; // accumulator for ch1
    reg signed [12:0] cal_mean;      // running mean estimate

    // Pre-magnetization registers
    reg [7:0] mag_cycle_cnt;         // magnetization Ts period counter

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
            adc_offset       <= 13'sd2048;  // Default until calibration
            cal_done         <= 1'b0;
            warmup_cnt       <= 0;
            cal_sample_cnt   <= 0;
            cal_valid_cnt    <= 0;
            cal_accum_ch0    <= 0;
            cal_accum_ch1    <= 0;
            cal_mean         <= 13'sd2048;
            mag_cycle_cnt    <= 0;
        end else if (ts_counter == 16'd9900 && state != S_WAIT_TS && state != S_IDLE && 
                     state != S_DISABLED && state != S_ERROR &&
                     state != S_CAL_WARMUP && state != S_CAL_ADC && state != S_CAL_WAIT &&
                     state != S_CAL_ACCUM && state != S_CAL_DONE &&
                     state != S_MAG_WAIT_TS && state != S_MAG_APPLY && 
                     state != S_MAG_ADC && state != S_MAG_WAIT_ADC && state != S_MAG_CHECK) begin
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
                    if (!cal_done)
                        state <= S_CAL_WARMUP;  // Auto-tare first
                    else if (enable)
                        state <= S_MAG_WAIT_TS; // Magnetize before MPC
                    else
                        state <= S_DISABLED;
                end

                S_DISABLED: begin
                    
                    gate_switch_state <= 3'b000;
                    gate_update       <= 1'b1;
                    if (enable)
                        state <= S_MAG_WAIT_TS; // Magnetize on enable
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

                        // Symmetric overcurrent protection around calibrated zero offset (+/- 5.0A = +/- 310 counts)
                        if (($signed({1'b0, adc_data_ch0}) - adc_offset) > `MAX_CURRENT_DELTA ||
                            ($signed({1'b0, adc_data_ch0}) - adc_offset) < -`MAX_CURRENT_DELTA ||
                            ($signed({1'b0, adc_data_ch1}) - adc_offset) > `MAX_CURRENT_DELTA ||
                            ($signed({1'b0, adc_data_ch1}) - adc_offset) < -`MAX_CURRENT_DELTA) begin
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
                    // Tri-state coast: keep all gates LOW (no gate_update!)
                    // gate_switch_state does not matter since gate_update stays 0
                    // Heartbeat is frozen (handled below) to signal fault
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

                // ============================================
                // Auto-Tare Calibration States
                // ============================================

                S_CAL_WARMUP: begin
                    // Wait 5 seconds for sensor/power supply to settle
                    if (warmup_cnt >= `AUTOTARE_WARMUP_CYCLES) begin
                        warmup_cnt     <= 0;
                        cal_sample_cnt <= 0;
                        cal_valid_cnt  <= 0;
                        cal_accum_ch0  <= 0;
                        cal_accum_ch1  <= 0;
                        cal_mean       <= 13'sd2048; // Initial estimate
                        state          <= S_CAL_ADC;
                    end else begin
                        warmup_cnt <= warmup_cnt + 1;
                    end
                end

                S_CAL_ADC: begin
                    // Trigger ADC conversion
                    adc_start <= 1'b1;
                    state     <= S_CAL_WAIT;
                end

                S_CAL_WAIT: begin
                    // Wait for ADC conversion to complete
                    if (adc_done) begin
                        state <= S_CAL_ACCUM;
                    end
                end

                S_CAL_ACCUM: begin
                    // Outlier rejection: check if sample is within ±THRESHOLD of mean
                    if (($signed({1'b0, adc_data_ch0}) > (cal_mean - `AUTOTARE_OUTLIER_THRESH)) &&
                        ($signed({1'b0, adc_data_ch0}) < (cal_mean + `AUTOTARE_OUTLIER_THRESH))) begin
                        cal_accum_ch0 <= cal_accum_ch0 + $signed({1'b0, adc_data_ch0});
                        cal_accum_ch1 <= cal_accum_ch1 + $signed({1'b0, adc_data_ch1});
                        cal_valid_cnt <= cal_valid_cnt + 1;
                        // Update running mean from accumulated average
                        if (cal_valid_cnt > 0)
                            cal_mean <= (cal_accum_ch0 + $signed({1'b0, adc_data_ch0})) / 
                                        ($signed({20'd0, cal_valid_cnt}) + 1);
                    end
                    // else: outlier, skip this sample

                    cal_sample_cnt <= cal_sample_cnt + 1;

                    if (cal_sample_cnt >= `AUTOTARE_NUM_SAMPLES - 1) begin
                        state <= S_CAL_DONE;
                    end else begin
                        state <= S_CAL_ADC; // Take next sample
                    end
                end

                S_CAL_DONE: begin
                    // Compute final offset = average of valid samples
                    if (cal_valid_cnt > 0)
                        adc_offset <= cal_accum_ch0[23:0] / $signed({15'd0, cal_valid_cnt});
                    else
                        adc_offset <= 13'sd2048; // Fallback if all samples rejected
                    cal_done <= 1'b1;
                    state    <= S_DISABLED; // Wait for enable
                end

                // ============================================
                // Pre-Magnetization Startup States
                // ============================================

                S_MAG_WAIT_TS: begin
                    // Wait for Ts tick to synchronize magnetization
                    if (!enable) begin
                        state <= S_DISABLED;
                    end else if (ts_tick) begin
                        state <= S_MAG_APPLY;
                    end
                end

                S_MAG_APPLY: begin
                    // Apply fixed voltage vector for flux build-up
                    gate_switch_state <= `MAG_VECTOR;
                    gate_update       <= 1'b1;
                    // Trigger ADC to monitor current
                    adc_start <= 1'b1;
                    state     <= S_MAG_ADC;
                end

                S_MAG_ADC: begin
                    state <= S_MAG_WAIT_ADC;
                end

                S_MAG_WAIT_ADC: begin
                    if (adc_done) begin
                        state <= S_MAG_CHECK;
                    end
                end

                S_MAG_CHECK: begin
                    // Check overcurrent protection during magnetization (25% limit = 1.25A = 78 counts)
                    if (($signed({1'b0, adc_data_ch0}) - adc_offset) > `MAG_CURRENT_MAX_DELTA ||
                        ($signed({1'b0, adc_data_ch0}) - adc_offset) < -`MAG_CURRENT_MAX_DELTA ||
                        ($signed({1'b0, adc_data_ch1}) - adc_offset) > `MAG_CURRENT_MAX_DELTA ||
                        ($signed({1'b0, adc_data_ch1}) - adc_offset) < -`MAG_CURRENT_MAX_DELTA) begin
                        // Current too high: stop magnetization, go to error
                        gate_switch_state <= 3'b000;
                        gate_update       <= 1'b1;
                        state             <= S_ERROR;
                    end else if (mag_cycle_cnt >= `MAG_CYCLES - 1) begin
                        // Magnetization complete: transition to normal MPC
                        mag_cycle_cnt <= 0;
                        sample_tick   <= 1'b1; // Initial sample tick for encoder
                        state         <= S_WAIT_TS;
                    end else begin
                        mag_cycle_cnt <= mag_cycle_cnt + 1;
                        state         <= S_MAG_WAIT_TS;
                    end
                end
            endcase
        end
    end

    always @(*) fsm_state_out = state;

endmodule
