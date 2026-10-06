`timescale 1ns / 1ps

module mpc_top (
    
    input  wire        sys_clk_i,       
    input  wire        cpu_resetn,      

    input  wire        btnu,            
    input  wire        btnc,            
    input  wire        btnd,            

    output wire        adc_cs_n,        
    input  wire        adc_d0,          
    input  wire        adc_d1,          
    output wire        adc_sclk,        

    input  wire        enc_a,           
    input  wire        enc_b,           
    input  wire        enc_z,           

    output wire        gate_ah,         
    output wire        gate_al,         
    output wire        gate_bh,         
    output wire        gate_bl,         
    output wire        gate_ch,         
    output wire        gate_cl,         
    output wire        inverter_en,     

    output wire [7:0]  led,             
    input  wire [7:0]  sw               
);

    `include "mpc_params.vh"

    
    
    wire clk = sys_clk_i;  

    reg rst_sync1, rst_sync2;
    always @(posedge clk or negedge cpu_resetn) begin
        if (!cpu_resetn) begin
            rst_sync1 <= 1'b0;
            rst_sync2 <= 1'b0;
        end else begin
            rst_sync1 <= 1'b1;
            rst_sync2 <= rst_sync1;
        end
    end
    wire rst_n = rst_sync2;  

    reg btnu_sync1, btnu_sync2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btnu_sync1 <= 1'b0;
            btnu_sync2 <= 1'b0;
        end else begin
            btnu_sync1 <= btnu;
            btnu_sync2 <= btnu_sync1;
        end
    end

    // Debounce filter: require stable input for 10ms (1,000,000 cycles)
    reg enable_debounced;
    reg [19:0] db_counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable_debounced <= 1'b0;
            db_counter <= 20'd0;
        end else if (btnu_sync2 != enable_debounced) begin
            if (db_counter >= 20'd999999) begin
                enable_debounced <= btnu_sync2;
                db_counter <= 20'd0;
            end else begin
                db_counter <= db_counter + 20'd1;
            end
        end else begin
            db_counter <= 20'd0;
        end
    end
    wire enable = enable_debounced;

    assign inverter_en = enable;

    

    
    wire        adc_start_w;
    wire        adc_done_w;
    wire [11:0] adc_data_ch0_w;
    wire [11:0] adc_data_ch1_w;

    wire        sample_tick_w;
    wire signed [31:0] enc_position_w;
    wire signed [`DATA_WIDTH-1:0] speed_elec_w;
    wire        speed_valid_w;

    wire        clarke_start_w;
    wire [11:0] clarke_ia_raw_w;
    wire [11:0] clarke_ib_raw_w;
    wire        clarke_done_w;
    wire signed [`DATA_WIDTH-1:0] i_alpha_w;
    wire signed [`DATA_WIDTH-1:0] i_beta_w;

    wire        flux_start_w;
    wire        flux_done_w;
    wire signed [`DATA_WIDTH-1:0] psi_r_alpha_w;
    wire signed [`DATA_WIDTH-1:0] psi_r_beta_w;
    wire signed [`DATA_WIDTH-1:0] wr_psi_alpha_w;
    wire signed [`DATA_WIDTH-1:0] wr_psi_beta_w;

    wire signed [`DATA_WIDTH-1:0] vdc_q_w;
    wire signed [`DATA_WIDTH-1:0] v1_alpha_w;
    wire signed [`DATA_WIDTH-1:0] v2_alpha_w;
    wire signed [`DATA_WIDTH-1:0] v2_beta_w;
    wire        vdc_valid_w;

    wire [2:0]  vec_idx_w;
    wire signed [`DATA_WIDTH-1:0] vs_alpha_w;
    wire signed [`DATA_WIDTH-1:0] vs_beta_w;
    wire [2:0]  lut_switch_state_w;

    wire        pred_start_w;
    wire        pred_done_w;
    wire signed [`DATA_WIDTH-1:0] is_alpha_pred_w;
    wire signed [`DATA_WIDTH-1:0] is_beta_pred_w;
    wire signed [`DATA_WIDTH-1:0] psi_r_alpha_pred_w;
    wire signed [`DATA_WIDTH-1:0] psi_r_beta_pred_w;

    wire        cost_start_w;
    wire        cost_done_w;
    wire signed [`DATA_WIDTH-1:0] cost_value_w;

    wire        sel_reset_w;
    wire        sel_cost_valid_w;
    wire        sel_done_w;
    wire [2:0]  opt_switch_state_w;

    wire        gate_update_w;
    wire [2:0]  gate_switch_state_w;

    wire [4:0]  fsm_state_w;  // 5-bit: supports states 0-17
    wire        heartbeat_w;

    // Speed PI controller output
    wire signed [`DATA_WIDTH-1:0] te_ref_w;

    // Auto-tare calibration signals
    wire signed [12:0] adc_offset_w;
    wire               cal_done_w;

    

    
    adc_pmod_ad1 #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_adc (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (adc_start_w),
        .adc_d0    (adc_d0),
        .adc_d1    (adc_d1),
        .adc_cs_n  (adc_cs_n),
        .adc_sclk  (adc_sclk),
        .data_ch0  (adc_data_ch0_w),
        .data_ch1  (adc_data_ch1_w),
        .done      (adc_done_w)
    );

    encoder_reader #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_encoder (
        .clk         (clk),
        .rst_n       (rst_n),
        .enc_a       (enc_a),
        .enc_b       (enc_b),
        .enc_z       (enc_z),
        .sample_tick (sample_tick_w),
        .position    (enc_position_w),
        .speed_elec  (speed_elec_w),
        .speed_valid (speed_valid_w)
    );

    clarke_transform #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_clarke (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (clarke_start_w),
        .ia_raw     (clarke_ia_raw_w),
        .ib_raw     (clarke_ib_raw_w),
        .adc_offset (adc_offset_w),    // Dynamic offset from auto-tare
        .i_alpha    (i_alpha_w),
        .i_beta     (i_beta_w),
        .done       (clarke_done_w)
    );

    flux_observer #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_flux (
        .clk          (clk),
        .rst_n        (rst_n),
        .start        (flux_start_w),
        .i_alpha      (i_alpha_w),
        .i_beta       (i_beta_w),
        .speed_elec   (speed_elec_w),
        .psi_r_alpha  (psi_r_alpha_w),
        .psi_r_beta   (psi_r_beta_w),
        .wr_psi_alpha (wr_psi_alpha_w),
        .wr_psi_beta  (wr_psi_beta_w),
        .done         (flux_done_w)
    );

    vdc_manager #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_vdc (
        .clk          (clk),
        .rst_n        (rst_n),
        .sw           (sw),
        .vdc_q        (vdc_q_w),
        .v1_alpha_out (v1_alpha_w),
        .v2_alpha_out (v2_alpha_w),
        .v2_beta_out  (v2_beta_w),
        .vdc_valid    (vdc_valid_w)
    );

    voltage_lut #(
        .DATA_WIDTH(`DATA_WIDTH)
    ) u_vlut (
        .vec_idx      (vec_idx_w),
        .v1_alpha     (v1_alpha_w),
        .v2_alpha     (v2_alpha_w),
        .v2_beta      (v2_beta_w),
        .vs_alpha     (vs_alpha_w),
        .vs_beta      (vs_beta_w),
        .switch_state (lut_switch_state_w)
    );

    motor_predictor #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_predictor (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (pred_start_w),
        .i_alpha          (i_alpha_w),
        .i_beta           (i_beta_w),
        .psi_r_alpha      (psi_r_alpha_w),
        .psi_r_beta       (psi_r_beta_w),
        .wr_psi_alpha     (wr_psi_alpha_w),
        .wr_psi_beta      (wr_psi_beta_w),
        .vs_alpha         (vs_alpha_w),
        .vs_beta          (vs_beta_w),
        .is_alpha_pred    (is_alpha_pred_w),
        .is_beta_pred     (is_beta_pred_w),
        .psi_r_alpha_pred (psi_r_alpha_pred_w),
        .psi_r_beta_pred  (psi_r_beta_pred_w),
        .done             (pred_done_w)
    );

    cost_evaluator #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_cost (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (cost_start_w),
        .is_alpha_pred    (is_alpha_pred_w),
        .is_beta_pred     (is_beta_pred_w),
        .psi_r_alpha_pred (psi_r_alpha_pred_w),
        .psi_r_beta_pred  (psi_r_beta_pred_w),
        .te_ref_in        (te_ref_w),          // Connected to speed_pi output
        .cost             (cost_value_w),
        .done             (cost_done_w)
    );

    // Speed PI controller: cascaded speed loop driving torque reference
    speed_pi #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_speed_pi (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_tick (sample_tick_w),
        .speed_ref   (`SPEED_REF_DEFAULT),
        .speed_fb    (speed_elec_w),
        .te_ref      (te_ref_w)
    );

    optimal_selector #(
        .DATA_WIDTH(`DATA_WIDTH)
    ) u_selector (
        .clk              (clk),
        .rst_n            (rst_n),
        .reset_search     (sel_reset_w),
        .cost             (cost_value_w),
        .switch_state     (lut_switch_state_w),
        .cost_valid       (sel_cost_valid_w),
        .opt_switch_state (opt_switch_state_w),
        .done             (sel_done_w)
    );

    gate_driver u_gate (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (enable),
        .switch_state (gate_switch_state_w),
        .update_tick  (gate_update_w),
        .gate_ah      (gate_ah),
        .gate_al      (gate_al),
        .gate_bh      (gate_bh),
        .gate_bl      (gate_bl),
        .gate_ch      (gate_ch),
        .gate_cl      (gate_cl)
    );

    control_fsm #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_fsm (
        .clk               (clk),
        .rst_n             (rst_n),
        .enable            (enable),
        
        .adc_start         (adc_start_w),
        .adc_done          (adc_done_w),
        .adc_data_ch0      (adc_data_ch0_w),
        .adc_data_ch1      (adc_data_ch1_w),
        
        .sample_tick       (sample_tick_w),
        .speed_elec        (speed_elec_w),
        .speed_valid       (speed_valid_w),
        
        .clarke_start      (clarke_start_w),
        .clarke_ia_raw     (clarke_ia_raw_w),
        .clarke_ib_raw     (clarke_ib_raw_w),
        .clarke_done       (clarke_done_w),
        .i_alpha           (i_alpha_w),
        .i_beta            (i_beta_w),
        
        .flux_start        (flux_start_w),
        .flux_done         (flux_done_w),
        .psi_r_alpha       (psi_r_alpha_w),
        .psi_r_beta        (psi_r_beta_w),
        .wr_psi_alpha      (wr_psi_alpha_w),
        .wr_psi_beta       (wr_psi_beta_w),
        
        .vec_idx           (vec_idx_w),
        .vs_alpha          (vs_alpha_w),
        .vs_beta           (vs_beta_w),
        .lut_switch_state  (lut_switch_state_w),
        
        .pred_start        (pred_start_w),
        .pred_done         (pred_done_w),
        .is_alpha_pred     (is_alpha_pred_w),
        .is_beta_pred      (is_beta_pred_w),
        .psi_r_alpha_pred  (psi_r_alpha_pred_w),
        .psi_r_beta_pred   (psi_r_beta_pred_w),
        
        .cost_start        (cost_start_w),
        .cost_done         (cost_done_w),
        .cost_value        (cost_value_w),
        
        .sel_reset         (sel_reset_w),
        .sel_cost_valid    (sel_cost_valid_w),
        .sel_done          (sel_done_w),
        .opt_switch_state  (opt_switch_state_w),
        
        .gate_update       (gate_update_w),
        .gate_switch_state (gate_switch_state_w),
        
        .adc_offset        (adc_offset_w),
        .cal_done          (cal_done_w),

        .fsm_state_out     (fsm_state_w),
        .heartbeat         (heartbeat_w)
    );

    

    
    
    assign led[7:4] = sw[7:4];
    assign led[3]   = heartbeat_w;
    assign led[2:0] = opt_switch_state_w;

endmodule
