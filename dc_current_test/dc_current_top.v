`timescale 1ns / 1ps
// =============================================================================
// Module: dc_current_top
// Description: Standalone DC Current & Voltage Telemetry with SSD1306 OLED Display
//              - Target Board: Digilent Nexys Video (Artix-7 XC7A200T-1SBG484C)
//              - Analog Front-End: Digilent Pmod AD1 (Dual AD7476A 12-bit SAR ADC)
//              - Current Sensors: Allegro ACS712-40AB (50 mV/A, 2.510V zero-offset)
//              - Features:
//                1. Robust Auto-Tare Calibration matching physical ADC dynamics:
//                   - 5-second warmup countdown timer (5s -> 4s -> 3s -> 2s -> 1s)
//                   - 10 kHz paced sampling matching measurement settling time
//                   - 256-sample outlier-filtered running accumulator (+/-100 LSB)
//                   - Power-of-2 division via arithmetic right shift (cal_accum >>> 8)
//                   - Immediate filter seeding on calibration completion (zero-offset start)
//                   - BTNC pushbutton triggers on-demand re-tare/recalibration
//                2. High-Precision Pipelined Physical Conversions:
//                   - Q12.8 fixed-point IIR smoothing filter (zero truncation bias)
//                   - Multi-stage pipelined physical calculations (timing closure @ 100 MHz)
//                   - Voltage: (raw * 3300 mV) / 4096 = exact millivolts on ADC pin
//                   - Current: (raw - offset) * 16.113 mA/count (50 mV/A sensitivity)
//                3. Multi-Stage Pipelined OLED Formatter (7 non-overlapping steps):
//                   - Eliminates chained divisions/modulos for zero timing violations
//                   - SW[0] = 0 (Multi-View): Voltage & Current for CH0 & CH1 + Offsets
//                   - SW[0] = 1 (Precise View): 3-decimal Volts (e.g. 1.650V) + Raw/Offset
//                4. Diagnostic Board LEDs LD7..LD0 for instant visual status.
// =============================================================================

`include "mpc_params.vh"

module dc_current_top #(
    parameter SIMULATION      = 1'b0,
    parameter WARMUP_CYCLES   = SIMULATION ? 29'd100 : 29'd500_000_000, // 5s at 100 MHz
    parameter DEBOUNCE_CYCLES = SIMULATION ? 20'd50  : 20'd1_000_000,   // 10 ms at 100 MHz
    parameter SAMPLE_PERIOD   = SIMULATION ? 16'd50  : 16'd10_000,      // 100 us / 10 kHz
    parameter FMT_PERIOD      = SIMULATION ? 22'd250 : 22'd2_500_000    // 25 ms / 40 Hz
)(
    input  wire       sys_clk_i,    // 100 MHz Onboard Oscillator (Pin R4)
    input  wire       cpu_resetn,   // Red Pushbutton "CPU RESET" (Pin G4, Active-Low)
    input  wire       btnc,         // Center Pushbutton (Pin B22, Manual Re-Tare)
    input  wire [7:0] sw,           // Slide Switches (Bank 12, Pin E22...)

    // Digilent Pmod AD1 Hardware Interface (Pmod JB)
    output wire       adc_cs_n,     // Chip Select (Pin AB22)
    input  wire       adc_d0,       // Channel 0 SPI Data (Pin AB21)
    input  wire       adc_d1,       // Channel 1 SPI Data (Pin AB20)
    output wire       adc_sclk,     // SPI Clock 12.5 MHz (Pin AB18)

    // Univision UG-2832HSWEG04 SSD1306 OLED Hardware Interface (Bank 14)
    output wire       oled_sclk,    // SPI Clock (Pin W21)
    output wire       oled_sdin,    // SPI Data (Pin Y22)
    output wire       oled_dc,      // Data/Command (Pin W22)
    output wire       oled_res,     // Reset (Pin U21)
    output wire       oled_vbat,    // Panel Power Enable (Pin P20)
    output wire       oled_vdd,     // Logic Power Enable (Pin V22)

    // Diagnostic LEDs LD7..LD0 (Bank 13)
    output wire [7:0] led
);

    // -------------------------------------------------------------------------
    // 1. Reset Synchronizer
    // -------------------------------------------------------------------------
    reg rst_sync1, rst_sync2;
    always @(posedge sys_clk_i or negedge cpu_resetn) begin
        if (!cpu_resetn) begin
            rst_sync1 <= 1'b0;
            rst_sync2 <= 1'b0;
        end else begin
            rst_sync1 <= 1'b1;
            rst_sync2 <= rst_sync1;
        end
    end
    wire rst_n = rst_sync2;

    // -------------------------------------------------------------------------
    // 2. Center Button (BTNC) Debouncer for Manual Auto-Tare Trigger
    // -------------------------------------------------------------------------
    reg [19:0] btnc_cnt;
    reg        btnc_sync1, btnc_sync2, btnc_debounced, btnc_prev;
    wire       btnc_pulse = btnc_debounced && !btnc_prev;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            btnc_sync1     <= 1'b0;
            btnc_sync2     <= 1'b0;
            btnc_debounced <= 1'b0;
            btnc_prev      <= 1'b0;
            btnc_cnt       <= 20'd0;
        end else begin
            btnc_sync1 <= btnc;
            btnc_sync2 <= btnc_sync1;
            btnc_prev  <= btnc_debounced;

            if (btnc_sync2 != btnc_debounced) begin
                if (btnc_cnt >= DEBOUNCE_CYCLES) begin
                    btnc_debounced <= btnc_sync2;
                    btnc_cnt       <= 20'd0;
                end else begin
                    btnc_cnt <= btnc_cnt + 20'd1;
                end
            end else begin
                btnc_cnt <= 20'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. Switch Synchronizer
    // -------------------------------------------------------------------------
    reg [7:0] sw_sync1, sw_sync2;
    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            sw_sync1 <= 8'd0;
            sw_sync2 <= 8'd0;
        end else begin
            sw_sync1 <= sw;
            sw_sync2 <= sw_sync1;
        end
    end
    wire mode_high_prec = sw_sync2[0]; // SW0: 0=Multi-View, 1=High-Precision Voltage

    // -------------------------------------------------------------------------
    // 4. ADC Pmod AD1 SPI Master Instance
    // -------------------------------------------------------------------------
    reg        adc_start;
    wire [11:0] adc_data_ch0;
    wire [11:0] adc_data_ch1;
    wire       adc_done;

    adc_pmod_ad1 #(
        .DATA_WIDTH (`DATA_WIDTH),
        .FRAC_BITS  (`FRAC_BITS)
    ) u_adc (
        .clk      (sys_clk_i),
        .rst_n    (rst_n),
        .start    (adc_start),
        .adc_d0   (adc_d0),
        .adc_d1   (adc_d1),
        .adc_cs_n (adc_cs_n),
        .adc_sclk (adc_sclk),
        .data_ch0 (adc_data_ch0),
        .data_ch1 (adc_data_ch1),
        .done     (adc_done)
    );

    // -------------------------------------------------------------------------
    // 5. Auto-Tare Calibration Engine & Measurement FSM
    //    Paced sampling at 10 kHz ensures identical analog settling dynamics
    //    between calibration and measurement, completely eliminating DC offset bias.
    // -------------------------------------------------------------------------
    localparam S_IDLE        = 4'd0;
    localparam S_CAL_WARMUP  = 4'd1;
    localparam S_CAL_ADC     = 4'd2;
    localparam S_CAL_WAIT    = 4'd3;
    localparam S_CAL_ACCUM   = 4'd4;
    localparam S_CAL_DONE    = 4'd5;
    localparam S_MEASURE     = 4'd6;
    localparam S_WAIT_MEAS   = 4'd7;
    localparam S_CAL_DELAY   = 4'd8;

    reg [3:0] state;

    reg [28:0]        warmup_cnt;
    reg [15:0]        cal_sample_cnt;
    reg [15:0]        cal_valid_cnt;
    reg signed [23:0] cal_accum_ch0;
    reg signed [23:0] cal_accum_ch1;
    reg signed [12:0] cal_mean_ch0;
    reg signed [12:0] cal_mean_ch1;
    reg signed [12:0] adc_offset_ch0;
    reg signed [12:0] adc_offset_ch1;
    reg               cal_done;
    reg               outlier_rejected;

    // Live ADC captured registers
    reg [11:0] live_raw_ch0;
    reg [11:0] live_raw_ch1;
    reg [15:0] meas_timer;

    // Signed sample wires for comparison
    wire signed [12:0] s13_ch0 = $signed({1'b0, adc_data_ch0});
    wire signed [12:0] s13_ch1 = $signed({1'b0, adc_data_ch1});
    wire signed [13:0] cal_ch0_diff = s13_ch0 - cal_mean_ch0;
    wire signed [13:0] cal_ch1_diff = s13_ch1 - cal_mean_ch1;

    // 5-second countdown signal for OLED display
    wire [2:0] cal_countdown = SIMULATION ? 3'd1 :
                               (warmup_cnt < 29'd100_000_000) ? 3'd5 :
                               (warmup_cnt < 29'd200_000_000) ? 3'd4 :
                               (warmup_cnt < 29'd300_000_000) ? 3'd3 :
                               (warmup_cnt < 29'd400_000_000) ? 3'd2 : 3'd1;

    // Forward declaration of filter registers for seeding in S_CAL_DONE
    reg signed [20:0] filt_q8_ch0;
    reg signed [20:0] filt_q8_ch1;
    reg signed [15:0] diff_reg_ch0, diff_reg_ch1;
    reg [11:0]        filt_u12_reg_0, filt_u12_reg_1;
    reg [23:0]        v_calc_reg_0, v_calc_reg_1;
    reg signed [31:0] i_calc_reg_0, i_calc_reg_1;
    reg [11:0]        v_mv_ch0_r, v_mv_ch1_r;
    reg signed [15:0] i_ma_ch0_r, i_ma_ch1_r;
    reg [1:0]         pipe_valid;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            state            <= S_IDLE;
            adc_start        <= 1'b0;
            warmup_cnt       <= 29'd0;
            cal_sample_cnt   <= 16'd0;
            cal_valid_cnt    <= 16'd0;
            cal_accum_ch0    <= 24'sd0;
            cal_accum_ch1    <= 24'sd0;
            cal_mean_ch0     <= 13'sd2048;
            cal_mean_ch1     <= 13'sd2048;
            adc_offset_ch0   <= 13'sd2048;
            adc_offset_ch1   <= 13'sd2048;
            cal_done         <= 1'b0;
            outlier_rejected <= 1'b0;
            live_raw_ch0     <= 12'd2048;
            live_raw_ch1     <= 12'd2048;
            meas_timer       <= 16'd0;
        end else begin
            adc_start <= 1'b0;

            // Manual re-tare button triggers fresh calibration from any operational state
            if (btnc_pulse && state != S_CAL_WARMUP && state != S_CAL_ADC && 
                state != S_CAL_WAIT && state != S_CAL_ACCUM && state != S_CAL_DELAY) begin
                cal_done         <= 1'b0;
                warmup_cnt       <= 29'd0;
                cal_sample_cnt   <= 16'd0;
                cal_valid_cnt    <= 16'd0;
                cal_accum_ch0    <= 24'sd0;
                cal_accum_ch1    <= 24'sd0;
                cal_mean_ch0     <= 13'sd2048;
                cal_mean_ch1     <= 13'sd2048;
                meas_timer       <= 16'd0;
                state            <= S_CAL_WARMUP;
            end else begin
                case (state)
                    S_IDLE: begin
                        if (!cal_done)
                            state <= S_CAL_WARMUP;
                        else
                            state <= S_MEASURE;
                    end

                    S_CAL_WARMUP: begin
                        // 5-second settling time for ACS712 and power supplies
                        if (warmup_cnt >= WARMUP_CYCLES) begin
                            warmup_cnt     <= 29'd0;
                            cal_sample_cnt <= 16'd0;
                            cal_valid_cnt  <= 16'd0;
                            cal_accum_ch0  <= 24'sd0;
                            cal_accum_ch1  <= 24'sd0;
                            cal_mean_ch0   <= 13'sd2048;
                            cal_mean_ch1   <= 13'sd2048;
                            meas_timer     <= 16'd0;
                            state          <= S_CAL_ADC;
                        end else begin
                            warmup_cnt <= warmup_cnt + 29'd1;
                        end
                    end

                    S_CAL_ADC: begin
                        adc_start <= 1'b1;
                        state     <= S_CAL_WAIT;
                    end

                    S_CAL_WAIT: begin
                        if (adc_done) begin
                            state <= S_CAL_ACCUM;
                        end
                    end

                    S_CAL_ACCUM: begin
                        cal_sample_cnt <= cal_sample_cnt + 16'd1;

                        // Outlier rejection: accept sample if within +/-100 LSBs of running mean
                        // On first sample (cal_valid_cnt == 0), seed cal_mean directly
                        if (cal_valid_cnt == 16'd0) begin
                            cal_mean_ch0     <= s13_ch0;
                            cal_mean_ch1     <= s13_ch1;
                            cal_accum_ch0    <= s13_ch0;
                            cal_accum_ch1    <= s13_ch1;
                            cal_valid_cnt    <= 16'd1;
                            outlier_rejected <= 1'b0;
                            meas_timer       <= 16'd0;
                            state            <= S_CAL_DELAY;
                        end else if ((s13_ch0 > (cal_mean_ch0 - 13'sd100)) &&
                                     (s13_ch0 < (cal_mean_ch0 + 13'sd100))) begin
                            cal_accum_ch0    <= cal_accum_ch0 + s13_ch0;
                            cal_accum_ch1    <= cal_accum_ch1 + s13_ch1;
                            cal_valid_cnt    <= cal_valid_cnt + 16'd1;
                            cal_mean_ch0     <= cal_mean_ch0 + (cal_ch0_diff >>> 4);
                            cal_mean_ch1     <= cal_mean_ch1 + (cal_ch1_diff >>> 4);
                            outlier_rejected <= 1'b0;

                            if (cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES - 1) begin
                                state <= S_CAL_DONE;
                            end else begin
                                meas_timer <= 16'd0;
                                state      <= S_CAL_DELAY;
                            end
                        end else begin
                            // Outlier sample rejected!
                            outlier_rejected <= 1'b1;

                            if (cal_sample_cnt >= 16'd1023) begin
                                state <= S_CAL_DONE;
                            end else begin
                                meas_timer <= 16'd0;
                                state      <= S_CAL_DELAY;
                            end
                        end
                    end

                    S_CAL_DELAY: begin
                        // Enforce exact 10 kHz sample period (100 us) between calibration samples
                        // Matches acquisition time of S_MEASURE to ensure zero dynamic offset bias
                        if (meas_timer >= SAMPLE_PERIOD - 1) begin
                            meas_timer <= 16'd0;
                            adc_start  <= 1'b1;
                            state      <= S_CAL_WAIT;
                        end else begin
                            meas_timer <= meas_timer + 16'd1;
                        end
                    end

                    S_CAL_DONE: begin
                        // Exactly 256 samples -> divide by 256 via 8-bit shift with rounding (+128)
                        if (cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) begin
                            adc_offset_ch0 <= (cal_accum_ch0 + 24'sd128) >>> 8;
                            adc_offset_ch1 <= (cal_accum_ch1 + 24'sd128) >>> 8;
                        end else begin
                            adc_offset_ch0 <= cal_mean_ch0;
                            adc_offset_ch1 <= cal_mean_ch1;
                        end
                        cal_done   <= 1'b1;
                        meas_timer <= 16'd0;
                        state      <= S_MEASURE;
                    end

                    S_MEASURE: begin
                        // Periodic ADC sampling at 10 kHz (100 us)
                        if (meas_timer >= SAMPLE_PERIOD - 1) begin
                            meas_timer <= 16'd0;
                            adc_start  <= 1'b1;
                            state      <= S_WAIT_MEAS;
                        end else begin
                            meas_timer <= meas_timer + 16'd1;
                        end
                    end

                    S_WAIT_MEAS: begin
                        if (adc_done) begin
                            live_raw_ch0 <= adc_data_ch0;
                            live_raw_ch1 <= adc_data_ch1;
                            state        <= S_MEASURE;
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // 6. High-Precision Q12.8 IIR Smoothing Filter & Multi-Stage DSP Pipeline
    //    - Q12.8 format prevents integer truncation asymmetry that biased old filter
    //    - Filter is seeded directly with calibrated offset on S_CAL_DONE for 0A start
    //    - Dedicated pipeline registers guarantee complete setup timing closure @ 100 MHz
    // -------------------------------------------------------------------------
    wire signed [20:0] adc_in_q8_0 = $signed({1'b0, adc_data_ch0, 8'b0000_0000});
    wire signed [20:0] adc_in_q8_1 = $signed({1'b0, adc_data_ch1, 8'b0000_0000});

    // Rounded integer filtered values
    wire signed [12:0] filt_int_0 = (filt_q8_ch0 + 21'sd128) >>> 8;
    wire signed [12:0] filt_int_1 = (filt_q8_ch1 + 21'sd128) >>> 8;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            filt_q8_ch0     <= 21'sd2048 <<< 8;
            filt_q8_ch1     <= 21'sd2048 <<< 8;
            diff_reg_ch0    <= 16'sd0;
            diff_reg_ch1    <= 16'sd0;
            filt_u12_reg_0  <= 12'd2048;
            filt_u12_reg_1  <= 12'd2048;
            v_calc_reg_0    <= 24'd0;
            v_calc_reg_1    <= 24'd0;
            i_calc_reg_0    <= 32'sd0;
            i_calc_reg_1    <= 32'sd0;
            v_mv_ch0_r      <= 12'd1650;
            v_mv_ch1_r      <= 12'd1650;
            i_ma_ch0_r      <= 16'sd0;
            i_ma_ch1_r      <= 16'sd0;
            pipe_valid      <= 2'b00;
        end else if (state == S_CAL_DONE) begin
            // Immediately seed filter to exact calibrated offset so initial reading is exactly 0.00A
            filt_q8_ch0     <= ((cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) ? 
                                ((cal_accum_ch0 + 24'sd128) >>> 8) : cal_mean_ch0) <<< 8;
            filt_q8_ch1     <= ((cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) ? 
                                ((cal_accum_ch1 + 24'sd128) >>> 8) : cal_mean_ch1) <<< 8;
            diff_reg_ch0    <= 16'sd0;
            diff_reg_ch1    <= 16'sd0;
            filt_u12_reg_0  <= ((cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) ? 
                                ((cal_accum_ch0 + 24'sd128) >>> 8) : cal_mean_ch0);
            filt_u12_reg_1  <= ((cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) ? 
                                ((cal_accum_ch1 + 24'sd128) >>> 8) : cal_mean_ch1);
            v_calc_reg_0    <= 24'd0;
            v_calc_reg_1    <= 24'd0;
            i_calc_reg_0    <= 32'sd0;
            i_calc_reg_1    <= 32'sd0;
            v_mv_ch0_r      <= 12'd1650;
            v_mv_ch1_r      <= 12'd1650;
            i_ma_ch0_r      <= 16'sd0;
            i_ma_ch1_r      <= 16'sd0;
            pipe_valid      <= 2'b00;
        end else begin
            // Pipeline Stage 1: Update IIR filter on ADC sample completion
            if (adc_done && state == S_WAIT_MEAS) begin
                filt_q8_ch0    <= filt_q8_ch0 + ((adc_in_q8_0 - filt_q8_ch0) >>> 3);
                filt_q8_ch1    <= filt_q8_ch1 + ((adc_in_q8_1 - filt_q8_ch1) >>> 3);
                pipe_valid[0]  <= 1'b1;
            end else begin
                pipe_valid[0]  <= 1'b0;
            end

            // Pipeline Stage 2: Register subtraction and clamp
            if (pipe_valid[0]) begin
                diff_reg_ch0   <= filt_int_0 - adc_offset_ch0;
                diff_reg_ch1   <= filt_int_1 - adc_offset_ch1;
                filt_u12_reg_0 <= (filt_int_0 < 0) ? 12'd0 : (filt_int_0 > 4095) ? 12'd4095 : filt_int_0[11:0];
                filt_u12_reg_1 <= (filt_int_1 < 0) ? 12'd0 : (filt_int_1 > 4095) ? 12'd4095 : filt_int_1[11:0];
                pipe_valid[1]  <= 1'b1;
            end else begin
                pipe_valid[1]  <= 1'b0;
            end

            // Pipeline Stage 3: DSP Multipliers
            if (pipe_valid[1]) begin
                v_calc_reg_0 <= filt_u12_reg_0 * 24'd3300 + 24'd2048;
                v_calc_reg_1 <= filt_u12_reg_1 * 24'd3300 + 24'd2048;
                i_calc_reg_0 <= diff_reg_ch0 * 32'sd33000 + ((diff_reg_ch0 >= 0) ? 32'sd1024 : -32'sd1024);
                i_calc_reg_1 <= diff_reg_ch1 * 32'sd33000 + ((diff_reg_ch1 >= 0) ? 32'sd1024 : -32'sd1024);

                // Stage 4: Extract shifted physical outputs
                v_mv_ch0_r   <= (filt_u12_reg_0 * 24'd3300 + 24'd2048) >> 12;
                v_mv_ch1_r   <= (filt_u12_reg_1 * 24'd3300 + 24'd2048) >> 12;
                i_ma_ch0_r   <= (diff_reg_ch0 * 32'sd33000 + ((diff_reg_ch0 >= 0) ? 32'sd1024 : -32'sd1024)) >>> 11;
                i_ma_ch1_r   <= (diff_reg_ch1 * 32'sd33000 + ((diff_reg_ch1 >= 0) ? 32'sd1024 : -32'sd1024)) >>> 11;
            end
        end
    end

    // Direct wires for top-level access / testbench compatibility
    wire signed [15:0] diff_ch0 = diff_reg_ch0;
    wire signed [15:0] diff_ch1 = diff_reg_ch1;
    wire [11:0]        v_mv_ch0 = v_mv_ch0_r;
    wire [11:0]        v_mv_ch1 = v_mv_ch1_r;
    wire signed [15:0] i_ma_ch0 = i_ma_ch0_r;
    wire signed [15:0] i_ma_ch1 = i_ma_ch1_r;

    // Overcurrent Detection: > 5.0 Amperes (|diff| > 310 counts)
    wire overcurrent_ch0 = (diff_reg_ch0 > `MAX_CURRENT_DELTA) || (diff_reg_ch0 < -`MAX_CURRENT_DELTA);
    wire overcurrent_ch1 = (diff_reg_ch1 > `MAX_CURRENT_DELTA) || (diff_reg_ch1 < -`MAX_CURRENT_DELTA);
    wire overcurrent_active = overcurrent_ch0 || overcurrent_ch1;

    // -------------------------------------------------------------------------
    // 7. 1 Hz Diagnostic Heartbeat Generator
    // -------------------------------------------------------------------------
    localparam CNT_1HZ_MAX = SIMULATION ? 26'd200 : 26'd50_000_000;
    reg [25:0] cnt_1hz;
    reg        heartbeat_1hz;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1hz       <= 26'd0;
            heartbeat_1hz <= 1'b0;
        end else if (cnt_1hz >= CNT_1HZ_MAX - 1) begin
            cnt_1hz       <= 26'd0;
            heartbeat_1hz <= ~heartbeat_1hz;
        end else begin
            cnt_1hz <= cnt_1hz + 26'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 8. Multi-Stage Pipelined Decimal Digit Formatter (40 Hz / 25 ms period)
    //    Distributes division/modulo operations across 7 non-overlapping clock cycles.
    //    Eliminates chained arithmetic to achieve clean positive timing slack.
    // -------------------------------------------------------------------------
    reg [21:0] fmt_timer;
    reg [3:0]  fmt_step;

    // Snapshot registers
    reg [11:0]        snap_v0, snap_v1;
    reg signed [15:0] snap_i0, snap_i1;
    reg [11:0]        snap_raw0, snap_raw1;
    reg signed [12:0] snap_off0;
    reg [8:0]         snap_valid_cnt;
    reg signed [12:0] snap_mean;
    reg [2:0]         snap_countdown;
    reg               snap_cal_done;
    reg [3:0]         snap_state;
    reg               snap_mode_high;

    // Intermediate unsigned values & signs (sized to eliminate wide division trees)
    reg        i0_neg, i1_neg;
    reg [14:0] i0_abs, i1_abs;
    reg [11:0] off0_abs;
    reg [11:0] mean_abs;

    // Voltage Digits: Units, Tenths, Hundredths, Thousandths (e.g. 1 . 6 5 0 V)
    reg [3:0] v0_d3, v0_d2, v0_d1, v0_d0;
    reg [3:0] v1_d3, v1_d2, v1_d1, v1_d0;
    reg [9:0] v0_rem_th;
    reg [6:0] v0_rem_h;
    reg [9:0] v1_rem_th;
    reg [6:0] v1_rem_h;

    // Current Digits (in 0.01A = centiamperes): e.g. + 1 . 6 1 A (max 999 = 9.99A)
    reg [9:0] i0_ca, i1_ca;
    reg [3:0] i0_w, i0_t, i0_h; // whole, tenths, hundredths
    reg [3:0] i1_w, i1_t, i1_h;
    reg [6:0] i0_rem_ca, i1_rem_ca;

    // Raw and Offset Digits: e.g. 2048
    reg [3:0] raw0_d3, raw0_d2, raw0_d1, raw0_d0;
    reg [3:0] off0_d3, off0_d2, off0_d1, off0_d0;
    reg [9:0] raw0_rem_th;
    reg [6:0] raw0_rem_h;
    reg [9:0] off0_rem_th;
    reg [6:0] off0_rem_h;

    // Calibration Progress Digits
    reg [3:0] valid_d2, valid_d1, valid_d0;
    reg [6:0] valid_rem_h;
    reg [3:0] mean_d3, mean_d2, mean_d1, mean_d0;
    reg [9:0] mean_rem_th;
    reg [6:0] mean_rem_h;

    // 64-Character OLED Display Buffer (4 lines x 16 characters)
    reg [7:0] oled_chars [0:63];

    integer k;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            fmt_timer      <= 22'd0;
            fmt_step       <= 4'd0;
            snap_v0        <= 12'd0;
            snap_v1        <= 12'd0;
            snap_i0        <= 16'sd0;
            snap_i1        <= 16'sd0;
            snap_raw0      <= 12'd0;
            snap_raw1      <= 12'd0;
            snap_off0      <= 13'sd2048;
            snap_valid_cnt <= 16'd0;
            snap_mean      <= 13'sd2048;
            snap_countdown <= 3'd5;
            snap_cal_done  <= 1'b0;
            snap_state     <= S_IDLE;
            snap_mode_high <= 1'b0;
            i0_neg         <= 1'b0;
            i1_neg         <= 1'b0;
            i0_abs         <= 15'd0;
            i1_abs         <= 15'd0;
            off0_abs       <= 12'd2048;
            mean_abs       <= 12'd2048;
            i0_ca          <= 10'd0;
            i1_ca          <= 10'd0;
            i0_w           <= 4'd0;
            i0_t           <= 4'd0;
            i0_h           <= 4'd0;
            i1_w           <= 4'd0;
            i1_t           <= 4'd0;
            i1_h           <= 4'd0;
            i0_rem_ca      <= 7'd0;
            i1_rem_ca      <= 7'd0;
            v0_d3          <= 4'd0;
            v0_d2          <= 4'd0;
            v0_d1          <= 4'd0;
            v0_d0          <= 4'd0;
            v1_d3          <= 4'd0;
            v1_d2          <= 4'd0;
            v1_d1          <= 4'd0;
            v1_d0          <= 4'd0;
            v0_rem_th      <= 10'd0;
            v0_rem_h       <= 7'd0;
            v1_rem_th      <= 10'd0;
            v1_rem_h       <= 7'd0;
            raw0_d3        <= 4'd0;
            raw0_d2        <= 4'd0;
            raw0_d1        <= 4'd0;
            raw0_d0        <= 4'd0;
            off0_d3        <= 4'd0;
            off0_d2        <= 4'd0;
            off0_d1        <= 4'd0;
            off0_d0        <= 4'd0;
            raw0_rem_th    <= 10'd0;
            raw0_rem_h     <= 7'd0;
            off0_rem_th    <= 10'd0;
            off0_rem_h     <= 7'd0;
            valid_d2       <= 4'd0;
            valid_d1       <= 4'd0;
            valid_d0       <= 4'd0;
            valid_rem_h    <= 7'd0;
            mean_d3        <= 4'd0;
            mean_d2        <= 4'd0;
            mean_d1        <= 4'd0;
            mean_d0        <= 4'd0;
            mean_rem_th    <= 10'd0;
            mean_rem_h     <= 7'd0;

            for (k = 0; k < 64; k = k + 1)
                oled_chars[k] <= 8'h20; // Space
        end else begin
            case (fmt_step)
                4'd0: begin
                    // Trigger new display frame conversion at 40 Hz
                    if (fmt_timer >= FMT_PERIOD - 1) begin
                        fmt_timer      <= 22'd0;
                        snap_v0        <= v_mv_ch0_r;
                        snap_v1        <= v_mv_ch1_r;
                        snap_i0        <= i_ma_ch0_r;
                        snap_i1        <= i_ma_ch1_r;
                        snap_raw0      <= live_raw_ch0;
                        snap_raw1      <= live_raw_ch1;
                        snap_off0      <= adc_offset_ch0;
                        snap_valid_cnt <= cal_valid_cnt;
                        snap_mean      <= cal_mean_ch0;
                        snap_countdown <= cal_countdown;
                        snap_cal_done  <= cal_done;
                        snap_state     <= state;
                        snap_mode_high <= mode_high_prec;

                        // Signs and Absolute Values from registered DSP outputs
                        i0_neg   <= (i_ma_ch0_r < 0);
                        i0_abs   <= (i_ma_ch0_r < 0) ? -i_ma_ch0_r : i_ma_ch0_r;
                        i1_neg   <= (i_ma_ch1_r < 0);
                        i1_abs   <= (i_ma_ch1_r < 0) ? -i_ma_ch1_r : i_ma_ch1_r;
                        off0_abs <= (adc_offset_ch0 < 0) ? -adc_offset_ch0 : adc_offset_ch0;
                        mean_abs <= (cal_mean_ch0 < 0) ? -cal_mean_ch0 : cal_mean_ch0;

                        fmt_step <= 4'd1;
                    end else begin
                        fmt_timer <= fmt_timer + 22'd1;
                    end
                end

                4'd1: begin
                    // Step 1: Centiamperes (0.01A units, max 9.99A) and Voltage thousands
                    i0_ca       <= ((i0_abs + 15'd5) / 15'd10 > 10'd999) ? 10'd999 : ((i0_abs + 15'd5) / 15'd10);
                    i1_ca       <= ((i1_abs + 15'd5) / 15'd10 > 10'd999) ? 10'd999 : ((i1_abs + 15'd5) / 15'd10);

                    v0_d3       <= snap_v0 / 1000;
                    v0_rem_th   <= snap_v0 % 1000;
                    v1_d3       <= snap_v1 / 1000;
                    v1_rem_th   <= snap_v1 % 1000;

                    fmt_step    <= 4'd2;
                end

                4'd2: begin
                    // Step 2: Raw, Offset, Mean thousands and Valid count hundreds
                    raw0_d3     <= snap_raw0 / 1000;
                    raw0_rem_th <= snap_raw0 % 1000;

                    off0_d3     <= off0_abs / 1000;
                    off0_rem_th <= off0_abs % 1000;

                    mean_d3     <= mean_abs / 1000;
                    mean_rem_th <= mean_abs % 1000;

                    // Fast bounded sample count hundreds (max 256)
                    valid_d2    <= (snap_valid_cnt >= 9'd200) ? 4'd2 : (snap_valid_cnt >= 9'd100) ? 4'd1 : 4'd0;
                    valid_rem_h <= (snap_valid_cnt >= 9'd200) ? (snap_valid_cnt - 9'd200) : 
                                   (snap_valid_cnt >= 9'd100) ? (snap_valid_cnt - 9'd100) : snap_valid_cnt[6:0];

                    fmt_step    <= 4'd3;
                end

                4'd3: begin
                    // Step 3: Voltage hundreds and Current whole/remainders
                    v0_d2     <= v0_rem_th / 100;
                    v0_rem_h  <= v0_rem_th % 100;
                    v1_d2     <= v1_rem_th / 100;
                    v1_rem_h  <= v1_rem_th % 100;

                    i0_w      <= i0_ca / 100;
                    i0_rem_ca <= i0_ca % 100;
                    i1_w      <= i1_ca / 100;
                    i1_rem_ca <= i1_ca % 100;

                    fmt_step  <= 4'd4;
                end

                4'd4: begin
                    // Step 4: Raw, Offset, Mean hundreds and Valid count tens/units
                    raw0_d2    <= raw0_rem_th / 100;
                    raw0_rem_h <= raw0_rem_th % 100;

                    off0_d2    <= off0_rem_th / 100;
                    off0_rem_h <= off0_rem_th % 100;

                    mean_d2    <= mean_rem_th / 100;
                    mean_rem_h <= mean_rem_th % 100;

                    valid_d1   <= valid_rem_h / 10;
                    valid_d0   <= valid_rem_h % 10;

                    fmt_step   <= 4'd5;
                end

                4'd5: begin
                    // Step 5: Tens and units for voltages, codes, and current
                    v0_d1 <= v0_rem_h / 10;
                    v0_d0 <= v0_rem_h % 10;
                    v1_d1 <= v1_rem_h / 10;
                    v1_d0 <= v1_rem_h % 10;

                    raw0_d1 <= raw0_rem_h / 10;
                    raw0_d0 <= raw0_rem_h % 10;

                    off0_d1 <= off0_rem_h / 10;
                    off0_d0 <= off0_rem_h % 10;

                    mean_d1 <= mean_rem_h / 10;
                    mean_d0 <= mean_rem_h % 10;

                    i0_t <= i0_rem_ca / 10;
                    i0_h <= i0_rem_ca % 10;
                    i1_t <= i1_rem_ca / 10;
                    i1_h <= i1_rem_ca % 10;

                    fmt_step <= 4'd6;
                end

                4'd6: begin
                    // Step 6: Render Text Buffer (4 Lines x 16 Chars = 64 Chars)
                    if (!snap_cal_done && snap_state == S_CAL_WARMUP) begin
                        // Screen 1: Auto-Tare Warmup Countdown
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "U"; oled_chars[2]  <= "T"; oled_chars[3]  <= "O";
                        oled_chars[4]  <= "T"; oled_chars[5]  <= "A"; oled_chars[6]  <= "R"; oled_chars[7]  <= "E";
                        oled_chars[8]  <= " "; oled_chars[9]  <= "C"; oled_chars[10] <= "A"; oled_chars[11] <= "L";
                        oled_chars[12] <= "I"; oled_chars[13] <= "B"; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        oled_chars[16] <= "W"; oled_chars[17] <= "A"; oled_chars[18] <= "R"; oled_chars[19] <= "M";
                        oled_chars[20] <= "I"; oled_chars[21] <= "N"; oled_chars[22] <= "G"; oled_chars[23] <= " ";
                        oled_chars[24] <= "U"; oled_chars[25] <= "P"; oled_chars[26] <= ":"; oled_chars[27] <= " ";
                        oled_chars[28] <= " ";
                        oled_chars[29] <= 8'h30 + {5'd0, snap_countdown};
                        oled_chars[30] <= "s"; oled_chars[31] <= " ";

                        oled_chars[32] <= "S"; oled_chars[33] <= "A"; oled_chars[34] <= "M"; oled_chars[35] <= "P";
                        oled_chars[36] <= "L"; oled_chars[37] <= "E"; oled_chars[38] <= "S"; oled_chars[39] <= ":";
                        oled_chars[40] <= " "; oled_chars[41] <= "0"; oled_chars[42] <= "0"; oled_chars[43] <= "0";
                        oled_chars[44] <= "/"; oled_chars[45] <= "2"; oled_chars[46] <= "5"; oled_chars[47] <= "6";

                        oled_chars[48] <= "O"; oled_chars[49] <= "F"; oled_chars[50] <= "F"; oled_chars[51] <= "S";
                        oled_chars[52] <= "E"; oled_chars[53] <= "T"; oled_chars[54] <= ":"; oled_chars[55] <= " ";
                        oled_chars[56] <= "C"; oled_chars[57] <= "A"; oled_chars[58] <= "L"; oled_chars[59] <= "C";
                        oled_chars[60] <= "."; oled_chars[61] <= "."; oled_chars[62] <= "."; oled_chars[63] <= " ";

                    end else if (!snap_cal_done) begin
                        // Screen 2: Auto-Tare Sampling in Progress
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "U"; oled_chars[2]  <= "T"; oled_chars[3]  <= "O";
                        oled_chars[4]  <= "T"; oled_chars[5]  <= "A"; oled_chars[6]  <= "R"; oled_chars[7]  <= "E";
                        oled_chars[8]  <= " "; oled_chars[9]  <= "C"; oled_chars[10] <= "A"; oled_chars[11] <= "L";
                        oled_chars[12] <= "I"; oled_chars[13] <= "B"; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        oled_chars[16] <= "S"; oled_chars[17] <= "A"; oled_chars[18] <= "M"; oled_chars[19] <= "P";
                        oled_chars[20] <= "L"; oled_chars[21] <= "I"; oled_chars[22] <= "N"; oled_chars[23] <= "G";
                        oled_chars[24] <= " "; oled_chars[25] <= "A"; oled_chars[26] <= "D"; oled_chars[27] <= "C";
                        oled_chars[28] <= "."; oled_chars[29] <= "."; oled_chars[30] <= "."; oled_chars[31] <= " ";

                        oled_chars[32] <= "S"; oled_chars[33] <= "A"; oled_chars[34] <= "M"; oled_chars[35] <= "P";
                        oled_chars[36] <= "L"; oled_chars[37] <= "E"; oled_chars[38] <= "S"; oled_chars[39] <= ":";
                        oled_chars[40] <= " ";
                        oled_chars[41] <= 8'h30 + {4'd0, valid_d2};
                        oled_chars[42] <= 8'h30 + {4'd0, valid_d1};
                        oled_chars[43] <= 8'h30 + {4'd0, valid_d0};
                        oled_chars[44] <= "/"; oled_chars[45] <= "2"; oled_chars[46] <= "5"; oled_chars[47] <= "6";

                        oled_chars[48] <= "M"; oled_chars[49] <= "E"; oled_chars[50] <= "A"; oled_chars[51] <= "N";
                        oled_chars[52] <= ":"; oled_chars[53] <= " "; oled_chars[54] <= " "; oled_chars[55] <= " ";
                        oled_chars[56] <= " ";
                        oled_chars[57] <= 8'h30 + {4'd0, mean_d3};
                        oled_chars[58] <= 8'h30 + {4'd0, mean_d2};
                        oled_chars[59] <= 8'h30 + {4'd0, mean_d1};
                        oled_chars[60] <= 8'h30 + {4'd0, mean_d0};
                        oled_chars[61] <= " "; oled_chars[62] <= " "; oled_chars[63] <= " ";

                    end else if (!snap_mode_high) begin
                        // Screen 3: Default Dual Voltage + Current View (SW0 = 0)
                        oled_chars[0]  <= "D"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "V";
                        oled_chars[4]  <= "O"; oled_chars[5]  <= "L"; oled_chars[6]  <= "T"; oled_chars[7]  <= " ";
                        oled_chars[8]  <= "&"; oled_chars[9]  <= " "; oled_chars[10] <= "C"; oled_chars[11] <= "U";
                        oled_chars[12] <= "R"; oled_chars[13] <= "R"; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        oled_chars[16] <= "0"; oled_chars[17] <= ":"; oled_chars[18] <= " ";
                        oled_chars[19] <= 8'h30 + {4'd0, v0_d3};
                        oled_chars[20] <= ".";
                        oled_chars[21] <= 8'h30 + {4'd0, v0_d2};
                        oled_chars[22] <= 8'h30 + {4'd0, v0_d1};
                        oled_chars[23] <= "V";
                        oled_chars[24] <= " "; oled_chars[25] <= " ";
                        oled_chars[26] <= i0_neg ? "-" : "+";
                        oled_chars[27] <= 8'h30 + {4'd0, i0_w};
                        oled_chars[28] <= ".";
                        oled_chars[29] <= 8'h30 + {4'd0, i0_t};
                        oled_chars[30] <= 8'h30 + {4'd0, i0_h};
                        oled_chars[31] <= "A";

                        oled_chars[32] <= "1"; oled_chars[33] <= ":"; oled_chars[34] <= " ";
                        oled_chars[35] <= 8'h30 + {4'd0, v1_d3};
                        oled_chars[36] <= ".";
                        oled_chars[37] <= 8'h30 + {4'd0, v1_d2};
                        oled_chars[38] <= 8'h30 + {4'd0, v1_d1};
                        oled_chars[39] <= "V";
                        oled_chars[40] <= " "; oled_chars[41] <= " ";
                        oled_chars[42] <= i1_neg ? "-" : "+";
                        oled_chars[43] <= 8'h30 + {4'd0, i1_w};
                        oled_chars[44] <= ".";
                        oled_chars[45] <= 8'h30 + {4'd0, i1_t};
                        oled_chars[46] <= 8'h30 + {4'd0, i1_h};
                        oled_chars[47] <= "A";

                        oled_chars[48] <= "R"; oled_chars[49] <= "0"; oled_chars[50] <= ":";
                        oled_chars[51] <= 8'h30 + {4'd0, raw0_d3};
                        oled_chars[52] <= 8'h30 + {4'd0, raw0_d2};
                        oled_chars[53] <= 8'h30 + {4'd0, raw0_d1};
                        oled_chars[54] <= 8'h30 + {4'd0, raw0_d0};
                        oled_chars[55] <= " "; oled_chars[56] <= " ";
                        oled_chars[57] <= "O"; oled_chars[58] <= "0"; oled_chars[59] <= ":";
                        oled_chars[60] <= 8'h30 + {4'd0, off0_d3};
                        oled_chars[61] <= 8'h30 + {4'd0, off0_d2};
                        oled_chars[62] <= 8'h30 + {4'd0, off0_d1};
                        oled_chars[63] <= 8'h30 + {4'd0, off0_d0};

                    end else begin
                        // Screen 4: High-Precision Voltage View (SW0 = 1)
                        oled_chars[0]  <= "D"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "V";
                        oled_chars[4]  <= "O"; oled_chars[5]  <= "L"; oled_chars[6]  <= "T"; oled_chars[7]  <= "A";
                        oled_chars[8]  <= "G"; oled_chars[9]  <= "E"; oled_chars[10] <= " "; oled_chars[11] <= "D";
                        oled_chars[12] <= "E"; oled_chars[13] <= "T"; oled_chars[14] <= "."; oled_chars[15] <= " ";

                        oled_chars[16] <= "C"; oled_chars[17] <= "H"; oled_chars[18] <= "0"; oled_chars[19] <= ":";
                        oled_chars[20] <= " ";
                        oled_chars[21] <= 8'h30 + {4'd0, v0_d3};
                        oled_chars[22] <= ".";
                        oled_chars[23] <= 8'h30 + {4'd0, v0_d2};
                        oled_chars[24] <= 8'h30 + {4'd0, v0_d1};
                        oled_chars[25] <= 8'h30 + {4'd0, v0_d0};
                        oled_chars[26] <= " "; oled_chars[27] <= "V"; oled_chars[28] <= " ";
                        oled_chars[29] <= " "; oled_chars[30] <= " "; oled_chars[31] <= " ";

                        oled_chars[32] <= "C"; oled_chars[33] <= "H"; oled_chars[34] <= "1"; oled_chars[35] <= ":";
                        oled_chars[36] <= " ";
                        oled_chars[37] <= 8'h30 + {4'd0, v1_d3};
                        oled_chars[38] <= ".";
                        oled_chars[39] <= 8'h30 + {4'd0, v1_d2};
                        oled_chars[40] <= 8'h30 + {4'd0, v1_d1};
                        oled_chars[41] <= 8'h30 + {4'd0, v1_d0};
                        oled_chars[42] <= " "; oled_chars[43] <= "V"; oled_chars[44] <= " ";
                        oled_chars[45] <= " "; oled_chars[46] <= " "; oled_chars[47] <= " ";

                        oled_chars[48] <= "O"; oled_chars[49] <= "F"; oled_chars[50] <= "F"; oled_chars[51] <= "S";
                        oled_chars[52] <= "E"; oled_chars[53] <= "T"; oled_chars[54] <= ":"; oled_chars[55] <= " ";
                        oled_chars[56] <= 8'h30 + {4'd0, off0_d3};
                        oled_chars[57] <= 8'h30 + {4'd0, off0_d2};
                        oled_chars[58] <= 8'h30 + {4'd0, off0_d1};
                        oled_chars[59] <= 8'h30 + {4'd0, off0_d0};
                        oled_chars[60] <= " "; oled_chars[61] <= "L"; oled_chars[62] <= "S"; oled_chars[63] <= "B";
                    end

                    fmt_step <= 4'd0;
                end

                default: fmt_step <= 4'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 9. Direct Memory Read Interface to OLED Controller
    // -------------------------------------------------------------------------
    wire [5:0] text_addr_w;
    wire [7:0] text_char_w = oled_chars[text_addr_w];
    wire       init_done_w;

    // -------------------------------------------------------------------------
    // 10. SSD1306 OLED Controller Instance
    // -------------------------------------------------------------------------
    oled_controller #(
        .SIMULATION (SIMULATION)
    ) u_oled (
        .clk        (sys_clk_i),
        .rst_n      (rst_n),
        .text_addr  (text_addr_w),
        .text_char  (text_char_w),
        .init_done  (init_done_w),
        .oled_sclk  (oled_sclk),
        .oled_sdin  (oled_sdin),
        .oled_dc    (oled_dc),
        .oled_res   (oled_res),
        .oled_vbat  (oled_vbat),
        .oled_vdd   (oled_vdd)
    );

    // -------------------------------------------------------------------------
    // 11. Diagnostic LEDs Mapping
    // -------------------------------------------------------------------------
    assign led[0] = heartbeat_1hz;                       // LD0: 1 Hz System Heartbeat
    assign led[1] = init_done_w;                         // LD1: OLED Panel Initialized (Solid ON)
    assign led[2] = cal_done;                            // LD2: Auto-Tare Calibration Complete
    assign led[3] = (state == S_WAIT_MEAS || state == S_CAL_WAIT); // LD3: ADC Conversion Active
    assign led[4] = overcurrent_active;                  // LD4: Overcurrent Alert (>5.0A)
    assign led[5] = outlier_rejected;                    // LD5: Outlier Sample Rejected Flag
    assign led[6] = (diff_reg_ch0 > 0);                  // LD6: CH0 Current Polarity (+: ON, -: OFF)
    assign led[7] = (diff_reg_ch1 > 0);                  // LD7: CH1 Current Polarity (+: ON, -: OFF)

endmodule
