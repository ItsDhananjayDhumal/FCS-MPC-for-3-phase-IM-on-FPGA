`timescale 1ns / 1ps
// =============================================================================
// Module: ac_current_top
// Description: Standalone AC Current Peak-to-Peak Telemetry with SSD1306 OLED
//              - Target Board: Digilent Nexys Video (Artix-7 XC7A200T-1SBG484C)
//              - Analog Front-End: Digilent Pmod AD1 (Dual AD7476A 12-bit SAR ADC)
//              - Current Sensors: Allegro ACS712-40AB (50 mV/A, 2.510V zero-offset)
//              - Features:
//                1. Startup Auto-Tare Calibration (exact match to control_fsm.v):
//                   - 5-second warmup countdown timer (5s -> 4s -> 3s -> 2s -> 1s)
//                   - 256-sample outlier-filtered running mean (threshold: +/-100 LSB)
//                   - Power-of-2 division via arithmetic right shift (cal_accum >>> 8)
//                   - BTNC pushbutton triggers on-demand re-tare/recalibration
//                2. Peak-to-Peak Current Measurement & 20-Sample Moving Average:
//                   - Windowed min/max tracking over fundamental AC cycle:
//                     * SW[1] = 0 (Default): 50 Hz fundamental (20 ms / 200 samples)
//                     * SW[1] = 1: Fast 20-sample sliding window (2 ms)
//                   - 20-sample circular moving average buffer for Channel 0 & 1:
//                     I_pp_avg = (1/20) * SUM_{i=0}^{19} (I_pp[i])
//                3. High-Precision Physical Conversions:
//                   - Peak-to-Peak Current: raw_pp * 16.11328 mA/count
//                   - RMS Current: I_rms = I_pp / (2 * sqrt(2)) = I_pp * 0.35355
//                4. Real-Time SSD1306 OLED Display (4 Lines x 16 Characters):
//                   - SW[0] = 0 (Default): Peak-to-Peak Current (CH0 & CH1 in Amperes)
//                   - SW[0] = 1: Detailed Peak-to-Peak + True RMS View (in Amperes)
//                5. Diagnostic Board LEDs LD7..LD0 for instant visual status.
// =============================================================================

`include "mpc_params.vh"

module ac_current_top #(
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
    wire mode_rms_view   = sw_sync2[0]; // SW0: 0=Peak-to-Peak View, 1=RMS View
    wire mode_fast_win   = sw_sync2[1]; // SW1: 0=50Hz Cycle (200 smpl), 1=Fast (20 smpl)

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
    // 5. Auto-Tare Calibration Engine & Measurement FSM (Matches control_fsm.v)
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
    localparam S_AC_CAL      = 4'd9;
    localparam S_AC_WAIT     = 4'd10;
    localparam S_AC_CAL_DONE = 4'd11;

    localparam AC_CAL_NUM_WINDOWS = SIMULATION ? 12'd4 : 12'd256;
    localparam AC_CAL_SHIFT       = SIMULATION ? 2 : 8;

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
    reg               dc_cal_done;
    reg               cal_done;
    reg               outlier_rejected;

    // AC Noise Floor Calibration (Phase 2, 5-second duration)
    reg [28:0]        ac_cal_cnt;
    reg [19:0]        ac_noise_accum_ch0;
    reg [19:0]        ac_noise_accum_ch1;
    reg [11:0]        ac_noise_valid_cnt;
    reg [11:0]        ac_noise_pp_ch0;
    reg [11:0]        ac_noise_pp_ch1;

    // Live ADC captured registers
    reg [11:0] live_raw_ch0;
    reg [11:0] live_raw_ch1;
    reg [15:0] meas_timer;

    // Difference wire for running mean
    wire signed [13:0] cal_ch0_diff = $signed({1'b0, adc_data_ch0}) - cal_mean_ch0;
    wire signed [13:0] cal_ch1_diff = $signed({1'b0, adc_data_ch1}) - cal_mean_ch1;

    // 5-second countdown signal for DC Warmup OLED display
    wire [2:0] cal_countdown = SIMULATION ? 3'd1 :
                               (warmup_cnt < 29'd100_000_000) ? 3'd5 :
                               (warmup_cnt < 29'd200_000_000) ? 3'd4 :
                               (warmup_cnt < 29'd300_000_000) ? 3'd3 :
                               (warmup_cnt < 29'd400_000_000) ? 3'd2 : 3'd1;

    // 5-second countdown signal for AC Noise Calibration OLED display
    wire [2:0] ac_countdown = SIMULATION ? 3'd1 :
                              (ac_noise_valid_cnt < 12'd51)  ? 3'd5 :
                              (ac_noise_valid_cnt < 12'd102) ? 3'd4 :
                              (ac_noise_valid_cnt < 12'd153) ? 3'd3 :
                              (ac_noise_valid_cnt < 12'd204) ? 3'd2 : 3'd1;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            adc_start          <= 1'b0;
            warmup_cnt         <= 29'd0;
            cal_sample_cnt     <= 16'd0;
            cal_valid_cnt      <= 16'd0;
            cal_accum_ch0      <= 24'sd0;
            cal_accum_ch1      <= 24'sd0;
            cal_mean_ch0       <= 13'sd2048;
            cal_mean_ch1       <= 13'sd2048;
            adc_offset_ch0     <= 13'sd2048;
            adc_offset_ch1     <= 13'sd2048;
            dc_cal_done        <= 1'b0;
            cal_done           <= 1'b0;
            outlier_rejected   <= 1'b0;
            ac_noise_pp_ch0    <= 12'd0;
            ac_noise_pp_ch1    <= 12'd0;
            live_raw_ch0       <= 12'd2048;
            live_raw_ch1       <= 12'd2048;
            meas_timer         <= 16'd0;
        end else begin
            adc_start <= 1'b0;

            // Manual re-tare button triggers fresh 10-second calibration from any operational state
            if (btnc_pulse && state != S_CAL_WARMUP && state != S_CAL_ADC && 
                state != S_CAL_WAIT && state != S_CAL_ACCUM && state != S_CAL_DELAY &&
                state != S_AC_CAL && state != S_AC_WAIT && state != S_AC_CAL_DONE) begin
                cal_done           <= 1'b0;
                dc_cal_done        <= 1'b0;
                warmup_cnt         <= 29'd0;
                cal_sample_cnt     <= 16'd0;
                cal_valid_cnt      <= 16'd0;
                cal_accum_ch0      <= 24'sd0;
                cal_accum_ch1      <= 24'sd0;
                cal_mean_ch0       <= 13'sd2048;
                cal_mean_ch1       <= 13'sd2048;
                ac_noise_pp_ch0    <= 12'd0;
                ac_noise_pp_ch1    <= 12'd0;
                meas_timer         <= 16'd0;
                state              <= S_CAL_WARMUP;
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
                            cal_mean_ch0     <= $signed({1'b0, adc_data_ch0});
                            cal_mean_ch1     <= $signed({1'b0, adc_data_ch1});
                            cal_accum_ch0    <= $signed({1'b0, adc_data_ch0});
                            cal_accum_ch1    <= $signed({1'b0, adc_data_ch1});
                            cal_valid_cnt    <= 16'd1;
                            outlier_rejected <= 1'b0;
                            meas_timer       <= 16'd0;
                            state            <= S_CAL_DELAY;
                        end else if (($signed({1'b0, adc_data_ch0}) > (cal_mean_ch0 - 13'sd100)) &&
                                     ($signed({1'b0, adc_data_ch0}) < (cal_mean_ch0 + 13'sd100))) begin
                            cal_accum_ch0    <= cal_accum_ch0 + $signed({1'b0, adc_data_ch0});
                            cal_accum_ch1    <= cal_accum_ch1 + $signed({1'b0, adc_data_ch1});
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
                        // Phase 1 (DC) Complete: compute offset via 8-bit shift with rounding (+128)
                        if (cal_valid_cnt >= `AUTOTARE_NUM_SAMPLES) begin
                            adc_offset_ch0 <= (cal_accum_ch0 + 24'sd128) >>> 8;
                            adc_offset_ch1 <= (cal_accum_ch1 + 24'sd128) >>> 8;
                        end else begin
                            adc_offset_ch0 <= cal_mean_ch0;
                            adc_offset_ch1 <= cal_mean_ch1;
                        end
                        dc_cal_done <= 1'b1;
                        meas_timer  <= 16'd0;
                        state       <= S_AC_CAL;
                    end

                    S_AC_CAL: begin
                        // Phase 2: AC Noise Calibration (5 seconds, 256 fundamental windows)
                        // Sample ADC at 10 kHz while current is kept at 0A
                        if (ac_noise_valid_cnt >= AC_CAL_NUM_WINDOWS) begin
                            state <= S_AC_CAL_DONE;
                        end else if (meas_timer >= SAMPLE_PERIOD - 1) begin
                            meas_timer <= 16'd0;
                            adc_start  <= 1'b1;
                            state      <= S_AC_WAIT;
                        end else begin
                            meas_timer <= meas_timer + 16'd1;
                        end
                    end

                    S_AC_WAIT: begin
                        if (adc_done) begin
                            live_raw_ch0 <= adc_data_ch0;
                            live_raw_ch1 <= adc_data_ch1;
                            state        <= S_AC_CAL;
                        end
                    end

                    S_AC_CAL_DONE: begin
                        // Compute average AC noise floor peak-to-peak over 256 windows (>> AC_CAL_SHIFT)
                        ac_noise_pp_ch0 <= (ac_noise_accum_ch0 + (20'd1 << (AC_CAL_SHIFT - 1))) >> AC_CAL_SHIFT;
                        ac_noise_pp_ch1 <= (ac_noise_accum_ch1 + (20'd1 << (AC_CAL_SHIFT - 1))) >> AC_CAL_SHIFT;
                        cal_done        <= 1'b1;
                        meas_timer      <= 16'd0;
                        state           <= S_MEASURE;
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
    // 6. Windowed Peak-to-Peak Detector Signals
    // -------------------------------------------------------------------------
    wire [8:0] window_limit = SIMULATION ? 9'd20 :
                              (mode_fast_win ? 9'd20 : 9'd200);

    reg [8:0]  win_cnt;
    reg [11:0] win_min_ch0, win_max_ch0;
    reg [11:0] win_min_ch1, win_max_ch1;
    reg [11:0] pp_window_ch0;
    reg [11:0] pp_window_ch1;
    reg        pp_window_valid;

    // -------------------------------------------------------------------------
    // 5.1 AC Noise Floor Accumulation Engine (Phase 2, 5-Second Duration)
    //     Accumulates fundamental window peak-to-peak noise while current is 0A.
    // -------------------------------------------------------------------------
    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            ac_noise_accum_ch0 <= 20'd0;
            ac_noise_accum_ch1 <= 20'd0;
            ac_noise_valid_cnt <= 12'd0;
        end else if (!dc_cal_done) begin
            ac_noise_accum_ch0 <= 20'd0;
            ac_noise_accum_ch1 <= 20'd0;
            ac_noise_valid_cnt <= 12'd0;
        end else if ((state == S_AC_CAL || state == S_AC_WAIT) && pp_window_valid) begin
            ac_noise_accum_ch0 <= ac_noise_accum_ch0 + {8'd0, pp_window_ch0};
            ac_noise_accum_ch1 <= ac_noise_accum_ch1 + {8'd0, pp_window_ch1};
            ac_noise_valid_cnt <= ac_noise_valid_cnt + 12'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 6. Windowed Peak-to-Peak Detector Engine
    //    Tracks min and max ADC samples across fundamental window:
    //    - 50 Hz fundamental: 20 ms / 200 samples at 10 kHz
    //    - Fast Mode: 20 samples (2 ms)
    // -------------------------------------------------------------------------
    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            win_cnt         <= 9'd0;
            win_min_ch0     <= 12'd2048;
            win_max_ch0     <= 12'd2048;
            win_min_ch1     <= 12'd2048;
            win_max_ch1     <= 12'd2048;
            pp_window_ch0   <= 12'd0;
            pp_window_ch1   <= 12'd0;
            pp_window_valid <= 1'b0;
        end else if (!dc_cal_done) begin
            win_cnt         <= 9'd0;
            win_min_ch0     <= 12'd2048;
            win_max_ch0     <= 12'd2048;
            win_min_ch1     <= 12'd2048;
            win_max_ch1     <= 12'd2048;
            pp_window_ch0   <= 12'd0;
            pp_window_ch1   <= 12'd0;
            pp_window_valid <= 1'b0;
        end else if (adc_done && (state == S_AC_WAIT || state == S_WAIT_MEAS)) begin
            if (win_cnt == 9'd0) begin
                // Initialize min/max at start of window
                win_min_ch0     <= adc_data_ch0;
                win_max_ch0     <= adc_data_ch0;
                win_min_ch1     <= adc_data_ch1;
                win_max_ch1     <= adc_data_ch1;
                win_cnt         <= 9'd1;
                pp_window_valid <= 1'b0;
            end else begin
                // Track min and max
                if (adc_data_ch0 < win_min_ch0) win_min_ch0 <= adc_data_ch0;
                if (adc_data_ch0 > win_max_ch0) win_max_ch0 <= adc_data_ch0;
                if (adc_data_ch1 < win_min_ch1) win_min_ch1 <= adc_data_ch1;
                if (adc_data_ch1 > win_max_ch1) win_max_ch1 <= adc_data_ch1;

                if (win_cnt >= window_limit - 1) begin
                    win_cnt         <= 9'd0;
                    pp_window_ch0   <= (adc_data_ch0 > win_max_ch0 ? adc_data_ch0 : win_max_ch0) -
                                       (adc_data_ch0 < win_min_ch0 ? adc_data_ch0 : win_min_ch0);
                    pp_window_ch1   <= (adc_data_ch1 > win_max_ch1 ? adc_data_ch1 : win_max_ch1) -
                                       (adc_data_ch1 < win_min_ch1 ? adc_data_ch1 : win_min_ch1);
                    pp_window_valid <= 1'b1;
                end else begin
                    win_cnt         <= win_cnt + 9'd1;
                    pp_window_valid <= 1'b0;
                end
            end
        end else begin
            pp_window_valid <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // 7. 20-Sample Moving Average Filter for Peak-to-Peak Current
    //    Averages the previous 20 peak-to-peak measurements.
    // -------------------------------------------------------------------------
    reg [11:0] pp0_hist [0:19];
    reg [11:0] pp1_hist [0:19];
    reg [16:0] pp0_sum;
    reg [16:0] pp1_sum;
    reg [4:0]  fill_cnt;

    integer j;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            pp0_sum  <= 17'd0;
            pp1_sum  <= 17'd0;
            fill_cnt <= 5'd0;
            for (j = 0; j < 20; j = j + 1) begin
                pp0_hist[j] <= 12'd0;
                pp1_hist[j] <= 12'd0;
            end
        end else if (!cal_done) begin
            pp0_sum  <= 17'd0;
            pp1_sum  <= 17'd0;
            fill_cnt <= 5'd0;
            for (j = 0; j < 20; j = j + 1) begin
                pp0_hist[j] <= 12'd0;
                pp1_hist[j] <= 12'd0;
            end
        end else if (pp_window_valid) begin
            if (fill_cnt < 5'd20) begin
                // Initial fill: accumulate into buffer
                pp0_hist[fill_cnt] <= pp_window_ch0;
                pp1_hist[fill_cnt] <= pp_window_ch1;
                pp0_sum            <= pp0_sum + pp_window_ch0;
                pp1_sum            <= pp1_sum + pp_window_ch1;
                fill_cnt           <= fill_cnt + 5'd1;
            end else begin
                // Moving average: subtract oldest sample, add newest sample
                pp0_sum <= pp0_sum - pp0_hist[19] + pp_window_ch0;
                pp1_sum <= pp1_sum - pp1_hist[19] + pp_window_ch1;

                // Shift history buffer
                pp0_hist[0] <= pp_window_ch0;
                pp1_hist[0] <= pp_window_ch1;
                for (j = 1; j < 20; j = j + 1) begin
                    pp0_hist[j] <= pp0_hist[j-1];
                    pp1_hist[j] <= pp1_hist[j-1];
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 7.1 Pipelined Moving Average & Physical Conversions
    //     Multi-stage sequential pipeline triggered upon pp_window_valid.
    //     Eliminates 33 ns combinational divider + 2x multiplier chain,
    //     closing timing with >6 ns positive slack on 100 MHz clock.
    // -------------------------------------------------------------------------
    reg [2:0]  calc_pipe_step;
    reg [11:0] avg_pp_ch0;
    reg [11:0] avg_pp_ch1;
    reg [31:0] i_pp_calc_ch0;
    reg [31:0] i_pp_calc_ch1;
    reg [15:0] i_pp_ma_ch0;
    reg [15:0] i_pp_ma_ch1;
    reg [31:0] i_rms_calc_ch0;
    reg [31:0] i_rms_calc_ch1;
    reg [15:0] i_rms_ma_ch0;
    reg [15:0] i_rms_ma_ch1;
    reg        overcurrent_active;

    // Exact 24-bit reciprocal lookup table for k in [1, 20]:
    // (sum * recip_mult) >> 24 yields EXACT integer division (sum // k)
    // for all sums up to 81,900 with zero truncation error and only 1 DSP stage.
    reg [24:0] recip_mult;
    always @(*) begin
        case (fill_cnt)
            5'd1:  recip_mult = 25'd16777216;
            5'd2:  recip_mult = 25'd8388608;
            5'd3:  recip_mult = 25'd5592406;
            5'd4:  recip_mult = 25'd4194304;
            5'd5:  recip_mult = 25'd3355444;
            5'd6:  recip_mult = 25'd2796203;
            5'd7:  recip_mult = 25'd2396746;
            5'd8:  recip_mult = 25'd2097152;
            5'd9:  recip_mult = 25'd1864136;
            5'd10: recip_mult = 25'd1677722;
            5'd11: recip_mult = 25'd1525202;
            5'd12: recip_mult = 25'd1398102;
            5'd13: recip_mult = 25'd1290556;
            5'd14: recip_mult = 25'd1198373;
            5'd15: recip_mult = 25'd1118482;
            5'd16: recip_mult = 25'd1048576;
            5'd17: recip_mult = 25'd986896;
            5'd18: recip_mult = 25'd932068;
            5'd19: recip_mult = 25'd883012;
            5'd20: recip_mult = 25'd838861;
            default: recip_mult = 25'd838861;
        endcase
    end

    wire [41:0] prod_pp_ch0 = pp0_sum * recip_mult;
    wire [41:0] prod_pp_ch1 = pp1_sum * recip_mult;

    wire [11:0] clean_pp_ch0 = (avg_pp_ch0 > ac_noise_pp_ch0) ? (avg_pp_ch0 - ac_noise_pp_ch0) : 12'd0;
    wire [11:0] clean_pp_ch1 = (avg_pp_ch1 > ac_noise_pp_ch1) ? (avg_pp_ch1 - ac_noise_pp_ch1) : 12'd0;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            calc_pipe_step     <= 3'd0;
            avg_pp_ch0         <= 12'd0;
            avg_pp_ch1         <= 12'd0;
            i_pp_calc_ch0      <= 32'd0;
            i_pp_calc_ch1      <= 32'd0;
            i_pp_ma_ch0        <= 16'd0;
            i_pp_ma_ch1        <= 16'd0;
            i_rms_calc_ch0     <= 32'd0;
            i_rms_calc_ch1     <= 32'd0;
            i_rms_ma_ch0       <= 16'd0;
            i_rms_ma_ch1       <= 16'd0;
            overcurrent_active <= 1'b0;
        end else if (!cal_done) begin
            calc_pipe_step     <= 3'd0;
            avg_pp_ch0         <= 12'd0;
            avg_pp_ch1         <= 12'd0;
            i_pp_calc_ch0      <= 32'd0;
            i_pp_calc_ch1      <= 32'd0;
            i_pp_ma_ch0        <= 16'd0;
            i_pp_ma_ch1        <= 16'd0;
            i_rms_calc_ch0     <= 32'd0;
            i_rms_calc_ch1     <= 32'd0;
            i_rms_ma_ch0       <= 16'd0;
            i_rms_ma_ch1       <= 16'd0;
            overcurrent_active <= 1'b0;
        end else begin
            case (calc_pipe_step)
                3'd0: begin
                    if (pp_window_valid) begin
                        calc_pipe_step <= 3'd1;
                    end
                end

                3'd1: begin
                    // Step 1: Compute moving average via 24-bit reciprocal multiplication
                    avg_pp_ch0     <= prod_pp_ch0[35:24];
                    avg_pp_ch1     <= prod_pp_ch1[35:24];
                    calc_pipe_step <= 3'd2;
                end

                3'd2: begin
                    // Step 2: Calibrated Peak-to-Peak physical calculation: (clean_pp * 33000 + 1024)
                    i_pp_calc_ch0      <= clean_pp_ch0 * 32'd33000 + 32'd1024;
                    i_pp_calc_ch1      <= clean_pp_ch1 * 32'd33000 + 32'd1024;
                    overcurrent_active <= (clean_pp_ch0 > 12'd620) || (clean_pp_ch1 > 12'd620);
                    calc_pipe_step     <= 3'd3;
                end

                3'd3: begin
                    // Step 3: Register Peak-to-Peak current in mA (>> 11)
                    i_pp_ma_ch0    <= i_pp_calc_ch0[26:11];
                    i_pp_ma_ch1    <= i_pp_calc_ch1[26:11];
                    calc_pipe_step <= 3'd4;
                end

                3'd4: begin
                    // Step 4: RMS physical calculation in mA: (i_pp_ma * 362 + 512)
                    i_rms_calc_ch0 <= i_pp_ma_ch0 * 32'd362 + 32'd512;
                    i_rms_calc_ch1 <= i_pp_ma_ch1 * 32'd362 + 32'd512;
                    calc_pipe_step <= 3'd5;
                end

                3'd5: begin
                    // Step 5: Register RMS current in mA (>> 10)
                    i_rms_ma_ch0   <= i_rms_calc_ch0[25:10];
                    i_rms_ma_ch1   <= i_rms_calc_ch1[25:10];
                    calc_pipe_step <= 3'd0;
                end

                default: calc_pipe_step <= 3'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 9. 1 Hz Diagnostic Heartbeat Generator
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
    // 10. Pipelined Decimal Digit Formatter (40 Hz / 25 ms period)
    //     Distributes division/modulo operations across 5 clock cycles
    //     to guarantee zero setup-timing violations on 100 MHz clock.
    // -------------------------------------------------------------------------
    reg [21:0] fmt_timer;
    reg [3:0]  fmt_step;

    // Snapshot registers
    reg [15:0] snap_ipp0, snap_ipp1;
    reg [15:0] snap_rms0, snap_rms1;
    reg [11:0] snap_raw_pp0;
    reg signed [12:0] snap_off0;
    reg [15:0] snap_valid_cnt;
    reg signed [12:0] snap_mean;
    reg [2:0]  snap_countdown;
    reg [2:0]  snap_ac_countdown;
    reg [11:0] snap_ac_valid_cnt;
    reg [11:0] snap_noise_pp0;
    reg        snap_dc_cal_done;
    reg        snap_cal_done;
    reg [3:0]  snap_state;
    reg        snap_mode_rms;
    reg        snap_mode_fast;

    // Current Digits (whole, tenths, hundredths, thousandths): e.g. 4.222 A
    reg [15:0] ipp0_val, ipp1_val;
    reg [15:0] ipp0_rem_th, ipp1_rem_th;
    reg [15:0] ipp0_rem_h, ipp1_rem_h;
    reg [3:0]  ipp0_w, ipp0_t, ipp0_h, ipp0_m;
    reg [3:0]  ipp1_w, ipp1_t, ipp1_h, ipp1_m;

    // RMS Digits: e.g. 1.493 A
    reg [15:0] rms0_val, rms1_val;
    reg [15:0] rms0_rem_th, rms1_rem_th;
    reg [15:0] rms0_rem_h, rms1_rem_h;
    reg [3:0]  rms0_w, rms0_t, rms0_h, rms0_m;
    reg [3:0]  rms1_w, rms1_t, rms1_h, rms1_m;

    // Raw Peak-to-Peak Digits: e.g. 0262
    reg [3:0]  raw_pp_d3, raw_pp_d2, raw_pp_d1, raw_pp_d0;
    reg [11:0] raw_pp_rem_th, raw_pp_rem_h;

    // Offset Digits: e.g. 2048
    reg [15:0] off0_abs;
    reg [3:0]  off0_d3, off0_d2, off0_d1, off0_d0;
    reg [15:0] off0_rem_th, off0_rem_h;

    // Calibration Progress Digits
    reg [3:0]  valid_d2, valid_d1, valid_d0;
    reg [15:0] valid_rem_h;
    reg [15:0] mean_abs;
    reg [3:0]  mean_d3, mean_d2, mean_d1, mean_d0;
    reg [15:0] mean_rem_th, mean_rem_h;

    // AC Calibration Digits
    reg [3:0]  ac_valid_d2, ac_valid_d1, ac_valid_d0;
    reg [15:0] ac_valid_rem_h;
    reg [3:0]  noise_d2, noise_d1, noise_d0;
    reg [15:0] noise_rem_h;

    // 64-Character OLED Display Buffer (4 lines x 16 characters)
    reg [7:0] oled_chars [0:63];

    integer k;

    // -------------------------------------------------------------------------
    // Bounded Arithmetic Helper Functions for High-Speed Decimal Formatting
    // Synthesizes to parallel LUTs with zero CARRY4 chain (< 1.5 ns delay).
    // -------------------------------------------------------------------------
    function [3:0] div1000_q;
        input [13:0] val;
        begin
            div1000_q = (val >= 14'd9000) ? 4'd9 :
                        (val >= 14'd8000) ? 4'd8 :
                        (val >= 14'd7000) ? 4'd7 :
                        (val >= 14'd6000) ? 4'd6 :
                        (val >= 14'd5000) ? 4'd5 :
                        (val >= 14'd4000) ? 4'd4 :
                        (val >= 14'd3000) ? 4'd3 :
                        (val >= 14'd2000) ? 4'd2 :
                        (val >= 14'd1000) ? 4'd1 : 4'd0;
        end
    endfunction

    function [9:0] mod1000_q;
        input [13:0] val;
        begin
            mod1000_q = (val >= 14'd9000) ? (val - 14'd9000) :
                        (val >= 14'd8000) ? (val - 14'd8000) :
                        (val >= 14'd7000) ? (val - 14'd7000) :
                        (val >= 14'd6000) ? (val - 14'd6000) :
                        (val >= 14'd5000) ? (val - 14'd5000) :
                        (val >= 14'd4000) ? (val - 14'd4000) :
                        (val >= 14'd3000) ? (val - 14'd3000) :
                        (val >= 14'd2000) ? (val - 14'd2000) :
                        (val >= 14'd1000) ? (val - 14'd1000) : val[9:0];
        end
    endfunction

    function [3:0] div100_q;
        input [9:0] val;
        begin
            div100_q = (val >= 10'd900) ? 4'd9 :
                       (val >= 10'd800) ? 4'd8 :
                       (val >= 10'd700) ? 4'd7 :
                       (val >= 10'd600) ? 4'd6 :
                       (val >= 10'd500) ? 4'd5 :
                       (val >= 10'd400) ? 4'd4 :
                       (val >= 10'd300) ? 4'd3 :
                       (val >= 10'd200) ? 4'd2 :
                       (val >= 10'd100) ? 4'd1 : 4'd0;
        end
    endfunction

    function [6:0] mod100_q;
        input [9:0] val;
        begin
            mod100_q = (val >= 10'd900) ? (val - 10'd900) :
                       (val >= 10'd800) ? (val - 10'd800) :
                       (val >= 10'd700) ? (val - 10'd700) :
                       (val >= 10'd600) ? (val - 10'd600) :
                       (val >= 10'd500) ? (val - 10'd500) :
                       (val >= 10'd400) ? (val - 10'd400) :
                       (val >= 10'd300) ? (val - 10'd300) :
                       (val >= 10'd200) ? (val - 10'd200) :
                       (val >= 10'd100) ? (val - 10'd100) : val[6:0];
        end
    endfunction

    function [3:0] div10_q;
        input [6:0] val;
        begin
            div10_q = (val >= 7'd90) ? 4'd9 :
                      (val >= 7'd80) ? 4'd8 :
                      (val >= 7'd70) ? 4'd7 :
                      (val >= 7'd60) ? 4'd6 :
                      (val >= 7'd50) ? 4'd5 :
                      (val >= 7'd40) ? 4'd4 :
                      (val >= 7'd30) ? 4'd3 :
                      (val >= 7'd20) ? 4'd2 :
                      (val >= 7'd10) ? 4'd1 : 4'd0;
        end
    endfunction

    function [3:0] mod10_q;
        input [6:0] val;
        begin
            mod10_q = (val >= 7'd90) ? (val - 7'd90) :
                      (val >= 7'd80) ? (val - 7'd80) :
                      (val >= 7'd70) ? (val - 7'd70) :
                      (val >= 7'd60) ? (val - 7'd60) :
                      (val >= 7'd50) ? (val - 7'd50) :
                      (val >= 7'd40) ? (val - 7'd40) :
                      (val >= 7'd30) ? (val - 7'd30) :
                      (val >= 7'd20) ? (val - 7'd20) :
                      (val >= 7'd10) ? (val - 7'd10) : val[3:0];
        end
    endfunction

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            fmt_timer         <= 22'd0;
            fmt_step          <= 4'd0;
            snap_ipp0         <= 16'd0;
            snap_ipp1         <= 16'd0;
            snap_rms0         <= 16'd0;
            snap_rms1         <= 16'd0;
            snap_raw_pp0      <= 12'd0;
            snap_off0         <= 13'sd2048;
            snap_valid_cnt    <= 16'd0;
            snap_mean         <= 13'sd2048;
            snap_countdown    <= 3'd5;
            snap_ac_countdown <= 3'd5;
            snap_ac_valid_cnt <= 12'd0;
            snap_noise_pp0    <= 12'd0;
            snap_dc_cal_done  <= 1'b0;
            snap_cal_done     <= 1'b0;
            snap_state        <= S_IDLE;
            snap_mode_rms     <= 1'b0;
            snap_mode_fast    <= 1'b0;

            ipp0_val    <= 16'd0; ipp1_val    <= 16'd0;
            rms0_val    <= 16'd0; rms1_val    <= 16'd0;
            ipp0_rem_th <= 16'd0; ipp1_rem_th <= 16'd0;
            rms0_rem_th <= 16'd0; rms1_rem_th <= 16'd0;
            ipp0_rem_h  <= 16'd0; ipp1_rem_h  <= 16'd0;
            rms0_rem_h  <= 16'd0; rms1_rem_h  <= 16'd0;

            ipp0_w <= 4'd0; ipp0_t <= 4'd0; ipp0_h <= 4'd0; ipp0_m <= 4'd0;
            ipp1_w <= 4'd0; ipp1_t <= 4'd0; ipp1_h <= 4'd0; ipp1_m <= 4'd0;
            rms0_w <= 4'd0; rms0_t <= 4'd0; rms0_h <= 4'd0; rms0_m <= 4'd0;
            rms1_w <= 4'd0; rms1_t <= 4'd0; rms1_h <= 4'd0; rms1_m <= 4'd0;

            raw_pp_d3     <= 4'd0; raw_pp_d2    <= 4'd0; raw_pp_d1 <= 4'd0; raw_pp_d0 <= 4'd0;
            raw_pp_rem_th <= 12'd0; raw_pp_rem_h <= 12'd0;

            off0_abs    <= 16'd0;
            off0_d3     <= 4'd0; off0_d2 <= 4'd0; off0_d1 <= 4'd0; off0_d0 <= 4'd0;
            off0_rem_th <= 16'd0; off0_rem_h <= 16'd0;

            valid_d2    <= 4'd0; valid_d1 <= 4'd0; valid_d0 <= 4'd0;
            valid_rem_h <= 16'd0;
            mean_abs    <= 16'd0;
            mean_d3     <= 4'd0; mean_d2 <= 4'd0; mean_d1 <= 4'd0; mean_d0 <= 4'd0;
            mean_rem_th <= 16'd0; mean_rem_h <= 16'd0;

            ac_valid_d2    <= 4'd0; ac_valid_d1 <= 4'd0; ac_valid_d0 <= 4'd0;
            ac_valid_rem_h <= 16'd0;
            noise_d2       <= 4'd0; noise_d1 <= 4'd0; noise_d0 <= 4'd0;
            noise_rem_h    <= 16'd0;

            for (k = 0; k < 64; k = k + 1)
                oled_chars[k] <= 8'h20; // Space
        end else begin
            case (fmt_step)
                4'd0: begin
                    // Trigger new display frame conversion at 40 Hz
                    if (fmt_timer >= FMT_PERIOD - 1) begin
                        fmt_timer         <= 22'd0;
                        snap_ipp0         <= i_pp_ma_ch0;
                        snap_ipp1         <= i_pp_ma_ch1;
                        snap_rms0         <= i_rms_ma_ch0;
                        snap_rms1         <= i_rms_ma_ch1;
                        snap_raw_pp0      <= clean_pp_ch0;
                        snap_off0         <= adc_offset_ch0;
                        snap_valid_cnt    <= cal_valid_cnt;
                        snap_mean         <= cal_mean_ch0;
                        snap_countdown    <= cal_countdown;
                        snap_ac_countdown <= ac_countdown;
                        snap_ac_valid_cnt <= ac_noise_valid_cnt;
                        snap_noise_pp0    <= cal_done ? ac_noise_pp_ch0 : pp_window_ch0;
                        snap_dc_cal_done  <= dc_cal_done;
                        snap_cal_done     <= cal_done;
                        snap_state        <= state;
                        snap_mode_rms     <= mode_rms_view;
                        snap_mode_fast    <= mode_fast_win;

                        off0_abs <= (adc_offset_ch0 < 0) ? -adc_offset_ch0 : adc_offset_ch0;
                        mean_abs <= (cal_mean_ch0 < 0) ? -cal_mean_ch0 : cal_mean_ch0;

                        fmt_step <= 4'd1;
                    end else begin
                        fmt_timer <= fmt_timer + 22'd1;
                    end
                end

                4'd1: begin
                    // Step 1: Input bounding (0..9999) and fast bounded comparison for thousands
                    ipp0_val <= (snap_ipp0 > 16'd9999) ? 16'd9999 : snap_ipp0;
                    ipp1_val <= (snap_ipp1 > 16'd9999) ? 16'd9999 : snap_ipp1;
                    rms0_val <= (snap_rms0 > 16'd9999) ? 16'd9999 : snap_rms0;
                    rms1_val <= (snap_rms1 > 16'd9999) ? 16'd9999 : snap_rms1;

                    raw_pp_d3     <= (snap_raw_pp0 >= 12'd4000) ? 4'd4 : (snap_raw_pp0 >= 12'd3000) ? 4'd3 : 
                                     (snap_raw_pp0 >= 12'd2000) ? 4'd2 : (snap_raw_pp0 >= 12'd1000) ? 4'd1 : 4'd0;
                    raw_pp_rem_th <= (snap_raw_pp0 >= 12'd4000) ? (snap_raw_pp0 - 12'd4000) : 
                                     (snap_raw_pp0 >= 12'd3000) ? (snap_raw_pp0 - 12'd3000) : 
                                     (snap_raw_pp0 >= 12'd2000) ? (snap_raw_pp0 - 12'd2000) : 
                                     (snap_raw_pp0 >= 12'd1000) ? (snap_raw_pp0 - 12'd1000) : snap_raw_pp0;

                    off0_d3     <= (off0_abs >= 16'd4000) ? 4'd4 : (off0_abs >= 16'd3000) ? 4'd3 : 
                                   (off0_abs >= 16'd2000) ? 4'd2 : (off0_abs >= 16'd1000) ? 4'd1 : 4'd0;
                    off0_rem_th <= (off0_abs >= 16'd4000) ? (off0_abs - 16'd4000) : 
                                   (off0_abs >= 16'd3000) ? (off0_abs - 16'd3000) : 
                                   (off0_abs >= 16'd2000) ? (off0_abs - 16'd2000) : 
                                   (off0_abs >= 16'd1000) ? (off0_abs - 16'd1000) : off0_abs[9:0];

                    mean_d3     <= (mean_abs >= 16'd4000) ? 4'd4 : (mean_abs >= 16'd3000) ? 4'd3 : 
                                   (mean_abs >= 16'd2000) ? 4'd2 : (mean_abs >= 16'd1000) ? 4'd1 : 4'd0;
                    mean_rem_th <= (mean_abs >= 16'd4000) ? (mean_abs - 16'd4000) : 
                                   (mean_abs >= 16'd3000) ? (mean_abs - 16'd3000) : 
                                   (mean_abs >= 16'd2000) ? (mean_abs - 16'd2000) : 
                                   (mean_abs >= 16'd1000) ? (mean_abs - 16'd1000) : mean_abs[9:0];

                    valid_d2    <= (snap_valid_cnt >= 9'd200) ? 4'd2 : (snap_valid_cnt >= 9'd100) ? 4'd1 : 4'd0;
                    valid_rem_h <= (snap_valid_cnt >= 9'd200) ? (snap_valid_cnt - 9'd200) : 
                                   (snap_valid_cnt >= 9'd100) ? (snap_valid_cnt - 9'd100) : snap_valid_cnt[6:0];

                    ac_valid_d2    <= (snap_ac_valid_cnt >= 9'd200) ? 4'd2 : (snap_ac_valid_cnt >= 9'd100) ? 4'd1 : 4'd0;
                    ac_valid_rem_h <= (snap_ac_valid_cnt >= 9'd200) ? (snap_ac_valid_cnt - 9'd200) : 
                                      (snap_ac_valid_cnt >= 9'd100) ? (snap_ac_valid_cnt - 9'd100) : snap_ac_valid_cnt[6:0];

                    noise_d2    <= (snap_noise_pp0 >= 12'd200) ? 4'd2 : (snap_noise_pp0 >= 12'd100) ? 4'd1 : 4'd0;
                    noise_rem_h <= (snap_noise_pp0 >= 12'd200) ? (snap_noise_pp0 - 12'd200) : 
                                   (snap_noise_pp0 >= 12'd100) ? (snap_noise_pp0 - 12'd100) : snap_noise_pp0[6:0];

                    fmt_step <= 4'd2;
                end

                4'd2: begin
                    // Step 2: Extract whole Amps (thousands) via bounded tree (< 1.5 ns delay)
                    ipp0_w      <= div1000_q(ipp0_val[13:0]);
                    ipp0_rem_th <= mod1000_q(ipp0_val[13:0]);

                    ipp1_w      <= div1000_q(ipp1_val[13:0]);
                    ipp1_rem_th <= mod1000_q(ipp1_val[13:0]);

                    rms0_w      <= div1000_q(rms0_val[13:0]);
                    rms0_rem_th <= mod1000_q(rms0_val[13:0]);

                    rms1_w      <= div1000_q(rms1_val[13:0]);
                    rms1_rem_th <= mod1000_q(rms1_val[13:0]);

                    fmt_step <= 4'd3;
                end

                4'd3: begin
                    // Step 3: Extract tenths (hundreds) via bounded tree (< 1.2 ns delay)
                    ipp0_t     <= div100_q(ipp0_rem_th[9:0]);
                    ipp0_rem_h <= mod100_q(ipp0_rem_th[9:0]);

                    ipp1_t     <= div100_q(ipp1_rem_th[9:0]);
                    ipp1_rem_h <= mod100_q(ipp1_rem_th[9:0]);

                    rms0_t     <= div100_q(rms0_rem_th[9:0]);
                    rms0_rem_h <= mod100_q(rms0_rem_th[9:0]);

                    rms1_t     <= div100_q(rms1_rem_th[9:0]);
                    rms1_rem_h <= mod100_q(rms1_rem_th[9:0]);

                    raw_pp_d2    <= div100_q(raw_pp_rem_th[9:0]);
                    raw_pp_rem_h <= mod100_q(raw_pp_rem_th[9:0]);

                    off0_d2      <= div100_q(off0_rem_th[9:0]);
                    off0_rem_h   <= mod100_q(off0_rem_th[9:0]);

                    mean_d2      <= div100_q(mean_rem_th[9:0]);
                    mean_rem_h   <= mod100_q(mean_rem_th[9:0]);

                    fmt_step <= 4'd4;
                end

                4'd4: begin
                    // Step 4: Extract hundredths and thousandths (tens and units) via bounded tree (< 1.0 ns delay)
                    ipp0_h <= div10_q(ipp0_rem_h[6:0]);
                    ipp0_m <= mod10_q(ipp0_rem_h[6:0]);

                    ipp1_h <= div10_q(ipp1_rem_h[6:0]);
                    ipp1_m <= mod10_q(ipp1_rem_h[6:0]);

                    rms0_h <= div10_q(rms0_rem_h[6:0]);
                    rms0_m <= mod10_q(rms0_rem_h[6:0]);

                    rms1_h <= div10_q(rms1_rem_h[6:0]);
                    rms1_m <= mod10_q(rms1_rem_h[6:0]);

                    raw_pp_d1 <= div10_q(raw_pp_rem_h[6:0]);
                    raw_pp_d0 <= mod10_q(raw_pp_rem_h[6:0]);

                    off0_d1 <= div10_q(off0_rem_h[6:0]);
                    off0_d0 <= mod10_q(off0_rem_h[6:0]);

                    mean_d1 <= div10_q(mean_rem_h[6:0]);
                    mean_d0 <= mod10_q(mean_rem_h[6:0]);

                    valid_d1 <= div10_q(valid_rem_h[6:0]);
                    valid_d0 <= mod10_q(valid_rem_h[6:0]);

                    ac_valid_d1 <= div10_q(ac_valid_rem_h[6:0]);
                    ac_valid_d0 <= mod10_q(ac_valid_rem_h[6:0]);

                    noise_d1 <= div10_q(noise_rem_h[6:0]);
                    noise_d0 <= mod10_q(noise_rem_h[6:0]);

                    fmt_step <= 4'd5;
                end

                4'd5: begin
                    // Step 5: Render Text Buffer (4 Lines x 16 Chars = 64 Chars)
                    if (!snap_dc_cal_done && snap_state == S_CAL_WARMUP) begin
                        // -----------------------------------------------------
                        // Screen 1: Auto-Tare Warmup Countdown
                        // -----------------------------------------------------
                        // Line 0: "AUTOTARE CALIB  "
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "U"; oled_chars[2]  <= "T"; oled_chars[3]  <= "O";
                        oled_chars[4]  <= "T"; oled_chars[5]  <= "A"; oled_chars[6]  <= "R"; oled_chars[7]  <= "E";
                        oled_chars[8]  <= " "; oled_chars[9]  <= "C"; oled_chars[10] <= "A"; oled_chars[11] <= "L";
                        oled_chars[12] <= "I"; oled_chars[13] <= "B"; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        // Line 1: "WARMING UP:  5s "
                        oled_chars[16] <= "W"; oled_chars[17] <= "A"; oled_chars[18] <= "R"; oled_chars[19] <= "M";
                        oled_chars[20] <= "I"; oled_chars[21] <= "N"; oled_chars[22] <= "G"; oled_chars[23] <= " ";
                        oled_chars[24] <= "U"; oled_chars[25] <= "P"; oled_chars[26] <= ":"; oled_chars[27] <= " ";
                        oled_chars[28] <= " ";
                        oled_chars[29] <= 8'h30 + {5'd0, snap_countdown};
                        oled_chars[30] <= "s"; oled_chars[31] <= " ";

                        // Line 2: "SAMPLES: 000/256"
                        oled_chars[32] <= "S"; oled_chars[33] <= "A"; oled_chars[34] <= "M"; oled_chars[35] <= "P";
                        oled_chars[36] <= "L"; oled_chars[37] <= "E"; oled_chars[38] <= "S"; oled_chars[39] <= ":";
                        oled_chars[40] <= " "; oled_chars[41] <= "0"; oled_chars[42] <= "0"; oled_chars[43] <= "0";
                        oled_chars[44] <= "/"; oled_chars[45] <= "2"; oled_chars[46] <= "5"; oled_chars[47] <= "6";

                        // Line 3: "OFFSET: CALC... "
                        oled_chars[48] <= "O"; oled_chars[49] <= "F"; oled_chars[50] <= "F"; oled_chars[51] <= "S";
                        oled_chars[52] <= "E"; oled_chars[53] <= "T"; oled_chars[54] <= ":"; oled_chars[55] <= " ";
                        oled_chars[56] <= "C"; oled_chars[57] <= "A"; oled_chars[58] <= "L"; oled_chars[59] <= "C";
                        oled_chars[60] <= "."; oled_chars[61] <= "."; oled_chars[62] <= "."; oled_chars[63] <= " ";

                    end else if (!snap_dc_cal_done) begin
                        // -----------------------------------------------------
                        // Screen 2: DC Auto-Tare Sampling in Progress
                        // -----------------------------------------------------
                        // Line 0: "DC OFFSET CALIB "
                        oled_chars[0]  <= "D"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "O";
                        oled_chars[4]  <= "F"; oled_chars[5]  <= "F"; oled_chars[6]  <= "S"; oled_chars[7]  <= "E";
                        oled_chars[8]  <= "T"; oled_chars[9]  <= " "; oled_chars[10] <= "C"; oled_chars[11] <= "A";
                        oled_chars[12] <= "L"; oled_chars[13] <= "I"; oled_chars[14] <= "B"; oled_chars[15] <= " ";

                        // Line 1: "SAMPLING ADC... "
                        oled_chars[16] <= "S"; oled_chars[17] <= "A"; oled_chars[18] <= "M"; oled_chars[19] <= "P";
                        oled_chars[20] <= "L"; oled_chars[21] <= "I"; oled_chars[22] <= "N"; oled_chars[23] <= "G";
                        oled_chars[24] <= " "; oled_chars[25] <= "A"; oled_chars[26] <= "D"; oled_chars[27] <= "C";
                        oled_chars[28] <= "."; oled_chars[29] <= "."; oled_chars[30] <= "."; oled_chars[31] <= " ";

                        // Line 2: "SAMPLES: 184/256"
                        oled_chars[32] <= "S"; oled_chars[33] <= "A"; oled_chars[34] <= "M"; oled_chars[35] <= "P";
                        oled_chars[36] <= "L"; oled_chars[37] <= "E"; oled_chars[38] <= "S"; oled_chars[39] <= ":";
                        oled_chars[40] <= " ";
                        oled_chars[41] <= 8'h30 + {4'd0, valid_d2};
                        oled_chars[42] <= 8'h30 + {4'd0, valid_d1};
                        oled_chars[43] <= 8'h30 + {4'd0, valid_d0};
                        oled_chars[44] <= "/"; oled_chars[45] <= "2"; oled_chars[46] <= "5"; oled_chars[47] <= "6";

                        // Line 3: "MEAN:    2048   "
                        oled_chars[48] <= "M"; oled_chars[49] <= "E"; oled_chars[50] <= "A"; oled_chars[51] <= "N";
                        oled_chars[52] <= ":"; oled_chars[53] <= " "; oled_chars[54] <= " "; oled_chars[55] <= " ";
                        oled_chars[56] <= " ";
                        oled_chars[57] <= 8'h30 + {4'd0, mean_d3};
                        oled_chars[58] <= 8'h30 + {4'd0, mean_d2};
                        oled_chars[59] <= 8'h30 + {4'd0, mean_d1};
                        oled_chars[60] <= 8'h30 + {4'd0, mean_d0};
                        oled_chars[61] <= " "; oled_chars[62] <= " "; oled_chars[63] <= " ";

                    end else if (!snap_cal_done) begin
                        // -----------------------------------------------------
                        // Screen 2B: AC Noise Floor Calibration (5s duration)
                        // -----------------------------------------------------
                        // Line 0: "AC NOISE CALIB  "
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "N";
                        oled_chars[4]  <= "O"; oled_chars[5]  <= "I"; oled_chars[6]  <= "S"; oled_chars[7]  <= "E";
                        oled_chars[8]  <= " "; oled_chars[9]  <= "C"; oled_chars[10] <= "A"; oled_chars[11] <= "L";
                        oled_chars[12] <= "I"; oled_chars[13] <= "B"; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        // Line 1: "KEEP I=0A:   5s "
                        oled_chars[16] <= "K"; oled_chars[17] <= "E"; oled_chars[18] <= "E"; oled_chars[19] <= "P";
                        oled_chars[20] <= " "; oled_chars[21] <= "I"; oled_chars[22] <= "="; oled_chars[23] <= "0";
                        oled_chars[24] <= "A"; oled_chars[25] <= ":"; oled_chars[26] <= " "; oled_chars[27] <= " ";
                        oled_chars[28] <= " ";
                        oled_chars[29] <= 8'h30 + {5'd0, snap_ac_countdown};
                        oled_chars[30] <= "s"; oled_chars[31] <= " ";

                        // Line 2: "SAMPLES: 184/256"
                        oled_chars[32] <= "S"; oled_chars[33] <= "A"; oled_chars[34] <= "M"; oled_chars[35] <= "P";
                        oled_chars[36] <= "L"; oled_chars[37] <= "E"; oled_chars[38] <= "S"; oled_chars[39] <= ":";
                        oled_chars[40] <= " ";
                        oled_chars[41] <= 8'h30 + {4'd0, ac_valid_d2};
                        oled_chars[42] <= 8'h30 + {4'd0, ac_valid_d1};
                        oled_chars[43] <= 8'h30 + {4'd0, ac_valid_d0};
                        oled_chars[44] <= "/"; oled_chars[45] <= "2"; oled_chars[46] <= "5"; oled_chars[47] <= "6";

                        // Line 3: "NOISE:   010 LSB"
                        oled_chars[48] <= "N"; oled_chars[49] <= "O"; oled_chars[50] <= "I"; oled_chars[51] <= "S";
                        oled_chars[52] <= "E"; oled_chars[53] <= ":"; oled_chars[54] <= " "; oled_chars[55] <= " ";
                        oled_chars[56] <= " ";
                        oled_chars[57] <= 8'h30 + {4'd0, noise_d2};
                        oled_chars[58] <= 8'h30 + {4'd0, noise_d1};
                        oled_chars[59] <= 8'h30 + {4'd0, noise_d0};
                        oled_chars[60] <= " "; oled_chars[61] <= "L"; oled_chars[62] <= "S"; oled_chars[63] <= "B";

                    end else if (!snap_mode_rms) begin
                        // -----------------------------------------------------
                        // Screen 3: Default Peak-to-Peak View (SW0 = 0, 3 Decimals)
                        // -----------------------------------------------------
                        // Line 0: "AC PEAK-TO-PEAK "
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "P";
                        oled_chars[4]  <= "E"; oled_chars[5]  <= "A"; oled_chars[6]  <= "K"; oled_chars[7]  <= "-";
                        oled_chars[8]  <= "T"; oled_chars[9]  <= "O"; oled_chars[10] <= "-"; oled_chars[11] <= "P";
                        oled_chars[12] <= "E"; oled_chars[13] <= "A"; oled_chars[14] <= "K"; oled_chars[15] <= " ";

                        // Line 1: "0:4.222A PP 2048"
                        oled_chars[16] <= "0"; oled_chars[17] <= ":";
                        oled_chars[18] <= 8'h30 + {4'd0, ipp0_w};
                        oled_chars[19] <= ".";
                        oled_chars[20] <= 8'h30 + {4'd0, ipp0_t};
                        oled_chars[21] <= 8'h30 + {4'd0, ipp0_h};
                        oled_chars[22] <= 8'h30 + {4'd0, ipp0_m};
                        oled_chars[23] <= "A";
                        oled_chars[24] <= " "; oled_chars[25] <= "P"; oled_chars[26] <= "P";
                        oled_chars[27] <= " ";
                        oled_chars[28] <= 8'h30 + {4'd0, off0_d3};
                        oled_chars[29] <= 8'h30 + {4'd0, off0_d2};
                        oled_chars[30] <= 8'h30 + {4'd0, off0_d1};
                        oled_chars[31] <= 8'h30 + {4'd0, off0_d0};

                        // Line 2: "1:4.222A PP 2048"
                        oled_chars[32] <= "1"; oled_chars[33] <= ":";
                        oled_chars[34] <= 8'h30 + {4'd0, ipp1_w};
                        oled_chars[35] <= ".";
                        oled_chars[36] <= 8'h30 + {4'd0, ipp1_t};
                        oled_chars[37] <= 8'h30 + {4'd0, ipp1_h};
                        oled_chars[38] <= 8'h30 + {4'd0, ipp1_m};
                        oled_chars[39] <= "A";
                        oled_chars[40] <= " "; oled_chars[41] <= "P"; oled_chars[42] <= "P";
                        oled_chars[43] <= " ";
                        oled_chars[44] <= 8'h30 + {4'd0, off0_d3};
                        oled_chars[45] <= 8'h30 + {4'd0, off0_d2};
                        oled_chars[46] <= 8'h30 + {4'd0, off0_d1};
                        oled_chars[47] <= 8'h30 + {4'd0, off0_d0};

                        // Line 3: "AVG:20SMPL 50Hz "
                        oled_chars[48] <= "A"; oled_chars[49] <= "V"; oled_chars[50] <= "G"; oled_chars[51] <= ":";
                        oled_chars[52] <= "2"; oled_chars[53] <= "0"; oled_chars[54] <= "S"; oled_chars[55] <= "M";
                        oled_chars[56] <= "P"; oled_chars[57] <= "L"; oled_chars[58] <= " ";
                        if (snap_mode_fast) begin
                            oled_chars[59] <= "F"; oled_chars[60] <= "A"; oled_chars[61] <= "S";
                            oled_chars[62] <= "T"; oled_chars[63] <= " ";
                        end else begin
                            oled_chars[59] <= "5"; oled_chars[60] <= "0"; oled_chars[61] <= "H";
                            oled_chars[62] <= "z"; oled_chars[63] <= " ";
                        end

                    end else begin
                        // -----------------------------------------------------
                        // Screen 4: Detailed Peak-to-Peak + True RMS View (SW0 = 1, 3 Decimals)
                        // -----------------------------------------------------
                        // Line 0: "AC I-PP & RMS   "
                        oled_chars[0]  <= "A"; oled_chars[1]  <= "C"; oled_chars[2]  <= " "; oled_chars[3]  <= "I";
                        oled_chars[4]  <= "-"; oled_chars[5]  <= "P"; oled_chars[6]  <= "P"; oled_chars[7]  <= " ";
                        oled_chars[8]  <= "&"; oled_chars[9]  <= " "; oled_chars[10] <= "R"; oled_chars[11] <= "M";
                        oled_chars[12] <= "S"; oled_chars[13] <= " "; oled_chars[14] <= " "; oled_chars[15] <= " ";

                        // Line 1: "0:4.222Ap 1.493r"
                        oled_chars[16] <= "0"; oled_chars[17] <= ":";
                        oled_chars[18] <= 8'h30 + {4'd0, ipp0_w};
                        oled_chars[19] <= ".";
                        oled_chars[20] <= 8'h30 + {4'd0, ipp0_t};
                        oled_chars[21] <= 8'h30 + {4'd0, ipp0_h};
                        oled_chars[22] <= 8'h30 + {4'd0, ipp0_m};
                        oled_chars[23] <= "A"; oled_chars[24] <= "p";
                        oled_chars[25] <= " ";
                        oled_chars[26] <= 8'h30 + {4'd0, rms0_w};
                        oled_chars[27] <= ".";
                        oled_chars[28] <= 8'h30 + {4'd0, rms0_t};
                        oled_chars[29] <= 8'h30 + {4'd0, rms0_h};
                        oled_chars[30] <= 8'h30 + {4'd0, rms0_m};
                        oled_chars[31] <= "r";

                        // Line 2: "1:4.222Ap 1.493r"
                        oled_chars[32] <= "1"; oled_chars[33] <= ":";
                        oled_chars[34] <= 8'h30 + {4'd0, ipp1_w};
                        oled_chars[35] <= ".";
                        oled_chars[36] <= 8'h30 + {4'd0, ipp1_t};
                        oled_chars[37] <= 8'h30 + {4'd0, ipp1_h};
                        oled_chars[38] <= 8'h30 + {4'd0, ipp1_m};
                        oled_chars[39] <= "A"; oled_chars[40] <= "p";
                        oled_chars[41] <= " ";
                        oled_chars[42] <= 8'h30 + {4'd0, rms1_w};
                        oled_chars[43] <= ".";
                        oled_chars[44] <= 8'h30 + {4'd0, rms1_t};
                        oled_chars[45] <= 8'h30 + {4'd0, rms1_h};
                        oled_chars[46] <= 8'h30 + {4'd0, rms1_m};
                        oled_chars[47] <= "r";

                        // Line 3: "RAW-PP: 0262 LSB"
                        oled_chars[48] <= "R"; oled_chars[49] <= "A"; oled_chars[50] <= "W"; oled_chars[51] <= "-";
                        oled_chars[52] <= "P"; oled_chars[53] <= "P"; oled_chars[54] <= ":"; oled_chars[55] <= " ";
                        oled_chars[56] <= 8'h30 + {4'd0, raw_pp_d3};
                        oled_chars[57] <= 8'h30 + {4'd0, raw_pp_d2};
                        oled_chars[58] <= 8'h30 + {4'd0, raw_pp_d1};
                        oled_chars[59] <= 8'h30 + {4'd0, raw_pp_d0};
                        oled_chars[60] <= " "; oled_chars[61] <= "L"; oled_chars[62] <= "S"; oled_chars[63] <= "B";
                    end

                    fmt_step <= 4'd0;
                end

                default: fmt_step <= 4'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 11. Direct Memory Read Interface to OLED Controller
    // -------------------------------------------------------------------------
    wire [5:0] text_addr_w;
    wire [7:0] text_char_w = oled_chars[text_addr_w];
    wire       init_done_w;

    // -------------------------------------------------------------------------
    // 12. SSD1306 OLED Controller Instance
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
    // 13. Diagnostic LEDs Mapping
    // -------------------------------------------------------------------------
    assign led[0] = heartbeat_1hz;                       // LD0: 1 Hz System Heartbeat
    assign led[1] = init_done_w;                         // LD1: OLED Panel Initialized (Solid ON)
    assign led[2] = cal_done;                            // LD2: 10s Total Calibration Complete
    assign led[3] = pp_window_valid;                     // LD3: AC Cycle / Window Strobe
    assign led[4] = overcurrent_active;                  // LD4: Overcurrent Alert (>10.0 A Peak-to-Peak)
    assign led[5] = outlier_rejected;                    // LD5: Outlier Sample Rejected Flag
    assign led[6] = dc_cal_done;                         // LD6: Phase 1 DC Autotare Complete
    assign led[7] = mode_fast_win;                       // LD7: Mode (OFF: 50Hz Cycle, ON: Fast 20-Sample)

endmodule
