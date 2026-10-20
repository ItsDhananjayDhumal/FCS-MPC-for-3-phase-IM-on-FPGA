`timescale 1ns / 1ps
// =============================================================================
// Module: tb_ac_current_test
// Description: Comprehensive Self-Checking Testbench for AC Current Telemetry
//              - Emulates Digilent Pmod AD1 (AD7476A SAR ADC) SPI responses.
//              - Generates synthetic sinusoidal AC currents to verify:
//                Test 1: Power-on Reset & Warmup Countdown (5s countdown)
//                Test 2: Auto-Tare Sampling & Running Mean with Outlier Filtering
//                Test 3: Calibration Completion & Offset Calculation (adc_offset == 2048)
//                Test 4: Sinusoidal AC Current Injection (~4.22A Peak-to-Peak, 1.49A RMS)
//                Test 5: 20-Sample Moving Average Convergence (Buffer Fills to 20)
//                Test 6: Dynamic Amplitude Step (~4.22A PP -> ~1.61A PP)
//                Test 7: Telemetry Mode Switch (SW0 = 0 PP View vs SW0 = 1 RMS View)
//                Test 8: Manual BTNC Recalibration / Re-Tare Trigger
// =============================================================================

module tb_ac_current_test;

    // Clock and Control Signals
    reg        sys_clk_i;
    reg        cpu_resetn;
    reg        btnc;
    reg  [7:0] sw;

    // ADC Interface
    wire       adc_cs_n;
    reg        adc_d0;
    reg        adc_d1;
    wire       adc_sclk;

    // OLED Interface
    wire       oled_sclk;
    wire       oled_sdin;
    wire       oled_dc;
    wire       oled_res;
    wire       oled_vbat;
    wire       oled_vdd;

    // LEDs
    wire [7:0] led;

    // Emulated ADC Response Values
    reg [11:0] emu_adc_val_ch0;
    reg [11:0] emu_adc_val_ch1;

    // Instantiate Unit Under Test (UUT) with fast SIMULATION parameters
    ac_current_top #(
        .SIMULATION      (1'b1),
        .WARMUP_CYCLES   (29'd100),
        .DEBOUNCE_CYCLES (20'd50),
        .SAMPLE_PERIOD   (16'd50),
        .FMT_PERIOD      (22'd250)
    ) uut (
        .sys_clk_i   (sys_clk_i),
        .cpu_resetn  (cpu_resetn),
        .btnc        (btnc),
        .sw          (sw),
        .adc_cs_n    (adc_cs_n),
        .adc_d0      (adc_d0),
        .adc_d1      (adc_d1),
        .adc_sclk    (adc_sclk),
        .oled_sclk   (oled_sclk),
        .oled_sdin   (oled_sdin),
        .oled_dc     (oled_dc),
        .oled_res    (oled_res),
        .oled_vbat   (oled_vbat),
        .oled_vdd    (oled_vdd),
        .led         (led)
    );

    // 100 MHz System Clock Generation (10 ns period)
    always #5 sys_clk_i = ~sys_clk_i;

    // -------------------------------------------------------------------------
    // AC Waveform Generator (Sinusoidal Look-up Table: 20 samples per cycle)
    // Peak amplitude = 131 counts -> Peak-to-Peak = 262 counts (~4.22 A PP)
    // -------------------------------------------------------------------------
    reg signed [11:0] sin_table [0:19];
    integer s_idx;
    real ac_scale;

    initial begin
        s_idx         = 0;
        ac_scale      = 1.0;
        sin_table[0]  =  12'sd0;
        sin_table[1]  =  12'sd40;
        sin_table[2]  =  12'sd77;
        sin_table[3]  =  12'sd106;
        sin_table[4]  =  12'sd125;
        sin_table[5]  =  12'sd131; // Positive Peak: +131 counts
        sin_table[6]  =  12'sd125;
        sin_table[7]  =  12'sd106;
        sin_table[8]  =  12'sd77;
        sin_table[9]  =  12'sd40;
        sin_table[10] =  12'sd0;
        sin_table[11] = -12'sd40;
        sin_table[12] = -12'sd77;
        sin_table[13] = -12'sd106;
        sin_table[14] = -12'sd125;
        sin_table[15] = -12'sd131; // Negative Peak: -131 counts
        sin_table[16] = -12'sd125;
        sin_table[17] = -12'sd106;
        sin_table[18] = -12'sd77;
        sin_table[19] = -12'sd40;
    end

    // -------------------------------------------------------------------------
    // Emulated Digilent Pmod AD1 (Analog Devices AD7476A)
    // - Format: 4 leading zeros + 12-bit ADC data
    // - CS falling edge drives MSB (bit 15) onto line
    // - SCLK falling edge shifts out subsequent bits
    // -------------------------------------------------------------------------
    reg [15:0] adc_shift_ch0;
    reg [15:0] adc_shift_ch1;
    reg        adc_d0_r;
    reg        adc_d1_r;

    always @(*) begin
        adc_d0 = adc_cs_n ? 1'b0 : adc_d0_r;
        adc_d1 = adc_cs_n ? 1'b0 : adc_d1_r;
    end

    always @(negedge adc_cs_n) begin
        if (uut.state == 4'd6 || uut.state == 4'd7) begin // In measurement mode
            emu_adc_val_ch0 = 12'd2048 + $rtoi(sin_table[s_idx] * ac_scale);
            emu_adc_val_ch1 = 12'd2048 + $rtoi(sin_table[s_idx] * ac_scale);
            s_idx = (s_idx >= 19) ? 0 : (s_idx + 1);
        end
        adc_shift_ch0 = {4'b0000, emu_adc_val_ch0};
        adc_shift_ch1 = {4'b0000, emu_adc_val_ch1};
        adc_d0_r      = adc_shift_ch0[15];
        adc_d1_r      = adc_shift_ch1[15];
    end

    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            adc_shift_ch0 = adc_shift_ch0 << 1;
            adc_shift_ch1 = adc_shift_ch1 << 1;
            adc_d0_r      = adc_shift_ch0[15];
            adc_d1_r      = adc_shift_ch1[15];
        end
    end

    // Test Assertions Counter
    integer tests_passed = 0;
    integer tests_failed = 0;

    task check(input [255:0] name, input condition);
        begin
            if (condition) begin
                $display("  [PASS] %0s", name);
                tests_passed = tests_passed + 1;
            end else begin
                $display("  [FAIL] %0s", name);
                tests_failed = tests_failed + 1;
            end
        end
    endtask





    // -------------------------------------------------------------------------
    // Main Verification Procedure
    // -------------------------------------------------------------------------
    integer cyc;

    initial begin
        $display("=================================================================");
        $display("STARTING AC CURRENT PEAK-TO-PEAK 20-SAMPLE AVERAGE TESTBENCH");
        $display("=================================================================");

        // Initial conditions
        sys_clk_i        = 0;
        cpu_resetn       = 0;
        btnc             = 0;
        sw               = 8'd0;
        adc_d0           = 0;
        adc_d1           = 0;
        s_idx            = 0;
        ac_scale         = 1.0;
        emu_adc_val_ch0  = 12'd2048; // Quiescent 0A during calibration
        emu_adc_val_ch1  = 12'd2048;

        // Reset Pulse
        #100;
        cpu_resetn = 1;
        #100;

        // ---------------------------------------------------------------------
        // TEST 1: Warmup Countdown State
        // ---------------------------------------------------------------------
        $display("\n--- TEST 1: Power-on Reset & Warmup Countdown ---");
        check("State is S_CAL_WARMUP", uut.state == 4'd1);
        check("Warmup counter is running", uut.warmup_cnt > 0);
        check("Cal done is 0", uut.cal_done == 0);

        // Wait for warmup to complete (100 cycles in simulation)
        wait(uut.state == 4'd2); // S_CAL_ADC
        check("Warmup completed and entered S_CAL_ADC", uut.state == 4'd2);

        // ---------------------------------------------------------------------
        // TEST 2: Auto-Tare Sampling & Running Mean with Outlier Filtering
        // ---------------------------------------------------------------------
        $display("\n--- TEST 2: Auto-Tare Sampling & Outlier Rejection ---");
        wait(uut.cal_valid_cnt >= 16'd50);
        check("Accumulated >= 50 valid samples", uut.cal_valid_cnt >= 16'd50);
        check("Running mean initialized to 2048", uut.cal_mean_ch0 == 13'sd2048);

        // Inject 5 OUTLIER samples (> 100 counts away from mean, e.g. 2300)
        emu_adc_val_ch0 = 12'd2300;
        #5000;
        check("Outlier sample was rejected (LED5 flag active)", uut.outlier_rejected == 1'b1);
        $display("  Outlier rejection verified: cal_valid_cnt = %0d, sample_cnt = %0d", 
                 uut.cal_valid_cnt, uut.cal_sample_cnt);

        // Restore normal 2048 samples
        emu_adc_val_ch0 = 12'd2048;

        // ---------------------------------------------------------------------
        // TEST 3: Calibration Completion & Offset Calculation
        // ---------------------------------------------------------------------
        $display("\n--- TEST 3: Two-Phase Calibration Completion & Offset Calculation ---");
        wait(uut.dc_cal_done == 1'b1);
        check("Phase 1 DC Cal done flag asserted", uut.dc_cal_done == 1'b1);
        check("Calibrated offset CH0 is 2048", uut.adc_offset_ch0 == 13'sd2048);
        check("Calibrated offset CH1 is 2048", uut.adc_offset_ch1 == 13'sd2048);
        check("LED6 (dc_cal_done indicator) is ON", led[6] == 1'b1);

        wait(uut.cal_done == 1'b1);
        check("Phase 2 AC Noise Cal complete (cal_done asserted)", uut.cal_done == 1'b1);
        check("FSM transitioned to S_MEASURE", uut.state == 4'd6);
        check("LED2 (cal_done indicator) is ON", led[2] == 1'b1);

        // ---------------------------------------------------------------------
        // TEST 4 & 5: AC Sinusoidal Current & 20-Sample Moving Average
        // ---------------------------------------------------------------------
        $display("\n--- TEST 4 & 5: AC Current & 20-Sample Moving Average ---");
        // Run for 22 AC cycles (each cycle is 20 samples = 20 * 1780 ns = ~35.6 us)
        for (cyc = 0; cyc < 22; cyc = cyc + 1) begin
            wait(uut.pp_window_valid == 1'b1);
            wait(uut.pp_window_valid == 1'b0);
        end

        #10000;
        $display("  Averaging buffer fill_cnt: %0d / 20", uut.fill_cnt);
        $display("  Average Raw Peak-to-Peak: %0d LSB (Expected: ~262)", uut.avg_pp_ch0);
        $display("  Physical I_pp: %0d mA (~4.222 A)", uut.i_pp_ma_ch0);
        $display("  Calculated I_rms: %0d mA (~1.493 A)", uut.i_rms_ma_ch0);

        check("Moving average buffer is full (fill_cnt == 20)", uut.fill_cnt == 5'd20);
        check("Average Raw Peak-to-Peak in range [260, 264] counts", 
              uut.avg_pp_ch0 >= 12'd260 && uut.avg_pp_ch0 <= 12'd264);
        check("Peak-to-Peak Current CH0 is ~4.22 A (4190..4250 mA)", 
              uut.i_pp_ma_ch0 >= 16'd4190 && uut.i_pp_ma_ch0 <= 16'd4250);
        check("RMS Current CH0 is ~1.49 A (1470..1510 mA)", 
              uut.i_rms_ma_ch0 >= 16'd1470 && uut.i_rms_ma_ch0 <= 16'd1510);
        check("Overcurrent inactive (< 10A PP)", uut.overcurrent_active == 1'b0);
        check("Line 1 Formatted with 3 Decimals: 0:4.222A", 
              uut.oled_chars[16] == "0" && uut.oled_chars[17] == ":" &&
              uut.oled_chars[18] == "4" && uut.oled_chars[19] == "." &&
              uut.oled_chars[20] == "2" && uut.oled_chars[21] == "2" &&
              uut.oled_chars[22] == "2" && uut.oled_chars[23] == "A");

        // ---------------------------------------------------------------------
        // TEST 6: Dynamic Amplitude Step (Step down to ~1.61 A Peak-to-Peak)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 6: Dynamic Amplitude Step Response ---");
        // Step AC amplitude to 100 counts peak-to-peak (~1.61 A PP)
        // 100 / 262 = 0.3817
        ac_scale = 0.3817;

        // Run for 22 AC cycles to completely flush the moving average buffer
        for (cyc = 0; cyc < 22; cyc = cyc + 1) begin
            wait(uut.pp_window_valid == 1'b1);
            wait(uut.pp_window_valid == 1'b0);
        end

        #10000;
        $display("  Post-step Raw Peak-to-Peak: %0d LSB (Expected: ~100)", uut.avg_pp_ch0);
        $display("  Post-step Physical I_pp: %0d mA (~1.61 A)", uut.i_pp_ma_ch0);
        check("Raw Peak-to-Peak settled to ~100 counts", uut.avg_pp_ch0 >= 12'd98 && uut.avg_pp_ch0 <= 12'd102);
        check("Peak-to-Peak Current settled to ~1.61 A (1580..1640 mA)", 
              uut.i_pp_ma_ch0 >= 16'd1580 && uut.i_pp_ma_ch0 <= 16'd1640);
        check("Line 1 Formatted with 3 Decimals: 0:1.61", 
              uut.oled_chars[16] == "0" && uut.oled_chars[17] == ":" &&
              uut.oled_chars[18] == "1" && uut.oled_chars[19] == "." &&
              uut.oled_chars[20] == "6" && uut.oled_chars[21] == "1");

        // ---------------------------------------------------------------------
        // TEST 7: Telemetry Mode Switch (SW0)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 7: Telemetry Mode Switch ---");
        check("Initial view is Peak-to-Peak (SW0=0)", uut.mode_rms_view == 1'b0);
        sw[0] = 1'b1;
        #25000;
        check("Switched to Detailed RMS View (SW0=1)", uut.mode_rms_view == 1'b1);
        check("Line 0 Displays AC I-PP & RMS", uut.oled_chars[0] == "A" && uut.oled_chars[8] == "&");
        sw[0] = 1'b0;
        #25000;
        check("Restored to Peak-to-Peak View (SW0=0)", uut.mode_rms_view == 1'b0);

        // ---------------------------------------------------------------------
        // TEST 8: Manual BTNC Recalibration / Re-Tare Trigger
        // ---------------------------------------------------------------------
        $display("\n--- TEST 8: Manual Pushbutton BTNC Recalibration ---");
        // Press BTNC
        btnc = 1'b1;
        #600;
        btnc = 1'b0;
        #500;

        check("Calibration restarted (cal_done is 0)", uut.cal_done == 1'b0);
        check("Moving average buffer reset (fill_cnt == 0)", uut.fill_cnt == 5'd0);
        check("FSM re-entered warmup", uut.state == 4'd1 || uut.state == 4'd2);

        // Wait for re-calibration to finish
        wait(uut.dc_cal_done == 1'b1);
        check("Phase 1 re-calibration completed (dc_cal_done asserted)", uut.dc_cal_done == 1'b1);
        wait(uut.cal_done == 1'b1);
        check("Phase 2 re-calibration completed (cal_done asserted)", uut.cal_done == 1'b1);
        check("OLED SPI Clock is active", oled_sclk !== 1'bx);

        // ---------------------------------------------------------------------
        // Final Summary
        // ---------------------------------------------------------------------
        $display("\n=================================================================");
        $display("VERIFICATION COMPLETE");
        $display("Total Assertions Passed: %0d", tests_passed);
        $display("Total Assertions Failed: %0d", tests_failed);
        if (tests_failed == 0) begin
            $display(">>> ALL ASSERTIONS PASSED (100%% SUCCESS) <<<");
        end else begin
            $display(">>> VERIFICATION FAILED! <<<");
        end
        $display("=================================================================");

        $finish;
    end

endmodule
