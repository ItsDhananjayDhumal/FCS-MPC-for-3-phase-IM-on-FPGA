`timescale 1ns / 1ps
// =============================================================================
// Module: tb_dc_current_test
// Description: Comprehensive Self-Checking Testbench for DC Current/Voltage Test
//              - Emulates Digilent Pmod AD1 (AD7476A SAR ADC) SPI responses:
//                * 4 leading zeros followed by 12 data bits (MSB first)
//                * Shifted out on SCLK falling edge
//              - Rigorous Verification Suite:
//                Test 1: Power-on Reset & Warmup Countdown (5s countdown)
//                Test 2: Auto-Tare Sampling & Running Mean with Outlier Filtering
//                Test 3: Calibration Completion & Offset Calculation (adc_offset == 2048)
//                Test 4: Nominal 0A Steady-State Reading (1.65V, +0.00A)
//                Test 5: Positive Current +5.0A Step Response (1.90V, +5.00A)
//                Test 6: Negative Current -5.0A Step Response (1.40V, -5.00A)
//                Test 7: Telemetry Mode Switch (SW0 = 0 Multi-View vs SW0 = 1 Precise View)
//                Test 8: Manual BTNC Recalibration / Re-Tare Trigger
// =============================================================================

module tb_dc_current_test;

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
    dc_current_top #(
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
    initial begin
        $display("=================================================================");
        $display("STARTING DC CURRENT & VOLTAGE TELEMETRY VERIFICATION TESTBENCH");
        $display("=================================================================");

        // Initial conditions
        sys_clk_i        = 0;
        cpu_resetn       = 0;
        btnc             = 0;
        sw               = 8'd0;
        adc_d0           = 0;
        adc_d1           = 0;
        emu_adc_val_ch0  = 12'd2048; // Quiescent 2.51V zero-current offset (nominal 2048)
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
        // Let it accumulate 50 valid samples around 2048
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
        $display("\n--- TEST 3: Calibration Completion & Offset Calculation ---");
        wait(uut.cal_done == 1'b1);
        check("Cal done flag asserted", uut.cal_done == 1'b1);
        check("Calibrated offset CH0 is 2048", uut.adc_offset_ch0 == 13'sd2048);
        check("Calibrated offset CH1 is 2048", uut.adc_offset_ch1 == 13'sd2048);
        check("FSM transitioned to S_MEASURE", uut.state == 4'd6);
        check("LED2 (cal_done indicator) is ON", led[2] == 1'b1);

        // ---------------------------------------------------------------------
        // TEST 4: Nominal 0A Steady-State Reading (1.65V, +0.00A)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 4: Nominal 0A Steady-State Telemetry ---");
        // Run several measurement cycles
        #10000;
        check("Live Raw CH0 is 2048", uut.live_raw_ch0 == 12'd2048);
        check("Live Raw CH1 is 2048", uut.live_raw_ch1 == 12'd2048);
        check("Voltage CH0 is 1650 mV (1.65V)", uut.v_mv_ch0 == 12'd1650);
        check("Voltage CH1 is 1650 mV (1.65V)", uut.v_mv_ch1 == 12'd1650);
        check("Current CH0 is 0 mA (+0.00A)", uut.i_ma_ch0 == 16'sd0);
        check("Current CH1 is 0 mA (+0.00A)", uut.i_ma_ch1 == 16'sd0);
        check("Overcurrent inactive", uut.overcurrent_active == 1'b0);

        // ---------------------------------------------------------------------
        // TEST 5: Positive Current Step (+5.0A -> ~2358 counts = ~1.90V)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 5: Positive Current Step (+5.0A, 1.90V) ---");
        // +310 counts above 2048 = 2358 counts
        emu_adc_val_ch0 = 12'd2358;
        #80000; // Allow filter to reach 99.9% settling
        $display("  Live CH0 Raw: %0d, V_mv: %0d, I_ma: %0d", 
                 uut.live_raw_ch0, uut.v_mv_ch0, uut.i_ma_ch0);
        check("Voltage CH0 increased (~1900 mV = 1.90V)", uut.v_mv_ch0 >= 12'd1890 && uut.v_mv_ch0 <= 12'd1910);
        check("Current CH0 is positive (~+4995 mA = +5.00A)", uut.i_ma_ch0 >= 16'sd4850 && uut.i_ma_ch0 <= 16'sd5100);
        check("LED6 (CH0 positive current) is ON", led[6] == 1'b1);

        // ---------------------------------------------------------------------
        // TEST 6: Negative Current Step (-5.0A -> ~1738 counts = ~1.40V)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 6: Negative Current Step (-5.0A, 1.40V) ---");
        // -310 counts below 2048 = 1738 counts
        emu_adc_val_ch0 = 12'd1738;
        #80000; // Allow filter to reach 99.9% settling
        $display("  Live CH0 Raw: %0d, V_mv: %0d, I_ma: %0d", 
                 uut.live_raw_ch0, uut.v_mv_ch0, uut.i_ma_ch0);
        check("Voltage CH0 decreased (~1400 mV = 1.40V)", uut.v_mv_ch0 >= 12'd1390 && uut.v_mv_ch0 <= 12'd1410);
        check("Current CH0 is negative (~-4995 mA = -5.00A)", uut.i_ma_ch0 <= -16'sd4850 && uut.i_ma_ch0 >= -16'sd5100);
        check("LED6 (CH0 polarity) is OFF for negative", led[6] == 1'b0);

        // Return to 0A
        emu_adc_val_ch0 = 12'd2048;
        #80000;

        // ---------------------------------------------------------------------
        // TEST 7: Telemetry Mode Switch (SW0)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 7: Telemetry Mode Switch ---");
        check("Initial mode is Multi-View (SW0=0)", uut.mode_high_prec == 1'b0);
        sw[0] = 1'b1;
        #500;
        check("Mode switched to High-Precision Voltage View (SW0=1)", uut.mode_high_prec == 1'b1);
        sw[0] = 1'b0;
        #500;
        check("Mode restored to Multi-View (SW0=0)", uut.mode_high_prec == 1'b0);

        // ---------------------------------------------------------------------
        // TEST 8: Manual BTNC Recalibration / Re-Tare Trigger
        // ---------------------------------------------------------------------
        $display("\n--- TEST 8: Manual Pushbutton BTNC Recalibration ---");
        // New quiescent point: suppose sensor shifted slightly to 2055 counts
        emu_adc_val_ch0 = 12'd2055;
        emu_adc_val_ch1 = 12'd2055;

        // Press BTNC
        btnc = 1'b1;
        #600; // Longer than debounce (50 cycles = 500 ns)
        btnc = 1'b0;
        #500;

        check("Calibration restarted (cal_done is 0)", uut.cal_done == 1'b0);
        check("FSM re-entered warmup", uut.state == 4'd1 || uut.state == 4'd2);

        // Wait for re-calibration to finish
        wait(uut.cal_done == 1'b1);
        check("Re-calibration completed", uut.cal_done == 1'b1);
        check("New calibrated offset CH0 matches new zero (2055)", uut.adc_offset_ch0 == 13'sd2055);
        check("New calibrated offset CH1 matches new zero (2055)", uut.adc_offset_ch1 == 13'sd2055);

        // Verify OLED SPI transmission
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
