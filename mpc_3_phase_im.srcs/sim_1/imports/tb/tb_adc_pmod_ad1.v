`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_adc_pmod_ad1;

    // -------------------------------------------------------------
    // 1. Clock and Reset Generation
    // -------------------------------------------------------------
    reg clk;
    reg rst_n;
    reg start;

    initial clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz system clock (10 ns period)

    // -------------------------------------------------------------
    // 2. DUT Interface Signals
    // -------------------------------------------------------------
    wire       adc_cs_n;
    wire       adc_sclk;
    wire       adc_d0;
    wire       adc_d1;
    wire [11:0] data_ch0;
    wire [11:0] data_ch1;
    wire       done;

    // -------------------------------------------------------------
    // 3. DUT Instantiation
    // -------------------------------------------------------------
    adc_pmod_ad1 #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (start),
        .adc_d0   (adc_d0),
        .adc_d1   (adc_d1),
        .adc_cs_n (adc_cs_n),
        .adc_sclk (adc_sclk),
        .data_ch0 (data_ch0),
        .data_ch1 (data_ch1),
        .done     (done)
    );

    // -------------------------------------------------------------
    // 4. Hardware Emulation Model: Dual AD7476A (Pmod AD1)
    // -------------------------------------------------------------
    // Emulates true AD7476A SPI behavior with realistic propagation delay
    reg [11:0] mock_adc_val0;
    reg [11:0] mock_adc_val1;
    reg [15:0] shift_reg0;
    reg [15:0] shift_reg1;
    reg        dout0_reg;
    reg        dout1_reg;
    integer    spi_bit_idx;

    // Tri-state drivers when CS is unasserted (high)
    assign adc_d0 = adc_cs_n ? 1'bz : dout0_reg;
    assign adc_d1 = adc_cs_n ? 1'bz : dout1_reg;

    // Falling edge of CS_N: Freeze sample and present MSB (leading zero)
    always @(negedge adc_cs_n) begin
        // AD7476A sends 4 leading zeros + 12-bit data MSB-first
        shift_reg0  = {4'b0000, mock_adc_val0};
        shift_reg1  = {4'b0000, mock_adc_val1};
        spi_bit_idx = 0;
        
        // Output MSB after t_CONVERT / access time (~15 ns)
        #15;
        dout0_reg = shift_reg0[15];
        dout1_reg = shift_reg1[15];
    end

    // Falling edge of SCLK: Shift out next bit with datasheet propagation delay (t4 ~ 15-22 ns)
    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            spi_bit_idx = spi_bit_idx + 1;
            if (spi_bit_idx < 16) begin
                // Realistic data access delay from SCLK falling edge
                #18;
                shift_reg0 = shift_reg0 << 1;
                shift_reg1 = shift_reg1 << 1;
                dout0_reg  = shift_reg0[15];
                dout1_reg  = shift_reg1[15];
            end
        end
    end

    // -------------------------------------------------------------
    // 5. Test Tracking and Diagnostics
    // -------------------------------------------------------------
    integer total_samples_tested = 0;
    integer passed_samples       = 0;
    integer failed_samples       = 0;
    integer quiet_violations     = 0;

    // Physical Protocol Assertion: Verify t_QUIET >= 40 ns is never violated
    time last_cs_rise_time = 0;
    always @(posedge adc_cs_n) begin
        last_cs_rise_time = $time;
    end

    always @(negedge adc_cs_n) begin
        if (rst_n && last_cs_rise_time > 0) begin
            if (($time - last_cs_rise_time) < 40) begin
                $display("[ERROR %0t] t_QUIET VIOLATED! CS_N high time was only %0d ns (min 40 ns required!)",
                         $time, ($time - last_cs_rise_time));
                quiet_violations = quiet_violations + 1;
            end
        end
    end

    // Timeout watchdog: ensure testbench never hangs indefinitely
    initial begin
        #50_000_000; // 50 ms timeout
        $display("\n[FATAL TIMEOUT] Testbench exceeded 50 ms without completion!");
        $finish;
    end

    // Task to trigger and verify a single conversion
    task run_sample_test;
        input [11:0] ch0_expected;
        input [11:0] ch1_expected;
        input integer idle_clks_before_start;
        input [127:0] description;
        integer timeout_cnt;
        reg sample_ok;
    begin
        // 1. Wait the requested idle cycles
        if (idle_clks_before_start > 0) begin
            repeat (idle_clks_before_start) @(posedge clk);
        end

        // 2. Set the expected ADC analog values on the mock ADC
        mock_adc_val0 = ch0_expected;
        mock_adc_val1 = ch1_expected;

        // 3. Pulse start for exactly 1 clock cycle
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // 4. Wait for done with timeout detection
        timeout_cnt = 0;
        while (!done && timeout_cnt < 250) begin // 1 conversion is ~135 clks
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end

        total_samples_tested = total_samples_tested + 1;

        if (timeout_cnt >= 250) begin
            $display("[FAIL #%0d] TIMEOUT: done signal never asserted! (%s)", 
                     total_samples_tested, description);
            failed_samples = failed_samples + 1;
        end else begin
            // 5. Verify received data matches expected data
            sample_ok = (data_ch0 === ch0_expected) && (data_ch1 === ch1_expected);
            if (sample_ok) begin
                passed_samples = passed_samples + 1;
                if (total_samples_tested <= 10 || total_samples_tested == 64 || 
                    total_samples_tested == 128 || total_samples_tested >= 155) begin
                    $display("[PASS #%03d] CH0: %04d (0x%03h) | CH1: %04d (0x%03h) | %s",
                             total_samples_tested, data_ch0, data_ch0, data_ch1, data_ch1, description);
                end
            end else begin
                failed_samples = failed_samples + 1;
                $display("[FAIL #%03d] MISMATCH! (%s)", total_samples_tested, description);
                $display("          CH0 Expected: %04d (0x%03h), Got: %04d (0x%03h)",
                         ch0_expected, ch0_expected, data_ch0, data_ch0);
                $display("          CH1 Expected: %04d (0x%03h), Got: %04d (0x%03h)",
                         ch1_expected, ch1_expected, data_ch1, data_ch1);
            end
        end
    end
    endtask

    // -------------------------------------------------------------
    // 6. Main Test Sequence
    // -------------------------------------------------------------
    integer i;
    reg [11:0] val_a, val_b;

    initial begin
        $display("===================================================================");
        $display("         STARTING Pmod AD1 (AD7476A) RIGOROUS VERIFICATION TB      ");
        $display("===================================================================");

        // --- Step 0: Power-on Reset ---
        rst_n = 1'b0;
        start = 1'b0;
        mock_adc_val0 = 12'd0;
        mock_adc_val1 = 12'd0;
        dout0_reg = 1'b0;
        dout1_reg = 1'b0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // Verify initial idle state
        if (adc_cs_n !== 1'b1 || adc_sclk !== 1'b0 || done !== 1'b0) begin
            $display("[FAIL] Reset state invalid: CS_N=%b, SCLK=%b, done=%b", 
                     adc_cs_n, adc_sclk, done);
            failed_samples = failed_samples + 1;
        end else begin
            $display("[PASS] Reset state verified: CS_N=1 (idle), SCLK=0, done=0.");
        end

        // =================================================================
        // PHASE 1: 128 SAMPLES (Rigorous Edge Cases, Patterns & Ramps)
        // =================================================================
        $display("\n--- PHASE 1: Running 128 Core Samples (Edge Cases & Patterns) ---");

        // Edge Case 1: Minimum value (All zeros)
        run_sample_test(12'h000, 12'h000, 5, "Edge: Minimum (0)");

        // Edge Case 2: Maximum value (All ones)
        run_sample_test(12'hFFF, 12'hFFF, 5, "Edge: Maximum (4095)");

        // Edge Case 3: Mid-scale baseline (2048 - Quiescent current zero)
        run_sample_test(12'h800, 12'h800, 5, "Edge: Mid-scale 0A (2048)");

        // Edge Case 4: Mid-scale - 1 LSB
        run_sample_test(12'h7FF, 12'h7FF, 5, "Edge: Mid-scale - 1 (2047)");

        // Edge Case 5: Mid-scale + 1 LSB
        run_sample_test(12'h801, 12'h801, 5, "Edge: Mid-scale + 1 (2049)");

        // Edge Case 6: Min + 1 LSB
        run_sample_test(12'h001, 12'h001, 5, "Edge: Min + 1 (1)");

        // Edge Case 7: Max - 1 LSB
        run_sample_test(12'hFFE, 12'hFFE, 5, "Edge: Max - 1 (4094)");

        // Edge Case 8: Alternating pattern A (1010_1010_1010)
        run_sample_test(12'hAAA, 12'hAAA, 5, "Pattern: 0xAAA (Checkerboard)");

        // Edge Case 9: Alternating pattern B (0101_0101_0101)
        run_sample_test(12'h555, 12'h555, 5, "Pattern: 0x555 (Inverted Checkerboard)");

        // Edge Case 10: Asymmetric Channels (Independent dual ADCs)
        run_sample_test(12'h123, 12'hEBA, 5, "Asymmetric: Ch0 != Ch1");

        // Edge Cases 11-22: Walking 1s (Single bit active across all 12 positions)
        for (i = 0; i < 12; i = i + 1) begin
            val_a = (12'b1 << i);
            val_b = (12'b1 << (11 - i));
            run_sample_test(val_a, val_b, 5, "Walking 1s test");
        end

        // Edge Cases 23-34: Walking 0s (Single bit zero across all 12 positions)
        for (i = 0; i < 12; i = i + 1) begin
            val_a = ~(12'b1 << i);
            val_b = ~(12'b1 << (11 - i));
            run_sample_test(val_a, val_b, 5, "Walking 0s test");
        end

        // Samples 35 to 80: Linear Ramp across input dynamic range
        for (i = 0; i < 46; i = i + 1) begin
            val_a = i * 88;       // Spanning 0 to 3960
            val_b = 4095 - val_a; // Complementary ramp
            run_sample_test(val_a, val_b, 5, "Dynamic range ramp");
        end

        // Samples 81 to 128: Pseudo-random combinations across full 12-bit range
        for (i = 81; i <= 128; i = i + 1) begin
            val_a = $urandom_range(0, 4095);
            val_b = $urandom_range(0, 4095);
            run_sample_test(val_a, val_b, 5, "Pseudo-random 12-bit verification");
        end

        $display(">>> Completed Phase 1: Total %0d samples taken. <<<", total_samples_tested);

        // =================================================================
        // PHASE 2: 32 SAMPLES WITH VARYING / RANDOM & CONSECUTIVE TIMING
        // =================================================================
        $display("\n--- PHASE 2: Running 32 Samples with Consecutive & Random Timing ---");

        // Group A: 16 Consecutive Back-to-Back Conversions (testing delays 0, 1, 2, 3, 4, 5 clks)
        // Tests maximum sampling throughput and zero-slack re-triggering across the 40ns quiet window
        $display("-> Sub-phase 2A: 16 Consecutive Back-to-Back samples (0-5 clk spacing across QUIET window)");
        for (i = 1; i <= 16; i = i + 1) begin
            val_a = 2048 + (i * 50);
            val_b = 2048 - (i * 50);
            // Tests delays: 0 (immediate), 1, 2, 3, 4, 5 clock cycles
            run_sample_test(val_a, val_b, ((i - 1) % 6), "Consecutive Back-to-Back Sample");
        end

        // Group B: 16 Samples with Random Time Intervals (5 to 300 clk cycles)
        // Tests asynchronous start triggers, variable idle times, clock domain jitter
        $display("-> Sub-phase 2B: 16 Random Interval samples (5 to 300 clock cycles spacing)");
        for (i = 1; i <= 16; i = i + 1) begin
            val_a = $urandom_range(100, 3900);
            val_b = $urandom_range(100, 3900);
            run_sample_test(val_a, val_b, $urandom_range(5, 300), "Random Interval Sample");
        end

        // =================================================================
        // PHASE 3: EDGE CASE SPECIAL - Mid-Transaction Reset Recovery
        // =================================================================
        $display("\n--- PHASE 3: Mid-Transaction Reset Recovery Verification ---");
        mock_adc_val0 = 12'hA5A;
        mock_adc_val1 = 12'h5A5;
        
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // Wait until halfway through SPI transaction (~60 clock cycles)
        repeat (60) @(posedge clk);
        $display("[%0t] Asserting rst_n LOW in the middle of active SPI conversion...", $time);
        rst_n <= 1'b0;

        repeat (4) @(posedge clk);
        if (adc_cs_n !== 1'b1 || adc_sclk !== 1'b0 || done !== 1'b0) begin
            $display("[FAIL] Mid-transaction reset failed to force CS_N=1, SCLK=0!");
            failed_samples = failed_samples + 1;
        end else begin
            $display("[PASS] Mid-transaction reset cleanly aborted conversion and parked lines.");
            passed_samples = passed_samples + 1;
        end

        // Release reset and perform a clean conversion to verify complete recovery
        rst_n <= 1'b1;
        repeat (10) @(posedge clk);
        run_sample_test(12'h3C9, 12'hC36, 10, "Post-Reset Recovery Sample");

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("                 PMOD AD1 TESTBENCH EXECUTION SUMMARY              ");
        $display("===================================================================");
        $display(" Total Conversion Samples Evaluated : %0d", total_samples_tested);
        $display(" Passed Samples                     : %0d", passed_samples);
        $display(" Failed Samples                     : %0d", failed_samples);
        $display(" t_QUIET Timing Violations (<40ns)  : %0d", quiet_violations);
        $display("===================================================================");

        if (failed_samples == 0 && quiet_violations == 0) begin
            $display(" >>> ALL 160+ TESTS PASSED PERFECTLY! ZERO ERRORS DETECTED. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d SAMPLE ERRORS, %0d TIMING VIOLATIONS! <<< \n", 
                     failed_samples, quiet_violations);
        end

        $finish;
    end

endmodule
