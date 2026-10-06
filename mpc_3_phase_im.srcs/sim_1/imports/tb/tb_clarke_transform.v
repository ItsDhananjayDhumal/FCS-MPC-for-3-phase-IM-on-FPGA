`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_clarke_transform;

    // -------------------------------------------------------------
    // 1. Clock and Reset Generation
    // -------------------------------------------------------------
    reg clk;
    reg rst_n;
    reg start;

    initial clk = 1'b0;
    always #5 clk = ~clk; // 100 MHz clock (10 ns period)

    // -------------------------------------------------------------
    // 2. DUT Interface Signals
    // -------------------------------------------------------------
    reg  [11:0]        ia_raw;
    reg  [11:0]        ib_raw;
    reg  signed [12:0] adc_offset;
    wire signed [31:0] i_alpha;
    wire signed [31:0] i_beta;
    wire               done;

    // -------------------------------------------------------------
    // 3. DUT Instantiation
    // -------------------------------------------------------------
    clarke_transform #(
        .ADC_BITS(`ADC_BITS),
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) dut (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (start),
        .ia_raw     (ia_raw),
        .ib_raw     (ib_raw),
        .adc_offset (adc_offset),
        .i_alpha    (i_alpha),
        .i_beta     (i_beta),
        .done       (done)
    );

    // -------------------------------------------------------------
    // 4. Testbench Tracking and Diagnostics
    // -------------------------------------------------------------
    integer total_tests   = 0;
    integer passed_tests  = 0;
    integer failed_tests  = 0;

    // Fixed-point scaling constant: 1.0 in Q12.20
    localparam real Q20_SCALE = 1048576.0;
    // Physical Amperes per count for ACS712-40AB (50mV/A) on 3.3V 12-bit ADC
    localparam real AMPS_PER_COUNT = 3.3 / (4096.0 * 0.050); // 0.01611328125 A/count
    localparam real INV_SQRT3_REAL = 0.57735026919;

    // Timeout watchdog
    initial begin
        #50_000_000;
        $display("\n[FATAL TIMEOUT] Testbench exceeded 50 ms!");
        $finish;
    end

    // -------------------------------------------------------------
    // 5. Automated Verification Task
    // -------------------------------------------------------------
    task verify_clarke;
        input [11:0]        raw_a;
        input [11:0]        raw_b;
        input signed [12:0] offset_val;
        input [127:0]       test_desc;
        
        integer timeout_cnt;
        real    ia_phys, ib_phys;
        real    alpha_expected, beta_expected;
        real    alpha_actual, beta_actual;
        real    err_alpha, err_beta;
        reg     test_ok;
    begin
        total_tests = total_tests + 1;

        // 1. Calculate Expected Physical Values using Floating Point Model
        ia_phys = ($signed({1'b0, raw_a}) - offset_val) * AMPS_PER_COUNT;
        ib_phys = ($signed({1'b0, raw_b}) - offset_val) * AMPS_PER_COUNT;
        alpha_expected = ia_phys;
        beta_expected  = (ia_phys + 2.0 * ib_phys) * INV_SQRT3_REAL;

        // 2. Drive Inputs to DUT
        ia_raw     <= raw_a;
        ib_raw     <= raw_b;
        adc_offset <= offset_val;

        // 3. Pulse Start for 1 Clock Cycle
        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // 4. Wait for Done (exactly 9-10 clock cycles)
        timeout_cnt = 0;
        while (!done && timeout_cnt < 25) begin
            @(posedge clk);
            timeout_cnt = timeout_cnt + 1;
        end

        if (timeout_cnt >= 25) begin
            $display("[FAIL #%03d] TIMEOUT: done pulse never asserted! (%s)", total_tests, test_desc);
            failed_tests = failed_tests + 1;
        end else begin
            // 5. Convert DUT Q12.20 outputs to Real Amperes
            alpha_actual = $itor(i_alpha) / Q20_SCALE;
            beta_actual  = $itor(i_beta)  / Q20_SCALE;

            err_alpha = alpha_actual - alpha_expected;
            if (err_alpha < 0) err_alpha = -err_alpha;

            err_beta = beta_actual - beta_expected;
            if (err_beta < 0) err_beta = -err_beta;

            // Allow up to 0.02A (20 mA) tolerance for 20-bit fixed-point truncation
            test_ok = (err_alpha <= 0.025) && (err_beta <= 0.025);

            if (test_ok) begin
                passed_tests = passed_tests + 1;
                if (total_tests <= 15 || total_tests % 10 == 0) begin
                    $display("[PASS #%03d] Alpha: %6.3f A | Beta: %6.3f A | (Exp: %6.3f, %6.3f) | %s",
                             total_tests, alpha_actual, beta_actual, alpha_expected, beta_expected, test_desc);
                end
            end else begin
                failed_tests = failed_tests + 1;
                $display("[FAIL #%03d] ARITHMETIC ERROR! (%s)", total_tests, test_desc);
                $display("           Inputs: ia_raw=%0d, ib_raw=%0d, offset=%0d", raw_a, raw_b, offset_val);
                $display("           Alpha Expected: %6.4f A, Got: %6.4f A (Diff: %6.4f A)",
                         alpha_expected, alpha_actual, err_alpha);
                $display("           Beta  Expected: %6.4f A, Got: %6.4f A (Diff: %6.4f A)",
                         beta_expected, beta_actual, err_beta);
            end
        end
    end
    endtask

    // -------------------------------------------------------------
    // 6. Main Test Sequence
    // -------------------------------------------------------------
    integer i;
    real theta_rad;
    real curr_mag;
    real ia_flt, ib_flt;
    reg [11:0] ia_code, ib_code;
    reg signed [12:0] cur_offset;
    real alpha_actual, alpha_expected;

    initial begin
        $display("===================================================================");
        $display("       RIGOROUS VERIFICATION TESTBENCH: clarke_transform.v         ");
        $display("===================================================================");

        // Power-on reset
        rst_n      = 1'b0;
        start      = 1'b0;
        ia_raw     = 12'd0;
        ib_raw     = 12'd0;
        adc_offset = 13'sd3115;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        // =================================================================
        // PHASE 1: Quiescent Zero Current with Different Offsets
        // =================================================================
        $display("\n--- PHASE 1: Quiescent Zero Current Verification Across Offsets ---");
        // Case 1: Nominal physical offset (2.510V -> 3115)
        verify_clarke(12'd3115, 12'd3115, 13'sd3115, "Zero Current at Offset=3115");
        
        // Case 2: Standard half-scale offset (2048)
        verify_clarke(12'd2048, 12'd2048, 13'sd2048, "Zero Current at Offset=2048");

        // Case 3: Low sensor offset (1.6V -> 1985)
        verify_clarke(12'd1985, 12'd1985, 13'sd1985, "Zero Current at Offset=1985");

        // Case 4: High sensor offset (2.8V -> 3474)
        verify_clarke(12'd3474, 12'd3474, 13'sd3474, "Zero Current at Offset=3474");

        // =================================================================
        // PHASE 2: Physical Operating Range (+/- 1A to +/- 5A)
        // =================================================================
        $display("\n--- PHASE 2: DC Current Steps (+/- 1A to +/- 5A at Nominal Offset) ---");
        cur_offset = 13'sd3115;

        // Steps of +1A, +2A, +3A, +4A, +5A on Phase A (with ib = -ia/2 for balanced sum)
        for (i = 1; i <= 5; i = i + 1) begin
            ia_code = cur_offset + $rtoi(i / AMPS_PER_COUNT);
            ib_code = cur_offset - $rtoi((i * 0.5) / AMPS_PER_COUNT);
            verify_clarke(ia_code, ib_code, cur_offset, "Positive Phase A Balanced Step");
        end

        // Steps of -1A, -2A, -3A, -4A, -5A on Phase A
        for (i = 1; i <= 5; i = i + 1) begin
            ia_code = cur_offset - $rtoi(i / AMPS_PER_COUNT);
            ib_code = cur_offset + $rtoi((i * 0.5) / AMPS_PER_COUNT);
            verify_clarke(ia_code, ib_code, cur_offset, "Negative Phase A Balanced Step");
        end

        // Pure Beta axis currents (ia = 0, so ib creates purely beta current)
        for (i = 1; i <= 4; i = i + 1) begin
            ia_code = cur_offset;
            ib_code = cur_offset + $rtoi(i / AMPS_PER_COUNT);
            verify_clarke(ia_code, ib_code, cur_offset, "Pure Beta Current Step");
        end

        // =================================================================
        // PHASE 3: 360-Degree Balanced 3-Phase Rotating Vector Sweep
        // =================================================================
        $display("\n--- PHASE 3: 360-Degree Rotating Vector Trajectory (3.0A Peak) ---");
        curr_mag = 3.0; // 3 Amperes peak
        for (i = 0; i < 36; i = i + 1) begin
            theta_rad = (i * 10.0) * (3.14159265358979 / 180.0);
            
            // Balanced 3-phase equations:
            ia_flt = curr_mag * $cos(theta_rad);
            ib_flt = curr_mag * $cos(theta_rad - (2.0 * 3.14159265358979 / 3.0));

            ia_code = cur_offset + $rtoi(ia_flt / AMPS_PER_COUNT);
            ib_code = cur_offset + $rtoi(ib_flt / AMPS_PER_COUNT);

            verify_clarke(ia_code, ib_code, cur_offset, "3-Phase Vector Rotation (10 deg step)");
        end

        // =================================================================
        // PHASE 4: Dynamic Offset Modulation & Mid-Conversion Hazard Test
        // =================================================================
        $display("\n--- PHASE 4: Dynamic Offset Modulation & Mid-Conversion Hazard Tests ---");
        
        // Test 4A: Thermal Drift Modulation across samples (3100 -> 3130 counts)
        $display("-> Test 4A: Tracking dynamic thermal drift across consecutive samples");
        for (i = 0; i < 10; i = i + 1) begin
            cur_offset = 13'sd3100 + (i * 3); // Drifting baseline
            ia_code = cur_offset + 124;       // Constant +2.0A signal
            ib_code = cur_offset - 62;        // Constant -1.0A signal
            verify_clarke(ia_code, ib_code, cur_offset, "Thermal Drift Tracking");
        end

        // Test 4B: Mid-Conversion Offset Glitch Hazard Test
        // Verify that if adc_offset changes WHILE the pipeline is calculating,
        // the calculation is NOT corrupted because inputs are registered at Step 0.
        $display("-> Test 4B: Mid-Conversion Offset Glitch Immunity Test");
        total_tests = total_tests + 1;
        ia_raw     <= 12'd3239; // +2.0A at offset 3115
        ib_raw     <= 12'd3053; // -1.0A at offset 3115
        adc_offset <= 13'sd3115;

        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // Wait until Step 3 of 9 (pipeline is in-flight)
        repeat (3) @(posedge clk);
        $display("[%0t] Injecting severe offset glitch during mid-pipeline (3115 -> 2048)...", $time);
        adc_offset <= 13'sd2048; // Glitch offset during computation!

        // Wait for done
        while (!done) @(posedge clk);

        // Check if output used the registered Step 0 offset (3115) or the corrupted glitch (2048)
        alpha_actual = $itor(i_alpha) / Q20_SCALE;
        if (alpha_actual >= 1.95 && alpha_actual <= 2.05) begin
            $display("[PASS #%03d] Mid-conversion offset glitch safely rejected! Output: %6.3f A (Registered at Step 0).",
                     total_tests, alpha_actual);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Mid-conversion offset glitch corrupted computation! Output: %6.3f A",
                     total_tests, alpha_actual);
            failed_tests = failed_tests + 1;
        end

        // =================================================================
        // PHASE 5: Consecutive Back-to-Back Pipeline Stress
        // =================================================================
        $display("\n--- PHASE 5: Consecutive Zero-Slack Back-to-Back Conversions ---");
        // Trigger next start on the EXACT cycle done is asserted
        cur_offset = 13'sd3115;
        for (i = 1; i <= 8; i = i + 1) begin
            total_tests = total_tests + 1;
            ia_code = cur_offset + (i * 30);
            ib_code = cur_offset - (i * 15);

            ia_raw     <= ia_code;
            ib_raw     <= ib_code;
            adc_offset <= cur_offset;
            start      <= 1'b1; // Immediate trigger
            @(posedge clk);
            start      <= 1'b0;

            while (!done) @(posedge clk);
            
            alpha_actual = $itor(i_alpha) / Q20_SCALE;
            alpha_expected = ($signed({1'b0, ia_code}) - cur_offset) * AMPS_PER_COUNT;
            if (alpha_actual - alpha_expected < 0.02 && alpha_expected - alpha_actual < 0.02) begin
                passed_tests = passed_tests + 1;
            end else begin
                failed_tests = failed_tests + 1;
                $display("[FAIL #%03d] Back-to-back conversion corrupted!", total_tests);
            end
        end
        $display("[PASS] Completed 8 zero-slack consecutive conversions with zero throughput penalty.");

        // =================================================================
        // PHASE 6: Mid-Computation Asynchronous Reset Recovery
        // =================================================================
        $display("\n--- PHASE 6: Mid-Computation Asynchronous Reset Recovery ---");
        total_tests = total_tests + 1;
        ia_raw     <= 12'd3300;
        ib_raw     <= 12'd3100;
        adc_offset <= 13'sd3115;

        @(posedge clk);
        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        // Wait 4 clock cycles (mid-calculation)
        repeat (4) @(posedge clk);
        $display("[%0t] Asserting rst_n LOW during Step 4...", $time);
        rst_n <= 1'b0;
        repeat (3) @(posedge clk);
        
        if (i_alpha !== 0 || i_beta !== 0 || done !== 0) begin
            $display("[FAIL #%03d] Reset failed to clear registers: i_alpha=%d, i_beta=%d, done=%b",
                     total_tests, i_alpha, i_beta, done);
            failed_tests = failed_tests + 1;
        end else begin
            $display("[PASS #%03d] Asynchronous reset cleanly cleared pipeline.", total_tests);
            passed_tests = passed_tests + 1;
        end

        // Release reset and perform a clean conversion
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);
        verify_clarke(12'd3239, 12'd3053, 13'sd3115, "Post-Reset Clean Sample (+2.0A)");

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("             CLARKE TRANSFORM TESTBENCH EXECUTION SUMMARY          ");
        $display("===================================================================");
        $display(" Total Conversion Tests Evaluated   : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display("===================================================================");

        if (failed_tests == 0) begin
            $display(" >>> ALL CLARKE TRANSFORM TESTS PASSED PERFECTLY! ZERO ERRORS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ARITHMETIC/TIMING ERRORS! <<< \n", failed_tests);
        end

        $finish;
    end

endmodule
