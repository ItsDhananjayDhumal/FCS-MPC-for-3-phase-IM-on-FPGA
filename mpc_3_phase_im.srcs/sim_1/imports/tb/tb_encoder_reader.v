`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_encoder_reader;

    // -------------------------------------------------------------
    // 1. Clock, Reset and DUT Signals
    // -------------------------------------------------------------
    reg clk;
    reg rst_n;
    reg enc_a;
    reg enc_b;
    reg enc_z;
    reg sample_tick;

    wire signed [31:0] position;
    wire signed [`DATA_WIDTH-1:0] speed_elec;
    wire speed_valid;

    // Instantiate DUT
    encoder_reader #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .enc_a(enc_a),
        .enc_b(enc_b),
        .enc_z(enc_z),
        .sample_tick(sample_tick),
        .position(position),
        .speed_elec(speed_elec),
        .speed_valid(speed_valid)
    );

    // -------------------------------------------------------------
    // 2. 100 MHz Master Clock (10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // 3. Periodic 10 kHz Sample Tick Generator (every 100 us)
    // -------------------------------------------------------------
    reg enable_sample_tick;
    integer tick_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sample_tick <= 1'b0;
            tick_timer  <= 0;
        end else if (enable_sample_tick) begin
            if (tick_timer >= `TS_COUNTER_MAX - 1) begin
                sample_tick <= 1'b1;
                tick_timer  <= 0;
            end else begin
                sample_tick <= 1'b0;
                tick_timer  <= tick_timer + 1;
            end
        end else begin
            sample_tick <= 1'b0;
            tick_timer  <= 0;
        end
    end

    // -------------------------------------------------------------
    // 4. Test Statistics & Physical Constants
    // -------------------------------------------------------------
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    localparam real Q20_SCALE = 1048576.0;
    // 4-pole motor: electrical rad/s = mech RPM * (4 * pi / 60)
    localparam real ELEC_RAD_PER_RPM = (4.0 * 3.141592653589793) / 60.0; // 0.20943951

    // Global simulation watchdog (500 ms)
    initial begin
        #500_000_000;
        $display("\n[FATAL TIMEOUT] Simulation exceeded 500 ms!");
        $finish;
    end

    // -------------------------------------------------------------
    // 5. Motor Emulator Tasks
    // -------------------------------------------------------------
    reg [1:0] quad_phase; // 0: 00, 1: 01, 2: 11, 3: 10

    task update_quad_pins;
    begin
        case (quad_phase)
            2'd0: begin enc_a <= 1'b0; enc_b <= 1'b0; end
            2'd1: begin enc_a <= 1'b0; enc_b <= 1'b1; end
            2'd2: begin enc_a <= 1'b1; enc_b <= 1'b1; end
            2'd3: begin enc_a <= 1'b1; enc_b <= 1'b0; end
        endcase
    end
    endtask

    // Step by single count forward (+1) or reverse (-1)
    task step_quad;
        input integer dir; // +1 = forward, -1 = reverse
        begin
            if (dir > 0)
                quad_phase = quad_phase + 2'd1;
            else
                quad_phase = quad_phase - 2'd1;
            update_quad_pins();
        end
    endtask

    // -------------------------------------------------------------
    // 6. Automated Speed Test Task (Continuous Pulse Generation + Sampling)
    // -------------------------------------------------------------
    task test_speed;
        input real target_rpm;
        input integer num_samples;
        input real tolerance_rpm;
        input [199:0] test_name;

        real step_period_ns;
        real next_step_time_ns;
        real elapsed_ns;
        integer sample_idx;
        integer dir;
        real act_rad_s;
        real act_rpm;
        real exp_rad_s;
        real err_rpm;
        reg  pass;
    begin
        total_tests = total_tests + 1;
        dir = (target_rpm >= 0.0) ? 1 : -1;
        act_rpm = 0.0;
        act_rad_s = 0.0;

        if (target_rpm == 0.0) begin
            repeat (num_samples * `TS_COUNTER_MAX) @(posedge clk);
            act_rad_s = $itor(speed_elec) / Q20_SCALE;
            act_rpm   = act_rad_s / ELEC_RAD_PER_RPM;
        end else begin
            step_period_ns = (60.0 * 1.0e9) / (10000.0 * (target_rpm >= 0.0 ? target_rpm : -target_rpm));
            elapsed_ns = 0.0;
            next_step_time_ns = step_period_ns;

            for (sample_idx = 0; sample_idx < num_samples; sample_idx = sample_idx + 1) begin
                repeat (`TS_COUNTER_MAX) begin
                    @(posedge clk);
                    elapsed_ns = elapsed_ns + 10.0;
                    if (elapsed_ns >= next_step_time_ns) begin
                        step_quad(dir);
                        next_step_time_ns = next_step_time_ns + step_period_ns;
                    end
                    // Capture speed on the speed_valid pulse during the final samples
                    if (sample_idx >= num_samples - 2 && speed_valid) begin
                        act_rad_s = $itor(speed_elec) / Q20_SCALE;
                        act_rpm   = act_rad_s / ELEC_RAD_PER_RPM;
                    end
                end
            end
        end

        exp_rad_s = target_rpm * ELEC_RAD_PER_RPM;
        err_rpm = act_rpm - target_rpm;
        if (err_rpm < 0.0) err_rpm = -err_rpm;

        pass = (err_rpm <= tolerance_rpm);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Target: %7.2f RPM | Measured: %7.2f RPM (%7.2f rad/s) | Err: %5.2f RPM | %0s",
                     total_tests, target_rpm, act_rpm, act_rad_s, err_rpm, test_name);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] SPEED OUT OF TOLERANCE! (%0s)", total_tests, test_name);
            $display("           Target   : %7.2f RPM (%7.2f rad/s)", target_rpm, exp_rad_s);
            $display("           Measured : %7.2f RPM (%7.2f rad/s)", act_rpm, act_rad_s);
            $display("           Error    : %7.2f RPM (Allowed Tol: %5.2f RPM)", err_rpm, tolerance_rpm);
        end
    end
    endtask

    // -------------------------------------------------------------
    // 7. Main Test Sequence
    // -------------------------------------------------------------
    integer initial_pos;

    initial begin
        $display("===================================================================");
        $display("       RIGOROUS VERIFICATION TESTBENCH: encoder_reader.v           ");
        $display("===================================================================");

        // Power-on Reset
        clk                = 1'b0;
        rst_n              = 1'b0;
        enc_a              = 1'b0;
        enc_b              = 1'b0;
        enc_z              = 1'b0;
        quad_phase         = 2'd0;
        enable_sample_tick = 1'b0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);
        enable_sample_tick = 1'b1;

        // =================================================================
        // PHASE 1: Reset & Standstill State Verification
        // =================================================================
        $display("\n--- PHASE 1: Reset & Standstill State ---");
        total_tests = total_tests + 1;
        if (position == 32'sd0 && speed_elec == 32'sd0) begin
            $display("[PASS #%03d] Initial reset state clean: position = 0, speed = 0.0 rad/s", total_tests);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Non-zero initial state! pos=%0d, speed=%0d", total_tests, position, speed_elec);
            failed_tests = failed_tests + 1;
        end

        // =================================================================
        // PHASE 2: Single-Step Quadrature Logic & Direction Decoding
        // =================================================================
        $display("\n--- PHASE 2: Single-Step Quadrature Edge & Direction Logic ---");
        
        // Forward 4 edges (1 full optical cycle = +4 counts)
        initial_pos = position;
        repeat (4) begin
            step_quad(1);
            repeat (5) @(posedge clk); // Allow 3-stage synchronizer to propagate
        end
        total_tests = total_tests + 1;
        if (position == initial_pos + 4) begin
            $display("[PASS #%03d] Forward 4 edges correctly advanced position by +4 (now %0d)", total_tests, position);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Forward count error! Expected %0d, got %0d", total_tests, initial_pos + 4, position);
            failed_tests = failed_tests + 1;
        end

        // Reverse 4 edges (-4 counts)
        initial_pos = position;
        repeat (4) begin
            step_quad(-1);
            repeat (5) @(posedge clk);
        end
        total_tests = total_tests + 1;
        if (position == initial_pos - 4) begin
            $display("[PASS #%03d] Reverse 4 edges correctly decremented position by -4 (now %0d)", total_tests, position);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Reverse count error! Expected %0d, got %0d", total_tests, initial_pos - 4, position);
            failed_tests = failed_tests + 1;
        end

        // Illegal double-transition test (00 -> 11 simultaneous bit flip)
        $display("-> Testing illegal simultaneous edge transition rejection (00 -> 11)");
        enc_a <= 1'b0; enc_b <= 1'b0;
        repeat (5) @(posedge clk);
        initial_pos = position;
        enc_a <= 1'b1; enc_b <= 1'b1; // Illegal simultaneous transition
        repeat (5) @(posedge clk);
        total_tests = total_tests + 1;
        if (position == initial_pos) begin
            $display("[PASS #%03d] Illegal transition rejected cleanly! Position remained %0d", total_tests, position);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Illegal transition corrupted position! got %0d, expected %0d", total_tests, position, initial_pos);
            failed_tests = failed_tests + 1;
        end
        quad_phase = 2'd2; // Synchronize quad_phase to A=1, B=1

        // Z-Index pulse immunity test (Verify position does NOT jump to 0)
        $display("-> Testing Z-index position reset immunity (ENCODER_Z_RESET = 0)");
        total_tests = total_tests + 1;
        initial_pos = position;
        enc_z <= 1'b1;
        repeat (5) @(posedge clk);
        enc_z <= 1'b0;
        repeat (5) @(posedge clk);
        if (position == initial_pos) begin
            $display("[PASS #%03d] Z-index pulse did not reset position! (Position = %0d preserved)", total_tests, position);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Z-index pulse caused position jump! pos=%0d, expected=%0d", total_tests, position, initial_pos);
            failed_tests = failed_tests + 1;
        end

        // =================================================================
        // PHASE 3: Exact Integer Speed Multiples of 60 RPM
        // =================================================================
        $display("\n--- PHASE 3: Integer Multiples of 60 RPM (Constant Velocity) ---");

        // Test 3A: +60 RPM (Exactly 1 count per 100 us sample)
        test_speed(60.0, 60, 1.5, "Forward +60.0 RPM (1 count/sample)");

        // Test 3B: +120 RPM (Exactly 2 counts per 100 us sample)
        test_speed(120.0, 60, 1.5, "Forward +120.0 RPM (2 counts/sample)");

        // Test 3C: +300 RPM (Exactly 5 counts per 100 us sample)
        test_speed(300.0, 60, 2.0, "Forward +300.0 RPM (5 counts/sample)");

        // Test 3D: +1500 RPM (Rated Synchronous Speed, 25 counts/sample)
        test_speed(1500.0, 65, 3.0, "Rated Synchronous Speed +1500.0 RPM");

        // =================================================================
        // PHASE 4: Sub-Quantum Fractional Speeds (Non-Multiples of 60 RPM)
        // =================================================================
        $display("\n--- PHASE 4: Sub-Quantum Fractional Speeds & IIR Averaging ---");

        // Test 4A: +30.0 RPM (0.5 counts per sample -> alternating [0, 1, 0, 1])
        // Note: 1st-order IIR with alpha=0.1 has natural theoretical dither ripple of +/- 1.58 RPM around 30 RPM
        test_speed(30.0, 75, 3.5, "Fractional Speed +30.0 RPM (0.5 cnt/sample dither)");

        // Test 4B: +45.0 RPM (0.75 counts per sample -> sequence [1, 1, 1, 0])
        test_speed(45.0, 65, 1.5, "Fractional Speed +45.0 RPM (0.75 cnt/sample dither)");

        // Test 4C: +1420.0 RPM (Typical Induction Motor Full-Load Operating Point)
        test_speed(1420.0, 65, 3.0, "Induction Motor Rated Load +1420.0 RPM");

        // Test 4D: Very Low Crawl Speed +15.0 RPM (0.25 counts/sample)
        test_speed(15.0, 70, 2.0, "Low Crawl Speed +15.0 RPM (0.25 cnt/sample)");

        // =================================================================
        // PHASE 5: Negative Speeds & Full Direction Reversal
        // =================================================================
        $display("\n--- PHASE 5: Reverse Speeds & Sudden Direction Inversion ---");

        // Test 5A: -60.0 RPM (-1 count/sample)
        test_speed(-60.0, 60, 1.5, "Reverse -60.0 RPM (-1 count/sample)");

        // Test 5B: -1500.0 RPM (-25 counts/sample)
        test_speed(-1500.0, 65, 3.0, "Reverse Full Speed -1500.0 RPM");

        // Test 5C: Reversal +1200 RPM -> -1200 RPM (2700 RPM step change)
        $display("-> Testing high-speed dynamic reversal (+1200 RPM -> -1200 RPM)");
        test_speed(1200.0, 75, 5.0, "Pre-reversal forward +1200 RPM");
        test_speed(-1200.0, 75, 5.0, "Post-reversal reverse -1200 RPM");

        // =================================================================
        // PHASE 6: Synchronizer Noise Glitch & Chatter Immunity
        // =================================================================
        $display("\n--- PHASE 6: Synchronizer High-Frequency Noise Glitch Immunity ---");
        $display("-> Injecting single-cycle 10ns noise glitches on enc_a and enc_b...");
        repeat (10) @(posedge clk);
        initial_pos = position;
        // Inject narrow 3ns asynchronous noise spike between clock edges
        #2 enc_a = ~enc_a;
        #3 enc_a = ~enc_a;
        repeat (10) @(posedge clk);

        total_tests = total_tests + 1;
        if (position == initial_pos) begin
            $display("[PASS #%03d] Sub-cycle asynchronous noise glitch safely rejected by synchronizer!", total_tests);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Noise glitch caused false position increment! pos=%0d, expected=%0d", total_tests, position, initial_pos);
            failed_tests = failed_tests + 1;
        end

        // =================================================================
        // PHASE 7: Mid-Operation Asynchronous Reset Recovery
        // =================================================================
        $display("\n--- PHASE 7: Mid-Operation Asynchronous Reset Recovery ---");
        // Spin motor at 600 RPM
        repeat (50) begin
            step_quad(1);
            repeat (100) @(posedge clk);
        end
        $display("[%0t] Asserting rst_n LOW while motor is spinning...", $time);
        rst_n <= 1'b0;
        repeat (5) @(posedge clk);
        
        total_tests = total_tests + 1;
        if (position == 0 && speed_elec == 0 && speed_valid == 0) begin
            $display("[PASS #%03d] Asynchronous reset instantly cleared all speed & position registers.", total_tests);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Reset failed to clear registers: pos=%0d, speed=%0d", total_tests, position, speed_elec);
            failed_tests = failed_tests + 1;
        end

        // Release reset and verify post-reset speed recovery
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);
        test_speed(300.0, 60, 2.0, "Post-Reset Speed Recovery (+300 RPM)");

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("             ENCODER READER TESTBENCH EXECUTION SUMMARY            ");
        $display("===================================================================");
        $display(" Total Verification Tests Evaluated : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display("===================================================================");

        if (failed_tests == 0) begin
            $display(" >>> ALL ENCODER READER TESTS PASSED PERFECTLY! ZERO ERRORS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ARITHMETIC/TIMING ERRORS! <<< \n", failed_tests);
        end

        $finish;
    end

endmodule
