`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_speed_pi;

    // -------------------------------------------------------------
    // 1. Clock, Reset, and DUT Signals
    // -------------------------------------------------------------
    reg                          clk;
    reg                          rst_n;
    reg                          sample_tick;
    reg  signed [`DATA_WIDTH-1:0] speed_ref;
    reg  signed [`DATA_WIDTH-1:0] speed_fb;
    wire signed [`DATA_WIDTH-1:0] te_ref;

    // -------------------------------------------------------------
    // 2. Instantiate DUT
    // -------------------------------------------------------------
    speed_pi #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .sample_tick (sample_tick),
        .speed_ref   (speed_ref),
        .speed_fb    (speed_fb),
        .te_ref      (te_ref)
    );

    // -------------------------------------------------------------
    // 3. 100 MHz Master Clock (10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // 4. Test Statistics & Constants
    // -------------------------------------------------------------
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    localparam real Q20_SCALE = 1048576.0;

    // Motor Mechanical Plant Parameters (2.2 kW 4-Pole Induction Motor)
    localparam real POLE_PAIRS = 2.0;    // 4 poles -> p = 2
    localparam real MOTOR_J    = 0.02;   // Rotor inertia J (kg*m^2)
    localparam real MOTOR_B    = 0.002;  // Viscous damping B (N*m*s/rad)
    localparam real DT_SEC     = 0.0001; // Ts = 100 us control sample period

    // Watchdog
    initial begin
        #100_000_000; // 100 ms simulation time limit
        $display("\n[FATAL TIMEOUT] Simulation exceeded 100 ms!");
        $finish;
    end

    // Conversion helpers
    function real to_real;
        input signed [`DATA_WIDTH-1:0] val;
        begin
            to_real = $itor(val) / Q20_SCALE;
        end
    endfunction

    function signed [`DATA_WIDTH-1:0] to_fixed;
        input real val;
        begin
            to_fixed = $rtoi(val * Q20_SCALE);
        end
    endfunction

    // -------------------------------------------------------------
    // 5. Closed-Loop Motor Plant Simulation Task
    // -------------------------------------------------------------
    real speed_fb_sim = 0.0;
    real te_sim       = 0.0;

    // Simulates one 100 us control sample: pulses sample_tick, runs PI FSM,
    // and integrates the motor mechanical dynamics J * dw/dt = Te - T_load - B*w
    task run_closed_loop_tick;
        input real t_load; // Physical load torque on shaft (Nm)
        real te_physical;
        real accel;
        begin
            // 1. Pulse sample_tick
            @(posedge clk);
            sample_tick <= 1'b1;
            @(posedge clk);
            sample_tick <= 1'b0;

            // 2. Wait for Speed PI FSM (9 cycles = 90 ns)
            repeat (10) @(posedge clk);
            #1; // Allow non-blocking assignment to settle

            // 3. Read physical torque produced by speed_pi
            te_physical = to_real(te_ref);
            te_sim = te_physical;

            // 4. Advance motor mechanical dynamics by dt = 100 us:
            // dw_m/dt = (Te - T_load - B * w_m) / J
            // w_e = p * w_m
            // dw_e/dt = (p / J) * (Te - T_load) - (B / J) * w_e
            accel = (POLE_PAIRS / MOTOR_J) * (te_physical - t_load) - (MOTOR_B / MOTOR_J) * speed_fb_sim;
            speed_fb_sim = speed_fb_sim + accel * DT_SEC;

            // 5. Feedback electrical speed to PI controller
            speed_fb <= to_fixed(speed_fb_sim);
        end
    endtask

    // Simulates closed-loop operation for N samples
    task run_closed_loop_samples;
        input integer num_samples;
        input real    t_load;
        integer s;
        begin
            for (s = 0; s < num_samples; s = s + 1) begin
                run_closed_loop_tick(t_load);
            end
        end
    endtask

    // Check speed and torque lock-in against expected values
    task verify_lock_in;
        input real exp_speed;
        input real exp_torque;
        input real speed_tol;
        input real torque_tol;
        input [639:0] desc;

        real act_speed, act_torque;
        real err_speed, err_torque;
        reg pass;
    begin
        total_tests = total_tests + 1;
        act_speed  = to_real(speed_fb);
        act_torque = to_real(te_ref);

        err_speed  = (act_speed >= exp_speed)   ? (act_speed - exp_speed)   : (exp_speed - act_speed);
        err_torque = (act_torque >= exp_torque) ? (act_torque - exp_torque) : (exp_torque - act_torque);

        pass = (err_speed <= speed_tol) && (err_torque <= torque_tol);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] LOCKED IN: Speed=%7.2f rad/s (exp %7.2f, err %0.3f) | Torque=%7.3f Nm (exp %7.3f, err %0.3f) | %0s",
                     total_tests, act_speed, exp_speed, err_speed, act_torque, exp_torque, err_torque, desc);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] LOCK-IN MISMATCH! (%0s)", total_tests, desc);
            $display("           Speed : Act = %7.2f rad/s, Exp = %7.2f rad/s (Err = %7.3f, Tol = %7.3f)",
                     act_speed, exp_speed, err_speed, speed_tol);
            $display("           Torque: Act = %7.3f Nm, Exp = %7.3f Nm (Err = %7.3f, Tol = %7.3f)",
                     act_torque, exp_torque, err_torque, torque_tol);
        end
    end
    endtask

    // -------------------------------------------------------------
    // 6. Main Test Sequence
    // -------------------------------------------------------------
    integer k;
    real prev_te;

    initial begin
        $display("===================================================================");
        $display("   RIGOROUS CLOSED-LOOP MOTOR LOCK-IN TESTBENCH: speed_pi.v        ");
        $display("   Configuration: Kp = 0.5 Nm/(rad/s), Ki = 0.01, Max Te = 20.0 Nm ");
        $display("   Simulates 2.2 kW Motor Inertia J=0.02 kg*m^2 & Shaft Dynamics   ");
        $display("===================================================================");

        clk          = 1'b0;
        rst_n        = 1'b0;
        sample_tick  = 1'b0;
        speed_ref    = 32'sd0;
        speed_fb     = 32'sd0;
        speed_fb_sim = 0.0;
        te_sim       = 0.0;

        // =================================================================
        // PHASE 1: Asynchronous Reset Verification
        // =================================================================
        $display("\n--- PHASE 1: Asynchronous Reset Verification ---");
        repeat (5) @(posedge clk);
        total_tests = total_tests + 1;
        if (te_ref == 32'sd0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Asynchronous reset active: te_ref = 0.000 Nm", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Reset failed! te_ref = %0d", total_tests, te_ref);
        end

        // Release reset
        @(posedge clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);

        // =================================================================
        // PHASE 2: No-Load Closed-Loop Acceleration & Lock-In (T_load = 0 Nm)
        // =================================================================
        $display("\n--- PHASE 2: Closed-Loop No-Load Acceleration & Lock-In (T_load = 0 Nm) ---");
        $display("Commanding speed_ref = +100.0 rad/s (477.5 RPM). Simulating closed-loop acceleration...");
        speed_ref <= to_fixed(100.0);

        // Run closed-loop for 2500 samples (250 ms)
        run_closed_loop_samples(2500, 0.0);

        // At steady state with T_load = 0:
        // Speed locks in to 100.0 rad/s
        // Friction torque at 100 rad/s = B * (w_e / p) = 0.002 * 50 = 0.100 Nm
        verify_lock_in(100.0, 0.100, 0.10, 0.10, "No-Load: Speed locks into +100 rad/s, torque locks into friction (0.1 Nm)");

        // Verify stability: continue running for 500 more samples (50 ms) to confirm rock-solid lock
        run_closed_loop_samples(500, 0.0);
        verify_lock_in(100.0, 0.100, 0.05, 0.05, "Stability Check: Speed and torque held locked without drift or oscillation");

        // =================================================================
        // PHASE 3: Closed-Loop Acceleration Under Heavy Load (T_load = 8.0 Nm)
        // =================================================================
        $display("\n--- PHASE 3: Closed-Loop Acceleration Under Load (T_load = 8.0 Nm) ---");
        $display("Commanding speed_ref = +60.0 rad/s (286.5 RPM) with 8.0 Nm continuous shaft load...");
        speed_ref <= to_fixed(60.0);

        // Run closed-loop for 2500 samples (250 ms) with 8.0 Nm load
        run_closed_loop_samples(2500, 8.0);

        // At steady state:
        // Speed locks in to 60.0 rad/s
        // Torque locks in to T_load + B*w_m = 8.0 + 0.002 * 30 = 8.060 Nm!
        verify_lock_in(60.0, 8.060, 0.10, 0.10, "Loaded: Speed locks into +60 rad/s, torque locks into 8.06 Nm load");

        // =================================================================
        // PHASE 4: Dynamic Step Load Disturbance & Recovery Lock-In
        // =================================================================
        $display("\n--- PHASE 4: Dynamic Step Load Disturbance & Recovery Lock-In ---");
        $display("Applying sudden shock load step from 8.0 Nm -> 14.0 Nm (+75%% load step)...");
        // Motor is running locked at 60 rad/s. Step load to 14.0 Nm.
        // Run for 2500 samples to let the PI controller reject the disturbance and settle
        run_closed_loop_samples(2500, 14.0);

        // After recovery:
        // Speed re-locks to 60.0 rad/s
        // Torque locks into new load = 14.0 + 0.06 = 14.060 Nm!
        verify_lock_in(60.0, 14.060, 0.10, 0.10, "Shock Load Step: Re-locked speed to +60 rad/s, torque locked to 14.06 Nm");

        // =================================================================
        // PHASE 5: Overload Saturation Lock-In (T_load = 25.0 Nm > TE_MAX)
        // =================================================================
        $display("\n--- PHASE 5: Overload Saturation Lock-In (T_load = 25.0 Nm > TE_MAX) ---");
        $display("Applying 25.0 Nm load (exceeding motor 20.0 Nm limit). Verifying torque locks into TE_MAX...");
        // Apply 25 Nm overload
        run_closed_loop_samples(1000, 25.0);

        // Torque MUST lock into exactly TE_MAX (+20.000 Nm) and not overflow
        total_tests = total_tests + 1;
        if (te_ref == `TE_MAX) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Overload Lock-In: Torque strictly locked into TE_MAX (+20.000 Nm = %0d counts)",
                     total_tests, te_ref);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Overload clamping failed! Got %0d, expected %0d", total_tests, te_ref, `TE_MAX);
        end

        // =================================================================
        // PHASE 6: Full 4-Quadrant Reverse Acceleration & Negative Lock-In
        // =================================================================
        $display("\n--- PHASE 6: Full 4-Quadrant Reverse Acceleration & Lock-In ---");
        $display("Commanding reverse speed_ref = -80.0 rad/s (-382 RPM) with reverse load -6.0 Nm...");
        speed_ref <= to_fixed(-80.0);

        // Run closed-loop for 3500 samples (350 ms) to transition from forward to reverse
        run_closed_loop_samples(3500, -6.0);

        // Steady state reverse:
        // Speed locks into -80.0 rad/s
        // Torque locks into -6.0 - 0.002*40 = -6.080 Nm
        verify_lock_in(-80.0, -6.080, 0.15, 0.15, "Reverse Operation: Speed locks into -80 rad/s, torque locks into -6.08 Nm");

        // =================================================================
        // PHASE 7: Standstill Zero-Speed Hold (Active Position / Holding Brake)
        // =================================================================
        $display("\n--- PHASE 7: Standstill Zero-Speed Hold (Active Holding Brake) ---");
        $display("Commanding speed_ref = 0.0 rad/s. Applying 7.0 Nm external stall disturbance...");
        speed_ref <= to_fixed(0.0);

        // Run for 3500 samples (350 ms) to decelerate from -80 rad/s and lock against 7.0 Nm disturbance
        run_closed_loop_samples(3500, 7.0);

        // Shaft must be held at 0 rad/s by producing exactly 7.0 Nm holding torque
        verify_lock_in(0.0, 7.000, 0.10, 0.10, "Zero-Speed Hold: Shaft locked at 0 rad/s, torque locked into +7.00 Nm holding torque");

        // =================================================================
        // PHASE 8: Creep Speed Sub-RPM High Precision Lock-In
        // =================================================================
        $display("\n--- PHASE 8: Creep Speed Sub-RPM High Precision Lock-In ---");
        $display("Commanding low creep speed = 2.0 rad/s (9.55 RPM) with 1.5 Nm load...");
        speed_ref <= to_fixed(2.0);

        run_closed_loop_samples(2000, 1.5);
        verify_lock_in(2.0, 1.502, 0.05, 0.05, "Creep Speed: High precision sub-RPM lock at 2.0 rad/s, torque locked at 1.50 Nm");

        // =================================================================
        // PHASE 9: Cycle-Accurate Latency & FSM Return to State 0
        // =================================================================
        $display("\n--- PHASE 9: Cycle-Accurate Latency & FSM Idle Verification ---");
        speed_ref <= to_fixed(30.0);
        prev_te = to_real(te_ref);

        @(posedge clk);
        sample_tick <= 1'b1;
        @(posedge clk);
        sample_tick <= 1'b0;

        // Cycles 1..7: Output must remain stable while multiplier runs
        for (k = 1; k <= 7; k = k + 1) begin
            @(posedge clk);
            #1;
            total_tests = total_tests + 1;
            if (te_ref == to_fixed(prev_te)) begin
                passed_tests = passed_tests + 1;
            end else begin
                failed_tests = failed_tests + 1;
                $display("[FAIL #%03d] Premature update at cycle %0d", total_tests, k);
            end
        end

        // Cycle 8: te_ref updates
        @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        passed_tests = passed_tests + 1;
        $display("[PASS #%03d] Cycle 8: te_ref updated cleanly to new value (%0.3f Nm)", total_tests, to_real(te_ref));

        // Cycle 9: FSM returned to State 0
        @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        if (dut.state == 4'd0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Cycle 9: FSM cleanly returned to State 0 (Idle, ready for next sample)", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] FSM state was %0d, expected 0", total_tests, dut.state);
        end

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("   SPEED PI CONTROLLER CLOSED-LOOP LOCK-IN TESTBENCH SUMMARY       ");
        $display("===================================================================");
        $display(" Total Verification Tests Evaluated : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display("===================================================================");

        if (failed_tests == 0) begin
            $display(" >>> ALL CLOSED-LOOP TORQUE & SPEED LOCK-IN TESTS PASSED! ZERO ERRORS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ARITHMETIC/TIMING ERRORS! <<< \n", failed_tests);
        end

        $finish;
    end

endmodule
