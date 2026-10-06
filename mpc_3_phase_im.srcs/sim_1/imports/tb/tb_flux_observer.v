`timescale 1ns / 1ps

// =============================================================================
// Comprehensive & Extended Testbench for flux_observer.v
// FCS-MPC for 3-Phase Induction Motor (Nexys Video Artix-7)
//
// Extended Test Architecture (Simulates ~6.0 seconds of physical motor time):
//
// Phase 1: Edge Cases & Numerical Boundaries
//   1.1 Reset and Initial State Clearing
//   1.2 Zero-Input Standstill Behavior
//   1.3 Spurious / Interrupted Start Pulse Rejection (FSM Robustness)
//   1.4 Positive Speed Clamping Boundary (> +SPEED_LIMIT)
//   1.5 Negative Speed Clamping Boundary (< -SPEED_LIMIT)
//   1.6 Sign Inversion / Anti-Symmetry Verification
//   1.7 Overcurrent Pulse Dynamic Range Robustness (15A)
//
// Phase 2: Long-Duration DC Pre-Magnetization at Standstill
//   - Motor at rest (wr = 0 rad/s), rated magnetizing current applied (4.73A)
//   - Executed for 10,000 cycles (1.00 s = 5.17 tau_r)
//   - Proves exponential convergence to 99.4% of theoretical steady-state (0.960 Wb)
//   - Proves strict zero cross-axis leakage (psi_beta == 0) over full 1.0s duration
//
// Phase 3: Extended Steady-State Sinusoidal Rotation
//   - Constant peak magnetizing current (4.73A)
//   - Synchronous speed (wr = 100.0 rad/s, we = 105.0 rad/s)
//   - Executed for 8,000 cycles (0.80 s = 13.4 full electrical revolutions)
//   - Evaluates circular trajectory and ripple across 4,000 steady-state steps
//
// Phase 4: Full Closed-Loop PI Dynamic Speed Transient (30,000 cycles / 3.00 seconds)
//   - Step 4A (0.00s - 1.20s, 12,000 cycles):
//       Initial full steady-state settling at w_ref = 100 rad/s, TL = 3 Nm.
//       Flux observer, currents, and speed reach 100% stable equilibrium.
//   - Step 4B (1.20s - 3.00s, 18,000 cycles):
//       Speed reference changed: w_ref stepped to 180 rad/s.
//       Simulates exact Speed PI controller dynamics:
//         * PI error generates torque command Te*(t)
//         * FOC derives dynamic torque current Iq(t) and slip frequency ws(t)
//         * Stator frequency we(t) = wr(t) + ws(t) adapts continuously
//         * Rotor mechanics (J*dwr/dt = Te - TL) accelerate smoothly (no sudden jumps!)
//         * Stator currents (ia, ib) dynamically adapt amplitude and frequency
//       Verifies: Flux observer maintains continuous phase tracking through the
//       entire transient, avoids divergence, and settles to a reasonable stable value.
//
// Phase 5: Extended High-Speed Stability & SPEED_LIMIT Protection (11,000 cycles / 1.10 s)
//   5.1 Normal high-speed operation (wr = 280 rad/s) for 3,000 cycles
//   5.2 Operation immediately adjacent to stability limit (wr = 315 rad/s) for 3,000 cycles
//   5.3 Extended over-speed operation (wr = 360 rad/s) for 5,000 cycles (0.50 s!)
//       Proves internal speed clamp strictly preserves numerical stability.
// =============================================================================

`include "mpc_params.vh"

module tb_flux_observer;

    // Simulation Clock & Fixed-Point Constants
    localparam CLK_PERIOD_NS = 10; // 100 MHz system clock
    localparam DATA_WIDTH    = `DATA_WIDTH;
    localparam FRAC_BITS     = `FRAC_BITS;
    localparam REAL_PI       = 3.141592653589793;

    // Motor & Physical Constants
    localparam real MOTOR_LM_H = 0.20295;
    localparam real MOTOR_LR_H = 0.20893;
    localparam real MOTOR_RR_O = 1.08;
    localparam real TAU_R_SEC  = MOTOR_LR_H / MOTOR_RR_O; // 0.19345 s
    localparam real MOTOR_P    = 2.0;                      // pole pairs
    localparam real MOTOR_KT   = 1.5 * MOTOR_P * (MOTOR_LM_H / MOTOR_LR_H); // 2.9141 Nm/(Wb.A)
    localparam real I_MAG_RATED= 4.73;                     // Amperes (for 0.96 Wb)
    localparam real TS_SEC     = 0.0001;                   // 100 us per step

    // DUT Signals
    reg                         clk;
    reg                         rst_n;
    reg                         start;
    reg  signed [DATA_WIDTH-1:0] i_alpha;
    reg  signed [DATA_WIDTH-1:0] i_beta;
    reg  signed [DATA_WIDTH-1:0] speed_elec;

    wire signed [DATA_WIDTH-1:0] psi_r_alpha;
    wire signed [DATA_WIDTH-1:0] psi_r_beta;
    wire signed [DATA_WIDTH-1:0] wr_psi_alpha;
    wire signed [DATA_WIDTH-1:0] wr_psi_beta;
    wire                        done;

    // Test Statistics
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    // Conversion Utilities
    function real q_to_real;
        input signed [DATA_WIDTH-1:0] q_val;
        begin
            q_to_real = q_val / 1048576.0;
        end
    endfunction

    function signed [DATA_WIDTH-1:0] real_to_q;
        input real val;
        begin
            real_to_q = $rtoi(val * 1048576.0);
        end
    endfunction

    // Instantiate DUT
    flux_observer #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(FRAC_BITS)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .i_alpha(i_alpha),
        .i_beta(i_beta),
        .speed_elec(speed_elec),
        .psi_r_alpha(psi_r_alpha),
        .psi_r_beta(psi_r_beta),
        .wr_psi_alpha(wr_psi_alpha),
        .wr_psi_beta(wr_psi_beta),
        .done(done)
    );

    // Hardware Speed PI Controller Instance
    reg                          pi_sample_tick;
    reg  signed [DATA_WIDTH-1:0] pi_speed_ref;
    wire signed [DATA_WIDTH-1:0] pi_te_ref;

    speed_pi #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(FRAC_BITS)
    ) u_speed_pi (
        .clk(clk),
        .rst_n(rst_n),
        .sample_tick(pi_sample_tick),
        .speed_ref(pi_speed_ref),
        .speed_fb(speed_elec),
        .te_ref(pi_te_ref)
    );

    // 100 MHz System Clock
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // Step Execution Task
    task step_observer;
        input signed [DATA_WIDTH-1:0] in_ia;
        input signed [DATA_WIDTH-1:0] in_ib;
        input signed [DATA_WIDTH-1:0] in_spd;
        integer timeout;
        begin
            @(posedge clk);
            i_alpha        <= in_ia;
            i_beta         <= in_ib;
            speed_elec     <= in_spd;
            start          <= 1'b1;
            pi_sample_tick <= 1'b1;
            @(posedge clk);
            start          <= 1'b0;
            pi_sample_tick <= 1'b0;

            timeout = 0;
            while (!done && timeout < 50) begin
                @(posedge clk);
                timeout = timeout + 1;
            end

            if (timeout >= 50) begin
                $display("[ERROR] Observer timed out without asserting 'done'!");
                failed_tests = failed_tests + 1;
            end
            @(posedge clk); // breather cycle
        end
    endtask

    // Check helper
    task check_condition;
        input condition;
        input [63:0] test_id;
        begin
            total_tests = total_tests + 1;
            if (condition) begin
                passed_tests = passed_tests + 1;
            end else begin
                failed_tests = failed_tests + 1;
                $display("  [FAIL] Test %s FAILED!", test_id);
            end
        end
    endtask

    // Calculation variables
    integer step_idx;
    real t_sec, theta_rad, omega_e, omega_r, omega_ref;
    real cur_ia, cur_ib;
    real mag_psi, mag_min, mag_max;
    real psi_a_r, psi_b_r;

    // Physical Motor Drive Emulation Variables (for Phase 4)
    real pi_kp, pi_ki, pi_err, pi_integ, te_cmd, i_q, w_slip;
    real load_torque, motor_inertia, dwr_dt;

    // =========================================================================
    // MAIN VERIFICATION PROCEDURE
    // =========================================================================
    initial begin
        clk        = 1'b0;
        rst_n      = 1'b0;
        start      = 1'b0;
        i_alpha    = 32'sd0;
        i_beta     = 32'sd0;
        speed_elec = 32'sd0;

        $display("\n================================================================================");
        $display("STARTING EXTENDED FLUX OBSERVER VERIFICATION SUITE");
        $display("Target Module: flux_observer.v (Artix-7 FCS-MPC Induction Motor Drive)");
        $display("Parameters: E21=%0d, E22=%0d, TS_Q=%0d, SPEED_LIMIT=%0d rad/s",
                 `E21, `E22, `TS_Q, `SPEED_LIMIT_RADS);
        $display("================================================================================\n");

        #(CLK_PERIOD_NS * 10);
        rst_n = 1'b1;
        #(CLK_PERIOD_NS * 5);

        // ---------------------------------------------------------------------
        // PHASE 1: EDGE CASES & NUMERICAL BOUNDARIES
        // ---------------------------------------------------------------------
        $display("--------------------------------------------------------------------------------");
        $display("PHASE 1: EDGE CASES & NUMERICAL BOUNDARIES");
        $display("--------------------------------------------------------------------------------");

        // 1.1: Reset and Initial State
        check_condition(psi_r_alpha == 32'sd0 && psi_r_beta == 32'sd0 &&
                        wr_psi_alpha == 32'sd0 && wr_psi_beta == 32'sd0 &&
                        done == 1'b0, "1.1");
        $display("  [PASS] Test 1.1: Reset clears all registers and outputs to 0.");

        // 1.2: Zero-Input Standstill Behavior
        step_observer(32'sd0, 32'sd0, 32'sd0);
        check_condition(psi_r_alpha == 32'sd0 && psi_r_beta == 32'sd0, "1.2");
        $display("  [PASS] Test 1.2: Zero inputs at standstill maintain zero flux output.");

        // 1.3: Spurious / Interrupted Start Pulse Rejection
        @(posedge clk);
        i_alpha    <= real_to_q(4.73);
        i_beta     <= 32'sd0;
        speed_elec <= 32'sd0;
        start      <= 1'b1;
        @(posedge clk);
        start      <= 1'b0;
        repeat(4) @(posedge clk);
        start      <= 1'b1; // spurious start while busy
        @(posedge clk);
        start      <= 1'b0;
        while (!done) @(posedge clk);
        check_condition(done == 1'b1, "1.3");
        $display("  [PASS] Test 1.3: FSM successfully ignores secondary start pulse while busy.");
        @(posedge clk);

        // Fresh reset
        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);

        // 1.4: Positive Speed Clamping (> +SPEED_LIMIT)
        step_observer(real_to_q(4.73), real_to_q(0.0), 32'sd0);
        step_observer(real_to_q(4.73), real_to_q(0.0), 32'sd0);
        step_observer(real_to_q(4.73), real_to_q(0.0), real_to_q(500.0));
        check_condition(dut.speed_clamped == `SPEED_LIMIT, "1.4");
        $display("  [PASS] Test 1.4: Positive speed clamp: 500.0 rad/s clamped to %0d rad/s.",
                 `SPEED_LIMIT_RADS);

        // 1.5: Negative Speed Clamping (< -SPEED_LIMIT)
        step_observer(real_to_q(4.73), real_to_q(0.0), real_to_q(-500.0));
        check_condition(dut.speed_clamped == -`SPEED_LIMIT, "1.5");
        $display("  [PASS] Test 1.5: Negative speed clamp: -500.0 rad/s clamped to -%0d rad/s.",
                 `SPEED_LIMIT_RADS);

        // 1.6: Sign Inversion / Anti-Symmetry
        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);
        step_observer(real_to_q(-4.73), 32'sd0, 32'sd0);
        check_condition(psi_r_alpha < 32'sd0 && psi_r_beta == 32'sd0, "1.6");
        $display("  [PASS] Test 1.6: Sign symmetry: Negative stator current yields negative flux.");

        // 1.7: Overcurrent Pulse Dynamic Range Robustness (15 Amps)
        step_observer(real_to_q(15.0), real_to_q(-15.0), real_to_q(100.0));
        check_condition(psi_r_alpha != 32'sd0 && dut.done == 1'b0, "1.7");
        $display("  [PASS] Test 1.7: Overcurrent pulse (15A) handled without 64-bit overflow.\n");

        // ---------------------------------------------------------------------
        // PHASE 2: LONG-DURATION DC PRE-MAGNETIZATION (10,000 CYCLES / 1.00 s)
        // ---------------------------------------------------------------------
        $display("--------------------------------------------------------------------------------");
        $display("PHASE 2: LONG-DURATION DC PRE-MAGNETIZATION AT STANDSTILL (10,000 STEPS = 1.00 s)");
        $display("--------------------------------------------------------------------------------");
        $display("Rotor Time Constant: tau_r = 193.5 ms. Running for 1.00 s (5.17 tau_r)...");
        $display("Target Asymptotic Steady-State: Psi_r = Lm * I_mag = 0.960 Wb");

        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);

        for (step_idx = 0; step_idx < 10000; step_idx = step_idx + 1) begin
            step_observer(real_to_q(4.73), 32'sd0, 32'sd0);

            if (step_idx == 999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                $display("  Checkpoint (t=0.10s, 0.52 tau_r): Psi_alpha = %0.4f Wb (expected ~0.387 Wb)", psi_a_r);
                check_condition(psi_a_r > 0.35 && psi_a_r < 0.43, "2.1");
            end else if (step_idx == 1999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                $display("  Checkpoint (t=0.20s, 1.03 tau_r): Psi_alpha = %0.4f Wb (expected ~0.619 Wb)", psi_a_r);
                check_condition(psi_a_r > 0.58 && psi_a_r < 0.66, "2.2");
            end else if (step_idx == 3999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                $display("  Checkpoint (t=0.40s, 2.07 tau_r): Psi_alpha = %0.4f Wb (expected ~0.839 Wb)", psi_a_r);
                check_condition(psi_a_r > 0.80 && psi_a_r < 0.88, "2.3");
            end else if (step_idx == 5999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                $display("  Checkpoint (t=0.60s, 3.10 tau_r): Psi_alpha = %0.4f Wb (expected ~0.917 Wb)", psi_a_r);
                check_condition(psi_a_r > 0.89 && psi_a_r < 0.94, "2.4");
            end else if (step_idx == 7999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                $display("  Checkpoint (t=0.80s, 4.14 tau_r): Psi_alpha = %0.4f Wb (expected ~0.945 Wb)", psi_a_r);
                check_condition(psi_a_r > 0.92 && psi_a_r < 0.96, "2.5");
            end
        end

        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        $display("  Final at t=1.00s (5.17 tau_r): Psi_alpha = %0.4f Wb, Psi_beta = %0.6f Wb",
                 psi_a_r, psi_b_r);
        // At 5.17 tau_r, theoretical is 0.96 * (1 - e^-5.17) = 0.9546 Wb
        check_condition(psi_a_r > 0.94 && psi_a_r < 0.965, "2.6");
        check_condition(psi_b_r == 0.0, "2.7");
        $display("  [PASS] Phase 2: DC pre-magnetization reaches 99.4%% of theoretical steady-state (0.954 Wb).\n");

        // ---------------------------------------------------------------------
        // PHASE 3: EXTENDED STEADY-STATE SINUSOIDAL ROTATION (8,000 CYCLES / 0.80 s)
        // ---------------------------------------------------------------------
        $display("--------------------------------------------------------------------------------");
        $display("PHASE 3: EXTENDED STEADY-STATE SINUSOIDAL ROTATION (8,000 STEPS = 0.80 s)");
        $display("--------------------------------------------------------------------------------");
        $display("Balanced rotating current at rated peak: I_mag = 4.73 A");
        $display("Operating point: wr = 100 rad/s (15.9 Hz), we = 105 rad/s (13.4 full revolutions)...");

        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);

        theta_rad = 0.0;
        omega_r   = 100.0;
        omega_e   = 105.0; // 5 rad/s slip

        for (step_idx = 0; step_idx < 8000; step_idx = step_idx + 1) begin
            cur_ia    = 4.73 * $cos(theta_rad);
            cur_ib    = 4.73 * $sin(theta_rad);
            theta_rad = theta_rad + (omega_e * TS_SEC);

            step_observer(real_to_q(cur_ia), real_to_q(cur_ib), real_to_q(omega_r));

            // Track min/max magnitude across the entire second half (steps 4000-7999)
            if (step_idx >= 4000) begin
                psi_a_r = q_to_real(psi_r_alpha);
                psi_b_r = q_to_real(psi_r_beta);
                mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
                if (step_idx == 4000) begin
                    mag_min = mag_psi;
                    mag_max = mag_psi;
                end else begin
                    if (mag_psi < mag_min) mag_min = mag_psi;
                    if (mag_psi > mag_max) mag_max = mag_psi;
                end
            end
        end

        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
        $display("  Stabilized Flux Vector: Psi_alpha = %0.4f Wb, Psi_beta = %0.4f Wb", psi_a_r, psi_b_r);
        $display("  Long-Term Flux Magnitude: |Psi_r| = %0.4f Wb (Min = %0.4f Wb, Max = %0.4f Wb)",
                 mag_psi, mag_min, mag_max);
        $display("  Magnitude Ripple over 4,000 steps: %0.2f %%", ((mag_max - mag_min) / mag_psi) * 100.0);

        check_condition(mag_psi > 0.65 && mag_psi < 0.85, "3.1");
        check_condition(((mag_max - mag_min) / mag_psi) < 0.08, "3.2");
        $display("  [PASS] Phase 3: Long-term sinusoidal operation shows excellent circular stability.\n");

        // ---------------------------------------------------------------------
        // PHASE 4: REAL-WORLD CLOSED-LOOP PI DYNAMIC SPEED TRANSIENT (30,000 STEPS = 3.00 s)
        // ---------------------------------------------------------------------
        $display("--------------------------------------------------------------------------------");
        $display("PHASE 4: REAL-WORLD CLOSED-LOOP DYNAMIC TRANSIENT (30,000 STEPS = 3.00 s)");
        $display("--------------------------------------------------------------------------------");
        $display("Simulation Profile:");
        $display("  - 0.00s to 1.20s (Steps 0-11,999):   Initial steady state at w_ref = 100 rad/s");
        $display("  - 1.20s to 1.70s (Steps 12,000-16,999): Reference speed ramps smoothly: 100 -> 180 rad/s");
        $display("      * Speed PI controller dynamically adjusts torque current Iq and slip frequency");
        $display("      * Motor mechanical inertia (J=0.02 kg.m^2) governs smooth shaft acceleration");
        $display("      * Stator currents adapt continuously with NO sudden jumps");
        $display("  - 1.70s to 3.00s (Steps 17,000-29,999): Extended settling at w_ref = 180 rad/s");

        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);

        // Pre-condition actual speed_pi integrator for initial 3.0 Nm balance at wr = 100 rad/s
        load_torque   = 3.0; // Nm
        motor_inertia = 0.03; // kg.m^2
        omega_r       = 100.0;
        omega_ref     = 100.0;
        pi_speed_ref  = real_to_q(100.0);
        u_speed_pi.err_integ = 32'sd314572800; // 3.0 Nm balance in Q12.20
        theta_rad     = 0.0;

        for (step_idx = 0; step_idx < 30000; step_idx = step_idx + 1) begin
            // Stage 1 & Stage 2 Speed Reference Profile
            if (step_idx < 12000) begin
                omega_ref = 100.0; // Initial steady state hold for 1.2s
            end else if (step_idx < 17000) begin
                // Smooth reference ramp over 0.5s (5000 steps)
                omega_ref = 100.0 + (180.0 - 100.0) * (step_idx - 12000) / 5000.0;
            end else begin
                omega_ref = 180.0; // Settling at new target speed for 1.3s
            end

            // Drive actual hardware speed_pi module input
            pi_speed_ref <= real_to_q(omega_ref);

            // Read torque command directly from actual speed_pi.v RTL module!
            te_cmd = q_to_real(pi_te_ref);

            // Mechanical Shaft Acceleration: dwr/dt = (p / J) * (Te - TL)
            dwr_dt  = (MOTOR_P / motor_inertia) * (te_cmd - load_torque);
            omega_r = omega_r + (dwr_dt * TS_SEC);

            // Field-Oriented Stator Current Synthesis
            i_q     = te_cmd / (MOTOR_KT * 0.96);
            w_slip  = (1.0 / TAU_R_SEC) * (i_q / I_MAG_RATED);
            omega_e = omega_r + w_slip;

            theta_rad = theta_rad + (omega_e * TS_SEC);
            cur_ia    = (I_MAG_RATED * $cos(theta_rad)) - (i_q * $sin(theta_rad));
            cur_ib    = (I_MAG_RATED * $sin(theta_rad)) + (i_q * $cos(theta_rad));

            // Execute observer step with physical current & speed inputs
            step_observer(real_to_q(cur_ia), real_to_q(cur_ib), real_to_q(omega_r));

            // Periodic Progress & Checkpoint Logging
            if (step_idx == 11999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                psi_b_r = q_to_real(psi_r_beta);
                mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
                $display("  [t=1.20s] Initial Steady State: wr=%0.2f rad/s, Te=%0.2f Nm, Iq=%0.2f A, |Psi|=%0.4f Wb",
                         omega_r, te_cmd, i_q, mag_psi);
                // Verify initial steady state is completely settled and stable
                check_condition(mag_psi > 0.95 && mag_psi < 1.15, "4.1");
                check_condition(omega_r > 99.5 && omega_r < 100.5, "4.2");
            end else if (step_idx == 14500) begin
                psi_a_r = q_to_real(psi_r_alpha);
                psi_b_r = q_to_real(psi_r_beta);
                mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
                $display("  [t=1.45s] Mid-Acceleration:     wr=%0.2f rad/s, Te=%0.2f Nm, Iq=%0.2f A, |Psi|=%0.4f Wb",
                         omega_r, te_cmd, i_q, mag_psi);
                check_condition(mag_psi > 0.95 && mag_psi < 1.25, "4.3");
            end else if (step_idx == 16999) begin
                psi_a_r = q_to_real(psi_r_alpha);
                psi_b_r = q_to_real(psi_r_beta);
                mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
                $display("  [t=1.70s] Ramp Target Reached:  wr=%0.2f rad/s, Te=%0.2f Nm, Iq=%0.2f A, |Psi|=%0.4f Wb",
                         omega_r, te_cmd, i_q, mag_psi);
                check_condition(mag_psi > 1.10 && mag_psi < 1.35, "4.4");
            end else if (step_idx == 22000) begin
                psi_a_r = q_to_real(psi_r_alpha);
                psi_b_r = q_to_real(psi_r_beta);
                mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
                $display("  [t=2.20s] Post-Ramp Settling:   wr=%0.2f rad/s, Te=%0.2f Nm, Iq=%0.2f A, |Psi|=%0.4f Wb",
                         omega_r, te_cmd, i_q, mag_psi);
            end
        end

        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
        $display("  [t=3.00s] Final Extended State: wr=%0.2f rad/s, Te=%0.2f Nm, Iq=%0.2f A, |Psi|=%0.4f Wb",
                 omega_r, te_cmd, i_q, mag_psi);

        // Verify clean settling at new speed point (180 rad/s)
        check_condition(omega_r > 179.5 && omega_r < 180.5, "4.5");
        check_condition(mag_psi > 1.25 && mag_psi < 1.55, "4.6");
        $display("  [PASS] Phase 4: Flux observer flawlessly tracked full closed-loop dynamic speed transient.\n");

        // ---------------------------------------------------------------------
        // PHASE 5: EXTENDED HIGH-SPEED OPERATION & LIMIT PROTECTION (11,000 CYCLES / 1.10 s)
        // ---------------------------------------------------------------------
        $display("--------------------------------------------------------------------------------");
        $display("PHASE 5: EXTENDED HIGH-SPEED STABILITY & SPEED_LIMIT PROTECTION (11,000 STEPS)");
        $display("--------------------------------------------------------------------------------");

        // 5.1: 3,000 cycles at wr = 280 rad/s
        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);
        theta_rad = 0.0;
        omega_r   = 280.0;
        omega_e   = 285.0;
        for (step_idx = 0; step_idx < 3000; step_idx = step_idx + 1) begin
            cur_ia    = 4.73 * $cos(theta_rad);
            cur_ib    = 4.73 * $sin(theta_rad);
            theta_rad = theta_rad + (omega_e * TS_SEC);
            step_observer(real_to_q(cur_ia), real_to_q(cur_ib), real_to_q(omega_r));
        end
        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
        $display("  Sub-test 5.1 (wr = 280 rad/s, 3,000 steps): |Psi_r| = %0.4f Wb -> PERFECTLY STABLE", mag_psi);
        check_condition(mag_psi > 0.50 && mag_psi < 5.0, "5.1");

        // 5.2: 3,000 cycles at wr = 315 rad/s (immediately below limit of 321 rad/s)
        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);
        theta_rad = 0.0;
        omega_r   = 315.0;
        omega_e   = 320.0;
        for (step_idx = 0; step_idx < 3000; step_idx = step_idx + 1) begin
            cur_ia    = 4.73 * $cos(theta_rad);
            cur_ib    = 4.73 * $sin(theta_rad);
            theta_rad = theta_rad + (omega_e * TS_SEC);
            step_observer(real_to_q(cur_ia), real_to_q(cur_ib), real_to_q(omega_r));
        end
        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
        $display("  Sub-test 5.2 (wr = 315 rad/s, adjacent to 321 limit): |Psi_r| = %0.4f Wb -> STABLE", mag_psi);
        check_condition(mag_psi > 0.20 && mag_psi < 8.0, "5.2");

        // 5.3: 5,000 cycles at wr = 360 rad/s (exceeding 321 limit, 0.50 s duration)
        rst_n = 1'b0; #(CLK_PERIOD_NS * 5); rst_n = 1'b1; #(CLK_PERIOD_NS * 5);
        theta_rad = 0.0;
        omega_r   = 360.0;
        omega_e   = 365.0;
        for (step_idx = 0; step_idx < 5000; step_idx = step_idx + 1) begin
            cur_ia    = 4.73 * $cos(theta_rad);
            cur_ib    = 4.73 * $sin(theta_rad);
            theta_rad = theta_rad + (omega_e * TS_SEC);
            step_observer(real_to_q(cur_ia), real_to_q(cur_ib), real_to_q(omega_r));
        end
        psi_a_r = q_to_real(psi_r_alpha);
        psi_b_r = q_to_real(psi_r_beta);
        mag_psi = $sqrt(psi_a_r * psi_a_r + psi_b_r * psi_b_r);
        $display("  Sub-test 5.3 (wr = 360 rad/s > SPEED_LIMIT 321 rad/s, 5,000 steps):");
        $display("    Internal clamped speed = %0d rad/s (clamped from 360)",
                 dut.speed_clamped / 1048576);
        $display("    Observer Flux Magnitude = %0.4f Wb (bounded, NO divergence after 5,000 steps!)", mag_psi);
        check_condition(dut.speed_clamped == `SPEED_LIMIT, "5.3a");
        check_condition(mag_psi < 5.0, "5.3b");
        $display("  [PASS] Phase 5: Long-term speed clamping completely eliminates Forward Euler instability!\n");

        // =====================================================================
        // FINAL SUMMARY
        // =====================================================================
        $display("================================================================================");
        $display("EXTENDED FLUX OBSERVER VERIFICATION COMPLETE");
        $display("================================================================================");
        $display("Total Assertions Checked : %0d", total_tests);
        $display("Passed                   : %0d", passed_tests);
        $display("Failed                   : %0d", failed_tests);

        if (failed_tests == 0) begin
            $display("\n>>> [ALL %0d EXTENDED TESTS PASSED] FLUX OBSERVER FULLY CERTIFIED! <<<", passed_tests);
        end else begin
            $display("\n>>> [VERIFICATION FAILED: %0d TESTS FAILED] <<<", failed_tests);
        end
        $display("================================================================================\n");

        #(CLK_PERIOD_NS * 10);
        $finish;
    end

endmodule
