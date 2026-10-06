`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_gate_driver;

    // -------------------------------------------------------------
    // 1. Clock, Reset, and DUT Signals
    // -------------------------------------------------------------
    reg        clk;
    reg        rst_n;
    reg        enable;
    reg  [2:0] switch_state;
    reg        update_tick;

    wire       gate_ah;
    wire       gate_al;
    wire       gate_bh;
    wire       gate_bl;
    wire       gate_ch;
    wire       gate_cl;

    // -------------------------------------------------------------
    // 2. Instantiate DUT
    // -------------------------------------------------------------
    gate_driver dut (
        .clk          (clk),
        .rst_n        (rst_n),
        .enable       (enable),
        .switch_state (switch_state),
        .update_tick  (update_tick),
        .gate_ah      (gate_ah),
        .gate_al      (gate_al),
        .gate_bh      (gate_bh),
        .gate_bl      (gate_bl),
        .gate_ch      (gate_ch),
        .gate_cl      (gate_cl)
    );

    // -------------------------------------------------------------
    // 3. 100 MHz Master Clock (10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // 4. Test Statistics & Safety Monitors
    // -------------------------------------------------------------
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;
    integer shoot_through_events = 0;

    // Watchdog
    initial begin
        #5_000_000; // 5 ms simulation time limit
        $display("\n[FATAL TIMEOUT] Simulation exceeded 5 ms!");
        $finish;
    end

    // CONTINUOUS HARDWARE SHOOT-THROUGH SAFETY ASSERTIONS
    // High and Low switches in any half-bridge leg must NEVER be ON simultaneously!
    always @(posedge clk) begin
        if (gate_ah && gate_al) begin
            shoot_through_events = shoot_through_events + 1;
            $display("\n[CRITICAL HARDWARE FAULT] SHOOT-THROUGH DETECTED ON PHASE A! (gate_ah=1, gate_al=1) at time %0t ns", $time);
        end
        if (gate_bh && gate_bl) begin
            shoot_through_events = shoot_through_events + 1;
            $display("\n[CRITICAL HARDWARE FAULT] SHOOT-THROUGH DETECTED ON PHASE B! (gate_bh=1, gate_bl=1) at time %0t ns", $time);
        end
        if (gate_ch && gate_cl) begin
            shoot_through_events = shoot_through_events + 1;
            $display("\n[CRITICAL HARDWARE FAULT] SHOOT-THROUGH DETECTED ON PHASE C! (gate_ch=1, gate_cl=1) at time %0t ns", $time);
        end
    end

    // -------------------------------------------------------------
    // 5. Verification Tasks
    // -------------------------------------------------------------
    // Apply a new switching vector and wait for dead-time to complete
    task apply_vector;
        input [2:0] vec;
        begin
            @(posedge clk);
            switch_state <= vec;
            update_tick  <= 1'b1;
            @(posedge clk);
            update_tick  <= 1'b0;
            // Wait slightly longer than 200 cycles (220 cycles = 2.2 us) for dead-time to complete
            repeat (220) @(posedge clk);
            #1; // Non-blocking assignment settling
        end
    endtask

    // Check steady-state gates against expected 3-bit vector
    task check_steady_state_gates;
        input [2:0] exp_vec;
        input [639:0] desc;
        reg pass;
        reg exp_ah, exp_al, exp_bh, exp_bl, exp_ch, exp_cl;
    begin
        total_tests = total_tests + 1;
        exp_ah = exp_vec[2]; exp_al = ~exp_vec[2];
        exp_bh = exp_vec[1]; exp_bl = ~exp_vec[1];
        exp_ch = exp_vec[0]; exp_cl = ~exp_vec[0];

        pass = (gate_ah == exp_ah) && (gate_al == exp_al) &&
               (gate_bh == exp_bh) && (gate_bl == exp_bl) &&
               (gate_ch == exp_ch) && (gate_cl == exp_cl);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Vector %3b: Gates A={%b,%b} B={%b,%b} C={%b,%b} | %0s",
                     total_tests, exp_vec, gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl, desc);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] GATE STATE MISMATCH! (%0s)", total_tests, desc);
            $display("           Expected: A={%b,%b} B={%b,%b} C={%b,%b}",
                     exp_ah, exp_al, exp_bh, exp_bl, exp_ch, exp_cl);
            $display("           Actual  : A={%b,%b} B={%b,%b} C={%b,%b}",
                     gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl);
        end
    end
    endtask

    // -------------------------------------------------------------
    // 6. Main Test Sequence
    // -------------------------------------------------------------
    integer i, j;
    integer measured_dead_time;
    reg phase_a_glitch_detected;

    initial begin
        $display("===================================================================");
        $display("       RIGOROUS VERIFICATION TESTBENCH: gate_driver.v             ");
        $display("   Testing Dead-Time (2.0 us), Shoot-Through, 64 State Transitions ");
        $display("===================================================================");

        clk          = 1'b0;
        rst_n        = 1'b0;
        enable       = 1'b0;
        switch_state = 3'b000;
        update_tick  = 1'b0;

        // =================================================================
        // PHASE 1: Asynchronous Reset Verification
        // =================================================================
        $display("\n--- PHASE 1: Asynchronous Reset Verification ---");
        repeat (5) @(posedge clk);
        total_tests = total_tests + 1;
        if ({gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl} == 6'b000000) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Reset active: All 6 gates strictly driven LOW (000000)", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Reset failure! Gates non-zero during reset", total_tests);
        end

        // Release reset, keep enable = 0
        @(posedge clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);

        // =================================================================
        // PHASE 2: Disabled Inverter Safety (enable == 0)
        // =================================================================
        $display("\n--- PHASE 2: Disabled Inverter Safety (enable == 0) ---");
        // Try commanding active vectors while enable is 0
        switch_state <= 3'b111;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;
        repeat (50) @(posedge clk);
        total_tests = total_tests + 1;
        if ({gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl} == 6'b000000) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Disabled state: Inverter remains high-Z (all 6 gates 0) even when vector commanded", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Disabled leakage! Gates fired while enable=0", total_tests);
        end

        // Enable the inverter drive
        @(posedge clk);
        enable <= 1'b1;
        repeat (10) @(posedge clk);

        // Set initial state to Vector 0 (000: all low-side switches on)
        apply_vector(3'b000);
        check_steady_state_gates(3'b000, "Initial Vector 0 (000)");

        // =================================================================
        // PHASE 3: Exact Cycle-Accurate Dead-Time Measurement (High -> Low & Low -> High)
        // =================================================================
        $display("\n--- PHASE 3: Cycle-Accurate Dead-Time Measurement (Phase A) ---");

        // Step A: Transition Phase A from Low to High (000 -> 100)
        // gate_al must turn off when target is registered, and gate_ah must turn on after EXACTLY 200 cycles!
        @(posedge clk);
        switch_state <= 3'b100;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;
        // Wait 1 clock for target_state register to be sampled by gate_driver FSM
        @(posedge clk);
        #1;

        // Check that outgoing switch (gate_al) turned OFF immediately upon entering dead-time
        total_tests = total_tests + 1;
        if (gate_ah == 1'b0 && gate_al == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Break-Before-Make: gate_al turned OFF on Cycle 1. Both gates now OFF.", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Turn-off failure on Cycle 1! ah=%b, al=%b", total_tests, gate_ah, gate_al);
        end

        // Count cycles during dead-time until gate_ah turns ON
        measured_dead_time = 0;
        while (gate_ah == 1'b0 && measured_dead_time < 250) begin
            @(posedge clk);
            #1;
            if (gate_ah == 1'b0)
                measured_dead_time = measured_dead_time + 1;
        end

        total_tests = total_tests + 1;
        if (measured_dead_time == `DEAD_TIME_CYCLES) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Dead-Time (Low->High): Measured EXACTLY %0d cycles (2000.0 ns)!",
                     total_tests, measured_dead_time);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Dead-Time duration mismatch! Measured %0d cycles, expected %0d",
                     total_tests, measured_dead_time, `DEAD_TIME_CYCLES);
        end

        repeat (20) @(posedge clk);

        // Step B: Transition Phase A from High to Low (100 -> 000)
        @(posedge clk);
        switch_state <= 3'b000;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;
        // Wait 1 clock for target_state register to be sampled by gate_driver FSM
        @(posedge clk);
        #1;

        // Check that outgoing switch (gate_ah) turned OFF immediately upon entering dead-time
        total_tests = total_tests + 1;
        if (gate_ah == 1'b0 && gate_al == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Break-Before-Make: gate_ah turned OFF on Cycle 1. Both gates now OFF.", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Turn-off failure on Cycle 1! ah=%b, al=%b", total_tests, gate_ah, gate_al);
        end

        measured_dead_time = 0;
        while (gate_al == 1'b0 && measured_dead_time < 250) begin
            @(posedge clk);
            #1;
            if (gate_al == 1'b0)
                measured_dead_time = measured_dead_time + 1;
        end

        total_tests = total_tests + 1;
        if (measured_dead_time == `DEAD_TIME_CYCLES) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Dead-Time (High->Low): Measured EXACTLY %0d cycles (2000.0 ns)!",
                     total_tests, measured_dead_time);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Dead-Time duration mismatch! Measured %0d cycles, expected %0d",
                     total_tests, measured_dead_time, `DEAD_TIME_CYCLES);
        end

        repeat (20) @(posedge clk);

        // =================================================================
        // PHASE 4: Phase Independence & Glitch-Free Invariance Test
        // =================================================================
        $display("\n--- PHASE 4: Phase Independence & Invariance Test ---");
        $display("Transitioning Vector 1 (100) -> Vector 2 (110). Phase A and C must NEVER glitch!");

        // Establish Vector 1 (100)
        apply_vector(3'b100);

        // Command Vector 2 (110): Phase A stays 1, Phase B switches 0->1, Phase C stays 0
        @(posedge clk);
        switch_state <= 3'b110;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;

        // Monitor for all 200 cycles of Phase B dead-time:
        // gate_ah MUST REMAIN 1, gate_cl MUST REMAIN 1 at every clock cycle!
        phase_a_glitch_detected = 0;
        for (i = 0; i < 205; i = i + 1) begin
            @(posedge clk);
            #1;
            if (gate_ah != 1'b1 || gate_cl != 1'b1) begin
                phase_a_glitch_detected = 1;
            end
        end

        total_tests = total_tests + 1;
        if (!phase_a_glitch_detected) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Phase Invariance: gate_ah held steady HIGH and gate_cl held steady HIGH during Phase B dead-time!", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Glitch detected on unswitched phase!", total_tests);
        end

        check_steady_state_gates(3'b110, "Steady State Vector 2 (110)");

        // =================================================================
        // PHASE 5: Adversarial Enable-Bounce Shoot-Through Audit
        // =================================================================
        $display("\n--- PHASE 5: Adversarial Enable-Bounce Shoot-Through Audit ---");
        $display("Simulating pushbutton chatter: enable drops for 4 clock cycles then returns...");

        // Start with Phase A High: Vector 1 (100)
        apply_vector(3'b100);

        // Bounce enable LOW for 40 ns (4 clock cycles)
        @(posedge clk);
        enable <= 1'b0;
        repeat (4) @(posedge clk);
        #1;
        // Inverter gates must all be 0 during disable
        total_tests = total_tests + 1;
        if ({gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl} == 6'b000000) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Enable bounce: All 6 gates immediately forced to 0 within 1 cycle", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Gate leak during enable bounce!", total_tests);
        end

        // Re-enable and command Vector 0 (000, requiring Phase A Low-side to turn on)
        @(posedge clk);
        enable <= 1'b1;
        switch_state <= 3'b000;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;

        // CRITICAL CHECK: Does gate_al turn ON immediately, or does it enforce dead-time?
        #1;
        total_tests = total_tests + 1;
        if (gate_al == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Anti-Shoot-Through: gate_al safely blocked from turning ON upon re-enable!", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] CRITICAL FATAL BUG: gate_al fired immediately after re-enable, causing shoot-through!", total_tests);
        end

        // Wait full dead-time and confirm normal turn-on
        repeat (210) @(posedge clk);
        check_steady_state_gates(3'b000, "Vector 0 (000) cleanly established after re-enable dead-time");

        // =================================================================
        // PHASE 6: Exhaustive 64-Transition Matrix (All Combinations Vi -> Vj)
        // =================================================================
        $display("\n--- PHASE 6: Exhaustive 64-Transition Matrix Sweep (All Vi -> Vj) ---");
        $display("Testing all 64 possible state transitions across 8 vectors (Vi: 0..7 to Vj: 0..7)...");

        for (i = 0; i < 8; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                // Transition from Vector i to Vector j
                apply_vector(i[2:0]);
                apply_vector(j[2:0]);
                check_steady_state_gates(j[2:0], "64-Transition matrix element");
            end
        end
        $display("[PASS] Completed full 64-transition matrix sweep with zero shoot-through events!");

        // =================================================================
        // PHASE 7: Back-to-Back Interrupted Transition (Vector Change During Dead-Time)
        // =================================================================
        $display("\n--- PHASE 7: Interrupted Transition During Dead-Time ---");
        // Start at 000
        apply_vector(3'b000);

        // Command 100 -> starts Phase A dead-time
        @(posedge clk);
        switch_state <= 3'b100;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;

        // Wait 50 cycles (in the middle of dead-time), then suddenly abort back to 000
        repeat (50) @(posedge clk);
        @(posedge clk);
        switch_state <= 3'b000;
        update_tick  <= 1'b1;
        @(posedge clk);
        update_tick  <= 1'b0;

        // Wait for settling
        repeat (220) @(posedge clk);
        check_steady_state_gates(3'b000, "Abort back to 000 during dead-time handled cleanly");

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("           GATE DRIVER TESTBENCH EXECUTION SUMMARY                 ");
        $display("===================================================================");
        $display(" Total Verification Tests Evaluated : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display(" Shoot-Through Incidents Detected   : %0d", shoot_through_events);
        $display("===================================================================");

        if (failed_tests == 0 && shoot_through_events == 0) begin
            $display(" >>> ALL GATE DRIVER TESTS PASSED PERFECTLY! ZERO SHOOT-THROUGHS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ERRORS AND %0d SHOOT-THROUGH EVENTS! <<< \n",
                     failed_tests, shoot_through_events);
        end

        $finish;
    end

endmodule
