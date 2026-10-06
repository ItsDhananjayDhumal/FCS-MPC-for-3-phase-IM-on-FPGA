`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_vdc_manager;

    // -------------------------------------------------------------
    // 1. Clock, Reset, and DUT Signals
    // -------------------------------------------------------------
    reg                          clk;
    reg                          rst_n;
    reg  [7:0]                   sw;

    // vdc_manager outputs
    wire signed [`DATA_WIDTH-1:0] vdc_q;
    wire signed [`DATA_WIDTH-1:0] v1_alpha_out;
    wire signed [`DATA_WIDTH-1:0] v2_alpha_out;
    wire signed [`DATA_WIDTH-1:0] v2_beta_out;
    wire                         vdc_valid;

    // voltage_lut signals
    reg  [2:0]                   vec_idx;
    wire signed [`DATA_WIDTH-1:0] vs_alpha;
    wire signed [`DATA_WIDTH-1:0] vs_beta;
    wire [2:0]                   switch_state;

    // -------------------------------------------------------------
    // 2. Instantiate DUTs (Connected as in mpc_top.v)
    // -------------------------------------------------------------
    vdc_manager #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) u_vdc (
        .clk          (clk),
        .rst_n        (rst_n),
        .sw           (sw),
        .vdc_q        (vdc_q),
        .v1_alpha_out (v1_alpha_out),
        .v2_alpha_out (v2_alpha_out),
        .v2_beta_out  (v2_beta_out),
        .vdc_valid    (vdc_valid)
    );

    voltage_lut #(
        .DATA_WIDTH(`DATA_WIDTH)
    ) u_vlut (
        .vec_idx      (vec_idx),
        .v1_alpha     (v1_alpha_out),
        .v2_alpha     (v2_alpha_out),
        .v2_beta      (v2_beta_out),
        .vs_alpha     (vs_alpha),
        .vs_beta      (vs_beta),
        .switch_state (switch_state)
    );

    // -------------------------------------------------------------
    // 3. 100 MHz Master Clock Generation (10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // 4. Test Statistics & Helpers
    // -------------------------------------------------------------
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    localparam real Q20_SCALE = 1048576.0;

    // Watchdog Timer (safety against hang)
    initial begin
        #120_000_000; // 120 ms simulation time limit
        $display("\n[FATAL TIMEOUT] Simulation exceeded 120 ms!");
        $finish;
    end

    // Real-number conversion helper
    function real to_real;
        input signed [`DATA_WIDTH-1:0] val;
        begin
            to_real = $itor(val) / Q20_SCALE;
        end
    endfunction

    // -------------------------------------------------------------
    // 5. Verification Tasks
    // -------------------------------------------------------------
    // Check Vdc manager outputs against expected floating point values
    task check_vdc_outputs;
        input real exp_vdc;
        input [639:0] desc;

        real act_vdc, act_v1a, act_v2a, act_v2b;
        real exp_v1a, exp_v2a, exp_v2b;
        real err_vdc, err_v1a, err_v2a, err_v2b;
        reg pass;
    begin
        total_tests = total_tests + 1;

        act_vdc = to_real(vdc_q);
        act_v1a = to_real(v1_alpha_out);
        act_v2a = to_real(v2_alpha_out);
        act_v2b = to_real(v2_beta_out);

        exp_v1a = exp_vdc * (2.0 / 3.0);
        exp_v2a = exp_vdc * (1.0 / 3.0);
        exp_v2b = exp_vdc * (1.0 / 1.732050807568877);

        err_vdc = (act_vdc >= exp_vdc) ? (act_vdc - exp_vdc) : (exp_vdc - act_vdc);
        err_v1a = (act_v1a >= exp_v1a) ? (act_v1a - exp_v1a) : (exp_v1a - act_v1a);
        err_v2a = (act_v2a >= exp_v2a) ? (act_v2a - exp_v2a) : (exp_v2a - act_v2a);
        err_v2b = (act_v2b >= exp_v2b) ? (act_v2b - exp_v2b) : (exp_v2b - act_v2b);

        // Fixed-point tolerances: Vdc is integer (exact), projections within 0.1V
        pass = (err_vdc < 0.001) && (err_v1a < 0.1) && (err_v2a < 0.1) && (err_v2b < 0.15);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Vdc=%6.1fV | V1a=%6.2fV | V2a=%6.2fV | V2b=%6.2fV | %0s",
                     total_tests, act_vdc, act_v1a, act_v2a, act_v2b, desc);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] VDC PROJECTION MISMATCH! (%0s)", total_tests, desc);
            $display("           Expected: Vdc=%6.1fV, V1a=%6.2fV, V2a=%6.2fV, V2b=%6.2fV",
                     exp_vdc, exp_v1a, exp_v2a, exp_v2b);
            $display("           Actual  : Vdc=%6.1fV, V1a=%6.2fV, V2a=%6.2fV, V2b=%6.2fV",
                     act_vdc, act_v1a, act_v2a, act_v2b);
            $display("           Errors  : eVdc=%6.4f, eV1a=%6.4f, eV2a=%6.4f, eV2b=%6.4f",
                     err_vdc, err_v1a, err_v2a, err_v2b);
        end
    end
    endtask

    // Check specific voltage vector configuration
    task check_vector;
        input [2:0]  v_idx;
        input [2:0]  exp_sw;
        input real   exp_alpha;
        input real   exp_beta;
        input real   exp_mag;
        input real   exp_angle_deg;
        input [639:0] desc;

        real act_alpha, act_beta, act_mag;
        real diff_a, diff_b, diff_mag;
        reg pass;
    begin
        total_tests = total_tests + 1;
        vec_idx = v_idx;
        #1; // Combinational settling for voltage_lut

        act_alpha = to_real(vs_alpha);
        act_beta  = to_real(vs_beta);
        act_mag   = $sqrt(act_alpha * act_alpha + act_beta * act_beta);

        diff_a   = (act_alpha >= exp_alpha) ? (act_alpha - exp_alpha) : (exp_alpha - act_alpha);
        diff_b   = (act_beta  >= exp_beta)  ? (act_beta  - exp_beta)  : (exp_beta  - act_beta);
        diff_mag = (act_mag   >= exp_mag)   ? (act_mag   - exp_mag)   : (exp_mag   - act_mag);

        pass = (switch_state == exp_sw) && (diff_a < 0.1) && (diff_b < 0.1) && (diff_mag < 0.15);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Vector V%0d: Gates=%3b | Alpha=%7.2fV | Beta=%7.2fV | Mag=%7.2fV (%4.0f deg) | %0s",
                     total_tests, v_idx, switch_state, act_alpha, act_beta, act_mag, exp_angle_deg, desc);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] VECTOR CONFIGURATION MISMATCH! (%0s)", total_tests, desc);
            $display("           Expected: Gates=%3b, Alpha=%7.2fV, Beta=%7.2fV, Mag=%7.2fV",
                     exp_sw, exp_alpha, exp_beta, exp_mag);
            $display("           Actual  : Gates=%3b, Alpha=%7.2fV, Beta=%7.2fV, Mag=%7.2fV",
                     switch_state, act_alpha, act_beta, act_mag);
        end
    end
    endtask

    // Check all 8 switching configurations for current Vdc
    task test_all_8_vectors;
        input real vdc_val;
        input [639:0] header_desc;

        real v1a, v2a, v2b, v_mag;
    begin
        v1a   = vdc_val * (2.0 / 3.0);
        v2a   = vdc_val * (1.0 / 3.0);
        v2b   = vdc_val * (1.0 / 1.732050807568877);
        v_mag = vdc_val * (2.0 / 3.0);

        $display("\n--- Testing All 8 Switching Vectors for Vdc = %0.1f V (%0s) ---", vdc_val, header_desc);

        // V0: Zero Vector (000)
        check_vector(3'd0, 3'b000, 0.0, 0.0, 0.0, 0.0, "V0 Zero Vector (000)");

        // V1: Active Vector 1 (100) -> Angle 0 deg
        check_vector(3'd1, 3'b100, v1a, 0.0, v_mag, 0.0, "V1 Active Vector (100) @ 0 deg");

        // V2: Active Vector 2 (110) -> Angle +60 deg
        check_vector(3'd2, 3'b110, v2a, v2b, v_mag, 60.0, "V2 Active Vector (110) @ +60 deg");

        // V3: Active Vector 3 (010) -> Angle +120 deg
        check_vector(3'd3, 3'b010, -v2a, v2b, v_mag, 120.0, "V3 Active Vector (010) @ +120 deg");

        // V4: Active Vector 4 (011) -> Angle 180 deg
        check_vector(3'd4, 3'b011, -v1a, 0.0, v_mag, 180.0, "V4 Active Vector (011) @ 180 deg");

        // V5: Active Vector 5 (001) -> Angle 240 deg (-120 deg)
        check_vector(3'd5, 3'b001, -v2a, -v2b, v_mag, 240.0, "V5 Active Vector (001) @ 240 deg");

        // V6: Active Vector 6 (101) -> Angle 300 deg (-60 deg)
        check_vector(3'd6, 3'b101, v2a, -v2b, v_mag, 300.0, "V6 Active Vector (101) @ 300 deg");

        // V7: Zero Vector (111)
        check_vector(3'd7, 3'b111, 0.0, 0.0, 0.0, 0.0, "V7 Zero Vector (111)");
    end
    endtask

    // -------------------------------------------------------------
    // 6. Main Test Sequence
    // -------------------------------------------------------------
    integer cycle_idx;
    real norm_v1, norm_v2, norm_diff;

    initial begin
        $display("===================================================================");
        $display("       RIGOROUS VERIFICATION TESTBENCH: vdc_manager.v & voltage_lut ");
        $display("===================================================================");

        clk     = 1'b0;
        rst_n   = 1'b0;
        sw      = 8'h00;
        vec_idx = 3'd0;

        // =================================================================
        // PHASE 1: Reset Safety & Cycle 1-20 Startup Transient
        // =================================================================
        $display("\n--- PHASE 1: Reset Safety & Cycle 1-20 Startup Transient ---");

        // Hold reset for 50 ns
        repeat (5) @(posedge clk);
        total_tests = total_tests + 1;
        if (to_real(vdc_q) == 311.0 && vdc_valid == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Asynchronous reset active: vdc_q seeded safely to 311V default, valid=0", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Reset state error! vdc_q=%f, valid=%b", total_tests, to_real(vdc_q), vdc_valid);
        end

        // Release reset at clock edge
        @(posedge clk);
        rst_n <= 1'b1;
        $display("[%0t ns] Released rst_n. Monitoring startup FSM (Steps 0..7)...", $time);

        // Wait for vdc_valid pulse on Step 7
        @(posedge vdc_valid);
        total_tests = total_tests + 1;
        passed_tests = passed_tests + 1;
        $display("[PASS #%03d] Startup FSM completed: vdc_valid pulsed high on Step 7!", total_tests);

        // Verify initial 311V projections
        @(posedge clk);
        #1;
        check_vdc_outputs(311.0, "Startup Default 311V Calculations");

        // ADVERSARIAL RE-AUDIT CHECK: Cycle 9 to Cycle 25 Zeroing Hazard Check
        // In the unpatched code, on Cycle 9 (step returns to 0), line 70 saw
        // (vdc_integer != vdc_int_reg) before debounce finished and zeroed Vdc!
        $display("\n--- Checking Adversarial Cycle 9 Zeroing Hazard ---");
        for (cycle_idx = 9; cycle_idx <= 25; cycle_idx = cycle_idx + 1) begin
            @(posedge clk);
            #1;
            if (to_real(vdc_q) == 0.0) begin
                failed_tests = failed_tests + 1;
                $display("[FAIL] CRITICAL BUG DETECTED AT CYCLE %0d: vdc_q was zeroed out before debounce finished!", cycle_idx);
            end
        end
        total_tests = total_tests + 1;
        if (to_real(vdc_q) == 311.0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Cycles 9..25 Passed: vdc_q remained stable at 311V! No startup zeroing hazard.", total_tests);
        end

        // Test all 8 vectors for 311V startup
        test_all_8_vectors(311.0, "Nominal 311V Bench Test");

        // =================================================================
        // PHASE 2: Mechanical Contact Chatter & Glitch Rejection (< 10 ms)
        // =================================================================
        $display("\n--- PHASE 2: Mechanical Contact Chatter & Glitch Rejection (< 10 ms) ---");
        $display("Applying rapid switch contact chatter (bounces shorter than 10 ms debounce filter)...");

        // Chatter Burst 1: Switch flips to 0xFF for 1 ms (100,000 cycles) then returns to 0x00
        @(posedge clk);
        sw <= 8'hFF;
        repeat (100000) @(posedge clk);
        sw <= 8'h00;
        repeat (50000) @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        if (to_real(vdc_q) == 311.0 && vdc_valid == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] 1 ms chatter burst rejected! vdc_q remained 311V, no false trigger.", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Debouncer failed to reject 1 ms chatter! vdc_q=%f", total_tests, to_real(vdc_q));
        end

        // Chatter Burst 2: Switch flips to 0xD7 (541V) for 9.9 ms (990,000 cycles) then returns to 0x00
        @(posedge clk);
        sw <= 8'hD7;
        repeat (990000) @(posedge clk);
        sw <= 8'h00;
        repeat (50000) @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        if (to_real(vdc_q) == 311.0 && vdc_valid == 1'b0) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] 9.9 ms chatter burst rejected! Exactly under 10 ms threshold.", total_tests);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Debouncer prematurely accepted 9.9 ms chatter! vdc_q=%f", total_tests, to_real(vdc_q));
        end

        // =================================================================
        // PHASE 3: Valid Debounce & Voltage Step Transitions (> 10 ms)
        // =================================================================
        $display("\n--- PHASE 3: Valid Debounce & Voltage Step Transitions (> 10 ms) ---");

        // Transition 1: Set sw = 8'h01 (Fine step: 0*40 + 1*3 = 3V)
        $display("[%0t ns] Applying sw = 8'h01 (Fine +3V)... waiting for debounce & calculation...", $time);
        @(posedge clk);
        sw <= 8'h01;
        @(posedge vdc_valid);
        @(posedge clk);
        #1;
        check_vdc_outputs(3.0, "Fine Increment: 3V (sw=0x01)");

        // Transition 2: Set sw = 8'h00 (0V Standstill / Variac at 0V)
        $display("[%0t ns] Applying sw = 8'h00 (0V Standstill)... waiting for debounce & calculation...", $time);
        @(posedge clk);
        sw <= 8'h00;
        @(posedge vdc_valid);
        @(posedge clk);
        #1;
        check_vdc_outputs(0.0, "Zero Volts State (sw=0x00 -> 0V)");
        test_all_8_vectors(0.0, "0V Standstill / Variac at 0");

        // Transition 3: Set sw = 8'h10 (Coarse step: 1*40 + 0*3 = 40V)
        $display("[%0t ns] Applying sw = 8'h10 (Coarse +40V)... waiting for debounce & calculation...", $time);
        @(posedge clk);
        sw <= 8'h10;
        @(posedge vdc_valid);
        @(posedge clk);
        #1;
        check_vdc_outputs(40.0, "Coarse Increment: 40V (sw=0x10)");

        // Transition 4: Set sw = 8'hD7 (400V Motor Nominal Rectified DC Bus: 13*40 + 7*3 = 520 + 21 = 541V)
        $display("[%0t ns] Applying sw = 8'hD7 (400V Motor Operating Bus: 541V)... waiting for debounce & calculation...", $time);
        @(posedge clk);
        sw <= 8'hD7;
        @(posedge vdc_valid);
        @(posedge clk);
        #1;
        check_vdc_outputs(541.0, "400V Motor Nominal DC Bus (sw=0xD7 -> 541V)");
        test_all_8_vectors(541.0, "400V Motor Full DC Bus (541V)");

        // Transition 5: Set sw = 8'hFF (Maximum Switch Range: 15*40 + 15*3 = 600 + 45 = 645V)
        $display("[%0t ns] Applying sw = 8'hFF (Maximum Full Scale: 645V)... waiting for debounce & calculation...", $time);
        @(posedge clk);
        sw <= 8'hFF;
        @(posedge vdc_valid);
        @(posedge clk);
        #1;
        check_vdc_outputs(645.0, "Maximum DIP Switch Dynamic Range (sw=0xFF -> 645V)");
        test_all_8_vectors(645.0, "Maximum Over-Voltage Test (645V)");

        // =================================================================
        // PHASE 4: Geometric Orthogonality & Vector Norm Verification
        // =================================================================
        $display("\n--- PHASE 4: Geometric Orthogonality & Vector Norm Verification ---");
        // Verify |V1| == |V2| at 541V
        total_tests = total_tests + 1;
        norm_v1 = to_real(v1_alpha_out);
        norm_v2 = $sqrt(to_real(v2_alpha_out)*to_real(v2_alpha_out) + to_real(v2_beta_out)*to_real(v2_beta_out));
        norm_diff = (norm_v1 >= norm_v2) ? (norm_v1 - norm_v2) : (norm_v2 - norm_v1);

        if (norm_diff < 0.15) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] Hexagonal Symmetry: |V1|=%0.2fV, |V2|=%0.2fV (Diff=%0.4fV < 0.15V)",
                     total_tests, norm_v1, norm_v2, norm_diff);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] Vector length asymmetry! |V1|=%0.2fV, |V2|=%0.2fV",
                     total_tests, norm_v1, norm_v2);
        end

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("       VDC MANAGER & VOLTAGE LUT TESTBENCH EXECUTION SUMMARY       ");
        $display("===================================================================");
        $display(" Total Verification Tests Evaluated : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display("===================================================================");

        if (failed_tests == 0) begin
            $display(" >>> ALL VDC MANAGER & VOLTAGE LUT TESTS PASSED PERFECTLY! ZERO ERRORS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ARITHMETIC/TIMING ERRORS! <<< \n", failed_tests);
        end

        $finish;
    end

endmodule
