`timescale 1ns / 1ps
// =============================================================================
// Testbench: tb_oled_test
// Description: Comprehensive testbench verifying:
//              1. OLED SSD1306 initialization sequence.
//              2. Quadrature encoder forward decoding and single-rev wrapping (0..4095).
//              3. Cumulative revolution counting (+1 on wrap 4095->0).
//              4. Sliding window moving average RPM calculation.
//              5. Reverse rotation decoding, wrapping (0->4095), and negative rev counting.
//              6. Z-index pulse detection and visual stretching.
//              7. Center pushbutton (BTNC) manual tare / zero reset.
//              8. Real-time OLED text buffer generation and character readout.
// =============================================================================

module tb_oled_test;

    reg  sys_clk;
    reg  cpu_resetn;
    reg  btnc;

    reg  enc_a;
    reg  enc_b;
    reg  enc_z;

    wire oled_sclk;
    wire oled_sdin;
    wire oled_dc;
    wire oled_res;
    wire oled_vbat;
    wire oled_vdd;
    wire [7:0] led;

    // Instantiate with SIMULATION = 1 (fast timers for simulation)
    oled_test_top #(
        .CPR        (4096),
        .SIMULATION (1'b1)
    ) dut (
        .sys_clk_i  (sys_clk),
        .cpu_resetn (cpu_resetn),
        .btnc       (btnc),
        .enc_a      (enc_a),
        .enc_b      (enc_b),
        .enc_z      (enc_z),
        .oled_sclk  (oled_sclk),
        .oled_sdin  (oled_sdin),
        .oled_dc    (oled_dc),
        .oled_res   (oled_res),
        .oled_vbat  (oled_vbat),
        .oled_vdd   (oled_vdd),
        .led        (led)
    );

    // 100 MHz system clock (10 ns period)
    always #5 sys_clk = ~sys_clk;

    integer passes = 0;
    integer failures = 0;

    task check(input [255:0] name, input condition);
        begin
            if (condition) begin
                $display("  [PASS] %0s", name);
                passes = passes + 1;
            end else begin
                $display("  [FAIL] %0s", name);
                failures = failures + 1;
            end
        end
    endtask

    // Task to generate N forward quadrature pulses
    task step_forward;
        input integer count;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1) begin
                case ({enc_a, enc_b})
                    2'b00: {enc_a, enc_b} = 2'b01;
                    2'b01: {enc_a, enc_b} = 2'b11;
                    2'b11: {enc_a, enc_b} = 2'b10;
                    2'b10: {enc_a, enc_b} = 2'b00;
                    default: {enc_a, enc_b} = 2'b00;
                endcase
                #40; // 40 ns between edges
            end
        end
    endtask

    // Task to generate N reverse quadrature pulses
    task step_reverse;
        input integer count;
        integer k;
        begin
            for (k = 0; k < count; k = k + 1) begin
                case ({enc_a, enc_b})
                    2'b00: {enc_a, enc_b} = 2'b10;
                    2'b10: {enc_a, enc_b} = 2'b11;
                    2'b11: {enc_a, enc_b} = 2'b01;
                    2'b01: {enc_a, enc_b} = 2'b00;
                    default: {enc_a, enc_b} = 2'b00;
                endcase
                #40; // 40 ns between edges
            end
        end
    endtask

    integer i;

    initial begin
        $display("=================================================================");
        $display("STARTING ENCODER + OLED STANDALONE VERIFICATION TESTBENCH");
        $display("=================================================================");

        sys_clk    = 0;
        cpu_resetn = 0;
        btnc       = 0;
        enc_a      = 0;
        enc_b      = 0;
        enc_z      = 0;

        // Reset for 100 ns
        #100;
        cpu_resetn = 1;
        #100;

        // ---------------------------------------------------------------------
        // TEST 1: OLED Initialization & Power-On Sequencing
        // ---------------------------------------------------------------------
        $display("\n--- TEST 1: OLED Display Power-Up Sequence ---");
        check("1.1 VDD power active-low", oled_vdd == 1'b0);
        wait (oled_res == 1'b1);
        check("1.2 OLED reset line deasserted high", oled_res == 1'b1);
        wait (dut.u_oled.init_done == 1'b1);
        @(posedge sys_clk);
        #1;
        check("1.3 OLED initialization complete (init_done = 1)", dut.u_oled.init_done == 1'b1);
        check("1.4 Panel power VBAT enabled (active-low = 0)", oled_vbat == 1'b0);
        check("1.5 Diagnostic LED1 ON (Display Ready)", led[1] == 1'b1);

        // ---------------------------------------------------------------------
        // TEST 2: Standstill Baseline Verification
        // ---------------------------------------------------------------------
        $display("\n--- TEST 2: Standstill Initial Conditions ---");
        check("2.1 Position within rev is 0", dut.u_enc_tester.pos_in_rev == 12'd0);
        check("2.2 Revolution count is 0", dut.u_enc_tester.rev_count == 16'sd0);
        check("2.3 Direction is STOP (0)", dut.u_enc_tester.dir_state == 2'b00);
        check("2.4 RPM is 0", dut.u_enc_tester.rpm == 32'sd0);

        // ---------------------------------------------------------------------
        // TEST 3: Forward Rotation & Single-Revolution Wrapping (0 -> 4095 -> 0)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 3: Forward Quadrature Pulses & Revolution Wrap ---");
        // Generate 100 forward pulses
        step_forward(100);
        #200;
        check("3.1 Position counted 100 pulses forward", dut.u_enc_tester.pos_in_rev == 12'd100);
        check("3.2 Motion active flag asserted", dut.u_enc_tester.motion_active == 1'b1);
        check("3.3 LD2 (FWD LED) is active", led[2] == 1'b1);

        // Advance to count 4095 (4095 - 100 = 3995 more pulses)
        step_forward(3995);
        #200;
        check("3.4 Position reached 4095 (one pulse before wrap)", dut.u_enc_tester.pos_in_rev == 12'd4095);
        check("3.5 Rev count is still 0 before wrap", dut.u_enc_tester.rev_count == 16'sd0);

        // 1 more pulse causes wrap to 0 and rev_count increment
        step_forward(1);
        #200;
        check("3.6 Position wrapped 4095 -> 0 cleanly", dut.u_enc_tester.pos_in_rev == 12'd0);
        check("3.7 Rev count incremented to +1", dut.u_enc_tester.rev_count == 16'sd1);

        // Step 50 more pulses into rev 1
        step_forward(50);
        #200;
        check("3.8 Position is 50 in rev 1", dut.u_enc_tester.pos_in_rev == 12'd50);
        check("3.9 Rev count remains +1", dut.u_enc_tester.rev_count == 16'sd1);

        // ---------------------------------------------------------------------
        // TEST 4: Reverse Rotation & Reverse Wrapping (50 -> 0 -> 4095)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 4: Reverse Quadrature Pulses & Underflow Wrap ---");
        // Step 50 pulses reverse to reach 0
        step_reverse(50);
        #200;
        check("4.1 Position back to 0", dut.u_enc_tester.pos_in_rev == 12'd0);
        check("4.2 Rev count still +1 at boundary", dut.u_enc_tester.rev_count == 16'sd1);

        // 1 more reverse pulse wraps to 4095 and decrements rev_count back to 0
        step_reverse(1);
        #200;
        check("4.3 Position wrapped 0 -> 4095 on reverse", dut.u_enc_tester.pos_in_rev == 12'd4095);
        check("4.4 Rev count decremented back to 0", dut.u_enc_tester.rev_count == 16'sd0);

        // ---------------------------------------------------------------------
        // TEST 5: Z-Index Channel Pulse Detection & Stretcher
        // ---------------------------------------------------------------------
        $display("\n--- TEST 5: Z-Index Hardware Pulse ---");
        enc_z = 1'b1;
        #100;
        enc_z = 1'b0;
        #200;
        check("5.1 Z-active indicator asserted (stretched pulse)", dut.u_enc_tester.z_active == 1'b1);
        check("5.2 Z-count incremented to 1", dut.u_enc_tester.z_count == 16'd1);
        check("5.3 LD5 (Z LED) active", led[5] == 1'b1);

        // ---------------------------------------------------------------------
        // TEST 6: Continuous Forward Rotation & Sliding Window RPM Measurement
        // ---------------------------------------------------------------------
        $display("\n--- TEST 6: Continuous Spinning & Sliding Window RPM ---");
        // Emulate forward spinning at ~1440 RPM:
        // In simulation with SUB_WIN_CYCLES = 250 cycles, spin for 8 sub-windows
        for (i = 0; i < 8; i = i + 1) begin
            step_forward(100);
            #1000;
        end
        @(posedge sys_clk);
        #200;
        check("6.1 Measured RPM is positive during spinning", dut.u_enc_tester.rpm > 0);
        check("6.2 Direction is confirmed FWD", dut.u_enc_tester.dir_state == 2'b01);

        // ---------------------------------------------------------------------
        // TEST 7: Manual Zero/Tare Pushbutton (BTNC)
        // ---------------------------------------------------------------------
        $display("\n--- TEST 7: Manual Tare Button (BTNC) ---");
        check("7.1 Non-zero position before tare", dut.u_enc_tester.pos_in_rev > 0);
        // Press BTNC for 100 clock cycles (longer than debounce)
        btnc = 1'b1;
        repeat (100) @(posedge sys_clk);
        btnc = 1'b0;
        repeat (50) @(posedge sys_clk);
        #100;
        check("7.2 Position cleared to 0 by BTNC", dut.u_enc_tester.pos_in_rev == 12'd0);
        check("7.3 Rev count cleared to 0 by BTNC", dut.u_enc_tester.rev_count == 16'sd0);

        // ---------------------------------------------------------------------
        // TEST 8: OLED Text Formatter Verification
        // ---------------------------------------------------------------------
        $display("\n--- TEST 8: Real-Time OLED Text Buffer Content ---");
        // Wait for next formatter cycle (250 cycles in simulation)
        #3000;
        @(posedge sys_clk);

        // Check Line 0: " ENCODER TESTER "
        check("8.1 Line 0 Title byte 1 is 'E'", dut.oled_chars[1] == "E");
        check("8.2 Line 0 Title byte 2 is 'N'", dut.oled_chars[2] == "N");
        check("8.3 Line 0 Title byte 3 is 'C'", dut.oled_chars[3] == "C");

        // Check Line 1: RPM header
        check("8.4 Line 1 Header byte 16 is 'R'", dut.oled_chars[16] == "R");
        check("8.5 Line 1 Header byte 17 is 'P'", dut.oled_chars[17] == "P");
        check("8.6 Line 1 Header byte 18 is 'M'", dut.oled_chars[18] == "M");

        // Check Line 2: Position header "POS: 00000/4096 "
        check("8.7 Line 2 Header byte 32 is 'P'", dut.oled_chars[32] == "P");
        check("8.8 Line 2 Position count is '0'", dut.oled_chars[41] == "0");
        check("8.9 Line 2 Denominator is '/'",   dut.oled_chars[42] == "/");
        check("8.10 Line 2 Max CPR is '4'",     dut.oled_chars[43] == "4");

        // Check Line 3: Revolution header "REV: +00000  Z:0"
        check("8.11 Line 3 Header byte 48 is 'R'", dut.oled_chars[48] == "R");
        check("8.12 Line 3 Header byte 49 is 'E'", dut.oled_chars[49] == "E");
        check("8.13 Line 3 Header byte 50 is 'V'", dut.oled_chars[50] == "V");

        // ---------------------------------------------------------------------
        // VERIFICATION SUMMARY
        // ---------------------------------------------------------------------
        #1000;
        $display("=================================================================");
        $display("ENCODER STANDALONE SIMULATION SUMMARY: Passes = %0d, Failures = %0d", passes, failures);
        $display("=================================================================");
        if (failures == 0)
            $display(">>> ALL 28 ENCODER & OLED STANDALONE TESTS PASSED SUCCESSFULLY! <<<");
        else
            $display(">>> TEST FAILURES DETECTED! <<<");

        $finish;
    end

endmodule
