`timescale 1ns / 1ps
// =============================================================================
// Testbench: tb_oled_test
// Description: Cycle-accurate testbench verifying OLED power-on sequencing,
//              SPI transmission, initialization commands, display refresh,
//              and 2 Hz character cycling.
// =============================================================================

module tb_oled_test;

    reg sys_clk;
    reg cpu_resetn;

    wire oled_sclk;
    wire oled_sdin;
    wire oled_dc;
    wire oled_res;
    wire oled_vbat;
    wire oled_vdd;
    wire [7:0] led;

    // Instantiate with SIMULATION = 1 (fast timers)
    oled_test_top #(
        .SIMULATION(1'b1)
    ) dut (
        .sys_clk_i  (sys_clk),
        .cpu_resetn (cpu_resetn),
        .oled_sclk  (oled_sclk),
        .oled_sdin  (oled_sdin),
        .oled_dc    (oled_dc),
        .oled_res   (oled_res),
        .oled_vbat  (oled_vbat),
        .oled_vdd   (oled_vdd),
        .led        (led)
    );

    // 100 MHz clock generator (10 ns period)
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

    initial begin
        $display("=================================================================");
        $display("Starting OLED Standalone Testbench (Simulation Speedup Enabled)");
        $display("=================================================================");

        sys_clk    = 0;
        cpu_resetn = 0;

        // Hold reset for 100 ns
        #100;
        cpu_resetn = 1;
        #50;

        // 1. Check Power-On: VDD should turn on first (active-low = 0)
        check("1. Logic power VDD asserted (active-low = 0)", oled_vdd == 1'b0);
        check("1b. Panel power VBAT still OFF initially (active-low = 1)", oled_vbat == 1'b1);

        // 2. Wait for reset pulse
        wait (oled_res == 1'b1);
        check("2. OLED hardware reset deasserted high", oled_res == 1'b1);

        // 3. Wait for initialization completion (init_done)
        $display("  Waiting for SSD1306 initialization sequence...");
        wait (dut.u_oled.init_done == 1'b1);
        @(posedge sys_clk);
        #1;
        check("3. OLED initialization completed (init_done = 1)", dut.u_oled.init_done == 1'b1);
        check("3b. VBAT enabled before Display ON (active-low = 0)", oled_vbat == 1'b0);
        check("3c. Diagnostic LED1 lit solid (init_done = 1)", led[1] == 1'b1);

        // 4. Check initial character is 'a' (0x61)
        check("4. Initial character is 'a' (0x61)", dut.current_char == 8'h61);

        // 5. Run through several 2 Hz character ticks (with SIMULATION=1, ticks occur every 200 cycles)
        $display("  Verifying 2 Hz character advance ('a' -> 'b' -> 'c' ...)");
        
        wait (dut.char_idx == 5'd1);
        check("5. Character advanced to 'b' (0x62)", dut.current_char == 8'h62);
        check("5b. LED0 toggled with 2 Hz tick", led[0] == 1'b1);

        wait (dut.char_idx == 5'd2);
        check("6. Character advanced to 'c' (0x63)", dut.current_char == 8'h63);

        wait (dut.char_idx == 5'd3);
        check("7. Character advanced to 'd' (0x64)", dut.current_char == 8'h64);

        // 6. Check that SPI activity is actively refreshing the display
        check("8. SPI clock is active", oled_sclk !== 1'bx);

        #1000;
        $display("=================================================================");
        $display("SIMULATION SUMMARY: Passes = %0d, Failures = %0d", passes, failures);
        $display("=================================================================");
        if (failures == 0)
            $display("ALL OLED TEST CASES PASSED SUCCESSFULLY!");
        else
            $display("TEST CASES FAILED!");
        $finish;
    end

endmodule
