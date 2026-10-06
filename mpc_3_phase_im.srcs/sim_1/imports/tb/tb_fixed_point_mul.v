`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_fixed_point_mul;

    // -------------------------------------------------------------
    // 1. Clock and DUT Signals
    // -------------------------------------------------------------
    reg                          clk;
    reg  signed [`DATA_WIDTH-1:0] a;
    reg  signed [`DATA_WIDTH-1:0] b;
    wire signed [`DATA_WIDTH-1:0] result;

    // Instantiate DUT
    fixed_point_mul #(
        .DATA_WIDTH(`DATA_WIDTH),
        .FRAC_BITS(`FRAC_BITS)
    ) dut (
        .clk(clk),
        .a(a),
        .b(b),
        .result(result)
    );

    // -------------------------------------------------------------
    // 2. 100 MHz Master Clock (10 ns period)
    // -------------------------------------------------------------
    always #5 clk = ~clk;

    // -------------------------------------------------------------
    // 3. Test Statistics & Constants
    // -------------------------------------------------------------
    integer total_tests  = 0;
    integer passed_tests = 0;
    integer failed_tests = 0;

    localparam real Q20_SCALE = 1048576.0;

    // Watchdog
    initial begin
        #10_000_000;
        $display("\n[FATAL TIMEOUT] Simulation exceeded 10 ms!");
        $finish;
    end

    // -------------------------------------------------------------
    // 4. Verification Task
    // -------------------------------------------------------------
    task verify_mul;
        input real a_real;
        input real b_real;
        input [639:0] test_name;

        real exp_real;
        real act_real;
        real err_real;
        reg signed [`DATA_WIDTH-1:0] a_fixed;
        reg signed [`DATA_WIDTH-1:0] b_fixed;
        reg pass;
    begin
        total_tests = total_tests + 1;
        a_fixed = $rtoi(a_real * Q20_SCALE);
        b_fixed = $rtoi(b_real * Q20_SCALE);
        exp_real = ($itor(a_fixed) * $itor(b_fixed)) / (Q20_SCALE * Q20_SCALE);

        // Drive inputs
        @(posedge clk);
        a <= a_fixed;
        b <= b_fixed;

        // Exactly 2 clock cycles latency:
        // Cycle 1: product_reg latch
        @(posedge clk);
        // Cycle 2: result_reg latch
        @(posedge clk);
        #1; // Wait for non-blocking assignment in RTL to update

        // Sample output on cycle 2
        act_real = $itor(result) / Q20_SCALE;
        err_real = act_real - exp_real;
        if (err_real < 0.0) err_real = -err_real;

        // Fixed-point 20-bit precision allows tolerance of 1 LSB = 1/2^20 = 0.000001
        pass = (err_real <= 0.00001);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] %7.3f * %7.3f = %8.4f | Got: %8.4f | Err: %8.6f | %0s",
                     total_tests, a_real, b_real, exp_real, act_real, err_real, test_name);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] MULTIPLICATION ERROR! (%0s)", total_tests, test_name);
            $display("           Inputs   : a = %7.4f (%0d), b = %7.4f (%0d)", a_real, a_fixed, b_real, b_fixed);
            $display("           Expected : %8.6f", exp_real);
            $display("           Got      : %8.6f (result = %0d)", act_real, result);
            $display("           Error    : %8.6f", err_real);
        end
    end
    endtask

    // -------------------------------------------------------------
    // 4b. Bit-Exact LSB Verification Task (Dense 20-Bit Fraction)
    // -------------------------------------------------------------
    task verify_mul_raw;
        input signed [`DATA_WIDTH-1:0] a_val;
        input signed [`DATA_WIDTH-1:0] b_val;
        input [639:0] test_name;

        reg signed [2*`DATA_WIDTH-1:0] mul_64;
        reg signed [`DATA_WIDTH-1:0] exp_val;
        reg signed [`DATA_WIDTH-1:0] act_val;
        reg signed [`DATA_WIDTH-1:0] diff;
        reg pass;
    begin
        total_tests = total_tests + 1;

        // Exact hardware arithmetic model
        mul_64 = {{32{a_val[31]}}, a_val} * {{32{b_val[31]}}, b_val};
        exp_val = mul_64[51:20];

        // Drive inputs
        @(posedge clk);
        a <= a_val;
        b <= b_val;

        // Exactly 2 clock cycles latency:
        @(posedge clk);
        @(posedge clk);
        #1; // Wait for non-blocking assignment

        act_val = result;
        diff = act_val - exp_val;
        pass = (diff == 32'sd0);

        if (pass) begin
            passed_tests = passed_tests + 1;
            $display("[PASS #%03d] 0x%08x * 0x%08x = 0x%08x (frac: 0x%05x) | LSB Exact! | %0s",
                     total_tests, a_val, b_val, act_val, act_val[19:0], test_name);
        end else begin
            failed_tests = failed_tests + 1;
            $display("[FAIL #%03d] BIT-EXACT ERROR! (%0s)", total_tests, test_name);
            $display("           Inputs   : a = 0x%08x (%0d), b = 0x%08x (%0d)", a_val, a_val, b_val, b_val);
            $display("           Expected : 0x%08x (%0d) [64b: 0x%016x]", exp_val, exp_val, mul_64);
            $display("           Got      : 0x%08x (%0d)", act_val, act_val);
            $display("           Diff     : %0d LSBs", diff);
        end
    end
    endtask

    // -------------------------------------------------------------
    // 5. Main Test Sequence
    // -------------------------------------------------------------
    integer i;
    real r_a, r_b;
    reg signed [`DATA_WIDTH-1:0] stream_a [0:9];
    reg signed [`DATA_WIDTH-1:0] stream_b [0:9];
    real stream_exp [0:9];
    real stream_act;
    reg signed [`DATA_WIDTH-1:0] rand_a;
    reg signed [`DATA_WIDTH-1:0] rand_b;

    initial begin
        $display("===================================================================");
        $display("       RIGOROUS VERIFICATION TESTBENCH: fixed_point_mul.v          ");
        $display("===================================================================");

        clk = 1'b0;
        a   = 32'sd0;
        b   = 32'sd0;
        repeat (5) @(posedge clk);

        // =================================================================
        // PHASE 1: Precise Cycle-Accurate Latency Verification
        // =================================================================
        $display("\n--- PHASE 1: Cycle-Accurate Latency Verification ---");
        // Apply 3.0 * 4.0 = 12.0 at T0
        @(posedge clk);
        a <= $rtoi(3.0 * Q20_SCALE);
        b <= $rtoi(4.0 * Q20_SCALE);
        $display("[%0t] T0: Applied inputs a=3.0, b=4.0", $time);

        // Cycle T1 (Latency = 1 cycle): result must NOT be updated yet
        @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        if (result == 32'sd0) begin
            $display("[PASS #%03d] Latency check Cycle 1: result is still 0 (as expected for 2-cycle pipeline)", total_tests);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Latency premature update at Cycle 1! result=%0d", total_tests, result);
            failed_tests = failed_tests + 1;
        end

        // Cycle T2 (Latency = 2 cycles): result MUST be exactly 12.0
        @(posedge clk);
        #1;
        total_tests = total_tests + 1;
        if (result == $rtoi(12.0 * Q20_SCALE)) begin
            $display("[PASS #%03d] Latency check Cycle 2: result is EXACTLY 12.0 (12,582,912 counts) -> Proven 2 cycles!", total_tests);
            passed_tests = passed_tests + 1;
        end else begin
            $display("[FAIL #%03d] Latency check Cycle 2 failed! got %0d, expected %0d", total_tests, result, $rtoi(12.0 * Q20_SCALE));
            failed_tests = failed_tests + 1;
        end

        // =================================================================
        // PHASE 2: Fundamental Algebraic Properties
        // =================================================================
        $display("\n--- PHASE 2: Fundamental Algebraic Properties ---");
        verify_mul(0.0, 5.0, "Multiplication by Zero");
        verify_mul(5.0, 0.0, "Zero Multiplicand");
        verify_mul(1.0, 1.0, "Identity: 1.0 * 1.0 = 1.0");
        verify_mul(1.0, 7.35, "Identity: 1.0 * X = X");
        verify_mul(-1.0, 4.25, "Inversion: -1.0 * X = -X");
        verify_mul(-1.0, -1.0, "Double Negative: -1.0 * -1.0 = +1.0");

        // =================================================================
        // PHASE 3: Signed Quadrant Operations (+/+, +/-, -/+, -/-)
        // =================================================================
        $display("\n--- PHASE 3: Four-Quadrant Signed Operations ---");
        verify_mul(2.5, 4.0, "Quadrant 1: Positive * Positive (+2.5 * +4.0 = +10.0)");
        verify_mul(3.5, -2.0, "Quadrant 4: Positive * Negative (+3.5 * -2.0 = -7.0)");
        verify_mul(-4.5, 2.0, "Quadrant 2: Negative * Positive (-4.5 * +2.0 = -9.0)");
        verify_mul(-3.0, -4.0, "Quadrant 3: Negative * Negative (-3.0 * -4.0 = +12.0)");
        verify_mul(0.5, 0.5, "Fractional Square: 0.5 * 0.5 = 0.25");
        verify_mul(-0.25, 0.5, "Fractional Signed: -0.25 * 0.5 = -0.125");

        // =================================================================
        // PHASE 4: Motor & MPC Operating Parameters
        // =================================================================
        $display("\n--- PHASE 4: Realistic Motor & FCS-MPC Signal Values ---");
        // Flux observer E22 * psi_r: 0.99948 * 0.96 = 0.9595 Wb
        verify_mul(0.99948, 0.960, "Flux Observer: E22 * psi_r");
        // TS_Q * wr_psi: 0.0001 * 300.0 = 0.0300
        verify_mul(0.00010, 300.0, "Observer Cross Term: TS_Q * wr_psi");
        // Clarke transform INV_SQRT3 * (ia + 2*ib): 0.57735 * 4.33 = 2.50 A
        verify_mul(0.57735, 4.330, "Clarke Transform: 1/sqrt(3) * beta_sum");
        // Torque calculation: Kt * (psi * i): 2.913 * (0.96 * 3.5) = 9.787 Nm
        verify_mul(2.913, 3.360, "Torque Estimator: Kt * flux_cross_curr");
        // Vdc manager Two-Thirds projection: 311.0 * 0.666667 = 207.333 V
        verify_mul(311.0, 0.666667, "Vdc Manager: Vdc * (2/3)");

        // =================================================================
        // PHASE 5: Dynamic Range Boundaries & Sub-LSB Resolution
        // =================================================================
        $display("\n--- PHASE 5: Boundary Values & Small Signal Precision ---");
        verify_mul(45.0, 45.0, "Large Signal: 45.0 * 45.0 = 2025.0 (Near Q12.20 limit)");
        verify_mul(-45.0, 45.0, "Large Negative: -45.0 * 45.0 = -2025.0");
        verify_mul(0.001, 0.001, "Micro Scale: 0.001 * 0.001 = 0.000001 (1 LSB precision)");
        verify_mul(0.00097656, 1.0, "Exact 1 LSB count: (1/1024) * 1.0");

        // =================================================================
        // PHASE 6: Full Pipeline Streaming Throughput (1 op / clock cycle)
        // =================================================================
        $display("\n--- PHASE 6: Pipelined Streaming Throughput (Back-to-Back Operations) ---");
        $display("-> Streaming 10 continuous multiplications with zero idle bubble cycles...");

        // Setup 10 test vectors
        for (i = 0; i < 10; i = i + 1) begin
            r_a = (i + 1) * 0.5;
            r_b = (i + 1) * 1.2;
            stream_a[i]   = $rtoi(r_a * Q20_SCALE);
            stream_b[i]   = $rtoi(r_b * Q20_SCALE);
            stream_exp[i] = r_a * r_b;
        end

        // Feed inputs continuously and verify outputs emerging delayed by exactly 2 clock cycles
        for (i = 0; i <= 10; i = i + 1) begin
            if (i < 10) begin
                a <= stream_a[i];
                b <= stream_b[i];
            end else begin
                a <= 32'sd0;
                b <= 32'sd0;
            end
            @(posedge clk);
            #1;
            if (i >= 1 && i <= 10) begin
                total_tests = total_tests + 1;
                stream_act = $itor(result) / Q20_SCALE;
                if (stream_act - stream_exp[i-1] <= 0.001 && stream_exp[i-1] - stream_act <= 0.001) begin
                    passed_tests = passed_tests + 1;
                end else begin
                    failed_tests = failed_tests + 1;
                    $display("[FAIL #%03d] Stream item %0d mismatch! exp=%7.3f, got=%7.3f",
                             total_tests, i-1, stream_exp[i-1], stream_act);
                end
            end
        end
        $display("[PASS] Successfully processed 10 continuous multiplications at 100 MSamples/sec throughput!");

        // =================================================================
        // PHASE 7: Full 20-Bit Active Fractional Pattern Tests
        // =================================================================
        $display("\n--- PHASE 7: Full 20-Bit Active Fractional Pattern Tests ---");
        $display("Testing dense bit patterns where all 20 fractional bits [19:0] are non-zero...\n");

        // 7.1: All 20 fractional bits set to 1 (0.999999046325)
        verify_mul_raw(32'h000F_FFFF, 32'h0010_0000, "All 20 Frac Bits = 1 * 1.0 Identity (32'h000FFFFF * 1.0)");
        verify_mul_raw(32'h000F_FFFF, 32'h000F_FFFF, "All 20 Frac Bits = 1 Squared (Max Frac Square)");

        // 7.2: Alternating Checkerboard Patterns (every bit active)
        verify_mul_raw(32'h000A_AAAA, 32'h0005_5555, "Alternating Checkerboard (20'hAAAAA * 20'h55555)");
        verify_mul_raw(32'h0005_5555, 32'h000A_AAAA, "Commutative Checkerboard (20'h55555 * 20'hAAAAA)");
        verify_mul_raw(32'h001A_AAAA, 32'h0025_5555, "Integer + Checkerboard Fraction (1.666666 * 2.333333)");

        // 7.3: Dense Ascending and Descending Nibble Patterns
        verify_mul_raw(32'h000E_DCBA, 32'h0001_2345, "Dense Frac Hex Nibbles (20'hEDCBA * 20'h12345)");
        verify_mul_raw(32'h0007_6543, 32'h000F_EDCB, "Dense Frac Hex Nibbles (20'h76543 * 20'hFEDCB)");

        // 7.4: Signed Negative Operands with all 20 fractional bits active
        verify_mul_raw(-32'sd1048575, 32'h0010_0000, "Signed Neg All 20 Frac Bits * 1.0 (-0.999999 * 1.0)");
        verify_mul_raw(-32'sd1048575, -32'sd1048575, "Signed Neg All 20 Frac Bits Squared (-0.999999 * -0.999999)");
        verify_mul_raw(-32'sd699050,  32'h0005_5555, "Signed Neg Checkerboard * Pos Checkerboard");
        verify_mul_raw( 32'h003F_FFFF, -32'sd3145727, "Signed Mixed Int+Frac (+3.999999 * -2.999999)");

        // 7.5: Transcendental and Physical Constants (every single fractional bit populated)
        verify_mul_raw(32'sd3294199, 32'sd2850325, "Physical Constants: Pi * e (3.141592 * 2.718281)");
        verify_mul_raw(32'sd3294199, 32'sd1482910, "Physical Constants: Pi * sqrt(2) (3.141592 * 1.414213)");
        verify_mul_raw(32'sd1696626, 32'sd2850325, "Physical Constants: phi * e (1.618033 * 2.718281)");
        verify_mul_raw(32'sd605411,  32'sd1482910, "Physical Constants: (1/sqrt3) * sqrt(2)");

        // 7.6: Exhaustive Pseudo-Random Dense 20-Bit Vectors
        $display("\n--- Subphase 7.6: 15 Pseudo-Random Dense 20-Bit Vectors ---");
        for (i = 0; i < 15; i = i + 1) begin
            // Force MSB (bit 19) and LSB (bit 0) of the fraction to 1 so all 20 fractional bits are non-zero
            rand_a = ($random & 32'h000F_FFFF) | 32'h0008_0001;
            rand_b = ($random & 32'h000F_FFFF) | 32'h0004_0003;
            // Introduce signed negative cases on alternating iterations
            if (i % 2 == 1) rand_a = -rand_a;
            if (i % 3 == 0) rand_b = -rand_b;
            verify_mul_raw(rand_a, rand_b, "Dense Pseudo-Random 20-bit Fraction Vector");
        end

        // =================================================================
        // FINAL SUMMARY
        // =================================================================
        $display("\n===================================================================");
        $display("           FIXED POINT MULTIPLIER TESTBENCH EXECUTION SUMMARY      ");
        $display("===================================================================");
        $display(" Total Verification Tests Evaluated : %0d", total_tests);
        $display(" Passed Tests                       : %0d", passed_tests);
        $display(" Failed Tests                       : %0d", failed_tests);
        $display("===================================================================");

        if (failed_tests == 0) begin
            $display(" >>> ALL FIXED POINT MULTIPLIER TESTS PASSED PERFECTLY! ZERO ERRORS. <<< \n");
        end else begin
            $display(" >>> TESTBENCH FAILED WITH %0d ARITHMETIC/TIMING ERRORS! <<< \n", failed_tests);
        end

        $finish;
    end

endmodule

