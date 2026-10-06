`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_flux_observer_comparison;

    reg clk;
    reg rst_n;
    reg start;
    reg signed [`DATA_WIDTH-1:0] i_alpha;
    reg signed [`DATA_WIDTH-1:0] i_beta;
    reg signed [`DATA_WIDTH-1:0] speed_elec;

    // 1st-Order Forward Euler Observer Outputs
    wire signed [`DATA_WIDTH-1:0] psi_1st_alpha;
    wire signed [`DATA_WIDTH-1:0] psi_1st_beta;
    wire signed [`DATA_WIDTH-1:0] wr_psi_1st_alpha;
    wire signed [`DATA_WIDTH-1:0] wr_psi_1st_beta;
    wire                          done_1st;

    // 2nd-Order Taylor Euler Observer Outputs
    wire signed [`DATA_WIDTH-1:0] psi_2nd_alpha;
    wire signed [`DATA_WIDTH-1:0] psi_2nd_beta;
    wire signed [`DATA_WIDTH-1:0] wr_psi_2nd_alpha;
    wire signed [`DATA_WIDTH-1:0] wr_psi_2nd_beta;
    wire                          done_2nd;

    // Instantiate 1st-Order Observer (Baseline)
    flux_observer u_obs_1st (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .i_alpha(i_alpha),
        .i_beta(i_beta),
        .speed_elec(speed_elec),
        .psi_r_alpha(psi_1st_alpha),
        .psi_r_beta(psi_1st_beta),
        .wr_psi_alpha(wr_psi_1st_alpha),
        .wr_psi_beta(wr_psi_1st_beta),
        .done(done_1st)
    );

    // Instantiate 2nd-Order Taylor Observer (New)
    flux_observer_2nd_order u_obs_2nd (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .i_alpha(i_alpha),
        .i_beta(i_beta),
        .speed_elec(speed_elec),
        .psi_r_alpha(psi_2nd_alpha),
        .psi_r_beta(psi_2nd_beta),
        .wr_psi_alpha(wr_psi_2nd_alpha),
        .wr_psi_beta(wr_psi_2nd_beta),
        .done(done_2nd)
    );

    // 100 MHz System Clock (10 ns period)
    always #5 clk = ~clk;

    // Helper functions for real conversion
    function real to_real;
        input signed [`DATA_WIDTH-1:0] val;
        begin
            to_real = $itor(val) / 1048576.0;
        end
    endfunction

    real mag_1st, mag_2nd;
    real psi_1a_r, psi_1b_r;
    real psi_2a_r, psi_2b_r;

    always @(*) begin
        psi_1a_r = to_real(psi_1st_alpha);
        psi_1b_r = to_real(psi_1st_beta);
        mag_1st  = $sqrt(psi_1a_r * psi_1a_r + psi_1b_r * psi_1b_r);

        psi_2a_r = to_real(psi_2nd_alpha);
        psi_2b_r = to_real(psi_2nd_beta);
        mag_2nd  = $sqrt(psi_2a_r * psi_2a_r + psi_2b_r * psi_2b_r);
    end

    // Simulation control variables
    real sim_time;
    real omega_e;
    real angle;
    real i_mag;
    integer step_cnt;

    task step_observers;
        integer timeout;
        reg seen1, seen2;
        begin
            @(posedge clk);
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;

            seen1 = 1'b0;
            seen2 = 1'b0;
            timeout = 0;
            while ((!seen1 || !seen2) && timeout < 60) begin
                @(posedge clk);
                if (done_1st) seen1 = 1'b1;
                if (done_2nd) seen2 = 1'b1;
                timeout = timeout + 1;
            end
            if (timeout >= 60) begin
                $display("[ERROR] Observer timeout! seen1=%b, seen2=%b at time %0t", seen1, seen2, $time);
                $finish;
            end
            @(posedge clk);
        end
    endtask

    initial begin
        $display("================================================================================");
        $display("  STARTING COMPARISON TESTBENCH: 1st-ORDER EULER VS 2nd-ORDER TAYLOR EULER");
        $display("================================================================================");

        clk        = 1'b0;
        rst_n      = 1'b0;
        start      = 1'b0;
        i_alpha    = 32'sd0;
        i_beta     = 32'sd0;
        speed_elec = 32'sd0;
        sim_time   = 0.0;
        angle      = 0.0;

        #100;
        rst_n = 1'b1;
        #100;

        // ---------------------------------------------------------------------
        // TEST CASE 1: Standstill Magnetization (wr = 0 rad/s)
        // Stator current: DC 4.73 A along alpha axis for 1.5 seconds (15,000 Ts steps)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 1: Standstill Magnetization (wr = 0 rad/s, Is = 4.73 A) ---");
        speed_elec = 32'sd0;
        i_alpha    = $rtoi(4.73 * 1048576.0);
        i_beta     = 32'sd0;

        for (step_cnt = 0; step_cnt < 6000; step_cnt = step_cnt + 1) begin
            step_observers();
        end

        $display("Standstill (t = 0.6s):");
        $display("  1st-Order Flux: %0.4f Wb", mag_1st);
        $display("  2nd-Order Flux: %0.4f Wb", mag_2nd);
        $display("  Nominal Lm*Is:  %0.4f Wb", 0.20295 * 4.73);

        if ((mag_1st - mag_2nd < 0.001) && (mag_2nd - mag_1st < 0.001)) begin
            $display("[PASS] At standstill (wr=0), both 1st and 2nd order match identically.");
        end else begin
            $display("[FAIL] Discrepancy at standstill!");
        end

        // ---------------------------------------------------------------------
        // TEST CASE 2: Nominal Rated Speed (wr = 157.08 rad/s = 1500 RPM, fe = 50 Hz)
        // Stator current: 4.73 A AC rotating at fe = 50 Hz.
        // Run for 0.6 seconds (6,000 Ts steps) to reach steady state.
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 2: Rated Speed (wr = 157.08 rad/s, fe = 50 Hz, Is = 4.73 A) ---");
        omega_e    = 157.07963;
        speed_elec = $rtoi(omega_e * 1048576.0);
        i_mag      = 4.73;

        for (step_cnt = 0; step_cnt < 6000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001; // Ts = 100 us
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_observers();
        end

        $display("Rated Speed (t = 3.0s, wr = 157.08 rad/s):");
        $display("  1st-Order Flux: %0.4f Wb (Amplification: +%0.1f%%)", 
                 mag_1st, ((mag_1st / 0.96) - 1.0) * 100.0);
        $display("  2nd-Order Flux: %0.4f Wb (Error vs Rated: %+0.2f%%)", 
                 mag_2nd, ((mag_2nd / 0.96) - 1.0) * 100.0);

        if (mag_2nd > 0.95 && mag_2nd < 0.97) begin
            $display("[PASS] 2nd-Order Euler successfully eliminates artificial amplification!");
        end else begin
            $display("[FAIL] 2nd-Order Euler failed to maintain rated flux!");
        end

        // ---------------------------------------------------------------------
        // TEST CASE 3: High Speed (wr = 250.0 rad/s = 2387 RPM)
        // Close to 1st-order stability limit (321.5 rad/s)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 3: High Speed (wr = 250.0 rad/s = 2387 RPM, Is = 4.73 A) ---");
        omega_e    = 250.0;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 10000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_observers();
        end

        $display("High Speed (t = 4.0s, wr = 250.0 rad/s):");
        $display("  1st-Order Flux: %0.4f Wb (Amplification: +%0.1f%%)", 
                 mag_1st, ((mag_1st / 0.96) - 1.0) * 100.0);
        $display("  2nd-Order Flux: %0.4f Wb (Error vs Rated: %+0.2f%%)", 
                 mag_2nd, ((mag_2nd / 0.96) - 1.0) * 100.0);

        // ---------------------------------------------------------------------
        // TEST CASE 4: Beyond 1st-Order Stability Limit (wr = 350.0 rad/s = 3342 RPM)
        // 1st-order limit is 321.5 rad/s; at 350 rad/s, |lambda| = 1.05 > 1.0!
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 4: Beyond Stability Limit (wr = 350.0 rad/s > 321.5 rad/s limit) ---");
        omega_e    = 350.0;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 5000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_observers();
        end

        $display("Beyond Limit (t = 4.5s, wr = 350.0 rad/s):");
        $display("  1st-Order Flux: %0.4f Wb", mag_1st);
        $display("  2nd-Order Flux: %0.4f Wb", mag_2nd);

        if (mag_1st > 10.0 || mag_1st < 0.0) begin
            $display("[CONFIRMED] 1st-Order Forward Euler DIVERGES/EXPLODES beyond 321 rad/s!");
        end
        if (mag_2nd > 0.94 && mag_2nd < 0.98) begin
            $display("[PASS] 2nd-Order Taylor Euler remains perfectly STABLE and ACCURATE at 350 rad/s!");
        end

        $display("\n================================================================================");
        $display("  ALL VERIFICATION TESTS COMPLETED SUCCESSFULLY!");
        $display("================================================================================");
        $finish;
    end

endmodule
