`timescale 1ns / 1ps

`include "mpc_params.vh"

module tb_flux_observer_3way_comparison;

    reg clk;
    reg rst_n;
    reg start;
    reg signed [`DATA_WIDTH-1:0] i_alpha;
    reg signed [`DATA_WIDTH-1:0] i_beta;
    reg signed [`DATA_WIDTH-1:0] speed_elec;

    // 1st-Order Forward Euler Outputs
    wire signed [`DATA_WIDTH-1:0] psi_1st_alpha, psi_1st_beta;
    wire signed [`DATA_WIDTH-1:0] wr_psi_1st_alpha, wr_psi_1st_beta;
    wire                          done_1st;

    // 2nd-Order Taylor Euler Outputs
    wire signed [`DATA_WIDTH-1:0] psi_2nd_alpha, psi_2nd_beta;
    wire signed [`DATA_WIDTH-1:0] wr_psi_2nd_alpha, wr_psi_2nd_beta;
    wire                          done_2nd;

    // Exact Matrix Exponential Outputs
    wire signed [`DATA_WIDTH-1:0] psi_ext_alpha, psi_ext_beta;
    wire signed [`DATA_WIDTH-1:0] wr_psi_ext_alpha, wr_psi_ext_beta;
    wire                          done_ext;

    // Module Instantiations
    flux_observer u_1st (
        .clk(clk), .rst_n(rst_n), .start(start),
        .i_alpha(i_alpha), .i_beta(i_beta), .speed_elec(speed_elec),
        .psi_r_alpha(psi_1st_alpha), .psi_r_beta(psi_1st_beta),
        .wr_psi_alpha(wr_psi_1st_alpha), .wr_psi_beta(wr_psi_1st_beta),
        .done(done_1st)
    );

    flux_observer_2nd_order u_2nd (
        .clk(clk), .rst_n(rst_n), .start(start),
        .i_alpha(i_alpha), .i_beta(i_beta), .speed_elec(speed_elec),
        .psi_r_alpha(psi_2nd_alpha), .psi_r_beta(psi_2nd_beta),
        .wr_psi_alpha(wr_psi_2nd_alpha), .wr_psi_beta(wr_psi_2nd_beta),
        .done(done_2nd)
    );

    flux_observer_exact u_exact (
        .clk(clk), .rst_n(rst_n), .start(start),
        .i_alpha(i_alpha), .i_beta(i_beta), .speed_elec(speed_elec),
        .psi_r_alpha(psi_ext_alpha), .psi_r_beta(psi_ext_beta),
        .wr_psi_alpha(wr_psi_ext_alpha), .wr_psi_beta(wr_psi_ext_beta),
        .done(done_ext)
    );

    // 100 MHz System Clock
    always #5 clk = ~clk;

    function real to_real;
        input signed [`DATA_WIDTH-1:0] val;
        begin
            to_real = $itor(val) / 1048576.0;
        end
    endfunction

    real mag_1st, mag_2nd, mag_ext;
    always @(*) begin
        mag_1st = $sqrt(to_real(psi_1st_alpha)**2 + to_real(psi_1st_beta)**2);
        mag_2nd = $sqrt(to_real(psi_2nd_alpha)**2 + to_real(psi_2nd_beta)**2);
        mag_ext = $sqrt(to_real(psi_ext_alpha)**2 + to_real(psi_ext_beta)**2);
    end

    // Step Execution Task with latching for all three observers
    task step_all;
        integer timeout;
        reg seen1, seen2, seen3;
        begin
            @(posedge clk);
            start <= 1'b1;
            @(posedge clk);
            start <= 1'b0;

            seen1 = 1'b0;
            seen2 = 1'b0;
            seen3 = 1'b0;
            timeout = 0;
            while ((!seen1 || !seen2 || !seen3) && timeout < 70) begin
                @(posedge clk);
                if (done_1st) seen1 = 1'b1;
                if (done_2nd) seen2 = 1'b1;
                if (done_ext) seen3 = 1'b1;
                timeout = timeout + 1;
            end
            if (timeout >= 70) begin
                $display("[ERROR] Timeout! seen1=%b, seen2=%b, seen3=%b at %0t", seen1, seen2, seen3, $time);
                $finish;
            end
            @(posedge clk);
        end
    endtask

    real omega_e, angle, i_mag;
    integer step_cnt;

    initial begin
        $display("================================================================================");
        $display("  3-WAY OBSERVER COMPARISON: 1st-ORDER vs 2nd-ORDER TAYLOR vs EXACT MATRIX EXP");
        $display("================================================================================");

        clk        = 1'b0;
        rst_n      = 1'b0;
        start      = 1'b0;
        i_alpha    = 32'sd0;
        i_beta     = 32'sd0;
        speed_elec = 32'sd0;
        angle      = 0.0;
        i_mag      = 4.73;

        #100;
        rst_n = 1'b1;
        #100;

        // ---------------------------------------------------------------------
        // TEST CASE 1: Standstill Magnetization (wr = 0 rad/s)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 1: Standstill Magnetization (wr = 0 rad/s, Is = 4.73 A) ---");
        speed_elec = 32'sd0;
        i_alpha    = $rtoi(4.73 * 1048576.0);
        i_beta     = 32'sd0;

        for (step_cnt = 0; step_cnt < 6000; step_cnt = step_cnt + 1) begin
            step_all();
        end

        $display("Standstill (t = 0.6s):");
        $display("  1st-Order Euler: %0.4f Wb", mag_1st);
        $display("  2nd-Order Euler: %0.4f Wb", mag_2nd);
        $display("  Exact MatrixExp: %0.4f Wb", mag_ext);
        $display("  Nominal Lm*Is:   %0.4f Wb", 0.20295 * 4.73);

        // ---------------------------------------------------------------------
        // TEST CASE 2: Nominal Rated Speed (wr = 157.08 rad/s = 1500 RPM, fe = 50 Hz)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 2: Rated Speed (wr = 157.08 rad/s, fe = 50 Hz, Is = 4.73 A) ---");
        omega_e    = 157.07963;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 6000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_all();
        end

        $display("Rated Speed (t = 1.2s, wr = 157.08 rad/s):");
        $display("  1st-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_1st, ((mag_1st/0.96) - 1.0)*100.0);
        $display("  2nd-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_2nd, ((mag_2nd/0.96) - 1.0)*100.0);
        $display("  Exact MatrixExp: %0.4f Wb (Error: %+0.2f%%)", mag_ext, ((mag_ext/0.96) - 1.0)*100.0);
        $display("  DEBUG EXACT: phi_d=%0d, phi_o=%0d, cos_poly=%0d, sin_poly=%0d, theta=%0d", 
                 u_exact.phi_diag, u_exact.phi_off, u_exact.cos_poly, u_exact.sin_poly, u_exact.theta);

        // ---------------------------------------------------------------------
        // TEST CASE 3: High Speed (wr = 250.0 rad/s = 2387 RPM)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 3: High Speed (wr = 250.0 rad/s = 2387 RPM, Is = 4.73 A) ---");
        omega_e    = 250.0;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 6000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_all();
        end

        $display("High Speed (t = 1.8s, wr = 250.0 rad/s):");
        $display("  1st-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_1st, ((mag_1st/0.96) - 1.0)*100.0);
        $display("  2nd-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_2nd, ((mag_2nd/0.96) - 1.0)*100.0);
        $display("  Exact MatrixExp: %0.4f Wb (Error: %+0.2f%%)", mag_ext, ((mag_ext/0.96) - 1.0)*100.0);

        // ---------------------------------------------------------------------
        // TEST CASE 4: Beyond 1st-Order Stability Limit (wr = 350.0 rad/s = 3342 RPM)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 4: Beyond Stability Limit (wr = 350.0 rad/s > 321.5 rad/s) ---");
        omega_e    = 350.0;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 5000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_all();
        end

        $display("Beyond Limit (t = 2.3s, wr = 350.0 rad/s):");
        $display("  1st-Order Euler: %0.4f Wb (Divergent/Saturated)", mag_1st);
        $display("  2nd-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_2nd, ((mag_2nd/0.96) - 1.0)*100.0);
        $display("  Exact MatrixExp: %0.4f Wb (Error: %+0.2f%%)", mag_ext, ((mag_ext/0.96) - 1.0)*100.0);

        // ---------------------------------------------------------------------
        // TEST CASE 5: Reverse Rated Speed (wr = -157.08 rad/s = -1500 RPM)
        // ---------------------------------------------------------------------
        $display("\n--- TEST CASE 5: Reverse Rated Speed (wr = -157.08 rad/s, Is = 4.73 A) ---");
        omega_e    = -157.07963;
        speed_elec = $rtoi(omega_e * 1048576.0);

        for (step_cnt = 0; step_cnt < 12000; step_cnt = step_cnt + 1) begin
            angle = angle + omega_e * 0.0001;
            i_alpha = $rtoi(i_mag * $cos(angle) * 1048576.0);
            i_beta  = $rtoi(i_mag * $sin(angle) * 1048576.0);
            step_all();
            if (step_cnt == 5999) begin
                $display("Reverse Speed at t = 0.6s (mid-transient):");
                $display("  1st-Order Euler: %0.4f Wb", mag_1st);
                $display("  2nd-Order Euler: %0.4f Wb", mag_2nd);
                $display("  Exact MatrixExp: %0.4f Wb (settling in progress)", mag_ext);
            end
        end

        $display("Reverse Speed at t = 1.2s (fully settled steady-state):");
        $display("  1st-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_1st, ((mag_1st/0.96) - 1.0)*100.0);
        $display("  2nd-Order Euler: %0.4f Wb (Error: %+0.2f%%)", mag_2nd, ((mag_2nd/0.96) - 1.0)*100.0);
        $display("  Exact MatrixExp: %0.4f Wb (Error: %+0.2f%%)", mag_ext, ((mag_ext/0.96) - 1.0)*100.0);
        $display("  DEBUG EXACT: phi_d=%0d, phi_o=%0d, cos_poly=%0d, sin_poly=%0d, theta=%0d", 
                 u_exact.phi_diag, u_exact.phi_off, u_exact.cos_poly, u_exact.sin_poly, u_exact.theta);

        $display("\n================================================================================");
        $display("  3-WAY COMPARISON TESTBENCH COMPLETED SUCCESSFULLY!");
        $display("================================================================================");
        $finish;
    end

endmodule
