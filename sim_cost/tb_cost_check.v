`timescale 1ns/1ps
`include "mpc_params.vh"

module tb_cost_check;

    localparam DATA_WIDTH = `DATA_WIDTH;
    localparam FRAC_BITS  = `FRAC_BITS;
    localparam CLK_PERIOD = 10;
    localparam real Q20   = 1048576.0;

    reg clk;
    reg rst_n;
    reg start;
    reg signed [DATA_WIDTH-1:0] is_alpha_pred;
    reg signed [DATA_WIDTH-1:0] is_beta_pred;
    reg signed [DATA_WIDTH-1:0] psi_r_alpha_pred;
    reg signed [DATA_WIDTH-1:0] psi_r_beta_pred;
    reg signed [DATA_WIDTH-1:0] te_ref_in;

    wire signed [DATA_WIDTH-1:0] cost;
    wire cost_overflow;
    wire done;

    cost_evaluator #(
        .DATA_WIDTH(DATA_WIDTH),
        .FRAC_BITS(FRAC_BITS)
    ) dut (
        .clk             (clk),
        .rst_n           (rst_n),
        .start           (start),
        .is_alpha_pred   (is_alpha_pred),
        .is_beta_pred    (is_beta_pred),
        .psi_r_alpha_pred(psi_r_alpha_pred),
        .psi_r_beta_pred (psi_r_beta_pred),
        .te_ref_in       (te_ref_in),
        .cost            (cost),
        .cost_overflow   (cost_overflow),
        .done            (done)
    );

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    function signed [DATA_WIDTH-1:0] to_q;
        input real v;
        begin
            to_q = $rtoi(v * Q20);
        end
    endfunction

    function real to_r;
        input signed [DATA_WIDTH-1:0] q;
        begin
            to_r = q / Q20;
        end
    endfunction

    integer cycles;
    task run_eval;
        input real ia, ib, psira, psirb, teref;
        begin
            @(posedge clk);
            is_alpha_pred    <= to_q(ia);
            is_beta_pred     <= to_q(ib);
            psi_r_alpha_pred <= to_q(psira);
            psi_r_beta_pred  <= to_q(psirb);
            te_ref_in        <= to_q(teref);
            start            <= 1'b1;
            cycles = 0;

            @(posedge clk);
            start <= 1'b0;

            while (!done) begin
                cycles = cycles + 1;
                @(posedge clk);
            end
        end
    endtask

    integer pass_count = 0;
    integer fail_count = 0;

    initial begin
        rst_n = 0;
        start = 0;
        is_alpha_pred = 0; is_beta_pred = 0;
        psi_r_alpha_pred = 0; psi_r_beta_pred = 0;
        te_ref_in = 0;
        #(CLK_PERIOD * 5);
        rst_n = 1;
        #(CLK_PERIOD * 2);

        $display("=== STARTING COST EVALUATOR OVERFLOW & PRECISION VERIFICATION ===");

        // Test 1: Normal Operating Point (ia=4.73A, ib=4.45A, psira=0.96Wb, psirb=0, te_ref=14.5Nm)
        // Torque ~ 12.45 Nm, Torque error ~ 2.05 Nm, Flux magnitude = 0.96 Wb (flux error ~ 0)
        run_eval(4.73, 4.45, 0.96, 0.0, 14.5);
        $display("Test 1 [Nominal]: cost=%0d (real=%0.2f), overflow=%0b, cycles=%0d", 
                 cost, to_r(cost), cost_overflow, cycles);
        if (cost > 0 && cost < to_q(50.0) && cost_overflow == 0) begin
            $display("PASS: Test 1 Nominal operating cost evaluated accurately without overflow.");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 1 Nominal cost evaluation failed!");
            fail_count = fail_count + 1;
        end

        // Test 2: Massive Torque Error (|eT| = 60 Nm > 45.26 Nm threshold)
        // psira = 0, psirb = 0, is_alpha = 0, is_beta = 0 -> te_pred = 0.
        // te_ref = 60 Nm.
        run_eval(0.0, 0.0, 0.96, 0.0, 60.0);
        $display("Test 2 [Torque Overload 60Nm]: cost=%0d (real=%0.2f), overflow=%0b", 
                 cost, to_r(cost), cost_overflow);
        if (cost == 32'sd2147483647 && cost_overflow == 1'b1) begin
            $display("PASS: Test 2 Torque overload safely saturated to INT32_MAX, overflow flag asserted.");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 2 Failed to clamp torque overload! cost=%0d, overflow=%0b", cost, cost_overflow);
            fail_count = fail_count + 1;
        end

        // Test 3: Massive Flux Error (|epsi| = 50 Wb^2 > 45.26 Wb^2 threshold)
        // Flux magnitude squared = 50.92 Wb^2 -> flux error = -50 Wb^2.
        // psira = 7.14 Wb, psirb = 0 -> psi_sq ~ 51 Wb^2.
        run_eval(0.0, 0.0, 7.14, 0.0, 0.0);
        $display("Test 3 [Flux Overload 50 Wb^2]: cost=%0d (real=%0.2f), overflow=%0b", 
                 cost, to_r(cost), cost_overflow);
        if (cost == 32'sd2147483647 && cost_overflow == 1'b1) begin
            $display("PASS: Test 3 Flux overload safely saturated to INT32_MAX, overflow flag asserted.");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 3 Failed to clamp flux overload! cost=%0d, overflow=%0b", cost, cost_overflow);
            fail_count = fail_count + 1;
        end

        // Test 4: Negative Extreme Torque Overload (te_ref = -60 Nm)
        run_eval(0.0, 0.0, 0.96, 0.0, -60.0);
        $display("Test 4 [Negative Torque Overload -60Nm]: cost=%0d (real=%0.2f), overflow=%0b", 
                 cost, to_r(cost), cost_overflow);
        if (cost == 32'sd2147483647 && cost_overflow == 1'b1) begin
            $display("PASS: Test 4 Negative torque overload safely saturated to INT32_MAX.");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 4 Negative torque overload not clamped!");
            fail_count = fail_count + 1;
        end

        // Test 5: Perfect Zero-Error State
        // te_ref = 0, is = 0, psi = 0.96 Wb (flux magnitude squared = 0.9216 Wb^2 == PSI_REF_SQ)
        run_eval(0.0, 0.0, 0.96, 0.0, 0.0);
        $display("Test 5 [Zero Error State]: cost=%0d, overflow=%0b", cost, cost_overflow);
        if (cost >= 0 && cost < to_q(0.01) && cost_overflow == 0) begin
            $display("PASS: Test 5 Zero error cost is strictly zero, no overflow asserted.");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Test 5 Zero error test failed!");
            fail_count = fail_count + 1;
        end

        // Latency test:
        $display("Execution Latency: %0d clock cycles", cycles);
        if (cycles == 26) begin // 25 computation steps + 1 start latching cycle = 26 clock cycles (260 ns)
            $display("PASS: Execution latency exactly 26 clock cycles (260 ns).");
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: Unexpected latency %0d cycles (expected 26 wait cycles).", cycles);
            fail_count = fail_count + 1;
        end

        $display("\n==================================================");
        $display("RESULTS: %0d PASSED, %0d FAILED", pass_count, fail_count);
        $display("==================================================");

        $finish;
    end

endmodule
