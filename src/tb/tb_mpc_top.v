`timescale 1ns / 1ps

module tb_mpc_top;

    `include "mpc_params.vh"

    
    
    reg sys_clk_i;
    initial sys_clk_i = 1'b0;
    always #5 sys_clk_i = ~sys_clk_i;

    
    
    reg         cpu_resetn;
    reg         btnu, btnc, btnd;
    wire        adc_cs_n, adc_sclk;
    reg         adc_d0, adc_d1;
    reg         enc_a, enc_b, enc_z;
    wire        gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl;
    wire        inverter_en;
    wire [7:0]  led;
    reg  [7:0]  sw;

    
    
    mpc_top uut (
        .sys_clk_i   (sys_clk_i),
        .cpu_resetn  (cpu_resetn),
        .btnu        (btnu),
        .btnc        (btnc),
        .btnd        (btnd),
        .adc_cs_n    (adc_cs_n),
        .adc_d0      (adc_d0),
        .adc_d1      (adc_d1),
        .adc_sclk    (adc_sclk),
        .enc_a       (enc_a),
        .enc_b       (enc_b),
        .enc_z       (enc_z),
        .gate_ah     (gate_ah),
        .gate_al     (gate_al),
        .gate_bh     (gate_bh),
        .gate_bl     (gate_bl),
        .gate_ch     (gate_ch),
        .gate_cl     (gate_cl),
        .inverter_en (inverter_en),
        .led         (led),
        .sw          (sw)
    );

    

    reg [11:0] adc_ch0_value; 
    reg [11:0] adc_ch1_value; 
    reg [15:0] adc_shift_ch0, adc_shift_ch1;
    integer    adc_bit_cnt;

    always @(negedge adc_cs_n) begin
        
        adc_shift_ch0 = {4'b0000, adc_ch0_value};
        adc_shift_ch1 = {4'b0000, adc_ch1_value};
        adc_bit_cnt   = 0;
        
        adc_d0 = adc_shift_ch0[15];
        adc_d1 = adc_shift_ch1[15];
    end

    always @(negedge adc_sclk) begin
        if (!adc_cs_n) begin
            adc_bit_cnt = adc_bit_cnt + 1;
            if (adc_bit_cnt < 16) begin
                adc_shift_ch0 = adc_shift_ch0 << 1;
                adc_shift_ch1 = adc_shift_ch1 << 1;
                adc_d0 = adc_shift_ch0[15];
                adc_d1 = adc_shift_ch1[15];
            end
        end
    end

    always @(posedge adc_cs_n) begin
        adc_d0 = 1'bz; 
        adc_d1 = 1'bz;
    end

    

    real encoder_speed_rpm;      
    integer enc_half_period_ns;  
    integer enc_pulse_count;

    task generate_encoder_pulses;
        input real speed_rpm;
        input integer num_revolutions;
        integer total_pulses;
        integer i;
        integer half_period;
    begin
        
        total_pulses = num_revolutions * ENCODER_PPR;

        if (speed_rpm > 0) begin
            half_period = 1_000_000_000 / (2 * ENCODER_PPR) * 60;
            half_period = half_period / speed_rpm;
        end else begin
            half_period = 1_000_000; 
        end

        for (i = 0; i < total_pulses; i = i + 1) begin

            enc_a = 1'b0; enc_b = 1'b0; #(half_period);
            enc_a = 1'b0; enc_b = 1'b1; #(half_period);
            enc_a = 1'b1; enc_b = 1'b1; #(half_period);
            enc_a = 1'b1; enc_b = 1'b0; #(half_period);
        end
    end
    endtask

    

    always @(posedge sys_clk_i) begin
        if (gate_ah && gate_al) begin
            $display("ERROR [%0t]: SHOOT-THROUGH on Phase A! gate_ah=%b, gate_al=%b",
                     $time, gate_ah, gate_al);
            $stop;
        end
        if (gate_bh && gate_bl) begin
            $display("ERROR [%0t]: SHOOT-THROUGH on Phase B! gate_bh=%b, gate_bl=%b",
                     $time, gate_bh, gate_bl);
            $stop;
        end
        if (gate_ch && gate_cl) begin
            $display("ERROR [%0t]: SHOOT-THROUGH on Phase C! gate_ch=%b, gate_cl=%b",
                     $time, gate_ch, gate_cl);
            $stop;
        end
    end

    
    
    reg [2:0] prev_gate_state;
    wire [2:0] cur_gate_state = {gate_ah, gate_bh, gate_ch};
    integer gate_change_count;

    always @(posedge sys_clk_i) begin
        if (cur_gate_state !== prev_gate_state) begin
            $display("[%0t] Gate change: Sa=%b Sb=%b Sc=%b",
                     $time, gate_ah, gate_bh, gate_ch);
            prev_gate_state <= cur_gate_state;
            gate_change_count <= gate_change_count + 1;
        end
    end

    
    
    initial begin
        
        cpu_resetn  = 1'b1;
        btnu        = 1'b0;
        btnc        = 1'b0;
        btnd        = 1'b0;
        adc_d0      = 1'bz;
        adc_d1      = 1'bz;
        enc_a       = 1'b0;
        enc_b       = 1'b0;
        enc_z       = 1'b0;
        prev_gate_state = 3'b000;
        gate_change_count = 0;

        sw = 8'b1100_0101;

        adc_ch0_value = 12'd2048;
        adc_ch1_value = 12'd2048;
        
        #10;
        cpu_resetn  = 1'b0;

        $display("=== MPC Top-Level Testbench ===");
        $display("[%0t] Asserting reset...", $time);

        #1000;
        cpu_resetn = 1'b1;
        $display("[%0t] Reset released.", $time);

        #5000;

        
        
        $display("\n--- Test 1: Disabled state ---");
        #200_000; 
        if (gate_ah == 0 && gate_al == 0 && gate_bh == 0 &&
            gate_bl == 0 && gate_ch == 0 && gate_cl == 0)
            $display("PASS: All gates OFF when disabled.");
        else
            $display("FAIL: Gates active when disabled!");

        
        
        $display("\n--- Test 2: Enable MPC (zero current, zero speed) ---");
        btnu = 1'b1; 
        $display("[%0t] Inverter ENABLED.", $time);

        #1_000_000;
        $display("[%0t] After 1ms: gate changes = %0d", $time, gate_change_count);

        
        
        $display("\n--- Test 3: Apply 5A on phase A ---");
        
        adc_ch0_value = 12'd2560; 
        adc_ch1_value = 12'd2048; 

        #500_000;
        $display("[%0t] Gate changes after current injection = %0d", $time, gate_change_count);

        
        
        $display("\n--- Test 4: Encoder at 900 RPM ---");
        fork
            generate_encoder_pulses(900.0, 1); 
        join_none

        #2_000_000;
        $display("[%0t] Motor running, gate changes = %0d", $time, gate_change_count);

        
        
        $display("\n--- Test 5: Disable inverter ---");
        btnu = 1'b0;
        #100_000;
        if (gate_ah == 0 && gate_al == 0 && gate_bh == 0 &&
            gate_bl == 0 && gate_ch == 0 && gate_cl == 0)
            $display("PASS: All gates OFF after disable.");
        else
            $display("FAIL: Gates still active after disable!");

        $display("\n=== Simulation Complete ===");
        $display("Total gate state changes: %0d", gate_change_count);
        $finish;
    end

    
    
    initial begin
        #50_000_000; 
        $display("TIMEOUT: Simulation exceeded 50 ms.");
        $finish;
    end

    
    
    initial begin
        $dumpfile("mpc_top_tb.vcd");
        $dumpvars(0, tb_mpc_top);
    end

endmodule
