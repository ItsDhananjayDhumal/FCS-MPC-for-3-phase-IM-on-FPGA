`timescale 1ns / 1ps
// =============================================================================
// Module: encoder_tester
// Description: Industrial Quadrature Optical Encoder Reader & Diagnostic Engine.
//              - Tailored for Autonics E50S8G5-1024Z-G24F (1024 PPR / 4096 CPR).
//              - 3-stage metastability synchronizers on A, B, Z matching main project.
//              - Single revolution position counter: wraps 0..4095 (0 to CPR-1).
//              - Revolution counter: signed rev count incremented/decremented on wrap.
//              - Sliding window moving average RPM estimator (200 ms window).
//              - Direction of rotation indicator (FWD, REV, STOP).
//              - Hardware Z-index pulse detection with 50 ms visual stretcher.
// =============================================================================

module encoder_tester #(
    parameter CPR        = 4096,   // Counts per revolution (1024 PPR * 4)
    parameter SIMULATION = 1'b0
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   enc_a,
    input  wire                   enc_b,
    input  wire                   enc_z,
    input  wire                   zero_btn,       // Tare button: resets pos & revs to 0

    output reg  signed [31:0]     rpm,            // Moving average RPM
    output reg  [1:0]             dir_state,      // 0=STOP, 1=FWD, 2=REV
    output reg  [11:0]            pos_in_rev,     // Position within revolution (0..CPR-1)
    output reg  signed [15:0]     rev_count,      // Cumulative revolution count
    output wire                   z_active,       // 50 ms stretched Z-pulse indicator
    output reg  [15:0]            z_count,        // Cumulative Z pulse count
    output wire                   motion_active,  // High when shaft is turning
    output wire                   a_sync_out,     // Synchronized A line for LED
    output wire                   b_sync_out      // Synchronized B line for LED
);

    // -------------------------------------------------------------------------
    // 1. Triple-Stage Metastability Synchronizers (Identical to main project)
    // -------------------------------------------------------------------------
    reg [2:0] a_sync, b_sync, z_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 3'b000;
            b_sync <= 3'b000;
            z_sync <= 3'b000;
        end else begin
            a_sync <= {a_sync[1:0], enc_a};
            b_sync <= {b_sync[1:0], enc_b};
            z_sync <= {z_sync[1:0], enc_z};
        end
    end

    wire a_in   = a_sync[1];
    wire b_in   = b_sync[1];
    wire z_in   = z_sync[1];
    wire a_prev = a_sync[2];
    wire b_prev = b_sync[2];
    wire z_prev = z_sync[2];
    wire z_rise = z_in && !z_prev;

    assign a_sync_out = a_in;
    assign b_sync_out = b_in;

    // -------------------------------------------------------------------------
    // 2. Quadrature Transition Decoder (Identical to main project)
    // -------------------------------------------------------------------------
    wire [3:0] quad_state = {a_prev, b_prev, a_in, b_in};
    reg  [1:0] move; // 0=None, 1=Forward (+), 2=Reverse (-)

    always @(*) begin
        case (quad_state)
            4'b0001, 4'b0111, 4'b1110, 4'b1000: move = 2'd1; // Forward (+1 count)
            4'b0010, 4'b1011, 4'b1101, 4'b0100: move = 2'd2; // Reverse (-1 count)
            default: move = 2'd0;
        endcase
    end

    // -------------------------------------------------------------------------
    // 3. Single-Revolution Position Counter & Revolution Counter
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_in_rev <= 12'd0;
            rev_count  <= 16'sd0;
            z_count    <= 16'd0;
        end else if (zero_btn) begin
            // Manual tare: reset position and revolution count
            pos_in_rev <= 12'd0;
            rev_count  <= 16'sd0;
        end else begin
            if (z_rise) begin
                z_count <= z_count + 16'd1;
            end

            if (move == 2'd1) begin
                // Forward movement
                if (pos_in_rev >= CPR - 1) begin
                    pos_in_rev <= 12'd0;
                    rev_count  <= rev_count + 16'sd1;
                end else begin
                    pos_in_rev <= pos_in_rev + 12'd1;
                end
            end else if (move == 2'd2) begin
                // Reverse movement
                if (pos_in_rev == 12'd0) begin
                    pos_in_rev <= CPR - 1;
                    rev_count  <= rev_count - 16'sd1;
                end else begin
                    pos_in_rev <= pos_in_rev - 12'd1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // 4. Z-Index Pulse Stretcher (50 ms visual hold for LED and Display)
    // -------------------------------------------------------------------------
    localparam Z_STRETCH_MAX = SIMULATION ? 23'd500 : 23'd5_000_000; // 50 ms at 100 MHz
    reg [22:0] z_stretch_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            z_stretch_cnt <= 23'd0;
        end else if (z_rise) begin
            z_stretch_cnt <= Z_STRETCH_MAX;
        end else if (z_stretch_cnt > 0) begin
            z_stretch_cnt <= z_stretch_cnt - 23'd1;
        end
    end

    assign z_active = (z_stretch_cnt > 0) || z_in;

    // -------------------------------------------------------------------------
    // 5. Motion Activity Watchdog (100 ms timeout)
    // -------------------------------------------------------------------------
    localparam MOTION_TIMEOUT_MAX = SIMULATION ? 24'd20_000 : 24'd10_000_000; // 100 ms
    reg [23:0] motion_timeout_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            motion_timeout_cnt <= 24'd0;
        end else if (move != 2'd0) begin
            motion_timeout_cnt <= MOTION_TIMEOUT_MAX;
        end else if (motion_timeout_cnt > 0) begin
            motion_timeout_cnt <= motion_timeout_cnt - 24'd1;
        end
    end

    assign motion_active = (motion_timeout_cnt > 0);

    // -------------------------------------------------------------------------
    // 6. Sliding Window Moving Average RPM Estimator
    //    - 8 sub-windows of 25 ms each = 200 ms total window
    //    - Window advance rate: 40 Hz (every 25 ms)
    //    - Formula: RPM = (N_200ms * 60) / (CPR * 0.2s) = (N_200ms * 75) / 1024
    // -------------------------------------------------------------------------
    localparam SUB_WIN_CYCLES = SIMULATION ? 22'd250 : 22'd2_500_000; // 25 ms at 100 MHz

    reg [21:0]        sub_win_timer;
    reg signed [17:0] sub_delta_accum;
    reg signed [17:0] delta_hist [0:7];
    reg [2:0]         hist_ptr;

    // Clean calculation state machine
    reg [2:0]         calc_step;
    reg signed [20:0] win_sum;
    reg signed [31:0] rpm_prod;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_win_timer   <= 22'd0;
            sub_delta_accum <= 18'sd0;
            delta_hist[0]   <= 18'sd0;
            delta_hist[1]   <= 18'sd0;
            delta_hist[2]   <= 18'sd0;
            delta_hist[3]   <= 18'sd0;
            delta_hist[4]   <= 18'sd0;
            delta_hist[5]   <= 18'sd0;
            delta_hist[6]   <= 18'sd0;
            delta_hist[7]   <= 18'sd0;
            hist_ptr        <= 3'd0;
            calc_step       <= 3'd0;
            win_sum         <= 21'sd0;
            rpm_prod        <= 32'sd0;
            rpm             <= 32'sd0;
            dir_state       <= 2'd0;
        end else begin
            // Accumulate live pulses continuously
            if (move == 2'd1) begin
                sub_delta_accum <= sub_delta_accum + 18'sd1;
            end else if (move == 2'd2) begin
                sub_delta_accum <= sub_delta_accum - 18'sd1;
            end

            case (calc_step)
                3'd0: begin
                    // Timer counts 25 ms sub-window
                    if (sub_win_timer >= SUB_WIN_CYCLES - 1) begin
                        sub_win_timer <= 22'd0;
                        delta_hist[hist_ptr] <= (move == 2'd1) ? (sub_delta_accum + 18'sd1) :
                                                (move == 2'd2) ? (sub_delta_accum - 18'sd1) : sub_delta_accum;
                        sub_delta_accum <= 18'sd0;
                        hist_ptr        <= hist_ptr + 3'd1;
                        calc_step       <= 3'd1;
                    end else begin
                        sub_win_timer <= sub_win_timer + 22'd1;
                        if (!motion_active) begin
                            rpm       <= 32'sd0;
                            dir_state <= 2'b00; // STOP
                        end
                    end
                end

                3'd1: begin
                    // Stage 1: Sum all 8 sub-windows (total counts in 200 ms)
                    win_sum   <= delta_hist[0] + delta_hist[1] + delta_hist[2] + delta_hist[3] +
                                 delta_hist[4] + delta_hist[5] + delta_hist[6] + delta_hist[7];
                    calc_step <= 3'd2;
                end

                3'd2: begin
                    // Stage 2: Multiply by 75
                    rpm_prod  <= win_sum * 32'sd75;
                    calc_step <= 3'd3;
                end

                3'd3: begin
                    // Stage 3: Divide by 1024 with rounding
                    if (rpm_prod >= 0)
                        rpm <= (rpm_prod + 32'sd512) >>> 10;
                    else
                        rpm <= (rpm_prod - 32'sd512) >>> 10;

                    if (win_sum > 21'sd0)
                        dir_state <= 2'b01; // FWD
                    else if (win_sum < -21'sd0)
                        dir_state <= 2'b10; // REV
                    else
                        dir_state <= 2'b00; // STOP

                    calc_step <= 3'd0;
                end

                default: calc_step <= 3'd0;
            endcase
        end
    end

endmodule
