`timescale 1ns / 1ps
// =============================================================================
// Module: oled_test_top
// Description: Standalone Encoder Diagnostics & Real-Time OLED Telemetry System.
//              - Targets the Digilent Nexys Video (Artix-7 XC7A200T).
//              - Interfaces directly with E50S8G5-1024Z-G24F optical encoder.
//              - Quadrature decoder with 3-stage metastability synchronizers.
//              - Real-time telemetry displayed on 128x32 OLED (4 lines x 16 chars):
//                * Line 0: " ENCODER TESTER "
//                * Line 1: "RPM: +1440  FWD "  (Sliding-window moving average RPM)
//                * Line 2: "POS: 02048/4096 "  (Wrap count 0..4095 per rev)
//                * Line 3: "REV: +00015  Z:0"  (Signed rev count & live Z index)
//              - Pushbutton BTNC (Pin B22) provides manual hardware zero/tare.
//              - Diagnostic LEDs LD7..LD0 provide instant board-level visual feedback.
// =============================================================================

module oled_test_top #(
    parameter CPR        = 4096,    // 1024 PPR * 4 quadrature counts/rev
    parameter SIMULATION = 1'b0
)(
    input  wire       sys_clk_i,    // 100 MHz Onboard Oscillator (Pin R4)
    input  wire       cpu_resetn,   // Red Pushbutton "CPU RESET" (Pin G4, Active-Low)
    input  wire       btnc,         // Center Pushbutton "BTNC" (Pin B22, Zero/Tare)

    // Encoder Hardware Interface (Pmod JA: Pins Y21, AA21, AA20)
    input  wire       enc_a,        // Channel A (Pin Y21)
    input  wire       enc_b,        // Channel B (Pin AA21)
    input  wire       enc_z,        // Index Channel Z (Pin AA20)

    // OLED Hardware Interface Pins (Bank 14)
    output wire       oled_sclk,    // SPI Clock (Pin W21)
    output wire       oled_sdin,    // SPI Data (Pin Y22)
    output wire       oled_dc,      // Data/Command (Pin W22)
    output wire       oled_res,     // Reset (Pin U21)
    output wire       oled_vbat,    // Panel Power Enable (Pin P20)
    output wire       oled_vdd,     // Logic Power Enable (Pin V22)

    // Diagnostic LEDs LD7..LD0 (Bank 13)
    output wire [7:0] led
);

    // -------------------------------------------------------------------------
    // 1. Reset Synchronizer
    // -------------------------------------------------------------------------
    reg rst_sync1, rst_sync2;
    always @(posedge sys_clk_i or negedge cpu_resetn) begin
        if (!cpu_resetn) begin
            rst_sync1 <= 1'b0;
            rst_sync2 <= 1'b0;
        end else begin
            rst_sync1 <= 1'b1;
            rst_sync2 <= rst_sync1;
        end
    end
    wire rst_n = rst_sync2;

    // -------------------------------------------------------------------------
    // 2. Center Button (BTNC) Debouncer for Manual Zero/Tare
    // -------------------------------------------------------------------------
    localparam DEBOUNCE_CYCLES = SIMULATION ? 20'd50 : 20'd1_000_000; // 10 ms at 100 MHz

    reg [19:0] btnc_cnt;
    reg        btnc_sync1, btnc_sync2, btnc_debounced, btnc_prev;
    wire       btnc_pulse = btnc_debounced && !btnc_prev;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            btnc_sync1     <= 1'b0;
            btnc_sync2     <= 1'b0;
            btnc_debounced <= 1'b0;
            btnc_prev      <= 1'b0;
            btnc_cnt       <= 20'd0;
        end else begin
            btnc_sync1 <= btnc;
            btnc_sync2 <= btnc_sync1;
            btnc_prev  <= btnc_debounced;

            if (btnc_sync2 != btnc_debounced) begin
                if (btnc_cnt >= DEBOUNCE_CYCLES) begin
                    btnc_debounced <= btnc_sync2;
                    btnc_cnt       <= 20'd0;
                end else begin
                    btnc_cnt <= btnc_cnt + 20'd1;
                end
            end else begin
                btnc_cnt <= 20'd0;
            end
        end
    end

    // -------------------------------------------------------------------------
    // 3. Encoder Tester Engine Instance
    // -------------------------------------------------------------------------
    wire signed [31:0] enc_rpm_w;
    wire [1:0]         enc_dir_w;
    wire [11:0]        enc_pos_w;
    wire signed [15:0] enc_rev_w;
    wire               enc_z_active_w;
    wire [15:0]        enc_z_count_w;
    wire               enc_motion_w;
    wire               enc_a_sync_w;
    wire               enc_b_sync_w;

    encoder_tester #(
        .CPR        (CPR),
        .SIMULATION (SIMULATION)
    ) u_enc_tester (
        .clk           (sys_clk_i),
        .rst_n         (rst_n),
        .enc_a         (enc_a),
        .enc_b         (enc_b),
        .enc_z         (enc_z),
        .zero_btn      (btnc_pulse),
        .rpm           (enc_rpm_w),
        .dir_state     (enc_dir_w),
        .pos_in_rev    (enc_pos_w),
        .rev_count     (enc_rev_w),
        .z_active      (enc_z_active_w),
        .z_count       (enc_z_count_w),
        .motion_active (enc_motion_w),
        .a_sync_out    (enc_a_sync_w),
        .b_sync_out    (enc_b_sync_w)
    );

    // -------------------------------------------------------------------------
    // 4. 1 Hz Heartbeat Generator
    // -------------------------------------------------------------------------
    localparam CNT_1HZ_MAX = SIMULATION ? 26'd200 : 26'd50_000_000;
    reg [25:0] cnt_1hz;
    reg        heartbeat_1hz;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1hz       <= 26'd0;
            heartbeat_1hz <= 1'b0;
        end else if (cnt_1hz >= CNT_1HZ_MAX - 1) begin
            cnt_1hz       <= 26'd0;
            heartbeat_1hz <= ~heartbeat_1hz;
        end else begin
            cnt_1hz <= cnt_1hz + 26'd1;
        end
    end

    // -------------------------------------------------------------------------
    // 5. Pipelined Decimal Digit Converter (Updates text at 40 Hz / 25 ms)
    //    Eliminates combinational division chains completely.
    // -------------------------------------------------------------------------
    localparam FMT_PERIOD = SIMULATION ? 22'd250 : 22'd2_500_000; // 25 ms
    reg [21:0] fmt_timer;
    reg [2:0]  fmt_step;

    // Captured values for conversion
    reg signed [31:0] c_rpm;
    reg [1:0]         c_dir;
    reg [11:0]        c_pos;
    reg signed [15:0] c_rev;
    reg               c_z;

    reg [15:0] rpm_abs;
    reg [15:0] rev_abs;
    reg        rpm_neg;
    reg        rev_neg;

    // Decimal Digit Registers
    reg [3:0] rpm_d3, rpm_d2, rpm_d1, rpm_d0;
    reg [3:0] pos_d4, pos_d3, pos_d2, pos_d1, pos_d0;
    reg [3:0] rev_d4, rev_d3, rev_d2, rev_d1, rev_d0;

    // Intermediate remainder registers
    reg [15:0] rpm_rem_th, rpm_rem_h;
    reg [11:0] pos_rem_th, pos_rem_h;
    reg [15:0] rev_rem_tth, rev_rem_th, rev_rem_h;

    // 64-character text display buffer
    reg [7:0] oled_chars [0:63];

    integer i;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            fmt_timer   <= 22'd0;
            fmt_step    <= 3'd0;
            c_rpm       <= 32'sd0;
            c_dir       <= 2'd0;
            c_pos       <= 12'd0;
            c_rev       <= 16'sd0;
            c_z         <= 1'b0;
            rpm_abs     <= 16'd0;
            rev_abs     <= 16'd0;
            rpm_neg     <= 1'b0;
            rev_neg     <= 1'b0;
            rpm_d3 <= 0; rpm_d2 <= 0; rpm_d1 <= 0; rpm_d0 <= 0;
            pos_d4 <= 0; pos_d3 <= 0; pos_d2 <= 0; pos_d1 <= 0; pos_d0 <= 0;
            rev_d4 <= 0; rev_d3 <= 0; rev_d2 <= 0; rev_d1 <= 0; rev_d0 <= 0;
            rpm_rem_th <= 0; rpm_rem_h <= 0;
            pos_rem_th <= 0; pos_rem_h <= 0;
            rev_rem_tth <= 0; rev_rem_th <= 0; rev_rem_h <= 0;

            for (i = 0; i < 64; i = i + 1)
                oled_chars[i] <= 8'h20; // Spaces
        end else begin
            case (fmt_step)
                3'd0: begin
                    // Timer counts 25 ms interval
                    if (fmt_timer >= FMT_PERIOD - 1) begin
                        fmt_timer <= 22'd0;
                        fmt_step  <= 3'd1;
                        // Latch inputs
                        c_rpm <= enc_rpm_w;
                        c_dir <= enc_dir_w;
                        c_pos <= enc_pos_w;
                        c_rev <= enc_rev_w;
                        c_z   <= enc_z_active_w;
                    end else begin
                        fmt_timer <= fmt_timer + 22'd1;
                    end
                end

                3'd1: begin
                    // Step 1: Pre-process signs and magnitudes
                    if (c_rpm < 0) begin
                        rpm_neg <= 1'b1;
                        rpm_abs <= (c_rpm < -32'sd9999) ? 16'd9999 : -c_rpm[15:0];
                    end else begin
                        rpm_neg <= 1'b0;
                        rpm_abs <= (c_rpm > 32'sd9999) ? 16'd9999 : c_rpm[15:0];
                    end

                    if (c_rev < 0) begin
                        rev_neg <= 1'b1;
                        rev_abs <= (c_rev < -16'sd32767) ? 16'd32767 : -c_rev;
                    end else begin
                        rev_neg <= 1'b0;
                        rev_abs <= c_rev;
                    end

                    pos_d4   <= 4'd0;
                    fmt_step <= 3'd2;
                end

                3'd2: begin
                    // Step 2: Compute thousands and ten-thousands
                    rpm_d3     <= rpm_abs / 1000;
                    rpm_rem_th <= rpm_abs % 1000;

                    pos_d3     <= c_pos / 1000;
                    pos_rem_th <= c_pos % 1000;

                    rev_d4      <= rev_abs / 10000;
                    rev_rem_tth <= rev_abs % 10000;

                    fmt_step <= 3'd3;
                end

                3'd3: begin
                    // Step 3: Compute hundreds
                    rpm_d2    <= rpm_rem_th / 100;
                    rpm_rem_h <= rpm_rem_th % 100;

                    pos_d2    <= pos_rem_th / 100;
                    pos_rem_h <= pos_rem_th % 100;

                    rev_d3     <= rev_rem_tth / 1000;
                    rev_rem_th <= rev_rem_tth % 1000;

                    fmt_step <= 3'd4;
                end

                3'd4: begin
                    // Step 4: Compute tens and units
                    rpm_d1 <= rpm_rem_h / 10;
                    rpm_d0 <= rpm_rem_h % 10;

                    pos_d1 <= pos_rem_h / 10;
                    pos_d0 <= pos_rem_h % 10;

                    rev_d2    <= rev_rem_th / 100;
                    rev_rem_h <= rev_rem_th % 100;

                    fmt_step <= 3'd5;
                end

                3'd5: begin
                    // Step 5: Final units and store into display buffer
                    rev_d1 <= rev_rem_h / 10;
                    rev_d0 <= rev_rem_h % 10;

                    // Line 0: " ENCODER TESTER "
                    oled_chars[0]  <= " "; oled_chars[1]  <= "E"; oled_chars[2]  <= "N"; oled_chars[3]  <= "C";
                    oled_chars[4]  <= "O"; oled_chars[5]  <= "D"; oled_chars[6]  <= "E"; oled_chars[7]  <= "R";
                    oled_chars[8]  <= " "; oled_chars[9]  <= "T"; oled_chars[10] <= "E"; oled_chars[11] <= "S";
                    oled_chars[12] <= "T"; oled_chars[13] <= "E"; oled_chars[14] <= "R"; oled_chars[15] <= " ";

                    // Line 1: "RPM: +1440  FWD "
                    oled_chars[16] <= "R";
                    oled_chars[17] <= "P";
                    oled_chars[18] <= "M";
                    oled_chars[19] <= ":";
                    oled_chars[20] <= " ";
                    oled_chars[21] <= (c_rpm == 0) ? " " : (rpm_neg ? "-" : "+");
                    oled_chars[22] <= (rpm_d3 == 0) ? " " : (8'h30 + {4'd0, rpm_d3});
                    oled_chars[23] <= (rpm_d3 == 0 && rpm_d2 == 0) ? " " : (8'h30 + {4'd0, rpm_d2});
                    oled_chars[24] <= (rpm_d3 == 0 && rpm_d2 == 0 && rpm_d1 == 0) ? " " : (8'h30 + {4'd0, rpm_d1});
                    oled_chars[25] <= 8'h30 + {4'd0, rpm_d0};
                    oled_chars[26] <= " ";
                    oled_chars[27] <= " ";
                    case (c_dir)
                        2'b01: begin oled_chars[28] <= "F"; oled_chars[29] <= "W"; oled_chars[30] <= "D"; oled_chars[31] <= " "; end
                        2'b10: begin oled_chars[28] <= "R"; oled_chars[29] <= "E"; oled_chars[30] <= "V"; oled_chars[31] <= " "; end
                        default: begin oled_chars[28] <= "S"; oled_chars[29] <= "T"; oled_chars[30] <= "O"; oled_chars[31] <= "P"; end
                    endcase

                    // Line 2: "POS: 02048/4096 "
                    oled_chars[32] <= "P";
                    oled_chars[33] <= "O";
                    oled_chars[34] <= "S";
                    oled_chars[35] <= ":";
                    oled_chars[36] <= " ";
                    oled_chars[37] <= "0";
                    oled_chars[38] <= 8'h30 + {4'd0, pos_d3};
                    oled_chars[39] <= 8'h30 + {4'd0, pos_d2};
                    oled_chars[40] <= 8'h30 + {4'd0, pos_d1};
                    oled_chars[41] <= 8'h30 + {4'd0, pos_d0};
                    oled_chars[42] <= "/";
                    oled_chars[43] <= "4";
                    oled_chars[44] <= "0";
                    oled_chars[45] <= "9";
                    oled_chars[46] <= "6";
                    oled_chars[47] <= " ";

                    // Line 3: "REV: +00015  Z:0"
                    oled_chars[48] <= "R";
                    oled_chars[49] <= "E";
                    oled_chars[50] <= "V";
                    oled_chars[51] <= ":";
                    oled_chars[52] <= " ";
                    oled_chars[53] <= rev_neg ? "-" : "+";
                    oled_chars[54] <= 8'h30 + {4'd0, rev_d4};
                    oled_chars[55] <= 8'h30 + {4'd0, rev_d3};
                    oled_chars[56] <= 8'h30 + {4'd0, rev_d2};
                    oled_chars[57] <= 8'h30 + {4'd0, rev_d1};
                    oled_chars[58] <= 8'h30 + {4'd0, rev_d0};
                    oled_chars[59] <= " ";
                    oled_chars[60] <= " ";
                    oled_chars[61] <= "Z";
                    oled_chars[62] <= ":";
                    oled_chars[63] <= c_z ? "1" : "0";

                    fmt_step <= 3'd0;
                end

                default: fmt_step <= 3'd0;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // 6. Direct Memory Read Multiplexer to OLED Controller
    // -------------------------------------------------------------------------
    wire [5:0] text_addr_w;
    wire [7:0] text_char_w = oled_chars[text_addr_w];
    wire       init_done_w;

    // -------------------------------------------------------------------------
    // 7. OLED Controller Instance (SSD1306 SPI Master Engine)
    // -------------------------------------------------------------------------
    oled_controller #(
        .SIMULATION (SIMULATION)
    ) u_oled (
        .clk        (sys_clk_i),
        .rst_n      (rst_n),
        .text_addr  (text_addr_w),
        .text_char  (text_char_w),
        .init_done  (init_done_w),
        .oled_sclk  (oled_sclk),
        .oled_sdin  (oled_sdin),
        .oled_dc    (oled_dc),
        .oled_res   (oled_res),
        .oled_vbat  (oled_vbat),
        .oled_vdd   (oled_vdd)
    );

    // -------------------------------------------------------------------------
    // 8. Diagnostic LEDs Mapping
    // -------------------------------------------------------------------------
    assign led[0] = heartbeat_1hz;                       // LD0: 1 Hz Heartbeat
    assign led[1] = init_done_w;                         // LD1: OLED Init Complete (Solid ON)
    assign led[2] = (enc_dir_w == 2'b01);                // LD2: Forward Rotation (FWD)
    assign led[3] = (enc_dir_w == 2'b10);                // LD3: Reverse Rotation (REV)
    assign led[4] = enc_motion_w;                        // LD4: Motion Active
    assign led[5] = enc_z_active_w;                      // LD5: Index Z Pulse (50 ms flash per rev)
    assign led[6] = enc_a_sync_w;                        // LD6: Live Channel A (flickers with shaft)
    assign led[7] = enc_b_sync_w;                        // LD7: Live Channel B (flickers with shaft)

endmodule
