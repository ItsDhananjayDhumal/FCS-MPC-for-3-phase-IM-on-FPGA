`timescale 1ns / 1ps
// =============================================================================
// Module: oled_test_top
// Description: Standalone 2 Hz Alphabet Display Test for Nexys Video OLED.
//              - Targets the onboard 128x32 SSD1306 monochrome OLED display.
//              - Cycles lowercase letters 'a', 'b', 'c', ... 'z' at exactly 2 Hz
//                (new letter every 0.5 seconds).
//              - Shows board status, current character, and ASCII hex code.
//              - Onboard LEDs provide real-time diagnostic verification:
//                * LED0: 2 Hz toggle (flashes with every letter change)
//                * LED1: OLED initialization complete (Solid ON)
//                * LED[7:2]: 6-bit binary counter of character index (0..25)
// =============================================================================

module oled_test_top #(
    parameter SIMULATION = 1'b0
)(
    input  wire       sys_clk_i,    // 100 MHz Onboard Oscillator (Pin R4)
    input  wire       cpu_resetn,   // Red Pushbutton "CPU RESET" (Pin G4, Active-Low)

    // OLED Hardware Interface Pins
    output wire       oled_sclk,    // SPI Clock (Pin W21)
    output wire       oled_sdin,    // SPI Data (Pin Y22)
    output wire       oled_dc,      // Data/Command (Pin W22)
    output wire       oled_res,     // Reset (Pin U21)
    output wire       oled_vbat,    // Panel Power Enable (Pin P20)
    output wire       oled_vdd,     // Logic Power Enable (Pin V22)

    // Diagnostic LEDs
    output wire [7:0] led           // User LEDs LD7..LD0
);

    // -------------------------------------------------------------------------
    // Reset Synchronizer
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
    // 2 Hz Tick Generator & Alphabet Character Sequencer
    // 2 Hz = 0.5 second period = 50,000,000 clock cycles at 100 MHz
    // -------------------------------------------------------------------------
    wire init_done_w;
    localparam CNT_2HZ_MAX = SIMULATION ? 26'd200 : 26'd50_000_000;

    reg [25:0] cnt_2hz;
    reg [4:0]  char_idx;      // 0 to 25 for 'a' to 'z'
    reg        heartbeat_2hz;

    always @(posedge sys_clk_i or negedge rst_n) begin
        if (!rst_n) begin
            cnt_2hz       <= 26'd0;
            char_idx      <= 5'd0;
            heartbeat_2hz <= 1'b0;
        end else if (!init_done_w) begin
            // Hold at 'a' while OLED is powering on and initializing
            cnt_2hz       <= 26'd0;
            char_idx      <= 5'd0;
            heartbeat_2hz <= 1'b0;
        end else begin
            // 2 Hz character cycling once display is active
            if (cnt_2hz >= CNT_2HZ_MAX - 1) begin
                cnt_2hz       <= 26'd0;
                heartbeat_2hz <= ~heartbeat_2hz;
                if (char_idx >= 5'd25)
                    char_idx <= 5'd0;
                else
                    char_idx <= char_idx + 5'd1;
            end else begin
                cnt_2hz <= cnt_2hz + 26'd1;
            end
        end
    end

    // Current ASCII character (starts at 'a' = 0x61, ends at 'z' = 0x7A)
    wire [7:0] current_char = 8'h61 + {3'd0, char_idx};

    // Helper functions to convert 4-bit nibbles to ASCII hex characters ('0'..'9', 'A'..'F')
    function [7:0] hex_to_ascii;
        input [3:0] nibble;
        begin
            if (nibble < 4'd10)
                hex_to_ascii = 8'h30 + {4'd0, nibble};       // '0'..'9'
            else
                hex_to_ascii = 8'h41 + {4'd0, nibble - 4'd10}; // 'A'..'F'
        end
    endfunction

    wire [7:0] ascii_hi = hex_to_ascii(current_char[7:4]);
    wire [7:0] ascii_lo = hex_to_ascii(current_char[3:0]);

    // -------------------------------------------------------------------------
    // 64-Character Text Buffer (4 Lines x 16 Characters)
    // -------------------------------------------------------------------------
    // Line 0: " NEXYS VIDEO OK "
    // Line 1: " OLED 2HZ TEST  "
    // Line 2: " LETTER:  >>> a "
    // Line 3: " CODE: 0x61 [a] "
    // -------------------------------------------------------------------------
    wire [5:0] text_addr_w;
    reg  [7:0] text_char_r;

    always @(*) begin
        case (text_addr_w)
            // Line 0 (Page 0): " NEXYS VIDEO OK "
            6'd0:  text_char_r = " ";
            6'd1:  text_char_r = "N";
            6'd2:  text_char_r = "E";
            6'd3:  text_char_r = "X";
            6'd4:  text_char_r = "Y";
            6'd5:  text_char_r = "S";
            6'd6:  text_char_r = " ";
            6'd7:  text_char_r = "V";
            6'd8:  text_char_r = "I";
            6'd9:  text_char_r = "D";
            6'd10: text_char_r = "E";
            6'd11: text_char_r = "O";
            6'd12: text_char_r = " ";
            6'd13: text_char_r = "O";
            6'd14: text_char_r = "K";
            6'd15: text_char_r = " ";

            // Line 1 (Page 1): " OLED 2HZ TEST  "
            6'd16: text_char_r = " ";
            6'd17: text_char_r = "O";
            6'd18: text_char_r = "L";
            6'd19: text_char_r = "E";
            6'd20: text_char_r = "D";
            6'd21: text_char_r = " ";
            6'd22: text_char_r = "2";
            6'd23: text_char_r = "H";
            6'd24: text_char_r = "Z";
            6'd25: text_char_r = " ";
            6'd26: text_char_r = "T";
            6'd27: text_char_r = "E";
            6'd28: text_char_r = "S";
            6'd29: text_char_r = "T";
            6'd30: text_char_r = " ";
            6'd31: text_char_r = " ";

            // Line 2 (Page 2): " LETTER:  >>> a "
            6'd32: text_char_r = " ";
            6'd33: text_char_r = "L";
            6'd34: text_char_r = "E";
            6'd35: text_char_r = "T";
            6'd36: text_char_r = "T";
            6'd37: text_char_r = "E";
            6'd38: text_char_r = "R";
            6'd39: text_char_r = ":";
            6'd40: text_char_r = " ";
            6'd41: text_char_r = " ";
            6'd42: text_char_r = ">";
            6'd43: text_char_r = ">";
            6'd44: text_char_r = ">";
            6'd45: text_char_r = " ";
            6'd46: text_char_r = current_char; // Live changing character 'a'..'z'!
            6'd47: text_char_r = " ";

            // Line 3 (Page 3): " CODE: 0x61 [a] "
            6'd48: text_char_r = " ";
            6'd49: text_char_r = "C";
            6'd50: text_char_r = "O";
            6'd51: text_char_r = "D";
            6'd52: text_char_r = "E";
            6'd53: text_char_r = ":";
            6'd54: text_char_r = " ";
            6'd55: text_char_r = "0";
            6'd56: text_char_r = "x";
            6'd57: text_char_r = ascii_hi;     // ASCII Hex High Nibble
            6'd58: text_char_r = ascii_lo;     // ASCII Hex Low Nibble
            6'd59: text_char_r = " ";
            6'd60: text_char_r = "[";
            6'd61: text_char_r = current_char; // Live character in brackets
            6'd62: text_char_r = "]";
            6'd63: text_char_r = " ";

            default: text_char_r = " ";
        endcase
    end

    // -------------------------------------------------------------------------
    // OLED Controller Instance
    // -------------------------------------------------------------------------
    oled_controller #(
        .SIMULATION (SIMULATION)
    ) u_oled (
        .clk        (sys_clk_i),
        .rst_n      (rst_n),
        .text_addr  (text_addr_w),
        .text_char  (text_char_r),
        .init_done  (init_done_w),
        .oled_sclk  (oled_sclk),
        .oled_sdin  (oled_sdin),
        .oled_dc    (oled_dc),
        .oled_res   (oled_res),
        .oled_vbat  (oled_vbat),
        .oled_vdd   (oled_vdd)
    );

    // -------------------------------------------------------------------------
    // Diagnostic LEDs
    // -------------------------------------------------------------------------
    assign led[0]   = heartbeat_2hz;          // Blinks at 2 Hz in sync with letter change
    assign led[1]   = init_done_w;            // Solid ON when OLED initialization finishes
    assign led[7:2] = {1'b0, char_idx};       // 5-bit character index (0..25)

endmodule
