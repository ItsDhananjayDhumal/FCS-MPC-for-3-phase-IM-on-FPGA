`timescale 1ns / 1ps
// =============================================================================
// Module: oled_font_rom
// Description: Standard 8x8 ASCII Font ROM for 128x32 OLED (SSD1306).
//              Maps an 8-bit ASCII character code and a 3-bit column index
//              to an 8-bit vertical pixel column (Bit 0 = top, Bit 7 = bottom).
// =============================================================================

module oled_font_rom (
    input  wire [7:0] ascii_code,
    input  wire [2:0] col_idx,
    output reg  [7:0] pixel_col
);

    always @(*) begin
        case (ascii_code)
            // Space (0x20)
            8'h20: pixel_col = 8'h00;

            // '!' (0x21)
            8'h21: case (col_idx)
                3'd2: pixel_col = 8'h5F;
                default: pixel_col = 8'h00;
            endcase

            // '"' (0x22)
            8'h22: case (col_idx)
                3'd1: pixel_col = 8'h07;
                3'd3: pixel_col = 8'h07;
                default: pixel_col = 8'h00;
            endcase

            // '#' (0x23)
            8'h23: case (col_idx)
                3'd1: pixel_col = 8'h14;
                3'd2: pixel_col = 8'h7F;
                3'd3: pixel_col = 8'h14;
                3'd4: pixel_col = 8'h7F;
                3'd5: pixel_col = 8'h14;
                default: pixel_col = 8'h00;
            endcase

            // '%' (0x25)
            8'h25: case (col_idx)
                3'd1: pixel_col = 8'h23;
                3'd2: pixel_col = 8'h13;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h64;
                3'd5: pixel_col = 8'h62;
                default: pixel_col = 8'h00;
            endcase

            // '+' (0x2B)
            8'h2B: case (col_idx)
                3'd1: pixel_col = 8'h08;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h3E;
                3'd4: pixel_col = 8'h08;
                3'd5: pixel_col = 8'h08;
                default: pixel_col = 8'h00;
            endcase

            // '-' (0x2D)
            8'h2D: case (col_idx)
                3'd1: pixel_col = 8'h08;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h08;
                3'd5: pixel_col = 8'h08;
                default: pixel_col = 8'h00;
            endcase

            // '.' (0x2E)
            8'h2E: case (col_idx)
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h40;
                default: pixel_col = 8'h00;
            endcase

            // '/' (0x2F)
            8'h2F: case (col_idx)
                3'd1: pixel_col = 8'h20;
                3'd2: pixel_col = 8'h10;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h04;
                3'd5: pixel_col = 8'h02;
                default: pixel_col = 8'h00;
            endcase

            // Digits '0' - '9' (0x30 - 0x39)
            8'h30: case (col_idx) // '0'
                3'd1: pixel_col = 8'h3E;
                3'd2: pixel_col = 8'h51;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h45;
                3'd5: pixel_col = 8'h3E;
                default: pixel_col = 8'h00;
            endcase
            8'h31: case (col_idx) // '1'
                3'd2: pixel_col = 8'h42;
                3'd3: pixel_col = 8'h7F;
                3'd4: pixel_col = 8'h40;
                default: pixel_col = 8'h00;
            endcase
            8'h32: case (col_idx) // '2'
                3'd1: pixel_col = 8'h42;
                3'd2: pixel_col = 8'h61;
                3'd3: pixel_col = 8'h51;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h46;
                default: pixel_col = 8'h00;
            endcase
            8'h33: case (col_idx) // '3'
                3'd1: pixel_col = 8'h21;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h45;
                3'd4: pixel_col = 8'h4B;
                3'd5: pixel_col = 8'h31;
                default: pixel_col = 8'h00;
            endcase
            8'h34: case (col_idx) // '4'
                3'd1: pixel_col = 8'h18;
                3'd2: pixel_col = 8'h14;
                3'd3: pixel_col = 8'h12;
                3'd4: pixel_col = 8'h7F;
                3'd5: pixel_col = 8'h10;
                default: pixel_col = 8'h00;
            endcase
            8'h35: case (col_idx) // '5'
                3'd1: pixel_col = 8'h27;
                3'd2: pixel_col = 8'h45;
                3'd3: pixel_col = 8'h45;
                3'd4: pixel_col = 8'h45;
                3'd5: pixel_col = 8'h39;
                default: pixel_col = 8'h00;
            endcase
            8'h36: case (col_idx) // '6'
                3'd1: pixel_col = 8'h3C;
                3'd2: pixel_col = 8'h4A;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h30;
                default: pixel_col = 8'h00;
            endcase
            8'h37: case (col_idx) // '7'
                3'd1: pixel_col = 8'h01;
                3'd2: pixel_col = 8'h71;
                3'd3: pixel_col = 8'h09;
                3'd4: pixel_col = 8'h05;
                3'd5: pixel_col = 8'h03;
                default: pixel_col = 8'h00;
            endcase
            8'h38: case (col_idx) // '8'
                3'd1: pixel_col = 8'h36;
                3'd2: pixel_col = 8'h49;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h36;
                default: pixel_col = 8'h00;
            endcase
            8'h39: case (col_idx) // '9'
                3'd1: pixel_col = 8'h06;
                3'd2: pixel_col = 8'h49;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h29;
                3'd5: pixel_col = 8'h1E;
                default: pixel_col = 8'h00;
            endcase

            // ':' (0x3A)
            8'h3A: case (col_idx)
                3'd2: pixel_col = 8'h36;
                3'd3: pixel_col = 8'h36;
                default: pixel_col = 8'h00;
            endcase

            // '<' (0x3C)
            8'h3C: case (col_idx)
                3'd1: pixel_col = 8'h08;
                3'd2: pixel_col = 8'h14;
                3'd3: pixel_col = 8'h22;
                3'd4: pixel_col = 8'h41;
                default: pixel_col = 8'h00;
            endcase

            // '=' (0x3D)
            8'h3D: case (col_idx)
                3'd1: pixel_col = 8'h14;
                3'd2: pixel_col = 8'h14;
                3'd3: pixel_col = 8'h14;
                3'd4: pixel_col = 8'h14;
                3'd5: pixel_col = 8'h14;
                default: pixel_col = 8'h00;
            endcase

            // '>' (0x3E)
            8'h3E: case (col_idx)
                3'd1: pixel_col = 8'h41;
                3'd2: pixel_col = 8'h22;
                3'd3: pixel_col = 8'h14;
                3'd4: pixel_col = 8'h08;
                default: pixel_col = 8'h00;
            endcase

            // Uppercase 'A' - 'Z' (0x41 - 0x5A)
            8'h41: case (col_idx) // 'A'
                3'd1: pixel_col = 8'h7E;
                3'd2: pixel_col = 8'h11;
                3'd3: pixel_col = 8'h11;
                3'd4: pixel_col = 8'h11;
                3'd5: pixel_col = 8'h7E;
                default: pixel_col = 8'h00;
            endcase
            8'h42: case (col_idx) // 'B'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h49;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h36;
                default: pixel_col = 8'h00;
            endcase
            8'h43: case (col_idx) // 'C'
                3'd1: pixel_col = 8'h3E;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h41;
                3'd5: pixel_col = 8'h22;
                default: pixel_col = 8'h00;
            endcase
            8'h44: case (col_idx) // 'D'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h22;
                3'd5: pixel_col = 8'h1C;
                default: pixel_col = 8'h00;
            endcase
            8'h45: case (col_idx) // 'E'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h49;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h41;
                default: pixel_col = 8'h00;
            endcase
            8'h46: case (col_idx) // 'F'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h09;
                3'd3: pixel_col = 8'h09;
                3'd4: pixel_col = 8'h09;
                3'd5: pixel_col = 8'h01;
                default: pixel_col = 8'h00;
            endcase
            8'h47: case (col_idx) // 'G'
                3'd1: pixel_col = 8'h3E;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h7A;
                default: pixel_col = 8'h00;
            endcase
            8'h48: case (col_idx) // 'H'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h08;
                3'd5: pixel_col = 8'h7F;
                default: pixel_col = 8'h00;
            endcase
            8'h49: case (col_idx) // 'I'
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h7F;
                3'd4: pixel_col = 8'h41;
                default: pixel_col = 8'h00;
            endcase
            8'h4A: case (col_idx) // 'J'
                3'd1: pixel_col = 8'h20;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h3F;
                3'd5: pixel_col = 8'h01;
                default: pixel_col = 8'h00;
            endcase
            8'h4B: case (col_idx) // 'K'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h14;
                3'd4: pixel_col = 8'h22;
                3'd5: pixel_col = 8'h41;
                default: pixel_col = 8'h00;
            endcase
            8'h4C: case (col_idx) // 'L'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h40;
                3'd4: pixel_col = 8'h40;
                3'd5: pixel_col = 8'h40;
                default: pixel_col = 8'h00;
            endcase
            8'h4D: case (col_idx) // 'M'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h02;
                3'd3: pixel_col = 8'h04;
                3'd4: pixel_col = 8'h02;
                3'd5: pixel_col = 8'h7F;
                default: pixel_col = 8'h00;
            endcase
            8'h4E: case (col_idx) // 'N'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h04;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h10;
                3'd5: pixel_col = 8'h7F;
                default: pixel_col = 8'h00;
            endcase
            8'h4F: case (col_idx) // 'O'
                3'd1: pixel_col = 8'h3E;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h41;
                3'd5: pixel_col = 8'h3E;
                default: pixel_col = 8'h00;
            endcase
            8'h50: case (col_idx) // 'P'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h09;
                3'd3: pixel_col = 8'h09;
                3'd4: pixel_col = 8'h09;
                3'd5: pixel_col = 8'h06;
                default: pixel_col = 8'h00;
            endcase
            8'h51: case (col_idx) // 'Q'
                3'd1: pixel_col = 8'h3E;
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h51;
                3'd4: pixel_col = 8'h21;
                3'd5: pixel_col = 8'h5E;
                default: pixel_col = 8'h00;
            endcase
            8'h52: case (col_idx) // 'R'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h09;
                3'd3: pixel_col = 8'h19;
                3'd4: pixel_col = 8'h29;
                3'd5: pixel_col = 8'h46;
                default: pixel_col = 8'h00;
            endcase
            8'h53: case (col_idx) // 'S'
                3'd1: pixel_col = 8'h46;
                3'd2: pixel_col = 8'h49;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h49;
                3'd5: pixel_col = 8'h31;
                default: pixel_col = 8'h00;
            endcase
            8'h54: case (col_idx) // 'T'
                3'd1: pixel_col = 8'h01;
                3'd2: pixel_col = 8'h01;
                3'd3: pixel_col = 8'h7F;
                3'd4: pixel_col = 8'h01;
                3'd5: pixel_col = 8'h01;
                default: pixel_col = 8'h00;
            endcase
            8'h55: case (col_idx) // 'U'
                3'd1: pixel_col = 8'h3F;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h40;
                3'd4: pixel_col = 8'h40;
                3'd5: pixel_col = 8'h3F;
                default: pixel_col = 8'h00;
            endcase
            8'h56: case (col_idx) // 'V'
                3'd1: pixel_col = 8'h1F;
                3'd2: pixel_col = 8'h20;
                3'd3: pixel_col = 8'h40;
                3'd4: pixel_col = 8'h20;
                3'd5: pixel_col = 8'h1F;
                default: pixel_col = 8'h00;
            endcase
            8'h57: case (col_idx) // 'W'
                3'd1: pixel_col = 8'h3F;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h38;
                3'd4: pixel_col = 8'h40;
                3'd5: pixel_col = 8'h3F;
                default: pixel_col = 8'h00;
            endcase
            8'h58: case (col_idx) // 'X'
                3'd1: pixel_col = 8'h63;
                3'd2: pixel_col = 8'h14;
                3'd3: pixel_col = 8'h08;
                3'd4: pixel_col = 8'h14;
                3'd5: pixel_col = 8'h63;
                default: pixel_col = 8'h00;
            endcase
            8'h59: case (col_idx) // 'Y'
                3'd1: pixel_col = 8'h07;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h70;
                3'd4: pixel_col = 8'h08;
                3'd5: pixel_col = 8'h07;
                default: pixel_col = 8'h00;
            endcase
            8'h5A: case (col_idx) // 'Z'
                3'd1: pixel_col = 8'h61;
                3'd2: pixel_col = 8'h51;
                3'd3: pixel_col = 8'h49;
                3'd4: pixel_col = 8'h45;
                3'd5: pixel_col = 8'h43;
                default: pixel_col = 8'h00;
            endcase

            // '[' (0x5B)
            8'h5B: case (col_idx)
                3'd2: pixel_col = 8'h7F;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h41;
                default: pixel_col = 8'h00;
            endcase

            // ']' (0x5D)
            8'h5D: case (col_idx)
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h41;
                3'd4: pixel_col = 8'h7F;
                default: pixel_col = 8'h00;
            endcase

            // '_' (0x5F)
            8'h5F: case (col_idx)
                3'd1: pixel_col = 8'h80;
                3'd2: pixel_col = 8'h80;
                3'd3: pixel_col = 8'h80;
                3'd4: pixel_col = 8'h80;
                3'd5: pixel_col = 8'h80;
                default: pixel_col = 8'h00;
            endcase

            // Lowercase 'a' - 'z' (0x61 - 0x7A)
            8'h61: case (col_idx) // 'a'
                3'd1: pixel_col = 8'h20;
                3'd2: pixel_col = 8'h54;
                3'd3: pixel_col = 8'h54;
                3'd4: pixel_col = 8'h54;
                3'd5: pixel_col = 8'h78;
                default: pixel_col = 8'h00;
            endcase
            8'h62: case (col_idx) // 'b'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h48;
                3'd3: pixel_col = 8'h44;
                3'd4: pixel_col = 8'h44;
                3'd5: pixel_col = 8'h38;
                default: pixel_col = 8'h00;
            endcase
            8'h63: case (col_idx) // 'c'
                3'd1: pixel_col = 8'h38;
                3'd2: pixel_col = 8'h44;
                3'd3: pixel_col = 8'h44;
                3'd4: pixel_col = 8'h44;
                3'd5: pixel_col = 8'h20;
                default: pixel_col = 8'h00;
            endcase
            8'h64: case (col_idx) // 'd'
                3'd1: pixel_col = 8'h38;
                3'd2: pixel_col = 8'h44;
                3'd3: pixel_col = 8'h44;
                3'd4: pixel_col = 8'h48;
                3'd5: pixel_col = 8'h7F;
                default: pixel_col = 8'h00;
            endcase
            8'h65: case (col_idx) // 'e'
                3'd1: pixel_col = 8'h38;
                3'd2: pixel_col = 8'h54;
                3'd3: pixel_col = 8'h54;
                3'd4: pixel_col = 8'h54;
                3'd5: pixel_col = 8'h18;
                default: pixel_col = 8'h00;
            endcase
            8'h66: case (col_idx) // 'f'
                3'd1: pixel_col = 8'h08;
                3'd2: pixel_col = 8'h7E;
                3'd3: pixel_col = 8'h09;
                3'd4: pixel_col = 8'h01;
                3'd5: pixel_col = 8'h02;
                default: pixel_col = 8'h00;
            endcase
            8'h67: case (col_idx) // 'g'
                3'd1: pixel_col = 8'h18;
                3'd2: pixel_col = 8'hA4;
                3'd3: pixel_col = 8'hA4;
                3'd4: pixel_col = 8'hA4;
                3'd5: pixel_col = 8'h7C;
                default: pixel_col = 8'h00;
            endcase
            8'h68: case (col_idx) // 'h'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h04;
                3'd4: pixel_col = 8'h04;
                3'd5: pixel_col = 8'h78;
                default: pixel_col = 8'h00;
            endcase
            8'h69: case (col_idx) // 'i'
                3'd2: pixel_col = 8'h44;
                3'd3: pixel_col = 8'h7D;
                3'd4: pixel_col = 8'h40;
                default: pixel_col = 8'h00;
            endcase
            8'h6A: case (col_idx) // 'j'
                3'd1: pixel_col = 8'h40;
                3'd2: pixel_col = 8'h80;
                3'd3: pixel_col = 8'h84;
                3'd4: pixel_col = 8'h7D;
                default: pixel_col = 8'h00;
            endcase
            8'h6B: case (col_idx) // 'k'
                3'd1: pixel_col = 8'h7F;
                3'd2: pixel_col = 8'h10;
                3'd3: pixel_col = 8'h28;
                3'd4: pixel_col = 8'h44;
                default: pixel_col = 8'h00;
            endcase
            8'h6C: case (col_idx) // 'l'
                3'd2: pixel_col = 8'h41;
                3'd3: pixel_col = 8'h7F;
                3'd4: pixel_col = 8'h40;
                default: pixel_col = 8'h00;
            endcase
            8'h6D: case (col_idx) // 'm'
                3'd1: pixel_col = 8'h7C;
                3'd2: pixel_col = 8'h04;
                3'd3: pixel_col = 8'h18;
                3'd4: pixel_col = 8'h04;
                3'd5: pixel_col = 8'h78;
                default: pixel_col = 8'h00;
            endcase
            8'h6E: case (col_idx) // 'n'
                3'd1: pixel_col = 8'h7C;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h04;
                3'd4: pixel_col = 8'h04;
                3'd5: pixel_col = 8'h78;
                default: pixel_col = 8'h00;
            endcase
            8'h6F: case (col_idx) // 'o'
                3'd1: pixel_col = 8'h38;
                3'd2: pixel_col = 8'h44;
                3'd3: pixel_col = 8'h44;
                3'd4: pixel_col = 8'h44;
                3'd5: pixel_col = 8'h38;
                default: pixel_col = 8'h00;
            endcase
            8'h70: case (col_idx) // 'p'
                3'd1: pixel_col = 8'hFC;
                3'd2: pixel_col = 8'h24;
                3'd3: pixel_col = 8'h24;
                3'd4: pixel_col = 8'h24;
                3'd5: pixel_col = 8'h18;
                default: pixel_col = 8'h00;
            endcase
            8'h71: case (col_idx) // 'q'
                3'd1: pixel_col = 8'h18;
                3'd2: pixel_col = 8'h24;
                3'd3: pixel_col = 8'h24;
                3'd4: pixel_col = 8'h28;
                3'd5: pixel_col = 8'hFC;
                default: pixel_col = 8'h00;
            endcase
            8'h72: case (col_idx) // 'r'
                3'd1: pixel_col = 8'h7C;
                3'd2: pixel_col = 8'h08;
                3'd3: pixel_col = 8'h04;
                3'd4: pixel_col = 8'h04;
                3'd5: pixel_col = 8'h08;
                default: pixel_col = 8'h00;
            endcase
            8'h73: case (col_idx) // 's'
                3'd1: pixel_col = 8'h48;
                3'd2: pixel_col = 8'h54;
                3'd3: pixel_col = 8'h54;
                3'd4: pixel_col = 8'h54;
                3'd5: pixel_col = 8'h20;
                default: pixel_col = 8'h00;
            endcase
            8'h74: case (col_idx) // 't'
                3'd1: pixel_col = 8'h04;
                3'd2: pixel_col = 8'h3F;
                3'd3: pixel_col = 8'h44;
                3'd4: pixel_col = 8'h40;
                3'd5: pixel_col = 8'h20;
                default: pixel_col = 8'h00;
            endcase
            8'h75: case (col_idx) // 'u'
                3'd1: pixel_col = 8'h3C;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h40;
                3'd4: pixel_col = 8'h20;
                3'd5: pixel_col = 8'h7C;
                default: pixel_col = 8'h00;
            endcase
            8'h76: case (col_idx) // 'v'
                3'd1: pixel_col = 8'h1C;
                3'd2: pixel_col = 8'h20;
                3'd3: pixel_col = 8'h40;
                3'd4: pixel_col = 8'h20;
                3'd5: pixel_col = 8'h1C;
                default: pixel_col = 8'h00;
            endcase
            8'h77: case (col_idx) // 'w'
                3'd1: pixel_col = 8'h3C;
                3'd2: pixel_col = 8'h40;
                3'd3: pixel_col = 8'h30;
                3'd4: pixel_col = 8'h40;
                3'd5: pixel_col = 8'h3C;
                default: pixel_col = 8'h00;
            endcase
            8'h78: case (col_idx) // 'x'
                3'd1: pixel_col = 8'h44;
                3'd2: pixel_col = 8'h28;
                3'd3: pixel_col = 8'h10;
                3'd4: pixel_col = 8'h28;
                3'd5: pixel_col = 8'h44;
                default: pixel_col = 8'h00;
            endcase
            8'h79: case (col_idx) // 'y'
                3'd1: pixel_col = 8'h1C;
                3'd2: pixel_col = 8'hA0;
                3'd3: pixel_col = 8'hA0;
                3'd4: pixel_col = 8'hA0;
                3'd5: pixel_col = 8'h7C;
                default: pixel_col = 8'h00;
            endcase
            8'h7A: case (col_idx) // 'z'
                3'd1: pixel_col = 8'h44;
                3'd2: pixel_col = 8'h64;
                3'd3: pixel_col = 8'h54;
                3'd4: pixel_col = 8'h4C;
                3'd5: pixel_col = 8'h44;
                default: pixel_col = 8'h00;
            endcase

            default: pixel_col = 8'h00;
        endcase
    end

endmodule
