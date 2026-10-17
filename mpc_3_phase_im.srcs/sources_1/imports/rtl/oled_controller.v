`timescale 1ns / 1ps
// =============================================================================
// Module: oled_controller
// Description: SSD1306 (128x32) OLED Controller with Power Sequencing & Refresh.
//              - Manages active-low power supplies: oled_vdd, oled_vbat, oled_res.
//              - Follows Univision UG-2832HSWEG04 power-up protocol.
//              - Reads 64-character text buffer (4 lines x 16 chars) via text_addr.
//              - Fetches 8x8 font columns from oled_font_rom.
//              - Continuously transmits page data over SPI at ~50 Hz refresh rate.
// =============================================================================

module oled_controller #(
    parameter SIMULATION = 1'b0
)(
    input  wire        clk,
    input  wire        rst_n,

    // Text buffer interface (4 lines x 16 characters = 64 characters)
    output reg  [5:0]  text_addr,     // Character index (0..63)
    input  wire [7:0]  text_char,     // ASCII code at text_addr

    // Status output
    output reg         init_done,     // High when OLED power-up and init complete

    // Physical OLED pins on Nexys Video
    output wire        oled_sclk,
    output wire        oled_sdin,
    output wire        oled_dc,
    output reg         oled_res,
    output reg         oled_vbat,
    output reg         oled_vdd
);

    // Timing parameters (at 100 MHz clock)
    localparam DELAY_20MS  = SIMULATION ? 24'd200  : 24'd2_000_000;
    localparam DELAY_1MS   = SIMULATION ? 24'd50   : 24'd100_000;
    localparam DELAY_100MS = SIMULATION ? 24'd500  : 24'd10_000_000;
    localparam DELAY_FRAME = SIMULATION ? 24'd200  : 24'd2_000_000; // ~20ms between frames (~50 Hz)

    // SPI transmitter instance
    reg        spi_start;
    reg        spi_dc;
    reg  [7:0] spi_byte;
    wire       spi_busy;
    wire       spi_done;

    oled_spi_tx u_spi (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (spi_start),
        .dc_in     (spi_dc),
        .byte_in   (spi_byte),
        .busy      (spi_busy),
        .done      (spi_done),
        .oled_sclk (oled_sclk),
        .oled_sdin (oled_sdin),
        .oled_dc   (oled_dc)
    );

    // Font ROM instance
    reg  [7:0] font_ascii;
    reg  [2:0] font_col;
    wire [7:0] font_pixel_col;

    oled_font_rom u_font (
        .ascii_code (font_ascii),
        .col_idx    (font_col),
        .pixel_col  (font_pixel_col)
    );

    // Total init commands = 24
    localparam NUM_INIT_CMDS = 24;
    function [7:0] get_init_cmd;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  get_init_cmd = 8'hAE; // Display OFF
                5'd1:  get_init_cmd = 8'hD5; // Set Display Clock Divide
                5'd2:  get_init_cmd = 8'h80; //   - Ratio: 0x80
                5'd3:  get_init_cmd = 8'hA8; // Set Multiplex Ratio
                5'd4:  get_init_cmd = 8'h1F; //   - 31 (for 128x32 display)
                5'd5:  get_init_cmd = 8'hD3; // Set Display Offset
                5'd6:  get_init_cmd = 8'h00; //   - 0
                5'd7:  get_init_cmd = 8'h40; // Set Display Start Line = 0
                5'd8:  get_init_cmd = 8'h8D; // Set Charge Pump
                5'd9:  get_init_cmd = 8'h14; //   - Enable internal charge pump (0x14)
                5'd10: get_init_cmd = 8'h20; // Set Memory Addressing Mode
                5'd11: get_init_cmd = 8'h02; //   - Page Addressing Mode (0x02)
                5'd12: get_init_cmd = 8'hA1; // Set Segment Re-map (COL127 -> SEG0)
                5'd13: get_init_cmd = 8'hC8; // Set COM Output Scan Direction (COM[N-1] -> COM0)
                5'd14: get_init_cmd = 8'hDA; // Set COM Pins Hardware Config
                5'd15: get_init_cmd = 8'h02; //   - 0x02 for 128x32
                5'd16: get_init_cmd = 8'h81; // Set Contrast Control
                5'd17: get_init_cmd = 8'h8F; //   - Contrast: 0x8F
                5'd18: get_init_cmd = 8'hD9; // Set Pre-charge Period
                5'd19: get_init_cmd = 8'hF1; //   - 0xF1
                5'd20: get_init_cmd = 8'hDB; // Set VCOMH Deselect Level
                5'd21: get_init_cmd = 8'h40; //   - 0x40
                5'd22: get_init_cmd = 8'hA4; // Entire Display ON Resume
                5'd23: get_init_cmd = 8'hA6; // Normal Display (non-inverted)
                default: get_init_cmd = 8'h00;
            endcase
        end
    endfunction

    // Controller FSM states
    localparam S_PWR_OFF      = 5'd0,
               S_PWR_VDD      = 5'd1,
               S_RES_LOW      = 5'd2,
               S_RES_HIGH     = 5'd3,
               S_INIT_CMD_TX  = 5'd4,
               S_INIT_CMD_WAIT= 5'd5,
               S_CLR_PG_CMD0  = 5'd6,
               S_CLR_PG_CMD1  = 5'd7,
               S_CLR_PG_CMD2  = 5'd8,
               S_CLR_DATA_TX  = 5'd9,
               S_CLR_DATA_WAIT= 5'd10,
               S_PWR_VBAT     = 5'd11,
               S_DISP_ON_TX   = 5'd12,
               S_DISP_ON_WAIT = 5'd13,
               // Refresh Loop States
               S_REF_PG_CMD0  = 5'd14,
               S_REF_PG_CMD1  = 5'd15,
               S_REF_PG_CMD2  = 5'd16,
               S_REF_FETCH    = 5'd17,
               S_REF_DATA_TX  = 5'd18,
               S_REF_DATA_WAIT= 5'd19,
               S_FRAME_DELAY  = 5'd20;

    reg [4:0]  state;
    reg [23:0] delay_cnt;
    reg [4:0]  init_cmd_idx;
    reg [1:0]  page_idx;      // 0..3 (4 pages for 32 vertical rows)
    reg [6:0]  col_counter;   // 0..127 columns per page
    reg [3:0]  char_col_cnt;  // 0..15 chars per page
    reg [2:0]  pixel_col_cnt; // 0..7 cols per char

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_PWR_OFF;
            delay_cnt     <= 24'd0;
            init_cmd_idx  <= 5'd0;
            page_idx      <= 2'd0;
            col_counter   <= 7'd0;
            char_col_cnt  <= 4'd0;
            pixel_col_cnt <= 3'd0;
            text_addr     <= 6'd0;
            font_ascii    <= 8'h20;
            font_col      <= 3'd0;
            spi_start     <= 1'b0;
            spi_dc        <= 1'b0;
            spi_byte      <= 8'h00;
            init_done     <= 1'b0;
            oled_vdd      <= 1'b1; // Logic Power OFF (active-low)
            oled_vbat     <= 1'b1; // Panel Power OFF (active-low)
            oled_res      <= 1'b0; // Reset active
        end else begin
            spi_start <= 1'b0; // Default pulse

            case (state)
                // 1. Initial State: Keep all rails off
                S_PWR_OFF: begin
                    oled_vdd  <= 1'b1;
                    oled_vbat <= 1'b1;
                    oled_res  <= 1'b0;
                    init_done <= 1'b0;
                    delay_cnt <= 24'd0;
                    state     <= S_PWR_VDD;
                end

                // 2. Turn on VDD and wait 20ms
                S_PWR_VDD: begin
                    oled_vdd <= 1'b0; // VDD ON
                    if (delay_cnt >= DELAY_20MS) begin
                        delay_cnt <= 24'd0;
                        oled_res  <= 1'b0; // Hold reset
                        state     <= S_RES_LOW;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // 3. Pulse reset low for 1ms
                S_RES_LOW: begin
                    if (delay_cnt >= DELAY_1MS) begin
                        delay_cnt <= 24'd0;
                        oled_res  <= 1'b1; // Release reset
                        state     <= S_RES_HIGH;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // 4. Wait 1ms after reset release
                S_RES_HIGH: begin
                    if (delay_cnt >= DELAY_1MS) begin
                        delay_cnt    <= 24'd0;
                        init_cmd_idx <= 5'd0;
                        state        <= S_INIT_CMD_TX;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // 5. Transmit SSD1306 Initialization Commands
                S_INIT_CMD_TX: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= get_init_cmd(init_cmd_idx);
                        state     <= S_INIT_CMD_WAIT;
                    end
                end

                S_INIT_CMD_WAIT: begin
                    if (spi_done) begin
                        if (init_cmd_idx >= NUM_INIT_CMDS - 1) begin
                            // Commands sent, now clear screen RAM
                            page_idx    <= 2'd0;
                            col_counter <= 7'd0;
                            state       <= S_CLR_PG_CMD0;
                        end else begin
                            init_cmd_idx <= init_cmd_idx + 5'd1;
                            state        <= S_INIT_CMD_TX;
                        end
                    end
                end

                // 6. Clear Screen RAM (write 0x00 to all 4 pages x 128 cols)
                S_CLR_PG_CMD0: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'hB0 | {6'd0, page_idx}; // Set Page Address (0xB0..0xB3)
                        state     <= S_CLR_PG_CMD1;
                    end
                end

                S_CLR_PG_CMD1: begin
                    if (spi_done) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'h00; // Lower column address 0
                        state     <= S_CLR_PG_CMD2;
                    end
                end

                S_CLR_PG_CMD2: begin
                    if (spi_done) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'h10; // Upper column address 0
                        col_counter <= 7'd0;
                        state     <= S_CLR_DATA_TX;
                    end
                end

                S_CLR_DATA_TX: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b1; // Display Data
                        spi_byte  <= 8'h00; // Zero
                        state     <= S_CLR_DATA_WAIT;
                    end
                end

                S_CLR_DATA_WAIT: begin
                    if (spi_done) begin
                        if (col_counter >= 7'd127) begin
                            if (page_idx >= 2'd3) begin
                                // All 4 pages cleared, now power on VBAT!
                                delay_cnt <= 24'd0;
                                state     <= S_PWR_VBAT;
                            end else begin
                                page_idx <= page_idx + 2'd1;
                                state    <= S_CLR_PG_CMD0;
                            end
                        end else begin
                            col_counter <= col_counter + 7'd1;
                            state       <= S_CLR_DATA_TX;
                        end
                    end
                end

                // 7. Power on VBAT and wait 100ms
                S_PWR_VBAT: begin
                    oled_vbat <= 1'b0; // VBAT ON
                    if (delay_cnt >= DELAY_100MS) begin
                        delay_cnt <= 24'd0;
                        state     <= S_DISP_ON_TX;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                // 8. Turn Display ON (0xAF)
                S_DISP_ON_TX: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'hAF; // Display ON
                        state     <= S_DISP_ON_WAIT;
                    end
                end

                S_DISP_ON_WAIT: begin
                    if (spi_done) begin
                        init_done <= 1'b1;
                        page_idx  <= 2'd0;
                        state     <= S_REF_PG_CMD0; // Begin continuous refresh loop!
                    end
                end

                // =============================================================
                // Continuous Display Refresh Loop
                // =============================================================

                // Set Page Address (0xB0, 0xB1, 0xB2, 0xB3)
                S_REF_PG_CMD0: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'hB0 | {6'd0, page_idx};
                        state     <= S_REF_PG_CMD1;
                    end
                end

                // Set Lower Column Address (0x00)
                S_REF_PG_CMD1: begin
                    if (spi_done) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b0; // Command
                        spi_byte  <= 8'h00;
                        state     <= S_REF_PG_CMD2;
                    end
                end

                // Set Higher Column Address (0x10)
                S_REF_PG_CMD2: begin
                    if (spi_done) begin
                        spi_start     <= 1'b1;
                        spi_dc        <= 1'b0; // Command
                        spi_byte      <= 8'h10;
                        char_col_cnt  <= 4'd0;
                        pixel_col_cnt <= 3'd0;
                        text_addr     <= {page_idx, 4'd0}; // First char of page
                        state         <= S_REF_FETCH;
                    end
                end

                // Fetch character from text buffer and prepare font column
                S_REF_FETCH: begin
                    font_ascii <= text_char;
                    font_col   <= pixel_col_cnt;
                    state      <= S_REF_DATA_TX;
                end

                // Transmit Pixel Column Data
                S_REF_DATA_TX: begin
                    if (!spi_busy) begin
                        spi_start <= 1'b1;
                        spi_dc    <= 1'b1; // Display Data
                        spi_byte  <= font_pixel_col;
                        state     <= S_REF_DATA_WAIT;
                    end
                end

                S_REF_DATA_WAIT: begin
                    if (spi_done) begin
                        if (pixel_col_cnt >= 3'd7) begin
                            // Finished this character (8 columns)
                            pixel_col_cnt <= 3'd0;
                            if (char_col_cnt >= 4'd15) begin
                                // Finished this page (16 characters = 128 columns)
                                char_col_cnt <= 4'd0;
                                if (page_idx >= 2'd3) begin
                                    // Finished all 4 pages! Wait for next frame
                                    delay_cnt <= 24'd0;
                                    state     <= S_FRAME_DELAY;
                                end else begin
                                    page_idx <= page_idx + 2'd1;
                                    state    <= S_REF_PG_CMD0;
                                end
                            end else begin
                                char_col_cnt <= char_col_cnt + 4'd1;
                                text_addr    <= {page_idx, char_col_cnt + 4'd1};
                                state        <= S_REF_FETCH;
                            end
                        end else begin
                            pixel_col_cnt <= pixel_col_cnt + 3'd1;
                            font_col      <= pixel_col_cnt + 3'd1;
                            state         <= S_REF_DATA_TX;
                        end
                    end
                end

                // Delay between frames (~20 ms for ~50 FPS refresh rate)
                S_FRAME_DELAY: begin
                    if (delay_cnt >= DELAY_FRAME) begin
                        delay_cnt <= 24'd0;
                        page_idx  <= 2'd0;
                        state     <= S_REF_PG_CMD0;
                    end else begin
                        delay_cnt <= delay_cnt + 24'd1;
                    end
                end

                default: state <= S_PWR_OFF;
            endcase
        end
    end

endmodule
