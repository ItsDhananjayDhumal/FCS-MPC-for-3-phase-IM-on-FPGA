`timescale 1ns / 1ps
// =============================================================================
// Module: oled_spi_tx
// Description: Low-Level 4-Wire SPI Byte Transmitter for SSD1306 OLED.
//              Operates at 6.25 MHz SPI Clock (100 MHz sys_clk / 16).
//              Transmits MSB first, setup on SCLK falling edge,
//              SSD1306 samples on SCLK rising edge.
// =============================================================================

module oled_spi_tx #(
    parameter CLK_DIV = 16  // 100 MHz / 16 = 6.25 MHz SPI clock (half-period = 8)
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       start,
    input  wire       dc_in,     // 0 = Command, 1 = Display Data
    input  wire [7:0] byte_in,

    output reg        busy,
    output reg        done,
    output reg        oled_sclk,
    output reg        oled_sdin,
    output reg        oled_dc
);

    localparam HALF_PERIOD = CLK_DIV / 2; // 8 cycles

    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [3:0] clk_cnt;
    reg [1:0] state;

    localparam S_IDLE = 2'd0,
               S_LOW  = 2'd1,
               S_HIGH = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy      <= 1'b0;
            done      <= 1'b0;
            oled_sclk <= 1'b0;
            oled_sdin <= 1'b0;
            oled_dc   <= 1'b0;
            shift_reg <= 8'h00;
            bit_cnt   <= 3'd0;
            clk_cnt   <= 4'd0;
            state     <= S_IDLE;
        end else begin
            done <= 1'b0; // default 1-cycle pulse

            case (state)
                S_IDLE: begin
                    oled_sclk <= 1'b0;
                    if (start) begin
                        busy      <= 1'b1;
                        oled_dc   <= dc_in;
                        shift_reg <= byte_in;
                        oled_sdin <= byte_in[7]; // Setup MSB
                        bit_cnt   <= 3'd0;
                        clk_cnt   <= 4'd0;
                        state     <= S_LOW;
                    end else begin
                        busy <= 1'b0;
                    end
                end

                S_LOW: begin
                    oled_sclk <= 1'b0;
                    if (clk_cnt >= HALF_PERIOD - 1) begin
                        clk_cnt   <= 4'd0;
                        oled_sclk <= 1'b1; // Rising edge -> SSD1306 samples
                        state     <= S_HIGH;
                    end else begin
                        clk_cnt <= clk_cnt + 4'd1;
                    end
                end

                S_HIGH: begin
                    oled_sclk <= 1'b1;
                    if (clk_cnt >= HALF_PERIOD - 1) begin
                        clk_cnt   <= 4'd0;
                        oled_sclk <= 1'b0; // Falling edge
                        if (bit_cnt == 3'd7) begin
                            // Completed all 8 bits
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            bit_cnt   <= bit_cnt + 3'd1;
                            shift_reg <= {shift_reg[6:0], 1'b0};
                            oled_sdin <= shift_reg[6]; // Setup next bit
                            state     <= S_LOW;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 4'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
