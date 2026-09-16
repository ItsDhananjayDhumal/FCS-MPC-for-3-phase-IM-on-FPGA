`timescale 1ns / 1ps

`include "mpc_params.vh"

module adc_pmod_ad1 #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter FRAC_BITS = `FRAC_BITS
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,
    input  wire adc_d0,
    input  wire adc_d1,
    output reg  adc_cs_n,
    output reg  adc_sclk,
    output reg  [`ADC_BITS-1:0] data_ch0,
    output reg  [`ADC_BITS-1:0] data_ch1,
    output reg  done
);

    reg [1:0] d0_sync, d1_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d0_sync <= 2'b00;
            d1_sync <= 2'b00;
        end else begin
            d0_sync <= {d0_sync[0], adc_d0};
            d1_sync <= {d1_sync[0], adc_d1};
        end
    end
    wire d0_in = d0_sync[1];
    wire d1_in = d1_sync[1];

    reg [2:0] clk_div;
    reg sclk_en;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_div <= 3'd0;
            adc_sclk <= 1'b0;
        end else if (sclk_en) begin
            clk_div <= clk_div + 3'd1;
            if (clk_div == 3'd3) adc_sclk <= 1'b1;
            else if (clk_div == 3'd7) adc_sclk <= 1'b0;
        end else begin
            clk_div <= 3'd0;
            adc_sclk <= 1'b0;
        end
    end
    wire sclk_rise = (clk_div == 3'd3) && sclk_en;
    wire sclk_fall = (clk_div == 3'd7) && sclk_en;

    localparam IDLE = 2'd0;
    localparam RUN = 2'd1;
    localparam QUIET = 2'd2;
    reg [1:0] state;

    reg [15:0] shift_ch0, shift_ch1;
    reg [4:0] bit_cnt;
    reg [2:0] quiet_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            adc_cs_n <= 1'b1;
            sclk_en <= 1'b0;
            data_ch0 <= 0;
            data_ch1 <= 0;
            done <= 1'b0;
            bit_cnt <= 5'd0;
            quiet_cnt <= 3'd0;
            shift_ch0 <= 16'd0;
            shift_ch1 <= 16'd0;
        end else begin
            done <= 1'b0;
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= RUN;
                        adc_cs_n <= 1'b0;
                        sclk_en <= 1'b1;
                        bit_cnt <= 5'd0;
                    end
                end
                RUN: begin
                    if (sclk_rise) begin
                        shift_ch0 <= {shift_ch0[14:0], d0_in};
                        shift_ch1 <= {shift_ch1[14:0], d1_in};
                    end else if (sclk_fall) begin
                        bit_cnt <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd15) begin
                            state <= QUIET;
                            adc_cs_n <= 1'b1;
                            sclk_en <= 1'b0;
                            data_ch0 <= shift_ch0[11:0];
                            data_ch1 <= shift_ch1[11:0];
                            done <= 1'b1;
                            quiet_cnt <= 3'd0;
                        end
                    end
                end
                QUIET: begin
                    if (quiet_cnt == 3'd4) begin
                        state <= IDLE;
                    end else begin
                        quiet_cnt <= quiet_cnt + 3'd1;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end

endmodule
