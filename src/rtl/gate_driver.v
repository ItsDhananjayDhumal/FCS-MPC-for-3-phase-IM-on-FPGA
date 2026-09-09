`timescale 1ns / 1ps

`include "mpc_params.vh"

module gate_driver (
    input  wire clk,
    input  wire rst_n,
    input  wire enable,
    input  wire [2:0] switch_state,
    input  wire update_tick,
    output reg  gate_ah,
    output reg  gate_al,
    output reg  gate_bh,
    output reg  gate_bl,
    output reg  gate_ch,
    output reg  gate_cl
);

    reg [2:0] current_state;
    reg [2:0] target_state;
    reg [2:0] dead_time_active;
    reg [11:0] dt_cnt_a, dt_cnt_b, dt_cnt_c;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            target_state <= 3'b000;
        end else if (update_tick) begin
            target_state <= switch_state;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gate_ah <= 1'b0; gate_al <= 1'b0;
            gate_bh <= 1'b0; gate_bl <= 1'b0;
            gate_ch <= 1'b0; gate_cl <= 1'b0;
            current_state <= 3'b000;
            dead_time_active <= 3'b000;
            dt_cnt_a <= 12'd0;
            dt_cnt_b <= 12'd0;
            dt_cnt_c <= 12'd0;
        end else if (!enable) begin
            gate_ah <= 1'b0; gate_al <= 1'b0;
            gate_bh <= 1'b0; gate_bl <= 1'b0;
            gate_ch <= 1'b0; gate_cl <= 1'b0;
            current_state <= 3'b000;
            dead_time_active <= 3'b000;
            dt_cnt_a <= 12'd0;
            dt_cnt_b <= 12'd0;
            dt_cnt_c <= 12'd0;
        end else begin
            
            if (target_state[2] != current_state[2]) begin
                if (!dead_time_active[2]) begin
                    gate_ah <= 1'b0;
                    gate_al <= 1'b0;
                    dead_time_active[2] <= 1'b1;
                    dt_cnt_a <= 12'd0;
                end else begin
                    if (dt_cnt_a == DEAD_TIME_CYCLES) begin
                        current_state[2] <= target_state[2];
                        dead_time_active[2] <= 1'b0;
                        gate_ah <= target_state[2];
                        gate_al <= ~target_state[2];
                    end else begin
                        dt_cnt_a <= dt_cnt_a + 12'd1;
                    end
                end
            end else if (!dead_time_active[2]) begin
                gate_ah <= current_state[2];
                gate_al <= ~current_state[2];
            end

            if (target_state[1] != current_state[1]) begin
                if (!dead_time_active[1]) begin
                    gate_bh <= 1'b0;
                    gate_bl <= 1'b0;
                    dead_time_active[1] <= 1'b1;
                    dt_cnt_b <= 12'd0;
                end else begin
                    if (dt_cnt_b == DEAD_TIME_CYCLES) begin
                        current_state[1] <= target_state[1];
                        dead_time_active[1] <= 1'b0;
                        gate_bh <= target_state[1];
                        gate_bl <= ~target_state[1];
                    end else begin
                        dt_cnt_b <= dt_cnt_b + 12'd1;
                    end
                end
            end else if (!dead_time_active[1]) begin
                gate_bh <= current_state[1];
                gate_bl <= ~current_state[1];
            end

            if (target_state[0] != current_state[0]) begin
                if (!dead_time_active[0]) begin
                    gate_ch <= 1'b0;
                    gate_cl <= 1'b0;
                    dead_time_active[0] <= 1'b1;
                    dt_cnt_c <= 12'd0;
                end else begin
                    if (dt_cnt_c == DEAD_TIME_CYCLES) begin
                        current_state[0] <= target_state[0];
                        dead_time_active[0] <= 1'b0;
                        gate_ch <= target_state[0];
                        gate_cl <= ~target_state[0];
                    end else begin
                        dt_cnt_c <= dt_cnt_c + 12'd1;
                    end
                end
            end else if (!dead_time_active[0]) begin
                gate_ch <= current_state[0];
                gate_cl <= ~current_state[0];
            end
        end
    end

endmodule
