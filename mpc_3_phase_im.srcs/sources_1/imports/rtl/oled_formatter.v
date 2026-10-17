`timescale 1ns / 1ps
// =============================================================================
// Module: oled_formatter
// Description: Real-time Telemetry Dashboard Formatter for Nexys Video OLED.
//              - Displays Autotare Countdown (5s) upon power-up/reset.
//              - Once calibration completes, provides 3 toggle-able pages cycled
//                via the center pushbutton (BTNC / Pin B22):
//                * Page 1: Speed, Status, Torque, DC-Bus Voltage
//                * Page 2: Stator Currents (Ia, Ib), Flux Magnitude, Fault Reason (NaN if none)
//                * Page 3: ADC Raw Offsets, Auto-Tare Calibration Result, Pk-Pk Current
// =============================================================================

module oled_formatter #(
    parameter DATA_WIDTH = 32,
    parameter FRAC_BITS  = 20
)(
    input  wire                          clk,
    input  wire                          rst_n,

    // Status and Calibration inputs
    input  wire                          cal_done,       // High when auto-tare calibration finishes
    input  wire [2:0]                    cal_countdown,  // 5..1 seconds remaining
    input  wire                          btnc_pulse,     // 1-cycle debounced pulse from center button
    input  wire [4:0]                    fsm_state,      // 17 = S_ERROR
    input  wire [2:0]                    opt_switch_state,// 0..7 active voltage vector
    input  wire                          enable,         // Run enable latch
    input  wire [1:0]                    fault_code,     // 0=NaN, 1=Overcurrent, 2=Premag, 3=Watchdog

    // Controller Telemetry inputs (Q12.20 format)
    input  wire signed [DATA_WIDTH-1:0]  speed_elec,     // Electrical speed in rad/s
    input  wire signed [DATA_WIDTH-1:0]  te_ref,         // Torque command in Nm
    input  wire [15:0]                   vdc_int,        // DC-link voltage in Volts (e.g. 311, 400, 540)
    input  wire signed [DATA_WIDTH-1:0]  ia,             // Phase A current in Amperes
    input  wire signed [DATA_WIDTH-1:0]  ib,             // Phase B current in Amperes
    input  wire signed [DATA_WIDTH-1:0]  psi_r_alpha,    // Rotor flux alpha in Wb
    input  wire signed [DATA_WIDTH-1:0]  psi_r_beta,     // Rotor flux beta in Wb
    input  wire signed [12:0]            adc_offset,     // Auto-tare calibrated offset (counts)
    input  wire [11:0]                   adc_data_ch0,   // Live raw ADC Ch0 reading (counts)
    input  wire [11:0]                   adc_data_ch1,   // Live raw ADC Ch1 reading (counts)

    // Text buffer interface to oled_controller (64 characters)
    input  wire [5:0]                    text_addr,
    output reg  [7:0]                    text_char,

    // Diagnostic page monitor output
    output reg  [1:0]                    active_page     // 0=Page1, 1=Page2, 2=Page3
);

    // -------------------------------------------------------------------------
    // Page Selection Logic (Cycled via BTNC only after calibration completes)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_page <= 2'd0;
        end else if (cal_done && btnc_pulse) begin
            if (active_page >= 2'd2)
                active_page <= 2'd0;
            else
                active_page <= active_page + 2'd1;
        end
    end

    // -------------------------------------------------------------------------
    // Phase A Peak-to-Peak Current Measurement (50 ms Rolling Window)
    // -------------------------------------------------------------------------
    reg [22:0] pk_cnt;
    reg signed [DATA_WIDTH-1:0] ia_max_cur, ia_min_cur;
    reg signed [DATA_WIDTH-1:0] ia_pk_pk; // Q12.20 Amperes

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pk_cnt     <= 23'd0;
            ia_max_cur <= -32'sd2147483647;
            ia_min_cur <= 32'sd2147483647;
            ia_pk_pk   <= 32'sd0;
        end else begin
            if (ia > ia_max_cur) ia_max_cur <= ia;
            if (ia < ia_min_cur) ia_min_cur <= ia;

            // 50 ms window at 100 MHz = 5,000,000 cycles
            if (pk_cnt >= 23'd5_000_000) begin
                pk_cnt     <= 23'd0;
                ia_pk_pk   <= (ia_max_cur > ia_min_cur) ? (ia_max_cur - ia_min_cur) : 32'sd0;
                ia_max_cur <= ia;
                ia_min_cur <= ia;
            end else begin
                pk_cnt <= pk_cnt + 23'd1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Rotor Flux Magnitude Estimation (Alpha-Max-Plus-Beta-Min, < 2.6% error)
    // |psi_r| = max(|a|, |b|) + (3/8)*min(|a|, |b|)
    // -------------------------------------------------------------------------
    wire signed [DATA_WIDTH-1:0] abs_pa = (psi_r_alpha < 0) ? -psi_r_alpha : psi_r_alpha;
    wire signed [DATA_WIDTH-1:0] abs_pb = (psi_r_beta < 0)  ? -psi_r_beta  : psi_r_beta;
    wire signed [DATA_WIDTH-1:0] max_p  = (abs_pa > abs_pb) ? abs_pa : abs_pb;
    wire signed [DATA_WIDTH-1:0] min_p  = (abs_pa > abs_pb) ? abs_pb : abs_pa;
    wire signed [DATA_WIDTH-1:0] flux_mag = max_p + ((min_p * 3) >>> 3);

    // -------------------------------------------------------------------------
    // Numeric to ASCII Helper Functions
    // -------------------------------------------------------------------------
    function [7:0] dec_char;
        input [3:0] val;
        begin
            dec_char = 8'h30 + {4'd0, val}; // '0'..'9'
        end
    endfunction

    // -------------------------------------------------------------------------
    // Dynamic Parameter Decoders (Updated at register stage)
    // -------------------------------------------------------------------------
    // 1. Speed (rad/s)
    wire [31:0] abs_spd = (speed_elec < 0) ? -speed_elec : speed_elec;
    wire [11:0] spd_int = abs_spd[31:20];
    wire [3:0]  spd_frac = ((abs_spd[19:0] * 10) >> 20); // 1 decimal place

    // 2. Torque (Nm)
    wire [31:0] abs_trq = (te_ref < 0) ? -te_ref : te_ref;
    wire [11:0] trq_int = abs_trq[31:20];
    wire [6:0]  trq_frac = ((abs_trq[19:0] * 100) >> 20); // 2 decimal places

    // 3. Current Ia (A)
    wire [31:0] abs_ia = (ia < 0) ? -ia : ia;
    wire [11:0] ia_int = abs_ia[31:20];
    wire [6:0]  ia_frac = ((abs_ia[19:0] * 100) >> 20);

    // 4. Current Ib (A)
    wire [31:0] abs_ib = (ib < 0) ? -ib : ib;
    wire [11:0] ib_int = abs_ib[31:20];
    wire [6:0]  ib_frac = ((abs_ib[19:0] * 100) >> 20);

    // 5. Flux magnitude (Wb)
    wire [11:0] flx_int = flux_mag[31:20];
    wire [9:0]  flx_frac = ((flux_mag[19:0] * 1000) >> 20); // 3 decimal places

    // 6. Peak-to-Peak Current (A)
    wire [11:0] pk_int = ia_pk_pk[31:20];
    wire [6:0]  pk_frac = ((ia_pk_pk[19:0] * 100) >> 20);

    // -------------------------------------------------------------------------
    // 64-Byte Display Buffer (4 Lines x 16 Characters)
    // -------------------------------------------------------------------------
    always @(*) begin
        if (!cal_done) begin
            // =================================================================
            // STARTUP COUNTDOWN SCREEN (First 5 Seconds)
            // =================================================================
            case (text_addr)
                // Line 0: " AUTOTARE WARMUP"
                6'd0:  text_char = " ";
                6'd1:  text_char = "A";
                6'd2:  text_char = "U";
                6'd3:  text_char = "T";
                6'd4:  text_char = "O";
                6'd5:  text_char = "T";
                6'd6:  text_char = "A";
                6'd7:  text_char = "R";
                6'd8:  text_char = "E";
                6'd9:  text_char = " ";
                6'd10: text_char = "W";
                6'd11: text_char = "A";
                6'd12: text_char = "R";
                6'd13: text_char = "M";
                6'd14: text_char = "U";
                6'd15: text_char = "P";

                // Line 1: " COUNTDOWN:   5s"
                6'd16: text_char = " ";
                6'd17: text_char = "C";
                6'd18: text_char = "O";
                6'd19: text_char = "U";
                6'd20: text_char = "N";
                6'd21: text_char = "T";
                6'd22: text_char = "D";
                6'd23: text_char = "O";
                6'd24: text_char = "W";
                6'd25: text_char = "N";
                6'd26: text_char = ":";
                6'd27: text_char = " ";
                6'd28: text_char = " ";
                6'd29: text_char = " ";
                6'd30: text_char = dec_char({1'b0, cal_countdown}); // 5 -> 4 -> 3 -> 2 -> 1
                6'd31: text_char = "s";

                // Line 2: " SENSORS ZEROING"
                6'd32: text_char = " ";
                6'd33: text_char = "S";
                6'd34: text_char = "E";
                6'd35: text_char = "N";
                6'd36: text_char = "S";
                6'd37: text_char = "O";
                6'd38: text_char = "R";
                6'd39: text_char = "S";
                6'd40: text_char = " ";
                6'd41: text_char = "Z";
                6'd42: text_char = "E";
                6'd43: text_char = "R";
                6'd44: text_char = "O";
                6'd45: text_char = "I";
                6'd46: text_char = "N";
                6'd47: text_char = "G";

                // Line 3: " PLEASE WAIT... "
                6'd48: text_char = " ";
                6'd49: text_char = "P";
                6'd50: text_char = "L";
                6'd51: text_char = "E";
                6'd52: text_char = "A";
                6'd53: text_char = "S";
                6'd54: text_char = "E";
                6'd55: text_char = " ";
                6'd56: text_char = "W";
                6'd57: text_char = "A";
                6'd58: text_char = "I";
                6'd59: text_char = "T";
                6'd60: text_char = ".";
                6'd61: text_char = ".";
                6'd62: text_char = ".";
                6'd63: text_char = " ";
                default: text_char = " ";
            endcase

        end else begin
            // =================================================================
            // OPERATIONAL DASHBOARD (3 Pages cycled via BTNC)
            // =================================================================
            case (active_page)

                // -------------------------------------------------------------
                // PAGE 1: Speed, Status, Torque, DC-Bus Voltage
                // -------------------------------------------------------------
                2'd0: case (text_addr)
                    // Line 0: Status ("STAT: RUN   [V4]" / "STAT: STOPPED   " / "STAT: FAULT TRIP")
                    6'd0:  text_char = "S";
                    6'd1:  text_char = "T";
                    6'd2:  text_char = "A";
                    6'd3:  text_char = "T";
                    6'd4:  text_char = ":";
                    6'd5:  text_char = " ";
                    6'd6:  text_char = (fsm_state == 5'd17) ? "F" : (!enable ? "S" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "P" : "R"));
                    6'd7:  text_char = (fsm_state == 5'd17) ? "A" : (!enable ? "T" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "R" : "U"));
                    6'd8:  text_char = (fsm_state == 5'd17) ? "U" : (!enable ? "O" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "E" : "N"));
                    6'd9:  text_char = (fsm_state == 5'd17) ? "L" : (!enable ? "P" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "-" : " "));
                    6'd10: text_char = (fsm_state == 5'd17) ? "T" : (!enable ? "P" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "M" : " "));
                    6'd11: text_char = (fsm_state == 5'd17) ? " " : (!enable ? "E" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "A" : "["));
                    6'd12: text_char = (fsm_state == 5'd17) ? "T" : (!enable ? "D" : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? "G" : "V"));
                    6'd13: text_char = (fsm_state == 5'd17) ? "R" : (!enable ? " " : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? " " : dec_char({1'b0, opt_switch_state})));
                    6'd14: text_char = (fsm_state == 5'd17) ? "I" : (!enable ? " " : (fsm_state >= 5'd23 && fsm_state <= 5'd27 ? " " : "]"));
                    6'd15: text_char = (fsm_state == 5'd17) ? "P" : " ";

                    // Line 1: Speed ("SPD: +100.0 R/S ")
                    6'd16: text_char = "S";
                    6'd17: text_char = "P";
                    6'd18: text_char = "D";
                    6'd19: text_char = ":";
                    6'd20: text_char = " ";
                    6'd21: text_char = (speed_elec < 0) ? "-" : "+";
                    6'd22: text_char = (spd_int >= 100) ? dec_char((spd_int / 100) % 10) : " ";
                    6'd23: text_char = (spd_int >= 10)  ? dec_char((spd_int / 10) % 10)  : " ";
                    6'd24: text_char = dec_char(spd_int % 10);
                    6'd25: text_char = ".";
                    6'd26: text_char = dec_char(spd_frac);
                    6'd27: text_char = " ";
                    6'd28: text_char = "R";
                    6'd29: text_char = "/";
                    6'd30: text_char = "S";
                    6'd31: text_char = " ";

                    // Line 2: Torque ("TRQ:  +5.21 N.M ")
                    6'd32: text_char = "T";
                    6'd33: text_char = "R";
                    6'd34: text_char = "Q";
                    6'd35: text_char = ":";
                    6'd36: text_char = " ";
                    6'd37: text_char = (te_ref < 0) ? "-" : "+";
                    6'd38: text_char = (trq_int >= 10) ? dec_char((trq_int / 10) % 10) : " ";
                    6'd39: text_char = dec_char(trq_int % 10);
                    6'd40: text_char = ".";
                    6'd41: text_char = dec_char((trq_frac / 10) % 10);
                    6'd42: text_char = dec_char(trq_frac % 10);
                    6'd43: text_char = " ";
                    6'd44: text_char = "N";
                    6'd45: text_char = ".";
                    6'd46: text_char = "M";
                    6'd47: text_char = " ";

                    // Line 3: DC-Bus Voltage ("VDC:   311 VOLTS")
                    6'd48: text_char = "V";
                    6'd49: text_char = "D";
                    6'd50: text_char = "C";
                    6'd51: text_char = ":";
                    6'd52: text_char = " ";
                    6'd53: text_char = (vdc_int >= 100) ? dec_char((vdc_int / 100) % 10) : " ";
                    6'd54: text_char = (vdc_int >= 10)  ? dec_char((vdc_int / 10) % 10)  : " ";
                    6'd55: text_char = dec_char(vdc_int % 10);
                    6'd56: text_char = " ";
                    6'd57: text_char = "V";
                    6'd58: text_char = "O";
                    6'd59: text_char = "L";
                    6'd60: text_char = "T";
                    6'd61: text_char = "S";
                    6'd62: text_char = " ";
                    6'd63: text_char = " ";
                    default: text_char = " ";
                endcase

                // -------------------------------------------------------------
                // PAGE 2: Stator Currents, Flux Magnitude, Reason of Fault
                // -------------------------------------------------------------
                2'd1: case (text_addr)
                    // Line 0: Current Ia ("IA:   +2.35 A   ")
                    6'd0:  text_char = "I";
                    6'd1:  text_char = "A";
                    6'd2:  text_char = ":";
                    6'd3:  text_char = " ";
                    6'd4:  text_char = (ia < 0) ? "-" : "+";
                    6'd5:  text_char = (ia_int >= 10) ? dec_char((ia_int / 10) % 10) : " ";
                    6'd6:  text_char = dec_char(ia_int % 10);
                    6'd7:  text_char = ".";
                    6'd8:  text_char = dec_char((ia_frac / 10) % 10);
                    6'd9:  text_char = dec_char(ia_frac % 10);
                    6'd10: text_char = " ";
                    6'd11: text_char = "A";
                    6'd12: text_char = " ";
                    6'd13: text_char = " ";
                    6'd14: text_char = " ";
                    6'd15: text_char = " ";

                    // Line 1: Current Ib ("IB:   -1.12 A   ")
                    6'd16: text_char = "I";
                    6'd17: text_char = "B";
                    6'd18: text_char = ":";
                    6'd19: text_char = " ";
                    6'd20: text_char = (ib < 0) ? "-" : "+";
                    6'd21: text_char = (ib_int >= 10) ? dec_char((ib_int / 10) % 10) : " ";
                    6'd22: text_char = dec_char(ib_int % 10);
                    6'd23: text_char = ".";
                    6'd24: text_char = dec_char((ib_frac / 10) % 10);
                    6'd25: text_char = dec_char(ib_frac % 10);
                    6'd26: text_char = " ";
                    6'd27: text_char = "A";
                    6'd28: text_char = " ";
                    6'd29: text_char = " ";
                    6'd30: text_char = " ";
                    6'd31: text_char = " ";

                    // Line 2: Flux Magnitude ("FLUX: 0.960 WB  ")
                    6'd32: text_char = "F";
                    6'd33: text_char = "L";
                    6'd34: text_char = "U";
                    6'd35: text_char = "X";
                    6'd36: text_char = ":";
                    6'd37: text_char = " ";
                    6'd38: text_char = dec_char(flx_int % 10);
                    6'd39: text_char = ".";
                    6'd40: text_char = dec_char((flx_frac / 100) % 10);
                    6'd41: text_char = dec_char((flx_frac / 10) % 10);
                    6'd42: text_char = dec_char(flx_frac % 10);
                    6'd43: text_char = " ";
                    6'd44: text_char = "W";
                    6'd45: text_char = "B";
                    6'd46: text_char = " ";
                    6'd47: text_char = " ";

                    // Line 3: Reason of Fault ("FLT:  NaN       " / "FLT: OVERCURRENT")
                    6'd48: text_char = "F";
                    6'd49: text_char = "L";
                    6'd50: text_char = "T";
                    6'd51: text_char = ":";
                    6'd52: text_char = " ";
                    6'd53: text_char = (fault_code == 2'b00) ? "N" : (fault_code == 2'b01 ? "O" : (fault_code == 2'b10 ? "P" : "W"));
                    6'd54: text_char = (fault_code == 2'b00) ? "a" : (fault_code == 2'b01 ? "V" : (fault_code == 2'b10 ? "R" : "A"));
                    6'd55: text_char = (fault_code == 2'b00) ? "N" : (fault_code == 2'b01 ? "E" : (fault_code == 2'b10 ? "E" : "T"));
                    6'd56: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "R" : (fault_code == 2'b10 ? "-" : "C"));
                    6'd57: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "C" : (fault_code == 2'b10 ? "M" : "H"));
                    6'd58: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "U" : (fault_code == 2'b10 ? "A" : "D"));
                    6'd59: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "R" : (fault_code == 2'b10 ? "G" : "O"));
                    6'd60: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "R" : (fault_code == 2'b10 ? " " : "G"));
                    6'd61: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "E" : (fault_code == 2'b10 ? "O" : " "));
                    6'd62: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "N" : (fault_code == 2'b10 ? "C" : " "));
                    6'd63: text_char = (fault_code == 2'b00) ? " " : (fault_code == 2'b01 ? "T" : (fault_code == 2'b10 ? " " : " "));
                    default: text_char = " ";
                endcase

                // -------------------------------------------------------------
                // PAGE 3: ADC Raw Offsets, Auto-Tare Results, Pk-Pk Current
                // -------------------------------------------------------------
                2'd2: case (text_addr)
                    // Line 0: Auto-Tare Offset ("TARE: 2048 CNTS ")
                    6'd0:  text_char = "T";
                    6'd1:  text_char = "A";
                    6'd2:  text_char = "R";
                    6'd3:  text_char = "E";
                    6'd4:  text_char = ":";
                    6'd5:  text_char = " ";
                    6'd6:  text_char = dec_char((adc_offset / 1000) % 10);
                    6'd7:  text_char = dec_char((adc_offset / 100) % 10);
                    6'd8:  text_char = dec_char((adc_offset / 10) % 10);
                    6'd9:  text_char = dec_char(adc_offset % 10);
                    6'd10: text_char = " ";
                    6'd11: text_char = "C";
                    6'd12: text_char = "N";
                    6'd13: text_char = "T";
                    6'd14: text_char = "S";
                    6'd15: text_char = " ";

                    // Line 1: ADC Channel 0 Raw ("CH0:  2051 CNTS ")
                    6'd16: text_char = "C";
                    6'd17: text_char = "H";
                    6'd18: text_char = "0";
                    6'd19: text_char = ":";
                    6'd20: text_char = " ";
                    6'd21: text_char = " ";
                    6'd22: text_char = dec_char((adc_data_ch0 / 1000) % 10);
                    6'd23: text_char = dec_char((adc_data_ch0 / 100) % 10);
                    6'd24: text_char = dec_char((adc_data_ch0 / 10) % 10);
                    6'd25: text_char = dec_char(adc_data_ch0 % 10);
                    6'd26: text_char = " ";
                    6'd27: text_char = "C";
                    6'd28: text_char = "N";
                    6'd29: text_char = "T";
                    6'd30: text_char = "S";
                    6'd31: text_char = " ";

                    // Line 2: ADC Channel 1 Raw ("CH1:  2046 CNTS ")
                    6'd32: text_char = "C";
                    6'd33: text_char = "H";
                    6'd34: text_char = "1";
                    6'd35: text_char = ":";
                    6'd36: text_char = " ";
                    6'd37: text_char = " ";
                    6'd38: text_char = dec_char((adc_data_ch1 / 1000) % 10);
                    6'd39: text_char = dec_char((adc_data_ch1 / 100) % 10);
                    6'd40: text_char = dec_char((adc_data_ch1 / 10) % 10);
                    6'd41: text_char = dec_char(adc_data_ch1 % 10);
                    6'd42: text_char = " ";
                    6'd43: text_char = "C";
                    6'd44: text_char = "N";
                    6'd45: text_char = "T";
                    6'd46: text_char = "S";
                    6'd47: text_char = " ";

                    // Line 3: Peak-to-Peak Current ("IPK-PK:  4.70 A ")
                    6'd48: text_char = "I";
                    6'd49: text_char = "P";
                    6'd50: text_char = "K";
                    6'd51: text_char = "-";
                    6'd52: text_char = "P";
                    6'd53: text_char = "K";
                    6'd54: text_char = ":";
                    6'd55: text_char = " ";
                    6'd56: text_char = (pk_int >= 10) ? dec_char((pk_int / 10) % 10) : " ";
                    6'd57: text_char = dec_char(pk_int % 10);
                    6'd58: text_char = ".";
                    6'd59: text_char = dec_char((pk_frac / 10) % 10);
                    6'd60: text_char = dec_char(pk_frac % 10);
                    6'd61: text_char = " ";
                    6'd62: text_char = "A";
                    6'd63: text_char = " ";
                    default: text_char = " ";
                endcase

                default: text_char = " ";
            endcase
        end
    end

endmodule
