# CODE OVERVIEW

---

## 1. `mpc_params.vh` (Global Parameters)

```verilog
1: //============================================================================
2: // MPC Parameters — Global Configuration Header
3: // FCS-MPC for 3-Phase Induction Motor
4: // Target: Digilent Nexys Video (Artix-7 XC7A200T)
5: //============================================================================
6: `ifndef MPC_PARAMS_VH
7: `define MPC_PARAMS_VH
```
* **L1-L5**: Standard header comments describing the file.
* **L6-L7**: Include guards. `ifndef` checks if `MPC_PARAMS_VH` is not defined yet. If not, it defines it on L7. This prevents the compiler from throwing errors if this file is included multiple times by different modules.

```verilog
10: //----------------------------------------------------------------------------
11: // Fixed-Point Arithmetic
12: //----------------------------------------------------------------------------
13: parameter DATA_WIDTH = 32;              // Total fixed-point word width
14: parameter FRAC_BITS  = 20;              // Fractional bits → Q12.20 format
```
* **L13-L14**: Defines the global fixed-point architecture. Every math variable in this project is 32 bits wide, with 20 bits reserved for the decimal (fractional) part. This means the integer part is 11 bits, plus 1 sign bit.

```verilog
21: parameter SYS_CLK_FREQ   = 100_000_000;  // 100 MHz system clock (Hz)
22: parameter SWITCHING_FREQ  = 10_000;       // 10 kHz switching / sampling (Hz)
23: parameter TS_COUNTER_MAX  = SYS_CLK_FREQ / SWITCHING_FREQ;  // 10,000 cycles
```
* **L21**: The physical clock speed of the FPGA oscillator (100 MHz).
* **L22**: The target MPC loop execution rate (10 kHz, which means $T_s = 100 \mu s$).
* **L23**: Calculates how many clock cycles exist in one $T_s$ period. 100,000,000 / 10,000 = 10,000 clock cycles. The FSM will count to this number to trigger the next loop.

```verilog
28: parameter DEAD_TIME_NS     = 2000;        // 2.0 μs dead time
29: parameter DEAD_TIME_CYCLES = SYS_CLK_FREQ / (1_000_000_000 / DEAD_TIME_NS);
```
* **L28**: Desired dead-time for the inverter switches in nanoseconds (2000 ns = 2 $\mu s$).
* **L29**: Converts nanoseconds to FPGA clock cycles. `1e9 / 2000 = 500,000`. `100MHz / 500k = 200 cycles`. The gate driver will wait 200 cycles before turning on a complementary transistor.

```verilog
47: parameter ENCODER_PPR = 2500;             // Pulses per revolution
48: parameter ENCODER_CPR = ENCODER_PPR * 4;  // 10,000 counts/rev (4× quadrature)
49: parameter NUM_POLES   = 4;
50: parameter ENCODER_Z_RESET = 1'b1;         // Enable Z-pulse reset
```
* **L47-L48**: Physical properties of the optical encoder. 2500 lines on the disk translates to 10,000 edge transitions (counts) per revolution in quadrature mode.
* **L49-L50**: The motor has 4 poles. `ENCODER_Z_RESET` is a boolean flag (1 bit) that tells the encoder module to reset its position counter to 0 whenever the physical Z-index pulse goes high.

```verilog
56: parameter signed [DATA_WIDTH-1:0] SPEED_SCALE = 32'sd13176795;
```
* **L56**: Pre-calculated conversion factor. To convert raw encoder delta-counts into electrical speed ($\omega_e$ in rad/s) in Q12.20 format. `32'sd` means 32-bit signed decimal. The value $13176795$ is $12.566 \times 2^{20}$.

```verilog
89: // c₁₁ = 1 + Ts·a₁₁ = 0.981613   → round(× 2^20) = 1029293
90: parameter signed [DATA_WIDTH-1:0] C11 = 32'sd1029293;
```
* **L89-L90**: (And lines 91-105 for C12, C13, D1, E21, E22). These are the discretized Euler constants for the motor model. $0.981613 \times 2^{20} = 1029293$. By pre-calculating these, the FPGA doesn't need to do any floating-point division at runtime.

---

## 2. `motor_predictor.v` (FSM Pipeline details)

```verilog
4: module motor_predictor #(
5:     parameter DATA_WIDTH = 32,
6:     parameter FRAC_BITS = 20
7: )(
8:     input  wire clk,
9:     input  wire rst_n,
10:    input  wire start,
...
22: );
```
* **L4-L22**: Module declaration. Takes the system clock, active-low reset, a `start` trigger from the main FSM, and inputs for currents, voltages, and fluxes. Outputs the predicted next-step states.

```verilog
36:     // Registered multiplier inference
37:     reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
38:     reg signed [2*DATA_WIDTH-1:0] mul_result_full;
39:     wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
40: 
41:     always @(posedge clk) begin
42:         mul_result_full <= mul_a * mul_b;
43:     end
```
* **L36-L43**: This is the heart of the DSP optimization. Instead of doing 14 different multiplications, we declare ONE set of multiplier inputs (`mul_a`, `mul_b`). 
* **L41-L43**: On every clock cycle, `mul_result_full` (64-bits) becomes the product of A and B. Because it is inside an `always @(posedge clk)` block, it takes **1 full clock cycle** for the result to appear.
* **L39**: The 64-bit result is immediately shifted back into Q12.20 format combinationally by slicing the bits from `[51:20]`.

```verilog
59:             if (!busy) begin
60:                 if (start) begin
61:                     busy <= 1;
62:                     step <= 0;
63:                     mul_a <= C11;
64:                     mul_b <= i_alpha;
65:                 end
```
* **L59-L65**: When the FSM is idle (`!busy`), it waits for `start`. When `start` goes high, it enters the busy state, starts at step 0, and loads the very first mathematical operation into the multiplier: $C_{11} \times i_{\alpha}$.

```verilog
74:                 case (step)
75:                     0: begin
76:                         // Waiting for first multiply to register (C11 * i_alpha)
77:                     end
```
* **L75-L77**: In Step 0, we do absolutely nothing. Why? Because the inputs were set on the previous clock cycle, and the flip-flops inside the FPGA's DSP slice are currently calculating the result.

```verilog
78:                     1: begin
79:                         t1 <= mul_result;
80:                         mul_a <= C12;
81:                         mul_b <= psi_r_alpha;
82:                     end
```
* **L78-L82**: Now the product is ready. We save `mul_result` into a temporary holding register `t1`. AT THE SAME TIME, we load the inputs for the *next* multiplication ($C_{12} \times \psi_{r\alpha}$). 

```verilog
83:                     2: begin
84:                         // Wait for C12 * psi_r_alpha
85:                     end
86:                     3: begin
87:                         t2 <= mul_result;
88:                         mul_a <= C13;
89:                         mul_b <= wr_psi_beta;
90:                     end
```
* **L83-L90**: The pattern repeats. Step 2 is a dead cycle to let the DSP compute. Step 3 saves the result to `t2`, and loads the next equation.

```verilog
98:                     7: begin
99:                         t4 <= mul_result;
100:                        is_alpha_pred <= t1 + t2 + t3 + mul_result;
101:                        mul_a <= C11;
102:                        mul_b <= i_beta;
103:                    end
```
* **L98-L103**: By Step 7, we have accumulated all four terms of the $i_{s\alpha}$ prediction equation. We add them all together combinationally and assign them to the output register `is_alpha_pred`. We then immediately start computing the terms for the $i_{s\beta}$ equation.
* *Note*: This staggered wait-state pipeline continues all the way to Step 27 until all 4 predictive equations are solved.

---

## 3. `control_fsm.v` (Main Orchestrator)

```verilog
81:     reg [15:0] ts_counter;
82:     wire ts_tick = (ts_counter == TS_COUNTER_MAX - 1);
83: 
84:     always @(posedge clk or negedge rst_n) begin
85:         if (!rst_n)
86:             ts_counter <= 16'd0;
87:         else if (ts_counter >= TS_COUNTER_MAX - 1)
88:             ts_counter <= 16'd0;
89:         else
90:             ts_counter <= ts_counter + 16'd1;
91:     end
```
* **L81-L91**: This is a free-running timer. It counts from 0 to 9,999. When it hits 9999, `ts_tick` goes high for exactly one clock cycle, and the counter resets. This guarantees our MPC loop fires at exactly 10 kHz regardless of how long the math takes.

```verilog
134:     always @(posedge clk or negedge rst_n) begin
135:         if (!rst_n) begin
136:             state            <= S_IDLE;
...
151:         end else begin
152:             // Default: all pulses are single-cycle
153:             adc_start      <= 1'b0;
154:             clarke_start   <= 1'b0;
```
* **L134-L154**: Standard sequential logic block. On reset, all output triggers are forced to 0. Inside the `else` block (normal operation), we set the default assignments. By defaulting `adc_start <= 0`, we ensure that any time we set it to `1` inside a case statement, it automatically acts as a 1-cycle pulse, saving us from writing logic to manually pull it back down.

```verilog
181:                 S_WAIT_TS: begin
182:                     if (!enable) begin
183:                         state <= S_DISABLED;
184:                     end else if (ts_tick) begin
185:                         sample_tick <= 1'b1; // Trigger encoder speed calc
186:                         state <= S_START_ADC;
187:                     end
188:                 end
```
* **L181-L188**: The FSM sleeps here. If the user flips the physical enable switch off, it goes to `S_DISABLED`. Otherwise, it waits for the `ts_tick` (the 10kHz timer) to pulse. When it does, it fires `sample_tick` (to tell the PI controller and Encoder to update) and moves to start the ADC.

```verilog
197:                 S_WAIT_ADC: begin
198:                     if (adc_done) begin
199:                         clarke_ia_raw <= adc_data_ch0;
200:                         clarke_ib_raw <= adc_data_ch1;
201:                         
202:                         // Overcurrent / hardware fault check
203:                         if (adc_data_ch0 > MAX_CURRENT_RAW || adc_data_ch0 < MIN_CURRENT_RAW ||
204:                             adc_data_ch1 > MAX_CURRENT_RAW || adc_data_ch1 < MIN_CURRENT_RAW) begin
205:                             state <= S_ERROR; // Trip overcurrent!
206:                         end else begin
207:                             state <= S_CLARKE;
208:                         end
209:                     end
210:                 end
```
* **L197-L210**: Once the SPI ADC finishes reading the physical sensors, `adc_done` pulses. The FSM immediately latches the raw 12-bit data.
* **L203-L206**: This is the hardware safety trip. It looks at the raw 12-bit integer. If the value is outside safe bounds (e.g., > 3800 or < 200, where 2048 is 0 Amps), it immediately branches to `S_ERROR` to protect the inverter from blowing up.

```verilog
238:                 S_MPC_INIT: begin
239:                     // Reset the optimal selector for new search
240:                     sel_reset <= 1'b1;
241:                     vec_idx   <= 3'd0;
242:                     state     <= S_MPC_PREDICT;
243:                 end
```
* **L238-L243**: Prepares for the 8-vector loop. It resets the `optimal_selector`'s running minimum cost, and sets the current vector index (`vec_idx`) to 0 (Vector 0).

```verilog
271:                 S_WAIT_SEL: begin
272:                     state <= S_MPC_NEXT;
273:                 end
274:                 
275:                 S_MPC_NEXT: begin
276:                     if (sel_done) begin
277:                         // All 8 vectors evaluated
278:                         state <= S_APPLY;
279:                     end else begin
280:                         vec_idx <= vec_idx + 3'd1;
281:                         state   <= S_MPC_PREDICT;
282:                     end
283:                 end
```
* **L271-L273**: This is the race-condition fix we implemented earlier! Because the optimal selector module requires 1 clock cycle to process the `cost_valid` pulse and potentially assert `sel_done`, this dummy state wastes 1 clock cycle.
* **L275-L283**: In `S_MPC_NEXT`, we check if the selector said it was done (which happens when `vec_idx` hits 7). If it's done, we apply the result. If not, we increment `vec_idx` by 1, and go all the way back to `S_MPC_PREDICT` to evaluate the next voltage vector.

```verilog
297:         end else if (ts_counter == 16'd9900 && state != S_WAIT_TS && state != S_IDLE && state != S_DISABLED && state != S_ERROR) begin
298:             state <= S_ERROR;
299:         end else begin
```
* **L297-L299**: This is the watchdog timer. `ts_counter` runs from 0 to 9999. If it reaches 9900 (meaning 99% of our 100 $\mu s$ time budget is gone), and the FSM is still churning through math states instead of sleeping in `S_WAIT_TS`, it means a mathematical module hung or failed to pulse its `done` flag. The system forces itself into `S_ERROR` to prevent catastrophic mis-timing on the gate drivers.

---

## 4. `adc_pmod_ad1.v` (Dual SPI ADC Interface)

This module talks to the Digilent Pmod AD1 (which contains two AD7476A 12-bit ADCs sharing a clock and chip select). 

```verilog
5: module adc_pmod_ad1 (
6:     input  wire clk,
7:     input  wire rst_n,
8:     input  wire start,
9:     input  wire adc_d0,
10:    input  wire adc_d1,
11:    output reg  adc_cs_n,
12:    output reg  adc_sclk,
13:    output reg  [ADC_BITS-1:0] data_ch0,
14:    output reg  [ADC_BITS-1:0] data_ch1,
15:    output reg  done
16: );
```
* **L5-L16**: The module definition. It takes the main 100MHz `clk`, an asynchronous reset, and a `start` pulse. Physical pins are `adc_d0`, `adc_d1` (data in), `adc_cs_n` (Chip Select, active low), and `adc_sclk` (SPI Clock). It outputs the two 12-bit channel readings.

```verilog
18:     // Synchronize inputs
19:     reg [1:0] d0_sync, d1_sync;
20:     always @(posedge clk or negedge rst_n) begin
...
26:             d0_sync <= {d0_sync[0], adc_d0};
27:             d1_sync <= {d1_sync[0], adc_d1};
```
* **L18-L27**: **Input Synchronizers**. `adc_d0` and `adc_d1` are coming from the external physical world. If they change exactly as the FPGA clock rises, it can cause metastability. By shifting the inputs through two flip-flops (`d0_sync[0]` then `d0_sync[1]`), we ensure a clean, stable 1 or 0 for the state machine.

```verilog
32:     // Clock divider and SCLK generation
33:     reg [2:0] clk_div;
34:     reg sclk_en;
...
40:             clk_div <= clk_div + 3'd1;
41:             if (clk_div == 3'd3) adc_sclk <= 1'b1;
42:             else if (clk_div == 3'd7) adc_sclk <= 1'b0;
```
* **L32-L42**: **SPI Clock Generation**. The FPGA clock is 100MHz. The AD7476A max clock is 20MHz. We use a 3-bit counter (`clk_div`) that rolls over every 8 cycles (0 to 7). `100MHz / 8 = 12.5MHz`. 
* **L41-L42**: When the counter hits 3, the SPI clock goes HIGH. When it hits 7, it goes LOW. This creates a perfect 50% duty cycle 12.5 MHz clock.

```verilog
48:     wire sclk_rise = (clk_div == 3'd3) && sclk_en;
49:     wire sclk_fall = (clk_div == 3'd7) && sclk_en;
```
* **L48-L49**: **Edge Triggers**. Instead of creating a new clock domain (which causes timing nightmares in FPGAs), the SPI state machine stays in the 100MHz domain. It simply uses these 1-cycle pulses (`sclk_rise`, `sclk_fall`) to know when the slow SPI clock is transitioning.

```verilog
84:                 RUN: begin
85:                     if (sclk_rise) begin
86:                         shift_ch0 <= {shift_ch0[14:0], d0_in};
87:                         shift_ch1 <= {shift_ch1[14:0], d1_in};
88:                     end else if (sclk_fall) begin
89:                         bit_cnt <= bit_cnt + 5'd1;
```
* **L84-L89**: **The SPI Core**. This implements SPI Mode 0. The external ADC puts new data on the line during the *falling* edge of `adc_sclk`. Therefore, the FPGA reads (samples) the data on the *rising* edge (`sclk_rise`) directly into the `shift_ch0` and `shift_ch1` shift registers. 
* On the *falling* edge (`sclk_fall`), it counts the bit to know how far along the 16-bit transaction is.

```verilog
101:                QUIET: begin
102:                    if (quiet_cnt == 3'd4) begin
103:                        state <= IDLE;
```
* **L101-L103**: After finishing 16 bits, the CS line goes high. The AD7476A datasheet specifies a minimum "Quiet Time" (delay) before pulling CS low again for the next reading. This state waits for 5 clock cycles to guarantee we meet that specification before going back to `IDLE`.

---

## 5. `encoder_reader.v` (Position & Velocity)

```verilog
17:     // Synchronizers
18:     reg [2:0] a_sync, b_sync, z_sync;
19:     always @(posedge clk or negedge rst_n) begin
...
25:             a_sync <= {a_sync[1:0], enc_a};
```
* **L17-L25**: Similar to the ADC, we use synchronizers for the physical A, B, and Z encoder pulses. However, this is a **3-stage** synchronizer. We use the third stage (`a_sync[2]`) as the "previous" state, and the second stage (`a_sync[1]`) as the "current" state. This allows us to detect rising and falling edges instantly!

```verilog
37:     wire z_rise = z_in && !z_prev;
38: 
39:     // Quadrature Decoding
40:     wire [3:0] quad_state = {a_prev, b_prev, a_in, b_in};
41:     reg [1:0] move; // 0=none, 1=fwd, 2=rev
```
* **L37**: Pure edge detection. If Z is high now, but was low last cycle, `z_rise` is TRUE for exactly 1 clock cycle.
* **L40**: Creates a 4-bit word representing the encoder's transition: `{Old_A, Old_B, New_A, New_B}`. 

```verilog
43:     always @(*) begin
44:         case (quad_state)
45:             4'b0001, 4'b0111, 4'b1110, 4'b1000: move = 2'd1; // Forward
46:             4'b0010, 4'b1011, 4'b1101, 4'b0100: move = 2'd2; // Reverse
47:             default: move = 2'd0;
48:         endcase
49:     end
```
* **L43-L49**: **4x Quadrature Decoding**. This combinatorial block looks at the 4-bit history. For example, `4'b0001` means A stayed 0, but B transitioned from 0 to 1. In a standard quadrature setup, this specific transition implies the motor turned one "tick" forward. By mapping all 8 valid transitions to forward (1) or reverse (2), we get 4x resolution (10,000 counts per 2500 line disk).

```verilog
51:     always @(posedge clk or negedge rst_n) begin
...
54:         end else if (z_rise && ENCODER_Z_RESET) begin
55:             position <= 32'sd0;
56:         end else if (move == 2'd1) begin
57:             position <= position + 32'sd1;
```
* **L51-L57**: Keeps a running tally of the absolute rotor position. If the physical Z-index pulse hits (once per revolution) and `ENCODER_Z_RESET` is enabled, the counter snaps back to 0.

```verilog
88:             case (calc_state)
89:                 3'd0: begin
90:                     if (sample_tick) begin
91:                         delta_reg <= position - prev_position;
92:                         prev_position <= position; // capture instantly
93:                         calc_state <= 3'd1;
94:                     end
95:                 end
```
* **L88-L95**: **Speed Pipeline (Step 1)**. The FSM waits for `sample_tick` (which pulses at 10 kHz). It calculates how many ticks the motor moved in the last $100 \mu s$ (`delta_reg`) and updates the `prev_position` snapshot.

```verilog
96:                 3'd1: begin
97:                     // Pipeline Stage 1: Multiply delta by SPEED_SCALE
98:                     raw_mult_reg <= delta_reg[DATA_WIDTH-1:0] * SPEED_SCALE;
99:                     calc_state <= 3'd2;
100:                 end
```
* **L96-L100**: **Speed Pipeline (Step 2)**. Because the FPGA runs at 100MHz, doing multiple 32-bit math operations in one clock cycle will fail timing analysis. Here, we isolate the first multiplication (`delta * SPEED_SCALE`) into its own clock cycle and store the 64-bit result in `raw_mult_reg`.

```verilog
101:                3'd2: begin
102:                    // Pipeline Stage 2: Apply IIR filter multiplications
103:                    filt_term1_reg <= SPEED_ALPHA * speed_raw;
104:                    filt_term2_reg <= SPEED_ONE_MINUS_ALPHA * speed_elec_prev;
105:                    calc_state <= 3'd3;
106:                end
```
* **L101-L106**: **Speed Pipeline (Step 3)**. Now we execute the two multiplications required for the Infinite Impulse Response (IIR) low-pass filter simultaneously. `speed_raw` is a wire that truncates `raw_mult_reg` back to Q12.20.

```verilog
107:                3'd3: begin
108:                    // Pipeline Stage 3: Add truncated filter terms
109:                    speed_elec <= t1_trunc + t2_trunc;
110:                    speed_valid <= 1'b1;
111:                    calc_state <= 3'd0;
112:                end
```
* **L107-L112**: **Speed Pipeline (Step 4)**. The truncated results are added together to form the final filtered electrical speed (`speed_elec`), and the FSM outputs a `speed_valid` pulse before returning to sleep.

---

## 6. `speed_pi.v` (Speed Control Loop)

This module wraps around the MPC torque tracker to provide speed regulation.

```verilog
18:     reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
19:     reg signed [2*DATA_WIDTH-1:0] mul_result_full;
20:     wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
21:     
22:     always @(posedge clk) begin
23:         mul_result_full <= mul_a * mul_b;
24:     end
```
* **L18-L24**: The shared pipelined multiplier. This is identical to the optimization used in `motor_predictor.v`. We only instantiate one DSP multiplier and time-share it for the Proportional (P) and Integral (I) calculations.

```verilog
43:                 0: begin
44:                     if (sample_tick) begin
45:                         err <= speed_ref - speed_fb;
46:                         state <= 1;
47:                     end
48:                 end
```
* **L43-L48**: When the 10kHz `sample_tick` arrives, it calculates the raw speed error: $\omega_{error} = \omega_{ref} - \omega_{feedback}$.

```verilog
49:                 1: begin
50:                     // Step 1: Accumulate error (Euler integration)
51:                     err_integ <= err_integ + err;
52:                     
53:                     // Setup P-term multiply
54:                     mul_a <= PI_KP;
55:                     mul_b <= err;
56:                     state <= 2;
57:                 end
```
* **L49-L57**: Standard discrete integration. It adds the new error to the running total (`err_integ`). Simultaneously, it loads the multiplier with $K_p \times error$.

```verilog
58:                 2: begin
59:                     // Wait for P multiply
60:                     state <= 3;
61:                 end
62:                 3: begin
63:                     p_term <= mul_result;
64:                     // Setup I-term multiply
65:                     mul_a <= PI_KI;
66:                     mul_b <= err_integ;
67:                     state <= 4;
68:                 end
```
* **L58-L68**: State 2 does nothing (waits for the DSP multiplier latency). State 3 latches the resulting $P$ term and immediately loads the inputs for $K_i \times \int error$.

```verilog
74:                 6: begin
75:                     unclipped_te <= p_term + i_term;
76:                     state <= 7;
77:                 end
```
* **L74-L77**: After waiting for the $I$ term and latching it, it adds both terms to calculate the raw, requested torque (`unclipped_te`).

```verilog
78:                 7: begin
79:                     // Saturation (Clamping)
80:                     if (unclipped_te > TE_MAX) begin
81:                         te_ref <= TE_MAX;
82:                     end else if (unclipped_te < TE_MIN) begin
83:                         te_ref <= TE_MIN;
84:                     end else begin
85:                         te_ref <= unclipped_te;
86:                     end
87:                     state <= 0;
88:                 end
```
* **L78-L88**: **Anti-Windup / Clamping**. The motor can only handle so much torque before breaking or causing extreme current spikes. If the requested torque is higher than `TE_MAX` (e.g., +20 Nm), it hard-clips the output `te_ref` to `TE_MAX`. If it's too negative, it clips to `TE_MIN`. Once clamped, it exports `te_ref` back to the top-level module where it is fed into the `cost_evaluator.v`, and goes to sleep until the next 10kHz pulse.

---

## 7. `cost_evaluator.v` (The MPC Optimization Metric)

This module calculates the penalty (cost) of a proposed voltage vector. The optimal selector will pick the vector with the lowest penalty.

```verilog
19:     // Cost function constants in Q12.20 format
20:     localparam signed [DATA_WIDTH-1:0] KT = 32'sd3055130;
21:     localparam signed [DATA_WIDTH-1:0] PSI_REF_SQ = 32'sd966367;
22:     localparam signed [DATA_WIDTH-1:0] LAMBDA_T = 32'sd1048576;
23:     localparam signed [DATA_WIDTH-1:0] LAMBDA_PSI = 32'sd104857600;
```
* **L19-L23**: Pulls the physical constants and optimization weights. Notice that `LAMBDA_PSI` (100.0) is much larger than `LAMBDA_T` (1.0). Flux is measured in Webers (usually ~1.0), while torque is in Nm (e.g., 5.0). Squaring them makes flux numbers extremely tiny, so the algorithm multiplies flux errors by 100 so they compete fairly with torque errors.

```verilog
40:     // Registered multiplier inference
41:     always @(posedge clk or negedge rst_n) begin
...
46:             mul_result_full <= mul_a * mul_b;
47:         end
48:     end
```
* **L40-L48**: Identical pipelined shared-multiplier structure to `motor_predictor.v`. We only instantiate one DSP multiplier slice and time-share it across the 23-state FSM.

```verilog
69:                     0: begin 
70:                         // Waiting for first multiply to register: (psi_r_alpha_pred * is_beta_pred)
71:                     end
72:                     1: begin
73:                         cross1 <= mul_result;
74:                         mul_a <= psi_r_beta_pred;
75:                         mul_b <= is_alpha_pred;
76:                     end
...
86:                     4: begin
87:                         mul_a <= KT;
88:                         mul_b <= cross_diff; // cross_diff is ready since it was registered last cycle
89:                     end
```
* **L69-L89**: **Torque Prediction**. Electromagnetic torque is the cross-product of flux and current: $T_e = K_T \times (\psi_{\alpha} i_{\beta} - \psi_{\beta} i_{\alpha})$. 
* Step 0 waits for $\psi_{\alpha} i_{\beta}$ (which was loaded right when `start` was pulsed).
* Step 1 latches it into `cross1`, and sets up $\psi_{\beta} i_{\alpha}$.
* Step 3 subtracts them (`cross_diff`).
* Step 4 multiplies the difference by the motor's torque constant `KT`.

```verilog
111:                    12: begin
112:                        psi_a_sq <= mul_result;
113:                        mul_a <= psi_r_beta_pred;
114:                        mul_b <= psi_r_beta_pred;
115:                    end
...
120:                    14: begin
121:                        psi_b_sq <= mul_result;
122:                        psi_sq <= psi_a_sq + mul_result;
123:                    end
```
* **L111-L123**: **Flux Magnitude Squared**. The cost function wants to penalize flux deviations. But calculating $|\psi| = \sqrt{\psi_{\alpha}^2 + \psi_{\beta}^2}$ requires a Square Root, which consumes massive logic resources on an FPGA. 
* Instead, we just use the squared magnitude: $\psi^2_{magnitude} = \psi_{\alpha}^2 + \psi_{\beta}^2$. We compare this directly to a squared reference (`PSI_REF_SQ`).

```verilog
166:                    22: begin
167:                        cost_f <= mul_result;
168:                        
169:                        // Check for overflow (simple clamping logic)
170:                        if ((cost_t > 0 && mul_result > 0 && (cost_t + mul_result) < 0) || 
171:                            cost_t > 32'sd1048576000 || mul_result > 32'sd1048576000) begin
172:                            cost <= 32'sd2147483647; // Max int
173:                            cost_overflow <= 1;
174:                        end else begin
175:                            cost <= cost_t + mul_result;
176:                            cost_overflow <= 0;
177:                        end
178:                    end
```
* **L166-L178**: **The Saturation Check**. After multiplying the squared errors by $\lambda_T$ and $\lambda_\psi$, we must add them together (`cost_t + mul_result`). If a massive transient error occurs, adding two giant 32-bit positive numbers could overflow the 32-bit register, wrapping around to a massive *negative* number.
* If the cost wrapped around to negative, the `optimal_selector` would think it was the "lowest cost" and choose the absolute worst voltage vector! 
* This block checks if an overflow happened. If so, it hard-clips the cost to `2,147,483,647` (the maximum possible signed 32-bit positive integer).

---

## 8. `optimal_selector.v` (The Min-Finder)

```verilog
26:     always @(posedge clk or negedge rst_n) begin
...
29:             min_cost         <= {1'b0, {(DATA_WIDTH-1){1'b1}}}; // Max positive value
30:             opt_switch_state <= 3'b000;
...
```
* **L26-L30**: At initialization (and whenever `reset_search` is pulsed at the start of the 8-vector MPC loop), `min_cost` is initialized to its maximum possible positive value (`0x7FFFFFFF`). 

```verilog
43:             } else if (cost_valid) begin
44:                 // Compare new cost against running minimum
45:                 if (cost < min_cost) begin
46:                     min_cost         <= cost;
47:                     opt_switch_state <= switch_state;
48:                 end
```
* **L43-L48**: When the `cost_evaluator` pulses `cost_valid`, this block looks at the incoming `cost`. If it is strictly less than the running `min_cost`, it overwrites `min_cost` with the new value, and memorizes which vector (`switch_state`) produced it.

```verilog
49:                 // Track how many vectors have been evaluated
50:                 if (eval_count == NUM_VECTORS - 1) begin
51:                     done <= 1'b1; // All 8 vectors evaluated
52:                 end
53:                 eval_count <= eval_count + 4'd1;
54:             end
```
* **L49-L54**: It increments `eval_count` each time a cost is submitted. Because there are 8 vectors ($V_0 \dots V_7$), it pulses `done` once `eval_count` hits 7, signaling the master FSM to apply the optimal vector to the physical pins.

---

## 9. `gate_driver.v` (Dead-Time Insertion)

This module translates the optimal 3-bit vector (e.g., `100` = Phase A High, B/C Low) into 6 physical IGBT/MOSFET gate signals with dead-time.

```verilog
24:     always @(posedge clk or negedge rst_n) begin
...
30:                 target_state <= switch_state;
```
* **L24-L30**: When the FSM pulses `update_tick`, the module latches the optimal vector into `target_state`.

```verilog
61:         if (current_state[2] != target_state[2]) begin
62:             // Phase A state change requested
63:             dead_time_active[2] <= 1'b1;
64:             gate_ah <= 1'b0;
65:             gate_al <= 1'b0;
66:             dt_cnt_a <= 12'd0;
```
* **L61-L66**: **Phase A Logic** (Phase B and C are identical). It constantly monitors if `current_state` matches `target_state`. If they differ, the physical gates must change.
* To prevent shoot-through (short-circuiting the DC bus through the inverter legs), we cannot immediately flip the gates. 
* We instantly turn OFF both the High (`gate_ah`) and Low (`gate_al`) gates, assert the `dead_time_active` flag, and reset a counter to 0.

```verilog
68:         end else if (dead_time_active[2]) begin
69:             if (dt_cnt_a < DEAD_TIME_CYCLES) begin
70:                 dt_cnt_a <= dt_cnt_a + 12'd1;
71:             end else begin
72:                 dead_time_active[2] <= 1'b0;
73:                 current_state[2] <= target_state[2];
74:                 gate_ah <= target_state[2];
75:                 gate_al <= ~target_state[2];
76:             end
77:         end
```
* **L68-L77**: While `dead_time_active` is true, the counter ticks up. Once it hits `DEAD_TIME_CYCLES` (200 cycles = 2 $\mu s$), it finally applies the `target_state` to the high gate, and the inverted state to the low gate. The leg is now safely flipped!

---

## 10. `clarke_transform.v` (Reference Frame Math)

Converts 3-phase $(a,b,c)$ values to stationary orthogonal $(\alpha, \beta)$ values. Because $I_a + I_b + I_c = 0$ in a balanced motor, we only need to measure 2 channels ($I_a, I_b$) via the ADC.

```verilog
60:                 0: begin
61:                     if (start) begin
62:                         // Step 0: Convert raw ADC to signed
63:                         ia_signed <= $signed({1'b0, ia_raw}) - $signed(ADC_OFFSET);
64:                         ib_signed <= $signed({1'b0, ib_raw}) - $signed(ADC_OFFSET);
65:                         step <= 1;
66:                     end
67:                 end
```
* **L60-L67**: The raw ADC values are 12-bit unsigned (0 to 4095). A current of 0 Amps physically reads as roughly 2048 (`ADC_OFFSET`). This step converts them to true signed integers (centered at 0) before doing any math.

```verilog
90:                 6: begin
91:                     // Step 3: temp = ia + (ib <<< 1) (which is ia + 2*ib)
92:                     temp <= ia + (ib <<< 1);
93:                     step <= 7;
94:                 end
95:                 7: begin
96:                     // Step 4: i_beta = temp * INV_SQRT3
97:                     mul_a <= temp;
98:                     mul_b <= INV_SQRT3;
99:                     step <= 8;
100:                end
```
* **L90-L100**: The standard Clarke equation for $\beta$ is: $i_\beta = \frac{1}{\sqrt{3}}(i_a + 2i_b)$.
* **L92**: Multiplication by 2 is executed combinationally by shifting left by 1 bit (`ib <<< 1`), saving DSP resources. 
* **L98**: It then multiplies the result by $\frac{1}{\sqrt{3}}$ (which is pre-calculated as `32'sd605510` in Q12.20).
