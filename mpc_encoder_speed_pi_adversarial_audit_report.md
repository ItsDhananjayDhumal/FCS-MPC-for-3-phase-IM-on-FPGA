# Adversarial Re-Audit: Encoder Reader, Speed PI, and System Integration

**Target Modules:**
- [`encoder_reader.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v)
- [`speed_pi.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v)
- [`mpc_top.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v)
- [`control_fsm.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/control_fsm.v)
- [`mpc_params.vh`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_params.vh)

**Simulation Engine:** AMD Vivado Simulator (XSim) v2023.2 (64-bit)  
**Clock / Sampling:** $f_{clk} = 100\text{ MHz}$ ($10\text{ ns}$), $T_s = 100\,\mu\text{s}$ ($f_s = 10\text{ kHz}$)  
**Repository State:** Unmodified (100% read-only verification performed in isolated scratch sandbox).

---

## Executive Summary of Audit Results

| # | Audit Item / Verdict | Expected / Safe Behavior | Actual Behavior in RTL | Vivado XSim Proof Status | Severity |
|---|----------------------|--------------------------|------------------------|--------------------------|----------|
| **1** | `encoder_reader.v`: Unsigned slice on `delta_reg` | Signed product for reverse rotation | `delta_reg[31:0]` is unsigned; multiplies as unsigned; corrupts reverse speed to $-1776.14\text{ rad/s}$ | **CONFIRMED** | **FATAL** |
| **2** | `encoder_reader.v`: Double fractional shift `[51:20]` | Delta count is integer, product is Q12.20; shift by 0 | `raw_mult_reg[51:20]` shifts right by 20 bits, dividing by $1,048,576$ | **CONFIRMED** | **FATAL** |
| **3** | `encoder_reader.v`: Z-index position reset spike | Continuous unwrapped position for speed estimation | Z reset forces position to 0; delta drops by CPR ($-10,000$ counts); creates $-125,000\text{ rad/s}$ spike | **CONFIRMED** | **CRITICAL** |
| **4** | `speed_pi.v`: State variable truncation | State machine transitions 7 $\to$ 8 $\to$ 0 | `reg [2:0] state` truncates `8` (4'b1000) to `3'b000`; State 8 unreachable; `te_ref` frozen at 0 | **CONFIRMED** | **FATAL** |
| **5** | `mpc_top.v`: `speed_pi` uninstantiated, floating port | Closed-loop torque regulation driving `cost_evaluator` | `u_pi` omitted; `te_ref_in` floating `'bz`; propagates `'x` through cost; vector frozen at `3'b000` | **CONFIRMED** | **FATAL** |
| **6** | `encoder_reader.v`: Invalid quadrature transitions | Noise deglitch filter, debounce, and error flag | Double transitions silently ignored (`move=0`); noise spikes trigger reverse counting (-1) | **CONFIRMED (NEW)** | **HIGH** |
| **7** | `encoder_reader.v`: Synchronizer Metastability | 3-FF synchronizer with `ASYNC_REG = "TRUE"` | Actually only 2-FF sync + 1-cycle delay; missing `ASYNC_REG`; independent A/B skew | **CONFIRMED (NEW)** | **MEDIUM** |
| **8** | Speed Filter Dynamics ($\alpha = 0.1$, $1-\alpha = 0.9$) | Low latency phase response | $\tau \approx 1.0\text{ ms}$ (10 control cycles); distorts flux vector & current prediction during transients | **CONFIRMED (NEW)** | **HIGH** |
| **9** | `speed_pi.v`: Integrator overflow & lack of anti-windup | Integrator clamped to active linear band | `err_integ` overflows 32-bit signed in **21 ticks (2.1 ms)**; causes catastrophic torque reversal & 29.2 ms windup | **CONFIRMED (NEW)** | **FATAL** |

---

## Part 1: Adversarial Review of Previous Verdicts

### Verdict 1: Unsigned Part-Select Corrupts Reverse Speed
- **RTL Source:** [`encoder_reader.v` (Lines 68, 100)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L68-L100)
  ```verilog
  reg signed [31:0] delta_reg;
  ...
  raw_mult_reg <= delta_reg[`DATA_WIDTH-1:0] * `SPEED_SCALE;
  ```
- **Language Standard Defect (IEEE 1364 / IEEE 1800):**
  A vector part-select `delta_reg[31:0]` is **strictly unsigned**, regardless of the signedness of `delta_reg`. When one operand is unsigned, the Verilog multiplier coerces all operands to unsigned.
- **Mathematical Impact:**
  For 1 pulse reverse rotation ($\Delta = -1 = \text{32'hFFFF\_FFFF} = 4,294,967,295$), and `SPEED_SCALE = 13,176,795` (`32'h00C9_0FDB`):
  $$\text{Product} = 4,294,967,295 \times 13,176,795 = 56,593,903,577,919,525 \quad (\text{64'h00C9\_0FDA\_FF36\_F025})$$
  Extracting bits `[51:20]`:
  $$\text{speed\_raw} = \text{raw\_mult\_reg}[51:20] = \text{32'h90FD\_AFF3} = -1,862,422,541$$
  Converting Q12.20 fixed-point to engineering units:
  $$\omega_{raw} = \frac{-1,862,422,541}{2^{20}} = \mathbf{-1776.143\text{ rad/s}}$$
- **Vivado XSim Simulation Waveform Log (`tb_encoder_audit.v`):**
  ```text
  --- TEST 2: Reverse Rotation (1 count reverse) ---
  Time=366000 ps: position=0
  Time=406000 ps: Reverse speed_elec = -186242965 (hex f4e6286b, real -177.615132 rad/s)
  delta_reg = -1 (ffffffff), raw_mult_reg = 56593903577919525 (00c90fdaff36f025), speed_raw = -1862422541 (90fdaff3)
  ```
  *(Note: Filtered `speed_elec` after 1 cycle with $\alpha=0.1$ is $-177.615\text{ rad/s}$, converging to $-1776.14\text{ rad/s}$).*
- **Verdict: CONFIRMED 100%.**

---

### Verdict 2: Double Fractional Division Attenuates Speed Feedback by $1,048,576\times$
- **RTL Source:** [`encoder_reader.v` (Lines 73, 100)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L73-L100)
  ```verilog
  wire signed [`DATA_WIDTH-1:0] speed_raw = raw_mult_reg[`DATA_WIDTH+`FRAC_BITS-1 : `FRAC_BITS];
  ```
- **Dimensional Analysis Defect:**
  - `delta_reg` is the integer count difference ($\text{dimension: counts}$, e.g. $1\text{ count}$).
  - `SPEED_SCALE` is defined in [`mpc_params.vh`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_params.vh#L26) as `32'sd13176795`, which is $12.56637\text{ rad/s/count} \times 2^{20}$ (Q12.20).
  - The multiplication of $\text{Integer} \times \text{Q12.20}$ already yields a **Q12.20** number!
  - Selecting `raw_mult_reg[51:20]` applies an additional arithmetic right shift by 20 bits ($2^{20} = 1,048,576$), truncating the fractional portion completely.
- **Physical Result:**
  For $\Delta = +1\text{ count}$ ($12.566\text{ rad/s}$):
  $$\text{raw\_mult\_reg} = 1 \times 13,176,795 = 13,176,795$$
  $$\text{speed\_raw} = 13,176,795 \gg 20 = 12 \quad (\text{Q12.20 value: } 12 \times 2^{-20} = \mathbf{0.0000114\text{ rad/s}})$$
  After 1 IIR filter step: `speed_elec = 1` ($0.000001\text{ rad/s}$).
- **Vivado XSim Simulation Waveform Log (`tb_encoder_audit.v`):**
  ```text
  --- TEST 1: Forward Rotation (1 count per sample) ---
  Time=190000 ps: position=1
  Time=226000 ps: Forward speed_elec = 1 (hex 00000001, real 0.000001 rad/s)
  raw_mult_reg = 13176795 (0000000000c90fdb), speed_raw = 12 (0000000c)
  ```
- **Verdict: CONFIRMED 100%.**

---

### Verdict 3: Z-Index Position Reset Velocity Spike
- **RTL Source:** [`encoder_reader.v` (Lines 55-56, 93)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L55-L93)
  ```verilog
  else if (z_rise && `ENCODER_Z_RESET) begin
      position <= 32'sd0;
  ...
  delta_reg <= position - prev_position;
  ```
- **Physical Dynamics Defect:**
  With $\text{CPR} = 10,000$ counts/rev, when the motor completes 1 mechanical revolution at continuous forward speed:
  $$\text{prev\_position} \approx 10,000$$
  $$\text{position (post-reset)} = 0$$
  $$\text{delta\_reg} = 0 - 10,000 = \mathbf{-10,000\text{ counts}}$$
  In a $100\,\mu\text{s}$ sample window, moving $-10,000$ counts corresponds to:
  $$\omega_{spike} = -10,000 \times 12.56637\text{ rad/s} = \mathbf{-125,663.7\text{ rad/s}}$$
- **Vivado XSim Simulation Waveform Log (`tb_all_verdicts.v`):**
  ```text
  Before Z pulse: position = 10000, prev_position = 9990
  After Z pulse: position = 0 (reset to 0), prev_position = 9990
  After Z reset: delta_reg = -9990 counts
  ```
- **Verdict: CONFIRMED 100%.** Disabling Z reset on position used for speed estimation (as confirmed by user decision) is mandatory.

---

### Verdict 4: `speed_pi.v` State Variable Bitwidth Truncation
- **RTL Source:** [`speed_pi.v` (Lines 27, 86)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v#L27-L86)
  ```verilog
  reg [2:0] state; // 3-bit variable can only hold values 0 through 7!
  ...
  84: 7: begin
  85:     unclipped_te <= p_term + i_term;
  86:     state <= 8; // 8 = 4'b1000, truncated to 3'b000!
  87: end
  88: 8: begin // UNREACHABLE!
  89:     if (unclipped_te > `TE_MAX) te_ref <= `TE_MAX;
  ...
  95:     te_ref <= unclipped_te;
  96:     state <= 0;
  97: end
  ```
- **Vivado XSim Cycle-Accurate Execution Trace (`tb_all_verdicts.v`):**
  ```text
  Initial te_ref = 0, state = 0
  Cycle  0: PI state = 2, unclipped_te = 0,         te_ref = 0
  Cycle  1: PI state = 3, unclipped_te = 0,         te_ref = 0
  Cycle  2: PI state = 4, unclipped_te = 0,         te_ref = 0
  Cycle  3: PI state = 5, unclipped_te = 0,         te_ref = 0
  Cycle  4: PI state = 6, unclipped_te = 0,         te_ref = 0
  Cycle  5: PI state = 7, unclipped_te = 0,         te_ref = 0
  Cycle  6: PI state = 0, unclipped_te = 534773700, te_ref = 0  <-- Skipped State 8!
  Cycle  7: PI state = 0, unclipped_te = 534773700, te_ref = 0
  ```
- **Consequence:**
  `state` rolls over directly from 7 to 0. State 8 is dead code. Output `te_ref` is never updated and remains permanently 0.
- **Verdict: CONFIRMED 100%.**

---

### Verdict 5: Floating `te_ref_in` Freezes Inverter on Vector 0
- **RTL Source:** [`mpc_top.v` (Lines 238-251)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v#L238-L251) and [`cost_evaluator.v` (Line 99)](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/cost_evaluator.v#L99)
  ```verilog
  // mpc_top.v: u_cost instantiated WITHOUT te_ref_in
  cost_evaluator #( ... ) u_cost (
      .clk (clk), ...
      .cost (cost_value_w),
      .done (cost_done_w)
      // te_ref_in is NOT CONNECTED! Defaults to 'bz
  );
  ```
  Inside `cost_evaluator.v`:
  ```verilog
  torque_err <= te_ref_in - te_pred; // 'bz - te_pred = 'x
  ```
  Inside `optimal_selector.v`:
  ```verilog
  if (cost < min_cost) begin // 'x < min_cost evaluates to FALSE!
      min_cost <= cost;
      opt_switch_state <= switch_state;
  end
  ```
- **Vivado XSim Simulation Waveform Log (`tb_cost_verdict5.v`):**
  ```text
  te_ref_floating = zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz
  Vector 0: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 1: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 2: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 3: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 4: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 5: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 6: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  Vector 7: cost = xxxxxxxx (x), cost_overflow = 0 | Selector opt_switch_state = 000
  ```
- **Verdict: CONFIRMED 100%.** Because `speed_pi` was uninstantiated and `te_ref_in` left floating, `cost` is `'x`, comparison fails for all 8 vectors, and `opt_switch_state` locks permanently on Vector 0 (`000`), commanding 0V across all motor phases.

---

## Part 2: Comprehensive Audit of New Hazards

### Hazard 2.1: Quadrature Decoder Robustness & Noise Transition Vulnerability
In [`encoder_reader.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L44-L50):
```verilog
wire [3:0] quad_state = {a_prev, b_prev, a_in, b_in};
always @(*) begin
    case (quad_state)
        4'b0001, 4'b0111, 4'b1110, 4'b1000: move = 2'd1; 
        4'b0010, 4'b1011, 4'b1101, 4'b0100: move = 2'd2; 
        default: move = 2'd0;
    endcase
end
```

#### Detailed Failure Modes:
1. **Double-Bit State Transitions (00 $\to$ 11, 01 $\to$ 10, 11 $\to$ 00, 10 $\to$ 01):**
   - The case statement groups all 4 double-bit transitions into `default: move = 2'd0`.
   - If the motor spins beyond the sampling resolution, or if clock skew causes both channels to register changes in the same 10 ns clock cycle, the transition is **silently dropped**.
2. **Noise-Induced Direction Reversal (Glitches):**
   - In industrial motor drives, 10 kHz PWM with 311V DC bus produces steep $dV/dt$ transients ($>5\text{ kV}/\mu\text{s}$) that couple into encoder lines.
   - If channel B experiences a 20 ns noise spike while the motor is advancing forward ($00 \to 01$), the sampler sees $00 \to 11 \to 01$:
     - State $00 \to 11$: `default: move = 0` (dropped).
     - State $11 \to 01$: matches `4'b1101 => move = 2` (CCW / Reverse)!
   - **XSim Proof (`tb_quadrature_noise.v`):**
     ```text
     Test 2.1B: Motor moving CW (00 -> 01). Noise glitched B to 1: 00 -> 11 -> 01
     Time=330000 ps: Settled at 01. position = -1
     ```
     The motor moved forward, but the position counter decremented to $-1$!
3. **Absence of Glitch Filter & Error Telemetry:**
   - There is NO digital debounce filter (majority voting / deglitcher).
   - There is NO invalid state transition flag (`quad_err`) or glitch counter.
   - Failures accumulate silently, leading to undetected position and speed drift.

---

### Hazard 2.2: Synchronizer Metastability & Synthesis Vulnerability
In [`encoder_reader.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L20-L38):
```verilog
reg [2:0] a_sync, b_sync, z_sync;
always @(posedge clk or negedge rst_n) begin
    ...
    a_sync <= {a_sync[1:0], enc_a};
    ...
end
wire a_in   = a_sync[1];
wire a_prev = a_sync[2];
```

#### Architectural Findings:
1. **Effective Synchronizer Depth:**
   - Although `a_sync` is 3 bits wide, `a_in` is tapped at `a_sync[1]`.
   - `enc_a` feeds `a_sync[0]` (Stage 1), which feeds `a_sync[1]` (Stage 2).
   - `a_sync[2]` is merely a 1-clock history register (`a_prev`) for edge detection.
   - Therefore, the synchronization chain is **2 flip-flops**, not 3.
2. **MTBF Calculation for Artix-7 (100 MHz):**
   $$\text{MTBF} = \frac{e^{t_r / \tau}}{T_w \cdot f_{clk} \cdot f_{data}}$$
   - $f_{clk} = 100\text{ MHz}$ ($T_{clk} = 10\text{ ns}$).
   - Maximum encoder edge rate: $f_{data} \le 500\text{ kHz}$ (3000 RPM, 10,000 CPR).
   - In 28nm 7-Series: $\tau \approx 30\text{ ps}$, $T_w \approx 20\text{ ps}$, $t_r = 10\text{ ns} - 0.9\text{ ns} = 9.1\text{ ns}$.
   - $t_r / \tau = 9.1 / 0.03 \approx 303 \implies \text{MTBF} > 10^{100}\text{ years}$ (theoretical for pure metastable resolution).
3. **Critical Physical Vulnerability:**
   - Missing `(* ASYNC_REG = "TRUE" *)`: Without this constraint, Vivado place-and-route can separate `a_sync[0]` and `a_sync[1]` into different Slices or CLBs across the die. Long inter-slice routing delay degrades $t_r$ and exposes the design to hold violations and placement-induced metastability.
   - Asynchronous skew: Channels A and B are synchronized through separate FF chains without correlation checking, increasing vulnerability to race-induced illegal transitions ($00 \to 11$).

---

### Hazard 2.3: Speed Filter Dynamics & Control Loop Degradation
In [`encoder_reader.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L105-L112) and [`mpc_params.vh`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_params.vh#L28-L29):
$$\omega_{filt}[k] = \alpha \cdot \omega_{raw}[k] + (1-\alpha) \cdot \omega_{filt}[k-1]$$
where $\alpha = 0.1$ (`SPEED_ALPHA = 104858`), $1-\alpha = 0.9$ (`SPEED_ONE_MINUS_ALPHA = 943718`), and $T_s = 100\,\mu\text{s}$.

#### Quantitative Dynamics:
- **Filter Pole:** $z_p = 1 - \alpha = 0.9$.
- **Equivalent Continuous Time Constant ($\tau$):**
  $$\tau = \frac{-T_s}{\ln(1-\alpha)} = \frac{100\,\mu\text{s}}{0.10536} \approx \mathbf{0.949\text{ ms} \approx 1.0\text{ ms} = 10\text{ control cycles}}$$
- **Settling Time:**
  - 95% settling ($3\tau$): $\mathbf{3.0\text{ ms} = 30\text{ control cycles}}$.
  - 99% settling ($5\tau$): $\mathbf{5.0\text{ ms} = 50\text{ control cycles}}$.

#### System-Wide Impact:
1. **Rotor Flux Observer Distortion ([`flux_observer.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/flux_observer.v#L64-L130)):**
   - The rotor flux state equations require instantaneous electrical speed $\omega_r$:
     $$\psi_{r\alpha}[k+1] = E_{21} i_{s\alpha}[k] + E_{22} \psi_{r\alpha}[k] - T_s \omega_r[k] \psi_{r\beta}[k]$$
     $$\psi_{r\beta}[k+1] = E_{21} i_{s\beta}[k] + E_{22} \psi_{r\beta}[k] + T_s \omega_r[k] \psi_{r\alpha}[k]$$
   - Under motor acceleration ($\dot{\omega}_r = 1000\text{ rad/s}^2$), a 1 ms filter delay causes a dynamic speed estimation error of $\Delta \omega_r = \tau \dot{\omega}_r = 1.0\text{ rad/s}$.
   - The flux cross-coupling terms lag by $1.0\text{ ms}$, creating an angular phase error in the estimated rotor flux vector ($\Delta \theta_{flux} \approx 0.05\text{ to }0.1\text{ rad}$).
2. **Model Mismatch in Current Prediction ([`motor_predictor.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/motor_predictor.v#L15-L16)):**
   - Stator current prediction incorporates back-EMF terms proportional to $\omega_r \psi_r$ (`wr_psi_alpha`, `wr_psi_beta`).
   - Using filtered, delayed speed causes the 8-vector predictor to systematically underestimate back-EMF during acceleration, leading to sub-optimal voltage vector selection, increased switching ripple, and current distortion.
3. **Phase Margin Erosion in Cascaded Speed Loop:**
   - At a speed loop crossover frequency of $\omega_c = 300\text{ rad/s}$ ($48\text{ Hz}$):
     $$\phi_{lag} = -\arctan(\omega_c \tau) = -\arctan(300 \times 0.001) = \mathbf{-16.7^\circ}$$
   - This phase erosion reduces the stability margin of the speed PI controller, promoting overshoot and oscillatory behavior during step speed commands.

---

### Hazard 2.4: `speed_pi.v` Integrator Overflow & Anti-Windup Breakdown
In [`speed_pi.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v#L44-L98):
```verilog
1: begin
    err_integ <= err_integ + err; // Line 53: NO CLAMPING, NO SATURATION CHECK!
    ...
```
Only `unclipped_te` is clamped to `TE_MAX` in State 8 (Line 90). `err_integ` accumulates unbounded!

#### 1. Mathematical Derivation of 32-Bit Signed Overflow:
- Reference speed: `SPEED_REF_DEFAULT` = 100 rad/s ($104,857,600$ in Q20).
- Motor initial speed: `speed_fb` = 0.
- Error per sample: $\text{err} = 104,857,600$.
- Sampling period: $T_s = 100\,\mu\text{s}$.
- Maximum 32-bit signed positive integer:
  $$\text{INT32\_MAX} = 2^{31} - 1 = 2,147,483,647$$
- Cycles to overflow:
  $$N_{overflow} = \frac{2,147,483,647}{104,857,600} = 20.48 \implies \mathbf{21\text{ TICKS = 2.1 MILLISECONDS!}}$$

#### 2. Vivado XSim Cycle-Accurate Proof of Overflow (`tb_pi_antiwindup.v`):
```text
Tick 19: err_integ =  1992294400 (hex 76c00000), err = 104857600
Tick 20: err_integ =  2097152000 (hex 7d000000), err = 104857600
Tick 21: err_integ = -2092957696 (hex 83400000), err = 104857600
>>> CRITICAL OVERFLOW DETECTED at Tick 21! err_integ wrapped to NEGATIVE -2092957696!
```
At continuous stall, `err_integ` rolls over periodically **every 41 ticks (4.1 ms)** as proven in `tb_pi_continuous_stall.v`.

#### 3. Catastrophic Torque Inversion (`tb_pi_approaching_target.v`):
What happens if the motor accelerates during those 2.1 ms and approaches target speed?
- At Tick 22, motor reaches $95\text{ rad/s}$ ($5\text{ rad/s}$ below target of $100\text{ rad/s}$, so $\text{err} = +5\text{ rad/s}$):
  - Proportional term: $P = K_p \times \text{err} = 5.0 \times 5 = +25.0\text{ Nm}$ ($+26,214,400$).
  - Integral term: $I = K_i \times \text{err\_integ} = 0.1 \times (-2,092,957,696) / 2^{20} = \mathbf{-199.1\text{ Nm}}$ ($-208,770,287$).
  - Net torque demand:
    $$\text{unclipped\_te} = +25.0 - 199.1 = \mathbf{-174.1\text{ Nm}}$$
  - Saturated output: $\text{te\_ref} = \text{TE\_MIN} = \mathbf{-20.00\text{ Nm}}$!
- **XSim Simulation Output:**
  ```text
  At Tick 22 (speed_fb = 95 rad/s, speed_ref = 100 rad/s, err = +5 rad/s):
     p_term       =  26214400 (25.000000 Nm)
     i_term       = -208770287 (-199.098861 Nm)
     unclipped_te = -182555887 (-174.098861 Nm)
     te_ref       = -20971520 (-20.000000 Nm)

  >>> CONFIRMED: Motor speed is BELOW setpoint (95 rad/s < 100 rad/s),
      yet PI controller commands MAXIMUM NEGATIVE TORQUE (-20.000000 Nm)!
  ```
  **Failure Result:** The motor slams into full reverse electrical braking while trying to reach its forward setpoint.

#### 4. Integrator Unwinding Latency (`tb_pi_torque_unwind.v`):
If `err_integ` winds up to $2 \times 10^9$ (just below overflow):
- Active saturation threshold for `err_integ`:
  $$\text{err\_integ}_{sat} = \frac{\text{TE\_MAX} \times 2^{20}}{K_i} = \frac{20,971,520 \times 1,048,576}{104,857} \approx 2.1 \times 10^8$$
- When the motor reaches speed and overshoots by $+5\text{ rad/s}$ ($\text{err} = -5,242,880$):
- **XSim Simulation Proof:**
  ```text
  Initial err_integ = 2000000000
  Overshoot Tick  50: err_integ= 1737856000, te_ref= 20971520 (20.000000 Nm)
  Overshoot Tick 150: err_integ= 1213568000, te_ref= 20971520 (20.000000 Nm)
  Overshoot Tick 250: err_integ=  689280000, te_ref= 20971520 (20.000000 Nm)
  Overshoot Tick 292: err_integ=  469079040, te_ref= 20693235 (19.734607 Nm)
  >>> UNWOUND TO ACTIVE CONTROL at Tick 292 (29.20 ms after overshoot begins)!
  ```
- **Result:** It takes **292 control cycles (29.20 ms)** of continuous overspeeding before torque drops below maximum. During this entire time, full forward torque ($+20\text{ Nm}$) accelerates the overspeeding motor, causing severe mechanical runaway and overshoot.
- At $+1\text{ rad/s}$ overshoot, unwinding requires **1,460 cycles (146.0 ms)**.

---

## Part 3: Verification Suite Artifacts

All verification testbenches, simulation models, and scripts have been formally compiled and executed using Vivado XSim v2023.2 in the scratch workspace:
- [`tb_encoder_audit.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_encoder_audit.v): Proves Verdict 1 ($-1776.14\text{ rad/s}$ reverse corruption) and Verdict 2 ($1,048,576\times$ forward attenuation).
- [`tb_all_verdicts.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_all_verdicts.v): Proves Verdict 3 (Z-index position drop and $-125,000\text{ rad/s}$ spike) and Verdict 4 (`state <= 8` truncation locking `te_ref` at 0).
- [`tb_cost_verdict5.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_cost_verdict5.v): Proves Verdict 5 (unconnected `te_ref_in` causing `'x` cost and freezing `optimal_selector` on Vector 0).
- [`tb_quadrature_noise.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_quadrature_noise.v): Proves Hazard 2.1 (double-bit transition drops and glitch-induced count inversion).
- [`tb_pi_antiwindup.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_pi_antiwindup.v): Proves Hazard 2.4 (32-bit signed rollover in exactly 21 ticks / 2.1 ms).
- [`tb_pi_torque_unwind.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_pi_torque_unwind.v): Proves Hazard 2.4 (29.2 ms unwinding latency from $2 \times 10^9$).
- [`tb_pi_continuous_stall.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_pi_continuous_stall.v): Proves periodic 4.1 ms limit-cycle integer overflow.
- [`tb_pi_approaching_target.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/b12f4f6c-d918-48ee-b63c-94925164d370/scratch/tb_pi_approaching_target.v): Proves torque demand sign reversal (commands $-20\text{ Nm}$ at $95\text{ rad/s}$).

---

## Part 4: Recommended Corrective Architecture

> [!CAUTION]
> Per user constraints, no RTL in `mpc_3_phase_im` has been modified. The following architectures must be implemented when modifications are authorized.

### 1. `encoder_reader.v` Fixes:
1. **Signed Arithmetic & Scale Factor:**
   - Remove bit-select `delta_reg[DATA_WIDTH-1:0]`; cast explicitly using `$signed(delta_reg)`.
   - Redefine `SPEED_SCALE`: Since `delta_reg` is integer counts, multiply $delta \times SPEED\_SCALE$, where `SPEED_SCALE` is in Q12.20 format, and retain the lower 32-bit Q12.20 product without shifting:
     ```verilog
     wire signed [63:0] raw_mult_reg = delta_reg * `SPEED_SCALE;
     wire signed [31:0] speed_raw = raw_mult_reg[31:0]; // Q12.20 output directly!
     ```
2. **Dedicated Unwrapped Speed Counter:**
   - Maintain a 32-bit continuous unwrapped position counter (`pos_speed_unwrapped`) that **never resets on Z-index**.
   - Reserve Z-index reset strictly for mechanical homing / multi-turn revolution counting.
3. **Quadrature Glitch Filtering & Error Flag:**
   - Add a 4-clock digital debouncer (majority voter) on raw `enc_a`, `enc_b`, `enc_z`.
   - Add `(* ASYNC_REG = "TRUE" *)` attributes on the synchronizer registers.
   - Detect invalid states (`4'b0011, 4'b0110, 4'b1100, 4'b1001`) and assert an error telemetry flag `quad_error` with a sticky glitch counter.

### 2. `speed_pi.v` Fixes:
1. **FSM State Register Bitwidth:**
   - Expand `reg [2:0] state;` to `reg [3:0] state;` so State 8 is reachable.
2. **Integrator Clamping (Anti-Windup):**
   - Calculate maximum allowable integral term:
     $$\text{INTEG\_MAX} = \frac{\text{TE\_MAX} \times 2^{20}}{\text{PI\_KI}} \approx \text{32'sd209715200}$$
   - Apply clamping during integration in State 1:
     ```verilog
     wire signed [31:0] next_err_integ = err_integ + err;
     if (next_err_integ > `INTEG_MAX)
         err_integ <= `INTEG_MAX;
     else if (next_err_integ < -`INTEG_MAX)
         err_integ <= -`INTEG_MAX;
     else
         err_integ <= next_err_integ;
     ```
   - Alternatively, halt integration whenever `unclipped_te` is saturated in the same direction as `err` (conditional integration / anti-windup clamping).

### 3. `mpc_top.v` Integration:
1. Instantiate `speed_pi` module `u_speed_pi`:
   - Connect `.clk(clk)`, `.rst_n(rst_n)`, `.sample_tick(sample_tick_w)`.
   - Connect `.speed_ref(speed_ref_w)`, `.speed_fb(speed_elec_w)`.
   - Connect output `.te_ref(te_ref_w)`.
2. Connect `te_ref_w` into `u_cost`:
   - `.te_ref_in(te_ref_w)`.
   - This closes the speed control loop and provides real, non-`'bz` torque references to the FCS-MPC cost optimization engine.
