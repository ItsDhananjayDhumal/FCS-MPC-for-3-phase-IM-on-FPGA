# Adversarial Re-Audit Report: Core FCS-MPC Mathematical Pipeline

**Target Files**:
- [`flux_observer.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/flux_observer.v)
- [`motor_predictor.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/motor_predictor.v)
- [`cost_evaluator.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/cost_evaluator.v)
- [`optimal_selector.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/optimal_selector.v)
- [`fixed_point_mul.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/fixed_point_mul.v)
- [`mpc_params.vh`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_params.vh)

**Simulation Proof Artifacts**:
- [`tb_audit_formal.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/3ef05d1c-fa86-4cbe-874e-0d25afc48935/scratch/tb_audit_formal.v)
- [`tb_cascade_proof.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/3ef05d1c-fa86-4cbe-874e-0d25afc48935/scratch/tb_cascade_proof.v)

---

## Executive Summary & Verdict Matrix

| Pipeline Stage / Module | Finding / Hazard | Severity | Prior Verdict | Re-Audit Verdict |
| :--- | :--- | :--- | :--- | :--- |
| **`cost_evaluator.v`** | Multiplier bit-51 silent overflow causes cost inversion ($J < 0$) when $\|err_\psi\| \ge 4.53\text{ Wb}^2$ | **CRITICAL** | "False Alarm in Nominal Operation" | **OVERRULED: SEVERE SAFETY HAZARD**. Silent negative wrapping latches worst inverter state. |
| **`optimal_selector.v`** | Cost comparison `cost < min_cost` latches inverted negative costs; Standstill zero-state deadlock on ties | **CRITICAL** | Not fully analyzed | **NEW HAZARD PROVEN**. 1-step prediction produces identical costs at standstill; selector locks to Vector 0 forever. |
| **`flux_observer.v`** | Forward Euler conditional stability boundary at $\mathbf{1532.9\text{ RPM}}$ ($f_e = 51.1\text{ Hz}$). Exponential divergence at $3000\text{ RPM}$ | **CRITICAL** | Assumed stable | **NEW HAZARD PROVEN**. Standalone observer poles cross unit circle ($|\lambda| = 1.00146$ at 3000 RPM). Explodes in $60.9\text{ ms}$. |
| **`motor_predictor.v` & `flux_observer.v`** | Cross-coupling signs: $+C_{13}\omega_r \psi_\beta$, $-C_{13}\omega_r \psi_\alpha$, $-T_s \omega_r \psi_\beta$, $+T_s \omega_r \psi_\alpha$ | **VERIFIED** | Questioned | **MATHEMATICALLY CORRECT** against continuous $-j\vec{\psi}_r$ space-vector model. |
| **`fixed_point_mul.v`** | 2-cycle latency mismatch: `fixed_point_mul.v` has 2 pipeline stages, while all FSMs assume 1-wait cycle private multiplier | **HIGH** | Unaudited | **ARCHITECTURAL TRAP**. `fixed_point_mul.v` is dead code; replacing inline multipliers with it breaks pipeline. |
| **`speed_pi.v` & `mpc_top.v`** | `speed_pi` state bit-width overflow (`reg [2:0] state; state <= 8`); `mpc_top.v` leaves `cost_evaluator.te_ref_in` completely unrouted | **CRITICAL** | Out of previous scope | **FATAL INTEGRATION DEFECTS**. PI saturation unreachable; torque loop floating. |

---

## Task 1: Adversarial Review of Previous Verdicts

### 1.1 Cost Evaluator: Multiplier Overflow & Cost Inversion ($|err_\psi| \ge 4.53\text{ Wb}^2$)
* **Prior Assessment**: Marked as *"False Alarm in Nominal Operation"* because nominal flux is $\approx 0.96\text{ Wb}$ and steady-state error is small.
* **Re-Audit Reversal**: **COMPLETELY UNJUSTIFIED AND DANGEROUSLY WRONG**.
* **Mechanism Proof**:
  In [`cost_evaluator.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/cost_evaluator.v#L30-L33):
  ```verilog
  reg signed [DATA_WIDTH-1:0] mul_a, mul_b;
  reg signed [2*DATA_WIDTH-1:0] mul_result_full;
  wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS]; // [51:20]
  ```
  At step 20-22:
  `mul_a <= LAMBDA_PSI;` ($\lambda_\psi = 100 \times 2^{20} = 104857600$)
  `mul_b <= flux_err_sq;`
  When $mul\_a \cdot mul\_b \ge 2^{51}$, bit 51 becomes `1`. In 32-bit two's complement slicing `[51:20]`, bit 51 is the MSB (sign bit).
  $$\text{Threshold: } flux\_err\_sq \ge \frac{2^{51}}{100 \times 2^{20}} = 21474836.48 \text{ (raw Q20)} \implies |err_\psi| \ge \sqrt{\frac{21474836.48}{2^{20}}} = \mathbf{4.5255\text{ Wb}^2}$$
* **Flawed Overflow Detection**:
  Lines 157-164:
  ```verilog
  if ((cost_t > 0 && mul_result > 0 && (cost_t + mul_result) < 0) || 
      cost_t > 32'sd1048576000 || mul_result > 32'sd1048576000) begin
      cost <= 32'sd2147483647; 
      cost_overflow <= 1;
  end else begin
      cost <= cost_t + mul_result;
      cost_overflow <= 0;
  end
  ```
  When the multiplier overflows bit 51, `mul_result` is **already negative**!
  Therefore, `mul_result > 0` is FALSE, and `mul_result > 1048576000` is FALSE!
  The entire overflow logic is bypassed: `cost_overflow` remains `0`, and `cost` outputs a huge negative number (e.g. `$-2,075,314,396$`).
* **Hardware Consequences**:
  In FCS-MPC, `optimal_selector.v` compares signed integers: `if (cost < min_cost)`. A huge negative cost is evaluated as strictly superior to all valid positive costs. The controller latches the catastrophic switching state, causing full shoot-through / inverter overcurrent destruction.
* **Xsim Cycle-Accurate Verification**:
  ```
  [Nominal Case] Flux = 1.0 Wb. Cost = 644500, Overflow Flag = 0
  [Adversarial Case] Flux = 2.35 Wb (|err| = 4.60 Wb^2):
     Resulting Cost = -2075314396 (Hex: 32'h844d3724)
     Cost Overflow Flag = 0
     >>> CRITICAL HAZARD CONFIRMED: Cost inverted to NEGATIVE without overflow detection!
  ```

---

### 1.2 Optimal Selector: Negative Costs, Ties, and Standstill Deadlock
* **Negative Costs**: [`optimal_selector.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/optimal_selector.v#L37-L40) treats cost as signed: `if (cost < min_cost)`. It provides zero sanity checks against negative costs ($J < 0$), blindly accepting and latching corrupted vectors.
* **Zero-State Standstill Deadlock (Fundamental Control-Theoretic Flaw)**:
  At motor standstill ($i_\alpha = 0, i_\beta = 0, \psi_{r\alpha} = 0, \psi_{r\beta} = 0$):
  1. Discrete forward prediction in [`motor_predictor.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/motor_predictor.v#L169-L195):
     $$\psi_{r\alpha}(k+1) = E_{21} i_\alpha + E_{22} \psi_{r\alpha} - T_s \omega_r \psi_{r\beta} = 0$$
     $$\psi_{r\beta}(k+1) = E_{21} i_\beta + E_{22} \psi_{r\beta} + T_s \omega_r \psi_{r\alpha} = 0$$
     *Notice: Applied voltage $v_s(k)$ has NO direct coupling to $\psi_r(k+1)$!*
  2. Predicted torque in [`cost_evaluator.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/cost_evaluator.v#L84-L96):
     $$T_e(k+1) = K_T (\psi_{r\alpha}^{pred} i_{s\beta}^{pred} - \psi_{r\beta}^{pred} i_{s\alpha}^{pred}) = K_T (0 - 0) = 0$$
     *Because predicted rotor flux is identically zero, predicted torque is zero for all 8 voltage vectors, even though $i_s(k+1) = D_1 v_s \ne 0$!*
  3. Cost function output:
     $$J(V_i) = \lambda_T (T_{e,ref} - 0)^2 + \lambda_\psi (\psi_{ref}^2 - 0)^2 = \text{IDENTICAL CONSTANT FOR ALL 8 VECTORS}$$
  4. Selector tie-breaking:
     `optimal_selector.v` uses strict inequality: `if (cost < min_cost)`.
     Vector 0 ($V_0 = 3\text{'b}000$, zero vector) is evaluated first at `eval_count = 0` and latches into `min_cost`.
     Vectors 1 through 7 produce the identical cost. Since $J < J$ is FALSE, none of vectors 1..7 can ever be selected.
  5. Result: **Vector 0 ($V_0$) is chosen indefinitely**. Zero voltage is applied $\implies$ currents remain zero $\implies$ flux remains zero. The motor is trapped in an eternal standstill dead-zone and can never self-start!
* **Xsim Cycle-Accurate Verification**:
  ```
  Evaluating all 8 vectors at standstill (i=0, psi=0, speed=0, te_ref=5.0 N*m):
     Vector 0: is_alpha_pred=0, psi_r_alpha_pred=0       -> Cost = 115274700
     Vector 1: is_alpha_pred=1869317, psi_r_alpha_pred=0 -> Cost = 115274700
     Vector 2: is_alpha_pred=1869317, psi_r_alpha_pred=0 -> Cost = 115274700
     ...
     Vector 7: is_alpha_pred=0, psi_r_alpha_pred=0       -> Cost = 115274700
     Standstill Selected Vector: 3'b000
     >>> STANDSTILL DEADLOCK CONFIRMED: All 8 vectors produce IDENTICAL cost.
     >>> Selector ties always pick Vector 0 (3'b000 = ZERO VOLTAGE). Motor cannot build flux!
  ```

---

## Task 2: Discovery & Proof of New Hazards

### 2.1 Numerical Stability of Forward Euler: Observer Divergence at 1533 RPM & 3000 RPM
* **Continuous System vs Discrete Forward Euler**:
  At $f_s = 10\text{ kHz}$ ($T_s = 100\,\mu\text{s}$), the continuous motor state matrix is:
  $$\dot{x} = A(\omega_r) x + B v_s, \quad x = [i_\alpha, i_\beta, \psi_{r\alpha}, \psi_{r\beta}]^T$$
  Forward Euler discretizes the system as $x(k+1) = (I + A(\omega_r) T_s) x(k) + B T_s v_s(k)$.
* **Full $4 \times 4$ Physical System vs $2 \times 2$ Observer Subsystem**:
  - Full $4 \times 4$ coupled motor system: Euler stability boundary is $6505\text{ RPM}$ ($217\text{ Hz}$).
  - **CRITICAL HAZARD in [`flux_observer.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/flux_observer.v)**:
    `flux_observer.v` is NOT a closed-loop Luenberger observer. It has NO stator current error feedback $(i_s - \hat{i}_s)$. It is an open-loop rotor-flux current model driven solely by input $i_s$:
    $$\begin{bmatrix} \psi_{r\alpha}(k+1) \\ \psi_{r\beta}(k+1) \end{bmatrix} = \underbrace{\begin{bmatrix} E_{22} & -T_s \omega_r \\ T_s \omega_r & E_{22} \end{bmatrix}}_{A_{obs}} \begin{bmatrix} \psi_{r\alpha}(k) \\ \psi_{r\beta}(k) \end{bmatrix} + \begin{bmatrix} E_{21} i_\alpha(k) \\ E_{21} i_\beta(k) \end{bmatrix}$$
* **Eigenvalues of $A_{obs}$**:
  $$\det(\lambda I - A_{obs}) = (\lambda - E_{22})^2 + (T_s \omega_r)^2 = 0 \implies \lambda = E_{22} \pm j T_s \omega_r$$
  $$|\lambda|^2 = E_{22}^2 + (T_s \omega_r)^2$$
  Since $E_{22} = 1 - \frac{T_s}{\tau_r} \approx 0.999483$, the stability criterion $|\lambda| < 1.0$ requires:
  $$\omega_r < \sqrt{\frac{1 - E_{22}^2}{T_s^2}} \approx 321.05\text{ rad/s}$$
  For a 4-pole motor ($p = 2$), mechanical angular speed $\omega_m = \frac{\omega_r}{2} = 160.52\text{ rad/s}$:
  $$N_{crit} = \frac{160.52 \times 60}{2\pi} = \mathbf{1532.9\text{ RPM}} \quad (f_{elec} = 51.10\text{ Hz})$$
* **Behavior at 3000 RPM (Field-Weakening Region)**:
  At 3000 RPM, $\omega_r = 628.32\text{ rad/s} \implies |\lambda| = \mathbf{1.001461} > 1.0$!
  The observer homogeneous response grows exponentially without bound ($|\psi_r(k)| \propto 1.00146^k$).
* **Xsim End-to-End Failure Cascade Proof**:
  Simulating [`tb_cascade_proof.v`](file:///C:/Users/Dhananjay%20Dhumal/.gemini/antigravity-cli/brain/3ef05d1c-fa86-4cbe-874e-0d25afc48935/scratch/tb_cascade_proof.v) in Vivado xsim at 3000 RPM from nominal 0.96 Wb flux:
  ```
  Step 550 (t=55.00 ms): psi_alpha=-2246991, psi_beta=  -42195 -> Cost =  2147483647, Overflow = 1
  Step 608 (t=60.80 ms): psi_alpha= 2118591, psi_beta= 1222631 -> Cost =  2147483647, Overflow = 1
  Step 609 (t=60.90 ms): psi_alpha= 2040572, psi_beta= 1355293 -> Cost = -2137441096, Overflow = 0

  ================================================================================
  >>> CATASTROPHIC VERILOG INVERSION PROVEN AT STEP 609 (t = 60.9 ms):
      Flux magnitude = 2.336162 Wb (Threshold was 2.35 Wb / 4.53 Wb^2)
      Inverted Negative Cost = -2137441096 (32'h80993cb8)
      Overflow Flag = 0 (COMPLETELY SILENT FAILURE!)
  ================================================================================
  ```
  *In exactly $60.9\text{ ms}$ at 3000 RPM, the observer blows up, crosses $4.53\text{ Wb}^2$, causes bit-51 multiplication overflow, inverts cost to $-2.137\times 10^9$, clears the overflow flag, and latches the inverter into runaway.*

---

### 2.2 Cross-Coupling Sign Verification
* **Mathematical Derivation from Space Vectors**:
  In stationary frame, $\vec{\psi}_r = \psi_{r\alpha} + j \psi_{r\beta}$.
  $$\frac{d\vec{\psi}_r}{dt} = -\frac{1}{\tau_r} \vec{\psi}_r + \frac{L_m}{\tau_r} \vec{i}_s + j \omega_r \vec{\psi}_r$$
  $j \vec{\psi}_r = j(\psi_{r\alpha} + j \psi_{r\beta}) = -\psi_{r\beta} + j \psi_{r\alpha}$.
  Thus:
  $$\psi_{r\alpha}(k+1) = E_{21} i_\alpha + E_{22} \psi_{r\alpha} - T_s \omega_r \psi_{r\beta}$$
  $$\psi_{r\beta}(k+1) = E_{21} i_\beta + E_{22} \psi_{r\beta} + T_s \omega_r \psi_{r\alpha}$$
  In [`flux_observer.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/flux_observer.v#L117-L143):
  - Line 117: `psi_a_new <= term1 + term2 - term3;` where `term1 = E21*ia`, `term2 = E22*psi_a`, `term3 = TS_Q*wr_psi_beta`. **Sign is $-T_s \omega_r \psi_\beta$ (CORRECT)**.
  - Line 143: `psi_b_new <= term4 + term5 + term6;` where `term4 = E21*ib`, `term5 = TS_Q*wr_psi_alpha`, `term6 = E22*psi_b`. **Sign is $+T_s \omega_r \psi_\alpha$ (CORRECT)**.

  For stator currents:
  $$\sigma L_s \frac{d\vec{i}_s}{dt} = \vec{v}_s - R_s \vec{i}_s - \frac{L_m}{L_r} \frac{d\vec{\psi}_r}{dt} = \vec{v}_s - \left(R_s + \frac{L_m^2}{\tau_r L_r}\right) \vec{i}_s + \frac{L_m}{\tau_r L_r} \vec{\psi}_r - j \omega_r \frac{L_m}{L_r} \vec{\psi}_r$$
  Cross-coupling term: $- j \omega_r \frac{L_m}{L_r} (\psi_{r\alpha} + j \psi_{r\beta}) = \omega_r \frac{L_m}{L_r} (\psi_{r\beta} - j \psi_{r\alpha})$.
  - Stator current $\alpha$: $+ C_{13} \omega_r \psi_{r\beta}$.
  - Stator current $\beta$: $- C_{13} \omega_r \psi_{r\alpha}$.
  In [`motor_predictor.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/motor_predictor.v#L109-L143):
  - Line 109: `is_alpha_pred <= t1 + t2 + t3 + mul_result;` where `t3 = C13*wr_psi_beta`. **Sign is $+C_{13}\omega_r \psi_\beta$ (CORRECT)**.
  - Line 143: `is_beta_pred <= t5 - t6 + t7 + mul_result;` where `t6 = C13*wr_psi_alpha`. **Sign is $-C_{13}\omega_r \psi_\alpha$ (CORRECT)**.

---

### 2.3 Multiplier Pipeline Latency & Architectural Mismatch
* [`fixed_point_mul.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/fixed_point_mul.v#L13-L22):
  ```verilog
  always @(posedge clk) product_reg <= a * b;
  always @(posedge clk) result_reg <= product_reg[51:20];
  ```
  **Latency = 2 Clock Cycles** (requires 3 posedges from FSM input registration to output latching).
* **Actual RTL Implementation**:
  Every computational module (`flux_observer`, `motor_predictor`, `cost_evaluator`, `clarke_transform`, `speed_pi`) **completely bypasses `fixed_point_mul.v`**. Instead, they duplicate private inline multipliers:
  ```verilog
  always @(posedge clk) mul_result_full <= mul_a * mul_b;
  wire signed [31:0] mul_result = mul_result_full[51:20];
  ```
  **Latency = 1 Clock Cycle**.
* **Cycle Trace Audit**:
  Every FSM allocates exactly 1 wait cycle between setting `mul_a, mul_b` and reading `mul_result`.
  While the internal inline multiplier functions correctly with this 1-wait cycle FSM, **substituting `fixed_point_mul.v` into any module will immediately corrupt the pipeline**, as the FSM will sample premature, invalid data.

---

### 2.4 Additional Discovered Bugs: `speed_pi.v` and `mpc_top.v`
1. **[`speed_pi.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v#L27-L98) FSM Bit-Width Overflow**:
   - `reg [2:0] state;` (Can only represent states 0 to 7).
   - In state 7: `state <= 8;`.
   - In 3-bit arithmetic, `8 = 4'b1000` truncates to `3'b000` (`state <= 0`).
   - **State 8 is NEVER EXECUTED**.
   - State 8 contains the output saturation and assignment:
     ```verilog
     8: begin
         if (unclipped_te > `TE_MAX) te_ref <= `TE_MAX;
         ...
     ```
     Because state 8 is unreachable, `te_ref` is **NEVER UPDATED and remains stuck at 0**, and anti-windup clamping is completely bypassed!
2. **[`mpc_top.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v#L239-L251) Unconnected Torque Reference**:
   - `cost_evaluator` has input port `te_ref_in`.
   - In `mpc_top.v`, instance `u_cost` **omits the port `te_ref_in` entirely**!
   - In Verilog, unconnected input ports default to high-Z (`1'bz`). The entire cascaded speed loop is physically disconnected in the top-level RTL!

---

## Actionable Recommendations & Fix Roadmap

1. **Fix `cost_evaluator.v` Multiplier Overflow**:
   - Detect 64-bit overflow prior to truncation:
     ```verilog
     wire mul_overflow = (mul_a > 0 && mul_b > 0 && mul_result_full[63:51] != 13'd0) ||
                         (mul_result_full[63] != mul_result_full[51]);
     ```
   - If `mul_overflow` or `cost_f < 0`, immediately clamp `cost <= 32'sh7FFF_FFFF` and assert `cost_overflow <= 1`.
2. **Fix Standstill Deadlock in `optimal_selector.v` / Controller**:
   - Add open-loop DC pre-magnetization (pre-fluxing) sequence before releasing the FCS-MPC FSM, OR
   - Implement a stator-flux cost metric or 2-step prediction horizon so that $v_s(k)$ directly impacts the cost function at zero initial rotor flux.
   - Clamp selector comparison so negative costs are rejected as invalid.
3. **Stabilize Rotor Flux Observer (`flux_observer.v`)**:
   - Replace Forward Euler with Tustin (Bilinear) or Backward Euler for the cross-coupling rotation matrix, which preserves unconditional stability ($|\lambda| \le 1.0$) across all rotor speeds up to 3000+ RPM.
   - Alternatively, use closed-loop Luenberger feedback with stator voltage and current to stabilize high-speed flux estimation.
4. **Fix `speed_pi.v` and `mpc_top.v`**:
   - In `speed_pi.v`, widen state register: `reg [3:0] state;`.
   - In `mpc_top.v`, instantiate `speed_pi` and connect its output `te_ref` to `u_cost.te_ref_in`.
