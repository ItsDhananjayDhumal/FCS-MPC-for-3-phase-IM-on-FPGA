# Independent Re-Verification Report: FCS-MPC 3-Phase IM Drive
## Final Verdict Audit — Round 4

**Date:** September 25, 2026  
**Methodology:** 5 independent subagents, each reading actual RTL code from scratch, performing cycle-by-cycle register tracing, and cross-referencing Verilog IEEE 1364 semantics.  
**Constraint:** Zero code modifications. Report only.

---

## 1. Executive Summary

22 verdicts from 3 prior audit rounds were independently re-examined. Results:

| Outcome | Count | Details |
|:---|:---:|:---|
| **Confirmed TRUE (real bug)** | 18 | All critical/fatal bugs verified as genuine |
| **Confirmed FALSE ALARM** | 2 | INV_SQRT3 discrepancy, ADC quiet time — correctly dismissed |
| **Partially Corrected** | 1 | speed_pi.v integrator overflow exists, but does NOT cause reverse torque |
| **Numerically Corrected** | 1 | MPC loop latency is 629 cycles, not 619 |

> [!IMPORTANT]
> **No false red flags were found among the CRITICAL/FATAL verdicts.** Every single hardware-destructive or control-crippling bug from previous rounds was independently confirmed as a genuine defect in the current RTL.

> [!NOTE]
> **One previous conclusion was partially wrong:** The speed_pi integrator overflow was correctly identified as a real bug (it does overflow in 21 cycles), but the claimed consequence ("commands maximum reverse torque") is **FALSE**. The proportional term dominates, keeping the output positive. The real danger is that after the error resolves, the wrapped-negative integrator creates a prolonged torque undershoot during unwinding.

---

## 2. Complete Verdict Matrix

### 2.1 Power Stage & Gate Driver

| # | Module | Verdict | Prior Status | Re-Verified Status | Notes |
|:---:|:---|:---|:---|:---|:---|
| 1 | [`gate_driver.v` L46](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/gate_driver.v#L46) | Enable-bounce shoot-through | CONFIRMED FATAL | ✅ **TRUE** | `current_state <= 3'b000` erases state history. Re-enable bypasses dead-time entirely. Low MOSFET asserts in 10ns while High is still conducting (~800ns toff). |
| 2 | [`gate_driver.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/gate_driver.v) | 37/64 transitions cause shoot-through | CONFIRMED | ✅ **TRUE** | Mathematically proven: safe pairs = 3³ = 27, total = 64, hazardous = 37 (57.8%) |
| 3 | [`mpc_top.v` L51-61](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v#L51) | No pushbutton debounce | CONFIRMED | ✅ **TRUE** | 2-FF synchronizer prevents metastability but not bounce (ms-scale). Directly triggers shoot-through on every button press/release. |
| 4 | [`gate_driver.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/gate_driver.v) | Dead-time `==` vs `>=` | CONFIRMED HAZARD | ✅ **TRUE** | SEU skip causes counter to roll over to 4095 before re-matching. Using `>=` is standard robust practice. |
| 5 | [`control_fsm.v` L253](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/control_fsm.v#L253) | S_ERROR low-side brake + zombie trap | CONFIRMED | ✅ **TRUE** | `3'b000` turns ON all low-side MOSFETs (dynamic brake, not safe coast). Heartbeat is decoupled from FSM state — blinks normally during fault, masking the error. |

### 2.2 Current Sensing & ADC

| # | Module | Verdict | Prior Status | Re-Verified Status | Notes |
|:---:|:---|:---|:---|:---|:---|
| 6 | [`adc_pmod_ad1.v` L85](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/adc_pmod_ad1.v#L85) | 1-bit ADC shift (val >> 1) | CONFIRMED | ✅ **TRUE** | 2-FF sync delay (20ns) + AD7476A access time (22ns) = 42ns total. But SCLK rise-to-fall is only 40ns. Data captured is the previous bit, right-shifting entire word. Max readable = 2047. |
| 7 | [`clarke_transform.v` L35](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/clarke_transform.v#L35) | 1,048,576× current attenuation | CONFIRMED FATAL | ✅ **TRUE** | `ia_signed` is Q32.0 × `ADC_SCALE` (Q12.20) = Q44.20 product. Extracting `[51:20]` discards the 20 fractional bits, leaving integer only. 5A → value `5` (interpreted as 4.77µA). |
| 8 | [`control_fsm.v` L141](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/control_fsm.v#L141) | Overcurrent trip can never fire | CONFIRMED | ✅ **TRUE** | ADC capped at 2047 due to verdict #6. Threshold `> 3800` is unreachable. System is 100% blind to overcurrent. |
| 9 | [`clarke_transform.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/clarke_transform.v) | INV_SQRT3 discrepancy (605510 vs 605396) | FALSE ALARM | ✅ **Confirmed False Alarm** | 114 LSB error = 0.000108 absolute = 0.019%. 10× below ADC quantization noise. Benign. |

### 2.3 Encoder & Speed Control

| # | Module | Verdict | Prior Status | Re-Verified Status | Notes |
|:---:|:---|:---|:---|:---|:---|
| 10 | [`encoder_reader.v` L100](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L100) | Unsigned part-select corrupts reverse speed | CONFIRMED FATAL | ✅ **TRUE** | IEEE 1364: part-select is always unsigned. `-1` → `4,294,967,295`. After multiply+slice, outputs `-1776.13 rad/s` instead of `-12.57 rad/s`. |
| 11 | [`encoder_reader.v` L73](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v#L73) | Double fractional division (10⁶× attenuation) | CONFIRMED FATAL | ✅ **TRUE** | `SPEED_SCALE` is already Q12.20. Product is already correct Q12.20. `[51:20]` slice divides by 2²⁰ again. |
| 12 | [`encoder_reader.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/encoder_reader.v) | Z-index velocity spike (-125,663 rad/s) | CONFIRMED | ✅ **TRUE** | Position reset 9999→0 creates delta = -9999. No rollover masking. Spike propagates directly to flux observer. |
| 13 | [`speed_pi.v` L37](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v#L37) | State truncation (reg [2:0] for state 8) | CONFIRMED FATAL | ✅ **TRUE** | `8` = `4'b1000` truncates to `3'b000`. State 8 unreachable. `te_ref` permanently frozen at 0. |
| 14 | [`speed_pi.v` L53](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/speed_pi.v#L53) | Integrator overflow (21 cycles) | CONFIRMED | ⚠️ **TRUE but consequence CORRECTED** | Overflow is real: `err_integ` wraps negative in ~21 ticks. **However, the proportional term (`p_term = err * KP >> 20 ≈ +524M`) massively dominates the negative integrator (`i_term ≈ -215M`). Net output remains positive and clamps to `TE_MAX`.** The real danger is during error recovery — the wrapped integrator creates prolonged torque undershoot as it unwinds, not reverse torque. |

### 2.4 MPC Core Mathematics

| # | Module | Verdict | Prior Status | Re-Verified Status | Notes |
|:---:|:---|:---|:---|:---|:---|
| 15 | [`cost_evaluator.v` L157](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/cost_evaluator.v#L157) | Cost overflow wraps negative (overruled from FA) | OVERRULED → CRITICAL | ✅ **TRUE** | When `LAMBDA_PSI × flux_err_sq` exceeds bit 51, `mul_result` wraps negative. The overflow check fails because it tests `mul_result > 0` which is already FALSE. Cost goes to ~-2×10⁹, causing selector to latch worst vector. |
| 16 | [`mpc_top.v` L238-251](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v#L238) | Floating `te_ref_in` (unconnected port) | CONFIRMED FATAL | ✅ **TRUE** | Port simply omitted from instantiation. In simulation: `'bz` → `'bx` propagation. In synthesis: likely tied to 0 or optimized unpredictably. |
| 17 | [`optimal_selector.v` L37](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/optimal_selector.v#L37) | Standstill deadlock (all 8 costs identical) | CONFIRMED | ✅ **TRUE** | At standstill, voltage has NO feedthrough to predicted rotor flux (1-step prediction). All 8 costs are identical. Strict `<` keeps Vector 0. Motor can never self-start. |
| 18 | [`flux_observer.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/flux_observer.v) | Forward Euler instability above 1535 RPM | CONFIRMED | ✅ **TRUE** | `|λ|² = E22² + (TS_Q·ωr)²`. At ωr > 321.3 rad/s (1534 RPM mechanical), poles exit unit circle. Observer diverges exponentially. |

### 2.5 System-Level & Testbench

| # | Module | Verdict | Prior Status | Re-Verified Status | Notes |
|:---:|:---|:---|:---|:---|:---|
| 19 | [`vdc_manager.v` L70](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/vdc_manager.v#L70) | Cycle 9 startup zeroing (Vdc=0) | CONFIRMED | ✅ **TRUE** | On cycle 9, `vdc_integer=0` ≠ `vdc_int_reg=311` → overwrites to 0 before debounce completes. |
| 20 | [`vdc_manager.v` L28](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/vdc_manager.v#L28) | DIP switch caps at 405V (can't reach 540V) | CONFIRMED | ✅ **TRUE** | `15×25 + 15×2 = 405V` max. 540V for 380V motor is unreachable. |
| 21 | [`mpc_top.v` L122](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl/mpc_top.v#L122) | fsm_state_w 4-bit truncation | CONFIRMED | ✅ **TRUE (debug only)** | S_WAIT_SEL(16)→0, S_ERROR(17)→1. Confirmed as debug-probe-only; no functional logic impact. |
| 22 | [`tb_mpc_top.v`](file:///C:/Users/Dhananjay%20Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sim_1/imports/tb/tb_mpc_top.v) | Vacuous testbench assertions | CONFIRMED | ✅ **TRUE** | Only 3.8ms runtime (< 10ms debounce). Tests 2-4 have zero assertions. Reports PASS despite gates changing only twice. |
| 23 | System | MPC loop latency | 619 cycles | 🔧 **CORRECTED → 629 cycles** | Prior count missed 10 cycles of Clarke transform latency. Actual: 629 cycles (6.29µs). Watchdog margin remains healthy at 93.6%. |

---

## 3. Corrections to Previous Audit Conclusions

### 3.1 Speed PI Integrator: Consequence Was Overstated

> [!WARNING]
> **Previous claim:** "Integrator overflow commands -20 Nm reverse torque while motor is below setpoint"  
> **Corrected finding:** The integrator DOES overflow in ~21 cycles (this is a real bug), but the proportional term completely dominates:
> - `p_term = (err × KP) >> 20 ≈ +524,288,000` (massive positive)
> - `i_term = (wrapped_err_integ × KI) >> 20 ≈ -214,748,364` (negative)  
> - `unclipped_te = p_term + i_term ≈ +309,500,000` → clamped to `TE_MAX` (+20,971,520)
>
> **Real danger:** When speed error finally resolves (speed reaches reference), the wrapped-negative integrator takes many cycles to unwind through zero. During this unwinding period, the controller will experience a torque undershoot — commanding less torque than needed — causing speed overshoot followed by damped oscillation. This is a control quality issue, not a hardware safety issue.

### 3.2 MPC Loop Latency: 629 Cycles, Not 619

The prior audit undercounted by 10 cycles. Corrected breakdown:

| Stage | Cycles | Cumulative |
|:---|:---:|:---:|
| FSM overhead + ADC start | 2 | 2 |
| ADC SPI execution | 128 | 130 |
| FSM wait → Clarke | 2 | 132 |
| Clarke transform | 10 | 142 |
| FSM wait → Flux | 2 | 144 |
| Flux observer | 18 | 162 |
| FSM wait → MPC | 2 | 164 |
| 8× MPC loop (Pred:28 + FSM:2 + Cost:23 + FSM:5) | 464 | 628 |
| Selector + Apply | 1 | 629 |

Watchdog at 9900 cycles → margin = 9271 cycles (93.6%). No timing risk.

---

## 4. Verified False Alarms (Correctly Dismissed)

These two items were previously marked as False Alarms and this audit **confirms they are indeed benign**:

1. **`INV_SQRT3` constant discrepancy (605510 vs 605396):** +114 LSB = 0.019% error. 10× below ADC quantization noise. ✅ Benign.
2. **ADC quiet time violation:** System FSM gives 98.7µs CS high time between conversions, 1147× above the 86ns minimum. ✅ Benign.

---

## 5. Summary of All Confirmed Real Bugs by Severity

### CATASTROPHIC (Hardware Destruction Risk)
1. **Gate driver shoot-through** on enable bounce (37/64 transitions affected)
2. **No pushbutton debounce** — triggers shoot-through on every button press
3. **Overcurrent protection 100% blind** — ADC capped at 2047, threshold at 3800
4. **S_ERROR dynamic brake** — shorts stator windings of spinning motor

### FATAL (System Non-Functional)
5. **Clarke transform 2²⁰× attenuation** — current feedback reads as microamps
6. **Encoder speed 2²⁰× attenuation** — speed feedback nearly zero
7. **Unsigned encoder part-select** — reverse speed corrupted to -1776 rad/s
8. **Speed PI state truncation** — te_ref permanently stuck at 0
9. **Floating te_ref_in** — cost evaluator gets 'bx, selector locked on Vector 0
10. **Standstill deadlock** — all 8 costs identical, motor can never self-start

### CRITICAL (Operational Limit / Transient Hazard)
11. **Cost evaluator overflow** wraps cost negative, selector latches worst vector
12. **Forward Euler instability** above 1535 RPM — observer diverges exponentially
13. **Z-index velocity spike** — -125,663 rad/s shockwave once per revolution
14. **DIP switch 405V cap** — cannot represent 540V for 380V motor
15. **Startup Vdc zeroing** — voltage resets to 0 on cycle 9

### HIGH (Control Quality / Robustness)
16. **Speed PI integrator overflow** — torque undershoot during error recovery
17. **Dead-time counter `==` vs `>=`** — SEU vulnerability
18. **S_ERROR zombie trap** — heartbeat masks fault state
19. **Testbench vacuous assertions** — PASS reported despite non-functional gates

---

## 6. Conclusion

> [!CAUTION]
> This independent re-verification confirms that **every CRITICAL and FATAL verdict from the previous three audit rounds was accurate**. No false red flags were found among the hardware-safety or control-critical findings.
>
> The only corrections are:
> 1. The speed_pi integrator overflow consequence was overstated (torque undershoot, not reverse torque)
> 2. The MPC loop latency is 629 cycles, not 619 (no functional impact)
>
> **The RTL in its current state will destroy the power stage hardware on the first button press** due to the combination of zero debounce + enable-bounce shoot-through. Even if that were fixed, the current/speed sensing pipeline is fundamentally broken (2²⁰× attenuation on both channels), the speed PI can never output a torque reference (state truncation), and the motor cannot self-start (standstill deadlock).
