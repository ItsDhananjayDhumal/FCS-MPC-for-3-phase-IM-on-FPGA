# FCS-MPC for 3-Phase Induction Motors: Dev Guide

This document is the ultimate technical reference for the Verilog implementation of this Model Predictive Control (MPC) project. It is designed to take a new developer through the exact orchestration, data paths, finite state machines (FSMs), and pipeline timing of every single module in the project.

---

## 1. Global Architecture & Orchestration

The system runs on a 100 MHz clock (`sys_clk_i`) and targets a Digilent Nexys Video (Artix-7). The entire MPC algorithm is synchronized to a rigid sampling period ($T_s$) of $100 \mu s$ (10 kHz). 

Every 10,000 clock cycles, the `control_fsm.v` (the master orchestrator) wakes up and sequences the following modules in order:
1.  **ADC Read**: Fetches phase A/B currents via SPI (`adc_pmod_ad1.v`).
2.  **Clarke Transform**: Converts $I_{a,b}$ to $I_{\alpha, \beta}$ (`clarke_transform.v`).
3.  **Flux Observer**: Estimates the current rotor flux (`flux_observer.v`).
4.  **MPC Loop (8x Iterations)**: 
    *   Predicts future state for Vector $V_n$ (`motor_predictor.v`).
    *   Evaluates the cost of Vector $V_n$ (`cost_evaluator.v`).
    *   Compares the cost and saves the minimum (`optimal_selector.v`).
5.  **Apply**: Outputs the winning vector to the PWM equivalent (`gate_driver.v`).

---

## 2. Fixed-Point Arithmetic (Q12.20)

All mathematical modules utilize a signed 32-bit Q12.20 fixed-point format (`DATA_WIDTH = 32`, `FRAC_BITS = 20`).
*   **Sign bit**: 1 bit
*   **Integer part**: 11 bits (Range: -2048 to +2047)
*   **Fractional part**: 20 bits (Resolution: $\approx 10^{-6}$)

### Multiplication Paradigm
Multiplying two Q12.20 numbers yields a Q24.40 result (64 bits). To return to Q12.20, the 64-bit result must be right-shifted by 20 bits. In Verilog, this looks like:
```verilog
wire signed [2*DATA_WIDTH-1:0] full = mul_a * mul_b;
wire signed [DATA_WIDTH-1:0] trunc = full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
```
*Crucial Dev Note*: Because 32x32 multiplications are computationally heavy, modules that execute multiple equations (like `motor_predictor.v`) share a *single* pipelined multiplier. This forces the FSMs to wait 1 clock cycle for the product to be valid.

---

## 3. Module Deep Dives

### `mpc_params.vh` (Global Constants)
This is the central configuration file. It contains:
*   Physical motor bounds and pre-calculated Euler discretization coefficients ($C_{11}, C_{12} \dots$).
*   `PI_KP` and `PI_KI` for the outer speed loop.
*   Protection thresholds (`MAX_CURRENT_RAW`).
*   *Dependency*: Included by almost every RTL file. Change a motor parameter here, and the entire math pipeline automatically adapts.

### `mpc_top.v` (Top-Level Wrapper)
*   **Function**: Instantiates all sub-modules and wires them together.
*   **Logic**: Contains a 2-stage asynchronous-assert, synchronous-deassert reset synchronizer (`rst_sync1`, `rst_sync2`) to prevent metastability on boot.

### `control_fsm.v` (The Master Brain)
*   **Function**: Sequences the algorithm.
*   **Dependencies**: Triggers all computational blocks via `start` pulses and waits for their respective `done` flags.
*   **FSM Walkthrough**:
    *   `S_WAIT_TS`: Idles until the 10,000-cycle counter rolls over. Asserts `sample_tick` to update the speed PI loop.
    *   `S_START_ADC` / `S_WAIT_ADC`: Pulses `adc_start`. When `adc_done` goes high, it checks for Overcurrent. If $I_a$ or $I_b$ exceed `MAX_CURRENT_RAW`, it branches to `S_ERROR`.
    *   `S_CLARKE` $\rightarrow$ `S_FLUX`: Converts currents, then runs the observer.
    *   `S_MPC_INIT`: Resets the `optimal_selector` and zero-indexes the vector counter (`vec_idx`).
    *   `S_MPC_PREDICT` $\rightarrow$ `S_MPC_COST` $\rightarrow$ `S_WAIT_SEL`: Runs the predictor and cost evaluator. The `S_WAIT_SEL` state acts as a 1-cycle breather to allow the `optimal_selector`'s synchronous logic to latch the cost and assert `sel_done`.
    *   `S_MPC_NEXT`: Checks if `vec_idx == 7`. If not, increments `vec_idx` and loops back to `S_MPC_PREDICT`. If yes, goes to `S_APPLY`.
    *   `S_APPLY`: Flushes the optimal vector to the `gate_driver.v`.
    *   `S_ERROR`: A safety latch. Forces all gates to 0. Can only be exited by toggling the physical `enable` switch off and on.
*   **Watchdog**: A background safety block checks if the internal `ts_counter` reaches 9900 while the FSM is still processing. If so, a module hung (failed to pulse `done`), and the FSM forces an `S_ERROR`.

### `speed_pi.v` (Outer Control Loop)
*   **Function**: Calculates the required torque (`te_ref`) to achieve the target `speed_ref`.
*   **Logic**: Runs on a 9-state pipeline triggered by `sample_tick`.
*   **Pipeline Details**:
    1.  Calculates $error = \omega_{ref} - \omega_{fb}$.
    2.  Integrates error: $err\_integ = err\_integ + error$.
    3.  Shares a pipelined multiplier to compute $P_{term} = K_p \cdot error$ and $I_{term} = K_i \cdot err\_integ$.
    4.  Adds them together and applies hard saturation at `TE_MAX` and `TE_MIN`.

### `encoder_reader.v` (Position & Speed)
*   **Function**: Decodes quadrature signals (A/B/Z) and computes filtered electrical speed.
*   **Dependencies**: Uses `SPEED_SCALE` and `SPEED_ALPHA` from params.
*   **Logic**:
    *   Passes inputs through 3-stage flip-flop synchronizers to prevent metastability.
    *   Tracks state transitions to increment/decrement `position`. Resets `position` to 0 if `enc_z` goes high.
    *   **Pipelined Math**: On `sample_tick`, it computes $\Delta position$. Over the next 3 clock cycles, it multiplies the delta by `SPEED_SCALE` and applies an IIR low-pass filter ($\omega_{new} = \alpha \cdot \omega_{raw} + (1-\alpha) \cdot \omega_{old}$).

### `motor_predictor.v` (The Heavy Lifter)
*   **Function**: Solves the discrete motor equations to predict $i_{s}$ and $\psi_r$ one step into the future.
*   **Dependencies**: Requires valid outputs from `clarke_transform`, `flux_observer`, and `vdc_manager`.
*   **FSM Pipeline**: This is a dense 29-state FSM. It utilizes a single registered multiplier.
    *   *The Pattern*: In state $N$, it sets `mul_a` and `mul_b`. In state $N+1$, it does absolutely nothing (waits for the multiplier flip-flops to latch). In state $N+2$, it reads `mul_result` and routes it into an accumulator (e.g., `t1 <= mul_result`), while simultaneously setting the inputs for the next multiplication.
    *   *Modification*: If you need to add an equation here, you MUST respect this 2-cycle cadence or you will read stale multiplier data.

### `cost_evaluator.v` (Optimization Metric)
*   **Function**: Calculates $J = \lambda_T (T_e^* - T_e^p)^2 + \lambda_\psi (|\psi_r^*|^2 - |\psi_r^p|^2)^2$.
*   **FSM Pipeline**: Uses the exact same 2-cycle wait-state pattern as the predictor.
*   **Saturation Logic**: The final addition (`cost_t + cost_f`) is prone to exceeding the signed 32-bit limit (`2,147,483,647`). A dedicated combinatorial check forces `cost` to `32'sd2147483647` if an overflow is detected, preventing the math from wrapping around to a negative number (which would falsely trick the selector into thinking an awful vector is actually optimal).

### `optimal_selector.v` (Min-Finder)
*   **Function**: Remembers the best vector across the 8 iterations.
*   **Logic**: Resets `min_cost` to the maximum possible positive integer when `reset_search` is pulsed. Every time `cost_valid` pulses, it compares the incoming `cost` to `min_cost`. If it's strictly less, it saves the cost and the corresponding `switch_state`. Once `eval_count` hits 7, it pulses `done`.

### `vdc_manager.v` (Voltage scaling)
*   **Function**: Debounces manual switches to set DC bus voltage, and calculates $\frac{2}{3}V_{dc}$, $\frac{1}{3}V_{dc}$, and $\frac{1}{\sqrt{3}}V_{dc}$ for the voltage lookup table.
*   **Startup Logic**: Utilizes a `startup_done` flag. At boot, it bypasses the physical switches and hardcodes a 311V output. It holds this output for 10ms until the debounce counter guarantees the physical switches have stopped bouncing, preventing dangerous 0V calculations at startup.

### `gate_driver.v` (PWM & Dead-Time)
*   **Function**: Converts the optimal 3-bit vector (e.g., `3'b100`) into 6 individual gate signals with inserted dead-time.
*   **Logic**:
    *   When the target state changes, it immediately shuts off the active transistor in the half-bridge.
    *   It starts a 12-bit counter `dt_cnt_a`.
    *   Only when `dt_cnt_a` exceeds `DEAD_TIME_CYCLES` (e.g., 200 cycles = 2 $\mu s$) does it turn on the complementary transistor.
    *   This is evaluated completely independently for Phase A, B, and C.

---

## 4. How to Modify the Project

### Adding a new motor parameter
1. Calculate its continuous and discrete values.
2. Add it to `mpc_params.vh`. Ensure you document the Q12.20 shifted value (e.g., `value * 1048576`).

### Modifying Predictive Equations
If you upgrade from Euler Forward to a 2nd-order Runge-Kutta, you must completely rewrite the FSM in `motor_predictor.v`. You will likely run out of clock cycles if you use a single multiplier. You will need to instantiate a second `mul_result_full` register to perform two multiplications in parallel.

### Changing the Sampling Rate
Change `SWITCHING_FREQ` in `mpc_params.vh`. Ensure that `TS_COUNTER_MAX` leaves enough clock cycles (currently 10,000) to execute the entire FSM chain. The current algorithm takes approximately 500 clock cycles to execute. You can safely increase the switching frequency up to roughly 150 kHz before the FSM starts missing deadlines.
