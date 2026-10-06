# Adversarial Re-Audit Report: ADC Interface & Clarke Transform
**Target Subsystems:** `adc_pmod_ad1.v`, `clarke_transform.v`, `control_fsm.v`, `mpc_params.vh`  
**Engineer:** Senior Digital Signal Processing & Mixed-Signal Verification Engineer  
**Date:** September 22, 2026  
**Simulation Tool:** Vivado Simulator (XSim v2023.2)  

---

## Executive Summary & Adversarial Verdict Matrix

An adversarial verification audit was executed against the analog front-end and coordinate transformation pipeline of the FCS-MPC 3-Phase Induction Motor drive. All previous audit verdicts were rigorously re-examined against manufacturer silicon specifications (Analog Devices AD7476A and Allegro ACS712-40AB datasheets), register-transfer logic, and cycle-accurate Vivado XSim testbenches.

| Module / Interface | Concern / Hypothesis | Previous Verdict | Adversarial Re-Audit Verdict | Severity | Key Empirical Evidence |
|---|---|---|---|---|---|
| **`adc_pmod_ad1.v`** | 1-bit right shift ($val \gg 1$) due to phase lag / sync delay | Bug (Critical) | **CONFIRMED REAL FATAL BUG** | **CRITICAL** | Swept $t_{access}$ in XSim: at $t_{access} \ge 20\text{ ns}$ (AD7476A is $25\text{ ns}$ typ, $40\text{ ns}$ max), capture slips to Bit $N-1$. Bit 11 is permanently zero. |
| **`adc_pmod_ad1.v`** | Quiet time $t_{QUIET}$ violation (50ns vs 86ns) | False Alarm | **CONFIRMED FALSE ALARM** | **BENIGN** | System FSM triggers at 10 kHz; CS stays HIGH for $98.67\,\mu\text{s}$ ($1,147\times$ the required 86ns). |
| **`clarke_transform.v`** | $1,048,576\times$ current attenuation on `mul_result` | Bug (Fatal) | **CONFIRMED REAL FATAL BUG** | **FATAL** | Multiplier product has only 20 fractional bits (Q44.20). Slicing `[51:20]` applies double $2^{20}$ division, attenuating 5.0A to $4.77\,\mu\text{A}$. Proper slice is `[31:0]`. |
| **`clarke_transform.v`** | `INV_SQRT3` constant error (605510 vs 605396) | False Alarm | **CONFIRMED FALSE ALARM** | **BENIGN** | Error is $+114$ LSBs ($0.94\text{ mA}$ at 5A), which is $17\times$ below ADC quantization noise (16.11 mA). |
| **`clarke_transform.v`** | Overflow of `temp <= ia + (ib <<< 1)` under ADC clamping | New Audit Point | **PROVEN MATHEMATICALLY IMPOSSIBLE** | **CLEAN** | Even under worst-case rail clamping ($i_{raw} \in \{0, 4095\}$), $|temp| \le 103,809,024$, consuming only $4.8\%$ of 32-bit signed range ($21\times$ margin). |
| **`adc` $\to$ `control_fsm`** | Overcurrent protection blindness & negative current lockup | New Audit Point | **NEW CRITICAL SYSTEM DEFECT** | **CRITICAL** | Because of the 1-bit ADC shift, `data_ch0` cannot exceed 2047 (`12'h7FF`). `MAX_CURRENT_RAW` (3800) is unreachable; overcurrent protection is 100% blind, and all currents evaluate as negative. |
| **Sensor / ADC Mixed-Signal** | ACS712 (5V) to Pmod AD1 (3.3V) Asymmetric Saturation | New Audit Point | **NEW HARDWARE WARNING** | **HIGH** | ACS712 2.510V 0A offset reaches 3.3V at only $+15.8\text{ A}$, giving asymmetric range ($-40\text{ A}$ to $+15.8\text{ A}$) and ESD rail-injection hazard. |

---

## 1. Adversarial Deep-Dive: ADC Timing & 1-Bit Shift in `adc_pmod_ad1.v`

### 1.1 Silicon Timing Mechanics (AD7476A Datasheet)
The Analog Devices AD7476A 12-bit SAR ADC operates as follows:
1. **Conversion Initiation:** Bringing $\overline{\text{CS}}$ low initiates conversion and switches track-and-hold into hold mode. Simultaneously, the device brings SDATA out of three-state and asserts the **first leading zero** (propagation time $t_4 \le 22\text{ ns}$).
2. **Clocking:** SCLK controls both conversion and serial data shifting. The device shifts out subsequent bits on the **falling edges of SCLK**.
3. **Data Framing:** A full transmission consists of **4 leading zeros** followed by **12 conversion bits (DB11 down to DB0)**, total 16 clock cycles.

### 1.2 RTL Analysis of `adc_pmod_ad1.v`
In `adc_pmod_ad1.v`:
```verilog
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_div <= 3'd0;
        adc_sclk <= 1'b0;
    end else if (sclk_en) begin
        clk_div <= clk_div + 3'd1;
        if (clk_div == 3'd3) adc_sclk <= 1'b1;
        else if (clk_div == 3'd7) adc_sclk <= 1'b0;
    end ...
end
wire sclk_rise = (clk_div == 3'd3) && sclk_en;
wire sclk_fall = (clk_div == 3'd7) && sclk_en;
```
- The 100 MHz clock period is $T = 10\text{ ns}$.
- `clk_div` counts from 0 to 7 (80 ns period $\implies 12.5\text{ MHz}$ SCLK).
- `adc_sclk` is **LOW for counts 0..3 (40 ns)** and **HIGH for counts 4..7 (40 ns)**.
- `sclk_rise` occurs at count 3: on the next posedge (`3` $\to$ `4`), `adc_sclk` rises to 1, and the shift register samples:
  ```verilog
  if (sclk_rise) begin
      shift_ch0 <= {shift_ch0[14:0], d0_in};
  ```
- `sclk_fall` occurs at count 7: on the next posedge (`7` $\to$ `0`), `adc_sclk` falls to 0, and `bit_cnt` increments:
  ```verilog
  else if (sclk_fall) begin
      bit_cnt <= bit_cnt + 5'd1;
  ```

### 1.3 Exact Propagation Delay & Synchronizer Latency
Let $t = 0\text{ ns}$ be the posedge clk where `adc_sclk` transitions $1 \to 0$ (falling edge):
1. **FPGA Pin & Board Egress:** SCLK falling edge exits FPGA and arrives at AD7476A pin at $t \approx 3.5\text{ ns}$ to $5.0\text{ ns}$.
2. **AD7476A Data Access Time ($t_5$):** The AD7476A outputs new serial data after SCLK falling edge.
   - Typical: $25\text{ ns}$.
   - Maximum ($V_{DD} = 3\text{ V}$): $40\text{ ns}$.
   - Minimum hold time ($t_7$): $10\text{ ns}$.
3. **Board Return & FPGA Pin Ingress:** SDATA returns to FPGA pin `adc_d0` at:
   - Typical: $t = 4\text{ ns} + 25\text{ ns} + 2\text{ ns} = \mathbf{31\text{ ns}}$.
   - Worst case: $t = 4\text{ ns} + 40\text{ ns} + 2\text{ ns} = \mathbf{46\text{ ns}}$.
4. **FPGA 2-Stage Synchronizer Latency:**
   ```verilog
   always @(posedge clk) begin
       d0_sync <= {d0_sync[0], adc_d0};
   end
   wire d0_in = d0_sync[1];
   ```
   At $t = 40\text{ ns}$ (the `sclk_rise` sampling edge):
   - The non-blocking assignment evaluates `d0_in = d0_sync[1]`.
   - `d0_sync[1]` was updated at $t = 30\text{ ns}$ from `d0_sync[0]`.
   - `d0_sync[0]` was updated at $t = 20\text{ ns}$ from the external pin `adc_d0`!
   - Therefore, at $t = 40\text{ ns}$, **`shift_ch0` captures the pin state that existed at $t = 20\text{ ns}$!**

### 1.4 The Inescapable Slip to Bit $N-1$
- At $t = 20\text{ ns}$ after the SCLK falling edge, the AD7476A has NOT updated its output pin (typical arrival is $31\text{ ns}$, worst-case $46\text{ ns}$).
- Therefore, at $t = 20\text{ ns}$, the pin is **guaranteed to hold the old bit (Bit $N-1$)**.
- On rising edge #1, SCLK has not fallen yet; it samples Leading Zero #1.
- On rising edge #2, it samples Leading Zero #1 *again* because Leading Zero #2 has not traversed the synchronizer.
- Every bit is shifted by 1 clock cycle:
  - Bit 15 of shift register gets Leading Zero #1
  - Bit 14 gets Leading Zero #1
  - Bit 13 gets Leading Zero #2
  - Bit 12 gets Leading Zero #3
  - Bit 11 gets Leading Zero #4
  - Bit 10 gets DB11 (MSB)
  - ...
  - Bit 0 gets DB1
  - **DB0 is NEVER sampled!**
- Output assignment: `data_ch0 <= shift_ch0[11:0]`.
  `shift_ch0[11:0]` is `{1'b0, DB11, DB10, ..., DB1}`, which equals:
  $$\mathbf{\text{data\_ch0} = \text{ADC\_WORD} \gg 1}$$

### 1.5 Cycle-Accurate Vivado Simulation Proof
In `scratch/tb_adc_sweep.v`, we swept $t_{access}$ from 0 to 35 ns in 5 ns steps:
```text
========================================================
SWEEPING t_access FROM 0 ns TO 35 ns IN 5 ns STEPS
Expected Data: 12'hA5C (2652) | Shifted Data: 12'h52E (1326)
========================================================
t_access =  0 ns | Captured = 12'ha5c (2652) | Status = CORRECT (Bit N)
t_access =  5 ns | Captured = 12'ha5c (2652) | Status = CORRECT (Bit N)
t_access = 10 ns | Captured = 12'ha5c (2652) | Status = CORRECT (Bit N)
t_access = 15 ns | Captured = 12'ha5c (2652) | Status = CORRECT (Bit N)
t_access = 20 ns | Captured = 12'h52e (1326) | Status = DEFECTIVE: SHIFTED (Bit N-1)
t_access = 25 ns | Captured = 12'h52e (1326) | Status = DEFECTIVE: SHIFTED (Bit N-1)
t_access = 30 ns | Captured = 12'h52e (1326) | Status = DEFECTIVE: SHIFTED (Bit N-1)
t_access = 35 ns | Captured = 12'h52e (1326) | Status = DEFECTIVE: SHIFTED (Bit N-1)
```
**Conclusion:** On real hardware ($t_{access} = 25\text{ ns to } 40\text{ ns}$), the shift register is deep in the defective region ($> 20\text{ ns}$). The previous verdict is **100% CONFIRMED**.

---

## 2. Adversarial Deep-Dive: Fixed-Point Format in `clarke_transform.v`

### 2.1 Fixed-Point Format Derivation
In `clarke_transform.v`:
```verilog
ia_signed <= $signed({1'b0, ia_raw}) - $signed(`ADC_OFFSET);
...
mul_a <= ia_signed;
mul_b <= `ADC_SCALE;
...
mul_result_full <= mul_a * mul_b;
wire signed [DATA_WIDTH-1:0] mul_result = mul_result_full[DATA_WIDTH+FRAC_BITS-1 : FRAC_BITS];
```
1. `ia_signed` is an integer count: format is **Q32.0** (0 fractional bits).
2. `ADC_SCALE` represents Amperes per LSB in standard control units: format is **Q12.20** (20 fractional bits).
3. The full product is:
   $$\text{mul\_result\_full} = \text{Q32.0} \times \text{Q12.20} = \mathbf{\text{Q44.20}}$$
   - Bits `[19:0]` are the **20 fractional bits**.
   - Bits `[63:20]` are the integer and sign bits.
4. To output a 32-bit signed Q12.20 value:
   - Fractional bits: `[19:0]`
   - Integer/sign bits: `[31:20]`
   - **The exact, mathematically required slice is:**
     $$\mathbf{\text{ia} = \text{mul\_result\_full}[31:0]}$$
5. Line 35 takes `mul_result_full[51:20]`:
   - It shifts right by 20 bits ($1,048,576\times$ division).
   - This treats `mul_result_full` as having 40 fractional bits (Q24.40, as if both inputs had been Q12.20).
   - For rated current $5.0\text{ A}$ ($5,242,880$ in Q12.20):
     $$\text{mul\_result\_full}[51:20] = 5,242,880 \gg 20 = \mathbf{5}$$
   - Outputting 5 in Q12.20 corresponds to:
     $$\frac{5}{2^{20}} = 0.00000477\text{ A} = \mathbf{4.77\,\mu\text{A}}$$

### 2.2 Proof from Vivado XSim (`scratch/tb_clarke_scaling.v`)
```text
TEST CASE: Nominal Positive Current (+5.0A on Phase A)
Inputs: ia_raw = 2560 (diff = +512), ib_raw = 2048 (diff = 0)
Internal mul_result_full for ia : 64'sd5242880 (hex: 64'h0000000000500000)
  Slice [31:0]  (Proper Q12.20) : 32'sd5242880 -> 5.000000 A
  Slice [51:20] (DUT mul_result): 32'sd5 -> 0.000005 A (Attenuated by 2^20!)
DUT Outputs: i_alpha = 32'sd5 (0.000005 A) | i_beta = 32'sd2 (0.000002 A)
```
**Conclusion:** The previous verdict is **100% CONFIRMED**. Current feedback is wiped out by $1,048,576\times$.

---

## 3. Analysis of New Hazards & Edge Cases

### 3.1 Can `temp <= ia + (ib <<< 1)` Ever Overflow 32 Bits?
**Mathematical Proof:**
- `temp` is declared as `reg signed [31:0] temp;` (range: $-2,147,483,648$ to $+2,147,483,647$).
- Maximum possible ADC inputs occur at the physical rails: $i_{raw} \in [0, 4095]$.
- `ADC_SCALE` for ACS712-40AB with 3.3V reference is $16896$ ($0.016113\text{ A/LSB} \times 2^{20}$).
- At positive rail ($i_{raw} = 4095$):
  $$\Delta i = 4095 - 2048 = +2047 \implies ia = 2047 \times 16896 = +34,586,112$$
- At negative rail ($i_{raw} = 0$):
  $$\Delta i = 0 - 2048 = -2048 \implies ia = -2048 \times 16896 = -34,603,008$$
- Maximum worst-case value for `temp`:
  $$\text{temp}_{\max} = ia_{\max} + 2 \cdot ib_{\max} = 3 \times 34,586,112 = \mathbf{+103,758,336}$$
- Minimum worst-case value for `temp`:
  $$\text{temp}_{\min} = ia_{\min} + 2 \cdot ib_{\min} = 3 \times (-34,603,008) = \mathbf{-103,809,024}$$
- Comparing to 32-bit limits:
  $$\frac{103,809,024}{2,147,483,647} = \mathbf{4.83\%}$$
**Verdict:** **IMPOSSIBLE TO OVERFLOW.** `temp` operates with a safety margin of $20.7\times$.

### 3.2 NEW CRITICAL DEFECT: Bit-11 Blindness & Overcurrent Disable
A critical cross-module failure was discovered between `adc_pmod_ad1.v` and `control_fsm.v`:
1. Because the 1-bit right shift inserts a 5th leading zero into bit 11:
   $$\text{data\_ch0}[11] \equiv 0 \implies \mathbf{\text{data\_ch0} \le 2047 \ (\text{12'h7FF})}$$
2. In `control_fsm.v`:
   ```verilog
   if (adc_data_ch0 > `MAX_CURRENT_RAW || adc_data_ch0 < `MIN_CURRENT_RAW || ...) begin
       state <= S_ERROR;
   ```
   `MAX_CURRENT_RAW` is defined as `3800`.
3. Since `adc_data_ch0` can NEVER exceed 2047, **the upper overcurrent threshold can never be reached under any physical condition!** The inverter has zero overcurrent protection on positive currents.
4. Furthermore, because `data_ch0 <= 2047`:
   $$\text{ia\_signed} = \text{data\_ch0} - 2048 \le -1$$
   The MPC controller believes that **current is permanently negative on both channels**, injecting wild, saturating switching commands.

### 3.3 NEW HARDWARE HAZARD: ACS712-40AB to 3.3V ADC Asymmetry
- ACS712-40AB supply is $5.0\text{ V}$. Quiescent output ($0\text{ A}$) is $2.510\text{ V}$. Sensitivity is $50\text{ mV/A}$.
- The Pmod AD1 reference is $3.3\text{ V}$.
- At positive currents, the sensor output reaches $3.3\text{ V}$ at:
  $$I_{\max} = \frac{3.300\text{ V} - 2.510\text{ V}}{0.050\text{ V/A}} = \mathbf{+15.8\text{ A}}$$
- At negative currents, the sensor reaches $0\text{ V}$ at:
  $$I_{\min} = \frac{0.000\text{ V} - 2.510\text{ V}}{0.050\text{ V/A}} = \mathbf{-50.2\text{ A}}$$
- This results in a severely unbalanced dynamic range:
  $$\mathbf{-40.0\text{ A} \le I \le +15.8\text{ A}}$$
- Any physical motor current $> +15.8\text{ A}$ will drive the Pmod AD1 analog input above $3.3\text{ V}$, forward-biasing the AD7476A ESD protection diodes and injecting current into the FPGA 3.3V rail unless a resistive divider or clamping diode is present on the PCB.

### 3.4 FPGA SCLK Glitch & Output Buffer Timing
- `adc_sclk` is driven directly by a registered flip-flop clocked by `clk` (100 MHz):
  ```verilog
  if (clk_div == 3'd3) adc_sclk <= 1'b1;
  else if (clk_div == 3'd7) adc_sclk <= 1'b0;
  ```
- Because it is registered, there are NO combinatorial glitches.
- The duty cycle is exactly 50% (4 cycles LOW, 4 cycles HIGH).
- Transition from `IDLE` to `RUN` provides 40 ns of CS-to-SCLK setup time, exceeding the 10 ns datasheet minimum ($t_3$).

---

## 4. Remediation Specifications

### Fix 1: Phase-Compensation in `adc_pmod_ad1.v`
To eliminate the 1-bit shift without altering external pin connections, sample the serial line at count 7 (just before SCLK falls) rather than count 3:
```verilog
// In adc_pmod_ad1.v:
wire sample_edge = (clk_div == 3'd7) && sclk_en; // 70ns after SCLK fall; allows full 60ns sync+access margin!
RUN: begin
    if (sample_edge) begin
        shift_ch0 <= {shift_ch0[14:0], d0_in};
        shift_ch1 <= {shift_ch1[14:0], d1_in};
    end
    if (sclk_fall) begin
        bit_cnt <= bit_cnt + 5'd1;
        if (bit_cnt == 5'd15) begin ...
```

### Fix 2: Slicing Correction in `clarke_transform.v`
```verilog
// In clarke_transform.v:
// For integer * Q12.20 -> result is Q44.20. Select [31:0] for Q12.20 output:
ia <= mul_result_full[DATA_WIDTH-1:0];
...
ib <= mul_result_full[DATA_WIDTH-1:0];
// For temp (Q12.20) * INV_SQRT3 (Q12.20) -> result is Q24.40. Select [51:20]:
i_beta <= mul_result; // mul_result_full[51:20] is correct for i_beta!
```

### Fix 3: Scale & Tare Calibration in `mpc_params.vh`
Update `ADC_SCALE` for ACS712-40AB with 3.3V reference:
$$\text{ADC\_SCALE} = \frac{33}{2048} \times 2^{20} = \mathbf{16896} \quad (\text{replacing } 10240)$$
Tare calibration register will correctly average quiescent counts ($\approx 3115$) at startup.
