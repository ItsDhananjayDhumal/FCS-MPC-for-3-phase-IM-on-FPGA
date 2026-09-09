# FCS-MPC for 3-Phase Induction Motor on FPGA

This project is a basic implementation of FSC MPC for a 3 phase induction motor. Given below are the impelmentation details.

---

## 1. Continuous-Time Motor Model

The controller operates in the stationary orthogonal reference frame ($\alpha-\beta$ frame). The dynamic equations of a squirrel-cage induction motor in the continuous-time domain are expressed using stator currents ($i_s$) and rotor fluxes ($\psi_r$) as state variables.

The state-space representation is:
$$ \frac{d}{dt} \mathbf{x} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} $$

Where the state vector is $\mathbf{x} = [i_{s\alpha}, i_{s\beta}, \psi_{r\alpha}, \psi_{r\beta}]^T$ and the input vector is $\mathbf{u} = [v_{s\alpha}, v_{s\beta}]^T$.

The continuous-time coefficients are derived from fundamental motor parameters:
*   $R_s, R_r$: Stator and rotor resistances
*   $L_s, L_r$: Stator and rotor inductances
*   $L_m$: Mutual inductance
*   $\sigma = 1 - \frac{L_m^2}{L_s L_r}$: Total leakage factor
*   $\tau_r = \frac{L_r}{R_r}$: Rotor time constant

The differential equations governing the stator currents are:
$$ \frac{di_{s\alpha}}{dt} = a_{11}i_{s\alpha} + a_{12}\psi_{r\alpha} + a_{13}\omega_r\psi_{r\beta} + b_1v_{s\alpha} $$
$$ \frac{di_{s\beta}}{dt} = a_{11}i_{s\beta} + a_{12}\psi_{r\beta} - a_{13}\omega_r\psi_{r\alpha} + b_1v_{s\beta} $$

The differential equations governing the rotor fluxes are:
$$ \frac{d\psi_{r\alpha}}{dt} = a_{21}i_{s\alpha} + a_{22}\psi_{r\alpha} - \omega_r\psi_{r\beta} $$
$$ \frac{d\psi_{r\beta}}{dt} = a_{21}i_{s\beta} + a_{22}\psi_{r\beta} + \omega_r\psi_{r\alpha} $$

Where the continuous-time constants are:
*   $a_{11} = -(\frac{R_s}{\sigma L_s} + \frac{R_r L_m^2}{\sigma L_s L_r^2})$
*   $a_{12} = \frac{L_m}{\sigma L_s L_r \tau_r}$
*   $a_{13} = \frac{L_m}{\sigma L_s L_r}$
*   $b_1 = \frac{1}{\sigma L_s}$
*   $a_{21} = \frac{L_m R_r}{L_r}$
*   $a_{22} = -\frac{R_r}{L_r} = -\frac{1}{\tau_r}$

---

## 2. Discretization and Predictive Equations

Because the FPGA evaluates control laws at discrete intervals ($T_s = 100 \mu s$), the continuous-time model must be discretized. This implementation uses the **Euler Forward (First-Order Rectangular)** approximation, substituting derivatives with forward differences:
$$ \frac{dx}{dt} \approx \frac{x(k+1) - x(k)}{T_s} $$

Applying this to the current and flux equations yields the next-state predictive equations implemented in `motor_predictor.v`.

### Stator Current Prediction
$$ i_{s\alpha}(k+1) = C_{11}i_{s\alpha}(k) + C_{12}\psi_{r\alpha}(k) + C_{13}\omega_r(k)\psi_{r\beta}(k) + D_1v_{s\alpha}(k) $$
$$ i_{s\beta}(k+1) = C_{11}i_{s\beta}(k) + C_{12}\psi_{r\beta}(k) - C_{13}\omega_r(k)\psi_{r\alpha}(k) + D_1v_{s\beta}(k) $$

### Rotor Flux Prediction
$$ \psi_{r\alpha}(k+1) = E_{21}i_{s\alpha}(k) + E_{22}\psi_{r\alpha}(k) - T_s\omega_r(k)\psi_{r\beta}(k) $$
$$ \psi_{r\beta}(k+1) = E_{21}i_{s\beta}(k) + E_{22}\psi_{r\beta}(k) + T_s\omega_r(k)\psi_{r\alpha}(k) $$

### Hardware Coefficients
To execute these equations efficiently, the continuous variables are pre-multiplied by $T_s$ and converted into discrete constants in `mpc_params.vh`:
*   $C_{11} = 1 + T_s a_{11}$
*   $C_{12} = T_s a_{12}$
*   $C_{13} = T_s a_{13}$
*   $D_1 = T_s b_1$
*   $E_{21} = T_s a_{21}$
*   $E_{22} = 1 + T_s a_{22}$
*   $T_{sQ} = T_s$ (Used for the $\omega_r$ cross-coupling terms in the flux equations).

---

## 3. Flux Observer

Because rotor flux cannot be measured directly, it is estimated using a discrete-time current model observer (`flux_observer.v`). The observer runs once per sampling period using the *measured* currents and *measured* speed to estimate the current rotor flux $\psi_r(k)$, which serves as the initial state for the predictive model.

The observer equations are structurally identical to the rotor flux predictive equations:
$$ \hat{\psi}_{r\alpha}(k) = E_{21}i_{s\alpha}(k-1) + E_{22}\hat{\psi}_{r\alpha}(k-1) - T_s\omega_r(k)\hat{\psi}_{r\beta}(k-1) $$
$$ \hat{\psi}_{r\beta}(k) = E_{21}i_{s\beta}(k-1) + E_{22}\hat{\psi}_{r\beta}(k-1) + T_s\omega_r(k)\hat{\psi}_{r\alpha}(k-1) $$

---

## 4. Cost Function Formulation

The FCS-MPC algorithm evaluates the state predictions for all 8 possible voltage vectors of the 2-level Voltage Source Inverter (VSI): $V_0 \dots V_7$.

For each vector, the `cost_evaluator.v` computes a scalar cost $J$. The objective of the controller is to track a torque reference ($T_e^*$) and a flux magnitude reference ($|\psi_r^*|^2$).

$$ J = \lambda_T (T_e^* - T_e^p)^2 + \lambda_\psi (|\psi_r^*|^2 - |\psi_r^p|^2)^2 $$

### Torque Calculation
Electromagnetic torque is calculated using the cross-product of estimated flux and predicted current:
$$ T_e^p = K_T (\psi_{r\alpha}^p i_{s\beta}^p - \psi_{r\beta}^p i_{s\alpha}^p) $$
Where $K_T = \frac{3}{2} \frac{P}{2} \frac{L_m}{L_r}$ ($P$ is the number of poles).

### Flux Magnitude Squared Calculation
To avoid computationally expensive square roots on the FPGA, the cost function regulates the *squared* flux magnitude:
$$ |\psi_r^p|^2 = (\psi_{r\alpha}^p)^2 + (\psi_{r\beta}^p)^2 $$

### Weighting Factors
*   $\lambda_T$: Torque weighting factor (Nominally 1.0).
*   $\lambda_\psi$: Flux weighting factor. Because flux magnitude ($\sim 1.0$ Wb) is numerically much smaller than torque ($\sim 5.0$ Nm), $\lambda_\psi$ is typically set high (e.g., 100.0) to normalize the penalization scales.

---

## 5. Analytical Errors and System Approximations

### 1. Euler Forward Discretization Instability
The Euler Forward approximation maps the left-half of the continuous-time s-plane into a circle of radius 1 centered at $(1, 0)$ in the discrete-time z-plane. For highly dynamic electrical systems or at large sampling times $T_s$, roots can migrate outside the unit circle, causing numerical instability. While $T_s = 100 \mu s$ is generally safe for 50/60 Hz induction motors, at very high electrical speeds, phase distortion and magnitude errors will artificially increase torque ripple. Higher-order discretizations (e.g., Tustin/Bilinear or Exact ZOH) can be implemented in future.

### 2. Lack of 1-Step Delay Compensation
Standard DSP and FPGA controllers exhibit a 1-step calculation delay. In a highly idealized MPC:
1. Sample at $k$.
2. Compute predictions and optimal vector.
3. Apply optimal vector exactly at $k$.
However, the computation takes finite time, meaning the vector for $k$ is actually applied near $k+1$. To analytically correct this, advanced MPC schemes use a two-step prediction horizon: they use $V_{applied}(k)$ to predict the state at $k+1$, and then run the 8-vector optimization to predict states at $k+2$. **This implementation utilizes a 1-step prediction horizon without delay compensation.** The high sampling rate (10 kHz) mitigates the delay's physical impact, but it will induce a phase lag in the control response. Since the current time utilization is around 17%, there is scope for higher horizon.

### 3. Fixed-Point Quantization and Truncation Errors
The FPGA utilizes a Q12.20 fixed-point arithmetic format.
*   **Resolution:** $2^{-20} \approx 0.954 \times 10^{-6}$.
*   **Double-Multiply Truncation:** Cross-coupling terms like $T_s \cdot \omega_r \cdot \psi_{r\beta}$ are computed in `flux_observer.v` by sequentially multiplying $\omega_r \cdot \psi_{r\beta}$, truncating the lower 20 bits, and then multiplying by $T_s$. Because $T_s$ is very small ($10^{-4}$), truncation of the intermediate result destroys low-magnitude precision. At very low speeds ($\omega_r \approx 0$), these terms will quantize to absolute zero.

### 4. Pure Integrator Drift in Observer
The `flux_observer.v` is an open-loop discrete integration of the motor voltage model. At very low speeds, the back-EMF is small, and any DC offset in the current measurements (ADC offset drift) will be integrated endlessly, causing the flux estimate to drift and saturate. Production drives usually cross over to a current-model observer at low speeds or introduce a low-pass filter (lossy integration) to pull the DC offset to zero.

### 5. Parameter Sensitivity
The coefficients $C_{11}, C_{12}, \dots$ are hardcoded in `mpc_params.vh`. In a real motor, $R_s$ and $R_r$ can increase by up to 50% due to thermal heating, and $L_m$ changes due to magnetic saturation. FCS-MPC is notoriously sensitive to parameter mismatches. Because this project lacks an online parameter estimator (like an Extended Kalman Filter or MRAS), torque tracking will exhibit steady-state errors as the motor heats up. A better flux observer and online estimator may be required based on the current performance of the drive.

### 6. Saturation and Overflow 
In `cost_evaluator.v`, the final cost sum $J$ can mathematically overflow the 32-bit signed integer space during extreme transients. To combat this, a clamping logic limits the cost to $2^{31}-1$. While this prevents arithmetic rollover (which would make a massive error look artificially small), it creates "cost plateaus" where multiple wildly wrong vectors might all hit the saturation ceiling and appear equally "bad" to the optimal selector.
