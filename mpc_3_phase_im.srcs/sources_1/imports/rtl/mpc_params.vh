`ifndef MPC_PARAMS_VH
`define MPC_PARAMS_VH

`define DATA_WIDTH 32
`define FRAC_BITS  20

`define SYS_CLK_FREQ   100_000_000
`define SWITCHING_FREQ 10_000
`define TS_COUNTER_MAX (`SYS_CLK_FREQ / `SWITCHING_FREQ)

`define DEAD_TIME_NS     2000
`define DEAD_TIME_CYCLES (`SYS_CLK_FREQ / (1_000_000_000 / `DEAD_TIME_NS))

`define ADC_BITS     12
`define ADC_SCLK_DIV 8
`define ADC_NUM_BITS 16
// ADC Scale: physical Amperes per count in Q12.20
// For ACS712-40AB (50 mV/A) and Pmod AD1 (3.3V reference, 12-bit):
// 3.3V / (4096 * 0.050 V/A) = 0.01611328 A/count -> 0.01611328 * 2^20 = 16896
`define ADC_SCALE 32'sd16896

`define ENCODER_PPR 2500
`define ENCODER_CPR (`ENCODER_PPR * 4)
`define NUM_POLES   4
`define POLE_PAIRS (`NUM_POLES / 2)
`define ENCODER_Z_RESET 1'b0  // Disabled: Z reserved for diagnostics only

// Speed scale: (2*pi*p*Fs / CPR) in Q12.20
`define SPEED_SCALE ($signed((64'd13176795 * `POLE_PAIRS * `SWITCHING_FREQ * 64'd10000) / (64'd2 * 64'd10000 * `ENCODER_CPR)))

`define SPEED_ALPHA           32'sd104858
`define SPEED_ONE_MINUS_ALPHA 32'sd943718

// =============================================================================
// MOTOR PHYSICAL PARAMETERS (Default: 2.2 kW, 400V, 50 Hz, 4-Pole Motor)
// Modify these base values when changing to a different induction motor!
// =============================================================================
`define MOTOR_RS_MOHM      64'd1120       // Stator resistance: 1.12 Ohm (1120 mOhm)
`define MOTOR_RR_MOHM      64'd1080       // Rotor resistance: 1.08 Ohm (1080 mOhm)
`define MOTOR_LM_UH        64'd202950     // Mutual magnetizing inductance: 202.95 mH (202950 uH)
`define MOTOR_LS_UH        64'd208930     // Stator total inductance (Lm + Lls): 208.93 mH (208930 uH)
`define MOTOR_LR_UH        64'd208930     // Rotor total inductance (Lm + Llr): 208.93 mH (208930 uH)
`define MOTOR_FREQ_HZ      50             // Rated supply frequency (Hz)
`define MOTOR_VOLTAGE_V    64'd400        // Rated line-to-line RMS voltage (V)

// =============================================================================
// DISCRETE INDUCTION MOTOR MODEL CONSTANTS (Q12.20 FORMAT)
// Dynamically derived from physical motor parameters (Rs, Rr, Lm, Lr, Ls, Fs, poles)
// Default motor compiles to identical values: TS_Q=105, E22=1048034, E21=110,
// D1=9016, C13=8758, C12=45224, C11=1029293, KT=3055130
// =============================================================================
// 1. TS_Q: Ts in Q12.20 (Ts = 1 / SWITCHING_FREQ)
`define TS_Q ($signed((64'd105 * 64'd10000) / `SWITCHING_FREQ))

// 2. E22: Rotor flux retention factor = 1 - (Ts * Rr / Lr) in Q12.20
`define DELTA_E22 ((64'd542 * `MOTOR_RR_MOHM * 64'd208930 * 64'd10000) / (64'd1080 * `MOTOR_LR_UH * `SWITCHING_FREQ))
`define E22 ($signed(32'sd1048576 - `DELTA_E22))

// 3. E21: Stator-to-rotor magnetization gain = Ts * (Rr / Lr) * Lm in Q12.20
`define E21_SCALE ((64'd110 * `MOTOR_RR_MOHM) / 64'd1080)
`define E21 ($signed((`E21_SCALE * `MOTOR_LM_UH * 64'd208930 * 64'd10000) / (64'd202950 * `MOTOR_LR_UH * `SWITCHING_FREQ)))

// 4. Leakage Inductance sigma * Ls and D1: Ts / (sigma * Ls) in Q12.20
`define SIGMA_LS_UH ((`MOTOR_LS_UH * `MOTOR_LR_UH - `MOTOR_LM_UH * `MOTOR_LM_UH) / `MOTOR_LR_UH)
`define D1 ($signed((64'd9016 * 64'd11788 * 64'd10000) / (`SIGMA_LS_UH * `SWITCHING_FREQ)))

// 5. C13: Speed cross-coupling gain = D1 * (Lm / Lr) in Q12.20
`define C13 ($signed((64'd8758 * `D1 * `MOTOR_LM_UH * 64'd208930) / (64'd9016 * 64'd202950 * `MOTOR_LR_UH)))

// 6. C12: Rotor flux damping gain = C13 * (Rr / Lr) in Q12.20
`define C12 ($signed((64'd45224 * `C13 * `MOTOR_RR_MOHM * 64'd208930) / (64'd8758 * 64'd1080 * `MOTOR_LR_UH)))

// 7. C11: Stator current retention factor = 1 - (D1*Rs + C12*Lm) in Q12.20
`define TERM_RS ((64'd10098 * `D1 * `MOTOR_RS_MOHM) / (64'd9016 * 64'd1120))
`define TERM_RR ((64'd9185 * `C12 * `MOTOR_LM_UH) / (64'd45224 * 64'd202950))
`define C11 ($signed(32'sd1048576 - (`TERM_RS + `TERM_RR)))

// 8. KT: Electromagnetic torque constant = (3/2) * p * (Lm / Lr) in Q12.20
`define KT ($signed((64'd3055130 * `POLE_PAIRS * `MOTOR_LM_UH * 64'd208930) / (64'd2 * 64'd202950 * `MOTOR_LR_UH)))

// =============================================================================
// FORWARD EULER NUMERICAL STABILITY SPEED LIMIT
// Discrete Euler eigenvalue magnitude: |lambda|^2 = E22^2 + (Ts * omega_r)^2 < 1.0
// Stability limit: omega_crit = sqrt(2 * Rr / (Lr * Ts))
// Computes dynamically from MOTOR_RR_MOHM, MOTOR_LR_UH, and Ts (100 us) at compile time!
// =============================================================================
`define OMEGA_CRIT_SQ ((64'd2 * `MOTOR_RR_MOHM * 64'd1000000000) / (`MOTOR_LR_UH * 64'd100))
`define NR_S0 64'd300
`define NR_S1 ((`NR_S0 + `OMEGA_CRIT_SQ / `NR_S0) / 64'd2)
`define NR_S2 ((`NR_S1 + `OMEGA_CRIT_SQ / `NR_S1) / 64'd2)
`define NR_S3 ((`NR_S2 + `OMEGA_CRIT_SQ / `NR_S2) / 64'd2)
`define NR_S4 ((`NR_S3 + `OMEGA_CRIT_SQ / `NR_S3) / 64'd2)

// Speed limit in rad/s (integer) and in Q12.20 signed format for flux_observer.v:
`define SPEED_LIMIT_RADS (`NR_S4)
`define SPEED_LIMIT      ($signed(32'sd0 + ((`NR_S4) <<< `FRAC_BITS)))

`define VDC_DEFAULT_INT 10'd311
`define VDC_COARSE_STEP 10'd40
`define VDC_FINE_STEP   10'd3

`define TWO_THIRDS 32'sd699051
`define ONE_THIRD  32'sd349525
`define INV_SQRT3  32'sd605510

`define LAMBDA_T 32'sd1048576
`define LAMBDA_PSI 32'sd104857600

// Reference Rotor Flux: nominal 0.96 Wb in Q12.20 (0.96 * 2^20 = 1006633)
// Dynamically derived from rated voltage, frequency, Lm, and Lr:
// Stage 1: V/f voltage-frequency ratio (nominal: 400V, 50 Hz -> 1006633)
`define PSI_V_F (((64'd1006633 * `MOTOR_VOLTAGE_V * 64'd50) / (64'd400 * `MOTOR_FREQ_HZ)))
// Stage 2: Lm/Lr inductance ratio scaling (nominal: Lm=202950, Lr=208930)
`define PSI_REF (((`PSI_V_F * `MOTOR_LM_UH * 64'd208930) / (64'd202950 * `MOTOR_LR_UH)))
// Stage 3: Squared flux magnitude in Q12.20 = (PSI_REF * PSI_REF) / 2^20
`define PSI_REF_SQ ($signed((`PSI_REF * `PSI_REF) / 64'd1048576))
`define TE_REF_DEFAULT 32'sd5242880
`define SPEED_REF_DEFAULT 32'sd104857600

`define PI_KP 32'sd524288       // K_P = 0.5 (Gentle, safe proportional response)
`define PI_KI 32'sd10486       // K_I = 0.01 (Slow, safe integration: reaches 20 Nm smoothly in ~200 ms)
`define TE_MAX 32'sd20971520    // T_MAX = 20 N.m
`define TE_MIN -32'sd20971520   // T_MIN = -20 N.m

// Overcurrent protection threshold: +/- 5.0 Amperes (in ADC counts)
// 5.0 A / (0.01611328 A/count) = 310 counts
`define MAX_CURRENT_DELTA 13'sd310
`define MAX_CURRENT_RAW 12'd3800
`define MIN_CURRENT_RAW 12'd200
`define NUM_VECTORS 8

// ADC auto-tare calibration parameters
`define AUTOTARE_WARMUP_CYCLES 500000000  // 5 seconds at 100 MHz
`define AUTOTARE_NUM_SAMPLES   256        // samples to average
`define AUTOTARE_OUTLIER_THRESH 12'd100   // reject > 100 LSBs from running mean

// Pre-magnetization startup parameters
`define MAG_VECTOR             3'b100     // V1 (Phase A high) for magnetization
`define MAG_CYCLES             100        // Ts periods for magnetization
// Pre-magnetization current limit: 25% of overcurrent threshold = 1.25 Amperes
// 310 counts * 0.25 = 78 counts
`define MAG_CURRENT_MAX_DELTA  13'sd78
`define MAG_CURRENT_MAX        12'd950

`endif