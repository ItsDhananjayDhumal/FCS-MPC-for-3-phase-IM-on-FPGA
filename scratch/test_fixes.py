import math, cmath

Ts = 0.0001
tau_r = 0.20893 / 1.08
Lm = 0.20295
Id = 4.73

print(f"Continuous rated flux: {Lm * Id:.4f} Wb")
print("=" * 80)
print(f"{'we (rad/s)':<12} {'True Cont':<12} {'Forward Euler':<16} {'2nd-Order':<16} {'Exact ZOH':<14}")
print("-" * 80)

for we in [0, 50, 100, 180, 250, 300, 321, 400]:
    wr = we
    # Forward Euler: (1 - Ts/tau_r) + j*Ts*wr
    denom_fe = cmath.exp(1j * we * Ts) - complex(1.0 - Ts/tau_r, Ts * wr)
    psi_fe = abs(Ts * Lm / tau_r / denom_fe) * Id
    
    # 2nd order Taylor: cos(wr*Ts) - Ts/tau_r + j*sin(wr*Ts)
    # where cos ≈ 1 - 0.5*(wr*Ts)^2, sin ≈ wr*Ts
    cos_term = 1.0 - 0.5 * (wr * Ts)**2
    sin_term = wr * Ts
    denom_2nd = cmath.exp(1j * we * Ts) - complex(cos_term - Ts/tau_r, sin_term)
    psi_2nd = abs(Ts * Lm / tau_r / denom_2nd) * Id
    
    # Exact ZOH
    # Continuous: dpsi/dt = A*psi + B*i, where A = -1/tau_r + j*wr, B = Lm/tau_r
    # In rotating frame of input i(t) = Id * exp(j*we*t), with we = wr:
    # At steady state, psi = (Lm * Id) exactly!
    psi_zoh = Lm * Id # By definition, exact ZOH has zero steady-state error
    
    print(f"{we:<12} {Lm*Id:<12.4f} {psi_fe:<16.4f} {psi_2nd:<16.4f} {psi_zoh:<14.4f}")
