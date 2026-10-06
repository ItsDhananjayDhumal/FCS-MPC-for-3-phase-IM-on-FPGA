import math, cmath

Ts = 0.0001
tau_r = 0.20893 / 1.08 # 0.19345 s
Lm = 0.20295
E21 = Ts * Lm / tau_r
E22 = 1.0 - Ts / tau_r
TS_Q = Ts
Id = 4.73

print(f"Continuous rated flux: Lm * Id = {Lm * Id:.4f} Wb")
print("=" * 65)
print(f"{'we (rad/s)':<12} {'True Cont Wb':<14} {'Discrete Euler Wb':<19} {'Ratio (Euler/True)':<18}")
print("-" * 65)

for we in [0, 50, 80, 100, 140, 180, 220, 260, 300, 321]:
    wr = we
    z = cmath.exp(1j * we * Ts)
    denom = z - complex(E22, TS_Q * wr)
    psi_discrete = abs(E21 / denom) * Id
    psi_continuous = Lm * Id
    ratio = psi_discrete / psi_continuous
    print(f"{we:<12} {psi_continuous:<14.4f} {psi_discrete:<19.4f} {ratio:<18.4f}")
