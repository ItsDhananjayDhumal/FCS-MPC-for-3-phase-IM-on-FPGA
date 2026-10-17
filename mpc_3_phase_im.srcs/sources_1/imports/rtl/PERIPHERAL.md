# PERIPHERAL MAPPINGS AND PINS

- `MASTER RESET (G4):` Active Low, Master reset for entire design
- `BTNU (F15):` Push Button, Start/Run (debounced)
- `BTND (D22):` Push Button, Stop/Fault Clear (debounced, to reset after faults, press `BTND` then `BTNU`)
- `SW[7:4] (M17, K13, J16, H17):` Slide Switches, Coarse DC-Bus Voltage Calib (40v/inc, 0 to 600v)
- `SW[3:0] (G22, G21, F21, E22):` Slide Switches, Fine DC-Bus Voltage Calib (3v/inc, 0 to 45v)
- `LED[7:4] (Y13, W15, W16, V15):` LEDs, DC-Bus Voltage Indicator (directly connected to `SW[7:4]`)
- `LED[3] (U16):` LED, Fault Indicator
- `LED[2:0] (T16, T15, T14):` LEDs, Optimal Inverter Vector State (in order a, b, c)

---

<!-- - `PMOD JA:`
* > `JA1 (V9) / JA2 (V8):` Phase A High (gate_ah) / Low (gate_al)
* > `JA3 (V7) / JA4 (W7):` Phase B High (gate_bh) / Low (gate_bl)
* > `JA7 (W9) / JA8 (Y9):` Phase C High (gate_ch) / Low (gate_cl)
* > `JA9 (Y8)`: Inverter Hardware Enable (inverter_en) -->


<!-- - `PMOD JA:`
* > `JA1 (V9):` Phase A High (gate_ah)
* > `JA2 (V8):` Phase A Low (gate_al)
* > `JA3 (V7):` Phase B High (gate_bh)
* > `JA4 (W7):` Phase B Low (gate_bl)
* > `JA7 (W9)` Phase C High (gate_ch)
* > `JA8 (Y9):` Phase C Low (gate_cl)
* > `JA9 (Y8)`: Inverter Hardware Enable (inverter_en) -->

- `PMOD JA:`
* > `JA1 (AB22):` adc_cs_n
* > `JA2 (AB21):` adc_d0
* > `JA3 (AB20):` adc_d1
* > `JA4 (AB18):` adc_sclk
* > `JA7 (Y21)` enc_a
* > `JA8 (AA21):` enc_b
* > `JA9 (AA20)`: enc_z

- `PMOD JB:`
* > `JB1 (V9):` Phase A High (gate_ah)
* > `JB2 (V8):` Phase A Low (gate_al)
* > `JB3 (V7):` Phase B High (gate_bh)
* > `JB4 (W7):` Phase B Low (gate_bl)
* > `JB7 (W9)` Phase C High (gate_ch)
* > `JB8 (Y9):` Phase C Low (gate_cl)
* > `JB9 (Y8)`: Inverter Hardware Enable (inverter_en)