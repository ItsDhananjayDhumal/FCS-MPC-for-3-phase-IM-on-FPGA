# Python exact behavioral simulation of adc_pmod_ad1.v and AD7476A ADC model

def simulate(val_ch0=0xA5C, val_ch1=0x3B7):
    # ADC internal state
    # 16-bit word: 4 leading zeros + 12-bit data
    # bits: [15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0]
    # bit 15: leading zero 0
    # bit 14: leading zero 1
    # bit 13: leading zero 2
    # bit 12: leading zero 3
    # bit 11..0: 12-bit conversion value
    adc_word0 = (val_ch0 & 0x0FFF) # 4 leading zeros + 12 bits
    adc_word1 = (val_ch1 & 0x0FFF)
    
    # State of ADC hardware
    adc_d0 = 0
    adc_d1 = 0
    adc_shift0 = adc_word0
    adc_shift1 = adc_word1
    adc_bit_cnt = 0
    prev_adc_cs_n = 1
    prev_adc_sclk = 0

    # FPGA internal registers (at posedge clk)
    d0_sync = 0 # 2-bit
    d1_sync = 0 # 2-bit
    clk_div = 0 # 3-bit
    adc_sclk = 0
    sclk_en = 0
    state = 0 # 0: IDLE, 1: RUN, 2: QUIET
    shift_ch0 = 0 # 16-bit
    shift_ch1 = 0 # 16-bit
    bit_cnt = 0 # 5-bit
    quiet_cnt = 0 # 3-bit
    adc_cs_n = 1
    data_ch0 = 0
    data_ch1 = 0
    done = 0

    # Trace log
    log = []

    # Run for 200 cycles
    start = 1 # pulse start for 1 cycle at cycle 2
    
    for cycle in range(160):
        # Current wire values before clock edge
        d0_in = (d0_sync >> 1) & 1
        d1_in = (d1_sync >> 1) & 1
        sclk_rise = 1 if ((clk_div == 3) and (sclk_en == 1)) else 0
        sclk_fall = 1 if ((clk_div == 7) and (sclk_en == 1)) else 0
        
        start_sig = 1 if cycle == 1 else 0

        # Log before edge
        log.append({
            'cycle': cycle,
            'state': state,
            'cs_n': adc_cs_n,
            'sclk': adc_sclk,
            'clk_div': clk_div,
            'sclk_rise': sclk_rise,
            'sclk_fall': sclk_fall,
            'bit_cnt': bit_cnt,
            'quiet_cnt': quiet_cnt,
            'd0_in': d0_in,
            'adc_d0': adc_d0,
            'shift_ch0': shift_ch0,
            'done': done,
            'data_ch0': data_ch0
        })

        # --- ADC Hardware Model behavior on wire changes ---
        # When CS falls
        # In hardware, when adc_cs_n transitions 1->0:
        # bit 15 is driven immediately onto adc_d0
        # When adc_sclk transitions 1->0:
        # next bit is driven onto adc_d0

        # Next cycle register values (non-blocking assignments in Verilog)
        # 1. d0_sync, d1_sync
        next_d0_sync = ((d0_sync << 1) & 2) | (adc_d0 & 1)
        next_d1_sync = ((d1_sync << 1) & 2) | (adc_d1 & 1)

        # 2. clk_div, adc_sclk
        next_clk_div = clk_div
        next_adc_sclk = adc_sclk
        if sclk_en:
            next_clk_div = (clk_div + 1) & 7
            if clk_div == 3:
                next_adc_sclk = 1
            elif clk_div == 7:
                next_adc_sclk = 0
        else:
            next_clk_div = 0
            next_adc_sclk = 0

        # 3. FSM
        next_state = state
        next_adc_cs_n = adc_cs_n
        next_sclk_en = sclk_en
        next_bit_cnt = bit_cnt
        next_quiet_cnt = quiet_cnt
        next_shift_ch0 = shift_ch0
        next_shift_ch1 = shift_ch1
        next_data_ch0 = data_ch0
        next_data_ch1 = data_ch1
        next_done = 0

        if state == 0: # IDLE
            if start_sig:
                next_state = 1 # RUN
                next_adc_cs_n = 0
                next_sclk_en = 1
                next_bit_cnt = 0
        elif state == 1: # RUN
            if sclk_rise:
                next_shift_ch0 = ((shift_ch0 << 1) & 0xFFFF) | d0_in
                next_shift_ch1 = ((shift_ch1 << 1) & 0xFFFF) | d1_in
            elif sclk_fall:
                next_bit_cnt = (bit_cnt + 1) & 0x1F
                if bit_cnt == 15:
                    next_state = 2 # QUIET
                    next_adc_cs_n = 1
                    next_sclk_en = 0
                    next_data_ch0 = shift_ch0 & 0x0FFF
                    next_data_ch1 = shift_ch1 & 0x0FFF
                    next_done = 1
                    next_quiet_cnt = 0
        elif state == 2: # QUIET
            if quiet_cnt == 4:
                next_state = 0 # IDLE
            else:
                next_quiet_cnt = (quiet_cnt + 1) & 7
        else:
            next_state = 0

        # Update FPGA registers
        d0_sync = next_d0_sync
        d1_sync = next_d1_sync
        clk_div = next_clk_div
        adc_sclk = next_adc_sclk
        state = next_state
        adc_cs_n = next_adc_cs_n
        sclk_en = next_sclk_en
        bit_cnt = next_bit_cnt
        quiet_cnt = next_quiet_cnt
        shift_ch0 = next_shift_ch0
        shift_ch1 = next_shift_ch1
        data_ch0 = next_data_ch0
        data_ch1 = next_data_ch1
        done = next_done

        # ADC reaction to new CS and SCLK
        # CS falling edge:
        if prev_adc_cs_n == 1 and adc_cs_n == 0:
            adc_shift0 = adc_word0
            adc_shift1 = adc_word1
            adc_bit_cnt = 0
            adc_d0 = (adc_shift0 >> 15) & 1
            adc_d1 = (adc_shift1 >> 15) & 1
        elif adc_cs_n == 0:
            # SCLK falling edge:
            if prev_adc_sclk == 1 and adc_sclk == 0:
                adc_bit_cnt += 1
                if adc_bit_cnt < 16:
                    adc_shift0 = (adc_shift0 << 1) & 0xFFFF
                    adc_shift1 = (adc_shift1 << 1) & 0xFFFF
                    adc_d0 = (adc_shift0 >> 15) & 1
                    adc_d1 = (adc_shift1 >> 15) & 1
        elif adc_cs_n == 1:
            adc_d0 = 0
            adc_d1 = 0

        prev_adc_cs_n = adc_cs_n
        prev_adc_sclk = adc_sclk

    return log, data_ch0, data_ch1, adc_word0, adc_word1

log, res0, res1, exp0, exp1 = simulate()
print(f"Expected Ch0: 0x{exp0:03X} ({exp0}) ({bin(exp0)})")
print(f"Result   Ch0: 0x{res0:03X} ({res0}) ({bin(res0)})")
print(f"Expected Ch1: 0x{exp1:03X} ({exp1}) ({bin(exp1)})")
print(f"Result   Ch1: 0x{res1:03X} ({res1}) ({bin(res1)})")

# Print sampling events
for entry in log:
    if entry['sclk_rise']:
        print(f"Cycle {entry['cycle']:3d}: sclk_rise! bit_cnt={entry['bit_cnt']:2d}, d0_in={entry['d0_in']}, adc_d0={entry['adc_d0']}, shift_ch0_before=0x{entry['shift_ch0']:04X}")
    if entry['sclk_fall']:
        print(f"Cycle {entry['cycle']:3d}: sclk_fall! bit_cnt={entry['bit_cnt']:2d}, done={entry['done']}, state={entry['state']}")
    if entry['done']:
        print(f"Cycle {entry['cycle']:3d}: DONE asserted! data_ch0=0x{entry['data_ch0']:03X}")
