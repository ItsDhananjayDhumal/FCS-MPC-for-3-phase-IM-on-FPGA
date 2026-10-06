def test_delays(t4_ns=40, tco_ns=4, tin_ns=2):
    val_ch0 = 0xA5C
    # 16-bit stream:
    # 4 zeros + 12-bit data (1010 0101 1100)
    bits = [0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0]
    
    # Let's simulate in 1ns time steps from t = 0 to t = 1500 ns
    # Clock period = 10 ns.
    # Clock posedge at t = 5, 15, 25, 35, 45, ...
    
    # State variables
    # ADC
    adc_cs_n_pin = 1
    adc_sclk_pin = 0
    adc_d0_pin = 0
    adc_bit_idx = 0
    adc_sclk_falling_time = -9999
    adc_cs_falling_time = -9999
    
    # FPGA pins/registers
    clk_div = 0
    sclk_en = 0
    adc_cs_n_reg = 1
    adc_sclk_reg = 0
    d0_sync = [0, 0]
    shift_ch0 = 0
    bit_cnt = 0
    quiet_cnt = 0
    state = 0 # 0=IDLE, 1=RUN, 2=QUIET
    done = 0
    data_ch0 = 0
    
    # Events queue for pin delays
    # (time, target, value)
    events = []
    
    start_time = 15 # posedge clk at 15 ns
    
    samples_captured = []
    
    for t in range(0, 1600):
        # Process pin delay events arriving at time t
        remaining_events = []
        for et, target, val in events:
            if et == t:
                if target == 'cs_pin':
                    adc_cs_n_pin = val
                    if val == 0:
                        adc_cs_falling_time = t
                        # CS falling edge drives bit 0 after t3
                        # t3 <= 22ns, say 10ns
                        events.append((t + 10, 'd0_pin', bits[0]))
                        adc_bit_idx = 0
                elif target == 'sclk_pin':
                    prev = adc_sclk_pin
                    adc_sclk_pin = val
                    if prev == 1 and val == 0:
                        # Falling edge on SCLK pin!
                        adc_bit_idx += 1
                        if adc_bit_idx < 16:
                            events.append((t + t4_ns, 'd0_pin', bits[adc_bit_idx]))
                        else:
                            events.append((t + 20, 'd0_pin', 0))
                elif target == 'd0_pin':
                    adc_d0_pin = val
            else:
                remaining_events.append((et, target, val))
        events = remaining_events
        
        # FPGA Clock edge at posedge clk (every 10ns, say at t % 10 == 5)
        if t % 10 == 5:
            # Combinational signals before edge
            d0_in = d0_sync[1]
            sclk_rise = (clk_div == 3) and sclk_en
            sclk_fall = (clk_div == 7) and sclk_en
            
            # Next values
            next_d0_sync = [d0_sync[0], d0_sync[1]]
            # d0_sync[0] samples adc_d0_pin (delayed by tin_ns? let's assume pin directly or pin+tin)
            next_d0_sync[0] = adc_d0_pin
            next_d0_sync[1] = d0_sync[0]
            
            next_clk_div = clk_div
            next_adc_sclk_reg = adc_sclk_reg
            if sclk_en:
                next_clk_div = (clk_div + 1) & 7
                if clk_div == 3:
                    next_adc_sclk_reg = 1
                    events.append((t + tco_ns, 'sclk_pin', 1))
                elif clk_div == 7:
                    next_adc_sclk_reg = 0
                    events.append((t + tco_ns, 'sclk_pin', 0))
            else:
                next_clk_div = 0
                next_adc_sclk_reg = 0
                
            next_state = state
            next_adc_cs_n_reg = adc_cs_n_reg
            next_sclk_en = sclk_en
            next_bit_cnt = bit_cnt
            next_quiet_cnt = quiet_cnt
            next_shift_ch0 = shift_ch0
            next_data_ch0 = data_ch0
            next_done = 0
            
            start_sig = 1 if t == start_time else 0
            
            if state == 0: # IDLE
                if start_sig:
                    next_state = 1
                    next_adc_cs_n_reg = 0
                    events.append((t + tco_ns, 'cs_pin', 0))
                    next_sclk_en = 1
                    next_bit_cnt = 0
            elif state == 1: # RUN
                if sclk_rise:
                    next_shift_ch0 = ((shift_ch0 << 1) & 0xFFFF) | d0_in
                    samples_captured.append((t, bit_cnt, d0_in))
                elif sclk_fall:
                    next_bit_cnt = (bit_cnt + 1) & 0x1F
                    if bit_cnt == 15:
                        next_state = 2
                        next_adc_cs_n_reg = 1
                        events.append((t + tco_ns, 'cs_pin', 1))
                        next_sclk_en = 0
                        next_data_ch0 = shift_ch0 & 0x0FFF
                        next_done = 1
                        next_quiet_cnt = 0
            elif state == 2: # QUIET
                if quiet_cnt == 4:
                    next_state = 0
                else:
                    next_quiet_cnt = (quiet_cnt + 1) & 7
                    
            d0_sync = next_d0_sync
            clk_div = next_clk_div
            adc_sclk_reg = next_adc_sclk_reg
            state = next_state
            adc_cs_n_reg = next_adc_cs_n_reg
            sclk_en = next_sclk_en
            bit_cnt = next_bit_cnt
            quiet_cnt = next_quiet_cnt
            shift_ch0 = next_shift_ch0
            data_ch0 = next_data_ch0
            done = next_done

    return data_ch0, samples_captured

for t4 in [0, 5, 10, 15, 20, 25, 30, 35, 40]:
    val, smp = test_delays(t4_ns=t4)
    bits_cap = [s[2] for s in smp]
    print(f"t4 = {t4:2d} ns -> Latched 0x{val:03X} (exp 0xA5C) | Bits: {bits_cap}")
