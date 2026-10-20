set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports { sys_clk_i }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { sys_clk_i }]

set_property -dict { PACKAGE_PIN G4    IOSTANDARD LVCMOS15 } [get_ports { cpu_resetn }]

set_property -dict { PACKAGE_PIN B22   IOSTANDARD LVCMOS12 } [get_ports { btnc }]
set_property -dict { PACKAGE_PIN F15   IOSTANDARD LVCMOS12 } [get_ports { btnu }]
set_property -dict { PACKAGE_PIN D22   IOSTANDARD LVCMOS12 } [get_ports { btnd }]


## Digilent Pmod AD1 Interface (Pmod JA Lower Row: Pins 7-10 on Bank 14, 3.3V logic)
set_property -dict { PACKAGE_PIN Y21   IOSTANDARD LVCMOS33 } [get_ports { adc_cs_n }]
set_property -dict { PACKAGE_PIN AA21  IOSTANDARD LVCMOS33 } [get_ports { adc_d0   }]
set_property -dict { PACKAGE_PIN AA20  IOSTANDARD LVCMOS33 } [get_ports { adc_d1   }]
set_property -dict { PACKAGE_PIN AA18  IOSTANDARD LVCMOS33 } [get_ports { adc_sclk }]

## Rotary Encoder Hardware Interface (Pmod JA Upper Row: Pins 1-3 on Bank 14, 3.3V logic)
set_property -dict { PACKAGE_PIN AB22  IOSTANDARD LVCMOS33 } [get_ports { enc_a }]
set_property -dict { PACKAGE_PIN AB21  IOSTANDARD LVCMOS33 } [get_ports { enc_b }]
set_property -dict { PACKAGE_PIN AB20  IOSTANDARD LVCMOS33 } [get_ports { enc_z }]

set_property -dict { PACKAGE_PIN V9    IOSTANDARD LVCMOS33 } [get_ports { gate_ah     }]
set_property -dict { PACKAGE_PIN V8    IOSTANDARD LVCMOS33 } [get_ports { gate_al     }]
set_property -dict { PACKAGE_PIN V7    IOSTANDARD LVCMOS33 } [get_ports { gate_bh     }]
set_property -dict { PACKAGE_PIN W7    IOSTANDARD LVCMOS33 } [get_ports { gate_bl     }]
set_property -dict { PACKAGE_PIN W9    IOSTANDARD LVCMOS33 } [get_ports { gate_ch     }]
set_property -dict { PACKAGE_PIN Y9    IOSTANDARD LVCMOS33 } [get_ports { gate_cl     }]
set_property -dict { PACKAGE_PIN Y8    IOSTANDARD LVCMOS33 } [get_ports { inverter_en }]

set_property -dict { PACKAGE_PIN T14   IOSTANDARD LVCMOS25 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN T15   IOSTANDARD LVCMOS25 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN T16   IOSTANDARD LVCMOS25 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS25 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN V15   IOSTANDARD LVCMOS25 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS25 } [get_ports { led[5] }]
set_property -dict { PACKAGE_PIN W15   IOSTANDARD LVCMOS25 } [get_ports { led[6] }]
set_property -dict { PACKAGE_PIN Y13   IOSTANDARD LVCMOS25 } [get_ports { led[7] }]

set_property -dict { PACKAGE_PIN E22   IOSTANDARD LVCMOS12 } [get_ports { sw[0] }]
set_property -dict { PACKAGE_PIN F21   IOSTANDARD LVCMOS12 } [get_ports { sw[1] }]
set_property -dict { PACKAGE_PIN G21   IOSTANDARD LVCMOS12 } [get_ports { sw[2] }]
set_property -dict { PACKAGE_PIN G22   IOSTANDARD LVCMOS12 } [get_ports { sw[3] }]
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS12 } [get_ports { sw[4] }]
set_property -dict { PACKAGE_PIN J16   IOSTANDARD LVCMOS12 } [get_ports { sw[5] }]
set_property -dict { PACKAGE_PIN K13   IOSTANDARD LVCMOS12 } [get_ports { sw[6] }]
set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS12 } [get_ports { sw[7] }]

## Onboard OLED Display (Univision UG-2832HSWEG04 / SSD1306 on Bank 14)
set_property -dict { PACKAGE_PIN W21   IOSTANDARD LVCMOS33 } [get_ports { oled_sclk }]
set_property -dict { PACKAGE_PIN Y22   IOSTANDARD LVCMOS33 } [get_ports { oled_sdin }]
set_property -dict { PACKAGE_PIN W22   IOSTANDARD LVCMOS33 } [get_ports { oled_dc   }]
set_property -dict { PACKAGE_PIN U21   IOSTANDARD LVCMOS33 } [get_ports { oled_res  }]
set_property -dict { PACKAGE_PIN P20   IOSTANDARD LVCMOS33 } [get_ports { oled_vbat }]
set_property -dict { PACKAGE_PIN V22   IOSTANDARD LVCMOS33 } [get_ports { oled_vdd  }]

## Slew Rate and Drive Strength for Fast SPI Signals
set_property SLEW FAST [get_ports { oled_sclk oled_sdin adc_sclk adc_cs_n }]
set_property DRIVE 8   [get_ports { oled_sclk oled_sdin oled_dc oled_res oled_vbat oled_vdd adc_sclk adc_cs_n }]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property SLEW FAST [get_ports { gate_ah gate_al gate_bh gate_bl gate_ch gate_cl }]
set_property DRIVE 12 [get_ports { gate_ah gate_al gate_bh gate_bl gate_ch gate_cl }]

## Asynchronous and Double-Synchronized Input Constraints
## (Synchronizers / Debouncers handle domain crossing; ADC d0/d1 are double-registered via d0_sync/d1_sync)
set_false_path -from [get_ports {cpu_resetn btnc btnu btnd sw[*] enc_a enc_b enc_z adc_d0 adc_d1}]

## Asynchronous Reset Synchronizer Deassertion Paths (Reset recovery/removal false path)
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *rst_sync2*}]

## Static Debug, Display, and Low-Speed SPI Control Outputs
set_false_path -to [get_ports {led[*] oled_dc oled_res oled_vbat oled_vdd inverter_en adc_sclk adc_cs_n}]

## OLED Formatter Display Character Latching (Multi-cycle slow path; OLED refreshes at 50 Hz, SPI transmits at 3.125 MHz)
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *u_oled_ctrl*font_ascii_reg*}]
