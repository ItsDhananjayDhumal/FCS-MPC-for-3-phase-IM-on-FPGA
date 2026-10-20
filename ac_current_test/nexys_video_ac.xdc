## =============================================================================
## File: nexys_video_ac.xdc
## Description: Physical Pin & Timing Constraints for AC Peak-to-Peak Telemetry
## Target Board: Digilent Nexys Video (AMD/Xilinx Artix-7 XC7A200T-1SBG484C)
## Analog Front-End: Digilent Pmod AD1 (Pmod JB)
## Display: Univision UG-2832HSWEG04 SSD1306 128x32 OLED (Bank 14)
## =============================================================================

## System Clock (100 MHz primary oscillator on Bank 14)
set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports { sys_clk_i }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { sys_clk_i }]

## CPU Reset Button (Red pushbutton "CPU RESET" on Bank 15, Active-Low)
set_property -dict { PACKAGE_PIN G4    IOSTANDARD LVCMOS15 } [get_ports { cpu_resetn }]

## Center Pushbutton (BTNC on Bank 12, 1.2V logic) - Manual Auto-Tare Calibration Trigger
set_property -dict { PACKAGE_PIN B22   IOSTANDARD LVCMOS12 } [get_ports { btnc }]

## Slide Switches (Bank 12, 1.2V logic)
## SW0: 0 = Peak-to-Peak View, 1 = Detailed Peak-to-Peak + True RMS View
## SW1: 0 = 50 Hz Cycle Window (200 samples), 1 = Fast 20-Sample Window
set_property -dict { PACKAGE_PIN E22   IOSTANDARD LVCMOS12 } [get_ports { sw[0] }]
set_property -dict { PACKAGE_PIN F21   IOSTANDARD LVCMOS12 } [get_ports { sw[1] }]
set_property -dict { PACKAGE_PIN G21   IOSTANDARD LVCMOS12 } [get_ports { sw[2] }]
set_property -dict { PACKAGE_PIN G22   IOSTANDARD LVCMOS12 } [get_ports { sw[3] }]
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS12 } [get_ports { sw[4] }]
set_property -dict { PACKAGE_PIN J16   IOSTANDARD LVCMOS12 } [get_ports { sw[5] }]
set_property -dict { PACKAGE_PIN K13   IOSTANDARD LVCMOS12 } [get_ports { sw[6] }]
set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS12 } [get_ports { sw[7] }]

## Digilent Pmod AD1 Interface (Pmod JA Lower Row: Pins 7-10 on Bank 14, 3.3V logic)
set_property -dict { PACKAGE_PIN Y21   IOSTANDARD LVCMOS33 } [get_ports { adc_cs_n }]
set_property -dict { PACKAGE_PIN AA21  IOSTANDARD LVCMOS33 } [get_ports { adc_d0   }]
set_property -dict { PACKAGE_PIN AA20  IOSTANDARD LVCMOS33 } [get_ports { adc_d1   }]
set_property -dict { PACKAGE_PIN AA18  IOSTANDARD LVCMOS33 } [get_ports { adc_sclk }]

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

## Onboard User LEDs (Bank 13, 2.5V logic)
## LD0: 1 Hz System Heartbeat
## LD1: OLED Panel Initialized (Solid ON)
## LD2: Auto-Tare Calibration Complete
## LD3: AC Window Strobe
## LD4: Overcurrent Alert (>10.0A Peak-to-Peak)
## LD5: Outlier Sample Rejected Flag
## LD6: 20-Sample Averaging Buffer Full
## LD7: Fast Window Mode (SW1)
set_property -dict { PACKAGE_PIN T14   IOSTANDARD LVCMOS25 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN T15   IOSTANDARD LVCMOS25 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN T16   IOSTANDARD LVCMOS25 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS25 } [get_ports { led[3] }]
set_property -dict { PACKAGE_PIN V15   IOSTANDARD LVCMOS25 } [get_ports { led[4] }]
set_property -dict { PACKAGE_PIN W16   IOSTANDARD LVCMOS25 } [get_ports { led[5] }]
set_property -dict { PACKAGE_PIN W15   IOSTANDARD LVCMOS25 } [get_ports { led[6] }]
set_property -dict { PACKAGE_PIN Y13   IOSTANDARD LVCMOS25 } [get_ports { led[7] }]

## FPGA Configuration Properties
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## -----------------------------------------------------------------------------
## Timing Constraints & False Paths
## -----------------------------------------------------------------------------
## Asynchronous and Double-Synchronized Input Constraints
set_false_path -from [get_ports {cpu_resetn btnc sw[*] adc_d0 adc_d1}]

## Asynchronous Reset Synchronizer Deassertion Paths
set_false_path -from [get_cells -hierarchical -filter {NAME =~ *rst_sync2*}]

## Static Debug & Power Control Outputs
set_false_path -to [get_ports {led[*] oled_dc oled_res oled_vbat oled_vdd adc_cs_n adc_sclk}]

## OLED Font Character ASCII Latch (Multi-cycle path; updates at 40 Hz)
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *u_oled*font_ascii_reg*}]
