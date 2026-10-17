## =============================================================================
## File: nexys_video_oled.xdc
## Description: Physical Constraints for Nexys Video OLED Standalone Test Project
## Target Board: Digilent Nexys Video (AMD/Xilinx Artix-7 XC7A200T-1SBG484C)
## =============================================================================

## System Clock (100 MHz primary oscillator on Bank 14)
set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports { sys_clk_i }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { sys_clk_i }]

## CPU Reset Button (Red pushbutton "CPU RESET" on Bank 15, Active-Low)
set_property -dict { PACKAGE_PIN G4    IOSTANDARD LVCMOS15 } [get_ports { cpu_resetn }]

## Onboard OLED Display (Univision UG-2832HSWEG04 / SSD1306 on Bank 14)
set_property -dict { PACKAGE_PIN W21   IOSTANDARD LVCMOS33 } [get_ports { oled_sclk }]
set_property -dict { PACKAGE_PIN Y22   IOSTANDARD LVCMOS33 } [get_ports { oled_sdin }]
set_property -dict { PACKAGE_PIN W22   IOSTANDARD LVCMOS33 } [get_ports { oled_dc   }]
set_property -dict { PACKAGE_PIN U21   IOSTANDARD LVCMOS33 } [get_ports { oled_res  }]
set_property -dict { PACKAGE_PIN P20   IOSTANDARD LVCMOS33 } [get_ports { oled_vbat }]
set_property -dict { PACKAGE_PIN V22   IOSTANDARD LVCMOS33 } [get_ports { oled_vdd  }]

## Slew Rate and Drive Strength for Fast SPI Signals
set_property SLEW FAST [get_ports { oled_sclk oled_sdin }]
set_property DRIVE 8   [get_ports { oled_sclk oled_sdin oled_dc oled_res oled_vbat oled_vdd }]

## Onboard User LEDs (Bank 13, 2.5V logic)
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
