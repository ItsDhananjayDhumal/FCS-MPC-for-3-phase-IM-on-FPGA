set_property -dict { PACKAGE_PIN R4    IOSTANDARD LVCMOS33 } [get_ports { sys_clk_i }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { sys_clk_i }]

set_property -dict { PACKAGE_PIN G4    IOSTANDARD LVCMOS15 } [get_ports { cpu_resetn }]

set_property -dict { PACKAGE_PIN B22   IOSTANDARD LVCMOS12 } [get_ports { btnc }]
set_property -dict { PACKAGE_PIN F15   IOSTANDARD LVCMOS12 } [get_ports { btnu }]
set_property -dict { PACKAGE_PIN D22   IOSTANDARD LVCMOS12 } [get_ports { btnd }]


set_property -dict { PACKAGE_PIN AB22  IOSTANDARD LVCMOS33 } [get_ports { adc_cs_n }]
set_property -dict { PACKAGE_PIN AB21  IOSTANDARD LVCMOS33 } [get_ports { adc_d0   }]
set_property -dict { PACKAGE_PIN AB20  IOSTANDARD LVCMOS33 } [get_ports { adc_d1   }]
set_property -dict { PACKAGE_PIN AB18  IOSTANDARD LVCMOS33 } [get_ports { adc_sclk }]

set_property -dict { PACKAGE_PIN Y21   IOSTANDARD LVCMOS33 } [get_ports { enc_a }]
set_property -dict { PACKAGE_PIN AA21  IOSTANDARD LVCMOS33 } [get_ports { enc_b }]
set_property -dict { PACKAGE_PIN AA20  IOSTANDARD LVCMOS33 } [get_ports { enc_z }]

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



set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

set_property SLEW FAST [get_ports { gate_ah gate_al gate_bh gate_bl gate_ch gate_cl }]
set_property DRIVE 12 [get_ports { gate_ah gate_al gate_bh gate_bl gate_ch gate_cl }]

set_output_delay -clock [get_clocks sys_clk_pin] 5.000 [get_ports {adc_sclk adc_cs_n}]
set_input_delay -clock [get_clocks sys_clk_pin] 5.000 [get_ports {adc_d0 adc_d1 enc_a enc_b enc_z}]
