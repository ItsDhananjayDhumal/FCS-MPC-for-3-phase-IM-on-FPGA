# run_timing.tcl
set_param general.maxThreads 8
create_project -in_memory -part xc7a200tsbg484-1

read_verilog "ac_current_test/ac_current_top.v"
read_verilog "ac_current_test/adc_pmod_ad1.v"
read_verilog "ac_current_test/oled_controller.v"
read_verilog "ac_current_test/oled_font_rom.v"
read_verilog "ac_current_test/oled_spi_tx.v"

read_xdc "ac_current_test/nexys_video_ac.xdc"

synth_design -top ac_current_top -part xc7a200tsbg484-1
opt_design
place_design
route_design

report_timing_summary -max_paths 10 -file ac_current_test/ac_current_top_timing_summary_routed.rpt
report_utilization -file ac_current_test/ac_current_top_utilization_routed.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "========================================================="
puts "FINAL ROUTED WNS: $wns ns"
puts "========================================================="
if {$wns < 0.0} {
    puts "CRITICAL: TIMING VIOLATION DETECTED"
    exit 1
} else {
    puts "SUCCESS: TIMING CONSTRAINTS MET (WNS >= 0.000 ns)"
}
