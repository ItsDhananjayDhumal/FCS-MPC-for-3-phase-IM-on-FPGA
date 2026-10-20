# run_mpc_timing.tcl - In-memory timing closure verification for mpc_top
set_param general.maxThreads 8
create_project -in_memory -part xc7a200tsbg484-1

# Read all RTL sources
read_verilog [glob mpc_3_phase_im.srcs/sources_1/imports/rtl/*.v]

# Read XDC constraints
read_xdc mpc_3_phase_im.srcs/constrs_1/imports/constrs/nexys_video.xdc

# Run Synthesis and Implementation
synth_design -top mpc_top -part xc7a200tsbg484-1
opt_design
place_design
route_design

# Generate Timing and Utilization Reports
report_timing_summary -max_paths 10 -file mpc_top_timing_summary_routed.rpt
report_utilization -file mpc_top_utilization_routed.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "========================================================="
puts "MPC TOP FINAL ROUTED WNS: $wns ns"
puts "========================================================="
if {$wns < 0.0} {
    puts "CRITICAL: TIMING VIOLATION DETECTED"
    exit 1
} else {
    puts "SUCCESS: TIMING CONSTRAINTS MET (WNS >= 0.000 ns)"
}
