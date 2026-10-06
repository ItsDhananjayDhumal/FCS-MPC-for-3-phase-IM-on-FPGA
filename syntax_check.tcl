cd {C:/Users/Dhananjay Dhumal/mpc_3_phase_im/mpc_3_phase_im.srcs/sources_1/imports/rtl}

set rtl_files [glob *.v]
puts "Found RTL files:"
foreach f $rtl_files {
    puts "  Reading: $f"
    read_verilog $f
}

puts "All files read successfully. Starting out-of-context synthesis for mpc_top..."
synth_design -top mpc_top -part xc7a35tcpg236-1 -mode out_of_context
puts "Synthesis run finished successfully!"
exit
