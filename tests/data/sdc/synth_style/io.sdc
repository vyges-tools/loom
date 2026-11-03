# I/O constraints, sourced from the top file.
proc io_delay {dir value ports {clock core_clk}} {
    if {$dir == "in"} {
        set_input_delay $value -clock $clock $ports
    } else {
        set_output_delay $value -clock $clock $ports
    }
}
append_to_collection IN_BUS [get_ports din*]
append_to_collection IN_BUS [get_port en]
io_delay in 0.4 $IN_BUS
io_delay out 0.5 [get_ports dout*]
