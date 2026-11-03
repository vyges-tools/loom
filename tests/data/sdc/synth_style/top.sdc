# A constraint file in the style a synthesis flow writes: setup and I/O files sourced, a proc,
# collection commands, singular queries, design rules on objects, senses and disabled arcs.
source setup.tcl
puts "applying constraints"
create_clock -name core_clk -period [expr $CLK_PERIOD*$PERIOD_SCALE] [get_port $CLK_PORT]
set_clock_uncertainty 0.05 [get_clock core_clk]
source io.sdc
set_max_transition 0.3 [get_ports dout*]
set_max_fanout 16 [current_design]
set_sense -type clock -stop_propagation [get_pin u_div/Q]
set_disable_timing -from A -to Y [get_cell u_mux]
set_disable_clock_gating_check [get_pins u_cg/E]
set_false_path -from [get_ports rst_n]
set_size_only [get_cells -hier *keep*] true
set_dont_touch [get_designs -hier prim_*]
foreach_in_collection p [all_inputs] { set_load 0.01 $p }
set_load 0.02 [get_ports dout*]
