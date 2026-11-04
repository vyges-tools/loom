# In the shape hammer (ucb-bar, BSD-3) emits from its YAML clock and pin settings: a clock path
# given bare before -name, a generated clock whose empty invert option leaves a double space,
# groups braced with inner spaces, a default load on all outputs, and delays with -add_delay
# after the object, an empty min/max leaving a double space. Values are our own.
create_clock clock -name clock -period 2.0
set_clock_uncertainty 0.05 [get_clocks clock]
create_generated_clock -name clock_div -source clock -divide_by 2  [get_pins clkdiv/q]
create_generated_clock -name clock_inv -source clock -divide_by 4 -invert [get_pins clkinv/q]
create_clock jtag_tck -name jtag_tck -period 50.0
set_clock_groups -asynchronous -group { clock clock_div clock_inv } -group { jtag_tck }


set_load 0.005 [all_outputs]
set_load 0.02 [get_ports serial_out]
set_input_delay 0.4 -clock clock -max [get_ports io_in] -add_delay
set_input_delay 0.1 -clock clock -min [get_ports io_in] -add_delay
set_output_delay 0.5 -clock clock  [get_ports io_out] -add_delay
