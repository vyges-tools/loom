//! The keys (flags that take a value) and flags (that take none) each SDC command accepts, in
//! the order the reference matches an ABBREVIATION against them: an argument that is not one
//! exactly is the FIRST key it begins, else the first flag — not the unique one. Generated from
//! the reference's command definitions; regenerate on a repin. Command NAMES (what is known, and
//! what a command abbreviation resolves to) are the reference's own lists in `sdc_known/`.

/// The commands of [`FLAGS`] whose reference parse passes an option it does not know through to
/// the command (`unknown_key_is_error` 0) rather than failing on it: they parse their own lists
/// (`-through`, `-group`) after it. Every OTHER command of [`FLAGS`] fails on an option that is
/// neither one of its keys or flags nor the beginning of one (the reference's Error 562). Observed
/// on the reference's own parser, 2026-10-31.
pub(crate) const PASSES_UNKNOWN: &[&str] = &[
    "group_path",
    "set_clock_groups",
    "set_clock_sense",
    "set_false_path",
    "set_max_delay",
    "set_min_delay",
    "set_multicycle_path",
    "set_path_margin",
    "set_sense",
    "unset_path_exceptions",
];

/// `(command, keys, flags)`.
pub(crate) const FLAGS: &[(&str, &[&str], &[&str])] = &[
    ("all_inputs", &[], &["-no_clocks"]),
    ("all_registers", &["-clock", "-rise_clock", "-fall_clock"], &["-cells", "-data_pins", "-clock_pins", "-async_pins", "-output_pins", "-level_sensitive", "-edge_triggered"]),
    ("create_clock", &["-name", "-period", "-waveform", "-comment"], &["-add"]),
    ("create_generated_clock", &["-name", "-source", "-master_clock", "-divide_by", "-multiply_by", "-duty_cycle", "-edges", "-edge_shift", "-comment"], &["-invert", "-combinational", "-add"]),
    ("delete_clock", &[], &["-all"]),
    ("delete_generated_clock", &[], &["-all"]),
    ("get_cells", &["-hsc", "-filter", "-of_objects"], &["-hierarchical", "-regexp", "-nocase", "-quiet"]),
    ("get_clocks", &["-filter"], &["-regexp", "-nocase", "-quiet"]),
    ("get_lib_cells", &["-hsc", "-of_objects", "-filter"], &["-regexp", "-nocase", "-quiet"]),
    ("get_lib_pins", &["-hsc", "-of_objects", "-filter"], &["-regexp", "-nocase", "-quiet"]),
    ("get_libs", &["-filter"], &["-regexp", "-nocase", "-quiet"]),
    ("get_nets", &["-hsc", "-of_objects", "-filter"], &["-hierarchical", "-regexp", "-nocase", "-quiet"]),
    ("get_pins", &["-hsc", "-of_objects", "-filter"], &["-hierarchical", "-regexp", "-nocase", "-quiet"]),
    ("get_ports", &["-of_objects", "-filter"], &["-regexp", "-nocase", "-quiet"]),
    ("group_path", &["-name", "-weight", "-critical_range", "-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-default"]),
    ("set_clock_gating_check", &["-setup", "-hold"], &["-rise", "-fall", "-high", "-low"]),
    ("set_clock_groups", &["-name", "-comment"], &["-logically_exclusive", "-physically_exclusive", "-asynchronous", "-allow_paths"]),
    ("set_clock_latency", &["-clock"], &["-rise", "-fall", "-min", "-max", "-source", "-late", "-early"]),
    ("set_clock_sense", &["-clock", "-clocks", "-pulse"], &["-positive", "-negative", "-stop_propagation"]),
    ("set_clock_transition", &[], &["-rise", "-fall", "-max", "-min"]),
    ("set_clock_uncertainty", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to"], &["-rise", "-fall", "-setup", "-hold"]),
    ("set_data_check", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-clock"], &["-setup", "-hold"]),
    ("set_disable_timing", &["-from", "-to"], &[]),
    ("set_drive", &[], &["-rise", "-fall", "-min", "-max"]),
    ("set_driving_cell", &["-lib_cell", "-cell", "-library", "-pin", "-from_pin", "-multiply_by", "-input_transition_rise", "-input_transition_fall"], &["-rise", "-fall", "-min", "-max", "-dont_scale", "-no_design_rule"]),
    ("set_false_path", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-setup", "-hold", "-rise", "-fall", "-reset_path"]),
    ("set_input_delay", &["-clock", "-reference_pin"], &["-rise", "-fall", "-max", "-min", "-clock_fall", "-add_delay", "-source_latency_included", "-network_latency_included"]),
    ("set_input_transition", &["-clock"], &["-rise", "-fall", "-max", "-min", "-clock_fall"]),
    ("set_load", &[], &["-rise", "-fall", "-min", "-max", "-subtract_pin_load", "-pin_load", "-wire_load"]),
    ("set_max_delay", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-rise", "-fall", "-ignore_clock_latency", "-reset_path", "-probe"]),
    ("set_max_transition", &[], &["-clock_path", "-data_path", "-rise", "-fall"]),
    ("set_min_delay", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-rise", "-fall", "-ignore_clock_latency", "-reset_path", "-probe"]),
    ("set_min_pulse_width", &[], &["-low", "-high"]),
    ("set_multicycle_path", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-setup", "-hold", "-rise", "-fall", "-start", "-end", "-reset_path"]),
    ("set_operating_conditions", &["-analysis_type", "-library", "-min", "-max", "-min_library", "-max_library"], &[]),
    ("set_output_delay", &["-clock", "-reference_pin"], &["-rise", "-fall", "-max", "-min", "-clock_fall", "-add_delay", "-source_latency_included", "-network_latency_included"]),
    ("set_path_margin", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-rise", "-fall", "-setup", "-hold"]),
    ("set_port_fanout_number", &[], &["-max", "-min"]),
    ("set_pvt", &["-process", "-voltage", "-temperature"], &["-min", "-max"]),
    ("set_resistance", &[], &["-max", "-min"]),
    ("set_sense", &["-type"], &[]),
    ("set_timing_derate", &[], &["-rise", "-fall", "-early", "-late", "-clock", "-data", "-net_delay", "-cell_delay", "-cell_check"]),
    ("set_units", &["-capacitance", "-resistance", "-time", "-voltage", "-current", "-power", "-distance"], &[]),
    ("set_voltage", &["-min", "-object_list"], &[]),
    ("set_wire_load_model", &["-name", "-library"], &["-min", "-max"]),
    ("set_wire_load_selection_group", &["-library"], &["-min", "-max"]),
    ("unset_clock_groups", &["-name"], &["-logically_exclusive", "-physically_exclusive", "-asynchronous", "-all"]),
    ("unset_clock_latency", &["-clock"], &["-source"]),
    ("unset_clock_uncertainty", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to"], &["-rise", "-fall", "-setup", "-hold"]),
    ("unset_disable_timing", &["-from", "-to"], &[]),
    ("unset_input_delay", &["-clock", "-reference_pin"], &["-rise", "-fall", "-max", "-min", "-clock_fall"]),
    ("unset_output_delay", &["-clock", "-reference_pin"], &["-rise", "-fall", "-max", "-min", "-clock_fall"]),
    ("unset_path_exceptions", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to"], &["-setup", "-hold", "-rise", "-fall"]),
    ("write_sdc", &["-mode", "-digits"], &["-map_hpins", "-compatible", "-gzip", "-no_timestamp"]),
];
