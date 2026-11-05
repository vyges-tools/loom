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
    // the public SDC parser and the reference: `value object_list`, no option (newer vendor SDC
    // 1.8 adds some: `VENDOR_18_ONLY`)
    ("set_max_capacitance", &[], &[]),
    ("set_max_delay", &["-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-comment"], &["-rise", "-fall", "-ignore_clock_latency", "-reset_path", "-probe"]),
    ("set_max_fanout", &[], &[]),
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

/// `(command, keys, flags)` the SDC standard declares that the reference timer's parse does not
/// have: the reference SDC parser's declarations (SDC 1.1 to 2.0) less [`FLAGS`]. They are valid
/// SDC, so they are read, not refused (the reference timer refuses them, Error 562; the difference
/// is recorded in the internal spec matrix). Each is applied where the model can apply it and is
/// otherwise kept, written back, and reported as kept-not-applied — see [`KEPT_OPTIONS`].
/// Matched after [`FLAGS`], so an abbreviation the reference resolves still resolves as it does.
pub(crate) const STANDARD_ONLY: &[(&str, &[&str], &[&str])] = &[
    ("all_inputs", &["-clock"], &["-level_sensitive", "-edge_triggered"]),
    ("all_outputs", &["-clock"], &["-level_sensitive", "-edge_triggered"]),
    ("all_registers", &[], &["-no_hierarchy", "-slave_clock_pins", "-master_slave"]),
    ("current_instance", &["-instance"], &[]),
    ("get_ports", &[], &["-hierarchical"]),
    ("set_data_check", &[], &["-rise", "-fall"]),
    ("set_driving_cell", &["-clock"], &["-clock_fall"]),
    // the parser's own spelling (1.7 on) of `-rise_through` / `-fall_through`, read as them
    ("set_false_path", &["-rise_throough", "-fall_throough"], &[]),
    ("set_input_delay", &[], &["-level_sensitive"]),
    // the parser's own spelling of `-subtract_pin_load`, read as it
    ("set_load", &[], &["-substract_pin_load"]),
    ("set_max_dynamic_power", &["-unit"], &[]),
    ("set_max_leakage_power", &["-unit"], &[]),
    ("set_operating_conditions", &["-object_list"], &[]),
    ("set_output_delay", &[], &["-level_sensitive"]),
    ("set_timing_derate", &[], &["-min", "-max"]),
    // SDC 1.1 and 1.2 name the group with `-group_name`; 1.3 on make it the positional
    ("set_wire_load_selection_group", &["-group_name"], &[]),
];

/// The [`STANDARD_ONLY`] options no consumer here applies: a command using one is kept and
/// written back with it, and `check` reports it as kept-not-applied. The others are applied:
/// `get_ports -hierarchical` is `get_ports` (a port is at the top level, with no hierarchy to
/// descend), `set_load -substract_pin_load` is `-subtract_pin_load`, and `current_instance
/// -instance i` is `current_instance i`.
pub(crate) const KEPT_OPTIONS: &[(&str, &str)] = &[
    ("all_inputs", "-clock"),
    ("all_inputs", "-level_sensitive"),
    ("all_inputs", "-edge_triggered"),
    ("all_outputs", "-clock"),
    ("all_outputs", "-level_sensitive"),
    ("all_outputs", "-edge_triggered"),
    ("all_registers", "-no_hierarchy"),
    ("all_registers", "-slave_clock_pins"),
    ("all_registers", "-master_slave"),
    ("set_data_check", "-rise"),
    ("set_data_check", "-fall"),
    ("set_driving_cell", "-clock"),
    ("set_driving_cell", "-clock_fall"),
    ("set_input_delay", "-level_sensitive"),
    ("set_output_delay", "-level_sensitive"),
    ("set_timing_derate", "-min"),
    ("set_timing_derate", "-max"),
    ("set_max_dynamic_power", "-unit"),
    ("set_max_leakage_power", "-unit"),
    ("set_operating_conditions", "-object_list"),
    ("set_wire_load_selection_group", "-group_name"),
    ("set_load", "-internal"),
    ("set_load", "-allow_negative_cap"),
    ("set_max_capacitance", "-clock_path"),
    ("set_max_capacitance", "-data_path"),
    ("set_max_capacitance", "-rise"),
    ("set_max_capacitance", "-fall"),
    ("set_max_capacitance", "-force"),
    ("set_max_capacitance", "-sms_scenarios"),
    ("set_max_transition", "-delay_penalty"),
    ("set_max_transition", "-force"),
];

/// `(command, keys, flags)` that newer releases of the vendor's SDC parser declare in SDC 1.8 and
/// that neither the public release ([`STANDARD_ONLY`]'s source) nor the reference timer
/// ([`FLAGS`]) has. They are SDC 1.8 to a user of those releases, so they are read, not refused:
/// each is kept with its command ([`KEPT_OPTIONS`]). `set_max_transition -delay_penalty` is a
/// flag there (it takes no value), so the word after it is the limit.
pub(crate) const VENDOR_18_ONLY: &[(&str, &[&str], &[&str])] = &[
    ("set_load", &[], &["-internal", "-allow_negative_cap"]),
    ("set_max_capacitance", &["-sms_scenarios"], &["-clock_path", "-data_path", "-rise", "-fall", "-force"]),
    ("set_max_transition", &[], &["-delay_penalty", "-force"]),
];

/// Every `(command, option)` newer releases of the vendor's SDC parser declare in SDC 1.8 that its
/// public release declares in no version: the reference timer's own options among them (`get_pins
/// -of_objects`, the queries' `-filter` and `-quiet`), which are therefore not
/// [`EXTENSION_OPTIONS`], and [`VENDOR_18_ONLY`]'s. The coverage table marks each one †.
pub(crate) const VENDOR_18: &[(&str, &str)] = &[
    ("get_cells", "-filter"),
    ("get_cells", "-quiet"),
    ("get_clocks", "-quiet"),
    ("get_lib_cells", "-filter"),
    ("get_lib_cells", "-quiet"),
    ("get_lib_pins", "-filter"),
    ("get_lib_pins", "-quiet"),
    ("get_libs", "-filter"),
    ("get_libs", "-quiet"),
    ("get_nets", "-filter"),
    ("get_nets", "-quiet"),
    ("get_pins", "-filter"),
    ("get_pins", "-of_objects"),
    ("get_pins", "-quiet"),
    ("get_ports", "-filter"),
    ("get_ports", "-quiet"),
    ("set_load", "-rise"),
    ("set_load", "-fall"),
    ("set_load", "-internal"),
    ("set_load", "-allow_negative_cap"),
    ("set_max_capacitance", "-clock_path"),
    ("set_max_capacitance", "-data_path"),
    ("set_max_capacitance", "-rise"),
    ("set_max_capacitance", "-fall"),
    ("set_max_capacitance", "-force"),
    ("set_max_capacitance", "-sms_scenarios"),
    ("set_max_transition", "-data_path"),
    ("set_max_transition", "-delay_penalty"),
    ("set_max_transition", "-force"),
];

/// A command's keys and flags: the reference timer's ([`FLAGS`]), then the standard's
/// ([`STANDARD_ONLY`]), then newer vendor SDC 1.8's ([`VENDOR_18_ONLY`]); `None` for a command
/// none lists.
pub(crate) fn options(cmd: &str) -> Option<(Vec<&'static str>, Vec<&'static str>)> {
    let reference = FLAGS.iter().find(|(n, _, _)| *n == cmd);
    let standard = STANDARD_ONLY.iter().find(|(n, _, _)| *n == cmd);
    let vendor = VENDOR_18_ONLY.iter().find(|(n, _, _)| *n == cmd);
    if reference.is_none() && standard.is_none() && vendor.is_none() {
        return None;
    }
    let (mut keys, mut flags) = (Vec::new(), Vec::new());
    for (_, k, f) in reference.into_iter().chain(standard).chain(vendor) {
        keys.extend_from_slice(k);
        flags.extend_from_slice(f);
    }
    Some((keys, flags))
}

/// [`options`]' keys alone (none for a command neither table lists).
pub(crate) fn keys(cmd: &str) -> Vec<&'static str> {
    options(cmd).map(|(k, _)| k).unwrap_or_default()
}

/// The reference timer's options that no SDC version declares (1.1 to 2.0 in the public reference
/// SDC parser, nor 1.8 in the vendor's newer releases: [`VENDOR_18`]), `(command, option)`. Read, as the reference reads them; `check` names each one a file
/// uses, at info level, so a file that leans on them is known not to be portable SDC.
pub(crate) const EXTENSION_OPTIONS: &[(&str, &str)] = &[
    ("all_inputs", "-no_clocks"),
    ("get_clocks", "-filter"),
    ("get_lib_cells", "-of_objects"),
    ("get_lib_pins", "-of_objects"),
    ("get_ports", "-nocase"),
    ("get_ports", "-of_objects"),
    ("group_path", "-critical_range"),
    ("set_clock_sense", "-clock"),
    ("set_driving_cell", "-cell"),
    ("set_false_path", "-reset_path"),
    ("set_max_delay", "-ignore_clock_latency"),
    ("set_max_delay", "-probe"),
    ("set_max_delay", "-reset_path"),
    ("set_min_delay", "-ignore_clock_latency"),
    ("set_min_delay", "-probe"),
    ("set_min_delay", "-reset_path"),
    ("set_multicycle_path", "-reset_path"),
    ("set_port_fanout_number", "-max"),
    ("set_port_fanout_number", "-min"),
    ("set_units", "-distance"),
];

/// The reference timer's commands that no SDC version declares and that a constraint file may
/// use: its undo commands (`unset_*`, `delete_*`; a signoff timer's `remove_*` / `reset_*` are
/// read as these), its own constraints, and `write_sdc`.
pub(crate) const EXTENSION_COMMANDS: &[&str] = &[
    "delete_clock",
    "delete_generated_clock",
    "set_disable_clock_gating_check",
    "set_path_margin",
    "set_pvt",
    "unset_case_analysis",
    "unset_clock_groups",
    "unset_clock_latency",
    "unset_clock_transition",
    "unset_clock_uncertainty",
    "unset_data_check",
    "unset_disable_clock_gating_check",
    "unset_disable_timing",
    "unset_input_delay",
    "unset_output_delay",
    "unset_path_exceptions",
    "unset_propagated_clock",
    "unset_timing_derate",
    "write_sdc",
];
