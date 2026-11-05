//! Every option an SDC version declares is read, never refused: the public SDC parser's (SDC 1.1
//! to 2.0) and the ones newer vendor releases of it declare in SDC 1.8, listed in
//! `tests/data/sdc_declared_options.txt`. Each is read alone, in one line that uses its command
//! the simplest way, and the reader must neither stop nor refuse the command. A query option that
//! selects by the design (`-of_objects`, `-filter`, `-regexp`) is not evaluated without one: that
//! is a command not applied, said as such, not an option refused.

use vyges_loom::sdc::Sdc;

const SETUP: &str = "create_clock -name c -period 10 [get_ports clk]\n";

/// `(command, option, kind, version, source)` per data line.
fn declared() -> Vec<(&'static str, &'static str, &'static str, &'static str, &'static str)> {
    include_str!("data/sdc_declared_options.txt")
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
        .map(|l| {
            let w: Vec<&str> = l.split_whitespace().collect();
            assert_eq!(w.len(), 5, "{l}");
            (w[0], w[1], w[2], w[3], w[4])
        })
        .collect()
}

/// A value for option `o` of kind `kind`, as a file would give it.
fn value(cmd: &str, o: &str, kind: &str) -> String {
    let v = match (cmd, o) {
        (_, "-hsc") => "/",
        ("set_clock_sense", "-pulse") => "rise_triggered_high_pulse",
        (_, "-analysis_type") => "single",
        (_, "-rule") => "all",
        (_, "-unit") => "mW",
        (_, "-time") => "ns",
        (_, "-capacitance") => "pF",
        (_, "-resistance") => "kOhm",
        (_, "-voltage") if cmd.starts_with("set_unit") => "V",
        (_, "-current") => "mA",
        (_, "-power") => "mW",
        (_, "-filter") => "{name==x}",
        ("get_cells" | "get_cell" | "get_pins", "-of_objects") => "[get_nets n1]",
        (_, "-of_objects") => "[get_cells u1]",
        (_, "-clock" | "-clocks" | "-master_clock" | "-group") => "c",
        ("all_registers", "-rise_clock" | "-fall_clock") => "c",
        (_, "-source") => "[get_ports clk]",
        (_, "-reference_pin") => "[get_pins r/CK]",
        (_, "-waveform") => "{0 5}",
        (_, "-edges") => "{1 3 5}",
        (_, "-edge_shift") => "{0 0 0}",
        (_, "-coordinate") => "{0 0 10 10}",
        (_, "-object_list") => "[get_cells u1]",
        (_, "-sms_scenarios") => "{s1 s2}",
        (_, "-library" | "-min_library" | "-max_library") => "lib",
        (_, "-lib_cell") => "BUF",
        (_, "-pin") => "Z",
        (_, "-from_pin") => "A",
        ("set_disable_timing", "-from") => "A",
        ("set_disable_timing", "-to") => "Z",
        ("set_operating_conditions", "-min" | "-max") => "typ",
        (_, "-instance") => "u1",
        (_, "-period") => "10",
        (_, "-duty_cycle") => "50",
        _ if o.contains("from") || o.contains("to") || o.contains("throug") => "[get_ports a]",
        _ => match kind {
            "String" => "x",
            "Int" => "2",
            "Float" => "1",
            "List" => "[get_ports a]",
            "Enum" => "x",
            _ => panic!("{cmd} {o}: kind {kind}"),
        },
    };
    v.to_string()
}

/// One line that uses `cmd` with option `name` (written `opt`, with its value) the simplest way
/// the command allows it: with the options it requires (`-add` needs `-master_clock`, `-early`
/// needs `-source`).
fn line(cmd: &str, name: &str, opt: &str) -> String {
    let template = match cmd {
        "all_inputs" | "all_outputs" => format!("set_false_path -to [{cmd} {opt}]"),
        "all_registers" => format!("set_false_path -to [{cmd} {opt}]"),
        "create_clock" => match name {
            "-name" | "-period" => format!("create_clock -period 10 -name d {opt} [get_ports clk2]"),
            _ => format!("create_clock -name d -period 10 {opt} [get_ports clk2]"),
        },
        "create_generated_clock" => match name {
            "-multiply_by" | "-edges" | "-duty_cycle" | "-edge_shift" | "-invert" | "-combinational" | "-add" => {
                let base = match name {
                    "-edge_shift" => "-edges {1 3 5}",
                    "-duty_cycle" => "-multiply_by 2",
                    "-invert" => "-divide_by 2",
                    "-add" => "-divide_by 2 -master_clock c",
                    _ => "",
                };
                format!("create_generated_clock -name g -source [get_ports clk] {base} {opt} [get_pins r/Q]")
            }
            "-name" | "-source" | "-divide_by" => "create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins r/Q]".to_string(),
            _ => format!("create_generated_clock -name g -source [get_ports clk] -divide_by 2 {opt} [get_pins r/Q]"),
        },
        "create_voltage_area" => format!("create_voltage_area -name va {opt} [get_cells u1]"),
        "current_instance" => format!("current_instance {opt}\ncurrent_instance"),
        c if c.starts_with("get_lib") => format!("set_disable_timing [{c} {opt} lib/BUF]"),
        "get_libs" => format!("set_driving_cell -lib_cell BUF -library [get_libs {opt} lib] [get_ports a]"),
        "get_clock" | "get_clocks" => format!("set_clock_uncertainty 0.1 [{cmd} {opt} c]"),
        "get_port" | "get_ports" => format!("set_input_delay 1 -clock c [{cmd} {opt} a]"),
        "get_cell" | "get_cells" | "get_net" | "get_nets" | "get_pin" | "get_pins" => {
            let pat = if cmd.starts_with("get_pin") { "r/D" } else if cmd.starts_with("get_net") { "n1" } else { "u1" };
            let pat = if name == "-of_objects" { "" } else { pat };
            format!("set_false_path -through [{cmd} {opt} {pat}]")
        }
        "group_path" => match name {
            "-default" => "group_path -default -to [get_ports y]".to_string(),
            _ => format!("group_path -name g -to [get_ports y] {opt}"),
        },
        "set_clock_gating_check" => match name {
            "-high" | "-low" => format!("set_clock_gating_check -setup 0.1 {opt} [get_cells u1]"),
            _ => format!("set_clock_gating_check -setup 0.1 {opt} [get_clocks c]"),
        },
        "set_clock_group" | "set_clock_groups" => format!("{cmd} -asynchronous -name g -group c {opt}"),
        "set_clock_latency" => match name {
            "-early" | "-late" => format!("set_clock_latency -source 0.5 {opt} [get_clocks c]"),
            _ => format!("set_clock_latency 0.5 {opt} [get_clocks c]"),
        },
        "set_clock_sense" => format!("set_clock_sense {opt} [get_pins u1/Z]"),
        "set_clock_transition" => format!("set_clock_transition {opt} 0.1 [get_clocks c]"),
        "set_clock_uncertainty" => match name {
            o if o.contains("from") => format!("set_clock_uncertainty {name} c -to c 0.1"),
            o if o.contains("to") => format!("set_clock_uncertainty -from c {name} c 0.1"),
            _ => format!("set_clock_uncertainty -from c -to c {opt} 0.1"),
        },
        "set_data_check" => match name {
            o if o.contains("from") => format!("set_data_check {name} [get_pins r/D] -to [get_pins s/D] 0.1"),
            o if o.contains("to") => format!("set_data_check -from [get_pins r/D] {name} [get_pins s/D] 0.1"),
            _ => format!("set_data_check -from [get_pins r/D] -to [get_pins s/D] {opt} 0.1"),
        },
        "set_disable_timing" => format!("set_disable_timing {opt} [get_cells u1]"),
        "set_drive" | "set_resistance" => format!("{cmd} {opt} 1 [get_ports a]"),
        "set_driving_cell" => match opt {
            "-lib_cell" => "set_driving_cell -lib_cell BUF [get_ports a]".to_string(),
            _ => format!("set_driving_cell -lib_cell BUF {opt} [get_ports a]"),
        },
        "set_false_path" | "set_max_delay" | "set_min_delay" | "set_multicycle_path" => {
            let n = match cmd {
                "set_false_path" => "",
                "set_multicycle_path" => "2",
                _ => "1",
            };
            let path = if name.contains("from") || name.contains("to") || name.contains("throug") { "" } else { "-to [get_ports y]" };
            format!("{cmd} {n} {path} {opt}")
        }
        "set_ideal_latency" | "set_ideal_transition" => format!("{cmd} {opt} 0.1 [get_pins r/CK]"),
        "set_ideal_network" => format!("set_ideal_network {opt} [get_ports clk]"),
        "set_input_delay" | "set_output_delay" => match name {
            "-clock" => format!("{cmd} 1 -clock c [get_ports a]"),
            "-clock_fall" | "-add_delay" | "-reference_pin" | "-level_sensitive" | "-network_latency_included" | "-source_latency_included" => {
                format!("{cmd} 1 -clock c {opt} [get_ports a]")
            }
            _ => format!("{cmd} 1 -clock c {opt} [get_ports a]"),
        },
        "set_input_transition" => match name {
            "-clock_fall" => format!("set_input_transition -clock c {opt} 0.1 [get_ports a]"),
            _ => format!("set_input_transition {opt} 0.1 [get_ports a]"),
        },
        "set_level_shifter_strategy" | "set_level_shifter_threshold" => format!("{cmd} {opt}"),
        "set_load" => format!("set_load {opt} 0.1 [get_ports y]"),
        "set_max_capacitance" | "set_max_transition" => format!("{cmd} {opt} 0.2 [current_design]"),
        "set_max_dynamic_power" | "set_max_leakage_power" => format!("{cmd} 1 {opt}"),
        "set_min_pulse_width" => format!("set_min_pulse_width {opt} 1 [get_clocks c]"),
        "set_operating_conditions" => format!("set_operating_conditions {opt} typ"),
        "set_timing_derate" => match name {
            "-early" | "-late" => format!("set_timing_derate {opt} 1.05"),
            _ => format!("set_timing_derate -late {opt} 1.05"),
        },
        "set_unit" | "set_units" => format!("{cmd} {opt}"),
        "set_voltage" => format!("set_voltage 1.0 {opt}"),
        "set_wire_load_model" => match name {
            "-name" => "set_wire_load_model -name wl".to_string(),
            _ => format!("set_wire_load_model -name wl {opt}"),
        },
        "set_wire_load_selection_group" => match name {
            "-group_name" => format!("set_wire_load_selection_group {opt}"),
            _ => format!("set_wire_load_selection_group {opt} sel"),
        },
        _ => panic!("{cmd}: no line for it here; add one"),
    };
    template
}

/// Why `line` is refused, or stops the read: `None` when it is read. A query not evaluated
/// without the design is read (and listed as not applied), not refused.
fn refusal(line: &str) -> Option<String> {
    match Sdc::parse(&format!("{SETUP}{line}\n")) {
        Err(e) => Some(format!("the read stopped: {e:?}")),
        Ok(s) => s.rejected.into_iter().find(|r| !r.contains("not evaluated")),
    }
}

#[test]
fn every_declared_option_is_read_never_refused() {
    let rows = declared();
    assert!(rows.len() > 300, "{} rows", rows.len());
    let mut refused = Vec::new();
    for (cmd, opt, kind, _, _) in &rows {
        let o = if *kind == "Flag" { opt.to_string() } else { format!("{opt} {}", value(cmd, opt, kind)) };
        let l = line(cmd, opt, &o);
        if let Some(why) = refusal(&l) {
            refused.push(format!("{cmd} {opt}: `{l}`: {why}"));
        }
    }
    assert!(refused.is_empty(), "{} declared option(s) refused:\n{}", refused.len(), refused.join("\n"));
}

/// The check can fail: an option no SDC version declares is refused in the same lines.
#[test]
fn an_option_no_version_declares_is_refused_in_the_same_lines() {
    for cmd in ["set_load", "set_max_transition", "set_input_delay", "set_false_path", "get_ports", "set_clock_latency"] {
        let l = line(cmd, "-bogus_option", "-bogus_option");
        assert!(refusal(&l).is_some(), "{cmd}: `{l}` was read");
    }
}

/// Both sources are present, and the vendor rows are SDC 1.8.
#[test]
fn the_list_has_both_sources() {
    let rows = declared();
    assert!(rows.iter().any(|r| r.4 == "public"));
    assert!(rows.iter().filter(|r| r.4 == "vendor").all(|r| r.3 == "1.8"));
    assert!(rows.iter().any(|r| r.4 == "vendor"));
}
