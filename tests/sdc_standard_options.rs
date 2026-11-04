//! Options the SDC standard declares that the reference timer's parse lacks. Each is valid SDC, so
//! it is read, never refused and never dropped: applied where the model can apply it, else kept,
//! written back, and reported as kept-not-applied (`Sdc::kept_options`). The standard here is the
//! reference SDC parser's declaration of each command (SDC 1.8 and 2.0).

use vyges_loom::sdc::Sdc;
use vyges_loom::sdc_write::WriteOpts;

const CLOCK: &str = "create_clock -name c -period 10 [get_ports clk]\n";

fn read(line: &str) -> Sdc {
    Sdc::parse(&format!("{CLOCK}{line}\n")).unwrap_or_else(|e| panic!("{line}: {e:?}"))
}

fn written(s: &Sdc) -> String {
    let w = s.write(&WriteOpts::default());
    assert!(w.not_written.is_empty(), "{:?}\n{}", w.not_written, w.text);
    w.text
}

fn kept(s: &Sdc) -> Vec<(String, usize)> {
    s.kept_not_applied().into_iter().map(|k| (k.command, k.count)).collect()
}

/// Read, not refused, nothing listed as not applied, and reported as kept-not-applied as `want`.
fn accepted(line: &str, want: &[&str]) -> Sdc {
    let s = read(line);
    assert!(s.rejected.is_empty(), "{line}: refused: {:?}", s.rejected);
    assert!(s.not_applied().is_empty(), "{line}: {:?}", s.not_applied());
    let got: Vec<String> = kept(&s).into_iter().map(|(c, _)| c).collect();
    assert_eq!(got, want, "{line}");
    s
}

/// SDC 1.8 `set_input_delay` / `set_output_delay` declare `-level_sensitive` (a delay from a
/// level-sensitive latch). The delay is applied as an I/O delay (the port is constrained), with
/// the flag in its record and written back; nothing here times a latch launch, so it is reported.
#[test]
fn level_sensitive_io_delays_are_read_with_the_flag() {
    for cmd in ["set_input_delay", "set_output_delay"] {
        let s = accepted(&format!("{cmd} 1 -clock c -level_sensitive [get_ports a]"), &[&format!("{cmd} -level_sensitive")]);
        let d = if cmd == "set_input_delay" { &s.input_delays } else { &s.output_delays };
        assert_eq!(d.len(), 1, "{cmd}");
        assert!(d[0].flags.iter().any(|f| f == "-level_sensitive"), "{:?}", d[0].flags);
        let text = written(&s);
        assert!(text.lines().any(|l| l.starts_with(cmd) && l.contains("-level_sensitive")), "{text}");
    }
}

/// SDC 1.5+ `set_timing_derate` declares `-min` / `-max` (the min or max operating condition) next
/// to `-early` / `-late`. Read with them in its flags; it then sets no design-wide derate (that
/// would apply it to both conditions), is written back, and is reported.
#[test]
fn a_min_max_derate_is_kept_with_its_condition() {
    let s = accepted("set_timing_derate -late -max 1.05", &["set_timing_derate -max"]);
    assert_eq!(s.late_derate, None);
    assert_eq!(s.derates.len(), 1);
    assert!(s.derates[0].flags.iter().any(|f| f == "-max"), "{:?}", s.derates[0].flags);
    assert!(written(&s).contains("set_timing_derate -late -max 1.05"), "{}", written(&s));
}

/// SDC 1.8 `set_driving_cell` declares `-clock` and `-clock_fall` (the drive holds for delays from
/// that clock edge). No drive here is clock-scoped, so the command is kept as written — not
/// applied as an unscoped drive — and both options are reported.
#[test]
fn a_clock_scoped_driving_cell_is_kept_not_applied_unscoped() {
    let s = accepted("set_driving_cell -lib_cell BUF -clock c -clock_fall [get_ports a]", &["set_driving_cell -clock", "set_driving_cell -clock_fall"]);
    assert!(s.driving_cells.is_empty(), "applied unscoped: {:?}", s.driving_cells);
    let text = written(&s);
    let line = text.lines().find(|l| l.starts_with("set_driving_cell")).unwrap_or_else(|| panic!("{text}"));
    for w in ["-lib_cell BUF", "-clock c", "-clock_fall", "[get_ports {a}]"] {
        assert!(line.contains(w), "{w}: {line}");
    }
}

/// SDC 1.1+ `set_load` declares `-min -max -substract_pin_load -pin_load -wire_load`: the parser's
/// spelling of `-subtract_pin_load`, read as it. SDC declares no `-internal`: still refused.
#[test]
fn set_load_reads_the_standard_spelling_and_refuses_what_it_lacks() {
    let s = accepted("set_load -substract_pin_load 0.1 [get_ports y]", &[]);
    assert!(s.env[0].flags.iter().any(|f| f == "-subtract_pin_load"), "{:?}", s.env[0].flags);
    let s = read("set_load -internal 0.1 [get_ports y]");
    assert!(!s.rejected.is_empty(), "-internal is not SDC: {:?}", s.env);
}

/// SDC 1.1+ `get_ports` declares `-hierarchical`. A port is at the top level, so it selects the
/// same ports; applied, and written as the plain query.
#[test]
fn get_ports_hierarchical_is_get_ports() {
    let s = accepted("set_input_delay 1 -clock c [get_ports -hierarchical a]", &[]);
    assert_eq!(s.input_delays[0].ports, ["a"]);
    let s2 = accepted("set_input_delay 1 -clock c [get_ports -hier a]", &[]);
    assert_eq!(s2.input_delays[0].ports, ["a"]);
    assert_eq!(written(&s), written(&read("set_input_delay 1 -clock c [get_ports a]")));
}

/// SDC 1.1+ `all_inputs` / `all_outputs` declare `-clock`, `-level_sensitive`, `-edge_triggered`
/// (the ports delayed from that clock, or by that kind of delay); SDC 1.7+ `all_registers`
/// declares `-no_hierarchy`, `-slave_clock_pins`, `-master_slave`. None is evaluated here, so the
/// command naming them is kept as written, not applied to every port or register.
#[test]
fn standard_collection_options_keep_their_command() {
    for (line, opt) in [
        ("set_input_delay 1 -clock c [all_inputs -clock c]", "all_inputs -clock"),
        ("set_input_delay 1 -clock c [all_inputs -edge_triggered]", "all_inputs -edge_triggered"),
        ("set_output_delay 1 -clock c [all_outputs -level_sensitive]", "all_outputs -level_sensitive"),
        ("set_false_path -to [all_registers -master_slave]", "all_registers -master_slave"),
        ("set_false_path -to [all_registers -no_hierarchy]", "all_registers -no_hierarchy"),
    ] {
        let s = accepted(line, &[opt]);
        assert!(s.input_delays.is_empty() && s.output_delays.is_empty() && s.exceptions.is_empty(), "{line}: applied");
        let text = written(&s);
        let o = opt.split(' ').nth(1).unwrap();
        assert!(text.lines().any(|l| l.contains(o) && l.starts_with(line.split(' ').next().unwrap())), "{line}:\n{text}");
    }
    // the same option twice: one entry, counted twice
    let s = read("set_input_delay 1 -clock c [all_inputs -clock c]\nset_input_delay 2 -clock c [all_inputs -clock c]");
    assert_eq!(kept(&s), [("all_inputs -clock".to_string(), 2)]);
}

/// What the standard does not declare is still refused: an `all_outputs` positional, an unknown
/// `all_inputs` option.
#[test]
fn what_the_standard_lacks_is_still_refused() {
    for line in ["set_output_delay 1 -clock c [all_outputs y]", "set_input_delay 1 -clock c [all_inputs -bogus]"] {
        assert!(!read(line).rejected.is_empty(), "{line}");
    }
}

/// `check` reports a kept option as it reports a kept command: a warning, with its count.
#[test]
fn check_reports_a_kept_option() {
    let s = read("set_driving_cell -lib_cell BUF -clock c [get_ports a]");
    let r = vyges_loom::sdc_lint::lint_without_design(&s);
    let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-kept-not-applied").collect();
    assert_eq!(f.len(), 1, "{:?}", r.findings);
    assert!(f[0].message.contains("`set_driving_cell -clock`"), "{}", f[0].message);
    assert_eq!(r.errors(), 0, "{:?}", r.findings);
}

/// The kept commands' SDC options: SDC 1.4+ `set_max_dynamic_power` / `set_max_leakage_power`
/// declare `-unit` (the reference takes the unit as a second positional); SDC 1.1+
/// `set_operating_conditions` declares `-object_list`; SDC 1.1 to 1.9
/// `set_wire_load_selection_group` name the group with `-group_name` (2.0 makes it the
/// positional). Each is kept with its option, written back, and reported.
#[test]
fn the_kept_commands_read_their_sdc_options() {
    for (line, opt) in [
        ("set_max_dynamic_power 1 -unit mW", "set_max_dynamic_power -unit"),
        ("set_max_leakage_power 1 -unit uW", "set_max_leakage_power -unit"),
        ("set_operating_conditions -object_list [get_cells u1] typ", "set_operating_conditions -object_list"),
        ("set_wire_load_selection_group -group_name sel", "set_wire_load_selection_group -group_name"),
    ] {
        let s = read(line);
        assert!(s.rejected.is_empty(), "{line}: {:?}", s.rejected);
        let got: Vec<String> = kept(&s).into_iter().map(|(c, _)| c).collect();
        assert!(got.contains(&opt.to_string()), "{line}: {got:?}");
        let o = opt.split(' ').nth(1).unwrap();
        let text = written(&s);
        assert!(text.lines().any(|l| l.starts_with(line.split(' ').next().unwrap()) && l.contains(o)), "{line}:\n{text}");
    }
    assert!(!read("set_wire_load_selection_group").rejected.is_empty(), "no group at all is still refused");
}
