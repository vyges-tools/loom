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
/// spelling of `-subtract_pin_load`, read as it. Newer vendor SDC 1.8 adds `-internal` and
/// `-allow_negative_cap`: read, and kept with the command (no timer here applies them). An option
/// none of them declares is refused.
#[test]
fn set_load_reads_the_standard_spelling_and_refuses_what_it_lacks() {
    let s = accepted("set_load -substract_pin_load 0.1 [get_ports y]", &[]);
    assert!(s.env[0].flags.iter().any(|f| f == "-subtract_pin_load"), "{:?}", s.env[0].flags);
    for o in ["-internal", "-allow_negative_cap"] {
        let s = accepted(&format!("set_load {o} 0.1 [get_ports y]"), &[&format!("set_load {o}")]);
        assert!(s.env.is_empty(), "{o}: applied as a load: {:?}", s.env);
        assert!(written(&s).contains(&format!("set_load {o} 0.1 [get_ports {{y}}]")), "{o}: {}", written(&s));
    }
    let s = read("set_load -total 0.1 [get_ports y]");
    assert!(!s.rejected.is_empty(), "-total is no SDC option: {:?}", s.env);
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

/// SDC 1.4+ `set_data_check` declares `-rise` / `-fall` (a check on that edge of the data). Read
/// as SDC's flags — the reference timer takes `-rise` as an abbreviation of `-rise_from` — kept
/// with them, written back, and reported: no consumer here times a one-edge data check.
#[test]
fn a_one_edge_data_check_is_kept_with_its_edge() {
    for edge in ["-rise", "-fall"] {
        let line = format!("set_data_check {edge} -from [get_pins r/D] -to [get_pins s/D] 0.1");
        let s = accepted(&line, &[&format!("set_data_check {edge}")]);
        let text = written(&s);
        let w = text.lines().find(|l| l.starts_with("set_data_check")).unwrap_or_else(|| panic!("{text}"));
        assert!(w.split(' ').any(|t| t == edge) && w.contains("-from") && w.contains("0.1"), "{w}");
    }
}

/// SDC 1.7+ `set_ideal_network` spells the option `-no_propagate`; the reference timer
/// `-no_propagation`. Both are read as the option — not as the command's object, which stays the
/// port — and written back as written.
#[test]
fn ideal_network_reads_both_spellings() {
    for opt in ["-no_propagate", "-no_propagation"] {
        let s = read(&format!("set_ideal_network {opt} [get_ports clk]"));
        assert!(s.rejected.is_empty(), "{opt}: {:?}", s.rejected);
        let k = s.kept.iter().find(|k| k.cmd == "set_ideal_network").unwrap_or_else(|| panic!("{opt}: not kept"));
        assert!(k.has(opt), "{opt}: not read as the option: {k:?}");
        assert!(matches!(k.args.as_slice(), [vyges_loom::sdc::KeptArg::Objects(_)]), "{opt}: {:?}", k.args);
        assert!(written(&s).lines().any(|l| l.starts_with("set_ideal_network") && l.contains(opt)), "{opt}: {}", written(&s));
    }
}

/// SDC 1.1+ `current_instance` names the instance with `-instance`; the reference timer takes it
/// as the positional. Both set the same scope.
#[test]
fn current_instance_takes_the_instance_option() {
    let scoped = |cmd: &str| {
        let s = read(&format!("{cmd}\nset_false_path -to [get_pins r1/D]"));
        assert!(s.rejected.is_empty(), "{cmd}: {:?}", s.rejected);
        s.exceptions[0].args[0].1.names.clone()
    };
    assert_eq!(scoped("current_instance -instance u1"), ["u1/r1/D"]);
    assert_eq!(scoped("current_instance -instance u1"), scoped("current_instance u1"));
}

/// The exceptions and `set_clock_groups` take their lists (`-through`, `-rise_through`,
/// `-fall_through`; `-group`) after the options; SDC declares nothing else on them. On an
/// exception an option that is neither is refused, not passed through unread. `set_clock_groups`
/// is different in the reference (sdc/Sdc.tcl; probe `clock-groups-probe.py`, 2026-11-07): it
/// warns on the word (STA-0402, a stray word STA-0403), ignores it and applies the groups — so
/// it is applied here too, with the words listed as ignored.
#[test]
fn exception_and_clock_group_options_are_checked() {
    let s = read("set_false_path -from [get_ports a] -through [get_pins u/A] -fall_through [get_pins u/B] -rise_through [get_pins u/C] -to [get_ports y]");
    assert!(s.rejected.is_empty(), "{:?}", s.rejected);
    assert_eq!(s.exceptions.len(), 1);
    let s = read("set_clock_groups -asynchronous -group c -group [get_clocks c]");
    assert!(s.rejected.is_empty(), "{:?}", s.rejected);
    for line in [
        "set_false_path -bogus x -to [get_ports y]",
        "set_multicycle_path 2 -setup -thru_all [get_pins u/A]",
        "group_path -name g -bogus -to [get_ports y]",
    ] {
        assert!(!read(line).rejected.is_empty(), "{line}");
    }
    let s = read("set_clock_groups -asynchronous -group c -bogus c");
    assert!(s.rejected.is_empty(), "{:?}", s.rejected);
    assert_eq!(s.clock_groups.len(), 1);
    assert_eq!(s.ignored_words, ["set_clock_groups -bogus", "set_clock_groups c"]);
}

/// Options no SDC version declares, and the reference timer lacks too: refused, as before.
/// (`set_load` declares `-min -max -substract_pin_load -pin_load -wire_load`, and newer vendor SDC
/// 1.8 `-rise -fall -internal -allow_negative_cap`; `set_max_fanout` declares no option.)
#[test]
fn options_neither_declares_are_refused() {
    for line in ["set_load -total 0.1 [get_ports y]", "set_max_fanout -force 8 [current_design]", "set_max_transition -min 0.5 [current_design]"] {
        assert!(!read(line).rejected.is_empty(), "{line}");
    }
}

/// Newer releases of the vendor's SDC parser declare more options in SDC 1.8 than its public
/// release: on `set_max_capacitance` `-clock_path -data_path -rise -fall -force -sms_scenarios`, on
/// `set_max_transition` `-delay_penalty -force`. They are SDC 1.8 to a user of those releases, so
/// they are read, not refused: the limit is kept as written (no repair here applies a limit
/// scoped to clock or data paths, an edge or a scenario), written back, and reported.
#[test]
fn newer_sdc_18_design_rule_options_are_kept() {
    for (line, opts, back) in [
        ("set_max_capacitance -clock_path 0.1 [get_clocks c]", &["-clock_path"][..], "set_max_capacitance -clock_path 0.1 [get_clocks {c}]"),
        ("set_max_capacitance -data_path -rise -fall 0.1 [current_design]", &["-data_path", "-rise", "-fall"][..], "set_max_capacitance -data_path -rise -fall 0.1 [current_design]"),
        ("set_max_capacitance -force -sms_scenarios {s1 s2} 0.1 [current_design]", &["-force", "-sms_scenarios"][..], "set_max_capacitance -sms_scenarios {s1 s2} -force 0.1 [current_design]"),
        ("set_max_transition -force 0.5 [current_design]", &["-force"][..], "set_max_transition -force 0.5 [current_design]"),
        // `-delay_penalty` is a flag in vendor SDC 1.8 (it takes no value): 0.5 is the limit
        ("set_max_transition -delay_penalty 0.5 [current_design]", &["-delay_penalty"][..], "set_max_transition -delay_penalty 0.5 [current_design]"),
    ] {
        let cmd = line.split(' ').next().unwrap();
        let want: Vec<String> = opts.iter().map(|o| format!("{cmd} {o}")).collect();
        let want: Vec<&str> = want.iter().map(String::as_str).collect();
        let s = accepted(line, &want);
        assert_eq!((s.max_capacitance, s.max_transition), (None, None), "{line}: applied as a design-wide limit");
        assert!(s.max_capacitance_objects.is_empty() && s.max_transition_objects.is_empty(), "{line}: applied");
        assert!(written(&s).contains(back), "{line}: {}", written(&s));
    }
    // vendor SDC 1.8 names the scenario list `-sms_scenarios` (a list); no release declares a
    // `-scenarios`, so it is an option no SDC version has: refused, not kept
    let s = read("set_max_capacitance -scenarios {s1 s2} 0.1 [current_design]");
    assert!(s.rejected.iter().any(|r| r.contains("-scenarios")), "{:?}", s.rejected);
    // and kept with that limit, not a design-wide one
    let s = read("set_max_transition -delay_penalty 0.5 [current_design]");
    let k = s.kept.iter().find(|k| k.cmd == "set_max_transition").expect("kept");
    assert_eq!((k.number(0), k.has("-delay_penalty"), k.keys.is_empty()), (Some(0.5), true, true), "{k:?}");
    // a value after it is a second positional: `wrong # args`, not a penalty
    assert!(!read("set_max_transition -delay_penalty 2 0.5 [current_design]").rejected.is_empty());
    // still a limit: its value and objects are checked
    let s = read("set_max_capacitance -clock_path 0.1");
    assert!(s.rejected.iter().any(|r| r.contains("missing objects")), "{:?}", s.rejected);
    assert!(s.kept.is_empty(), "{:?}", s.kept);
}

/// The reference timer's options and commands that no SDC version declares are read as it reads
/// them, and `check` names each one used, with its count, at info level only. An option SDC does
/// declare (`-comment`), or newer vendor SDC 1.8 does (`get_ports -filter`, `get_ports -quiet`,
/// `get_pins -of_objects`, and `-filter` on the library queries), is not named.
#[test]
fn reference_extensions_are_named_at_info_level() {
    let s = read("set_input_delay 1 -clock c [all_inputs -no_clocks]\nset_clock_uncertainty 0.1 [get_clocks -filter {name==c} *]\n\
                  set_clock_uncertainty 0.1 [get_clocks -filter {name==c} *]\nset_false_path -to [get_ports y] -comment x\nset_pvt [get_cells u1] -voltage 1.0");
    assert_eq!(s.extensions, ["all_inputs -no_clocks", "get_clocks -filter", "get_clocks -filter", "set_pvt"]);
    let r = vyges_loom::sdc_lint::lint_without_design(&s);
    let f: Vec<_> = r.findings.iter().filter(|f| f.code == "sdc-extension").collect();
    assert_eq!(f.len(), 3, "{:?}", r.findings);
    assert!(f.iter().all(|f| f.severity == vyges_loom::sdc_lint::Severity::Info), "{f:?}");
    assert!(f[1].message.contains("`get_clocks -filter` (2 uses)"), "{}", f[1].message);
    assert!(read("create_clock -name d -period 5 -comment x [get_ports d]").extensions.is_empty());
    let s = read("set_input_delay 1 -clock c [get_ports -filter {direction==in} -quiet *]\nset_false_path -to [get_pins -quiet -of_objects [get_cells u1]]\nset_load -rise 0.1 [get_ports y]");
    assert!(s.extensions.is_empty(), "newer vendor SDC 1.8 options named as extensions: {:?}", s.extensions);
    let s = read("set_disable_timing [get_lib_cells -filter {name==BUF} lib/*]\nset_disable_timing [get_lib_pins -filter {name==A} lib/BUF/*]\n\
                  set_driving_cell -lib_cell BUF -library [get_libs -filter {name==lib} *] [get_ports a]");
    assert!(s.extensions.is_empty(), "vendor SDC 1.8's library-query -filter named as an extension: {:?}", s.extensions);
}

/// SDC 1.7 on declares `set_false_path -rise_throough` / `-fall_throough`, the public parser's
/// spelling of `-rise_through` / `-fall_through`: read as them, applied, and written in the
/// usual spelling.
#[test]
fn the_parsers_throough_spelling_is_through() {
    for (o, as_) in [("-rise_throough", "-rise_through"), ("-fall_throough", "-fall_through")] {
        let s = accepted(&format!("set_false_path {o} [get_pins u1/A] -to [get_ports y]"), &[]);
        assert_eq!(s.exceptions.len(), 1, "{o}");
        let text = written(&s);
        assert!(text.contains(&format!("{as_} [get_pins {{u1/A}}]")), "{o}: {text}");
        // and the coverage table lists it as a standard option the reference timer lacks
        assert!(vyges_loom::sdc::registry::standard_options().iter().any(|(c, x, _)| *c == "set_false_path" && *x == o), "{o}");
    }
}
