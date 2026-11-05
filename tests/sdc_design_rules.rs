//! The design-rule limits are checked as SDC declares them: the public SDC parser gives
//! `set_max_capacitance` and `set_max_fanout` `value object_list` alone, and `set_max_transition`
//! `[-clock_path] [-rise] [-fall] value object_list` (the reference timer adds `-data_path`; newer
//! vendor SDC 1.8 adds options to `set_max_capacitance` and `set_max_transition`, read in
//! `sdc_standard_options.rs`). Each REQUIRES both and a value that is a positive float (`$par>=0`). A missing value or object,
//! a misplaced option, a negative or non-numeric value is refused with a message — never read as
//! a limit with no value, never dropped without a word.

use vyges_loom::sdc::Sdc;

fn read(line: &str) -> Sdc {
    Sdc::parse(&format!("{line}\n")).unwrap_or_else(|e| panic!("{line}: {e:?}"))
}

/// Refused, with `why` in the message, and nothing of it kept as a limit.
fn refused(line: &str, why: &str) {
    let s = read(line);
    assert_eq!(s.rejected.len(), 1, "{line}: {:?}", s.rejected);
    assert!(s.rejected[0].contains(why), "{line}: {:?}", s.rejected);
    let limits = s.max_capacitance.is_some() as usize
        + s.max_fanout.is_some() as usize
        + s.max_transition.is_some() as usize
        + s.max_capacitance_objects.len()
        + s.max_fanout_objects.len()
        + s.max_transition_objects.len();
    assert_eq!(limits, 0, "{line}: a limit was kept");
    // and `check` says so, as an error: the limit went unapplied
    let r = vyges_loom::sdc_lint::lint_without_design(&s);
    assert!(r.errors() > 0, "{line}: {:?}", r.findings);
}

#[test]
fn a_limit_without_its_value_or_objects_is_refused_with_a_message() {
    for cmd in ["set_max_capacitance", "set_max_fanout", "set_max_transition"] {
        refused(&format!("{cmd} 0.2"), "missing objects");
        refused(cmd, "missing value");
        refused(&format!("{cmd} -0.2 [get_ports a]"), "not a positive float");
        refused(&format!("{cmd} big [get_ports a]"), "not a positive float");
        refused(&format!("{cmd} 0.2 [get_ports a] [get_ports b]"), "wrong # args");
    }
}

#[test]
fn an_option_the_limit_does_not_have_is_refused() {
    // a misplaced option used to read `0.2` as the objects of a limit with no value
    refused("set_max_capacitance -min 0.2 [get_ports a]", "-min is not an option of set_max_capacitance");
    refused("set_max_fanout -max 8 [current_design]", "-max is not an option of set_max_fanout");
    refused("set_max_transition -min 0.2 [current_design]", "-min is not an option of set_max_transition");
    // `set_max_fanout` has no option in any SDC (newer vendor SDC 1.8 gives `set_max_capacitance`
    // `-clock_path`: `sdc_standard_options.rs`)
    refused("set_max_fanout -clock_path 8 [get_clocks c]", "-clock_path is not an option of set_max_fanout");
}

#[test]
fn well_formed_limits_are_read() {
    let s = read("set_max_capacitance 0.2 [current_design]\nset_max_capacitance 0.1 [get_ports a]\nset_max_fanout 8 [current_design]\n\
                  set_max_transition -clock_path 0.3 [get_clocks c]\nset_max_transition 0.5 [current_design]");
    assert!(s.rejected.is_empty(), "{:?}", s.rejected);
    assert_eq!((s.max_capacitance, s.max_fanout, s.max_transition), (Some(0.2), Some(8.0), Some(0.5)));
    assert_eq!((s.max_capacitance_objects.len(), s.max_transition_objects.len()), (1, 1));
}
