//! A real constraint file, read whole: CoralNPU's `rvv_core_mini` ASAP7 constraints
//! (`tests/data/sdc/coralnpu/`, Apache-2.0, unmodified; see the README there). This pins what the
//! reader makes of it without a netlist, as structured results:
//! - it reads to the end (no command the reference does not know);
//! - each command it uses is applied, kept or run as Tcl, as the command registry says;
//! - the two `if` blocks whose condition is the length of an object query (`llength [get_ports
//!   -quiet io_te]`) need the design, so without one they are listed as not applied;
//! - `group_path` (×3) is carried in its records for the `sta` timer, which applies it, and is
//!   not reported as kept-not-applied;
//! - its reference-timer extensions are `-quiet` on the queries and `all_inputs -no_clocks`.
//!
//! The file takes its period from `$::env(CLK_PERIOD_PS)` when that is set, as Tcl does, else
//! 5000: the expected period follows the same rule, so the test holds either way.

use vyges_loom::sdc::registry::{command, Status};
use vyges_loom::sdc::Sdc;

const FILE: &str = "tests/data/sdc/coralnpu/rvv_core_mini-asap7.sdc";

fn period() -> f64 {
    std::env::var("CLK_PERIOD_PS").ok().and_then(|v| v.trim().parse().ok()).unwrap_or(5000.0)
}

fn read() -> Sdc {
    Sdc::load(FILE).unwrap_or_else(|e| panic!("{FILE}: {}", e.0))
}

#[test]
fn reads_to_the_end_with_its_clocks_and_delays() {
    let s = read();
    assert!(s.rejected.is_empty(), "{:?}", s.rejected);
    let clocks: Vec<(&str, f64)> = s.clocks.iter().map(|c| (c.name.as_str(), c.period)).collect();
    assert_eq!(clocks, [("io_aclk", period()), ("vclk_io_aclk", period())]);
    assert_eq!(s.current_design.as_deref(), Some("RvvCoreMiniAxi"));
    // -max and -min on every input and on every output
    assert_eq!((s.input_delays.len(), s.output_delays.len()), (2, 2));
    // the six set_false_path outside an `if`
    assert_eq!(s.exceptions.len(), 6);
    assert_eq!(s.uncertainties.len(), 2);
}

/// Every command the file uses, as the registry has it: Applied and Tcl are neither listed as
/// not applied nor reported kept; Kept is reported. `if` is the one exception, and only for its
/// two query conditions.
#[test]
fn each_command_is_what_the_registry_says() {
    let s = read();
    let used = [
        "all_clocks", "all_inputs", "all_outputs", "create_clock", "current_design", "expr", "get_clocks", "get_pins",
        "get_ports", "group_path", "if", "set", "set_case_analysis", "set_clock_latency", "set_clock_transition",
        "set_clock_uncertainty", "set_false_path", "set_input_delay", "set_input_transition", "set_load", "set_output_delay",
    ];
    let not_applied: Vec<String> = s.not_applied().into_iter().map(|n| n.command).collect();
    let kept: Vec<String> = s.kept_not_applied().into_iter().map(|k| k.command).collect();
    for c in used {
        let st = command(c).unwrap_or_else(|| panic!("{c}: not in the registry")).status;
        match st {
            Status::Kept => assert!(kept.iter().any(|k| k == c), "{c}: Kept, not reported: {kept:?}"),
            Status::Applied | Status::Tcl => {
                assert!(!kept.iter().any(|k| k == c), "{c}: {st:?}, yet reported kept");
                assert!(c == "if" || !not_applied.iter().any(|n| n == c), "{c}: {st:?}, yet not applied: {not_applied:?}");
            }
            other => panic!("{c}: {other:?}"),
        }
    }
}

/// The two `if {[llength <query>] > 0}` blocks: the query's length needs the design. Without a
/// netlist each is listed as not applied (an error in `check`, since its body constrains timing),
/// and nothing in their bodies is applied: no case value, no extra false path.
#[test]
fn query_conditions_are_not_evaluated_without_a_design() {
    let s = read();
    assert_eq!(s.ignored.iter().filter(|c| *c == "if").count(), 2, "{:?}", s.ignored);
    let na: Vec<(String, bool)> = s.not_applied().into_iter().map(|n| (n.command, n.affects_timing)).collect();
    assert_eq!(na, [("if".to_string(), true)]);
    assert!(s.case_values.is_empty(), "{:?}", s.case_values.len());
    let r = vyges_loom::sdc_lint::lint_without_design(&s);
    let errors: Vec<(&str, &str)> = r.findings.iter().filter(|f| f.severity == vyges_loom::sdc_lint::Severity::Error).map(|f| (f.code, f.message.as_str())).collect();
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert_eq!(errors[0].0, "sdc-not-applied");
    assert!(errors[0].1.starts_with("`if` (2 commands)"), "{}", errors[0].1);
}

#[test]
fn group_path_is_carried_for_the_timer() {
    let s = read();
    assert!(s.kept_not_applied().is_empty(), "{:?}", s.kept_not_applied());
    assert_eq!(s.other_exceptions.iter().filter(|e| e.cmd == "group_path").count(), 3);
}

#[test]
fn its_extension_is_no_clocks() {
    let s = read();
    let mut seen: Vec<(&str, usize)> = Vec::new();
    for e in &s.extensions {
        match seen.iter_mut().find(|(n, _)| n == e) {
            Some((_, c)) => *c += 1,
            None => seen.push((e, 1)),
        }
    }
    // its `-quiet` queries are newer vendor SDC 1.8, not extensions
    assert_eq!(seen, [("all_inputs -no_clocks", 6)]);
}
