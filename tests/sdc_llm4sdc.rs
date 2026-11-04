//! The command floor of a generated-SDC flow (LLM4SDC, DAC 2026: its text says 17 commands, its
//! Table 1 names 16, and those 16 are what is pinned here) read, linted and written back. The rule pinned here: none of the 17 is refused or left
//! unmodelled, design-free lint raises no error, and `write_sdc` output reads back to the same
//! model and writes byte-identically a second time (the writer is a fixed point).

use vyges_loom::sdc::Sdc;
use vyges_loom::sdc_lint::{lint_without_design, Severity};
use vyges_loom::sdc_write::WriteOpts;

const FLOOR: [&str; 16] = [
    "set_input_transition", "set_load", "set_max_capacitance", "set_max_fanout",
    "set_max_transition", "set_min_capacitance", "create_clock", "create_generated_clock",
    "set_clock_groups", "set_clock_latency", "set_clock_transition", "set_clock_uncertainty",
    "set_input_delay", "set_output_delay", "set_multicycle_path", "set_false_path",
];

fn load() -> Sdc {
    Sdc::load("tests/data/sdc/llm4sdc/floor.sdc").expect("read")
}

#[test]
fn every_command_of_the_floor_is_in_the_fixture() {
    let text = std::fs::read_to_string("tests/data/sdc/llm4sdc/floor.sdc").unwrap();
    for c in FLOOR {
        assert!(text.lines().any(|l| l.starts_with(&format!("{c} "))), "fixture lacks {c}");
    }
}

#[test]
fn none_of_the_floor_is_refused_or_unmodelled() {
    let s = load();
    let na = s.not_applied();
    let hit: Vec<&str> = na.iter().map(|n| n.command.as_str()).filter(|c| FLOOR.contains(c)).collect();
    assert!(hit.is_empty(), "not applied: {hit:?}");
    assert!(s.unevaluated_queries.is_empty(), "{:?}", s.unevaluated_queries);
    assert_eq!(s.clocks.len(), 3, "{:?}", s.clocks.iter().map(|c| &c.name).collect::<Vec<_>>());
}

#[test]
fn design_free_lint_raises_no_error() {
    let r = lint_without_design(&load());
    let errs: Vec<_> = r.findings.iter().filter(|f| matches!(f.severity, Severity::Error)).collect();
    assert!(errs.is_empty(), "{errs:?}");
}

#[test]
fn write_sdc_is_complete_and_a_fixed_point() {
    let opts = WriteOpts { design: None, header: None };
    let first = load().write(&opts);
    assert!(first.not_written.is_empty(), "{:?}", first.not_written);
    let again = Sdc::parse(&first.text).expect("written SDC reads back");
    assert!(again.not_applied().iter().all(|n| !FLOOR.contains(&n.command.as_str())));
    let second = again.write(&opts);
    assert_eq!(first.text, second.text, "write -> read -> write must be identical");
}
