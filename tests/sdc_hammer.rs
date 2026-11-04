//! A second, independent SDC source: constraints in the shape hammer (ucb-bar, BSD-3) generates
//! from its YAML settings. Pinned: every line is modelled (none refused or unmodelled), the
//! objects come out as hammer means them (a bare clock path is the port of that name; a delay's
//! `-add_delay` after the object still adds), and `write_sdc` is complete and a fixed point.

use vyges_loom::sdc::Sdc;
use vyges_loom::sdc_lint::{lint_without_design, Severity};
use vyges_loom::sdc_write::WriteOpts;

fn load() -> Sdc {
    Sdc::load("tests/data/sdc/hammer/emitted.sdc").expect("read")
}

#[test]
fn every_hammer_line_is_modelled() {
    let s = load();
    assert!(s.not_applied().is_empty(), "{:?}", s.not_applied().iter().map(|n| &n.command).collect::<Vec<_>>());
    let names: Vec<&str> = s.clocks.iter().map(|c| c.name.as_str()).collect();
    assert_eq!(names, ["clock", "clock_div", "clock_inv", "jtag_tck"]);
    assert_eq!(s.clocks[0].sources, ["clock"]);
    // -min and -max on one port with -add_delay: two delays, not one replacing the other
    assert_eq!(s.input_delays.len(), 2);
    assert_eq!(s.output_delays.len(), 1);
}

#[test]
fn hammer_constraints_lint_clean_without_a_design() {
    let r = lint_without_design(&load());
    let errs: Vec<_> = r.findings.iter().filter(|f| matches!(f.severity, Severity::Error)).collect();
    assert!(errs.is_empty(), "{errs:?}");
}

#[test]
fn hammer_constraints_write_back_as_a_fixed_point() {
    let opts = WriteOpts { design: None, header: None };
    let first = load().write(&opts);
    assert!(first.not_written.is_empty(), "{:?}", first.not_written);
    let second = Sdc::parse(&first.text).expect("reads back").write(&opts);
    assert_eq!(first.text, second.text);
}
