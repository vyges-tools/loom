//! A synthesis-flow-style constraint set read whole (field item: commercial-style fixtures):
//! `source` of a setup script and an I/O file, a `proc` with a default, collection commands (a set-wise
//! `foreach_in_collection`), singular queries, design rules on objects, a clock sense and disabled arcs. Every command is
//! either modelled or reported as not applied, and the read reaches the end of the file.

use vyges_loom::sdc::Sdc;

#[test]
fn a_synthesis_style_constraint_set_is_read_to_the_end() {
    let s = Sdc::load("tests/data/sdc/synth_style/top.sdc").expect("read");
    assert_eq!(s.clocks.len(), 1);
    assert!((s.clocks[0].period - 1.8).abs() < 1e-9, "{}", s.clocks[0].period);
    assert_eq!(s.clocks[0].sources, ["clk"]);
    let ins: Vec<(f64, Vec<String>)> = s.input_delays.iter().map(|d| (d.value, d.ports.clone())).collect();
    assert_eq!(ins, [(0.4, vec!["din*".to_string(), "en".to_string()])]);
    assert_eq!(s.output_delays.len(), 1);
    assert_eq!(s.output_delays[0].clock.as_deref(), Some("core_clk"));
    assert_eq!(s.max_transition_objects.len(), 1);
    assert_eq!((s.senses.len(), s.disabled_timing.len(), s.disabled_gating_checks.len()), (1, 1, 1));
    assert_eq!(s.exceptions.len(), 1);
    // the last line was applied: the read reached the end of the file
    assert!(s.env.iter().any(|e| e.objects == ["dout*"]), "{:?}", s.env);
    // the per-object loop hands `$p` to set_load as its object: set_load on the whole collection
    assert_eq!(s.env.len(), 2, "{:?}", s.env);
    // what was not applied is said: the design query (not timing)
    let na = s.not_applied();
    let names: Vec<(&str, bool)> = na.iter().map(|n| (n.command.as_str(), n.affects_timing)).collect();
    assert!(!names.iter().any(|n| n.0 == "foreach_in_collection"), "{names:?}");
    assert!(names.contains(&("set_dont_touch", false)), "{names:?}");
    assert!(s.unevaluated_queries.is_empty(), "{:?}", s.unevaluated_queries);
}
