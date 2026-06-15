//! End-to-end: the seeded parsers load a full design into the shared DB.
//!
//! Proves the parse-once / query-many foundation works against real standard
//! files (the `examples/top` set: structural Verilog + Liberty + SDC + SPEF).

use vyges_loom::Design;

fn ex(name: &str) -> String {
    format!("{}/examples/top/{name}", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn loads_full_design_from_standard_files() {
    let mut d = Design::new();
    d.load(&ex("top.v")).unwrap();
    d.load(&ex("cells.lib")).unwrap();
    d.load(&ex("top.sdc")).unwrap();
    d.load(&ex("top.spef")).unwrap();

    let nl = d.netlist.as_ref().expect("netlist loaded");
    assert_eq!(nl.module, "top");
    assert!(!nl.insts.is_empty(), "netlist has instances");
    assert!(d.lib_cell_count() >= 1, "liberty has cells");
    assert!(!d.sdc.as_ref().unwrap().clocks.is_empty(), "sdc has a clock");
    assert!(!d.spef.as_ref().unwrap().nets.is_empty(), "spef has nets");

    // cross-step state recorded every load, in order.
    assert_eq!(d.steps.len(), 4);
    assert_eq!(d.steps[0].kind, "netlist");

    // both output contracts hold.
    assert!(d.to_json().contains("\"module\":\"top\""));
    assert!(d.summary().contains("netlist"));
}

#[test]
fn load_dispatches_by_extension() {
    let mut d = Design::new();
    assert_eq!(d.load(&ex("top.v")).unwrap(), "netlist");
    assert_eq!(d.load(&ex("cells.lib")).unwrap(), "liberty");
    assert_eq!(d.load(&ex("top.sdc")).unwrap(), "sdc");
    assert_eq!(d.load(&ex("top.spef")).unwrap(), "spef");
}
