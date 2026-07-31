// SPDX-License-Identifier: Apache-2.0
//! Cell equivalence classes — which cells may stand in for one another.
//!
//! This is what a resize move needs, and the last thing standing between the timing-repair loop
//! and setup repair: OpenDB refuses a swap whose *pins* do not match, but nothing downstream
//! checks *function*, so a planner choosing a replacement has to know which cells actually
//! compute the same thing.
//!
//! The fixture deliberately mixes the cases a real library mixes: a vendor-declared footprint
//! group, a group that can only be matched structurally, a same-shape cell with a *different*
//! function (the confusion a structural match must avoid), and a timing-only cell where
//! equivalence is simply not knowable.
use vyges_loom::liberty::Lib;

const LIB: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/equiv.lib");

fn lib() -> Lib {
    Lib::load(LIB).unwrap()
}

fn names(cells: Vec<&vyges_loom::liberty::Cell>) -> Vec<String> {
    cells.into_iter().map(|c| c.name.clone()).collect()
}

#[test]
fn function_and_footprint_and_area_are_parsed() {
    let l = lib();
    let inv = l.cells.get("INV_X2").unwrap();
    assert_eq!(inv.cell_footprint.as_deref(), Some("inv"));
    assert_eq!(inv.area, 2.0);
    assert_eq!(inv.pins["Y"].function.as_deref(), Some("!A"), "function lives on the OUTPUT pin");
    assert_eq!(inv.pins["A"].function, None, "inputs have no function");

    // absence is not an error — a timing-only library is perfectly valid
    let nope = l.cells.get("NOPE").unwrap();
    assert_eq!(nope.cell_footprint, None);
    assert_eq!(nope.pins["Y"].function, None);
}

#[test]
fn a_footprint_group_is_a_drive_ladder_ranked_by_area() {
    let l = lib();
    assert_eq!(names(l.equivalence_class("INV_X2")), ["INV_X1", "INV_X2", "INV_X4"]);
    // the class includes the cell itself, so a caller can see where it sits on the ladder
    assert_eq!(names(l.upsize_candidates("INV_X2")), ["INV_X4"]);
    assert_eq!(names(l.downsize_candidates("INV_X2")), ["INV_X1"]);
    // the ends of the ladder have nowhere further to go
    assert!(l.upsize_candidates("INV_X4").is_empty());
    assert!(l.downsize_candidates("INV_X1").is_empty());
}

#[test]
fn cells_without_a_footprint_are_matched_by_function() {
    // BUF_X1/X2 declare no footprint, so equivalence has to come from the function attribute.
    let l = lib();
    assert_eq!(names(l.equivalence_class("BUF_X1")), ["BUF_X1", "BUF_X2"]);
    assert_eq!(names(l.upsize_candidates("BUF_X1")), ["BUF_X2"]);
}

#[test]
fn a_different_function_is_never_interchangeable() {
    // The failure that would matter: INV and BUF have identical pin names and directions, and
    // differ only in what they compute. A swap between them passes OpenDB's pin check and
    // silently inverts the design.
    let l = lib();
    assert!(!names(l.equivalence_class("BUF_X1")).contains(&"INV_X1".to_string()));
    assert!(!names(l.equivalence_class("INV_X1")).contains(&"BUF_X1".to_string()));
}

#[test]
fn a_different_pin_count_is_never_interchangeable() {
    let l = lib();
    let class = names(l.equivalence_class("NAND2_X1"));
    assert_eq!(class, ["NAND2_X1"], "nothing else is a 2-input NAND");
}

#[test]
fn a_timing_only_cell_reports_no_equivalents_rather_than_guessing() {
    // NOPE has the same pins as INV_X1 and BUF_X1 but declares no function and no footprint.
    // Matching it to either would be a guess, and a wrong guess changes what the design does —
    // so "same pins, no function anywhere" must NOT count as evidence.
    let l = lib();
    assert_eq!(names(l.equivalence_class("NOPE")), ["NOPE"]);
    assert!(l.upsize_candidates("NOPE").is_empty());
}

#[test]
fn sequential_cells_are_matched_only_by_footprint() {
    // A flop's behaviour is in its `ff` group, not a pin function, so a structural match would
    // be guessing — and guessing wrong swaps a design's state element. With a footprint the
    // vendor has told us, so the ladder is allowed.
    let l = lib();
    assert_eq!(names(l.equivalence_class("DFF_X1")), ["DFF_X1", "DFF_X2"]);
    assert_eq!(names(l.upsize_candidates("DFF_X1")), ["DFF_X2"]);
    // and a flop never joins a combinational class
    assert!(!names(l.equivalence_class("INV_X1")).contains(&"DFF_X1".to_string()));
}

#[test]
fn an_unknown_cell_yields_an_empty_class() {
    let l = lib();
    assert!(l.equivalence_class("NO_SUCH_CELL").is_empty());
    assert!(l.upsize_candidates("NO_SUCH_CELL").is_empty());
}

#[test]
fn the_cache_round_trips_the_new_fields() {
    // The Liberty cache is a hand-rolled binary format. A field that is parsed but not cached
    // would vanish on the second load — equivalence would work once and then quietly stop.
    let l = lib();
    let bytes = vyges_loom::libcache::encode(&l);
    let back = vyges_loom::libcache::decode(&bytes).expect("cache should decode");

    let a = back.cells.get("INV_X2").unwrap();
    assert_eq!(a.cell_footprint.as_deref(), Some("inv"));
    assert_eq!(a.area, 2.0);
    assert_eq!(a.pins["Y"].function.as_deref(), Some("!A"));
    // and the query still works off a cached lib
    assert_eq!(names(back.equivalence_class("INV_X2")), ["INV_X1", "INV_X2", "INV_X4"]);
}
