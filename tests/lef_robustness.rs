//! Schema-driven robustness for the LEF reader.
//!
//! LEF is a published standard with a formal grammar, so the set of constructs a conforming
//! file may contain is **enumerable** rather than a matter of memory. `tests/data/lef_keywords.txt`
//! holds all 447 terminals of that grammar (see its header for provenance).
//!
//! We read a deliberately small slice of LEF — per-layer width, thickness, R, C and current
//! density, plus macro pin directions. Everything else must be *ignored*, and this asserts that
//! literally: for every keyword in the standard, inject a line using it into a known-good file
//! and require that nothing we read moves.
//!
//! **Why this shape.** The failure mode this format keeps producing is not a crash, it is a
//! construct we half-recognise. A `SPACINGTABLE` inside a `LAYER` block carries `WIDTH 3 0.28`
//! rows; matched as the layer's default width, last-write-wins, met1 reported a routing width
//! of **3 µm** instead of 0.14. Nothing downstream failed — coupling simply computed every
//! edge-to-edge gap as negative and clamped every neighbour to full coefficient. One keyword,
//! silently, in the middle of a correlation study. This test exists so the next one is loud.

use std::collections::BTreeMap;

use vyges_loom::lef::Lef;

/// The keywords of the LEF grammar, minus comments.
fn keywords() -> Vec<&'static str> {
    include_str!("data/lef_keywords.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect()
}

/// The subset we deliberately act on. Injecting one of these *should* change the result, so
/// they are asserted individually elsewhere rather than swept here.
const HANDLED: &[&str] = &[
    "LAYER",
    "END",
    "MACRO",
    "PIN",
    "DIRECTION",
    "VIA",
    "VIARULE",
    "WIDTH",
    "THICKNESS",
    "TYPE",
    "RESISTANCE",
    "CAPACITANCE",
    "EDGECAPACITANCE",
    "DCCURRENTDENSITY",
    "ACCURRENTDENSITY",
    "ROUTING",
    "RPERSQ",
    "CPERSQDIST",
    "AVERAGE",
    "RMS",
    "PEAK",
    "INPUT",
    "OUTPUT",
    "INOUT",
];

/// A tech LEF with one routing layer, one cut layer and one macro — everything this reader
/// claims to understand, and nothing it does not.
fn good_lef(extra_in_layer: &str, extra_at_top: &str) -> String {
    format!(
        "VERSION 5.8 ;\n\
         BUSBITCHARS \"[]\" ;\n\
         DIVIDERCHAR \"/\" ;\n\
         UNITS\n  DATABASE MICRONS 1000 ;\n  CAPACITANCE PICOFARADS 1 ;\n  RESISTANCE OHMS 1 ;\nEND UNITS\n\
         {extra_at_top}\
         LAYER met1\n\
         \x20 TYPE ROUTING ;\n\
         \x20 WIDTH 0.14 ;\n\
         \x20 THICKNESS 0.35 ;\n\
         \x20 RESISTANCE RPERSQ 0.125 ;\n\
         \x20 CAPACITANCE CPERSQDIST 0.000025 ;\n\
         \x20 EDGECAPACITANCE 0.0000406 ;\n\
         \x20 DCCURRENTDENSITY AVERAGE 1.7 ;\n\
         \x20 ACCURRENTDENSITY RMS 2.3 ;\n\
         {extra_in_layer}\
         END met1\n\
         LAYER via1\n  TYPE CUT ;\n  RESISTANCE 4.5 ;\nEND via1\n\
         MACRO INV_X1\n  PIN A\n    DIRECTION INPUT ;\n  END A\n  PIN Y\n    DIRECTION OUTPUT ;\n  END Y\nEND INV_X1\n"
    )
}

/// Everything this reader is supposed to extract from `good_lef`, as one comparable value.
fn fingerprint(l: &Lef) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    for (n, x) in &l.layers {
        m.insert(
            format!("layer:{n}"),
            format!(
                "routing={} w={:.6} t={:.6} r={:.6} c={:.9} e={:.9} cut={:.6} dc={:.6} rms={:.6} pk={:.6}",
                x.routing,
                x.width_um,
                x.thickness_um,
                x.rpersq,
                x.cpersqdist,
                x.edge_cap,
                x.cut_res,
                x.dc_jmax,
                x.ac_rms,
                x.ac_peak
            ),
        );
    }
    for (n, mac) in &l.macros {
        for (p, pin) in &mac.pins {
            m.insert(format!("pin:{n}:{p}"), format!("{:?}", pin.direction));
        }
    }
    m
}

fn parse(text: &str) -> Lef {
    Lef::parse(text).expect("the base file is valid LEF")
}

/// The property under test is that an unhandled construct cannot *silently* change what we
/// read. A hard parse error is not a violation — it is loud, which is the whole point. So an
/// injection is a failure only when the reader returns success with different values.
fn perturbs(text: &str, base: &BTreeMap<String, String>) -> bool {
    match Lef::parse(text) {
        Err(_) => false,
        Ok(l) => l.health().is_none() && fingerprint(&l) != *base,
    }
}

/// The baseline itself: if this is wrong every other case is meaningless.
#[test]
fn the_known_good_file_reads_as_expected() {
    let l = parse(&good_lef("", ""));
    let met1 = l.layers.get("met1").expect("met1");
    assert!(met1.routing);
    assert_eq!(met1.width_um, 0.14);
    assert_eq!(met1.thickness_um, 0.35);
    assert_eq!(met1.rpersq, 0.125);
    assert_eq!(met1.cpersqdist, 0.000025);
    assert_eq!(met1.edge_cap, 0.0000406);
    assert_eq!(met1.dc_jmax, 1.7);
    assert_eq!(met1.ac_rms, 2.3);
    let via1 = l.layers.get("via1").expect("via1");
    assert!(!via1.routing);
    assert_eq!(via1.cut_res, 4.5);
    assert_eq!(l.widths.get("met1"), Some(&0.14));
    assert_eq!(l.thicknesses.get("met1"), Some(&0.35));
    let inv = l.macros.get("INV_X1").expect("macro");
    assert_eq!(inv.pins.len(), 2);
}

/// **The generated sweep.** Every keyword of the standard, injected inside the `LAYER` block.
/// None of them may move a value we read.
#[test]
fn no_unhandled_keyword_inside_a_layer_can_perturb_what_we_read() {
    let base = fingerprint(&parse(&good_lef("", "")));
    let mut broke: Vec<String> = Vec::new();
    for kw in keywords() {
        if HANDLED.contains(&kw) {
            continue;
        }
        // Several plausible shapes: a bare statement, one numeric argument, two, and a quoted
        // string — a reader that keys on a token's *position* rather than its identity trips
        // on one of these even when it survives the others.
        for body in [
            format!("  {kw} ;\n"),
            format!("  {kw} 3 ;\n"),
            format!("  {kw} 3 0.28 ;\n"),
            format!("  {kw} \"TYPE NWELL ;\" ;\n"),
        ] {
            if perturbs(&good_lef(&body, ""), &base) {
                broke.push(format!("{kw}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} keyword(s) perturbed a value we read:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// The same sweep at file scope. A keyword outside any block must not invent a layer, nor
/// leave the reader in a state where the next real block is misread.
#[test]
fn no_unhandled_keyword_at_top_level_can_perturb_what_we_read() {
    let base = fingerprint(&parse(&good_lef("", "")));
    let mut broke: Vec<String> = Vec::new();
    for kw in keywords() {
        if HANDLED.contains(&kw) {
            continue;
        }
        for body in [format!("{kw} ;\n"), format!("{kw} foo 3 ;\n")] {
            if perturbs(&good_lef("", &body), &base) {
                broke.push(format!("{kw}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} keyword(s) perturbed a value we read:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// A block we do not model must not leak its contents into the layer table. `PROPERTYDEFINITIONS`
/// legally contains `LAYER <property-name> STRING ;` — a line that reads exactly like the start
/// of a tech layer, and does in sky130's own tech LEF.
#[test]
fn a_layer_line_inside_propertydefinitions_invents_no_layer() {
    let l = parse(&good_lef(
        "",
        "PROPERTYDEFINITIONS\n  LAYER LEF58_TYPE STRING ;\n  LAYER RESISTANCE REAL ;\nEND PROPERTYDEFINITIONS\n",
    ));
    assert!(
        !l.layers.contains_key("LEF58_TYPE") && !l.layers.contains_key("RESISTANCE"),
        "property names became layers: {:?}",
        l.layers.keys().collect::<Vec<_>>()
    );
    // and the real layers are untouched
    assert_eq!(l.layers.get("met1").map(|x| x.width_um), Some(0.14));
    assert_eq!(l.layers.len(), 2, "{:?}", l.layers.keys().collect::<Vec<_>>());
}

/// Every attribute we act on, with a wrong value — the body to bury inside someone else's
/// construct. NOTE the absence of `;`: a LEF statement ends at the semicolon, so these tokens
/// belong to whatever construct opened before them. `WIDTH 3 0.28` is the real `SPACINGTABLE`
/// row that once became met1's routing width.
const POISON: &str = "    PARALLELRUNLENGTH 0 0.5\n\
                      \x20   WIDTH 3 0.28\n\
                      \x20   THICKNESS 9.9\n\
                      \x20   RESISTANCE RPERSQ 99\n\
                      \x20   CAPACITANCE CPERSQDIST 99\n\
                      \x20   EDGECAPACITANCE 99\n\
                      \x20   DCCURRENTDENSITY AVERAGE 99\n\
                      \x20   ACCURRENTDENSITY RMS 99\n";

/// **The sweep that matters.** The failure this format produces is not an unknown keyword — it
/// is a keyword we *do* handle, appearing inside a construct we do not, in a context where it
/// means something else entirely. That is exactly the `SPACINGTABLE`/`WIDTH` bug, and a sweep
/// of lone unknown keywords cannot see it.
///
/// So: for each keyword of the standard, open a sub-block with it inside `LAYER met1`, fill it
/// with every attribute we read set to a wrong value, and require that none of them lands.
#[test]
fn a_handled_keyword_buried_in_an_unhandled_block_must_not_land() {
    let base = fingerprint(&parse(&good_lef("", "")));
    let mut broke: Vec<String> = Vec::new();
    for kw in keywords() {
        if HANDLED.contains(&kw) {
            continue;
        }
        for body in [
            // one multi-line statement, closed by a single `;` at the very end
            format!("  {kw}\n{POISON}  ;\n"),
            format!("  {kw} 0 0.3\n{POISON}  ;\n"),
            // and the same buried in a quoted property string, whose semicolons end nothing
            format!("  PROPERTY {kw} \"\n{POISON}  \" ;\n"),
        ] {
            if perturbs(&good_lef(&body, ""), &base) {
                broke.push(kw.to_string());
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} construct(s) let a wrong value through:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}
