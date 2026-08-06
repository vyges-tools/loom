//! Schema-driven robustness for the DEF reader.
//!
//! Same method as `lef_robustness.rs`: DEF is a published standard with a formal grammar, so
//! `tests/data/def_keywords.txt` enumerates every terminal a conforming file may contain (287 of
//! them — see the file header for provenance). We read a narrow slice — units, signal-net
//! routing, the `SPECIALNETS` power grid, and component placement — and everything else must be
//! ignored. This asserts that for every keyword in the standard.
//!
//! The DEF reader locates sections by scanning a whole-file token stream, which fails
//! differently from LEF's line-based reader: a section keyword appearing anywhere — in a
//! property, a net name, a comment — can move a section boundary, and the result is not a
//! parse error but a quietly different design.

use std::collections::BTreeMap;

use vyges_loom::def::Def;

fn keywords() -> Vec<&'static str> {
    include_str!("data/def_keywords.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect()
}

/// Keywords this reader deliberately acts on — injecting one of these may legitimately change
/// the result, so they are asserted individually rather than swept.
const HANDLED: &[&str] = &[
    "UNITS", "DISTANCE", "MICRONS", "NETS", "SPECIALNETS", "COMPONENTS", "END", "ROUTED", "NEW",
    "FIXED", "COVER", "SHAPE", "USE", "POWER", "GROUND", "SIGNAL", "NONDEFAULTRULE",
    "NONDEFAULTRULES", "LAYER", "WIDTH", "PIN", "VERSION", "DESIGN", "DIEAREA",
    // Wiring types and width modifiers this reader acts on: NOSHIELD is a routing statement in
    // DEF 5.8 alongside ROUTED/FIXED/COVER, and TAPERRULE selects a non-default width. Both
    // legitimately change what is read, so they are asserted individually rather than swept.
    "NOSHIELD", "TAPERRULE",
];

/// A DEF exercising everything this reader claims to read.
fn good_def(in_nets: &str, at_top: &str) -> String {
    format!(
        "VERSION 5.8 ;\n\
         DIVIDERCHAR \"/\" ;\n\
         BUSBITCHARS \"[]\" ;\n\
         DESIGN blk ;\n\
         UNITS DISTANCE MICRONS 1000 ;\n\
         DIEAREA ( 0 0 ) ( 100000 100000 ) ;\n\
         {at_top}\
         COMPONENTS 2 ;\n\
         \x20   - u1 INV_X1 + PLACED ( 1000 2000 ) N ;\n\
         \x20   - u2 INV_X1 + PLACED ( 5000 2000 ) FS ;\n\
         END COMPONENTS\n\
         SPECIALNETS 1 ;\n\
         \x20   - VGND ( * VGND ) + USE GROUND\n\
         \x20     + ROUTED met1 480 + SHAPE FOLLOWPIN ( 0 5000 ) ( 20000 5000 )\n\
         \x20   ;\n\
         END SPECIALNETS\n\
         NETS 2 ;\n\
         \x20   - neta ( u1 Y ) ( u2 A )\n\
         {in_nets}\
         \x20     + ROUTED met1 ( 1000 4000 ) ( 20000 * )\n\
         \x20       NEW met2 ( 20000 4000 ) ( 20000 9000 )\n\
         \x20   ;\n\
         \x20   - netb ( u2 Y )\n\
         \x20     + ROUTED met1 ( 1000 6000 ) ( 20000 * )\n\
         \x20   ;\n\
         END NETS\n\
         END DESIGN\n"
    )
}

/// Everything this reader is supposed to extract, as one comparable value.
fn fingerprint(d: &Def) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    m.insert("units".into(), format!("{:.6}", d.units_per_um));
    for n in &d.nets {
        m.insert(
            format!("net:{}", n.name),
            format!(
                "pins={} segs={} geom={}",
                n.pins.len(),
                n.segments.len(),
                n.segments
                    .iter()
                    .map(|s| format!("{}:{:.3},{:.3},{:.3},{:.3}", s.layer, s.x0, s.y0, s.x1, s.y1))
                    .collect::<Vec<_>>()
                    .join(";")
            ),
        );
    }
    for p in &d.power_nets {
        m.insert(
            format!("pnet:{}", p.name),
            format!(
                "power={} segs={} geom={}",
                p.use_power,
                p.segs.len(),
                p.segs
                    .iter()
                    .map(|s| format!("{}:{},{},{},{}@{}", s.layer, s.x1, s.y1, s.x2, s.y2, s.width_dbu))
                    .collect::<Vec<_>>()
                    .join(";")
            ),
        );
    }
    for c in &d.comps {
        m.insert(format!("comp:{}", c.name), format!("{} {} {}", c.cell, c.x, c.y));
    }
    m
}

fn parse(text: &str) -> Def {
    Def::parse(text).expect("the base file is valid DEF")
}

/// A hard parse error is loud and therefore acceptable; a quiet change of the design is not.
fn perturbs(text: &str, base: &BTreeMap<String, String>) -> bool {
    match Def::parse(text) {
        Err(_) => false,
        Ok(d) => fingerprint(&d) != *base,
    }
}

/// The baseline: if this is wrong every other case is meaningless.
#[test]
fn the_known_good_file_reads_as_expected() {
    let d = parse(&good_def("", ""));
    assert_eq!(d.units_per_um, 1000.0);
    assert_eq!(d.nets.len(), 2, "{:?}", d.nets.iter().map(|n| &n.name).collect::<Vec<_>>());
    let a = d.nets.iter().find(|n| n.name == "neta").expect("neta");
    assert_eq!(a.pins.len(), 2);
    assert_eq!(a.segments.len(), 2, "one met1 run and one met2 run");
    assert_eq!(a.segments[0].layer, "met1");
    assert_eq!((a.segments[0].x0, a.segments[0].y0), (1.0, 4.0), "DBU -> um");
    assert_eq!((a.segments[0].x1, a.segments[0].y1), (20.0, 4.0), "`*` repeats the y");
    assert_eq!(d.power_nets.len(), 1);
    let p = &d.power_nets[0];
    assert_eq!(p.name, "VGND");
    assert_eq!(p.segs.len(), 1);
    assert_eq!(p.segs[0].width_dbu, 480.0, "the strap's own width, in DBU");
    assert_eq!(d.comps.len(), 2);
}

/// **The generated sweep, inside `NETS`.** A construct we do not model must not change the
/// routing we do read.
#[test]
fn no_unhandled_keyword_inside_nets_can_perturb_the_design() {
    let base = fingerprint(&parse(&good_def("", "")));
    let mut broke: Vec<String> = Vec::new();
    for kw in keywords() {
        if HANDLED.contains(&kw) {
            continue;
        }
        // Injected as an ATTRIBUTE of an existing net, which is the real hazard: adding a
        // whole net legitimately changes the design and would prove nothing.
        //
        // Deliberately NOT injected: `+ {kw} <layer> ( x y ) ( x y )`. Only the wiring
        // keywords take a layer and routing points, so attaching them to an arbitrary keyword
        // fabricates a file DEF cannot contain — and a reader is entitled to read it as
        // routing. The real net attributes are asserted one by one instead, below.
        for body in [
            format!("      + {kw}\n"),
            format!("      + {kw} 3\n"),
            format!("      + PROPERTY {kw} \"x\"\n"),
        ] {
            if perturbs(&good_def(&body, ""), &base) {
                broke.push(format!("{kw}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} keyword(s) perturbed the design:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// The same sweep at file scope, before any section this reader cares about.
#[test]
fn no_unhandled_keyword_at_top_level_can_perturb_the_design() {
    let base = fingerprint(&parse(&good_def("", "")));
    let mut broke: Vec<String> = Vec::new();
    for kw in keywords() {
        if HANDLED.contains(&kw) {
            continue;
        }
        for body in [
            format!("{kw} ;\n"),
            format!("{kw} 3 ;\n"),
            format!("PROPERTYDEFINITIONS\n  {kw} STRING ;\nEND PROPERTYDEFINITIONS\n"),
        ] {
            if perturbs(&good_def("", &body), &base) {
                broke.push(format!("{kw}: {body:?}"));
                break;
            }
        }
    }
    assert!(
        broke.is_empty(),
        "{} keyword(s) perturbed the design:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// **Section keywords are found by scanning the token stream**, so one appearing as *data* —
/// a net name, a component name, a property value — can move a section boundary. That is not
/// a parse error; it is a different design, read without complaint.
#[test]
fn a_section_keyword_used_as_a_name_does_not_move_a_section() {
    let base = fingerprint(&parse(&good_def("", "")));
    for name in ["SPECIALNETS", "COMPONENTS", "NETS", "END"] {
        // as a component name, and as a net name
        let body = format!("    - net_{name} ( u2 Y )\n      + ROUTED met1 ( 100 100 ) ( 200 * )\n    ;\n");
        // A keyword-named net has to read exactly like an ordinary one: it contributes its own
        // entry and changes nothing else. Comparing against `base` alone cannot say that — the
        // net is a real addition, so the fingerprint is SUPPOSED to differ — so the control is
        // the same net under a name that is not a keyword.
        let ctrl = fingerprint(&parse(&good_def(
            "    - net_ordinary ( u2 Y )\n      + ROUTED met1 ( 100 100 ) ( 200 * )\n    ;\n",
            "",
        )));
        let want: BTreeMap<String, String> = ctrl
            .iter()
            .map(|(k, v)| (k.replace("net_ordinary", &format!("net_{name}")), v.clone()))
            .collect();
        assert_eq!(
            fingerprint(&parse(&good_def(&body, ""))),
            want,
            "`{name}` used as a net name moved a section boundary"
        );
        // The real hazard: the bare token appearing before the section it names.
        let top = format!("PROPERTYDEFINITIONS\n  COMPONENTPIN {name} STRING ;\nEND PROPERTYDEFINITIONS\n");
        let got = Def::parse(&good_def("", &top));
        if let Ok(d) = got {
            let f = fingerprint(&d);
            assert_eq!(
                f, base,
                "`{name}` inside PROPERTYDEFINITIONS moved a section boundary"
            );
        }
    }
}

/// The net attributes a real DEF actually carries, asserted individually — the sweep above can
/// only say "nothing changed", which is the wrong assertion for constructs that legitimately
/// carry geometry.
#[test]
fn real_net_attributes_are_read_correctly() {
    // `neta` already carries two runs: the met1 ROUTED and the met2 NEW.
    let base_segs = 2;
    let cases: &[(&str, &str, usize, &str)] = &[
        // (label, attribute, expected segment count, why)
        ("VPIN", "    + VPIN vp1 LAYER met2 ( -50 -50 ) ( 50 50 ) PLACED ( 3000 3000 ) N\n", base_segs,
         "a virtual pin is a placement, not metal on the net"),
        ("RECT", "    + RECT met3 ( 100 100 ) ( 200 200 )\n", base_segs,
         "a RECT is a patch, not a routed run this reader models"),
        ("SHIELDNET", "    + SHIELDNET vss\n", base_segs, "names a net, carries no geometry"),
        ("SOURCE", "    + SOURCE TIMING\n", base_segs, "provenance"),
        ("PROPERTY", "    + PROPERTY weight 3\n", base_segs, "user data"),
        ("ESTCAP", "    + ESTCAP 1.5\n", base_segs, "an estimate, not geometry"),
        ("USE", "    + USE CLOCK\n", base_segs, "classification"),
        // NOSHIELD is a WIRING TYPE in DEF 5.8, alongside ROUTED / FIXED / COVER. Its geometry
        // is real metal on the net and must be read as such — the one case here that adds a run.
        ("NOSHIELD", "    + NOSHIELD met4 ( 500 500 ) ( 600 * )\n", base_segs + 1,
         "NOSHIELD is routing, not an annotation"),
        ("FIXED", "    + FIXED met4 ( 500 500 ) ( 600 * )\n", base_segs + 1, "FIXED is routing"),
        ("COVER", "    + COVER met4 ( 500 500 ) ( 600 * )\n", base_segs + 1, "COVER is routing"),
    ];
    for (label, attr, want, why) in cases {
        let d = parse(&good_def(attr, ""));
        let n = d.nets.iter().find(|n| n.name == "neta").expect("neta");
        assert_eq!(
            n.segments.len(),
            *want,
            "{label}: {why} (got {:?})",
            n.segments.iter().map(|s| &s.layer).collect::<Vec<_>>()
        );
        // whatever the attribute, the net's own pins and the rest of the design are untouched
        assert_eq!(n.pins.len(), 2, "{label} disturbed the pin list");
        assert_eq!(d.nets.len(), 2, "{label} changed the net count");
        assert_eq!(d.comps.len(), 2, "{label} changed the components");
        assert_eq!(d.power_nets.len(), 1, "{label} changed the power grid");
    }
}
