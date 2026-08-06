//! Schema-driven robustness for the Liberty reader, plus the vendor differences that broke it.
//!
//! Liberty has no freely-licensed grammar we can mine — its reference parsers are GPLv3 — so the
//! inventory in `tests/data/liberty_attributes.txt` is mined from the **data**: every attribute
//! and group name occurring in 12 real libraries across sky130 and ASAP7. That is the list that
//! matters, being what we are actually asked to read.
//!
//! Two sources, deliberately, because they disagree in every way that has bitten us:
//!
//! | | sky130 | ASAP7 |
//! | --- | --- | --- |
//! | time | `time_unit : "1ns"` | `time_unit : "1ps"` |
//! | capacitance | `capacitive_load_unit(1.0, "pf")` | `capacitive_load_unit (1,ff)` — unquoted |
//! | `area` | `area : 11.26;` | `area : 0.20412` — **no semicolon** |
//! | leakage | `cell_leakage_power : x;` | `leakage_power () { value ; when ; }` groups |
//!
//! A reader tuned to one vendor reports the other as zero — silently, since a missing number in
//! Liberty is indistinguishable from a number that is genuinely zero.

use std::collections::BTreeMap;

use vyges_loom::liberty::Lib;

fn attributes() -> Vec<&'static str> {
    include_str!("data/liberty_attributes.txt")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .collect()
}

/// Names this reader deliberately acts on.
const HANDLED: &[&str] = &[
    "library", "cell", "pin", "timing", "direction", "capacitance", "area", "clock", "function",
    "cell_leakage_power", "leakage_power", "value", "when", "related_pin", "timing_sense",
    "timing_type", "cell_rise", "cell_fall", "rise_transition", "fall_transition", "index_1",
    "index_2", "values", "time_unit", "voltage_unit", "current_unit", "leakage_power_unit",
    "capacitive_load_unit", "ff", "latch", "clear", "preset", "cell_footprint", "internal_power",
    "rise_power", "fall_power", "nominal_voltage", "default_max_transition", "slew_lower_threshold_pct_rise",
    "slew_upper_threshold_pct_rise", "slew_lower_threshold_pct_fall", "slew_upper_threshold_pct_fall",
    "input_threshold_pct_rise", "output_threshold_pct_rise", "input_threshold_pct_fall",
    "output_threshold_pct_fall", "receiver_capacitance1_rise", "receiver_capacitance2_rise",
    "receiver_capacitance1_fall", "receiver_capacitance2_fall", "output_current_rise",
    "output_current_fall", "rise_constraint", "fall_constraint", "ocv_sigma_cell_rise",
    "ocv_sigma_cell_fall",
];

/// A library exercising everything this reader claims to read.
fn good_lib(in_library: &str, in_cell: &str, in_pin: &str, in_timing: &str) -> String {
    format!(
        r#"library (L) {{
  time_unit : "1ns";
  voltage_unit : "1V";
  current_unit : "1mA";
  leakage_power_unit : "1nW";
  capacitive_load_unit(1.0, "pf");
  nom_voltage : 1.8;
{in_library}
  cell (INV) {{
    area : 3.5;
    cell_leakage_power : 2.0;
{in_cell}
    pin (A) {{
      direction : input;
      capacitance : 0.004;
{in_pin}
    }}
    pin (Y) {{
      direction : output;
      function : "!A";
      timing () {{
        related_pin : "A";
        timing_sense : negative_unate;
{in_timing}
        cell_rise (t) {{
          index_1 ("0.01, 0.1");
          index_2 ("0.001, 0.01");
          values ("0.10, 0.20", "0.30, 0.40");
        }}
        cell_fall (t) {{
          index_1 ("0.01, 0.1");
          index_2 ("0.001, 0.01");
          values ("0.11, 0.21", "0.31, 0.41");
        }}
        rise_transition (t) {{
          index_1 ("0.01, 0.1");
          index_2 ("0.001, 0.01");
          values ("0.02, 0.03", "0.04, 0.05");
        }}
        fall_transition (t) {{
          index_1 ("0.01, 0.1");
          index_2 ("0.001, 0.01");
          values ("0.06, 0.07", "0.08, 0.09");
        }}
      }}
    }}
  }}
}}
"#
    )
}

/// Everything this reader is supposed to extract, as one comparable value.
fn fingerprint(l: &Lib) -> BTreeMap<String, String> {
    let mut m = BTreeMap::new();
    for (cn, c) in &l.cells {
        m.insert(
            format!("cell:{cn}"),
            format!("area={:.6} leak={:.6e} seq={}", c.area, c.leakage_w, c.is_seq),
        );
        for (pn, p) in &c.pins {
            m.insert(
                format!("pin:{cn}:{pn}"),
                format!(
                    "dir={:?} cap={:.9} capF={:.6e} clk={} fn={:?} arcs={}",
                    p.direction,
                    p.capacitance,
                    p.cap_f,
                    p.clock,
                    p.function,
                    p.arcs.len()
                ),
            );
            for (i, a) in p.arcs.iter().enumerate() {
                m.insert(
                    format!("arc:{cn}:{pn}:{i}"),
                    format!(
                        "rel={} sense={} d={:.6} t={:.6}",
                        a.related_pin,
                        a.sense,
                        a.cell_rise.lookup(0.05, 0.005),
                        a.rise_transition.lookup(0.05, 0.005)
                    ),
                );
            }
        }
    }
    m
}

fn parse(text: &str) -> Lib {
    Lib::parse(text).expect("the base library is valid Liberty")
}

/// A hard error is loud and acceptable; a quiet change of the numbers is not.
fn perturbs(text: &str, base: &BTreeMap<String, String>) -> bool {
    match Lib::parse(text) {
        Err(_) => false,
        Ok(l) => fingerprint(&l) != *base,
    }
}

#[test]
fn the_known_good_library_reads_as_expected() {
    let l = parse(&good_lib("", "", "", ""));
    let c = l.cells.get("INV").expect("INV");
    assert_eq!(c.area, 3.5);
    assert!((c.leakage_w - 2.0e-9).abs() < 1e-18, "2 nW, got {}", c.leakage_w);
    let a = c.pins.get("A").expect("pin A");
    assert_eq!(a.capacitance, 0.004, "library units (pF), the NLDM load axis");
    assert!((a.cap_f - 4.0e-15).abs() < 1e-21, "4 fF in SI, got {}", a.cap_f);
    let y = c.pins.get("Y").expect("pin Y");
    assert_eq!(y.function.as_deref(), Some("!A"));
    assert_eq!(y.arcs.len(), 1);
    // exactly on a table corner, so interpolation cannot hide a mis-parse
    assert!((y.arcs[0].cell_rise.lookup(0.01, 0.001) - 0.10).abs() < 1e-12);
    assert!((y.arcs[0].cell_fall.lookup(0.1, 0.01) - 0.41).abs() < 1e-12);
}

/// **The generated sweep.** Every attribute name real libraries use, injected at each scope.
#[test]
fn no_unhandled_attribute_can_perturb_what_we_read() {
    let base = fingerprint(&parse(&good_lib("", "", "", "")));
    let mut broke: Vec<String> = Vec::new();
    for a in attributes() {
        if HANDLED.contains(&a) {
            continue;
        }
        // simple attribute, quoted attribute, group with a body, and a group carrying a table —
        // the shapes a Liberty attribute actually takes.
        let shapes = [
            format!("  {a} : 1.0;\n"),
            format!("  {a} : \"x\";\n"),
            format!("  {a} () {{ value : 99.0; }}\n"),
            format!("  {a} (g) {{ index_1 (\"9\"); values (\"99\"); }}\n"),
        ];
        let mut hit = false;
        for s in &shapes {
            if perturbs(&good_lib(s, "", "", ""), &base)
                || perturbs(&good_lib("", s, "", ""), &base)
                || perturbs(&good_lib("", "", s, ""), &base)
                || perturbs(&good_lib("", "", "", s), &base)
            {
                hit = true;
                break;
            }
        }
        if hit {
            broke.push(a.to_string());
        }
    }
    assert!(
        broke.is_empty(),
        "{} attribute(s) perturbed a value we read:\n  {}",
        broke.len(),
        broke.join("\n  ")
    );
}

/// **ASAP7 omits the terminating semicolon on `area`.** Scanning to the next `;` then swallows
/// the following lines, the value fails to parse, and the cell reports an area of ZERO — which
/// sorts it first among interchangeable cells, so a sizer would pick it every time.
#[test]
fn an_attribute_without_a_semicolon_still_parses() {
    let l = parse(&good_lib("", "", "", "").replace("area : 3.5;", "area : 3.5"));
    assert_eq!(l.cells["INV"].area, 3.5);

    // and the attribute after it is unaffected
    assert!((l.cells["INV"].leakage_w - 2.0e-9).abs() < 1e-18);

    // the pathological form: no semicolon, immediately followed by a group
    let text = good_lib("", "", "", "").replace(
        "area : 3.5;",
        "area : 3.5\n    pg_pin (VDD) {\n      pg_type : primary_power;\n    }",
    );
    assert_eq!(parse(&text).cells["INV"].area, 3.5);
}

/// **ASAP7 states leakage per logic state**, with no `cell_leakage_power` at all. Reading only
/// the scalar reports those cells as leaking nothing.
#[test]
fn per_state_leakage_groups_are_read_when_the_scalar_is_absent() {
    let groups = "\
    leakage_power () { value : 496.771; when : \"(A * Y)\"; related_pg_pin : VDD; }
    leakage_power () { value : 0; when : \"(A * Y)\"; related_pg_pin : VSS; }
    leakage_power () { value : 286.02; when : \"(!A * !Y)\"; related_pg_pin : VDD; }
";
    let text = good_lib("", groups, "", "").replace("    cell_leakage_power : 2.0;\n", "");
    let l = parse(&text);
    // Two states: (A*Y) = 496.771 + 0, (!A*!Y) = 286.02. Mean = 391.3955 nW.
    let want = 391.3955e-9;
    let got = l.cells["INV"].leakage_w;
    assert!((got - want).abs() < 1e-15, "want {want:e}, got {got:e}");

    // the explicit scalar still wins where a library provides both
    let both = good_lib("", groups, "", "");
    assert!((parse(&both).cells["INV"].leakage_w - 2.0e-9).abs() < 1e-18);

    // and a library that characterises no leakage at all honestly reports none
    let none = good_lib("", "", "", "").replace("    cell_leakage_power : 2.0;\n", "");
    assert_eq!(parse(&none).cells["INV"].leakage_w, 0.0);
}

/// Units, in both vendors' spellings. A capacitance read in the wrong unit is off by 1000 and
/// looks entirely plausible.
#[test]
fn both_vendor_unit_spellings_are_understood() {
    // sky130: quoted, with a decimal multiplier
    let pf = parse(&good_lib("", "", "", ""));
    assert!((pf.cells["INV"].pins["A"].cap_f - 4.0e-15).abs() < 1e-21);

    // ASAP7: unquoted, space before the paren, femtofarads
    let ff = parse(
        &good_lib("", "", "", "")
            .replace("capacitive_load_unit(1.0, \"pf\");", "capacitive_load_unit (1,ff);")
            .replace("capacitance : 0.004;", "capacitance : 4.0;"),
    );
    assert!(
        (ff.cells["INV"].pins["A"].cap_f - 4.0e-15).abs() < 1e-21,
        "4 fF, got {:e}",
        ff.cells["INV"].pins["A"].cap_f
    );
    // the NLDM axis stays in library units either way — that is what the tables are indexed on
    assert_eq!(ff.cells["INV"].pins["A"].capacitance, 4.0);
}

/// Table values live in the library's own time unit, so a picosecond library and a nanosecond
/// one are not comparable. Pin this, because it is the assumption every delay depends on.
#[test]
fn table_values_stay_in_library_time_units() {
    let ns = parse(&good_lib("", "", "", ""));
    let ps = parse(&good_lib("", "", "", "").replace("time_unit : \"1ns\";", "time_unit : \"1ps\";"));
    let d = |l: &Lib| l.cells["INV"].pins["Y"].arcs[0].cell_rise.lookup(0.01, 0.001);
    assert_eq!(d(&ns), d(&ps), "the number is the file's; the unit is the caller's to track");
}
