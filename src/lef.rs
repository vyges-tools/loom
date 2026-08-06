//! Unified tech-LEF reader — the per-layer electrical + geometric attributes the
//! engines need, parsed once. **Superset** of the views the engines historically
//! kept separately:
//!
//! - extraction needs default routing **WIDTH** + metal **THICKNESS** (edge-gap
//!   coupling + the field kernel);
//! - PDN / EM needs sheet **RESISTANCE** (`RPERSQ`), **WIDTH**, and current-density
//!   limits (`DCCURRENTDENSITY AVERAGE`, `ACCURRENTDENSITY RMS|PEAK`).
//!
//! Reads `LAYER <name> … END <name>` blocks (ignoring vias / macros / pins).
//! Pure std — fully unit-tested offline.
//!
//! The struct keeps three projections in sync at parse time so every historical
//! consumer works unchanged: `layers` (full per-layer record), and the
//! width/thickness-only maps `widths` / `thicknesses`.

use std::collections::BTreeMap;

/// Per-layer attributes (union of the timing-extraction and PDN/EM views).
#[derive(Debug, Clone, Default)]
pub struct Layer {
    pub routing: bool,     // TYPE ROUTING (vs CUT / other) — the metal stack
    pub width_um: f64,     // default routing width (um)
    pub thickness_um: f64, // metal thickness (um) — field kernel
    pub rpersq: f64,       // sheet resistance (ohm/square) — PDN + RC
    pub cpersqdist: f64,   // area capacitance to the plane below (per unit^2) — RC
    pub edge_cap: f64,     // fringe / edge capacitance (per unit length) — RC
    pub cut_res: f64,      // per-cut resistance (ohm) on a CUT layer — via RC
    pub dc_jmax: f64,      // DC average current-density limit (mA/um) — EM
    pub ac_rms: f64,       // AC RMS current-density limit (mA/um)
    pub ac_peak: f64,      // AC peak current-density limit (mA/um)
}

/// Pin direction from a cell-LEF `MACRO`/`PIN` (`DIRECTION INPUT|OUTPUT|INOUT`).
/// Drives SPEF `*CONN` driver/load marking.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PinDir {
    #[default]
    Unknown,
    Input,
    Output,
    Inout,
}

#[derive(Debug, Clone, Default)]
pub struct MacroPin {
    pub name: String,
    pub direction: PinDir,
}

/// A std-cell abstract from the cell LEF `MACRO` section — the pin list + each
/// pin's direction (PORT geometry is skipped; RC comes from the tech LEF/liberty).
#[derive(Debug, Clone, Default)]
pub struct Macro {
    pub name: String,
    pub pins: BTreeMap<String, MacroPin>,
}

#[derive(Debug, Clone, Default)]
pub struct Lef {
    pub layers: BTreeMap<String, Layer>,
    /// Cell abstracts (MACRO section) — empty for a pure tech LEF.
    pub macros: BTreeMap<String, Macro>,
    /// routing layers in LEF **declaration order** (the metal stack, bottom→top) —
    /// so consumers that index by stack position (e.g. an OpenRCX captable's
    /// `Metal N`) map correctly even when names don't sort (metal2 vs metal10).
    pub routing_order: Vec<String>,
    /// layer → default routing width (um) — projection of `layers`.
    pub widths: BTreeMap<String, f64>,
    /// layer → metal thickness (um) — projection of `layers`.
    pub thicknesses: BTreeMap<String, f64>,
    /// via name → the layers its `VIA` block lists, in declaration order.
    ///
    /// A DEF net states a via by name at a point; only the LEF says which two routing
    /// layers that via actually joins. Everything else is inference — the routing layer
    /// in effect when the via is declared can be either side of the pair, so a consumer
    /// guessing from that alone will connect the wrong two layers wherever three meet.
    /// The list includes the cut layer, since a tech LEF is what tells routing from cut.
    pub vias: BTreeMap<String, Vec<String>>,
    /// `UNITS` conversion factors this reader did NOT apply, as `(quantity, factor)`.
    ///
    /// LEF states its electrical units in a `UNITS` block — `CAPACITANCE PICOFARADS 1 ;`,
    /// `RESISTANCE OHMS 1 ;`. The factor is 1 in every PDK we have seen, and this reader
    /// assumes it. If a file says otherwise, every resistance and capacitance we report is
    /// scaled wrong, and nothing about the numbers would look unusual — so the assumption is
    /// recorded rather than made silently. See [`Lef::health`].
    pub unapplied_units: Vec<(String, f64)>,
}

/// Interpret ONE complete `;`-terminated statement from a `LAYER` block.
///
/// The statement is the whole construct, so its first token names it unambiguously: a
/// `SPACINGTABLE ... WIDTH 3 0.28` arrives here as one `SPACINGTABLE` statement and matches
/// nothing, where a line-based reader would have seen a bare `WIDTH 3 0.28` row.
fn apply_layer_stmt(stmt: &[String], l: &mut Layer) {
    let t: Vec<&str> = stmt.iter().map(String::as_str).collect();
    let num = |s: &str| s.parse::<f64>().ok();
    match t.as_slice() {
        ["WIDTH", w] => {
            if let Some(v) = num(w) {
                l.width_um = v;
            }
        }
        ["THICKNESS", x] => {
            if let Some(v) = num(x) {
                l.thickness_um = v;
            }
        }
        ["TYPE", "ROUTING"] => l.routing = true,
        ["RESISTANCE", "RPERSQ", v] => {
            if let Some(x) = num(v) {
                l.rpersq = x;
            }
        }
        // plain RESISTANCE <ohm> on a CUT layer = per-cut via resistance
        ["RESISTANCE", v] => {
            if let Some(x) = num(v) {
                l.cut_res = x;
            }
        }
        ["CAPACITANCE", "CPERSQDIST", v] => {
            if let Some(x) = num(v) {
                l.cpersqdist = x;
            }
        }
        ["EDGECAPACITANCE", v] => {
            if let Some(x) = num(v) {
                l.edge_cap = x;
            }
        }
        ["DCCURRENTDENSITY", "AVERAGE", v] => {
            if let Some(x) = num(v) {
                l.dc_jmax = x;
            }
        }
        ["ACCURRENTDENSITY", "RMS", v] => {
            if let Some(x) = num(v) {
                l.ac_rms = x;
            }
        }
        ["ACCURRENTDENSITY", "PEAK", v] => {
            if let Some(x) = num(v) {
                l.ac_peak = x;
            }
        }
        _ => {}
    }
}

impl Lef {
    /// One line on whether the read can be trusted, or `None` when nothing looks wrong.
    pub fn health(&self) -> Option<String> {
        let mut notes = Vec::new();
        for (q, f) in &self.unapplied_units {
            notes.push(format!(
                "UNITS {q} conversion factor is {f}, not 1 — this reader does not apply it, so                  every {} value it reports is scaled by {f}",
                q.to_ascii_lowercase()
            ));
        }
        // A **cell** LEF is macros with no LAYER blocks, and a **PDN** LEF names layers with no
        // attributes at all — both perfectly normal. Only a file that carries per-layer
        // ELECTRICAL data, and therefore means to be a tech LEF, is suspect without a routing
        // stack: that is the one whose numbers we would go on to use.
        let has_electrical = self
            .layers
            .values()
            .any(|l| l.width_um > 0.0 || l.rpersq > 0.0 || l.thickness_um > 0.0);
        if has_electrical && self.routing_order.is_empty() {
            notes.push(
                "layers carry electrical data but none is TYPE ROUTING — nothing to extract on"
                    .to_string(),
            );
        }
        (!notes.is_empty()).then(|| notes.join("; "))
    }
}

#[derive(Debug)]
pub struct LefError(pub String);
impl std::fmt::Display for LefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "lef error: {}", self.0)
    }
}
impl std::error::Error for LefError {}

impl Lef {
    /// Parse LEF text. Errors when no `LAYER` blocks are found (a LEF with no
    /// layers is almost always a wrong/empty file). Consumers that want a lenient
    /// "no LEF" path use `Lef::default()` instead of parsing an empty string.
    pub fn parse(text: &str) -> Result<Lef, LefError> {
        let mut layers: BTreeMap<String, Layer> = BTreeMap::new();
        let mut routing_order: Vec<String> = Vec::new();
        let mut cur: Option<(String, Layer)> = None;
        let mut macros: BTreeMap<String, Macro> = BTreeMap::new();
        // Macro-section state. While inside a MACRO we must NOT treat its inner
        // `LAYER <m> ;` (PIN PORT geometry) lines as tech-layer starts.
        let mut cur_macro: Option<Macro> = None;
        let mut cur_pin: Option<MacroPin> = None;
        let mut vias: BTreeMap<String, Vec<String>> = BTreeMap::new();
        // VIA / VIARULE blocks carry their own `LAYER <l> ;` lines. Those are not tech-layer
        // declarations and must not be read as such, so the block is shielded the same way
        // a MACRO is. VIA blocks additionally record what they join; VIARULE is generated
        // geometry and only needs shielding.
        let mut cur_via: Option<(String, Vec<String>, bool)> = None;
        let mut in_propdefs = false;
        // A LEF statement is `;`-terminated and may span lines; these carry the partial one.
        let mut stmt: Vec<String> = Vec::new();
        let mut in_quote = false;
        let mut in_units = false;
        let mut unapplied_units: Vec<(String, f64)> = Vec::new();
        for raw in text.lines() {
            let line = match raw.find('#') {
                Some(i) => &raw[..i],
                None => raw,
            };
            let toks: Vec<&str> = line.split_whitespace().collect();

            // ---- via mode: record the layers a VIA joins, shield tech parsing ----
            if let Some((name, ls, record)) = cur_via.as_mut() {
                match toks.as_slice() {
                    ["LAYER", l, ..] if *record => ls.push(l.trim_end_matches(';').to_string()),
                    ["END", n, ..] if n == name => {
                        let (n, ls, record) = cur_via.take().unwrap();
                        if record {
                            vias.insert(n, ls);
                        }
                    }
                    _ => {}
                }
                continue;
            }
            // ---- property-definition mode: shield tech parsing ----
            // `PROPERTYDEFINITIONS` legally contains `LAYER <property-name> STRING ;`, a line
            // shaped exactly like the start of a tech layer — and sky130's own tech LEF has one.
            // Without the shield the reader opens a layer named after the property; today it
            // survives only because the next real `LAYER` overwrites it before it is inserted,
            // which is luck, not a rule.
            if in_propdefs {
                if let ["END", "PROPERTYDEFINITIONS", ..] = toks.as_slice() {
                    in_propdefs = false;
                }
                continue;
            }
            match toks.as_slice() {
                ["PROPERTYDEFINITIONS", ..] => {
                    in_propdefs = true;
                    continue;
                }
                ["UNITS", ..] => {
                    in_units = true;
                    continue;
                }
                ["VIA", name, ..] => {
                    cur_via = Some((name.to_string(), Vec::new(), true));
                    continue;
                }
                ["VIARULE", name, ..] => {
                    cur_via = Some((name.to_string(), Vec::new(), false));
                    continue;
                }
                _ => {}
            }

            // ---- units mode: record the conversion factors, shield tech parsing ----
            if in_units {
                match toks.as_slice() {
                    ["END", "UNITS", ..] => in_units = false,
                    [q @ ("CAPACITANCE" | "RESISTANCE"), _unit, f, ..] => {
                        if let Ok(v) = f.trim_end_matches(';').parse::<f64>() {
                            if (v - 1.0).abs() > 1e-12 {
                                unapplied_units.push((q.to_string(), v));
                            }
                        }
                    }
                    _ => {}
                }
                continue;
            }

            // ---- macro mode: consume MACRO/PIN/DIRECTION, shield tech parsing ----
            if let Some(m) = cur_macro.as_mut() {
                match toks.as_slice() {
                    ["PIN", name, ..] => {
                        cur_pin = Some(MacroPin {
                            name: name.to_string(),
                            direction: PinDir::default(),
                        });
                    }
                    ["DIRECTION", d, ..] => {
                        if let Some(p) = cur_pin.as_mut() {
                            p.direction =
                                match d.trim_end_matches(';').to_ascii_uppercase().as_str() {
                                    "INPUT" => PinDir::Input,
                                    "OUTPUT" | "OUTPUT_TRISTATE" => PinDir::Output,
                                    "INOUT" => PinDir::Inout,
                                    _ => PinDir::Unknown,
                                };
                        }
                    }
                    ["END", name, ..] => {
                        if cur_pin.as_ref().map(|p| &p.name == name).unwrap_or(false) {
                            let p = cur_pin.take().unwrap();
                            m.pins.insert(p.name.clone(), p);
                        } else if &m.name == name {
                            let done = cur_macro.take().unwrap();
                            macros.insert(done.name.clone(), done);
                        }
                    }
                    _ => {} // SIZE/ORIGIN/FOREIGN/PORT/LAYER/RECT... ignored
                }
                continue;
            }
            if let ["MACRO", name, ..] = toks.as_slice() {
                cur_macro = Some(Macro {
                    name: name.to_string(),
                    pins: BTreeMap::new(),
                });
                cur_pin = None;
                continue;
            }

            match toks.as_slice() {
                ["LAYER", name, ..] => {
                    stmt.clear();
                    in_quote = false;
                    cur = Some((name.to_string(), Layer::default()));
                }
                // `END <name>` closes a block and carries no `;`. Anything still buffered is
                // an unterminated statement — dropped, not guessed at.
                ["END", name, ..] if cur.as_ref().map(|(n, _)| n == name).unwrap_or(false) => {
                    stmt.clear();
                    in_quote = false;
                    if let Some((n, l)) = cur.take() {
                        if l.routing {
                            routing_order.push(n.clone());
                        }
                        layers.insert(n, l);
                    }
                }
                rest => {
                    if cur.is_some() {
                        // ---- STATEMENTS, NOT LINES ----
                        //
                        // A LEF statement ends at `;`, and may span many lines. `SPACINGTABLE`
                        // is one statement whose body carries `WIDTH 0 0.15` / `WIDTH 3 0.28`
                        // rows; read line by line, those look exactly like the layer's own
                        // `WIDTH 0.14 ;`, and last-write-wins once made met1 report a routing
                        // width of 3 um in the middle of a correlation study.
                        //
                        // Matching per line makes every such row a candidate and leaves the
                        // reader guarding each collision by hand — a generated sweep over the
                        // grammar found 422 of 423 keywords able to smuggle a wrong value in
                        // this way. Accumulating to the `;` instead dissolves the whole class:
                        // a nested construct is ONE statement, its first token names it, and
                        // nothing inside it is ever mistaken for an attribute of the layer.
                        //
                        // Quotes are respected, because a `PROPERTY LEF58_x "...;..."` string
                        // legally contains semicolons that do not end anything.
                        for t in rest {
                            if in_quote {
                                stmt.push((*t).to_string());
                                if t.ends_with('"') {
                                    in_quote = false;
                                }
                                continue;
                            }
                            if t.starts_with('"') && !(t.len() > 1 && t.ends_with('"')) {
                                in_quote = true;
                                stmt.push((*t).to_string());
                                continue;
                            }
                            if *t == ";" {
                                apply_layer_stmt(&stmt, cur.as_mut().map(|(_, l)| l).unwrap());
                                stmt.clear();
                            } else if let Some(head) = t.strip_suffix(';') {
                                if !head.is_empty() {
                                    stmt.push(head.to_string());
                                }
                                apply_layer_stmt(&stmt, cur.as_mut().map(|(_, l)| l).unwrap());
                                stmt.clear();
                            } else {
                                stmt.push((*t).to_string());
                            }
                        }
                    }
                }
            }
        }
        // A pure cell LEF has MACROs but no LAYER blocks — that's valid. Only a LEF
        // with neither (a wrong/empty file) is an error.
        if layers.is_empty() && macros.is_empty() {
            return Err(LefError("no LAYER or MACRO blocks found".into()));
        }
        let mut widths = BTreeMap::new();
        let mut thicknesses = BTreeMap::new();
        for (n, l) in &layers {
            if l.width_um != 0.0 {
                widths.insert(n.clone(), l.width_um);
            }
            if l.thickness_um != 0.0 {
                thicknesses.insert(n.clone(), l.thickness_um);
            }
        }
        Ok(Lef {
            layers,
            macros,
            routing_order,
            widths,
            thicknesses,
            vias,
            unapplied_units,
        })
    }

    pub fn load(path: &str) -> Result<Lef, LefError> {
        let text = std::fs::read_to_string(path).map_err(|e| LefError(format!("{path}: {e}")))?;
        Lef::parse(&text)
    }

    /// Direction of `pin` on cell `cell` from the MACRO section (Unknown if absent).
    pub fn pin_dir(&self, cell: &str, pin: &str) -> PinDir {
        self.macros
            .get(cell)
            .and_then(|m| m.pins.get(pin))
            .map(|p| p.direction)
            .unwrap_or(PinDir::Unknown)
    }

    /// Default routing width for a layer (0.0 if unknown).
    pub fn width(&self, layer: &str) -> f64 {
        self.widths.get(layer).copied().unwrap_or(0.0)
    }

    /// Metal thickness for a layer (0.0 if unknown).
    pub fn thickness(&self, layer: &str) -> f64 {
        self.thicknesses.get(layer).copied().unwrap_or(0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_extraction_and_pdn_fields_from_one_lef() {
        let text = "\
LAYER met1
  TYPE ROUTING ;
  WIDTH 0.14 ;
  THICKNESS 0.36 ;
  RESISTANCE RPERSQ 0.125 ;
  DCCURRENTDENSITY AVERAGE 1.5 ;
  ACCURRENTDENSITY RMS 2.0 ;
  ACCURRENTDENSITY PEAK 4.0 ;
END met1
";
        let lef = Lef::parse(text).unwrap();
        let l = lef.layers.get("met1").expect("met1");
        assert_eq!(l.width_um, 0.14);
        assert_eq!(l.thickness_um, 0.36);
        assert_eq!(l.rpersq, 0.125);
        assert_eq!(l.dc_jmax, 1.5);
        assert_eq!(l.ac_rms, 2.0);
        assert_eq!(l.ac_peak, 4.0);
        // projections kept in sync
        assert_eq!(lef.width("met1"), 0.14);
        assert_eq!(lef.thickness("met1"), 0.36);
        assert_eq!(lef.widths.get("met1"), Some(&0.14));
    }

    #[test]
    fn empty_errors() {
        assert!(Lef::parse("# no layers here\n").is_err());
    }

    #[test]
    fn a_spacing_table_does_not_overwrite_the_routing_width() {
        // Verbatim shape from `sky130_fd_sc_hd__nom.tlef`. The SPACINGTABLE rows are
        // (width, spacing) pairs, not width declarations; taking the last WIDTH seen made
        // met1 3 um wide, which silently multiplied every coupling capacitance on the block.
        let lef = "\
LAYER met1
  TYPE ROUTING ;
  WIDTH 0.14 ;
  SPACING 0.14 ;
  SPACINGTABLE
    PARALLELRUNLENGTH 0
    WIDTH 0 0.14
    WIDTH 3 0.28 ;
  THICKNESS 0.35 ;
END met1
";
        let l = Lef::parse(lef).unwrap();
        assert_eq!(
            l.widths["met1"], 0.14,
            "the declared routing width, not a table row"
        );
        assert!((l.thicknesses["met1"] - 0.35).abs() < 1e-9);
    }

    #[test]
    fn a_via_block_records_what_it_joins_and_does_not_leak_into_the_layers() {
        // Verbatim from a sky130 tech LEF. The `LAYER via ;` / `LAYER met1 ;` lines inside a
        // VIA block are not tech-layer declarations, and a reader that treats them as such
        // parses a via's geometry into a layer record.
        let lef = "\
LAYER met1
  TYPE ROUTING ;
  WIDTH 0.14 ;
END met1
VIA M1M2_PR DEFAULT
  LAYER via ;
  RECT -0.075 -0.075 0.075 0.075 ;
  LAYER met1 ;
  RECT -0.16 -0.13 0.16 0.13 ;
  LAYER met2 ;
  RECT -0.13 -0.16 0.13 0.16 ;
END M1M2_PR
VIARULE via1_rule GENERATE
  LAYER met1 ;
  LAYER met2 ;
END via1_rule
LAYER met2
  TYPE ROUTING ;
  WIDTH 0.14 ;
END met2
";
        let l = Lef::parse(lef).unwrap();
        assert_eq!(
            l.vias.get("M1M2_PR").map(|v| v.as_slice()),
            Some(["via", "met1", "met2"].map(String::from).as_slice()),
            "the layers the via joins, cut included"
        );
        assert!(
            !l.vias.contains_key("via1_rule"),
            "VIARULE is generated geometry, not a placeable via"
        );
        // and the real layers came through untouched, on both sides of the via blocks
        assert_eq!(l.routing_order, vec!["met1", "met2"]);
        assert!((l.widths["met1"] - 0.14).abs() < 1e-9);
        assert!(
            !l.layers.contains_key("via"),
            "a via cut is not a tech layer here"
        );
    }

    #[test]
    fn parses_macro_pins_with_direction() {
        // cell LEF: MACRO/PIN with DIRECTION, incl. a PORT `LAYER` that must NOT be
        // mistaken for a tech layer.
        let text = "\
MACRO INV_X1
  SIZE 1.2 BY 2.0 ;
  PIN A
    DIRECTION INPUT ;
    PORT
      LAYER met1 ;
      RECT 0.1 0.1 0.2 0.3 ;
    END
  END A
  PIN Y
    DIRECTION OUTPUT ;
    PORT
      LAYER met1 ;
    END
  END Y
END INV_X1
MACRO DFF_X1
  PIN CK
    DIRECTION INPUT ;
  END CK
  PIN Q
    DIRECTION OUTPUT ;
  END Q
END DFF_X1
";
        let lef = Lef::parse(text).unwrap();
        assert_eq!(lef.layers.len(), 0); // PORT `LAYER met1` did NOT leak into layers
        assert_eq!(lef.macros.len(), 2);
        assert_eq!(lef.pin_dir("INV_X1", "A"), PinDir::Input);
        assert_eq!(lef.pin_dir("INV_X1", "Y"), PinDir::Output);
        assert_eq!(lef.pin_dir("DFF_X1", "Q"), PinDir::Output);
        assert_eq!(lef.pin_dir("INV_X1", "ZZ"), PinDir::Unknown);
    }
}
