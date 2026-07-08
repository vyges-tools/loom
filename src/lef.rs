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

#[derive(Debug, Clone, Default)]
pub struct Lef {
    pub layers: BTreeMap<String, Layer>,
    /// layer → default routing width (um) — projection of `layers`.
    pub widths: BTreeMap<String, f64>,
    /// layer → metal thickness (um) — projection of `layers`.
    pub thicknesses: BTreeMap<String, f64>,
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
        let mut cur: Option<(String, Layer)> = None;
        for raw in text.lines() {
            let line = match raw.find('#') {
                Some(i) => &raw[..i],
                None => raw,
            };
            let toks: Vec<&str> = line.split_whitespace().collect();
            match toks.as_slice() {
                ["LAYER", name, ..] => cur = Some((name.to_string(), Layer::default())),
                ["END", name, ..] if cur.as_ref().map(|(n, _)| n == name).unwrap_or(false) => {
                    if let Some((n, l)) = cur.take() {
                        layers.insert(n, l);
                    }
                }
                rest => {
                    if let Some((_, l)) = cur.as_mut() {
                        let num = |s: &str| s.trim_end_matches(';').parse::<f64>().ok();
                        match rest {
                            ["WIDTH", w, ..] => {
                                if let Some(v) = num(w) {
                                    l.width_um = v;
                                }
                            }
                            ["THICKNESS", t, ..] => {
                                if let Some(v) = num(t) {
                                    l.thickness_um = v;
                                }
                            }
                            ["TYPE", "ROUTING", ..] => l.routing = true,
                            ["RESISTANCE", "RPERSQ", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.rpersq = x;
                                }
                            }
                            // plain RESISTANCE <ohm> on a CUT layer = per-cut via resistance
                            ["RESISTANCE", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.cut_res = x;
                                }
                            }
                            ["CAPACITANCE", "CPERSQDIST", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.cpersqdist = x;
                                }
                            }
                            ["EDGECAPACITANCE", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.edge_cap = x;
                                }
                            }
                            ["DCCURRENTDENSITY", "AVERAGE", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.dc_jmax = x;
                                }
                            }
                            ["ACCURRENTDENSITY", "RMS", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.ac_rms = x;
                                }
                            }
                            ["ACCURRENTDENSITY", "PEAK", v, ..] => {
                                if let Some(x) = num(v) {
                                    l.ac_peak = x;
                                }
                            }
                            _ => {}
                        }
                    }
                }
            }
        }
        if layers.is_empty() {
            return Err(LefError("no LAYER blocks found".into()));
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
        Ok(Lef { layers, widths, thicknesses })
    }

    pub fn load(path: &str) -> Result<Lef, LefError> {
        let text = std::fs::read_to_string(path).map_err(|e| LefError(format!("{path}: {e}")))?;
        Lef::parse(&text)
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
}
