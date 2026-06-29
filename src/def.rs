//! Unified DEF reader — parsed **once** into both views the engines need:
//!
//! - **signal `NETS`** (routed geometry in microns) → [`DefNet`] / [`Segment`],
//!   what RC **extraction** consumes;
//! - **`SPECIALNETS`** power grid (geometry in DB units) → [`NetGeom`] / [`Seg`]
//!   plus **`COMPONENTS`** placement → [`Comp`], what **PDN / IR-drop** consumes.
//!
//! Superset of the two readers the engines historically kept separately. One
//! tokenize + scale; the signal pass (µm, `f64`) and the power/components pass
//! (DBU, `i64`) run over the same token stream. `( * y )` / `( x * )` shorthand is
//! resolved in both. Pure std — unit-tested offline.

use std::collections::BTreeMap;

// ─────────────────────────── signal view (extraction, µm) ──────────────────────

#[derive(Debug, Clone)]
pub struct Segment {
    pub layer: String,
    pub x0: f64,
    pub y0: f64,
    pub x1: f64,
    pub y1: f64,
    /// Drawn routing width (µm) when a non-default rule (NDR / `TAPERRULE`) sets
    /// one; `0.0` means "use the layer's default width" (the LEF routing width).
    pub width_um: f64,
}

impl Segment {
    /// A wire segment with the layer's default width (`width_um == 0`).
    pub fn wire(layer: impl Into<String>, x0: f64, y0: f64, x1: f64, y1: f64) -> Segment {
        Segment { layer: layer.into(), x0, y0, x1, y1, width_um: 0.0 }
    }

    /// Manhattan length in microns.
    pub fn len_um(&self) -> f64 {
        (self.x1 - self.x0).abs() + (self.y1 - self.y0).abs()
    }
    pub fn is_horizontal(&self) -> bool {
        (self.y1 - self.y0).abs() < 1e-9 && (self.x1 - self.x0).abs() > 1e-9
    }
    pub fn is_vertical(&self) -> bool {
        (self.x1 - self.x0).abs() < 1e-9 && (self.y1 - self.y0).abs() > 1e-9
    }
    /// Footprint rectangle (xmin, ymin, xmax, ymax) when swept by `width`.
    pub fn footprint(&self, width: f64) -> (f64, f64, f64, f64) {
        let hw = width / 2.0;
        let (xlo, xhi) = (self.x0.min(self.x1), self.x0.max(self.x1));
        let (ylo, yhi) = (self.y0.min(self.y1), self.y0.max(self.y1));
        if self.is_horizontal() {
            (xlo, ylo - hw, xhi, yhi + hw)
        } else if self.is_vertical() {
            (xlo - hw, ylo, xhi + hw, yhi)
        } else {
            (xlo, ylo, xhi, yhi)
        }
    }
}

#[derive(Debug, Clone)]
pub struct DefNet {
    pub name: String,
    pub pins: Vec<(String, String)>, // (instance, pin)
    pub segments: Vec<Segment>,
    pub vias: usize,
}

// ─────────────────────────── power view (PDN, DBU) ─────────────────────────────

#[derive(Debug, Clone)]
pub struct Seg {
    pub layer: String,
    pub width_dbu: f64,
    pub x1: i64,
    pub y1: i64,
    pub x2: i64,
    pub y2: i64,
}

#[derive(Debug, Clone, Default)]
pub struct NetGeom {
    pub name: String,
    pub use_power: bool,
    pub segs: Vec<Seg>,
    pub vias: Vec<(i64, i64)>,
    /// Every listed coordinate with its wire layer (incl. via-only landings).
    pub points: Vec<(String, i64, i64)>,
}

/// A placed instance from the DEF `COMPONENTS` section.
#[derive(Debug, Clone)]
pub struct Comp {
    pub name: String,
    pub cell: String,
    pub x: i64,
    pub y: i64,
}

// ─────────────────────────── the unified design ────────────────────────────────

#[derive(Debug, Clone, Default)]
pub struct Def {
    /// DB units per micron (DEF `UNITS DISTANCE MICRONS`). `units_per_um` and
    /// `dbu` are the same value, named for each consumer's historical field.
    pub units_per_um: f64,
    pub dbu: f64,
    /// Signal nets (microns) — RC extraction.
    pub nets: Vec<DefNet>,
    /// Power grid special nets (DB units) — PDN.
    pub power_nets: Vec<NetGeom>,
    /// Placed instances — per-instance current loads.
    pub comps: Vec<Comp>,
}

const POWER_NAMES: &[&str] = &["VPWR", "VDD", "VCCD", "VCC", "VDDP"];

impl Def {
    /// The power net to analyze: `USE POWER`, else a known power name, else first.
    pub fn power_net(&self) -> Option<&NetGeom> {
        self.power_nets
            .iter()
            .find(|n| n.use_power)
            .or_else(|| self.power_nets.iter().find(|n| POWER_NAMES.contains(&n.name.as_str())))
            .or_else(|| self.power_nets.first())
    }

    pub fn parse(text: &str) -> Result<Def, DefError> {
        let tv = tokenize(text);
        let scale = units(&tv);
        let ndr = parse_ndr(&tv, scale);
        let nets = parse_signal(&tv, scale, &ndr)?;
        let tref: Vec<&str> = tv.iter().map(|s| s.as_str()).collect();
        let power_nets = match tref.iter().position(|&t| t == "SPECIALNETS") {
            Some(s) => {
                let end = (s..tref.len())
                    .find(|&i| tref[i] == "END" && tref.get(i + 1) == Some(&"SPECIALNETS"))
                    .unwrap_or(tref.len());
                parse_specialnets(&tref[s + 1..end])
            }
            None => Vec::new(),
        };
        let comps = parse_components(&tref);
        Ok(Def { units_per_um: scale, dbu: scale, nets, power_nets, comps })
    }

    pub fn load(path: &str) -> Result<Def, DefError> {
        let text = std::fs::read_to_string(path).map_err(|e| DefError(format!("{path}: {e}")))?;
        Def::parse(&text)
    }
}

#[derive(Debug)]
pub struct DefError(pub String);
impl std::fmt::Display for DefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "def error: {}", self.0)
    }
}
impl std::error::Error for DefError {}

/// Module-level `parse`/`load` (extraction historically called `def::parse`).
pub fn parse(text: &str) -> Result<Def, DefError> {
    Def::parse(text)
}
pub fn load(path: &str) -> Result<Def, DefError> {
    Def::load(path)
}

// ─────────────────────────── shared tokenize / units ───────────────────────────

/// Tokenize DEF, treating `(`, `)`, and `;` as standalone tokens.
fn tokenize(text: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let flush = |cur: &mut String, out: &mut Vec<String>| {
        if !cur.is_empty() {
            out.push(std::mem::take(cur));
        }
    };
    for ch in text.chars() {
        match ch {
            '(' | ')' | ';' => {
                flush(&mut cur, &mut out);
                out.push(ch.to_string());
            }
            c if c.is_whitespace() => flush(&mut cur, &mut out),
            c => cur.push(c),
        }
    }
    flush(&mut cur, &mut out);
    out
}

/// `UNITS DISTANCE MICRONS <n>` → n (default 1000).
fn units(t: &[String]) -> f64 {
    for w in t.windows(4) {
        if w[0] == "UNITS" && w[1] == "DISTANCE" && w[2] == "MICRONS" {
            if let Ok(n) = w[3].trim_end_matches(';').parse::<f64>() {
                return n;
            }
        }
    }
    1000.0
}

// ─────────────────────────── signal pass (extraction) ──────────────────────────

fn is_decoration(tok: &str) -> bool {
    matches!(tok, "TAPER" | "TAPERRULE" | "RECT" | "MASK" | "STYLE" | "VIRTUAL" | "ORIENT")
}

fn coord(tok: &str, prev: f64, scale: f64) -> Result<f64, DefError> {
    if tok == "*" {
        Ok(prev)
    } else {
        tok.parse::<f64>()
            .map(|v| v / scale)
            .map_err(|_| DefError(format!("bad coordinate {tok:?}")))
    }
}

/// Parse the `NONDEFAULTRULES` section into `rule -> layer -> width (µm)`. A net or
/// wire that references one of these rules draws wider/narrower than the default, so
/// its resistance differs — the extractor reads the width off each segment.
fn parse_ndr(t: &[String], scale: f64) -> BTreeMap<String, BTreeMap<String, f64>> {
    let mut out: BTreeMap<String, BTreeMap<String, f64>> = BTreeMap::new();
    let Some(start) = t.iter().position(|x| x == "NONDEFAULTRULES") else { return out };
    let end = (start..t.len())
        .find(|&i| t[i] == "END" && t.get(i + 1).map(String::as_str) == Some("NONDEFAULTRULES"))
        .unwrap_or(t.len());
    let mut rule: Option<String> = None;
    let mut pend_layer: Option<String> = None;
    let mut i = start + 1;
    while i < end {
        match t[i].as_str() {
            "-" => {
                rule = t.get(i + 1).cloned();
                pend_layer = None;
                i += 2;
            }
            "LAYER" => {
                pend_layer = t.get(i + 1).cloned();
                i += 2;
            }
            "WIDTH" => {
                if let (Some(r), Some(l)) = (&rule, &pend_layer) {
                    if let Some(w) =
                        t.get(i + 1).and_then(|s| s.trim_end_matches(';').parse::<f64>().ok())
                    {
                        out.entry(r.clone()).or_default().insert(l.clone(), w / scale);
                    }
                }
                pend_layer = None;
                i += 2;
            }
            _ => i += 1,
        }
    }
    out
}

fn parse_signal(
    t: &[String],
    scale: f64,
    ndr: &BTreeMap<String, BTreeMap<String, f64>>,
) -> Result<Vec<DefNet>, DefError> {
    let mut nets = Vec::new();
    let mut i = match t.iter().position(|x| x == "NETS") {
        Some(p) => p,
        None => return Ok(nets), // no signal nets
    };
    while i < t.len() && t[i] != ";" {
        i += 1;
    }
    i += 1;

    while i < t.len() {
        if t[i] == "END" {
            break;
        }
        if t[i] != "-" {
            i += 1;
            continue;
        }
        i += 1; // consume '-'
        let name = t.get(i).cloned().unwrap_or_default();
        i += 1;

        let mut net = DefNet { name, pins: Vec::new(), segments: Vec::new(), vias: 0 };
        let mut in_routing = false;
        let mut layer: Option<String> = None;
        let mut prev: Option<(f64, f64)> = None;
        // non-default routing rule: net-level (`+ NONDEFAULTRULE r`) or per-wire
        // (`TAPERRULE r`); the per-wire override wins while it is in effect.
        let mut net_rule: Option<String> = None;
        let mut wire_rule: Option<String> = None;
        // width (µm) for the current layer under the effective rule (0 = default)
        let width_of = |layer: &Option<String>, wire: &Option<String>, net: &Option<String>| {
            let rule = wire.as_ref().or(net.as_ref());
            match (rule, layer) {
                (Some(r), Some(l)) => ndr.get(r).and_then(|m| m.get(l)).copied().unwrap_or(0.0),
                _ => 0.0,
            }
        };

        while i < t.len() && t[i] != ";" {
            match t[i].as_str() {
                "+" => {
                    let status = t.get(i + 1).map(String::as_str).unwrap_or("");
                    if matches!(status, "ROUTED" | "FIXED" | "COVER" | "NOSHIELD") {
                        in_routing = true;
                        layer = t.get(i + 2).cloned();
                        prev = None;
                        wire_rule = None;
                        i += 3;
                    } else if status == "NONDEFAULTRULE" {
                        net_rule = t.get(i + 2).cloned();
                        i += 3;
                    } else {
                        i += 1;
                    }
                }
                "NEW" => {
                    layer = t.get(i + 1).cloned();
                    prev = None;
                    wire_rule = None;
                    i += 2;
                }
                "TAPERRULE" => {
                    wire_rule = t.get(i + 1).cloned();
                    i += 2;
                }
                "(" => {
                    let mut j = i + 1;
                    let mut inner = Vec::new();
                    while j < t.len() && t[j] != ")" {
                        inner.push(t[j].clone());
                        j += 1;
                    }
                    if !in_routing {
                        if inner.len() >= 2 {
                            net.pins.push((inner[0].clone(), inner[1].clone()));
                        }
                    } else if inner.len() >= 2 {
                        let (px, py) = prev.unwrap_or((0.0, 0.0));
                        let x = coord(&inner[0], px, scale)?;
                        let y = coord(&inner[1], py, scale)?;
                        if let (Some(l), Some((ox, oy))) = (&layer, prev) {
                            if (x - ox).abs() + (y - oy).abs() > 0.0 {
                                net.segments.push(Segment {
                                    layer: l.clone(),
                                    x0: ox,
                                    y0: oy,
                                    x1: x,
                                    y1: y,
                                    width_um: width_of(&layer, &wire_rule, &net_rule),
                                });
                            }
                        }
                        prev = Some((x, y));
                    }
                    i = j + 1;
                }
                tok if is_decoration(tok) => {
                    i += 1;
                }
                _ => {
                    if in_routing {
                        net.vias += 1;
                    }
                    i += 1;
                }
            }
        }
        nets.push(net);
        i += 1; // past ';'
    }
    Ok(nets)
}

// ─────────────────────────── power / components pass (PDN) ──────────────────────

fn parse_components(toks: &[&str]) -> Vec<Comp> {
    let Some(s) = toks.iter().position(|&t| t == "COMPONENTS") else {
        return Vec::new();
    };
    let end = (s..toks.len())
        .find(|&i| toks[i] == "END" && toks.get(i + 1) == Some(&"COMPONENTS"))
        .unwrap_or(toks.len());
    let body = &toks[s + 1..end];
    let mut comps = Vec::new();
    let mut i = 0;
    while i < body.len() {
        if body[i] == "-" {
            let name = body.get(i + 1).copied().unwrap_or("").to_string();
            let cell = body.get(i + 2).copied().unwrap_or("").to_string();
            let mut j = i + 3;
            let mut xy = None;
            while j < body.len() && body[j] != ";" {
                if (body[j] == "PLACED" || body[j] == "FIXED") && body.get(j + 1) == Some(&"(") {
                    let x = body.get(j + 2).and_then(|t| t.parse().ok());
                    let y = body.get(j + 3).and_then(|t| t.parse().ok());
                    if let (Some(x), Some(y)) = (x, y) {
                        xy = Some((x, y));
                    }
                    break;
                }
                j += 1;
            }
            if let Some((x, y)) = xy {
                if !name.is_empty() && !cell.is_empty() {
                    comps.push(Comp { name, cell, x, y });
                }
            }
            i += 3;
        } else {
            i += 1;
        }
    }
    comps
}

fn parse_specialnets(body: &[&str]) -> Vec<NetGeom> {
    let mut nets: Vec<NetGeom> = Vec::new();
    let mut cur: Option<NetGeom> = None;
    let mut layer = String::new();
    let mut width = 0.0f64;
    let mut last: Option<(i64, i64)> = None;
    let mut i = 0;
    while i < body.len() {
        let t = body[i];
        match t {
            "-" => {
                if let Some(n) = cur.take() {
                    nets.push(n);
                }
                let name = body.get(i + 1).copied().unwrap_or("").to_string();
                cur = Some(NetGeom { name, ..Default::default() });
                last = None;
                i += 2;
            }
            ";" => {
                last = None;
                i += 1;
            }
            "USE" => {
                if body.get(i + 1) == Some(&"POWER") {
                    if let Some(n) = cur.as_mut() {
                        n.use_power = true;
                    }
                }
                i += 2;
            }
            "ROUTED" | "NEW" => {
                layer = body.get(i + 1).copied().unwrap_or("").to_string();
                width = body.get(i + 2).and_then(|w| w.parse().ok()).unwrap_or(0.0);
                last = None;
                i += 3;
            }
            "(" => {
                let xr = body.get(i + 1).copied().unwrap_or("0");
                let yr = body.get(i + 2).copied().unwrap_or("0");
                let prev = last.unwrap_or((0, 0));
                let px_ok = xr == "*" || xr.parse::<i64>().is_ok();
                let py_ok = yr == "*" || yr.parse::<i64>().is_ok();
                let mut j = i + 1;
                while j < body.len() && body[j] != ")" {
                    j += 1;
                }
                let next_i = j + 1;
                if !px_ok || !py_ok {
                    i = next_i;
                    continue;
                }
                let x = if xr == "*" { prev.0 } else { xr.parse().unwrap_or(0) };
                let y = if yr == "*" { prev.1 } else { yr.parse().unwrap_or(0) };
                i = next_i;
                if let Some(n) = cur.as_mut() {
                    if !layer.is_empty() {
                        n.points.push((layer.clone(), x, y));
                    }
                    if let Some((px, py)) = last {
                        if px != x || py != y {
                            n.segs.push(Seg {
                                layer: layer.clone(),
                                width_dbu: width,
                                x1: px,
                                y1: py,
                                x2: x,
                                y2: y,
                            });
                        }
                    }
                }
                last = Some((x, y));
            }
            "+" => i += 1,
            other => {
                if other.chars().next().map(|c| c.is_ascii_alphabetic()).unwrap_or(false) {
                    if let (Some(n), Some(p)) = (cur.as_mut(), last) {
                        if !is_qualifier(other) {
                            n.vias.push(p);
                        }
                    }
                }
                i += 1;
            }
        }
    }
    if let Some(n) = cur.take() {
        nets.push(n);
    }
    nets
}

fn is_qualifier(t: &str) -> bool {
    matches!(
        t,
        "SHAPE" | "STRIPE" | "FOLLOWPIN" | "STYLE" | "FIXED" | "COVER" | "POWER" | "GROUND"
            | "RECT" | "PIN" | "MASK" | "RING" | "BLOCKWIRE" | "PADRING" | "BLOCKAGEWIRE"
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_nets_in_microns() {
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
NETS 1 ;
- n1 ( u1 A ) ( u2 Z )
  + ROUTED met1 ( 0 0 ) ( 1000 0 ) via1 ( 1000 0 ) ( 1000 500 ) ;
END NETS
";
        let d = Def::parse(def).unwrap();
        assert_eq!(d.units_per_um, 1000.0);
        assert_eq!(d.nets.len(), 1);
        let n = &d.nets[0];
        assert_eq!(n.name, "n1");
        assert_eq!(n.pins.len(), 2);
        assert!(n.vias >= 1);
        assert!(n.segments.iter().any(|s| (s.len_um() - 1.0).abs() < 1e-9)); // 1000 dbu = 1 um
        assert!(n.segments.iter().all(|s| s.width_um == 0.0), "default width without an NDR");
    }

    #[test]
    fn nondefault_rule_sets_segment_width() {
        // a clock net on a 2x-wide non-default rule: segments carry the NDR width;
        // a TAPERRULE reference (which used to be miscounted as a via) is honoured.
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
NONDEFAULTRULES 1 ;
- DBL
  + LAYER met1 WIDTH 280
  + LAYER met2 WIDTH 280 ;
END NONDEFAULTRULES
NETS 2 ;
- clk ( u1 A ) ( u2 Z )
  + NONDEFAULTRULE DBL
  + ROUTED met1 ( 0 0 ) ( 1000 0 ) ;
- sig ( u3 A )
  + ROUTED met1 TAPERRULE DBL ( 0 0 ) ( 1000 0 ) ;
END NETS
";
        let d = Def::parse(def).unwrap();
        let clk = d.nets.iter().find(|n| n.name == "clk").unwrap();
        assert!(clk.segments.iter().all(|s| (s.width_um - 0.28).abs() < 1e-9), "280 dbu = 0.28 um");
        let sig = d.nets.iter().find(|n| n.name == "sig").unwrap();
        assert!((sig.segments[0].width_um - 0.28).abs() < 1e-9, "TAPERRULE width applied");
        assert_eq!(sig.vias, 0, "the TAPERRULE rule name must not be miscounted as a via");
    }

    #[test]
    fn special_nets_and_components_in_dbu() {
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
COMPONENTS 1 ;
- u1 INV_X1 + PLACED ( 100 200 ) N ;
END COMPONENTS
SPECIALNETS 1 ;
- VDD + USE POWER + ROUTED met5 1600 ( 0 0 ) ( 5000 0 ) ;
END SPECIALNETS
";
        let d = Def::parse(def).unwrap();
        assert_eq!(d.dbu, 1000.0);
        assert_eq!(d.comps.len(), 1);
        assert_eq!(d.comps[0].cell, "INV_X1");
        let p = d.power_net().expect("power net");
        assert!(p.use_power);
        assert_eq!(p.name, "VDD");
        assert!(!p.segs.is_empty());
    }

    #[test]
    fn empty_is_lenient() {
        let d = Def::parse("DESIGN top ;\n").unwrap();
        assert!(d.nets.is_empty() && d.power_nets.is_empty() && d.comps.is_empty());
        assert!(d.power_net().is_none());
    }
}
