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

use crate::names::unescape;
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
        Segment {
            layer: layer.into(),
            x0,
            y0,
            x1,
            y1,
            width_um: 0.0,
        }
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

/// A via placement inside a net's routing.
///
/// DEF states a via as a **point** plus the name of a LEF via, declared while routing on
/// `layer` — `NEW met1 ( 230690 1040230 ) M1M2_PR`. That point is a connection between two
/// layers, and it need not be an endpoint of the wire on the layer the via reaches: it very
/// often lands **mid-span** of it.
///
/// So the location is load-bearing. Keeping only a count — which is all this reader used to do
/// — leaves a consumer no way to know that a wire has to be split there, and the branch beyond
/// the via silently becomes a separate, unreachable piece of the net.
#[derive(Debug, Clone)]
pub struct ViaPoint {
    pub x: f64,
    pub y: f64,
    /// The routing layer in effect where the via was declared.
    pub layer: String,
    /// The LEF via name, e.g. `M1M2_PR`.
    pub name: String,
}

#[derive(Debug, Clone, Default)]
pub struct DefNet {
    pub name: String,
    /// The name **exactly as the DEF spelled it**, escaping intact — `CFG_REG\[0\]`, not
    /// `CFG_REG[0]`.
    ///
    /// [`name`](Self::name) is unescaped so it joins against the netlist and the SPEF, and that
    /// is the right canonical form for matching. But unescaping DESTROYS information a writer
    /// needs: `count[0]` and `CFG_REG\[0\]` unescape to names that look alike, while meaning
    /// different things — bit 0 of a bus, versus a scalar net whose own name contains brackets
    /// (what synthesis produces when it flattens a bus into `wire \CFG_REG[0] ;`).
    ///
    /// Nothing downstream can recover the distinction from the canonical name alone, so a writer
    /// that re-derives escaping gets it wrong in one direction or the other. vyges-extract emits
    /// this field verbatim into its SPEF name map; DEF and SPEF escape the same way, so
    /// round-tripping the source spelling is both correct and lossless.
    pub raw_name: String,
    pub pins: Vec<(String, String)>, // (instance, pin)
    pub segments: Vec<Segment>,
    pub vias: usize,
    /// Where each of those vias sits. See [`ViaPoint`].
    pub via_points: Vec<ViaPoint>,
}

impl DefNet {
    /// The name to WRITE into a downstream format: the source spelling when we have one,
    /// otherwise the canonical name.
    ///
    /// The fallback is what makes this safe for nets that never came from a DEF at all — the
    /// `gds → DefNet` front-end synthesises nets from traced geometry, so there is no original
    /// spelling to preserve and `raw_name` is empty. Reading the field directly would write an
    /// empty name into a SPEF name map there; a reader would take the whole file as junk.
    pub fn write_name(&self) -> &str {
        if self.raw_name.is_empty() {
            &self.name
        } else {
            &self.raw_name
        }
    }
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
    /// The same vias, each with the name of the via definition that placed it.
    ///
    /// A via's resistance is the cut layer's per-cut resistance divided by the number of
    /// cuts, and **neither is knowable from the point alone** — both live in the `VIAS`
    /// definition this names (see [`ViaDef`]). Parallel to `vias`, same order.
    pub via_names: Vec<(String, i64, i64)>,
    /// Every listed coordinate with its wire layer (incl. via-only landings).
    pub points: Vec<(String, i64, i64)>,
}

/// A block terminal from the DEF `PINS` section, with its port shapes in absolute DBU.
///
/// This is where the design says **the supply actually enters the die**. A PDN analysis that
/// treats the whole top metal as an ideal source instead of these shapes understates IR drop,
/// because it removes resistance the real supply has to cross.
///
/// The rectangles as written are relative to the pin's `PLACED`/`FIXED` origin; `shapes` holds
/// them already oriented and translated, so a consumer never repeats that transform.
#[derive(Debug, Clone, Default)]
pub struct DefPin {
    pub name: String,
    /// The net it belongs to (`+ NET <name>`), which is what ties it to a power net.
    pub net: String,
    /// `+ USE POWER` or `+ USE GROUND`.
    pub use_power: bool,
    pub use_ground: bool,
    /// `(layer, x1, y1, x2, y2)` per port rectangle, absolute and normalised so x1<=x2.
    pub shapes: Vec<(String, i64, i64, i64, i64)>,
}

/// A DEF `VIAS` definition — what a via placement actually refers to.
///
/// The name carries no reliable information: a via called `via2_3_…` may bridge met1→met2.
/// `LAYERS <below> <cut> <above>` is authoritative for the cut layer, and `ROWCOL <rows>
/// <cols>` for the cut count (absent means a single cut).
#[derive(Debug, Clone, Default)]
pub struct ViaDef {
    pub name: String,
    /// The cut layer — the middle entry of `LAYERS`, whose LEF `RESISTANCE` is per cut.
    pub cut_layer: String,
    /// The routing layers it bridges (`LAYERS` first and last).
    pub below: String,
    pub above: String,
    /// Cuts in the array: `ROWCOL rows cols` multiplied out, else 1.
    pub cuts: u32,
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
    /// `VIAS` definitions, by name — the cut layer and cut count behind each via placement.
    pub via_defs: BTreeMap<String, ViaDef>,
    /// `PINS` — block terminals with their port shapes, absolute. Where supply enters the die.
    pub pins: Vec<DefPin>,
}

const POWER_NAMES: &[&str] = &["VPWR", "VDD", "VCCD", "VCC", "VDDP"];

impl Def {
    /// The power net to analyze: `USE POWER`, else a known power name, else first.
    pub fn power_net(&self) -> Option<&NetGeom> {
        self.power_nets
            .iter()
            .find(|n| n.use_power)
            .or_else(|| {
                self.power_nets
                    .iter()
                    .find(|n| POWER_NAMES.contains(&n.name.as_str()))
            })
            .or_else(|| self.power_nets.first())
    }

    pub fn parse(text: &str) -> Result<Def, DefError> {
        let tv = tokenize(text);
        let scale = units(&tv);
        let ndr = parse_ndr(&tv, scale);
        let nets = parse_signal(&tv, scale, &ndr)?;
        let tref: Vec<&str> = tv.iter().map(|s| s.as_str()).collect();
        let power_nets = match section_start(&tref, "SPECIALNETS") {
            Some(s) => parse_specialnets(&tref[s + 1..section_end(&tref, "SPECIALNETS", s)]),
            None => Vec::new(),
        };
        let comps = parse_components(&tref);
        let via_defs = parse_vias(&tref);
        let pins = parse_pins(&tref);
        Ok(Def {
            units_per_um: scale,
            dbu: scale,
            nets,
            power_nets,
            comps,
            via_defs,
            pins,
        })
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
    matches!(
        tok,
        "TAPER" | "TAPERRULE" | "RECT" | "MASK" | "STYLE" | "VIRTUAL" | "ORIENT"
    )
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
    let Some(start) = t.iter().position(|x| x == "NONDEFAULTRULES") else {
        return out;
    };
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
                    if let Some(w) = t
                        .get(i + 1)
                        .and_then(|s| s.trim_end_matches(';').parse::<f64>().ok())
                    {
                        out.entry(r.clone())
                            .or_default()
                            .insert(l.clone(), w / scale);
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
    let tref: Vec<&str> = t.iter().map(String::as_str).collect();
    let mut i = match section_start(&tref, "NETS") {
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
        // ESCAPED, LIKE EVERY OTHER NAME IN THE FLOW. DEF writes a bussed net as
        // `CFG_REG\[0\]` because `[` is its BUSBITCHARS delimiter. Left escaped, the name
        // matches nothing anywhere else: the SPEF extracted from this very DEF calls the same
        // net `CFG_REG[0]`, and so does the netlist. On real designs that was 10-20% of nets —
        // every bussed one — joining to nothing, with both readers reporting success.
        // Both forms are kept: `name` canonical (so it joins), `raw_name` as written (so a writer
        // can reproduce the escaping it cannot re-derive). See [`DefNet::raw_name`].
        let raw_name = t.get(i).cloned().unwrap_or_default();
        let name = unescape(&raw_name);
        i += 1;

        let mut net = DefNet {
            name,
            raw_name,
            ..DefNet::default()
        };
        let mut in_routing = false;
        // Has any `+` attribute been seen? Past the first one, no group is a connection.
        let mut seen_plus = false;
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
                    seen_plus = true;
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
                // `RECT ( dx1 dy1 dx2 dy2 )` is a patch of metal stated as an OFFSET
                // rectangle from the preceding point, not a route to somewhere. Skipping the
                // keyword alone leaves its parenthesised body to be read as a coordinate —
                // and since the offsets are small signed numbers, that draws a wire from the
                // routing point to somewhere near the origin. Two of those in one net meet
                // there and tie distant parts of it together, which is a loop in the
                // extracted RC. Skip the keyword AND its group.
                //
                // The patch's own area is not modelled; it is a via-landing enlargement,
                // typically a few tenths of a micron square. Dropping it understates that
                // net's capacitance slightly. Drawing a wire across the die does much worse.
                "RECT" => {
                    i += 1;
                    if t.get(i).map(String::as_str) == Some("(") {
                        while i < t.len() && t[i] != ")" {
                            i += 1;
                        }
                        i += 1; // past ')'
                    }
                }
                "(" => {
                    let mut j = i + 1;
                    let mut inner = Vec::new();
                    while j < t.len() && t[j] != ")" {
                        inner.push(t[j].clone());
                        j += 1;
                    }
                    // CONNECTIONS COME FIRST. A net statement lists its `( inst pin )`
                    // connections and only then its `+` attributes, so a parenthesised group
                    // after the first `+` belongs to that attribute — it is not a connection.
                    //
                    // Keying on "not yet routing" instead was silently wrong for any attribute
                    // that carries coordinates before the wiring does. `+ VPIN vp LAYER met2
                    // ( -50 -50 ) ( 50 50 ) PLACED ( 3000 3000 ) N` is valid DEF 5.8, and gave
                    // the net three extra pins named after its own coordinates — connectivity
                    // invented out of geometry, which extraction then builds an RC network to.
                    if !seen_plus {
                        if inner.len() >= 2 {
                            net.pins.push((unescape(&inner[0]), unescape(&inner[1])));
                        }
                    } else if in_routing && inner.len() >= 2 {
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
                        // `prev` is the point the via sits at: DEF writes the coordinate first
                        // and the via name immediately after it.
                        if let (Some((x, y)), Some(l)) = (prev, &layer) {
                            net.via_points.push(ViaPoint {
                                x,
                                y,
                                layer: l.clone(),
                                name: t[i].clone(),
                            });
                        }
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

/// Find a DEF **section header**, not merely the keyword.
///
/// A section opens as `KEYWORD <count> ;` and closes as `END KEYWORD`. Scanning the token
/// stream for the bare keyword instead finds it wherever it occurs — as a property name, a
/// component or net name, an entry in `PROPERTYDEFINITIONS`. The result is not a parse error:
/// the reader takes the boundary from the wrong place and returns a different design without
/// complaint. Measured: a `PROPERTYDEFINITIONS` entry naming `SPECIALNETS` made the COMPONENTS
/// section parse as the power grid, inventing a power net per instance — and the power grid is
/// what the extractor uses to decide which wires are shielded from each other.
///
/// Requiring the header shape costs two token comparisons and removes the whole class.
fn section_start(toks: &[&str], name: &str) -> Option<usize> {
    (0..toks.len()).find(|&i| {
        toks[i] == name
            && toks
                .get(i + 1)
                .is_some_and(|c| c.parse::<i64>().is_ok())
            && toks.get(i + 2) == Some(&";")
    })
}

/// The matching `END <name>`, searched from the header.
fn section_end(toks: &[&str], name: &str, from: usize) -> usize {
    (from..toks.len())
        .find(|&i| toks[i] == "END" && toks.get(i + 1) == Some(&name))
        .unwrap_or(toks.len())
}

fn parse_components(toks: &[&str]) -> Vec<Comp> {
    let Some(s) = section_start(toks, "COMPONENTS") else {
        return Vec::new();
    };
    let end = section_end(toks, "COMPONENTS", s);
    let body = &toks[s + 1..end];
    let mut comps = Vec::new();
    let mut i = 0;
    while i < body.len() {
        if body[i] == "-" {
            let name = unescape(body.get(i + 1).copied().unwrap_or(""));
            let cell = unescape(body.get(i + 2).copied().unwrap_or(""));
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

/// Apply a DEF orientation to a point, about the origin. The eight LEF/DEF orientations.
///
/// A rectangle's corners must both be transformed and then re-normalised: `S` maps a
/// lower-left corner to an upper-right one, so carrying the corners through unchanged
/// produces an inverted box that contains nothing.
fn orient_pt(x: i64, y: i64, orient: &str) -> (i64, i64) {
    match orient {
        "N" | "R0" => (x, y),
        "W" | "R90" => (-y, x),
        "S" | "R180" => (-x, -y),
        "E" | "R270" => (y, -x),
        "FN" | "MY" => (-x, y),
        "FS" | "MX" => (x, -y),
        "FW" | "MX90" | "MYR90" => (y, x),
        "FE" | "MY90" | "MXR90" => (-y, -x),
        _ => (x, y),
    }
}

/// Read the `PINS` section: block terminals and their port shapes, placed and oriented.
///
/// Shape: `- <name> + NET <net> + SPECIAL + DIRECTION … + USE POWER + PORT + LAYER <l> ( x1 y1 )
/// ( x2 y2 ) … + FIXED ( ox oy ) <orient> ;`. Several `LAYER` rectangles may share one `PORT`
/// and one placement, which is exactly how a power pin is written — so rectangles are collected
/// and the placement applied to all of them when the entry ends.
fn parse_pins(toks: &[&str]) -> Vec<DefPin> {
    let mut out = Vec::new();
    let Some(s) = section_start(toks, "PINS") else {
        return out;
    };
    let end = section_end(toks, "PINS", s);
    let mut i = s + 3; // past `PINS <n> ;`

    // rectangles as written (relative to the placement), flushed when the entry closes
    let mut cur: Option<DefPin> = None;
    let mut rel: Vec<(String, i64, i64, i64, i64)> = Vec::new();
    let mut origin = (0i64, 0i64);
    let mut orient = "N".to_string();

    let flush = |cur: &mut Option<DefPin>,
                 rel: &mut Vec<(String, i64, i64, i64, i64)>,
                 origin: &mut (i64, i64),
                 orient: &mut String,
                 out: &mut Vec<DefPin>| {
        if let Some(mut p) = cur.take() {
            for (layer, x1, y1, x2, y2) in rel.drain(..) {
                let (ax1, ay1) = orient_pt(x1, y1, orient);
                let (ax2, ay2) = orient_pt(x2, y2, orient);
                p.shapes.push((
                    layer,
                    ax1.min(ax2) + origin.0,
                    ay1.min(ay2) + origin.1,
                    ax1.max(ax2) + origin.0,
                    ay1.max(ay2) + origin.1,
                ));
            }
            out.push(p);
        }
        rel.clear();
        *origin = (0, 0);
        *orient = "N".to_string();
    };

    let num = |t: &str| t.parse::<i64>().ok();

    while i < end {
        match toks[i] {
            "-" => {
                flush(&mut cur, &mut rel, &mut origin, &mut orient, &mut out);
                i += 1;
                if i < end {
                    cur = Some(DefPin {
                        name: unescape(toks[i]),
                        ..Default::default()
                    });
                    i += 1;
                }
            }
            "NET" if cur.is_some() && i + 1 < end => {
                cur.as_mut().unwrap().net = unescape(toks[i + 1]);
                i += 2;
            }
            "USE" if cur.is_some() && i + 1 < end => {
                match toks[i + 1] {
                    "POWER" => cur.as_mut().unwrap().use_power = true,
                    "GROUND" => cur.as_mut().unwrap().use_ground = true,
                    _ => {}
                }
                i += 2;
            }
            // + LAYER <l> ( x1 y1 ) ( x2 y2 )
            //   tokens:   i+1  i+2 i+3 i+4 i+5 i+6 i+7 i+8 i+9
            //             <l>   (   x1  y1   )   (   x2  y2   )
            "LAYER" if cur.is_some() && i + 9 < end => {
                let layer = toks[i + 1].to_string();
                let c = [
                    num(toks[i + 3]),
                    num(toks[i + 4]),
                    num(toks[i + 7]),
                    num(toks[i + 8]),
                ];
                if toks[i + 2] == "(" && toks[i + 6] == "(" {
                    if let [Some(x1), Some(y1), Some(x2), Some(y2)] = c {
                        rel.push((layer, x1, y1, x2, y2));
                    }
                }
                i += 10;
            }
            // + PLACED|FIXED|COVER ( x y ) <orient>
            "PLACED" | "FIXED" | "COVER" if cur.is_some() && i + 5 < end => {
                if toks[i + 1] == "(" {
                    if let (Some(x), Some(y)) = (num(toks[i + 2]), num(toks[i + 3])) {
                        origin = (x, y);
                    }
                    orient = toks[i + 5].to_string();
                }
                i += 6;
            }
            _ => i += 1,
        }
    }
    flush(&mut cur, &mut rel, &mut origin, &mut orient, &mut out);
    out
}

/// Read the `VIAS` section: name -> cut layer + cut count.
///
/// Each entry runs `- <name> + VIARULE … + LAYERS <below> <cut> <above> + ROWCOL <r> <c> … ;`.
/// Only `LAYERS` and `ROWCOL` are read; the rest (cut size, spacing, enclosure) describes
/// geometry we do not model. A `ROWCOL`-less entry is a single cut, which is the DEF default
/// and not an omission.
fn parse_vias(toks: &[&str]) -> BTreeMap<String, ViaDef> {
    let mut out = BTreeMap::new();
    let Some(s) = section_start(toks, "VIAS") else {
        return out;
    };
    let end = section_end(toks, "VIAS", s);
    let mut i = s + 1;
    // skip the count that follows `VIAS`, and its `;`
    while i < end && (toks[i] == ";" || toks[i].parse::<i64>().is_ok()) {
        i += 1;
    }
    let mut cur: Option<ViaDef> = None;
    while i < end {
        match toks[i] {
            "-" => {
                if let Some(v) = cur.take() {
                    out.insert(v.name.clone(), v);
                }
                i += 1;
                if i < end {
                    cur = Some(ViaDef {
                        name: toks[i].to_string(),
                        cuts: 1,
                        ..Default::default()
                    });
                    i += 1;
                }
            }
            "LAYERS" if cur.is_some() => {
                if i + 3 < end {
                    let v = cur.as_mut().unwrap();
                    v.below = toks[i + 1].to_string();
                    v.cut_layer = toks[i + 2].to_string();
                    v.above = toks[i + 3].to_string();
                }
                i += 4;
            }
            "ROWCOL" if cur.is_some() => {
                if i + 2 < end {
                    let r: u32 = toks[i + 1].parse().unwrap_or(1);
                    let c: u32 = toks[i + 2].parse().unwrap_or(1);
                    cur.as_mut().unwrap().cuts = (r * c).max(1);
                }
                i += 3;
            }
            _ => i += 1,
        }
    }
    if let Some(v) = cur.take() {
        out.insert(v.name.clone(), v);
    }
    out
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
                let name = unescape(body.get(i + 1).copied().unwrap_or(""));
                cur = Some(NetGeom {
                    name,
                    ..Default::default()
                });
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
                let x = if xr == "*" {
                    prev.0
                } else {
                    xr.parse().unwrap_or(0)
                };
                let y = if yr == "*" {
                    prev.1
                } else {
                    yr.parse().unwrap_or(0)
                };
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
                if other
                    .chars()
                    .next()
                    .map(|c| c.is_ascii_alphabetic())
                    .unwrap_or(false)
                {
                    if let (Some(n), Some(p)) = (cur.as_mut(), last) {
                        if !is_qualifier(other) {
                            n.vias.push(p);
                            n.via_names.push((other.to_string(), p.0, p.1));
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
        "SHAPE"
            | "STRIPE"
            | "FOLLOWPIN"
            | "STYLE"
            | "FIXED"
            | "COVER"
            | "POWER"
            | "GROUND"
            | "RECT"
            | "PIN"
            | "MASK"
            | "RING"
            | "BLOCKWIRE"
            | "PADRING"
            | "BLOCKAGEWIRE"
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
        assert!(
            n.segments.iter().all(|s| s.width_um == 0.0),
            "default width without an NDR"
        );
    }

    /// Escaping is not noise on the way to a canonical name: it is the only thing separating two
    /// different objects. `count[0]` is bit 0 of bus `count`; `CFG_REG\[0\]` is a scalar net whose
    /// own name contains brackets, which is what synthesis emits when it flattens a bus into
    /// `wire \CFG_REG[0] ;`. Both unescape to something bracket-shaped, so a writer holding only
    /// the canonical name has to guess, and guessing is wrong half the time.
    ///
    /// So: `name` canonical for joining, `raw_name` verbatim for writing. Losing either one is a
    /// defect that shows up as a silently under-annotated SPEF, not as an error.
    #[test]
    fn a_nets_source_spelling_survives_the_canonical_name() {
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
NETS 3 ;
- count[0] ( u1 A )
  + ROUTED met1 ( 0 0 ) ( 1000 0 ) ;
- CFG_REG\\[0\\] ( u2 A )
  + ROUTED met1 ( 0 0 ) ( 1000 0 ) ;
- plain_net ( u3 A )
  + ROUTED met1 ( 0 0 ) ( 1000 0 ) ;
END NETS
";
        let d = Def::parse(def).unwrap();
        let by = |n: &str| d.nets.iter().find(|x| x.name == n).expect(n).clone();

        // a real bus bit: DEF did not escape it, so neither do we
        let bus = by("count[0]");
        assert_eq!(bus.raw_name, "count[0]");
        assert_eq!(bus.write_name(), "count[0]");

        // an escaped identifier: canonical name matches the netlist, source spelling preserved
        let esc = by("CFG_REG[0]");
        assert_eq!(esc.raw_name, "CFG_REG\\[0\\]", "the DEF's own spelling, intact");
        assert_eq!(esc.write_name(), "CFG_REG\\[0\\]");
        assert_ne!(
            esc.raw_name, esc.name,
            "if these were equal the distinction would already be lost"
        );

        // nothing to escape: both forms agree, and no backslash is invented
        let plain = by("plain_net");
        assert_eq!(plain.raw_name, "plain_net");
        assert_eq!(plain.write_name(), "plain_net");

        // the fallback: a net that never came from a DEF (the gds front-end builds these)
        let synth = DefNet {
            name: "traced_net".into(),
            ..DefNet::default()
        };
        assert_eq!(
            synth.write_name(),
            "traced_net",
            "an empty raw_name must fall back, never write an empty name"
        );
    }

    #[test]
    fn a_power_pin_yields_its_port_shapes_placed_and_oriented() {
        // The VPWR pin of a real sky130 block, verbatim: ONE pin, ONE port, NINE rectangles
        // across two layers, and a single FIXED placement they all share. This is where the
        // supply actually enters the die, and a PDN analysis that instead treats the whole top
        // metal as an ideal source removes resistance the real supply has to cross.
        let d = Def::parse(
            r#"
UNITS DISTANCE MICRONS 1000 ;
PINS 1 ;
    - VPWR + NET VPWR + SPECIAL + DIRECTION INOUT + USE POWER
      + PORT
        + LAYER met5 ( -344550 -800 ) ( 344550 800 )
        + LAYER met4 ( 285610 -616890 ) ( 287210 60870 )
      + FIXED ( 349830 627530 ) N ;
END PINS
END DESIGN
"#,
        )
        .unwrap();

        assert_eq!(d.pins.len(), 1);
        let p = &d.pins[0];
        assert_eq!(p.name, "VPWR");
        assert_eq!(p.net, "VPWR");
        assert!(p.use_power && !p.use_ground);
        assert_eq!(p.shapes.len(), 2, "every LAYER rect of the port is a shape");

        // Rectangles are written RELATIVE to the placement; the reader returns them absolute,
        // so a consumer never repeats the transform (or forgets to).
        assert_eq!(
            p.shapes[0],
            ("met5".to_string(), 5280, 626730, 694380, 628330),
            "met5 strap translated by the FIXED origin"
        );
        assert_eq!(
            p.shapes[1],
            ("met4".to_string(), 635440, 10640, 637040, 688400)
        );
    }

    #[test]
    fn a_rotated_pin_keeps_a_rectangle_that_contains_something() {
        // S maps a lower-left corner to an upper-right one. Carrying the corners through a
        // rotation unchanged yields x1 > x2 — a box that contains no point, so every source
        // node inside it silently disappears and the supply is left connected by nothing.
        let d = Def::parse(
            r#"
UNITS DISTANCE MICRONS 1000 ;
PINS 1 ;
    - VPWR + NET VPWR + USE POWER
      + PORT
        + LAYER met5 ( 100 200 ) ( 500 600 )
      + FIXED ( 1000 1000 ) S ;
END PINS
END DESIGN
"#,
        )
        .unwrap();

        let (layer, x1, y1, x2, y2) = d.pins[0].shapes[0].clone();
        assert_eq!(layer, "met5");
        assert!(x1 < x2 && y1 < y2, "the rectangle is normalised, not inverted");
        // R180 about the origin then translate: (-500,-600)..(-100,-200) + (1000,1000)
        assert_eq!((x1, y1, x2, y2), (500, 400, 900, 800));
    }

    #[test]
    fn a_via_definition_yields_its_cut_layer_and_cut_count() {
        // The four PDN via definitions of a real sky130 block, verbatim. Two things here are
        // not guessable from a via placement alone, and both are why this section is read:
        //
        //  - the NAME LIES. `via2_3_…` bridges met1->met2, not met2->met3. Only `LAYERS` is
        //    authoritative, and its MIDDLE entry is the cut layer whose LEF RESISTANCE is
        //    stated per cut.
        //  - the CUT COUNT comes from `ROWCOL rows cols`. A via array of 5 cuts has a fifth
        //    of one cut's resistance, so treating every via alike is wrong by that factor.
        //    The last entry has no ROWCOL, which is a single cut by DEF default.
        let d = Def::parse(
            r#"
UNITS DISTANCE MICRONS 1000 ;
VIAS 4 ;
    - via2_3_1600_480_1_5_320_320 + VIARULE M1M2_PR + CUTSIZE 150 150  + LAYERS met1 via met2  + CUTSPACING 170 170  + ENCLOSURE 85 165 55 85  + ROWCOL 1 5  ;
    - via3_4_1600_480_1_4_400_400 + VIARULE M2M3_PR + CUTSIZE 200 200  + LAYERS met2 via2 met3  + CUTSPACING 200 200  + ENCLOSURE 40 85 65 65  + ROWCOL 1 4  ;
    - via4_5_1600_480_1_4_400_400 + VIARULE M3M4_PR + CUTSIZE 200 200  + LAYERS met3 via3 met4  + CUTSPACING 200 200  + ENCLOSURE 90 60 100 65  + ROWCOL 1 4  ;
    - via5_6_1600_1600_1_1_1600_1600 + VIARULE M4M5_PR + CUTSIZE 800 800  + LAYERS met4 via4 met5  + CUTSPACING 800 800  + ENCLOSURE 400 190 310 400  ;
END VIAS
END DESIGN
"#,
        )
        .unwrap();

        assert_eq!(d.via_defs.len(), 4, "every VIAS entry is read");

        let v = &d.via_defs["via2_3_1600_480_1_5_320_320"];
        assert_eq!(v.cut_layer, "via", "the cut layer is the MIDDLE of LAYERS");
        assert_eq!(
            (v.below.as_str(), v.above.as_str()),
            ("met1", "met2"),
            "the name says via2_3 but it bridges met1->met2 — the name is not usable"
        );
        assert_eq!(v.cuts, 5, "ROWCOL 1 5");

        assert_eq!(d.via_defs["via3_4_1600_480_1_4_400_400"].cuts, 4);
        assert_eq!(d.via_defs["via3_4_1600_480_1_4_400_400"].cut_layer, "via2");
        assert_eq!(d.via_defs["via4_5_1600_480_1_4_400_400"].cuts, 4);
        assert_eq!(d.via_defs["via4_5_1600_480_1_4_400_400"].cut_layer, "via3");

        let single = &d.via_defs["via5_6_1600_1600_1_1_1600_1600"];
        assert_eq!(single.cut_layer, "via4");
        assert_eq!(
            single.cuts, 1,
            "no ROWCOL is one cut by DEF default, not an unknown"
        );
    }

    #[test]
    fn a_special_net_via_carries_the_name_that_defines_it() {
        // A point alone cannot price a via. Without the name there is no cut layer to look up
        // and no cut count to divide by, which is exactly the gap that made a PDN extractor
        // fall back to one flat resistance for every via on the die.
        let d = Def::parse(
            r#"
UNITS DISTANCE MICRONS 1000 ;
SPECIALNETS 1 ;
    - VPWR ( * VPWR ) + USE POWER
      + ROUTED met4 1600 ( 5000 10000 ) ( 60000 10000 )
      NEW met4 1600 ( 20000 10000 ) 0 via5_6_1600_1600_1_1_1600_1600
      NEW met4 1600 ( 40000 10000 ) 0 via5_6_1600_1600_1_1_1600_1600 ;
END SPECIALNETS
END DESIGN
"#,
        )
        .unwrap();

        let n = d.power_net().unwrap();
        assert_eq!(n.vias.len(), 2);
        assert_eq!(
            n.via_names.len(),
            n.vias.len(),
            "every via keeps the name of its definition"
        );
        assert!(
            n.via_names
                .iter()
                .all(|(nm, _, _)| nm == "via5_6_1600_1600_1_1_1600_1600"),
            "the via name is recorded, not the layer or a qualifier"
        );
        assert_eq!(
            (n.via_names[0].1, n.via_names[0].2),
            (20000, 10000),
            "and it stays attached to its own location"
        );
    }

    #[test]
    fn a_via_keeps_its_location_not_just_a_tally() {
        // Real geometry, from `_00768_` of an fft control block. The two vias here are the
        // whole point: `M1M2_PR` at (230690, 1040230) sits **mid-span** of the met2 run from
        // y=1035470 to y=1043460, so it is not an endpoint of anything on met2. A consumer
        // that only knows "this net has 2 vias" cannot join those layers, and the met1 branch
        // west of it becomes an unreachable island.
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
NETS 1 ;
- n1 ( a A ) ( b Y )
  + ROUTED met2 ( 230690 1035470 ) ( * 1043460 )
    NEW met1 ( 226550 1040230 ) ( 230690 * )
    NEW met1 ( 230690 1040230 ) M1M2_PR
    NEW li1 ( 226550 1040230 ) L1M1_PR_MR ;
END NETS
";
        let n = &Def::parse(def).unwrap().nets[0];
        assert_eq!(n.vias, 2);
        assert_eq!(
            n.via_points.len(),
            2,
            "every counted via keeps its location"
        );

        let m1m2 = n.via_points.iter().find(|v| v.name == "M1M2_PR").unwrap();
        assert!(
            (m1m2.x - 230.690).abs() < 1e-9,
            "microns, like the segments"
        );
        assert!((m1m2.y - 1040.230).abs() < 1e-9);
        assert_eq!(
            m1m2.layer, "met1",
            "the layer in effect where it was declared"
        );

        // and it is genuinely mid-span of the met2 wire — the case that motivated all this
        let met2 = n.segments.iter().find(|s| s.layer == "met2").unwrap();
        assert!(
            m1m2.y > met2.y0.min(met2.y1) && m1m2.y < met2.y0.max(met2.y1),
            "strictly inside, so it is an endpoint of nothing"
        );
    }

    #[test]
    fn a_rect_patch_is_not_a_route_to_the_origin() {
        // Real tail of an fft control net. `RECT ( 0 -150 390 150 )` is an offset rectangle
        // from the preceding point — a via-landing enlargement. Reading its body as a
        // coordinate draws a wire from (254.610, 899.980) to (0.000, -0.150), right across
        // the die; two of them in one net meet down there and tie distant parts of it
        // together, which shows up downstream as a loop in the extracted RC.
        let def = "\
UNITS DISTANCE MICRONS 1000 ;
NETS 1 ;
- n1 ( a A ) ( b Y )
  + ROUTED met3 ( 254380 899980 ) ( 254610 * )
    NEW met3 ( 254610 899980 ) RECT ( 0 -150 390 150 )
    NEW met3 ( 254380 1000620 ) RECT ( 0 -150 390 150 ) ;
END NETS
";
        let n = &Def::parse(def).unwrap().nets[0];
        assert_eq!(n.segments.len(), 1, "one real wire, and no phantom ones");
        assert!(
            n.segments
                .iter()
                .all(|s| s.x0 > 100.0 && s.x1 > 100.0 && s.y0 > 100.0 && s.y1 > 100.0),
            "nothing runs off toward the origin: {:?}",
            n.segments
                .iter()
                .map(|s| (s.x0, s.y0, s.x1, s.y1))
                .collect::<Vec<_>>()
        );
        assert_eq!(n.vias, 0, "a RECT patch is not a via either");
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
        assert!(
            clk.segments
                .iter()
                .all(|s| (s.width_um - 0.28).abs() < 1e-9),
            "280 dbu = 0.28 um"
        );
        let sig = d.nets.iter().find(|n| n.name == "sig").unwrap();
        assert!(
            (sig.segments[0].width_um - 0.28).abs() < 1e-9,
            "TAPERRULE width applied"
        );
        assert_eq!(
            sig.vias, 0,
            "the TAPERRULE rule name must not be miscounted as a via"
        );
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
