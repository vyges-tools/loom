//! Liberty (`.lib`) reader + NLDM bilinear interpolation.
//!
//! Reads the timing view the STA engine needs: per cell, each pin's direction
//! and input capacitance, and for each output-pin timing arc the four NLDM
//! tables (`cell_rise` / `cell_fall` / `rise_transition` / `fall_transition`).
//! `Table::lookup(slew, load)` does clamped bilinear interpolation over
//! (index_1 = input_net_transition, index_2 = total_output_net_capacitance).
//!
//! Tolerant of both the `vyges-char` emitter's form and foundry libs: cell and
//! template names may be quoted or bare. Pure std — fully unit-tested offline.

use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    In,
    Out,
    Inout,
    Other,
}

#[derive(Debug, Clone, Default)]
pub struct Table {
    pub index_1: Vec<f64>, // input slews
    pub index_2: Vec<f64>, // output loads
    pub values: Vec<Vec<f64>>, // values[i][j] over (slew_i, load_j)
}

#[derive(Debug, Clone)]
pub struct Arc {
    pub related_pin: String,
    pub sense: String,
    pub cell_rise: Table,
    pub cell_fall: Table,
    pub rise_transition: Table,
    pub fall_transition: Table,
    pub ccs: crate::ccs::CcsArc, // CCS current waveforms (empty if NLDM-only)
    // LVF (Liberty Variation Format): per-(slew,load) delay sigma. Empty -> no LVF;
    // POCV then falls back to the global pocv_sigma fraction.
    pub sigma_rise: Table,
    pub sigma_fall: Table,
}

/// A setup or hold constraint: rise/fall tables indexed by
/// (index_1 = related/clock transition, index_2 = constrained/data transition).
/// Evaluated by bilinear interpolation at the operating slews (like delay arcs),
/// not collapsed to a table-max — matching OpenSTA.
#[derive(Debug, Clone, Default)]
pub struct Constraint {
    pub rise: Table,
    pub fall: Table,
}

impl Constraint {
    /// Worst (max) of rise/fall, interpolated at the clock and data transitions.
    pub fn eval(&self, clock_slew: f64, data_slew: f64) -> f64 {
        self.rise.lookup(clock_slew, data_slew).max(self.fall.lookup(clock_slew, data_slew))
    }
}

/// CCS receiver capacitance on an input pin: the two-segment input load a driver
/// sees. C1 = effective cap over the first half of the input transition (static
/// gate cap); C2 = over the second half (Miller-inflated by the switching output).
/// Tables indexed by (input_net_transition, total_output_net_capacitance).
#[derive(Debug, Clone, Default)]
pub struct RecvCap {
    pub c1_rise: Table,
    pub c2_rise: Table,
    pub c1_fall: Table,
    pub c2_fall: Table,
}

impl RecvCap {
    /// Representative full-swing input load (pF): the mean of (C1+C2)/2 over the
    /// grid, averaged across rise/fall. The full-swing equivalent cap **including
    /// Miller** — larger than a NLDM-only static `capacitance`. v1 is a scalar
    /// (slew/load-resolved receiver load is future, once the fanin driver's output
    /// slew is known at load-accumulation time).
    pub fn effective_load(&self) -> f64 {
        let mean = |t: &Table| {
            let mut sum = 0.0;
            let mut n = 0usize;
            for row in &t.values {
                for &v in row {
                    sum += v;
                    n += 1;
                }
            }
            if n == 0 { None } else { Some(sum / n as f64) }
        };
        // average the two segments per edge, then the two edges; skip empty tables.
        let edge = |c1: &Table, c2: &Table| match (mean(c1), mean(c2)) {
            (Some(a), Some(b)) => Some((a + b) / 2.0),
            (Some(a), None) | (None, Some(a)) => Some(a),
            (None, None) => None,
        };
        let r = edge(&self.c1_rise, &self.c2_rise);
        let f = edge(&self.c1_fall, &self.c2_fall);
        match (r, f) {
            (Some(a), Some(b)) => (a + b) / 2.0,
            (Some(a), None) | (None, Some(a)) => a,
            (None, None) => 0.0,
        }
    }

    pub fn is_empty(&self) -> bool {
        self.c1_rise.values.is_empty()
            && self.c2_rise.values.is_empty()
            && self.c1_fall.values.is_empty()
            && self.c2_fall.values.is_empty()
    }
}

#[derive(Debug, Clone)]
pub struct Pin {
    pub name: String,
    pub direction: Dir,
    pub capacitance: f64,        // input capacitance in library units (timing/NLDM load axis)
    pub cap_f: f64,              // same capacitance in Farads (power: net-load summation)
    pub recv: Option<RecvCap>,   // CCS receiver model (input pins); None -> use `capacitance`
    pub clock: bool,             // `clock : true` — the cell's clock pin
    pub setup: Vec<Constraint>,  // setup constraint group(s) vs the clock
    pub hold: Vec<Constraint>,   // hold constraint group(s) vs the clock
    pub arcs: Vec<Arc>,          // delay arcs (e.g. CK->Q on a flop output)
}

impl Pin {
    /// The capacitive load this input pin presents to its driver (pF): the
    /// Miller-aware receiver load when characterized, else the static `capacitance`.
    pub fn load_cap(&self) -> f64 {
        match &self.recv {
            Some(r) if !r.is_empty() => r.effective_load(),
            _ => self.capacitance,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Cell {
    pub name: String,
    pub pins: BTreeMap<String, Pin>,
    pub is_seq: bool,                // has an `ff`/`latch` group
    pub clock_pin: Option<String>,   // the pin marked `clock : true`
    pub leakage_w: f64,              // cell_leakage_power → Watts (power)
    pub int_energy_j: f64,           // representative per-transition internal energy → Joules (power)
}

impl Cell {
    /// Input capacitance (Farads) of a pin — 0.0 if absent. (power)
    pub fn input_cap(&self, pin: &str) -> f64 {
        self.pins.get(pin).map(|p| p.cap_f).unwrap_or(0.0)
    }
    /// Output pins. (power)
    pub fn outputs(&self) -> impl Iterator<Item = &Pin> {
        self.pins.values().filter(|p| p.direction == Dir::Out)
    }
}

#[derive(Debug, Clone, Default)]
pub struct Lib {
    pub cells: BTreeMap<String, Cell>,
    pub voltage: f64, // nominal supply (V) — power; 0.0 if unknown
}

#[derive(Debug)]
pub struct LibError(pub String);
impl std::fmt::Display for LibError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "liberty error: {}", self.0)
    }
}
impl std::error::Error for LibError {}

impl Table {
    /// Clamped bilinear interpolation; edge-clamps rather than extrapolating.
    pub fn lookup(&self, slew: f64, load: f64) -> f64 {
        if self.values.is_empty() {
            return 0.0;
        }
        if self.index_1.is_empty() || self.index_2.is_empty() {
            return self.values[0][0];
        }
        let (i0, i1, tx) = bracket(&self.index_1, slew);
        let (j0, j1, ty) = bracket(&self.index_2, load);
        let v = |i: usize, j: usize| self.values[i][j];
        let a = v(i0, j0) * (1.0 - tx) + v(i1, j0) * tx;
        let b = v(i0, j1) * (1.0 - tx) + v(i1, j1) * tx;
        a * (1.0 - ty) + b * ty
    }
}

/// Return (lo, hi, frac) bracketing `v` in ascending grid `g`; clamps at edges.
fn bracket(g: &[f64], v: f64) -> (usize, usize, f64) {
    let n = g.len();
    if n == 1 {
        return (0, 0, 0.0);
    }
    if v <= g[0] {
        return (0, 1, 0.0);
    }
    if v >= g[n - 1] {
        return (n - 2, n - 1, 1.0);
    }
    for k in 0..n - 1 {
        if v <= g[k + 1] {
            let t = (v - g[k]) / (g[k + 1] - g[k]);
            return (k, k + 1, t);
        }
    }
    (n - 2, n - 1, 1.0)
}

// ---- parser ---------------------------------------------------------------

fn matching(b: &[u8], mut i: usize) -> usize {
    let mut depth = 0i32;
    while i < b.len() {
        match b[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    return i;
                }
            }
            _ => {}
        }
        i += 1;
    }
    b.len()
}

fn is_ident(c: u8) -> bool {
    c.is_ascii_alphanumeric() || c == b'_'
}

/// Next `kw ( args ) { body }` at/after `from`. Returns (args, body, after_idx).
fn next_block(s: &str, from: usize, kw: &str) -> Option<(String, String, usize)> {
    let b = s.as_bytes();
    let mut p = from;
    loop {
        let hit = s[p..].find(kw)? + p;
        // token boundary before kw
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + kw.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b'(' {
            let close_paren = s[q..].find(')')? + q;
            let args = s[q + 1..close_paren].trim().trim_matches('"').to_string();
            let mut r = close_paren + 1;
            while r < b.len() && b[r].is_ascii_whitespace() {
                r += 1;
            }
            if r < b.len() && b[r] == b'{' {
                let end = matching(b, r);
                return Some((args, s[r + 1..end].to_string(), end + 1));
            }
        }
        p = hit + kw.len();
    }
}

fn simple_attr(body: &str, key: &str) -> Option<String> {
    // matches `key : value ;`
    let b = body.as_bytes();
    let mut p = 0;
    loop {
        let hit = body[p..].find(key)? + p;
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + key.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b':' {
            let semi = body[q..].find(';')? + q;
            return Some(body[q + 1..semi].trim().trim_matches('"').to_string());
        }
        p = hit + key.len();
    }
}

fn floats(s: &str) -> Vec<f64> {
    s.split(',').filter_map(|t| t.trim().parse::<f64>().ok()).collect()
}

fn parse_table(body: &str) -> Table {
    // index_1/index_2 use paren+quote form: `index_1 ("0.01, 0.04");`
    let idx = |kw: &str| {
        next_paren_after(body, kw).map(|s| floats(&s.replace('"', ""))).unwrap_or_default()
    };
    let index_1 = idx("index_1");
    let index_2 = idx("index_2");
    // values ( "a, b", "c, d" ) — collect each quoted row
    let values = next_paren_after(body, "values")
        .map(|v| {
            let mut rows = Vec::new();
            let mut rest = v.as_str();
            while let Some(start) = rest.find('"') {
                let after = &rest[start + 1..];
                if let Some(endq) = after.find('"') {
                    rows.push(floats(&after[..endq]));
                    rest = &after[endq + 1..];
                } else {
                    break;
                }
            }
            rows
        })
        .unwrap_or_default();
    Table { index_1, index_2, values }
}

/// Content of the `( ... )` following `kw` (paren-matched), e.g. `values ( ... )`.
fn next_paren_after(s: &str, kw: &str) -> Option<String> {
    let b = s.as_bytes();
    let mut p = 0;
    loop {
        let hit = s[p..].find(kw)? + p;
        let before_ok = hit == 0 || !is_ident(b[hit - 1]);
        let mut q = hit + kw.len();
        while q < b.len() && b[q].is_ascii_whitespace() {
            q += 1;
        }
        if before_ok && q < b.len() && b[q] == b'(' {
            // paren-match
            let mut depth = 0i32;
            let mut r = q;
            while r < b.len() {
                match b[r] {
                    b'(' => depth += 1,
                    b')' => {
                        depth -= 1;
                        if depth == 0 {
                            return Some(s[q + 1..r].to_string());
                        }
                    }
                    _ => {}
                }
                r += 1;
            }
            return None;
        }
        p = hit + kw.len();
    }
}

fn parse_arc(timing_body: &str, skip_ccs: bool) -> Arc {
    let tbl = |name: &str| {
        next_block(timing_body, 0, name).map(|(_, body, _)| parse_table(&body)).unwrap_or_default()
    };
    Arc {
        related_pin: simple_attr(timing_body, "related_pin").unwrap_or_default(),
        sense: simple_attr(timing_body, "timing_sense").unwrap_or_else(|| "non_unate".into()),
        cell_rise: tbl("cell_rise"),
        cell_fall: tbl("cell_fall"),
        rise_transition: tbl("rise_transition"),
        fall_transition: tbl("fall_transition"),
        // CCS output_current waveforms — skipped (empty) for NLDM-only parses.
        ccs: if skip_ccs { crate::ccs::CcsArc::default() } else { parse_ccs(timing_body) },
        sigma_rise: tbl("ocv_sigma_cell_rise"),
        sigma_fall: tbl("ocv_sigma_cell_fall"),
    }
}

/// Parse CCS `output_current_rise`/`output_current_fall` waveforms from an arc.
fn parse_ccs(timing_body: &str) -> crate::ccs::CcsArc {
    crate::ccs::CcsArc {
        rise: parse_ccs_set(timing_body, "output_current_rise"),
        fall: parse_ccs_set(timing_body, "output_current_fall"),
    }
}

/// Collect every `vector (...) { ... }` under an output_current group.
fn parse_ccs_set(timing_body: &str, group: &str) -> Vec<crate::ccs::CcsWaveform> {
    let Some((_, gbody, _)) = next_block(timing_body, 0, group) else {
        return Vec::new();
    };
    let first = |kw: &str, b: &str| {
        next_paren_after(b, kw).map(|s| floats(&s.replace('"', ""))).unwrap_or_default()
    };
    let mut out = Vec::new();
    let mut at = 0;
    while let Some((_, vbody, after)) = next_block(&gbody, at, "vector") {
        let time = first("index_3", &vbody);
        let current = first("values", &vbody);
        if time.len() >= 2 && time.len() == current.len() {
            out.push(crate::ccs::CcsWaveform {
                in_slew: first("index_1", &vbody).first().copied().unwrap_or(0.0),
                out_cap: first("index_2", &vbody).first().copied().unwrap_or(0.0),
                ref_time: simple_attr(&vbody, "reference_time").and_then(|s| s.parse().ok()).unwrap_or(0.0),
                time,
                current,
            });
        }
        at = after;
    }
    out
}

/// Parse a setup/hold constraint group's rise/fall tables.
fn parse_constraint(timing_body: &str) -> Constraint {
    let tbl = |name: &str| {
        next_block(timing_body, 0, name).map(|(_, b, _)| parse_table(&b)).unwrap_or_default()
    };
    Constraint { rise: tbl("rise_constraint"), fall: tbl("fall_constraint") }
}

fn parse_pin(name: String, body: &str, cap_unit_f: f64, skip_ccs: bool) -> Pin {
    let direction = match simple_attr(body, "direction").as_deref() {
        Some("input") => Dir::In,
        Some("output") => Dir::Out,
        Some("inout") => Dir::Inout,
        _ => Dir::Other,
    };
    let capacitance =
        simple_attr(body, "capacitance").and_then(|s| s.parse().ok()).unwrap_or(0.0);
    let cap_f = capacitance * cap_unit_f;
    // CCS receiver capacitance group (input pins): the Miller-aware two-segment load.
    // Skipped for NLDM-only parses (consumers fall back to lumped Ceff).
    let recv = if skip_ccs {
        None
    } else {
        next_block(body, 0, "receiver_capacitance").map(|(_, rbody, _)| {
            let tbl = |name: &str| {
                next_block(&rbody, 0, name).map(|(_, b, _)| parse_table(&b)).unwrap_or_default()
            };
            RecvCap {
                c1_rise: tbl("receiver_capacitance1_rise"),
                c2_rise: tbl("receiver_capacitance2_rise"),
                c1_fall: tbl("receiver_capacitance1_fall"),
                c2_fall: tbl("receiver_capacitance2_fall"),
            }
        })
    };
    let clock = simple_attr(body, "clock").as_deref() == Some("true");
    let mut arcs = Vec::new();
    let mut setup: Vec<Constraint> = Vec::new();
    let mut hold: Vec<Constraint> = Vec::new();
    let mut at = 0;
    while let Some((_, tbody, after)) = next_block(body, at, "timing") {
        match simple_attr(&tbody, "timing_type").as_deref() {
            Some(tt) if tt.starts_with("setup") => setup.push(parse_constraint(&tbody)),
            Some(tt) if tt.starts_with("hold") => hold.push(parse_constraint(&tbody)),
            // async set/reset (clear/preset) and check arcs (recovery/removal/
            // pulse_width) are NOT max-delay data arcs — don't propagate data through
            // them (e.g. dfrtp RESET_B->Q is an async clear, not a launch path).
            Some(tt)
                if tt.starts_with("clear")
                    || tt.starts_with("preset")
                    || tt.starts_with("recovery")
                    || tt.starts_with("removal")
                    || tt.contains("pulse_width") => {}
            _ => arcs.push(parse_arc(&tbody, skip_ccs)), // delay arc (incl. rising_edge CK->Q)
        }
        at = after;
    }
    Pin { name, direction, capacitance, cap_f, recv, clock, setup, hold, arcs }
}

fn parse_cell(name: String, body: &str, units: &Units, skip_ccs: bool) -> Cell {
    let mut pins = BTreeMap::new();
    let mut at = 0;
    while let Some((pname, pbody, after)) = next_block(body, at, "pin") {
        let pin = parse_pin(pname.clone(), &pbody, units.cap_f, skip_ccs);
        pins.insert(pname, pin);
        at = after;
    }
    let is_seq = next_block(body, 0, "ff").is_some() || next_block(body, 0, "latch").is_some();
    let clock_pin = pins.iter().find(|(_, p)| p.clock).map(|(n, _)| n.clone());
    // power: leakage + representative internal (switching) energy.
    let leakage_w = simple_attr(body, "cell_leakage_power")
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(0.0)
        * units.leak_w;
    let ivals = internal_values(body);
    let int_energy_j = if ivals.is_empty() {
        0.0
    } else {
        (ivals.iter().sum::<f64>() / ivals.len() as f64) * units.energy_j
    };
    Cell { name, pins, is_seq, clock_pin, leakage_w, int_energy_j }
}

/// Options controlling how much of a Liberty file is parsed.
#[derive(Clone, Copy, Debug, Default)]
pub struct LibOpts {
    /// Skip CCS `receiver_capacitance` + `output_current` groups at parse time.
    /// For NLDM-only runs (cell delay/transition tables) this cuts parse time and
    /// peak memory on large multi-corner libs. Consuming engines then fall back to
    /// the NLDM delay path + lumped Ceff, so results match a full-CCS load only when
    /// CCS was not going to be used — otherwise it is a deliberate speed/accuracy
    /// trade the caller opts into (never the default).
    pub skip_ccs: bool,
}

impl Lib {
    pub fn parse(text: &str) -> Result<Lib, LibError> {
        Lib::parse_opts(text, LibOpts::default())
    }

    /// Like [`Lib::parse`] but honoring [`LibOpts`] (e.g. `skip_ccs` for NLDM-only).
    pub fn parse_opts(text: &str, opts: LibOpts) -> Result<Lib, LibError> {
        let units = Units::from_lib(text);
        let voltage = lib_voltage(text).unwrap_or(1.8);
        let mut cells = BTreeMap::new();
        let mut at = 0;
        while let Some((cname, cbody, after)) = next_block(text, at, "cell") {
            cells.insert(cname.clone(), parse_cell(cname, &cbody, &units, opts.skip_ccs));
            at = after;
        }
        if cells.is_empty() {
            return Err(LibError("no cells found".into()));
        }
        Ok(Lib { cells, voltage })
    }

    pub fn load(path: &str) -> Result<Lib, LibError> {
        Lib::load_opts(path, LibOpts::default())
    }

    /// Like [`Lib::load`] but honoring [`LibOpts`] (e.g. `skip_ccs` for NLDM-only).
    pub fn load_opts(path: &str, opts: LibOpts) -> Result<Lib, LibError> {
        let text = std::fs::read_to_string(path).map_err(|e| LibError(format!("{path}: {e}")))?;
        Lib::parse_opts(&text, opts)
    }

    pub fn cell(&self, name: &str) -> Option<&Cell> {
        self.cells.get(name)
    }

    /// Merge another lib's cells into this one (multi-lib jobs). Existing cells win.
    pub fn merge(&mut self, other: Lib) {
        if self.voltage == 0.0 {
            self.voltage = other.voltage;
        }
        for (k, v) in other.cells {
            self.cells.entry(k).or_insert(v);
        }
    }

    /// Serialize the parsed IR to a structured JSON view (std-only, no deps) — the
    /// shared Liberty intermediate that sta-si and vyges-power both consume, made
    /// inspectable for tooling / debug / MCP (sta-si `--emit-liberty-json`). Emits
    /// per-cell pin directions, capacitances, CCS presence and per-arc table shapes
    /// (`[slews, loads]`) — a structural summary, not the full NLDM table values, to
    /// stay tractable on real PDKs.
    pub fn to_json(&self) -> String {
        let mut s = String::new();
        s.push('{');
        s.push_str(&format!("\"voltage\":{},", jnum(self.voltage)));
        s.push_str(&format!("\"cell_count\":{},", self.cells.len()));
        s.push_str("\"cells\":{");
        for (ci, (cname, cell)) in self.cells.iter().enumerate() {
            if ci > 0 {
                s.push(',');
            }
            s.push_str(&format!("{}:{{", jstr(cname)));
            s.push_str(&format!("\"is_seq\":{},", cell.is_seq));
            s.push_str(&format!(
                "\"clock_pin\":{},",
                cell.clock_pin.as_deref().map(jstr).unwrap_or_else(|| "null".into())
            ));
            s.push_str(&format!("\"leakage_w\":{},", jnum(cell.leakage_w)));
            s.push_str(&format!("\"int_energy_j\":{},", jnum(cell.int_energy_j)));
            s.push_str("\"pins\":{");
            for (pi, (pname, pin)) in cell.pins.iter().enumerate() {
                if pi > 0 {
                    s.push(',');
                }
                s.push_str(&format!("{}:{{", jstr(pname)));
                s.push_str(&format!("\"direction\":{},", jstr(dir_str(pin.direction))));
                s.push_str(&format!("\"capacitance\":{},", jnum(pin.capacitance)));
                s.push_str(&format!("\"cap_f\":{},", jnum(pin.cap_f)));
                s.push_str(&format!("\"clock\":{},", pin.clock));
                let has_recv = pin.recv.as_ref().map(|r| !r.is_empty()).unwrap_or(false);
                s.push_str(&format!("\"has_recv_ccs\":{},", has_recv));
                s.push_str(&format!("\"setup_groups\":{},", pin.setup.len()));
                s.push_str(&format!("\"hold_groups\":{},", pin.hold.len()));
                s.push_str("\"arcs\":[");
                for (ai, arc) in pin.arcs.iter().enumerate() {
                    if ai > 0 {
                        s.push(',');
                    }
                    s.push_str(&format!(
                        "{{\"related_pin\":{},\"sense\":{},\"has_ccs\":{},\"cell_rise\":{},\"cell_fall\":{},\"rise_transition\":{},\"fall_transition\":{}}}",
                        jstr(&arc.related_pin),
                        jstr(&arc.sense),
                        !arc.ccs.is_empty(),
                        dims(&arc.cell_rise),
                        dims(&arc.cell_fall),
                        dims(&arc.rise_transition),
                        dims(&arc.fall_transition),
                    ));
                }
                s.push_str("]}"); // arcs, pin
            }
            s.push_str("}}"); // pins, cell
        }
        s.push_str("}}\n"); // cells, root
        s
    }
}

// ── JSON helpers for `Lib::to_json` (std-only) ───────────────────────────────────

/// A finite f64 as a JSON number (full round-trippable decimal, so tiny physical
/// quantities like leakage_w / cap_f keep their magnitude); non-finite → `null`.
fn jnum(v: f64) -> String {
    if v.is_finite() {
        format!("{v}")
    } else {
        "null".to_string()
    }
}

/// A JSON-escaped, double-quoted string.
fn jstr(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\t' => out.push_str("\\t"),
            '\r' => out.push_str("\\r"),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

fn dir_str(d: Dir) -> &'static str {
    match d {
        Dir::In => "input",
        Dir::Out => "output",
        Dir::Inout => "inout",
        Dir::Other => "other",
    }
}

/// A table's shape as `[slews, loads]` (index_1 × index_2).
fn dims(t: &Table) -> String {
    format!("[{},{}]", t.index_1.len(), t.index_2.len())
}

// ── Library units (power) ───────────────────────────────────────────────────────
// Parsed once per library so per-cell power numbers come out in SI (W, F, J).

struct Units {
    cap_f: f64,    // Farads per capacitance unit
    leak_w: f64,   // Watts per leakage_power_unit
    energy_j: f64, // Joules per dynamic-energy unit (voltage·current·time)
}

impl Units {
    fn from_lib(text: &str) -> Units {
        let leak_w =
            simple_attr(text, "leakage_power_unit").as_deref().map(parse_si_power).unwrap_or(1.0e-9);
        let time_s =
            simple_attr(text, "time_unit").as_deref().map(parse_si_time).unwrap_or(1.0e-9);
        let cap_f = cap_load_unit(text).unwrap_or(1.0e-12);
        // Dynamic-energy unit = power_unit × time, where dynamic power_unit =
        // voltage_unit × current_unit (NOT leakage_power_unit). sky130: 1V·1mA·1ns = 1e-12 J.
        let v = simple_attr(text, "voltage_unit").as_deref().map(parse_si_voltage).unwrap_or(1.0);
        let a = simple_attr(text, "current_unit").as_deref().map(parse_si_current).unwrap_or(1.0);
        Units { cap_f, leak_w, energy_j: v * a * time_s }
    }
}

fn parse_si(s: &str, units: &[(&str, f64)]) -> f64 {
    let s = s.trim().trim_matches('"').trim();
    for (suf, scale) in units {
        if let Some(num) = s.strip_suffix(suf) {
            return num.trim().parse::<f64>().unwrap_or(1.0) * scale;
        }
    }
    s.parse::<f64>().unwrap_or(1.0)
}
fn parse_si_power(s: &str) -> f64 {
    parse_si(s, &[("fW", 1e-15), ("pW", 1e-12), ("nW", 1e-9), ("uW", 1e-6), ("mW", 1e-3), ("W", 1.0)])
}
fn parse_si_time(s: &str) -> f64 {
    parse_si(s, &[("fs", 1e-15), ("ps", 1e-12), ("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)])
}
fn parse_si_voltage(s: &str) -> f64 {
    parse_si(s, &[("uV", 1e-6), ("mV", 1e-3), ("kV", 1e3), ("V", 1.0)])
}
fn parse_si_current(s: &str) -> f64 {
    parse_si(s, &[("pA", 1e-12), ("nA", 1e-9), ("uA", 1e-6), ("mA", 1e-3), ("A", 1.0)])
}

/// `capacitive_load_unit (1, pf)` → Farads-per-unit.
fn cap_load_unit(lib_body: &str) -> Option<f64> {
    let p = lib_body.find("capacitive_load_unit")?;
    let open = lib_body[p..].find('(')? + p;
    let close = lib_body[open..].find(')')? + open;
    let mut parts = lib_body[open + 1..close].split(',');
    let scale: f64 = parts.next()?.trim().parse().unwrap_or(1.0);
    let base = match parts.next().unwrap_or("pf").trim().to_lowercase().as_str() {
        "ff" => 1e-15,
        "pf" => 1e-12,
        "nf" => 1e-9,
        _ => 1e-12,
    };
    Some(scale * base)
}

/// nom_voltage, else an operating_conditions `voltage :`, else None.
fn lib_voltage(text: &str) -> Option<f64> {
    if let Some(v) = simple_attr(text, "nom_voltage").and_then(|s| s.parse().ok()) {
        return Some(v);
    }
    let mut at = 0;
    while let Some((_, oc, after)) = next_block(text, at, "operating_conditions") {
        if let Some(v) = simple_attr(&oc, "voltage").and_then(|s| s.parse().ok()) {
            return Some(v);
        }
        at = after;
    }
    None
}

/// Mean-able numbers inside every `values(...)` of a cell's `internal_power` groups.
fn internal_values(cell_body: &str) -> Vec<f64> {
    let mut out = Vec::new();
    let mut at = 0;
    while let Some((_, ip, after)) = next_block(cell_body, at, "internal_power") {
        let b = ip.as_bytes();
        let mut idx = 0;
        while let Some(rel) = ip[idx..].find("values") {
            let p = idx + rel;
            let Some(orel) = ip[p..].find('(') else { break };
            let open = p + orel;
            let mut d = 0;
            let mut k = open;
            let mut close = open;
            while k < b.len() {
                match b[k] {
                    b'(' => d += 1,
                    b')' => {
                        d -= 1;
                        if d == 0 {
                            close = k;
                            break;
                        }
                    }
                    _ => {}
                }
                k += 1;
            }
            for tok in ip[open + 1..close].split(|c: char| {
                !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E')
            }) {
                if let Ok(v) = tok.trim().parse::<f64>() {
                    out.push(v.abs());
                }
            }
            idx = close + 1;
        }
        at = after;
    }
    out
}

#[cfg(test)]
mod power_tests {
    use super::*;

    const LIB: &str = r#"
library (demo) {
  leakage_power_unit : 1nW;
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  nom_voltage : 1.8;
  cell (INV) {
    cell_leakage_power : 2.0;
    pin (A) { direction : input; capacitance : 0.004; }
    pin (Y) { direction : output;
      internal_power () { related_pin : "A";
        rise_power (t) { values("0.010, 0.012"); }
        fall_power (t) { values("0.008, 0.010"); }
      }
    }
  }
}
"#;

    #[test]
    fn parses_power_units_leakage_caps_energy() {
        let lib = Lib::parse(LIB).unwrap();
        assert!((lib.voltage - 1.8).abs() < 1e-9);
        let inv = lib.cell("INV").unwrap();
        assert!((inv.leakage_w - 2.0e-9).abs() < 1e-18); // 2 nW
        assert!((inv.input_cap("A") - 0.004e-12).abs() < 1e-21); // 0.004 pF -> F
        // mean(0.010,0.012,0.008,0.010)=0.010 ; energy unit = V·I·t = 1·1·1ns = 1e-9 J
        assert!((inv.int_energy_j - 0.010e-9).abs() < 1e-13);
        assert_eq!(inv.outputs().count(), 1);
        assert_eq!(inv.pins.get("A").unwrap().direction, Dir::In);
    }
}

#[cfg(test)]
mod ccs_skip_tests {
    use super::*;

    // A cell carrying both CCS groups: input receiver_capacitance + an output_current arc.
    const CCS_LIB: &str = r#"
library (demo) {
  capacitive_load_unit (1, pf);
  cell (INV) {
    pin (A) {
      direction : input;
      capacitance : 0.004;
      receiver_capacitance () {
        receiver_capacitance1_rise (t) { values("0.001, 0.002"); }
        receiver_capacitance2_rise (t) { values("0.003, 0.004"); }
        receiver_capacitance1_fall (t) { values("0.001, 0.002"); }
        receiver_capacitance2_fall (t) { values("0.003, 0.004"); }
      }
    }
    pin (Y) {
      direction : output;
      timing () {
        related_pin : "A";
        cell_rise (t) { values("0.1, 0.2"); }
        cell_fall (t) { values("0.1, 0.2"); }
        output_current_rise () {
          vector (v) {
            index_1("0.01");
            index_2("0.005");
            index_3("0.0, 0.1, 0.2");
            values("0.0, 0.5, 1.0");
          }
        }
      }
    }
  }
}
"#;

    #[test]
    fn skip_ccs_drops_receiver_and_output_current_keeps_nldm() {
        // Default parse keeps CCS (receiver_capacitance + output_current).
        let full = Lib::parse(CCS_LIB).unwrap();
        let a = full.cell("INV").unwrap().pins.get("A").unwrap();
        let y = full.cell("INV").unwrap().pins.get("Y").unwrap();
        assert!(a.recv.is_some(), "receiver_capacitance present on full parse");
        assert_eq!(y.arcs.len(), 1);
        assert!(!y.arcs[0].ccs.is_empty(), "output_current present on full parse");

        // skip_ccs drops both CCS groups but leaves the NLDM delay arc intact.
        let nldm = Lib::parse_opts(CCS_LIB, LibOpts { skip_ccs: true }).unwrap();
        let a2 = nldm.cell("INV").unwrap().pins.get("A").unwrap();
        let y2 = nldm.cell("INV").unwrap().pins.get("Y").unwrap();
        assert!(a2.recv.is_none(), "receiver_capacitance skipped");
        assert_eq!(y2.arcs.len(), 1, "NLDM delay arc preserved");
        assert!(y2.arcs[0].ccs.is_empty(), "output_current skipped");
    }

    #[test]
    fn to_json_emits_structured_ir() {
        let js = Lib::parse(CCS_LIB).unwrap().to_json();
        assert!(js.starts_with('{') && js.trim_end().ends_with('}'));
        assert!(js.contains("\"cell_count\":1"));
        assert!(js.contains("\"INV\""));
        assert!(js.contains("\"direction\":\"input\""));
        assert!(js.contains("\"direction\":\"output\""));
        assert!(js.contains("\"has_recv_ccs\":true")); // pin A: receiver_capacitance
        assert!(js.contains("\"has_ccs\":true")); // pin Y arc: output_current
        assert!(js.contains("\"related_pin\":\"A\""));

        // NLDM-only parse flips the CCS presence flags to false.
        let js2 = Lib::parse_opts(CCS_LIB, LibOpts { skip_ccs: true }).unwrap().to_json();
        assert!(js2.contains("\"has_recv_ccs\":false"));
        assert!(js2.contains("\"has_ccs\":false"));
    }
}
