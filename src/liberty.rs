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
    pub index_1: Vec<f64>,     // input slews
    pub index_2: Vec<f64>,     // output loads
    pub values: Vec<Vec<f64>>, // values[i][j] over (slew_i, load_j)
}

/// The clock edge a **sequential** delay arc launches on — Liberty `timing_type :
/// rising_edge` / `falling_edge`.
///
/// It is not the same information as `timing_sense`, and it cannot be recovered from it:
/// sky130's `rising_edge` arcs declare no `timing_sense` at all, so they parse as
/// `non_unate` and every input edge appears to drive every output edge. A flop's CK->Q may
/// be launched by ONE clock edge; letting the other launch it is how a min-delay pass ends
/// up carrying the FALLING edge of the clock into the data path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClkEdge {
    Rise,
    Fall,
}

#[derive(Debug, Clone)]
pub struct Arc {
    pub related_pin: String,
    pub sense: String,
    /// `Some` only on a sequential (`rising_edge` / `falling_edge`) arc; `None` on a
    /// combinational one. See [`ClkEdge`].
    pub clk_edge: Option<ClkEdge>,
    pub cell_rise: Table,
    pub cell_fall: Table,
    pub rise_transition: Table,
    pub fall_transition: Table,
    pub ccs: crate::ccs::CcsArc, // CCS current waveforms (empty if NLDM-only)
    /// Did the arc DECLARE an `output_current_*` group, whatever we read from it? Kept separate
    /// from `ccs.is_empty()` so a library characterised for CCS whose tables we failed to read is
    /// distinguishable from one that never had any — the first is this reader falling back to
    /// NLDM without saying so, the second is an NLDM library being used as intended.
    pub ccs_declared: bool,
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
    ///
    /// ⚠️ Use [`Constraint::eval_edge`] when the ARRIVAL is also per edge. A check exists
    /// for one data edge at a time — `VisitPathEnds::visitCheckEnd` matches
    /// `check_arc->toEdge()` against the path's `end_rf` — so pairing the earliest arrival
    /// with the other edge's constraint scores a check that does not exist.
    pub fn eval(&self, clock_slew: f64, data_slew: f64) -> f64 {
        self.rise
            .lookup(clock_slew, data_slew)
            .max(self.fall.lookup(clock_slew, data_slew))
    }

    /// The constraint for ONE data edge: `rise` when `rising`, else `fall`.
    pub fn eval_edge(&self, rising: bool, clock_slew: f64, data_slew: f64) -> f64 {
        if rising {
            self.rise.lookup(clock_slew, data_slew)
        } else {
            self.fall.lookup(clock_slew, data_slew)
        }
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
            if n == 0 {
                None
            } else {
                Some(sum / n as f64)
            }
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
    pub capacitance: f64, // input capacitance in library units (timing/NLDM load axis)
    pub cap_f: f64,       // same capacitance in Farads (power: net-load summation)
    pub recv: Option<RecvCap>, // CCS receiver model (input pins); None -> use `capacitance`
    pub clock: bool,      // `clock : true` — the cell's clock pin
    /// Boolean `function` of an output pin, verbatim from Liberty (e.g. `"!A"`, `"A&B"`).
    /// `None` on inputs, and on outputs in libraries that omit it — a timing-only library is
    /// perfectly valid, so absence is not an error.
    pub function: Option<String>,
    pub setup: Vec<Constraint>, // setup constraint group(s) vs the clock
    pub hold: Vec<Constraint>,  // hold constraint group(s) vs the clock
    /// `recovery_*` — the async set/reset release must arrive early enough before the
    /// clock edge. The asynchronous counterpart of setup, on the SET/RESET pin.
    pub recovery: Vec<Constraint>,
    /// `removal_*` — the async set/reset release must stay stable long enough after the
    /// clock edge. The asynchronous counterpart of hold, on the SET/RESET pin.
    ///
    /// These are separate constraints with their own tables; applying a *data* setup/hold
    /// table to an async pin is not an approximation, it is the wrong check.
    pub removal: Vec<Constraint>,
    pub arcs: Vec<Arc>, // delay arcs (e.g. CK->Q on a flop output)
}

impl Pin {
    /// True when this output pin drives a **fixed logic level** — a tie cell, declared in
    /// Liberty as `function : "1"` or `function : "0"` with no timing arcs (sky130's
    /// `conb_1` has both, as `HI` and `LO`).
    ///
    /// A net driven only by such a pin can never switch, so it carries no timing at all:
    /// no arrival to propagate and no check to apply. Treating one as an ordinary
    /// undriven node instead makes it look like a path source at t=0, which manufactures
    /// violations on wires that never toggle.
    pub fn is_constant(&self) -> bool {
        if !matches!(self.direction, Dir::Out | Dir::Inout) {
            return false;
        }
        match self.function.as_deref() {
            Some(f) => {
                let f = f.trim().trim_matches('"').trim();
                f == "0" || f == "1"
            }
            None => false,
        }
    }

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
    pub is_seq: bool,              // has an `ff`/`latch` group
    pub clock_pin: Option<String>, // the pin marked `clock : true`
    pub leakage_w: f64,            // cell_leakage_power → Watts (power)
    pub int_energy_j: f64,         // representative per-transition internal energy → Joules (power)
    /// `cell_footprint` — the vendor's own grouping of interchangeable cells. When a library
    /// provides it, it is the most reliable equivalence key there is: the foundry is asserting
    /// these cells are drop-in for one another.
    pub cell_footprint: Option<String>,
    /// `area` in library units. Zero when absent. Ranks an equivalence class: for cells of the
    /// same function, area is a good proxy for drive strength.
    pub area: f64,
    /// Pins that **asynchronously** force the output — the `clear` / `preset` expressions in
    /// the `ff` / `latch` group, e.g. `clear : "!RESET_B"` yields `["RESET_B"]`.
    ///
    /// These are the reset-domain sources, and they are what makes reset-domain-crossing
    /// analysis possible at all. A synchronous reset is just data on `next_state` and is
    /// deliberately NOT collected here: it is timed like any other path and cannot cause the
    /// deassertion race an RDC check exists to find.
    pub async_reset_pins: Vec<String>,
    /// `clock_gating_integrated_cell` as written (`latch_posedge`, `latch_negedge_precontrol`,
    /// …), or `None` when the cell is not an integrated clock gate. The sequential rule already
    /// reads it — a clock gate's clock pin does not make it a register — and a consumer that
    /// traces clocks needs the same answer.
    pub clock_gating: Option<String>,
    /// Scan roles, `pin -> signal_type` as the cell's `test_cell` states them
    /// — one of the nine `test_scan_*` values (`test_scan_in`, `test_scan_enable`, `test_scan_out`,
    /// their `_inverted` forms, `test_scan_clock`, `_a`, `_b`); any other value is ignored. Empty
    /// for a cell with no `test_cell`.
    ///
    /// 🔑 This is the ONLY thing read from a `test_cell`. Its pins repeat the real ones in scan
    /// mode and must never replace them (see `parse_cell`), but the roles exist nowhere else.
    pub scan_pins: BTreeMap<String, String>,
}

impl Lib {
    /// The cells interchangeable with `cell`, **ranked by `area` ascending** — a drive-strength
    /// ladder including `cell` itself, so a caller can see where it sits.
    ///
    /// This is what a resize move needs: swapping a cell only makes sense if the replacement
    /// computes the same thing. OpenDB will refuse a swap whose pins do not match, but nothing
    /// downstream checks *function*, so that check has to happen here.
    ///
    /// Equivalence is decided in order of how much the library is telling us:
    ///
    /// 1. **`cell_footprint`**, when both cells declare one. This is the vendor asserting the
    ///    cells are drop-in for one another — better evidence than anything we can infer.
    /// 2. Otherwise, **identical pin names and identical output functions**. Structural, and
    ///    only as good as the library's `function` attributes.
    ///
    /// A **sequential** cell is only ever matched by footprint. Its behaviour lives in an `ff`
    /// group rather than a pin function, so a structural match would be guessing — and guessing
    /// wrong about a flop swaps a design's state element.
    ///
    /// Returns an empty vec if `cell` is unknown, and a single-element vec (the cell itself) if
    /// nothing is interchangeable with it — including the common case of a timing-only library
    /// with no `function` and no `cell_footprint`, where equivalence is simply not knowable.
    pub fn equivalence_class(&self, cell: &str) -> Vec<&Cell> {
        let Some(c) = self.cells.get(cell) else {
            return Vec::new();
        };
        let mut out: Vec<&Cell> = self
            .cells
            .values()
            .filter(|o| interchangeable(c, o))
            .collect();
        out.sort_by(|a, b| {
            a.area
                .partial_cmp(&b.area)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then_with(|| a.name.cmp(&b.name))
        });
        out
    }

    /// Interchangeable cells **larger** than `cell` — upsize candidates, weakest first. The
    /// setup-repair move: a bigger drive on a critical arc.
    pub fn upsize_candidates(&self, cell: &str) -> Vec<&Cell> {
        let area = self.cells.get(cell).map(|c| c.area).unwrap_or(0.0);
        self.equivalence_class(cell)
            .into_iter()
            .filter(|c| c.area > area && c.name != cell)
            .collect()
    }

    /// Interchangeable cells **smaller** than `cell` — downsize candidates, smallest first. Adds
    /// delay, so it is a hold-repair move as well as an area/leakage one.
    pub fn downsize_candidates(&self, cell: &str) -> Vec<&Cell> {
        let area = self.cells.get(cell).map(|c| c.area).unwrap_or(0.0);
        self.equivalence_class(cell)
            .into_iter()
            .filter(|c| c.area < area && c.name != cell)
            .collect()
    }
}

/// Whether `b` can stand in for `a`. See [`Lib::equivalence_class`] for the ordering of evidence.
fn interchangeable(a: &Cell, b: &Cell) -> bool {
    // the vendor's own grouping wins whenever it is available
    if let (Some(fa), Some(fb)) = (&a.cell_footprint, &b.cell_footprint) {
        return fa == fb;
    }
    // a flop's behaviour is not in a pin function, so never match one structurally
    if a.is_seq || b.is_seq {
        return a.name == b.name;
    }
    if a.pins.len() != b.pins.len() {
        return false;
    }
    let mut any_function = false;
    for (name, pa) in &a.pins {
        let Some(pb) = b.pins.get(name) else {
            return false; // pin sets must match, or OpenDB would refuse the swap anyway
        };
        if pa.direction != pb.direction {
            return false;
        }
        match (&pa.function, &pb.function) {
            (Some(x), Some(y)) => {
                any_function = true;
                if x != y {
                    return false;
                }
            }
            // one declares a function and the other does not: not comparable
            (Some(_), None) | (None, Some(_)) => return false,
            (None, None) => {}
        }
    }
    // Same pins and no function anywhere is NOT evidence of equivalence — it is the signature of
    // a timing-only library, where every one-input/one-output cell would otherwise look alike.
    any_function || a.name == b.name
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
    /// How this library measures transitions and delays. Needed by any consumer that
    /// builds a *waveform* from a Liberty slew: the number is only meaningful together
    /// with the two percentage points it was measured between.
    pub thresholds: Thresholds,
}

/// Waveform measurement conventions declared in the library header.
///
/// A Liberty transition time is the time to move between `slew_lower_*` and
/// `slew_upper_*` — **not** a full 0→100 % edge. Reconstructing a ramp from it, or
/// reporting a slew back, requires these numbers; assuming 0→100 % makes the edge
/// `1/(upper-lower)` too fast, and measuring the result between different points
/// rescales it again. sky130 uses 20/80, but 10/90 and 30/70 are all in the wild, so
/// this is read from the library rather than assumed.
///
/// Values are **fractions** (0.2, not 20). Defaults are the Liberty defaults: 20/80
/// for slew, 50 % for delay, derate 1.0.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Thresholds {
    pub slew_lower_rise: f64,
    pub slew_upper_rise: f64,
    pub slew_lower_fall: f64,
    pub slew_upper_fall: f64,
    /// Delay measurement point on an input edge (`input_threshold_pct_*`).
    pub input_rise: f64,
    pub input_fall: f64,
    /// Delay measurement point on an output edge (`output_threshold_pct_*`).
    pub output_rise: f64,
    pub output_fall: f64,
    /// `slew_derate_from_library` — table transition values are scaled by this to get
    /// the real edge. sky130 declares 1.0; some libraries use 0.5.
    pub slew_derate: f64,
}

impl Default for Thresholds {
    fn default() -> Self {
        Thresholds {
            slew_lower_rise: 0.2,
            slew_upper_rise: 0.8,
            slew_lower_fall: 0.2,
            slew_upper_fall: 0.8,
            input_rise: 0.5,
            input_fall: 0.5,
            output_rise: 0.5,
            output_fall: 0.5,
            slew_derate: 1.0,
        }
    }
}

impl Thresholds {
    /// Fraction of a full 0→100 % edge that a declared transition time spans, averaged
    /// over rise and fall. For 20/80 this is 0.6, so a full edge lasts `slew / 0.6`.
    pub fn slew_span(&self) -> f64 {
        let r = self.slew_upper_rise - self.slew_lower_rise;
        let f = self.slew_upper_fall - self.slew_lower_fall;
        let s = (r + f) / 2.0;
        if s > 0.0 && s <= 1.0 {
            s
        } else {
            0.6 // a nonsense declaration should not silently scale every delay
        }
    }

    fn from_lib(text: &str) -> Thresholds {
        let d = Thresholds::default();
        // Liberty states these as percentages; store fractions.
        let pct = |key: &str, fallback: f64| -> f64 {
            simple_attr(text, key)
                .and_then(|s| lnum(&s))
                .map(|v| v / 100.0)
                .filter(|v| (0.0..=1.0).contains(v))
                .unwrap_or(fallback)
        };
        Thresholds {
            slew_lower_rise: pct("slew_lower_threshold_pct_rise", d.slew_lower_rise),
            slew_upper_rise: pct("slew_upper_threshold_pct_rise", d.slew_upper_rise),
            slew_lower_fall: pct("slew_lower_threshold_pct_fall", d.slew_lower_fall),
            slew_upper_fall: pct("slew_upper_threshold_pct_fall", d.slew_upper_fall),
            input_rise: pct("input_threshold_pct_rise", d.input_rise),
            input_fall: pct("input_threshold_pct_fall", d.input_fall),
            output_rise: pct("output_threshold_pct_rise", d.output_rise),
            output_fall: pct("output_threshold_pct_fall", d.output_fall),
            slew_derate: simple_attr(text, "slew_derate_from_library")
                .and_then(|s| lnum(&s))
                .filter(|v| *v > 0.0)
                .unwrap_or(d.slew_derate),
        }
    }
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
    // ⛔ EXTRAPOLATE past either end; do not clamp. The reference's `findValueIndex`
    // returns index 0 below the axis and `size - 2` above it — its comment says "Return
    // max_index-1 for value too large so interpolation pts are index,index+1" — and
    // `Table2::findValue` then forms `dx = (x - xl) / (xu - xl)` from the ACTUAL x, which
    // is negative below the axis and greater than 1 above it.
    //
    // Clamping instead makes the table flat outside its characterisation range, so a gate
    // driving more than it was characterised for shows NO extra delay and NO extra slew.
    // It also silently zeroes `gateModelRd` — two lookups a hair apart return the same
    // number — and a zero Rd sends `setCeffAlgorithm` down the capacitive branch, which
    // degrades no slew at all. That is how a 100 fF load behind 10 kΩ came out with an
    // undegraded edge.
    if v <= g[0] {
        return (0, 1, (v - g[0]) / (g[1] - g[0]));
    }
    if v >= g[n - 1] {
        return (n - 2, n - 1, (v - g[n - 2]) / (g[n - 1] - g[n - 2]));
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
/// Representative leakage from per-state `leakage_power ()` groups, in library units.
///
/// Each group gives one power rail's leakage in one logic state (`when`). The cell's leakage in
/// that state is the SUM over its rails — a ground-referenced entry of 0 is half of the same
/// state, not a state of its own — and the representative figure is the MEAN over states, which
/// is what `cell_leakage_power` means where a library provides both.
///
/// Returns 0.0 when there are no such groups, which is the honest answer for a library that
/// simply does not characterise leakage.
fn state_leakage(body: &str) -> f64 {
    use std::collections::BTreeMap;
    let mut by_state: BTreeMap<String, f64> = BTreeMap::new();
    let mut anon = 0usize;
    let mut at = 0;
    while let Some((_, gbody, after)) = next_block(body, at, "leakage_power") {
        at = after;
        let Some(v) = simple_attr(&gbody, "value").and_then(|s| lnum(&s)) else {
            continue;
        };
        // Groups with no `when` are unconditional; keep each on its own so they are averaged
        // rather than summed into one another.
        let state = simple_attr(&gbody, "when").unwrap_or_else(|| {
            anon += 1;
            format!("\u{0}anon{anon}")
        });
        *by_state.entry(state).or_default() += v;
    }
    if by_state.is_empty() {
        return 0.0;
    }
    by_state.values().sum::<f64>() / by_state.len() as f64
}

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

/// Identifiers in a Liberty boolean expression — `"!RESET_B"` -> `["RESET_B"]`,
/// `"!CLR & SET"` -> `["CLR", "SET"]`. Operators, constants and whitespace fall away.
fn idents_in(expr: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    for ch in expr.chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            cur.push(ch);
        } else if !cur.is_empty() {
            out.push(std::mem::take(&mut cur));
        }
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    // A bare `1`/`0` is a constant, not a pin.
    out.retain(|t| !t.chars().all(|c| c.is_ascii_digit()));
    out
}

/// `key : value ;` stated at `body`'s OWN level — not inside a nested group — and the LAST such
/// statement where there are several, as the reference's parser keeps it (a later simple
/// attribute replaces an earlier one).
///
/// ⛔ It used to return the FIRST textual match anywhere in `body`, nested groups included: an
/// attribute of an inner group could answer for the outer one, and a repeated attribute took its
/// first value where the reference takes its last. Quoted strings are skipped, so a key inside a
/// value is not a match.
///
/// ⚠️ It scans the whole of `body`. A hot path reads its attributes from [`own_attrs`], computed
/// once, rather than from a body that carries large tables.
fn simple_attr(body: &str, key: &str) -> Option<String> {
    let b = body.as_bytes();
    let k = key.as_bytes();
    let (mut i, mut depth, mut quoted) = (0usize, 0i32, false);
    let mut found = None;
    while i < b.len() {
        let c = b[i];
        if quoted {
            match c {
                b'\\' => i += 1,
                b'"' => quoted = false,
                _ => {}
            }
            i += 1;
            continue;
        }
        match c {
            b'"' => quoted = true,
            b'{' => depth += 1,
            b'}' => depth -= 1,
            _ if depth == 0 && b[i..].starts_with(k) && (i == 0 || !is_ident(b[i - 1])) => {
                let mut q = i + k.len();
                while q < b.len() && b[q].is_ascii_whitespace() {
                    q += 1;
                }
                if q < b.len() && b[q] == b':' {
                    found = Some(attr_value(body, q));
                }
                i = q;
                continue;
            }
            _ => {}
        }
        i += 1;
    }
    found
}

/// The value of a simple attribute whose `:` is at `colon`.
///
/// It ends at `;` OR at the end of the line, whichever comes first. Liberty asks for the
/// semicolon and real libraries do not always write one: asap7 states `area : 0.20412` with no
/// terminator on 37 cells. Scanning to the next `;` then swallows everything up to some later
/// attribute, the value fails to parse, and the cell silently reports an area of ZERO — which
/// ranks it first among interchangeable cells. A value genuinely continued onto the next line is
/// rare but legal, so an empty first line falls back to the semicolon.
fn attr_value(body: &str, colon: usize) -> String {
    let semi = body[colon..].find(';').map(|i| i + colon).unwrap_or(body.len());
    let nl = body[colon..].find('\n').map(|i| i + colon).unwrap_or(body.len());
    let v = body[colon + 1..semi.min(nl)].trim().trim_matches('"').trim();
    if !v.is_empty() {
        return v.to_string();
    }
    body[colon + 1..semi].trim().trim_matches('"').to_string()
}

/// A number as the reference's Liberty lexer reads it: `strtof`, straight to `float` — ONE
/// rounding. ⛔ Not `parse::<f64>() as f32`, which rounds twice; that is the SPEF reader's rule
/// (`(float)atof`), and the two differ on decimals whose `double` lands near a `float` tie
/// (cpp-to-rust-numeric-reference §18).
fn lnum(s: &str) -> Option<f64> {
    s.trim().parse::<f32>().ok().map(f64::from)
}

/// The lexer's view of the text, applied before anything reads it: `/* */` comments outside a
/// quoted string become blanks, and a backslash-newline inside one is a continuation. Byte
/// offsets do not move — every replaced byte becomes a space, newlines inside a comment stay —
/// so nothing downstream sees a different layout.
///
/// ⛔ Without it a commented-out `cell (X) { … }` was a real cell, and a commented-out
/// `area : 7.0;` set the area: this reader scans text for keywords, and a comment is text.
fn lex_clean(b: &mut [u8]) {
    let (mut i, mut quoted) = (0, false);
    while i < b.len() {
        match b[i] {
            b'"' => quoted = !quoted,
            // inside a string a backslash escapes the next byte; before a line end it is a
            // continuation, and both go (`\r\n` counts as one line end)
            b'\\' if quoted => {
                if matches!(b.get(i + 1), Some(b'\n' | b'\r')) {
                    b[i] = b' ';
                    i += 1;
                    let crlf = b[i] == b'\r' && b.get(i + 1) == Some(&b'\n');
                    b[i] = b' ';
                    if crlf {
                        i += 1;
                        b[i] = b' ';
                    }
                } else {
                    i += 1;
                }
            }
            b'/' if !quoted && b.get(i + 1) == Some(&b'*') => {
                while i < b.len() && !(b[i] == b'*' && b.get(i + 1) == Some(&b'/')) {
                    if b[i] != b'\n' {
                        b[i] = b' ';
                    }
                    i += 1;
                }
                for c in b.iter_mut().skip(i).take(2) {
                    *c = b' ';
                }
                i += 1;
            }
            _ => {}
        }
        i += 1;
    }
}

/// Does the text need [`lex_clean`]? Cheap, so a clean file is not copied.
fn needs_lex_clean(text: &str) -> bool {
    text.contains("/*") || text.contains("\\\n") || text.contains("\\\r")
}

fn floats(s: &str) -> Vec<f64> {
    s.split(',')
        .filter_map(lnum)
        .collect()
}

fn parse_table(body: &str) -> Table {
    // index_1/index_2 use paren+quote form: `index_1 ("0.01, 0.04");`
    let idx = |kw: &str| {
        next_paren_after(body, kw)
            .map(|s| floats(&s.replace('"', "")))
            .unwrap_or_default()
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
    Table {
        index_1,
        index_2,
        values,
    }
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

fn parse_arc(timing_body: &str, own: &str, spans: &[GroupSpan], skip_ccs: bool) -> Arc {
    let tbl = |name: &str| span_table(timing_body, spans, name);
    // `rising_edge` / `falling_edge` name the launching clock edge. Anything else is a
    // combinational arc; the checks (setup_*/hold_*/recovery_*/removal_*) never reach
    // here — the caller routes them into the constraint vectors before this point.
    let clk_edge = match simple_attr(own, "timing_type").as_deref() {
        Some("rising_edge") => Some(ClkEdge::Rise),
        Some("falling_edge") => Some(ClkEdge::Fall),
        _ => None,
    };
    Arc {
        related_pin: simple_attr(own, "related_pin").unwrap_or_default(),
        sense: simple_attr(own, "timing_sense").unwrap_or_else(|| "non_unate".into()),
        clk_edge,
        cell_rise: tbl("cell_rise"),
        cell_fall: tbl("cell_fall"),
        rise_transition: tbl("rise_transition"),
        fall_transition: tbl("fall_transition"),
        // CCS output_current waveforms — skipped (empty) for NLDM-only parses.
        ccs_declared: spans.iter().any(|g| g.kw == "output_current_rise" || g.kw == "output_current_fall"),
        ccs: if skip_ccs {
            crate::ccs::CcsArc::default()
        } else {
            parse_ccs(timing_body)
        },
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
        next_paren_after(b, kw)
            .map(|s| floats(&s.replace('"', "")))
            .unwrap_or_default()
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
                ref_time: simple_attr(&vbody, "reference_time")
                    .and_then(|s| lnum(&s))
                    .unwrap_or(0.0),
                time,
                current,
            });
        }
        at = after;
    }
    out
}

/// Parse a setup/hold constraint group's rise/fall tables.
fn parse_constraint(timing_body: &str, spans: &[GroupSpan]) -> Constraint {
    Constraint {
        rise: span_table(timing_body, spans, "rise_constraint"),
        fall: span_table(timing_body, spans, "fall_constraint"),
    }
}

fn parse_pin(name: String, body: &str, cap_unit_f: f64, skip_ccs: bool) -> Pin {
    let (own, pin_groups) = level(body);
    let direction = match simple_attr(&own, "direction").as_deref() {
        Some("input") => Dir::In,
        Some("output") => Dir::Out,
        Some("inout") => Dir::Inout,
        _ => Dir::Other,
    };
    let capacitance = simple_attr(&own, "capacitance")
        .and_then(|s| lnum(&s))
        .unwrap_or(0.0);
    let cap_f = capacitance * cap_unit_f;
    // CCS receiver capacitance group (input pins): the Miller-aware two-segment load.
    // Skipped for NLDM-only parses (consumers fall back to lumped Ceff).
    let recv = if skip_ccs {
        None
    } else {
        pin_groups.iter().find(|g| g.kw == "receiver_capacitance").map(|g| {
            let rbody = &body[g.body.clone()];
            let (_, rspans) = level(rbody);
            let tbl = |name: &str| span_table(rbody, &rspans, name);
            RecvCap {
                c1_rise: tbl("receiver_capacitance1_rise"),
                c2_rise: tbl("receiver_capacitance2_rise"),
                c1_fall: tbl("receiver_capacitance1_fall"),
                c2_fall: tbl("receiver_capacitance2_fall"),
            }
        })
    };
    let clock = simple_attr(&own, "clock").as_deref() == Some("true");
    // Liberty puts `function` on the OUTPUT pin, not the cell. It is what makes two cells
    // comparable: a resize must not change what a gate computes.
    let function = simple_attr(&own, "function").filter(|f| !f.is_empty());
    let mut arcs = Vec::new();
    let mut setup: Vec<Constraint> = Vec::new();
    let mut hold: Vec<Constraint> = Vec::new();
    let mut recovery: Vec<Constraint> = Vec::new();
    let mut removal: Vec<Constraint> = Vec::new();
    for g in pin_groups.iter().filter(|g| g.kw == "timing") {
        let tbody = &body[g.body.clone()];
        let (town, tspans) = level(tbody);
        match simple_attr(&town, "timing_type").as_deref() {
            Some(tt) if tt.starts_with("setup") => setup.push(parse_constraint(tbody, &tspans)),
            Some(tt) if tt.starts_with("hold") => hold.push(parse_constraint(tbody, &tspans)),
            // recovery/removal are CHECKS on the async set/reset pin, with their own
            // tables — kept, and applied by the timer as the async counterparts of
            // setup/hold. They are not max-delay data arcs.
            Some(tt) if tt.starts_with("recovery") => recovery.push(parse_constraint(tbody, &tspans)),
            Some(tt) if tt.starts_with("removal") => removal.push(parse_constraint(tbody, &tspans)),
            // clear/preset are async *effect* arcs (dfrtp RESET_B->Q), not launch paths —
            // propagating data through them would invent a path that cannot exist.
            // min_pulse_width is a check we do not implement yet.
            Some(tt)
                if tt.starts_with("clear")
                    || tt.starts_with("preset")
                    || tt.contains("pulse_width") => {}
            _ => arcs.push(parse_arc(tbody, &town, &tspans, skip_ccs)), // delay arc (incl. rising_edge CK->Q)
        }
    }
    Pin {
        name,
        direction,
        capacitance,
        cap_f,
        recv,
        clock,
        function,
        setup,
        hold,
        recovery,
        removal,
        arcs,
    }
}

/// The groups DIRECTLY inside `body` — `kw (args) { body }` at brace depth 0 — in file order.
///
/// 🔑 **Depth matters, and the substring scan ([`next_block`]) cannot see it.** A cell's `pin`,
/// `ff` and `statetable` belong to it only at its own level: the same words inside a `test_cell`
/// describe the scan-mode view of the cell, and inside a `bus` they name bits of that bus.
fn top_groups(body: &str) -> Vec<(String, String, &str)> {
    top_group_spans(body)
        .into_iter()
        .map(|g| (g.kw, g.args, &body[g.body]))
        .collect()
}

/// ONE pass over `body` at depth 0: the attributes stated at this level, and its groups.
fn level(body: &str) -> (String, Vec<GroupSpan>) {
    let spans = top_group_spans(body);
    let mut own = String::with_capacity(256);
    let mut at = 0;
    for g in &spans {
        own.push_str(&body[at..g.whole.start]);
        at = g.whole.end;
    }
    own.push_str(&body[at..]);
    (own, spans)
}

/// The first group named `kw` among `spans` of `body`, as a parsed table.
fn span_table(body: &str, spans: &[GroupSpan], kw: &str) -> Table {
    spans.iter().find(|g| g.kw == kw).map(|g| parse_table(&body[g.body.clone()])).unwrap_or_default()
}

struct GroupSpan {
    kw: String,
    args: String,
    /// The whole group, keyword through closing brace.
    whole: std::ops::Range<usize>,
    /// Inside the braces.
    body: std::ops::Range<usize>,
}

/// One linear pass over `body`: each depth-0 group with where it lies.
fn top_group_spans(body: &str) -> Vec<GroupSpan> {
    let b = body.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    // Start of the current statement: just past the last `;`, `}` or `{` at depth 0.
    let mut stmt = 0;
    while i < b.len() {
        match b[i] {
            b';' | b'}' => stmt = i + 1,
            b'{' => {
                let end = matching(b, i).min(b.len());
                let raw = &body[stmt..i];
                if let Some(open) = raw.find('(') {
                    // The keyword is the identifier just before `(`. Not everything since the last
                    // `;`: an attribute written without one (`area : 3.5` then `pg_pin (VDD) {`)
                    // would become part of the group's head, and be removed with the group.
                    let before = raw[..open].trim_end();
                    let kw_at = before.rfind(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).map_or(0, |k| k + 1);
                    let kw = before[kw_at..].to_string();
                    let args = raw[open + 1..].trim_end().trim_end_matches(')').trim().trim_matches('"');
                    out.push(GroupSpan {
                        kw,
                        args: args.to_string(),
                        whole: stmt + kw_at..(end + 1).min(b.len()),
                        body: i + 1..end,
                    });
                }
                i = end;
                stmt = end + 1;
            }
            _ => {}
        }
        i += 1;
    }
    out
}

/// `body` with every depth-0 group of kind `kw` removed, so a bus's own attributes and timing
/// are read without its inner `pin` groups'. Linear: a macro's bus can hold thousands of pins.
fn without_groups(body: &str, kw: &str) -> String {
    let mut out = String::with_capacity(body.len());
    let mut at = 0;
    for g in top_group_spans(body).into_iter().filter(|g| g.kw == kw) {
        out.push_str(&body[at..g.whole.start]);
        at = g.whole.end;
    }
    out.push_str(&body[at..]);
    out
}

/// `body` with EVERY depth-0 group removed: the attributes stated at this level and no deeper.
fn own_attrs(body: &str) -> String {
    level(body).0
}

/// Liberty `type (name) { bit_from ; bit_to ; }` groups, anywhere in the file: name -> (from, to).
fn bus_types(text: &str) -> BTreeMap<String, (i64, i64)> {
    let mut out = BTreeMap::new();
    let mut at = 0;
    while let Some((name, tbody, after)) = next_block(text, at, "type") {
        let int = |k: &str| simple_attr(&tbody, k).and_then(|v| v.parse::<i64>().ok());
        if let (Some(from), Some(to)) = (int("bit_from"), int("bit_to")) {
            out.insert(name, (from, to));
        }
        at = after;
    }
    out
}

/// The bit names a pin group inside a bus names: `d[3]` is one, `d[3:0]` is a range, `d` is all.
fn named_bits(name: &str, bus: &str, all: &[String]) -> Vec<String> {
    let Some(sub) = name.strip_prefix(bus).and_then(|r| r.strip_prefix('[')).and_then(|r| r.strip_suffix(']')) else {
        return if name == bus { all.to_vec() } else { vec![name.to_string()] };
    };
    match sub.split_once(':') {
        Some((a, b)) => match (a.trim().parse::<i64>(), b.trim().parse::<i64>()) {
            (Ok(a), Ok(b)) => bit_range(a, b).map(|k| format!("{bus}[{k}]")).collect(),
            _ => vec![name.to_string()],
        },
        None => vec![format!("{bus}[{}]", sub.trim())],
    }
}

/// `from` to `to` inclusive, in either direction — the order Liberty lists a bus's bits in.
fn bit_range(from: i64, to: i64) -> Box<dyn Iterator<Item = i64>> {
    if from >= to {
        Box::new((to..=from).rev())
    } else {
        Box::new(from..=to)
    }
}

/// A member pin: the group's attributes (a `bus` or `bundle`) with what an inner `pin` group
/// states over them. The inner group's timing is ADDED to the group's, which applies to every bit.
fn merged_pin(name: &str, group: &Pin, inner: Option<(&Pin, &str)>) -> Pin {
    let mut p = group.clone();
    p.name = name.to_string();
    if let Some((ip, ibody)) = inner {
        if ip.direction != Dir::Other {
            p.direction = ip.direction;
        }
        if simple_attr(ibody, "capacitance").is_some() {
            p.capacitance = ip.capacitance;
            p.cap_f = ip.cap_f;
        }
        if ip.recv.is_some() {
            p.recv = ip.recv.clone();
        }
        p.clock |= ip.clock;
        if ip.function.is_some() {
            p.function = ip.function.clone();
        }
        p.setup.extend(ip.setup.iter().cloned());
        p.hold.extend(ip.hold.iter().cloned());
        p.recovery.extend(ip.recovery.iter().cloned());
        p.removal.extend(ip.removal.iter().cloned());
        p.arcs.extend(ip.arcs.iter().cloned());
    }
    p
}

/// The `signal_type` values a scan role is taken from; any other value is ignored, as the
/// reference reader does (it warns and drops it).
const SCAN_SIGNAL_TYPES: [&str; 9] = [
    "test_scan_enable",
    "test_scan_enable_inverted",
    "test_scan_clock",
    "test_scan_clock_a",
    "test_scan_clock_b",
    "test_scan_in",
    "test_scan_in_inverted",
    "test_scan_out",
    "test_scan_out_inverted",
];

fn parse_cell(
    name: String,
    body: &str,
    units: &Units,
    skip_ccs: bool,
    types: &BTreeMap<String, (i64, i64)>,
) -> Cell {
    let mut pins = BTreeMap::new();
    let mut scan_pins = BTreeMap::new();
    let (own, spans) = level(body);
    let groups: Vec<(String, String, &str)> = spans.into_iter().map(|g| (g.kw, g.args, &body[g.body])).collect();
    // Pins at the CELL's level only, plus the bits of its buses and the members of its bundles.
    //
    // ⛔ A `test_cell` group repeats the cell's pins in scan mode — without `clock : true` and
    // with `signal_type` instead. Read by the substring scan, it came last and overwrote the real
    // pins: scan flops lost their clock pin, caps, checks and arcs — the public sky130 `sdf*` cells
    // among them.
    for (kw, args, gbody) in &groups {
        match kw.as_str() {
            "pin" => {
                for pname in args.split(',').map(|n| n.trim().trim_matches('"')).filter(|n| !n.is_empty()) {
                    pins.insert(pname.to_string(), parse_pin(pname.to_string(), gbody, units.cap_f, skip_ccs));
                }
            }
            // ⛔ A bus's direction and capacitance are on the BUS, and a bus need not repeat them
            // in per-bit `pin` groups — or have any. Unexpanded, an SRAM's D/Q/DM bits had no
            // direction at all. A bus is its bits `name[i]`, bit_from to
            // bit_to; the bus's attributes reach every bit; a pin group inside states its own.
            "bus" | "bundle" => {
                let own = parse_pin(args.clone(), &without_groups(gbody, "pin"), units.cap_f, skip_ccs);
                let members: Vec<String> = if kw == "bus" {
                    match simple_attr(gbody, "bus_type").and_then(|t| types.get(&t).copied()) {
                        Some((from, to)) => bit_range(from, to).map(|k| format!("{args}[{k}]")).collect(),
                        None => Vec::new(),
                    }
                } else {
                    next_paren_after(gbody, "members")
                        .map(|m| m.split(',').map(|x| x.trim().trim_matches('"').to_string()).filter(|x| !x.is_empty()).collect())
                        .unwrap_or_default()
                };
                for m in &members {
                    pins.insert(m.clone(), merged_pin(m, &own, None));
                }
                for (ikw, iargs, ibody) in top_groups(gbody) {
                    if ikw != "pin" {
                        continue;
                    }
                    let ip = parse_pin(iargs.clone(), ibody, units.cap_f, skip_ccs);
                    for bit in named_bits(&iargs, args, &members) {
                        let base = pins.get(&bit).cloned().unwrap_or_else(|| merged_pin(&bit, &own, None));
                        pins.insert(bit.clone(), merged_pin(&bit, &base, Some((&ip, ibody))));
                    }
                }
            }
            // Only the scan ROLES — see `Cell::scan_pins`.
            "test_cell" => {
                for (ikw, iargs, ibody) in top_groups(gbody) {
                    if ikw != "pin" {
                        continue;
                    }
                    let Some(role) = simple_attr(&own_attrs(ibody), "signal_type").filter(|r| SCAN_SIGNAL_TYPES.contains(&r.as_str())) else {
                        continue;
                    };
                    for pname in iargs.split(',').map(|n| n.trim().trim_matches('"')).filter(|n| !n.is_empty()) {
                        scan_pins.insert(pname.to_string(), role.clone());
                    }
                }
            }
            _ => {}
        }
    }
    let ff = groups
        .iter()
        .find(|(k, _, _)| k == "ff" || k == "latch")
        .map(|(_, a, b)| (a.clone(), b.to_string(), 0usize));
    let clock_pin = pins.iter().find(|(_, p)| p.clock).map(|(n, _)| n.clone());
    // SEQUENTIAL: an `ff`/`latch` group (or a bank of them), a `statetable` — synchronizer cells
    // model their state that way and have no `ff` at all — or a clock pin on a cell that is not a
    // clock gate. ⛔ Read at the cell's own level: a `test_cell`'s `ff` describes scan mode.
    let clock_gating = simple_attr(&own, "clock_gating_integrated_cell").filter(|v| !v.is_empty());
    let is_seq = groups
        .iter()
        .any(|(k, _, _)| matches!(k.as_str(), "ff" | "latch" | "ff_bank" | "latch_bank" | "statetable"))
        || (clock_pin.is_some() && clock_gating.is_none());
    // The `clear`/`preset` expressions name the asynchronous reset pins. Take only names the
    // cell actually declares as pins, so an expression referring to an internal node cannot
    // invent one.
    let async_reset_pins = ff
        .as_ref()
        .map(|(_, fbody, _)| {
            let mut v: Vec<String> = ["clear", "preset"]
                .iter()
                .filter_map(|k| simple_attr(fbody, k))
                .flat_map(|e| idents_in(&e))
                .filter(|n| pins.contains_key(n))
                .collect();
            v.sort();
            v.dedup();
            v
        })
        .unwrap_or_default();
    // power: leakage + representative internal (switching) energy.
    // `cell_leakage_power` is the cell's single representative number. Libraries that model
    // leakage per logic state give `leakage_power () { value ; when ; related_pg_pin ; }` groups
    // INSTEAD — asap7 has 222 of them and no per-cell attribute — and reading only the scalar
    // reports those cells as leaking exactly nothing, which is not a number anyone would query.
    let leakage_w =
        match simple_attr(&own, "cell_leakage_power").and_then(|s| lnum(&s)) {
            Some(v) => v * units.leak_w,
            None => state_leakage(body) * units.leak_w,
        };
    let ivals = internal_values(body);
    let int_energy_j = if ivals.is_empty() {
        0.0
    } else {
        (ivals.iter().sum::<f64>() / ivals.len() as f64) * units.energy_j
    };
    let cell_footprint = simple_attr(&own, "cell_footprint").filter(|f| !f.is_empty());
    let area = simple_attr(&own, "area")
        .and_then(|s| lnum(&s))
        .unwrap_or(0.0);
    Cell {
        name,
        pins,
        is_seq,
        clock_pin,
        leakage_w,
        int_energy_j,
        cell_footprint,
        area,
        async_reset_pins,
        clock_gating,
        scan_pins,
    }
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

// ── In-process parse-once cache for `Lib::load_opts` (#37) ────────────────────────
// Content-addressed (hash of the file bytes) so a changed file is always re-parsed —
// robust against coarse mtime granularity. Keyed on `LibOpts` too, since a `skip_ccs`
// parse yields a different (smaller) Lib. Reading the file is cheap; parsing large
// multi-corner Liberty is the cost this removes when the same lib is loaded more than
// once in a process (a run emitting report + SDF + liberty-json, or repeated corners).
// Bounded (coarse clear-on-overflow) so a long-running process can't grow unbounded.

const LIB_CACHE_CAP: usize = 256;

type LibCacheKey = (u64, u64, bool); // (content hash, byte length, skip_ccs)

fn lib_cache(
) -> &'static std::sync::Mutex<std::collections::HashMap<LibCacheKey, std::sync::Arc<Lib>>> {
    static C: std::sync::OnceLock<
        std::sync::Mutex<std::collections::HashMap<LibCacheKey, std::sync::Arc<Lib>>>,
    > = std::sync::OnceLock::new();
    C.get_or_init(|| std::sync::Mutex::new(std::collections::HashMap::new()))
}

/// The identity of "what parsing a `.lib` means", stamped at build time from the parser's own
/// source (see `build.rs`).
///
/// The cache is keyed on the FILE, so without this a parser fix never reaches anyone whose cache
/// is warm: the file has not changed, the key has not changed, and the stale answer is served for
/// ever. Measured twice in one sitting — a fix to attribute termination, then a fix to
/// state-dependent leakage, both correct and both invisible, because entries written minutes
/// earlier were still on disk. A hand-bumped constant was tried first and forgotten immediately,
/// which is why this is derived rather than remembered.
///
/// The magic/version tag inside the cache format guards the SERIALISATION; this guards the
/// SEMANTICS. They are different things and both are needed.
const LIB_PARSER_STAMP: &str = env!("VYGES_LIB_PARSER_STAMP");

fn lib_cache_key(text: &str, opts: LibOpts) -> LibCacheKey {
    use std::hash::{Hash, Hasher};
    let mut h = std::collections::hash_map::DefaultHasher::new();
    text.hash(&mut h);
    LIB_PARSER_STAMP.hash(&mut h);
    env!("CARGO_PKG_VERSION").hash(&mut h);
    (h.finish(), text.len() as u64, opts.skip_ccs)
}

/// Insert `lib` into the in-process cache under `key` (bounded) and return a clone.
fn cache_store(key: LibCacheKey, lib: Lib) -> Lib {
    let arc = std::sync::Arc::new(lib);
    if let Ok(mut m) = lib_cache().lock() {
        if m.len() >= LIB_CACHE_CAP {
            m.clear(); // bounded: coarse, but keeps a long-running process in check
        }
        m.insert(key, arc.clone());
    }
    (*arc).clone()
}

impl Lib {
    pub fn parse(text: &str) -> Result<Lib, LibError> {
        Lib::parse_opts(text, LibOpts::default())
    }

    /// Like [`Lib::parse`] but honoring [`LibOpts`] (e.g. `skip_ccs` for NLDM-only).
    pub fn parse_opts(text: &str, opts: LibOpts) -> Result<Lib, LibError> {
        let cleaned;
        let text = if needs_lex_clean(text) {
            let mut b = text.as_bytes().to_vec();
            lex_clean(&mut b);
            // only ASCII bytes were replaced, by ASCII, so the text is still UTF-8
            cleaned = String::from_utf8(b).map_err(|e| LibError(e.to_string()))?;
            cleaned.as_str()
        } else {
            text
        };
        // The library's own attributes, once: they sit one level down, inside `library () { }`,
        // and are read many times.
        let lib_own = top_group_spans(text)
            .into_iter()
            .find(|g| g.kw == "library")
            .map(|g| own_attrs(&text[g.body]))
            .unwrap_or_else(|| own_attrs(text));
        let units = Units::from_lib(&lib_own);
        let voltage = lib_voltage(&lib_own, text).unwrap_or(1.8);
        let thresholds = Thresholds::from_lib(&lib_own);
        let types = bus_types(text);
        let mut cells = BTreeMap::new();
        let mut at = 0;
        while let Some((cname, cbody, after)) = next_block(text, at, "cell") {
            cells.insert(
                cname.clone(),
                parse_cell(cname, &cbody, &units, opts.skip_ccs, &types),
            );
            at = after;
        }
        if cells.is_empty() {
            return Err(LibError("no cells found".into()));
        }
        Ok(Lib {
            cells,
            voltage,
            thresholds,
        })
    }

    pub fn load(path: &str) -> Result<Lib, LibError> {
        Lib::load_opts(path, LibOpts::default())
    }

    /// Like [`Lib::load`] but honoring [`LibOpts`] (e.g. `skip_ccs` for NLDM-only).
    /// Cached content-addressed so the same lib parses once: in-process (#37) →
    /// on-disk cross-process (`VYGES_LIB_CACHE`, #37/#38) → parse.
    pub fn load_opts(path: &str, opts: LibOpts) -> Result<Lib, LibError> {
        let text = std::fs::read_to_string(path).map_err(|e| LibError(format!("{path}: {e}")))?;
        // Cleaned IN PLACE: a large CCS library is not copied a second time.
        let mut b = text.into_bytes();
        lex_clean(&mut b);
        let text = String::from_utf8(b).map_err(|e| LibError(format!("{path}: {e}")))?;
        let key = lib_cache_key(&text, opts);
        // 1) in-process cache
        if let Some(hit) = lib_cache().lock().ok().and_then(|m| m.get(&key).cloned()) {
            return Ok((*hit).clone());
        }
        // 2) cross-process on-disk cache (env-gated; no-op unless enabled)
        if let Some(lib) = crate::libcache::disk_get(key) {
            return Ok(cache_store(key, lib));
        }
        // 3) parse, then populate both the disk and in-process caches
        let lib = Lib::parse_opts(&text, opts)?;
        crate::libcache::disk_put(key, &lib);
        Ok(cache_store(key, lib))
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
                cell.clock_pin
                    .as_deref()
                    .map(jstr)
                    .unwrap_or_else(|| "null".into())
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
        let leak_w = simple_attr(text, "leakage_power_unit")
            .as_deref()
            .map(parse_si_power)
            .unwrap_or(1.0e-9);
        let time_s = simple_attr(text, "time_unit")
            .as_deref()
            .map(parse_si_time)
            .unwrap_or(1.0e-9);
        let cap_f = cap_load_unit(text).unwrap_or(1.0e-12);
        // Dynamic-energy unit = power_unit × time, where dynamic power_unit =
        // voltage_unit × current_unit (NOT leakage_power_unit). sky130: 1V·1mA·1ns = 1e-12 J.
        let v = simple_attr(text, "voltage_unit")
            .as_deref()
            .map(parse_si_voltage)
            .unwrap_or(1.0);
        let a = simple_attr(text, "current_unit")
            .as_deref()
            .map(parse_si_current)
            .unwrap_or(1.0);
        Units {
            cap_f,
            leak_w,
            energy_j: v * a * time_s,
        }
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
    parse_si(
        s,
        &[
            ("fW", 1e-15),
            ("pW", 1e-12),
            ("nW", 1e-9),
            ("uW", 1e-6),
            ("mW", 1e-3),
            ("W", 1.0),
        ],
    )
}
fn parse_si_time(s: &str) -> f64 {
    parse_si(
        s,
        &[
            ("fs", 1e-15),
            ("ps", 1e-12),
            ("ns", 1e-9),
            ("us", 1e-6),
            ("ms", 1e-3),
            ("s", 1.0),
        ],
    )
}
fn parse_si_voltage(s: &str) -> f64 {
    parse_si(s, &[("uV", 1e-6), ("mV", 1e-3), ("kV", 1e3), ("V", 1.0)])
}
fn parse_si_current(s: &str) -> f64 {
    parse_si(
        s,
        &[
            ("pA", 1e-12),
            ("nA", 1e-9),
            ("uA", 1e-6),
            ("mA", 1e-3),
            ("A", 1.0),
        ],
    )
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

/// nom_voltage, else an operating_conditions `voltage :`, else None. `own` is the library's own
/// attributes; `text` the file, for its groups.
fn lib_voltage(own: &str, text: &str) -> Option<f64> {
    if let Some(v) = simple_attr(own, "nom_voltage").and_then(|s| lnum(&s)) {
        return Some(v);
    }
    let mut at = 0;
    while let Some((_, oc, after)) = next_block(text, at, "operating_conditions") {
        if let Some(v) = simple_attr(&oc, "voltage").and_then(|s| lnum(&s)) {
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
                if let Some(v) = lnum(tok) {
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
        assert_eq!(lib.voltage, 1.8f32 as f64, "read as strtof");
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
        assert!(
            a.recv.is_some(),
            "receiver_capacitance present on full parse"
        );
        assert_eq!(y.arcs.len(), 1);
        assert!(
            !y.arcs[0].ccs.is_empty(),
            "output_current present on full parse"
        );

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
        let js2 = Lib::parse_opts(CCS_LIB, LibOpts { skip_ccs: true })
            .unwrap()
            .to_json();
        assert!(js2.contains("\"has_recv_ccs\":false"));
        assert!(js2.contains("\"has_ccs\":false"));
    }
}

#[cfg(test)]
mod cache_tests {
    use super::*;

    const CACHE_LIB: &str = r#"
library (demo) {
  capacitive_load_unit (1, pf);
  cell (INV) {
    pin (A) {
      direction : input;
      capacitance : 0.004;
      receiver_capacitance () { receiver_capacitance1_rise (t) { values("0.001, 0.002"); } }
    }
    pin (Y) { direction : output; timing () { related_pin : "A"; cell_rise (t) { values("0.1, 0.2"); } } }
  }
}
"#;

    #[test]
    fn load_opts_is_consistent_and_opts_keyed() {
        std::env::set_var("VYGES_LIB_CACHE", "0"); // keep this test off the real ~/.vyges
        let path =
            std::env::temp_dir().join(format!("vyges_loom_cache_{}.lib", std::process::id()));
        std::fs::write(&path, CACHE_LIB).unwrap();
        let p = path.to_str().unwrap();

        // Two loads of the same file → identical Lib (cache hit is transparent).
        let a = Lib::load_opts(p, LibOpts::default()).unwrap();
        let b = Lib::load_opts(p, LibOpts::default()).unwrap();
        assert_eq!(a.cells.len(), b.cells.len());
        assert!(a.cell("INV").unwrap().pins.get("A").unwrap().recv.is_some());

        // skip_ccs is part of the cache key → a distinct entry with CCS pruned.
        let nldm = Lib::load_opts(p, LibOpts { skip_ccs: true }).unwrap();
        assert!(nldm
            .cell("INV")
            .unwrap()
            .pins
            .get("A")
            .unwrap()
            .recv
            .is_none());

        std::fs::remove_file(&path).ok();
    }
}

#[cfg(test)]
mod threshold_tests {
    use super::*;

    const BODY: &str = r#"
  cell (INV) { pin(A) { direction : input; capacitance : 0.001; }
               pin(Y) { direction : output; } }
}"#;

    #[test]
    fn slew_thresholds_are_read_from_the_library_not_assumed() {
        // A Liberty transition time is measured BETWEEN two percentage points, so the number
        // is meaningless without them. sky130 uses 20/80; 10/90 and 30/70 exist too, and
        // assuming one silently rescales every delay built from a slew.
        let text = format!(
            "library(t) {{\n  time_unit : \"1ns\";\n  capacitive_load_unit (1, pf);\n\
             slew_lower_threshold_pct_rise : 10.0;\n  slew_upper_threshold_pct_rise : 90.0;\n\
             slew_lower_threshold_pct_fall : 10.0;\n  slew_upper_threshold_pct_fall : 90.0;\n\
             slew_derate_from_library : 0.5;\n{BODY}"
        );
        let t = Lib::parse(&text).unwrap().thresholds;
        assert_eq!(
            t.slew_lower_rise, 0.10,
            "stored as a FRACTION, not a percentage"
        );
        assert_eq!(t.slew_upper_rise, 0.90);
        assert_eq!(t.slew_derate, 0.5);
        assert!(
            (t.slew_span() - 0.8).abs() < 1e-12,
            "10/90 spans 80% of a full edge"
        );
    }

    #[test]
    fn a_library_that_declares_nothing_gets_the_liberty_defaults() {
        let text = format!(
            "library(t) {{\n  time_unit : \"1ns\";\n  capacitive_load_unit (1, pf);\n{BODY}"
        );
        let t = Lib::parse(&text).unwrap().thresholds;
        assert_eq!((t.slew_lower_rise, t.slew_upper_rise), (0.2, 0.8));
        assert!((t.slew_span() - 0.6).abs() < 1e-12);
        assert_eq!(t.slew_derate, 1.0);
    }

    #[test]
    fn a_nonsense_threshold_declaration_does_not_rescale_every_delay() {
        // slew_span divides the driver ramp, so a zero or inverted span would scale every
        // delay in the design by infinity. Refuse it rather than propagate it.
        let t = Thresholds {
            slew_lower_rise: 0.9,
            slew_upper_rise: 0.1, // inverted
            slew_lower_fall: 0.9,
            slew_upper_fall: 0.1,
            ..Thresholds::default()
        };
        assert!(
            (t.slew_span() - 0.6).abs() < 1e-12,
            "falls back rather than returning <= 0"
        );
    }
}

#[cfg(test)]
mod async_check_tests {
    use super::*;

    /// A flop with an async reset, shaped like sky130's `dfrtp`: RESET_B carries
    /// recovery/removal checks and a clear *effect* arc, not data setup/hold.
    const DFRTP: &str = r#"library(t) {
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  cell (DFRTP) {
    ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
    pin(CLK) { direction : input; clock : true; capacitance : 0.001; }
    pin(D) { direction : input; capacitance : 0.001;
      timing () { related_pin : "CLK"; timing_type : setup_rising;
        rise_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.05"); }
        fall_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.05"); } }
      timing () { related_pin : "CLK"; timing_type : hold_rising;
        rise_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.02"); }
        fall_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.02"); } } }
    pin(RESET_B) { direction : input; capacitance : 0.001;
      timing () { related_pin : "CLK"; timing_type : recovery_rising;
        rise_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.11"); }
        fall_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.11"); } }
      timing () { related_pin : "CLK"; timing_type : removal_rising;
        rise_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.07"); }
        fall_constraint(s) { index_1("0.01"); index_2("0.01"); values("0.07"); } } }
    pin(Q) { direction : output; function : "IQ";
      timing () { related_pin : "CLK"; timing_type : rising_edge;
        cell_rise(t) { index_1("0.01"); index_2("0.001"); values("0.30"); }
        cell_fall(t) { index_1("0.01"); index_2("0.001"); values("0.30"); }
        rise_transition(t) { index_1("0.01"); index_2("0.001"); values("0.04"); }
        fall_transition(t) { index_1("0.01"); index_2("0.001"); values("0.04"); } }
      timing () { related_pin : "RESET_B"; timing_type : clear;
        cell_fall(t) { index_1("0.01"); index_2("0.001"); values("0.25"); }
        fall_transition(t) { index_1("0.01"); index_2("0.001"); values("0.05"); } } }
  }
}"#;

    #[test]
    fn an_async_reset_pin_carries_recovery_and_removal_not_setup_and_hold() {
        // These were previously parsed and DISCARDED, so an async pin got no check at
        // all. Applying the D pin's setup/hold tables to it instead is not an
        // approximation — it is a different constraint with different numbers.
        let lib = Lib::parse(DFRTP).unwrap();
        let rb = &lib.cells["DFRTP"].pins["RESET_B"];
        assert_eq!(rb.recovery.len(), 1, "recovery must be kept");
        assert_eq!(rb.removal.len(), 1, "removal must be kept");
        assert!(
            rb.setup.is_empty() && rb.hold.is_empty(),
            "not data setup/hold"
        );
        assert!((rb.recovery[0].eval(0.01, 0.01) - 0.11).abs() < 1e-9);
        assert!((rb.removal[0].eval(0.01, 0.01) - 0.07).abs() < 1e-9);

        let d = &lib.cells["DFRTP"].pins["D"];
        assert_eq!((d.setup.len(), d.hold.len()), (1, 1));
        assert!(
            d.recovery.is_empty() && d.removal.is_empty(),
            "D is data, not async"
        );
        // and the numbers really do differ, or checking the wrong one would be harmless
        assert_ne!(d.hold[0].eval(0.01, 0.01), rb.removal[0].eval(0.01, 0.01));
    }

    /// The `ff` group's `clear`/`preset` expressions name the ASYNC reset pins, and nothing
    /// else does. Without them there is no way to tell a reset-domain crossing from any other
    /// path, because a synchronous reset is just data and is timed like data.
    #[test]
    fn async_reset_pins_come_from_the_ff_group() {
        let lib = Lib::parse(
            r#"library(t) {
  cell (dfrtp) {
    ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
    pin (CLK) { direction : input; clock : true; }
    pin (D)   { direction : input; }
    pin (RESET_B) { direction : input; }
    pin (Q)   { direction : output; }
  }
  cell (dfstp) {
    ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; preset : "!SET_B"; clear : "!CLR"; }
    pin (CLK) { direction : input; clock : true; }
    pin (D)   { direction : input; }
    pin (SET_B) { direction : input; }
    pin (CLR) { direction : input; }
    pin (Q)   { direction : output; }
  }
  cell (dfxtp) {
    ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; }
    pin (CLK) { direction : input; clock : true; }
    pin (D)   { direction : input; }
    pin (Q)   { direction : output; }
  }
  cell (nand2) {
    pin (A) { direction : input; }
    pin (B) { direction : input; }
    pin (Y) { direction : output; }
  }
}
"#,
        )
        .unwrap();
        assert_eq!(
            lib.cells["dfrtp"].async_reset_pins,
            vec!["RESET_B"],
            "polarity stripped"
        );
        assert_eq!(
            lib.cells["dfstp"].async_reset_pins,
            vec!["CLR", "SET_B"],
            "both clear and preset, sorted"
        );
        assert!(
            lib.cells["dfxtp"].async_reset_pins.is_empty(),
            "a flop with no async reset has none — not an empty-string entry"
        );
        assert!(
            lib.cells["nand2"].async_reset_pins.is_empty(),
            "combinational cells have no ff group at all"
        );
    }

    #[test]
    fn a_clear_arc_is_still_not_a_data_launch_path() {
        // RESET_B -> Q is an async *effect*, not a max-delay arc. Propagating data
        // through it would invent a launch path that cannot exist.
        let lib = Lib::parse(DFRTP).unwrap();
        let q = &lib.cells["DFRTP"].pins["Q"];
        assert_eq!(
            q.arcs.len(),
            1,
            "only the CLK->Q edge arc: {:?}",
            q.arcs.iter().map(|a| &a.related_pin).collect::<Vec<_>>()
        );
        assert_eq!(q.arcs[0].related_pin, "CLK");
    }

    #[test]
    fn the_real_sky130_flop_agrees_with_that_shape() {
        // Guards against the fixture being wishful. Skips when the PDK is absent.
        let path = concat!(
            env!("HOME"),
            "/.ciel/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
        );
        let Ok(lib) = Lib::load(path) else { return };
        let Some(cell) = lib.cells.get("sky130_fd_sc_hd__dfrtp_1") else {
            return;
        };
        let rb = cell.pins.get("RESET_B").expect("dfrtp has RESET_B");
        assert!(
            !rb.removal.is_empty(),
            "sky130 dfrtp RESET_B must carry removal"
        );
        assert!(
            !rb.recovery.is_empty(),
            "sky130 dfrtp RESET_B must carry recovery"
        );
        assert!(rb.hold.is_empty(), "and must NOT be a data hold pin");
    }
}

#[cfg(test)]
mod constant_tests {
    use super::*;

    #[test]
    fn a_tie_cell_output_is_recognised_as_constant() {
        // sky130's conb_1 shape: two outputs tied high and low, no timing arcs.
        let lib = Lib::parse(
            r#"library(t) {
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  cell (CONB) {
    pin("HI") { direction : "output"; function : "1"; }
    pin("LO") { direction : "output"; function : "0"; }
  }
  cell (INV) {
    pin(A) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "!A"; }
  }
}"#,
        )
        .unwrap();
        let c = &lib.cells["CONB"];
        assert!(c.pins["HI"].is_constant(), "function \"1\" is a tie-high");
        assert!(c.pins["LO"].is_constant(), "function \"0\" is a tie-low");
        // an ordinary gate is not constant, however simple its function
        assert!(!lib.cells["INV"].pins["Y"].is_constant());
        // nor is an input pin, whatever it says
        assert!(!lib.cells["INV"].pins["A"].is_constant());
    }

    #[test]
    fn the_real_sky130_tie_cell_is_recognised() {
        let path = concat!(
            env!("HOME"),
            "/.ciel/sky130A/libs.ref/sky130_fd_sc_hd/lib/sky130_fd_sc_hd__tt_025C_1v80.lib"
        );
        let Ok(lib) = Lib::load(path) else { return };
        let Some(c) = lib.cells.get("sky130_fd_sc_hd__conb_1") else {
            return;
        };
        assert!(c.pins["HI"].is_constant() && c.pins["LO"].is_constant());
        // and a real logic cell in the same library is not swept up by it
        if let Some(inv) = lib.cells.get("sky130_fd_sc_hd__inv_2") {
            assert!(!inv.pins["Y"].is_constant());
        }
    }
}

#[cfg(test)]
mod cell_structure_tests {
    use super::*;

    // Synthetic library, written for these rules.
    const LIB: &str = r#"library (gaps) {
      capacitive_load_unit (1, pf);
      type (bus4) { base_type : array; data_type : bit; bit_width : 4; bit_from : 3; bit_to : 0; downto : true; }
      cell (scanff) {
        ff (IQ, IQN) { clocked_on : "CK"; next_state : "(D&!SE)|(SI&SE)"; }
        pin (CK) { direction : input; clock : true; capacitance : 0.002; }
        pin (D) { direction : input; capacitance : 0.001; }
        pin (SI) { direction : input; capacitance : 0.001; }
        pin (SE) { direction : input; capacitance : 0.001; }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.1"); } } }
        test_cell () {
          ff (IQ, IQN) { clocked_on : "CK"; next_state : "D"; }
          pin (CK) { direction : input; }
          pin (D) { direction : input; signal_type : not_a_scan_role; }
          pin (SI) { direction : input; signal_type : test_scan_in; }
          pin (SE) { direction : input; signal_type : test_scan_enable; }
          pin (Q) { direction : output; function : "IQ"; signal_type : test_scan_out; }
        }
      }
      cell (sync3) {
        statetable ("CK D", "Q1") { table : "R L : - : L, R H : - : H, ~R - : - : N"; }
        pin (CK) { direction : input; clock : true; }
        pin (D) { direction : input; }
        pin (Q) { direction : output; state_function : "Q1"; }
      }
      cell (icg) {
        clock_gating_integrated_cell : "latch_posedge";
        pin (CLK) { direction : input; clock : true; clock_gate_clock_pin : true; }
        pin (EN) { direction : input; clock_gate_enable_pin : true; }
        pin (GCLK) { direction : output; clock_gate_out_pin : true; }
      }
      cell (sram) {
        pin (CLK) { direction : input; clock : true; }
        bus (D) { bus_type : bus4; direction : input; capacitance : 0.003;
          pin (D[0]) { capacitance : 0.005; }
          pin (D[3:2]) { capacitance : 0.007; } }
        bus (Q) { bus_type : bus4; direction : output; }
        bundle (DM) { members (DM0, DM1); direction : input; capacitance : 0.004; }
      }
    }"#;

    // Rule: a `test_cell` describes the scan-mode view; its pins never replace the cell's own.
    #[test]
    fn test_cell_pins_do_not_replace_the_real_ones() {
        let l = Lib::parse(LIB).unwrap();
        let c = l.cell("scanff").unwrap();
        assert_eq!(c.clock_pin.as_deref(), Some("CK"));
        assert!(c.pins["CK"].clock);
        assert_eq!(c.pins["CK"].capacitance, 0.002f32 as f64);
        assert_eq!(c.pins["Q"].arcs.len(), 1, "the real Q, with its arc");
        assert!(c.is_seq);
    }

    // Rule: sequential = ff/latch (or a bank), a statetable, or a clock pin on a non-clock-gate.
    #[test]
    fn a_statetable_cell_is_sequential_and_a_clock_gate_needs_its_latch() {
        let l = Lib::parse(LIB).unwrap();
        assert!(l.cell("sync3").unwrap().is_seq, "statetable");
        assert!(!l.cell("icg").unwrap().is_seq, "a clock gate's clock pin alone is not state");
        assert!(l.cell("sram").unwrap().is_seq, "a clock pin on a non-clock-gate (the SRAM)");
    }

    // Rule: a bus is its bits `name[i]`, bit_from to bit_to; the bus's attributes reach every bit;
    // a pin group inside (one bit or a range) states its own. A bundle is its members likewise.
    #[test]
    fn bus_and_bundle_members_get_the_groups_direction() {
        let l = Lib::parse(LIB).unwrap();
        let c = l.cell("sram").unwrap();
        for b in 0..4 {
            assert_eq!(c.pins[&format!("D[{b}]")].direction, Dir::In);
            assert_eq!(c.pins[&format!("Q[{b}]")].direction, Dir::Out);
        }
        assert_eq!(c.pins["D[1]"].capacitance, 0.003f32 as f64, "the bus's");
        assert_eq!(c.pins["D[0]"].capacitance, 0.005f32 as f64, "the inner pin's own");
        assert_eq!(c.pins["D[3]"].capacitance, 0.007f32 as f64, "a range names each bit");
        assert_eq!(c.pins["D[2]"].capacitance, 0.007f32 as f64);
        assert_eq!(c.pins["DM1"].direction, Dir::In);
        assert_eq!(c.pins["DM0"].capacitance, 0.004f32 as f64);
        assert!(!c.pins.contains_key("D") && !c.pins.contains_key("DM"), "the group is not a pin");
    }

    // Rule: `clock_gating_integrated_cell` is exposed as written; a cell without it has `None`.
    #[test]
    fn a_clock_gate_says_so() {
        let l = Lib::parse(LIB).unwrap();
        assert_eq!(l.cell("icg").unwrap().clock_gating.as_deref(), Some("latch_posedge"));
        assert_eq!(l.cell("scanff").unwrap().clock_gating, None);
    }

    // Rule: scan roles are the `test_cell` pins' `signal_type`, and the only thing taken from it;
    // a value outside the nine `test_scan_*` roles is ignored (D's `not_a_scan_role`).
    #[test]
    fn scan_roles_come_from_the_test_cell() {
        let l = Lib::parse(LIB).unwrap();
        let c = l.cell("scanff").unwrap();
        let roles: Vec<(&str, &str)> = c.scan_pins.iter().map(|(k, v)| (k.as_str(), v.as_str())).collect();
        assert_eq!(roles, [("Q", "test_scan_out"), ("SE", "test_scan_enable"), ("SI", "test_scan_in")]);
        assert!(l.cell("sync3").unwrap().scan_pins.is_empty());
    }
}

/// The lexer's rules, each the reference's (`LibertyLex.ll`).
#[cfg(test)]
mod lexer_tests {
    use super::*;

    const LIB: &str = "library (l) {\n  capacitive_load_unit (1, pf);\n\
        /* cell (GHOST) { area : 9.0; pin (A) { direction : input; } } */\n\
        cell (INV) {\n    area : 2.0; /* area : 7.0; */\n\
        pin (A) { direction : input; capacitance : 0.1; }\n\
        pin (Y) { direction : output; function : \"!A\";\n\
          timing () { related_pin : \"A\";\n\
            cell_rise (scalar) { index_1 (\"0.01, \\\n 0.02\"); values (\"0.1, \\\n 0.2\"); } } }\n  }\n}\n";

    /// A comment is not text: a commented-out cell or attribute is not read.
    #[test]
    fn comments_are_not_read() {
        let l = Lib::parse(LIB).unwrap();
        assert!(l.cell("GHOST").is_none());
        assert_eq!(l.cell("INV").unwrap().area, 2.0);
    }

    /// A backslash-newline inside a quoted string is a continuation.
    #[test]
    fn a_continued_string_is_one_string() {
        let l = Lib::parse(LIB).unwrap();
        let t = &l.cell("INV").unwrap().pins["Y"].arcs[0].cell_rise;
        assert_eq!(t.index_1, [0.01f32 as f64, 0.02f32 as f64]);
        assert_eq!(t.values[0], [0.1f32 as f64, 0.2f32 as f64]);
    }

    /// A simple attribute is the LAST one stated at the group's own level; an inner group's
    /// attribute of the same name does not answer for it.
    #[test]
    fn the_last_attribute_at_its_own_level_wins() {
        let lib = LIB.replace("area : 2.0;", "area : 2.0; area : 3.0; leakage_power () { area : 9.0; value : 1; }");
        assert_eq!(Lib::parse(&lib).unwrap().cell("INV").unwrap().area, 3.0);
        let nested_only = LIB.replace("area : 2.0;", "leakage_power () { area : 9.0; value : 1; }");
        assert_eq!(Lib::parse(&nested_only).unwrap().cell("INV").unwrap().area, 0.0);
    }

    /// A number is `strtof`: one rounding, straight to f32.
    #[test]
    fn a_number_is_read_as_strtof() {
        assert_eq!(lnum("0.1"), Some(0.1f32 as f64));
        assert_ne!(lnum("0.1"), Some(0.1));
        // a cleaned file keeps its layout byte for byte
        let mut b = LIB.as_bytes().to_vec();
        lex_clean(&mut b);
        assert_eq!(b.len(), LIB.len());
        assert_eq!(b.iter().filter(|&&c| c == b'\n').count() + 2, LIB.matches('\n').count());
    }
}
