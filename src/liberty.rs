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
///
/// A timing CHECK names its clock edge the same way (`setup_rising` / `hold_falling` …): see
/// [`Constraint::clk_edge`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ClkEdge {
    #[default]
    Rise,
    Fall,
}

/// Which OUTPUT edges a delay arc drives.
///
/// `combinational_rise` / `_fall` and `three_state_*_rise` / `_fall` build only the named edge,
/// as the reference builds them; every other delay type builds both. ⛔ Without it a one-edge
/// arc read as two, and the edge it does not have — its table empty — timed as ZERO delay: a
/// clock gate's CLK->GCLK, among others.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutEdges {
    #[default]
    Both,
    Rise,
    Fall,
}

impl OutEdges {
    /// Does the arc drive the output edge `rising` (true) or falling (false)?
    pub fn drives(self, rising: bool) -> bool {
        match self {
            OutEdges::Both => true,
            OutEdges::Rise => rising,
            OutEdges::Fall => !rising,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Arc {
    /// The ONE pin this arc is timed from. A timing group's `related_pin` may name several pins
    /// and buses; the reader makes one arc per bit, as the reference does (see `expand_arcs`).
    pub related_pin: String,
    /// `related_bus_pins` as read, until `expand_arcs` turns it into arcs (then empty).
    pub(crate) related_bus_pins: String,
    /// The group's `timing_type` as read (empty when absent), for `derive_senses`.
    pub(crate) kind: String,
    pub sense: String,
    /// The output edges this arc drives — see [`OutEdges`].
    pub out_edges: OutEdges,
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

impl Arc {
    /// Does an input edge (`from_rise`) launch the output edge (`to_rise`) through this arc — the
    /// reference's timing arcs (`LibertyBuilder`), which pair edges by the arc's KIND as well as its
    /// sense. A caller times `to_rise` only where [`OutEdges::drives`] it.
    ///
    /// - a sequential arc launches on its clock edge, to either output edge;
    /// - a tri-state enable or disable arc (`makeTristateEnableArcs` / `…DisableArcs`) launches
    ///   on ONE input edge to both output edges — the rise when positive, the fall when negative
    ///   (Z->1 / 0->Z are timed with the rise table, Z->0 / 1->Z with the fall table);
    /// - a latch D-to-Q arc (`makeLatchDtoQArcs`) is negative only when the latch's `data_in` is,
    ///   and otherwise pairs like edges — non-unate included;
    /// - any other arc: positive pairs like edges, negative opposite ones, non-unate every pair.
    pub fn feeds(&self, from_rise: bool, to_rise: bool) -> bool {
        if let Some(e) = self.clk_edge {
            return from_rise == (e == ClkEdge::Rise);
        }
        match (self.kind.as_str(), self.sense.as_str()) {
            (k, "positive_unate") if k.starts_with("three_state_") => from_rise,
            (k, "negative_unate") if k.starts_with("three_state_") => !from_rise,
            ("latch_d_to_q", "negative_unate") => from_rise != to_rise,
            ("latch_d_to_q", _) => from_rise == to_rise,
            (_, "positive_unate") => from_rise == to_rise,
            (_, "negative_unate") => from_rise != to_rise,
            _ => true,
        }
    }
}

/// A setup or hold constraint: rise/fall tables indexed by
/// (index_1 = related/clock transition, index_2 = constrained/data transition).
/// Evaluated by bilinear interpolation at the operating slews (like delay arcs),
/// not collapsed to a table-max — as the reference does.
#[derive(Debug, Clone, Default)]
pub struct Constraint {
    pub rise: Table,
    pub fall: Table,
    /// The clock edge the check is made at: `Rise` for `setup_rising` / `hold_rising` /
    /// `recovery_rising` / `removal_rising`, `Fall` for the `_falling` forms. The reference
    /// builds the check arc from that edge of the related pin, and its cycle accounting
    /// pairs the launching edge with THIS one — a negative-edge register is captured half a
    /// period from a rising launch.
    pub clk_edge: ClkEdge,
    /// The pin the check is made against (`related_pin`), as written: a cell with two clock
    /// pins (a dual-port SRAM) checks each data pin against its own.
    pub related_pin: String,
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

/// A pin's capacitance per output edge and per analysis (min/max), in library units — the
/// reference's four values.
///
/// `capacitance` sets all four; `rise_capacitance` / `fall_capacitance` override their edge;
/// `rise_capacitance_range (min, max)` / `fall_…` override min and max apart; and whatever is
/// still unset is the library's `default_{input,output,inout}_pin_cap` for the pin's direction.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct PinCaps {
    pub rise_min: f64,
    pub rise_max: f64,
    pub fall_min: f64,
    pub fall_max: f64,
}

impl PinCaps {
    /// The capacitance for one edge (`rising`) in a late (`max`) or early analysis.
    pub fn get(&self, rising: bool, max: bool) -> f64 {
        match (rising, max) {
            (true, true) => self.rise_max,
            (true, false) => self.rise_min,
            (false, true) => self.fall_max,
            (false, false) => self.fall_min,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Pin {
    pub name: String,
    pub direction: Dir,
    /// Input capacitance in library units (timing/NLDM load axis): the stated `capacitance`,
    /// else the largest of [`Pin::caps`] — one figure for a consumer that does not time per edge.
    /// ⛔ It was 0 for a pin that states only `rise_`/`fall_capacitance`, or nothing at all.
    pub capacitance: f64,
    /// Per edge and analysis — see [`PinCaps`]. What the reference times with.
    pub caps: PinCaps,
    pub cap_f: f64,       // same capacitance in Farads (power: net-load summation)
    pub recv: Option<RecvCap>, // CCS receiver model (input pins); None -> use `capacitance`
    pub clock: bool,      // `clock : true` — the cell's clock pin
    /// A REGISTER CLOCK by its arcs, whatever `clock` says (`LibertyPort::isRegClk`): the pin a
    /// `rising_edge` / `falling_edge` arc launches from, as a register clock-to-Q or a latch
    /// enable-to-Q — see [`edge_arc_role`]. `clock : true` is optional in Liberty; a hard macro
    /// (a launch pin on a hard macro, a PHY, some memories) often leaves it off.
    ///
    /// ⚠️ Not classified by an `ff_bank` / `latch_bank` group: an edge arc from a bank's clear or
    /// preset pin reads as a register clock-to-Q here.
    pub is_reg_clk: bool,
    /// The output such an arc drives (`LibertyPort::isRegOutput`).
    pub is_reg_output: bool,
    /// A CHECK CLOCK (`LibertyPort::isCheckClk`): the pin a timing check is made against — the
    /// `related_pin` of a setup, hold, recovery, removal, skew or non-sequential check — and a pin
    /// with a pulse-width or period check on itself (`min_pulse_width`, `minimum_period`, the
    /// `min_pulse_width_high/low` attributes), whose check arc runs from the pin to itself. Set
    /// whether or not the group carries tables, as the reference sets it.
    ///
    /// ⚠️ So a pin with a pulse-width check on ITSELF is a check clock too (a flop's RESET_B with
    /// `min_pulse_width_low`), as it is in the reference. Tracing clocks, take [`Pin::is_reg_clk`];
    /// this adds the pins checks are made against.
    pub is_check_clk: bool,
    /// While reading: each check group's `related_pin` / `related_bus_pins` names, as written, and
    /// `""` for a check on the pin itself — resolved to [`Pin::is_check_clk`] at the cell.
    pub(crate) check_from: Vec<String>,
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
    /// What the reader does not model on this pin, as written, once each: CHECK `timing_type`s
    /// (`non_seq_setup_rising`, `skew_*`, `min_pulse_width`, `minimum_period`) and an arc's
    /// `retaining_rise` / `retaining_fall` contamination tables. Not applied — kept so a timer can
    /// SAY what the reference does that it left out, rather than drop it in silence.
    pub unmodelled_checks: Vec<String>,
    /// `three_state` (the DISABLE expression), for `derive_senses`.
    pub(crate) three_state: Option<String>,
    /// While reading: the pins a set/clear arc INTO this pin is timed from (`timing_type : clear` /
    /// `preset` with its table), for `preset_clear_check_roles`.
    pub(crate) set_clr_from: Vec<String>,
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
    /// The pin marked `clock : true` — an optional attribute: a macro may mark none, or launch
    /// from several. Which pins ARE clocks by their arcs is [`Pin::is_reg_clk`] /
    /// [`Pin::is_check_clk`], and [`Cell::reg_clk_pins`] lists the first.
    pub clock_pin: Option<String>,
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
    /// `dont_use : true` — the library withholds the cell from optimisation. Read as the
    /// reference reads a cell boolean: exactly `true` or `false`, anything else leaves the
    /// default (`false`). A design may still instantiate the cell; only a resize/insert move
    /// may not choose it (see [`Lib::equivalence_class`]).
    pub dont_use: bool,
    /// Its own library's slew and delay thresholds, when they are not the merged library's
    /// ([`Lib::merge`] sets it on a later library's cells whose thresholds differ). The reference
    /// times a driver with its own library's thresholds and converts each wire delay and load
    /// slew to the load's (`thresholdAdjust`). `None`: the library's own ([`Lib::cell_thresholds`]).
    pub thresholds: Option<Thresholds>,
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
    /// A `dont_use` cell is never offered as a replacement — the reference's resizer skips it
    /// in every candidate list — but `cell` itself stays in the ladder even when it is one, so
    /// a caller can still see where it sits.
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
            .filter(|o| o.name == c.name || (!o.dont_use && interchangeable(c, o)))
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
    /// Every pin a register clock-to-Q or latch enable-to-Q arc launches from
    /// ([`Pin::is_reg_clk`]), in name order — none, one, or several, with or without `clock : true`.
    pub fn reg_clk_pins(&self) -> Vec<&str> {
        self.pins.iter().filter(|(_, p)| p.is_reg_clk).map(|(n, _)| n.as_str()).collect()
    }

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
    /// The units its times and capacitances are in (see [`LibUnits`]).
    pub units: LibUnits,
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
/// A boolean attribute: `true` / `false` in ANY case, as the reference reads them (`getAttrBool`
/// compares with `stringEqual`, which folds case). ⛔ An exact `== "true"` missed gf180's SRAM macro,
/// which says `dont_use : TRUE;`.
fn bool_attr(body: &str, key: &str) -> bool {
    simple_attr(body, key).is_some_and(|v| v.eq_ignore_ascii_case("true"))
}

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
    let edits = lex_edits(b);
    blank(b, edits);
}

/// Apply [`lex_edits`]: each byte becomes a space, a comment keeping its newlines.
fn blank(b: &mut [u8], edits: Vec<(std::ops::Range<usize>, bool)>) {
    for (r, keep_newlines) in edits {
        for c in &mut b[r] {
            if !(keep_newlines && *c == b'\n') {
                *c = b' ';
            }
        }
    }
}

/// What [`lex_clean`] blanks: `(range, keep its newlines)`, in order. Empty for a file with
/// nothing to clean, which is then not copied.
///
/// ⚡ It jumps between the only bytes that can start an edit — `/` and `\` — and tracks whether
/// it is inside a string by counting the quotes in each gap. CCS tables are dense with quoted
/// rows; a walk over every byte cost a large library a third of a second, this about half that.
fn lex_edits(b: &[u8]) -> Vec<(std::ops::Range<usize>, bool)> {
    let mut edits = Vec::new();
    let (mut at, mut quoted) = (0usize, false);
    while let Some(k) = find2(b, at, b'/', b'\\') {
        quoted ^= b[at..k].iter().filter(|&&c| c == b'"').count() % 2 == 1;
        let next = b.get(k + 1).copied();
        if b[k] == b'\\' {
            if quoted && matches!(next, Some(b'\n' | b'\r')) {
                // `\r\n` counts as one line end
                let end = if next == Some(b'\r') && b.get(k + 2) == Some(&b'\n') { k + 3 } else { k + 2 };
                edits.push((k..end, false));
                at = end;
            } else if quoted {
                // an escaped byte — a quote included — is not a toggle
                at = (k + 2).min(b.len());
            } else {
                at = k + 1;
            }
        } else if !quoted && next == Some(b'*') {
            let mut j = k + 2;
            while j < b.len() && !(b[j] == b'*' && b.get(j + 1) == Some(&b'/')) {
                j += 1;
            }
            let end = (j + 2).min(b.len());
            edits.push((k..end, true));
            at = end;
        } else {
            at = k + 1;
        }
    }
    edits
}

/// The next index at or after `from` holding `x` or `y`, eight bytes at a time.
fn find2(b: &[u8], from: usize, x: u8, y: u8) -> Option<usize> {
    const LO: u64 = 0x0101_0101_0101_0101;
    const HI: u64 = 0x8080_8080_8080_8080;
    let (bx, by) = (LO * x as u64, LO * y as u64);
    let mut i = from;
    while i + 8 <= b.len() {
        let w = u64::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3], b[i + 4], b[i + 5], b[i + 6], b[i + 7]]);
        let (p, q) = (w ^ bx, w ^ by);
        // a zero byte in either word means a match in this word
        if ((p.wrapping_sub(LO) & !p) | (q.wrapping_sub(LO) & !q)) & HI != 0 {
            break;
        }
        i += 8;
    }
    b.get(i..)?.iter().position(|&c| c == x || c == y).map(|k| i + k)
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
    // values ( "a, b", "c, d" ): each quoted string is a row. A BARE number is a row of its own
    // (the reference's rule), so `values (0.1, 0.2)` is two one-entry rows — `orient` then pads
    // them to the axes as the reference does. ⛔ Bare numbers were dropped: the table came back
    // empty and the arc lost that edge's delay without a word.
    let values = next_paren_after(body, "values")
        .map(|v| {
            let mut rows = Vec::new();
            let b = v.as_bytes();
            let mut k = 0;
            while k < b.len() {
                match b[k] {
                    b'"' => {
                        let end = v[k + 1..].find('"').map_or(b.len(), |e| k + 1 + e);
                        rows.push(floats(&v[k + 1..end.min(b.len())]));
                        k = end + 1;
                    }
                    c if c == b'-' || c == b'+' || c == b'.' || c.is_ascii_digit() => {
                        let end = v[k..].find(|ch: char| ch == ',' || ch.is_whitespace() || ch == '"').map_or(b.len(), |e| k + e);
                        rows.push(floats(&v[k..end]));
                        k = end;
                    }
                    _ => k += 1,
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

fn parse_arc(timing_body: &str, own: &str, spans: &[GroupSpan], ctx: &Ctx) -> Arc {
    let tbl = |name: &str| span_table(timing_body, spans, name, ctx, Axes::SlewLoad);
    // `rising_edge` / `falling_edge` name the launching clock edge. Anything else is a
    // combinational arc; the checks (setup_*/hold_*/recovery_*/removal_*) never reach
    // here — the caller routes them into the constraint vectors before this point.
    let clk_edge = match simple_attr(own, "timing_type").as_deref() {
        Some("rising_edge") => Some(ClkEdge::Rise),
        Some("falling_edge") => Some(ClkEdge::Fall),
        _ => None,
    };
    let tt = simple_attr(own, "timing_type").unwrap_or_default();
    let out_edges = if tt.ends_with("_rise") {
        OutEdges::Rise
    } else if tt.ends_with("_fall") {
        OutEdges::Fall
    } else {
        OutEdges::Both
    };
    Arc {
        related_pin: simple_attr(own, "related_pin").unwrap_or_default(),
        related_bus_pins: simple_attr(own, "related_bus_pins").unwrap_or_default(),
        kind: tt.clone(),
        // empty = not stated: `derive_senses` takes it from the pin's function, as the reference does
        sense: simple_attr(own, "timing_sense").unwrap_or_default(),
        out_edges,
        clk_edge,
        cell_rise: tbl("cell_rise"),
        cell_fall: tbl("cell_fall"),
        rise_transition: tbl("rise_transition"),
        fall_transition: tbl("fall_transition"),
        // CCS output_current waveforms — skipped (empty) for NLDM-only parses.
        ccs_declared: spans.iter().any(|g| g.kw == "output_current_rise" || g.kw == "output_current_fall"),
        ccs: if ctx.skip_ccs {
            crate::ccs::CcsArc::default()
        } else {
            parse_ccs(timing_body, ctx.ccs_current)
        },
        sigma_rise: tbl("ocv_sigma_cell_rise"),
        sigma_fall: tbl("ocv_sigma_cell_fall"),
    }
}

/// Parse CCS `output_current_rise`/`output_current_fall` waveforms from an arc, each current
/// scaled by `current` into the library's capacitance unit per time unit.
fn parse_ccs(timing_body: &str, current: f64) -> crate::ccs::CcsArc {
    let mut arc = crate::ccs::CcsArc {
        rise: parse_ccs_set(timing_body, "output_current_rise"),
        fall: parse_ccs_set(timing_body, "output_current_fall"),
    };
    if current != 1.0 {
        for w in arc.rise.iter_mut().chain(arc.fall.iter_mut()) {
            w.current.iter_mut().for_each(|i| *i *= current);
        }
    }
    arc
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
fn parse_constraint(timing_body: &str, spans: &[GroupSpan], ctx: &Ctx, own: &str, timing_type: &str) -> Constraint {
    Constraint {
        rise: span_table(timing_body, spans, "rise_constraint", ctx, Axes::RelatedConstrained),
        fall: span_table(timing_body, spans, "fall_constraint", ctx, Axes::RelatedConstrained),
        clk_edge: if timing_type.ends_with("_falling") { ClkEdge::Fall } else { ClkEdge::Rise },
        related_pin: simple_attr(own, "related_pin").unwrap_or_default(),
    }
}

fn parse_pin(name: String, body: &str, ctx: &Ctx) -> Pin {
    let (cap_unit_f, skip_ccs) = (ctx.cap_f, ctx.skip_ccs);
    let (own, pin_groups) = level(body);
    let direction = match simple_attr(&own, "direction").as_deref() {
        Some("input") => Dir::In,
        Some("output") => Dir::Out,
        Some("inout") => Dir::Inout,
        _ => Dir::Other,
    };
    let caps = pin_caps(&own, direction, ctx.default_cap);
    let capacitance = simple_attr(&own, "capacitance")
        .and_then(|s| lnum(&s))
        .unwrap_or_else(|| caps.rise_max.max(caps.fall_max));
    let cap_f = capacitance * cap_unit_f;
    // CCS receiver capacitance group (input pins): the Miller-aware two-segment load.
    // Skipped for NLDM-only parses (consumers fall back to lumped Ceff).
    let recv = if skip_ccs {
        None
    } else {
        pin_groups.iter().find(|g| g.kw == "receiver_capacitance").map(|g| {
            let rbody = &body[g.body.clone()];
            let (_, rspans) = level(rbody);
            let tbl = |name: &str| span_table(rbody, &rspans, name, ctx, Axes::SlewLoad);
            RecvCap {
                c1_rise: tbl("receiver_capacitance1_rise"),
                c2_rise: tbl("receiver_capacitance2_rise"),
                c1_fall: tbl("receiver_capacitance1_fall"),
                c2_fall: tbl("receiver_capacitance2_fall"),
            }
        })
    };
    let clock = bool_attr(&own, "clock");
    // Liberty puts `function` on the OUTPUT pin, not the cell. It is what makes two cells
    // comparable: a resize must not change what a gate computes.
    let function = simple_attr(&own, "function").filter(|f| !f.is_empty());
    let three_state = simple_attr(&own, "three_state").filter(|f| !f.is_empty());
    let mut arcs = Vec::new();
    let mut setup: Vec<Constraint> = Vec::new();
    let mut hold: Vec<Constraint> = Vec::new();
    let mut recovery: Vec<Constraint> = Vec::new();
    let mut removal: Vec<Constraint> = Vec::new();
    let mut unmodelled_checks: Vec<String> = Vec::new();
    let mut set_clr_from: Vec<String> = Vec::new();
    // `readMinPulseWidth`: a `min_pulse_width_high` / `_low` attribute builds a width check arc
    // from the pin to itself.
    let mut check_from: Vec<String> = Vec::new();
    if ["min_pulse_width_high", "min_pulse_width_low"].iter().any(|k| simple_attr(&own, k).is_some()) {
        check_from.push(String::new());
    }
    for g in pin_groups.iter().filter(|g| g.kw == "timing") {
        let tbody = &body[g.body.clone()];
        let (town, tspans) = level(tbody);
        // The groups the reference builds a timing-CHECK arc for, from each `related_pin` /
        // `related_bus_pins` name (none: no arc, a warning) — a pulse width or a period with no
        // related pin is made on the pin itself. `nochange_*` builds nothing.
        if let Some(tt) = simple_attr(&town, "timing_type") {
            let on_self = tt == "min_pulse_width" || tt == "minimum_period";
            let check = on_self
                || ["setup_", "hold_", "recovery_", "removal_", "skew_", "non_seq_setup_", "non_seq_hold_"]
                    .iter()
                    .any(|k| tt.starts_with(k) && matches!(&tt[k.len()..], "rising" | "falling"));
            if check {
                let names: Vec<String> = ["related_pin", "related_bus_pins"]
                    .iter()
                    .filter_map(|k| simple_attr(&town, k))
                    .flat_map(|v| v.split_whitespace().map(|n| n.trim_matches('"').to_string()).collect::<Vec<_>>())
                    .filter(|n| !n.is_empty())
                    .collect();
                if names.is_empty() && on_self {
                    check_from.push(String::new());
                }
                check_from.extend(names);
            }
        }
        // `retaining_rise/_fall` (+ `retain_*_slew`): the arc's contamination delay, which the
        // reference times the MIN analysis with. Not read; named, so a timer can say so.
        for (kw, _, _) in top_groups(tbody) {
            if (kw == "retaining_rise" || kw == "retaining_fall") && !unmodelled_checks.contains(&kw) {
                unmodelled_checks.push(kw);
            }
        }
        match simple_attr(&town, "timing_type").as_deref() {
            Some(tt) if tt.starts_with("setup") => setup.push(parse_constraint(tbody, &tspans, ctx, &town, tt)),
            Some(tt) if tt.starts_with("hold") => hold.push(parse_constraint(tbody, &tspans, ctx, &town, tt)),
            // recovery/removal are CHECKS on the async set/reset pin, with their own
            // tables — kept, and applied by the timer as the async counterparts of
            // setup/hold. They are not max-delay data arcs.
            Some(tt) if tt.starts_with("recovery") => recovery.push(parse_constraint(tbody, &tspans, ctx, &town, tt)),
            Some(tt) if tt.starts_with("removal") => removal.push(parse_constraint(tbody, &tspans, ctx, &town, tt)),
            // DELAY arcs, by the reference's own list; a
            // missing `timing_type` is combinational.
            None
            | Some(
                "combinational" | "combinational_rise" | "combinational_fall" | "rising_edge"
                | "falling_edge" | "three_state_enable" | "three_state_enable_rise"
                | "three_state_enable_fall" | "three_state_disable" | "three_state_disable_rise"
                | "three_state_disable_fall",
            ) => arcs.push(parse_arc(tbody, &town, &tspans, ctx)),
            // Everything else is NOT a delay arc, and read as one it was a zero-delay path that
            // does not exist — onto an input pin, or a clock pin to itself:
            //   * clear / preset are async EFFECT arcs (dfrtp RESET_B->Q); the reference builds
            //     them as set/clear arcs its timer leaves off by default;
            //   * skew, non_seq_setup/hold, min_pulse_width, minimum_period are checks this
            //     reader does not model;
            //   * nochange_*, retaining_time and unknown types the reference drops itself.
            // a set/clear arc exists only with the table of the edge it drives (`makePresetClrArcs`:
            // a preset drives the rise, a clear the fall)
            Some(tt @ ("clear" | "preset")) => {
                let a = parse_arc(tbody, &town, &tspans, ctx);
                let t = if tt == "preset" { &a.cell_rise } else { &a.cell_fall };
                if !t.values.is_empty() {
                    let names = format!("{} {}", a.related_pin, a.related_bus_pins);
                    set_clr_from.extend(names.split(|c: char| c.is_whitespace() || c == ',').map(|n| n.trim_matches('"')).filter(|n| !n.is_empty()).map(String::from));
                }
            }
            Some(tt) => {
                let check = ["non_seq_setup", "non_seq_hold", "skew_", "min_pulse_width", "minimum_period"].iter().any(|k| tt.starts_with(k));
                if check && !unmodelled_checks.iter().any(|c| c == tt) {
                    unmodelled_checks.push(tt.to_string());
                }
            }
        }
    }
    Pin {
        name,
        direction,
        capacitance,
        caps,
        cap_f,
        recv,
        clock,
        function,
        three_state,
        set_clr_from,
        setup,
        hold,
        recovery,
        removal,
        arcs,
        unmodelled_checks,
        is_reg_clk: false,
        is_reg_output: false,
        is_check_clk: false,
        check_from,
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

/// The first group named `kw` among `spans` of `body`, as a parsed table, its axes in this
/// reader's order for `kind` (see [`orient`]).
fn span_table(body: &str, spans: &[GroupSpan], kw: &str, ctx: &Ctx, kind: Axes) -> Table {
    spans
        .iter()
        .find(|g| g.kw == kw)
        .map(|g| {
            // ⛔ a table names its template; one that is not a timing (`lu_table_template`)
            // template, nor the built-in `scalar`, is dropped as the reference drops it
            // (`readTableModel`: warning 1253) — and an arc left with no table has no arc
            let name = g.args.trim().trim_matches('"');
            match ctx.templates.get(name) {
                Some(t) => orient(parse_table(&body[g.body.clone()]), Some(t), kind),
                None if name == "scalar" => orient(parse_table(&body[g.body.clone()]), None, kind),
                None => Table::default(),
            }
        })
        .unwrap_or_default()
}

/// What a table's two axes mean to its consumer — the order [`Table::lookup`] takes them in.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Axes {
    /// (input slew, output load) — delay, transition, sigma, receiver capacitance
    SlewLoad,
    /// (related pin slew, constrained pin slew) — setup, hold, recovery, removal
    RelatedConstrained,
}

/// A `*_template` group: its axis variables and default indices.
#[derive(Clone, Debug, Default)]
struct Template {
    vars: [String; 2],
    index: [Vec<f64>; 2],
}

/// The library's table templates, by name.
type Templates = BTreeMap<String, Template>;

/// Everything a cell's reading needs from its library.
struct Ctx<'a> {
    cap_f: f64,
    /// CCS current in the library's (capacitance unit / time unit), per its `current_unit`: see
    /// [`Units::ccs_current`].
    ccs_current: f64,
    skip_ccs: bool,
    templates: &'a Templates,
    /// `default_input_pin_cap`, `default_output_pin_cap`, `default_inout_pin_cap`
    default_cap: [f64; 3],
}

/// The library's TIMING table templates: its `lu_table_template` groups — the only kind a delay,
/// transition, constraint or receiver table is looked up in (`TableTemplateType::delay`); power,
/// current and OCV templates are other kinds. A name defined twice is one template whose axes the
/// later definition overwrites, each axis it states (`makeTableTemplate` finds the first, then
/// `setAxisN`).
fn templates(lib_body: &str, spans: &[GroupSpan]) -> Templates {
    let mut out = Templates::new();
    for g in spans.iter().filter(|g| g.kw == "lu_table_template") {
        let body = &lib_body[g.body.clone()];
        let own = own_attrs(body);
        let t = out.entry(g.args.trim().trim_matches('"').to_string()).or_default();
        for (n, (vk, ik)) in [("variable_1", "index_1"), ("variable_2", "index_2")].into_iter().enumerate() {
            if let Some(v) = simple_attr(&own, vk) {
                t.vars[n] = v;
                t.index[n] = next_paren_after(&own, ik).map(|v| floats(&v.replace('"', ""))).unwrap_or_default();
            }
        }
    }
    out
}

/// Which of `kind`'s two axes a template variable is: `Some(0)` first, `Some(1)` second.
fn axis_role(var: &str, kind: Axes) -> Option<usize> {
    match (kind, var) {
        (Axes::SlewLoad, "input_net_transition" | "input_transition_time") => Some(0),
        (Axes::SlewLoad, "total_output_net_capacitance") => Some(1),
        (Axes::RelatedConstrained, "related_pin_transition") => Some(0),
        (Axes::RelatedConstrained, "constrained_pin_transition") => Some(1),
        _ => None,
    }
}

/// A table as written, put in `kind`'s axis order using its template — the reference's rule.
///
/// ⛔ **The axes are what the TEMPLATE says, not where they sit.** The reference maps each axis by
/// its template's `variable_N` (`input_net_transition`, `constrained_pin_transition`, …). This
/// reader took `index_1` as the clock slew of every constraint: right for sky130 hd, whose
/// templates put the related pin first, and wrong for asap7, gf180, IHP and sky130 hs, which put
/// the constrained pin first — a setup check read at swapped slews, 0.28 ps where the reference
/// gives 21.9. Delay templates that put the load first (memory macros) had the same fault.
///
/// Also the reference's: an index the table does not state is the template's, and a table with
/// ONE axis is one-dimensional — given here a single-point second axis, so the lookup
/// interpolates along the axis it has instead of returning its first value.
fn orient(mut t: Table, tpl: Option<&Template>, kind: Axes) -> Table {
    let Some(tpl) = tpl else { return t };
    if t.index_1.is_empty() {
        t.index_1 = tpl.index[0].clone();
    }
    if t.index_2.is_empty() {
        t.index_2 = tpl.index[1].clone();
    }
    // The reference's shape: rows x columns from the axes (two axes: |1| x |2|; one: 1 x |1|),
    // each short row padded with 0 and missing rows added as 0 (warnings 1259 / 1261). Entries
    // past the axes are never read by a lookup; they are dropped here.
    let (rows, cols) = match (t.index_1.len(), t.index_2.len()) {
        (0, _) => (0, 0),
        (n, 0) => (1, n),
        (n, m) => (n, m),
    };
    if rows > 0 && !t.values.is_empty() {
        t.values.truncate(rows);
        t.values.resize(rows, Vec::new());
        for r in &mut t.values {
            r.resize(cols, 0.0);
        }
    }
    let (r1, r2) = (axis_role(&tpl.vars[0], kind), axis_role(&tpl.vars[1], kind));
    // one axis means one VARIABLE; a template that declares none says nothing about its axes
    let one_d = !tpl.vars[0].is_empty() && tpl.vars[1].is_empty();
    if one_d {
        // values is a single row along axis 1
        let row = t.values.first().cloned().unwrap_or_default();
        let axis = std::mem::take(&mut t.index_1);
        if row.len() != axis.len() {
            return Table { index_1: axis, index_2: Vec::new(), values: t.values };
        }
        return match r1 {
            Some(1) => Table { index_1: vec![0.0], index_2: axis, values: vec![row] },
            _ => Table { index_1: axis, index_2: vec![0.0], values: row.into_iter().map(|v| vec![v]).collect() },
        };
    }
    if r1 == Some(1) && r2 == Some(0) {
        let (n, m) = (t.index_1.len(), t.index_2.len());
        if t.values.len() == n && t.values.iter().all(|r| r.len() == m) {
            let values = (0..m).map(|j| (0..n).map(|i| t.values[i][j]).collect()).collect();
            return Table { index_1: std::mem::take(&mut t.index_2), index_2: std::mem::take(&mut t.index_1), values };
        }
    }
    t
}

struct GroupSpan {
    kw: String,
    args: String,
    /// The whole group, keyword through closing brace.
    whole: std::ops::Range<usize>,
    /// Inside the braces.
    body: std::ops::Range<usize>,
}

/// Where a group head's argument list opens: the `(` matching the LAST `)` before the brace.
///
/// ⛔ Not the first `(`. A statement written without its `;` runs into the group after it, and
/// its value may hold parentheses of its own — `when : "((A | B) & C)"` then
/// `rise_constraint ("t") {` (an IHP SRAM writes exactly this). Taking the first lost the
/// table's keyword, and the table with it.
fn args_open(head: &str) -> Option<usize> {
    let b = head.as_bytes();
    let close = head.trim_end().len().checked_sub(1).filter(|&k| b[k] == b')')?;
    let mut depth = 0i32;
    for k in (0..=close).rev() {
        match b[k] {
            b')' => depth += 1,
            b'(' => {
                depth -= 1;
                if depth == 0 {
                    return Some(k);
                }
            }
            _ => {}
        }
    }
    None
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
                if let Some(open) = args_open(raw) {
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
/// A pin's [`PinCaps`] from its own attributes, by the reference's rule.
fn pin_caps(own: &str, dir: Dir, default_cap: [f64; 3]) -> PinCaps {
    let num = |k: &str| simple_attr(own, k).and_then(|v| lnum(&v));
    let range = |k: &str| {
        next_paren_after(own, k)
            .map(|v| floats(&v.replace('"', "")))
            .filter(|v| v.len() == 2)
            .map(|v| (v[0], v[1]))
    };
    let base = num("capacitance");
    let edge = |rf: &str| {
        let (mut lo, mut hi) = (base, base);
        if let Some(c) = num(&format!("{rf}_capacitance")) {
            (lo, hi) = (Some(c), Some(c));
        }
        if let Some((a, b)) = range(&format!("{rf}_capacitance_range")) {
            (lo, hi) = (Some(a), Some(b));
        }
        (lo, hi)
    };
    let dflt = match dir {
        Dir::In => default_cap[0],
        Dir::Out => default_cap[1],
        Dir::Inout => default_cap[2],
        Dir::Other => 0.0,
    };
    let ((rl, rh), (fl, fh)) = (edge("rise"), edge("fall"));
    PinCaps {
        rise_min: rl.unwrap_or(dflt),
        rise_max: rh.unwrap_or(dflt),
        fall_min: fl.unwrap_or(dflt),
        fall_max: fh.unwrap_or(dflt),
    }
}

/// One arc per (from bit, to bit), as the reference's reader builds them: `related_pin` and
/// `related_bus_pins` are lists of names,
/// split on spaces and tabs, each name a pin, a bus or a bus range. Per timing group and to-pin,
/// the `related_pin` names in order, then the `related_bus_pins` names; per name its bits:
///   * to a plain pin (or a bus bit's own timing group): one arc from each bit (one/bus -> one);
///   * to a bus, from the BUS-level group: from one pin, an arc to every bit; from a bus,
///     `related_pin` pairs the bits one to one — when the sizes differ the LEADING bits of the
///     larger side are dropped — and `related_bus_pins` takes the cross product.
///
/// A name the cell does not declare makes no arc (the reference warns, port not found), and
/// neither does a delay group with no related pin at all (warning 1243).
fn expand_arcs(pins: &mut BTreeMap<String, Pin>, buses: &[(String, Vec<String>)], bus_bits: &BTreeMap<String, (usize, usize, usize)>) {
    let names: std::collections::BTreeSet<String> = pins.keys().cloned().collect();
    let bits_of = |name: &str| -> Vec<String> {
        if let Some((_, m)) = buses.iter().find(|(b, _)| b == name) {
            return m.clone();
        }
        if let Some((b, m)) = buses.iter().find(|(b, _)| name.starts_with(b.as_str()) && name[b.len()..].starts_with('[')) {
            return named_bits(name, b, m).into_iter().filter(|x| names.contains(x)).collect();
        }
        if names.contains(name) {
            vec![name.to_string()]
        } else {
            Vec::new()
        }
    };
    for (pname, pin) in pins.iter_mut() {
        let place = bus_bits.get(pname);
        let mut out = Vec::with_capacity(pin.arcs.len());
        for (j, arc) in pin.arcs.iter().enumerate() {
            // to a bus only for the arcs its BUS-level groups gave this bit
            let to_bus = place.filter(|&&(_, _, k)| j < k).map(|&(i, t, _)| (i, t));
            let lists = [(arc.related_pin.as_str(), true), (arc.related_bus_pins.as_str(), false)];
            for (list, one_to_one) in lists {
                for name in list.split([' ', '\t']).filter(|n| !n.is_empty()) {
                    let from = bits_of(name);
                    let chosen: Vec<&String> = match to_bus {
                        Some((i, t)) if from.len() > 1 && one_to_one => {
                            let f = from.len();
                            let (skip_from, skip_to) = (f.saturating_sub(t), t.saturating_sub(f));
                            if i < skip_to { Vec::new() } else { from.get(skip_from + i - skip_to).into_iter().collect() }
                        }
                        _ => from.iter().collect(),
                    };
                    for b in chosen {
                        let mut a = arc.clone();
                        a.related_pin = b.clone();
                        a.related_bus_pins = String::new();
                        out.push(a);
                    }
                }
            }
        }
        pin.arcs = out;
    }
}

/// A register or latch timing group that states no `timing_type` (or says `combinational`), on an
/// output whose function is one sequential's output (`IQ`): as the reference builds it, from a
/// pin in the CLOCK expression (`clocked_on`, a latch's `enable`) of positive or negative sense
/// it is a `rising_edge` arc — negative too, as the reference has it; from a pin in `clear` or
/// `preset` it is that async effect arc, which is not a delay arc here. From a pin in a latch's
/// `data_in` it is the latch D->Q arc (see [`Arc::feeds`]). Otherwise it stays combinational.
fn sequential_arcs(pins: &mut BTreeMap<String, Pin>, seqs: &[(String, String, String)]) {
    use crate::liberty_func::{Func, Sense};
    if seqs.is_empty() {
        return;
    }
    let names: std::collections::BTreeSet<String> = pins.keys().cloned().collect();
    let expr = |body: &str, key: &str| simple_attr(body, key).and_then(|e| Func::parse_in(&e, |p| names.contains(p)));
    for pin in pins.values_mut() {
        let Some(Func::Port(out)) = pin.function.as_deref().and_then(|f| Func::parse_in(f, |_| true)) else { continue };
        let Some((kw, _, body)) = seqs.iter().find(|(_, args, _)| args.split(',').any(|a| a.trim().trim_matches('"') == out)) else {
            continue;
        };
        let clock = expr(body, if kw == "ff" { "clocked_on" } else { "enable" });
        let (clear, preset) = (expr(body, "clear"), expr(body, "preset"));
        let data = if kw == "latch" { expr(body, "data_in") } else { None };
        let mut set_clr = Vec::new();
        pin.arcs.retain_mut(|a| {
            if a.clk_edge.is_some() || !matches!(a.kind.as_str(), "" | "combinational") {
                return true;
            }
            let from = a.related_pin.as_str();
            match &clock {
                Some(c) if c.has_port(from) => {
                    if matches!(c.port_timing_sense(from), Sense::Positive | Sense::Negative) {
                        a.clk_edge = Some(ClkEdge::Rise);
                        a.kind = "rising_edge".into();
                        if a.sense.is_empty() {
                            a.sense = "non_unate".into();
                        }
                    }
                    true
                }
                _ if clear.as_ref().is_some_and(|e| e.has_port(from)) || preset.as_ref().is_some_and(|e| e.has_port(from)) => {
                    // a set/clear arc, built only with the table of the edge it drives
                    let t = if clear.as_ref().is_some_and(|e| e.has_port(from)) { &a.cell_fall } else { &a.cell_rise };
                    if !t.values.is_empty() {
                        set_clr.push(from.to_string());
                    }
                    false
                }
                _ => {
                    // a latch's D->Q (`makeLatchDtoQArcs`): its edges follow the sense of `data_in`
                    // at `from`, whatever the group or the output (an inverted IQN among them) says
                    if let Some(d) = data.as_ref().filter(|d| d.has_port(from)) {
                        a.kind = "latch_d_to_q".into();
                        a.sense = match d.port_timing_sense(from) {
                            Sense::Positive => "positive_unate",
                            Sense::Negative => "negative_unate",
                            _ => "non_unate",
                        }
                        .into();
                    }
                    true
                }
            }
        });
        pin.set_clr_from.extend(set_clr);
    }
}

/// `LibertyCell::translatePresetClrCheckRoles`: a setup or hold check ON a pin that some set/clear
/// arc is timed from is a recovery or removal check — libraries write the async pin's checks as
/// `setup_rising` / `hold_rising` (icsprout55's DFFRQN on RN, among others).
fn preset_clear_check_roles(pins: &mut BTreeMap<String, Pin>) {
    let from: std::collections::BTreeSet<String> = pins.values().flat_map(|p| p.set_clr_from.iter().cloned()).collect();
    for name in from {
        if let Some(p) = pins.get_mut(&name) {
            let setup = std::mem::take(&mut p.setup);
            let hold = std::mem::take(&mut p.hold);
            p.recovery.extend(setup);
            p.removal.extend(hold);
        }
    }
}

/// The role the reference gives a `rising_edge` / `falling_edge` arc from `from` to an output with
/// `function` (`LibertyBuilder::makeRegLatchArcs`), over the cell's `ff` / `latch` groups `seqs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum EdgeRole {
    /// `regClkToQ` — and the fallback when no `ff` / `latch` claims the arc.
    RegClkToQ,
    LatchEnToQ,
    LatchDtoQ,
    RegSetClr,
}

/// `makeRegLatchArcs`: for each port the output's `function` names, the state group whose output
/// (`IQ` or `IQN`) it is decides — `from` in its clock (`clocked_on`, a latch's `enable`) is a
/// register clock-to-Q (a latch enable-to-Q); in a latch's `data_in`, latch D-to-Q; in `clear` or
/// `preset`, the set/clear arc; else the next port. When none decides — no function, no state
/// group, as on a hard macro whose launch pin has edge arcs and no `ff` — the arc is ASSUMED a register
/// clock-to-Q (`setHasInferedRegTimingArcs`).
///
/// ⚠️ The reference walks the ports in POINTER order (`LibertyPortSet` = `std::set<LibertyPort*>`);
/// this walks them left to right. They differ only when a function names two state outputs that
/// disagree about `from`, which no cell library we have does.
pub(crate) fn edge_arc_role(from: &str, function: Option<&str>, seqs: &[(String, String, String)], names: &std::collections::BTreeSet<String>) -> EdgeRole {
    use crate::liberty_func::Func;
    let expr = |body: &str, key: &str| simple_attr(body, key).and_then(|e| Func::parse_in(&e, |p| names.contains(p)));
    let Some(func) = function.and_then(|f| Func::parse_in(f, |_| true)) else { return EdgeRole::RegClkToQ };
    for port in func.ports() {
        let Some((kw, _, body)) = seqs.iter().find(|(_, args, _)| args.split(',').take(2).any(|a| a.trim().trim_matches('"') == port)) else {
            continue;
        };
        let latch = kw == "latch";
        if expr(body, if latch { "enable" } else { "clocked_on" }).is_some_and(|c| c.has_port(from)) {
            return if latch { EdgeRole::LatchEnToQ } else { EdgeRole::RegClkToQ };
        }
        if latch && expr(body, "data_in").is_some_and(|d| d.has_port(from)) {
            return EdgeRole::LatchDtoQ;
        }
        if ["clear", "preset"].iter().any(|k| expr(body, k).is_some_and(|e| e.has_port(from))) {
            return EdgeRole::RegSetClr;
        }
    }
    EdgeRole::RegClkToQ
}

/// `LibertyCell::makeTimingArcPortMaps`, the facts it sets on each port: a register clock-to-Q or
/// latch enable-to-Q arc marks its from-pin [`Pin::is_reg_clk`] and its to-pin
/// [`Pin::is_reg_output`]; every timing-check arc marks its from-pin [`Pin::is_check_clk`]. A
/// related name that is a bus marks each of its bits.
fn clock_roles(pins: &mut BTreeMap<String, Pin>, seqs: &[(String, String, String)], buses: &[(String, Vec<String>)]) {
    let names: std::collections::BTreeSet<String> = pins.keys().cloned().collect();
    let mut reg_clk = Vec::new();
    let mut reg_out = Vec::new();
    let mut check_clk = Vec::new();
    for (pname, pin) in pins.iter() {
        for a in pin.arcs.iter().filter(|a| a.clk_edge.is_some()) {
            if matches!(edge_arc_role(&a.related_pin, pin.function.as_deref(), seqs, &names), EdgeRole::RegClkToQ | EdgeRole::LatchEnToQ) {
                reg_clk.push(a.related_pin.clone());
                reg_out.push(pname.clone());
            }
        }
        for n in &pin.check_from {
            if n.is_empty() {
                check_clk.push(pname.clone());
            } else if let Some((_, bits)) = buses.iter().find(|(b, _)| b == n) {
                check_clk.extend(bits.iter().cloned());
            } else {
                check_clk.push(n.clone());
            }
        }
    }
    for (list, set) in [(reg_clk, 0), (reg_out, 1), (check_clk, 2)] {
        for n in list {
            if let Some(p) = pins.get_mut(&n) {
                match set {
                    0 => p.is_reg_clk = true,
                    1 => p.is_reg_output = true,
                    _ => p.is_check_clk = true,
                }
            }
        }
    }
}

/// The sense of every arc whose group states no `timing_sense`, as the reference builds the arcs:
///   * a combinational arc: the output pin's `function`, when it names the related pin; else, on a
///     three-state pin whose enable names it, non-unate;
///   * a three-state enable/disable arc: the enable (the inverted `three_state` expression);
///   * anything else, or no answer (a function naming a port the cell lacks parses to nothing):
///     non-unate, which times both input edges, as the reference does for an unknown sense.
fn derive_senses(pins: &mut BTreeMap<String, Pin>) {
    use crate::liberty_func::{Func, Sense};
    let names: std::collections::BTreeSet<String> = pins.keys().cloned().collect();
    let is_port = |p: &str| names.contains(p);
    for pin in pins.values_mut() {
        if pin.arcs.iter().all(|a| !a.sense.is_empty()) {
            continue;
        }
        let func = pin.function.as_deref().and_then(|f| Func::parse_in(f, is_port));
        let enable = pin.three_state.as_deref().and_then(|f| Func::parse_in(f, is_port)).map(Func::invert);
        for a in pin.arcs.iter_mut().filter(|a| a.sense.is_empty()) {
            let from = a.related_pin.as_str();
            let combinational = a.clk_edge.is_none() && matches!(a.kind.as_str(), "" | "combinational" | "combinational_rise" | "combinational_fall");
            let sense = if combinational {
                match (&func, &enable) {
                    (Some(f), _) if f.has_port(from) => f.port_timing_sense(from),
                    (_, Some(e)) if e.has_port(from) => Sense::NonUnate,
                    _ => Sense::Unknown,
                }
            } else if a.kind.starts_with("three_state_enable") {
                enable.as_ref().map_or(Sense::Unknown, |e| e.port_timing_sense(from))
            } else if a.kind.starts_with("three_state_disable") {
                // the OPPOSITE of the enable's sense (`makeTristateDisableArcs`): the edge that disables
                match enable.as_ref().map_or(Sense::Unknown, |e| e.port_timing_sense(from)) {
                    Sense::Positive => Sense::Negative,
                    Sense::Negative => Sense::Positive,
                    s => s,
                }
            } else {
                Sense::Unknown
            };
            a.sense = match sense {
                Sense::Positive => "positive_unate",
                Sense::Negative => "negative_unate",
                _ => "non_unate",
            }
            .to_string();
        }
    }
}

fn merged_pin(name: &str, group: &Pin, inner: Option<(&Pin, &str)>) -> Pin {
    let mut p = group.clone();
    p.name = name.to_string();
    if let Some((ip, ibody)) = inner {
        if ip.direction != Dir::Other {
            p.direction = ip.direction;
        }
        // a pin group that states any capacitance of its own states all of it
        let own = own_attrs(ibody);
        if ["capacitance", "rise_capacitance", "fall_capacitance", "rise_capacitance_range", "fall_capacitance_range"]
            .iter()
            .any(|k| simple_attr(&own, k).is_some() || next_paren_after(&own, k).is_some())
        {
            p.capacitance = ip.capacitance;
            p.caps = ip.caps;
            p.cap_f = ip.cap_f;
        }
        if ip.recv.is_some() {
            p.recv = ip.recv.clone();
        }
        p.clock |= ip.clock;
        if ip.function.is_some() {
            p.function = ip.function.clone();
        }
        if ip.three_state.is_some() {
            p.three_state = ip.three_state.clone();
        }
        p.setup.extend(ip.setup.iter().cloned());
        p.hold.extend(ip.hold.iter().cloned());
        p.recovery.extend(ip.recovery.iter().cloned());
        p.removal.extend(ip.removal.iter().cloned());
        p.arcs.extend(ip.arcs.iter().cloned());
        p.check_from.extend(ip.check_from.iter().cloned());
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
    templates: &Templates,
    default_cap: [f64; 3],
) -> Cell {
    let ctx = Ctx { cap_f: units.cap_f, ccs_current: units.ccs_current(), skip_ccs, templates, default_cap };
    let mut pins = BTreeMap::new();
    let mut scan_pins = BTreeMap::new();
    // Each bus or bundle's members in order, and for each member: its place in that bus and how
    // many of its leading arcs came from the BUS-level timing groups — what `expand_arcs` needs.
    let mut buses: Vec<(String, Vec<String>)> = Vec::new();
    let mut bus_bits: BTreeMap<String, (usize, usize, usize)> = BTreeMap::new();
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
                    pins.insert(pname.to_string(), parse_pin(pname.to_string(), gbody, &ctx));
                }
            }
            // ⛔ A bus's direction and capacitance are on the BUS, and a bus need not repeat them
            // in per-bit `pin` groups — or have any. Unexpanded, an SRAM's D/Q/DM bits had no
            // direction at all. A bus is its bits `name[i]`, bit_from to
            // bit_to; the bus's attributes reach every bit; a pin group inside states its own.
            "bus" | "bundle" => {
                let own = parse_pin(args.clone(), &without_groups(gbody, "pin"), &ctx);
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
                for (i, m) in members.iter().enumerate() {
                    pins.insert(m.clone(), merged_pin(m, &own, None));
                    bus_bits.insert(m.clone(), (i, members.len(), own.arcs.len()));
                }
                buses.push((args.clone(), members.clone()));
                for (ikw, iargs, ibody) in top_groups(gbody) {
                    if ikw != "pin" {
                        continue;
                    }
                    let ip = parse_pin(iargs.clone(), ibody, &ctx);
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
    expand_arcs(&mut pins, &buses, &bus_bits);
    let seqs: Vec<(String, String, String)> = groups
        .iter()
        .filter(|(k, _, _)| k == "ff" || k == "latch")
        .map(|(k, a, b)| (k.clone(), a.clone(), b.to_string()))
        .collect();
    sequential_arcs(&mut pins, &seqs);
    preset_clear_check_roles(&mut pins);
    derive_senses(&mut pins);
    clock_roles(&mut pins, &seqs, &buses);
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
    let dont_use = bool_attr(&own, "dont_use");
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
        dont_use,
        thresholds: None,
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

/// `include_file (name) ;` replaced by that file's text, as the reference's lexer reads it: the
/// name as written (opened from the working directory, not the library's), one level only — an
/// include inside an included file is not followed — and a file that cannot be opened leaves
/// nothing (the reference warns and reads on). ⛔ The statement was skipped: every cell, template
/// or default it brought in was missing, and nothing said so.
fn splice_includes(text: String) -> String {
    const KW: &str = "include_file";
    if !text.contains(KW) {
        return text;
    }
    let mut out = String::with_capacity(text.len());
    let mut rest = text.as_str();
    while let Some(at) = rest.find(KW) {
        let before_ok = at == 0 || !is_ident(rest.as_bytes()[at - 1]);
        let tail = &rest[at + KW.len()..];
        let open = tail.len() - tail.trim_start().len();
        let parsed = tail[open..].strip_prefix('(').and_then(|t| t.find(')').map(|c| (t[..c].trim().trim_matches('"').to_string(), open + 1 + c + 1)));
        match (before_ok, parsed) {
            (true, Some((name, used))) => {
                out.push_str(&rest[..at]);
                let mut after = &tail[used..];
                let semi = after.len() - after.trim_start().len();
                if after[semi..].starts_with(';') {
                    after = &after[semi + 1..];
                }
                if let Ok(inc) = std::fs::read_to_string(&name) {
                    out.push_str(&inc);
                }
                rest = after;
            }
            _ => {
                out.push_str(&rest[..at + KW.len()]);
                rest = tail;
            }
        }
    }
    out.push_str(rest);
    out
}

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
        let edits = lex_edits(text.as_bytes());
        if edits.is_empty() {
            return Lib::parse_lexed(text, opts);
        }
        let mut b = text.as_bytes().to_vec();
        blank(&mut b, edits);
        // only ASCII bytes were replaced, by ASCII, so the text is still UTF-8
        let cleaned = String::from_utf8(b).map_err(|e| LibError(e.to_string()))?;
        Lib::parse_lexed(&cleaned, opts)
    }

    /// [`Lib::parse_opts`] on text [`lex_clean`] has already seen.
    fn parse_lexed(text: &str, opts: LibOpts) -> Result<Lib, LibError> {
        // The library group, read once at its own level: its attributes, its templates, its
        // cells. A file with no `library` group is read as if its body were one.
        let lib_body = top_group_spans(text)
            .into_iter()
            .find(|g| g.kw == "library")
            .map(|g| &text[g.body])
            .unwrap_or(text);
        let (lib_own, lib_spans) = level(lib_body);
        let units = Units::from_lib(&lib_own);
        let voltage = lib_voltage(&lib_own, text).unwrap_or(1.8);
        let thresholds = Thresholds::from_lib(&lib_own);
        let types = bus_types(text);
        let tpls = templates(lib_body, &lib_spans);
        let dcap = |k: &str| simple_attr(&lib_own, k).and_then(|v| lnum(&v)).unwrap_or(0.0);
        let default_cap = [dcap("default_input_pin_cap"), dcap("default_output_pin_cap"), dcap("default_inout_pin_cap")];
        let mut cells = BTreeMap::new();
        for g in lib_spans.iter().filter(|g| g.kw == "cell") {
            let cname = g.args.clone();
            cells.insert(
                cname.clone(),
                parse_cell(cname, &lib_body[g.body.clone()], &units, opts.skip_ccs, &types, &tpls, default_cap),
            );
        }
        if cells.is_empty() {
            return Err(LibError("no cells found".into()));
        }
        Ok(Lib {
            cells,
            units: units.timing,
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
        // Inflated when gzipped and read lossily, as the netlist reader does: the reference reads
        // a `.lib.gz` directly (its own test libraries, and asap7's, ship that way), and a
        // comment in a legacy code page must not make a whole library unreadable.
        let text = crate::netlist::read_text(path).map_err(|e| LibError(e.0))?;
        let text = splice_includes(text);
        // keyed on the file as read (its includes spliced in): a cache hit costs no lexing at all
        let key = lib_cache_key(&text, opts);
        // 1) in-process cache
        if let Some(hit) = lib_cache().lock().ok().and_then(|m| m.get(&key).cloned()) {
            return Ok((*hit).clone());
        }
        // 2) cross-process on-disk cache (env-gated; no-op unless enabled)
        if let Some(lib) = crate::libcache::disk_get(key) {
            return Ok(cache_store(key, lib));
        }
        // 3) parse, then populate both the disk and in-process caches. Cleaned IN PLACE: a large
        // CCS library is not copied a second time.
        let mut b = text.into_bytes();
        lex_clean(&mut b);
        // only ASCII bytes were replaced, by ASCII, so the text is still UTF-8
        let text = String::from_utf8(b).map_err(|e| LibError(format!("{path}: {e}")))?;
        let lib = Lib::parse_lexed(&text, opts)?;
        crate::libcache::disk_put(key, &lib);
        Ok(cache_store(key, lib))
    }

    pub fn cell(&self, name: &str) -> Option<&Cell> {
        self.cells.get(name)
    }

    /// Merge another lib's cells into this one (multi-lib jobs), in this one's units. Existing
    /// cells win: the reference links a cell from the first library holding it.
    pub fn merge(&mut self, mut other: Lib) {
        if self.voltage == 0.0 {
            self.voltage = other.voltage;
        }
        other.rescale_to(self.units);
        let own = (other.thresholds != self.thresholds).then_some(other.thresholds);
        for (k, mut v) in other.cells {
            if v.thresholds.is_none() {
                v.thresholds = own;
            }
            self.cells.entry(k).or_insert(v);
        }
    }

    /// The thresholds a cell is timed with: its own library's ([`Cell::thresholds`]), else this
    /// library's. An unknown cell, like a port, takes this (the first-read) library's.
    pub fn cell_thresholds(&self, cell: &str) -> Thresholds {
        self.cells.get(cell).and_then(|c| c.thresholds).unwrap_or(self.thresholds)
    }

    /// Put every time and capacitance in `to`'s units: table axes (a delay's slew and load, a
    /// constraint's two slews) and values, pin capacitances, receiver models, sigma tables and CCS
    /// waveforms (whose currents are in capacitance per time, so they scale with both).
    pub fn rescale_to(&mut self, to: LibUnits) {
        let (t, c) = (self.units.time_s / to.time_s, self.units.cap_f / to.cap_f);
        self.units = to;
        let one = |f: f64| (f - 1.0).abs() < 1e-12;
        if one(t) && one(c) {
            return;
        }
        let table = |tb: &mut Table, i1: f64, i2: f64, v: f64| {
            tb.index_1.iter_mut().for_each(|x| *x *= i1);
            tb.index_2.iter_mut().for_each(|x| *x *= i2);
            tb.values.iter_mut().flatten().for_each(|x| *x *= v);
        };
        for cell in self.cells.values_mut() {
            for pin in cell.pins.values_mut() {
                pin.capacitance *= c;
                for x in [&mut pin.caps.rise_min, &mut pin.caps.rise_max, &mut pin.caps.fall_min, &mut pin.caps.fall_max] {
                    *x *= c;
                }
                if let Some(r) = pin.recv.as_mut() {
                    for tb in [&mut r.c1_rise, &mut r.c2_rise, &mut r.c1_fall, &mut r.c2_fall] {
                        table(tb, t, c, c);
                    }
                }
                for k in pin.setup.iter_mut().chain(pin.hold.iter_mut()).chain(pin.recovery.iter_mut()).chain(pin.removal.iter_mut()) {
                    table(&mut k.rise, t, t, t);
                    table(&mut k.fall, t, t, t);
                }
                for a in &mut pin.arcs {
                    for tb in [&mut a.cell_rise, &mut a.cell_fall, &mut a.rise_transition, &mut a.fall_transition, &mut a.sigma_rise, &mut a.sigma_fall] {
                        table(tb, t, c, t);
                    }
                    for w in a.ccs.rise.iter_mut().chain(a.ccs.fall.iter_mut()) {
                        w.in_slew *= t;
                        w.out_cap *= c;
                        w.ref_time *= t;
                        w.time.iter_mut().for_each(|x| *x *= t);
                        w.current.iter_mut().for_each(|x| *x *= c / t);
                    }
                }
            }
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
                s.push_str(&format!("\"is_reg_clk\":{},\"is_reg_output\":{},\"is_check_clk\":{},", pin.is_reg_clk, pin.is_reg_output, pin.is_check_clk));
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
    /// The timing units, the reference's way: see [`LibUnits`].
    timing: LibUnits,
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
            timing: LibUnits::from_lib(text),
        }
    }

    /// The factor that puts a CCS current in the library's capacitance unit per time unit, so a
    /// waveform's current × time / capacitance is in volts whatever its units (1 for ns, pF, mA).
    fn ccs_current(&self) -> f64 {
        let u = &self.timing;
        let f = u.current_a * u.time_s / u.cap_f;
        if (f - 1.0).abs() < 1e-9 {
            1.0
        } else {
            f
        }
    }
}

/// A library's timing units as SI scales: `time_unit`, `capacitive_load_unit` and `current_unit`,
/// each defaulting as the reference's reader does (1 ns, 1 pF, 1 mA), and a capacitance unit
/// other than `ff` or `pf` left at the default as it leaves it.
///
/// The reference scales every library's values to SI as it reads them and reports in the FIRST
/// library's units (`Sta::readLiberty`: "Set units from default (first) library"); SDC values are
/// read in those units too. Values here stay in their library's units, so libraries are put in
/// the first one's units when merged ([`Lib::merge`]) — the same numbers the reference reports.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LibUnits {
    pub time_s: f64,
    pub cap_f: f64,
    pub current_a: f64,
}

impl Default for LibUnits {
    fn default() -> Self {
        LibUnits { time_s: 1e-9, cap_f: 1e-12, current_a: 1e-3 }
    }
}

impl LibUnits {
    fn from_lib(text: &str) -> LibUnits {
        let d = LibUnits::default();
        let time_s = simple_attr(text, "time_unit").as_deref().map(parse_si_time).unwrap_or(d.time_s);
        let current_a = simple_attr(text, "current_unit").as_deref().map(parse_si_current).unwrap_or(d.current_a);
        let cap_f = match cap_load_unit(text) {
            Some(c) if cap_load_suffix(text).is_some_and(|s| s == "ff" || s == "pf") => c,
            _ => d.cap_f,
        };
        LibUnits { time_s, cap_f, current_a }
    }

    /// The unit's name as the reference prints it (`1ns`, `1ps`, `1pF`, `1fF`), or the scale in
    /// SI when it is none of those.
    pub fn time_name(&self) -> String {
        unit_name(self.time_s, &[(1e-9, "ns"), (1e-12, "ps"), (1e-6, "us"), (1e-15, "fs")], "s")
    }

    pub fn cap_name(&self) -> String {
        unit_name(self.cap_f, &[(1e-12, "pF"), (1e-15, "fF")], "F")
    }

    /// The time unit as a label after a value: `ns`, `ps`, or `10ps` for a unit of ten.
    pub fn time_label(&self) -> String {
        let n = self.time_name();
        match n.strip_prefix('1') {
            Some(rest) if rest.starts_with(|c: char| c.is_ascii_alphabetic()) => rest.to_string(),
            _ => n,
        }
    }

    /// Nanoseconds per time unit.
    pub fn ns_per_unit(&self) -> f64 {
        self.time_s / 1e-9
    }
}

fn unit_name(scale: f64, known: &[(f64, &str)], si: &str) -> String {
    for &(base, name) in known {
        let k = scale / base;
        if (1.0..1000.0).contains(&(k * (1.0 + 1e-9))) {
            let k = if (k - k.round()).abs() < 1e-9 { k.round() } else { k };
            return format!("{k}{name}");
        }
    }
    format!("{scale:e}{si}")
}

/// The suffix of `capacitive_load_unit (n, suffix)`, lower-cased.
fn cap_load_suffix(lib_body: &str) -> Option<String> {
    let p = lib_body.find("capacitive_load_unit")?;
    let open = lib_body[p..].find('(')? + p;
    let close = lib_body[open..].find(')')? + open;
    let suffix = lib_body[open + 1..close].split(',').nth(1)?;
    Some(suffix.trim().trim_matches('"').trim().to_lowercase())
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
    // the suffix may be quoted (`"ff"`): the reference's lexer reads it as the string either way
    let base = match parts.next().unwrap_or("pf").trim().trim_matches('"').trim().to_lowercase().as_str() {
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

/// A check the reader does not model is NAMED on its pin and never built as a delay arc: the
/// reference times `non_seq_setup_rising` (a gate's select against its clock) as a check, and a
/// design checked only by it reported nothing at all when the group was dropped silently.
#[cfg(test)]
mod unmodelled_check_tests {
    use super::*;

    #[test]
    fn a_non_sequential_check_is_named_not_timed() {
        let lib = Lib::parse(
            r#"library (u) { cell (CKMUX) {
  pin (CLK1) { direction : input; capacitance : 0.001; }
  pin (CLKSEL) { direction : input; capacitance : 0.001;
    timing () { related_pin : "CLK1"; timing_type : non_seq_setup_rising; rise_constraint (s) { values ("0.05"); } }
    timing () { related_pin : "CLK1"; timing_type : non_seq_hold_rising; rise_constraint (s) { values ("0.02"); } } }
  pin (CLKOUT) { direction : output; function : "CLK1"; } } }"#,
        )
        .unwrap();
        let p = &lib.cell("CKMUX").unwrap().pins["CLKSEL"];
        assert!(p.arcs.is_empty() && p.setup.is_empty() && p.hold.is_empty());
        assert_eq!(p.unmodelled_checks, ["non_seq_setup_rising", "non_seq_hold_rising"]);
        let r = Lib::parse(
            r#"library (r) { cell (B) { pin (A) { direction : input; capacitance : 0.001; }
  pin (Y) { direction : output; function : "A";
    timing () { related_pin : "A"; cell_rise (s) { values ("4.0"); } cell_fall (s) { values ("5.0"); }
      rise_transition (s) { values ("0.4"); } fall_transition (s) { values ("0.5"); }
      retaining_rise (s) { values ("1.0"); } retaining_fall (s) { values ("1.2"); } } } } }"#,
        )
        .unwrap();
        assert_eq!(r.cell("B").unwrap().pins["Y"].unmodelled_checks, ["retaining_rise", "retaining_fall"]);
        let back = crate::libcache::decode(&crate::libcache::encode(&lib)).unwrap();
        assert_eq!(back.cell("CKMUX").unwrap().pins["CLKSEL"].unmodelled_checks, p.unmodelled_checks);
    }
}

/// The reference reads a gzipped Liberty directly; so does `Lib::load` (with `gzip`).
#[cfg(all(test, feature = "gzip"))]
mod gzip_tests {
    use super::*;

    #[test]
    fn a_gzipped_library_loads_as_its_text() {
        let text = "library (z) { cell (INV) { pin (A) { direction : input; capacitance : 0.002; } pin (Y) { direction : output; function : \"!A\"; } } }\n";
        let mut g = vec![0x1f, 0x8b, 8, 0, 0, 0, 0, 0, 0, 3];
        g.extend(miniz_oxide::deflate::compress_to_vec(text.as_bytes(), 6));
        g.extend_from_slice(&[0; 4]);
        g.extend_from_slice(&(text.len() as u32).to_le_bytes());
        let f = std::env::temp_dir().join(format!("loom-lib-gz-{}.lib.gz", std::process::id()));
        std::fs::write(&f, g).unwrap();
        let lib = Lib::load(f.to_str().unwrap()).unwrap();
        let _ = std::fs::remove_file(&f);
        assert!((lib.cell("INV").unwrap().pins["A"].capacitance - 0.002).abs() < 1e-9);
    }
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
  lu_table_template (t) { variable_1 : input_net_transition; index_1 ("0.01, 0.1"); }
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
  lu_table_template(t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  lu_table_template(s) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition; }
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

    /// A group whose head follows a semicolon-less attribute holding parentheses is still that
    /// group (an IHP SRAM writes `when : "((A | B) & C)"` then `rise_constraint (t) {`).
    #[test]
    fn a_group_after_an_unterminated_attribute_with_parens_is_found() {
        let lib = LIB.replace(
            "timing () { related_pin : \"A\";",
            "timing () { related_pin : \"A\"; when : \"((A | A) & A)\"\n sdf_cond : \"x\"\n",
        );
        assert_ne!(lib, LIB, "the fixture must change");
        let l = Lib::parse(&lib).unwrap();
        assert_eq!(l.cell("INV").unwrap().pins["Y"].arcs[0].cell_rise.index_1.len(), 2);
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

/// A table's axes are what its TEMPLATE says — each rule here the reference's.
#[cfg(test)]
mod table_axis_tests {
    use super::*;

    fn lib(templates: &str, cell: &str) -> Lib {
        Lib::parse(&format!("library (l) {{\n capacitive_load_unit (1, pf);\n{templates}\n cell (C) {{\n{cell}\n }}\n}}\n")).unwrap()
    }

    const TPL: &str = "
      lu_table_template (rc) { variable_1 : related_pin_transition; variable_2 : constrained_pin_transition;
        index_1 (\"1, 2\"); index_2 (\"10, 20\"); }
      lu_table_template (cr) { variable_1 : constrained_pin_transition; variable_2 : related_pin_transition;
        index_1 (\"10, 20\"); index_2 (\"1, 2\"); }
      lu_table_template (sl) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance;
        index_1 (\"1, 2\"); index_2 (\"10, 20\"); }
      lu_table_template (ls) { variable_1 : total_output_net_capacitance; variable_2 : input_net_transition;
        index_1 (\"10, 20\"); index_2 (\"1, 2\"); }
      lu_table_template (load1) { variable_1 : total_output_net_capacitance; index_1 (\"10, 20\"); }";

    fn setup(tpl: &str, values: &str) -> Constraint {
        let l = lib(TPL, &format!(
            "pin (CK) {{ direction : input; clock : true; }}
             pin (D) {{ direction : input; timing () {{ related_pin : CK; timing_type : setup_rising;
               rise_constraint ({tpl}) {{ values ({values}); }} fall_constraint ({tpl}) {{ values ({values}); }} }} }}"
        ));
        l.cell("C").unwrap().pins["D"].setup[0].clone()
    }

    /// A check keeps the clock edge its `timing_type` names and the pin it is related to. The
    /// reference builds each check arc from that edge of `related_pin`; a dual-port SRAM checks
    /// `addr1` against `clk1`, not the cell's first clock pin.
    #[test]
    fn a_check_keeps_its_clock_edge_and_related_pin() {
        let l = lib(TPL, "pin (clk0) { direction : input; clock : true; }
             pin (clk1) { direction : input; clock : true; }
             pin (a0) { direction : input; timing () { related_pin : \"clk0\"; timing_type : setup_rising;
               rise_constraint (rc) { values (\"1, 2\", \"3, 4\"); } } }
             pin (a1) { direction : input; timing () { related_pin : \"clk1\"; timing_type : setup_falling;
               rise_constraint (rc) { values (\"1, 2\", \"3, 4\"); } }
               timing () { related_pin : \"clk1\"; timing_type : removal_falling;
               rise_constraint (rc) { values (\"1, 2\", \"3, 4\"); } } }");
        let c = l.cell("C").unwrap();
        assert_eq!((c.pins["a0"].setup[0].clk_edge, c.pins["a0"].setup[0].related_pin.as_str()), (ClkEdge::Rise, "clk0"));
        assert_eq!((c.pins["a1"].setup[0].clk_edge, c.pins["a1"].setup[0].related_pin.as_str()), (ClkEdge::Fall, "clk1"));
        assert_eq!(c.pins["a1"].removal[0].clk_edge, ClkEdge::Fall);
    }

    /// The same check written related-first and constrained-first reads the same — at
    /// (related slew, constrained slew), whichever order the file uses.
    #[test]
    fn a_constraint_is_read_by_its_template_variables() {
        // value = related*100 + constrained, so the axes cannot be confused
        let rc = setup("rc", "\"110, 120\", \"210, 220\"");
        let cr = setup("cr", "\"110, 210\", \"120, 220\"");
        for c in [&rc, &cr] {
            assert_eq!(c.eval_edge(true, 1.0, 20.0), 120.0);
            assert_eq!(c.eval_edge(true, 2.0, 10.0), 210.0);
        }
    }

    fn delay(tpl: &str, values: &str, inline: &str) -> Table {
        let l = lib(TPL, &format!(
            "pin (A) {{ direction : input; }}
             pin (Y) {{ direction : output; function : \"!A\"; timing () {{ related_pin : A;
               cell_rise ({tpl}) {{ {inline} values ({values}); }} }} }}"
        ));
        l.cell("C").unwrap().pins["Y"].arcs[0].cell_rise.clone()
    }

    /// A delay template that puts the load first is read at (slew, load) all the same, and a
    /// table that states no index takes the template's.
    #[test]
    fn a_load_first_delay_table_and_a_template_only_index() {
        let sl = delay("sl", "\"110, 120\", \"210, 220\"", "");
        let ls = delay("ls", "\"110, 210\", \"120, 220\"", "");
        for t in [&sl, &ls] {
            assert_eq!(t.index_1, [1.0, 2.0]);
            assert_eq!(t.lookup(1.0, 20.0), 120.0);
            assert_eq!(t.lookup(2.0, 15.0), 215.0, "interpolated, not a constant");
        }
    }

    /// A one-axis table interpolates along its axis.
    #[test]
    fn a_one_dimensional_table_is_not_a_constant() {
        let t = delay("load1", "\"1, 2\"", "");
        assert_eq!(t.lookup(0.5, 10.0), 1.0);
        assert_eq!(t.lookup(0.5, 15.0), 1.5);
        assert_eq!(t.lookup(9.0, 20.0), 2.0, "slew does not matter");
    }
}

/// Which timing groups are delay arcs, and which edges they drive — the reference's lists.
#[cfg(test)]
mod timing_type_tests {
    use super::*;

    fn arcs(types: &[&str]) -> Vec<Arc> {
        let groups: String = types
            .iter()
            .map(|t| format!("timing () {{ related_pin : A; timing_type : {t}; cell_rise (scalar) {{ values (\"1\"); }} }}\n"))
            .collect();
        let l = Lib::parse(&format!(
            "library (l) {{ cell (C) {{ pin (A) {{ direction : input; }} pin (Y) {{ direction : output; {groups} }} }} }}"
        ))
        .unwrap();
        l.cell("C").unwrap().pins["Y"].arcs.clone()
    }

    /// Only the reference's delay types are delay arcs; checks loom does not model, and types
    /// the reference drops, are not zero-delay paths.
    #[test]
    fn only_delay_types_make_delay_arcs() {
        let kept = ["combinational", "rising_edge", "falling_edge", "three_state_enable", "three_state_disable_rise"];
        let dropped = ["non_seq_setup_rising", "non_seq_hold_falling", "minimum_period", "min_pulse_width",
                       "skew_rising", "nochange_high_high", "retaining_time", "clear", "preset", "bogus_type"];
        assert_eq!(arcs(&kept).len(), kept.len());
        assert!(arcs(&dropped).is_empty());
    }

    /// `combinational_rise` drives only the rising output edge, `_fall` only the falling one.
    #[test]
    fn a_one_edge_type_drives_one_edge() {
        let a = arcs(&["combinational_rise", "combinational_fall", "combinational", "three_state_enable_fall"]);
        let e: Vec<OutEdges> = a.iter().map(|x| x.out_edges).collect();
        assert_eq!(e, [OutEdges::Rise, OutEdges::Fall, OutEdges::Both, OutEdges::Fall]);
        assert!(OutEdges::Rise.drives(true) && !OutEdges::Rise.drives(false));
    }
}

/// A pin's capacitance per edge — the reference's rule.
#[cfg(test)]
mod pin_cap_tests {
    use super::*;

    fn pin(lib_attrs: &str, pin_attrs: &str, dir: &str) -> Pin {
        Lib::parse(&format!(
            "library (l) {{ {lib_attrs} cell (C) {{ pin (P) {{ direction : {dir}; {pin_attrs} }} }} }}"
        ))
        .unwrap()
        .cell("C")
        .unwrap()
        .pins["P"]
        .clone()
    }

    #[test]
    fn rise_and_fall_override_capacitance_and_a_range_splits_min_from_max() {
        let p = pin("", "capacitance : 2; rise_capacitance : 3; fall_capacitance_range (1, 4);", "input");
        assert_eq!(p.caps, PinCaps { rise_min: 3.0, rise_max: 3.0, fall_min: 1.0, fall_max: 4.0 });
        assert_eq!(p.capacitance, 2.0, "the stated figure");
    }

    /// A pin that states no capacitance takes the library default for its direction, and its
    /// single figure is no longer zero.
    #[test]
    fn an_unstated_capacitance_is_the_library_default() {
        let lib = "default_input_pin_cap : 0.5; default_output_pin_cap : 0.25;";
        assert_eq!(pin(lib, "", "input").caps.get(true, true), 0.5);
        assert_eq!(pin(lib, "", "output").capacitance, 0.25);
        let only_rise = pin("", "rise_capacitance : 3; fall_capacitance : 2;", "input");
        assert_eq!(only_rise.capacitance, 3.0, "the largest, not zero");
    }
}

#[cfg(test)]
mod related_pin_lists {
    use super::*;

    fn lib() -> Lib {
        Lib::parse(
            r#"library(t) {
  time_unit : "1ns";
  capacitive_load_unit (1, pf);
  type (b4) { base_type : array; data_type : bit; bit_width : 4; bit_from : 3; bit_to : 0; }
  type (b2) { base_type : array; data_type : bit; bit_width : 2; bit_from : 1; bit_to : 0; }
  cell (AO) {
    pin(A) { direction : input; }
    pin(B) { direction : input; }
    pin(Y) { direction : output;
      timing () { related_pin : "A  B"; cell_rise (scalar) { values("0.1"); } }
      timing () { related_pin : "NOPE"; cell_rise (scalar) { values("0.3"); } }
      timing () { cell_rise (scalar) { values("0.4"); } }
    }
  }
  cell (MEM) {
    pin(CK) { direction : input; clock : true; }
    bus (D) { bus_type : b4; direction : input; }
    bus (S) { bus_type : b2; direction : input; }
    pin(P) { direction : output; timing () { related_pin : "D"; cell_rise (scalar) { values("0.2"); } } }
    bus (Q) { bus_type : b4; direction : output;
      timing () { related_pin : "CK"; timing_type : rising_edge; cell_rise (scalar) { values("0.5"); } }
      timing () { related_pin : "D"; cell_rise (scalar) { values("0.6"); } }
      timing () { related_pin : "S"; cell_rise (scalar) { values("0.7"); } }
      timing () { related_bus_pins : "S"; cell_rise (scalar) { values("0.8"); } }
    }
  }
}"#,
        )
        .unwrap()
    }

    fn from(p: &Pin) -> Vec<&str> {
        p.arcs.iter().map(|a| a.related_pin.as_str()).collect()
    }

    /// Reference rule: `related_pin` is a list split on spaces and tabs, one arc
    /// set per name in order; a group with no related pin makes none (warning 1243). Checked on
    /// this library: the reference's `report_lib_cell` lists exactly these arcs for AO and MEM.
    /// ⚠️ A name the cell lacks: the reference warns (1294, port not found) and then CRASHES (its
    /// empty iterator takes the bus-to-bus branch); here it makes no arc.
    #[test]
    fn a_related_pin_list_is_one_arc_per_pin() {
        let l = lib();
        assert_eq!(from(&l.cells["AO"].pins["Y"]), vec!["A", "B"]);
    }

    /// bus -> one: every bit of the bus, in bus order.
    #[test]
    fn a_bus_related_pin_to_one_pin_is_every_bit() {
        let l = lib();
        assert_eq!(from(&l.cells["MEM"].pins["P"]), vec!["D[3]", "D[2]", "D[1]", "D[0]"]);
    }

    /// one -> bus: the pin to each bit. bus -> bus by `related_pin`: one to one, the larger
    /// side's LEADING bits dropped when the sizes differ (S has 2 bits, Q 4: Q[3], Q[2] get none
    /// from S). bus -> bus by `related_bus_pins`: the cross product.
    #[test]
    fn bus_to_bus_pairs_bits_and_related_bus_pins_crosses_them() {
        let l = lib();
        let m = &l.cells["MEM"];
        assert_eq!(from(&m.pins["Q[3]"]), vec!["CK", "D[3]", "S[1]", "S[0]"]);
        assert_eq!(from(&m.pins["Q[1]"]), vec!["CK", "D[1]", "S[1]", "S[1]", "S[0]"]);
        assert_eq!(from(&m.pins["Q[0]"]), vec!["CK", "D[0]", "S[0]", "S[1]", "S[0]"]);
        assert!(m.pins["Q[0]"].arcs.iter().all(|a| a.related_bus_pins.is_empty()));
    }
}

#[cfg(test)]
mod derived_sense {
    use super::*;

    /// Reference rule: a combinational group with no `timing_sense` takes it from the output's
    /// function; a three-state pin's enable pin, absent from the function, is
    /// non-unate. Checked on this library: the reference times exactly these edges (`report_edges`:
    /// AND2 A ^->^ v->v; NOR2 A ^->v v->^; MUX2 S all four, B ^->^ v->v; XOR2 A all four; TBUF A
    /// ^->v v->^, EN all four).
    /// ⚠️ ODD's function names a port the cell lacks: the reference warns (1130) and then CRASHES
    /// reading it; here the expression is absent and the arc non-unate.
    #[test]
    fn a_missing_timing_sense_comes_from_the_function() {
        let l = Lib::parse(r#"library(s) {
  delay_model : table_lookup;
  time_unit : "1ns"; capacitive_load_unit (1, pf); voltage_unit : "1V"; current_unit : "1mA";
  cell (AND2) { pin(A) { direction : input; capacitance : 0.001; } pin(B) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "A B";
      timing () { related_pin : "A"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } }
      timing () { related_pin : "B"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (NOR2) { pin(A) { direction : input; capacitance : 0.001; } pin(B) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "(A+B)'";
      timing () { related_pin : "A"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (MUX2) { pin(S) { direction : input; capacitance : 0.001; } pin(A) { direction : input; capacitance : 0.001; } pin(B) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "(A !S) | (B S)";
      timing () { related_pin : "S"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } }
      timing () { related_pin : "B"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (XOR2) { pin(A) { direction : input; capacitance : 0.001; } pin(B) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "A^B";
      timing () { related_pin : "A"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (TBUF) { pin(A) { direction : input; capacitance : 0.001; } pin(EN) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "!A"; three_state : "!EN";
      timing () { related_pin : "A"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } }
      timing () { related_pin : "EN"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (ODD) { pin(A) { direction : input; capacitance : 0.001; }
    pin(Y) { direction : output; function : "A & IQ";
      timing () { related_pin : "A"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
}
"#).unwrap();
        let sense = |c: &str, from: &str| -> String {
            l.cells[c].pins["Y"].arcs.iter().find(|a| a.related_pin == from).map(|a| a.sense.clone()).unwrap()
        };
        assert_eq!(sense("AND2", "A"), "positive_unate");
        assert_eq!(sense("AND2", "B"), "positive_unate");
        assert_eq!(sense("NOR2", "A"), "negative_unate");
        assert_eq!(sense("MUX2", "S"), "non_unate");
        assert_eq!(sense("MUX2", "B"), "positive_unate");
        assert_eq!(sense("XOR2", "A"), "non_unate");
        assert_eq!(sense("TBUF", "A"), "negative_unate");
        assert_eq!(sense("TBUF", "EN"), "non_unate");
        assert_eq!(sense("ODD", "A"), "non_unate");
    }
}

#[cfg(test)]
mod sequential_without_timing_type {
    use super::*;

    /// Reference rule (checked on this library with `report_lib_cell`): a flop's timing group with
    /// no `timing_type` is a clock-to-Q arc from the RISING clock edge — on DFFN too, whose
    /// `clocked_on` is `!CLKN` (the reference's own quirk, reproduced) — and one from the `clear`
    /// pin is a set/clear arc, which is not a delay arc here.
    #[test]
    fn a_flop_group_without_timing_type_is_clock_to_q() {
        let l = Lib::parse(r#"library(s) {
  delay_model : table_lookup;
  time_unit : "1ns"; capacitive_load_unit (1, pf); voltage_unit : "1V"; current_unit : "1mA";
  cell (DFF) { ff (IQ, IQN) { next_state : "D"; clocked_on : "CLK"; }
    pin(CLK) { direction : input; clock : true; capacitance : 0.001; }
    pin(D) { direction : input; capacitance : 0.001; }
    pin(Q) { direction : output; function : "IQ"; timing () { related_pin : "CLK"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (DFFN) { ff (IQ, IQN) { next_state : "D"; clocked_on : "!CLKN"; }
    pin(CLKN) { direction : input; clock : true; capacitance : 0.001; }
    pin(D) { direction : input; capacitance : 0.001; }
    pin(Q) { direction : output; function : "IQ"; timing () { related_pin : "CLKN"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
  cell (DFFR) { ff (IQ, IQN) { next_state : "D"; clocked_on : "CLK"; clear : "!RESET_B"; }
    pin(CLK) { direction : input; clock : true; capacitance : 0.001; }
    pin(D) { direction : input; capacitance : 0.001; }
    pin(RESET_B) { direction : input; capacitance : 0.001; }
    pin(Q) { direction : output; function : "IQ";
      timing () { related_pin : "CLK"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } }
      timing () { related_pin : "RESET_B"; cell_rise (scalar) { values("0.1"); } cell_fall (scalar) { values("0.1"); } rise_transition (scalar) { values("0.1"); } fall_transition (scalar) { values("0.1"); } } } }
}
"#).unwrap();
        for (c, ck) in [("DFF", "CLK"), ("DFFN", "CLKN"), ("DFFR", "CLK")] {
            let q = &l.cells[c].pins["Q"];
            assert_eq!(q.arcs.len(), 1, "{c}: only the clock arc is a delay arc");
            assert_eq!((q.arcs[0].related_pin.as_str(), q.arcs[0].clk_edge), (ck, Some(ClkEdge::Rise)), "{c}");
        }
    }
}

#[cfg(test)]
mod table_rows_and_includes {
    use super::*;

    /// Reference rule: a bare number in `values` is a row of one; rows are padded to the axes with
    /// 0. Checked on the reference: this cell's fall delay at (0.1 ns, 0.001 pF) is 0.2 and its
    /// rise 0.3 (it warns 1259 / 1261 on the fall table).
    #[test]
    fn bare_numbers_are_rows_padded_to_the_axes() {
        let l = Lib::parse(
            r#"library(u) {
  delay_model : table_lookup; time_unit : "1ns"; capacitive_load_unit (1, pf);
  lu_table_template (t) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 ("0.01, 0.1"); index_2 ("0.001, 0.01"); }
  cell (INVQ) { pin(A) { direction : input; capacitance : 0.002; }
    pin(Y) { direction : output; function : "!A";
      timing () { related_pin : "A"; timing_sense : negative_unate;
        cell_rise (t) { values ("0.1, 0.2", "0.3, 0.4"); }
        cell_fall (t) { values (0.1, 0.2, 0.3, 0.4); } } } }
}"#,
        )
        .unwrap();
        let a = &l.cells["INVQ"].pins["Y"].arcs[0];
        assert_eq!(a.cell_fall.values, vec![vec![0.1f32 as f64, 0.0], vec![0.2f32 as f64, 0.0]]);
        assert!((a.cell_fall.lookup(0.1, 0.001) - 0.2).abs() < 1e-6);
        assert!((a.cell_rise.lookup(0.1, 0.001) - 0.3).abs() < 1e-6);
    }

    /// `include_file` splices the named file in (one level), as the reference's lexer does.
    #[test]
    fn include_file_brings_its_cells() {
        let dir = std::env::temp_dir().join(format!("vyges_inc_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let extra = dir.join("extra.lib");
        std::fs::write(&extra, "  cell (BUFX) { pin(A) { direction : input; capacitance : 0.003; } pin(Y) { direction : output; function : \"A\"; } }\n").unwrap();
        let main = dir.join("main.lib");
        std::fs::write(&main, format!("library(u) {{\n  time_unit : \"1ns\";\n  include_file ({});\n  cell (INV) {{ pin(A) {{ direction : input; }} }}\n}}\n", extra.display())).unwrap();
        let l = Lib::load(main.to_str().unwrap()).unwrap();
        assert!(l.cells.contains_key("BUFX") && l.cells.contains_key("INV"), "{:?}", l.cells.keys().collect::<Vec<_>>());
        let _ = std::fs::remove_dir_all(&dir);
    }
}

/// Library units, the reference's way: each library's own, defaults 1 ns / 1 pF / 1 mA, and a
/// merged library is in the FIRST one's units.
#[cfg(test)]
mod unit_tests {
    use super::*;

    /// One inverter, its numbers given in nanoseconds and picofarads and written in `time` /
    /// `cap` units (`scale` converts: 1000 for ps and fF), with a CCS vector.
    fn inv(name: &str, time: &str, cap: &str, current: &str, scale: f64) -> String {
        let n = |v: f64| format!("{}", v * scale);
        format!(
            "library ({name}) {{\n delay_model : table_lookup;\n time_unit : \"{time}\";\n capacitive_load_unit (1, {cap});\n current_unit : \"{current}\";\n\
             lu_table_template (t) {{ variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; index_1 (\"{a}, {b}\"); index_2 (\"{c}, {d}\"); }}\n\
             cell (INV_{name}) {{\n pin (A) {{ direction : input; capacitance : {cap_a}; }}\n pin (Y) {{ direction : output; function : \"!A\";\n\
             timing () {{ related_pin : \"A\"; timing_sense : negative_unate;\n\
             cell_rise (t) {{ values (\"{v1}, {v2}\", \"{v3}, {v4}\"); }}\n cell_fall (t) {{ values (\"{v1}, {v2}\", \"{v3}, {v4}\"); }}\n\
             rise_transition (t) {{ values (\"{v1}, {v2}\", \"{v3}, {v4}\"); }}\n fall_transition (t) {{ values (\"{v1}, {v2}\", \"{v3}, {v4}\"); }}\n\
             output_current_rise () {{ vector (ccs) {{ reference_time : {rt}; index_1 (\"{a}\"); index_2 (\"{c}\"); index_3 (\"{t0}, {t1}\"); values (\"0.1, 0.2\"); }} }}\n\
             }} }} }}\n}}\n",
            a = n(0.01), b = n(0.08), c = n(0.001), d = n(0.01), cap_a = n(0.0015),
            v1 = n(0.08), v2 = n(0.2), v3 = n(0.12), v4 = n(0.28), rt = n(0.005), t0 = n(0.01), t1 = n(0.02),
        )
    }

    #[test]
    fn units_default_as_the_reference_reads_them() {
        let l = Lib::parse("library (d) { cell (X) { pin (A) { direction : input; } } }").unwrap();
        assert_eq!(l.units, LibUnits::default());
        assert_eq!((l.units.time_name(), l.units.cap_name()), ("1ns".to_string(), "1pF".to_string()));
        let ps = Lib::parse(&inv("p", "1ps", "ff", "1uA", 1000.0)).unwrap();
        assert_eq!((ps.units.time_name(), ps.units.cap_name()), ("1ps".to_string(), "1fF".to_string()));
        assert_eq!((ps.units.time_label(), l.units.time_label()), ("ps".to_string(), "ns".to_string()));
        let ten = LibUnits { time_s: 1e-11, ..LibUnits::default() };
        assert_eq!((ten.time_name(), ten.time_label()), ("10ps".to_string(), "10ps".to_string()));
        // a quoted suffix is the same unit
        let quoted = Lib::parse(&inv("q", "1ps", "\"ff\"", "1uA", 1000.0)).unwrap();
        assert_eq!(quoted.units, ps.units);
        // `nf` is not a capacitance unit the reference reads: the default stays.
        let nf = Lib::parse(&inv("n", "1ns", "nf", "1mA", 1.0)).unwrap();
        assert_eq!(nf.units.cap_f, 1e-12);
    }

    /// The same inverter in ps / fF merged into a ns / pF library is the ns / pF inverter.
    #[test]
    fn a_merged_library_is_in_the_first_ones_units() {
        let ns = Lib::parse(&inv("a", "1ns", "pf", "1mA", 1.0)).unwrap();
        let mut merged = Lib::parse(&inv("b", "1ns", "pf", "1mA", 1.0)).unwrap();
        merged.cells.clear();
        merged.merge(Lib::parse(&inv("a", "1ps", "ff", "1uA", 1000.0)).unwrap());
        let (want, got) = (&ns.cells["INV_a"], &merged.cells["INV_a"]);
        // values are read as `strtof` reads them, so the two spellings agree to float precision
        let close = |x: f64, y: f64| (x - y).abs() <= 1e-6 * y.abs().max(1e-9);
        assert!(close(got.pins["A"].capacitance, want.pins["A"].capacitance));
        let (wa, ga) = (&want.pins["Y"].arcs[0], &got.pins["Y"].arcs[0]);
        for (w, g) in [(&wa.cell_rise, &ga.cell_rise), (&wa.fall_transition, &ga.fall_transition)] {
            assert!(w.index_1.iter().zip(&g.index_1).all(|(x, y)| close(*y, *x)));
            assert!(w.index_2.iter().zip(&g.index_2).all(|(x, y)| close(*y, *x)));
            assert!(w.values.iter().flatten().zip(g.values.iter().flatten()).all(|(x, y)| close(*y, *x)));
        }
        // CCS: the time axis in ns, the current in pF/ns = mA (1 uA here is 1e-3 of it).
        let (ww, gw) = (&wa.ccs.rise[0], &ga.ccs.rise[0]);
        assert!(close(gw.ref_time, ww.ref_time) && close(gw.time[1], ww.time[1]) && close(gw.in_slew, ww.in_slew));
        assert!(close(gw.current[1], ww.current[1] * 1e-3));
        assert_eq!(merged.units, LibUnits::default());
    }

    /// The first library holding a cell is the one it comes from.
    #[test]
    fn the_first_library_holding_a_cell_wins() {
        let mut first = Lib::parse(&inv("a", "1ns", "pf", "1mA", 1.0)).unwrap();
        let second = Lib::parse(&inv("a", "1ns", "pf", "1mA", 2.0)).unwrap();
        let cap = first.cells["INV_a"].pins["A"].capacitance;
        first.merge(second);
        assert_eq!(first.cells["INV_a"].pins["A"].capacitance, cap);
    }
}

#[cfg(test)]
mod dont_use_cells {
    use super::*;

    const LIB: &str = r#"library (d) {
  cell (INV_X1) { cell_footprint : "inv"; area : 1.0; dont_use : true;
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "!A"; } }
  cell (INV_X2) { cell_footprint : "inv"; area : 2.0; dont_use : true;
    pin (A) { direction : input; capacitance : 0.002; }
    pin (Y) { direction : output; function : "!A"; } }
  cell (INV_X4) { cell_footprint : "inv"; area : 4.0; dont_use : false;
    pin (A) { direction : input; capacitance : 0.004; }
    pin (Y) { direction : output; function : "!A"; } }
  cell (INV_X8) { cell_footprint : "inv"; area : 8.0; dont_use : yes;
    pin (A) { direction : input; capacitance : 0.008; }
    pin (Y) { direction : output; function : "!A"; } }
}"#;

    /// Reference rule: a cell's `dont_use` is a boolean attribute — exactly `true` or `false`;
    /// any other value is warned about and leaves the cell usable. The resizer then never offers
    /// a dont_use cell as a swap or buffer candidate, while a design may still contain one.
    #[test]
    fn a_dont_use_cell_is_never_a_replacement_but_keeps_its_own_rung() {
        let l = Lib::parse(LIB).unwrap();
        assert!(l.cells["INV_X1"].dont_use && l.cells["INV_X2"].dont_use);
        assert!(!l.cells["INV_X4"].dont_use, "`false` is usable");
        assert!(!l.cells["INV_X8"].dont_use, "a non-boolean value leaves the default");
        let class = |c: &str| l.equivalence_class(c).iter().map(|c| c.name.clone()).collect::<Vec<_>>();
        assert_eq!(class("INV_X1"), ["INV_X1", "INV_X4", "INV_X8"], "X2 is withheld; X1 keeps its rung");
        assert_eq!(class("INV_X4"), ["INV_X4", "INV_X8"]);
        let ups: Vec<_> = l.upsize_candidates("INV_X1").iter().map(|c| c.name.clone()).collect();
        assert_eq!(ups, ["INV_X4", "INV_X8"]);
        assert!(l.downsize_candidates("INV_X4").is_empty(), "X1 and X2 are both dont_use");
        // the cache round-trips the flag
        let back = crate::libcache::decode(&crate::libcache::encode(&l)).unwrap();
        assert!(back.cells["INV_X2"].dont_use && !back.cells["INV_X4"].dont_use);
    }

    /// Reference rule (`makeLatchDtoQArcs`): a latch's D->Q arc pairs edges by the sense of the
    /// latch's `data_in` at D — negative pairs opposite edges, anything else like edges — whatever
    /// the timing group declares and whichever of IQ / IQN the output is. So sky130 `dlrbn`'s
    /// D->Q_N, declared negative_unate, is timed D rise -> Q_N rise. The edges survive the cache.
    #[test]
    fn a_latch_d_to_q_arc_pairs_edges_by_the_data_in_sense() {
        let l = Lib::parse(
            r#"library (l) {
  cell (LAT) { area : 1.0;
    latch (IQ, IQN) { enable : "G"; data_in : "D"; }
    pin (D) { direction : input; capacitance : 0.001; }
    pin (G) { direction : input; capacitance : 0.001; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "D"; timing_sense : positive_unate; cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); } } }
    pin (QN) { direction : output; function : "IQN";
      timing () { related_pin : "D"; timing_sense : negative_unate; cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); } } } }
  cell (LATI) { area : 1.0;
    latch (IQ, IQN) { enable : "G"; data_in : "!D"; }
    pin (D) { direction : input; capacitance : 0.001; }
    pin (G) { direction : input; capacitance : 0.001; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "D"; cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); } } } }
  cell (LATX) { area : 1.0;
    latch (IQ, IQN) { enable : "G"; data_in : "D ^ S"; }
    pin (D) { direction : input; capacitance : 0.001; }
    pin (S) { direction : input; capacitance : 0.001; }
    pin (G) { direction : input; capacitance : 0.001; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "D"; cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); } } } }
  cell (INV) { area : 1.0;
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "!A";
      timing () { related_pin : "A"; cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); } } } }
}"#,
        )
        .unwrap();
        let pairs = |l: &Lib, c: &str, p: &str| {
            let a = &l.cells[c].pins[p].arcs[0];
            [(true, true), (true, false), (false, true), (false, false)].map(|(f, t)| a.feeds(f, t))
        };
        let like = [true, false, false, true];
        let opposite = [false, true, true, false];
        let back = crate::libcache::decode(&crate::libcache::encode(&l)).unwrap();
        for lib in [&l, &back] {
            assert_eq!(pairs(lib, "LAT", "Q"), like);
            assert_eq!(pairs(lib, "LAT", "QN"), like, "data_in D is positive: the IQN output does not invert it");
            assert_eq!(pairs(lib, "LATI", "Q"), opposite, "data_in !D is negative");
            assert_eq!(pairs(lib, "LATX", "Q"), like, "a NON-unate data_in still pairs like edges only");
            assert_eq!(pairs(lib, "INV", "Y"), opposite, "a plain arc keeps its function's sense");
        }
    }

    /// Reference rule (`makeTristateEnableArcs` / `makeTristateDisableArcs`): a tri-state arc
    /// launches on ONE input edge to both output edges — the rise when its sense is positive, the
    /// fall when negative. With no `timing_sense`, an enable arc takes the sense of the tri-state
    /// ENABLE (`three_state` inverted) at the pin and a disable arc the opposite. sky130 `ebufn`
    /// (`three_state : "TE_B"`): enabled by TE_B falling, disabled by TE_B rising.
    #[test]
    fn a_tristate_arc_launches_on_one_input_edge() {
        let t = r#"cell_rise (scalar) { values ("1"); } cell_fall (scalar) { values ("2"); }"#;
        let l = Lib::parse(&format!(
            r#"library (t) {{
  cell (EBUF) {{ area : 1.0;
    pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (TE_B) {{ direction : input; capacitance : 0.001; }}
    pin (Z) {{ direction : output; function : "A"; three_state : "TE_B";
      timing () {{ related_pin : "TE_B"; timing_type : three_state_enable; {t} }}
      timing () {{ related_pin : "TE_B"; timing_type : three_state_disable; {t} }}
      timing () {{ related_pin : "A"; {t} }} }} }}
  cell (EBUFD) {{ area : 1.0;
    pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (TE_B) {{ direction : input; capacitance : 0.001; }}
    pin (Z) {{ direction : output; function : "A"; three_state : "TE_B";
      timing () {{ related_pin : "TE_B"; timing_type : three_state_enable; timing_sense : positive_unate; {t} }} }} }}
}}"#
        ))
        .unwrap();
        let pairs = |l: &Lib, c: &str, k: usize| {
            let a = &l.cells[c].pins["Z"].arcs[k];
            [(true, true), (true, false), (false, true), (false, false)].map(|(f, t)| a.feeds(f, t))
        };
        let from_rise = [true, true, false, false];
        let from_fall = [false, false, true, true];
        let back = crate::libcache::decode(&crate::libcache::encode(&l)).unwrap();
        for lib in [&l, &back] {
            let kinds: Vec<&str> = lib.cells["EBUF"].pins["Z"].arcs.iter().map(|a| a.kind.as_str()).collect();
            assert_eq!(kinds, ["three_state_enable", "three_state_disable", ""]);
            assert_eq!(pairs(lib, "EBUF", 0), from_fall, "enabled by TE_B falling, to either edge");
            assert_eq!(pairs(lib, "EBUF", 1), from_rise, "disabled by TE_B rising, to either edge");
            assert_eq!(pairs(lib, "EBUF", 2), [true, false, false, true], "A->Z stays positive unate");
            assert_eq!(pairs(lib, "EBUFD", 0), from_rise, "a declared sense is taken as written");
        }
    }

    /// Reference rule (`translatePresetClrCheckRoles`): a setup / hold check on a pin that a
    /// set/clear arc is timed from becomes recovery / removal. The arc exists only with the table of
    /// the edge it drives (`makePresetClrArcs`: clear -> fall, preset -> rise), and an untyped group
    /// from the `ff`'s clear/preset pin is one too (`makeTimingArcs`).
    #[test]
    fn checks_on_a_set_or_clear_pin_are_recovery_and_removal() {
        let chk = r#"timing () { related_pin : "CK"; timing_type : setup_rising; rise_constraint (scalar) { values ("1"); } }
      timing () { related_pin : "CK"; timing_type : hold_rising; rise_constraint (scalar) { values ("2"); } }"#;
        let cell = |name: &str, q_timing: &str| {
            format!(
                r#"cell ({name}) {{ area : 1.0;
    ff (IQ, IQN) {{ clocked_on : "CK"; next_state : "D"; clear : "!RN"; }}
    pin (CK) {{ direction : input; clock : true; capacitance : 0.001; }}
    pin (D) {{ direction : input; capacitance : 0.001; {chk} }}
    pin (RN) {{ direction : input; capacitance : 0.001; {chk} }}
    pin (Q) {{ direction : output; function : "IQ"; {q_timing} }} }}"#
            )
        };
        let l = Lib::parse(&format!(
            "library (r) {{ {} {} {} }}",
            cell("EXPL", r#"timing () { related_pin : "RN"; timing_type : clear; timing_sense : positive_unate; cell_fall (scalar) { values ("3"); } }"#),
            cell("INFER", r#"timing () { related_pin : "RN"; timing_sense : positive_unate; cell_fall (scalar) { values ("3"); } }"#),
            cell("NOFALL", r#"timing () { related_pin : "RN"; timing_type : clear; cell_rise (scalar) { values ("3"); } }"#),
        ))
        .unwrap();
        let roles = |c: &str, p: &str| {
            let p = &l.cells[c].pins[p];
            (p.setup.len(), p.hold.len(), p.recovery.len(), p.removal.len())
        };
        assert_eq!(roles("EXPL", "RN"), (0, 0, 1, 1), "a timing_type clear arc from RN");
        assert_eq!(roles("INFER", "RN"), (0, 0, 1, 1), "an untyped group from the ff's clear pin");
        assert_eq!(roles("NOFALL", "RN"), (1, 1, 0, 0), "a clear with no fall table builds no arc");
        for c in ["EXPL", "INFER", "NOFALL"] {
            assert_eq!(roles(c, "D"), (1, 1, 0, 0), "{c}: D keeps its setup and hold");
        }
    }

    /// Reference rule (`readTableModel` / `readTableTemplates`): a timing table is looked up in the
    /// library's `lu_table_template`s only (plus the built-in `scalar`); one naming any other
    /// template — undefined, or a `power_lut_template` — is DROPPED (warning 1253), and an arc
    /// with no table of an edge has no arc for it. A template defined twice keeps the later axes.
    #[test]
    fn a_table_with_no_timing_template_is_dropped() {
        let l = Lib::parse(
            r#"library (t) {
  lu_table_template (dl) { variable_1 : input_net_transition; variable_2 : total_output_net_capacitance; }
  lu_table_template (dl) { variable_1 : total_output_net_capacitance; variable_2 : input_net_transition; }
  power_lut_template (pw) { variable_1 : input_transition_time; variable_2 : total_output_net_capacitance; }
  cell (C) { area : 1.0;
    pin (A) { direction : input; capacitance : 0.001; }
    pin (Y) { direction : output; function : "A";
      timing () { related_pin : "A";
        cell_rise (dl) { index_1 ("1, 2"); index_2 ("10, 20"); values ("1, 2", "3, 4"); }
        cell_fall (pw) { index_1 ("1, 2"); index_2 ("10, 20"); values ("1, 2", "3, 4"); }
        rise_transition (scalar) { values ("5"); }
        fall_transition (nope) { values ("6"); } } } }
}"#,
        )
        .unwrap();
        let a = &l.cells["C"].pins["Y"].arcs[0];
        // the later `dl` puts the load first: the table is turned to (slew, load)
        assert_eq!(a.cell_rise.index_1, [10.0, 20.0], "axes by the LATER definition of dl");
        assert_eq!(a.cell_rise.values, [[1.0, 3.0], [2.0, 4.0]]);
        assert!(a.cell_fall.values.is_empty(), "a power template is not a timing template");
        assert_eq!(a.rise_transition.lookup(0.0, 0.0), 5.0, "scalar is built in");
        assert!(a.fall_transition.values.is_empty(), "an undefined template drops the table");
    }

    /// Reference rule (`getAttrBool`): `true` / `false` are matched IGNORING CASE (`stringEqual`
    /// folds case). gf180's SRAM macro writes `dont_use : TRUE;` and its clock pin could as well.
    #[test]
    fn boolean_attributes_ignore_case() {
        let l = Lib::parse(
            r#"library (b) {
  cell (M1) { area : 1.0; dont_use : TRUE;
    pin (CLK) { direction : input; clock : True; capacitance : 0.001; }
    pin (A) { direction : input; clock : FALSE; capacitance : 0.001; } }
  cell (M2) { area : 1.0; dont_use : False;
    pin (A) { direction : input; capacitance : 0.001; } }
}"#,
        )
        .unwrap();
        assert!(l.cells["M1"].dont_use, "TRUE is true");
        assert!(!l.cells["M2"].dont_use, "False is false");
        assert!(l.cells["M1"].pins["CLK"].clock, "True is true");
        assert!(!l.cells["M1"].pins["A"].clock);
    }
}

#[cfg(test)]
mod per_library_thresholds {
    use super::*;

    fn lib(name: &str, cell: &str, lower: u32, upper: u32) -> Lib {
        Lib::parse(&format!(
            r#"library ({name}) {{ time_unit : "1ns"; capacitive_load_unit (1, pf);
  slew_lower_threshold_pct_rise : {lower}; slew_lower_threshold_pct_fall : {lower};
  slew_upper_threshold_pct_rise : {upper}; slew_upper_threshold_pct_fall : {upper};
  input_threshold_pct_rise : 50; input_threshold_pct_fall : 50;
  output_threshold_pct_rise : 50; output_threshold_pct_fall : 50;
  cell ({cell}) {{ pin (A) {{ direction : input; capacitance : 0.001; }}
    pin (Y) {{ direction : output; function : "!A"; }} }} }}"#
        ))
        .unwrap()
    }

    /// Reference rule: each cell is timed with ITS OWN library's thresholds (the driver's for its
    /// waveform, the load's after `thresholdAdjust`); a port, with the first-read library's.
    #[test]
    fn a_later_librarys_cells_keep_their_own_thresholds() {
        let mut a = lib("a", "INV_A", 20, 80);
        a.merge(lib("b", "INV_B", 10, 90));
        a.merge(lib("c", "INV_C", 20, 80));
        assert_eq!(a.cell_thresholds("INV_A"), a.thresholds);
        assert_eq!(a.cell_thresholds("INV_C"), a.thresholds, "same thresholds: nothing to carry");
        assert_eq!(a.cells["INV_C"].thresholds, None);
        let b = a.cell_thresholds("INV_B");
        assert_eq!((b.slew_lower_rise, b.slew_upper_fall), (0.1, 0.9));
        assert_eq!(a.cell_thresholds("no_such_cell"), a.thresholds, "a port: the first library's");
    }
}

/// Which pins are clocks, by their ARCS — the facts the reference reader sets on each port
/// (`LibertyCell::makeTimingArcPortMaps`), none of which needs `clock : true`.
#[cfg(test)]
mod clock_roles_by_arcs {
    use super::*;

    const LIB: &str = r#"library (l) {
      cell (MACRO) {
        pin (LCLK) { direction : input; }
        pin (SEL) { direction : input;
          timing () { related_pin : "LCLK"; timing_type : non_seq_setup_rising; rise_constraint (scalar) { values ("0.1"); } }
          timing () { related_pin : "LCLK"; timing_type : non_seq_hold_falling; rise_constraint (scalar) { values ("0.1"); } } }
        pin (Q) { direction : output;
          timing () { related_pin : "LCLK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("0.1"); }
            cell_fall (scalar) { values ("1"); } fall_transition (scalar) { values ("0.1"); } } }
      }
      cell (DFR) {
        ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RESET_B"; }
        pin (CLK) { direction : input; clock : true; min_pulse_width_high : 0.2; }
        pin (D) { direction : input;
          timing () { related_pin : "CLK"; timing_type : setup_rising; rise_constraint (scalar) { values ("0.1"); } } }
        pin (RESET_B) { direction : input; min_pulse_width_low : 0.3;
          timing () { related_pin : "CLK"; timing_type : recovery_rising; rise_constraint (scalar) { values ("0.1"); } } }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "CLK"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.3"); } rise_transition (scalar) { values ("0.1"); }
            cell_fall (scalar) { values ("0.3"); } fall_transition (scalar) { values ("0.1"); } }
          timing () { related_pin : "RESET_B"; timing_type : falling_edge;
            cell_fall (scalar) { values ("0.2"); } fall_transition (scalar) { values ("0.1"); } } }
      }
      cell (LAT) {
        latch (IQ, IQN) { enable : "G"; data_in : "D"; }
        pin (G) { direction : input; }
        pin (D) { direction : input; }
        pin (Q) { direction : output; function : "IQ";
          timing () { related_pin : "G"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.3"); } rise_transition (scalar) { values ("0.1"); } }
          timing () { related_pin : "D"; timing_type : rising_edge;
            cell_rise (scalar) { values ("0.2"); } rise_transition (scalar) { values ("0.1"); } } }
      }
      cell (NC) {
        pin (A) { direction : input; }
        pin (B) { direction : input;
          timing () { related_pin : "A"; timing_type : nochange_high_high; } }
        pin (W) { direction : input;
          timing () { timing_type : min_pulse_width; rise_constraint (scalar) { values ("0.1"); } } }
      }
    }"#;

    fn pin(lib: &Lib, cell: &str, pin: &str) -> (bool, bool, bool) {
        let p = &lib.cell(cell).unwrap().pins[pin];
        (p.is_reg_clk, p.is_reg_output, p.is_check_clk)
    }

    /// Rules: an edge arc with no `ff` / `latch` behind its output is ASSUMED a register clock-to-Q
    /// (`makeRegLatchArcs`, `setHasInferedRegTimingArcs`), so its from-pin is a register clock with
    /// no `clock : true`; a non-sequential check is a timing check, so its related pin is a check
    /// clock. The macro's launch pin is both.
    #[test]
    fn a_macro_launch_pin_with_no_clock_attribute_is_a_register_and_check_clock() {
        let lib = Lib::parse(LIB).unwrap();
        assert!(!lib.cell("MACRO").unwrap().pins["LCLK"].clock);
        assert_eq!(pin(&lib, "MACRO", "LCLK"), (true, false, true));
        assert_eq!(pin(&lib, "MACRO", "Q"), (false, true, false));
        assert_eq!(pin(&lib, "MACRO", "SEL"), (false, false, false));
    }

    /// Rules: an edge arc from the `ff`'s `clear` pin is the set/clear arc, not clock-to-Q; a
    /// recovery check makes its related pin (CLK) a check clock; a `min_pulse_width_*` attribute is
    /// a width check from the pin to itself, so RESET_B is a check clock too.
    #[test]
    fn a_clear_edge_arc_is_not_a_clock_and_a_pulse_width_attribute_is_a_check_on_itself() {
        let lib = Lib::parse(LIB).unwrap();
        assert_eq!(pin(&lib, "DFR", "CLK"), (true, false, true));
        assert_eq!(pin(&lib, "DFR", "RESET_B"), (false, false, true));
        assert_eq!(pin(&lib, "DFR", "D"), (false, false, false));
        assert_eq!(pin(&lib, "DFR", "Q"), (false, true, false));
    }

    /// Rules: a latch's enable-to-Q is a register clock (`latchEnToQ`); its data-to-Q is not
    /// (`latchDtoQ`); `nochange_*` builds no arc, so marks nothing; a `min_pulse_width` group with no
    /// related pin is a check on the pin itself.
    #[test]
    fn latch_enable_nochange_and_a_bare_pulse_width_group() {
        let lib = Lib::parse(LIB).unwrap();
        assert_eq!(pin(&lib, "LAT", "G"), (true, false, false));
        assert_eq!(pin(&lib, "LAT", "D"), (false, false, false));
        assert_eq!(pin(&lib, "NC", "A"), (false, false, false));
        assert_eq!(pin(&lib, "NC", "W"), (false, false, true));
    }

    /// A macro with several launching pins, none marked `clock : true`, and a check made against a
    /// BUS (`related_bus_pins`): every launching pin is a register clock, every bit a check clock.
    #[test]
    fn a_macro_with_several_clocks_and_a_bus_related_check() {
        let lib = Lib::parse(r#"library (l) {
          type (b2) { base_type : array; data_type : bit; bit_width : 2; bit_from : 1; bit_to : 0; downto : true; }
          cell (PHY) {
            pin (TXCLK) { direction : input; }
            pin (RXCLK) { direction : input; }
            bus (CK) { bus_type : b2; direction : input; }
            pin (EN) { direction : input;
              timing () { related_bus_pins : "CK"; timing_type : setup_rising; rise_constraint (scalar) { values ("0.1"); } } }
            pin (TX) { direction : output;
              timing () { related_pin : "TXCLK"; timing_type : rising_edge; cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("0.1"); } } }
            pin (RX) { direction : output;
              timing () { related_pin : "RXCLK"; timing_type : falling_edge; cell_rise (scalar) { values ("1"); } rise_transition (scalar) { values ("0.1"); } } }
          }
        }"#).unwrap();
        let c = lib.cell("PHY").unwrap();
        assert_eq!(c.clock_pin, None);
        assert_eq!(c.reg_clk_pins(), ["RXCLK", "TXCLK"]);
        assert!(c.pins["CK[0]"].is_check_clk && c.pins["CK[1]"].is_check_clk);
        assert!(!c.pins["EN"].is_check_clk);
    }

    /// The facts survive the library cache.
    #[test]
    fn the_cache_keeps_the_clock_facts() {
        let lib = Lib::parse(LIB).unwrap();
        let back = crate::libcache::decode(&crate::libcache::encode(&lib)).unwrap();
        for (c, p) in [("MACRO", "LCLK"), ("DFR", "RESET_B"), ("LAT", "G"), ("DFR", "Q")] {
            assert_eq!(pin(&back, c, p), pin(&lib, c, p), "{c}/{p}");
        }
    }
}
