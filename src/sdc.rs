//! SDC reader — the standard timing-constraint format.
//!
//! Real flows (synthesis, OpenROAD/LibreLane) emit `.sdc`. This module parses
//! the sign-off-relevant subset of SDC (which is Tcl) into a self-contained
//! [`Sdc`] model — clocks, I/O timing, uncertainty, derates, and timing
//! exceptions — that loom holds as part of the shared design DB. Engines (e.g.
//! `vyges-sta-si`) consume the `Sdc` and apply it to their own job model; the
//! netlist, libraries, and SPEF come from their own readers (not from SDC).
//!
//! Supported commands (others are collected in [`Sdc::ignored`], never fatal):
//!
//! - `create_clock -name N -period P [-waveform {e0 e1 …}] [-add] {obj}` — without `-add` it
//!   replaces the clocks on its sources, and a name already defined is redefined in place
//!   (see `Sdc::delete_pin_clocks`) — and
//!   `create_generated_clock -name N -source S -divide_by D -multiply_by M {obj}`
//! - `set_input_delay` / `set_output_delay` (default via `all_inputs`/`all_outputs`
//!   or `-clock`, plus per-port overrides) — the I/O timing budget, each with its
//!   clock, flags (`-max`, `-min`, `-rise`, `-fall`, `-add_delay`, …) and accessor
//! - `set_clock_uncertainty [-setup|-hold]` — setup/hold guard band
//! - `set_clock_latency` — captured (source latency applied to the I/O budget), and
//!   kept per command
//! - `set_input_transition`, `set_load` — boundary slew / load
//! - `set_timing_derate -late|-early` — OCV derate
//! - `set_false_path` / `set_multicycle_path` — timing exceptions
//! - `set_units` — time/capacitance scaling to the engine's ns/pF
//!
//! Kept for a consumer that models them, and still listed in [`Sdc::ignored`] for
//! one that does not: `set_max_transition`, `set_max_fanout`, `set_max_capacitance`
//! on the design; `set_driving_cell`; `set_propagated_clock`'s objects;
//! `set_max_delay` / `set_min_delay` as written.
//!
//! The parser is std-only: it joins `\`-continuations, strips `#` comments,
//! resolves `set var`/`$var`, and understands `{...}` groups, `[get_* ...]`
//! accessors, and `[all_inputs]`/`[all_outputs]`.

use std::collections::HashMap;

/// A timing exception kind (`set_false_path` / `set_multicycle_path`). Defined
/// here in loom (the data plane) so the SDC model is self-contained; engines
/// consume these as part of the shared design DB.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExcKind {
    FalsePath,
    Multicycle(u32),
}

/// One timing exception: a kind plus its `-from`/`-to` endpoints (`*` = any).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Exception {
    pub kind: ExcKind,
    /// Every object named by `-from` / `-rise_from` / `-fall_from`. Empty, or containing
    /// `*`, means "any".
    ///
    /// A **set**, not one name: a sign-off SDC routinely cuts a whole bus with
    /// `-from [list [get_ports {a[0]}] [get_ports {a[1]}] ...]`, and keeping only the first
    /// left every other member of that bus timed while the SDC said it was cut. The failure
    /// was silent in the worst direction — paths the design has declared unreal get reported
    /// as violations, and an engineer chases them before concluding the tool is wrong.
    pub from: Vec<String>,
    /// Every object named by `-to` / `-rise_to` / `-fall_to`. See [`Exception::from`].
    pub to: Vec<String>,
}

/// Is `name` in an exception endpoint set? An empty set, or one holding `*`, matches anything.
fn endpoint_matches(set: &[String], name: &str) -> bool {
    set.is_empty() || set.iter().any(|o| o == "*" || o == name)
}

impl Exception {
    /// Does this exception's `-from` set cover `name`?
    pub fn from_matches(&self, name: &str) -> bool {
        endpoint_matches(&self.from, name)
    }
    /// Does this exception's `-to` set cover `name`?
    pub fn to_matches(&self, name: &str) -> bool {
        endpoint_matches(&self.to, name)
    }
    /// Does it cover this launch→capture pair?
    pub fn covers(&self, launch: &str, capture: &str) -> bool {
        self.from_matches(launch) && self.to_matches(capture)
    }
    /// Endpoint names that are not the `*` wildcard — what a linter can check for existence.
    pub fn named_endpoints(&self) -> impl Iterator<Item = (&'static str, &String)> {
        self.from
            .iter()
            .map(|o| ("-from", o))
            .chain(self.to.iter().map(|o| ("-to", o)))
            .filter(|(_, o)| o.as_str() != "*" && !o.is_empty())
    }
}

/// A parsed clock (regular or fully-resolved generated).
#[derive(Debug, Clone)]
pub struct SdcClock {
    pub name: String,
    /// Port name or `inst/pin`. **Empty for a virtual clock** — see [`SdcClock::is_virtual`].
    pub source: String,
    /// Every source object, in the order the command lists them (`source` is the first).
    /// `create_clock -period P {a b c}` is ONE clock on three ports: each is a clock pin.
    pub sources: Vec<String>,
    pub period: f64, // ns
    /// `-waveform {e0 e1 …}`: the edge times as written, scaled like `period`; `None` without the
    /// flag (the default `{0, period/2}`). Every edge is kept — a consumer that models two must
    /// refuse any other count, including the empty list an unreadable edge leaves.
    pub waveform: Option<Vec<f64>>,
}

impl SdcClock {
    /// A clock with no source object: it constrains I/O timing but launches nothing in this
    /// design, so nothing should look for a port to attach it to.
    pub fn is_virtual(&self) -> bool {
        self.source.is_empty()
    }
}

/// One `set_input_delay`/`set_output_delay`: a value plus its target. `default`
/// means it came from `all_inputs`/`all_outputs` (or a bare `-clock`).
#[derive(Debug, Clone)]
pub struct IoDelay {
    /// The delay (ns); 0 when it could not be read — see [`IoDelay::value_read`].
    pub value: f64, // ns
    /// The value positional was a number. False for a missing value or an unresolved variable,
    /// which leave `value` 0: a consumer that must not guess refuses it.
    pub value_read: bool,
    pub default: bool,
    pub ports: Vec<String>,
    /// `-clock`'s clock, resolved (`[get_clocks {c}]` → `c`); `None` without the flag.
    pub clock: Option<String>,
    /// Every flag that takes no value, as written and in order: `-max`, `-min`, `-rise`, `-fall`,
    /// `-add_delay`, `-clock_fall`, `-level_sensitive`, `-network_latency_included`,
    /// `-source_latency_included`, or any other. A consumer reads the ones it models and refuses
    /// the rest — without them a `-min`-only delay reads as a setup delay.
    pub flags: Vec<String>,
    /// `-reference_pin`, resolved.
    pub reference_pin: Option<String>,
    /// The accessor that named the objects (`get_ports`, `get_pins`, `all_inputs`, … or `""`).
    pub accessor: String,
}

/// An object list as written: the accessor that named it (`get_clocks`, `get_pins`, … `list`, or
/// `""` for a bare name or brace list) and the names it resolves to.
#[derive(Debug, Clone, PartialEq)]
pub struct Objects {
    pub accessor: String,
    pub names: Vec<String>,
}

impl Objects {
    fn of(tok: &str) -> Objects {
        let accessor = tok.strip_prefix('[').and_then(|i| tokenize(i.trim_end_matches(']').trim()).into_iter().next()).unwrap_or_default();
        Objects { accessor, names: resolve_objs(tok) }
    }
}

/// One `set_max_delay` / `set_min_delay` as written: the value (scaled like an I/O delay), every
/// object-valued flag in order (`-from`, `-to`, `-through`, `-rise_from`, …) with its objects, and
/// every flag that takes no value (`-ignore_clock_latency`, `-probe`, `-reset_path`, …).
#[derive(Debug, Clone, PartialEq)]
pub struct PathDelayCmd {
    /// `set_max_delay` (else `set_min_delay`).
    pub max: bool,
    /// NaN when the value is not a number (an unresolved variable): never fatal here; a
    /// consumer that models path delays refuses it.
    pub value: f64,
    pub objects: Vec<(String, Objects)>,
    pub flags: Vec<String>,
}

/// A command's tokens split into flags with a value (from `valued`), flags without one, and
/// positionals. A negative number is a positional, not a flag.
fn split_args<'t>(toks: &'t [String], valued: &[&str]) -> (Vec<(&'t str, &'t String)>, Vec<String>, Vec<&'t String>) {
    let (mut with, mut flags, mut pos) = (Vec::new(), Vec::new(), Vec::new());
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        if valued.contains(&t.as_str()) {
            if let Some(v) = toks.get(k + 1) {
                with.push((t.as_str(), v));
            }
            k += 2;
            continue;
        }
        if t.starts_with('-') && t.parse::<f64>().is_err() {
            flags.push(t.clone());
        } else {
            pos.push(t);
        }
        k += 1;
    }
    (with, flags, pos)
}

/// One `set_load` / `set_input_transition` as written: its value in the SDC's user unit (NOT
/// scaled by `set_units` — the reference's `set_units` only checks, values stay in the library
/// unit), its flags (`-min`, `-rise`, `-pin_load`, …), the valued `-clock` if any, and its objects
/// with the accessor that named them (`get_nets`, `get_ports`, … or `""` for a bare list).
#[derive(Debug, Clone, PartialEq)]
pub struct EnvCmd {
    pub cmd: String,
    pub value: f64,
    pub flags: Vec<String>,
    pub clock: Option<String>,
    pub accessor: String,
    /// Flags written inside the object accessor's brackets, e.g. `-no_clocks` in
    /// `[all_inputs -no_clocks]`. The `*INPUTS*` sentinel alone cannot carry it.
    pub accessor_flags: Vec<String>,
    pub objects: Vec<String>,
}

/// One `set_driving_cell`: `-lib_cell`, `-library`, `-pin`, `-from_pin`, the input transitions
/// (`-input_transition_rise/-fall`, in the SDC's user unit; 0 when absent), any other flag, and
/// the ports.
#[derive(Debug, Clone, PartialEq)]
pub struct DrivingCell {
    pub lib_cell: Option<String>,
    pub library: Option<String>,
    pub pin: Option<String>,
    pub from_pin: Option<String>,
    pub input_transition: [f64; 2],
    pub flags: Vec<String>,
    pub objects: Vec<String>,
}

fn driving_cell(toks: &[String]) -> DrivingCell {
    let valued = ["-lib_cell", "-library", "-pin", "-from_pin", "-input_transition_rise", "-input_transition_fall", "-multiply_by"];
    let mut d = DrivingCell { lib_cell: None, library: None, pin: None, from_pin: None, input_transition: [0.0; 2], flags: Vec::new(), objects: Vec::new() };
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        if valued.contains(&t.as_str()) {
            let v = toks.get(k + 1).map(|v| v.trim_matches(|c| c == '{' || c == '}').to_string());
            match t.as_str() {
                "-lib_cell" => d.lib_cell = v,
                "-library" => d.library = v,
                "-pin" => d.pin = v,
                "-from_pin" => d.from_pin = v,
                "-input_transition_rise" => d.input_transition[0] = v.and_then(|v| v.parse().ok()).unwrap_or(0.0),
                "-input_transition_fall" => d.input_transition[1] = v.and_then(|v| v.parse().ok()).unwrap_or(0.0),
                other => d.flags.push(other.to_string()),
            }
            k += 2;
            continue;
        }
        if t.starts_with('-') {
            d.flags.push(t.clone());
        } else {
            d.objects.extend(resolve_objs(t));
        }
        k += 1;
    }
    d
}

/// `(value, objects)` positionals of an environment command, its flags, and a valued `-clock`.
fn env_cmd(toks: &[String]) -> Option<EnvCmd> {
    let mut flags = Vec::new();
    let mut clock = None;
    let mut pos = Vec::new();
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        if t == "-clock" {
            clock = toks.get(k + 1).cloned();
            k += 2;
            continue;
        }
        if t.starts_with('-') && t.parse::<f64>().is_err() {
            flags.push(t.clone());
        } else {
            pos.push(t.clone());
        }
        k += 1;
    }
    let value = pos.first()?.parse::<f64>().ok()?;
    let obj = pos.get(1)?;
    let inner = obj.strip_prefix('[').map(|i| tokenize(i.trim_end_matches(']').trim())).unwrap_or_default();
    let accessor = inner.first().cloned().unwrap_or_default();
    let accessor_flags = inner.iter().skip(1).filter(|t| t.starts_with('-')).cloned().collect();
    Some(EnvCmd { cmd: toks[0].clone(), value, flags, clock, accessor, accessor_flags, objects: resolve_objs(obj) })
}

#[derive(Debug, Default)]
pub struct Sdc {
    pub clocks: Vec<SdcClock>,
    /// Each clock definition a later `create_clock` overwrote (same name) or removed (its sources
    /// taken without `-add`), as it stood, in file order — not timed; for a linter to report.
    pub superseded_clocks: Vec<SdcClock>,
    pub input_delays: Vec<IoDelay>,
    pub output_delays: Vec<IoDelay>,
    pub setup_uncertainty: f64,
    pub hold_uncertainty: f64,
    pub clock_latency: f64, // source/network latency (ns), applied to the I/O budget
    pub input_transition: Option<f64>,
    pub load: Option<f64>,
    /// `set_max_transition V [current_design]`, ns: the design-wide slew limit, for a consumer
    /// that models it. The command is STILL recorded in [`Sdc::ignored`] — a consumer that does
    /// not model it must keep seeing it as timing-affecting.
    pub max_transition: Option<f64>,
    /// A `set_max_transition` on anything but the design (ports, clocks, cells): there is no
    /// value for it here, so a consumer modelling only [`Sdc::max_transition`] must refuse.
    pub max_transition_on_objects: bool,
    /// `set_max_fanout V [current_design]`: the design-wide fanout limit (unitless), for a consumer
    /// that models it. Recorded in [`Sdc::ignored`] as well, like `set_max_transition`.
    pub max_fanout: Option<f64>,
    /// A `set_max_fanout` on anything but the design.
    pub max_fanout_on_objects: bool,
    /// `set_max_capacitance V [current_design]`: the design-wide capacitance limit AS WRITTEN —
    /// not scaled by `set_units`, which in the reference only checks the unit; the value is in the
    /// library's capacitance unit. Recorded in [`Sdc::ignored`] as well, like `set_max_transition`.
    pub max_capacitance: Option<f64>,
    /// A `set_max_capacitance` on anything but the design.
    pub max_capacitance_on_objects: bool,
    /// Every `set_max_capacitance V <objects>` on anything but the design, in file order: the
    /// value AS WRITTEN (like [`Sdc::max_capacitance`]; NaN when it is not a number) and the
    /// objects as named, for a consumer that models port or pin limits.
    pub max_capacitance_objects: Vec<(f64, Objects)>,
    /// Every `set_propagated_clock`'s objects, in file order (still in [`Sdc::ignored`] for a
    /// consumer that does not model a propagated clock).
    pub propagated: Vec<Objects>,
    /// Every `set_clock_latency`, per command (its value is also folded into
    /// [`Sdc::clock_latency`]); a consumer that does not model latency refuses any.
    pub clock_latencies: Vec<EnvCmd>,
    /// Every `set_max_delay` / `set_min_delay`, in file order (still in [`Sdc::ignored`]).
    pub path_delays: Vec<PathDelayCmd>,
    /// Every `set_driving_cell`, as written (still in [`Sdc::ignored`] for a consumer that does
    /// not model input drives).
    pub driving_cells: Vec<DrivingCell>,
    /// Every `set_load` and `set_input_transition`, per command, for a consumer that models
    /// them per object ([`Sdc::load`] / [`Sdc::input_transition`] keep one design-wide value).
    pub env: Vec<EnvCmd>,
    pub late_derate: Option<f64>,
    pub early_derate: Option<f64>,
    pub exceptions: Vec<Exception>,
    /// Per entry of [`Sdc::exceptions`], index for index: the objects its `-from` side and its
    /// `-to` side named, accessor kept (`-rise_from` / `-fall_from` on the from side, likewise to).
    /// [`Exception::from`] / [`Exception::to`] are derived from these.
    pub exception_objects: Vec<[Vec<Objects>; 2]>,
    /// `set_clock_groups -asynchronous`: each inner vec is one `-group` of clock
    /// names. Two clocks in *different* groups are mutually asynchronous — paths
    /// that launch and capture across them are cut (no setup **or** hold check).
    pub async_groups: Vec<Vec<String>>,
    pub ignored: Vec<String>, // commands we recognised but do not model
}

/// SDC commands that CHANGE A TIMING ANSWER when they are not modelled.
///
/// Every unmodelled command is recorded in [`Sdc::ignored`], but that list mixes two very
/// different things. `set_dont_touch` and `set_max_area` are instructions to synthesis and cost
/// a timer nothing. `set_driving_cell` sets the slew every input path starts from — and it is
/// the most common input constraint there is: 480 uses against 65 of `set_input_transition`
/// across this organisation's own constraint files. Reporting both in one undifferentiated list
/// leaves the reader to know which is which.
///
/// Membership means: ignoring this can move a reported slack. It does not mean we should
/// necessarily model it — only that the warning must say so.
const TIMING_AFFECTING: &[&str] = &[
    // ── the environment a path starts and ends in ──────────────────────────────────────
    "set_driving_cell",      // input drive -> the slew every input path starts from
    "set_drive",             // ditto, as a resistance
    "set_operating_conditions", // the PVT corner itself — every delay in the design
    "set_voltage",           // operating voltage, likewise
    "set_resistance",        // a net's resistance, overriding what extraction said
    "set_wire_load_model",   // pre-layout delays come from here when there is no SPEF
    "set_wire_load_mode",
    "set_wire_load_selection_group",
    "set_wire_load_min_block_size",
    "set_fanout_load",       // feeds the fanout-based load estimate
    "set_port_fanout_number",
    // ── logic held constant, so paths through it cannot toggle ─────────────────────────
    "set_case_analysis",
    "set_logic_zero",
    "set_logic_one",
    "set_logic_dc",
    "set_disable_timing",    // arcs removed from the graph entirely
    // ── constraints that replace or reshape the clock-derived one ──────────────────────
    "set_max_delay",
    "set_min_delay",
    "set_max_time_borrow",   // how much a latch may borrow, hence whether a path passes
    "set_data_check",        // a non-clock setup/hold relationship
    "set_min_pulse_width",
    "group_path",            // path grouping changes what is reported as critical
    // ── the clock network ──────────────────────────────────────────────────────────────
    "set_clock_transition",  // the clock's own slew, hence every launch/capture delay
    "set_propagated_clock",  // ideal vs propagated -> latency and skew
    "set_clock_sense",       // which edge propagates through a divider or mux
    "set_sense",
    "set_clock_gating_check", // setup/hold on the gating enable
    "set_ideal_network",     // nets excluded from delay calculation
    "set_ideal_latency",
    "set_ideal_transition",
    "derive_clock_uncertainty", // generates the uncertainty we would otherwise be given
    "derive_pll_clocks",     // generates clocks that would otherwise not exist
    // ── design rules, whose violations ARE timing findings ─────────────────────────────
    "set_max_transition",
    "set_max_fanout",
    "set_max_capacitance",
    "set_min_capacitance",
    // ── SDC 2.x ────────────────────────────────────────────────────────────────────────
    "set_disable_clock_gating_check", // removes a check, so its violations vanish
    "unset_propagated_clock",         // reverts to an ideal network -> latency and skew
    "set_max_trans",                  // Tcl accepts unambiguous abbreviations
];

/// Explicitly NOT timing-affecting: instructions to synthesis, placement or power
/// optimisation. Listed rather than merely omitted, so the distinction is a decision on record
/// instead of an oversight — `set_max_area` and `set_driving_cell` are both "unmodelled", and
/// only one of them can move a slack.
pub const BENIGN_FOR_TIMING: &[&str] = &[
    "set_max_area",
    "set_max_dynamic_power",
    "set_max_leakage_power",
    "create_voltage_area",
    "set_level_shifter_strategy",
    "set_level_shifter_threshold",
    "set_dont_touch",
    "set_dont_use",
    "set_size_only",
    // synthesis / DFT / physical directives — nothing a timer consults
    "set_clock_gating_enable",
    "set_clock_gating_style",
    "set_clock_gating_verification",
    "set_timing_enable_verification",
    "set_dont_touch_network",
    "set_optimize_design",
    "set_scan_configuration",
    "set_min_porosity",
    "set_critical_range",  // steers optimisation effort; does not change a slack
];

impl Sdc {
    /// A `create_clock` without `-add` on `pins`: every other clock on any of them loses those
    /// pins; one left with none is removed, and so is what referred to it — its I/O delays, its
    /// latencies, its `set_propagated_clock`, its place in path delays (a `-from`/`-to` left
    /// empty deletes the command) and in clock groups. `defining` (the clock being made) keeps
    /// its entry: it is redefined. A virtual clock (no pins) replaces nothing.
    /// A false path or multicycle path loses the clock the same way, read from
    /// [`Sdc::exception_objects`] (only a `get_clocks` object names a clock; a port of the same
    /// name stays).
    fn delete_pin_clocks(&mut self, defining: &str, pins: &[String]) {
        let mut removed = Vec::new();
        for c in self.clocks.iter_mut().filter(|c| c.name != defining) {
            let before = c.sources.len();
            c.sources.retain(|s| !pins.contains(s));
            if c.sources.len() == before {
                continue;
            }
            match c.sources.first() {
                Some(first) => c.source = first.clone(),
                None => removed.push(c.name.clone()),
            }
        }
        let (gone, kept): (Vec<SdcClock>, Vec<SdcClock>) = std::mem::take(&mut self.clocks).into_iter().partition(|c| removed.contains(&c.name));
        self.clocks = kept;
        self.superseded_clocks.extend(gone);
        for name in &removed {
            self.remove_clock_references(name);
        }
    }

    /// Everything that names clock `name`, gone with it (see [`Sdc::delete_pin_clocks`]).
    fn remove_clock_references(&mut self, name: &str) {
        let is_it = |c: &Option<String>| c.as_deref() == Some(name);
        self.input_delays.retain(|d| !is_it(&d.clock));
        self.output_delays.retain(|d| !is_it(&d.clock));
        let drop_name = |o: &mut Objects| {
            if o.accessor == "get_clocks" {
                o.names.retain(|n| n != name);
            }
        };
        self.clock_latencies.retain_mut(|l| {
            if is_it(&l.clock) {
                return false;
            }
            if l.accessor == "get_clocks" {
                l.objects.retain(|n| n != name);
                return !l.objects.is_empty();
            }
            true
        });
        self.propagated.retain_mut(|o| {
            drop_name(o);
            !o.names.is_empty()
        });
        // An exception whose `-from` (else its `-to`) named the clock loses it; one with nothing
        // left on that side is deleted.
        self.path_delays.retain_mut(|p| {
            for side in ["from", "to"] {
                let on_side = |k: &str| k.trim_start_matches('-').trim_start_matches("rise_").trim_start_matches("fall_") == side;
                let names_it = p.objects.iter().any(|(k, o)| on_side(k) && o.accessor == "get_clocks" && o.names.iter().any(|n| n == name));
                if names_it {
                    p.objects.iter_mut().filter(|(k, _)| on_side(k)).for_each(|(_, o)| drop_name(o));
                    p.objects.retain(|(k, o)| !(on_side(k) && o.names.is_empty()));
                    return p.objects.iter().any(|(k, _)| on_side(k));
                }
            }
            true
        });
        for g in &mut self.async_groups {
            g.retain(|n| n != name);
        }
        self.async_groups.retain(|g| !g.is_empty());
        // `deleteExceptionsReferencing`: the `-from` side, if its clocks hold the clock, loses it,
        // else the `-to` side; a side left with no object deletes the exception.
        let mut keep = Vec::with_capacity(self.exceptions.len());
        for (e, objs) in self.exceptions.iter_mut().zip(self.exception_objects.iter_mut()) {
            let mut kept = true;
            for (side, list) in objs.iter_mut().enumerate() {
                if list.iter().any(|o| o.accessor == "get_clocks" && o.names.iter().any(|n| n == name)) {
                    list.iter_mut().for_each(drop_name);
                    list.retain(|o| !o.names.is_empty());
                    kept = !list.is_empty();
                    let names = reduce_exception_objects(list);
                    if side == 0 { e.from = names } else { e.to = names }
                    break;
                }
            }
            keep.push(kept);
        }
        let mut k = keep.iter();
        self.exceptions.retain(|_| *k.next().unwrap());
        let mut k = keep.iter();
        self.exception_objects.retain(|_| *k.next().unwrap());
    }

    /// The subset of [`Sdc::ignored`] that can move a reported slack — see [`TIMING_AFFECTING`].
    /// Deduplicated and sorted, so a file constraining forty ports with `set_driving_cell`
    /// yields one name rather than forty.
    pub fn ignored_affecting_timing(&self) -> Vec<&str> {
        let mut v: Vec<&str> = self
            .ignored
            .iter()
            .map(String::as_str)
            .filter(|c| TIMING_AFFECTING.contains(c))
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }

    /// One line on what was read and what was passed over, or `None` when nothing was ignored
    /// that could matter. Cheap enough to print on every load, which is the point.
    pub fn health(&self) -> Option<String> {
        let mut notes = Vec::new();
        // A clock with no period is read faithfully — the file says what it says — but every
        // setup check against it is meaningless, and a generated constraint file really does
        // sometimes write `-period 0.0`. Parse it, then say so.
        let bad: Vec<&str> = self
            .clocks
            .iter()
            .filter(|c| !(c.period.is_finite() && c.period > 0.0))
            .map(|c| c.name.as_str())
            .collect();
        if !bad.is_empty() {
            notes.push(format!(
                "clock(s) with no usable period: {} — every check against them is vacuous",
                bad.join(", ")
            ));
        }
        let affecting = self.ignored_affecting_timing();
        if !affecting.is_empty() {
            notes.push(format!(
                "{} unmodelled constraint(s) that can move a slack: {}",
                affecting.len(),
                affecting.join(", ")
            ));
        }
        (!notes.is_empty()).then(|| notes.join("; "))
    }
}

#[derive(Debug)]
pub struct SdcError(pub String);
impl std::fmt::Display for SdcError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "sdc error: {}", self.0)
    }
}
impl std::error::Error for SdcError {}

// ---- Tcl-subset lexing ---------------------------------------------------

/// Join `\`-continuations and split into logical command lines, dropping `#`
/// comments. A `#` starts a comment only at the beginning of a command (line
/// start, after whitespace) or after a `;`.
fn logical_lines(text: &str) -> Vec<String> {
    let mut joined = String::new();
    for raw in text.lines() {
        let line = raw.strip_suffix('\r').unwrap_or(raw);
        if let Some(stripped) = line.strip_suffix('\\') {
            joined.push_str(stripped);
            joined.push(' ');
        } else {
            joined.push_str(line);
            joined.push('\n');
        }
    }
    let mut out = Vec::new();
    for seg in joined.split(['\n', ';']) {
        let t = seg.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        out.push(t.to_string());
    }
    out
}

/// Split one command into tokens, keeping `{...}` and `[...]` groups whole
/// (nesting-aware) and respecting `"..."`. Braces are stripped from the token;
/// brackets are kept so accessors can be post-processed.
fn tokenize(line: &str) -> Vec<String> {
    let mut toks = Vec::new();
    let cs: Vec<char> = line.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        let c = cs[i];
        if c.is_whitespace() {
            i += 1;
            continue;
        }
        if c == '#' {
            break; // trailing comment
        }
        if c == '{' {
            let mut depth = 1;
            let mut s = String::new();
            i += 1;
            while i < cs.len() && depth > 0 {
                match cs[i] {
                    '{' => depth += 1,
                    '}' => {
                        depth -= 1;
                        if depth == 0 {
                            break;
                        }
                    }
                    _ => {}
                }
                s.push(cs[i]);
                i += 1;
            }
            i += 1; // past '}'
            toks.push(s);
        } else if c == '[' {
            let mut depth = 1;
            let mut s = String::from("[");
            i += 1;
            while i < cs.len() && depth > 0 {
                match cs[i] {
                    '[' => depth += 1,
                    ']' => depth -= 1,
                    _ => {}
                }
                s.push(cs[i]);
                i += 1;
            }
            toks.push(s);
        } else if c == '"' {
            let mut s = String::new();
            i += 1;
            while i < cs.len() && cs[i] != '"' {
                s.push(cs[i]);
                i += 1;
            }
            i += 1;
            toks.push(s);
        } else {
            let mut s = String::new();
            while i < cs.len() && !cs[i].is_whitespace() && cs[i] != '[' && cs[i] != '{' {
                s.push(cs[i]);
                i += 1;
            }
            toks.push(s);
        }
    }
    toks
}

/// Resolve a token to a list of object names. Handles `[get_ports {a b}]`,
/// `[get_pins x/y]`, `[get_clocks clk]`, `[all_inputs]`, `[all_outputs]`, a
/// brace list, or a bare name. Returns the sentinel `*INPUTS*` / `*OUTPUTS*`
/// for the `all_*` accessors so the caller can expand against the netlist.
fn resolve_objs(tok: &str) -> Vec<String> {
    if let Some(inner) = tok.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        let inner = inner.trim();
        let parts = tokenize(inner);
        if parts.is_empty() {
            return Vec::new();
        }
        match parts[0].as_str() {
            "all_inputs" => return vec!["*INPUTS*".into()],
            "all_outputs" => return vec!["*OUTPUTS*".into()],
            "all_registers" => return vec!["*REGS*".into()],
            // `[list [get_ports a] [get_ports b] ...]` — a Tcl list of object collections,
            // which real sign-off SDCs use for wide exceptions. Resolve each element and
            // concatenate. Without this the whole construct collapsed to the literal token
            // `list`, which matches nothing in the design: the exception silently did not
            // apply, and the timer went on timing paths the SDC meant to cut.
            "list" | "concat" => {
                let mut names = Vec::new();
                for p in &parts[1..] {
                    names.extend(resolve_objs(p));
                }
                return names;
            }
            "get_ports" | "get_pins" | "get_clocks" | "get_nets" | "get_cells" => {
                let mut names = Vec::new();
                for p in &parts[1..] {
                    if p.starts_with('-') {
                        continue; // e.g. -hierarchical
                    }
                    names.extend(p.split_whitespace().map(|s| s.to_string()));
                }
                return names;
            }
            _ => return vec![parts[0].clone()],
        }
    }
    // brace list or bare name -> split on whitespace
    tok.split_whitespace().map(|s| s.to_string()).collect()
}

/// A clock period, accepting the two forms real tools write: a TIME, or a FREQUENCY.
///
/// Quartus documents `-period "100 MHz"` as an alternative to a time, and Intel FPGA board
/// constraint files use it — `create_clock -period "100 MHz" -name {refclk} {pcie_refclk}`.
/// Rejecting it fails the whole file, taking every other constraint with it. Found by running
/// this reader over a third-party constraint corpus; none of our own 154 files use the form,
/// because they are all ASIC flows.
fn parse_period(v: &str) -> Option<f64> {
    if let Some(t) = parse_time(v) {
        return Some(t);
    }
    // `<number> <unit>` with the unit naming a frequency; the space is optional.
    let t = v.trim().trim_matches('"').trim();
    let split = t
        .find(|c: char| !(c.is_ascii_digit() || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-'))?;
    let (num, unit) = t.split_at(split);
    let hz = match unit.trim().to_ascii_lowercase().as_str() {
        "hz" => 1.0,
        "khz" => 1.0e3,
        "mhz" => 1.0e6,
        "ghz" => 1.0e9,
        _ => return None,
    };
    let f: f64 = num.trim().parse().ok()?;
    (f > 0.0).then(|| 1.0e9 / (f * hz)) // -> nanoseconds
}

/// A time value, with or without an SI suffix, normalised to the file's time unit.
///
/// Quartus and several board vendors write `-period 20.000ns`; a bare `parse::<f64>()` rejects
/// that, and the whole constraint file went with it. Six of the 154 files in this organisation
/// are of exactly that form. A bare number keeps its existing meaning — the file's own time
/// unit — so this only ever adds cases that used to fail.
fn parse_time(v: &str) -> Option<f64> {
    let t = v.trim();
    if let Ok(x) = t.parse::<f64>() {
        return Some(x);
    }
    // Longest suffix first, so `ps` is not read as `s`.
    for (suf, scale) in [
        ("fs", 1e-6),
        ("ps", 1e-3),
        ("ns", 1.0),
        ("us", 1e3),
        ("ms", 1e6),
        ("s", 1e9),
    ] {
        if let Some(num) = t.strip_suffix(suf) {
            if let Ok(x) = num.trim().parse::<f64>() {
                return Some(x * scale);
            }
        }
    }
    None
}

/// Apply `set var value` substitution to a logical line (`$var`, `${var}`, `$::env(NAME)`).
///
/// **`$::env(...)` is how a real flow parameterises constraints.** OpenLane writes
/// `create_clock -name clk -period $::env(CLOCK_PERIOD) [get_ports clk]`, and 8 of the 154
/// constraint files in this organisation do exactly that. Resolving only `$var` left the period
/// empty, which failed the whole file — every other constraint in it lost along with the clock.
/// Tcl reads `::env` from the process environment and so do we, falling back to `set` for a
/// variable of that name.
fn subst_vars(line: &str, vars: &HashMap<String, String>) -> String {
    if !line.contains('$') {
        return line.to_string();
    }
    let mut out = String::new();
    let cs: Vec<char> = line.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        if cs[i] == '$' {
            // `$::env(NAME)` / `$env(NAME)` — a parenthesised array read, not a bare name.
            let rest: String = cs[i..].iter().collect();
            let env_pfx = ["$::env(", "$env("]
                .iter()
                .find(|p| rest.starts_with(**p))
                .copied();
            if let Some(pfx) = env_pfx {
                if let Some(close) = rest.find(')') {
                    let name = &rest[pfx.len()..close];
                    match vars.get(name).cloned().or_else(|| std::env::var(name).ok()) {
                        Some(v) => out.push_str(&v),
                        // UNRESOLVED KEEPS ITS TEXT. Substituting nothing deletes the token,
                        // and the argument after it slides into its place: `-period $undefined
                        // [get_ports clk]` then reads the port expression as the period, and
                        // `-period $undefined 5` would silently yield a period of 5. Leaving
                        // `$::env(NAME)` in place makes it fail to parse as a time, and the
                        // error names the variable.
                        None => out.push_str(&rest[..close + 1]),
                    }
                    i += close + 1;
                    continue;
                }
            }
            let braced = i + 1 < cs.len() && cs[i + 1] == '{';
            let mut j = if braced { i + 2 } else { i + 1 };
            let start = j;
            while j < cs.len() {
                let c = cs[j];
                if braced {
                    if c == '}' {
                        break;
                    }
                } else if !(c.is_alphanumeric() || c == '_') {
                    break;
                }
                j += 1;
            }
            let name: String = cs[start..j].iter().collect();
            match vars.get(&name) {
                Some(val) => out.push_str(val),
                // See the note above: an unresolved variable keeps its text rather than
                // vanishing and letting the next argument take its place.
                None => {
                    let end = if braced { j + 1 } else { j };
                    out.extend(cs[i..end.min(cs.len())].iter());
                }
            }
            i = if braced { j + 1 } else { j };
        } else {
            out.push(cs[i]);
            i += 1;
        }
    }
    out
}

/// Pull a flag's value: `-flag value`. Returns the token after the flag.
fn flag_val<'a>(toks: &'a [String], flag: &str) -> Option<&'a String> {
    toks.iter()
        .position(|t| t == flag)
        .and_then(|p| toks.get(p + 1))
}

fn has_flag(toks: &[String], flag: &str) -> bool {
    toks.iter().any(|t| t == flag)
}

/// The trailing positional object token (last token that is not a flag or a
/// flag's value), e.g. the `{obj}` of `create_clock ... {obj}`.
fn trailing_obj(toks: &[String], valued_flags: &[&str]) -> Option<String> {
    let mut skip = false;
    let mut last = None;
    for (k, t) in toks.iter().enumerate().skip(1) {
        if skip {
            skip = false;
            continue;
        }
        if t.starts_with('-') {
            if valued_flags.contains(&t.as_str()) {
                skip = true;
            }
            continue;
        }
        let _ = k;
        last = Some(t.clone());
    }
    last
}

// ---- unit scaling --------------------------------------------------------

/// Parse a `set_units` magnitude like `1ns`, `10ps`, `1pF`, `1ff` into a scale
/// to the engine's base (ns for time, pF for cap). Returns multiplier.
fn unit_scale(spec: &str, time: bool) -> Option<f64> {
    let s = spec.trim().to_lowercase();
    let (num, unit): (String, String) = s
        .chars()
        .partition(|c| c.is_ascii_digit() || *c == '.' || *c == 'e' || *c == '-' || *c == '+');
    let mag: f64 = if num.is_empty() {
        1.0
    } else {
        num.parse().ok()?
    };
    let base = if time {
        match unit.as_str() {
            "s" => 1e9,
            "ms" => 1e6,
            "us" => 1e3,
            "ns" => 1.0,
            "ps" => 1e-3,
            "fs" => 1e-6,
            _ => return None,
        }
    } else {
        match unit.as_str() {
            "f" => 1e6,
            "pf" => 1.0,
            "ff" => 1e-3,
            "nf" => 1e3,
            "uf" => 1e6,
            _ => return None,
        }
    };
    Some(mag * base)
}

// ---- parse ---------------------------------------------------------------

impl Sdc {
    pub fn parse(text: &str) -> Result<Sdc, SdcError> {
        let mut sdc = Sdc::default();
        let mut vars: HashMap<String, String> = HashMap::new();
        let mut t_scale = 1.0; // -> ns
        let mut c_scale = 1.0; // -> pF
                               // (name, source, divide, multiply) for generated clocks, resolved last.
        let mut gen: Vec<(String, String, f64, f64)> = Vec::new();

        for line in logical_lines(text) {
            let line = subst_vars(&line, &vars);
            let toks = tokenize(&line);
            if toks.is_empty() {
                continue;
            }
            match toks[0].as_str() {
                "set" => {
                    if toks.len() >= 3 {
                        vars.insert(toks[1].clone(), toks[2].clone());
                    }
                }
                // SDC accepts the singular and plural spellings interchangeably, and tools
                // emit both. Matching one silently ignores the other — and for
                // `set_clock_group` that means losing an asynchronous-clock declaration, so
                // paths between clocks that never relate get checked and report false
                // violations. Found by diffing our command inventory against SDC 2.1.
                "set_units" | "set_unit" => {
                    if let Some(v) = flag_val(&toks, "-time") {
                        if let Some(s) = unit_scale(v, true) {
                            t_scale = s;
                        }
                    }
                    if let Some(v) = flag_val(&toks, "-capacitance") {
                        if let Some(s) = unit_scale(v, false) {
                            c_scale = s;
                        }
                    }
                }
                "create_clock" => {
                    let raw = flag_val(&toks, "-period");
                    let period: f64 = raw
                        .and_then(|v| parse_period(v))
                        .ok_or_else(|| {
                            // Name what was actually there. "create_clock without -period" is
                            // true and useless: the value is nearly always present and simply
                            // unresolvable — an `${VAR}` the flow supplies, or a period written
                            // with a unit suffix. Saying which turns a dead end into a fix.
                            SdcError(match raw {
                                None => "create_clock without -period".to_string(),
                                Some(v) if v.is_empty() => format!(
                                    "create_clock -period is empty on `{}` — an unresolved \
                                     variable (set it, or export it for $::env)",
                                    line.trim()
                                ),
                                Some(v) => {
                                    format!("create_clock -period `{v}` is neither a time nor a frequency")
                                }
                            })
                        })?;
                    let obj = trailing_obj(&toks, &["-name", "-period", "-waveform", "-comment"]);
                    let sources: Vec<String> = obj.as_deref().map(resolve_objs).unwrap_or_default();
                    let source = sources.first().cloned().unwrap_or_default();
                    let name = flag_val(&toks, "-name").cloned().unwrap_or_else(|| {
                        if source.is_empty() {
                            "clk".into()
                        } else {
                            source.clone()
                        }
                    });
                    // A VIRTUAL CLOCK HAS NO SOURCE, and must not be given one. `create_clock
                    // -name clk -period 1.0` with no object list is a reference clock for I/O
                    // budgeting that exists nowhere in the design. Defaulting its source to its
                    // own name makes it indistinguishable from a clock defined ON a port called
                    // `clk` — so if such a port exists, the virtual clock silently becomes a
                    // real one and launches and captures through it. An empty source is what
                    // "virtual" means; `is_virtual()` says so at the call site.
                    let src = source;
                    // An edge that is not a time (an unresolved variable) leaves the list EMPTY
                    // rather than failing the file: a consumer modelling two edges refuses it.
                    let waveform = flag_val(&toks, "-waveform").map(|w| {
                        w.split_whitespace().map(|e| parse_time(e).map(|t| t * t_scale)).collect::<Option<Vec<f64>>>().unwrap_or_default()
                    });
                    // `-add` keeps the clocks already on these sources; without it a new clock
                    // REPLACES them there (each loses these sources, and one left with none is
                    // removed with every reference to it). A clock of the same name is redefined
                    // in place: it keeps its position and its references, and takes the new
                    // sources, period and waveform.
                    let add = has_flag(&toks, "-add");
                    if add && flag_val(&toks, "-name").is_none() {
                        return Err(SdcError(format!("create_clock -add requires -name: `{}`", line.trim())));
                    }
                    if !add {
                        sdc.delete_pin_clocks(&name, &sources);
                    }
                    let clock = SdcClock { name, source: src, sources, period: period * t_scale, waveform };
                    match sdc.clocks.iter().position(|c| c.name == clock.name) {
                        Some(i) => sdc.superseded_clocks.push(std::mem::replace(&mut sdc.clocks[i], clock)),
                        None => sdc.clocks.push(clock),
                    }
                }
                "create_generated_clock" => {
                    let obj = trailing_obj(
                        &toks,
                        &[
                            "-name",
                            "-source",
                            "-divide_by",
                            "-multiply_by",
                            "-edges",
                            "-comment",
                            "-master_clock",
                        ],
                    );
                    let target = obj
                        .as_deref()
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
                    let name = flag_val(&toks, "-name")
                        .cloned()
                        .unwrap_or_else(|| target.clone());
                    let source = flag_val(&toks, "-source")
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
                    let div: f64 = flag_val(&toks, "-divide_by")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1.0);
                    let mul: f64 = flag_val(&toks, "-multiply_by")
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1.0);
                    let tgt = if target.is_empty() {
                        name.clone()
                    } else {
                        target
                    };
                    gen.push((name, tgt, div.max(1.0), mul.max(1.0)));
                    let _ = source;
                }
                "set_input_delay" | "set_output_delay" => {
                    // `-max` / `-min` take NO value: `delay_value port_pin_list` are the two
                    // positionals. Reading `-max` as valued swallowed the port list of
                    // `set_input_delay 1 -clock clk -max [get_ports a]`.
                    let (with, flags, pos) = split_args(&toks, &["-clock", "-reference_pin", "-comment"]);
                    let read: Option<f64> = pos.first().and_then(|v| v.parse().ok());
                    let val = read.unwrap_or(0.0);
                    let named = pos.get(1).map(|o| Objects::of(o)).unwrap_or(Objects { accessor: String::new(), names: Vec::new() });
                    let accessor = named.accessor;
                    let objs = named.names;
                    let default =
                        objs.is_empty() || objs.iter().any(|o| o == "*INPUTS*" || o == "*OUTPUTS*");
                    let ports: Vec<String> =
                        objs.into_iter().filter(|o| !o.starts_with('*')).collect();
                    let valued = |f: &str| with.iter().find(|(k, _)| *k == f).and_then(|(_, v)| resolve_objs(v).into_iter().next());
                    let d = IoDelay {
                        value: val * t_scale,
                        value_read: read.is_some(),
                        default,
                        ports,
                        clock: valued("-clock"),
                        flags,
                        reference_pin: valued("-reference_pin"),
                        accessor,
                    };
                    if toks[0] == "set_input_delay" {
                        sdc.input_delays.push(d);
                    } else {
                        sdc.output_delays.push(d);
                    }
                }
                "set_clock_uncertainty" => {
                    let val: f64 = toks
                        .iter()
                        .skip(1)
                        .find(|t| {
                            !t.starts_with('-') && !t.starts_with('[') && t.parse::<f64>().is_ok()
                        })
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0);
                    let v = val * t_scale;
                    let setup = has_flag(&toks, "-setup");
                    let hold = has_flag(&toks, "-hold");
                    if setup || !hold {
                        sdc.setup_uncertainty = sdc.setup_uncertainty.max(v);
                    }
                    if hold || !setup {
                        sdc.hold_uncertainty = sdc.hold_uncertainty.max(v);
                    }
                }
                "set_clock_latency" => {
                    sdc.clock_latencies.extend(env_cmd(&toks));
                    if let Some(v) = toks.iter().skip(1).find_map(|t| {
                        if t.starts_with('-') || t.starts_with('[') {
                            None
                        } else {
                            t.parse::<f64>().ok()
                        }
                    }) {
                        sdc.clock_latency = sdc.clock_latency.max(v * t_scale);
                    }
                }
                "set_input_transition" => {
                    sdc.env.extend(env_cmd(&toks));
                    if let Some(v) = toks.get(1).and_then(|t| t.parse::<f64>().ok()) {
                        sdc.input_transition = Some(v * t_scale);
                    }
                }
                "set_max_fanout" => {
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_fanout = Some(v),
                        _ => sdc.max_fanout_on_objects = true,
                    }
                    sdc.ignored.push("set_max_fanout".to_string());
                }
                "set_max_capacitance" => {
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_capacitance = Some(v),
                        (_, v) => {
                            sdc.max_capacitance_on_objects = true;
                            if let Some(o) = toks.get(2) {
                                sdc.max_capacitance_objects.push((v.unwrap_or(f64::NAN), Objects::of(o)));
                            }
                        }
                    }
                    sdc.ignored.push("set_max_capacitance".to_string());
                }
                "set_propagated_clock" => {
                    if let Some(o) = toks.get(1) {
                        sdc.propagated.push(Objects::of(o));
                    }
                    sdc.ignored.push("set_propagated_clock".to_string());
                }
                "set_max_delay" | "set_min_delay" => {
                    let valued = ["-from", "-to", "-through", "-rise_from", "-rise_to", "-fall_from", "-fall_to", "-rise_through", "-fall_through", "-comment"];
                    let (with, flags, pos) = split_args(&toks, &valued);
                    let value = pos.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(f64::NAN);
                    sdc.path_delays.push(PathDelayCmd {
                        max: toks[0] == "set_max_delay",
                        value: value * t_scale,
                        objects: with.iter().filter(|(k, _)| *k != "-comment").map(|(k, v)| (k.to_string(), Objects::of(v))).collect(),
                        flags,
                    });
                    sdc.ignored.push(toks[0].clone());
                }
                "set_max_transition" => {
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_transition = Some(v * t_scale),
                        _ => sdc.max_transition_on_objects = true,
                    }
                    sdc.ignored.push("set_max_transition".to_string());
                }
                "set_driving_cell" => {
                    sdc.driving_cells.push(driving_cell(&toks));
                    sdc.ignored.push("set_driving_cell".to_string());
                }
                "set_load" => {
                    sdc.env.extend(env_cmd(&toks));
                    let v = toks.iter().skip(1).find_map(|t| {
                        if t.starts_with('-') {
                            None
                        } else {
                            t.parse::<f64>().ok()
                        }
                    });
                    if let Some(v) = v {
                        sdc.load = Some(v * c_scale);
                    }
                }
                "set_timing_derate" => {
                    if let Some(v) = flag_val(&toks, "-late").and_then(|v| v.parse().ok()) {
                        sdc.late_derate = Some(v);
                    }
                    if let Some(v) = flag_val(&toks, "-early").and_then(|v| v.parse().ok()) {
                        sdc.early_derate = Some(v);
                    }
                }
                "set_clock_groups" | "set_clock_group" => {
                    // -asynchronous / -exclusive: clocks in different -group blocks are
                    // unrelated. Collect each -group's resolved clock names.
                    let mut groups: Vec<Vec<String>> = Vec::new();
                    let mut i = 0;
                    while i < toks.len() {
                        if toks[i] == "-group" {
                            if let Some(v) = toks.get(i + 1) {
                                let names: Vec<String> = resolve_objs(v)
                                    .into_iter()
                                    .filter(|o| !o.starts_with('*'))
                                    .collect();
                                if !names.is_empty() {
                                    groups.push(names);
                                }
                                i += 2;
                                continue;
                            }
                        }
                        i += 1;
                    }
                    // Kept even when there is only ONE group. A single group cuts no timing
                    // path — that needs two clocks in *different* groups — which is why this
                    // was once dropped as uninteresting. But it still states something the SDC
                    // has no other way to say: the clocks inside it are related to each other.
                    // A CDC check reads exactly that, and dropping it made a design whose SDC
                    // declared its clocks related report every one of those pairs as a
                    // crossing. Store what the file said; leave the reading to each consumer.
                    sdc.async_groups.extend(groups);
                }
                "set_false_path" => {
                    let (from, to) = from_to(&toks);
                    sdc.exception_objects.push(exception_sides(&toks));
                    sdc.exceptions.push(Exception {
                        kind: ExcKind::FalsePath,
                        from,
                        to,
                    });
                }
                "set_multicycle_path" => {
                    let n: u32 = toks
                        .get(1)
                        .filter(|t| !t.starts_with('-'))
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1);
                    let (from, to) = from_to(&toks);
                    sdc.exception_objects.push(exception_sides(&toks));
                    sdc.exceptions.push(Exception {
                        kind: ExcKind::Multicycle(n),
                        from,
                        to,
                    });
                }
                other => sdc.ignored.push(other.to_string()),
            }
        }

        // resolve generated clocks against their master period.
        for (name, target, div, mul) in gen {
            let master = sdc.clocks.first().map(|c| c.period).unwrap_or(0.0);
            let period = master * div / mul;
            sdc.clocks.push(SdcClock {
                name,
                sources: if target.is_empty() { Vec::new() } else { vec![target.clone()] },
                source: target,
                period,
                waveform: None,
            });
        }
        Ok(sdc)
    }

    pub fn load(path: &str) -> Result<Sdc, SdcError> {
        let text = std::fs::read_to_string(path).map_err(|e| SdcError(format!("{path}: {e}")))?;
        Sdc::parse(&text)
    }
}

/// Extract `-from`/`-to` object names (first object of each), `*` if absent.
/// A pin object (`reg/Q`) is reduced to its instance (`reg`) so it matches the
/// engine's instance-level exception matching; a port keeps its name.
fn from_to(toks: &[String]) -> (Vec<String>, Vec<String>) {
    let [from, to] = exception_sides(toks);
    (reduce_exception_objects(&from), reduce_exception_objects(&to))
}

/// An exception's objects per side: every `-from` / `-rise_from` / `-fall_from`, then every `-to`
/// / `-rise_to` / `-fall_to`, accessor kept.
fn exception_sides(toks: &[String]) -> [Vec<Objects>; 2] {
    let pick = |flags: &[&str]| -> Vec<Objects> { flags.iter().filter_map(|f| flag_val(toks, f)).map(|v| Objects::of(v)).collect() };
    [pick(&["-from", "-rise_from", "-fall_from"]), pick(&["-to", "-rise_to", "-fall_to"])]
}

/// One side of [`Exception`] from its objects. ALL objects each flag names, not the first: a
/// `-from [list a b c]` that cut only `a` left `b` and `c` timed against the SDC's stated intent.
fn reduce_exception_objects(objs: &[Objects]) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for name in objs.iter().flat_map(|o| o.names.iter()) {
        if name.starts_with('*') {
            continue; // an `all_inputs`-style collection, not a named object
        }
        // a pin object (`reg/Q`) reduces to its instance, matching the engine's
        // instance-level comparison
        out.push(match name.rsplit_once('/') {
            Some((inst, _pin)) => inst.to_string(),
            None => name.clone(),
        });
    }
    out.sort();
    out.dedup();
    if out.is_empty() {
        out.push("*".to_string()); // the flag was absent or named only a collection
    }
    out
}

#[cfg(test)]
mod async_group_tests {
    use super::*;
    /// Rule (create_clock): one clock on several objects — every object is a source, in the
    /// order listed; the written form `write_sdc` emits (a `[list …]` over continuation lines)
    /// reads the same as the brace list a script writes.
    #[test]
    fn a_clock_on_several_ports_keeps_every_source() {
        let written = "create_clock -name clk1 -period 1.0000 \\\n    [list [get_ports {clk1}]\\\n          [get_ports {clk2}]\\\n          [get_ports {clk3}]]\n";
        let s = Sdc::parse(written).unwrap();
        assert_eq!(s.clocks.len(), 1);
        assert_eq!(s.clocks[0].sources, ["clk1", "clk2", "clk3"]);
        assert_eq!(s.clocks[0].source, "clk1");
        let scripted = Sdc::parse("create_clock -period 1 [get_ports {clk1 clk2 clk3}]\n").unwrap();
        assert_eq!(scripted.clocks[0].sources, ["clk1", "clk2", "clk3"]);
        let virt = Sdc::parse("create_clock -name v -period 1\n").unwrap();
        assert!(virt.clocks[0].sources.is_empty() && virt.clocks[0].is_virtual());
    }

    #[test]
    fn clock_groups_asynchronous_parses_groups() {
        let s = "create_clock -name a -period 10 [get_ports ca]\n\
                 create_clock -name b -period 3 [get_ports cb]\n\
                 set_clock_groups -asynchronous -group [get_clocks a] -group [get_clocks b]\n";
        let sdc = Sdc::parse(s).unwrap();
        assert_eq!(sdc.async_groups.len(), 2, "two groups");
        assert!(sdc.async_groups.iter().any(|g| g == &vec!["a".to_string()]));
        assert!(sdc.async_groups.iter().any(|g| g == &vec!["b".to_string()]));
    }

    #[test]
    fn a_single_clock_group_is_kept_because_it_states_relatedness() {
        // One group cuts no timing path — a cut needs two clocks in *different* groups — so
        // this was once discarded as having no effect. It does have one: it says the clocks
        // inside it are related to each other, which is the only way an SDC can say so, and a
        // CDC check reads it to avoid reporting those pairs as crossings.
        let s = "create_clock -name a -period 10 [get_ports ca]\n\
                 create_clock -name b -period 10 [get_ports cb]\n\
                 set_clock_groups -asynchronous -group {a b}\n";
        let sdc = Sdc::parse(s).unwrap();
        assert_eq!(sdc.async_groups, vec![vec!["a".to_string(), "b".to_string()]]);
    }
}

#[cfg(test)]
mod list_obj_tests {
    use super::*;

    /// `[list ...]` around object collections is ordinary in sign-off SDCs, and resolving it
    /// to the literal token `list` makes the exception match nothing — so it silently does not
    /// apply and the paths stay timed. Found on a real pad-wrapper SDC.
    #[test]
    fn a_tcl_list_of_object_collections_resolves_to_its_members() {
        assert_eq!(
            resolve_objs("[list [get_ports {a}] [get_ports {b}] [get_ports {c}]]"),
            vec!["a", "b", "c"]
        );
    }

    #[test]
    fn a_false_path_from_a_list_names_a_real_port_not_the_word_list() {
        let sdc = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_false_path -from [list [get_ports {mask_rev[0]}] [get_ports {mask_rev[1]}]] \
             -to [get_ports {out}]\n",
        )
        .unwrap();
        assert_eq!(sdc.exceptions.len(), 1);
        assert_eq!(
            sdc.exceptions[0].from,
            vec!["mask_rev[0]", "mask_rev[1]"],
            "every member of the list, not the first and not the Tcl command name"
        );
        assert_eq!(sdc.exceptions[0].to, vec!["out"]);
        assert!(
            sdc.exceptions[0].covers("mask_rev[1]", "out"),
            "the second member is cut too"
        );
    }

    #[test]
    fn a_plain_get_ports_is_unaffected() {
        assert_eq!(resolve_objs("[get_ports {clk}]"), vec!["clk"]);
        assert_eq!(resolve_objs("[all_inputs]"), vec!["*INPUTS*"]);
    }
}

#[cfg(test)]
mod max_transition_tests {
    use super::*;

    // Rule: a set_driving_cell keeps its cell, pins, transitions and ports, and stays in `ignored`.
    #[test]
    fn driving_cell_is_kept_as_written() {
        let s = Sdc::parse("set_driving_cell -lib_cell BUF_X1 -pin {Z} -input_transition_rise 0.0100 -input_transition_fall 0.0200 [get_ports {in1}]\n").unwrap();
        let d = &s.driving_cells[0];
        assert_eq!((d.lib_cell.as_deref(), d.pin.as_deref(), d.from_pin.as_deref(), d.input_transition), (Some("BUF_X1"), Some("Z"), None, [0.01, 0.02]));
        assert_eq!(d.objects, vec!["in1".to_string()]);
        assert!(d.flags.is_empty());
        assert!(s.ignored_affecting_timing().contains(&"set_driving_cell"));
    }

    // Rule: the design-wide set_max_fanout keeps its value (unitless) and stays in `ignored`;
    // any other target is flagged.
    #[test]
    fn max_fanout_on_the_design_keeps_its_value() {
        let s = Sdc::parse("set_max_fanout 5.0000 [current_design]\n").unwrap();
        assert_eq!((s.max_fanout, s.max_fanout_on_objects), (Some(5.0), false));
        assert!(s.ignored_affecting_timing().contains(&"set_max_fanout"));
        let s = Sdc::parse("set_max_fanout 3 [get_ports a]\n").unwrap();
        assert!(s.max_fanout_on_objects);
    }

    // Rule: each set_load / set_input_transition is kept as written — value unscaled by
    // set_units, flags, accessor and objects.
    #[test]
    fn environment_commands_keep_value_flags_and_objects() {
        let s = Sdc::parse("set_units -capacitance 1pF\nset_load -pin_load 2000.0 [get_ports {out}]\nset_load  0.0000 [get_nets {out1 out2}]\nset_input_transition -rise 10.0 -clock clk [get_ports {in1}]\n").unwrap();
        assert_eq!(s.env.len(), 3);
        assert_eq!((s.env[0].cmd.as_str(), s.env[0].value, s.env[0].flags.clone(), s.env[0].accessor.as_str(), s.env[0].objects.clone()), ("set_load", 2000.0, vec!["-pin_load".to_string()], "get_ports", vec!["out".to_string()]));
        assert_eq!((s.env[1].accessor.as_str(), s.env[1].objects.clone()), ("get_nets", vec!["out1".to_string(), "out2".to_string()]));
        assert_eq!((s.env[2].value, s.env[2].clock.as_deref(), s.env[2].flags.clone()), (10.0, Some("clk"), vec!["-rise".to_string()]));
    }

    // Rule: an `all_inputs` / `all_outputs` collection keeps its sentinel AND the flags written
    // inside its brackets — the reference's `all_inputs -no_clocks` drops clock-source ports
    // (`Sdc::allInputs(no_clks)`), which the `*INPUTS*` sentinel alone cannot say.
    #[test]
    fn environment_commands_keep_all_ports_collections_and_their_flags() {
        let s = Sdc::parse("set_load 0.0334 [all_outputs]\nset_input_transition 0.1 [all_inputs -no_clocks]\n").unwrap();
        assert_eq!((s.env[0].accessor.as_str(), s.env[0].objects.clone(), s.env[0].accessor_flags.clone()), ("all_outputs", vec!["*OUTPUTS*".to_string()], Vec::<String>::new()));
        assert_eq!((s.env[1].accessor.as_str(), s.env[1].objects.clone(), s.env[1].accessor_flags.clone()), ("all_inputs", vec!["*INPUTS*".to_string()], vec!["-no_clocks".to_string()]));
    }

    // Rule: the design-wide form keeps its value; every form stays in `ignored` (timing-
    // affecting for a consumer that does not model it); any other target is flagged.
    #[test]
    fn set_max_transition_keeps_the_design_value_and_flags_other_targets() {
        let s = Sdc::parse("current_design top\nset_max_transition 0.0800 [current_design]\n").unwrap();
        assert_eq!((s.max_transition, s.max_transition_on_objects), (Some(0.08), false));
        assert_eq!(s.ignored_affecting_timing(), vec!["set_max_transition"]);
        let s = Sdc::parse("set_max_transition 0.1 [get_ports a]\n").unwrap();
        assert_eq!((s.max_transition, s.max_transition_on_objects), (None, true));
        assert_eq!(s.ignored_affecting_timing(), vec!["set_max_transition"]);
    }
}

#[cfg(test)]
mod lossless_tests {
    use super::*;

    // Rule (create_clock): `-waveform` keeps every edge as written; none without the flag;
    // a generated clock has none.
    #[test]
    fn a_clock_keeps_its_waveform() {
        let s = Sdc::parse("create_clock -name clk -period 1.0000 -waveform {0.2000 0.7000} [get_ports {clk}]\ncreate_clock -name v -period 2\n").unwrap();
        assert_eq!(s.clocks[0].waveform, Some(vec![0.2, 0.7]));
        assert_eq!(s.clocks[0].sources, vec!["clk".to_string()]);
        assert_eq!(s.clocks[1].waveform, None);
        let bad = Sdc::parse("create_clock -name clk -period 1 -waveform {0 $undefined} [get_ports clk]\n").unwrap();
        assert_eq!(bad.clocks[0].waveform, Some(vec![]));
    }

    // Rule (set_input_delay): `-max` / `-min` take no value — the value and the port list are
    // the positionals, wherever the flags sit; the flags, `-clock` and `-reference_pin` are kept.
    #[test]
    fn an_io_delay_keeps_its_flags_clock_and_ports() {
        let s = Sdc::parse("set_input_delay 0.1000 -clock [get_clocks {clk}] -max -add_delay [get_ports {in1 in2}]\nset_input_delay -min -0.2 -clock clk2 -rise [get_ports {in1}]\nset_output_delay 0.3 -clock clk -reference_pin [get_pins {r1/CK}] [get_ports {out}]\n").unwrap();
        let d = &s.input_delays[0];
        assert_eq!((d.value, d.clock.as_deref(), d.flags.clone(), d.ports.clone(), d.default), (0.1, Some("clk"), vec!["-max".to_string(), "-add_delay".to_string()], vec!["in1".to_string(), "in2".to_string()], false));
        let d = &s.input_delays[1];
        assert_eq!((d.value, d.clock.as_deref(), d.flags.clone(), d.ports.clone()), (-0.2, Some("clk2"), vec!["-min".to_string(), "-rise".to_string()], vec!["in1".to_string()]));
        let d = &s.output_delays[0];
        assert_eq!((d.reference_pin.as_deref(), d.ports.clone(), d.accessor.as_str()), (Some("r1/CK"), vec!["out".to_string()], "get_ports"));
        let d = Sdc::parse("set_input_delay 1 -clock clk\nset_input_delay $undefined -clock clk [get_ports a]\n").unwrap();
        assert!(d.input_delays[0].default && d.input_delays[0].ports.is_empty() && d.input_delays[0].value_read);
        assert!(!d.input_delays[1].value_read && d.input_delays[1].value == 0.0);
    }

    // Rule: the design-wide set_max_capacitance keeps its value AS WRITTEN (set_units only
    // checks) and stays in `ignored`; any other target is flagged.
    #[test]
    fn max_capacitance_on_the_design_keeps_its_value() {
        let s = Sdc::parse("set_units -capacitance 1pF\nset_max_capacitance 46.0800 [current_design]\n").unwrap();
        assert_eq!((s.max_capacitance, s.max_capacitance_on_objects), (Some(46.08), false));
        assert!(s.ignored_affecting_timing().contains(&"set_max_capacitance"));
        let s = Sdc::parse("set_max_capacitance 3 [get_ports a]\nset_max_capacitance 0.5 [get_pins {u1/A u2/A}]\n").unwrap();
        assert_eq!((s.max_capacitance, s.max_capacitance_on_objects), (None, true));
        let objs: Vec<(f64, &str, Vec<&str>)> = s.max_capacitance_objects.iter().map(|(v, o)| (*v, o.accessor.as_str(), o.names.iter().map(String::as_str).collect())).collect();
        assert_eq!(objs, [(3.0, "get_ports", vec!["a"]), (0.5, "get_pins", vec!["u1/A", "u2/A"])]);
    }

    // Rule: set_propagated_clock, set_clock_latency and set_max_delay / set_min_delay are kept
    // per command (the first and last still in `ignored`), objects with their accessor.
    #[test]
    fn propagated_latency_and_path_delays_are_kept_as_written() {
        let s = Sdc::parse("set_propagated_clock [get_clocks {clk}]\nset_clock_latency -source 0.2 [get_clocks {clk}]\nset_max_delay\\
    -ignore_clock_latency\\
    -from [get_pins {r1/CK}]\\
    -to [get_clocks {clk}] 0.5000\nset_min_delay -from [get_pins {a/Z}] $undefined\n").unwrap();
        assert_eq!(s.propagated, vec![Objects { accessor: "get_clocks".into(), names: vec!["clk".into()] }]);
        assert_eq!((s.clock_latencies.len(), s.clock_latencies[0].flags.clone(), s.clock_latency), (1, vec!["-source".to_string()], 0.2));
        let p = &s.path_delays[0];
        assert_eq!((p.max, p.value, p.flags.clone()), (true, 0.5, vec!["-ignore_clock_latency".to_string()]));
        assert_eq!(p.objects, vec![("-from".to_string(), Objects { accessor: "get_pins".into(), names: vec!["r1/CK".into()] }), ("-to".to_string(), Objects { accessor: "get_clocks".into(), names: vec!["clk".into()] })]);
        assert!(!s.path_delays[1].max && s.path_delays[1].value.is_nan());
        assert!(s.ignored.iter().any(|c| c == "set_propagated_clock") && s.ignored.iter().filter(|c| c.ends_with("_delay")).count() == 2);
    }
}

#[cfg(test)]
mod clock_replace_tests {
    use super::*;

    fn names(s: &Sdc) -> Vec<&str> {
        s.clocks.iter().map(|c| c.name.as_str()).collect()
    }

    // Rule (create_clock): without `-add`, a clock on a source already clocked replaces the clock
    // there — the old one, left with no source, is removed with its I/O delays. With `-add` both stay.
    #[test]
    fn without_add_a_clock_replaces_the_one_on_its_port() {
        let s = Sdc::parse("create_clock -name a -period 1 [get_ports clk]\nset_input_delay 0.1 -clock a [get_ports in]\ncreate_clock -name b -period 2 [get_ports clk]\n").unwrap();
        assert_eq!(names(&s), ["b"]);
        assert!(s.input_delays.is_empty());
        assert_eq!(s.superseded_clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["a"]);
        let s = Sdc::parse("create_clock -name a -period 1 [get_ports clk]\nset_input_delay 0.1 -clock a [get_ports in]\ncreate_clock -name b -add -period 2 [get_ports clk]\n").unwrap();
        assert_eq!(names(&s), ["a", "b"]);
        assert_eq!(s.input_delays.len(), 1);
        assert_eq!(s.clocks[1].sources, ["clk"]);
    }

    // Rule: a clock on several sources loses only the ones the new clock takes, and stays.
    #[test]
    fn a_clock_on_several_sources_loses_only_the_shared_one() {
        let s = Sdc::parse("create_clock -name a -period 1 [get_ports {c1 c2}]\ncreate_clock -name b -period 2 [get_ports c1]\n").unwrap();
        assert_eq!(names(&s), ["a", "b"]);
        assert_eq!((s.clocks[0].sources.clone(), s.clocks[0].source.as_str()), (vec!["c2".to_string()], "c2"));
    }

    // Rule: a clock of the same name is redefined IN PLACE — its position and its references kept,
    // its source, period and waveform replaced. A virtual clock replaces nothing.
    #[test]
    fn the_same_name_redefines_in_place() {
        let s = Sdc::parse("create_clock -name a -period 1 [get_ports c1]\ncreate_clock -name v -period 3\nset_input_delay 0.1 -clock a [get_ports in]\ncreate_clock -name a -period 5 -waveform {1 3} [get_ports c2]\ncreate_clock -name w -period 4\n").unwrap();
        assert_eq!(names(&s), ["a", "v", "w"]);
        assert_eq!((s.clocks[0].period, s.clocks[0].sources.clone(), s.clocks[0].waveform.clone()), (5.0, vec!["c2".to_string()], Some(vec![1.0, 3.0])));
        assert_eq!(s.input_delays.len(), 1);
        assert_eq!((s.superseded_clocks.len(), s.superseded_clocks[0].period), (1, 1.0));
    }

    // Rule: `-add` requires `-name`.
    #[test]
    fn add_without_a_name_is_an_error() {
        assert!(Sdc::parse("create_clock -add -period 1 [get_ports clk]\n").is_err());
    }

    // Rule: what named the removed clock goes with it — its propagated flag, its latencies, its
    // clock-group membership, and a path delay whose `-from` held only it; a path delay with
    // another `-from` object keeps that one. A timing exception naming it is flagged, not guessed.
    #[test]
    fn references_to_a_removed_clock_go_with_it() {
        let s = Sdc::parse(
            "create_clock -name a -period 1 [get_ports clk]\ncreate_clock -name x -period 1 [get_ports clk2]\n\
             set_propagated_clock [get_clocks {a x}]\nset_clock_latency 0.2 [get_clocks a]\n\
             set_clock_groups -asynchronous -group {a} -group {x}\n\
             set_max_delay 1 -from [get_clocks a] -to [get_pins r/D]\nset_max_delay 2 -from [get_clocks a] -from [get_pins r2/CK]\n\
             create_clock -name b -period 2 [get_ports clk]\n",
        )
        .unwrap();
        assert_eq!(names(&s), ["x", "b"]);
        assert_eq!(s.propagated.len(), 1);
        assert_eq!(s.propagated[0].names, ["x"]);
        assert!(s.clock_latencies.is_empty());
        assert_eq!(s.async_groups, vec![vec!["x".to_string()]]);
        assert_eq!(s.path_delays.len(), 1);
        assert_eq!(s.path_delays[0].value, 2.0);
    }

    // Rule (`deleteExceptionsReferencing`): a false or multicycle path whose `-from` clocks hold
    // the removed clock loses it, else its `-to` side does; a side left empty deletes it. Only a
    // `get_clocks` object names a clock: a port of the clock's name stays.
    #[test]
    fn exceptions_lose_a_removed_clock_by_side() {
        let s = Sdc::parse(
            "create_clock -name clk -period 1 [get_ports clk]\ncreate_clock -name x -period 1 [get_ports clk2]\n\
             set_false_path -from [get_clocks clk]\n\
             set_false_path -from [get_clocks {clk x}] -to [get_clocks clk]\n\
             set_multicycle_path 2 -from [get_ports clk] -to [get_clocks clk]\n\
             set_false_path -from [get_ports clk] -to [get_pins r/D]\n\
             create_clock -name b -period 2 [get_ports clk]\n",
        )
        .unwrap();
        let rows: Vec<(Vec<&str>, Vec<&str>)> = s.exceptions.iter().map(|e| (e.from.iter().map(String::as_str).collect(), e.to.iter().map(String::as_str).collect())).collect();
        // 1: from {clk} only -> deleted. 2: from loses clk, to untouched. 3: from is a PORT, so the
        // to side loses clk and, empty, deletes it. 4: names no clock.
        assert_eq!(rows, [(vec!["x"], vec!["clk"]), (vec!["clk"], vec!["r"])]);
        assert_eq!(s.exceptions.len(), s.exception_objects.len());
        assert!(!s.ignored.iter().any(|i| i.starts_with("create_clock")));
    }

    // The reference's own reading (2026-10-11, probe exception-replace.sdc read and written back):
    // b replaces clk; of the five exceptions it keeps the false path from x to qa, the false path
    // from the PORT clk to a_capture/D and the multicycle 3 to x — the false path from clk alone
    // and the multicycle from the port clk to the clock clk are gone.
    #[test]
    fn exceptions_after_a_replace_read_as_the_reference_reads_them() {
        let s = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\ncreate_clock -name x -period 7 [get_ports din]\n\
             set_false_path -from [get_clocks clk]\n\
             set_false_path -from [get_clocks {clk x}] -to [get_ports qa]\n\
             set_multicycle_path 2 -from [get_ports clk] -to [get_clocks clk]\n\
             set_false_path -from [get_ports clk] -to [get_pins a_capture/D]\n\
             set_multicycle_path 3 -to [get_clocks {clk x}]\n\
             create_clock -name b -period 20 [get_ports clk]\n",
        )
        .unwrap();
        let rows: Vec<(ExcKind, Vec<&str>, Vec<&str>)> = s.exceptions.iter().map(|e| (e.kind.clone(), e.from.iter().map(String::as_str).collect(), e.to.iter().map(String::as_str).collect())).collect();
        assert_eq!(
            rows,
            [(ExcKind::FalsePath, vec!["x"], vec!["qa"]), (ExcKind::FalsePath, vec!["clk"], vec!["a_capture"]), (ExcKind::Multicycle(3), vec!["*"], vec!["x"])]
        );
    }

    // The reference's own reading of a mixed sequence (2026-10-11: the file read, then written back
    // and its clocks listed): a takes {clk din}; b takes clk; c takes din, so a goes with its output
    // delay, its max delay and its group; v is redefined in place; d is added on clk; b, redefined
    // onto din without -add, takes din from c, which goes. Left: v (period 9, edges 1 and 4),
    // b (din, 50, propagated), d (clk, 40, propagated), in that order; one output delay (on v), no
    // path delay, one clock group holding v.
    #[test]
    fn a_mixed_sequence_reads_as_the_reference_reads_it() {
        let s = Sdc::parse("create_clock -name a -period 10 [get_ports {clk din}]\n\
             create_clock -name v -period 7\n\
             set_output_delay 1 -clock a [get_ports qa]\n\
             set_output_delay 2 -clock v [get_ports qb]\n\
             set_max_delay 3 -from [get_clocks a] -to [get_ports qb]\n\
             set_clock_groups -asynchronous -group {a} -group {v}\n\
             create_clock -name b -period 20 [get_ports clk]\n\
             create_clock -name c -period 30 [get_ports din]\n\
             create_clock -name v -period 9 -waveform {1 4}\n\
             create_clock -name d -add -period 40 [get_ports clk]\n\
             set_propagated_clock [get_clocks {b d}]\n\
             create_clock -name b -period 50 [get_ports din]\n").unwrap();
        assert_eq!(names(&s), ["v", "b", "d"]);
        let got: Vec<(f64, Vec<String>)> = s.clocks.iter().map(|c| (c.period, c.sources.clone())).collect();
        assert_eq!(got, [(9.0, vec![]), (50.0, vec!["din".to_string()]), (40.0, vec!["clk".to_string()])]);
        assert_eq!(s.clocks[0].waveform, Some(vec![1.0, 4.0]));
        assert_eq!(s.propagated.iter().flat_map(|o| o.names.clone()).collect::<Vec<_>>(), ["b", "d"]);
        assert_eq!(s.output_delays.iter().map(|d| d.clock.as_deref()).collect::<Vec<_>>(), [Some("v")]);
        assert!(s.path_delays.is_empty());
        assert_eq!(s.async_groups, vec![vec!["v".to_string()]]);
        assert!(!s.ignored.iter().any(|i| i.starts_with("create_clock")));
    }
}
