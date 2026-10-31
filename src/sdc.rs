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
//! - `set_clock_latency` — kept per command ([`Sdc::clock_latencies`], with its flags)
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
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum ExcKind {
    #[default]
    FalsePath,
    Multicycle(u32),
}

/// One timing exception: a kind plus its `-from`/`-to` endpoints (`*` = any).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
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
    /// A multicycle that applies to the HOLD check (`-hold`); setup otherwise.
    pub hold: bool,
    /// The multiplier counts the LAUNCH clock (`-start`) rather than the capture clock (`-end`).
    /// The reference's defaults: `-end` for setup, `-start` for hold.
    pub start: bool,
    /// Why an instance-level match cannot express this exception, when it cannot: then
    /// [`Exception::covers`] is false — it is NOT applied — and the reason is reported.
    ///
    /// ⛔ **Never "any" by default.** A `-through`-only false path, read without its `-through`,
    /// became `-from * -to *`; `-to [get_pins */SETN]`, reduced to its instance, became `*`; both
    /// cut EVERY path in the design. Not applying an exception is pessimistic and visible;
    /// applying it to everything is optimistic and silent.
    pub refused: Option<String>,
    /// Every object-valued flag as written, in order (`-from`, `-rise_through`, `-to`, …), with
    /// its objects — `-through` included, which [`Exception::from`] / [`Exception::to`] cannot
    /// carry. What a writer writes back; a timer reads `from` / `to` / `refused`.
    pub args: Vec<(String, Objects)>,
    /// Every flag that takes no value, as written (`-setup`, `-hold`, `-start`, `-end`, `-rise`,
    /// `-fall`, …).
    pub flags: Vec<String>,
}

/// Is `name` in an exception endpoint set? An empty set, or one holding `*`, matches anything;
/// a pattern with `*` / `?` matches as the reference's object query does (a glob).
fn endpoint_matches(set: &[String], name: &str) -> bool {
    set.is_empty() || set.iter().any(|o| o == "*" || o == name || (o.contains(['*', '?']) && glob(o, name)))
}

/// Does an SDC object name match `name` — itself, or as a `*` / `?` pattern the way the
/// reference's object queries match one (`get_ports in*`)? For a consumer applying a per-object
/// constraint: an exact-name lookup silently never applied `set_input_delay … [get_ports in*]`.
pub fn object_matches(pattern: &str, name: &str) -> bool {
    pattern == name || (pattern.contains(['*', '?']) && glob(pattern, name))
}

/// Tcl `string match` for `*` and `?` — what `get_ports in*` matches against.
fn glob(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut i, mut j, mut star, mut mark) = (0usize, 0usize, None, 0usize);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(st) = star {
            i = st + 1;
            mark += 1;
            j = mark;
        } else {
            return false;
        }
    }
    while i < p.len() && p[i] == '*' {
        i += 1;
    }
    i == p.len()
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
    /// Does it cover this launch→capture pair? Never, when it is [`Exception::refused`].
    pub fn covers(&self, launch: &str, capture: &str) -> bool {
        self.refused.is_none() && self.from_matches(launch) && self.to_matches(capture)
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
#[derive(Debug, Clone, Default)]
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
    /// `-add`: defined alongside the clocks already on its sources rather than replacing them.
    pub add: bool,
    /// For `create_generated_clock`, its definition as written — `period` and `waveform` are
    /// what it derives to. `None` for `create_clock`.
    pub generated: Option<GeneratedClock>,
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
    /// The delay, in the library's time units (as written — see `set_units`); 0 when it could
    /// not be read — see [`IoDelay::value_read`].
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
    /// The accessor's own flags (`-no_clocks` in `[all_inputs -no_clocks]`) — see [`Objects::flags`].
    pub accessor_flags: Vec<String>,
}

/// An object list as written: the accessor that named it (`get_clocks`, `get_pins`, … `list`, or
/// `""` for a bare name or brace list) and the names it resolves to.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Objects {
    pub accessor: String,
    pub names: Vec<String>,
    /// The accessor's flags that take no value, as written (`-hierarchical`, `-nocase`,
    /// `-no_clocks`, `-clock_pins`, …). They change what the names match, so writing the names
    /// back without them names different objects. Empty for a query evaluated against the design
    /// (`-filter`, `-of_objects`): its names are then the objects it resolved to.
    pub flags: Vec<String>,
    /// The accessor's keys with their values, as written (`-clock` in `[all_registers -clock c]`,
    /// `-hsc`). Empty for an evaluated query, as [`Objects::flags`].
    pub keys: Vec<(String, String)>,
    /// For `[list …]` / `[concat …]`: each element as written, with its own accessor; `names` is
    /// theirs concatenated. A list of ports and pins written back as one accessor would name only
    /// one kind.
    pub items: Vec<Objects>,
}

impl Objects {
    pub(crate) fn of(tok: &str) -> Objects {
        let accessor = tok.strip_prefix('[').and_then(|i| tokenize(i.trim_end_matches(']').trim()).into_iter().next()).unwrap_or_default();
        let parts = tok
            .strip_prefix('[')
            .and_then(|i| i.strip_suffix(']'))
            .map(|i| expand_abbreviations(tokenize(i.trim())))
            .unwrap_or_default();
        let (mut flags, mut keys_written, mut items) = (Vec::new(), Vec::new(), Vec::new());
        match accessor.as_str() {
            "list" | "concat" => items = parts.iter().skip(1).map(|p| Objects::of(p)).collect(),
            _ if parts.iter().any(|p| p == "-filter" || p == "-of_objects") => {}
            _ => {
                let keys = crate::sdc_flags::FLAGS.iter().find(|(n, _, _)| *n == accessor).map_or(&[][..], |(_, k, _)| *k);
                let mut k = 1;
                while k < parts.len() {
                    let p = &parts[k];
                    if keys.contains(&p.as_str()) {
                        if let Some(v) = parts.get(k + 1) {
                            keys_written.push((p.clone(), v.clone()));
                        }
                        k += 2;
                        continue;
                    }
                    if p.starts_with('-') && p.parse::<f64>().is_err() {
                        flags.push(p.clone());
                    }
                    k += 1;
                }
            }
        }
        Objects { accessor, names: resolve_objs(tok), flags, keys: keys_written, items }
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
    /// The objects as written (`None` for a command that named none) — see [`Objects::flags`].
    pub written: Option<Objects>,
}

/// One `set_clock_groups` command: its kind (`asynchronous`, `logically_exclusive`,
/// `physically_exclusive`), `-allow_paths`, and its `-group`s of clock names.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClockGroups {
    /// `-name`; `None` for an unnamed command, which the reference names `group<n>`.
    pub name: Option<String>,
    pub kind: String,
    pub allow_paths: bool,
    pub groups: Vec<Vec<String>>,
}

/// Are clocks `a` and `b` separated by any `set_clock_groups` — the reference's rule: one
/// command puts them in DIFFERENT groups, or a command of ONE group holds exactly one of them (a
/// single group separates its clocks from every other clock). `-asynchronous -allow_paths`
/// separates nothing.
///
/// ⛔ Merged into one list across commands, clocks named in two different commands were cut
/// from each other, though no command separated them; and a single group cut nothing.
pub fn clocks_unrelated(groups: &[ClockGroups], a: &str, b: &str) -> bool {
    if a == b {
        return false;
    }
    groups.iter().filter(|g| !g.allow_paths).any(|g| {
        let at = |c: &str| g.groups.iter().position(|grp| grp.iter().any(|n| n == c));
        match (at(a), at(b), g.groups.len()) {
            (Some(x), Some(y), _) => x != y,
            (Some(_), None, 1) | (None, Some(_), 1) => true,
            _ => false,
        }
    })
}

/// One `set_clock_uncertainty`: a value for `-setup`, `-hold` or both, on clocks or pins (objects)
/// or between two clocks (`-from` / `-to`, and their `-rise_` / `-fall_` forms).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ClockUncertainty {
    pub value: f64,
    pub setup: bool,
    pub hold: bool,
    pub objects: Vec<String>,
    pub from: Vec<String>,
    pub to: Vec<String>,
    /// As written: the object lists (`objects` is theirs concatenated) and every `-from` / `-to`
    /// form with its clocks, in order — `-rise_from` and `-fall_to` say which edges.
    pub written: Vec<Objects>,
    pub written_sides: Vec<(String, Objects)>,
    /// Every flag without a value, as written (`-setup`, `-hold`, `-rise`, `-fall`).
    pub flags: Vec<String>,
}

/// One `set_case_analysis` / `set_logic_zero` / `set_logic_one` / `set_logic_dc`: the value as
/// written (`0`, `1`, `zero`, `one`, `rise`, `rising`, `fall`, `falling`; `set_logic_*` give
/// `0`, `1`, `X`), the object accessor and its objects.
#[derive(Debug, Clone, PartialEq)]
pub struct LogicValueCmd {
    pub cmd: String,
    pub value: String,
    pub accessor: String,
    pub objects: Vec<String>,
    /// The objects as written — see [`Objects::flags`].
    pub written: Objects,
}

impl LogicValueCmd {
    /// The constant the value holds a pin at: `Some(false)` / `Some(true)` for 0 and 1, `None`
    /// for a transition or don't-care (no constant).
    pub fn constant(&self) -> Option<bool> {
        match self.value.as_str() {
            "0" | "zero" => Some(false),
            "1" | "one" => Some(true),
            _ => None,
        }
    }
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
    /// `-multiply_by`: the drive's resistance scale, as written.
    pub multiply_by: Option<f64>,
    pub flags: Vec<String>,
    pub objects: Vec<String>,
    /// The object lists as written — see [`Objects::flags`].
    pub written: Vec<Objects>,
}

fn driving_cell(toks: &[String]) -> DrivingCell {
    let valued = ["-lib_cell", "-library", "-pin", "-from_pin", "-input_transition_rise", "-input_transition_fall", "-multiply_by"];
    let mut d = DrivingCell { lib_cell: None, library: None, pin: None, from_pin: None, input_transition: [0.0; 2], multiply_by: None, flags: Vec::new(), objects: Vec::new(), written: Vec::new() };
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
                "-multiply_by" => d.multiply_by = v.and_then(|v| v.parse().ok()),
                other => d.flags.push(other.to_string()),
            }
            k += 2;
            continue;
        }
        if t.starts_with('-') {
            d.flags.push(t.clone());
        } else {
            d.objects.extend(resolve_objs(t));
            d.written.push(Objects::of(t));
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
    Some(EnvCmd { cmd: toks[0].clone(), value, flags, clock, accessor, accessor_flags, objects: resolve_objs(obj), written: Some(Objects::of(obj)) })
}

#[derive(Debug, Default)]
pub struct Sdc {
    /// `current_design <name>`: the design the constraints were written for (the last one named).
    pub current_design: Option<String>,
    pub clocks: Vec<SdcClock>,
    /// Each clock definition a later `create_clock` overwrote (same name) or removed (its sources
    /// taken without `-add`), as it stood, in file order — not timed; for a linter to report.
    pub superseded_clocks: Vec<SdcClock>,
    pub input_delays: Vec<IoDelay>,
    pub output_delays: Vec<IoDelay>,
    pub setup_uncertainty: f64,
    pub hold_uncertainty: f64,
    /// The LARGEST `set_clock_latency` value, of any kind and on any object (library time units).
    ///
    /// ⚠️ Kept for compatibility only — do not apply it to I/O delays. A clock's latency is on both
    /// the launch and the capture side of a path, so on one clock it cancels; what does not cancel
    /// (per clock, source vs network, on a pin) is only visible in [`Sdc::clock_latencies`].
    pub clock_latency: f64,
    /// What `set_units` declared, `(flag, value)` — `("-time", "1ps")`. NOT applied: every value
    /// stays in the library's units, as the reference reads them; a consumer compares these with
    /// its library and warns on a mismatch, as the reference does.
    pub declared_units: Vec<(String, String)>,
    /// Commands the reference would REJECT (an error, not a warning) — so they were not applied
    /// here either — each with why.
    pub rejected: Vec<String>,
    /// Commands NOT applied because an object query in them needs the design to evaluate
    /// (`-filter`, `-of_objects`, `-regexp`), each as `<command>: [<query>]`; also in
    /// [`Sdc::rejected`]. An engine that refuses rather than approximates refuses on any of these.
    pub unevaluated_queries: Vec<String>,
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
    /// Every `set_clock_latency`, per command, with its flags (`-source`, `-min`/`-max`,
    /// `-rise`/`-fall`, `-early`/`-late`): the record a consumer models latency from. A consumer
    /// that does not model latency where it fails to cancel refuses on these.
    pub clock_latencies: Vec<EnvCmd>,
    /// Every `set_clock_transition`, per command (`-rise`/`-fall`/`-min`/`-max` in its flags):
    /// an IDEAL clock's slew at its register clock pins. Still in [`Sdc::ignored`] for a consumer
    /// that does not model it.
    pub clock_transitions: Vec<EnvCmd>,
    /// Every `set_max_delay` / `set_min_delay`, in file order (still in [`Sdc::ignored`]).
    pub path_delays: Vec<PathDelayCmd>,
    /// Every `set_driving_cell`, as written (still in [`Sdc::ignored`] for a consumer that does
    /// not model input drives).
    pub driving_cells: Vec<DrivingCell>,
    /// Every `set_case_analysis`, and every `set_logic_*`, in file order (still in
    /// [`Sdc::ignored`] for a consumer that does not model constants).
    pub case_values: Vec<LogicValueCmd>,
    pub logic_values: Vec<LogicValueCmd>,
    /// Every `set_load` and `set_input_transition`, per command, for a consumer that models
    /// them per object ([`Sdc::load`] / [`Sdc::input_transition`] keep one design-wide value).
    pub env: Vec<EnvCmd>,
    /// The design-wide derates: from `set_timing_derate` commands that name no objects and no
    /// delay or path type — the only ones a single value can stand for. See [`Sdc::derates`].
    pub late_derate: Option<f64>,
    pub early_derate: Option<f64>,
    /// Every `set_timing_derate`, per command: its value (the positional — `-early` / `-late` are
    /// FLAGS, not valued), its flags (`-early`, `-late`, `-cell_delay`, `-net_delay`, `-data`, …)
    /// and its objects.
    pub derates: Vec<EnvCmd>,
    /// Every `set_clock_uncertainty`, per command — see [`ClockUncertainty`].
    pub uncertainties: Vec<ClockUncertainty>,
    pub exceptions: Vec<Exception>,
    /// Per entry of [`Sdc::exceptions`], index for index: the objects its `-from` side and its
    /// `-to` side named, accessor kept (`-rise_from` / `-fall_from` on the from side, likewise to).
    /// [`Exception::from`] / [`Exception::to`] are derived from these.
    pub exception_objects: Vec<[Vec<Objects>; 2]>,
    /// `set_clock_groups -asynchronous`: each inner vec is one `-group` of clock
    /// names. Two clocks in *different* groups are mutually asynchronous — paths
    /// that launch and capture across them are cut (no setup **or** hold check).
    pub async_groups: Vec<Vec<String>>,
    /// Every `set_clock_groups`, PER COMMAND — see [`ClockGroups`] and [`clocks_unrelated`].
    /// [`Sdc::async_groups`] is every command's groups in one list, and cannot say which clocks
    /// one command separated.
    pub clock_groups: Vec<ClockGroups>,
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
    "set_timing_derate",     // recorded here only when SCOPED (objects, a delay or path type)
    // ── Tcl control structure: what it would have done is unknown without running it ────
    "if",
    "foreach",
    "for",
    "while",
    "switch",
    "proc",
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
                    let (names, refused) = reduce_exception_objects(list);
                    if side == 0 { e.from = names } else { e.to = names }
                    e.refused = e.refused.take().or(refused);
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
        if !self.rejected.is_empty() {
            notes.push(format!("{} command(s) rejected: {}", self.rejected.len(), self.rejected.join("; ")));
        }
        let refused = self.exceptions.iter().filter(|e| e.refused.is_some()).count();
        if refused > 0 {
            notes.push(format!(
                "{refused} timing exception(s) NOT applied — they name a -through list or a pin pattern, \
                 which an instance-level match cannot express"
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

/// Split the file into COMMANDS, as Tcl does: a command ends at a newline or `;` only OUTSIDE
/// braces, brackets and quotes; a backslash-newline is a space; `#` starts a comment only where a
/// command starts, and runs to the end of its line.
///
/// ⛔ It split on every newline and `;`, whatever the nesting. The body of `if {0} { … }` then
/// ran as though unconditional (a clock the file never creates was created), a `foreach` body
/// was read without its loop, and a brace list written across lines became fragments. A control
/// structure now arrives whole, as one command this reader does not run — and says so.
fn logical_lines(text: &str) -> Vec<String> {
    let cs: Vec<char> = text.chars().filter(|&c| c != '\r').collect();
    let mut out = Vec::new();
    let mut cur = String::new();
    let (mut brace, mut bracket, mut quote) = (0i32, 0i32, false);
    let push = |cur: &mut String, out: &mut Vec<String>| {
        let t = cur.trim();
        if !t.is_empty() {
            out.push(t.to_string());
        }
        cur.clear();
    };
    let mut i = 0;
    while i < cs.len() {
        let c = cs[i];
        if c == '\\' && i + 1 < cs.len() {
            if cs[i + 1] == '\n' {
                // a continuation: the newline and the next line's leading blanks are one space
                cur.push(' ');
                i += 2;
                while i < cs.len() && (cs[i] == ' ' || cs[i] == '\t') {
                    i += 1;
                }
            } else {
                cur.push(c);
                cur.push(cs[i + 1]);
                i += 2;
            }
            continue;
        }
        if c == '#' && brace == 0 && bracket == 0 && !quote && cur.trim().is_empty() {
            // a comment where a command starts, to the end of the line (a backslash continues it)
            while i < cs.len() && cs[i] != '\n' {
                i += if cs[i] == '\\' { 2 } else { 1 };
            }
            continue;
        }
        match c {
            '{' if !quote => brace += 1,
            '}' if !quote => brace = (brace - 1).max(0),
            '[' if brace == 0 && !quote => bracket += 1,
            ']' if brace == 0 && !quote => bracket = (bracket - 1).max(0),
            '"' if brace == 0 => quote = !quote,
            '\n' | ';' if brace == 0 && bracket == 0 && !quote => {
                push(&mut cur, &mut out);
                i += 1;
                continue;
            }
            _ => {}
        }
        cur.push(c);
        i += 1;
    }
    push(&mut cur, &mut out);
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
            while i < cs.len() && !cs[i].is_whitespace() && cs[i] != '{' {
                // A BUS INDEX is part of the word: `a[0]`, `d[7:0]`. The reference's unknown-
                // command handler gives `[0]` back as written rather than running it, so
                // `[get_ports a[0]]` names bit 0 of `a`; split there, it named `a` and `[0]`.
                if cs[i] == '[' {
                    let close = cs[i..].iter().position(|&c| c == ']').map(|k| i + k);
                    let index = close.filter(|&e| e > i + 1 && cs[i + 1..e].iter().all(|c| c.is_ascii_digit() || *c == ':'));
                    match index {
                        Some(e) if !s.is_empty() => {
                            s.extend(&cs[i..=e]);
                            i = e + 1;
                            continue;
                        }
                        _ => break,
                    }
                }
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
/// `[lsearch -inline -all -not -exact [all_inputs] [get_ports P]]` — every input but P, the usual
/// way to constrain every input except the clock's — as the accessor `all_inputs_except` (or
/// `all_outputs_except`) naming the excluded port. Tcl's `lsearch -not -exact` drops the elements
/// EQUAL to the pattern: one port is a one-element list equal to its element; a pattern of two or
/// more ports equals no element, so it is not read here (`None`).
fn all_ports_except(tok: &str) -> Option<(String, Vec<String>)> {
    let inner = tok.strip_prefix('[')?.strip_suffix(']')?.trim();
    let parts = tokenize(inner);
    let [cmd, a, b, c, d, list, pattern] = parts.as_slice() else { return None };
    if cmd != "lsearch" {
        return None;
    }
    let mut opts = [a.as_str(), b.as_str(), c.as_str(), d.as_str()];
    opts.sort_unstable();
    if opts != ["-all", "-exact", "-inline", "-not"] {
        return None;
    }
    let accessor = match resolve_objs(list).as_slice() {
        [one] if one == "*INPUTS*" => "all_inputs_except",
        [one] if one == "*OUTPUTS*" => "all_outputs_except",
        _ => return None,
    };
    let excluded = resolve_objs(pattern);
    (excluded.len() == 1).then(|| (accessor.to_string(), excluded))
}

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
                // The patterns are the POSITIONAL arguments: a key (`-filter`, `-of_objects`,
                // `-hsc`, abbreviated or not) takes the next word as its value, a flag
                // (`-hierarchical`, `-quiet`) none. ⛔ Skipping only the `-` words read a filter
                // expression or an `-of_objects` query as object names.
                let parts = expand_abbreviations(parts);
                // Against a linked design, a query that selects by property or connectivity is
                // evaluated there (one it cannot evaluate was refused by the command already).
                if parts.iter().any(|p| p == "-filter" || p == "-of_objects") {
                    if let Some(Ok(names)) = with_design(|d| crate::sdc_query::eval(&parts, d)) {
                        return names;
                    }
                }
                let keys = crate::sdc_flags::FLAGS.iter().find(|(n, _, _)| *n == parts[0]).map_or(&[][..], |(_, k, _)| *k);
                let mut names = Vec::new();
                let mut k = 1;
                while k < parts.len() {
                    let p = &parts[k];
                    if keys.contains(&p.as_str()) {
                        k += 2;
                        continue;
                    }
                    if !p.starts_with('-') {
                        names.extend(p.split_whitespace().map(|s| s.to_string()));
                    }
                    k += 1;
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
/// What `create_generated_clock` asks of its master, as written.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct GeneratedClock {
    /// `-source`: the master's pin, as written.
    pub source: Objects,
    /// `-master_clock`, when given (else the master is the clock on `source`).
    pub master_clock: Option<String>,
    pub divide_by: Option<i64>,
    pub multiply_by: Option<i64>,
    pub duty_cycle: Option<f64>,
    pub edges: Vec<i64>,
    pub edge_shift: Vec<f64>,
    pub invert: bool,
    pub combinational: bool,
}

/// A generated clock's period and waveform from its master — the reference's `Clock::generate`:
/// a power-of-two divide keeps the master's rise and gives a 50% duty cycle; any other divide or
/// a multiply SCALES the master's waveform (or applies `-duty_cycle`); `-edges` picks master edges
/// (1-based, counting through the master's edge list cycle after cycle), shifted by `-edge_shift`;
/// `-invert` then rotates the edges by one. `None` when the request cannot be built.
fn generate(master: &SdcClock, g: &GeneratedClock) -> Option<(f64, Vec<f64>)> {
    let mp = master.period;
    let mw: Vec<f64> = match &master.waveform {
        Some(w) if w.len() >= 2 => w.clone(),
        _ => vec![0.0, mp / 2.0],
    };
    let scaled = |scale: f64| -> (f64, Vec<f64>) {
        let p = mp * scale;
        match g.duty_cycle {
            Some(d) if d != 0.0 => {
                let rise = mw[0] * scale;
                (p, vec![rise, rise + p * d / 100.0])
            }
            _ => (p, mw.iter().map(|t| t * scale).collect()),
        }
    };
    let (period, mut wave) = match (g.divide_by, g.multiply_by) {
        (Some(1), _) | (None, None) if g.edges.is_empty() => (mp, vec![mw[0], mw[1]]),
        (Some(d), _) if d > 1 => {
            if d & (d - 1) == 0 {
                let p = mp * d as f64;
                (p, vec![mw[0], mw[0] + p / 2.0])
            } else {
                scaled(d as f64)
            }
        }
        (_, Some(m)) if m >= 1 => scaled(1.0 / m as f64),
        _ if g.edges.len() == 3 => {
            let n = mw.len() as i64;
            let at = |e: i64| -> f64 {
                let k = e - 1;
                mw[k.rem_euclid(n) as usize] + k.div_euclid(n) as f64 * mp
            };
            let sh = |k: usize| g.edge_shift.get(k).copied().unwrap_or(0.0);
            let rise = at(g.edges[0]) + sh(0);
            let fall = at(g.edges[1]) + sh(1);
            let period = at(g.edges[2]) - rise + sh(2);
            (period, vec![rise, fall])
        }
        _ => return None,
    };
    if g.invert {
        let first = wave[0];
        let offset = if first >= period { period } else { 0.0 };
        let n = wave.len();
        for k in 0..n - 1 {
            wave[k] = wave[k + 1] - offset;
        }
        wave[n - 1] = first - offset + period;
    }
    Some((period, wave))
}

/// Abbreviations, expanded as the reference expands them before a command runs: a command name
/// that begins exactly one SDC command is that command (`set_max_trans`), and an argument that
/// is not one of the command's keys or flags exactly is the FIRST key it begins, else the first
/// flag, in the reference's order (`-per` is `-period`). Lists the reference parses on their own
/// (`-through`, `-group`) are left as written.
///
/// ⛔ Unexpanded, `create_clock -per 10` failed the whole file, and `set_max_trans 0.5` was an
/// unknown command whose limit was lost.
fn expand_abbreviations(mut toks: Vec<String>) -> Vec<String> {
    use crate::sdc_flags::{COMMANDS, FLAGS};
    const OWN: [&str; 4] = ["-through", "-rise_through", "-fall_through", "-group"];
    let Some(c) = toks.first() else { return toks };
    if !COMMANDS.contains(&c.as_str()) && c.starts_with(|ch: char| ch.is_ascii_lowercase()) && c.contains('_') {
        let m: Vec<&&str> = COMMANDS.iter().filter(|k| k.starts_with(c.as_str())).collect();
        if m.len() == 1 {
            toks[0] = m[0].to_string();
        }
    }
    let Some((_, keys, flags)) = FLAGS.iter().find(|(n, _, _)| *n == toks[0]) else { return toks };
    let mut k = 1;
    while k < toks.len() {
        let t = toks[k].as_str();
        if t.len() > 1 && t.starts_with('-') && t.parse::<f64>().is_err() {
            if keys.contains(&t) {
                k += 2;
                continue;
            }
            if !flags.contains(&t) && !OWN.contains(&t) {
                if let Some(full) = keys.iter().find(|x| x.starts_with(t)) {
                    toks[k] = full.to_string();
                    k += 2;
                    continue;
                }
                if let Some(full) = flags.iter().find(|x| x.starts_with(t)) {
                    toks[k] = full.to_string();
                }
            }
        }
        k += 1;
    }
    toks
}

/// The first object query in `toks` (at any depth of `[...]`) this reader cannot evaluate, as
/// written: `-filter`, `-of_objects` and `-regexp` select objects by the design's properties and
/// connectivity (and `-regexp` by a different pattern language), which needs the design. Matched
/// as names they selected the wrong objects or none, silently; the caller refuses the command.
fn unevaluated_query(toks: &[String]) -> Option<String> {
    const NEEDS_DESIGN: [&str; 3] = ["-filter", "-of_objects", "-regexp"];
    for t in toks {
        let Some(inner) = t.strip_prefix('[').and_then(|s| s.strip_suffix(']')) else { continue };
        let parts = expand_abbreviations(tokenize(inner.trim()));
        let Some(cmd) = parts.first() else { continue };
        // ⛔ A COMMAND THIS READER DOES NOT KNOW is not a name. `[remove_from_collection [all_inputs]
        // {clk}]` (a synthesis tool's command, which the reference does not have either) was read as
        // a port called `remove_from_collection`: the delay went nowhere, and nothing said so.
        // `expr` is evaluated where it can be, and a value it leaves is reported where it is read.
        let known = KNOWN_QUERIES.contains(&cmd.as_str()) || cmd == "expr" || (cmd == "lsearch" && all_ports_except(t).is_some());
        if !known {
            return Some(format!("[{}] (a command this reader does not evaluate)", inner.trim()));
        }
        if cmd.starts_with("get_") && parts.iter().any(|p| NEEDS_DESIGN.contains(&p.as_str())) {
            match with_design(|d| crate::sdc_query::eval(&parts, d)) {
                Some(Ok(_)) => continue,
                Some(Err(why)) => return Some(format!("[{}] ({why})", inner.trim())),
                None => return Some(format!("[{}] (needs the design)", inner.trim())),
            }
        }
        if let Some(q) = unevaluated_query(&parts[1..]) {
            return Some(q);
        }
    }
    None
}

/// The commands an object list or a value may be written with, which this reader evaluates.
const KNOWN_QUERIES: [&str; 15] = [
    "get_ports", "get_pins", "get_cells", "get_nets", "get_clocks", "get_lib_cells", "get_lib_pins", "get_libs",
    "all_inputs", "all_outputs", "all_clocks", "all_registers", "list", "concat", "current_design",
];

/// The first variable read OUTSIDE braces that is not defined — `$name`, `${name}`, `$::env(NAME)`.
/// Tcl substitutes nothing inside `{…}`, so a `$` there is text (an escaped name's `$_DFF_P_`).
fn unresolved_var(raw: &str, vars: &HashMap<String, String>) -> Option<String> {
    let cs: Vec<char> = raw.chars().collect();
    let (mut depth, mut i) = (0usize, 0usize);
    while i < cs.len() {
        match cs[i] {
            '\\' => i += 1,
            '{' => depth += 1,
            '}' => depth = depth.saturating_sub(1),
            '$' if depth == 0 => {
                let rest: String = cs[i + 1..].iter().collect();
                let env = ["::env(", "env("].iter().find(|p| rest.starts_with(**p)).and_then(|p| rest[p.len()..].split(')').next().map(str::to_string));
                let name = match env {
                    Some(n) => {
                        if vars.contains_key(&n) || std::env::var(&n).is_ok() {
                            i += 1;
                            continue;
                        }
                        return Some(format!("$::env({n})"));
                    }
                    None if rest.starts_with('{') => rest[1..].split('}').next().unwrap_or("").to_string(),
                    None => rest.chars().take_while(|c| c.is_alphanumeric() || *c == '_' || *c == ':').collect(),
                };
                if !name.is_empty() && !vars.contains_key(name.trim_start_matches("::")) {
                    return Some(format!("${name}"));
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// `[info exists NAME]` in a condition as Tcl answers it: 1 when the variable is set (`::env(X)`
/// from the process environment, as Tcl reads it), else 0. A flow's `if {[info exists
/// ::env(CLK_PERIOD_PS)]}` override was refused as undecidable, though nothing in it needs a design.
fn subst_info_exists(cond: &str, vars: &HashMap<String, String>) -> String {
    let mut out = String::new();
    let mut rest = cond;
    while let Some(k) = rest.find("[info exists ") {
        out.push_str(&rest[..k]);
        let tail = &rest[k + "[info exists ".len()..];
        let Some(e) = tail.find(']') else {
            out.push_str(&rest[k..]);
            return out;
        };
        let name = tail[..e].trim();
        let env = ["::env(", "env("].iter().find_map(|p| name.strip_prefix(p)).and_then(|n| n.strip_suffix(')'));
        let set = match env {
            Some(n) => vars.contains_key(n) || std::env::var(n).is_ok(),
            None => vars.contains_key(name.trim_start_matches("::")),
        };
        out.push_str(if set { "1" } else { "0" });
        rest = &tail[e + 1..];
    }
    out.push_str(rest);
    out
}

/// A value list as words: `{0 5}` arrives braces stripped; `[list 0 5]` — Tcl's list command, as
/// `-waveform [list 0 [expr $p/2]]` writes it — is its elements.
fn list_words(v: &str) -> String {
    match v.trim().strip_prefix("[list").and_then(|i| i.strip_suffix(']')) {
        Some(inner) => tokenize(inner.trim()).join(" "),
        None => v.to_string(),
    }
}

/// What a control command asks for: its chosen commands, a refusal (it cannot be evaluated
/// here), or nothing (it is not a control command).
enum Control {
    Run(Vec<String>),
    Refuse(String),
    None,
}

/// `if` and `foreach`, evaluated as the reference's Tcl would — where they can be.
///
/// `if {cond} {body} ?elseif {cond} {body}? ?else {body}?` runs the first branch whose condition
/// holds; `foreach v {a b} {body}` runs its body once per element with `v` set. A condition or a
/// list this reader cannot evaluate (a design query, a `proc`) REFUSES the whole command — the
/// branch it would have taken is unknown, and running every branch is how a file's default and
/// its override both applied, in file order.
fn control(raw: &str, vars: &HashMap<String, String>) -> Control {
    let head = raw.split_whitespace().next().unwrap_or("");
    match head {
        "if" => {
            let toks = tokenize(raw);
            let mut k = 1;
            while k + 1 < toks.len() {
                let cond = subst_vars(&subst_info_exists(&toks[k], vars), vars);
                let mut body = k + 1;
                if toks.get(body).map(String::as_str) == Some("then") {
                    body += 1;
                }
                match eval_expr(&cond, vars).map(|v| v.truthy()) {
                    Some(Some(true)) => return Control::Run(logical_lines(&toks[body])),
                    Some(Some(false)) => {}
                    _ => return Control::Refuse("if".into()),
                }
                match toks.get(body + 1).map(String::as_str) {
                    Some("elseif") => k = body + 2,
                    Some("else") => return Control::Run(logical_lines(toks.get(body + 2).map(String::as_str).unwrap_or(""))),
                    _ => return Control::Run(Vec::new()),
                }
            }
            Control::Refuse("if".into())
        }
        "foreach" => {
            let toks = tokenize(raw);
            if toks.len() != 4 || toks[2].starts_with('[') {
                return Control::Refuse("foreach".into());
            }
            let list = subst_vars(&toks[2], vars);
            if list.contains('[') || list.contains('$') {
                return Control::Refuse("foreach".into());
            }
            let body = logical_lines(&toks[3]);
            let mut out = Vec::new();
            for e in list.split_whitespace() {
                out.push(format!("set {} {{{e}}}", toks[1]));
                out.extend(body.iter().cloned());
            }
            Control::Run(out)
        }
        _ => Control::None,
    }
}

/// A Tcl `expr` value: integer, double, or string — Tcl keeps the three apart, and integer
/// division truncates (`10 / 4` is 2).
#[derive(Debug, Clone, PartialEq)]
enum Val {
    Int(i64),
    Dbl(f64),
    Str(String),
}

impl Val {
    fn num(&self) -> Option<f64> {
        match self {
            Val::Int(i) => Some(*i as f64),
            Val::Dbl(d) => Some(*d),
            Val::Str(s) => s.trim().parse().ok(),
        }
    }
    /// Tcl's truth: a number is true when non-zero; `true`/`yes`/`on` and their opposites.
    fn truthy(&self) -> Option<bool> {
        if let Some(n) = self.num() {
            return Some(n != 0.0);
        }
        match self {
            Val::Str(s) => match s.to_ascii_lowercase().as_str() {
                "true" | "yes" | "on" => Some(true),
                "false" | "no" | "off" => Some(false),
                _ => None,
            },
            _ => None,
        }
    }
    /// As Tcl prints it: an integer bare, a double in its shortest exact form (with `.0` when
    /// it is integral, as Tcl writes `3.0`).
    fn text(&self) -> String {
        match self {
            Val::Int(i) => i.to_string(),
            Val::Dbl(d) if d.fract() == 0.0 && d.is_finite() => format!("{d:.1}"),
            Val::Dbl(d) => d.to_string(),
            Val::Str(s) => s.clone(),
        }
    }
    fn of(tok: &str) -> Val {
        let t = tok.trim();
        if let Ok(i) = t.parse::<i64>() {
            Val::Int(i)
        } else if let Ok(d) = t.parse::<f64>() {
            Val::Dbl(d)
        } else {
            Val::Str(t.to_string())
        }
    }
}

/// Evaluate a Tcl expression — the subset SDC files use: numbers, strings, `$var` / `${var}` /
/// `$::env(X)` (already substituted by the caller where set), `[info exists name]`, `( )`,
/// unary `- + !`, `* / %`, `+ -`, `< > <= >=`, `== != eq ne`, `&& ||`, and the functions `int`,
/// `double`, `round`, `abs`, `floor`, `ceil`, `min`, `max`. `None` for anything else, which the
/// caller refuses rather than guesses.
fn eval_expr(src: &str, vars: &HashMap<String, String>) -> Option<Val> {
    let toks = expr_tokens(src, vars)?;
    let mut p = ExprParser { t: toks, i: 0 };
    let v = p.or()?;
    (p.i == p.t.len()).then_some(v)
}

#[derive(Debug, Clone, PartialEq)]
enum ET {
    V(Val),
    Op(&'static str),
    Ident(String),
}

fn expr_tokens(src: &str, vars: &HashMap<String, String>) -> Option<Vec<ET>> {
    let s = subst_vars(src.trim(), vars);
    let cs: Vec<char> = s.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    const OPS: [&str; 20] = ["&&", "||", "==", "!=", "<=", ">=", "<", ">", "+", "-", "*", "/", "%", "!", "(", ")", ",", "?", ":", "**"];
    while i < cs.len() {
        let c = cs[i];
        if c.is_whitespace() || c == '{' || c == '}' {
            i += 1;
            continue;
        }
        if c == '[' {
            // `[info exists NAME]` — the only command an SDC condition needs
            let close = cs[i..].iter().position(|&x| x == ']')? + i;
            let inner: String = cs[i + 1..close].iter().collect();
            let w: Vec<&str> = inner.split_whitespace().collect();
            match w.as_slice() {
                ["info", "exists", name] => {
                    let name = name.trim_start_matches("::");
                    let set = match name.strip_prefix("env(").and_then(|n| n.strip_suffix(')')) {
                        Some(e) => std::env::var(e).is_ok() || vars.contains_key(e),
                        None => vars.contains_key(name),
                    };
                    out.push(ET::V(Val::Int(set as i64)));
                }
                ["expr", rest @ ..] => out.push(ET::V(eval_expr(&rest.join(" "), vars)?)),
                _ => return None,
            }
            i = close + 1;
            continue;
        }
        if c == '"' {
            let close = cs[i + 1..].iter().position(|&x| x == '"')? + i + 1;
            out.push(ET::V(Val::Str(cs[i + 1..close].iter().collect())));
            i = close + 1;
            continue;
        }
        if c == '$' {
            return None; // a variable left unsubstituted: not set
        }
        if c.is_ascii_digit() || (c == '.' && cs.get(i + 1).is_some_and(|d| d.is_ascii_digit())) {
            let mut j = i;
            while j < cs.len() && (cs[j].is_ascii_alphanumeric() || cs[j] == '.' || ((cs[j] == '-' || cs[j] == '+') && matches!(cs[j - 1], 'e' | 'E'))) {
                j += 1;
            }
            let t: String = cs[i..j].iter().collect();
            match Val::of(&t) {
                Val::Str(_) => return None,
                v => out.push(ET::V(v)),
            }
            i = j;
            continue;
        }
        if c.is_alphabetic() || c == '_' {
            let mut j = i;
            while j < cs.len() && (cs[j].is_alphanumeric() || cs[j] == '_') {
                j += 1;
            }
            let w: String = cs[i..j].iter().collect();
            match w.as_str() {
                "eq" => out.push(ET::Op("==")),
                "ne" => out.push(ET::Op("!=")),
                "true" | "false" | "yes" | "no" | "on" | "off" => out.push(ET::V(Val::Str(w))),
                _ => out.push(ET::Ident(w)),
            }
            i = j;
            continue;
        }
        let two: String = cs[i..(i + 2).min(cs.len())].iter().collect();
        if let Some(op) = OPS.iter().find(|o| o.len() == 2 && **o == two) {
            out.push(ET::Op(op));
            i += 2;
            continue;
        }
        let one = c.to_string();
        let op = OPS.iter().find(|o| o.len() == 1 && **o == one)?;
        out.push(ET::Op(op));
        i += 1;
    }
    Some(out)
}

struct ExprParser {
    t: Vec<ET>,
    i: usize,
}

impl ExprParser {
    fn peek(&self) -> Option<&ET> {
        self.t.get(self.i)
    }
    fn eat(&mut self, op: &str) -> bool {
        match self.peek() {
            Some(ET::Op(o)) if *o == op => {
                self.i += 1;
                true
            }
            _ => false,
        }
    }
    fn or(&mut self) -> Option<Val> {
        let mut v = self.and()?;
        while self.eat("||") {
            let r = self.and()?;
            v = Val::Int((v.truthy()? || r.truthy()?) as i64);
        }
        Some(v)
    }
    fn and(&mut self) -> Option<Val> {
        let mut v = self.cmp()?;
        while self.eat("&&") {
            let r = self.cmp()?;
            v = Val::Int((v.truthy()? && r.truthy()?) as i64);
        }
        Some(v)
    }
    fn cmp(&mut self) -> Option<Val> {
        let mut v = self.add()?;
        loop {
            let op = match self.peek() {
                Some(ET::Op(o @ ("==" | "!=" | "<" | ">" | "<=" | ">="))) => *o,
                _ => return Some(v),
            };
            self.i += 1;
            let r = self.add()?;
            let ord = match (v.num(), r.num()) {
                (Some(a), Some(b)) => a.partial_cmp(&b)?,
                _ => v.text().cmp(&r.text()),
            };
            use std::cmp::Ordering::*;
            let t = match op {
                "==" => ord == Equal,
                "!=" => ord != Equal,
                "<" => ord == Less,
                ">" => ord == Greater,
                "<=" => ord != Greater,
                _ => ord != Less,
            };
            v = Val::Int(t as i64);
        }
    }
    fn add(&mut self) -> Option<Val> {
        let mut v = self.mul()?;
        loop {
            let op = match self.peek() {
                Some(ET::Op(o @ ("+" | "-"))) => *o,
                _ => return Some(v),
            };
            self.i += 1;
            let r = self.mul()?;
            v = arith(op, &v, &r)?;
        }
    }
    fn mul(&mut self) -> Option<Val> {
        let mut v = self.unary()?;
        loop {
            let op = match self.peek() {
                Some(ET::Op(o @ ("*" | "/" | "%"))) => *o,
                _ => return Some(v),
            };
            self.i += 1;
            let r = self.unary()?;
            v = arith(op, &v, &r)?;
        }
    }
    fn unary(&mut self) -> Option<Val> {
        if self.eat("-") {
            return arith("-", &Val::Int(0), &self.unary()?);
        }
        if self.eat("+") {
            return self.unary();
        }
        if self.eat("!") {
            return Some(Val::Int(!self.unary()?.truthy()? as i64));
        }
        self.atom()
    }
    fn atom(&mut self) -> Option<Val> {
        match self.t.get(self.i).cloned()? {
            ET::V(v) => {
                self.i += 1;
                Some(v)
            }
            ET::Op("(") => {
                self.i += 1;
                let v = self.or()?;
                self.eat(")").then_some(v)
            }
            ET::Ident(f) => {
                self.i += 1;
                if !self.eat("(") {
                    return None;
                }
                let mut args = vec![self.or()?];
                while self.eat(",") {
                    args.push(self.or()?);
                }
                if !self.eat(")") {
                    return None;
                }
                let x = args[0].num()?;
                Some(match (f.as_str(), args.len()) {
                    ("int", 1) => Val::Int(x.trunc() as i64),
                    ("round", 1) => Val::Int(x.round() as i64),
                    ("double", 1) => Val::Dbl(x),
                    ("abs", 1) => match args[0] {
                        Val::Int(i) => Val::Int(i.abs()),
                        _ => Val::Dbl(x.abs()),
                    },
                    ("floor", 1) => Val::Dbl(x.floor()),
                    ("ceil", 1) => Val::Dbl(x.ceil()),
                    ("min" | "max", _) => {
                        let mut best = args[0].clone();
                        for a in &args[1..] {
                            let (b, y) = (best.num()?, a.num()?);
                            if (f == "min" && y < b) || (f == "max" && y > b) {
                                best = a.clone();
                            }
                        }
                        best
                    }
                    _ => return None,
                })
            }
            _ => None,
        }
    }
}

/// Tcl arithmetic: two integers stay integer — and Tcl's integer `/` rounds toward NEGATIVE
/// infinity, not toward zero as C's does (`-7 / 2` is -4), with `%` taking the divisor's sign;
/// anything with a double is double.
fn arith(op: &str, a: &Val, b: &Val) -> Option<Val> {
    let ints = match (a, b) {
        (Val::Int(x), Val::Int(y)) => Some((*x, *y)),
        _ => None,
    };
    if let Some((x, y)) = ints {
        return Some(Val::Int(match op {
            "+" => x.checked_add(y)?,
            "-" => x.checked_sub(y)?,
            "*" => x.checked_mul(y)?,
            "/" | "%" if y != 0 => {
                let q = x / y - i64::from(x % y != 0 && ((x < 0) != (y < 0)));
                if op == "/" {
                    q
                } else {
                    x - y * q
                }
            }
            _ => return None,
        }));
    }
    let (x, y) = (a.num()?, b.num()?);
    Some(Val::Dbl(match op {
        "+" => x + y,
        "-" => x - y,
        "*" => x * y,
        "/" if y != 0.0 => x / y,
        _ => return None,
    }))
}

/// Replace every `[expr …]` in a command by its value — the evaluation Tcl does before the
/// command runs. One this reader cannot evaluate is left as written, so a consumer sees an
/// unread value rather than a guessed one.
///
/// ⛔ Unevaluated, `set_input_delay [expr {$period * 0.2}] …` read as a delay of ZERO (marked
/// unread, and timed as 0 anyway): 130 commands in 41 files of the corpus.
fn subst_exprs(line: &str, vars: &HashMap<String, String>) -> String {
    let mut out = String::new();
    let mut rest = line;
    while let Some(k) = rest.find("[expr") {
        out.push_str(&rest[..k]);
        let tail = &rest[k..];
        // the matching `]`
        let mut depth = 0;
        let mut end = None;
        for (j, c) in tail.char_indices() {
            match c {
                '[' => depth += 1,
                ']' => {
                    depth -= 1;
                    if depth == 0 {
                        end = Some(j);
                        break;
                    }
                }
                _ => {}
            }
        }
        let Some(e) = end else {
            out.push_str(tail);
            return out;
        };
        let inner = &tail[5..e];
        match eval_expr(inner, vars) {
            Some(v) => out.push_str(&v.text()),
            None => out.push_str(&tail[..=e]),
        }
        rest = &tail[e + 1..];
    }
    out.push_str(rest);
    out
}

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


/// What an object list names, each as `(is_clock, name)`: `get_clocks` / `all_clocks` name clocks,
/// `get_pins` / `get_ports` pins, and a bare name is a clock when a clock has that name.
fn object_targets(o: &Objects, clocks: &[SdcClock], out: &mut std::collections::HashSet<(bool, String)>) {
    if !o.items.is_empty() {
        for i in &o.items {
            object_targets(i, clocks, out);
        }
        return;
    }
    for n in &o.names {
        let clock = match o.accessor.as_str() {
            "get_clocks" | "all_clocks" => true,
            "get_pins" | "get_ports" => false,
            _ => clocks.iter().any(|c| &c.name == n),
        };
        out.insert((clock, n.clone()));
    }
}

/// `o` without the objects in `targets` (an element of a `[list …]` left empty is dropped).
fn drop_targets(o: &mut Objects, targets: &std::collections::HashSet<(bool, String)>, clocks: &[SdcClock]) {
    if !o.items.is_empty() {
        for i in &mut o.items {
            drop_targets(i, targets, clocks);
        }
        o.items.retain(|i| !i.names.is_empty());
        o.names = o.items.iter().flat_map(|i| i.names.iter().cloned()).collect();
        return;
    }
    let acc = o.accessor.clone();
    o.names.retain(|n| {
        let mut one = std::collections::HashSet::new();
        object_targets(&Objects { accessor: acc.clone(), names: vec![n.clone()], ..Default::default() }, clocks, &mut one);
        one.is_disjoint(targets)
    });
}

impl Sdc {
    fn targets(&self, o: &Objects) -> std::collections::HashSet<(bool, String)> {
        let mut t = std::collections::HashSet::new();
        object_targets(o, &self.clocks, &mut t);
        t
    }

    /// A clock command's `[all_clocks]` as the clocks defined so far — what it names when it runs.
    fn clocks_now(&self, mut e: EnvCmd) -> EnvCmd {
        if e.accessor == "all_clocks" {
            e.objects = self.clocks.iter().map(|c| c.name.clone()).collect();
            if let Some(w) = &mut e.written {
                w.names = e.objects.clone();
            }
        }
        e
    }
}

// ---- parse ---------------------------------------------------------------

impl Sdc {
    pub fn parse(text: &str) -> Result<Sdc, SdcError> {
        let mut sdc = Sdc::default();
        let mut vars: HashMap<String, String> = HashMap::new();

        // A QUEUE of commands, not a list: a chosen `if` branch or a `foreach` body is put back at
        // its front, so it runs in place and in order — and its variables are substituted when
        // each of its commands runs, as Tcl does, not when the `if` was read.
        let mut queue: std::collections::VecDeque<String> = logical_lines(text).into();
        while let Some(raw) = queue.pop_front() {
            match control(&raw, &vars) {
                Control::Run(cmds) => {
                    for c in cmds.into_iter().rev() {
                        queue.push_front(c);
                    }
                    continue;
                }
                Control::Refuse(kw) => {
                    sdc.ignored.push(kw);
                    continue;
                }
                Control::None => {}
            }
            let line = subst_exprs(&subst_vars(&raw, &vars), &vars);
            let toks = expand_abbreviations(tokenize(&line));
            if toks.is_empty() {
                continue;
            }
            // ⛔ An UNDEFINED VARIABLE is an error to Tcl, so to the reference; not applied here
            // either, and listed with the queries not evaluated — the list every consumer refuses
            // on. Kept as text, `-lib_cell ${DRIVING_CELL}` split into a lib cell `$` and a stray
            // word, and an unread delay was applied as 0 by a consumer that did not check
            // `value_read`. `create_clock` reports its own, naming the variable; `set` stores text.
            if !matches!(toks[0].as_str(), "set" | "create_clock") {
                if let Some(v) = unresolved_var(&raw, &vars) {
                    sdc.rejected.push(format!("{}: the variable {v} is not defined; not applied", toks[0]));
                    sdc.unevaluated_queries.push(format!("{}: {v} (not defined)", toks[0]));
                    continue;
                }
            }
            // An object query this reader cannot evaluate: the command is NOT applied, and says
            // so (a `set` only stores the text; it is checked where the variable is used).
            if toks[0] != "set" {
                if let Some(q) = unevaluated_query(&toks) {
                    sdc.rejected.push(format!("{}: {q} not evaluated; not applied", toks[0]));
                    sdc.unevaluated_queries.push(format!("{}: {q}", toks[0]));
                    continue;
                }
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
                "current_design" => {
                    if let Some(n) = toks.get(1) {
                        sdc.current_design = resolve_objs(n).into_iter().next();
                    }
                }
                "set_units" | "set_unit" => {
                    // ⛔ CHECKED, NOT APPLIED — the reference's rule. `set_units` only asserts what
                    // the library's units are (it warns when they differ); every value in the file
                    // stays in the library's units. Applied as a scale, `set_units -time ps` on an
                    // ns library turned a 10 ns period into 0.01. Kept, for a consumer to check
                    // against its library.
                    let mut k = 1;
                    while k + 1 < toks.len() {
                        if toks[k].starts_with('-') {
                            sdc.declared_units.push((toks[k].clone(), toks[k + 1].clone()));
                            k += 2;
                        } else {
                            k += 1;
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
                        list_words(w).split_whitespace().map(parse_time).collect::<Option<Vec<f64>>>().unwrap_or_default()
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
                    let clock = SdcClock { name, source: src, sources, period, waveform, add, generated: None };
                    match sdc.clocks.iter().position(|c| c.name == clock.name) {
                        Some(i) => sdc.superseded_clocks.push(std::mem::replace(&mut sdc.clocks[i], clock)),
                        None => sdc.clocks.push(clock),
                    }
                }
                "create_generated_clock" => {
                    let (with, flags, pos) = split_args(
                        &toks,
                        &["-name", "-source", "-divide_by", "-multiply_by", "-edges", "-edge_shift", "-duty_cycle", "-comment", "-master_clock"],
                    );
                    let val = |f: &str| with.iter().find(|(k, _)| *k == f).map(|(_, v)| v.as_str());
                    let has = |f: &str| flags.iter().any(|x| x == f);
                    let sources: Vec<String> = pos.iter().flat_map(|o| resolve_objs(o)).collect();
                    let name = val("-name").map(str::to_string).or_else(|| sources.first().cloned()).unwrap_or_default();
                    let src_obj = val("-source").and_then(|o| resolve_objs(o).into_iter().next());
                    // THE MASTER is `-master_clock`, else the clock on the `-source` pin — not the
                    // file's first clock, which gave a divide-by-2 of a 4 ns clock the period of an
                    // unrelated 10 ns one.
                    let master = match val("-master_clock") {
                        Some(m) => resolve_objs(m).into_iter().next().and_then(|m| sdc.clocks.iter().find(|c| c.name == m)),
                        None => src_obj.as_ref().and_then(|s| sdc.clocks.iter().find(|c| c.sources.iter().any(|x| x == s))),
                    };
                    let nums = |f: &str| -> Option<Vec<f64>> {
                        val(f).map(|v| list_words(v).split_whitespace().filter_map(|x| x.parse::<f64>().ok()).collect())
                    };
                    let spec = GeneratedClock {
                        source: val("-source").map(Objects::of).unwrap_or_default(),
                        master_clock: val("-master_clock").and_then(|m| resolve_objs(m).into_iter().next()),
                        combinational: has("-combinational"),
                        divide_by: val("-divide_by").and_then(|v| v.parse::<f64>().ok()).filter(|d| d.fract() == 0.0).map(|d| d as i64),
                        multiply_by: val("-multiply_by").and_then(|v| v.parse::<f64>().ok()).filter(|d| d.fract() == 0.0).map(|d| d as i64),
                        duty_cycle: val("-duty_cycle").and_then(|v| v.parse().ok()),
                        edges: nums("-edges").map(|v| v.into_iter().map(|e| e as i64).collect()).unwrap_or_default(),
                        edge_shift: nums("-edge_shift").unwrap_or_default(),
                        invert: has("-invert"),
                    };
                    let (period, waveform) = match master.and_then(|m| generate(m, &spec)) {
                        Some((p, w)) => (p, Some(w)),
                        // no master this reader can find: no usable period, which `health` reports
                        None => (0.0, None),
                    };
                    let source = sources.first().cloned().unwrap_or_default();
                    // In DEFINITION order, as the reference keeps clocks.
                    sdc.clocks.push(SdcClock { name, source, sources, period, waveform, add: has("-add"), generated: Some(spec) });
                }
                "set_input_delay" | "set_output_delay" => {
                    // `-max` / `-min` take NO value: `delay_value port_pin_list` are the two
                    // positionals. Reading `-max` as valued swallowed the port list of
                    // `set_input_delay 1 -clock clk -max [get_ports a]`.
                    let (with, flags, pos) = split_args(&toks, &["-clock", "-reference_pin", "-comment"]);
                    let read: Option<f64> = pos.first().and_then(|v| v.parse().ok());
                    let val = read.unwrap_or(0.0);
                    // ⛔ NO OBJECTS is an error to the reference (STA-0567), not "every port": read as
                    // a default it applied to every input. An EMPTY list applies to nothing.
                    let Some(named) = pos.get(1).map(|o| Objects::of(o)) else {
                        sdc.rejected.push(format!("{} without a port list", toks[0]));
                        continue;
                    };
                    let mut accessor_flags = named.flags.clone();
                    let (accessor, objs) = match pos.get(1).and_then(|o| all_ports_except(o)) {
                        Some(except) => {
                            // the flags of the `[all_inputs …]` inside the `lsearch`
                            let inner = pos[1].strip_prefix('[').and_then(|i| i.strip_suffix(']')).map(tokenize).unwrap_or_default();
                            accessor_flags = inner.iter().find(|t| t.starts_with("[all_")).map(|t| Objects::of(t).flags).unwrap_or_default();
                            except
                        }
                        None => (named.accessor, named.names),
                    };
                    // the `all_*` collections, by exact name — a pattern like `*_in` is a name
                    let default = objs.iter().any(|o| o == "*INPUTS*" || o == "*OUTPUTS*");
                    let ports: Vec<String> = objs.into_iter().filter(|o| !SENTINELS.contains(&o.as_str())).collect();
                    let valued = |f: &str| with.iter().find(|(k, _)| *k == f).and_then(|(_, v)| resolve_objs(v).into_iter().next());
                    let d = IoDelay {
                        value: val,
                        value_read: read.is_some(),
                        default,
                        ports,
                        clock: valued("-clock"),
                        flags,
                        reference_pin: valued("-reference_pin"),
                        accessor,
                        accessor_flags,
                    };
                    if toks[0] == "set_input_delay" {
                        sdc.input_delays.push(d);
                    } else {
                        sdc.output_delays.push(d);
                    }
                }
                "set_clock_uncertainty" => {
                    // ⛔ PER TARGET, LAST WRITE WINS — the reference keeps one value per clock or
                    // pin (and per clock pair for `-from`/`-to`). Folded to one maximum over every
                    // command, a later, smaller uncertainty never took effect and one clock's set
                    // every clock's. The design-wide scalar is the largest clock's FINAL value;
                    // inter-clock values stay in `uncertainties`.
                    let valued = ["-from", "-to", "-rise_from", "-fall_from", "-rise_to", "-fall_to"];
                    let (with, flags, pos) = split_args(&toks, &valued);
                    let side = |names: &[&str]| -> Vec<String> {
                        with.iter().filter(|(k, _)| names.contains(k)).flat_map(|(_, v)| resolve_objs(v)).collect()
                    };
                    let has = |f: &str| flags.iter().any(|x| x == f);
                    let (setup, hold) = (has("-setup") || !has("-hold"), has("-hold") || !has("-setup"));
                    let (Some(value), objects) = (pos.first().and_then(|v| v.parse::<f64>().ok()), pos.iter().skip(1).flat_map(|o| resolve_objs(o)).collect::<Vec<_>>()) else {
                        sdc.rejected.push("set_clock_uncertainty without a value".into());
                        continue;
                    };
                    let written = pos.iter().skip(1).map(|o| Objects::of(o)).collect();
                    let written_sides = with.iter().map(|(k, v)| (k.to_string(), Objects::of(v))).collect();
                    let u = ClockUncertainty { value, setup, hold, objects, from: side(&["-from", "-rise_from", "-fall_from"]), to: side(&["-to", "-rise_to", "-fall_to"]), written, written_sides, flags: flags.clone() };
                    if u.objects.is_empty() && u.from.is_empty() && u.to.is_empty() {
                        sdc.rejected.push("set_clock_uncertainty without objects (the reference requires them)".into());
                        continue;
                    }
                    sdc.uncertainties.push(u);
                    let fold = |hold_side: bool| -> f64 {
                        let mut last: std::collections::BTreeMap<&str, f64> = Default::default();
                        for u in &sdc.uncertainties {
                            if u.from.is_empty() && u.to.is_empty() && (if hold_side { u.hold } else { u.setup }) {
                                for o in &u.objects {
                                    last.insert(o.as_str(), u.value);
                                }
                            }
                        }
                        last.values().fold(0.0f64, |a, b| a.max(*b))
                    };
                    sdc.setup_uncertainty = fold(false);
                    sdc.hold_uncertainty = fold(true);
                }
                "set_clock_transition" => {
                    sdc.clock_transitions.extend(env_cmd(&toks).map(|e| sdc.clocks_now(e)));
                    sdc.ignored.push("set_clock_transition".to_string());
                }
                "set_clock_latency" => {
                    if let Some(e) = env_cmd(&toks).map(|e| sdc.clocks_now(e)) {
                        // `Sdc::setClockLatency`: a NETWORK latency (not `-source`) removes
                        // `set_propagated_clock` on the same clock or pin — the later one wins
                        if !e.flags.iter().any(|f| f == "-source") {
                            if let Some(w) = &e.written {
                                let targets = sdc.targets(w);
                                for p in &mut sdc.propagated {
                                    drop_targets(p, &targets, &sdc.clocks);
                                }
                                sdc.propagated.retain(|p| !p.names.is_empty());
                            }
                        }
                        sdc.clock_latencies.push(e);
                    }
                    if let Some(v) = toks.iter().skip(1).find_map(|t| {
                        if t.starts_with('-') || t.starts_with('[') {
                            None
                        } else {
                            t.parse::<f64>().ok()
                        }
                    }) {
                        sdc.clock_latency = sdc.clock_latency.max(v);
                    }
                }
                "set_input_transition" => {
                    // the value is the positional wherever it is written (`-max 0.2 …` too); the
                    // design-wide scalar is not set by a `-min`-only command
                    if let Some(e) = env_cmd(&toks) {
                        let min_only = e.flags.iter().any(|f| f == "-min") && !e.flags.iter().any(|f| f == "-max");
                        if !min_only {
                            sdc.input_transition = Some(e.value);
                        }
                        sdc.env.push(e);
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
                        let mut o = Objects::of(o);
                        // `[all_clocks]` is the clocks defined so far, not those defined later
                        if o.accessor == "all_clocks" {
                            o.names = sdc.clocks.iter().map(|c| c.name.clone()).collect();
                        }
                        // `Sdc::setPropagatedClock`: the clock or pin loses its NETWORK latency
                        // (`removeClockLatency`; a `-source` latency is an insertion, kept)
                        let targets = sdc.targets(&o);
                        for e in sdc.clock_latencies.iter_mut().filter(|e| !e.flags.iter().any(|f| f == "-source")) {
                            if let Some(w) = &mut e.written {
                                drop_targets(w, &targets, &sdc.clocks);
                                e.objects = w.names.clone();
                            }
                        }
                        sdc.clock_latencies.retain(|e| e.written.is_none() || !e.objects.is_empty());
                        sdc.propagated.push(o);
                    }
                    sdc.ignored.push("set_propagated_clock".to_string());
                }
                "set_max_delay" | "set_min_delay" => {
                    let valued = ["-from", "-to", "-through", "-rise_from", "-rise_to", "-fall_from", "-fall_to", "-rise_through", "-fall_through", "-comment"];
                    let (with, flags, pos) = split_args(&toks, &valued);
                    let value = pos.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(f64::NAN);
                    sdc.path_delays.push(PathDelayCmd {
                        max: toks[0] == "set_max_delay",
                        value,
                        objects: with.iter().filter(|(k, _)| *k != "-comment").map(|(k, v)| (k.to_string(), Objects::of(v))).collect(),
                        flags,
                    });
                    sdc.ignored.push(toks[0].clone());
                }
                "set_max_transition" => {
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_transition = Some(v),
                        _ => sdc.max_transition_on_objects = true,
                    }
                    sdc.ignored.push("set_max_transition".to_string());
                }
                "set_case_analysis" | "set_logic_zero" | "set_logic_one" | "set_logic_dc" => {
                    let pos: Vec<&String> = toks[1..].iter().filter(|t| !t.starts_with('-')).collect();
                    let (value, obj) = match toks[0].as_str() {
                        "set_case_analysis" => (pos.first().map(|v| v.to_string()), pos.get(1)),
                        "set_logic_zero" => (Some("0".into()), pos.first()),
                        "set_logic_one" => (Some("1".into()), pos.first()),
                        _ => (Some("X".into()), pos.first()),
                    };
                    match (value, obj) {
                        (Some(value), Some(obj)) => {
                            let inner = obj.strip_prefix('[').map(|i| tokenize(i.trim_end_matches(']').trim())).unwrap_or_default();
                            let c = LogicValueCmd { cmd: toks[0].clone(), value, accessor: inner.first().cloned().unwrap_or_default(), objects: resolve_objs(obj), written: Objects::of(obj) };
                            if toks[0] == "set_case_analysis" { sdc.case_values.push(c) } else { sdc.logic_values.push(c) }
                        }
                        _ => sdc.rejected.push(format!("{} without a value and objects", toks[0])),
                    }
                    sdc.ignored.push(toks[0].clone());
                }
                "set_driving_cell" => {
                    sdc.driving_cells.push(driving_cell(&toks));
                    sdc.ignored.push("set_driving_cell".to_string());
                }
                "set_load" => {
                    // ⛔ The design-wide load is a PORT pin load: not a net's load, not a `-min`-only
                    // value, not a `-wire_load`. Taken from the last `set_load` of any kind, a net
                    // load late in the file became every output's load.
                    if let Some(e) = env_cmd(&toks) {
                        let has = |f: &str| e.flags.iter().any(|x| x == f);
                        let port_pin_load = e.accessor != "get_nets"
                            && !(has("-min") && !has("-max"))
                            && !(has("-wire_load") && !has("-pin_load"));
                        if port_pin_load {
                            sdc.load = Some(e.value);
                        }
                        sdc.env.push(e);
                    }
                }
                "set_timing_derate" => {
                    // ⛔ `-early` / `-late` are FLAGS; the value is the positional. Read as valued,
                    // `set_timing_derate -late 1.05` was fine by luck and `-early -cell_delay 0.95`
                    // set nothing, while an object-scoped `-late 1.1 [get_cells u1]` became the
                    // design's late derate.
                    let (_, flags, pos) = split_args(&toks, &[]);
                    let has = |f: &str| flags.iter().any(|x| x == f);
                    let Some(value) = pos.first().and_then(|v| v.parse::<f64>().ok()) else {
                        sdc.rejected.push("set_timing_derate without a value".into());
                        continue;
                    };
                    let objects: Vec<String> = pos.iter().skip(1).flat_map(|o| resolve_objs(o)).collect();
                    let written = pos.get(1).map(|o| Objects::of(o));
                    let scoped = !objects.is_empty()
                        || ["-cell_delay", "-net_delay", "-cell_check", "-data", "-clock", "-rise", "-fall"].iter().any(|f| has(f));
                    if scoped {
                        sdc.ignored.push("set_timing_derate".into());
                    } else {
                        let (early, late) = (has("-early") || !has("-late"), has("-late") || !has("-early"));
                        if late {
                            sdc.late_derate = Some(value);
                        }
                        if early {
                            sdc.early_derate = Some(value);
                        }
                    }
                    sdc.derates.push(EnvCmd { cmd: toks[0].clone(), value, flags, clock: None, accessor: String::new(), accessor_flags: Vec::new(), objects, written });
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
                    let kind = ["-asynchronous", "-logically_exclusive", "-physically_exclusive"]
                        .iter()
                        .find(|f| has_flag(&toks, f))
                        .map(|f| f.trim_start_matches('-').to_string());
                    match kind {
                        Some(kind) => {
                            let name = flag_val(&toks, "-name").and_then(|n| resolve_objs(n).into_iter().next());
                            // `Sdc::makeClockGroups`: a name already defined is REPLACED
                            if name.is_some() {
                                sdc.clock_groups.retain(|g| g.name != name);
                            }
                            sdc.clock_groups.push(ClockGroups { name, kind, allow_paths: has_flag(&toks, "-allow_paths"), groups: groups.clone() })
                        }
                        // a type is required (the reference's STA-0400)
                        None => sdc.rejected.push("set_clock_groups without -asynchronous / -logically_exclusive / -physically_exclusive".into()),
                    }
                    sdc.async_groups.extend(groups);
                }
                "set_false_path" => {
                    let (from, to, refused) = from_to(&toks);
                    let (args, flags) = exception_args(&toks);
                    sdc.exception_objects.push(exception_sides(&toks));
                    sdc.exceptions.push(Exception { kind: ExcKind::FalsePath, from, to, refused, args, flags, ..Default::default() });
                }
                "set_multicycle_path" => {
                    // The multiplier is THE positional left after the flags, wherever it is
                    // written (`-setup 3 -from …` as often as `3 -setup -from …`); it must be an
                    // integer, as the reference requires. `-hold` makes it a hold multicycle, and
                    // `-start` / `-end` say which clock it counts — `-end` for setup and `-start`
                    // for hold when neither is given, as the reference defaults them.
                    let (_, flags, pos) = split_args(&toks, &EXC_VALUED);
                    let (from, to, mut refused) = from_to(&toks);
                    let n = match pos.as_slice() {
                        [v] => v.parse::<u32>().ok(),
                        _ => None,
                    };
                    if n.is_none() && refused.is_none() {
                        refused = Some(format!("multiplier {:?} is not one integer", pos));
                    }
                    let has = |f: &str| flags.iter().any(|x| x == f);
                    let hold = has("-hold");
                    let (args, written_flags) = exception_args(&toks);
                    sdc.exception_objects.push(exception_sides(&toks));
                    sdc.exceptions.push(Exception {
                        kind: ExcKind::Multicycle(n.unwrap_or(1)),
                        from,
                        to,
                        hold,
                        start: has("-start") || (hold && !has("-end")),
                        refused,
                        args,
                        flags: written_flags,
                    });
                }
                other => sdc.ignored.push(other.to_string()),
            }
        }

        Ok(sdc)
    }

    pub fn load(path: &str) -> Result<Sdc, SdcError> {
        Sdc::parse(&read_lossy(path)?)
    }

    /// [`Sdc::parse`] against a linked design, as the reference reads constraints after linking:
    /// object queries (`-filter`, `-of_objects`) are evaluated on `design`, by the reference's
    /// rules, instead of refusing their commands. What the design view cannot
    /// answer still refuses, in [`Sdc::unevaluated_queries`].
    pub fn parse_with(text: &str, design: &SdcDesign) -> Result<Sdc, SdcError> {
        let _scope = DesignScope::enter(design);
        Sdc::parse(text)
    }

    /// [`Sdc::load`] against a linked design — see [`Sdc::parse_with`].
    pub fn load_with(path: &str, design: &SdcDesign) -> Result<Sdc, SdcError> {
        Sdc::parse_with(&read_lossy(path)?, design)
    }
}

/// A constraint file's text. A byte that is not UTF-8 is read lossily, as the reference's Tcl
/// `source` decodes it instead of refusing the file: such bytes sit in comments (a Latin-1 `©` in
/// a vendor header), and refusing them dropped every constraint in the file.
fn read_lossy(path: &str) -> Result<String, SdcError> {
    let bytes = std::fs::read(path).map_err(|e| SdcError(format!("{path}: {e}")))?;
    Ok(match String::from_utf8(bytes) {
        Ok(s) => s,
        Err(e) => String::from_utf8_lossy(e.as_bytes()).into_owned(),
    })
}

pub use crate::sdc_query::SdcDesign;

// The design a `parse_with` reads against, for the queries resolved deep in the command parsers.
// Set only for the length of that call (the guard clears it, on unwind too), on this thread.
thread_local! {
    static DESIGN: std::cell::Cell<*const SdcDesign> = const { std::cell::Cell::new(std::ptr::null()) };
}

struct DesignScope(*const SdcDesign);

impl DesignScope {
    fn enter(d: &SdcDesign) -> DesignScope {
        DesignScope(DESIGN.with(|c| c.replace(d as *const SdcDesign)))
    }
}

impl Drop for DesignScope {
    fn drop(&mut self) {
        DESIGN.with(|c| c.set(self.0));
    }
}

/// The query evaluated on the design in scope: `None` without one.
fn with_design<T>(f: impl FnOnce(&SdcDesign) -> T) -> Option<T> {
    let p = DESIGN.with(|c| c.get());
    // SAFETY: `p` is set only by `DesignScope::enter` from a reference that outlives the scope,
    // and cleared when the scope ends; it is read on the same thread inside that scope.
    (!p.is_null()).then(|| f(unsafe { &*p }))
}

/// A bracketed query's words, abbreviations expanded.
pub(crate) fn query_words(inner: &str) -> Vec<String> {
    expand_abbreviations(tokenize(inner.trim()))
}

/// Extract `-from`/`-to` object names (first object of each), `*` if absent.
/// A pin object (`reg/Q`) is reduced to its instance (`reg`) so it matches the
/// engine's instance-level exception matching; a port keeps its name.
/// The flags of an exception command that take a value.
const EXC_VALUED: [&str; 10] =
    ["-from", "-to", "-through", "-rise_from", "-rise_to", "-fall_from", "-fall_to", "-rise_through", "-fall_through", "-comment"];

/// An exception's two sides, and why it cannot be applied at instance level, if it cannot.
fn from_to(toks: &[String]) -> (Vec<String>, Vec<String>, Option<String>) {
    let [from, to] = exception_sides(toks);
    let mut refused = None;
    if ["-through", "-rise_through", "-fall_through"].iter().any(|f| has_flag(toks, f)) {
        refused = Some("a -through list, which an instance-level match cannot express".to_string());
    }
    let (f, rf) = reduce_exception_objects(&from);
    let (t, rt) = reduce_exception_objects(&to);
    (f, t, refused.or(rf).or(rt))
}

/// An exception's object-valued flags as written, in order, and its flags without a value.
fn exception_args(toks: &[String]) -> (Vec<(String, Objects)>, Vec<String>) {
    let (with, flags, _) = split_args(toks, &EXC_VALUED);
    (with.into_iter().filter(|(k, _)| *k != "-comment").map(|(k, v)| (k.to_string(), Objects::of(v))).collect(), flags)
}

/// An exception's objects per side: every `-from` / `-rise_from` / `-fall_from`, then every `-to`
/// / `-rise_to` / `-fall_to`, accessor kept.
fn exception_sides(toks: &[String]) -> [Vec<Objects>; 2] {
    let pick = |flags: &[&str]| -> Vec<Objects> { flags.iter().filter_map(|f| flag_val(toks, f)).map(|v| Objects::of(v)).collect() };
    [pick(&["-from", "-rise_from", "-fall_from"]), pick(&["-to", "-rise_to", "-fall_to"])]
}

/// The names the `all_*` accessors resolve to — collections, not objects.
const SENTINELS: [&str; 3] = ["*INPUTS*", "*OUTPUTS*", "*REGS*"];

/// One side of [`Exception`] from its objects, and a refusal if the side cannot be expressed at
/// instance level. ALL objects each flag names, not the first: a `-from [list a b c]` that cut only
/// `a` left `b` and `c` timed against the SDC's stated intent.
fn reduce_exception_objects(objs: &[Objects]) -> (Vec<String>, Option<String>) {
    let mut out: Vec<String> = Vec::new();
    let mut refused = None;
    for name in objs.iter().flat_map(|o| o.names.iter()) {
        // ⛔ A COLLECTION is one of these exact names — not anything starting with `*`, which
        // dropped `*sync_rst*` as though it named nothing, leaving the side "any".
        if SENTINELS.contains(&name.as_str()) {
            continue;
        }
        // A wildcard PIN pattern (`*/SETN`, `u_core/*/D`) names pins of many instances: reduced
        // to its instance part it would match instances whatever the pin, and `*/SETN` became
        // every path. Refused; a wildcard instance or port name stays a pattern.
        if name.contains(['*', '?']) && name.contains('/') {
            refused = Some(format!("the pin pattern `{name}`, which an instance-level match cannot express"));
            continue;
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
    (out, refused)
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

    // `lsearch -not -exact` drops the inputs EQUAL to the pattern: one port is removed; a pattern
    // of two ports equals no element and is not read as an exclusion.
    #[test]
    fn every_input_but_the_clock_port() {
        let t = "set clk_port [get_ports clk]\ncreate_clock -name clk -period 690 $clk_port\n\
                 set non_clock_inputs [lsearch -inline -all -not -exact [all_inputs] $clk_port]\n\
                 set_input_delay 138 -clock clk $non_clock_inputs\n";
        let s = Sdc::parse(t).unwrap();
        let d = &s.input_delays[0];
        assert_eq!((d.accessor.as_str(), d.ports.clone(), d.default), ("all_inputs_except", vec!["clk".to_string()], false));
        assert_eq!(all_ports_except("[lsearch -inline -all -not -exact [all_inputs] [get_ports {a b}]]"), None);
        assert_eq!(all_ports_except("[lsearch -inline -all -exact [all_inputs] [get_ports a]]"), None);
        assert_eq!(all_ports_except("[lsearch -all -inline -exact -not [all_outputs] [get_ports y]]"), Some(("all_outputs_except".into(), vec!["y".into()])));
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

    // Rule: set_case_analysis and set_logic_* keep their value and objects, in file order, and
    // stay in `ignored`; only 0/zero and 1/one hold a pin constant.
    #[test]
    fn case_and_logic_values_are_kept_as_written() {
        let s = Sdc::parse("set_case_analysis 0 [get_ports {kld}]\nset_case_analysis rise [get_pins u1/A]\nset_logic_one [get_ports {a b}]\n").unwrap();
        let c: Vec<(&str, &str, Vec<String>, Option<bool>)> = s.case_values.iter().map(|c| (c.value.as_str(), c.accessor.as_str(), c.objects.clone(), c.constant())).collect();
        assert_eq!(c, vec![("0", "get_ports", vec!["kld".to_string()], Some(false)), ("rise", "get_pins", vec!["u1/A".to_string()], None)]);
        assert_eq!((s.logic_values[0].constant(), s.logic_values[0].objects.clone()), (Some(true), vec!["a".to_string(), "b".to_string()]));
        assert!(s.ignored_affecting_timing().contains(&"set_case_analysis"));
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
        // no port list is rejected (the reference's STA-0567), not read as every input
        // and an undefined variable is an error (Tcl's), not a value of 0: rejected, and listed
        // with the queries not evaluated, which every consumer refuses on
        let d = Sdc::parse("set_input_delay 1 -clock clk\nset_input_delay $undefined -clock clk [get_ports a]\n").unwrap();
        assert_eq!(d.rejected.len(), 2);
        assert!(d.input_delays.is_empty() && d.unevaluated_queries == ["set_input_delay: $undefined (not defined)"]);
        // a value that is not a number is still kept as unread
        let d = Sdc::parse("set_input_delay abc -clock clk [get_ports a]\n").unwrap();
        assert!(!d.input_delays[0].value_read && d.input_delays[0].value == 0.0);
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
        assert_eq!(s.propagated, vec![Objects { accessor: "get_clocks".into(), names: vec!["clk".into()], ..Default::default() }]);
        assert_eq!((s.clock_latencies.len(), s.clock_latencies[0].flags.clone(), s.clock_latency), (1, vec!["-source".to_string()], 0.2));
        let p = &s.path_delays[0];
        assert_eq!((p.max, p.value, p.flags.clone()), (true, 0.5, vec!["-ignore_clock_latency".to_string()]));
        assert_eq!(p.objects, vec![("-from".to_string(), Objects { accessor: "get_pins".into(), names: vec!["r1/CK".into()], ..Default::default() }), ("-to".to_string(), Objects { accessor: "get_clocks".into(), names: vec!["clk".into()], ..Default::default() })]);
        // `$undefined` is not a value: the command is not kept (see `unresolved_var`)
        assert_eq!(s.path_delays.len(), 1);
        assert_eq!(s.unevaluated_queries, ["set_min_delay: $undefined (not defined)"]);
        assert!(s.ignored.iter().any(|c| c == "set_propagated_clock") && s.ignored.iter().filter(|c| c.ends_with("_delay")).count() == 1);
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

/// How the file is split into commands — Tcl's rules, as the reference reads it.
#[cfg(test)]
mod tcl_structure_tests {
    use super::*;

    /// A control structure is ONE command, not run: the body of `if {0}` creates nothing.
    #[test]
    fn a_control_structure_is_not_run_line_by_line() {
        let s = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             if {0} {\n  create_clock -name bogus -period 1 [get_ports b]\n}\n\
             foreach p {a b} {\n  set_input_delay 1 -clock clk [get_ports $p]\n}\n",
        )
        .unwrap();
        assert_eq!(s.clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["clk"]);
        // `if {0}` is evaluated (false) and its body skipped; the foreach over a literal list runs
        assert_eq!(s.input_delays.iter().flat_map(|d| d.ports.clone()).collect::<Vec<_>>(), ["a", "b"]);
    }

    /// A brace list may run across lines; a `;#` comment and a continued line are Tcl's.
    #[test]
    fn a_brace_list_across_lines_and_comments() {
        let s = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports {clk}] ;# the clock\n\
             set_input_delay 1 -clock clk [get_ports {in1\n   in2}]\n\
             # set_input_delay 9 -clock clk [get_ports nope]\n\
             set_output_delay 2 \\\n    -clock clk [get_ports out]\n",
        )
        .unwrap();
        assert_eq!(s.clocks.len(), 1);
        assert_eq!(s.input_delays.len(), 1, "the commented-out delay is not read");
        assert_eq!(s.input_delays[0].ports, ["in1", "in2"]);
        assert_eq!(s.output_delays[0].ports, ["out"]);
    }

    /// An unbraced bus bit is one name: `a[0]`, as the reference keeps it.
    #[test]
    fn an_unbraced_bus_bit_is_one_name() {
        let s = Sdc::parse("create_clock -name clk -period 10 [get_ports clk]\nset_input_delay 1 -clock clk [get_ports a[0]]\n").unwrap();
        assert_eq!(s.input_delays[0].ports, ["a[0]"]);
    }
}

/// Tcl `if` / `foreach` / `expr`, evaluated where the reference's Tcl would evaluate them.
#[cfg(test)]
mod tcl_eval_tests {
    use super::*;

    fn ev(s: &str) -> Option<String> {
        eval_expr(s, &HashMap::new()).map(|v| v.text())
    }

    #[test]
    fn expr_follows_tcl_arithmetic() {
        assert_eq!(ev("10 / 4").as_deref(), Some("2"), "integer division");
        assert_eq!(ev("-7 / 2").as_deref(), Some("-4"), "toward negative infinity");
        assert_eq!(ev("-7 % 2").as_deref(), Some("1"), "the divisor's sign");
        assert_eq!(ev("10.0 / 4").as_deref(), Some("2.5"));
        assert_eq!(ev("2 * 1.5").as_deref(), Some("3.0"));
        assert_eq!(ev("(1 + 2) * 3 - 1").as_deref(), Some("8"));
        assert_eq!(ev("max(1, 2.5, 2)").as_deref(), Some("2.5"));
        assert_eq!(ev("1 < 2 && \"a\" eq \"a\"").as_deref(), Some("1"));
        assert_eq!(ev("[get_ports x] > 0"), None, "a design query is not evaluated");
    }

    /// The idiom every LibreLane SDC uses: the flow's value when it is set, else a default.
    /// Only the CHOSEN branch runs.
    #[test]
    fn if_runs_only_the_branch_its_condition_chooses() {
        let src = "if { [info exists ::env(VYGES_TEST_UNSET_PERIOD_X)] } {\n  set p $::env(VYGES_TEST_UNSET_PERIOD_X)\n} else {\n  set p 15.0\n}\n\
                   set io [expr {$p * 0.20}]\n\
                   create_clock -name clk -period $p [get_ports clk]\n\
                   set_input_delay $io -clock clk [get_ports in]\n";
        let s = Sdc::parse(src).unwrap();
        assert_eq!(s.clocks[0].period, 15.0);
        assert!(s.input_delays[0].value_read);
        assert_eq!(s.input_delays[0].value, 15.0 * 0.20);
        let s = Sdc::parse("set a 1\nif {$a == 2} { create_clock -name x -period 1 } elseif {$a == 1} { create_clock -name y -period 2 }\n").unwrap();
        assert_eq!(s.clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["y"]);
    }

    /// `foreach` over a literal list runs its body per element; over a design query it is
    /// refused, not run once.
    #[test]
    fn foreach_over_a_literal_list() {
        let s = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             foreach p {a b} { set_input_delay 1 -clock clk [get_ports $p] }\n\
             foreach q [all_inputs] { set_input_delay 2 -clock clk $q }\n",
        )
        .unwrap();
        let ports: Vec<String> = s.input_delays.iter().flat_map(|d| d.ports.clone()).collect();
        assert_eq!(ports, ["a", "b"]);
        assert_eq!(s.ignored_affecting_timing(), ["foreach"]);
    }
}

/// `set_units` asserts the library's units; it does not rescale the file — the reference's rule.
#[cfg(test)]
mod units_tests {
    use super::*;

    #[test]
    fn set_units_does_not_scale_values() {
        let s = Sdc::parse("set_units -time ps -capacitance fF\ncreate_clock -name c -period 10 [get_ports clk]\nset_input_delay 2 -clock c [get_ports a]\nset_load 3 [get_ports y]\n").unwrap();
        assert_eq!(s.clocks[0].period, 10.0);
        assert_eq!(s.input_delays[0].value, 2.0);
        assert_eq!(s.load, Some(3.0));
    }
}

/// Exceptions as the reference applies them — and never "every path" by accident.
#[cfg(test)]
mod exception_rule_tests {
    use super::*;

    fn exc(cmd: &str) -> Exception {
        Sdc::parse(&format!("create_clock -name c -period 10 [get_ports clk]\n{cmd}\n")).unwrap().exceptions.remove(0)
    }

    /// A `-through` list and a wildcard pin pattern cannot be expressed at instance level: they
    /// are refused — not applied — rather than read as `* -> *`.
    #[test]
    fn an_exception_that_cannot_be_expressed_is_not_applied_to_every_path() {
        for cmd in ["set_false_path -through [get_pins u1/Z]", "set_false_path -to [get_pins */SETN]"] {
            let e = exc(cmd);
            assert!(e.refused.is_some(), "{cmd}");
            assert!(!e.covers("r1", "r2"), "{cmd} must not cut an unrelated path");
        }
        // a name that merely starts with `*` is a pattern, not a collection
        let e = exc("set_false_path -from [get_ports *sync_rst*]");
        assert_eq!(e.from, ["*sync_rst*"]);
        assert!(e.covers("a_sync_rst_n", "r2") && !e.covers("data_in", "r2"));
        // a plain bus pattern on ports globs
        assert!(exc("set_false_path -from [get_ports in*]").covers("in1", "x"));
    }

    /// The multiplier is the positional wherever it is written; `-hold` / `-start` / `-end`.
    #[test]
    fn a_multicycle_keeps_its_multiplier_check_and_clock() {
        let e = exc("set_multicycle_path -setup 3 -from [get_cells a] -to [get_cells b]");
        assert_eq!((e.kind.clone(), e.hold, e.start), (ExcKind::Multicycle(3), false, false));
        let e = exc("set_multicycle_path -hold 2 -from [get_cells a]");
        assert_eq!((e.kind.clone(), e.hold, e.start), (ExcKind::Multicycle(2), true, true));
        let e = exc("set_multicycle_path 2 -setup -start -to [get_cells b]");
        assert_eq!((e.kind.clone(), e.hold, e.start), (ExcKind::Multicycle(2), false, true));
        assert!(exc("set_multicycle_path 2.0 -to [get_cells b]").refused.is_some(), "not an integer");
    }
}

/// Generated clocks from their master, as the reference's `Clock::generate` builds them; the
/// values are the reference's own, measured on the same commands.
#[cfg(test)]
mod generated_clock_tests {
    use super::*;

    #[test]
    fn a_generated_clock_is_built_from_the_clock_on_its_source() {
        let s = Sdc::parse(
            "create_clock -name c1 -period 10 -waveform {2 7} [get_ports clk1]\n\
             create_clock -name c2 -period 4 [get_ports clk2]\n\
             create_generated_clock -name g_div -source [get_ports clk2] -divide_by 2 [get_pins r3/CK]\n\
             create_generated_clock -name g_inv -source [get_ports clk1] -divide_by 1 -invert [get_pins r2/Q]\n\
             create_generated_clock -name g_edge -source [get_ports clk1] -edges {1 3 5} [get_pins u1/Z]\n\
             create_generated_clock -name g_d2 -source [get_ports clk1] -divide_by 2 [get_pins r4/CK]\n\
             create_clock -name c3 -period 3 [get_ports clk3]\n",
        )
        .unwrap();
        let get = |n: &str| s.clocks.iter().find(|c| c.name == n).map(|c| (c.period, c.waveform.clone().unwrap_or_default())).unwrap();
        assert_eq!(get("g_div"), (8.0, vec![0.0, 4.0]), "the master is clk2's clock, not the first");
        assert_eq!(get("g_inv"), (10.0, vec![7.0, 12.0]));
        assert_eq!(get("g_edge"), (20.0, vec![2.0, 12.0]));
        assert_eq!(get("g_d2"), (20.0, vec![2.0, 12.0]), "a power-of-two divide keeps the rise");
        let order: Vec<&str> = s.clocks.iter().map(|c| c.name.as_str()).collect();
        assert_eq!(order, ["c1", "c2", "g_div", "g_inv", "g_edge", "g_d2", "c3"], "definition order");
        // no master this reader can find: no usable period, reported
        let s = Sdc::parse("create_generated_clock -name g -source [get_pins u/Q] -divide_by 2 [get_pins r/CK]\n").unwrap();
        assert_eq!(s.clocks[0].period, 0.0);
        assert!(s.health().unwrap().contains("no usable period"));
    }
}

/// I/O delays name what they constrain: no list is rejected, an empty list is nothing, a
/// pattern is a pattern.
#[cfg(test)]
mod io_delay_object_tests {
    use super::*;

    #[test]
    fn an_io_delay_names_its_ports() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock c\n\
             set_input_delay 2 -clock c [list]\n\
             set_input_delay 3 -clock c [get_ports *_in]\n\
             set_input_delay 4 -clock c [all_inputs]\n",
        )
        .unwrap();
        assert_eq!(s.rejected.len(), 1, "no port list is an error");
        let d: Vec<(f64, bool, Vec<String>)> = s.input_delays.iter().map(|d| (d.value, d.default, d.ports.clone())).collect();
        assert_eq!(d, [(2.0, false, vec![]), (3.0, false, vec!["*_in".to_string()]), (4.0, true, vec![])]);
        assert!(object_matches("*_in", "a_in") && !object_matches("*_in", "a_out") && object_matches("a", "a"));
    }
}

/// Uncertainty, derates, transitions and loads per command — the values are the reference's, on
/// the same commands.
#[cfg(test)]
mod per_command_tests {
    use super::*;

    #[test]
    fn uncertainty_is_per_clock_and_last_write_wins() {
        let s = Sdc::parse(
            "create_clock -name c1 -period 10 [get_ports clk1]\ncreate_clock -name c2 -period 5 [get_ports clk2]\n\
             set_clock_uncertainty 0.5 [get_clocks c1]\nset_clock_uncertainty 0.2 [get_clocks c1]\n\
             set_clock_uncertainty -setup 0.3 [get_clocks c2]\n\
             set_clock_uncertainty -from [get_clocks c1] -to [get_clocks c2] -hold 0.9\n\
             set_clock_uncertainty 0.7\n",
        )
        .unwrap();
        // reference: c1 0.2 (the later write), c2 setup 0.3, c1->c2 hold 0.9 between clocks
        assert_eq!((s.setup_uncertainty, s.hold_uncertainty), (0.3, 0.2), "not the maximum over commands");
        let inter = s.uncertainties.iter().find(|u| !u.from.is_empty()).unwrap();
        assert_eq!((inter.value, inter.hold, inter.setup, inter.to.clone()), (0.9, true, false, vec!["c2".to_string()]));
        assert_eq!(s.rejected.len(), 1, "no objects is an error");
    }

    #[test]
    fn derate_flags_take_no_value_and_scoped_derates_are_not_global() {
        let s = Sdc::parse(
            "set_timing_derate 1.05 -late\nset_timing_derate -early -cell_delay 0.95\n\
             set_timing_derate -late 1.1 [get_cells u1]\nset_timing_derate -early -net_delay 0.9\n",
        )
        .unwrap();
        assert_eq!((s.late_derate, s.early_derate), (Some(1.05), None));
        let v: Vec<f64> = s.derates.iter().map(|d| d.value).collect();
        assert_eq!(v, [1.05, 0.95, 1.1, 0.9]);
        assert_eq!(s.derates[2].objects, ["u1"]);
        assert!(s.ignored_affecting_timing().contains(&"set_timing_derate"));
    }

    #[test]
    fn a_transition_or_load_value_is_the_positional_and_a_net_load_is_not_the_port_load() {
        let s = Sdc::parse("set_input_transition -max 0.2 [all_inputs]\nset_input_transition -min 0.05 [all_inputs]\nset_load 3 [get_ports out]\nset_load 9 [get_nets n1]\nset_load -min 1 [get_ports out]\n").unwrap();
        assert_eq!(s.input_transition, Some(0.2));
        assert_eq!(s.load, Some(3.0));
        assert_eq!(s.env.len(), 5);
    }
}

#[cfg(test)]
mod clock_group_tests {
    use super::*;

    /// Clocks are separated only by the command that separates them; one group separates its
    /// clocks from all others; `-allow_paths` separates nothing.
    #[test]
    fn clock_groups_are_per_command() {
        let s = Sdc::parse(
            "set_clock_groups -asynchronous -group [get_clocks c1] -group [get_clocks c2]\n\
             set_clock_groups -asynchronous -group [get_clocks c1] -group [get_clocks c3]\n\
             set_clock_groups -logically_exclusive -group [get_clocks c4]\n\
             set_clock_groups -asynchronous -allow_paths -group [get_clocks c5] -group [get_clocks c6]\n",
        )
        .unwrap();
        let u = |a, b| clocks_unrelated(&s.clock_groups, a, b);
        assert!(u("c1", "c2") && u("c1", "c3"));
        assert!(!u("c2", "c3"), "no command separated c2 from c3");
        assert!(u("c4", "c1") && u("c2", "c4"), "a single group separates its clock from every other");
        assert!(!u("c5", "c6"), "-allow_paths");
        assert_eq!(s.clock_groups[2].kind, "logically_exclusive");
    }
}

#[cfg(test)]
mod abbreviation_tests {
    use super::*;

    /// A flag is the FIRST key (else flag) it begins, in the reference's order; a command name the
    /// one SDC command it begins.
    #[test]
    fn abbreviations_expand_as_the_reference_expands_them() {
        let s = Sdc::parse("create_clock -n c -per 10 -wave {0 5} [get_ports clk]\nset_max_trans 0.5 [current_design]\nset_input_delay 1 -cl c -ma [get_ports a]\n").unwrap();
        assert_eq!((s.clocks[0].name.as_str(), s.clocks[0].period), ("c", 10.0));
        assert_eq!(s.clocks[0].waveform, Some(vec![0.0, 5.0]));
        assert_eq!(s.max_transition, Some(0.5));
        assert_eq!((s.input_delays[0].clock.as_deref(), s.input_delays[0].flags.clone()), (Some("c"), vec!["-max".to_string()]));
    }
}

#[cfg(test)]
mod object_queries {
    use super::*;

    /// A query's patterns are its positional arguments: a key takes the next word (abbreviated or
    /// not), a flag none. The filter expression or -hsc separator is never read as a name.
    #[test]
    fn a_query_key_value_is_not_an_object_name() {
        assert_eq!(resolve_objs("[get_ports -quiet {a b}]"), vec!["a", "b"]);
        assert_eq!(resolve_objs("[get_pins -hsc / u1/A]"), vec!["u1/A"]);
        assert_eq!(resolve_objs("[get_cells -hier -hs / top*]"), vec!["top*"]);
    }

    /// -filter, -of_objects and -regexp need the design: the command is not applied, and is
    /// listed — never matched as names (it was: `ref_name==DFF` and `[get_nets` became names).
    #[test]
    fn a_query_that_needs_the_design_refuses_its_command() {
        let s = Sdc::parse(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_input_delay 1 -clock clk [get_ports -filter \"direction==in\" *]\n\
             set_false_path -to [get_pins -of_obj [get_nets n1]]\n\
             set_load 0.1 [get_ports -regexp {out_[0-9]+}]\n\
             set ports [get_ports -filter {is_clock==false}]\n\
             set_output_delay 2 -clock clk [list [get_ports o1] [get_ports -filt x *]]\n",
        )
        .unwrap();
        assert_eq!(s.clocks.len(), 1, "an ordinary query still applies");
        assert!(s.input_delays.is_empty() && s.output_delays.is_empty() && s.exceptions.is_empty() && s.load.is_none());
        assert_eq!(s.unevaluated_queries.len(), 4, "{:?}", s.unevaluated_queries);
        assert!(s.unevaluated_queries[1].starts_with("set_false_path: [get_pins -of_obj"));
        assert!(s.health().unwrap().contains("needs the design"));
    }
}

#[cfg(test)]
mod clock_transition_kept {
    use super::*;

    #[test]
    fn set_clock_transition_is_kept_per_command() {
        let s = Sdc::parse("create_clock -name c -period 1 [get_ports clk]\nset_clock_transition -rise 0.04 [get_clocks c]\nset_clock_transition 0.05 [all_clocks]\n").unwrap();
        assert_eq!(s.clock_transitions.len(), 2);
        assert_eq!((s.clock_transitions[0].value, s.clock_transitions[0].flags.clone(), s.clock_transitions[0].objects.clone()), (0.04, vec!["-rise".to_string()], vec!["c".to_string()]));
        assert!(s.ignored.iter().any(|c| c == "set_clock_transition"));
    }

    // Rule (reference: Tcl `source` decodes the file): a byte that is not UTF-8 does not refuse the
    // file. A contest SDC carries a Latin-1 `©` (0xA9) in its header comment; read strictly, every
    // constraint after it was lost.
    #[test]
    fn a_byte_that_is_not_utf8_does_not_refuse_the_file() {
        let dir = std::env::temp_dir().join(format!("loom-sdc-latin1-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("c.sdc");
        let mut bytes = b"# Copyright \xa9 2011\n".to_vec();
        bytes.extend_from_slice(b"create_clock -name mclk -period 300.0 [get_ports clk]\n");
        std::fs::write(&p, bytes).unwrap();
        let s = Sdc::load(p.to_str().unwrap()).unwrap();
        assert_eq!((s.clocks.len(), s.clocks[0].period), (1, 300.0));
        let with = Sdc::load_with(p.to_str().unwrap(), &SdcDesign::default()).unwrap();
        assert_eq!(with.clocks.len(), 1);
        std::fs::remove_dir_all(&dir).ok();
    }
}

#[cfg(test)]
mod clock_state_rules {
    use super::*;

    /// Rule (`Sdc::setPropagatedClock` → `removeClockLatency`; `Sdc::setClockLatency` →
    /// `removePropagatedClock`): a network latency and propagation on the SAME clock or pin cancel —
    /// the later command wins. A `-source` latency is an insertion and survives both. Kept both, a
    /// writer emitting them in kind order flipped which one won.
    #[test]
    fn propagation_and_network_latency_on_one_object_cancel_last_wins() {
        let s = Sdc::parse(
            "create_clock -name c -period 1 [get_ports clk]\n\
             set_clock_latency 0.1 [get_clocks c]\n\
             set_clock_latency -source 0.2 [get_clocks c]\n\
             set_clock_latency 0.3 [get_pins r/CK]\n\
             set_propagated_clock [get_clocks c]\n",
        )
        .unwrap();
        let left: Vec<(f64, Vec<String>)> = s.clock_latencies.iter().map(|e| (e.value, e.objects.clone())).collect();
        assert_eq!(left, vec![(0.2, vec!["c".to_string()]), (0.3, vec!["r/CK".to_string()])], "the clock's network latency went");
        let s = Sdc::parse("create_clock -name c -period 1 [get_ports clk]\nset_propagated_clock [get_clocks c]\nset_clock_latency 0.1 [get_clocks c]\n").unwrap();
        assert!(s.propagated.is_empty(), "a later network latency makes the clock ideal again");
        let s = Sdc::parse("create_clock -name c -period 1 [get_ports clk]\nset_propagated_clock [get_clocks c]\nset_clock_latency -source 0.1 [get_clocks c]\n").unwrap();
        assert_eq!(s.propagated.len(), 1, "a source latency leaves propagation alone");
    }

    /// Rule (Tcl: the command's argument is evaluated when the command runs): `[all_clocks]` names
    /// the clocks defined SO FAR. Written back after every clock, it named later clocks too.
    #[test]
    fn all_clocks_names_the_clocks_defined_so_far() {
        let s = Sdc::parse("create_clock -name a -period 1 [get_ports x]\nset_propagated_clock [all_clocks]\ncreate_clock -name b -period 2 [get_ports y]\n").unwrap();
        assert_eq!(s.propagated[0].names, vec!["a".to_string()]);
        assert_eq!(s.propagated[0].accessor, "all_clocks", "the accessor stays as written");
    }

    /// Rule (`Sdc::makeClockGroups`): a group of a name already defined REPLACES it; an unnamed one
    /// is named `group<n>` by the reference, so it is kept unnamed and in order.
    #[test]
    fn a_named_clock_group_replaces_its_namesake() {
        let s = Sdc::parse(
            "create_clock -name a -period 1 [get_ports x]\ncreate_clock -name b -period 2 [get_ports y]\n\
             set_clock_groups -name g -asynchronous -group a -group b\n\
             set_clock_groups -asynchronous -group a\n\
             set_clock_groups -name g -logically_exclusive -group a -group b\n",
        )
        .unwrap();
        let got: Vec<(Option<String>, String)> = s.clock_groups.iter().map(|g| (g.name.clone(), g.kind.clone())).collect();
        assert_eq!(got, vec![(None, "asynchronous".to_string()), (Some("g".to_string()), "logically_exclusive".to_string())]);
    }

    /// The `[all_inputs …]` inside the `lsearch … -not -exact` form keeps ITS flags, not lsearch's.
    #[test]
    fn all_inputs_except_keeps_the_inner_flags() {
        let s = Sdc::parse("create_clock -name c -period 1 [get_ports clk]\nset_input_delay 1 -clock c [lsearch -inline -all -not -exact [all_inputs -no_clocks] clk]\n").unwrap();
        assert_eq!(s.input_delays[0].accessor_flags, vec!["-no_clocks".to_string()]);
    }
}

#[cfg(test)]
mod unevaluable_is_refused {
    use super::*;

    /// Rule (Tcl, so the reference): a command the interpreter does not have is an error, and so
    /// is an undefined variable. Neither is a name or a value: the command is not applied and is
    /// listed with the queries not evaluated. Found reading OpenTitan's constraint files, written
    /// for a synthesis tool: `remove_from_collection` became a port of that name, and
    /// `${DRIVING_CELL}` (set by the flow) a lib cell `$`.
    #[test]
    fn an_unknown_command_or_undefined_variable_refuses_its_command() {
        let s = Sdc::parse(
            "create_clock -name c -period 8 [get_ports clk_i]\n\
             set_input_delay 1 [remove_from_collection [all_inputs] {clk_i}] -clock c\n\
             set_load [load_of lib/BUF/A] [all_outputs]\n\
             set_driving_cell -lib_cell ${DRIVING_CELL} [all_inputs]\n\
             set_output_delay 1 [all_outputs] -clock c\n",
        )
        .unwrap();
        assert!(s.input_delays.is_empty() && s.env.is_empty() && s.driving_cells.is_empty(), "{s:?}");
        assert_eq!(s.output_delays.len(), 1, "the rest of the file still applies");
        assert_eq!(s.unevaluated_queries.len(), 3, "{:?}", s.unevaluated_queries);
        assert!(s.unevaluated_queries[2].contains("$DRIVING_CELL"), "{:?}", s.unevaluated_queries);
    }

    /// Inside braces Tcl substitutes nothing: an escaped name's `$` is text, not a variable.
    #[test]
    fn a_dollar_inside_braces_is_text() {
        let s = Sdc::parse("set_false_path -to [get_pins {ctrl.state.out[0]$_DFF_P_/D}]\n").unwrap();
        assert!(s.unevaluated_queries.is_empty() && s.exceptions.len() == 1, "{:?}", s.rejected);
    }
}

#[cfg(test)]
mod tcl_values {
    use super::*;

    /// Rule (Tcl): `[list a b]` is the list `a b`, and `[info exists ::env(X)]` is 1 exactly when
    /// the environment has X. Both from public constraint files (a RISC-V core's
    /// `-waveform [list 0 [expr $p/2]]`, an accelerator flow's period override).
    #[test]
    fn list_values_and_info_exists_are_evaluated() {
        let s = Sdc::parse("set p 1000\ncreate_clock -name c -period $p -waveform [list 0 [expr $p/2]] [get_ports clk]\n").unwrap();
        assert_eq!(s.clocks[0].waveform, Some(vec![0.0, 500.0]));
        let s = Sdc::parse("set p 5\nif {[info exists ::env(VYGES_SDC_TEST_UNSET_VAR)]} {\n  set p 7\n}\nif {[info exists p]} {\n  create_clock -name c -period $p [get_ports clk]\n}\n").unwrap();
        assert!(s.ignored.is_empty(), "{:?}", s.ignored);
        assert_eq!(s.clocks[0].period, 5.0);
    }
}
