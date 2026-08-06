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
//! - `create_clock -name N -period P {obj}` and
//!   `create_generated_clock -name N -source S -divide_by D -multiply_by M {obj}`
//! - `set_input_delay` / `set_output_delay` (default via `all_inputs`/`all_outputs`
//!   or `-clock`, plus per-port overrides) — the I/O timing budget
//! - `set_clock_uncertainty [-setup|-hold]` — setup/hold guard band
//! - `set_clock_latency` — captured (source latency applied to the I/O budget)
//! - `set_input_transition`, `set_load` — boundary slew / load
//! - `set_timing_derate -late|-early` — OCV derate
//! - `set_false_path` / `set_multicycle_path` — timing exceptions
//! - `set_units` — time/capacitance scaling to the engine's ns/pF
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
    pub period: f64, // ns
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
    pub value: f64, // ns
    pub default: bool,
    pub ports: Vec<String>,
}

#[derive(Debug, Default)]
pub struct Sdc {
    pub clocks: Vec<SdcClock>,
    pub input_delays: Vec<IoDelay>,
    pub output_delays: Vec<IoDelay>,
    pub setup_uncertainty: f64,
    pub hold_uncertainty: f64,
    pub clock_latency: f64, // source/network latency (ns), applied to the I/O budget
    pub input_transition: Option<f64>,
    pub load: Option<f64>,
    pub late_derate: Option<f64>,
    pub early_derate: Option<f64>,
    pub exceptions: Vec<Exception>,
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
                    let source = obj
                        .as_deref()
                        .map(|o| resolve_objs(o).first().cloned().unwrap_or_default())
                        .unwrap_or_default();
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
                    sdc.clocks.push(SdcClock {
                        name,
                        source: src,
                        period: period * t_scale,
                    });
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
                    let val: f64 = toks
                        .get(1)
                        .filter(|t| !t.starts_with('-'))
                        .or_else(|| flag_val(&toks, "-max"))
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(0.0);
                    let obj = trailing_obj(
                        &toks,
                        &["-clock", "-max", "-min", "-reference_pin", "-comment"],
                    );
                    let objs = obj.as_deref().map(resolve_objs).unwrap_or_default();
                    let default =
                        objs.is_empty() || objs.iter().any(|o| o == "*INPUTS*" || o == "*OUTPUTS*");
                    let ports: Vec<String> =
                        objs.into_iter().filter(|o| !o.starts_with('*')).collect();
                    let d = IoDelay {
                        value: val * t_scale,
                        default,
                        ports,
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
                    if let Some(v) = toks.get(1).and_then(|t| t.parse::<f64>().ok()) {
                        sdc.input_transition = Some(v * t_scale);
                    }
                }
                "set_load" => {
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
                    if groups.len() >= 2 {
                        sdc.async_groups.extend(groups);
                    }
                }
                "set_false_path" => {
                    let (from, to) = from_to(&toks);
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
                source: target,
                period,
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
    // ALL objects each flag names, not the first. A `-from [list a b c]` that cut only `a`
    // left `b` and `c` timed against the SDC's stated intent.
    let pick = |flags: &[&str]| -> Vec<String> {
        let mut out: Vec<String> = Vec::new();
        for f in flags {
            if let Some(v) = flag_val(toks, f) {
                for name in resolve_objs(v) {
                    if name.starts_with('*') {
                        continue; // an `all_inputs`-style collection, not a named object
                    }
                    // a pin object (`reg/Q`) reduces to its instance, matching the engine's
                    // instance-level comparison
                    out.push(match name.rsplit_once('/') {
                        Some((inst, _pin)) => inst.to_string(),
                        None => name,
                    });
                }
            }
        }
        out.sort();
        out.dedup();
        if out.is_empty() {
            out.push("*".to_string()); // the flag was absent or named only a collection
        }
        out
    };
    (
        pick(&["-from", "-rise_from", "-fall_from"]),
        pick(&["-to", "-rise_to", "-fall_to"]),
    )
}

#[cfg(test)]
mod async_group_tests {
    use super::*;
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
