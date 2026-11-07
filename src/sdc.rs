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

mod kept;
pub mod registry;
mod scope;
mod tcl;

/// A running proc's variables, and its `upvar` links (local name, caller's name) written back
/// to the caller at its `return`.
type ProcFrame = (HashMap<String, String>, Vec<(String, String)>);
mod unset;

pub use kept::{ExceptionCmd, KeptArg, KeptCmd};
pub(crate) use kept::write_kept as kept_line;

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
    /// `-comment`, as written: the reference keeps it with the command and writes it back.
    pub comment: Option<String>,
}

/// Is `name` in an exception endpoint set? An empty set, or one holding `*`, matches anything;
/// a pattern with `*` / `?` matches as the reference's object query does (a glob).
fn endpoint_matches(set: &[String], name: &str) -> bool {
    set.is_empty() || set.iter().any(|o| o == "*" || o == name || (o.contains(['*', '?']) && glob(o, name)))
}

/// Does an SDC object name match `name` — itself, or as a `*` / `?` pattern the way the
/// reference's object queries match one (`get_ports in*`)? For a consumer applying a per-object
/// constraint: an exact-name lookup silently never applied `set_input_delay … [get_ports in*]`.
///
/// A bit of a bus port (`boot_addr_i[3]`) is also matched by its BUS: the reference matches a
/// pattern against port names, a bus port's name being the bus (`boot_addr_i`), and a port
/// query or an implicit object argument names every bit of a bus it matches. A pattern with a
/// subscript matches the bus part as a pattern, then the bit: `[n]`, a `[a:b]` range, or `[*]`.
///
/// A pattern is bus-form when its last `[…]` starts with a digit or `*` (`parseBusName`); only
/// then is it matched as a bus, and NEVER as a whole-name glob: `req_msg[1?]` is bit 1 (the
/// subscript is read as an integer, its leading digits), not bits 10 to 19.
pub fn object_matches(pattern: &str, name: &str) -> bool {
    object_matches_by(pattern, name, &|p: &str, n: &str| p == n || (p.contains(['*', '?']) && glob(p, n)))
}

/// A pattern's bus form, `(bus, subscript)`, when its last `[…]` starts with a digit or `*`
/// (`parseBusName`) — for a glob and a `-regexp` pattern alike.
pub(crate) fn bus_form(pattern: &str) -> Option<(&str, &str)> {
    pattern
        .strip_suffix(']')
        .and_then(|p| p.rsplit_once('['))
        .filter(|(b, sub)| !b.is_empty() && !b.ends_with('\\') && sub.starts_with(|c: char| c.is_ascii_digit() || c == '*'))
}

/// [`object_matches`] with the name matcher given: the reference's port finder applies the same
/// bus rule whether the pattern is a glob or (`-regexp`) a regular expression.
pub(crate) fn object_matches_by(pattern: &str, name: &str, named: &dyn Fn(&str, &str) -> bool) -> bool {
    let Some((pbase, sub)) = bus_form(pattern) else {
        // a plain pattern is matched against PORT names: a scalar's, or a bit's bus (every bit of
        // a bus it names) — never a bit's own name, so `req_msg[?]` names nothing
        return match bus_bit(name) {
            Some((base, _)) => named(pattern, base),
            None => named(pattern, name),
        };
    };
    if pattern == name {
        return true;
    }
    let Some((base, bit)) = bus_bit(name) else { return false };
    // `std::stoi`: the leading digits (after optional space and sign)
    let stoi = |t: &str| -> Option<i64> {
        let t = t.trim_start();
        let n = t.char_indices().take_while(|&(i, c)| c.is_ascii_digit() || (i == 0 && (c == '-' || c == '+'))).count();
        t[..n].parse().ok()
    };
    named(pbase, base)
        && if sub.starts_with('*') {
            true
        } else {
            match sub.split_once(':') {
                Some((a, b)) => matches!((stoi(a), stoi(b)), (Some(a), Some(b)) if (a.min(b)..=a.max(b)).contains(&bit)),
                None => stoi(sub) == Some(bit),
            }
        }
}

/// `name[n]` as (`name`, n) — a bus bit, as the netlist names one; an escaped `\[` is part of a name.
fn bus_bit(name: &str) -> Option<(&str, i64)> {
    let (base, idx) = name.strip_suffix(']')?.rsplit_once('[')?;
    (!base.is_empty() && !base.ends_with('\\')).then_some(())?;
    Some((base, idx.parse().ok()?))
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

/// `escapeChars(token, chars…, '\\')`: each of `chars` gets a backslash before it; a character
/// already escaped is kept as it is.
fn escape_chars(token: &str, chars: &[char]) -> String {
    let mut out = String::with_capacity(token.len());
    let mut it = token.chars();
    while let Some(c) = it.next() {
        if c == '\\' {
            out.push(c);
            if let Some(n) = it.next() {
                out.push(n);
            }
        } else {
            if chars.contains(&c) {
                out.push('\\');
            }
            out.push(c);
        }
    }
    out
}

/// One scan of a `get_pins` pattern (`SdcNetwork::visitMatches`): at each divider not escaped,
/// the path before it with its earlier dividers escaped, whether a bracket (not escaped) came
/// before it, and the rest of the pattern after it; then the whole path, its dividers escaped.
fn pin_pattern_splits(pattern: &str) -> (Vec<(String, bool, &str)>, String, bool) {
    let mut splits = Vec::new();
    let mut path = String::new();
    let mut brackets = false;
    let mut chars = pattern.char_indices();
    while let Some((i, c)) = chars.next() {
        match c {
            '\\' => {
                path.push(c);
                if let Some((_, n)) = chars.next() {
                    path.push(n);
                }
            }
            '/' => {
                splits.push((path.clone(), brackets, &pattern[i + 1..]));
                path.push_str("\\/");
            }
            _ => {
                brackets |= c == '[' || c == ']';
                path.push(c);
            }
        }
    }
    (splits, path, brackets)
}

/// `get_pins <pattern>` over a flat netlist, as the reference's SDC network finds pins
/// (`SdcNetwork::findPinsMatching` -> `visitMatches` -> `visitPinTail`). The netlist's names keep
/// the database's escapes (`out\[1\]$_DFF_P_`); SDC names, as `write_sdc` writes them, do not.
///
/// At each divider of the pattern not escaped, the path before it (earlier dividers escaped) is
/// matched against the instance names as a `*` / `?` glob; when NO instance matches and a bracket
/// came before the divider, the path with its brackets escaped is matched instead. In each
/// instance matched, the rest of the pattern (its dividers escaped) is matched against the port
/// names, and against each port name with its dividers escaped; when no port of that instance
/// matches and the rest has a bracket, the rest with its brackets escaped is tried. A pattern
/// with no divider names no pin (top ports are not pins). `pins`: `(instance, port)`; the
/// matches are returned in that order. The pattern `*` alone is every pin. Bus ports (a pattern naming every bit of a bus) are not
/// read: a cell port here is one bit.
pub fn pins_matching(pattern: &str, insts: &[&str], pins: &[(usize, &str)]) -> Vec<usize> {
    // `*` alone: every instance's pins.
    if pattern == "*" {
        return (0..pins.len()).collect();
    }
    let mut hit = vec![false; pins.len()];
    let mut by_inst: Vec<Vec<usize>> = vec![Vec::new(); insts.len()];
    for (j, &(k, _)) in pins.iter().enumerate() {
        if let Some(v) = by_inst.get_mut(k) {
            v.push(j);
        }
    }
    let (splits, _, _) = pin_pattern_splits(pattern);
    for (path, brackets, tail) in splits {
        let mut matched: Vec<usize> = (0..insts.len()).filter(|&k| glob(&path, insts[k])).collect();
        if matched.is_empty() && brackets {
            let escaped = escape_chars(&path, &['[', ']']);
            matched = (0..insts.len()).filter(|&k| glob(&escaped, insts[k])).collect();
        }
        // `visitMatches(child, tail)` on a leaf: no children, so its whole tail goes to
        // `visitPinTail`.
        let (_, tail_path, tail_brackets) = pin_pattern_splits(tail);
        for k in matched {
            let ports = &by_inst[k];
            let port_hit = |t: &str, p: &str| glob(t, p) || glob(t, &escape_chars(p, &['/']));
            let mut found = false;
            for &j in ports {
                if port_hit(&tail_path, pins[j].1) {
                    hit[j] = true;
                    found = true;
                }
            }
            if !found && tail_brackets {
                let escaped = escape_chars(&tail_path, &['[', ']']);
                for &j in ports {
                    if port_hit(&escaped, pins[j].1) {
                        hit[j] = true;
                    }
                }
            }
        }
    }
    (0..pins.len()).filter(|&j| hit[j]).collect()
}

/// `get_cells <pattern>` over a flat netlist, as the reference's SDC network finds instances
/// (`SdcNetwork::findInstancesMatching` -> `visitMatches` with the children finder as its tail).
/// In a flat netlist no instance has children, so a split at a divider never finds one: what
/// matches is the WHOLE pattern, its dividers not escaped now escaped (`h/u1` names the
/// instance `h\/u1`), as a `*` / `?` glob against the instance names; when NO instance matches
/// and the pattern has a bracket not escaped, the pattern with its brackets escaped is matched
/// instead (`out[1]$_DFF_P_` names the database's `out\[1\]$_DFF_P_`, a literal `x[2]`
/// preventing the retry). The matches are returned in `insts` order.
pub fn insts_matching(pattern: &str, insts: &[&str]) -> Vec<usize> {
    let (_, path, brackets) = pin_pattern_splits(pattern);
    let matched: Vec<usize> = (0..insts.len()).filter(|&k| glob(&path, insts[k])).collect();
    if !matched.is_empty() || !brackets {
        return matched;
    }
    let escaped = escape_chars(&path, &['[', ']']);
    (0..insts.len()).filter(|&k| glob(&escaped, insts[k])).collect()
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
    /// Does it apply to the HOLD check (`true`) or the SETUP check (`false`)?
    ///
    /// A false path: `-setup` / `-hold` scope it as the reference reads them (MinMaxAll: neither
    /// or both is both) — `set_false_path -hold` leaves the setup check timed. A multicycle: a
    /// `-hold` one does not move the setup check; any one moves the hold check (a setup
    /// multiplier moves the hold edge with it).
    ///
    /// ⛔ A consumer that cuts both checks for every false path cuts the setup paths a `-hold`
    /// false path left timed — optimistic and silent.
    pub fn applies_to(&self, hold_check: bool) -> bool {
        let has = |f: &str| self.flags.iter().any(|x| x == f);
        match self.kind {
            ExcKind::FalsePath => {
                if hold_check { has("-hold") || !has("-setup") } else { has("-setup") || !has("-hold") }
            }
            ExcKind::Multicycle(_) => hold_check || !self.hold,
        }
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
    /// `-comment`, as written: the reference keeps it with the command and writes it back.
    pub comment: Option<String>,
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
                let keys = crate::sdc_flags::keys(&accessor);
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
    /// `-comment`, as written: the reference keeps it with the command and writes it back.
    pub comment: Option<String>,
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

/// `toks` read as the reference's `parse_key_args` reads them for a command with these keys and
/// flags, then `check_argc_eq1`: exactly ONE word is left, the objects (Tcl's "wrong # args"
/// otherwise — an option the command does not know is left as a word, and so fails it).
fn arc_cmd(toks: &[String], keys: &[&str], flags: &[&str]) -> Result<ArcCmd, String> {
    let (mut kv, mut fl, mut pos) = (Vec::new(), Vec::new(), Vec::new());
    let mut k = 1;
    while k < toks.len() {
        let t = toks[k].as_str();
        if keys.contains(&t) {
            let Some(v) = toks.get(k + 1) else { return Err(format!("{}: {t} without a value; not applied", toks[0])) };
            let v = if t.starts_with("-clock") { resolve_objs(v).join(" ") } else { v.clone() };
            kv.push((t.to_string(), v));
            k += 2;
            continue;
        }
        if flags.contains(&t) {
            fl.push(t.to_string());
        } else {
            pos.push(&toks[k]);
        }
        k += 1;
    }
    let [obj] = pos.as_slice() else {
        return Err(format!("{}: {} words where the objects are one (the reference's wrong # args); not applied", toks[0], pos.len()));
    };
    Ok(ArcCmd { cmd: toks[0].clone(), keys: kv, flags: fl, objects: resolve_objs(obj), written: Objects::of(obj) })
}

/// A clock sense, as the reference applies one (`set_sense`, `set_clock_sense_cmd1` in its SDC
/// Tcl): `set_clock_sense …` always; `set_sense` with `-type clock`, and without `-type` (below);
/// `-type data` the reference does not support (Warning 413), and neither does this reader
/// ([`SENSE_TYPE_DATA`]). `-clocks` replaces `-clock` when both are given; `-pulse` is read and
/// not supported (Warning 416), and the three senses together only warn (417): each is kept as
/// written for a consumer to refuse.
///
/// Returns the sense, and whether it is a `set_sense` written without `-type` (`true`): SDC 2.1
/// makes `-type clock` the default, so it is read as `-type clock` and written back with it.
///
/// ⚠️ The reference differs here: its `set_sense` proc defaults the type to clock, but calls its
/// sense command only inside the branch for an explicit `-type`, so without `-type` it applies
/// nothing (though its own help says the type defaults to clock). Lint reports each such command.
/// Why a `set_sense -type data` is refused. Lint reports it as a warning, not an error: the
/// reference timer does not apply it either (its Warning 413), so a run here and a run there agree.
pub const SENSE_TYPE_DATA: &str = "-type data is not modelled (the reference timer does not apply it either, its Warning 413); not applied";

fn sense_cmd(toks: &[String]) -> Result<(ArcCmd, bool), String> {
    const KEYS: [&str; 3] = ["-clock", "-clocks", "-pulse"];
    const FLAGS: [&str; 3] = ["-positive", "-negative", "-stop_propagation"];
    let defaulted = toks[0] == "set_sense" && !toks.iter().any(|t| t == "-type");
    let rest: Vec<String> = if toks[0] == "set_sense" {
        let ty = toks.iter().position(|t| t == "-type").and_then(|i| toks.get(i + 1).map(|v| (i, v.as_str())));
        match ty {
            Some((i, "clock")) => toks.iter().enumerate().filter(|(j, _)| *j != i && *j != i + 1).map(|(_, t)| t.clone()).collect(),
            Some((_, "data")) => return Err(format!("set_sense: {SENSE_TYPE_DATA}")),
            Some((_, other)) => return Err(format!("set_sense: -type {other} is not clock or data (the reference's Error 414); not applied")),
            // `-type` as the last word: the value is missing
            None if !defaulted => return Err("set_sense: -type needs a value (clock or data); not applied".into()),
            None => toks.to_vec(),
        }
    } else {
        toks.to_vec()
    };
    // abbreviations expand as `set_clock_sense_cmd1`'s own parse expands them
    let mut as_cmd1 = rest;
    let name = std::mem::replace(&mut as_cmd1[0], "set_clock_sense".into());
    let mut as_cmd1 = expand_abbreviations(as_cmd1);
    as_cmd1[0] = name;
    let mut c = arc_cmd(&as_cmd1, &KEYS, &FLAGS)?;
    if c.keys.iter().any(|(k, _)| k == "-clocks") {
        c.keys.retain(|(k, _)| k != "-clock");
    }
    Ok((c, defaulted))
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
    /// `-comment`, as written: the reference keeps it with the command and writes it back.
    pub comment: Option<String>,
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

/// One `set_disable_timing` / `unset_disable_timing`, `set_sense -type clock` / `set_clock_sense`,
/// or `set_disable_clock_gating_check` / `unset_…`, as the reference reads it: its keys with their
/// values as written (`-from`/`-to` are a cell's port NAMES; `-clock`/`-clocks` hold clocks), its
/// flags (`-positive`, `-negative`, `-stop_propagation`), and its one object list.
#[derive(Debug, Clone, PartialEq)]
pub struct ArcCmd {
    pub cmd: String,
    pub keys: Vec<(String, String)>,
    pub flags: Vec<String>,
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

#[derive(Debug, Clone, Default)]
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
    /// Every `set_max_transition` on anything but the design alone (ports, clocks with
    /// `-clock_path` / `-data_path`, `-rise` / `-fall`), in file order.
    pub max_transition_objects: Vec<DesignRuleCmd>,
    /// Every `set_max_fanout` on anything but the design alone, in file order.
    pub max_fanout_objects: Vec<DesignRuleCmd>,
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
    /// Every `set_disable_timing` and `unset_disable_timing`, in file order (still in
    /// [`Sdc::ignored`] for a consumer that does not model them).
    pub disabled_timing: Vec<ArcCmd>,
    /// Every clock sense the reference applies — `set_sense -type clock`, `set_clock_sense` — in
    /// file order (still in [`Sdc::ignored`]).
    pub senses: Vec<ArcCmd>,
    /// The indices in [`Sdc::senses`] of each `set_sense` written without `-type`: read as
    /// `-type clock`, the SDC 2.1 default (the reference applies none of them — see `sense_cmd`).
    pub senses_type_defaulted: Vec<usize>,
    /// Every `set_disable_clock_gating_check` and `unset_…`, in file order (still in
    /// [`Sdc::ignored`]).
    pub disabled_gating_checks: Vec<ArcCmd>,
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
    /// Every `group_path` and `set_path_margin`, in file order: the reference's other path
    /// exceptions, which a reset reaches as it reaches a false path (still in [`Sdc::ignored`]).
    pub other_exceptions: Vec<ExceptionCmd>,
    /// Every command the reference reads that has no typed model here, kept as written, in file
    /// order — see [`KeptCmd`] (each still in [`Sdc::ignored`]).
    pub kept: Vec<KeptCmd>,
    /// Each use of an SDC option no consumer here applies (`set_input_delay -level_sensitive`,
    /// `all_inputs -clock`: [`crate::sdc_flags::KEPT_OPTIONS`]), as `command -option`, in file
    /// order. Its command is kept and written back — in its typed record when the record carries
    /// the option, else as written in [`Sdc::kept`] — and `check` reports it as kept-not-applied.
    /// Each is also in [`Sdc::ignored`], for a consumer that reads only that.
    pub kept_options: Vec<String>,
    /// Each use of a reference-timer command or option that no SDC version declares
    /// (`get_ports -filter`, `set_pvt`: [`crate::sdc_flags::EXTENSION_OPTIONS`] /
    /// [`crate::sdc_flags::EXTENSION_COMMANDS`]), as `command` or `command -option`, in file
    /// order. Read as the reference reads it; `check` names each at info level (portability only).
    pub extensions: Vec<String>,
    /// Each word the reference warns about and IGNORES while still applying its command, as
    /// `command word`, in file order: `set_clock_groups` takes only its flags and `-group`, and
    /// warns on any other option (STA-0402 "unknown keyword argument", e.g.
    /// `-include_generated_clocks`) and on a stray word (STA-0403 "extra positional argument").
    /// Its groups are applied without them. `check` reports each at warning level.
    pub ignored_words: Vec<String>,
    pub ignored: Vec<String>, // commands we recognised but do not model
    /// Per entry of [`Sdc::ignored`], index for index: `(the entry, the command it came from)`,
    /// the command's text as read with its variables substituted (a whole `if` or `foreach` block
    /// for one the reader did not evaluate). Two files whose unmodelled commands have the same
    /// text have the same unmodelled constraints: `sdc_diff` compares them so.
    pub unmodelled_text: Vec<(String, String)>,
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
    // ── outside the SDC versions the reference parser declares (to 2.0) ─────────────────
    "set_disable_clock_gating_check", // removes a check, so its violations vanish
    "set_max_trans",                  // Tcl accepts unambiguous abbreviations
];

/// Explicitly NOT timing-affecting: instructions to synthesis, placement or power
/// optimisation. Listed rather than merely omitted, so the distinction is a decision on record
/// instead of an oversight — `set_max_area` and `set_driving_cell` are both "unmodelled", and
/// only one of them can move a slack.
///
/// One of these that the reference does not know (`set_size_only` … `set_critical_range`) is read
/// past rather than stopping the read, as an unknown command otherwise does — see [`Sdc::parse`].
/// ⚠️ Membership is therefore also a decision to read past: add only what no timer consults.
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
        // group paths and path margins are exceptions too, by the same rule
        self.other_exceptions.retain_mut(|p| {
            for side in ["from", "to"] {
                let on_side = |k: &str| k.trim_start_matches('-').trim_start_matches("rise_").trim_start_matches("fall_") == side;
                let names_it = p.args.iter().any(|(k, o)| on_side(k) && o.accessor == "get_clocks" && o.names.iter().any(|n| n == name));
                if names_it {
                    p.args.iter_mut().filter(|(k, _)| on_side(k)).for_each(|(_, o)| drop_name(o));
                    p.args.retain(|(k, o)| !(on_side(k) && o.names.is_empty()));
                    return p.args.iter().any(|(k, _)| on_side(k));
                }
            }
            true
        });
        self.remove_kept_clock(name);
        for g in &mut self.async_groups {
            g.retain(|n| n != name);
        }
        self.async_groups.retain(|g| !g.is_empty());
        // `clockGroupsDeleteClkRefs`: each command's groups lose the clock, an emptied group goes;
        // the command itself stays
        for g in &mut self.clock_groups {
            for grp in &mut g.groups {
                grp.retain(|n| n != name);
            }
            g.groups.retain(|grp| !grp.is_empty());
        }
        // `deleteMasterClkRefs`: a generated clock of this master has none
        for c in &mut self.clocks {
            if let Some(g) = c.generated.as_mut().filter(|g| g.master_clock.as_deref() == Some(name)) {
                g.master_clock = None;
            }
        }
        // `deleteInterClockUncertaintiesReferencing`: every pair with the clock on either side
        // goes; the clock's own uncertainty and slew (`set_clock_transition`) go with the clock
        let on_clock = |o: &Objects| o.accessor == "get_clocks" || (o.accessor.is_empty() && o.names.iter().any(|n| n == name));
        self.uncertainties.retain_mut(|u| {
            if !u.from.is_empty() || !u.to.is_empty() {
                u.from.retain(|n| n != name);
                u.to.retain(|n| n != name);
                for (_, o) in &mut u.written_sides {
                    o.names.retain(|n| n != name);
                }
                return !u.from.is_empty() && !u.to.is_empty();
            }
            for w in u.written.iter_mut().filter(|w| on_clock(w)) {
                w.names.retain(|n| n != name);
            }
            u.written.retain(|w| !w.names.is_empty());
            u.objects = u.written.iter().flat_map(|w| w.names.iter().cloned()).collect();
            !u.objects.is_empty()
        });
        self.refold_uncertainty();
        self.clock_transitions.retain_mut(|e| {
            if let Some(w) = e.written.as_mut().filter(|w| on_clock(w)) {
                w.names.retain(|n| n != name);
                e.objects = w.names.clone();
                return !e.objects.is_empty();
            }
            true
        });
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

    /// Every command the model does NOT carry, grouped by command and reason, in first-seen
    /// order: each [`Sdc::rejected`] entry (which includes every unevaluated query) and each
    /// [`Sdc::ignored`] command that is not modelled elsewhere ([`MODELLED_BUT_LISTED`]).
    ///
    /// This is exactly what [`Sdc::write`] refuses on, seen from a checker's side: a constraint
    /// checker that does not report these can say "clean" about a file whose constraints it never
    /// looked at. `affects_timing` is false only for [`BENIGN_FOR_TIMING`] commands — a command
    /// that is unknown to that list is treated as able to move a slack.
    pub fn not_applied(&self) -> Vec<NotApplied> {
        let mut out: Vec<NotApplied> = Vec::new();
        let mut add = |command: String, why: String| {
            if let Some(n) = out.iter_mut().find(|n| n.command == command && n.why == why) {
                n.count += 1;
                return;
            }
            let affects_timing = !BENIGN_FOR_TIMING.contains(&command.as_str());
            out.push(NotApplied { command, why, count: 1, affects_timing });
        };
        for r in &self.rejected {
            // `<command>: <why>` or `<command> without …`
            let cut = r.find([':', ' ']).unwrap_or(r.len());
            let why = r[cut..].trim_start_matches(':').trim().to_string();
            add(r[..cut].to_string(), if why.is_empty() { "rejected".into() } else { format!("rejected: {why}") });
        }
        for c in self.ignored.iter().filter(|c| !MODELLED_BUT_LISTED.contains(&c.as_str()) && !self.kept_options.contains(c)) {
            add(c.clone(), "not modelled; read past".into());
        }
        out
    }
}

impl Sdc {
    /// Every command read that is kept and written back but applied by no checker or timer here
    /// ([`registry::Status::Kept`]), with how many times, in first-seen order. `vyges-sdc check`
    /// reports each, so a file is not called clean while one of these goes unapplied.
    pub fn kept_not_applied(&self) -> Vec<KeptNotApplied> {
        let mut out: Vec<KeptNotApplied> = Vec::new();
        let options = self.kept_options.iter();
        for c in self.ignored.iter().filter(|c| registry::kept_not_applied(c)).chain(options) {
            match out.iter_mut().find(|k| &k.command == c) {
                Some(k) => k.count += 1,
                None => out.push(KeptNotApplied { command: c.clone(), count: 1, affects_timing: !BENIGN_FOR_TIMING.contains(&c.as_str()) }),
            }
        }
        out
    }
}

/// One kind of command kept and written back but applied by nothing here — see
/// [`Sdc::kept_not_applied`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeptNotApplied {
    /// The command (or `command -option`), as the reader names it.
    pub command: String,
    pub count: usize,
    /// Whether applying it could move a slack (everything not in [`BENIGN_FOR_TIMING`]).
    pub affects_timing: bool,
}

/// Kinds the reader models but ALSO lists in [`Sdc::ignored`], for a consumer that does not model
/// them: they are carried in their own records ([`Sdc::path_delays`], [`Sdc::case_values`], …),
/// so neither the writer nor [`Sdc::not_applied`] reports them as dropped.
pub const MODELLED_BUT_LISTED: &[&str] = &[
    "set_clock_transition",
    "set_max_fanout",
    "set_max_capacitance",
    "set_propagated_clock",
    "set_max_delay",
    "set_min_delay",
    "set_max_transition",
    "set_case_analysis",
    "set_logic_zero",
    "set_logic_one",
    "set_logic_dc",
    "set_driving_cell",
    "set_timing_derate",
    "set_disable_timing",
    "unset_disable_timing",
    "set_sense",
    "set_clock_sense",
    "set_disable_clock_gating_check",
    "unset_disable_clock_gating_check",
    // kept as written (`sdc/kept.rs`)
    "group_path",
    "set_path_margin",
    "set_min_pulse_width",
    "set_max_time_borrow",
    "set_clock_gating_check",
    "set_data_check",
    "unset_data_check",
    "set_drive",
    "set_port_fanout_number",
    "set_resistance",
    "set_min_capacitance",
    "set_fanout_load",
    "set_max_area",
    "set_max_dynamic_power",
    "set_max_leakage_power",
    "set_operating_conditions",
    "set_wire_load_mode",
    "set_wire_load_model",
    "set_wire_load_selection_group",
    "set_wire_load_min_block_size",
    "set_voltage",
    "set_pvt",
    "set_ideal_latency",
    "set_ideal_network",
    "set_ideal_transition",
    "create_voltage_area",
    "set_level_shifter_strategy",
    "set_level_shifter_threshold",
];

/// One kind of command the model does not carry — see [`Sdc::not_applied`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NotApplied {
    /// The command, as written (`set_disable_timing`, `remove_from_collection`, …).
    pub command: String,
    /// Why: `rejected: <reason>` or `not modelled; read past`.
    pub why: String,
    /// How many commands of this kind, for this reason.
    pub count: usize,
    /// Whether leaving it out can move a slack (everything not in [`BENIGN_FOR_TIMING`]).
    pub affects_timing: bool,
}

/// A design-rule limit on objects (`set_max_transition -clock_path 0.2 [get_clocks c]`): the
/// value AS WRITTEN (NaN when it is not a number), the flags that take no value, and the objects.
#[derive(Debug, Clone)]
pub struct DesignRuleCmd {
    pub value: f64,
    pub flags: Vec<String>,
    pub objects: Objects,
}

/// Why a design-rule limit's arguments are refused, by its definition. The public SDC parser
/// declares `set_max_capacitance` and `set_max_fanout` as `value object_list`, and
/// `set_max_transition` as `[-clock_path] [-rise] [-fall] value object_list`; the reference timer
/// adds `-data_path`, and newer vendor SDC 1.8 adds options to `set_max_capacitance` and
/// `set_max_transition` ([`crate::sdc_flags::VENDOR_18_ONLY`]). An option none of them has is
/// refused before here (Error 562); `set_max_fanout` has none. Each requires a value and
/// objects, and a value that is a positive float (`$par>=0`; the reference's Error 572); any
/// other word count is `wrong # args`.
/// ⛔ Unchecked, `set_max_capacitance -min 0.2 [get_ports a]` read `0.2` as the objects of a limit
/// with no value, and `set_max_capacitance 0.2` was dropped without a word.
fn design_rule_fault(toks: &[String]) -> Option<String> {
    let (keys, known) = crate::sdc_flags::options(&toks[0]).unwrap_or_default();
    let (_, flags, pos) = split_args(toks, &keys);
    if let Some(f) = flags.iter().find(|f| !known.contains(&f.as_str())) {
        return Some(format!("{f} is not an option of {}", toks[0]));
    }
    let Some(value) = pos.first() else { return Some("missing value".into()) };
    if pos.len() < 2 {
        return Some("missing objects (the value applies to an object list, `[current_design]` for the design)".into());
    }
    if pos.len() > 2 {
        return Some(format!("{} positional arguments where it takes a value and objects (wrong # args)", pos.len()));
    }
    if value.trim_matches(|c| c == '{' || c == '}').trim().parse::<f64>().map_or(true, |v| v < 0.0) {
        return Some(format!("value '{value}' is not a positive float (Error 572)"));
    }
    None
}

/// `<cmd> [flags] value objects`, or `None` when there is no object word.
fn design_rule_cmd(toks: &[String]) -> Option<DesignRuleCmd> {
    let (_, flags, pos) = split_args(toks, &[]);
    let value = pos.first().and_then(|v| v.parse::<f64>().ok()).unwrap_or(f64::NAN);
    Some(DesignRuleCmd { value, flags, objects: Objects::of(pos.get(1)?) })
}

/// `set_units` / `check_unit` (OpenSTA `sdc/Sdc.tcl`): each unit's value is `[scale][prefix]suffix`,
/// the suffix matched without case (`s`, `f`, `ohm`, `v`, `A`, `W`, `m`), the scale `1`, `10`, `100`,
/// ... (a `.` and trailing zeros allowed) and the prefix one of `M k m u n p f`. The units are
/// checked in the reference's order, not the file's, so the first bad one in that order is named.
fn check_units(toks: &[String]) -> Result<(), String> {
    const UNITS: [(&str, &str, &str); 7] = [
        ("time", "-time", "s"),
        ("capacitance", "-capacitance", "f"),
        ("resistance", "-resistance", "ohm"),
        ("voltage", "-voltage", "v"),
        ("current", "-current", "A"),
        ("power", "-power", "W"),
        ("distance", "-distance", "m"),
    ];
    for (unit, key, suffix) in UNITS {
        let Some(value) = toks.iter().position(|t| t == key).and_then(|i| toks.get(i + 1)) else { continue };
        let chars: Vec<char> = value.chars().collect();
        let n = suffix.chars().count();
        let split = chars.len().saturating_sub(n);
        let arg_suffix: String = chars[split..].iter().collect();
        if !arg_suffix.eq_ignore_ascii_case(suffix) {
            return Err(format!("incorrect unit suffix '{arg_suffix}' (Error 501)"));
        }
        let prefix: String = chars[..split].iter().collect();
        if !unit_prefix_ok(&prefix) {
            return Err(format!("unknown unit {unit} prefix '{prefix}' (Error 343)"));
        }
    }
    Ok(())
}

/// `^(10*\.?0*)?([Mkmunpf])?$`, the reference's unit prefix pattern.
fn unit_prefix_ok(p: &str) -> bool {
    let mut s = p;
    if let Some(r) = s.strip_prefix('1') {
        s = r.trim_start_matches('0');
        s = s.strip_prefix('.').unwrap_or(s);
        s = s.trim_start_matches('0');
    }
    let mut c = s.chars();
    match (c.next(), c.next()) {
        (None, _) => true,
        (Some(ch), None) => "Mkmunpf".contains(ch),
        _ => false,
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
pub(crate) fn logical_lines(text: &str) -> Vec<String> {
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
pub(crate) fn tokenize(line: &str) -> Vec<String> {
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
/// EQUAL to the pattern, and the reference's collection elements are object HANDLES
/// (`_e0ca…_p_Port`), not names (observed 2026-10-31): `[get_ports P]` of one port is a handle
/// equal to its element, so P is excluded; a NAME (`clk`, `{clk}`) or a query of two or more ports
/// equals no element, so nothing is excluded (an empty list); a wildcard query, whose port count
/// needs the design, is not read here (`None`). ⛔ A bare name was read as excluding its port: the
/// reference constrains it.
///
/// `[remove_from_collection [all_inputs] <ports>]` — a synthesis tool's spelling of the same, which
/// the reference does NOT have — is read the same way, every named port excluded (a LOOM
/// EXTENSION, user 2026-10-31: 23 of OpenTitan's 40 constraint files constrain their inputs so).
/// Only that form: `-intersect`, or any other base collection, is not read (`None`). A name
/// matching no port excludes nothing, as a collection removes nothing it does not hold.
fn all_ports_except(tok: &str) -> Option<(String, Vec<String>)> {
    let inner = tok.strip_prefix('[')?.strip_suffix(']')?.trim();
    let parts = tokenize(inner);
    if let [cmd, list, objects] = parts.as_slice() {
        if cmd != "remove_from_collection" || objects.starts_with('-') {
            return None;
        }
        let accessor = match resolve_objs(list).as_slice() {
            [one] if one == "*INPUTS*" => "all_inputs_except",
            [one] if one == "*OUTPUTS*" => "all_outputs_except",
            _ => return None,
        };
        return Some((accessor.to_string(), resolve_objs(objects)));
    }
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
    let one_port = pattern.strip_prefix("[get_ports").filter(|_| pattern.ends_with(']')).map(|_| resolve_objs(pattern));
    match one_port {
        Some(p) if p.iter().any(|n| n.contains(['*', '?'])) => None,
        Some(p) if p.len() == 1 => Some((accessor.to_string(), p)),
        _ => Some((accessor.to_string(), Vec::new())),
    }
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
            "get_ports" | "get_pins" | "get_clocks" | "get_nets" | "get_cells" | "get_lib_cells" | "get_lib_pins" | "get_libs" => {
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
                let keys = crate::sdc_flags::keys(&parts[0]);
                let mut names = Vec::new();
                let mut k = 1;
                while k < parts.len() {
                    let p = &parts[k];
                    if keys.contains(&p.as_str()) {
                        k += 2;
                        continue;
                    }
                    if p.starts_with("[concat") {
                        // names computed by a nested `[concat …]` (a `get_object_name`, see
                        // `dialect_queries`) are this query's patterns
                        names.extend(resolve_objs(p));
                    } else if !p.starts_with('-') {
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

/// A clock period: a POSITIVE number, in the file's time unit — standard SDC, and the reference's
/// rule (`create_clock`, Error 572 "is not a positive float"). A frequency (`"100 MHz"`) or a
/// time with a unit (`20ns`) is a Quartus extension: the reference refuses it and so does this
/// reader, with an error that says so (accepted 2026-08 .. 2026-10, a value no other tool
/// would have timed). One level of Tcl quoting (`"10"`, `{10}`) is the number itself.
fn parse_period(v: &str) -> Option<f64> {
    let t = v.trim();
    let t = t.strip_prefix('"').and_then(|x| x.strip_suffix('"')).or_else(|| t.strip_prefix('{').and_then(|x| x.strip_suffix('}'))).unwrap_or(t);
    t.trim().parse::<f64>().ok().filter(|p| p.is_finite() && *p > 0.0)
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

/// The commands the reference knows where `read_sdc` evaluates a file, observed on the pinned
/// reference (`info commands`, 2026-10-31): `resolve.txt` is what a command word resolves against
/// (the file runs in namespace `sta`: its commands, then global), `global.txt` the global
/// namespace alone, which is all the reference's abbreviation fallback searches. Regenerate on a
/// repin.
fn known_commands() -> &'static (std::collections::HashSet<&'static str>, Vec<&'static str>) {
    static KNOWN: std::sync::OnceLock<(std::collections::HashSet<&'static str>, Vec<&'static str>)> =
        std::sync::OnceLock::new();
    KNOWN.get_or_init(|| {
        let global: Vec<&'static str> = include_str!("sdc_known/global.txt").lines().filter(|l| !l.is_empty()).collect();
        let all = include_str!("sdc_known/resolve.txt").lines().filter(|l| !l.is_empty()).chain(global.iter().copied()).collect();
        (all, global)
    })
}

/// Whether `word` names a command the reference runs where `read_sdc` evaluates a file — itself
/// or the one global command it abbreviates. A consumer that splits a script into its own commands
/// and constraints sends every such word to [`Sdc::parse`], which keeps, applies or lists it.
pub fn is_command(word: &str) -> bool {
    resolve_command(word).is_ok()
}

/// The command a word names, by the reference's rule: a command it knows, else the ONE global
/// command the word begins (`set_max_trans` is `set_max_transition`). None of them is `invalid
/// command name`, several is `Ambiguous command name`, and either stops the reference's read.
fn resolve_command(word: &str) -> Result<String, String> {
    let (known, global) = known_commands();
    if known.contains(word) {
        return Ok(word.to_string());
    }
    let mut m: Vec<&str> = global.iter().copied().filter(|c| c.starts_with(word)).collect();
    match m.len() {
        1 => Ok(m[0].to_string()),
        0 => Err(format!("invalid command name \"{word}\"")),
        _ => {
            m.sort_unstable();
            Err(format!("Ambiguous command name \"{word}\": {}", m.join(" ")))
        }
    }
}

/// The first command word in `line` the reference does not know, with its error: the line's own
/// command and every `[...]` substitution outside braces, at any depth, in the order Tcl evaluates
/// them (a substitution in an argument runs before the command it is an argument of). A bus
/// subscript (`[0]`, `[7:0]`, `[*]`) is the reference's literal, not a command.
fn unknown_command(line: &str) -> Option<String> {
    unknown_substituted_command(line).or_else(|| resolve_command(&tokenize(line.trim()).into_iter().next()?).err())
}

/// The reference's query aliases (`define_cmd_alias` in its SDC Tcl): each singular spelling is
/// the plural query, nothing else.
const QUERY_ALIASES: [(&str, &str); 8] = [
    ("get_cell", "get_cells"),
    ("get_clock", "get_clocks"),
    ("get_lib_cell", "get_lib_cells"),
    ("get_lib_pin", "get_lib_pins"),
    ("get_lib", "get_libs"),
    ("get_net", "get_nets"),
    ("get_pin", "get_pins"),
    ("get_port", "get_ports"),
];

/// `line` with each substituted query word (`[get_cell …]`) spelled as the query it names
/// (`[get_cells …]`): a singular spelling — legal SDC since 1.5 — is an alias of the plural query
/// in the reference ([`QUERY_ALIASES`]), and an abbreviation resolves to the ONE global command it
/// begins (`sta_unknown`); matched by exact name here, both were refused. Inside braces nothing is substituted, so nothing
/// is rewritten; a word that names no query, or more than one, is left for the read checks.
fn canonical_queries(line: &str) -> String {
    let b = line.as_bytes();
    let mut out = String::with_capacity(line.len());
    let (mut depth, mut i, mut last) = (0usize, 0usize, 0usize);
    while i < b.len() {
        match b[i] {
            b'\\' => i += 1,
            b'{' => depth += 1,
            b'}' => depth = depth.saturating_sub(1),
            b'[' if depth == 0 => {
                let start = i + 1;
                let end = line[start..].find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).map_or(b.len(), |e| start + e);
                let word = &line[start..end];
                if !word.is_empty() && !KNOWN_QUERIES.contains(&word) && word.starts_with("get_") {
                    let full = QUERY_ALIASES
                        .iter()
                        .find(|(a, _)| *a == word)
                        .map(|(_, q)| q.to_string())
                        .or_else(|| resolve_command(word).ok());
                    // a signoff timer abbreviates ITS commands too (`get_attr`): a word the
                    // reference does not know that begins exactly one extension query is that
                    // query, refused by name below with the read going on
                    let ext = || {
                        let m: Vec<&str> = EXTENSION_QUERIES.iter().copied().filter(|q| q.starts_with(word)).collect();
                        (full.is_none() && m.len() == 1).then(|| m[0])
                    };
                    if let Some(q) = ext() {
                        out.push_str(&line[last..start]);
                        out.push_str(q);
                        last = end;
                    } else if let Some(full) = full.filter(|f| KNOWN_QUERIES.contains(&f.as_str())) {
                        out.push_str(&line[last..start]);
                        out.push_str(&full);
                        last = end;
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    out.push_str(&line[last..]);
    out
}

/// Queries a synthesis tool has and the reference does not, known here so that a constraint using
/// one is refused by name and the read goes on (user 2026-10-31), not stopped as at an unknown
/// command. `remove_from_collection` is read in one form ([`all_ports_except`]). `load_of
/// <lib>/<cell>/<pin>` (a library pin's capacitance) is evaluated when the libraries are given
/// ([`subst_load_of`], user 2026-11-04); without them its command is refused (listed with the
/// queries not evaluated).
///
/// The rest are a synthesis tool's collection commands, known so that a
/// constraint using one is refused by name, with the read going on: `get_designs` (a design, not
/// a timing object), and the ones that select by attribute, count, index or the design's
/// connectivity, which this reader does not evaluate. `add_to_collection` and `get_object_name`
/// are rewritten to `list` / `concat` before this check ([`dialect_queries`]).
const EXTENSION_QUERIES: [&str; 10] = [
    "remove_from_collection", "load_of", "get_designs", "filter_collection", "get_attribute",
    "sizeof_collection", "index_collection", "sort_collection", "all_fanin", "all_fanout",
];

/// `line` with each `[load_of lib/cell/pin]` outside braces replaced by the library pin's
/// capacitance (in the library's unit, as `set_load` reads its value), when the design in scope
/// has the libraries. One it cannot answer is left as written, and its command is refused with
/// the reason ([`unevaluated_query`]).
fn subst_load_of(line: &str) -> String {
    if !line.contains("[load_of ") {
        return line.to_string();
    }
    let cs: Vec<char> = line.chars().collect();
    let (mut out, mut depth, mut i) = (String::new(), 0usize, 0usize);
    while i < cs.len() {
        let c = cs[i];
        match c {
            '{' => depth += 1,
            '}' => depth = depth.saturating_sub(1),
            '[' if depth == 0 => {
                let rest: String = cs[i..].iter().collect();
                if let Some(inner) = rest.strip_prefix("[load_of ").and_then(|r| r.split_once(']')).map(|(a, _)| a.to_string()) {
                    let arg = inner.trim().trim_matches(|c| c == '{' || c == '}').to_string();
                    if !arg.contains(['[', ' ']) {
                        if let Some(Ok(cap)) = with_design(|d| d.load_of(&arg)) {
                            // the library holds capacitances as floats (as the reference's
                            // Liberty reader does): written in the float's shortest form, the
                            // library's own text (`0.0015`, not `0.001500000013038516`)
                            out.push_str(&(cap as f32).to_string());
                            i += "[load_of ".len() + inner.len() + 1;
                            continue;
                        }
                    }
                }
            }
            _ => {}
        }
        out.push(c);
        i += 1;
    }
    out
}

/// `line` with a synthesis tool's collection spellings written as the Tcl lists they are, outside
/// braces: `[add_to_collection ?-unique? A B]` is `[list A B]` (`-unique` only drops a repeat,
/// which an object list already ignores), and `[get_object_name C]` is `[concat C]`, the names.
fn dialect_queries(line: &str) -> String {
    const SPELLED: [(&str, &str); 2] = [("add_to_collection", "list"), ("get_object_name", "concat")];
    let b = line.as_bytes();
    let mut out = String::with_capacity(line.len());
    let (mut depth, mut i, mut last) = (0usize, 0usize, 0usize);
    while i < b.len() {
        match b[i] {
            b'\\' => i += 1,
            b'{' => depth += 1,
            b'}' => depth = depth.saturating_sub(1),
            b'[' if depth == 0 => {
                let start = i + 1;
                let end = line[start..].find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).map_or(b.len(), |e| start + e);
                if let Some((_, to)) = SPELLED.iter().find(|(from, _)| *from == &line[start..end]) {
                    out.push_str(&line[last..start]);
                    out.push_str(to);
                    last = end;
                    let rest = &line[end..];
                    let trimmed = rest.trim_start();
                    if *to == "list" && trimmed.starts_with("-unique") && trimmed[7..].starts_with(char::is_whitespace) {
                        last = end + (rest.len() - trimmed.len()) + 7;
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    out.push_str(&line[last..]);
    out
}

/// [`unknown_command`] in the `[...]` substitutions of `line` only, not its own command word.
fn unknown_substituted_command(line: &str) -> Option<String> {
    let cs: Vec<char> = line.chars().collect();
    let is_subscript = |s: &str| {
        let s = s.trim();
        !s.is_empty() && (s.chars().all(|c| c.is_ascii_digit()) || s == "*" || {
            let mut p = s.splitn(2, ':');
            matches!((p.next(), p.next()), (Some(a), Some(b)) if !a.is_empty() && !b.is_empty()
                && a.chars().all(|c| c.is_ascii_digit()) && b.chars().all(|c| c.is_ascii_digit()))
        })
    };
    // the substitutions at this level, as (inner text), skipping braced words and escapes
    let mut subs = Vec::new();
    let (mut i, mut brace) = (0usize, 0i32);
    while i < cs.len() {
        match cs[i] {
            '\\' => i += 1,
            '{' => brace += 1,
            '}' => brace -= 1,
            '[' if brace == 0 => {
                let (start, mut depth, mut b) = (i + 1, 0i32, 0i32);
                while i < cs.len() {
                    match cs[i] {
                        '\\' => i += 1,
                        '{' => b += 1,
                        '}' => b -= 1,
                        '[' if b == 0 => depth += 1,
                        ']' if b == 0 => {
                            depth -= 1;
                            if depth == 0 {
                                break;
                            }
                        }
                        _ => {}
                    }
                    i += 1;
                }
                subs.push(cs[start..i.min(cs.len())].iter().collect::<String>());
            }
            _ => {}
        }
        i += 1;
    }
    for inner in &subs {
        if is_subscript(inner) {
            continue;
        }
        // a query this reader knows beyond the reference's (`remove_from_collection`, see
        // `all_ports_except`; `load_of`, never evaluated): its own arguments are still checked,
        // and a form it does not read is refused with its command rather than stopping the read
        let own = tokenize(inner.trim()).first().is_some_and(|c| EXTENSION_QUERIES.contains(&c.as_str()));
        if let Some(why) = if own { unknown_substituted_command(inner) } else { unknown_command(inner) } {
            return Some(why);
        }
    }
    None
}

/// Abbreviations, expanded as the reference expands them before a command runs: a command name
/// the reference does not know is the one global command it begins ([`resolve_command`]:
/// `set_max_trans`), and an argument that is not one of the command's keys or flags exactly is
/// the FIRST key it begins, else the first flag, in the reference's order (`-per` is `-period`).
/// Lists the reference parses on their own (`-through`, `-group`) are left as written.
///
/// ⛔ Unexpanded, `create_clock -per 10` failed the whole file, and `set_max_trans 0.5` was an
/// unknown command whose limit was lost.
fn expand_abbreviations(mut toks: Vec<String>) -> Vec<String> {
    const OWN: [&str; 4] = ["-through", "-rise_through", "-fall_through", "-group"];
    let Some(c) = toks.first() else { return toks };
    if let Ok(full) = resolve_command(c) {
        toks[0] = full;
    }
    let Some((keys, flags)) = crate::sdc_flags::options(&toks[0]) else { return toks };
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
    // SDC spellings the model reads as another: the parser's `-substract_pin_load` is
    // `-subtract_pin_load`, and its `set_false_path -rise_throough` / `-fall_throough` (SDC 1.7
    // on) are `-rise_through` / `-fall_through`; `get_ports -hierarchical` is `get_ports` (a port
    // is at the top level, with no hierarchy below it to search), so the flag selects nothing
    // more and is not kept
    match toks[0].as_str() {
        "set_false_path" => {
            for t in toks.iter_mut() {
                match t.as_str() {
                    "-rise_throough" => *t = "-rise_through".into(),
                    "-fall_throough" => *t = "-fall_through".into(),
                    _ => {}
                }
            }
        }
        "set_load" => {
            for t in toks.iter_mut().filter(|t| *t == "-substract_pin_load") {
                *t = "-subtract_pin_load".into();
            }
        }
        "get_ports" => toks.retain(|t| t != "-hierarchical"),
        _ => {}
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
        let known = KNOWN_QUERIES.contains(&cmd.as_str()) || cmd == "expr" || (matches!(cmd.as_str(), "lsearch" | "remove_from_collection") && all_ports_except(t).is_some());
        if cmd == "load_of" {
            let arg = parts.get(1).cloned().unwrap_or_default();
            let why = with_design(|d| d.load_of(&arg).err()).flatten().unwrap_or_else(|| "no library was given".into());
            return Some(format!("[{}] (a library pin's capacitance, not evaluated: {why})", inner.trim()));
        }
        if !known {
            return Some(format!("[{}] (a command this reader does not evaluate)", inner.trim()));
        }
        // `check_path_divider`: an `-hsc` divider is ONE of the SDC dividers, or the query errors
        if cmd.starts_with("get_") {
            if let Some(v) = parts.iter().position(|p| p == "-hsc").map(|i| parts.get(i + 1).map_or("", String::as_str)) {
                if !(v.chars().count() == 1 && "/@^#.|".contains(v)) {
                    return Some(format!("[{}] (hierarchy separator must be one of '/@^#.|' (Error 342))", inner.trim()));
                }
            }
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

/// An every-port-but collection ([`all_ports_except`]) anywhere but as the ports of
/// `set_input_delay` / `set_output_delay`, the one place it is read: elsewhere its words were
/// taken as object names (`-from [remove_from_collection [all_inputs] clk]` cut paths from a port
/// called `remove_from_collection`).
fn misplaced_except(toks: &[String]) -> Option<String> {
    fn find(toks: &[String]) -> Option<String> {
        toks.iter().find_map(|t| {
            let inner = t.strip_prefix('[').and_then(|s| s.strip_suffix(']'))?;
            if all_ports_except(t).is_some() {
                return Some(format!("[{}] (an every-port-but collection, read only as the ports of set_input_delay / set_output_delay)", inner.trim()));
            }
            find(tokenize(inner.trim()).get(1..).unwrap_or_default())
        })
    }
    if matches!(toks.first()?.as_str(), "set_input_delay" | "set_output_delay") {
        return None;
    }
    find(&toks[1..])
}

/// The words of a `set_clock_groups` the reference warns about and ignores (sdc/Sdc.tcl
/// `set_clock_groups`): after its keys (`-name`, `-comment`) and flags are taken, it walks the
/// rest — `-group` takes the next word; any other option is STA-0402 "unknown keyword argument"
/// and any other word STA-0403 "extra positional argument", each skipped alone. The command is
/// still applied.
fn clock_groups_ignored_words(toks: &[String]) -> Vec<String> {
    let (keys, flags) = crate::sdc_flags::options("set_clock_groups").unwrap_or_default();
    let mut out = Vec::new();
    let mut k = 1;
    while k < toks.len() {
        let t = toks[k].as_str();
        if keys.contains(&t) || t == "-group" {
            k += 2;
            continue;
        }
        if !flags.contains(&t) {
            out.push(t.to_string());
        }
        k += 1;
    }
    out
}

/// The first option in `toks` (at any depth of `[...]`) that is not one of its command's options,
/// as the reference's argument parser decides it: a word is an option when it is `-` and a letter
/// (`-0.5` is a value); it is one of the command's keys (which takes the next word as its value)
/// or flags exactly, or else the beginning of one. Anything else fails the command in the
/// reference (Error 562, "not a known keyword or flag"), so it builds no constraint; read as a
/// constraint here it ignored the option. The options SDC declares and the reference timer lacks
/// (`all_inputs -edge_triggered`, `set_driving_cell -clock`, `get_ports -hierarchical`:
/// [`crate::sdc_flags::STANDARD_ONLY`]) are valid SDC and are accepted. A command whose parse
/// passes unknown options on ([`PASSES_UNKNOWN`]), or that this table does not list, is not judged.
fn unknown_option(toks: &[String]) -> Option<String> {
    use crate::sdc_flags::PASSES_UNKNOWN;
    let is_option = |t: &str| t.len() >= 2 && t.starts_with('-') && t[1..].starts_with(|c: char| c.is_ascii_alphabetic());
    let cmd = toks.first()?.as_str();
    // The commands whose reference parse passes unknown options on parse their own lists after
    // it: those lists are their options too (SDC declares `-through` and its edge forms on the
    // exceptions, `-group` on `set_clock_groups`), and anything else is not one. The senses keep
    // the reference's rule: SDC declares neither.
    // `set_clock_groups` is not judged here: the reference warns on an option it does not know
    // and applies the command without it ([`clock_groups_ignored_words`]).
    let own: &[&str] = match cmd {
        "set_clock_groups" => &[],
        "set_sense" | "set_clock_sense" => &[],
        c if PASSES_UNKNOWN.contains(&c) => &["-through", "-rise_through", "-fall_through"],
        _ => &[],
    };
    let judged = cmd != "set_clock_groups" && (!PASSES_UNKNOWN.contains(&cmd) || !own.is_empty());
    if let Some((mut keys, flags)) = crate::sdc_flags::options(cmd).filter(|_| judged) {
        keys.extend_from_slice(own);
        let mut k = 1;
        while k < toks.len() {
            let t = toks[k].as_str();
            if is_option(t) {
                if keys.contains(&t) || (!flags.contains(&t) && keys.iter().any(|x| x.starts_with(t))) {
                    k += 2;
                    continue;
                }
                if !flags.iter().any(|x| x.starts_with(t)) {
                    return Some(format!("`{cmd} {t}`: {t} is not an option of {cmd}"));
                }
            }
            k += 1;
        }
    }
    toks[1..].iter().find_map(|t| {
        let inner = t.strip_prefix('[').and_then(|s| s.strip_suffix(']'))?;
        unknown_option(&expand_abbreviations(tokenize(inner.trim())))
    })
}

/// The reference-timer extensions `toks` uses, at any depth of `[...]`: the command itself when no
/// SDC version declares it, and each option of it no SDC version declares.
fn extensions_in(toks: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    let Some(cmd) = toks.first() else { return out };
    if crate::sdc_flags::EXTENSION_COMMANDS.contains(&cmd.as_str()) {
        out.push(cmd.clone());
    }
    for t in &toks[1..] {
        if crate::sdc_flags::EXTENSION_OPTIONS.iter().any(|(c, o)| c == cmd && o == t) {
            out.push(format!("{cmd} {t}"));
        }
        if let Some(inner) = t.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            out.extend(extensions_in(&expand_abbreviations(tokenize(inner.trim()))));
        }
    }
    out
}

/// Each SDC option in `toks` (at any depth of `[...]`) that no consumer here applies
/// ([`crate::sdc_flags::KEPT_OPTIONS`]), as `command -option`, in the order written.
fn kept_options_in(toks: &[String]) -> Vec<String> {
    let mut out = Vec::new();
    let Some(cmd) = toks.first() else { return out };
    for t in &toks[1..] {
        if crate::sdc_flags::KEPT_OPTIONS.iter().any(|(c, o)| c == cmd && o == t) {
            out.push(format!("{cmd} {t}"));
        }
        if let Some(inner) = t.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
            out.extend(kept_options_in(&expand_abbreviations(tokenize(inner.trim()))));
        }
    }
    out
}

/// The first query in `toks` (at any depth of `[...]`) given positional arguments its definition
/// refuses: `all_clocks` takes none and no option (Tcl: wrong # args), `all_outputs` and
/// `all_registers` options only (Error 564, "positional arguments not supported"; SDC gives
/// `all_outputs` the options `-clock`, `-level_sensitive` and `-edge_triggered`, which the
/// reference timer's lacks). `all_inputs` ignores a stray word, as the reference does.
fn argument_fault(toks: &[String]) -> Option<String> {
    let cmd = toks.first()?.as_str();
    let fault = match cmd {
        "all_clocks" => toks.len() > 1,
        // SDC's `all_outputs` takes its options (`-clock`, `-level_sensitive`, `-edge_triggered`)
        // and no positional; the reference timer's takes nothing
        "all_outputs" | "all_registers" => {
            let keys = crate::sdc_flags::keys(cmd);
            let mut k = 1;
            let mut positional = false;
            while k < toks.len() {
                let t = toks[k].as_str();
                if !t.starts_with('-') {
                    positional = true;
                    break;
                }
                k += if keys.iter().any(|x| x.starts_with(t)) { 2 } else { 1 };
            }
            positional
        }
        _ => false,
    };
    if fault {
        return Some(format!("`{}`: {cmd} takes no positional argument{}", toks.join(" "), if cmd == "all_clocks" { " or option" } else { "" }));
    }
    toks[1..].iter().find_map(|t| {
        let inner = t.strip_prefix('[').and_then(|s| s.strip_suffix(']'))?;
        argument_fault(&expand_abbreviations(tokenize(inner.trim())))
    })
}

/// Why the reference's `set_clock_uncertainty` fails on these arguments, in the order it checks:
/// no value (Error 420); a value that is not a float (571); a `-from` side without a `-to` side or
/// the reverse (421); with both, a positional besides the value (565); with neither, not exactly
/// the value and the objects (567), or `-rise`/`-fall`, which only edge a `-to` (422). Without
/// these the reader applied an uncertainty the reference never set, from `-from` alone onto no
/// clock pair, or a single-clock one the reference refused for its `-rise`.
fn clock_uncertainty_fault(with: &[(&str, &String)], flags: &[String], pos: &[&String]) -> Option<String> {
    let has = |f: &str| flags.iter().any(|x| x == f);
    let side = |ks: [&str; 3]| with.iter().any(|(k, _)| ks.contains(k));
    let Some(value) = pos.first() else { return Some("missing uncertainty value (Error 420)".into()) };
    if value.parse::<f64>().is_err() {
        return Some(format!("uncertainty '{value}' is not a float (Error 571)"));
    }
    match (side(["-from", "-rise_from", "-fall_from"]), side(["-to", "-rise_to", "-fall_to"])) {
        (true, false) | (false, true) => Some("-from/-to must be used together (Error 421)".into()),
        (true, true) => (pos.len() != 1).then(|| "-from/-to requires one positional argument (Error 565)".into()),
        (false, false) if pos.len() != 2 => Some("requires two positional arguments (Error 567)".into()),
        (false, false) => (has("-rise") || has("-fall"))
            .then(|| "-rise, -fall options not allowed for single clock uncertainty (Error 422)".into()),
    }
}

/// The first WORD of a command that cannot name what it appears to, by Tcl's word rules, with why.
///
/// A Tcl word is a CONCATENATION of parts — literal text, `[command]` substitutions, `$variables` —
/// so `[get_ports a]0` is the collection's text with `0` after it, not port `a`; and a braced word is
/// literal, so `{[get_ports a]}` is the two names `[get_ports` and `a]`, not a query. The tokenizer
/// keeps a word's groups apart (and strips braces), which read the first as port `a` plus a stray
/// `0` and the second as the query. The reference (OpenSTA reading through Tcl, measured on the
/// spec-matrix probes 2026-10-31): `-from [get_ports din]0` and `[get_ports dout][get_ports din]`
/// are Error 100 "unsupported object type"; `x[get_ports dout]` and `{[get_ports dout]}` name
/// objects that are not found (Warning 101), so nothing is applied. Either way no constraint is
/// built; here the command is refused, saying which word and why.
///
/// A bus index is literal, as before: `a[0]`, `d[7:0]` (the reference's unknown-command handler
/// gives `[0]` back as written). `{*}` expansion and variables (substituted before this runs) are
/// not faults.
fn word_fault(line: &str) -> Option<String> {
    let cs: Vec<char> = line.chars().collect();
    // the end of a `[...]` group starting at `i` (the index past its `]`), nesting- and brace-aware
    let close_bracket = |mut i: usize| -> usize {
        let (mut depth, mut brace) = (0i32, 0i32);
        while i < cs.len() {
            match cs[i] {
                '\\' => i += 1,
                '{' => brace += 1,
                '}' => brace -= 1,
                '[' if brace == 0 => depth += 1,
                ']' if brace == 0 => {
                    depth -= 1;
                    if depth == 0 {
                        return i + 1;
                    }
                }
                _ => {}
            }
            i += 1;
        }
        cs.len()
    };
    let is_index = |inner: &str| !inner.is_empty() && inner.chars().all(|c| c.is_ascii_digit() || c == ':');
    let mut i = 0;
    while i < cs.len() {
        while i < cs.len() && cs[i].is_whitespace() {
            i += 1;
        }
        if i >= cs.len() || cs[i] == '#' {
            break; // the tokenizer's trailing comment
        }
        let start = i;
        if cs[i..].starts_with(&['{', '*', '}']) {
            i += 3; // `{*}` expansion: the word that follows is an ordinary word
            continue;
        }
        if cs[i] == '{' {
            let mut depth = 0i32;
            while i < cs.len() {
                match cs[i] {
                    '\\' => i += 1,
                    '{' => depth += 1,
                    '}' => {
                        depth -= 1;
                        if depth == 0 {
                            i += 1;
                            break;
                        }
                    }
                    _ => {}
                }
                i += 1;
            }
            let word: String = cs[start..i.min(cs.len())].iter().collect();
            if i < cs.len() && !cs[i].is_whitespace() {
                return Some(format!("`{word}…`: extra characters after a close-brace (a Tcl error)"));
            }
            // an unclosed brace (`… ignored {` at the end of a line) is the word `{` alone, and the
            // last character may be any character: strip by character, never by byte offset
            let inner = word.strip_prefix('{').map_or(word.as_str(), |w| w.strip_suffix('}').unwrap_or(w));
            if inner.split_whitespace().any(|e| e.starts_with('[') && e[1..].starts_with(|c: char| c.is_ascii_alphabetic())) {
                return Some(format!("`{word}`: braces stop substitution, so this is the literal names {:?}, not a query", inner.split_whitespace().collect::<Vec<_>>()));
            }
            continue;
        }
        // a quoted or bare word: count its parts
        let quoted = cs[i] == '"';
        if quoted {
            i += 1;
        }
        let (mut subs, mut text) = (Vec::new(), false);
        while i < cs.len() {
            let c = cs[i];
            if quoted && c == '"' {
                i += 1;
                break;
            }
            if !quoted && c.is_whitespace() {
                break;
            }
            if c == '\\' {
                text = true;
                i += 2;
                continue;
            }
            if c == '[' {
                let end = close_bracket(i);
                let inner: String = cs[i + 1..end.saturating_sub(1).max(i + 1)].iter().collect();
                if text && is_index(&inner) {
                    i = end; // a bus index, part of the name
                    continue;
                }
                subs.push(inner);
                i = end;
                continue;
            }
            text = true;
            i += 1;
        }
        let word: String = cs[start..i.min(cs.len())].iter().collect();
        if quoted && i < cs.len() && !cs[i].is_whitespace() {
            return Some(format!("`{word}…`: extra characters after a close-quote (a Tcl error)"));
        }
        if !subs.is_empty() && (subs.len() > 1 || text) {
            return Some(format!(
                "`{word}`: a word joining a [command] to other text is that text concatenated (a string), not the command's objects"
            ));
        }
        for s in &subs {
            if let Some(f) = word_fault(s) {
                return Some(f);
            }
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

/// Tcl's error for reading `var` (as [`unresolved_var`] names it: `$X`, `$::env(X)`), word for
/// word as tclsh 8.6 gives it: `can't read "X": no such variable`; for the environment array, whose
/// read trace raises the error, `no such variable` and the trace line.
pub(super) const NO_SUCH_VARIABLE: &str = "no such variable";

pub(super) fn no_such_variable(var: &str) -> String {
    let name = var.trim_start_matches('$');
    if name.starts_with("::env(") || name.starts_with("env(") {
        format!("no such variable (read trace on \"{name}\")")
    } else {
        format!("can't read \"{name}\": no such variable")
    }
}

/// The first variable an executed line reads that is not defined: outside braces (Tcl substitutes
/// nothing inside them), and inside the braced expression of an `[expr {…}]`, which `expr`
/// substitutes itself.
pub(super) fn undefined_read(raw: &str, vars: &HashMap<String, String>) -> Option<String> {
    // the tool-name variable: read as the signoff timer's name when undefined (`tcl::TOOL_NAME_DEFAULT`)
    let mut known = vars.clone();
    known.entry(tcl::TOOL_NAME_VAR.to_string()).or_default();
    if let Some(v) = unresolved_var(raw, &known) {
        return Some(v);
    }
    // `[expr {…}]` outside braces: the braced expression is substituted by `expr` itself
    let cs: Vec<char> = raw.chars().collect();
    let (mut depth, mut i) = (0usize, 0usize);
    while i < cs.len() {
        match cs[i] {
            '\\' => i += 1,
            '{' => depth += 1,
            '}' => depth = depth.saturating_sub(1),
            '[' if depth == 0 => {
                let rest: String = cs[i + 1..].iter().collect();
                if let Some(e) = rest.strip_prefix("expr ").map(str::trim_start).filter(|e| e.starts_with('{')) {
                    let toks = tokenize(e.split(']').next().unwrap_or(""));
                    if let Some(v) = toks.first().and_then(|t| unresolved_var(t, &known)) {
                        return Some(v);
                    }
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// Why the reference's `create_generated_clock` stops before making the clock, checked in its
/// order (`sdc/Sdc.tcl`, Errors 374–390), as far as the text decides it. ⛔ Read as a clock anyway,
/// a PLL output declared with no `-source` (an FPGA tool infers it) became a clock of period 0.
fn generated_clock_fault(val: &dyn Fn(&str) -> Option<String>, has: &dyn Fn(&str) -> bool, no_objects: bool) -> Option<String> {
    let int_ge1 = |v: &str| v.trim().parse::<i64>().is_ok_and(|n| n >= 1);
    let (divide, multiply, edges) = (val("-divide_by"), val("-multiply_by"), val("-edges"));
    let (divide, multiply, edges) = (divide.as_deref(), multiply.as_deref(), edges.as_deref());
    let combinational = has("-combinational");
    if no_objects {
        return Some("Error 374: empty ports/pins/nets argument".into());
    }
    if val("-name").is_none() && has("-add") {
        return Some("Error 375: -add requires -name".into());
    }
    if val("-source").is_none() {
        return Some("Error 377: missing -source argument".into());
    }
    if val("-master_clock").is_some_and(|m| m.trim().is_empty() || m.trim() == "{}") {
        return Some("Error 378: -master_clock argument empty".into());
    }
    if val("-master_clock").is_none() && has("-add") {
        return Some("Error 379: -add requires -master_clock".into());
    }
    if divide.is_some() && multiply.is_some() {
        return Some("Error 380: -multiply_by and -divide_by options are exclusive".into());
    }
    if let Some(d) = divide {
        if !int_ge1(d) {
            return Some("Error 381: -divide_by is not an integer greater than one".into());
        }
        if combinational && d.trim() != "1" {
            return Some("Error 382: -combinational implies -divide_by 1".into());
        }
    } else if let Some(m) = multiply {
        if !int_ge1(m) {
            return Some("Error 383: -multiply_by is not an integer greater than one".into());
        }
        if let Some(dc) = val("-duty_cycle").as_deref() {
            if !dc.trim().parse::<f64>().is_ok_and(|v| (0.0..=100.0).contains(&v)) {
                return Some("Error 384: -duty_cycle is not a float between 0 and 100".into());
            }
        }
    } else if let Some(e) = edges {
        let words = list_words(e);
        let es: Vec<&str> = words.split_whitespace().collect();
        if es.len() != 3 {
            return Some("Error 385: -edges only supported for three edges".into());
        }
        let nums: Vec<Option<i64>> = es.iter().map(|x| x.parse().ok()).collect();
        if nums.iter().any(Option::is_none) || nums.windows(2).any(|w| w[1] <= w[0]) {
            return Some("Error 386: edges times are not monotonically increasing".into());
        }
        if let Some(s) = val("-edge_shift") {
            if list_words(&s).split_whitespace().count() != 3 {
                return Some("Error 387: -edge_shift length does not match -edges length".into());
            }
        }
    } else if !combinational {
        return Some("Error 388: missing -multiply_by, -divide_by, -combinational or -edges argument".into());
    }
    if has("-invert") && divide.is_none() && multiply.is_none() && !combinational {
        return Some("Error 389: cannot specify -invert without -multiply_by, -divide_by or -combinational".into());
    }
    if val("-duty_cycle").is_some() && multiply.is_none() {
        return Some("Error 390: -duty_cycle requires -multiply_by value".into());
    }
    None
}

/// `[llength X]` in a condition as Tcl answers it, X a query (`[get_ports -quiet io_te]`, written
/// in place or through a variable `set` to it) or a plain list. A query is counted on the design
/// in scope (`parse_with`); with no design, or a query the design cannot answer (clocks), `None`
/// and the `if` refuses. A flow's `if {[llength $te_port] > 0} { set_case_analysis 0 $te_port }`
/// guard is how a file stays valid on a block without the port.
fn subst_llength(cond: &str, vars: &HashMap<String, String>) -> Option<String> {
    let mut out = String::new();
    let mut rest = cond;
    while let Some(k) = rest.find("[llength ") {
        out.push_str(&rest[..k]);
        let tail = &rest[k + "[llength ".len()..];
        let mut depth = 0usize;
        let mut end = None;
        for (i, c) in tail.char_indices() {
            match c {
                '[' => depth += 1,
                ']' if depth == 0 => {
                    end = Some(i);
                    break;
                }
                ']' => depth -= 1,
                _ => {}
            }
        }
        let e = end?;
        let arg = tail[..e].trim();
        let val = match arg.strip_prefix('$') {
            Some(n) => vars.get(n.trim_start_matches("::").trim_start_matches('{').trim_end_matches('}'))?.clone(),
            // a literal word in braces is its contents, as Tcl hands it to `llength`
            None => arg.strip_prefix('{').and_then(|a| a.strip_suffix('}')).unwrap_or(arg).to_string(),
        };
        let val = val.trim();
        let n = match val.strip_prefix('[').and_then(|v| v.strip_suffix(']')) {
            Some(q) => {
                let parts = query_words(q);
                if !parts.first().is_some_and(|c| c.starts_with("get_") && c != "get_clocks") {
                    return None;
                }
                with_design(|d| crate::sdc_query::eval(&parts, d))?.ok()?.len()
            }
            None if val.contains('[') || val.contains('$') => return None,
            None => tokenize(val).len(),
        };
        out.push_str(&n.to_string());
        rest = &tail[e + 1..];
    }
    out.push_str(rest);
    Some(out)
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
    /// Tcl's error, which stops the read (a condition reading an undefined variable).
    Error(String),
    None,
}

/// The first variable a braced `expr` condition reads that is not defined (`expr` substitutes it
/// itself). The tool-name variable is the exception this reader makes on purpose: undefined, a
/// condition reads it as the signoff timer's name (`tcl::TOOL_NAME_DEFAULT`).
fn undefined_in_condition(cond: &str, vars: &HashMap<String, String>) -> Option<String> {
    let mut known = vars.clone();
    known.entry(tcl::TOOL_NAME_VAR.to_string()).or_default();
    unresolved_var(cond, &known)
}

/// `if` and `foreach`, evaluated as the reference's Tcl would — where they can be.
///
/// `if {cond} {body} ?elseif {cond} {body}? ?else {body}?` runs the first branch whose condition
/// holds; `foreach v {a b} {body}` runs its body once per element with `v` set. A condition or a
/// list this reader cannot evaluate (a design query, a `proc`) REFUSES the whole command — the
/// branch it would have taken is unknown, and running every branch is how a file's default and
/// its override both applied, in file order.
fn control(raw: &str, vars: &HashMap<String, String>, procs: &HashMap<String, TclProc>) -> Control {
    let head = raw.split_whitespace().next().unwrap_or("");
    match head {
        "if" => {
            let toks = tokenize(raw);
            // a vendor file's tool-name test reads a signoff timer's name when the flow does not
            // define it (`tcl::TOOL_NAME_DEFAULT`); `info exists` still sees only what is defined
            let mut operands = as_operands(vars);
            operands.entry(tcl::TOOL_NAME_VAR.to_string()).or_insert_with(|| format!("\"{}\"", tcl::TOOL_NAME_DEFAULT));
            let mut k = 1;
            while k + 1 < toks.len() {
                if let Some(v) = undefined_in_condition(&toks[k], vars) {
                    return Control::Error(format!("{} in the condition `{}`", no_such_variable(&v), toks[k]));
                }
                // `[dict exists …]`, `[a_proc …]` in the condition, answered first
                let Ok(valued) = tcl::subst_values(&toks[k], vars, procs, &ReadOpts::default(), 0) else {
                    return Control::Refuse("if".into());
                };
                let Some(counted) = subst_llength(&valued, vars) else {
                    return Control::Refuse("if".into());
                };
                let cond = subst_vars(&subst_info_exists(&counted, vars), &operands);
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
            let tag = format!("f{}", FOREACH_TAGS.fetch_add(1, std::sync::atomic::Ordering::Relaxed));
            let mut out = Vec::new();
            for e in list.split_whitespace() {
                out.push(format!("set {} {{{e}}}", toks[1]));
                out.extend(body.iter().cloned());
                out.push(format!("{LOOP_CONT}{tag}"));
            }
            out.push(format!("{LOOP_END}{tag}"));
            Control::Run(out)
        }
        _ => Control::None,
    }
}

/// The variables as `expr` reads them in a BRACED condition: each value one operand, so a string
/// value is a string (`{$dir == "in"}` compares the string `in`), not words to parse. A number
/// stays bare; a value holding a `"` is left as text, which the expression then refuses.
fn as_operands(vars: &HashMap<String, String>) -> HashMap<String, String> {
    vars.iter()
        .map(|(k, v)| {
            let v = if Val::of(v) != Val::Str(v.trim().to_string()) || v.contains('"') { v.clone() } else { format!("\"{v}\"") };
            (k.clone(), v)
        })
        .collect()
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

/// [`subst_vars`] as Tcl substitutes a command: NOT inside a braced word. A `{` that begins a word
/// (after a space, `[`, `;` or at the start) opens a literal to its matching `}`, so
/// `[get_ports {$P}]` names the port `$P`, as the reference reads it ("port '$P' not found"); a
/// brace elsewhere, or inside quotes, is text, and `${name}` is the variable. `expr` substitutes
/// its own braced arguments ([`subst_exprs`]). ⛔ Substituted, OpenTitan's
/// `remove_from_collection [all_inputs] {${MAIN_CLK_PIN}}` removed the clock port the reference
/// keeps.
fn subst_command_vars(line: &str, vars: &HashMap<String, String>) -> String {
    let cs: Vec<char> = line.chars().collect();
    let (mut out, mut seg, mut quoted, mut i) = (String::new(), String::new(), false, 0usize);
    let word_start = |out: &str, seg: &str| seg.chars().last().or_else(|| out.chars().last()).is_none_or(|c| c.is_whitespace() || c == '[' || c == ';');
    while i < cs.len() {
        let c = cs[i];
        match c {
            '\\' => {
                seg.extend(cs[i..(i + 2).min(cs.len())].iter());
                i += 2;
                continue;
            }
            '$' if cs.get(i + 1) == Some(&'{') => {
                let end = cs[i..].iter().position(|&x| x == '}').map_or(cs.len(), |p| i + p + 1);
                seg.extend(cs[i..end].iter());
                i = end;
                continue;
            }
            '"' => quoted = !quoted,
            '{' if !quoted && word_start(&out, &seg) => {
                out.push_str(&subst_vars(&std::mem::take(&mut seg), vars));
                let mut depth = 0i32;
                while i < cs.len() {
                    match cs[i] {
                        '\\' => {
                            out.push(cs[i]);
                            i += 1;
                        }
                        '{' => depth += 1,
                        '}' => depth -= 1,
                        _ => {}
                    }
                    if let Some(&x) = cs.get(i) {
                        out.push(x);
                    }
                    i += 1;
                    if depth == 0 {
                        break;
                    }
                }
                continue;
            }
            _ => {}
        }
        seg.push(c);
        i += 1;
    }
    out.push_str(&subst_vars(&seg, vars));
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
                } else if c == ':' && cs.get(j + 1) == Some(&':') {
                    // a namespace separator is part of a Tcl variable name (`$::x`)
                    j += 2;
                    continue;
                } else if !(c.is_alphanumeric() || c == '_') {
                    break;
                }
                j += 1;
            }
            let name: String = cs[start..j].iter().collect();
            // at the top level `::x` is the global `x`, which is how `set` stores it
            match vars.get(name.trim_start_matches("::")) {
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
/// `-comment`'s text: its one word, without the braces or quotes that made it one.
fn comment_of(toks: &[String]) -> Option<String> {
    let v = flag_val(toks, "-comment")?;
    let inner = v.strip_prefix('{').and_then(|x| x.strip_suffix('}')).or_else(|| v.strip_prefix('"').and_then(|x| x.strip_suffix('"')));
    Some(inner.unwrap_or(v).to_string())
}

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

    /// An exception command's `-reset_path`: its reset runs first (see [`Sdc::reset_path`]); a
    /// reset only the design can decide is listed in [`Sdc::rejected`], the exception still made.
    fn reset_path_first(&mut self, toks: &[String], args: &[(String, Objects)], flags: &[String], both_checks: bool) {
        if flags.iter().any(|f| f == "-reset_path") {
            if let Err(why) = self.reset_path(args, flags, both_checks) {
                self.rejected.push(format!("{} -reset_path: {why}; the reset not applied", toks[0]));
            }
        }
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
        Sdc::parse_opts(text, &ReadOpts::default())
    }

    /// [`Sdc::parse`] with the flow's context: variables its setup script defines before the
    /// constraints are read, and the directory a relative `source` is also looked for in.
    pub fn parse_opts(text: &str, opts: &ReadOpts) -> Result<Sdc, SdcError> {
        let mut sdc = Sdc::default();
        let mut vars: HashMap<String, String> = opts.defines.iter().cloned().collect();
        let mut procs: HashMap<String, TclProc> = HashMap::new();
        // the variables of each proc call in progress, restored when it returns
        let mut frames: Vec<ProcFrame> = Vec::new();
        let mut sourced = 0usize;
        let mut loops: Vec<TclLoop> = Vec::new();
        // `current_instance`: the instance pin, cell and net names are read under (`None` the top)
        let mut instance_scope: Option<String> = None;

        // A QUEUE of commands, not a list: a chosen `if` branch or a `foreach` body is put back at
        // its front, so it runs in place and in order — and its variables are substituted when
        // each of its commands runs, as Tcl does, not when the `if` was read.
        let mut queue: std::collections::VecDeque<String> = logical_lines(text).into();
        // the command being read, as text, for each `ignored` entry it adds (`unmodelled_text`)
        let mut current_text = String::new();
        while let Some(raw) = queue.pop_front() {
            record_unmodelled(&mut sdc, &current_text);
            // A proc's call ends: its local variables go, the ones it declared `global` stay.
            if raw == PROC_RETURN {
                if let Some((mut outer, links)) = frames.pop() {
                    // `global x` links x to x; `upvar other local` links local to the caller's other
                    for (local, other) in links {
                        match vars.get(&local) {
                            Some(v) => outer.insert(other, v.clone()),
                            None => outer.remove(&other),
                        };
                    }
                    vars = outer;
                }
                continue;
            }
            // A sourced file's end: a `return` at its top level stops here (see `Dialect::Return`).
            if raw == SOURCE_END {
                continue;
            }
            // A take-back read from an SDC command's spelling: the command it is read as is not an
            // extension in this file.
            let (raw, sdc_spelling) = match raw.strip_prefix(tcl::SDC_SPELLING) {
                Some(line) => (line.to_string(), true),
                None => (raw, false),
            };
            current_text = format!("{}{}", subst_exprs(&subst_command_vars(&raw, &vars), &vars), referenced_values(&raw, &vars));
            // A signoff timer's take-back of every clock's delay, as the reference's per clock.
            if let Some(line) = raw.strip_prefix(tcl::EVERY_CLOCK) {
                let clocks: Vec<String> = sdc.clocks.iter().map(|c| c.name.clone()).collect();
                for c in tcl::every_clock_lines(line, &clocks).into_iter().rev() {
                    queue.push_front(c);
                }
                continue;
            }
            if raw.starts_with(LOOP_CONT) || raw.starts_with(LOOP_END) {
                continue;
            }
            // Tcl `break` / `continue`: the rest of the innermost running loop's pass is dropped,
            // through its continue marker; `break` drops on through the loop's stop line. One
            // with no loop running in its proc or file is Tcl's error, which stops the read.
            if matches!(raw.trim(), "break" | "continue") {
                let word = raw.trim().to_string();
                let marker = queue.iter().position(|q| q.starts_with(LOOP_CONT) || q == PROC_RETURN || q == SOURCE_END);
                let Some(at) = marker.filter(|&k| queue[k].starts_with(LOOP_CONT)) else {
                    return Err(SdcError(format!("invoked \"{word}\" outside of a loop (Tcl's error; the read stops)")));
                };
                let tag = queue[at][LOOP_CONT.len()..].to_string();
                queue.drain(..=at);
                if word == "break" {
                    let stop = loop_stop(&tag);
                    if let Some(k) = queue.iter().position(|q| *q == stop) {
                        queue.drain(..=k);
                    }
                }
                continue;
            }
            // A loop's test, queued after each pass of its body: run the body again while it holds.
            if let Some(id) = raw.strip_prefix(LOOP_TEST) {
                let Some(l) = id.parse::<usize>().ok().and_then(|i| loops.get_mut(i)) else { continue };
                if let Some(v) = undefined_in_condition(&l.cond, &vars) {
                    return Err(SdcError(format!(
                        "{} in the condition `{}` of `{}`: Tcl stops at a variable that is not defined, as the reference does, so this file is not read",
                        no_such_variable(&v),
                        l.cond,
                        l.cmd
                    )));
                }
                let cond = subst_vars(&subst_info_exists(&l.cond, &vars), &as_operands(&vars));
                match eval_expr(&cond, &vars).and_then(|v| v.truthy()) {
                    Some(true) => {
                        l.passes += 1;
                        if l.passes > MAX_LOOP_PASSES {
                            return Err(SdcError(format!("`{} {}`: more than {MAX_LOOP_PASSES} passes; not read as a loop that ends", l.cmd, l.cond)));
                        }
                        queue.push_front(raw.clone());
                        for c in l.next.iter().rev() {
                            queue.push_front(c.clone());
                        }
                        queue.push_front(format!("{LOOP_CONT}t{id}"));
                        for c in l.body.iter().rev() {
                            queue.push_front(c.clone());
                        }
                    }
                    Some(false) => {}
                    None => sdc.rejected.push(format!("{}: the condition `{}` is not evaluated; the loop is not run (on)", l.cmd, l.cond)),
                }
                continue;
            }
            // ⛔ AN UNDEFINED VARIABLE STOPS THE READ, as Tcl stops `source` there and so the
            // reference's `read_sdc` (user 2026-11-04): nothing after it is applied. Read on, `set
            // MAIN_CLK_PIN $MAIN_CLK_PIN` stored the text `$MAIN_CLK_PIN`, and a clock was made on
            // a pin of that name.
            if let Some(v) = undefined_read(&raw, &vars) {
                return Err(SdcError(format!(
                    "{} in `{}`: Tcl stops at a variable that is not defined, as the reference does, so this file is not read",
                    no_such_variable(&v),
                    raw.trim()
                )));
            }
            // `[dict …]`, `[a_proc …]` and the tool-name query answered in place, outside braces
            // (`sdc/tcl.rs`); a substitution that cannot be answered refuses its command.
            let raw = match tcl::subst_values(&raw, &vars, &procs, opts, frames.len()) {
                Ok(r) => r,
                // a proc called for its value read an undefined variable: Tcl's error, so the stop
                Err(why) if why.contains(NO_SUCH_VARIABLE) => {
                    return Err(SdcError(format!("{why}: Tcl stops at a variable that is not defined, as the reference does, so this file is not read")));
                }
                Err(why) => {
                    let cmd = raw.split_whitespace().next().unwrap_or("").to_string();
                    sdc.rejected.push(format!("{cmd}: {why}; not applied"));
                    if !BENIGN_FOR_TIMING.contains(&cmd.as_str()) && !matches!(cmd.as_str(), "set" | "puts" | "echo") {
                        sdc.unevaluated_queries.push(format!("{cmd}: {why}"));
                    }
                    continue;
                }
            };
            match dialect(&raw, &vars, &mut procs, &mut frames, &mut loops, opts)? {
                Dialect::Sourced(cmds) => {
                    sourced += 1;
                    if sourced > MAX_SOURCED {
                        return Err(SdcError(format!("more than {MAX_SOURCED} files sourced: a `source` loop")));
                    }
                    queue.push_front(SOURCE_END.into());
                    for c in cmds.into_iter().rev() {
                        queue.push_front(c);
                    }
                    continue;
                }
                // Tcl `return`: the rest of the running proc's body is skipped (up to its
                // PROC_RETURN, which then ends the call), or the rest of the sourced file, or —
                // at the top of the file read — the rest of the file, as `source` stops there.
                Dialect::Return => {
                    while let Some(next) = queue.front() {
                        if next == PROC_RETURN {
                            break;
                        }
                        let end = next == SOURCE_END;
                        queue.pop_front();
                        if end {
                            break;
                        }
                    }
                    continue;
                }
                Dialect::Run(cmds) => {
                    for c in cmds.into_iter().rev() {
                        queue.push_front(c);
                    }
                    if frames.len() > MAX_PROC_DEPTH {
                        return Err(SdcError(format!("proc calls nested deeper than {MAX_PROC_DEPTH}: a recursive proc")));
                    }
                    continue;
                }
                Dialect::Refuse(why) => {
                    sdc.rejected.push(why);
                    continue;
                }
                Dialect::Done => continue,
                Dialect::None => {}
            }
            match control(&raw, &vars, &procs) {
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
                Control::Error(why) => {
                    return Err(SdcError(format!("{why}: Tcl stops at a variable that is not defined, as the reference does, so this file is not read")));
                }
                Control::None => {}
            }
            let line = dialect_queries(&canonical_queries(&subst_exprs(&subst_command_vars(&raw, &vars), &vars)));
            let line = subst_load_of(&line);
            let line = match &instance_scope {
                None => line,
                Some(scope) => {
                    let clocks: Vec<String> = sdc.clocks.iter().map(|c| c.name.clone()).collect();
                    match scope::scoped_line(&line, scope, &clocks) {
                        Ok(l) => l,
                        Err(why) => {
                            let cmd = tokenize(&line).into_iter().next().unwrap_or_default();
                            sdc.rejected.push(format!("{cmd}: {why}; not applied"));
                            if !BENIGN_FOR_TIMING.contains(&cmd.as_str()) {
                                sdc.unevaluated_queries.push(format!("{cmd}: {why}"));
                            }
                            continue;
                        }
                    }
                }
            };
            let toks = expand_abbreviations(tokenize(&line));
            if toks.is_empty() {
                continue;
            }
            sdc.extensions.extend(extensions_in(&toks).into_iter().filter(|e| !sdc_spelling || *e != toks[0]));
            // A clock made INSIDE another command (`set_input_delay -clock [create_clock …] …`):
            // Tcl runs the substitution first, and the reference's `create_clock` returns nothing,
            // so the clock is made and the outer command sees an empty word.
            if let Some((inner, outer)) = nested_clock_definition(&line) {
                queue.push_front(outer);
                queue.push_front(inner);
                continue;
            }
            // A SYNTHESIS-ONLY command the reference does not know (`set_size_only`,
            // `set_critical_range`: [`BENIGN_FOR_TIMING`]) is READ PAST, whole, arguments not
            // evaluated — a deliberate difference from the reference, which stops at it (user
            // 2026-10-31): OpenTitan's constraint files use `set_size_only` 25 times and
            // `set_critical_range` 24, and nothing a timer consults depends on either. Recorded in
            // `ignored`, like any command not modelled.
            if BENIGN_FOR_TIMING.contains(&toks[0].as_str()) && resolve_command(&toks[0]).is_err() {
                sdc.ignored.push(toks[0].clone());
                continue;
            }
            // ⛔ A COMMAND THE REFERENCE DOES NOT KNOW STOPS THE READ, as it stops the reference's
            // `read_sdc` (user 2026-10-31): nothing after it is applied there, so nothing is
            // applied here either. Read on, this reader built constraints the reference never had
            // (`derive_pll_clocks`, then a `set_input_delay` the reference never saw).
            if let Some(why) = unknown_command(&line) {
                return Err(SdcError(format!(
                    "{why} in `{}`: the reference stops reading the file at a command it does not know, so this file is not read",
                    line.trim()
                )));
            }
            // A word that cannot name what it appears to (`[get_ports a]0`, `{[get_ports a]}`): the
            // reference builds no constraint from it, so neither does this reader — said, not
            // approximated. A `set` stores its text; it is checked where the variable is used.
            if toks[0] != "set" {
                if let Some(why) = word_fault(&line) {
                    sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                    continue;
                }
                // An option the command does not have fails it in the reference; so here.
                if let Some(why) = unknown_option(&toks).or_else(|| argument_fault(&toks)) {
                    sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                    continue;
                }
            }
            // An object query this reader cannot evaluate: the command is NOT applied, and says
            // so (a `set` only stores the text; it is checked where the variable is used).
            if toks[0] != "set" {
                if let Some(q) = unevaluated_query(&toks).or_else(|| misplaced_except(&toks)) {
                    sdc.rejected.push(format!("{}: {q} not evaluated; not applied", toks[0]));
                    // a command no timer consults (`set_dont_touch [get_designs …]`) is listed as
                    // not applied, not with the queries every consumer refuses the file on
                    if !BENIGN_FOR_TIMING.contains(&toks[0].as_str()) {
                        sdc.unevaluated_queries.push(format!("{}: {q}", toks[0]));
                    }
                    continue;
                }
            }
            // ⛔ An SDC option no consumer here applies (`set_driving_cell -clock`, `[all_inputs
            // -clock c]`: `sdc_flags::KEPT_OPTIONS`) is valid SDC: neither refused nor dropped. Its
            // command is kept and written back — in its typed record where that carries the
            // option (an I/O delay's flags; a derate's flags, which then sets no design-wide
            // derate), else as written — and reported as kept-not-applied.
            let options = kept_options_in(&toks);
            if !options.is_empty() {
                // a design-rule limit kept with its option is still checked as one
                if matches!(toks[0].as_str(), "set_max_capacitance" | "set_max_transition") {
                    if let Some(why) = design_rule_fault(&toks) {
                        sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                        continue;
                    }
                }
                sdc.ignored.extend(options.iter().cloned());
                sdc.kept_options.extend(options.iter().cloned());
                let typed = (matches!(toks[0].as_str(), "set_input_delay" | "set_output_delay" | "set_timing_derate") || kept::KEPT_COMMANDS.contains(&toks[0].as_str()))
                    && options.iter().all(|o| o.split(' ').next() == Some(toks[0].as_str()));
                if !typed {
                    sdc.kept.push(kept::verbatim(&toks));
                    continue;
                }
            }
            match toks[0].as_str() {
                "set" => {
                    if toks.len() >= 3 {
                        // At the top level `::x` IS `x` (the global namespace), and a read looks
                        // the name up with its `::` taken off: store it the same way. Inside a
                        // proc the qualified name stays as written (a global from a proc body is
                        // not modelled here).
                        let name = if frames.is_empty() { toks[1].trim_start_matches("::") } else { toks[1].as_str() };
                        vars.insert(name.to_string(), toks[2].clone());
                    }
                }
                // SDC accepts the singular and plural spellings interchangeably, and tools
                // emit both. Matching one silently ignores the other — and for
                // `set_clock_group` that means losing an asynchronous-clock declaration, so
                // paths between clocks that never relate get checked and report false
                // violations. Found by diffing our command inventory against SDC 2.1.
                // A CHANGED SCOPE: every later pin, cell and net name is read under the instance
                // (see `sdc/scope.rs`). Its argument is an absolute instance path (the reference's
                // `get_instance_error`); none, or the top, goes back to the top. More than one
                // instance is the reference's Error 123, and the scope is left as it was.
                // SDC declares the instance as `-instance`; the reference as the positional
                "current_instance" | "set_current_instance" => match toks.iter().skip(1).find(|t| *t != "-instance").filter(|t| !matches!(t.as_str(), "/" | "." | "" | "{}")) {
                    None => instance_scope = None,
                    Some(i) => match resolve_objs(i).as_slice() {
                        [one] => instance_scope = Some(one.trim_end_matches('/').to_string()),
                        _ => sdc.rejected.push(format!("{}: instance must be a single instance (Error 123); not applied", toks[0])),
                    },
                },
                "set_hierarchy_separator" => {
                    if let Some(sep) = toks.get(1).filter(|t| t.as_str() != "/") {
                        return Err(SdcError(format!(
                            "`set_hierarchy_separator {sep}`: only `/` is supported, and names using another separator would not resolve; the file is not read"
                        )));
                    }
                }
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
                    if let Err(why) = check_units(&toks) {
                        sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                        continue;
                    }
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
                                Some(v) => format!(
                                    "create_clock -period `{v}` is not a positive float — SDC takes the period as a \
                                     number in the file's time unit (a frequency such as `100 MHz` or a unit suffix \
                                     such as `20ns` is a Quartus extension; the reference refuses it too, Error 572)"
                                ),
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
                    // each edge a plain number (`check_float "-waveform edge"`), no unit suffix
                    let waveform = flag_val(&toks, "-waveform").map(|w| {
                        list_words(w).split_whitespace().map(|e| e.parse::<f64>().ok()).collect::<Option<Vec<f64>>>().unwrap_or_default()
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
                    let clock = SdcClock { name, source: src, sources, period, waveform, add, generated: None, comment: comment_of(&toks) };
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
                    if let Some(why) = generated_clock_fault(&|f| val(f).map(str::to_string), &has, pos.is_empty()) {
                        sdc.rejected.push(format!("create_generated_clock: {why}; not applied"));
                        continue;
                    }
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
                    sdc.clocks.push(SdcClock { name, source, sources, period, waveform, add: has("-add"), generated: Some(spec), comment: comment_of(&toks) });
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
                    // `get_port_pin_error`: against a linked design the reference pin is a port or
                    // an instance pin of it, else the command errors
                    if let Some(r) = valued("-reference_pin") {
                        if with_design(|d| d.has_port_or_pin(&r)) == Some(false) {
                            sdc.rejected.push(format!("{}: -reference_pin: pin {r} not found (Error 130); not applied", toks[0]));
                            continue;
                        }
                    }
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
                    if let Some(why) = clock_uncertainty_fault(&with, &flags, &pos) {
                        sdc.rejected.push(format!("set_clock_uncertainty: {why}; not applied"));
                        continue;
                    }
                    let (Some(value), objects) = (pos.first().and_then(|v| v.parse::<f64>().ok()), pos.iter().skip(1).flat_map(|o| resolve_objs(o)).collect::<Vec<_>>()) else {
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
                    sdc.refold_uncertainty();
                }
                "set_clock_transition" => {
                    sdc.clock_transitions.extend(env_cmd(&toks).map(|e| sdc.clocks_now(e)));
                    sdc.ignored.push("set_clock_transition".to_string());
                }
                "set_clock_latency" => {
                    // `set_clock_latency`: `-early`/`-late` qualify a SOURCE latency only
                    if toks.iter().any(|t| t == "-early" || t == "-late") && !toks.iter().any(|t| t == "-source") {
                        sdc.rejected.push("set_clock_latency: -early/-late is only allowed with -source (Error 410); not applied".into());
                        continue;
                    }
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
                    if let Some(why) = design_rule_fault(&toks) {
                        sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                        continue;
                    }
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_fanout = Some(v),
                        _ => {
                            sdc.max_fanout_on_objects = true;
                            match design_rule_cmd(&toks) {
                                Some(d) => sdc.max_fanout_objects.push(d),
                                None => sdc.rejected.push(format!("{} without objects", toks[0])),
                            }
                        }
                    }
                    sdc.ignored.push("set_max_fanout".to_string());
                }
                "set_max_capacitance" => {
                    if let Some(why) = design_rule_fault(&toks) {
                        sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                        continue;
                    }
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
                    let objects: Vec<(String, Objects)> = with.iter().filter(|(k, _)| *k != "-comment").map(|(k, v)| (k.to_string(), Objects::of(v))).collect();
                    sdc.reset_path_first(&toks, &objects, &flags, true);
                    sdc.path_delays.push(PathDelayCmd {
                        max: toks[0] == "set_max_delay",
                        value,
                        objects,
                        flags,
                        comment: comment_of(&toks),
                    });
                    sdc.ignored.push(toks[0].clone());
                }
                "set_max_transition" => {
                    if let Some(why) = design_rule_fault(&toks) {
                        sdc.rejected.push(format!("{}: {why}; not applied", toks[0]));
                        continue;
                    }
                    match (toks.len() == 3 && toks[2] == "[current_design]", toks.get(1).and_then(|v| v.parse::<f64>().ok())) {
                        (true, Some(v)) => sdc.max_transition = Some(v),
                        _ => {
                            sdc.max_transition_on_objects = true;
                            match design_rule_cmd(&toks) {
                                Some(d) => sdc.max_transition_objects.push(d),
                                None => sdc.rejected.push(format!("{} without objects", toks[0])),
                            }
                        }
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
                "set_disable_timing" | "unset_disable_timing" => {
                    match arc_cmd(&toks, &["-from", "-to"], &[]) {
                        // `parse_disable_inst_ports`: on instances or library cells `-from`/`-to` is
                        // a PORT NAME of the cell; an object query there evaluates to an object, whose
                        // name is never a port name, so the reference errors (Error 431)
                        Ok(c) if matches!(c.written.accessor.as_str(), "get_cells" | "get_lib_cells") && c.keys.iter().any(|(_, v)| v.starts_with('[')) => {
                            sdc.rejected.push(format!("{}: -from/-to on cells is a port name, not an object (the reference's Error 431); not applied", toks[0]));
                        }
                        Ok(c) => sdc.disabled_timing.push(c),
                        Err(why) => sdc.rejected.push(why),
                    }
                    sdc.ignored.push(toks[0].clone());
                }
                "set_sense" | "set_clock_sense" => {
                    // listed in `ignored` (so kept-not-applied) only when read: a refused one
                    // is counted once, as not applied
                    match sense_cmd(&toks) {
                        Ok((c, defaulted)) => {
                            if defaulted {
                                sdc.senses_type_defaulted.push(sdc.senses.len());
                            }
                            sdc.senses.push(c);
                            sdc.ignored.push(toks[0].clone());
                        }
                        Err(why) => sdc.rejected.push(why),
                    }
                }
                "set_disable_clock_gating_check" | "unset_disable_clock_gating_check" => {
                    match arc_cmd(&toks, &[], &[]) {
                        Ok(c) => sdc.disabled_gating_checks.push(c),
                        Err(why) => sdc.rejected.push(why),
                    }
                    sdc.ignored.push(toks[0].clone());
                }
                // Deprecated spellings the reference runs as `set_disable_clock_gating_check` /
                // `unset_…` (its Search.tcl, Warnings 528/529): one parameter, the objects.
                "set_disable_inferred_clock_gating" | "unset_disable_inferred_clock_gating" => {
                    let cmd = toks[0].replace("_inferred_clock_gating", "_clock_gating_check");
                    match toks.as_slice() {
                        [_, o] => sdc.disabled_gating_checks.push(ArcCmd { cmd: cmd.clone(), keys: Vec::new(), flags: Vec::new(), objects: resolve_objs(o), written: Objects::of(o) }),
                        _ => sdc.rejected.push(format!("{}: {} words where the reference takes one (wrong # args); not applied", toks[0], toks.len() - 1)),
                    }
                    sdc.ignored.push(cmd);
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
                    // `parse_early_late_flags`: the two are exclusive (both = an error, not both)
                    if has("-early") && has("-late") {
                        sdc.rejected.push("set_timing_derate: only one of -early and -late can be specified (Error 110); not applied".into());
                        continue;
                    }
                    let Some(value) = pos.first().and_then(|v| v.parse::<f64>().ok()) else {
                        sdc.rejected.push("set_timing_derate without a value".into());
                        continue;
                    };
                    let objects: Vec<String> = pos.iter().skip(1).flat_map(|o| resolve_objs(o)).collect();
                    let written = pos.get(1).map(|o| Objects::of(o));
                    // SDC's `-min` / `-max` (a min or max operating condition) scope it too
                    let scoped = !objects.is_empty()
                        || ["-cell_delay", "-net_delay", "-cell_check", "-data", "-clock", "-rise", "-fall", "-min", "-max"].iter().any(|f| has(f));
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
                            for w in clock_groups_ignored_words(&toks) {
                                sdc.ignored_words.push(format!("set_clock_groups {w}"));
                            }
                            let name = flag_val(&toks, "-name").and_then(|n| resolve_objs(n).into_iter().next());
                            // `Sdc::makeClockGroups`: a name already defined is REPLACED
                            if name.is_some() {
                                sdc.clock_groups.retain(|g| g.name != name);
                            }
                            sdc.clock_groups.push(ClockGroups { name, kind, allow_paths: has_flag(&toks, "-allow_paths"), groups: groups.clone(), comment: comment_of(&toks) })
                        }
                        // a type is required (the reference's STA-0400)
                        None => sdc.rejected.push("set_clock_groups without -asynchronous / -logically_exclusive / -physically_exclusive".into()),
                    }
                    sdc.async_groups.extend(groups);
                }
                "set_false_path" => {
                    let (from, to, refused) = from_to(&toks);
                    let (args, flags) = exception_args(&toks);
                    sdc.reset_path_first(&toks, &args, &flags, false);
                    sdc.exception_objects.push(exception_sides(&toks));
                    sdc.exceptions.push(Exception { kind: ExcKind::FalsePath, from, to, refused, args, flags, comment: comment_of(&toks), ..Default::default() });
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
                    sdc.reset_path_first(&toks, &args, &written_flags, false);
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
                        comment: comment_of(&toks),
                    });
                }
                cmd if kept::KEPT_COMMANDS.contains(&cmd) => {
                    match kept::kept_cmd(&toks) {
                        Ok(c) => sdc.kept.push(c),
                        Err(why) => sdc.rejected.push(format!("{cmd}: {why}; not applied")),
                    }
                    sdc.ignored.push(cmd.to_string());
                }
                cmd if kept::EXCEPTION_COMMANDS.contains(&cmd) => {
                    match kept::exception_cmd(&toks) {
                        Ok(x) => sdc.other_exceptions.push(x),
                        Err(why) => sdc.rejected.push(format!("{cmd}: {why}; not applied")),
                    }
                    sdc.ignored.push(cmd.to_string());
                }
                cmd if unset::UNSET_COMMANDS.contains(&cmd) => {
                    if let Err(why) = sdc.apply_unset(&toks) {
                        sdc.rejected.push(format!("{cmd}: {why}; not applied"));
                    }
                }
                other => sdc.ignored.push(other.to_string()),
            }
        }
        record_unmodelled(&mut sdc, &current_text);

        Ok(sdc)
    }

    pub fn load(path: &str) -> Result<Sdc, SdcError> {
        let base_dir = std::path::Path::new(path).parent().map(|p| p.to_path_buf());
        Sdc::parse_opts(&read_lossy(path)?, &ReadOpts { base_dir, ..Default::default() })
    }

    /// [`Sdc::parse`] against a linked design, as the reference reads constraints after linking:
    /// object queries (`-filter`, `-of_objects`) are evaluated on `design`, by the reference's
    /// rules, instead of refusing their commands. What the design view cannot
    /// answer still refuses, in [`Sdc::unevaluated_queries`].
    pub fn parse_with(text: &str, design: &SdcDesign) -> Result<Sdc, SdcError> {
        Sdc::parse_with_opts(text, design, &ReadOpts::default())
    }

    /// [`Sdc::parse_with`] with the flow's context ([`ReadOpts`]).
    pub fn parse_with_opts(text: &str, design: &SdcDesign, opts: &ReadOpts) -> Result<Sdc, SdcError> {
        let _scope = DesignScope::enter(design);
        Sdc::parse_opts(text, opts)
    }

    /// [`Sdc::load`] against a linked design — see [`Sdc::parse_with`].
    pub fn load_with(path: &str, design: &SdcDesign) -> Result<Sdc, SdcError> {
        let base_dir = std::path::Path::new(path).parent().map(|p| p.to_path_buf());
        Sdc::parse_with_opts(&read_lossy(path)?, design, &ReadOpts { base_dir, ..Default::default() })
    }
}

/// What a constraint file is read with besides its text: what the flow around it supplies.
#[derive(Debug, Clone, Default)]
pub struct ReadOpts {
    /// Variables set before the file is read, in order (`NAME`, value) — what a synthesis flow's
    /// setup script defines and its constraint files only use (`${DRIVING_CELL}`, a period factor).
    pub defines: Vec<(String, String)>,
    /// The directory of the file being read: a relative `source` not found from the working
    /// directory (where Tcl looks) is looked for here.
    pub base_dir: Option<std::path::PathBuf>,
}

// ---- the synthesis-tool dialect: `proc`, `source`, collections ------------------------------

/// A procedure the file defines: its parameters (with defaults) and its body's commands.
#[derive(Debug, Clone)]
struct TclProc {
    params: Vec<(String, Option<String>)>,
    body: Vec<String>,
}

/// A `for` / `while` loop being run: its test, the `for` step, its body, and the passes so far.
#[derive(Debug, Clone)]
struct TclLoop {
    cmd: String,
    cond: String,
    next: Vec<String>,
    body: Vec<String>,
    passes: usize,
}

/// The line the reader queues before each pass of a loop: `LOOP_TEST` and the loop's index.
const LOOP_TEST: &str = "\u{2}loop-test ";
/// The line queued at the end of each pass of a loop's body, with the loop's tag: `continue` drops
/// the queue through the first one ahead (the innermost running loop's), `break` on to its stop
/// line ([`loop_stop`]).
const LOOP_CONT: &str = "\u{2}loop-cont ";
/// The line queued after a `foreach`'s last pass, with its tag: where its `break` stops.
const LOOP_END: &str = "\u{2}loop-end ";

/// The line a `break` in the loop tagged `tag` drops the queue through: a `for`/`while` (`t<id>`)
/// its test, a `foreach` (`f<n>`) its end.
fn loop_stop(tag: &str) -> String {
    match tag.strip_prefix('t') {
        Some(id) => format!("{LOOP_TEST}{id}"),
        None => format!("{LOOP_END}{tag}"),
    }
}

/// Tags for `foreach` loops, unique within the process.
static FOREACH_TAGS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
const MAX_LOOP_PASSES: usize = 100_000;

/// The line the reader queues after a proc's body: its call has returned.
const PROC_RETURN: &str = "\u{1}proc-return";
/// The line the reader queues after a sourced file's commands: where a `return` in it stops.
const SOURCE_END: &str = "\u{3}source-end";
const MAX_PROC_DEPTH: usize = 64;
const MAX_SOURCED: usize = 256;

/// What a dialect command asks of the read.
enum Dialect {
    /// these commands run next, in order
    Run(Vec<String>),
    /// a sourced file's commands run next
    Sourced(Vec<String>),
    /// not applied, and why
    Refuse(String),
    /// handled; nothing further
    Done,
    /// Tcl `return`: the running proc's body (or sourced file) ends here
    Return,
    /// not a dialect command
    None,
}

/// The commands a synthesis tool's Tcl adds to SDC, read as that Tcl would run them (constraint
/// files written for a synthesis tool use them, and otherwise stopped the read at the first one). Each is a LOOM EXTENSION where the reference does not have it; the
/// reference's own Tcl commands (`proc`, `source`, `puts`, `global`) follow Tcl.
///
/// - `proc name {params} {body}` is remembered; a call runs its body with the parameters set
///   (defaults and `args` as Tcl binds them), its variables local to the call except those it
///   declares `global`. A body that would `return` early, or uses `upvar`/`uplevel`, is refused.
/// - `source ?-echo? ?-verbose? file` reads the file in place (from the working directory, as Tcl
///   does, then from the directory of the file being read). A file that is not there is an error,
///   as it is to Tcl: the read stops.
/// - `puts` / `echo` print and change nothing: read past.
/// - `append_to_collection v C` is `set v [list $v C]` (just `C` when `v` is not yet set).
fn dialect(
    raw: &str,
    vars: &HashMap<String, String>,
    procs: &mut HashMap<String, TclProc>,
    frames: &mut Vec<ProcFrame>,
    loops: &mut Vec<TclLoop>,
    opts: &ReadOpts,
) -> Result<Dialect, SdcError> {
    let Some(head) = raw.split_whitespace().next() else { return Ok(Dialect::None) };
    match head {
        "proc" => {
            let toks = tokenize(raw);
            let [_, name, params, body] = toks.as_slice() else {
                return Ok(Dialect::Refuse(format!("proc: `{}` is not `proc name params body`; not defined", raw.trim())));
            };
            let mut ps = Vec::new();
            for p in tokenize(params) {
                let pt = tokenize(&p);
                match pt.as_slice() {
                    [n] => ps.push((n.clone(), None)),
                    [n, d] => ps.push((n.clone(), Some(d.clone()))),
                    _ => return Ok(Dialect::Refuse(format!("proc {name}: parameter `{p}` not read; not defined"))),
                }
            }
            procs.insert(name.clone(), TclProc { params: ps, body: logical_lines(body) });
            Ok(Dialect::Done)
        }
        "puts" | "echo" => Ok(Dialect::Done),
        // Tcl `for {init} {test} {next} {body}` and `while {test} {body}`: the test is evaluated
        // before each pass, with the variables as they are then (a queued test line, see the
        // read loop). `continue` ends the pass (the `next` and the test still run), `break` the loop.
        "for" | "while" => {
            let toks = tokenize(raw);
            let (init, cond, next, body) = match (head, toks.as_slice()) {
                ("for", [_, i, c, n, b]) => (logical_lines(i), c.clone(), logical_lines(n), b),
                ("while", [_, c, b]) => (Vec::new(), c.clone(), Vec::new(), b),
                _ => return Ok(Dialect::Refuse(format!("{head}: `{}` is not the {head} form; not run", raw.trim()))),
            };
            loops.push(TclLoop { cmd: head.to_string(), cond, next, body: logical_lines(body), passes: 0 });
            let mut cmds = init;
            cmds.push(format!("{LOOP_TEST}{}", loops.len() - 1));
            Ok(Dialect::Run(cmds))
        }
        // Tcl `switch ?-exact|-glob? ?--? value {pattern body …}` (or the pairs inline): the first
        // matching pattern's body runs, `default` matching anything; a body `-` falls through to
        // the next one. `-regexp` matches as Tcl's `regexp` does; `-nocase` folds case.
        "switch" => {
            let line = subst_command_vars(raw, vars);
            let toks = tokenize(&line);
            let mut k = 1;
            // ONE mode option: a second is Tcl's error (`bad option "-regexp": -glob option
            // already found`, Tcl 8.6.14)
            let (mut mode, mut nocase) = (None, false);
            while let Some(t) = toks.get(k).filter(|t| t.starts_with('-')) {
                match t.as_str() {
                    m @ ("-exact" | "-glob" | "-regexp") => {
                        if let Some(first) = mode {
                            return Ok(Dialect::Refuse(format!("switch: bad option \"{m}\": {first} option already found (Tcl's error); not run")));
                        }
                        mode = Some(m);
                    }
                    "-nocase" => nocase = true,
                    "--" => {
                        k += 1;
                        break;
                    }
                    other => return Ok(Dialect::Refuse(format!("switch {other}: not evaluated; not run"))),
                }
                k += 1;
            }
            let Some(value) = toks.get(k) else { return Ok(Dialect::Refuse("switch: no value; not run".into())) };
            // the tool-name test of a vendor file, the variable not defined by the flow: read as
            // a signoff timer's name (`tcl::TOOL_NAME_DEFAULT`; `--define` picks another)
            let tool = tcl::TOOL_NAME_DEFAULT.to_string();
            let value = if [format!("${}", tcl::TOOL_NAME_VAR), format!("$::{}", tcl::TOOL_NAME_VAR)].contains(value) { &tool } else { value };
            let pairs: Vec<String> = match &toks[k + 1..] {
                [one] => tokenize(one),
                many => many.to_vec(),
            };
            if !pairs.len().is_multiple_of(2) {
                return Ok(Dialect::Refuse("switch: patterns and bodies do not pair; not run".into()));
            }
            let fold = |s: &str| if nocase { s.to_lowercase() } else { s.to_string() };
            let mut hit = false;
            for pb in pairs.chunks(2) {
                let (pat, body) = (&pb[0], &pb[1]);
                // `-regexp` is Tcl's `regexp`: unanchored, the first pattern found anywhere in it
                let matched = match mode.unwrap_or("-exact") {
                    _ if pat == "default" => true,
                    "-regexp" => match crate::sdc_query::tcl_regex(pat, false, nocase) {
                        Ok(r) => r.is_match(value),
                        Err(why) => return Ok(Dialect::Refuse(format!("switch: {why}; not run"))),
                    },
                    "-glob" => glob(&fold(pat), &fold(value)),
                    _ => fold(pat) == fold(value),
                };
                hit = hit || matched;
                if hit && body != "-" {
                    return Ok(Dialect::Run(logical_lines(body)));
                }
            }
            Ok(Dialect::Done)
        }
        // Tcl `lappend v ?elem …?`: the variable's list with each element appended, as one word.
        "lappend" => {
            let line = subst_command_vars(raw, vars);
            let toks = tokenize(&line);
            let Some(var) = toks.get(1) else { return Ok(Dialect::Refuse("lappend: no variable; not applied".into())) };
            let mut list: Vec<String> = vars.get(var.as_str()).filter(|v| !v.trim().is_empty()).map(|v| vec![v.trim().to_string()]).unwrap_or_default();
            list.extend(toks[2..].iter().map(|e| brace_word(e)));
            Ok(Dialect::Run(vec![format!("set {var} {{{}}}", list.join(" "))]))
        }
        // Tcl `incr v ?n?`: an unset variable counts from 0, as in Tcl 8.5+.
        "incr" => {
            let toks = tokenize(&subst_command_vars(raw, vars));
            let Some(var) = toks.get(1) else { return Ok(Dialect::Refuse("incr: no variable; not applied".into())) };
            let by = toks.get(2).map_or(Some(1), |n| n.parse::<i64>().ok());
            let now = vars.get(var.as_str()).map_or(Some(0), |v| v.trim().parse::<i64>().ok());
            match (now, by) {
                (Some(a), Some(b)) => Ok(Dialect::Run(vec![format!("set {var} {}", a + b)])),
                _ => Ok(Dialect::Refuse(format!("incr {var}: not an integer; not applied"))),
            }
        }
        // `foreach_in_collection v C {body}`: a body that only hands `$v` to constraint commands
        // as an object argument runs ONCE on the whole collection, which applies each command to
        // each object alike ([`tcl::collection_body_is_setwise`]); a body that names, counts or
        // tests the object needs the objects one by one, which only the design has: refused.
        "foreach_in_collection" => {
            let toks = tokenize(raw);
            if let [_, var, coll, body] = toks.as_slice() {
                let lines = logical_lines(body);
                if tcl::collection_body_is_setwise(var, &lines, procs) {
                    let mut cmds = vec![format!("set {var} {coll}")];
                    cmds.extend(lines);
                    return Ok(Dialect::Run(cmds));
                }
            }
            Ok(Dialect::Refuse(format!(
                "foreach_in_collection: `{}` uses each object on its own (its name, a test, a derived name), which needs the design's objects one by one; not applied",
                raw.split_whitespace().take(3).collect::<Vec<_>>().join(" ")
            )))
        }
        "global" => {
            // inside a call, these names outlive it; at the top every name is global already
            if let Some((_, links)) = frames.last_mut() {
                links.extend(tokenize(raw).into_iter().skip(1).map(|g| {
                    let g = g.trim_start_matches("::").to_string();
                    (g.clone(), g)
                }));
            }
            Ok(Dialect::Done)
        }
        "return" => Ok(Dialect::Return),
        // Tcl `upvar ?level? other local …`: `local` is the caller's `other` for the rest of the
        // call (read now, written back when the call returns). At the top it is the same name.
        "upvar" => {
            let links = match tcl::upvar_links(raw) {
                Ok(l) => l,
                Err(why) => return Ok(Dialect::Refuse(format!("{why}; not applied"))),
            };
            let sets: Vec<String> = links.iter().filter_map(|(local, other)| vars.get(other).map(|v| format!("set {local} {}", brace_word(v)))).collect();
            if let Some((_, l)) = frames.last_mut() {
                l.extend(links);
            }
            Ok(Dialect::Run(sets))
        }
        "dict" => Ok(tcl::dict_statement(raw, vars)),
        // a signoff timer's `remove_*` / `reset_*`: the reference's `unset_*` / `delete_*` command
        w if tcl::take_back_alias(w).is_some() => Ok(tcl::rewrite_take_back(raw).map_or(Dialect::None, |l| Dialect::Run(vec![l]))),
        // `set_current_design`, which the vendor releases of the SDC parser declare in SDC 1.1, is
        // read as `current_design` (the reference timer does not know it, and stops there)
        "set_current_design" => Ok(Dialect::Run(vec![format!("{}current_design{}", tcl::SDC_SPELLING, &raw.trim_start()["set_current_design".len()..])])),
        "source" => {
            let line = subst_command_vars(raw, vars);
            let toks = tokenize(&line);
            let files: Vec<&String> = toks[1..].iter().filter(|t| !matches!(t.as_str(), "-echo" | "-verbose" | "-e" | "-v")).collect();
            let [file] = files.as_slice() else {
                return Ok(Dialect::Refuse(format!("source: `{}` names no one file; not read", raw.trim())));
            };
            if file.contains(['$', '[']) {
                return Ok(Dialect::Refuse(format!("source: the file `{file}` is not resolved; not read")));
            }
            let path = std::path::Path::new(file.as_str());
            let found = if path.is_absolute() || path.exists() {
                Some(path.to_path_buf())
            } else {
                opts.base_dir.as_ref().map(|b| b.join(path)).filter(|p| p.exists())
            };
            let Some(found) = found else {
                return Err(SdcError(format!("`source {file}`: no such file; the reference stops reading at an error, so this file is not read")));
            };
            let text = read_lossy(&found.to_string_lossy())?;
            Ok(Dialect::Sourced(logical_lines(&text)))
        }
        "append_to_collection" => {
            let line = subst_command_vars(raw, vars);
            let toks = tokenize(&line);
            let rest: Vec<&String> = toks[1..].iter().filter(|t| *t != "-unique").collect();
            let [var, coll] = rest.as_slice() else {
                return Ok(Dialect::Refuse(format!("append_to_collection: `{}` not read; not applied", raw.trim())));
            };
            let set = match vars.get(var.as_str()) {
                Some(old) => format!("set {var} {{[list {old} {coll}]}}"),
                None => format!("set {var} {{{coll}}}"),
            };
            Ok(Dialect::Run(vec![set]))
        }
        name if procs.contains_key(name) => {
            let line = subst_exprs(&subst_command_vars(raw, vars), vars);
            let args = tokenize(&line).split_off(1);
            let p = &procs[name];
            if p.body.iter().any(|c| c.split_whitespace().next() == Some("uplevel")) {
                return Ok(Dialect::Refuse(format!("{name}: a proc that runs code in its caller's frame (`uplevel`); not run")));
            }
            let mut sets = Vec::new();
            let mut a = args.iter();
            for (k, (pn, def)) in p.params.iter().enumerate() {
                let is_last = k + 1 == p.params.len();
                let v = if pn == "args" && is_last {
                    Some(a.by_ref().map(|s| brace_word(s)).collect::<Vec<_>>().join(" "))
                } else {
                    a.next().map(|s| s.to_string()).or_else(|| def.clone())
                };
                let Some(v) = v else {
                    return Ok(Dialect::Refuse(format!("{name}: no value for parameter `{pn}`; not run")));
                };
                sets.push(format!("set {pn} {}", brace_word(&v)));
            }
            if a.next().is_some() {
                return Ok(Dialect::Refuse(format!("{name}: more arguments than parameters; not run")));
            }
            frames.push((vars.clone(), Vec::new()));
            let mut cmds = sets;
            cmds.extend(p.body.iter().cloned());
            cmds.push(PROC_RETURN.into());
            Ok(Dialect::Run(cmds))
        }
        _ => Ok(Dialect::None),
    }
}

/// The first `[create_clock …]` / `[create_generated_clock …]` substituted in `line` (outside
/// braces, in a command that is not itself one), as `(that command, line with it replaced by
/// `{}`)` — what the reference's `create_clock` returns (its `make_clock` returns nothing).
fn nested_clock_definition(line: &str) -> Option<(String, String)> {
    let b = line.as_bytes();
    let first = tokenize(line).into_iter().next()?;
    if first == "create_clock" || first == "create_generated_clock" {
        return None;
    }
    let mut depth = 0usize;
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'\\' => i += 1,
            b'{' => depth += 1,
            b'}' => depth = depth.saturating_sub(1),
            b'[' if depth == 0 => {
                let word_end = line[i + 1..].find(|c: char| !(c.is_ascii_alphanumeric() || c == '_')).map_or(b.len(), |e| i + 1 + e);
                if matches!(&line[i + 1..word_end], "create_clock" | "create_generated_clock") {
                    let mut d = 0usize;
                    let mut j = i;
                    while j < b.len() {
                        match b[j] {
                            b'\\' => j += 1,
                            b'[' => d += 1,
                            b']' => {
                                d -= 1;
                                if d == 0 {
                                    break;
                                }
                            }
                            _ => {}
                        }
                        j += 1;
                    }
                    let inner = line.get(i + 1..j)?.to_string();
                    let outer = format!("{}{{}}{}", &line[..i], line.get(j + 1..).unwrap_or(""));
                    return Some((inner, outer));
                }
            }
            _ => {}
        }
        i += 1;
    }
    None
}

/// `w` as one Tcl word whose value is `w`: braced unless it is one plain word already.
pub(crate) fn brace_word(w: &str) -> String {
    if !w.is_empty() && !w.contains(|c: char| c.is_whitespace() || "{}[]$\"\\;".contains(c)) {
        w.to_string()
    } else {
        format!("{{{w}}}")
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

/// The values of the variables `raw` names (`$v`, `${v}`), as ` ;# v=…` in name order. A braced
/// body (an `if` the reader did not evaluate) is not substituted, as in Tcl, so the same text can
/// mean different constraints under different values: the values make the text say which.
fn referenced_values(raw: &str, vars: &HashMap<String, String>) -> String {
    let mut names: Vec<String> = Vec::new();
    let cs: Vec<char> = raw.chars().collect();
    let mut i = 0;
    while i < cs.len() {
        if cs[i] == '$' {
            let mut j = i + 1;
            let name: String = if cs.get(j) == Some(&'{') {
                j += 1;
                let n: String = cs[j..].iter().take_while(|c| **c != '}').collect();
                n
            } else {
                cs[j..].iter().take_while(|c| c.is_alphanumeric() || **c == '_' || **c == ':').collect()
            };
            if !name.is_empty() && vars.contains_key(&name) && !names.contains(&name) {
                names.push(name);
            }
            i = j;
        }
        i += 1;
    }
    names.sort();
    names.iter().map(|n| format!(" ;# {n}={}", vars[n])).collect()
}

/// Pairs each [`Sdc::ignored`] entry added since the last call with `text`, the command that
/// added it, keeping [`Sdc::unmodelled_text`] index for index with `ignored`.
fn record_unmodelled(sdc: &mut Sdc, text: &str) {
    while sdc.unmodelled_text.len() < sdc.ignored.len() {
        let entry = sdc.ignored[sdc.unmodelled_text.len()].clone();
        sdc.unmodelled_text.push((entry, text.to_string()));
    }
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
    // The reference keeps an edge per side (`-rise_from` / `-fall_from`, `-rise_to` / `-fall_to`)
    // and one at the endpoint (`-rise` / `-fall`): the exception holds on that edge only. Applied
    // to both edges it cuts (or relaxes) the paths of the edge it left alone — optimistic, silent.
    if refused.is_none() {
        if let Some(f) = ["-rise_from", "-fall_from", "-rise_to", "-fall_to", "-rise", "-fall"].iter().find(|f| has_flag(toks, f)) {
            refused = Some(format!("{f}: an exception on one edge only, which an instance-level match cannot express"));
        }
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
mod exception_check_tests {
    use super::*;
    /// Rule (set_false_path / set_multicycle_path, `-setup` / `-hold` read as MinMaxAll): a false
    /// path with neither flag, or both, cuts both checks; `-hold` cuts the hold check only and
    /// `-setup` the setup check only. A `-hold` multicycle leaves setup alone; any multicycle
    /// reaches the hold check.
    /// Rule (parse_from_arg / parse_to_arg: `-rise_from` / `-fall_from` set the from edge,
    /// `-rise_to` / `-fall_to` the to edge, `-rise` / `-fall` the endpoint edge): such an exception
    /// holds on one edge only, so it is refused — never applied to both edges.
    #[test]
    fn an_exception_on_one_edge_is_refused() {
        let s = Sdc::parse(
            "set_false_path -rise_from a -to b\nset_false_path -from a -fall_to b\nset_multicycle_path 2 -rise -to b\n\
             set_false_path -from a -to b\n",
        )
        .unwrap();
        let refused: Vec<bool> = s.exceptions.iter().map(|e| e.refused.is_some()).collect();
        assert_eq!(refused, [true, true, true, false]);
        assert!(!s.exceptions[0].covers("a", "b"));
    }

    /// Tcl's word rule: a word is the CONCATENATION of its parts and a braced word is literal. The
    /// reference builds nothing from these (measured 2026-10-31, spec-matrix `word:*` probes):
    /// `[get_ports din]0` and `[get_ports a][get_ports b]` are Error 100 "unsupported object type";
    /// `x[get_ports dout]` and `{[get_ports dout]}` name objects not found (Warning 101). Each was
    /// read here as a constraint on a real port. Now each is refused, and nothing else is.
    #[test]
    fn a_word_that_is_not_its_query_is_refused() {
        let bad = [
            "set_false_path -from [get_ports din]0 -to [get_ports dout]",
            "set_load 1 x[get_ports dout]",
            "set_load 1 [get_ports dout][get_ports din]",
            "set_load 1 {[get_ports dout]}",
            "set_load 1 \"x[get_ports dout]\"",
            "set_load 1 [get_ports [get_ports a]0]",
            "set_load 1 {dout}x",
        ];
        for line in bad {
            let s = Sdc::parse(line).unwrap();
            assert_eq!(s.rejected.len(), 1, "{line}: {:?}", s.rejected);
            assert!(s.exceptions.is_empty() && s.env.is_empty() && s.load.is_none(), "{line} was applied");
        }
        // what stays: one substitution as the whole word (quoted too), a bus index, a brace list
        // of names (bus bits included), `[list …]`, `{*}` expansion
        let good = [
            "set_load 1 [get_ports dout]",
            "set_load 1 \"[get_ports dout]\"",
            "set_load 1 [get_ports d[0]]",
            "set_load 1 {d[0] d[1]}",
            "set_load 1 [get_ports {d[7:0]}]",
            "set_false_path -from [list [get_ports a] [get_ports b]] -to [get_ports c]",
            "set_load 1 {*}[get_ports dout]",
        ];
        for line in good {
            let s = Sdc::parse(line).unwrap();
            assert!(s.rejected.is_empty(), "{line}: {:?}", s.rejected);
        }
    }

    /// The reference's argument parser: a word is an option when it is `-` and a letter; it must be
    /// one of the command's keys or flags exactly, or the beginning of one (first key, then first
    /// flag), or the command fails (Error 562, measured 2026-10-31 on the spec-matrix probes and on
    /// the parser itself). An SDC option it does not implement is such an option. A command whose
    /// parse passes unknown options on (the exception commands) is not judged here. An option
    /// the SDC standard declares and the reference lacks (`all_inputs -edge_triggered`,
    /// `set_driving_cell -clock`, `get_ports -hierarchical`) is valid SDC and is read: see
    /// `tests/sdc_standard_options.rs`.
    #[test]
    fn an_option_the_command_does_not_have_is_refused() {
        let bad = [
            "set_clock_latency -dynamic 1 [get_clocks clk]",
            "set_timing_derate -increment 1.05",
            "set_load -total 1 [get_ports dout]",
            "set_input_delay 1 -clock clk [all_inputs -bogus]",
            // a synthesis tool's query options that no SDC version declares (the public parser's
            // sdc1.1 … sdc2.0) and the reference does not read (its Error 562): refused, both
            "set_false_path -to [get_pins -leaf u1/A]",
            "set_false_path -to [get_nets -segments n1]",
        ];
        for line in bad {
            let s = Sdc::parse(line).unwrap();
            assert_eq!(s.rejected.len(), 1, "{line}: {:?}", s.rejected);
            assert!(s.rejected[0].contains("is not an option of"), "{line}: {:?}", s.rejected);
            let applied = !s.input_delays.is_empty() || !s.driving_cells.is_empty() || !s.clock_latencies.is_empty()
                || !s.derates.is_empty() || s.late_derate.is_some() || s.early_derate.is_some() || !s.env.is_empty() || s.load.is_some();
            assert!(!applied, "{line} was applied");
        }
        let good = [
            "create_clock -per 10 -name clk [get_ports clk]",
            "set_input_delay -0.5 -clock clk [get_ports din]",
            "set_input_delay 1 -clock clk -add_delay -max [get_ports din]",
            "set_clock_uncertainty -setup 0.1 [get_clocks clk]",
            "set_load 1 [all_outputs]",
            // newer vendor SDC 1.8 declares it: read and kept
            "set_load -internal 1 [get_ports dout]",
            "set_input_delay 1 -clock clk [all_inputs -no_clocks]",
            "set_false_path -through [get_pins r/Q] -to [get_ports dout]",
            "set_multicycle_path 2 -setup -from [get_ports din] -to [get_ports dout]",
        ];
        for line in good {
            let s = Sdc::parse(line).unwrap();
            assert!(s.rejected.iter().all(|r| !r.contains("is not an option of")), "{line}: {:?}", s.rejected);
        }
    }
    /// The reference's `read_sdc` stops at a command it does not know (`invalid command name`)
    /// or an abbreviation of several (`Ambiguous command name`): its unknown handler resolves a
    /// word only to the ONE global command it begins. Measured 2026-10-31: `derive_pll_clocks`
    /// stopped the read, and a `set_input_delay` after it was never applied. So the whole read
    /// fails here (user 2026-10-31), wherever the command is evaluated, inside `[...]` and under
    /// `set` too; a bus subscript is a literal, and an `if` branch not taken is not evaluated.
    #[test]
    fn a_command_the_reference_does_not_know_stops_the_read() {
        let stops = [
            ("create_clock -name clk -period 10 [get_ports clk]\nderive_pll_clocks\nset_input_delay 1 -clock clk [get_ports din]", "invalid command name \"derive_pll_clocks\""),
            ("set_false_path -from [append_to_collection x [all_inputs]] -to [get_ports dout]", "invalid command name \"append_to_collection\""),
            ("remove_from_collection [all_inputs] clk", "invalid command name \"remove_from_collection\""),
            ("set x [derive_pll_clocks]", "invalid command name \"derive_pll_clocks\""),
            ("set_max 1", "Ambiguous command name \"set_max\""),
            ("set_clock_jitter 0.1 [get_clocks clk]", "invalid command name \"set_clock_jitter\""),
        ];
        for (text, why) in stops {
            let e = Sdc::parse(text).err().unwrap_or_else(|| panic!("{text}: read on"));
            assert!(e.0.contains(why), "{text}: {}", e.0);
        }
        let reads = [
            "set_max_trans 0.5",
            "set_load 1 [get_ports d[0]]",
            "set_load 1 [get_ports {d[7:0]}]",
            "report_checks",
            "if {0} {derive_pll_clocks}\nset_load 1 [get_ports dout]",
        ];
        for text in reads {
            assert!(Sdc::parse(text).is_ok(), "{text}: {:?}", Sdc::parse(text).err());
        }
        assert_eq!(Sdc::parse("set_max_trans 0.5 [current_design]").unwrap().max_transition, Some(0.5));
    }

    /// Not the reference's rule (it stops here too): a synthesis-only command it does not know is
    /// read past whole — its arguments not evaluated, so an unknown command inside them does not
    /// stop the read either — and what follows it is applied (user 2026-10-31).
    #[test]
    fn a_synthesis_only_command_is_read_past() {
        let s = Sdc::parse(
            "set_size_only [remove_from_collection [get_cells *] u0]\nset_critical_range 0.2 [current_design]\n\
             set_load 1 [get_ports dout]",
        )
        .unwrap();
        assert_eq!(s.ignored, ["set_size_only", "set_critical_range"]);
        assert_eq!(s.env.len(), 1);
        // an abbreviation of one is not recognised: the reference's prefix rule finds no command
        assert!(Sdc::parse("set_size_o [get_cells u0]").is_err());
    }

    /// Rule (`all_outputs`, `all_clocks`, `all_registers`, Sdc.tcl, observed 2026-10-31):
    /// `all_clocks` takes no argument, `all_registers` options only; `all_inputs` ignores a stray
    /// word. SDC gives `all_outputs` options (`-clock`, `-level_sensitive`, `-edge_triggered`) and
    /// no positional, so an option is read (and kept: nothing here evaluates it) and a positional
    /// still fails the command.
    #[test]
    fn a_query_given_arguments_it_does_not_take_fails_its_command() {
        let clk = "create_clock -name clk -period 10 [get_ports clk]\n";
        for cmd in [
            "set_output_delay 1 -clock clk [all_outputs dout]",
            "set_load 1 [all_outputs {}]",
            "set_clock_latency 1 [all_clocks x]",
            "set_false_path -to [all_registers foo]",
        ] {
            let s = Sdc::parse(&format!("{clk}{cmd}\n")).unwrap();
            assert!(s.rejected.iter().any(|r| r.contains("takes no positional")), "{cmd}: {:?}", s.rejected);
        }
        for cmd in [
            "set_output_delay 1 -clock clk [all_outputs]",
            "set_output_delay 1 -clock clk [all_outputs -clock clk]",
            "set_input_delay 1 -clock clk [all_inputs din]",
            "set_false_path -to [all_registers -clock clk -edge_triggered]",
        ] {
            let s = Sdc::parse(&format!("{clk}{cmd}\n")).unwrap();
            assert!(s.rejected.is_empty(), "{cmd}: {:?}", s.rejected);
        }
    }

    /// Rule (Tcl, so the reference, observed 2026-10-31): nothing is substituted inside a braced
    /// word — `[get_ports {$P}]` is the port `$P` ("port '$P' not found") — while a bare or quoted
    /// `$P` is the variable and `expr` substitutes its own braces.
    #[test]
    fn a_variable_inside_braces_is_text() {
        let head = "set P dout\nset V 0.5\n";
        for (cmd, objects, value) in [
            ("set_load 1 [get_ports {$P}]", "$P", 1.0),
            ("set_load 1 [get_ports {${P}}]", "${P}", 1.0),
            ("set_load 1 [get_ports $P]", "dout", 1.0),
            ("set_load 1 [get_ports \"$P\"]", "dout", 1.0),
            ("set_load [expr {$V * 2}] [get_ports dout]", "dout", 1.0),
        ] {
            let s = Sdc::parse(&format!("{head}{cmd}\n")).unwrap();
            assert_eq!((s.env[0].objects.join(" "), s.env[0].value), (objects.to_string(), value), "{cmd}");
        }
    }

    /// A LOOM EXTENSION (the reference does not have the command; user 2026-10-31):
    /// `[remove_from_collection [all_inputs] <ports>]` as the ports of an I/O delay is every input
    /// but those, as `lsearch -not -exact` is. OpenTitan's form names the clock port in braces,
    /// `{${MAIN_CLK_PIN}}`, which Tcl does not substitute: the literal name excludes no port. Any
    /// other form, or the form anywhere else, is refused with its command; a bare command still
    /// stops the read.
    #[test]
    fn remove_from_collection_is_every_port_but() {
        let s = Sdc::parse(
            "set MAIN_CLK_PIN clk_i\ncreate_clock -name clk_i -period 10 [get_ports clk_i]\n\
             set_input_delay 1 [remove_from_collection [all_inputs] {${MAIN_CLK_PIN}}] -clock ${MAIN_CLK_PIN}\n\
             set_input_delay 2 [remove_from_collection [all_inputs] [get_ports clk_i]] -clock clk_i\n\
             set_output_delay 3 [remove_from_collection [all_outputs] {a b}] -clock clk_i\n\
             set_input_delay 4 [remove_from_collection -intersect [all_inputs] {a}] -clock clk_i\n\
             set_input_delay 5 [remove_from_collection [get_ports *] {a}] -clock clk_i\n\
             set_false_path -from [remove_from_collection [all_inputs] clk_i]\n",
        )
        .unwrap();
        let got: Vec<(f64, String, Vec<String>, Option<String>)> =
            s.input_delays.iter().chain(&s.output_delays).map(|d| (d.value, d.accessor.clone(), d.ports.clone(), d.clock.clone())).collect();
        let clk = Some("clk_i".to_string());
        assert_eq!(
            got,
            [
                (1.0, "all_inputs_except".into(), vec!["${MAIN_CLK_PIN}".to_string()], clk.clone()),
                (2.0, "all_inputs_except".into(), vec!["clk_i".to_string()], clk.clone()),
                (3.0, "all_outputs_except".into(), vec!["a".to_string(), "b".to_string()], clk.clone()),
            ]
        );
        assert_eq!(s.unevaluated_queries.len(), 3, "{:?}", s.unevaluated_queries);
        assert!(s.exceptions.is_empty());
        // excluding two ports has no form the reference reads: not written, and said
        let w = s.write(&crate::sdc_write::WriteOpts::default());
        assert!(w.not_written.iter().any(|n| n.contains("more than one port")), "{:?}", w.not_written);
        assert!(w.text.contains("[lsearch -inline -all -not -exact [all_inputs] [get_ports {clk_i}]]"), "{}", w.text);
    }

    /// A LOOM EXTENSION (user 2026-10-31): `[load_of <lib>/<cell>/<pin>]` is known; with no
    /// library it is not evaluated — its command is refused by name and what follows is read (with
    /// the libraries: `load_of_is_the_library_pins_capacitance`); a bare `load_of`
    /// still stops the read, as any command the reference does not know.
    #[test]
    fn load_of_is_refused_with_its_command_and_the_read_goes_on() {
        let s = Sdc::parse("set_load [load_of lib/BUF/A] [all_outputs]\nset_load 1 [get_ports x]\n").unwrap();
        assert_eq!(s.env.len(), 1);
        assert_eq!(s.env[0].objects, ["x"]);
        assert!(s.unevaluated_queries.iter().any(|q| q.contains("load_of lib/BUF/A")), "{:?}", s.unevaluated_queries);
        assert!(Sdc::parse("load_of lib/BUF/A").err().is_some_and(|e| e.0.contains("invalid command name \"load_of\"")));
    }

    /// Rule (`set_clock_uncertainty`, Sdc.tcl): -from and -to come together and take only the
    /// value; without them, exactly the value and the objects, and no -rise/-fall. Each failure
    /// builds no uncertainty (Errors 420, 571, 421, 565, 567, 422).
    #[test]
    fn a_clock_uncertainty_the_reference_refuses_is_not_applied() {
        let clk = "create_clock -name a -period 10 [get_ports clk]\ncreate_clock -name b -period 5 [get_ports clk2]\n";
        for (cmd, why) in [
            ("set_clock_uncertainty", "Error 420"),
            ("set_clock_uncertainty x [get_clocks a]", "Error 571"),
            ("set_clock_uncertainty -from [get_clocks a] 0.1", "Error 421"),
            ("set_clock_uncertainty -rise_to [get_clocks a] 0.1", "Error 421"),
            ("set_clock_uncertainty -from [get_clocks a] -to [get_clocks b] 0.1 [get_clocks a]", "Error 565"),
            ("set_clock_uncertainty 0.1", "Error 567"),
            ("set_clock_uncertainty -rise 0.1 [get_clocks a]", "Error 422"),
            ("set_clock_uncertainty -fall -setup 0.1 [get_clocks a]", "Error 422"),
        ] {
            let s = Sdc::parse(&format!("{clk}{cmd}\n")).unwrap();
            assert!(s.uncertainties.is_empty(), "{cmd}: applied");
            assert!(s.rejected.iter().any(|r| r.contains(why)), "{cmd}: {:?}", s.rejected);
        }
        for cmd in [
            "set_clock_uncertainty 0.1 [get_clocks a]",
            "set_clock_uncertainty -from [get_clocks a] -to [get_clocks b] -rise 0.1",
            "set_clock_uncertainty -rise_from [get_clocks a] -fall_to [get_clocks b] -hold 0.1",
        ] {
            let s = Sdc::parse(&format!("{clk}{cmd}\n")).unwrap();
            assert_eq!(s.uncertainties.len(), 1, "{cmd}: {:?}", s.rejected);
        }
    }

    #[test]
    fn a_false_path_cuts_only_the_check_it_names() {
        let s = Sdc::parse(
            "set_false_path -from a\nset_false_path -hold -from a\nset_false_path -setup -from a\n\
             set_false_path -setup -hold -from a\nset_multicycle_path 2 -from a\nset_multicycle_path 1 -hold -from a\n",
        )
        .unwrap();
        let got: Vec<(bool, bool)> = s.exceptions.iter().map(|e| (e.applies_to(false), e.applies_to(true))).collect();
        assert_eq!(got, [(true, true), (false, true), (true, false), (true, true), (true, true), (false, true)]);
    }
}

#[cfg(test)]
mod pin_match_tests {
    use super::*;

    /// Rules (SdcNetwork::visitMatches / visitPinTail, ParseBus escapeChars): the database keeps
    /// `\[` `\]` in a name (`dpath.b_reg.out\[1\]$_DFFE_PP_`) and `write_sdc` drops them, so an
    /// instance path with a bracket that matches no instance as written is matched again with
    /// its brackets escaped — ONLY when the written form matches none (an instance literally
    /// named `x[2]` keeps `x\[2\]` out). A divider no instance takes is escaped and the scan
    /// goes on (`h/u1/A` is pin A of the flat `h\/u1`). The port part is a glob on the port
    /// name; a pattern with no divider names no pin.
    #[test]
    fn a_pin_pattern_escapes_brackets_only_when_nothing_matches() {
        let insts = ["dpath.b_reg.out\\[1\\]$_DFFE_PP_", "x[2]", "x\\[2\\]", "h\\/u1", "u1", "u2"];
        let pins: Vec<(usize, &str)> = vec![(0, "D"), (0, "CLK"), (1, "D"), (2, "D"), (3, "A"), (4, "A"), (4, "Q[0]"), (5, "A"), (5, "B\\[1\\]"), (5, "x/y")];
        let m = |p: &str| pins_matching(p, &insts, &pins);
        assert_eq!(m("dpath.b_reg.out[1]$_DFFE_PP_/D"), [0]);
        assert_eq!(m("dpath.b_reg.out\\[1\\]$_DFFE_PP_/CLK"), [1], "already escaped: matched as written");
        assert_eq!(m("x[2]/D"), [2], "the literal x[2] matches, so x\\[2\\] is not tried");
        assert_eq!(m("h/u1/A"), [4]);
        assert_eq!(m("u*/A"), [5, 7]);
        assert_eq!(m("u2/B[1]"), [8], "a port part with a bracket is tried escaped when no port matches it");
        assert_eq!(m("u2/x/y"), [9], "a port name is matched with its dividers escaped too");
        assert_eq!(m("u1/Q[0]"), [6]);
        assert_eq!(m("u1/*"), [5, 6]);
        assert_eq!(m("u2/*"), [7, 8, 9]);
        assert!(m("u1").is_empty() && m("nope/A").is_empty());
        assert_eq!(m("*").len(), pins.len(), "`*` alone is every instance pin");
    }

    /// Rules (SdcNetwork::findInstancesMatching1 / visitMatches, Network::findChildrenMatching):
    /// on a flat netlist the instance is the whole pattern with its dividers escaped, a glob on
    /// the instance name; a bracket not escaped retries the pattern with its brackets escaped,
    /// ONLY when the written form matches no instance.
    #[test]
    fn an_instance_pattern_escapes_dividers_and_retries_brackets() {
        let insts = ["dpath.b_reg.out\\[1\\]$_DFFE_PP_", "x[2]", "x\\[2\\]", "h\\/u1", "u1", "u2"];
        let m = |p: &str| insts_matching(p, &insts);
        assert_eq!(m("dpath.b_reg.out[1]$_DFFE_PP_"), [0]);
        assert_eq!(m("dpath.b_reg.out\\[1\\]$_DFFE_PP_"), [0], "already escaped: matched as written");
        assert_eq!(m("x[2]"), [1], "the literal x[2] matches, so x\\[2\\] is not tried");
        assert_eq!(m("h/u1"), [3], "a divider is escaped: the flat instance h\\/u1");
        assert_eq!(m("h/*"), [3]);
        assert_eq!(m("u*"), [4, 5]);
        assert_eq!(m("*"), [0, 1, 2, 3, 4, 5]);
        assert_eq!(m("*out[1]*"), [0], "a glob with a bracket is retried escaped too");
        assert!(m("u3").is_empty() && m("u1/A").is_empty());
    }
}

#[cfg(test)]
mod bus_match_tests {
    use super::*;
    /// Rule (Network::findPortsMatching, find_port_pins_matching): a pattern is matched against
    /// PORT names, a bus port's name being the bus, and a bus it matches names every bit. A
    /// subscripted pattern matches the bus part as a pattern, then one bit, a range or `[*]`.
    /// OpenTitan's ibex SDC writes `set_input_delay -clock clk_i 0.0 boot_addr_i`: the
    /// reference constrains all 32 bits; an exact-name lookup constrained none.
    #[test]
    fn a_bus_name_matches_each_of_its_bits() {
        for (p, n, hit) in [
            ("boot_addr_i", "boot_addr_i[3]", true),
            ("boot*", "boot_addr_i[3]", true),
            ("*_i", "boot_addr_i[3]", true),
            ("boot_addr", "boot_addr_i[3]", false),
            ("boot_addr_i[3]", "boot_addr_i[3]", true),
            ("boot_addr_i[4]", "boot_addr_i[3]", false),
            ("boot_addr_i[*]", "boot_addr_i[3]", true),
            ("b*[7:0]", "boot_addr_i[3]", true),
            ("boot_addr_i[0:2]", "boot_addr_i[3]", false),
            ("clk", "clk", true),
            ("clk", "clk2", false),
            // an escaped subscript is part of a scalar's name, not a bus bit
            ("a", "a\\[3]", false),
            // parseBusName: a subscript starting with a digit is read as an integer (stoi), one
            // starting with `*` names every bit; anything else is no subscript, and a plain
            // pattern never matches a bit's own name
            ("req_msg[1?]", "req_msg[1]", true),
            ("req_msg[1?]", "req_msg[10]", false),
            ("req_msg[1*]", "req_msg[12]", false),
            ("req_msg[*x]", "req_msg[5]", true),
            ("req_msg[?]", "req_msg[5]", false),
        ] {
            assert_eq!(object_matches(p, n), hit, "{p} vs {n}");
        }
    }

    #[test]
    fn get_ports_against_the_design_names_each_bit_of_a_bus() {
        let d = SdcDesign { ports: (0..3).map(|k| (format!("d[{k}]"), "input".into())).chain([("clk".into(), "input".into())]).collect(), ..Default::default() };
        let parts: Vec<String> = ["get_ports", "-filter", "direction==input", "d"].iter().map(|s| s.to_string()).collect();
        assert_eq!(crate::sdc_query::eval(&parts, &d).unwrap(), ["d[0]", "d[1]", "d[2]"]);
    }

    /// Rule (Tcl `llength` on the collection a `-quiet` query returns, empty when nothing
    /// matches): a guard `if {[llength $p] > 0}` runs its body only on a design that has the
    /// object. Without a design the count is unknown and the `if` refuses, as before.
    #[test]
    fn llength_of_a_query_is_counted_on_the_design() {
        let text = "set te [get_ports -quiet io_te]\n\
                    if {[llength $te] > 0} {\n  set_input_delay 1 -clock c $te\n}\n\
                    if {[llength [get_ports -quiet nope]] > 0} {\n  set_input_delay 2 -clock c [get_ports nope]\n}\n\
                    if {[llength {a b c}] == 3} {\n  set_input_delay 3 -clock c [get_ports clk]\n}\n";
        let with = |ports: &[&str]| {
            let d = SdcDesign { ports: ports.iter().map(|p| (p.to_string(), "input".into())).collect(), ..Default::default() };
            Sdc::parse_with(&format!("create_clock -name c -period 1\n{text}"), &d).unwrap()
        };
        let s = with(&["io_te", "clk"]);
        assert_eq!(s.input_delays.len(), 2, "the io_te guard and the plain list ran: {:?}", s.input_delays);
        assert!(!s.ignored.iter().any(|i| i == "if"), "{:?}", s.ignored);
        let s = with(&["clk"]);
        assert_eq!(s.input_delays.len(), 1, "only the plain-list body: {:?}", s.input_delays);
        let s = Sdc::parse(&format!("create_clock -name c -period 1\n{text}")).unwrap();
        assert_eq!(s.ignored.iter().filter(|i| *i == "if").count(), 2, "no design: the two query guards refuse");
    }

    /// Rule (`create_generated_clock` in the reference's `sdc/Sdc.tcl`): it stops with Error 377
    /// when `-source` is missing, 388 when no -divide_by/-multiply_by/-edges/-combinational, and
    /// 380 when both -divide_by and -multiply_by — the clock is never made. A PLL output an FPGA
    /// tool infers (`create_generated_clock -name clk_main [get_pin pll/CLKOUT0]`) was read as a
    /// clock of period 0.
    #[test]
    fn a_generated_clock_the_reference_refuses_is_not_made() {
        let base = "create_clock -name m -period 10 [get_ports clk]\n";
        for (line, code) in [
            ("create_generated_clock -name g [get_pins pll/CLKOUT0]", "Error 377"),
            ("create_generated_clock -name g -source [get_ports clk] [get_pins d/Q]", "Error 388"),
            ("create_generated_clock -name g -source [get_ports clk] -divide_by 2 -multiply_by 2 [get_pins d/Q]", "Error 380"),
            ("create_generated_clock -name g -source [get_ports clk] -divide_by 0 [get_pins d/Q]", "Error 381"),
            ("create_generated_clock -name g -source [get_ports clk] -edges {1 3} [get_pins d/Q]", "Error 385"),
            ("create_generated_clock -name g -source [get_ports clk] -divide_by 2 -duty_cycle 50 [get_pins d/Q]", "Error 390"),
        ] {
            let s = Sdc::parse(&format!("{base}{line}\n")).unwrap();
            assert_eq!(s.clocks.len(), 1, "{line}: not made");
            assert!(s.rejected.iter().any(|r| r.contains(code)), "{line}: {:?}", s.rejected);
        }
        let s = Sdc::parse(&format!("{base}create_generated_clock -name g -source [get_ports clk] -divide_by 2 [get_pins d/Q]\n")).unwrap();
        assert_eq!(s.clocks.len(), 2);
        assert_eq!(s.clocks[1].period, 20.0);
        assert!(s.rejected.is_empty(), "{:?}", s.rejected);
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

    // `lsearch -not -exact` drops the inputs EQUAL to the pattern, and the reference's elements
    // are handles (observed 2026-10-31): `[get_ports P]` of one port removes P; a name, or a query
    // of two ports, equals no element and removes nothing; a wildcard query is not read.
    #[test]
    fn every_input_but_the_clock_port() {
        let t = "set clk_port [get_ports clk]\ncreate_clock -name clk -period 690 $clk_port\n\
                 set non_clock_inputs [lsearch -inline -all -not -exact [all_inputs] $clk_port]\n\
                 set_input_delay 138 -clock clk $non_clock_inputs\n";
        let s = Sdc::parse(t).unwrap();
        let d = &s.input_delays[0];
        assert_eq!((d.accessor.as_str(), d.ports.clone(), d.default), ("all_inputs_except", vec!["clk".to_string()], false));
        assert_eq!(all_ports_except("[lsearch -inline -all -not -exact [all_inputs] [get_ports {a b}]]"), Some(("all_inputs_except".into(), vec![])));
        assert_eq!(all_ports_except("[lsearch -inline -all -not -exact [all_inputs] clk]"), Some(("all_inputs_except".into(), vec![])));
        assert_eq!(all_ports_except("[lsearch -inline -all -not -exact [all_inputs] {clk}]"), Some(("all_inputs_except".into(), vec![])));
        assert_eq!(all_ports_except("[lsearch -inline -all -not -exact [all_inputs] [get_ports c*]]"), None);
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
        // and an undefined variable is Tcl's error, not a value of 0: the read stops there
        let d = Sdc::parse("set_input_delay 1 -clock clk\n").unwrap();
        assert_eq!(d.rejected.len(), 1);
        assert!(Sdc::parse("set_input_delay $undefined -clock clk [get_ports a]\n").is_err());
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
    -to [get_clocks {clk}] 0.5000\n").unwrap();
        assert_eq!(s.propagated, vec![Objects { accessor: "get_clocks".into(), names: vec!["clk".into()], ..Default::default() }]);
        assert_eq!((s.clock_latencies.len(), s.clock_latencies[0].flags.clone(), s.clock_latency), (1, vec!["-source".to_string()], 0.2));
        let p = &s.path_delays[0];
        assert_eq!((p.max, p.value, p.flags.clone()), (true, 0.5, vec!["-ignore_clock_latency".to_string()]));
        assert_eq!(p.objects, vec![("-from".to_string(), Objects { accessor: "get_pins".into(), names: vec!["r1/CK".into()], ..Default::default() }), ("-to".to_string(), Objects { accessor: "get_clocks".into(), names: vec!["clk".into()], ..Default::default() })]);
        assert_eq!(s.path_delays.len(), 1);
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

    /// Rule (sdc/Sdc.tcl `set_clock_groups`; probe `clock-groups-probe.py`
    /// include-generated-dropped, 2026-11-07): an option the command does not have is warned
    /// (STA-0402) and ignored, a stray word too (STA-0403), and the groups ARE applied — the
    /// command is not refused. `-include_generated_clocks` therefore puts no generated clock in a
    /// group: the reference times A's generated clock against B.
    #[test]
    fn clock_groups_warn_on_an_unknown_word_and_still_apply() {
        let s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports a]\n\
             create_clock -name B -period 7 [get_ports b]\n\
             create_generated_clock -name G -source [get_ports a] -divide_by 2 [get_pins d/Q]\n\
             set_clock_groups -asynchronous -include_generated_clocks -group A -group B stray\n",
        )
        .expect("parse");
        assert!(s.rejected.is_empty(), "{:?}", s.rejected);
        assert_eq!(s.ignored_words, ["set_clock_groups -include_generated_clocks", "set_clock_groups stray"]);
        let u = |a, b| clocks_unrelated(&s.clock_groups, a, b);
        assert!(u("A", "B") && !u("G", "B") && !u("A", "G"));
        let w = s.write(&crate::sdc_write::WriteOpts::default()).text;
        assert!(w.contains("set_clock_groups") && !w.contains("include_generated"), "{w}");
    }

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

    /// Tcl: at the top level `::x` names the global `x`, so `set ::x v` then `$::x` or `$x` reads
    /// `v`. The reference's own SDC test (`sdc_variables`) sets `::sta_crpr_mode` and reads it back;
    /// storing the name with its `::` made that read an undefined variable and stopped the file.
    #[test]
    fn a_global_qualified_variable_is_the_same_variable() {
        let s = Sdc::parse("set ::p 7\nset q $::p\ncreate_clock -name c -period $q [get_ports clk]\nset ::r 3\ncreate_clock -name d -period $r [get_ports clk2]\n")
            .expect("`::p` set at the top level reads back as `$::p` and `::r` as `$r`");
        assert_eq!(s.clocks.iter().map(|c| c.period).collect::<Vec<_>>(), vec![7.0, 3.0]);
    }

    /// Rule (Tcl, so the reference): reading an undefined variable is an error, and an error in a
    /// sourced file stops `source` there — the reference's `read_sdc` reads nothing after it (the
    /// pinned OpenSTA on `set X $X` then a `create_clock`: `can't read "X": no such variable`, and
    /// the clock after it is not made). The messages are tclsh 8.6's, word for word. Read on, `set
    /// MAIN_CLK_PIN $MAIN_CLK_PIN` stored the text, and a clock was made on a pin named
    /// `$MAIN_CLK_PIN`; before that, `${DRIVING_CELL}` became a lib cell `$`.
    #[test]
    fn an_undefined_variable_stops_the_read() {
        let stop = |text: &str| Sdc::parse(&format!("create_clock -name c -period 8 [get_ports clk_i]\n{text}\ncreate_clock -name d -period 4 [get_ports d]\n")).unwrap_err().0;
        for (text, msg) in [
            ("set MAIN_CLK_PIN $MAIN_CLK_PIN", "can't read \"MAIN_CLK_PIN\": no such variable"),
            ("set_driving_cell -lib_cell ${DRIVING_CELL} [all_inputs]", "can't read \"DRIVING_CELL\": no such variable"),
            ("set_load 1 [get_ports $port]", "can't read \"port\": no such variable"),
            ("puts \"x$T\"", "can't read \"T\": no such variable"),
            ("set_load [expr {$R * 2}] [get_ports x]", "can't read \"R\": no such variable"),
            ("set ::V $::V", "can't read \"::V\": no such variable"),
            ("set_load $::env(VYGES_NO_SUCH_VAR_ZZ) [get_ports x]", "no such variable (read trace on \"::env(VYGES_NO_SUCH_VAR_ZZ)\")"),
            ("if {$Q} { set_load 1 [get_ports x] }", "can't read \"Q\": no such variable"),
            ("while {$n < 3} { incr n }", "can't read \"n\": no such variable"),
            ("proc f {} { return $nope }\nset_load [f] [get_ports x]", "no such variable"),
        ] {
            let e = stop(text);
            assert!(e.contains(msg), "{text}: {e}");
            assert!(e.contains("so this file is not read"), "{text}: {e}");
        }
        // defined, or inside braces, or an `info exists` test: no error
        let s = Sdc::parse_opts(
            "set_load $L [get_ports x]\nset_false_path -to [get_pins {a$_DFF_P_/D}]\nif {[info exists W]} { set_load $W [get_ports y] }\n",
            &ReadOpts { defines: vec![("L".into(), "1".into())], base_dir: None },
        )
        .unwrap();
        assert_eq!((s.env.len(), s.exceptions.len()), (1, 1));
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

#[cfg(test)]
mod query_spelling_tests {
    use super::*;

    /// Rule (reference `define_cmd_alias`): `get_port` IS `get_ports`, `get_cell` IS `get_cells`,
    /// and so on for every singular query. Read by exact name, they were refused.
    #[test]
    fn singular_queries_are_their_plurals() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_port clk]\n\
             set_input_delay 1 -clock [get_clock c] [get_port {a b}]\n\
             set_false_path -from [get_cell r1] -to [get_pin r2/D]\n",
        )
        .unwrap();
        assert!(s.rejected.is_empty(), "{:?}", s.rejected);
        assert_eq!(s.clocks[0].source, "clk");
        assert_eq!(s.input_delays.len(), 1);
        assert_eq!(s.input_delays[0].ports, ["a", "b"]);
        assert_eq!(s.exception_objects[0][0][0].accessor, "get_cells");
        assert_eq!(s.exception_objects[0][1][0].accessor, "get_pins");
    }

    /// Inside braces nothing is substituted, so nothing is rewritten.
    #[test]
    fn a_braced_query_word_is_not_rewritten() {
        assert_eq!(canonical_queries("set x {[get_port a]}"), "set x {[get_port a]}");
        assert_eq!(canonical_queries("set_load 1 [get_port a]"), "set_load 1 [get_ports a]");
        assert_eq!(canonical_queries("set_load 1 a[0]"), "set_load 1 a[0]");
    }

    /// `get_lib_cells` / `get_lib_pins` / `get_libs` name their patterns; the command word itself
    /// was taken as the object name.
    #[test]
    fn library_queries_name_their_patterns() {
        assert_eq!(resolve_objs("[get_lib_cells {lib/BUF_X1}]"), ["lib/BUF_X1"]);
        assert_eq!(resolve_objs("[get_lib_pins lib/BUF_X1/A]"), ["lib/BUF_X1/A"]);
        assert_eq!(resolve_objs("[get_libs mylib]"), ["mylib"]);
        assert_eq!(resolve_objs("[get_lib_cell lib/INV]"), ["get_lib_cell"], "resolve_objs sees canonical words only");
    }
}

#[cfg(test)]
mod scope_and_rule_tests {
    use super::*;
    use crate::sdc_write::WriteOpts;

    /// A design-rule limit on objects is kept and written back, not only flagged.
    #[test]
    fn design_rules_on_objects_are_kept_and_written() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]\n\
             set_max_transition -clock_path 0.2 [get_clocks c]\n\
             set_max_transition 0.3 [get_ports {a b}]\n\
             set_max_fanout 8 [get_ports a]\n",
        )
        .unwrap();
        assert_eq!(s.max_transition_objects.len(), 2);
        assert_eq!(s.max_transition_objects[0].flags, ["-clock_path"]);
        assert_eq!(s.max_transition_objects[0].value, 0.2);
        assert_eq!(s.max_fanout_objects[0].objects.names, ["a"]);
        let w = s.write(&WriteOpts::default());
        assert!(w.not_written.is_empty(), "{:?}", w.not_written);
        assert!(w.text.contains("set_max_transition -clock_path 0.2 [get_clocks {c}]"), "{}", w.text);
        assert!(w.text.contains("set_max_fanout 8 [get_ports {a}]"), "{}", w.text);
        assert!(s.not_applied().is_empty(), "{:?}", s.not_applied());
    }

    #[test]
    fn a_design_rule_without_objects_is_rejected() {
        let s = Sdc::parse("set_max_transition 0.3\n").unwrap();
        assert_eq!(s.rejected.len(), 1, "{:?}", s.rejected);
        assert!(s.rejected[0].starts_with("set_max_transition: missing objects"), "{:?}", s.rejected);
    }

    /// `current_instance u1` makes every later name relative to `u1` (see `sdc/scope.rs`); a bare
    /// name under it needs the design to classify, so its command is refused, not read from the
    /// top. Back to the top is read.
    #[test]
    fn a_changed_instance_scope_reads_under_it() {
        let s = Sdc::parse("current_instance u1\nset_false_path -from a\n").unwrap();
        assert!(s.exceptions.is_empty() && s.rejected[0].contains("current_instance u1"), "{:?}", s.rejected);
        assert!(Sdc::parse("current_instance\nset_false_path -from a\n").is_ok());
        assert!(Sdc::parse("current_instance /\n").is_ok());
        assert!(Sdc::parse("set_hierarchy_separator .\n").is_err());
        assert!(Sdc::parse("set_hierarchy_separator /\n").is_ok());
    }
}

/// The synthesis-tool dialect: what each command is read as. Tcl's own rules
/// where the reference has the command (`proc`, `source`, `puts`, `global`); LOOM EXTENSIONS for
/// the collection commands it does not have.
#[cfg(test)]
mod dialect_tests {
    use super::*;

    fn ports(s: &Sdc) -> Vec<Vec<String>> {
        s.input_delays.iter().map(|d| d.ports.clone()).collect()
    }

    /// Tcl `proc`: parameters bound in order, a default when the call leaves one out, `args`
    /// taking the rest; the body's variables are the call's own (gone when it returns) unless it
    /// declares them `global`.
    #[test]
    fn a_proc_runs_its_body_with_its_parameters_set() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]\n\
             proc io {p {d 2}} { set local 1\n set_input_delay $d -clock c [get_ports $p] }\n\
             io a\nio b 3\n\
             proc g {} { global G\n set G 7 }\ng\n\
             set_input_delay $G -clock c [get_ports e]\n",
        )
        .unwrap();
        let got: Vec<(f64, Vec<String>)> = s.input_delays.iter().map(|d| (d.value, d.ports.clone())).collect();
        assert_eq!(got, [(2.0, vec!["a".to_string()]), (3.0, vec!["b".into()]), (7.0, vec!["e".into()])]);
        assert!(s.ignored.iter().all(|i| i != "proc"), "{:?}", s.ignored);
        // a call's local is gone after it returns: reading it is Tcl's error, which stops the read
        let e = Sdc::parse("proc f {} { set local 1 }\nf\nset_load $local [get_ports x]\n").unwrap_err();
        assert!(e.0.starts_with("can't read \"local\": no such variable"), "{}", e.0);
    }

    /// Tcl: a variable in a braced `if` condition is one operand, so a string value compares as
    /// a string; substituted as text, `{$dir == "in"}` became the bare word `in` and was refused.
    #[test]
    fn a_string_variable_in_a_condition_is_an_operand() {
        let s = Sdc::parse("set dir in
if {$dir == \"in\"} { set_load 1 [get_ports a] } else { set_load 2 [get_ports b] }
").unwrap();
        assert_eq!(s.env.len(), 1);
        assert_eq!(s.env[0].objects, ["a"]);
        assert!(s.ignored.iter().all(|i| i != "if"), "{:?}", s.ignored);
    }

    /// Tcl `for` / `while` / `incr`: the test before each pass with the variables as they are
    /// then; `switch` exact, glob, `default` and `-` fall-through; `lappend` builds a list. A loop
    /// that never ends stops the read rather than hanging it.
    #[test]
    fn loops_switch_and_lappend_run_as_tcl_runs_them() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]
             for {set i 0} {$i < 3} {incr i} { set_input_delay $i -clock c [get_ports d$i] }
             set n 2
while {$n > 0} { set_output_delay $n -clock c [get_ports q$n]
 incr n -1 }
             set mode fast
switch -glob $mode { slow { set_load 1 [get_ports a] } f* - x { set_load 2 [get_ports b] } default { set_load 3 [get_ports c] } }
             switch $mode { slow { set_load 4 [get_ports e] } }
             lappend L x
lappend L {y z}
set_load 5 [get_ports $L]
",
        )
        .unwrap();
        let ins: Vec<(f64, Vec<String>)> = s.input_delays.iter().map(|d| (d.value, d.ports.clone())).collect();
        assert_eq!(ins, [(0.0, vec!["d0".to_string()]), (1.0, vec!["d1".into()]), (2.0, vec!["d2".into()])]);
        let outs: Vec<f64> = s.output_delays.iter().map(|d| d.value).collect();
        assert_eq!(outs, [2.0, 1.0]);
        let env: Vec<Vec<String>> = s.env.iter().map(|e| e.objects.clone()).collect();
        assert_eq!(env, [vec!["b".to_string()], vec!["x".into(), "y".into(), "z".into()]]);
        assert!(Sdc::parse("while {1} { set x 1 }
").is_err());
        assert!(Sdc::parse("for {set i 0} {$i < 3} {incr i} { if {$i} { break } }
").unwrap().rejected.is_empty());
    }

    /// Rule (Tcl 8.6 `switch`, measured on the reference's interpreter 2026-11-03): `-regexp` is
    /// `regexp`, UNANCHORED (`slow` hits `^s`, `x9` hits `[0-9]`, `fast` falls to `default`);
    /// `-nocase` folds case for `-glob` too; a second mode option is an error, nothing runs.
    #[test]
    fn switch_regexp_and_nocase_as_tcl() {
        let loads = |text: &str| -> Vec<Vec<String>> { Sdc::parse(text).unwrap().env.iter().map(|e| e.objects.clone()).collect() };
        for (v, port) in [("slow", "s"), ("fast", "dflt"), ("x9", "digit")] {
            let t = format!("set v {v}\nswitch -regexp $v {{ {{^s}} {{ set_load 1 [get_ports s] }} {{[0-9]}} {{ set_load 1 [get_ports digit] }} default {{ set_load 1 [get_ports dflt] }} }}\n");
            assert_eq!(loads(&t), [vec![port.to_string()]], "{v}");
        }
        assert_eq!(loads("switch -nocase -glob ABC { a* { set_load 1 [get_ports hit] } default { set_load 1 [get_ports miss] } }\n"), [vec!["hit".to_string()]]);
        let s = Sdc::parse("switch -glob -regexp abc { b { set_load 1 [get_ports r] } default { set_load 1 [get_ports g] } }\n").unwrap();
        assert!(s.env.is_empty() && s.rejected.iter().any(|r| r.contains("already found")), "{:?}", s.rejected);
    }

    /// What a proc call cannot be read as is refused, and the read goes on; a recursive proc
    /// stops it rather than looping. A proc that only `return`s a value is a command that does
    /// nothing as a statement (Tcl 8.6 `return`): run, not refused.
    #[test]
    fn a_proc_that_cannot_be_run_is_refused() {
        let s = Sdc::parse("proc f {a} { set_load $a [get_ports x] }\nf\nf 1 2\nproc r {} { return 1 }\nr\nset_load 4 [get_ports y]\n").unwrap();
        assert_eq!(s.env.len(), 1);
        assert_eq!(s.rejected.len(), 2, "{:?}", s.rejected);
        assert!(Sdc::parse("proc r {} { r }\nr\n").is_err());
    }

    /// Tcl `source`: the file's commands run in place, found from the working directory or the
    /// reading file's own; a file not there is Tcl's error, which stops the read.
    #[test]
    fn source_reads_the_file_in_place() {
        let dir = std::env::temp_dir().join(format!("loom-sdc-source-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("inner.sdc"), "set_input_delay 1 -clock c [get_ports a]\n").unwrap();
        std::fs::write(dir.join("top.sdc"), "create_clock -name c -period 10 [get_ports clk]\nsource -echo inner.sdc\nset_input_delay 2 -clock c [get_ports b]\n").unwrap();
        let s = Sdc::load(&dir.join("top.sdc").to_string_lossy()).unwrap();
        assert_eq!(ports(&s), [vec!["a".to_string()], vec!["b".to_string()]]);
        std::fs::write(dir.join("loop.sdc"), "source loop.sdc\n").unwrap();
        assert!(Sdc::load(&dir.join("loop.sdc").to_string_lossy()).is_err());
        assert!(Sdc::parse("source /no/such/file.sdc\n").is_err());
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// A flow's variables, defined before the file is read: what its setup script supplies.
    #[test]
    fn defines_are_set_before_the_file() {
        let text = "create_clock -name c -period [expr 10*$F] [get_ports clk]\n";
        assert!(Sdc::parse(text).is_err());
        let s = Sdc::parse_opts(text, &ReadOpts { defines: vec![("F".into(), "0.5".into())], ..Default::default() }).unwrap();
        assert_eq!(s.clocks[0].period, 5.0);
    }

    /// Collection commands (LOOM EXTENSIONS): `append_to_collection` grows a variable,
    /// `add_to_collection` is a list, `get_object_name` its names; the ones that select by
    /// attribute or connectivity are refused by name and the read goes on; `puts` changes nothing.
    #[test]
    fn collection_commands_are_lists_or_refused_by_name() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]\nputs \"reading [lreplace a b]\"\n\
             append_to_collection B [get_ports x*]\nappend_to_collection B [get_ports y]\n\
             set_input_delay 1 -clock c $B\n\
             set_input_delay 2 -clock c [add_to_collection -unique [get_ports p] [get_ports q]]\n\
             set_input_delay 3 -clock c [get_ports [get_object_name [get_ports r]]]\n\
             set_input_delay 4 -clock c [filter_collection [all_inputs] {name =~ a*}]\n\
             set_dont_touch [get_designs -h prim_*]\n\
             foreach_in_collection p [all_inputs] { set_load 1 $p }\n\
             set_input_delay 5 -clock c [get_ports z]\n",
        )
        .unwrap();
        let v = |x: &[&str]| x.iter().map(|s| s.to_string()).collect::<Vec<_>>();
        assert_eq!(ports(&s), [v(&["x*", "y"]), v(&["p", "q"]), v(&["r"]), v(&["z"])]);
        assert_eq!(s.unevaluated_queries.len(), 1, "{:?}", s.unevaluated_queries);
        assert!(s.rejected.iter().any(|r| r.starts_with("set_dont_touch")), "{:?}", s.rejected);
        // a body that hands `$p` to a constraint as its object is that constraint on the collection
        assert!(!s.rejected.iter().any(|r| r.starts_with("foreach_in_collection")), "{:?}", s.rejected);
        assert_eq!(s.env.len(), 1, "set_load on [all_inputs]: {:?}", s.env);
        // inside braces nothing is a command, so nothing is rewritten
        assert_eq!(dialect_queries("x {[add_to_collection a b]} [add_to_collection -unique a b]"), "x {[add_to_collection a b]} [list a b]");
    }
}

/// Arcs and senses the reference models, read as its SDC / search Tcl reads
/// them (`set_disable_timing`, `set_sense` / `set_clock_sense_cmd1`, `set_disable_clock_gating_check`).
#[cfg(test)]
mod arc_tests {
    use super::*;

    type Keys = Vec<(String, String)>;

    /// Rule (`set_disable_timing`): keys `-from`/`-to`, no flags, exactly one object list
    /// (`check_argc_eq1`); `unset_` the same, kept in order with the sets.
    #[test]
    fn disable_timing_keeps_from_to_and_one_object_list() {
        let s = Sdc::parse(
            "set_disable_timing -from A -to Y [get_cells u1]\nset_disable_timing [get_pins u2/A]\n\
             set_disable_timing [get_pins u3/A] [get_pins u4/A]\nunset_disable_timing [get_pins u2/A]\n",
        )
        .unwrap();
        let got: Vec<(&str, Keys, Vec<String>)> =
            s.disabled_timing.iter().map(|c| (c.cmd.as_str(), c.keys.clone(), c.objects.clone())).collect();
        assert_eq!(
            got,
            [
                ("set_disable_timing", vec![("-from".into(), "A".into()), ("-to".into(), "Y".into())], vec!["u1".to_string()]),
                ("set_disable_timing", vec![], vec!["u2/A".to_string()]),
                ("unset_disable_timing", vec![], vec!["u2/A".to_string()]),
            ]
        );
        assert_eq!(s.rejected.len(), 1, "{:?}", s.rejected);
        // only the refused one is reported as not applied; the three read are modelled
        let na: Vec<usize> = s.not_applied().iter().filter(|n| n.command.ends_with("disable_timing")).map(|n| n.count).collect();
        assert_eq!(na, [1]);
        let w = s.write(&crate::sdc_write::WriteOpts::default()).text;
        assert!(w.contains("set_disable_timing -from {A} -to {Y} [get_cells {u1}]"), "{w}");
        assert!(w.contains("unset_disable_timing [get_pins {u2/A}]"), "{w}");
    }

    /// Rule (`set_sense`): `-type clock` is a clock sense, and so is a `set_sense` without `-type`
    /// (SDC 2.1 makes clock the default); `-type data` is not supported (the reference's Warning
    /// 413); `-type` with no value is refused. `set_clock_sense` always applies; `-clocks` wins
    /// over `-clock`; a word it does not know fails it (wrong # args).
    ///
    /// ⚠️ Reference quirk, pinned and NOT followed: the reference's `set_sense` proc sets the type
    /// to clock by default, but calls its sense command only inside `if` `-type` was given, so a
    /// `set_sense` without `-type` applies nothing there, though its help says clock is the
    /// default. Here it is read as clock and listed in `senses_type_defaulted` for lint to report.
    #[test]
    fn a_sense_is_applied_as_the_reference_applies_it() {
        let s = Sdc::parse(
            "create_clock -name c -period 10 [get_ports clk]\ncreate_clock -name d -period 10 [get_ports clk2]\n\
             set_sense -type clock -stop_propagation -clocks [get_clocks c] [get_pins u1/Y]\n\
             set_sense -stop_propagation [get_pins u2/Y]\nset_sense -type data -positive [get_pins u3/Y]\n\
             set_clock_sense -neg -clock [get_clocks d] -clocks c [get_pins u4/Y]\n\
             set_clock_sense -logical_stop_propagation [get_pins u5/Y]\n",
        )
        .unwrap();
        let got: Vec<(&str, Keys, Vec<String>, Vec<String>)> =
            s.senses.iter().map(|c| (c.cmd.as_str(), c.keys.clone(), c.flags.clone(), c.objects.clone())).collect();
        assert_eq!(
            got,
            [
                ("set_sense", vec![("-clocks".into(), "c".into())], vec!["-stop_propagation".to_string()], vec!["u1/Y".to_string()]),
                ("set_sense", vec![], vec!["-stop_propagation".to_string()], vec!["u2/Y".to_string()]),
                ("set_clock_sense", vec![("-clocks".into(), "c".into())], vec!["-negative".to_string()], vec!["u4/Y".to_string()]),
            ]
        );
        assert_eq!(s.senses_type_defaulted, [1]);
        assert_eq!(s.rejected.len(), 2, "{:?}", s.rejected);
        assert!(s.rejected[0].contains("-type data"), "{:?}", s.rejected);
        let w = s.write(&crate::sdc_write::WriteOpts::default()).text;
        assert!(w.contains("set_sense -type clock -clocks [get_clocks {c}] -stop_propagation [get_pins {u1/Y}]"), "{w}");
        // the defaulted one is written back with the type it was read as
        assert!(w.contains("set_sense -type clock -stop_propagation [get_pins {u2/Y}]"), "{w}");
    }

    /// Rule: `-type` is a valued option; written with no value it names no type, and is refused
    /// rather than read as the default.
    #[test]
    fn a_sense_type_with_no_value_is_refused() {
        let s = Sdc::parse("set_sense -stop_propagation [get_pins u2/Y] -type\n").unwrap();
        assert!(s.senses.is_empty(), "{:?}", s.senses);
        assert!(s.senses_type_defaulted.is_empty());
        assert_eq!(s.rejected, ["set_sense: -type needs a value (clock or data); not applied"]);
    }

    /// Rule (`set_disable_clock_gating_check { objects }`): one object list, Tcl's own arity.
    #[test]
    fn a_gating_check_disable_names_its_objects() {
        let s = Sdc::parse("set_disable_clock_gating_check [get_pins cg1/B]\nset_disable_clock_gating_check a b\nunset_disable_clock_gating_check [get_cells cg2]\n").unwrap();
        let got: Vec<(&str, Vec<String>)> = s.disabled_gating_checks.iter().map(|c| (c.cmd.as_str(), c.objects.clone())).collect();
        assert_eq!(got, [("set_disable_clock_gating_check", vec!["cg1/B".to_string()]), ("unset_disable_clock_gating_check", vec!["cg2".to_string()])]);
        assert_eq!(s.rejected.len(), 1);
        let w = s.write(&crate::sdc_write::WriteOpts::default()).text;
        assert!(w.contains("set_disable_clock_gating_check [get_pins {cg1/B}]"), "{w}");
    }
}

/// Commands the reference errors on (OpenSTA `sdc/Sdc.tcl`, `tcl/CmdArgs.tcl` at cc9027d6) and this
/// reader once applied: found by the SDC spec matrix as LENIENT, each now rejected with the reason.
#[cfg(test)]
mod reference_error_tests {
    use super::*;
    use crate::liberty::LibGroups;

    const CLK: &str = "create_clock -name clk -period 10 [get_ports clk]\n";

    fn rejected(text: &str) -> Vec<String> {
        Sdc::parse(&format!("{CLK}{text}\n")).unwrap().rejected
    }

    fn rejected_with(text: &str, d: &SdcDesign) -> Vec<String> {
        Sdc::parse_with(&format!("{CLK}{text}\n"), d).unwrap().rejected
    }

    fn design() -> SdcDesign {
        SdcDesign {
            ports: vec![("clk".into(), "input".into()), ("din".into(), "input".into())],
            insts: vec![("r".into(), "DFF".into())],
            pins: vec![(0, "D".into(), "input".into(), Some("din".into())), (0, "CK".into(), "input".into(), Some("clk".into()))],
            libs: vec![LibGroups {
                name: "seq_demo".into(),
                operating_conditions: vec!["tt".into()],
                wire_loads: vec!["small".into()],
                wire_load_selections: vec!["sel".into()],
            }],
            lib_pins: vec![("DFF".into(), "D".into(), 0.0021)],
        }
    }

    /// `load_of lib/cell/pin` (a synthesis tool's query; neither SDC nor the reference declares
    /// it) is the library pin's capacitance, in the library's unit as `set_load` takes it, when
    /// the libraries are given; a library, cell or pin they do not have, or no library, refuses
    /// the command with the reason. Variables in its argument are substituted first.
    #[test]
    fn load_of_is_the_library_pins_capacitance() {
        let d = design();
        let s = Sdc::parse_with_opts(
            "set_load [load_of seq_demo/DFF/D] [get_ports din]\nset_load [load_of ${L}/DFF/D] [get_ports clk]\n",
            &d,
            &ReadOpts { defines: vec![("L".into(), "seq_demo".into())], base_dir: None },
        )
        .unwrap();
        let got: Vec<(f64, Vec<String>)> = s.env.iter().map(|e| (e.value, e.objects.clone())).collect();
        assert_eq!(got, [(0.0021, vec!["din".to_string()]), (0.0021, vec!["clk".to_string()])]);
        for (line, why) in [
            ("set_load [load_of other/DFF/D] [get_ports din]", "no library `other` was read"),
            ("set_load [load_of seq_demo/DFF/Q] [get_ports din]", "library `seq_demo` has no pin DFF/Q"),
            ("set_load [load_of DFF/D] [get_ports din]", "`DFF/D` is not lib/cell/pin"),
        ] {
            let s = Sdc::parse_with(line, &d).unwrap();
            assert!(s.env.is_empty(), "{line}");
            assert!(s.unevaluated_queries.iter().any(|q| q.contains(why)), "{line}: {:?}", s.unevaluated_queries);
        }
        let s = Sdc::parse_with("set_load [load_of seq_demo/DFF/D] [get_ports din]", &SdcDesign::default()).unwrap();
        assert!(s.unevaluated_queries.iter().any(|q| q.contains("not evaluated: no library was given")), "{:?}", s.unevaluated_queries);
    }

    fn one(r: &[String], needle: &str) {
        assert!(r.len() == 1 && r[0].contains(needle), "{r:?}");
    }

    /// Rule (`parse_key_args`, the reference's `tcl/Util.tcl`): a short option is the FIRST key
    /// whose name it begins, keys tried before flags, with no ambiguity check. So `get_cells -h`
    /// is `-hsc` (a key), not `-hierarchical` (a flag): its pattern becomes the divider, which is
    /// Error 342 there and here (the pinned OpenSTA stops `read_sdc` at it; this reader refuses
    /// the command and reads on, as at every reference error). The public SDC parser instead calls
    /// `-h` ambiguous (two options begin with it): refused either way. `-hi` is `-hierarchical`.
    #[test]
    fn a_short_h_is_the_hsc_key_as_the_reference_reads_it() {
        let s = Sdc::parse(&format!("{CLK}set_false_path -to [get_cells -h *u_pad*]\n")).unwrap();
        assert!(s.exceptions.is_empty(), "{:?}", s.exceptions);
        assert_eq!(s.rejected.len(), 1, "{:?}", s.rejected);
        assert!(s.rejected[0].contains("hierarchy separator must be one of '/@^#.|' (Error 342)"), "{:?}", s.rejected);
        let s = Sdc::parse(&format!("{CLK}set_false_path -to [get_cells -hi *u_pad*]\n")).unwrap();
        assert!(!s.rejected.iter().any(|r| r.contains("Error 342")), "{:?}", s.rejected);
    }

    /// Rule (`check_path_divider`): an `-hsc` divider is one character of `/@^#.|` (Error 342).
    #[test]
    fn an_hsc_divider_outside_the_sdc_set_is_refused() {
        let s = Sdc::parse(&format!("{CLK}set_false_path -to [get_pins -hsc 1 r/D]\n")).unwrap();
        assert!(s.exceptions.is_empty() && s.rejected.iter().any(|r| r.contains("Error 342")), "{:?}", s.rejected);
        let s = Sdc::parse(&format!("{CLK}set_false_path -to [get_pins -hsc | r|D]\n")).unwrap();
        assert!(!s.rejected.iter().any(|r| r.contains("Error 342")), "{:?}", s.rejected);
    }

    /// Rule (`set_units` / `check_unit`): the value ends in the unit's suffix, case aside (501), and
    /// what precedes it is `[1 0* .? 0*][M k m u n p f]` (343).
    #[test]
    fn a_unit_without_its_suffix_or_with_an_unknown_prefix_is_refused() {
        one(&rejected("set_units -time 1"), "Error 501");
        one(&rejected("set_units -capacitance 1pX"), "Error 501");
        one(&rejected("set_units -time 2ns"), "Error 343");
        // the suffix is matched without case, the prefix is not
        one(&rejected("set_units -time 1.0NS"), "Error 343");
        for ok in ["set_units -time ns", "set_units -time 1.0ns", "set_units -time nS", "set_units -capacitance 1pF", "set_units -resistance kohm", "set_units -time 100ps"] {
            assert!(rejected(ok).is_empty(), "{ok}: {:?}", rejected(ok));
        }
    }

    /// Rule (`set_clock_latency`): `-early`/`-late` only with `-source` (Error 410).
    #[test]
    fn an_early_late_network_latency_is_refused() {
        one(&rejected("set_clock_latency -early 1 [get_clocks clk]"), "Error 410");
        assert!(rejected("set_clock_latency -source -early 1 [get_clocks clk]").is_empty());
    }

    /// Rule (`parse_early_late_flags`): `-early` and `-late` together are an error (Error 110).
    #[test]
    fn a_derate_both_early_and_late_is_refused() {
        let s = Sdc::parse(&format!("{CLK}set_timing_derate -late -early 0.9\n")).unwrap();
        assert!(s.late_derate.is_none() && s.early_derate.is_none());
        one(&s.rejected, "Error 110");
    }

    /// Rule (`set_clock_gating_check1`): with -high/-low, a clock among the objects is an error (399).
    #[test]
    fn a_high_low_gating_check_on_a_clock_is_refused() {
        one(&rejected("set_clock_gating_check -high -setup 0.1 [get_clocks clk]"), "Error 399");
        assert!(rejected("set_clock_gating_check -high -setup 0.1 [get_pins r/CK]").is_empty());
    }

    /// Rule (`parse_op_cond`, `set_wire_load_model`, `set_wire_load_selection_group`): the group is
    /// looked up by name in the `-library` library (one, found by name: Errors 113/115), else in
    /// every library; not found is 474 in a named library, 475 otherwise; 480 and 481 either way.
    #[test]
    fn a_library_group_the_libraries_lack_is_refused() {
        let d = design();
        one(&rejected_with("set_operating_conditions typical", &d), "Error 475");
        one(&rejected_with("set_operating_conditions -library seq_demo typical", &d), "Error 474");
        one(&rejected_with("set_operating_conditions -max typical", &d), "Error 475");
        one(&rejected_with("set_operating_conditions -library nolib tt", &d), "Error 115");
        one(&rejected_with("set_wire_load_model -name wl", &d), "Error 480");
        one(&rejected_with("set_wire_load_model -name wl -library [get_libs seq_demo]", &d), "Error 480");
        one(&rejected_with("set_wire_load_selection_group sg", &d), "Error 481");
        for ok in ["set_operating_conditions tt", "set_operating_conditions -library seq_demo -max tt -max_library seq_demo", "set_wire_load_model -name small -library seq_demo", "set_wire_load_selection_group sel"] {
            assert!(rejected_with(ok, &d).is_empty(), "{ok}: {:?}", rejected_with(ok, &d));
        }
        // no library read: not checked
        assert!(rejected("set_operating_conditions typical").is_empty());
    }

    /// Rule (`get_port_pin_error`): a `-reference_pin` the design lacks is an error (Error 130).
    #[test]
    fn a_reference_pin_the_design_lacks_is_refused() {
        let d = design();
        one(&rejected_with("set_input_delay -reference_pin 1 1 -clock clk [get_ports din]", &d), "Error 130");
        assert!(rejected_with("set_input_delay -reference_pin [get_pins r/CK] 1 -clock clk [get_ports din]", &d).is_empty());
    }

    /// Rule (`parse_disable_inst_ports`): on cells, `-from`/`-to` is a port NAME; an object there
    /// is never one, so the reference errors (Error 431).
    #[test]
    fn a_disable_timing_from_object_on_cells_is_refused() {
        let s = Sdc::parse(&format!("{CLK}set_disable_timing -from [get_ports din] [get_cells r]\n")).unwrap();
        assert!(s.disabled_timing.is_empty());
        one(&s.rejected, "Error 431");
        let s = Sdc::parse(&format!("{CLK}set_disable_timing -from D -to Q [get_cells r]\n")).unwrap();
        assert_eq!(s.disabled_timing.len(), 1, "{:?}", s.rejected);
    }
}

/// Tcl `break` and `continue` (Tcl 8.6, the reference's interpreter; each expected list below was
/// run in `tclsh` with `set_input_delay` as a proc that records its value): `continue` ends the
/// pass, the `for` step and test still run; `break` ends the innermost running loop; either one
/// with no loop running is Tcl's error ("invoked break outside of a loop").
#[cfg(test)]
mod break_continue_tests {
    use super::*;

    fn delays(body: &str) -> (Vec<f64>, Vec<String>) {
        let s = Sdc::parse(&format!("create_clock -name c -period 10 [get_ports clk]\n{body}\n")).unwrap();
        (s.input_delays.iter().map(|d| d.value).collect(), s.rejected)
    }

    #[test]
    fn for_continue_skips_the_rest_of_the_pass() {
        let (v, r) = delays("for {set i 0} {$i < 4} {incr i} { if {$i == 1} { continue }\n set_input_delay $i -clock c [get_ports a] }");
        assert_eq!((v, r), (vec![0.0, 2.0, 3.0], vec![]));
    }

    #[test]
    fn while_break_ends_the_loop() {
        let (v, r) = delays("set i 0\nwhile {1} { if {$i == 2} { break }\n set_input_delay $i -clock c [get_ports a]\n incr i }\nset_input_delay 9 -clock c [get_ports a]");
        assert_eq!((v, r), (vec![0.0, 1.0, 9.0], vec![]));
    }

    #[test]
    fn foreach_break_and_continue() {
        let (v, _) = delays("foreach x {1 2 3 4} { if {$x == 2} { continue }\n if {$x == 4} { break }\n set_input_delay $x -clock c [get_ports a] }");
        assert_eq!(v, vec![1.0, 3.0]);
    }

    #[test]
    fn break_ends_only_the_inner_loop() {
        let (v, _) = delays("foreach x {1 2} { foreach y {10 20 30} { if {$y == 20} { break }\n set_input_delay [expr {$x + $y}] -clock c [get_ports a] }\n set_input_delay $x -clock c [get_ports a] }");
        assert_eq!(v, vec![11.0, 1.0, 12.0, 2.0]);
    }

    #[test]
    fn break_outside_a_loop_is_an_error() {
        for w in ["break", "continue"] {
            let e = Sdc::parse(&format!("{w}\nset_input_delay 1 -clock c [get_ports a]\n")).unwrap_err();
            assert!(e.0.contains("outside of a loop"), "{e}");
        }
        // in a proc called from a loop body, the proc's frame ends the search: still an error
        assert!(Sdc::parse("proc p {} { break }\nforeach x {1 2} { p }\n").is_err());
    }
}
