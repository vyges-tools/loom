// SPDX-License-Identifier: Apache-2.0
//! SDC object queries evaluated against the design, as the reference evaluates them when it reads
//! the constraints of a linked design: `get_ports`, `get_cells`, `get_pins` and `get_nets` with
//! patterns, `-of_objects` and `-filter`.
//!
//! The rules, each as the reference has them:
//!   * no pattern is `*`; a pattern is a glob (`*`, `?`) over names — a pin's is `inst/pin`;
//!   * `-of_objects`: `get_ports` takes NETS (a clock list gives nothing, with a warning);
//!     `get_cells` pins or nets; `get_pins` nets or cells (instance pins only); `get_nets` pins,
//!     cells or ports; the patterns are then ignored;
//!   * `-filter "<expr>"`: predicates `prop op arg` (op `==` `!=` `=~` `!~`; a bare `prop` is
//!     `prop == 1`), `defined(prop)` / `undefined(prop)`, `!` over `&&` over `||`, parentheses;
//!     a bool property compares as `1`/`0` (`true`/`false` accepted). The result is sorted by name;
//!   * properties: a port's `name` `full_name` `direction`; an instance's `name` `full_name`
//!     `ref_name` `cell` `is_hierarchical`, and its library cell's `is_buffer` `is_inverter`
//!     `is_clock_gate` `is_macro` `is_memory` (false for a cell no library defines; not evaluated
//!     with no library at all), and `is_sequential`, an EXTENSION (below); a pin's `name`
//!     `lib_pin_name` `full_name` `direction` `is_port` `is_hierarchical`; a net's `name`
//!     `full_name`.
//!
//!   * `-regexp`: each pattern is a regular expression ANCHORED as the reference anchors it, a
//!     `^` before and a `$` after with no group (so `a|b` is "starts with a, or ends with b"),
//!     `-nocase` making it case-blind; a bus port or a pin is matched the way a glob is (see
//!     [`tcl_regex`]).
//!
//!   * EXTENSION (user 2026-11-08): an instance's `is_sequential` is true when its library cell
//!     holds state as this engine's registers do ([`crate::liberty::Cell::is_seq`]: an `ff` or
//!     `latch` group or bank, a `statetable`, or a clock pin on a cell that is not a clock gate).
//!     The reference has no such instance property: it errors there and stops the file.
//!
//! What this cannot answer it says so ([`Err`]) and the command is not applied: a `-regexp`
//! pattern the reference's dialect reads differently ([`tcl_regex`]), and a property the
//! reference has but this view does not carry (timing, Liberty attributes).
//!
//! A property the reference does not know at all ([`known_property`]) is the reference's error
//! when the filter is applied to at least one object: the query fails, and `read_sdc` stops at
//! that command (decided 2026-11-08: as the reference does). Its [`Err`] carries
//! [`UNKNOWN_PROPERTY`], and the reader stops on it.
//!
//! A filter the reference cannot parse or evaluate (its errors 2600-2608: a character no token
//! starts with, an extraneous `)`, an unmatched `(`, an operator short of operands, an empty
//! expression, two sets left) is the same kind of error, under the same condition (at least one
//! object), and stops the read the same way: its [`Err`] carries [`MALFORMED_FILTER`] (decided
//! 2026-11-08). [`stops_the_read`] says which errors these are.

use crate::liberty::{Dir, Lib};
use crate::netlist::Netlist;

/// The design as an SDC query sees it: flat, at the top.
#[derive(Debug, Clone, Default)]
pub struct SdcDesign {
    /// `(name, direction)` in declaration order; direction as the reference names it.
    pub ports: Vec<(String, String)>,
    /// `(instance, cell)` in netlist order.
    pub insts: Vec<(String, String)>,
    /// Instance pins: `(instance index, pin, direction, net)`.
    pub pins: Vec<(usize, String, String, Option<String>)>,
    /// The libraries read, in read order, with the group names SDC looks up. Empty when no
    /// library was given: a lookup is then not checked.
    pub libs: Vec<crate::liberty::LibGroups>,
    /// Each library cell pin's capacitance, in the library's unit: `(cell, pin, capacitance)`,
    /// the libraries merged ([`crate::liberty::Pin::capacitance`]). Empty with no library. What
    /// `[load_of lib/cell/pin]` reads.
    pub lib_pins: Vec<(String, String, f64)>,
    /// `--rtl`: the design is an RTL elaboration, and an object pattern is read as the RTL names
    /// it (a declared bit index, an RTL form of a register name), by the rules `sdc map` uses
    /// ([`crate::sdc_rtl_map::RtlNaming`]). `None`: every pattern as written.
    pub rtl: Option<crate::sdc_rtl_map::RtlNaming>,
    /// The instance booleans the reference reads from an instance's library cell, by cell name,
    /// for the cells the libraries define. `None`: no library was given, so they are not known.
    pub cell_bools: Option<std::collections::BTreeMap<String, CellBools>>,
}

/// A library cell's booleans as an instance property reads them (`Properties::getProperty` of an
/// instance: `liberty_cell && liberty_cell->isBuffer()` and so on).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CellBools {
    pub is_buffer: bool,
    pub is_inverter: bool,
    pub is_clock_gate: bool,
    pub is_macro: bool,
    pub is_memory: bool,
    /// EXTENSION, not the reference's: the cell holds state ([`crate::liberty::Cell::is_seq`]).
    pub is_sequential: bool,
}

impl CellBools {
    pub fn of(c: &crate::liberty::Cell) -> CellBools {
        CellBools {
            is_buffer: c.is_buffer(),
            is_inverter: c.is_inverter(),
            is_clock_gate: c.is_clock_gate(),
            is_macro: c.is_macro,
            is_memory: c.is_memory,
            is_sequential: c.is_seq,
        }
    }
}

/// Marks the [`Err`] of a filter naming a property the reference does not know for its objects;
/// the reader stops the file on it ([`crate::sdc::Sdc::parse_with`]).
pub(crate) const UNKNOWN_PROPERTY: &str = "a property the reference timer does not know";

/// Marks the [`Err`] of a filter the reference cannot parse or evaluate (its STA-2600..2608); the
/// reader stops the file on it, as on [`UNKNOWN_PROPERTY`].
pub(crate) const MALFORMED_FILTER: &str = "a -filter the reference timer cannot read";

/// Whether an [`eval`] error is one the reference raises as an error (so its `read_sdc` stops
/// there), not a query this view cannot evaluate.
pub(crate) fn stops_the_read(why: &str) -> bool {
    why.contains(UNKNOWN_PROPERTY) || why.contains(MALFORMED_FILTER)
}

/// The reference's filter error `id` with its text.
fn malformed(id: u32, text: &str) -> String {
    format!("{text} (STA-{id}, {MALFORMED_FILTER})")
}

/// Whether the reference knows `prop` for the objects of `o` (`Properties::getProperty` per
/// object type, at the pin, plus what OpenROAD registers: `src` on instances and pins), and the
/// extension `is_sequential` on instances. Known is not evaluated: see [`property`].
fn known_property(o: &Objs, prop: &str) -> bool {
    const PORT: &[&str] = &["name", "full_name", "direction", "port_direction", "liberty_port", "activity"];
    const INST: &[&str] = &[
        "name", "full_name", "ref_name", "liberty_cell", "cell", "is_hierarchical", "is_buffer", "is_clock_gate",
        "is_inverter", "is_macro", "is_memory", "src",
    ];
    const PIN: &[&str] = &[
        "name", "lib_pin_name", "full_name", "direction", "pin_direction", "is_hierarchical", "is_port", "is_clock",
        "is_register_clock", "clocks", "clock_domains", "activity", "arrival_max_rise", "arrival_max_fall",
        "arrival_min_rise", "arrival_min_fall", "src",
    ];
    const TIMING: &[&str] = &["max", "max_fall", "max_rise", "min", "min_fall", "min_rise"];
    let timed = |p: &str| ["slack_", "slew_"].iter().any(|k| p.strip_prefix(k).is_some_and(|t| TIMING.contains(&t)));
    match o {
        Objs::Ports(_) => PORT.contains(&prop) || timed(prop),
        // the extension (module doc)
        Objs::Cells(_) => INST.contains(&prop) || prop == "is_sequential",
        Objs::Pins(_) => PIN.contains(&prop) || timed(prop),
        Objs::Nets(_) => matches!(prop, "name" | "full_name"),
        Objs::Clocks => true,
    }
}

fn unknown_property(o: &Objs, prop: &str) -> String {
    format!("-filter: `{prop}` is not a property of {} ({UNKNOWN_PROPERTY})", kind(o))
}

fn dir_name(d: Dir, tristate: bool) -> &'static str {
    match d {
        Dir::In => "input",
        Dir::Out if tristate => "tristate",
        Dir::Out => "output",
        Dir::Inout => "bidirect",
        Dir::Other => "internal",
    }
}

impl SdcDesign {
    /// From a netlist's top and the libraries that define its cells (pin directions).
    pub fn from_netlist(n: &Netlist, lib: Option<&Lib>) -> SdcDesign {
        let mut ports: Vec<(String, String)> = Vec::new();
        ports.extend(n.inputs.iter().map(|p| (p.clone(), "input".to_string())));
        ports.extend(n.outputs.iter().map(|p| (p.clone(), "output".to_string())));
        ports.extend(n.inouts.iter().map(|p| (p.clone(), "bidirect".to_string())));
        let mut insts = Vec::with_capacity(n.insts.len());
        let mut pins = Vec::new();
        for (k, i) in n.insts.iter().enumerate() {
            insts.push((i.name.clone(), i.cell.clone()));
            let cell = lib.and_then(|l| l.cells.get(&i.cell));
            let dir = |p: &str| {
                cell.and_then(|c| c.pins.get(p)).map_or("unknown", |q| dir_name(q.direction, q.three_state.is_some())).to_string()
            };
            for (p, net) in &i.conns {
                pins.push((k, p.clone(), dir(p), Some(net.clone())));
            }
            for (p, _) in &i.ties {
                pins.push((k, p.clone(), dir(p), None));
            }
        }
        let mut lib_pins = Vec::new();
        for (cell, c) in lib.map(|l| &l.cells).into_iter().flatten() {
            lib_pins.extend(c.pins.iter().map(|(p, q)| (cell.clone(), p.clone(), q.capacitance)));
        }
        let cell_bools = lib.map(|l| {
            let mut m = std::collections::BTreeMap::new();
            for i in &n.insts {
                if let Some(c) = l.cells.get(&i.cell) {
                    m.entry(i.cell.clone()).or_insert_with(|| CellBools::of(c));
                }
            }
            m
        });
        SdcDesign { ports, insts, pins, libs: lib.map(|l| l.libraries.clone()).unwrap_or_default(), lib_pins, rtl: None, cell_bools }
    }

    /// `load_of lib/cell/pin`: the library pin's capacitance, in the library's unit. `Err` says
    /// why it has none: no library given, a library, cell or pin the libraries do not have.
    pub(crate) fn load_of(&self, arg: &str) -> Result<f64, String> {
        if self.libs.is_empty() {
            return Err("no library was given".into());
        }
        let parts: Vec<&str> = arg.rsplitn(3, '/').collect();
        let [pin, cell, lib] = parts.as_slice() else {
            return Err(format!("`{arg}` is not lib/cell/pin"));
        };
        if !self.libs.iter().any(|l| l.name == *lib) {
            return Err(format!("no library `{lib}` was read"));
        }
        self.lib_pins
            .iter()
            .find(|(c, p, _)| c == cell && p == pin)
            .map(|(_, _, cap)| *cap)
            .ok_or_else(|| format!("library `{lib}` has no pin {cell}/{pin}"))
    }

    /// One object pattern as the design reads it: under `rtl`, the RTL reading's names when it
    /// differs from the written one ([`crate::sdc_rtl_map::RtlNaming::rewrite`]); else as written.
    pub(crate) fn read_pattern(&self, accessor: &str, pattern: &str) -> Vec<String> {
        self.rtl.as_ref().and_then(|r| r.rewrite(accessor, pattern)).unwrap_or_else(|| vec![pattern.to_string()])
    }

    /// `name` is one of the design's ports or instance pins.
    pub(crate) fn has_port_or_pin(&self, name: &str) -> bool {
        self.ports.iter().any(|(p, _)| p == name) || (0..self.pins.len()).any(|k| self.pin_name(k) == name)
    }

    fn pin_name(&self, k: usize) -> String {
        let (i, p, _, _) = &self.pins[k];
        format!("{}/{}", self.insts[*i].0, p)
    }

    fn nets(&self) -> Vec<String> {
        let mut v: Vec<String> = self.ports.iter().map(|p| p.0.clone()).collect();
        v.extend(self.pins.iter().filter_map(|p| p.3.clone()));
        let mut seen = std::collections::BTreeSet::new();
        v.retain(|n| seen.insert(n.clone()));
        v
    }
}

/// What a query returns: objects of one kind, by name.
#[derive(Debug, Clone, PartialEq)]
enum Objs {
    Ports(Vec<usize>),
    Cells(Vec<usize>),
    Pins(Vec<usize>),
    Nets(Vec<String>),
    Clocks,
}

/// A glob as the reference matches one: `*` any run, `?` one character.
pub(crate) fn glob(pat: &str, s: &str) -> bool {
    let (p, t): (Vec<char>, Vec<char>) = (pat.chars().collect(), s.chars().collect());
    let (mut i, mut j, mut star, mut mark) = (0, 0, None, 0);
    while j < t.len() {
        if i < p.len() && (p[i] == '?' || p[i] == t[j]) {
            i += 1;
            j += 1;
        } else if i < p.len() && p[i] == '*' {
            star = Some(i);
            mark = j;
            i += 1;
        } else if let Some(si) = star {
            i = si + 1;
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

/// Evaluate a bracketed query's words (`get_ports -filter … *`, abbreviations expanded) to the
/// names of the objects it selects, or why it cannot be evaluated here.
pub(crate) fn eval(parts: &[String], d: &SdcDesign) -> Result<Vec<String>, String> {
    Ok(names(&objects(parts, d)?, d))
}

fn names(o: &Objs, d: &SdcDesign) -> Vec<String> {
    match o {
        Objs::Ports(v) => v.iter().map(|&k| d.ports[k].0.clone()).collect(),
        Objs::Cells(v) => v.iter().map(|&k| d.insts[k].0.clone()).collect(),
        Objs::Pins(v) => v.iter().map(|&k| d.pin_name(k)).collect(),
        Objs::Nets(v) => v.clone(),
        Objs::Clocks => Vec::new(),
    }
}

fn objects(parts: &[String], d: &SdcDesign) -> Result<Objs, String> {
    let cmd = parts.first().map(String::as_str).unwrap_or("");
    let keys = crate::sdc_flags::keys(cmd);
    let (mut filter, mut of, mut regexp, mut hier, mut nocase) = (None, None, false, false, false);
    let mut patterns: Vec<String> = Vec::new();
    let mut k = 1;
    while k < parts.len() {
        let w = parts[k].as_str();
        if keys.contains(&w) {
            let v = parts.get(k + 1).cloned().unwrap_or_default();
            match w {
                "-filter" => filter = Some(v),
                "-of_objects" => of = Some(v),
                _ => {} // -hsc: the divider; names here are flat
            }
            k += 2;
            continue;
        }
        match w {
            "-regexp" => regexp = true,
            "-nocase" => nocase = true,
            "-hierarchical" => hier = true,
            w if w.starts_with('-') => {}
            w => patterns.extend(w.split_whitespace().map(String::from)),
        }
        k += 1;
    }
    if regexp && cmd == "get_clocks" {
        return Err("get_clocks -regexp is not evaluated".into());
    }
    if cmd == "get_clocks" {
        return if filter.is_some() || of.is_some() { Err("get_clocks with -filter / -of_objects is not evaluated".into()) } else { Ok(Objs::Clocks) };
    }
    if patterns.is_empty() {
        patterns.push("*".into());
    }
    if !regexp {
        patterns = patterns.iter().flat_map(|p| d.read_pattern(cmd, p)).collect();
    }
    let base = match of {
        Some(of) => of_objects(cmd, &of, d)?,
        None if regexp => by_regexp(cmd, &patterns, nocase, d)?,
        None => by_pattern(cmd, &patterns, hier, d)?,
    };
    match filter {
        Some(f) => apply_filter(&f, base, d),
        None => Ok(base),
    }
}

fn by_pattern(cmd: &str, patterns: &[String], _hier: bool, d: &SdcDesign) -> Result<Objs, String> {
    let pick = |n: usize, name: &dyn Fn(usize) -> String, hit: &dyn Fn(&str, &str) -> bool| -> Vec<usize> {
        let mut out = Vec::new();
        for p in patterns {
            for k in 0..n {
                if hit(p, &name(k)) && !out.contains(&k) {
                    out.push(k);
                }
            }
        }
        out
    };
    Ok(match cmd {
        // ports are the netlist's bits; a pattern naming a bus names each of them
        "get_ports" => Objs::Ports(pick(d.ports.len(), &|k| d.ports[k].0.clone(), &crate::sdc::object_matches)),
        // `SdcNetwork::findInstancesMatching`: dividers escaped, brackets escaped when nothing
        // matches as written
        "get_cells" => {
            let insts: Vec<&str> = d.insts.iter().map(|i| i.0.as_str()).collect();
            let mut out: Vec<usize> = Vec::new();
            for p in patterns {
                for k in crate::sdc::insts_matching(p, &insts) {
                    if !out.contains(&k) {
                        out.push(k);
                    }
                }
            }
            Objs::Cells(out)
        }
        // `SdcNetwork::findPinsMatching`: the instance part with its brackets escaped when
        // nothing matches it as written (the database keeps `\[`, written SDC does not)
        "get_pins" => {
            let insts: Vec<&str> = d.insts.iter().map(|i| i.0.as_str()).collect();
            let pins: Vec<(usize, &str)> = d.pins.iter().map(|p| (p.0, p.1.as_str())).collect();
            let mut out: Vec<usize> = Vec::new();
            for p in patterns {
                for k in crate::sdc::pins_matching(p, &insts, &pins) {
                    if !out.contains(&k) {
                        out.push(k);
                    }
                }
            }
            Objs::Pins(out)
        }
        "get_nets" => {
            let all = d.nets();
            let mut out: Vec<String> = Vec::new();
            for p in patterns {
                out.extend(all.iter().filter(|n| glob(p, n) && !out.contains(n)).cloned().collect::<Vec<_>>());
            }
            Objs::Nets(out)
        }
        c => return Err(format!("{c} is not evaluated")),
    })
}

/// A Tcl regular expression (Advanced RE, as the reference compiles one) as a Rust [`Regex`],
/// `anchored` as `PatternMatch::compileRegexp` anchors it: `^` + pattern + `$`, NOT grouped.
///
/// The two dialects agree on what object patterns use (classes, `\d` `\w` `\s`, `.`, `*` `+` `?`,
/// bounds, alternation, groups). Where they differ, Rust either fails to compile (back-references,
/// lookahead, `\m` `\M` `\y` `\Y` `\e`, `[[:<:]]`) — refused — or reads it differently: `\b` is
/// a backspace and `\B` a backslash in Tcl, word boundaries in Rust, so those are refused too.
///
/// [`Regex`]: regex::Regex
pub(crate) fn tcl_regex(pat: &str, anchored: bool, nocase: bool) -> Result<regex::Regex, String> {
    let mut chars = pat.chars();
    while let Some(c) = chars.next() {
        if c == '\\' && matches!(chars.next(), Some('b' | 'B')) {
            return Err(format!("-regexp `{pat}`: `\\b` / `\\B` mean a backspace / backslash to the reference, a word boundary here; not evaluated"));
        }
    }
    let full = if anchored { format!("^{pat}$") } else { pat.to_string() };
    regex::RegexBuilder::new(&full)
        .case_insensitive(nocase)
        .build()
        .map_err(|e| format!("-regexp `{pat}`: not a pattern evaluated here as the reference evaluates it ({})", e.to_string().lines().last().unwrap_or("")))
}

/// `-regexp` patterns over names, as the reference's finders apply them: a port pattern through
/// the bus rule ([`crate::sdc::object_matches_by`]), a pin pattern split at its LAST `/` into an
/// instance pattern and a pin pattern (`pathNameLast`), each matched whole; cells and nets whole.
fn by_regexp(cmd: &str, patterns: &[String], nocase: bool, d: &SdcDesign) -> Result<Objs, String> {
    let mut compiled: std::collections::HashMap<String, regex::Regex> = std::collections::HashMap::new();
    let mut need = |p: &str| -> Result<(), String> {
        if !compiled.contains_key(p) {
            compiled.insert(p.to_string(), tcl_regex(p, true, nocase)?);
        }
        Ok(())
    };
    let mut split_pins = Vec::new();
    for p in patterns {
        match cmd {
            "get_ports" => {
                need(p)?;
                if let Some((base, _)) = crate::sdc::bus_form(p) {
                    need(base)?;
                }
            }
            "get_pins" => {
                let Some((inst, pin)) = p.rsplit_once('/') else {
                    return Err(format!("get_pins -regexp `{p}`: a pattern with no instance part is not evaluated"));
                };
                need(inst)?;
                need(pin)?;
                split_pins.push((inst.to_string(), pin.to_string()));
            }
            "get_cells" | "get_nets" => need(p)?,
            c => return Err(format!("{c} -regexp is not evaluated")),
        }
    }
    let is = |p: &str, n: &str| compiled.get(p).is_some_and(|r| r.is_match(n));
    let pick = |n: usize, name: &dyn Fn(usize) -> String, hit: &dyn Fn(&str, &str) -> bool| -> Vec<usize> {
        let mut out = Vec::new();
        for p in patterns {
            for k in 0..n {
                if hit(p, &name(k)) && !out.contains(&k) {
                    out.push(k);
                }
            }
        }
        out
    };
    Ok(match cmd {
        "get_ports" => Objs::Ports(pick(d.ports.len(), &|k| d.ports[k].0.clone(), &|p, n| crate::sdc::object_matches_by(p, n, &is))),
        "get_cells" => Objs::Cells(pick(d.insts.len(), &|k| d.insts[k].0.clone(), &is)),
        "get_pins" => {
            let mut out = Vec::new();
            for (ip, pp) in &split_pins {
                for k in 0..d.pins.len() {
                    if is(ip, &d.insts[d.pins[k].0].0) && is(pp, &d.pins[k].1) && !out.contains(&k) {
                        out.push(k);
                    }
                }
            }
            Objs::Pins(out)
        }
        _ => {
            let all = d.nets();
            let mut out: Vec<String> = Vec::new();
            for p in patterns {
                out.extend(all.iter().filter(|n| is(p, n) && !out.contains(n)).cloned().collect::<Vec<_>>());
            }
            Objs::Nets(out)
        }
    })
}

/// The objects an `-of_objects` value names: a nested query, or bare names of the kind the outer
/// command takes by default (nets for get_ports / get_pins / get_cells, pins for get_nets).
fn of_objects(cmd: &str, value: &str, d: &SdcDesign) -> Result<Objs, String> {
    let v = value.trim();
    let inner = if let Some(q) = v.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        objects(&crate::sdc::query_words(q), d)?
    } else {
        let words: Vec<String> = v.split_whitespace().map(String::from).collect();
        match cmd {
            "get_nets" => by_pattern("get_pins", &words, false, d)?,
            _ => Objs::Nets(words),
        }
    };
    let net_of_pin = |k: usize| d.pins[k].3.clone();
    Ok(match (cmd, inner) {
        ("get_ports", Objs::Nets(ns)) => Objs::Ports(ns.iter().filter_map(|n| d.ports.iter().position(|p| &p.0 == n)).collect()),
        ("get_ports", Objs::Clocks) => Objs::Ports(Vec::new()), // the reference: unsupported object type, nothing
        ("get_cells", Objs::Pins(ps)) => Objs::Cells(dedup(ps.iter().map(|&k| d.pins[k].0))),
        ("get_cells", Objs::Nets(ns)) => Objs::Cells(dedup(d.pins.iter().filter(|p| p.3.as_ref().is_some_and(|n| ns.contains(n))).map(|p| p.0))),
        ("get_pins", Objs::Nets(ns)) => Objs::Pins((0..d.pins.len()).filter(|&k| net_of_pin(k).is_some_and(|n| ns.contains(&n))).collect()),
        ("get_pins", Objs::Cells(cs)) => Objs::Pins((0..d.pins.len()).filter(|&k| cs.contains(&d.pins[k].0)).collect()),
        ("get_nets", Objs::Pins(ps)) => Objs::Nets(dedup_s(ps.iter().filter_map(|&k| net_of_pin(k)))),
        ("get_nets", Objs::Cells(cs)) => Objs::Nets(dedup_s((0..d.pins.len()).filter(|&k| cs.contains(&d.pins[k].0)).filter_map(net_of_pin))),
        ("get_nets", Objs::Ports(ps)) => Objs::Nets(ps.iter().map(|&k| d.ports[k].0.clone()).collect()),
        (c, o) => return Err(format!("{c} -of_objects of {} is not evaluated", kind(&o))),
    })
}

fn kind(o: &Objs) -> &'static str {
    match o {
        Objs::Ports(_) => "ports",
        Objs::Cells(_) => "cells",
        Objs::Pins(_) => "pins",
        Objs::Nets(_) => "nets",
        Objs::Clocks => "clocks",
    }
}

fn dedup(it: impl Iterator<Item = usize>) -> Vec<usize> {
    let mut v: Vec<usize> = Vec::new();
    for x in it {
        if !v.contains(&x) {
            v.push(x);
        }
    }
    v
}

fn dedup_s(it: impl Iterator<Item = String>) -> Vec<String> {
    let mut v: Vec<String> = Vec::new();
    for x in it {
        if !v.contains(&x) {
            v.push(x);
        }
    }
    v
}

/// A property's value as the filter compares it, and whether it is a bool. `Err` for a property
/// the reference has but this view does not carry (not evaluated); one the reference does not
/// know was stopped before this ([`known_property`]).
fn property(o: &Objs, k: usize, prop: &str, d: &SdcDesign) -> Result<(String, bool), String> {
    let s = |v: &str| Ok((v.to_string(), false));
    let b = |v: bool| Ok((if v { "1" } else { "0" }.to_string(), true));
    match o {
        Objs::Ports(v) => {
            let (name, dir) = &d.ports[v[k]];
            match prop {
                "name" | "full_name" => s(name),
                "direction" | "port_direction" => s(dir),
                p => Err(format!("port property {p} is not evaluated")),
            }
        }
        Objs::Cells(v) => {
            let (name, cell) = &d.insts[v[k]];
            match prop {
                "name" | "full_name" => s(name),
                "ref_name" | "cell" => s(cell),
                "is_hierarchical" => b(false),
                "is_buffer" | "is_inverter" | "is_clock_gate" | "is_macro" | "is_memory" | "is_sequential" => {
                    let Some(m) = &d.cell_bools else {
                        return Err(format!("instance property {prop} is not evaluated: no library was given"));
                    };
                    // a cell no library defines has no liberty cell: every one of these is false
                    let c = m.get(cell).copied().unwrap_or_default();
                    b(match prop {
                        "is_buffer" => c.is_buffer,
                        "is_inverter" => c.is_inverter,
                        "is_clock_gate" => c.is_clock_gate,
                        "is_macro" => c.is_macro,
                        // EXTENSION (module doc): the reference errors on it
                        "is_sequential" => c.is_sequential,
                        _ => c.is_memory,
                    })
                }
                p => Err(format!("instance property {p} is not evaluated")),
            }
        }
        Objs::Pins(v) => {
            let (_, pin, dir, _) = &d.pins[v[k]];
            match prop {
                "name" | "lib_pin_name" => s(pin),
                "full_name" => s(&d.pin_name(v[k])),
                "direction" | "pin_direction" => s(dir),
                "is_port" | "is_hierarchical" => b(false),
                p => Err(format!("pin property {p} is not evaluated")),
            }
        }
        Objs::Nets(v) => match prop {
            "name" | "full_name" => s(&v[k]),
            p => Err(format!("net property {p} is not evaluated")),
        },
        Objs::Clocks => Err("clock properties are not evaluated".into()),
    }
}

#[derive(Debug, Clone, PartialEq)]
enum FTok {
    Pred(String, String, String),
    Defined(String, bool),
    And,
    Or,
    Not,
    LParen,
    RParen,
}

/// The reference's filter lexer: at each point the first of its token patterns that matches.
fn lex(expr: &str) -> Result<Vec<FTok>, String> {
    let c: Vec<char> = expr.chars().collect();
    let ident = |i: usize| -> usize {
        let mut j = i;
        while j < c.len() && (c[j].is_ascii_alphabetic() || c[j] == '_') {
            j += 1;
        }
        j
    };
    let argc = |ch: char| ch.is_ascii_alphanumeric() || "_/$[]*?.".contains(ch);
    let mut out = Vec::new();
    let mut i = 0;
    while i < c.len() {
        if c[i].is_whitespace() {
            i += 1;
            continue;
        }
        let rest: String = c[i..].iter().collect();
        let mut matched = false;
        for (word, def) in [("defined(", true), ("undefined(", false)] {
            if rest.starts_with(word) {
                let s = i + word.len();
                let e = ident(s);
                if e > s && c.get(e) == Some(&')') {
                    out.push(FTok::Defined(c[s..e].iter().collect(), def));
                    i = e + 1;
                    matched = true;
                }
                break;
            }
        }
        if matched {
            continue;
        }
        // `@?prop( *op *arg)?`
        let s = if c[i] == '@' { i + 1 } else { i };
        let e = ident(s);
        if e > s {
            let prop: String = c[s..e].iter().collect();
            let mut j = e;
            while j < c.len() && c[j] == ' ' {
                j += 1;
            }
            let op: String = c[j..(j + 2).min(c.len())].iter().collect();
            if ["==", "!=", "=~", "!~"].contains(&op.as_str()) {
                let mut a = j + 2;
                while a < c.len() && c[a] == ' ' {
                    a += 1;
                }
                let mut z = a;
                while z < c.len() && argc(c[z]) {
                    z += 1;
                }
                if z > a {
                    out.push(FTok::Pred(prop, op, c[a..z].iter().collect()));
                    i = z;
                    continue;
                }
            }
            out.push(FTok::Pred(prop, "==".into(), "1".into()));
            i = e;
            continue;
        }
        let two: String = c[i..(i + 2).min(c.len())].iter().collect();
        let (t, n) = match (two.as_str(), c[i]) {
            ("&&", _) => (FTok::And, 2),
            ("||", _) => (FTok::Or, 2),
            (_, '!') => (FTok::Not, 1),
            (_, '(') => (FTok::LParen, 1),
            (_, ')') => (FTok::RParen, 1),
            _ => return Err(malformed(2600, &format!("-filter parsing failed at '{rest}'."))),
        };
        out.push(t);
        i += n;
    }
    Ok(out)
}

/// The reference's shunting-yard: precedence `!` over `&&` over `||`; `defined`/`undefined` and
/// `!` go on the operator stack. Its errors as it raises them: a `)` with the operator stack empty
/// is 2601, one that empties it before a `(` is found 2602, a `(` left at the end 2603.
fn postfix(toks: Vec<FTok>) -> Result<Vec<FTok>, String> {
    let rank = |t: &FTok| match t {
        FTok::Or => 4,
        FTok::And => 5,
        FTok::Not => 6,
        FTok::Defined(_, true) => 7,
        FTok::Defined(_, false) => 8,
        FTok::LParen => 2,
        _ => 0,
    };
    let (mut out, mut ops): (Vec<FTok>, Vec<FTok>) = (Vec::new(), Vec::new());
    for t in toks {
        match t {
            FTok::Pred(..) => out.push(t),
            FTok::Or | FTok::And => {
                while ops.last().is_some_and(|top| rank(top) > rank(&t)) {
                    out.push(ops.pop().unwrap());
                }
                ops.push(t);
            }
            FTok::Not | FTok::Defined(..) | FTok::LParen => ops.push(t),
            FTok::RParen => {
                if ops.is_empty() {
                    return Err(malformed(2601, "-filter extraneous )."));
                }
                while let Some(top) = ops.pop_if(|top| *top != FTok::LParen) {
                    out.push(top);
                    if ops.is_empty() {
                        return Err(malformed(2602, "-filter extraneous )."));
                    }
                }
                ops.pop(); // the `(`
            }
        }
    }
    while let Some(t) = ops.pop() {
        if t == FTok::LParen {
            return Err(malformed(2603, "-filter unmatched (."));
        }
        out.push(t);
    }
    Ok(out)
}

/// The errors the reference raises while it evaluates the postfix over at least one object, in
/// its order (`filterObjects`): each predicate and `defined()` looks its property up on the
/// objects, and one the object type lacks throws ([`known_property`]); `||` with fewer than two
/// sets is 2604, `&&` 2605, `!` with none 2606; at the end no set is 2607, more than one 2608.
/// Walked as the stack's depth, before evaluating, so an unknown property is found even after one
/// this view cannot evaluate (which the reference evaluates fine).
fn reference_errors(toks: &[FTok], base: &Objs) -> Result<(), String> {
    let mut depth = 0usize;
    for t in toks {
        match t {
            FTok::Pred(prop, ..) | FTok::Defined(prop, _) => {
                if !known_property(base, prop) {
                    return Err(unknown_property(base, prop));
                }
                depth += 1;
            }
            FTok::Or | FTok::And if depth < 2 => {
                return Err(if *t == FTok::Or {
                    malformed(2604, "-filter logical OR requires at least two operands.")
                } else {
                    malformed(2605, "-filter logical AND requires two operands.")
                });
            }
            FTok::Or | FTok::And => depth -= 1,
            FTok::Not if depth == 0 => return Err(malformed(2606, "-filter NOT missing operand.")),
            FTok::Not | FTok::LParen | FTok::RParen => {}
        }
    }
    match depth {
        0 => Err(malformed(2607, "-filter expression is empty.")),
        1 => Ok(()),
        _ => Err(malformed(2608, "-filter expression evaluated to multiple sets.")),
    }
}

fn apply_filter(expr: &str, base: Objs, d: &SdcDesign) -> Result<Objs, String> {
    use std::collections::BTreeSet;
    let n = match &base {
        Objs::Ports(v) | Objs::Cells(v) | Objs::Pins(v) => v.len(),
        Objs::Nets(v) => v.len(),
        Objs::Clocks => 0,
    };
    // Rule (SWIG `tclListSeqPtr` gives a null list for an empty one, and `filterObjects` runs
    // only `if (objects)`): over NO object the filter is not even parsed, so neither a malformed
    // filter nor an unknown property is an error there.
    if n == 0 {
        return Ok(base);
    }
    let all: BTreeSet<usize> = (0..n).collect();
    let mut stack: Vec<BTreeSet<usize>> = Vec::new();
    let toks = postfix(lex(expr)?)?;
    reference_errors(&toks, &base)?;
    for t in toks {
        match t {
            FTok::Pred(prop, op, mut arg) => {
                let mut set = BTreeSet::new();
                for k in 0..n {
                    let (val, is_bool) = property(&base, k, &prop, d)?;
                    if is_bool {
                        if arg.eq_ignore_ascii_case("true") {
                            arg = "1".into();
                        } else if arg.eq_ignore_ascii_case("false") {
                            arg = "0".into();
                        }
                    }
                    let hit = match op.as_str() {
                        "==" => val == arg,
                        "!=" => val != arg,
                        "=~" => glob(&arg, &val),
                        _ => !glob(&arg, &val),
                    };
                    if hit {
                        set.insert(k);
                    }
                }
                stack.push(set);
            }
            FTok::Defined(prop, want) => {
                let mut set = BTreeSet::new();
                for k in 0..n {
                    let (val, is_bool) = property(&base, k, &prop, d)?;
                    let defined = if is_bool { val == "1" } else { !val.is_empty() };
                    if defined == want {
                        set.insert(k);
                    }
                }
                stack.push(set);
            }
            FTok::And | FTok::Or => {
                let (Some(a), Some(b)) = (stack.pop(), stack.pop()) else { return Err("-filter: operands checked by reference_errors".into()) };
                stack.push(if t == FTok::And { a.intersection(&b).copied().collect() } else { a.union(&b).copied().collect() });
            }
            FTok::Not => {
                let Some(a) = stack.pop() else { return Err("-filter: operand checked by reference_errors".into()) };
                stack.push(all.difference(&a).copied().collect());
            }
            FTok::LParen | FTok::RParen => {}
        }
    }
    if stack.len() != 1 {
        return Err("-filter: one set left, checked by reference_errors".into());
    }
    let keep = stack.pop().unwrap();
    // Sorted by name, as the reference returns a filtered list.
    let pick = |v: &Vec<usize>| -> Vec<usize> { v.iter().enumerate().filter(|(i, _)| keep.contains(i)).map(|(_, &k)| k).collect() };
    let mut out = match &base {
        Objs::Ports(v) => Objs::Ports(pick(v)),
        Objs::Cells(v) => Objs::Cells(pick(v)),
        Objs::Pins(v) => Objs::Pins(pick(v)),
        Objs::Nets(v) => Objs::Nets(v.iter().enumerate().filter(|(i, _)| keep.contains(i)).map(|(_, s)| s.clone()).collect()),
        Objs::Clocks => Objs::Clocks,
    };
    match &mut out {
        Objs::Ports(v) => v.sort_by(|&a, &b| d.ports[a].0.cmp(&d.ports[b].0)),
        Objs::Cells(v) => v.sort_by(|&a, &b| d.insts[a].0.cmp(&d.insts[b].0)),
        Objs::Pins(v) => v.sort_by_key(|&a| d.pin_name(a)),
        Objs::Nets(v) => v.sort(),
        Objs::Clocks => {}
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIB: &str = r#"library(s) {
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
}
"#;
    const NET: &str = r#"module top (a, b, s, en, y1, y2, y3, y4, y5, y6);
  input a, b, s, en; output y1, y2, y3, y4, y5, y6;
  AND2 u1 (.A(a), .B(b), .Y(y1));
  NOR2 u2 (.A(a), .B(b), .Y(y2));
  MUX2 u3 (.S(s), .A(a), .B(b), .Y(y3));
  XOR2 u4 (.A(a), .B(b), .Y(y4));
  TBUF u5 (.A(a), .EN(en), .Y(y5));
endmodule
"#;

    fn design() -> SdcDesign {
        let lib = Lib::parse(LIB).unwrap();
        SdcDesign::from_netlist(&crate::netlist::parse(NET).unwrap(), Some(&lib))
    }

    fn q(s: &str) -> Result<Vec<String>, String> {
        eval(&crate::sdc::query_words(s), &design())
    }

    fn ok(s: &str) -> String {
        q(s).unwrap().join(" ")
    }

    /// Every answer here is the reference's on this design and library (one query each, after
    /// `link_design top` and `create_clock -name c -period 10 [get_ports s]`).
    #[test]
    fn queries_answer_as_the_reference_does() {
        assert_eq!(ok("get_ports -filter \"direction == input && name !~ e*\" *"), "a b s");
        assert_eq!(ok("get_ports -filter \"direction == output\""), "y1 y2 y3 y4 y5 y6");
        assert_eq!(ok("get_ports -of_objects [get_clocks]"), "");
        assert_eq!(ok("get_ports -of_objects [get_nets a]"), "a");
        assert_eq!(ok("get_ports -filter \"!(direction == input) || name == a\" *"), "a y1 y2 y3 y4 y5 y6");
        assert_eq!(ok("get_ports -filter \"direction == input && ! name =~ ?\" *"), "en");
        assert_eq!(ok("get_cells -filter \"ref_name == AND2 || ref_name =~ X*\" *"), "u1 u4");
        assert_eq!(ok("get_nets -of_objects [get_pins u1/Y]"), "y1");
        assert_eq!(ok("get_nets -filter \"name =~ y*\" *"), "y1 y2 y3 y4 y5 y6");
        assert_eq!(ok("get_pins */A"), "u1/A u2/A u3/A u4/A u5/A");
        assert_eq!(ok("get_pins -hierarchical -filter \"full_name =~ u*/A\""), "u1/A u2/A u3/A u4/A u5/A");
        // a tri-state output's direction is `tristate`, so u5/Y is not an output
        assert_eq!(ok("get_pins -hierarchical -filter \"direction == output\" *"), "u1/Y u2/Y u3/Y u4/Y");
        assert_eq!(ok("get_pins -of_objects [get_cells u3]"), "u3/S u3/A u3/B u3/Y");
        assert_eq!(ok("get_pins -filter \"lib_pin_name == S\" */*"), "u3/S");
        assert_eq!(ok("get_pins *").split(" ").count(), 16);
    }

    /// Instance booleans read from the library cell (`Properties::getProperty` of an instance:
    /// `liberty_cell && liberty_cell->isBuffer()` and so on), with the bool forms the filter takes.
    /// Every answer is the reference's (box probe `filter-probe-20261108T083952Z`, Nangate45 +
    /// fakeram45_64x7: these cells mirror the ones it read): a clock buffer is a buffer, a memory
    /// group makes a memory but not a macro, an ICG is a clock gate. `is_sequential` is the
    /// EXTENSION (module doc; the reference errors on it): `Cell::is_seq`, so the flop, the latch,
    /// the clocked memory and the ICG (its `statetable`), not a buffer or gate.
    #[test]
    fn instance_booleans_answer_as_the_reference_does() {
        const L: &str = r#"library(p) {
  cell (DFF_X1) { ff (IQ, IQN) { next_state : "D"; clocked_on : "CK"; }
    pin (D) { direction : input; } pin (CK) { direction : input; clock : true; }
    pin (Q) { direction : output; function : "IQ"; } }
  cell (BUF_X1) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A"; } }
  cell (CLKBUF_X1) { pin (A) { direction : input; } pin (Z) { direction : output; function : "A"; } }
  cell (INV_X1) { pin (A) { direction : input; } pin (ZN) { direction : output; function : "!A"; } }
  cell (AND2_X1) { pin (A1) { direction : input; } pin (A2) { direction : input; }
    pin (ZN) { direction : output; function : "(A1 & A2)"; } }
  cell (CLKGATETST_X1) { clock_gating_integrated_cell : "latch_posedge_precontrol";
    statetable ("CK E SE", "IQ") { table : "L L L : - : L"; }
    pin (CK) { direction : input; clock_gate_clock_pin : true; } pin (E) { direction : input; }
    pin (SE) { direction : input; } pin (GCK) { direction : output; state_function : "(CK * IQ)"; } }
  cell (DLH_X1) { latch (IQ, IQN) { enable : "G"; data_in : "D"; }
    pin (D) { direction : input; } pin (G) { direction : input; } pin (Q) { direction : output; function : "IQ"; } }
  cell (fakeram45_64x7) { memory () { type : ram; address_width : 6; word_width : 7; }
    pin (clk) { direction : input; clock : true; } }
}"#;
        const N: &str = "module top(clk, a, en, y1, y2, y3, y4, q, z);\ninput clk, a, en; output y1, y2, y3, y4, q, z;\nwire gclk;\n\
          DFF_X1 r1(.D(a), .CK(clk), .Q(q));\nBUF_X1 b1(.A(a), .Z(y1));\nINV_X1 i1(.A(a), .ZN(y2));\n\
          CLKGATETST_X1 cg(.CK(clk), .E(en), .SE(1'b0), .GCK(gclk));\nAND2_X1 n1(.A1(a), .A2(en), .ZN(y3));\n\
          CLKBUF_X1 cb(.A(clk), .Z(y4));\nDLH_X1 l1(.D(a), .G(gclk), .Q(z));\nfakeram45_64x7 m1(.clk(clk));\nendmodule\n";
        let lib = Lib::parse(L).unwrap();
        let d = SdcDesign::from_netlist(&crate::netlist::parse(N).unwrap(), Some(&lib));
        let f = |e: &str| eval(&crate::sdc::query_words(&format!("get_cells -filter {{{e}}} *")), &d).map(|mut v| {
            v.sort();
            v.join(" ")
        });
        for (e, want) in [
            ("is_buffer", "b1 cb"),
            ("is_buffer == true", "b1 cb"),
            ("is_buffer == TRUE", "b1 cb"),
            ("is_buffer == 1", "b1 cb"),
            ("is_buffer != false", "b1 cb"),
            ("!is_buffer", "cg i1 l1 m1 n1 r1"),
            ("is_buffer =~ 1", "b1 cb"),
            ("is_buffer =~ tr*", ""),
            ("is_buffer == yes", ""),
            ("is_inverter", "i1"),
            ("is_clock_gate", "cg"),
            ("is_macro", ""),
            ("is_memory", "m1"),
            ("is_hierarchical", ""),
            ("defined(is_buffer)", "b1 cb"),
            ("undefined(is_memory)", "b1 cb cg i1 l1 n1 r1"),
            ("is_buffer || is_inverter", "b1 cb i1"),
            ("is_buffer && ref_name =~ CLK*", "cb"),
            ("is_memory == false", "b1 cb cg i1 l1 n1 r1"),
        ] {
            assert_eq!(f(e).as_deref(), Ok(want), "{e}");
        }
        for (e, want) in [
            ("is_sequential", "cg l1 m1 r1"),
            ("is_sequential == true", "cg l1 m1 r1"),
            ("!is_sequential", "b1 cb i1 n1"),
            ("is_sequential && !is_clock_gate && !is_memory", "l1 r1"),
        ] {
            assert_eq!(f(e).as_deref(), Ok(want), "extension: {e}");
        }
        // no library at all: not known, so not evaluated (never read as false)
        let bare = SdcDesign::from_netlist(&crate::netlist::parse(N).unwrap(), None);
        assert!(eval(&crate::sdc::query_words("get_cells -filter is_buffer *"), &bare).unwrap_err().contains("no library"));
    }

    /// The reference's answer is a SET for these; ours lists the same members.
    #[test]
    fn of_objects_members_match_the_reference() {
        let mut c: Vec<String> = q("get_cells -of_objects [get_nets a]").unwrap();
        c.sort();
        assert_eq!(c.join(" "), "u1 u2 u3 u4 u5");
        let mut p: Vec<String> = q("get_pins -of_objects [get_nets a]").unwrap();
        p.sort();
        assert_eq!(p.join(" "), "u1/A u2/A u3/A u4/A u5/A");
    }

    /// Rule (`-regexp`, `PatternMatch::compileRegexp` + the finders): `^` + pattern + `$` unGROUPED
    /// (`a|y1` is "starts with a or ends with y1"); a PORT pattern ending `[digits…]` is first read
    /// as a bus subscript, so on scalar ports `y[0-9]` finds nothing, `-nocase` included; cells and
    /// nets are not bus-read; a pin pattern is instance and pin. Every answer is the reference's
    /// (OpenSTA on this design and library, 2026-11-03, sorted).
    #[test]
    fn regexp_queries_match_the_reference() {
        let sorted = |s: &str| {
            let mut v = q(s).unwrap();
            v.sort();
            v.join(" ")
        };
        for (query, want) in [
            ("get_ports -regexp {y[1-3]}", ""),
            ("get_ports -regexp {y[0-9]}", ""),
            ("get_ports -regexp {a|y1}", "a y1"),
            ("get_ports -regexp {y.}", "y1 y2 y3 y4 y5 y6"),
            ("get_ports -regexp -nocase {Y[25]}", ""),
            ("get_ports -regexp {e.*}", "en"),
            ("get_cells -regexp {u[1-3]}", "u1 u2 u3"),
            ("get_cells -regexp {u.}", "u1 u2 u3 u4 u5"),
            ("get_pins -regexp {u[12]/[AB]}", "u1/A u1/B u2/A u2/B"),
            ("get_pins -regexp {u.*/Y}", "u1/Y u2/Y u3/Y u4/Y u5/Y"),
            ("get_nets -regexp {y[456]}", "y4 y5 y6"),
            ("get_ports -regexp {y\\d}", "y1 y2 y3 y4 y5 y6"),
            ("get_ports -regexp {(a|b)}", "a b"),
            ("get_pins -regexp {u1/.}", "u1/A u1/B u1/Y"),
            ("get_ports -regexp {y1|b}", "b y1"),
        ] {
            assert_eq!(sorted(query), want, "{query}");
        }
        // a case-blind regexp that is not bus-read does fold case
        assert_eq!(sorted("get_cells -regexp -nocase {U[12]}"), "u1 u2");
    }

    /// What this cannot answer it says so: one the view does not carry, and a -regexp pattern
    /// Rust would read differently.
    #[test]
    fn what_cannot_be_evaluated_says_why() {
        assert!(q("get_cells -filter liberty_cell *").unwrap_err().contains("not evaluated"));
        assert_eq!(q("get_cells -filter is_buffer *").unwrap(), Vec::<String>::new(), "evaluated now: no buffer here");
        assert!(q("get_ports -regexp {y\\b}").is_err());
    }

    /// Rule (`Properties::getProperty` per object type, OpenSTA at the 4155ac3 pin): a property
    /// the object type does not have THROWS (`PropertyUnknown`, no message id), which fails the
    /// query; `filterObjects` looks it up per object, so over no object it never throws. Every
    /// case is the reference's (box probe `unknown-prop-probe-20261108T155720Z`): an unknown
    /// property on ports, pins, nets and instances errors (port `is_clock` too, a pin property),
    /// inside `defined()`, and after a known predicate; `src` is known on instances and pins
    /// (OpenROAD registers it), not on nets; a known timing property is not an unknown one.
    #[test]
    fn an_unknown_property_is_the_references_error() {
        let unknown = |s: &str| q(s).unwrap_err().contains(UNKNOWN_PROPERTY);
        for s in [
            "get_ports -filter is_bogus *",
            "get_ports -filter is_clock *",
            "get_ports -filter slack_bogus *",
            "get_pins -filter is_bogus */*",
            "get_nets -filter is_bogus *",
            "get_nets -filter src *",
            "get_cells -filter is_bogus *",
            "get_cells -filter {defined(is_bogus)} *",
            "get_cells -filter {ref_name =~ AND* && is_bogus} *",
            // after (and beside) a property this view cannot evaluate: the reference evaluates it
            "get_cells -filter {liberty_cell == x || is_bogus} *",
            "get_pins -of_objects [get_cells -filter is_bogus *]",
        ] {
            assert!(unknown(s), "{s}: {:?}", q(s));
        }
        for s in [
            "get_pins -filter src */*",
            "get_cells -filter src *",
            "get_pins -filter is_clock */*",
            "get_ports -filter slack_max *",
            "get_ports -filter activity *",
        ] {
            let e = q(s).unwrap_err();
            assert!(e.contains("not evaluated") && !e.contains(UNKNOWN_PROPERTY), "{s}: {e}");
        }
        // no object to look the property up on: no error, nothing
        assert_eq!(q("get_cells -filter is_bogus nomatch*").unwrap(), Vec::<String>::new());
        assert_eq!(q("get_nets -filter src nomatch*").unwrap(), Vec::<String>::new());
    }

    /// Rule (`include_file`, OpenSTA `tcl/Util.tcl` at the pin, `sta_continue_on_error` 0 by
    /// default): the query's error ends the read at that command and `read_sdc` errors; an
    /// unknown property over no object is no error, so the file reads on. The reference on the
    /// probe (`unknown-prop-probe-20261108T155720Z`): a mid-file `set_false_path -to [get_cells
    /// -filter is_bogus *]` kept the `set_max_delay` before it, never applied the one after, and
    /// `read_sdc` returned the error (uncaught, the run exits 1); a `set v [...]` stopped at the
    /// `set`; over `nomatch*` the next command was applied. This reader stops as it does at a
    /// command the reference does not know: the file is not read. Without a design the query is
    /// refused as before (not evaluated), the file read on.
    #[test]
    fn an_unknown_filter_property_stops_the_file() {
        let head = "create_clock -name c -period 10 [get_ports s]\nset_max_delay 3 -to [get_ports y1]\n";
        for bad in [
            "set_false_path -to [get_cells -filter is_bogus *]",
            "set v [get_cells -filter is_bogus *]",
            "set_false_path -to [get_pins -of_objects [get_cells -filter {ref_name =~ AND* && is_bogus} *]]",
            "set_input_delay 1 -clock c [get_ports -filter is_clock *]",
            "set_false_path -to [list [get_cells -filter is_bogus *]]",
        ] {
            let sdc = format!("{head}{bad}\nset_max_delay 4 -to [get_ports y2]\n");
            let e = crate::sdc::Sdc::parse_with(&sdc, &design()).unwrap_err().0;
            assert!(e.contains(UNKNOWN_PROPERTY) && e.contains("not read"), "{bad}: {e}");
            let without = crate::sdc::Sdc::parse(&sdc).unwrap();
            assert_eq!(without.unevaluated_queries.len(), usize::from(!bad.starts_with("set ")), "{bad}");
        }
        let empty = format!("{head}set_false_path -from [get_ports a] -to [get_cells -quiet -filter is_bogus nomatch*]\nset_max_delay 4 -to [get_ports y2]\n");
        let read = crate::sdc::Sdc::parse_with(&empty, &design()).unwrap();
        assert_eq!(read.path_delays.len(), 2, "{:?}", read.rejected);
        // a property the reference has but this view does not carry: refused, the file read on
        let unread = format!("{head}set_false_path -to [get_cells -filter liberty_cell *]\nset_max_delay 4 -to [get_ports y2]\n");
        let read = crate::sdc::Sdc::parse_with(&unread, &design()).unwrap();
        assert_eq!((read.path_delays.len(), read.unevaluated_queries.len()), (2, 1), "{:?}", read.rejected);
    }

    /// Rule (OpenSTA `FilterObjects.cc` at the 4155ac3 pin): `FilterExpr::lex` raises 2600 at a
    /// character no token starts with; `shuntingYard` 2601 for a `)` with the operator stack
    /// empty, 2602 for one that empties it, 2603 for a `(` left over; `filterObjects` evaluates the
    /// postfix and raises 2604 (`||`), 2605 (`&&`), 2606 (`!`) short of operands, then 2607 for no
    /// set and 2608 for more than one. Each is `Report::error`, which throws, so the query fails.
    /// All of it runs only `if (objects)`, and SWIG's `tclListSeqPtr` gives a null list for an
    /// empty one: over no object nothing is parsed. Predicates throw an unknown property in
    /// postfix order, so `in put` is the unknown property `put`, not 2608. Every id below is the
    /// reference's on the probe (box `filter-followups-20261108T165629Z/ref-probe`, `-threads 1`).
    #[test]
    fn a_malformed_filter_is_the_references_error() {
        for (s, id) in [
            ("get_cells -filter {ref_name ~= AND*} *", 2600),
            ("get_pins -filter {direction ==} */*", 2600),
            ("get_nets -filter {name =~ \"y*\"} *", 2600),
            ("get_cells -filter {is_buffer)} *", 2601),
            ("get_cells -filter {!is_buffer)} *", 2602),
            ("get_cells -filter {(is_buffer} *", 2603),
            ("get_pins -of_objects [get_cells -filter {(ref_name =~ AND*} *]", 2603),
            ("get_cells -filter {is_buffer ||} *", 2604),
            ("get_cells -filter {is_buffer &&} *", 2605),
            ("get_cells -filter {!} *", 2606),
            ("get_cells -filter {} *", 2607),
            ("get_cells -filter {   } *", 2607),
            ("get_cells -filter {is_buffer is_inverter} *", 2608),
        ] {
            let e = q(s).unwrap_err();
            assert!(e.contains(&format!("STA-{id}")) && e.contains(MALFORMED_FILTER) && stops_the_read(&e), "{s}: {e}");
        }
        // the reference's order: lexed whole first, then the postfix with its property lookups
        let e = q("get_ports -filter {direction == in put} *").unwrap_err();
        assert!(e.contains(UNKNOWN_PROPERTY) && !e.contains(MALFORMED_FILTER), "{e}");
        let e = q("get_cells -filter {is_bogus ~= x} *").unwrap_err();
        assert!(e.contains("STA-2600"), "lexed before any lookup: {e}");
        // over no object: not parsed, no error, nothing
        for s in ["get_cells -quiet -filter {ref_name ~= x} nomatch*", "get_cells -quiet -filter {is_buffer &&} nomatch*"] {
            assert_eq!(q(s).unwrap(), Vec::<String>::new(), "{s}");
        }
        // well formed (the probe's controls), and a property not evaluated is not a stop
        assert_eq!(ok("get_cells -filter {ref_name==AND2} *"), "u1");
        assert_eq!(ok("get_cells -filter {@ref_name=~*R2&&!is_buffer} *"), "u2 u4");
        assert!(!stops_the_read(&q("get_cells -filter {liberty_cell == x} *").unwrap_err()));
    }

    /// Rule (`include_file`, as for an unknown property): a malformed filter's error ends the read
    /// at that command and `read_sdc` errors; over no object it is no error, so the file reads on.
    /// The reference on the probe: each of 2600-2608 mid-file kept the `set_max_delay` before and
    /// not the one after (`READ_SDC ERROR`), a `set v [...]` and one inside `-of_objects`
    /// stopped too; over `-quiet nomatch*` the next command was applied (`READ_SDC_OK`).
    #[test]
    fn a_malformed_filter_stops_the_file() {
        let head = "create_clock -name c -period 10 [get_ports s]\nset_max_delay 3 -to [get_ports y1]\n";
        for bad in [
            "set_false_path -to [get_cells -filter {ref_name ~= AND*} *]",
            "set_false_path -to [get_cells -filter {is_buffer)} *]",
            "set_false_path -to [get_cells -filter {(is_buffer} *]",
            "set_false_path -to [get_cells -filter {is_buffer &&} *]",
            "set_false_path -to [get_cells -filter {} *]",
            "set_false_path -to [get_cells -filter {is_buffer is_inverter} *]",
            "set v [get_cells -filter {ref_name ~= x} *]",
            "set_false_path -to [get_pins -of_objects [get_cells -filter {(ref_name =~ AND*} *]]",
        ] {
            let sdc = format!("{head}{bad}\nset_max_delay 4 -to [get_ports y2]\n");
            let e = crate::sdc::Sdc::parse_with(&sdc, &design()).unwrap_err().0;
            assert!(e.contains(MALFORMED_FILTER) && e.contains("not read"), "{bad}: {e}");
        }
        for none in [
            "set_false_path -from [get_ports a] -to [get_cells -quiet -filter {ref_name ~= x} nomatch*]",
            "set_false_path -from [get_ports a] -to [get_cells -quiet -filter {is_buffer &&} nomatch*]",
        ] {
            let sdc = format!("{head}{none}\nset_max_delay 4 -to [get_ports y2]\n");
            let read = crate::sdc::Sdc::parse_with(&sdc, &design()).unwrap();
            assert_eq!((read.path_delays.len(), read.unevaluated_queries.len()), (2, 0), "{none}: {:?}", read.rejected);
        }
    }

    /// The command is applied once a design is in scope, and still refused without one.
    #[test]
    fn a_constraint_file_read_against_the_design_applies_its_queries() {
        let sdc = "create_clock -name c -period 10 [get_ports s]\n\
                   set_input_delay 1 -clock c [get_ports -filter \"direction == input && name !~ s\" *]\n";
        let without = crate::sdc::Sdc::parse(sdc).unwrap();
        assert!(without.input_delays.is_empty() && without.unevaluated_queries.len() == 1);
        let with = crate::sdc::Sdc::parse_with(sdc, &design()).unwrap();
        assert!(with.unevaluated_queries.is_empty(), "{:?}", with.unevaluated_queries);
        let mut ports: Vec<String> = with.input_delays.iter().flat_map(|d| d.ports.clone()).collect();
        ports.sort();
        assert_eq!(ports.join(" "), "a b en");
    }
}
