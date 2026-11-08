//! An RTL constraint file's objects, resolved against the ELABORATED names and carried to an
//! implementation netlist through a proved register map.
//!
//! A constraint file written before synthesis names the design as it was elaborated: ports,
//! registers and nets by their RTL names. The elaborated netlist (a synthesis front end that keeps
//! RTL names on nets) names each register by the net on its stored-value output, not by its cell,
//! which is `u<n>`. The implementation names the same registers differently again. This module:
//!
//! 1. [`elab_names`]: the elaborated name space: ports, registers (by the net on their stored-value
//!    output), the other cells, their pins, and the nets.
//! 2. [`object_refs`]: every object list in the file, from its canonical write (the form `diff`
//!    compares), with the query that named it.
//! 3. [`translate_bits`]: an RTL bit index turned into the elaborated one by the vector's
//!    declared range (elaboration numbers every vector's bits from 0), then [`resolve`]: each
//!    list's pattern matched against the names its query can name.
//! 4. [`carry`]: each object it names carried to the implementation: a port as itself, a clock as
//!    itself, a register (or a register's pin) through the register map; anything else is not
//!    carried, and why is said.
//!
//! ⛔ Nothing is carried by a name heuristic. A register is carried only through the map (a map
//! that `lec map` proves); a net or a combinational cell is not carried at all until a proved
//! match exists for it. A pattern that names nothing is reported, with the query that named it.

use std::collections::{BTreeMap, BTreeSet};

use crate::liberty::{Dir, Lib};
use crate::netlist::Netlist;
use crate::sdc::{bus_form, object_matches, tokenize, Objects, Sdc};
use crate::sdc_write::WriteOpts;

/// What an elaborated object is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ObjKind {
    Port,
    Register,
    RegisterPin,
    /// A cell that is not a register (combinational).
    Cell,
    CellPin,
    Net,
    Clock,
}

impl ObjKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ObjKind::Port => "port",
            ObjKind::Register => "register",
            ObjKind::RegisterPin => "register_pin",
            ObjKind::Cell => "cell",
            ObjKind::CellPin => "cell_pin",
            ObjKind::Net => "net",
            ObjKind::Clock => "clock",
        }
    }
}

/// The elaborated name space.
#[derive(Debug, Clone, Default)]
pub struct ElabNames {
    pub ports: BTreeSet<String>,
    pub inputs: BTreeSet<String>,
    pub outputs: BTreeSet<String>,
    /// A register by its name (the net on its stored-value output, else its instance) -> its
    /// cell's pin names.
    pub registers: BTreeMap<String, Vec<String>>,
    /// The cells that are not registers -> their pin names.
    pub cells: BTreeMap<String, Vec<String>>,
    pub nets: BTreeSet<String>,
    pub clocks: BTreeSet<String>,
    /// The output pins of the register cells (`Q`, `QN`): what an RTL name may end in.
    pub register_outputs: BTreeSet<String>,
}

/// One object list in the file: the canonical line it is on, the query that named it
/// (`get_cells`, … `""` for a bare name), and one pattern of it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjectRef {
    pub line: String,
    pub accessor: String,
    /// `None` for a query with no pattern (`all_inputs`, `all_registers`).
    pub pattern: Option<String>,
}

/// An elaborated object a pattern names, and where it goes in the implementation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Member {
    pub name: String,
    pub kind: ObjKind,
    /// The implementation's name for it; `None` when it is not carried.
    pub carried: Option<String>,
    /// Why it is not carried: `register-not-proved` (no map entry), `combinational` (a net or
    /// a cell that is not a register: no proved match exists for it), `port-not-in-revised`,
    /// `mapped-cell-not-in-revised`.
    pub reason: Option<&'static str>,
}

/// One pattern's resolution: empty `members` is a pattern that names nothing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolved {
    pub object: ObjectRef,
    pub members: Vec<Member>,
    /// What the declared range did to the pattern's bit index.
    pub bits: BitIndex,
    /// How an RTL form of a register's name matched the elaborated one, when it did not match
    /// as written.
    pub alias: Alias,
}

/// What [`alias`] did to a pattern that named nothing as written.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum Alias {
    /// Not tried (the pattern matched as written), or nothing matched.
    #[default]
    None,
    /// Matched the elaborated registers by these rules: `separator` (`/` read as `.`),
    /// `reg-suffix` (the register name's `_reg` dropped), `output-pin` (a trailing register
    /// output pin, `/Q`, read as the register).
    Matched { rules: Vec<&'static str> },
    /// The RTL form matches more than one elaborated register that a name could mean (two
    /// elaborated names with the same alias): reported, never guessed.
    Ambiguous { candidates: Vec<String> },
}

/// Each vector's range AS DECLARED in the RTL, `(left, right)` of `[left:right]`, by the name
/// the elaborated netlist gives the vector (the instance path joined by `.`, then the name:
/// `u_sub.t`, `g[0].u_in.t`; a top-level port or variable by its name alone).
pub type DeclaredRanges = BTreeMap<String, (i64, i64)>;

/// What [`translate_bits`] did to a pattern.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub enum BitIndex {
    /// No declared range applies (no index, a glob index, a vector the ranges do not list), or
    /// the index is the same after elaboration (`[3:0]`).
    #[default]
    Same,
    /// The RTL index named another elaborated bit: the pattern resolved is `to`.
    Translated { to: String, declared: (i64, i64) },
    /// An index outside the declared range: it names nothing, and no bit is guessed.
    OutOfRange { declared: (i64, i64) },
}

/// The whole file's resolution.
#[derive(Debug, Clone, Default)]
#[must_use]
pub struct RtlMap {
    pub resolved: Vec<Resolved>,
    /// Lists this module cannot resolve without evaluating them (`-filter`, `-of_objects`, an
    /// accessor it does not know), as written.
    pub not_resolved: Vec<ObjectRef>,
    /// Commands the canonical write does not hold, with why: their objects were never read.
    pub not_written: Vec<String>,
}

impl RtlMap {
    /// Patterns that name nothing in the elaborated design.
    pub fn unknown(&self) -> impl Iterator<Item = &Resolved> {
        self.resolved.iter().filter(|r| r.members.is_empty())
    }
    /// Objects named but not carried to the implementation.
    pub fn uncarried(&self) -> impl Iterator<Item = (&Resolved, &Member)> {
        self.resolved.iter().flat_map(|r| r.members.iter().filter(|m| m.carried.is_none()).map(move |m| (r, m)))
    }
    /// Every object resolved and carried, and nothing left unresolved.
    pub fn complete(&self) -> bool {
        self.unknown().next().is_none() && self.uncarried().next().is_none() && self.not_resolved.is_empty() && self.not_written.is_empty()
    }
}

/// The thin sequence of the stages the module doc names: `sdc` against the elaborated netlist
/// `elab` (with `lib` for the registers), carried through `map` (elaborated register name ->
/// implementation cell), checked against the implementation `revised` when given.
pub fn rtl_map(sdc: &Sdc, elab: &Netlist, lib: &Lib, ranges: &DeclaredRanges, map: &[(String, String)], revised: Option<&Netlist>) -> RtlMap {
    let names = elab_names(elab, lib, sdc);
    let (mut refs, not_written) = object_refs(sdc);
    refs.extend(dropped_group_members(sdc));
    let mut out = RtlMap { not_written, ..RtlMap::default() };
    for r in refs {
        let bits = r.pattern.as_deref().map_or(BitIndex::Same, |p| translate_bits(p, ranges));
        let elab_ref = match &bits {
            BitIndex::Translated { to, .. } => ObjectRef { pattern: Some(to.clone()), ..r.clone() },
            _ => r.clone(),
        };
        let (members, bits, alias) = match (bits, resolve(&elab_ref, &names)) {
            (bits @ BitIndex::OutOfRange { .. }, _) => (Some(Vec::new()), bits, Alias::None),
            (bits, Some(m)) if !m.is_empty() => (Some(m), bits, Alias::None),
            (bits, found) => match alias(&r, &names, ranges) {
                Some((m, a, b)) => (Some(m), b, a),
                None => (found, bits, Alias::None),
            },
        };
        match members {
            Some(m) => out.resolved.push(Resolved { members: carry(m, map, revised), object: r, bits, alias }),
            None => out.not_resolved.push(r),
        }
    }
    out
}

/// Stage 3a: an RTL bit index as the elaborated netlist numbers it. Elaboration (ours and the
/// reference's `synthesize` alike) numbers every vector's bits from 0 at the RIGHT bound of its
/// declaration, ports included: bit `i` of `[left:right]` is elaborated bit `|i - right|`
/// (`logic [11:8] r`: `r[8]` is `r[0]`; `logic [8:11] s`: `s[11]` is `s[0]`, `s[8]` is `s[3]`;
/// witnessed on the reference, `correlation/harness/sdc/register-map/ranges-probe/`). The
/// index is the pattern's LAST `[n]` or `[n:m]` (an SDC bus range), or, in a pin pattern
/// `reg[n]/D`, the instance part's. An index outside the declared range is
/// [`BitIndex::OutOfRange`]: reported, never moved to a bit that exists. A glob index (`[*]`)
/// names every bit either way, and is left as written.
pub fn translate_bits(pattern: &str, ranges: &DeclaredRanges) -> BitIndex {
    let (head, pin) = match (bus_form(pattern), pattern.rsplit_once('/')) {
        (Some(_), _) => (pattern, ""),
        (None, Some((inst, _))) if bus_form(inst).is_some() => (inst, &pattern[inst.len()..]),
        _ => return BitIndex::Same,
    };
    let Some((base, sub)) = bus_form(head) else { return BitIndex::Same };
    let Some(&(left, right)) = ranges.get(base) else { return BitIndex::Same };
    let idx: Vec<i64> = match sub.split(':').map(|t| t.parse::<i64>()).collect::<Result<Vec<_>, _>>() {
        Ok(v) if (1..=2).contains(&v.len()) => v,
        _ => return BitIndex::Same,
    };
    let (lo, hi) = (left.min(right), left.max(right));
    if idx.iter().any(|i| *i < lo || *i > hi) {
        return BitIndex::OutOfRange { declared: (left, right) };
    }
    let to: Vec<String> = idx.iter().map(|i| (i - right).abs().to_string()).collect();
    if idx.iter().map(|i| (i - right).abs()).eq(idx.iter().copied()) {
        return BitIndex::Same;
    }
    BitIndex::Translated { to: format!("{base}[{}]{pin}", to.join(":")), declared: (left, right) }
}

/// An RTL register name's alias key: `/` read as `.`, the register name's `_reg` dropped (the
/// last name, before its index), and the rules that changed it.
fn alias_key(name: &str) -> (String, Vec<&'static str>) {
    let mut rules = Vec::new();
    let mut k = name.to_string();
    if k.contains('/') {
        k = k.replace('/', ".");
        rules.push("separator");
    }
    let last = k.rfind('.').map_or(0, |i| i + 1);
    let (head, tail) = k.split_at(last);
    let (stem, index) = tail.find('[').map_or((tail, ""), |i| tail.split_at(i));
    if let Some(s) = stem.strip_suffix("_reg").filter(|s| !s.is_empty()) {
        k = format!("{head}{s}{index}");
        rules.push("reg-suffix");
    }
    (k, rules)
}

/// Stage 3b: a pattern that named nothing as written, read as an RTL form of an elaborated
/// register's name (decided 2026-11-08): `/` or `.` as the hierarchy separator, with or without
/// the register name's `_reg` (on either side: the elaborated name may be the one that has it),
/// and, on a cell or bare query, with or without a trailing register output pin (`r_reg[1]/Q`;
/// `get_pins` keeps its pin). Both sides are compared by that key, after the bit index is
/// translated by the declared range. Only queries that can name a register (`get_cells`,
/// `get_pins`, a bare name) are read this way. When
/// two elaborated registers share the key a name matches, it is [`Alias::Ambiguous`], with no
/// members. `None`: nothing matched.
pub fn alias(r: &ObjectRef, n: &ElabNames, ranges: &DeclaredRanges) -> Option<(Vec<Member>, Alias, BitIndex)> {
    let pat = r.pattern.as_deref()?;
    let pins = r.accessor == "get_pins";
    if !(pins || r.accessor == "get_cells" || r.accessor.is_empty()) {
        return None;
    }
    // the register part, the pin a get_pins pattern names, and the output-pin rule
    let (reg, pin, mut rules) = match pat.rsplit_once('/') {
        Some((inst, p)) if pins => (inst, Some(p), Vec::new()),
        Some((inst, p)) if n.register_outputs.contains(p) => (inst, None, vec!["output-pin"]),
        _ if pins => return None,
        _ => (pat, None, Vec::new()),
    };
    let (key, more) = alias_key(reg);
    rules.extend(more);
    let (key, bits) = match translate_bits(&key, ranges) {
        BitIndex::Translated { to, declared } => (to, BitIndex::Translated { to: String::new(), declared }),
        b => (key, b),
    };
    if matches!(bits, BitIndex::OutOfRange { .. }) {
        return Some((Vec::new(), Alias::Matched { rules }, bits));
    }
    let mut by_key: BTreeMap<String, Vec<&String>> = BTreeMap::new();
    for e in n.registers.keys() {
        by_key.entry(alias_key(e).0).or_default().push(e);
    }
    let hits: Vec<(&String, &Vec<&String>)> = by_key.iter().filter(|(k, _)| object_matches(&key, k)).collect();
    if hits.is_empty() {
        return None;
    }
    // the `_reg` may be on the elaborated side only (`a.t_reg[0]` named `a/t[0]`)
    if !rules.contains(&"reg-suffix") && hits.iter().flat_map(|(k, v)| v.iter().map(move |e| (k, e))).any(|(k, e)| k.as_str() != e.as_str()) {
        rules.push("reg-suffix");
    }
    let shared: Vec<String> = hits.iter().filter(|(_, v)| v.len() > 1).flat_map(|(_, v)| v.iter().map(|e| e.to_string())).collect();
    if !shared.is_empty() {
        return Some((Vec::new(), Alias::Ambiguous { candidates: shared }, bits));
    }
    let member = |name: String, kind| Member { name, kind, carried: None, reason: None };
    let members: Vec<Member> = hits
        .iter()
        .flat_map(|(_, v)| v.iter())
        .flat_map(|e| match pin {
            None => vec![member(e.to_string(), ObjKind::Register)],
            Some(p) => n.registers[*e].iter().filter(|x| object_matches(p, x)).map(|x| member(format!("{e}/{x}"), ObjKind::RegisterPin)).collect(),
        })
        .collect();
    let bits = match bits {
        BitIndex::Translated { declared, .. } => BitIndex::Translated { to: key, declared },
        b => b,
    };
    Some((members, Alias::Matched { rules }, bits))
}

/// Stage 1: the elaborated name space. A register is a cell with an `ff` / `latch` group, named
/// by the net on its output whose function is the group's stored value (`IQ`), else by its
/// instance.
pub fn elab_names(nl: &Netlist, lib: &Lib, sdc: &Sdc) -> ElabNames {
    let mut n = ElabNames::default();
    n.ports.extend(nl.inputs.iter().chain(&nl.outputs).chain(&nl.inouts).cloned());
    n.inputs.extend(nl.inputs.iter().chain(&nl.inouts).cloned());
    n.outputs.extend(nl.outputs.iter().chain(&nl.inouts).cloned());
    n.nets.extend(n.ports.iter().cloned());
    for inst in &nl.insts {
        n.nets.extend(inst.conns.iter().map(|(_, net)| net.clone()));
        let Some(cell) = lib.cells.get(&inst.cell) else {
            n.cells.insert(inst.name.clone(), inst.conns.iter().map(|(p, _)| p.clone()).collect());
            continue;
        };
        let pins: Vec<String> = cell.pins.keys().cloned().collect();
        if cell.is_seq {
            n.register_outputs.extend(cell.pins.iter().filter(|(_, x)| x.direction == Dir::Out).map(|(p, _)| p.clone()));
        }
        let q = cell.seq_group.as_ref().filter(|_| cell.is_seq).and_then(|g| {
            inst.conns.iter().find(|(p, _)| {
                cell.pins.get(p).is_some_and(|x| x.direction == Dir::Out && x.function.as_deref().map(str::trim) == Some(g.iq.as_str()))
            })
        });
        match (cell.is_seq, q) {
            (true, Some((_, net))) => n.registers.insert(net.clone(), pins),
            (true, None) => n.registers.insert(inst.name.clone(), pins),
            (false, _) => n.cells.insert(inst.name.clone(), pins),
        };
    }
    n.clocks.extend(sdc.clocks.iter().map(|c| c.name.clone()));
    n
}

/// The options whose value is an object list even when it is written bare (`-from {r[0]}`, no
/// query): the canonical write keeps such a value as written, so it is read as bare names. Any
/// other option's bare value (`-name`, `-period`, `-waveform`, …) is a value, not an object.
const OBJECT_OPTIONS: [&str; 14] = [
    "-from", "-rise_from", "-fall_from", "-to", "-rise_to", "-fall_to", "-through", "-rise_through", "-fall_through",
    "-clock", "-clocks", "-source", "-master_clock", "-group",
];

/// What the `all_*` queries read as (`[all_registers]` is the name `*REGS*`): the whole kind,
/// not a pattern.
const ALL_OF_A_KIND: [&str; 3] = ["*INPUTS*", "*OUTPUTS*", "*REGS*"];

/// Stage 2: every object list on every canonical line: an option's value (a bracketed query, or,
/// for an [`OBJECT_OPTIONS`] option, bare names), and each positional list (a bracketed query, or
/// a bare name or brace list). A `[list …]` / `[concat …]` is its items; a query with no pattern,
/// or an `all_*` query, is one ref with `pattern: None`.
/// Also returns each command the canonical write does not hold (rejected, or a query that needs
/// the design to evaluate), with why: its objects are not read here, so it is reported, never
/// passed over.
pub fn object_refs(sdc: &Sdc) -> (Vec<ObjectRef>, Vec<String>) {
    let w = sdc.write(&WriteOpts { header: Some(String::new()), ..WriteOpts::default() });
    let mut out = Vec::new();
    for line in w.text.lines().map(str::trim).filter(|l| !l.is_empty() && !l.starts_with('#')) {
        let toks = tokenize(line);
        let Some(cmd) = toks.first() else { continue };
        let valued: Vec<&str> = crate::sdc_flags::keys(cmd);
        let mut k = 1;
        while k < toks.len() {
            let t = &toks[k];
            let is_flag = t.starts_with('-') && t.parse::<f64>().is_err();
            if is_flag && valued.contains(&t.as_str()) && k + 1 < toks.len() {
                if toks[k + 1].starts_with('[') {
                    push_objects(&Objects::of(&toks[k + 1]), line, &mut out);
                } else if OBJECT_OPTIONS.contains(&t.as_str()) {
                    push_objects(&bare(&toks[k + 1]), line, &mut out);
                }
                k += 2;
                continue;
            }
            if !is_flag && t.split_whitespace().any(|w| w.parse::<f64>().is_err()) {
                let o = if t.starts_with('[') { Objects::of(t) } else { bare(t) };
                push_objects(&o, line, &mut out);
            }
            k += 1;
        }
    }
    (out, w.not_written)
}

/// Stage 2b: each `set_clock_groups -group` member the reader left out because it names no clock
/// defined before it (the reference's own behaviour, a warning). The canonical write holds the
/// group without it, so [`object_refs`] never sees it; it is a pattern that names nothing.
pub fn dropped_group_members(sdc: &Sdc) -> Vec<ObjectRef> {
    sdc.ignored_words
        .iter()
        .filter_map(|w| w.strip_prefix("set_clock_groups -group ").map(|m| (w, m)))
        .map(|(w, m)| ObjectRef { line: w.clone(), accessor: "get_clocks".into(), pattern: Some(m.to_string()) })
        .collect()
}

/// A bare name or brace list, as names with no query.
fn bare(t: &str) -> Objects {
    Objects { names: t.split_whitespace().map(str::to_string).collect(), ..Objects::default() }
}

fn push_objects(o: &Objects, line: &str, out: &mut Vec<ObjectRef>) {
    if !o.items.is_empty() {
        for i in &o.items {
            push_objects(i, line, out);
        }
        return;
    }
    if o.names.is_empty() || o.names.iter().all(|n| ALL_OF_A_KIND.contains(&n.as_str())) {
        out.push(ObjectRef { line: line.to_string(), accessor: o.accessor.clone(), pattern: None });
        return;
    }
    for n in &o.names {
        out.push(ObjectRef { line: line.to_string(), accessor: o.accessor.clone(), pattern: Some(n.clone()) });
    }
}

/// Stage 3: what one pattern names, by the kinds its query can name (a bare name: any kind).
/// `None`: the list cannot be resolved without evaluating it.
pub fn resolve(r: &ObjectRef, n: &ElabNames) -> Option<Vec<Member>> {
    let member = |name: &String, kind| Member { name: name.clone(), kind, carried: None, reason: None };
    let all = |set: &mut dyn Iterator<Item = &String>, kind: ObjKind, pat: &Option<String>| -> Vec<Member> {
        set.filter(|x| pat.as_deref().is_none_or(|p| object_matches(p, x))).map(|x| member(x, kind)).collect()
    };
    let pins = |pat: &str| -> Vec<Member> {
        let Some((ip, pp)) = pat.rsplit_once('/') else { return Vec::new() };
        let mut v = Vec::new();
        for (cells, kind) in [(&n.registers, ObjKind::RegisterPin), (&n.cells, ObjKind::CellPin)] {
            for (c, ps) in cells.iter().filter(|(c, _)| object_matches(ip, c)) {
                v.extend(ps.iter().filter(|p| object_matches(pp, p)).map(|p| member(&format!("{c}/{p}"), kind)));
            }
        }
        v
    };
    let p = &r.pattern;
    if p.is_none() && !r.accessor.starts_with("all_") {
        return None;
    }
    Some(match r.accessor.as_str() {
        "get_ports" => all(&mut n.ports.iter(), ObjKind::Port, p),
        "all_inputs" => all(&mut n.inputs.iter(), ObjKind::Port, p),
        "all_outputs" => all(&mut n.outputs.iter(), ObjKind::Port, p),
        "get_clocks" | "all_clocks" => all(&mut n.clocks.iter(), ObjKind::Clock, p),
        "all_registers" if p.is_none() => all(&mut n.registers.keys(), ObjKind::Register, p),
        "get_cells" => {
            let mut v = all(&mut n.registers.keys(), ObjKind::Register, p);
            v.extend(all(&mut n.cells.keys(), ObjKind::Cell, p));
            v
        }
        "get_pins" => pins(p.as_deref()?),
        "get_nets" => all(&mut n.nets.iter(), ObjKind::Net, p),
        "" => {
            let pat = p.as_deref()?;
            let mut v = all(&mut n.ports.iter(), ObjKind::Port, p);
            v.extend(all(&mut n.clocks.iter(), ObjKind::Clock, p));
            v.extend(all(&mut n.registers.keys(), ObjKind::Register, p));
            if v.is_empty() {
                v.extend(pins(pat));
            }
            if v.is_empty() {
                v.extend(all(&mut n.nets.iter(), ObjKind::Net, p));
            }
            v
        }
        _ => return None,
    })
}

/// Stage 4: each member carried to the implementation, or the reason it is not.
pub fn carry(members: Vec<Member>, map: &[(String, String)], revised: Option<&Netlist>) -> Vec<Member> {
    let lookup: BTreeMap<&str, &str> = map.iter().map(|(a, b)| (a.as_str(), b.as_str())).collect();
    let has_port = |p: &str| revised.is_none_or(|r| r.inputs.iter().chain(&r.outputs).chain(&r.inouts).any(|x| x == p));
    let has_cell = |c: &str| revised.is_none_or(|r| r.insts.iter().any(|i| i.name == c));
    members
        .into_iter()
        .map(|mut m| {
            let (carried, reason) = match m.kind {
                ObjKind::Port if has_port(&m.name) => (Some(m.name.clone()), None),
                ObjKind::Port => (None, Some("port-not-in-revised")),
                ObjKind::Clock => (Some(m.name.clone()), None),
                ObjKind::Register | ObjKind::RegisterPin => {
                    let (reg, pin) = match m.kind {
                        ObjKind::RegisterPin => m.name.rsplit_once('/').map_or((m.name.as_str(), None), |(a, b)| (a, Some(b))),
                        _ => (m.name.as_str(), None),
                    };
                    match lookup.get(reg) {
                        Some(to) if has_cell(to) => (Some(pin.map_or_else(|| to.to_string(), |p| format!("{to}/{p}"))), None),
                        Some(_) => (None, Some("mapped-cell-not-in-revised")),
                        None => (None, Some("register-not-proved")),
                    }
                }
                ObjKind::Cell | ObjKind::CellPin | ObjKind::Net => (None, Some("combinational")),
            };
            m.carried = carried;
            m.reason = reason;
            m
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const LIB: &str = "library (t) {\n\
      cell (INV) { pin (A) { direction : input; } pin (Y) { direction : output; function : \"!A\"; } }\n\
      cell (DFF) { ff (IQ, IQN) { clocked_on : \"CK\"; next_state : \"D\"; }\n\
        pin (CK) { direction : input; clock : true; } pin (D) { direction : input; }\n\
        pin (Q) { direction : output; function : \"IQ\"; } }\n}\n";

    /// An elaborated netlist: registers `u1`, `u2` whose Q nets carry the RTL names `cnt[0]`,
    /// `cnt[1]`; a combinational cell `u3` on net `nx`.
    const ELAB: &str = "module t(clk,a,y);\ninput clk,a; output y;\nwire nx;\nwire [1:0] cnt;\n\
      INV u3(.A(a),.Y(nx));\nDFF u1(.CK(clk),.D(nx),.Q(cnt[0]));\nDFF u2(.CK(clk),.D(cnt[0]),.Q(cnt[1]));\n\
      INV u4(.A(cnt[1]),.Y(y));\nendmodule\n";
    /// The implementation: the same registers as `_10_`, `_11_`; no cell named like the RTL.
    const IMPL: &str = "module t(clk,a,y);\ninput clk,a; output y;\nwire n1,q0,q1;\n\
      INV _5_(.A(a),.Y(n1));\nDFF _10_(.CK(clk),.D(n1),.Q(q0));\nDFF _11_(.CK(clk),.D(q0),.Q(q1));\n\
      INV _6_(.A(q1),.Y(y));\nendmodule\n";

    fn run(sdc: &str, map: &[(&str, &str)]) -> RtlMap {
        run_on(ELAB, IMPL, sdc, map)
    }
    fn run_on(elab: &str, imp: &str, sdc: &str, map: &[(&str, &str)]) -> RtlMap {
        let lib = Lib::parse(LIB).unwrap();
        let elab = crate::netlist::parse(elab).unwrap();
        let imp = crate::netlist::parse(imp).unwrap();
        let s = Sdc::parse(sdc).unwrap();
        let m: Vec<(String, String)> = map.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
        rtl_map(&s, &elab, &lib, &DeclaredRanges::new(), &m, Some(&imp))
    }
    const MAP: [(&str, &str); 2] = [("cnt[0]", "_10_"), ("cnt[1]", "_11_")];

    fn carried(r: &RtlMap) -> Vec<(String, String)> {
        let mut v: Vec<(String, String)> =
            r.resolved.iter().flat_map(|x| x.members.iter().filter_map(|m| m.carried.clone().map(|c| (m.name.clone(), c)))).collect();
        v.sort();
        v.dedup();
        v
    }

    /// A register is named by the net on its stored-value output (its cell is `u<n>`), and is
    /// carried through the map: a cell, a pin of it, a glob over a register vector.
    #[test]
    fn registers_are_carried_through_the_map() {
        let r = run(
            "create_clock -name clk -period 10 [get_ports clk]\n\
             set_false_path -from [get_cells {cnt[0]}] -to [get_pins {cnt[1]/D}]\n\
             set_multicycle_path 2 -setup -to [get_cells {cnt[*]}]\n",
            &MAP,
        );
        assert!(r.complete(), "{r:?}");
        let c = carried(&r);
        for want in [("cnt[0]", "_10_"), ("cnt[1]", "_11_"), ("cnt[1]/D", "_11_/D"), ("clk", "clk")] {
            assert!(c.contains(&(want.0.to_string(), want.1.to_string())), "{want:?} not in {c:?}");
        }
    }

    /// A pattern that names nothing (a misspelled register) is reported with its query; it is not
    /// dropped and not guessed.
    #[test]
    fn a_misspelled_object_is_reported() {
        let r = run("set_false_path -to [get_cells {cnt_q[0]}]\n", &MAP);
        let unknown: Vec<(&str, Option<&str>)> = r.unknown().map(|x| (x.object.accessor.as_str(), x.object.pattern.as_deref())).collect();
        assert_eq!(unknown, [("get_cells", Some("cnt_q[0]"))]);
        assert!(!r.complete());
    }

    /// Not carried, and why: a register with no map entry (`register-not-proved`), a net and a
    /// combinational cell (`combinational`: no proved match exists for them), a map entry naming
    /// a cell the implementation lacks.
    #[test]
    fn what_is_not_carried_says_why() {
        let reason = |r: &RtlMap| -> Vec<(String, &'static str)> {
            let mut v: Vec<(String, &'static str)> = r.uncarried().map(|(_, m)| (m.name.clone(), m.reason.expect("an uncarried member has a reason"))).collect();
            v.sort();
            v
        };
        let r = run("set_false_path -to [get_cells {cnt[1]}]\nset_false_path -through [get_nets nx]\nset_disable_timing [get_cells u3]\n", &MAP[..1]);
        assert_eq!(
            reason(&r),
            [("cnt[1]".to_string(), "register-not-proved"), ("nx".to_string(), "combinational"), ("u3".to_string(), "combinational")]
        );
        let r = run("set_false_path -to [get_cells {cnt[0]}]\n", &[("cnt[0]", "_99_")]);
        assert_eq!(reason(&r), [("cnt[0]".to_string(), "mapped-cell-not-in-revised")]);
    }

    /// The query decides the kind: `get_ports` names no register and `get_cells` no port, so a
    /// register's name under `get_ports` names nothing.
    #[test]
    fn the_query_decides_what_a_pattern_can_name() {
        let r = run("set_input_delay 1 -clock clk [get_ports {cnt[0]}]\ncreate_clock -name clk -period 10 [get_ports clk]\n", &MAP);
        assert_eq!(r.unknown().count(), 1, "{r:?}");
        let r = run("set_false_path -to [get_cells a]\n", &MAP);
        assert_eq!(r.unknown().count(), 1, "{r:?}");
    }

    /// A flattened name keeps its hierarchy separator: the register `blk/cnt[0]` has the pin
    /// `blk/cnt[0]/D`. The pin is the part after the LAST `/`, so the register is found and the
    /// pin carried as the mapped cell's pin, as written (no alias).
    #[test]
    fn a_pin_is_split_at_the_last_separator() {
        let elab = "module t(clk,a,y);\ninput clk,a; output y;\nwire \\blk/cnt[0] ;\n\
          DFF u1(.CK(clk),.D(a),.Q(\\blk/cnt[0] ));\nINV u2(.A(\\blk/cnt[0] ),.Y(y));\nendmodule\n";
        let imp = "module t(clk,a,y);\ninput clk,a; output y;\nwire q;\nDFF _10_(.CK(clk),.D(a),.Q(q));\nINV _6_(.A(q),.Y(y));\nendmodule\n";
        let r = run_on(elab, imp, "set_false_path -to [get_pins {blk/cnt[0]/D}]\n", &[("blk/cnt[0]", "_10_")]);
        assert!(r.complete(), "{r:?}");
        assert_eq!(carried(&r), [("blk/cnt[0]/D".to_string(), "_10_/D".to_string())]);
        // matched as written: not an RTL alias (no separator rule)
        assert_eq!(r.resolved[0].alias, Alias::None, "{r:?}");
    }

    /// A port is carried as itself only when the implementation has it (`port-not-in-revised`
    /// otherwise): a renamed port is a finding, not a silent carry.
    #[test]
    fn a_port_the_implementation_lacks_is_not_carried() {
        let imp = IMPL.replace("(clk,a,y)", "(clk,a_i,y)").replace("input clk,a;", "input clk,a_i;").replace(".A(a)", ".A(a_i)");
        let r = run_on(ELAB, &imp, "set_input_delay 1 -clock clk [get_ports a]\ncreate_clock -name clk -period 10 [get_ports clk]\n", &MAP);
        let why: Vec<(&str, Option<&str>)> = r.uncarried().map(|(_, m)| (m.name.as_str(), m.reason)).collect();
        assert_eq!(why, [("a", Some("port-not-in-revised"))]);
    }

    /// A `[list …]` is its items, each resolved by its own query: a register and a port in one
    /// list are both carried.
    #[test]
    fn a_list_is_its_items() {
        let r = run("set_false_path -from [list [get_cells {cnt[0]}] [get_ports a]]\n", &MAP);
        assert!(r.complete(), "{r:?}");
        assert_eq!(carried(&r), [("a".to_string(), "a".to_string()), ("cnt[0]".to_string(), "_10_".to_string())]);
    }

    /// An `all_*` query with no pattern names every object of its kind (`all_registers`: both
    /// registers, carried). Any other query with no pattern is not read as `*`: `[get_nets]` is
    /// reported as not resolved, not expanded into every net.
    #[test]
    fn only_an_all_query_names_everything() {
        let r = run("set_false_path -to [all_registers]\n", &MAP);
        assert!(r.complete(), "{r:?}");
        assert_eq!(carried(&r).len(), 2);
        let r = run("set_load 1 [get_nets]\n", &MAP);
        assert_eq!((r.resolved.len(), r.not_resolved.len()), (0, 1), "{r:?}");
    }

    /// A command the canonical write does not hold (here a query that needs the design to
    /// evaluate) is reported with why, never passed over: its objects were not read, so the
    /// file is not complete.
    #[test]
    fn a_command_not_written_is_reported() {
        let r = run("set_false_path -to [get_cells -of_objects [get_nets nx]]\n", &MAP);
        assert_eq!(r.not_written.len(), 1, "{r:?}");
        assert!(!r.complete());
    }

    /// A bare object list in an exception's `-from` / `-to` (no query) is read as names, as the
    /// reference reads it, and resolved: a misspelled one is reported, not skipped. A bare
    /// `-name` is a value, not an object.
    #[test]
    fn a_bare_option_value_is_resolved() {
        let r = run("create_clock -name clk -period 10 [get_ports clk]\nset_false_path -from {cnt[0]} -to {cnt[9]}\n", &MAP);
        let unknown: Vec<Option<&str>> = r.unknown().map(|x| x.object.pattern.as_deref()).collect();
        assert_eq!(unknown, [Some("cnt[9]")], "{r:?}");
        assert!(carried(&r).contains(&("cnt[0]".to_string(), "_10_".to_string())));
        // `cnt[0]` is also the register's output net; the bare name is the register only (a
        // net is tried only when no port, clock or register has the name), so nothing is left
        // uncarried
        assert_eq!(r.uncarried().count(), 0, "{r:?}");
    }

    /// A bare name is the first kind that has it: a whole register name is that register, and
    /// is read as `<register>/<pin>` only when no port, clock or register has the whole name.
    /// Here `a/D` is a register (its output net is `a/D`) and also the D pin of register `a`.
    #[test]
    fn a_bare_name_that_names_a_register_is_not_also_a_pin() {
        let elab = "module t(clk,x,y);\ninput clk,x; output y;\nwire a, \\a/D ;\n\
          DFF u1(.CK(clk),.D(x),.Q(a));\nDFF u2(.CK(clk),.D(a),.Q(\\a/D ));\nINV u3(.A(\\a/D ),.Y(y));\nendmodule\n";
        let imp = "module t(clk,x,y);\ninput clk,x; output y;\nwire q0,q1;\n\
          DFF _10_(.CK(clk),.D(x),.Q(q0));\nDFF _11_(.CK(clk),.D(q0),.Q(q1));\nINV _6_(.A(q1),.Y(y));\nendmodule\n";
        let r = run_on(elab, imp, "set_false_path -to {a/D}\n", &[("a", "_10_"), ("a/D", "_11_")]);
        assert!(r.complete(), "{r:?}");
        assert_eq!(carried(&r), [("a/D".to_string(), "_11_".to_string())]);
    }

    /// A `set_clock_groups -group` member that names no clock is left out by the reader (as the
    /// reference does, with a warning), so the canonical write never shows it; it is still a
    /// pattern that names nothing, and is reported. A stray option the reader ignores is not.
    #[test]
    fn a_clock_group_member_naming_no_clock_is_reported() {
        let r = run(
            "create_clock -name clk -period 10 [get_ports clk]\nset_clock_groups -asynchronous -group clk -group {clkx} -include_generated_clocks\n",
            &MAP,
        );
        let unknown: Vec<(&str, Option<&str>)> = r.unknown().map(|x| (x.object.accessor.as_str(), x.object.pattern.as_deref())).collect();
        assert_eq!(unknown, [("get_clocks", Some("clkx"))], "{r:?}");
    }

    fn ranges(r: &[(&str, i64, i64)]) -> DeclaredRanges {
        r.iter().map(|(n, l, rt)| (n.to_string(), (*l, *rt))).collect()
    }

    /// The rule, as the reference elaborates: bit `i` of `[left:right]` is elaborated bit
    /// `|i - right|`. `[11:8]`: 8 -> 0, 11 -> 3. `[8:11]` (ascending): 11 -> 0, 8 -> 3. A bus
    /// range is translated end by end, a pin pattern by its instance part. `[3:0]` changes
    /// nothing; a glob index, or a vector the ranges do not list, is left as written.
    #[test]
    fn a_bit_index_is_counted_from_the_declared_right_bound() {
        let rg = ranges(&[("r", 11, 8), ("s", 8, 11), ("z", 3, 0), ("u_sub.t", 5, 4)]);
        let to = |p: &str| match translate_bits(p, &rg) {
            BitIndex::Translated { to, .. } => to,
            other => format!("{other:?}"),
        };
        assert_eq!(to("r[8]"), "r[0]");
        assert_eq!(to("r[11]"), "r[3]");
        assert_eq!(to("s[11]"), "s[0]");
        assert_eq!(to("s[8]"), "s[3]");
        assert_eq!(to("r[9:8]"), "r[1:0]");
        assert_eq!(to("r[10]/D"), "r[2]/D");
        assert_eq!(to("u_sub.t[5]"), "u_sub.t[1]");
        for same in ["z[2]", "r[*]", "r", "w[9]", "r/D"] {
            assert_eq!(translate_bits(same, &rg), BitIndex::Same, "{same}");
        }
    }

    /// An index outside the declared range names nothing and is reported with the range, on
    /// either side of it; it is never moved to a bit that exists (`r[12]` of `[11:8]` is not
    /// `r[4]`, and `r[7]` is not `r[1]`).
    #[test]
    fn an_index_outside_the_declared_range_is_reported() {
        let rg = ranges(&[("r", 11, 8), ("z", 3, 0)]);
        for p in ["r[12]", "r[7]", "r[7:8]", "z[4]", "r[12]/D"] {
            assert!(matches!(translate_bits(p, &rg), BitIndex::OutOfRange { .. }), "{p}");
        }
    }

    /// A non-zero-based register vector and port (`logic [9:8] r`, `output [9:8] q`): the
    /// elaborated netlist numbers both from 0. `r[9]` names elaborated `r[1]` and is carried
    /// through the map; `q[8]` names port `q[0]`; each says it was translated. Without the
    /// declared ranges `r[9]` and `q[8]` name nothing: RTL indices the netlist does not have.
    #[test]
    fn rtl_bits_of_a_register_and_a_port_are_translated_by_the_declared_range() {
        let elab = "module t(clk,a,q);\ninput clk; input [1:0] a; output [1:0] q;\nwire [1:0] r;\n\
          DFF u1(.CK(clk),.D(a[0]),.Q(r[0]));\nDFF u2(.CK(clk),.D(a[1]),.Q(r[1]));\n\
          INV u3(.A(r[0]),.Y(q[0]));\nINV u4(.A(r[1]),.Y(q[1]));\nendmodule\n";
        let imp = "module t(clk,a,q);\ninput clk; input [1:0] a; output [1:0] q;\nwire q0, q1;\n\
          DFF _10_(.CK(clk),.D(a[0]),.Q(q0));\nDFF _11_(.CK(clk),.D(a[1]),.Q(q1));\n\
          INV _5_(.A(q0),.Y(q[0]));\nINV _6_(.A(q1),.Y(q[1]));\nendmodule\n";
        let sdc = "create_clock -name clk -period 10 [get_ports clk]\nset_false_path -to [get_cells {r[9]}]\nset_output_delay 1 -clock clk [get_ports {q[8]}]\n";
        let go = |rg: &DeclaredRanges| {
            let lib = Lib::parse(LIB).unwrap();
            let m: Vec<(String, String)> = [("r[0]", "_10_"), ("r[1]", "_11_")].iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
            rtl_map(&Sdc::parse(sdc).unwrap(), &crate::netlist::parse(elab).unwrap(), &lib, rg, &m, Some(&crate::netlist::parse(imp).unwrap()))
        };
        let r = go(&ranges(&[("r", 9, 8), ("q", 9, 8)]));
        assert!(r.complete(), "{r:?}");
        assert_eq!(carried(&r), [("clk".to_string(), "clk".to_string()), ("q[0]".to_string(), "q[0]".to_string()), ("r[1]".to_string(), "_11_".to_string())]);
        let translated: Vec<(&str, &str)> = r
            .resolved
            .iter()
            .filter_map(|x| match &x.bits {
                BitIndex::Translated { to, .. } => Some((x.object.pattern.as_deref().unwrap(), to.as_str())),
                _ => None,
            })
            .collect();
        assert_eq!(translated, [("q[8]", "q[0]"), ("r[9]", "r[1]")]);
        let r = go(&DeclaredRanges::new());
        assert_eq!(r.unknown().count(), 2, "{r:?}");
    }

    /// Out of range inside the whole flow: `r[10]` of `[9:8]` names nothing even though the
    /// elaborated netlist could have a bit that number would land on.
    #[test]
    fn an_out_of_range_bit_names_nothing_in_the_map() {
        let r = run_on(ELAB, IMPL, "set_false_path -to [get_cells {cnt[2]}]\n", &MAP);
        assert_eq!(r.unknown().count(), 1);
        let lib = Lib::parse(LIB).unwrap();
        let m: Vec<(String, String)> = MAP.iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
        let r = rtl_map(
            &Sdc::parse("set_false_path -to [get_cells {cnt[2]}]\n").unwrap(),
            &crate::netlist::parse(ELAB).unwrap(),
            &lib,
            &ranges(&[("cnt", 2, 1)]),
            &m,
            None,
        );
        assert!(r.complete(), "[2:1]: cnt[2] is elaborated cnt[1] {r:?}");
        let r = rtl_map(
            &Sdc::parse("set_false_path -to [get_cells {cnt[0]}]\n").unwrap(),
            &crate::netlist::parse(ELAB).unwrap(),
            &lib,
            &ranges(&[("cnt", 2, 1)]),
            &m,
            None,
        );
        assert_eq!(r.unknown().map(|x| x.bits.clone()).collect::<Vec<_>>(), [BitIndex::OutOfRange { declared: (2, 1) }]);
    }

    fn aliased(r: &RtlMap) -> Vec<(String, Vec<&'static str>, Vec<String>)> {
        r.resolved
            .iter()
            .filter_map(|x| match &x.alias {
                Alias::Matched { rules } => Some((x.object.pattern.clone().unwrap(), rules.clone(), x.members.iter().map(|m| m.name.clone()).collect())),
                _ => None,
            })
            .collect()
    }

    /// The RTL forms of a register's name (decided 2026-11-08): `/` for `.`, the register name
    /// with `_reg`, and a trailing output pin (`/Q`) all name the elaborated register `cnt[1]`;
    /// a `get_pins` pattern keeps its pin; a glob names every register it matches. Each says
    /// which rules matched it, and every one is carried through the map.
    #[test]
    fn rtl_forms_of_a_register_name_match_the_elaborated_one() {
        let r = run(
            "set_false_path -to [get_cells {cnt_reg[0]}]\n\
             set_false_path -from [get_cells {cnt_reg[1]/Q}]\n\
             set_false_path -to [get_pins {cnt_reg[1]/D}]\n\
             set_multicycle_path 2 -setup -to [get_cells {cnt_reg[*]}]\n",
            &MAP,
        );
        assert!(r.complete(), "{r:?}");
        let mut a = aliased(&r);
        a.sort();
        assert_eq!(
            a,
            [
                ("cnt_reg[*]".to_string(), vec!["reg-suffix"], vec!["cnt[0]".to_string(), "cnt[1]".to_string()]),
                ("cnt_reg[0]".to_string(), vec!["reg-suffix"], vec!["cnt[0]".to_string()]),
                ("cnt_reg[1]/D".to_string(), vec!["reg-suffix"], vec!["cnt[1]/D".to_string()]),
                ("cnt_reg[1]/Q".to_string(), vec!["output-pin", "reg-suffix"], vec!["cnt[1]".to_string()]),
            ]
        );
    }

    /// A hierarchy written with `/` names the elaborated `.` path; with the declared range, the
    /// RTL bit index is translated after the alias (`u_sub/t_reg[5]` of `[5:4]` is `u_sub.t[1]`).
    #[test]
    fn a_slash_hierarchy_and_a_declared_range_together() {
        let elab = "module t(clk,a,y);\ninput clk,a; output y;\nwire \\u_sub.t[0] , \\u_sub.t[1] ;\n\
          DFF u1(.CK(clk),.D(a),.Q(\\u_sub.t[0] ));\nDFF u2(.CK(clk),.D(\\u_sub.t[0] ),.Q(\\u_sub.t[1] ));\nINV u3(.A(\\u_sub.t[1] ),.Y(y));\nendmodule\n";
        let lib = Lib::parse(LIB).unwrap();
        let m: Vec<(String, String)> = vec![("u_sub.t[0]".into(), "_10_".into()), ("u_sub.t[1]".into(), "_11_".into())];
        let rg: DeclaredRanges = [("u_sub.t".to_string(), (5, 4))].into_iter().collect();
        let sdc = Sdc::parse("set_false_path -to [get_cells {u_sub/t_reg[5]}]\n").unwrap();
        let r = rtl_map(&sdc, &crate::netlist::parse(elab).unwrap(), &lib, &rg, &m, None);
        assert!(r.complete(), "{r:?}");
        assert_eq!(aliased(&r), [("u_sub/t_reg[5]".to_string(), vec!["separator", "reg-suffix"], vec!["u_sub.t[1]".to_string()])]);
        assert_eq!(r.resolved[0].bits, BitIndex::Translated { to: "u_sub.t[1]".into(), declared: (5, 4) });
        let sdc = Sdc::parse("set_false_path -to [get_cells {u_sub/t_reg[6]}]\n").unwrap();
        let r = rtl_map(&sdc, &crate::netlist::parse(elab).unwrap(), &lib, &rg, &m, None);
        assert!(matches!(r.resolved[0].bits, BitIndex::OutOfRange { .. }) && r.resolved[0].members.is_empty(), "{r:?}");
    }

    /// Two elaborated registers that share an alias (`a.t[0]` and `a.t_reg[0]`): an RTL form
    /// that could mean either is reported ambiguous with both, and names neither. A name that
    /// matches as written is not read as an alias at all.
    #[test]
    fn an_alias_two_registers_share_is_ambiguous() {
        let elab = "module t(clk,a,y);\ninput clk,a; output y;\nwire \\a.t[0] , \\a.t_reg[0] ;\n\
          DFF u1(.CK(clk),.D(a),.Q(\\a.t[0] ));\nDFF u2(.CK(clk),.D(\\a.t[0] ),.Q(\\a.t_reg[0] ));\nINV u3(.A(\\a.t_reg[0] ),.Y(y));\nendmodule\n";
        let lib = Lib::parse(LIB).unwrap();
        let go = |sdc: &str| rtl_map(&Sdc::parse(sdc).unwrap(), &crate::netlist::parse(elab).unwrap(), &lib, &DeclaredRanges::new(), &[], None);
        let r = go("set_false_path -to [get_cells {a/t[0]}]\n");
        assert_eq!(r.resolved[0].alias, Alias::Ambiguous { candidates: vec!["a.t[0]".into(), "a.t_reg[0]".into()] });
        assert!(r.resolved[0].members.is_empty() && !r.complete());
        let r = go("set_false_path -to [get_cells {a.t_reg[0]}]\n");
        assert_eq!((r.resolved[0].alias.clone(), r.resolved[0].members.len()), (Alias::None, 1));
    }

    /// Only a query that can name a register is read as an alias: `get_ports cnt_reg[0]` and
    /// `get_nets cnt_reg[0]` name nothing.
    #[test]
    fn a_port_or_net_query_is_not_read_as_a_register_alias() {
        let r = run("set_input_delay 1 -clock clk [get_ports {cnt_reg[0]}]\ncreate_clock -name clk -period 10 [get_ports clk]\nset_false_path -through [get_nets {cnt_reg[0]}]\n", &MAP);
        assert_eq!(r.unknown().count(), 2, "{r:?}");
        assert!(aliased(&r).is_empty());
    }

    /// The `_reg` is on whichever side has it: an RTL `a/t[0]` names the elaborated `a.t_reg[0]`.
    /// The `_reg` is the register name's, after the last separator, even under a generate
    /// element whose name has an index (`g[0].u_in.t_reg[4]`).
    #[test]
    fn the_reg_suffix_is_matched_on_either_side_and_only_on_the_last_name() {
        let lib = Lib::parse(LIB).unwrap();
        let elab = "module t(clk,a,y);\ninput clk,a; output y;\nwire \\a.t_reg[0] , \\g[0].u_in.t[0] ;\n\
          DFF u1(.CK(clk),.D(a),.Q(\\a.t_reg[0] ));\nDFF u2(.CK(clk),.D(\\a.t_reg[0] ),.Q(\\g[0].u_in.t[0] ));\nINV u3(.A(\\g[0].u_in.t[0] ),.Y(y));\nendmodule\n";
        let rg: DeclaredRanges = [("g[0].u_in.t".to_string(), (5, 4))].into_iter().collect();
        let r = rtl_map(
            &Sdc::parse("set_false_path -to [get_cells {a/t[0]}]\nset_false_path -from [get_cells {g[0]/u_in/t_reg[4]}]\n").unwrap(),
            &crate::netlist::parse(elab).unwrap(),
            &lib,
            &rg,
            &[],
            None,
        );
        let mut a = aliased(&r);
        a.sort();
        assert_eq!(
            a,
            [
                ("a/t[0]".to_string(), vec!["separator", "reg-suffix"], vec!["a.t_reg[0]".to_string()]),
                ("g[0]/u_in/t_reg[4]".to_string(), vec!["separator", "reg-suffix"], vec!["g[0].u_in.t[0]".to_string()]),
            ],
            "{r:?}"
        );
    }

    /// A `get_pins` pattern names pins only: `get_pins {cnt_reg[0]}`, with no pin, names
    /// nothing, as it does as written.
    #[test]
    fn a_pin_query_without_a_pin_is_not_read_as_a_register() {
        let r = run("set_false_path -to [get_pins {cnt_reg[0]}]\n", &MAP);
        assert_eq!(r.unknown().count(), 1, "{r:?}");
    }
}

