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
//!     `ref_name` `cell` `is_hierarchical`; a pin's `name` `lib_pin_name` `full_name` `direction`
//!     `is_port` `is_hierarchical`; a net's `name` `full_name`.
//!
//! What this cannot answer it says so ([`Err`]) and the command is not applied: `-regexp`, a
//! property the reference has but this view does not carry (timing, Liberty attributes), and a
//! property the reference does not know (an error there too).

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
        SdcDesign { ports, insts, pins }
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
    let keys = crate::sdc_flags::FLAGS.iter().find(|(n, _, _)| *n == cmd).map_or(&[][..], |(_, k, _)| *k);
    let (mut filter, mut of, mut regexp, mut hier) = (None, None, false, false);
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
            "-hierarchical" => hier = true,
            w if w.starts_with('-') => {}
            w => patterns.extend(w.split_whitespace().map(String::from)),
        }
        k += 1;
    }
    if regexp {
        return Err("-regexp is not evaluated".into());
    }
    if cmd == "get_clocks" {
        return if filter.is_some() || of.is_some() { Err("get_clocks with -filter / -of_objects is not evaluated".into()) } else { Ok(Objs::Clocks) };
    }
    if patterns.is_empty() {
        patterns.push("*".into());
    }
    let base = match of {
        Some(of) => of_objects(cmd, &of, d)?,
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
        "get_cells" => Objs::Cells(pick(d.insts.len(), &|k| d.insts[k].0.clone(), &glob)),
        "get_pins" => Objs::Pins(pick(d.pins.len(), &|k| d.pin_name(k), &glob)),
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

/// A property's value as the filter compares it, `Ok(None)` for an object kind that has the
/// property in the reference but not in this view (cannot evaluate), `Err` for one the
/// reference does not know.
fn property(o: &Objs, k: usize, prop: &str, d: &SdcDesign) -> Result<(String, bool), String> {
    let s = |v: &str| Ok((v.to_string(), false));
    let b = |v: bool| Ok((if v { "1" } else { "0" }.to_string(), true));
    match o {
        Objs::Ports(v) => {
            let (name, dir) = &d.ports[v[k]];
            match prop {
                "name" | "full_name" => s(name),
                "direction" | "port_direction" => s(dir),
                "liberty_port" | "activity" => Err(format!("port property {prop} is not evaluated")),
                p if p.starts_with("slack_") || p.starts_with("slew_") => Err(format!("port property {prop} is not evaluated")),
                p => Err(format!("port objects do not have a {p} property")),
            }
        }
        Objs::Cells(v) => {
            let (name, cell) = &d.insts[v[k]];
            match prop {
                "name" | "full_name" => s(name),
                "ref_name" | "cell" => s(cell),
                "is_hierarchical" => b(false),
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
            _ => return Err(format!("-filter parsing failed at '{rest}'")),
        };
        out.push(t);
        i += n;
    }
    Ok(out)
}

/// The reference's shunting-yard: precedence `!` over `&&` over `||`; `defined`/`undefined` and
/// `!` go on the operator stack.
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
                loop {
                    match ops.pop() {
                        Some(FTok::LParen) => break,
                        Some(x) => out.push(x),
                        None => return Err("-filter extraneous )".into()),
                    }
                }
            }
        }
    }
    while let Some(t) = ops.pop() {
        if t == FTok::LParen {
            return Err("-filter unmatched (".into());
        }
        out.push(t);
    }
    Ok(out)
}

fn apply_filter(expr: &str, base: Objs, d: &SdcDesign) -> Result<Objs, String> {
    use std::collections::BTreeSet;
    let n = match &base {
        Objs::Ports(v) | Objs::Cells(v) | Objs::Pins(v) => v.len(),
        Objs::Nets(v) => v.len(),
        Objs::Clocks => 0,
    };
    let all: BTreeSet<usize> = (0..n).collect();
    let mut stack: Vec<BTreeSet<usize>> = Vec::new();
    for t in postfix(lex(expr)?)? {
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
                let (Some(a), Some(b)) = (stack.pop(), stack.pop()) else { return Err("-filter operator needs two operands".into()) };
                stack.push(if t == FTok::And { a.intersection(&b).copied().collect() } else { a.union(&b).copied().collect() });
            }
            FTok::Not => {
                let Some(a) = stack.pop() else { return Err("-filter NOT missing operand".into()) };
                stack.push(all.difference(&a).copied().collect());
            }
            FTok::LParen | FTok::RParen => {}
        }
    }
    if stack.len() != 1 {
        return Err("-filter expression is empty or evaluated to several sets".into());
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

    /// What this cannot answer it says so: an unknown property (an error in the reference too),
    /// one the view does not carry, and -regexp.
    #[test]
    fn what_cannot_be_evaluated_says_why() {
        assert!(q("get_ports -filter is_clock *").unwrap_err().contains("do not have a is_clock"));
        assert!(q("get_cells -filter is_buffer *").unwrap_err().contains("not evaluated"));
        assert!(q("get_ports -regexp {y[1-3]}").is_err());
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
