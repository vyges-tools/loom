// SPDX-License-Identifier: Apache-2.0
//! SDC diff — [`diff`]: two constraint sets compared as CONSTRAINTS, not as text.
//!
//! **How.** Each side is written by [`Sdc::write`], whose output is the canonical form the
//! reference round trip scores: one command per line, values in their shortest exact form. Each
//! line becomes one or more [`Record`]s:
//!
//! - **The key** is what the constraint is ON: the command, its flags, the options that name what
//!   it applies to (`-clock`, `-from`, `-to`, `-through`, `-name`, …) and its objects. Each object
//!   list is compared as a set: the accessor, its flags, and the names sorted.
//! - **The values** are what it SAYS: the numbers (`-period`, the delay, the multiplier), the
//!   other valued options (`-lib_cell`, `-waveform`) and `-comment`.
//! - **Per-object commands are split** into one record per object: `set_input_delay 1 -clock c
//!   [get_ports {a b}]` equals the same command written once for `a` and once for `b`.
//! - **A later record with the same key replaces the earlier**, as SDC's own merge does (the
//!   later `set_input_delay` without `-add_delay`, the later `set_load`). The replaced record is
//!   listed in [`SdcDiff::superseded`] and does not take part in the comparison.
//!
//! Two records with one key and different values are one [`ChangeKind::Changed`]. A key on one
//! side only is `Added` or `Removed`. Each change says whether it [`Direction::Loosens`] or
//! [`Direction::Tightens`] the constraints, by a per-command rule (a longer clock period
//! loosens; a new false path loosens; a larger I/O `-max` delay tightens). `Unknown` means no rule
//! decides it; a gate treats `Unknown` as it treats `Loosens`.
//!
//! **Objects are compared as written**, after the optional rename map: a pattern is not expanded
//! against a design here. Read both sides against the same design (`Sdc::load_with`) for what
//! the reader resolves; queries it evaluates are then compared by the objects they resolved to.
//!
//! **Commands the model does not carry are compared by their text.** The reader keeps each one's
//! text with its variables substituted ([`Sdc::unmodelled_text`]): an `if` block it did not
//! evaluate, a command it reads past. The same text on both sides, against the same design, is
//! the same constraint; a text on one side only is an `Added` or `Removed` change of direction
//! `Unknown`, keyed `text: …`.
//!
//! **What cannot be compared is said.** A command the writer did not write and whose text is not
//! known (a rejected command; any unmodelled one when a rename map is given, since a raw text
//! cannot be renamed faithfully) is listed in [`SdcDiff::not_compared`], and the two sets are
//! never [`SdcDiff::equivalent`] while either list is non-empty.

use crate::sdc::{tokenize, Objects, Sdc, MODELLED_BUT_LISTED};
use crate::sdc_write::WriteOpts;

/// Whether a change makes the constraints looser (fewer or easier checks) or tighter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Loosens,
    Tightens,
    /// No effect on what is checked (a `-comment`).
    Neutral,
    /// No rule decides it (a waveform, a latency, a mixed `-min`/`-max` change).
    Unknown,
}

impl Direction {
    pub fn as_str(self) -> &'static str {
        match self {
            Direction::Loosens => "loosens",
            Direction::Tightens => "tightens",
            Direction::Neutral => "neutral",
            Direction::Unknown => "unknown",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChangeKind {
    Added,
    Removed,
    Changed,
}

impl ChangeKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ChangeKind::Added => "added",
            ChangeKind::Removed => "removed",
            ChangeKind::Changed => "changed",
        }
    }
}

/// One constraint: see the module documentation for what is key and what is value.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub command: String,
    /// The canonical identity: the command, its identity options and objects, in a fixed order.
    pub key: String,
    /// `(field, value)`, in a fixed order. A positional number is field `value`.
    pub values: Vec<(String, String)>,
    /// The line the record came from (after the rename map, before the split).
    pub text: String,
}

/// One field of a [`ChangeKind::Changed`] record.
#[derive(Debug, Clone, PartialEq)]
pub struct FieldChange {
    pub field: String,
    pub a: Option<String>,
    pub b: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Change {
    pub kind: ChangeKind,
    pub command: String,
    pub key: String,
    /// The changed fields (`Changed` only).
    pub fields: Vec<FieldChange>,
    /// The record's line on each side, where it exists.
    pub a: Option<String>,
    pub b: Option<String>,
    pub direction: Direction,
}

/// Options for [`diff`].
#[derive(Debug, Clone, Default)]
pub struct DiffOpts {
    /// Object names on side A renamed before comparing, `(from, to)`: a name equal to `from`,
    /// or starting with `from/` (a pin of a renamed instance), takes `to` in its place. RTL names
    /// against netlist names, or one delivery's hierarchy against the next.
    pub rename: Vec<(String, String)>,
}

/// The result of [`diff`].
#[must_use]
#[derive(Debug, Clone, Default)]
pub struct SdcDiff {
    /// Each change, in side B's canonical order, then each removal in side A's.
    pub changes: Vec<Change>,
    /// Per side, each command that could not be compared (it was not written; with why).
    pub not_compared: [Vec<String>; 2],
    /// Per side, each record a later record with the same key replaced, as its line.
    pub superseded: [Vec<String>; 2],
    /// The two sides declare different `set_units`: values are compared as written.
    pub units_differ: bool,
}

impl SdcDiff {
    /// No change, and nothing left uncompared on either side.
    pub fn equivalent(&self) -> bool {
        self.changes.is_empty() && self.not_compared.iter().all(|v| v.is_empty())
    }
}

/// Valued options that name what a constraint applies to (part of the key); every other valued
/// option is a value.
const IDENTITY_OPTIONS: &[&str] = &[
    "-clock",
    "-from",
    "-to",
    "-through",
    "-rise_from",
    "-rise_to",
    "-fall_from",
    "-fall_to",
    "-rise_through",
    "-fall_through",
    "-reference_pin",
    "-source",
    "-master_clock",
    "-group",
    "-name",
];

/// Flags that change neither what a constraint applies to nor what it says, kept as values so a
/// change is still seen: `-add_delay` (whether the command added or replaced, the merge above
/// decides) and `-add` on a clock.
const VALUE_FLAGS: &[&str] = &["-add_delay", "-add"];

/// Commands whose effect is per object, split into one record per object name.
const PER_OBJECT: &[&str] = &[
    "set_input_delay",
    "set_output_delay",
    "set_load",
    "set_input_transition",
    "set_driving_cell",
    "set_drive",
    "set_case_analysis",
    "set_logic_zero",
    "set_logic_one",
    "set_logic_dc",
    "set_propagated_clock",
    "set_clock_latency",
    "set_clock_transition",
    "set_clock_uncertainty",
    "set_max_transition",
    "set_max_fanout",
    "set_max_capacitance",
    "set_disable_timing",
    "set_dont_touch",
    "set_ideal_network",
];

/// Commands that REMOVE checks: adding one loosens.
const RELAXING: &[&str] = &[
    "set_false_path",
    "set_multicycle_path",
    "set_clock_groups",
    "set_disable_timing",
    "set_case_analysis",
    "set_logic_zero",
    "set_logic_one",
    "set_logic_dc",
    "set_max_time_borrow",
];

/// Commands that ADD or strengthen checks: adding one tightens.
const CONSTRAINING: &[&str] = &[
    "create_clock",
    "create_generated_clock",
    "set_input_delay",
    "set_output_delay",
    "set_clock_uncertainty",
    "set_load",
    "set_input_transition",
    "set_driving_cell",
    "set_drive",
    "set_max_transition",
    "set_max_fanout",
    "set_max_capacitance",
    "set_min_pulse_width",
    "set_data_check",
];

/// `name` after the rename map.
fn renamed(name: &str, rename: &[(String, String)]) -> String {
    for (from, to) in rename {
        if name == from {
            return to.clone();
        }
        if let Some(rest) = name.strip_prefix(from.as_str()).and_then(|r| r.strip_prefix('/')) {
            return format!("{to}/{rest}");
        }
    }
    name.to_string()
}

/// An object list in canonical form: `accessor flags keys {names sorted}`, a `[list …]` as its
/// items' canonical forms sorted.
fn canon_objects(o: &Objects, rename: &[(String, String)]) -> String {
    if !o.items.is_empty() {
        let mut items: Vec<String> = o.items.iter().map(|i| canon_objects(i, rename)).collect();
        items.sort();
        items.dedup();
        return format!("[{}]", items.join(" "));
    }
    let mut flags = o.flags.clone();
    flags.sort();
    let mut keys: Vec<String> = o.keys.iter().map(|(k, v)| format!("{k} {v}")).collect();
    keys.sort();
    let mut names: Vec<String> = o.names.iter().map(|n| renamed(n, rename)).collect();
    names.sort();
    names.dedup();
    let acc = if o.accessor.is_empty() || o.accessor == "list" { "objects" } else { o.accessor.as_str() };
    let mut s = acc.to_string();
    for f in flags.iter().chain(&keys) {
        s += " ";
        s += f;
    }
    format!("{s} {{{}}}", names.join(" "))
}

fn is_number(t: &str) -> bool {
    t.parse::<f64>().is_ok()
}

fn is_number_list(t: &str) -> bool {
    let mut any = false;
    for w in t.split_whitespace() {
        if !is_number(w) {
            return false;
        }
        any = true;
    }
    any
}

/// A canonical value: a number in its shortest exact form, a number list likewise.
fn canon_value(t: &str) -> String {
    if is_number_list(t) {
        t.split_whitespace().map(|w| format!("{}", w.parse::<f64>().unwrap_or(0.0))).collect::<Vec<_>>().join(" ")
    } else {
        t.to_string()
    }
}

/// One written line as its records.
fn line_records(line: &str, rename: &[(String, String)]) -> Vec<Record> {
    let toks = tokenize(line);
    let Some(cmd) = toks.first().cloned() else { return Vec::new() };
    let valued: Vec<&str> = crate::sdc_flags::options(&cmd).map(|(k, _)| k).unwrap_or_default();
    let mut key_parts: Vec<String> = Vec::new();
    let mut values: Vec<(String, String)> = Vec::new();
    // the positional object lists, kept apart: a per-object command splits on them
    let mut positional_objects: Vec<Objects> = Vec::new();
    let mut positional_values = 0usize;
    let mut k = 1;
    while k < toks.len() {
        let t = &toks[k];
        let is_flag = t.starts_with('-') && !is_number(t);
        if is_flag && (valued.contains(&t.as_str()) || IDENTITY_OPTIONS.contains(&t.as_str()) || t == "-comment") && k + 1 < toks.len() {
            let v = &toks[k + 1];
            let v_canon = if v.starts_with('[') {
                canon_objects(&Objects::of(v), rename)
            } else if IDENTITY_OPTIONS.contains(&t.as_str()) {
                // a bare name list after an identity option (`-group {a b}`, `-name c`)
                let mut names: Vec<String> = v.split_whitespace().map(|n| renamed(n, rename)).collect();
                names.sort();
                names.join(" ")
            } else {
                canon_value(v)
            };
            if IDENTITY_OPTIONS.contains(&t.as_str()) {
                key_parts.push(format!("{t} {v_canon}"));
            } else {
                values.push((t.clone(), v_canon));
            }
            k += 2;
            continue;
        }
        if is_flag {
            if VALUE_FLAGS.contains(&t.as_str()) {
                values.push((t.clone(), "1".into()));
            } else {
                key_parts.push(t.clone());
            }
        } else if t.starts_with('[') {
            positional_objects.push(Objects::of(t));
        } else if is_number_list(t) {
            let field = if positional_values == 0 { "value".to_string() } else { format!("value{}", positional_values + 1) };
            positional_values += 1;
            values.push((field, canon_value(t)));
        } else {
            // a bare word or brace list in an object position: names as written
            positional_objects.push(Objects { accessor: String::new(), names: t.split_whitespace().map(str::to_string).collect(), flags: Vec::new(), keys: Vec::new(), items: Vec::new() });
        }
        k += 1;
    }
    // a clock is identified by its name alone: its sources and period are what it says
    if cmd == "create_clock" || cmd == "create_generated_clock" {
        let (name, rest): (Vec<String>, Vec<String>) = key_parts.into_iter().partition(|p| p.starts_with("-name "));
        for r in rest {
            match r.split_once(' ') {
                Some((f, v)) => values.push((f.to_string(), v.to_string())),
                None => values.push((r, "1".into())),
            }
        }
        let srcs: Vec<String> = positional_objects.iter().map(|o| canon_objects(o, rename)).collect();
        if !srcs.is_empty() {
            values.push(("sources".into(), srcs.join(" ")));
        }
        key_parts = name;
        if key_parts.is_empty() {
            // an unnamed clock is named after its first source
            key_parts.push(format!("-name {}", srcs.first().cloned().unwrap_or_default()));
        }
        positional_objects.clear();
    }
    // `set_clock_groups -name n`: the name is the key and the groups are what it says
    if cmd == "set_clock_groups" && key_parts.iter().any(|p| p.starts_with("-name ")) {
        let (name, rest): (Vec<String>, Vec<String>) = key_parts.into_iter().partition(|p| p.starts_with("-name "));
        let mut groups: Vec<String> = rest.iter().filter(|p| p.starts_with("-group ")).cloned().collect();
        groups.sort();
        values.push(("groups".into(), groups.join(" ")));
        for r in rest.into_iter().filter(|p| !p.starts_with("-group ")) {
            values.push((r, "1".into()));
        }
        key_parts = name;
    }
    key_parts.sort();
    values.sort();
    let base = format!("{cmd} {}", key_parts.join(" "));
    let split = PER_OBJECT.contains(&cmd.as_str()) && positional_objects.len() == 1 && positional_objects[0].items.is_empty() && positional_objects[0].names.len() > 1;
    if split {
        let o = &positional_objects[0];
        return o
            .names
            .iter()
            .map(|n| {
                let one = Objects { accessor: o.accessor.clone(), names: vec![n.clone()], flags: o.flags.clone(), keys: o.keys.clone(), items: Vec::new() };
                Record { command: cmd.clone(), key: format!("{base} {}", canon_objects(&one, rename)).trim().to_string(), values: values.clone(), text: line.to_string() }
            })
            .collect();
    }
    let mut objs: Vec<String> = positional_objects.iter().map(|o| canon_objects(o, rename)).collect();
    objs.sort();
    vec![Record { command: cmd, key: format!("{base} {}", objs.join(" ")).trim().to_string(), values, text: line.to_string() }]
}

/// A side's records, after the merge (a later record with the same key replaces the earlier),
/// with what it superseded and what could not be compared.
#[must_use]
pub fn records(s: &Sdc, opts: &DiffOpts) -> (Vec<Record>, Vec<String>, Vec<String>) {
    let w = s.write(&WriteOpts { header: Some(String::new()), ..WriteOpts::default() });
    let mut out: Vec<Record> = Vec::new();
    let mut superseded = Vec::new();
    for line in w.text.lines() {
        let l = line.trim();
        if l.is_empty() || l.starts_with('#') {
            continue;
        }
        for r in line_records(l, &opts.rename) {
            if let Some(at) = out.iter().position(|o| o.key == r.key) {
                superseded.push(out.remove(at).text);
            }
            out.push(r);
        }
    }
    (out, superseded, w.not_written)
}

/// The direction of one changed field.
fn field_direction(command: &str, key: &str, field: &str, a: Option<&str>, b: Option<&str>) -> Direction {
    if field == "-comment" {
        return Direction::Neutral;
    }
    let num = |v: Option<&str>| v.and_then(|v| v.parse::<f64>().ok());
    let (Some(x), Some(y)) = (num(a), num(b)) else { return Direction::Unknown };
    if x == y {
        return Direction::Neutral;
    }
    let larger = y > x;
    let has = |f: &str| key.split_whitespace().any(|w| w == f);
    // `true` when a larger value loosens
    let larger_loosens = match (command, field) {
        ("create_clock", "-period") | ("create_generated_clock", "-divide_by") => true,
        ("create_generated_clock", "-multiply_by") => false,
        ("set_input_delay" | "set_output_delay", "value") => {
            if has("-min") && !has("-max") {
                true
            } else if has("-max") && !has("-min") {
                false
            } else {
                return Direction::Unknown; // one value for both: tighter setup, looser hold
            }
        }
        ("set_multicycle_path", "value") => true,
        ("set_max_delay", "value") => true,
        ("set_min_delay", "value") => false,
        ("set_clock_uncertainty", "value") => false,
        ("set_load" | "set_input_transition" | "set_clock_transition", "value") => false,
        ("set_max_transition" | "set_max_fanout" | "set_max_capacitance", "value") => true,
        ("set_timing_derate", "value") => {
            if has("-early") && !has("-late") {
                true
            } else if has("-late") && !has("-early") {
                false
            } else {
                return Direction::Unknown;
            }
        }
        _ => return Direction::Unknown,
    };
    if larger == larger_loosens {
        Direction::Loosens
    } else {
        Direction::Tightens
    }
}

/// The direction of a whole change: one field's, or `Unknown` when the fields disagree.
fn combine(ds: impl IntoIterator<Item = Direction>) -> Direction {
    let mut out = Direction::Neutral;
    for d in ds {
        out = match (out, d) {
            (o, Direction::Neutral) => o,
            (Direction::Neutral, d) => d,
            (o, d) if o == d => o,
            _ => Direction::Unknown,
        };
    }
    out
}

fn presence_direction(command: &str, added: bool) -> Direction {
    let d = if RELAXING.contains(&command) {
        Direction::Loosens
    } else if CONSTRAINING.contains(&command) {
        Direction::Tightens
    } else {
        return Direction::Unknown;
    };
    match (added, d) {
        (true, d) => d,
        (false, Direction::Loosens) => Direction::Tightens,
        (false, _) => Direction::Loosens,
    }
}

/// The prefix the writer gives a command it did not write because the model does not carry it.
const NOT_MODELLED: &str = "not modelled: ";

/// A side's unmodelled commands as `(command, text)`, whitespace collapsed, sorted, and what is
/// left of `not_written` once they are taken out. `None` for the texts when they cannot be
/// compared (a rename map, or a model whose texts do not cover every entry).
fn unmodelled(s: &Sdc, not_written: Vec<String>, renaming: bool) -> (Option<Vec<(String, String)>>, Vec<String>) {
    if renaming || s.unmodelled_text.len() != s.ignored.len() {
        return (None, not_written);
    }
    let rest: Vec<String> = not_written.into_iter().filter(|n| !n.starts_with(NOT_MODELLED)).collect();
    // the writer's own filter: what it lists as not modelled
    let mut texts: Vec<(String, String)> = s
        .unmodelled_text
        .iter()
        .filter(|(c, _)| !MODELLED_BUT_LISTED.contains(&c.as_str()) && !s.kept_options.contains(c))
        .map(|(c, t)| (c.clone(), t.split_whitespace().collect::<Vec<_>>().join(" ")))
        .collect();
    texts.sort();
    (Some(texts), rest)
}

/// `a` against `b`: see the module documentation.
pub fn diff(a: &Sdc, b: &Sdc, opts: &DiffOpts) -> SdcDiff {
    let (ra, sa, na) = records(a, opts);
    let (rb, sb, nb) = records(b, &DiffOpts::default());
    let renaming = !opts.rename.is_empty();
    let (ua, na) = unmodelled(a, na, renaming);
    let (ub, nb) = unmodelled(b, nb, renaming);
    let mut changes = Vec::new();
    for r in &rb {
        match ra.iter().find(|x| x.key == r.key) {
            None => changes.push(Change {
                kind: ChangeKind::Added,
                command: r.command.clone(),
                key: r.key.clone(),
                fields: Vec::new(),
                a: None,
                b: Some(r.text.clone()),
                direction: presence_direction(&r.command, true),
            }),
            Some(x) if x.values != r.values => {
                let mut names: Vec<&String> = x.values.iter().chain(&r.values).map(|(f, _)| f).collect();
                names.sort();
                names.dedup();
                let get = |vs: &[(String, String)], f: &str| vs.iter().find(|(n, _)| n == f).map(|(_, v)| v.clone());
                let fields: Vec<FieldChange> = names
                    .into_iter()
                    .filter_map(|f| {
                        let (va, vb) = (get(&x.values, f), get(&r.values, f));
                        (va != vb).then(|| FieldChange { field: f.clone(), a: va, b: vb })
                    })
                    .collect();
                let direction = combine(fields.iter().map(|fc| field_direction(&r.command, &r.key, &fc.field, fc.a.as_deref(), fc.b.as_deref())));
                changes.push(Change { kind: ChangeKind::Changed, command: r.command.clone(), key: r.key.clone(), fields, a: Some(x.text.clone()), b: Some(r.text.clone()), direction });
            }
            Some(_) => {}
        }
    }
    for x in &ra {
        if !rb.iter().any(|r| r.key == x.key) {
            changes.push(Change {
                kind: ChangeKind::Removed,
                command: x.command.clone(),
                key: x.key.clone(),
                fields: Vec::new(),
                a: Some(x.text.clone()),
                b: None,
                direction: presence_direction(&x.command, false),
            });
        }
    }
    let (na, nb) = match (ua, ub) {
        (Some(ua), Some(ub)) => {
            changes.extend(text_changes(&ua, &ub));
            (na, nb)
        }
        // texts on one side only: the other side's unmodelled commands cannot be matched, so
        // both sides' stay uncompared
        (ua, ub) => (with_texts_back(na, ua), with_texts_back(nb, ub)),
    };
    let units_differ = a.declared_units != b.declared_units;
    SdcDiff { changes, not_compared: [na, nb], superseded: [sa, sb], units_differ }
}

/// The unmodelled commands put back as not compared, in the writer's words.
fn with_texts_back(mut not_written: Vec<String>, texts: Option<Vec<(String, String)>>) -> Vec<String> {
    for (c, _) in texts.unwrap_or_default() {
        not_written.push(format!("{NOT_MODELLED}{c}"));
    }
    not_written
}

/// Two sorted lists of unmodelled texts, as changes: each text on one side more often than on the
/// other is an `Added` or `Removed` of direction `Unknown`.
fn text_changes(a: &[(String, String)], b: &[(String, String)]) -> Vec<Change> {
    let count = |v: &[(String, String)], x: &(String, String)| v.iter().filter(|y| *y == x).count();
    let mut out = Vec::new();
    let mut seen: Vec<&(String, String)> = Vec::new();
    for x in a.iter().chain(b) {
        if seen.contains(&x) {
            continue;
        }
        seen.push(x);
        let (ca, cb) = (count(a, x), count(b, x));
        let (kind, n) = if cb > ca { (ChangeKind::Added, cb - ca) } else { (ChangeKind::Removed, ca - cb) };
        for _ in 0..n {
            let text = Some(x.1.clone());
            out.push(Change {
                kind,
                command: x.0.clone(),
                key: format!("text: {}", x.1),
                fields: Vec::new(),
                a: if kind == ChangeKind::Removed { text.clone() } else { None },
                b: if kind == ChangeKind::Added { text } else { None },
                direction: Direction::Unknown,
            });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(t: &str) -> Sdc {
        Sdc::parse(t).expect("parse")
    }

    fn d(a: &str, b: &str) -> SdcDiff {
        diff(&p(a), &p(b), &DiffOpts::default())
    }

    const BASE: &str = "\
create_clock -name clk -period 10 [get_ports clk]
create_clock -name vclk -period 10
set_input_delay 2 -clock vclk -max [get_ports {a b}]
set_input_delay 0.5 -clock vclk -min [get_ports {a b}]
set_output_delay 3 -clock vclk [get_ports y]
set_clock_uncertainty 0.25 [get_clocks clk]
set_false_path -from [get_ports rst_n]
set_multicycle_path 2 -setup -from [get_cells u_cfg/r*] -to [get_cells u_dp/q*]
set_load 0.01 [get_ports y]
set_case_analysis 0 [get_ports te]
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks vclk]
";

    /// A file against itself, and against its own write-back, is equivalent: the baseline every
    /// other answer here rests on.
    #[test]
    fn a_file_equals_itself_and_its_write_back() {
        assert!(d(BASE, BASE).equivalent(), "{:?}", d(BASE, BASE).changes);
        let back = p(BASE).write(&WriteOpts::default()).text;
        let r = d(BASE, &back);
        assert!(r.equivalent(), "{:?}", r.changes);
    }

    /// Command order across kinds, and name order inside a list, is not a difference.
    #[test]
    fn order_is_not_a_difference() {
        let mut lines: Vec<&str> = BASE.lines().collect();
        lines.reverse();
        // clocks must still come first for the reader to know them
        lines.sort_by_key(|l| !l.starts_with("create_clock"));
        let swapped = lines.join("\n").replace("{a b}", "{b a}");
        let r = d(BASE, &swapped);
        assert!(r.equivalent(), "{:?}", r.changes);
    }

    /// Names inside a list that is NOT split per object (an exception's `-from`) are a set too.
    #[test]
    fn name_order_in_an_exception_is_not_a_difference() {
        let a = "create_clock -name clk -period 10 [get_ports clk]\nset_false_path -from [get_ports {rst_n te}]\n";
        let r = d(a, &a.replace("{rst_n te}", "{te rst_n}"));
        assert!(r.equivalent(), "{:?}", r.changes);
    }

    /// One command on two ports is the same constraint as one command per port.
    #[test]
    fn a_per_object_command_is_split_per_object() {
        let split = BASE.replace(
            "set_input_delay 2 -clock vclk -max [get_ports {a b}]",
            "set_input_delay 2 -clock vclk -max [get_ports a]\nset_input_delay 2 -clock vclk -max [get_ports b]",
        );
        let r = d(BASE, &split);
        assert!(r.equivalent(), "{:?}", r.changes);
        // and a change to one of the two ports is ONE change
        let one = BASE.replace(
            "set_input_delay 2 -clock vclk -max [get_ports {a b}]",
            "set_input_delay 2 -clock vclk -max [get_ports a]\nset_input_delay 2.5 -clock vclk -max [get_ports b]",
        );
        let r = d(BASE, &one);
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!(r.changes[0].kind, ChangeKind::Changed);
        assert_eq!(r.changes[0].direction, Direction::Tightens, "a larger -max input delay leaves less of the cycle");
    }

    /// SDC's merge: a later command on the same key replaces the earlier one. A file that says
    /// a value and then overrides it equals a file that says only the override.
    #[test]
    fn a_later_command_on_the_same_key_replaces_the_earlier() {
        let twice = format!("{BASE}set_load 0.02 [get_ports y]\n");
        let once = BASE.replace("set_load 0.01 [get_ports y]", "set_load 0.02 [get_ports y]");
        let r = d(&twice, &once);
        assert!(r.equivalent(), "{:?}", r.changes);
        assert_eq!(r.superseded[0].len(), 1, "the overridden set_load is reported");
    }

    #[test]
    fn a_longer_period_loosens_and_a_shorter_tightens() {
        let r = d(BASE, &BASE.replace("-name clk -period 10", "-name clk -period 12"));
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!(r.changes[0].kind, ChangeKind::Changed);
        assert_eq!(r.changes[0].fields[0].field, "-period");
        assert_eq!(r.changes[0].direction, Direction::Loosens);
        let r = d(BASE, &BASE.replace("-name clk -period 10", "-name clk -period 8"));
        assert_eq!(r.changes[0].direction, Direction::Tightens);
    }

    #[test]
    fn a_new_false_path_loosens_and_a_removed_one_tightens() {
        let more = format!("{BASE}set_false_path -to [get_ports y]\n");
        let r = d(BASE, &more);
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!((r.changes[0].kind, r.changes[0].direction), (ChangeKind::Added, Direction::Loosens));
        let r = d(&more, BASE);
        assert_eq!((r.changes[0].kind, r.changes[0].direction), (ChangeKind::Removed, Direction::Tightens));
    }

    #[test]
    fn a_larger_multicycle_loosens() {
        let r = d(BASE, &BASE.replace("set_multicycle_path 2", "set_multicycle_path 3"));
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!(r.changes[0].direction, Direction::Loosens);
    }

    /// One value for both `-min` and `-max`: larger is tighter for setup and looser for hold,
    /// so no rule decides it.
    #[test]
    fn a_delay_for_both_min_and_max_is_unknown() {
        let r = d(BASE, &BASE.replace("set_output_delay 3", "set_output_delay 4"));
        assert_eq!(r.changes[0].direction, Direction::Unknown);
    }

    #[test]
    fn a_removed_uncertainty_loosens() {
        let fewer = BASE.replace("set_clock_uncertainty 0.25 [get_clocks clk]\n", "");
        let r = d(BASE, &fewer);
        assert_eq!((r.changes[0].kind, r.changes[0].direction), (ChangeKind::Removed, Direction::Loosens));
    }

    /// A clock is its name: a new source or period is a change to that clock, not a new clock.
    #[test]
    fn a_clock_is_keyed_by_its_name() {
        let r = d(BASE, &BASE.replace("[get_ports clk]\n", "[get_ports clk2]\n"));
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!(r.changes[0].kind, ChangeKind::Changed);
        assert_eq!(r.changes[0].fields[0].field, "sources");
    }

    /// A comment says why, not what: a change to it is neutral, and still a change.
    #[test]
    fn a_comment_change_is_neutral() {
        let a = "create_clock -name clk -period 10 [get_ports clk]\nset_false_path -from [get_ports rst_n] -comment {async reset}\n";
        let b = a.replace("async reset", "synchronized reset");
        let r = d(a, &b);
        assert_eq!(r.changes.len(), 1, "{:?}", r.changes);
        assert_eq!(r.changes[0].direction, Direction::Neutral);
    }

    /// The rename map: the same constraints on renamed objects are equivalent with the map and
    /// differ without it.
    #[test]
    fn a_rename_map_makes_renamed_objects_equal() {
        let rtl = "create_clock -name clk -period 10 [get_ports clk]\nset_multicycle_path 2 -setup -from [get_pins u_cfg/q_reg/Q] -to [get_cells u_dp]\n";
        let net = rtl.replace("u_cfg/q_reg", "u17").replace("u_dp", "u_dp_0");
        let opts = DiffOpts { rename: vec![("u_cfg/q_reg".into(), "u17".into()), ("u_dp".into(), "u_dp_0".into())] };
        assert!(diff(&p(rtl), &p(&net), &opts).equivalent());
        assert!(!d(rtl, &net).equivalent());
    }

    /// An `if` the reader cannot evaluate without a design is compared by its text: the same
    /// block on both sides is not a difference, a changed block is, and with a rename map (which
    /// cannot rename raw text) it is not compared at all.
    #[test]
    fn unmodelled_commands_are_compared_by_their_text() {
        let a = "create_clock -name clk -period 10 [get_ports clk]\nset te [get_ports -quiet te]\nif {[llength $te] > 0} {\n  set_case_analysis 0 $te\n}\n";
        let same = d(a, a);
        assert!(same.equivalent(), "{:?} {:?}", same.changes, same.not_compared);
        let b = a.replace("set_case_analysis 0", "set_case_analysis 1");
        let r = d(a, &b);
        assert_eq!(r.changes.len(), 2, "one text removed, one added: {:?}", r.changes);
        assert!(r.changes.iter().all(|c| c.direction == Direction::Unknown && c.key.starts_with("text: if")), "{:?}", r.changes);
        let opts = DiffOpts { rename: vec![("x".into(), "y".into())] };
        let renamed = diff(&p(a), &p(a), &opts);
        assert!(!renamed.equivalent());
        assert!(renamed.not_compared[0].iter().any(|n| n == "not modelled: if"), "{:?}", renamed.not_compared);
    }

    /// Indentation and line layout inside an unmodelled block are not a difference.
    #[test]
    fn an_unmodelled_block_reindented_is_the_same() {
        let a = "create_clock -name clk -period 10 [get_ports clk]\nset te [get_ports -quiet te]\nif {[llength $te] > 0} {\n  set_case_analysis 0 $te\n}\n";
        let b = a.replace("  set_case_analysis", "        set_case_analysis").replace("> 0} {", ">   0}   {");
        let r = d(a, &b);
        assert!(r.equivalent(), "{:?} {:?}", r.changes, r.not_compared);
    }

    /// The text is read with variables substituted: the same `if` under a different variable
    /// value is a different constraint.
    #[test]
    fn an_unmodelled_text_is_compared_after_substitution() {
        let a = "set v 0\ncreate_clock -name clk -period 10 [get_ports clk]\nset te [get_ports -quiet te]\nif {[llength $te] > 0} {\n  set_case_analysis $v $te\n}\n";
        let b = a.replace("set v 0", "set v 1");
        assert!(!d(a, &b).equivalent());
    }

    /// The reader keeps one text per `ignored` entry, index for index.
    #[test]
    fn every_ignored_entry_has_its_text() {
        let s = p("create_clock -name clk -period 10 [get_ports clk]\nset_driving_cell -lib_cell BUF [get_ports a]\nset te [get_ports -quiet te]\nif {[llength $te] > 0} { set_false_path -from $te }\nset_dont_touch [get_cells u1]\n");
        assert_eq!(s.unmodelled_text.len(), s.ignored.len(), "{:?} {:?}", s.ignored, s.unmodelled_text);
        for ((entry, text), ign) in s.unmodelled_text.iter().zip(&s.ignored) {
            assert_eq!(entry, ign);
            assert!(!text.is_empty(), "{entry}");
        }
    }

    #[test]
    fn different_units_are_flagged() {
        let a = "set_units -time ns\ncreate_clock -name clk -period 10 [get_ports clk]\n";
        let b = "set_units -time ps\ncreate_clock -name clk -period 10 [get_ports clk]\n";
        assert!(d(a, b).units_differ);
        assert!(!d(a, a).units_differ);
    }

    /// Teeth: every number on every line of the base file, nudged on its own, is exactly one
    /// change. A diff that misses a field, or that a nudge splits into several, fails here.
    #[test]
    fn every_number_nudged_is_exactly_one_change() {
        let mut tried = 0;
        for (i, line) in BASE.lines().enumerate() {
            let words: Vec<&str> = line.split(' ').collect();
            for (j, w) in words.iter().enumerate() {
                let Ok(v) = w.parse::<f64>() else { continue };
                let mut nudged = words.clone();
                let nv = format!("{}", v + 1.0);
                nudged[j] = &nv;
                let mut lines: Vec<String> = BASE.lines().map(str::to_string).collect();
                lines[i] = nudged.join(" ");
                let r = d(BASE, &(lines.join("\n") + "\n"));
                // a per-object line nudged changes each of its objects
                let objects = if line.contains("{a b}") { 2 } else { 1 };
                assert_eq!(r.changes.len(), objects, "line {i} word {j}: {:?}", r.changes);
                assert!(r.changes.iter().all(|c| c.kind == ChangeKind::Changed), "line {i}: {:?}", r.changes);
                tried += 1;
            }
        }
        assert!(tried >= 9, "the base file has a number on each value-bearing line ({tried})");
    }
}
