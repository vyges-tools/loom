//! The commands that TAKE A CONSTRAINT BACK — `delete_clock`, `delete_generated_clock` and the
//! `unset_*` family — applied to the model, as the reference applies them.
//!
//! Each follows the reference's Tcl proc (its argument rules) and the `Sdc` method that proc
//! reaches (what it removes, keyed how). What the reference removes is gone from the model, so
//! the writer writes the state the reference writes, and a consumer times what the reference
//! times.
//!
//! ⛔ **What only the design can decide is refused, never guessed.** A removal that would split
//! a wildcard (`set_case_analysis 0 [get_pins u1/*]`, then `unset_case_analysis u1/A`), or that
//! names one port of an `[all_inputs]` delay, needs the design's object list to express. It is
//! refused — listed in [`Sdc::rejected`] with why, and the model left as it was — not applied to
//! the wrong objects. (`unset_disable_timing` and `unset_disable_clock_gating_check` are not
//! here: they are kept in file order with their `set_` forms; see [`super::ArcCmd`].)

use std::collections::BTreeSet;

use super::{
    glob, object_matches, reduce_exception_objects, resolve_objs, ClockGroups, ClockUncertainty, Exception,
    IoDelay, Objects, PathDelayCmd, Sdc, SdcClock, SENTINELS,
};

/// The commands this module applies.
pub(super) const UNSET_COMMANDS: &[&str] = &[
    "delete_clock",
    "delete_generated_clock",
    "unset_case_analysis",
    "unset_clock_groups",
    "unset_clock_latency",
    "unset_clock_transition",
    "unset_clock_uncertainty",
    "unset_input_delay",
    "unset_output_delay",
    "unset_path_exceptions",
    "unset_propagated_clock",
    "unset_timing_derate",
];

/// A command's words read as the reference's `parse_key_args` reads them: keys with their
/// values, flags, and what is left (positionals). A key given twice keeps its LAST value, as a Tcl
/// array does.
struct Args {
    keys: Vec<(String, String)>,
    flags: Vec<String>,
    pos: Vec<String>,
}

impl Args {
    fn key(&self, k: &str) -> Option<&str> {
        self.keys.iter().rev().find(|(n, _)| n == k).map(|(_, v)| v.as_str())
    }
    fn has(&self, f: &str) -> bool {
        self.flags.iter().any(|x| x == f)
    }
    /// The first of `keys` given, in that order — the reference's `if … elseif …` over them.
    fn first_key(&self, keys: &[&'static str]) -> Option<(&'static str, &str)> {
        keys.iter().find_map(|k| self.key(k).map(|v| (*k, v)))
    }
}

fn key_args(toks: &[String], keys: &[&str], flags: &[&str]) -> Result<Args, String> {
    let mut a = Args { keys: Vec::new(), flags: Vec::new(), pos: Vec::new() };
    let mut k = 1;
    while k < toks.len() {
        let t = toks[k].as_str();
        if keys.contains(&t) {
            let Some(v) = toks.get(k + 1) else { return Err(format!("{t} without a value")) };
            a.keys.push((t.to_string(), v.clone()));
            k += 2;
            continue;
        }
        if flags.contains(&t) {
            a.flags.push(t.to_string());
        } else {
            a.pos.push(toks[k].clone());
        }
        k += 1;
    }
    Ok(a)
}

/// `check_argc_eq1`: exactly one word left.
fn one(a: &Args) -> Result<&str, String> {
    match a.pos.as_slice() {
        [o] => Ok(o.as_str()),
        p => Err(format!("{} words where the reference takes one (wrong # args)", p.len())),
    }
}

/// `check_argc_eq0`.
fn none(a: &Args) -> Result<(), String> {
    match a.pos.len() {
        0 => Ok(()),
        n => Err(format!("{n} words where the reference takes none (wrong # args)")),
    }
}

/// A proc with ONE fixed parameter (`proc unset_case_analysis { pins }`): exactly one word, an
/// option included — Tcl's "wrong # args" otherwise.
fn fixed_one(toks: &[String]) -> Result<&str, String> {
    match toks {
        [_, o] => Ok(o.as_str()),
        _ => Err(format!("{} words where the reference takes one (wrong # args)", toks.len() - 1)),
    }
}

/// Does an object a removal names (`target`) hit an object a constraint names (`held`)?
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Hit {
    Yes,
    No,
    /// They overlap in a way only the design's object list can settle.
    Unknown,
}

fn wild(n: &str) -> bool {
    n.contains(['*', '?']) || SENTINELS.contains(&n)
}

/// `target` against `held`, both names as written. The same name is a hit; a pattern hits the
/// concrete names it matches. A HELD pattern (or `all_*` collection) that is not named exactly
/// is split by a removal of some of its objects — unknown without the design.
fn name_hit(target: &str, held: &str) -> Hit {
    if target == held {
        return Hit::Yes;
    }
    if wild(held) {
        let overlaps = SENTINELS.contains(&held) || SENTINELS.contains(&target) || wild(target) || glob(held, target);
        return if overlaps { Hit::Unknown } else { Hit::No };
    }
    if SENTINELS.contains(&target) {
        return Hit::Unknown;
    }
    if target.contains(['*', '?']) {
        return if object_matches(target, held) { Hit::Yes } else { Hit::No };
    }
    Hit::No
}

/// A removal's objects, ready to test held names against: exact names by hash, patterns (and
/// `all_*` collections) by [`name_hit`] — so a removal of N objects from a constraint on M is
/// N + M, not N × M, unless they are patterns.
struct Targets<'t> {
    exact: std::collections::HashSet<(bool, &'t str)>,
    /// Every target, for a held name that is itself a pattern, and the pattern targets.
    all: Vec<(bool, &'t str)>,
    patterns: Vec<(bool, &'t str)>,
}

impl<'t> Targets<'t> {
    fn new(it: impl Iterator<Item = (bool, &'t str)>) -> Self {
        let all: Vec<(bool, &str)> = it.collect();
        let patterns = all.iter().copied().filter(|(_, t)| wild(t)).collect();
        Targets { exact: all.iter().copied().collect(), all, patterns }
    }

    /// Does the removal hit `held` (a clock name when `clock`)?
    fn hit(&self, clock: bool, held: &str) -> Result<Hit, String> {
        if self.exact.contains(&(clock, held)) {
            return Ok(Hit::Yes);
        }
        let candidates = if wild(held) { &self.all } else { &self.patterns };
        let mut hit = Hit::No;
        for (_, t) in candidates.iter().filter(|(c, _)| *c == clock) {
            match name_hit(t, held) {
                Hit::Yes => hit = Hit::Yes,
                Hit::Unknown => return Err(format!("`{t}` names part of `{held}`, which only the design can split")),
                Hit::No => {}
            }
        }
        Ok(hit)
    }
}

/// What an object names: a clock, or a design object (pin, port, cell, net).
fn is_clock(accessor: &str, name: &str, clocks: &[SdcClock]) -> bool {
    match accessor {
        "get_clocks" | "all_clocks" => true,
        "" => clocks.iter().any(|c| c.name == name),
        _ => false,
    }
}

/// The removal's objects, each as `(is_clock, name)`, with `[all_clocks]` as the clocks now.
fn targets_of(tok: &str, clocks: &[SdcClock]) -> Vec<(bool, String)> {
    let o = Objects::of(tok);
    let mut out = Vec::new();
    fn walk(o: &Objects, clocks: &[SdcClock], out: &mut Vec<(bool, String)>) {
        if !o.items.is_empty() {
            for i in &o.items {
                walk(i, clocks, out);
            }
            return;
        }
        if o.accessor == "all_clocks" {
            out.extend(clocks.iter().map(|c| (true, c.name.clone())));
            return;
        }
        for n in &o.names {
            out.push((is_clock(&o.accessor, n, clocks), n.clone()));
        }
    }
    walk(&o, clocks, &mut out);
    out
}

/// `o` split into what the removal leaves and the single objects it takes, accessor kept. An
/// element of a `[list …]` left empty is dropped. `held_is_clock` says what `o`'s names are when
/// the caller knows (a clock command's bare names are clocks), else they are classified.
fn split_hits(o: &Objects, targets: &[(bool, String)], clocks: &[SdcClock], held_is_clock: Option<bool>) -> Result<(Objects, Vec<Objects>), String> {
    if !o.items.is_empty() {
        let (mut kept, mut taken) = (o.clone(), Vec::new());
        kept.items.clear();
        for i in &o.items {
            let (k, t) = split_hits(i, targets, clocks, held_is_clock)?;
            if !k.names.is_empty() {
                kept.items.push(k);
            }
            taken.extend(t);
        }
        kept.names = kept.items.iter().flat_map(|i| i.names.iter().cloned()).collect();
        return Ok((kept, taken));
    }
    let mut kept = Objects { names: Vec::new(), ..one_object(o, "") };
    let mut taken = Vec::new();
    let lookup = Targets::new(targets.iter().map(|(c, t)| (*c, t.as_str())));
    for n in &o.names {
        let clock = held_is_clock.unwrap_or_else(|| is_clock(&o.accessor, n, clocks));
        let hit = lookup.hit(clock, n)?;
        if hit == Hit::Yes {
            taken.push(one_object(o, n));
        } else {
            kept.names.push(n.clone());
        }
    }
    Ok((kept, taken))
}

/// Rise and fall (or max and min, or setup and hold) as two bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Both(bool, bool);

impl Both {
    const ALL: Both = Both(true, true);
    /// `parse_rise_fall_flags` / `parse_min_max_all_flags`: one flag alone selects it; both or
    /// neither select both.
    fn of(first: bool, second: bool) -> Both {
        if first != second { Both(first, second) } else { Both::ALL }
    }
    fn minus(self, o: Both) -> Both {
        Both(self.0 && !o.0, self.1 && !o.1)
    }
    fn any(self) -> bool {
        self.0 || self.1
    }
}

impl Sdc {
    /// Apply one take-back command, or say why the reference would not have (an error) or why
    /// this reader cannot (it needs the design). On `Err` the model is unchanged.
    pub(super) fn apply_unset(&mut self, toks: &[String]) -> Result<(), String> {
        match toks[0].as_str() {
            "delete_clock" | "delete_generated_clock" => self.delete_clock(toks),
            "unset_case_analysis" => self.unset_case_analysis(toks),
            "unset_clock_groups" => self.unset_clock_groups(toks),
            "unset_clock_latency" => self.unset_clock_latency(toks),
            "unset_clock_transition" => self.unset_clock_transition(toks),
            "unset_clock_uncertainty" => self.unset_clock_uncertainty(toks),
            "unset_input_delay" | "unset_output_delay" => self.unset_port_delay(toks),
            "unset_path_exceptions" => self.unset_path_exceptions(toks),
            "unset_propagated_clock" => self.unset_propagated_clock(toks),
            "unset_timing_derate" => self.unset_timing_derate(toks),
            other => Err(format!("{other} is not a take-back command")),
        }
    }

    /// The `ignored` entries of `kind`, dropped once nothing of that kind is left — a consumer
    /// that does not model it has nothing left to refuse.
    fn forget_if_none(&mut self, kind: &str, none_left: bool) {
        if none_left {
            self.ignored.retain(|c| c != kind);
        }
    }

    /// The clocks a `get_clocks_warn` of `tok` returns, in definition order: names and patterns
    /// against the clocks defined now (one that matches nothing only warns).
    fn clocks_named(&self, tok: &str) -> Vec<String> {
        let o = Objects::of(tok);
        if o.accessor == "all_clocks" {
            return self.clocks.iter().map(|c| c.name.clone()).collect();
        }
        let names = resolve_objs(tok);
        self.clocks.iter().filter(|c| names.iter().any(|n| *n == c.name || (n.contains(['*', '?']) && glob(n, &c.name)))).map(|c| c.name.clone()).collect()
    }

    /// `delete_clock` / `delete_generated_clock` (`remove_gclk_cmd`): `-all` (and no other word)
    /// or exactly one clock list; `delete_generated_clock` takes only the generated ones. Each
    /// goes through `Sdc::removeClock` — the clock, and everything that names it.
    fn delete_clock(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &[], &["-all"])?;
        let names = if a.has("-all") {
            none(&a)?;
            self.clocks.iter().map(|c| c.name.clone()).collect()
        } else {
            self.clocks_named(one(&a)?)
        };
        let generated_only = toks[0] == "delete_generated_clock";
        for name in names {
            let Some(i) = self.clocks.iter().position(|c| c.name == name) else { continue };
            if generated_only && self.clocks[i].generated.is_none() {
                continue;
            }
            self.clocks.remove(i);
            self.remove_clock_references(&name);
        }
        Ok(())
    }

    /// `unset_case_analysis pins` (`Sdc::removeCaseAnalysis`): each pin's case value is erased,
    /// whichever command set it. One fixed parameter — any other word count is Tcl's error.
    fn unset_case_analysis(&mut self, toks: &[String]) -> Result<(), String> {
        let targets: Vec<(bool, String)> = resolve_objs(fixed_one(toks)?).into_iter().map(|n| (false, n)).collect();
        let mut next = Vec::with_capacity(self.case_values.len());
        for c in &self.case_values {
            let (kept, _) = split_hits(&c.written, &targets, &self.clocks, Some(false))?;
            if !kept.names.is_empty() {
                let mut c = c.clone();
                c.objects = kept.names.clone();
                c.written = kept;
                next.push(c);
            }
        }
        self.case_values = next;
        let none_left = self.case_values.is_empty();
        self.forget_if_none("set_case_analysis", none_left);
        Ok(())
    }

    /// `unset_propagated_clock objects` (`removePropagatedClock`): a clock is ideal again, a pin
    /// leaves the propagated set. Its latency is NOT restored. One fixed parameter.
    fn unset_propagated_clock(&mut self, toks: &[String]) -> Result<(), String> {
        let targets = targets_of(fixed_one(toks)?, &self.clocks);
        let mut next = Vec::with_capacity(self.propagated.len());
        for p in &self.propagated {
            let (kept, _) = split_hits(p, &targets, &self.clocks, None)?;
            if !kept.names.is_empty() {
                next.push(kept);
            }
        }
        self.propagated = next;
        let none_left = self.propagated.is_empty();
        self.forget_if_none("set_propagated_clock", none_left);
        Ok(())
    }

    /// `unset_timing_derate` (`deleteDeratingFactors`): no arguments; every derate, global and
    /// scoped, is gone.
    fn unset_timing_derate(&mut self, toks: &[String]) -> Result<(), String> {
        if toks.len() != 1 {
            return Err(format!("{} words where the reference takes none (wrong # args)", toks.len() - 1));
        }
        self.derates.clear();
        self.late_derate = None;
        self.early_derate = None;
        self.forget_if_none("set_timing_derate", true);
        Ok(())
    }

    /// `unset_clock_transition clocks` (`Clock::removeSlew`): exactly one word (no options — the
    /// proc reads none); every slew of each clock goes, rise, fall, min and max.
    fn unset_clock_transition(&mut self, toks: &[String]) -> Result<(), String> {
        let a = Args { keys: Vec::new(), flags: Vec::new(), pos: toks[1..].to_vec() };
        let targets: Vec<(bool, String)> = self.clocks_named(one(&a)?).into_iter().map(|n| (true, n)).collect();
        let mut next = Vec::with_capacity(self.clock_transitions.len());
        for e in &self.clock_transitions {
            let Some(w) = &e.written else {
                next.push(e.clone());
                continue;
            };
            let (kept, _) = split_hits(w, &targets, &self.clocks, Some(true))?;
            if !kept.names.is_empty() {
                let mut e = e.clone();
                e.objects = kept.names.clone();
                e.written = Some(kept);
                next.push(e);
            }
        }
        self.clock_transitions = next;
        let none_left = self.clock_transitions.is_empty();
        self.forget_if_none("set_clock_transition", none_left);
        Ok(())
    }

    /// `unset_clock_latency [-source] [-clock c] objects` (`unset_clk_latency_cmd`): on a clock,
    /// the latency set on that clock; on a pin, the latency set on that pin FOR `-clock` (without
    /// `-clock`, only the one set without it) — `removeClockLatency(clk, pin)` /
    /// `removeClockInsertion` find one entry by exactly that pair. `-source` takes the source
    /// (insertion) latency, else the network latency; every edge and min/max of it goes. `-clock`
    /// with clock objects only warns (411). (Error 412, `-source` on a pin that is not a clock
    /// pin, needs the design and is not checked here.)
    fn unset_clock_latency(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &["-clock"], &["-source"])?;
        let targets = targets_of(one(&a)?, &self.clocks);
        // `get_clock_warn`: a -clock that names no clock is NULL
        let pin_clk = a.key("-clock").and_then(|c| self.clocks_named(c).into_iter().next());
        let source = a.has("-source");
        let clock_targets: Vec<(bool, String)> = targets.iter().filter(|(c, _)| *c).cloned().collect();
        let pin_targets: Vec<(bool, String)> = targets.iter().filter(|(c, _)| !*c).cloned().collect();
        let mut next = Vec::with_capacity(self.clock_latencies.len());
        for e in &self.clock_latencies {
            let is_source = e.flags.iter().any(|f| f == "-source");
            let Some(w) = e.written.as_ref().filter(|_| is_source == source) else {
                next.push(e.clone());
                continue;
            };
            let held_clk = e.clock.as_deref().and_then(|c| resolve_objs(c).into_iter().next());
            let (mut kept, _) = split_hits(w, &clock_targets, &self.clocks, None)?;
            if held_clk == pin_clk {
                kept = split_hits(&kept, &pin_targets, &self.clocks, None)?.0;
            }
            if !kept.names.is_empty() {
                let mut e = e.clone();
                e.objects = kept.names.clone();
                e.written = Some(kept);
                next.push(e);
            }
        }
        self.clock_latencies = next;
        self.clock_latency = self.clock_latencies.iter().map(|e| e.value).fold(0.0, f64::max);
        Ok(())
    }

    /// `unset_clock_groups -name names | -all` with exactly one of `-logically_exclusive`,
    /// `-physically_exclusive`, `-asynchronous` (`unset_clk_groups_cmd`; Errors 404–407). A named
    /// group of ANOTHER kind is left alone. An unnamed `set_clock_groups` is the reference's
    /// `group<n>`: the lowest such name not already taken when it was made.
    fn unset_clock_groups(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &["-name"], &["-logically_exclusive", "-physically_exclusive", "-asynchronous", "-all"])?;
        let all = a.has("-all");
        let names: Vec<String> = a.key("-name").map(resolve_objs).unwrap_or_default();
        if all && !names.is_empty() {
            return Err("the -all and -name options are mutually exclusive (Error 404)".into());
        }
        if !all && names.is_empty() {
            return Err("either -all or -name options must be specified (Error 405)".into());
        }
        let kinds: Vec<&str> =
            ["logically_exclusive", "physically_exclusive", "asynchronous"].into_iter().filter(|k| a.has(&format!("-{k}"))).collect();
        let kind = match kinds.as_slice() {
            [k] => *k,
            [] => return Err("one of -logically_exclusive, -physically_exclusive or -asynchronous is required (Error 406)".into()),
            _ => return Err("-logically_exclusive, -physically_exclusive and -asynchronous are mutually exclusive (Error 407)".into()),
        };
        let effective = clock_group_names(&self.clock_groups);
        let (mut kept, mut gone) = (Vec::new(), Vec::new());
        for (mut g, name) in self.clock_groups.drain(..).zip(effective) {
            if g.kind == kind && (all || names.contains(&name)) {
                gone.push(g);
            } else {
                // its name was given when it was made: with an earlier group gone, the same
                // command would be named differently if read again, so it keeps it explicitly
                g.name = Some(name);
                kept.push(g);
            }
        }
        self.clock_groups = kept;
        // `async_groups` is every command's groups in one list: take back the removed ones
        for g in gone.iter().flat_map(|g| g.groups.iter()) {
            if let Some(i) = self.async_groups.iter().position(|x| x == g) {
                self.async_groups.remove(i);
            }
        }
        Ok(())
    }

    /// `unset_input_delay` / `unset_output_delay [-clock c] [-clock_fall] [-rise|-fall]
    /// [-max|-min] [-reference_pin p] ports` (`unset_port_delay`): the delay of each port FOR
    /// that clock edge (`-clock` and `-clock_fall`; none for a delay set without `-clock`) loses
    /// the values of those edges and min/max (`removeValue`). `-reference_pin` is read and plays
    /// no part. An unknown `-clock` is NULL (`get_clock_warn` only warns): it then takes the
    /// delays set without a clock.
    ///
    /// The reference deletes an input delay left with no value; an output delay it keeps, empty —
    /// which constrains nothing and is not written, so here it goes too.
    fn unset_port_delay(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &["-clock", "-reference_pin"], &["-rise", "-fall", "-max", "-min", "-clock_fall"])?;
        let ports = resolve_objs(one(&a)?);
        let clk = a.key("-clock").and_then(|c| self.clocks_named(c).into_iter().next());
        let clk_fall = a.has("-clock_fall");
        let rf = Both::of(a.has("-rise"), a.has("-fall"));
        let mm = Both::of(a.has("-max"), a.has("-min"));
        let lookup = Targets::new(ports.iter().map(|p| (false, p.as_str())));
        let list = if toks[0] == "unset_input_delay" { &self.input_delays } else { &self.output_delays };
        let mut next: Vec<IoDelay> = Vec::with_capacity(list.len());
        for d in list {
            let has = |f: &str| d.flags.iter().any(|x| x == f);
            if d.clock != clk || has("-clock_fall") != clk_fall {
                next.push(d.clone());
                continue;
            }
            let held_rf = Both::of(has("-rise"), has("-fall"));
            let held_mm = Both::of(has("-max"), has("-min"));
            // the cells this command set that survive: per edge, the min/max left
            let left = [held_rf.0.then(|| held_mm.minus(if rf.0 { mm } else { Both(false, false) })), held_rf.1.then(|| held_mm.minus(if rf.1 { mm } else { Both(false, false) }))];
            if left == [held_rf.0.then_some(held_mm), held_rf.1.then_some(held_mm)] {
                next.push(d.clone());
                continue;
            }
            if d.default {
                if ports.iter().any(|p| SENTINELS.contains(&p.as_str())) && d.ports.is_empty() {
                    next.extend(port_delay_rest(d, Vec::new(), true, left));
                    continue;
                }
                return Err(format!("{} on ports of an `{}` delay, which only the design's port list can split", toks[0], d.accessor));
            }
            let (mut kept, mut taken) = (Vec::new(), Vec::new());
            for p in &d.ports {
                if lookup.hit(false, p)? == Hit::Yes { taken.push(p.clone()) } else { kept.push(p.clone()) }
            }
            if taken.is_empty() {
                next.push(d.clone());
                continue;
            }
            if !kept.is_empty() {
                next.push(IoDelay { ports: kept, ..d.clone() });
            }
            next.extend(port_delay_rest(d, taken, false, left));
        }
        if toks[0] == "unset_input_delay" {
            self.input_delays = next;
        } else {
            self.output_delays = next;
        }
        Ok(())
    }
}

/// What is left of delay `d` on `ports` after a removal, as commands: per edge (`left[0]` rise,
/// `left[1]` fall) the min/max that survive — one command when both edges keep the same, else one
/// per edge. The first keeps `d`'s own `-add_delay` (or its absence: it replaced other clocks'
/// delays when it ran); the second adds to it.
fn port_delay_rest(d: &IoDelay, ports: Vec<String>, default: bool, left: [Option<Both>; 2]) -> Vec<IoDelay> {
    let keep = |b: Option<Both>| b.filter(|b| b.any());
    let pieces: Vec<(Both, Both)> = match (keep(left[0]), keep(left[1])) {
        (Some(r), Some(f)) if r == f => vec![(Both::ALL, r)],
        (r, f) => [(Both(true, false), r), (Both(false, true), f)].into_iter().filter_map(|(e, m)| m.map(|m| (e, m))).collect(),
    };
    let base: Vec<String> = d.flags.iter().filter(|f| !matches!(f.as_str(), "-rise" | "-fall" | "-max" | "-min")).cloned().collect();
    let mut out = Vec::new();
    for (i, (edge, mm)) in pieces.into_iter().enumerate() {
        let mut flags = base.clone();
        if i > 0 && !flags.iter().any(|f| f == "-add_delay") {
            flags.push("-add_delay".into());
        }
        match edge {
            Both(true, false) => flags.push("-rise".into()),
            Both(false, true) => flags.push("-fall".into()),
            _ => {}
        }
        match mm {
            Both(true, false) => flags.push("-max".into()),
            Both(false, true) => flags.push("-min".into()),
            _ => {}
        }
        out.push(IoDelay { ports: ports.clone(), default, flags, ..d.clone() });
    }
    out
}

/// Each `set_clock_groups`' name as the reference holds it: `-name`, else `group<n>` — the lowest
/// `n` whose name no group holds when it is made (`makeClockGroupsName`; a named command replaces
/// a group of its name, which is already applied in the list).
fn clock_group_names(groups: &[ClockGroups]) -> Vec<String> {
    let mut taken: BTreeSet<String> = BTreeSet::new();
    let mut out = Vec::with_capacity(groups.len());
    for g in groups {
        let name = match &g.name {
            Some(n) => n.clone(),
            None => (1..).map(|i| format!("group{i}")).find(|n| !taken.contains(n)).expect("an unbounded range yields a free name"),
        };
        taken.insert(name.clone());
        out.push(name);
    }
    out
}

// ---- unset_clock_uncertainty ---------------------------------------------------------------

/// The edge a `-from` / `-to` form selects (`-rise_from` rise only, `-from` both).
fn edge_of(key: &str) -> Both {
    if key.starts_with("-rise") {
        Both(true, false)
    } else if key.starts_with("-fall") {
        Both(false, true)
    } else {
        Both::ALL
    }
}

const FROM_KEYS: [&str; 3] = ["-from", "-rise_from", "-fall_from"];
const TO_KEYS: [&str; 3] = ["-to", "-rise_to", "-fall_to"];

impl Sdc {
    /// `unset_clock_uncertainty` (`unset_clk_uncertainty_cmd`). `-setup` or `-hold` alone picks
    /// one check, else both. `-from` and `-to` go together (Error 423) and take no other word: the
    /// uncertainty between each pair of clocks loses those edges and checks. Otherwise exactly one
    /// word, clocks and pins, each losing those checks. ⚠️ The reference's guard against `-rise` /
    /// `-fall` on a single clock tests KEYS that are flags and never fires: they are accepted and
    /// change nothing.
    fn unset_clock_uncertainty(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &[&FROM_KEYS[..], &TO_KEYS[..]].concat(), &["-rise", "-fall", "-setup", "-hold"])?;
        let sides = Both::of(a.has("-setup"), a.has("-hold"));
        let next = match (a.first_key(&FROM_KEYS), a.first_key(&TO_KEYS)) {
            (Some(_), None) | (None, Some(_)) => return Err("-from/-to must be used together (Error 423)".into()),
            (Some((fk, fv)), Some((tk, tv))) => {
                none(&a)?;
                let pairs = (self.clocks_named(fv), edge_of(fk), self.clocks_named(tv), edge_of(tk));
                self.uncertainties_without_pairs(&pairs, sides)?
            }
            (None, None) => {
                let targets = targets_of(one(&a)?, &self.clocks);
                self.uncertainties_without_objects(&targets, sides)?
            }
        };
        self.uncertainties = next;
        self.refold_uncertainty();
        Ok(())
    }

    fn uncertainties_without_objects(&self, targets: &[(bool, String)], sides: Both) -> Result<Vec<ClockUncertainty>, String> {
        let mut next = Vec::with_capacity(self.uncertainties.len());
        for u in &self.uncertainties {
            if !(u.from.is_empty() && u.to.is_empty()) {
                next.push(u.clone());
                continue;
            }
            let left = Both(u.setup, u.hold).minus(sides);
            let (mut written, mut taken) = (Vec::new(), Vec::new());
            for w in &u.written {
                let (k, t) = split_hits(w, targets, &self.clocks, None)?;
                if !k.names.is_empty() {
                    written.push(k);
                }
                taken.extend(t);
            }
            if taken.is_empty() {
                next.push(u.clone());
                continue;
            }
            if !written.is_empty() {
                let objects = written.iter().flat_map(|w| w.names.iter().cloned()).collect();
                next.push(ClockUncertainty { objects, written, ..u.clone() });
            }
            // the objects taken keep the check the removal left them
            if left.any() {
                let objects = taken.iter().flat_map(|w| w.names.iter().cloned()).collect();
                next.push(ClockUncertainty { setup: left.0, hold: left.1, objects, written: taken, flags: side_flags(&u.flags, left), ..u.clone() });
            }
        }
        Ok(next)
    }

    /// Inter-clock uncertainties without the pairs `from × to` on those edges and checks
    /// (`InterClockUncertainty::removeUncertainty`). A command naming several clocks per side is
    /// split into its pairs only when a pair of it is touched.
    fn uncertainties_without_pairs(&self, pairs: &(Vec<String>, Both, Vec<String>, Both), sides: Both) -> Result<Vec<ClockUncertainty>, String> {
        let (from, from_e, to, to_e) = pairs;
        let mut next = Vec::with_capacity(self.uncertainties.len());
        for u in &self.uncertainties {
            if u.from.is_empty() || u.to.is_empty() {
                next.push(u.clone());
                continue;
            }
            let touched = u.from.iter().any(|f| from.contains(f)) && u.to.iter().any(|t| to.contains(t));
            if !touched {
                next.push(u.clone());
                continue;
            }
            let fk = u.written_sides.iter().map(|(k, _)| k.as_str()).find(|k| FROM_KEYS.contains(k)).unwrap_or("-from");
            let tk = u.written_sides.iter().map(|(k, _)| k.as_str()).find(|k| TO_KEYS.contains(k)).unwrap_or("-to");
            let (held_fe, held_te) = (edge_of(fk), edge_of(tk));
            for f in &u.from {
                for t in &u.to {
                    let one_pair = |fk: &str, tk: &str, setup: bool, hold: bool| ClockUncertainty {
                        setup,
                        hold,
                        from: vec![f.clone()],
                        to: vec![t.clone()],
                        written_sides: vec![
                            (fk.to_string(), Objects { accessor: "get_clocks".into(), names: vec![f.clone()], ..Default::default() }),
                            (tk.to_string(), Objects { accessor: "get_clocks".into(), names: vec![t.clone()], ..Default::default() }),
                        ],
                        flags: side_flags(&u.flags, Both(setup, hold)),
                        ..u.clone()
                    };
                    if !(from.contains(f) && to.contains(t)) {
                        next.push(one_pair(fk, tk, u.setup, u.hold));
                        continue;
                    }
                    // per (from edge, to edge), the checks left
                    let cell = |fe: bool, te: bool| -> Both {
                        let held = Both(u.setup, u.hold);
                        if !fe || !te {
                            return Both(false, false);
                        }
                        held
                    };
                    let mut left = [[Both(false, false); 2]; 2];
                    for (i, fe) in [held_fe.0, held_fe.1].into_iter().enumerate() {
                        for (j, te) in [held_te.0, held_te.1].into_iter().enumerate() {
                            let removed = [from_e.0, from_e.1][i] && [to_e.0, to_e.1][j];
                            let c = cell(fe, te);
                            left[i][j] = if removed { c.minus(sides) } else { c };
                        }
                    }
                    let keys = |i: usize, j: usize, fe_both: bool, te_both: bool| -> (&'static str, &'static str) {
                        let f = if fe_both { "-from" } else if i == 0 { "-rise_from" } else { "-fall_from" };
                        let t = if te_both { "-to" } else if j == 0 { "-rise_to" } else { "-fall_to" };
                        (f, t)
                    };
                    let all_same = left[0][0] == left[0][1] && left[0][0] == left[1][0] && left[0][0] == left[1][1];
                    if all_same {
                        if left[0][0].any() {
                            let (fk, tk) = keys(0, 0, true, true);
                            next.push(one_pair(fk, tk, left[0][0].0, left[0][0].1));
                        }
                        continue;
                    }
                    for (i, row) in left.iter().enumerate() {
                        if row[0] == row[1] {
                            if row[0].any() {
                                let (fk, tk) = keys(i, 0, false, true);
                                next.push(one_pair(fk, tk, row[0].0, row[0].1));
                            }
                            continue;
                        }
                        for (j, cell) in row.iter().enumerate().filter(|(_, c)| c.any()) {
                            let (fk, tk) = keys(i, j, false, false);
                            next.push(one_pair(fk, tk, cell.0, cell.1));
                        }
                    }
                }
            }
        }
        Ok(next)
    }
}

/// `flags` with `-setup` / `-hold` set for the checks `sides` holds.
fn side_flags(flags: &[String], sides: Both) -> Vec<String> {
    let mut out: Vec<String> = flags.iter().filter(|f| !matches!(f.as_str(), "-setup" | "-hold")).cloned().collect();
    match sides {
        Both(true, false) => out.push("-setup".into()),
        Both(false, true) => out.push("-hold".into()),
        _ => {}
    }
    out
}

// ---- unset_path_exceptions ------------------------------------------------------------------

/// One point of an exception (`ExceptionFrom` / `ExceptionThru` / `ExceptionTo` hold pins,
/// clocks, instances and, through, nets): one object, its accessor kept.
type Point = Objects;

/// One side of an exception: the key it was written with (its edge), and its points.
#[derive(Debug, Clone, PartialEq)]
struct Side {
    key: String,
    points: Vec<Point>,
}

/// An exception as `resetMatch` sees it: from, thrus in order, to (with the endpoint edge,
/// `-rise` / `-fall`), and the check it holds for.
#[derive(Debug, Clone, PartialEq)]
struct ExcShape {
    from: Option<Side>,
    thrus: Vec<Side>,
    to: Option<Side>,
    end: Both,
    min_max: Both,
}

const THRU_KEYS: [&str; 3] = ["-through", "-rise_through", "-fall_through"];

fn points(o: &Objects) -> Vec<Point> {
    if !o.items.is_empty() {
        return o.items.iter().flat_map(points).collect();
    }
    o.names.iter().map(|n| one_object(o, n)).collect()
}

/// `o`'s accessor and options over ONE of its names — without copying its whole name list per
/// name, which on a 10 000-object list is 10^8 string copies.
fn one_object(o: &Objects, name: &str) -> Objects {
    Objects { accessor: o.accessor.clone(), names: vec![name.to_string()], flags: o.flags.clone(), keys: o.keys.clone(), items: Vec::new() }
}

/// The kind of object a point is: `c` clock, `p` pin or port, `i` instance, `n` net, `?` a bare
/// name that is not a clock (the reference finds it as whichever object has the name).
fn kind_of(p: &Point, clocks: &[SdcClock]) -> char {
    match p.accessor.as_str() {
        "get_clocks" | "all_clocks" => 'c',
        "get_pins" | "get_ports" | "all_inputs" | "all_outputs" | "all_registers" => 'p',
        "get_cells" => 'i',
        "get_nets" => 'n',
        _ if p.names.first().is_some_and(|n| clocks.iter().any(|c| &c.name == n)) => 'c',
        _ => '?',
    }
}

/// `ExceptionPt::intersectsPts`: the same edge (and endpoint edge, for a to), and one object in
/// common.
fn sides_intersect(reset: &Side, held: &Side, clocks: &[SdcClock]) -> Hit {
    if edge_of(&reset.key) != edge_of(&held.key) {
        return Hit::No;
    }
    let mut out = Hit::No;
    for r in &reset.points {
        for h in &held.points {
            let (kr, kh) = (kind_of(r, clocks), kind_of(h, clocks));
            let compatible = kr == kh || ((kr == '?' || kh == '?') && kr != 'c' && kh != 'c');
            if !compatible {
                continue;
            }
            match name_hit(&r.names[0], &h.names[0]) {
                Hit::Yes => return Hit::Yes,
                Hit::Unknown => out = Hit::Unknown,
                Hit::No => {}
            }
        }
    }
    out
}

fn and(a: Hit, b: Hit) -> Hit {
    match (a, b) {
        (Hit::No, _) | (_, Hit::No) => Hit::No,
        (Hit::Yes, Hit::Yes) => Hit::Yes,
        _ => Hit::Unknown,
    }
}

/// `ExceptionPath::resetMatch`: only the reset's own points must match — a `-from` reset takes
/// any exception whose from intersects it, however specific; each kind of point the reset names,
/// the exception must have (and none it does not name); thrus pairwise over the shorter list; and
/// the check: the reset's (`-setup` / `-hold`) is all, or the exception's exactly.
fn reset_match(reset: &ExcShape, held: &ExcShape, clocks: &[SdcClock]) -> Hit {
    if !(reset.min_max == Both::ALL || reset.min_max == held.min_max) {
        return Hit::No;
    }
    let mut hit = Hit::Yes;
    match (&reset.from, &held.from) {
        (Some(r), Some(h)) => hit = and(hit, sides_intersect(r, h, clocks)),
        (Some(_), None) => return Hit::No,
        (None, _) => {}
    }
    if !reset.thrus.is_empty() {
        if held.thrus.is_empty() {
            return Hit::No;
        }
        for (r, h) in reset.thrus.iter().zip(&held.thrus) {
            hit = and(hit, sides_intersect(r, h, clocks));
        }
    }
    match (&reset.to, &held.to) {
        (Some(r), Some(h)) => {
            if reset.end != held.end {
                return Hit::No;
            }
            hit = and(hit, sides_intersect(r, h, clocks));
        }
        (Some(_), None) => return Hit::No,
        (None, _) => {}
    }
    hit
}

/// What survives a reset of `held`, as `Sdc::resetPath` leaves it — WITHOUT enumerating its
/// expansion.
///
/// The reference expands a matched exception to one point per side (`ExpandedExceptionVisitor`:
/// every from point × every point of each thru × every to point; a side with no points, a to that
/// is only `-rise`, yields none) and keeps the expansions the reset does not match. An expansion
/// is matched exactly when, in EVERY dimension the reset names, its point intersects the reset's
/// (the edge and check tests are the same for all of them). So the matched expansions are the
/// product of the matching points `M_d` of those dimensions, and what survives is the full
/// product minus that box — written as one exception per named dimension `d_i`, last first: `M`
/// on the named dimensions after it, the points NOT matched on `d_i`, and all points elsewhere. That is
/// linear in the points; enumerating the product is not (10 000 from points × 10 000 to points is
/// 10^8 expansions, on a large design routine).
fn residue(reset: &ExcShape, held: &ExcShape, clocks: &[SdcClock]) -> Result<Vec<ExcShape>, String> {
    // (reset side, held side) per named dimension, by position: 0 from, 1.. thrus, last to
    let mut named: Vec<(usize, &Side, &Side)> = Vec::new();
    if let (Some(r), Some(h)) = (&reset.from, &held.from) {
        named.push((0, r, h));
    }
    for (i, (r, h)) in reset.thrus.iter().zip(&held.thrus).enumerate() {
        named.push((i + 1, r, h));
    }
    let to_at = held.thrus.len() + 1;
    if let (Some(r), Some(h)) = (&reset.to, &held.to) {
        named.push((to_at, r, h));
    }
    // per named dimension, its points split into those the reset matches and the rest
    let mut split: Vec<(usize, Side, Side)> = Vec::new();
    for (at, r, h) in named {
        let (mut hit, mut rest) = (Vec::new(), Vec::new());
        for p in &h.points {
            match sides_intersect(r, &Side { key: h.key.clone(), points: vec![p.clone()] }, clocks) {
                Hit::Yes => hit.push(p.clone()),
                Hit::No => rest.push(p.clone()),
                Hit::Unknown => return Err("an exception it matches has objects only the design can split".into()),
            }
        }
        split.push((at, Side { key: h.key.clone(), points: hit }, Side { key: h.key.clone(), points: rest }));
    }
    let set = |e: &mut ExcShape, at: usize, side: Side| {
        if at == 0 {
            e.from = Some(side);
        } else if at == to_at {
            e.to = Some(side);
        } else {
            e.thrus[at - 1] = side;
        }
    };
    // LAST named dimension first, as the reference's merge of the kept expansions leaves them
    // (measured 2026-11-03: from `{req_val reset}` × to `{resp_val req_rdy}` reset at req_val to
    // resp_val keeps `{req_val reset}` to req_rdy, then reset to resp_val)
    let mut out = Vec::new();
    for (i, (at, _, rest)) in split.iter().enumerate().rev() {
        if rest.points.is_empty() {
            continue;
        }
        let mut e = held.clone();
        for (at_j, hit_j, _) in &split[i + 1..] {
            set(&mut e, *at_j, hit_j.clone());
        }
        set(&mut e, *at, rest.clone());
        // a side with no points makes an empty product: nothing of it survives
        let empty = e.from.as_ref().is_some_and(|f| f.points.is_empty())
            || e.thrus.iter().any(|t| t.points.is_empty())
            || e.to.as_ref().is_some_and(|t| t.points.is_empty());
        if !empty {
            out.push(e);
        }
    }
    Ok(out)
}

/// Points back to one object list: one accessor when they share it, else a `[list …]`.
fn objects_of(points: &[Point]) -> Objects {
    let same = points.windows(2).all(|w| w[0].accessor == w[1].accessor && w[0].flags == w[1].flags && w[0].keys == w[1].keys);
    match points.first() {
        Some(p) if same => Objects { names: points.iter().map(|q| q.names[0].clone()).collect(), items: Vec::new(), ..p.clone() },
        _ => Objects { accessor: "list".into(), names: points.iter().map(|q| q.names[0].clone()).collect(), items: points.to_vec(), ..Default::default() },
    }
}

fn shape_args(s: &ExcShape) -> Vec<(String, Objects)> {
    let mut out = Vec::new();
    out.extend(s.from.iter().map(|f| (f.key.clone(), objects_of(&f.points))));
    out.extend(s.thrus.iter().map(|t| (t.key.clone(), objects_of(&t.points))));
    out.extend(s.to.iter().map(|t| (t.key.clone(), objects_of(&t.points))));
    out
}

/// An exception's shape from its written arguments.
fn shape_of(args: &[(String, Objects)], flags: &[String], min_max: Both) -> ExcShape {
    let has = |f: &str| flags.iter().any(|x| x == f);
    let side = |keys: &[&str]| args.iter().find(|(k, _)| keys.contains(&k.as_str())).map(|(k, o)| Side { key: k.clone(), points: points(o) });
    let end = Both::of(has("-rise"), has("-fall"));
    let mut to = side(&TO_KEYS);
    // a `-rise` / `-fall` with no `-to` is still a to: the endpoint edge, with no points
    if to.is_none() && end != Both::ALL {
        to = Some(Side { key: "-to".into(), points: Vec::new() });
    }
    ExcShape {
        from: side(&FROM_KEYS),
        thrus: args.iter().filter(|(k, _)| THRU_KEYS.contains(&k.as_str())).map(|(k, o)| Side { key: k.clone(), points: points(o) }).collect(),
        to,
        end,
        min_max,
    }
}

/// The check an exception holds for: `-setup` alone max, `-hold` alone min, else both.
fn check_of(flags: &[String]) -> Both {
    let has = |f: &str| flags.iter().any(|x| x == f);
    Both::of(has("-setup"), has("-hold"))
}

/// `e` over new arguments, its sides and refusal derived as when it was read.
fn exception_over(e: &Exception, args: Vec<(String, Objects)>) -> (Exception, [Vec<Objects>; 2]) {
    let pick = |keys: &[&str]| -> Vec<Objects> { args.iter().filter(|(k, _)| keys.contains(&k.as_str())).map(|(_, o)| o.clone()).collect() };
    let sides = [pick(&FROM_KEYS), pick(&TO_KEYS)];
    let mut refused = None;
    if args.iter().any(|(k, _)| THRU_KEYS.contains(&k.as_str())) {
        refused = Some("a -through list, which an instance-level match cannot express".to_string());
    }
    if refused.is_none() {
        let edge = ["-rise_from", "-fall_from", "-rise_to", "-fall_to"].into_iter().find(|f| args.iter().any(|(k, _)| k == f));
        let edge = edge.or_else(|| ["-rise", "-fall"].into_iter().find(|f| e.flags.iter().any(|x| x == f)));
        if let Some(f) = edge {
            refused = Some(format!("{f}: an exception on one edge only, which an instance-level match cannot express"));
        }
    }
    let (from, rf) = reduce_exception_objects(&sides[0]);
    let (to, rt) = reduce_exception_objects(&sides[1]);
    let multiplier = e.refused.clone().filter(|r| r.starts_with("multiplier"));
    let x = Exception { from, to, refused: refused.or(rf).or(rt).or(multiplier), args, ..e.clone() };
    (x, sides)
}

impl Sdc {
    /// `unset_path_exceptions [-setup|-hold] [-from …] [-through …]… [-to …] [-rise|-fall]`
    /// (`unset_path_exceptions_cmd`, `Sdc::resetPath`). A from, through or to is required (449),
    /// no other word (448). Each exception the reset matches ([`reset_match`]) keeps what its
    /// expansion keeps ([`residue`]). False paths, multicycles and `set_max_delay` / `set_min_delay` are all
    /// exceptions here, and so are `group_path` and `set_path_margin` ([`super::ExceptionCmd`]).
    fn unset_path_exceptions(&mut self, toks: &[String]) -> Result<(), String> {
        let a = key_args(toks, &[&FROM_KEYS[..], &TO_KEYS[..], &THRU_KEYS[..]].concat(), &["-setup", "-hold", "-rise", "-fall"])?;
        if !a.pos.is_empty() {
            return Err("positional arguments not supported (Error 448)".into());
        }
        let mut args: Vec<(String, Objects)> = Vec::new();
        if let Some((k, v)) = a.first_key(&FROM_KEYS) {
            args.push((k.to_string(), Objects::of(v)));
        }
        args.extend(a.keys.iter().filter(|(k, _)| THRU_KEYS.contains(&k.as_str())).map(|(k, v)| (k.clone(), Objects::of(v))));
        if let Some((k, v)) = a.first_key(&TO_KEYS) {
            args.push((k.to_string(), Objects::of(v)));
        }
        let reset = shape_of(&args, &a.flags, check_of(&a.flags));
        if reset.from.is_none() && reset.thrus.is_empty() && reset.to.is_none() {
            return Err("-from, -through or -to required (Error 449)".into());
        }
        self.reset(&reset)
    }

    /// `-reset_path` on `set_false_path` / `set_multicycle_path` (the command's own check) and
    /// `set_max_delay` / `set_min_delay` (always both checks): `reset_path_cmd` over the command's
    /// own from, thrus and to, run BEFORE the new exception is made — so it never takes itself.
    pub(super) fn reset_path(&mut self, args: &[(String, Objects)], flags: &[String], both_checks: bool) -> Result<(), String> {
        let min_max = if both_checks { Both::ALL } else { check_of(flags) };
        let reset = shape_of(args, flags, min_max);
        if reset.from.is_none() && reset.thrus.is_empty() && reset.to.is_none() {
            return Ok(());
        }
        self.reset(&reset)
    }

    /// `Sdc::resetPath`.
    fn reset(&mut self, reset: &ExcShape) -> Result<(), String> {
        let clocks = self.clocks.clone();
        // one pass decides every exception, so a refusal leaves the model as it was
        let reset_one = |held: &ExcShape| -> Result<Option<Vec<ExcShape>>, String> {
            match reset_match(reset, held, &clocks) {
                Hit::No => Ok(None),
                Hit::Unknown => Err("whether it matches an exception depends on objects only the design can resolve".into()),
                Hit::Yes => residue(reset, held, &clocks).map(Some),
            }
        };
        let mut exceptions = Vec::with_capacity(self.exceptions.len());
        let mut exception_objects = Vec::with_capacity(self.exceptions.len());
        for (e, objs) in self.exceptions.iter().zip(&self.exception_objects) {
            match reset_one(&shape_of(&e.args, &e.flags, check_of(&e.flags)))? {
                None => {
                    exceptions.push(e.clone());
                    exception_objects.push(objs.clone());
                }
                Some(rest) => {
                    for s in rest {
                        let (x, sides) = exception_over(e, shape_args(&s));
                        exceptions.push(x);
                        exception_objects.push(sides);
                    }
                }
            }
        }
        let mut path_delays: Vec<PathDelayCmd> = Vec::with_capacity(self.path_delays.len());
        for p in &self.path_delays {
            let min_max = if p.max { Both(true, false) } else { Both(false, true) };
            match reset_one(&shape_of(&p.objects, &p.flags, min_max))? {
                None => path_delays.push(p.clone()),
                Some(rest) => path_delays.extend(rest.iter().map(|s| PathDelayCmd { objects: shape_args(s), ..p.clone() })),
            }
        }
        // `group_path` (both checks) and `set_path_margin` (its own) are exceptions to a reset too
        let mut others = Vec::with_capacity(self.other_exceptions.len());
        for x in &self.other_exceptions {
            let min_max = if x.cmd == "group_path" { Both::ALL } else { check_of(&x.flags) };
            match reset_one(&shape_of(&x.args, &x.flags, min_max))? {
                None => others.push(x.clone()),
                Some(rest) => others.extend(rest.iter().map(|s| super::ExceptionCmd { args: shape_args(s), ..x.clone() })),
            }
        }
        self.exceptions = exceptions;
        self.exception_objects = exception_objects;
        self.path_delays = path_delays;
        self.other_exceptions = others;
        Ok(())
    }

    /// The design-wide uncertainty scalars: the largest clock's FINAL value per check (each
    /// clock's last `set_clock_uncertainty` on it wins); inter-clock values stay in the records.
    pub(super) fn refold_uncertainty(&mut self) {
        let fold = |hold_side: bool| -> f64 {
            let mut last: std::collections::BTreeMap<&str, f64> = Default::default();
            for u in &self.uncertainties {
                if u.from.is_empty() && u.to.is_empty() && (if hold_side { u.hold } else { u.setup }) {
                    for o in &u.objects {
                        last.insert(o.as_str(), u.value);
                    }
                }
            }
            last.values().fold(0.0f64, |a, b| a.max(*b))
        };
        self.setup_uncertainty = fold(false);
        self.hold_uncertainty = fold(true);
    }
}

#[cfg(test)]
mod tests {
    use super::super::Sdc;

    fn read(text: &str) -> Sdc {
        Sdc::parse(text).expect("the test SDC reads")
    }

    const CLOCKS: &str = "create_clock -name a -period 2 [get_ports ca]\ncreate_clock -name b -period 3 [get_ports cb]\n";

    /// Rule (delete_clock → `Sdc::removeClock`): the clock goes, and with it every I/O delay
    /// relative to it, every exception side naming it (an emptied side deletes the exception),
    /// every inter-clock uncertainty with it on either side, its own uncertainty and slew, and its
    /// place in each clock group. Another clock's constraints stay.
    #[test]
    fn delete_clock_takes_what_names_it() {
        let s = read(&format!(
            "{CLOCKS}set_input_delay 1 -clock a [get_ports i1]\nset_input_delay 1 -clock b [get_ports i2]\n\
             set_false_path -from [get_clocks a] -to [get_clocks b]\nset_clock_uncertainty 0.2 -from [get_clocks a] -to [get_clocks b]\n\
             set_clock_uncertainty 0.1 [get_clocks {{a b}}]\nset_clock_transition 0.05 [get_clocks {{a b}}]\n\
             set_clock_groups -asynchronous -group [get_clocks a] -group [get_clocks b]\ndelete_clock [get_clocks a]\n"
        ));
        assert_eq!(s.clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["b"]);
        assert_eq!(s.input_delays.len(), 1, "{:?}", s.input_delays);
        assert_eq!(s.input_delays[0].clock.as_deref(), Some("b"));
        assert!(s.exceptions.is_empty(), "{:?}", s.exceptions);
        assert_eq!(s.uncertainties.len(), 1, "{:?}", s.uncertainties);
        assert_eq!(s.uncertainties[0].objects, ["b"]);
        assert_eq!(s.clock_transitions[0].objects, ["b"]);
        assert_eq!(s.clock_groups[0].groups, [vec!["b".to_string()]]);
        assert!(s.rejected.is_empty() && !s.ignored.iter().any(|c| c == "delete_clock"), "{:?} {:?}", s.rejected, s.ignored);
    }

    /// Rule (delete_clock): `-all` takes every clock and no other word; without it exactly one
    /// word. `delete_generated_clock` takes only generated clocks; a generated clock whose master
    /// is deleted keeps no master (`deleteMasterClkRefs`).
    #[test]
    fn delete_all_and_generated_only() {
        let s = read(&format!("{CLOCKS}delete_clock -all\n"));
        assert!(s.clocks.is_empty());
        let s = read(&format!("{CLOCKS}delete_clock -all a\n"));
        assert_eq!(s.clocks.len(), 2, "-all with a word is wrong # args");
        assert_eq!(s.rejected.len(), 1, "{:?}", s.rejected);
        let g = "create_generated_clock -name g -source [get_ports ca] -master_clock [get_clocks a] -divide_by 2 [get_pins r/Q]\n";
        let s = read(&format!("{CLOCKS}{g}delete_generated_clock -all\n"));
        assert_eq!(s.clocks.iter().map(|c| c.name.as_str()).collect::<Vec<_>>(), ["a", "b"]);
        let s = read(&format!("{CLOCKS}{g}delete_clock a\n"));
        let gen = s.clocks.iter().find(|c| c.name == "g").expect("g stays");
        assert_eq!(gen.generated.as_ref().and_then(|g| g.master_clock.clone()), None);
    }

    /// Rule (unset_case_analysis → `removeCaseAnalysis`): each named pin's value is erased; other
    /// pins of the same command keep theirs. Splitting a wildcard needs the design: refused, the
    /// model unchanged.
    #[test]
    fn unset_case_analysis_per_pin() {
        let s = read("set_case_analysis 0 [get_pins {u1/A u2/A}]\nunset_case_analysis [get_pins u1/A]\n");
        assert_eq!(s.case_values.len(), 1);
        assert_eq!(s.case_values[0].objects, ["u2/A"]);
        let s = read("set_case_analysis 0 [get_pins u1/A]\nunset_case_analysis [get_pins u1/A]\n");
        assert!(s.case_values.is_empty());
        assert!(!s.ignored.iter().any(|c| c == "set_case_analysis"), "{:?}", s.ignored);
        let s = read("set_case_analysis 0 [get_pins u1/*]\nunset_case_analysis [get_pins u1/A]\n");
        assert_eq!(s.case_values[0].objects, ["u1/*"]);
        assert_eq!(s.rejected.len(), 1, "{:?}", s.rejected);
    }

    /// Rule (unset_input_delay → `removeInputDelay`): the delay of the port FOR that clock edge
    /// loses the edges and min/max named, and only those; a delay for another clock stays.
    #[test]
    fn unset_input_delay_per_clock_and_cell() {
        let s = read(&format!(
            "{CLOCKS}set_input_delay 1 -clock a [get_ports {{i1 i2}}]\nset_input_delay 2 -clock b -add_delay [get_ports i1]\n\
             unset_input_delay -clock a -max [get_ports i1]\n"
        ));
        let d: Vec<(Vec<String>, Option<String>, Vec<String>)> = s.input_delays.iter().map(|d| (d.ports.clone(), d.clock.clone(), d.flags.clone())).collect();
        assert_eq!(
            d,
            [
                (vec!["i2".to_string()], Some("a".to_string()), vec![]),
                (vec!["i1".to_string()], Some("a".to_string()), vec!["-min".to_string()]),
                (vec!["i1".to_string()], Some("b".to_string()), vec!["-add_delay".to_string()]),
            ]
        );
        // a delay set without -clock is taken only by an unset without -clock
        let s = read(&format!("{CLOCKS}set_input_delay 1 [get_ports i1]\nunset_input_delay -clock a [get_ports i1]\n"));
        assert_eq!(s.input_delays.len(), 1);
        let s = read(&format!("{CLOCKS}set_input_delay 1 [get_ports i1]\nunset_input_delay [get_ports i1]\n"));
        assert!(s.input_delays.is_empty());
    }

    /// Rule (removeInputDelay): removing max on one edge leaves a set that is not one command's —
    /// written as two, the second with `-add_delay` so it does not replace other clocks' delays.
    #[test]
    fn unset_input_delay_partial_edge_splits() {
        let s = read(&format!("{CLOCKS}set_input_delay 1 -clock a [get_ports i1]\nunset_input_delay -clock a -rise -max [get_ports i1]\n"));
        let f: Vec<Vec<String>> = s.input_delays.iter().map(|d| d.flags.clone()).collect();
        assert_eq!(f, [vec!["-rise".to_string(), "-min".to_string()], vec!["-add_delay".to_string(), "-fall".to_string()]]);
    }

    /// Rule (unset_clock_latency → `removeClockLatency(clk, pin)`): one entry by exactly that
    /// pair — on a pin, only the latency set for the `-clock` given (none: the one set without);
    /// `-source` takes the insertion, not the network latency.
    #[test]
    fn unset_clock_latency_by_pair() {
        let base = format!("{CLOCKS}set_clock_latency 0.3 -clock a [get_pins r/CK]\nset_clock_latency -source 0.5 [get_clocks a]\nset_clock_latency 0.2 [get_clocks a]\n");
        let s = read(&format!("{base}unset_clock_latency [get_pins r/CK]\n"));
        assert_eq!(s.clock_latencies.len(), 3, "a pin latency for -clock a is not the one without -clock");
        let s = read(&format!("{base}unset_clock_latency -clock a [get_pins r/CK]\n"));
        assert_eq!(s.clock_latencies.len(), 2);
        let s = read(&format!("{base}unset_clock_latency [get_clocks a]\n"));
        assert_eq!(s.clock_latencies.len(), 2);
        assert!(s.clock_latencies.iter().any(|e| e.flags.iter().any(|f| f == "-source")), "the insertion stays");
        let s = read(&format!("{base}unset_clock_latency -source [get_clocks a]\n"));
        assert!(!s.clock_latencies.iter().any(|e| e.flags.iter().any(|f| f == "-source")));
    }

    /// Rule (unset_clock_uncertainty): `-setup` alone takes the setup value and leaves hold;
    /// `-from` without `-to` is Error 423; `-rise` on a single clock is accepted (the reference's
    /// guard tests keys, never flags).
    #[test]
    fn unset_clock_uncertainty_checks() {
        let s = read(&format!("{CLOCKS}set_clock_uncertainty 0.1 [get_clocks {{a b}}]\nunset_clock_uncertainty -setup [get_clocks a]\n"));
        assert_eq!(s.uncertainties.len(), 2, "{:?}", s.uncertainties);
        assert_eq!((s.uncertainties[1].objects.clone(), s.uncertainties[1].setup, s.uncertainties[1].hold), (vec!["a".to_string()], false, true));
        assert_eq!(s.setup_uncertainty, 0.1, "b keeps its setup value");
        let s = read(&format!("{CLOCKS}set_clock_uncertainty 0.1 [get_clocks a]\nunset_clock_uncertainty -rise [get_clocks a]\n"));
        assert!(s.uncertainties.is_empty() && s.rejected.is_empty(), "{:?}", s.rejected);
        let s = read(&format!("{CLOCKS}unset_clock_uncertainty -from [get_clocks a]\n"));
        assert!(s.rejected[0].contains("423"), "{:?}", s.rejected);
    }

    /// Rule (removeUncertainty on a clock pair): a command over several clocks per side loses
    /// only the pair named; the other pairs stay.
    #[test]
    fn unset_inter_clock_pair() {
        let s = read(&format!(
            "{CLOCKS}create_clock -name c -period 4 [get_ports cc]\nset_clock_uncertainty 0.2 -from [get_clocks {{a b}}] -to [get_clocks c]\n\
             unset_clock_uncertainty -from [get_clocks a] -to [get_clocks c]\n"
        ));
        let pairs: Vec<(Vec<String>, Vec<String>)> = s.uncertainties.iter().map(|u| (u.from.clone(), u.to.clone())).collect();
        assert_eq!(pairs, [(vec!["b".to_string()], vec!["c".to_string()])]);
    }

    /// Rule (resetPath / resetMatch): a `-from` reset takes the from points it names out of every
    /// exception whose from intersects it, however specific; the other points stay. A `-setup`
    /// reset does not take an exception that holds for both checks (its min_max must be equal).
    #[test]
    fn unset_path_exceptions_points() {
        let s = read("set_false_path -from [get_ports {a b}] -to [get_ports z]\nunset_path_exceptions -from [get_ports a]\n");
        assert_eq!(s.exceptions.len(), 1, "{:?}", s.exceptions);
        assert_eq!(s.exceptions[0].args[0].1.names, ["b"]);
        assert_eq!(s.exceptions[0].from, ["b"]);
        let s = read("set_false_path -from [get_ports a] -to [get_ports z]\nunset_path_exceptions -from [get_ports a]\n");
        assert!(s.exceptions.is_empty() && s.exception_objects.is_empty());
        let s = read("set_false_path -from [get_ports a]\nunset_path_exceptions -setup -from [get_ports a]\n");
        assert_eq!(s.exceptions.len(), 1, "min_max all is not max");
        let s = read("set_max_delay 2 -from [get_ports a] -to [get_ports z]\nunset_path_exceptions -to [get_ports z]\n");
        assert!(s.path_delays.is_empty());
        let s = read("set_false_path -to [get_ports z]\nunset_path_exceptions -from [get_ports a]\n");
        assert_eq!(s.exceptions.len(), 1, "a -from reset needs a from");
        let s = read("unset_path_exceptions -setup\n");
        assert!(s.rejected[0].contains("449"), "{:?}", s.rejected);
    }

    /// Rule (resetPath expansion): a reset naming two dimensions takes only their product — from
    /// `{a b}` × to `{y z}` reset `-from a -to y` keeps `{a b}` to z, then b to y (the
    /// reference's form).
    #[test]
    fn unset_path_exceptions_two_dimensions() {
        let s = read("set_false_path -from [get_ports {a b}] -to [get_ports {y z}]\nunset_path_exceptions -from [get_ports a] -to [get_ports y]\n");
        let kept: Vec<(Vec<String>, Vec<String>)> = s.exceptions.iter().map(|e| (e.args[0].1.names.clone(), e.args[1].1.names.clone())).collect();
        assert_eq!(kept, [(vec!["a".to_string(), "b".to_string()], vec!["z".to_string()]), (vec!["b".to_string()], vec!["y".to_string()])]);
    }

    /// A reset of a huge exception is linear in its points: 10 000 × 10 000 would be 10^8
    /// expansions if enumerated.
    #[test]
    fn unset_path_exceptions_scales() {
        let from: Vec<String> = (0..10_000).map(|i| format!("f{i}")).collect();
        let to: Vec<String> = (0..10_000).map(|i| format!("t{i}")).collect();
        let text = format!("set_false_path -from [get_pins {{{}}}] -to [get_pins {{{}}}]\nunset_path_exceptions -from [get_pins f17]\n", from.join(" "), to.join(" "));
        let t0 = std::time::Instant::now();
        let s = read(&text);
        assert_eq!(s.exceptions.len(), 1);
        assert_eq!(s.exceptions[0].args[0].1.names.len(), 9_999);
        assert_eq!(s.exceptions[0].args[1].1.names.len(), 10_000);
        assert!(t0.elapsed().as_secs() < 5, "took {:?}", t0.elapsed());
    }

    /// Rule (`-reset_path`): `set_max_delay` / `set_min_delay` reset over their own points for
    /// BOTH checks before the new exception is made; `set_false_path` with its own check (a
    /// `-setup` false path does not reset one that holds for both).
    #[test]
    fn reset_path_runs_first() {
        let s = read("set_false_path -from [get_ports a] -to [get_ports z]\nset_max_delay 2 -reset_path -from [get_ports a]\n");
        assert!(s.exceptions.is_empty(), "{:?}", s.exceptions);
        assert_eq!(s.path_delays.len(), 1, "the new delay is made after the reset");
        let s = read("set_false_path -from [get_ports a]\nset_false_path -setup -reset_path -from [get_ports a]\n");
        assert_eq!(s.exceptions.len(), 2);
        let s = read("set_multicycle_path 2 -setup -from [get_ports a]\nset_false_path -setup -reset_path -from [get_ports a]\n");
        assert_eq!(s.exceptions.len(), 1);
        assert_eq!(s.exceptions[0].kind, super::super::ExcKind::FalsePath);
    }

    /// Rule (unset_clock_groups): `-name` or `-all` (404/405) and one kind (406/407); an unnamed
    /// command is `group<n>`; a group of another kind stays.
    #[test]
    fn unset_clock_groups_by_name_and_kind() {
        let g = "set_clock_groups -asynchronous -group [get_clocks a] -group [get_clocks b]\nset_clock_groups -name x -physically_exclusive -group [get_clocks a] -group [get_clocks b]\n";
        let s = read(&format!("{CLOCKS}{g}unset_clock_groups -asynchronous -name group1\n"));
        assert_eq!(s.clock_groups.len(), 1);
        assert_eq!(s.clock_groups[0].name.as_deref(), Some("x"));
        assert!(s.async_groups.len() == 2, "only the asynchronous command's groups go: {:?}", s.async_groups);
        let s = read(&format!("{CLOCKS}{g}unset_clock_groups -asynchronous -name x\n"));
        assert_eq!(s.clock_groups.len(), 2, "x is physically exclusive");
        let s = read(&format!("{CLOCKS}{g}unset_clock_groups -all\n"));
        assert!(s.rejected[0].contains("406"), "{:?}", s.rejected);
        // a later unnamed command was `group2` when made; with group1 gone it keeps that name
        // (measured on the reference, which writes `-name group2`)
        let s = read(&format!("{CLOCKS}{g}set_clock_groups -asynchronous -group [get_clocks b]\nunset_clock_groups -asynchronous -name group1\n"));
        let names: Vec<Option<&str>> = s.clock_groups.iter().map(|g| g.name.as_deref()).collect();
        assert_eq!(names, [Some("x"), Some("group2")]);
    }

    /// Rule (unset_timing_derate → deleteDeratingFactors; unset_propagated_clock;
    /// unset_clock_transition → removeSlew): each takes all of its kind on what it names.
    #[test]
    fn unset_derate_propagated_transition() {
        let s = read("set_timing_derate -late 1.05\nset_timing_derate -early 0.9 [get_cells u1]\nunset_timing_derate\n");
        assert!(s.derates.is_empty() && s.late_derate.is_none() && !s.ignored.iter().any(|c| c == "set_timing_derate"));
        let s = read(&format!("{CLOCKS}set_propagated_clock [get_clocks {{a b}}]\nunset_propagated_clock [get_clocks a]\n"));
        assert_eq!(s.propagated[0].names, ["b"]);
        let s = read(&format!("{CLOCKS}set_clock_transition -rise 0.1 [get_clocks a]\nunset_clock_transition [get_clocks a]\n"));
        assert!(s.clock_transitions.is_empty());
    }
}
