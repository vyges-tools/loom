//! Overlapping timing exceptions: which one wins on the paths they share, decided as the
//! reference timer decides it (SDC scope S2).
//!
//! An exception B is reported when another exception A covers every path B names and A wins on
//! every one of them, in every analysis (setup, hold) where B applies:
//!
//! - `exception-redundant` (info): A says the same thing (the same command and value). B can go.
//! - `exception-conflict` (warning): A has the same priority and a different value. Either A was
//!   read later and replaced B on those paths, or the reference's choice between them is not
//!   defined (a tie its tie-break cannot settle).
//! - `exception-shadowed` (warning): A outranks B and says something else. B never applies.
//!
//! The reference's rules, as read at the pin (and witnessed by the probe harness
//! `correlation/harness/sdc/exception-priority-probe.py` in the internal repo):
//!
//! - **Type first.** False path 4000, path delay 3000, multicycle 2000, path margin 500,
//!   group path 0 (`ExceptionPath::*Priority`).
//! - **Then the points** (`fromThruToPriority`): -from pins/cells/ports 64, -to pins 32, any
//!   -through 16, -from clocks 8, -to clocks 4. A -from or -to that names both pins and clocks
//!   is split into two exceptions, one per kind (`Sdc::addException`, `addException1`).
//! - **A multicycle adds 2** in the analysis its -setup/-hold names, and 1 when it names neither
//!   (`MultiCyclePath::priority`). A -setup multicycle also applies to hold, at no bonus
//!   (`MultiCyclePath::matches`, `exactly` false): it moves the hold check.
//! - **Equal priority, read time.** A multicycle, path delay or path margin first deletes the
//!   earlier exceptions of the same type, priority and min/max whose points intersect
//!   (`Sdc::addException2` → `deleteMatchingExceptions`, `overrides`, `intersectsPts`). Points
//!   intersect only when both have a -from or neither does, the same number of -through, and
//!   both a -to or neither. A false path is never deleted this way.
//! - **Equal priority, on the path.** Otherwise the tighter wins (`tighterThan`: the smaller
//!   multiplier, the smaller max delay, the larger min delay). Between two false paths, or two
//!   margins, or two group paths, there is no tie-break.
//! - **Layers** (`VisitPathEnds::visitCheckEnd`). The winner among all types decides the check:
//!   a false path removes it, a path delay replaces it, a multicycle is applied. A path margin
//!   is looked up on its own and applied with a multicycle, but not under a false path or a path
//!   delay. A group path is looked up on its own and only a false path removes it.
//!
//! ⚠️ **Only what can be shown without the design.** A covers B only when each of A's points
//! names, with the same accessor, every object B's point names (a glob in A may match B's
//! name). A clock that launches a pin, a cell that owns a pin, and query results are not
//! resolved, so such pairs are not reported. Nothing here is a guess: a missed overlap is
//! possible, a false one is not.
//!
//! A file with `-reset_path` is not checked: it removes earlier exceptions, which this does not
//! model.

use crate::sdc::{object_matches, ExcKind, Objects, Sdc};
use crate::sdc_lint::{Finding, Severity};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Ty {
    False,
    Delay,
    Multicycle,
    Margin,
    Group,
}

impl Ty {
    /// `ExceptionPath::falsePathPriority` and its siblings.
    fn base(self) -> i32 {
        match self {
            Ty::False => 4000,
            Ty::Delay => 3000,
            Ty::Multicycle => 2000,
            Ty::Margin => 500,
            Ty::Group => 0,
        }
    }
    fn name(self) -> &'static str {
        match self {
            Ty::False => "a false path",
            Ty::Delay => "a path delay",
            Ty::Multicycle => "a multicycle",
            Ty::Margin => "a path margin",
            Ty::Group => "a path group",
        }
    }
    /// The false path, path delay and multicycle compete for the check itself.
    fn timing(self) -> bool {
        matches!(self, Ty::False | Ty::Delay | Ty::Multicycle)
    }
}

/// The analysis an exception names: -setup (max), -hold (min), or both.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mm {
    Max,
    Min,
    All,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Class {
    /// Pins, ports, cells and register or port queries: the -from/-to "pin/instance" kind.
    Pins,
    Clocks,
    Nets,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Rf {
    Both,
    Rise,
    Fall,
}

/// One -from, -through or -to: its kind, its object lists (a `[list …]` flattened), and its edge.
#[derive(Clone, Debug)]
struct Pt {
    class: Class,
    objs: Vec<Objects>,
    rf: Rf,
}

/// One exception as the reference holds it, after the mixed-kind split.
#[derive(Clone, Debug)]
struct Exc {
    /// The command it came from (several when a mixed -from/-to was split).
    src: usize,
    ty: Ty,
    /// File order within its type: what "read later" compares.
    order: usize,
    mm: Mm,
    /// `-rise` / `-fall`: the path's end edge.
    end_rf: Rf,
    from: Option<Pt>,
    thrus: Vec<Pt>,
    to: Option<Pt>,
    /// What it does: two exceptions of one type with the same effect are interchangeable.
    effect: String,
    /// The multiplier or the delay, for `tighterThan`.
    value: f64,
    text: String,
}

impl Exc {
    /// `fromThruToPriority`.
    ///
    /// ℹ️ The ORDER among these bits cannot change a finding here: a cover's points are a subset
    /// of the covered one's, so within a type the covered exception's bits always include the
    /// cover's. The order decides partial overlaps, which are not reported; the reference probes
    /// (`bits-*`) witness it, and mutating it here survives by construction.
    fn bits(&self) -> i32 {
        let mut p = 0;
        if self.from.as_ref().is_some_and(|f| f.class == Class::Pins) {
            p |= 1 << 6;
        }
        if self.to.as_ref().is_some_and(|t| t.class == Class::Pins) {
            p |= 1 << 5;
        }
        if !self.thrus.is_empty() {
            p |= 1 << 4;
        }
        if self.from.as_ref().is_some_and(|f| f.class == Class::Clocks) {
            p |= 1 << 3;
        }
        if self.to.as_ref().is_some_and(|t| t.class == Class::Clocks) {
            p |= 1 << 2;
        }
        p
    }

    /// `priority(min_max)`: the base and the bits, plus the multicycle's min/max bonus.
    fn priority(&self, an: Mm) -> i32 {
        let bonus = match (self.ty, self.mm) {
            (Ty::Multicycle, Mm::All) => 1,
            (Ty::Multicycle, m) if m == an => 2,
            _ => 0,
        };
        self.ty.base() + self.bits() + bonus
    }

    /// Does it apply in analysis `an` (`Max` or `Min`)? A multicycle that names -setup also
    /// applies to hold: its multiplier moves the hold check.
    fn applies(&self, an: Mm) -> bool {
        match self.ty {
            Ty::Multicycle => an == Mm::Min || self.mm != Mm::Min,
            _ => self.mm == Mm::All || self.mm == an,
        }
    }

    /// The same kinds of points, as `intersectsPts` requires before one replaces another.
    fn same_shape(&self, o: &Exc) -> bool {
        self.from.is_some() == o.from.is_some() && self.thrus.len() == o.thrus.len() && self.to.is_some() == o.to.is_some()
    }

    /// `tighterThan`: strictly tighter than `o`.
    fn tighter(&self, o: &Exc) -> bool {
        match self.ty {
            Ty::Multicycle => self.value < o.value,
            Ty::Delay if self.mm == Mm::Min => self.value > o.value,
            Ty::Delay => self.value < o.value,
            _ => false,
        }
    }
}

fn has_glob(n: &str) -> bool {
    n.contains(['*', '?'])
}

fn class_of(o: &Objects) -> Option<Class> {
    match o.accessor.as_str() {
        "get_pins" | "get_ports" | "get_cells" | "all_registers" | "all_inputs" | "all_outputs" => Some(Class::Pins),
        "get_clocks" | "all_clocks" => Some(Class::Clocks),
        "get_nets" => Some(Class::Nets),
        _ => None,
    }
}

fn leaves(o: &Objects) -> Vec<Objects> {
    if matches!(o.accessor.as_str(), "list" | "concat") && !o.items.is_empty() {
        o.items.iter().flat_map(leaves).collect()
    } else {
        vec![o.clone()]
    }
}

/// Does object list `a` name every object `b` names?
fn obj_covers(a: &Objects, b: &Objects) -> bool {
    if a == b {
        return true;
    }
    let plain = |o: &Objects| o.flags.is_empty() && o.keys.is_empty() && o.accessor.starts_with("get_");
    a.accessor == b.accessor
        && plain(a)
        && plain(b)
        && !b.names.is_empty()
        && b.names.iter().all(|n| a.names.contains(n) || (!has_glob(n) && a.names.iter().any(|p| object_matches(p, n))))
}

/// The class test is implied by [`obj_covers`] (the same accessor is the same class); it stays
/// as the statement of the rule.
fn pt_covers(a: &Pt, b: &Pt) -> bool {
    a.class == b.class && (a.rf == Rf::Both || a.rf == b.rf) && b.objs.iter().all(|bo| a.objs.iter().any(|ao| obj_covers(ao, bo)))
}

/// Does `a` match every path `b` matches? Each of `a`'s points covers `b`'s; `a`'s -through
/// points cover a subsequence of `b`'s, in order (a path through all of `b`'s passes `a`'s).
fn covers(a: &Exc, b: &Exc) -> bool {
    let side = |x: &Option<Pt>, y: &Option<Pt>| match (x, y) {
        (None, _) => true,
        (Some(x), Some(y)) => pt_covers(x, y),
        (Some(_), None) => false,
    };
    if !(a.end_rf == Rf::Both || a.end_rf == b.end_rf) || !side(&a.from, &b.from) || !side(&a.to, &b.to) {
        return false;
    }
    let mut k = 0;
    for at in &a.thrus {
        while k < b.thrus.len() && !pt_covers(at, &b.thrus[k]) {
            k += 1;
        }
        if k == b.thrus.len() {
            return false;
        }
        k += 1;
    }
    true
}

/// Would `a`, read after `b`, delete `b` on their shared paths (`deleteMatchingExceptions`): the
/// same type among those it deletes, the same min/max, and points that intersect.
fn replaces(a: &Exc, b: &Exc) -> bool {
    a.ty == b.ty && matches!(a.ty, Ty::Delay | Ty::Multicycle | Ty::Margin) && a.mm == b.mm && a.same_shape(b)
}

/// On a path both match, in analysis `an` where both apply: does `a` decide it? `Some(true)` for
/// `a`, `Some(false)` when `b` is still applied (it wins, or the two combine), `None` when the
/// reference's choice is not defined.
fn wins(a: &Exc, b: &Exc, an: Mm) -> Option<bool> {
    let (pa, pb) = (a.priority(an), b.priority(an));
    let replaces = || replaces(a, b);
    match b.ty {
        t if t.timing() => {
            if !a.ty.timing() {
                return Some(false);
            }
            if pa != pb {
                return Some(pa > pb);
            }
            if replaces() {
                return Some(a.order > b.order);
            }
            if a.tighter(b) {
                Some(true)
            } else if b.tighter(a) {
                Some(false)
            } else {
                None
            }
        }
        Ty::Margin => match a.ty {
            Ty::False | Ty::Delay => Some(true),
            Ty::Margin if pa != pb => Some(pa > pb),
            Ty::Margin if replaces() => Some(a.order > b.order),
            Ty::Margin => None,
            _ => Some(false),
        },
        _ => match a.ty {
            Ty::False => Some(true),
            Ty::Group if pa != pb => Some(pa > pb),
            Ty::Group => None,
            _ => Some(false),
        },
    }
}

/// One command's points: `None` when one of them cannot be classified (a bare name, an
/// evaluated query), a -from or -to is given twice, or a -from/-to names nets.
#[allow(clippy::type_complexity)]
fn points(args: &[(String, Objects)]) -> Option<(Vec<Pt>, Vec<Pt>, Vec<Pt>)> {
    let (mut from, mut thrus, mut to) = (Vec::new(), Vec::new(), Vec::new());
    for (k, o) in args {
        let rf = if k.starts_with("-rise_") {
            Rf::Rise
        } else if k.starts_with("-fall_") {
            Rf::Fall
        } else {
            Rf::Both
        };
        let side = k.trim_start_matches("-rise_").trim_start_matches("-fall_").trim_start_matches('-');
        let ls = leaves(o);
        let mut by: Vec<Pt> = Vec::new();
        for l in ls {
            let c = class_of(&l)?;
            match by.iter_mut().find(|p| p.class == c) {
                Some(p) => p.objs.push(l),
                None => by.push(Pt { class: c, objs: vec![l], rf }),
            }
        }
        match side {
            "from" if from.is_empty() && by.iter().all(|p| p.class != Class::Nets) => from = by,
            "to" if to.is_empty() && by.iter().all(|p| p.class != Class::Nets) => to = by,
            // a -through of mixed kinds stays one point: the reference does not split it
            "through" => thrus.push(Pt { class: by.first()?.class, objs: by.into_iter().flat_map(|p| p.objs).collect(), rf }),
            _ => return None,
        }
    }
    Some((from, thrus, to))
}

fn mm_of(flags: &[String]) -> Mm {
    let (s, h) = (flags.iter().any(|f| f == "-setup"), flags.iter().any(|f| f == "-hold"));
    match (s, h) {
        (true, false) => Mm::Max,
        (false, true) => Mm::Min,
        _ => Mm::All,
    }
}

fn end_rf_of(flags: &[String]) -> Rf {
    let (r, f) = (flags.iter().any(|x| x == "-rise"), flags.iter().any(|x| x == "-fall"));
    match (r, f) {
        (true, false) => Rf::Rise,
        (false, true) => Rf::Fall,
        _ => Rf::Both,
    }
}

fn text_of(cmd: &str, value: &str, flags: &[String], args: &[(String, Objects)]) -> String {
    let mut t = cmd.to_string();
    if !value.is_empty() {
        t += &format!(" {value}");
    }
    for f in flags {
        t += &format!(" {f}");
    }
    for (k, o) in args {
        t += &format!(" {k} {}", crate::sdc_write::objects(o).unwrap_or_else(|| o.names.join(" ")));
    }
    t
}

/// Every exception the file states, split by kind as the reference splits it, and the commands
/// that could not be classified.
fn exceptions(sdc: &Sdc) -> (Vec<Exc>, Vec<String>) {
    struct Src<'a> {
        ty: Ty,
        mm: Mm,
        flags: &'a [String],
        args: &'a [(String, Objects)],
        effect: String,
        value: f64,
        text: String,
    }
    let mut srcs: Vec<Src> = Vec::new();
    for e in &sdc.exceptions {
        let (ty, value, effect) = match e.kind {
            ExcKind::FalsePath => (Ty::False, 0.0, "false".to_string()),
            ExcKind::Multicycle(n) => (Ty::Multicycle, f64::from(n), format!("{n}{}", if e.start { " -start" } else { " -end" })),
        };
        let v = if let ExcKind::Multicycle(n) = e.kind { n.to_string() } else { String::new() };
        let cmd = if ty == Ty::False { "set_false_path" } else { "set_multicycle_path" };
        srcs.push(Src { ty, mm: mm_of(&e.flags), flags: &e.flags, args: &e.args, effect, value, text: text_of(cmd, &v, &e.flags, &e.args) });
    }
    for p in &sdc.path_delays {
        let cmd = if p.max { "set_max_delay" } else { "set_min_delay" };
        let mut fl: Vec<&String> = p.flags.iter().collect();
        fl.sort();
        let effect = format!("{} {}", p.value, fl.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(" "));
        srcs.push(Src {
            ty: Ty::Delay,
            mm: if p.max { Mm::Max } else { Mm::Min },
            flags: &p.flags,
            args: &p.objects,
            effect,
            value: p.value,
            text: text_of(cmd, &crate::sdc_write::num(p.value), &p.flags, &p.objects),
        });
    }
    for x in &sdc.other_exceptions {
        let ty = match x.cmd.as_str() {
            "set_path_margin" => Ty::Margin,
            "group_path" => Ty::Group,
            _ => continue,
        };
        let effect = match ty {
            Ty::Group => x.name.clone().unwrap_or_else(|| "-default".into()),
            _ => x.value.map(|v| v.to_string()).unwrap_or_default(),
        };
        let v = x.value.map(crate::sdc_write::num).unwrap_or_default();
        let head = match (&x.name, x.default) {
            (Some(n), _) => format!("{} -name {n}", x.cmd),
            (None, true) => format!("{} -default", x.cmd),
            _ => x.cmd.clone(),
        };
        srcs.push(Src { ty, mm: mm_of(&x.flags), flags: &x.flags, args: &x.args, effect, value: x.value.unwrap_or(0.0), text: text_of(&head, &v, &x.flags, &x.args) });
    }
    let mut out = Vec::new();
    let mut unclassified = Vec::new();
    let mut order_of_type = [0usize; 5];
    for (src, s) in srcs.into_iter().enumerate() {
        let slot = s.ty as usize;
        let order = order_of_type[slot];
        order_of_type[slot] += 1;
        let Some((from, thrus, to)) = points(s.args) else {
            unclassified.push(s.text);
            continue;
        };
        let opt = |v: Vec<Pt>| if v.is_empty() { vec![None] } else { v.into_iter().map(Some).collect() };
        for f in opt(from) {
            for t in opt(to.clone()) {
                out.push(Exc {
                    src,
                    ty: s.ty,
                    order,
                    mm: s.mm,
                    end_rf: end_rf_of(s.flags),
                    from: f.clone(),
                    thrus: thrus.clone(),
                    to: t,
                    effect: s.effect.clone(),
                    value: s.value,
                    text: s.text.clone(),
                });
            }
        }
    }
    (out, unclassified)
}

/// The findings: one per exception that another covers and beats everywhere it applies.
pub(crate) fn overlap_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    let reset = sdc.exceptions.iter().any(|e| e.flags.iter().any(|x| x == "-reset_path"))
        || sdc.path_delays.iter().any(|p| p.flags.iter().any(|x| x == "-reset_path"));
    if reset {
        f.push(Finding {
            severity: Severity::Info,
            code: "exception-overlap-not-checked",
            message: "`-reset_path` removes earlier exceptions, which the overlap check does not model; exception overlaps were not checked".into(),
        });
        return;
    }
    let (all, unclassified) = exceptions(sdc);
    let mut srcs: Vec<usize> = all.iter().map(|e| e.src).collect();
    srcs.dedup();
    for b_src in srcs {
        let parts: Vec<&Exc> = all.iter().filter(|e| e.src == b_src).collect();
        // Every part of B, in every analysis it applies to, must be decided by another command:
        // the parts of a split -from/-to may each lose to a different one.
        let mut worst: Option<(Outcome, &Exc)> = None;
        let decided = parts.iter().all(|b| {
            let ans: Vec<Mm> = [Mm::Max, Mm::Min].into_iter().filter(|an| b.applies(*an)).collect();
            !ans.is_empty()
                && ans.iter().all(|an| {
                    let by = all.iter().filter(|a| a.src != b_src).find_map(|a| decide(a, b, *an).map(|o| (o, a)));
                    if let Some((o, a)) = by {
                        if worst.is_none_or(|(w, _)| o > w) {
                            worst = Some((o, a));
                        }
                    }
                    by.is_some()
                })
        });
        let Some((outcome, a)) = worst.filter(|_| decided) else { continue };
        let b = parts[0];
        let (severity, code, message) = match outcome {
            Outcome::Redundant => (Severity::Info, "exception-redundant", format!("`{}` adds nothing: `{}` covers every path it names and says the same", b.text, a.text)),
            Outcome::Replaced => (
                Severity::Warning,
                "exception-conflict",
                format!("`{}` is replaced by the later `{}`: the same priority on every path it names, a different value, and the reference keeps the later one", b.text, a.text),
            ),
            Outcome::Tighter => (
                Severity::Warning,
                "exception-conflict",
                format!("`{}` never applies: `{}` covers every path it names at the same priority, and the reference takes the tighter value there", b.text, a.text),
            ),
            Outcome::Undefined => (
                Severity::Warning,
                "exception-conflict",
                format!("`{}` and `{}` cover the same paths at the same priority with different values, and the reference's choice between them there is not defined", b.text, a.text),
            ),
            Outcome::Shadowed => (
                Severity::Warning,
                "exception-shadowed",
                format!("`{}` never applies: `{}` covers every path it names and outranks it ({} {} over {} {})", b.text, a.text, a.ty.name(), reach(a), b.ty.name(), reach(b)),
            ),
        };
        f.push(Finding { severity, code, message });
    }
    if let Some(first) = unclassified.first() {
        f.push(Finding {
            severity: Severity::Info,
            code: "exception-overlap-not-checked",
            message: format!(
                "{} exception(s) name objects by a query or bare name the overlap check cannot classify, and were not checked for overlap (the first: `{first}`)",
                unclassified.len()
            ),
        });
    }
}

/// How `a` decides a path of `b`, ordered from the least to the most serious finding.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Outcome {
    /// `a` wins, or ties, and says the same.
    Redundant,
    /// The same type and priority, `a` read later: the reference deleted `b` on these paths.
    Replaced,
    /// The same type and priority, and `a` is the tighter (`tighterThan`).
    Tighter,
    /// The same priority, different values, and no tie-break between them.
    Undefined,
    /// `a` outranks `b` and says something else.
    Shadowed,
}

/// In analysis `an`, on every path `b` names: does `a` decide it, and how? `None` when `a` does
/// not cover `b`, does not apply there, or leaves `b` applied.
fn decide(a: &Exc, b: &Exc, an: Mm) -> Option<Outcome> {
    if !a.applies(an) || !covers(a, b) {
        return None;
    }
    let same = a.ty == b.ty && a.effect == b.effect;
    // Two exact duplicates cover each other: report only the later one.
    if same && a.src > b.src && covers(b, a) {
        return None;
    }
    // Whichever false path wins, the path is false: one covered by another adds nothing, even
    // when it outranks it (no other type can come between two false paths).
    if a.ty == Ty::False && b.ty == Ty::False {
        return Some(Outcome::Redundant);
    }
    let tie = a.ty == b.ty && a.priority(an) == b.priority(an);
    match wins(a, b, an) {
        None if same => Some(Outcome::Redundant),
        None => Some(Outcome::Undefined),
        Some(false) => None,
        Some(true) if same => Some(Outcome::Redundant),
        Some(true) if tie && replaces(a, b) => Some(Outcome::Replaced),
        Some(true) if tie => Some(Outcome::Tighter),
        Some(true) => Some(Outcome::Shadowed),
    }
}

/// The points that set an exception's priority, in the reference's order, for a message.
fn reach(e: &Exc) -> String {
    let mut p = Vec::new();
    if e.from.as_ref().is_some_and(|f| f.class == Class::Pins) {
        p.push("-from pins");
    }
    if e.to.as_ref().is_some_and(|t| t.class == Class::Pins) {
        p.push("-to pins");
    }
    if !e.thrus.is_empty() {
        p.push("-through");
    }
    if e.from.as_ref().is_some_and(|f| f.class == Class::Clocks) {
        p.push("-from clocks");
    }
    if e.to.as_ref().is_some_and(|t| t.class == Class::Clocks) {
        p.push("-to clocks");
    }
    if p.is_empty() {
        "with no points".into()
    } else {
        format!("by {}", p.join(", "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    

    fn codes(text: &str) -> Vec<(&'static str, String)> {
        let sdc = Sdc::parse(&format!("create_clock -name clk -period 10 [get_ports clk]\n{text}")).expect("parse");
        let mut f = Vec::new();
        overlap_findings(&sdc, &mut f);
        f.into_iter().map(|x| (x.code, x.message)).collect()
    }
    fn only(text: &str) -> Vec<&'static str> {
        codes(text).into_iter().map(|(c, _)| c).collect()
    }

    const FP: &str = "-from [get_pins r1/CLK]";
    const TP: &str = "-to [get_pins r2/D]";
    const T1: &str = "-through [get_pins u1/X]";
    const T2: &str = "-through [get_pins u2/X]";

    // Rule: type first — a false path outranks a multicycle on the same points (falsePathPriority
    // 4000 > multiCyclePathPriority 2000). Probe type-false-over-multicycle, both orders.
    #[test]
    fn a_false_path_shadows_a_multicycle_in_either_order() {
        for t in [format!("set_false_path {FP}\nset_multicycle_path 3 {FP}\n"), format!("set_multicycle_path 3 {FP}\nset_false_path {FP}\n")] {
            assert_eq!(only(&t), vec!["exception-shadowed"], "{t}");
        }
    }

    // Rule: a path delay outranks a multicycle (3000 > 2000) and a false path outranks a path
    // delay (4000 > 3000); a path margin under a path delay is never looked at (visitCheckEnd
    // takes the path delay branch). Probes type-delay-over-multicycle, type-false-over-delay and
    // margin-ignored-under-delay.
    #[test]
    fn a_path_delay_shadows_a_multicycle_and_a_margin() {
        // ... but a -setup multicycle also moves the hold check, where set_max_delay does not
        // apply: it keeps an effect, and is not reported
        assert!(only(&format!("set_max_delay 7 {FP}\nset_multicycle_path -setup 3 {FP}\n")).is_empty());
        assert_eq!(only(&format!("set_max_delay 7 {FP}\nset_multicycle_path -hold 1 {FP}\n")), Vec::<&str>::new());
        assert_eq!(only(&format!("set_false_path -setup {FP}\nset_max_delay 7 {FP}\n")), vec!["exception-shadowed"]);
        assert_eq!(only(&format!("set_max_delay 7 {T1}\nset_path_margin -setup 1 {T1}\n")), vec!["exception-shadowed"]);
    }

    // Rule: within a type, -from pins (64) outrank -to pins (32): a -from-and-to exception is
    // covered by a -from one but outranks it, so the narrower is NOT shadowed — the wider is not
    // covered by it either. Probe bits-from-and-to-over-from.
    #[test]
    fn a_narrower_exception_that_outranks_the_wider_is_not_reported() {
        assert!(only(&format!("set_multicycle_path 2 {FP}\nset_multicycle_path 3 {FP} {TP}\n")).is_empty());
    }

    // Rule: equal priority, same shape, read later: the later multicycle REPLACES the earlier
    // (deleteMatchingExceptions), it is not a tighter-wins tie. Probe equal-multicycle-later-
    // replaces: 3 then 2 keeps 2, 2 then 3 keeps 3.
    #[test]
    fn the_later_of_two_equal_multicycles_replaces_the_earlier() {
        let c = codes(&format!("set_multicycle_path 2 {T1}\nset_multicycle_path 3 {T1}\n"));
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].0, "exception-conflict");
        assert!(c[0].1.starts_with("`set_multicycle_path 2"), "{}", c[0].1);
        assert!(c[0].1.contains("replaced by the later"), "{}", c[0].1);
    }

    // Rule: equal priority but a different number of -through points: intersectsPts is false,
    // nothing is deleted, and the tighter wins on the path. A wider multicycle 2 beats a
    // narrower 3; a narrower 2 is not beaten by a wider 3.
    #[test]
    fn different_through_counts_tie_break_on_the_tighter() {
        assert_eq!(only(&format!("set_multicycle_path 2 {T1}\nset_multicycle_path 3 {T1} {T2}\n")), vec!["exception-conflict"]);
        assert!(only(&format!("set_multicycle_path 3 {T1}\nset_multicycle_path 2 {T1} {T2}\n")).is_empty());
    }

    // Rule: two false paths never replace each other and say the same thing: the covered one is
    // redundant, whichever outranks.
    #[test]
    fn a_false_path_inside_another_is_redundant() {
        assert_eq!(only(&format!("set_false_path {FP}\nset_false_path {FP} {TP}\n")), vec!["exception-redundant"]);
        assert_eq!(only("set_false_path -from [get_pins r*/CLK]\nset_false_path -from [get_pins r1/CLK]\n"), vec!["exception-redundant"]);
    }

    // Rule: a -setup multicycle also applies to hold (MultiCyclePath::matches, not exactly), so a
    // -setup false path does not remove it. Probe false-setup-leaves-multicycle-hold.
    #[test]
    fn a_setup_false_path_leaves_a_setup_multicycle_its_hold_effect() {
        assert!(only(&format!("set_false_path -setup {FP}\nset_multicycle_path -setup 3 {FP}\n")).is_empty());
        assert_eq!(only(&format!("set_false_path {FP}\nset_multicycle_path -setup 3 {FP}\n")), vec!["exception-shadowed"]);
    }

    // Rule: a path margin is applied together with a multicycle, not instead of it. Probe
    // margin-applies-with-multicycle.
    #[test]
    fn a_margin_and_a_multicycle_combine() {
        assert!(only(&format!("set_path_margin 1 {T1}\nset_multicycle_path 2 {T1}\n")).is_empty());
    }

    // Rule: a multicycle with -setup outranks one with neither in setup (+2 over +1). The
    // unqualified one still applies to hold, so it is not reported. Probe multicycle-setup-over-
    // unqualified.
    #[test]
    fn a_setup_multicycle_outranks_an_unqualified_one_only_in_setup() {
        assert!(only(&format!("set_multicycle_path -setup 3 {T1}\nset_multicycle_path 2 {T1}\n")).is_empty());
    }

    // A pin that a clock launches, or a cell that owns a pin, is not resolved without the design:
    // the pair is not reported (a missed overlap, never a false one).
    #[test]
    fn objects_of_another_kind_are_not_assumed_to_overlap() {
        assert!(only(&format!("set_false_path -from [get_clocks clk]\nset_multicycle_path 2 {FP}\n")).is_empty());
        assert!(only("set_false_path -from [get_cells r1]\nset_multicycle_path 2 -from [get_pins r1/CLK]\n").is_empty());
    }

    // A -from that names pins and clocks is split into two exceptions (addException); each part
    // must be beaten for the command to be reported.
    #[test]
    fn a_mixed_from_is_reported_only_when_both_parts_lose() {
        let mixed = "set_multicycle_path 2 -from [list [get_pins r1/CLK] [get_clocks clk]]\n";
        assert!(only(&format!("set_false_path {FP}\n{mixed}")).is_empty());
        assert_eq!(only(&format!("set_false_path {FP}\nset_false_path -from [get_clocks clk]\n{mixed}")), vec!["exception-shadowed"]);
    }

    #[test]
    fn reset_path_turns_the_check_off_and_says_so() {
        assert_eq!(only(&format!("set_false_path {FP}\nset_multicycle_path 3 -reset_path {FP}\n")), vec!["exception-overlap-not-checked"]);
    }
}
