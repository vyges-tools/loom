//! What the timer does with one register-to-register path's SETUP check, decided as the reference
//! decides it (SDC scope S4b): for a CDC crossing, whether the timer times it and against what.
//!
//! - **Clock groups first.** A pair of clocks a `set_clock_groups` command separates is never
//!   searched (`Sdc::makeClkGroupExclusions`): untimed. `-allow_paths` separates nothing.
//! - **Then the winning exception** (`VisitPathEnds::visitCheckEnd`): a false path removes the
//!   check, a path delay replaces it, a multicycle moves the capture edge. The winner is chosen by
//!   the rules [`crate::sdc_overlap`] ports (type, then points, then read order or tightness).
//! - **No exception:** timed against the two clocks' edges.
//!
//! An exception's points are matched against the path as the reference matches them:
//! - `-from`: the launch register (`get_cells`), its clock pin (`get_pins`), or its clock. A pin
//!   of the launch register that is not its clock pin is no startpoint
//!   (`Sdc::isExceptionStartpoint`, warned STA-1554): a false path or multicycle from it cuts
//!   nothing; a path delay from it makes an internal startpoint, which is not modelled here.
//! - `-to`: the capture register, its data pin, or its clock.
//! - `-through`: a pin or net the path passes, the launch output and the capture data pin
//!   included. Known only for a path with no logic between the registers.
//!
//! Witnessed by `correlation/harness/sdc/crossing-fate-probe.py` in the internal repo (20 cases).
//!
//! ⚠️ Never guessed: [`Fate::Undecided`] whenever an exception this cannot match (a query, a
//! bare name, a `-through` on a path through logic, an edge-specific point, an internal
//! startpoint) could apply and could beat the winner, or when the winner is not defined (a tie
//! between different values). The hold check is not decided here.

use crate::sdc::{object_matches, Objects, Sdc};
use crate::sdc_overlap::{exceptions, wins, Class, Exc, Mm, Pt, Rf, Ty};

/// One register-to-register path.
#[derive(Debug, Clone)]
pub struct RegPath<'a> {
    pub launch_inst: &'a str,
    /// The launch register's clock pin (`CLK`).
    pub launch_clock_pin: &'a str,
    pub launch_clock: &'a str,
    pub capture_inst: &'a str,
    /// The capture register's data pin (`D`).
    pub capture_data_pin: &'a str,
    pub capture_clock: &'a str,
    /// The pins and nets between the registers, when the path has no logic on it: the launch
    /// output pin, the net, the capture data pin. `None` for a path through logic.
    pub through: Option<Through>,
}

/// What a path with no logic on it passes (`inst/pin` names, and net names).
#[derive(Debug, Clone, Default)]
pub struct Through {
    pub pins: Vec<String>,
    pub nets: Vec<String>,
}

/// The setup check's fate.
#[derive(Debug, Clone, PartialEq)]
pub enum Fate {
    /// Never searched: the named `set_clock_groups` command separates the two clocks.
    ClockGroups(String),
    /// A false path wins: the check is removed.
    FalsePath(String),
    /// A `set_max_delay` wins: the check is against that delay.
    MaxDelay(f64, String),
    /// A multicycle wins: the capture edge moves by its multiplier.
    Multicycle(f64, String),
    /// No exception applies: the check is against the two clocks' edges.
    ClockEdges,
    /// Not decided here; the reason says why.
    Undecided(String),
}

impl Fate {
    /// Does the timer check this path (setup)? `None` when not decided.
    pub fn timed(&self) -> Option<bool> {
        match self {
            Fate::ClockGroups(_) | Fate::FalsePath(_) => Some(false),
            Fate::MaxDelay(..) | Fate::Multicycle(..) | Fate::ClockEdges => Some(true),
            Fate::Undecided(_) => None,
        }
    }
}

/// How a point (or an exception) meets the path.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Hit {
    Yes,
    No,
    /// Could match; this cannot tell.
    Maybe,
}

impl Hit {
    fn and(self, o: Hit) -> Hit {
        match (self, o) {
            (Hit::No, _) | (_, Hit::No) => Hit::No,
            (Hit::Yes, Hit::Yes) => Hit::Yes,
            _ => Hit::Maybe,
        }
    }
    fn or(self, o: Hit) -> Hit {
        match (self, o) {
            (Hit::Yes, _) | (_, Hit::Yes) => Hit::Yes,
            (Hit::No, Hit::No) => Hit::No,
            _ => Hit::Maybe,
        }
    }
}

/// A plain object list: names this can match (no query flags or keys).
fn plain(o: &Objects) -> bool {
    o.flags.is_empty() && o.keys.is_empty() && o.accessor.starts_with("get_") && !o.names.is_empty()
}

fn any_name(o: &Objects, name: &str) -> bool {
    o.names.iter().any(|p| object_matches(p, name))
}

/// The instance part of a pin pattern (`l1/CLK` -> `l1`).
fn inst_part(p: &str) -> &str {
    p.rsplit_once('/').map_or(p, |(i, _)| i)
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    From,
    To,
}

/// Does a -from / -to point name this end of the path?
fn end_hit(pt: &Pt, side: Side, path: &RegPath, ty: Ty) -> Hit {
    let (inst, pin, clock) = match side {
        Side::From => (path.launch_inst, path.launch_clock_pin, path.launch_clock),
        Side::To => (path.capture_inst, path.capture_data_pin, path.capture_clock),
    };
    let full = format!("{inst}/{pin}");
    let mut hit = Hit::No;
    for o in &pt.objs {
        if !plain(o) {
            hit = hit.or(Hit::Maybe);
            continue;
        }
        let h = match (pt.class, o.accessor.as_str()) {
            (Class::Clocks, _) => any_name(o, clock),
            (Class::Pins, "get_cells") => any_name(o, inst),
            (Class::Pins, "get_pins") => {
                if any_name(o, &full) {
                    true
                } else if side == Side::From && ty == Ty::Delay && o.names.iter().any(|p| object_matches(inst_part(p), inst)) {
                    // another pin of the launch register: a path delay starts there
                    // (`recordPathDelayInternalFrom`), a path this does not model
                    hit = hit.or(Hit::Maybe);
                    false
                } else {
                    false
                }
            }
            // a port is neither a register's clock pin nor its data pin
            (Class::Pins, "get_ports") => false,
            _ => {
                hit = hit.or(Hit::Maybe);
                false
            }
        };
        if h {
            hit = Hit::Yes;
        }
    }
    hit
}

/// Does a -through point name a pin or net the path passes?
fn thru_hit(pt: &Pt, path: &RegPath) -> Hit {
    let Some(t) = &path.through else { return Hit::Maybe };
    let mut hit = Hit::No;
    for o in &pt.objs {
        if !plain(o) {
            hit = hit.or(Hit::Maybe);
            continue;
        }
        let h = match o.accessor.as_str() {
            "get_pins" => t.pins.iter().any(|p| any_name(o, p)),
            "get_nets" => t.nets.iter().any(|n| any_name(o, n)),
            "get_ports" => false,
            _ => {
                hit = hit.or(Hit::Maybe);
                false
            }
        };
        if h {
            hit = Hit::Yes;
        }
    }
    hit
}

/// Does exception `e` match the path?
fn exc_hit(e: &Exc, path: &RegPath) -> Hit {
    let mut h = Hit::Yes;
    if let Some(f) = &e.from {
        h = h.and(end_hit(f, Side::From, path, e.ty));
    }
    if let Some(t) = &e.to {
        h = h.and(end_hit(t, Side::To, path, e.ty));
    }
    for t in &e.thrus {
        h = h.and(thru_hit(t, path));
    }
    // an edge-specific point applies to one transition of the path only
    let edge = e.end_rf != Rf::Both || e.from.iter().chain(e.to.iter()).chain(e.thrus.iter()).any(|p| p.rf != Rf::Both);
    if edge && h == Hit::Yes {
        h = Hit::Maybe;
    }
    h
}

/// The first `set_clock_groups` command that separates the two clocks, by the reference's rule
/// ([`crate::sdc::clocks_unrelated`]), as a label.
fn separating_groups(sdc: &Sdc, a: &str, b: &str) -> Option<String> {
    if a == b {
        return None;
    }
    let mut of_kind: std::collections::BTreeMap<&str, usize> = std::collections::BTreeMap::new();
    for g in &sdc.clock_groups {
        let n = of_kind.entry(g.kind.as_str()).or_insert(0);
        *n += 1;
        if crate::sdc::clocks_unrelated(std::slice::from_ref(g), a, b) {
            let kind = format!("set_clock_groups -{}", g.kind);
            return Some(match &g.name {
                Some(name) => format!("{kind} -name {name}"),
                None => format!("{kind} (command {n} of its kind)"),
            });
        }
    }
    None
}

/// The setup check's fate for `path` under `sdc`.
pub fn setup_fate(sdc: &Sdc, path: &RegPath) -> Fate {
    if let Some(g) = separating_groups(sdc, path.launch_clock, path.capture_clock) {
        return Fate::ClockGroups(g);
    }
    let reset = sdc.exceptions.iter().any(|e| e.flags.iter().any(|x| x == "-reset_path"))
        || sdc.path_delays.iter().any(|p| p.flags.iter().any(|x| x == "-reset_path"));
    if reset {
        return Fate::Undecided("`-reset_path` removes earlier exceptions, which this does not model".into());
    }
    let (all, unclassified) = exceptions(sdc);
    let timing: Vec<&Exc> = all.iter().filter(|e| matches!(e.ty, Ty::False | Ty::Delay | Ty::Multicycle) && e.applies(Mm::Max)).collect();
    // a min delay is a hold-side path delay (`applies(Max)` already leaves it out)
    let sure: Vec<&Exc> = timing.iter().copied().filter(|e| exc_hit(e, path) == Hit::Yes).collect();
    let maybe: Vec<&Exc> = timing.iter().copied().filter(|e| exc_hit(e, path) == Hit::Maybe).collect();
    // The winner: one that every other match loses to. Two false paths are one outcome.
    let mut best: Option<&Exc> = None;
    for &c in &sure {
        best = match best {
            None => Some(c),
            Some(b) if wins(c, b, Mm::Max) == Some(true) => Some(c),
            keep => keep,
        };
    }
    if let Some(b) = best {
        let undefined = sure
            .iter()
            .any(|&x| !std::ptr::eq(x, b) && !(x.ty == Ty::False && b.ty == Ty::False) && wins(b, x, Mm::Max) != Some(true) && x.effect != b.effect);
        if undefined {
            return Fate::Undecided(format!("`{}` ties with another exception of a different value, and the reference's choice is not defined", b.text));
        }
    }
    // an exception that might match decides nothing only if the winner beats it
    let threat = maybe.iter().find(|&&m| match best {
        None => true,
        Some(b) => !(m.ty == Ty::False && b.ty == Ty::False) && wins(b, m, Mm::Max) != Some(true),
    });
    if let Some(m) = threat {
        return Fate::Undecided(format!("`{}` may apply to this path, which this cannot tell", m.text));
    }
    if best.is_none() {
        if let Some(u) = unclassified.iter().min() {
            return Fate::Undecided(format!("an exception names objects by a query or bare name this cannot match (one of them: `{u}`)"));
        }
    }
    match best {
        None => Fate::ClockEdges,
        Some(b) => match b.ty {
            Ty::False => Fate::FalsePath(b.text.clone()),
            Ty::Delay => Fate::MaxDelay(b.value, b.text.clone()),
            _ => Fate::Multicycle(b.value, b.text.clone()),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The probe's design: clocks A (10) and B (7); p1 = l1 -> c1 directly (net q1), p2 = l2 ->
    /// and2 g2 -> c2.
    fn p1() -> RegPath<'static> {
        RegPath {
            launch_inst: "l1",
            launch_clock_pin: "CLK",
            launch_clock: "A",
            capture_inst: "c1",
            capture_data_pin: "D",
            capture_clock: "B",
            through: Some(Through { pins: vec!["l1/Q".into(), "c1/D".into()], nets: vec!["q1".into()] }),
        }
    }
    fn p2() -> RegPath<'static> {
        RegPath { launch_inst: "l2", capture_inst: "c2", through: None, ..p1() }
    }

    /// `(p1, p2)` fates under `cmds`, in the probe's words.
    fn fates(cmds: &[&str]) -> (String, String) {
        let s = Sdc::parse(&format!(
            "create_clock -name A -period 10 [get_ports clka]\ncreate_clock -name B -period 7 [get_ports clkb]\n{}\n",
            cmds.join("\n")
        ))
        .expect("sdc");
        let word = |f: Fate| match f {
            Fate::ClockGroups(_) | Fate::FalsePath(_) => "untimed".to_string(),
            Fate::MaxDelay(v, _) => format!("delay {v}"),
            Fate::Multicycle(n, _) => format!("multicycle {n}"),
            Fate::ClockEdges => "edges".to_string(),
            Fate::Undecided(_) => "undecided".to_string(),
        };
        (word(setup_fate(&s, &p1())), word(setup_fate(&s, &p2())))
    }

    fn pair(a: &str, b: &str) -> (String, String) {
        (a.to_string(), b.to_string())
    }

    /// Every probe case (`crossing-fate-probe.py`, 20/20 against the reference). The probe's
    /// "edge 21" is the clocks' own edges here, and "edge 28" a multicycle of 2. Where the probe
    /// times p2 through logic and the exception has a -through, this cannot see p2's pins and
    /// says so.
    #[test]
    fn each_crossing_is_timed_as_the_reference_times_it() {
        const CK: &str = "-from [get_clocks A] -to [get_clocks B]";
        let cases: Vec<(String, (String, String))> = vec![
            (String::new(), pair("edges", "edges")),
            ("set_clock_groups -asynchronous -group A -group B".into(), pair("untimed", "untimed")),
            ("set_clock_groups -asynchronous -group A".into(), pair("untimed", "untimed")),
            ("set_clock_groups -asynchronous -allow_paths -group A -group B".into(), pair("edges", "edges")),
            (format!("set_false_path {CK}"), pair("untimed", "untimed")),
            (format!("set_false_path -hold {CK}"), pair("edges", "edges")),
            (format!("set_false_path -setup {CK}"), pair("untimed", "untimed")),
            (format!("set_max_delay 3 {CK}"), pair("delay 3", "delay 3")),
            (format!("set_max_delay 3 {CK}\nset_false_path -from [get_cells l1]"), pair("untimed", "delay 3")),
            (format!("set_max_delay 3 {CK}\nset_max_delay 5 -from [get_cells l1]"), pair("delay 5", "delay 3")),
            ("set_max_delay 5 -from [get_clocks A]\nset_max_delay 3 -to [get_clocks B]".into(), pair("delay 5", "delay 5")),
            (format!("set_max_delay 3 {CK}\nset_max_delay 5 {CK}"), pair("delay 5", "delay 5")),
            (format!("set_multicycle_path 2 {CK}"), pair("multicycle 2", "multicycle 2")),
            (format!("set_multicycle_path 2 -from [get_cells l2] -to [get_cells c2]\nset_max_delay 3 {CK}"), pair("delay 3", "delay 3")),
            ("set_false_path -through [get_nets q1]".into(), pair("untimed", "undecided")),
            ("set_false_path -through [get_pins c1/D]".into(), pair("untimed", "undecided")),
            ("set_false_path -to [get_pins c1/D]".into(), pair("untimed", "edges")),
            ("set_false_path -from [get_clocks A] -to [get_cells c1]".into(), pair("untimed", "edges")),
            ("set_false_path -from [get_pins l1/Q]".into(), pair("edges", "edges")),
        ];
        for (cmds, want) in cases {
            assert_eq!(fates(&[cmds.as_str()]), want, "{cmds}");
        }
    }

    /// A -through on the direct path that names a pin it does not pass leaves it timed; a path
    /// delay from a non-clock pin of the launch register starts a path inside it, not modelled.
    #[test]
    fn what_this_cannot_see_is_undecided_and_nothing_else() {
        assert_eq!(fates(&["set_false_path -through [get_pins g2/X]"]).0, "edges");
        assert_eq!(fates(&["set_max_delay 3 -from [get_pins l1/Q]"]).0, "undecided");
        // a winner that outranks the one it cannot match still decides
        let (a, b) = fates(&["set_false_path -through [get_pins g2/X]", "set_false_path -from [get_clocks A]"]);
        assert_eq!((a.as_str(), b.as_str()), ("untimed", "untimed"));
        // an edge-specific exception applies to one transition only
        let (r, _) = fates(&["set_false_path -rise_to [get_clocks B]"]);
        assert_eq!(r, "undecided");
        // a query this cannot match, with no winner, is not assumed away
        let (q, _) = fates(&["set_false_path -from [all_registers -clock A]"]);
        assert_eq!(q, "undecided");
        // equal priority, different shapes (one -through and two), equal multipliers but -start
        // and -end: `tighterThan` cannot choose, and the reference's choice is not defined
        let (t, _) = fates(&[
            "set_multicycle_path 2 -start -through [get_nets q1]",
            "set_multicycle_path 2 -end -through [get_nets q1] -through [get_pins c1/D]",
        ]);
        assert_eq!(t, "undecided");
        // a bare name is not classified at all
        assert_eq!(fates(&["set_false_path -from l1"]).0, "undecided");
        // a port is neither a register's clock pin nor its data pin
        assert_eq!(fates(&["set_false_path -from [get_ports clka]", "set_false_path -to [get_ports out1]"]), pair("edges", "edges"));
        let (m, _) = fates(&["set_false_path -from [get_clocks A] -to [get_clocks B] -reset_path"]);
        assert_eq!(m, "undecided");
    }

    /// The labels name the command that decided it, and an undecided fate says which one.
    #[test]
    fn the_fate_names_what_decided_it() {
        let s = Sdc::parse("create_clock -name A -period 10 [get_ports clka]\nset_false_path -through [get_nets q1]\n").expect("sdc");
        match setup_fate(&s, &p2()) {
            Fate::Undecided(why) => assert!(why.contains("set_false_path -through") && why.contains("q1"), "{why}"),
            other => panic!("{other:?}"),
        }
        let s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports clka]\ncreate_clock -name B -period 7 [get_ports clkb]\n\
             set_clock_groups -asynchronous -name io -group A -group B\n",
        )
        .expect("sdc");
        assert_eq!(setup_fate(&s, &p1()), Fate::ClockGroups("set_clock_groups -asynchronous -name io".into()));
        assert_eq!(setup_fate(&s, &p1()).timed(), Some(false));
    }
}
