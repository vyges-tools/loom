//! Clock relations a `set_clock_groups` command gets wrong, shown without the design (SDC scope
//! S4, first part).
//!
//! - `clock-generated-ungrouped` (warning): a generated clock whose master (or an earlier
//!   ancestor) is in a group of a command, while it is in no group of that command. The
//!   reference puts in a group exactly the clocks its `-group` names, so:
//!   - with two or more groups, the generated clock is timed against the other groups' clocks,
//!     though its master is not (what `-include_generated_clocks` asks for elsewhere, and the
//!     reference ignores);
//!   - with ONE group, the command separates the group's clocks from every other clock, the
//!     generated clock included: paths between it and its own master go untimed.
//! - `clock-related-declared-async` (warning): two clocks of one family (one generated from the
//!   other, or both from one clock) in different groups of an `-asynchronous` command. Their
//!   paths go untimed. That is right only when every crossing between them is synchronized,
//!   which nothing here can see; the finding names the pair so a person decides.
//!
//! The reference's rules (sdc/Sdc.tcl `set_clock_groups`, `Sdc::makeClkGroupExclusions`,
//! `Genclks::ensureMaster`; witnessed by `correlation/harness/sdc/clock-groups-probe.py` in the
//! internal repo):
//! - two or more groups: a clock of one group is separated from every clock of the others; one
//!   group: each of its clocks from every clock not in it; `-asynchronous -allow_paths`
//!   separates nothing. The exclusive kinds separate the same way.
//! - A generated clock's master is its `-master_clock`; without one, a clock defined on its
//!   `-source` pin. When no clock is defined there, the reference searches back through the
//!   netlist: those generated clocks are not checked here, and are counted in
//!   `clock-relations-not-checked` (info) when a command could concern them.

use crate::liberty::Lib;
use crate::netlist::Netlist;
use crate::sdc::{ClockGroups, Sdc, SdcClock};
use crate::sdc_lint::{Finding, Severity};
use std::collections::BTreeMap;

/// What a generated clock's master is.
#[derive(Debug, PartialEq)]
enum Master {
    /// A `create_clock`.
    None,
    Known(String),
    /// Not decided: without the design, no `-master_clock` and not exactly one clock on its
    /// `-source`; with it, what [`crate::sdc_genclk::master`] leaves undecided.
    Unknown(String),
    /// With the design: the search reached no clock, so the reference finds no master.
    Absent,
}

/// Every clock's master, once: from the file alone, or through the design when one is given.
fn masters(sdc: &Sdc, design: Option<(&Netlist, &Lib)>) -> BTreeMap<String, Master> {
    let clocks = &sdc.clocks;
    let mut out = BTreeMap::new();
    for c in clocks {
        let Some(g) = &c.generated else {
            out.insert(c.name.clone(), Master::None);
            continue;
        };
        let m = if let Some((nl, lib)) = design {
            match crate::sdc_genclk::master(sdc, c, nl, lib) {
                crate::sdc_genclk::Master::Clock(m) => Master::Known(m),
                crate::sdc_genclk::Master::None => Master::Absent,
                crate::sdc_genclk::Master::Undecided(why) => Master::Unknown(why),
            }
        } else if let Some(m) = &g.master_clock {
            Master::Known(m.clone())
        } else {
            let on_source: Vec<&SdcClock> =
                clocks.iter().filter(|m| m.name != c.name && m.sources.iter().any(|s| g.source.names.contains(s))).collect();
            match on_source.as_slice() {
                [m] => Master::Known(m.name.clone()),
                _ => Master::Unknown("its master is found through the design".into()),
            }
        };
        out.insert(c.name.clone(), m);
    }
    out
}

/// `c`'s ancestors, nearest first (its master, its master's master, …), stopping at a cycle or
/// an undecided master; and that undecided master's reason, when the chain stopped there.
fn ancestors<'a>(c: &'a str, masters: &'a BTreeMap<String, Master>) -> (Vec<&'a str>, Option<&'a str>) {
    let mut out: Vec<&str> = Vec::new();
    let mut cur = c;
    loop {
        match masters.get(cur) {
            Some(Master::Known(m)) => {
                if m == c || out.contains(&m.as_str()) {
                    return (out, None);
                }
                out.push(m);
                if !masters.contains_key(m.as_str()) {
                    return (out, Some("its master is not a clock of the file"));
                }
                cur = m;
            }
            Some(Master::Unknown(why)) => return (out, Some(why)),
            _ => return (out, None),
        }
    }
}

/// The clock a family is named by: the last ancestor (a `create_clock`, or where the chain stops).
fn root<'a>(c: &'a str, masters: &'a BTreeMap<String, Master>) -> &'a str {
    ancestors(c, masters).0.last().copied().unwrap_or(c)
}

fn label(g: &ClockGroups, index: usize) -> String {
    let kind = format!("set_clock_groups -{}", g.kind);
    match &g.name {
        Some(n) => format!("`{kind} -name {n}`"),
        None => format!("`{kind}` (command {} of its kind)", index + 1),
    }
}

/// Every clock-relations finding for `sdc`, from the file alone (see the module documentation).
pub fn clock_relation_findings(sdc: &Sdc, f: &mut Vec<Finding>) {
    findings(sdc, None, f);
}

/// [`clock_relation_findings`] with the design: each generated clock's master found as the
/// reference finds it ([`crate::sdc_genclk::master`]), and `clock-generated-no-master` for one
/// whose search reaches no clock.
pub fn clock_relation_findings_with_design(sdc: &Sdc, nl: &Netlist, lib: &Lib, f: &mut Vec<Finding>) {
    findings(sdc, Some((nl, lib)), f);
}

fn findings(sdc: &Sdc, design: Option<(&Netlist, &Lib)>, f: &mut Vec<Finding>) {
    let clocks = &sdc.clocks;
    let masters = masters(sdc, design);
    for c in clocks.iter().filter(|c| masters.get(&c.name) == Some(&Master::Absent)) {
        let src = c.generated.as_ref().map(|g| g.source.names.join(" ")).unwrap_or_default();
        f.push(Finding {
            severity: Severity::Warning,
            code: "clock-generated-no-master",
            message: format!(
                "no clock reaches the -source of `{}` (`{src}`) through the design: the reference finds no master \
                 for it and derives no waveform",
                c.name
            ),
        });
    }
    if let Some((nl, lib)) = design {
        for c in clocks {
            if let Some(crate::sdc_genclk::DividerDriver::Logic(inst, cell)) = crate::sdc_genclk::divider_driver(c, nl, lib) {
                f.push(Finding {
                    severity: Severity::Warning,
                    code: "clock-divider-not-register",
                    message: format!(
                        "`{}` divides its master at a pin driven by `{inst}` (`{cell}`), combinational logic: a divided \
                         clock taken from logic can glitch; divide in a register",
                        c.name
                    ),
                });
            }
        }
    }
    let mut unknown: Vec<(&str, &str)> = Vec::new();
    for (index, g) in sdc.clock_groups.iter().enumerate().filter(|(_, g)| !g.allow_paths) {
        let at = |n: &str| g.groups.iter().position(|grp| grp.iter().any(|x| x == n));
        let what = label(g, index);
        for c in clocks.iter().filter(|c| c.generated.is_some() && at(&c.name).is_none()) {
            let (up, stopped) = ancestors(&c.name, &masters);
            let grouped = up.iter().find_map(|m| at(m).map(|i| (*m, i)));
            let Some((m, i)) = grouped else {
                if let Some(why) = stopped.filter(|_| !unknown.iter().any(|(n, _)| *n == c.name)) {
                    unknown.push((&c.name, why));
                }
                continue;
            };
            let message = if g.groups.len() == 1 {
                format!(
                    "`{}` is generated from `{m}`, in the only group of {what}, and is not in it: the reference \
                     separates `{}` from every clock of the group, `{m}` included, so paths between them go untimed",
                    c.name, c.name
                )
            } else {
                let others: Vec<&str> =
                    g.groups.iter().enumerate().filter(|(j, _)| *j != i).flat_map(|(_, grp)| grp.iter().map(String::as_str)).collect();
                format!(
                    "`{}` is generated from `{m}`, which {what} puts in a group, but is in none: the reference times \
                     `{}` against `{}`, though not `{m}`",
                    c.name,
                    c.name,
                    others.join("`, `")
                )
            };
            f.push(Finding { severity: Severity::Warning, code: "clock-generated-ungrouped", message });
        }
        if g.kind != "asynchronous" || g.groups.len() < 2 {
            continue;
        }
        let mut pairs: Vec<(String, String, String)> = Vec::new();
        for (i, gi) in g.groups.iter().enumerate() {
            for gj in &g.groups[i + 1..] {
                for a in gi {
                    for b in gj {
                        if !masters.contains_key(a) || !masters.contains_key(b) {
                            continue;
                        }
                        let (ra, rb) = (root(a, &masters), root(b, &masters));
                        if a != b && ra == rb {
                            pairs.push((a.clone(), b.clone(), ra.to_string()));
                        }
                    }
                }
            }
        }
        if let Some((a, b, r)) = pairs.first() {
            let more = if pairs.len() > 1 { format!(" ({} pairs in all)", pairs.len()) } else { String::new() };
            f.push(Finding {
                severity: Severity::Warning,
                code: "clock-related-declared-async",
                message: format!(
                    "`{a}` and `{b}` both derive from `{r}`, but {what} puts them in different groups{more}: paths \
                     between them go untimed, which is right only if every crossing between them is synchronized"
                ),
            });
        }
    }
    if let Some((first, why)) = unknown.iter().min() {
        f.push(Finding {
            severity: Severity::Info,
            code: "clock-relations-not-checked",
            message: format!(
                "{} generated clock(s) whose master was not decided, so their place in the clock groups was not \
                 checked (one of them: `{first}`: {why})",
                unknown.len()
            ),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn findings(text: &str) -> Vec<(&'static str, String)> {
        let s = Sdc::parse(text).expect("parse");
        let mut f = Vec::new();
        clock_relation_findings(&s, &mut f);
        f.into_iter().map(|f| (f.code, f.message)).collect()
    }

    const CLOCKS: &str = "create_clock -name A -period 10 [get_ports clka]\n\
                          create_clock -name B -period 7 [get_ports clkb]\n\
                          create_generated_clock -name G -source [get_ports clka] -divide_by 2 [get_pins rdiv/Q]\n";

    /// Rule (probe two-groups): the group holds only the clocks named; G (of A) is timed against B.
    #[test]
    fn a_generated_clock_outside_its_masters_groups_is_timed_against_them() {
        let f = findings(&format!("{CLOCKS}set_clock_groups -asynchronous -group A -group B\n"));
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].0, "clock-generated-ungrouped");
        assert!(f[0].1.contains("times `G` against `B`, though not `A`"), "{}", f[0].1);
    }

    /// Rule (probe one-group-excludes-all): one group separates its clocks from every other clock,
    /// so A from its own G.
    #[test]
    fn one_group_separates_a_master_from_its_own_generated_clock() {
        let f = findings(&format!("{CLOCKS}set_clock_groups -asynchronous -group A\n"));
        assert_eq!(f.len(), 1, "{f:?}");
        assert!(f[0].1.contains("`A` included, so paths between them go untimed"), "{}", f[0].1);
    }

    /// Rule (probe family-in-one-group, allow-paths-excludes-nothing): G grouped with A is right;
    /// `-allow_paths` separates nothing; a master named by `-master_clock` counts as on the source.
    #[test]
    fn a_grouped_family_and_allow_paths_say_nothing() {
        assert_eq!(findings(&format!("{CLOCKS}set_clock_groups -asynchronous -group {{A G}} -group B\n")), []);
        assert_eq!(findings(&format!("{CLOCKS}set_clock_groups -asynchronous -allow_paths -group A -group B\n")), []);
    }

    /// The family is followed through generated clocks of generated clocks, and through
    /// `-master_clock`; two of one family in different -asynchronous groups are named.
    #[test]
    fn related_clocks_declared_asynchronous_are_named() {
        let f = findings(&format!(
            "{CLOCKS}create_generated_clock -name G4 -source [get_pins rdiv/Q] -divide_by 2 [get_pins r4/Q]\n\
             create_generated_clock -name H -source [get_pins pll/CLKIN] -master_clock A -divide_by 3 [get_pins pll/OUT]\n\
             set_clock_groups -asynchronous -group {{A G}} -group {{G4 B}} -group H\n"
        ));
        let codes: Vec<&str> = f.iter().map(|x| x.0).collect();
        assert_eq!(codes, ["clock-related-declared-async"], "{f:?}");
        assert!(f[0].1.starts_with("`A` and `G4` both derive from `A`") && f[0].1.contains("(5 pairs in all)"), "{}", f[0].1);
        // the exclusive kinds are meant for one family (a clock mux): not reported
        let f = findings(&format!("{CLOCKS}set_clock_groups -logically_exclusive -group {{A}} -group {{G B}}\n"));
        assert_eq!(f, []);
    }

    /// Two clocks on the -source pin and no -master_clock: the reference infers one of them
    /// (`Genclks::ensureMaster` counts more than one); which is not decided here, so not checked.
    #[test]
    fn two_clocks_on_the_source_are_not_guessed_between() {
        let f = findings(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_clock -name A2 -period 5 -add [get_ports clka]\n\
             create_clock -name B -period 7 [get_ports clkb]\n\
             create_generated_clock -name G -source [get_ports clka] -divide_by 2 [get_pins rdiv/Q]\n\
             set_clock_groups -asynchronous -group A -group B\n",
        );
        let codes: Vec<&str> = f.iter().map(|x| x.0).collect();
        assert_eq!(codes, ["clock-relations-not-checked"], "{f:?}");
    }

    /// A generated clock whose master is found only through the design is counted, not guessed.
    #[test]
    fn a_master_only_the_design_knows_is_not_checked() {
        let f = findings(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_clock -name B -period 7 [get_ports clkb]\n\
             create_generated_clock -name P -source [get_pins pll/CLKIN] -divide_by 2 [get_pins pll/OUT]\n\
             set_clock_groups -asynchronous -group A -group B\n",
        );
        assert_eq!(f.len(), 1, "{f:?}");
        assert_eq!(f[0].0, "clock-relations-not-checked");
        assert!(f[0].1.contains("(one of them: `P`: its master is found through the design)"), "{}", f[0].1);
    }
}
