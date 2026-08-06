// SPDX-License-Identifier: Apache-2.0
//! How much of a design its inputs actually cover.
//!
//! Every reader in this crate has the same exposure: a file that parses **without error** but
//! yields less than the design needs produces a run that succeeds and reports numbers, with
//! nothing saying that part of the design went untimed, unextracted or unpowered. Two real
//! defects have taken exactly that shape — a SPEF that matched no net because the reader
//! resolved only `*NAME_MAP` names, and Liberty timing groups that were recognised and then
//! discarded, leaving whole check categories silently unexercised.
//!
//! This module computes the numbers. It deliberately does **not** decide what they mean:
//!
//! - loom is the data plane and takes no dependency on the events crate, so it cannot emit;
//! - and the verdict is genuinely engine-specific. Thin SPEF coverage is a warning to a timer
//!   and irrelevant to a netlist lint; instances with no timing view are the overwhelming
//!   majority on a routed design and a *fact*, not a complaint.
//!
//! So each engine reads these counts and applies its own calibration. That split is the point:
//! the counting is shared so it cannot drift between engines, and the judgement stays where the
//! domain knowledge is.
use crate::liberty::{Dir, Lib};
use crate::netlist::Netlist;
use crate::spef::Spef;
use std::collections::BTreeSet;

/// Instances split by whether the timer/analyser can do anything with them.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct NetlistCoverage {
    pub instances: usize,
    pub masters: usize,
    /// No connections, or power/ground only, and absent from the library: fill, tap, decap and
    /// antenna diodes. Legitimately present and legitimately skipped — on a post-route design
    /// they are usually most of the instance count.
    pub physical_only: usize,
    /// Absent from the library **and** carrying real signal pins. These are the ones an analysis
    /// cannot be trusted without.
    pub unresolved_signal: usize,
}

impl NetlistCoverage {
    /// Instances an analysis can actually act on.
    pub fn analysable(&self) -> usize {
        self.instances
            .saturating_sub(self.physical_only + self.unresolved_signal)
    }
}

/// Whether a pin name is a power/ground rail.
///
/// Name-based because a netlist alone does not say; the library would, but these are precisely
/// the instances whose master is *not* in the library. Covers the sky130, gf180 and generic
/// spellings, plus well/body-bias pins.
pub fn is_power_pin(pin: &str) -> bool {
    let p = pin.trim_start_matches('\\').to_ascii_uppercase();
    const RAILS: &[&str] = &[
        "VPWR", "VGND", "VPB", "VNB", "VDD", "VSS", "VDDA", "VSSA", "VCC", "VEE", "VPP", "GND",
        "VNEG", "VCCD", "VCCD1", "VCCD2", "VSSD", "VSSD1", "VSSD2", "VDDIO", "VSSIO", "VDDPST",
        "VSSPST", "KAPWR", "VSWITCH", "VCCHIB", "VNW", "VPW", "VNWELL", "VPWELL", "VWELL", "VSUBS",
        "VBP", "VBN", "VBB", "VBG", "VB", "VBODY", "AVDD", "AVSS", "DVDD", "DVSS", "VPWRIN",
    ];
    RAILS.contains(&p.as_str())
        || p.starts_with("VDD")
        || p.starts_with("VSS")
        || p.starts_with("VPWR")
        || p.starts_with("VGND")
}

pub fn netlist(nl: &Netlist, lib: &Lib) -> NetlistCoverage {
    let mut masters: BTreeSet<&str> = BTreeSet::new();
    let (mut physical_only, mut unresolved_signal) = (0usize, 0usize);
    for i in &nl.insts {
        masters.insert(i.cell.as_str());
        if lib.cell(&i.cell).is_some() {
            continue;
        }
        if i.conns.is_empty() || i.conns.iter().all(|(p, _)| is_power_pin(p)) {
            physical_only += 1;
        } else {
            unresolved_signal += 1;
        }
    }
    NetlistCoverage {
        instances: nl.insts.len(),
        masters: masters.len(),
        physical_only,
        unresolved_signal,
    }
}

/// What the library provides for the cells the design actually uses.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct LibertyCoverage {
    pub cells_in_lib: usize,
    pub resolved_masters: usize,
    /// Masters that **drive a non-constant output and still carry no delay arc** — a path
    /// through one is analysed as though it were free.
    ///
    /// The qualifier is what makes this a signal rather than noise. Tie cells, decaps, antenna
    /// diodes and fill are arc-less *by design*, and on a routed block they are most of what is
    /// arc-less; a check that flagged them would fire on every correct design and teach the
    /// reader to ignore it.
    pub driving_without_arcs: usize,
    /// Sequential masters with no `setup`, `hold`, `recovery` or `removal` anywhere — flops
    /// whose pins can never become endpoints, so their paths are never checked at all.
    pub sequential_without_constraints: usize,
}

pub fn liberty(nl: &Netlist, lib: &Lib) -> LibertyCoverage {
    let masters: BTreeSet<&str> = nl.insts.iter().map(|i| i.cell.as_str()).collect();
    let mut c = LibertyCoverage {
        cells_in_lib: lib.cells.len(),
        ..Default::default()
    };
    for m in masters {
        let Some(cell) = lib.cell(m) else { continue };
        c.resolved_masters += 1;
        let drives = cell
            .pins
            .values()
            .any(|p| p.direction == Dir::Out && !p.is_constant());
        if drives && !cell.pins.values().any(|p| !p.arcs.is_empty()) {
            c.driving_without_arcs += 1;
        }
        if cell.is_seq
            && !cell.pins.values().any(|p| {
                !p.setup.is_empty()
                    || !p.hold.is_empty()
                    || !p.recovery.is_empty()
                    || !p.removal.is_empty()
            })
        {
            c.sequential_without_constraints += 1;
        }
    }
    c
}

/// How much of the design the parasitics describe.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct SpefCoverage {
    pub design_nets: usize,
    pub file_nets: usize,
    pub matched: usize,
    /// Nets the FILE carries that name nothing in the design. The mirror of `matched`, and the
    /// more sensitive of the two: a percentage of design nets covered is a ratio that stays
    /// reassuring while a whole naming convention fails. On a routed block whose netlist reader
    /// kept the backslash of an escaped identifier, coverage read 94.6 % — comfortably above any
    /// threshold anyone would set — while 767 nets carried no parasitics at all.
    pub file_nets_unmatched: usize,
    /// Coupling references whose aggressor names nothing in the design, out of `coupling_refs`.
    /// A timer looks each one up and drops the ones it cannot find, which removes crosstalk from
    /// the analysis with no other symptom. Same block: 4527 of 177680.
    pub coupling_unresolved: usize,
    pub coupling_refs: usize,
}

impl SpefCoverage {
    pub fn percent(&self) -> f64 {
        if self.design_nets == 0 {
            0.0
        } else {
            100.0 * self.matched as f64 / self.design_nets as f64
        }
    }
    /// `file_nets > 0 && matched == 0` — read plenty, none of it belongs to this design. Kept
    /// distinct from "read nothing" because the causes and the fixes differ.
    pub fn read_but_unmatched(&self) -> bool {
        self.file_nets > 0 && self.matched == 0
    }

    /// Does every name in the file correspond to something in the design?
    ///
    /// **This is the check worth gating on**, not a coverage percentage. A design legitimately
    /// has nets a SPEF omits (an extractor skips what it treats as ideal), so `matched` is
    /// always a fraction and any threshold on it is a guess. The converse is not: a parasitic
    /// the file went to the trouble of describing, for a net nothing in the design is called,
    /// means the two files disagree about names — and everything downstream of that disagreement
    /// is silently missing.
    pub fn names_all_correspond(&self) -> bool {
        self.file_nets_unmatched == 0 && self.coupling_unresolved == 0
    }
}

pub fn spef(nl: &Netlist, sp: &Spef) -> SpefCoverage {
    let mut design: BTreeSet<&str> = BTreeSet::new();
    for i in &nl.insts {
        for (_pin, net) in &i.conns {
            design.insert(net.as_str());
        }
    }
    for p in nl.inputs.iter().chain(nl.outputs.iter()) {
        design.insert(p.as_str());
    }
    let (mut coupling_refs, mut coupling_unresolved) = (0usize, 0usize);
    for rc in sp.nets.values() {
        for (agg, _) in &rc.coupling {
            coupling_refs += 1;
            if !design.contains(agg.as_str()) {
                coupling_unresolved += 1;
            }
        }
    }
    SpefCoverage {
        matched: design.iter().filter(|n| sp.nets.contains_key(**n)).count(),
        design_nets: design.len(),
        file_nets: sp.nets.len(),
        file_nets_unmatched: sp.nets.keys().filter(|n| !design.contains(n.as_str())).count(),
        coupling_refs,
        coupling_unresolved,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn power_pins_are_recognised_across_the_spellings_that_appear_in_practice() {
        for p in ["VPWR", "vgnd", "\\VDD", "VSSD1", "VPWELL", "vddio_x"] {
            assert!(is_power_pin(p), "{p} should be a rail");
        }
        for p in ["A", "Y", "CLK", "D", "VOUT", "VALID"] {
            assert!(!is_power_pin(p), "{p} is a signal, not a rail");
        }
    }

    #[test]
    fn analysable_never_underflows_on_inconsistent_counts() {
        // These counts come from separate passes in callers; a saturating subtraction is
        // cheaper than a panic in a diagnostic that exists to make failures visible.
        let c = NetlistCoverage {
            instances: 2,
            physical_only: 5,
            ..Default::default()
        };
        assert_eq!(c.analysable(), 0);
    }

    #[test]
    fn spef_percent_and_the_unmatched_case_are_distinct() {
        let none_read = SpefCoverage {
            design_nets: 10,
            file_nets: 0,
            matched: 0,
            ..Default::default()
        };
        assert!(
            !none_read.read_but_unmatched(),
            "nothing was read — a different failure"
        );
        let read_unmatched = SpefCoverage {
            design_nets: 10,
            file_nets: 99,
            matched: 0,
            ..Default::default()
        };
        assert!(read_unmatched.read_but_unmatched());
        assert_eq!(read_unmatched.percent(), 0.0);
        assert_eq!(
            SpefCoverage::default()
            .percent(),
            0.0
        );
    }
}
