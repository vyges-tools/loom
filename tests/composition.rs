//! **Do the parsers agree with each other, and with what we write?**
//!
//! The per-format suites (`spef_grammar`, `lef_robustness`, …) ask whether one reader is right
//! about one file. That is necessary and it is not sufficient: every reader here can be
//! individually correct while the design they describe between them is incoherent. A net is
//! called `q[0]` by one and `q\[0\]` by another; a bus bit is `r_nxt[2]` in a netlist and
//! `r_nxt` in a dump; a hierarchy divides on `/` in one file and `.` in the next. Each parser
//! passes its own tests. The design still does not join up.
//!
//! Two properties are checked here, and neither needs a golden file:
//!
//! 1. **The cycle.** Read a SPEF, write it, read it back, write it again. The second text must
//!    equal the first (a fixed point), and the second parse must agree with the first about
//!    every net, cap, resistor, pin and coupling pair — not about three fields someone chose.
//!    Anything the writer drops or the reader cannot recover shows up as a difference.
//!
//! 2. **The join.** Load a design from all of its files at once and ask whether they refer to
//!    the same circuit: SPEF nets that are nets of the netlist, instance pins that exist on the
//!    Liberty cell, SDC clocks on ports that exist. This is the only place an interface defect
//!    can be seen, because by construction it lives between two readers and not inside either.

use std::collections::{BTreeMap, BTreeSet};

use vyges_loom::design::Design;

mod common;
use common::survives_the_cycle;

fn ex(name: &str) -> String {
    format!("{}/examples/top/{name}", env!("CARGO_MANIFEST_DIR"))
}

#[test]
fn the_example_spef_survives_a_write_and_reread() {
    let text = std::fs::read_to_string(ex("top.spef")).expect("examples/top/top.spef");
    survives_the_cycle(&text, "examples/top/top.spef");
}

/// The constructs the grammar suite found defects in, put through the cycle rather than through
/// one reader: escaped names, coupling, reduced networks, unit scaling, triplets.
#[test]
fn the_awkward_constructs_survive_the_cycle() {
    let cases: &[(&str, &str)] = &[
        (
            "escaped hierarchical and bussed names",
            "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
             *NAME_MAP\n*1 u_a\\.q\\[0\\]\n*2 u_b\n\n\
             *D_NET *1 10\n*CONN\n*I *2:A I\n*CAP\n1 *1:1 8\n*RES\n1 *1:1 *2:A 40\n*END\n",
        ),
        (
            "coupling between two nets",
            "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
             *NAME_MAP\n*1 aggressor\n*2 victim\n\n\
             *D_NET *1 12\n*CAP\n1 *1:1 6\n2 *1:1 *2:1 3\n*END\n\
             *D_NET *2 9\n*CAP\n1 *2:1 6\n2 *2:1 *1:1 3\n*END\n",
        ),
        (
            "scaled units",
            "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n*T_UNIT 1 PS\n*C_UNIT 1 PF\n*R_UNIT 1 KOHM\n\
             *NAME_MAP\n*1 n\n*2 m\n\n\
             *D_NET *1 0.01\n*CAP\n1 *1:1 0.008\n*RES\n1 *1:1 *2:1 0.02\n*END\n",
        ),
        (
            // A writer formatting to six decimal PLACES rounds this to 0.041625, and anything
            // below 5e-7 to 0.000000 — which trims to 0 and deletes the capacitor. Small
            // coupling capacitors live exactly here.
            "capacitance past the sixth decimal place",
            "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
             *NAME_MAP\n*1 tiny\n*2 other\n\n\
             *D_NET *1 0.0416254\n*CAP\n1 *1:1 0.0416254\n2 *1:1 *2:1 0.00000017\n*END\n\
             *D_NET *2 0.00000017\n*CAP\n1 *2:1 *1:1 0.00000017\n*END\n",
        ),
        (
            "a net with no parasitics at all",
            "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
             *NAME_MAP\n*1 dangling\n\n*D_NET *1 0\n*END\n",
        ),
    ];
    for (what, text) in cases {
        survives_the_cycle(text, what);
    }
}

// ---------------------------------------------------------------------------------------------
// The join: do the formats describe the same circuit?
// ---------------------------------------------------------------------------------------------

fn loaded() -> Design {
    let mut d = Design::new();
    for f in ["top.v", "cells.lib", "top.sdc", "top.spef"] {
        d.load(&ex(f)).unwrap_or_else(|e| panic!("{f}: {e:?}"));
    }
    d
}

/// Every net the SPEF carries parasitics for has to BE a net of the netlist. A name that
/// resolves nowhere is not a loud failure — the net simply carries no parasitics, and the
/// timing that follows is computed as though the wire were ideal.
///
/// This is where a defect between two readers surfaces: both are right about their own file and
/// disagree about what a name looks like.
#[test]
fn every_net_in_the_spef_is_a_net_of_the_netlist() {
    let d = loaded();
    let nl = d.netlist.as_ref().expect("netlist");
    let spef = d.spef.as_ref().expect("spef");

    let mut known: BTreeSet<&str> = BTreeSet::new();
    for i in &nl.insts {
        for (_, net) in &i.conns {
            known.insert(net.as_str());
        }
    }
    for p in nl.inputs.iter().chain(nl.outputs.iter()) {
        known.insert(p.as_str());
    }
    for (a, b) in &nl.aliases {
        known.insert(a.as_str());
        known.insert(b.as_str());
    }

    let orphans: Vec<&String> = spef.nets.keys().filter(|n| !known.contains(n.as_str())).collect();
    assert!(
        orphans.is_empty(),
        "{} SPEF net(s) name nothing in the netlist, so they carry parasitics for no wire: {:?}\n\
         (netlist knows: {:?})",
        orphans.len(),
        orphans,
        known
    );
}

/// The other direction of the same join: every instance pin the SPEF hooks a net up to must be
/// an instance and a pin the netlist actually has.
#[test]
fn every_pin_the_spef_hooks_up_exists_in_the_netlist() {
    let d = loaded();
    let nl = d.netlist.as_ref().expect("netlist");
    let spef = d.spef.as_ref().expect("spef");

    let insts: BTreeMap<&str, &vyges_loom::netlist::Inst> =
        nl.insts.iter().map(|i| (i.name.as_str(), i)).collect();
    let mut bad = Vec::new();
    for (net, rc) in &spef.nets {
        for (inst, pin, _) in &rc.pins {
            match insts.get(inst.as_str()) {
                None => bad.push(format!("{net}: no instance `{inst}`")),
                Some(i) if !i.conns.iter().any(|(p, _)| p == pin) => {
                    bad.push(format!("{net}: instance `{inst}` has no pin `{pin}`"))
                }
                _ => {}
            }
        }
    }
    assert!(bad.is_empty(), "{} SPEF connection(s) name nothing in the netlist:\n  {}", bad.len(), bad.join("\n  "));
}

/// Every cell the netlist instantiates must be a cell the Liberty defines. A missing one is not
/// a parse error anywhere: the netlist reader is content, the Liberty reader is content, and the
/// instance simply has no timing — which reads downstream as a design that meets timing.
#[test]
fn every_cell_the_netlist_instantiates_is_in_the_liberty() {
    let d = loaded();
    let nl = d.netlist.as_ref().expect("netlist");
    let known: BTreeSet<&str> =
        d.libs.iter().flat_map(|l| l.cells.keys().map(String::as_str)).collect();
    let missing: BTreeSet<&str> =
        nl.insts.iter().map(|i| i.cell.as_str()).filter(|c| !known.contains(c)).collect();
    assert!(
        missing.is_empty(),
        "{} cell type(s) instantiated with no Liberty definition, so they carry no timing: {:?}",
        missing.len(),
        missing
    );
}

/// And every pin the netlist connects must exist on that Liberty cell. A pin named slightly
/// differently by the two files (case, an escaped bus index) leaves the arc unfound rather than
/// unreported.
#[test]
fn every_pin_the_netlist_connects_exists_on_the_liberty_cell() {
    let d = loaded();
    let nl = d.netlist.as_ref().expect("netlist");
    let mut bad = Vec::new();
    for i in &nl.insts {
        let Some(cell) = d.libs.iter().find_map(|l| l.cells.get(&i.cell)) else { continue };
        for (pin, _) in &i.conns {
            if !cell.pins.contains_key(pin) {
                bad.push(format!("{}/{} : `{}` has no pin `{pin}`", i.name, pin, i.cell));
            }
        }
    }
    assert!(bad.is_empty(), "{} connection(s) name a pin the cell does not have:\n  {}", bad.len(), bad.join("\n  "));
}

/// Every port an SDC constrains must be a port of the design. A clock defined on a port that
/// does not exist constrains nothing, and an unconstrained clock reports no violations at all.
#[test]
fn every_port_the_sdc_constrains_is_a_port_of_the_netlist() {
    let d = loaded();
    let nl = d.netlist.as_ref().expect("netlist");
    let sdc = d.sdc.as_ref().expect("sdc");
    let ports: BTreeSet<&str> =
        nl.inputs.iter().chain(nl.outputs.iter()).map(String::as_str).collect();
    // A clock's source is a port OR an internal `inst/pin`; only the former is checkable here.
    let mut missing: Vec<String> = sdc
        .clocks
        .iter()
        .filter(|c| !c.source.is_empty() && !c.source.contains('/') && !ports.contains(c.source.as_str()))
        .map(|c| format!("clock `{}` on `{}`", c.name, c.source))
        .collect();
    // The same for every port an input/output delay budgets, minus the `all_inputs` forms,
    // which name no port of their own.
    for d in sdc.input_delays.iter().chain(sdc.output_delays.iter()) {
        if d.default {
            continue;
        }
        for p in &d.ports {
            if !p.contains('/') && !ports.contains(p.as_str()) {
                missing.push(format!("io delay on `{p}`"));
            }
        }
    }
    assert!(
        missing.is_empty(),
        "{} constraint(s) name a port the design does not have: {:?} (ports: {:?})",
        missing.len(),
        missing,
        ports
    );
}

/// **Every net of the netlist has to be findable in the activity dump.** This is the join that
/// power estimation walks: a netlist net is looked up by name in the VCD/SAIF, and a net that
/// does not resolve is not an error — it silently contributes zero switching, so the design
/// reports less dynamic power than it burns.
///
/// The trap is naming, and it is not hypothetical. A dumper that writes a vector as one 1-bit
/// `$var` per bit (ModelSim, Quartus) made every bit of every bus resolve to nothing, because
/// the reader dropped the bit-select and merged them onto one net called `d`. Each reader was
/// right about its own file. Only asking them about the same design shows it.
#[test]
fn every_net_of_the_netlist_is_findable_in_the_activity_dump() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/join");
    let nl = vyges_loom::netlist::load(&format!("{dir}/bus_top.v")).expect("netlist");
    let v = vyges_loom::vcd::Vcd::load(&format!("{dir}/bus_top.vcd")).expect("vcd");

    let mut nets: BTreeSet<&str> = BTreeSet::new();
    for i in &nl.insts {
        for (_, n) in &i.conns {
            nets.insert(n.as_str());
        }
    }
    assert!(nets.contains("d[3]"), "the fixture must have bussed nets: {nets:?}");

    let unresolved: Vec<&str> = nets.iter().copied().filter(|n| v.idx.resolve(n).is_none()).collect();
    assert!(
        unresolved.is_empty(),
        "{} netlist net(s) resolve to no signal in the dump of the same run, so they count as \
         never switching: {:?}",
        unresolved.len(),
        unresolved
    );

    // And the join carries the right numbers, not merely a name that resolves: d[3] is dumped
    // one bit per $var, q[3:0] as a whole vector, and both are read per bit.
    assert_eq!(v.idx.resolve("d[3]"), Some(2), "dumped bit by bit: 0 -> 1 -> 0");
    // dumped whole: b0000 -> b1011 -> b0100, so bit 2 moves once and the rest move twice
    assert_eq!(v.idx.resolve("q[3]"), Some(2));
    assert_eq!(v.idx.resolve("q[2]"), Some(1));
    assert_eq!(v.idx.resolve("q[1]"), Some(2));
    assert_eq!(v.idx.resolve("q[0]"), Some(2));
}
