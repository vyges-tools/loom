//! A generated clock's master, found through the design as the reference finds it
//! (`Genclks::ensureMaster`, search/Genclks.cc): SDC scope S4b.
//!
//! - `-master_clock` names it.
//! - Else a clock defined ON the `-source` pin, other than the generated clock itself.
//! - Else a breadth-first search BACKWARD from the `-source` pin, through wires, combinational
//!   arcs and register clock-to-Q arcs (`GenClkMasterSearchPred`): not through a tristate
//!   enable, a set/clear arc, a latch's enable-to-Q or D-to-Q, a disabled arc or a loop
//!   (`ClkTreeSearchPred::searchThru`). The first pin reached with a clock defined on it gives the
//!   master, and the search stops there.
//!
//! [`update_generated_clks`] then derives each one's waveform from that master, as the reference's
//! `Sta::updateGeneratedClks` does after the read.
//!
//! Witnessed by `correlation/harness/sdc/genclk-master-probe.py` in the internal repo (10 cases).
//!
//! ⚠️ What is NOT reproduced, and so is reported [`Master::Undecided`] rather than guessed:
//! - **The reference's edge order.** When two clocks are reached at the same depth (a clock mux:
//!   the probe's reference takes the mux's `A1` clock), the reference takes the one first in its
//!   fanin edge order, which this does not model.
//! - A pin with two clocks defined on it (`-add`), and two clocks on the `-source` pin.
//! - An instance whose cell is not in the library (a kept block, a missing library), a cell
//!   with more than one state group (a bank, a `statetable`), and any `set_disable_timing` in the
//!   file: the search could pass where this cannot see.
//! - `inferLatchRoles`: a cell with no `ff`/`latch` group whose edge arc pairs with a
//!   combinational D-to-Q arc is a latch to the reference; here its edge arc stays a register
//!   clock-to-Q. No library we have has one.

use crate::liberty::{Arc, Cell, Dir, Lib};
use crate::netlist::Netlist;
use crate::sdc::{Sdc, SdcClock};
use std::collections::{BTreeMap, BTreeSet};

/// A generated clock's master.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Master {
    /// Named by `-master_clock`, defined on `-source`, or found by the search.
    Clock(String),
    /// The search ran to its end and reached no clock: the reference finds no master.
    None,
    /// Not decided here; the reason says why.
    Undecided(String),
}

/// A point of the search: a top-level port, or an instance pin (`inst`, `pin`).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
enum Vertex {
    Port(String),
    Pin(usize, String),
}

/// The netlist as the backward search reads it.
struct Fanin<'a> {
    nl: &'a Netlist,
    lib: &'a Lib,
    /// net -> its drivers: input ports and instance output pins
    drivers: BTreeMap<&'a str, Vec<Vertex>>,
    /// (inst, pin) -> net
    net_of: BTreeMap<(usize, &'a str), &'a str>,
    inst: BTreeMap<&'a str, usize>,
}

impl<'a> Fanin<'a> {
    fn new(nl: &'a Netlist, lib: &'a Lib) -> Self {
        let mut f = Fanin { nl, lib, drivers: BTreeMap::new(), net_of: BTreeMap::new(), inst: BTreeMap::new() };
        for p in &nl.inputs {
            f.drivers.entry(p.as_str()).or_default().push(Vertex::Port(p.clone()));
        }
        for (i, inst) in nl.insts.iter().enumerate() {
            f.inst.insert(&inst.name, i);
            let cell = lib.cells.get(&inst.cell);
            for (pin, net) in &inst.conns {
                f.net_of.insert((i, pin.as_str()), net.as_str());
                if cell.and_then(|c| c.pins.get(pin)).map(|p| p.direction) == Some(Dir::Out) {
                    f.drivers.entry(net.as_str()).or_default().push(Vertex::Pin(i, pin.clone()));
                }
            }
        }
        f
    }

    /// The vertex an SDC object names: a port, or `inst/pin`.
    fn vertex(&self, name: &str) -> Option<Vertex> {
        if self.nl.inputs.iter().chain(&self.nl.outputs).chain(&self.nl.inouts).any(|p| p == name) {
            return Some(Vertex::Port(name.to_string()));
        }
        let (inst, pin) = name.rsplit_once('/')?;
        self.inst.get(inst).map(|&i| Vertex::Pin(i, pin.to_string()))
    }

    fn name(&self, v: &Vertex) -> String {
        match v {
            Vertex::Port(p) => p.clone(),
            Vertex::Pin(i, p) => format!("{}/{p}", self.nl.insts[*i].name),
        }
    }

    /// The vertices one search step back from `v`, or why this cannot say.
    fn fanin(&self, v: &Vertex) -> Result<Vec<Vertex>, String> {
        let Vertex::Pin(i, pin) = v else { return Ok(Vec::new()) }; // a port starts nothing further back
        let inst = &self.nl.insts[*i];
        let Some(cell) = self.lib.cells.get(&inst.cell) else {
            return Err(format!("the search reaches `{}`, whose cell `{}` is not in the library", inst.name, inst.cell));
        };
        let Some(p) = cell.pins.get(pin) else { return Ok(Vec::new()) };
        if p.direction != Dir::Out {
            // a load pin: the wire back to its net's drivers
            let net = self.net_of.get(&(*i, pin.as_str()));
            return Ok(net.and_then(|n| self.drivers.get(n)).cloned().unwrap_or_default());
        }
        let mut out = Vec::new();
        for a in &p.arcs {
            if searched_through(a, cell).map_err(|why| format!("`{}` ({}): {why}", inst.name, inst.cell))? {
                out.push(Vertex::Pin(*i, a.related_pin.clone()));
            }
        }
        Ok(out)
    }
}

/// `GenClkMasterSearchPred::searchThruAllow`, less the arcs `ClkTreeSearchPred::searchThru`
/// excludes: a combinational arc and a register clock-to-Q arc are searched through; a
/// tristate enable or disable, a set/clear, a latch enable-to-Q or D-to-Q, and a check are not.
fn searched_through(a: &Arc, cell: &Cell) -> Result<bool, String> {
    if a.clk_edge.is_some() {
        // `edge_arc_role`: a register clock-to-Q unless the cell's state group is a latch (its
        // enable-to-Q) or the from-pin is no register clock (a set/clear)
        let from_clock = cell.pins.get(&a.related_pin).is_some_and(|p| p.is_reg_clk);
        return match &cell.seq_group {
            Some(g) => Ok(from_clock && !g.latch),
            None if cell.is_seq => Err("more than one state group (a bank or a statetable)".into()),
            None => Ok(from_clock),
        };
    }
    Ok(matches!(a.kind.as_str(), "" | "combinational" | "combinational_rise" | "combinational_fall"))
}

/// The clocks defined on `name` (a port or `inst/pin`), in definition order, other than `skip`.
fn clocks_on<'s>(sdc: &'s Sdc, name: &str, skip: &str) -> Vec<&'s SdcClock> {
    sdc.clocks.iter().filter(|c| c.name != skip && c.sources.iter().any(|s| s == name)).collect()
}

/// The master of `gclk`, as the reference finds it, through `nl` and `lib`.
pub fn master(sdc: &Sdc, gclk: &SdcClock, nl: &Netlist, lib: &Lib) -> Master {
    let Some(g) = &gclk.generated else { return Master::Undecided(format!("`{}` is not a generated clock", gclk.name)) };
    if let Some(m) = &g.master_clock {
        return Master::Clock(m.clone());
    }
    let srcs = &g.source.names;
    // `findClocks(src_pin)` first, as the reference does. The search below would reach the same
    // clocks at its first step (a mutant dropping this survives for that reason); kept so the
    // stages read as the reference's.
    let on: Vec<&SdcClock> = srcs.iter().flat_map(|s| clocks_on(sdc, s, &gclk.name)).collect();
    match on.as_slice() {
        [m] => return Master::Clock(m.name.clone()),
        [] => {}
        _ => return Master::Undecided(format!("{} clocks are defined on the -source of `{}`", on.len(), gclk.name)),
    }
    if !sdc.disabled_timing.is_empty() {
        return Master::Undecided("the file disables timing arcs (set_disable_timing), which the search would respect".into());
    }
    let f = Fanin::new(nl, lib);
    let mut level: Vec<Vertex> = Vec::new();
    for s in srcs {
        match f.vertex(s) {
            Some(v) => level.push(v),
            None => return Master::Undecided(format!("the -source `{s}` of `{}` is not in the design", gclk.name)),
        }
    }
    let mut seen: BTreeSet<Vertex> = level.iter().cloned().collect();
    while !level.is_empty() {
        let mut found: Vec<(String, Vec<&SdcClock>)> = Vec::new();
        for v in &level {
            let name = f.name(v);
            let here = clocks_on(sdc, &name, &gclk.name);
            if !here.is_empty() {
                found.push((name, here));
            }
        }
        match found.as_slice() {
            [] => {}
            [(_, clocks)] if clocks.len() == 1 => return Master::Clock(clocks[0].name.clone()),
            _ => {
                let pins: Vec<&str> = found.iter().map(|(n, _)| n.as_str()).collect();
                return Master::Undecided(format!(
                    "the search from `{}` reaches clocks on `{}` at the same depth; the reference takes the first by its edge order",
                    gclk.name,
                    pins.join("`, `")
                ));
            }
        }
        let mut next = Vec::new();
        for v in &level {
            match f.fanin(v) {
                Ok(vs) => {
                    for u in vs {
                        if seen.insert(u.clone()) {
                            next.push(u);
                        }
                    }
                }
                Err(why) => return Master::Undecided(why),
            }
        }
        level = next;
    }
    Master::None
}

/// What drives a dividing generated clock's pin, traced back through buffers and inverters.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DividerDriver {
    /// A register (or a cell with a state group: a clock gate, a latch).
    Register,
    /// A combinational cell other than a buffer or inverter: `(instance, cell)`.
    Logic(String, String),
    /// A port, a cell outside the library, or a pin not in the design: not judged.
    Unknown,
}

/// The cell that drives `gclk`'s pins, through buffers and inverters (a cell with one input
/// that its output's arcs come from). For a clock that DIVIDES (`-divide_by` above 1, or
/// `-edges`); `None` for any other (a multiply is a PLL, a divide by 1 a mux or a gate).
pub fn divider_driver(gclk: &SdcClock, nl: &Netlist, lib: &Lib) -> Option<DividerDriver> {
    let g = gclk.generated.as_ref()?;
    // (`-multiply_by` excludes `-divide_by` and `-edges` in the reference's argument check)
    let divides = (g.divide_by.is_some_and(|d| d > 1) || !g.edges.is_empty()) && !g.combinational;
    if !divides {
        return None;
    }
    let f = Fanin::new(nl, lib);
    let mut worst = DividerDriver::Register;
    for src in &gclk.sources {
        let d = match f.vertex(src) {
            Some(v) => f.driving_cell(v),
            None => DividerDriver::Unknown,
        };
        match (&worst, &d) {
            (_, DividerDriver::Logic(..)) => return Some(d),
            (DividerDriver::Register, DividerDriver::Unknown) => worst = d,
            _ => {}
        }
    }
    Some(worst)
}

/// Each generated clock's period and waveform from the master the design gives — the reference's
/// `Sta::updateGeneratedClks`, run after the whole file is read: a fixed point over the clocks in
/// definition order, where a generated clock whose waveform is not yet valid is generated from
/// its master ([`master`]) once that master's waveform is valid. A chain resolves whatever order
/// its clocks are defined in.
///
/// As in the reference, every generated clock starts invalid (a `create_clock` is valid when made)
/// and is generated once (`Clock::generate` sets it valid whatever period comes out), so the loop
/// ends. The read ([`Sdc::parse`]) has already generated a clock whose master is `-master_clock`
/// or a clock on its `-source`, perhaps from a master the read could not yet know; this generates
/// it again from that master's final waveform. A clock whose master is not decided here keeps
/// what the read gave it; one with no master keeps period 0. Returns the names of the clocks it
/// generated, in the order it generated them.
pub fn update_generated_clks(sdc: &mut Sdc, nl: &Netlist, lib: &Lib) -> Vec<String> {
    // `Clock::waveformValid()`, by clock index
    let mut valid: Vec<bool> = sdc.clocks.iter().map(|c| c.generated.is_none()).collect();
    let mut generated = Vec::new();
    let mut changed = true;
    while changed {
        changed = false;
        for i in 0..sdc.clocks.len() {
            if valid[i] {
                continue;
            }
            let m = match master(sdc, &sdc.clocks[i], nl, lib) {
                Master::Clock(m) => m,
                Master::Undecided(_) if sdc.clocks[i].period > 0.0 => {
                    // not decided here: what the read generated stands, and is what a clock
                    // generated from this one is generated from
                    valid[i] = true;
                    changed = true;
                    continue;
                }
                _ => continue,
            };
            let Some(mi) = sdc.clocks.iter().position(|c| c.name == m).filter(|&mi| valid[mi]) else { continue };
            let spec = sdc.clocks[i].generated.as_ref().expect("an invalid clock is a generated one");
            let Some((p, w)) = crate::sdc::generate(&sdc.clocks[mi], spec) else { continue };
            let c = &mut sdc.clocks[i];
            c.period = p;
            c.waveform = Some(w);
            generated.push(c.name.clone());
            valid[i] = true;
            changed = true;
        }
    }
    generated
}

impl Fanin<'_> {
    /// The first cell back from `v` that is not a buffer or inverter, judged.
    fn driving_cell(&self, v: Vertex) -> DividerDriver {
        let mut v = v;
        let mut seen = BTreeSet::new();
        loop {
            if !seen.insert(v.clone()) {
                return DividerDriver::Unknown;
            }
            let Vertex::Pin(i, pin) = &v else { return DividerDriver::Unknown };
            let inst = &self.nl.insts[*i];
            let Some(cell) = self.lib.cells.get(&inst.cell) else { return DividerDriver::Unknown };
            let Some(p) = cell.pins.get(pin) else { return DividerDriver::Unknown };
            if p.direction != Dir::Out {
                // a load pin: its net's one driver
                let drivers = self.net_of.get(&(*i, pin.as_str())).and_then(|n| self.drivers.get(n));
                match drivers.map(Vec::as_slice) {
                    Some([d]) => v = d.clone(),
                    _ => return DividerDriver::Unknown,
                }
                continue;
            }
            if cell.is_seq {
                return DividerDriver::Register;
            }
            let from: BTreeSet<&str> = p.arcs.iter().map(|a| a.related_pin.as_str()).collect();
            match from.into_iter().collect::<Vec<_>>().as_slice() {
                [one] => v = Vertex::Pin(*i, one.to_string()),
                _ => return DividerDriver::Logic(inst.name.clone(), inst.cell.clone()),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The probe's cells, reduced to what the search reads: a buffer, a flop (CLK->Q and an arc
    /// from D that is no clock), a mux, a tristate buffer, a latch.
    fn lib() -> Lib {
        Lib::parse(
            r#"library (t) {
  cell (bf) { pin (A) { direction : input; } pin (X) { direction : output; function : "A";
    timing () { related_pin : "A"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
  cell (dff) { ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; }
    pin (CLK) { direction : input; clock : true; } pin (D) { direction : input;
      timing () { related_pin : "CLK"; timing_type : setup_rising; } }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CLK"; timing_type : rising_edge; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
  cell (mux) { pin (A0) { direction : input; } pin (A1) { direction : input; } pin (S) { direction : input; }
    pin (X) { direction : output; function : "(A0&!S)|(A1&S)";
      timing () { related_pin : "A0"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } timing () { related_pin : "A1"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } timing () { related_pin : "S"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
  cell (tbuf) { pin (A) { direction : input; } pin (TE_B) { direction : input; }
    pin (Z) { direction : output; function : "A"; three_state : "TE_B";
      timing () { related_pin : "A"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } timing () { related_pin : "TE_B"; timing_type : three_state_enable; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
  cell (dffr) { ff (IQ, IQN) { clocked_on : "CLK"; next_state : "D"; clear : "!RN"; }
    pin (CLK) { direction : input; clock : true; } pin (D) { direction : input; } pin (RN) { direction : input; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "CLK"; timing_type : rising_edge; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } }
      timing () { related_pin : "RN"; timing_type : rising_edge; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
  cell (lat) { latch (IQ, IQN) { enable : "GATE"; data_in : "D"; }
    pin (GATE) { direction : input; } pin (D) { direction : input; }
    pin (Q) { direction : output; function : "IQ";
      timing () { related_pin : "GATE"; timing_type : rising_edge; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } timing () { related_pin : "D"; cell_rise (scalar) { values ("0.1"); } cell_fall (scalar) { values ("0.1"); } } } }
}"#,
        )
        .expect("library")
    }

    fn nl() -> Netlist {
        crate::netlist::parse(
            "module probe (clka, clkb, noclk, sel);\n\
             input clka, clkb, noclk, sel;\n\
             wire ca, gq, mx, nx, tz, lq;\n\
             bf ub (.A(clka), .X(ca));\n\
             dff rdiv (.CLK(clka), .D(clkb), .Q(gq));\n\
             mux um (.A0(clka), .A1(clkb), .S(sel), .X(mx));\n\
             bf ux (.A(noclk), .X(nx));\n\
             tbuf ut (.A(clkb), .TE_B(clka), .Z(tz));\n\
             lat rl (.GATE(clka), .D(clkb), .Q(lq));\n\
             dffr rr (.CLK(noclk), .D(sel), .RN(clka), .Q(rq));\n\
             kept uk (.A(clka), .X(kx));\n\
             endmodule\n",
        )
        .expect("netlist")
    }

    /// `Sta::updateGeneratedClks` (witnessed by the probe's chain cases): H's master A is found
    /// only through the design (ub/X), G's is H (on G's -source); G = 10 x 2 x 2 = 40 whichever is
    /// defined first. A clock with no master (N) keeps period 0.
    #[test]
    fn a_generated_clock_takes_its_waveform_from_the_master_the_design_gives() {
        for (first, second) in [("H", "G"), ("G", "H")] {
            let def = |n: &str| match n {
                "H" => "create_generated_clock -name H -divide_by 2 -source [get_pins ub/X] [get_pins rdiv/CLK]\n",
                _ => "create_generated_clock -name G -divide_by 2 -source [get_pins rdiv/CLK] [get_pins r1/CLK]\n",
            };
            let mut s = Sdc::parse(&format!(
                "create_clock -name A -period 10 [get_ports clka]\n{}{}\
                 create_generated_clock -name N -divide_by 2 -source [get_pins ux/X] [get_pins r2/CLK]\n",
                def(first),
                def(second)
            ))
            .expect("sdc");
            assert!(s.clocks.iter().filter(|c| c.generated.is_some()).all(|c| c.period == 0.0), "the read alone finds none");
            let done = update_generated_clks(&mut s, &nl(), &lib());
            let period = |n: &str| s.clocks.iter().find(|c| c.name == n).expect("clock").period;
            assert_eq!((period("H"), period("G"), period("N")), (20.0, 40.0, 0.0), "{first} first");
            assert_eq!(done, ["H", "G"], "a clock waits for its master: {first} first");
            let w = s.clocks.iter().find(|c| c.name == "G").and_then(|c| c.waveform.clone());
            assert_eq!(w, Some(vec![0.0, 20.0]));
        }
    }

    /// `Clock::generate` sets the waveform valid whatever period comes out: a clock generated to
    /// no period (edges shifted onto each other) is generated once and the loop ends, and one the
    /// read generated is generated again from its master's final waveform.
    #[test]
    fn each_generated_clock_is_generated_once() {
        let mut s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_generated_clock -name Z -edges {1 2 3} -edge_shift {0 0 -10} -source [get_ports clka] [get_pins r3/CLK]\n\
             create_generated_clock -name G -divide_by 2 -source [get_pins rdiv/CLK] -master_clock H [get_pins r1/CLK]\n\
             create_generated_clock -name H -divide_by 2 -source [get_pins ub/X] [get_pins rdiv/CLK]\n",
        )
        .expect("sdc");
        let period = |s: &Sdc, n: &str| s.clocks.iter().find(|c| c.name == n).expect("clock").period;
        assert_eq!((period(&s, "Z"), period(&s, "G")), (0.0, 0.0), "the read: Z spans nothing, G's master H is not yet known");
        let done = update_generated_clks(&mut s, &nl(), &lib());
        assert_eq!(done, ["Z", "H", "G"]);
        assert_eq!((period(&s, "Z"), period(&s, "H"), period(&s, "G")), (0.0, 20.0, 40.0));
    }

    /// A clock whose master is not decided here (two clocks on its -source) keeps the read's
    /// waveform, and a clock generated from it is generated from that.
    #[test]
    fn a_clock_not_decided_here_keeps_what_the_read_gave_it() {
        let mut s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_clock -name A2 -period 4 -add [get_ports clka]\n\
             create_generated_clock -name V -divide_by 2 -source [get_pins rdiv/CLK] -master_clock U [get_pins r1/CLK]\n\
             create_generated_clock -name U -divide_by 2 -source [get_ports clka] [get_pins rdiv/CLK]\n",
        )
        .expect("sdc");
        let u = s.clocks.iter().find(|c| c.name == "U").expect("U");
        assert!(matches!(master(&s, u, &nl(), &lib()), Master::Undecided(_)), "two clocks on the -source");
        let read_u = u.period;
        assert!(read_u > 0.0, "the read generated U from one of them");
        assert_eq!(update_generated_clks(&mut s, &nl(), &lib()), ["V"]);
        let period = |n: &str| s.clocks.iter().find(|c| c.name == n).expect("clock").period;
        assert_eq!((period("U"), period("V")), (read_u, read_u * 2.0));
    }

    fn master_of(extra: &str, src: &str) -> Master {
        let s = Sdc::parse(&format!(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_clock -name B -period 7 [get_ports clkb]\n{extra}\
             create_generated_clock -name G -divide_by 2 -source {src} [get_pins r1/CLK]\n"
        ))
        .expect("sdc");
        let g = s.clocks.iter().find(|c| c.name == "G").expect("G");
        master(&s, g, &nl(), &lib())
    }

    /// Rules, each a case of the reference probe `genclk-master-probe.py` (2026-11-07).
    #[test]
    fn the_master_is_found_as_the_reference_finds_it() {
        let a = Master::Clock("A".into());
        // on-source-pin, through-buffer, through-clk-to-q-not-d (D is B's, not searched)
        assert_eq!(master_of("", "[get_ports clka]"), a);
        assert_eq!(master_of("", "[get_pins ub/X]"), a);
        assert_eq!(master_of("", "[get_pins rdiv/Q]"), a);
        // not-through-tristate-enable: the enable is A's, the data B's
        assert_eq!(master_of("", "[get_pins ut/Z]"), Master::Clock("B".into()));
        // no-clock-upstream, not-through-latch-d (neither enable-to-Q nor D-to-Q)
        assert_eq!(master_of("", "[get_pins ux/X]"), Master::None);
        assert_eq!(master_of("", "[get_pins rl/Q]"), Master::None);
        // an edge arc from a pin that is no register clock (`edge_arc_role`: the set/clear RN) is
        // not searched through: rr's clock is unclocked, its reset is A's
        assert_eq!(master_of("", "[get_pins rr/Q]"), Master::None);
        // `-master_clock` names it, whatever the design reaches (`Clock::initGeneratedClk`)
        assert_eq!(master_of("", "[get_pins ub/X] -master_clock B"), Master::Clock("B".into()));
        // defined-on-source-wins
        assert_eq!(master_of("create_clock -name C -period 3 [get_pins ub/X]\n", "[get_pins ub/X]"), Master::Clock("C".into()));
    }

    /// Through the design, a master the file alone cannot name feeds the group checks: G (from
    /// `ub/X`, A's buffer) is in no group of a command that groups A; and a generated clock whose
    /// search reaches no clock is reported.
    #[test]
    fn the_group_checks_use_the_master_the_design_gives() {
        let s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_clock -name B -period 7 [get_ports clkb]\n\
             create_generated_clock -name G -divide_by 2 -source [get_pins ub/X] [get_pins r1/CLK]\n\
             create_generated_clock -name N -divide_by 2 -source [get_pins ux/X] [get_pins r4/CLK]\n\
             set_clock_groups -asynchronous -group A -group B\n",
        )
        .expect("sdc");
        let codes = |design: bool| {
            let mut f = Vec::new();
            if design {
                crate::sdc_clock_relations::clock_relation_findings_with_design(&s, &nl(), &lib(), &mut f);
            } else {
                crate::sdc_clock_relations::clock_relation_findings(&s, &mut f);
            }
            f.into_iter().map(|x| (x.code, x.message)).collect::<Vec<_>>()
        };
        let without: Vec<&str> = codes(false).iter().map(|x| x.0).collect();
        assert_eq!(without, ["clock-relations-not-checked"]);
        let with = codes(true);
        let got: Vec<&str> = with.iter().map(|x| x.0).collect();
        assert_eq!(got, ["clock-generated-no-master", "clock-generated-ungrouped"], "{with:?}");
        assert!(with[0].1.contains("`N` (`ux/X`)") && with[1].1.contains("`G` is generated from `A`"), "{with:?}");
    }

    /// A dividing generated clock is judged by the cell that drives its pin, through buffers:
    /// a register is right, logic (a mux) can glitch; a multiply or a divide by 1 is not judged.
    #[test]
    fn a_divider_is_judged_by_its_driving_cell() {
        let s = Sdc::parse(
            "create_clock -name A -period 10 [get_ports clka]\n\
             create_generated_clock -name R -divide_by 2 -source [get_ports clka] [get_pins rdiv/Q]\n\
             create_generated_clock -name RB -divide_by 2 -source [get_ports clka] [get_pins r1/CLK]\n\
             create_generated_clock -name M -divide_by 2 -source [get_ports clka] [get_pins um/X]\n\
             create_generated_clock -name M1 -divide_by 1 -source [get_ports clka] [get_pins um/X]\n\
             create_generated_clock -name P -multiply_by 2 -source [get_ports clka] [get_pins um/X]\n",
        )
        .expect("sdc");
        let n = crate::netlist::parse(
            "module d (clka, clkb, sel);\n input clka, clkb, sel;\n wire gq, gb, mx;\n\
             dff rdiv (.CLK(clka), .D(clkb), .Q(gq));\n bf ub (.A(gq), .X(gb));\n dff r1 (.CLK(gb), .D(sel), .Q(q1));\n\
             mux um (.A0(clka), .A1(clkb), .S(sel), .X(mx));\nendmodule\n",
        )
        .expect("netlist");
        let at = |name: &str| divider_driver(s.clocks.iter().find(|c| c.name == name).expect("clock"), &n, &lib());
        assert_eq!(at("R"), Some(DividerDriver::Register));
        assert_eq!(at("RB"), Some(DividerDriver::Register), "through the buffer on the divider's output");
        assert_eq!(at("M"), Some(DividerDriver::Logic("um".into(), "mux".into())));
        assert_eq!((at("M1"), at("P")), (None, None));
    }

    /// What the search does not reproduce is said, not guessed: two clocks at one depth (the
    /// reference's edge order, the probe's mux-first-found), a cell outside the library, a file
    /// that disables arcs.
    #[test]
    fn what_is_not_reproduced_is_undecided() {
        let undecided = |m: Master| matches!(m, Master::Undecided(_));
        assert!(undecided(master_of("", "[get_pins um/X]")));
        assert!(undecided(master_of("", "[get_pins uk/X]")));
        assert!(undecided(master_of("set_disable_timing [get_cells ub]\n", "[get_pins ub/X]")));
        assert!(undecided(master_of("", "[get_pins nosuch/X]")));
    }
}
