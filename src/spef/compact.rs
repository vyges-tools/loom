//! The read-only, analysis form of a parsed SPEF: every name interned once, every node an id.
//!
//! [`Spef`] holds each node as its own `String`, once per grounded cap, twice per resistor, three
//! times per coupling cap and three times per `*CONN` entry: about nine times the file in memory,
//! and every delay walk hashes those strings. A timer needs none of that after the read. This form
//! keeps what the delay walks and the net loads read (caps, resistors, coupling on this net's own
//! nodes, per-aggressor coupling, the pins' nodes) with names held once in one table.
//!
//! The walks are the SAME code as [`NetRc`]'s ([`elmore_of`], [`transient_of`],
//! [`pi_reduce_of`] over [`RcNet`]): the same nodes in the same file order, so the results are
//! the same bits. It is not a writer's form: what only the writer needs (`conns`, the aggressor's
//! node of each coupling cap) is dropped, so keep the [`Spef`] if the file is to be written back.

use super::{elmore_of, pi_reduce_of, transient_of, NetRc, NodeWaves, RcNet, Spef, SpefError};
use std::collections::HashMap;

/// An interned name: an index into [`CompactSpef::names`].
pub type Name = u32;

/// An id no name is given: a node a caller names that no net holds. It is never in a network,
/// so a walk from it finds nothing and a capacitance keyed by it is never read, exactly as an
/// unknown node NAME behaves in [`NetRc`]'s walks.
pub const NO_NAME: Name = Name::MAX;

/// One net's parasitics, names as [`Name`]s. Every vector keeps the file order of the [`NetRc`]
/// it came from.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CompactNet {
    pub name: Name,
    /// as [`NetRc::cap_ff`]
    pub cap_ff: f64,
    /// as [`NetRc::res_ohm`]
    pub res_ohm: f64,
    /// as [`NetRc::coupling_ff`]
    pub coupling_ff: f64,
    /// as [`NetRc::coupling`]: (aggressor net, Cc)
    pub coupling: Vec<(Name, f64)>,
    /// each coupling cap's node on THIS net and its value, as [`NetRc::coupling_nodes`]' second
    /// and fourth fields
    pub cc_own: Vec<(Name, f64)>,
    pub net_node: Name,
    pub ground: Vec<(Name, f64)>,
    pub res: Vec<(Name, Name, f64)>,
    /// (instance, pin, node), as [`NetRc::pins`]
    pub pins: Vec<(Name, Name, Name)>,
}

/// A whole file in the analysis form. Nets are in name order, as [`Spef::nets`] iterates them.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CompactSpef {
    /// every name, each once; ids are given in the order the nets, then each net's fields, are
    /// visited, so the same file gives the same ids
    pub names: Vec<Box<str>>,
    pub nets: Vec<CompactNet>,
    /// as the same fields of [`Spef`]
    pub skipped: usize,
    pub unresolved: usize,
    pub triplets: bool,
    pub reduced: usize,
    pub pnets: usize,
}

impl RcNet for &CompactNet {
    type K = Name;
    fn n_ground(&self) -> usize {
        self.ground.len()
    }
    fn ground(&self, i: usize) -> (Name, f64) {
        self.ground[i]
    }
    fn n_res(&self) -> usize {
        self.res.len()
    }
    fn res(&self, i: usize) -> (Name, Name, f64) {
        self.res[i]
    }
    fn n_cc(&self) -> usize {
        self.cc_own.len()
    }
    fn cc_own(&self, i: usize) -> (Name, f64) {
        self.cc_own[i]
    }
}

impl CompactNet {
    /// [`NetRc::elmore`]: each node's Elmore time constant (ns) in the walk's order; a node the
    /// walk lists twice keeps the later value in `NetRc::elmore`'s map. `pin_cap` gives a load
    /// pin's receiver capacitance (fF) at its node.
    pub fn elmore(&self, driver: Name, xtalk_cap_ff: f64, pin_cap: impl Fn(Name) -> Option<f64>) -> Option<Vec<(Name, f64)>> {
        elmore_of(&self, driver, xtalk_cap_ff, pin_cap)
    }

    /// [`NetRc::transient`]: each node's (delay, slew) in the walk's order.
    pub fn transient(&self, driver: Name, driver_slew_ns: f64, xtalk_cap_ff: f64, th: crate::liberty::Thresholds) -> Option<NodeWaves<Name>> {
        transient_of(&self, driver, driver_slew_ns, xtalk_cap_ff, th)
    }

    /// [`NetRc::pi_reduce`].
    pub fn pi_reduce(&self, driver: Name, xtalk_cap_ff: f64, pin_cap: impl Fn(Name) -> Option<f64>) -> Option<(f64, f64, f64)> {
        pi_reduce_of(&self, driver, xtalk_cap_ff, pin_cap)
    }
}

impl CompactSpef {
    /// The analysis form of `spef`, consuming it: each net's strings are freed as it is converted,
    /// so the two forms are never both whole in memory.
    pub fn from_spef(spef: Spef) -> CompactSpef {
        let Spef { nets, skipped, unresolved, triplets, reduced, pnets } = spef;
        let mut ids: HashMap<String, Name> = HashMap::new();
        let mut names: Vec<Box<str>> = Vec::new();
        let mut id = |s: String| -> Name {
            if let Some(&i) = ids.get(&s) {
                return i;
            }
            let i = push_name(&mut names, &s);
            ids.insert(s, i);
            i
        };
        let mut out = Vec::with_capacity(nets.len());
        for (name, rc) in nets {
            let NetRc { cap_ff, res_ohm, coupling_ff, coupling, coupling_nodes, net_node, ground, res, pins, conns: _ } = rc;
            let name = id(name);
            let coupling = coupling.into_iter().map(|(a, c)| (id(a), c)).collect();
            let cc_own = coupling_nodes.into_iter().map(|(_, own, _, c)| (id(own), c)).collect();
            let net_node = id(net_node);
            let ground = ground.into_iter().map(|(n, c)| (id(n), c)).collect();
            let res = res.into_iter().map(|(a, b, r)| (id(a), id(b), r)).collect();
            let pins = pins.into_iter().map(|(i, p, n)| (id(i), id(p), id(n))).collect();
            out.push(CompactNet { name, cap_ff, res_ohm, coupling_ff, coupling, cc_own, net_node, ground, res, pins });
        }
        CompactSpef { names, nets: out, skipped, unresolved, triplets, reduced, pnets }
    }

    /// [`CompactSpef::from_spef`] without consuming `spef`: the same ids, in the same order, for a
    /// caller that keeps the [`Spef`] (to write it back) or only borrows it.
    pub fn from_ref<'a>(spef: &'a Spef) -> CompactSpef {
        let mut ids: HashMap<&'a str, Name> = HashMap::new();
        let mut names: Vec<Box<str>> = Vec::new();
        let mut nets = Vec::with_capacity(spef.nets.len());
        for (name, rc) in &spef.nets {
            let mut id = |s: &'a str| -> Name { *ids.entry(s).or_insert_with(|| push_name(&mut names, s)) };
            let NetRc { cap_ff, res_ohm, coupling_ff, coupling, coupling_nodes, net_node, ground, res, pins, conns: _ } = rc;
            let name = id(name);
            let coupling = coupling.iter().map(|(a, c)| (id(a), *c)).collect();
            let cc_own = coupling_nodes.iter().map(|(_, own, _, c)| (id(own), *c)).collect();
            let net_node = id(net_node);
            let ground = ground.iter().map(|(n, c)| (id(n), *c)).collect();
            let res = res.iter().map(|(a, b, r)| (id(a), id(b), *r)).collect();
            let pins = pins.iter().map(|(i, p, n)| (id(i), id(p), id(n))).collect();
            nets.push(CompactNet { name, cap_ff: *cap_ff, res_ohm: *res_ohm, coupling_ff: *coupling_ff, coupling, cc_own, net_node, ground, res, pins });
        }
        CompactSpef { names, nets, skipped: spef.skipped, unresolved: spef.unresolved, triplets: spef.triplets, reduced: spef.reduced, pnets: spef.pnets }
    }

    /// The id of `name` as a node of `net` (a grounded cap's, a resistor end's, a coupling cap's
    /// or a pin's), if the net has such a node: what [`NetRc`]'s walks match a node NAME against.
    pub fn node(&self, net: &CompactNet, name: &str) -> Option<Name> {
        let is = |id: &Name| self.name(*id) == name;
        net.ground.iter().map(|g| g.0).find(is)
            .or_else(|| net.res.iter().flat_map(|r| [r.0, r.1]).find(is))
            .or_else(|| net.cc_own.iter().map(|c| c.0).find(is))
            .or_else(|| net.pins.iter().map(|p| p.2).find(is))
    }

    /// The name an id stands for.
    pub fn name(&self, id: Name) -> &str {
        &self.names[id as usize]
    }

    /// A net by name (a binary search: nets are in name order).
    pub fn net(&self, name: &str) -> Option<&CompactNet> {
        self.nets.binary_search_by(|n| self.name(n.name).cmp(name)).ok().map(|i| &self.nets[i])
    }

    /// [`NetRc::pin_node`]: the node of an instance pin, if the net's `*CONN` lists it.
    pub fn pin_node(&self, net: &CompactNet, inst: &str, pin: &str) -> Option<Name> {
        net.pins.iter().find(|&&(i, p, _)| self.name(i) == inst && self.name(p) == pin).map(|&(_, _, n)| n)
    }

    /// [`Spef::wire_load_pf`].
    pub fn wire_load_pf(&self, net: &str) -> f64 {
        self.net(net).map(|rc| rc.cap_ff / 1000.0).unwrap_or(0.0)
    }

    /// [`Spef::net_delay_ns`].
    pub fn net_delay_ns(&self, net: &str) -> f64 {
        self.net(net).map(|rc| rc.res_ohm * rc.cap_ff * 1e-6).unwrap_or(0.0)
    }
}

/// Append a name to the table; its id.
fn push_name(names: &mut Vec<Box<str>>, s: &str) -> Name {
    let i = Name::try_from(names.len()).ok().filter(|&i| i != NO_NAME).expect("more than 2^32 - 1 names in one SPEF");
    names.push(s.into());
    i
}

/// [`Spef::load`], then the analysis form. A caller that renames nets to the design
/// ([`Spef::rename_to_design`]) or writes the file back does that on the [`Spef`] and converts
/// with [`CompactSpef::from_spef`] after.
pub fn load_compact(path: &str) -> Result<CompactSpef, SpefError> {
    Ok(CompactSpef::from_spef(Spef::load(path)?))
}

/// [`Spef::parse`], then the analysis form.
pub fn parse_compact(text: &str) -> CompactSpef {
    CompactSpef::from_spef(Spef::parse(text))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    /// A net per shape the walks branch on: a branching tree with coupling caps on internal
    /// nodes, a resistor loop (the Elmore walk drops the closing resistor), a node reached by
    /// two resistors in parallel, a driver with no resistor, and a net with no coupling.
    const SHAPES: &str = r#"*SPEF "IEEE 1481-1999"
*DESIGN "shapes"
*DIVIDER /
*DELIMITER :
*BUS_DELIMITER [ ]
*T_UNIT 1 PS
*C_UNIT 1 FF
*R_UNIT 1 OHM
*L_UNIT 1 HENRY
*NAME_MAP
*1 tree
*2 loop
*3 par
*4 bare
*5 quiet
*10 u0
*11 u1
*12 u2
*13 u3
*14 u4
*D_NET *1 41.5
*CONN
*I *10:Y O
*I *11:A I
*I *12:B I
*I *13:A I
*CAP
1 *10:Y 1.25
2 *1:1 3.5
3 *1:2 2.75
4 *1:3 0.375
5 *11:A 0.9
6 *12:B 1.1
7 *1:2 *2:1 4.125
8 *1:3 *3:2 0.6
9 *13:A 0.7
*RES
1 *10:Y *1:1 12.5
2 *1:1 *1:2 33.3
3 *1:2 *11:A 7.75
4 *1:1 *1:3 51.0
5 *1:3 *12:B 9.0
6 *1:3 *13:A 18.0
*END
*D_NET *2 20.0
*CONN
*I *11:Y O
*I *14:A I
*CAP
1 *11:Y 0.5
2 *2:1 2.0
3 *2:2 1.5
4 *14:A 1.0
5 *2:1 *1:2 4.125
*RES
1 *11:Y *2:1 10.0
2 *2:1 *2:2 20.0
3 *2:2 *14:A 30.0
4 *14:A *2:1 40.0
*END
*D_NET *3 9.0
*CONN
*I *12:Y O
*I *14:B I
*CAP
1 *12:Y 0.25
2 *3:1 1.75
3 *3:2 *1:3 0.6
4 *14:B 0.8
*RES
1 *12:Y *3:1 15.0
2 *12:Y *3:1 45.0
3 *3:1 *14:B 5.0
*END
*D_NET *4 3.0
*CONN
*I *13:Y O
*I *14:C I
*CAP
1 *13:Y 1.5
2 *14:C 1.5
*END
*D_NET *5 2.0
*CONN
*I *14:Y O
*I *10:A I
*CAP
1 *14:Y 0.5
2 *10:A 0.75
*RES
1 *14:Y *10:A 100.0
*END
"#;

    fn files() -> Vec<(&'static str, String)> {
        let top = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/top/top.spef")).expect("examples/top/top.spef");
        vec![("shapes", SHAPES.to_string()), ("examples/top", top)]
    }

    fn bits(v: f64) -> u64 {
        v.to_bits()
    }

    /// 🔑 The analysis form must give the String form's results BIT FOR BIT: a timer switched
    /// onto it may not move one endpoint. Every net, every node as driver, with and without a
    /// crosstalk load (below, at and above the coupling the net carries on its own nodes, the
    /// three cases `node_caps` branches on) and with receiver caps on the pins.
    #[test]
    fn the_compact_walks_give_the_string_walks_bits() {
        for (what, text) in files() {
            let s = Spef::parse(&text);
            let c = parse_compact(&text);
            assert_eq!(s.nets.len(), c.nets.len(), "{what}");
            let mut walked = 0;
            for (name, rc) in &s.nets {
                let cn = c.net(name).unwrap_or_else(|| panic!("{what}: {name} missing"));
                let mut pin_cap: BTreeMap<String, f64> = BTreeMap::new();
                for (k, (_, _, n)) in rc.pins.iter().enumerate() {
                    pin_cap.insert(n.clone(), 0.3 + 0.1 * k as f64);
                }
                let pc = |id: Name| pin_cap.get(c.name(id)).copied();
                let at_nodes: f64 = rc.coupling_nodes.iter().map(|x| x.3).sum();
                let mut nodes: Vec<&str> = rc.ground.iter().map(|(n, _)| n.as_str()).collect();
                nodes.extend(rc.res.iter().flat_map(|(a, b, _)| [a.as_str(), b.as_str()]));
                nodes.sort();
                nodes.dedup();
                for drv in nodes {
                    let d = c.names.iter().position(|n| &**n == drv).unwrap() as Name;
                    for x in [0.0, at_nodes * 0.5, at_nodes, at_nodes + 2.5] {
                        let e1 = rc.elmore(drv, x, &pin_cap);
                        let e2 = cn.elmore(d, x, pc).map(|v| v.into_iter().map(|(k, t)| (c.name(k).to_string(), t)).collect::<BTreeMap<_, _>>());
                        assert_eq!(e1.as_ref().map(|m| m.iter().map(|(k, v)| (k.clone(), bits(*v))).collect::<Vec<_>>()), e2.as_ref().map(|m| m.iter().map(|(k, v)| (k.clone(), bits(*v))).collect::<Vec<_>>()), "{what} {name} elmore from {drv} x={x}");
                        let p1 = rc.pi_reduce(drv, x, &pin_cap).map(|(a, b, c)| (bits(a), bits(b), bits(c)));
                        let p2 = cn.pi_reduce(d, x, pc).map(|(a, b, c)| (bits(a), bits(b), bits(c)));
                        assert_eq!(p1, p2, "{what} {name} pi from {drv} x={x}");
                        let th = crate::liberty::Thresholds::default();
                        let t1 = rc.transient(drv, 0.05, x, th).map(|m| m.into_iter().map(|(k, (a, b))| (k, bits(a), bits(b))).collect::<Vec<_>>());
                        let t2 = cn.transient(d, 0.05, x, th).map(|v| v.into_iter().map(|(k, (a, b))| (c.name(k).to_string(), (bits(a), bits(b)))).collect::<BTreeMap<_, _>>().into_iter().map(|(k, (a, b))| (k, a, b)).collect::<Vec<_>>());
                        assert_eq!(t1, t2, "{what} {name} transient from {drv} x={x}");
                        walked += usize::from(e1.is_some()) + usize::from(p1.is_some()) + usize::from(t1.is_some());
                    }
                }
            }
            // a comparison of two Nones proves nothing: the walks must have run
            assert!(walked > 40, "{what}: only {walked} walks produced a result");
        }
    }

    /// The per-net scalars and lists the timer reads, and the name table, are the String form's.
    #[test]
    fn the_compact_form_keeps_the_string_forms_fields() {
        for (what, text) in files() {
            let s = Spef::parse(&text);
            let c = parse_compact(&text);
            assert_eq!((s.skipped, s.unresolved, s.triplets, s.reduced, s.pnets), (c.skipped, c.unresolved, c.triplets, c.reduced, c.pnets), "{what}");
            // nets in the String form's (name) order, so `net` can binary-search them
            let order: Vec<&str> = c.nets.iter().map(|n| c.name(n.name)).collect();
            assert_eq!(order, s.nets.keys().map(String::as_str).collect::<Vec<_>>(), "{what}");
            for (name, rc) in &s.nets {
                let cn = c.net(name).unwrap();
                let n = |id: Name| c.name(id).to_string();
                assert_eq!((bits(rc.cap_ff), bits(rc.res_ohm), bits(rc.coupling_ff)), (bits(cn.cap_ff), bits(cn.res_ohm), bits(cn.coupling_ff)), "{what} {name}");
                assert_eq!(rc.net_node, n(cn.net_node));
                assert_eq!(rc.coupling, cn.coupling.iter().map(|&(a, v)| (n(a), v)).collect::<Vec<_>>());
                assert_eq!(rc.coupling_nodes.iter().map(|x| (x.1.clone(), x.3)).collect::<Vec<_>>(), cn.cc_own.iter().map(|&(a, v)| (n(a), v)).collect::<Vec<_>>());
                assert_eq!(rc.ground, cn.ground.iter().map(|&(a, v)| (n(a), v)).collect::<Vec<_>>());
                assert_eq!(rc.res, cn.res.iter().map(|&(a, b, v)| (n(a), n(b), v)).collect::<Vec<_>>());
                assert_eq!(rc.pins, cn.pins.iter().map(|&(a, b, d)| (n(a), n(b), n(d))).collect::<Vec<_>>());
                for (i, p, node) in &rc.pins {
                    assert_eq!(c.pin_node(cn, i, p).map(n).as_deref(), rc.pin_node(i, p).or(Some(node.as_str())));
                }
                assert_eq!(bits(c.wire_load_pf(name)), bits(s.wire_load_pf(name)));
                assert_eq!(bits(c.net_delay_ns(name)), bits(s.net_delay_ns(name)));
            }
            assert!(c.net("no such net").is_none());
            // the borrowing conversion gives the same table and the same nets
            assert!(CompactSpef::from_ref(&s) == c, "{what}: from_ref differs from from_spef");
            // `node` finds every node of a net by name, and nothing that is not one
            for (name, rc) in &s.nets {
                let cn = c.net(name).unwrap();
                for nm in rc.ground.iter().map(|g| &g.0).chain(rc.res.iter().flat_map(|r| [&r.0, &r.1])).chain(rc.coupling_nodes.iter().map(|x| &x.1)).chain(rc.pins.iter().map(|p| &p.2)) {
                    assert_eq!(c.node(cn, nm).map(|i| c.name(i)), Some(nm.as_str()), "{what} {name} {nm}");
                }
                assert_eq!(c.node(cn, "no:such"), None);
            }
            // each name once
            let mut seen = std::collections::HashSet::new();
            assert!(c.names.iter().all(|n| seen.insert(n.clone())), "{what}: a name interned twice");
        }
    }
}
