//! One flat netlist from a hierarchical one.
//!
//! A hierarchical netlist (`-hierarchy all` style: every module of the design in one file, the
//! top instantiating blocks that instantiate cells) is read whole by [`super::load_compact`],
//! and its child pins are bound to the child's ports by `bind_defined_child_pins`. Nothing
//! below the top is a cell of the design until it is flattened: [`CompactDesign::flatten`]
//! expands every instance of a module DEFINED in the file into its contents, merges each child
//! port's net with the net the parent connects to it, and returns ONE module.
//!
//! 🔑 **The rules follow the reference's linker and its database import**, not a reading of the
//! LRM:
//! - an instance's flat name is its path from the top, joined with `/` — the reference's
//!   `Network::pathName`, its divider `/`;
//! - a `/` (or a `\`) INSIDE one name of the path is escaped with `\`, as the reference's
//!   `verilogToSta` escapes a divider in an escaped identifier, so `u1` / `\a/b ` is `u1/a\/b`
//!   and splitting at an unescaped `/` gives the path back. Brackets are not escaped: loom keeps
//!   names canonical and records which arrived escaped (see [`Netlist::escaped_names`]);
//! - a merged net is named by its HIGHEST occurrence — the database import (`makeDbNets`) walks
//!   the top first and skips a child net that reaches a port of its instance, so the parent's
//!   name is the one kept. Within one module, a port bit's name wins over a wire's, as the
//!   per-module reader already canonicalises it;
//! - a child port the parent leaves UNCONNECTED keeps its own net, named under the child's path:
//!   what the child connects to it is still there, undriven, not dropped;
//! - `inout` stays apart from `input` and `output` at the top, as it does in every module.
//!
//! Top-level names are kept exactly as the unflattened top has them, so a flat netlist of a
//! design whose top has no module instances is the same netlist as its top — except that a pin
//! on a net an `assign` holds constant (`assign n = 1'b0;`) is recorded as a tie, as a constant
//! written on the pin is, rather than as a connection to a net named after the constant.

use super::{CellPort, CompactDesign, CompactInst, CompactModule, Design, Netlist, StrPool, Sym};
use std::collections::{BTreeMap, HashMap, HashSet};

/// What to flatten and against what.
#[derive(Debug, Clone, Default)]
pub struct FlattenOptions {
    /// Modules kept as LEAF instances, not expanded: an assembly top keeps its block macros as
    /// blocks. A name the file does not define is reported in [`FlattenReport::keep_not_defined`].
    pub keep: Vec<String>,
    /// The cells a library defines (Liberty cell names). An instantiated cell that is neither a
    /// module of the file nor named here is reported in [`FlattenReport::undefined_cells`].
    /// `None`: no library to check against, and EVERY cell that is not a module of the file is
    /// reported there, so that a misspelt module never passes as a leaf unseen.
    pub library_cells: Option<HashSet<String>>,
}

/// What flattening found. Nothing here stops the flat netlist being made; each list is
/// something a consumer must know before trusting it — see [`FlattenReport::health`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
#[must_use]
pub struct FlattenReport {
    /// The top module flattened.
    pub top: String,
    /// Instances of modules defined in the file that were expanded.
    pub expanded: usize,
    /// Leaf instances in the flat netlist (cells, black boxes and kept blocks).
    pub leaves: usize,
    /// Kept blocks, `(module, instances)`, by name.
    pub kept_blocks: Vec<(String, usize)>,
    /// Names in [`FlattenOptions::keep`] that the file does not define.
    pub keep_not_defined: Vec<String>,
    /// Instantiated cells that are neither defined in the file nor library cells, `(cell,
    /// instances)`, by name. They are leaves in the flat netlist, with the pins they were given.
    pub undefined_cells: Vec<(String, usize)>,
    /// A library was given to check cells against ([`FlattenOptions::library_cells`]).
    pub library_checked: bool,
    /// Child port bits their parent left unconnected; each keeps its own (undriven) net.
    pub unconnected_child_ports: usize,
    /// Modules that instantiate themselves, directly or through others; such an instance is
    /// kept as a leaf rather than expanded without end.
    pub recursive: Vec<String>,
    /// Top port bits merged with another top port (`assign o1 = o2;`, or a feedthrough block
    /// between two ports): each stays a port and is recorded in [`Netlist::aliases`].
    pub port_aliases: usize,
    /// Nets held to 0 and to 1 at once (a supply or tie of each value merged into one net).
    pub constant_conflicts: usize,
}

impl FlattenReport {
    /// One line on anything a consumer should know, or `None` when the flattening is clean.
    pub fn health(&self) -> Option<String> {
        let mut notes = Vec::new();
        let list = |v: &[(String, usize)]| {
            let mut s: Vec<String> = v.iter().take(8).map(|(n, k)| format!("{n} ({k})")).collect();
            if v.len() > 8 {
                s.push(format!("and {} more", v.len() - 8));
            }
            s.join(", ")
        };
        if !self.undefined_cells.is_empty() {
            let why = if self.library_checked {
                "neither defined in the netlist nor in the library"
            } else {
                "not defined in the netlist, and no library given to check them against"
            };
            notes.push(format!(
                "{} cell type(s) {why}, taken as leaves: {}",
                self.undefined_cells.len(),
                list(&self.undefined_cells)
            ));
        }
        if !self.keep_not_defined.is_empty() {
            notes.push(format!("keep-as-block name(s) not defined in the netlist: {}", self.keep_not_defined.join(", ")));
        }
        if !self.recursive.is_empty() {
            notes.push(format!("recursive module(s) kept as leaves: {}", self.recursive.join(", ")));
        }
        if self.unconnected_child_ports > 0 {
            notes.push(format!(
                "{} child port bit(s) left unconnected by the parent, each kept as its own undriven net",
                self.unconnected_child_ports
            ));
        }
        if self.constant_conflicts > 0 {
            notes.push(format!("{} net(s) tied to both 0 and 1", self.constant_conflicts));
        }
        (!notes.is_empty()).then(|| notes.join("; "))
    }
}

/// One path component, escaped so that the path can be split at its unescaped `/` again — the
/// reference's `verilogToSta` escapes a divider and the escape character itself.
fn escape_component(c: &str, out: &mut String) {
    for ch in c.chars() {
        if ch == '/' || ch == '\\' {
            out.push('\\');
        }
        out.push(ch);
    }
}

/// The nets of the flat netlist, as a union-find over every (scope, local name) the expansion
/// touches. The root of a set is the node whose name the flat net takes: the lowest `key`.
#[derive(Default)]
struct Nets {
    parent: Vec<u32>,
    /// `depth << 40 | order`, with `1 << 39` added for a constant literal: the highest module
    /// first, then the order nodes were made in (port bits, then connections, then `assign`s)
    key: Vec<u64>,
    /// `(scope, local name)`
    name: Vec<(u32, Sym)>,
    /// 0 none, 1 held 0, 2 held 1 — by a constant on a pin or an `assign` (pins become ties)
    literal: Vec<u8>,
    /// 0 none, 1 held 0, 2 held 1 — by a `supply0`/`supply1` (pins stay connected)
    supply: Vec<u8>,
    order: u64,
    conflicts: usize,
}

impl Nets {
    fn make(&mut self, scope: u32, s: Sym, depth: u32, literal: u8) -> u32 {
        let id = u32::try_from(self.parent.len()).expect("fewer than 4 billion nets in one netlist");
        self.parent.push(id);
        let lit = if literal != 0 { 1u64 << 39 } else { 0 };
        self.key.push(((depth as u64) << 40) | lit | (self.order & ((1 << 39) - 1)));
        self.order += 1;
        self.name.push((scope, s));
        self.literal.push(literal);
        self.supply.push(0);
        id
    }

    fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            let p = self.parent[x as usize];
            self.parent[x as usize] = self.parent[p as usize];
            x = p;
        }
        x
    }

    fn merge_flag(a: u8, b: u8, conflicts: &mut usize) -> u8 {
        match (a, b) {
            (0, v) | (v, 0) => v,
            (x, y) if x == y => x,
            (x, _) => {
                *conflicts += 1;
                x
            }
        }
    }

    fn union(&mut self, a: u32, b: u32) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        let (keep, gone) = if self.key[ra as usize] <= self.key[rb as usize] { (ra, rb) } else { (rb, ra) };
        self.parent[gone as usize] = keep;
        let (k, g) = (keep as usize, gone as usize);
        self.literal[k] = Self::merge_flag(self.literal[k], self.literal[g], &mut self.conflicts);
        self.supply[k] = Self::merge_flag(self.supply[k], self.supply[g], &mut self.conflicts);
    }
}

/// One module instance being expanded: its local names' nets, and the next instance to visit.
struct Frame {
    module: usize,
    scope: u32,
    depth: u32,
    next: usize,
    local: HashMap<Sym, u32>,
}

/// A leaf of the flat netlist while the expansion runs: its connections point at nets, named
/// once every merge is known.
struct Leaf {
    cell: Sym,
    scope: u32,
    name: Sym,
    conns: (u32, u32),
    ties: (u32, u32),
}

struct Flattener<'d> {
    d: &'d CompactDesign,
    nets: Nets,
    /// the escaped path of every scope; scope 0 is the top, with an empty path
    scopes: Vec<String>,
    lit0: Option<Sym>,
    lit1: Option<Sym>,
}

impl Flattener<'_> {
    fn literal_of(&self, s: Sym) -> u8 {
        if Some(s) == self.lit0 {
            1
        } else if Some(s) == self.lit1 {
            2
        } else {
            0
        }
    }

    fn node(&mut self, local: &mut HashMap<Sym, u32>, scope: u32, depth: u32, s: Sym) -> u32 {
        if let Some(&n) = local.get(&s) {
            return n;
        }
        let lit = self.literal_of(s);
        let n = self.nets.make(scope, s, depth, lit);
        local.insert(s, n);
        n
    }

    /// A module's own nets, in the order their names are preferred: port bits, then every net a
    /// connection names, then `assign`s and supplies.
    fn open(&mut self, module: usize, scope: u32, depth: u32, local: &mut HashMap<Sym, u32>) {
        let m = &self.d.modules[module];
        for &p in m.inputs.iter().chain(&m.outputs).chain(&m.inouts).chain(&m.undirected) {
            self.node(local, scope, depth, p);
        }
        for &(_, net) in &m.conn_table {
            self.node(local, scope, depth, net);
        }
        for &(a, b) in &m.aliases {
            let (na, nb) = (self.node(local, scope, depth, a), self.node(local, scope, depth, b));
            self.nets.union(na, nb);
        }
        for &(s, v) in &m.constants {
            let n = self.node(local, scope, depth, s);
            let r = self.nets.find(n) as usize;
            let f = if v { 2 } else { 1 };
            self.nets.supply[r] = Nets::merge_flag(self.nets.supply[r], f, &mut self.nets.conflicts);
        }
    }

    /// The flat name of `s` in `scope`.
    fn flat_name(&self, scope: u32, s: Sym, out: &mut String) {
        out.clear();
        let text = self.d.pool.get(s);
        if scope == 0 {
            out.push_str(text);
            return;
        }
        out.push_str(&self.scopes[scope as usize]);
        out.push('/');
        escape_component(text, out);
    }
}

impl CompactDesign {
    /// The design as ONE module: every instance of a module defined in the file expanded into
    /// its contents, down to the leaves — see the [module docs](self) for the naming rules.
    ///
    /// The result holds the flat top only; the hierarchy it was made from is unchanged.
    pub fn flatten(&self, opts: &FlattenOptions) -> (CompactDesign, FlattenReport) {
        let d = self;
        let top = d.top;
        let mut report = FlattenReport {
            top: d.pool.get(d.modules[top].name).to_string(),
            library_checked: opts.library_cells.is_some(),
            ..Default::default()
        };
        let keep: HashSet<Sym> = opts.keep.iter().filter_map(|k| d.pool.find(k)).collect();
        report.keep_not_defined = opts.keep.iter().filter(|k| d.module(k).is_none()).cloned().collect();
        let mut fl = Flattener {
            d,
            nets: Nets::default(),
            scopes: vec![String::new()],
            lit0: d.pool.find("1'b0"),
            lit1: d.pool.find("1'b1"),
        };
        let mut leaves: Vec<Leaf> = Vec::new();
        let mut leaf_conns: Vec<(Sym, u32)> = Vec::new();
        let mut leaf_ties: Vec<(Sym, bool)> = Vec::new();
        let mut kept: BTreeMap<String, usize> = BTreeMap::new();
        let mut undefined: BTreeMap<String, usize> = BTreeMap::new();
        let mut recursive: HashSet<usize> = HashSet::new();
        let (mut skipped_assigns, mut unbound_pins, mut behavioural) = (0usize, 0usize, 0usize);
        let mut on_stack = vec![false; d.modules.len()];

        let mut local = HashMap::new();
        fl.open(top, 0, 0, &mut local);
        let top_nodes: Vec<(Sym, u32)> = {
            let tm = &d.modules[top];
            let ports = tm.inputs.iter().chain(&tm.outputs).chain(&tm.inouts).chain(&tm.undirected);
            ports.filter_map(|p| local.get(p).map(|&n| (*p, n))).collect()
        };
        let mut stack = vec![Frame { module: top, scope: 0, depth: 0, next: 0, local }];
        on_stack[top] = true;
        let tm = &d.modules[top];
        (skipped_assigns, unbound_pins, behavioural) =
            (skipped_assigns + tm.skipped_assigns, unbound_pins + tm.unbound_pins, behavioural + tm.behavioural);

        while let Some(f) = stack.last_mut() {
            let m = &d.modules[f.module];
            let Some(x) = m.insts.get(f.next).copied() else {
                on_stack[f.module] = false;
                stack.pop();
                continue;
            };
            f.next += 1;
            let (scope, depth) = (f.scope, f.depth);
            let child = d.by_name.get(&x.cell).copied();
            let expand = match child {
                Some(c) if keep.contains(&x.cell) => {
                    *kept.entry(d.pool.get(d.modules[c].name).to_string()).or_default() += 1;
                    false
                }
                Some(c) if on_stack[c] => {
                    recursive.insert(c);
                    false
                }
                Some(_) => true,
                None => {
                    let known = opts.library_cells.as_ref().is_some_and(|l| l.contains(d.pool.get(x.cell)));
                    if !known {
                        *undefined.entry(d.pool.get(x.cell).to_string()).or_default() += 1;
                    }
                    false
                }
            };
            if !expand {
                let c0 = leaf_conns.len() as u32;
                for &(pin, net) in m.conns(&x) {
                    let n = fl.node(&mut f.local, scope, depth, net);
                    leaf_conns.push((pin, n));
                }
                let t0 = leaf_ties.len() as u32;
                leaf_ties.extend_from_slice(m.ties(&x));
                leaves.push(Leaf {
                    cell: x.cell,
                    scope,
                    name: x.name,
                    conns: (c0, leaf_conns.len() as u32),
                    ties: (t0, leaf_ties.len() as u32),
                });
                continue;
            }
            let c = child.expect("expand is only set for a module defined in the file");
            report.expanded += 1;
            // the child's scope: its path, each component escaped
            let mut path = String::new();
            if scope != 0 {
                path.push_str(&fl.scopes[scope as usize]);
                path.push('/');
            }
            escape_component(d.pool.get(x.name), &mut path);
            let cscope = u32::try_from(fl.scopes.len()).expect("fewer than 4 billion module instances");
            fl.scopes.push(path);
            // the child's port bits join the nets the parent connects to them
            let mut clocal: HashMap<Sym, u32> = HashMap::new();
            let mut bound: HashSet<Sym> = HashSet::new();
            for &(pin, net) in m.conns(&x) {
                let pn = fl.node(&mut f.local, scope, depth, net);
                let cn = fl.node(&mut clocal, cscope, depth + 1, pin);
                fl.nets.union(pn, cn);
                bound.insert(pin);
            }
            for &(pin, v) in m.ties(&x) {
                let cn = fl.node(&mut clocal, cscope, depth + 1, pin);
                let r = fl.nets.find(cn) as usize;
                let lit = if v { 2 } else { 1 };
                fl.nets.literal[r] = Nets::merge_flag(fl.nets.literal[r], lit, &mut fl.nets.conflicts);
                bound.insert(pin);
            }
            let cm = &d.modules[c];
            report.unconnected_child_ports += cm
                .inputs
                .iter()
                .chain(&cm.outputs)
                .chain(&cm.inouts)
                .chain(&cm.undirected)
                .filter(|p| !bound.contains(p))
                .count();
            (skipped_assigns, unbound_pins, behavioural) =
                (skipped_assigns + cm.skipped_assigns, unbound_pins + cm.unbound_pins, behavioural + cm.behavioural);
            fl.open(c, cscope, depth + 1, &mut clocal);
            on_stack[c] = true;
            stack.push(Frame { module: c, scope: cscope, depth: depth + 1, next: 0, local: clocal });
        }

        // ---- every merge is known: name the nets, and write the flat module -----------------
        let mut pool: StrPool = d.pool.clone();
        let mut escaped = d.escaped.clone();
        let mark = |escaped: &mut Vec<u64>, s: Sym| {
            let (w, b) = (s.index() / 64, s.index() % 64);
            if escaped.len() <= w {
                escaped.resize(w + 1, 0);
            }
            escaped[w] |= 1 << b;
        };
        let mut buf = String::new();
        let mut net_sym: Vec<Option<Sym>> = vec![None; fl.nets.parent.len()];
        let mut name_of = |fl: &mut Flattener, pool: &mut StrPool, escaped: &mut Vec<u64>, n: u32| -> Sym {
            let r = fl.nets.find(n);
            if let Some(s) = net_sym[r as usize] {
                return s;
            }
            let (scope, local) = fl.nets.name[r as usize];
            let s = if scope == 0 {
                local
            } else {
                fl.flat_name(scope, local, &mut buf);
                let s = pool.intern(&buf);
                if d.is_escaped(local) {
                    mark(escaped, s);
                }
                s
            };
            net_sym[r as usize] = Some(s);
            s
        };

        let tm = &d.modules[top];
        let mut flat = CompactModule {
            name: tm.name,
            inputs: tm.inputs.clone(),
            outputs: tm.outputs.clone(),
            inouts: tm.inouts.clone(),
            undirected: tm.undirected.clone(),
            canonical: tm.canonical.clone(),
            cell_ports: tm.cell_ports.clone(),
            skipped_assigns,
            unbound_pins,
            behavioural,
            ..Default::default()
        };
        let none = pool.intern("");
        let mut iname = String::new();
        for l in &leaves {
            let name = if l.scope == 0 {
                l.name
            } else {
                fl.flat_name(l.scope, l.name, &mut iname);
                let s = pool.intern(&iname);
                if d.is_escaped(l.name) {
                    mark(&mut escaped, s);
                }
                s
            };
            let (c0, t0) = (flat.conn_table.len() as u32, flat.tie_table.len() as u32);
            flat.tie_table.extend_from_slice(&leaf_ties[l.ties.0 as usize..l.ties.1 as usize]);
            for &(pin, n) in &leaf_conns[l.conns.0 as usize..l.conns.1 as usize] {
                let r = fl.nets.find(n) as usize;
                // a pin on a net a constant holds is tied, as the pin would be in the parent
                match fl.nets.literal[r] {
                    0 => {
                        let s = name_of(&mut fl, &mut pool, &mut escaped, n);
                        flat.conn_table.push((pin, s));
                    }
                    v => flat.tie_table.push((pin, v == 2)),
                }
            }
            flat.insts.push(CompactInst {
                cell: l.cell,
                name,
                conns: (c0, flat.conn_table.len() as u32),
                ties: (t0, flat.tie_table.len() as u32),
            });
        }
        flat.positional = flat.conn_table.iter().filter(|(p, _)| *p == none).count();
        // top ports: a port merged with an earlier port keeps its name and is recorded as an alias
        let mut port_root: HashMap<u32, Sym> = HashMap::new();
        for &(p, n) in &top_nodes {
            let r = fl.nets.find(n);
            match port_root.get(&r) {
                Some(&first) if first != p => {
                    flat.aliases.push((p, first));
                    report.port_aliases += 1;
                }
                Some(_) => {}
                None => {
                    port_root.insert(r, p);
                }
            }
            let r = r as usize;
            if fl.nets.literal[r] != 0 && port_root.get(&(r as u32)) == Some(&p) {
                flat.constants.push((p, fl.nets.literal[r] == 2));
            }
        }
        // supplies: the net keeps its pins, and says its value
        let mut seen_supply: HashSet<u32> = HashSet::new();
        for n in 0..fl.nets.parent.len() as u32 {
            let r = fl.nets.find(n);
            if fl.nets.supply[r as usize] != 0 && fl.nets.literal[r as usize] == 0 && seen_supply.insert(r) {
                let s = name_of(&mut fl, &mut pool, &mut escaped, r);
                flat.constants.push((s, fl.nets.supply[r as usize] == 2));
            }
        }
        report.constant_conflicts = fl.nets.conflicts;
        report.leaves = flat.insts.len();
        report.kept_blocks = kept.into_iter().collect();
        report.undefined_cells = undefined.into_iter().collect();
        let mut rec: Vec<String> = recursive.into_iter().map(|c| d.pool.get(d.modules[c].name).to_string()).collect();
        rec.sort();
        report.recursive = rec;

        let mut by_name = HashMap::new();
        by_name.insert(flat.name, 0);
        (CompactDesign { pool, modules: vec![flat], top: 0, escaped, by_name }, report)
    }

    /// The TOP as an owned [`Netlist`], with the file-wide lists (`modules`, `escaped_names`)
    /// on it — what [`super::load`] returns, made without converting any other module.
    pub fn top_netlist(&self) -> Netlist {
        let mut nl = self.top().to_netlist(&self.pool);
        nl.modules = self.modules.iter().map(|m| self.pool.get(m.name).to_string()).collect();
        nl.escaped_names = self.escaped_names().map(|s| self.pool.get(s).to_string()).collect();
        nl
    }

    /// The compact form of an owned [`Design`] — every name interned once.
    pub fn from_design(d: &Design) -> CompactDesign {
        let mut pool = StrPool::default();
        let mut modules = Vec::with_capacity(d.modules.len());
        for m in &d.modules {
            let mut i = |s: &str| pool.intern(s);
            let mut cm = CompactModule {
                name: i(&m.module),
                inputs: m.inputs.iter().map(|s| i(s)).collect(),
                outputs: m.outputs.iter().map(|s| i(s)).collect(),
                inouts: m.inouts.iter().map(|s| i(s)).collect(),
                undirected: m.undirected.iter().map(|s| i(s)).collect(),
                aliases: m.aliases.iter().map(|(a, b)| (i(a), i(b))).collect(),
                constants: m.constants.iter().map(|(s, v)| (i(s), *v)).collect(),
                canonical: m.canonical.iter().map(|(a, b)| (i(a), i(b))).collect(),
                skipped_assigns: m.skipped_assigns,
                unbound_pins: m.unbound_pins,
                behavioural: m.behavioural,
                positional: m.positional,
                ..Default::default()
            };
            for x in &m.insts {
                let (c0, t0) = (cm.conn_table.len() as u32, cm.tie_table.len() as u32);
                for (p, n) in &x.conns {
                    cm.conn_table.push((pool.intern(p), pool.intern(n)));
                }
                for (p, v) in &x.ties {
                    cm.tie_table.push((pool.intern(p), *v));
                }
                let (cell, name) = (pool.intern(&x.cell), pool.intern(&x.name));
                cm.insts.push(CompactInst {
                    cell,
                    name,
                    conns: (c0, cm.conn_table.len() as u32),
                    ties: (t0, cm.tie_table.len() as u32),
                });
            }
            // as a parent sees it: one port per bit (the owned form keeps no bundles)
            cm.cell_ports = cm
                .inputs
                .iter()
                .chain(&cm.outputs)
                .chain(&cm.inouts)
                .chain(&cm.undirected)
                .map(|&b| CellPort { name: b, bits: vec![b], bundle: false })
                .collect();
            modules.push(cm);
        }
        let mut escaped: Vec<u64> = Vec::new();
        if let Some(t) = d.modules.get(d.top) {
            for e in &t.escaped_names {
                let s = pool.intern(e);
                let (w, b) = (s.index() / 64, s.index() % 64);
                if escaped.len() <= w {
                    escaped.resize(w + 1, 0);
                }
                escaped[w] |= 1 << b;
            }
        }
        let mut by_name = HashMap::with_capacity(modules.len());
        for (k, m) in modules.iter().enumerate() {
            by_name.insert(m.name, k);
        }
        CompactDesign { pool, modules, top: d.top, escaped, by_name }
    }
}

impl Design {
    /// [`CompactDesign::flatten`] on the owned form: the design as ONE module (the result's
    /// only module, and its top).
    pub fn flatten(&self, opts: &FlattenOptions) -> (Design, FlattenReport) {
        let (flat, report) = CompactDesign::from_design(self).flatten(opts);
        (flat.to_design(), report)
    }
}

#[cfg(test)]
mod tests {
    use super::super::{parse, parse_all, parse_compact, parse_flat, Inst};
    use super::*;

    fn conns(x: &Inst) -> Vec<(&str, &str)> {
        x.conns.iter().map(|(p, n)| (p.as_str(), n.as_str())).collect()
    }

    fn inst<'a>(nl: &'a Netlist, name: &str) -> &'a Inst {
        nl.insts.iter().find(|x| x.name == name).unwrap_or_else(|| panic!("no instance {name}: {:?}", nl.insts.iter().map(|x| &x.name).collect::<Vec<_>>()))
    }

    const THREE_LEVELS: &str = "
module leaf (a, y); input a; output y; wire n;
  INV u0 (.A(a), .Y(n));
  BUF u1 (.A(n), .X(y));
endmodule
module mid (i, o); input i; output o; wire w;
  leaf l0 (.a(i), .y(w));
  leaf l1 (.a(w), .y(o));
endmodule
module chip (a, y); input a; output y; wire m;
  mid m0 (.i(a), .o(m));
  BUF ob (.A(m), .X(y));
endmodule
";

    /// Rule (the reference's `Network::pathName`, divider `/`): a leaf's flat name is its path
    /// from the top; and (`makeDbNets`) a net that crosses a port is named by its HIGHEST
    /// occurrence — `a` at the top, not `m0/i` or `m0/l0/a`.
    #[test]
    fn a_hierarchy_flattens_to_its_leaves_with_path_names() {
        let (nl, rep) = parse_flat(THREE_LEVELS, &FlattenOptions::default()).unwrap();
        assert_eq!(nl.module, "chip");
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["m0/l0/u0", "m0/l0/u1", "m0/l1/u0", "m0/l1/u1", "ob"], "depth first, statement order");
        assert_eq!(conns(inst(&nl, "m0/l0/u0")), [("A", "a"), ("Y", "m0/l0/n")]);
        assert_eq!(conns(inst(&nl, "m0/l0/u1")), [("A", "m0/l0/n"), ("X", "m0/w")], "the child's `y` is mid's `w`");
        assert_eq!(conns(inst(&nl, "m0/l1/u0")), [("A", "m0/w"), ("Y", "m0/l1/n")]);
        assert_eq!(conns(inst(&nl, "m0/l1/u1")), [("A", "m0/l1/n"), ("X", "m")], "mid's `o` is the top's `m`");
        assert_eq!(conns(inst(&nl, "ob")), [("A", "m"), ("X", "y")]);
        assert_eq!((nl.inputs.clone(), nl.outputs.clone()), (vec!["a".to_string()], vec!["y".to_string()]));
        assert_eq!((rep.expanded, rep.leaves), (3, 5));
        // the top alone has two instances and none of the cells below them — and says so
        let top = parse(THREE_LEVELS).unwrap();
        assert_eq!(top.insts.len(), 2);
        assert!(top.health().unwrap().contains("flatten"), "{:?}", top.health());
        assert!(nl.health().is_none() || !nl.health().unwrap().contains("flatten"));
    }

    /// Rule: a child port its parent leaves unconnected keeps its own net — named under the
    /// child — with whatever the child hangs on it, undriven; it is not dropped.
    #[test]
    fn an_unconnected_child_port_is_kept_as_an_undriven_net() {
        let src = "module sub (a, b, y); input a, b; output y; AND2 g (.A(a), .B(b), .X(y)); endmodule\n\
                   module top (x, y); input x; output y; sub s (.a(x), .y(y)); endmodule\n";
        let (nl, rep) = parse_flat(src, &FlattenOptions::default()).unwrap();
        assert_eq!(conns(inst(&nl, "s/g")), [("A", "x"), ("B", "s/b"), ("X", "y")]);
        assert_eq!(rep.unconnected_child_ports, 1);
        assert!(rep.health().unwrap().contains("unconnected"));
    }

    /// Rule: every instantiated cell that is neither a module of the file nor a library cell is
    /// REPORTED — with no library, every non-module cell is, so a misspelt module never passes as
    /// a leaf unseen.
    #[test]
    fn a_cell_neither_defined_nor_in_the_library_is_reported() {
        let src = "module sub (a, y); input a; output y; INV u (.A(a), .Y(y)); endmodule\n\
                   module top (a, y); input a; output y; wire n; sub s (.a(a), .y(n)); subb t (.a(n), .y(y)); endmodule\n";
        let (_, rep) = parse_flat(src, &FlattenOptions::default()).unwrap();
        assert!(!rep.library_checked);
        assert_eq!(rep.undefined_cells, [("INV".to_string(), 1), ("subb".to_string(), 1)]);
        let lib: HashSet<String> = ["INV".to_string()].into();
        let (_, rep) = parse_flat(src, &FlattenOptions { library_cells: Some(lib), ..Default::default() }).unwrap();
        assert!(rep.library_checked);
        assert_eq!(rep.undefined_cells, [("subb".to_string(), 1)], "the misspelt module, and only it");
        assert!(rep.health().unwrap().contains("subb (1)"));
    }

    /// Rule: a module named in `keep` stays a leaf instance with its port-bit pins; the others
    /// flatten. A keep name the file does not define is reported.
    #[test]
    fn a_kept_block_stays_a_leaf() {
        let opts = FlattenOptions { keep: vec!["leaf".into(), "nosuch".into()], ..Default::default() };
        let (nl, rep) = parse_flat(THREE_LEVELS, &opts).unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["m0/l0", "m0/l1", "ob"]);
        assert_eq!(inst(&nl, "m0/l0").cell, "leaf");
        assert_eq!(conns(inst(&nl, "m0/l1")), [("a", "m0/w"), ("y", "m")]);
        assert_eq!(rep.kept_blocks, [("leaf".to_string(), 2)]);
        assert_eq!(rep.keep_not_defined, ["nosuch"]);
    }

    /// Rule (the reference's `verilogToSta`): a `/` inside one name of the path is escaped with
    /// `\`, so the path splits back at its unescaped dividers. A top-level name is kept as the
    /// unflattened top has it.
    #[test]
    fn a_divider_inside_a_name_is_escaped_in_the_path() {
        let src = "module sub (a, y); input a; output y; INV \\u/0 (.A(a), .Y(y)); endmodule\n\
                   module top (a, y); input a; output y; wire n; sub \\x/y (.a(a), .y(n)); BUF \\b/1 (.A(n), .X(y)); endmodule\n";
        let (nl, _) = parse_flat(src, &FlattenOptions::default()).unwrap();
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["x\\/y/u\\/0", "b/1"]);
        assert!(nl.escaped_names.contains("x\\/y/u\\/0"), "the leaf's own name arrived escaped");
    }

    /// Rule (`mergeAssignNet`, then the port boundary): an `assign` in a child joins its two
    /// names, so the parent's net reaches the cell behind it; a feedthrough between two TOP
    /// ports keeps both ports, recorded as an alias.
    #[test]
    fn assigns_in_a_child_are_resolved_across_the_boundary() {
        let src = "module sub (i, o, p); input i; output o, p; wire n; assign o = n; INV u (.A(i), .Y(n)); assign p = i; endmodule\n\
                   module top (a, y, z); input a; output y, z; sub s (.i(a), .o(y), .p(z)); BUF b (.A(z), .X()); endmodule\n";
        let (nl, rep) = parse_flat(src, &FlattenOptions::default()).unwrap();
        assert_eq!(conns(inst(&nl, "s/u")), [("A", "a"), ("Y", "y")]);
        assert_eq!(conns(inst(&nl, "b")), [("A", "a")], "z is a, through the feedthrough");
        assert_eq!(nl.aliases, [("z".to_string(), "a".to_string())]);
        assert_eq!(rep.port_aliases, 1);
    }

    /// Rule: a constant on a module instance's pin holds the child's port net; the cells on it
    /// are tied, as they would be with the constant written on their own pins.
    #[test]
    fn a_constant_on_a_module_pin_ties_the_cells_inside() {
        let src = "module sub (e, a, y); input e, a; output y; AND2 g (.A(a), .B(e), .X(y)); endmodule\n\
                   module top (a, y); input a; output y; sub s (.e(1'b1), .a(a), .y(y)); endmodule\n";
        let (nl, _) = parse_flat(src, &FlattenOptions::default()).unwrap();
        let g = inst(&nl, "s/g");
        assert_eq!(conns(g), [("A", "a"), ("X", "y")]);
        assert_eq!(g.ties, [("B".to_string(), true)]);
    }

    /// Rule: `inout` stays apart from `input` and `output` at the top, and an inout child port
    /// joins the parent's net like any other.
    #[test]
    fn inout_pads_stay_inouts() {
        let src = "module pad (p, d); inout p; input d; PADCELL c (.PAD(p), .D(d)); endmodule\n\
                   module top (io, d); inout io; input d; pad u (.p(io), .d(d)); endmodule\n";
        let (nl, _) = parse_flat(src, &FlattenOptions::default()).unwrap();
        assert_eq!(nl.inouts, ["io"]);
        assert_eq!(nl.inputs, ["d"]);
        assert!(nl.outputs.is_empty());
        assert_eq!(conns(inst(&nl, "u/c")), [("PAD", "io"), ("D", "d")]);
    }

    /// A design with no module instances flattens to its own top, unchanged.
    #[test]
    fn a_flat_design_flattens_to_itself() {
        let src = "module top (a, b, y); input a, b; output [1:0] y; wire n; supply0 g;\n\
                   NAND2 u0 (.A(a), .B(b), .Y(n)); INV u1 (.A(n), .Y(y[0])); BUF u2 (.A({n}), .X(y[1]));\n\
                   TIE t (.LO(g)); endmodule\n";
        let top = parse(src).unwrap();
        let (flat, rep) = parse_flat(src, &FlattenOptions::default()).unwrap();
        let key = |nl: &Netlist| nl.insts.iter().map(|x| (x.cell.clone(), x.name.clone(), x.conns.clone(), x.ties.clone())).collect::<Vec<_>>();
        assert_eq!(key(&flat), key(&top));
        assert_eq!((&flat.inputs, &flat.outputs, &flat.inouts), (&top.inputs, &top.outputs, &top.inouts));
        assert_eq!(flat.constants, top.constants);
        assert_eq!(rep.expanded, 0);
    }

    /// A module that instantiates itself is kept as a leaf at the repeat, not expanded forever.
    #[test]
    fn a_recursive_module_is_reported_not_expanded_forever() {
        let src = "module r (a); input a; r again (.a(a)); INV u (.A(a)); endmodule\n\
                   module top (a); input a; r x (.a(a)); endmodule\n";
        let (nl, rep) = parse_flat(src, &FlattenOptions::default()).unwrap();
        assert_eq!(rep.recursive, ["r"]);
        let names: Vec<&str> = nl.insts.iter().map(|x| x.name.as_str()).collect();
        assert_eq!(names, ["x/again", "x/u"]);
    }

    /// The owned design flattens as the compact one does.
    #[test]
    fn the_owned_form_flattens_as_the_compact_one() {
        let c = parse_compact(THREE_LEVELS).unwrap();
        let (cflat, crep) = c.flatten(&FlattenOptions::default());
        let (oflat, orep) = parse_all(THREE_LEVELS).unwrap().flatten(&FlattenOptions::default());
        assert_eq!(crep, orep);
        let a = cflat.top_netlist();
        let b = oflat.top();
        assert_eq!(a.insts.iter().map(|x| (&x.name, &x.conns)).collect::<Vec<_>>(), b.insts.iter().map(|x| (&x.name, &x.conns)).collect::<Vec<_>>());
    }
}
