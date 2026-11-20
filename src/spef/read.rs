//! The SPEF reader itself, every name held once as an id while it reads.
//!
//! [`Spef::parse`] and the analysis form ([`CompactSpef`]) are both made from what this reads, so
//! there is one reader. It used to build the String form first: each node a `String` once per
//! entry that names it, the two-node caps held as strings until every block was read, and the
//! node-to-net maps keyed by strings. On a 1.7M-net file that was about 50 GiB live before the
//! analysis form, 11 GiB, was made from it. Here a name is interned when it is first met and
//! everything after holds a 4-byte id; [`Read::into_spef`] spells the ids back out and
//! [`Read::into_compact`] renumbers them in the analysis form's own order, so each result is the
//! same as before to the bit.
//!
//! The reading rules are the String reader's, in its order; see the comments at each one.

use super::compact::{exact, CompactNet, CompactSpef, Name, NO_NAME};
use super::{is_triplet, keyword, parse_val, read_scaled, Lexer, NetRc, PinConn, Sect, Spef, WalkHasher, WalkMap};
use crate::names::unescape;
use std::borrow::Cow;
use std::collections::BTreeMap;
use std::hash::{BuildHasher, BuildHasherDefault};

/// [`unescape`] that borrows when there is nothing to unescape.
fn unesc(s: &str) -> Cow<'_, str> {
    if s.contains('\\') { Cow::Owned(unescape(s)) } else { Cow::Borrowed(s) }
}

/// Every name the reader meets, each once, ids in first-seen order: the text of all of them in
/// one buffer, each id's end offset, and a table from a name's hash to its id (the rare second
/// name with the same 64-bit hash goes to `more`).
#[derive(Default)]
pub(crate) struct Names {
    text: String,
    ends: Vec<usize>,
    index: WalkMap<u64, Name>,
    more: WalkMap<u64, Vec<Name>>,
}

impl Names {
    fn hash(s: &str) -> u64 {
        BuildHasherDefault::<WalkHasher>::default().hash_one(s)
    }
    pub(crate) fn len(&self) -> usize {
        self.ends.len()
    }
    pub(crate) fn get(&self, id: Name) -> &str {
        let i = id as usize;
        let start = if i == 0 { 0 } else { self.ends[i - 1] };
        &self.text[start..self.ends[i]]
    }
    pub(crate) fn find(&self, s: &str) -> Option<Name> {
        let h = Self::hash(s);
        let &first = self.index.get(&h)?;
        if self.get(first) == s {
            return Some(first);
        }
        self.more.get(&h)?.iter().copied().find(|&i| self.get(i) == s)
    }
    pub(crate) fn intern(&mut self, s: &str) -> Name {
        if let Some(i) = self.find(s) {
            return i;
        }
        let id = Name::try_from(self.ends.len()).ok().filter(|&i| i != NO_NAME).expect("more than 2^32 - 1 names in one SPEF");
        self.text.push_str(s);
        self.ends.push(self.text.len());
        match self.index.entry(Self::hash(s)) {
            std::collections::hash_map::Entry::Occupied(e) => self.more.entry(*e.key()).or_default().push(id),
            std::collections::hash_map::Entry::Vacant(e) => {
                e.insert(id);
            }
        }
        id
    }
}

/// A `*CONN` entry, names as ids ([`PinConn`]).
struct IdConn {
    inst: Name,
    pin: Name,
    node: Name,
    dir: crate::lef::PinDir,
    cap_ff: f64,
    is_port: bool,
}

/// One net as read, names as ids ([`NetRc`], field for field).
#[derive(Default)]
struct IdNet {
    cap_ff: f64,
    res_ohm: f64,
    coupling_ff: f64,
    coupling: Vec<(Name, f64)>,
    coupling_nodes: Vec<(Name, Name, Name, f64)>,
    net_node: Name,
    ground: Vec<(Name, f64)>,
    res: Vec<(Name, Name, f64)>,
    pins: Vec<(Name, Name, Name)>,
    conns: Vec<IdConn>,
}

/// A whole file as read: the names and each net by its name's id.
pub(crate) struct Read {
    names: Names,
    nets: WalkMap<Name, IdNet>,
    skipped: usize,
    unresolved: usize,
    triplets: bool,
    reduced: usize,
    pnets: usize,
}

/// A two-node cap awaiting resolution: (block serial, block net, node a, node b, fF); a node the
/// file does not resolve is [`NO_NAME`].
type PendingCc = (u32, Name, Name, Name, f64);

/// The name map: index -> the name's id.
type NameMap = WalkMap<usize, Name>;

/// Resolve a node reference to REAL NAMES: `*3:Y` -> `g1:Y`, `*1` -> `n1`, `*1:2` -> `n1:2`.
///
/// A node reference is a name-map index (or a literal name) followed by an optional
/// `<delimiter><suffix>` naming a pin or an internal node number; only the head is mapped.
/// Keeping the raw index made every node meaningless to every consumer — `3:Y` says nothing about
/// which instance that is — and it broke our own writer, which cannot tell the node `3:Y` from a
/// net someone NAMED `3:Y`. That is what a read-write-read cycle over the whole structure catches
/// and a field-by-field round trip does not, because nobody thinks to assert on node identity.
///
/// The head and suffix split at the LAST declared delimiter, as the reference splits them, and
/// the node is written back with `:`, the only delimiter our writer emits. `None` for an index
/// the name map does not hold: the reference resolves it to nothing and drops the entry.
/// Here as the id of that name.
fn node_tok(t: &str, map: &NameMap, names: &mut Names, delim: char, buf: &mut String) -> Option<Name> {
    let head = |h: &str, names: &mut Names| -> Option<Name> {
        match h.strip_prefix('*') {
            Some(body) => match body.parse::<usize>() {
                Ok(i) => map.get(&i).copied(),
                Err(_) => Some(names.intern(body)),
            },
            None => Some(names.intern(h)),
        }
    };
    match t.rsplit_once(delim) {
        Some((h, suf)) => {
            let hid = head(h, names)?;
            buf.clear();
            buf.push_str(names.get(hid));
            buf.push(':');
            buf.push_str(suf);
            Some(names.intern(buf))
        }
        None => head(t, names),
    }
}

/// A SPEF identifier — a `*NAME_MAP` index (`*17`) or a **literal name**. `None` for an index the
/// map does not hold.
///
/// The map is OPTIONAL in IEEE 1481 and extractors do write names directly; resolving only the
/// mapped form discarded the whole file. The leading `*` is the discriminator the format itself
/// uses — do NOT infer it from whether the body parses as a number: a literal net may
/// legitimately be named `123`.
/// Here as an id.
fn resolve(tok: &str, map: &NameMap, names: &mut Names) -> Option<Name> {
    match tok.strip_prefix('*') {
        Some(body) => match body.parse::<usize>() {
            Ok(i) => map.get(&i).copied(),
            Err(_) => Some(names.intern(body)),
        },
        None => Some(names.intern(&unesc(tok))),
    }
}

/// An instance pin `<inst><delim><pin>` -> (instance name, pin), split at the LAST delimiter.
fn pin_of(tok: &str, map: &NameMap, names: &mut Names, delim: char) -> Option<(Name, Name)> {
    let (ids, pin) = tok.rsplit_once(delim)?;
    let inst = resolve(ids, map, names)?;
    Some((inst, names.intern(pin)))
}

impl Read {
    /// Read a SPEF text. The rules, and their order, are [`Spef::parse`]'s.
    pub(crate) fn parse(text: &str) -> Read {
        let mut toks = Lexer::new(text);
        let mut names = Names::default();
        let mut buf = String::new();
        let mut c_scale = 1.0f64;
        let mut r_scale = 1.0f64;
        let mut delim = ':';
        let mut map: NameMap = NameMap::default();
        let mut nets: WalkMap<Name, IdNet> = WalkMap::default();
        // the block that won for each net — see `Duplicate *D_NET` below
        let mut winner: WalkMap<Name, u32> = WalkMap::default();
        let mut cur: Option<(Name, u32, IdNet)> = None;
        let mut serial = 0u32;
        let mut sect = Sect::None;
        // A node's net, from the entries that tie it to one — see `owner_of`.
        let mut conn_owner: WalkMap<Name, Name> = WalkMap::default();
        let mut weak_owner: WalkMap<Name, Name> = WalkMap::default();
        let mut pending_cc: Vec<PendingCc> = Vec::new();
        let (mut skipped, mut unresolved, mut reduced, mut pnets) = (0usize, 0usize, 0usize, 0usize);
        let mut triplets = false;
        // The design's top-level ports as the file declares them: a node named without a
        // delimiter is a port or nothing.
        let mut ports: super::WalkSet<Name> = super::WalkSet::default();
        let mut port_name_next = true;

        let finish = |cur: &mut Option<(Name, u32, IdNet)>, nets: &mut WalkMap<Name, IdNet>, winner: &mut WalkMap<Name, u32>| {
            if let Some((name, ser, rc)) = cur.take() {
                // Duplicate `*D_NET`: the LAST block replaces the net entirely.
                winner.insert(name, ser);
                nets.insert(name, rc);
            }
        };

        let mut i = 0;
        loop {
            toks.release(i);
            let Some(t) = toks.get(i) else { break };
            let kw = keyword(t);
            // Inside a block this reader does not model, only the block's end (or the start of
            // another block) matters.
            if sect == Sect::Skip && !matches!(kw, Some("*END" | "*D_NET" | "*R_NET" | "*D_PNET" | "*R_PNET")) {
                i += 1;
                continue;
            }
            if let Some(kw) = kw {
                i += 1;
                match kw {
                    "*C_UNIT" | "*R_UNIT" | "*T_UNIT" | "*L_UNIT" => {
                        let scale = toks.get(i).and_then(parse_val).unwrap_or(1.0) as f32 as f64;
                        let unit = toks.get(i + 1).map(|u| u.to_ascii_uppercase()).unwrap_or_default();
                        match kw {
                            "*C_UNIT" => c_scale = scale * match unit.as_str() { "PF" => 1e3, "NF" => 1e6, _ => 1.0 },
                            "*R_UNIT" => r_scale = scale * match unit.as_str() { "KOHM" => 1e3, "MOHM" => 1e6, _ => 1.0 },
                            _ => {}
                        }
                        i += 2;
                    }
                    "*DELIMITER" => {
                        if let Some(c) = toks.get(i).and_then(|d| d.chars().next()) {
                            delim = c;
                        }
                        i += 1;
                    }
                    "*NAME_MAP" => sect = Sect::NameMap,
                    "*POWER_NETS" | "*GROUND_NETS" | "*PHYSICAL_PORTS" | "*DEFINE" | "*PDEFINE" => sect = Sect::Ignore,
                    "*PORTS" => {
                        sect = Sect::Ports;
                        port_name_next = true;
                    }
                    "*D_NET" => {
                        finish(&mut cur, &mut nets, &mut winner);
                        serial += 1;
                        let idtok = toks.get(i).unwrap_or("");
                        let total = toks.get(i + 1).unwrap_or("");
                        triplets |= is_triplet(total);
                        i += 2;
                        let name = resolve(idtok, &map, &mut names);
                        let net_node = name.and_then(|_| node_tok(idtok, &map, &mut names, delim, &mut buf));
                        match (name, net_node) {
                            (Some(name), Some(net_node)) => {
                                let stated = read_scaled(total, c_scale).unwrap_or(0.0);
                                weak_owner.insert(net_node, name);
                                let rc = IdNet { cap_ff: stated, net_node, ..Default::default() };
                                cur = Some((name, serial, rc));
                                sect = Sect::Net;
                            }
                            _ => {
                                unresolved += 1;
                                sect = Sect::Skip;
                            }
                        }
                    }
                    "*R_NET" | "*D_PNET" | "*R_PNET" => {
                        finish(&mut cur, &mut nets, &mut winner);
                        if kw == "*R_NET" { reduced += 1 } else { pnets += 1 }
                        sect = Sect::Skip;
                    }
                    "*CONN" => sect = Sect::Conn,
                    "*CAP" => sect = Sect::Cap,
                    "*RES" => sect = Sect::Res,
                    "*INDUC" => sect = Sect::Induc,
                    "*END" => {
                        finish(&mut cur, &mut nets, &mut winner);
                        sect = Sect::None;
                    }
                    _ => sect = Sect::Header,
                }
                continue;
            }
            match sect {
                Sect::NameMap => match (t.strip_prefix('*').and_then(|d| d.parse::<usize>().ok()), toks.get(i + 1)) {
                    (Some(id), Some(name)) => {
                        let name = name.strip_prefix('"').and_then(|x| x.strip_suffix('"')).unwrap_or(name);
                        let n = names.intern(&unesc(name));
                        map.insert(id, n);
                        i += 2;
                    }
                    _ => {
                        skipped += 1;
                        i += 1;
                    }
                },
                Sect::Ports => match t.to_ascii_uppercase().as_str() {
                    "*C" | "*S" => i += 3,
                    "*L" | "*D" => i += 2,
                    "I" | "O" | "B" if !port_name_next => {
                        port_name_next = true;
                        i += 1;
                    }
                    _ => {
                        if let Some(p) = resolve(t, &map, &mut names) {
                            ports.insert(p);
                        }
                        port_name_next = false;
                        i += 1;
                    }
                },
                Sect::Conn => {
                    let up = t.to_ascii_uppercase();
                    if up != "*P" && up != "*I" && up != "*N" {
                        skipped += 1;
                        i += 1;
                        continue;
                    }
                    let node = toks.get(i + 1).unwrap_or("");
                    i += 2;
                    let dir = if up == "*N" {
                        None
                    } else {
                        match toks.get(i) {
                            Some(d @ ("I" | "O" | "B")) => {
                                i += 1;
                                Some(d)
                            }
                            _ => None,
                        }
                    };
                    let mut load = 0.0;
                    while let Some(a) = toks.get(i) {
                        match a.to_ascii_uppercase().as_str() {
                            "*C" | "*S" => i += 3,
                            "*L" => {
                                load = toks.get(i + 1).and_then(|v| read_scaled(v, c_scale)).unwrap_or(0.0);
                                i += 2;
                            }
                            "*D" => i += 2,
                            _ => break,
                        }
                    }
                    if up == "*N" {
                        continue;
                    }
                    let is_port = up == "*P";
                    if cur.is_none() {
                        // the String reader resolved the node first; resolving interns, which
                        // changes no result
                        let _ = node_tok(node, &map, &mut names, delim, &mut buf);
                        unresolved += 1;
                        continue;
                    }
                    let Some(node_id) = node_tok(node, &map, &mut names, delim, &mut buf) else {
                        unresolved += 1;
                        continue;
                    };
                    let hookup = if is_port {
                        resolve(node, &map, &mut names).inspect(|&p| {
                            ports.insert(p);
                        }).map(|p| (names.intern(""), p))
                    } else {
                        pin_of(node, &map, &mut names, delim)
                    };
                    let Some((inst, pin)) = hookup else {
                        unresolved += 1;
                        continue;
                    };
                    let dir = match dir {
                        Some("O") => crate::lef::PinDir::Output,
                        Some("B") => crate::lef::PinDir::Inout,
                        Some("I") => crate::lef::PinDir::Input,
                        _ => crate::lef::PinDir::Unknown,
                    };
                    let Some((cname, _, rc)) = cur.as_mut() else { continue };
                    conn_owner.insert(node_id, *cname);
                    if !is_port {
                        rc.pins.push((inst, pin, node_id));
                    }
                    rc.conns.push(IdConn { inst, pin, node: node_id, dir, cap_ff: load, is_port });
                }
                Sect::Cap | Sect::Res | Sect::Induc => {
                    if !t.bytes().all(|c| c.is_ascii_digit()) {
                        skipped += 1;
                        i += 1;
                        continue;
                    }
                    let a = toks.get(i + 1).unwrap_or("");
                    let b = toks.get(i + 2).unwrap_or("");
                    let grounded = sect == Sect::Cap && parse_val(b).is_some();
                    let vt = if grounded { b } else { toks.get(i + 3).unwrap_or("") };
                    i += if grounded { 3 } else { 4 };
                    let scale = if sect == Sect::Cap { c_scale } else { r_scale };
                    let Some(v) = read_scaled(vt, scale) else {
                        skipped += 1;
                        continue;
                    };
                    triplets |= is_triplet(vt);
                    let Some((name, ser, rc)) = cur.as_mut() else { continue };
                    // a node without the delimiter is a top-level port, else nothing
                    let mut node_of = |raw: &str| node_tok(raw, &map, &mut names, delim, &mut buf).filter(|n| raw.contains(delim) || ports.contains(n));
                    let na = node_of(a);
                    match sect {
                        Sect::Induc => {}
                        Sect::Res => {
                            let nb = node_of(b);
                            let (Some(na), Some(nb)) = (na, nb) else {
                                unresolved += 1;
                                continue;
                            };
                            weak_owner.insert(na, *name);
                            weak_owner.insert(nb, *name);
                            rc.res_ohm += v;
                            rc.res.push((na, nb, v));
                        }
                        _ if grounded => {
                            let Some(na) = na else {
                                unresolved += 1;
                                continue;
                            };
                            weak_owner.insert(na, *name);
                            rc.ground.push((na, v));
                        }
                        _ => {
                            let nb = node_of(b);
                            pending_cc.push((*ser, *name, na.unwrap_or(NO_NAME), nb.unwrap_or(NO_NAME), v));
                        }
                    }
                }
                _ => i += 1,
            }
        }
        finish(&mut cur, &mut nets, &mut winner);

        // ── a node's net: the String reader's `owner_of`, on ids ──
        let owner_of = |node: Name, names: &Names| -> Option<Name> {
            if let Some(&o) = conn_owner.get(&node) {
                return Some(o);
            }
            let s = names.get(node);
            if let Some((head, suf)) = s.rsplit_once(':') {
                let head = unesc(head);
                if suf.bytes().all(|c| c.is_ascii_digit()) {
                    if let Some(h) = names.find(&head).filter(|h| nets.contains_key(h)) {
                        return Some(h);
                    }
                }
            }
            if let Some(&o) = weak_owner.get(&node) {
                return Some(o);
            }
            let bare = unesc(s);
            if s.contains(':') {
                return None;
            }
            names.find(&bare).filter(|b| nets.contains_key(b))
        };

        // ── the coupling caps: the String reader's rules, on ids ──
        let mut coupling: WalkMap<Name, f64> = WalkMap::default();
        // per net, each aggressor's summed Cc, in first-seen order (sorted by name below)
        let mut agg_at: WalkMap<(Name, Name), usize> = WalkMap::default();
        let mut coupling_list: WalkMap<Name, Vec<(Name, f64)>> = WalkMap::default();
        #[allow(clippy::type_complexity)]
        let mut coupling_nodes: WalkMap<Name, Vec<(Name, Name, Name, f64)>> = WalkMap::default();
        let mut ground_extra: Vec<(Name, Name, f64)> = Vec::new();
        let empty = names.intern("");
        for (ser, net, a, b, v) in pending_cc {
            if winner.get(&net) != Some(&ser) || v <= 0.0 {
                continue;
            }
            let on = |x: Name| x != NO_NAME && owner_of(x, &names) == Some(net);
            let (la, lb) = (on(a), on(b));
            match (la, lb) {
                (true, true) => {
                    ground_extra.push((net, a, v));
                    ground_extra.push((net, b, v));
                }
                (true, false) | (false, true) => {
                    let (local, far) = if la { (a, b) } else { (b, a) };
                    let local = if local == NO_NAME { empty } else { local };
                    let far = if far == NO_NAME { empty } else { far };
                    let agg = match owner_of(far, &names) {
                        Some(o) => o,
                        None => {
                            let f = names.get(far);
                            let h = unesc(f.rsplit_once(':').map(|(h, _)| h).unwrap_or(f)).into_owned();
                            names.intern(&h)
                        }
                    };
                    *coupling.entry(net).or_default() += v;
                    let list = coupling_list.entry(net).or_default();
                    match agg_at.get(&(net, agg)) {
                        Some(&k) => list[k].1 += v,
                        None => {
                            agg_at.insert((net, agg), list.len());
                            list.push((agg, v));
                        }
                    }
                    coupling_nodes.entry(net).or_default().push((agg, local, far, v));
                }
                (false, false) => unresolved += 1,
            }
        }
        drop(agg_at);
        for (net, node, v) in ground_extra {
            if let Some(rc) = nets.get_mut(&net) {
                rc.ground.push((node, v));
            }
        }
        for (name, rc) in nets.iter_mut() {
            rc.coupling_ff = coupling.get(name).copied().unwrap_or(0.0);
            rc.coupling_nodes = coupling_nodes.remove(name).unwrap_or_default();
            // the String reader's CANONICAL ORDER: by (aggressor, local, far) NAME
            rc.coupling_nodes.sort_by(|a, b| (names.get(a.0), names.get(a.1), names.get(a.2)).cmp(&(names.get(b.0), names.get(b.1), names.get(b.2))));
            // one entry per aggressor, in aggressor NAME order (the String reader's map range)
            let mut list = coupling_list.remove(name).unwrap_or_default();
            list.sort_by(|a, b| names.get(a.0).cmp(names.get(b.0)));
            rc.coupling = list;
            let summed: f64 = rc.ground.iter().map(|(_, c)| c).sum::<f64>() + rc.coupling_ff;
            if !rc.ground.is_empty() || rc.coupling_ff != 0.0 {
                rc.cap_ff = summed;
            }
        }
        Read { names, nets, skipped, unresolved, triplets, reduced, pnets }
    }

    /// The String form ([`Spef`]), each id spelled out.
    pub(crate) fn into_spef(self) -> Spef {
        let Read { names, nets, skipped, unresolved, triplets, reduced, pnets } = self;
        let s = |id: Name| names.get(id).to_string();
        let nets: BTreeMap<String, NetRc> = nets
            .into_iter()
            .map(|(name, rc)| {
                let rc = NetRc {
                    cap_ff: rc.cap_ff,
                    res_ohm: rc.res_ohm,
                    coupling_ff: rc.coupling_ff,
                    coupling: rc.coupling.into_iter().map(|(a, c)| (s(a), c)).collect(),
                    coupling_nodes: rc.coupling_nodes.into_iter().map(|(a, l, f, c)| (s(a), s(l), s(f), c)).collect(),
                    net_node: s(rc.net_node),
                    ground: rc.ground.into_iter().map(|(n, c)| (s(n), c)).collect(),
                    res: rc.res.into_iter().map(|(a, b, r)| (s(a), s(b), r)).collect(),
                    pins: rc.pins.into_iter().map(|(i, p, n)| (s(i), s(p), s(n))).collect(),
                    conns: rc.conns.into_iter().map(|c| PinConn { inst: s(c.inst), pin: s(c.pin), node: s(c.node), dir: c.dir, cap_ff: c.cap_ff, is_port: c.is_port }).collect(),
                };
                (s(name), rc)
            })
            .collect();
        Spef { nets, skipped, unresolved, triplets, reduced, pnets }
    }

    /// [`Spef::rename_to_design`], on ids: a net the design names otherwise takes the design's
    /// name unless the file already has a net by that name; the moves go in net-name order (a
    /// later one onto the same name replaces the earlier); then every aggressor is renamed.
    pub(crate) fn rename_to_design(&mut self, nl: &crate::netlist::Netlist) -> usize {
        if nl.canonical.is_empty() {
            return 0;
        }
        let names = &mut self.names;
        let mut keys: Vec<Name> = self.nets.keys().copied().collect();
        keys.sort_by(|a, b| names.get(*a).cmp(names.get(*b)));
        let mut movable: Vec<(Name, Name)> = Vec::new();
        for k in keys {
            let Some(to) = nl.canonical.get(names.get(k)) else { continue };
            if names.find(to).is_some_and(|t| self.nets.contains_key(&t)) {
                continue;
            }
            let to = names.intern(to);
            movable.push((k, to));
        }
        let mut moved = 0usize;
        for (from, to) in movable {
            if let Some(rc) = self.nets.remove(&from) {
                self.nets.insert(to, rc);
                moved += 1;
            }
        }
        for rc in self.nets.values_mut() {
            for (agg, _) in rc.coupling.iter_mut() {
                if let Some(c) = nl.canonical.get(names.get(*agg)) {
                    *agg = names.intern(c);
                }
            }
        }
        moved
    }

    /// The analysis form, as [`CompactSpef::from_spef`] of [`Read::into_spef`] makes it: nets in
    /// name order and ids renumbered in the order that visits them (each net's name, its
    /// aggressors, its coupling caps' own nodes, its node, grounded caps, resistors, pins), so
    /// the same file gives the same ids either way.
    pub(crate) fn into_compact(self) -> CompactSpef {
        let Read { names, nets, skipped, unresolved, triplets, reduced, pnets } = self;
        let mut order: Vec<(Name, IdNet)> = nets.into_iter().collect();
        order.sort_by(|a, b| names.get(a.0).cmp(names.get(b.0)));
        let mut renum: Vec<Name> = vec![NO_NAME; names.len()];
        let mut out_names: Vec<Box<str>> = Vec::new();
        let mut id = |old: Name| -> Name {
            let slot = &mut renum[old as usize];
            if *slot == NO_NAME {
                *slot = Name::try_from(out_names.len()).expect("ids fit");
                out_names.push(names.get(old).into());
            }
            *slot
        };
        let mut out = Vec::with_capacity(order.len());
        for (name, rc) in order {
            let IdNet { cap_ff, res_ohm, coupling_ff, coupling, coupling_nodes, net_node, ground, res, pins, conns: _ } = rc;
            let name = id(name);
            let coupling = exact(coupling.into_iter().map(|(a, c)| (id(a), c)));
            let cc_own = exact(coupling_nodes.into_iter().map(|(_, own, _, c)| (id(own), c)));
            let net_node = id(net_node);
            let ground = exact(ground.into_iter().map(|(n, c)| (id(n), c)));
            let res = exact(res.into_iter().map(|(a, b, r)| (id(a), id(b), r)));
            let pins = exact(pins.into_iter().map(|(i, p, n)| (id(i), id(p), id(n))));
            out.push(CompactNet { name, cap_ff, res_ohm, coupling_ff, coupling, cc_own, net_node, ground, res, pins });
        }
        CompactSpef { names: out_names, nets: out, skipped, unresolved, triplets, reduced, pnets }
    }
}

#[cfg(test)]
mod order_rule {
    use super::super::Spef;

    /// The rule: a net's coupling caps are held in (aggressor, own node, far node) NAME order and
    /// its aggressors in NAME order, whatever order the file lists them in, so that writing back
    /// what was read is a fixed point. Ids are given in first-seen order, which is file order;
    /// sorting by id instead of name would keep the file's order.
    #[test]
    fn coupling_is_held_in_name_order_not_file_order() {
        let text = "*SPEF \"IEEE 1481-1998\"\n*DELIMITER :\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
                    *D_NET a 1.0\n*CONN\n*I u1:Y O\n*CAP\n1 a:2 z:1 0.5\n2 a:1 m:1 0.25\n3 a:1 b:1 0.125\n4 a:2 b:1 0.0625\n*RES\n1 u1:Y a:1 1.0\n2 a:1 a:2 1.0\n*END\n\
                    *D_NET z 1.0\n*CONN\n*I u2:Y O\n*RES\n1 u2:Y z:1 1.0\n*END\n\
                    *D_NET m 1.0\n*CONN\n*I u3:Y O\n*RES\n1 u3:Y m:1 1.0\n*END\n\
                    *D_NET b 1.0\n*CONN\n*I u4:Y O\n*RES\n1 u4:Y b:1 1.0\n*END\n";
        let s = Spef::parse(text);
        let a = &s.nets["a"];
        let nodes: Vec<(&str, &str, &str)> = a.coupling_nodes.iter().map(|(g, l, f, _)| (g.as_str(), l.as_str(), f.as_str())).collect();
        assert_eq!(nodes, vec![("b", "a:1", "b:1"), ("b", "a:2", "b:1"), ("m", "a:1", "m:1"), ("z", "a:2", "z:1")]);
        let aggs: Vec<(&str, f64)> = a.coupling.iter().map(|(g, c)| (g.as_str(), *c)).collect();
        assert_eq!(aggs, vec![("b", 0.1875), ("m", 0.25), ("z", 0.5)]);
    }
}
