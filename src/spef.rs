//! SPEF reader — per-net RC network for STA (lumped fallback + per-pin Elmore).
//!
//! STA needs the net capacitance loading the driver and the interconnect delay
//! to each sink. This reads the per-net total cap (`*D_NET`), the `*RES`
//! resistors, the grounded `*CAP` entries, the two-node `*CAP` coupling entries,
//! and the `*CONN` instance pins. From that it offers a lumped Elmore (`R·C`)
//! and a true **per-pin tree Elmore** (delay to each sink = Σ over the
//! driver→sink path of `R · downstream-cap`). Units are assumed fF / Ω (what
//! `vyges-extract` emits).
//!
//! Pure std — fully unit-tested offline.
//!
//! # Format decisions, and why they are not obvious
//!
//! Each of these was measured by writing a file and having OpenSTA read it back
//! (`tests/opensta.rs`), not derived from the standard alone. They are recorded here because
//! every one of them is a place where the obvious answer is wrong, and where our own tests
//! cannot tell — a reader and writer that share a mistaken convention round-trip perfectly.
//!
//! **A node reference is not a net reference.** `*<id>` alone denotes a top-level PORT.
//! A net's own node is `*<id>:0`, and a net-internal node is `*<id>:<n>`; an instance pin is
//! `*<instid>:<pin>`. Writing a bare `*<id>` for an internal net asks the reader for a port the
//! design does not have — OpenSTA drops the capacitor with `pin <name> not found` and reports a
//! faster path than the file describes. The exception is a net that IS a port, where the bare
//! form is exactly right.
//!
//! **Names are written as the source spelled them, and never re-escaped.** Both `count[0]` and
//! `count\[0\]` are legal SPEF and they mean different things: the grammar reads the first as
//! a BIT_IDENT (bit 0 of bus `count`) and the second as an ID whose characters include the
//! brackets. Which is correct depends on whether the netlist declares `output [7:0] count;` or
//! `wire \CFG_REG[0] ;` — the characters alone cannot say. Where there is no source spelling to
//! preserve, the plain form is the safe one: OpenSTA's lookup tries the name, then
//! `escapeDividers`, then `escapeBrackets`, then both, so it only ever ADDS escapes.
//! Under-escaping is recoverable; over-escaping is a hard miss.
//!
//! **A coupling capacitor is written in BOTH nets' blocks.** This looks like redundancy and is
//! not: a reader applies the capacitor to the net whose block it appears in, so listing it once
//! leaves the other net believing it is uncoupled. It is what OpenRCX does. The READER dedupes
//! by node pair — counting both listings would double every net's crosstalk — so the two halves
//! of this are asymmetric on purpose.
//!
//! **Numbers carry significant figures, not decimal places.** `{:.6}` is six places: it wrote
//! every capacitance below 5e-7 as `0.000000`, deleting the capacitor in a file that still
//! parses, and small coupling capacitors are exactly where that lands.
//!
//! **What the reader keeps that it does not itself need.** [`NetRc::coupling_nodes`] holds the
//! node pair each coupling capacitor came off, alongside the per-aggressor aggregate an SI pass
//! actually wants. The aggregate is lossy in a way that only matters on the way back OUT: put
//! the capacitor back on the wrong node and it still loads the net, but at the wrong point in
//! the RC network, and the timer answers differently for the same design.

use crate::names::unescape;
use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, Default)]
pub struct NetRc {
    pub cap_ff: f64,                  // total cap (grounded + coupling), from *D_NET
    pub res_ohm: f64,                 // summed *RES (lumped fallback)
    pub coupling_ff: f64,             // total coupling cap (sum over neighbours)
    pub coupling: Vec<(String, f64)>, // per-aggressor coupling (net, Cc) for window-aware SI
    /// Per NODE-PAIR coupling as read: `(aggressor net, this net's node, the aggressor's node,
    /// Cc)`. `coupling` above is the aggregate an SI pass wants; this is what the file said, and
    /// it is kept so the writer can put a capacitor back on the node it came off. Attached to
    /// the wrong node it still loads the net, but at the wrong point in the RC network, and the
    /// timer reports a different number for the same design. Empty when the parasitics were
    /// built rather than read.
    #[allow(clippy::type_complexity)]
    pub coupling_nodes: Vec<(String, String, String, f64)>,
    // RC network (for per-pin tree Elmore):
    pub net_node: String,                 // node where coupling attaches (the net node)
    pub ground: Vec<(String, f64)>,       // (node, grounded cap fF)
    pub res: Vec<(String, String, f64)>,  // (node a, node b, ohm)
    pub pins: Vec<(String, String, String)>, // (instance, pin, node) from *CONN
    /// Rich `*CONN` with direction + per-pin load cap (when a cell LEF / liberty
    /// resolved them). Parallel to `pins` (kept for back-compat); the writer emits
    /// from `conns` when non-empty, so driver (`O`) / load (`I`) marking + `*L cap`
    /// survive. Empty on a bare read with no direction/cap.
    pub conns: Vec<PinConn>,
}

/// A `*CONN` entry with direction and input-pin load capacitance (fF).
#[derive(Debug, Clone, Default)]
pub struct PinConn {
    /// Empty for a top-level port — see [`PinConn::is_port`].
    pub inst: String,
    pub pin: String,
    pub node: String,
    pub dir: crate::lef::PinDir,
    pub cap_ff: f64,
    /// A `*P` entry: the net reaches a TOP-LEVEL PORT rather than an instance pin. Dropping
    /// these — which is what happens if you only look for `<inst>:<pin>` — leaves every net that
    /// touches the boundary short one connection, and a timer loads it accordingly.
    pub is_port: bool,
}

#[derive(Debug, Clone, Default)]
pub struct Spef {
    pub nets: BTreeMap<String, NetRc>,
    /// Lines inside a recognised section that this reader could not interpret.
    ///
    /// **A parser that cannot fail cannot warn.** `parse` returns a `Spef` whatever it is given,
    /// so every gap it has ever had produced a quietly smaller answer rather than an error: a
    /// file of corner triplets read as zero parasitics, a lower-case file read as no nets at all.
    /// This is the tell. Non-zero on a file you expect to be understood means the reader is
    /// behind the writer — check it before trusting the numbers, and see [`Spef::health`].
    pub skipped: usize,
    /// The file carried **corner triplets** (`min:typ:max`) and this reader took the first
    /// field of each. Single-corner by construction; a caller that needs a specific corner
    /// must not read it from here.
    pub triplets: bool,
    /// `*R_NET` blocks seen — **reduced** (pi-model) parasitics, which this reader does not
    /// model. A file of these has no detailed RC to read, and without this the result is an
    /// empty design with no indication that the format, not the design, was the reason.
    pub reduced: usize,
    /// `*D_PNET` / `*R_PNET` blocks seen — power-net parasitics, skipped by design.
    pub pnets: usize,
}

impl Spef {
    /// One line on whether the read can be trusted, or `None` when nothing looks wrong.
    /// Cheap enough to log on every load, which is the point.
    pub fn health(&self) -> Option<String> {
        let mut notes = Vec::new();
        if self.nets.is_empty() {
            notes.push("no nets were parsed".to_string());
        }
        if self.skipped > 0 {
            notes.push(format!("{} line(s) in known sections not understood", self.skipped));
        }
        if self.triplets {
            notes.push("corner triplets present — the first (min) field was taken".to_string());
        }
        if self.reduced > 0 {
            notes.push(format!(
                "{} *R_NET block(s) skipped — reduced (pi-model) parasitics are not read",
                self.reduced
            ));
        }
        (!notes.is_empty()).then(|| notes.join("; "))
    }
}

/// Options for the SPEF writer ([`Spef::to_spef`]). Kept minimal and
/// deterministic — no wall-clock timestamp unless `date` is supplied, so the
/// same design extracts byte-identically (the suite's reproducibility contract).
#[derive(Debug, Clone)]
pub struct WriteOpts {
    pub design: String,  // *DESIGN "<name>"
    pub program: String, // *PROGRAM "<tool>"
    pub version: String, // *VERSION "<ver>"
    /// `*DATE` — a FIXED default is used when None. Never the wall clock: the field is required
    /// by the grammar, and output must stay byte-reproducible.
    pub date: Option<String>,
}

impl Default for WriteOpts {
    fn default() -> Self {
        WriteOpts {
            design: "top".into(),
            program: "vyges-loom".into(),
            version: "0".into(),
            date: None,
        }
    }
}

#[derive(Debug)]
pub struct SpefError(pub String);
impl std::fmt::Display for SpefError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "spef error: {}", self.0)
    }
}
impl std::error::Error for SpefError {}

/// Resolve a node reference to REAL NAMES: `*3:Y` -> `g1:Y`, `*1` -> `n1`, `*1:2` -> `n1:2`.
///
/// A node reference is a name-map index (or a literal name) followed by an optional `:<suffix>`
/// naming a pin or an internal node number; only the head is mapped. Keeping the raw index made
/// every node meaningless to every consumer — `3:Y` says nothing about which instance that is —
/// and it broke our own writer, which cannot tell the node `3:Y` from a net someone NAMED
/// `3:Y`. It interned it as a name, so the RC network we wrote no longer joined the pins in
/// `*CONN`: a driver attached to nothing, in a file that still parsed. That is what a
/// read-write-read cycle over the whole structure catches and a field-by-field round trip does
/// not, because nobody thinks to assert on node identity.
///
/// The suffix delimiter is `:`, the format's default and the only one this writer emits.
fn node_tok(t: &str, names: &BTreeMap<usize, String>) -> String {
    let head = |h: &str| -> String {
        match h.strip_prefix('*') {
            Some(body) => body
                .parse::<usize>()
                .ok()
                .and_then(|i| names.get(&i).cloned())
                .unwrap_or_else(|| body.to_string()),
            None => h.to_string(),
        }
    };
    match t.split_once(':') {
        Some((h, suf)) => format!("{}:{suf}", head(h)),
        None => head(t),
    }
}

/// A SPEF value may be a single number or a **corner triplet** `min:typ:max`, written whenever
/// more than one process corner was extracted. `f64::from_str` rejects the triplet outright, so
/// a reader that parses naively silently returns *no* capacitance and *no* resistance for every
/// net in such a file — the net still appears, it just has no parasitics, which a timer reads as
/// ideal interconnect. This reader is single-corner, so it takes the first field and says so.
fn parse_val(t: &str) -> Option<f64> {
    t.split(':').next()?.parse::<f64>().ok()
}

/// Did this value token carry a corner triplet?
fn is_triplet(t: &str) -> bool {
    t.contains(':') && t.split(':').count() > 1 && t.split(':').all(|f| f.parse::<f64>().is_ok())
}


/// Add SPEF name escaping — the inverse of [`unescape`].
///
/// **The declared delimiters are NOT escaped.** A SPEF header declares `*DIVIDER /`,
/// `*DELIMITER :` and `*BUS_DELIMITER [ ]` precisely so those characters can appear in a name
/// meaning what they say, and that is how every extractor writes them: OpenRCX emits
/// `*4 count[0]`, not `*4 count\[0\]`. Escaping them anyway makes the backslashes part of the
/// name — OpenSTA reports `net count\[0\] not found` and drops the parasitics for every bussed
/// net in the design, which on a hardened counter was ten of them and a 20 ps shift in slack.
///
/// What still has to be escaped is a character that would break the tokenising: a backslash
/// itself, and whitespace, which would end the name.
fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        if c == '\\' || c.is_whitespace() {
            out.push('\\');
        }
        out.push(c);
    }
    out
}

/// Spread a crosstalk capacitance over the wire it actually sits on.
///
/// **Not at `net_node`.** That field is the net's own node, and it is a node of the RC network
/// only when the extractor wrote the bare `*<netid>` form. A real OpenRCX file names its nodes
/// `*<netid>:<n>` and `*<instid>:<pin>`, so `net_node` matches nothing in the network: measured
/// on a routed sky130 block, it was a network node for **221 of 14238 nets**. For the other 98 %
/// every femtofarad of coupling was deposited on a node the tree never visits, and left the
/// delay without trace — `miller` could be set to any value at all and the answer did not move.
///
/// Coupling follows the wire, so it is distributed like the wire's own capacitance: in
/// proportion to each node's grounded cap, falling back to the driver when a net carries none.
/// A lumped net with one node degenerates to "all of it there", which is the old behaviour for
/// the files that behaved.
///
/// (The reader does keep the node each capacitor came off — see `NetRc::coupling_nodes` — but
/// the caller has already summed the window-overlapping aggressors by the time it gets here.
/// Passing that through per node would be better still, and needs the caller to change too.)
fn spread_xtalk<'a>(
    cap: &mut HashMap<&'a str, f64>,
    ground: &'a [(String, f64)],
    driver: &'a str,
    xtalk_cap_ff: f64,
) {
    if xtalk_cap_ff == 0.0 {
        return;
    }
    let total: f64 = ground.iter().map(|(_, c)| *c).sum();
    if total > 0.0 {
        for (n, c) in ground {
            *cap.entry(n.as_str()).or_default() += xtalk_cap_ff * (c / total);
        }
    } else {
        *cap.entry(driver).or_default() += xtalk_cap_ff;
    }
}

/// Drop a `//` line comment, which is legal anywhere in a SPEF line. Left in place it becomes
/// extra tokens, and an entry's field count is how this format distinguishes a grounded cap from
/// a coupling one — so a trailing comment turns a ground cap into an unparseable coupling cap.
fn strip_comment(line: &str) -> &str {
    match line.find("//") {
        Some(i) => &line[..i],
        None => line,
    }
}

impl NetRc {
    /// Per-node Elmore delays (ns) for the net's RC tree rooted at `driver`,
    /// with `xtalk_cap_ff` spread over the wire (the Miller crosstalk load) — see [`spread_xtalk`].
    /// Returns `None` if the network is not a tree reachable from the driver
    /// (caller falls back to the lumped delay).
    /// Per-sink **Elmore time constant** (`sum R_k * C_downstream(k)`), keyed by SPEF node.
    ///
    /// ⚠️ It is a TIME CONSTANT, not a 50 % delay. The reference converts it —
    /// `wire_delay = -tau * ln(1 - Vth)` in `DelayCalcBase::dspfWireDelaySlew` — so a
    /// caller that uses tau directly over-states the delay by 1/ln(2) = 1.44x.
    ///
    /// `pin_cap_ff` supplies the RECEIVER capacitance at each load node. SPEF carries the
    /// wire's own capacitance; the load pins' input capacitance comes from Liberty, and
    /// upstream adds it during parasitic reduction unless the SPEF declared pin caps
    /// (`read_spef -pin_cap_included`). Omitting it leaves every subtree capacitance short
    /// and the network too fast — measured on a fanout-298 net, tau came out 4.7x below
    /// the reference's.
    pub fn elmore(
        &self,
        driver: &str,
        xtalk_cap_ff: f64,
        pin_cap_ff: &BTreeMap<String, f64>,
    ) -> Option<BTreeMap<String, f64>> {
        if self.res.is_empty() {
            return None;
        }
        // node capacitances
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (node, c) in &self.ground {
            *cap.entry(node.as_str()).or_default() += c;
        }
        spread_xtalk(&mut cap, &self.ground, driver, xtalk_cap_ff);
        // Receiver capacitance sits ON its load node, not spread over the wire.
        for (node, c) in pin_cap_ff {
            if let Some(k) = self
                .ground
                .iter()
                .map(|(n, _)| n)
                .chain(self.res.iter().flat_map(|(a, b, _)| [a, b]))
                .find(|n| *n == node)
            {
                *cap.entry(k.as_str()).or_default() += *c;
            }
        }
        // adjacency
        let mut adj: HashMap<&str, Vec<(&str, f64)>> = HashMap::new();
        for (a, b, r) in &self.res {
            adj.entry(a).or_default().push((b, *r));
            adj.entry(b).or_default().push((a, *r));
            cap.entry(a.as_str()).or_default();
            cap.entry(b.as_str()).or_default();
        }
        if !adj.contains_key(driver) {
            return None;
        }
        // BFS tree from the driver; record parent + parent-edge R, in visit order
        let mut parent: HashMap<&str, (&str, f64)> = HashMap::new();
        let mut order: Vec<&str> = vec![driver];
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        seen.insert(driver);
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for &(v, r) in adj.get(u).map(|x| x.as_slice()).unwrap_or(&[]) {
                if !seen.insert(v) {
                    if parent.get(u).map(|p| p.0) != Some(v) {
                        return None; // a cycle reached an already-visited node -> not a tree
                    }
                    continue;
                }
                parent.insert(v, (u, r));
                order.push(v);
            }
        }
        // subtree caps: reverse BFS order accumulates child caps into parents
        let mut sub: HashMap<&str, f64> = HashMap::new();
        for &nd in &order {
            *sub.entry(nd).or_default() += cap.get(nd).copied().unwrap_or(0.0);
        }
        for &nd in order.iter().skip(1).rev() {
            let (p, _) = parent[nd];
            let add = sub[nd];
            *sub.get_mut(p).unwrap() += add;
        }
        // delays: delay[child] = delay[parent] + R_edge * subtree_cap[child]
        let mut delay: BTreeMap<String, f64> = BTreeMap::new();
        delay.insert(driver.to_string(), 0.0);
        for &nd in order.iter().skip(1) {
            let (p, r) = parent[nd];
            // ⛔ TYPES, from the reference (see `cpp-to-rust-numeric-reference.md` §1):
            // `ReduceToPiElmore::reduceElmoreDfs` computes
            //   `double onode_elmore = elmore + r * downstreamCap(onode);`
            // where BOTH `Parasitics::value(ParasiticResistor*)` and
            // `ReduceToPi::downstreamCap` return **float**. So the per-step increment is an
            // f32 multiply — about 7 significant digits — and only the running sum is
            // double. Computing the increment in f64 is MORE ACCURATE than the reference,
            // which is a divergence here, not an improvement.
            let inc = (r as f32 * sub[nd] as f32) as f64 * 1e-6; // R[Ω]·C[fF] -> ns
            let d = delay[p] + inc;
            delay.insert(nd.to_string(), d);
        }
        Some(delay)
    }

    /// Transient node response: drive the RC tree with the driver's output edge as a
    /// forced saturated ramp from t=0, integrate with backward Euler over the rooted
    /// tree (an O(N) up/down sweep per step), and read each node's delay and slew at the
    /// **library's own measurement thresholds** (`th`). `xtalk_cap_ff` adds at the net
    /// node. This is the waveform-into-RC convolution — more accurate than Elmore (a
    /// single RC gives 0.69·RC, not R·C).
    ///
    /// `driver_slew_ns` is a **Liberty transition time**, i.e. the time to cross from
    /// `th.slew_lower_*` to `th.slew_upper_*` — *not* a full 0→100 % edge. The ramp is
    /// therefore stretched by `1/th.slew_span()`, and the result is read back between the
    /// same two thresholds so it is directly comparable to a Liberty slew.
    ///
    /// Getting either end wrong rescales every delay in the design: treating a 20–80 %
    /// slew as a full edge and reading 30→70 % out returns `0.4×` the input on a lightly
    /// loaded net, so a sink looks *sharper* than its driver — which RC can never do.
    ///
    /// Returns node → (delay_ns, slew_ns), or None if not a tree from `driver`.
    pub fn transient(
        &self,
        driver: &str,
        driver_slew_ns: f64,
        xtalk_cap_ff: f64,
        th: crate::liberty::Thresholds,
    ) -> Option<BTreeMap<String, (f64, f64)>> {
        if self.res.is_empty() {
            return None;
        }
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (n, c) in &self.ground {
            *cap.entry(n.as_str()).or_default() += c;
        }
        spread_xtalk(&mut cap, &self.ground, driver, xtalk_cap_ff);
        let mut adj: HashMap<&str, Vec<(&str, f64)>> = HashMap::new();
        for (a, b, r) in &self.res {
            adj.entry(a).or_default().push((b, *r));
            adj.entry(b).or_default().push((a, *r));
            cap.entry(a.as_str()).or_default();
            cap.entry(b.as_str()).or_default();
        }
        if !adj.contains_key(driver) {
            return None;
        }
        // rooted tree (BFS): parent + parent-edge R
        let mut parent: HashMap<&str, (&str, f64)> = HashMap::new();
        let mut order: Vec<&str> = vec![driver];
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        seen.insert(driver);
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            for &(v, r) in adj.get(u).map(|x| x.as_slice()).unwrap_or(&[]) {
                if !seen.insert(v) {
                    if parent.get(u).map(|p| p.0) != Some(v) {
                        return None; // not a tree
                    }
                    continue;
                }
                parent.insert(v, (u, r));
                order.push(v);
            }
        }
        let nn = order.len();
        let idx: HashMap<&str, usize> = order.iter().enumerate().map(|(i, &n)| (n, i)).collect();
        let cvec: Vec<f64> = order.iter().map(|&n| cap.get(n).copied().unwrap_or(0.0)).collect();
        let mut par_idx = vec![usize::MAX; nn];
        let mut par_r = vec![0.0f64; nn];
        let mut children: Vec<Vec<usize>> = vec![Vec::new(); nn];
        for (i, &n) in order.iter().enumerate() {
            if let Some(&(p, r)) = parent.get(n) {
                let pi = idx[p];
                par_idx[i] = pi;
                par_r[i] = r;
                children[pi].push(i);
            }
        }
        // time grid: ramp + ~6 lumped time constants, fixed step count
        let total_c: f64 = cvec.iter().sum();
        let total_r: f64 = self.res.iter().map(|(_, _, r)| r).sum();
        let tau_lump = (total_r * total_c * 1e-6).max(1e-6); // ns
        // A Liberty slew spans only `slew_span` of the full swing, so the 0→100 % ramp
        // that produced it lasts proportionally longer.
        let span = th.slew_span();
        let tr = (driver_slew_ns / span).max(1e-4); // full 0->100% ramp duration
        let nsteps = 800usize;
        let dt = ((tr + 6.0 * tau_lump) / nsteps as f64).max(1e-7);
        let vdrv = |t: f64| if t <= 0.0 { 0.0 } else if t >= tr { 1.0 } else { t / tr };

        // measurement points, from the library (normalized rising edge: the tree is
        // linear, so a falling edge is the mirror image and shares these spans)
        let (v_lo, v_hi) = (th.slew_lower_rise, th.slew_upper_rise);
        let (v_lo, v_hi) = if v_hi > v_lo { (v_lo, v_hi) } else { (0.2, 0.8) };
        let v_mid = th.output_rise.clamp(0.01, 0.99);

        let didx = idx[driver];
        let mut v = vec![0.0f64; nn];
        let (mut t_lo, mut t_mid, mut t_hi) =
            (vec![f64::INFINITY; nn], vec![f64::INFINITY; nn], vec![f64::INFINITY; nn]);
        let mut a_co = vec![0.0f64; nn];
        let mut b_co = vec![0.0f64; nn];
        let mut vnew = vec![0.0f64; nn];
        let mut t = 0.0;
        for _ in 0..nsteps {
            t += dt;
            let vd = vdrv(t);
            // up-sweep (leaves->root): V_i = a_co[i]*V_parent + b_co[i]
            for &n in order.iter().rev() {
                let i = idx[n];
                if i == didx {
                    continue;
                }
                let gc = cvec[i] * 1e-6 / dt; // cap conductance (scaled to S)
                let gpar = 1.0 / par_r[i];
                let mut diag = gc + gpar;
                let mut rhs = gc * v[i];
                for &c in &children[i] {
                    let gr = 1.0 / par_r[c];
                    diag += gr - gr * a_co[c];
                    rhs += gr * b_co[c];
                }
                a_co[i] = gpar / diag;
                b_co[i] = rhs / diag;
            }
            // down-sweep (root forced)
            vnew[didx] = vd;
            for &n in &order {
                let i = idx[n];
                if i != didx {
                    vnew[i] = a_co[i] * vnew[par_idx[i]] + b_co[i];
                }
            }
            // record threshold crossings (linear interp within the step)
            for i in 0..nn {
                let cross = |thr: f64| (t - dt) + (thr - v[i]) / (vnew[i] - v[i]).max(1e-12) * dt;
                if t_lo[i].is_infinite() && vnew[i] >= v_lo && v[i] < v_lo {
                    t_lo[i] = cross(v_lo);
                }
                if t_mid[i].is_infinite() && vnew[i] >= v_mid && v[i] < v_mid {
                    t_mid[i] = cross(v_mid);
                }
                if t_hi[i].is_infinite() && vnew[i] >= v_hi && v[i] < v_hi {
                    t_hi[i] = cross(v_hi);
                }
            }
            std::mem::swap(&mut v, &mut vnew);
        }
        let td_mid = tr * v_mid; // the forced ramp crosses v_mid here, by construction
        let mut out = BTreeMap::new();
        for (i, &n) in order.iter().enumerate() {
            let d = if t_mid[i].is_finite() { (t_mid[i] - td_mid).max(0.0) } else { 0.0 };
            // returned in the SAME convention as a Liberty slew (lower->upper), so the
            // caller can feed it straight back into an NLDM index_1 lookup
            let s = if t_hi[i].is_finite() && t_lo[i].is_finite() {
                (t_hi[i] - t_lo[i]).max(0.0)
            } else {
                0.0
            };
            out.insert(n.to_string(), (d, s));
        }
        Some(out)
    }

    /// Reduce the net to (near cap C1 fF, shielding time constant τ ns) seen from
    /// `driver`, for the effective-capacitance model. C1 = the driver node's own
    /// ground cap (sees ~0 resistance); τ = R·C2 ≈ Σ_k c_k·r_k (resistance-weighted
    /// cap, the net's first RC moment), in ns. Returns None if the net has no
    /// resistors (purely lumped — no shielding).
    pub fn pi_reduce(&self, driver: &str) -> Option<(f64, f64)> {
        if self.res.is_empty() {
            return None;
        }
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (node, c) in &self.ground {
            *cap.entry(node.as_str()).or_default() += c;
        }
        let mut adj: HashMap<&str, Vec<(&str, f64)>> = HashMap::new();
        for (a, b, r) in &self.res {
            adj.entry(a).or_default().push((b, *r));
            adj.entry(b).or_default().push((a, *r));
            cap.entry(a.as_str()).or_default();
            cap.entry(b.as_str()).or_default();
        }
        if !adj.contains_key(driver) {
            return None;
        }
        // BFS from driver, accumulating path resistance to each node
        let mut rpath: HashMap<&str, f64> = HashMap::new();
        rpath.insert(driver, 0.0);
        let mut order: Vec<&str> = vec![driver];
        let mut seen: std::collections::HashSet<&str> = std::collections::HashSet::new();
        seen.insert(driver);
        let mut head = 0;
        while head < order.len() {
            let u = order[head];
            head += 1;
            let ru = rpath[u];
            for &(v, r) in adj.get(u).map(|x| x.as_slice()).unwrap_or(&[]) {
                if seen.insert(v) {
                    rpath.insert(v, ru + r);
                    order.push(v);
                }
            }
        }
        let c1 = cap.get(driver).copied().unwrap_or(0.0); // near cap (fF)
        let m2: f64 = cap.iter().map(|(nd, c)| c * rpath.get(nd).copied().unwrap_or(0.0)).sum();
        Some((c1, m2 * 1e-6)) // (fF, ns)
    }

    /// SPEF node token for an instance pin, if present in `*CONN`.
    pub fn pin_node(&self, inst: &str, pin: &str) -> Option<&str> {
        self.pins.iter().find(|(i, p, _)| i == inst && p == pin).map(|(_, _, n)| n.as_str())
    }
}

impl Spef {
    pub fn parse(text: &str) -> Spef {
        // unit scaling -> our internal fF / Ω (default 1.0 = already fF/Ω)
        let mut c_scale = 1.0f64;
        let mut r_scale = 1.0f64;
        let cap_unit = |u: Option<&&str>| match u.map(|s| s.to_ascii_uppercase()).as_deref() {
            Some("FF") => 1.0,
            Some("PF") => 1000.0,
            Some("NF") => 1.0e6,
            _ => 1.0,
        };
        let res_unit = |u: Option<&&str>| match u.map(|s| s.to_ascii_uppercase()).as_deref() {
            Some("OHM") => 1.0,
            Some("KOHM") => 1000.0,
            Some("MOHM") => 1.0e6,
            _ => 1.0,
        };
        let mut names: BTreeMap<usize, String> = BTreeMap::new();
        let mut nets: BTreeMap<String, NetRc> = BTreeMap::new();
        let mut coupling: BTreeMap<String, f64> = BTreeMap::new();
        let mut coupling_list: BTreeMap<(String, String), f64> = BTreeMap::new();
        #[allow(clippy::type_complexity)]
        let mut coupling_nodes: BTreeMap<String, Vec<(String, String, String, f64)>> =
            BTreeMap::new();
        let mut cur: Option<(String, String, NetRc)> = None; // (name, net_node_token, rc)
        let mut sect = ""; // "", "namemap", "conn", "cap", "res"
        // Node token -> owning net. A coupling entry names one node on this net and one on a
        // net defined elsewhere in the file, so the far end can only be resolved at the end.
        let mut owner: BTreeMap<String, String> = BTreeMap::new();
        // (node a, node b, fF), in file order — resolved to nets and deduped after the loop.
        let mut pending_cc: Vec<(String, String, f64)> = Vec::new();
        let mut skipped = 0usize;
        let mut triplets = false;
        let mut reduced = 0usize;
        let mut pnets = 0usize;

        // Record that `node` sits on the net whose block we are inside.
        let claim = |cur: &Option<(String, String, NetRc)>,
                     owner: &mut BTreeMap<String, String>,
                     node: &str| {
            if let Some((name, _, _)) = cur.as_ref() {
                owner.insert(node.to_string(), name.clone());
            }
        };
        let finish = |cur: &mut Option<(String, String, NetRc)>,
                      nets: &mut BTreeMap<String, NetRc>| {
            if let Some((name, _, rc)) = cur.take() {
                if !name.is_empty() {
                    nets.insert(name, rc);
                }
            }
        };
        // A SPEF identifier is either a `*NAME_MAP` index (`*17`) or a **literal name**. The
        // map is OPTIONAL in IEEE 1481 and extractors do write names directly, so resolving
        // only the mapped form does not degrade — it discards the whole file: every net ends
        // up keyed by the empty string, every `*CONN` pin is dropped, and the caller gets the
        // no-parasitics answer back with no error and no warning.
        //
        // The leading `*` is the discriminator the format itself uses. Do NOT infer it from
        // whether the body parses as a number: a literal net may legitimately be named `123`.
        let resolve = |tok: &str, names: &BTreeMap<usize, String>| -> String {
            match tok.strip_prefix('*') {
                // an index with no map entry falls back to its own text rather than vanishing
                Some(body) => body
                    .parse::<usize>()
                    .ok()
                    .and_then(|i| names.get(&i).cloned())
                    .unwrap_or_else(|| body.to_string()),
                None => unescape(tok),
            }
        };
        // resolve a pin token "iid:pin" -> (instance name, pin)
        let pin_of = |tok: &str, names: &BTreeMap<usize, String>| -> Option<(String, String)> {
            let (ids, pin) = tok.split_once(':')?;
            Some((resolve(ids, names), pin.to_string()))
        };

        for raw in text.lines() {
            // Keywords are case-insensitive in the SPEF grammar, and a `//` comment is legal
            // anywhere. A reader that assumes upper case and ignores comments does not fail on a
            // file that uses either — it returns an EMPTY design, or an entry short of a field.
            let t = strip_comment(raw).trim();
            let kw = {
                let head = t.split_whitespace().next().unwrap_or("");
                head.to_ascii_uppercase()
            };
            if kw == "*C_UNIT" {
                let p: Vec<&str> = t.split_whitespace().collect();
                c_scale = p.get(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0) * cap_unit(p.get(2));
                continue;
            }
            if kw == "*R_UNIT" {
                let p: Vec<&str> = t.split_whitespace().collect();
                r_scale = p.get(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0) * res_unit(p.get(2));
                continue;
            }
            if kw == "*NAME_MAP" {
                sect = "namemap";
                continue;
            }
            // Net-section variants we do not model. Recognised explicitly so the reader can
            // say WHICH thing it skipped: a file of reduced (pi-model) nets otherwise reads as
            // an empty design, and "no nets" does not tell you the format was the reason.
            if kw == "*R_NET" {
                finish(&mut cur, &mut nets);
                sect = "";
                reduced += 1;
                continue;
            }
            if kw == "*D_PNET" || kw == "*R_PNET" {
                finish(&mut cur, &mut nets);
                sect = "";
                pnets += 1;
                continue;
            }
            if kw == "*D_NET" {
                sect = "";
                finish(&mut cur, &mut nets);
                let toks: Vec<&str> = t.split_whitespace().collect();
                let idtok = toks.get(1).copied().unwrap_or("");
                triplets |= toks.get(2).is_some_and(|s| is_triplet(s));
                let cap = toks.get(2).and_then(|s| parse_val(s)).unwrap_or(0.0) * c_scale;
                let name = resolve(idtok, &names);
                let net_node = node_tok(idtok, &names);
                owner.insert(net_node.clone(), name.clone());
                cur = Some((
                    name,
                    net_node.clone(),
                    NetRc { cap_ff: cap, net_node, ..Default::default() },
                ));
                continue;
            }
            match kw.as_str() {
                "*CONN" => {
                    sect = "conn";
                    continue;
                }
                "*CAP" => {
                    sect = "cap";
                    continue;
                }
                "*RES" => {
                    sect = "res";
                    continue;
                }
                "*END" => {
                    sect = "";
                    finish(&mut cur, &mut nets);
                    continue;
                }
                _ => {}
            }
            let toks: Vec<&str> = t.split_whitespace().collect();
            match sect {
                "namemap" if t.starts_with('*') => {
                    if let (Some(idtok), Some(name)) = (toks.first(), toks.get(1)) {
                        if let Ok(id) = idtok.trim_start_matches('*').parse::<usize>() {
                            names.insert(id, unescape(name));
                        }
                    }
                }
                "conn"
                    if toks
                        .first()
                        .is_some_and(|t| t.eq_ignore_ascii_case("*I") || t.eq_ignore_ascii_case("*P")) =>
                {
                    if let Some(node) = toks.get(1) {
                        // `*P <port> <dir>` names a top-level port, whose node is the port
                        // itself and not `<inst>:<pin>`, so it never matched the instance form.
                        let is_port = toks
                            .first()
                            .is_some_and(|t| t.eq_ignore_ascii_case("*P"));
                        let hookup = if is_port {
                            Some((String::new(), resolve(node, &names)))
                        } else {
                            pin_of(node, &names)
                        };
                        if let Some((inst, pin)) = hookup {
                            let dir = match toks.get(2).copied() {
                                Some("O") => crate::lef::PinDir::Output,
                                Some("B") => crate::lef::PinDir::Inout,
                                Some("I") => crate::lef::PinDir::Input,
                                _ => crate::lef::PinDir::Unknown,
                            };
                            // optional `*L <cap>` load capacitance
                            let cap_ff = toks
                                .iter()
                                .position(|t| t.eq_ignore_ascii_case("*L"))
                                .and_then(|i| toks.get(i + 1))
                                .and_then(|s| parse_val(s))
                                .map(|c| c * c_scale)
                                .unwrap_or(0.0);
                            claim(&cur, &mut owner, &node_tok(node, &names));
                            if let Some((_, _, rc)) = cur.as_mut() {
                                // `pins` is instance hookup, which is what a netlist join asks
                                // for; a port has no instance to name there.
                                if !is_port {
                                    rc.pins.push((
                                        inst.clone(),
                                        pin.clone(),
                                        node_tok(node, &names),
                                    ));
                                }
                                rc.conns.push(PinConn {
                                    inst,
                                    pin,
                                    node: node_tok(node, &names),
                                    dir,
                                    cap_ff,
                                    is_port,
                                });
                            }
                        }
                    }
                }
                "res" => {
                    // `<idx> *a *b <ohm>`
                    if toks.len() < 4 || parse_val(toks[3]).is_none() {
                        skipped += 1;
                    }
                    if toks.len() >= 4 {
                        triplets |= is_triplet(toks[3]);
                        if let Some(r) = parse_val(toks[3]) {
                            let r = r * r_scale;
                            let (na, nb) = (node_tok(toks[1], &names), node_tok(toks[2], &names));
                            claim(&cur, &mut owner, &na);
                            claim(&cur, &mut owner, &nb);
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.res_ohm += r;
                                rc.res.push((na, nb, r));
                            }
                        }
                    }
                }
                "cap" => {
                    let understood = (toks.len() >= 4 && parse_val(toks[3]).is_some())
                        || (toks.len() == 3 && parse_val(toks[2]).is_some());
                    if !understood {
                        skipped += 1;
                    }
                    if toks.len() >= 4 {
                        // Two-node coupling cap `<idx> <A> <B> <ff>`. Both ends are usually
                        // NODE tokens (`*262:A`), whose owning net is only known once the
                        // whole file has been read, so these are resolved after the loop.
                        triplets |= is_triplet(toks[3]);
                        if let Some(v) = parse_val(toks[3]) {
                            // NOT claimed for the net whose block this is. The two nodes are
                            // written in the SAME order in both nets' blocks, so the first one
                            // is not reliably the local end — claiming it reassigns the other
                            // net's node to this one and the coupling then resolves to a single
                            // net and is discarded as intra-net. Ownership comes from the
                            // grounded caps and resistors, and from the fallback in `net_of`.
                            pending_cc.push((
                                node_tok(toks[1], &names),
                                node_tok(toks[2], &names),
                                v * c_scale,
                            ));
                        }
                    } else if toks.len() >= 3 {
                        // grounded cap `<idx> *node <ff>`
                        triplets |= is_triplet(toks[2]);
                        if let Some(v) = parse_val(toks[2]) {
                            let node = node_tok(toks[1], &names);
                            claim(&cur, &mut owner, &node);
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.ground.push((node, v * c_scale));
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        finish(&mut cur, &mut nets);

        // ── resolve the coupling caps ────────────────────────────────────────────────────
        //
        // ONE physical cap, up to TWO listings. A coupling cap belongs to two nets and SPEF
        // lets it appear in either block; OpenRCX writes it in BOTH, at full value, so each
        // net's block stands alone (`extSpef::writeSrcCouplingCaps` + `writeTgtCouplingCaps`).
        // Crediting every listing therefore doubles every net's crosstalk load. Deduping by
        // the NODE pair is correct for either convention: written once, it is counted once;
        // written twice, the second listing is the same cap and is dropped.
        //
        // Both ends are resolved through the node->net map built above, because an endpoint is
        // normally an instance-pin or internal node (`*262:A`), not a net name. Reading only
        // the bare-net form silently discarded most of the coupling in a real SPEF.
        let net_of = |tok: &str| -> Option<String> {
            if let Some(n) = owner.get(tok) {
                return Some(n.clone());
            }
            // FALL BACK TO WHAT THE NODE IS NAMED AFTER. Nodes read as `<owner>:<suffix>`, so a
            // node the far net never mentions anywhere else — the only place it appears is this
            // coupling entry — still says which net it belongs to. Requiring an exact match in
            // the ownership map dropped exactly those, and they are not exotic: a net whose
            // only parasitic IS the coupling capacitor has no other line to claim a node from.
            if let Some((head, _)) = tok.split_once(':') {
                if nets.contains_key(head) || owner.values().any(|n| n == head) {
                    return Some(head.to_string());
                }
            }
            // A bare identifier that named no node is the net itself.
            (!tok.contains(':')).then(|| resolve(tok, &names))
        };
        let mut seen: std::collections::BTreeSet<(String, String)> = Default::default();
        for (a, b, v) in pending_cc {
            let key = if a <= b { (a.clone(), b.clone()) } else { (b.clone(), a.clone()) };
            if !seen.insert(key) {
                continue; // the same cap, listed again in the other net's block
            }
            let (Some(na), Some(nb)) = (net_of(&a), net_of(&b)) else {
                continue;
            };
            if na == nb {
                continue; // intra-net cap, not crosstalk
            }
            *coupling.entry(na.clone()).or_default() += v;
            *coupling.entry(nb.clone()).or_default() += v;
            // One entry per AGGRESSOR NET, not per node pair: two nets routed alongside each
            // other for a while couple at several distinct node pairs, and a window-aware SI
            // pass wants one Cc per aggressor to switch against.
            *coupling_list.entry((na.clone(), nb.clone())).or_default() += v;
            *coupling_list.entry((nb.clone(), na.clone())).or_default() += v;
            coupling_nodes.entry(na.clone()).or_default().push((nb.clone(), a.clone(), b.clone(), v));
            coupling_nodes.entry(nb).or_default().push((na, b, a, v));
        }

        for (name, rc) in nets.iter_mut() {
            rc.coupling_ff = coupling.get(name).copied().unwrap_or(0.0);
            rc.coupling_nodes = coupling_nodes.remove(name).unwrap_or_default();
            // A CANONICAL ORDER, so the writer is a fixed point on its own output. These arrive
            // in file order, and a file we wrote lists them in ours — so without this, writing
            // what we just read produces the same parasitics in a different order, and every
            // further pass through a flow shuffles the file again.
            rc.coupling_nodes.sort_by(|a, b| (&a.0, &a.1, &a.2).cmp(&(&b.0, &b.1, &b.2)));
            rc.coupling = coupling_list
                .range((name.clone(), String::new())..)
                .take_while(|((n, _), _)| n == name)
                .map(|((_, other), v)| (other.clone(), *v))
                .collect();
        }
        Spef { nets, skipped, triplets, reduced, pnets }
    }

    pub fn load(path: &str) -> Result<Spef, SpefError> {
        let text = std::fs::read_to_string(path).map_err(|e| SpefError(format!("{path}: {e}")))?;
        Ok(Spef::parse(&text))
    }

    /// Extra driver load from wire capacitance, in pF (SPEF cap is fF).
    pub fn wire_load_pf(&self, net: &str) -> f64 {
        self.nets.get(net).map(|rc| rc.cap_ff / 1000.0).unwrap_or(0.0)
    }

    /// Lumped Elmore interconnect delay for a net, in ns (R[Ω]·C[fF] → ns).
    pub fn net_delay_ns(&self, net: &str) -> f64 {
        self.nets.get(net).map(|rc| rc.res_ohm * rc.cap_ff * 1e-6).unwrap_or(0.0)
    }

    /// Serialize to standard SPEF text (IEEE-1481, fF / Ω / PS units). The output
    /// is name-mapped and round-trips through [`Spef::parse`] at the semantic level
    /// (net names, caps, resistances, pin hookup, coupling). Deterministic: no
    /// timestamp unless `opts.date` is set.
    ///
    /// Node identity: net nodes and instance pins are emitted as name-map indices
    /// (`*<id>` / `*<id>:<pin>`); any other node string is name-mapped verbatim.
    /// Grounded cap / resistor node strings equal to a net name resolve to that
    /// net's node, so a star network built as `(net_name → pin)` writes cleanly.
    pub fn to_spef(&self, opts: &WriteOpts) -> String {
        // Name map: nets first (so a net's node index is stable), then instances,
        // then any leftover named nodes appearing in the RC network.
        let mut id_of: BTreeMap<String, usize> = BTreeMap::new();
        let mut order: Vec<String> = Vec::new();
        let intern = |name: &str, id_of: &mut BTreeMap<String, usize>, order: &mut Vec<String>| {
            if !id_of.contains_key(name) {
                order.push(name.to_string());
                id_of.insert(name.to_string(), order.len());
            }
        };
        for net in self.nets.keys() {
            intern(net, &mut id_of, &mut order);
        }
        // instances (from *CONN pins), deterministic order
        let mut insts: Vec<String> = Vec::new();
        for rc in self.nets.values() {
            for (inst, _, _) in &rc.pins {
                if !insts.contains(inst) {
                    insts.push(inst.clone());
                }
            }
            for c in &rc.conns {
                // A PORT entry has no instance. Interning its empty name put a nameless entry
                // in the name map (`*51 ` with nothing after it), which is a syntax error to
                // the next reader — the whole file, rejected, for one blank line.
                if !c.is_port && !c.inst.is_empty() && !insts.contains(&c.inst) {
                    insts.push(c.inst.clone());
                }
            }
        }
        insts.sort();
        for inst in &insts {
            intern(inst, &mut id_of, &mut order);
        }

        // Resolve an RC-network node string to a SPEF node token.
        let node_tok = |s: &str,
                        owner: &str,
                        owner_is_port: bool,
                        id_of: &mut BTreeMap<String, usize>,
                        order: &mut Vec<String>|
         -> String {
            // THE OBJECT'S OWN NODE STILL NEEDS A NODE NUMBER. `*<id>` on its own is a PORT
            // reference in SPEF, not "the node of net <id>", so a bare index for an internal
            // net names something the design does not have. OpenSTA reads it, cannot find the
            // port, drops the capacitor with a warning, and reports a much faster path than the
            // one the file describes — on the three-inverter example, arrival 1.77 ns against
            // the 7.25 ns the same parasitics give when they are attached.
            // The net's OWN node, and only that. `*<id>` alone is a PORT reference in SPEF,
            // so an internal net written that way names something the design does not have —
            // OpenSTA drops the capacitor and reports a faster path than the file describes.
            // A node named after anything ELSE (a port, another net) is left bare, because for
            // a port the bare form is the correct one and adding a node number would invent a
            // node on it.
            if s == owner {
                if let Some(id) = id_of.get(s) {
                    // ON A NET THAT REACHES A TOP-LEVEL PORT, the node named after the net IS
                    // the port — and a bare `*<id>` is exactly how SPEF says "this port". Adding
                    // a node number there invents an internal node instead, which disconnects
                    // the boundary: the grounded capacitance and the resistor that used to sit
                    // on the port now sit on a node nothing else reaches. OpenSTA reported
                    // 6.8 ps at `rst_n (in)` where the same file gives 9.1 ps, and a clock
                    // network 26 ps faster than it is.
                    return if owner_is_port {
                        format!("*{id}")
                    } else {
                        format!("*{id}:0")
                    };
                }
            } else if let Some(id) = id_of.get(s) {
                return format!("*{id}");
            }
            // `<name>:<suffix>` is an instance pin OR a net-internal node, and BOTH are written
            // the same way. Only instances were recognised here, so every `n1:1` fell through
            // to the fallback below and was interned in the name map as a name with an escaped
            // colon — a node no reader can join to the net it belongs to. A real extraction is
            // almost entirely these.
            if let Some((pre, suf)) = s.split_once(':') {
                if let Some(id) = id_of.get(pre) {
                    return format!("*{id}:{suf}");
                }
            }
            // opaque internal node — intern it and emit its index
            order.push(s.to_string());
            let id = order.len();
            id_of.insert(s.to_string(), id);
            format!("*{id}")
        };

        // Body first (it grows the name map with internal nodes), header after.
        let mut body = String::new();
        for (net, rc) in &self.nets {
            let nid = id_of[net];
            // Does this net reach a top-level port of the same name?
            let net_is_port = rc.conns.iter().any(|c| c.is_port && &c.pin == net);
            body.push_str(&format!("\n*D_NET *{nid} {}\n", fmtf(rc.cap_ff)));
            if !rc.conns.is_empty() {
                body.push_str("*CONN\n");
                for c in &rc.conns {
                    let d = match c.dir {
                        crate::lef::PinDir::Output => "O",
                        crate::lef::PinDir::Inout => "B",
                        _ => "I", // input / unknown → load
                    };
                    // A port entry names the port, not an instance pin.
                    if c.is_port {
                        intern(&c.pin, &mut id_of, &mut order);
                        let pid = id_of[&c.pin];
                        body.push_str(&format!("*P *{pid} {d}\n"));
                        continue;
                    }
                    let iid = id_of[&c.inst];
                    if c.cap_ff > 0.0 {
                        body.push_str(&format!("*I *{iid}:{} {d} *L {}\n", c.pin, fmtf(c.cap_ff)));
                    } else {
                        body.push_str(&format!("*I *{iid}:{} {d}\n", c.pin));
                    }
                }
            } else if !rc.pins.is_empty() {
                body.push_str("*CONN\n");
                for (inst, pin, _) in &rc.pins {
                    let iid = id_of[inst];
                    body.push_str(&format!("*I *{iid}:{pin} I\n"));
                }
            }
            // *CAP: grounded entries then coupling entries (each coupling once).
            let mut cap_lines: Vec<String> = Vec::new();
            if rc.ground.is_empty() {
                let grounded = (rc.cap_ff - rc.coupling_ff).max(0.0);
                if grounded > 0.0 {
                    // `:0` for the same reason as everywhere else here: `*<id>` alone is a PORT
                    // reference, so a lumped cap written that way attaches to nothing.
                    cap_lines.push(format!("*{nid}:0 {}", fmtf(grounded)));
                }
            } else {
                for (node, c) in &rc.ground {
                    let tok = node_tok(node, net, net_is_port, &mut id_of, &mut order);
                    cap_lines.push(format!("{tok} {}", fmtf(*c)));
                }
            }
            // EMITTED IN BOTH NETS' BLOCKS, which is what OpenRCX does and is not redundancy:
            // a reader applies a coupling capacitor to the net whose block it appears in, so a
            // cap listed once loads one of the two nets and leaves the other believing it is
            // not coupled to anything. Writing each one once put the whole design 3 ps fast.
            // The READER still dedupes by node pair — counting both listings would double every
            // net's crosstalk — so this round-trips.
            for (other, near, far, cc) in &rc.coupling_nodes {
                {
                    let far_is_port = self
                        .nets
                        .get(other)
                        .is_some_and(|o| o.conns.iter().any(|c| c.is_port && &c.pin == other));
                    let ta = node_tok(near, net, net_is_port, &mut id_of, &mut order);
                    let tb = node_tok(far, other, far_is_port, &mut id_of, &mut order);
                    cap_lines.push(format!("{ta} {tb} {}", fmtf(*cc)));
                }
            }
            for (other, cc) in rc.coupling.iter().filter(|_| rc.coupling_nodes.is_empty()) {
                {
                    // emit under the lexicographically-smaller net only (dedupe)
                    //
                    // ATTACHED TO A NODE THE NET ACTUALLY HAS. The reader keeps one coupling
                    // figure per aggressor NET, not per node pair, so the original node is not
                    // recoverable — but writing it on a node number nothing else references
                    // leaves the capacitor hanging off a stub, loading nothing. A timer then
                    // reads the file, finds the coupling attached to an isolated node, and
                    // reports a faster path than the parasitics describe: on the three-inverter
                    // example, -5.04 ns of slack against the -6.45 ns the same file gives when
                    // the capacitor sits on the wire.
                    let near = rep_node(rc);
                    let far = self.nets.get(other).map(rep_node).unwrap_or_else(|| "0".into());
                    let far_is_port = self
                        .nets
                        .get(other)
                        .is_some_and(|o| o.conns.iter().any(|c| c.is_port && &c.pin == other));
                    if id_of.contains_key(other) {
                        let ta = node_tok(&near, net, net_is_port, &mut id_of, &mut order);
                        let tb = node_tok(&far, other, far_is_port, &mut id_of, &mut order);
                        cap_lines.push(format!("{ta} {tb} {}", fmtf(*cc)));
                    }
                }
            }
            if !cap_lines.is_empty() {
                body.push_str("*CAP\n");
                for (i, line) in cap_lines.iter().enumerate() {
                    body.push_str(&format!("{} {line}\n", i + 1));
                }
            }
            // *RES
            if !rc.res.is_empty() {
                body.push_str("*RES\n");
                for (i, (a, b, r)) in rc.res.iter().enumerate() {
                    let ta = node_tok(a, net, net_is_port, &mut id_of, &mut order);
                    let tb = node_tok(b, net, net_is_port, &mut id_of, &mut order);
                    body.push_str(&format!("{} {ta} {tb} {}\n", i + 1, fmtf(*r)));
                }
            }
            body.push_str("*END\n");
        }

        let mut out = String::new();
        out.push_str("*SPEF \"IEEE 1481-1999\"\n");
        out.push_str(&format!("*DESIGN \"{}\"\n", opts.design));
        // Always emitted: `*DATE` is REQUIRED by the SPEF grammar OpenSTA implements, so a file
        // without it is a syntax error at that line to OpenROAD, LibreLane and anything built on
        // them. It was optional here to keep output byte-reproducible; a FIXED default achieves
        // that without dropping a required field. Measured — the sibling writer in vyges-extract
        // had the same hole and its output was rejected outright.
        out.push_str(&format!(
            "*DATE \"{}\"\n",
            opts.date.as_deref().unwrap_or("00:00:00 Thursday January 01, 1970")
        ));
        out.push_str("*VENDOR \"Vyges\"\n");
        out.push_str(&format!("*PROGRAM \"{}\"\n", opts.program));
        out.push_str(&format!("*VERSION \"{}\"\n", opts.version));
        // States what the file carries: names local to the design, wire capacitance only.
        out.push_str("*DESIGN_FLOW \"NAME_SCOPE LOCAL\" \"PIN_CAP NONE\"\n");
        out.push_str("*DIVIDER /\n*DELIMITER :\n*BUS_DELIMITER [ ]\n");
        out.push_str("*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n*L_UNIT 1 HENRY\n");
        out.push_str("\n*NAME_MAP\n");
        for (i, name) in order.iter().enumerate() {
            // Escaped on the way out, unescaped on the way in — a hierarchical or bussed name
            // written raw is a syntax error to a conforming reader, and one that survives with
            // its backslashes silently matches no net in the design.
            out.push_str(&format!("*{} {}\n", i + 1, escape(name)));
        }
        out.push_str(&body);
        out
    }

    /// Re-key nets to the names the NETLIST uses, and say how many moved.
    ///
    /// ⛔ **The two files can spell the same net differently, and nothing complains.** The Verilog
    /// reader reports a net tied to a port by `assign port = net;` under the PORT's name, because a
    /// DEF, an SDC and most SPEFs use it. OpenROAD's SPEF for a routed sky130 block does not: it names
    /// those nets by the local wire (`net2007`, never `tl_o[2]`). Every one of them was then looked up,
    /// missed, and timed as **ideal wire** — 53 nets and 147 coupling references on that block, with no
    /// symptom other than optimistic slack.
    ///
    /// Applying the reader's own renaming to the file is the join. ⚠️ A key is moved only when the
    /// destination is free: if a file genuinely carries both spellings they are different entries and
    /// merging them would invent parasitics.
    ///
    /// Coupling aggressors are re-keyed too — they are looked up by name in exactly the same way.
    pub fn rename_to_design(&mut self, nl: &crate::netlist::Netlist) -> usize {
        if nl.canonical.is_empty() {
            return 0;
        }
        let mut moved = 0usize;
        let movable: Vec<(String, String)> = self
            .nets
            .keys()
            .filter_map(|k| nl.canonical.get(k).map(|c| (k.clone(), c.clone())))
            .filter(|(_, to)| !self.nets.contains_key(to))
            .collect();
        for (from, to) in movable {
            if let Some(rc) = self.nets.remove(&from) {
                self.nets.insert(to, rc);
                moved += 1;
            }
        }
        for rc in self.nets.values_mut() {
            for (agg, _) in rc.coupling.iter_mut() {
                if let Some(c) = nl.canonical.get(agg.as_str()) {
                    *agg = c.clone();
                }
            }
        }
        moved
    }
}

/// A node string this net demonstrably has: the first node its own network references, or the
/// net itself when it has no network at all (a lumped net, whose single node is its own).
fn rep_node(rc: &NetRc) -> String {
    if let Some((n, _)) = rc.ground.first() {
        return n.clone();
    }
    if let Some((a, _, _)) = rc.res.first() {
        return a.clone();
    }
    if let Some((_, _, n)) = rc.pins.first() {
        return n.clone();
    }
    rc.net_node.clone()
}

/// Compact float for SPEF numbers: the shortest decimal that reads back as the same value.
///
/// **Not a fixed number of decimal PLACES.** `{:.6}` is six places, not six significant
/// figures, so it rounded a 0.0416254 fF capacitor to 0.041625 — and any value below 5e-7
/// straight to `0.000000`, which trims to `0` and deletes the capacitor outright. Small
/// coupling capacitors are exactly where that lands, and the file still parses afterwards.
///
/// Rust's `Display` for `f64` gives the shortest representation that round-trips and never uses
/// an exponent, so the output stays plain decimal for any real parasitic. The exponent fallback
/// is for a value no extractor produces (below ~1e-16), where the decimal form would run to
/// hundreds of digits; SPEF readers take it, and nothing physical reaches it.
fn fmtf(v: f64) -> String {
    if v == 0.0 {
        return "0".into();
    }
    let s = format!("{v}");
    if s.len() > 24 {
        return format!("{v:e}");
    }
    s
}

#[cfg(test)]
mod writer_tests {
    use super::*;

    fn sample() -> Spef {
        let mut nets = BTreeMap::new();
        nets.insert(
            "neta".to_string(),
            NetRc {
                cap_ff: 10.0,
                res_ohm: 100.0,
                coupling_ff: 2.0,
                coupling: vec![("netb".to_string(), 2.0)],
                net_node: "neta".to_string(),
                ground: vec![("neta".to_string(), 8.0)],
                res: vec![("neta".to_string(), "u1:A".to_string(), 100.0)],
                pins: vec![("u1".to_string(), "A".to_string(), "u1:A".to_string())],
                conns: vec![],
                coupling_nodes: vec![],
            },
        );
        nets.insert(
            "netb".to_string(),
            NetRc {
                cap_ff: 7.0,
                res_ohm: 50.0,
                coupling_ff: 2.0,
                coupling: vec![("neta".to_string(), 2.0)],
                net_node: "netb".to_string(),
                ground: vec![("netb".to_string(), 5.0)],
                res: vec![("netb".to_string(), "u2:Y".to_string(), 50.0)],
                pins: vec![("u2".to_string(), "Y".to_string(), "u2:Y".to_string())],
                conns: vec![],
                coupling_nodes: vec![],
            },
        );
        Spef { nets, ..Default::default() }
    }

    #[test]
    fn roundtrip_semantic() {
        let spef = sample();
        let text = spef.to_spef(&WriteOpts { design: "blk".into(), ..Default::default() });
        // sanity: header + name map present, no wall-clock date
        assert!(text.contains("*SPEF \"IEEE 1481-1999\""));
        assert!(text.contains("*DESIGN \"blk\""));
        assert!(text.contains("*DATE"), "required by the grammar OpenSTA implements");
        assert!(text.contains("*NAME_MAP"));

        let back = Spef::parse(&text);
        assert_eq!(back.nets.len(), 2);
        let a = back.nets.get("neta").expect("neta round-trips");
        assert_eq!(a.cap_ff, 10.0);
        assert_eq!(a.res_ohm, 100.0);
        assert!(a.pins.iter().any(|(i, p, _)| i == "u1" && p == "A"));
        // coupling emitted once, applied to both nets
        assert_eq!(a.coupling_ff, 2.0);
        assert_eq!(back.nets.get("netb").unwrap().coupling_ff, 2.0);
        assert_eq!(back.nets.get("netb").unwrap().res_ohm, 50.0);
    }

    /// A coupling cap belongs to two nets, and SPEF lets it appear in either block. OpenRCX
    /// writes it in BOTH, at full value, so each net's block stands alone. Crediting every
    /// listing doubles the crosstalk load on every net in a foreign SPEF — count each cap once.
    #[test]
    fn a_coupling_cap_listed_in_both_blocks_is_counted_once() {
        let text = "\
*SPEF \"IEEE 1481-1999\"
*DESIGN \"blk\"
*DATE \"x\"
*DIVIDER /
*DELIMITER :
*BUS_DELIMITER []
*T_UNIT 1 NS
*C_UNIT 1 FF
*R_UNIT 1 OHM
*L_UNIT 1 HENRY

*NAME_MAP
*1 neta
*2 netb

*D_NET *1 10
*CAP
1 *1 8
2 *1 *2 2
*END

*D_NET *2 7
*CAP
1 *2 5
2 *1 *2 2
*END
";
        let s = Spef::parse(text);
        assert_eq!(s.nets.get("neta").unwrap().coupling_ff, 2.0, "not 4.0");
        assert_eq!(s.nets.get("netb").unwrap().coupling_ff, 2.0, "not 4.0");
        // and the aggressor list names netb once, not twice
        assert_eq!(s.nets.get("neta").unwrap().coupling, vec![("netb".to_string(), 2.0)]);
    }

    /// The other convention: each cap listed ONCE, in one net's block. Both are legal and both
    /// come out of OpenRCX — its `couplingFlow` output lists every cap twice, its pattern-
    /// extraction output lists each once. A reader that "corrects" by halving is right on one
    /// and wrong on the other; deduping by node pair is right on both, which is why the two
    /// cases below must agree.
    #[test]
    fn a_coupling_cap_listed_once_is_read_the_same_way() {
        let head = "\
*SPEF \"IEEE 1481-1999\"
*DESIGN \"blk\"
*DATE \"x\"
*DIVIDER /
*DELIMITER :
*BUS_DELIMITER []
*T_UNIT 1 NS
*C_UNIT 1 FF
*R_UNIT 1 OHM
*L_UNIT 1 HENRY

*NAME_MAP
*1 neta
*2 netb
*3 u1
*4 u2

*D_NET *1 10
*CONN
*I *3:A I
*CAP
1 *3:A 8
2 *3:A *4:Y 2
*END

*D_NET *2 7
*CONN
*I *4:Y O
*CAP
1 *4:Y 5
";
        let once = Spef::parse(&format!("{head}*END\n"));
        let twice = Spef::parse(&format!("{head}2 *3:A *4:Y 2\n*END\n"));
        for s in [&once, &twice] {
            assert_eq!(s.nets.get("neta").unwrap().coupling_ff, 2.0);
            assert_eq!(s.nets.get("netb").unwrap().coupling_ff, 2.0);
        }
    }

    /// Coupling endpoints are normally NODE tokens — an instance pin or an internal node,
    /// whose owning net is only known from the `*CONN` / `*RES` entries elsewhere in the file.
    /// Reading only the bare-net form discards most of the coupling in a real SPEF.
    /// **Crosstalk must actually reach the delay** on a net named the way a real extractor
    /// names one.
    ///
    /// The capacitance used to be deposited at `net_node` — the net's own name — which is a node
    /// of the RC network only when the file uses the bare `*<netid>` form. OpenRCX writes
    /// `*<netid>:<n>` and `*<instid>:<pin>`, so on a routed sky130 block `net_node` matched a
    /// network node for 221 of 14238 nets: for the other 98 % every femtofarad of coupling
    /// landed on a node the tree never visits. Nothing failed. The Miller factor could be set to
    /// any value at all and the answer did not move — an SI term that was structurally incapable
    /// of doing anything.
    ///
    /// This fixture is deliberately spelled the OpenRCX way, because the bare-node spelling is
    /// exactly the one that hid the defect.
    #[test]
    fn crosstalk_changes_the_delay_of_a_net_named_the_way_extractors_name_them() {
        let text = "*SPEF \"IEEE 1481-1998\"\n*DESIGN \"blk\"\n\
                    *T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n\
                    *NAME_MAP\n*1 victim\n*2 drv\n*3 snk\n\n\
                    *D_NET *1 10\n*CONN\n*I *2:Y O\n*I *3:A I\n\
                    *CAP\n1 *1:1 5\n2 *3:A 5\n\
                    *RES\n1 *2:Y *1:1 100\n2 *1:1 *3:A 100\n*END\n";
        let s = Spef::parse(text);
        let rc = s.nets.get("victim").expect("victim");
        // the precondition that makes this test meaningful
        let nodes: Vec<&str> = rc.res.iter().flat_map(|(a, b, _)| [a.as_str(), b.as_str()]).collect();
        assert!(
            !nodes.contains(&rc.net_node.as_str()),
            "the fixture must name its nodes the way an extractor does, not as the bare net"
        );

        let th = crate::liberty::Thresholds::default();
        let worst = |xc: f64| {
            rc.transient("drv:Y", 0.15, xc, th)
                .expect("a tree rooted at the driver")
                .values()
                .map(|(d, _)| *d)
                .fold(0.0f64, f64::max)
        };
        let (quiet, coupled) = (worst(0.0), worst(50.0));
        assert!(
            coupled > quiet * 1.05,
            "50 fF of crosstalk must slow the net: {quiet:.6} ns -> {coupled:.6} ns"
        );
        // and the same through the Elmore path, which shares the defect and the fix
        let e = |xc: f64| {
            rc.elmore("drv:Y", xc, &BTreeMap::new()).expect("elmore").values().copied().fold(0.0f64, f64::max)
        };
        assert!(e(50.0) > e(0.0) * 1.05, "and through Elmore: {} -> {}", e(0.0), e(50.0));
    }

    #[test]
    fn coupling_between_node_tokens_resolves_to_their_nets() {
        let text = "\
*SPEF \"IEEE 1481-1999\"
*DESIGN \"blk\"
*DATE \"x\"
*DIVIDER /
*DELIMITER :
*BUS_DELIMITER []
*T_UNIT 1 NS
*C_UNIT 1 FF
*R_UNIT 1 OHM
*L_UNIT 1 HENRY

*NAME_MAP
*1 neta
*2 netb
*3 u1
*4 u2

*D_NET *1 10
*CONN
*I *3:A I
*CAP
1 *3:A 8
2 *3:A *4:Y 3
*END

*D_NET *2 7
*CONN
*I *4:Y O
*CAP
1 *4:Y 5
2 *3:A *4:Y 3
*END
";
        let s = Spef::parse(text);
        assert_eq!(s.nets.get("neta").unwrap().coupling_ff, 3.0);
        assert_eq!(s.nets.get("netb").unwrap().coupling_ff, 3.0);
        assert_eq!(s.nets.get("netb").unwrap().coupling, vec![("neta".to_string(), 3.0)]);
    }

    /// Two nodes on the SAME net are an intra-net cap, not crosstalk.
    #[test]
    fn a_cap_between_two_nodes_of_one_net_is_not_coupling() {
        let text = "\
*SPEF \"IEEE 1481-1999\"
*DESIGN \"blk\"
*DATE \"x\"
*DIVIDER /
*DELIMITER :
*BUS_DELIMITER []
*T_UNIT 1 NS
*C_UNIT 1 FF
*R_UNIT 1 OHM
*L_UNIT 1 HENRY

*NAME_MAP
*1 neta
*3 u1

*D_NET *1 10
*CONN
*I *3:A I
*RES
1 *1 *3:A 100
*CAP
1 *1 8
2 *1 *3:A 2
*END
";
        let s = Spef::parse(text);
        assert_eq!(s.nets.get("neta").unwrap().coupling_ff, 0.0);
    }

    #[test]
    fn conn_direction_and_cin_roundtrip() {
        use crate::lef::PinDir;
        let mut nets = BTreeMap::new();
        nets.insert(
            "n0".to_string(),
            NetRc {
                cap_ff: 5.0,
                net_node: "n0".to_string(),
                ground: vec![("n0".to_string(), 5.0)],
                res: vec![("n0".to_string(), "u1:Y".to_string(), 20.0)],
                conns: vec![
                    PinConn { inst: "u1".into(), pin: "Y".into(), node: "u1:Y".into(), dir: PinDir::Output, cap_ff: 0.0, is_port: false },
                    PinConn { inst: "u2".into(), pin: "A".into(), node: "u2:A".into(), dir: PinDir::Input, cap_ff: 1.5, is_port: false },
                ],
                ..Default::default()
            },
        );
        let text = Spef { nets, ..Default::default() }.to_spef(&WriteOpts::default());
        assert!(text.contains(" O\n") || text.contains(" O *L")); // driver marked O
        assert!(text.contains("*L 1.5")); // load cap emitted
        let back = Spef::parse(&text);
        let c = &back.nets.get("n0").unwrap().conns;
        let y = c.iter().find(|c| c.pin == "Y").expect("Y pin");
        assert_eq!(y.dir, PinDir::Output);
        let a = c.iter().find(|c| c.pin == "A").expect("A pin");
        assert_eq!(a.dir, PinDir::Input);
        assert_eq!(a.cap_ff, 1.5);
    }

    #[test]
    fn deterministic_and_dated() {
        let spef = sample();
        let o = WriteOpts { date: Some("2026-07-13T00:00:00Z".into()), ..Default::default() };
        assert_eq!(spef.to_spef(&o), spef.to_spef(&o)); // stable
        assert!(spef.to_spef(&o).contains("*DATE \"2026-07-13T00:00:00Z\""));
    }
}

#[cfg(test)]
mod name_map_optional_tests {
    use super::*;

    // `*NAME_MAP` is OPTIONAL in IEEE 1481. These are the SAME net written the two ways the
    // standard allows — through a map index, and with literal names. Extractors emit both;
    // resolving only the mapped form did not degrade the answer, it discarded the file.
    const MAPPED: &str = r#"*SPEF "IEEE 1481-1998"
*DESIGN "t"
*DIVIDER /
*DELIMITER :
*T_UNIT 1 PS
*C_UNIT 1 FF
*R_UNIT 1 OHM

*NAME_MAP
*1 sig
*2 u_drv
*3 u_snk

*D_NET *1 12.5
*CONN
*I *2:Y O
*I *3:A I
*CAP
1 *1:0 5.0
2 *1:1 7.5
*RES
1 *2:Y *1:0 40.0
2 *1:0 *1:1 10.0
3 *1:1 *3:A 30.0
*END
"#;

    const LITERAL: &str = r#"*SPEF "IEEE 1481-1998"
*DESIGN "t"
*DIVIDER /
*DELIMITER :
*T_UNIT 1 PS
*C_UNIT 1 FF
*R_UNIT 1 OHM

*D_NET sig 12.5
*CONN
*I u_drv:Y O
*I u_snk:A I
*CAP
1 sig:0 5.0
2 sig:1 7.5
*RES
1 u_drv:Y sig:0 40.0
2 sig:0 sig:1 10.0
3 sig:1 u_snk:A 30.0
*END
"#;

    #[test]
    fn a_literal_named_spef_does_not_parse_to_nothing() {
        // The failure this guards is silent: the file streams through, every net lands under
        // the empty name, and the caller gets the no-parasitics answer with no error at all.
        let s = Spef::parse(LITERAL);
        assert_eq!(
            s.nets.len(),
            1,
            "a SPEF without a *NAME_MAP must still yield its nets"
        );
        assert!(
            s.nets.contains_key("sig"),
            "and must be keyed by the net's own name"
        );
        let rc = &s.nets["sig"];
        assert!(
            !rc.pins.is_empty(),
            "*CONN pins must survive a literal instance name"
        );
        assert!(
            !rc.ground.is_empty(),
            "*CAP entries must survive an unprefixed node token"
        );
    }

    #[test]
    fn both_forms_give_the_timer_the_same_delay() {
        // The node TOKENS differ between the forms by construction (`1:0` vs `sig:0`), so
        // comparing them would be comparing labels. Compare the physics instead, through the
        // same public path the timer uses: driver pin -> Elmore -> sink pin.
        let (m, l) = (Spef::parse(MAPPED), Spef::parse(LITERAL));
        let delay = |s: &Spef| -> f64 {
            let rc = s.nets.get("sig").expect("net present in both forms");
            let drv = rc.pin_node("u_drv", "Y").expect("driver pin resolves");
            let snk = rc.pin_node("u_snk", "A").expect("sink pin resolves");
            *rc.elmore(drv, 0.0, &BTreeMap::new())
                .expect("RC is a tree")
                .get(snk)
                .expect("sink is reachable")
        };
        let (dm, dl) = (delay(&m), delay(&l));
        assert!(
            dm > 0.0,
            "the fixture must produce a real delay, else this proves nothing"
        );
        assert!(
            (dm - dl).abs() < 1e-12,
            "the two spellings of one net must time identically: mapped {dm}, literal {dl}"
        );

        // and the lumped values the caller reads directly
        assert_eq!(m.nets["sig"].cap_ff, l.nets["sig"].cap_ff);
        assert_eq!(m.nets["sig"].res_ohm, l.nets["sig"].res_ohm);
        assert_eq!(m.wire_load_pf("sig"), l.wire_load_pf("sig"));
    }
}

#[cfg(test)]
mod rename_tests {
    use super::*;

    // ⛔ The defect this exists for: the netlist reader reports `assign tl_o[2] = net2007;` as
    // `tl_o[2]`, and OpenROAD's SPEF for the same block calls that net `net2007`. Without the
    // rename the parasitics join to nothing and the net is timed as ideal wire.
    const NL: &str = "module m (tl_o);\n output [3:0] tl_o;\n wire net2007;\n \
                      CELL u1 (.LO(net2007));\n assign tl_o[2] = net2007;\nendmodule\n";

    fn spef_with(net: &str, agg: &str) -> Spef {
        Spef::parse(&format!(
            "*SPEF \"ieee 1481-1999\"\n*DESIGN \"m\"\n*DIVIDER /\n*DELIMITER :\n\
             *BUS_DELIMITER []\n*T_UNIT 1 NS\n*C_UNIT 1 PF\n*R_UNIT 1 OHM\n*L_UNIT 1 HENRY\n\n\
             *NAME_MAP\n*1 {net}\n*2 {agg}\n\n*D_NET *1 0.5\n*CAP\n1 *1 *2 0.25\n*END\n\
             *D_NET *2 0.5\n*END\n"
        ))
    }

    #[test]
    fn a_net_the_reader_renamed_is_re_keyed_to_the_name_the_netlist_uses() {
        let nl = crate::netlist::parse(NL).expect("parses");
        assert_eq!(nl.canonical.get("net2007").map(String::as_str), Some("tl_o[2]"));

        let mut sp = spef_with("net2007", "other");
        assert!(sp.nets.contains_key("net2007") && !sp.nets.contains_key("tl_o[2]"));
        assert_eq!(sp.rename_to_design(&nl), 1);
        assert!(sp.nets.contains_key("tl_o[2]"), "the net now joins");
        assert!(!sp.nets.contains_key("net2007"), "and the old key is gone, not duplicated");
    }

    // A coupling aggressor is looked up by name in exactly the same way, so it moves too.
    #[test]
    fn a_coupling_aggressor_is_re_keyed_as_well() {
        let nl = crate::netlist::parse(NL).expect("parses");
        let mut sp = spef_with("other", "net2007");
        sp.rename_to_design(&nl);
        let names: Vec<&str> =
            sp.nets.values().flat_map(|rc| rc.coupling.iter().map(|(a, _)| a.as_str())).collect();
        assert!(names.contains(&"tl_o[2]"), "aggressor re-keyed, got {names:?}");
        assert!(!names.contains(&"net2007"));
    }

    // ⚠️ If the file genuinely carries BOTH spellings they are different entries; merging them
    // would invent parasitics on one and destroy them on the other.
    #[test]
    fn an_occupied_destination_is_left_alone() {
        let nl = crate::netlist::parse(NL).expect("parses");
        let mut sp = spef_with("net2007", "tl_o[2]");
        assert_eq!(sp.rename_to_design(&nl), 0, "nothing may move onto an existing key");
        assert!(sp.nets.contains_key("net2007") && sp.nets.contains_key("tl_o[2]"));
    }

    // A netlist with no aliases renames nothing and costs nothing.
    #[test]
    fn a_netlist_that_renamed_nothing_moves_nothing() {
        let nl = crate::netlist::parse("module m (a);\n input a;\n CELL u1 (.A(a));\nendmodule\n")
            .expect("parses");
        assert!(nl.canonical.is_empty());
        let mut sp = spef_with("net2007", "other");
        assert_eq!(sp.rename_to_design(&nl), 0);
        assert!(sp.nets.contains_key("net2007"));
    }
}
