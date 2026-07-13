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

use std::collections::{BTreeMap, HashMap};

#[derive(Debug, Clone, Default)]
pub struct NetRc {
    pub cap_ff: f64,                  // total cap (grounded + coupling), from *D_NET
    pub res_ohm: f64,                 // summed *RES (lumped fallback)
    pub coupling_ff: f64,             // total coupling cap (sum over neighbours)
    pub coupling: Vec<(String, f64)>, // per-aggressor coupling (net, Cc) for window-aware SI
    // RC network (for per-pin tree Elmore):
    pub net_node: String,                 // node where coupling attaches (the net node)
    pub ground: Vec<(String, f64)>,       // (node, grounded cap fF)
    pub res: Vec<(String, String, f64)>,  // (node a, node b, ohm)
    pub pins: Vec<(String, String, String)>, // (instance, pin, node) from *CONN
}

#[derive(Debug, Clone, Default)]
pub struct Spef {
    pub nets: BTreeMap<String, NetRc>,
}

/// Options for the SPEF writer ([`Spef::to_spef`]). Kept minimal and
/// deterministic — no wall-clock timestamp unless `date` is supplied, so the
/// same design extracts byte-identically (the suite's reproducibility contract).
#[derive(Debug, Clone)]
pub struct WriteOpts {
    pub design: String,  // *DESIGN "<name>"
    pub program: String, // *PROGRAM "<tool>"
    pub version: String, // *VERSION "<ver>"
    pub date: Option<String>, // *DATE "<iso>" — omitted when None
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

fn node_tok(t: &str) -> String {
    t.trim_start_matches('*').to_string()
}

impl NetRc {
    /// Per-node Elmore delays (ns) for the net's RC tree rooted at `driver`,
    /// with `xtalk_cap_ff` added at the net node (the Miller crosstalk load).
    /// Returns `None` if the network is not a tree reachable from the driver
    /// (caller falls back to the lumped delay).
    pub fn elmore(&self, driver: &str, xtalk_cap_ff: f64) -> Option<BTreeMap<String, f64>> {
        if self.res.is_empty() {
            return None;
        }
        // node capacitances
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (node, c) in &self.ground {
            *cap.entry(node.as_str()).or_default() += c;
        }
        *cap.entry(self.net_node.as_str()).or_default() += xtalk_cap_ff;
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
            let d = delay[p] + r * sub[nd] * 1e-6; // R[Ω]·C[fF] -> ns
            delay.insert(nd.to_string(), d);
        }
        Some(delay)
    }

    /// Transient node response: drive the RC tree with the driver's output edge (a
    /// saturated ramp 0→1 over `driver_slew_ns`, from t=0) as a forced source,
    /// integrate with backward Euler over the rooted tree (an O(N) up/down sweep per
    /// step), and read each node's 50% delay (relative to the driver's 50%) and
    /// 30→70% slew. `xtalk_cap_ff` adds at the net node. This is the waveform-into-RC
    /// convolution — more accurate than Elmore (a single RC gives 0.69·RC, not R·C).
    /// Returns node → (delay_ns, slew_ns), or None if not a tree from `driver`.
    pub fn transient(
        &self,
        driver: &str,
        driver_slew_ns: f64,
        xtalk_cap_ff: f64,
    ) -> Option<BTreeMap<String, (f64, f64)>> {
        if self.res.is_empty() {
            return None;
        }
        let mut cap: HashMap<&str, f64> = HashMap::new();
        for (n, c) in &self.ground {
            *cap.entry(n.as_str()).or_default() += c;
        }
        *cap.entry(self.net_node.as_str()).or_default() += xtalk_cap_ff;
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
        let tr = driver_slew_ns.max(1e-4); // ramp duration
        let nsteps = 800usize;
        let dt = ((tr + 6.0 * tau_lump) / nsteps as f64).max(1e-7);
        let vdrv = |t: f64| if t <= 0.0 { 0.0 } else if t >= tr { 1.0 } else { t / tr };

        let didx = idx[driver];
        let mut v = vec![0.0f64; nn];
        let (mut t30, mut t50, mut t70) =
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
                if t30[i].is_infinite() && vnew[i] >= 0.3 && v[i] < 0.3 {
                    t30[i] = cross(0.3);
                }
                if t50[i].is_infinite() && vnew[i] >= 0.5 && v[i] < 0.5 {
                    t50[i] = cross(0.5);
                }
                if t70[i].is_infinite() && vnew[i] >= 0.7 && v[i] < 0.7 {
                    t70[i] = cross(0.7);
                }
            }
            std::mem::swap(&mut v, &mut vnew);
        }
        let td50 = tr * 0.5; // forced ramp midpoint
        let mut out = BTreeMap::new();
        for (i, &n) in order.iter().enumerate() {
            let d = if t50[i].is_finite() { (t50[i] - td50).max(0.0) } else { 0.0 };
            let s = if t70[i].is_finite() && t30[i].is_finite() {
                (t70[i] - t30[i]).max(0.0)
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
        let mut coupling_list: BTreeMap<String, Vec<(String, f64)>> = BTreeMap::new();
        let mut cur: Option<(String, String, NetRc)> = None; // (name, net_node_token, rc)
        let mut sect = ""; // "", "namemap", "conn", "cap", "res"

        let finish = |cur: &mut Option<(String, String, NetRc)>,
                      nets: &mut BTreeMap<String, NetRc>| {
            if let Some((name, _, rc)) = cur.take() {
                if !name.is_empty() {
                    nets.insert(name, rc);
                }
            }
        };
        let netname = |tok: &str, names: &BTreeMap<usize, String>| -> Option<String> {
            let body = tok.trim_start_matches('*');
            if body.contains(':') {
                return None;
            }
            body.parse::<usize>().ok().and_then(|i| names.get(&i).cloned())
        };
        // resolve a pin token "iid:pin" -> (instance name, pin)
        let pin_of = |tok: &str, names: &BTreeMap<usize, String>| -> Option<(String, String)> {
            let body = tok.trim_start_matches('*');
            let (ids, pin) = body.split_once(':')?;
            let inst = ids.parse::<usize>().ok().and_then(|i| names.get(&i).cloned())?;
            Some((inst, pin.to_string()))
        };

        for raw in text.lines() {
            let t = raw.trim();
            if t.starts_with("*C_UNIT") {
                let p: Vec<&str> = t.split_whitespace().collect();
                c_scale = p.get(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0) * cap_unit(p.get(2));
                continue;
            }
            if t.starts_with("*R_UNIT") {
                let p: Vec<&str> = t.split_whitespace().collect();
                r_scale = p.get(1).and_then(|s| s.parse::<f64>().ok()).unwrap_or(1.0) * res_unit(p.get(2));
                continue;
            }
            if t == "*NAME_MAP" {
                sect = "namemap";
                continue;
            }
            if t.starts_with("*D_NET") {
                sect = "";
                finish(&mut cur, &mut nets);
                let toks: Vec<&str> = t.split_whitespace().collect();
                let idtok = toks.get(1).copied().unwrap_or("");
                let id = idtok.trim_start_matches('*').parse::<usize>().ok();
                let cap = toks.get(2).and_then(|s| s.parse::<f64>().ok()).unwrap_or(0.0) * c_scale;
                let name = id.and_then(|i| names.get(&i).cloned()).unwrap_or_default();
                let net_node = node_tok(idtok);
                cur = Some((
                    name,
                    net_node.clone(),
                    NetRc { cap_ff: cap, net_node, ..Default::default() },
                ));
                continue;
            }
            match t {
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
                            names.insert(id, name.to_string());
                        }
                    }
                }
                "conn" if toks.first() == Some(&"*I") => {
                    if let Some(node) = toks.get(1) {
                        if let Some((inst, pin)) = pin_of(node, &names) {
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.pins.push((inst, pin, node_tok(node)));
                            }
                        }
                    }
                }
                "res" => {
                    // `<idx> *a *b <ohm>`
                    if toks.len() >= 4 {
                        if let Ok(r) = toks[3].parse::<f64>() {
                            let r = r * r_scale;
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.res_ohm += r;
                                rc.res.push((node_tok(toks[1]), node_tok(toks[2]), r));
                            }
                        }
                    }
                }
                "cap" => {
                    if toks.len() >= 4 && toks[1].starts_with('*') && toks[2].starts_with('*') {
                        // two-node coupling cap `<idx> *A *B <ff>`
                        if let (Some(a), Some(b), Ok(v)) =
                            (netname(toks[1], &names), netname(toks[2], &names), toks[3].parse::<f64>())
                        {
                            let v = v * c_scale;
                            *coupling.entry(a.clone()).or_default() += v;
                            *coupling.entry(b.clone()).or_default() += v;
                            coupling_list.entry(a.clone()).or_default().push((b.clone(), v));
                            coupling_list.entry(b).or_default().push((a, v));
                        }
                    } else if toks.len() >= 3 && toks[1].starts_with('*') {
                        // grounded cap `<idx> *node <ff>`
                        if let Ok(v) = toks[2].parse::<f64>() {
                            if let Some((_, _, rc)) = cur.as_mut() {
                                rc.ground.push((node_tok(toks[1]), v * c_scale));
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        finish(&mut cur, &mut nets);
        for (name, rc) in nets.iter_mut() {
            rc.coupling_ff = coupling.get(name).copied().unwrap_or(0.0);
            rc.coupling = coupling_list.get(name).cloned().unwrap_or_default();
        }
        Spef { nets }
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
        }
        insts.sort();
        for inst in &insts {
            intern(inst, &mut id_of, &mut order);
        }
        let inst_set: std::collections::BTreeSet<&String> = insts.iter().collect();

        // Resolve an RC-network node string to a SPEF node token.
        let node_tok = |s: &str,
                        id_of: &mut BTreeMap<String, usize>,
                        order: &mut Vec<String>|
         -> String {
            if let Some(id) = id_of.get(s) {
                return format!("*{id}"); // net node (or already-interned node)
            }
            if let Some((pre, suf)) = s.split_once(':') {
                if inst_set.contains(&pre.to_string()) {
                    let id = id_of[pre];
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
            body.push_str(&format!("\n*D_NET *{nid} {}\n", fmtf(rc.cap_ff)));
            if !rc.pins.is_empty() {
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
                    cap_lines.push(format!("*{nid} {}", fmtf(grounded)));
                }
            } else {
                for (node, c) in &rc.ground {
                    let tok = node_tok(node, &mut id_of, &mut order);
                    cap_lines.push(format!("{tok} {}", fmtf(*c)));
                }
            }
            for (other, cc) in &rc.coupling {
                if net.as_str() < other.as_str() {
                    // emit under the lexicographically-smaller net only (dedupe)
                    if let Some(oid) = id_of.get(other).copied() {
                        cap_lines.push(format!("*{nid} *{oid} {}", fmtf(*cc)));
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
                    let ta = node_tok(a, &mut id_of, &mut order);
                    let tb = node_tok(b, &mut id_of, &mut order);
                    body.push_str(&format!("{} {ta} {tb} {}\n", i + 1, fmtf(*r)));
                }
            }
            body.push_str("*END\n");
        }

        let mut out = String::new();
        out.push_str("*SPEF \"IEEE 1481-1999\"\n");
        out.push_str(&format!("*DESIGN \"{}\"\n", opts.design));
        if let Some(d) = &opts.date {
            out.push_str(&format!("*DATE \"{d}\"\n"));
        }
        out.push_str("*VENDOR \"Vyges\"\n");
        out.push_str(&format!("*PROGRAM \"{}\"\n", opts.program));
        out.push_str(&format!("*VERSION \"{}\"\n", opts.version));
        out.push_str("*DESIGN_FLOW \"\"\n");
        out.push_str("*DIVIDER /\n*DELIMITER :\n*BUS_DELIMITER [ ]\n");
        out.push_str("*T_UNIT 1 PS\n*C_UNIT 1 FF\n*R_UNIT 1 OHM\n*L_UNIT 1 HENRY\n");
        out.push_str("\n*NAME_MAP\n");
        for (i, name) in order.iter().enumerate() {
            out.push_str(&format!("*{} {}\n", i + 1, name));
        }
        out.push_str(&body);
        out
    }
}

/// Compact fixed-ish float for SPEF numbers: integers stay integer, otherwise up
/// to 6 significant decimals with trailing zeros trimmed (stable, no exponent).
fn fmtf(v: f64) -> String {
    if v == 0.0 {
        return "0".into();
    }
    if v.fract() == 0.0 && v.abs() < 1e15 {
        return format!("{}", v as i64);
    }
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    s.to_string()
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
            },
        );
        Spef { nets }
    }

    #[test]
    fn roundtrip_semantic() {
        let spef = sample();
        let text = spef.to_spef(&WriteOpts { design: "blk".into(), ..Default::default() });
        // sanity: header + name map present, no wall-clock date
        assert!(text.contains("*SPEF \"IEEE 1481-1999\""));
        assert!(text.contains("*DESIGN \"blk\""));
        assert!(!text.contains("*DATE"));
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

    #[test]
    fn deterministic_and_dated() {
        let spef = sample();
        let o = WriteOpts { date: Some("2026-07-13T00:00:00Z".into()), ..Default::default() };
        assert_eq!(spef.to_spef(&o), spef.to_spef(&o)); // stable
        assert!(spef.to_spef(&o).contains("*DATE \"2026-07-13T00:00:00Z\""));
    }
}
