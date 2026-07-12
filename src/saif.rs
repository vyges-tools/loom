//! Minimal SAIF (Switching Activity Interchange Format) reader for **vectored**
//! activity — the cumulative-statistics counterpart to a VCD.
//!
//! Where a VCD logs every transition (size grows with sim length), SAIF stores
//! per net the time spent in 0/1/X (`T0`/`T1`/`TX`) plus a **toggle count** `TC`
//! — so its size scales with the *design*, not the run. Verilator emits it via
//! `--trace-saif` (`VerilatedSaifC`); OpenSTA reads it with `read_saif`. This
//! reader turns `TC` over the run `DURATION` into a per-net toggle rate — the
//! exact quantity [`crate::vcd::Vcd`] provides, so it feeds the same `Activity`.
//!
//! v0 scope: "backward" SAIF (the power flavour). Nets are keyed by their **full
//! `INSTANCE` path** (`counter_tb.dut.clk_in`), and a netlist net resolves to one by
//! leaf + optional `scope:` — see [`crate::names`]. If `DURATION` is absent the per-net
//! `T0+T1+TX` span is used. Depth reserved: bit-level vector nets, glitch (`IG`) power.

use crate::names::NetIndex;

#[derive(Debug, Clone, Default)]
pub struct Saif {
    pub idx: NetIndex,   // full-path TC counts + leaf index + optional design scope
    pub sim_time_s: f64, // DURATION in seconds
}

#[derive(Debug)]
pub struct SaifError(pub String);
impl std::fmt::Display for SaifError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "saif error: {}", self.0)
    }
}
impl std::error::Error for SaifError {}

impl Saif {
    /// Transitions / second for a netlist net (0 if unresolved, ambiguous, or
    /// zero-duration run). Resolution is scope-aware — see [`crate::names::NetIndex`].
    pub fn toggle_rate(&self, net: &str) -> f64 {
        match self.idx.resolve(net) {
            Some(n) if self.sim_time_s > 0.0 => n as f64 / self.sim_time_s,
            _ => 0.0,
        }
    }

    /// Set the design scope (job `scope:`) used to disambiguate leaf names.
    pub fn with_scope(mut self, scope: Option<String>) -> Self {
        self.idx.scope = scope;
        self
    }

    /// Number of leaf names declared under more than one scope.
    pub fn collisions(&self) -> usize {
        self.idx.collisions()
    }

    pub fn load(path: &str) -> Result<Saif, SaifError> {
        let text = std::fs::read_to_string(path).map_err(|e| SaifError(format!("{path}: {e}")))?;
        Saif::parse(&text)
    }

    /// Load and apply a design scope.
    pub fn load_scoped(path: &str, scope: Option<String>) -> Result<Saif, SaifError> {
        Ok(Saif::load(path)?.with_scope(scope))
    }

    pub fn parse(text: &str) -> Result<Saif, SaifError> {
        let root = Node::parse(text)?;

        let timescale_s = root
            .find_kv("TIMESCALE")
            .map(|v| parse_timescale(&v))
            .unwrap_or(1.0e-9);
        let duration_units = root
            .find_kv("DURATION")
            .and_then(|v| v.first().and_then(|s| s.parse::<f64>().ok()));

        let mut idx = NetIndex::default();
        let mut max_span_units = 0.0_f64;
        let mut path: Vec<String> = Vec::new();
        root.walk_scoped(&mut path, &mut |full, tc, span| {
            idx.declare(&full);
            idx.add_toggles(&full, tc);
            if span > max_span_units {
                max_span_units = span;
            }
        });

        let dur_units = duration_units.filter(|d| *d > 0.0).unwrap_or(max_span_units);
        Ok(Saif { idx, sim_time_s: dur_units * timescale_s })
    }
}

fn parse_timescale(v: &[String]) -> f64 {
    // "(TIMESCALE 1 ns)" -> ["1","ns"]; "(TIMESCALE 1ns)" -> ["1ns"]
    let s = v.join("").to_lowercase();
    let units = [("fs", 1e-15), ("ps", 1e-12), ("ns", 1e-9), ("us", 1e-6), ("ms", 1e-3), ("s", 1.0)];
    for (suf, scale) in units {
        if let Some(num) = s.strip_suffix(suf) {
            let n: f64 = num.trim().parse().unwrap_or(1.0);
            return n * scale;
        }
    }
    1.0e-9
}

// ---- tiny s-expression tree (SAIF is parenthesised) ----------------------------

enum Node {
    Atom(String),
    List(Vec<Node>),
}

impl Node {
    /// Parse the whole text into one synthetic root list of top-level nodes.
    fn parse(text: &str) -> Result<Node, SaifError> {
        let toks = tokenize(text);
        let mut pos = 0;
        let mut roots = Vec::new();
        while pos < toks.len() {
            roots.push(parse_node(&toks, &mut pos)?);
        }
        Ok(Node::List(roots))
    }

    fn head(&self) -> Option<&str> {
        match self {
            Node::List(items) => match items.first() {
                Some(Node::Atom(a)) => Some(a.as_str()),
                _ => None,
            },
            _ => None,
        }
    }

    /// First (pre-order) list whose head matches `key`; returns its trailing atoms.
    fn find_kv(&self, key: &str) -> Option<Vec<String>> {
        if let Node::List(items) = self {
            if self.head().map(|h| h.eq_ignore_ascii_case(key)).unwrap_or(false) {
                let tail = items[1..]
                    .iter()
                    .filter_map(|n| match n {
                        Node::Atom(a) => Some(a.clone()),
                        _ => None,
                    })
                    .collect();
                return Some(tail);
            }
            for it in items {
                if let Some(v) = it.find_kv(key) {
                    return Some(v);
                }
            }
        }
        None
    }

    /// Visit every net entry under any `NET` group: `f(leaf_name, toggle_count, span_units)`.
    /// Walk the tree tracking the `INSTANCE` path, emitting each net as
    /// `f(full_path, tc, span)` where `full_path` = `inst1.inst2...leaf`. `(INSTANCE
    /// <name> …)` pushes a scope; `(NET …)` emits its nets under the current path.
    fn walk_scoped<F: FnMut(String, u64, f64)>(&self, path: &mut Vec<String>, f: &mut F) {
        let Node::List(items) = self else { return };
        match self.head() {
            Some(h) if h.eq_ignore_ascii_case("INSTANCE") => {
                let name = match items.get(1) {
                    Some(Node::Atom(a)) => Some(a.clone()),
                    _ => None,
                };
                if let Some(n) = &name {
                    path.push(n.clone());
                }
                for child in &items[2..] {
                    child.walk_scoped(path, f);
                }
                if name.is_some() {
                    path.pop();
                }
            }
            Some(h) if h.eq_ignore_ascii_case("NET") => {
                for child in &items[1..] {
                    if let (Node::List(_), Some(leaf)) = (child, child.head()) {
                        let tc = child
                            .find_kv("TC")
                            .and_then(|v| v.first().and_then(|s| s.parse::<f64>().ok()))
                            .unwrap_or(0.0) as u64;
                        let span: f64 = ["T0", "T1", "TX"]
                            .iter()
                            .map(|k| {
                                child
                                    .find_kv(k)
                                    .and_then(|v| v.first().and_then(|s| s.parse::<f64>().ok()))
                                    .unwrap_or(0.0)
                            })
                            .sum();
                        let full = if path.is_empty() {
                            leaf.to_string()
                        } else {
                            format!("{}.{}", path.join("."), leaf)
                        };
                        f(full, tc, span);
                    }
                }
            }
            _ => {
                for it in items {
                    it.walk_scoped(path, f);
                }
            }
        }
    }
}

enum Tok {
    Open,
    Close,
    Atom(String),
}

fn tokenize(text: &str) -> Vec<Tok> {
    let mut toks = Vec::new();
    let mut chars = text.chars().peekable();
    while let Some(&c) = chars.peek() {
        match c {
            '(' => {
                toks.push(Tok::Open);
                chars.next();
            }
            ')' => {
                toks.push(Tok::Close);
                chars.next();
            }
            '"' => {
                chars.next(); // opening quote
                let mut s = String::new();
                for ch in chars.by_ref() {
                    if ch == '"' {
                        break;
                    }
                    s.push(ch);
                }
                toks.push(Tok::Atom(s));
            }
            ws if ws.is_whitespace() => {
                chars.next();
            }
            _ => {
                let mut s = String::new();
                while let Some(&ch) = chars.peek() {
                    if ch == '(' || ch == ')' || ch == '"' || ch.is_whitespace() {
                        break;
                    }
                    s.push(ch);
                    chars.next();
                }
                toks.push(Tok::Atom(s));
            }
        }
    }
    toks
}

fn parse_node(toks: &[Tok], pos: &mut usize) -> Result<Node, SaifError> {
    match toks.get(*pos) {
        Some(Tok::Open) => {
            *pos += 1;
            let mut items = Vec::new();
            loop {
                match toks.get(*pos) {
                    Some(Tok::Close) => {
                        *pos += 1;
                        return Ok(Node::List(items));
                    }
                    None => return Err(SaifError("unbalanced '(' — missing ')'".into())),
                    _ => items.push(parse_node(toks, pos)?),
                }
            }
        }
        Some(Tok::Atom(a)) => {
            let n = Node::Atom(a.clone());
            *pos += 1;
            Ok(n)
        }
        Some(Tok::Close) => Err(SaifError("unexpected ')'".into())),
        None => Err(SaifError("unexpected end of input".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAIF: &str = r#"
(SAIFILE
  (SAIFVERSION "2.0")
  (DIRECTION "backward")
  (DIVIDER / )
  (TIMESCALE 1 ns)
  (DURATION 50.0)
  (INSTANCE top
    (NET
      (clk (T0 25.0) (T1 25.0) (TX 0.0) (TC 10) (IG 0))
      (n1  (T0 30.0) (T1 20.0) (TX 0.0) (TC 4)  (IG 0))
      (q   (T0 35.0) (T1 15.0) (TX 0.0) (TC 2)  (IG 0))
    )
    (INSTANCE sub
      (NET
        (deep (T0 40.0) (T1 10.0) (TX 0.0) (TC 6) (IG 0))
      )
    )
  )
)
"#;

    // clk collides: present in both `top` and the nested `top.dut`.
    const SAIF_HIER: &str = r#"
(SAIFILE
  (TIMESCALE 1 ns) (DURATION 50.0)
  (INSTANCE top
    (NET (clk (T0 25.0) (T1 25.0) (TX 0.0) (TC 10)))
    (INSTANCE dut
      (NET (clk (T0 25.0) (T1 25.0) (TX 0.0) (TC 4)) (q (T0 40.0) (T1 10.0) (TX 0.0) (TC 6)))
    )
  )
)
"#;

    #[test]
    fn parses_counts_and_rates() {
        let s = Saif::parse(SAIF).unwrap();
        assert!((s.sim_time_s - 50.0e-9).abs() < 1e-18);
        assert_eq!(*s.idx.toggles.get("top.clk").unwrap(), 10);
        assert_eq!(*s.idx.toggles.get("top.n1").unwrap(), 4);
        assert_eq!(*s.idx.toggles.get("top.q").unwrap(), 2);
        // nested INSTANCE/NET nets keep their full path
        assert_eq!(*s.idx.toggles.get("top.sub.deep").unwrap(), 6);
        // TC / DURATION: n1 = 4 / 50ns = 8e7 (leaf resolves, unique)
        assert!((s.toggle_rate("n1") - 8.0e7).abs() < 1.0);
        assert!((s.toggle_rate("q") - 4.0e7).abs() < 1.0);
        assert_eq!(s.toggle_rate("absent"), 0.0);
    }

    #[test]
    fn falls_back_to_span_without_duration() {
        let no_dur = "(SAIFILE (TIMESCALE 1 ns) (NET (a (T0 60.0) (T1 40.0) (TX 0.0) (TC 5))))";
        let s = Saif::parse(no_dur).unwrap();
        // span = 100 ns -> rate = 5 / 100ns = 5e7
        assert!((s.sim_time_s - 100.0e-9).abs() < 1e-18);
        assert!((s.toggle_rate("a") - 5.0e7).abs() < 1.0);
    }

    #[test]
    fn scope_aware_resolution() {
        let s = Saif::parse(SAIF_HIER).unwrap();
        assert_eq!(*s.idx.toggles.get("top.clk").unwrap(), 10);
        assert_eq!(*s.idx.toggles.get("top.dut.clk").unwrap(), 4);
        assert_eq!(s.collisions(), 1);
        // bare `clk` collides -> unresolved, no silent last-write-wins
        assert_eq!(s.toggle_rate("clk"), 0.0);
        // scope: dut -> top.dut.clk = 4 / 50ns
        let scoped = Saif::parse(SAIF_HIER).unwrap().with_scope(Some("dut".to_string()));
        assert!((scoped.toggle_rate("clk") - 8.0e7).abs() < 1.0);
        // `q` is unique -> resolves regardless of scope
        assert!((s.toggle_rate("q") - 1.2e8).abs() < 1.0); // 6 / 50ns
    }
}
