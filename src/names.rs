//! Scope-aware net resolution shared by the VCD and SAIF activity readers.
//!
//! Both readers key toggle counts by a signal's **full hierarchical path** (e.g.
//! `counter_tb.dut.clk_in`) and index each leaf name to the paths that carry it. A
//! netlist net — a flat leaf like `clk_in`, optionally qualified by the job's
//! `scope:` (e.g. `dut` or `counter_tb.dut`) — resolves to a single dumped signal by
//! matching `scope.net` as a `.`-boundary suffix of a full path.
//!
//! The correctness rule: a **unique** match resolves; an **ambiguous** match (the
//! same leaf in more than one scope, e.g. a testbench net and the DUT net of the same
//! name) is left **unresolved** so the caller falls back to the vectorless factor —
//! never a silent last-write-wins pick. Set `scope:` to disambiguate.
//!
//! # One spelling for a net, across every reader
//!
//! A design is described by five or six files written by different tools, and an engine joins
//! them **by name**. Every reader here therefore normalises a net name to the SAME form: the
//! characters of the name, with each format's own escaping removed.
//!
//! | file | on disk | what the reader must yield |
//! | --- | --- | --- |
//! | gate-level Verilog | `wire \u_a.q[0] ;` | `u_a.q[0]` — the `\` introduces an escaped identifier and is no more part of the name than the terminating space (LRM: `\foo ` IS `foo`) |
//! | DEF | `- u_a\.q\[0\]` | `u_a.q[0]` — `\` escapes the character that follows |
//! | SPEF | `*7 u_a.q[0]` or `*7 u_a\.q\[0\]` | `u_a.q[0]` — both spellings occur and mean the same characters |
//! | VCD (ModelSim) | `$var wire 1 ) r [2]` | `r[2]` — a bit-select is part of the name; a `[3:0]` RANGE is not |
//!
//! **This is the defect class that keeps recurring, and it is silent every time.** A name that
//! does not join is not an error: the net simply has no parasitics, or no activity, or no
//! timing, and the answer comes out optimistic with nothing to say it was incomplete. Measured
//! instances, all on real designs: 10-20 % of nets when DEF names were left escaped; 767 of
//! 14238 nets and 4527 coupling references when the netlist kept its backslash — which removed
//! that crosstalk from the timing analysis without a word; and every bit of every bus when a
//! ModelSim dump's bit-selects were dropped.
//!
//! The rule follows from that: **normalise on the way in, and write back what the source
//! spelled.** A reader that leaves a name in its own format's escaping has not finished, and a
//! writer that re-derives escaping from the characters cannot — the same characters are spelled
//! two legal ways and only the source knows which. `tests/composition.rs` asserts the join
//! directly, which is the only place this class of defect is visible.

use std::collections::HashMap;

/// Full-path → toggle count, a leaf → full-paths index, and an optional design scope.
/// Shared storage/resolution for both the VCD and SAIF readers.
#[derive(Clone, Default)]
pub struct NetIndex {
    pub toggles: HashMap<String, u64>,         // full hierarchical path -> transition count
    pub by_leaf: HashMap<String, Vec<String>>, // leaf name -> declared full paths
    pub scope: Option<String>,                 // design instance path (job `scope:`)
}

/// Rendered in **sorted key order**, not the maps' own.
///
/// `HashMap` is the right storage here and `BTreeMap` is not: `add_toggles` runs once per value
/// change in a VCD, so this is the hottest map in the crate and the lookup wants to stay O(1).
/// But a derived `Debug` walks the map, and Rust seeds its hasher per process — so the derive
/// printed these in a different order on every run. `Debug` output is output: it lands in
/// diagnostics, in test failure messages, and in anything comparing two runs. An unstable
/// rendering makes a byte-comparison between processes meaningless, which is exactly what
/// `tests/determinism.rs` needs to be able to do.
///
/// So: fast storage, ordered rendering. Sorting costs nothing at runtime because nothing formats
/// a `NetIndex` on a hot path.
impl std::fmt::Debug for NetIndex {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let toggles: std::collections::BTreeMap<_, _> = self.toggles.iter().collect();
        let by_leaf: std::collections::BTreeMap<_, _> = self.by_leaf.iter().collect();
        f.debug_struct("NetIndex")
            .field("toggles", &toggles)
            .field("by_leaf", &by_leaf)
            .field("scope", &self.scope)
            .finish()
    }
}

impl NetIndex {
    /// Record a declared signal at `full_path` (indexes its leaf for resolution and
    /// collision detection). Idempotent per path.
    pub fn declare(&mut self, full_path: &str) {
        let leaf = leaf_of(full_path).to_string();
        let paths = self.by_leaf.entry(leaf).or_default();
        if !paths.iter().any(|p| p == full_path) {
            paths.push(full_path.to_string());
        }
    }

    /// Add `n` transitions to `full_path`.
    pub fn add_toggles(&mut self, full_path: &str, n: u64) {
        *self.toggles.entry(full_path.to_string()).or_insert(0) += n;
    }

    /// Resolve a netlist `net` to the toggle count of a *unique* dumped signal.
    /// `None` = unresolved (absent) or ambiguous (leaf in multiple scopes) → the
    /// caller should fall back to the vectorless factor.
    pub fn resolve(&self, net: &str) -> Option<u64> {
        let leaf = leaf_of(net);
        let target = match &self.scope {
            Some(s) => format!("{s}.{net}"),
            None => net.to_string(),
        };
        let dot_target = format!(".{target}");
        let cands = self.by_leaf.get(leaf)?;
        let mut hits = cands.iter().filter(|p| **p == target || p.ends_with(&dot_target));
        let first = hits.next()?;
        if hits.next().is_some() {
            None // ambiguous — refuse to guess
        } else {
            Some(self.toggles.get(first).copied().unwrap_or(0))
        }
    }

    /// Number of leaf names declared under more than one scope. When this is > 0 and
    /// no `scope:` is set, bare-leaf lookups for those names are ambiguous.
    pub fn collisions(&self) -> usize {
        self.by_leaf.values().filter(|paths| paths.len() > 1).count()
    }

    /// Leaf names declared under more than one scope (the ambiguous ones), sorted.
    pub fn colliding_leaves(&self) -> Vec<String> {
        let mut v: Vec<String> = self
            .by_leaf
            .iter()
            .filter(|(_, paths)| paths.len() > 1)
            .map(|(leaf, _)| leaf.clone())
            .collect();
        v.sort();
        v
    }
}

/// The last `.`-separated component of a hierarchical path.
pub fn leaf_of(path: &str) -> &str {
    path.rsplit('.').next().unwrap_or(path)
}

/// Strip SPEF name escaping: `u_a\.q\[0\]` -> `u_a.q[0]`.
///
/// Names are matched against the design's own net names, and a backslash that survives the read
/// makes every hierarchical or bussed name miss — silently, as a net with no parasitics.
pub(crate) fn unescape(s: &str) -> String {
    if !s.contains('\\') {
        return s.to_string();
    }
    let mut out = String::with_capacity(s.len());
    let mut esc = false;
    for c in s.chars() {
        if esc {
            out.push(c);
            esc = false;
        } else if c == '\\' {
            esc = true;
        } else {
            out.push(c);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idx() -> NetIndex {
        // clk_in appears in the testbench AND the DUT; q only in the DUT.
        let mut i = NetIndex::default();
        i.declare("counter_tb.clk_in");
        i.add_toggles("counter_tb.clk_in", 79);
        i.declare("counter_tb.dut.clk_in");
        i.add_toggles("counter_tb.dut.clk_in", 40);
        i.declare("counter_tb.dut.q");
        i.add_toggles("counter_tb.dut.q", 13);
        i
    }

    #[test]
    fn ambiguous_leaf_is_unresolved_without_scope() {
        let i = idx();
        assert_eq!(i.resolve("clk_in"), None); // collides tb vs dut -> refuse
        assert_eq!(i.resolve("q"), Some(13)); // unique -> resolves
        assert_eq!(i.collisions(), 1);
    }

    #[test]
    fn scope_disambiguates() {
        let mut i = idx();
        i.scope = Some("dut".to_string());
        assert_eq!(i.resolve("clk_in"), Some(40)); // dut.clk_in via ".dut.clk_in" suffix
        assert_eq!(i.resolve("q"), Some(13));
    }

    #[test]
    fn full_scope_path_exact() {
        let mut i = idx();
        i.scope = Some("counter_tb.dut".to_string());
        assert_eq!(i.resolve("clk_in"), Some(40));
    }

    #[test]
    fn single_scope_leaf_resolves() {
        let mut i = NetIndex::default();
        i.declare("counter.clk_in");
        i.add_toggles("counter.clk_in", 5);
        assert_eq!(i.resolve("clk_in"), Some(5)); // backward-compatible single-scope
        assert_eq!(i.collisions(), 0);
    }
}
