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

use std::collections::HashMap;

/// Full-path → toggle count, a leaf → full-paths index, and an optional design scope.
/// Shared storage/resolution for both the VCD and SAIF readers.
#[derive(Debug, Clone, Default)]
pub struct NetIndex {
    pub toggles: HashMap<String, u64>,         // full hierarchical path -> transition count
    pub by_leaf: HashMap<String, Vec<String>>, // leaf name -> declared full paths
    pub scope: Option<String>,                 // design instance path (job `scope:`)
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
