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
    pub toggles: HashMap<String, u64>, // full hierarchical path -> transition count
    pub by_leaf: HashMap<String, Vec<String>>, // leaf name -> declared full paths
    pub scope: Option<String>,         // design instance path (job `scope:`)
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
        let path = self.resolve_path(net)?;
        Some(self.toggles.get(path).copied().unwrap_or(0))
    }

    /// Resolve a netlist `net` to the **full dumped path** of a unique signal, without
    /// reading a count. Same rule as [`resolve`](NetIndex::resolve) — ambiguous is
    /// unresolved.
    ///
    /// Separate from `resolve` because a sweep holds **one** declaration index and **many**
    /// per-window count maps: the name→path join is identical in every window, so it is done
    /// once here and the counts are looked up per window. Cloning the index per window
    /// instead would duplicate every declared path for every window measured.
    pub fn resolve_path(&self, net: &str) -> Option<&str> {
        let leaf = leaf_of(net);
        let target = match &self.scope {
            Some(s) => format!("{s}.{net}"),
            None => net.to_string(),
        };
        let dot_target = format!(".{target}");
        let cands = self.by_leaf.get(leaf)?;
        let mut hits = cands
            .iter()
            .filter(|p| **p == target || p.ends_with(&dot_target));
        let first = hits.next()?;
        if hits.next().is_some() {
            None // ambiguous — refuse to guess
        } else {
            Some(first.as_str())
        }
    }

    /// Number of leaf names declared under more than one scope. When this is > 0 and
    /// no `scope:` is set, bare-leaf lookups for those names are ambiguous.
    pub fn collisions(&self) -> usize {
        self.by_leaf
            .values()
            .filter(|paths| paths.len() > 1)
            .count()
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

/// Split an `inst/pin` object reference into `(instance, pin)` — at the **last** `/`.
///
/// This is how an SDC names a pin (`create_generated_clock … [get_pins u_div/Q]`,
/// `set_false_path -to core/r2/D`) and how a timing graph labels one. The split is at the last
/// separator because **a flattened netlist keeps the hierarchy in the instance name**:
/// synthesis writes `core/u_div` as a single escaped identifier, so `core/u_div/Q` is pin `Q`
/// of instance `core/u_div`, not pin `u_div/Q` of instance `core`.
///
/// Correct under either hierarchy convention: with `/` as the divider (OpenSTA/OpenROAD) the
/// last one is the pin boundary, and with `.` (Yosys `flatten`) there is only one `/` and both
/// readings agree.
///
/// `None` when there is no separator — a primary port, which is its own object.
///
/// # Why this lives here
///
/// Splitting at the *first* separator is the same defect this module exists to name, in a
/// different format: the instance resolves to a name no node carries, so the clock never
/// attaches, the clock group never applies, the false path never matches — and **nothing
/// errors**. The report reads exactly like one from an SDC that never mentioned the object.
/// Measured instance: it was wrong at four sites in the timer while the constraint linter beside
/// it was right, so the two halves of one tool disagreed about what `core/u_div/Q` meant.
pub fn split_inst_pin(obj: &str) -> Option<(&str, &str)> {
    obj.rsplit_once('/')
}

/// The instance part of an `inst/pin` object reference; the whole string when it names no pin
/// (a primary port is its own instance). See [`split_inst_pin`].
pub fn instance_of(obj: &str) -> &str {
    split_inst_pin(obj).map(|(i, _)| i).unwrap_or(obj)
}

/// Split a name into its base and bit index: `data_reg[3]` → `("data_reg", Some(3))`.
///
/// A bus does not survive synthesis as a bus — it survives as one net or one flop per bit,
/// named `base[i]`. That suffix is the only structural evidence left that those bits belong
/// together, which is what any check reasoning about a *group* of signals (a multi-bit domain
/// crossing, a bussed exception, a per-bit report rolled up) has to key on.
///
/// `(name, None)` when the name does not end in a single-bit select — including a **range**
/// (`data[3:0]`), which names a whole bus rather than one bit of it, per the rule this module
/// already follows for VCD `$var` declarations. Also `None` for a name that is *only* a
/// bit-select (`[3]`), whose base would be empty and would group unrelated signals together.
pub fn split_bit_select(name: &str) -> (&str, Option<i64>) {
    let Some(open) = name.rfind('[') else {
        return (name, None);
    };
    if !name.ends_with(']') || open == 0 {
        return (name, None);
    }
    match name[open + 1..name.len() - 1].parse::<i64>() {
        Ok(bit) => (&name[..open], Some(bit)),
        Err(_) => (name, None), // a range, or not an index at all
    }
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
    fn an_inst_pin_reference_splits_at_the_pin_not_the_hierarchy() {
        // THE ONE THAT KEEPS BEING GOT WRONG. Everything after the LAST separator is the pin;
        // everything before it is the instance, hierarchy included.
        assert_eq!(split_inst_pin("u_div/Q"), Some(("u_div", "Q")));
        assert_eq!(split_inst_pin("core/u_div/Q"), Some(("core/u_div", "Q")));
        assert_eq!(instance_of("core/u_div/Q"), "core/u_div");
        // A port names no pin, and is its own instance — not an empty string, which would
        // match every exception whose endpoint set is empty.
        assert_eq!(split_inst_pin("clk"), None);
        assert_eq!(instance_of("clk"), "clk");
        // Yosys-style hierarchy (`.` inside the instance name) has one `/`, and both readings
        // of it agree — the rule is convention-independent.
        assert_eq!(split_inst_pin("core.u_div/Q"), Some(("core.u_div", "Q")));
    }

    #[test]
    fn a_bit_select_separates_from_its_base_but_a_range_does_not() {
        assert_eq!(split_bit_select("data_reg[3]"), ("data_reg", Some(3)));
        assert_eq!(
            split_bit_select("core/data_reg[12]"),
            ("core/data_reg", Some(12))
        );
        // A RANGE names the whole bus, not one bit of it — the same distinction this module
        // already draws for a VCD `$var`.
        assert_eq!(split_bit_select("data[3:0]"), ("data[3:0]", None));
        // Nothing to split.
        assert_eq!(split_bit_select("clk"), ("clk", None));
        assert_eq!(split_bit_select("data[]"), ("data[]", None));
        assert_eq!(split_bit_select("data[x]"), ("data[x]", None));
        // A name that is ONLY a bit-select has no base; grouping on an empty one would put
        // every such signal in the same bus.
        assert_eq!(split_bit_select("[3]"), ("[3]", None));
        // The last dimension wins, which is the one a per-bit flop carries.
        assert_eq!(split_bit_select("mem[0][1]"), ("mem[0]", Some(1)));
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
